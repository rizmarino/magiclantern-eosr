//
// Offline (faster-than-real-time) OPL driver for Chocolate Doom's opl library.
//
// This is a port of choc/opl_sdl.c with SDL/SDL_mixer removed: instead of an
// audio callback pulling samples, the renderer calls OfflineOPL_Render(),
// which clocks the Nuked OPL3 core and the OPL callback queue in sample time
// exactly as OPL_Mix_Callback()/AdvanceTime() do in opl_sdl.c.
//
// It is registered under the symbol name opl_sdl_driver so that the
// unmodified opl.c driver table picks it up.
//
// GPL-2.0-or-later, like the Chocolate Doom code it is derived from.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "opl3.h"
#include "opl.h"
#include "opl_internal.h"
#include "opl_queue.h"
#include "SDL.h"

#include "opl_offline.h"

typedef struct
{
    unsigned int rate;
    unsigned int enabled;
    unsigned int value;
    uint64_t expire_time;
} opl_timer_t;

static opl_callback_queue_t *callback_queue;
static uint64_t current_time;      // us since driver init
static uint64_t total_samples;     // samples generated since driver init
static int opl_offline_paused;
static uint64_t pause_offset;
static opl3_chip opl_chip;
static int opl_opl3mode;
static int register_num = 0;
static unsigned int mixing_freq;

static opl_timer_t timer1 = { 12500, 0, 0, 0 };
static opl_timer_t timer2 = { 3125, 0, 0, 0 };

// Key-on tracking (for "last note-off" bookkeeping, observation only).
static uint8_t keyon_state[0x200];
static uint64_t last_keyoff_sample;
static int any_keyoff;

// Capture buffer (stereo interleaved int16 from the emulator).
static int16_t *cap_buf = NULL;
static size_t cap_len = 0;      // in frames
static size_t cap_alloc = 0;
static int capture_enabled = 0;
static int16_t scratch[2 * 4096];

// --- tiny SDL shim (single threaded) ---------------------------------------

static int dummy_obj;
SDL_mutex *SDL_CreateMutex(void) { return (SDL_mutex *) &dummy_obj; }
void SDL_DestroyMutex(SDL_mutex *m) { (void) m; }
int SDL_LockMutex(SDL_mutex *m) { (void) m; return 0; }
int SDL_UnlockMutex(SDL_mutex *m) { (void) m; return 0; }
SDL_cond *SDL_CreateCond(void) { return (SDL_cond *) &dummy_obj; }
void SDL_DestroyCond(SDL_cond *c) { (void) c; }
int SDL_CondSignal(SDL_cond *c) { (void) c; return 0; }

// OPL_Delay() waits on a condition that is signalled by a queued callback.
// Offline, "waiting" means running the emulator forward until the next
// callback in the queue has been invoked.
int SDL_CondWait(SDL_cond *c, SDL_mutex *m)
{
    (void) c; (void) m;
    if (OPL_Queue_IsEmpty(callback_queue))
    {
        fprintf(stderr, "opl_offline: CondWait with empty queue\n");
        exit(1);
    }
    OfflineOPL_RunUntilNextCallback();
    return 0;
}

// ---------------------------------------------------------------------------

static void UpdateTime(void)
{
    current_time = (total_samples * OPL_SECOND) / mixing_freq;
}

// Invoke callbacks that are due (equivalent of AdvanceTime() in opl_sdl.c,
// except time is derived from the exact sample count, so no per-chunk
// rounding drift accumulates).
static void InvokeDueCallbacks(void)
{
    opl_callback_t callback;
    void *callback_data;

    while (!OPL_Queue_IsEmpty(callback_queue)
        && current_time >= OPL_Queue_Peek(callback_queue) + pause_offset)
    {
        if (!OPL_Queue_Pop(callback_queue, &callback, &callback_data))
        {
            break;
        }
        callback(callback_data);
    }
}

static void AppendCapture(const int16_t *buf, size_t frames)
{
    if (cap_len + frames > cap_alloc)
    {
        size_t n = cap_alloc ? cap_alloc : (size_t) mixing_freq * 16;
        while (n < cap_len + frames) n *= 2;
        cap_buf = realloc(cap_buf, n * 2 * sizeof(int16_t));
        if (cap_buf == NULL) { fprintf(stderr, "out of memory\n"); exit(1); }
        cap_alloc = n;
    }
    memcpy(cap_buf + 2 * cap_len, buf, frames * 2 * sizeof(int16_t));
    cap_len += frames;
}

static void GenerateSamples(uint64_t nsamples)
{
    uint64_t pausedus0 = current_time;

    while (nsamples > 0)
    {
        unsigned int n = nsamples > 4096 ? 4096 : (unsigned int) nsamples;
        OPL3_GenerateStream(&opl_chip, scratch, n);
        if (capture_enabled)
        {
            AppendCapture(scratch, n);
        }
        total_samples += n;
        nsamples -= n;
    }
    UpdateTime();
    if (opl_offline_paused)
    {
        pause_offset += current_time - pausedus0;
    }
}

// Number of samples until the next callback is due (ceil), like
// OPL_Mix_Callback() in opl_sdl.c.
static uint64_t SamplesToNextCallback(void)
{
    uint64_t next = OPL_Queue_Peek(callback_queue) + pause_offset;
    uint64_t ns;
    if (next <= current_time)
    {
        return 0;
    }
    ns = (next - current_time) * mixing_freq;
    ns = (ns + OPL_SECOND - 1) / OPL_SECOND;
    return ns;
}

void OfflineOPL_RunUntilNextCallback(void)
{
    // Advance until the head of the queue has been invoked once.
    uint64_t ns = SamplesToNextCallback();
    GenerateSamples(ns);
    // Rounding: make sure we really reached it.
    while (!OPL_Queue_IsEmpty(callback_queue)
        && current_time < OPL_Queue_Peek(callback_queue) + pause_offset)
    {
        GenerateSamples(1);
    }
    InvokeDueCallbacks();
}

int OfflineOPL_QueueEmpty(void)
{
    return OPL_Queue_IsEmpty(callback_queue);
}

void OfflineOPL_RunUntilQueueEmpty(uint64_t max_samples)
{
    uint64_t start = total_samples;
    InvokeDueCallbacks();
    while (!OPL_Queue_IsEmpty(callback_queue))
    {
        if (total_samples - start >= max_samples)
        {
            fprintf(stderr, "opl_offline: song did not end within limit\n");
            return;
        }
        OfflineOPL_RunUntilNextCallback();
    }
}

void OfflineOPL_RunSamples(uint64_t nsamples)
{
    // Plain time advance (still services callbacks, if any).
    while (nsamples > 0)
    {
        uint64_t n = nsamples;
        if (!OPL_Queue_IsEmpty(callback_queue))
        {
            uint64_t nc = SamplesToNextCallback();
            if (nc < n) n = nc;
        }
        GenerateSamples(n);
        nsamples -= n;
        InvokeDueCallbacks();
    }
}

void OfflineOPL_SetCapture(int enabled) { capture_enabled = enabled; }
uint64_t OfflineOPL_TotalSamples(void) { return total_samples; }
unsigned int OfflineOPL_Rate(void) { return mixing_freq; }

const int16_t *OfflineOPL_Captured(size_t *frames)
{
    *frames = cap_len;
    return cap_buf;
}

void OfflineOPL_ClearCapture(void) { cap_len = 0; }

int OfflineOPL_LastKeyOff(uint64_t *sample)
{
    *sample = last_keyoff_sample;
    return any_keyoff;
}

void OfflineOPL_ResetKeyOffTracking(void)
{
    any_keyoff = 0;
    last_keyoff_sample = 0;
}

int OfflineOPL_VoicesKeyedOn(void)
{
    int i, n = 0;
    for (i = 0; i < 9; ++i)
    {
        n += keyon_state[0xb0 + i] != 0;
        n += keyon_state[0x1b0 + i] != 0;
    }
    return n;
}

// --- driver ----------------------------------------------------------------

static void OPL_Offline_Shutdown(void)
{
    if (callback_queue != NULL)
    {
        OPL_Queue_Destroy(callback_queue);
        callback_queue = NULL;
    }
}

static int OPL_Offline_Init(unsigned int port_base)
{
    (void) port_base;

    opl_offline_paused = 0;
    pause_offset = 0;
    callback_queue = OPL_Queue_Create();
    current_time = 0;
    total_samples = 0;
    mixing_freq = opl_sample_rate;
    memset(keyon_state, 0, sizeof(keyon_state));
    any_keyoff = 0;
    last_keyoff_sample = 0;
    timer1.enabled = timer2.enabled = 0;

    OPL3_Reset(&opl_chip, mixing_freq);
    opl_opl3mode = 0;
    return 1;
}

static unsigned int OPL_Offline_PortRead(opl_port_t port)
{
    unsigned int result = 0;

    if (port == OPL_REGISTER_PORT_OPL3)
    {
        return 0xff;
    }

    if (timer1.enabled && current_time > timer1.expire_time)
    {
        result |= 0x80;
        result |= 0x40;
    }

    if (timer2.enabled && current_time > timer2.expire_time)
    {
        result |= 0x80;
        result |= 0x20;
    }

    return result;
}

static void OPLTimer_CalculateEndTime(opl_timer_t *timer)
{
    int tics;

    if (timer->enabled)
    {
        tics = 0x100 - timer->value;
        timer->expire_time = current_time
                           + ((uint64_t) tics * OPL_SECOND) / timer->rate;
    }
}

static void WriteRegister(unsigned int reg_num, unsigned int value)
{
    // Observe key-on/off transitions (0xB0-0xB8, bit 5).
    if ((reg_num & 0xff) >= 0xb0 && (reg_num & 0xff) <= 0xb8)
    {
        uint8_t on = (value & 0x20) != 0;
        if (keyon_state[reg_num] && !on)
        {
            last_keyoff_sample = total_samples;
            any_keyoff = 1;
        }
        keyon_state[reg_num] = on;
    }

    switch (reg_num)
    {
        case OPL_REG_TIMER1:
            timer1.value = value;
            OPLTimer_CalculateEndTime(&timer1);
            break;

        case OPL_REG_TIMER2:
            timer2.value = value;
            OPLTimer_CalculateEndTime(&timer2);
            break;

        case OPL_REG_TIMER_CTRL:
            if (value & 0x80)
            {
                timer1.enabled = 0;
                timer2.enabled = 0;
            }
            else
            {
                if ((value & 0x40) == 0)
                {
                    timer1.enabled = (value & 0x01) != 0;
                    OPLTimer_CalculateEndTime(&timer1);
                }

                if ((value & 0x20) == 0)
                {
                    timer2.enabled = (value & 0x02) != 0;
                    OPLTimer_CalculateEndTime(&timer2);
                }
            }
            break;

        case OPL_REG_NEW:
            opl_opl3mode = value & 0x01;
            // fall through

        default:
            OPL3_WriteRegBuffered(&opl_chip, reg_num, value);
            break;
    }
}

static void OPL_Offline_PortWrite(opl_port_t port, unsigned int value)
{
    if (port == OPL_REGISTER_PORT)
    {
        register_num = value;
    }
    else if (port == OPL_REGISTER_PORT_OPL3)
    {
        register_num = value | 0x100;
    }
    else if (port == OPL_DATA_PORT)
    {
        WriteRegister(register_num, value);
    }
}

static void OPL_Offline_SetCallback(uint64_t us, opl_callback_t callback,
                                    void *data)
{
    OPL_Queue_Push(callback_queue, callback, data,
                   current_time - pause_offset + us);
}

static void OPL_Offline_ClearCallbacks(void)
{
    OPL_Queue_Clear(callback_queue);
}

static void OPL_Offline_Lock(void) { }
static void OPL_Offline_Unlock(void) { }

static void OPL_Offline_SetPaused(int paused)
{
    opl_offline_paused = paused;
}

static void OPL_Offline_AdjustCallbacks(float factor)
{
    OPL_Queue_AdjustCallbacks(callback_queue, current_time, factor);
}

opl_driver_t opl_sdl_driver =
{
    "offline",
    OPL_Offline_Init,
    OPL_Offline_Shutdown,
    OPL_Offline_PortRead,
    OPL_Offline_PortWrite,
    OPL_Offline_SetCallback,
    OPL_Offline_ClearCallbacks,
    OPL_Offline_Lock,
    OPL_Offline_Unlock,
    OPL_Offline_SetPaused,
    OPL_Offline_AdjustCallbacks,
};
