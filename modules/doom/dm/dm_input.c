/* dm_input.c - EOS R raw button codes (BGMT_*, see platform/R.180/gui.h and kitor's list there) -> Doom key events.
 * Portable: the camera feeds it from CBR_KEYPRESS_RAW (GUI task), the host harness from a script. One producer,
 * one consumer (the Doom task), lock-free ring. Buttons that only send a press (PLAY, Trash, dials, touch-bar
 * swipes) become pulses: press now, release ~90 ms later, so Doom sees them for 3 tics. */

#include <stdint.h>
#include "dm_input.h"

/* doomkeys.h */
#define DK_RIGHT 0xae
#define DK_LEFT  0xac
#define DK_UP    0xad
#define DK_DOWN  0xaf
#define DK_SL    0xa0
#define DK_SR    0xa1
#define DK_USE   0xa2
#define DK_FIRE  0xa3
#define DK_ESC   27
#define DK_ENTER 13
#define DK_TAB   9

enum { HOLD, RELEASE, PULSE, QUIT };
static const struct { int16_t param; uint8_t key, kind, also_enter; } keymap[] = {
    { 0x31, DK_UP, HOLD, 0 },    { 0x32, DK_UP, RELEASE, 0 },      /* joystick */
    { 0x33, DK_DOWN, HOLD, 0 },  { 0x34, DK_DOWN, RELEASE, 0 },
    { 0x2F, DK_LEFT, HOLD, 0 },  { 0x30, DK_LEFT, RELEASE, 0 },
    { 0x2D, DK_RIGHT, HOLD, 0 }, { 0x2E, DK_RIGHT, RELEASE, 0 },
    { 0x04, DK_FIRE, HOLD, 1 },  { 0x05, DK_FIRE, RELEASE, 0 },    /* Q/SET: fire; Enter pulse for menus */
    { 0x81, DK_FIRE, HOLD, 0 },  { 0x82, DK_FIRE, RELEASE, 0 },    /* AF-ON */
    { 0x1A, DK_USE, HOLD, 0 },   { 0x1B, DK_USE, RELEASE, 0 },     /* M-Fn */
    { 0x0C, DK_USE, PULSE, 0 },                                    /* PLAY: doors, switches */
    { 0x0E, DK_ESC, PULSE, 0 },                                    /* Trash: Doom menu */
    { 0x08, DK_TAB, PULSE, 0 },                                    /* INFO: automap */
    { 0x02, '[', PULSE, 0 },     { 0x03, ']', PULSE, 0 },          /* top dial: previous / next weapon */
    { 0x56, DK_SL, PULSE, 0 },   { 0x55, DK_SR, PULSE, 0 },        /* touch bar swipe: strafe */
    { 0x06, 0, QUIT, 0 },                                          /* MENU: leave Doom */
    /* blocked from Canon while Doom runs, logged, not mapped yet: star, AF-point/zoom, touch-bar taps/lifts,
     * half-shutter */
    { 0x85, 0, PULSE, 0 }, { 0x15, 0, PULSE, 0 }, { 0x16, 0, PULSE, 0 }, { 0x4F, 0, PULSE, 0 },
    { 0x50, 0, PULSE, 0 }, { 0x51, 0, PULSE, 0 }, { 0x52, 0, PULSE, 0 }, { 0x7D, 0, PULSE, 0 },
};

#define KQ 64
static volatile struct { uint8_t key, down, pulse; } kq[KQ];
static volatile int kq_w, kq_r;
#define NPEND 8
static struct { uint8_t key; uint32_t due; } pend[NPEND];
volatile int dm_input_quit;

static void kq_push(int key, int down, int pulse)
{
    int n = (kq_w + 1) % KQ;
    if (n == kq_r) return;
    kq[kq_w].key = key; kq[kq_w].down = down; kq[kq_w].pulse = pulse;
    __asm__ volatile("" ::: "memory");
    kq_w = n;
}

void dm_input_reset(void)
{
    kq_r = kq_w = 0;
    for (int i = 0; i < NPEND; i++) pend[i].key = 0;
    dm_input_quit = 0;
}

int dm_input_raw(int param)
{
    for (unsigned i = 0; i < sizeof keymap / sizeof keymap[0]; i++)
    {
        if (keymap[i].param != param) continue;
        int k = keymap[i].key;
        switch (keymap[i].kind)
        {
            case HOLD:    if (k) kq_push(k, 1, 0); if (keymap[i].also_enter) kq_push(DK_ENTER, 1, 1); break;
            case RELEASE: if (k) kq_push(k, 0, 0); break;
            case PULSE:   if (k) kq_push(k, 1, 1); break;
            case QUIT:    dm_input_quit = 1; break;
        }
        return 1;   /* known button: block it from Canon */
    }
    return 0;
}

int dm_input_getkey(uint32_t now, int *pressed, unsigned char *key)
{
    for (int i = 0; i < NPEND; i++)
    {
        if (pend[i].key && (int32_t)(now - pend[i].due) >= 0)
        {
            *pressed = 0; *key = pend[i].key; pend[i].key = 0;
            return 1;
        }
    }
    if (kq_r == kq_w) return 0;
    __asm__ volatile("" ::: "memory");
    int k = kq[kq_r].key, d = kq[kq_r].down, p = kq[kq_r].pulse;
    kq_r = (kq_r + 1) % KQ;
    if (p)
    {
        int placed = 0;
        for (int i = 0; i < NPEND && !placed; i++)
            if (!pend[i].key) { pend[i].key = k; pend[i].due = now + 90; placed = 1; }
        if (!placed) return dm_input_getkey(now, pressed, key);   /* no slot: drop the pulse rather than stick */
    }
    *pressed = d; *key = k;
    return 1;
}
