/* doom.mo - Doom (doomgeneric) for Magic Lantern on the EOS R (R.180, eosr fork v4.48+).
 *
 * The engine draws 320x200 8-bit frames; dm_blit converts them through Doom's palette straight into ML's RGBA layer
 * (960x540, visible 720x480 at 120,30) and we push the layer with XimrExe, while ML's own conversion is held
 * (r_menu_draw_begin + r_refresh_hold) so it can't paint the menu over the game. Keys come from CBR_KEYPRESS_RAW
 * (per-direction press/unpress codes on the R) and are blocked from Canon while Doom runs.
 * Memory: zone (6 MB) + a small arena come from ML's allocator (shoot memory for large blocks), freed at quit.
 * The engine's globals can't be re-initialised, so one game per power-on.
 *
 * Card: ML/DOOM/DOOM1.WAD (or DOOM.WAD / DOOM2.WAD / FREEDOOM1.WAD ...). Logs: ML/DOOM/DOOMLOG.TXT, ML/DOOM/KEYS.TXT */

#include <module.h>
#include <dryos.h>
#include <bmp.h>
#include <menu.h>
#include <config.h>
#include <property.h>
#include <powersave.h>
#include "dm/include/dm_sys.h"
#include "dm/dm_blit.h"
#include "dm/dm_input.h"

/* ---- engine (declared here so this file doesn't need the engine's libc shim headers) */
extern uint8_t *DG_ScreenBuffer;
extern uint32_t colors[256];        /* struct color {b,g,r,a} == 0xAARRGGBB */
extern int palette_changed;         /* boolean */
extern int key_menu_confirm;
void doomgeneric_Create(int argc, char **argv);
void doomgeneric_Tick(void);
int  dm_snprintf(char *buf, size_t n, const char *fmt, ...);   /* dm_libc: full printf, not Canon's */

#define DK_ENTER 13

/* ---- ML core symbols (eosr fork) */
/* struct MARV as on DIGIC 8 (src/vram.h, CONFIG_DIGIC_678X without the DIGIC X fields); modules build
 * camera-independent, so it isn't visible here */
struct r_marv { uint32_t signature; uint8_t *bitmap_data; uint8_t *opacity_data; uint32_t flags, width, height; void *pmem; };
extern struct r_marv *rgb_vram_info;
extern volatile int r_refresh_hold;
extern int menu_redraw_blocked;
extern uint32_t ml_refresh_display_needed;
extern void r_menu_draw_begin(void);
extern void r_menu_draw_end(void);
extern void rgba_buffer_clear(void);
extern void XimrExe(void *);
extern int display_is_on(void);
extern struct semaphore *winsys_sem;
extern char camera_model[];
extern char firmware_version[];
#define R180_XIMR_CONTEXT ((void *)0xbce810)   /* platform/R.180/consts.h */

#define WIN_X 120
#define WIN_Y 30
#define WIN_W 720
#define WIN_H 480

static CONFIG_INT("doom.scale", doom_scale, DM_SCALE_ASPECT);

static volatile int doom_running;      /* 1 game, 2 display/input test, 3 sound test */
static int doom_played;                /* engine globals are dirty after one run */
static char wad_path[48];

/* ------------------------------------------------------------------ log (RAM, flushed to card) */

static char logbuf[16384];
static int  loglen;

void dm_sys_log(const char *s)
{
    int n = strlen(s);
    if (loglen + n >= (int)sizeof(logbuf) - 1) n = sizeof(logbuf) - 1 - loglen;
    if (n <= 0) return;
    memcpy(logbuf + loglen, s, n);
    loglen += n;
    logbuf[loglen] = 0;
}

static void logf_(const char *fmt, ...)
{
    char tmp[256];
    va_list ap;
    va_start(ap, fmt);
    extern int dm_vsnprintf(char *, size_t, const char *, va_list);
    dm_vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    dm_sys_log(tmp);
}

static void log_flush(void)
{
    if (!loglen) return;
    FIO_CreateDirectory("ML/DOOM");
    FILE *f = FIO_CreateFileOrAppend("ML/DOOM/DOOMLOG.TXT");
    if (f)
    {
        FIO_WriteFile(f, logbuf, loglen);
        FIO_CloseFile(f);
    }
    loglen = 0;
}

/* ------------------------------------------------------------------ dm_sys / dm_io for the camera */

void *dm_sys_alloc(size_t n)
{
    void *p = __mem_malloc(n, MEM_TEMPORARY, "doom", __LINE__);   /* shoot memory: keeps AllocateMemory for Canon */
    logf_("alloc %d -> %x\n", (int)n, (int)p);
    return p;
}

void dm_sys_free(void *p) { if (p) __mem_free(p); }

struct cam_file { FILE *f; long size, pos; };
static uint8_t *bounce;                  /* uncacheable, for FIO_ReadFile */
#define BOUNCE_SIZE (32 * 1024)

void *dm_io_open(const char *path, int mode)
{
    FILE *f;
    uint32_t size = 0;
    if (mode == 0)
    {
        if (FIO_GetFileSize(path, &size) != 0) return NULL;
        f = FIO_OpenFile(path, O_RDONLY);
    }
    else if (mode == 1) f = FIO_CreateFile(path);
    else
    {
        FIO_GetFileSize(path, &size);
        f = FIO_CreateFileOrAppend(path);
    }
    if (!f) return NULL;
    struct cam_file *h = malloc(sizeof(*h));
    if (!h) { FIO_CloseFile(f); return NULL; }
    h->f = f;
    h->size = mode == 1 ? 0 : (long)size;
    h->pos = mode == 2 ? (long)size : 0;
    return h;
}

int dm_io_read(void *hv, void *buf, size_t n)
{
    struct cam_file *h = hv;
    if (!bounce) bounce = fio_malloc(BOUNCE_SIZE);
    if (!bounce) return -1;
    size_t done = 0;
    while (done < n)
    {
        size_t k = n - done < BOUNCE_SIZE ? n - done : BOUNCE_SIZE;
        int r = FIO_ReadFile(h->f, bounce, k);
        if (r <= 0) break;
        memcpy((uint8_t *)buf + done, bounce, r);
        done += r;
        if ((size_t)r < k) break;
    }
    h->pos += done;
    return (int)done;
}

int dm_io_write(void *hv, const void *buf, size_t n)
{
    struct cam_file *h = hv;
    int r = FIO_WriteFile(h->f, buf, n);
    if (r > 0) { h->pos += r; if (h->pos > h->size) h->size = h->pos; }
    return r;
}

int dm_io_seek(void *hv, long pos)
{
    struct cam_file *h = hv;
    if (FIO_SeekSkipFile(h->f, pos, SEEK_SET) < 0) return -1;
    h->pos = pos;
    return 0;
}

long dm_io_size(void *hv) { return ((struct cam_file *)hv)->size; }
void dm_io_close(void *hv) { struct cam_file *h = hv; FIO_CloseFile(h->f); free(h); }
int  dm_io_remove(const char *p) { return FIO_RemoveFile(p); }
int  dm_io_rename(const char *a, const char *b) { return FIO_RenameFile(a, b); }
int  dm_io_mkdir(const char *p) { return FIO_CreateDirectory(p); }

/* ------------------------------------------------------------------ display */

static uint32_t *layer;
static int pitch;
static int frames, blit_us_max, ximr_us_max;
static uint64_t blit_us_sum, ximr_us_sum, t_last_sleep;
static int yields;

static int display_take(void)
{
    if (!rgb_vram_info || !rgb_vram_info->bitmap_data) { logf_("no rgb_vram_info\n"); return 0; }
    layer = (uint32_t *)rgb_vram_info->bitmap_data;
    pitch = rgb_vram_info->width;
    logf_("layer %x %dx%d (cacheable range: %s)\n", (int)layer, (int)rgb_vram_info->width,
          (int)rgb_vram_info->height, ((uint32_t)layer & 0x40000000) ? "no" : "yes");
    if (rgb_vram_info->width < WIN_X + WIN_W || rgb_vram_info->height < WIN_Y + WIN_H) return 0;
    menu_redraw_blocked = 1;
    r_menu_draw_begin();            /* waits for a running ML conversion, then holds new ones */
    r_refresh_hold = 1;
    dm_blit_clear(layer, pitch, WIN_X, WIN_Y, WIN_W, WIN_H);
    return 1;
}

static void display_push(void)
{
    if (!display_is_on() || !winsys_sem) return;     /* panel/EVF switching or display off: skip, like ML's refresh */
    uint64_t t = get_us_clock();
    take_semaphore(winsys_sem, 0);
    XimrExe(R180_XIMR_CONTEXT);
    give_semaphore(winsys_sem);
    uint32_t dt = (uint32_t)(get_us_clock() - t);
    ximr_us_sum += dt;
    if ((int)dt > ximr_us_max) ximr_us_max = dt;
}

static void display_release(void)
{
    r_refresh_hold = 0;
    rgba_buffer_clear();
    menu_redraw_blocked = 0;
    r_menu_draw_end();              /* clears the hold and requests a full refresh */
    ml_refresh_display_needed = 1;
}

/* ------------------------------------------------------------------ keys */

#define KLOG 400
static struct { uint32_t ms; int16_t param; int16_t arg; uint8_t blocked; } klog[KLOG];
static int klog_n, klog_lost;
static volatile int last_param = -1, key_events;

static unsigned int doom_keypress_raw(unsigned int ctx)
{
    if (!doom_running) return 1;
    struct event *ev = (struct event *)ctx;
    if (!ev) return 1;
    int param = ev->param;
    int blocked = dm_input_raw(param);

    if (klog_n < KLOG)
    {
        klog[klog_n].ms = get_ms_clock();
        klog[klog_n].param = param;
        klog[klog_n].arg = ev->arg;
        klog[klog_n].blocked = blocked;
        klog_n++;
    }
    else klog_lost++;
    last_param = param;
    key_events++;
    return blocked ? 0 : 1;
}

static void klog_flush(const char *what)
{
    FIO_CreateDirectory("ML/DOOM");
    FILE *f = FIO_CreateFileOrAppend("ML/DOOM/KEYS.TXT");
    if (!f) return;
    char line[96];
    int n = dm_snprintf(line, sizeof line, "=== %s: %d events (%d not logged)\n", what, klog_n, klog_lost);
    FIO_WriteFile(f, line, n);
    for (int i = 0; i < klog_n; i++)
    {
        n = dm_snprintf(line, sizeof line, "t=%u param=0x%02x arg=%d %s\n", klog[i].ms, klog[i].param & 0xffff,
                        klog[i].arg, klog[i].blocked ? "blocked" : "passed");
        FIO_WriteFile(f, line, n);
    }
    FIO_CloseFile(f);
    klog_n = klog_lost = 0;
}

/* ------------------------------------------------------------------ doomgeneric platform */

void DG_Init(void) { }
void DG_SetWindowTitle(const char *t) { logf_("title: %s\n", t); }
uint32_t DG_GetTicksMs(void) { return get_ms_clock(); }

void DG_SleepMs(uint32_t ms)
{
    msleep(ms < 10 ? 10 : ms);
    t_last_sleep = get_ms_clock();
}

int DG_GetKey(int *pressed, unsigned char *key) { return dm_input_getkey(get_ms_clock(), pressed, key); }

void DG_DrawFrame(void)
{
    uint64_t t = get_us_clock();
    if (palette_changed || frames == 0) { dm_blit_set_palette(NULL, colors); palette_changed = 0; }
    dm_blit_frame(layer, pitch, WIN_X, WIN_Y, WIN_W, WIN_H, DG_ScreenBuffer, doom_scale);
    uint32_t dt = (uint32_t)(get_us_clock() - t);
    blit_us_sum += dt;
    if ((int)dt > blit_us_max) blit_us_max = dt;
    display_push();
    frames++;
    if (frames == 1) log_flush();     /* breadcrumb: init finished, first frame drawn */
    /* if the engine is too slow to ever sleep, give lower-priority tasks a turn now and then */
    if (get_ms_clock() - t_last_sleep > 200) { msleep(10); t_last_sleep = get_ms_clock(); yields++; }
    if (dm_input_quit) dm_exit(1);
}

/* ------------------------------------------------------------------ tasks */

static int doom_main(void)
{
    static char *argv[] = { "doom", "-iwad", wad_path, NULL };
    doomgeneric_Create(3, argv);
    key_menu_confirm = DK_ENTER;      /* "press Y to quit" etc.: SET answers yes */
    for (;;) doomgeneric_Tick();
    return 0;
}

static void run_stats(const char *what, uint32_t t0)
{
    uint32_t ms = get_ms_clock() - t0;
    logf_("%s: %d frames in %d ms = %d.%d fps; blit avg %d us max %d; ximr avg %d us max %d; yields %d; keys %d\n",
          what, frames, (int)ms, frames * 1000 / (ms ? ms : 1), (frames * 10000 / (ms ? ms : 1)) % 10,
          frames ? (int)(blit_us_sum / frames) : 0, blit_us_max, frames ? (int)(ximr_us_sum / frames) : 0,
          ximr_us_max, yields, key_events);
}

static void reset_stats(void)
{
    frames = blit_us_max = ximr_us_max = yields = key_events = 0;
    blit_us_sum = ximr_us_sum = 0;
    dm_input_reset();
    t_last_sleep = get_ms_clock();
}

static void doom_task(void *unused)
{
    reset_stats();
    logf_("\n=== doom start: %s fw %s, scale %d, wad %s, free malloc %d KB, AllocateMemory %d KB\n", camera_model,
          firmware_version, doom_scale, wad_path, GetFreeMemForMalloc() / 1024, GetFreeMemForAllocateMemory() / 1024);
    char rep[512];
    int st = dm_libc_selftest(rep, sizeof rep);
    dm_sys_log(rep);
    log_flush();
    if (st) { NotifyBox(4000, "Doom: libc self-test failed, see DOOMLOG.TXT"); doom_running = 0; return; }

    if (!display_take()) { log_flush(); NotifyBox(4000, "Doom: unexpected display layer"); doom_running = 0; return; }
    powersave_prohibit();
    doom_played = 1;
    uint32_t t0 = get_ms_clock();
    int rc = dm_run_catching_exit(doom_main);
    run_stats("game", t0);
    logf_("exit code %d; arena high water %d KB; big blocks %d (%d KB)\n", rc, (int)(dm_arena_high_water() / 1024),
          dm_big_count(), (int)(dm_big_bytes() / 1024));
    dm_libc_release_all();
    if (bounce) { free(bounce); bounce = NULL; }
    display_release();
    powersave_permit();
    doom_running = 0;
    log_flush();
    klog_flush("game");
    NotifyBox(3000, "Doom: %d frames. Restart camera to play again.", frames);
}

/* Display & input test: no WAD needed. A palette-ramp pattern with a moving bar, through the same blit and push
 * path as the game; the border flashes a new colour on every key event. MENU (or 30 s) ends it. */
static void test_task(void *unused)
{
    reset_stats();
    logf_("\n=== display/input test: %s fw %s, scale %d\n", camera_model, firmware_version, doom_scale);
    char rep[512];
    dm_libc_selftest(rep, sizeof rep);
    dm_sys_log(rep);
    log_flush();
    uint8_t *src = malloc(DM_SRC_W * DM_SRC_H);
    uint32_t *pal = malloc(256 * 4);
    if (!src || !pal || !display_take()) { free(src); free(pal); doom_running = 0; return; }
    powersave_prohibit();
    for (int i = 0; i < 256; i++) pal[i] = (i < 128) ? ((i * 2) << 16) | ((255 - i * 2) << 8) : (((i - 128) * 2) << 0) | 0x404040;
    dm_blit_set_palette(NULL, pal);
    uint32_t t0 = get_ms_clock(), t_idle = t0;
    int seen = key_events;
    while (!dm_input_quit && get_ms_clock() - t_idle < 30000)
    {
        int bar = frames % DM_SRC_W;
        int border = (key_events * 37) & 0xff;
        for (int y = 0; y < DM_SRC_H; y++)
            for (int x = 0; x < DM_SRC_W; x++)
                src[y * DM_SRC_W + x] = (x < 8 || x >= DM_SRC_W - 8 || y < 8 || y >= DM_SRC_H - 8) ? border
                                      : (x >= bar && x < bar + 12) ? 255 : (uint8_t)((x + y) & 0xff);
        uint64_t t = get_us_clock();
        dm_blit_frame(layer, pitch, WIN_X, WIN_Y, WIN_W, WIN_H, src, doom_scale);
        uint32_t dt = (uint32_t)(get_us_clock() - t);
        blit_us_sum += dt;
        if ((int)dt > blit_us_max) blit_us_max = dt;
        display_push();
        frames++;
        if (key_events != seen) { seen = key_events; t_idle = get_ms_clock(); }   /* keep going while keys arrive */
        msleep(10);
    }
    run_stats("test", t0);
    display_release();
    powersave_permit();
    free(src); free(pal);
    doom_running = 0;
    log_flush();
    klog_flush("test");
    NotifyBox(3000, "Doom test: %d frames, %d key events logged", frames, key_events);
}

static int find_wad(void);

/* ------------------------------------------------------------------ sound test (Canon AudioBeep PCM)
 * R.180 AudioBeep (AudioDS::AudioBeep.c), ghidra notes "PCM playback for Doom sound" (snd-decomp1-6.c):
 * FUN_e0294a78(id, spec) copies a 0x40-byte SE spec and posts EV_START_BEEP; with spec[0] = 1 (PCM) it first
 * sets the player state to 0 (ready). The start handler (e029442a) refuses while Canon plays a movie ("Movie
 * Playing", result 0x1b) or when the state isn't 0, else starts the output (FUN_e03021fa, result in +0x24).
 * State block at *(0x9d00): +0x48 state 0 ready / 1 start / 2 stop / 3 complete / 4 continue, +0x24 result.
 * Canon's own wait (FUN_e0294da0) has no timeout, so we poll the state ourselves. Buffers: fio_malloc
 * (uncacheable), as Canon's WAV loader uses alloc_dma_memory. Log: ML/DOOM/SOUND.TXT. */
#define AB_BLOCK      (*(volatile uint32_t *)0x9d00)
#define AB_INT(off)   (*(volatile int *)(AB_BLOCK + (off)))
static int (*const ab_start)(int id, uint32_t *spec) = (void *)0xE0294A79;
static int (*const ab_stop)(int arg) = (void *)0xE0294AEF;
#define AB_RATE_24K   0x18
#define AB_RATE_48K   0x30

static char slog[6144];
static int  sloglen;
static void slogf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    extern int dm_vsnprintf(char *, size_t, const char *, va_list);
    if (sloglen < (int)sizeof(slog) - 200) sloglen += dm_vsnprintf(slog + sloglen, sizeof(slog) - sloglen, fmt, ap);
    va_end(ap);
}
static void slog_flush(void)
{
    if (!sloglen) return;
    FIO_CreateDirectory("ML/DOOM");
    FILE *f = FIO_CreateFileOrAppend("ML/DOOM/SOUND.TXT");
    if (f) { FIO_WriteFile(f, slog, sloglen); FIO_CloseFile(f); }
    sloglen = 0;
}

/* wait until the player is not starting/playing; returns ms waited, or -1 after timeout_ms */
static int ab_wait_idle(int timeout_ms)
{
    uint32_t t0 = get_ms_clock();
    for (;;)
    {
        int st = AB_INT(0x48);
        if (st == 3 || st == 0) return get_ms_clock() - t0;
        if ((int)(get_ms_clock() - t0) > timeout_ms) return -1;
        msleep(2);
    }
}

/* play one buffer; logs start latency and length. wait_done: wait for "complete" (up to len + 1 s) */
static int ab_play(const char *what, void *buf, int bytes, int rate, int ch, int outsel, int vol, int wait_done)
{
    uint32_t spec[16];
    memset(spec, 0, sizeof spec);
    spec[0] = 1;  spec[1] = (uint32_t)buf;  spec[2] = bytes;  spec[5] = vol;  spec[6] = 0;  spec[7] = outsel;
    spec[9] = rate;  spec[10] = 16;  spec[11] = ch;  spec[12] = 0;
    int st0 = AB_INT(0x48);
    if (st0 != 0 && st0 != 3)
    {
        ab_stop(1);
        int w = ab_wait_idle(500);
        slogf("  (was playing: state %d, stop took %d ms)\n", st0, w);
    }
    uint64_t t0 = get_us_clock();
    int rc = ab_start(0, spec);
    int t_start = -1, t_end = -1, st, last = -1, ms_len = bytes * 1000 / (2 * ch * (rate == AB_RATE_48K ? 48000 : 24000));
    int limit = wait_done ? ms_len + 1000 : 300;
    for (;;)
    {
        st = AB_INT(0x48);
        int ms = (int)((get_us_clock() - t0) / 1000);
        if (st != last) { last = st; if (st == 1 && t_start < 0) t_start = ms; if (st == 3 && t_start >= 0) { t_end = ms; break; } }
        if (!wait_done && t_start >= 0) break;
        if (ms > limit) break;
        msleep(1);
    }
    slogf("%s: buf %x %d bytes, rate %x ch %d out %d vol %d: post rc %d, state before %d, started after %d ms, "
          "complete after %d ms (expected %d), state now %d, result %d\n", what, (int)buf, bytes, rate, ch, outsel, vol,
          rc, st0, t_start, t_end, ms_len, AB_INT(0x48), AB_INT(0x24));
    return t_end >= 0 ? t_end : -1;
}

static int16_t *tone(int16_t *p, int n, int rate_hz, int ch, int freq, int amp)
{
    /* triangle wave (no float or sine table needed), 5 ms fade in/out against clicks */
    int period = rate_hz / freq, fade = rate_hz / 200;
    for (int i = 0; i < n; i++)
    {
        int ph = i % period, v = ph < period / 2 ? ph * 4 * amp / period - amp : 3 * amp - ph * 4 * amp / period;
        if (i < fade) v = v * i / fade;
        if (n - i < fade) v = v * (n - i) / fade;
        for (int c = 0; c < ch; c++) *p++ = (int16_t)v;
    }
    return p;
}

/* Doom sfx lump (format 3: u16 3, u16 rate, u32 n, 16 pad, samples u8, 16 pad) -> 24 kHz s16 mono. bytes or 0 */
static int load_sfx(const char *lump, int16_t *out, int max_samples)
{
    if (!find_wad()) return 0;
    FILE *f = FIO_OpenFile(wad_path, O_RDONLY);
    if (!f) return 0;
    int got = 0;
    uint8_t *b = fio_malloc(64 * 1024);
    if (b && FIO_ReadFile(f, b, 12) == 12 && !memcmp(b + 1, "WAD", 3))
    {
        int nl = *(int *)(b + 4), dir = *(int *)(b + 8), pos = -1, size = 0;
        for (int i = 0; i < nl && pos < 0; i += 2048)
        {
            int k = MIN(2048, nl - i);
            FIO_SeekSkipFile(f, dir + i * 16, SEEK_SET);
            if (FIO_ReadFile(f, b, k * 16) != k * 16) break;
            for (int j = 0; j < k; j++) if (!strncmp((char *)b + j * 16 + 8, lump, 8)) { pos = *(int *)(b + j * 16); size = *(int *)(b + j * 16 + 4); break; }
        }
        if (pos > 0 && size > 40 && size <= 64 * 1024)
        {
            FIO_SeekSkipFile(f, pos, SEEK_SET);
            if (FIO_ReadFile(f, b, size) == size && b[0] == 3)
            {
                int rate = b[2] | (b[3] << 8), n = *(int *)(b + 4) - 32;
                uint8_t *smp = b + 8 + 16;
                if (n > size - 40) n = size - 40;
                int m = (int)((int64_t)n * 24000 / rate);
                if (m > max_samples) m = max_samples;
                for (int i = 0; i < m; i++)             /* linear interpolation, 16.16 */
                {
                    uint32_t x = (uint32_t)((int64_t)i * rate * 65536 / 24000);
                    int i0 = x >> 16, fr = x & 0xffff, a = smp[i0] - 128, c = smp[MIN(i0 + 1, n - 1)] - 128;
                    out[i] = (int16_t)((a * 65536 + (c - a) * fr) >> 8);
                }
                got = m * 2;
                slogf("sfx %s: %d samples at %d Hz -> %d at 24000\n", lump, n, rate, m);
            }
        }
        else slogf("sfx %s: not found (pos %d size %d)\n", lump, pos, size);
    }
    if (b) free(b);
    FIO_CloseFile(f);
    return got;
}

static void sound_task(void *unused)
{
    slogf("\n=== sound test: %s fw %s, lv %d, movie mode %d, AudioBeep block %x state %d result %d\n", camera_model,
          firmware_version, lv, is_movie_mode(), (int)AB_BLOCK, AB_BLOCK ? AB_INT(0x48) : -1, AB_BLOCK ? AB_INT(0x24) : -1);
    if (!AB_BLOCK) { slog_flush(); NotifyBox(4000, "Sound test: AudioBeep not initialised"); doom_running = 0; return; }
    int16_t *b = fio_malloc(200 * 1024);
    if (!b) { slogf("no memory\n"); slog_flush(); doom_running = 0; return; }
    slogf("buffer %x\n", (int)b);
    slog_flush();
    int n;

    NotifyBox(2000, "Sound test 1/6: low tone (24 kHz, output 0)");
    n = tone(b, 12000, 24000, 1, 440, 8000) - b;
    ab_play("1 tone 440 Hz 24k mono out0", b, n * 2, AB_RATE_24K, 1, 0, 5, 1); slog_flush(); msleep(700);

    NotifyBox(2000, "Sound test 2/6: low tone (24 kHz, output 1)");
    ab_play("2 tone 440 Hz 24k mono out1", b, n * 2, AB_RATE_24K, 1, 1, 5, 1); slog_flush(); msleep(700);

    NotifyBox(2000, "Sound test 3/6: higher tone (48 kHz stereo)");
    n = tone(b, 24000, 48000, 2, 660, 8000) - b;
    ab_play("3 tone 660 Hz 48k stereo out0", b, n * 2, AB_RATE_48K, 2, 0, 5, 1); slog_flush(); msleep(700);

    NotifyBox(2000, "Sound test 4/6: Doom pistol");
    int pb = load_sfx("DSPISTOL", b, 100 * 1024 / 2);
    if (pb) ab_play("4 DSPISTOL 24k mono out0", b, pb, AB_RATE_24K, 1, 0, 5, 1);
    slog_flush(); msleep(700);

    NotifyBox(3000, "Sound test 5/6: 10 short beeps back to back");
    int16_t *b2 = b + 2400 + 64;                       /* two 100 ms buffers: 440 / 880 Hz */
    tone(b, 2400, 24000, 1, 440, 8000); tone(b2, 2400, 24000, 1, 880, 8000);
    uint64_t t5 = get_us_clock();
    for (int i = 0; i < 10; i++)
    {
        char w[32]; dm_snprintf(w, sizeof w, "5.%d back-to-back 100 ms", i);
        ab_play(w, i & 1 ? b2 : b, 4800, AB_RATE_24K, 1, 0, 5, 1);
    }
    slogf("5 total %d ms for 10 x 100 ms\n", (int)((get_us_clock() - t5) / 1000));
    slog_flush(); msleep(700);

    NotifyBox(3000, "Sound test 6/6: long tone, cut off by a high one");
    n = tone(b, 24000, 24000, 1, 440, 8000) - b;
    int16_t *b3 = b + n + 64;
    int n3 = tone(b3, 7200, 24000, 1, 880, 8000) - b3;
    ab_play("6a long 1 s tone (not waited)", b, n * 2, AB_RATE_24K, 1, 0, 5, 0);
    msleep(200);
    ab_play("6b cut-in 300 ms", b3, n3 * 2, AB_RATE_24K, 1, 0, 5, 1);
    slogf("end: state %d result %d\n", AB_INT(0x48), AB_INT(0x24));
    slog_flush();
    free(b);
    doom_running = 0;
    NotifyBox(4000, "Sound test done: see ML/DOOM/SOUND.TXT");
}

/* ------------------------------------------------------------------ menu */

static const char *wads[] = { "DOOM1.WAD", "DOOM.WAD", "DOOMU.WAD", "DOOM2.WAD", "FREEDOOM1.WAD", "FREEDOOM2.WAD" };

static int find_wad(void)
{
    for (unsigned i = 0; i < COUNT(wads); i++)
    {
        char p[48];
        uint32_t size;
        dm_snprintf(p, sizeof p, "ML/DOOM/%s", wads[i]);
        if (FIO_GetFileSize(p, &size) == 0 && size > 1024 * 1024)
        {
            strcpy(wad_path, p);
            return 1;
        }
    }
    wad_path[0] = 0;
    return 0;
}

static MENU_SELECT_FUNC(doom_play)
{
    if (doom_running) return;
    if (doom_played) { NotifyBox(3000, "Doom already ran this power-on: restart the camera"); return; }
    if (!find_wad()) { NotifyBox(4000, "Put DOOM1.WAD into ML/DOOM on the card"); return; }
    doom_running = 1;
    task_create("doom_task", 0x1d, 0x8000, doom_task, 0);
}

static MENU_SELECT_FUNC(doom_test)
{
    if (doom_running) return;
    doom_running = 2;
    task_create("doom_test", 0x1d, 0x2000, test_task, 0);
}

static MENU_SELECT_FUNC(doom_sound_test)
{
    if (doom_running) return;
    doom_running = 3;
    task_create("doom_snd", 0x1d, 0x2000, sound_task, 0);
}

static MENU_UPDATE_FUNC(doom_play_update)
{
    if (doom_played) MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Already played this power-on: restart the camera.");
    else if (!find_wad()) MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "No WAD: copy DOOM1.WAD into ML/DOOM on the card.");
    else MENU_SET_RINFO("%s", wad_path + 8);
}

static struct menu_entry doom_menu[] =
{
    {
        .name    = "Doom",
        .select  = doom_play,
        .update  = doom_play_update,
        .help    = "SET: play. PLAY: options. In game MENU quits, Trash = Doom menu.",
        .help2   = "Joystick move/turn, SET or AF-ON fire, PLAY/M-Fn use, dial weapon.",
        .children = (struct menu_entry[])
        {
            {
                .name    = "Picture",
                .priv    = &doom_scale,
                .max     = 3,
                .choices = CHOICES("640x480 (4:3)", "640x400 (2x)", "320x200 (1x)", "720x480 (fill)"),
                .help    = "How the 320x200 game picture is scaled on screen.",
            },
            {
                .name    = "Display & input test",
                .select  = doom_test,
                .help    = "Test pattern + key log (ML/DOOM/KEYS.TXT). Border changes on each key.",
                .help2   = "MENU ends it (or 30 s without keys). No WAD needed.",
            },
            {
                .name    = "Sound test",
                .select  = doom_sound_test,
                .help    = "6 short sounds through Canon's speaker (~15 s). Log: ML/DOOM/SOUND.TXT.",
                .help2   = "Tones, the Doom pistol, back-to-back and cut-in timing. Photo mode first.",
            },
            MENU_EOL,
        },
    },
};

static unsigned int doom_init(void)
{
    menu_add("Games", doom_menu, COUNT(doom_menu));
    return 0;
}

static unsigned int doom_deinit(void) { return 0; }

MODULE_INFO_START()
    MODULE_INIT(doom_init)
    MODULE_DEINIT(doom_deinit)
MODULE_INFO_END()

MODULE_CBRS_START()
    MODULE_CBR(CBR_KEYPRESS_RAW, doom_keypress_raw, 0)
MODULE_CBRS_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(doom_scale)
MODULE_CONFIGS_END()
