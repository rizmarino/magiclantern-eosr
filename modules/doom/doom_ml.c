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
/* Canon SoundDev output volume 0..10 for the movie-play path (the factory test uses 7) */
static CONFIG_INT("doom.spkvol", doom_spk_vol, 8);

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

/* the game task (WAD, saves) and doom_musio (music) both read: one bounce buffer, so one reader at a time */
static struct semaphore *io_sem;
static int dm_io_read_locked(struct cam_file *h, void *buf, size_t n);
int dm_io_read(void *hv, void *buf, size_t n)
{
    if (!io_sem) io_sem = create_named_semaphore("doom_io", SEM_CREATE_UNLOCKED);   /* first call: game task, before doom_musio exists */
    take_semaphore(io_sem, 0);
    int r = dm_io_read_locked(hv, buf, n);
    give_semaphore(io_sem);
    return r;
}
static int dm_io_read_locked(struct cam_file *h, void *buf, size_t n)
{
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

void M_SaveDefaults(void);          /* engine/m_config.c: writes ML/DOOM/default.cfg + extra.cfg */

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
    if (dm_input_quit) { M_SaveDefaults(); dm_exit(1); }   /* MENU skips Doom's I_Quit, which saves the config */
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

static void snd_open(void);
static void snd_close(void);

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
    snd_open();
    uint32_t t0 = get_ms_clock();
    int rc = dm_run_catching_exit(doom_main);
    run_stats("game", t0);
    snd_close();                      /* before dm_libc_release_all: stops Canon reading our buffers */
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
                    int v = ((a * 65536 + (c - a) * fr) >> 8) * 3;   /* x3 louder, clipped */
                    out[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
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

    /* 2026-11-10 camera: every mono step played to length; 48k stereo (96 KB) and 24k stereo (48 KB) both froze
     * the camera, so the stereo steps are gone. Step 6 checks a buffer as big as the game's (96 KB). */
#define STEP(msg) do { slogf("> %s\n", msg); slog_flush(); NotifyBox(2500, "Sound test %s", msg); } while (0)
    STEP("1/6: low tone (24 kHz mono)");
    n = tone(b, 12000, 24000, 1, 440, 26000) - b;
    ab_play("1 tone 440 Hz 24k mono out0", b, n * 2, AB_RATE_24K, 1, 0, 5, 1); slog_flush(); msleep(700);

    STEP("2/6: Doom pistol");
    int pb = load_sfx("DSPISTOL", b, 100 * 1024 / 2);
    if (pb) ab_play("2 DSPISTOL 24k mono out0", b, pb, AB_RATE_24K, 1, 0, 5, 1);
    slog_flush(); msleep(700);

    STEP("3/6: 10 short beeps back to back");
    int16_t *b2 = b + 2400 + 64;                       /* two 100 ms buffers: 440 / 880 Hz */
    tone(b, 2400, 24000, 1, 440, 26000); tone(b2, 2400, 24000, 1, 880, 26000);
    uint64_t t5 = get_us_clock();
    for (int i = 0; i < 10; i++)
    {
        char w[32]; dm_snprintf(w, sizeof w, "3.%d back-to-back 100 ms", i);
        ab_play(w, i & 1 ? b2 : b, 4800, AB_RATE_24K, 1, 0, 5, 1);
    }
    slogf("3 total %d ms for 10 x 100 ms\n", (int)((get_us_clock() - t5) / 1000));
    slog_flush(); msleep(700);

    STEP("4/6: long tone, cut off by a high one");
    n = tone(b, 24000, 24000, 1, 440, 26000) - b;
    int16_t *b3 = b + n + 64;
    int n3 = tone(b3, 7200, 24000, 1, 880, 26000) - b3;
    ab_play("4a long 1 s tone (not waited)", b, n * 2, AB_RATE_24K, 1, 0, 5, 0);
    msleep(200);
    ab_play("4b cut-in 300 ms", b3, n3 * 2, AB_RATE_24K, 1, 0, 5, 1);
    slog_flush(); msleep(700);

    STEP("5/6: small 48 kHz mono (0.1 s)");
    n = tone(b, 4800, 48000, 1, 660, 26000) - b;
    ab_play("5 tone 660 Hz 48k mono 9.6 KB", b, n * 2, AB_RATE_48K, 1, 0, 5, 1); slog_flush(); msleep(700);

    STEP("6/6: long 2 s tone, 24 kHz mono (96 KB)");
    n = tone(b, 48000, 24000, 1, 440, 26000) - b;
    ab_play("6 tone 440 Hz 24k mono 96 KB", b, n * 2, AB_RATE_24K, 1, 0, 5, 1); slog_flush();

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

/* ------------------------------------------------------------------ stream test (Canon movie-play audio path)
 * The beep player above leaves ~12 ms between buffers. Movie playback is gapless: it streams through
 * AudioDS::SoundApi.c play mode, the same API Canon's FileMgr::SoundState.c player (BGM) and the factory audio
 * test use (ghidra notes "Gapless PCM: SoundDev play path", snd-decomp8-11.c):
 *   SetAudioInit(5, 0, sync, fmt) -> SetPlayParam({48000, 16, 2, endian 0, fmt 1 = LPCM, 0, 0, sync, StartCBR,
 *   FreeCBR, PlayCBR, StopCBR, inst, ErrorCBR, errinst}) -> SetStartPlayBuffer(buf) x4 -> SetMovieStart(6);
 *   StartCBR(inst, err): SetOutStart() + unmute; PlayCBR: queue one more buffer (SetNextPlayBuffer); FreeCBR(inst,
 *   adr): that buffer may be reused; stop = SetMovieStop(7) -> StopCBR, then SetAudioEnd(8).
 * Play format is fixed at 48 kHz 16-bit stereo (SoundDev_CheckSamplingRate). Every ACTRL play call asserts while
 * Canon records ("Just Recording", ACTRL_IsActiveState(0x22)), so check that first. Buffers are passed BY VALUE
 * as a 0xE4-byte struct: w[0] = 1 block, w[1] = bytes, w[0x33] start adr, w[0x34] end adr, w[0x35] adr,
 * w[0x36] bytes. Log: ML/DOOM/SOUND.TXT. */
struct sd_buf { uint32_t w[57]; };
static int  (*const actrl_is_active)(int mask, int lock) = (void *)0xE024D0FD;
static int  (*const sd_set_out)(int out) = (void *)0xE05EE115;
static void (*const sd_set_out_volume)(int vol) = (void *)0xE05EE141;
static int  (*const sd_audio_init)(int mode, int a, int sync, int fmt) = (void *)0xE05EDAF7;
static int  (*const sd_play_param)(const int *p) = (void *)0xE05EE2EB;
static int  (*const sd_start_buf)(struct sd_buf b, uint32_t delay_ms) = (void *)0xE05EE8D9;
static int  (*const sd_next_buf)(struct sd_buf b) = (void *)0xE05EE941;
static int  (*const sd_movie_start)(int m) = (void *)0xE05EDB95;
static int  (*const sd_movie_stop)(int m) = (void *)0xE05EDD73;
static int  (*const sd_audio_end)(int m) = (void *)0xE05EDDAF;
static int  (*const sd_out_start)(void) = (void *)0xE05EDE15;
static void (*const sd_mute)(int on) = (void *)0xE05EEA77;

#define ST_SLOTS 10                      /* ring; Canon holds at most ~6 (4 queued + DMA current/next) */
#define ST_EV    160
static struct { uint32_t us; uint8_t type; uint32_t a, b; } st_ev[ST_EV];
static volatile int st_nev;
static volatile int st_plays, st_frees, st_starts, st_stops, st_errs, st_err_id, st_start_err;
static struct semaphore *st_sem, *st_stop_sem;
static uint8_t *st_mem;
static int st_chunk;
static uint64_t st_t0;
static void (*st_fillfn)(int16_t *d, int frames);
static int st_open_ok, st_slot, st_seen, st_submitted, st_mix_us_max;
static uint64_t st_mix_us_sum;
static int st_mix_n;

static void st_log_ev(int type, uint32_t a, uint32_t b)
{
    int i = st_nev;
    if (i >= ST_EV) return;
    st_ev[i].us = (uint32_t)(get_us_clock() - st_t0); st_ev[i].type = type; st_ev[i].a = a; st_ev[i].b = b;
    st_nev = i + 1;
}
static void st_start_cbr(int inst, int err)
{
    st_start_err = err; st_starts++; st_log_ev('S', err, 0);
    if (err == 0) { sd_out_start(); sd_mute(0); }      /* what Canon's BGM player and factory test do here */
}
/* camera 2026-11-15: FreeCBR comes when a buffer is accepted (adr = that buffer), PlayCBR when Canon wants the next
 * one (adr = the buffer it took), one per buffer period after the first two */
static void st_free_cbr(int inst, uint32_t adr, int c, int d) { st_frees++; st_log_ev('F', adr, c); }
static void st_play_cbr(int inst, int b, int c, int d) { st_plays++; st_log_ev('P', b, c); give_semaphore(st_sem); }
static void st_stop_cbr(int inst) { st_stops++; st_log_ev('E', 0, 0); give_semaphore(st_stop_sem); }
static void st_err_cbr(int inst, int id) { st_errs++; st_err_id = id; st_log_ev('X', id, 0); }

/* phase-continuous test signal, 48 kHz stereo: t < 2 s 440 Hz both, 2-3 s 440 Hz left only, 3-4 s 660 Hz right
 * only, then a 440 -> 880 Hz sweep on both. Any gap or repeated buffer is an audible click. */
static uint32_t st_phase, st_frame;
static int st_sine(uint32_t ph)               /* parabolic sine, ph in [0, 2^32) */
{
    int x = (int)(ph >> 16) - 32768;          /* -32768..32767 = -pi..pi */
    int y = (x * 4 / 2 * (32768 - (x < 0 ? -x : x))) >> 15;     /* 4/pi x - 4/pi^2 x|x|, scaled */
    return -(y * 5 / 4);                      /* peak ~20500, like the beep test tones (26000 was loud enough) */
}
static void st_test_fill(int16_t *d, int frames)
{
    for (int i = 0; i < frames; i++, st_frame++)
    {
        uint32_t f = st_frame;
        uint32_t hz = f < 96000 ? 440 : f < 144000 ? 440 : f < 192000 ? 660 : 440 + (f - 192000) / 240;
        if (hz > 880) hz = 880;
        st_phase += hz * 89478u;              /* 2^32 / 48000 = 89478.5 */
        int v = st_sine(st_phase);
        int l = f >= 144000 && f < 192000 ? 0 : v;
        int r = f >= 96000 && f < 144000 ? 0 : v;
        d[2 * i] = l; d[2 * i + 1] = r;
    }
}
static void st_fill_slot(int slot)
{
    uint64_t t = get_us_clock();
    st_fillfn((int16_t *)(st_mem + slot * st_chunk), st_chunk / 4);
    int us = (int)(get_us_clock() - t);
    st_mix_us_sum += us; st_mix_n++;
    if (us > st_mix_us_max) st_mix_us_max = us;
}
static int st_submit(int slot, int first)
{
    struct sd_buf b;
    memset(&b, 0, sizeof b);
    uint32_t adr = (uint32_t)st_mem + slot * st_chunk;
    b.w[0] = 1; b.w[1] = st_chunk;
    b.w[0x33] = adr; b.w[0x34] = adr + st_chunk; b.w[0x35] = adr; b.w[0x36] = st_chunk;
    return first ? sd_start_buf(b, 0) : sd_next_buf(b);
}

typedef void (*st_logfn)(const char *fmt, ...);
#define SCALL(what, expr) ({ lg("> %s\n", what); if (flush) flush(); int _r = (int)(expr); lg("  rc %d\n", _r); _r; })

/* open Canon's play device and start a gapless 48 kHz stereo stream fed by fill(); 0 = running */
static int st_open(int chunk, int vol, void (*fill)(int16_t *, int), st_logfn lg, void (*flush)(void))
{
    if (st_open_ok) return 1;
    if (!st_sem) st_sem = create_named_semaphore("doom_st", SEM_CREATE_LOCKED);
    if (!st_stop_sem) st_stop_sem = create_named_semaphore("doom_st_stop", SEM_CREATE_LOCKED);
    st_chunk = chunk; st_fillfn = fill; st_nev = 0;
    st_plays = st_frees = st_starts = st_stops = st_errs = st_err_id = 0; st_start_err = -1;
    st_submitted = st_mix_us_max = st_mix_n = 0; st_mix_us_sum = 0;
    lg("stream: chunk %d B (%d us), volume %d; ACTRL active 0x22 %d, 0x11 %d, 0x20 %d\n", chunk,
       (int)(chunk * 1000LL / 192), vol, actrl_is_active(0x22, 0), actrl_is_active(0x11, 0), actrl_is_active(0x20, 0));
    if (actrl_is_active(0x22, 0)) { lg("Canon audio input busy (movie mode or recording): no sound\n"); return 1; }
    st_mem = fio_malloc(ST_SLOTS * chunk);
    if (!st_mem) { lg("stream: no memory\n"); return 1; }
    memset(st_mem, 0, ST_SLOTS * chunk);
    static int param[15];
    param[0] = 48000; param[1] = 16; param[2] = 2; param[3] = 0; param[4] = 1; param[5] = 0; param[6] = 0;
    param[7] = 1; param[8] = (int)st_start_cbr; param[9] = (int)st_free_cbr; param[10] = (int)st_play_cbr;
    param[11] = (int)st_stop_cbr; param[12] = 0x57; param[13] = (int)st_err_cbr; param[14] = 0x57;

    SCALL("SetOut(0)", sd_set_out(0));
    lg("> SetOutVolume(%d)\n", vol); if (flush) flush(); sd_set_out_volume(vol);
    if (SCALL("SetAudioInit(5, 0, 1, 1)", sd_audio_init(5, 0, 1, 1))) goto fail_mem;
    if (SCALL("SetPlayParam", sd_play_param(param))) goto fail_end;
    st_t0 = get_us_clock();
    for (int i = 0; i < 4; i++)
    {
        st_fill_slot(i);
        int r = st_submit(i, 1);
        if (r) lg("  start buf %d rc %d\n", i, r);
        st_submitted++;
    }
    if (SCALL("SetMovieStart(6)", sd_movie_start(6))) goto fail_stop;
    st_slot = 4; st_seen = 0;
    st_fill_slot(st_slot);
    st_open_ok = 1;
    return 0;
fail_stop:
    SCALL("SetMovieStop(7)", sd_movie_stop(7));
    take_semaphore(st_stop_sem, 2000);
fail_end:
    SCALL("SetAudioEnd(8)", sd_audio_end(8));
fail_mem:
    free(st_mem); st_mem = NULL;
    return 1;
}

/* queue one buffer per PlayCBR (waiting up to wait_ms for one), then prepare the next slot; limit < 0: no limit */
static int st_pump(int wait_ms, int limit)
{
    if (!st_open_ok) { msleep(wait_ms); return 0; }
    take_semaphore(st_sem, wait_ms);
    int n = 0;
    while (st_seen < st_plays && (limit < 0 || st_submitted < limit))
    {
        st_seen++;
        int r = st_submit(st_slot, 0);
        if (r) st_log_ev('R', r, st_slot);
        st_submitted++; n++;
        st_slot = (st_slot + 1) % ST_SLOTS;
        st_fill_slot(st_slot);
    }
    return n;
}

static void st_close(st_logfn lg, void (*flush)(void))
{
    if (!st_open_ok) return;
    st_open_ok = 0;
    SCALL("SetMovieStop(7)", sd_movie_stop(7));
    { int r = take_semaphore(st_stop_sem, 2000); lg("  StopCBR wait %s\n", r ? "TIMEOUT" : "ok"); }
    SCALL("SetAudioEnd(8)", sd_audio_end(8));
    msleep(50);
    lg("stream result: submitted %d, PlayCBR %d, FreeCBR %d, StartCBR %d (err %d), StopCBR %d, ErrorCBR %d (id %x); "
       "fill avg %d us max %d us\n", st_submitted, st_plays, st_frees, st_starts, st_start_err, st_stops, st_errs,
       st_err_id, st_mix_n ? (int)(st_mix_us_sum / st_mix_n) : 0, st_mix_us_max);
    free(st_mem); st_mem = NULL;
}

static void st_dump_events(st_logfn lg)
{
    uint32_t prev = 0; int dmin = 1 << 30, dmax = 0, np = 0;
    for (int i = 0; i < st_nev; i++)
    {
        if (i < 24 || i >= st_nev - 6) lg("  %c t=%u us a=%x b=%x\n", st_ev[i].type, st_ev[i].us, st_ev[i].a, st_ev[i].b);
        if (st_ev[i].type == 'P') { if (np++) { int d = st_ev[i].us - prev; if (d < dmin) dmin = d; if (d > dmax) dmax = d; } prev = st_ev[i].us; }
    }
    if (np > 1) lg("  PlayCBR spacing %d..%d us (one buffer = %d us), %d events logged\n", dmin, dmax,
                   (int)(st_chunk * 1000LL / 192), st_nev);
}

/* one test run; returns 0 if it played to the end */
static int stream_run(int chunk, int seconds, int vol)
{
    st_phase = st_frame = 0;
    slogf("\n--- stream: %d s\n", seconds);
    if (st_open(chunk, vol, st_test_fill, slogf, slog_flush)) { slog_flush(); return 1; }
    int total = seconds * 192000 / chunk;
    uint32_t tstart = get_ms_clock();
    while (st_submitted < total && get_ms_clock() - tstart < (uint32_t)(seconds * 1000 + 3000)) st_pump(200, total);
    msleep(4 * chunk * 1000 / 192000 + 100);  /* let the queued buffers play out */
    int ok = st_submitted == total;
    slogf("ran %d ms for %d ms of audio\n", (int)(get_ms_clock() - tstart), st_submitted * chunk * 1000 / 192000);
    st_close(slogf, slog_flush);
    st_dump_events(slogf);
    slog_flush();
    msleep(200);
    return ok && st_plays > 0 ? 0 : 1;
}
#undef SCALL

static void stream_task(void *unused)
{
    slogf("\n=== stream test: %s fw %s, lv %d, movie mode %d (lv_movie_select %d)\n", camera_model, firmware_version,
          lv, is_movie_mode(), lv_movie_select);
    slog_flush();
    NotifyBox(3000, "Stream test 1/2: 6 s tone, 85 ms buffers");
    int r1 = stream_run(16384, 6, doom_spk_vol);
    msleep(800);
    int r2 = 1;
    if (!r1)
    {
        NotifyBox(3000, "Stream test 2/2: 4 s tone, 21 ms buffers");
        r2 = stream_run(4096, 4, doom_spk_vol);
    }
    slogf("end: run1 %s, run2 %s; AudioBeep state %d\n", r1 ? "FAILED" : "ok", r1 ? "skipped" : r2 ? "FAILED" : "ok",
          AB_BLOCK ? AB_INT(0x48) : -1);
    slog_flush();
    doom_running = 0;
    NotifyBox(4000, "Stream test done: see ML/DOOM/SOUND.TXT");
}
static MENU_SELECT_FUNC(doom_stream_test)
{
    if (doom_running) return;
    doom_running = 3;
    task_create("doom_stream", 0x1c, 0x2000, stream_task, 0);
}

/* ------------------------------------------------------------------ in-game sound and music (dm/dm_sound.c)
 * The game's mixer feeds the Stream test's path: doom_audio (one priority step above doom_task) queues a 21 ms
 * buffer per PlayCBR and mixes the next one (dm_snd_mix), so the game never waits on Canon. doom_musio (below
 * doom_task) streams the pre-rendered music file from the card (dm_mus_service); it runs whenever the game sleeps or
 * yields (at least every 200 ms) and keeps ~5 s of music ahead. */
#define GS_CHUNK 4096                     /* 1024 frames = 21.3 ms: the camera-verified small size */
static volatile int gs_run, gs_alive, mio_run, mio_alive;

static void gs_fill(int16_t *d, int frames) { dm_snd_mix(d, frames); }
static void gs_task(void *unused)
{
    while (gs_run) st_pump(100, -1);
    gs_alive = 0;
}
static void mio_task(void *unused)
{
    while (mio_run) { dm_mus_service(); msleep(20); }
    mio_alive = 0;
}
void dm_sys_sleep(int ms) { msleep(ms < 10 ? 10 : ms); }

int dm_snd_hw_open(void)
{
    if (gs_run) return 0;
    if (st_open(GS_CHUNK, doom_spk_vol, gs_fill, logf_, log_flush)) return 1;
    gs_run = gs_alive = 1;
    task_create("doom_audio", 0x1c, 0x1000, gs_task, 0);
    mio_run = mio_alive = 1;
    task_create("doom_musio", 0x1e, 0x1000, mio_task, 0);
    return 0;
}

/* idempotent: Doom's own quit calls it (Snd_Shutdown), doom_task again after any exit */
void dm_snd_hw_close(void)
{
    if (mio_run) { mio_run = 0; for (int i = 0; i < 200 && mio_alive; i++) msleep(10); }
    if (mio_alive) logf_("music task did not end\n");
    else dm_mus_release();
    if (!gs_run) return;
    gs_run = 0;
    give_semaphore(st_sem);
    for (int i = 0; i < 100 && gs_alive; i++) msleep(10);
    if (gs_alive) { logf_("sound: audio task did not end, stream kept\n"); return; }
    st_close(logf_, log_flush);
}

static void snd_open(void) { }
static void snd_close(void)
{
    char line[256];
    dm_snd_stats(line, sizeof line);
    logf_("%s", line);
    dm_snd_hw_close();                    /* before dm_libc_release_all: stops Canon reading our buffers */
}

static MENU_UPDATE_FUNC(doom_play_update)
{
    if (doom_played) MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Already played this power-on: restart the camera.");
    else if (!find_wad()) MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "No WAD: copy DOOM1.WAD into ML/DOOM on the card.");
    else
    {
        MENU_SET_RINFO("%s", wad_path + 8);
        /* Canon's audio input runs in movie live view (sound recording on): the stream is not opened then (st_open) */
        if (actrl_is_active(0x22, 0)) MENU_SET_WARNING(MENU_WARN_ADVICE, "No sound in movie mode: switch to photo mode for sound.");
    }
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
                .name    = "Speaker volume",
                .priv    = &doom_spk_vol,
                .min     = 1,
                .max     = 10,
                .help    = "Canon output level for Doom's sound (1-10). Doom's own sliders mix effects/music.",
            },
            {
                .name    = "Display & input test",
                .select  = doom_test,
                .help    = "Test pattern + key log (ML/DOOM/KEYS.TXT). Border changes on each key.",
                .help2   = "MENU ends it (or 30 s without keys). No WAD needed.",
            },
#ifdef DOOM_TESTS   /* release cleanup: audio bring-up tests (camera-verified 2026-11-10/15), research builds only */
            {
                .name    = "Sound test",
                .select  = doom_sound_test,
                .help    = "6 short sounds through Canon's speaker (~15 s). Log: ML/DOOM/SOUND.TXT.",
                .help2   = "Mono only: stereo froze the camera twice (2026-11-10).",
            },
            {
                .name    = "Stream test",
                .select  = doom_stream_test,
                .help    = "Gapless audio test (movie-play path): 6 s + 4 s continuous tone.",
                .help2   = "Listen for clicks/gaps. Photo mode, not recording. Log: ML/DOOM/SOUND.TXT.",
            },
#endif
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
    MODULE_CONFIG(doom_spk_vol)
MODULE_CONFIGS_END()
