/* riz v4.9 (EOS R): RAW histogram and RAW zebras in movie mode, from Canon's LV raw tap (MEM1).
 *
 * The R build has no CONFIG_RAW_LIVEVIEW (raw.c's LV path targets DIGIC 4/5 registers), so this is a small
 * self-contained sampler instead. While Overlay > Histogram type is "RAW-based (RGB)" or Zebras > Use RAW zebras is on,
 * in movie-mode LV with Global Draw on, it keeps Canon's raw tap on (call("lv_save_raw", 1), the same switch the RAW
 * recorder uses) and ~4 times a second reads one RGGB quad per cell of a 96x54 grid from the raw frame:
 * - format straight from Canon's Mem1Path struct (0x133CC): size +0x10/+0x14, address +0x58, pack cfg +0x64.. (f2 =
 *   14/12/10-bit, f3 = 1 Canon LSB-first bitstream / 0 ML raw_pixblock order); so it reads correctly while the RAW
 *   recorder or LV silent pictures have the pack unit at 10/12-bit;
 * - black level and noise from the left optical-black columns (FHD 48, 4K 144), white 16383 / clip 16300 (14-bit,
 *   scaled for 10/12-bit);
 * - per cell: R / G / B EV bins (12 stops over 128 bins, as ML's raw histogram) and clip / under flags for zebras.
 * Uncached reads (~1.2 us each, 4 per quad): ~25 ms per pass. Results are double-buffered; zebra.c / histogram.c read
 * them via r_rawlv_hist_fill / r_rawlv_cells. The tap is switched off again when nothing needs it, when Canon records
 * H.264, and when LV stops (lv_save_raw only sets two flags: flags 0x1311C+4 and Mem1Path+0x40). It never switches
 * the tap off while the recorder / silent pictures run (rawrec_running). */

#ifndef RL_HOST
#include "dryos.h"
#include "property.h"
#include "propvalues.h"
#include "bmp.h"
#include "menu.h"
#include "config.h"
#include "histogram.h"
#include "zebra.h"
#include "lens.h"
#define RL_RD32(a) (*(volatile uint32_t *)(a))
extern int rawrec_running(void);
extern int r_rawlv_wanted(void);        /* zebra.c: RAW histogram or RAW zebras selected and drawn */
extern int get_global_draw_setting(void);
#endif

#define RL_PATH   0x133CC
#define RL_BUF    0x28EEA940
#define RL_BUFLEN 0xFB9D90
#define RL_UNC    0x40000000
#define RL_GW     96
#define RL_GH     54
#define RL_NOB    64                    /* optical-black samples per pass */

struct rl_fmt { uint32_t w, h, pitch, bpp, ml_order, obl, obt; };

/* 0 = not usable (tap off, unknown size, pack cfg mid-change) */
static int rl_read_fmt(struct rl_fmt *f)
{
    if (!MEM(RL_PATH + 0x40)) return 0;
    uint32_t w = MEM(RL_PATH + 0x10), h = MEM(RL_PATH + 0x14), a = MEM(RL_PATH + 0x58);
    if ((a & ~RL_UNC) != RL_BUF) return 0;
    if (w == 2304 && h == 1290) { f->obl = 48; f->obt = 22; }           /* FHD movie LV */
    else if (w == 4256 && h == 2214) { f->obl = 144; f->obt = 46; }     /* 4K movie LV */
    else return 0;
    uint32_t f2 = MEM(RL_PATH + 0x64 + 8), f3 = MEM(RL_PATH + 0x64 + 12);
    uint32_t bpp = f2 == 2 ? 14 : f2 == 1 ? 12 : f2 == 0 ? 10 : 0;
    if (!bpp || f3 > 1) return 0;
    uint32_t pitch = w * bpp / 8;
    if (((MEM(RL_PATH + 0x18) * 14) >> 3) != pitch || (pitch & 3) || pitch * h > RL_BUFLEN) return 0;
    f->w = w; f->h = h; f->pitch = pitch; f->bpp = bpp; f->ml_order = f3 == 0;
    return 1;
}

/* one pixel; row = uncached row address (4-aligned), x < w - 8 */
static inline uint32_t rl_px(uint32_t row, uint32_t x, const struct rl_fmt *f)
{
    uint32_t bit = x * f->bpp, o = bit & 31, mask = (1u << f->bpp) - 1;
    uint32_t wa = row + ((bit >> 5) << 2);
    uint32_t w0 = RL_RD32(wa), w1 = (o + f->bpp > 32) ? RL_RD32(wa + 4) : 0;
    if (!f->ml_order) return (uint32_t)(((uint64_t)w1 << 32 | w0) >> o) & mask;
    /* ML order: 16-bit LE words, MSB-first: stream bits of one LE u32 = (low half << 16) | high half */
    uint64_t s = (uint64_t)((w0 << 16) | (w0 >> 16)) << 32 | ((w1 << 16) | (w1 >> 16));
    return (uint32_t)(s >> (64 - o - f->bpp)) & mask;
}

static const uint8_t rl_l2tab[64] = {
    0, 6, 11, 17, 22, 28, 33, 38, 44, 49, 54, 59, 63, 68, 73, 78, 82, 87, 92, 96, 100, 105, 109, 113, 118, 122, 126,
    130, 134, 138, 142, 146, 150, 154, 157, 161, 165, 169, 172, 176, 179, 183, 186, 190, 193, 197, 200, 203, 207, 210,
    213, 216, 220, 223, 226, 229, 232, 235, 238, 241, 244, 247, 250, 253 };
static int rl_log2_256(uint32_t d)      /* 256 * log2(d), d >= 1, error < 1/64 EV */
{
    int e = 31 - __builtin_clz(d);
    uint32_t m = e >= 6 ? (d >> (e - 6)) & 63 : (d << (6 - e)) & 63;
    return e * 256 + rl_l2tab[m];
}
static uint32_t rl_isqrt(uint32_t v)
{
    uint32_t r = 0, b = 1u << 30;
    while (b > v) b >>= 2;
    while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; }
    return r;
}

/* results (double-buffered) */
struct rl_res
{
    uint32_t hist_r[HIST_WIDTH], hist_g[HIST_WIDTH], hist_b[HIST_WIDTH], total;
    uint8_t cell[RL_GH][RL_GW];         /* bit0/1/2 R/G/B clipped, bit3 under ((R+2G+B)/4 < black + 2 sigma + 1) */
    int black, sigma_x10, dr_x10, w, h, bpp, ml_order, pass_ms, t_ms, t0_ms;
};
#define RL_CLIP_R 1
#define RL_CLIP_G 2
#define RL_CLIP_B 4
#define RL_UNDER  8
static struct rl_res *rl_buf[2];
static volatile int rl_front = -1;      /* -1 = no result yet */
static volatile int rl_passes, rl_skips;
static const char *volatile rl_why = "starting";

static inline int rl_bin(int v, int black, int clip, int lfull)
{
    if (v >= clip) return HIST_WIDTH - 1;
    int d = v - black;
    if (d <= 0) return 0;
    int b = (rl_log2_256(d) - lfull + 12 * 256) * (HIST_WIDTH - 1) / (12 * 256);
    return b < 0 ? 0 : b > HIST_WIDTH - 2 ? HIST_WIDTH - 2 : b;
}

/* one pass into r; returns 0 if the format changed underneath (discard) */
static int rl_pass(struct rl_res *r, const struct rl_fmt *f)
{
    const uint32_t base = RL_BUF | RL_UNC;
    const int sh = 14 - f->bpp, white = 16383 >> sh, clip = 16300 >> sh;
    const uint32_t aw = f->w - f->obl, ah = f->h - f->obt;

    /* black + noise: green pixels in the left OB columns */
    uint32_t sum = 0; uint64_t sum2 = 0;
    for (int k = 0; k < RL_NOB; k++)
    {
        uint32_t x = (8 + (k % 8) * ((f->obl - 16) / 8)) & ~1u, y = (f->obt + (uint32_t)k * (ah - 2) / RL_NOB) & ~1u;
        uint32_t g = rl_px(base + y * f->pitch, x + 1, f);
        sum += g; sum2 += (uint64_t)g * g;
    }
    int black = sum / RL_NOB;
    uint64_t n2 = (uint64_t)RL_NOB * sum2 - (uint64_t)sum * sum;      /* N^2 * variance, exact */
    uint32_t var_x100 = (uint32_t)(n2 * 100 / ((uint64_t)RL_NOB * RL_NOB));
    int sigma_x10 = rl_isqrt(var_x100);
    if (black <= 0 || black >= clip / 2) return 0;      /* not a raw frame (or not in this format) */
    int lfull = rl_log2_256(white - black);
    const int under_x10 = 2 * sigma_x10 + 10;      /* (R+2G+B)/4 within ~1 EV of the noise floor (+1 count: quantisation) */

    memset(r->hist_r, 0, sizeof(r->hist_r)); memset(r->hist_g, 0, sizeof(r->hist_g)); memset(r->hist_b, 0, sizeof(r->hist_b));
    for (int j = 0; j < RL_GH; j++)
    {
        uint32_t y = (f->obt + (2 * j + 1) * ah / (2 * RL_GH)) & ~1u;
        uint32_t row0 = base + y * f->pitch, row1 = row0 + f->pitch;
        for (int i = 0; i < RL_GW; i++)
        {
            uint32_t x = (f->obl + (2 * i + 1) * aw / (2 * RL_GW)) & ~1u;
            int R = rl_px(row0, x, f), G = rl_px(row0, x + 1, f), B = rl_px(row1, x + 1, f);
            r->hist_r[rl_bin(R, black, clip, lfull)]++;
            r->hist_g[rl_bin(G, black, clip, lfull)]++;
            r->hist_b[rl_bin(B, black, clip, lfull)]++;
            int m_x10 = (R + 2 * G + B) * 10 / 4 - black * 10;
            r->cell[j][i] = (R >= clip ? RL_CLIP_R : 0) | (G >= clip ? RL_CLIP_G : 0) | (B >= clip ? RL_CLIP_B : 0)
                            | (m_x10 < under_x10 ? RL_UNDER : 0);
        }
    }
    r->total = RL_GW * RL_GH;
    r->black = black; r->sigma_x10 = sigma_x10;
    r->dr_x10 = sigma_x10 ? (lfull - rl_log2_256(MAX(sigma_x10, 1)) + rl_log2_256(10)) * 10 / 256 : 0;
    r->w = f->w; r->h = f->h; r->bpp = f->bpp; r->ml_order = f->ml_order;
    struct rl_fmt g;                    /* pack cfg changed during the pass (recorder start/stop): discard */
    return rl_read_fmt(&g) && g.bpp == f->bpp && g.ml_order == f->ml_order && g.w == f->w;
}

/* ---- consumers (overlay task) ---- */
static int rl_fresh(void)
{
    int k = rl_front;
    return k >= 0 && rl_buf[k] && get_ms_clock() - rl_buf[k]->t_ms < 1500;
}

/* zebra.c hist_build: fill ML's histogram from the newest raw pass; 0 = no raw data (fall back to YUV) */
int r_rawlv_hist_fill(struct Histogram *h)
{
    if (!rl_fresh()) return 0;
    struct rl_res *r = rl_buf[rl_front];
    memcpy(h->hist_r, r->hist_r, sizeof(h->hist_r));
    memcpy(h->hist_g, r->hist_g, sizeof(h->hist_g));
    memcpy(h->hist_b, r->hist_b, sizeof(h->hist_b));
    for (int i = 0; i < HIST_WIDTH; i++) h->hist[i] = (r->hist_r[i] + 2 * r->hist_g[i] + r->hist_b[i]) / 4;
    uint32_t mx = 0;
    for (int i = 0; i < HIST_WIDTH; i++) mx = MAX(mx, MAX(MAX(h->hist_r[i], h->hist_g[i]), h->hist_b[i]));
    h->max = mx;
    h->total_px = r->total;
    h->is_raw = 1; h->is_rgb = 1;
    return 1;
}

/* histogram.c: dynamic range of the newest pass (0.1 EV), 0 = none */
int r_rawlv_dr_x10(void) { return rl_fresh() ? rl_buf[rl_front]->dr_x10 : 0; }

/* zebra.c raw zebras: cell flags of the newest pass, NULL = none */
const uint8_t *r_rawlv_cells(int *gw, int *gh)
{
    if (!rl_fresh()) return 0;
    *gw = RL_GW; *gh = RL_GH;
    return &rl_buf[rl_front]->cell[0][0];
}

/* Histogram > RAW source (menu) */
void r_rawlv_status(char *s, int n, int *ok)
{
    *ok = rl_fresh();
    if (*ok)
    {
        struct rl_res *r = rl_buf[rl_front];
        snprintf(s, n, "%dx%d %d-bit, black %d, noise %d.%d, %d ms", r->w, r->h, r->bpp, r->black, r->sigma_x10 / 10,
                 r->sigma_x10 % 10, r->pass_ms);
    }
    else snprintf(s, n, "%s", rl_why);
}

/* ---- sampler task ---- */
static int r_ettr_wants_raw(void);
static int rl_own, rl_lv_since, rl_settle_until;
static void rl_step(void)
{
    const int now = get_ms_clock();
    if (!lv)
    {
        if (rl_own && !rawrec_running()) call("lv_save_raw", 0);   /* flag writes only (Vram.c lv_save_raw) */
        rl_own = 0; rl_lv_since = 0; rl_why = "LiveView off";
        return;
    }
    if (!rl_lv_since) rl_lv_since = now;
    int canon_rec = RECORDING && !rawrec_running();
    int want = (r_rawlv_wanted() || r_ettr_wants_raw()) && lv_movie_select == LVMS_ENABLE_MOVIE && get_global_draw_setting() && !canon_rec;
    if (!want)
    {
        if (rl_own && MEM(RL_PATH + 0x40) && !rawrec_running())
        {
            call("lv_save_raw", 0);
            if (rawrec_running()) call("lv_save_raw", 1);    /* the recorder started in between: give it back */
        }
        rl_own = 0;
        rl_why = canon_rec ? "paused: Canon recording" : lv_movie_select != LVMS_ENABLE_MOVIE ? "movie mode only" : "off";
        return;
    }
    if (now - rl_lv_since < 1000) return;            /* let LV start fully first (race in Canon code, raw.c) */
    if (!MEM(RL_PATH + 0x40))
    {
        if (rawrec_running()) return;               /* recorder / silent pictures switching it themselves */
        call("lv_save_raw", 1);
        rl_own = 1; rl_settle_until = now + 500; rl_why = "starting";
        return;
    }
    if (now < rl_settle_until || !DISPLAY_IS_ON) return;   /* also runs with the ML menu up: Histogram > RAW source */
    struct rl_fmt f;
    if (!rl_read_fmt(&f)) { rl_skips++; rl_why = "raw format not recognised"; return; }
    if (!rl_buf[0])
    {
        rl_buf[0] = malloc(sizeof(struct rl_res)); rl_buf[1] = malloc(sizeof(struct rl_res));
        if (!rl_buf[0] || !rl_buf[1]) { rl_why = "out of memory"; if (rl_buf[0]) free(rl_buf[0]); if (rl_buf[1]) free(rl_buf[1]); rl_buf[0] = rl_buf[1] = 0; return; }
    }
    int k = rl_front == 0 ? 1 : 0;
    struct rl_res *r = rl_buf[k];
    int t0 = get_ms_clock();
    if (!rl_pass(r, &f)) { rl_skips++; return; }
    r->t_ms = get_ms_clock(); r->pass_ms = r->t_ms - t0; r->t0_ms = t0;
    rl_front = k;
    rl_passes++;
}

static void rl_task(void *unused)
{
    for (;;)
    {
        msleep(rawrec_running() ? 500 : 250);
        rl_step();
    }
}

/* ---- v4.10: movie ETTR (Expo > Auto ETTR (movie)) ----
 * Expose to the right with ISO only (video keeps its shutter angle): from the newest raw pass, find the brightest
 * channel's level above which only "Highlight ignore" of the samples lie, and set ISO so that level sits "Headroom"
 * below clipping. Clipped beyond the ignore share: step down (-1 EV - headroom) and measure again. Up to 5 steps, each
 * on a raw pass that started >= 400 ms after the ISO change. ISO in the R's own 1/3-stop codes (15 = ISO 100, +3 per
 * stop, lens.c r_iso_to_ml_raw), so the readback check is exact. Never while recording (Canon H.264 or RAW video).
 * Triggers: menu "Run now", half-press (movie LV, ML menu closed), or Continuous (every ~2 s, hysteresis 0.7 EV). */
static CONFIG_INT("r.ettr.trig", ettr_trig, 0);         /* 0 OFF, 1 half-press, 2 continuous */
static CONFIG_INT("r.ettr.ignore", ettr_ign_sel, 1);     /* 0 / 0.1 / 0.5 / 1 / 2 % */
static CONFIG_INT("r.ettr.head", ettr_head_sel, 1);      /* 0.3 / 0.5 / 1.0 EV */
static CONFIG_INT("r.ettr.maxiso", ettr_max_sel, 2);     /* 800 1600 3200 6400 12800 */
static const int ettr_ign_x1000[] = { 0, 1, 5, 10, 20 };   /* per mille */
static const int ettr_head_x10[] = { 3, 5, 10 };
static const int ettr_max_iso[] = { 800, 1600, 3200, 6400, 12800 };
static volatile int ettr_run_req, ettr_busy;
static char ettr_last[48] = "not run yet";

static int r_ettr_wants_raw(void) { return ettr_trig || ettr_run_req || ettr_busy; }

static int rl_div_round(int a, int b) { return a >= 0 ? (2 * a + b) / (2 * b) : -((-2 * a + b) / (2 * b)); }
static int ettr_code_of_raw(int raw) { return raw ? 15 + rl_div_round((raw - 72) * 3, 8) : 0; }    /* = lens.c ml_raw_to_r_iso */
static int ettr_raw_of_code(int code) { return 72 + rl_div_round((code - 15) * 8, 3); }            /* = lens.c r_iso_to_ml_raw */
static int ettr_code_of_iso(int iso) { int c = 15; for (int i = 100; i < iso; i *= 2) c += 3; return c; }

/* EV (x100) from the ignore-percentile highlight level to clipping; -1 = clipped beyond the ignore share */
static int ettr_stops_x100(const struct rl_res *r, int ign_x1000)
{
    uint32_t lim = (uint32_t)((uint64_t)r->total * ign_x1000 / 1000);
    const uint32_t *h[3] = { r->hist_r, r->hist_g, r->hist_b };
    int top = 0;
    for (int c = 0; c < 3; c++)
    {
        uint32_t acc = 0;
        int b = HIST_WIDTH - 1;
        for (; b > 0; b--) { acc += h[c][b]; if (acc > lim) break; }
        if (b > top) top = b;
    }
    if (top >= HIST_WIDTH - 1) return -1;
    return (HIST_WIDTH - 1 - top) * 1200 / (HIST_WIDTH - 1) - 1200 / (HIST_WIDTH - 1) / 2;   /* bin centre */
}

/* newest pass that started at or after t (waits up to ~3 s) */
static struct rl_res *ettr_wait_pass(int t)
{
    for (int i = 0; i < 60; i++)
    {
        int k = rl_front;
        if (k >= 0 && rl_buf[k] && rl_buf[k]->t0_ms >= t) return rl_buf[k];
        msleep(50);
    }
    return 0;
}

static int ettr_blocked(void) { return RECORDING || rawrec_running() || !lv || lv_movie_select != LVMS_ENABLE_MOVIE; }

/* one ETTR run; quiet = continuous mode (no message box unless an error); returns 1 if ISO changed */
static int ettr_run(int quiet, int hyst_x100)
{
    if (ettr_blocked()) { snprintf(ettr_last, sizeof(ettr_last), "%s", lv_movie_select != LVMS_ENABLE_MOVIE ? "movie mode only" : RECORDING || rawrec_running() ? "not while recording" : "LiveView off"); return 0; }
    if (!lens_info.raw_iso) { snprintf(ettr_last, sizeof(ettr_last), "ISO is Auto: set a manual ISO"); if (!quiet) NotifyBox(3000, "ETTR: %s", ettr_last); return 0; }
    ettr_busy = 1;
    const int iso0 = lens_info.raw_iso, code0 = ettr_code_of_raw(iso0);
    const int maxc = ettr_code_of_iso(ettr_max_iso[COERCE(ettr_max_sel, 0, 4)]);
    const int head = ettr_head_x10[COERCE(ettr_head_sel, 0, 2)] * 10, ign = ettr_ign_x1000[COERCE(ettr_ign_sel, 0, 4)];
    int code = code0, changed = 0, steps = 0, t = get_ms_clock() - 300, err = 0, st = 0;
    for (; steps < 5; steps++)
    {
        struct rl_res *r = ettr_wait_pass(t);
        if (!r) { err = 1; snprintf(ettr_last, sizeof(ettr_last), "no raw data (%s)", rl_why); break; }
        if (ettr_blocked()) { err = 1; snprintf(ettr_last, sizeof(ettr_last), "stopped: recording / mode change"); break; }
        st = ettr_stops_x100(r, ign);
        int delta_x100 = st < 0 ? -(100 + head) : st - head;
        if (st >= 0 && ABS(delta_x100) < (steps ? 20 : hyst_x100)) break;           /* on target */
        /* 1/3-stop codes, rounded to the safe side: raise at most to the headroom, lower at least to it */
        int dc = delta_x100 >= 0 ? delta_x100 * 3 / 100 : -((-delta_x100 * 3 + 99) / 100);
        if (!dc) break;                                 /* within 1/3 stop below the target */
        int nc = COERCE(code + dc, 15, maxc);
        if (nc == code) break;                          /* at a limit */
        lens_set_rawiso(ettr_raw_of_code(nc));
        msleep(200);
        if (lens_info.raw_iso != ettr_raw_of_code(nc))
        { err = 1; snprintf(ettr_last, sizeof(ettr_last), "ISO change not accepted (%d)", raw2iso(lens_info.raw_iso)); break; }
        code = nc; changed = 1;
        t = get_ms_clock() + 400;                       /* Canon applies the gain a few frames later */
    }
    if (!err)
    {
        int ev10 = (code - code0) * 10 / 3;
        const char *lim = st < 0 && code == 15 ? ", still clipped" : code == maxc && st > head + 20 ? ", max ISO" : "";
        if (changed) snprintf(ettr_last, sizeof(ettr_last), "ISO %d -> %d (%s%d.%d EV)%s", raw2iso(iso0), raw2iso(lens_info.raw_iso),
                              ev10 < 0 ? "-" : "+", ABS(ev10) / 10, ABS(ev10) % 10, lim);
        else snprintf(ettr_last, sizeof(ettr_last), "ISO %d kept%s", raw2iso(iso0), *lim ? lim : " (on target)");
    }
    if (!quiet || err) NotifyBox(3000, "ETTR: %s", ettr_last);
    ettr_busy = 0;
    return changed;
}

static void ettr_task(void *unused)
{
    int last_half = 0, next_cont = 0;
    for (;;)
    {
        msleep(50);
        if (ettr_run_req) { while (gui_menu_shown()) msleep(100); ettr_run(0, 20); ettr_run_req = 0; continue; }
        if (ettr_trig == 1)
        {
            int h = get_halfshutter_pressed();
            if (h && !last_half && !gui_menu_shown() && !ettr_blocked()) ettr_run(0, 20);
            last_half = h;
        }
        else if (ettr_trig == 2 && get_ms_clock() >= next_cont)
        {
            next_cont = get_ms_clock() + 2000;
            if (!gui_menu_shown() && !ettr_blocked() && lens_info.raw_iso && rl_fresh()) ettr_run(1, 70);
        }
    }
}

#ifndef RL_HOST
static MENU_UPDATE_FUNC(ettr_main_update)
{
    MENU_SET_VALUE("%s", ettr_trig == 0 ? "OFF" : ettr_trig == 1 ? "Half-press" : "Continuous");
    MENU_SET_RINFO("%s", ettr_busy ? "running" : "");
    MENU_SET_WARNING(MENU_WARN_INFO, "Last: %s", ettr_last);
}
static MENU_SELECT_FUNC(ettr_run_now) { ettr_run_req = 1; }
static struct menu_entry ettr_menu[] = {
    {
        .name     = "Auto ETTR (movie)",
        .priv     = &ettr_trig,
        .max      = 2,
        .choices  = CHOICES("OFF", "Half-press", "Continuous"),
        .update   = ettr_main_update,
        .help     = "Expose to the right with ISO, from the raw data: highlights just below clipping.",
        .help2    = "Movie mode, manual ISO. Never changes ISO while recording. Shutter/aperture untouched.",
        .children = (struct menu_entry[]) {
            {
                .name    = "Run now",
                .select  = ettr_run_now,
                .icon_type = IT_ACTION,
                .help    = "One ETTR run as soon as the ML menu is closed.",
            },
            {
                .name    = "Highlight ignore",
                .priv    = &ettr_ign_sel,
                .max     = 4,
                .choices = CHOICES("0%", "0.1%", "0.5%", "1%", "2%"),
                .help    = "Share of the image allowed to clip (light sources, specular highlights).",
            },
            {
                .name    = "Headroom",
                .priv    = &ettr_head_sel,
                .max     = 2,
                .choices = CHOICES("0.3 EV", "0.5 EV", "1.0 EV"),
                .help    = "How far below clipping the highlights are placed.",
            },
            {
                .name    = "Max ISO",
                .priv    = &ettr_max_sel,
                .max     = 4,
                .choices = CHOICES("800", "1600", "3200", "6400", "12800"),
                .help    = "ETTR never goes above this ISO (noise).",
            },
            MENU_EOL
        },
    },
};
static void ettr_init(void *unused) { menu_add("Expo", ettr_menu, COUNT(ettr_menu)); }
INIT_FUNC("r_ettr", ettr_init);
TASK_CREATE("r_ettr", ettr_task, 0, 0x1e, 0x1000);
#endif

#ifndef RL_HOST
TASK_CREATE("r_rawlv", rl_task, 0, 0x1e, 0x1000);
#endif
