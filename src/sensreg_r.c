/* R: Debug > Sensor regs log (EOS R 1.8.0, v4.58; v4.59 volatile-register filter; v4.60 Tap test). Personal diagnostic, not for upstream.
 *
 * Read-only survey for wider/taller 1:1 4K (crop_rec on the R). While "Watch" is ON a task polls, every 50 ms,
 * Canon's CMOS command buffers (the lists FUN_e06a19a4 sends to the sensor, 0xffffffff-terminated):
 *   - 10 LV buffers at *0x176a8 + 0x1bac + n * 0xac (FUN_e06cb1f4), the still buffer at *0x1769c + 0x28dc;
 *   - m_SensorMode (ImageSequence state 0xef24 + 0xc), Mem1Path size (0x133CC + 0x10/0x14/0x18);
 *   - SensorDrive state 0xcfd4 (+0x70 fps slot, +0x78..0x80 HD/line timing) and SetTgParam struct 0x5825c.
 * Whenever the buffers, the sensor mode or the MEM1 size change it keeps a snapshot (max SR_NSNAP). "Save log"
 * appends them to ML/LOGS/SENSREG.TXT. Diffing photo LV / FHD / 4K / magnify snapshots should show which
 * registers set the readout window and the line skipping (command word = reg << 22 | data).
 * Reads RAM only (the same reads v4.52 ran on the camera); no ROM, register or property writes.
 */
#include "dryos.h"
#include "menu.h"
#include "fio-ml.h"
#include "gui.h"
#include "propvalues.h"

#ifndef MEM
#define MEM(x) (*(volatile uint32_t *)(x))
#endif

#define SR_LOG      "ML/LOGS/SENSREG.TXT"
#define SR_STILLP   0x1769c
#define SR_LVP      0x176a8
#define SR_NLV      10
#define SR_LVW      (0xac / 4)
#define SR_STW      128
#define SR_STATE    0xef24
#define SR_M1P      0x133CC
#define SR_SDRV     0xcfd4
#define SR_SDW      40
#define SR_TG       0x5825c
#define SR_TGW      20
#define SR_NSNAP    48
#define SR_NBUF     (SR_NLV + 1)        /* LV lists + still list */
#define SR_VOLAT    3                   /* v4.59: a register that changed this often stops triggering snapshots */

struct sr_snap {
    uint32_t ms, mode, m1w, m1h, m1w18, lvbase, stbase;
    uint32_t sd[SR_SDW], tg[SR_TGW];
    uint32_t lv[SR_NLV][SR_LVW];
    uint32_t st[SR_STW];
};

static struct sr_snap *sr_snaps;          /* malloc'd at the first Watch ON (~100 KB): bss would cost Canon heap */
static volatile int sr_n, sr_lost, sr_polls, sr_bad_ptr;
static volatile int sr_run, sr_task_alive;
static int sr_saved_total;

static int sr_ram_ok(uint32_t a) { return a >= 0x1000 && a < 0x80000000 && !(a & 3); }

static uint32_t sr_fnv(uint32_t h, uint32_t v)
{
    for (int i = 0; i < 4; i++) { h ^= (v >> (i * 8)) & 0xff; h *= 16777619u; }
    return h;
}

/* read everything into s; returns the change hash */
static uint32_t sr_read(struct sr_snap *s)
{
    uint32_t h = 2166136261u;
    s->ms = get_ms_clock();
    s->mode = *(volatile uint32_t *)(SR_STATE + 0xc);
    s->m1w = *(volatile uint32_t *)(SR_M1P + 0x10);
    s->m1h = *(volatile uint32_t *)(SR_M1P + 0x14);
    s->m1w18 = *(volatile uint32_t *)(SR_M1P + 0x18);
    h = sr_fnv(h, s->mode); h = sr_fnv(h, s->m1w); h = sr_fnv(h, s->m1h);
    for (int i = 0; i < SR_SDW; i++) s->sd[i] = *(volatile uint32_t *)(SR_SDRV + i * 4);
    for (int i = 0; i < SR_TGW; i++) s->tg[i] = *(volatile uint32_t *)(SR_TG + i * 4);
    for (int i = 0x70 / 4; i <= 0x80 / 4; i++) h = sr_fnv(h, s->sd[i]);    /* fps slot + line timing only */

    uint32_t p = *(volatile uint32_t *)SR_LVP;
    s->lvbase = p;
    for (int n = 0; n < SR_NLV; n++)
    {
        int end = 0;
        for (int i = 0; i < SR_LVW; i++)
        {
            uint32_t v = 0;
            if (!end && sr_ram_ok(p)) v = *(volatile uint32_t *)(p + 0x1bac + n * 0xac + i * 4);
            s->lv[n][i] = v;
            if (v == 0xffffffff) end = 1;
            h = sr_fnv(h, v);
        }
    }
    uint32_t q = *(volatile uint32_t *)SR_STILLP;
    s->stbase = q;
    int end = 0;
    for (int i = 0; i < SR_STW; i++)
    {
        uint32_t v = 0;
        if (!end && sr_ram_ok(q)) v = *(volatile uint32_t *)(q + 0x28dc + i * 4);
        s->st[i] = v;
        if (v == 0xffffffff) end = 1;
        h = sr_fnv(h, v);
    }
    if (!sr_ram_ok(p) || !sr_ram_ok(q)) sr_bad_ptr++;
    return h;
}

/* v4.59: v4.58 filled all 40 snapshots in 12 s of photo LV (4098 lost): AE rewrites regs 0x29/0x50/0x80 by +-1
 * every few frames. Compare with the last KEPT snapshot and count changes per (list, register): a register that has
 * changed SR_VOLAT times stops triggering snapshots (its value is still in every snapshot; Save lists them). */
static uint8_t *sr_cnt;                   /* [SR_NBUF][1024] change counts, malloc'd with the snapshots */
static volatile int sr_quiet;             /* polls with only volatile changes */

static const uint32_t *sr_list(const struct sr_snap *s, int b, int *n)
{
    if (b < SR_NLV) { *n = SR_LVW; return s->lv[b]; }
    *n = SR_STW; return s->st;
}

static int sr_worthy(const struct sr_snap *ref, const struct sr_snap *cur)
{
    if (ref->mode != cur->mode || ref->m1w != cur->m1w || ref->m1h != cur->m1h || ref->m1w18 != cur->m1w18) return 1;
    for (int i = 0x70 / 4; i <= 0x80 / 4; i++) if (ref->sd[i] != cur->sd[i]) return 1;
    int worthy = 0;
    for (int b = 0; b < SR_NBUF; b++)
    {
        int n;
        const uint32_t *x = sr_list(ref, b, &n), *y = sr_list(cur, b, &n);
        for (int i = 0; i < n; i++)
        {
            if (x[i] == y[i]) continue;
            int reg = (y[i] == 0xffffffff || x[i] == 0xffffffff || (x[i] >> 22) != (y[i] >> 22)) ? 0x3ff : (y[i] >> 22);
            uint8_t *c = &sr_cnt[b * 1024 + reg];
            if (reg == 0x3ff || *c < SR_VOLAT) worthy = 1;      /* list layout change: always keep */
            if (*c < 255) (*c)++;
        }
    }
    return worthy;
}

static void sr_task(void *unused)
{
    sr_task_alive = 1;
    struct sr_snap *tmp = &sr_snaps[SR_NSNAP];       /* scratch */
    struct sr_snap *ref = &sr_snaps[SR_NSNAP + 1];   /* last kept state */
    int have_ref = 0;
    while (sr_run)
    {
        sr_read(tmp);
        sr_polls++;
        if (!have_ref || sr_worthy(ref, tmp))
        {
            memcpy(ref, tmp, sizeof(*tmp)); have_ref = 1;
            int i = sr_n;
            if (i < SR_NSNAP) { memcpy(&sr_snaps[i], tmp, sizeof(*tmp)); sr_n = i + 1; }
            else sr_lost++;
        }
        else sr_quiet++;
        msleep(50);
    }
    sr_task_alive = 0;
}

static void tp_save_log(void);
static volatile int tp_running;
static int tp_nev_get(void);

/* ---- log writing (menu task) ---- */
#define SR_BUF 4096
static char *sr_buf;
static int sr_len;
static FILE *sr_f;
static void sr_flush(void)
{
    if (sr_len && sr_f) FIO_WriteFile(sr_f, sr_buf, sr_len);
    sr_len = 0;
}
static void sr_printf(const char *fmt, ...)
{
    if (sr_len > SR_BUF - 200) sr_flush();
    va_list ap;
    va_start(ap, fmt);
    sr_len += vsnprintf(sr_buf + sr_len, SR_BUF - sr_len, fmt, ap);
    va_end(ap);
}
static void sr_words(const char *tag, const uint32_t *w, int n, int stop_ff)
{
    int last = n;
    if (stop_ff) { for (int i = 0; i < n; i++) if (w[i] == 0xffffffff) { last = i + 1; break; } }
    else { while (last > 0 && w[last - 1] == 0) last--; }
    sr_printf("  %s (%d):", tag, last);
    for (int i = 0; i < last; i++)
    {
        if (i && i % 8 == 0) sr_printf("\n     ");
        sr_printf(" %08x", (int)w[i]);
    }
    sr_printf("\n");
}

static int sr_save(void)
{
    if (!sr_buf && !(sr_buf = malloc(SR_BUF))) return -1;
    sr_f = FIO_CreateFileOrAppend(SR_LOG);
    if (!sr_f) return -2;
    int n = sr_n;
    sr_printf("=== sensreg v4.59 save at %d ms: %d snapshots (%d lost), %d polls (%d volatile-only), bad pointers %d\n",
              (int)get_ms_clock(), n, sr_lost, sr_polls, sr_quiet, sr_bad_ptr);
    if (sr_cnt)
    {
        sr_printf("  changed registers (list:reg=count, >= %d = volatile):", SR_VOLAT);
        int k = 0;
        for (int b = 0; b < SR_NBUF; b++)
            for (int r = 0; r < 1024; r++)
                if (sr_cnt[b * 1024 + r])
                {
                    if (k++ % 8 == 0) sr_printf("\n    ");
                    sr_printf(" %d:%x=%d", b, r, sr_cnt[b * 1024 + r]);
                }
        sr_printf("\n");
    }
    for (int k = 0; k < n; k++)
    {
        struct sr_snap *s = &sr_snaps[k];
        sr_printf("--- snap %d t=%d ms: m_SensorMode %d, MEM1 %dx%d (w18 %d), sdrv slot %d HD %d/%d/%d, LV %x still %x\n",
                  sr_saved_total + k, (int)s->ms, (int)s->mode, (int)s->m1w, (int)s->m1h, (int)s->m1w18,
                  (int)s->sd[0x70 / 4], (int)s->sd[0x78 / 4], (int)s->sd[0x7c / 4], (int)s->sd[0x80 / 4],
                  (int)s->lvbase, (int)s->stbase);
        sr_words("sdrv cfd4", s->sd, SR_SDW, 0);
        sr_words("tg 5825c", s->tg, SR_TGW, 0);
        for (int b = 0; b < SR_NLV; b++)
        {
            char t[8];
            snprintf(t, sizeof(t), "lv%d", b);
            sr_words(t, s->lv[b], SR_LVW, 1);
        }
        sr_words("still", s->st, SR_STW, 1);
        sr_flush();
    }
    sr_flush();
    tp_save_log();
    FIO_CloseFile(sr_f);
    sr_f = 0;
    sr_saved_total += n;
    sr_n = 0; sr_lost = 0;
    return n;
}


/* ---- v4.60 Tap test (4K): does a LiveView path rebuild keep the recorder's 10-bit MEM1 setup? ----
 * Taller-4K blocker (lv-survey stage 4): MEM1 0xFB9D90 is exactly 4256x2214 at 14-bit with live YUV buffers behind
 * it, so a taller frame must never arrive at 14-bit. This turns on lv_save_raw + 10-bit packing exactly like the
 * recorder (rectest_r.c rt_pack_apply), then for 30 s logs every change of the Mem1Path fields, the pack unit
 * register and m_SensorMode while the user enters/leaves magnify (a SAP path rebuild, FUN_e0637bd8, which rewrites
 * Mem1Path +0x18 with the record width). Safety reflex: if +0x18 no longer matches the 10-bit width while f2 is still
 * 10-bit, put Canon's 14-bit defaults back at once (logged). Magnify is smaller than 4K, so no outcome can overrun
 * the buffer. Restores everything and lv_save_raw(0) at the end; opening the ML menu cancels. */
#define TP_PATH     0x133CC
#define TP_UNIT     0xD0422224u
#define TP_BUF_UNC  0x68EEA940u
#define TP_NF       16
#define TP_NEV      240
#define TP_MS       30000
static const uint16_t tp_off[TP_NF] = { 0x00, 0x10, 0x14, 0x18, 0x1c, 0x28, 0x50, 0x58, 0x64, 0x68, 0x6c, 0x70, 0x74,
                                        0xffff /* unit reg */, 0xfffe /* m_SensorMode */, 0xfffd /* stall */ };
static struct { uint32_t ms; uint8_t f; uint32_t v; } *tp_ev;
static volatile int tp_nev, tp_lost, tp_reflex, tp_cancel;
static int tp_nev_get(void) { return tp_nev; }
static uint32_t tp_last[TP_NF];
static char tp_result[64];

static uint32_t tp_read(int f, uint32_t stall)
{
    uint16_t o = tp_off[f];
    if (o == 0xffff) return *(volatile uint32_t *)TP_UNIT;
    if (o == 0xfffe) return *(volatile uint32_t *)(SR_STATE + 0xc);
    if (o == 0xfffd) return stall;
    return *(volatile uint32_t *)(TP_PATH + o);
}
static void tp_note(int f, uint32_t v)
{
    if (v == tp_last[f]) return;
    tp_last[f] = v;
    int i = tp_nev;
    if (i < TP_NEV) { tp_ev[i].ms = get_ms_clock(); tp_ev[i].f = f; tp_ev[i].v = v; tp_nev = i + 1; }
    else tp_lost++;
}
static uint32_t tp_rowsum(void)          /* one MEM1 line from the middle: does the DMA still write frames? */
{
    volatile uint32_t *r = (volatile uint32_t *)(TP_BUF_UNC + 1100 * 5320);
    uint32_t h = 0;
    for (int i = 0; i < 256; i += 4) h = h * 31 + r[i];
    return h;
}

static void tp_task(void *unused)
{
    uint32_t orig[5], w0 = 0, wset = 0;
    int raw = 0, applied = 0;
    tp_nev = tp_lost = tp_reflex = tp_cancel = 0;
    for (int k = 0; k < 100 && gui_menu_shown(); k++) msleep(100);     /* wait for the ML menu to close */
    if (gui_menu_shown()) { snprintf(tp_result, sizeof(tp_result), "menu not closed: not started"); goto out; }
    if (!lv) { snprintf(tp_result, sizeof(tp_result), "no live view: not started"); goto out; }
    call("lv_save_raw", 1); raw = 1;
    msleep(500);
    uint32_t w = MEM(TP_PATH + 0x10), h = MEM(TP_PATH + 0x14);
    for (int i = 0; i < 5; i++) orig[i] = MEM(TP_PATH + 0x64 + 4 * i);
    w0 = MEM(TP_PATH + 0x18);
    for (int f = 0; f < TP_NF; f++) { tp_last[f] = 0xdeadbeef; tp_note(f, tp_read(f, 0)); }
    if (w != 4256 || h != 2214)
    { snprintf(tp_result, sizeof(tp_result), "MEM1 %dx%d, not 4K 4256x2214: set Movie 4K", (int)w, (int)h); goto out; }
    if (orig[0] != 0 || orig[1] != 1 || orig[2] != 2 || orig[3] != 1 || orig[4] != 0 || w0 != w || MEM(TP_PATH) >= 0x14)
    { snprintf(tp_result, sizeof(tp_result), "Mem1Path not Canon's default: not touched"); goto out; }
    wset = ((w * 10 / 8) * 8 + 13) / 14;                         /* recorder's 10-bit width field (3040 at 4256) */
    if (((wset * 14) >> 3) != w * 10 / 8) { snprintf(tp_result, sizeof(tp_result), "no 10-bit width field"); goto out; }
    uint32_t old = cli();
    MEM(TP_PATH + 0x18) = wset; MEM(TP_PATH + 0x64 + 8) = 0; MEM(TP_PATH + 0x64 + 12) = 0;   /* 10-bit, ML order */
    sei(old);
    applied = 1;
    NotifyBox(3000, "Tap test running: magnify in/out now (30 s)");
    uint32_t t0 = get_ms_clock(), last_sum = tp_rowsum(), last_change = t0;
    while (get_ms_clock() - t0 < TP_MS)
    {
        if (gui_menu_shown()) { tp_cancel = 1; break; }
        uint32_t now = get_ms_clock(), sum = tp_rowsum();
        if (sum != last_sum) { last_sum = sum; last_change = now; }
        uint32_t stall = now - last_change > 1000;
        for (int f = 0; f < TP_NF; f++) tp_note(f, tp_read(f, stall));
        /* reflex: width field rewritten by Canon while f2 still says 10-bit -> back to 14-bit defaults */
        if (MEM(TP_PATH + 0x18) != wset && MEM(TP_PATH + 0x64 + 8) == 0)
        {
            uint32_t o2 = cli();
            MEM(TP_PATH + 0x64 + 8) = orig[2]; MEM(TP_PATH + 0x64 + 12) = orig[3];
            sei(o2);
            tp_reflex++; applied = 0;
            for (int f = 0; f < TP_NF; f++) tp_note(f, tp_read(f, stall));
        }
        /* reflex 2: Canon put the pack cfg back to 14-bit but left our narrowed width field -> width = current w */
        else if (MEM(TP_PATH + 0x18) == wset && MEM(TP_PATH + 0x64 + 8) != 0)
        {
            uint32_t o2 = cli();
            MEM(TP_PATH + 0x18) = MEM(TP_PATH + 0x10);
            sei(o2);
            tp_reflex += 0x100; applied = 0;
            for (int f = 0; f < TP_NF; f++) tp_note(f, tp_read(f, stall));
        }
        msleep(20);
    }
    snprintf(tp_result, sizeof(tp_result), "%s: %d events, reflex %x", tp_cancel ? "cancelled" : "done", tp_nev, tp_reflex);
out:
    if (applied && MEM(TP_PATH + 0x18) == wset)
    {
        uint32_t o3 = cli();
        MEM(TP_PATH + 0x18) = w0; MEM(TP_PATH + 0x64 + 8) = orig[2]; MEM(TP_PATH + 0x64 + 12) = orig[3];
        sei(o3);
    }
    else if (applied)
    {
        uint32_t o3 = cli();
        MEM(TP_PATH + 0x64 + 8) = orig[2]; MEM(TP_PATH + 0x64 + 12) = orig[3];
        sei(o3);
    }
    msleep(100);
    if (raw) call("lv_save_raw", 0);
    msleep(200);
    tp_note(13, tp_read(13, 0)); tp_note(3, tp_read(3, 0)); tp_note(10, tp_read(10, 0));
    tp_running = 0;
    NotifyBox(5000, "Tap test %s. Open Sensor regs log > Save log.", tp_result);
}

static void tp_select(void *priv, int delta)
{
    if (tp_running || sr_run) { NotifyBox(2000, "Busy: stop Watch / wait for the tap test"); return; }
    if (!tp_ev && !(tp_ev = malloc(TP_NEV * sizeof(*tp_ev)))) { NotifyBox(2000, "not enough memory"); return; }
    tp_running = 1;
    tp_result[0] = 0;
    task_create("sr_tap", 0x1e, 0x1000, tp_task, 0);
    NotifyBox(4000, "Tap test: close the ML menu to start");
}

static void tp_save_log(void)
{
    if (!tp_ev || !tp_nev) return;
    static const char *nm[TP_NF] = { "m1 mode", "m1 w", "m1 h", "m1 +18 width", "m1 +1c", "m1 +28", "m1 +50", "m1 +58 addr",
                                     "pack f0", "pack f1", "pack f2", "pack f3", "pack f4", "unit reg", "m_SensorMode", "stall" };
    sr_printf("=== tap test v4.60: %s (%d events, %d lost); fields: Mem1Path 0x133CC, pack unit %x\n", tp_result,
              tp_nev, tp_lost, (int)TP_UNIT);
    int t0 = tp_ev[0].ms;
    for (int i = 0; i < tp_nev; i++)
    {
        sr_printf("  +%d ms  %s = %x (%d)\n", (int)tp_ev[i].ms - t0, nm[tp_ev[i].f], (int)tp_ev[i].v, (int)tp_ev[i].v);
        if (i % 32 == 31) sr_flush();
    }
    sr_flush();
    tp_nev = 0;
}

/* ---- menu ---- */
static void sr_watch_toggle(void *priv, int delta)
{
    if (tp_running) { NotifyBox(2000, "Tap test running"); return; }
    if (sr_run) { sr_run = 0; NotifyBox(2000, "Sensor regs: watch OFF (%d snapshots kept)", sr_n); return; }
    if (sr_task_alive) { NotifyBox(2000, "Sensor regs: still stopping, try again"); return; }
    if (!sr_cnt) sr_cnt = malloc(SR_NBUF * 1024);
    if (!sr_snaps) sr_snaps = malloc((SR_NSNAP + 2) * sizeof(struct sr_snap));
    if (!sr_snaps || !sr_cnt)
    {
        NotifyBox(3000, "Sensor regs: not enough memory");
        return;
    }
    sr_polls = 0; sr_bad_ptr = 0; sr_quiet = 0;
    memset(sr_cnt, 0, SR_NBUF * 1024);
    sr_run = 1;
    task_create("sensreg", 0x1e, 0x1000, sr_task, 0);
    NotifyBox(2000, "Sensor regs: watch ON");
}

static void sr_save_select(void *priv, int delta)
{
    if (sr_run)                             /* stop the watch first: the task appends to sr_snaps */
    {
        sr_run = 0;
        for (int i = 0; i < 40 && sr_task_alive; i++) msleep(25);
        if (sr_task_alive) { NotifyBox(3000, "Sensor regs: watch still stopping, Save again"); return; }
    }
    if (tp_running) { NotifyBox(2000, "Tap test still running"); return; }
    if (!sr_snaps && !tp_nev_get()) { NotifyBox(2000, "Sensor regs: nothing recorded"); return; }
    int r = sr_save();
    if (r >= 0) NotifyBox(3000, "Saved %d snapshots to " SR_LOG, r);
    else NotifyBox(3000, "Save failed (%d)", r);
}

static MENU_UPDATE_FUNC(sr_update)
{
    if (sr_run) MENU_SET_VALUE("ON, %d snaps", sr_n);
    else MENU_SET_VALUE("OFF, %d snaps", sr_n);
    if (sr_lost) MENU_SET_WARNING(MENU_WARN_INFO, "%d changes not kept (full): Save log, then go on.", sr_lost);
}

static struct menu_entry sr_menu[] = {
    {
        .name   = "Sensor regs log",
        .select = menu_open_submenu,
        .update = sr_update,
        .help   = "Read-only: records Canon's sensor command lists whenever the LV mode changes.",
        .help2  = "For the wider-4K research. Log: " SR_LOG ". Always OFF at power-on.",
        .children = (struct menu_entry[]) {
            {
                .name   = "Watch",
                .select = sr_watch_toggle,
                .update = sr_update,
                .help   = "SET: ON/OFF. ON keeps a snapshot at every change (max 48).",
            },
            {
                .name   = "Tap test (4K)",
                .select = tp_select,
                .help   = "Movie 4K only: 10-bit raw tap like the recorder, then logs what magnify in/out",
                .help2  = "does to it for 30 s. Opening the ML menu cancels. Then Save log.",
            },
            {
                .name   = "Save log",
                .select = sr_save_select,
                .help   = "Stops Watch, appends the snapshots to " SR_LOG " and clears them.",
            },
            MENU_EOL
        },
    },
};

static void sr_init(void *unused)
{
    menu_add("Debug", sr_menu, COUNT(sr_menu));
}

INIT_FUNC("sensreg_r", sr_init);
