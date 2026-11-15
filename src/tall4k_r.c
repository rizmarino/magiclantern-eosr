/* R: Debug > Taller 4K (test) (EOS R 1.8.0, v4.61; v4.62 RAM-list check tolerates calibrated values). Personal diagnostic, not for upstream.
 *
 * First sensor-window write: 4K mode +40 rows (MEM1 4256x2214 -> 4256x2254), notes lv-survey stages 2-5.
 *   1. Checks: movie 4K live view (MEM1 4256x2214 after lv_save_raw), Mem1Path pack cfg at Canon's default, the
 *      SensorDrive table pointer *(0xcfd4 + 0x6c) maps the FROM blob (ROM 0xE1C60000) into RAM (base = ptr - 0x15CE0,
 *      first 64 table bytes equal ROM), the 4K full init list (ROM 0xE1C62028, 113 words) and per-frame LV template
 *      (ROM 0xE1C6288C, 38 words) are word-for-word equal in RAM, regs 0x69 / 0x97 hold 0x6aa / 0x453, and the
 *      MEM1 size record (ROM 0xE0F2E608) is {4256, 2214}. Any mismatch: nothing is written.
 *   2. lv_save_raw(1) + 10-bit packing like the recorder (Mem1Path +0x18 = 3040, f2 = f3 = 0).
 *   3. Patch: record h 2214 -> 2254 (shared ROM remap page 0xE0F2E000), RAM lists 0x69 += 20 (bottom edge, 2-row
 *      units), 0x97 += 20 (output rows / 2). The user then enters and leaves magnify, which rebuilds the 4K path.
 *   4. A 5 ms poll re-narrows +0x18 to 3040 whenever Canon rewrites it with 4256 (rebuild; tap test v4.60 showed port
 *      3 follows 100-200 ms later) and restores 14-bit when the width is anything else (magnify, no 10-bit width).
 *      Even a missed frame at 14-bit overruns MEM1 by only 40 x 7448 B, inside the small-YUV block behind it.
 *   5. Once back in 4K: logs Mem1Path size, live LV list 0x68/0x69 (written directly if the rebuild didn't take the
 *      template), frame interval, stalls; saves rows 2140..2253 and 1100..1103 to ML/LOGS/TALL4K.BIN.
 *   6. Reverts the record and lists, asks for one more magnify in/out (normal 4K path), restores the pack cfg and
 *      lv_save_raw(0). Breadcrumbs: every step is appended to ML/LOGS/TALL4K.TXT and the file closed at once.
 * Opening the ML menu cancels (reverts as in 6). */
#include "dryos.h"
#include "menu.h"
#include "fio-ml.h"
#include "gui.h"
#include "mem.h"
#include "propvalues.h"
#include "romremap_r.h"

#ifndef MEM
#define MEM(x) (*(volatile uint32_t *)(x))
#endif

#define T4_LOG      "ML/LOGS/TALL4K.TXT"
#define T4_BIN      "ML/LOGS/TALL4K.BIN"
#define T4_PATH     0x133CC
#define T4_STATE    0xef24              /* ImageSequence: +0xc m_SensorMode (12 = 4K, 13 = magnify) */
#define T4_SDRV     0xcfd4
#define T4_UNIT     0xD0422224u
#define T4_MEM1     0x68EEA940u
#define T4_LVP      0x176a8
#define T4_FROM     0xE1C60000u
#define T4_SDTBL    0xE1C75CE0u
#define T4_FULL     0xE1C62028u
#define T4_NFULL    113
#define T4_LVT      0xE1C6288Cu
#define T4_NLVT     38
#define T4_REC      0xE0F2E608u         /* MEM1 record 1 (4K): {w, h, 0, 0} */
#define T4_DROWS    40                  /* extra rows (sensor regs count 2-row units: +20) */
#define T4_W        4256
#define T4_H        2214
#define T4_WSET     3040                /* 10-bit width field: (3040 * 14) >> 3 = 5320 = 4256 * 10 / 8 */
#define T4_PITCH    5320

static volatile int t4_running;
static char t4_result[80];

static void t4_log(const char *fmt, ...)
{
    char b[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    FILE *f = FIO_CreateFileOrAppend(T4_LOG);
    if (f) { FIO_WriteFile(f, b, n); FIO_CloseFile(f); }
}

static int t4_ram_ok(uint32_t a) { return a >= 0x40001000 && a < 0x7f000000 && !(a & 3); }

/* index of the word carrying reg r in a 0xffffffff-terminated list of n words, -1 if none */
static int t4_find(volatile uint32_t *l, int n, int r)
{
    for (int i = 0; i < n; i++) { if (l[i] == 0xffffffff) return -1; if ((l[i] >> 22) == (uint32_t)r) return i; }
    return -1;
}

/* write one list word through the cached alias and clean it, so cached and uncached readers both see it */
static void t4_put(volatile uint32_t *unc, uint32_t v)
{
    uint32_t *c = (uint32_t *)((uint32_t)unc & ~0x40000000);
    *c = v;
    romremap_flush(c, 4);
    if (*unc != v) *unc = v;
}

static volatile uint32_t *t4_live_lv7(void)
{
    uint32_t p = MEM(T4_LVP);
    if (p < 0x1000 || p >= 0x80000000 || (p & 3)) return 0;
    return (volatile uint32_t *)(p + 0x1bac + 7 * 0xac);
}

static uint32_t t4_rowsum(int row)
{
    volatile uint32_t *r = (volatile uint32_t *)(T4_MEM1 + row * T4_PITCH);
    uint32_t h = 0;
    for (int i = 0; i < 256; i += 4) h = h * 31 + r[i];
    return h;
}

/* keep the pack setup consistent with whatever width Canon just wrote; returns 1 if it acted */
static int t4_reflex(int *narrowed, int *reflex14)
{
    uint32_t w18 = MEM(T4_PATH + 0x18), f2 = MEM(T4_PATH + 0x6c);
    if (w18 == T4_W && f2 == 0)                         /* 4K rebuild: re-narrow for 10-bit */
    {
        uint32_t o = cli(); MEM(T4_PATH + 0x18) = T4_WSET; sei(o);
        (*narrowed)++;
        return 1;
    }
    if (w18 != T4_W && w18 != T4_WSET && f2 == 0)       /* other mode (magnify): no 10-bit width -> 14-bit */
    {
        uint32_t o = cli(); MEM(T4_PATH + 0x6c) = 2; MEM(T4_PATH + 0x70) = 1; sei(o);
        (*reflex14)++;
        return 1;
    }
    if (w18 == T4_W && f2 == 2)                         /* back in 4K at 14-bit (only 4K has w 4256): 10-bit */
    {
        uint32_t o = cli(); MEM(T4_PATH + 0x18) = T4_WSET; MEM(T4_PATH + 0x6c) = 0; MEM(T4_PATH + 0x70) = 0; sei(o);
        (*narrowed)++;
        return 1;
    }
    return 0;
}

/* wait for a magnify-in/-out cycle (m_SensorMode 13 then 12) while running the reflex; 0 = done, 1 = cancel, 2 = timeout */
static int t4_cycle(int timeout_ms, int *narrowed, int *reflex14)
{
    int seen13 = 0;
    uint32_t t0 = get_ms_clock();
    while (get_ms_clock() - t0 < (uint32_t)timeout_ms)
    {
        if (gui_menu_shown()) return 1;
        if (t4_reflex(narrowed, reflex14))
            t4_log("  +%d ms reflex: mode %d, +18 %d, f2 %d, m1 %dx%d\n", (int)(get_ms_clock() - t0),
                   (int)MEM(T4_STATE + 0xc), (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x6c),
                   (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14));
        int m = MEM(T4_STATE + 0xc);
        if (m == 13) seen13 = 1;
        if (seen13 && m == 12 && MEM(T4_PATH + 0x1c) != 1496 && MEM(T4_PATH + 0x14) != 1496)
        {
            /* back in 4K and port 3 reprogrammed: give it 1 s of reflexes to settle */
            uint32_t t1 = get_ms_clock();
            while (get_ms_clock() - t1 < 1000) { t4_reflex(narrowed, reflex14); msleep(5); }
            return 0;
        }
        msleep(5);
    }
    return 2;
}

static void t4_task(void *unused)
{
    int raw = 0, packed = 0, patched = 0, narrowed = 0, reflex14 = 0, rc;
    uint32_t base = 0, orig_pack[5] = {0}, w0 = 0;
    volatile uint32_t *full = 0, *lvt = 0;
    int i69f = -1, i97f = -1, i69t = -1;
    uint32_t o69f = 0, o97f = 0, o69t = 0, orec_h = 0;
    uint32_t *rec_ram = 0;
    t4_result[0] = 0;
    t4_log("=== taller 4K v4.62 test at %d ms\n", (int)get_ms_clock());
    for (int k = 0; k < 100 && gui_menu_shown(); k++) msleep(100);
    if (gui_menu_shown() || !lv) { snprintf(t4_result, sizeof(t4_result), "menu open or no LV: not started"); goto out; }

    /* 1. checks (read-only) */
    uint32_t sd = MEM(T4_SDRV + 0x6c);
    base = sd - (T4_SDTBL - T4_FROM);
    t4_log("sdrv table ptr %x -> FROM blob base %x\n", (int)sd, (int)base);
    if (!t4_ram_ok(sd) || !t4_ram_ok(base)) { snprintf(t4_result, sizeof(t4_result), "sdrv ptr %x unexpected", (int)sd); goto out; }
    for (int i = 0; i < 16; i++)
        if (MEM(sd + i * 4) != MEM(T4_SDTBL + i * 4))
        { snprintf(t4_result, sizeof(t4_result), "sdrv table != ROM at word %d", i); goto out; }
    full = (volatile uint32_t *)(base + (T4_FULL - T4_FROM));
    lvt = (volatile uint32_t *)(base + (T4_LVT - T4_FROM));
    /* v4.62: camera v4.61 found 13 of 151 words different (calibration written after boot?). Accept the RAM copy
     * when the layout is identical (same register in every word, same terminator), at most 16 values differ, and
     * the words this test patches equal ROM (checked below via the expected values). Log every difference. */
    int bad = 0, layout = 0;
    for (int k = 0; k < 2; k++)
    {
        volatile uint32_t *l = k ? lvt : full;
        uint32_t rom = k ? T4_LVT : T4_FULL;
        int n = k ? T4_NLVT : T4_NFULL;
        for (int i = 0; i <= n; i++)
        {
            uint32_t a = l[i], b = MEM(rom + i * 4);
            if (a == b) continue;
            if ((a >> 22) != (b >> 22) || i == n) layout++;
            else bad++;
            t4_log("  %s[%d] RAM %x ROM %x (reg %x)\n", k ? "template" : "full", i, (int)a, (int)b, (int)(b >> 22));
        }
    }
    t4_log("RAM lists: full %x, LV template %x: %d values differ from ROM, %d layout differences\n", (int)full, (int)lvt,
           bad, layout);
    if (layout || bad > 16)
    { snprintf(t4_result, sizeof(t4_result), "RAM lists != ROM (%d values, %d layout): not touched", bad, layout); goto out; }
    i69f = t4_find(full, T4_NFULL, 0x69); i97f = t4_find(full, T4_NFULL, 0x97); i69t = t4_find(lvt, T4_NLVT, 0x69);
    if (i69f < 0 || i97f < 0 || i69t < 0) { snprintf(t4_result, sizeof(t4_result), "regs not found"); goto out; }
    o69f = full[i69f]; o97f = full[i97f]; o69t = lvt[i69t];
    t4_log("full[%d] = %x, full[%d] = %x, template[%d] = %x\n", i69f, (int)o69f, i97f, (int)o97f, i69t, (int)o69t);
    if ((o69f & 0x3fffff) != 0x6aa || (o97f & 0x3fffff) != 0x453 || (o69t & 0x3fffff) != 0x6aa)
    { snprintf(t4_result, sizeof(t4_result), "reg values unexpected"); goto out; }
    if (MEM(T4_REC) != T4_W || MEM(T4_REC + 4) != T4_H) { snprintf(t4_result, sizeof(t4_result), "MEM1 record unexpected"); goto out; }
    rc = romremap_init();
    rec_ram = (uint32_t *)romremap_ram(T4_REC);
    t4_log("romremap %d (%s), record RAM %x\n", rc, romremap_stage(), (int)rec_ram);
    if (rc || !rec_ram) { snprintf(t4_result, sizeof(t4_result), "ROM remap failed (%d)", rc); goto out; }

    /* 2. raw tap + 10-bit */
    call("lv_save_raw", 1); raw = 1;
    msleep(500);
    for (int i = 0; i < 5; i++) orig_pack[i] = MEM(T4_PATH + 0x64 + 4 * i);
    w0 = MEM(T4_PATH + 0x18);
    t4_log("MEM1 %dx%d, +18 %d, mode %d, pack {%d,%d,%d,%d,%d}, m_SensorMode %d\n", (int)MEM(T4_PATH + 0x10),
           (int)MEM(T4_PATH + 0x14), (int)w0, (int)MEM(T4_PATH), (int)orig_pack[0], (int)orig_pack[1], (int)orig_pack[2],
           (int)orig_pack[3], (int)orig_pack[4], (int)MEM(T4_STATE + 0xc));
    if (MEM(T4_PATH + 0x10) != T4_W || MEM(T4_PATH + 0x14) != T4_H || w0 != T4_W || MEM(T4_STATE + 0xc) != 12)
    { snprintf(t4_result, sizeof(t4_result), "not movie 4K (MEM1 %dx%d)", (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14)); goto out; }
    if (orig_pack[0] != 0 || orig_pack[1] != 1 || orig_pack[2] != 2 || orig_pack[3] != 1 || orig_pack[4] != 0)
    { snprintf(t4_result, sizeof(t4_result), "pack cfg not Canon's default"); goto out; }
    uint32_t o = cli();
    MEM(T4_PATH + 0x18) = T4_WSET; MEM(T4_PATH + 0x6c) = 0; MEM(T4_PATH + 0x70) = 0;
    sei(o);
    packed = 1;
    msleep(300);
    t4_log("10-bit: MEM1 w %d, unit %x\n", (int)MEM(T4_PATH + 0x10), (int)MEM(T4_UNIT));

    /* 3. patch */
    t4_log("patching: record h %d -> %d, 0x69 %x -> %x, 0x97 %x -> %x\n", T4_H, T4_H + T4_DROWS, (int)o69f,
           (int)(o69f + T4_DROWS / 2), (int)o97f, (int)(o97f + T4_DROWS / 2));
    orec_h = rec_ram[1];
    rec_ram[1] = T4_H + T4_DROWS; romremap_flush(&rec_ram[1], 4);
    t4_put(&full[i69f], o69f + T4_DROWS / 2);
    t4_put(&full[i97f], o97f + T4_DROWS / 2);
    t4_put(&lvt[i69t], o69t + T4_DROWS / 2);
    patched = 1;
    t4_log("patched: record reads %d, full 0x69 %x, 0x97 %x, template 0x69 %x\n", (int)MEM(T4_REC + 4),
           (int)full[i69f], (int)full[i97f], (int)lvt[i69t]);
    NotifyBox(5000, "Taller 4K: now magnify x5, then exit magnify");

    /* 4. rebuild via magnify in/out */
    rc = t4_cycle(30000, &narrowed, &reflex14);
    t4_log("cycle 1: %s, narrowed %d, 14-bit reflex %d\n", rc == 0 ? "done" : rc == 1 ? "cancelled" : "timeout",
           narrowed, reflex14);
    if (rc) { snprintf(t4_result, sizeof(t4_result), "%s before the rebuild", rc == 1 ? "cancelled" : "no magnify seen"); goto revert; }

    /* 5. measure */
    uint32_t m1w = MEM(T4_PATH + 0x10), m1h = MEM(T4_PATH + 0x14);
    volatile uint32_t *l7 = t4_live_lv7();
    int i68 = l7 ? t4_find(l7, 43, 0x68) : -1, i69 = l7 ? t4_find(l7, 43, 0x69) : -1;
    t4_log("after rebuild: MEM1 %dx%d (+18 %d, +1c %d), f2 %d, unit %x, live lv7 0x68 %x 0x69 %x\n", (int)m1w, (int)m1h,
           (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x1c), (int)MEM(T4_PATH + 0x6c), (int)MEM(T4_UNIT),
           i68 >= 0 ? (int)l7[i68] : -1, i69 >= 0 ? (int)l7[i69] : -1);
    if (i69 >= 0 && (l7[i69] & 0x3fffff) == (o69t & 0x3fffff))
    {
        l7[i69] = o69t + T4_DROWS / 2;
        t4_log("live lv7 0x69 still Canon's: written directly -> %x\n", (int)l7[i69]);
    }
    /* frame interval and stall check from row 1100, 1.5 s */
    {
        uint32_t t0 = get_ms_clock(), last = t4_rowsum(1100), lt = t0, nchg = 0, maxgap = 0;
        while (get_ms_clock() - t0 < 1500)
        {
            t4_reflex(&narrowed, &reflex14);
            uint32_t s = t4_rowsum(1100), now = get_ms_clock();
            if (s != last) { if (now - lt > maxgap) maxgap = now - lt; last = s; lt = now; nchg++; }
            msleep(2);
        }
        t4_log("row 1100 changed %d times in 1.5 s (max gap %d ms)\n", (int)nchg, (int)maxgap);
    }
    if (m1w == T4_WSET && m1h == T4_H + T4_DROWS && MEM(T4_PATH + 0x6c) == 0)
    {
        int nrows = (T4_H + T4_DROWS) - 2140 + 4;
        uint8_t *buf = fio_malloc(nrows * T4_PITCH + 64);
        if (buf)
        {
            uint32_t hdr[16] = { 0x4b344c54 /* "TL4K" */, T4_W, T4_H + T4_DROWS, 10, T4_PITCH, 2140, nrows - 4, 1100, 4 };
            memcpy(buf, hdr, 64);
            for (int r = 0; r < nrows - 4; r++) memcpy(buf + 64 + r * T4_PITCH, (void *)(T4_MEM1 + (2140 + r) * T4_PITCH), T4_PITCH);
            for (int r = 0; r < 4; r++) memcpy(buf + 64 + (nrows - 4 + r) * T4_PITCH, (void *)(T4_MEM1 + (1100 + r) * T4_PITCH), T4_PITCH);
            FILE *f = FIO_CreateFile(T4_BIN);
            if (f) { FIO_WriteFile(f, buf, 64 + nrows * T4_PITCH); FIO_CloseFile(f); }
            fio_free(buf);
            t4_log("saved rows 2140..%d + 1100..1103 to " T4_BIN "\n", T4_H + T4_DROWS - 1);
        }
        else t4_log("no memory for the row dump\n");
        snprintf(t4_result, sizeof(t4_result), "MEM1 %dx%d at 10-bit: rows saved", T4_W, (int)m1h);
    }
    else snprintf(t4_result, sizeof(t4_result), "after rebuild MEM1 %dx%d, f2 %d", (int)m1w, (int)m1h, (int)MEM(T4_PATH + 0x6c));

revert:
    /* 6. revert the patch, then one more rebuild back to the normal 4K path */
    if (patched)
    {
        rec_ram[1] = orec_h; romremap_flush(&rec_ram[1], 4);
        t4_put(&full[i69f], o69f); t4_put(&full[i97f], o97f); t4_put(&lvt[i69t], o69t);
        patched = 0;
        t4_log("reverted: record %d, full 0x69 %x 0x97 %x, template 0x69 %x\n", (int)MEM(T4_REC + 4), (int)full[i69f],
               (int)full[i97f], (int)lvt[i69t]);
        if (MEM(T4_PATH + 0x1c) != T4_H)
        {
            NotifyBox(5000, "Taller 4K: magnify x5 and exit once more (restores normal 4K)");
            rc = t4_cycle(30000, &narrowed, &reflex14);
            t4_log("cycle 2: %s, MEM1 %dx%d\n", rc == 0 ? "done" : rc == 1 ? "cancelled" : "timeout",
                   (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14));
        }
    }
out:
    if (raw && MEM(T4_PATH + 0x1c) != T4_H && MEM(T4_PATH + 0x1c) != 1496)
    {
        /* path still tall: stop the tap first, so no 14-bit frame of the taller size reaches MEM1 */
        call("lv_save_raw", 0); raw = 0;
        t4_log("path still tall (+1c %d): tap stopped before restoring 14-bit; power-cycle advised\n", (int)MEM(T4_PATH + 0x1c));
    }
    if (packed)
    {
        /* pack back to Canon's default; width field back to what Canon set for this mode */
        uint32_t o2 = cli();
        if (MEM(T4_PATH + 0x18) == T4_WSET) MEM(T4_PATH + 0x18) = T4_W;
        MEM(T4_PATH + 0x6c) = orig_pack[2]; MEM(T4_PATH + 0x70) = orig_pack[3];
        sei(o2);
        msleep(100);
    }
    if (raw) call("lv_save_raw", 0);
    t4_log("end: %s; narrowed %d, 14-bit reflex %d; MEM1 %dx%d, +18 %d, f2 %d\n", t4_result, narrowed, reflex14,
           (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14), (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x6c));
    t4_running = 0;
    NotifyBox(8000, "Taller 4K: %s", t4_result);
}

static void t4_select(void *priv, int delta)
{
    if (t4_running) { NotifyBox(2000, "Taller 4K test already running"); return; }
    t4_running = 1;
    task_create("tall4k", 0x1d, 0x1000, t4_task, 0);
    NotifyBox(4000, "Taller 4K: close the ML menu to start");
}

static MENU_UPDATE_FUNC(t4_update)
{
    if (t4_running) MENU_SET_VALUE("RUNNING");
    else if (t4_result[0]) MENU_SET_VALUE("last: %s", t4_result);
    else MENU_SET_VALUE("OFF");
}

static struct menu_entry t4_menu[] = {
    {
        .name   = "Taller 4K (test)",
        .select = t4_select,
        .update = t4_update,
        .help   = "Movie 4K 23.98p: first sensor-window test, +40 rows. Follow the on-screen prompts",
        .help2  = "(magnify in/out). Reverts at the end; ML menu cancels. Log ML/LOGS/TALL4K.TXT.",
    },
};

static void t4_init(void *unused)
{
    menu_add("Debug", t4_menu, COUNT(t4_menu));
}

INIT_FUNC("tall4k_r", t4_init);
