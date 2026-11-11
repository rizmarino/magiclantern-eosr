/* EOS R: dual ISO, first camera test (v4.51). Shoot > Dual ISO (test).
 *
 * Static RE (ghidra notes "Dual ISO, stage 1/2"): Canon's still readouts (M_/E_/SRS_SetReadOut -> FUN_e06a0dca)
 * and LiveView (FUN_e06a0696) copy one ISO column of m_AnalogIsoGainFix (ROM 0xE0B40A58 + mode * 0x33c, 23 CMOS
 * registers x 9 ISO steps, stride 0x24) into the sensor command buffer. Register 0x2D (row 1) is the analog ISO:
 * four identical 4-bit fields (bits 4-19) stepping 0..7 per stop, ISO 100..12800 (ISO 25600 = 12800 + digital).
 * Same shape as the 200D / 6D2, whose dual ISO keeps some fields at the base ISO and sets the rest to the second
 * one, so alternate row groups read out at different ISOs.
 *
 * This test sets the chosen fields to the second ISO in every ISO column of all 9 modes, through the shared ROM
 * remap (romremap_r.c). Which field drives which rows is the open question: "All 4" is the control (the whole image
 * should come out at the second ISO). Still capture rebuilds the commands per shot; LiveView only when the ISO or
 * mode changes. OFF restores Canon's values. Not saved: always OFF at power-on. Log: ML/DUALISO.TXT. */

#include "dryos.h"
#include "menu.h"
#include "config.h"
#include "fio-ml.h"
#include "romremap_r.h"

#define DI_TBL      0xE0B40A58          /* m_AnalogIsoGainFix, mode 0 */
#define DI_MSTRIDE  0x33c
#define DI_NROWS    2                   /* patchable rows: 1 = CMOS reg 0x2D, 3 = reg 0x2F */
#define DI_NMODES   9
#define DI_NISO     9
#define DI_STATE    0xef24              /* ImageSequence state: +0 ISO index (byte), +4 short, +0xc m_SensorMode, +0x48 */
#define DI_LOG      "ML/DUALISO.TXT"

/* v4.53 (camera 2026-11-11, v4.52): Canon's still command buffer carried every patched reg 0x2D word; field 3 (bits
 * 12-15) alone = whole-frame analog gain (+2.87 stops for step 4), fields 1, 2, 4 no visible effect. Now a list of
 * candidates, one experiment each: an enable bit above the second field (as on the 5D2: bit 20/21 of the 22-bit
 * data, always 0 in Canon's table), and one of the four look-alike registers 0x2F-0x32. */
static const uint8_t di_rows[DI_NROWS] = { 1, 3 };
static const uint8_t di_regs[DI_NROWS] = { 0x2D, 0x2F };
struct di_cand { uint8_t row; uint32_t isof; uint32_t bits; int8_t from_col; };
static const struct di_cand di_cands[] = {
    { 0, 0x0F000, 0,        -1 },       /* 0x2D field 3 = 2nd ISO: whole-frame control */
    { 0, 0xF0000, 0x100000, -1 },       /* field 4 + bit 20 */
    { 0, 0xF0000, 0x200000, -1 },       /* field 4 + bit 21 */
    { 0, 0x00FF0, 0x100000, -1 },       /* fields 1+2 + bit 20 */
    { 0, 0x00FF0, 0x200000, -1 },       /* fields 1+2 + bit 21 */
    { 0, 0xF0FF0, 0x300000, -1 },       /* fields 1, 2, 4 + bits 20+21 */
    { 1, 0,       0,         8 },       /* 0x2F = its ISO 25600 value (2222) */
};
static int di_on, di_err, di_applied_iso = -1, di_applied_pat = -1, di_bad, di_skipped;
static CONFIG_INT("dualiso.test.iso", di_iso, 4);       /* 0..7 = ISO 100..12800 */
static CONFIG_INT("dualiso.test.cand", di_pat, 0);
static uint32_t di_orig[DI_NROWS][DI_NMODES][DI_NISO];
static int di_have_orig;

static uint32_t di_rom_r(int r, int m, int c) { return DI_TBL + m * DI_MSTRIDE + di_rows[r] * 0x24 + c * 4; }
static uint32_t di_rom(int m, int c) { return di_rom_r(0, m, c); }

/* v4.52 (camera 2026-11-11: patched table read back fine, but 0 effect on CR3s/LV): also watch what Canon puts into
 * the command buffers the CMOS DMA sends (FUN_e06a19a4 walks them up to 0xffffffff): the still buffer
 * (*0x1769c + 0x28dc, filled by M_/E_/SRS_SetReadOut via FUN_e06a0dca; word 1 = reg 0x2D) and the LV buffers
 * (*0x176a8 + 0x1bac + n * 0xac, FUN_e06cb1f4; reg 0x2D word found by scanning), and re-check the patched table
 * every 200 ms (the first apply's dump showed whole 32-byte lines back at Canon's values). */
#define DI_STILLP   0x1769c
#define DI_LVP      0x176a8
#define DI_NLV      10
static int di_ram_ok(uint32_t a) { return a >= 0x1000 && a < 0x80000000 && !(a & 3); }
static uint32_t di_still_buf(void)
{
    uint32_t p = *(volatile uint32_t *)DI_STILLP;
    return di_ram_ok(p) ? p + 0x28dc : 0;
}
static uint32_t di_lv_word(int n)
{
    uint32_t p = *(volatile uint32_t *)DI_LVP;
    if (!di_ram_ok(p)) return 0;
    volatile uint32_t *b = (volatile uint32_t *)(p + 0x1bac + n * 0xac);
    for (int i = 0; i < 0xac / 4; i++)
    {
        if (b[i] == 0xffffffff) break;
        if ((b[i] >> 22) == 0x2D) return b[i];
    }
    return 0;
}
#define DI_NW 64
static struct { uint32_t ms; uint8_t which; uint32_t v; } di_w[DI_NW];
static volatile int di_nw, di_nw_lost;
static uint32_t di_wlast[2 + DI_NLV];
static volatile int di_mism, di_mism_checks;
static volatile uint32_t di_mism_first, di_mism_val, di_mism_ms;
static uint32_t di_want[DI_NROWS][DI_NMODES][DI_NISO];
static int di_cpu;

static void di_note_word(int which, uint32_t v)
{
    if (v == di_wlast[which]) return;
    di_wlast[which] = v;
    int i = di_nw;
    if (i < DI_NW) { di_w[i].ms = get_ms_clock(); di_w[i].which = which; di_w[i].v = v; di_nw = i + 1; }
    else di_nw_lost++;
}

/* sensor-state transitions while the test is on (which m_SensorMode does a still capture use?) */
#define DI_NS 96
static struct { uint32_t ms; uint8_t iso; uint8_t mode; uint8_t fine; uint8_t mapped; int16_t v4; } di_s[DI_NS];
static volatile int di_ns, di_ns_lost;
static volatile int di_task_run;

#define DI_BUF 4096
static char *di_buf;                    /* malloc'd on first use: bss comes out of Canon's heap */
static int di_len;
static void di_printf(const char *fmt, ...)
{
    if (!di_buf && !(di_buf = malloc(DI_BUF))) return;
    va_list ap;
    va_start(ap, fmt);
    if (di_len < DI_BUF - 160) di_len += vsnprintf(di_buf + di_len, DI_BUF - di_len, fmt, ap);
    va_end(ap);
}
static void di_flush(void)
{
    if (!di_len) return;
    FILE *f = FIO_CreateFileOrAppend(DI_LOG);
    if (f) { FIO_WriteFile(f, di_buf, di_len); FIO_CloseFile(f); }
    di_len = 0;
}

static int di_check_orig(int r, int m)
{
    if (r)                              /* reg 0x2F: just the register number */
    {
        for (int c = 0; c < DI_NISO; c++) if ((di_orig[r][m][c] >> 22) != di_regs[r]) return 0;
        return 1;
    }
    /* every column must be reg 0x2D with four equal fields */
    for (int c = 0; c < DI_NISO; c++)
    {
        uint32_t v = di_orig[0][m][c];
        uint32_t f = (v >> 4) & 0xF;
        if ((v >> 22) != 0x2D || ((v >> 8) & 0xF) != f || ((v >> 12) & 0xF) != f || ((v >> 16) & 0xF) != f || f > 7)
            return 0;
    }
    return 1;
}

static void di_dump_rows(const char *what)
{
    di_printf("%s (read through the ROM addresses):\n", what);
    for (int r = 0; r < DI_NROWS; r++)
        for (int m = 0; m < DI_NMODES; m++)
        {
            if (m != 0 && m != 4) continue;     /* still (table 0) and photo LV (table 4) */
            di_printf("  reg %x mode %d %x:", di_regs[r], m, (int)di_rom_r(r, m, 0));
            for (int c = 0; c < DI_NISO; c++) di_printf(" %x", (int)(*(volatile uint32_t *)di_rom_r(r, m, c) & 0x3fffff));
            di_printf("\n");
        }
}

/* write the table: on = patched with (iso, pat), off = Canon's values. returns 0 or an error code */
static int di_write(int on)
{
    int err = romremap_init();
    if (err) return 0x20000 | (-err);
    if (!di_have_orig)
    {
        for (int r = 0; r < DI_NROWS; r++)
            for (int m = 0; m < DI_NMODES; m++)
                for (int c = 0; c < DI_NISO; c++) di_orig[r][m][c] = *(volatile uint32_t *)di_rom_r(r, m, c);
        di_have_orig = 1;
    }
    const struct di_cand *k = &di_cands[di_pat];
    uint32_t alt = (uint32_t)di_iso * 0x11110, mask = k->isof | k->bits;
    di_bad = di_skipped = 0;
    di_cpu = get_cpu_id();
    for (int r = 0; r < DI_NROWS; r++)
        for (int m = 0; m < DI_NMODES; m++)
        {
            if (!di_check_orig(r, m)) { di_skipped++; continue; }
            for (int c = 0; c < DI_NISO; c++)
            {
                uint32_t o = di_orig[r][m][c], v = o;
                if (on && r == k->row)
                    v = k->from_col >= 0 ? di_orig[r][m][k->from_col] : (o & ~mask) | (alt & k->isof) | k->bits;
                di_want[r][m][c] = v;
                uint32_t *ram = (uint32_t *)romremap_ram(di_rom_r(r, m, c));
                if (!ram) return 0x30000;
                if (*ram != v) { *ram = v; romremap_flush(ram, 4); }
                if (*(volatile uint32_t *)di_rom_r(r, m, c) != v) di_bad++;
            }
        }
    return di_bad ? 0x40000 | di_bad : 0;
}

static void di_sample_task(void *unused)
{
    int last = -1, tick = 0;
    while (di_task_run)
    {
        volatile uint8_t *s = (volatile uint8_t *)DI_STATE;
        int iso = s[0], mode = *(volatile uint32_t *)(DI_STATE + 0xc), fine = *(volatile uint32_t *)(DI_STATE + 0x48);
        int key = iso | (mode << 8) | (fine << 16);
        if (key != last)
        {
            last = key;
            int i = di_ns;
            if (i < DI_NS)
            {
                di_s[i].ms = get_ms_clock(); di_s[i].iso = iso; di_s[i].mode = mode; di_s[i].fine = fine;
                di_s[i].mapped = mode < 14 ? *(volatile uint32_t *)(0xE0B40700 + mode * 4) : 0xff;
                di_s[i].v4 = *(volatile int16_t *)(DI_STATE + 4);
                di_ns = i + 1;
            }
            else di_ns_lost++;
        }
        uint32_t sb = di_still_buf();
        if (sb) { di_note_word(0, *(volatile uint32_t *)(sb + 4)); di_note_word(1 + DI_NLV, *(volatile uint32_t *)(sb + 12)); }
        for (int n = 0; n < DI_NLV; n++) di_note_word(1 + n, di_lv_word(n));
        if (++tick % 40 == 0 && di_have_orig)
        {
            di_mism_checks++;
            for (int r = 0; r < DI_NROWS; r++)
            for (int m = 0; m < DI_NMODES; m++)
                for (int c = 0; c < DI_NISO; c++)
                {
                    uint32_t v = *(volatile uint32_t *)di_rom_r(r, m, c);
                    if (v != di_want[r][m][c])
                    {
                        if (!di_mism) { di_mism_first = di_rom_r(r, m, c); di_mism_val = v; di_mism_ms = get_ms_clock(); }
                        di_mism++;
                    }
                }
        }
        msleep(5);
    }
}

static void di_log_words(void)
{
    uint32_t sb = di_still_buf();
    di_printf("command buffers: still %x, LV base %x; table re-checks %d, mismatches %d (first %x = %x at %d ms); "
              "patch ran on cpu %d\n", (int)sb, (int)*(volatile uint32_t *)DI_LVP, di_mism_checks, di_mism,
              (int)di_mism_first, (int)di_mism_val, (int)di_mism_ms, di_cpu);
    if (sb)
    {
        di_printf("  still buffer now:");
        for (int i = 0; i < 40; i++)
        {
            uint32_t v = *(volatile uint32_t *)(sb + i * 4);
            di_printf(" %x", (int)v);
            if (v == 0xffffffff) break;
        }
        di_printf("\n");
    }
    /* GAIN9-12 for stills come from records behind the pointer table at 0x68668 (FUN_e06a0dca), not this table */
    for (int i = 0; i < 9; i++)
    {
        uint32_t p = *(volatile uint32_t *)(0x68668 + i * 4);
        di_printf("  gain ptr %d: %x", i, (int)p);
        if (di_ram_ok(p) || (p >= 0xE0000000 && p < 0xE2000000 && !(p & 3)))
            for (int j = 0; j < 4; j++) di_printf(" %x", (int)*(volatile uint32_t *)(p + j * 4));
        di_printf("\n");
    }
    di_printf("reg word changes (%d, %d not logged): ms buffer(0 = still 0x2D, n = LV n-1 0x2D, %d = still 0x2F) word\n",
              di_nw, di_nw_lost, 1 + DI_NLV);
    for (int i = 0; i < di_nw; i++) di_printf("  %d %d %x\n", (int)di_w[i].ms, di_w[i].which, (int)di_w[i].v);
    di_nw = di_nw_lost = 0;
    di_mism = di_mism_checks = 0;
}

static void di_log_samples(void)
{
    di_log_words();
    di_printf("sensor state changes (%d, %d not logged): ms iso_idx m_SensorMode -> table mode, fine, v4\n", di_ns, di_ns_lost);
    for (int i = 0; i < di_ns; i++)
        di_printf("  %d %x %d -> %d %d %d\n", (int)di_s[i].ms, di_s[i].iso, di_s[i].mode, di_s[i].mapped, di_s[i].fine, di_s[i].v4);
    di_ns = di_ns_lost = 0;
}

static void di_apply(int on)
{
    int t0 = get_ms_clock();
    di_err = di_write(on);
    di_printf("\n=== %d ms: dual ISO test %s: second ISO %d (step %d), candidate %d (reg %x isof %x bits %x col %d); "
              "result %x (%s), %d rows skipped, %d bad read-backs, %d ms\n", t0, on ? "ON" : "OFF", 100 << di_iso, di_iso,
              di_pat, di_regs[di_cands[di_pat].row], (int)di_cands[di_pat].isof, (int)di_cands[di_pat].bits,
              di_cands[di_pat].from_col, di_err, romremap_stage(), di_skipped, di_bad, (int)(get_ms_clock() - t0));
    if (!di_err) di_dump_rows(on ? "patched rows" : "restored rows");
    if (!on) di_log_samples();
    di_flush();
    if (di_err) { di_on = 0; NotifyBox(5000, "Dual ISO test: FAILED (%x). See ML/DUALISO.TXT", di_err); return; }
    di_on = on;
    di_applied_iso = on ? di_iso : -1;
    di_applied_pat = on ? di_pat : -1;
    if (on && !di_task_run) { di_task_run = 1; di_ns = di_ns_lost = 0; task_create("di_sample", 0x1e, 0x800, di_sample_task, 0); }
    /* v4.52: the watcher keeps running after OFF, so reference shots are logged too */
    NotifyBox(4000, on ? "Dual ISO test ON: candidate %d, ISO %d" : "Dual ISO test OFF: Canon values back", di_pat, 100 << di_iso);
}

static MENU_SELECT_FUNC(di_toggle) { di_apply(!di_on); }
static MENU_SELECT_FUNC(di_iso_select)
{
    menu_numeric_toggle(&di_iso, delta, 0, 7);
    if (di_on) di_apply(1);
}
static MENU_SELECT_FUNC(di_pat_select)
{
    menu_numeric_toggle(&di_pat, delta, 0, COUNT(di_cands) - 1);
    if (di_on) di_apply(1);
}
static MENU_SELECT_FUNC(di_save_select)
{
    if (!di_task_run) { di_task_run = 1; task_create("di_sample", 0x1e, 0x800, di_sample_task, 0); }
    di_printf("\n=== %d ms: log saved (test %s)\n", (int)get_ms_clock(), di_on ? "ON" : "OFF");
    di_log_samples();
    di_flush();
    NotifyBox(3000, "Dual ISO: log written to ML/DUALISO.TXT");
}

static MENU_UPDATE_FUNC(di_update)
{
    if (di_err)
    {
        MENU_SET_VALUE("ERROR %x", di_err);
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Failed (%s). Power cycle.", romremap_stage());
    }
    else if (di_on)
    {
        MENU_SET_VALUE("ON, ISO %d", 100 << di_applied_iso);
        MENU_SET_WARNING(MENU_WARN_INFO, "Test only: shoot RAW (CR3). Change Canon ISO once for LiveView.");
    }
    else MENU_SET_VALUE("OFF");
}

static struct menu_entry di_menu[] = {
    {
        .name   = "Dual ISO (test)",
        .select = menu_open_submenu,
        .update = di_update,
        .help   = "First test of dual ISO on the R: sets part of the sensor's ISO register",
        .help2  = "to a second ISO. Shoot RAW, then bring the CR3s. Always OFF at power-on.",
        .children = (struct menu_entry[]) {
            {
                .name   = "Dual ISO test",
                .select = di_toggle,
                .update = di_update,
                .help   = "SET: ON/OFF. ON patches CMOS register 0x2D in Canon's ISO table (RAM copy).",
                .help2  = "OFF puts Canon's values back. Log: ML/DUALISO.TXT.",
            },
            {
                .name    = "Second ISO",
                .priv    = &di_iso,
                .max     = 7,
                .choices = CHOICES("100", "200", "400", "800", "1600", "3200", "6400", "12800"),
                .select  = di_iso_select,
                .help    = "ISO for the patched fields. Canon's ISO setting stays the base ISO.",
            },
            {
                .name    = "Candidate",
                .priv    = &di_pat,
                .max     = 6,
                .choices = CHOICES("0 control", "1 f4+b20", "2 f4+b21", "3 f12+b20", "4 f12+b21", "5 f124+b2021", "6 reg 2F"),
                .select  = di_pat_select,
                .help    = "Experiment: 0 = whole frame at 2nd ISO (control); 1-5 = enable-bit tries",
                .help2   = "on reg 0x2D; 6 = reg 0x2F at its ISO 25600 value. One photo each.",
            },
            {
                .name    = "Save log",
                .select  = di_save_select,
                .help    = "Writes the sensor-mode changes seen since ON to ML/DUALISO.TXT.",
            },
            MENU_EOL
        },
    },
};

static void di_init(void *unused)
{
    menu_add("Shoot", di_menu, COUNT(di_menu));
}

INIT_FUNC("dualiso_r", di_init);
