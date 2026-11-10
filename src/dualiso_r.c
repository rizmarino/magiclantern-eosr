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
#define DI_ROW      0x24                /* register row 1 = CMOS reg 0x2D */
#define DI_NMODES   9
#define DI_NISO     9
#define DI_STATE    0xef24              /* ImageSequence state: +0 ISO index (byte), +4 short, +0xc m_SensorMode, +0x48 */
#define DI_LOG      "ML/DUALISO.TXT"

static int di_on, di_err, di_applied_iso = -1, di_applied_pat = -1, di_bad, di_skipped;
static CONFIG_INT("dualiso.test.iso", di_iso, 4);       /* 0..7 = ISO 100..12800 */
static CONFIG_INT("dualiso.test.fields", di_pat, 0);
static const uint32_t di_masks[] = { 0x00FF0, 0x0F0F0, 0xF00F0, 0xFFFF0 };
static uint32_t di_orig[DI_NMODES][DI_NISO];
static int di_have_orig;

static uint32_t di_rom(int m, int c) { return DI_TBL + m * DI_MSTRIDE + DI_ROW + c * 4; }

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

static int di_check_orig(int m)
{
    /* every column must be reg 0x2D with four equal fields */
    for (int c = 0; c < DI_NISO; c++)
    {
        uint32_t v = di_orig[m][c];
        uint32_t f = (v >> 4) & 0xF;
        if ((v >> 22) != 0x2D || ((v >> 8) & 0xF) != f || ((v >> 12) & 0xF) != f || ((v >> 16) & 0xF) != f || f > 7)
            return 0;
    }
    return 1;
}

static void di_dump_rows(const char *what)
{
    di_printf("%s (read through the ROM addresses):\n", what);
    for (int m = 0; m < DI_NMODES; m++)
    {
        di_printf("  mode %d %x:", m, (int)di_rom(m, 0));
        for (int c = 0; c < DI_NISO; c++) di_printf(" %x", (int)(*(volatile uint32_t *)di_rom(m, c) & 0x3fffff));
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
        for (int m = 0; m < DI_NMODES; m++)
            for (int c = 0; c < DI_NISO; c++) di_orig[m][c] = *(volatile uint32_t *)di_rom(m, c);
        di_have_orig = 1;
    }
    uint32_t mask = di_masks[di_pat], alt = (uint32_t)di_iso * 0x11110;
    di_bad = di_skipped = 0;
    for (int m = 0; m < DI_NMODES; m++)
    {
        if (!di_check_orig(m)) { di_skipped++; continue; }
        for (int c = 0; c < DI_NISO; c++)
        {
            uint32_t v = on ? (di_orig[m][c] & ~mask) | (alt & mask) : di_orig[m][c];
            uint32_t *ram = (uint32_t *)romremap_ram(di_rom(m, c));
            if (!ram) return 0x30000;
            *ram = v;
            romremap_flush(ram, 4);
            if (*(volatile uint32_t *)di_rom(m, c) != v) di_bad++;
        }
    }
    return di_bad ? 0x40000 | di_bad : 0;
}

static void di_sample_task(void *unused)
{
    int last = -1;
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
        msleep(5);
    }
}

static void di_log_samples(void)
{
    di_printf("sensor state changes (%d, %d not logged): ms iso_idx m_SensorMode -> table mode, fine, v4\n", di_ns, di_ns_lost);
    for (int i = 0; i < di_ns; i++)
        di_printf("  %d %x %d -> %d %d %d\n", (int)di_s[i].ms, di_s[i].iso, di_s[i].mode, di_s[i].mapped, di_s[i].fine, di_s[i].v4);
    di_ns = di_ns_lost = 0;
}

static void di_apply(int on)
{
    int t0 = get_ms_clock();
    di_err = di_write(on);
    di_printf("\n=== %d ms: dual ISO test %s: second ISO %d (step %d), fields mask %x; result %x (%s), %d modes skipped, "
              "%d bad read-backs, %d ms\n", t0, on ? "ON" : "OFF", 100 << di_iso, di_iso, (int)di_masks[di_pat], di_err,
              romremap_stage(), di_skipped, di_bad, (int)(get_ms_clock() - t0));
    if (!di_err) di_dump_rows(on ? "patched rows" : "restored rows");
    if (!on) di_log_samples();
    di_flush();
    if (di_err) { di_on = 0; NotifyBox(5000, "Dual ISO test: FAILED (%x). See ML/DUALISO.TXT", di_err); return; }
    di_on = on;
    di_applied_iso = on ? di_iso : -1;
    di_applied_pat = on ? di_pat : -1;
    if (on && !di_task_run) { di_task_run = 1; di_ns = di_ns_lost = 0; task_create("di_sample", 0x1e, 0x800, di_sample_task, 0); }
    if (!on) di_task_run = 0;
    NotifyBox(4000, on ? "Dual ISO test ON (ISO %d on fields %x). Change Canon ISO once." : "Dual ISO test OFF: Canon values back",
              100 << di_iso, (int)di_masks[di_pat]);
}

static MENU_SELECT_FUNC(di_toggle) { di_apply(!di_on); }
static MENU_SELECT_FUNC(di_iso_select)
{
    menu_numeric_toggle(&di_iso, delta, 0, 7);
    if (di_on) di_apply(1);
}
static MENU_SELECT_FUNC(di_pat_select)
{
    menu_numeric_toggle(&di_pat, delta, 0, COUNT(di_masks) - 1);
    if (di_on) di_apply(1);
}
static MENU_SELECT_FUNC(di_save_select)
{
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
                .name    = "Fields",
                .priv    = &di_pat,
                .max     = 3,
                .choices = CHOICES("1+2", "1+3", "1+4", "All 4 (control)"),
                .select  = di_pat_select,
                .help    = "Which of the register's 4 ISO fields get the second ISO.",
                .help2   = "All 4 = whole picture at the second ISO (proves the patch works).",
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
