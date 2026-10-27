/* riz/eosr-dmlog: read-only snapshot of Canon's live-view frame-job structures.
 *
 * Debug -> LV job dump. Takes 8 samples, 250 ms apart, of RAM structures found
 * via Ghidra + the v2 LV logs (EOS R 1.8.0), and appends them to ML/LVJOB.BIN:
 *   JOBR  Epp job ring passed to PostRaw2YuvStage (4 x 0x14C at 0x3AE10)
 *   R2YP  raw2yuv stage globals (0x83E0..)        R2YS  *(0x83E0) stage object
 *   R2YA  raw2yuv MEMNAVI address array (0x3BFB4..) MEMF  Memif window table (0x742A8)
 *   LVBC  LvBufferController pointer array (12)   LVBn/LVSn  each controller + its slots
 * Only reads memory (pointers are range-checked) and writes the file.
 * Record: "LVJB" u32 run, sample, ms, sig; then sections: tag[4] u32 addr u32 len, data.
 * Personal diagnostic build, not for upstream.
 */
#include "dryos.h"
#include "menu.h"
#include "beep.h"
#include "propvalues.h"
#include "consts.h"
#include "timer.h"
#include "mem.h"

#define LVJ_SAMPLES   8
#define LVJ_PERIOD_MS 250
#define LVJ_BUF       0x4000

static volatile int lvj_busy = 0;
static int lvj_runs = 0;
static uint8_t *lvj_buf;
static int lvj_len;

/* RAM is 1 GB cached at 0x0, uncached alias at 0x40000000 */
static int lvj_ram_ok(uint32_t p, uint32_t len)
{
    uint32_t c = p & ~0x40000000;
    return p < 0x80000000 && c >= 0x1000 && c + len < 0x40000000;
}

static void lvj_put(const void *src, int len)
{
    if (lvj_len + len > LVJ_BUF) return;
    memcpy(lvj_buf + lvj_len, src, len);
    lvj_len += len;
}

static void lvj_section(const char *tag, uint32_t addr, uint32_t len)
{
    if (!lvj_ram_ok(addr, len)) len = 0;
    uint32_t hdr[3];
    memcpy(&hdr[0], tag, 4); hdr[1] = addr; hdr[2] = len;
    lvj_put(hdr, sizeof(hdr));
    if (len) lvj_put((void *)addr, len);
}

static void lvj_task(void *unused)
{
    lvj_buf = malloc(LVJ_BUF);
    FILE *f = lvj_buf ? FIO_CreateFileOrAppend("ML/LVJOB.BIN") : 0;
    if (!f) { NotifyBox(3000, "LV job dump: no file/memory"); goto out; }

    for (int s = 0; s < LVJ_SAMPLES; s++)
    {
        lvj_len = 0;
        uint32_t rec[5] = { 0x424A564C /* "LVJB" */, lvj_runs, s, get_ms_clock(),
                            (lv ? 1 : 0) | (is_movie_mode() ? 2 : 0) | (RECORDING ? 4 : 0) | (R_LV_ON_EVF() ? 8 : 0) };
        lvj_put(rec, sizeof(rec));

        lvj_section("JOBR", 0x3AE10, 4 * 0x14C);
        lvj_section("R2YP", 0x83E0, 0x60);
        lvj_section("R2YS", MEM(0x83E0), 0x80);
        lvj_section("R2YA", 0x3BFB4, 0x40);
        lvj_section("MEMF", 0x742A8, 0x180);
        lvj_section("LVBC", 0x7F9A4, 12 * 4);
        for (int i = 0; i < 12; i++)
        {
            uint32_t c = MEM(0x7F9A4 + 4 * i);
            char tb[4] = { 'L', 'V', 'B', 'A' + i }, ts[4] = { 'L', 'V', 'S', 'A' + i };
            lvj_section(tb, c, 0x24);
            if (lvj_ram_ok(c, 0x24))
            {
                uint32_t n = MEM(c + 0x10);
                lvj_section(ts, MEM(c + 8), (n <= 16 ? n : 16) * 8);
            }
        }
        FIO_WriteFile(f, lvj_buf, lvj_len);
        msleep(LVJ_PERIOD_MS);
    }
    FIO_CloseFile(f);
    lvj_runs++;
    beep();
out:
    if (lvj_buf) free(lvj_buf);
    lvj_buf = 0;
    lvj_busy = 0;
}

static MENU_SELECT_FUNC(lvj_select)
{
    if (lvj_busy) return;
    lvj_busy = 1;
    task_create("lvj_task", 0x1e, 0x1000, lvj_task, 0);
}

static MENU_UPDATE_FUNC(lvj_update)
{
    MENU_SET_VALUE(lvj_busy ? "sampling..." : "%d runs saved", lvj_runs);
}

static struct menu_entry lvj_menu[] = {
    {
        .name   = "LV job dump",
        .select = lvj_select,
        .update = lvj_update,
        .help   = "Read-only: 8 snapshots of Canon's LV frame-job structs -> ML/LVJOB.BIN",
        .help2  = "SET, close the menu, keep live view running until the beep (~2 s).",
    },
};

static void lvj_init(void *unused)
{
    menu_add("Debug", lvj_menu, COUNT(lvj_menu));
}

INIT_FUNC("lvjob", lvj_init);
