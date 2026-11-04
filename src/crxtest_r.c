/* riz v4.17 (EOS R): Debug > "CRX encode test" - step 2 of the lossless 4K work (~/ml/ghidra/notes/crx/STEP1.md).
 *
 * Compresses ONE live-view raw frame with Canon's own CRX encoder through its encoder manager (algsmgr, running
 * since boot) using descriptor type 0 = Canon's lossless CR3 profile, and saves everything needed to check the
 * result bit-exactly on the PC (~/ml/tests/crx_host):
 *   ML/LOGS/CRXTEST.LOG  stage log (open/append/close per line: survives a hang)
 *   ML/CRXTEST.BIN       "CRX1" header + CMP1 main header + every ring entry (entry 0 = mdat header, rest = chunks)
 *   ML/CRXRAW.BIN        the uncompressed MEM1 frame that was encoded (Canon pack cfg, as in memory)
 * Input: Canon's MEM1 raw tap buffer, frozen by switching the tap off after a frame (checksummed to confirm).
 * While it runs, rawrec_running() reports busy so the RAW overlays leave the tap alone.
 * Canon calls used (ROM 1.8.0, decompiled):
 *   ALGSMGR_StartEncode(desc[0x2C], n, cb, ctx) e066b032; cb(ctx, handles[]) from the algsmgr task
 *   GetRingList(h, &n, &addrs, &sizes) e066b49c; GetMainHeader(h, &addr, &size) e066b4e4
 *   ALGSMGR_FreeEncBuffer(h, 0, 0) e066b522 (Canon always passes 0, 0)
 * Risk: if Canon's resource manager never grants the encoder's output memory, algsmgr's task waits forever
 * (camera photos would hang until a restart). The test times out after 8 s, logs the stage and says so. */

#include <stdarg.h>
#include "dryos.h"
#include "menu.h"
#include "bmp.h"
#include "property.h"

extern volatile int r_crx_busy;          /* rectest_r.c: rawrec_running() includes it */

#define CX_PATH   0x133CC                 /* Canon Mem1Path struct (see rawlv_r.c) */
#define CX_UNC    0x40000000
#define CX_MGR    0x5A80                  /* algsmgr manager pointer (DAT_e066b110) */
#define CX_LOG    "ML/LOGS/CRXTEST.LOG"

typedef int (*cx_start_t)(uint32_t *desc, uint32_t n, void (*cb)(void *, uint32_t *), void *ctx);
typedef int (*cx_ring_t)(uint32_t *h, uint32_t *n, uint32_t **addrs, uint32_t **sizes);
typedef int (*cx_hdr_t)(uint32_t *h, uint32_t *addr, uint32_t *size);
typedef int (*cx_free_t)(uint32_t *h, int a, int b);
#define CX_START ((cx_start_t)0xE066B033)
#define CX_RING  ((cx_ring_t) 0xE066B49D)
#define CX_HDR   ((cx_hdr_t)  0xE066B4E5)
#define CX_FREE  ((cx_free_t) 0xE066B523)

/* v4.18: algsmgr reports its output buffers as physical + 0x80000000 (e.g. 0xBB990000 for shoot memory that ML's
 * shoot_malloc sees at 0x7B990000); the CPU reads 0x80000000-0xBFFFFFFF as open bus except panel VRAM. v4.17 OR'ed
 * 0x40000000 onto them -> 0xFB99.../0xC378... (ROM / MMIO) and hung while saving (riz 2026-11-04). CPU uncached view of
 * any of the three aliases (cached 0x0..., uncached 0x4.../0x7..., encoder 0x8.../0xB...): */
static inline uint32_t cx_cpu(uint32_t a) { return (a & 0x3FFFFFFF) | CX_UNC; }
/* v4.19: that alias held only 0xFF on the camera (v4.18: headers fine, every chunk byte 0xFF), so the chunks are not
 * physical + 0x80000000 as seen by the CPU. Canon's stills code hands these addresses straight to the card writer
 * (CreateMemorySuite on each ring entry, e055c7ba), whose DMA reads them. v4.19 saves the chunks the same way: ML's
 * FIO_WriteFile passes addresses >= 0x40000000 through unchanged to Canon's _FIO_WriteFile. */

static char cx_last[64] = "not run";
static volatile int cx_running;
static struct semaphore *cx_sem;
static volatile uint32_t *cx_handle;
static volatile int cx_cb_count;

static void cx_log(const char *fmt, ...)
{
    char buf[200];
    int n = snprintf(buf, sizeof(buf), "%7d ", get_ms_clock());
    va_list ap; va_start(ap, fmt);
    n += vsnprintf(buf + n, sizeof(buf) - n - 1, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(buf) - 2) n = sizeof(buf) - 2;
    buf[n++] = '\n';
    FILE *f = FIO_CreateFileOrAppend(CX_LOG);
    if (f) { FIO_WriteFile(f, buf, n); FIO_CloseFile(f); }
}

static void cx_mgr_log(const char *when)
{
    uint32_t m = MEM(CX_MGR);
    if (!m) { cx_log("%s: algsmgr manager pointer is 0", when); return; }
    cx_log("%s: mgr %08x flags %x prohibit %d allocs %d/%d bufs %d/%d rawsize %d", when, m, MEM(m + 0xC4),
           MEM(m + 0x184), MEM(m + 0x3C8), MEM(m + 0x3CC), MEM(m + 0x108), MEM(m + 0x10C), MEM(m + 0x3D0));
}

/* called from the algsmgr task when every tile is done */
static void cx_cb(void *ctx, uint32_t *handles)
{
    cx_handle = handles ? (uint32_t *)handles[0] : 0;
    cx_cb_count++;
    give_semaphore(cx_sem);
}

static uint32_t cx_sum(uint32_t base, uint32_t pitch, uint32_t h)
{
    uint32_t s = 0;
    for (int k = 1; k <= 3; k++)                /* 3 rows, 256 words each */
    {
        uint32_t row = base + (h * k / 4) * pitch;
        for (int i = 0; i < 256; i++) s = s * 31 + MEM(cx_cpu(row + i * 4));
    }
    return s;
}

struct cx_hdr
{
    char magic[4];                          /* "CRX1" */
    uint32_t hdr_size, w, h, bpp, pitch, pack[5], addr, frozen, enc_ms, main_hdr_size, n_entries;
    uint32_t entry_size[64];
};

static void cx_task(void *unused)
{
    cx_running = 1;
    uint32_t t0 = get_ms_clock();
    cx_log("---- CRX encode test (v4.19)");
    snprintf(cx_last, sizeof(cx_last), "running...");

    int tap_was = MEM(CX_PATH + 0x40);
    r_crx_busy = 1;
    if (!cx_sem) cx_sem = create_named_semaphore("crx_test", 0);

    if (!lv || RECORDING) { snprintf(cx_last, sizeof(cx_last), "needs movie LV, not recording"); goto out; }
    if (!tap_was) { call("lv_save_raw", 1); msleep(600); }
    uint32_t w = MEM(CX_PATH + 0x10), h = MEM(CX_PATH + 0x14), addr = MEM(CX_PATH + 0x58);
    uint32_t pk[5]; for (int i = 0; i < 5; i++) pk[i] = MEM(CX_PATH + 0x64 + 4 * i);
    uint32_t bpp = pk[2] == 2 ? 14 : pk[2] == 1 ? 12 : pk[2] == 0 ? 10 : 0;
    uint32_t pitch = w * bpp / 8;
    cx_log("MEM1 %dx%d at %08x, pack {%d,%d,%d,%d,%d} -> %d-bit pitch %d, width field %d, tap was %d",
           w, h, addr, pk[0], pk[1], pk[2], pk[3], pk[4], bpp, pitch, MEM(CX_PATH + 0x18), tap_was);
    if (!((w == 4256 && h == 2214) || (w == 2304 && h == 1290)) || !bpp || !MEM(CX_PATH + 0x40))
    { snprintf(cx_last, sizeof(cx_last), "MEM1 %dx%d / %d-bit not usable", w, h, bpp); cx_log("abort: %s", cx_last); goto out; }

    /* freeze: switch the tap off, let the in-flight frame finish, confirm the buffer stops changing */
    uint32_t live1 = cx_sum(addr, pitch, h); msleep(100);
    uint32_t live2 = cx_sum(addr, pitch, h);
    call("lv_save_raw", 0);
    msleep(250);
    uint32_t s1 = cx_sum(addr, pitch, h); msleep(150);
    uint32_t s2 = cx_sum(addr, pitch, h); msleep(150);
    uint32_t s3 = cx_sum(addr, pitch, h);
    int frozen = s1 == s2 && s2 == s3;
    cx_log("tap on: sums %08x %08x (%s) | tap off: %08x %08x %08x -> %s", live1, live2,
           live1 == live2 ? "static!" : "changing", s1, s2, s3, frozen ? "FROZEN" : "still changing");

    cx_mgr_log("before");
    uint32_t desc[11] = { 0, w, h, 0, 0, bpp, addr, w, 0, 0, 0 };   /* type 0 = Canon lossless CR3 profile */
    cx_handle = 0; cx_cb_count = 0;
    uint32_t te = get_ms_clock();
    cx_log("StartEncode type 0, %dx%d %d-bit, line %d px, addr %08x", w, h, bpp, w, addr);
    int rc = CX_START(desc, 1, cx_cb, 0);
    cx_log("StartEncode returned %d", rc);
    if (rc & 1) { snprintf(cx_last, sizeof(cx_last), "StartEncode failed %d", rc); goto out_tap; }
    int to = take_semaphore(cx_sem, 8000);
    uint32_t enc_ms = get_ms_clock() - te;
    cx_mgr_log("after");
    {   /* algsmgr's allocation table from the resource manager (completion e02f1c30 copies 0x248 B to mgr+0x188):
         * lower +0x188, upper +0x218, sizes +0x2A8, flags +0x338, count +0x3C8 */
        uint32_t m = MEM(CX_MGR);
        for (int k = 0; m && k < 4; k++)
            cx_log("alloc[%d]: lower %08x upper %08x size %08x flags %08x", k, MEM(m + 0x188 + 4 * k),
                   MEM(m + 0x218 + 4 * k), MEM(m + 0x2A8 + 4 * k), MEM(m + 0x338 + 4 * k));
    }
    if (to)
    {
        snprintf(cx_last, sizeof(cx_last), "TIMEOUT after 8 s - restart the camera");
        cx_log("TIMEOUT: no callback after %d ms (cb count %d). algsmgr may be stuck waiting for its output memory;"
               " restart the camera before taking photos.", enc_ms, cx_cb_count);
        goto out_tap;
    }
    uint32_t *hd = (uint32_t *)cx_handle;
    cx_log("callback after %d ms, handle %08x", enc_ms, hd);
    if (!hd) { snprintf(cx_last, sizeof(cx_last), "callback without a handle"); goto out_tap; }
    cx_log("handle: %08x %08x %08x %08x %08x %08x %08x %08x %08x", hd[0], hd[1], hd[2], hd[3], hd[4], hd[5], hd[6],
           hd[7], hd[8]);

    uint32_t ha = 0, hs = 0, n = 0, *ea = 0, *es = 0;
    int r1 = CX_HDR(hd, &ha, &hs);
    int r2 = CX_RING(hd, &n, &ea, &es);
    cx_log("GetMainHeader %d: %08x size %d | GetRingList %d: %d entries", r1, ha, hs, r2, n);
    uint32_t total = 0;
    if (!r1 && !r2 && n && n <= 64)
    {
        for (uint32_t i = 0; i < n; i++)
        {
            /* one test read of the first word through the CPU alias, logged before any file write */
            cx_log("  entry %d: %08x size %d", i, ea[i], es[i]);
            if (es[i]) cx_log("    first word: raw %08x = %08x | alias %08x = %08x", ea[i], MEM(ea[i]), cx_cpu(ea[i]),
                              MEM(cx_cpu(ea[i])));   /* 32-bit reads of 0x8...-0xB... are safe (v8 probe) */
            total += es[i];
        }
        cx_log("main header cpu %08x first word %08x", cx_cpu(ha), hs ? MEM(cx_cpu(ha)) : 0);
        struct cx_hdr H = { {'C','R','X','1'}, sizeof(struct cx_hdr), w, h, bpp, pitch,
                            {pk[0], pk[1], pk[2], pk[3], pk[4]}, addr, frozen, enc_ms, hs, n, {0} };
        for (uint32_t i = 0; i < n; i++) H.entry_size[i] = es[i];
        FILE *f = FIO_CreateFile("ML/CRXTEST.BIN");
        if (f)
        {
            FIO_WriteFile(f, &H, sizeof(H));
            if (hs) FIO_WriteFile(f, (void *)cx_cpu(ha), hs);
            for (uint32_t i = 0; i < n; i++) if (es[i]) FIO_WriteFile(f, (void *)ea[i], es[i]);   /* as Canon */
            FIO_CloseFile(f);
        }
        f = FIO_CreateFile("ML/CRXRAW.BIN");
        if (f) { FIO_WriteFile(f, (void *)cx_cpu(addr), pitch * h); FIO_CloseFile(f); }
        cx_log("files written");
        uint32_t s4 = cx_sum(addr, pitch, h);
        uint32_t bpp100 = (uint32_t)((uint64_t)total * 800 / (w * h));
        cx_log("saved: compressed %d bytes (%d.%02d bits/px), raw %d bytes; frame still unchanged after save: %s",
               total + hs, bpp100 / 100, bpp100 % 100, pitch * h, s4 == s1 ? "yes" : "NO");
        snprintf(cx_last, sizeof(cx_last), "OK %d ms, %d.%02d bpp%s", enc_ms, bpp100 / 100, bpp100 % 100,
                 frozen ? "" : " (frame moved)");
    }
    else snprintf(cx_last, sizeof(cx_last), "no data (hdr %d ring %d n %d)", r1, r2, n);
    int rf = CX_FREE(hd, 0, 0);
    cx_log("FreeEncBuffer %d", rf);
    msleep(200);
    cx_mgr_log("freed");

out_tap:
    if (tap_was) call("lv_save_raw", 1);    /* the RAW overlays take it back over otherwise */
out:
    r_crx_busy = 0;
    cx_log("done in %d ms: %s", get_ms_clock() - t0, cx_last);
    NotifyBox(5000, "CRX test: %s", cx_last);
    cx_running = 0;
}

static MENU_SELECT_FUNC(cx_select)
{
    if (cx_running) return;
    task_create("crx_test", 0x1e, 0x4000, cx_task, 0);
}

static MENU_UPDATE_FUNC(cx_update)
{
    MENU_SET_VALUE("%s", cx_last);
    MENU_SET_HELP("Compresses one LV raw frame with Canon's lossless CRX encoder; saves ML/CRXTEST.BIN + CRXRAW.BIN.");
    MENU_SET_WARNING(MENU_WARN_INFO, "4K movie mode, LV on, not recording. Restart the camera if it says TIMEOUT.");
}

static struct menu_entry cx_menu[] = {
    { .name = "CRX encode test", .select = cx_select, .update = cx_update, .icon_type = IT_ACTION },
};

static void cx_init(void *unused) { menu_add("Debug", cx_menu, COUNT(cx_menu)); }
INIT_FUNC("crx_test", cx_init);
