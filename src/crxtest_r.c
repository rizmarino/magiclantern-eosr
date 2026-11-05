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
#include "timer.h"

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
static void cx_reinit(int bpp);
static void cx_release_cfg(void);
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
static uint32_t *volatile cx_hq[16];      /* v4.20: handles in completion order (stream test pipelines 2) */
static volatile uint64_t cx_hq_us[16];
static void cx_cb(void *ctx, uint32_t *handles)
{
    cx_handle = handles ? (uint32_t *)handles[0] : 0;
    cx_hq[cx_cb_count & 15] = (uint32_t *)cx_handle;
    cx_hq_us[cx_cb_count & 15] = get_us_clock();
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
    cx_log("---- CRX encode test (v4.24)");
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
    cx_reinit(bpp);
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
    cx_release_cfg();

out_tap:
    if (tap_was) call("lv_save_raw", 1);    /* the RAW overlays take it back over otherwise */
out:
    r_crx_busy = 0;
    cx_log("done in %d ms: %s", get_ms_clock() - t0, cx_last);
    NotifyBox(5000, "CRX test: %s", cx_last);
    cx_running = 0;
}

/* ======================= v4.20: Debug > "CRX stream test" (lossless 4K step 3) =======================
 * 1. memory: how much shoot memory ML's recorder could get (16 MB contiguous chunks, the recorder's own
 *    allocation, capped at 24 = 384 MB, freed at once) - before the first encode, after the encodes, and after
 *    ALGSMGR_FreeAllEncBuffer e066b592 (what Canon's ENCDRWRAP_ShutdownEncoder calls)
 * 2. for 14, 12, 10-bit (Canon's LSB-first packing, the recorder's pack switch with f3=1): freeze a frame, save it
 *    as ML/CRXT<b>.BIN + CRXR<b>.BIN (bit-exact check on the PC), then 2 s of back-to-back encodes of it
 * 3. 10-bit, 3 s: encode frame k+1 while writing frame k's chunks to ML/CRXSTRM.BIN (the recorder's pipeline)
 * 4. tap on, 2 s: copy each live frame to an ML buffer and encode the copy (v4.21, see cx_freeze)
 * Per-frame numbers go to ML/LOGS/CRXSTRM.TSV, the summary to ML/LOGS/CRXTEST.LOG. */

#define CX_FREEALL ((int (*)(void))0xE066B593)
#define CX_MAXF 600
static struct cx_fr { uint8_t phase, bpp; uint16_t n; uint32_t enc_us, wr_us, bytes, copy_us, free_us, cyc_us; } *cx_f;   /* malloc'd per run */
static int cx_nf;
static uint32_t cx_pk_orig[5], cx_pk_w0; static int cx_pk_applied;

static int cx_pack_apply(int bpp)          /* rectest_r.c rt_pack_apply(bpp, 0): Canon bit order */
{
    for (int i = 0; i < 5; i++) cx_pk_orig[i] = MEM(CX_PATH + 0x64 + 4 * i);
    cx_pk_w0 = MEM(CX_PATH + 0x18);
    uint32_t md = MEM(CX_PATH);
    if (cx_pk_orig[0] != 0 || cx_pk_orig[1] != 1 || cx_pk_orig[2] != 2 || cx_pk_orig[3] != 1 || cx_pk_orig[4] != 0)
    { cx_log("pack cfg not Canon's default"); return -1; }
    if (md >= 0x14 || (MEM(CX_PATH + 0x28) && (md == 0xb || md == 0xf || md == 9)) || cx_pk_w0 != MEM(CX_PATH + 0x10))
    { cx_log("Mem1Path mode %d / width %d vs %d: untested layout", md, cx_pk_w0, MEM(CX_PATH + 0x10)); return -1; }
    if (bpp == 14) return 0;
    uint32_t lb = cx_pk_w0 * bpp / 8, wset = (lb * 8 + 13) / 14;
    if (((wset * 14) >> 3) != lb || (cx_pk_w0 * bpp) % 8) { cx_log("no width field for %d B/line", lb); return -1; }
    uint32_t old = cli();
    MEM(CX_PATH + 0x18) = wset; MEM(CX_PATH + 0x64 + 8) = bpp == 12 ? 1 : 0; MEM(CX_PATH + 0x64 + 12) = 1;
    sei(old);
    cx_pk_applied = 1;
    msleep(300);
    cx_log("pack %d-bit: f2=%d f3=1 width field %d -> %d", bpp, bpp == 12 ? 1 : 0, cx_pk_w0, wset);
    return 0;
}
static void cx_pack_restore(void)
{
    if (!cx_pk_applied) return;
    uint32_t old = cli();
    MEM(CX_PATH + 0x18) = cx_pk_w0; MEM(CX_PATH + 0x64 + 8) = cx_pk_orig[2]; MEM(CX_PATH + 0x64 + 12) = cx_pk_orig[3];
    sei(old);
    cx_pk_applied = 0;
    msleep(150);
}

static int cx_probe_mem(const char *when)
{
    struct memSuite *s[24]; int n = 0;
    for (; n < 24; n++) if (!(s[n] = shoot_malloc_suite_contig(16 * 1024 * 1024))) break;
    for (int i = 0; i < n; i++) shoot_free_suite(s[i]);
    cx_log("memory %s: recorder could get %d x 16 MB%s", when, n, n == 24 ? " (cap reached)" : "");
    return n;
}

static uint32_t cx_frame_bytes(uint32_t *hd, uint32_t *n_out)
{
    uint32_t n = 0, *ea = 0, *es = 0, t = 0;
    if (CX_RING(hd, &n, &ea, &es) || n > 64) n = 0;
    for (uint32_t i = 0; i < n; i++) t += es[i];
    if (n_out) *n_out = n;
    return t;
}

static int cx_wait(int ms) { return take_semaphore(cx_sem, ms); }

/* v4.22: algsmgrEncInit (e02f4136) configures encoder id N only while mgr+0x124+4N is 0, then sets it to 1, and
 * nothing in Canon's code clears it (stills are always 14-bit, one size). So the first encode after boot fixes the
 * bit depth: v4.21's 12-bit run came out as 14-bit CMP1 at 14.02 bpp (noise) and its 10-bit encode hung the camera
 * (riz 2026-11-04). Type 0 = encoder id 0. Clear the flag while no encode is in flight -> the next StartEncode
 * re-runs Canon's own init (e0361c70) with the new depth. */
static int cx_cfg_bpp;                    /* depth algsmgr was last configured with by ML (0 = not by ML) */
static void cx_reinit(int bpp)
{
    uint32_t m = MEM(CX_MGR);
    if (!m) return;
    cx_log("enc init flags: %d %d %d %d (configured %d-bit, want %d)", MEM(m + 0x124), MEM(m + 0x128), MEM(m + 0x12C),
           MEM(m + 0x130), cx_cfg_bpp, bpp);
    if (MEM(m + 0x124) && cx_cfg_bpp != bpp)
    {
        msleep(100);                      /* the last FreeEncBuffer event is processed first */
        MEM(m + 0x124) = 0;
        cx_log("  cleared id 0 init flag -> re-init at %d-bit", bpp);
    }
    cx_cfg_bpp = bpp;
}

/* hand the encoder back: Canon's stills use the same algsmgr encoder, and would otherwise run with ML's 4K
 * frame size / depth; cleared, Canon's next encode re-inits from its own descriptor */
static void cx_release_cfg(void)
{
    uint32_t m = MEM(CX_MGR);
    if (!m) return;
    msleep(100);
    MEM(m + 0x124) = 0;
    cx_cfg_bpp = 0;
    cx_log("encoder config released (id 0 init flag cleared for Canon's next encode)");
}

/* v4.23: v4.22 hung in the 14-bit frozen burst after frame 2 (riz 2026-11-04) - the same stage ran 46 frames in
 * v4.21, so back-to-back encodes hang intermittently. Canon frees asynchronously (event 4) and drives the codec core
 * through an 8-slot command ring; Canon's own users never restart within ms of a free (the single test, 5/5 OK,
 * waits 200 ms). Free, then wait until algsmgr's outstanding-result count (mgr+0x108) is 0, plus a 10 ms gap. */
static uint32_t cx_free_wait(uint32_t *hd)
{
    uint32_t m = MEM(CX_MGR);
    uint64_t t = get_us_clock();
    CX_FREE(hd, 0, 0);
    while (m && MEM(m + 0x108) && get_us_clock() - t < 500000) msleep(2);
    uint32_t us = (uint32_t)(get_us_clock() - t);
    if (m && MEM(m + 0x108)) cx_log("  free not done after %d us (bufs %d)", us, MEM(m + 0x108));
    msleep(10);
    return us;
}

/* CMP1 bit depth byte (offset 24) of a result, from the CPU-readable main header */
static int cx_hdr_bits(uint32_t *hd)
{
    uint32_t ha = 0, hs = 0;
    if (CX_HDR(hd, &ha, &hs) || hs < 28) return -1;
    return *(volatile uint8_t *)(cx_cpu(ha) + 24);
}

/* one encode of the current MEM1 at bpp; returns handle or 0 */
static uint32_t *cx_enc1(uint32_t w, uint32_t h, uint32_t bpp, uint32_t addr, uint32_t *us)
{
    uint32_t desc[11] = { 0, w, h, 0, 0, bpp, addr, w, 0, 0, 0 };
    int c0 = cx_cb_count;
    uint64_t t = get_us_clock();
    int rc = CX_START(desc, 1, cx_cb, 0);
    if (rc & 1) { cx_log("StartEncode %d", rc); return 0; }
    if (cx_wait(2000)) { cx_log("TIMEOUT waiting for encode (%d bpp)", bpp); return 0; }
    *us = (uint32_t)(cx_hq_us[c0 & 15] - t);
    return cx_hq[c0 & 15];
}

static void cx_stats(int phase, int bpp, uint32_t w, uint32_t h, int from, uint32_t wall_ms)
{
    uint32_t n = cx_nf - from, mn = ~0u, mx = 0; uint64_t se = 0, sb = 0, sw = 0;
    for (int i = from; i < cx_nf; i++)
    { uint32_t e = cx_f[i].enc_us; mn = MIN(mn, e); mx = MAX(mx, e); se += e; sb += cx_f[i].bytes; sw += cx_f[i].wr_us; }
    if (!n) { cx_log("phase %d %d-bit: no frames", phase, bpp); return; }
    uint32_t bp100 = (uint32_t)(sb * 800 / ((uint64_t)n * w * h));
    uint32_t fps10 = wall_ms ? n * 10000 / wall_ms : 0;
    cx_log("phase %d %d-bit: %d frames in %d ms = %d.%d fps; encode us min %d avg %d max %d; avg %d B = %d.%02d bpp;"
           " write avg %d us", phase, bpp, n, wall_ms, fps10 / 10, fps10 % 10, mn, (uint32_t)(se / n), mx,
           (uint32_t)(sb / n), bp100 / 100, bp100 % 100, (uint32_t)(sw / n));
}

/* v4.21: v4.20 hung the camera at its first stream encode (riz 2026-11-04, twice, nothing logged after the memory
 * probe): it encoded the LIVE MEM1 buffer while the raw tap kept writing it; every working run encoded a frozen
 * buffer (tap off). Now all depth/speed/pipeline phases use frozen frames, and the live case comes last, in its
 * least risky form (tap on, frame copied to an ML buffer, encoder reads the copy), after the data is saved. */
#define CX_L(k, ...) do { if ((k) < 3) cx_log(__VA_ARGS__); } while (0)   /* stage log for the first frames */

static int cx_freeze(uint32_t addr, uint32_t pitch, uint32_t h)
{
    call("lv_save_raw", 0); msleep(250);
    uint32_t s1 = cx_sum(addr, pitch, h); msleep(150);
    uint32_t s2 = cx_sum(addr, pitch, h);
    return s1 == s2;
}

static int cx_save1(uint32_t w, uint32_t h, uint32_t bpp, uint32_t addr, int frozen)
{
    uint32_t pitch = w * bpp / 8, us = 0;
    cx_log("save %d-bit: encode", bpp);
    uint32_t *hd = cx_enc1(w, h, bpp, addr, &us);
    if (!hd) return -1;
    int hb = cx_hdr_bits(hd);
    cx_log("save %d-bit: done %d us, CMP1 says %d-bit", bpp, us, hb);
    if (hb != (int)bpp) { cx_log("  depth NOT applied - stopping before more encodes of this frame"); CX_FREE(hd, 0, 0); return -1; }
    uint32_t ha = 0, hs = 0, n = 0, *ea = 0, *es = 0;
    if (!CX_HDR(hd, &ha, &hs) && !CX_RING(hd, &n, &ea, &es) && n && n <= 64)
    {
        uint32_t pk[5]; for (int i = 0; i < 5; i++) pk[i] = MEM(CX_PATH + 0x64 + 4 * i);
        struct cx_hdr H = { {'C','R','X','1'}, sizeof(struct cx_hdr), w, h, bpp, pitch,
                            {pk[0], pk[1], pk[2], pk[3], pk[4]}, addr, frozen, us / 1000, hs, n, {0} };
        uint32_t tot = 0;
        for (uint32_t i = 0; i < n; i++) { H.entry_size[i] = es[i]; tot += es[i]; }
        char fn[32]; snprintf(fn, sizeof(fn), "ML/CRXT%d.BIN", bpp);
        FILE *f = FIO_CreateFile(fn);
        if (f)
        {
            FIO_WriteFile(f, &H, sizeof(H));
            FIO_WriteFile(f, (void *)cx_cpu(ha), hs);
            for (uint32_t i = 0; i < n; i++) if (es[i]) FIO_WriteFile(f, (void *)ea[i], es[i]);
            FIO_CloseFile(f);
        }
        snprintf(fn, sizeof(fn), "ML/CRXR%d.BIN", bpp);
        f = FIO_CreateFile(fn);
        if (f) { FIO_WriteFile(f, (void *)cx_cpu(addr), pitch * h); FIO_CloseFile(f); }
        cx_log("saved %d-bit frame (%s): %d us, %d B, %d entries -> CRXT%d/CRXR%d.BIN", bpp,
               frozen ? "static" : "MOVED", us, tot, n, bpp, bpp);
    }
    cx_free_wait(hd);
    return 0;
}

static void cx_rec(int phase, int bpp, int k, uint32_t enc_us, uint32_t wr_us, uint32_t bytes)
{
    if (cx_nf >= CX_MAXF) return;
    cx_f[cx_nf].phase = phase; cx_f[cx_nf].bpp = bpp; cx_f[cx_nf].n = k;
    cx_f[cx_nf].enc_us = enc_us; cx_f[cx_nf].wr_us = wr_us; cx_f[cx_nf].bytes = bytes;
    cx_f[cx_nf].copy_us = cx_f[cx_nf].free_us = cx_f[cx_nf].cyc_us = 0; cx_nf++;
}

/* phase 2: back-to-back encodes of the frozen frame */
static int cx_burst(uint32_t w, uint32_t h, uint32_t bpp, uint32_t addr, uint32_t ms)
{
    int from = cx_nf, k = 0; uint32_t ts = get_ms_clock();
    for (; get_ms_clock() - ts < ms && cx_nf < CX_MAXF; k++)
    {
        uint32_t us = 0;
        CX_L(k, "  p2 %d-bit frame %d: start", bpp, k);
        uint32_t *hd = cx_enc1(w, h, bpp, addr, &us);
        if (!hd) return -1;
        uint32_t b = cx_frame_bytes(hd, 0);
        CX_L(k, "  p2 frame %d: %d us %d B, free", k, us, b);
        uint32_t fw = cx_free_wait(hd);
        CX_L(k, "  p2 frame %d: freed in %d us", k, fw);
        cx_rec(2, bpp, k, us, fw, b);
    }
    cx_stats(2, bpp, w, h, from, get_ms_clock() - ts);
    return 0;
}

/* phase 3: encode k+1 while frame k's chunks are written to the card */
static int cx_pipe(uint32_t w, uint32_t h, uint32_t bpp, uint32_t addr, uint32_t ms)
{
    uint32_t desc[11] = { 0, w, h, 0, 0, bpp, addr, w, 0, 0, 0 };
    cx_log("  p3: create CRXSTRM.BIN");
    FILE *f = FIO_CreateFile("ML/CRXSTRM.BIN");
    if (!f) return -1;
    int from = cx_nf, done = cx_cb_count, pipelined = 0, k = 0, ret = 0;   /* v4.23: sequential only */
    uint64_t total = 0, tstart = get_us_clock(), tsub = tstart;
    uint32_t ts = get_ms_clock();
    CX_L(0, "  p3 frame 0: start");
    int rc = CX_START(desc, 1, cx_cb, 0);
    while (!(rc & 1) && cx_nf < CX_MAXF)
    {
        if (cx_wait(2000)) { cx_log("TIMEOUT in phase 3, frame %d", k); ret = -1; break; }
        uint32_t *hd = cx_hq[done & 15];
        uint32_t enc_us = (uint32_t)(cx_hq_us[done & 15] - tsub);
        done++;
        int more = get_ms_clock() - ts < ms;
        CX_L(k, "  p3 frame %d: done %d us%s", k, enc_us, more && pipelined ? ", start next" : "");
        if (more && pipelined)
        {
            tsub = get_us_clock();
            rc = CX_START(desc, 1, cx_cb, 0);
            if (rc & 1) { cx_log("pipelined StartEncode refused (%d): sequential", rc); pipelined = 0; rc = 0; }
        }
        uint32_t n = 0, *ea = 0, *es = 0, bytes = 0;
        uint64_t tw = get_us_clock();
        if (!CX_RING(hd, &n, &ea, &es) && n <= 64)
        {
            uint32_t rec[2] = { 0x46585243 /* CRXF */, n };
            FIO_WriteFile(f, rec, 8); FIO_WriteFile(f, es, 4 * n);
            for (uint32_t i = 0; i < n; i++) if (es[i])
            {
                int wr = FIO_WriteFile(f, (void *)ea[i], es[i]);
                if (wr != (int)es[i]) { cx_log("short write %d of %d", wr, es[i]); more = 0; }
                bytes += es[i];
            }
        }
        uint32_t wr_us = (uint32_t)(get_us_clock() - tw);
        CX_L(k, "  p3 frame %d: written %d B in %d us, free", k, bytes, wr_us);
        cx_free_wait(hd);
        total += bytes;
        cx_rec(3, bpp, k, enc_us, wr_us, bytes);
        k++;
        if (!more) break;
        if (!pipelined) { tsub = get_us_clock(); rc = CX_START(desc, 1, cx_cb, 0); }
    }
    FIO_CloseFile(f);
    uint32_t el = (uint32_t)((get_us_clock() - tstart) / 1000);
    cx_stats(3, bpp, w, h, from, el);
    uint32_t mbs10 = el ? (uint32_t)(total * 10000 / (el * 1048576ull)) : 0;
    cx_log("phase 3: %s, %d MB written in %d ms = %d.%d MiB/s", pipelined ? "pipelined" : "sequential",
           (uint32_t)(total >> 20), el, mbs10 / 10, mbs10 % 10);
    return ret;
}

/* phase 4 (last, riskiest): tap ON, each frame copied to an ML buffer, the encoder reads the copy */
static int cx_live_copy(uint32_t w, uint32_t h, uint32_t ms)
{
    uint32_t pitch = w * 14 / 8, sz = pitch * h;
    struct memSuite *s = shoot_malloc_suite_contig(sz + 8192);
    if (!s) { cx_log("p4: no %d B buffer", sz); return -1; }
    uint32_t a = (uint32_t)GetMemoryAddressOfMemoryChunk(GetFirstChunkFromSuite(s));
    uint32_t dst = ((uint32_t)UNCACHEABLE(a) + 4095) & ~4095;
    uint32_t src = MEM(CX_PATH + 0x58);
    cx_log("p4: copy buffer %08x (chunk %08x), tap %d, MEM1 %08x", dst, a, MEM(CX_PATH + 0x40), src);
    int from = cx_nf, k = 0, ret = 0; uint32_t ts = get_ms_clock();
    for (; get_ms_clock() - ts < ms && cx_nf < CX_MAXF; k++)
    {
        CX_L(k, "  p4 frame %d: copy", k);
        uint64_t tc = get_us_clock();
        memcpy((void *)dst, (void *)cx_cpu(src), sz);
        uint32_t cu = (uint32_t)(get_us_clock() - tc), us = 0;
        CX_L(k, "  p4 frame %d: copied in %d us, encode", k, cu);
        uint32_t *hd = cx_enc1(w, h, 14, dst, &us);
        if (!hd) { ret = -1; break; }
        uint32_t b = cx_frame_bytes(hd, 0);
        CX_L(k, "  p4 frame %d: %d us %d B, free", k, us, b);
        cx_free_wait(hd);
        cx_rec(4, 14, k, us, cu, b);
    }
    cx_stats(4, 14, w, h, from, get_ms_clock() - ts);
    msleep(300);                        /* let algsmgr process the last free before the buffer goes */
    shoot_free_suite(s);
    return ret;
}

static void cx_write_tsv(void)
{
    FILE *f = FIO_CreateFile("ML/LOGS/CRXSTRM.TSV");
    if (!f) return;
    char ln[128];
    int k = snprintf(ln, sizeof(ln), "phase\tbpp\tframe\tenc_us\twrite_or_copy_us\tbytes\tcopy_us\tfree_us\tcycle_us\n");
    FIO_WriteFile(f, ln, k);
    for (int i = 0; i < cx_nf; i++)
    {
        k = snprintf(ln, sizeof(ln), "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", cx_f[i].phase, cx_f[i].bpp, cx_f[i].n,
                     cx_f[i].enc_us, cx_f[i].wr_us, cx_f[i].bytes, cx_f[i].copy_us, cx_f[i].free_us, cx_f[i].cyc_us);
        FIO_WriteFile(f, ln, k);
    }
    FIO_CloseFile(f);
}

static void cx_stream_task(void *unused)
{
    cx_running = 1;
    uint32_t t0 = get_ms_clock();
    cx_nf = 0;
    cx_log("---- CRX stream test (v4.24)");
    cx_f = malloc(CX_MAXF * sizeof(struct cx_fr));
    if (!cx_f) { snprintf(cx_last, sizeof(cx_last), "no memory for stats"); cx_log("%s", cx_last); cx_running = 0; return; }
    snprintf(cx_last, sizeof(cx_last), "running...");
    int tap_was = MEM(CX_PATH + 0x40), ok = 0, mem0 = -1, mem1 = -1, mem2 = -1;
    r_crx_busy = 1;
    if (!cx_sem) cx_sem = create_named_semaphore("crx_test", 0);
    while (!take_semaphore(cx_sem, 1)) ;    /* drain */
    if (!lv || RECORDING) { snprintf(cx_last, sizeof(cx_last), "needs movie LV, not recording"); goto out; }
    if (!tap_was) { call("lv_save_raw", 1); msleep(600); }
    uint32_t w = MEM(CX_PATH + 0x10), h = MEM(CX_PATH + 0x14);
    if (w != 4256 || h != 2214 || !MEM(CX_PATH + 0x40))
    { snprintf(cx_last, sizeof(cx_last), "MEM1 %dx%d: set 4K movie mode", w, h); cx_log("abort: %s", cx_last); goto out; }

    cx_mgr_log("start");
    mem0 = cx_probe_mem("before encoding");

    static const int depths[3] = { 14, 12, 10 };
    for (int d = 0; d < 3; d++)
    {
        int bpp = depths[d];
        NotifyBox(4000, "CRX stream test: %d-bit...", bpp);
        cx_log("%d-bit: pack", bpp);
        if (cx_pack_apply(bpp)) { snprintf(cx_last, sizeof(cx_last), "pack %d-bit failed", bpp); goto out_pack; }
        uint32_t addr = MEM(CX_PATH + 0x58);
        int frozen = cx_freeze(addr, w * bpp / 8, h);
        cx_log("%d-bit: MEM1 %08x frozen %d", bpp, addr, frozen);
        cx_reinit(bpp);
        if (cx_save1(w, h, bpp, addr, frozen)) { snprintf(cx_last, sizeof(cx_last), "%d-bit not applied/failed", bpp); goto out_pack; }
        if (cx_burst(w, h, bpp, addr, 2000)) { snprintf(cx_last, sizeof(cx_last), "%d-bit encode failed", bpp); goto out_pack; }
        if (bpp == 10)
        {
            NotifyBox(4000, "CRX stream test: 10-bit encode + write...");
            if (cx_pipe(w, h, bpp, addr, 3000)) { snprintf(cx_last, sizeof(cx_last), "phase 3 failed"); goto out_pack; }
        }
        call("lv_save_raw", 1); msleep(300);
        cx_pack_restore();
        cx_write_tsv();                   /* v4.22: keep each depth's numbers even if a later one hangs */
    }
    ok = 1;

out_pack:
    cx_pack_restore();
    call("lv_save_raw", 1); msleep(300);
    cx_mgr_log("after encoding");
    mem1 = cx_probe_mem("after encoding");
    cx_write_tsv();
    if (ok)
    {
        NotifyBox(4000, "CRX stream test: live copy (last step)...");
        cx_log("phase 4: live frames via a copy (tap on)");
        cx_reinit(14);
        if (cx_live_copy(w, h, 2000)) snprintf(cx_last, sizeof(cx_last), "phase 4 failed");
        cx_write_tsv();
    }
    {
        int rfa = CX_FREEALL();
        msleep(1000);
        cx_log("FreeAllEncBuffer %d", rfa);
        cx_mgr_log("after FreeAll");
        cx_release_cfg();
        mem2 = cx_probe_mem("after FreeAll");
    }
    if (ok && !strcmp(cx_last, "running..."))
        snprintf(cx_last, sizeof(cx_last), "done, %d frames; mem %d/%d/%d x16MB", cx_nf, mem0, mem1, mem2);
out:
    call("lv_save_raw", tap_was ? 1 : 0);
    r_crx_busy = 0;
    free(cx_f); cx_f = 0;
    cx_log("stream test done in %d ms: %s", get_ms_clock() - t0, cx_last);
    NotifyBox(6000, "CRX stream: %s", cx_last);
    cx_running = 0;
}

static MENU_SELECT_FUNC(cx_stream_select)
{
    if (cx_running) return;
    task_create("crx_strm", 0x1e, 0x8000, cx_stream_task, 0);
}

static MENU_UPDATE_FUNC(cx_stream_update)
{
    MENU_SET_VALUE("%s", cx_last);
    MENU_SET_HELP("~20 s: encoder speed/size at 14/12/10-bit, 10-bit encode+write to card, memory before/after.");
    MENU_SET_WARNING(MENU_WARN_INFO, "4K movie mode, LV on, not recording. Writes ~300 MB to ML/CRXSTRM.BIN.");
}

/* ======================= v4.24: Debug > "CRX pipeline test" (lossless 4K step 3b) =======================
 * v4.23 (riz 2026-11-04): bit-exact at 14/12/10-bit, encode 10-14 ms, but (a) FIO straight from the encoder's
 * chunk addresses ran ~5 MB/s in its loop (8 unaligned ~540 KB writes per frame after a 76-byte record) while the
 * frozen-frame saves wrote the same kind of chunks at ~47 MB/s; (b) a CPU memcpy of a live frame took 5.6 s;
 * (c) ~85 ms per frame of ML-side wake/sleep overhead (prio 0x1e msleep(1) = 2-46 ms in LV, dma-copy notes).
 * Stages, proven-safe first, results saved after each (ML/LOGS/CRXTEST.LOG + CRXSTRM.TSV, phase numbers 5-9):
 *  5 card baseline in this state: 6 x 16 MB aligned writes from an ML shoot buffer (file deleted afterwards)
 *  6 one frozen 10-bit result written straight from the encoder's addresses: 5x with every chunk padded to 4 KiB
 *    (all writes 4 KiB-aligned, ML/CRXDA.BIN, "CRXP" records), 3x exact/unaligned as v4.23 (file deleted)
 *  7 the real pipeline, 4 s, tap ON: recorder's EDMAC copy MEM1 -> ML buffer, encode the copy, aligned direct
 *    writes to ML/CRXLIVE.BIN, free-wait; per-frame copy / encode / write / free / cycle times
 *  8 (unknown, may hang) HPCopy (XDMAC) of a frozen result's chunks into an ML buffer -> ML/CRXT10H.BIN + CRXR10H.BIN
 *  9 (unknown, may hang) EDMAC copy of the same kind of chunks -> ML/CRXT10E.BIN + CRXR10E.BIN
 * Task priority 0x1c (the recorder's writer). */
extern int r_m2m_open(void);
extern int r_m2m_copy(uint32_t src, uint32_t src_pitch, uint32_t dst, uint32_t row_bytes, uint32_t rows, uint32_t *dma_us);
extern void r_m2m_close(void);
static int (*const HPCopy)(void *dst, void *src, uint32_t len) = (void *)0xE0328A33;   /* System::HPCopy.c */

#define AL4K(x) (((x) + 4095) & ~4095u)
#define AL64(x) (((x) + 63) & ~63u)
static struct memSuite *pl_suite[2];
static uint32_t pl_buf[2];                 /* [0] frame copy (+ record header at +15 MB), [1] staging / baseline */
#define PL_BUF_BYTES (16 * 1024 * 1024)
#define PL_HDR (pl_buf[0] + 0xF00000)

static int pl_alloc(void)
{
    for (int i = 0; i < 2; i++)
    {
        pl_suite[i] = shoot_malloc_suite_contig(PL_BUF_BYTES + 8192);
        if (!pl_suite[i]) { cx_log("buffer %d: no 16 MB", i); return -1; }
        uint32_t a = (uint32_t)GetMemoryAddressOfMemoryChunk(GetFirstChunkFromSuite(pl_suite[i]));
        pl_buf[i] = ((uint32_t)UNCACHEABLE(a) + 4095) & ~4095u;
        cx_log("buffer %d at %08x (chunk %08x)", i, pl_buf[i], a);
    }
    return 0;
}
static void pl_free(void)
{
    for (int i = 0; i < 2; i++) if (pl_suite[i]) { shoot_free_suite(pl_suite[i]); pl_suite[i] = 0; }
}

static struct cx_fr *pl_rec(int phase, int k)
{
    if (cx_nf >= CX_MAXF) return 0;
    struct cx_fr *r = &cx_f[cx_nf++];
    memset(r, 0, sizeof(*r)); r->phase = phase; r->bpp = 10; r->n = k;
    return r;
}

/* one "CRXP" record: 4 KiB header {magic, n, sizes[n]} then each chunk padded to 4 KiB, straight from the
 * encoder's addresses; returns payload bytes or -1 */
static int pl_write_rec(FILE *f, uint32_t *hd, int aligned)
{
    uint32_t n = 0, *ea = 0, *es = 0, tot = 0;
    if (CX_RING(hd, &n, &ea, &es) || !n || n > 64) return -1;
    uint32_t *h = (uint32_t *)PL_HDR;
    memset(h, 0, 4096);
    h[0] = aligned ? 0x50585243 /* CRXP */ : 0x46585243 /* CRXF */; h[1] = n;
    for (uint32_t i = 0; i < n; i++) h[2 + i] = es[i];
    if (FIO_WriteFile(f, h, aligned ? 4096 : 8 + 4 * n) <= 0) return -1;
    for (uint32_t i = 0; i < n; i++) if (es[i])
    {
        uint32_t len = aligned ? AL4K(es[i]) : es[i];
        if (FIO_WriteFile(f, (void *)ea[i], len) != (int)len) { cx_log("short write, entry %d", i); return -1; }
        tot += es[i];
    }
    return tot;
}

/* stage 5 */
static void pl_baseline(void)
{
    FILE *f = FIO_CreateFile("ML/CRXBASE.BIN");
    if (!f) return;
    uint64_t t = get_us_clock();
    for (int i = 0; i < 6; i++)
    {
        uint64_t t1 = get_us_clock();
        int w = FIO_WriteFile(f, (void *)pl_buf[1], PL_BUF_BYTES);
        struct cx_fr *r = pl_rec(5, i); if (r) { r->wr_us = (uint32_t)(get_us_clock() - t1); r->bytes = w; }
    }
    FIO_CloseFile(f);
    uint32_t us = (uint32_t)(get_us_clock() - t);
    uint32_t mbs10 = (uint32_t)(6ull * PL_BUF_BYTES * 10000000ull / ((uint64_t)us * 1048576));
    cx_log("stage 5 card baseline: 96 MiB in %d ms = %d.%d MiB/s (16 MiB aligned writes)", us / 1000, mbs10 / 10, mbs10 % 10);
    FIO_RemoveFile("ML/CRXBASE.BIN");
}

/* stage 6 */
static void pl_direct(uint32_t *hd)
{
    for (int aligned = 1; aligned >= 0; aligned--)
    {
        const char *fn = aligned ? "ML/CRXDA.BIN" : "ML/CRXUA.BIN";
        FILE *f = FIO_CreateFile(fn);
        if (!f) return;
        int reps = aligned ? 5 : 3; uint64_t tot = 0, t = get_us_clock();
        for (int i = 0; i < reps; i++)
        {
            uint64_t t1 = get_us_clock();
            int b = pl_write_rec(f, hd, aligned);
            struct cx_fr *r = pl_rec(6, i + (aligned ? 0 : 100));
            if (r) { r->wr_us = (uint32_t)(get_us_clock() - t1); r->bytes = b; }
            if (b < 0) break;
            tot += b;
        }
        FIO_CloseFile(f);
        uint32_t us = (uint32_t)(get_us_clock() - t);
        uint32_t mbs10 = us ? (uint32_t)(tot * 10000000ull / ((uint64_t)us * 1048576)) : 0;
        cx_log("stage 6 %s direct writes: %d frames, %d KB in %d ms = %d.%d MiB/s", aligned ? "4K-aligned" : "unaligned",
               reps, (uint32_t)(tot >> 10), us / 1000, mbs10 / 10, mbs10 % 10);
        if (!aligned) FIO_RemoveFile(fn);
    }
}

/* stage 7: tap on, pack 10-bit applied */
static int pl_live(uint32_t w, uint32_t h, uint32_t ms)
{
    uint32_t pitch = w * 10 / 8, src = MEM(CX_PATH + 0x58);
    int st = r_m2m_open();
    cx_log("stage 7: EDMAC open %d, MEM1 %08x pitch %d, tap %d", st, src, pitch, MEM(CX_PATH + 0x40));
    if (st) return -1;
    FILE *f = FIO_CreateFile("ML/CRXLIVE.BIN");
    if (!f) { r_m2m_close(); return -1; }
    int k = 0, ret = 0; uint64_t tot = 0, t0 = get_us_clock();
    uint32_t ts = get_ms_clock();
    for (; get_ms_clock() - ts < ms && cx_nf < CX_MAXF; k++)
    {
        uint64_t tc = get_us_clock();
        uint32_t dma = 0;
        CX_L(k, "  p7 frame %d: copy", k);
        if (r_m2m_copy(src, pitch, pl_buf[0], pitch, h, &dma)) { cx_log("  p7 frame %d: EDMAC copy failed", k); ret = -1; break; }
        uint32_t cu = (uint32_t)(get_us_clock() - tc), eu = 0;
        CX_L(k, "  p7 frame %d: copied %d us (dma %d), encode", k, cu, dma);
        uint32_t *hd = cx_enc1(w, h, 10, pl_buf[0], &eu);
        if (!hd) { ret = -1; break; }
        uint64_t tw = get_us_clock();
        int b = pl_write_rec(f, hd, 1);
        uint32_t wu = (uint32_t)(get_us_clock() - tw);
        CX_L(k, "  p7 frame %d: encoded %d us, wrote %d B in %d us, free", k, eu, b, wu);
        uint32_t fu = cx_free_wait(hd);
        struct cx_fr *r = pl_rec(7, k);
        if (r) { r->copy_us = cu; r->enc_us = eu; r->wr_us = wu; r->bytes = b; r->free_us = fu;
                 r->cyc_us = (uint32_t)(get_us_clock() - tc); }
        if (b < 0) { ret = -1; break; }
        tot += b;
    }
    FIO_CloseFile(f);
    r_m2m_close();
    uint32_t us = (uint32_t)(get_us_clock() - t0);
    uint32_t fps10 = us ? (uint32_t)(k * 10000000ull / us) : 0, mbs10 = us ? (uint32_t)(tot * 10000000ull / ((uint64_t)us * 1048576)) : 0;
    cx_log("stage 7 live pipeline: %d frames in %d ms = %d.%d fps, %d.%d MiB/s compressed to card", k, us / 1000,
           fps10 / 10, fps10 % 10, mbs10 / 10, mbs10 % 10);
    return ret;
}

/* stages 8/9: copy a frozen result's chunks into pl_buf[1] with HPCopy (mode 0) or EDMAC (mode 1), save in the
 * CRXT format (+ the raw frame) for the PC bit-exact check */
static int pl_copy_chunks(uint32_t w, uint32_t h, uint32_t addr, int mode)
{
    const char *nm = mode ? "EDMAC" : "HPCopy";
    uint32_t us = 0, *hd = cx_enc1(w, h, 10, addr, &us);
    if (!hd) return -1;
    uint32_t ha = 0, hs = 0, n = 0, *ea = 0, *es = 0;
    if (CX_HDR(hd, &ha, &hs) || CX_RING(hd, &n, &ea, &es) || !n || n > 64) { cx_free_wait(hd); return -1; }
    if (mode && r_m2m_open()) { cx_log("stage 9: EDMAC busy"); cx_free_wait(hd); return -1; }
    uint32_t off = 0, slot[64], tcopy = 0;
    int ok = 1;
    for (uint32_t i = 0; i < n; i++)
    {
        slot[i] = pl_buf[1] + off;
        if (!es[i]) continue;
        uint32_t len = AL64(es[i]);
        if (i < 2) cx_log("  %s entry %d: %08x -> %08x, %d B", nm, i, ea[i], slot[i], len);
        uint64_t t = get_us_clock();
        int r;
        if (mode) { uint32_t d = 0; uint32_t rows = AL4K(es[i]) / 4096; r = r_m2m_copy(ea[i], 4096, slot[i], 4096, rows, &d); }
        else { int tries = 0; while ((r = HPCopy((void *)slot[i], (void *)ea[i], len)) == 1 && tries++ < 50) msleep(1); }
        tcopy += (uint32_t)(get_us_clock() - t);
        if (r) { cx_log("  %s entry %d returned %d", nm, i, r); ok = 0; break; }
        off += AL4K(es[i]);
    }
    if (mode) r_m2m_close();
    uint32_t tot = 0; for (uint32_t i = 0; i < n; i++) tot += es[i];
    struct cx_fr *rr = pl_rec(8 + mode, 0); if (rr) { rr->copy_us = tcopy; rr->bytes = tot; rr->enc_us = us; }
    if (ok)
    {
        uint32_t z = 0, ff = 0, *q = (uint32_t *)slot[1];
        for (int i = 0; i < 256; i++) { z += q[i] == 0; ff += q[i] == 0xFFFFFFFF; }
        uint32_t mbs10 = tcopy ? (uint32_t)((uint64_t)tot * 10000000ull / ((uint64_t)tcopy * 1048576)) : 0;
        cx_log("stage %d %s: %d B in %d us = %d.%d MiB/s; first 256 words of entry 1: %d zero, %d 0xFFFFFFFF, first %08x %08x",
               8 + mode, nm, tot, tcopy, mbs10 / 10, mbs10 % 10, z, ff, q[0], q[1]);
        uint32_t pk[5]; for (int i = 0; i < 5; i++) pk[i] = MEM(CX_PATH + 0x64 + 4 * i);
        struct cx_hdr H = { {'C','R','X','1'}, sizeof(struct cx_hdr), w, h, 10, w * 10 / 8,
                            {pk[0], pk[1], pk[2], pk[3], pk[4]}, addr, 1, us / 1000, hs, n, {0} };
        for (uint32_t i = 0; i < n; i++) H.entry_size[i] = es[i];
        FILE *f = FIO_CreateFile(mode ? "ML/CRXT10E.BIN" : "ML/CRXT10H.BIN");
        if (f)
        {
            FIO_WriteFile(f, &H, sizeof(H));
            FIO_WriteFile(f, (void *)cx_cpu(ha), hs);
            for (uint32_t i = 0; i < n; i++) if (es[i]) FIO_WriteFile(f, (void *)slot[i], es[i]);
            FIO_CloseFile(f);
        }
        f = FIO_CreateFile(mode ? "ML/CRXR10E.BIN" : "ML/CRXR10H.BIN");
        if (f) { FIO_WriteFile(f, (void *)cx_cpu(addr), w * 10 / 8 * h); FIO_CloseFile(f); }
    }
    cx_free_wait(hd);
    return ok ? 0 : -1;
}

static void pl_task(void *unused)
{
    cx_running = 1;
    uint32_t t0 = get_ms_clock();
    cx_nf = 0;
    cx_log("---- CRX pipeline test (v4.24)");
    cx_f = malloc(CX_MAXF * sizeof(struct cx_fr));
    if (!cx_f) { snprintf(cx_last, sizeof(cx_last), "no memory for stats"); cx_running = 0; return; }
    snprintf(cx_last, sizeof(cx_last), "running...");
    int tap_was = MEM(CX_PATH + 0x40), stage = 0;
    r_crx_busy = 1;
    if (!cx_sem) cx_sem = create_named_semaphore("crx_test", 0);
    while (!take_semaphore(cx_sem, 1)) ;
    if (!lv || RECORDING) { snprintf(cx_last, sizeof(cx_last), "needs movie LV, not recording"); goto out; }
    if (!tap_was) { call("lv_save_raw", 1); msleep(600); }
    uint32_t w = MEM(CX_PATH + 0x10), h = MEM(CX_PATH + 0x14);
    if (w != 4256 || h != 2214 || !MEM(CX_PATH + 0x40))
    { snprintf(cx_last, sizeof(cx_last), "MEM1 %dx%d: set 4K movie mode", w, h); cx_log("abort: %s", cx_last); goto out; }
    if (pl_alloc()) { snprintf(cx_last, sizeof(cx_last), "no buffers"); goto out_mem; }

    NotifyBox(3000, "CRX pipeline: card baseline...");
    stage = 5; pl_baseline(); cx_write_tsv();

    NotifyBox(3000, "CRX pipeline: direct writes...");
    if (cx_pack_apply(10)) { snprintf(cx_last, sizeof(cx_last), "pack 10-bit failed"); goto out_pack; }
    uint32_t addr = MEM(CX_PATH + 0x58);
    int frozen = cx_freeze(addr, w * 10 / 8, h);
    cx_reinit(10);
    {
        uint32_t us = 0, *hd = cx_enc1(w, h, 10, addr, &us);
        if (!hd || cx_hdr_bits(hd) != 10)
        { if (hd) cx_free_wait(hd); snprintf(cx_last, sizeof(cx_last), "10-bit encode failed"); goto out_pack; }
        cx_log("stage 6: frozen %d, encode %d us", frozen, us);
        stage = 6; pl_direct(hd); cx_free_wait(hd); cx_write_tsv();
    }

    NotifyBox(5000, "CRX pipeline: live (EDMAC copy + encode + write)...");
    cx_pack_restore();                    /* tap on, then the 10-bit packing applied fresh (don't assume it survived) */
    call("lv_save_raw", 1); msleep(400);
    if (cx_pack_apply(10)) { snprintf(cx_last, sizeof(cx_last), "pack 10-bit (live) failed"); goto out_pack; }
    cx_log("stage 7 pack check: f2 %d f3 %d width field %d", MEM(CX_PATH + 0x64 + 8), MEM(CX_PATH + 0x64 + 12), MEM(CX_PATH + 0x18));
    if (MEM(CX_PATH + 0x64 + 8) != 0 || MEM(CX_PATH + 0x64 + 12) != 1 || MEM(CX_PATH + 0x18) != w * 10 / 14)
    { snprintf(cx_last, sizeof(cx_last), "10-bit packing not in effect"); goto out_pack; }
    stage = 7;
    if (pl_live(w, h, 4000)) { snprintf(cx_last, sizeof(cx_last), "stage 7 failed"); }
    cx_write_tsv();

    NotifyBox(3000, "CRX pipeline: HPCopy test...");
    frozen = cx_freeze(addr, w * 10 / 8, h);
    stage = 8;
    if (pl_copy_chunks(w, h, addr, 0)) cx_log("stage 8 HPCopy: failed");
    cx_write_tsv();
    NotifyBox(3000, "CRX pipeline: EDMAC chunk test...");
    stage = 9;
    if (pl_copy_chunks(w, h, addr, 1)) cx_log("stage 9 EDMAC: failed");
    cx_write_tsv();
    stage = 10;

out_pack:
    cx_pack_restore();
    call("lv_save_raw", 1); msleep(300);
    {
        int rfa = CX_FREEALL();
        msleep(1000);
        cx_log("FreeAllEncBuffer %d", rfa);
        cx_release_cfg();
    }
out_mem:
    pl_free();
    if (!strcmp(cx_last, "running...")) snprintf(cx_last, sizeof(cx_last), "done (stage %d), %d records", stage, cx_nf);
out:
    call("lv_save_raw", tap_was ? 1 : 0);
    r_crx_busy = 0;
    free(cx_f); cx_f = 0;
    cx_log("pipeline test done in %d ms: %s", get_ms_clock() - t0, cx_last);
    NotifyBox(6000, "CRX pipeline: %s", cx_last);
    cx_running = 0;
}

static MENU_SELECT_FUNC(pl_select)
{
    if (cx_running) return;
    task_create("crx_pipe", 0x1c, 0x8000, pl_task, 0);
}

static MENU_UPDATE_FUNC(pl_update)
{
    MENU_SET_VALUE("%s", cx_last);
    MENU_SET_HELP("~25 s: card baseline, direct writes, live EDMAC copy+encode+write, HPCopy/EDMAC of encoder output.");
    MENU_SET_WARNING(MENU_WARN_INFO, "4K movie mode, LV on, not recording. Writes ~250 MB under ML/.");
}

/* ======================= v4.25: lossless recorder API (rectest_r.c, Movie > RAW video > Compression) =============
 * Same rules the tests proved (STEP1.md steps 2-3b): encode a copy, never the live MEM1; re-init the encoder for the
 * clip's size/depth (flag cleared at begin); wait for each free before the next start; HPCopy the output out (the CPU
 * can't read the chunks); hand the encoder back to Canon at the end. Errors go to ML/LOGS/CRXTEST.LOG. */
int crx_rec_begin(void)
{
    if (!cx_sem) cx_sem = create_named_semaphore("crx_test", 0);
    while (!take_semaphore(cx_sem, 1)) ;
    uint32_t m = MEM(CX_MGR);
    if (!m) return -1;
    MEM(m + 0x124) = 0;                   /* next StartEncode re-runs Canon's init with the clip's descriptor */
    cx_cfg_bpp = 0;
    return 0;
}
uint32_t *crx_rec_encode(uint32_t w, uint32_t h, uint32_t bpp, uint32_t addr, uint32_t *us)
{
    return cx_enc1(w, h, bpp, addr, us);
}
int crx_rec_parts(uint32_t *hd, uint32_t *ha, uint32_t *hs, uint32_t *n, uint32_t **ea, uint32_t **es)
{
    if (CX_HDR(hd, ha, hs)) return -1;
    if (CX_RING(hd, n, ea, es) || !*n || *n > 32) return -2;
    return 0;
}
uint32_t crx_rec_free(uint32_t *hd)        /* free, wait until algsmgr has processed it (no extra gap) */
{
    uint32_t m = MEM(CX_MGR);
    uint64_t t = get_us_clock();
    CX_FREE(hd, 0, 0);
    while (m && MEM(m + 0x108) && get_us_clock() - t < 500000) msleep(1);
    return (uint32_t)(get_us_clock() - t);
}
int crx_rec_copy(uint32_t dst, uint32_t src, uint32_t len)   /* HPCopy (XDMAC); 1 = no free channel, retried */
{
    int r, tries = 0;
    while ((r = HPCopy((void *)dst, (void *)src, len)) == 1 && tries++ < 100) msleep(1);
    return r;
}
void crx_rec_end(void)
{
    CX_FREEALL();
    msleep(300);
    uint32_t m = MEM(CX_MGR);
    if (m) MEM(m + 0x124) = 0;            /* Canon's next encode (stills) re-inits from its own descriptor */
    cx_cfg_bpp = 0;
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
    { .name = "CRX stream test", .select = cx_stream_select, .update = cx_stream_update, .icon_type = IT_ACTION },
    { .name = "CRX pipeline test", .select = pl_select, .update = pl_update, .icon_type = IT_ACTION },
};

static void cx_init(void *unused) { menu_add("Debug", cx_menu, COUNT(cx_menu)); }
INIT_FUNC("crx_test", cx_init);
