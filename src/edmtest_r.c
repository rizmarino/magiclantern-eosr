/* riz/eosr-dmlog v11: DMA copy test (EOS R 1.8.0). Personal diagnostic build,
 * not for upstream.
 *
 * v10 showed CPU memcpy from the MEM1 raw buffer runs at ~10 MiB/s, 10x too
 * slow for 1080p. This tests the two DMA engines found by static analysis
 * (notes/lv-survey.md "stage 3"):
 *  - EDMAC memory-to-memory via Canon's MemoryToMemoryEsub1 driver, exactly
 *    as Canon's EFsCropVcopy uses it: same resource lock {0x100AD,0x100BB},
 *    same port pair (read 0x2E -> write 0x0D), same power-domain call,
 *    bitmode 0. 2D copy: xb bytes per line, yb+1 lines, off1b skipped after
 *    each source line, so it can cut a crop out of the full frame.
 *  - XDMAC linear copy via Canon's HPCopy(dst, src, len).
 * Phases (log ML/EDMTEST.TXT, flushed after each):
 *  1. state before (m2m driver state, port registers, lock check)
 *  2. synthetic copies on our own buffers, verified word by word
 *     (EDMAC twice, HPCopy)
 *  3. lv_save_raw(1); live timing: EDMAC 1080p crop copies, HPCopy bands,
 *     HPCopy per row
 *  4. 3 s burst: each new frame's 1080p crop copied by EDMAC (Canon's full
 *     setup/cleanup per copy, as EFsCropVcopy does) into a ring of 4
 *     buffers, a writer task writes ML/EDMREC.BIN; counts misses/tears
 *  5. lv_save_raw(0); last: EDMAC x3 after a single setup (verified)
 * Every DMA wait has a timeout; on a timeout EDMAC is reset, released and
 * not used again in this run. No ROM writes, no property changes.
 */
#include "dryos.h"
#include "menu.h"
#include "propvalues.h"
#include "timer.h"
#include "mem.h"

/* ---- Canon functions (EOS R 1.8.0, Thumb: address | 1) ---- */
#ifndef ET_HOST
static void *(*const ErscCreateLockEntry)(uint32_t *ids, int n) = (void *)0xE054761F;
static int   (*const ErscLock)(void *entry)                     = (void *)0xE0547807;
static int   (*const ErscUnlock)(void *entry)                   = (void *)0xE05478A7;
static int   (*const ErscDelete)(void *entry)                   = (void *)0xE0547703;
static int   (*const ErscCheck)(void *entry)                    = (void *)0xE0547799; /* 0 free, 0x1F busy */
static void  (*const PwrSubChipOn)(const void *list)            = (void *)0xE0671743;
static void  (*const PwrSubChipOff)(const void *list)           = (void *)0xE0671771;
static void  (*const M2mSetup)(uint32_t *ports)                 = (void *)0xE0712BAD; /* {rd, wr} */
static void  (*const M2mSetCbr)(void (*cbr)(void *), void *ctx) = (void *)0xE0712CA1;
static void  (*const M2mResetCbr)(void)                         = (void *)0xE0712CAB;
static void  (*const M2mSetAddr)(uint32_t *addr)                = (void *)0xE0712D87; /* {src, dst} */
static void  (*const M2mSetInfo)(uint32_t *p)                   = (void *)0xE0712DA1; /* {rd_info*, wr_info*, bitmode} */
static void  (*const M2mStart)(void)                            = (void *)0xE0712CB7;
static void  (*const EdmacUnregCbr)(int port)                   = (void *)0xE0535E17;
static void  (*const EdmacBoomerOff)(int port)                  = (void *)0xE0535F77;
static void  (*const EdmacReset)(int port)                      = (void *)0xE0535999;
static int   (*const HPCopy)(void *dst, void *src, uint32_t len) = (void *)0xE0328A33;
#define ET_DMA_TIMER() GET_DIGIC_TIMER()
#else
#include "et_host.h"
#endif

#define ERSC_MGR       0x10924          /* +0x0C: recursive lock of the resource manager */
#define PWR_LIST_VCOPY ((const void *)0xE0F2673C)   /* {0, 7}: sub-chip 0 */
#define M2M_STATE      0x243A8          /* Esub1 globals: cbr, ctx, .., +0x10 rd, +0x14 wr, +0x18 done bits */
#define ET_RD_PORT     0x2E
#define ET_WR_PORT     0x0D
#define ET_PORT_MMIO(p) ((p) == ET_RD_PORT ? 0xD0421600 : 0xD0420C00)

#define ET_PATH    0x133CC              /* Mem1Path struct */
#define ET_BUF     0x28EEA940
#define ET_BUFLEN  0xFB9D90
#define ET_UNC     0x40000000

#define ET_NBUF     4
#define ET_BURST_US 3000000
#define ET_MAXFR    200
#define ET_LOGLEN   0x8000
#define ET_TIMEOUT  500                 /* ms per DMA wait */

static volatile int et_busy = 0;
static int et_runs = 0;

static char *et_logbuf;
static int et_loglen;

static void et_printf(const char *fmt, ...)
{
    if (!et_logbuf || et_loglen > ET_LOGLEN - 200) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(et_logbuf + et_loglen, ET_LOGLEN - et_loglen, fmt, ap);
    va_end(ap);
    if (n > 0) et_loglen += MIN(n, ET_LOGLEN - et_loglen - 1);
}

static void et_flush(void)
{
    if (!et_logbuf || !et_loglen) return;
    FILE *f = FIO_CreateFileOrAppend("ML/EDMTEST.TXT");
    if (f)
    {
        FIO_WriteFile(f, et_logbuf, et_loglen);
        FIO_CloseFile(f);
    }
    et_loglen = 0;
}

static inline uint32_t et_us(void) { return (uint32_t)get_us_clock(); }

static void et_words(const char *tag, uint32_t addr, int n)
{
    for (int i = 0; i < n; i += 8)
    {
        et_printf("%s %08x:", tag, addr + i * 4);
        for (int k = i; k < n && k < i + 8; k++) et_printf(" %08x", MEM(addr + k * 4));
        et_printf("\n");
    }
}

static void et_ports(const char *when)
{
    et_printf("-- ports %s; m2m state:", when);
    for (int i = 0; i < 8; i++) et_printf(" %08x", MEM(M2M_STATE + 4 * i));
    et_printf("\n");
    /* same register windows the v9 MEM1 test read safely on port 3 */
    for (int k = 0; k < 2; k++)
    {
        uint32_t b = ET_PORT_MMIO(k ? ET_RD_PORT : ET_WR_PORT);
        const char *tag = k ? "P2E" : "P0D";
        et_words(tag, b, 4);
        et_words(tag, b + 0x40, 16);
        et_words(tag, b + 0xA0, 8);
    }
}

/* ---- EDMAC m2m session ---- */
static uint32_t et_res_ids[2] = { 0x000100AD, 0x000100BB };
static void *et_entry;
static int et_locked, et_edmac_ok;
static struct semaphore *et_sem;
static volatile uint32_t et_cbr_t, et_cbr_n;

struct et_info { uint32_t w[15]; };   /* off1s off1a off1b off2s off2a off2b off3 xs xa xb ys ya yb xn yn */
static struct et_info et_ri, et_wi;

static void et_cbr(void *ctx)          /* interrupt context: timestamp + wake the task */
{
    et_cbr_t = ET_DMA_TIMER();
    et_cbr_n++;
    give_semaphore(et_sem);
}

static int et_lock(void)
{
    if (!et_entry) et_entry = ErscCreateLockEntry(et_res_ids, 2);
    if (!et_entry) return -1;
    void *rl = (void *)MEM(ERSC_MGR + 0x0C);
    AcquireRecursiveLock(rl, 0);
    int st = ErscCheck(et_entry);
    ReleaseRecursiveLock(rl);
    if (st) return st;                  /* someone holds 0x100AD/0x100BB: don't wait */
    st = ErscLock(et_entry);
    et_locked = !st;
    return st;
}

static void et_unlock(void)
{
    if (et_locked) { int u = ErscUnlock(et_entry); et_printf("unlock %d\n", u); et_locked = 0; }
    if (et_entry) { int d = ErscDelete(et_entry); et_printf("delete lock entry %d\n", d); et_entry = 0; }
}

static void et_begin(void)
{
    PwrSubChipOn(PWR_LIST_VCOPY);
    uint32_t ports[2] = { ET_RD_PORT, ET_WR_PORT };
    M2mSetup(ports);
    M2mSetCbr(et_cbr, 0);
}

static void et_end(void)
{
    M2mResetCbr();
    EdmacUnregCbr(ET_RD_PORT);
    EdmacUnregCbr(ET_WR_PORT);
    EdmacBoomerOff(ET_WR_PORT);
    PwrSubChipOff(PWR_LIST_VCOPY);
}

/* one 2D copy: rows x row_bytes from src (line pitch src_pitch) to dst (packed).
 * Returns 0 ok, else timeout; *dma_us = start -> completion interrupt. */
static int et_copy(uint32_t src, uint32_t src_pitch, uint32_t dst, uint32_t row_bytes, uint32_t rows, uint32_t *dma_us)
{
    memset(&et_ri, 0, sizeof(et_ri));
    memset(&et_wi, 0, sizeof(et_wi));
    et_ri.w[2] = src_pitch - row_bytes; et_ri.w[9] = row_bytes; et_ri.w[12] = rows - 1;
    et_wi.w[9] = row_bytes; et_wi.w[12] = rows - 1;
    uint32_t a[2] = { src, dst };
    uint32_t p[3] = { (uint32_t)&et_ri, (uint32_t)&et_wi, 0 };
    M2mSetAddr(a);
    M2mSetInfo(p);
    uint32_t n0 = et_cbr_n;
    uint32_t t0 = ET_DMA_TIMER();
    M2mStart();
    int r = take_semaphore(et_sem, ET_TIMEOUT);
    if (r || et_cbr_n == n0) { *dma_us = 0; return r ? r : -1; }
    *dma_us = (et_cbr_t - t0) & 0xFFFFF;
    return 0;
}

/* after a timeout: log, reset the ports, release everything, stop using EDMAC */
static void et_fail(const char *where)
{
    et_printf("!! EDMAC timeout in %s, cbr count %d\n", where, et_cbr_n);
    et_ports("after timeout");
    EdmacReset(ET_RD_PORT);
    EdmacReset(ET_WR_PORT);
    et_end();
    et_edmac_ok = 0;
    et_flush();
}

/* ---- buffers ---- */
static void *et_alloc(uint32_t len, void **raw)
{
    *raw = fio_malloc(len + 256);
    if (!*raw) return 0;
    return (void *)(((uint32_t)*raw + 127) & ~127);    /* EDMAC_ALIGNMENT 0x80 */
}

/* ---- phase 2: synthetic, verified ---- */
#define SY_PITCH 2048
#define SY_ROWS  96
#define SY_X0B   256
#define SY_RB    1024
#define SY_CROWS 64
#define SY_GUARD 256

static int et_verify(uint32_t *dst, int y0, const char *what)
{
    int bad = 0;
    for (int r = 0; r < SY_CROWS; r++)
        for (int c = 0; c < SY_RB / 4; c++)
        {
            uint32_t want = ((y0 + r) << 16) | (SY_X0B / 4 + c), got = dst[r * (SY_RB / 4) + c];
            if (got != want && bad++ < 4)
                et_printf("  %s mismatch row %d word %d: got %08x (src row %d word %d) want %08x\n",
                          what, r, c, got, got >> 16, got & 0xFFFF, want);
        }
    int gbad = 0;
    for (int i = 0; i < SY_GUARD / 4; i++) if (dst[SY_CROWS * SY_RB / 4 + i] != 0xDEADBEEF) gbad++;
    et_printf("%s: %s, %d bad words, guard %s\n", what, bad ? "FAIL" : "OK", bad, gbad ? "OVERWRITTEN" : "intact");
    return bad || gbad;
}

static void et_fill_dst(uint32_t *dst)
{
    for (int i = 0; i < (SY_CROWS * SY_RB + SY_GUARD) / 4; i++) dst[i] = 0xDEADBEEF;
}

static int et_single_ok, et_hp_ok, et_hp_cached;

static void et_synthetic(void)
{
    void *rs = 0, *rd = 0;
    uint32_t *src = et_alloc(SY_PITCH * SY_ROWS, &rs);
    uint32_t *dst = et_alloc(SY_CROWS * SY_RB + SY_GUARD, &rd);
    if (!src || !dst) { et_printf("synthetic: no memory\n"); goto out; }
    src = UNCACHEABLE(src); dst = UNCACHEABLE(dst);
    for (int r = 0; r < SY_ROWS; r++)
        for (int c = 0; c < SY_PITCH / 4; c++) src[r * (SY_PITCH / 4) + c] = (r << 16) | c;
    et_printf("== synthetic: src %08x pitch %d, crop x0 %d bytes, %d bytes x %d rows, dst %08x\n",
              (uint32_t)src, SY_PITCH, SY_X0B, SY_RB, SY_CROWS, (uint32_t)dst);

    if (et_edmac_ok)
    {
        uint32_t us, t;
        et_fill_dst(dst);
        et_printf("EDM single: begin\n"); et_flush();
        t = et_us(); et_begin(); uint32_t tb = et_us() - t;
        int r = et_copy((uint32_t)src + 16 * SY_PITCH + SY_X0B, SY_PITCH, (uint32_t)dst, SY_RB, SY_CROWS, &us);
        if (r) { et_fail("synthetic single"); goto hp; }
        t = et_us(); et_end(); uint32_t te = et_us() - t;
        et_printf("EDM single: begin %d us, dma %d us, end %d us\n", tb, us, te);
        et_ports("after synthetic single");
        et_single_ok = !et_verify(dst, 16, "EDM single");
        et_flush();
        /* a second, different copy: catches a port that only works once per setup... */
        et_fill_dst(dst);
        et_begin();
        r = et_copy((uint32_t)src + 24 * SY_PITCH + SY_X0B, SY_PITCH, (uint32_t)dst, SY_RB, SY_CROWS, &us);
        if (r) { et_fail("synthetic second"); goto hp; }
        et_end();
        et_single_ok &= !et_verify(dst, 24, "EDM second");
        et_flush();
    }
hp:
    /* HPCopy is linear: 64 KB from src row 16 = 32 whole src rows. Try the
     * uncached addresses first (as EDMAC uses), then the cached alias. */
    for (int k = 0; k < 2; k++)
    {
        et_fill_dst(dst);
        uint32_t *d = k ? CACHEABLE(dst) : dst;
        uint8_t *sp = (uint8_t *)(k ? CACHEABLE(src) : src) + 16 * SY_PITCH;
        et_printf("HPCopy %s: calling\n", k ? "cached" : "uncached"); et_flush();
        uint32_t t = et_us();
        int r = HPCopy(d, sp, SY_CROWS * SY_RB);
        t = et_us() - t;
        int bad = 0;
        for (int i = 0; i < SY_CROWS * SY_RB / 4; i++)
        {
            uint32_t want = ((16 + i / (SY_PITCH / 4)) << 16) | (i % (SY_PITCH / 4));
            if (dst[i] != want && bad++ < 3) et_printf("  HP mismatch word %d: got %08x want %08x\n", i, dst[i], want);
        }
        et_printf("HPCopy %s %d bytes: ret %d, %d us, %s (%d bad)\n", k ? "cached" : "uncached",
                  SY_CROWS * SY_RB, r, t, bad ? "FAIL" : "OK", bad);
        if (!bad) { et_hp_cached = k; et_hp_ok = 1; break; }
    }
out:
    if (rs) free(rs);
    if (rd) free(rd);
    et_flush();
}

/* last experiment (a hang here can't affect the other results): does the
 * port pair accept further starts after one setup, as a recorder would want? */
static void et_repeat(void)
{
    void *rs = 0, *rd = 0;
    uint32_t *src = et_alloc(SY_PITCH * SY_ROWS, &rs);
    uint32_t *dst = et_alloc(SY_CROWS * SY_RB + SY_GUARD, &rd);
    if (!src || !dst) { et_printf("repeat: no memory\n"); goto out; }
    src = UNCACHEABLE(src); dst = UNCACHEABLE(dst);
    for (int r = 0; r < SY_ROWS; r++)
        for (int c = 0; c < SY_PITCH / 4; c++) src[r * (SY_PITCH / 4) + c] = (r << 16) | c;
    et_printf("== repeat: 3 copies after one setup\n"); et_flush();
    et_begin();
    for (int k = 0; k < 3 && et_edmac_ok; k++)
    {
        uint32_t us;
        et_fill_dst(dst);
        if (et_copy((uint32_t)src + (8 + 8 * k) * SY_PITCH + SY_X0B, SY_PITCH, (uint32_t)dst, SY_RB, SY_CROWS, &us))
        { et_fail("repeat"); break; }
        et_printf("EDM repeat %d: dma %d us\n", k, us);
        et_verify(dst, 8 + 8 * k, "EDM repeat");
    }
    if (et_edmac_ok) et_end();
out:
    if (rs) free(rs);
    if (rd) free(rd);
    et_flush();
}

/* ---- live ---- */
static uint32_t et_w, et_h, et_pitch, et_ubuf, et_period_us;

static uint32_t et_probe(uint32_t row)
{
    uint32_t h = 0x811C9DC5;
    for (int s = 1; s <= 3; s++)
    {
        const volatile uint32_t *p = (const volatile uint32_t *)
            (et_ubuf + row * et_pitch + ((et_pitch * s / 4) & ~3));
        for (int i = 0; i < 4; i++) h = (h ^ p[i]) * 0x01000193;
    }
    return h;
}

static uint32_t et_probe_stable(uint32_t row)
{
    uint32_t a = et_probe(row);
    uint32_t t0 = et_us(), t = t0;
    for (;;)
    {
        uint32_t now = et_us();
        if (now - t >= 50 || now - t0 > 2000) break;
        uint32_t b = et_probe(row);
        if (b != a) { a = b; t = now; }
    }
    return a;
}

static void et_crop(uint32_t want_w, uint32_t want_h, uint32_t *x0, uint32_t *y0, uint32_t *cw, uint32_t *ch)
{
    *cw = MIN(want_w, et_w) & ~63;      /* 64 px = 112 bytes: keeps rows 16-byte aligned */
    *ch = MIN(want_h, et_h) & ~1;
    *x0 = ((et_w - *cw) / 2) & ~63;
    *y0 = ((et_h - *ch) / 2) & ~1;
}

/* frame period from 12 changes of the crop's bottom row */
static void et_period(uint32_t row)
{
    uint32_t prev = et_probe_stable(row), t0 = et_us(), first = 0, last = 0;
    int n = 0;
    while (n < 12 && et_us() - t0 < 1000000)
    {
        if (et_probe(row) == prev) continue;
        prev = et_probe_stable(row);
        uint32_t now = et_us();
        if (n && now - last < 3000) continue;
        if (!n) first = now;
        last = now; n++;
    }
    et_period_us = n >= 3 ? (last - first) / (n - 1) : 0;
    et_printf("PERIOD %d us (%d changes)\n", et_period_us, n);
}

static uint8_t *ring[ET_NBUF];
static void *ring_raw[ET_NBUF];
static int nring;
static uint32_t ring_bytes;

static void et_live_timing(uint32_t x0, uint32_t y0, uint32_t cw, uint32_t ch)
{
    uint32_t rb = cw * 14 / 8;
    uint32_t src = et_ubuf + y0 * et_pitch + x0 * 14 / 8;
    et_printf("== live: crop %dx%d at (%d,%d), src %08x, %d bytes/row, pitch %d\n", cw, ch, x0, y0, src, rb, et_pitch);

    if (et_edmac_ok)
    {
        et_printf("EDM live: first copy\n"); et_flush();
        for (int k = 0; k < 5 && et_edmac_ok; k++)
        {
            uint32_t us, t = et_us();
            et_begin();
            int r = et_copy(src, et_pitch, (uint32_t)ring[0], rb, ch, &us);
            if (r) { et_fail("live single"); break; }
            et_end();
            t = et_us() - t;
            uint32_t mibs10 = us ? (uint32_t)((uint64_t)rb * ch * 10000000ULL / us / (1024 * 1024)) : 0;
            et_printf("EDM live %d: dma %d us (%d.%d MiB/s), with begin/end %d us\n", k, us, mibs10 / 10, mibs10 % 10, t);
            msleep(20);
        }
        et_flush();
    }

    if (!et_hp_ok) { et_printf("HPCopy live tests skipped (synthetic failed)\n"); et_flush(); return; }
    uint8_t *hd = et_hp_cached ? CACHEABLE(ring[0]) : ring[0];
    uint32_t hs = et_hp_cached ? (uint32_t)CACHEABLE(et_ubuf) : et_ubuf;
    /* HPCopy: full-width band of the crop's rows (as much as fits a ring buffer) */
    uint32_t band = MIN(et_pitch * ch, ring_bytes) & ~127;
    for (int k = 0; k < 3; k++)
    {
        uint32_t t = et_us();
        int r = HPCopy(hd, (void *)(hs + y0 * et_pitch), band);
        t = et_us() - t;
        uint32_t mibs10 = t ? (uint32_t)((uint64_t)band * 10000000ULL / t / (1024 * 1024)) : 0;
        et_printf("HP band %d: %d bytes, ret %d, %d us (%d.%d MiB/s)\n", k, band, r, t, mibs10 / 10, mibs10 % 10);
        msleep(20);
    }
    /* HPCopy one call per crop row (shows per-call overhead) */
    {
        uint32_t t = et_us(); int fails = 0;
        uint32_t s0 = hs + y0 * et_pitch + x0 * 14 / 8;
        for (uint32_t r = 0; r < ch; r++)
            fails += HPCopy(hd + r * rb, (void *)(s0 + r * et_pitch), rb) != 0;
        t = et_us() - t;
        uint32_t mibs10 = t ? (uint32_t)((uint64_t)rb * ch * 10000000ULL / t / (1024 * 1024)) : 0;
        et_printf("HP per-row: %d calls, %d failed, %d us (%d.%d MiB/s)\n", ch, fails, t, mibs10 / 10, mibs10 % 10);
    }
    et_flush();
}

/* ---- burst ---- */
struct et_mem { uint32_t fr_t[ET_MAXFR], fr_dma[ET_MAXFR], fr_tot[ET_MAXFR]; uint8_t fr_q[ET_MAXFR], fr_f[ET_MAXFR]; };
static struct et_mem *cm;
static volatile int q_head, q_tail, w_stop, w_done, w_err;
static uint32_t w_n, w_us, w_max;
static FILE *w_file;

static void et_writer(void *unused)
{
    while (!w_stop || q_tail != q_head)
    {
        if (q_tail == q_head) { msleep(5); continue; }
        uint32_t t = et_us();
        int r = FIO_WriteFile(w_file, ring[q_tail % nring], ring_bytes);
        t = et_us() - t;
        if (r != (int)ring_bytes) { w_err = r ? r : -1; break; }
        w_n++; w_us += t; if (t > w_max) w_max = t;
        q_tail++;
    }
    w_done = 1;
}

static void et_burst(uint32_t x0, uint32_t y0, uint32_t cw, uint32_t ch)
{
    uint32_t rb = cw * 14 / 8;
    uint32_t src = et_ubuf + y0 * et_pitch + x0 * 14 / 8;
    uint32_t r_top = y0, r_bot = y0 + ch - 1;
    et_printf("== burst: EDMAC, crop %dx%d at (%d,%d), %d bytes/frame, %d buffers, setup per frame, period %d us\n",
              cw, ch, x0, y0, ring_bytes, nring, et_period_us);
    et_flush();

    w_file = FIO_CreateFile("ML/EDMREC.BIN");
    if (!w_file) { et_printf("burst: cannot create ML/EDMREC.BIN\n"); return; }
    q_head = q_tail = w_stop = w_done = w_err = 0;
    w_n = w_us = w_max = 0;
    task_create("et_writer", 0x1d, 0x1000, et_writer, 0);

    int frames = 0, overruns = 0, missed = 0, tears = 0, nlog = 0;
    uint32_t dmin = 0xFFFFFFFF, dmax = 0, dsum = 0, last_t = 0;
    uint32_t dead_us = et_period_us ? et_period_us * 6 / 10 : 3000;
    uint32_t prevb = et_probe_stable(r_bot), top_ref = et_probe(r_top), last_same_t = 0;
    uint32_t t0 = et_us();
    while (et_edmac_ok)
    {
        uint32_t now = et_us() - t0;
        if (now > ET_BURST_US || w_err) break;
        uint32_t tv = et_probe(r_top);
        uint32_t v = et_probe(r_bot);
        if (v == prevb) { top_ref = tv; last_same_t = now; continue; }
        prevb = et_probe_stable(r_bot);
        if (last_t && now - last_t < dead_us) { top_ref = et_probe(r_top); last_same_t = now; continue; }
        if (last_t && et_period_us)
        {
            uint32_t k = (now - last_t + et_period_us / 2) / et_period_us;
            if (k > 1) missed += k - 1;
        }
        last_t = now;
        int late = now - last_same_t > 1000;
        if (q_head - q_tail >= nring) { overruns++; continue; }

        uint32_t top0 = et_probe(r_top), us, t = et_us();
        et_begin();
        int r = et_copy(src, et_pitch, (uint32_t)ring[q_head % nring], rb, ch, &us);
        if (r) { et_fail("burst"); break; }
        et_end();
        t = et_us() - t;
        uint32_t bot1 = et_probe(r_bot);
        int tear = top0 != top_ref || bot1 != prevb;
        int qd = q_head - q_tail;
        q_head++;

        frames++; tears += tear; dsum += us;
        if (us < dmin) dmin = us;
        if (us > dmax) dmax = us;
        if (nlog < ET_MAXFR)
        {
            cm->fr_t[nlog] = now; cm->fr_dma[nlog] = us; cm->fr_tot[nlog] = t;
            cm->fr_q[nlog] = qd; cm->fr_f[nlog] = tear | (late << 1); nlog++;
        }
        if (et_period_us > t + 15000) msleep(10);
        top_ref = et_probe(r_top);
        last_same_t = et_us() - t0;
        if (et_probe(r_bot) != prevb) last_same_t = 0;
    }
    uint32_t dur = et_us() - t0;

    w_stop = 1;
    for (int i = 0; i < 1000 && !w_done; i++) msleep(10);
    if (w_done) FIO_CloseFile(w_file);
    else et_printf("burst: writer still busy after 10 s, file left open, buffers kept\n");

    et_printf("BURST %d us: copied %d, missed %d, overruns %d, torn %d\n", dur, frames, missed, overruns, tears);
    if (frames) et_printf("BURST dma us min %d avg %d max %d\n", dmin, dsum / frames, dmax);
    if (w_n)
    {
        uint32_t mibs10 = (uint32_t)((uint64_t)ring_bytes * w_n * 10000000ULL / w_us / (1024 * 1024));
        et_printf("BURST write: %d frames, avg %d us, max %d us, %d.%d MiB/s, err %d\n",
                  w_n, w_us / w_n, w_max, mibs10 / 10, mibs10 % 10, w_err);
    }
    for (int i = 0; i < nlog; i++)
        et_printf("FR %7d dma %6d tot %6d q %d%s%s\n", cm->fr_t[i], cm->fr_dma[i], cm->fr_tot[i], cm->fr_q[i],
                  (cm->fr_f[i] & 1) ? " TEAR" : "", (cm->fr_f[i] & 2) ? " LATE" : "");
}

static void et_task(void *unused)
{
    et_logbuf = malloc(ET_LOGLEN);
    cm = malloc(sizeof(*cm));
    et_loglen = 0;
    w_file = 0; w_done = 0; nring = 0;
    if (!et_logbuf || !cm) { NotifyBox(6000, "DMA test: out of memory"); goto out; }
    if (!et_sem) et_sem = create_named_semaphore("et_dma", SEM_CREATE_LOCKED);

    et_printf("start v11 dma test run %d t=%d lv=%d\n", et_runs, get_ms_clock(), lv ? 1 : 0);
    et_ports("before");
    int st = et_lock();
    et_edmac_ok = !st;
    et_printf("resource lock {%08x %08x}: %d (%s)\n", et_res_ids[0], et_res_ids[1], st,
              st == 0 ? "locked" : st == 0x1F ? "BUSY, EDMAC skipped" : "error, EDMAC skipped");
    et_flush();

    et_synthetic();

    et_printf("call lv_save_raw 1\n"); et_flush();
    call("lv_save_raw", 1);
    msleep(500);
    et_w = MEM(ET_PATH + 0x10); et_h = MEM(ET_PATH + 0x14);
    uint32_t a = MEM(ET_PATH + 0x58);
    et_pitch = et_w * 14 / 8;
    et_ubuf = ET_BUF | ET_UNC;
    et_printf("M1 w=%d h=%d addr=%08x pitch=%d\n", et_w, et_h, a, et_pitch);
    if (!et_w || !et_h || et_pitch * et_h > ET_BUFLEN || (a & ~ET_UNC) != ET_BUF || et_h < 64)
    {
        et_printf("MEM1 not active or size unexpected, skipping live tests\n");
        goto off;
    }

    uint32_t x0, y0, cw, ch;
    et_crop(1920, 1080, &x0, &y0, &cw, &ch);
    ring_bytes = cw * ch * 14 / 8;
    for (int i = 0; i < ET_NBUF; i++)
    {
        ring[i] = et_alloc(ring_bytes, &ring_raw[i]);
        if (!ring[i]) break;
        ring[i] = UNCACHEABLE(ring[i]);
        nring++;
    }
    et_printf("ring: %d x %d bytes\n", nring, ring_bytes);
    if (!nring) goto off;

    et_period(y0 + ch - 1);
    et_live_timing(x0, y0, cw, ch);
    if (et_edmac_ok && et_single_ok && nring >= 2) et_burst(x0, y0, cw, ch);
    else et_printf("burst skipped (edmac %d, verified %d, buffers %d)\n", et_edmac_ok, et_single_ok, nring);
    et_flush();

off:
    et_printf("call lv_save_raw 0\n"); et_flush();
    call("lv_save_raw", 0);
    if (!w_file || w_done)
        for (int i = 0; i < nring; i++) free(ring_raw[i]);
    nring = 0;
    if (et_edmac_ok && et_single_ok) et_repeat();
    et_ports("at end");
    et_unlock();
    et_printf("done t=%d\n", get_ms_clock());
    et_flush();
    et_runs++;
    NotifyBox(6000, "LV DMA test: DONE");
out:
    if (et_logbuf) free(et_logbuf);
    if (cm) free(cm);
    et_logbuf = 0; cm = 0;
    info_led_blink(3, 100, 100);
    et_busy = 0;
}

static MENU_SELECT_FUNC(et_select)
{
    if (et_busy) return;
    et_busy = 1;
    task_create("et_task", 0x1e, 0x2000, et_task, 0);
}

static MENU_UPDATE_FUNC(et_update)
{
    MENU_SET_VALUE(et_busy ? "running..." : "%d runs saved", et_runs);
}

static struct menu_entry et_menu[] = {
    {
        .name   = "LV DMA copy test",
        .select = et_select,
        .update = et_update,
        .help   = "EDMAC + HPCopy: verify on test data, time 1080p crop copies, 3 s burst",
        .help2  = "NOT recording. SET, close menu, wait for DONE (~10 s). Freeze: pull battery.",
    },
};

static void et_init(void *unused)
{
    menu_add("Debug", et_menu, COUNT(et_menu));
}

INIT_FUNC("edmtest", et_init);
