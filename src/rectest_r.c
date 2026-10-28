/* riz/eosr-dmlog v13: frame-synchronous LV raw record test (EOS R 1.8.0).
 * Personal diagnostic build, not for upstream.
 *
 * v12 showed the card keeps up (110 MiB/s with ~15 MB writes) but a polling
 * capture loop at task priority 0x1e is starved in LV (msleep(1) = 2..46 ms):
 * at 29.97p 26 frames were never seen, 45 saved twice and 16 torn.
 * v13 syncs to the frame instead of polling:
 *  - IRQ sync: Mem1Path (lv_save_raw) programs EDMAC port 3 and registers an
 *    empty complete-CBR (0xE0638680: bx lr) with RegisterEDmacCompleteCBR
 *    (0xE0535A82). The EDMAC ISR (0xE0537990) calls table[port] = {cbr, ctx}
 *    at RAM 0x73DE8 on transfer complete. We swap port 3's cbr for a wrapper
 *    (records time + row hashes, wakes the capture task, then calls Canon's
 *    cbr) and put it back afterwards (only if Canon hasn't replaced it).
 *  - Timer sync (fallback / for comparison): HPTimer wakes the capture task
 *    3 ms before the next frame is due, then it polls briefly.
 *  - Capture task at priority 0x12; it blocks between frames and never spins
 *    for long. Duplicate guard: a frame whose bottom crop row equals the last
 *    saved frame's is skipped.
 * Before recording, a 1 s survey (HPTimer sampling the crop's top/bottom rows
 * every 1 ms, plus the IRQ log) measures frame period, readout and where the
 * IRQ falls; raw data -> ML/RTnn.SYN. Auto mode uses IRQ sync only if the
 * survey shows one IRQ per new frame.
 * Buffers/writer as v12: contiguous shoot_malloc chunks of ~16 MB, one
 * FIO_WriteFile per chunk -> ML/RTnn.BIN (.Bxx every ~2 GB); log
 * ML/RECTEST.TXT, per-frame ML/RTnn.IDX.
 * Every DMA wait has a timeout; on a timeout EDMAC is reset and released and
 * the run stops. No ROM writes, no property changes; the only write to Canon
 * RAM is the port-3 CBR pointer, restored at the end.
 */
#include "dryos.h"
#include "menu.h"
#include "propvalues.h"
#include "timer.h"
#include "mem.h"
#include "exmem.h"
#include "fio-ml.h"

#ifndef RT_HOST
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
#define RT_DMA_TIMER() GET_DIGIC_TIMER()
#define RT_SYNC_CACHES() sync_caches()
#define RT_DMB() asm volatile("dmb" ::: "memory")
#define RT_CALL_ORIG(f, ctx) ((void (*)(void *))(f))(ctx)
#else
#include "rt_host.h"
#endif

#define ERSC_MGR       0x10924
#define PWR_LIST_VCOPY ((const void *)0xE0F2673C)
#define RT_RD_PORT     0x2E
#define RT_WR_PORT     0x0D

#define RT_PATH    0x133CC              /* Mem1Path struct; +0x50 = initialised (port 3 CBR registered) */
#define RT_BUF     0x28EEA940
#define RT_BUFLEN  0xFB9D90
#define RT_UNC     0x40000000

#define EDMAC_CBR_TABLE 0x73DE8         /* {cbr, ctx} per port (RegisterEDmacCompleteCBR 0xE0535A82) */
#define M1_PORT         3
#define M1_CBR_CANON    0xE0638681      /* Mem1Path's complete CBR: bx lr */

#define RT_MAXCH      10
#define RT_CHUNK_TGT  (16 * 1024 * 1024)
#define RT_SPLIT      0x7F000000u       /* new part file before ~2.1 GB */
#define RT_MAXFR      3000
#define RT_MAXW       1024
#define RT_MAXTL      480               /* 250 ms timeline samples */
#define RT_MAXIRQ     96
#define RT_NSMP       1000              /* survey: 1 ms samples */
#define RT_LOGLEN     0x10000
#define RT_TIMEOUT    500               /* ms per DMA wait */
#define RT_CAP_PRIO   0x12

/* per-frame flags in the .IDX */
#define F_DROP   1                      /* no free slot: not recorded */
#define F_LATE   2                      /* timer mode: frame already there on the first poll after the wake */
#define F_TEAR   4                      /* top crop row changed before the copy started */
#define F_TOPCHG 8                      /* top crop row changed during the copy (informational) */
#define F_BOTCHG 16                     /* bottom crop row changed before the copy ended: torn */
#define F_DUP    32                     /* same bottom row as the last saved frame: skipped */

enum { SYNC_AUTO = 0, SYNC_IRQ, SYNC_TIMER };
static int rt_crop_sel = 0;             /* 1920x1080, 1920x960, 1920x800 */
static int rt_dur_sel = 1;              /* 10, 20, 40 s */
static int rt_sync_sel = SYNC_AUTO;
static const int rt_crop_h[] = { 1080, 960, 800 };
static const int rt_dur_s[] = { 10, 20, 40 };
static const char *rt_sync_name[] = { "auto", "IRQ", "timer" };

static volatile int rt_busy = 0;
static int rt_runs = 0;
static int rt_run_id;
static char rt_last[48] = "";

/* ---- log ---- */
static char *rt_logbuf;
static int rt_loglen;

static void rt_printf(const char *fmt, ...)
{
    if (!rt_logbuf || rt_loglen > RT_LOGLEN - 200) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(rt_logbuf + rt_loglen, RT_LOGLEN - rt_loglen, fmt, ap);
    va_end(ap);
    if (n > 0) rt_loglen += MIN(n, RT_LOGLEN - rt_loglen - 1);
}

static void rt_flush(void)
{
    if (!rt_logbuf || !rt_loglen) return;
    FILE *f = FIO_CreateFileOrAppend("ML/RECTEST.TXT");
    if (f)
    {
        FIO_WriteFile(f, rt_logbuf, rt_loglen);
        FIO_CloseFile(f);
    }
    rt_loglen = 0;
}

static inline uint32_t rt_us(void) { return (uint32_t)get_us_clock(); }
static inline uint32_t d20(uint32_t a, uint32_t b) { return (a - b) & 0xFFFFF; }   /* DIGIC timer difference */

static void sort_u32(uint32_t *v, int n)
{
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { uint32_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
}

static uint32_t median_u32(uint32_t *v, int n)
{
    if (n <= 0) return 0;
    sort_u32(v, n);
    return n & 1 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

/* ---- EDMAC m2m (sequence as Canon's EFsCropVcopy; verified on camera in v11/v12) ---- */
static uint32_t rt_res_ids[2] = { 0x000100AD, 0x000100BB };
static void *rt_entry;
static int rt_locked, rt_begun;
static struct semaphore *rt_dsem;
static volatile uint32_t rt_cbr_t, rt_cbr_n;
struct rt_info { uint32_t w[15]; };     /* off1s off1a off1b off2s off2a off2b off3 xs xa xb ys ya yb xn yn */
static struct rt_info rt_ri, rt_wi;

static void rt_cbr(void *ctx)
{
    rt_cbr_t = RT_DMA_TIMER();
    rt_cbr_n++;
    give_semaphore(rt_dsem);
}

static int rt_lock(void)
{
    rt_entry = ErscCreateLockEntry(rt_res_ids, 2);
    if (!rt_entry) return -1;
    void *rl = (void *)MEM(ERSC_MGR + 0x0C);
    AcquireRecursiveLock(rl, 0);
    int st = ErscCheck(rt_entry);
    ReleaseRecursiveLock(rl);
    if (st) return st;
    st = ErscLock(rt_entry);
    rt_locked = !st;
    return st;
}

static void rt_unlock(void)
{
    if (rt_locked) { ErscUnlock(rt_entry); rt_locked = 0; }
    if (rt_entry) { ErscDelete(rt_entry); rt_entry = 0; }
}

static void rt_begin(void)
{
    PwrSubChipOn(PWR_LIST_VCOPY);
    uint32_t ports[2] = { RT_RD_PORT, RT_WR_PORT };
    M2mSetup(ports);
    M2mSetCbr(rt_cbr, 0);
    rt_begun = 1;
}

static void rt_end(void)
{
    if (!rt_begun) return;
    M2mResetCbr();
    EdmacUnregCbr(RT_RD_PORT);
    EdmacUnregCbr(RT_WR_PORT);
    EdmacBoomerOff(RT_WR_PORT);
    PwrSubChipOff(PWR_LIST_VCOPY);
    rt_begun = 0;
}

static void rt_fail(void)
{
    EdmacReset(RT_RD_PORT);
    EdmacReset(RT_WR_PORT);
    rt_end();
}

static int rt_copy(uint32_t src, uint32_t src_pitch, uint32_t dst, uint32_t row_bytes, uint32_t rows, uint32_t *dma_us)
{
    memset(&rt_ri, 0, sizeof(rt_ri));
    memset(&rt_wi, 0, sizeof(rt_wi));
    rt_ri.w[2] = src_pitch - row_bytes; rt_ri.w[9] = row_bytes; rt_ri.w[12] = rows - 1;
    rt_wi.w[9] = row_bytes; rt_wi.w[12] = rows - 1;
    uint32_t a[2] = { src, dst };
    uint32_t p[3] = { (uint32_t)&rt_ri, (uint32_t)&rt_wi, 0 };
    M2mSetAddr(a);
    M2mSetInfo(p);
    uint32_t n0 = rt_cbr_n;
    uint32_t t0 = RT_DMA_TIMER();
    M2mStart();
    int r = take_semaphore(rt_dsem, RT_TIMEOUT);
    if (r || rt_cbr_n == n0) { *dma_us = 0; return r ? r : -1; }
    *dma_us = d20(rt_cbr_t, t0);
    return 0;
}

/* ---- buffers: contiguous shoot_malloc chunks ---- */
struct rt_chunk { struct memSuite *suite; uint32_t base; volatile int state; volatile int n; };
enum { CH_FREE = 0, CH_FILL, CH_FULL };
static struct rt_chunk ch[RT_MAXCH];
static int nch, kslots;
static uint32_t slot_bytes, frame_bytes;

static void rt_free_chunks(void)
{
    for (int i = 0; i < nch; i++) if (ch[i].suite) shoot_free_suite(ch[i].suite);
    memset(ch, 0, sizeof(ch));
    nch = 0;
}

static void rt_alloc_chunks(void)
{
    uint32_t want = kslots * slot_bytes + 4096;
    for (int i = 0; i < RT_MAXCH; i++)
    {
        struct memSuite *s = shoot_malloc_suite_contig(want);
        if (!s) { rt_printf("chunk %d: shoot_malloc_suite_contig(%d) failed\n", i, want); break; }
        struct memChunk *c = GetFirstChunkFromSuite(s);
        uint32_t a = (uint32_t)GetMemoryAddressOfMemoryChunk(c);
        uint32_t sz = GetSizeOfMemoryChunk(c);
        if (sz < want) { rt_printf("chunk %d smaller than requested (%d), dropping it\n", i, sz); shoot_free_suite(s); break; }
        ch[i].suite = s;
        ch[i].base = ((uint32_t)UNCACHEABLE(a) + 4095) & ~4095;
        ch[i].state = CH_FREE; ch[i].n = 0;
        nch++;
    }
    RT_SYNC_CACHES();                   /* no dirty cache lines may later land on DMA-written data */
}

/* ---- MEM1 row probes ---- */
static uint32_t m1_w, m1_h, pitch, ubuf, r_top, r_bot;

static uint32_t rt_probe(uint32_t row)
{
    uint32_t h = 0x811C9DC5;
    for (int s = 1; s <= 3; s++)
    {
        const volatile uint32_t *p = (const volatile uint32_t *)(ubuf + row * pitch + ((pitch * s / 4) & ~3));
        for (int i = 0; i < 4; i++) h = (h ^ p[i]) * 0x01000193;
    }
    return h;
}

/* value once the row has been unchanged for 50 us. Re-reads before every time
 * check: v12 could return a stale mid-write value after being preempted. */
static uint32_t rt_probe_stable(uint32_t row)
{
    uint32_t a = rt_probe(row);
    uint32_t t0 = rt_us(), t = t0;
    for (;;)
    {
        uint32_t b = rt_probe(row);
        uint32_t now = rt_us();
        if (b != a) { a = b; t = now; continue; }
        if (now - t >= 50 || now - t0 > 2000) break;
    }
    return a;
}

/* ---- frame-end IRQ hook ---- */
static struct semaphore *rt_fsem;       /* wakes the capture task (IRQ, HPTimer) */
static uint32_t hook_orig;
static int hook_on;
static volatile uint32_t irq_seq, irq_n, irq_t, irq_top, irq_bot;
static volatile int irq_armed, irq_log_on, irq_log_n;
struct rt_ev { uint32_t t, top, bot; };
static struct rt_ev *irq_log;           /* allocated once, never freed (the ISR may use it) */

static void rt_m1_isr(void *ctx)
{
    uint32_t t = RT_DMA_TIMER();
    uint32_t top = rt_probe(r_top), bot = rt_probe(r_bot);
    irq_seq++;
    RT_DMB();
    irq_t = t; irq_top = top; irq_bot = bot; irq_n++;
    RT_DMB();
    irq_seq++;
    if (irq_log_on && irq_log_n < RT_MAXIRQ)
    {
        irq_log[irq_log_n].t = t; irq_log[irq_log_n].top = top; irq_log[irq_log_n].bot = bot;
        irq_log_n++;
    }
    if (irq_armed) give_semaphore(rt_fsem);
    RT_CALL_ORIG(hook_orig, ctx);
}

static int rt_hook(void)
{
    uint32_t e = EDMAC_CBR_TABLE + M1_PORT * 8;
    uint32_t cbr = MEM(e), ctx = MEM(e + 4), init = MEM(RT_PATH + 0x50);
    rt_printf("port 3 complete CBR %08x ctx %08x, Mem1Path initialised %d\n", cbr, ctx, init);
    if (cbr != M1_CBR_CANON || init != 1) { rt_printf("not Canon's Mem1Path CBR: no IRQ hook\n"); return -1; }
    hook_orig = cbr;
    irq_n = 0;
    RT_DMB();
    MEM(e) = (uint32_t)&rt_m1_isr;
    hook_on = 1;
    return 0;
}

static void rt_unhook(void)
{
    if (!hook_on) return;
    uint32_t e = EDMAC_CBR_TABLE + M1_PORT * 8;
    irq_armed = 0; irq_log_on = 0;
    if (MEM(e) == (uint32_t)&rt_m1_isr) { MEM(e) = hook_orig; rt_printf("IRQ hook removed, %d IRQs seen\n", irq_n); }
    else rt_printf("IRQ hook was already replaced by Canon (%08x), left alone\n", MEM(e));
    hook_on = 0;
}

static int rt_irq_snapshot(uint32_t *n, uint32_t *t, uint32_t *top, uint32_t *bot)
{
    for (int i = 0; i < 100; i++)
    {
        uint32_t s1 = irq_seq;
        RT_DMB();
        *n = irq_n; *t = irq_t; *top = irq_top; *bot = irq_bot;
        RT_DMB();
        if (s1 == irq_seq && !(s1 & 1)) return 0;
    }
    return -1;
}

/* ---- HPTimer: self-check, survey sampler, wake-up ---- */
static volatile uint32_t hpt_fired;
static int hpt_ok;
static struct rt_ev *smp;
static volatile int smp_n, smp_over;

static void rt_hpt_chk(int last_expiry, void *p) { hpt_fired = RT_DMA_TIMER(); give_semaphore(rt_fsem); }
static void rt_hpt_wake(int last_expiry, void *p) { give_semaphore(rt_fsem); }

static void rt_smp_over(int last_expiry, void *p);
static void rt_smp_cbr(int last_expiry, void *p)
{
    if (smp_n < RT_NSMP)
    {
        smp[smp_n].t = RT_DMA_TIMER(); smp[smp_n].top = rt_probe(r_top); smp[smp_n].bot = rt_probe(r_bot);
        smp_n++;
    }
    if (smp_n < RT_NSMP) SetHPTimerNextTick(last_expiry, 1000, rt_smp_cbr, rt_smp_over, 0);
    else give_semaphore(rt_fsem);
}

static void rt_smp_over(int last_expiry, void *p)
{
    smp_over++;
    if (smp_n < RT_NSMP && smp_over < 200) SetHPTimerAfterNow(1000, rt_smp_cbr, rt_smp_over, 0);
    else give_semaphore(rt_fsem);
}

static void rt_drain(struct semaphore *s)
{
    for (int i = 0; i < 1000 && !take_semaphore(s, 1); i++) ;
}

/* ---- survey: 1 s of 1 ms samples + IRQ log ---- */
static uint32_t sv_est, sv_irq_med, sv_readout, sv_irq_after_bot, sv_irq_to_top;
static int sv_irq_n, sv_irq_new, sv_bot_events, sv_irq_chk, sv_irq_settled;

static void rt_survey(void)
{
    sv_est = sv_irq_med = sv_readout = sv_irq_after_bot = sv_irq_to_top = 0;
    sv_irq_n = sv_irq_new = sv_bot_events = sv_irq_chk = sv_irq_settled = 0;
    smp_n = 0; smp_over = 0; irq_log_n = 0;
    irq_log_on = hook_on;
    rt_drain(rt_fsem);
    if (hpt_ok)
    {
        int r = SetHPTimerAfterNow(1000, rt_smp_cbr, rt_smp_over, 0);
        if (r & 1) { rt_printf("survey: SetHPTimerAfterNow returned %d\n", r); msleep(1000); }
        else take_semaphore(rt_fsem, 3000);
    }
    else msleep(1000);
    irq_log_on = 0;
    int ns = smp_n, ni = irq_log_n;
    rt_printf("survey: %d samples (%d overruns), %d IRQs\n", ns, smp_over, ni);

    /* change events in the samples (new value vs the previous sample; >3 ms apart = new event) */
    uint32_t *bt = malloc(3 * 200 * sizeof(uint32_t));
    if (!bt) { rt_printf("survey: no memory for analysis\n"); return; }
    uint32_t *tt = bt + 200, *tmp = bt + 400;
    int nb = 0, nt = 0, k;
    for (int i = 1; i < ns; i++)
    {
        if (smp[i].bot != smp[i - 1].bot && nb < 200 && (!nb || d20(smp[i].t, bt[nb - 1]) > 3000)) bt[nb++] = smp[i].t;
        if (smp[i].top != smp[i - 1].top && nt < 200 && (!nt || d20(smp[i].t, tt[nt - 1]) > 3000)) tt[nt++] = smp[i].t;
    }
    sv_bot_events = nb;
    k = 0;
    for (int i = 1; i < nb; i++) tmp[k++] = d20(bt[i], bt[i - 1]);
    sv_est = median_u32(tmp, k);
    k = 0;                              /* readout: top change -> following bottom change */
    for (int i = 0; i < nb; i++)
    {
        uint32_t best = 0xFFFFFFFF;
        for (int j = 0; j < nt; j++) { uint32_t d = d20(bt[i], tt[j]); if (d < best) best = d; }
        if (best < (sv_est ? sv_est : 200000)) tmp[k++] = best;
    }
    sv_readout = median_u32(tmp, k);

    sv_irq_n = ni;
    k = 0;
    for (int i = 1; i < ni; i++) { tmp[k++] = d20(irq_log[i].t, irq_log[i - 1].t); if (irq_log[i].bot != irq_log[i - 1].bot) sv_irq_new++; }
    sv_irq_med = median_u32(tmp, k);
    k = 0;                              /* IRQ: time since the last bottom change */
    for (int i = 0; i < ni; i++)
    {
        uint32_t best = 0xFFFFFFFF;
        for (int j = 0; j < nb; j++) { uint32_t d = d20(irq_log[i].t, bt[j]); if (d < best) best = d; }
        if (best < 500000) tmp[k++] = best;
    }
    sv_irq_after_bot = median_u32(tmp, k);
    k = 0;                              /* IRQ: time until the next top change */
    for (int i = 0; i < ni; i++)
    {
        uint32_t best = 0xFFFFFFFF;
        for (int j = 0; j < nt; j++) { uint32_t d = d20(tt[j], irq_log[i].t); if (d < best) best = d; }
        if (best < 500000) tmp[k++] = best;
    }
    sv_irq_to_top = median_u32(tmp, k);
    /* the IRQ must mark a frame that has just completed: the bottom crop row
     * changed within the last 8 ms, and the next frame hasn't reached the top
     * crop row yet (no top change within 1 ms after the IRQ) */
    if (nb)
        for (int i = 0; i < ni; i++)
        {
            uint32_t ab = 0xFFFFFFFF, nt_ = 0xFFFFFFFF;
            for (int j = 0; j < nb; j++) { uint32_t d = d20(irq_log[i].t, bt[j]); if (d < ab) ab = d; }
            for (int j = 0; j < nt; j++) { uint32_t d = d20(tt[j], irq_log[i].t); if (d < nt_) nt_ = d; }
            if (ab > 500000) continue;      /* no sample coverage around this IRQ */
            sv_irq_chk++;
            if (ab < 8000 && nt_ > 1000) sv_irq_settled++;
        }
    rt_printf("survey: %d bottom-row changes, period %d us; readout (top->bottom crop row) %d us\n", nb, sv_est, sv_readout);
    rt_printf("survey: IRQ period %d us, %d of %d IRQ intervals had a new bottom row; IRQ %d us after the bottom-row "
              "change; next top-row change %d us after the IRQ\n", sv_irq_med, sv_irq_new, ni > 1 ? ni - 1 : 0,
              sv_irq_after_bot, sv_irq_to_top);
    rt_printf("survey: IRQ right after a completed frame (bottom row changed < 8 ms before, top row not yet) "
              "in %d of %d checked IRQs\n", sv_irq_settled, sv_irq_chk);

    char n[32]; snprintf(n, sizeof(n), "ML/RT%02d.SYN", rt_run_id);
    FILE *f = FIO_CreateFile(n);
    if (f)
    {
        uint32_t hdr[4] = { 0x4E595352, ns, ni, sizeof(struct rt_ev) };   /* "RSYN", samples, IRQs, record size */
        FIO_WriteFile(f, hdr, sizeof(hdr));
        if (ns) FIO_WriteFile(f, smp, ns * sizeof(struct rt_ev));
        if (ni) FIO_WriteFile(f, irq_log, ni * sizeof(struct rt_ev));
        FIO_CloseFile(f);
    }
    free(bt);
}

/* ---- writer ---- */
static struct semaphore *rt_wsem;
static volatile int w_stop, w_done, w_err;
static volatile uint64_t w_bytes;      /* > 4 GB in a 40 s run */
static int w_part, w_n;
static FILE *w_file;
static char w_name[32];
static uint32_t w_fsize;
struct rt_wrec { uint32_t t_ms, us, bytes; };
static struct rt_wrec *wl;
static uint32_t rt_t0;

static void rt_part_name(char *out, int run, int part)
{
    if (part) snprintf(out, 32, "ML/RT%02d.B%02d", run, part);
    else snprintf(out, 32, "ML/RT%02d.BIN", run);
}

static void rt_writer(void *unused)
{
    int wi = 0;
    for (;;)
    {
        if (ch[wi].state == CH_FULL)
        {
            uint32_t bytes = ch[wi].n * slot_bytes;
            if (w_fsize + bytes > RT_SPLIT)
            {
                FIO_CloseFile(w_file);
                rt_part_name(w_name, rt_run_id, ++w_part);
                w_file = FIO_CreateFile(w_name);
                w_fsize = 0;
                if (!w_file) { w_err = -2; break; }
            }
            uint32_t t = rt_us();
            int r = FIO_WriteFile(w_file, (void *)ch[wi].base, bytes);
            t = rt_us() - t;
            if (w_n < RT_MAXW) { wl[w_n].t_ms = (rt_us() - rt_t0) / 1000; wl[w_n].us = t; wl[w_n].bytes = bytes; }
            w_n++;
            if (r != (int)bytes) { w_err = r ? r : -1; break; }
            w_fsize += bytes;
            w_bytes += bytes;
            ch[wi].n = 0;
            ch[wi].state = CH_FREE;
            wi = (wi + 1) % nch;
            continue;
        }
        if (w_stop) break;
        take_semaphore(rt_wsem, 50);
    }
    w_done = 1;
}

/* ---- capture ---- */
struct rt_frame { uint32_t t_us; uint16_t dma_us; uint8_t lat; uint8_t flags; };   /* lat: 0.1 ms, 255 = >= 25.5 ms */
struct rt_tl { uint16_t t_ms4; uint16_t frames; uint16_t drops; uint16_t used; uint32_t mb_written; };
static struct rt_frame *fl;
static struct rt_tl *tl;

static int c_mode, c_dur_s;
static uint32_t c_src, c_rb, c_ch, c_est;
static int nfr, frames, drops, dups, missed, late, tears, topchg, botchg, ntl, irq_timeouts;
static int cur, next_fill, first_drop_ms, c_abort;
static uint32_t first_t, next_tl, max_used, dmax, dsum, last_rec_bot, t_last_ev;
static volatile int c_done;
static struct semaphore *rt_csem;

static void rt_timeline(uint32_t now)
{
    if (!first_t || now - first_t < next_tl || ntl >= RT_MAXTL) return;
    int used = 0;
    for (int i = 0; i < nch; i++) if (ch[i].state != CH_FREE) used += ch[i].n;
    tl[ntl].t_ms4 = (now - first_t) / 4000; tl[ntl].frames = frames; tl[ntl].drops = drops;
    tl[ntl].used = used; tl[ntl].mb_written = (uint32_t)(w_bytes >> 20); ntl++;
    next_tl += 250000;
}

/* one new frame: dedupe, take a slot, copy, check. td = frame time (rt_us);
 * lat_us = frame event -> now; top_ref/bot_ref = crop rows at the frame event */
static int rt_frame(uint32_t td, uint32_t lat_us, int flags, uint32_t top_ref, uint32_t bot_ref)
{
    if (!first_t) { first_t = td; next_tl = 0; }
    uint32_t us = 0;
    if (frames && bot_ref == last_rec_bot)
    {
        flags |= F_DUP; dups++;
    }
    else
    {
        uint32_t dst = 0;
        if (cur < 0 && ch[next_fill].state == CH_FREE) { cur = next_fill; ch[cur].n = 0; ch[cur].state = CH_FILL; }
        if (cur >= 0) dst = ch[cur].base + ch[cur].n * slot_bytes;
        if (!dst)
        {
            flags |= F_DROP; drops++;
            if (first_drop_ms < 0) first_drop_ms = (td - first_t) / 1000;
        }
        else
        {
            uint32_t top0 = rt_probe(r_top);
            if (top0 != top_ref) { flags |= F_TEAR; tears++; }
            if (rt_copy(c_src, pitch, dst, c_rb, c_ch, &us))
            {
                rt_printf("!! EDMAC timeout at frame %d, stopping\n", nfr);
                rt_fail();
                return -1;
            }
            if (rt_probe(r_top) != top0) { flags |= F_TOPCHG; topchg++; }
            if (rt_probe(r_bot) != bot_ref) { flags |= F_BOTCHG; botchg++; }
            frames++; dsum += us; if (us > dmax) dmax = us;
            last_rec_bot = bot_ref;
            if (++ch[cur].n == kslots)
            {
                ch[cur].state = CH_FULL;
                give_semaphore(rt_wsem);
                next_fill = (cur + 1) % nch;
                cur = -1;
            }
        }
    }
    if (flags & F_LATE) late++;
    if (nfr < RT_MAXFR)
    {
        fl[nfr].t_us = td - first_t; fl[nfr].dma_us = MIN(us, 65535);
        fl[nfr].lat = MIN(lat_us / 100, 255); fl[nfr].flags = flags;
    }
    nfr++;
    int used = 0;
    for (int i = 0; i < nch; i++) if (ch[i].state != CH_FREE) used += ch[i].n;
    if ((uint32_t)used > max_used) max_used = used;
    return 0;
}

static void rt_capture_irq(void)
{
    uint32_t n_last, n, t, top, bot;
    rt_drain(rt_fsem);
    rt_irq_snapshot(&n_last, &t, &top, &bot);
    irq_armed = 1;
    int first = 1, to = 0;
    for (;;)
    {
        uint32_t now = rt_us();
        if (first_t && now - first_t > (uint32_t)c_dur_s * 1000000) break;
        if (w_err) { rt_printf("writer error %d, stopping\n", w_err); break; }
        rt_timeline(now);
        if (take_semaphore(rt_fsem, 200))
        {
            irq_timeouts++;
            if (++to >= 5) { rt_printf("no frame IRQ for 1 s, stopping\n"); break; }
            continue;
        }
        to = 0;
        if (rt_irq_snapshot(&n, &t, &top, &bot)) continue;
        if (n == n_last) continue;                  /* extra semaphore count for an IRQ already handled */
        if (!first) missed += n - n_last - 1;
        first = 0;
        n_last = n;
        uint32_t lat = d20(RT_DMA_TIMER(), t);
        uint32_t td = rt_us() - lat;
        t_last_ev = td;
        if (rt_frame(td, lat, 0, top, bot)) { c_abort = 1; break; }
    }
    irq_armed = 0;
}

static void rt_capture_timer(uint32_t est)
{
    uint32_t prevb = rt_probe_stable(r_bot), top_same = rt_probe(r_top), t_prev = 0, t0 = rt_us();
    int after_wake = 0;
    for (;;)
    {
        uint32_t now = rt_us();
        if (first_t && now - first_t > (uint32_t)c_dur_s * 1000000) break;
        if (w_err) { rt_printf("writer error %d, stopping\n", w_err); break; }
        if (now - (t_prev ? t_prev : t0) > 1000000) { rt_printf("no new frame for 1 s, stopping\n"); break; }
        rt_timeline(now);

        /* block until 3 ms before the next frame is due */
        if (t_prev && !after_wake)
        {
            int32_t rem = (int32_t)(t_prev + est - 3000 - now);
            if (rem > 200)
            {
                rt_drain(rt_fsem);
                if (SetHPTimerAfterNow(rem, rt_hpt_wake, rt_hpt_wake, 0) & 1) msleep(MAX(rem / 1000, 1));
                else take_semaphore(rt_fsem, 200);
                after_wake = 1;
                continue;
            }
        }
        /* poll, at most 8 ms at a time, so this high-priority task never spins for long */
        uint32_t spin0 = rt_us(), t_same = spin0;
        int changed = 0, first_poll = 1;
        while (rt_us() - spin0 < 8000)
        {
            uint32_t vb = rt_probe(r_bot), vt = rt_probe(r_top);
            if (vb != prevb) { changed = 1; break; }
            top_same = vt; first_poll = 0; t_same = rt_us();
        }
        if (!changed)
        {
            after_wake = 0;             /* frame not there yet: let others run for ~1 ms, look again */
            rt_drain(rt_fsem);
            if (SetHPTimerAfterNow(1000, rt_hpt_wake, rt_hpt_wake, 0) & 1) msleep(1);
            else take_semaphore(rt_fsem, 20);
            continue;
        }
        prevb = rt_probe_stable(r_bot);
        uint32_t td = rt_us();
        int flags = (after_wake && first_poll) ? F_LATE : 0;
        if (t_prev)
        {
            uint32_t d = td - t_prev, k = (d + est / 2) / est;
            if (k >= 1)
            {
                missed += k - 1;
                int32_t err = (int32_t)(d / k) - (int32_t)est;
                if (err > -(int32_t)est / 10 && err < (int32_t)est / 10) est += err / 8;
            }
        }
        t_prev = td;
        t_last_ev = td;
        after_wake = 0;
        /* latency: since the last poll that still saw the old frame (unknown, 25.5 ms, when late) */
        uint32_t lat = (flags & F_LATE) ? 25500 : td - t_same;
        if (rt_frame(td, lat, flags, (flags & F_LATE) ? rt_probe(r_top) : top_same, prevb)) { c_abort = 1; break; }
        top_same = rt_probe(r_top);
    }
    c_est = est;
}

static void rt_capture_task(void *unused)
{
    if (c_mode == SYNC_IRQ) rt_capture_irq();
    else rt_capture_timer(c_est);
    c_done = 1;
    give_semaphore(rt_csem);
}

static void rt_export(int run);
static void rt_copy_file(const char *src, const char *dst);

static void rt_task(void *unused)
{
    rt_logbuf = malloc(RT_LOGLEN);
    fl = malloc(sizeof(*fl) * RT_MAXFR);
    wl = malloc(sizeof(*wl) * RT_MAXW);
    tl = malloc(sizeof(*tl) * RT_MAXTL);
    smp = malloc(sizeof(*smp) * RT_NSMP);
    if (!irq_log) irq_log = malloc(sizeof(*irq_log) * RT_MAXIRQ);
    rt_loglen = 0;
    w_file = 0; w_done = 1; nch = 0; hook_on = 0;
    int lv_raw = 0, mode = -1;
    if (!rt_logbuf || !fl || !wl || !tl || !irq_log || !smp) { NotifyBox(6000, "Record test: out of memory"); goto out; }
    if (!rt_dsem) rt_dsem = create_named_semaphore("rt_dma", SEM_CREATE_LOCKED);
    if (!rt_wsem) rt_wsem = create_named_semaphore("rt_wr", SEM_CREATE_LOCKED);
    if (!rt_fsem) rt_fsem = create_named_semaphore("rt_frm", SEM_CREATE_LOCKED);
    if (!rt_csem) rt_csem = create_named_semaphore("rt_cap", SEM_CREATE_LOCKED);

    uint32_t crop_w = 1920, crop_h = rt_crop_h[rt_crop_sel];
    int dur_s = rt_dur_s[rt_dur_sel];
    for (rt_run_id = 0; rt_run_id < 99; rt_run_id++)
    {
        char n[32]; rt_part_name(n, rt_run_id, 0);
        if (!is_file(n)) break;
    }
    rt_printf("start v13 record test run %d: crop %dx%d, %d s, sync %s, t=%d lv=%d, card free %d MB\n", rt_run_id,
              crop_w, crop_h, dur_s, rt_sync_name[rt_sync_sel], get_ms_clock(), lv ? 1 : 0,
              get_free_space_32k(get_shooting_card()) / 32);

    /* HPTimer self-check: one 2 ms timer */
    {
        rt_drain(rt_fsem);
        hpt_fired = 0;
        uint32_t t0 = RT_DMA_TIMER();
        int r = SetHPTimerAfterNow(2000, rt_hpt_chk, rt_hpt_chk, 0);
        int w = (r & 1) ? -1 : take_semaphore(rt_fsem, 100);
        uint32_t d = hpt_fired ? d20(hpt_fired, t0) : 0;
        hpt_ok = !(r & 1) && !w && d >= 1500 && d < 20000;
        rt_printf("HPTimer check: returned %d, %s after %d us -> %s\n", r, w ? "did NOT fire" : "fired", d,
                  hpt_ok ? "ok" : "unusable");
    }

    int st = rt_lock();
    rt_printf("resource lock: %d (%s)\n", st, st == 0 ? "locked" : st == 0x1F ? "BUSY" : "error");
    if (st) { NotifyBox(6000, "Record test: DMA busy (%d)", st); goto done; }

    call("lv_save_raw", 1);
    lv_raw = 1;
    msleep(500);
    m1_w = MEM(RT_PATH + 0x10); m1_h = MEM(RT_PATH + 0x14);
    uint32_t a = MEM(RT_PATH + 0x58);
    pitch = m1_w * 14 / 8;
    ubuf = RT_BUF | RT_UNC;
    rt_printf("M1 w=%d h=%d addr=%08x pitch=%d\n", m1_w, m1_h, a, pitch);
    if (!m1_w || !m1_h || pitch * m1_h > RT_BUFLEN || (a & ~RT_UNC) != RT_BUF || m1_h < 64)
    { rt_printf("MEM1 not active or size unexpected, abort\n"); NotifyBox(6000, "Record test: no LV raw"); goto done; }

    crop_w = MIN(crop_w, m1_w) & ~63;
    crop_h = MIN(crop_h, m1_h) & ~1;
    uint32_t x0 = ((m1_w - crop_w) / 2) & ~63, y0 = ((m1_h - crop_h) / 2) & ~1;
    c_rb = crop_w * 14 / 8;
    c_ch = crop_h;
    c_src = ubuf + y0 * pitch + x0 * 14 / 8;
    r_top = y0; r_bot = y0 + crop_h - 1;
    frame_bytes = c_rb * crop_h;
    slot_bytes = (frame_bytes + 4095) & ~4095;
    kslots = MAX(1, RT_CHUNK_TGT / slot_bytes);
    rt_printf("crop %dx%d at (%d,%d), %d bytes/frame, slot %d, %d slots/chunk (%d bytes per write)\n",
              crop_w, crop_h, x0, y0, frame_bytes, slot_bytes, kslots, kslots * slot_bytes);

    /* frame-end IRQ hook + survey */
    rt_hook();
    rt_survey();
    rt_flush();
    /* IRQ sync needs: one IRQ per frame, a new bottom row each time, and (if the
     * HPTimer samples exist) the frame already complete when the IRQ comes */
    int irq_ok = hook_on && sv_irq_n >= 10 && sv_irq_new >= (sv_irq_n - 1) * 9 / 10 &&
                 (!sv_est || (sv_irq_med > sv_est * 95 / 100 && sv_irq_med < sv_est * 105 / 100)) &&
                 (!sv_irq_chk || sv_irq_settled >= sv_irq_chk * 9 / 10);
    if (irq_ok && !sv_irq_chk) rt_printf("sync: IRQ position not verified (no HPTimer samples)\n");
    int timer_ok = hpt_ok && sv_est >= 8000 && sv_est <= 200000;
    mode = rt_sync_sel;
    if (mode == SYNC_AUTO) mode = irq_ok ? SYNC_IRQ : timer_ok ? SYNC_TIMER : -1;
    else if ((mode == SYNC_IRQ && !irq_ok) || (mode == SYNC_TIMER && !timer_ok)) mode = -1;
    rt_printf("sync: IRQ %s, timer %s -> using %s\n", irq_ok ? "usable" : "NOT usable", timer_ok ? "usable" : "NOT usable",
              mode < 0 ? "nothing (abort)" : rt_sync_name[mode]);
    if (mode < 0) { snprintf(rt_last, sizeof(rt_last), "RT%02d: no frame sync", rt_run_id); goto done; }
    if (mode == SYNC_TIMER) rt_unhook();

    rt_alloc_chunks();
    rt_printf("buffer: %d chunks = %d frames (%d MB)\n", nch, nch * kslots, nch * kslots * slot_bytes >> 20);
    rt_flush();
    if (nch < 2) { snprintf(rt_last, sizeof(rt_last), "not enough memory (%d)", nch); goto done; }

    rt_begin();
    {
        uint32_t us;
        if (rt_copy(c_src, pitch, ch[0].base, c_rb, c_ch, &us))
        { rt_printf("!! first EDMAC copy timed out\n"); rt_fail(); snprintf(rt_last, sizeof(rt_last), "DMA timeout"); goto done; }
        rt_printf("first copy: dma %d us\n", us);
    }

    rt_part_name(w_name, rt_run_id, 0);
    w_file = FIO_CreateFile(w_name);
    if (!w_file) { rt_printf("cannot create %s\n", w_name); goto done; }
    w_part = 0; w_fsize = 0; w_bytes = 0; w_n = 0; w_stop = 0; w_err = 0; w_done = 0;
    rt_t0 = rt_us();
    task_create("rt_writer", 0x1c, 0x1000, rt_writer, 0);
    rt_flush();

    /* capture in a high-priority task that blocks between frames */
    nfr = frames = drops = dups = missed = late = tears = topchg = botchg = ntl = irq_timeouts = 0;
    cur = -1; next_fill = 0; first_drop_ms = -1; c_abort = 0;
    first_t = next_tl = max_used = dmax = dsum = last_rec_bot = t_last_ev = 0;
    c_mode = mode; c_dur_s = dur_s; c_est = sv_est; c_done = 0;
    uint32_t irq0 = irq_n;
    rt_drain(rt_csem);
    task_create("rt_capture", RT_CAP_PRIO, 0x1000, rt_capture_task, 0);
    take_semaphore(rt_csem, (dur_s + 10) * 1000);
    if (!c_done) { rt_printf("capture task did not finish in time\n"); for (int i = 0; i < 500 && !c_done; i++) msleep(10); }
    uint32_t irq_rec = irq_n - irq0;
    uint32_t dur_us = t_last_ev - first_t;

    /* hand over the partial chunk, stop the writer */
    if (cur >= 0) { if (ch[cur].n) ch[cur].state = CH_FULL; else ch[cur].state = CH_FREE; }
    uint32_t ts = rt_us();
    w_stop = 1;
    give_semaphore(rt_wsem);
    for (int i = 0; i < 3000 && !w_done; i++) msleep(10);
    uint32_t drain_ms = (rt_us() - ts) / 1000;
    if (w_done) { FIO_CloseFile(w_file); w_file = 0; }
    else rt_printf("writer still busy after 30 s; file left open, buffers kept\n");
    if (!c_abort) rt_end();
    rt_unhook();

    /* summary */
    uint32_t wus = 0, wmax = 0, wmin = 0xFFFFFFFF;
    for (int i = 0; i < MIN(w_n, RT_MAXW); i++) { wus += wl[i].us; wmax = MAX(wmax, wl[i].us); wmin = MIN(wmin, wl[i].us); }
    uint64_t wb = w_bytes;
    uint32_t est = mode == SYNC_IRQ ? sv_irq_med : c_est;
    uint32_t need10 = est ? (uint32_t)((uint64_t)frame_bytes * 10000000ULL / est / (1024 * 1024)) : 0;
    uint32_t card10 = wus ? (uint32_t)(wb * 10000000ULL / wus / (1024 * 1024)) : 0;
    uint32_t wall10 = dur_us ? (uint32_t)(wb * 10000000ULL / ((uint64_t)dur_us + drain_ms * 1000) / (1024 * 1024)) : 0;
    /* start-latency percentiles from a histogram of the 8-bit .IDX values (no 12 KB sort buffer in bss) */
    uint16_t hist[256];
    memset(hist, 0, sizeof(hist));
    int nl = 0;
    for (int i = 0; i < MIN(nfr, RT_MAXFR); i++) if (!(fl[i].flags & (F_DROP | F_DUP))) { hist[fl[i].lat]++; nl++; }
    uint32_t l50 = 0, l90 = 0, lmax = 0;
    for (int v = 0, c = 0; v < 256; v++)
    {
        if (!hist[v]) continue;
        if (c <= nl / 2 && c + hist[v] > nl / 2) l50 = v;
        if (c <= nl * 9 / 10 && c + hist[v] > nl * 9 / 10) l90 = v;
        c += hist[v]; lmax = v;
    }
    rt_printf("RESULT run %d: sync %s, %dx%d, %d.%03d s, %d frame events, %d recorded, %d dropped (first drop at %d ms), "
              "%d missed, %d duplicates skipped, %d written to card\n", rt_run_id, rt_sync_name[mode], crop_w, crop_h,
              dur_us / 1000000, dur_us / 1000 % 1000, nfr, frames, drops, first_drop_ms, missed, dups,
              (uint32_t)(wb / slot_bytes));
    rt_printf("RESULT data rate needed %d.%d MiB/s; card %d.%d MiB/s while writing, %d.%d MiB/s overall; "
              "%d writes, min %d max %d us; buffer peak %d of %d frames; drain %d ms\n",
              need10 / 10, need10 % 10, card10 / 10, card10 % 10, wall10 / 10, wall10 % 10,
              w_n, wmin, wmax, max_used, nch * kslots, drain_ms);
    rt_printf("RESULT copy dma avg %d max %d us; start latency p50 %d.%d p90 %d.%d max %d.%d ms; late %d, tear %d, "
              "topchg %d, botchg(torn) %d; IRQs during capture %d, IRQ timeouts %d; writer err %d\n",
              frames ? dsum / frames : 0, dmax, l50 / 10, l50 % 10, l90 / 10, l90 % 10, lmax / 10, lmax % 10,
              late, tears, topchg, botchg, irq_rec, irq_timeouts, w_err);
    rt_printf("files: %s (last part %d), %d MB total\n", w_name, w_part, (uint32_t)(wb >> 20));
    snprintf(rt_last, sizeof(rt_last), "RT%02d %s: %d rec, %d miss", rt_run_id, rt_sync_name[mode], frames, missed + drops);
    rt_flush();
    for (int i = 0; i < ntl; i++)
        rt_printf("TL %6d ms frames %5d drops %5d buffered %3d written %5d MB\n", tl[i].t_ms4 * 4, tl[i].frames,
                  tl[i].drops, tl[i].used, tl[i].mb_written);
    rt_flush();
    for (int i = 0; i < MIN(w_n, RT_MAXW); i++)
        rt_printf("W %6d ms %7d us %9d bytes\n", wl[i].t_ms, wl[i].us, wl[i].bytes);
    rt_flush();
    {
        char n[32]; snprintf(n, sizeof(n), "ML/RT%02d.IDX", rt_run_id);
        FILE *f = FIO_CreateFile(n);
        if (f) { FIO_WriteFile(f, fl, sizeof(*fl) * MIN(nfr, RT_MAXFR)); FIO_CloseFile(f); }
    }

done:
    rt_unhook();
    if (lv_raw) call("lv_save_raw", 0);
    if (rt_begun) rt_end();
    if (w_done) rt_free_chunks();
    rt_unlock();
    rt_printf("done t=%d\n", get_ms_clock());
    rt_flush();
    rt_export(rt_run_id);
    rt_runs++;
    NotifyBox(8000, "Record test: DONE %s", rt_last);
out:
    if (rt_logbuf) free(rt_logbuf);
    if (fl) free(fl);
    if (wl) free(wl);
    if (tl) free(tl);
    if (smp) free(smp);
    rt_logbuf = 0; fl = 0; wl = 0; tl = 0; smp = 0;
    info_led_blink(3, 100, 100);
    rt_busy = 0;
}

/* ---- v14 pack test: does Mem1Path's EDMAC pack-unit config select the bit order? ----
 * Static RE (notes lv-survey.md, "Pixel packing control"): port 3 (MEM1) has pack
 * unit 2 (mmio 0xD0422200); its config word at +0x24 is composed by 0xE0537416
 * from 5 fields kept in the Mem1Path struct at +0x64 (initial {0, 1, 2, 1, 0}):
 * reg = f1<<20 | f4<<16 | f0<<8 | f2<<4 | f3. Hypotheses: f2 = bit depth (2 = 14),
 * f1 (only 1 in Mem1Path) = bit order, f4 = swap mode. For f1 x f4, set the struct
 * fields (Canon re-applies them per frame, if FUN_e0638b7a runs per frame) and the
 * register itself, wait, copy a 16-row band of MEM1, log the register before/after.
 * 14-bit only, so the port-3 sizes stay right. Everything is restored at the end.
 * Output: ML/PACKTEST.BIN (header + band per config), analysed on the PC. */
#define PK_CFG      (RT_PATH + 0x64)    /* 5 words */
#define PK_REG      0xD0422224u         /* pack unit 2 config */
#define PK_MASK     0x00130131u
#define PK_ROWS     16
#define PK_FILE     "ML/PACKTEST.BIN"

static uint32_t pk_compose(const uint32_t *f)
{
    return (f[1] & 1) << 20 | (f[4] & 3) << 16 | (f[0] & 1) << 8 | (f[2] & 3) << 4 | (f[3] & 1);
}

struct pk_hdr
{
    char magic[4];                      /* "PKT1" */
    uint32_t idx, f1, f4, reg_before, reg_set, reg_after, w, h, pitch, row0, rows, t_ms;
};

static void rt_pack_task(void *unused)
{
    static const uint8_t combos[][2] = {    /* {f1, f4}; first = Canon's default, last = after restore */
        { 1, 0 }, { 0, 0 }, { 1, 1 }, { 0, 1 }, { 1, 2 }, { 0, 2 }, { 1, 3 }, { 0, 3 },
    };
    rt_logbuf = malloc(RT_LOGLEN);
    rt_loglen = 0;
    uint8_t *band = 0;
    FILE *f = 0;
    int lv_raw = 0, changed = 0, nb = 0;
    uint32_t orig[5], reg0 = 0;
    if (!rt_logbuf) { NotifyBox(6000, "Pack test: out of memory"); goto out; }
    rt_printf("start v14 pack test: t=%d lv=%d\n", get_ms_clock(), lv ? 1 : 0);
    if (!lv) { rt_printf("no live view, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: no LV"); goto done; }

    call("lv_save_raw", 1);
    lv_raw = 1;
    msleep(500);
    m1_w = MEM(RT_PATH + 0x10); m1_h = MEM(RT_PATH + 0x14);
    uint32_t a = MEM(RT_PATH + 0x58);
    pitch = m1_w * 14 / 8;
    ubuf = RT_BUF | RT_UNC;
    rt_printf("M1 w=%d h=%d addr=%08x pitch=%d\n", m1_w, m1_h, a, pitch);
    if (!m1_w || !m1_h || pitch * m1_h > RT_BUFLEN || (a & ~RT_UNC) != RT_BUF || m1_h < 64)
    { rt_printf("MEM1 not active or size unexpected, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: no LV raw"); goto done; }

    for (int i = 0; i < 5; i++) orig[i] = MEM(PK_CFG + 4 * i);
    reg0 = MEM(PK_REG);
    rt_printf("pack cfg {%d,%d,%d,%d,%d} -> composed %08x; unit reg %08x (masked %08x)\n", orig[0], orig[1], orig[2],
              orig[3], orig[4], pk_compose(orig), reg0, reg0 & PK_MASK);
    if (orig[0] != 0 || orig[1] != 1 || orig[2] != 2 || orig[3] != 1 || orig[4] != 0)
    { rt_printf("cfg is not Canon's default {0,1,2,1,0}: not touching it, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: cfg unexpected"); goto done; }
    rt_printf("unit reg %s the struct\n", (reg0 & PK_MASK) == pk_compose(orig) ? "matches" : "does NOT match");

    band = malloc(PK_ROWS * pitch);
    f = FIO_CreateFile(PK_FILE);
    if (!band || !f) { rt_printf("no buffer/file, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: no buffer"); goto done; }
    uint32_t row0 = (m1_h / 2) & ~1;
    rt_flush();

    for (int c = 0; c <= COUNT(combos); c++)
    {
        int restore = c == COUNT(combos);
        uint32_t cfg[5] = { orig[0], orig[1], orig[2], orig[3], orig[4] };
        if (!restore) { cfg[1] = combos[c][0]; cfg[4] = combos[c][1]; }
        uint32_t rb = MEM(PK_REG);
        MEM(PK_CFG + 4) = cfg[1];
        MEM(PK_CFG + 16) = cfg[4];
        if (!restore) MEM(PK_REG) = (rb & ~PK_MASK) | pk_compose(cfg);
        changed = 1;
        uint32_t rs = MEM(PK_REG);
        msleep(250);                    /* ~7 frames at 29.97p */
        uint32_t ra = MEM(PK_REG);
        if (restore)                    /* struct only so far: did Canon re-apply it per frame? */
        {
            rt_printf("restore: struct only -> unit reg after 250 ms %08x (%s)\n", ra,
                      ra == reg0 ? "Canon re-applies the struct every frame" : "NOT re-applied; writing the register");
            if (ra != reg0) { MEM(PK_REG) = reg0; msleep(250); ra = MEM(PK_REG); }
        }
        changed = !restore;
        memcpy(band, (void *)(ubuf + row0 * pitch), PK_ROWS * pitch);
        struct pk_hdr h = { { 'P', 'K', 'T', '1' }, c, cfg[1], cfg[4], rb, rs, ra, m1_w, m1_h, pitch, row0, PK_ROWS,
                            get_ms_clock() };
        FIO_WriteFile(f, &h, sizeof(h));
        FIO_WriteFile(f, band, PK_ROWS * pitch);
        nb++;
        rt_printf("band %d: f1=%d f4=%d%s reg before %08x set %08x after 250 ms %08x; first bytes %02x%02x%02x%02x\n", c,
                  cfg[1], cfg[4], restore ? " (restored)" : "", rb, rs, ra, band[pitch / 2], band[pitch / 2 + 1],
                  band[pitch / 2 + 2], band[pitch / 2 + 3]);
    }
    changed = 0;
    snprintf(rt_last, sizeof(rt_last), "pack: %d bands", nb);

done:
    if (changed)                        /* aborted mid-way: put Canon's values back */
    {
        MEM(PK_CFG + 4) = orig[1]; MEM(PK_CFG + 16) = orig[4]; MEM(PK_REG) = reg0;
        rt_printf("restored cfg after an abort\n");
    }
    if (f) FIO_CloseFile(f);
    if (band) free(band);
    if (lv_raw) call("lv_save_raw", 0);
    rt_printf("pack test done t=%d: %s\n", get_ms_clock(), rt_last);
    rt_flush();
    rt_copy_file(PK_FILE, "DCIM/100EOS_R/RTPK0998.JPG");
    rt_copy_file("ML/RECTEST.TXT", "DCIM/100EOS_R/RTLG0997.JPG");
    NotifyBox(8000, "Pack test: DONE %s", rt_last);
out:
    if (rt_logbuf) free(rt_logbuf);
    rt_logbuf = 0;
    info_led_blink(3, 100, 100);
    rt_busy = 0;
}

static MENU_SELECT_FUNC(rt_pack_select)
{
    if (rt_busy) return;
    rt_busy = 1;
    rt_last[0] = 0;
    task_create("rt_pack", 0x1e, 0x2000, rt_pack_task, 0);
}

static MENU_SELECT_FUNC(rt_select)
{
    if (rt_busy) return;
    rt_busy = 1;
    rt_last[0] = 0;
    task_create("rt_task", 0x1e, 0x2000, rt_task, 0);
}

static MENU_UPDATE_FUNC(rt_update)
{
    if (rt_busy) MENU_SET_VALUE("running...");
    else if (rt_last[0]) MENU_SET_VALUE("%s", rt_last);
    else MENU_SET_VALUE("%d runs", rt_runs);
}

static struct menu_entry rt_menu[] = {
    {
        .name   = "LV record test",
        .select = menu_open_submenu,
        .help   = "Raw record test: frame-synced EDMAC crop copy + batched card writes",
        .children = (struct menu_entry[]) {
            {
                .name    = "Crop",
                .priv    = &rt_crop_sel,
                .max     = 2,
                .choices = CHOICES("1920x1080", "1920x960", "1920x800"),
            },
            {
                .name    = "Duration",
                .priv    = &rt_dur_sel,
                .max     = 2,
                .choices = CHOICES("10 s", "20 s", "40 s"),
            },
            {
                .name    = "Sync",
                .priv    = &rt_sync_sel,
                .max     = 2,
                .choices = CHOICES("auto", "IRQ", "timer"),
                .help    = "auto: frame-end IRQ if the survey shows it works, else HPTimer",
            },
            {
                .name   = "Start",
                .select = rt_select,
                .update = rt_update,
                .help   = "NOT recording. SET, close menu, wait for DONE (duration + ~5 s).",
                .help2  = "Writes ML/RTnn.BIN (~2 GB per 20 s). Freeze: pull battery.",
            },
            {
                .name   = "Pack test",
                .select = rt_pack_select,
                .update = rt_update,
                .help   = "v14: tries EDMAC pack-unit settings on the LV raw, saves bands to ML/PACKTEST.BIN.",
                .help2  = "~3 s. Restores Canon's values. Freeze: pull battery.",
            },
            MENU_EOL,
        },
    },
};

/* ---- export for USB: over PTP the R shows only files in DCIM/100EOS_R with
 * image-like names (v13.2 probe: RTPB0002.JPG visible; .TXT files, MISC and the
 * DCIM root hidden). v13.3 test: a file is listed only if its 4-digit number
 * is 0001-9999 AND unique in the folder (two names sharing a number: both
 * hidden). So each run's exports get their own numbers: run r -> RTLG(3r+1),
 * RTIX(3r+2), RTSY(3r+3) .JPG (runs 0..99 -> 0001..0300); status RTST0999.JPG.
 * Canon playback shows them as unreadable images; harmless. ---- */
#define RT_EXPORT_DIR "DCIM/100EOS_R/"

static void rt_copy_file(const char *src, const char *dst)
{
    int len = 0;
    uint8_t *b = read_entire_file(src, &len);
    if (!b) return;
    FILE *f = FIO_CreateFile(dst);
    if (f) { FIO_WriteFile(f, b, len); FIO_CloseFile(f); }
    fio_free(b);
}

static void rt_export(int run)
{
    char s[32], d[48];
    int n = 3 * run + 1;
    snprintf(d, sizeof(d), RT_EXPORT_DIR "RTLG%04d.JPG", n); rt_copy_file("ML/RECTEST.TXT", d);
    snprintf(s, sizeof(s), "ML/RT%02d.IDX", run); snprintf(d, sizeof(d), RT_EXPORT_DIR "RTIX%04d.JPG", n + 1); rt_copy_file(s, d);
    snprintf(s, sizeof(s), "ML/RT%02d.SYN", run); snprintf(d, sizeof(d), RT_EXPORT_DIR "RTSY%04d.JPG", n + 2); rt_copy_file(s, d);
}

/* status for USB: which plan ran, and what's left */
static void rt_status(const char *fmt, ...)
{
    char t[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(t, sizeof(t) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    n = MIN(n, (int)sizeof(t) - 2);
    t[n++] = '\n';
    FILE *f = FIO_CreateFile(RT_EXPORT_DIR "RTST0999.JPG");
    if (f) { FIO_WriteFile(f, t, n); FIO_CloseFile(f); }
}

/* ---- auto-run: ML/RTAUTO.TXT, unattended runs ----
 * Read 10 s after boot. Plans are separated by lines starting with "---";
 * each boot runs the next plan (ML/RTAUTO.POS holds its number). A plan has up
 * to 6 lines "<crop height> <seconds> <auto|irq|timer>", e.g. "1080 20 auto", or "pack" (v14 pack test);
 * '#' starts a comment. Waits up to 30 s for live view; without it (e.g. USB
 * cable in: PC-connection screen) nothing is used up. RTAUTO.POS is advanced
 * before the plan runs, so a freeze + battery pull skips that plan instead of
 * looping. After the last plan RTAUTO.TXT is renamed to RTAUTO.DON. */
#define RT_AUTO_CFG "ML/RTAUTO.TXT"
#define RT_AUTO_POS "ML/RTAUTO.POS"
#define RT_AUTO_DON "ML/RTAUTO.DON"

static int rt_auto_skipped;
static int rt_auto_parse(const char *t, int len, int (*runs)[3], int max)
{
    int n = 0, i = 0;
    while (i < len && n < max)
    {
        int v[3] = { -1, -1, -1 }, k = 0;
        while (i < len && t[i] != '\n')
        {
            char c = t[i];
            if (c == '#') { while (i < len && t[i] != '\n') i++; break; }
            if (c >= '0' && c <= '9' && k < 2)
            {
                int x = 0;
                while (i < len && t[i] >= '0' && t[i] <= '9') x = x * 10 + (t[i++] - '0');
                v[k++] = x;
                continue;
            }
            if (k == 0 && (c == 'p' || c == 'P'))  /* "pack": pack test */
            {
                k = 9;
                while (i < len && t[i] != '\n') i++;
                break;
            }
            if (k == 2 && (c == 'a' || c == 'i' || c == 'I' || c == 't'))
            {
                v[2] = c == 'a' ? SYNC_AUTO : c == 't' ? SYNC_TIMER : SYNC_IRQ;
                k = 3;
                while (i < len && t[i] != '\n' && t[i] != ' ' && t[i] != '\t' && t[i] != '\r') i++;
                continue;
            }
            i++;
        }
        i++;
        if (k == 0) continue;           /* blank or comment line */
        if (k == 9) { runs[n][0] = -1; runs[n][1] = runs[n][2] = 0; n++; continue; }
        int cs = v[0] == 1080 ? 0 : v[0] == 960 ? 1 : v[0] == 800 ? 2 : -1;
        int ds = v[1] == 10 ? 0 : v[1] == 20 ? 1 : v[1] == 40 ? 2 : -1;
        if (k < 2 || cs < 0 || ds < 0) { rt_auto_skipped++; continue; }
        runs[n][0] = cs; runs[n][1] = ds; runs[n][2] = v[2] < 0 ? SYNC_AUTO : v[2];
        n++;
    }
    return n;
}

/* find plan k (0-based) in the text: start offset and length of its chunk; returns number of plans */
static int rt_auto_plan(const char *t, int len, int k, int *start, int *plen)
{
    int np = 0, ls = 0, cs = 0;
    *start = 0; *plen = 0;
    for (int i = 0; i <= len; i++)
    {
        if (i < len && t[i] != '\n') continue;
        int sep = i - ls >= 3 && t[ls] == '-' && t[ls + 1] == '-' && t[ls + 2] == '-';
        if (sep || i == len)
        {
            int end = sep ? ls : len;
            if (np == k) { *start = cs; *plen = end - cs; }
            np++;
            cs = i + 1;
        }
        ls = i + 1;
    }
    return np;
}

static int rt_read_int(const char *name, int dflt)
{
    int len = 0, v = 0, any = 0;
    char *b = (char *)read_entire_file(name, &len);
    if (!b) return dflt;
    for (int i = 0; i < len && b[i] >= '0' && b[i] <= '9'; i++) { v = v * 10 + b[i] - '0'; any = 1; }
    fio_free(b);
    return any ? v : dflt;
}

static void rt_write_int(const char *name, int v)
{
    char t[16];
    int n = snprintf(t, sizeof(t), "%d\n", v);
    FILE *f = FIO_CreateFile(name);
    if (f) { FIO_WriteFile(f, t, n); FIO_CloseFile(f); }
}

static void rt_auto_task(void *unused)
{
    msleep(10000);
    if (!is_file(RT_AUTO_CFG)) return;
    int len = 0;
    char *t = (char *)read_entire_file(RT_AUTO_CFG, &len);
    if (!t) return;
    int pos = rt_read_int(RT_AUTO_POS, 0), ps, pl;
    int np = rt_auto_plan(t, len, pos, &ps, &pl);
    int runs[6][3];
    rt_auto_skipped = 0;
    int n = pos < np ? rt_auto_parse(t + ps, pl, runs, 6) : 0;
    fio_free(t);
    if (pos >= np)
    {
        FIO_RemoveFile(RT_AUTO_DON);
        FIO_RenameFile(RT_AUTO_CFG, RT_AUTO_DON);
        FIO_RemoveFile(RT_AUTO_POS);
        rt_status("auto: all %d plans done", np);
        return;
    }
    for (int w = 0; w < 30 && !lv; w++) msleep(1000);
    if (!lv)
    {
        NotifyBox(6000, "Record test auto: no live view, plan %d of %d kept", pos + 1, np);
        rt_status("auto: plan %d of %d waiting (no live view at boot)", pos + 1, np);
        return;
    }
    rt_write_int(RT_AUTO_POS, pos + 1);     /* before running: a freeze can't repeat this plan */
    if (rt_auto_skipped)
    {
        NotifyBox(5000, "Record test auto: %d invalid line(s) skipped", rt_auto_skipped);
        msleep(5000);
    }
    if (!n) rt_status("auto: plan %d of %d has no valid runs", pos + 1, np);
    int first_run = -1;
    for (int i = 0; i < n; i++)
    {
        while (rt_busy) msleep(500);
        NotifyBox(3000, "Record test auto: plan %d/%d, run %d of %d", pos + 1, np, i + 1, n);
        rt_status("auto: plan %d of %d, run %d of %d in progress", pos + 1, np, i + 1, n);
        msleep(3000);
        rt_busy = 1;
        rt_last[0] = 0;
        if (runs[i][0] < 0) { rt_pack_task(0); continue; }   /* synchronous; clears rt_busy */
        rt_crop_sel = runs[i][0]; rt_dur_sel = runs[i][1]; rt_sync_sel = runs[i][2];
        rt_task(0);                     /* synchronous; clears rt_busy */
        if (first_run < 0) first_run = rt_run_id;
        msleep(3000);
    }
    if (pos + 1 >= np)
    {
        FIO_RemoveFile(RT_AUTO_DON);
        FIO_RenameFile(RT_AUTO_CFG, RT_AUTO_DON);
        FIO_RemoveFile(RT_AUTO_POS);
    }
    rt_status("auto: plan %d of %d done, runs RT%02d..RT%02d; last: %s", pos + 1, np, first_run, rt_run_id, rt_last);
    NotifyBox(10000, "Record test auto: plan %d of %d DONE (%d runs)", pos + 1, np, n);
}

static void rt_init(void *unused)
{
    menu_add("Debug", rt_menu, COUNT(rt_menu));
    task_create("rt_auto", 0x1e, 0x2000, rt_auto_task, 0);
}

INIT_FUNC("rectest", rt_init);
