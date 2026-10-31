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
#include "config.h"
#include "bmp.h"
#include "gui-common.h"
#include "property.h"   /* LVMS_ENABLE_MOVIE (crop marker) */
#include "zebra.h"      /* get_global_draw (crop marker) */

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

#define RT_MAXCH      32                /* v20: silent pictures take one-frame chunks (v3.13: + SRM slots); video stays at 10 */
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
#define F_MIDTEAR 64                    /* v20: a sampled row of the copy differs from the source at copy start */
#define F_TORN   (F_TEAR | F_BOTCHG | F_MIDTEAR)

enum { SYNC_AUTO = 0, SYNC_IRQ, SYNC_TIMER };
static CONFIG_INT("r.rawrec.crop", rt_crop_sel, 0);             /* index into rt_crop_wh; v16.6: 3 = full MEM1 frame */
static int rt_dur_sel = 1;              /* 10, 20, 40 s */
static int rt_sync_sel = SYNC_AUTO;
static CONFIG_INT("r.rawrec.fmt", rt_fmt_sel, 0);              /* v15: 0 = BIN 14-bit Canon order, 1 = MLV 14-bit, 2 = MLV 12-bit, v16.6: 3 = MLV 10-bit */
static const char *rt_fmt_name[] = { "BIN 14-bit", "MLV 14-bit", "MLV 12-bit", "MLV 10-bit" };
static const int rt_fmt_bpp[] = { 14, 14, 12, 10 };
/* v17: crops as width x height; 0 x 0 = the whole MEM1 frame (FHD movie 2304x1290, 4K movie 4256x2214).
 * The 2560 crops need Canon's 4K movie size (MEM1 4256 px wide, 1:1); in FHD they clamp to 2304. */
static const uint16_t rt_crop_wh[][2] = { { 1920, 1080 }, { 1920, 960 }, { 1920, 800 }, { 0, 0 }, { 2560, 1080 }, { 2560, 1440 },
                                          { 2880, 1216 } };     /* v17.2: 2.37:1, 10-bit 23.98p needs 100.1 MiB/s */
#define RT_CROP_FULL 3
static const int rt_dur_s[] = { 10, 20, 40 };
static const char *rt_sync_name[] = { "auto", "IRQ", "timer" };

static volatile int rt_busy = 0;
int rawrec_running(void) { return rt_busy; }   /* zebra.c: no overlay LV copies while the recorder runs */
/* v16: REC-button raw recording. rr_on = menu switch; rr_active = the current run was started by REC
 * (no fixed duration; stops on the next REC press, rr_stop); rr_t0/rr_frames feed the on-screen status. */
static CONFIG_INT("r.rawrec.rec", rr_on, 0);
static volatile int rr_active, rr_stop;
/* v18: personal recorder front end (Movie > RAW video). REC-button clips are named like mlv_lite's
 * (M<day>-<hhmm>.MLV at the card root). Their test diagnostics (.IDX/.SYN, USB exports, key log, timeline
 * and write lists in the log) are written only with Debug > LV record test > REC diagnostics ON; timed test
 * runs (menu Start, auto-run plans) always write them. */
static CONFIG_INT("r.rawrec.diag", rr_diag, 0);
static CONFIG_INT("r.rawrec.card10", rr_card10, 1060);       /* card write speed of the last clip, MiB/s x10 */
static char rr_base[16];                /* clip name without extension */
static volatile int rr_full;            /* last clip stopped because the card was nearly full */
/* v20: LV silent pictures (Shoot > LV silent pictures). A session is one rt_task run with sp_active set: it arms the
 * engine (MEM1 tap, pack unit in ML order, IRQ hook, chunks), copies whole MEM1 frames (4K movie LV: 4256x2214 1:1)
 * while the trigger is held (Burst) or all the time into a short ring (Pre-capture), and sp_writer turns each kept
 * frame into a DNG (4096-byte header in the slot, pixels byte-swapped to DNG's MSB-first order). Keys: see sp_keys. */
static CONFIG_INT("r.silent.lv", sp_mode, 0);            /* 0 OFF, 1 Burst, 2 Pre-capture */
static CONFIG_INT("r.silent.lv.bpp", sp_bits, 0);        /* v3.8: 0 = 10-bit, 1 = 12-bit (12-bit 4K copy ~30 ms: tears) */
static CONFIG_INT("r.silent.lv.pre", sp_pre_sel, 1);     /* frames kept before the press: sp_pre_n[] */
static CONFIG_INT("r.silent.lv.arm", sp_arm_sel, 0);     /* 0 half-press, 1 always (movie LV, ML menu closed) */
static const int sp_pre_n[] = { 3, 5, 7, 9 };
static volatile int sp_active;          /* the current rt_task run is a silent-picture session */
static volatile int sp_ready;           /* armed: capture loop running */
static volatile int sp_trig;            /* trigger held (may come while arming: saving starts once armed) */
static volatile int sp_end;             /* end the session (released, cancelled, LV gone) */
static volatile int sp_rel;             /* trigger released: end once at least one frame is kept (quick taps)  */
static volatile int sp_sim;             /* Debug timed test running */
static volatile int sp_kept, sp_saved, sp_pre_kept;   /* frames kept for DNG, DNGs written, of them before the press */
static volatile int sp_half;            /* v3.8: half-press held (0x7D seen, 0x7E not yet) */
static volatile int sp_n85;             /* lone 0x85 events in a row (full press held: see sp_keys) */
static volatile uint32_t sp_t85;        /* ms of the last 0x85 */
static volatile int sp_wait_rel;
/* v3.9: bmp.c skips ML display refreshes while this is set (from the press to the end of capture). It did NOT fix the
 * mid-burst slowdown (v3.9 camera: 0 refreshes, same 31-37 ms copies): that followed the chunk (v3.10 survey). Kept:
 * cheap, and the screen doesn't need updating during a ~0.4 s burst. */
volatile int r_refresh_hold;
static uint32_t sp_ref0, sp_ref_cap;   /* refreshes since the press / during capture */        /* a session ended with the full press still held: no new one until it's let go */
#define SP_TICK_GAP_MS  250
/* full press let go but the half-press still held: the 0x85 stream stops */
static inline void sp_check_tick_end(void)
{
    if (sp_trig && !sp_sim && !sp_rel && get_ms_clock() - sp_t85 > SP_TICK_GAP_MS) sp_rel = 1;
}
static inline int rt_diag(void) { return !(rr_active || sp_active) || rr_diag; }
static volatile int rt_osd_run;         /* auto-run "osd": show the REC status during a timed run (A/B cost test) */
static volatile uint32_t rr_t0_ms;
static int rt_runs = 0;
static int rt_run_id;
static char rt_last[80] = "";

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
struct rt_chunk { struct memSuite *suite; uint32_t base; volatile int state; volatile int n;
                  uint32_t seq, t_us; int fl; uint32_t cp_us; };   /* v20 silent pictures (1 frame per chunk): frame event no., time, flags; v3.10 survey copy time */
enum { CH_FREE = 0, CH_FILL, CH_FULL, CH_RING, CH_SKIP };   /* v20: CH_RING = pre-capture frame, not (yet) to be written; v3.10: CH_SKIP = too slow to copy into */
#define RT_MAXCH_VIDEO 10                   /* raw video: as before v20 */
static struct rt_chunk ch[RT_MAXCH];
static int nch, kslots;
static uint32_t slot_bytes, frame_bytes;

static void rt_free_chunks(void)
{
    for (int i = 0; i < nch; i++) if (ch[i].suite) shoot_free_suite(ch[i].suite);
    memset(ch, 0, sizeof(ch));
    nch = 0;
}

/* ---- v3.13: Canon's still-capture (SRM "1st job") buffers as extra silent-picture memory (Shoot > LV silent
 * pictures > Extra memory). Addresses from static RE (lv-survey.md "SRM 1st job buffers on the R"). Each buffer is
 * carved into one-frame slots appended to ch[] (suite 0: not shoot memory). Freed in allocation order after the
 * writer is done. Canon can't take a photo while they're held (Err 70 on other bodies); movie mode doesn't. */
#ifndef RT_HOST
static void (*const SRM_Alloc1st)(void (*cbr)(void *, void *, uint32_t), void *ctx) = (void *)0xE04E41BF;
static void (*const SRM_Free1st)(void *buf, void *cbr, void *ctx)                    = (void *)0xE04E7591;
#endif
static CONFIG_INT("r.silent.lv.srm", sp_srm_sel, 0);    /* 0 OFF, 1 one buffer, 2 up to 4 */
#define SP_SRM_MAX 4
static void *volatile sp_srm_buf[SP_SRM_MAX + 1];
static volatile uint32_t sp_srm_sz[SP_SRM_MAX + 1];
static int sp_srm_n;
static struct semaphore *sp_srm_sem;
static void sp_srm_cbr(void *ctx, void *buf, uint32_t size)
{
    int i = (int)ctx;
    if (i >= 0 && i <= SP_SRM_MAX) { sp_srm_buf[i] = buf; sp_srm_sz[i] = size; }
    give_semaphore(sp_srm_sem);
}
static void sp_srm_alloc(int want)
{
    if (!sp_srm_sem) sp_srm_sem = create_named_semaphore("sp_srm", SEM_CREATE_LOCKED);
    for (int i = 0; i <= SP_SRM_MAX; i++) { sp_srm_buf[i] = 0; sp_srm_sz[i] = 0; }
    sp_srm_n = 0;
    for (int i = 0; i < MIN(want, SP_SRM_MAX); i++)
    {
        SRM_Alloc1st(sp_srm_cbr, (void *)i);
        if (take_semaphore(sp_srm_sem, 1000)) { rt_printf("SRM: request %d got no buffer within 1 s\n", i); break; }
        rt_printf("SRM: buffer %d at %08x, %d KB\n", i, (uint32_t)sp_srm_buf[i], sp_srm_sz[i] / 1024);
        sp_srm_n++;
    }
}
static void sp_srm_free(void)
{
    for (int i = 0; i < sp_srm_n; i++) { SRM_Free1st(sp_srm_buf[i], 0, 0); msleep(20); }
    if (sp_srm_n < SP_SRM_MAX && sp_srm_buf[sp_srm_n])        /* a request served after its timeout */
    { rt_printf("SRM: late buffer %08x freed\n", (uint32_t)sp_srm_buf[sp_srm_n]); SRM_Free1st(sp_srm_buf[sp_srm_n], 0, 0); sp_srm_buf[sp_srm_n] = 0; }
    if (sp_srm_n) rt_printf("SRM: %d buffers freed\n", sp_srm_n);
    sp_srm_n = 0;
}
/* append each SRM buffer's whole slots to ch[] */
static void sp_srm_carve(void)
{
    for (int b = 0; b < sp_srm_n; b++)
    {
        uint32_t a = (uint32_t)sp_srm_buf[b], base = ((uint32_t)UNCACHEABLE(a) + 4095) & ~4095;
        uint32_t end = (uint32_t)UNCACHEABLE(a) + sp_srm_sz[b];
        int k = 0;
        for (uint32_t p = base; p + slot_bytes <= end && nch < RT_MAXCH; p += slot_bytes, k++)
        {
            memset(&ch[nch], 0, sizeof(ch[nch]));
            ch[nch].base = p; ch[nch].state = CH_FREE;
            nch++;
        }
        rt_printf("SRM buffer %d: %d frame slots\n", b, k);
    }
}

static void rt_alloc_chunks(int maxch)
{
    uint32_t want = kslots * slot_bytes + 4096;
    for (int i = 0; i < MIN(maxch, RT_MAXCH); i++)
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
        rt_printf("chunk %d at %08x, used from %08x, %d bytes\n", i, a, ch[i].base, sz);   /* v3.10 */
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

/* v3.11 (silent pictures, mid-frame copy): at each frame-end IRQ hash SP_NIH rows of the frame just completed; the
 * copy of that frame, started mid-frame, is checked against them afterwards. Two sets, by IRQ number parity. */
#define SP_NIH 16
static volatile int sp_ih_on;
static uint32_t sp_ih_row[SP_NIH];
static volatile uint32_t sp_ih[2][SP_NIH], sp_ih_n[2];

static void rt_m1_isr(void *ctx)
{
    uint32_t t = RT_DMA_TIMER();
    uint32_t top = rt_probe(r_top), bot = rt_probe(r_bot);
    irq_seq++;
    RT_DMB();
    irq_t = t; irq_top = top; irq_bot = bot; irq_n++;
    RT_DMB();
    irq_seq++;
    if (sp_ih_on)
    {
        uint32_t nn = irq_n, k = nn & 1;
        sp_ih_n[k] = 0;
        for (int j = 0; j < SP_NIH; j++) sp_ih[k][j] = rt_probe(sp_ih_row[j]);
        RT_DMB();
        sp_ih_n[k] = nn;
    }
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
     * crop row yet (no top change within 1 ms after the IRQ).
     * v17.1: 8 ms was tuned with the probe row ~107 rows above the frame end (FHD 1080 crop). A probe row
     * higher up (4K mode 2560x1080: 568 rows above the end, IRQ ~7.8 ms after it, rejected on 6-8 of 31 IRQs)
     * gets the measured time of the extra rows added: rows to the standard probe row x (readout / probe span). */
    uint32_t pm = MAX(107, m1_h * 107 / 1290), lim = 8000;
    if (sv_readout && r_bot > r_top + 64 && m1_h > pm && r_bot < m1_h - pm)
        lim += (uint32_t)((uint64_t)(m1_h - pm - r_bot) * sv_readout / (r_bot - r_top));
    lim = MIN(lim, 20000);
    if (nb)
        for (int i = 0; i < ni; i++)
        {
            uint32_t ab = 0xFFFFFFFF, nt_ = 0xFFFFFFFF;
            for (int j = 0; j < nb; j++) { uint32_t d = d20(irq_log[i].t, bt[j]); if (d < ab) ab = d; }
            for (int j = 0; j < nt; j++) { uint32_t d = d20(tt[j], irq_log[i].t); if (d < nt_) nt_ = d; }
            if (ab > 500000) continue;      /* no sample coverage around this IRQ */
            sv_irq_chk++;
            if (ab < lim && nt_ > 1000) sv_irq_settled++;
        }
    rt_printf("survey: %d bottom-row changes, period %d us; readout (top->bottom crop row) %d us\n", nb, sv_est, sv_readout);
    rt_printf("survey: IRQ period %d us, %d of %d IRQ intervals had a new bottom row; IRQ %d us after the bottom-row "
              "change; next top-row change %d us after the IRQ\n", sv_irq_med, sv_irq_new, ni > 1 ? ni - 1 : 0,
              sv_irq_after_bot, sv_irq_to_top);
    rt_printf("survey: IRQ right after a completed frame (bottom row changed < %d.%d ms before, top row not yet) "
              "in %d of %d checked IRQs\n", lim / 1000, lim / 100 % 10, sv_irq_settled, sv_irq_chk);

    char n[32]; snprintf(n, sizeof(n), "ML/RT%02d.SYN", rt_run_id);
    FILE *f = rt_diag() ? FIO_CreateFile(n) : 0;
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
/* ---- v15: MLV output ----
 * Pack-unit settings found by the v14.1 test (notes lv-survey.md): Mem1Path pack cfg
 * f3 = 0 gives ML raw_pixblock bit order, f2 = 1 gives 12-bit (with the width field
 * +0x18 set so Canon's port-3 xsize = (width*14)>>3 = w*12/8). Canon re-applies the
 * struct every frame. In MLV mode each buffer slot is one MLV block group:
 * VIDF header (32) + frameSpace (224) + frame, then a NULL block for any slack, so
 * the card writes stay whole chunks. Each file starts with MLVI+RAWI+IDNT+RTCI; the
 * MLVI frame count is rewritten when the file is closed. */
#define RT_VIDF_PRE  256
#define RT_BLACK14   2043               /* left OB of the movie-LV dump */
#define RT_WHITE14   16300              /* clip pile-up at 16382 */
#define RT_OB_LEFT   48                 /* FHD movie MEM1 2304x1290: optical-black columns (left) */
#define RT_OB_TOP    22                 /* ... and rows (top) */
#define RT_HDR_INFO  (52 + 180 + 84 + 44)
#define RT_HDR_BYTES 4096               /* header + NULL padding: keeps every chunk write 4 KiB-aligned in the file.
                                         * v15/v16 wrote 360-byte headers and the card ran at 98-99 MiB/s instead of
                                         * the 111-112 of BIN runs (all writes 360 bytes off sector alignment). */
struct __attribute__((packed)) rt_mlvi
{
    char magic[4]; uint32_t size; char ver[8]; uint64_t guid; uint16_t file_num, file_count; uint32_t flags;
    uint16_t vclass, aclass; uint32_t vframes, aframes, fps_nom, fps_den;
};
struct __attribute__((packed)) rt_vidf
{
    char type[4]; uint32_t size; uint64_t ts; uint32_t frame; uint16_t crop_x, crop_y, pan_x, pan_y; uint32_t space;
};
struct __attribute__((packed)) rt_blk { char type[4]; uint32_t size; uint64_t ts; };
static int c_bpp = 14, c_mlv, c_dng, pk_applied;   /* v20: c_dng = silent-picture session (DNG per frame) */
static uint32_t c_rb, c_ch;
static uint32_t slot_pre, slot_null, c_x0, c_y0, c_cw, mlv_fps_nom, mlv_fps_den;
static uint64_t mlv_guid;
static uint32_t pk_orig[5], pk_w0;
static uint8_t *mlv_hdr;                /* RT_HDR_BYTES, malloc'd per MLV run */

/* v17: EDMAC copy self-test with the run's exact geometry: source pitch, crop byte offset, bytes per row and
 * address alignment (chunk bases are 4 KiB-aligned like MEM1 at 0x...A940 mod 64 = 0). In 4K movie mode the
 * 4256-px rows are only 8-byte aligned at 10/14-bit; every camera-verified copy before had 16-byte aligned rows.
 * Copies a 64-row pattern from chunk 0 to chunk 1 and compares every word. 0 = exact, -1 = mismatch, -2 = DMA timeout. */
static int rt_copy_selftest(uint32_t xoff, uint32_t chunk_bytes)
{
    uint32_t rows = 64, sb = ch[0].base, db = ch[1].base;
    rows = MIN(rows, MIN(chunk_bytes / pitch, chunk_bytes / c_rb));
    for (uint32_t r = 0; r < rows; r++)
        for (uint32_t i = 0; i < pitch / 4; i++) MEM(sb + r * pitch + 4 * i) = 0xA5000000u ^ (r << 16) ^ i;
    for (uint32_t i = 0; i < rows * c_rb / 4; i++) MEM(db + 4 * i) = 0xEEEEEEEEu;
    uint32_t us = 0;
    if (rt_copy(sb + xoff, pitch, db, c_rb, rows, &us)) { rt_printf("!! copy self-test: DMA timed out\n"); return -2; }
    int bad = 0;
    for (uint32_t r = 0; r < rows; r++)
        for (uint32_t j = 0; j < c_rb / 4; j++)
        {
            uint32_t got = MEM(db + r * c_rb + 4 * j), want = MEM(sb + r * pitch + xoff + 4 * j);
            if (got != want)
            {
                if (!bad) rt_printf("!! copy self-test: row %d byte %d: got %08x, want %08x\n", r, 4 * j, got, want);
                bad++;
                break;                      /* count rows */
            }
        }
    rt_printf("copy self-test: %d rows, pitch %d, offset %d, %d bytes/row, row starts mod 16 = %d/%d: %s (%d bad rows), dma %d us\n",
              rows, pitch, xoff, c_rb, xoff % 16, (xoff + pitch) % 16, bad ? "MISMATCH" : "exact", bad, us);
    return bad ? -1 : 0;
}

static int rt_pack_apply(int bpp, int ml_order)
{
    for (int i = 0; i < 5; i++) pk_orig[i] = MEM(RT_PATH + 0x64 + 4 * i);
    pk_w0 = MEM(RT_PATH + 0x18);
    uint32_t md = MEM(RT_PATH);
    if (pk_orig[0] != 0 || pk_orig[1] != 1 || pk_orig[2] != 2 || pk_orig[3] != 1 || pk_orig[4] != 0)
    { rt_printf("pack cfg {%d,%d,%d,%d,%d} is not Canon's default: not touching it\n", pk_orig[0], pk_orig[1], pk_orig[2], pk_orig[3], pk_orig[4]); return -1; }
    if (md >= 0x14 || (MEM(RT_PATH + 0x28) && (md == 0xb || md == 0xf || md == 9)) || pk_w0 != MEM(RT_PATH + 0x10))
    { rt_printf("Mem1Path mode %d / width source %d vs %d: not the layout tested in v14.1\n", md, pk_w0, MEM(RT_PATH + 0x10)); return -1; }
    uint32_t wset = pk_w0, f2 = bpp == 14 ? 2 : bpp == 12 ? 1 : 0;   /* v14.1: f2 2/1/0 = 14/12/10-bit */
    if (bpp != 14)
    {
        uint32_t lb = pk_w0 * bpp / 8;
        wset = (lb * 8 + 13) / 14;                  /* Canon sets port-3 xsize = (width * 14) >> 3 */
        if (((wset * 14) >> 3) != lb || (pk_w0 * bpp) % 8) { rt_printf("no width field gives %d bytes per line\n", lb); return -1; }
    }
    uint32_t old = cli();
    MEM(RT_PATH + 0x18) = wset;
    MEM(RT_PATH + 0x64 + 8) = f2;
    MEM(RT_PATH + 0x64 + 12) = ml_order ? 0 : 1;
    sei(old);
    pk_applied = 1;
    msleep(300);
    rt_printf("pack cfg: f2=%d f3=%d, width field %d -> %d (port-3 line %d bytes); unit reg %08x\n", f2,
              ml_order ? 0 : 1, pk_w0, wset, (wset * 14) >> 3, MEM(0xD0422224u));
    return 0;
}

static void rt_pack_restore(void)
{
    if (!pk_applied) return;
    uint32_t old = cli();
    MEM(RT_PATH + 0x18) = pk_w0;
    MEM(RT_PATH + 0x64 + 8) = pk_orig[2];
    MEM(RT_PATH + 0x64 + 12) = pk_orig[3];
    sei(old);
    pk_applied = 0;
    msleep(100);
    rt_printf("pack cfg restored: width field %d, unit reg %08x\n", MEM(RT_PATH + 0x18), MEM(0xD0422224u));
}

static void rt_fps_of(uint32_t period_us, uint32_t *nom, uint32_t *den)
{
    static const uint32_t c[][2] = { { 24000, 1001 }, { 24000, 1000 }, { 25000, 1000 }, { 30000, 1001 }, { 30000, 1000 },
                                     { 50000, 1000 }, { 60000, 1001 } };
    *nom = period_us ? (uint32_t)(1000000000ULL / period_us) : 30000; *den = 1000;
    for (int i = 0; i < COUNT(c); i++)
    {
        uint32_t p = (uint32_t)(1000000ULL * c[i][1] / c[i][0]);
        if (period_us > p - p / 50 && period_us < p + p / 50) { *nom = c[i][0]; *den = c[i][1]; return; }
    }
}

/* MLVI + RAWI + IDNT + RTCI (+ WAVI with audio, v19) for one file */
static volatile int ra_on;              /* v19 audio (see below) */
static volatile uint32_t ra_part_count;
static void rt_mlv_build_hdr(int file_num, uint32_t vframes)
{
    memset(mlv_hdr, 0, RT_HDR_BYTES);
    struct rt_mlvi *m = (struct rt_mlvi *)mlv_hdr;
    memcpy(m->magic, "MLVI", 4); m->size = 52; memcpy(m->ver, "v2.0", 5); m->guid = mlv_guid;
    m->file_num = file_num; m->file_count = 0; m->vclass = 1; m->vframes = vframes;
    m->fps_nom = mlv_fps_nom; m->fps_den = mlv_fps_den;
    if (ra_on) { m->aclass = 1; m->aframes = ra_part_count; }
    uint8_t *p = mlv_hdr + 52;
    struct rt_blk *b = (struct rt_blk *)p;                       /* RAWI */
    memcpy(b->type, "RAWI", 4); b->size = 180;
    uint16_t *res = (uint16_t *)(p + 16); res[0] = c_cw; res[1] = c_ch;
    int32_t *ri = (int32_t *)(p + 20);                           /* raw_info, 40 words */
    static const int32_t cm[9] = { 6446, -366, -864, -4436, 12204, 2513, -952, 2496, 6348 };  /* Adobe EOS R D65 */
    ri[0] = 1; ri[2] = c_ch; ri[3] = c_cw; ri[4] = c_rb; ri[5] = frame_bytes; ri[6] = c_bpp;
    int lsh = 14 - c_bpp;                                        /* hardware drops low bits: 12-bit ~ /4, 10-bit ~ /16 */
    ri[7] = (RT_BLACK14 + (lsh ? 1 << (lsh - 1) : 0)) >> lsh; ri[8] = RT_WHITE14 >> lsh;
    /* v16.7: optical-black borders of the FHD movie MEM1 frame (2304x1290, measured on v16.6 full-frame runs:
     * 48 columns left, 22 rows top, black 127.5 at 10-bit). Mark them outside the active area so raw
     * converters crop them (DNG ActiveArea); the data stays in the file for black-level checks. */
    uint32_t ax = 0, ay = 0, obl = 0, obt = 0;
    if (m1_w == 2304 && m1_h == 1290) { obl = RT_OB_LEFT; obt = RT_OB_TOP; }
    if (m1_w == 4256 && m1_h == 2214) { obl = 144; obt = 46; }   /* v17.1: 4K movie MEM1, measured on v17 RT00 (black 2049 @14-bit) */
    if (c_x0 < obl) ax = obl - c_x0;
    if (c_y0 < obt) ay = obt - c_y0;
    ri[9] = 0; ri[10] = 0; ri[11] = c_cw - ax; ri[12] = c_ch - ay; /* jpeg crop, relative to the active area */
    ri[13] = ay; ri[14] = ax; ri[15] = c_ch; ri[16] = c_cw;       /* active area: y1, x1, y2, x2 */
    ri[19] = 0x02010100; ri[20] = 21;                            /* RGGB, D65 */
    for (int i = 0; i < 9; i++) { ri[21 + 2 * i] = cm[i]; ri[22 + 2 * i] = 10000; }
    ri[39] = 1100;
    p += 180;
    b = (struct rt_blk *)p;                                      /* IDNT */
    memcpy(b->type, "IDNT", 4); b->size = 84;
    memcpy(p + 16, "Canon EOS R", 11);
    *(uint32_t *)(p + 48) = 0x80000424;
    p += 84;
    b = (struct rt_blk *)p;                                      /* RTCI */
    memcpy(b->type, "RTCI", 4); b->size = 44;
    struct tm now;
    memset(&now, 0, sizeof(now));
    LoadCalendarFromRTC(&now);
    uint16_t *t = (uint16_t *)(p + 16);
    t[0] = now.tm_sec; t[1] = now.tm_min; t[2] = now.tm_hour; t[3] = now.tm_mday; t[4] = now.tm_mon;
    t[5] = now.tm_year; t[6] = now.tm_wday; t[7] = now.tm_yday; t[8] = now.tm_isdst;
    p += 44;
    int wavi = 0;
    if (ra_on)                                                   /* v19: WAVI, 48 kHz 16-bit stereo PCM */
    {
        b = (struct rt_blk *)p;
        memcpy(b->type, "WAVI", 4); b->size = 32; b->ts = 1;         /* non-zero: mlv_dump ignores a WAVI at 0 */
        uint16_t *w16 = (uint16_t *)(p + 16); uint32_t *w32 = (uint32_t *)(p + 20);
        w16[0] = 1; w16[1] = 2;                                  /* format PCM, channels */
        w32[0] = 48000; w32[1] = 192000;                         /* sampling rate, bytes per second */
        uint16_t *w16b = (uint16_t *)(p + 28); w16b[0] = 4; w16b[1] = 16;   /* block align, bits per sample */
        p += 32; wavi = 32;
    }
    b = (struct rt_blk *)p;                                      /* NULL padding to RT_HDR_BYTES */
    memcpy(b->type, "NULL", 4); b->size = RT_HDR_BYTES - RT_HDR_INFO - wavi;
}

/* rewrite this file's MLVI with its frame count, then close it */
static void rt_mlv_close(FILE *f, int file_num, uint32_t vframes)
{
    rt_mlv_build_hdr(file_num, vframes);
    FIO_SeekSkipFile(f, 0, SEEK_SET);
    FIO_WriteFile(f, mlv_hdr, 52);
    FIO_CloseFile(f);
}

/* headers inside a slot: VIDF before the frame, NULL block after it */
static void rt_mlv_slot(uint32_t slot, uint32_t frame_no, uint32_t ts_us)
{
    struct rt_vidf v;
    memset(&v, 0, sizeof(v));
    memcpy(v.type, "VIDF", 4); v.size = RT_VIDF_PRE + frame_bytes; v.ts = ts_us; v.frame = frame_no;
    v.crop_x = c_x0 & ~7; v.crop_y = c_y0; v.pan_x = c_x0; v.pan_y = c_y0; v.space = RT_VIDF_PRE - sizeof(v);
    memcpy((void *)slot, &v, sizeof(v));
    if (slot_null)
    {
        struct rt_blk n = { { 'N', 'U', 'L', 'L' }, slot_null, ts_us };
        memcpy((void *)(slot + RT_VIDF_PRE + frame_bytes), &n, sizeof(n));
    }
}

static struct semaphore *rt_wsem;
static uint32_t first_t;     /* v19: forward (capture section) */

/* ---- v19 audio: the camera's microphone into the MLV (48 kHz, 16-bit, stereo) ----
 * Canon's SoundDev API (stage 2 probe, camera-verified 2026-10-30: clean start/stop, 0.5 s chunks of 96096 B at
 * 29.97p in a 5 s ring, samples big-endian). Canon's callbacks only queue chunk addresses; ra_task copies each chunk
 * (uncached, byte-swapped to little-endian) into a prepared AUDF block (padded with a NULL block to a 4 KiB multiple,
 * so the video writes stay 4 KiB-aligned); the writer task writes prepared blocks between video chunks.
 * Audio starts (SetInStart) just before video capture; the samples before the first video frame are dropped, so
 * audio and video start together (within about a frame). Needs Canon's live-view audio already on (Sound recording
 * Auto/Manual): ML never lets Canon allocate its audio area while the recorder holds the shoot memory. */
#ifndef RT_HOST                         /* host test: fakes in tests/rectest_host */
static int      (*const SoundDev_SetAudioInit)(int mode, int z, int sync, int fmt) = (void *)0xE05EDAF7;
static int      (*const SoundDev_SetRecParam)(int *p)                               = (void *)0xE05EE3F9;
static int      (*const SoundDev_SetMovieStart)(int mode)                           = (void *)0xE05EDB95;
static int      (*const SoundDev_SetInStart)(void)                                  = (void *)0xE05EDDF7;
static int      (*const SoundDev_SetMovieStop)(int mode)                            = (void *)0xE05EDD73;
static int      (*const SoundDev_SetAudioEnd)(int mode)                             = (void *)0xE05EDDAF;
static uint32_t (*const AprocCtrl_GetMovieAdr)(void)                                = (void *)0xE0293C4F;
static uint32_t (*const AprocCtrl_GetMovieSize)(void)                               = (void *)0xE0293C81;
static uint32_t (*const AprocCtrl_GetRecEndian)(void)                               = (void *)0xE0293DA9;
static int      (*const ACTRL_IsActiveState)(int mask, int lock)                    = (void *)0xE024D0FD;
#endif

static CONFIG_INT("r.rawrec.audio", rr_audio, 0);
/* v3.17 bisect (Debug > Silent shutter test > Bisect: bg tasks): 0 all on, 1 all off, 2 info-bar task off,
 * 3 LV-silent watcher + crop marker off. An "off" task only sleeps (stack and size unchanged). Saved. */
CONFIG_INT("r.bisect.bg", r_bisect_bg, 0);

#define RA_NBUF   4
#define RA_BLK    98304                 /* AUDF 24 + RA_PAYLOAD 96000 + NULL padding, a multiple of 4096 */
#define RA_QMAX   32
struct __attribute__((packed)) rt_audf { char type[4]; uint32_t size; uint64_t ts; uint32_t frame, space; };
struct ra_q { uint32_t adr1, size1, adr2, size2; };
static struct ra_q ra_q[RA_QMAX];
static volatile int ra_qhead, ra_qtail, ra_qover;
static uint8_t *ra_buf;                 /* RA_NBUF x RA_BLK, fio_malloc'd before the video chunks */
static volatile int ra_ready[RA_NBUF];  /* 0 free, else bytes ready to write */
static int ra_widx;                     /* writer side */
static volatile int ra_started, ra_start_cb, ra_stop_cb, ra_err_cb, ra_stop_req, ra_task_done, ra_task_alive;   /* ra_on, ra_part_count: above */
static volatile int ra_skip_d, ra_skip_logged;
static int ra_stuck;                    /* Canon never confirmed a stop: no audio until power-cycle */   /* first frame - audio start (us); logged by the main task */
static uint32_t ra_count, ra_samples, ra_skip, ra_lost, ra_t_in, ra_endian, ra_bytes_written;
static int ra_sync;
static int ra_p[19];

static void ra_queue(int *info)
{
    int h = ra_qhead, n = (h + 1) % RA_QMAX;
    if (!info) return;
    if (n == ra_qtail) { ra_qover++; return; }
    ra_q[h].adr1 = info[0x35]; ra_q[h].size1 = info[0x36]; ra_q[h].adr2 = info[0x37]; ra_q[h].size2 = info[0x38];
    ra_qhead = n;
}
static void ra_cb_start(int inst, int err) { ra_start_cb = 1; }
static void ra_cb_rec(int inst, int *info) { ra_queue(info); }
static void ra_cb_stop(int inst, int *info) { ra_queue(info); ra_stop_cb = 1; }
static void ra_cb_err(int inst, int code) { ra_err_cb++; }
static void ra_cb_pause(int inst, int *info) { }

static int ra_sync_code(void)
{
    if (mlv_fps_nom == 30000 && mlv_fps_den == 1001) return 4;
    if (mlv_fps_nom == 24000 && mlv_fps_den == 1001) return 7;
    if (mlv_fps_nom == 25000) return 5;
    if (mlv_fps_nom == 24000) return 6;
    if (mlv_fps_nom == 60000 && mlv_fps_den == 1001) return 1;
    if (mlv_fps_nom == 50000) return 2;
    return -1;
}

/* Audio goes into the MLV in fixed 0.5 s AUDF blocks (RA_PAYLOAD bytes each, the last one padded with silence):
 * mlv_dump places each AUDF at frameNumber x its own payload size, so a short block (the trimmed first chunk, the
 * stop chunk) left gaps or overwrote audio in its WAV. Canon's chunks (~0.5 s, size depends on the sync code) are
 * re-cut into these blocks. Timestamps: WAVI 1 us, AUDF n = 2 us + n x 0.5 s (non-zero: mlv_dump treats a WAVI with
 * timestamp 0 as missing, and AUDF must sort after WAVI). */
#define RA_PAYLOAD 96000                /* 0.5 s of 48 kHz 16-bit stereo */
static int ra_fill_idx, ra_fill_n;      /* block being filled, bytes in it */
static volatile uint32_t ra_fpk[2], ra_fclip_n[2];   /* v19.2: peak / full-scale samples over EVERY sample (ra_task) */

/* close the block being filled (pad with silence to RA_PAYLOAD), hand it to the writer */
static void ra_finish_block(void)
{
    if (!ra_fill_n) return;
    uint8_t *blk = ra_buf + ra_fill_idx * RA_BLK;
    if (ra_fill_n < RA_PAYLOAD) memset(blk + 24 + ra_fill_n, 0, RA_PAYLOAD - ra_fill_n);
    struct rt_audf *h = (struct rt_audf *)blk;
    memcpy(h->type, "AUDF", 4); h->size = 24 + RA_PAYLOAD; h->frame = ra_count; h->space = 0;
    h->ts = 2 + (uint64_t)ra_count * 500000;
    ra_count++;
    ra_samples += ra_fill_n / 4;            /* real samples (padding not counted) */
    uint32_t used = 24 + RA_PAYLOAD, total = (used + 16 + 4095) & ~4095;
    struct rt_blk *nb = (struct rt_blk *)(blk + used);
    memcpy(nb->type, "NULL", 4); nb->size = total - used; nb->ts = h->ts;
    ra_ready[ra_fill_idx] = total;
    ra_fill_idx = (ra_fill_idx + 1) % RA_NBUF;
    ra_fill_n = 0;
    give_semaphore(rt_wsem);
}

/* append one Canon chunk (skip applied, byte-swapped) to the AUDF blocks */
static void ra_consume(struct ra_q *q)
{
    uint32_t parts[2][2] = { { q->adr1, q->size1 }, { q->adr2, q->size2 } };
    for (int k = 0; k < 2; k++)
    {
        uint32_t a = parts[k][0], sz = parts[k][1] & ~3;
        if (!a || !sz) continue;
        if (ra_skip) { uint32_t s = MIN(ra_skip, sz); a += s; sz -= s; ra_skip -= s; }
        while (sz)
        {
            if (!ra_fill_n)                     /* new block: wait for the writer to free this buffer */
            {
                int waited = 0;
                while (ra_ready[ra_fill_idx] && waited < 3000) { msleep(10); waited += 10; }
                if (ra_ready[ra_fill_idx]) { ra_lost++; return; }   /* writer stuck: drop the rest of this chunk */
            }
            uint32_t take = MIN(sz, (uint32_t)(RA_PAYLOAD - ra_fill_n));
            const uint8_t *src = UNCACHEABLE(a);
            uint8_t *d = ra_buf + ra_fill_idx * RA_BLK + 24 + ra_fill_n;
            if (ra_endian == 1)
                for (uint32_t i = 0; i < take; i += 2) { d[i] = src[i + 1]; d[i + 1] = src[i]; }
            else
                memcpy(d, src, take);
            {   /* v19.2: meter peaks and clips from every sample (the live meter reads only 20 ms per 125 ms and
                 * missed claps on the camera); d is frame-aligned (skip, take and ra_fill_n are multiples of 4) */
                const int16_t *sm = (const int16_t *)d;
                uint32_t p0 = ra_fpk[0], p1 = ra_fpk[1], c0 = 0, c1 = 0;
                for (uint32_t i = 0; i < take / 2; i += 2)
                {
                    int l = sm[i], r = sm[i + 1];
                    if (l < 0) l = -l;
                    if (r < 0) r = -r;
                    if ((uint32_t)l > p0) p0 = l;
                    if ((uint32_t)r > p1) p1 = r;
                    if (l >= 32767) c0++;
                    if (r >= 32767) c1++;
                }
                ra_fpk[0] = p0; ra_fpk[1] = p1; ra_fclip_n[0] += c0; ra_fclip_n[1] += c1;
            }
            ra_fill_n += take; a += take; sz -= take;
            if (ra_fill_n == RA_PAYLOAD) ra_finish_block();
        }
    }
}

static void ra_task(void *unused)
{
    for (;;)
    {
        if (ra_qtail == ra_qhead) { if (ra_stop_req) break; msleep(20); continue; }
        if (!first_t) { if (ra_stop_req) break; msleep(10); continue; }   /* skip is known once the first frame is in */
        if (ra_skip == 0xFFFFFFFF)
        {
            int32_t d = (int32_t)(first_t - ra_t_in);
            ra_skip = d > 0 ? ((uint32_t)((uint64_t)d * 48000 / 1000000) * 4) : 0;
            ra_skip_d = d; ra_skip_logged = 1;    /* printed by the main task (rt_printf is not task-safe) */
        }
        ra_consume(&ra_q[ra_qtail]);
        ra_qtail = (ra_qtail + 1) % RA_QMAX;
    }
    if (first_t) ra_finish_block();          /* last partial block, padded with silence */
    ra_task_done = 1;
    ra_task_alive = 0;
}

/* ---- v19.1 audio meters while recording (Movie > RAW video > Audio meters) ----
 * Canon hands chunks over only every ~0.5 s, too slow for a meter, so ra_meter_task reads the newest 20 ms of
 * samples straight from Canon's MEM2 ring behind its write pointer (AprocCtrl_GetWriteAdr: MMIO read, polled
 * every 0.5 s by the camera-verified audio probe) ~8x/s, takes the peak per channel and draws two bars on ML's layer
 * under the REC status: -48..0 dBFS, green / yellow from -12 / red from -3, a peak-hold tick (1.5 s) and a CLIP box
 * per channel (lit for 2 s after a sample at full scale). Read-only; drawing uses bmp_fill only. The clip log gets
 * update count, peak levels, clip counts and the screen-refresh cost during the clip (A/B against meters off). */
#ifndef RT_HOST
static uint32_t (*const AprocCtrl_GetWriteAdr)(void)                                = (void *)0xE0293BBB;
#endif
extern uint32_t r_refresh_count, r_refresh_us_max, r_ximr_us_max;
static CONFIG_INT("r.rawrec.meter", rr_meter, 1);
static const uint16_t ra_db_tbl[49] = { 32767, 29205, 26029, 23198, 20675, 18427, 16423, 14637, 13045, 11627, 10362, 9235, 8231, 7336, 6538, 5827, 5193, 4629, 4125, 3677, 3277, 2920, 2603, 2320, 2068, 1843, 1642, 1464, 1305, 1163, 1036, 924, 823, 734, 654, 583, 519, 463, 413, 368, 328, 292, 260, 232, 207, 184, 164, 146, 130 };   /* 32768 x 10^(-k/20) */
static volatile int ra_meter_alive, ra_meter_stop;
static uint32_t ra_madr, ra_msize;
static uint32_t ra_m_updates, ra_m_bad, ra_m_clip[2], ra_m_maxpk[2], ra_m_ref0, ra_m_refus0, ra_m_ximr0;

/* v19.3: to the right of the REC status (riz), in the black band above the 16:9 movie image (rows 0-36 on the rear
 * screen; v19.2's bars below the status covered the image top and ML's top info line). The bars start after the
 * widest status text and run to the right edge; rr_meter_layout() computes x and the scale once (fonts loaded). */
static int RA_M_X = 300, RA_M_W = 384, RA_M_PX = 8, rr_m_layout_done;
#define RA_M_Y   10
#define RA_M_H   8
#define RA_M_GAP 2
#define RA_M_CLIP_W 20
#define RA_M_RIGHT 712
static void rr_meter_layout(void)
{
    if (rr_m_layout_done) return;
    int w = bmp_string_width(FONT(FONT_MED, COLOR_WHITE, COLOR_BLACK), " REC 00:00 2880x1216 14b ");
    int x = (8 + w + 10 + 3) & ~3;
    if (x < 200 || x > 420) x = 300;                 /* font not measurable: fixed fallback */
    int px = (RA_M_RIGHT - 4 - RA_M_CLIP_W - x) / 48;
    if (px < 4) px = 4;
    RA_M_X = x; RA_M_PX = px; RA_M_W = 48 * px;
    rr_m_layout_done = 1;
}

static int ra_db_of(uint32_t pk)        /* 0 .. 48 = dB below full scale, 49 = below -48 dB */
{
    for (int k = 0; k < 49; k++) if (pk >= ra_db_tbl[k]) return k;
    return 49;
}

/* peaks of the newest ~20 ms in Canon's ring; 0 if the write pointer looks wrong */
static int ra_meter_read(uint32_t *pl, uint32_t *pr)
{
    uint32_t w = AprocCtrl_GetWriteAdr();
    if (w < ra_madr || w >= ra_madr + ra_msize) { ra_m_bad++; return 0; }
    uint32_t off = (w - ra_madr) & ~3;
    uint32_t back = 256 + 960 * 4;        /* skip the last 256 B (may be in flight), then 960 frames = 20 ms */
    off = (off + ra_msize - back) % ra_msize;
    const uint8_t *ring = UNCACHEABLE(ra_madr);
    uint32_t L = 0, R = 0;
    for (int i = 0; i < 960; i++)
    {
        const uint8_t *p = ring + off;
        int16_t l, r;
        if (ra_endian == 1) { l = (int16_t)(p[0] << 8 | p[1]); r = (int16_t)(p[2] << 8 | p[3]); }
        else { l = (int16_t)(p[1] << 8 | p[0]); r = (int16_t)(p[3] << 8 | p[2]); }
        uint32_t al = l < 0 ? -(int)l : l, ar = r < 0 ? -(int)r : r;
        if (al > L) L = al;
        if (ar > R) R = ar;
        off += 4; if (off >= ra_msize) off = 0;
    }
    *pl = L; *pr = R;
    return 1;
}

static void ra_meter_bar(int ch, int db, int hold_db, int clip_lit)
{
    int y = RA_M_Y + ch * (RA_M_H + RA_M_GAP);
    int len = db >= 48 ? 0 : (48 - db) * RA_M_PX;      /* 0 .. RA_M_W px */
    int g = MIN(len, 36 * RA_M_PX), yl = MIN(len, 45 * RA_M_PX);   /* green to -12 dB, yellow to -3 dB, red above */
    if (g) bmp_fill(COLOR_GREEN1, RA_M_X, y, g, RA_M_H);
    if (yl > g) bmp_fill(COLOR_YELLOW, RA_M_X + g, y, yl - g, RA_M_H);
    if (len > yl) bmp_fill(COLOR_RED, RA_M_X + yl, y, len - yl, RA_M_H);
    if (len < RA_M_W) bmp_fill(COLOR_GRAY(20), RA_M_X + len, y, RA_M_W - len, RA_M_H);
    if (hold_db < 48)
    {
        int hx = RA_M_X + (48 - hold_db) * RA_M_PX - 3;
        if (hx < RA_M_X) hx = RA_M_X;
        bmp_fill(COLOR_WHITE, hx, y, 3, RA_M_H);
    }
    bmp_fill(clip_lit ? COLOR_RED : COLOR_GRAY(20), RA_M_X + RA_M_W + 4, y, RA_M_CLIP_W, RA_M_H);
}

static void ra_meter_task(void *unused)
{
    int hold[2] = { 49, 49 }, hold_ms[2] = { 0, 0 }, clip_ms[2] = { 0, 0 }, clip_on[2] = { 0, 0 }, drawn = 0;
    uint32_t last_clipn[2] = { ra_fclip_n[0], ra_fclip_n[1] };
    rr_meter_layout();
    ra_m_updates = ra_m_bad = 0; ra_m_clip[0] = ra_m_clip[1] = ra_m_maxpk[0] = ra_m_maxpk[1] = 0;
    ra_m_ref0 = r_refresh_count; ra_m_refus0 = r_refresh_us_max; ra_m_ximr0 = r_ximr_us_max;
    while (!ra_meter_stop)
    {
        msleep(125);
        if (ra_meter_stop) break;
        if (gui_menu_shown()) { drawn = 0; continue; }
        uint32_t pk[2];
        if (!ra_meter_read(&pk[0], &pk[1])) continue;
        int now = get_ms_clock();
        if (!drawn)
        {
            bmp_fill(COLOR_BLACK, RA_M_X - 4, RA_M_Y - 1, RA_M_W + RA_M_CLIP_W + 12, 2 * RA_M_H + RA_M_GAP + 2);
            drawn = 1;
        }
        uint32_t fp[2] = { ra_fpk[0], ra_fpk[1] };      /* every sample since the last update (ra_task, <= 0.5 s late) */
        ra_fpk[0] = ra_fpk[1] = 0;
        for (int c = 0; c < 2; c++)
        {
            int db = ra_db_of(pk[c]);                     /* bar: live 20 ms */
            uint32_t eff = MAX(pk[c], fp[c]);             /* hold / peak / clip: nothing missed */
            int edb = ra_db_of(eff);
            uint32_t cn = ra_fclip_n[c];
            int new_clip = cn != last_clipn[c] || pk[c] >= 32767;
            last_clipn[c] = cn;
            if (eff > ra_m_maxpk[c]) ra_m_maxpk[c] = eff;
            if (clip_on[c] && now - clip_ms[c] >= 2000) clip_on[c] = 0;
            if (new_clip) { if (!clip_on[c]) ra_m_clip[c]++; clip_on[c] = 1; clip_ms[c] = now; }   /* flags, not time 0: robust to clock wrap */
            if (edb <= hold[c] || now - hold_ms[c] > 1500) { hold[c] = edb; hold_ms[c] = now; }
            ra_meter_bar(c, db, hold[c], clip_on[c]);
        }
        ra_m_updates++;
    }
    if (drawn && !gui_menu_shown())
        bmp_fill(COLOR_EMPTY, RA_M_X - 4, RA_M_Y - 1, RA_M_W + RA_M_CLIP_W + 12, 2 * RA_M_H + RA_M_GAP + 2);
    ra_meter_alive = 0;
}

static void ra_meter_start(uint32_t madr, uint32_t msize)
{
    ra_meter_stop = 0;
    if (!rr_meter || !rr_active || ra_meter_alive) return;    /* REC-button clips only (timed tests: no overlay) */
    ra_madr = madr; ra_msize = msize;
    ra_meter_alive = 1;
    task_create("ra_meter", 0x1f, 0x1000, ra_meter_task, 0);
}

static void ra_meter_end(void)
{
    if (!ra_meter_alive) return;
    ra_meter_stop = 1;
    for (int t = 0; t < 1000 && ra_meter_alive; t += 10) msleep(10);
    rt_printf("meters: %d updates (%d bad write pointer), peak L/R -%d/-%d dBFS, clip events L/R %d/%d (full-scale samples %d/%d); "
              "screen refreshes %d, refresh max %d us, Ximr max %d us\n",
              (int)ra_m_updates, (int)ra_m_bad, ra_db_of(ra_m_maxpk[0]), ra_db_of(ra_m_maxpk[1]),
              (int)ra_m_clip[0], (int)ra_m_clip[1], (int)ra_fclip_n[0], (int)ra_fclip_n[1], (int)(r_refresh_count - ra_m_ref0), (int)r_refresh_us_max, (int)r_ximr_us_max);
}

/* writer side: one prepared block, if any; returns bytes written, < 0 on error */
static int ra_write_one(FILE *f)
{
    int n = ra_ready[ra_widx];
    if (!n) return 0;
    int r = FIO_WriteFile(f, ra_buf + ra_widx * RA_BLK, n);
    if (r != n) return r ? r : -1;
    ra_ready[ra_widx] = 0;
    ra_widx = (ra_widx + 1) % RA_NBUF;
    ra_part_count++;
    ra_bytes_written += n;
    return n;
}

/* start Canon's audio (before the video capture). 0 = running, else why not */
static int ra_start(void)
{
    ra_on = ra_started = ra_start_cb = ra_stop_cb = ra_err_cb = ra_stop_req = ra_task_done = 0;
    ra_qhead = ra_qtail = ra_qover = 0; ra_widx = 0; ra_count = ra_samples = ra_lost = ra_bytes_written = 0;
    ra_part_count = 0; ra_skip = 0xFFFFFFFF; ra_skip_d = 0; ra_skip_logged = 0;
    ra_fill_idx = 0; ra_fill_n = 0;
    ra_fpk[0] = ra_fpk[1] = 0; ra_fclip_n[0] = ra_fclip_n[1] = 0;
    first_t = 0;                        /* ra_task waits for this clip's first frame, not the last clip's */
    for (int i = 0; i < RA_NBUF; i++) ra_ready[i] = 0;
    if (!ra_buf) return 1;
    if (ra_stuck) { rt_printf("audio: off until power-cycle (Canon didn't confirm the last audio stop), no audio\n"); return 7; }
    ra_sync = ra_sync_code();
    if (ra_sync < 0) { rt_printf("audio: no sync code for %d/%d fps, no audio\n", mlv_fps_nom, mlv_fps_den); return 2; }
    uint32_t madr = AprocCtrl_GetMovieAdr(), msize = AprocCtrl_GetMovieSize();
    if (!madr || !msize) { rt_printf("audio: Canon's audio is off (Sound recording disabled?), no audio\n"); return 3; }
    if (ACTRL_IsActiveState(0x11, 0)) { rt_printf("audio: Canon audio busy (playback), no audio\n"); return 4; }
    int rc = SoundDev_SetAudioInit(1, 0, ra_sync, 1);
    if (rc) { rt_printf("audio: SetAudioInit = %d, no audio\n", rc); return 5; }
    ra_endian = AprocCtrl_GetRecEndian();
    memset(ra_p, 0, sizeof(ra_p));
    ra_p[0] = 48000; ra_p[1] = 16; ra_p[2] = 2; ra_p[5] = 1; ra_p[8] = ra_sync;
    ra_p[9] = (int)ra_cb_start; ra_p[10] = (int)ra_cb_rec; ra_p[11] = (int)ra_cb_stop; ra_p[13] = (int)ra_cb_err;
    ra_p[15] = (int)ra_cb_pause;
    rc = SoundDev_SetRecParam(ra_p);
    if (!rc) rc = SoundDev_SetMovieStart(2) ? 100 : 0;
    if (rc) { rt_printf("audio: SetRecParam/SetMovieStart failed (%d), no audio\n", rc); SoundDev_SetAudioEnd(4); return 6; }
    ra_started = 1;
    for (int t = 0; t < 1000 && !ra_start_cb; t += 10) msleep(10);
    SoundDev_SetInStart();
    ra_t_in = rt_us();
    ra_task_alive = 1;
    task_create("ra_task", 0x1d, 0x1000, ra_task, 0);
    ra_on = 1;
    rt_printf("audio: on, sync %d, endian %d, ring %08x+%x, StartCBR %d\n", ra_sync, (int)ra_endian, (int)madr, (int)msize, ra_start_cb);
    ra_meter_start(madr, msize);
    return 0;
}

/* stop Canon's audio and let ra_task finish preparing; the writer still has to write the rest */
static void ra_stop(void)
{
    if (!ra_started) return;
    ra_meter_end();                     /* v19.1: before Canon's audio stops */
    SoundDev_SetMovieStop(3);
    for (int t = 0; t < 3000 && !ra_stop_cb; t += 10) msleep(10);
    ra_stop_req = 1;
    for (int t = 0; t < 5000 && ra_on && !ra_task_done; t += 10) msleep(10);
    if (ra_stop_cb) SoundDev_SetAudioEnd(4);
    else { ra_stuck = 1; rt_printf("audio: no stop confirmation from Canon: audio left on; power-cycle before the next clip\n"); }
    if (ra_skip_logged) rt_printf("audio: first frame %d us after audio start, %d bytes skipped\n", ra_skip_d, (int)(((uint32_t)(ra_skip_d > 0 ? ra_skip_d : 0) * 48ULL / 1000) * 4));
    if (ra_task_alive) rt_printf("audio: ra_task still running after stop (audio blocks kept allocated)\n");
    rt_printf("audio: %d blocks, %d.%03d s, %d chunks lost (queue %d), err CBR %d\n", (int)ra_count,
              (int)(ra_samples / 48000), (int)(ra_samples % 48000 / 48), (int)ra_lost, (int)ra_qover, ra_err_cb);
    ra_started = 0;
}

static volatile int w_stop, w_done, w_err;
static volatile uint64_t w_bytes;      /* > 4 GB in a 40 s run */
static int w_part, w_n;
static uint64_t w_us_sum;               /* all writes (wl[] keeps only the first RT_MAXW for the log) */
static uint32_t w_us_min, w_us_max;
static uint32_t w_part_frames;
static FILE *w_file;
static char w_name[32];
static uint32_t w_fsize;
struct rt_wrec { uint32_t t_ms, us, bytes; };
static struct rt_wrec *wl;
static uint32_t rt_t0;

static void rt_part_name(char *out, int run, int part)
{
    if (rr_active && c_mlv)             /* v18: REC-button clip, mlv_lite-style names at the card root */
    { if (part) snprintf(out, 32, "%s.M%02d", rr_base, part - 1); else snprintf(out, 32, "%s.MLV", rr_base); return; }
    if (c_mlv) { if (part) snprintf(out, 32, "ML/RT%02d.M%02d", run, part - 1); else snprintf(out, 32, "ML/RT%02d.MLV", run); }
    else if (part) snprintf(out, 32, "ML/RT%02d.B%02d", run, part);
    else snprintf(out, 32, "ML/RT%02d.BIN", run);
}

/* v18: clip name M<day>-<hhmm> (8.3, like mlv_lite); a second clip in the same minute gets A..Z for the dash */
static void rr_pick_name(void)
{
    struct tm now;
    memset(&now, 0, sizeof(now));
    LoadCalendarFromRTC(&now);
    static const char sep[] = "-ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    for (int i = 0; sep[i]; i++)
    {
        char n[24];
        /* no %c: Canon's vsnprintf (the one ML uses) doesn't support it; v18.1 named a clip "M29------------" */
        snprintf(rr_base, sizeof(rr_base), "M%02d-%02d%02d", now.tm_mday % 100, now.tm_hour % 100, now.tm_min % 100);
        rr_base[3] = sep[i];
        snprintf(n, sizeof(n), "%s.MLV", rr_base);
        if (!is_file(n)) return;
    }
}

/* v18: stop before the card is full: free space at the start minus what's written, the buffer still to
 * be written and 64 MB spare. A write that runs out of space would leave a cut-off block at the file end. */
static uint64_t c_free0;
static int rt_card_low(void)
{
    uint64_t reserve = (uint64_t)nch * kslots * slot_bytes + (64 << 20) + (uint64_t)(w_part + 2) * RT_HDR_BYTES;
    return c_free0 && w_bytes + reserve > c_free0;
}

static void rt_writer(void *unused)
{
    int wi = 0;
    for (;;)
    {
        if (ra_on && ra_ready[ra_widx])        /* v19: prepared audio block first (small, keeps the A/V order) */
        {
            int r = ra_write_one(w_file);
            if (r < 0) { w_err = r; break; }
            w_fsize += r; w_bytes += r;
            continue;
        }
        if (ch[wi].state == CH_FULL)
        {
            uint32_t bytes = ch[wi].n * slot_bytes;
            if (w_fsize + bytes > RT_SPLIT)
            {
                if (c_mlv) rt_mlv_close(w_file, w_part, w_part_frames);
                else FIO_CloseFile(w_file);
                ra_part_count = 0;
                rt_part_name(w_name, rt_run_id, ++w_part);
                w_file = FIO_CreateFile(w_name);
                w_fsize = 0; w_part_frames = 0;
                if (!w_file) { w_err = -2; break; }
                if (c_mlv)
                {
                    rt_mlv_build_hdr(w_part, 0);
                    if (FIO_WriteFile(w_file, mlv_hdr, RT_HDR_BYTES) != RT_HDR_BYTES) { w_err = -3; break; }
                    w_fsize = RT_HDR_BYTES;
                }
            }
            uint32_t t = rt_us();
            int r = FIO_WriteFile(w_file, (void *)ch[wi].base, bytes);
            t = rt_us() - t;
            if (w_n < RT_MAXW) { wl[w_n].t_ms = (rt_us() - rt_t0) / 1000; wl[w_n].us = t; wl[w_n].bytes = bytes; }
            w_n++;
            w_us_sum += t; if (t < w_us_min) w_us_min = t; if (t > w_us_max) w_us_max = t;
            if (r != (int)bytes) { w_err = r ? r : -1; break; }
            w_fsize += bytes;
            w_bytes += bytes;
            w_part_frames += ch[wi].n;
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
static uint32_t c_src, c_est;
static int nfr, frames, drops, dups, missed, late, tears, topchg, botchg, ntl, irq_timeouts;
static int cur, next_fill, first_drop_ms, c_abort;
static uint32_t next_tl, max_used, dmax, dsum, last_rec_bot, t_last_ev;   /* first_t: declared above (v19) */
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

/* ---- v20: LV silent pictures -> one DNG per kept frame ----
 * Slot = 4096-byte DNG header + frame (pack unit in ML order: 16-bit LE words, MSB-first bits). DNG wants the bits
 * MSB-first in byte order, so the writer swaps the bytes of every 16-bit word in place (cached reads after a line
 * invalidate, uncached writes: no dirty lines can land on later EDMAC data), then writes header + frame in one
 * 4 KiB-aligned write. 64 sampled words per frame are checked against the swap of their old value. */
#define SP_HDR       4096
#define SP_SWAP_BLK  16384
#define SP_NCHK      64
#define SP_MAXREC    400
#define SP_SLOW_US   40000              /* v3.10 survey limit; v3.11: mid-frame copies tolerate the 0x3Axxxxxx chunks (~31-39 ms), so only
                                         * pathological ones are dropped; v3.10 camera: 0x49-0x4D 23-26.5 ms, 0x7A-0x7E 30.7-38.9 ms */
static int sp_nfast;
static char sp_dir[16];
static uint8_t *sp_hdr;                 /* SP_HDR, malloc'd per session (cached; copied into the slot) */
static struct tm sp_tm;                 /* RTC at the press */
static volatile uint32_t sp_press_us;   /* rt_us() at the press (key handler) */
static int sp_run_mode, sp_ring_n, sp_ring_max, sp_trig_cap;
static volatile uint32_t sp_swap_us_sum, sp_swap_us_max, sp_wr_us_sum, sp_wr_us_max, sp_chk_bad, sp_torn;
struct sp_rec { uint32_t seq, t_us, swap_us, wr_us; int fl, chunk; };
static struct sp_rec *sp_recs;          /* per written DNG (main task logs them; the writer never calls rt_printf) */

static inline uint32_t sp_sw16(uint32_t v)
{
#ifndef RT_HOST
    uint32_t r;
    asm ("rev16 %0, %1" : "=r" (r) : "r" (v));
    return r;
#else
    return ((v & 0x00FF00FFu) << 8) | ((v >> 8) & 0x00FF00FFu);
#endif
}
static int sp_pick_dir(void);

/* invalidate L1 D-cache lines of [p, p+n) (DCIMVAC; same as zebra.c lv_dcache_invalidate, camera-proven) */
static void sp_dcache_inval(uint32_t p, uint32_t n)
{
#ifndef RT_HOST
    uint32_t ctr;
    asm volatile ("mrc p15, 0, %0, c0, c0, 1" : "=r" (ctr));
    uint32_t line = 4 << ((ctr >> 16) & 0xF);
    if (line < 16) line = 16;
    asm volatile ("dsb sy");
    for (uint32_t a = p & ~(line - 1); a < p + n; a += line)
        asm volatile ("mcr p15, 0, %0, c7, c6, 1" : : "r" (a));
    asm volatile ("dsb sy");
#endif
}

/* swap the bytes of every 16-bit word of [unc, unc+n) in place; returns self-check mismatches */
static uint32_t sp_swap(uint32_t unc, uint32_t n)
{
    uint32_t step = (n / SP_NCHK) & ~3u, orig[SP_NCHK];
    for (int j = 0; j < SP_NCHK; j++) orig[j] = MEM(unc + j * step);
    uint32_t cac = unc & ~RT_UNC;
    for (uint32_t off = 0; off < n; off += SP_SWAP_BLK)
    {
        uint32_t len = MIN(SP_SWAP_BLK, n - off);
        sp_dcache_inval(cac + off, len);
        const uint32_t *s = (const uint32_t *)(cac + off);
        uint32_t *d = (uint32_t *)(unc + off);
        for (uint32_t i = 0; i < len / 4; i++) d[i] = sp_sw16(s[i]);
    }
    RT_DMB();
    uint32_t bad = 0;
    for (int j = 0; j < SP_NCHK; j++) if (MEM(unc + j * step) != sp_sw16(orig[j])) bad++;
    return bad;
}

/* ---- minimal DNG 1.4 header (little-endian TIFF, one raw CFA IFD, strip at SP_HDR) ---- */
struct sp_tag { uint16_t tag, type; uint32_t count; const void *data; uint32_t v; };
#define SP_MAXTAG 32
static struct sp_tag sp_tags[SP_MAXTAG];
static int sp_ntag;
static void sp_add(uint16_t tag, uint16_t type, uint32_t count, const void *data, uint32_t v)
{
    if (sp_ntag >= SP_MAXTAG) return;
    struct sp_tag *t = &sp_tags[sp_ntag++];
    t->tag = tag; t->type = type; t->count = count; t->data = data; t->v = v;
}
static int sp_type_size(int type) { return type == 3 ? 2 : type == 4 ? 4 : type == 5 || type == 10 ? 8 : 1; }

/* returns the header length (<= SP_HDR) or -1 */
static int sp_dng_header(uint8_t *h, const char *desc, const char *dt)
{
    static const int32_t cm[18] = { 6446, 10000, -366, 10000, -864, 10000, -4436, 10000, 12204, 10000, 2513, 10000,
                                    -952, 10000, 2496, 10000, 6348, 10000 };   /* Adobe EOS R D65 (as the MLV RAWI) */
    static const uint16_t cfa_dim[2] = { 2, 2 };
    static const uint8_t cfa[4] = { 0, 1, 1, 2 }, dngv[4] = { 1, 4, 0, 0 };
    uint32_t obl = 0, obt = 0;
    if (m1_w == 2304 && m1_h == 1290) { obl = RT_OB_LEFT; obt = RT_OB_TOP; }
    if (m1_w == 4256 && m1_h == 2214) { obl = 144; obt = 46; }
    uint32_t ax = c_x0 < obl ? obl - c_x0 : 0, ay = c_y0 < obt ? obt - c_y0 : 0;
    uint32_t area[4] = { ay, ax, c_ch, c_cw }, corg[2] = { 0, 0 }, csz[2] = { c_cw - ax, c_ch - ay };
    int lsh = 14 - c_bpp;
    uint32_t black = (RT_BLACK14 + (lsh ? 1 << (lsh - 1) : 0)) >> lsh, white = RT_WHITE14 >> lsh;
    sp_ntag = 0;
    sp_add(0x00FE, 4, 1, 0, 0);                          /* NewSubFileType: main image */
    sp_add(0x0100, 4, 1, 0, c_cw);
    sp_add(0x0101, 4, 1, 0, c_ch);
    sp_add(0x0102, 3, 1, 0, c_bpp);
    sp_add(0x0103, 3, 1, 0, 1);                          /* uncompressed */
    sp_add(0x0106, 3, 1, 0, 32803);                      /* CFA */
    sp_add(0x010E, 2, strlen(desc) + 1, desc, 0);
    sp_add(0x010F, 2, 6, "Canon", 0);
    sp_add(0x0110, 2, 12, "Canon EOS R", 0);
    sp_add(0x0111, 4, 1, 0, SP_HDR);                     /* StripOffsets */
    sp_add(0x0112, 3, 1, 0, 1);
    sp_add(0x0115, 3, 1, 0, 1);
    sp_add(0x0116, 4, 1, 0, c_ch);
    sp_add(0x0117, 4, 1, 0, frame_bytes);
    sp_add(0x011C, 3, 1, 0, 1);
    sp_add(0x0131, 2, 30, "Magic Lantern LV silent (riz)", 0);
    sp_add(0x0132, 2, 20, dt, 0);
    sp_add(0x828D, 3, 2, cfa_dim, 0);
    sp_add(0x828E, 1, 4, cfa, 0);                        /* RGGB */
    sp_add(0xC612, 1, 4, dngv, 0);
    sp_add(0xC614, 2, 12, "Canon EOS R", 0);
    sp_add(0xC61A, 4, 1, 0, black);
    sp_add(0xC61D, 4, 1, 0, white);
    sp_add(0xC61F, 4, 2, corg, 0);
    sp_add(0xC620, 4, 2, csz, 0);
    sp_add(0xC621, 10, 9, cm, 0);
    sp_add(0xC65A, 3, 1, 0, 21);                         /* D65 */
    sp_add(0xC68D, 4, 4, area, 0);                       /* ActiveArea: top, left, bottom, right */
    memset(h, 0, SP_HDR);
    h[0] = 'I'; h[1] = 'I'; h[2] = 42; h[4] = 8;
    uint32_t ifd = 8, x = ifd + 2 + sp_ntag * 12 + 4;    /* x: extra data */
    h[ifd] = sp_ntag; h[ifd + 1] = sp_ntag >> 8;
    for (int i = 0; i < sp_ntag; i++)
    {
        struct sp_tag *t = &sp_tags[i];
        uint8_t *e = h + ifd + 2 + i * 12;
        uint32_t sz = t->count * sp_type_size(t->type);
        memcpy(e, &t->tag, 2); memcpy(e + 2, &t->type, 2); memcpy(e + 4, &t->count, 4);
        if (!t->data) { if (t->type == 3) { uint16_t s = t->v; memcpy(e + 8, &s, 2); } else memcpy(e + 8, &t->v, 4); }
        else if (sz <= 4) memcpy(e + 8, t->data, sz);
        else
        {
            if (x + sz > SP_HDR) return -1;
            memcpy(h + x, t->data, sz); memcpy(e + 8, &x, 4);
            x = (x + sz + 1) & ~1u;
        }
    }
    return x;
}

/* writer: oldest kept frame first (chunks are a ring in pre-capture mode, so not in index order) */
static int sp_write_dng(int k)
{
    struct rt_chunk *c = &ch[k];
    char name[40], desc[96], dt[24];
    if (!sp_dir[0] && sp_pick_dir()) { sp_dir[0] = 0; return -5; }
    int32_t ms = (int32_t)(c->t_us - sp_press_us) / 1000;
    snprintf(name, sizeof(name), "%s/F%04d.DNG", sp_dir, sp_saved);
    snprintf(desc, sizeof(desc), "LV silent picture F%04d, %s%d ms from the press, %d-bit%s", sp_saved, ms < 0 ? "-" : "+",
             ms < 0 ? -ms : ms, c_bpp, (c->fl & F_TORN) ? ", may be torn" : "");
    snprintf(dt, sizeof(dt), "%04d:%02d:%02d %02d:%02d:%02d", sp_tm.tm_year + 1900, sp_tm.tm_mon + 1, sp_tm.tm_mday,
             sp_tm.tm_hour, sp_tm.tm_min, sp_tm.tm_sec);
    if (sp_dng_header(sp_hdr, desc, dt) < 0) return -4;
    memcpy((void *)c->base, sp_hdr, SP_HDR);
    uint32_t t0 = rt_us();
    uint32_t bad = sp_swap(c->base + SP_HDR, frame_bytes);
    uint32_t t1 = rt_us();
    FILE *f = FIO_CreateFile(name);
    if (!f) return -2;
    int r = FIO_WriteFile(f, (void *)c->base, SP_HDR + frame_bytes);
    FIO_CloseFile(f);
    uint32_t t2 = rt_us();
    if (r != (int)(SP_HDR + frame_bytes)) return r < 0 ? r : -1;
    sp_chk_bad += bad;
    if (c->fl & F_TORN) sp_torn++;
    sp_swap_us_sum += t1 - t0; if (t1 - t0 > sp_swap_us_max) sp_swap_us_max = t1 - t0;
    sp_wr_us_sum += t2 - t1; if (t2 - t1 > sp_wr_us_max) sp_wr_us_max = t2 - t1;
    if (sp_recs && sp_saved < SP_MAXREC)
    {
        struct sp_rec *q = &sp_recs[sp_saved];
        q->seq = c->seq; q->t_us = c->t_us; q->swap_us = t1 - t0; q->wr_us = t2 - t1; q->fl = c->fl | (bad ? 0x100 : 0); q->chunk = k;
    }
    w_bytes += SP_HDR + frame_bytes; w_n++; w_us_sum += t2 - t1;
    if (t2 - t1 < w_us_min) w_us_min = t2 - t1;
    if (t2 - t1 > w_us_max) w_us_max = t2 - t1;
    sp_saved++;
    return 0;
}

static void sp_writer(void *unused)
{
    for (;;)
    {
        int k = -1;
        uint32_t best = 0xFFFFFFFF;
        for (int i = 0; i < nch; i++) if (ch[i].state == CH_FULL && ch[i].seq < best) { best = ch[i].seq; k = i; }
        if (k >= 0 && sp_ready && !w_stop) k = -1;     /* v3.8: no swap/card writes while frames are copied: the
                                                         * 4K camera test showed copies 31 -> 43 ms with them */
        if (k >= 0)
        {
            int r = sp_write_dng(k);
            if (r) { w_err = r; break; }
            ch[k].state = CH_FREE;
            continue;
        }
        if (w_stop) break;
        take_semaphore(rt_wsem, 50);
    }
    w_done = 1;
}

/* burst folder S<day><hh><mm> at the card root, A..Z appended on a clash (8.3) */
static int sp_pick_dir(void)
{
    struct tm now;
    memset(&now, 0, sizeof(now));
    LoadCalendarFromRTC(&now);
    static const char sfx[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    for (int i = 0; sfx[i]; i++)
    {
        snprintf(sp_dir, sizeof(sp_dir), "S%02d%02d%02d ", now.tm_mday % 100, now.tm_hour % 100, now.tm_min % 100);
        sp_dir[7] = sfx[i];
        if (!i) sp_dir[7] = 0;
        if (!is_dir(sp_dir)) { FIO_CreateDirectory(sp_dir); return is_dir(sp_dir) ? 0 : -1; }
    }
    return -1;
}

/* same sampling as rt_probe, on a copied frame (full-frame copy: same pitch as MEM1) */
#define SP_NPROBE 64                    /* v3.9: camera tears were as narrow as 31 rows */
static uint32_t sp_midtear;
static uint32_t sp_rowhash(uint32_t base, uint32_t row)
{
    uint32_t h = 0x811C9DC5;
    for (int s = 1; s <= 3; s++)
    {
        const volatile uint32_t *p = (const volatile uint32_t *)(base + row * pitch + ((pitch * s / 4) & ~3));
        for (int i = 0; i < 4; i++) h = (h ^ p[i]) * 0x01000193;
    }
    return h;
}

/* one LV frame event in a silent-picture session (capture task) */
static int sp_frame(uint32_t td, uint32_t lat_us, int flags, uint32_t top_ref, uint32_t bot_ref)
{
    if (!first_t) { first_t = td; next_tl = 0; }
    uint32_t us = 0;
    sp_check_tick_end();
    int trig = sp_trig;
    if (trig && !sp_trig_cap)           /* press: the ring frames become kept frames */
    {
        sp_trig_cap = 1;
        r_refresh_hold = 1;             /* v3.9: no display refresh until capture ends */
        sp_ref0 = r_refresh_count;
        for (int i = 0; i < nch; i++) if (ch[i].state == CH_RING) { ch[i].state = CH_FULL; sp_pre_kept++; sp_kept++; }
        sp_ring_n = 0;
        give_semaphore(rt_wsem);
    }
    int want = trig || (sp_run_mode == 2 && !sp_trig_cap);
    if (!want) goto out;
    if (frames && bot_ref == last_rec_bot) { flags |= F_DUP; dups++; goto out; }
    int k = -1;
    for (int i = 0; i < nch; i++) if (ch[i].state == CH_FREE) { k = i; break; }
    if (!trig && (k < 0 || sp_ring_n >= sp_ring_max))     /* ring full: reuse the oldest ring frame */
    {
        int o = -1;
        uint32_t best = 0xFFFFFFFF;
        for (int i = 0; i < nch; i++) if (ch[i].state == CH_RING && ch[i].seq < best) { best = ch[i].seq; o = i; }
        if (o >= 0) { k = o; sp_ring_n--; }
    }
    if (k < 0)
    {
        flags |= F_DROP; drops++;
        if (first_drop_ms < 0) first_drop_ms = (td - first_t) / 1000;
        if (trig) sp_end = 1;           /* v3.8: memory full: stop and start writing (no writes during capture) */
        goto out;
    }
    ch[k].state = CH_FILL;
    uint32_t top0 = rt_probe(r_top);
    if (top0 != top_ref) { flags |= F_TEAR; tears++; }
    /* v20: 32 rows between the probes, hashed in MEM1 just before the copy and in the copy after it: a copy that fell
     * behind the next frame mid-way (only the top/bottom probes are checked otherwise) shows as a mismatch */
    uint32_t ph[SP_NPROBE], prow[SP_NPROBE], dst = ch[k].base + slot_pre;
    for (int j = 0; j < SP_NPROBE; j++)
    {
        prow[j] = r_top + 1 + (uint32_t)j * (r_bot - r_top - 2) / (SP_NPROBE - 1);
        ph[j] = rt_probe(prow[j]);
    }
    if (rt_copy(c_src, pitch, dst, c_rb, c_ch, &us))
    {
        rt_printf("!! EDMAC timeout at frame %d, stopping\n", nfr);
        ch[k].state = CH_FREE;
        rt_fail();
        return -1;
    }
    if (rt_probe(r_top) != top0) { flags |= F_TOPCHG; topchg++; }
    if (rt_probe(r_bot) != bot_ref) { flags |= F_BOTCHG; botchg++; }
    {
        int mb = 0;
        for (int j = 0; j < SP_NPROBE; j++) if (sp_rowhash(dst, prow[j]) != ph[j]) mb++;
        /* rows below the bottom probe may still be written at the frame event: compare them with MEM1 after the
         * copy instead (the next frame reaches them only ~30 ms after the event) */
        uint32_t h1 = c_ch - 1;
        for (int j = 0; j < 6 && r_bot + 1 < h1; j++)
        {
            uint32_t row = r_bot + 1 + (uint32_t)j * (h1 - r_bot - 1) / 5;
            if (sp_rowhash(dst, row) != rt_probe(row)) mb++;
        }
        if (mb) { flags |= F_MIDTEAR; sp_midtear++; }
    }
    frames++; dsum += us; if (us > dmax) dmax = us;
    last_rec_bot = bot_ref;
    ch[k].seq = nfr; ch[k].t_us = td; ch[k].fl = flags;
    if (trig) { sp_kept++; ch[k].state = CH_FULL; give_semaphore(rt_wsem); }
    else { ch[k].state = CH_RING; sp_ring_n++; }
out:
    if (sp_rel && sp_kept) { sp_trig = 0; sp_end = 1; }  /* released: done once something is kept */
    if (flags & F_LATE) late++;
    if (nfr < RT_MAXFR)
    {
        fl[nfr].t_us = td - first_t; fl[nfr].dma_us = MIN(us, 65535);
        fl[nfr].lat = MIN(lat_us / 100, 255); fl[nfr].flags = flags;
    }
    nfr++;
    int used = 0;
    for (int i = 0; i < nch; i++) if (ch[i].state != CH_FREE && ch[i].state != CH_SKIP) used++;
    if ((uint32_t)used > max_used) max_used = used;
    return 0;
}

/* ---- v3.11: mid-frame copy ----
 * v3.8-v3.10 camera: copies started at the frame-end IRQ had ~1.8 ms head start on the next frame, and EDMAC stalls
 * of a few ms mid-copy let the sensor overtake it (tears in 40% of frames, 25-37 ms copies). Now the copy of frame
 * n+1 starts ~SP_MID_OFS after that frame begins: the sensor is then ~1250 rows ahead (a copy at full EDMAC speed,
 * ~8 us/row, can't catch it before the end) and the next frame starts writing only ~15 ms later (a 37 ms copy at
 * 16.7 us/row stays ahead of it). Check: SP_NIH rows hashed by the ISR at frame n+1's end vs the copy, plus the rows
 * below the bottom probe vs MEM1 after the copy. A copy that would start > 1 ms late is skipped (frame dropped). */
#define SP_MID_OFS   18000
#define F_UNVER      128                /* no IRQ hash set for this frame: unverified */
static struct semaphore *sp_tsem;
static uint32_t sp_mid_delay, sp_late, sp_unver, irq_t_of_n;
static int sp_mid_on;
static void sp_mid_wake(int last_expiry, void *p) { give_semaphore(sp_tsem); }

static int sp_frame_mid(uint32_t td_irq, uint32_t lat_us, uint32_t n)
{
    if (!first_t) { first_t = td_irq; next_tl = 0; }
    uint32_t us = 0;
    int flags = 0;
    sp_check_tick_end();
    int trig = sp_trig;
    if (trig && !sp_trig_cap)
    {
        sp_trig_cap = 1;
        r_refresh_hold = 1;
        sp_ref0 = r_refresh_count;
        for (int i = 0; i < nch; i++) if (ch[i].state == CH_RING) { ch[i].state = CH_FULL; sp_pre_kept++; sp_kept++; }
        sp_ring_n = 0;
        give_semaphore(rt_wsem);
    }
    int want = trig || (sp_run_mode == 2 && !sp_trig_cap);
    if (!want) goto out;
    /* too late for SP_MID_OFS into frame n+1 (the last copy ran long): skip it, before touching any chunk */
    uint32_t el = d20(RT_DMA_TIMER(), irq_t_of_n);
    int32_t wait = (int32_t)sp_mid_delay - (int32_t)el;
    if (wait < -1000) { sp_late++; flags |= F_LATE; goto out; }
    int k = -1;
    for (int i = 0; i < nch; i++) if (ch[i].state == CH_FREE) { k = i; break; }
    if (!trig && (k < 0 || sp_ring_n >= sp_ring_max))
    {
        int o = -1;
        uint32_t best = 0xFFFFFFFF;
        for (int i = 0; i < nch; i++) if (ch[i].state == CH_RING && ch[i].seq < best) { best = ch[i].seq; o = i; }
        if (o >= 0) { k = o; sp_ring_n--; }
    }
    if (k < 0)
    {
        flags |= F_DROP; drops++;
        if (first_drop_ms < 0) first_drop_ms = (td_irq - first_t) / 1000;
        if (trig) sp_end = 1;
        goto out;
    }
    /* wait until SP_MID_OFS into frame n+1 */
    if (wait > 300)
    {
        if (SetHPTimerAfterNow(wait, sp_mid_wake, sp_mid_wake, 0) & 1) msleep(MAX(wait / 1000, 1));
        else take_semaphore(sp_tsem, 100);
    }
    uint32_t td = rt_us();
    ch[k].state = CH_FILL;
    uint32_t dst = ch[k].base + slot_pre;
    if (rt_copy(c_src, pitch, dst, c_rb, c_ch, &us))
    {
        rt_printf("!! EDMAC timeout at frame %d, stopping\n", nfr);
        ch[k].state = CH_FREE;
        rt_fail();
        return -1;
    }
    /* verify against the ISR's hashes of frame n+1 (its IRQ comes ~13 ms into a 25 ms copy) */
    {
        for (int i = 0; i < 60 && irq_n < n + 1; i++) msleep(1);
        int mb = 0, set = (n + 1) & 1;
        if (sp_ih_n[set] != n + 1) { flags |= F_UNVER; sp_unver++; }
        else
        {
            uint32_t h[SP_NIH];
            for (int j = 0; j < SP_NIH; j++) h[j] = sp_ih[set][j];
            RT_DMB();
            if (sp_ih_n[set] != n + 1) { flags |= F_UNVER; sp_unver++; }
            else for (int j = 0; j < SP_NIH; j++) if (sp_rowhash(dst, sp_ih_row[j]) != h[j]) mb++;
        }
        uint32_t h1 = c_ch - 1;
        for (int j = 0; j < 6 && r_bot + 1 < h1; j++)
        {
            uint32_t row = r_bot + 1 + (uint32_t)j * (h1 - r_bot - 1) / 5;
            if (sp_rowhash(dst, row) != rt_probe(row)) mb++;
        }
        if (mb) { flags |= F_MIDTEAR; sp_midtear++; }
    }
    frames++; dsum += us; if (us > dmax) dmax = us;
    ch[k].seq = nfr; ch[k].t_us = td; ch[k].fl = flags;
    if (trig) { sp_kept++; ch[k].state = CH_FULL; give_semaphore(rt_wsem); }
    else { ch[k].state = CH_RING; sp_ring_n++; }
out:
    if (sp_rel && sp_kept) { sp_trig = 0; sp_end = 1; }
    if (nfr < RT_MAXFR)
    {
        fl[nfr].t_us = td_irq - first_t; fl[nfr].dma_us = MIN(us, 65535);
        fl[nfr].lat = MIN(lat_us / 100, 255); fl[nfr].flags = flags;
    }
    nfr++;
    int used = 0;
    for (int i = 0; i < nch; i++) if (ch[i].state != CH_FREE && ch[i].state != CH_SKIP) used++;
    if ((uint32_t)used > max_used) max_used = used;
    return 0;
}

/* one new frame: dedupe, take a slot, copy, check. td = frame time (rt_us);
 * lat_us = frame event -> now; top_ref/bot_ref = crop rows at the frame event */
static int rt_frame(uint32_t td, uint32_t lat_us, int flags, uint32_t top_ref, uint32_t bot_ref)
{
    if (c_dng) return sp_frame(td, lat_us, flags, top_ref, bot_ref);
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
            if (c_mlv) rt_mlv_slot(dst, nfr, td - first_t);
            if (rt_copy(c_src, pitch, dst + slot_pre, c_rb, c_ch, &us))
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
        if (c_dur_s > 0 && first_t && now - first_t > (uint32_t)c_dur_s * 1000000) break;
        if (rr_stop) { rt_printf("stop requested (REC button)\n"); break; }
        if (sp_active && sp_end) break;                                     /* v20: silent session ends */
        if (w_err) { rt_printf("writer error %d, stopping\n", w_err); break; }
        if (rt_card_low()) { rt_printf("card nearly full (%d MB written), stopping\n", (uint32_t)(w_bytes >> 20)); rr_full = 1; break; }
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
        if (sp_mid_on)                  /* v3.11: silent pictures copy frame n+1 mid-frame */
        {
            irq_t_of_n = t;
            if (sp_frame_mid(td, lat, n)) { c_abort = 1; break; }
            continue;
        }
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
        if (c_dur_s > 0 && first_t && now - first_t > (uint32_t)c_dur_s * 1000000) break;
        if (rr_stop) { rt_printf("stop requested (REC button)\n"); break; }
        if (sp_active && sp_end) break;                                     /* v20: silent session ends */
        if (w_err) { rt_printf("writer error %d, stopping\n", w_err); break; }
        if (rt_card_low()) { rt_printf("card nearly full (%d MB written), stopping\n", (uint32_t)(w_bytes >> 20)); rr_full = 1; break; }
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

    if (rt_crop_sel < 0 || rt_crop_sel >= COUNT(rt_crop_wh)) rt_crop_sel = 0;
    int full = rt_crop_sel == RT_CROP_FULL || sp_active;    /* v20: silent pictures take the whole MEM1 frame */
    uint32_t crop_w = rt_crop_wh[rt_crop_sel][0], crop_h = rt_crop_wh[rt_crop_sel][1];   /* full: set from MEM1 below */
    int dur_s = (rr_active || sp_active) ? 0 : rt_dur_s[rt_dur_sel];   /* 0 = until stopped (REC button) */
    for (rt_run_id = 0; rt_run_id < 99; rt_run_id++)
    {
        char n[32], m[32], x[32];
        snprintf(n, sizeof(n), "ML/RT%02d.BIN", rt_run_id); snprintf(m, sizeof(m), "ML/RT%02d.MLV", rt_run_id);
        snprintf(x, sizeof(x), "ML/RT%02d.IDX", rt_run_id);   /* v18: clips write no RTnn.MLV; keep their .IDX apart */
        if (!is_file(n) && !is_file(m) && !is_file(x)) break;
    }
    if (rr_active && rt_fmt_sel == 0) rt_fmt_sel = 1;   /* v18: REC clips are always MLV (BIN = test format) */
    if (rr_active) rr_pick_name();
    rr_full = 0;
    c_free0 = (uint64_t)get_free_space_32k(get_shooting_card()) * 32768;
    c_dng = sp_active ? 1 : 0;          /* v20 */
    c_mlv = rt_fmt_sel != 0 && !c_dng;
    c_bpp = c_dng ? (sp_bits ? 12 : 10) : rt_fmt_bpp[rt_fmt_sel];
    pk_applied = 0;
    if (!rr_active) rr_t0_ms = 0;
    if (RECORDING) rt_printf("!! Canon reports movie recording active at start (H.264 shares the card)\n");
    rt_printf("start v17 record test run %d: crop %dx%d%s, %d s%s%s, sync %s, format %s, t=%d lv=%d shooting_mode=%d movie=%d, card free %d MB\n", rt_run_id,
              crop_w, crop_h, full ? " (full frame)" : "", dur_s, rr_active ? " (REC button, until stopped)" : "", rt_osd_run ? " (status shown)" : "", rt_sync_name[rt_sync_sel],
              c_dng ? (c_bpp == 10 ? "DNG 10-bit" : "DNG 12-bit") : rt_fmt_name[rt_fmt_sel], get_ms_clock(), lv ? 1 : 0, shooting_mode, is_movie_mode() ? 1 : 0,
              get_free_space_32k(get_shooting_card()) / 32);
    if (rr_active) rt_printf("v18 clip %s.MLV, diagnostics %s\n", rr_base, rr_diag ? "on" : "off");
    if (c_dng)
    {
        sp_run_mode = sp_mode == 2 ? 2 : 1;
        sp_ready = 0; sp_kept = sp_saved = sp_pre_kept = 0; sp_ring_n = 0; sp_trig_cap = 0;
        sp_swap_us_sum = sp_swap_us_max = sp_wr_us_sum = sp_wr_us_max = sp_chk_bad = sp_torn = sp_midtear = 0;
        sp_dir[0] = 0;
        rt_printf("v20 silent session (v3.8 keys): %s, %d-bit, arm %s, keep %d before the press%s, lv_movie_select %d\n",
                  sp_run_mode == 2 ? "pre-capture" : "burst", c_bpp, sp_arm_sel ? "always" : "half-press",
                  sp_pre_n[sp_pre_sel & 3], sp_sim ? ", timed test" : "", lv_movie_select);
        if (!sp_hdr) sp_hdr = malloc(SP_HDR);
        if (!sp_recs) sp_recs = malloc(sizeof(*sp_recs) * SP_MAXREC);
        if (!sp_hdr || !sp_recs) { snprintf(rt_last, sizeof(rt_last), "no memory"); goto done; }
    }

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
    /* v17: Mem1Path layout fields, logged before any pack change (4K-mode survey: rt_pack_apply only runs
     * with mode < 0x14 and width source == width, as tested in FHD) */
    rt_printf("M1 path: mode %d, alt %d, width src %dx%d, w/h %dx%d, pack cfg {%d,%d,%d,%d,%d}\n", MEM(RT_PATH),
              MEM(RT_PATH + 0x28), MEM(RT_PATH + 0x18), MEM(RT_PATH + 0x1c), m1_w, m1_h, MEM(RT_PATH + 0x64),
              MEM(RT_PATH + 0x68), MEM(RT_PATH + 0x6C), MEM(RT_PATH + 0x70), MEM(RT_PATH + 0x74));
    if ((c_mlv || c_dng) && m1_w && m1_h)
    {
        if (rt_pack_apply(c_bpp, 1))
        { snprintf(rt_last, sizeof(rt_last), "RT%02d: pack cfg unexpected", rt_run_id); NotifyBox(6000, "Record test: %s", rt_last); goto done; }
    }
    uint32_t a = MEM(RT_PATH + 0x58);
    pitch = m1_w * c_bpp / 8;
    ubuf = RT_BUF | RT_UNC;
    rt_printf("M1 w=%d h=%d addr=%08x pitch=%d (%d-bit)\n", m1_w, m1_h, a, pitch, c_bpp);
    if (!m1_w || !m1_h || pitch * m1_h > RT_BUFLEN || (a & ~RT_UNC) != RT_BUF || m1_h < 64)
    { rt_printf("MEM1 not active or size unexpected, abort\n"); NotifyBox(6000, "Record test: no LV raw"); goto done; }

    if (full) { crop_w = m1_w; crop_h = m1_h & ~1; }   /* v17: the whole width (v16.6 capped it at 4096: 4K mode is 4256) */
    else { crop_w = MIN(crop_w, m1_w) & ~63; crop_h = MIN(crop_h, m1_h) & ~1; }
    uint32_t x0 = ((m1_w - crop_w) / 2) & ~63, y0 = ((m1_h - crop_h) / 2) & ~1;
    c_rb = crop_w * c_bpp / 8;
    c_ch = crop_h; c_cw = crop_w; c_x0 = x0; c_y0 = y0;
    c_src = ubuf + y0 * pitch + x0 * c_bpp / 8;
    r_top = y0; r_bot = y0 + crop_h - 1;
    /* v16.6: the frame-end IRQ comes ~1 ms after the 1080 crop's bottom row (1183 of 1290); a probe on the
     * frame's last rows changes too close to (or after) the IRQ for the 1 ms survey and the per-frame checks.
     * Keep the bottom probe >= 106 rows above the frame end (= the proven 1080 crop row); the offline tear
     * check (mlv_check) still covers every row. v17: margin scaled with the frame height (4K mode: 183 rows),
     * since the row time there is unmeasured. */
    uint32_t pm = MAX(107, m1_h * 107 / 1290);
    if (m1_h > pm && r_bot > m1_h - pm) r_bot = MAX(r_top, m1_h - pm);
    if (r_bot != y0 + crop_h - 1) rt_printf("probe rows: top %d, bottom %d (crop bottom %d too close to the frame end)\n",
                                            r_top, r_bot, y0 + crop_h - 1);
    frame_bytes = c_rb * crop_h;
    slot_pre = c_dng ? SP_HDR : c_mlv ? RT_VIDF_PRE : 0;
    slot_bytes = (frame_bytes + slot_pre + 4095) & ~4095;
    slot_null = c_mlv ? slot_bytes - slot_pre - frame_bytes : 0;   /* 0 or >= 16 (frame_bytes is a multiple of 16) */
    kslots = c_dng ? 1 : MAX(1, RT_CHUNK_TGT / slot_bytes);       /* v20: one frame per chunk (ring, DNG per slot) */
    rt_printf("crop %dx%d at (%d,%d), %d bytes/frame, slot %d, %d slots/chunk (%d bytes per write)\n",
              crop_w, crop_h, x0, y0, frame_bytes, slot_bytes, kslots, kslots * slot_bytes);
    if (c_mlv) rt_printf("MLV: VIDF %d + frameSpace %d before each frame, NULL block %d after\n", (int)sizeof(struct rt_vidf),
                         RT_VIDF_PRE - (int)sizeof(struct rt_vidf), slot_null);

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

    if (rr_audio && c_mlv && !ra_buf)       /* v19: audio blocks before the video chunks take the memory */
    {
        ra_buf = fio_malloc(RA_NBUF * RA_BLK);
        if (!ra_buf) rt_printf("audio: no memory for the audio blocks, no audio\n");
    }
    /* v3.14: SRM "Extra memory" removed: Canon's 1st-job buffer is carved from the same shoot memory (no gain), and an
     * unanswered request in LV blocks all later allocations until a power cycle (lv-survey.md "SRM static RE"). */
    rt_alloc_chunks(c_dng ? RT_MAXCH : RT_MAXCH_VIDEO);
    (void)sp_srm_carve; (void)sp_srm_alloc; (void)sp_srm_sel;
    rt_printf("buffer: %d chunks = %d frames (%d MB)\n", nch, nch * kslots, nch * kslots * slot_bytes >> 20);
    rt_flush();
    if (nch < 2) { snprintf(rt_last, sizeof(rt_last), "not enough memory (%d)", nch); goto done; }
    w_bytes = 0; w_part = 0;
    if (rt_card_low())                  /* v18 */
    {
        rt_printf("card free %d MB: less than the buffer + 64 MB, not starting\n", (uint32_t)(c_free0 >> 20));
        snprintf(rt_last, sizeof(rt_last), "card full");
        rr_full = 1;
        goto done;
    }

    rt_begin();
    {
        int s = rt_copy_selftest(c_x0 * c_bpp / 8, kslots * slot_bytes);
        if (s == -2) { rt_fail(); snprintf(rt_last, sizeof(rt_last), "DMA timeout (self-test)"); goto done; }
        if (s) { snprintf(rt_last, sizeof(rt_last), "RT%02d: copy self-test failed", rt_run_id); NotifyBox(6000, "Record test: %s", rt_last); goto done; }
    }
    {
        uint32_t us;
        if (rt_copy(c_src, pitch, ch[0].base + slot_pre, c_rb, c_ch, &us))
        { rt_printf("!! first EDMAC copy timed out\n"); rt_fail(); snprintf(rt_last, sizeof(rt_last), "DMA timeout"); goto done; }
        rt_printf("first copy: dma %d us\n", us);
    }
    if (c_dng)                          /* v3.10: copy time into every chunk (v3.8/v3.9 camera: frames in chunks 6+ copied in
                                         * 30-37 ms and tore, chunks 0-5 in ~25 ms); chunks over SP_SLOW_US aren't used */
    {
        int nfast = 0;
        for (int i = 0; i < nch; i++)
        {
            uint32_t mx = 0;
            for (int pass = 0; pass < 2; pass++)
            {
                uint32_t us;
                if (rt_copy(c_src, pitch, ch[i].base + slot_pre, c_rb, c_ch, &us))
                { rt_printf("!! survey copy timed out\n"); rt_fail(); snprintf(rt_last, sizeof(rt_last), "DMA timeout"); goto done; }
                if (us > mx) mx = us;
            }
            ch[i].cp_us = mx;
            if (mx <= SP_SLOW_US) nfast++;
        }
        for (int i = 0; i < nch; i++)
            rt_printf("chunk %d at %08x: copy %d us%s\n", i, ch[i].base, ch[i].cp_us,
                      ch[i].cp_us > SP_SLOW_US ? (nfast >= 3 ? " -> too slow, not used" : " -> slow (kept: fewer than 3 others)") : "");
        if (nfast >= 3) for (int i = 0; i < nch; i++) if (ch[i].cp_us > SP_SLOW_US) ch[i].state = CH_SKIP;
        sp_nfast = nfast >= 3 ? nfast : nch;
        /* v3.11: fastest chunks first (a burst fills them in index order) */
        for (int i = 1; i < nch; i++)
            for (int j = i; j > 0 && ch[j - 1].cp_us > ch[j].cp_us; j--) { struct rt_chunk t = ch[j]; ch[j] = ch[j - 1]; ch[j - 1] = t; }
    }

    if (c_dng)                          /* v20: a folder per session, the writer makes one DNG per kept frame */
    {
        w_file = 0;
        sp_dir[0] = 0;                  /* folder made by the writer at the first DNG: no empty folders */
        sp_ring_max = MAX(1, MIN(sp_pre_n[sp_pre_sel & 3], sp_nfast - 2));
        rt_printf("v20: %d frames fit in memory (%d usable), pre-capture ring %d\n", nch, sp_nfast, sp_ring_max);
    }
    else
    {
        rt_part_name(w_name, rt_run_id, 0);
        w_file = FIO_CreateFile(w_name);
        if (!w_file) { rt_printf("cannot create %s\n", w_name); goto done; }
    }
    w_part = 0; w_fsize = 0; w_bytes = 0; w_n = 0; w_us_sum = 0; w_us_min = 0xFFFFFFFF; w_us_max = 0; w_stop = 0; w_err = 0; w_done = 0; w_part_frames = 0;
    ra_bytes_written = 0;               /* v19: also when this clip has no audio */
    if (c_mlv && !mlv_hdr) mlv_hdr = malloc(RT_HDR_BYTES);
    if (c_mlv && !mlv_hdr) { rt_printf("no memory for the MLV header\n"); FIO_CloseFile(w_file); w_file = 0; goto done; }
    if (c_mlv)
    {
        rt_fps_of(mode == SYNC_IRQ ? sv_irq_med : sv_est, &mlv_fps_nom, &mlv_fps_den);
        mlv_guid = ((uint64_t)rt_us() << 32) ^ (uint64_t)get_ms_clock() * 2654435761u ^ rt_run_id;
        if (rr_audio && ra_buf) ra_start();     /* v19: sets ra_on, so the header gets WAVI */
        rt_mlv_build_hdr(0, 0);
        if (FIO_WriteFile(w_file, mlv_hdr, RT_HDR_BYTES) != RT_HDR_BYTES)
        { rt_printf("cannot write MLV header\n"); FIO_CloseFile(w_file); w_file = 0; goto done; }
        w_fsize = RT_HDR_BYTES;
        rt_printf("MLV: %d/%d fps, guid %08x%08x\n", mlv_fps_nom, mlv_fps_den, (uint32_t)(mlv_guid >> 32), (uint32_t)mlv_guid);
    }
    rt_t0 = rt_us();
    if (c_dng) task_create("sp_writer", 0x1c, 0x2000, sp_writer, 0);
    else task_create("rt_writer", 0x1c, 0x1000, rt_writer, 0);
    rt_flush();

    /* capture in a high-priority task that blocks between frames */
    nfr = frames = drops = dups = missed = late = tears = topchg = botchg = ntl = irq_timeouts = 0;
    cur = -1; next_fill = 0; first_drop_ms = -1; c_abort = 0;
    first_t = next_tl = max_used = dmax = dsum = last_rec_bot = t_last_ev = 0;
    c_mode = mode; c_dur_s = dur_s; c_est = sv_est; c_done = 0;
    uint32_t irq0 = irq_n;
    rt_drain(rt_csem);
    rr_t0_ms = get_ms_clock();
    sp_mid_on = 0;
    if (c_dng && mode == SYNC_IRQ)      /* v3.11: mid-frame copies (see sp_frame_mid) */
    {
        if (!sp_tsem) sp_tsem = create_named_semaphore("sp_tim", SEM_CREATE_LOCKED);
        rt_drain(sp_tsem);
        sp_late = sp_unver = 0;
        for (int j = 0; j < SP_NIH; j++) sp_ih_row[j] = r_top + 1 + (uint32_t)j * (r_bot - r_top - 2) / (SP_NIH - 1);
        sp_ih_n[0] = sp_ih_n[1] = 0;
        sp_mid_delay = (sv_irq_to_top > 500 && sv_irq_to_top < 5000 ? sv_irq_to_top : 1850) + SP_MID_OFS;
        RT_DMB();
        sp_ih_on = 1;
        sp_mid_on = 1;
        rt_printf("v3.11 mid-frame copy: start %d us after the frame-end IRQ (frame start + %d us)\n", sp_mid_delay, SP_MID_OFS);
        rt_flush();
    }
    task_create("rt_capture", RT_CAP_PRIO, 0x1000, rt_capture_task, 0);   /* (rr_t0_ms: on-screen timer) */
    if (c_dng) sp_ready = 1;            /* v20: armed (status shows READY) */
    if (dur_s > 0)
    {
        take_semaphore(rt_csem, (dur_s + 10) * 1000);
        if (!c_done) { rt_printf("capture task did not finish in time\n"); for (int i = 0; i < 500 && !c_done; i++) msleep(10); }
    }
    else while (!c_done) take_semaphore(rt_csem, 1000);       /* REC run: until stopped or an error */
    uint32_t irq_rec = irq_n - irq0;
    uint32_t dur_us = t_last_ev - first_t;

    /* hand over the partial chunk, stop the writer */
    if (cur >= 0) { if (ch[cur].n) ch[cur].state = CH_FULL; else ch[cur].state = CH_FREE; }
    if (c_dng)                              /* v20: drop unused ring frames; free the DMA engine before the slow DNG writes */
    {
        sp_ref_cap = sp_trig_cap ? r_refresh_count - sp_ref0 : 0;
        r_refresh_hold = 0;
        sp_ih_on = 0;
        if (sp_mid_on) rt_printf("v3.11: %d copies skipped (would start late), %d unverified\n", sp_late, sp_unver);
        sp_mid_on = 0;
        sp_ready = 0;
        for (int i = 0; i < nch; i++) if (ch[i].state == CH_RING || ch[i].state == CH_FILL) ch[i].state = CH_FREE;
        if (!c_abort) rt_end();
        rt_unhook();
        rt_pack_restore();
        if (lv_raw) { call("lv_save_raw", 0); lv_raw = 0; }
        rt_unlock();
        rt_printf("v20 capture ended: %d kept (%d before the press), writing DNGs\n", sp_kept, sp_pre_kept);
        rt_flush();
    }
    if (ra_started)                         /* v19: last audio chunks, written before the writer stops */
    {
        ra_stop();
        give_semaphore(rt_wsem);
        for (int i = 0; i < 500; i++)
        {
            int pending = 0;
            for (int k = 0; k < RA_NBUF; k++) pending += ra_ready[k] != 0;
            if (!pending || w_done) break;
            msleep(10);
        }
    }
    uint32_t ts = rt_us();
    w_stop = 1;
    give_semaphore(rt_wsem);
    for (int i = 0; i < (c_dng ? 60000 : 3000) && !w_done; i++) msleep(10);   /* v20: DNGs take ~1-2 s each */
    uint32_t drain_ms = (rt_us() - ts) / 1000;
    if (w_done && !c_dng)
    {
        if (c_mlv) rt_mlv_close(w_file, w_part, w_part_frames);
        else FIO_CloseFile(w_file);
        w_file = 0;
        if (!w_bytes && !w_part) { FIO_RemoveFile(w_name); rt_printf("no frames written: %s removed\n", w_name); }
    }
    else if (!w_done) rt_printf("writer still busy after %d s; file left open, buffers kept\n", c_dng ? 600 : 30);
    if (!c_abort) rt_end();
    rt_unhook();

    /* summary */
    uint64_t wus = w_us_sum;
    uint32_t wmax = w_us_max, wmin = w_n ? w_us_min : 0;
    uint64_t wb = w_bytes - ra_bytes_written;   /* v19: video only (audio writes are not timed, nor frames) */
    uint32_t est = mode == SYNC_IRQ ? sv_irq_med : c_est;
    /* v17.1: data rate from the nominal frame rate when the measured period is near one (timer mode's adaptive
     * estimate drifted on 4K runs: "needed 111.6" for 1080 14-bit at 29.97p, really 103.7) */
    {
        uint32_t fn, fd;
        rt_fps_of(mode == SYNC_IRQ ? sv_irq_med : sv_est, &fn, &fd);
        if (fd != 1000 || fn % 1000 == 0) est = (uint32_t)(1000000ULL * fd / fn);
    }
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
    if (!c_dng) rt_printf("files: %s (last part %d), %d MB total, format %s; Canon recording now: %d\n", w_name, w_part, (uint32_t)(wb >> 20),
              rt_fmt_name[rt_fmt_sel], RECORDING ? 1 : 0);
    snprintf(rt_last, sizeof(rt_last), "RT%02d %s%s: %d rec, %d miss", rt_run_id, c_mlv ? (c_bpp == 10 ? "MLV10 " : c_bpp == 12 ? "MLV12 " : "MLV14 ") : "",
             rt_sync_name[mode], frames, missed + drops);
    if (rr_active)                      /* v18: clip summary for the user */
    {
        snprintf(rt_last, sizeof(rt_last), "%s.MLV %dx%d %d-bit, %d frames%s", rr_base, crop_w, crop_h, c_bpp, frames,
                 missed + drops ? ", FRAMES LOST" : rr_full ? ", card full" : "");
        if (missed + drops) rt_printf("!! clip lost %d frames (%d dropped: buffer full, %d missed)\n", missed + drops, drops, missed);
        if (w_n >= 20 && !w_err && card10 > 300) rr_card10 = card10;   /* menu's card speed */
    }
    if (c_dng)                          /* v20: silent session summary, per-frame list, INFO.TXT in the folder */
    {
        int n = sp_saved, nr = MIN(n, SP_MAXREC);
        rt_printf("SP RESULT: %s %dx%d %d-bit, %d kept (%d before the press), %d DNGs written, %d dropped (memory full), "
                  "torn %d (mid-copy %d), display refreshes during capture %d, swap check bad %d; swap avg %d max %d ms, card write avg %d max %d ms per DNG; writer err %d\n",
                  sp_dir[0] ? sp_dir : "(no folder)", crop_w, crop_h, c_bpp, sp_kept, sp_pre_kept, n, drops, sp_torn,
                  sp_midtear, sp_ref_cap, sp_chk_bad, n ? sp_swap_us_sum / n / 1000 : 0, sp_swap_us_max / 1000, n ? sp_wr_us_sum / n / 1000 : 0,
                  sp_wr_us_max / 1000, w_err);
        const int cap = 200 + nr * 112;
        char *b = malloc(cap);
        if (b && sp_dir[0])
        {
            int len = snprintf(b, cap, "LV silent pictures %s: %dx%d %d-bit, %s, %d DNGs (%d before the press), %d dropped\n"
                               "name     ms from press  dma us  swap ms  write ms  chunk  start ms  flags\n", sp_dir, crop_w, crop_h, c_bpp,
                               sp_run_mode == 2 ? "pre-capture" : "burst", n, sp_pre_kept, drops);
            for (int i = 0; i < nr && len < cap - 112; i++)
            {
                struct sp_rec *q = &sp_recs[i];
                int32_t ms = (int32_t)(q->t_us - sp_press_us) / 1000;
                int lat = q->seq < RT_MAXFR ? fl[q->seq].lat : 0;     /* frame event -> copy start, 0.1 ms */
                int w = snprintf(b + len, cap - len, "F%04d.DNG %s%5d  %6d  %7d  %8d  %5d  %4d.%d  %s%s%s\n", i, ms < 0 ? "-" : "+",
                                 ms < 0 ? -ms : ms, q->seq < RT_MAXFR ? fl[q->seq].dma_us : 0, q->swap_us / 1000,
                                 q->wr_us / 1000, q->chunk, lat / 10, lat % 10, (q->fl & F_TORN) ? "TORN? " : "",
                                 (q->fl & F_LATE) ? "late " : "", (q->fl & 0x100) ? "SWAP-CHECK-BAD" : "");
                if (w > 0) len += MIN(w, cap - len - 1);
            }
            char nm[32];
            snprintf(nm, sizeof(nm), "%s/INFO.TXT", sp_dir);
            FILE *f = FIO_CreateFile(nm);
            if (f) { FIO_WriteFile(f, b, len); FIO_CloseFile(f); }
            rt_printf("%s", b);
        }
        if (b) free(b);
        if (n || sp_kept || w_err)
            snprintf(rt_last, sizeof(rt_last), "%s: %d DNG (%d before press)%s%s", sp_dir[0] ? sp_dir : "-", n, sp_pre_kept,
                     drops ? ", some frames dropped" : "", w_err ? ", WRITE ERROR" : "");
        else rt_last[0] = 0;
    }
    rt_flush();
    if (rt_diag())
    {
        for (int i = 0; i < ntl; i++)
            rt_printf("TL %6d ms frames %5d drops %5d buffered %3d written %5d MB\n", tl[i].t_ms4 * 4, tl[i].frames,
                      tl[i].drops, tl[i].used, tl[i].mb_written);
        rt_flush();
        for (int i = 0; i < MIN(w_n, RT_MAXW); i++)
            rt_printf("W %6d ms %7d us %9d bytes\n", wl[i].t_ms, wl[i].us, wl[i].bytes);
        rt_flush();
        char n[32]; snprintf(n, sizeof(n), "ML/RT%02d.IDX", rt_run_id);
        FILE *f = FIO_CreateFile(n);
        if (f) { FIO_WriteFile(f, fl, sizeof(*fl) * MIN(nfr, RT_MAXFR)); FIO_CloseFile(f); }
    }

done:
    sp_ih_on = 0; sp_mid_on = 0;            /* v3.11 */
    if (ra_started) ra_stop();              /* v19: error paths */
    if (ra_on) rt_printf("audio: %d blocks written (%d KB)\n", (int)ra_count, (int)(ra_bytes_written >> 10));
    ra_on = 0;
    if (ra_buf && w_done && !ra_task_alive) { fio_free(ra_buf); ra_buf = 0; }   /* never under a live ra_task */
    rt_unhook();
    rt_pack_restore();
    if (lv_raw) call("lv_save_raw", 0);
    if (rt_begun) rt_end();
    if (w_done) rt_free_chunks();
    if (w_done) sp_srm_free();              /* v3.13: after the writer is done with its slots */
    rt_unlock();
    rt_printf("done t=%d\n", get_ms_clock());
    rt_flush();
    if (rt_diag()) rt_export(rt_run_id);
    rt_runs++;
    if (c_dng) { if (rt_last[0]) NotifyBox(4000, "Silent: %s", rt_last); }   /* v20: nothing for a cancelled session */
    else if (rr_active) NotifyBox(8000, "RAW: %s%s", strncmp(rt_last, rr_base, strlen(rr_base)) ? "NOT recorded: " : "", rt_last);
    else NotifyBox(8000, "Record test: DONE %s", rt_last);
out:
    if (rt_logbuf) free(rt_logbuf);
    if (fl) free(fl);
    if (wl) free(wl);
    if (tl) free(tl);
    if (smp) free(smp);
    rt_logbuf = 0; fl = 0; wl = 0; tl = 0; smp = 0;
    if (!c_dng || rt_last[0]) info_led_blink(3, 100, 100);
    rt_busy = 0;
}

/* ---- v16: REC button starts/stops raw recording ----
 * RECORD button events on the R (platform/R.180/gui.h note): 0x21 press, 0x22 release.
 * Called from handle_common_events_by_feature (gui-common.c) before the module keys.
 * Only when the menu switch is on, in movie-mode live view, with the ML menu closed; the
 * press and release are swallowed so Canon doesn't start H.264 recording. */
/* v16.2/v16.3 camera key logs: each REC press gives 0x20 at once (with 0x85), then 0x21 ~1-1.3 s later.
 * Passing 0x20 let Canon record H.264 alongside (MP4s in DCIM matching both REC runs; card shared, 88-93
 * MiB/s, dropped frames). So 0x20 is the press; 0x21 looks like Canon's record start/stop notification. */
#define RR_KEY_REC_PRESS   0x20
#define RR_KEY_REC_NOTIFY  0x21
#define RR_KEY_REC_UNPRESS 0x22
static void rr_task(void *unused)
{
    rr_t0_ms = 0;
    rr_active = 1; rr_stop = 0;
    rt_last[0] = 0;
    rt_task(0);                         /* synchronous; clears rt_busy */
    rr_active = 0; rr_stop = 0;
}

static void rr_key_flush(void);

#define RR_STATUS_Y 8                   /* v19.3: meters sit to the right of it (RA_M_X) */
#define RR_STATUS_CLEAR_W (RA_M_X - 12)   /* clearing the status never touches the meters */
static void rr_status_task(void *unused)
{
    /* on-screen status while a REC-button run is active. v16 redrew every 250 ms and that run wrote
     * 93 MiB/s against 110 for a run without it (DMA copies slower too): redraw only when the text
     * changes, at most once a second. */
    int shown = 0, tick = 0;
    char last[40] = "";
    for (;;)
    {
        msleep(250);
        if (++tick % 4 == 0 && !rr_active && !rt_osd_run && !rt_busy && (rr_diag || sp_mode)) rr_key_flush();   /* no card writes of ours during a recording; v18: key log only with REC diagnostics; v20: or silent pictures on */
        int on = rr_active || rt_osd_run || sp_active;
        if (!on) { if (shown) { rr_meter_layout(); bmp_fill(COLOR_EMPTY, 8, RR_STATUS_Y, RR_STATUS_CLEAR_W, 30); shown = 0; last[0] = 0; } continue; }
        if (gui_menu_shown() || (tick % 4 && !sp_active)) continue;   /* v20: silent status every 250 ms */
        char t[40];
        uint32_t s_rec = !rr_t0_ms ? 0 : (get_ms_clock() - rr_t0_ms) / 1000;
        int red = rr_t0_ms && !rr_stop;
        if (sp_active)
        {
            red = sp_ready && sp_trig;
            if (sp_ready && sp_trig) snprintf(t, sizeof(t), " SILENT %d ", sp_kept);
            else if (sp_ready) snprintf(t, sizeof(t), " SILENT ready %db ", c_bpp);
            else if (sp_kept) snprintf(t, sizeof(t), " saving %d/%d ", sp_saved, sp_kept);
            else snprintf(t, sizeof(t), " SILENT arming ");
        }
        else if (rr_stop) snprintf(t, sizeof(t), " stopping... ");
        else if (!rr_t0_ms) snprintf(t, sizeof(t), " raw: starting ");
        else snprintf(t, sizeof(t), " REC %02d:%02d %dx%d %db ", s_rec / 60, s_rec % 60, c_cw, c_ch, c_bpp);   /* v18: + size */
        if (shown && !strcmp(t, last)) continue;
        if (shown && strlen(t) != strlen(last)) { rr_meter_layout(); bmp_fill(COLOR_EMPTY, 8, RR_STATUS_Y, RR_STATUS_CLEAR_W, 30); }
        bmp_printf(FONT(FONT_MED, COLOR_WHITE, red ? COLOR_RED : COLOR_BLACK), 8, RR_STATUS_Y, "%s", t);
        snprintf(last, sizeof(last), "%s", t);
        shown = 1;
    }
}

/* v16.1 key log: v16 on the camera let REC through to Canon. is_movie_mode() looks wrong on the R
 * (shooting_mode == 0x14; v10 logged movie=0 in movie mode), so the movie check is gone; and every
 * button event (not Canon's 0x9D info events) is logged with the state, flushed to ML/RRKEYS.TXT. */
struct rr_key { uint32_t t_ms; int16_t param; uint8_t lv, menu, movie, taken; uint16_t smode; uint8_t rec, sp; };   /* v20: sp = silent session bits */
#define RR_NKEY 128
static struct rr_key rr_keys[RR_NKEY];
static volatile uint32_t rr_nkey;       /* total logged; ring index = n % RR_NKEY */
static uint32_t rr_nkey_flushed;

static int rr_key_log(int param, int taken)
{
    static uint32_t t85;                /* v3.8: a held full press sends 0x85 every ~100 ms: log 2 per second */
    if (param == 0x85)
    {
        uint32_t t = get_ms_clock();
        if (t - t85 < 500) return taken ? 0 : 1;
        t85 = t;
    }
    uint32_t n = rr_nkey;
    struct rr_key *k = &rr_keys[n % RR_NKEY];
    k->t_ms = get_ms_clock(); k->param = param; k->lv = lv ? 1 : 0; k->menu = gui_menu_shown() ? 1 : 0;
    k->movie = is_movie_mode() ? 1 : 0; k->taken = taken; k->smode = shooting_mode; k->rec = RECORDING ? 1 : 0;
    k->sp = (sp_active ? 1 : 0) | (sp_ready ? 2 : 0) | (sp_trig ? 4 : 0) | (lv_movie_select == LVMS_ENABLE_MOVIE ? 8 : 0) | (sp_half ? 0x10 : 0);
    rr_nkey = n + 1;
    return taken ? 0 : 1;
}

static void rr_key_flush(void)
{
    uint32_t n = rr_nkey;
    if (n == rr_nkey_flushed) return;
    if (n - rr_nkey_flushed > RR_NKEY) rr_nkey_flushed = n - RR_NKEY;
    const int cap = RR_NKEY * 128;
    char *b = malloc(cap);
    if (!b) return;
    int len = 0;
    for (; rr_nkey_flushed < n && len < cap - 128; rr_nkey_flushed++)
    {
        struct rr_key *k = &rr_keys[rr_nkey_flushed % RR_NKEY];
        int w = snprintf(b + len, cap - len, "t=%d ev=%02x lv=%d menu=%d movie=%d shooting_mode=%d canon_rec=%d rec_switch=%d silent=%d sp=%x -> %s\n",
                         k->t_ms, k->param & 0xFFFF, k->lv, k->menu, k->movie, k->smode, k->rec, rr_on, sp_mode, k->sp, k->taken ? "TAKEN" : "passed");
        if (w > 0) len += MIN(w, cap - len - 1);
    }
    FILE *f = FIO_CreateFileOrAppend("ML/RRKEYS.TXT");
    if (f) { FIO_WriteFile(f, b, len); FIO_CloseFile(f); }
    free(b);
}

/* ---- v20: LV silent pictures: keys, session start, watcher, timed test ----
 * Shutter on the R (camera key log, v3.7 test 2026-10-30): half-press = 0x85 0x85 0x81 0x7D, half release = 0x82 0x7E.
 * The full press has NO code of its own: while it is held Canon sends a lone 0x85 about every 100 ms (0x85 also comes
 * right before every other key's code, so a lone 0x85 is one not followed by another code). Full press = 2 lone 0x85
 * within 250 ms while the half-press is held; its end = 0x7E, or no 0x85 for 250 ms (sp_frame / sp_watch_task).
 * 0x81/0x82 come with every half-press, so AF-ON can't be a separate trigger.
 * Half-press arms a session (~2 s: LV raw on, survey, memory), the full press held = frames kept, its release ends the
 * session (then the DNGs are written); a half release without a full press cancels. All shutter events still go to
 * Canon (AF; the full press does nothing in movie mode). REC is taken during a session. */
#define SP_KEY_HALF     0x7D
#define SP_KEY_UNHALF   0x7E
#define SP_KEY_TICK     0x85
#define SP_PND          "ML/SETTINGS/SPARM.PND"   /* Arm = Always session running (see sp_watch_task) */

static int sp_movie_lv(void) { return lv && lv_movie_select == LVMS_ENABLE_MOVIE && !gui_menu_shown(); }

static void sp_task(void *unused)
{
    rt_task(0);                         /* synchronous; clears rt_busy */
    r_refresh_hold = 0;                 /* any exit path */
    if (is_file(SP_PND)) FIO_RemoveFile(SP_PND);     /* session finished normally */
    if (sp_half && get_ms_clock() - sp_t85 < SP_TICK_GAP_MS) sp_wait_rel = 1;   /* memory full while held */
    sp_active = 0; sp_ready = 0; sp_trig = 0; sp_end = 0; sp_rel = 0;   /* key log: rr_status_task flushes it once idle */
}

static void sp_mark_press(void) { sp_press_us = rt_us(); LoadCalendarFromRTC(&sp_tm); }

/* claim the engine and start a session; trig = the trigger is already held */
static int sp_try_start(int trig)
{
    uint32_t old = cli();
    int ok = !rt_busy && !sp_active;    /* sp_active: the last session's task is still finishing */
    if (ok) rt_busy = 1;
    sei(old);
    if (!ok) return 0;
    sp_end = 0; sp_rel = 0; sp_ready = 0; sp_trig = 0; sp_kept = sp_saved = sp_pre_kept = 0;   /* status: no stale counts */
    if (trig) { sp_mark_press(); sp_trig = 1; }
    sp_active = 1;
    rt_last[0] = 0;
    task_create("sp_task", 0x1e, 0x2000, sp_task, 0);
    return 1;
}

/* -1: not a silent-picture key; else 0 = taken, 1 = passed to Canon */
static int sp_keys(int p)
{
    uint32_t now = get_ms_clock();
    if (p == SP_KEY_TICK)
    {
        if (sp_half)
        {
            sp_n85 = (sp_n85 && now - sp_t85 < SP_TICK_GAP_MS) ? sp_n85 + 1 : 1;
            sp_t85 = now;
            if (sp_n85 >= 2)            /* full press held */
            {
                if (sp_active) { if (!sp_end && !sp_trig) { sp_mark_press(); sp_trig = 1; } }
                else if (sp_mode && !rt_busy && !sp_wait_rel && sp_movie_lv()) sp_try_start(1);
            }
        }
        return 1;
    }
    sp_n85 = 0;                         /* the 0x85 before this one belonged to this key */
    if (p == SP_KEY_HALF)
    {
        sp_half = 1;
        if (sp_mode && sp_arm_sel == 0 && !sp_active && !rt_busy && sp_movie_lv()) sp_try_start(0);
        return 1;
    }
    if (p == SP_KEY_UNHALF)
    {
        sp_half = 0; sp_wait_rel = 0;
        if (sp_active && sp_trig && !sp_sim) sp_rel = 1;                       /* sp_frame ends it after >= 1 kept frame */
        else if (sp_active && !sp_trig && !sp_sim && sp_arm_sel == 0) sp_end = 1;   /* released without a picture */
        return 1;
    }
    if (sp_active && p == RR_KEY_REC_PRESS)
    {
        if (!sp_trig) sp_end = 1;
        return 0;
    }
    return -1;
}

/* every 200 ms: cancel an armed session when live view / movie mode / the closed ML menu is gone; Arm = Always
 * starts a session after 1 s of movie LV with nothing else running. Always-armed sessions leave a marker file while
 * they run: if one is still there at boot, the last one never finished (freeze?), so Arm goes back to Half-press. */
static void sp_watch_task(void *unused)
{
    extern int ml_started;
    int ok_ms = 0;
    while (!ml_started) msleep(100);
    msleep(3000);                       /* config loaded by now */
    if (is_file(SP_PND))
    {
        FIO_RemoveFile(SP_PND);
        if (sp_arm_sel) { sp_arm_sel = 0; NotifyBox(6000, "LV silent: last armed session did not finish; Arm = Half-press"); }
    }
    for (;;)
    {
        msleep(200);
        if (r_bisect_bg == 1 || r_bisect_bg == 3) { msleep(800); continue; }
        int mlv = sp_movie_lv() && DISPLAY_IS_ON;
        sp_check_tick_end();
        if (!mlv) sp_half = 0;          /* a release may have gone to the ML menu or Canon's */
        if (sp_wait_rel && (!sp_half || get_ms_clock() - sp_t85 > SP_TICK_GAP_MS)) sp_wait_rel = 0;
        if (sp_active && !sp_trig && !sp_end && (!mlv || (!sp_mode && !sp_sim))) sp_end = 1;
        if (sp_mode && sp_arm_sel == 1 && mlv && !rt_busy && !sp_active && !sp_sim) ok_ms += 200; else ok_ms = 0;
        if (ok_ms >= 1000)
        {
            ok_ms = 0;
            FILE *f = FIO_CreateFile(SP_PND);
            if (f) FIO_CloseFile(f);
            if (!sp_try_start(0)) FIO_RemoveFile(SP_PND);
        }
    }
}

/* Debug: timed test without any key (arm, ~1-2 s, trigger held 1 s, release). Opening the ML menu cancels it. */
static void sp_sim_task(void *unused)
{
    for (int i = 0; i < 300 && gui_menu_shown(); i++) msleep(100);
    msleep(1000);
    if (!sp_movie_lv() || !sp_try_start(0)) { NotifyBox(4000, "Silent test: needs movie-mode live view, ML menu closed"); sp_sim = 0; return; }
    for (int i = 0; i < 150 && sp_active && !sp_ready && !sp_end; i++) msleep(100);
    if (sp_active && sp_ready && !sp_end)
    {
        msleep(sp_mode == 2 ? 2000 : 1000);
        if (sp_active && !sp_end) { sp_mark_press(); sp_trig = 1; }
        msleep(1000);
        sp_trig = 0; sp_end = 1;
    }
    else sp_end = 1;
    while (sp_active) msleep(100);
    sp_sim = 0;
}

static void sp_sim_select(void *priv, int delta)
{
    if (sp_sim || rt_busy) return;
    if (!sp_mode) { NotifyBox(3000, "Set LV silent pictures to Burst or Pre-capture first"); return; }
    sp_sim = 1;
    task_create("sp_sim", 0x1e, 0x1000, sp_sim_task, 0);
}

int handle_rawrec_keys(struct event *event)
{
    int p0 = event->param;
    if ((sp_mode || sp_active) && p0 != 0x9D)  /* v20: silent-picture keys first; every key is logged while it's on */
    {
        int r = sp_keys(p0);
        if (r >= 0) return rr_key_log(p0, !r);
        if (!rr_on) return rr_key_log(p0, 0);
    }
    if (!rr_on) return 1;
    int p = event->param;
    if (p == 0x9D) return 1;            /* GMT_OLC_INFO_CHANGED: Canon-internal, frequent */
    if (p != RR_KEY_REC_PRESS && p != RR_KEY_REC_NOTIFY && p != RR_KEY_REC_UNPRESS) return rr_key_log(p, 0);
    if (!lv || gui_menu_shown()) return rr_key_log(p, 0);
    if (p != RR_KEY_REC_PRESS) return rr_key_log(p, RECORDING ? 0 : 1);  /* if Canon records anyway, let its notices through */
    if (rr_active) { rr_stop = 1; return rr_key_log(p, 1); }
    if (rt_busy) return rr_key_log(p, 1);   /* a test run is going: ignore, but don't start Canon recording either */
    rt_busy = 1;
    task_create("rr_task", 0x1e, 0x2000, rr_task, 0);
    return rr_key_log(p, 1);
}

/* ---- v14 pack test (v14.1 settings) ----
 * Static RE (notes lv-survey.md, "Pixel packing control"): port 3 (MEM1) has pack
 * unit 2 (mmio 0xD0422200); its config word at +0x24 is composed by 0xE0537416
 * from 5 fields in the Mem1Path struct at +0x64 (Canon default {0, 1, 2, 1, 0}):
 * reg = f1<<20 | f4<<16 | f0<<8 | f2<<4 | f3. Canon re-applies the struct to the
 * register every frame (FUN_e0638b7a, confirmed on camera by v14).
 * v14 result: f1 and f4 don't change the bit order. v14.1 tries:
 *   f3 = 0 (bit 0, cleared by Canon's lv_set_mm), still 14-bit;
 *   f2 = 1 / 0 (hypothesis: 12 / 10-bit). The port-3 line length comes from the
 *   same struct: xsize = (width * 14) >> 3 with width = +0x18 in this mode, so the
 *   width field is set to a value giving exactly w*12/8 (w*10/8) bytes. Canon
 *   applies both from the struct in the same per-frame call, so they stay
 *   consistent; the two words are written with interrupts off.
 * Each setting: wait, check the band still changes (DMA not stalled), copy 16
 * lines (at the setting's line length), log. Struct restored at the end.
 * Output: ML/PACKTEST.BIN ("PKT2" header + band per setting). */
#define PK_CFG      (RT_PATH + 0x64)    /* 5 words */
#define PK_WSRC     (RT_PATH + 0x18)    /* width used for port-3 xsize in this mode */
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
    char magic[4];                      /* "PKT2" */
    uint32_t idx, f1, f4, reg_before, reg_set, reg_after, w, h, pitch, row0, rows, t_ms;
    uint32_t f2, f3, bpp, wsrc, changing;
};

static uint32_t pk_hash(const uint8_t *p, int n)
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i += 4) h = (h ^ *(const uint32_t *)(p + i)) * 16777619u;
    return h;
}

static void rt_pack_task(void *unused)
{
    static const struct { uint8_t f2, f3, bpp; } combos[] = {   /* first = Canon's default */
        { 2, 1, 14 }, { 2, 0, 14 }, { 1, 1, 12 }, { 0, 1, 10 },
    };
    rt_logbuf = malloc(RT_LOGLEN);
    rt_loglen = 0;
    uint8_t *band = 0;
    FILE *f = 0;
    int lv_raw = 0, changed = 0, nb = 0;
    uint32_t orig[5], reg0 = 0, w0 = 0;
    if (!rt_logbuf) { NotifyBox(6000, "Pack test: out of memory"); goto out; }
    rt_printf("start v14.1 pack test: t=%d lv=%d\n", get_ms_clock(), lv ? 1 : 0);
    if (!lv) { rt_printf("no live view, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: no LV"); goto done; }

    call("lv_save_raw", 1);
    lv_raw = 1;
    msleep(500);
    m1_w = MEM(RT_PATH + 0x10); m1_h = MEM(RT_PATH + 0x14);
    uint32_t a = MEM(RT_PATH + 0x58);
    pitch = m1_w * 14 / 8;
    ubuf = RT_BUF | RT_UNC;
    rt_printf("M1 w=%d h=%d addr=%08x pitch=%d; mode %d, flag+0x28 %d, w/h sources +0x18 %d/%d +0x20 %d/%d +0x2c %d/%d +0x34 %d/%d\n",
              m1_w, m1_h, a, pitch, MEM(RT_PATH), MEM(RT_PATH + 0x28), MEM(RT_PATH + 0x18), MEM(RT_PATH + 0x1c),
              MEM(RT_PATH + 0x20), MEM(RT_PATH + 0x24), MEM(RT_PATH + 0x2c), MEM(RT_PATH + 0x30), MEM(RT_PATH + 0x34),
              MEM(RT_PATH + 0x38));
    if (!m1_w || !m1_h || pitch * m1_h > RT_BUFLEN || (a & ~RT_UNC) != RT_BUF || m1_h < 64)
    { rt_printf("MEM1 not active or size unexpected, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: no LV raw"); goto done; }

    for (int i = 0; i < 5; i++) orig[i] = MEM(PK_CFG + 4 * i);
    reg0 = MEM(PK_REG);
    w0 = MEM(PK_WSRC);
    rt_printf("pack cfg {%d,%d,%d,%d,%d} -> composed %08x; unit reg %08x (masked %08x)\n", orig[0], orig[1], orig[2],
              orig[3], orig[4], pk_compose(orig), reg0, reg0 & PK_MASK);
    if (orig[0] != 0 || orig[1] != 1 || orig[2] != 2 || orig[3] != 1 || orig[4] != 0)
    { rt_printf("cfg is not Canon's default {0,1,2,1,0}: not touching it, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: cfg unexpected"); goto done; }
    /* the width trick needs: this mode takes w/h from +0x18/+0x1c (FUN_e0638b7a: mode < 0x14 and not the
     * alternate source), and that equals the active width */
    uint32_t md = MEM(RT_PATH);
    int wsrc_ok = md < 0x14 && (MEM(RT_PATH + 0x28) == 0 || (md != 0xb && md != 0xf && md != 9)) && w0 == m1_w;
    rt_printf("width source +0x18 = %d: %s\n", w0, wsrc_ok ? "usable for 12/10-bit line length" : "NOT usable, 14-bit settings only");

    band = malloc(PK_ROWS * pitch);
    f = FIO_CreateFile(PK_FILE);
    if (!band || !f) { rt_printf("no buffer/file, abort\n"); snprintf(rt_last, sizeof(rt_last), "pack: no buffer"); goto done; }
    uint32_t row0 = (m1_h / 2) & ~1;
    rt_flush();

    for (int c = 0; c <= COUNT(combos); c++)
    {
        int restore = c == COUNT(combos);
        int bpp = restore ? 14 : combos[c].bpp;
        uint32_t f2 = restore ? orig[2] : combos[c].f2, f3 = restore ? orig[3] : combos[c].f3;
        uint32_t lb = m1_w * bpp / 8, wset = w0;
        if (bpp != 14)
        {
            if (!wsrc_ok) { rt_printf("setting %d (%d-bit): skipped\n", c, bpp); continue; }
            wset = (lb * 8 + 13) / 14;
            if (((wset * 14) >> 3) != lb || (m1_w * bpp) % 8)
            { rt_printf("setting %d (%d-bit): no width gives %d bytes, skipped\n", c, bpp, lb); continue; }
        }
        uint32_t rb = MEM(PK_REG);
        uint32_t old = cli();
        MEM(PK_WSRC) = wset;
        MEM(PK_CFG + 8) = f2;
        MEM(PK_CFG + 12) = f3;
        sei(old);
        changed = 1;
        uint32_t cfg[5] = { orig[0], orig[1], f2, f3, orig[4] };
        msleep(250);                    /* ~7 frames at 29.97p */
        uint32_t ra = MEM(PK_REG);
        uint32_t src = ubuf + row0 * lb;
        uint32_t h1 = pk_hash((uint8_t *)src, PK_ROWS * lb);
        msleep(120);
        uint32_t h2 = pk_hash((uint8_t *)src, PK_ROWS * lb);
        memcpy(band, (void *)src, PK_ROWS * lb);
        struct pk_hdr h = { { 'P', 'K', 'T', '2' }, c, cfg[1], cfg[4], rb, (rb & ~PK_MASK) | pk_compose(cfg), ra, m1_w,
                            m1_h, lb, row0, PK_ROWS, get_ms_clock(), f2, f3, bpp, wset, h1 != h2 };
        FIO_WriteFile(f, &h, sizeof(h));
        FIO_WriteFile(f, band, PK_ROWS * lb);
        nb++;
        rt_printf("band %d: f2=%d f3=%d (%d-bit test%s), width field %d -> line %d bytes; reg before %08x after 250 ms %08x "
                  "(expected %08x); M1 w now %d; frames %s\n", c, f2, f3, bpp, restore ? ", restored" : "", wset, lb, rb,
                  ra, h.reg_set, MEM(RT_PATH + 0x10), h1 != h2 ? "updating" : "NOT updating (stalled?)");
        rt_flush();
        if (restore) changed = 0;
    }
    snprintf(rt_last, sizeof(rt_last), "pack: %d bands", nb);

done:
    if (changed)                        /* aborted mid-way: put Canon's values back */
    {
        uint32_t old = cli();
        MEM(PK_WSRC) = w0; MEM(PK_CFG + 8) = orig[2]; MEM(PK_CFG + 12) = orig[3];
        sei(old);
        rt_printf("restored cfg after an abort\n");
    }
    if (reg0) rt_printf("final: unit reg %08x (start %08x), width field %d (start %d), M1 w %d\n", MEM(PK_REG), reg0,
                        MEM(PK_WSRC), w0, MEM(RT_PATH + 0x10));
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

/* ---- v18: Movie > RAW video (R): the user front end for REC-button clips ---- */
/* data rate in MiB/s x10 at 23.976p (fps30 = 0) or 29.97p; full frame = the FHD frame (4K full can't be recorded) */
static uint32_t rr_rate10(int sel, int bpp, int fps30)
{
    uint32_t w = rt_crop_wh[sel][0], h = rt_crop_wh[sel][1];
    if (!w) { w = 2304; h = 1290; }
    uint64_t b = (uint64_t)w * h * bpp / 8;
    return (uint32_t)(b * 10 * (fps30 ? 30000 : 24000) / 1001 / (1024 * 1024));
}

static MENU_UPDATE_FUNC(rr_main_update)
{
    if (!rr_on) return;
    if (rt_crop_sel < 0 || rt_crop_sel >= COUNT(rt_crop_wh)) rt_crop_sel = 0;
    int bpp = rt_fmt_bpp[rt_fmt_sel ? rt_fmt_sel : 1];
    if (rt_crop_sel == RT_CROP_FULL) MENU_SET_VALUE("Full frame, %d-bit", bpp);
    else MENU_SET_VALUE("%dx%d %d-bit", rt_crop_wh[rt_crop_sel][0], rt_crop_wh[rt_crop_sel][1], bpp);
    if (rr_active) MENU_SET_RINFO("REC");
}

static MENU_UPDATE_FUNC(rr_rate_update)
{
    if (rt_crop_sel < 0 || rt_crop_sel >= COUNT(rt_crop_wh)) rt_crop_sel = 0;
    int bpp = rt_fmt_bpp[rt_fmt_sel ? rt_fmt_sel : 1];
    uint32_t r24 = rr_rate10(rt_crop_sel, bpp, 0), r30 = rr_rate10(rt_crop_sel, bpp, 1), card = rr_card10;
    MENU_SET_VALUE("%d / %d MiB/s", (r24 + 5) / 10, (r30 + 5) / 10);
    MENU_SET_RINFO("card %d", (card + 5) / 10);
    if (r24 > card)
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Too fast for the card (~%d MiB/s) even at 23.98p: frames will be lost.", card / 10);
    else if (r30 > card)
        MENU_SET_WARNING(MENU_WARN_ADVICE, "23.98p only: 29.97p needs %d MiB/s, the card writes ~%d.", (r30 + 5) / 10, card / 10);
    else if (r30 * 100 > card * 95)
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Close to the card's speed at 29.97p: long clips may lose frames.");
}

/* Crop marker (2026-10-30): a frame on the live view around the area the raw recorder will record. Shown in movie
 * mode (PROP_LV_MOVIE_SELECT 2) with RAW video ON, Global Draw on and the ML menu closed. The crop is placed like
 * the recorder does (centred in the MEM1 frame, x on a 64-px grid), then mapped from the MEM1 active area (optical
 * black removed) onto the LV image. MEM1 size from the Mem1Path struct when it holds a known size, else FHD. */
static CONFIG_INT("r.rawrec.mark", rr_mark, 1);
static int rr_mark_4k_guess;            /* 1 = MEM1 reported 4K, 0 = FHD (reported or assumed) */
static int rr_mark_assumed;             /* 1 = MEM1 size not readable, FHD assumed */

static int rr_mark_rect(int *bx, int *by, int *bw, int *bh)
{
    uint32_t m1w = MEM(RT_PATH + 0x10), m1h = MEM(RT_PATH + 0x14);
    int obl, obt;
    rr_mark_assumed = 0;
    if (m1w == 4256 && m1h == 2214) { obl = 144; obt = 46; rr_mark_4k_guess = 1; }
    else
    {
        if (!(m1w == 2304 && m1h == 1290)) rr_mark_assumed = 1;
        m1w = 2304; m1h = 1290; obl = RT_OB_LEFT; obt = RT_OB_TOP; rr_mark_4k_guess = 0;
    }
    int sel = rt_crop_sel;
    if (sel < 0 || sel >= COUNT(rt_crop_wh)) sel = 0;
    uint32_t cw, ch;
    if (sel == RT_CROP_FULL) { cw = m1w; ch = m1h & ~1; }
    else { cw = MIN(rt_crop_wh[sel][0], m1w) & ~63; ch = MIN(rt_crop_wh[sel][1], m1h) & ~1; }
    uint32_t x0 = ((m1w - cw) / 2) & ~63, y0 = ((m1h - ch) / 2) & ~1;
    int aw = m1w - obl, ah = m1h - obt;
    int ax0 = MAX((int)x0 - obl, 0), ay0 = MAX((int)y0 - obt, 0);
    int ax1 = MIN((int)(x0 + cw) - obl, aw), ay1 = MIN((int)(y0 + ch) - obt, ah);
    struct vram_info *v = get_yuv422_vram();   /* also refreshes the overlay geometry */
    if (!v || !vram_lv.width || !vram_lv.height) return 0;
    int lx0 = ax0 * vram_lv.width / aw, lx1 = ax1 * vram_lv.width / aw;
    int ly0 = ay0 * vram_lv.height / ah, ly1 = ay1 * vram_lv.height / ah;
    *bx = LV2BM_X(lx0); *by = LV2BM_Y(ly0);
    *bw = LV2BM_X(lx1) - *bx; *bh = LV2BM_Y(ly1) - *by;
    return *bw > 8 && *bh > 8;
}

static void rr_mark_draw(int color, int x, int y, int w, int h)
{
    bmp_draw_rect(color, x, y, w, h);
    bmp_draw_rect(color, x + 1, y + 1, w - 2, h - 2);
}

static void rr_mark_task(void *unused)
{
    int drawn = 0, dx = 0, dy = 0, dw = 0, dh = 0, tick = 0;
    for (;;)
    {
        msleep(250);
        if (r_bisect_bg == 1 || r_bisect_bg == 3) { msleep(750); continue; }
        tick++;
        int want = rr_mark && rr_on && lv && lv_movie_select == LVMS_ENABLE_MOVIE && get_global_draw()
                   && !gui_menu_shown() && DISPLAY_IS_ON;
        int x = 0, y = 0, w = 0, h = 0;
        if (want) want = rr_mark_rect(&x, &y, &w, &h);
        if (!want)
        {
            if (drawn && !gui_menu_shown()) { rr_mark_draw(COLOR_EMPTY, dx, dy, dw, dh); drawn = 0; }
            continue;
        }
        if (drawn && (x != dx || y != dy || w != dw || h != dh)) { rr_mark_draw(COLOR_EMPTY, dx, dy, dw, dh); drawn = 0; }
        if (!drawn || tick % 8 == 0)    /* redraw every 2 s: overlay wipes / Canon redraws may erase it */
        {
            rr_mark_draw(COLOR_YELLOW, x, y, w, h);
            dx = x; dy = y; dw = w; dh = h; drawn = 1;
        }
    }
}

static MENU_UPDATE_FUNC(rr_mark_update)
{
    if (!rr_mark) return;
    if (rr_mark_assumed)
        MENU_SET_WARNING(MENU_WARN_INFO, "Movie size not known yet: frame assumes FHD (exact after the first clip).");
    else if (rr_mark_4k_guess)
        MENU_SET_WARNING(MENU_WARN_INFO, "4K: frame position not verified on the camera yet.");
}

static struct menu_entry rr_menu[] = {
    {
        .name     = "RAW video (R)",
        .priv     = &rr_on,
        .max      = 1,
        .update   = rr_main_update,
        .help     = "ON: the REC button records raw MLV (no audio) instead of Canon video.",
        .help2    = "Movie mode, ML menu closed. Clips: M<day>-<hhmm>.MLV at the card root. Open in MLV App.",
        .children = (struct menu_entry[]) {
            {
                .name    = "Resolution",
                .priv    = &rt_crop_sel,
                .max     = 6,
                .choices = CHOICES("1920x1080", "1920x960", "1920x800", "Full frame", "2560x1080", "2560x1440", "2880x1216"),
                .help    = "2560 and 2880 need Canon movie size 4K (1:1 pixels, narrower view). In FHD they shrink",
                .help2   = "to 2304 wide. Full frame: FHD 2304x1290 (4K full frame is too fast to record).",
            },
            {
                .name    = "Bit depth",
                .priv    = &rt_fmt_sel,
                .min     = 1,
                .max     = 3,
                .choices = CHOICES("14-bit", "12-bit", "10-bit"),   /* v18.1: ML shows choices[value - min] */
                .help    = "Fewer bits = lower data rate. 10-bit keeps slightly less shadow detail.",
            },
            {
                .name    = "Audio",
                .priv    = &rr_audio,
                .max     = 1,
                .help    = "Records the camera's mic into the MLV (48 kHz stereo, MLV App plays it).",
                .help2   = "Needs Canon Sound recording Auto or Manual. Camera-verified: sync within a frame.",
            },
            {
                .name    = "Audio meters",
                .priv    = &rr_meter,
                .max     = 1,
                .help    = "L/R level bars next to the REC status while a clip with audio records: -48..0 dBFS, peak, CLIP.",
                .help2   = "Red from -3 dB; CLIP lights for 2 s after a sample hits full scale.",
            },
            {
                .name    = "Crop marker",
                .priv    = &rr_mark,
                .max     = 1,
                .update  = rr_mark_update,
                .help    = "Yellow frame on the live view around the area the raw recorder records.",
                .help2   = "Movie mode, RAW video ON, Global Draw ON.",
            },
            {
                .name    = "Data rate",
                .update  = rr_rate_update,
                .help    = "Needed at 23.98p / 29.97p. Card: write speed measured during the last clip.",
                .help2   = "Hardware-tested: 2560x1440 10-bit 23.98p, 2560x1080 10-bit 29.97p (40 s each).",
            },
            MENU_EOL,
        },
    },
};

/* v20: Shoot > LV silent pictures */
static MENU_UPDATE_FUNC(sp_menu_update)
{
    if (!sp_mode) return;
    if (lv_movie_select != LVMS_ENABLE_MOVIE)
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Movie mode only. Canon movie size 4K gives 4256x2214 (1:1, ~1.7x crop).");
    else if (rr_on)
        MENU_SET_WARNING(MENU_WARN_INFO, "RAW video is ON too: REC records raw only while no silent session is armed.");
}

static struct menu_entry sp_menu[] = {
    {
        .name     = "LV silent pictures",
        .priv     = &sp_mode,
        .max      = 2,
        .choices  = CHOICES("OFF", "Burst", "Pre-capture"),
        .update   = sp_menu_update,
        .help     = "DNG stills from the live-view raw feed, no shutter. Movie mode, Canon size 4K: 4256x2214.",
        .help2    = "Half-press arms (~2 s), full press = frames while held (~12 max). Pre-capture: frames before too.",
        .children = (struct menu_entry[]) {
            {
                .name    = "Bit depth",
                .priv    = &sp_bits,
                .max     = 1,
                .choices = CHOICES("10-bit", "12-bit"),
                .help    = "10-bit: whole 4K frame copied in ~20 ms, ahead of the sensor. 12-bit takes ~30 ms:",
                .help2   = "as slow as the sensor readout, so 4K frames tear (camera-tested). 12-bit only for FHD.",
            },
            {
                .name    = "Frames before press",
                .priv    = &sp_pre_sel,
                .max     = 3,
                .choices = CHOICES("3", "5", "7", "9"),
                .help    = "Pre-capture: frames kept from before the full press (30 fps: 5 = ~0.17 s).",
                .help2   = "At most all frames in memory minus 2 (~8-10 at 4K).",
            },
            {
                .name    = "Arm",
                .priv    = &sp_arm_sel,
                .max     = 1,
                .choices = CHOICES("Half-press", "Always"),
                .help    = "Always: armed whenever movie live view is up (ML overlays pause, REC raw is blocked).",
                .help2   = "Pictures still need half + full press; Always only saves the ~2 s arming wait.",
            },
            {
                .name    = "Timed test",
                .select  = sp_sim_select,
                .help    = "No buttons: close the menu, it arms, then 'presses' for 1 s. Opening the menu cancels.",
                .help2   = "Pictures go to S<day><hhmm>/ at the card root; log ML/RECTEST.TXT.",
            },
            MENU_EOL,
        },
    },
};

static struct menu_entry rt_menu[] = {
    {
        .name   = "LV record test",
        .select = menu_open_submenu,
        .help   = "Raw record test: frame-synced EDMAC crop copy + batched card writes",
        .children = (struct menu_entry[]) {
            {
                .name    = "Crop",
                .priv    = &rt_crop_sel,
                .max     = 6,
                .choices = CHOICES("1920x1080", "1920x960", "1920x800", "Full frame", "2560x1080", "2560x1440", "2880x1216"),
                .help    = "Full frame: whole LV raw frame (FHD movie 2304x1290, 4K movie 4256x2214, incl. black borders)",
                .help2   = "2560/2880 crops need Canon movie size 4K (1:1 pixels); 2880x1216: 10-bit 23.98p only",
            },
            {
                .name    = "Duration",
                .priv    = &rt_dur_sel,
                .max     = 2,
                .choices = CHOICES("10 s", "20 s", "40 s"),
            },
            {
                .name    = "Format",
                .priv    = &rt_fmt_sel,
                .max     = 3,
                .choices = CHOICES("BIN 14-bit", "MLV 14-bit", "MLV 12-bit", "MLV 10-bit"),
                .help    = "BIN: Canon's bit order (PC converter). MLV: camera writes .MLV (pack unit set to ML order).",
            },
            {
                .name    = "REC diagnostics",
                .priv    = &rr_diag,
                .max     = 1,
                .help    = "ON: REC-button clips also write test files: ML/RTnn.IDX/.SYN, ML/RRKEYS.TXT, the full",
                .help2   = "log and USB copies in DCIM. REC button switch: Movie > RAW video (R).",
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
                .help   = "v14.1: tries pack-unit bit 0, 12-bit, 10-bit on the LV raw; bands to ML/PACKTEST.BIN.",
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
 * to 6 lines "<crop height|full> <seconds> [auto|irq|timer] [bin|mlv14|mlv12|mlv10]", e.g. "1080 20 auto mlv12",
 * or "pack" (v14 pack test);
 * '#' starts a comment. Waits up to 30 s for live view; without it (e.g. USB
 * cable in: PC-connection screen) nothing is used up. RTAUTO.POS is advanced
 * before the plan runs, so a freeze + battery pull skips that plan instead of
 * looping. After the last plan RTAUTO.TXT is renamed to RTAUTO.DON. */
#define RT_AUTO_CFG "ML/RTAUTO.TXT"
#define RT_AUTO_POS "ML/RTAUTO.POS"
#define RT_AUTO_DON "ML/RTAUTO.DON"

static int rt_auto_skipped;
static int rt_auto_parse(const char *t, int len, int (*runs)[5], int max)
{
    int n = 0, i = 0;
    while (i < len && n < max)
    {
        int v[5] = { -1, -1, -1, 0, 0 }, k = 0, vw = 0;
        while (i < len && t[i] != '\n')
        {
            char c = t[i];
            if (c == '#') { while (i < len && t[i] != '\n') i++; break; }
            if (c >= '0' && c <= '9' && k < 2)
            {
                int x = 0;
                while (i < len && t[i] >= '0' && t[i] <= '9') x = x * 10 + (t[i++] - '0');
                if (k == 0 && i + 1 < len && (t[i] == 'x' || t[i] == 'X') && t[i + 1] >= '0' && t[i + 1] <= '9')
                {                       /* v17: "WxH" crop, e.g. 2560x1440 */
                    vw = x; x = 0; i++;
                    while (i < len && t[i] >= '0' && t[i] <= '9') x = x * 10 + (t[i++] - '0');
                }
                v[k++] = x;
                continue;
            }
            if (k == 0 && (c == 'f' || c == 'F'))  /* "full": whole MEM1 frame */
            {
                v[k++] = 9999;
                while (i < len && t[i] != '\n' && t[i] != ' ' && t[i] != '\t' && t[i] != '\r') i++;
                continue;
            }
            if (k == 0 && (c == 'p' || c == 'P'))  /* "pack": pack test */
            {
                k = 9;
                while (i < len && t[i] != '\n') i++;
                break;
            }
            if (k >= 2 && (c == 'o' || c == 'O'))   /* "osd": REC status on screen during this run */
            {
                v[4] = 1;
                while (i < len && t[i] != '\n' && t[i] != ' ' && t[i] != '\t' && t[i] != '\r') i++;
                if (k < 4) k = 4;
                continue;
            }
            if ((k == 2 || k == 3) && (c == 'b' || c == 'B' || c == 'm' || c == 'M'))   /* format: bin, mlv14, mlv12 */
            {
                int twelve = 0, ten = 0;
                while (i < len && t[i] != '\n' && t[i] != ' ' && t[i] != '\t' && t[i] != '\r')
                {
                    if (t[i] == '1' && i + 1 < len && t[i + 1] == '2') twelve = 1;
                    if (t[i] == '1' && i + 1 < len && t[i + 1] == '0') ten = 1;
                    i++;
                }
                v[3] = (c == 'b' || c == 'B') ? 0 : ten ? 3 : twelve ? 2 : 1;
                k = 4;
                continue;
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
        if (k == 9) { runs[n][0] = -1; runs[n][1] = runs[n][2] = runs[n][3] = runs[n][4] = 0; n++; continue; }
        int cs = v[0] == 9999 && !vw ? RT_CROP_FULL : -1;
        for (int q = 0; q < COUNT(rt_crop_wh) && cs < 0 && v[0] > 0; q++)   /* "1080" = 1920x1080; v17: "WxH" */
            if (q != RT_CROP_FULL && rt_crop_wh[q][0] == (vw ? vw : 1920) && rt_crop_wh[q][1] == v[0]) cs = q;
        int ds = v[1] == 10 ? 0 : v[1] == 20 ? 1 : v[1] == 40 ? 2 : -1;
        if (k < 2 || cs < 0 || ds < 0) { rt_auto_skipped++; continue; }
        runs[n][0] = cs; runs[n][1] = ds; runs[n][2] = v[2] < 0 ? SYNC_AUTO : v[2]; runs[n][3] = v[3]; runs[n][4] = v[4];
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
    int runs[6][5];
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
        rt_crop_sel = runs[i][0]; rt_dur_sel = runs[i][1]; rt_sync_sel = runs[i][2]; rt_fmt_sel = runs[i][3];
        rt_osd_run = runs[i][4];
        rt_task(0);                     /* synchronous; clears rt_busy */
        rt_osd_run = 0;
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
    task_create("rr_status", 0x1f, 0x1000, rr_status_task, 0);
    task_create("rr_mark", 0x1f, 0x1000, rr_mark_task, 0);
    menu_add("Movie", rr_menu, COUNT(rr_menu));       /* v18: user front end */
    menu_add("Debug", rt_menu, COUNT(rt_menu));
    menu_add("Shoot", sp_menu, COUNT(sp_menu));     /* v20 */
    task_create("sp_watch", 0x1f, 0x1000, sp_watch_task, 0);
    task_create("rt_auto", 0x1e, 0x2000, rt_auto_task, 0);
}

INIT_FUNC("rectest", rt_init);
