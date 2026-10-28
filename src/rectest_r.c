/* riz/eosr-dmlog v12: sustained LV raw record test (EOS R 1.8.0).
 * Personal diagnostic build, not for upstream.
 *
 * v11 proved Canon's MemoryToMemoryEsub1 EDMAC copies a 1920x1080 14-bit
 * crop out of the MEM1 raw buffer in ~5 ms, and that one setup serves many
 * copies. The burst was then limited by card writes of one 3.6 MB frame each
 * (79 MiB/s). This test records for 10/20/40 s the way a recorder would:
 *  - buffers: up to 10 contiguous shoot_malloc chunks of K frame slots each
 *    (K chosen so a chunk is ~16 MB; slots padded to 4 KiB), as mlv_lite does;
 *  - capture task: waits for each new frame (bottom crop row changes), sleeps
 *    between frames once the frame interval is known, copies the crop with
 *    EDMAC (one setup for the whole run) into the next free slot; when no
 *    slot is free the frame is dropped and counted;
 *  - writer task (higher priority): writes each full chunk with ONE
 *    FIO_WriteFile call to ML/RTnn.BIN (new part file every ~2 GB).
 * Logs ML/RECTEST.TXT (summary, timeline, per-write times) and ML/RTnn.IDX
 * (8 bytes per frame: time, DMA time, latency, flags).
 * Every DMA wait has a timeout; on a timeout EDMAC is reset and released and
 * the run stops. No ROM writes, no property changes.
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
#else
#include "rt_host.h"
#endif

#define ERSC_MGR       0x10924
#define PWR_LIST_VCOPY ((const void *)0xE0F2673C)
#define RT_RD_PORT     0x2E
#define RT_WR_PORT     0x0D

#define RT_PATH    0x133CC              /* Mem1Path struct */
#define RT_BUF     0x28EEA940
#define RT_BUFLEN  0xFB9D90
#define RT_UNC     0x40000000

#define RT_MAXCH      10
#define RT_CHUNK_TGT  (16 * 1024 * 1024)
#define RT_SPLIT      0x7F000000u       /* new part file before ~2.1 GB */
#define RT_MAXFR      3000
#define RT_MAXW       1024
#define RT_MAXTL      480               /* 250 ms timeline samples */
#define RT_LOGLEN     0x10000
#define RT_TIMEOUT    500               /* ms per DMA wait */

/* per-frame flags in the .IDX */
#define F_DROP   1                      /* no free slot: not recorded */
#define F_LATE   2                      /* first poll after a sleep already saw the new frame */
#define F_TEAR   4                      /* top crop row changed before the copy started (not late) */
#define F_TOPCHG 8                      /* top crop row changed during the copy (informational) */
#define F_BOTCHG 16                     /* bottom crop row changed before the copy ended: torn */

static int rt_crop_sel = 0;             /* 1920x1080, 1920x960, 1920x800 */
static int rt_dur_sel = 1;              /* 10, 20, 40 s */
static const int rt_crop_h[] = { 1080, 960, 800 };
static const int rt_dur_s[] = { 10, 20, 40 };

static volatile int rt_busy = 0;
static int rt_runs = 0;
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

/* ---- EDMAC m2m (sequence as Canon's EFsCropVcopy; verified on camera in v11) ---- */
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
    *dma_us = (rt_cbr_t - t0) & 0xFFFFF;
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
        uint32_t t = rt_us();
        struct memSuite *s = shoot_malloc_suite_contig(want);
        t = rt_us() - t;
        if (!s) { rt_printf("chunk %d: shoot_malloc_suite_contig(%d) failed (%d us)\n", i, want, t); break; }
        struct memChunk *c = GetFirstChunkFromSuite(s);
        uint32_t a = (uint32_t)GetMemoryAddressOfMemoryChunk(c);
        uint32_t sz = GetSizeOfMemoryChunk(c);
        ch[i].suite = s;
        ch[i].base = ((uint32_t)UNCACHEABLE(a) + 4095) & ~4095;
        ch[i].state = CH_FREE; ch[i].n = 0;
        nch++;
        rt_printf("chunk %d: %08x size %d (%d us)\n", i, a, sz, t);
        if (sz < want) { rt_printf("chunk %d smaller than requested, dropping it\n", i); nch--; shoot_free_suite(s); ch[i].suite = 0; break; }
    }
    RT_SYNC_CACHES();                   /* no dirty cache lines may later land on DMA-written data */
}

/* ---- live frame detection ---- */
static uint32_t m1_w, m1_h, pitch, ubuf;

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

static uint32_t rt_probe_stable(uint32_t row)
{
    uint32_t a = rt_probe(row);
    uint32_t t0 = rt_us(), t = t0;
    for (;;)
    {
        uint32_t now = rt_us();
        if (now - t >= 50 || now - t0 > 2000) break;
        uint32_t b = rt_probe(row);
        if (b != a) { a = b; t = now; }
    }
    return a;
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

static void rt_part_name(char *out, int run, int part)
{
    if (part) snprintf(out, 32, "ML/RT%02d.B%02d", run, part);
    else snprintf(out, 32, "ML/RT%02d.BIN", run);
}

static int rt_run_id;
static uint32_t rt_t0;

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
struct rt_frame { uint32_t t_us; uint16_t dma_us; uint8_t lat; uint8_t flags; };   /* lat: 0.1 ms units, 255 = unknown */
struct rt_tl { uint16_t t_ms4; uint16_t frames; uint16_t drops; uint16_t used; uint32_t mb_written; };
static struct rt_frame *fl;
static struct rt_tl *tl;

static void sort_u32(uint32_t *v, int n)
{
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { uint32_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
}

/* watch 9 frames without copying: frame interval (median of 8) and the time
 * from the top crop row changing to the bottom crop row changing (readout) */
static uint32_t rd_meas[8];
static int nrd;

static uint32_t rt_warmup(uint32_t r_top, uint32_t r_bot)
{
    uint32_t iv[8], niv = 0, t_prev = 0, top_t = 0;
    uint32_t prevb = rt_probe_stable(r_bot), prevt = rt_probe(r_top), t0 = rt_us();
    nrd = 0;
    while (niv < 8 && rt_us() - t0 < 2000000)
    {
        uint32_t vt = rt_probe(r_top);
        if (vt != prevt) { prevt = vt; top_t = rt_us(); }
        if (rt_probe(r_bot) == prevb) continue;
        prevb = rt_probe_stable(r_bot);
        uint32_t td = rt_us();
        if (top_t && nrd < 8) rd_meas[nrd++] = td - top_t;
        top_t = 0;
        if (t_prev) iv[niv++] = td - t_prev;
        t_prev = td;
    }
    if (niv < 8) return 0;
    sort_u32(iv, 8);
    return (iv[3] + iv[4]) / 2;
}

static void rt_task(void *unused)
{
    rt_logbuf = malloc(RT_LOGLEN);
    fl = malloc(sizeof(*fl) * RT_MAXFR);
    wl = malloc(sizeof(*wl) * RT_MAXW);
    tl = malloc(sizeof(*tl) * RT_MAXTL);
    rt_loglen = 0;
    w_file = 0; w_done = 1; nch = 0;
    int lv_raw = 0;
    if (!rt_logbuf || !fl || !wl || !tl) { NotifyBox(6000, "Record test: out of memory"); goto out; }
    if (!rt_dsem) rt_dsem = create_named_semaphore("rt_dma", SEM_CREATE_LOCKED);
    if (!rt_wsem) rt_wsem = create_named_semaphore("rt_wr", SEM_CREATE_LOCKED);

    uint32_t crop_w = 1920, crop_h = rt_crop_h[rt_crop_sel];
    int dur_s = rt_dur_s[rt_dur_sel];
    for (rt_run_id = 0; rt_run_id < 99; rt_run_id++)
    {
        char n[32]; rt_part_name(n, rt_run_id, 0);
        if (!is_file(n)) break;
    }
    rt_printf("start v12 record test run %d: crop %dx%d, %d s, t=%d lv=%d, card free %d MB\n", rt_run_id, crop_w, crop_h,
              dur_s, get_ms_clock(), lv ? 1 : 0, get_free_space_32k(get_shooting_card()) / 32);

    /* msleep granularity (the capture loop sleeps between frames) */
    uint32_t sl[5];
    for (int i = 0; i < 5; i++) { uint32_t t = rt_us(); msleep(1); sl[i] = rt_us() - t; }
    sort_u32(sl, 5);
    uint32_t ms1 = sl[2];
    rt_printf("msleep(1) took %d %d %d %d %d us\n", sl[0], sl[1], sl[2], sl[3], sl[4]);

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
    uint32_t rb = crop_w * 14 / 8;
    uint32_t src = ubuf + y0 * pitch + x0 * 14 / 8;
    uint32_t r_top = y0, r_bot = y0 + crop_h - 1;
    frame_bytes = rb * crop_h;
    slot_bytes = (frame_bytes + 4095) & ~4095;
    kslots = MAX(1, RT_CHUNK_TGT / slot_bytes);
    rt_printf("crop %dx%d at (%d,%d), %d bytes/frame, slot %d, %d slots/chunk (%d bytes per write)\n",
              crop_w, crop_h, x0, y0, frame_bytes, slot_bytes, kslots, kslots * slot_bytes);
    rt_alloc_chunks();
    rt_printf("buffer: %d chunks = %d frames (%d MB)\n", nch, nch * kslots, nch * kslots * slot_bytes >> 20);
    rt_flush();
    if (nch < 2) { NotifyBox(6000, "Record test: not enough memory (%d)", nch); goto done; }

    /* quick sanity copy (crop of the live frame into chunk 0), as in v11 */
    rt_begin();
    {
        uint32_t us;
        if (rt_copy(src, pitch, ch[0].base, rb, crop_h, &us))
        { rt_printf("!! first EDMAC copy timed out\n"); rt_fail(); NotifyBox(6000, "Record test: DMA timeout"); goto done; }
        rt_printf("first copy: dma %d us\n", us);
    }

    uint32_t est = rt_warmup(r_top, r_bot);
    if (!est || est < 8000) { rt_printf("no steady LV frames (interval %d us), abort\n", est); rt_end(); NotifyBox(6000, "Record test: no LV frames"); goto done; }
    rt_printf("frame interval %d us (%d.%02d fps), median of 8\n", est, 100000000 / est / 100, 100000000 / est % 100);

    rt_part_name(w_name, rt_run_id, 0);
    w_file = FIO_CreateFile(w_name);
    if (!w_file) { rt_printf("cannot create %s\n", w_name); goto done; }
    w_part = 0; w_fsize = 0; w_bytes = 0; w_n = 0; w_stop = 0; w_err = 0; w_done = 0;
    rt_t0 = rt_us();
    task_create("rt_writer", 0x1c, 0x1000, rt_writer, 0);
    rt_flush();

    /* capture loop */
    int nfr = 0, frames = 0, drops = 0, missed = 0, late = 0, tears = 0, topchg = 0, botchg = 0, ntl = 0;
    int cur = -1, next_fill = 0, first_drop_ms = -1, abort = 0;
    uint32_t prevb = rt_probe_stable(r_bot), top_same = rt_probe(r_top);
    uint32_t last_same = rt_us(), t_prev = 0, first_t = 0, next_tl = 0, max_used = 0;
    int after_sleep = 0;
    uint32_t dmax = 0, dsum = 0, wait0 = rt_us();
    while (!abort)
    {
        uint32_t now = rt_us();
        if (first_t && now - first_t > (uint32_t)dur_s * 1000000) break;
        if (w_err) { rt_printf("writer error %d, stopping\n", w_err); break; }
        if (now - (t_prev ? t_prev : wait0) > 1000000) { rt_printf("no new frame for 1 s, stopping\n"); break; }

        /* timeline every 250 ms */
        if (first_t && now - first_t >= next_tl && ntl < RT_MAXTL)
        {
            int used = 0;
            for (int i = 0; i < nch; i++) if (ch[i].state != CH_FREE) used += ch[i].n;
            tl[ntl].t_ms4 = (now - first_t) / 4000; tl[ntl].frames = frames; tl[ntl].drops = drops;
            tl[ntl].used = used; tl[ntl].mb_written = (uint32_t)(w_bytes >> 20); ntl++;
            next_tl += 250000;
        }

        /* sleep until shortly before the next frame is due */
        if (t_prev)
        {
            int32_t rem = (int32_t)(t_prev + est - 4000 - now);
            int n = (rem - (int32_t)ms1) / 1000;
            if (n >= 1) { msleep(n); after_sleep = 1; continue; }
        }

        uint32_t vb = rt_probe(r_bot);
        uint32_t vt = rt_probe(r_top);
        if (vb == prevb)
        {
            top_same = vt; last_same = rt_us(); after_sleep = 0;
            continue;
        }
        /* new frame complete */
        prevb = rt_probe_stable(r_bot);
        uint32_t td = rt_us();
        int flags = 0;
        uint32_t lat_us = td - last_same;
        if (after_sleep) { flags |= F_LATE; late++; }
        /* frames the loop never saw: frame number from the time since the first
         * frame vs frames seen (interval counting misses a frame whose bottom
         * row changed during a copy) */
        if (first_t)
        {
            int seq = (td - first_t + est / 2) / est;
            if (seq - nfr > missed) missed = seq - nfr;
        }
        if (!first_t) { first_t = td; next_tl = 0; }
        t_prev = td;

        /* slot */
        uint32_t dst = 0;
        if (cur < 0 && ch[next_fill].state == CH_FREE) { cur = next_fill; ch[cur].n = 0; ch[cur].state = CH_FILL; }
        if (cur >= 0) dst = ch[cur].base + ch[cur].n * slot_bytes;

        uint32_t us = 0;
        if (!dst)
        {
            flags |= F_DROP; drops++;
            if (first_drop_ms < 0) first_drop_ms = (td - first_t) / 1000;
        }
        else
        {
            uint32_t top0 = rt_probe(r_top);
            if (!(flags & F_LATE) && top0 != top_same) { flags |= F_TEAR; tears++; }
            if (rt_copy(src, pitch, dst, rb, crop_h, &us))
            {
                rt_printf("!! EDMAC timeout at frame %d, stopping\n", nfr);
                rt_fail();
                abort = 1;
                break;
            }
            if (rt_probe(r_top) != top0) { flags |= F_TOPCHG; topchg++; }
            if (rt_probe(r_bot) != prevb) { flags |= F_BOTCHG; botchg++; }
            frames++; dsum += us; if (us > dmax) dmax = us;
            if (++ch[cur].n == kslots)
            {
                ch[cur].state = CH_FULL;
                give_semaphore(rt_wsem);
                next_fill = (cur + 1) % nch;
                cur = -1;
            }
        }
        if (nfr < RT_MAXFR)
        {
            fl[nfr].t_us = td - first_t; fl[nfr].dma_us = MIN(us, 65535);
            fl[nfr].lat = (flags & F_LATE) ? 255 : MIN(lat_us / 100, 254); fl[nfr].flags = flags;
        }
        nfr++;
        int used = 0;
        for (int i = 0; i < nch; i++) if (ch[i].state != CH_FREE) used += ch[i].n;
        if ((uint32_t)used > max_used) max_used = used;
        top_same = rt_probe(r_top);
        last_same = rt_us();
        after_sleep = 0;
    }
    uint32_t dur_us = t_prev - first_t;

    /* hand over the partial chunk, stop the writer */
    if (cur >= 0) { if (ch[cur].n) { ch[cur].state = CH_FULL; } else ch[cur].state = CH_FREE; }
    uint32_t ts = rt_us();
    w_stop = 1;
    give_semaphore(rt_wsem);
    for (int i = 0; i < 3000 && !w_done; i++) msleep(10);
    uint32_t drain_ms = (rt_us() - ts) / 1000;
    if (w_done) { FIO_CloseFile(w_file); w_file = 0; }
    else rt_printf("writer still busy after 30 s; file left open, buffers kept\n");
    if (!abort) rt_end();

    /* summary */
    uint32_t wus = 0, wmax = 0, wmin = 0xFFFFFFFF;
    for (int i = 0; i < MIN(w_n, RT_MAXW); i++) { wus += wl[i].us; wmax = MAX(wmax, wl[i].us); wmin = MIN(wmin, wl[i].us); }
    uint32_t need10 = est ? (uint32_t)((uint64_t)frame_bytes * 10000000ULL / est / (1024 * 1024)) : 0;
    uint64_t wb = w_bytes;
    uint32_t card10 = wus ? (uint32_t)(wb * 10000000ULL / wus / (1024 * 1024)) : 0;
    uint32_t wall10 = dur_us ? (uint32_t)(wb * 10000000ULL / ((uint64_t)dur_us + drain_ms * 1000) / (1024 * 1024)) : 0;
    rt_printf("RESULT run %d: %dx%d, %d.%03d s, %d frames seen, %d recorded, %d dropped (first drop at %d ms), "
              "%d missed by the capture loop, %d written to card\n", rt_run_id, crop_w, crop_h, dur_us / 1000000,
              dur_us / 1000 % 1000, nfr, frames, drops, first_drop_ms, missed, (uint32_t)(wb / slot_bytes));
    rt_printf("RESULT data rate needed %d.%d MiB/s; card %d.%d MiB/s while writing, %d.%d MiB/s overall; "
              "%d writes, min %d max %d us; buffer peak %d of %d frames; drain %d ms\n",
              need10 / 10, need10 % 10, card10 / 10, card10 % 10, wall10 / 10, wall10 % 10,
              w_n, wmin, wmax, max_used, nch * kslots, drain_ms);
    rt_printf("RESULT copy dma avg %d max %d us; late %d, tear %d, topchg %d, botchg(torn) %d; writer err %d\n",
              frames ? dsum / frames : 0, dmax, late, tears, topchg, botchg, w_err);
    rt_printf("readout (top->bottom crop row) us:");
    for (int i = 0; i < nrd; i++) rt_printf(" %d", rd_meas[i]);
    rt_printf("\nfiles: %s (last part %d), %d MB total\n", w_name, w_part, (uint32_t)(wb >> 20));
    snprintf(rt_last, sizeof(rt_last), "RT%02d: %d rec, %d drop", rt_run_id, frames, drops);
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
    if (lv_raw) call("lv_save_raw", 0);
    if (rt_begun) rt_end();
    if (w_done) rt_free_chunks();
    rt_unlock();
    rt_printf("done t=%d\n", get_ms_clock());
    rt_flush();
    rt_runs++;
    NotifyBox(8000, "Record test: DONE %s", rt_last);
out:
    if (rt_logbuf) free(rt_logbuf);
    if (fl) free(fl);
    if (wl) free(wl);
    if (tl) free(tl);
    rt_logbuf = 0; fl = 0; wl = 0; tl = 0;
    info_led_blink(3, 100, 100);
    rt_busy = 0;
}

static MENU_SELECT_FUNC(rt_select)
{
    if (rt_busy) return;
    rt_busy = 1;
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
        .help   = "Sustained raw record test: EDMAC crop copy + batched card writes",
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
                .name   = "Start",
                .select = rt_select,
                .update = rt_update,
                .help   = "NOT recording. SET, close menu, wait for DONE (duration + ~5 s).",
                .help2  = "Writes ML/RTnn.BIN (~2 GB per 20 s). Freeze: pull battery.",
            },
            MENU_EOL,
        },
    },
};

static void rt_init(void *unused)
{
    menu_add("Debug", rt_menu, COUNT(rt_menu));
}

INIT_FUNC("rectest", rt_init);
