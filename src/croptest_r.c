/* riz/eosr-dmlog v10: LV crop copy test (EOS R 1.8.0). Personal diagnostic
 * build, not for upstream.
 *
 * Turns on Canon's lv_save_raw debug tap (same as the v9 MEM1 test, which
 * puts a 14-bit packed LV raw frame at 0x28EEA940 every frame), then:
 *  1. Frame timing: polls 4 probe rows (frame top, crop top, crop bottom,
 *     frame bottom) for 600 ms and logs when the DMA rewrites each one.
 *  2. Copy speed: memcpy of centred crops, row by row, out of the uncached
 *     buffer into a cached or uncached destination; plus the cached source
 *     alias and one contiguous memcpy as references.
 *  3. 3 s copy+write burst at the recording crop (1920x1080, or what fits):
 *     this task copies each new frame into a ring of buffers, a writer task
 *     writes them to ML/CROPREC.BIN. Counts frames, missed frames, tears
 *     (crop top row rewritten during the copy) and ring overruns.
 * Then lv_save_raw(0). Results: ML/CROPTEST.TXT (flushed after each phase).
 * Reads RAM and writes card files only; no ROM writes, no Canon property
 * changes besides the debug switch, which is reset at the end.
 */
#include "dryos.h"
#include "menu.h"
#include "propvalues.h"
#include "timer.h"
#include "mem.h"

#define CT_PATH    0x133CC      /* Mem1Path struct (+0x10 w, +0x14 h, +0x58 addr) */
#define CT_BUF     0x28EEA940   /* MEMNAVI 0x10007, cached alias */
#define CT_BUFLEN  0xFB9D90
#define CT_UNC     0x40000000

#define CT_NBUF    4
#define CT_BURST_US 3000000
#define CT_MAXEV   512
#define CT_MAXFR   200
#define CT_LOGLEN  0x8000

static volatile int ct_busy = 0;
static int ct_runs = 0;

static char *ct_logbuf;
static int ct_loglen;

static uint32_t ct_w, ct_h, ct_pitch, ct_ubuf;

static void ct_printf(const char *fmt, ...)
{
    if (!ct_logbuf || ct_loglen > CT_LOGLEN - 200) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(ct_logbuf + ct_loglen, CT_LOGLEN - ct_loglen, fmt, ap);
    va_end(ap);
    if (n > 0) ct_loglen += MIN(n, CT_LOGLEN - ct_loglen - 1);
}

/* append the log so far to the card; a freeze later leaves this much */
static void ct_flush(void)
{
    if (!ct_logbuf || !ct_loglen) return;
    FILE *f = FIO_CreateFileOrAppend("ML/CROPTEST.TXT");
    if (f)
    {
        FIO_WriteFile(f, ct_logbuf, ct_loglen);
        FIO_CloseFile(f);
    }
    ct_loglen = 0;
}

static inline uint32_t ct_us(void) { return (uint32_t)get_us_clock(); }

/* hash 3 x 16 bytes of one row (left quarter, centre, right quarter) */
static uint32_t ct_probe(uint32_t row)
{
    uint32_t h = 0x811C9DC5;
    for (int s = 1; s <= 3; s++)
    {
        const volatile uint32_t *p = (const volatile uint32_t *)
            (ct_ubuf + row * ct_pitch + ((ct_pitch * s / 4) & ~3));
        for (int i = 0; i < 4; i++) h = (h ^ p[i]) * 0x01000193;
    }
    return h;
}

/* A row can be caught while the DMA is rewriting it, so a frame would show
 * up as 2+ changes. Two equal back-to-back reads are NOT enough (the DMA may
 * be between the sampled spots; one probe ~1 us, one row write several us):
 * accept a value once it has not changed for CT_STABLE_US. Also ignore new
 * changes on the same row for CT_DEAD_US (frames are >= 8 ms apart). */
#define CT_DEAD_US 3000
#define CT_LATE_US 1000
#define CT_STABLE_US 50
static uint32_t ct_probe_stable(uint32_t row)
{
    uint32_t a = ct_probe(row);
    uint32_t t0 = ct_us(), t = t0;
    for (;;)
    {
        uint32_t now = ct_us();
        if (now - t >= CT_STABLE_US || now - t0 > 2000) break;
        uint32_t b = ct_probe(row);
        if (b != a) { a = b; t = now; }
    }
    return a;
}

/* copy a centred-crop window row by row; dst wraps if smaller than the crop */
static void ct_copy(uint8_t *dst, uint32_t dstlen, uint32_t src,
                    uint32_t x0, uint32_t y0, uint32_t cw, uint32_t ch)
{
    uint32_t rb = cw * 14 / 8;
    uint32_t s = src + y0 * ct_pitch + x0 * 14 / 8;
    uint32_t d = 0;
    for (uint32_t r = 0; r < ch; r++)
    {
        if (d + rb > dstlen) d = 0;
        memcpy(dst + d, (void *)(s + r * ct_pitch), rb);
        d += rb;
    }
}

/* centred crop, x0/width multiples of 16 px (keeps rows 4-byte aligned), y0 even */
static void ct_crop(uint32_t want_w, uint32_t want_h, uint32_t *x0, uint32_t *y0, uint32_t *cw, uint32_t *ch)
{
    *cw = MIN(want_w, ct_w) & ~15;
    *ch = MIN(want_h, ct_h) & ~1;
    *x0 = ((ct_w - *cw) / 2) & ~15;
    *y0 = ((ct_h - *ch) / 2) & ~1;
}

/* ---- phase 1: frame timing ---- */
/* sample arrays live in one runtime allocation (keeps ML's boot footprint small) */
struct ct_mem {
    uint32_t ev_t[CT_MAXEV];
    uint8_t  ev_p[CT_MAXEV];
    uint32_t fr_t[CT_MAXFR], fr_copy[CT_MAXFR];
    uint8_t  fr_q[CT_MAXFR], fr_tear[CT_MAXFR];
};
static struct ct_mem *cm;
static uint32_t ct_period_us;

static void ct_timing(uint32_t ry0, uint32_t ry1)
{
    uint32_t rows[4] = { 0, ry0, ry1, ct_h - 1 };
    uint32_t prev[4], last_ev[4] = { 0 };
    for (int i = 0; i < 4; i++) prev[i] = ct_probe_stable(rows[i]);

    int n = 0;
    uint32_t t0 = ct_us();
    while (n < CT_MAXEV)
    {
        uint32_t now = ct_us() - t0;
        if (now > 600000) break;
        for (int i = 0; i < 4; i++)
        {
            uint32_t v = ct_probe(rows[i]);
            if (v == prev[i]) continue;
            prev[i] = v = ct_probe_stable(rows[i]);
            if (last_ev[i] && now - last_ev[i] < CT_DEAD_US) continue;   /* same frame */
            last_ev[i] = now;
            cm->ev_t[n] = now; cm->ev_p[n] = i;
            if (++n >= CT_MAXEV) break;
        }
    }

    ct_printf("== timing: probe rows 0=%d 1=%d 2=%d 3=%d, %d events in 600 ms\n",
              rows[0], rows[1], rows[2], rows[3], n);
    for (int i = 0; i < n; i++) ct_printf("EV %6d %d\n", cm->ev_t[i], cm->ev_p[i]);

    /* frame period from the crop-bottom probe */
    int first = -1, last = -1, cnt = 0;
    for (int i = 0; i < n; i++)
        if (cm->ev_p[i] == 2) { if (first < 0) first = i; last = i; cnt++; }
    ct_period_us = (cnt >= 3) ? (cm->ev_t[last] - cm->ev_t[first]) / (cnt - 1) : 0;
    ct_printf("PERIOD %d us (%d crop-bottom changes)\n", ct_period_us, cnt);
}

/* ---- phase 2: copy speed ---- */
static void ct_speed_one(const char *what, uint8_t *dst, uint32_t dstlen, uint32_t src,
                         uint32_t want_w, uint32_t want_h)
{
    uint32_t x0, y0, cw, ch;
    ct_crop(want_w, want_h, &x0, &y0, &cw, &ch);
    if (cw < want_w || ch < want_h) { ct_printf("SPEED %-10s %dx%d: does not fit %dx%d, skipped\n", what, want_w, want_h, ct_w, ct_h); return; }
    uint32_t bytes = cw * ch * 14 / 8, best = 0xFFFFFFFF, sum = 0;
    for (int rep = 0; rep < 5; rep++)
    {
        uint32_t t = ct_us();
        ct_copy(dst, dstlen, src, x0, y0, cw, ch);
        t = ct_us() - t;
        sum += t; if (t < best) best = t;
        msleep(20);
    }
    /* MiB/s x10 from the best run */
    uint32_t mibs10 = best ? (uint32_t)((uint64_t)bytes * 10000000ULL / best / (1024 * 1024)) : 0;
    ct_printf("SPEED %-10s %4dx%-4d %8d bytes: best %6d us, avg %6d us, %d.%d MiB/s\n",
              what, cw, ch, bytes, best, sum / 5, mibs10 / 10, mibs10 % 10);
}

static void ct_speed(uint8_t *dst_unc, uint32_t dstlen)
{
    uint8_t *dst_c = CACHEABLE(dst_unc);
    ct_printf("== copy speed (5 reps each, dst %d bytes, wraps for big crops)\n", dstlen);
    static const uint16_t sz[][2] = { {1280, 720}, {1920, 1080}, {2560, 1440}, {3840, 2160} };
    for (int i = 0; i < 4; i++)
    {
        ct_speed_one("unc->unc", dst_unc, dstlen, ct_ubuf, sz[i][0], sz[i][1]);
        ct_speed_one("unc->cac", dst_c,   dstlen, ct_ubuf, sz[i][0], sz[i][1]);
    }
    /* the recording crop (1920x1080 or what fits), then references
     * (data from the cached source may be stale) */
    uint32_t x0, y0, cw, ch;
    ct_crop(1920, 1080, &x0, &y0, &cw, &ch);
    ct_speed_one("rec unc->unc", dst_unc, dstlen, ct_ubuf, cw, ch);
    ct_speed_one("rec unc->cac", dst_c,   dstlen, ct_ubuf, cw, ch);
    ct_speed_one("cac->cac", dst_c, dstlen, ct_ubuf & ~CT_UNC, cw, ch);

    uint32_t bytes = MIN(cw * ch * 14 / 8, dstlen), best = 0xFFFFFFFF;
    for (int rep = 0; rep < 5; rep++)
    {
        uint32_t t = ct_us();
        memcpy(dst_c, (void *)ct_ubuf, bytes);
        t = ct_us() - t;
        if (t < best) best = t;
        msleep(20);
    }
    uint32_t mibs10 = (uint32_t)((uint64_t)bytes * 10000000ULL / best / (1024 * 1024));
    ct_printf("SPEED contiguous unc->cac %d bytes: best %d us, %d.%d MiB/s\n", bytes, best, mibs10 / 10, mibs10 % 10);
}

/* ---- phase 3: copy + write burst ---- */
static uint8_t *ring[CT_NBUF];
static int nring;
static uint32_t ring_bytes;
static volatile int q_head, q_tail, w_stop, w_done, w_err;
static uint32_t w_n, w_us, w_max;
static FILE *w_file;

static void ct_writer(void *unused)
{
    while (!w_stop || q_tail != q_head)
    {
        if (q_tail == q_head) { msleep(5); continue; }
        uint32_t t = ct_us();
        int r = FIO_WriteFile(w_file, ring[q_tail % nring], ring_bytes);
        t = ct_us() - t;
        if (r != (int)ring_bytes) { w_err = r ? r : -1; break; }
        w_n++; w_us += t; if (t > w_max) w_max = t;
        q_tail++;
    }
    w_done = 1;
}

static void ct_burst(int dst_cached)
{
    uint32_t x0, y0, cw, ch;
    ct_crop(1920, 1080, &x0, &y0, &cw, &ch);
    uint32_t r_top = y0, r_bot = y0 + ch - 1;
    ct_printf("== burst: crop %dx%d at (%d,%d), %d bytes/frame, %d buffers, dst %s, period %d us\n",
              cw, ch, x0, y0, ring_bytes, nring, dst_cached ? "cached" : "uncached", ct_period_us);
    ct_flush();

    w_file = FIO_CreateFile("ML/CROPREC.BIN");
    if (!w_file) { ct_printf("burst: cannot create ML/CROPREC.BIN\n"); return; }
    q_head = q_tail = w_stop = w_done = w_err = 0;
    w_n = w_us = w_max = 0;
    task_create("ct_writer", 0x1d, 0x1000, ct_writer, 0);

    /* Both the DMA and the copy go top to bottom. A copy of frame N is clean
     * if (a) the next frame had not rewritten the crop's top row when the
     * copy started, and (b) it had not reached the crop's bottom row when the
     * copy ended (if the DMA overtakes the copy anywhere, it gets to the
     * bottom row first). The next frame rewriting rows the copy has already
     * passed is fine. top_ref = top row as seen by the last poll where the
     * bottom row was still old; "late" = that poll was > CT_LATE_US before the
     * detection (we were copying or sleeping), so (a) can't be checked. */
    int frames = 0, overruns = 0, missed = 0, tears = 0, lates = 0, nlog = 0;
    uint32_t cmin = 0xFFFFFFFF, cmax = 0, csum = 0, last_t = 0;
    /* once the period is known, ignore re-triggers within 60% of it */
    uint32_t dead_us = ct_period_us ? ct_period_us * 6 / 10 : CT_DEAD_US;
    uint32_t prevb = ct_probe_stable(r_bot);
    uint32_t top_ref = ct_probe(r_top), last_same_t = 0;
    uint32_t t0 = ct_us();
    for (;;)
    {
        uint32_t now = ct_us() - t0;
        if (now > CT_BURST_US || w_err) break;
        uint32_t tv = ct_probe(r_top);
        uint32_t v = ct_probe(r_bot);
        if (v == prevb) { top_ref = tv; last_same_t = now; continue; }
        prevb = ct_probe_stable(r_bot);         /* crop's last row fully written */
        if (last_t && now - last_t < dead_us) { top_ref = ct_probe(r_top); last_same_t = now; continue; }
        if (last_t && ct_period_us)
        {
            uint32_t k = (now - last_t + ct_period_us / 2) / ct_period_us;
            if (k > 1) missed += k - 1;
        }
        last_t = now;
        int late = now - last_same_t > CT_LATE_US;

        if (q_head - q_tail >= nring) { overruns++; continue; }

        uint8_t *dst = ring[q_head % nring];
        if (dst_cached) dst = CACHEABLE(dst);
        uint32_t top0 = ct_probe(r_top);
        uint32_t t = ct_us();
        ct_copy(dst, ring_bytes, ct_ubuf, x0, y0, cw, ch);
        t = ct_us() - t;
        uint32_t bot1 = ct_probe(r_bot);
        int tear = top0 != top_ref || bot1 != prevb;
        int qd = q_head - q_tail;
        q_head++;

        frames++; tears += tear; lates += late; csum += t;
        if (t < cmin) cmin = t;
        if (t > cmax) cmax = t;
        if (nlog < CT_MAXFR) { cm->fr_t[nlog] = now; cm->fr_copy[nlog] = t; cm->fr_q[nlog] = qd; cm->fr_tear[nlog] = tear | (late << 1); nlog++; }

        /* let lower-priority tasks run once per frame if there's time; a
         * frame finished meanwhile is picked up by the next poll (as late) */
        if (ct_period_us > t + 15000) msleep(10);
        top_ref = ct_probe(r_top);
        last_same_t = ct_us() - t0;
        if (ct_probe(r_bot) != prevb) last_same_t = 0;   /* bottom changed meanwhile: late */
    }
    uint32_t dur = ct_us() - t0;

    w_stop = 1;
    for (int i = 0; i < 1000 && !w_done; i++) msleep(10);
    if (w_done) FIO_CloseFile(w_file);
    else ct_printf("burst: writer still busy after 10 s, file left open, buffers not freed\n");

    ct_printf("BURST %d us: copied %d, missed %d, overruns %d, torn %d, late %d (clean = neither)\n", dur, frames, missed, overruns, tears, lates);
    if (frames)
        ct_printf("BURST copy us min %d avg %d max %d\n", cmin, csum / frames, cmax);
    if (w_n)
    {
        uint32_t mibs10 = (uint32_t)((uint64_t)ring_bytes * w_n * 10000000ULL / w_us / (1024 * 1024));
        ct_printf("BURST write: %d frames, avg %d us, max %d us, %d.%d MiB/s while copying, err %d\n",
                  w_n, w_us / w_n, w_max, mibs10 / 10, mibs10 % 10, w_err);
    }
    for (int i = 0; i < nlog; i++)
        ct_printf("FR %7d copy %6d q %d%s%s\n", cm->fr_t[i], cm->fr_copy[i], cm->fr_q[i], (cm->fr_tear[i] & 1) ? " TEAR" : "", (cm->fr_tear[i] & 2) ? " LATE" : "");
}

static void ct_task(void *unused)
{
    ct_logbuf = malloc(CT_LOGLEN);
    ct_loglen = 0;
    cm = malloc(sizeof(*cm));
    if (!ct_logbuf || !cm)
    {
        NotifyBox(6000, "LV crop test: out of memory");
        goto out;
    }
    ct_printf("start v10 crop test run %d t=%d lv=%d movie=%d\n", ct_runs, get_ms_clock(), lv ? 1 : 0, is_movie_mode());
    ct_flush();

    ct_printf("call lv_save_raw 1\n"); ct_flush();
    call("lv_save_raw", 1);
    msleep(500);

    ct_w = MEM(CT_PATH + 0x10); ct_h = MEM(CT_PATH + 0x14);
    uint32_t a = MEM(CT_PATH + 0x58);
    ct_pitch = ct_w * 14 / 8;
    ct_ubuf = CT_BUF | CT_UNC;
    ct_printf("M1 w=%d h=%d addr=%08x pitch=%d\n", ct_w, ct_h, a, ct_pitch);

    if (!ct_w || !ct_h || ct_pitch * ct_h > CT_BUFLEN || (a & ~CT_UNC) != CT_BUF || ct_h < 64)
    {
        ct_printf("MEM1 not active or size unexpected, skipping tests\n");
        goto off;
    }

    uint32_t x0, y0, cw, ch;
    ct_crop(1920, 1080, &x0, &y0, &cw, &ch);
    ring_bytes = cw * ch * 14 / 8;
    nring = 0;
    for (int i = 0; i < CT_NBUF; i++)
    {
        ring[i] = fio_malloc(ring_bytes);
        if (!ring[i]) break;
        nring++;
    }
    ct_printf("ring: %d x %d bytes\n", nring, ring_bytes);
    if (!nring) goto off;

    ct_timing(y0, y0 + ch - 1);
    ct_flush();

    ct_speed(ring[0], ring_bytes);
    ct_flush();

    if (nring >= 2)
    {
        /* use the faster destination type at the recording crop */
        uint32_t tu = ct_us(); ct_copy(ring[0], ring_bytes, ct_ubuf, x0, y0, cw, ch); tu = ct_us() - tu;
        uint32_t tc = ct_us(); ct_copy(CACHEABLE(ring[0]), ring_bytes, ct_ubuf, x0, y0, cw, ch); tc = ct_us() - tc;
        w_file = 0; w_done = 0;
        ct_burst(tc < tu);
    }
    else ct_printf("burst skipped: need 2 buffers\n");
    ct_flush();

    /* never free buffers a stuck writer may still be reading */
    if (w_done || !w_file)
        for (int i = 0; i < nring; i++) free(ring[i]);
    nring = 0;

off:
    ct_printf("call lv_save_raw 0\n"); ct_flush();
    call("lv_save_raw", 0);
    ct_printf("done t=%d\n", get_ms_clock());
    ct_flush();
    ct_runs++;
    NotifyBox(6000, "LV crop test: DONE");
out:
    if (ct_logbuf) free(ct_logbuf);
    if (cm) free(cm);
    ct_logbuf = 0; cm = 0;
    info_led_blink(3, 100, 100);
    ct_busy = 0;
}

static MENU_SELECT_FUNC(ct_select)
{
    if (ct_busy) return;
    ct_busy = 1;
    task_create("ct_task", 0x1e, 0x2000, ct_task, 0);
}

static MENU_UPDATE_FUNC(ct_update)
{
    MENU_SET_VALUE(ct_busy ? "running..." : "%d runs saved", ct_runs);
}

static struct menu_entry ct_menu[] = {
    {
        .name   = "LV crop copy test",
        .select = ct_select,
        .update = ct_update,
        .help   = "MEM1 on: frame timing, crop copy speed, 3 s 1080p copy+write -> ML/CROP*",
        .help2  = "NOT recording. SET, close menu, wait for DONE (~10 s). Freeze: pull battery.",
    },
};

static void ct_init(void *unused)
{
    menu_add("Debug", ct_menu, COUNT(ct_menu));
}

INIT_FUNC("croptest", ct_init);
