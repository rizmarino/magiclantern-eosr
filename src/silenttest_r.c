/* riz/eosr-dmlog: silent (electronic) shutter test, stage 2 (EOS R 1.8.0).
 * Personal diagnostic build, not for upstream.
 *
 * Debug -> Silent shutter test. Read-only: no property writes, no settings
 * changes, nothing patched.
 *
 *  - Prop watch: for 30/90/180 s, logs the silent-shutter, drive, job-state,
 *    counter, file-number, mode and Tv properties with timestamps, plus three
 *    words of Canon's capture-control struct in RAM (Warp/CaptureState, 0x7368:
 *    +0x10 factory-mode bits, +0x2C the flag that makes ShutType 1, +0x58 the
 *    sht_EnableManualSilent flag), while the user changes Canon's menus.
 *    Answers: does Canon only grey out continuous drive with Silent shutter
 *    on, or does it force PROP_DRIVE back to single?
 *  - Shot test: fires releases through SetEventIrRemoteReleaseBtn, the same
 *    call lens_take_picture() (intervalometer, bracketing) uses on the R, and
 *    logs when each one is accepted (release counter or file number change).
 *    Paced: next press once the camera is idle again. Retry: press again every
 *    ~250 ms until accepted (Canon drops presses while busy), which measures
 *    the fastest single-shot cadence. Mechanical shutter counter delta shows
 *    whether the shutter moved.
 *  - Canon log (optional): Canon's DbgMgr store levels (LV classes muted,
 *    capture/GUI classes fully, everything else at level >= 3), frozen and
 *    saved with dumpf (LOGnnn.LOG on the card root). In a shot test it is
 *    frozen ~4 s after the first accepted shot, so it holds one whole capture
 *    (CaptureState logs "ShutType(%d) ROMode(%d) ReleaseType(%d)" per shot).
 *
 * Log: ML/SILENT.TXT (appended, one block per run). Nothing is written to the
 * card while shots are in progress: the log is written once the camera has
 * been idle for 2 s (card I/O racing Canon's own image write deadlocked the
 * bracketing code on the R, see lens.c).
 */
#include "dryos.h"
#include "menu.h"
#include "property.h"
#include "propvalues.h"
#include "lens.h"
#include "timer.h"
#include "config.h"
#include "fio-ml.h"
#ifdef CONFIG_MMU_REMAP
#include "patch.h"
#endif

#ifndef ST_HOST
#include "arm-mcr.h"
extern void SetEventIrRemoteReleaseBtn(int on);                 /* stubs.S 0xE0190214 */
extern void dm_set_store_level(uint32_t class, uint32_t level); /* stubs.S 0xE0563720 */
#define ST_RD(a)        (*(volatile uint32_t *)(a))
#define ST_LOCK()       uint32_t _st_irq = cli()
#define ST_UNLOCK()     sei(_st_irq)
#define ST_JOB()        (lens_info.job_state)
#else
#include "st_host.h"
#endif

#define ST_VER              "v1.3"
#define PROP_SILENT_SHUTTER 0x8004007A  /* GUI_SetSilentShutter -> prop_request_change(0x80040032 + 0x48) */
#define ST_SCS              0x7368      /* Warp/CaptureState struct (RAM .data) */

#define ST_EV_MAX   1024
#define ST_LOG_MAX  0x18000
#define ST_SHOT_MAX 64

enum { EV_PROP = 0, EV_SCS, EV_PRESS, EV_UNPRESS, EV_NOTE };

struct st_ev {
    uint32_t ms;
    uint32_t id;                        /* property id, or note number */
    uint16_t kind, len;
    uint32_t v[3];
};

/* properties logged (all read-only) */
static const uint32_t st_props[] = {
    PROP_SILENT_SHUTTER, PROP_DRIVE, PROP_LAST_JOB_STATE, PROP_RELEASE_COUNTER,
    PROP_MECHA_COUNTER, PROP_FILE_NUMBER_A, PROP_SHOOTING_MODE, PROP_SHUTTER,
};
static const char *st_pname[] = { "SILENT", "DRIVE", "JOB", "RELCNT", "MECHA", "FILENO", "MODE", "TV" };
enum { P_SILENT = 0, P_DRIVE, P_JOB, P_RELCNT, P_MECHA, P_FILENO, P_MODE, P_TV, P_N };

static volatile uint32_t st_cur[P_N];   /* latest value (first word), kept even when not armed */
static volatile uint16_t st_cur_len[P_N];
static volatile uint32_t st_cnt[P_N];   /* handler calls since boot */
static volatile uint32_t st_job_edges;  /* job state 0 -> non-zero transitions since boot */

static struct st_ev *st_ev;
static volatile int st_n, st_over, st_armed;
static uint32_t st_t0;

static void st_add(int kind, uint32_t id, int len, const uint32_t *v)
{
    if (!st_armed) return;
    int i;
    {
        ST_LOCK();
        i = st_n;
        if (st_ev && i < ST_EV_MAX) st_n = i + 1;
        else { st_over++; i = -1; }
        ST_UNLOCK();
    }
    if (i < 0) return;
    struct st_ev *e = &st_ev[i];
    e->ms = get_ms_clock();
    e->id = id;
    e->kind = kind;
    e->len = len;
    e->v[0] = v ? v[0] : 0;
    e->v[1] = v ? v[1] : 0;
    e->v[2] = v ? v[2] : 0;
}

static void st_note(uint32_t id, uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t v[3] = { a, b, c };
    st_add(EV_NOTE, id, 0, v);
}

static void st_prop(unsigned property, void *priv, void *buf, unsigned len)
{
    int k;
    for (k = 0; k < P_N; k++)
        if (st_props[k] == property) break;
    if (k == P_N || !buf) return;

    uint32_t v[3] = { 0, 0, 0 };
    unsigned n = len < sizeof(v) ? len : sizeof(v);
    memcpy(v, buf, n);

    if (k == P_JOB && st_cur[P_JOB] == 0 && v[0] != 0) st_job_edges++;
    st_cur[k] = v[0];
    st_cur_len[k] = len;
    st_cnt[k]++;
    st_add(EV_PROP, property, len, v);
}

#ifndef ST_HOST
REGISTER_PROP_HANDLER(PROP_SILENT_SHUTTER,  st_prop);
REGISTER_PROP_HANDLER(PROP_DRIVE,           st_prop);
REGISTER_PROP_HANDLER(PROP_LAST_JOB_STATE,  st_prop);
REGISTER_PROP_HANDLER(PROP_RELEASE_COUNTER, st_prop);
REGISTER_PROP_HANDLER(PROP_MECHA_COUNTER,   st_prop);
REGISTER_PROP_HANDLER(PROP_FILE_NUMBER_A,   st_prop);
REGISTER_PROP_HANDLER(PROP_SHOOTING_MODE,   st_prop);
REGISTER_PROP_HANDLER(PROP_SHUTTER,         st_prop);
#endif

/* ---- Canon capture-control struct, sampled by the run task ---- */
static uint32_t st_scs[3];
static void st_scs_read(uint32_t *o)
{
    o[0] = ST_RD(ST_SCS + 0x10);
    o[1] = ST_RD(ST_SCS + 0x2C);
    o[2] = ST_RD(ST_SCS + 0x58);
}
static void st_scs_poll(void)
{
    uint32_t s[3];
    st_scs_read(s);
    if (s[0] != st_scs[0] || s[1] != st_scs[1] || s[2] != st_scs[2])
    {
        st_scs[0] = s[0]; st_scs[1] = s[1]; st_scs[2] = s[2];
        st_add(EV_SCS, ST_SCS, 12, s);
    }
}

/* ---- Canon DbgMgr log ---- */
static const uint8_t st_lv_classes[] = { 0xC9, 0x9C, 0x9D, 0x9E, 0xCA, 0x93, 0xB7, 0x92 };  /* muted; 0x92 = LVGMT/LVEZM (v1: flooded the log) */
static const uint8_t st_cap_classes[] = {
    0x8E,   /* CaptureState: ShutType/ROMode/ReleaseType, scs* */
    0x8C,   /* CAPD: ImageSequence / ImageController (EshutState) */
    0x82,   /* GUI: GUI_SetSilentShutter, Dlg* */
    0x81,   /* PROP_DRIVE_MODE defaults per shot mode */
    0x0C,   /* "%s : PROP_DRIVE_MODE(%d)" */
    0x96, 0x97, 0x84,   /* PROP_SHOOTING_TYPE */
};
static int st_dm_on;

static void st_dm_start(int run)
{
    dm_set_store_level(0xFF, 3);
    for (int i = 0; i < COUNT(st_lv_classes); i++) dm_set_store_level(st_lv_classes[i], 15);
    for (int i = 0; i < COUNT(st_cap_classes); i++) dm_set_store_level(st_cap_classes[i], 0);
    st_dm_on = 1;
    DryosDebugMsg(0, 15, "=== riz silt %s run %d start", ST_VER, run);
}

/* Burst log (v1.2): only the release/capture classes, everything else muted (stage-1 plan, lv-survey.md) */
static const uint8_t st_burst_full[] = {
    0xDC,   /* PO: SCN, SEQ, CAPREADY + fps + Plan, CAPREADY_DELAY, TRSEND, LvPoCom, burst counters */
    0x41,   /* CFLW ConcurrentFlow: every release step Start/Complete */
    0x8E,   /* SHTC CaptureState: ShutType/ROMode, scsShootStart/scsContinuousShootStart */
    0x96,   /* FSS: SW1/SW2 */
};
static const uint8_t st_burst_l3[] = {
    0x8C,   /* CAPD: SRS/VSize/CapEnd/ReadoutDone/LV_* */
    0x8D,   /* CAPE */
    0x33,   /* JOB */
    0xDF,   /* ICODEC */
};
static int st_burst_mode;

static void st_dm_start_burst(int run)
{
    dm_set_store_level(0xFF, 15);
    for (int i = 0; i < COUNT(st_burst_full); i++) dm_set_store_level(st_burst_full[i], 0);
    for (int i = 0; i < COUNT(st_burst_l3); i++) dm_set_store_level(st_burst_l3[i], 3);
    st_dm_on = 1;
    DryosDebugMsg(0, 15, "=== riz silt %s run %d burst start", ST_VER, run);
}

static void st_dm_freeze(int run, const char *why)
{
    if (!st_dm_on) return;
    DryosDebugMsg(0, 15, "=== riz silt run %d freeze %s", run, why);
    dm_set_store_level(0xFF, 15);
    st_dm_on = 0;
}

/* ---- text log ---- */
static char *st_log;
static int st_loglen;

static void st_printf(const char *fmt, ...)
{
    if (!st_log || st_loglen > ST_LOG_MAX - 200) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(st_log + st_loglen, ST_LOG_MAX - st_loglen, fmt, ap);
    va_end(ap);
    if (n > 0) st_loglen += MIN(n, ST_LOG_MAX - st_loglen - 1);
}

static int st_flush(void)
{
    if (!st_log || !st_loglen) return 0;
    FILE *f = FIO_CreateFileOrAppend("ML/SILENT.TXT");
    if (!f) return 0;
    FIO_WriteFile(f, st_log, st_loglen);
    FIO_CloseFile(f);
    st_loglen = 0;
    return 1;
}

static const char *st_note_name(uint32_t id)
{
    switch (id)
    {
        case 1: return "hold-300";      /* retry mode: no accept with 100 ms holds, switched */
        case 2: return "no-accept";
        case 3: return "accept";
        case 4: return "dm-freeze";
        case 5: return "abort";
        case 6: return "idle-timeout";
        case 7: return "queue-probe";   /* retry mode: 3 presses while busy; extra shots = queued */
        case 8: return "runaway";       /* Canon fired without our press: presses, shots */
        case 9: return "runaway-stop";  /* shots after release-up, after press cycle */
        default: return "note";
    }
}

static int st_dec(int k) { return k == P_RELCNT || k == P_MECHA || k == P_FILENO; }   /* counters in decimal */

static void st_dump_state(const char *tag)
{
    uint32_t s[3];
    st_scs_read(s);
    st_printf("%s:", tag);
    for (int k = 0; k < P_N; k++)
        st_printf(st_dec(k) ? " %s=%d" : " %s=%x", st_pname[k], (unsigned)st_cur[k]);
    st_printf(" jobedges=%d SCS10=%x SCS2C=%x SCS58=%x\n", (unsigned)st_job_edges, (unsigned)s[0], (unsigned)s[1], (unsigned)s[2]);
    st_printf("%s handler calls:", tag);
    for (int k = 0; k < P_N; k++)
        st_printf(" %s=%d/%db", st_pname[k], (unsigned)st_cnt[k], (unsigned)st_cur_len[k]);
    st_printf("\n");
}

static void st_dump_events(void)
{
    int n = st_n;
    st_printf("events (%d, %d lost): ms_from_start kind name values\n", n, st_over);
    for (int i = 0; i < n; i++)
    {
        struct st_ev *e = &st_ev[i];
        int t = (int)(e->ms - st_t0);
        switch (e->kind)
        {
            case EV_PROP:
            {
                int pk = -1;
                for (int k = 0; k < P_N; k++) if (st_props[k] == e->id) pk = k;
                st_printf(pk >= 0 && st_dec(pk) ? "%6d P %s len %d: %d %d %d\n" : "%6d P %s len %d: %x %x %x\n",
                          t, pk >= 0 ? st_pname[pk] : "?", e->len, (unsigned)e->v[0], (unsigned)e->v[1], (unsigned)e->v[2]);
                break;
            }
            case EV_SCS:
                st_printf("%6d S SCS +10=%x +2C=%x +58=%x\n", t, (unsigned)e->v[0], (unsigned)e->v[1], (unsigned)e->v[2]);
                break;
            case EV_PRESS:
                st_printf("%6d > press %d (hold %d ms)\n", t, (unsigned)e->id, (unsigned)e->v[0]);
                break;
            case EV_UNPRESS:
                st_printf("%6d < unpress %d\n", t, (unsigned)e->id);
                break;
            default:
                st_printf("%6d N %s %d: %d %d %d\n", t, st_note_name(e->id), (unsigned)e->id, (unsigned)e->v[0], (unsigned)e->v[1], (unsigned)e->v[2]);
        }
    }
}

/* ---- settings ---- */
static int st_watch_sel = 1;            /* 30 / 90 / 180 s */
static const int st_watch_s[] = { 30, 90, 180 };
static int st_mode_sel = 0;             /* 0 paced, 1 retry */
static int st_nsel = 1;                 /* 5 / 10 / 20 shots */
static const int st_nshots[] = { 5, 10, 20 };
static int st_dm_sel = 1;               /* Canon log */

static volatile int st_busy;
static int st_runs;
static char st_last[64] = "";

static int st_menu_wait(void)
{
    /* wait for the ML menu to close (max 30 s), then 2 s so the user can let go of the camera */
    for (int i = 0; i < 300 && gui_menu_shown(); i++) msleep(100);
    if (gui_menu_shown()) return 0;
    msleep(2000);
    return 1;
}

static void st_begin(int run, const char *what)
{
    st_n = 0;
    st_over = 0;
    st_loglen = 0;
    st_t0 = get_ms_clock();
    st_scs_read(st_scs);
    st_printf("\n=== silent test %s run %d: %s, Canon log %s, boot+%d ms\n", ST_VER, run, what,
              !st_dm_sel ? "OFF" : st_burst_mode ? "BURST" : "ON", (unsigned)st_t0);
    st_dump_state("start");
    st_armed = 1;
    if (st_dm_sel) { if (st_burst_mode) st_dm_start_burst(run); else st_dm_start(run); }
}

static void st_end(int run)
{
    st_dm_freeze(run, "end");
    st_armed = 0;
    msleep(100);                        /* a handler may still be filling its last entry */
    st_dump_state("end");
    st_dump_events();
}

static void st_save(int run)
{
    int ok = st_flush();
    if (st_dm_sel)
    {
        msleep(200);
        call("dumpf");                  /* LOGnnn.LOG on the card root */
    }
    NotifyBox(8000, "Silent test run %d: DONE%s. %s", run, ok ? "" : " (LOG WRITE FAILED)", st_last);
    info_led_blink(ok ? 1 : 3, 100, 100);
}

/* ---- prop watch ---- */
static void st_watch_task(void *unused)
{
    int run = ++st_runs;
    int dur = st_watch_s[st_watch_sel];
    if (!st_menu_wait()) { snprintf(st_last, sizeof(st_last), "menu still open"); st_busy = 0; return; }
    st_begin(run, "prop watch");
    st_printf("watch %d s\n", dur);
    NotifyBox(3000, "Silent test: watching %d s. Change Canon menus now.", dur);
    uint32_t t_end = st_t0 + dur * 1000;
    while ((int)(get_ms_clock() - t_end) < 0)
    {
        st_scs_poll();
        msleep(20);
    }
    int evs = st_n;
    st_end(run);
    snprintf(st_last, sizeof(st_last), "watch: %d events", evs);
    st_printf("summary: %s\n", st_last);
    st_save(run);
    st_busy = 0;
}

/* ---- shot test ---- */
static uint32_t st_rel0, st_file0;

static int st_progress(void)
{
    int a = (int)(st_cur[P_RELCNT] - st_rel0);
    int b = (int)(st_cur[P_FILENO] - st_file0);
    return MAX(a, b);
}

static void st_sleep_poll(int ms)
{
    for (int t = 0; t < ms; t += 10) { st_scs_poll(); msleep(10); }
}

/* v1.1: release-up sent twice. In v1 Canon once kept firing ~1/s for 12 s after a press (single drive, silent on),
 * as if the release-up was lost, and stopped after the next press/release cycle. */
static void st_release_up(void)
{
    SetEventIrRemoteReleaseBtn(0);
    st_sleep_poll(30);
    SetEventIrRemoteReleaseBtn(0);
}

/* runaway guard: more shots than this means Canon fired without a press of ours (0 = off) */
static int st_guard;

/* job state non-zero seen, then zero for `quiet` ms; returns 0 on timeout, -1 when the guard trips */
static int st_wait_idle(int busy_wait, int quiet, int max_ms)
{
    uint32_t t0 = get_ms_clock();
    while (ST_JOB() == 0 && (int)(get_ms_clock() - t0) < busy_wait) st_sleep_poll(10);
    uint32_t q0 = get_ms_clock();
    while ((int)(get_ms_clock() - t0) < max_ms)
    {
        if (st_guard && st_progress() > st_guard) return -1;
        if (ST_JOB() != 0) q0 = get_ms_clock();
        else if ((int)(get_ms_clock() - q0) >= quiet) return 1;
        st_sleep_poll(10);
    }
    return 0;
}

/* Canon fired without our press: release-up again, watch 2.5 s; if it still fires, one press/release cycle (what
 * ended the v1 runaway), watch 3 s more. Returns the shots seen after the guard tripped. */
static int st_runaway_stop(int presses)
{
    st_guard = 0;
    int p0 = st_progress();
    st_note(8, presses, p0, 0);
    NotifyBox(5000, "Silent test: camera fired on its own. Switch it OFF if it keeps going.");
    st_release_up();
    st_sleep_poll(2500);
    int more1 = st_progress() - p0, more2 = 0;
    if (more1 > 0)
    {
        SetEventIrRemoteReleaseBtn(1);
        st_sleep_poll(300);
        st_release_up();
        int p1 = st_progress();
        st_sleep_poll(3000);
        more2 = st_progress() - p1;
    }
    st_note(9, more1, more2, 0);
    st_printf("RUNAWAY: Canon fired without a press (shots %d > expected after %d presses); "
              "after release-up %d more, after press cycle %d more\n", p0, presses, more1, more2);
    return more1 + more2;
}

struct st_shot { uint16_t press, hold; uint32_t t_press, t_acc; uint8_t step, after_probe; };

static void st_shot_task(void *unused)
{
    int run = ++st_runs;
    int nshots = st_nshots[st_nsel];
    int retry = st_mode_sel == 1;
    struct st_shot *sh = malloc(ST_SHOT_MAX * sizeof(struct st_shot));
    if (!sh) { snprintf(st_last, sizeof(st_last), "no memory"); st_busy = 0; return; }
    if (!st_menu_wait()) { snprintf(st_last, sizeof(st_last), "menu still open"); free(sh); st_busy = 0; return; }

    st_begin(run, retry ? "shot test RETRY" : "shot test PACED");
    st_printf("v1.1 safety: release-up sent twice, runaway guard on\n");
    st_printf("shots %d, mode %s\n", nshots, retry ? "retry (hold 100 ms, re-press after 150 ms without accept)" : "paced (hold 300 ms, next press after job idle 150 ms)");
    st_rel0 = st_cur[P_RELCNT];
    st_file0 = st_cur[P_FILENO];
    uint32_t mech0 = st_cur[P_MECHA], edges0 = st_job_edges;

    int hold = retry ? 100 : 300;
    int acc = 0, presses = 0, miss_run = 0, aborted = 0, nsh = 0, frozen = !st_dm_sel;
    int probed = !retry, probe_mark = 0, runaway = 0;
    uint32_t t_first_acc = 0;
    const char *why = "done";

    while (acc < nshots)
    {
        /* each press gives at most one shot; paced: nothing may arrive after the accepted one */
        if (st_progress() > (retry ? presses : acc)) { why = "RUNAWAY (camera fired on its own)"; aborted = 1; runaway = 1; break; }
        if ((int)(get_ms_clock() - st_t0) > 90000) { why = "90 s limit"; aborted = 1; break; }
        if (presses >= (retry ? nshots * 30 : nshots * 3)) { why = "press limit"; aborted = 1; break; }

        presses++;
        uint32_t tp = get_ms_clock();
        uint32_t hv[3] = { hold, 0, 0 };
        st_add(EV_PRESS, presses, 0, hv);
        SetEventIrRemoteReleaseBtn(1);
        st_sleep_poll(hold);
        st_release_up();
        st_add(EV_UNPRESS, presses, 0, 0);

        /* accepted = release counter or file number moved on */
        int wait = retry ? 150 : 5000;
        uint32_t tu = get_ms_clock();
        while (st_progress() <= acc && (int)(get_ms_clock() - tu) < wait) st_sleep_poll(10);
        int p = st_progress();
        int got = p > acc;
        if (got && st_progress() > presses) { why = "RUNAWAY (camera fired on its own)"; aborted = 1; runaway = 1; break; }
        if (got)
        {
            uint32_t ta = get_ms_clock();
            st_note(3, p, ta - tp, ST_JOB());
            for (int k = acc; k < p && nsh < ST_SHOT_MAX; k++)
            {
                sh[nsh].press = presses; sh[nsh].hold = hold;
                sh[nsh].t_press = tp - st_t0; sh[nsh].t_acc = ta - st_t0; sh[nsh].step = p - acc;
                sh[nsh].after_probe = probe_mark; probe_mark = 0;
                nsh++;
            }
            if (!t_first_acc) t_first_acc = ta;
            acc = p;
            miss_run = 0;
        }
        else
        {
            st_note(2, presses, hold, ST_JOB());
            miss_run++;
            if (retry && hold == 100 && acc == 0 && miss_run >= 5)
            {
                hold = 300;             /* short holds may not register: fall back to lens.c's 300 ms */
                miss_run = 0;
                st_note(1, presses, 0, 0);
            }
            else if (miss_run >= (retry ? 40 : 3)) { why = "no accept"; aborted = 1; st_note(5, presses, 0, 0); break; }
        }

        if (!frozen && t_first_acc && (int)(get_ms_clock() - t_first_acc) >= 4000)
        {
            st_dm_freeze(run, "after shot 1");
            st_note(4, acc, 0, 0);
            frozen = 1;
        }
        st_guard = retry ? presses : acc;
        if (!retry && got && acc < nshots && !st_wait_idle(3000, 150, 15000)) st_note(6, presses, 0, 0);

        /* retry mode, once after the first shot: 3 quick presses while the camera is busy. Canon dropped such
         * presses in the bracketing tests (lens.c); if it queues them instead, retry mode would leave up to
         * ~30 queued shots per counted one, so stop here (at most 3 extra photos). */
        if (got && !probed)
        {
            probed = 1;
            for (int i = 0; i < 3; i++)
            {
                presses++;
                uint32_t pv[3] = { hold, 0, 0 };
                st_add(EV_PRESS, presses, 0, pv);
                SetEventIrRemoteReleaseBtn(1);
                st_sleep_poll(hold);
                st_release_up();
                st_add(EV_UNPRESS, presses, 0, 0);
                st_sleep_poll(50);
            }
            st_guard = presses;
            st_wait_idle(3000, 500, 15000);
            st_sleep_poll(500);
            int extra = st_progress() - acc;
            st_note(7, extra, 0, 0);
            st_printf("queue probe: 3 presses while busy -> %d extra shot(s)\n", extra);
            if (extra >= 2) { why = "camera queues releases"; aborted = 1; acc = st_progress(); break; }
            acc = st_progress();        /* 0 or 1 extra (camera became ready during the probe): not timed */
            probe_mark = 1;
        }
    }

    /* let Canon finish writing before any card I/O of ours: idle for 2 s (max 30 s) */
    /* (runaway: stop it first, before any waiting) */
    int run_more = runaway ? st_runaway_stop(presses) : 0;
    if (!frozen)
    {
        for (int t = 0; t < 4000 && (runaway || st_progress() <= presses); t += 10) st_sleep_poll(10);
        st_dm_freeze(run, "after shots");
    }
    st_guard = runaway ? 0 : presses;
    int idle = st_wait_idle(0, 2000, 30000);
    if (!runaway && st_progress() > presses)
    {
        why = "RUNAWAY (camera fired on its own)"; aborted = 1; runaway = 1;
        run_more = st_runaway_stop(presses);
        idle = st_wait_idle(0, 2000, 30000);
    }
    st_guard = 0;
    st_sleep_poll(1000);
    st_end(run);

    /* summary */
    st_printf("summary: %s; presses %d, accepted %d of %d%s; RELCNT +%d, FILENO +%d, MECHA +%d, job edges +%d; final idle %s\n",
              why, presses, acc, nshots, aborted ? " (ABORTED)" : "",
              (int)(st_cur[P_RELCNT] - st_rel0), (int)(st_cur[P_FILENO] - st_file0), (int)(st_cur[P_MECHA] - mech0),
              (unsigned)(st_job_edges - edges0), idle > 0 ? "ok" : "TIMEOUT");
    if (runaway) st_printf("RUNAWAY: %d shot(s) after the guard tripped\n", run_more);
    st_printf("shot press# hold t_press t_accept latency interval\n");
    int imin = 0, imax = 0, isum = 0, nint = 0;
    for (int i = 0; i < nsh; i++)
    {
        int lat = sh[i].t_acc - sh[i].t_press;
        int itv = i ? (int)(sh[i].t_acc - sh[i - 1].t_acc) : 0;
        st_printf("%4d %6d %4d %7d %8d %7d %8d%s%s\n", i + 1, sh[i].press, sh[i].hold, (unsigned)sh[i].t_press, (unsigned)sh[i].t_acc, lat, itv,
                  sh[i].step > 1 ? " (several at once)" : "", sh[i].after_probe ? " (after queue probe, not timed)" : "");
        if (i && !sh[i].after_probe)
        {
            if (!nint || itv < imin) imin = itv;
            if (!nint || itv > imax) imax = itv;
            isum += itv;
            nint++;
        }
    }
    if (nint)
    {
        st_printf("interval ms (%d timed): min %d, mean %d, max %d = %d.%02d shots/s at the mean\n",
                  nint, imin, isum / nint, imax, 1000 / MAX(1, isum / nint), (100000 / MAX(1, isum / nint)) % 100);
        snprintf(st_last, sizeof(st_last), "%d/%d shots, mean %d ms", acc, nshots, isum / nint);
    }
    else
        snprintf(st_last, sizeof(st_last), "%d/%d shots (%s)", acc, nshots, why);
    if (runaway) snprintf(st_last, sizeof(st_last), "RUNAWAY: camera fired on its own (%d/%d)", acc, nshots);

    st_save(run);
    free(sh);
    st_busy = 0;
}

/* ---- silent H burst fps patch (v1.3, needs CONFIG_MMU_REMAP) ----
 * Canon plans each frame of a silent High-speed burst 1e8 / fps100 us after the previous one (FUN_e01d74fc,
 * PoReleaseRequestManager.c). fps100 = 500 for scene seq 61/62 (LvSilentHiContiOneshot/Servo) in both period
 * tables (A 0xE0F16BF8, B 0xE0F16A00; 8-byte entries {fps100, lag_us}). Patched through ML's MMU remap of the
 * 64 KB ROM page 0xE0F10000 into RAM: cpu0 only, gone at power off, never written to flash. */
#define ST_FPS_N 4
static const uint32_t st_fps_addr[ST_FPS_N] = { 0xE0F16DE0, 0xE0F16DE8, 0xE0F16BE8, 0xE0F16BF0 };
static const int st_fps_val[] = { 500, 550, 600, 650, 700, 750 };
static int st_fps_sel;                  /* menu choice; not saved, so every boot starts at Canon's 5.0 */
static int st_fps_applied;              /* choice currently in effect */
static int st_fps_err;

static void st_fps_table(uint32_t *o)
{
#ifdef CONFIG_MMU_REMAP
    for (int i = 0; i < ST_FPS_N; i++) o[i] = *(volatile uint32_t *)st_fps_addr[i];
#else
    for (int i = 0; i < ST_FPS_N; i++) o[i] = 0;
#endif
}

static void st_fps_apply(int sel)
{
#ifdef CONFIG_MMU_REMAP
    if (sel == st_fps_applied) return;
    int err = 0;
    if (st_fps_applied != 0)
    {
        for (int i = 0; i < ST_FPS_N; i++) err |= unpatch_memory(st_fps_addr[i]);
        if (!err) st_fps_applied = 0;
    }
    if (!err && sel != 0)
    {
        struct patch p[ST_FPS_N];
        for (int i = 0; i < ST_FPS_N; i++)
        {
            p[i].addr = (uint8_t *)st_fps_addr[i];
            p[i].old_value = 500;       /* apply_patches refuses all four on any mismatch */
            p[i].new_value = st_fps_val[sel];
            p[i].size = 4;
            p[i].description = "Silent H burst fps";
            p[i].is_instruction = 0;
        }
        err = apply_patches(p, ST_FPS_N);
        if (!err) st_fps_applied = sel;
    }
    st_fps_err = err;
    if (err) st_fps_sel = st_fps_applied;
#else
    st_fps_err = -1;
    st_fps_sel = 0;
#endif
}

/* ---- burst log (v1.2): the user holds the shutter in High-speed continuous; ML never presses ---- */
static void st_burst_task(void *unused)
{
    int run = ++st_runs;
    int nmax = st_nshots[st_nsel];
    if (!st_menu_wait()) { snprintf(st_last, sizeof(st_last), "menu still open"); st_busy = 0; return; }

    st_burst_mode = 1;
    st_begin(run, "burst log (user holds the shutter)");
    st_printf("v1.2 burst log: read-only, no presses; stop after %d frames, 3 s, or 600 ms without a frame\n", nmax);
    {
        uint32_t tb[ST_FPS_N];
        st_fps_table(tb);
        st_printf("v1.3 silent H fps patch: %d (err %x); table A61 %d A62 %d B61 %d B62 %d\n",
                  st_fps_val[st_fps_applied], st_fps_err, (int)tb[0], (int)tb[1], (int)tb[2], (int)tb[3]);
    }
    uint32_t rel0 = st_cur[P_RELCNT], mech0 = st_cur[P_MECHA];
    NotifyBox(5000, "Burst log armed: hold the shutter about 3 s now");

    uint32_t tw = get_ms_clock();
    while (st_cur[P_RELCNT] == rel0 && (int)(get_ms_clock() - tw) < 60000) st_sleep_poll(10);

    const char *why = "no frame within 60 s";
    int win = 0;
    if (st_cur[P_RELCNT] != rel0)
    {
        uint32_t t1 = get_ms_clock(), tlast = t1, last = st_cur[P_RELCNT];
        while (1)
        {
            st_sleep_poll(10);
            uint32_t c = st_cur[P_RELCNT];
            if (c != last) { last = c; tlast = get_ms_clock(); }
            if ((int)(c - rel0) >= nmax) { why = "frame limit"; break; }
            if ((int)(get_ms_clock() - t1) >= 3000) { why = "3 s limit"; break; }
            if ((int)(get_ms_clock() - tlast) >= 600) { why = "shutter released"; break; }
        }
        win = (int)(st_cur[P_RELCNT] - rel0);
        st_sleep_poll(300);
        st_dm_freeze(run, "after burst");
    }

    /* Canon is still writing (and the user may still hold the shutter): no card I/O until idle for 2 s */
    int idle = st_wait_idle(0, 2000, 30000);
    st_sleep_poll(500);
    st_end(run);

    int total = (int)(st_cur[P_RELCNT] - rel0);
    st_printf("summary: %s; %d frames in the log window, %d in total; MECHA +%d; SILENT=%x DRIVE=%x; final idle %s\n",
              why, win, total, (int)(st_cur[P_MECHA] - mech0), (unsigned)st_cur[P_SILENT], (unsigned)st_cur[P_DRIVE],
              idle > 0 ? "ok" : "TIMEOUT");

    st_printf("frame t_ms interval_ms\n");
    int n = st_n, k = 0, imin = 0, imax = 0, isum = 0, nint = 0, tprev = 0;
    for (int i = 0; i < n; i++)
    {
        struct st_ev *e = &st_ev[i];
        if (e->kind != EV_PROP || e->id != PROP_RELEASE_COUNTER || (int)(e->v[0] - rel0) <= 0) continue;
        int t = (int)(e->ms - st_t0);
        int itv = k ? t - tprev : 0;
        k++;
        st_printf("%5d %7d %8d\n", k, t, itv);
        if (k > 1)
        {
            if (!nint || itv < imin) imin = itv;
            if (!nint || itv > imax) imax = itv;
            isum += itv;
            nint++;
        }
        tprev = t;
    }
    if (nint)
    {
        int mean = isum / nint;
        st_printf("interval ms (%d): min %d, mean %d, max %d = %d.%02d fps at the mean\n",
                  nint, imin, mean, imax, 1000 / MAX(1, mean), (100000 / MAX(1, mean)) % 100);
        snprintf(st_last, sizeof(st_last), "%d frames, mean %d ms", k, mean);
    }
    else
        snprintf(st_last, sizeof(st_last), "%d frames (%s)", k, why);

    st_save(run);
    st_burst_mode = 0;
    st_busy = 0;
}

static void st_start(void (*task)(void *), const char *name)
{
    if (st_busy) return;
    if (!st_ev) st_ev = malloc(ST_EV_MAX * sizeof(struct st_ev));     /* kept for the session: handlers may hold the pointer */
    if (!st_log) st_log = malloc(ST_LOG_MAX);
    if (!st_ev || !st_log) { snprintf(st_last, sizeof(st_last), "no memory"); return; }
    st_busy = 1;
    task_create(name, 0x1e, 0x2000, task, 0);
}

#ifndef ST_HOST
static MENU_SELECT_FUNC(st_watch_select) { st_start(st_watch_task, "st_watch"); }
static MENU_SELECT_FUNC(st_shot_select)  { st_start(st_shot_task, "st_shots"); }
static MENU_SELECT_FUNC(st_burst_select) { st_start(st_burst_task, "st_burst"); }

static MENU_SELECT_FUNC(st_fps_select)
{
    if (st_busy) return;                /* not while a test runs */
    menu_numeric_toggle(&st_fps_sel, delta, 0, COUNT(st_fps_val) - 1);
    st_fps_apply(st_fps_sel);
}

static MENU_UPDATE_FUNC(st_fps_update)
{
    uint32_t tb[ST_FPS_N];
    st_fps_table(tb);
    int v = st_fps_val[st_fps_applied];
    if (st_fps_err)
    {
        MENU_SET_VALUE("ERROR %x", st_fps_err);
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Patch failed (%x); table %d %d %d %d. Power cycle.", st_fps_err,
                         (int)tb[0], (int)tb[1], (int)tb[2], (int)tb[3]);
    }
    else
    {
        MENU_SET_VALUE(st_fps_applied ? "%d.%d fps" : "Canon %d.%d", v / 100, (v / 10) % 10);
        MENU_SET_WARNING(MENU_WARN_INFO, "Table now %d %d %d %d (500 = Canon). RAM only, gone at power off.",
                         (int)tb[0], (int)tb[1], (int)tb[2], (int)tb[3]);
    }
}

static MENU_UPDATE_FUNC(st_update)
{
    if (st_busy)
        MENU_SET_VALUE("running (close menu)");
    else if (st_last[0])
        MENU_SET_VALUE("%s", st_last);
    else
        MENU_SET_VALUE("ready");
}

static struct menu_entry st_menu[] = {
    {
        .name   = "Silent shutter test",
        .select = menu_open_submenu,
        .help   = "Read-only diagnostics for Canon's electronic (silent) shutter.",
        .help2  = "Logs to ML/SILENT.TXT; Canon log to LOGnnn.LOG on the card root.",
        .children = (struct menu_entry[]) {
            {
                .name   = "Prop watch",
                .select = st_watch_select,
                .update = st_update,
                .help   = "Logs silent/drive/job properties while you change Canon menus.",
                .help2  = "SET, close menu, toggle Silent shutter + Drive, wait for DONE.",
            },
            {
                .name    = "Watch time",
                .priv    = &st_watch_sel,
                .max     = 2,
                .choices = CHOICES("30 s", "90 s", "180 s"),
            },
            {
                .name   = "Shot test",
                .select = st_shot_select,
                .update = st_update,
                .help   = "Fires releases (same call as the intervalometer), logs timing.",
                .help2  = "SET, close menu, keep the camera still, wait for DONE.",
            },
            {
                .name    = "Shot mode",
                .priv    = &st_mode_sel,
                .max     = 1,
                .choices = CHOICES("Paced", "Retry"),
                .help    = "Paced: next press when the camera is idle again.",
                .help2   = "Retry: press again every ~250 ms until Canon accepts it.",
            },
            {
                .name    = "Shots",
                .priv    = &st_nsel,
                .max     = 2,
                .choices = CHOICES("5", "10", "20"),
            },
            {
                .name    = "Silent H fps",
                .priv    = &st_fps_sel,
                .max     = 5,
                .choices = CHOICES("Canon 5.0", "5.5", "6.0", "6.5", "7.0", "7.5"),
                .select  = st_fps_select,
                .update  = st_fps_update,
                .help    = "Patches Canon's silent High-speed burst rate in RAM.",
                .help2   = "Not saved: power off returns to Canon 5.0.",
            },
            {
                .name   = "Burst log",
                .select = st_burst_select,
                .update = st_update,
                .help   = "You hold the shutter in High-speed continuous; ML only logs.",
                .help2  = "Logs the first 5/10/20 frames (Shots). SILENT.TXT + LOGnnn.LOG",
            },
            {
                .name    = "Canon log",
                .priv    = &st_dm_sel,
                .max     = 1,
                .choices = CHOICES("OFF", "ON"),
                .help    = "Also save Canon's debug log (LOGnnn.LOG) for each run.",
            },
            MENU_EOL
        },
    },
};

static void st_init(void *unused)
{
    menu_add("Debug", st_menu, COUNT(st_menu));
}

INIT_FUNC("silenttest", st_init);
#endif
