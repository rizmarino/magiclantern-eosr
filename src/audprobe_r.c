/* riz/eosr-dmlog: audio probe for raw video, stage 2 (EOS R 1.8.0). Personal diagnostic build, not for upstream.
 *
 * Debug -> Audio probe. In movie-mode LiveView (no recording), records 6 s of microphone audio through Canon's
 * SoundDev API (the sequence Canon's factory audio test uses, no movie recorder involved) and writes:
 *   ML/AUDPRB.WAV  48 kHz 16-bit stereo, the samples exactly as Canon's audio block wrote them
 *   ML/AUDPRB.LOG  every step, every chunk (time, sizes, ring addresses), peak/RMS read little- and big-endian
 *   LOGnnn.LOG     Canon's debug log, audio class 0x0C only (one dump at the end)
 * Static analysis: ~/ml/ghidra/notes/lv-survey.md "Audio for raw video: stage 1".
 *
 * Safety: one run per boot; marker ML/SETTINGS/AUD.PND while it runs (if a run froze the camera, the next attempt
 * refuses once and removes it); Canon's callbacks only queue chunk addresses (all copying and card I/O happens in
 * ML's own task); every wait has a timeout; stop/end calls only follow a start that succeeded; SetAudioEnd is
 * skipped if Canon never confirms the stop.
 */
#include "dryos.h"
#include "menu.h"
#include "property.h"
#include "propvalues.h"
#include "fio-ml.h"
#include "mem.h"
#include "mem_defs.h"
#include "bmp.h"

/* Canon functions (R 1.8.0, thumb) */
static int      (*const SoundDev_SetAudioInit)(int mode, int z, int sync, int fmt) = (void *)0xE05EDAF7;
static int      (*const SoundDev_SetRecParam)(int *p)                               = (void *)0xE05EE3F9;
static int      (*const SoundDev_SetMovieStart)(int mode)                           = (void *)0xE05EDB95;
static int      (*const SoundDev_SetInStart)(void)                                  = (void *)0xE05EDDF7;
static int      (*const SoundDev_SetMovieStop)(int mode)                            = (void *)0xE05EDD73;
static int      (*const SoundDev_SetAudioEnd)(int mode)                             = (void *)0xE05EDDAF;
static uint32_t (*const AprocCtrl_GetMovieAdr)(void)                                = (void *)0xE0293C4F;
static uint32_t (*const AprocCtrl_GetMovieSize)(void)                               = (void *)0xE0293C81;
static uint32_t (*const AprocCtrl_GetWriteAdr)(void)                                = (void *)0xE0293BBB;
static uint32_t (*const AprocCtrl_GetRecEndian)(void)                               = (void *)0xE0293DA9;
static uint32_t (*const ACTRL_GetAudioBit)(void)                                    = (void *)0xE024D055;
static int      (*const ACTRL_IsActiveState)(int mask, int lock)                    = (void *)0xE024D0FD;
extern void dm_set_store_level(uint32_t cls, uint32_t level);  /* stubs.S 0xE0563720 */

#define AP_LOG     "ML/AUDPRB.LOG"
#define AP_WAV     "ML/AUDPRB.WAV"
#define AP_MARKER  "ML/SETTINGS/AUD.PND"
#define AP_SECONDS 6
#define AP_BUFLEN  (2 * 1024 * 1024)
#define AP_QMAX    32

/* sync codes from Canon's movie recorder table (fps x100 -> code) */
static const int   ap_sync_code[] = { 4, 7, 5, 6, 1, 2 };
static const char *ap_sync_name[] = { "29.97", "23.98", "25", "24", "59.94", "50" };
static int ap_sync_sel = 0;

static volatile int ap_busy, ap_done_once, ap_start_cb, ap_stop_cb, ap_err_cb, ap_err_code, ap_pause_cb;
static char ap_last[64] = "";

/* chunk queue filled by Canon's audio stream task, drained by ours */
struct ap_q { uint32_t ms, n, bytes, adr1, size1, adr2, size2, from_stop; };
static struct ap_q ap_q[AP_QMAX];
static volatile int ap_qn, ap_qover;

static void ap_queue(int *info, int from_stop)
{
    int i = ap_qn;
    if (i >= AP_QMAX) { ap_qover++; return; }
    ap_q[i].ms = get_ms_clock();
    ap_q[i].n = info ? info[0] : 0;
    ap_q[i].bytes = info ? info[1] : 0;
    ap_q[i].adr1 = info ? info[0x35] : 0; ap_q[i].size1 = info ? info[0x36] : 0;
    ap_q[i].adr2 = info ? info[0x37] : 0; ap_q[i].size2 = info ? info[0x38] : 0;
    ap_q[i].from_stop = from_stop;
    ap_qn = i + 1;
}

/* Canon callbacks: run in Canon's AStrm task. Record and return, nothing else. */
static void ap_cb_start(int inst, int err) { ap_start_cb = 1; ap_err_code = err ? err : ap_err_code; }
static void ap_cb_rec(int inst, int *info) { ap_queue(info, 0); }
static void ap_cb_stop(int inst, int *info) { ap_queue(info, 1); ap_stop_cb = 1; }
static void ap_cb_err(int inst, int code) { ap_err_cb++; ap_err_code = code; }
static void ap_cb_pause(int inst, int *info) { ap_pause_cb++; }

static void ap_log(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(line) - 2) n = sizeof(line) - 2;
    line[n++] = '\n';
    FILE *f = FIO_CreateFileOrAppend(AP_LOG);
    if (f) { FIO_WriteFile(f, line, n); FIO_CloseFile(f); }
}

static int ap_wait(volatile int *flag, int ms)
{
    for (int t = 0; t < ms && !*flag; t += 10) msleep(10);
    return *flag;
}

static uint8_t *ap_buf;
static int ap_len;

/* copy queued chunks out of Canon's ring (uncached alias: Aproc writes by DMA) */
static int ap_drain(int *done)
{
    int copied = 0;
    while (*done < ap_qn)
    {
        struct ap_q *q = &ap_q[*done];
        uint32_t parts[2][2] = { { q->adr1, q->size1 }, { q->adr2, q->size2 } };
        for (int k = 0; k < 2; k++)
        {
            uint32_t a = parts[k][0], s = parts[k][1];
            if (!a || !s || s > 0x100000) continue;
            if (ap_len + (int)s > AP_BUFLEN) s = AP_BUFLEN - ap_len;
            if (!s) continue;
            memcpy(ap_buf + ap_len, UNCACHEABLE(a), s);
            ap_len += s;
            copied += s;
        }
        (*done)++;
    }
    return copied;
}

static int ap_isqrt(int64_t v)
{
    int64_t r = 0, b = (int64_t)1 << 40;
    while (b > v) b >>= 2;
    while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; }
    return (int)r;
}

static void ap_stats(const uint8_t *p, int bytes, int be, int *peak, int *rms, int *lr_diff)
{
    int64_t sq = 0;
    int pk = 0, n = 0, diff = 0;
    for (int i = 0; i + 3 < bytes; i += 4)
    {
        int16_t l = be ? (int16_t)((p[i] << 8) | p[i + 1]) : (int16_t)(p[i] | (p[i + 1] << 8));
        int16_t r = be ? (int16_t)((p[i + 2] << 8) | p[i + 3]) : (int16_t)(p[i + 2] | (p[i + 3] << 8));
        int al = l < 0 ? -l : l;
        if (al > pk) pk = al;
        sq += (int64_t)l * l;
        n++;
        if (l != r) diff++;
    }
    *peak = pk;
    *rms = n ? ap_isqrt(sq / n) : 0;
    *lr_diff = diff;
}

static void ap_write_wav(void)
{
    uint8_t h[44];
    uint32_t data = ap_len & ~3, riff = data + 36, rate = 48000, brate = 48000 * 4;
    memcpy(h, "RIFF", 4); memcpy(h + 4, &riff, 4); memcpy(h + 8, "WAVEfmt ", 8);
    uint32_t fmtlen = 16; uint16_t pcm = 1, ch = 2, align = 4, bits = 16;
    memcpy(h + 16, &fmtlen, 4); memcpy(h + 20, &pcm, 2); memcpy(h + 22, &ch, 2); memcpy(h + 24, &rate, 4);
    memcpy(h + 28, &brate, 4); memcpy(h + 32, &align, 2); memcpy(h + 34, &bits, 2);
    memcpy(h + 36, "data", 4); memcpy(h + 40, &data, 4);
    FILE *f = FIO_CreateFile(AP_WAV);
    if (!f) { ap_log("cannot create %s", AP_WAV); return; }
    FIO_WriteFile(f, h, 44);
    if (data) FIO_WriteFile(f, ap_buf, data);
    FIO_CloseFile(f);
    ap_log("wrote %s: %d bytes of samples (%d.%02d s)", AP_WAV, (int)data, (int)(data / 192000), (int)(data % 192000 / 1920));
}

static void ap_task(void *unused)
{
    int sync = ap_sync_code[ap_sync_sel];
    int init_ok = 0, start_ok = 0, stop_ok = 0, qdone = 0, rc = 0, busy_play = 0, got_start = 0, tick = 0;
    uint32_t sz, madr = 0, msize = 0, t0 = get_ms_clock();
    static int p[19];
    ap_qn = 0; ap_qover = 0; ap_start_cb = ap_stop_cb = ap_err_cb = ap_err_code = ap_pause_cb = 0; ap_len = 0;

    for (int i = 0; i < 300 && gui_menu_shown(); i++) msleep(100);
    if (gui_menu_shown()) { snprintf(ap_last, sizeof(ap_last), "menu still open"); ap_busy = 0; return; }
    msleep(1000);

    if (FIO_GetFileSize(AP_MARKER, &sz) == 0)
    {
        FIO_RemoveFile(AP_MARKER);
        snprintf(ap_last, sizeof(ap_last), "last run didn't finish: refused once");
        NotifyBox(6000, "Audio probe: the last run didn't finish. Refused once; SET again to retry.");
        ap_busy = 0; return;
    }
    if (!lv || lv_movie_select != LVMS_ENABLE_MOVIE || RECORDING)
    {
        snprintf(ap_last, sizeof(ap_last), "needs movie LiveView, not recording");
        NotifyBox(5000, "Audio probe: switch to video mode LiveView first.");
        ap_busy = 0; return;
    }
    ap_buf = fio_malloc(AP_BUFLEN);
    if (!ap_buf) { snprintf(ap_last, sizeof(ap_last), "no memory"); ap_busy = 0; return; }
    { FILE *m = FIO_CreateFile(AP_MARKER); if (m) FIO_CloseFile(m); }
    ap_done_once = 1;

    ap_log("=== audio probe v1, sync %d (%s), t=%d ===", sync, ap_sync_name[ap_sync_sel], (int)get_ms_clock());
    busy_play = ACTRL_IsActiveState(0x11, 0);
    ap_log("pre: IsActiveState(0x11)=%d AudioBit=%x MEM2 adr=%x size=%x endian=%d",
           busy_play, (int)ACTRL_GetAudioBit(), (int)AprocCtrl_GetMovieAdr(), (int)AprocCtrl_GetMovieSize(),
           (int)AprocCtrl_GetRecEndian());
    if (busy_play) { ap_log("abort: audio is in a playback state"); snprintf(ap_last, sizeof(ap_last), "audio busy (playback)"); goto out; }

    dm_set_store_level(0xFF, 15);
    dm_set_store_level(0x0C, 2);
    DryosDebugMsg(0, 15, "=== riz audprb start");

    rc = SoundDev_SetAudioInit(1, 0, sync, 1);
    ap_log("SetAudioInit(1,0,%d,1) = %d; AudioBit=%x", sync, rc, (int)ACTRL_GetAudioBit());
    if (rc) { snprintf(ap_last, sizeof(ap_last), "SetAudioInit failed (%d)", rc); goto out_log; }
    init_ok = 1;
    for (int i = 0; i < 100 && (!AprocCtrl_GetMovieAdr() || !AprocCtrl_GetMovieSize()); i++) msleep(10);
    madr = AprocCtrl_GetMovieAdr(); msize = AprocCtrl_GetMovieSize();
    ap_log("MEM2 adr=%x size=%x endian=%d", (int)madr, (int)msize, (int)AprocCtrl_GetRecEndian());
    if (!madr || !msize) { snprintf(ap_last, sizeof(ap_last), "no audio ring after init"); goto out_end; }

    memset(p, 0, sizeof(p));
    p[0] = 48000; p[1] = 16; p[2] = 2; p[5] = 1; p[8] = sync;
    p[9] = (int)ap_cb_start; p[10] = (int)ap_cb_rec; p[11] = (int)ap_cb_stop; p[12] = 0;
    p[13] = (int)ap_cb_err; p[15] = (int)ap_cb_pause;
    rc = SoundDev_SetRecParam(p);
    ap_log("SetRecParam = %d", rc);
    if (rc) { snprintf(ap_last, sizeof(ap_last), "SetRecParam failed (%d)", rc); goto out_end; }

    rc = SoundDev_SetMovieStart(2);
    ap_log("SetMovieStart(2) = %d", rc);
    if (rc) { snprintf(ap_last, sizeof(ap_last), "SetMovieStart failed (%d)", rc); goto out_end; }
    start_ok = 1;
    got_start = ap_wait(&ap_start_cb, 2000);
    rc = SoundDev_SetInStart();
    ap_log("StartCBR %s; SetInStart = %d; write adr=%x", got_start ? "yes" : "NO (2 s)", rc, (int)AprocCtrl_GetWriteAdr());

    NotifyBox(3000, "Audio probe: recording 6 s - clap at 1 s and 4 s");
    t0 = get_ms_clock();
    while ((int)(get_ms_clock() - t0) < AP_SECONDS * 1000)
    {
        msleep(100);
        if (++tick % 5 == 0) ap_log("t=%d write adr=%x chunks=%d err=%d/%x", (int)(get_ms_clock() - t0),
                                    (int)AprocCtrl_GetWriteAdr(), ap_qn, ap_err_cb, ap_err_code);
        ap_drain(&qdone);
    }

    rc = SoundDev_SetMovieStop(3);
    stop_ok = ap_wait(&ap_stop_cb, 3000);
    ap_drain(&qdone);
    ap_log("SetMovieStop(3) = %d; StopCBR %s", rc, stop_ok ? "yes" : "NO (3 s)");
    start_ok = 0;

out_end:
    if (init_ok && (stop_ok || !start_ok))
    {
        rc = SoundDev_SetAudioEnd(4);
        ap_log("SetAudioEnd(4) = %d; AudioBit=%x", rc, (int)ACTRL_GetAudioBit());
    }
    else if (init_ok)
        ap_log("SetAudioEnd skipped: no StopCBR. Power-cycle the camera before using audio.");

out_log:
    DryosDebugMsg(0, 15, "=== riz audprb end");
    dm_set_store_level(0xFF, 15);
    msleep(200);
    call("dumpf");

    /* chunk table and sample statistics */
    ap_log("chunks %d (lost %d), err CBR %d code %x, pause CBR %d, copied %d bytes", ap_qn, ap_qover, ap_err_cb,
           ap_err_code, ap_pause_cb, ap_len);
    for (int i = 0; i < ap_qn; i++)
        ap_log("chunk %d: t=%d n=%d bytes=%d adr1=%x size1=%d adr2=%x size2=%d%s", i, (int)(ap_q[i].ms - t0),
               (int)ap_q[i].n, (int)ap_q[i].bytes, (int)ap_q[i].adr1, (int)ap_q[i].size1, (int)ap_q[i].adr2,
               (int)ap_q[i].size2, ap_q[i].from_stop ? " (stop)" : "");
    for (int off = 0; off + 48000 <= ap_len; off += 48000)    /* 0.25 s windows */
    {
        int pl, rl, dl, pb, rb, db;
        ap_stats(ap_buf + off, 48000, 0, &pl, &rl, &dl);
        ap_stats(ap_buf + off, 48000, 1, &pb, &rb, &db);
        ap_log("win %d.%02d s: LE peak %d rms %d | BE peak %d rms %d | L!=R %d", off / 192000, off % 192000 / 1920,
               pl, rl, pb, rb, dl);
    }
    if (ap_len) ap_write_wav();
    if (!ap_last[0] || ap_len)
        snprintf(ap_last, sizeof(ap_last), "%d chunks, %d.%d s", ap_qn, ap_len / 192000, ap_len % 192000 / 19200);

out:
    if (ap_buf) { fio_free(ap_buf); ap_buf = 0; }
    FIO_RemoveFile(AP_MARKER);
    NotifyBox(6000, "Audio probe DONE: %s", ap_last);
    ap_busy = 0;
}

static MENU_SELECT_FUNC(ap_select)
{
    if (ap_busy) return;
    if (ap_done_once) { NotifyBox(4000, "Audio probe: one run per power-on. Restart the camera to run again."); return; }
    ap_busy = 1;
    ap_last[0] = 0;
    task_create("audprb", 0x1e, 0x2000, ap_task, 0);
}

static MENU_UPDATE_FUNC(ap_update)
{
    if (ap_busy) MENU_SET_VALUE("running (close menu)");
    else if (ap_last[0]) MENU_SET_VALUE("%s", ap_last);
    else MENU_SET_VALUE("ready");
}

static struct menu_entry ap_menu[] = {
    {
        .name   = "Audio probe",
        .select = menu_open_submenu,
        .update = ap_update,
        .help   = "Test: record 6 s of mic audio via Canon's SoundDev API (video mode, no raw).",
        .help2  = "Writes ML/AUDPRB.WAV + ML/AUDPRB.LOG. One run per power-on.",
        .children = (struct menu_entry[]) {
            {
                .name   = "Run",
                .select = ap_select,
                .update = ap_update,
                .help   = "SET, close the menu, then clap at about 1 s and 4 s. Wait for DONE.",
            },
            {
                .name    = "Canon frame rate",
                .priv    = &ap_sync_sel,
                .max     = 5,
                .choices = CHOICES("29.97", "23.98", "25", "24", "59.94", "50"),
                .help    = "Set to match Canon's movie frame rate (audio sync code).",
            },
            MENU_EOL
        },
    },
};

static void ap_init(void *unused)
{
    menu_add("Debug", ap_menu, COUNT(ap_menu));
}

INIT_FUNC("audprobe", ap_init);
