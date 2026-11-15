/* Doom sound effects + music for the EOS R (portable part; built for the camera and the host harness).
 *
 * Output: one continuous 48 kHz s16 stereo stream. The platform pulls it in chunks with dm_snd_mix() from its own
 * audio context (camera: Canon's movie-play audio path, gapless, camera-verified 2026-11-15 by the Stream test;
 * the older AudioBeep path played one mono buffer at a time with ~12 ms gaps, so it could not carry music).
 *
 * Effects: a software mixer over Doom's channels (8-bit 11/22 kHz lumps, linear interpolation to 48 kHz, Doom's
 * volume 0..127 and stereo separation 0..254 as in Chocolate Doom's i_sdlsound.c). The game task never touches the
 * mixer state: it posts per-channel requests (start / stop / new volume) that the mixer applies at the start of
 * each chunk. On the camera the mixer task has a higher priority than the game task, so it never sees a request
 * half-written: the request fields are written first and the sequence number last.
 *
 * Music: pre-rendered (on the PC, with Chocolate Doom's OPL music code) to ML/DOOM/MUSIC/<LUMP>.WAV, mono s16
 * 24 kHz. dm_mus_service() (called by the platform from a context that may block on the card) streams the file into
 * a ring buffer; the mixer upsamples x2 and adds it to both channels. No file = no music, nothing else changes.
 *
 * Platform side (dm_sys.h): dm_snd_hw_open / dm_snd_hw_close; the platform calls dm_snd_mix and dm_mus_service. */

#include <stdio.h>
#include <string.h>

#include "doomtype.h"
#include "i_sound.h"
#include "w_wad.h"
#include "z_zone.h"
#include "deh_str.h"
#include "m_misc.h"
#include "doomgeneric.h"
#include "dm_sys.h"

#define DM_NCH          16              /* Doom asks for snd_channels (8); room for more */
/* gain: s16 per 8-bit unit = vol * DM_SND_GAIN; at the default volume (sfx_volume 8 -> 64) a full-scale effect peaks
 * at full scale (Chocolate Doom: half). The AudioBeep driver used x3 (12) for one effect at a time; with up to 8 mixed,
 * host demo1 put 32% of samples into the limiter at 12, 5% at 4. Loudness comes from the speaker level instead. */
#define DM_SND_GAIN     4
/* Doom's effects are normalised to full scale and several can play at once: a soft knee on the mix keeps the
 * loudness without hard-clipping edges */
#define DM_SND_KNEE     20000

/* ------------------------------------------------------------------ effects */

struct sfx_data { const uint8_t *s; uint32_t n; uint32_t step; };   /* step: 16.16 source samples per 48 kHz frame */

enum { REQ_NONE, REQ_START, REQ_STOP, REQ_PARAMS };
struct req { const struct sfx_data *sfx; int gl, gr; int op; };
struct mch { const struct sfx_data *sfx; uint32_t pos; int gl, gr; };

static struct req        req[DM_NCH];
static volatile uint32_t req_seq[DM_NCH];      /* written by the game task (last), read by the mixer */
static volatile uint32_t req_done[DM_NCH];     /* written by the mixer */
static struct mch        mch[DM_NCH];          /* mixer only */
static volatile uint8_t  ch_playing[DM_NCH];   /* written by the mixer */

static volatile int ready;
static int use_prefix;
static int n_start, n_stop, n_bad, n_clip_samples, n_chunks;

static boolean Snd_Init(boolean prefix)
{
    use_prefix = prefix;
    memset(req, 0, sizeof req);
    memset(mch, 0, sizeof mch);
    for (int i = 0; i < DM_NCH; i++) { req_seq[i] = req_done[i] = 0; ch_playing[i] = 0; }
    if (dm_snd_hw_open() != 0)
    {
        printf("DM sound: no output\n");
        return false;
    }
    ready = 1;
    printf("DM sound: 48 kHz stereo stream, software mix of %d channels, music from ML/DOOM/MUSIC\n", DM_NCH);
    return true;
}

static void Snd_Shutdown(void)
{
    if (ready) dm_snd_hw_close();
    ready = 0;
}

static int Snd_GetSfxLumpNum(sfxinfo_t *sfx)
{
    char name[16];
    if (sfx->link) sfx = sfx->link;
    if (use_prefix) M_snprintf(name, sizeof name, "ds%s", DEH_String(sfx->name));
    else M_StringCopy(name, DEH_String(sfx->name), sizeof name);
    return W_CheckNumForName(name);
}

static void Snd_Update(void) { }

/* Doom: vol 0..127, sep 0..254 (128 = centre). Chocolate Doom i_sdlsound: left = (254 - sep) * vol / 127. */
static void gains(int vol, int sep, int *gl, int *gr)
{
    if (vol < 0) vol = 0;
    if (vol > 127) vol = 127;
    if (sep < 0) sep = 0;
    if (sep > 254) sep = 254;
    *gl = (254 - sep) * vol / 127 * DM_SND_GAIN;
    *gr = sep * vol / 127 * DM_SND_GAIN;
}

static void post(int ch, int op, const struct sfx_data *sfx, int vol, int sep)
{
    struct req *r = &req[ch];
    int gl, gr;
    gains(vol, sep, &gl, &gr);
    if (op == REQ_PARAMS && req_seq[ch] != req_done[ch] && r->op == REQ_START)
    {
        r->gl = gl; r->gr = gr;                     /* start not applied yet: just update its volume */
        return;
    }
    r->op = op; r->sfx = sfx; r->gl = gl; r->gr = gr;
    req_seq[ch] = req_seq[ch] + 1;                  /* publish last */
}

static void Snd_UpdateSoundParams(int channel, int vol, int sep)
{
    if (channel < 0 || channel >= DM_NCH) return;
    post(channel, REQ_PARAMS, NULL, vol, sep);
}

/* cache the effect once, PU_STATIC for good: the mixer reads it from another task, so the zone must never purge it */
static const struct sfx_data *get_sfx(sfxinfo_t *sfx)
{
    if (sfx->driver_data) return sfx->driver_data;
    int len = W_LumpLength(sfx->lumpnum);
    const uint8_t *d = W_CacheLumpNum(sfx->lumpnum, PU_STATIC);
    int rate = d[2] | (d[3] << 8), n = d[4] | (d[5] << 8) | (d[6] << 16) | (d[7] << 24);
    if (len < 8 || d[0] != 3 || d[1] != 0 || n > len - 8 || n <= 48 || rate < 4000 || rate > 48000)
    {
        W_ReleaseLumpNum(sfx->lumpnum);
        return NULL;
    }
    struct sfx_data *s = Z_Malloc(sizeof *s, PU_STATIC, NULL);
    s->s = d + 8 + 16;                              /* the DMX library skips the first and last 16 bytes (as Chocolate) */
    s->n = n - 32;
    s->step = ((uint32_t)rate << 16) / 48000;
    sfx->driver_data = s;
    return s;
}

static int Snd_StartSound(sfxinfo_t *sfx, int channel, int vol, int sep)
{
    if (!ready || sfx->lumpnum < 0 || channel < 0 || channel >= DM_NCH) return -1;
    const struct sfx_data *s = get_sfx(sfx);
    if (!s) { n_bad++; return -1; }
    post(channel, REQ_START, s, vol, sep);
    n_start++;
    return channel;
}

static void Snd_StopSound(int channel)
{
    if (channel < 0 || channel >= DM_NCH) return;
    post(channel, REQ_STOP, NULL, 0, 128);
    n_stop++;
}

static boolean Snd_SoundIsPlaying(int channel)
{
    if (channel < 0 || channel >= DM_NCH) return false;
    if (req_seq[channel] != req_done[channel]) return req[channel].op == REQ_START || ch_playing[channel];
    return ch_playing[channel];
}

static void Snd_CacheSounds(sfxinfo_t *sounds, int num_sounds) { }

/* ------------------------------------------------------------------ music (pre-rendered WAV, streamed) */

#define MUS_RING        (128 * 1024)            /* samples (24 kHz mono): 5.5 s */
#define MUS_BLOCK       (16 * 1024)             /* samples per card read (32 KB) */

static int16_t          *mus_ring;
static volatile uint32_t mus_wr, mus_rd;        /* free-running sample counters; wr by service, rd by mixer */
static volatile int      mus_on;                /* mixer may read the ring */
static volatile int      mus_paused;
static volatile int      mus_vol = 64;          /* 0..127 */
static volatile uint32_t mus_mixed_gen;         /* chunks mixed, for the service's hand-over wait */
static int16_t           mus_last;              /* mixer: previous source sample (upsampling) */

/* requests from the game task to the service */
static volatile uint32_t mus_req_seq;
static uint32_t          mus_req_done;
static char              mus_req_name[9];
static volatile int      mus_req_play, mus_req_loop;

/* service state */
static void   *mus_file;
static long    mus_data0, mus_data_end;         /* byte offsets of the sample data */
static long    mus_pos;
static int     mus_loop, mus_missing_logged;
static int     n_mus_underrun, n_mus_reads, n_mus_open, n_mus_loops;

static boolean Mus_Init(void)
{
    if (!mus_ring) mus_ring = Z_Malloc(MUS_RING * 2, PU_STATIC, NULL);
    mus_wr = mus_rd = 0; mus_on = 0; mus_paused = 0;
    return true;
}
static void Mus_Shutdown(void)
{
    mus_req_play = 0; mus_req_seq = mus_req_seq + 1;
}
static void Mus_SetVolume(int v) { mus_vol = v < 0 ? 0 : v > 127 ? 127 : v; }
static void Mus_Pause(void)  { mus_paused = 1; }
static void Mus_Resume(void) { mus_paused = 0; }

/* the engine passes the cached lump data; find which lump it is (D_E1M1 ...) */
static void *Mus_Register(void *data, int len)
{
    for (unsigned i = 0; i < numlumps; i++)
        if (lumpinfo[i].cache == data) return (void *)(uintptr_t)(i + 1);
    return NULL;
}
static void Mus_UnRegister(void *h) { }

static void Mus_Play(void *h, boolean looping)
{
    if (!h) return;
    unsigned i = (unsigned)(uintptr_t)h - 1;
    memcpy(mus_req_name, lumpinfo[i].name, 8);
    mus_req_name[8] = 0;
    mus_req_loop = looping;
    mus_req_play = 1;
    mus_req_seq = mus_req_seq + 1;                  /* publish last */
}
static void Mus_Stop(void)
{
    mus_req_play = 0;
    mus_req_seq = mus_req_seq + 1;
}
static boolean Mus_IsPlaying(void) { return mus_on || (mus_req_seq != mus_req_done && mus_req_play); }
static void Mus_Poll(void) { }

static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

/* open ML/DOOM/MUSIC/<name>.WAV: must be PCM 16-bit mono 24000 Hz; finds the data chunk */
static int mus_open(const char *name)
{
    char path[64];
    M_snprintf(path, sizeof path, "ML/DOOM/MUSIC/%s.WAV", name);
    void *f = dm_io_open(path, 0);
    if (!f)
    {
        if (!mus_missing_logged++) printf("DM music: no %s (music off for missing tracks)\n", path);
        return -1;
    }
    static uint8_t h[512];
    int n = dm_io_read(f, h, sizeof h);
    long off = 12, data0 = -1, datalen = 0;
    int fmt_ok = 0;
    if (n < 44 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) n = 0;
    while (n && off + 8 <= n)
    {
        uint32_t sz = rd32(h + off + 4);
        if (!memcmp(h + off, "fmt ", 4) && off + 24 <= n)
            fmt_ok = (h[off + 8] | h[off + 9] << 8) == 1 && (h[off + 10] | h[off + 11] << 8) == 1
                     && rd32(h + off + 12) == 24000 && (h[off + 22] | h[off + 23] << 8) == 16;
        if (!memcmp(h + off, "data", 4)) { data0 = off + 8; datalen = sz; break; }
        off += 8 + sz + (sz & 1);
    }
    if (!fmt_ok || data0 < 0)
    {
        printf("DM music: %s is not PCM 16-bit mono 24 kHz WAV\n", path);
        dm_io_close(f);
        return -1;
    }
    long size = dm_io_size(f);
    if (data0 + datalen > size) datalen = size - data0;
    mus_file = f; mus_data0 = data0; mus_data_end = data0 + (datalen & ~1L); mus_pos = data0;
    if (dm_io_seek(f, data0)) { dm_io_close(f); mus_file = NULL; return -1; }
    n_mus_open++;
    return 0;
}

static void mus_close(void)
{
    if (mus_file) dm_io_close(mus_file);
    mus_file = NULL;
}

/* read up to MUS_BLOCK samples into the ring; returns samples added (0 when full or at the end of the song) */
static int mus_fill_block(void)
{
    uint32_t space = MUS_RING - (mus_wr - mus_rd);
    if (space < MUS_BLOCK) return 0;
    if (mus_pos >= mus_data_end)
    {
        if (!mus_loop) return 0;
        if (dm_io_seek(mus_file, mus_data0)) return 0;
        mus_pos = mus_data0;
        n_mus_loops++;
    }
    /* whole blocks only, so a block never wraps the ring (MUS_RING is a multiple of MUS_BLOCK); the write
     * position is kept block-aligned by padding a short tail (end of file) with silence */
    long want = MUS_BLOCK * 2;
    if (want > mus_data_end - mus_pos) want = mus_data_end - mus_pos;
    uint32_t at = mus_wr % MUS_RING;
    int r = dm_io_read(mus_file, mus_ring + at, want);
    if (r <= 0) { mus_pos = mus_data_end; return 0; }
    mus_pos += r;
    n_mus_reads++;
    int got = r / 2;
    if (got < MUS_BLOCK)
    {
        if (mus_loop && mus_pos >= mus_data_end && got > 0)
        {
            /* loop: fill the rest of the block from the start of the song (seamless loop) */
            while (got < MUS_BLOCK && dm_io_seek(mus_file, mus_data0) == 0)
            {
                long w2 = (MUS_BLOCK - got) * 2;
                if (w2 > mus_data_end - mus_data0) w2 = mus_data_end - mus_data0;
                int r2 = dm_io_read(mus_file, mus_ring + at + got, w2);
                if (r2 <= 0) break;
                got += r2 / 2;
                mus_pos = mus_data0 + r2;
                n_mus_loops++;
                if (got < MUS_BLOCK && mus_pos < mus_data_end) break;   /* short read: card error, pad below */
            }
        }
        if (got < MUS_BLOCK) memset(mus_ring + at + got, 0, (MUS_BLOCK - got) * 2);
        got = MUS_BLOCK;
    }
    mus_wr = mus_wr + got;                          /* publish after the data */
    return got;
}

/* platform: call regularly (camera: every ~20 ms from a low-priority task), never from the audio context */
void dm_mus_service(void)
{
    if (!mus_ring) return;
    uint32_t s = mus_req_seq;
    if (s != mus_req_done)
    {
        mus_req_done = s;
        int play = mus_req_play, loop = mus_req_loop;
        char name[9];
        memcpy(name, mus_req_name, 9);
        /* hand over: stop the mixer reading, wait for it to finish the chunk it may be in, then reset the ring */
        mus_on = 0;
        uint32_t g = mus_mixed_gen;
        for (int i = 0; i < 20 && mus_mixed_gen == g && ready; i++) dm_sys_sleep(5);
        mus_close();
        mus_wr = mus_rd = 0;
        mus_last = 0;
        if (play && mus_open(name) == 0)
        {
            mus_loop = loop;
            while (mus_wr < MUS_RING / 2 && mus_fill_block() > 0) { }
            mus_on = 1;
        }
    }
    if (mus_on && mus_file) while (mus_fill_block() > 0) { }
    if (mus_on && mus_file && !mus_loop && mus_pos >= mus_data_end && mus_wr == mus_rd) { mus_on = 0; mus_close(); }
}

/* game shutdown: close the file (the platform stops calling dm_mus_service first) */
void dm_mus_release(void)
{
    mus_on = 0;
    mus_close();
}

/* ------------------------------------------------------------------ the mix (audio context) */

static inline int16_t limit(int v)
{
    int av = v < 0 ? -v : v;
    if (av > DM_SND_KNEE)           /* soft limiter: above the knee, approach full scale instead of hard clipping */
    {
        int over = av - DM_SND_KNEE;
        if (over > (1 << 22)) over = 1 << 22;
        av = DM_SND_KNEE + over * (32767 - DM_SND_KNEE) / (over + (32767 - DM_SND_KNEE));
        n_clip_samples++;
        return v < 0 ? -av : av;
    }
    return v;
}

#define MIX_MAX 2048
static int32_t accl[MIX_MAX], accr[MIX_MAX];

void dm_snd_mix(int16_t *out, int frames)
{
    if (frames > MIX_MAX) frames = MIX_MAX;
    memset(accl, 0, frames * 4);
    memset(accr, 0, frames * 4);

    for (int c = 0; c < DM_NCH; c++)
    {
        uint32_t s = req_seq[c];
        if (s != req_done[c])
        {
            struct req r = req[c];
            req_done[c] = s;
            if (r.op == REQ_START) { mch[c].sfx = r.sfx; mch[c].pos = 0; mch[c].gl = r.gl; mch[c].gr = r.gr; }
            else if (r.op == REQ_STOP) mch[c].sfx = NULL;
            else if (r.op == REQ_PARAMS) { mch[c].gl = r.gl; mch[c].gr = r.gr; }
        }
        struct mch *m = &mch[c];
        if (!m->sfx) { ch_playing[c] = 0; continue; }
        const uint8_t *src = m->sfx->s;
        uint32_t n = m->sfx->n, step = m->sfx->step, x = m->pos;
        int gl = m->gl, gr = m->gr;
        int i;
        for (i = 0; i < frames; i++, x += step)
        {
            uint32_t i0 = x >> 16;
            if (i0 >= n) break;
            int a = src[i0] - 128, b = (i0 + 1 < n ? src[i0 + 1] : src[i0]) - 128;
            int v = (a << 8) + (((b - a) * (int)(x & 0xffff)) >> 8);     /* 8-bit unit = 256 */
            accl[i] += (v * gl) >> 8;
            accr[i] += (v * gr) >> 8;
        }
        m->pos = x;
        if (i < frames || (x >> 16) >= n) m->sfx = NULL;
        ch_playing[c] = m->sfx != NULL;
    }

    /* music: 24 kHz mono -> 48 kHz (linear: midpoint, then the sample), gain 256 = 1.0 at volume 64 */
    if (mus_on && !mus_paused)
    {
        int g = mus_vol * 512 / 127;
        uint32_t rd = mus_rd, avail = mus_wr - rd;
        int16_t last = mus_last;
        for (int i = 0; i + 1 < frames; i += 2)
        {
            if (!avail) { n_mus_underrun++; break; }
            int16_t cur = mus_ring[rd % MUS_RING];
            rd++; avail--;
            int mid = (((last + cur) >> 1) * g) >> 8, now = (cur * g) >> 8;
            accl[i] += mid; accr[i] += mid;
            accl[i + 1] += now; accr[i + 1] += now;
            last = cur;
        }
        mus_last = last;
        mus_rd = rd;
    }
    mus_mixed_gen = mus_mixed_gen + 1;

    for (int i = 0; i < frames; i++)
    {
        out[2 * i] = limit(accl[i]);
        out[2 * i + 1] = limit(accr[i]);
    }
    n_chunks++;
}

int dm_snd_stats(char *buf, size_t n)
{
    return snprintf(buf, n, "sound: %d started, %d stopped, %d bad lumps, %d chunks mixed, %d samples soft-limited; "
                    "music: %d tracks opened, %d reads, %d loops, %d underruns\n", n_start, n_stop, n_bad, n_chunks,
                    n_clip_samples, n_mus_open, n_mus_reads, n_mus_loops, n_mus_underrun);
}

static snddevice_t dm_snd_devices[] =
{
    SNDDEVICE_SB, SNDDEVICE_PAS, SNDDEVICE_GUS, SNDDEVICE_WAVEBLASTER, SNDDEVICE_SOUNDCANVAS, SNDDEVICE_AWE32,
};

sound_module_t DM_sound_module =
{
    dm_snd_devices,
    sizeof dm_snd_devices / sizeof dm_snd_devices[0],
    Snd_Init,
    Snd_Shutdown,
    Snd_GetSfxLumpNum,
    Snd_Update,
    Snd_UpdateSoundParams,
    Snd_StartSound,
    Snd_StopSound,
    Snd_SoundIsPlaying,
    Snd_CacheSounds,
};

music_module_t DM_music_module =
{
    dm_snd_devices,
    sizeof dm_snd_devices / sizeof dm_snd_devices[0],
    Mus_Init,
    Mus_Shutdown,
    Mus_SetVolume,
    Mus_Pause,
    Mus_Resume,
    Mus_Register,
    Mus_UnRegister,
    Mus_Play,
    Mus_Stop,
    Mus_IsPlaying,
    Mus_Poll,
};
