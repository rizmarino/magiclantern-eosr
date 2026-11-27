//
// musrender: render Doom MUS music lumps to WAV using Chocolate Doom's own
// OPL (Adlib / Sound Blaster FM) music code, offline and faster than
// real time.
//
//   i_oplmusic.c + mus2mid.c + midifile.c + opl.c + opl_queue.c + opl3.c
//   are taken unmodified from Chocolate Doom (see choc/UPSTREAM_COMMIT).
//   opl_offline.c replaces opl_sdl.c (SDL audio callback -> offline clock).
//
// Pipeline per lump:
//   MUS -> (mus2mid) -> MIDI -> i_oplmusic (DMX v1.9 OPL2 voice allocation,
//   GENMIDI instruments from the WAD) -> Nuked OPL3 @ 49716 Hz (native rate,
//   OPL2 mode; L/R differ slightly because of the emulated YMF262
//   channel-sample-delay quirk) -> mono (L+R)/2 -> Kaiser windowed-sinc
//   resample -> 5 Hz DC blocker (AC coupling) -> one fixed gain -> 16-bit WAV.
//
// One pass only: rendering stops at the MUS score-end event; at that point
// any still-held notes are released (I_OPL_StopSong) and the release tail is
// rendered; trailing digital silence is trimmed, but never earlier than
// 0.5 s after the last key-off.
//
// GPL-2.0-or-later.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <ctype.h>
#include <errno.h>
#include <sys/stat.h>

#include "doomtype.h"
#include "i_sound.h"
#include "opl.h"

#include "opl_offline.h"
#include "wadfile.h"

#define NATIVE_RATE     49716
#define MAX_SONG_SECS   900
#define TAIL_SECS       2.0     // rendered after score end (release tail)
#define KEEP_AFTER_OFF  0.5     // always kept after the last key-off
#define MAX_AFTER_OFF   1.5     // tail cap after the last key-off
#define SILENCE_WIN     0.010   // trailing-silence detector window (s)
#define SILENCE_RMS     8.0     // window RMS (LSB) counted as silence (~-72 dBFS)

typedef struct
{
    char name[9];
    float *pcm;              // resampled mono, gain 1.0 (emulator units)
    size_t len;              // samples at out rate (untrimmed)
    size_t eos;              // score-end position (out rate)
    size_t last_off;         // last key-off position (out rate)
    int held_at_eos;         // voices still keyed on at score end
    int16_t raw_peak;        // peak of raw emulator output (native rate)
    size_t lr_mismatch;      // native frames where L != R
    double native_secs;
    // filled after gain:
    size_t out_len;
    int tail_capped;         // release tail still ringing at the cap
    int peak;
    double rms;
    long file_size;
} track_t;

// ---------------------------------------------------------------------------
// Resampler: Kaiser-windowed sinc, exact rational phase (polyphase table).

static double BesselI0(double x)
{
    double sum = 1.0, term = 1.0, k;
    for (k = 1.0; k < 50.0; k += 1.0)
    {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
        if (term < sum * 1e-17) break;
    }
    return sum;
}

static unsigned long Gcd(unsigned long a, unsigned long b)
{
    while (b) { unsigned long t = a % b; a = b; b = t; }
    return a;
}

typedef struct
{
    unsigned long L, M;   // out = in * L / M
    int half;             // taps each side
    float *table;         // L phases x (2*half) taps
} resampler_t;

static void Resampler_Init(resampler_t *r, unsigned long in_rate,
                           unsigned long out_rate, double cutoff_hz,
                           int half, double beta)
{
    unsigned long g = Gcd(in_rate, out_rate);
    unsigned long p;
    int k;
    double fc = cutoff_hz / in_rate;   // cycles per input sample
    double i0b = BesselI0(beta);

    r->L = out_rate / g;
    r->M = in_rate / g;
    r->half = half;
    if (r->L > 100000)
    {
        fprintf(stderr, "resampler: rate ratio too awkward (L=%lu)\n", r->L);
        exit(1);
    }
    r->table = malloc(sizeof(float) * r->L * 2 * half);

    for (p = 0; p < r->L; ++p)
    {
        double frac = (double) p / r->L;
        double sum = 0.0;
        double tmp[1024];
        for (k = 0; k < 2 * half; ++k)
        {
            // tap k multiplies x[i0 - half + 1 + k]; distance t from the
            // ideal (fractional) position.
            double t = (double) (k - half + 1) - frac;
            double s, w, u;
            s = (t == 0.0) ? 2.0 * fc
                           : sin(2.0 * M_PI * fc * t) / (M_PI * t);
            u = t / half;
            w = (fabs(u) >= 1.0) ? 0.0 : BesselI0(beta * sqrt(1.0 - u * u)) / i0b;
            tmp[k] = s * w;
            sum += tmp[k];
        }
        for (k = 0; k < 2 * half; ++k)
        {
            r->table[p * 2 * half + k] = (float) (tmp[k] / sum);  // unity DC
        }
    }
}

static float *Resample(resampler_t *r, const float *in, size_t in_len,
                       size_t *out_len)
{
    size_t n_out = (size_t) (((unsigned long long) in_len * r->L) / r->M);
    float *out = malloc(sizeof(float) * (n_out ? n_out : 1));
    size_t n;
    int k;

    for (n = 0; n < n_out; ++n)
    {
        unsigned long long pos = (unsigned long long) n * r->M;
        long long i0 = (long long) (pos / r->L);
        unsigned long ph = (unsigned long) (pos % r->L);
        const float *h = r->table + (size_t) ph * 2 * r->half;
        double acc = 0.0;
        long long base = i0 - r->half + 1;
        for (k = 0; k < 2 * r->half; ++k)
        {
            long long idx = base + k;
            if (idx >= 0 && (size_t) idx < in_len)
            {
                acc += (double) h[k] * in[idx];
            }
        }
        out[n] = (float) acc;
    }
    *out_len = n_out;
    return out;
}

// ---------------------------------------------------------------------------

// One-pole DC blocker (models the AC-coupling capacitor on a real sound
// card's output). OPL half-/abs-sine waveforms carry a large DC component
// that would otherwise eat ~10% of the headroom and cause pops at the
// start/end of playback.
static void DcBlock(float *x, size_t n, double fc, unsigned long rate)
{
    double a = exp(-2.0 * M_PI * fc / rate);
    double xp = 0.0, yp = 0.0;
    size_t i;
    for (i = 0; i < n; ++i)
    {
        double y = a * (yp + x[i] - xp);
        xp = x[i];
        yp = y;
        x[i] = (float) y;
    }
}

static int IsMusLump(const unsigned char *d, int len)
{
    return len >= 16 && memcmp(d, "MUS\x1a", 4) == 0;
}

static int RenderLump(const char *name, int volume, resampler_t *rs,
                      unsigned long out_rate, double dc_cutoff, track_t *t)
{
    unsigned char *data;
    int len;
    void *handle;
    uint64_t start, eos_native, last_off_native;
    size_t frames, i;
    const int16_t *cap;
    float *mono;
    int16_t pk = 0;

    data = WAD_ReadLump(name, &len);
    if (data == NULL || !IsMusLump(data, len))
    {
        fprintf(stderr, "%s: missing or not a MUS lump\n", name);
        free(data);
        return 0;
    }

    memset(t, 0, sizeof(*t));
    snprintf(t->name, sizeof(t->name), "%s", name);
    for (i = 0; t->name[i]; ++i) t->name[i] = toupper((unsigned char) t->name[i]);

    // Fresh chip + music module for every song.
    OfflineOPL_SetCapture(0);
    OfflineOPL_ClearCapture();
    if (!music_opl_module.Init())
    {
        fprintf(stderr, "OPL music init failed\n");
        exit(1);
    }
    music_opl_module.SetMusicVolume(volume);

    handle = music_opl_module.RegisterSong(data, len);
    if (handle == NULL)
    {
        fprintf(stderr, "%s: RegisterSong failed\n", name);
        exit(1);
    }

    OfflineOPL_ResetKeyOffTracking();
    OfflineOPL_ClearCapture();
    OfflineOPL_SetCapture(1);
    start = OfflineOPL_TotalSamples();

    music_opl_module.PlaySong(handle, false);   // one pass, no looping
    OfflineOPL_RunUntilQueueEmpty((uint64_t) MAX_SONG_SECS * NATIVE_RATE);
    eos_native = OfflineOPL_TotalSamples() - start;
    t->held_at_eos = OfflineOPL_VoicesKeyedOn();

    // Release anything still held, as Chocolate Doom does on stop.
    music_opl_module.StopSong();
    OfflineOPL_RunSamples((uint64_t) (TAIL_SECS * NATIVE_RATE));

    if (OfflineOPL_LastKeyOff(&last_off_native))
    {
        last_off_native -= start;
    }
    else
    {
        last_off_native = eos_native;
    }

    cap = OfflineOPL_Captured(&frames);
    if (getenv("MUSRENDER_DUMP_RAW") != NULL)
    {
        // Debug: raw native-rate stereo s16le emulator output.
        char dp[4096];
        FILE *df;
        snprintf(dp, sizeof(dp), "%s/%s.raw", getenv("MUSRENDER_DUMP_RAW"), t->name);
        df = fopen(dp, "wb");
        if (df) { fwrite(cap, 4, frames, df); fclose(df); }
    }
    mono = malloc(sizeof(float) * (frames ? frames : 1));
    for (i = 0; i < frames; ++i)
    {
        int16_t l = cap[2 * i], r = cap[2 * i + 1];
        int16_t a = l < 0 ? -l : l, b = r < 0 ? -r : r;
        if (l != r) t->lr_mismatch++;
        if (a > pk) pk = a;
        if (b > pk) pk = b;
        mono[i] = 0.5f * ((float) l + (float) r);
    }
    t->raw_peak = pk;
    t->native_secs = (double) eos_native / NATIVE_RATE;

    music_opl_module.UnRegisterSong(handle);
    music_opl_module.Shutdown();
    OfflineOPL_SetCapture(0);
    free(data);

    t->pcm = Resample(rs, mono, frames, &t->len);
    free(mono);
    if (dc_cutoff > 0.0)
    {
        DcBlock(t->pcm, t->len, dc_cutoff, out_rate);
    }
    t->eos = (size_t) ((eos_native * out_rate + NATIVE_RATE / 2) / NATIVE_RATE);
    t->last_off = (size_t) ((last_off_native * out_rate + NATIVE_RATE / 2)
                            / NATIVE_RATE);
    return 1;
}

static void Put16(unsigned char *p, unsigned v) { p[0] = v; p[1] = v >> 8; }
static void Put32(unsigned char *p, unsigned long v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static long WriteWav(const char *path, const int16_t *s, size_t n,
                     unsigned long rate)
{
    unsigned char h[44];
    FILE *f;
    size_t i;
    unsigned long data_bytes = (unsigned long) n * 2;

    memcpy(h, "RIFF", 4);
    Put32(h + 4, 36 + data_bytes);
    memcpy(h + 8, "WAVE", 4);
    memcpy(h + 12, "fmt ", 4);
    Put32(h + 16, 16);
    Put16(h + 20, 1);          // PCM
    Put16(h + 22, 1);          // mono
    Put32(h + 24, rate);
    Put32(h + 28, rate * 2);   // byte rate
    Put16(h + 32, 2);          // block align
    Put16(h + 34, 16);         // bits
    memcpy(h + 36, "data", 4);
    Put32(h + 40, data_bytes);

    f = fopen(path, "wb");
    if (f == NULL) { perror(path); exit(1); }
    fwrite(h, 1, 44, f);
    for (i = 0; i < n; ++i)
    {
        unsigned char b[2];
        Put16(b, (uint16_t) s[i]);
        fwrite(b, 1, 2, f);
    }
    if (fclose(f) != 0) { perror(path); exit(1); }
    return 44 + (long) data_bytes;
}

static void Usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [options] WAD [LUMP ...]\n"
        "  Renders MUS lumps (default: every MUS lump in the WAD) to\n"
        "  <OUTDIR>/<LUMP>.WAV, 16-bit mono PCM, plus <OUTDIR>/INDEX.TXT.\n"
        "options:\n"
        "  -o DIR      output directory (default .)\n"
        "  -r RATE     output sample rate (default 24000)\n"
        "  -g GAIN     fixed linear gain applied to the emulator output\n"
        "              (default: auto = target / loudest peak of this run)\n"
        "  -t PEAK     auto-gain target peak (default 23000, about -3 dBFS)\n"
        "  -v VOL      Doom music volume 0-127 (default 127)\n"
        "  -c HZ       resampler cutoff (default 0.47 * out rate)\n"
        "  -H HZ       DC-blocking high-pass corner (default 5, 0 = off)\n",
        argv0);
    exit(2);
}

int main(int argc, char **argv)
{
    const char *outdir = ".";
    const char *wad = NULL;
    unsigned long out_rate = 24000;
    double gain = 0.0, target = 23000.0, cutoff = 0.0, dc_hz = 5.0;
    int volume = 127;
    char **names = NULL;
    int num_names = 0, i, ntracks = 0;
    track_t *tracks;
    resampler_t rs;
    double max_peak = 0.0;
    char path[4096];
    FILE *idx;

    for (i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-o") && i + 1 < argc) outdir = argv[++i];
        else if (!strcmp(argv[i], "-r") && i + 1 < argc) out_rate = strtoul(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "-g") && i + 1 < argc) gain = atof(argv[++i]);
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) target = atof(argv[++i]);
        else if (!strcmp(argv[i], "-v") && i + 1 < argc) volume = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-c") && i + 1 < argc) cutoff = atof(argv[++i]);
        else if (!strcmp(argv[i], "-H") && i + 1 < argc) dc_hz = atof(argv[++i]);
        else if (argv[i][0] == '-') Usage(argv[0]);
        else if (wad == NULL) wad = argv[i];
        else
        {
            names = realloc(names, sizeof(char *) * (num_names + 1));
            names[num_names++] = argv[i];
        }
    }
    if (wad == NULL || out_rate < 8000 || volume < 0 || volume > 127) Usage(argv[0]);
    if (!WAD_Open(wad)) return 1;
    mkdir(outdir, 0755);

    if (num_names == 0)
    {
        for (i = 0; i < WAD_NumLumps(); ++i)
        {
            int len;
            unsigned char *d;
            if (strncasecmp(WAD_LumpName(i), "D_", 2) != 0) continue;
            d = WAD_ReadLumpNum(i, &len);
            if (d && IsMusLump(d, len))
            {
                names = realloc(names, sizeof(char *) * (num_names + 1));
                names[num_names++] = strdup(WAD_LumpName(i));
            }
            free(d);
        }
    }

    if (cutoff <= 0.0) cutoff = 0.47 * out_rate;
    // ~90 dB stopband (beta 9), 128 taps each side at 49716 Hz.
    Resampler_Init(&rs, NATIVE_RATE, out_rate, cutoff, 128, 9.0);

    // Doom 1.9 IWAD -> DMX 1.9 OPL behaviour (Chocolate Doom's default for
    // this game version); OPL2 mode (snd_dmxoption has no "-opl3").
    I_SetOPLDriverVer(opl_doom_1_9);

    tracks = calloc(num_names, sizeof(track_t));
    for (i = 0; i < num_names; ++i)
    {
        size_t k;
        double pk = 0.0;
        if (!RenderLump(names[i], volume, &rs, out_rate, dc_hz,
                        &tracks[ntracks]))
            continue;
        for (k = 0; k < tracks[ntracks].len; ++k)
        {
            double a = fabs(tracks[ntracks].pcm[k]);
            if (a > pk) pk = a;
        }
        if (pk > max_peak) max_peak = pk;
        fprintf(stderr, "%-8s score %.3f s, held@end %d, raw peak %d, "
                "resampled peak %.1f, L!=R frames %zu\n",
                tracks[ntracks].name, tracks[ntracks].native_secs,
                tracks[ntracks].held_at_eos, tracks[ntracks].raw_peak, pk,
                tracks[ntracks].lr_mismatch);
        ++ntracks;
    }
    if (ntracks == 0) { fprintf(stderr, "nothing rendered\n"); return 1; }

    if (gain <= 0.0)
    {
        gain = target / max_peak;
    }
    fprintf(stderr, "gain %.6f (loudest emulator peak %.1f)\n", gain, max_peak);

    snprintf(path, sizeof(path), "%s/INDEX.TXT", outdir);
    idx = fopen(path, "w");
    if (idx == NULL) { perror(path); return 1; }
    fprintf(idx, "# Doom MUS -> Chocolate Doom OPL (DMX 1.9, OPL2, GENMIDI) "
                 "-> %lu Hz mono s16le WAV, one pass\n", out_rate);
    fprintf(idx, "# fixed gain %.6f for all tracks, Doom music volume %d/127, "
                 "DC block %.1f Hz\n", gain, volume, dc_hz);
    fprintf(idx, "# LUMP      DURATION_S  SAMPLES  PEAK   RMS       FILE_BYTES  "
                 "SCORE_END_SAMPLE\n");

    for (i = 0; i < ntracks; ++i)
    {
        track_t *t = &tracks[i];
        int16_t *q = malloc(sizeof(int16_t) * (t->len ? t->len : 1));
        size_t k, keep, clipped = 0;
        double ss = 0.0;
        int pk = 0;

        for (k = 0; k < t->len; ++k)
        {
            double v = t->pcm[k] * gain;
            long s = lrint(v);
            if (s > 32767) { s = 32767; ++clipped; }
            if (s < -32768) { s = -32768; ++clipped; }
            q[k] = (int16_t) s;
        }

        // Trailing silence trim. The emulator never reaches exact digital
        // zero (operators at full attenuation leave a few LSB of DC/residue,
        // and GENMIDI voices with release rate 0 sustain forever after
        // key-off), so: keep at least until last key-off + 0.5 s (and never
        // before the score end); beyond that keep the release tail while a
        // 10 ms window is above ~-72 dBFS RMS, capped at key-off + 1.5 s.
        {
            size_t min_keep, max_keep, win = (size_t) (SILENCE_WIN * out_rate);
            size_t end;
            min_keep = t->last_off + (size_t) (KEEP_AFTER_OFF * out_rate);
            if (min_keep < t->eos) min_keep = t->eos;
            if (min_keep > t->len) min_keep = t->len;
            max_keep = t->last_off + (size_t) (MAX_AFTER_OFF * out_rate);
            if (max_keep < min_keep) max_keep = min_keep;
            if (max_keep > t->len) max_keep = t->len;

            keep = min_keep;
            for (end = max_keep; end >= min_keep + win; end -= win)
            {
                double e = 0.0;
                size_t j;
                for (j = end - win; j < end; ++j) e += (double) q[j] * q[j];
                if (sqrt(e / win) > SILENCE_RMS)
                {
                    keep = end;
                    break;
                }
            }
            t->tail_capped = (keep == max_keep && max_keep > min_keep);
            if (t->tail_capped)
            {
                // Sound still ringing at the cap: 10 ms raised-cosine
                // de-click on the release tail only (never on the score).
                for (k = 0; k < win; ++k)
                {
                    double g = 0.5 * (1.0 + cos(M_PI * (k + 1) / win));
                    size_t j = keep - win + k;
                    q[j] = (int16_t) lrint(q[j] * g);
                }
            }
        }
        t->out_len = keep;

        for (k = 0; k < keep; ++k)
        {
            int a = q[k] < 0 ? -q[k] : q[k];
            if (a > pk) pk = a;
            ss += (double) q[k] * q[k];
        }
        t->peak = pk;
        t->rms = keep ? sqrt(ss / keep) : 0.0;

        snprintf(path, sizeof(path), "%s/%s.WAV", outdir, t->name);
        t->file_size = WriteWav(path, q, keep, out_rate);
        fprintf(idx, "%-10s %10.3f %8zu %6d %9.1f %10ld %8zu\n",
                t->name, (double) keep / out_rate, keep, t->peak, t->rms,
                t->file_size, t->eos);
        fprintf(stderr, "%-8s -> %s: %.3f s (%zu samples), score end %zu, "
                "last key-off %zu, peak %d, rms %.1f, clipped %zu%s\n",
                t->name, path, (double) keep / out_rate, keep, t->eos,
                t->last_off, t->peak, t->rms, clipped,
                t->tail_capped ? " [tail capped, 10ms de-click]" : "");
        free(q);
    }
    fclose(idx);
    return 0;
}
