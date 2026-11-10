/* Doom sound effects for the EOS R (portable part; built for the camera and the host harness).
 *
 * Canon's sound-effect player (AudioBeep PCM) plays ONE mono buffer at a time: stereo froze the camera twice, and
 * back-to-back buffers leave ~12 ms gaps (camera 2026-11-10), so there is no streamed mix. Each effect is converted
 * whole (8-bit 11/22 kHz -> 16-bit 24 kHz mono, scaled by Doom's volume) into one of three buffers and handed to the
 * platform, which replaces whatever is playing. A new effect is dropped only while the current one is fresh
 * (< DM_SND_HOLD_MS) and more important (Doom's sfx priority: lower = more important; same priority: louder).
 *
 * Platform side (dm_sys.h): dm_snd_hw_alloc / dm_snd_hw_play / dm_snd_hw_stop / dm_snd_hw_busy. */

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

#define DM_SND_NBUF     3
#define DM_SND_MAX      (48000)         /* samples per buffer: 2 s at 24 kHz (largest DOOM1 sfx, DSBAREXP, 1.69 s) */
#define DM_SND_HOLD_MS  150
/* gain: s16 per 8-bit unit = vol * DM_SND_GAIN; at the default volume (sfx_volume 8 -> 64) that is 768, the
 * level riz found "loud enough" in the Sound test (pistol x3, clipped) */
#define DM_SND_GAIN     12
/* Doom's effects are normalised to full scale, so x3 would hard-clip up to 40% of a sound (DSTINK); a soft knee
 * keeps the loudness without the harsh edges */
#define DM_SND_KNEE     20000

static int16_t *sbuf[DM_SND_NBUF];
static int      ready, use_prefix, last_buf = -1;
static int      cur_ch = -1, cur_vol, cur_prio;
static uint32_t cur_t0, cur_end;
static int      n_start, n_drop, n_stop, n_bad, n_clip_samples;

static boolean Snd_Init(boolean prefix)
{
    use_prefix = prefix;
    for (int i = 0; i < DM_SND_NBUF; i++)
        if (!(sbuf[i] = dm_snd_hw_alloc(DM_SND_MAX * 2)))
        {
            printf("DM sound: no output (buffer %d)\n", i);
            return false;
        }
    ready = 1;
    printf("DM sound: Canon AudioBeep PCM, 24 kHz mono, %d x %d KB, newest wins\n", DM_SND_NBUF, DM_SND_MAX * 2 / 1024);
    return true;
}

static void Snd_Shutdown(void)
{
    if (ready) dm_snd_hw_stop();
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

static void Snd_UpdateSoundParams(int channel, int vol, int sep)
{
    if (channel == cur_ch) cur_vol = vol;
}

static int playing_now(void)
{
    return cur_ch >= 0 && (int32_t)(cur_end - DG_GetTicksMs()) > 0;
}

/* 8-bit unsigned at rate -> s16 24 kHz, linear interpolation; returns samples written */
static int convert(const uint8_t *s, int n, int rate, int vol, int16_t *out)
{
    if (n > 80000) n = 80000;                               /* keeps n * 24000 in 31 bits */
    int m = n * 24000 / rate;
    if (m > DM_SND_MAX) m = DM_SND_MAX;
    uint32_t step = ((uint32_t)rate << 16) / 24000, x = 0;
    int g = vol * DM_SND_GAIN;
    for (int i = 0; i < m; i++, x += step)
    {
        int i0 = x >> 16, fr = x & 0xffff;
        int a = s[i0] - 128, c = (i0 + 1 < n ? s[i0 + 1] : s[i0]) - 128;
        int v = ((a << 16) + (c - a) * fr) >> 8;            /* 8-bit unit = 256 */
        v = v * g >> 8;
        int av = v < 0 ? -v : v;
        if (av > DM_SND_KNEE)       /* soft limiter: above the knee, approach full scale instead of hard clipping */
        {
            int over = av - DM_SND_KNEE;
            av = DM_SND_KNEE + over * (32767 - DM_SND_KNEE) / (over + (32767 - DM_SND_KNEE));
            v = v < 0 ? -av : av;
            n_clip_samples++;
        }
        out[i] = (int16_t)v;
    }
    return m;
}

static int Snd_StartSound(sfxinfo_t *sfx, int channel, int vol, int sep)
{
    if (!ready || sfx->lumpnum < 0) return -1;
    uint32_t now = DG_GetTicksMs();
    if (playing_now() && now - cur_t0 < DM_SND_HOLD_MS
        && (sfx->priority > cur_prio || (sfx->priority == cur_prio && vol < cur_vol)))
    {
        n_drop++;
        return -1;
    }

    int len = W_LumpLength(sfx->lumpnum);
    const uint8_t *d = W_CacheLumpNum(sfx->lumpnum, PU_STATIC);
    int rate = d[2] | (d[3] << 8), n = d[4] | (d[5] << 8) | (d[6] << 16) | (d[7] << 24);
    if (len < 8 || d[0] != 3 || d[1] != 0 || n > len - 8 || n <= 48 || rate < 4000 || rate > 48000)
    {
        W_ReleaseLumpNum(sfx->lumpnum);
        n_bad++;
        return -1;
    }

    /* a buffer the hardware isn't reading and that isn't the one just handed over (it may not be started yet) */
    const void *busy = dm_snd_hw_busy();
    int b = 0;
    while (b == last_buf || sbuf[b] == busy) b++;

    /* the DMX library skips the first and last 16 bytes of the lump (as Chocolate Doom does) */
    int m = convert(d + 16, n - 32, rate, vol, sbuf[b]);
    W_ReleaseLumpNum(sfx->lumpnum);

    dm_snd_hw_play(sbuf[b], m);
    last_buf = b;
    cur_ch = channel; cur_vol = vol; cur_prio = sfx->priority;
    cur_t0 = now; cur_end = now + m / 24 + 20;
    n_start++;
    return channel;
}

static void Snd_StopSound(int channel)
{
    if (channel == cur_ch && playing_now())
    {
        dm_snd_hw_stop();
        n_stop++;
    }
    if (channel == cur_ch) cur_ch = -1;
}

static boolean Snd_SoundIsPlaying(int channel)
{
    return channel == cur_ch && playing_now();
}

static void Snd_CacheSounds(sfxinfo_t *sounds, int num_sounds) { }

int dm_snd_stats(char *buf, size_t n)
{
    return snprintf(buf, n, "sound: %d started, %d dropped (priority), %d stopped, %d bad lumps, %d samples soft-limited\n",
                    n_start, n_drop, n_stop, n_bad, n_clip_samples);
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
