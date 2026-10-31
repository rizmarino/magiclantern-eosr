/* riz/eosr-dmlog: Debug > Memory survey (EOS R 1.8.0). Read-only map of where memory goes, per camera mode.
 * Personal diagnostic build, not for upstream.
 *
 * Question (riz, 2026-10-31): LV silent pictures get only 13 4K frames (~150 MB of shoot memory in movie LV).
 * How much memory is there, what is it used for, and is more free in some mode?
 * One press = one survey of the current mode, appended to ML/MEMSURV.TXT:
 *  1. Mode: lv, movie/photo, ML's video-mode props, last MEM1 frame size.
 *  2. Canon heap (AllocateMemory pool): total / free.
 *  3. Shoot memory: largest contiguous block, then everything (fragmented suite) with every piece's address and size,
 *     and how many 4K 10-bit / 12-bit / 2560x1440 10-bit frames fit if a frame may not span pieces. Freed at once.
 *  4. 1 GB activity map: every 1 MB block of the 1st GB (the R's only general RAM: the MMU maps 0-1 GB, and the
 *     v8 probe found nothing behind the 2nd-GB window except VRAM), 64 words sampled through the uncached alias, twice
 *     0.5 s apart: '.' all zero, 'f' all 0xFF.., 's' unchanging data, 'C' changing (live buffers), 'S' a shoot-memory
 *     piece that was free in step 3. Reads only, all inside the RAM the MMU maps as normal memory.
 * Runs after the ML menu closes (Trash). Takes ~2-4 s. Nothing is written to camera memory or properties. */
#include "dryos.h"
#include "menu.h"
#include "propvalues.h"
#include "mem.h"
#include "exmem.h"
#include "fio-ml.h"
#include "bmp.h"

extern int GetMemoryInformation(int *total, int *free);
extern int rawrec_running(void);

#define MS_BLOCK   (1024 * 1024)
#define MS_NBLK    1024                 /* 1 GB */
#define MS_NSAMP   64
#define MS_UNC     0x40000000
#define MS_MAXPC   64
#define MS_LOG     0x8000

static volatile int ms_busy;
static char *ms_log;
static int ms_len;

static void ms_printf(const char *fmt, ...)
{
    if (!ms_log || ms_len > MS_LOG - 200) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(ms_log + ms_len, MS_LOG - ms_len, fmt, ap);
    va_end(ap);
    if (n > 0) ms_len += MIN(n, MS_LOG - ms_len - 1);
}

static uint32_t ms_hash_block(uint32_t blk, int *kind)
{
    uint32_t base = MS_UNC + blk * MS_BLOCK, h = 0x811C9DC5;
    int zero = 1, ff = 1;
    for (int i = 0; i < MS_NSAMP; i++)
    {
        uint32_t v = *(volatile uint32_t *)(base + i * (MS_BLOCK / MS_NSAMP) + (i & 7) * 4);
        if (v) zero = 0;
        if (v != 0xFFFFFFFF) ff = 0;
        h = (h ^ v) * 0x01000193;
    }
    *kind = zero ? 1 : ff ? 2 : 0;
    return h;
}

struct ms_piece { uint32_t a, n; };

static void ms_task(void *unused)
{
    for (int i = 0; i < 300 && gui_menu_shown(); i++) msleep(100);
    msleep(1500);
    ms_log = malloc(MS_LOG);
    uint32_t *h1 = malloc(MS_NBLK * 4);
    uint8_t *k1 = malloc(MS_NBLK), *map = malloc(MS_NBLK + 1);
    struct ms_piece *pc = malloc(sizeof(*pc) * MS_MAXPC);
    int npc = 0;
    if (!ms_log || !h1 || !k1 || !map || !pc) { NotifyBox(4000, "Memory survey: out of memory"); goto out; }
    ms_len = 0;
    if (rawrec_running()) { NotifyBox(4000, "Memory survey: recorder / silent session busy"); goto out; }

    struct tm now;
    memset(&now, 0, sizeof(now));
    LoadCalendarFromRTC(&now);
    ms_printf("=== memory survey %04d-%02d-%02d %02d:%02d:%02d: lv=%d lv_movie_select=%d (2 = movie) shooting_mode=%d "
              "video_mode res %d fps %d crop %d; last MEM1 frame %dx%d\n", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
              now.tm_hour, now.tm_min, now.tm_sec, lv ? 1 : 0, lv_movie_select, shooting_mode, video_mode_resolution,
              video_mode_fps, video_mode_crop, MEM(0x133CC + 0x10), MEM(0x133CC + 0x14));

    int tot = 0, fr = 0;
    GetMemoryInformation(&tot, &fr);
    ms_printf("Canon heap (AllocateMemory): total %d KB, free %d KB\n", tot / 1024, fr / 1024);

    /* shoot memory */
    uint32_t contig = 0, full = 0;
    {
        struct memSuite *s = shoot_malloc_suite_contig(0);
        if (s) { contig = s->size; shoot_free_suite(s); }
    }
    {
        struct memSuite *s = shoot_malloc_suite(0);
        if (s)
        {
            full = s->size;
            for (struct memChunk *c = GetFirstChunkFromSuite(s); c && npc < MS_MAXPC; c = GetNextMemoryChunk(s, c))
            {
                pc[npc].a = (uint32_t)GetMemoryAddressOfMemoryChunk(c) & ~MS_UNC;
                pc[npc].n = GetSizeOfMemoryChunk(c);
                npc++;
            }
            shoot_free_suite(s);
        }
    }
    ms_printf("shoot memory: largest contiguous %d KB, all pieces %d KB in %d pieces\n", contig / 1024, full / 1024, npc);
    static const struct { const char *name; uint32_t bytes; } fr_sz[] = {
        { "4K 4256x2214 10-bit", 11784192 }, { "4K 4256x2214 12-bit", 14139392 },
        { "2560x1440 10-bit", 4612096 }, { "FHD 2304x1290 10-bit", 3719168 },
    };
    uint32_t fits[4] = { 0 };
    for (int i = 0; i < npc; i++)
    {
        ms_printf("  piece %2d: %08x .. %08x  %6d KB\n", i, pc[i].a, pc[i].a + pc[i].n - 1, pc[i].n / 1024);
        for (int f = 0; f < 4; f++) fits[f] += pc[i].n / (fr_sz[f].bytes + 4096);
    }
    for (int f = 0; f < 4; f++)
        ms_printf("  frames that fit (whole frames per piece): %s: %d (if one could span pieces: %d)\n", fr_sz[f].name,
                  fits[f], full / fr_sz[f].bytes);

    /* 1 GB activity map */
    uint32_t t0 = get_ms_clock();
    for (int b = 0; b < MS_NBLK; b++) { int k; h1[b] = ms_hash_block(b, &k); k1[b] = k; }
    msleep(500);
    int cnt[5] = { 0 };
    for (int b = 0; b < MS_NBLK; b++)
    {
        int k;
        uint32_t h2 = ms_hash_block(b, &k);
        char c = (k1[b] == 1 && k == 1) ? '.' : (k1[b] == 2 && k == 2) ? 'f' : (h2 == h1[b]) ? 's' : 'C';
        uint32_t a = b * MS_BLOCK;
        for (int i = 0; i < npc; i++)
            if (a >= pc[i].a && a + MS_BLOCK <= pc[i].a + pc[i].n) { c = 'S'; break; }
        map[b] = c;
        cnt[c == '.' ? 0 : c == 'f' ? 1 : c == 's' ? 2 : c == 'C' ? 3 : 4]++;
    }
    map[MS_NBLK] = 0;
    ms_printf("1 GB map (1 char = 1 MB, 64 MB per line; . zero, f 0xFF, s static data, C changing, S free shoot memory), "
              "%d ms:\n", get_ms_clock() - t0);
    for (int r = 0; r < MS_NBLK / 64; r++)
    {
        char line[72];
        memcpy(line, map + r * 64, 64);
        line[64] = 0;
        ms_printf("  %08x %s\n", r * 64 * MS_BLOCK, line);
    }
    ms_printf("blocks: zero %d, 0xFF %d, static %d, changing %d, free shoot %d (MB)\n", cnt[0], cnt[1], cnt[2], cnt[3], cnt[4]);
    FILE *f = FIO_CreateFileOrAppend("ML/MEMSURV.TXT");
    if (f) { FIO_WriteFile(f, ms_log, ms_len); FIO_CloseFile(f); }
    NotifyBox(6000, "Memory survey: shoot %d MB (largest %d MB, %d pieces), %d 4K frames. ML/MEMSURV.TXT",
              full >> 20, contig >> 20, npc, fits[0]);
out:
    if (ms_log) free(ms_log);
    if (h1) free(h1);
    if (k1) free(k1);
    if (map) free(map);
    if (pc) free(pc);
    ms_log = 0;
    ms_busy = 0;
}

/* ---- v3.13: SRM probe. Canon's still-capture ("1st job") buffers, borrowed one at a time and handed back at once:
 * how many, how big, where. Static RE: lv-survey.md "SRM 1st job buffers on the R". */
static void (*const SRM_Alloc1st)(void (*cbr)(void *, void *, uint32_t), void *ctx) = (void *)0xE04E41BF;
static void (*const SRM_Free1st)(void *buf, void *cbr, void *ctx)                    = (void *)0xE04E7591;
#define MS_SRM_MAX 4
static struct semaphore *ms_srm_sem;
static void *volatile ms_srm_buf[MS_SRM_MAX + 1];
static volatile uint32_t ms_srm_sz[MS_SRM_MAX + 1], ms_srm_cb;
static void ms_srm_cbr(void *ctx, void *buf, uint32_t size)
{
    int i = (int)ctx;
    if (i >= 0 && i <= MS_SRM_MAX) { ms_srm_buf[i] = buf; ms_srm_sz[i] = size; }
    ms_srm_cb++;
    give_semaphore(ms_srm_sem);
}

static void ms_srm_task(void *unused)
{
    for (int i = 0; i < 300 && gui_menu_shown(); i++) msleep(100);
    msleep(1500);
    ms_log = malloc(MS_LOG);
    if (!ms_log) { ms_busy = 0; return; }
    ms_len = 0;
    if (rawrec_running()) { NotifyBox(4000, "SRM probe: recorder / silent session busy"); goto out; }
    if (!ms_srm_sem) ms_srm_sem = create_named_semaphore("ms_srm", SEM_CREATE_LOCKED);
    for (int i = 0; i <= MS_SRM_MAX; i++) { ms_srm_buf[i] = 0; ms_srm_sz[i] = 0; }
    ms_srm_cb = 0;
    ms_printf("=== SRM probe: lv=%d lv_movie_select=%d (2 = movie) video_mode res %d\n", lv ? 1 : 0, lv_movie_select,
              video_mode_resolution);
    int n = 0;
    uint32_t total = 0;
    for (int i = 0; i < MS_SRM_MAX; i++)
    {
        uint32_t t0 = get_ms_clock();
        SRM_Alloc1st(ms_srm_cbr, (void *)i);
        int r = take_semaphore(ms_srm_sem, 1000);
        if (r) { ms_printf("  request %d: no buffer within 1 s (memory full or refused)\n", i); break; }
        ms_printf("  buffer %d: %08x, %d KB (callback after %d ms)\n", i, (uint32_t)ms_srm_buf[i], ms_srm_sz[i] / 1024,
                  get_ms_clock() - t0);
        total += ms_srm_sz[i];
        n++;
    }
    msleep(300);
    for (int i = 0; i < n; i++) { SRM_Free1st(ms_srm_buf[i], 0, 0); msleep(50); }
    msleep(500);
    if (ms_srm_buf[n]) { ms_printf("  late buffer %d arrived after the timeout: %08x, freeing it\n", n, (uint32_t)ms_srm_buf[n]); SRM_Free1st(ms_srm_buf[n], 0, 0); }
    ms_printf("SRM probe: %d buffers, %d KB total, %d callbacks; all freed\n", n, total / 1024, ms_srm_cb);
    FILE *f = FIO_CreateFileOrAppend("ML/MEMSURV.TXT");
    if (f) { FIO_WriteFile(f, ms_log, ms_len); FIO_CloseFile(f); }
    NotifyBox(6000, "SRM probe: %d buffers x %d MB. ML/MEMSURV.TXT", n, n ? ms_srm_sz[0] >> 20 : 0);
out:
    free(ms_log);
    ms_log = 0;
    ms_busy = 0;
}

static void ms_srm_select(void *priv, int delta)
{
    if (ms_busy) return;
    ms_busy = 1;
    task_create("srm_probe", 0x1e, 0x2000, ms_srm_task, 0);
}

static void ms_select(void *priv, int delta)
{
    if (ms_busy) return;
    ms_busy = 1;
    task_create("mem_survey", 0x1e, 0x2000, ms_task, 0);
}

static struct menu_entry ms_menu[] = {
    {
        .name   = "Memory survey",
        .select = ms_select,
        .help   = "Read-only: shoot memory pieces, Canon heap, 1 GB usage map. SET, close menu, wait for the box.",
        .help2  = "Run once per mode (photo LV, movie FHD, movie 4K). Appends to ML/MEMSURV.TXT.",
    },
    {
        .name   = "SRM probe",
        .select = ms_srm_select,
        .help   = "Borrows Canon's still-capture buffers (up to 4) and hands them back at once; logs size/address.",
        .help2  = "SET, close menu, wait for the box (~2 s). Don't press the shutter meanwhile. ML/MEMSURV.TXT.",
    },
};

static void ms_init(void *unused)
{
    menu_add("Debug", ms_menu, COUNT(ms_menu));
}

INIT_FUNC("memsurvey", ms_init);
