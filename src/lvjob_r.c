/* riz/eosr-dmlog: read-only snapshot of Canon's live-view frame-job structures.
 *
 * Debug -> LV job dump. Takes 8 samples, 250 ms apart, of RAM structures found
 * via Ghidra + the v2 LV logs (EOS R 1.8.0), and appends them to ML/LVJOB.BIN:
 *   JOBR  Epp job ring passed to PostRaw2YuvStage (4 x 0x14C at 0x3AE10)
 *   R2YP  raw2yuv stage globals (0x83E0..)        R2YS  *(0x83E0) stage object
 *   R2YA  raw2yuv MEMNAVI address array (0x3BFB4..) MEMF  Memif window table (0x742A8)
 *   LVBC  LvBufferController pointer array (12)   LVBn/LVSn  each controller + its slots
 * Only reads memory (pointers are range-checked) and writes the file.
 * Record: "LVJB" u32 run, sample, ms, sig; then sections: tag[4] u32 addr u32 len, data.
 * Personal diagnostic build, not for upstream.
 */
#include "dryos.h"
#include "menu.h"
#include "beep.h"
#include "propvalues.h"
#include "consts.h"
#include "timer.h"
#include "mem.h"

#define LVJ_SAMPLES   8
#define LVJ_PERIOD_MS 250
#define LVJ_BUF       0x4000

static volatile int lvj_busy = 0;
static int lvj_runs = 0;
static uint8_t *lvj_buf;
static int lvj_len;

/* RAM is 1 GB cached at 0x0, uncached alias at 0x40000000 */
static int lvj_ram_ok(uint32_t p, uint32_t len)
{
    uint32_t c = p & ~0x40000000;
    return p < 0x80000000 && c >= 0x1000 && c + len < 0x40000000;
}

static void lvj_put(const void *src, int len)
{
    if (lvj_len + len > LVJ_BUF) return;
    memcpy(lvj_buf + lvj_len, src, len);
    lvj_len += len;
}

static void lvj_section(const char *tag, uint32_t addr, uint32_t len)
{
    if (!lvj_ram_ok(addr, len)) len = 0;
    uint32_t hdr[3];
    memcpy(&hdr[0], tag, 4); hdr[1] = addr; hdr[2] = len;
    lvj_put(hdr, sizeof(hdr));
    if (len) lvj_put((void *)addr, len);
}

static void lvj_task(void *unused)
{
    lvj_buf = malloc(LVJ_BUF);
    FILE *f = lvj_buf ? FIO_CreateFileOrAppend("ML/LVJOB.BIN") : 0;
    if (!f) { NotifyBox(3000, "LV job dump: no file/memory"); goto out; }

    for (int s = 0; s < LVJ_SAMPLES; s++)
    {
        lvj_len = 0;
        uint32_t rec[5] = { 0x424A564C /* "LVJB" */, lvj_runs, s, get_ms_clock(),
                            (lv ? 1 : 0) | (is_movie_mode() ? 2 : 0) | (RECORDING ? 4 : 0) | (R_LV_ON_EVF() ? 8 : 0) };
        lvj_put(rec, sizeof(rec));

        lvj_section("JOBR", 0x3AE10, 4 * 0x14C);
        lvj_section("R2YP", 0x83E0, 0x60);
        lvj_section("R2YS", MEM(0x83E0), 0x80);
        lvj_section("R2YA", 0x3BFB4, 0x40);
        lvj_section("MEMF", 0x742A8, 0x180);
        lvj_section("LVBC", 0x7F9A4, 12 * 4);
        for (int i = 0; i < 12; i++)
        {
            uint32_t c = MEM(0x7F9A4 + 4 * i);
            char tb[4] = { 'L', 'V', 'B', 'A' + i }, ts[4] = { 'L', 'V', 'S', 'A' + i };
            lvj_section(tb, c, 0x24);
            if (lvj_ram_ok(c, 0x24))
            {
                uint32_t n = MEM(c + 0x10);
                lvj_section(ts, MEM(c + 8), (n <= 16 ? n : 16) * 8);
            }
        }
        FIO_WriteFile(f, lvj_buf, lvj_len);
        msleep(LVJ_PERIOD_MS);
    }
    FIO_CloseFile(f);
    lvj_runs++;
    beep();
out:
    if (lvj_buf) free(lvj_buf);
    lvj_buf = 0;
    lvj_busy = 0;
}

static MENU_SELECT_FUNC(lvj_select)
{
    if (lvj_busy) return;
    lvj_busy = 1;
    task_create("lvj_task", 0x1e, 0x1000, lvj_task, 0);
}

/* LV buffer dump: copies the raw2yuv input buffer currently referenced by the
 * stage object (+0x20; must be one of the 4 MEMNAVI buffers seen in v3) to
 * ML/LVBUFnn.BIN. Header: "LVBF" u32 run, ms, addr, len, then R2YS (0x80),
 * MEMF (0x180), R2YA (0x40) snapshots, then the buffer. Read-only apart from
 * the file. The buffer keeps being rewritten while we copy (3-4 frame ring),
 * so expect tearing between chunks; aim at a static scene. */
#define LVB_SIZE  0x1540000
#define LVB_CHUNK 0x40000
static const uint32_t lvb_known[] = { 0x20000000, 0x21540000, 0x22A80000, 0x23FC0000 };

static void lvb_task(void *unused)
{
    char name[32];
    uint32_t stage = MEM(0x83E0), cur = 0;
    if (lvj_ram_ok(stage, 0x80)) cur = MEM(stage + 0x20) & ~0x40000000;
    int ok = 0;
    for (int i = 0; i < COUNT(lvb_known); i++) ok |= (cur == lvb_known[i]);
    if (!ok) { NotifyBox(4000, "LV buffer dump: unexpected buffer %x", cur); goto out; }

    uint32_t chunk = LVB_CHUNK;
    lvj_buf = malloc(chunk);
    if (!lvj_buf) { chunk = 0x10000; lvj_buf = malloc(chunk); }
    snprintf(name, sizeof(name), "ML/LVBUF%02d.BIN", lvj_runs);
    FILE *f = lvj_buf ? FIO_CreateFile(name) : 0;
    if (!f) { NotifyBox(4000, "LV buffer dump: no file/memory"); goto out; }

    lvj_len = 0;
    uint32_t hdr[5] = { 0x4642564C /* "LVBF" */, lvj_runs, get_ms_clock(), cur, LVB_SIZE };
    lvj_put(hdr, sizeof(hdr));
    lvj_put((void *)stage, 0x80);
    lvj_put((void *)0x742A8, 0x180);
    lvj_put((void *)0x3BFB4, 0x40);
    FIO_WriteFile(f, lvj_buf, lvj_len);

    uint32_t src = cur | 0x40000000;            /* uncached alias, as Canon uses */
    for (uint32_t off = 0; off < LVB_SIZE; off += chunk)
    {
        memcpy(lvj_buf, (void *)(src + off), chunk);
        FIO_WriteFile(f, lvj_buf, chunk);
    }
    FIO_CloseFile(f);
    lvj_runs++;
    beep();
out:
    if (lvj_buf) free(lvj_buf);
    lvj_buf = 0;
    lvj_busy = 0;
}

static MENU_SELECT_FUNC(lvb_select)
{
    if (lvj_busy) return;
    lvj_busy = 1;
    task_create("lvb_task", 0x1e, 0x1000, lvb_task, 0);
}

/* LV memory survey (v5). Samples 4 x 64 bytes of every 128 KB block of the low
 * 1 GB, three times (0, 150, 600 ms), and records per block: hash, change bits,
 * zero bytes, odd bytes < 0x40 (hint for unpacked 14-bit LE), sum |byte diff|.
 * Blocks below 0x30000000 are read through the uncached alias (+0x40000000, as
 * Canon does for LV buffers); 0x30000000-0x3FFFFFFF cached (ML treats
 * 0x7xxxxxxx as unsafe). The map goes to ML/LVMAPnn.BIN. Then dumps, to
 * ML/LVRGnn_k.BIN, the fixed raw2yuv side windows 0x25500000 (+4 MB) and up to
 * 3 of the largest changing regions (>= 4 MB, first 8 MB each) outside the 4
 * known YUV buffers 0x20000000-0x254FFFFF. Read-only apart from the files. */
#define LVS_BLK     0x20000
#define LVS_NBLK    (0x40000000 / LVS_BLK)
#define LVS_REC     16
#define LVS_DUMPMAX 0x800000

static uint32_t lvs_addr(uint32_t a)
{
    return a < 0x30000000 ? (a | 0x40000000) : a;
}

static void lvs_pass(uint8_t *map, int pass)
{
    static const uint32_t offs[4] = { 0x0, 0x8000, 0x10000, 0x18000 };
    for (int b = 0; b < LVS_NBLK; b++)
    {
        uint32_t h = 0x811C9DC5, zeros = 0, lt40 = 0, sad = 0;
        for (int k = 0; k < 4; k++)
        {
            uint32_t a = b * LVS_BLK + offs[k];
            if (a < 0x1000) a = 0x1000;
            const volatile uint8_t *p = (const volatile uint8_t *)lvs_addr(a);
            uint8_t prev = p[0];
            for (int i = 0; i < 64; i++)
            {
                uint8_t v = p[i];
                h = (h ^ v) * 0x01000193;
                if (pass == 0)
                {
                    zeros += (v == 0);
                    if (i & 1) lt40 += (v < 0x40);
                    sad += (v > prev) ? v - prev : prev - v;
                    prev = v;
                }
            }
        }
        uint32_t *r = (uint32_t *)(map + b * LVS_REC);
        if (pass == 0)
        {
            r[0] = h; r[1] = 0; r[2] = zeros | (lt40 << 16); r[3] = sad;
        }
        else
        {
            /* r[1]: bit0 = changed 0->150 ms, bit1 = changed 150->600 ms,
             * bits 8-31 = low 24 bits of the previous pass's hash */
            uint32_t prev24 = pass == 1 ? (r[0] & 0xFFFFFF) : (r[1] >> 8);
            if ((h & 0xFFFFFF) != prev24) r[1] |= (1 << (pass - 1));
            r[1] = (r[1] & 0xFF) | ((h & 0xFFFFFF) << 8);
        }
    }
}

static void lvs_dump(const char *name, uint32_t addr, uint32_t len, uint8_t *buf, uint32_t chunk)
{
    FILE *f = FIO_CreateFile(name);
    if (!f) return;
    uint32_t hdr[4] = { 0x4752564C /* "LVRG" */, get_ms_clock(), addr, len };
    FIO_WriteFile(f, hdr, sizeof(hdr));
    for (uint32_t off = 0; off < len; off += chunk)
    {
        memcpy(buf, (void *)lvs_addr(addr + off), chunk);
        FIO_WriteFile(f, buf, chunk);
    }
    FIO_CloseFile(f);
}

static void lvs_task(void *unused)
{
    char name[32];
    uint8_t *map = malloc(LVS_NBLK * LVS_REC);
    uint32_t chunk = LVB_CHUNK;
    uint8_t *buf = malloc(chunk);
    if (!buf) { chunk = 0x10000; buf = malloc(chunk); }
    if (!map || !buf) { NotifyBox(4000, "LV survey: no memory"); goto out; }

    uint32_t t0 = get_ms_clock();
    lvs_pass(map, 0);
    msleep(150);
    lvs_pass(map, 1);
    msleep(450);
    lvs_pass(map, 2);
    uint32_t t1 = get_ms_clock();

    snprintf(name, sizeof(name), "ML/LVMAP%02d.BIN", lvj_runs);
    FILE *f = FIO_CreateFile(name);
    if (!f) { NotifyBox(4000, "LV survey: no file"); goto out; }
    uint32_t hdr[5] = { 0x50414D4C /* "LMAP" */, lvj_runs, t0, t1 - t0, LVS_BLK };
    FIO_WriteFile(f, hdr, sizeof(hdr));
    FIO_WriteFile(f, map, LVS_NBLK * LVS_REC);
    FIO_CloseFile(f);

    /* largest runs of blocks that changed in both intervals (1-block gaps allowed) */
    uint32_t best_s[3] = {0}, best_n[3] = {0};
    int s = -1, gap = 0;
    for (int b = 0; b <= LVS_NBLK; b++)
    {
        uint32_t a = b * LVS_BLK;
        int known = a >= 0x20000000 && a < 0x25500000;
        int ch = b < LVS_NBLK && !known && (((uint32_t *)(map + b * LVS_REC))[1] & 3) == 3;
        if (ch) { if (s < 0) s = b; gap = 0; continue; }
        if (s >= 0 && b < LVS_NBLK && !known && gap == 0) { gap = 1; continue; }
        if (s >= 0)
        {
            uint32_t n = b - gap - s;
            for (int k = 0; k < 3; k++)
                if (n > best_n[k])
                {
                    for (int j = 2; j > k; j--) { best_n[j] = best_n[j - 1]; best_s[j] = best_s[j - 1]; }
                    best_n[k] = n; best_s[k] = s;
                    break;
                }
        }
        s = -1; gap = 0;
    }

    snprintf(name, sizeof(name), "ML/LVRG%02d_W.BIN", lvj_runs);
    lvs_dump(name, 0x25500000, 0x400000, buf, chunk);
    for (int k = 0; k < 3; k++)
    {
        if (best_n[k] * LVS_BLK < 0x400000) break;
        uint32_t len = best_n[k] * LVS_BLK;
        if (len > LVS_DUMPMAX) len = LVS_DUMPMAX;
        snprintf(name, sizeof(name), "ML/LVRG%02d_%d.BIN", lvj_runs, k);
        lvs_dump(name, best_s[k] * LVS_BLK, len, buf, chunk);
    }
    lvj_runs++;
    beep();
out:
    if (map) free(map);
    if (buf) free(buf);
    lvj_busy = 0;
}

static MENU_SELECT_FUNC(lvs_select)
{
    if (lvj_busy) return;
    lvj_busy = 1;
    task_create("lvs_task", 0x1e, 0x1000, lvs_task, 0);
}

/* ---- v6 ---------------------------------------------------------------
 * LV memory survey 2: reads the MMU registers and the effective L1 table
 * (+ up to 64 L2 tables) to ML/LVMMUnn.BIN, then surveys only the 1 MB
 * sections the table maps as Normal memory with a physical address in the
 * second GB (PA 0x80000000-0xBFFFFFFF), each PA once (prefers a
 * non-cacheable alias). Same 3-pass change map as v5 -> ML/LVM2nn.BIN, then
 * dumps up to 3 of the largest changing regions (first 8 MB) -> ML/LVR2nn_k.BIN.
 * LV DAF region dump: copies 0x25880000-0x264C0000 (12.25 MB, uncached) ->
 * ML/LVDAFnn.BIN. Everything is read-only apart from the files. */
struct lvm_regs { uint32_t ttbr0, ttbr1, ttbcr, sctlr, prrr, nmrr; };

static void lvm_read_regs(struct lvm_regs *m)
{
    asm volatile ("mrc p15, 0, %0, c2, c0, 0" : "=r" (m->ttbr0));
    asm volatile ("mrc p15, 0, %0, c2, c0, 1" : "=r" (m->ttbr1));
    asm volatile ("mrc p15, 0, %0, c2, c0, 2" : "=r" (m->ttbcr));
    asm volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r" (m->sctlr));
    asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r" (m->prrr));
    asm volatile ("mrc p15, 0, %0, c10, c2, 1" : "=r" (m->nmrr));
}

static uint32_t lvm_table(const struct lvm_regs *m, uint32_t va)
{
    uint32_t n = m->ttbcr & 7;
    if (n && (va >> (32 - n))) return m->ttbr1 & ~0x3FFF;
    return m->ttbr0 & ~((1u << (14 - n)) - 1);
}

/* Canon keeps its MMU tables in ROM (R 1.8.0: TTBR0 0xE0004800, main L1 at 0xE0000000) */
static int lvm_tab_ok(uint32_t t)
{
    return (t >= 0x1000 && t < 0x40000000) || (t >= 0xE0000000 && t < 0xE2000000);
}

static int lvm_desc(const struct lvm_regs *m, uint32_t va, uint32_t *d)
{
    uint32_t t = lvm_table(m, va);
    *d = 0;
    if (!lvm_tab_ok(t)) return 0;
    *d = MEM(t + ((va >> 20) << 2));
    return 1;
}

/* 0 fault, 1 L2 table, 2 strongly-ordered/device, 3 normal non-cacheable, 4 normal cacheable */
static int lvm_class(const struct lvm_regs *m, uint32_t d, uint32_t va, uint32_t *pa)
{
    uint32_t t = d & 3;
    if (t == 0) return 0;
    if (t == 1) return 1;
    *pa = (d & (1 << 18)) ? ((d & 0xFF000000) | (va & 0x00F00000)) : (d & 0xFFF00000);
    uint32_t tex = (d >> 12) & 7, c = (d >> 3) & 1, b = (d >> 2) & 1;
    if (m->sctlr & (1 << 28))       /* TEX remap */
    {
        uint32_t idx = ((tex & 1) << 2) | (c << 1) | b;
        if (((m->prrr >> (2 * idx)) & 3) != 2) return 2;
        return ((m->nmrr >> (2 * idx)) & 3) ? 4 : 3;
    }
    if (tex == 0 && c == 0) return 2;
    if (tex == 2 && c == 0 && b == 0) return 2;
    if (tex == 1 && c == 0 && b == 0) return 3;
    if (tex & 4) return (c || b) ? 4 : 3;
    return 4;
}

#define LV2_PA0   0x80000000   /* 2nd GB; 0x40000000-0x7FFFFFFF is the uncached view of the 1st */
#define LV2_NSEC  1024

static void lvs2_pass(uint8_t *map, const uint32_t *pick, int pass)
{
    static const uint32_t offs[4] = { 0x0, 0x8000, 0x10000, 0x18000 };
    for (int b = 0; b < LV2_NSEC * 8; b++)
    {
        uint32_t *r = (uint32_t *)(map + b * LVS_REC);
        uint32_t sec = pick[b >> 3];
        if (!sec)
        {
            if (pass == 0) { r[0] = 0; r[1] = 0x80; r[2] = 0; r[3] = 0; }
            continue;
        }
        uint32_t h = 0x811C9DC5, zeros = 0, lt40 = 0, sad = 0;
        for (int k = 0; k < 4; k++)
        {
            const volatile uint8_t *p = (const volatile uint8_t *)(sec + (b & 7) * LVS_BLK + offs[k]);
            uint8_t prev = p[0];
            for (int i = 0; i < 64; i++)
            {
                uint8_t v = p[i];
                h = (h ^ v) * 0x01000193;
                if (pass == 0)
                {
                    zeros += (v == 0);
                    if (i & 1) lt40 += (v < 0x40);
                    sad += (v > prev) ? v - prev : prev - v;
                    prev = v;
                }
            }
        }
        if (pass == 0)
        {
            r[0] = h; r[1] = 0; r[2] = zeros | (lt40 << 16); r[3] = sad;
        }
        else
        {
            uint32_t prev24 = pass == 1 ? (r[0] & 0xFFFFFF) : (r[1] >> 8);
            if ((h & 0xFFFFFF) != prev24) r[1] |= (1 << (pass - 1));
            r[1] = (r[1] & 0xFF) | ((h & 0xFFFFFF) << 8);
        }
    }
}

static void lvs2_dump(const char *name, uint32_t pa, uint32_t len, const uint32_t *pick, uint8_t *buf, uint32_t chunk)
{
    FILE *f = FIO_CreateFile(name);
    if (!f) return;
    uint32_t hdr[4] = { 0x4752564C /* "LVRG" */, get_ms_clock(), pa, len };
    FIO_WriteFile(f, hdr, sizeof(hdr));
    uint32_t piece = chunk < 0x20000 ? chunk : 0x20000;
    for (uint32_t off = 0; off < len; off += chunk)
    {
        for (uint32_t q = 0; q < chunk; q += piece)
        {
            uint32_t p = pa + off + q;
            uint32_t sec = pick[(p - LV2_PA0) >> 20];
            if (sec) memcpy(buf + q, (void *)(sec + (p & 0xFFFFF)), piece);
            else memset(buf + q, 0, piece);
        }
        FIO_WriteFile(f, buf, chunk);
    }
    FIO_CloseFile(f);
}

static void lvs2_task(void *unused)
{
    char name[32];
    struct lvm_regs m;
    lvm_read_regs(&m);
    uint32_t *pick = malloc(LV2_NSEC * 4);
    uint8_t *pcls = malloc(LV2_NSEC);
    uint8_t *map = malloc(LV2_NSEC * 8 * LVS_REC);
    uint32_t chunk = LVB_CHUNK;
    uint8_t *buf = malloc(chunk);
    if (!buf) { chunk = 0x10000; buf = malloc(chunk); }
    if (!pick || !pcls || !map || !buf) { NotifyBox(4000, "LV survey 2: no memory"); goto out; }
    uint32_t t0tab = lvm_table(&m, 0);
    if (!lvm_tab_ok(t0tab) || !lvm_tab_ok(lvm_table(&m, 0x80000000))) { NotifyBox(4000, "LV survey 2: TTBR %x/%x/%x unexpected", m.ttbr0, m.ttbr1, m.ttbcr); goto out; }

    /* 1. MMU registers + effective L1 table + L2 tables, saved before any wide reads */
    snprintf(name, sizeof(name), "ML/LVMMU%02d.BIN", lvj_runs);
    FILE *f = FIO_CreateFile(name);
    if (!f) { NotifyBox(4000, "LV survey 2: no file"); goto out; }
    uint32_t mh[8] = { 0x554D4D4C /* "LMMU" */, m.ttbr0, m.ttbr1, m.ttbcr, m.sctlr, m.prrr, m.nmrr, 0 };
    FIO_WriteFile(f, mh, sizeof(mh));
    uint32_t *b32 = (uint32_t *)buf;
    for (uint32_t s = 0; s < 4096; s++) lvm_desc(&m, s << 20, &b32[s]);
    FIO_WriteFile(f, buf, 4096 * 4);
    int nl2 = 0;
    for (uint32_t s = 0; s < 4096 && nl2 < 64; s++)
    {
        uint32_t d;
        lvm_desc(&m, s << 20, &d);
        if ((d & 3) != 1) continue;
        uint32_t l2 = d & ~0x3FF;
        if (!lvm_tab_ok(l2)) continue;
        b32[0] = s << 20; b32[1] = l2;
        memcpy(buf + 8, (void *)l2, 1024);
        FIO_WriteFile(f, buf, 8 + 1024);
        nl2++;
    }
    FIO_CloseFile(f);

    /* 2. pick one VA per second-GB physical section */
    memset(pick, 0, LV2_NSEC * 4);
    memset(pcls, 0, LV2_NSEC);
    int npick = 0, nbeyond = 0, nl2all = 0;
    for (uint32_t s = 0; s < 0xC00; s++)
    {
        uint32_t d, pa = 0;
        lvm_desc(&m, s << 20, &d);
        int cls = lvm_class(&m, d, s << 20, &pa);
        if (cls == 1) nl2all++;
        if (cls < 3) continue;
        if (pa >= 0xC0000000) { nbeyond++; continue; }
        if (pa < LV2_PA0) continue;
        uint32_t i = (pa - LV2_PA0) >> 20;
        if (!pick[i]) npick++;
        if (!pick[i] || (cls == 3 && pcls[i] == 4)) { pick[i] = s << 20; pcls[i] = cls; }
    }

    /* 3. change map */
    uint32_t t0 = get_ms_clock();
    if (npick)
    {
        lvs2_pass(map, pick, 0);
        msleep(150);
        lvs2_pass(map, pick, 1);
        msleep(450);
        lvs2_pass(map, pick, 2);
    }
    uint32_t t1 = get_ms_clock();
    snprintf(name, sizeof(name), "ML/LVM2%02d.BIN", lvj_runs);
    f = FIO_CreateFile(name);
    if (!f) { NotifyBox(4000, "LV survey 2: no file"); goto out; }
    uint32_t hdr[8] = { 0x32504D4C /* "LMP2" */, lvj_runs, t0, t1 - t0, LVS_BLK, LV2_PA0, npick, nl2all | (nbeyond << 16) };
    FIO_WriteFile(f, hdr, sizeof(hdr));
    if (npick) FIO_WriteFile(f, map, LV2_NSEC * 8 * LVS_REC);
    FIO_WriteFile(f, pick, LV2_NSEC * 4);
    FIO_WriteFile(f, pcls, LV2_NSEC);
    FIO_CloseFile(f);

    /* 4. dumps of the largest changing runs */
    if (npick)
    {
        uint32_t best_s[3] = {0}, best_n[3] = {0};
        int s = -1, gap = 0;
        for (int b = 0; b <= LV2_NSEC * 8; b++)
        {
            uint32_t fl = b < LV2_NSEC * 8 ? ((uint32_t *)(map + b * LVS_REC))[1] : 0x80;
            int skipped = fl & 0x80;
            int ch = !skipped && (fl & 3) == 3;
            if (ch) { if (s < 0) s = b; gap = 0; continue; }
            if (s >= 0 && !skipped && gap == 0) { gap = 1; continue; }
            if (s >= 0)
            {
                uint32_t n = b - gap - s;
                for (int k = 0; k < 3; k++)
                    if (n > best_n[k])
                    {
                        for (int j = 2; j > k; j--) { best_n[j] = best_n[j - 1]; best_s[j] = best_s[j - 1]; }
                        best_n[k] = n; best_s[k] = s;
                        break;
                    }
            }
            s = -1; gap = 0;
        }
        for (int k = 0; k < 3; k++)
        {
            if (best_n[k] * LVS_BLK < 0x400000) break;
            uint32_t len = best_n[k] * LVS_BLK;
            if (len > LVS_DUMPMAX) len = LVS_DUMPMAX;
            snprintf(name, sizeof(name), "ML/LVR2%02d_%d.BIN", lvj_runs, k);
            lvs2_dump(name, LV2_PA0 + best_s[k] * LVS_BLK, len, pick, buf, chunk);
        }
    }
    NotifyBox(4000, "survey 2: %d MB mapped, %d L2, %d >2GB", npick, nl2all, nbeyond);
    lvj_runs++;
    beep();
out:
    if (pick) free(pick);
    if (pcls) free(pcls);
    if (map) free(map);
    if (buf) free(buf);
    lvj_busy = 0;
}

static void lvd_task(void *unused)
{
    char name[32];
    uint32_t chunk = LVB_CHUNK;
    uint8_t *buf = malloc(chunk);
    if (!buf) { chunk = 0x10000; buf = malloc(chunk); }
    if (!buf) { NotifyBox(4000, "LV DAF dump: no memory"); goto out; }
    snprintf(name, sizeof(name), "ML/LVDAF%02d.BIN", lvj_runs);
    lvs_dump(name, 0x25880000, 0xC40000, buf, chunk);
    lvj_runs++;
    beep();
out:
    if (buf) free(buf);
    lvj_busy = 0;
}

static MENU_SELECT_FUNC(lvs2_select)
{
    /* v8: disabled, reading parts of 0x80000000-0xBFFFFFFF froze the camera */
    NotifyBox(3000, "Survey 2 disabled: run LV 2nd-GB probe");
}
static void __attribute__((unused)) *lvs2_keep = lvs2_task;

/* v8 LV 2nd-GB probe: finds how much of 0x80000000-0xBFFFFFFF is actually
 * backed (survey 2 froze the camera there). For each 16 MB group it appends
 * "G <addr>" to ML/LVPROBE.TXT and closes the file *before* reading one word
 * per MB, then appends "ok <addr> <words>". After a freeze, the last "G" line
 * without an "ok" is the group that hangs. Starts with the known-good panel
 * VRAM (0x9F230000) as a control. Read-only apart from the file. */
static void lvp_line(const char *fmt, uint32_t a, const uint32_t *w, int n)
{
    char line[200];
    int len = snprintf(line, sizeof(line), fmt, a);
    for (int i = 0; i < n && len < (int)sizeof(line) - 12; i++)
        len += snprintf(line + len, sizeof(line) - len, " %08x", w[i]);
    if (len < (int)sizeof(line) - 2) { line[len++] = '\n'; line[len] = 0; }
    FILE *f = FIO_CreateFileOrAppend("ML/LVPROBE.TXT");
    if (!f) return;
    FIO_WriteFile(f, line, len);
    FIO_CloseFile(f);
}

static void lvp_task(void *unused)
{
    uint32_t w[16];
    lvp_line("start run %d", lvj_runs, 0, 0);
    lvp_line("G %08x (control: panel VRAM)", 0x9F230000, 0, 0);
    w[0] = MEM(0x9F230000);
    lvp_line("ok %08x", 0x9F230000, w, 1);
    for (uint32_t g = 0x80000000; g < 0xC0000000; g += 0x1000000)
    {
        lvp_line("G %08x", g, 0, 0);
        for (int i = 0; i < 16; i++) w[i] = MEM(g + i * 0x100000);
        lvp_line("ok %08x", g, w, 16);
    }
    lvp_line("done %08x", 0xC0000000, 0, 0);
    lvj_runs++;
    beep();
    lvj_busy = 0;
}

static MENU_SELECT_FUNC(lvp_select)
{
    if (lvj_busy) return;
    lvj_busy = 1;
    task_create("lvp_task", 0x1e, 0x1000, lvp_task, 0);
}

static MENU_SELECT_FUNC(lvd_select)
{
    if (lvj_busy) return;
    lvj_busy = 1;
    task_create("lvd_task", 0x1e, 0x1000, lvd_task, 0);
}

static MENU_UPDATE_FUNC(lvj_update)
{
    MENU_SET_VALUE(lvj_busy ? "sampling..." : "%d runs saved", lvj_runs);
}

static struct menu_entry lvj_menu[] = {
    {
        .name   = "LV job dump",
        .select = lvj_select,
        .update = lvj_update,
        .help   = "Read-only: 8 snapshots of Canon's LV frame-job structs -> ML/LVJOB.BIN",
        .help2  = "SET, close the menu, keep live view running until the beep (~2 s).",
    },
    {
        .name   = "LV buffer dump",
        .select = lvb_select,
        .update = lvj_update,
        .help   = "Read-only: copies the current LV YUV frame buffer (21 MB) -> ML/LVBUFnn.BIN",
        .help2  = "Static scene, NOT recording. SET, close menu, wait for beep (~5-15 s).",
    },
    {
        .name   = "LV memory survey",
        .select = lvs_select,
        .update = lvj_update,
        .help   = "Read-only: maps changing RAM, dumps side windows + biggest changing areas",
        .help2  = "Static scene, NOT recording. SET, close menu, wait for beep (~10-20 s).",
    },
    {
        .name   = "LV memory survey 2",
        .select = lvs2_select,
        .update = lvj_update,
        .help   = "DISABLED in v8 (froze the camera). Run LV 2nd-GB probe instead.",
        .help2  = "Re-enabled once the probe shows which 2nd-GB range is safe.",
        .shidden = 1,
    },
    {
        .name   = "LV 2nd-GB probe",
        .select = lvp_select,
        .update = lvj_update,
        .help   = "Reads 1 word/MB of 0x80000000-0xBFFFFFFF, logging each 16 MB step first",
        .help2  = "WILL likely freeze: pull battery, then copy ML/LVPROBE.TXT. Not recording.",
    },
    {
        .name   = "LV DAF region dump",
        .select = lvd_select,
        .update = lvj_update,
        .help   = "Read-only: copies 12 MB at 0x25880000 (2-copy mono image) -> ML/LVDAFnn.BIN",
        .help2  = "Movie mode, NOT recording. Run once in focus, once with focus way off.",
    },
};

static void lvj_init(void *unused)
{
    menu_add("Debug", lvj_menu, COUNT(lvj_menu));
}

INIT_FUNC("lvjob", lvj_init);
