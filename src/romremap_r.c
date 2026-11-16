/* EOS R: one shared ROM -> RAM remap for ML's runtime ROM data patches (cpu0 only).
 *
 * Generalised from silenttest_r.c v1.8-v3.18 (camera-verified: silent H burst fps patch). Canon's L1 table is copied,
 * the 0xE0 supersection split, and each listed 4 KB ROM page backed by a RAM copy through a small-page L2 table for
 * its 1 MB section; then TTBR is switched once to the copy. All pages are set up together on first use: a second,
 * independent remap would install its own L1 copy and silently drop the first one's pages.
 * Pages start as exact copies of ROM, so nothing changes until a feature writes to its page. Never freed (the active
 * tables live in it); gone at power off, never written to flash. Memory: Canon's AllocateMemory pool (~1.8 MB free),
 * not ML's heap/bss (Canon needs > ~450 KB free malloc heap for full-buffer bursts). */

#include "dryos.h"
#include "mem.h"
#include "romremap_r.h"

#ifdef CONFIG_MMU
#include "mmu_utils.h"
extern void change_mmu_tables(uint8_t *ttbr0, uint8_t *ttbr1, uint32_t cpu_id);
extern void *_AllocateMemory(size_t size);
extern void *_malloc(size_t size);

/* page list: silent burst fps tables; dual ISO: m_AnalogIsoGainFix (ISO register 0x2D rows, 9 modes);
 * v4.61 taller 4K: SAP/MEM1 size records 0xE0F2E5F8 (same 1 MB section as the silent fps page) */
static const uint32_t rmp_pages[RR_NPAGES] = { 0xE0F16000, 0xE0B40000, 0xE0B41000, 0xE0B42000, 0xE0F2E000 };
static const uint32_t rmp_sections[] = { 0xE0F00000, 0xE0B00000 };
#define RR_NSECT    2
#define RR_L1_SIZE  0x4900              /* Canon L1 (0x4000) + core L2s for 0-1 MB + core L1s for 0-32 MB */
#define RR_L2_OFF   0x4C00              /* RR_NSECT x 1 KB L2 tables */
#define RR_PAGE_OFF 0x6000              /* RR_NPAGES x 4 KB RAM pages */
#define RR_SPAN     (RR_PAGE_OFF + RR_NPAGES * 0x1000)
#define RR_ALLOC    (RR_SPAN + 0x4000)  /* slack for the 16 KB L1 alignment: a full 0x4000 (silenttest v3.15) */

static uint8_t *rmp_buf, *rmp_l1;
static int rmp_from_heap, rmp_on, rmp_err;
static char rmp_stage[64];

static void rmp_log(const char *stage, uint32_t a, uint32_t b)
{
    snprintf(rmp_stage, sizeof(rmp_stage), "%s %x %x", stage, (unsigned)a, (unsigned)b);
}

static int rmp_setup(void)
{
    if (get_cpu_id() != 0) return -1;
    if (!rmp_buf)
    {
        uint8_t *p = _AllocateMemory(RR_ALLOC);
        if (p && (uint32_t)p >= 0x40000000) p = 0;      /* must be a plain cacheable RAM address (TTBR, L2 pointers); leaked */
        if (!p) { p = _malloc(RR_ALLOC); rmp_from_heap = 1; }
        if (!p || (uint32_t)p >= 0x40000000) return -10;
        rmp_buf = p;
    }
    uint32_t l1 = ((uint32_t)rmp_buf + 0x3FFF) & ~0x3FFF;
    if (l1 + RR_SPAN > (uint32_t)rmp_buf + RR_ALLOC) return -9;
    rmp_l1 = (uint8_t *)l1;
    rmp_log(rmp_from_heap ? "init heap" : "init AM", l1, l1 + RR_PAGE_OFF);
    if (copy_mmu_tables_ex(l1, 0xE0000000, RR_L1_SIZE)) return -2;
    uint32_t *a = (uint32_t *)l1, *b = (uint32_t *)0xE0000000, diff = 0;
    for (int i = 0; i < RR_L1_SIZE / 4; i++) if (a[i] != b[i] && i != 0x1200 && i != 0x1220) diff++;
    rmp_log("copy-diff", diff, a[0xE0F00000 >> 20]);
    if (diff) return -3;
    if (split_l1_supersection(0xE0000000, l1)) return -4;
    for (int s = 0; s < RR_NSECT; s++)
    {
        uint32_t sect = rmp_sections[s];
        uint32_t lf = get_l2_largepage_flags_from_l1_section(sect, l1);
        if (lf == 0xffffffff) return -5;
        /* large-page flags -> small-page descriptor: B C AP AP2 S nG stay, TEX 14:12 -> 8:6, XN 15 -> 0, type 0b1x */
        uint32_t sp = (lf & 0xE3C) | (((lf >> 12) & 7) << 6) | ((lf >> 15) & 1) | 2;
        uint32_t *l2 = (uint32_t *)(l1 + RR_L2_OFF + s * 0x400);
        for (int i = 0; i < 256; i++) l2[i] = (sect + i * 0x1000) | sp;
        for (int p = 0; p < RR_NPAGES; p++)
        {
            if ((rmp_pages[p] & 0xFFF00000) != sect) continue;
            uint8_t *ram = (uint8_t *)(l1 + RR_PAGE_OFF + p * 0x1000);
            memcpy(ram, (void *)rmp_pages[p], 0x1000);
            l2[(rmp_pages[p] - sect) >> 12] = (uint32_t)ram | sp;
        }
        if (assign_l2_table_to_l1_section(sect, l1, (uint32_t)l2)) return -6;
        rmp_log("tables", sect, sp);
    }
    dcache_clean(l1, RR_SPAN);
    rmp_log("ttbr-next", 0, 0);
    uint32_t old = cli();
    change_mmu_tables(rmp_l1 + RR_L1_SIZE - 0x100, rmp_l1, 0);   /* also TLBIALL */
    sei(old);
    _sync_caches();
    rmp_on = 1;
    /* the pages are still identical to ROM: prove each is mapped by writing a probe word through RAM and reading
     * it back through the ROM address, then restoring it */
    for (int p = 0; p < RR_NPAGES; p++)
    {
        volatile uint32_t *ram = (volatile uint32_t *)(l1 + RR_PAGE_OFF + p * 0x1000 + 0xFFC);
        volatile uint32_t *rom = (volatile uint32_t *)(rmp_pages[p] + 0xFFC);
        uint32_t keep = *ram, seen;
        uint32_t irq = cli();           /* no Canon task may read the probe value */
        *ram = keep ^ 0x5A5A5A5A;
        seen = *rom;
        *ram = keep;
        sei(irq);
        if (seen != (keep ^ 0x5A5A5A5A)) { rmp_log("probe", rmp_pages[p], seen); return -7; }
    }
    rmp_log("ttbr-done", rmp_pages[0], RR_NPAGES);
    return 0;
}

int romremap_init(void)
{
    if (rmp_on) return 0;
    if (rmp_err) return rmp_err;          /* don't retry a failed setup in the same power-on */
    rmp_err = rmp_setup();
    return rmp_err;
}

int romremap_on(void) { return rmp_on; }
const char *romremap_stage(void) { return rmp_stage; }

uint8_t *romremap_ram(uint32_t rom_addr)
{
    if (!rmp_on) return NULL;
    for (int p = 0; p < RR_NPAGES; p++)
        if ((rom_addr & ~0xFFF) == rmp_pages[p]) return rmp_l1 + RR_PAGE_OFF + p * 0x1000 + (rom_addr & 0xFFF);
    return NULL;
}

void romremap_flush(void *ram, size_t n)
{
    uint32_t a = (uint32_t)ram & ~0x1F;
    dcache_clean(a, ((uint32_t)ram + n - a + 0x1F) & ~0x1F);
}

/* v4.64: switch cpu1 to the same tables via Canon's inter-core RPC (stub _request_RPC 0xDF002BAA: the other core
 * calls f(arg) until clear_RPC_request()). The R's task_create_ex can't target cpu1 and CONFIG_RPC/SGI are off, so
 * this is the only way to run code there. The callee checks it really runs on cpu1, switches TTBR with cpu1's
 * per-core part (L1 + 0x4880, the slot copy_mmu_tables_ex fixes up for cpu1), then reads probe_addr through the ROM
 * address so cpu0 can tell whether cpu1 sees the remapped page. Returns 0 if cpu1 switched; results in cpu, val. */
extern int _request_RPC(void (*f)(void *), void *o);
extern int clear_RPC_request(void);
static volatile int rr1_done, rr1_cpu, rr1_calls;
static volatile uint32_t rr1_val, rr1_probe_addr;
static volatile int rmp_on1;
static void rr1_func(void *unused)
{
    rr1_calls++;
    if (!rr1_done)
    {
        int cpu = get_cpu_id();
        rr1_cpu = cpu;
        if (cpu == 1 && rmp_on && !rmp_on1)   /* v4.72: later calls only probe */
        {
            uint32_t old = cli();
            change_mmu_tables(rmp_l1 + RR_L1_SIZE - 0x100 + 0x80, rmp_l1, 1);
            sei(old);
            rmp_on1 = 1;
        }
        rr1_val = rr1_probe_addr ? *(volatile uint32_t *)rr1_probe_addr : 0;
        rr1_done = 1;
    }
    clear_RPC_request();
}

int romremap_cpu1(uint32_t probe_addr, int *cpu, uint32_t *val, int *calls)
{
    if (!rmp_on) return -1;
    if (get_cpu_id() != 0) return -2;
    rr1_done = 0; rr1_cpu = -1; rr1_val = 0; rr1_calls = 0; rr1_probe_addr = probe_addr;
    int st = _request_RPC(rr1_func, 0);
    for (int i = 0; i < 100 && !rr1_done; i++) msleep(10);
    *cpu = rr1_cpu; *val = rr1_val; *calls = rr1_calls;
    if (!rr1_done) return 0x100 | (st & 0xff);
    return rmp_on1 ? 0 : -3;
}
int romremap_on_cpu1(void) { return rmp_on1; }

#else
int romremap_init(void) { return -1; }
int romremap_on(void) { return 0; }
const char *romremap_stage(void) { return "no MMU"; }
uint8_t *romremap_ram(uint32_t rom_addr) { return NULL; }
void romremap_flush(void *ram, size_t n) { }
int romremap_cpu1(uint32_t probe_addr, int *cpu, uint32_t *val, int *calls) { return -1; }
int romremap_on_cpu1(void) { return 0; }
#endif
