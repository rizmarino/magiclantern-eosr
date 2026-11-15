/* R: Debug > Sensor regs log (EOS R 1.8.0, v4.58). Personal diagnostic, not for upstream.
 *
 * Read-only survey for wider/taller 1:1 4K (crop_rec on the R). While "Watch" is ON a task polls, every 50 ms,
 * Canon's CMOS command buffers (the lists FUN_e06a19a4 sends to the sensor, 0xffffffff-terminated):
 *   - 10 LV buffers at *0x176a8 + 0x1bac + n * 0xac (FUN_e06cb1f4), the still buffer at *0x1769c + 0x28dc;
 *   - m_SensorMode (ImageSequence state 0xef24 + 0xc), Mem1Path size (0x133CC + 0x10/0x14/0x18);
 *   - SensorDrive state 0xcfd4 (+0x70 fps slot, +0x78..0x80 HD/line timing) and SetTgParam struct 0x5825c.
 * Whenever the buffers, the sensor mode or the MEM1 size change it keeps a snapshot (max SR_NSNAP). "Save log"
 * appends them to ML/LOGS/SENSREG.TXT. Diffing photo LV / FHD / 4K / magnify snapshots should show which
 * registers set the readout window and the line skipping (command word = reg << 22 | data).
 * Reads RAM only (the same reads v4.52 ran on the camera); no ROM, register or property writes.
 */
#include "dryos.h"
#include "menu.h"
#include "fio-ml.h"

#define SR_LOG      "ML/LOGS/SENSREG.TXT"
#define SR_STILLP   0x1769c
#define SR_LVP      0x176a8
#define SR_NLV      10
#define SR_LVW      (0xac / 4)
#define SR_STW      128
#define SR_STATE    0xef24
#define SR_M1P      0x133CC
#define SR_SDRV     0xcfd4
#define SR_SDW      40
#define SR_TG       0x5825c
#define SR_TGW      20
#define SR_NSNAP    40

struct sr_snap {
    uint32_t ms, mode, m1w, m1h, m1w18, lvbase, stbase;
    uint32_t sd[SR_SDW], tg[SR_TGW];
    uint32_t lv[SR_NLV][SR_LVW];
    uint32_t st[SR_STW];
};

static struct sr_snap *sr_snaps;          /* malloc'd at the first Watch ON (~100 KB): bss would cost Canon heap */
static volatile int sr_n, sr_lost, sr_polls, sr_bad_ptr;
static volatile int sr_run, sr_task_alive;
static uint32_t sr_last_hash;
static int sr_saved_total;

static int sr_ram_ok(uint32_t a) { return a >= 0x1000 && a < 0x80000000 && !(a & 3); }

static uint32_t sr_fnv(uint32_t h, uint32_t v)
{
    for (int i = 0; i < 4; i++) { h ^= (v >> (i * 8)) & 0xff; h *= 16777619u; }
    return h;
}

/* read everything into s; returns the change hash */
static uint32_t sr_read(struct sr_snap *s)
{
    uint32_t h = 2166136261u;
    s->ms = get_ms_clock();
    s->mode = *(volatile uint32_t *)(SR_STATE + 0xc);
    s->m1w = *(volatile uint32_t *)(SR_M1P + 0x10);
    s->m1h = *(volatile uint32_t *)(SR_M1P + 0x14);
    s->m1w18 = *(volatile uint32_t *)(SR_M1P + 0x18);
    h = sr_fnv(h, s->mode); h = sr_fnv(h, s->m1w); h = sr_fnv(h, s->m1h);
    for (int i = 0; i < SR_SDW; i++) s->sd[i] = *(volatile uint32_t *)(SR_SDRV + i * 4);
    for (int i = 0; i < SR_TGW; i++) s->tg[i] = *(volatile uint32_t *)(SR_TG + i * 4);
    for (int i = 0x70 / 4; i <= 0x80 / 4; i++) h = sr_fnv(h, s->sd[i]);    /* fps slot + line timing only */

    uint32_t p = *(volatile uint32_t *)SR_LVP;
    s->lvbase = p;
    for (int n = 0; n < SR_NLV; n++)
    {
        int end = 0;
        for (int i = 0; i < SR_LVW; i++)
        {
            uint32_t v = 0;
            if (!end && sr_ram_ok(p)) v = *(volatile uint32_t *)(p + 0x1bac + n * 0xac + i * 4);
            s->lv[n][i] = v;
            if (v == 0xffffffff) end = 1;
            h = sr_fnv(h, v);
        }
    }
    uint32_t q = *(volatile uint32_t *)SR_STILLP;
    s->stbase = q;
    int end = 0;
    for (int i = 0; i < SR_STW; i++)
    {
        uint32_t v = 0;
        if (!end && sr_ram_ok(q)) v = *(volatile uint32_t *)(q + 0x28dc + i * 4);
        s->st[i] = v;
        if (v == 0xffffffff) end = 1;
        h = sr_fnv(h, v);
    }
    if (!sr_ram_ok(p) || !sr_ram_ok(q)) sr_bad_ptr++;
    return h;
}

static void sr_task(void *unused)
{
    sr_task_alive = 1;
    struct sr_snap *tmp = &sr_snaps[SR_NSNAP];   /* scratch slot at the end of the malloc'd array */
    while (sr_run)
    {
        uint32_t h = sr_read(tmp);
        sr_polls++;
        if (h != sr_last_hash)
        {
            sr_last_hash = h;
            int i = sr_n;
            if (i < SR_NSNAP) { memcpy(&sr_snaps[i], tmp, sizeof(*tmp)); sr_n = i + 1; }
            else sr_lost++;
        }
        msleep(50);
    }
    sr_task_alive = 0;
}

/* ---- log writing (menu task) ---- */
#define SR_BUF 4096
static char *sr_buf;
static int sr_len;
static FILE *sr_f;
static void sr_flush(void)
{
    if (sr_len && sr_f) FIO_WriteFile(sr_f, sr_buf, sr_len);
    sr_len = 0;
}
static void sr_printf(const char *fmt, ...)
{
    if (sr_len > SR_BUF - 200) sr_flush();
    va_list ap;
    va_start(ap, fmt);
    sr_len += vsnprintf(sr_buf + sr_len, SR_BUF - sr_len, fmt, ap);
    va_end(ap);
}
static void sr_words(const char *tag, const uint32_t *w, int n, int stop_ff)
{
    int last = n;
    if (stop_ff) { for (int i = 0; i < n; i++) if (w[i] == 0xffffffff) { last = i + 1; break; } }
    else { while (last > 0 && w[last - 1] == 0) last--; }
    sr_printf("  %s (%d):", tag, last);
    for (int i = 0; i < last; i++)
    {
        if (i && i % 8 == 0) sr_printf("\n     ");
        sr_printf(" %08x", (int)w[i]);
    }
    sr_printf("\n");
}

static int sr_save(void)
{
    if (!sr_buf && !(sr_buf = malloc(SR_BUF))) return -1;
    sr_f = FIO_CreateFileOrAppend(SR_LOG);
    if (!sr_f) return -2;
    int n = sr_n;
    sr_printf("=== sensreg v4.58 save at %d ms: %d snapshots (%d lost), %d polls, bad pointers %d\n",
              (int)get_ms_clock(), n, sr_lost, sr_polls, sr_bad_ptr);
    for (int k = 0; k < n; k++)
    {
        struct sr_snap *s = &sr_snaps[k];
        sr_printf("--- snap %d t=%d ms: m_SensorMode %d, MEM1 %dx%d (w18 %d), sdrv slot %d HD %d/%d/%d, LV %x still %x\n",
                  sr_saved_total + k, (int)s->ms, (int)s->mode, (int)s->m1w, (int)s->m1h, (int)s->m1w18,
                  (int)s->sd[0x70 / 4], (int)s->sd[0x78 / 4], (int)s->sd[0x7c / 4], (int)s->sd[0x80 / 4],
                  (int)s->lvbase, (int)s->stbase);
        sr_words("sdrv cfd4", s->sd, SR_SDW, 0);
        sr_words("tg 5825c", s->tg, SR_TGW, 0);
        for (int b = 0; b < SR_NLV; b++)
        {
            char t[8];
            snprintf(t, sizeof(t), "lv%d", b);
            sr_words(t, s->lv[b], SR_LVW, 1);
        }
        sr_words("still", s->st, SR_STW, 1);
        sr_flush();
    }
    sr_flush();
    FIO_CloseFile(sr_f);
    sr_f = 0;
    sr_saved_total += n;
    sr_n = 0; sr_lost = 0;
    return n;
}

/* ---- menu ---- */
static void sr_watch_toggle(void *priv, int delta)
{
    if (sr_run) { sr_run = 0; NotifyBox(2000, "Sensor regs: watch OFF (%d snapshots kept)", sr_n); return; }
    if (sr_task_alive) { NotifyBox(2000, "Sensor regs: still stopping, try again"); return; }
    if (!sr_snaps && !(sr_snaps = malloc((SR_NSNAP + 1) * sizeof(struct sr_snap))))
    {
        NotifyBox(3000, "Sensor regs: not enough memory");
        return;
    }
    sr_last_hash = 0; sr_polls = 0; sr_bad_ptr = 0;
    sr_run = 1;
    task_create("sensreg", 0x1e, 0x1000, sr_task, 0);
    NotifyBox(2000, "Sensor regs: watch ON");
}

static void sr_save_select(void *priv, int delta)
{
    if (sr_run)                             /* stop the watch first: the task appends to sr_snaps */
    {
        sr_run = 0;
        for (int i = 0; i < 40 && sr_task_alive; i++) msleep(25);
        if (sr_task_alive) { NotifyBox(3000, "Sensor regs: watch still stopping, Save again"); return; }
    }
    if (!sr_snaps) { NotifyBox(2000, "Sensor regs: nothing recorded (Watch was never ON)"); return; }
    int r = sr_save();
    if (r >= 0) NotifyBox(3000, "Saved %d snapshots to " SR_LOG, r);
    else NotifyBox(3000, "Save failed (%d)", r);
}

static MENU_UPDATE_FUNC(sr_update)
{
    if (sr_run) MENU_SET_VALUE("ON, %d snaps", sr_n);
    else MENU_SET_VALUE("OFF, %d snaps", sr_n);
    if (sr_lost) MENU_SET_WARNING(MENU_WARN_INFO, "%d changes not kept (full): Save log, then go on.", sr_lost);
}

static struct menu_entry sr_menu[] = {
    {
        .name   = "Sensor regs log",
        .select = menu_open_submenu,
        .update = sr_update,
        .help   = "Read-only: records Canon's sensor command lists whenever the LV mode changes.",
        .help2  = "For the wider-4K research. Log: " SR_LOG ". Always OFF at power-on.",
        .children = (struct menu_entry[]) {
            {
                .name   = "Watch",
                .select = sr_watch_toggle,
                .update = sr_update,
                .help   = "SET: ON/OFF. ON keeps a snapshot at every change (max 40).",
            },
            {
                .name   = "Save log",
                .select = sr_save_select,
                .help   = "Stops Watch, appends the snapshots to " SR_LOG " and clears them.",
            },
            MENU_EOL
        },
    },
};

static void sr_init(void *unused)
{
    menu_add("Debug", sr_menu, COUNT(sr_menu));
}

INIT_FUNC("sensreg_r", sr_init);
