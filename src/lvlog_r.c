/* riz/eosr-dmlog: capture Canon's own debug log for the live-view classes.
 *
 * Debug -> LV log capture. Uses only Canon's DbgMgr store levels and dumpf;
 * nothing in ROM is patched. Mutes all classes except the live-view ones,
 * waits for a live-view change (LV on/off, photo/movie, recording, panel/EVF,
 * zoom), keeps logging 1.5 s after it, then freezes Canon's log buffer by
 * muting everything and saves it with dumpf (LOGnnn.LOG on the card root).
 *
 * v1 findings (2026-10-27): Canon's log buffer holds ~350 KB, ~3.7 s of these
 * classes at 60 fps, so captures must be triggered; dmstop stops storage for
 * good (dmstart doesn't resume it), and storage is already on at boot, so
 * neither is used here.
 * Class IDs come from DryosDebugMsg call sites in the 1.8.0 ROM
 * (~/ml/ghidra/notes/lv-survey.md). Personal diagnostic build, not for upstream.
 */
#include "dryos.h"
#include "menu.h"
#include "beep.h"
#include "propvalues.h"
#include "config.h"
#include "consts.h"
#include "timer.h"

extern void dm_set_store_level(uint32_t class, uint32_t level); /* stubs.S, 0xE0563720 */

static const uint8_t lv_classes[] = {
    0xC9,   /* EPPVRM: raw2yuv, LvDev*Controller (movie dev paths) */
    0x9C,   /* LVCOM:  LvBufferController, LvComTim, LvMovieLog */
    0x9D,   /* LVCPTH: MvRawProPath, Path.c */
    0x9E,   /* LVSDV:  SensorDrive */
    0xCA,   /* EPPDEL: DevPathExec, DeliverStage */
    0x93,   /* LVEPP:  VramSubStage, CompositeHdrStage */
    0x8C,   /* CAPD:   ImageSequence / ImageController */
    0xB7,   /* LVEVFD: EvfCommand, Vram */
};

#define LVLOG_AFTER_MS 1500

static CONFIG_INT("r.lvlog.wait", lvlog_wait_idx, 1);
static const int lvlog_wait_s[] = { 10, 30, 60 };
static volatile int lvlog_busy = 0;
static int lvlog_runs = 0;
static int lvlog_last_trig = 0;

static int lvlog_sig(void)
{
    return (lv ? 1 : 0) | (is_movie_mode() ? 2 : 0) | (RECORDING ? 4 : 0)
         | (R_LV_ON_EVF() ? 8 : 0) | (lv_dispsize << 8);
}

static void lvlog_task(void *unused)
{
    int wait_ms = lvlog_wait_s[lvlog_wait_idx] * 1000;

    dm_set_store_level(0xFF, 15);               /* mute everything (level 15 only) */
    for (int i = 0; i < COUNT(lv_classes); i++)
        dm_set_store_level(lv_classes[i], 0);   /* keep every level of the LV classes */

    int sig0 = lvlog_sig();
    DryosDebugMsg(0, 15, "=== riz lvlog v2 run %d armed: sig=%x lv=%d movie=%d rec=%d evf=%d dispsize=%d",
                  lvlog_runs, sig0, lv, is_movie_mode(), RECORDING, R_LV_ON_EVF(), lv_dispsize);

    int t0 = get_ms_clock(), trig = 0, sig1 = sig0;
    while (1)
    {
        msleep(20);
        int now = get_ms_clock();
        if (!trig && (sig1 = lvlog_sig()) != sig0)
        {
            trig = now;
            DryosDebugMsg(0, 15, "=== riz lvlog v2 run %d TRIGGER: sig %x -> %x after %d ms",
                          lvlog_runs, sig0, sig1, now - t0);
        }
        if (trig && now - trig >= LVLOG_AFTER_MS)
            break;
        if (!trig && now - t0 >= wait_ms)
            break;
    }

    DryosDebugMsg(0, 15, "=== riz lvlog v2 run %d freeze: %s sig=%x",
                  lvlog_runs, trig ? "triggered" : "timeout", lvlog_sig());
    dm_set_store_level(0xFF, 15);               /* freeze Canon's log buffer */
    msleep(200);
    call("dumpf");

    lvlog_last_trig = trig ? 1 : 0;
    lvlog_runs++;
    /* no CONFIG_BEEP on the R: show the result on screen instead */
    NotifyBox(6000, trig ? "LV log: DONE (change caught)" : "LV log: DONE (timed out, no change)");
    info_led_blink(trig ? 1 : 3, 100, 100);
    lvlog_busy = 0;
}

static MENU_SELECT_FUNC(lvlog_select)
{
    if (lvlog_busy)
        return;
    lvlog_busy = 1;
    task_create("lvlog_task", 0x1e, 0x1000, lvlog_task, 0);
}

static MENU_UPDATE_FUNC(lvlog_update)
{
    if (lvlog_busy)
        MENU_SET_VALUE("waiting for change...");
    else if (lvlog_runs)
        MENU_SET_VALUE("%d saved, last %s", lvlog_runs, lvlog_last_trig ? "triggered" : "timed out");
    else
        MENU_SET_VALUE("wait %ds", lvlog_wait_s[lvlog_wait_idx]);
}

static struct menu_entry lvlog_menu[] = {
    {
        .name   = "LV log capture",
        .select = lvlog_select,
        .update = lvlog_update,
        .help   = "Saves Canon's LV log around the next LV change (mode, rec, EVF).",
        .help2  = "SET, close menu, make ONE change, wait for DONE message.",
        .children = (struct menu_entry[]) {
            {
                .name    = "Max wait",
                .priv    = &lvlog_wait_idx,
                .max     = 2,
                .choices = CHOICES("10 s", "30 s", "60 s"),
                .help    = "How long to wait for a change before saving anyway.",
            },
            MENU_EOL
        },
    },
};

static void lvlog_init(void *unused)
{
    menu_add("Debug", lvlog_menu, COUNT(lvlog_menu));
}

INIT_FUNC("lvlog", lvlog_init);
