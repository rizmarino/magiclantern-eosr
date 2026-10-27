/* riz/eosr-dmlog: capture Canon's own debug log for the live-view classes.
 *
 * Debug -> LV log capture. Uses only Canon's DbgMgr (store levels, dmstart,
 * dmstop, dumpf); nothing in ROM is patched. For N seconds, every message from
 * the live-view classes is stored (all other classes muted), then Canon's dumpf
 * writes the log to the card root as LOGnnn.LOG and the camera beeps.
 * Class IDs come from DryosDebugMsg call sites in the 1.8.0 ROM
 * (~/ml/ghidra/notes/lv-survey.md). Personal diagnostic build, not for upstream.
 */
#include "dryos.h"
#include "menu.h"
#include "beep.h"
#include "propvalues.h"
#include "config.h"

extern void dm_set_store_level(uint32_t class, uint32_t level); /* stubs.S, 0xE0563720 */

static const uint8_t lv_classes[] = {
    0xC9,   /* raw2yuv, LvDev*Controller (movie dev paths) */
    0x9C,   /* LvBufferController, LvComTim, LvMovieLog */
    0x9D,   /* MvRawProPath, Path.c */
    0x9E,   /* SensorDrive */
    0xCA,   /* DevPathExec, DeliverStage */
    0x93,   /* VramSubStage, CompositeHdrStage */
    0x8C,   /* ImageSequence / ImageController */
    0xB7,   /* EvfCommand, Vram */
};

static CONFIG_INT("r.lvlog.secs", lvlog_secs_idx, 1);
static const int lvlog_secs[] = { 3, 10, 30 };
static volatile int lvlog_busy = 0;
static int lvlog_runs = 0;

static void lvlog_task(void *unused)
{
    int secs = lvlog_secs[lvlog_secs_idx];

    dm_set_store_level(0xFF, 15);               /* mute everything (level 15 only) */
    for (int i = 0; i < COUNT(lv_classes); i++)
        dm_set_store_level(lv_classes[i], 0);   /* keep every level of the LV classes */

    call("dmstart");
    DryosDebugMsg(0, 15, "=== riz lvlog start run %d: %ds lv=%d movie=%d dispsize=%d",
                  lvlog_runs, secs, lv, is_movie_mode(), lv_dispsize);
    msleep(secs * 1000);
    DryosDebugMsg(0, 15, "=== riz lvlog stop run %d: lv=%d movie=%d dispsize=%d",
                  lvlog_runs, lv, is_movie_mode(), lv_dispsize);
    call("dmstop");
    call("dumpf");

    lvlog_runs++;
    beep();
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
        MENU_SET_VALUE("recording...");
    else
        MENU_SET_VALUE("%ds, %d saved", lvlog_secs[lvlog_secs_idx], lvlog_runs);
}

static struct menu_entry lvlog_menu[] = {
    {
        .name   = "LV log capture",
        .select = lvlog_select,
        .update = lvlog_update,
        .help   = "Stores Canon's live-view log for N s, then saves LOGnnn.LOG.",
        .help2  = "Press SET, close the menu, do ONE live-view action, wait for beep.",
        .children = (struct menu_entry[]) {
            {
                .name    = "Duration",
                .priv    = &lvlog_secs_idx,
                .max     = 2,
                .choices = CHOICES("3 s", "10 s", "30 s"),
                .help    = "Short captures avoid wrapping Canon's log buffer.",
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
