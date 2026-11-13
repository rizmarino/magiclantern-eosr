/* EOS R: Rack focus between two stored points (v4.56). Focus > Rack focus.
 *
 * Stock ML's Focus End Point / Rack Focus count focus steps and set the end point in the "menu hidden, live view
 * visible" mode, which the R doesn't have (its ML menu sits on Canon dialog 0x68; menu_enable_lv_transparent_mode()
 * is a no-op), so they were dropped in v4.40. Here the points are taken from the lens itself: focus on A with AF-ON,
 * SET on "Point A"; same for B. A rack closes the menu and steps the lens (lens_focus(), open loop, camera-verified
 * v4.40) until the lens-reported focus distance reaches the other point. Positive steps go nearer (foctest
 * 2026-11-08: +3 x size 3 = 620 -> 220). A rack stops early when the ML menu opens or the half-shutter is pressed.
 * Log: ML/LOGS/RACKFOC.TXT, one line per move. */

#include "dryos.h"
#include "menu.h"
#include "config.h"
#include "lens.h"
#include "focus.h"
#include "fio-ml.h"
#include "propvalues.h"
#include "property.h"

extern int lens_focus(int num_steps, int stepsize, int wait, int extra_delay);

#define RF_LOG "ML/LOGS/RACKFOC.TXT"

static CONFIG_INT("rackfoc.a.dist", rf_a_dist, 0);
static CONFIG_INT("rackfoc.a.pos", rf_a_pos, 0);
static CONFIG_INT("rackfoc.b.dist", rf_b_dist, 0);
static CONFIG_INT("rackfoc.b.pos", rf_b_pos, 0);
static CONFIG_INT("rackfoc.speed", rf_speed, 1);         /* 0..2 = step size 1..3 */
static CONFIG_INT("rackfoc.interval", rf_interval, 1);   /* index into rf_iv_ms */
static CONFIG_INT("rackfoc.delay", rf_delay, 0);         /* seconds before a rack starts */
static const int rf_iv_ms[] = { 80, 120, 200, 300, 500 };

static volatile int rf_busy;
static char rf_last[80];

static int rf_have(int which) { return which ? rf_b_dist > 0 : rf_a_dist > 0; }

static void rf_log(const char *fmt, ...)
{
    char line[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    FIO_CreateDirectory("ML/LOGS");
    FILE *f = FIO_CreateFileOrAppend(RF_LOG);
    if (f) { FIO_WriteFile(f, line, n); FIO_CloseFile(f); }
}

/* why the lens can't be driven now, or NULL */
static const char *rf_blocked(void)
{
    if (!lv) return "needs LiveView";
    if (is_manual_focus()) return "lens is in MF: set AF";
    if (is_continuous_af())
        return lv_movie_select == LVMS_ENABLE_MOVIE ? "Movie Servo AF is on: disable it" : "Continuous AF is on: disable it";
    if (lens_info.job_state) return "camera busy";
    return NULL;
}

struct rf_job { int target; int speed; int delay; int fast; };
static struct rf_job rf_job;

/* move until the lens-reported distance reaches target_dist (stop on crossing) */
static void rf_task(void *unused)
{
    struct rf_job j = rf_job;
    int to = j.target;
    int tdist = to ? rf_b_dist : rf_a_dist, tpos = to ? rf_b_pos : rf_a_pos;
    int size = j.fast ? 3 : j.speed + 1, iv = j.fast ? 80 : rf_iv_ms[COERCE(rf_interval, 0, COUNT(rf_iv_ms) - 1)];

    gui_stop_menu();
    if (j.delay)
        for (int s = j.delay; s > 0; s--)
        {
            NotifyBox(1100, "Rack focus to %s in %d s", to ? "B" : "A", s);
            msleep(1000);
        }
    msleep(300);                        /* let the menu close: the abort check watches for it */

    const char *why = rf_blocked();
    int d0 = lens_info.focus_dist, p0 = lens_info.focus_pos, steps = 0, still = 0, t0 = get_ms_clock();
    int dir = tdist < d0 ? 1 : -1;      /* + = nearer */
    int last = d0, flipped = 0;
    /* how many steps without a distance change count as "lens at its end" */
    int still_max = size == 1 ? 24 : size == 2 ? 10 : 4;
    if (!why && tdist == d0) why = "already there";
    while (!why)
    {
        if (gui_menu_shown()) { why = "cancelled (ML menu)"; break; }
        if (get_halfshutter_pressed()) { why = "cancelled (half-press)"; break; }
        if (steps >= 400) { why = "step limit (400)"; break; }
        if (!lens_focus(dir, size, 0, iv)) { why = rf_blocked(); if (!why) why = "lens_focus refused"; break; }
        steps++;
        int d = lens_info.focus_dist;
        if (dir > 0 ? d <= tdist : d >= tdist) { why = "reached"; break; }
        /* other lenses may step the other way: after 3 steps, turn round once if we moved away from the target */
        if (steps == 3 && !flipped && ABS(d - tdist) > ABS(d0 - tdist)) { dir = -dir; flipped = 1; still = 0; }
        if (d == last) { if (++still >= still_max) { why = "lens stopped moving (end of range?)"; break; } }
        else { still = 0; last = d; }
    }
    msleep(150);
    int d1 = lens_info.focus_dist, p1 = lens_info.focus_pos, ms = get_ms_clock() - t0;
    snprintf(rf_last, sizeof(rf_last), "to %s: %s, %d steps, %d.%d s", to ? "B" : "A", why, steps, ms / 1000, (ms / 100) % 10);
    rf_log("%d ms: %s to %s: target dist %d pos %d; size %d, interval %d ms, delay %d s; start dist %d pos %d; end dist %d "
           "pos %d; %d steps in %d ms%s; %s; movie %d servo photo %d movie %d af_mode %d\n", t0, j.fast ? "go" : "rack", to ? "B" : "A", tdist, tpos, size, iv, j.delay,
           d0, p0, d1, p1, steps, ms, flipped ? " (direction flipped)" : "", why,
           lv_movie_select == LVMS_ENABLE_MOVIE, continuous_af_photo, continuous_af_movie, af_mode);
    NotifyBox(3000, "Rack focus %s", rf_last);
    rf_busy = 0;
}

static void rf_start(int target, int fast)
{
    if (rf_busy) return;
    if (!rf_have(target)) { NotifyBox(3000, "Set point %s first (focus with AF-ON, then SET)", target ? "B" : "A"); return; }
    const char *why = rf_blocked();
    if (why) { NotifyBox(3000, "Rack focus: %s", why); return; }
    rf_job.target = target; rf_job.speed = COERCE(rf_speed, 0, 2); rf_job.fast = fast;
    rf_job.delay = fast ? 0 : COERCE(rf_delay, 0, 30);
    rf_busy = 1;
    task_create("rack_focus", 0x1c, 0x1000, rf_task, 0);
}

static void rf_store(int which)
{
    if (!lv) { NotifyBox(3000, "Rack focus: needs LiveView"); return; }
    int d = lens_info.focus_dist, p = lens_info.focus_pos;
    if (d <= 0) { NotifyBox(3000, "Lens reports no focus distance"); return; }
    if (which) { rf_b_dist = d; rf_b_pos = p; } else { rf_a_dist = d; rf_a_pos = p; }
    rf_log("%d ms: point %s stored: dist %d pos %d (%s); movie %d servo photo %d movie %d af_mode %d\n", (int)get_ms_clock(),
           which ? "B" : "A", d, p, lens_info.name, lv_movie_select == LVMS_ENABLE_MOVIE, continuous_af_photo,
           continuous_af_movie, af_mode);
    NotifyBox(2500, "Point %s stored: %d cm", which ? "B" : "A", d);
}

static MENU_SELECT_FUNC(rf_set_a) { rf_store(0); }
static MENU_SELECT_FUNC(rf_set_b) { rf_store(1); }
static MENU_SELECT_FUNC(rf_go_a) { rf_start(0, 1); }
static MENU_SELECT_FUNC(rf_go_b) { rf_start(1, 1); }
static MENU_SELECT_FUNC(rf_rack)
{
    if (!rf_have(0) || !rf_have(1)) { NotifyBox(3000, "Set points A and B first"); return; }
    /* rack to whichever point is farther from where the lens is now */
    int d = lens_info.focus_dist;
    rf_start(ABS(d - rf_a_dist) <= ABS(d - rf_b_dist) ? 1 : 0, 0);
}

static MENU_UPDATE_FUNC(rf_point_update)
{
    int b = entry->priv == (void *)1;
    int d = b ? rf_b_dist : rf_a_dist;
    if (d > 0) MENU_SET_VALUE("%d cm", d);
    else MENU_SET_VALUE("not set");
    MENU_SET_WARNING(MENU_WARN_INFO, "Now: %d cm. Focus with AF-ON, then SET to store.", (int)lens_info.focus_dist);
}

static MENU_UPDATE_FUNC(rf_rack_update)
{
    if (rf_busy) { MENU_SET_VALUE("running"); return; }
    if (rf_last[0]) MENU_SET_VALUE("%s", rf_last);
    const char *why = rf_blocked();
    if (!rf_have(0) || !rf_have(1)) MENU_SET_WARNING(MENU_WARN_ADVICE, "Store points A and B first.");
    else if (why) MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Can't drive the lens: %s.", why);
    else MENU_SET_WARNING(MENU_WARN_INFO, "Goes to the point farther from the lens now. Half-press = stop.");
}

static MENU_UPDATE_FUNC(rf_main_update)
{
    if (rf_have(0) && rf_have(1)) MENU_SET_VALUE("%d <-> %d cm", rf_a_dist, rf_b_dist);
    else MENU_SET_VALUE("set A and B");
}

static struct menu_entry rf_menu[] = {
    {
        .name   = "Rack focus",
        .select = menu_open_submenu,
        .update = rf_main_update,
        .help   = "Move focus smoothly between two stored points (A, B).",
        .help2  = "Lens on AF, Servo/continuous AF off. Points come from the lens.",
        .children = (struct menu_entry[]) {
            {
                .name   = "Point A",
                .priv   = (void *)0,
                .select = rf_set_a,
                .update = rf_point_update,
                .help   = "Focus on the first subject with AF-ON, open this menu, SET here.",
            },
            {
                .name   = "Point B",
                .priv   = (void *)1,
                .select = rf_set_b,
                .update = rf_point_update,
                .help   = "Focus on the second subject with AF-ON, open this menu, SET here.",
            },
            {
                .name   = "Rack now",
                .select = rf_rack,
                .update = rf_rack_update,
                .help   = "SET: closes the menu and racks to the other point (after Start delay).",
                .help2  = "Don't press AF-ON while it runs. Half-press or opening ML stops it.",
            },
            {
                .name    = "Speed",
                .priv    = &rf_speed,
                .max     = 2,
                .choices = CHOICES("Slow (step 1)", "Medium (step 2)", "Fast (step 3)"),
                .help    = "Focus step size per command. Slow is smoothest but may be very slow",
                .help2   = "on some lenses; Fast jumps in coarse steps.",
            },
            {
                .name    = "Step interval",
                .priv    = &rf_interval,
                .max     = 4,
                .choices = CHOICES("80 ms", "120 ms", "200 ms", "300 ms", "500 ms"),
                .help    = "Time between focus steps: longer = slower rack.",
            },
            {
                .name    = "Start delay",
                .priv    = &rf_delay,
                .max     = 10,
                .icon_type = IT_PERCENT_OFF,
                .help    = "Seconds between SET and the start of the rack (time to start",
                .help2   = "recording or step back). 0 = immediately.",
            },
            {
                .name   = "Go to A",
                .select = rf_go_a,
                .help   = "Moves the lens to point A quickly (to set up the start of a rack).",
            },
            {
                .name   = "Go to B",
                .select = rf_go_b,
                .help   = "Moves the lens to point B quickly.",
            },
            MENU_EOL
        },
    },
};

static void rf_init(void *unused)
{
    menu_add("Focus", rf_menu, COUNT(rf_menu));
}

INIT_FUNC("rackfocus_r", rf_init);
