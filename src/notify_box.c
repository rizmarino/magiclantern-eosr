/** \file
 * Dialog test code
 * Based on http://code.google.com/p/400plus/source/browse/trunk/menu_developer.c
 */
#include "dryos.h"
#include "bmp.h"
#include "tasks.h"
#include "debug.h"
#include "menu.h"
#include "property.h"
#include "config.h"
#include "gui.h"
#include "lens.h"

static struct semaphore * notify_box_show_sem = 0;
static struct semaphore * notify_box_main_sem = 0;

static int notify_box_timeout = 0;
static int notify_box_stop_request = 0;
static int notify_box_dirty = 0;
static char notify_box_msg[100];
static char notify_box_msg_tmp[100];

#if defined(CONFIG_R)
/* riz v4.26: on DIGIC 8 _redraw_do() redraws Canon's LiveViewApp dialog, which doesn't touch ML's own overlay layer
 * (it only clrscr()s when that dialog is missing), so in live view a finished NotifyBox stayed on screen until a
 * view change cleared the layer (riz 2026-11-05). Erase exactly the area the last message used, then push. */
static char notify_box_drawn[100];
static void notify_box_erase(void)
{
    if (!notify_box_drawn[0]) return;
    int w = bmp_string_width(FONT_LARGE, notify_box_drawn), h = fontspec_height(FONT_LARGE);
    bmp_fill(COLOR_EMPTY, 50, 50, MIN(w + 8, 720 - 50), h + 4);
    notify_box_drawn[0] = 0;
    extern uint32_t ml_refresh_display_needed;
    ml_refresh_display_needed = 1;
}
#endif

static void NotifyBox_task(void* priv)
{
    TASK_LOOP
    {
        // wait until some other task asks for a notification
        int err = take_semaphore(notify_box_show_sem, 500);
        if (err)
            continue;
        
        if (!notify_box_timeout) 
            continue;
        
        // show notification for a while, then redraw to erase it
        notify_box_stop_request = 0;
        notify_box_dirty = 0;
        for ( ; notify_box_timeout > 0 ; notify_box_timeout -= 50)
        {
            if (notify_box_dirty)
            {
                /* clear old message */
#if defined(CONFIG_R)
                notify_box_erase();
#endif
                redraw();
                notify_box_dirty = 0;
            }
            bmp_printf(FONT_LARGE,  50,  50, notify_box_msg);
#if defined(CONFIG_R)
            memcpy(notify_box_drawn, notify_box_msg, sizeof(notify_box_drawn));
#endif
            msleep(50);
            if (notify_box_stop_request)
                break;
        }
        notify_box_timeout = 0;
#if defined(CONFIG_R)
        notify_box_erase();
#endif
        redraw();
    }
}

TASK_CREATE( "notifybox_task", NotifyBox_task, 0, 0x1b, 0x1000 );

void NotifyBoxHide()
{
    notify_box_stop_request = 1;
}

void NotifyBox(int timeout, char* fmt, ...) 
{
    // make sure this is thread safe
    take_semaphore(notify_box_main_sem, 0);
    
    va_list ap;
    va_start( ap, fmt );
    vsnprintf( notify_box_msg_tmp, sizeof(notify_box_msg_tmp)-1, fmt, ap );
    va_end( ap );
    
    if (notify_box_timeout && streq(notify_box_msg_tmp, notify_box_msg)) 
    {
        // same message: do not redraw, just re-set the timeout
        notify_box_timeout = MAX(timeout, 100);
        goto end;
    }

    // new message
    qprint("[NotifyBox] "); qprint(notify_box_msg_tmp); qprint("\n");
    memcpy(notify_box_msg, notify_box_msg_tmp, sizeof(notify_box_msg));
    notify_box_msg[sizeof(notify_box_msg)-1] = '\0';
    notify_box_timeout = MAX(timeout, 100);
    if (notify_box_timeout)
        notify_box_dirty = 1; // ask for a redraw, message changed

    give_semaphore(notify_box_show_sem); // request displaying the notification box

end:
    give_semaphore(notify_box_main_sem); // done, other call can be made now
}

static void dlg_init()
{
    notify_box_show_sem = create_named_semaphore("nbox_show_sem", SEM_CREATE_LOCKED);
    notify_box_main_sem = create_named_semaphore("nbox_done_sem", SEM_CREATE_UNLOCKED);
}

INIT_FUNC(__FILE__, dlg_init);
