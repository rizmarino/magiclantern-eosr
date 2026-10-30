#define FEATURE_VRAM_RGBA

#define CONFIG_COMPOSITOR_XCM
#define CONFIG_COMPOSITOR_DEDICATED_LAYER
#define CONFIG_COMPOSITOR_XCM_V1

/* Remap ROM pages to RAM (ML ROM patching, cpu0 only), from PR #281. Used by the silent burst fps patch. */
/* v1.8: ML's patch_mmu is no longer used (silenttest_r.c has its own 24 KB, 4 KB-page remap); see lv-survey.md */
// #define CONFIG_SGI_HANDLERS
// #define CONFIG_MMU_REMAP
/* riz: set up the remap only when first needed, in ~148 KB from Canon's AllocateMemory pool instead of ML's bss.
 * A 544 KB ML (bss incl. generic_mmu_space) left Canon too little user_mem: silent bursts locked up / Err 70 once the
 * buffer filled, with or without the remap (camera-tested 2026-10-30). */
// #define CONFIG_MMU_REMAP_LAZY

// Don't Click Me menu looks to be intended as a place
// for devs to put custom code in debug.c run_test(),
// and allowing triggering from a menu context.
#define FEATURE_DONT_CLICK_ME

#define FEATURE_SHOW_SHUTTER_COUNT

// working but incomplete, some allocators don't report
// anything yet as they're faked / not yet found
#define FEATURE_SHOW_FREE_MEMORY

#define CONFIG_TSKMON
#define FEATURE_SHOW_TASKS
#define FEATURE_SHOW_CPU_USAGE
#define FEATURE_SHOW_GUI_EVENTS

#define CONFIG_ADDITIONAL_VERSION
#define FEATURE_SCREENSHOT

// enable global draw
#define FEATURE_GLOBAL_DRAW
#define FEATURE_CROPMARKS

// EXPERIMENTAL (riz/eosr-experimental): LiveView analysis overlays.
// These only READ the LV YUV422 buffer (YUV422_LV_BUFFER_*, DispVram struct,
// see consts.h) and draw into ML's own bitmap layer, so the failure mode is a
// wrong / garbled overlay or a crash, not a settings change.
// UNTESTED ON HARDWARE.
#define FEATURE_ZEBRA
#undef  FEATURE_ZEBRA_FAST          // D45 palette trick, not valid on RGBA compositor
#define FEATURE_FOCUS_PEAK
#define FEATURE_FALSE_COLOR
#define FEATURE_HISTOGRAM
#define FEATURE_WAVEFORM
#define FEATURE_SPOTMETER
#define FEATURE_VECTORSCOPE         // also required: zebra.c histogram path references vectorscope_draw

// enable for testing gui structure changes
#define CONFIG_RESTORE_AFTER_FORMAT

// Image capture now works: lens_take_picture uses the CameraConductor
// remote-release path, which finalizes the job cleanly and tolerates AF-lock
// failure (verified on hardware: easy subject captures, hard subject fails
// gracefully, no crash). So the old "not working" gate is lifted.
// #define CONFIG_IMAGE_CAPTURE_NOT_WORKING

#define FEATURE_PICSTYLE
#define CONFIG_PROP_REQUEST_CHANGE

// capture-driven features (ride on take_a_pic, now working)
#define FEATURE_INTERVALOMETER
#define FEATURE_HDR_BRACKETING

// keep upstream dev's crash log + ROM autobackup: crash logs are essential
// for first hardware tests, ROM dumps enable qemu-eos emulation later
#define CONFIG_CRASH_LOG
#define CONFIG_AUTOBACKUP_ROM
