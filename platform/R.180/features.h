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

// riz v4.0 "quick wins" (NOT camera-tested yet; see kit README):
// Expo menu ISO / Shutter / Aperture (uses the R's verified byte-1 / hi-byte property writes; the toggles
// are also what the adv_int module needs) and Expo presets (SET or INFO swaps two ISO/Tv/Av sets).
#define FEATURE_EXPO_ISO
#define FEATURE_EXPO_SHUTTER
#define FEATURE_EXPO_APERTURE
#define FEATURE_EXPO_PRESET
// riz v4.36 (batch D): Expo > White Balance - Kelvin + Auto adjust Kelvin (WB mode/Kelvin props verified on the R
// with Prop watch v4.35 and whitelisted; WB shift / custom gains hidden, see shoot.c)
#define FEATURE_WHITE_BALANCE
// riz v4.39/4.40 (batch D): Focus End Point + Rack Focus drive the lens with PROP_LV_LENS_DRIVE_REMOTE (R firmware
// uses the same encoding: FUN_e034ff36; the lens moves, camera 2026-11-08). Follow Focus is off: in LV with the menu
// closed the R delivers the arrow keys to ML only as event 0x85 (Canon moves the AF point), never as LEFT/RIGHT.
// v4.40 camera 2026-11-08: Focus End Point can't be set as built. menu_enable_lv_transparent_mode() is a no-op on
// the R (menu.c, the menu sits on Canon's 0x68 dialog), so PLAY never hides the menu and the hidden-menu
// dial/arrow focus overrides in handle_rack_focus_menu_overrides() never run. The top dial changed shutter speed.
// riz v4.41: dropped on the R for now (riz 2026-11-08). lens_focus() stays open-loop for Lua lens.focus.
//#define FEATURE_RACK_FOCUS
// Shoot > Motion detect and Focus > Trap focus (LV, MF, hold half-press): on the R both read a sampled
// grid of the LV luma (r_spot_luma in zebra.c), fire through take_a_pic (remote-release path).
#define FEATURE_MOTION_DETECT
#define FEATURE_TRAP_FOCUS
// Prefs > Warning for bad settings (read-only: mode, WB, AF/MF; quality/ALO hidden on the R, encodings unknown)
#define FEATURE_WARNINGS_FOR_BAD_SETTINGS
// Debug: LV image buffer sizes and the EFIC temperature property (raw value, already registered)
#define FEATURE_SHOW_IMAGE_BUFFERS_INFO
#define FEATURE_SHOW_CMOS_TEMPERATURE

// riz v4.7 (NOT camera-tested): Display > Clear overlays (bmp_off/bmp_on mute ML's RGBA layer, bmp.c r_bmp_mute)
// and Display > Color scheme (applied in bmp.c r_build_lut; the stock LCD_Palette pokes are compiled out on the R)
#define FEATURE_CLEAR_OVERLAYS
#define FEATURE_COLOR_SCHEME

// keep upstream dev's crash log + ROM autobackup: crash logs are essential
// for first hardware tests, ROM dumps enable qemu-eos emulation later
#define CONFIG_CRASH_LOG
#define CONFIG_AUTOBACKUP_ROM
