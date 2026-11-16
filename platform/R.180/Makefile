MODEL = R
FW_VERSION = 180

TOP_DIR = ../..

AUTOEXEC_BASE = 0x800000
FIR_BASE = 0x800120

#CANON_FIR  = EOSR0180.FIR   # not good
#UPDATE_NAME_FIR = LOG_R180.FIR
FIRMWARE_ID = 0x00000424
MAIN_FIRMWARE_ADDR = 0xE0040000

# user_mem_start from cstart struct. Align to 0x10 up
RESTARTSTART = 0xb0080 # 0xb0078 in ROM

# Cortex A9, binaries loaded as Thumb
CFLAGS_PLATFORM = -mthumb -mlong-calls
PLATFORM_ARCH = armv7-a

ML_BOOT_OBJ = boot-d678.o
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/function_overrides.o $(BUILD_DIR)/test_features.o
# riz/eosr-dmlog: Debug -> LV log capture (Canon DbgMgr, no ROM patching)
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/lvlog_r.o $(BUILD_DIR)/rectest_r.o $(BUILD_DIR)/silenttest_r.o
# v4.80: lvjob_r.o (Debug > LV job dump, v3 research, finished) left out: Canon heap (v4.79 boot Err 70 at 476 KB free)
# v4.9: RAW histogram / RAW zebras in movie mode (LV raw tap)
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/rawlv_r.o
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/crxtest_r.o   # v4.17 lossless 4K step 2
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/romremap_r.o   # v4.51 shared ROM remap (silent burst fps patch)
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/rackfocus_r.o  # v4.56 Focus > Rack focus (stored points, lens-distance loop)
# v4.80: sensreg_r.o (Debug > Sensor regs log, v4.58-v4.60) left out for Canon heap; re-add for the wider-4K stage
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/tall4k_r.o     # v4.61 Debug > Taller 4K (test): first sensor-window write
# v4.55: dualiso_r.o (Shoot > Dual ISO (test)) left out: no per-line ISO control found on the R (parked 2026-11-11);
# the source stays for reference (skill references/dual-iso.md)
# v3.16: audprobe_r.o (Debug > Audio probe) and memsurvey_r.o (Debug > Memory survey) left out: finished test
# tools; ML footprint is Canon heap (silent-burst lock-up when the buffer clears)

include ../Makefile
.DEFAULT_GOAL := $(BUILD_DIR)/cam_complete

# v4.11: Lua scripts not suited to the R: api_test.lua (photos, 10 s bulb, LV pause), recdelay.lua (movie.start,
# no route on the R), copy2m.lua (writes raw property values; R byte-1 encodings unchecked). Runs after the copies.
zip_contents:: | $(BUILD_DIR)
	rm -f $(ZIP_DIR)/ML/scripts/api_test.lua $(ZIP_DIR)/ML/scripts/recdelay.lua $(ZIP_DIR)/ML/scripts/copy2m.lua
