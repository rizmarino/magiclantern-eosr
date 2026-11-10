# Doom for Magic Lantern (EOS R). The engine and its libc shim are built with their own flags (shim headers first,
# no ML headers), pre-linked into one object whose symbols are made local except the __module_* tables, so the
# hundreds of engine globals (players, lines, sectors...) can't clash with ML core or other modules.

MODULE_NAME = doom
TOP_DIR = ../..

ENGINE_SRCS = dummy am_map doomdef doomstat dstrings d_event d_items d_iwad d_loop d_main d_mode d_net f_finale \
        f_wipe g_game hu_lib hu_stuff info i_cdmus i_endoom i_joystick i_scale i_sound i_system i_timer memio m_argv \
        m_bbox m_cheat m_config m_controls m_fixed m_menu m_misc m_random p_ceilng p_doors p_enemy p_floor p_inter \
        p_lights p_map p_maputl p_mobj p_plats p_pspr p_saveg p_setup p_sight p_spec p_switch p_telept p_tick \
        p_user r_bsp r_data r_draw r_main r_plane r_segs r_sky r_things sha1 sounds statdump st_lib st_stuff \
        s_sound tables v_video wi_stuff w_checksum w_file w_main w_wad z_zone w_file_stdc i_input i_video doomgeneric

BUILD_DIR ?= build
ENGINE_OBJS = $(addprefix $(BUILD_DIR)/engine/,$(addsuffix .o,$(ENGINE_SRCS))) \
              $(BUILD_DIR)/engine/dm_libc.o $(BUILD_DIR)/engine/dm_blit.o $(BUILD_DIR)/engine/dm_input.o

MODULE_OBJS = $(BUILD_DIR)/doom_all.o

include $(TOP_DIR)/modules/Makefile
.DEFAULT_GOAL := $(BUILD_DIR)/module_complete

ARM_CC  = $(ARM_BINPATH)/arm-none-eabi-gcc
ARM_LD  = $(ARM_BINPATH)/arm-none-eabi-ld
ARM_OBJCOPY = $(ARM_BINPATH)/arm-none-eabi-objcopy

ENGINE_CFLAGS = -Os -march=armv5te -mlong-calls -fno-common -fno-strict-aliasing -std=gnu99 -w \
        -fstack-usage -Idm/include -include dm/include/dm_force.h -Iengine \
        -DCMAP256 -DDOOMGENERIC_RESX=320 -DDOOMGENERIC_RESY=200

$(BUILD_DIR)/engine/%.o: engine/%.c dm/include/dm_force.h | $(BUILD_DIR)/engine
	$(ARM_CC) $(ENGINE_CFLAGS) -c -o $@ $<
$(BUILD_DIR)/engine/dm_libc.o: dm/dm_libc.c $(wildcard dm/include/*.h) | $(BUILD_DIR)/engine
	$(ARM_CC) $(ENGINE_CFLAGS) -Wall -Wextra -Wno-unused-parameter -Werror -c -o $@ $<
$(BUILD_DIR)/engine/dm_blit.o: dm/dm_blit.c dm/dm_blit.h | $(BUILD_DIR)/engine
	$(ARM_CC) $(ENGINE_CFLAGS) -Wall -Werror -c -o $@ $<
$(BUILD_DIR)/engine:
	@mkdir -p $@

# doom_ml.o is built by ML's generic rule (ML headers, ML warning flags)
$(BUILD_DIR)/doom_all.o: $(ENGINE_OBJS) $(BUILD_DIR)/doom_ml.o
	$(ARM_LD) -r -o $@.tmp $(filter %.o,$^)
	$(ARM_OBJCOPY) -w --keep-global-symbol='__module_*' $@.tmp $@
	@rm -f $@.tmp

$(BUILD_DIR)/doom_ml.o: $(BUILD_DIR)/module_strings.h
$(BUILD_DIR)/engine/dm_input.o: dm/dm_input.c dm/dm_input.h | $(BUILD_DIR)/engine
	$(ARM_CC) $(ENGINE_CFLAGS) -Wall -Werror -c -o $@ $<
