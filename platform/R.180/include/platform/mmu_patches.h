#ifndef __PLATFORM_MMU_PATCHES_H__
#define __PLATFORM_MMU_PATCHES_H__

#include "patch.h"

/* EOS R 1.8.0: no compile-time patches. The silent burst fps patch (src/silenttest_r.c) is applied at runtime
 * from the menu with apply_patches() and undone with unpatch_memory(). */

#if CONFIG_FW_VERSION == 180 // ensure hard-coded patch addresses are not broken by a FW upgrade

struct patch early_data_patches[] =
{
};

struct patch normal_data_patches[] =
{
};

struct function_hook_patch early_code_patches[] =
{
};

struct function_hook_patch normal_code_patches[] =
{
};

#endif // R FW_VERSION 180

#endif // __PLATFORM_MMU_PATCHES_H__
