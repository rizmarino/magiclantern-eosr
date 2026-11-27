/* Minimal config.h for the offline Chocolate Doom OPL music renderer. */
#ifndef MUSRENDER_CONFIG_H
#define MUSRENDER_CONFIG_H
#define PACKAGE_NAME "Chocolate Doom (musrender)"
#define PACKAGE_STRING "Chocolate Doom (musrender)"
#define PACKAGE_TARNAME "chocolate-doom"
#define PROGRAM_PREFIX "chocolate-"
#define HAVE_DECL_STRCASECMP 1
#define HAVE_DECL_STRNCASECMP 1
/* No HAVE_IOPERM: no hardware OPL drivers. SDL2_mixer is "present" so that
 * opl.c lists opl_sdl_driver, which this tool provides as an offline driver. */
#endif
