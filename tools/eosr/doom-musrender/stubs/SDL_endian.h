/* Stub SDL_endian.h: x86 is little endian, WAD data is little endian. */
#ifndef MUSRENDER_SDL_ENDIAN_H
#define MUSRENDER_SDL_ENDIAN_H
#include <stdint.h>
#define SDL_LIL_ENDIAN 1234
#define SDL_BIG_ENDIAN 4321
#define SDL_BYTEORDER SDL_LIL_ENDIAN
#define SDL_SwapLE16(x) ((uint16_t)(x))
#define SDL_SwapLE32(x) ((uint32_t)(x))
static inline uint16_t SDL_Swap16(uint16_t x) { return (uint16_t)((x << 8) | (x >> 8)); }
static inline uint32_t SDL_Swap32(uint32_t x) { return __builtin_bswap32(x); }
#define SDL_SwapBE16(x) SDL_Swap16(x)
#define SDL_SwapBE32(x) SDL_Swap32(x)
#endif
