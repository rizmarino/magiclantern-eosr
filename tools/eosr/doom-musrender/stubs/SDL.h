/* Stub SDL.h for the offline renderer.
 * Single-threaded: mutexes are no-ops. SDL_CondWait() is only used by
 * OPL_Delay() (opl.c) to wait for a timer callback; offline we satisfy the
 * wait by advancing emulated time until the next queued callback fires. */
#ifndef MUSRENDER_SDL_H
#define MUSRENDER_SDL_H
#include <stdint.h>
#include "SDL_endian.h"
typedef struct SDL_mutex SDL_mutex;
typedef struct SDL_cond SDL_cond;
typedef uint8_t Uint8;
typedef uint16_t Uint16;
typedef uint32_t Uint32;
SDL_mutex *SDL_CreateMutex(void);
void SDL_DestroyMutex(SDL_mutex *m);
int SDL_LockMutex(SDL_mutex *m);
int SDL_UnlockMutex(SDL_mutex *m);
SDL_cond *SDL_CreateCond(void);
void SDL_DestroyCond(SDL_cond *c);
int SDL_CondSignal(SDL_cond *c);
int SDL_CondWait(SDL_cond *c, SDL_mutex *m);
#endif
