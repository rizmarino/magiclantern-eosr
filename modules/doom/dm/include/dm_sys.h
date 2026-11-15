/* Interface between dm_libc.c (portable) and the platform: doom_ml.c on the camera,
 * host/host_main.c on the PC. Plain C types only, so both sides can include it. */
#ifndef DM_SYS_H
#define DM_SYS_H

#include <stddef.h>
#include <stdint.h>

/* --- provided by the platform --- */
void *dm_sys_alloc(size_t n);          /* large blocks (arena, zone); NULL on failure */
void  dm_sys_free(void *p);
void  dm_sys_log(const char *s);       /* one chunk of console text (may lack '\n') */

/* files: handle is opaque, NULL = failure. mode: 0 read, 1 write (create/truncate), 2 append */
void *dm_io_open(const char *path, int mode);
int   dm_io_read(void *h, void *buf, size_t n);         /* bytes read, <0 error */
int   dm_io_write(void *h, const void *buf, size_t n);  /* bytes written, <0 error */
int   dm_io_seek(void *h, long pos);                    /* absolute; 0 ok */
long  dm_io_size(void *h);
void  dm_io_close(void *h);
int   dm_io_remove(const char *path);
int   dm_io_rename(const char *from, const char *to);
int   dm_io_mkdir(const char *path);

/* sound (optional; dm_sound.c mixes, the platform plays): 48 kHz s16 stereo interleaved, pulled in chunks.
 * dm_snd_hw_open starts an output that calls dm_snd_mix() for every chunk from its own audio context (0 = running);
 * after dm_snd_hw_close returns, dm_snd_mix is no longer called. Music: the platform calls dm_mus_service() regularly
 * from a context that may block on the card (never the audio one), and dm_mus_release() at the end. */
int   dm_snd_hw_open(void);
void  dm_snd_hw_close(void);
void  dm_snd_mix(int16_t *out, int frames);           /* provided by dm_sound.c */
void  dm_mus_service(void);                           /* provided by dm_sound.c */
void  dm_mus_release(void);                           /* provided by dm_sound.c */
void  dm_sys_sleep(int ms);
int   dm_snd_stats(char *buf, size_t n);              /* provided by dm_sound.c: one log line */

/* --- provided by dm_libc.c --- */
/* Run fn() with exit() caught; returns the exit code (or fn's return). */
int  dm_run_catching_exit(int (*fn)(void));
void dm_exit(int code) __attribute__((noreturn));
/* Free everything dm_libc allocated (arena + big blocks). Only after the engine is done. */
void dm_libc_release_all(void);
/* Stats for the log. */
size_t dm_arena_high_water(void);
size_t dm_big_bytes(void);
int    dm_big_count(void);
int    dm_libc_selftest(char *report, size_t n);   /* 0 = all pass */

#define DM_ARENA_SIZE   (256 * 1024)    /* small allocations; host demo1 + play peaked at ~50 KB */
#define DM_BIG_MIN      (128 * 1024)    /* allocations from here up get their own block */

#endif
