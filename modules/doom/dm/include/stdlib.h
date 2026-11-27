/* Doom-on-ML libc shim: stdlib (allocator is dm_libc's arena, not ML's malloc). */
#ifndef DM_STDLIB_H
#define DM_STDLIB_H

#include <stddef.h>

#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
#endif

#define malloc   dm_malloc
#define calloc   dm_calloc
#define realloc  dm_realloc
#define free     dm_free
#define atoi     dm_atoi
#define atol     dm_atoi
#define atof     dm_atof
#define strtol   dm_strtol
#define exit     dm_exit
#define abort()  dm_exit(-2)
#define getenv   dm_getenv
#define system   dm_system
#define abs      dm_abs

void  *dm_malloc(size_t n);
void  *dm_calloc(size_t n, size_t m);
void  *dm_realloc(void *p, size_t n);
void   dm_free(void *p);
int    dm_atoi(const char *s);
double dm_atof(const char *s);
long   dm_strtol(const char *s, char **end, int base);
void   dm_exit(int code) __attribute__((noreturn));
char  *dm_getenv(const char *name);
int    dm_system(const char *cmd);
static inline int dm_abs(int x) { return x < 0 ? -x : x; }

#endif
