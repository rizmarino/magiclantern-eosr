/* Doom-on-ML libc shim: stdio.
 * Replaces the toolchain's stdio.h for the engine, so every stdio call lands in dm_libc.c
 * (own printf engine: Canon's vsnprintf lacks %u, %c, '-', '%%' and length modifiers). */
#ifndef DM_STDIO_H
#define DM_STDIO_H

#include <stddef.h>
#include <stdarg.h>

typedef struct dm_file FILE;

extern FILE *dm_stdout, *dm_stderr;
#define stdout dm_stdout
#define stderr dm_stderr

#ifndef EOF
#define EOF (-1)
#endif
#ifndef SEEK_SET
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2
#endif

#define fopen      dm_fopen
#define fclose     dm_fclose
#define fread      dm_fread
#define fwrite     dm_fwrite
#define fseek      dm_fseek
#define ftell      dm_ftell
#define fflush     dm_fflush
#define fputs      dm_fputs
#define fputc      dm_fputc
#define putc       dm_fputc
#define fgets      dm_fgets
#define feof       dm_feof
#define printf     dm_printf
#define fprintf    dm_fprintf
#define vprintf    dm_vprintf
#define vfprintf   dm_vfprintf
#define puts       dm_puts
#define putchar    dm_putchar
#define sprintf    dm_sprintf
#define snprintf   dm_snprintf
#define vsnprintf  dm_vsnprintf
#define sscanf     dm_sscanf
#define remove     dm_remove
#define rename     dm_rename

FILE  *dm_fopen(const char *path, const char *mode);
int    dm_fclose(FILE *f);
size_t dm_fread(void *ptr, size_t size, size_t n, FILE *f);
size_t dm_fwrite(const void *ptr, size_t size, size_t n, FILE *f);
int    dm_fseek(FILE *f, long off, int whence);
long   dm_ftell(FILE *f);
int    dm_fflush(FILE *f);
int    dm_fputs(const char *s, FILE *f);
int    dm_fputc(int c, FILE *f);
char  *dm_fgets(char *s, int n, FILE *f);
int    dm_feof(FILE *f);
int    dm_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int    dm_fprintf(FILE *f, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
int    dm_vprintf(const char *fmt, va_list ap);
int    dm_vfprintf(FILE *f, const char *fmt, va_list ap);
int    dm_puts(const char *s);
int    dm_putchar(int c);
int    dm_sprintf(char *buf, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
int    dm_snprintf(char *buf, size_t n, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
int    dm_vsnprintf(char *buf, size_t n, const char *fmt, va_list ap);
int    dm_sscanf(const char *str, const char *fmt, ...);
int    dm_remove(const char *path);
int    dm_rename(const char *from, const char *to);

#endif
