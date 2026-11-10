/* dm_libc.c - the libc subset the Doom engine needs, for the ML module and the host harness alike.
 *
 * Why not ML's / Canon's: Canon's vsnprintf drops %u, %c, '-', '%%' and length modifiers; ML's strdup and malloc
 * use ML's allocator table (many small allocations, freed with our free); FIO_ReadFile wants uncacheable buffers.
 * Everything the engine allocates lives in one arena (small blocks) or a few big blocks (zone), all freed at quit.
 * The platform supplies dm_sys_* and dm_io_* (dm_sys.h). */

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "dm_force.h"
#include "dm_sys.h"
#include <sys/types.h>
#include <sys/stat.h>

/* ------------------------------------------------------------------ memory */

/* Arena: bump allocator with a size header (realloc needs it). free() only gives back the top block;
 * Doom allocates almost everything outside the zone once at startup. */
static uint8_t *arena;
static size_t arena_used, arena_hw;

#define BIG_MAX 24
static void  *big_ptr[BIG_MAX];
static size_t big_len[BIG_MAX];

typedef struct { uint32_t size; uint32_t magic; } blk_hdr;
#define BLK_MAGIC 0xD0D0B10Cu
#define BIG_MAGIC 0xB16B10C5u

static int arena_init(void)
{
    if (arena) return 1;
    arena = dm_sys_alloc(DM_ARENA_SIZE);
    arena_used = 0;
    return arena != NULL;
}

void *dm_malloc(size_t n)
{
    if (n == 0) n = 1;
    if (n >= DM_BIG_MIN)
    {
        for (int i = 0; i < BIG_MAX; i++)
        {
            if (!big_ptr[i])
            {
                uint8_t *p = dm_sys_alloc(n + 16);
                if (!p) return NULL;
                big_ptr[i] = p;
                big_len[i] = n;
                blk_hdr *h = (blk_hdr *)(p + 8);
                h->size = n;
                h->magic = BIG_MAGIC;
                return p + 16;
            }
        }
        return NULL;
    }
    if (!arena_init()) return NULL;
    size_t need = (sizeof(blk_hdr) + n + 7) & ~(size_t)7;
    if (arena_used + need > DM_ARENA_SIZE) return NULL;
    blk_hdr *h = (blk_hdr *)(arena + arena_used);
    h->size = n;
    h->magic = BLK_MAGIC;
    arena_used += need;
    if (arena_used > arena_hw) arena_hw = arena_used;
    return h + 1;
}

static blk_hdr *hdr_of(void *p) { return ((blk_hdr *)p) - 1; }

void dm_free(void *p)
{
    if (!p) return;
    blk_hdr *h = hdr_of(p);
    if (h->magic == BIG_MAGIC)
    {
        for (int i = 0; i < BIG_MAX; i++)
        {
            if (big_ptr[i] && (uint8_t *)big_ptr[i] + 16 == (uint8_t *)p)
            {
                h->magic = 0;
                dm_sys_free(big_ptr[i]);
                big_ptr[i] = NULL;
                big_len[i] = 0;
                return;
            }
        }
        return;
    }
    if (h->magic != BLK_MAGIC) return;      /* not ours / double free: ignore */
    size_t need = (sizeof(blk_hdr) + h->size + 7) & ~(size_t)7;
    h->magic = 0;
    if ((uint8_t *)h + need == arena + arena_used) arena_used -= need;   /* top block: reclaim */
}

void *dm_calloc(size_t n, size_t m)
{
    size_t t = n * m;
    void *p = dm_malloc(t);
    if (p) memset(p, 0, t);
    return p;
}

void *dm_realloc(void *p, size_t n)
{
    if (!p) return dm_malloc(n);
    if (n == 0) { dm_free(p); return NULL; }
    blk_hdr *h = hdr_of(p);
    size_t old = h->size;
    if (h->magic == BLK_MAGIC)
    {
        /* grow in place if it's the top block */
        size_t old_need = (sizeof(blk_hdr) + old + 7) & ~(size_t)7;
        size_t new_need = (sizeof(blk_hdr) + n + 7) & ~(size_t)7;
        if ((uint8_t *)h + old_need == arena + arena_used && n < DM_BIG_MIN &&
            arena_used - old_need + new_need <= DM_ARENA_SIZE)
        {
            arena_used = arena_used - old_need + new_need;
            if (arena_used > arena_hw) arena_hw = arena_used;
            h->size = n;
            return p;
        }
        if (n <= old) { h->size = n; return p; }
    }
    else if (h->magic != BIG_MAGIC) return NULL;
    void *q = dm_malloc(n);
    if (!q) return NULL;
    memcpy(q, p, old < n ? old : n);
    dm_free(p);
    return q;
}

void dm_libc_release_all(void)
{
    for (int i = 0; i < BIG_MAX; i++)
    {
        if (big_ptr[i]) { dm_sys_free(big_ptr[i]); big_ptr[i] = NULL; big_len[i] = 0; }
    }
    if (arena) { dm_sys_free(arena); arena = NULL; }
    arena_used = 0;
}

size_t dm_arena_high_water(void) { return arena_hw; }
size_t dm_big_bytes(void) { size_t t = 0; for (int i = 0; i < BIG_MAX; i++) t += big_len[i]; return t; }
int    dm_big_count(void) { int c = 0; for (int i = 0; i < BIG_MAX; i++) c += big_ptr[i] != NULL; return c; }

/* ------------------------------------------------------------------ strings */

char *dm_strdup(const char *s)
{
    size_t n = strlen(s) + 1;
    char *d = dm_malloc(n);
    if (d) memcpy(d, s, n);
    return d;
}

int dm_strncasecmp(const char *a, const char *b, size_t n)
{
    for (; n; n--, a++, b++)
    {
        int ca = dm_tolower((unsigned char)*a), cb = dm_tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        if (!ca) return 0;
    }
    return 0;
}

int dm_strcasecmp(const char *a, const char *b) { return dm_strncasecmp(a, b, (size_t)-1); }

char *dm_strchr(const char *s, int c)
{
    for (;; s++)
    {
        if (*s == (char)c) return (char *)s;
        if (!*s) return NULL;
    }
}

char *dm_strrchr(const char *s, int c)
{
    const char *r = NULL;
    for (;; s++)
    {
        if (*s == (char)c) r = s;
        if (!*s) return (char *)r;
    }
}

char *dm_strstr(const char *h, const char *n)
{
    size_t ln = strlen(n);
    if (!ln) return (char *)h;
    for (; *h; h++) if (*h == *n && !strncmp(h, n, ln)) return (char *)h;
    return NULL;
}

long dm_strtol(const char *s, char **end, int base)
{
    const char *p = s;
    while (dm_isspace((unsigned char)*p)) p++;
    int neg = 0;
    if (*p == '-' || *p == '+') neg = (*p++ == '-');
    if ((base == 0 || base == 16) && p[0] == '0' && (p[1] == 'x' || p[1] == 'X') && dm_isxdigit((unsigned char)p[2]))
    { p += 2; base = 16; }
    else if (base == 0 && p[0] == '0') base = 8;
    else if (base == 0) base = 10;
    unsigned long v = 0;
    const char *start = p;
    for (;; p++)
    {
        int c = (unsigned char)*p, d;
        if (dm_isdigit(c)) d = c - '0';
        else if (dm_isalpha(c)) d = dm_tolower(c) - 'a' + 10;
        else break;
        if (d >= base) break;
        v = v * base + d;
    }
    if (end) *end = (char *)(p == start ? s : p);
    return neg ? -(long)v : (long)v;
}

int dm_atoi(const char *s) { return (int)dm_strtol(s, NULL, 10); }

double dm_atof(const char *s)
{
    while (dm_isspace((unsigned char)*s)) s++;
    int neg = 0;
    if (*s == '-' || *s == '+') neg = (*s++ == '-');
    double v = 0, scale = 1;
    while (dm_isdigit((unsigned char)*s)) v = v * 10 + (*s++ - '0');
    if (*s == '.') { s++; while (dm_isdigit((unsigned char)*s)) { v = v * 10 + (*s++ - '0'); scale *= 10; } }
    v /= scale;
    if (*s == 'e' || *s == 'E')
    {
        int e = dm_atoi(s + 1);
        while (e > 0) { v *= 10; e--; }
        while (e < 0) { v /= 10; e++; }
    }
    return neg ? -v : v;
}

char *dm_getenv(const char *name) { (void)name; return NULL; }
int   dm_system(const char *cmd)  { (void)cmd; return -1; }
int   dm_mkdir(const char *path, mode_t mode) { (void)mode; return dm_io_mkdir(path); }
int   dm_access(const char *path, int mode)
{
    (void)mode;
    void *h = dm_io_open(path, 0);
    if (!h) return -1;
    dm_io_close(h);
    return 0;
}

static int dm_errno_val;
int *__errno(void) { return &dm_errno_val; }

/* ------------------------------------------------------------------ printf engine */

typedef struct { char *buf; size_t cap, len; } outbuf;

static void ob_putc(outbuf *o, char c)
{
    if (o->len + 1 < o->cap) o->buf[o->len] = c;
    o->len++;
}

static void ob_pad(outbuf *o, char c, int n) { while (n-- > 0) ob_putc(o, c); }

static void fmt_int(outbuf *o, unsigned long long v, int neg, int base, int upper,
                    int width, int prec, int left, int zero, char sign, int alt)
{
    char tmp[24];
    int n = 0;
    const char *dig = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    if (v == 0 && prec != 0) tmp[n++] = '0';
    while (v) { tmp[n++] = dig[v % base]; v /= base; }
    int nd = n < prec ? prec : n;
    const char *pre = "";
    if (neg) pre = "-";
    else if (sign) pre = sign == '+' ? "+" : " ";
    if (alt && base == 16 && n && !(n == 1 && tmp[0] == '0')) pre = upper ? "0X" : "0x";
    if (alt && base == 8 && nd <= n && !(n == 1 && tmp[0] == '0')) nd = n + 1;
    int lp = (int)strlen(pre);
    int total = lp + nd;
    if (prec >= 0) zero = 0;
    if (!left && !zero) ob_pad(o, ' ', width - total);
    for (int i = 0; i < lp; i++) ob_putc(o, pre[i]);
    if (!left && zero) ob_pad(o, '0', width - total);
    ob_pad(o, '0', nd - n);
    while (n) ob_putc(o, tmp[--n]);
    if (left) ob_pad(o, ' ', width - total);
}

static void fmt_str(outbuf *o, const char *s, int width, int prec, int left)
{
    if (!s) s = "(null)";
    int n = 0;
    while (s[n] && (prec < 0 || n < prec)) n++;
    if (!left) ob_pad(o, ' ', width - n);
    for (int i = 0; i < n; i++) ob_putc(o, s[i]);
    if (left) ob_pad(o, ' ', width - n);
}

static void fmt_float(outbuf *o, double v, int width, int prec, int left, int zero, char sign)
{
    char tmp[48];
    int n = 0;
    if (prec < 0) prec = 6;
    if (prec > 9) prec = 9;
    int neg = v < 0;
    if (neg) v = -v;
    double r = 0.5;
    for (int i = 0; i < prec; i++) r /= 10;
    v += r;
    if (v > 2e9) v = 2e9;                  /* Doom never prints bigger; keeps to __aeabi_d2iz (ML exports it) */
    unsigned int ip = (unsigned int)(int)v;
    double fp = v - (double)(int)ip;
    char *t = tmp;
    if (neg) *t++ = '-'; else if (sign) *t++ = sign;
    char d[24]; int nd = 0;
    do { d[nd++] = '0' + ip % 10; ip /= 10; } while (ip && nd < 20);
    while (nd) *t++ = d[--nd];
    if (prec)
    {
        *t++ = '.';
        for (int i = 0; i < prec; i++) { fp *= 10; int dd = (int)fp; *t++ = '0' + dd; fp -= dd; }
    }
    *t = 0;
    n = (int)(t - tmp);
    if (!left) ob_pad(o, zero ? '0' : ' ', width - n);
    for (int i = 0; i < n; i++) ob_putc(o, tmp[i]);
    if (left) ob_pad(o, ' ', width - n);
}

int dm_vsnprintf(char *buf, size_t cap, const char *fmt, va_list ap)
{
    outbuf o = { buf, cap, 0 };
    for (; *fmt; fmt++)
    {
        if (*fmt != '%') { ob_putc(&o, *fmt); continue; }
        fmt++;
        int left = 0, zero = 0, alt = 0; char sign = 0;
        for (;; fmt++)
        {
            if (*fmt == '-') left = 1;
            else if (*fmt == '0') zero = 1;
            else if (*fmt == '+') sign = '+';
            else if (*fmt == ' ') { if (!sign) sign = ' '; }
            else if (*fmt == '#') alt = 1;
            else break;
        }
        int width = 0, prec = -1;
        if (*fmt == '*') { width = va_arg(ap, int); if (width < 0) { left = 1; width = -width; } fmt++; }
        else while (dm_isdigit((unsigned char)*fmt)) width = width * 10 + (*fmt++ - '0');
        if (*fmt == '.')
        {
            fmt++; prec = 0;
            if (*fmt == '*') { prec = va_arg(ap, int); fmt++; }
            else while (dm_isdigit((unsigned char)*fmt)) prec = prec * 10 + (*fmt++ - '0');
        }
        int lng = 0;   /* 0 int, 1 long, 2 long long, -1 short, -2 char, 3 size_t */
        for (;; fmt++)
        {
            if (*fmt == 'l') lng = lng == 1 ? 2 : 1;
            else if (*fmt == 'h') lng = lng == -1 ? -2 : -1;
            else if (*fmt == 'z' || *fmt == 't') lng = 3;
            else if (*fmt == 'j' || *fmt == 'q') lng = 2;
            else break;
        }
        char c = *fmt;
        if (!c) break;
        switch (c)
        {
            case 'd': case 'i':
            {
                long long v;
                if (lng == 2) v = va_arg(ap, long long);
                else if (lng == 1) v = va_arg(ap, long);
                else if (lng == 3) v = (long long)va_arg(ap, size_t);
                else v = va_arg(ap, int);
                if (lng == -1) v = (short)v; else if (lng == -2) v = (signed char)v;
                unsigned long long u = v < 0 ? (unsigned long long)(-(v + 1)) + 1 : (unsigned long long)v;
                fmt_int(&o, u, v < 0, 10, 0, width, prec, left, zero, sign, 0);
                break;
            }
            case 'u': case 'x': case 'X': case 'o':
            {
                unsigned long long v;
                if (lng == 2) v = va_arg(ap, unsigned long long);
                else if (lng == 1) v = va_arg(ap, unsigned long);
                else if (lng == 3) v = va_arg(ap, size_t);
                else v = va_arg(ap, unsigned int);
                if (lng == -1) v = (unsigned short)v; else if (lng == -2) v = (unsigned char)v;
                int base = c == 'u' ? 10 : c == 'o' ? 8 : 16;
                fmt_int(&o, v, 0, base, c == 'X', width, prec, left, zero, 0, alt);
                break;
            }
            case 'p':
                fmt_int(&o, (uintptr_t)va_arg(ap, void *), 0, 16, 0, width, prec, left, zero, 0, 1);
                break;
            case 'c':
            {
                char ch = (char)va_arg(ap, int);
                if (!left) ob_pad(&o, ' ', width - 1);
                ob_putc(&o, ch);
                if (left) ob_pad(&o, ' ', width - 1);
                break;
            }
            case 's':
                fmt_str(&o, va_arg(ap, const char *), width, prec, left);
                break;
            case 'f': case 'F': case 'g': case 'G': case 'e': case 'E':
                fmt_float(&o, va_arg(ap, double), width, prec, left, zero, sign);
                break;
            case '%':
                ob_putc(&o, '%');
                break;
            default:   /* unknown: print it verbatim */
                ob_putc(&o, '%');
                ob_putc(&o, c);
                break;
        }
    }
    if (cap) buf[o.len < cap ? o.len : cap - 1] = 0;
    return (int)o.len;
}

int dm_snprintf(char *buf, size_t n, const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int r = dm_vsnprintf(buf, n, fmt, ap);
    va_end(ap);
    return r;
}

int dm_sprintf(char *buf, const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int r = dm_vsnprintf(buf, 4096, fmt, ap);   /* callers use small fixed buffers */
    va_end(ap);
    return r;
}

/* ------------------------------------------------------------------ sscanf (the subset Doom uses) */

int dm_sscanf(const char *s, const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int count = 0;
    while (*fmt)
    {
        if (dm_isspace((unsigned char)*fmt))
        {
            while (dm_isspace((unsigned char)*s)) s++;
            fmt++;
            continue;
        }
        if (*fmt != '%')
        {
            if (*s != *fmt) break;
            s++; fmt++;
            continue;
        }
        fmt++;
        if (*fmt == '%') { if (*s != '%') break; s++; fmt++; continue; }
        int width = 0;
        while (dm_isdigit((unsigned char)*fmt)) width = width * 10 + (*fmt++ - '0');
        if (*fmt == 'l' || *fmt == 'h') fmt++;
        char c = *fmt++;
        if (c == 's')
        {
            while (dm_isspace((unsigned char)*s)) s++;
            if (!*s) break;
            char *d = va_arg(ap, char *);
            int n = 0;
            while (*s && !dm_isspace((unsigned char)*s) && (!width || n < width)) d[n++] = *s++;
            d[n] = 0;
            count++;
            continue;
        }
        int base = c == 'd' || c == 'u' ? 10 : c == 'x' || c == 'X' ? 16 : c == 'o' ? 8 : c == 'i' ? 0 : -1;
        if (base < 0) break;
        while (dm_isspace((unsigned char)*s)) s++;
        char *end;
        long v = dm_strtol(s, &end, base);
        if (end == s) break;
        s = end;
        *va_arg(ap, int *) = (int)v;
        count++;
    }
    va_end(ap);
    return count;
}

/* ------------------------------------------------------------------ FILE */

struct dm_file
{
    void *h;
    int console;        /* stdout/stderr */
    int eof, err;
    long pos;
};

static struct dm_file f_out = { NULL, 1, 0, 0, 0 }, f_err = { NULL, 2, 0, 0, 0 };
FILE *dm_stdout = &f_out, *dm_stderr = &f_err;

FILE *dm_fopen(const char *path, const char *mode)
{
    int m = mode[0] == 'w' ? 1 : mode[0] == 'a' ? 2 : 0;
    void *h = dm_io_open(path, m);
    if (!h) return NULL;
    FILE *f = dm_malloc(sizeof(*f));
    if (!f) { dm_io_close(h); return NULL; }
    f->h = h; f->console = 0; f->eof = 0; f->err = 0;
    f->pos = m == 2 ? dm_io_size(h) : 0;
    return f;
}

int dm_fclose(FILE *f)
{
    if (!f || f->console) return 0;
    dm_io_close(f->h);
    dm_free(f);
    return 0;
}

size_t dm_fread(void *ptr, size_t size, size_t n, FILE *f)
{
    if (!f || f->console || !size) return 0;
    int r = dm_io_read(f->h, ptr, size * n);
    if (r < 0) { f->err = 1; return 0; }
    f->pos += r;
    if ((size_t)r < size * n) f->eof = 1;
    return (size_t)r / size;
}

static int console_write(const char *s, size_t n)
{
    char tmp[257];
    size_t done = 0;
    while (done < n)
    {
        size_t k = n - done > 256 ? 256 : n - done;
        memcpy(tmp, s + done, k);
        tmp[k] = 0;
        dm_sys_log(tmp);
        done += k;
    }
    return (int)n;
}

size_t dm_fwrite(const void *ptr, size_t size, size_t n, FILE *f)
{
    if (!f || !size) return 0;
    if (f->console) return console_write(ptr, size * n) / size;
    int r = dm_io_write(f->h, ptr, size * n);
    if (r < 0) { f->err = 1; return 0; }
    f->pos += r;
    return (size_t)r / size;
}

int dm_fseek(FILE *f, long off, int whence)
{
    if (!f || f->console) return -1;
    long p = whence == SEEK_SET ? off : whence == SEEK_CUR ? f->pos + off : dm_io_size(f->h) + off;
    if (p < 0 || dm_io_seek(f->h, p)) return -1;
    f->pos = p;
    f->eof = 0;
    return 0;
}

long dm_ftell(FILE *f) { return f && !f->console ? f->pos : -1; }
int  dm_fflush(FILE *f) { (void)f; return 0; }
int  dm_feof(FILE *f) { return f ? f->eof : 1; }
int  dm_fputs(const char *s, FILE *f) { return dm_fwrite(s, 1, strlen(s), f) == strlen(s) ? 0 : EOF; }
int  dm_fputc(int c, FILE *f) { char ch = (char)c; return dm_fwrite(&ch, 1, 1, f) == 1 ? c : EOF; }

char *dm_fgets(char *s, int n, FILE *f)
{
    int i = 0;
    while (i < n - 1)
    {
        char c;
        if (dm_fread(&c, 1, 1, f) != 1) break;
        s[i++] = c;
        if (c == '\n') break;
    }
    if (!i) return NULL;
    s[i] = 0;
    return s;
}

int dm_vfprintf(FILE *f, const char *fmt, va_list ap)
{
    char tmp[512];
    va_list ap2;
    va_copy(ap2, ap);
    int n = dm_vsnprintf(tmp, sizeof(tmp), fmt, ap);
    if (n < (int)sizeof(tmp)) { dm_fwrite(tmp, 1, n, f); va_end(ap2); return n; }
    char *big = dm_malloc(n + 1);
    if (big) { dm_vsnprintf(big, n + 1, fmt, ap2); dm_fwrite(big, 1, n, f); dm_free(big); }
    else dm_fwrite(tmp, 1, sizeof(tmp) - 1, f);
    va_end(ap2);
    return n;
}

int dm_fprintf(FILE *f, const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int r = dm_vfprintf(f, fmt, ap);
    va_end(ap);
    return r;
}

int dm_vprintf(const char *fmt, va_list ap) { return dm_vfprintf(dm_stdout, fmt, ap); }

int dm_printf(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int r = dm_vfprintf(dm_stdout, fmt, ap);
    va_end(ap);
    return r;
}

int dm_puts(const char *s) { dm_fputs(s, dm_stdout); dm_fputc('\n', dm_stdout); return 0; }
int dm_putchar(int c) { return dm_fputc(c, dm_stdout); }
int dm_remove(const char *path) { return dm_io_remove(path); }
int dm_rename(const char *a, const char *b) { return dm_io_rename(a, b); }

/* ------------------------------------------------------------------ exit() */

/* Doom quits with exit(). Catch it with gcc's builtin setjmp (no libc needed) and return to the caller. */
static void *exit_jb[5];
static volatile int exit_armed, exit_code;

void dm_exit(int code)
{
    exit_code = code;
    if (exit_armed) __builtin_longjmp(exit_jb, 1);
    for (;;) { }   /* not armed: can't happen in practice */
}

int dm_run_catching_exit(int (*fn)(void))
{
    if (__builtin_setjmp(exit_jb) == 0)
    {
        exit_armed = 1;
        int r = fn();
        exit_armed = 0;
        return r;
    }
    exit_armed = 0;
    return exit_code;
}

/* ------------------------------------------------------------------ self-test (runs on host and camera) */

int dm_libc_selftest(char *rep, size_t cap)
{
    struct { const char *want; char got[64]; } t[12];
    int k = 0, fail = 0;
#define T(want_, ...) do { t[k].want = want_; dm_snprintf(t[k].got, sizeof(t[k].got), __VA_ARGS__); k++; } while (0)
    T("[  42|-7   |002a|%]", "[%4d|%-5d|%04x|%%]", 42, -7, 42);
    T("u=4000000000 c=Z", "u=%u c=%c", 4000000000u, 'Z');
    T("[E1M1    ]x", "[%-8s]%s", "E1M1", "x");
    T("ll=-123456789012 lx=ffffffff", "ll=%lld lx=%lx", -123456789012LL, 0xffffffffUL);
    T("zu=123 p=0x10", "zu=%zu p=%p", (size_t)123, (void *)0x10);
    T("f=3.14 g=-0.500000", "f=%.2f g=%f", 3.14159, -0.5);
    T("[ab][  +5][0x1f]", "[%.2s][%+4d][%#x]", "abcdef", 5, 31);
    T("STCFN033", "STCFN%.3d", 33);
#undef T
    size_t used = 0;
    rep[0] = 0;
    for (int i = 0; i < k; i++)
    {
        if (strcmp(t[i].want, t[i].got))
        {
            fail++;
            used += dm_snprintf(rep + used, used < cap ? cap - used : 0, "FAIL printf want '%s' got '%s'\n", t[i].want, t[i].got);
        }
    }
    int a = -1, b = -1, c = -1;
    int n1 = dm_sscanf("0x1F", "%x", &a);
    int n2 = dm_sscanf(" 0x2a", " 0x%x", &b);
    int n3 = dm_sscanf("  -17", " %d", &c);
    if (n1 != 1 || a != 0x1f || n2 != 1 || b != 42 || n3 != 1 || c != -17)
    {
        fail++;
        used += dm_snprintf(rep + used, used < cap ? cap - used : 0, "FAIL sscanf %d/%x %d/%x %d/%d\n", n1, a, n2, b, n3, c);
    }
    if (dm_strncasecmp("DOOM1.WAD", "doom1.wad", 9) || !dm_strncasecmp("abc", "abd", 3) || dm_atoi(" 35") != 35 ||
        dm_atof("2.5") != 2.5 || strcmp(dm_strrchr("a/b/c", '/'), "/c") || !dm_strstr("hello", "ll"))
    {
        fail++;
        used += dm_snprintf(rep + used, used < cap ? cap - used : 0, "FAIL string helpers\n");
    }
    char *p = dm_malloc(100), *q = dm_realloc(p, 300);
    if (!p || !q) { fail++; used += dm_snprintf(rep + used, used < cap ? cap - used : 0, "FAIL arena alloc\n"); }
    dm_free(q);
    if (!fail) dm_snprintf(rep, cap, "selftest: all %d checks pass\n", k + 3);
    return fail;
}
