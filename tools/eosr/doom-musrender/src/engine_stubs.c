//
// Minimal stand-ins for the Doom engine services that i_oplmusic.c,
// mus2mid.c, midifile.c and memio.c reference (W_, Z_, M_, I_ helpers).
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <strings.h>
#include <unistd.h>

#include "doomtype.h"
#include "i_sound.h"
#include "i_system.h"
#include "m_misc.h"
#include "w_wad.h"
#include "z_zone.h"

#include "wadfile.h"

// Nuked OPL3 native rate: OPL3_Reset() then performs no resampling.
int snd_samplerate = 49716;

// --- W_ --------------------------------------------------------------------

void *W_CacheLumpName(const char *name, int tag)
{
    void *data;
    int len;
    (void) tag;

    data = WAD_ReadLump(name, &len);
    if (data == NULL)
    {
        I_Error("W_CacheLumpName: %s not found!", name);
    }
    return data;
}

void W_ReleaseLumpName(const char *name)
{
    // The GENMIDI copy is leaked deliberately (tiny, and freed at exit);
    // i_oplmusic.c keeps pointers into it until shutdown.
    (void) name;
}

// --- Z_ --------------------------------------------------------------------

void *Z_Malloc(int size, int tag, void *user)
{
    void *p;
    (void) tag; (void) user;
    p = malloc(size > 0 ? (size_t) size : 1);
    if (p == NULL)
    {
        I_Error("Z_Malloc: failed on allocation of %i bytes", size);
    }
    return p;
}

void Z_Free(void *ptr)
{
    free(ptr);
}

// --- I_ --------------------------------------------------------------------

void I_Error(const char *error, ...)
{
    va_list args;
    va_start(args, error);
    fprintf(stderr, "I_Error: ");
    vfprintf(stderr, error, args);
    fprintf(stderr, "\n");
    va_end(args);
    exit(1);
}

void *I_Realloc(void *ptr, size_t size)
{
    void *p = realloc(ptr, size);
    if (size != 0 && p == NULL)
    {
        I_Error("I_Realloc: failed on reallocation of %zu bytes", size);
    }
    return p;
}

// --- M_ --------------------------------------------------------------------

FILE *M_fopen(const char *filename, const char *mode)
{
    return fopen(filename, mode);
}

int M_remove(const char *path)
{
    return remove(path);
}

boolean M_WriteFile(const char *name, const void *source, int length)
{
    FILE *f = fopen(name, "wb");
    size_t n;
    if (f == NULL)
    {
        return false;
    }
    n = fwrite(source, 1, (size_t) length, f);
    fclose(f);
    return n == (size_t) length;
}

char *M_TempFile(const char *s)
{
    const char *dir = getenv("TMPDIR");
    char *result;
    size_t len;

    if (dir == NULL || *dir == '\0')
    {
        dir = "/tmp";
    }
    len = strlen(dir) + strlen(s) + 40;
    result = malloc(len);
    snprintf(result, len, "%s/musrender-%ld-%s", dir, (long) getpid(), s);
    return result;
}

int M_vsnprintf(char *buf, size_t buf_len, const char *s, va_list args)
{
    int result;
    if (buf_len < 1)
    {
        return 0;
    }
    result = vsnprintf(buf, buf_len, s, args);
    if (result < 0 || (size_t) result >= buf_len)
    {
        buf[buf_len - 1] = '\0';
        result = (int) buf_len - 1;
    }
    return result;
}

int M_snprintf(char *buf, size_t buf_len, const char *s, ...)
{
    va_list args;
    int result;
    va_start(args, s);
    result = M_vsnprintf(buf, buf_len, s, args);
    va_end(args);
    return result;
}

boolean M_StringCopy(char *dest, const char *src, size_t dest_size)
{
    size_t len;
    if (dest_size >= 1)
    {
        dest[dest_size - 1] = '\0';
        strncpy(dest, src, dest_size - 1);
    }
    else
    {
        return false;
    }
    len = strlen(dest);
    return src[len] == '\0';
}

boolean M_StringConcat(char *dest, const char *src, size_t dest_size)
{
    size_t offset = strlen(dest);
    if (offset > dest_size)
    {
        offset = dest_size;
    }
    return M_StringCopy(dest + offset, src, dest_size - offset);
}

// --- DEH_ (no dehacked: identity) ------------------------------------------

#include "deh_str.h"
#undef DEH_String
const char *DEH_String(const char *s)
{
    return s;
}
