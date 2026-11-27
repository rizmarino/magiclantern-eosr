// Minimal WAD reader (IWAD/PWAD directory, read lumps by name or index).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>

#include "wadfile.h"

typedef struct
{
    char name[9];
    uint32_t pos;
    uint32_t size;
} lump_t;

static FILE *wad_fp = NULL;
static lump_t *lumps = NULL;
static int num_lumps = 0;

static uint32_t rd32(const unsigned char *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

int WAD_Open(const char *path)
{
    unsigned char hdr[12];
    unsigned char *dir;
    uint32_t n, ofs, i;

    wad_fp = fopen(path, "rb");
    if (wad_fp == NULL)
    {
        perror(path);
        return 0;
    }
    if (fread(hdr, 1, 12, wad_fp) != 12
     || (memcmp(hdr, "IWAD", 4) != 0 && memcmp(hdr, "PWAD", 4) != 0))
    {
        fprintf(stderr, "%s: not a WAD file\n", path);
        return 0;
    }
    n = rd32(hdr + 4);
    ofs = rd32(hdr + 8);
    dir = malloc((size_t) n * 16);
    lumps = calloc(n, sizeof(lump_t));
    if (fseek(wad_fp, ofs, SEEK_SET) != 0
     || fread(dir, 16, n, wad_fp) != n)
    {
        fprintf(stderr, "%s: cannot read directory\n", path);
        return 0;
    }
    for (i = 0; i < n; ++i)
    {
        lumps[i].pos = rd32(dir + 16 * i);
        lumps[i].size = rd32(dir + 16 * i + 4);
        memcpy(lumps[i].name, dir + 16 * i + 8, 8);
        lumps[i].name[8] = '\0';
    }
    free(dir);
    num_lumps = (int) n;
    return 1;
}

int WAD_NumLumps(void) { return num_lumps; }
const char *WAD_LumpName(int i) { return lumps[i].name; }

void *WAD_ReadLumpNum(int i, int *len)
{
    void *buf;
    buf = malloc(lumps[i].size ? lumps[i].size : 1);
    if (fseek(wad_fp, lumps[i].pos, SEEK_SET) != 0
     || fread(buf, 1, lumps[i].size, wad_fp) != lumps[i].size)
    {
        free(buf);
        return NULL;
    }
    *len = (int) lumps[i].size;
    return buf;
}

void *WAD_ReadLump(const char *name, int *len)
{
    int i;
    for (i = num_lumps - 1; i >= 0; --i)
    {
        if (strncasecmp(lumps[i].name, name, 8) == 0)
        {
            return WAD_ReadLumpNum(i, len);
        }
    }
    return NULL;
}
