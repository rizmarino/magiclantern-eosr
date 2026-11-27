#ifndef WADFILE_H
#define WADFILE_H

int WAD_Open(const char *path);
int WAD_NumLumps(void);
const char *WAD_LumpName(int i);   // NUL-terminated, uppercase as stored
void *WAD_ReadLumpNum(int i, int *len);
void *WAD_ReadLump(const char *name, int *len);  // case-insensitive, last match

#endif
