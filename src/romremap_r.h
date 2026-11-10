/* EOS R shared ROM -> RAM remap (romremap_r.c) */
#ifndef ROMREMAP_R_H
#define ROMREMAP_R_H

#include <stdint.h>
#include <stddef.h>

#define RR_NPAGES 4

int romremap_init(void);                    /* 0, or a negative step number (sticky for this power-on) */
int romremap_on(void);
const char *romremap_stage(void);           /* last setup step, for error messages */
uint8_t *romremap_ram(uint32_t rom_addr);   /* RAM alias of a remapped ROM address, NULL if not remapped */
void romremap_flush(void *ram, size_t n);   /* after writing through romremap_ram() */

#endif
