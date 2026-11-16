/* EOS R shared ROM -> RAM remap (romremap_r.c) */
#ifndef ROMREMAP_R_H
#define ROMREMAP_R_H

#include <stdint.h>
#include <stddef.h>

#define RR_NPAGES 5

int romremap_init(void);                    /* 0, or a negative step number (sticky for this power-on) */
int romremap_on(void);
const char *romremap_stage(void);           /* last setup step, for error messages */
uint8_t *romremap_ram(uint32_t rom_addr);   /* RAM alias of a remapped ROM address, NULL if not remapped */
void romremap_flush(void *ram, size_t n);   /* after writing through romremap_ram() */
/* v4.64: also switch cpu1 (Canon RPC); reads probe_addr on cpu1 afterwards. 0 = cpu1 switched */
int romremap_cpu1(uint32_t probe_addr, int *cpu, uint32_t *val, int *calls);
int romremap_on_cpu1(void);

#endif
