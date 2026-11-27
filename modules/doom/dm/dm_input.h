#ifndef DM_INPUT_H
#define DM_INPUT_H
#include <stdint.h>
extern volatile int dm_input_quit;
void dm_input_reset(void);
int  dm_input_raw(int param);                                         /* 1 = mapped, block from Canon */
int  dm_input_getkey(uint32_t now_ms, int *pressed, unsigned char *key);
#endif
