#ifndef DM_BLIT_H
#define DM_BLIT_H
#include <stdint.h>

#define DM_SRC_W 320
#define DM_SRC_H 200

enum { DM_SCALE_ASPECT = 0, DM_SCALE_2X = 1, DM_SCALE_1X = 2, DM_SCALE_FILL = 3 };

void dm_blit_set_palette(const uint8_t *unused, const void *colors_bgra);
void dm_blit_mode_geometry(int mode, int *w, int *h);
void dm_blit_clear(uint32_t *layer, int pitch, int win_x, int win_y, int win_w, int win_h);
int  dm_blit_frame(uint32_t *layer, int pitch, int win_x, int win_y, int win_w, int win_h,
                   const uint8_t *src, int mode);
#endif
