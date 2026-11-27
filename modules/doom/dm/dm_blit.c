/* dm_blit.c - Doom's 320x200 8-bit frame -> ARGB8888 window of ML's RGBA layer (EOS R: 960x540, visible 720x480
 * at x=120, y=30). Portable: the host harness runs the same code into a fake layer and saves it as an image. */

#include <stdint.h>
#include <string.h>
#include "dm_blit.h"

static uint32_t lut[256];
static uint32_t rowbuf[DM_SRC_W * 3];      /* one expanded row, cached memory */

void dm_blit_set_palette(const uint8_t *rgb_triplets_or_null, const void *colors_bgra)
{
    /* colors_bgra: doomgeneric's struct color { b, g, r, a } bitfields, i.e. little-endian 0xAARRGGBB with a=0 */
    const uint32_t *c = colors_bgra;
    for (int i = 0; i < 256; i++) lut[i] = 0xFF000000u | (c[i] & 0x00FFFFFFu);
    (void)rgb_triplets_or_null;
}

void dm_blit_mode_geometry(int mode, int *w, int *h)
{
    switch (mode)
    {
        case DM_SCALE_2X:     *w = 640; *h = 400; break;
        case DM_SCALE_1X:     *w = 320; *h = 200; break;
        case DM_SCALE_FILL: *w = 720; *h = 480; break;
        default:              *w = 640; *h = 480; break;   /* DM_SCALE_ASPECT: 2x wide, 2.4x tall = 4:3 */
    }
}

/* Fill the visible window with opaque black (the borders around the game picture). */
void dm_blit_clear(uint32_t *layer, int pitch, int win_x, int win_y, int win_w, int win_h)
{
    for (int y = 0; y < win_h; y++)
    {
        uint32_t *d = layer + (win_y + y) * pitch + win_x;
        for (int x = 0; x < win_w; x++) d[x] = 0xFF000000u;
    }
}

static void expand_row(const uint8_t *s, int xs)
{
    uint32_t *r = rowbuf;
    if (xs == 1)
        for (int x = 0; x < DM_SRC_W; x++) r[x] = lut[s[x]];
    else if (xs == 2)
        for (int x = 0; x < DM_SRC_W; x++) { uint32_t v = lut[s[x]]; r[0] = v; r[1] = v; r += 2; }
    else /* DM_SCALE_FILL: 720 wide, 9/4 */
        for (int x = 0; x < 720; x++) r[x] = lut[s[x * 4 / 9]];
}

/* Draw one frame centred in the window. Returns the number of layer words written. */
int dm_blit_frame(uint32_t *layer, int pitch, int win_x, int win_y, int win_w, int win_h,
                  const uint8_t *src, int mode)
{
    int ow, oh;
    dm_blit_mode_geometry(mode, &ow, &oh);
    int xs = mode == DM_SCALE_1X ? 1 : mode == DM_SCALE_FILL ? 3 : 2;
    int ox = win_x + (win_w - ow) / 2, oy = win_y + (win_h - oh) / 2;
    int last_sy = -1;
    for (int y = 0; y < oh; y++)
    {
        int sy = y * DM_SRC_H / oh;
        if (sy != last_sy) { expand_row(src + sy * DM_SRC_W, xs); last_sy = sy; }
        uint32_t *d = layer + (oy + y) * pitch + ox;
        memcpy(d, rowbuf, ow * 4);
    }
    return ow * oh;
}
