# Stills: silent burst and LV silent pictures (Shoot tab)

## Silent burst
Canon's **Silent shutter + High-speed continuous** shoots at 5 fps. ML raises the rate.

1. Canon: Silent shutter **Enable** (Shooting menu), Drive **High-speed continuous**.
2. ML: Shoot > **Silent burst** > Burst rate: **6.0 / 6.5 / 7.0** fps (Canon 5.0 = off). Saved.
3. Hold the shutter as usual.

| Entry | Choices | Notes |
|---|---|---|
| Burst rate | Canon 5.0, 5.5 ... 10.0 | ~7 fps is the camera's real maximum; 7.5-10 run at ~7 |
| Fast frames | All, 10, 15, 20 | frames at the faster rate per burst, then Canon 5.0 until you let go |

Camera-verified: 5.5-7.0 fps measured from Canon's EXIF timestamps; RAW bursts held 10-17 s past a full buffer at
5.0 and 7.0 fps with no error. Limitation: **C-RAW** held 10 s+ past a full buffer at a raised rate can freeze the
camera; use RAW, or Fast frames 15/20 for long C-RAW holds.

## LV silent pictures
Raw DNG stills taken from the live-view feed with no shutter at all, in movie mode: a short burst, or
**pre-capture** (frames from just before you pressed).

1. Canon: movie mode, Movie rec size **4K** (4256x2214 raw frame; FHD gives 2304x1290).
2. ML: Shoot > **LV silent pictures**: Burst or Pre-capture.
3. Half-press to arm (~2 s), full press: frames are kept while held (up to ~12). Pictures go to `S<day><hhmm>/` on
   the card root as DNG.

| Entry | Choices | Notes |
|---|---|---|
| LV silent pictures | OFF / Burst / Pre-capture | |
| Bit depth | 10-bit / 12-bit | 10-bit for 4K (12-bit is as slow as the sensor readout and tears 4K frames); 12-bit fine in FHD |
| Frames before press | 3, 5, 7, 9 | pre-capture: frames kept from before the full press (~30 fps) |
| Arm | Half-press / Always | Always: armed whenever movie live view is up (ML overlays pause, raw REC blocked) |

Limits: 13 frames at most (memory), movie-mode live view only, the exposure is the live view's (shutter speed of the
movie settings).
