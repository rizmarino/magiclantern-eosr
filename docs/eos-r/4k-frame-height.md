# 4K frame height, whole-frame view and Frame guide

Canon's 4K mode reads a fixed 4256x2214 window of the sensor (about 1.7x crop) and records 3840x2160 of it. ML can
make the sensor read **more rows** (16:9, Tall) or **more rows and columns** (Wide) at the same crop factor, and
record all of it as lossless raw.

Movie > RAW video (R) > **4K frame height**:

| Choice | Recorded picture | Aspect | vs. Canon 4K (3840x2160) |
|---|---|---|---|
| Normal | 4104x2168 (Resolution: Full frame) | 1.89:1 | Canon's own readout |
| **16:9** | **4104x2312** | 16:9 | +7 % width, +7 % height |
| **Tall** | **4104x2648** | 1.55:1 (about 3:2) | +7 % width, +23 % height |
| Wide | 4320x2168 | 1.99:1 | +12.5 % width |
| **Wide 16:9** | **4320x2430** | 16:9 | +12.5 % width, +12.5 % height |

All camera-tested in lossless 10-bit 23.98p (clean clips, 0 torn frames, bit-exact decode on the PC).

## Requirements
- Movie mode, Movie rec size **4K 23.98p** (the only frame rate tested).
- **RAW video (R) ON**, Compression **Lossless**, Bit depth **10-bit**. With RAW video OFF the setting does nothing
  and Canon's video stays normal 4K.
- **Overlay > Global Draw OFF**.
- Canon **AF method: 1-point AF** (or another method with a magnify icon on screen). The switch goes through Canon's
  magnified view; Face+Tracking has no magnify, and the menu then says "paused: Canon refused zoom".
- Movie Servo AF **Disable** and Movie digital IS **Disable** recommended.

## How it switches
About 2 s after you close the ML menu in idle 4K movie live view, ML zooms in and out once by itself and shows
**"4K 16:9 4104x2312"** (or the chosen size). Record as usual. Changing the height, switching RAW video OFF, MENU or PLAY, a Canon
screen opening, or leaving 4K movie live view goes back to normal 4K first. Recording is blocked for the second the
switch takes ("4K height: switching, wait").

If the camera ever freezes during a switch: battery out, and the next start turns the 4K frame height off by itself
("4K height OFF: froze at last switch").

## Screen view (what you see)
Canon's live view keeps its own 3840x2160 window, so by default (**Centre 1:1**) the screen shows the middle of the
recording and the extra area is recorded but not visible.

**Screen view: Whole frame** makes Canon's display scale a 4096x2304 cut of the picture to the screen instead:

| Height | Whole frame shows |
|---|---|
| 16:9 | all of it (4096 of 4104 columns) |
| Tall | full width, 87 % of the height |
| Wide 16:9 | 95 % per axis |
| Wide (4320x2168) | not available (too few rows), stays Centre 1:1 |

The switch message ends with ", whole view" when it worked, or ", centre view" plus a reason on the menu line.
AF works on the scaled picture: camera-checked, AF measures under the box in whole view.

## Frame guide
Movie > RAW video (R) > **Frame guide** (OFF / bottom left / bottom right / top left / top right): a small map while a
taller/wider height is on. Yellow = recorded picture, white = what the screen shows, with the recorded pixels beyond
each screen edge (e.g. "L132 R348" in Wide 16:9: the extra width is all on the right, see below).

## Notes per mode
- **Wide** keeps Canon's 4K window start and adds all 216 extra columns on the right of the picture, because Canon's
  dual-pixel AF assumes that start (a centred window broke AF). So Wide's picture is not centred on the lens axis:
  ~108 px of the frame centre shift to the left. AF works normally (camera-verified).
- **16:9 / Tall / Wide 16:9** keep the extra rows centred.
- The picture size stays a 1.7x crop of the full sensor: these modes add area at the 4K crop, they don't widen the
  field of view to full frame.

## Limitations
- 23.98p only; 29.97p is beyond the sensor's readout time at these sizes.
- **Wide 16:9 + Whole frame**: a coloured band (~3 % wide) at the right edge of the **screen**. Recordings are clean.
  Cosmetic; cause not found.
- Not tested with Canon 10-bit / C-Log modes (ML refuses whole view when Canon 10-bit is on).
