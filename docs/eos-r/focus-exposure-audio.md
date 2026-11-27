# Focus, exposure and audio additions

## Rack focus (Focus tab)
Moves focus smoothly between two stored points, in photo or movie live view, also while recording.

1. Lens on AF. Canon: Continuous AF **Disable** (photo) / Movie Servo AF **Disable** (movie); with servo on, Rack now is
   greyed out.
2. Focus on the first subject with **AF-ON**, open ML, Focus > Rack focus > **Point A** > SET. Same for **Point B**.
3. **Rack now** (SET): the menu closes and the lens racks to the other point after the Start delay.

| Entry | Choices |
|---|---|
| Speed | Slow (step 1), Medium (step 2), Fast (step 3) |
| Step interval | 80, 120, 200, 300, 500 ms |
| Start delay | seconds before the rack starts (time to press REC) |
| Go to A / Go to B | quick move to a stored point |

Don't press AF-ON while it runs; half-press or opening the ML menu stops it. ML steps the lens until the distance the
lens reports crosses the stored one (open loop on the lens side). Camera-verified with the RF 16mm F2.8 STM: every
rack landed on the stored distance. In movie mode Slow can be very slow on some lenses. Log: `ML/LOGS/RACKFOC.TXT`.
Follow focus and focus end point (stock ML) are not available on the R.

## Auto ETTR (movie) (Expo tab)
Sets ISO from the raw data so the highlights sit just below clipping. Movie mode, manual ISO. Never changes ISO while
recording; shutter and aperture are untouched.

| Entry | Choices |
|---|---|
| Auto ETTR (movie) | OFF / Half-press / Continuous |
| Run now | one run when the menu closes |
| Highlight ignore | 0 - 2 % of the image allowed to clip |
| Headroom | 0.3 / 0.5 / 1.0 EV below clipping |
| Max ISO | 800 - 12800 |

## Kelvin white balance (Expo tab)
White balance in Kelvin steps, photo and movie, plus **Auto adjust Kelvin**: adjusts the Kelvin value once for the
current live-view scene.
Camera-verified from Canon's recorded EXIF white balance.

## RAW histogram and RAW zebras (Overlay tab, movie mode)
Histogram and zebras computed from the live raw feed instead of the processed picture: they show real sensor
clipping, independent of picture style.

## Beeps (Audio tab)
ML's beeps (intervalometer, trap focus, Lua `beep`) play through Canon's own sound-effect player. Photo mode only,
never while recording. Beep sounds 1A (short) and 14 (long) by default; 1B also works.

## Audio in raw clips
See [raw-video.md](raw-video.md): 48 kHz stereo in the MLV, sync within a frame (camera-checked with clap tests).
