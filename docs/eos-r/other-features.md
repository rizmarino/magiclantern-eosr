# Other ML features on the EOS R

Standard Magic Lantern features, ported to the R and checked on the camera. They work as described in the general
Magic Lantern documentation (https://wiki.magiclantern.fm); R-specific notes below.

| Tab | Features |
|---|---|
| Expo | ISO, shutter, aperture, exposure presets, Kelvin WB (see [focus-exposure-audio.md](focus-exposure-audio.md)), Auto ETTR (movie) |
| Overlay | Global Draw, zebras, focus peaking, false colour, histogram, waveform, spotmeter, vectorscope, cropmarks, Clear overlays, colour scheme, RAW histogram / zebras (movie) |
| Shoot | Intervalometer, advanced bracketing (HDR / DOF), motion detect, silent burst, LV silent pictures |
| Focus | Trap focus (MF, hold half-press), rack focus, Lua `lens.focus` steps |
| Display | Info bars with auto-hide, rear screen and EVF |
| Scripts | Lua scripting (examples: calc, editor, pong, sokoban) |
| Modules | adv_int (advanced intervalometer), arkanoid, bench (card benchmark), file_man, lua, doom |
| Debug | Free memory, battery report, temperatures, image buffers, screenshot, task list |

R-specific notes:
- **Trash** opens/closes the ML menu (the R has no dedicated ML button). PLAY opens an entry's submenu.
- The ML menu can't be made transparent over live view on the R, so features that rely on "PLAY hides the menu, then
  arrows adjust" in stock ML work differently: set values in the menu, then close it.
- Motion detect and trap focus measure the frame centre only.
- No ML beeps in movie mode.
- Not available on the R: follow focus, focus end point, FPS override, bitrate control, dual ISO (no per-line ISO
  control found in the R's sensor registers), Canon H.264 crop modes.
