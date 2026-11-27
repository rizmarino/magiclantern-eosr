# Doom (optional module)

Doom runs on the EOS R as an ML module: full game, sound effects and music through the camera speaker, saves on the
card. Engine: doomgeneric (Chocolate Doom behaviour), ported to ML.

**No game data is included.** Bring your own WAD: the free **shareware DOOM1.WAD**, your copy of DOOM.WAD /
DOOM2.WAD, or the free FreeDoom WADs.

## Setup
1. Copy the WAD to `ML/DOOM/` on the card (`DOOM1.WAD`, `DOOM.WAD`, `DOOMU.WAD`, `DOOM2.WAD`, `FREEDOOM1.WAD` or
   `FREEDOOM2.WAD`; the first one found is used).
2. ML menu > **Modules** > doom: enable, restart the camera.
3. ML menu > **Games** > **Doom** > SET plays. PLAY on the entry opens its options.

Optional music: render it on a PC from your own WAD with `tools/eosr/doom-musrender` (Chocolate Doom's OPL music code)
and copy the WAV files to `ML/DOOM/MUSIC/` (~83 MB for DOOM1.WAD). Without them the game plays sound effects only.

## Controls
| Camera | Doom |
|---|---|
| Multi-controller / joystick | move, turn (menus: up/down) |
| SET or AF-ON | fire; SET also confirms in Doom's menus |
| PLAY or M-Fn | use (doors, switches) |
| Top dial | previous / next weapon |
| Touch bar swipe | strafe |
| INFO | automap |
| Trash | Doom's menu (save, load, options) |
| **MENU** | quit Doom at once |

Options: **Picture** (640x480 4:3, 640x400 2x, 320x200 1x, 720x480 fill), **Speaker volume** (1-10), **Display &
input test** (no WAD needed).

## Notes
- 25-34 fps in game on the camera.
- **One game per power-on** (the engine can't be reset): restart the camera to play again.
- Saves: `ML/DOOM/SAVES/`. Config: `ML/DOOM/default.cfg`, `extra.cfg`.
- Memory: the module takes ~0.6 MB of shoot memory while enabled and ~6 MB more while playing (freed on quit). **Disable the module
  when you're not playing**: it reduces the memory the raw recorder can use.
- Confirming a Doom menu item with SET also fires one shot when the game resumes.
- Camera-verified: play, save/load, effects and music (no gaps).
