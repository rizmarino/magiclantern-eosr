# Raw video (Movie tab > RAW video (R))

ML records raw sensor data as MLV files instead of Canon's H.264. Two ways:

- **Uncompressed** (Compression: OFF): 10/12/14-bit raw, any MLV tool reads it directly. Limited by card speed to
  about 1920-2560 px wide.
- **Lossless** (Compression: Lossless): each frame goes through **Canon's own CR3 lossless encoder (CRX)**, the same
  one the camera uses for CR3 stills. Files are ~25-45 % of uncompressed, which makes 4K raw possible. Standard MLV
  tools can't decode CRX frames yet: convert them on the PC first (below).

## Quick start
1. Canon: movie mode, **Movie rec size 4K 23.98p** (or FHD), Movie digital IS off.
2. ML (Trash): Movie > **RAW video (R)**: ON. Inside (PLAY on the entry):
   Resolution **Full frame**, Bit depth **10-bit**, Compression **Lossless**, Lossless source **Direct**.
3. Close the ML menu, press **REC**. Press REC again to stop. Clips go to the card root as `M<day>-<hhmm>.MLV`
   (with `.M00`, `.M01` parts for long clips).
4. On the PC: `python3 tools/eosr/lossless/crx2mlv.py M27-1522.MLV`, then open the `_RAW.MLV` in MLV App.

## Menu: Movie > RAW video (R)
| Entry | Choices | Notes |
|---|---|---|
| RAW video (R) | OFF / ON | ON: REC records MLV instead of Canon video |
| Resolution | 1920x1080, 1920x960, 1920x800, **Full frame**, 2560x1080, 2560x1440, 2880x1216, 3840x2160 | 2560 / 2880 / 3840 need Canon movie size 4K (1:1 pixels). Full frame: FHD 2256x1268, 4K 4104x2168 |
| 4K frame height | Normal, 16:9, Tall, Wide, Wide 16:9 | records more of the sensor: [4k-frame-height.md](4k-frame-height.md) |
| Frame guide, Screen view | | [4k-frame-height.md](4k-frame-height.md) |
| Bit depth | 14 / 12 / 10-bit | 10-bit is the usual choice for 4K |
| Compression | OFF / Lossless | Lossless needed for 4K Full frame and 3840x2160 |
| Lossless source | Copy / **Direct** / Direct+verify | Direct: Canon's encoder reads the frame buffer itself; needed for 29.97p. +verify stores 1 uncompressed frame per second (diagnostic) |
| Lossless copy-out | HPCopy / HPCopy+EDMAC | two copy engines (faster); 12-bit 4K 29.97p always uses HPCopy |
| Pre-record | OFF / 0.5 / 1 / 2 s | 1st REC arms (PRE-REC), 2nd REC starts the clip that far back, 3rd stops. Trash cancels |
| Audio | OFF / ON | camera mic or input into the MLV, 48 kHz stereo; needs Canon Sound recording Auto or Manual |
| Audio meters | OFF / ON | L/R bars while recording |
| Crop marker | OFF / ON | frame around the recorded area (needs Global Draw ON) |
| Data rate | (info) | rate the current setting needs, and the card speed measured in the last clip |

## Camera-tested modes
Tested on one body, PNY X-Pro 90 (V90) and Lexar 1667x cards. "Clean" = all frames recorded, PC check found no torn
or stale rows and the frames decode bit-exact.

**Lossless**
| Canon mode | ML resolution | Depth | Frame rate | Result |
|---|---|---|---|---|
| 4K | Full frame 4104x2168 | 10-bit | 23.98p | clean (99/99, 244/245) |
| 4K | 3840x2160 | 10-bit | 23.98p, 29.97p | clean (29.97p: 3 drops in 483 frames) |
| 4K | 3840x2160 | 12-bit | 23.98p | clean (373/374) |
| FHD | Full frame 2256x1268 | 12-bit, 14-bit | 29.97p | clean |
| 4K | 4K frame heights (16:9, Tall, Wide, Wide 16:9) | 10-bit | 23.98p | clean, see [4k-frame-height.md](4k-frame-height.md) |

**Uncompressed**
| Canon mode | ML resolution | Depth | Frame rate | Result |
|---|---|---|---|---|
| FHD | 1920x1080 | 14-bit | 29.97p | clean, 3 min take (Lexar) |
| FHD | Full frame 2256x1268 | 10-bit 29.97p, 12-bit 23.98p | | clean, 20-40 s |
| 4K | 2560x1440 | 10-bit | 23.98p | clean, 40 s (needs 105 MiB/s, ~99 % of the Lexar) |
| 4K | 2560x1080 | 10-bit 29.97p, 12-bit 23.98p | | clean |

Many clips lose 1-3 frames in the first ~0.1 s; the clip's on-screen message then says FRAMES LOST and the MLV keeps
the gap (frame numbers skip).

## Card speed
The camera writes at most ~170-180 MiB/s to the fastest cards (Canon's own limit). Measured in ML:

| Card | Uncompressed recorder | Lossless recorder |
|---|---|---|
| PNY X-Pro 90 128 GB (V90) | ~150 MiB/s | ~170-180 MiB/s while writing |
| Lexar 1667x 64 GB | 110-112 MiB/s | |

What the modes need (uncompressed MiB/s = width x height x bits / 8 x fps / 2^20):

| Mode | 10-bit | 12-bit | 14-bit |
|---|---|---|---|
| FHD Full frame 2304x1290, 29.97p | 106 | 127 | 149 |
| 2560x1440, 23.98p | 105 | 126 | 147 |
| 4K Full frame 4256x2214, 23.98p | 269 (lossless: ~75-105 measured) | 323 | 377 |

Lossless needs depend on the scene: 2.6-3.9 bits per pixel measured at 10-bit (25-38 % of raw), so a detailed or
noisy scene needs more. The Data rate menu line shows the last clip's figures.

## PC side: converting lossless clips
`tools/eosr/lossless/crx2mlv.py` turns a lossless MLV into a normal uncompressed MLV (same bit depth, audio and
metadata copied), which MLV App, mlv_dump and other MLV tools open.

    python3 crx2mlv.py M27-1522.MLV                 # -> M27-1522_RAW.MLV next to it
    python3 crx2mlv.py CLIP.MLV OUT.MLV --check 3   # also re-reads 3 frames and verifies them

- Needs Python 3 with numpy, and `crxdec` next to the script: a small decoder built on LibRaw's CRX code
  (`make LIBRAW=/path/to/LibRaw` in that folder, see [building.md](building.md)). Linux tested; any OS that builds
  LibRaw should work.
- Uses all CPU cores; about 0.3-0.6 s per 4K frame on a 4-core PC. Output is ~3x the size of the lossless clip.
- Verified: camera CRX frames -> converted MLV -> mlv_dump DNG -> LibRaw are bit-identical to the camera's own decode.
- `crx_verify_rawc.py` checks Direct+verify clips (the uncompressed test frames must match the compressed ones).

## Known limits
- **Lossless 12-bit 4K at 29.97p: not supported** (drops ~1 frame in 3). Use 10-bit for 4K 29.97p, or 23.98p for 12-bit.
  Every attempt to speed it up with a second memory transfer froze the camera in that mode.
- Lossless in photo-mode live view drops ~20 % of frames: use movie mode.
- Pre-record clips have no audio.
- FHD Full frame 10-bit 29.97p uncompressed is at the limit of most cards: use 23.98p or 1920x1080 for long clips.
- ML can't see Canon's own (H.264) recording; with RAW video ON, REC always records raw.
