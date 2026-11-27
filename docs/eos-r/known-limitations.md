# Known limitations and issues (EOS R)

Tested on one EOS R (firmware 1.8.0, build 7.3.9) with two cards. Expect differences on other bodies, lenses and
cards, and please report them with the card's `ML/LOGS/` and any `CRASHnn.LOG`.

## Limitations (by design or not solvable so far)
- **Lossless 12-bit 4K 29.97p** drops ~1 frame in 3. Use 10-bit at 29.97p, or 12-bit at 23.98p.
- **4K frame heights** (16:9, Tall, Wide): 23.98p only, lossless 10-bit, Global Draw OFF, an AF method with magnify
  (1-point AF), RAW video ON.
- **Wide 16:9 + Screen view Whole frame**: coloured band at the right edge of the live view (recordings are clean).
- **Wide** heights: the picture is not centred on the lens axis (extra columns on the right; needed for AF).
- Taller/wider frames stay at the 4K crop (~1.7x): no full-sensor-width 4K raw.
- Lossless in photo-mode live view drops ~20 % of frames: use movie mode.
- Pre-record clips have no audio. Many clips drop 1-3 frames in the first ~0.1 s (marked in the clip).
- C-RAW bursts held 10 s+ past a full buffer at a raised silent-burst rate can freeze the camera (RAW is fine).
- LV silent pictures: 13 frames max; 12-bit tears in 4K.
- No dual ISO (no per-line ISO control found), no FPS override, no follow focus / focus end point, no ML beeps in
  movie mode. Motion detect and trap focus use the frame centre only.
- With RAW video ON, REC always records raw; ML can't see Canon's own recording.
- Doom: one game per power-on; disable the module when not playing.
- Lossless clips need the PC converter (`tools/eosr/lossless/crx2mlv.py`) before MLV App / mlv_dump can read them.

## Known issues (rare, not fixed)
- Photo -> movie switch crashed once in ~26 tries (Global Draw, vectorscope and RAW zebras on): Canon assert
  `EvfState.c:1790`. Cause not found.
- The ML menu froze once for ~26 s after a SET in movie mode, then recovered (Canon `ImageController.c:981`).
- A 4K frame height switch occasionally refuses with "cpu1 check failed" and retries once by itself; after a refusal,
  set the height to Normal and back.
- Wide 16:9 stalled once after a Canon dialog had ended the taller frame; a power cycle cleared it.

## Memory
ML leaves Canon ~512 KB of its malloc heap free (measured on v5.18). Canon needs about 490 KB for a RAW burst that
fills the buffer: builds that add tens of KB to ML need that test again. Disabling unused modules frees memory.
