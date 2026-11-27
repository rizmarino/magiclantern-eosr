# musrender

Offline renderer: Doom MUS lumps -> WAV using Chocolate Doom's own OPL music
code (i_oplmusic.c DMX 1.9 voice allocation + GENMIDI from the WAD + Nuked
OPL3 core in OPL2 mode). Sources in `choc/` are verbatim copies from
chocolate-doom (commit in `choc/UPSTREAM_COMMIT`, GPL-2, see
`choc/COPYING.md`). `src/opl_offline.c` replaces `opl_sdl.c`; `stubs/`
provides config.h / SDL.h shims; `src/engine_stubs.c` supplies W_/Z_/M_/I_.

    make            # build ./musrender
    make render WAD=/path/to/DOOM1.WAD   # render all MUS lumps of your WAD to ./music
    python3 verify.py music              # format/duration/RMS checks + preview_E1M1.wav
    python3 add_loop.py music music/card # add the loop point the camera player uses

Copy `music/card/*.WAV` to `ML/DOOM/MUSIC/` on the card (~83 MB for DOOM1.WAD). The rendered music comes from your
own WAD: don't share it.

Options: `-o DIR -r RATE -g GAIN -t PEAK -v VOL -c CUTOFF -H DCHZ` (run without
args for help). Default: 24000 Hz mono s16le, auto fixed gain = 23000 / loudest
peak over all rendered tracks, music volume 127, 5 Hz DC blocker.

Per song: one pass to the MUS score-end event (no loop), then I_OPL_StopSong
and a release tail; tail kept >= 0.5 s after the last key-off, trimmed once a
10 ms window falls below ~-72 dBFS, hard-capped at key-off + 1.5 s (10 ms
de-click) because some GENMIDI voices have release rate 0 and never decay.
INDEX.TXT's last column SCORE_END_SAMPLE is the sample where the score ends
(use it as the loop point; Chocolate Doom restarts 5 ms after score end).
