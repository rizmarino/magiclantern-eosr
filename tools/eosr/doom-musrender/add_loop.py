#!/usr/bin/env python3
"""Card copies of the rendered music: insert a 'dmlp' RIFF chunk before 'data' with the loop end sample.

Chocolate Doom restarts a song 5 ms after the score ends (SCORE_END_SAMPLE in INDEX.TXT); the rendered file keeps
up to 1.5 s of ringing tail after that. doom.mo's music player plays [0, loop_end) on every pass and, when it wraps,
mixes the tail [loop_end, end) into the restart (the voices keep ringing over the new start, as on the synth).
Non-looping playback (title) plays the whole file.

dmlp chunk: 8 bytes = uint32 loop_end (samples from the data start), uint32 version (1).
Usage: add_loop.py <music dir> <out dir>"""
import os, struct, sys, wave

src, dst = sys.argv[1], sys.argv[2]
os.makedirs(dst, exist_ok=True)
score_end = {}
for line in open(os.path.join(src, 'INDEX.TXT')):
    if line.startswith('#') or not line.strip():
        continue
    f = line.split()
    score_end[f[0]] = int(f[6])

for lump, se in sorted(score_end.items()):
    w = wave.open(os.path.join(src, lump + '.WAV'))
    assert (w.getnchannels(), w.getsampwidth(), w.getframerate()) == (1, 2, 24000), lump
    n = w.getnframes()
    data = w.readframes(n)
    loop_end = min(n, se + 120)                      # score end + 5 ms
    fmt = struct.pack('<4sIHHIIHH', b'fmt ', 16, 1, 1, 24000, 48000, 2, 16)
    dmlp = struct.pack('<4sIII', b'dmlp', 8, loop_end, 1)
    body = b'WAVE' + fmt + dmlp + struct.pack('<4sI', b'data', len(data)) + data
    with open(os.path.join(dst, lump + '.WAV'), 'wb') as o:
        o.write(b'RIFF' + struct.pack('<I', len(body)) + body)
    print('%-9s samples %8d  loop_end %8d  tail %5d (%.3f s)' % (lump, n, loop_end, n - loop_end, (n - loop_end) / 24000))
