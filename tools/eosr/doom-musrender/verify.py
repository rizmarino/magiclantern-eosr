#!/usr/bin/env python3
"""Verify musrender output: WAV format, durations, per-second RMS, header
sanity vs INDEX.TXT, and write a 20 s preview of D_E1M1. Stdlib only."""
import array, math, os, struct, sys, wave

MUSIC = sys.argv[1] if len(sys.argv) > 1 else "music"
EXPECT = ["D_E1M1", "D_E1M2", "D_E1M3", "D_E1M4", "D_E1M5", "D_E1M6",
          "D_E1M7", "D_E1M8", "D_E1M9", "D_INTER", "D_INTRO", "D_VICTOR",
          "D_INTROA"]

index = {}
for line in open(os.path.join(MUSIC, "INDEX.TXT")):
    if line.startswith("#") or not line.strip():
        continue
    f = line.split()
    index[f[0]] = f

ok = True
total = 0
print(f"{'LUMP':9} {'dur_s':>8} {'samples':>8} {'peak':>6} {'rms':>7} "
      f"{'1s-RMS min/med/max':>22} {'silent_s':>8} {'zc/s':>6} {'dc':>6}")
for name in EXPECT:
    path = os.path.join(MUSIC, name + ".WAV")
    if not os.path.exists(path):
        print(name, "MISSING"); ok = False; continue
    raw = open(path, "rb").read()
    total += len(raw)
    # exact 44-byte canonical header
    hdr = struct.unpack("<4sI4s4sIHHIIHH4sI", raw[:44])
    exp_hdr = (b"RIFF", len(raw) - 8, b"WAVE", b"fmt ", 16, 1, 1, 24000,
               48000, 2, 16, b"data", len(raw) - 44)
    if hdr != exp_hdr:
        print(name, "BAD HEADER", hdr); ok = False
    w = wave.open(path, "rb")
    p = w.getparams()
    if (p.nchannels, p.sampwidth, p.framerate, p.comptype) != (1, 2, 24000, "NONE"):
        print(name, "BAD PARAMS", p); ok = False
    a = array.array("h"); a.frombytes(w.readframes(p.nframes)); w.close()
    if sys.byteorder != "little":
        a.byteswap()
    n = len(a)
    peak = max(abs(x) for x in a)
    rms = math.sqrt(sum(x * x for x in a) / n)
    dc = sum(a) / n
    win = [math.sqrt(sum(x * x for x in a[i:i + 24000]) / 24000)
           for i in range(0, n - 23999, 24000)]
    ws = sorted(win)
    silent = sum(1 for r in win if r < 30)          # < ~-61 dBFS
    zc = sum(1 for i in range(1, n) if (a[i - 1] < 0) != (a[i] < 0)) / (n / 24000)
    ix = index.get(name)
    if ix is None or int(ix[2]) != n or int(ix[3]) != peak or \
       abs(float(ix[4]) - rms) > 0.1 or int(ix[5]) != len(raw) or \
       abs(float(ix[1]) - n / 24000) > 0.0005:
        print(name, "INDEX MISMATCH", ix, n, peak, rms, len(raw)); ok = False
    print(f"{name:9} {n/24000:8.3f} {n:8d} {peak:6d} {rms:7.1f} "
          f"{ws[0]:7.1f}/{ws[len(ws)//2]:6.1f}/{ws[-1]:7.1f} {silent:8d} "
          f"{zc:6.0f} {dc:6.1f}")
    if name == "D_E1M1":
        with wave.open(os.path.join(MUSIC, "preview_E1M1.wav"), "wb") as o:
            o.setnchannels(1); o.setsampwidth(2); o.setframerate(24000)
            o.writeframes(a[:20 * 24000].tobytes())

print("total WAV bytes (13 tracks):", total, f"= {total/1048576:.2f} MiB")
print("ALL OK" if ok else "PROBLEMS FOUND")
