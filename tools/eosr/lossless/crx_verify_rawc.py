#!/usr/bin/env python3
"""crx_verify_rawc: check a v4.28 "Lossless source: Direct+verify" clip. About once a second the camera also
EDMAC-copied the frame it was compressing straight from Canon's frame buffer and wrote it uncompressed as a 'RAWC'
block. Each RAWC frame is compared with the decoded CRX frame of the same frame number: identical = the encoder read
the whole frame before the sensor overwrote it (no tearing from the direct source).
usage: crx_verify_rawc.py CLIP.MLV"""
import glob, os, struct, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from crx2mlv import blocks, crx_record, decode, unpack_ml


def unpack_lsb(buf, n, bpp):                     # Canon's bit order (lossless clips are packed so)
    b = np.frombuffer(buf, np.uint8, n * bpp // 8)
    bits = np.unpackbits(b, bitorder='little').reshape(-1, bpp).astype(np.uint32)
    return (bits << np.arange(bpp, dtype=np.uint32)).sum(1).astype(np.uint16)


src = sys.argv[1]
parts = [src] + sorted(glob.glob(src[:-4] + '.M[0-9][0-9]'))
data = [open(p, 'rb').read() for p in parts]
w = h = bpp = None
vidf, rawc = {}, {}
for k, b in enumerate(data):
    for o, t, sz in blocks(b):
        if t == b'RAWI' and w is None:
            w, h = struct.unpack_from('<HH', b, o + 16); bpp = struct.unpack_from('<i', b, o + 20 + 24)[0]
        elif t == b'VIDF':
            vidf[struct.unpack_from('<I', b, o + 16)[0]] = (k, o)
        elif t == b'RAWC':
            rawc[struct.unpack_from('<I', b, o + 16)[0]] = (k, o)
print(f'{len(vidf)} VIDF, {len(rawc)} RAWC verify frames, {w}x{h} {bpp}-bit')
ok = bad = 0
for fno in sorted(rawc):
    k, o = rawc[fno]
    b = data[k]
    space = struct.unpack_from('<I', b, o + 28)[0]
    ref = unpack_lsb(b[o + 32 + space:o + 32 + space + w * h * bpp // 8], w * h, bpp).reshape(h, w)
    if fno not in vidf:
        print(f'  RAWC {fno}: no VIDF with that frame number'); bad += 1; continue
    kk, oo = vidf[fno]
    f2, ts, crop, cmp1, sample = crx_record(data[kk], oo)
    _, packed, err = decode((fno, cmp1, sample, w, h, bpp))
    if err:
        print(f'  RAWC {fno}: decode failed: {err}'); bad += 1; continue
    got = unpack_ml(packed, w * h, bpp).reshape(h, w)
    diff = np.nonzero((got != ref).any(1))[0]
    if not len(diff):
        ok += 1; print(f'  frame {fno}: identical'); continue
    bad += 1
    print(f'  frame {fno}: {len(diff)} rows differ (first {diff[0]}, last {diff[-1]}); '
          f'{int((got != ref).sum())} px; mean |diff| {np.abs(got.astype(int) - ref).mean():.2f}')
print(f'RAWC VERIFY: {ok} identical, {bad} differ' + (' -> PASS' if ok and not bad else ' -> FAIL' if bad else ''))
sys.exit(1 if bad or not ok else 0)
