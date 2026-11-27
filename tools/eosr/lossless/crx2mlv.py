#!/usr/bin/env python3
"""crx2mlv: turn an EOS R lossless clip (ML v4.25+, Movie > RAW video > Compression: Lossless) into a standard
uncompressed MLV that MLV App / mlv_dump / MLV-to-DNG tools read.

The camera writes each frame as a VIDF block holding a 'CRXV' record: Canon's CRX (CR3 lossless) main header
plus the compressed tile/plane entries (each padded to 64 bytes). This decodes every frame with LibRaw's CRX
decoder (crxdec, next to this script) and writes it back as ML raw_pixblock-packed pixels at the clip's bit depth.
All other blocks (RAWI, IDNT, RTCI, WAVI, AUDF ...) are copied in file order; MLVI gets the plain RAW class and the
total frame count. 'RAWC' blocks (v4.28 Direct+verify test frames, uncompressed copies) and NULL padding are dropped;
crx_verify_rawc.py checks the RAWC frames.

usage: crx2mlv.py CLIP.MLV [OUT.MLV] [--check N]   (.M00, .M01 ... parts are picked up automatically)
  --check N: also verify N frames by re-reading the written output (unpack == decoded values)
"""
import glob, os, struct, subprocess, sys, tempfile, time
from multiprocessing import Pool
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CRXDEC = os.path.join(HERE, 'crxdec')
AL = lambda x: (x + 63) & ~63


def pack_ml(v, bpp, step=1 << 18):
    """ML raw_pixblock order (inverse of mlv_check.unpack_ml): bits MSB-first, then 16-bit little-endian words.
    Packs in slices of `step` pixels (a multiple of 16, so each slice ends on a 16-bit word): the whole-frame
    version needed ~450 MB per worker at 4480x2478 and, with 4 workers, pushed the box into systemd-oomd
    (2026-11-27)."""
    sh = np.arange(bpp - 1, -1, -1, dtype=np.uint16)
    out = []
    for i in range(0, v.size, step):
        bits = ((v[i:i + step].astype(np.uint16)[:, None] >> sh) & 1).astype(np.uint8)
        b = np.packbits(bits.reshape(-1))
        out.append(b.reshape(-1, 2)[:, ::-1].reshape(-1).tobytes())
    return b''.join(out)


def unpack_ml(buf, n, bpp):
    b = np.frombuffer(buf, np.uint8, n * bpp // 8)
    b = b.reshape(-1, 2)[:, ::-1].reshape(-1)
    bits = np.unpackbits(b).reshape(-1, bpp).astype(np.uint32)
    return (bits << np.arange(bpp - 1, -1, -1, dtype=np.uint32)).sum(1).astype(np.uint16)


def blocks(b):
    o = 0
    while o + 16 <= len(b):
        t, sz = b[o:o + 4], struct.unpack_from('<I', b, o + 4)[0]
        if sz < 16 or o + sz > len(b):
            raise SystemExit(f'bad block {t} size {sz} at {o}')
        yield o, t, sz
        o += sz


def crx_record(b, o):
    """VIDF at o -> (frame no, ts, crop/pan words, cmp1 bytes, sample bytes)"""
    ts, fno = struct.unpack_from('<QI', b, o + 8)
    crop = b[o + 20:o + 28]
    space = struct.unpack_from('<I', b, o + 28)[0]
    q = o + 32 + space
    mg, ver, n, hs = struct.unpack_from('<4sIII', b, q)
    if mg != b'CRXV':
        raise SystemExit(f'VIDF {fno}: not a lossless (CRXV) frame')
    sizes = struct.unpack_from(f'<{n}I', b, q + 32)
    cmp1 = b[q + 192:q + 192 + hs]
    e, parts = q + 192 + AL(hs), []
    for s in sizes:
        if s:
            parts.append(b[e:e + s]); e += AL(s)
    return fno, ts, crop, cmp1, b''.join(parts)


_MAPS = None


def _open_parts(paths):
    """Pool initializer: each worker maps the clip parts itself, so jobs stay (part, offset) pairs instead of
    ~5 MB of compressed bytes each (all 407 frames of a 17 s Wide take were queued at once = ~2 GB, 2026-11-27)."""
    global _MAPS, _FHS
    import mmap
    _FHS = [open(p, 'rb') for p in paths]   # keep the files open: a temporary open() is closed before mmap() runs
    _MAPS = [mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) for fh in _FHS]


def decode(job):
    k, o, w, h, bpp = job
    fno, ts, crop, cmp1, sample = crx_record(_MAPS[k], o)
    with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as t:
        open(f'{t}/c', 'wb').write(cmp1); open(f'{t}/s', 'wb').write(sample)
        r = subprocess.run([CRXDEC, f'{t}/c', f'{t}/s', f'{t}/o'], capture_output=True, text=True)
        if r.returncode:
            return fno, None, r.stderr.strip()
        v = np.fromfile(f'{t}/o', '<u2')
    if v.size != w * h:
        return fno, None, f'decoded {v.size} px, expected {w}x{h}'
    if int(v.max()) >= 1 << bpp:
        return fno, None, f'value {int(v.max())} exceeds {bpp} bits'
    return fno, pack_ml(v, bpp), None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    nchk = int(sys.argv[sys.argv.index('--check') + 1]) if '--check' in sys.argv else 0
    if nchk: args = [a for a in args if a != str(nchk)]
    src = args[0]
    out = args[1] if len(args) > 1 else src[:-4] + '_RAW.MLV'
    base = src[:-4]
    parts = [src] + sorted(glob.glob(base + '.M[0-9][0-9]'))
    import mmap                              # mmap the parts: read() of a long take exceeds RAM (OOM-killed, 2026-11-15)
    fhs = [open(p, 'rb') for p in parts]   # keep the file objects alive while the maps are used
    data = [mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) for fh in fhs]
    b0 = data[0]
    if b0[:4] != b'MLVI':
        raise SystemExit('not an MLV')
    vclass = struct.unpack_from('<H', b0, 32)[0]
    if not vclass & 0x1000:
        raise SystemExit(f'{src}: not a lossless clip (video class {vclass:#x})')
    hdr = bytearray()
    w = h = bpp = None
    for o, t, sz in blocks(b0):
        if t == b'VIDF':
            break
        if t == b'RAWC':                    # v4.28 verify frame (can be written before the first VIDF)
            continue
        blk = bytearray(b0[o:o + sz])
        if t == b'MLVI':
            struct.pack_into('<H', blk, 32, 1)                     # plain RAW video class
            struct.pack_into('<HH', blk, 24, 0, 1)                 # file 0 of 1
        if t == b'RAWI':
            w, h = struct.unpack_from('<HH', blk, 16)
            bpp = struct.unpack_from('<i', blk, 20 + 24)[0]
        hdr += blk
    recs, seq, hdr_done = [], [], False       # seq: ('V', record index) or ('B', block bytes), in file order
    for k, b in enumerate(data):
        prev = None
        for o, t, sz in blocks(b):
            if t == b'VIDF':
                hdr_done = True
                seq.append(('V', len(recs))); recs.append((k, o))
            elif not hdr_done or t in (b'RAWC', b'MLVI', b'RAWI', b'IDNT', b'RTCI', b'WAVI', b'EXPO', b'LENS', b'WBAL'):
                pass
            elif t != b'NULL' or prev == b'AUDF':  # AUDF etc. after the header, with the NULL padding after AUDF
                seq.append(('B', b[o:o + sz]))
            prev = t
    n = len(recs)
    frame_bytes = w * h * bpp // 8
    print(f'{len(parts)} part(s), {n} frames, {w}x{h} {bpp}-bit -> {out}')
    struct.pack_into('<I', hdr, 36, n)                              # MLVI vframes
    t0 = time.time()
    jobs = [(k, o, w, h, bpp) for k, o in recs]
    fno_of = {struct.unpack_from('<I', data[k], o + 16)[0]: j for j, (k, o) in enumerate(recs)}
    meta = {struct.unpack_from('<I', data[k], o + 16)[0]: data[k][o + 8:o + 28] for k, o in recs}
    bad, done = [], 0
    nblk = 0
    with open(out, 'wb') as f, Pool(initializer=_open_parts, initargs=(parts,)) as pool:
        f.write(hdr)
        res = pool.imap(decode, jobs, chunksize=2)
        for kind, item in seq:
            if kind == 'B':
                f.write(item); nblk += 1; continue
            fno, packed, err = next(res)
            if err:
                bad.append((fno, err)); continue
            m = meta[fno]
            vidf = b'VIDF' + struct.pack('<I', 32 + frame_bytes) + m + struct.pack('<I', 0)
            f.write(vidf); f.write(packed)
            done += 1
            if done % 50 == 0:
                print(f'  {done}/{n} frames, {time.time() - t0:.0f} s')
    if nblk: print(f'  {nblk} other blocks copied (audio etc.)')
    if bad:
        print(f'!! {len(bad)} frames failed to decode, first: {bad[:3]}')
        b = bytearray(open(out, 'rb').read(len(hdr)))
        struct.pack_into('<I', b, 36, n - len(bad))
        with open(out, 'r+b') as f:
            f.write(b)
    print(f'wrote {done} frames in {time.time() - t0:.0f} s')
    if nchk:
        import mmap                          # never read() the whole output: a UHD clip is bigger than the PC's RAM
        fo = open(out, 'rb')
        ob = mmap.mmap(fo.fileno(), 0, access=mmap.ACCESS_READ)
        vids = []
        for o, t, sz in blocks(ob):
            if t == b'VIDF':
                vids.append((o, sz))
                if len(vids) >= nchk: break
        for i, (o, sz) in enumerate(vids):
            fno = struct.unpack_from('<I', ob, o + 16)[0]
            if _MAPS is None: _open_parts(parts)
            _, packed, _ = decode(jobs[fno_of[fno]])
            got = unpack_ml(ob[o + 32:o + 32 + frame_bytes], w * h, bpp)
            ref = unpack_ml(packed, w * h, bpp)
            print(f'  check frame {fno}: {"OK" if np.array_equal(got, ref) else "MISMATCH"}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
