#!/usr/bin/env python3
"""Make an exFAT SD/CF card bootable for Magic Lantern (Canon "EOS_DEVELOP"/"BOOTDISK" flags).

Usage: sudo python3 make_bootable_exfat.py /dev/sdX1        (the exFAT PARTITION, not /dev/sdX)
       python3 make_bootable_exfat.py image.img --check     (read-only: report flags + checksum)

What it does, and nothing else:
  - verifies the target is exFAT (OEM name "EXFAT   " at 0x03) and its boot checksum is valid
  - writes "EOS_DEVELOP" at 0x82 and "BOOTDISK" at 0x7A of the main boot sector (sector 0)
    and the backup boot sector (sector 12)
  - recomputes the exFAT boot-region checksum over sectors 0..10 and rewrites sector 11
    (and the backup checksum sector 23)
  - re-reads and verifies
Offsets 0x7A..0x8C lie in the exFAT BootCode area (0x78..0x1FD), not in any filesystem field.
Files on the card are untouched. A backup of the 24 boot sectors is saved next to this script.
The camera's own boot flag must already be enabled.
"""
import os, struct, sys, time

def checksum(region, bps):
    s = 0
    for i, b in enumerate(region[: 11 * bps]):
        if i in (106, 107, 112):          # VolumeFlags (2 bytes) and PercentInUse are excluded
            continue
        s = (((s & 1) << 31) | (s >> 1)) + b
        s &= 0xFFFFFFFF
    return s

def info(region, bps):
    s0 = region[:bps]
    return dict(dev=s0[0x82:0x8D] == b'EOS_DEVELOP', boot=s0[0x7A:0x82] == b'BOOTDISK',
                ck_stored=struct.unpack_from('<I', region, 11 * bps)[0], ck_calc=checksum(region, bps))

def main():
    path = sys.argv[1]; check_only = '--check' in sys.argv
    with open(path, 'rb') as f:
        head = f.read(512)
        if head[3:11] != b'EXFAT   ':
            sys.exit(f'{path}: not exFAT (OEM name {head[3:11]!r}). Point this at the exFAT partition, e.g. /dev/sdX1.')
        bps = 1 << head[0x6C]
        if bps not in (512, 1024, 2048, 4096):
            sys.exit(f'unexpected sector size {bps}')
        f.seek(0); region = bytearray(f.read(24 * bps))
    i = info(region, bps)
    print(f'{path}: exFAT, {bps} B/sector; EOS_DEVELOP={i["dev"]} BOOTDISK={i["boot"]} '
          f'checksum stored {i["ck_stored"]:#010x} calc {i["ck_calc"]:#010x} {"OK" if i["ck_stored"] == i["ck_calc"] else "MISMATCH"}')
    if check_only:
        return
    if i['ck_stored'] != i['ck_calc']:
        sys.exit('boot checksum already invalid; refusing to modify (reformat the card in the camera first)')
    if i['dev'] and i['boot']:
        print('already bootable, nothing to do'); return
    bak = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'exfat-bootregion-backup-{int(time.time())}.bin')
    open(bak, 'wb').write(region); print('backup of boot region:', bak)
    for base in (0, 12 * bps):                       # main and backup boot sectors
        region[base + 0x7A: base + 0x82] = b'BOOTDISK'
        region[base + 0x82: base + 0x8D] = b'EOS_DEVELOP'
    ck = checksum(region, bps)
    for sec in (11, 23):
        region[sec * bps:(sec + 1) * bps] = struct.pack('<I', ck) * (bps // 4)
    fd = os.open(path, os.O_RDWR | getattr(os, 'O_SYNC', 0))
    try:
        os.lseek(fd, 0, 0); os.write(fd, bytes(region)); os.fsync(fd)
    finally:
        os.close(fd)
    with open(path, 'rb') as f:
        again = f.read(24 * bps)
    j = info(again, bps)
    ok = j['dev'] and j['boot'] and j['ck_stored'] == j['ck_calc'] and again == bytes(region)
    print(f'written; verify: EOS_DEVELOP={j["dev"]} BOOTDISK={j["boot"]} checksum {j["ck_stored"]:#010x} '
          f'{"OK" if ok else "FAILED"}')
    if not ok:
        sys.exit(1)

if __name__ == '__main__':
    main()
