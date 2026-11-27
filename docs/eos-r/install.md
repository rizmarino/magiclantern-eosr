# Install and uninstall (EOS R, firmware 1.8.0)

## You need
- **Canon EOS R with firmware 1.8.0.** Canon shipped two 1.8.0 builds; ML targets the one with internal build
  **7.3.9** (July 2020). Check under Setup > Firmware (shows 1.8.0) and, if in doubt, with the Canon Basic
  `CamInfo_Debug(1)` script below (writes `CAM_INFO.XML`). Canon's download page offers both; the camera accepts the
  older one through its normal firmware update menu.
- An SD card you can dedicate to ML. For raw video a fast **V90 UHS-II** card (see [raw-video.md](raw-video.md), card
  speed). Tested: PNY X-Pro 90 128 GB (exFAT), Lexar 1667x 64 GB (exFAT).
- A PC with an SD reader. The card-flag tools below are Python 3.

## 1. Enable the camera's boot flag (once per camera)
The EOS R only runs `autoexec.bin` from a card if a flag in the camera is set. It is set once with a Canon Basic
script, Canon's own scripting feature, and stays set.

1. Use a card of **32 GB or smaller** for this step (Canon Basic needs FAT32). Format it **in the camera**.
2. On the PC, mark it as a script card:
   `sudo python3 tools/card-flags/edit_card_flags.py /dev/sdX1` (the partition; unmount it first).
3. Copy two text files to the card root:
   - `script.req` containing exactly: `for DC_scriptdisk`
   - `extend.m` containing:
     ```
     private sub Initialize()
         EnableBootDisk()
     end sub
     ```
4. Put the card in the camera, switch on, press **Playback**, then **SET**. The script runs once.
5. Delete the two files. To undo later, run the same with `DisableBootDisk()`.

## 2. Prepare the ML card
1. Format the card **in the camera**. Cards of 64 GB and more come out exFAT, 32 GB and less FAT32.
2. Set the card's boot flags on the PC (unmount the partition first):
   - exFAT (64 GB+): `sudo python3 tools/eosr/make_bootable_exfat.py /dev/sdX1`
     (checks it is exFAT, writes the flags, fixes the boot checksum, keeps a backup of the boot region).
   - FAT32: `sudo python3 tools/card-flags/edit_card_flags.py /dev/sdX1`
3. Unzip the release zip (`magiclantern-R180-vX.YY.zip`) to the **card root**, overwriting.

## 3. First start
- Switch on. ML loads alongside Canon's firmware. **Trash** opens and closes the ML menu; the multi-controller and
  dials move through tabs and entries; **SET** changes a value or runs an action; **PLAY** opens an entry's submenu.
- The first start writes a backup of the camera's ROM to `ML/LOGS/ROM0.BIN` / `ROM1.BIN` (standard ML behaviour on
  new ports). **Keep these private**: they are Canon's firmware.
- Modules (Doom, Lua, file manager, benchmarks, ...) load only once enabled in the **Modules** tab, then restart.

## Updating
Unzip the new release over the old one. Settings (`ML/SETTINGS/`) are kept. If the zip removed modules you had,
delete leftover `ML/modules/*.hid` files you didn't create.

## Uninstall
- Easiest: use a card **without** the boot flags. The camera then starts Canon-only.
- Don't just rename or delete `autoexec.bin` on a flagged card: the EOS R then refuses to start from that card.
- Full removal: run the Canon Basic script with `DisableBootDisk()` (step 1).

## After a freeze or crash
Switch off (take the battery out if the camera doesn't respond), then put the card in a PC **before** the next start.
`CRASHnn.LOG` on the card root and `ML/LOGS/` say what happened; please include them in bug reports.
If a 4K frame height switch froze the camera, the next start switches that mode off by itself.
