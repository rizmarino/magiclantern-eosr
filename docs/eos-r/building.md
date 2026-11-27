# Building from source and PC tools

## Camera build (Linux)
1. ARM toolchain: Arm GNU Toolchain 12.3.rel1, `arm-none-eabi` (x86_64 Linux tarball from developer.arm.com).
2. Python 3 with `docutils` (module docs) on PATH.
3. Build:
   ```
   cd platform/R.180
   make -j4 ARM_BINPATH=/path/to/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin
   ```
   Result: `platform/R.180/build/magiclantern.zip`. Modules are built by `make -C modules` (the platform build
   packages what is there; rebuild modules after changing their sources).
4. Optional checks: the [qemu-eos](https://github.com/reticulatedpines/qemu-eos) emulator boots the R build to the
   point where ML starts (needs your own ROM dump; ROMs are Canon's and are never distributed).

### Research build
The camera build leaves out the research and test tools used to develop these features (sensor register logger,
Canon log capture, step-by-step 4K readout tests, recorder test runs). To get them back, add `#define R_TESTS 1` at the
top of the R source files that use it (`src/rectest_r.c`, `src/tall4k_r.c`, `src/silenttest_r.c`,
`src/crxtest_r.c`) and build with `make R_TESTS=y`. These tools write camera registers and Canon memory: use them only
if you know what they do. Doom's audio bring-up tests: `-DDOOM_TESTS` in the module.

### Where things are
| Path | What |
|---|---|
| `src/rectest_r.c` | raw / lossless video recorder, LV silent pictures |
| `src/tall4k_r.c` | 4K frame heights, whole-frame view, Frame guide |
| `src/romremap_r.c` | shared ROM-page remap (MMU) used by the 4K heights and silent burst |
| `src/silenttest_r.c` | silent burst |
| `src/rackfocus_r.c` | rack focus |
| `src/rawlv_r.c` | raw histogram / zebras, movie Auto ETTR |
| `modules/doom/` | Doom module |
| `platform/R.180/` | EOS R platform files |

## PC tools (`tools/eosr/`)
- `lossless/crx2mlv.py`: lossless MLV -> uncompressed MLV ([raw-video.md](raw-video.md)). Build its decoder:
  1. LibRaw 0.22.1 from https://www.libraw.org: `make -f Makefile.dist` (builds `lib/libraw.a`).
  2. `cd tools/eosr/lossless && make LIBRAW=/path/to/LibRaw` (needs g++, zlib).
  Python 3 with numpy. `crx_verify_rawc.py` checks Direct+verify clips.
- `doom-musrender/`: renders Doom's music from your WAD to WAV for the camera ([doom.md](doom.md)). `make`, then
  `make render WAD=/path/to/DOOM1.WAD`.
- `make_bootable_exfat.py`: sets the ML boot flags on an exFAT card ([install.md](install.md)).

## How this was developed
The R-specific code was written by an AI coding assistant (Anthropic's Claude) working with the repository owner, who
did all camera testing. Each feature went through: static analysis of the R's firmware (Ghidra, on a private ROM dump),
host-side simulations of the recorder and switching logic, the qemu boot test, and camera test rounds whose logs were
checked on the PC (frame-level checks of every test clip). Commit messages record what each version changed and what
the camera showed. See the disclosure in [README.md](README.md).
