# Magic Lantern for the Canon EOS R (firmware 1.8.0)

An unofficial Magic Lantern build for the EOS R. It runs alongside Canon's firmware from the memory card, like any
Magic Lantern build. Remove the card (or use one without ML) and the camera is stock again.

**Status: experimental.** Everything below was tested on one EOS R body with firmware 1.8.0. Each feature page says
what was verified on the camera and what was only checked on a PC. Use it at your own risk; keep a card without ML for
paid work.

> **AI disclosure.** Nearly all code in this fork that is not upstream Magic Lantern was written by an AI coding
> assistant (Anthropic's Claude, via the Hermes agent), directed, reviewed and camera-tested by the fork's owner.
> Every such commit says "LLM-written (disclosed)". Upstream Magic Lantern's [LLM_POLICY](../../LLM_POLICY.md) does
> not accept LLM-written changes larger than ~15 lines, so this work is published as a fork, not as upstream pull
> requests. Small fixes may be offered upstream separately.

## What's new on the EOS R

Upstream Magic Lantern for the EOS R (as of `dev` 142472e95, October 2026) offers menus, Global Draw, cropmarks and
picture style. No DIGIC 8 camera records raw video in upstream ML. This fork adds:

| Feature | Page | State |
|---|---|---|
| **Raw video** (MLV, 10/12/14-bit, audio, pre-record) | [raw-video.md](raw-video.md) | Camera-verified |
| **Lossless raw video** with Canon's own CR3 (CRX) encoder, up to 4K | [raw-video.md](raw-video.md) | Camera-verified (tested modes listed) |
| **4K frame height**: 16:9, Tall (~3:2) and Wide sensor readouts beyond Canon's 4K window | [4k-frame-height.md](4k-frame-height.md) | Camera-verified at 23.98p |
| **Whole-frame screen view** of those taller/wider frames | [4k-frame-height.md](4k-frame-height.md) | Camera-verified |
| **Frame guide**: map of recorded picture vs. screen | [4k-frame-height.md](4k-frame-height.md) | Camera-verified |
| **Silent burst** above Canon's 5 fps (up to ~7 fps) | [stills.md](stills.md) | Camera-verified |
| **LV silent pictures**: 4K raw DNG burst / pre-capture from movie live view | [stills.md](stills.md) | Camera-verified |
| **Rack focus** between two stored focus points | [focus-exposure-audio.md](focus-exposure-audio.md) | Camera-verified |
| Movie **Auto ETTR**, **Kelvin white balance**, raw histogram / zebras | [focus-exposure-audio.md](focus-exposure-audio.md) | Camera-verified |
| **Beeps** through Canon's sound-effect player | [focus-exposure-audio.md](focus-exposure-audio.md) | Camera-verified |
| **Doom** (optional module, bring your own WAD) | [doom.md](doom.md) | Camera-verified |
| Stock ML features ported: exposure presets, overlays, intervalometer, bracketing, Lua, ... | [other-features.md](other-features.md) | Camera-verified |

Read [known-limitations.md](known-limitations.md) before relying on any of it.

## Start here

1. [install.md](install.md): card preparation, install, uninstall.
2. [raw-video.md](raw-video.md): the main feature, plus the PC converter for lossless clips.
3. [known-limitations.md](known-limitations.md).

Building from source and the PC tools: [building.md](building.md).

## Credits

- Magic Lantern team and contributors (GPL), and **jed2nd** for the EOS R port groundwork (upstream PR #281, merged
  in this fork).
- doomgeneric (ozkl), Chocolate Doom, id Software's Doom source (GPL-2).
- LibRaw (LGPL-2.1 / CDDL): CRX decoder used by the PC converter.
- Fork owner and camera testing: riz ([rizmarino](https://github.com/rizmarino)).

License: GPL-2.0, like Magic Lantern (see [COPYING](../../COPYING)).
