# fzero-gx-recomp — next (ringout path)

Recomp approach now matches [RingOut](https://github.com/jackpoison-prog/RingOut) SC2 GC static recomp: `vendor/RingOut` (`DolRecomp`+`ModernGekko`+vendored Dolphin) vendored at `ca9bda2`.

- `GFZE01` DOL `sys/main.dol` SHA `421c881…` + 14 RELs `fze.*.rel` (2.45 MB at `orig/GFZE01/files/files/`) instead of SC2's GRSEAF.
- `host/` stays until ModernGekko proves boot: `host/src/main_host.c` WndProc stub 13312 B → replace with ModernGekko launch (setup.sh extracts RVZ→recompiles ~535k ins to C→x86-64, runtime Vulkan on Adreno630 vk1.1, interpreter 0.7% for exception vectors 0x500/0xC00/0x800).
- Needs on `recomp`: cmake/ninja/python3/clang+vulkan driver ~1.5GB scratch (already 2022 toolchain). First run minutes, then instant; Steam Deck pkg pattern reusable for Android (`arm64-v8a` targetSdk35 crDroid 15 beryllium `d1cadee8`).
