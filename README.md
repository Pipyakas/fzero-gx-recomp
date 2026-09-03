# fzero-gx-recomp — F-Zero GX static recompilation

Native port of F-Zero GX (GameCube `GFZE01`, NTSC-U) via **static recompilation**: the disc's PowerPC executable is translated ahead of time to C (PPC→C→native), compiled for x86-64/ARM64, and run inside a runtime providing graphics, audio, input and hardware emulation.

No game data or code is distributed. You supply a disc image you already own (`orig/GFZE01/f-zero gx (usa).rvz`).

## Status

Host window + Vulkan WSI boots (GX-blue present). DOL recompiles (36 chunks, 144k instr, 0 unknown). Full playable build in progress. Android target (arm64-v8a, crDroid 15 SD845) shares the same recompiled chunks — see `android/`.

## Toolchain (Windows native)

- Visual Studio 2022 + VS LLVM `clang 19.1.5` (`vcvars64.bat` + Ninja)
- Vulkan SDK 1.4.350
- CMake 3.20+, Ninja, Python 3

No WSL/Cygwin. The emitted chunks are plain C and retarget cleanly to ARM64 (same `recomp_core` lib, `VK_USE_PLATFORM_ANDROID_KHR` + `os_posix` shims on Android).

## Quick start

```bat
:: dolrecomp tool
call "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat"
cmake -S vendor/RingOut/DolRecomp -B build/dolrecomp -G Ninja -DCMAKE_C_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang.exe"
cmake --build build/dolrecomp --target dolrecomp

:: DOL -> C (run once; output shared by Windows + Android)
build/dolrecomp/dolrecomp --gamecube orig/GFZE01/sys/main.dol build/recomp_gen

:: host — Windows (Vulkan)
cmake -S host -B build/host --config Release
cmake --build build/host --config Release
build/host/Release/fzero-gx.exe

:: host — Android (arm64-v8a, NDK r28)
:: see android/README.md — gradle + ndk + cmake -> app-debug.apk -> adb install
```

## Layout

- `host/` — native host (Win32 + Vulkan WSI, DVD/OS shims; `VK_USE_PLATFORM_WIN32_KHR` / `VK_USE_PLATFORM_ANDROID_KHR` per platform)
- `vendor/RingOut/DolRecomp/` — DolRecomp fork (vendored submodule, Windows patches: `S_ISDIR`, `fseeki64`, `dirent`→Win32, `pthread`→`CRITICAL_SECTION`)
- `android/` — Android target (crDroid 15, SD845, `targetSdk 35`, Vulkan 1.1)
- `orig/GFZE01/` — disc image (gitignored, user-supplied)
- `build/` — generated chunks + objects (gitignored)

## Credits

- [DolRecomp](https://github.com/ExpansionPak/DolRecomp) — upstream static recompiler (vendored here via [RingOut](https://github.com/jackpoison-prog/RingOut) at `f66a308` + Windows patches: `S_ISDIR`/`fseeki64`/`dirent`/`CRITICAL_SECTION`; own fork to be split out as `vendor/DolRecomp/`)
- [ModernGekko](https://github.com/ExpansionPak/ModernGekko) / [GXRuntime](https://github.com/ExpansionPak/ModernGekko) — Dolphin-derived runtime (GX, DVD, VI, SI, audio) used for the playable chassis
- [gcrecomp](https://github.com/sp00nznet/gcrecomp) — GC recomp inspiration
