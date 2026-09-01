# fzero-gx-recomp — F-Zero GX static recompilation for Windows

Native Windows port of F-Zero GX (GameCube `GFZE01`, NTSC-U) via **static recompilation**: the disc's PowerPC executable is translated ahead of time to C, compiled for x86-64, and run as native code inside a runtime providing graphics/audio/input.

No game data or code is distributed. You supply a disc image you already own (`orig/GFZE01/f-zero gx (usa).rvz`).

## Status

Host window + Vulkan WSI boots (GX-blue present). DOL recompiles (36 chunks, 144k instr, 0 unknown). Full playable build in progress.

## Toolchain (Windows native)

- Visual Studio 2022 + VS LLVM `clang 19.1.5` (`vcvars64.bat` + Ninja)
- Vulkan SDK 1.4.350
- CMake 3.20+, Ninja, Python 3

No WSL/Cygwin.

## Quick start

```bat
:: dolrecomp tool
call "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat"
cmake -S vendor/RingOut/DolRecomp -B build/dolrecomp -G Ninja -DCMAKE_C_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang.exe"
cmake --build build/dolrecomp --target dolrecomp

:: DOL -> C
build/dolrecomp/dolrecomp --gamecube orig/GFZE01/sys/main.dol build/recomp_gen

:: host (Vulkan)
cmake -S host -B build/host
cmake --build build/host --config Release
build/host/Release/fzero-gx.exe
```

## Layout

- `host/` — Windows host (Win32 + Vulkan WSI, DVD/OS shims)
- `vendor/RingOut/` — DolRecomp + ModernGekko (submodule, with Windows patches)
- `android/` — Android target (crDroid 15, SD845)
- `orig/GFZE01/` — disc image (gitignored, user-supplied)
- `build/` — generated (gitignored)

## Credits

- [RingOut](https://github.com/jackpoison-prog/RingOut) — DolRecomp + ModernGekko static recomp pipeline (SC2 GRSEAF)
- [gcrecomp](https://github.com/sp00nznet/gcrecomp) — GC recomp inspiration
