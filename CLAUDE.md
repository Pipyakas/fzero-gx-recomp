# fzero-gx-recomp — static recomp (GFZE01)

Fresh root. Prior matching-decomp history at pre-recomp archive; `main` is the only branch.

## Toolchain (Windows native only)

- VS 2022 `vcvars64.bat -arch=x64` + Ninja + VS LLVM `clang 19.1.5` (`.../VC/Tools/Llvm/x64/bin/clang.exe`)
- Vulkan SDK 1.4.350 (`C:/VulkanSDK/1.4.350.0`), `vulkan-1.lib`, `VK_USE_PLATFORM_WIN32_KHR`
- No WSL/Cygwin. All targets build with native Windows toolchain.

## Build

```
call "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat"
cmake -S vendor/RingOut/DolRecomp -B build/dolrecomp -G Ninja -DCMAKE_C_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang.exe"
cmake --build build/dolrecomp --target dolrecomp -j16
build/dolrecomp/dolrecomp --gamecube orig/GFZE01/sys/main.dol build/recomp_gen -j16

cmake -S host -B build/host --config Release
cmake --build build/host --config Release -j16
build/host/Release/fzero-gx.exe   # Vulkan WSI, 960x540 GX-blue
```

`orig/GFZE01/f-zero gx (usa).rvz` (1.2 GiB) is user-supplied, gitignored. Extract to `orig/GFZE01/sys/main.dol` + `orig/GFZE01/files/files/fze.*.rel` (14 RELs).

## Layout

- `host/` — Win32 host (`main_host.c` WndProc, `gx_vulkan.c` Vulkan WSI, `dvd_host.c` DVD->FS, `os_win32.c` QPC)
- `vendor/RingOut/` — submodule `jackpoison-prog/RingOut` (DolRecomp + ModernGekko), patched for Windows (S_ISDIR, fseeki64, dirent->Win32, pthread->CRITICAL_SECTION)
- `android/` — Android target (targetSdk35, crDroid 15 beryllium SD845)
- `build/`, `orig/` — gitignored

## Recomp flow

DOL/REL -> DolRecomp --gamecube -> C chunks -> compile with ModernGekko runtime (CPU ctx, GX->Vulkan, OS HLE) -> fzero-gx.exe
