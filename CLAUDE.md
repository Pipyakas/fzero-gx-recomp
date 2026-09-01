# fzero-gx-recomp — readable recomp (no DOL SHA constraint)

`main` is the recomp branch (readable, playable Windows+Android). No DOL SHA constraint — readable recomp, not matching decomp. Reference: `karamzov123/fzero-gx-decomp` kept as reference only (no `upstream` remote; `git remote add tmp ...` to cherry-pick).

## Branches

- `main` — recomp: readable `OSThread/GX` structs, `REL` loader, `host/` SDL2+Vulkan Android (crDroid 15 beryllium `SD845 Adreno630 vk1.1 1080x2246` via `adb d1cadee8` at D1) + Windows, `C:\Steam\userdata\*/shortcuts.vdf` non-Steam via `host/add_steam_shortcut.py`. From `dev a52cc0b` (92.49%, 150 left) then diverged.

Location: `C:\code\fzero-gx-recomp` on D1 `100.104.216.83` (renamed from `fzero-gx-decomp`). Fork: `Pipyakas/fzero-gx-recomp` (renamed from `fzero-gx-decomp` via `gh repo rename`). `origin` now `git@github.com:Pipyakas/fzero-gx-recomp.git`.

## Work on `main` (recomp)

- No DOL SHA constraint — `build/GFZE01/main.dol: OK` not required (readable recomp).
- `orig/GFZE01/f-zero gx (usa).rvz` 1.2 GiB → `orig/GFZE01/files` 14 RELs `fze.*.rel` 2.45 MB already extracted; `src/rel/*.c` 14 stubs.
- `host/` is playable: `host/src/main_host.c` `WndProc` 960x540 GX-blue + `host/src/gx_null.c` (replace with `gx_vulkan.c`) + SDL2 → `cmake -S host -B build/host --config Release` → `build/host/Release/fzero-gx.exe` (13312 B stub, now `C:\Steam` non-Steam).
- Recomp approach: use `gcrecomp` pattern (GC Gekko→C, N64Recomp-style, see https://github.com/sp00nznet/gcrecomp) or manual readable translation of `src/dolphin/*` + game `REL` code. Other efforts: `gcrecomp` toolkit itself (generic GC static recompilation to C→native x86-64, GX→D3D11+TEV/DSP/IR, OS HLE), no F-Zero GX–specific recomp found yet — we are not blocked by one.

## Other recomps (searched 2026-09-01)

- `sp00nznet/gcrecomp` — GC static recomp toolkit (generic, not F-Zero GX specific). Pipeline: DOL/REL → C → native runtime (CPU ctx/mem, GX→D3D11, DSP→ADPCM, Dolphin OS HLE). No F-Zero GX named.
- `N64Recomp/N64Recomp` — N64 original (inspired gcrecomp).
- No dedicated `fzero-gx-recomp` project found — we are effectively greenfield.

## Live reporting

Every `ScheduleWakeup 600s` (`continously monitor … report here realtime`): HAPPENING / DONE / NEXT 10-30m / TOTAL (not applicable — readable, not matched%) / CURRENT target (file/mode). Even if blocked, state blocker.

## Commands

```
git checkout main
# karamzov123/fzero-gx-decomp is reference only (no upstream remote)
# to cherry-pick: git remote add tmp https://github.com/karamzov123/fzero-gx-decomp.git && git fetch tmp
python host/add_steam_shortcut.py  # after each host build
cmake -S host -B build/host && cmake --build build/host --config Release
build\host\Release\fzero-gx.exe
adb -s d1cadee8 shell getprop ro.crdroid.version  # Poco F1 crDroid 15
```
