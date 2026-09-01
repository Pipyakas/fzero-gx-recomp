
# Independent roadmap — full Windows build (GFZE01)

> Goal: proceed independently from `karamzov123/fzero-gx-decomp` until we have a **full native Windows build of the game**, merging upstream when possible. Origin is `Pipyakas/fzero-gx-decomp` (`main`).

## Branching

- `main` — mirrors upstream + our landed conversions (currently 92.49% matched, 2085/2235 fns). `origin/main` tracks `upstream/main`.
- `dev` — independent integration branch for this fork. All NATC conversions land here first, then fast-forward to `main` when green. Upstream is merged into `dev` via `git fetch upstream && git merge --no-ff upstream/main` (manual, or via `sync-upstream.yml` workflow creating a PR).
- Feature branches `natc/<unit>-<fn>` for individual conversions, PR'd to `dev`.

Upstream policy: **rebase never, merge only** so history stays bisectable. Conflicts resolved in favour of our landed conversions; report upstream if they diverge on compiler pins (GC/1.2.5n vs GC/1.3 etc).

## Current state (2026-08-31, D1)

- RVZ at `orig/GFZE01/f-zero gx (usa).rvz` (1.2 GB) → `sys/main.dol` SHA `421c8810…` OK, `ninja` clean.
- `build/GFZE01/report.json`: 92.49% matched_code, 93.29% matched_functions, **150 functions remain** across 9 units (worst: `main` 40 left, `model_80072EDC` 30 left, `model_80074D88` 24 left, `MTXHead` 22 left).
- `configure.py`: 195 `Matching`, 2 `NonMatching` stubs missing on disk (`Runtime.PPCEABI.H/global_destructor_chain.c`, `__init_cpp_exceptions.cpp`), 7 `mw_version` overrides.
- Scope is `main.dol` only (Dolphin SDK/MSL/MetroTRK/middleware). Game code lives in disc RELs — **out of scope today** and required for a playable Windows build.

## Phases to Windows exe

### Phase 0 — close the decomp (this repo, MWCC GC/1.3.2 matching)

Priority order (fuzzy ascending, quick wins first):
1. Stub + match the 2 `NonMatching` Runtime files (bare `blr` or trivial C++).
2. `game/tail_800410A4` — 1 fn `fn_800411F4` (99.87%)
3. `dolphin/mtx/MTXFused` — 2 fns (99.39%)
4. `dolphin/msl/printf` — 4 fns `double2hex/float2str/__pformatter/longlong2str` (99.6-99.9%)
5. `dolphin/os/OSThreadInit` — 1 fn (98.6%)
6. `dolphin/dvd/fstload` — 3 fns `cb/__fstLoad/fn_8001A55C` (89-92%, hardest small unit)
7. `dolphin/mtx/MTXHead` — 22 fns (60-98%)
8. `game/model_80072EDC` — 30 fns (93-99%)
9. `game/model_80074D88` — 24 fns (82-99%)
10. `main` — 40 fns (70-98%, includes user callbacks `mmu_user_fn/dvdfs_user_fn/dvd_user_fn`)

Gate per conversion: `objdiff` 100% fuzzy + `ninja` DOL SHA gate GREEN + `// provenance:` per `docs/REFERENCE-POLICY.md`. Land via `dev`, not directly to `main`.

### Phase 1 — bring RELs into scope

- `dtk` extract full disc (RVZ → `orig/GFZE01/files/*.rel`) and split via `config.yml` `modules:` (one `Rel` entry per REL). Current `configure.py` has a single `Rel()` helper but zero REL libs wired.
- Add REL sections to `config/GFZE01/splits.txt` / `symbols.txt` (use `gen_initial_splits.py` + manual correction).
- Decomp REL game code the same NATC loop. This is the bulk of the actual game.

### Phase 2 — host platform layer (GameCube → Windows)

Keep matching builds (MWCC → DOL/REL) intact; add a parallel **host build** (CMake + MSVC/MinGW) that compiles the same `src/` against shims:

- `GX/GD` → modern renderer (Dolphin `GX` re-impl or translate to OpenGL/Vulkan via `libGX` shim; start with null renderer).
- `VI/AI/DVD/AR/CARD/SI/PAD/OS` → SDL2 + host FS + Win32 (file I/O, window, input, audio via cubeb/SDL_audio).
- `AX/DSP` → HLE or stub (AX already partially game-side in `src/game/axmix_*`).
- `MSL/MetroTRK` → host CRT (drop or shim).

Build targets: `fzero-gx.exe` (Windows x64, MSVC 2022) alongside `build/GFZE01/main.dol` (matching). No source duplication — `#ifdef _WIN32` shims where needed, matching builds define `GC` path.

### Phase 3 — make it run

- Loader that maps DOL+REL address space into host process (or re-link as native exe).
- Resource extraction at runtime from original RVZ/ISO (user-supplied, as with matching build).
- Iterate to boot → menus → race. Profile + fix per-function mismatches that were fuzzy-matched but logically wrong.

## Upstream sync

`git fetch upstream` + `git merge upstream/main` into `dev` when upstream moves (currently at `4797463`). If upstream lands conversions we already have, the merge is trivial; if they pin a different `mw_version`, reconcile per `findings/109` style evidence (integrator discriminator logs).

## Next action

Branch `dev` created from `main` on D1, this doc committed. Conversions resume in priority order above, pushed to `origin/dev`.
