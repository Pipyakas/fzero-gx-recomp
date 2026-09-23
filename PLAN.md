# F-Zero GX — Revisited Playable Path

Goal: parity with Dolphin (not white screen). White `pc=0x80000000 draws=0` is architectural failure, not register value.

## Diagnosis
Current runner violates guest semantics: invented MMIO `0xFFFE`, per-PC `poke*`, `sc`=nop-then-branch, guessed `LR` returns, forced `MSR.FP`, swallowed exceptions. REL chunks compiled with synthetic `0x8050xxxx` bases but never wired to `OSLink` runtime placement; SMC/code-patching never invalidates chunks.

## Verifiable Milestones (no PC-guessing)
1. **M0 Deterministic harness** — same RAM+seed → same `pc`/hash at first exception; every miss/MMIO logged.
2. **M1 DOL-static+interp parity** — interpreter-only vs DOL-static identical hashes through `OSInit`; stale chunks demoted.
3. **M2 REL via interpreter** — `fze.title.rel` loaded by guest `OSLink`, prolog runs at runtime-allocated addr (not `0x805...` constant).
4. **M3 First XFB** — FIFO via production GX (CP/XF/BP+textures+EFB/XFB), frame hash matches reference.
5. **M4 PAD** — scripted input passes title deterministically.
6. **M5 Race** — AI/DSP/DMA/interrupts scheduled; audio on/off identical guest state.
7. **M6 Native RELs** — per-module toggle identical hashes; reload invalidates dispatch.

## Immediate Fixes (this slice)
- Runner: BSS-before-data (already), arena `0x817FEC60` + `0x817FFF00` stack, `hid2|=LCE`, `msr=FP`, `sc` as `cia+4` sync barrier (matches Strikers host), vector `rfi` only at `0x200/0xD00`, no PC-skips/guesses.
- CMake: `build/recomp_all/generated/chunks` (not `dol/generated`), both include dirs.
- Remove: `poke*` table, `0xCC00→0xFFFE`, timebase hacks, `ctr` clamps, `34E4/B470` text patches.

## Next Integration (ModernGekko/GXRuntime chassis, DOL-only first)
- Adopt `dol_load_into_ram` + `boot_setup_os_globals` + `mmio_bus` + `vi_clock` + `interrupts` + `hle_core/hle_dvd/hle_input` + `gx_recomp` instead of custom `dvd_host.c`/`gx_fifo_bridge` stubs.
- Do NOT compile REL chunks into `recomp_core`; run `0x8050+` via interpreter with real `OSLink` relocation.
- Route `0xCC008000` gather pipe + `VIConfigure/GXCopyDisp` through production path; bridge only as trace validator.
- Fix DolRecomp REL frontend: module-0 imports, REL24 overflow thunk, aligned section packing, per-module dispatch metadata.

## Anti-patterns Banned
Per-PC memory pokes, blanket MMIO ready bits, wall-clock `timebase` bumps, blind `rfi`, `chunk_*.c` text edits as boot fix, recursive REL `GLOB`, checkout-absolute `DVDOpen` paths, expanding parser as GX renderer.
