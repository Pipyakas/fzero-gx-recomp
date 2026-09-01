# Windows roadmap — fzero-gx-recomp

Goal: playable native Windows exe for GFZE01 via static recompilation.

## Pipeline

```
orig/GFZE01/f-zero gx (usa).rvz (1.2 GiB, gitignored)
  -> sys/main.dol + files/files/fze.*.rel (14 RELs, 2.45 MB)
  -> DolRecomp --gamecube -> build/recomp_gen/generated/ (36 chunks, 144k instr)
  -> compile with ModernGekko runtime (CPU ctx, GX->Vulkan, OS/DVD/PAD HLE)
  -> host/fzero-gx.exe (Win32 + Vulkan WSI 960x540)
```

## Current state

- Host boots: WndProc + Vulkan swapchain (GX-blue present), DVD host FS probe, QPC tick HLE.
- DolRecomp builds native Windows (VS LLVM clang 19.1.5, vcvars64+Ninja); DOL emits clean.
- RELs: alignment error at 0x80500000 — next fix.
- Runtime wiring (ModernGekko link into host) is the current phase.

## Next phases

1. Fix REL emission (base/section alignment).
2. Build ModernGekko as host runtime library; link recomp C chunks.
3. Boot to title screen; input/audio bringup.
4. PGO + packaging (portable zip, Steam shortcut via host/add_steam_shortcut.py).
