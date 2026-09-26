// host/src/probe_main.c — headless probe runner (console, no window/Vulkan).
// Runs a fixed number of recomp slices, then exits. Usage:
//   fzero-probe.exe [dol_path] [slices]
#include <stdio.h>
#include <stdlib.h>
#include "../recomp_runner.h"

int main(int argc, char **argv) {
    const char *dol = argc > 1 ? argv[1] : "orig/GFZE01/sys/main.dol";
    long slices = argc > 2 ? atol(argv[2]) : 20000;
    if (!recomp_init(dol)) {
        printf("[probe] DOL load FAILED: %s\n", dol);
        return 1;
    }
    printf("[probe] ready pc=0x%08X slices=%ld\n", recomp_pc(), slices);
    for (long i = 0; i < slices; i++) {
        recomp_run_slice();
        recomp_flush_gp();
        if ((i % 2000) == 0) {
            printf("[probe] slice %ld pc=0x%08X\n", i, recomp_pc());
            fflush(stdout);
        }
    }
    printf("[probe] done pc=0x%08X frames=%u gp_bytes=%u mmio_r=%llu\n",
           recomp_pc(), recomp_frames(), recomp_gp_bytes(),
           (unsigned long long)recomp_mmio_reads());
    fflush(stdout);
    recomp_shutdown();
    return 0;
}
