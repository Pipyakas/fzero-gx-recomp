#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int recomp_init(const char* dol_path);
void recomp_shutdown(void);
void recomp_run_slice(void);
unsigned recomp_pc(void);
int recomp_inited(void);
uint32_t recomp_gp_bytes(void);
uint32_t recomp_frames(void);
uint64_t recomp_mmio_reads(void);
#ifdef __cplusplus
}
#endif
