#pragma once
#ifdef __cplusplus
extern "C" {
#endif
int recomp_init(const char* dol_path);
void recomp_shutdown(void);
void recomp_run_slice(void); // run ~1k blocks
unsigned recomp_pc(void);
int recomp_inited(void);
#ifdef __cplusplus
}
#endif
