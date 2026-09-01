#pragma once
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
void gx_fifo_write(const uint8_t* data, size_t len);
uint64_t gx_fifo_draws(void);
uint64_t gx_fifo_cmds(void);
#ifdef __cplusplus
}
#endif
