// Minimal <stdatomic.h> shadow for MSVC (compat include dir is only on the
// path for MSVC builds — see CMakeLists). The harness is single-threaded:
// the fence is a nop. Only what cpu.c uses is provided.
#pragma once
#ifdef _MSC_VER
typedef enum memory_order {
    memory_order_relaxed,
    memory_order_consume,
    memory_order_acquire,
    memory_order_release,
    memory_order_acq_rel,
    memory_order_seq_cst
} memory_order;
#define atomic_thread_fence(order) ((void)0)
#endif
