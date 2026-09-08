// msvc_compat.h — force-included (/FI) for MSVC builds only (see CMakeLists).
//
// DolRecomp cpu.c + GXRuntime headers use GCC/Clang spellings:
//   __attribute__((visibility/always_inline/destructor)), __builtin_bswap*,
//   <stdatomic.h> atomic_thread_fence. MSVC cl.exe rejects all of these.
// The harness is single-threaded, so the seq_cst fence is a nop here.
#pragma once
#ifdef _MSC_VER
#include <stdlib.h>
#include <stdio.h>
// dvd.c uses POSIX fseeko()/off_t; UCRT has _fseeki64/__int64 instead.
#ifdef fseeko
#undef fseeko
#endif
#define fseeko(f, off, wh) _fseeki64(f, (long long)(off), wh)
#ifndef off_t
typedef long long off_t;
#endif
#define __attribute__(x)
#define __builtin_bswap16(v) _byteswap_ushort(v)
#define __builtin_bswap32(v) _byteswap_ulong(v)
#define __builtin_bswap64(v) _byteswap_uint64(v)
// NOTE: atomic_thread_fence is provided by the shadow stdatomic.h in this
// dir, NOT macro-defined here (a macro would break that header's definition).
#endif
