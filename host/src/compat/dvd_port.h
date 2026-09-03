// dvd_port.h — force-include shim for GXRuntime dvd.c on Windows UCRT.
//
// dvd.c uses POSIX fseeko()/off_t, which the Windows UCRT does not declare
// (it provides _fseeki64/__int64 instead). Redefining fseeko on the command
// line breaks <stdio.h> itself, so this header is force-included (-include)
// AFTER system headers: it maps the two POSIX spellings used by dvd.c.
// Included only for the dvd.c TU via set_source_files_properties.
#pragma once
#if defined(_WIN32)
#include <stdio.h>
#ifdef fseeko
#undef fseeko
#endif
#define fseeko(f, off, wh) _fseeki64(f, (long long)(off), wh)
#ifndef off_t
typedef long long dvd_port_off_t;
#define off_t dvd_port_off_t
#endif
#endif
