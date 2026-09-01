// host/src/gx_null.c -- null GX renderer shim (Phase 2).
// All GX/GD calls become no-ops until a real GL/Vulkan backend lands.
#include <stddef.h>
void GXBegin(void) {}
void GXInit(void) {}
void GXSetLineWidth(unsigned char w, unsigned char fmt) {(void)w;(void)fmt;}
void GXSetPointSize(unsigned char s, unsigned char fmt) {(void)s;(void)fmt;}
