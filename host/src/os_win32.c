// host/src/os_win32.c — OS/VI/AI stubs on Win32.
#include <stdio.h>
#include <windows.h>

static LARGE_INTEGER g_freq;
static int g_has_freq = 0;

void OSInit(void) {
    if (!g_has_freq) { QueryPerformanceFrequency(&g_freq); g_has_freq = 1; }
    printf("[os_win32] OSInit freq=%lld\n", (long long)g_freq.QuadPart);
}
void VIInit(void) { printf("[os_win32] VIInit (60Hz via WSI FIFO)\n"); }
void AIInit(void* p) { (void)p; printf("[os_win32] AIInit (null audio)\n"); }
long long OSGetTick(void) { LARGE_INTEGER c; QueryPerformanceCounter(&c); return c.QuadPart; }
long long OSGetTickFreq(void) { if (!g_has_freq) OSInit(); return g_freq.QuadPart; }
