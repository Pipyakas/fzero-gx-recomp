// host/src/dvd_host.c — DVD -> host FS (orig/GFZE01).
#include <stdio.h>
#include <string.h>
#include <windows.h>

static char g_root[MAX_PATH] = {0};
static int g_inited = 0;

static void init_root(void) {
    if (g_inited) return;
    char exe[MAX_PATH] = {0};
    GetModuleFileNameA(NULL, exe, MAX_PATH);
    char *p = strrchr(exe, '\\');
    if (p) *p = '\0';
    const char *candidates[] = {
        "..\\..\\..\\orig\\GFZE01",
        "..\\..\\orig\\GFZE01",
        "..\\orig\\GFZE01",
        "orig\\GFZE01",
        "C:\\code\\fzero-gx-recomp\\orig\\GFZE01",
        NULL
    };
    for (int i = 0; candidates[i]; i++) {
        char test[MAX_PATH];
        snprintf(test, sizeof(test), "%s\\%s\\sys\\main.dol", exe, candidates[i]);
        if (GetFileAttributesA(test) != INVALID_FILE_ATTRIBUTES) {
            snprintf(g_root, sizeof(g_root), "%s\\%s", exe, candidates[i]);
            g_inited = 1; return;
        }
        snprintf(test, sizeof(test), "%s\\sys\\main.dol", candidates[i]);
        if (GetFileAttributesA(test) != INVALID_FILE_ATTRIBUTES) {
            strncpy(g_root, candidates[i], sizeof(g_root)-1);
            g_inited = 1; return;
        }
    }
    strncpy(g_root, "orig\\GFZE01", sizeof(g_root)-1);
    g_inited = 1;
}

void DVDInit(void) { init_root(); printf("[dvd_host] root=%s\n", g_root); }
int DVDOpen(const char* path, void* entry) {
    (void)entry; init_root();
    if (!path) return 0;
    char host[MAX_PATH];
    snprintf(host, sizeof(host), "%s\\%s", g_root, path);
    for (char *c = host; *c; c++) if (*c == '/') *c = '\\';
    DWORD a = GetFileAttributesA(host);
    if (a == INVALID_FILE_ATTRIBUTES) { printf("[dvd_host] miss: %s -> %s\n", path, host); return 0; }
    printf("[dvd_host] open: %s -> %s\n", path, host);
    return 1;
}
const char* DVDHostRoot(void) { init_root(); return g_root; }
