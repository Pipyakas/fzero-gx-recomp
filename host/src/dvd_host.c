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
// FST blob writer (see recomp_runner.c hle_host_call 0x800102AC).
// Walks <root>/files recursively, writes N 12-byte FST entries + string table
// at guest `base` (0x81200000). ram = guest MEM1 base, ram_size in bytes.
// Returns entry count, or <=0 when the tree/blob doesn't fit.
#include <stdint.h>
#define DVD_GUEST_BASE 0x80000000u
static void wbe32(uint8_t* p, uint32_t v){ p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
typedef struct { char rel[260]; int is_dir; unsigned size; } DvdWalkEnt;
static DvdWalkEnt s_walk[4096]; static int s_walk_n;
static char s_walk_str[1<<20]; static unsigned s_walk_strlen;
static unsigned walk_add_str(const char* s){
    unsigned off = s_walk_strlen;
    unsigned n = (unsigned)strlen(s)+1;
    if(off+n > sizeof(s_walk_str)) return 0;
    memcpy(s_walk_str+off, s, n); s_walk_strlen += n;
    return off;
}
static void walk_dir(const char* root, const char* rel){
    char path[MAX_PATH];
    // Extracted tree is <root>/files/{files,sys}/... (double-nested): the
    // disc-root FST starts at <root>/files, so rel paths are relative to it.
    // (Earlier revision wrongly descended <root>/files/files, doubling the
    // first component: first=files.)
    if(rel[0]) snprintf(path, sizeof(path), "%s\\files\\%s", root, rel);
    else snprintf(path, sizeof(path), "%s\\files", root);
    for(char* c=path; *c; c++) if(*c=='/') *c='\\';
    WIN32_FIND_DATAA fd; HANDLE h;
    char pat[MAX_PATH]; snprintf(pat, sizeof(pat), "%s\\*", path);
    h = FindFirstFileA(pat, &fd);
    if(h == INVALID_HANDLE_VALUE) return;
    // collect names first for deterministic (sorted) order
    char names[1024][64]; int nn = 0;
    do {
        if(!strcmp(fd.cFileName,".") || !strcmp(fd.cFileName,"..")) continue;
        if(nn < 1024){ strncpy(names[nn], fd.cFileName, 63); names[nn][63]=0; nn++; }
    } while(FindNextFileA(h, &fd));
    FindClose(h);
    for(int i=0;i<nn;i++) for(int j=i+1;j<nn;j++) if(strcmp(names[i],names[j])>0){
        char t[64]; strcpy(t,names[i]); strcpy(names[i],names[j]); strcpy(names[j],t);
    }
    for(int i=0;i<nn && s_walk_n < (int)(sizeof(s_walk)/sizeof(s_walk[0]));i++){
        char child[260];
        if(rel[0]) snprintf(child, sizeof(child), "%s/%s", rel, names[i]);
        else snprintf(child, sizeof(child), "%s", names[i]);
        char fpath[MAX_PATH]; snprintf(fpath, sizeof(fpath), "%s\\files\\%s", root, child);
        for(char* c=fpath; *c; c++) if(*c=='/') *c='\\';
        DWORD a = GetFileAttributesA(fpath);
        int is_dir = (a != INVALID_FILE_ATTRIBUTES) && (a & FILE_ATTRIBUTE_DIRECTORY);
        DvdWalkEnt* e = &s_walk[s_walk_n++];
        strncpy(e->rel, child, sizeof(e->rel)-1); e->rel[sizeof(e->rel)-1]=0;
        e->is_dir = is_dir; e->size = 0;
        if(!is_dir && a != INVALID_FILE_ATTRIBUTES){
            HANDLE fh = CreateFileA(fpath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
            if(fh != INVALID_HANDLE_VALUE){ e->size = GetFileSize(fh, NULL); CloseHandle(fh); }
        }
        if(is_dir) walk_dir(root, child);
    }
}
int dvd_build_fst_from_tree(uint8_t* ram, unsigned ram_size, unsigned base){
    init_root();
    // Prefer the extractor's real fst.bin (byte-exact, includes disc offsets
    // the tree walk can't reconstruct). Fall back to the tree walk only when
    // fst.bin is missing/unreadable.
    {
        char fpath[MAX_PATH]; snprintf(fpath, sizeof(fpath), "%s\\files\\sys\\fst.bin", g_root);
        FILE* f = fopen(fpath, "rb");
        if(f){
            fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
            if(sz > 12 && base >= DVD_GUEST_BASE && base - DVD_GUEST_BASE + (unsigned)sz <= ram_size){
                size_t got = fread(ram + (base - DVD_GUEST_BASE), 1, (size_t)sz, f);
                fclose(f);
                if(got == (size_t)sz){
                    uint8_t* b = ram + (base - DVD_GUEST_BASE);
                    unsigned n = ((unsigned)b[8]<<24)|((unsigned)b[9]<<16)|((unsigned)b[10]<<8)|b[11];
                    { static int _l=0; if(_l<1){ fprintf(stderr,"[dvd] real fst.bin %u entries (%ldB) @0x%08X\n", n, sz, base); _l++; } }
                    return (int)n;
                }
            } else fclose(f);
        }
    }
    s_walk_n = 0; s_walk_strlen = 0;
    walk_add_str("");
    walk_dir(g_root, "");
    int ndirs = 0, nfiles = 0;
    for(int i=0;i<s_walk_n;i++){ if(s_walk[i].is_dir) ndirs++; else nfiles++; }
    // entries: [0]=root dir + one per walk entry, in walk (preorder) order
    int nentries = 1 + s_walk_n;
    unsigned need = (unsigned)nentries*12u + s_walk_strlen + 64u;
    if(base < DVD_GUEST_BASE || base - DVD_GUEST_BASE + need > ram_size) return -1;
    if(nentries < 2){ return 0; }
    uint8_t* dst = ram + (base - DVD_GUEST_BASE);
    // string offsets: name = after last '/' (dirs keep full rel for prefix match)
    unsigned* stroff = (unsigned*)malloc(sizeof(unsigned)*(size_t)nentries);
    if(!stroff) return -1;
    // entry 0 root: name off 0
    // children string offsets assigned in walk order
    for(int i=0;i<s_walk_n;i++){
        const char* rel = s_walk[i].rel;
        const char* nm = strrchr(rel, '/');
        nm = nm ? nm+1 : rel;
        stroff[1+i] = walk_add_str(nm);
    }
    // second pass: dir child ranges. dirs in walk order; a dir's children are
    // the run of following entries until depth returns. Track via rel prefix.
    // entry index map: walk[i] -> 1+i.
    // root spans all.
    wbe32(dst+0, 0x01000000u); wbe32(dst+4, 0u); wbe32(dst+8, (uint32_t)nentries);
    for(int i=0;i<s_walk_n;i++){
        uint8_t* e = dst + (size_t)(1+i)*12u;
        const char* rel = s_walk[i].rel;
        if(s_walk[i].is_dir){
            // next index after this dir's subtree: first walk[j] not under rel+"/"
            size_t rl = strlen(rel);
            int j = i+1;
            while(j < s_walk_n){
                const char* r2 = s_walk[j].rel;
                if(strlen(r2) > rl && !memcmp(r2, rel, rl) && r2[rl]=='/') j++;
                else break;
            }
            unsigned parent = 0;
            // parent = nearest earlier dir whose rel is a prefix, else root
            for(int k=i-1;k>=0;k--) if(s_walk[k].is_dir){
                const char* rp = s_walk[k].rel;
                size_t pl = strlen(rp);
                if(pl==0 || (strlen(rel)>pl && !memcmp(rel,rp,pl) && rel[pl]=='/')){ parent = (unsigned)(1+k); break; }
            }
            wbe32(e+0, 0x01000000u | (stroff[1+i] & 0xFFFFFFu));
            wbe32(e+4, parent);
            wbe32(e+8, (uint32_t)(1+j));
        } else {
            wbe32(e+0, (stroff[1+i] & 0xFFFFFFu));
            wbe32(e+4, 0u); // filepos unknown (extracted tree has no disc offsets); reader uses size only
            wbe32(e+8, s_walk[i].size);
        }
    }
    memcpy(dst + (size_t)nentries*12u, s_walk_str, s_walk_strlen);
    free(stroff);
    { static int _l=0; if(_l<1){ fprintf(stderr,"[dvd] FST %d entries (%d dirs, %d files) @0x%08X strings=%uB first=%s\n", nentries, ndirs, nfiles, base, s_walk_strlen, s_walk_n?s_walk[0].rel:"(empty)"); _l++; } }
    return nentries;
}
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
