// host/src/recomp_runner.c — load DOL into CPU RAM and tick it
#include "../recomp_runner.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../vendor/RingOut/DolRecomp/src/cpu/cpu.h"
#include "../../build/recomp_all/generated/dol/generated/generated.h"
#include "gx_fifo_bridge.h"

static CPUState g_cpu;
static int g_inited = 0;
static u64 g_tb = 0;
static uint64_t s_mmio_reads=0, s_mmio_writes=0;
static uint32_t s_last_exc_pc=0, s_last_exc=0;
// GX gather pipe -> Vulkan: guest stores to 0xCC008000 land here
#define GP_SIZE (128*1024)
static uint8_t s_gp_buf[GP_SIZE + 64];
static uint8_t *s_gp_ptr = s_gp_buf;
static uint8_t *s_gp_base_ptr = s_gp_buf;
static uint8_t *s_gp_ptr_storage = NULL;
static uint8_t **s_gp_cursor_ref = NULL;
static uint64_t s_gp_flushes=0, s_gp_bytes=0;
static uint32_t s_frames=0;
static void gp_flush(void* u){ (void)u; size_t n=(size_t)(s_gp_ptr - s_gp_base_ptr); if(n){ gx_fifo_write(s_gp_base_ptr, n); s_gp_bytes += (uint64_t)n; if(n>64) s_frames++; } s_gp_flushes++; s_gp_ptr = s_gp_base_ptr; if(s_gp_cursor_ref) *s_gp_cursor_ref = s_gp_ptr; }
extern void ppc_set_gather_pipe(uint8_t** cursor, uint8_t* const* base, void (*flush)(void*), void* user, const unsigned char* bypass);
extern float gx_guest_frame_progress(void); // provided by gx_vulkan

static uint64_t hle_external_read(CPUState* cpu, uint32_t addr, uint8_t size){
    s_mmio_reads++;
    if(s_mmio_reads<8) fprintf(stderr,"[hle] read%u 0x%08X\n",size,addr);
    (void)cpu;
    // DVD/DSP/GX regs return 0; keep guest moving
    if((addr&0xFF000000u)==0xCC000000u || (addr&0xFF000000u)==0xCD000000u) return 0;
    return 0;
}
static void hle_external_write(CPUState* cpu, uint32_t addr, uint64_t val, uint8_t size){
    s_mmio_writes++;
    if(s_mmio_writes<8) fprintf(stderr,"[hle] write%u 0x%08X <- 0x%llX\n",size,addr,(unsigned long long)val);
    (void)cpu;
}
static uint32_t hle_external_read32(CPUState* cpu, uint32_t addr, uint8_t rid){
    (void)cpu;(void)rid; s_mmio_reads++; return 0;
}
static void hle_external_write32(CPUState* cpu, uint32_t addr, uint32_t val, uint8_t rid){
    (void)cpu;(void)rid; s_mmio_writes++;
}
static void hle_fallback(CPUState* cpu, uint32_t raw, uint32_t cia){
    fprintf(stderr,"[hle] fallback raw=0x%08X @0x%08X\n",raw,cia);
    ppc_program_exception(cpu, PPC_PROGRAM_ILLEGAL, cia);
}
static bool hle_host_call(CPUState* cpu, uint32_t addr){
    (void)cpu;(void)addr;
    // No SDK patch yet — let dolrecomp_call_original handle it
    return false;
}

static int load_dol(const char* path, CPUState* cpu) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "[recomp] open DOL failed: %s\n", path); return 0; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = (uint8_t*)malloc(sz);
    if (!buf) { fclose(f); return 0; }
    if ((long)fread(buf,1,sz,f) != sz) { free(buf); fclose(f); return 0; }
    fclose(f);
    if (sz < 0x100) { free(buf); return 0; }
    // DOL header BE: 18 offsets at 0x00, 18 addrs at 0x48, 18 sizes at 0x90, bss at 0xD8, entry at 0xE0
    #define BE32(p) ((uint32_t)((p)[0]<<24|(p)[1]<<16|(p)[2]<<8|(p)[3]))
    uint32_t offs[18], addrs[18], sizes[18];
    for(int i=0;i<18;i++) offs[i]=BE32(buf+i*4);
    for(int i=0;i<18;i++) addrs[i]=BE32(buf+0x48+i*4);
    for(int i=0;i<18;i++) sizes[i]=BE32(buf+0x90+i*4);
    uint32_t bss_addr=BE32(buf+0xD8), bss_size=BE32(buf+0xDC), entry=BE32(buf+0xE0);
    (void)bss_addr; (void)bss_size;
    for(int i=0;i<18;i++){
        if(!sizes[i]) continue;
        uint32_t addr=addrs[i];
        uint8_t* dst=NULL; uint32_t avail=0;
        if(addr>=GC_RAM_BASE && addr<GC_RAM_BASE+cpu->ram_size){ dst=cpu->ram+(addr-GC_RAM_BASE); avail=cpu->ram_size-(addr-GC_RAM_BASE); }
        else if(addr>=GC_RAM_UNCACHED && addr<GC_RAM_UNCACHED+cpu->ram_size){ dst=cpu->ram+(addr-GC_RAM_UNCACHED); avail=cpu->ram_size-(addr-GC_RAM_UNCACHED); }
        if(!dst || avail < sizes[i]){
            fprintf(stderr,"[recomp] DOL seg %d addr 0x%08X size 0x%X unmapped\n",i,addr,sizes[i]);
            free(buf); return 0;
        }
        if(offs[i]+sizes[i] > (uint32_t)sz){ free(buf); return 0; }
        memcpy(dst, buf+offs[i], sizes[i]);
    }
    if(bss_size){
        uint8_t* dst=NULL; uint32_t avail=0;
        if(bss_addr>=GC_RAM_BASE && bss_addr+bss_size<=GC_RAM_BASE+cpu->ram_size){ dst=cpu->ram+(bss_addr-GC_RAM_BASE); avail=cpu->ram_size-(bss_addr-GC_RAM_BASE); }
        else if(bss_addr>=GC_RAM_UNCACHED && bss_addr+bss_size<=GC_RAM_UNCACHED+cpu->ram_size){ dst=cpu->ram+(bss_addr-GC_RAM_UNCACHED); avail=cpu->ram_size-(bss_addr-GC_RAM_UNCACHED); }
        if(dst && avail>=bss_size) memset(dst,0,bss_size);
    }
    cpu->pc = entry;
    // sane MSR: FP enabled, EE? keep as game left
    cpu->msr = 0x00002000u; // FP
    free(buf);
    printf("[recomp] DOL loaded entry=0x%08X pc=0x%08X bss 0x%08X/0x%X\n",entry,cpu->pc,bss_addr,bss_size);
    return 1;
}

int recomp_init(const char* dol_path) {
    if(g_inited) return 1;
    if(!cpu_init(&g_cpu)){ fprintf(stderr,"[recomp] cpu_init fail\n"); return 0; }
    g_cpu.external_read = hle_external_read;
    g_cpu.external_write = hle_external_write;
    g_cpu.external_read32 = hle_external_read32;
    g_cpu.external_write32 = hle_external_write32;
    g_cpu.instruction_fallback = hle_fallback;
    g_cpu.host_call = hle_host_call;
    g_cpu.timebase = 0;
    if(!load_dol(dol_path, &g_cpu)){ cpu_free(&g_cpu); return 0; }
    // install gather pipe so 0xCC008000 stores land in s_gp_buf instead of external_write
    s_gp_ptr = s_gp_buf; s_gp_base_ptr = s_gp_buf; s_gp_ptr_storage = s_gp_buf;
    s_gp_cursor_ref = &s_gp_ptr_storage;
    ppc_set_gather_pipe(&s_gp_ptr_storage, (uint8_t* const*)&s_gp_base_ptr, gp_flush, NULL, NULL);
    g_inited = 1;
    return 1;
}
void recomp_shutdown(void){ ppc_set_gather_pipe(NULL,NULL,NULL,NULL,NULL); if(g_inited){ cpu_free(&g_cpu); g_inited=0; } }
void recomp_run_slice(void){
    if(!g_inited) return;
    g_cpu.timebase += 486000000ULL/240;
    // simple DEC handling so guest OS tick doesn't spin forever
    for(int i=0;i<4096;i++){
        if(g_cpu.exception){
            if(g_cpu.pc!=s_last_exc_pc || g_cpu.exception!=s_last_exc){
                fprintf(stderr,"[hle] exception 0x%X pc=0x%08X -> vec 0x%08X\n", g_cpu.exception, s_last_exc_pc, g_cpu.pc);
                s_last_exc_pc=g_cpu.pc; s_last_exc=g_cpu.exception;
            }
            // clear and keep ticking — lets us see if guest recovers or loops on same vector
            g_cpu.exception=0;
        }
        uint32_t pc=g_cpu.pc;
        if(!dolrecomp_call(&g_cpu, pc)){
            if(g_cpu.exception==0) fprintf(stderr,"[hle] dolrecomp_call miss pc=0x%08X\n",pc);
            break;
        }
    }
}
unsigned recomp_pc(void){ return g_cpu.pc; }
int recomp_inited(void){ return g_inited; }
uint32_t recomp_gp_bytes(void){ return (uint32_t)(s_gp_ptr - s_gp_base_ptr); }
uint32_t recomp_frames(void){ return s_frames; }
uint64_t recomp_mmio_reads(void){ return s_mmio_reads; }
