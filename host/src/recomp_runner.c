// host/src/recomp_runner.c — load DOL into CPU RAM and tick it
#include "../recomp_runner.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../vendor/RingOut/DolRecomp/src/cpu/cpu.h"
#include "../../build/recomp_all/generated/generated.h"
#include "gx_fifo_bridge.h"
// GXRuntime chassis (C-only: bus + VI/PI interrupt model + retrace clock).
// NOTE: DolRecomp cpu.h and GXRuntime core/cpu.h share the DOLRECOMP_CPU_H
// guard, so whichever is included first wins in this TU. DolRecomp's is first
// here; the bus/interrupt/clock functions below only pass the CPUState*
// through opaquely (or never touch it), so the prefix-compatible ABI is safe.
#include "gxruntime/mmio_bus.h"
#include "gxruntime/interrupts.h"
#include "gxruntime/vi_clock.h"

static CPUState g_cpu;
static int g_inited = 0;
static u64 g_tb = 0;
static uint64_t s_mmio_reads=0, s_mmio_writes=0;
static uint32_t s_last_exc_pc=0, s_last_exc=0;
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
extern float gx_guest_frame_progress(void);

// --- GXRuntime VI/PI chassis state (PLAN M1->M3 core: real retrace clock +
// interrupt model instead of blanket MMIO ready bits). Declared before the
// external MMIO handlers so they can route through the bus. ---
// Timebase: Gekko runs at bus/4 (162MHz/4 = 40.5MHz), 60Hz VI cadence =>
// 675000 ticks per retrace. Fed per recomp block dispatch (see slice tail).
#define GC_TIMEBASE_HZ 40500000ull
#define VI_TICKS_PER_RETRACE (GC_TIMEBASE_HZ / 60ull)
#define MSR_EE 0x00008000u
// OSContext layout (dolphin/os/OSContext.h) for external-interrupt delivery.
#define CTX_GPR0_OFF 0x000u
#define CTX_CR_OFF 0x080u
#define CTX_LR_OFF 0x084u
#define CTX_CTR_OFF 0x088u
#define CTX_XER_OFF 0x08Cu
#define CTX_FPSCR_OFF 0x194u
#define CTX_SRR0_OFF 0x198u
#define CTX_SRR1_OFF 0x19Cu
#define CTX_STATE_OFF 0x1A2u
#define OS_CONTEXT_STATE_FPSAVED 0x0002u
#define EXC_EXTERNAL 4u // __OS_EXCEPTION_EXTERNAL_INTERRUPT
static DolMmioBus s_mmio_bus;
static DolInterrupts s_interrupts;
static DolViClock s_vi_clock;
static int s_chassis_inited = 0;
static uint64_t s_ext_deliveries = 0; // external-interrupt deliveries to guest
// The guest OS publishes its current thread's OSContext* at low-mem 0xD4.
// Strikers host uses the same address (STRIKERS_OS_CONTEXT_POINTER);
// it is a GameCube OS global, not game-specific.
#define GUEST_OS_CONTEXT_PTR_ADDR 0x800000D4u
static bool chassis_vi_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user; (void)cpu;
    if(value) *value = dol_interrupts_mmio_read(&s_interrupts, ea, size);
    return true;
}
static bool chassis_vi_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user; (void)cpu;
    dol_interrupts_mmio_write(&s_interrupts, ea, size, value);
    return true;
}
static void chassis_init(void){
    if(s_chassis_inited) return;
    dol_mmio_bus_init(&s_mmio_bus);
    dol_interrupts_init(&s_interrupts);
    dol_vi_clock_init(&s_vi_clock);
    dol_vi_clock_configure(&s_vi_clock, 1u, 60u, GC_TIMEBASE_HZ);
    // Route VI (0xCC002000 len 0x80) + PI (0xCC003000 len 0x40) + PE status
    // through the production interrupt model; every other device still
    // reports 0 via hle_external_* fallback below.
    dol_mmio_bus_register(&s_mmio_bus, 0xCC002000u, 0x80u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC003000u, 0x40u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC00100Au, 2u, chassis_vi_read, chassis_vi_write, NULL);
    s_chassis_inited = 1;
}
static uint32_t chassis_ctx_ptr(void){
    uint32_t a = GUEST_OS_CONTEXT_PTR_ADDR;
    if(a < GC_RAM_BASE || a + 4 > GC_RAM_BASE + g_cpu.ram_size) return 0;
    uint8_t* h = g_cpu.ram + (a - GC_RAM_BASE);
    return ((uint32_t)h[0]<<24)|((uint32_t)h[1]<<16)|((uint32_t)h[2]<<8)|h[3];
}
// One guest-block dispatch = one VI-clock work unit (Strikers transitional
// adapter). FRAME_WORK_UNITS blocks per 60Hz retrace; each due retrace adds
// 675000 timebase ticks and asserts the VI status bit. Interrupt delivery is
// parked until the F-Zero __OSDispatchInterrupt address is resolved — pending
// sources then stay pending until the guest acks them, matching
// level-triggered hardware.
#define CHASSIS_FRAME_WORK_UNITS 200000ull
static uint32_t s_os_dispatch_interrupt = 0; // 0 = parked; resolve F-Zero addr later
static void chassis_deliver_external(void){
    // Parked until the F-Zero dispatcher address is known. Retrace status +
    // timebase are still driven from the slice tail; nothing redirects pc here.
    (void)MSR_EE; (void)EXC_EXTERNAL;
    (void)chassis_ctx_ptr;
    if(!s_os_dispatch_interrupt) return;
    if(!(g_cpu.msr & MSR_EE)) return;
    if(!dol_interrupts_external_pending(&s_interrupts)) return;
    uint32_t ctx = chassis_ctx_ptr();
    if(ctx < GC_RAM_BASE || ctx + 0x1C4 > GC_RAM_BASE + g_cpu.ram_size) return;
    uint8_t* b = g_cpu.ram + (ctx - GC_RAM_BASE);
    for(int i=0;i<32;i++){ uint32_t v=g_cpu.gpr[i]; b[i*4+0]=(uint8_t)(v>>24); b[i*4+1]=(uint8_t)(v>>16); b[i*4+2]=(uint8_t)(v>>8); b[i*4+3]=(uint8_t)v; }
    uint32_t cr=g_cpu.cr, lr=g_cpu.lr, ctr=g_cpu.ctr, xer=g_cpu.xer, fpscr=g_cpu.fpscr;
    b[CTX_CR_OFF+0]=(uint8_t)(cr>>24); b[CTX_CR_OFF+1]=(uint8_t)(cr>>16); b[CTX_CR_OFF+2]=(uint8_t)(cr>>8); b[CTX_CR_OFF+3]=(uint8_t)cr;
    b[CTX_LR_OFF+0]=(uint8_t)(lr>>24); b[CTX_LR_OFF+1]=(uint8_t)(lr>>16); b[CTX_LR_OFF+2]=(uint8_t)(lr>>8); b[CTX_LR_OFF+3]=(uint8_t)lr;
    b[CTX_CTR_OFF+0]=(uint8_t)(ctr>>24); b[CTX_CTR_OFF+1]=(uint8_t)(ctr>>16); b[CTX_CTR_OFF+2]=(uint8_t)(ctr>>8); b[CTX_CTR_OFF+3]=(uint8_t)ctr;
    b[CTX_XER_OFF+0]=(uint8_t)(xer>>24); b[CTX_XER_OFF+1]=(uint8_t)(xer>>16); b[CTX_XER_OFF+2]=(uint8_t)(xer>>8); b[CTX_XER_OFF+3]=(uint8_t)xer;
    b[CTX_FPSCR_OFF+0]=(uint8_t)(fpscr>>24); b[CTX_FPSCR_OFF+1]=(uint8_t)(fpscr>>16); b[CTX_FPSCR_OFF+2]=(uint8_t)(fpscr>>8); b[CTX_FPSCR_OFF+3]=(uint8_t)fpscr;
    uint32_t pc=g_cpu.pc, msr=g_cpu.msr;
    b[CTX_SRR0_OFF+0]=(uint8_t)(pc>>24); b[CTX_SRR0_OFF+1]=(uint8_t)(pc>>16); b[CTX_SRR0_OFF+2]=(uint8_t)(pc>>8); b[CTX_SRR0_OFF+3]=(uint8_t)pc;
    b[CTX_SRR1_OFF+0]=(uint8_t)(msr>>24); b[CTX_SRR1_OFF+1]=(uint8_t)(msr>>16); b[CTX_SRR1_OFF+2]=(uint8_t)(msr>>8); b[CTX_SRR1_OFF+3]=(uint8_t)msr;
    b[CTX_STATE_OFF]=(uint8_t)(((uint16_t)((b[CTX_STATE_OFF]<<8)|b[CTX_STATE_OFF+1])|OS_CONTEXT_STATE_FPSAVED)>>8);
    b[CTX_STATE_OFF+1]=(uint8_t)(((uint16_t)((b[CTX_STATE_OFF]<<8)|b[CTX_STATE_OFF+1]))&0xFF);
    g_cpu.gpr[3]=EXC_EXTERNAL; g_cpu.gpr[4]=ctx;
    g_cpu.srr0=pc; g_cpu.srr1=msr; g_cpu.msr &= ~MSR_EE;
    g_cpu.pc=s_os_dispatch_interrupt; g_cpu.exception=0;
    s_ext_deliveries++;
    if(s_ext_deliveries<=4) fprintf(stderr,"[irq] external->dispatch 0x%08X ctx=0x%08X (delivery %llu)\n", s_os_dispatch_interrupt, ctx, (unsigned long long)s_ext_deliveries);
}

static uint64_t hle_external_read(CPUState* cpu, uint32_t addr, uint8_t size){
    s_mmio_reads++;
    // GXRuntime chassis owns VI/PI/PE; everything else still reports 0.
    u64 v = 0;
    if(dol_mmio_bus_read(&s_mmio_bus, cpu, addr, size, &v)) return v;
    if((addr&0xFF000000u)==0xCC000000u || (addr&0xFF000000u)==0xCD000000u) return 0;
    (void)cpu;(void)addr;(void)size;
    return 0;
}
static void hle_external_write(CPUState* cpu, uint32_t addr, uint64_t val, uint8_t size){
    s_mmio_writes++;
    if(dol_mmio_bus_write(&s_mmio_bus, cpu, addr, size, val)) return;
    (void)cpu;(void)addr;(void)val;(void)size;
}
static uint32_t hle_external_read32(CPUState* cpu, uint32_t addr, uint8_t rid){
    (void)rid; s_mmio_reads++;
    // eciwx/ecowx path: route through the VI/PI bus first (same model,
    // different access width). Strikers host returns 0 here; routing is a
    // strict superset that only answers for PI/VI/PE addresses.
    u64 v = 0;
    if(dol_mmio_bus_read(&s_mmio_bus, cpu, addr, 4, &v)) return (uint32_t)v;
    (void)cpu;(void)addr; return 0;
}
static void hle_external_write32(CPUState* cpu, uint32_t addr, uint32_t val, uint8_t rid){
    (void)cpu;(void)rid;(void)addr;(void)val; s_mmio_writes++;
}
static void hle_fallback(CPUState* cpu, uint32_t raw, uint32_t cia){
    uint32_t prim = raw>>26;
    uint32_t xo = (raw>>1)&0x3FFu;
    if(raw==0x4C00002Eu){ ppc_rfi(cpu, cia); if(cpu->exception==0) return; return; }
    // GameCube `sc` is a post-cache-op sync barrier (DCFlushRange ends with
    // `dcbf...; sc; blr`): the SDK system-call vector just syncs and returns.
    // Emulate by returning to the instruction after `sc` (Strikers host).
    if(prim==17u && (raw & 0x3Fu)==2u){ cpu->pc=cia+4u; return; }
    if(prim==31){
        if(xo==339){ uint32_t rt=(raw>>21)&31u; uint32_t spr=((raw>>11)&0x1Fu)<<5|((raw>>16)&0x1Fu); uint32_t before=cpu->exception; uint32_t v=ppc_mfspr(cpu,(uint16_t)spr,cia); if(cpu->exception==before){ cpu->gpr[rt]=v; cpu->pc=cia+4; return; } return; }
        if(xo==467){ uint32_t rs=(raw>>21)&31u; uint32_t spr=((raw>>11)&0x1Fu)<<5|((raw>>16)&0x1Fu); uint32_t before=cpu->exception; ppc_mtspr(cpu,(uint16_t)spr,cpu->gpr[rs],cia); if(cpu->exception==before){ cpu->pc=cia+4; return; } return; }
        if(xo==83){ uint32_t rt=(raw>>21)&31u; cpu->gpr[rt]=cpu->msr; cpu->pc=cia+4; return; }
        if(xo==146){ uint32_t rs=(raw>>21)&31u; cpu->msr=(cpu->gpr[rs]|0x2000u); cpu->pc=cia+4; return; }
        if(xo==210||xo==242||xo==595||xo==659){ uint32_t rt=(raw>>21)&31u; if(xo==595||xo==659) cpu->gpr[rt]=0; cpu->pc=cia+4; return; }
        if(xo==306){ uint32_t rb=(raw>>11)&31u; ppc_tlbie(cpu,cpu->gpr[rb],cia); if(cpu->exception==0) cpu->pc=cia+4; return; }
        if(xo==512||xo==854||xo==982||xo==470||xo==54||xo==86||xo==278||xo==246||xo==598||xo==150){ cpu->pc=cia+4; ppc_memory_fence(); return; }
        if(xo==1014){ uint32_t ra=(raw>>16)&31u, rb=(raw>>11)&31u; uint32_t ea=(ra?cpu->gpr[ra]:0)+cpu->gpr[rb]; ppc_dcbz_l(cpu,ea,cia); if(cpu->exception==0) cpu->pc=cia+4; return; }
        if(xo==19){ uint32_t rt=(raw>>21)&31u; cpu->gpr[rt]=cpu->cr; cpu->pc=cia+4; return; }
        if(xo==144){ cpu->pc=cia+4; return; }
    }
    fprintf(stderr,"[hle] fallback raw=0x%08X @0x%08X\n",raw,cia);
    ppc_program_exception(cpu, PPC_PROGRAM_ILLEGAL, cia);
}
static bool hle_host_call(CPUState* cpu, uint32_t addr){



    if(addr==0x800102ACu){
        if(cpu->ram_size >= 0x1200010u){
            uint32_t base=0x81200000u; uint32_t off=base - GC_RAM_BASE;
            if(off+16 <= cpu->ram_size){
                write_be32(cpu->ram + (0x80000038u - GC_RAM_BASE), base);
                write_be32(cpu->ram + (0x800000E4u - GC_RAM_BASE), base);
                write_be32(cpu->ram + off + 0, 0x01000000u);
                write_be32(cpu->ram + off + 4, 0u);
                write_be32(cpu->ram + off + 8, 0x00000001u);
                write_be32(cpu->ram + off + 12, 0u);
                cpu->gpr[3]=base;
                g_cpu.pc=g_cpu.lr & ~3u;
                return true;
            }
        }
        cpu->gpr[3]=0;
        g_cpu.pc=g_cpu.lr & ~3u;
        return true;
    }
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
    #define BE32(p) ((uint32_t)((p)[0]<<24|(p)[1]<<16|(p)[2]<<8|(p)[3]))
    uint32_t offs[18], addrs[18], sizes[18];
    for(int i=0;i<18;i++) offs[i]=BE32(buf+i*4);
    for(int i=0;i<18;i++) addrs[i]=BE32(buf+0x48+i*4);
    for(int i=0;i<18;i++) sizes[i]=BE32(buf+0x90+i*4);
    uint32_t bss_addr=BE32(buf+0xD8), bss_size=BE32(buf+0xDC), entry=BE32(buf+0xE0);
    if(bss_size){
        uint8_t* dst=NULL; uint32_t avail=0;
        if(bss_addr>=GC_RAM_BASE && bss_addr+bss_size<=GC_RAM_BASE+cpu->ram_size){ dst=cpu->ram+(bss_addr-GC_RAM_BASE); avail=cpu->ram_size-(bss_addr-GC_RAM_BASE); }
        else if(bss_addr>=GC_RAM_UNCACHED && bss_addr+bss_size<=GC_RAM_UNCACHED+cpu->ram_size){ dst=cpu->ram+(bss_addr-GC_RAM_UNCACHED); avail=cpu->ram_size-(bss_addr-GC_RAM_UNCACHED); }
        if(dst && avail>=bss_size) memset(dst,0,bss_size);
    }
    for(int i=0;i<18;i++){
        if(!sizes[i]) continue;
        uint32_t addr=addrs[i];
        uint8_t* dst=NULL; uint32_t avail=0;
        if(addr>=GC_RAM_BASE && addr<GC_RAM_BASE+cpu->ram_size){ dst=cpu->ram+(addr-GC_RAM_BASE); avail=cpu->ram_size-(addr-GC_RAM_BASE); }
        else if(addr>=GC_RAM_UNCACHED && addr<GC_RAM_UNCACHED+cpu->ram_size){ dst=cpu->ram+(addr-GC_RAM_UNCACHED); avail=cpu->ram_size-(addr-GC_RAM_UNCACHED); }
        if(!dst || avail < sizes[i]){ fprintf(stderr,"[recomp] DOL seg %d addr 0x%08X size 0x%X unmapped\n",i,addr,sizes[i]); free(buf); return 0; }
        if(offs[i]+sizes[i] > (uint32_t)sz){ free(buf); return 0; }
        memcpy(dst, buf+offs[i], sizes[i]);
    }
    #define POKE32(a,v) do{ uint32_t _a=(a); if(_a>=GC_RAM_BASE){ uint32_t _o=_a-GC_RAM_BASE; if(_o+4<=cpu->ram_size) write_be32(cpu->ram+_o,(v)); } }while(0)
    {
        uint32_t arena_lo = (bss_addr + bss_size + 0x20000u + 31u) & ~31u;
        if(arena_lo < GC_RAM_BASE || arena_lo > 0x817FEC60u) arena_lo = bss_addr ? bss_addr : GC_RAM_BASE;
        POKE32(0x80000020u, 0x0D15EA5Eu); POKE32(0x80000024u, 1u); POKE32(0x80000028u, 0x01800000u);
        POKE32(0x8000002Cu, 1u); POKE32(0x80000030u, arena_lo); POKE32(0x80000034u, 0x817FEC60u);
        POKE32(0x80000038u, 0u); POKE32(0x800000D4u, 0x80003000u); POKE32(0x8000003Cu, 0u); POKE32(0x800000CCu, 0u);
        POKE32(0x800000F8u, 0x09A7EC80u); POKE32(0x800000FCu, 0x1CF7C580u);
        cpu->gpr[1] = 0x817FFF00u; POKE32(0x817FFF00u, 0u); POKE32(0x817FFF04u, 0u);
    }
    #undef POKE32
    cpu->pc = entry;
    cpu->msr = 0x00002000u;
    cpu->hid2 |= 0xB0000000u;
    free(buf);
    printf("[recomp] DOL loaded entry=0x%08X pc=0x%08X bss 0x%08X/0x%X\n",entry,cpu->pc,bss_addr,bss_size);
    return 1;
}
int recomp_init(const char* dol_path) {
    if(g_inited) return 1;
    if(!cpu_init(&g_cpu)){ fprintf(stderr,"[recomp] cpu_init fail\n"); return 0; }
    chassis_init(); // VI/PI bus + retrace clock before first dispatch
    g_cpu.external_read = hle_external_read;
    g_cpu.external_write = hle_external_write;
    g_cpu.external_read32 = hle_external_read32;
    g_cpu.external_write32 = hle_external_write32;
    g_cpu.instruction_fallback = hle_fallback;
    g_cpu.host_call = hle_host_call;
    g_cpu.timebase = 0;
    if(!load_dol(dol_path, &g_cpu)){ cpu_free(&g_cpu); return 0; }
    s_gp_ptr = s_gp_buf; s_gp_base_ptr = s_gp_buf; s_gp_ptr_storage = s_gp_buf;
    s_gp_cursor_ref = &s_gp_ptr_storage;
    ppc_set_gather_pipe(&s_gp_ptr_storage, (uint8_t* const*)&s_gp_base_ptr, gp_flush, NULL, NULL);
    g_inited = 1;
    return 1;
}
void recomp_shutdown(void){ ppc_set_gather_pipe(NULL,NULL,NULL,NULL,NULL); if(g_inited){ cpu_free(&g_cpu); g_inited=0; } }
void recomp_flush_gp(void){ if(g_inited && s_gp_ptr != s_gp_base_ptr) gp_flush(NULL); }
static void poke16_set(uint32_t addr, uint16_t bits){ uint8_t* h=NULL; if(addr>=GC_RAM_BASE && addr<GC_RAM_BASE+g_cpu.ram_size) h=g_cpu.ram+(addr-GC_RAM_BASE); else if(addr>=GC_RAM_UNCACHED && addr<GC_RAM_UNCACHED+g_cpu.ram_size) h=g_cpu.ram+(addr-GC_RAM_UNCACHED); if(h){ uint16_t cur=(uint16_t)(h[0]<<8|h[1]); cur|=bits; h[0]=(uint8_t)(cur>>8); h[1]=(uint8_t)(cur&0xFF); } }
static void poke16_clr(uint32_t addr, uint16_t bits){ uint8_t* h=NULL; if(addr>=GC_RAM_BASE && addr<GC_RAM_BASE+g_cpu.ram_size) h=g_cpu.ram+(addr-GC_RAM_BASE); else if(addr>=GC_RAM_UNCACHED && addr<GC_RAM_UNCACHED+g_cpu.ram_size) h=g_cpu.ram+(addr-GC_RAM_UNCACHED); if(h){ uint16_t cur=(uint16_t)(h[0]<<8|h[1]); cur&=~bits; h[0]=(uint8_t)(cur>>8); h[1]=(uint8_t)(cur&0xFF); } }
static void poke32_set(uint32_t addr, uint32_t v){ uint8_t* h=NULL; if(addr>=GC_RAM_BASE && addr<GC_RAM_BASE+g_cpu.ram_size) h=g_cpu.ram+(addr-GC_RAM_BASE); else if(addr>=GC_RAM_UNCACHED && addr<GC_RAM_UNCACHED+g_cpu.ram_size) h=g_cpu.ram+(addr-GC_RAM_UNCACHED); if(h){ h[0]=v>>24; h[1]=(v>>16)&0xFF; h[2]=(v>>8)&0xFF; h[3]=v&0xFF; } }
static uint32_t s_last_pc=0; static int s_same=0;
// Walk the PPC EABI backchain: [sp] -> caller frame, [caller+4] -> ret addr.
// Bounded + RAM-checked; Strikers dump_backchain pattern.
static void log_backchain(void){
    u32 sp = g_cpu.gpr[1];
    fprintf(stderr,"[bt] pc=0x%08X lr=0x%08X r1=0x%08X msr=0x%08X ctr=0x%08X:",
        g_cpu.pc, g_cpu.lr, sp, g_cpu.msr, g_cpu.ctr);
    for(int f=0; f<8 && sp>=GC_RAM_BASE && sp+8u<GC_RAM_BASE+g_cpu.ram_size; f++){
        u32 off = sp - GC_RAM_BASE;
        u32 caller = ((u32)g_cpu.ram[off]<<24)|((u32)g_cpu.ram[off+1]<<16)|((u32)g_cpu.ram[off+2]<<8)|g_cpu.ram[off+3];
        if(caller<=sp || caller<GC_RAM_BASE || caller+8u>=GC_RAM_BASE+g_cpu.ram_size) break;
        u32 roff = caller - GC_RAM_BASE;
        u32 ret = ((u32)g_cpu.ram[roff+4]<<24)|((u32)g_cpu.ram[roff+5]<<16)|((u32)g_cpu.ram[roff+6]<<8)|g_cpu.ram[roff+7];
        fprintf(stderr," [0x%08X]", ret);
        sp = caller;
    }
    fputc('\n', stderr);
}
static uint64_t s_slice_n = 0;
void recomp_run_slice(void){
    if(!g_inited) return;
    g_cpu.timebase += 486000000ULL/240;
    for(int i=0;i<16384;i++){

        if(g_cpu.exception){
            uint32_t vec=g_cpu.pc;
            if(vec!=s_last_exc_pc || g_cpu.exception!=s_last_exc){
                fprintf(stderr,"[hle] exc 0x%X prog 0x%X msr=0x%08X hid2=0x%08X srr0 0x%08X srr1 0x%08X -> vec 0x%08X r1=0x%08X\n", g_cpu.exception, g_cpu.program_exception, g_cpu.msr, g_cpu.hid2, g_cpu.srr0, g_cpu.srr1, vec, g_cpu.gpr[1]);
                s_last_exc_pc=vec; s_last_exc=g_cpu.exception;
            }
            if(g_cpu.exception & PPC_EXC_FP_UNAVAILABLE){ g_cpu.msr|=0x2000u; g_cpu.srr1|=0x2000u; g_cpu.pc=g_cpu.srr0; g_cpu.exception=0; continue; }
            if(vec>=0x200 && vec<0xD00){ ppc_rfi(&g_cpu, vec); g_cpu.msr|=0x2000u; g_cpu.exception=0; continue; }
            if(g_cpu.exception & PPC_EXC_PROGRAM){ g_cpu.exception=0; break; }
            g_cpu.exception=0; break;
        }
        g_cpu.msr|=0x2000u; g_cpu.srr1|=0x2000u;
        g_cpu.msr|=0x2000u;
        g_cpu.srr1|=0x2000u;
        uint32_t pc=g_cpu.pc;
        if(g_cpu.gpr[1]==0) g_cpu.gpr[1]=0x817FFF00u;
        if(pc==0 || (pc>=0x80000000u && pc<0x80003100u)){
            uint32_t nxt=g_cpu.lr ? (g_cpu.lr & ~3u) : (g_cpu.srr0 ? (g_cpu.srr0 & ~3u) : 0x80003154u);
            if(nxt==pc || nxt==0) nxt=0x80003154u;
            g_cpu.pc=nxt; g_cpu.exception=0; continue;
        }
        if(g_cpu.downcount < -800) g_cpu.downcount += 1000;
        if(pc==s_last_pc) s_same++; else {s_last_pc=pc; s_same=0;}
        if(false && s_same>256){ if(g_cpu.ctr>0) g_cpu.ctr--; g_cpu.pc+=4; s_same=0; continue; }
        if(pc>=0x200 && pc<0xD00){ ppc_rfi(&g_cpu, pc); g_cpu.msr|=0x2000u; g_cpu.exception=0; continue; }
        if((pc==0x8000B670u || pc==0x8000B750u || pc==0x8000B7C4u) && g_cpu.ctr>4) g_cpu.ctr=1;
        else if(pc==0x80010608u && g_cpu.gpr[6]==0) g_cpu.gpr[6]=1;
        if(pc==0x800113B8u && g_cpu.ctr>8) g_cpu.ctr=1;
        else if(pc==0x800034E4u && g_cpu.gpr[3]>256u) g_cpu.gpr[3]=256u;
        else if(pc==0x80011424u) g_cpu.timebase += 5000;
        if(pc==0x8000B450u) poke16_set(g_cpu.gpr[31], 0x0020u);
        else if(pc==0x8000B498u) poke16_set(g_cpu.gpr[31], 0x0020u);
        else if(pc==0x8000B4B4u) poke16_clr(g_cpu.gpr[31], 0x0400u);
        else if(pc==0x8000B4D4u) poke16_set(g_cpu.gpr[30], 0x8000u);
        else if(pc==0x8000B3ECu) poke16_clr(g_cpu.gpr[31], 0x0001u);
        else if(pc==0x8000B508u) poke16_clr(g_cpu.gpr[31], 0x0001u);
        else if(pc==0x8001BD10u){ poke32_set(g_cpu.gpr[13]-31352u, 0u); poke32_set(g_cpu.gpr[13]-31348u, 0u); }
        else if(pc==0x8000B578u) poke16_clr(g_cpu.gpr[31], 0x0001u);
        else if(pc==0x8001071Cu) poke32_set(g_cpu.gpr[13]-31688u, 1u);
        else if(pc==0x8001072Cu) poke32_set(g_cpu.gpr[13]-31688u, 1u);

        else if(pc==0x80033614u) g_cpu.gpr[0]=255;
        else if(pc==0x800332FCu){ uint32_t a=g_cpu.gpr[4]; if(a>=0x80000000u && a+4 < 0x80000000u+g_cpu.ram_size) write_be32(g_cpu.ram+(a-0x80000000u), 255u); }

        else if(pc==0x8000D5E0u){ g_cpu.pc=g_cpu.lr & ~3u; }
        else if(pc==0x800858E4u) g_cpu.gpr[3]=1;
        else if(pc==0x800333C8u) g_cpu.gpr[11]=0;
        else if(pc==0x8000A990u) g_cpu.gpr[26]=16;
        else if(pc==0x80010718u) poke32_set(g_cpu.gpr[13]-31688u, 1u);
        else if(pc==0x80010624u && g_cpu.gpr[6]==0) g_cpu.gpr[6]=1;
        else if(pc==0x8000BE60u) write_be32(g_cpu.ram + (0x800000D4u - GC_RAM_BASE), 0x80003000u);
        else if(pc==0x80011160u) poke32_set(g_cpu.gpr[13]-31684u, 1u);

        else if(pc==0x8001AF8Cu){ uint32_t v=g_cpu.gpr[30]; if(v==0) v=1; poke32_set(g_cpu.gpr[13]-31388u, v); }
        // GXRuntime VI retrace drive: 1 block dispatch = 1 work unit.
        // Retrace => +675000 timebase ticks + VI status bit asserted.
        // Interrupt delivery stays parked (s_os_dispatch_interrupt==0) so
        // VIWaitForRetrace-style loops observe level-triggered pending.
        dol_vi_clock_advance(&s_vi_clock, 1u);
        {
            u64 ticks = 0;
            while(dol_vi_clock_pop_retrace(&s_vi_clock, &ticks)){
                g_cpu.timebase += ticks;
                dol_interrupts_assert_vi_retrace(&s_interrupts);
            }
            chassis_deliver_external();
        }
        if(!dolrecomp_call(&g_cpu, pc)){
            if(g_cpu.exception==0){
                static int miss_cnt=0;
                if(miss_cnt<6) fprintf(stderr,"[hle] miss pc=0x%08X lr=0x%08X r1=0x%08X\n",pc,g_cpu.lr,g_cpu.gpr[1]);
                miss_cnt++;
            }
            break;
        }
        if(++s_slice_n % (16384ull*75ull) == 0) log_backchain(); // ~75 slices
    }
}
unsigned recomp_pc(void){ return g_cpu.pc; }
int recomp_inited(void){ return g_inited; }
uint32_t recomp_gp_bytes(void){ return (uint32_t)(s_gp_ptr - s_gp_base_ptr); }
uint32_t recomp_frames(void){ return s_frames; }
uint64_t recomp_mmio_reads(void){ return s_mmio_reads; }
