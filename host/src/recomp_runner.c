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
#include "gxruntime/di.h"
#include "gxruntime/dvd.h"
#include "gxruntime/si.h"
// Local callback trampoline (host/src/hle_callback.c), extracted from
// GXRuntime hle_core.c — hle_core.c itself can't link here (newer CPUState
// + card/ARAM/platform deps). Keep hle_abi.h out too (guest_memory dep);
// this TU only needs the four callback functions + the return sentinel.
#define HLE_CALLBACK_RETURN 0x7FFF0000u
bool dol_hle_queue_guest_callback(u32 address, s32 channel, s32 result);
bool dol_hle_poll_callback(CPUState* cpu);
bool dol_hle_handle_callback_return(CPUState* cpu, u32 address);
void dol_hle_init(const void* config);

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
static DolDi s_di;
static DolSiDevice s_si;
static int s_chassis_inited = 0;
static int s_disc_present_logged = 0;
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
static bool chassis_di_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user;
    if(!dol_di_mmio_contains(ea)) return false;
    if(value) *value = dol_di_mmio_read(&s_di, ea, (u8)size);
    // Level-triggered DI source follows the device model.
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_DI, dol_di_interrupt_pending(&s_di));
    (void)cpu;
    return true;
}
static bool chassis_si_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user; (void)cpu;
    if(!dol_si_mmio_contains(ea)) return false;
    if(value) *value = dol_si_mmio_read(&s_si, ea, size);
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_SI, dol_si_interrupt_pending(&s_si));
    return true;
}
static bool chassis_si_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user; (void)cpu;
    if(!dol_si_mmio_contains(ea)) return false;
    dol_si_mmio_write(&s_si, ea, (u8)size, value);
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_SI, dol_si_interrupt_pending(&s_si));
    return true;
}
static bool chassis_di_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user;
    if(!dol_di_mmio_contains(ea)) return false;
    // Trace every DI MMIO write (bounded): command words + TSTART show the
    // exact SDK sequence without needing a symbol map.
    { static int _n=0; if(_n<40){ fprintf(stderr,"[di] %s write 0x%08X <- 0x%08X (pc=0x%08X)\n",
        ea==0xCC00601Cu?"CTL":ea==0xCC006014u?"DMAADDR":ea==0xCC006018u?"DMALEN":"REG",
        ea, (u32)value, cpu?cpu->pc:0); _n++; } }
    dol_di_mmio_write(&s_di, cpu, ea, (u8)size, value);
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_DI, dol_di_interrupt_pending(&s_di));
    return true;
}
// DI command executor: serve DVD reads from the opened disc image.
// Register map (dolsdk2001 dvdlow.c: __DIRegs[2..7] == DI COMMAND_0..DMA_LEN
// + CONTROL): a DVDLowRead programs c0=0xA8000000, c1=(offset>>2),
// c2=length, dma_address=guest addr, dma_length=length, control=TSTART|DMA.
// Command words arrive as raw MMIO writes (not shifted); offset = c1<<2.
// Only the DMA-read path is served; everything else completes as ERROR so
// the guest sees a real failure instead of a stuck status bit.
static DolDiCommandResult chassis_di_execute(void* user, DolDiCommand* cmd){
    (void)user;
    if(!cmd || !cmd->cpu) return DOL_DI_COMMAND_ERROR;
    u32 c0 = cmd->command[0];
    { static unsigned _nInq=0,_nStop=0,_nRead=0,_nOther=0,_nTot=0; static u32 _lastOther=0;
      _nTot++;
      if(c0==0x12000000u) _nInq++;
      else if((c0&0xFF000000u)==0xE3000000u) _nStop++;
      else if((c0&0xFF000000u)==0xA8000000u) _nRead++;
      else { _nOther++; _lastOther=c0; }
      if(_nTot==1 || _nTot%2000==0 || ((c0&0xFF000000u)!=0x12000000u&&(c0&0xFF000000u)!=0xE3000000u&&(c0&0xFF000000u)!=0xA8000000u&&_nOther<8))
        fprintf(stderr,"[di] mix tot=%u inq=%u stop=%u read=%u other=%u lastOther=0x%08X c0=0x%08X\n",
          _nTot,_nInq,_nStop,_nRead,_nOther,_lastOther,c0); }
    if(c0 == 0x12000000u && cmd->dma && !cmd->write && cmd->dma_length){
        // DVDLowInquiry: 32-byte DVDDriveInfo struct (dolsdk2001 dvd.h:
        // u16 revision, u16 deviceCode, u32 releaseDate, u8 pad[24]).
        // Realizarre values from Dolphin: rev 1, device 0, date 0x20011023-ish.
        // (GameCube SDK rev used by F-Zero era titles.)
        u32 a = cmd->dma_address;
        if(a >= GC_RAM_BASE && a + 32 <= GC_RAM_BASE + cmd->cpu->ram_size){
            uint8_t* d = cmd->cpu->ram + (a - GC_RAM_BASE);
            d[0]=0; d[1]=1; d[2]=0; d[3]=0;
            d[4]=0x20; d[5]=0x01; d[6]=0x10; d[7]=0x23;
            memset(d+8, 0, 24);
        }
        { static int _n=0; if(_n<3){ fprintf(stderr,"[di] exec INQUIRY -> guest 0x%08X\n", cmd->dma_address); _n++; } }
        // Completion signal: the 15FC0/16018 waiter polls DI status itself
        // (0xCC006000) and invokes the stored callback via its own blrl at
        // 16270 with r3=r29. The old code ALSO dispatched 0x80018D1C through
        // the HLE trampoline => double invocation per command => the
        // inq/stop retry loop (2000 cmds, 0 reads). Now just raise the flag
        // the waiter checks (r13-31592=1, set at 16018->ori r29,8), the way
        // the parked DI-interrupt handler would have. TCINT is already set
        // by dol_di_complete_command via the COMPLETE return below.
        { u32 fa = cmd->cpu->gpr[13] - 31592u;
          if(fa >= GC_RAM_BASE && fa + 4 <= GC_RAM_BASE + cmd->cpu->ram_size){
            uint8_t* p = cmd->cpu->ram + (fa - GC_RAM_BASE);
            p[0]=0; p[1]=0; p[2]=0; p[3]=1;
            { static int _m=0; if(_m<3){ fprintf(stderr,"[di] INQUIRY flag r13-31592=1 (r13=0x%08X)\n", cmd->cpu->gpr[13]); _m++; } }
          } }
        // SDK completion signature: DVDLowCallback(s32 result, DVDCommandBlock
        // *block) => r3=result (0=OK), r4=block. The old code passed r4=0, so
        // the 18D1C body could never advance the block state (tag stayed 14
        // => 18CC8 re-issued inquiry forever). Block addr comes from the
        // r13-31488 current-block global (0x8015BF20 per 16A38 probe).
        { u32 ba = cmd->cpu->gpr[13] - 31488u; u32 block = 0x8015BF20u;
          if(ba >= GC_RAM_BASE && ba + 4 <= GC_RAM_BASE + cmd->cpu->ram_size){
            uint8_t* p = cmd->cpu->ram + (ba - GC_RAM_BASE);
            block = ((u32)p[0]<<24)|((u32)p[1]<<16)|((u32)p[2]<<8)|p[3];
            if(block < GC_RAM_BASE) block = 0x8015BF20u;
          }
          dol_hle_queue_guest_callback(0x80018D1Cu, 0, (s32)block); }
        return DOL_DI_COMMAND_COMPLETE;
    }
    // Motor/stop/reset class (dolsdk2001 DVDLowStopMotor 0xE3, Reset etc.):
    // no payload, no DMA. COMPLETE (not ERROR) so the SDK state machine
    // advances instead of retrying down an error path.
    if((c0 & 0xFF000000u) == 0xE3000000u){
        { static int _n=0; if(_n<3){ fprintf(stderr,"[di] exec STOPMOTOR (complete)\n"); _n++; } }
        return DOL_DI_COMMAND_COMPLETE;
    }
    if((c0 & 0xFF000000u) == 0xA8000000u
       && cmd->dma && !cmd->write && cmd->dma_length){
        u32 disc_off = cmd->command[1] << 2;
        { static int _n=0; if(_n<6){ fprintf(stderr,"[di] exec read c0=0x%08X off=0x%08X len=%u -> guest 0x%08X\n", c0, disc_off, cmd->dma_length, cmd->dma_address); _n++; } }
        // Backing store: extracted tree via FST offset map (dvd_host.c).
        // dvd.c's ISO handle is empty (no plain ISO — only .rvz + tree),
        // so route through the tree mapper; fall back to dvd.c if an image
        // ever opens (it zero-fills past EOF the same way).
        extern unsigned dvd_read_disc_bytes(const uint8_t* fst, unsigned fst_size, unsigned disc_off, uint8_t* dst, unsigned len);
        uint32_t base = 0x81200000u;
        if(base >= GC_RAM_BASE && base < GC_RAM_BASE + cmd->cpu->ram_size){
            uint8_t* f = cmd->cpu->ram + (base - GC_RAM_BASE);
            unsigned n = ((unsigned)f[8]<<24)|((unsigned)f[9]<<16)|((unsigned)f[10]<<8)|f[11];
            unsigned fsz = n*12u;
            // string table extent unknown here; pass a large cap — the
            // mapper bounds-checks n*12 against it and names against NULs.
            // fst.bin is 129405B; cap at 1MB.
            if(fsz < 1024*1024){
                uint32_t ga = cmd->dma_address;
                u8* dst = NULL;
                if(ga >= GC_RAM_BASE && ga + cmd->dma_length <= GC_RAM_BASE + cmd->cpu->ram_size)
                    dst = cmd->cpu->ram + (ga - GC_RAM_BASE);
                else if(ga >= GC_RAM_UNCACHED && ga + cmd->dma_length <= GC_RAM_UNCACHED + cmd->cpu->ram_size)
                    dst = cmd->cpu->ram + (ga - GC_RAM_UNCACHED);
                if(dst){
                    unsigned got = dvd_read_disc_bytes(f, 1024*1024, disc_off, dst, cmd->dma_length);
                    if(got < cmd->dma_length) memset(dst+got, 0, cmd->dma_length-got);
                    { static int _n=0; if(_n<4){ fprintf(stderr,"[di] tree read off=0x%08X got=%u/%u\n", disc_off, got, cmd->dma_length); _n++; } }
                    return DOL_DI_COMMAND_COMPLETE;
                }
            }
        }
        dvd_read_to_guest(cmd->cpu, cmd->dma_address, disc_off, cmd->dma_length);
        return DOL_DI_COMMAND_COMPLETE;
    }
    { static int _n=0; if(_n<6){ fprintf(stderr,"[di] exec UNHANDLED c0=0x%08X c1=0x%08X c2=0x%08X dma=%u wr=%u addr=0x%08X len=%u\n", cmd->command[0], cmd->command[1], cmd->command[2], cmd->dma?1:0, cmd->write?1:0, cmd->dma_address, cmd->dma_length); _n++; } }
    return DOL_DI_COMMAND_ERROR;
}
static void chassis_init(void){
    if(s_chassis_inited) return;
    dol_hle_init(NULL);
    dol_mmio_bus_init(&s_mmio_bus);
    dol_interrupts_init(&s_interrupts);
    dol_vi_clock_init(&s_vi_clock);
    dol_vi_clock_configure(&s_vi_clock, 1u, 60u, GC_TIMEBASE_HZ);
    dol_di_init(&s_di);
    dol_di_set_command_callback(&s_di, chassis_di_execute, NULL);
    // Route VI (0xCC002000 len 0x80) + PI (0xCC003000 len 0x40) + PE status
    // through the production interrupt model; DI (0xCC006000 len 0x28)
    // through the production DI model; every other device still
    // reports 0 via hle_external_* fallback below.
    dol_mmio_bus_register(&s_mmio_bus, 0xCC002000u, 0x80u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC003000u, 0x40u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC00100Au, 2u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC006000u, 0x28u, chassis_di_read, chassis_di_write, NULL);
    { extern bool chassis_si_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value);
      extern bool chassis_si_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value);
      dol_si_init(&s_si);
      dol_mmio_bus_register(&s_mmio_bus, 0xCC006400u, 0x100u, chassis_si_read, chassis_si_write, NULL); }
    // Bootstrapping: the SDK enables TCINT (transfer-complete) interrupt
    // delivery before issuing reads. Until the guest programs the mask
    // itself, pre-enable it so the first inquiry completion is observable
    // at PI cause as well as DI status (both are level-triggered; the
    // guest ack clears them per the write-1-to-clear model).
    dol_interrupts_mmio_write(&s_interrupts, 0xCC006000u, 4u,
        DOL_DI_STATUS_TCINTMASK | DOL_DI_STATUS_DEINTMASK);
    // Open the disc image once so DMA reads have backing bytes.
    // dvd_open_image is idempotent (first success wins). Candidates: the
    // extracted fst.bin's siblings first (boot.bin/bi2.bin live next to a
    // full dump), then <root>.iso/.gcm, then the .rvz itself (handled by
    // DolRecomp's disc extractor only — dvd.c needs a plain ISO, so the
    // .rvz probe is expected to fail and fall through to no-image).
    {
        extern const char* DVDHostRoot(void);
        const char* root = DVDHostRoot();
        char ipath[640];
        static const char* tails[] = {
            "\\sys\\boot.bin", "\\files\\sys\\boot.bin",
            ".iso", ".gcm",
            "\\f-zero gx (usa).rvz", "\\f-zero gx (usa).iso",
            NULL
        };
        for(int t=0; tails[t]; t++){
            if(tails[t][0]=='.'){
                char parent[512]; snprintf(parent, sizeof(parent), "%s", root);
                char* bs = strrchr(parent, '\\');
                if(!bs) bs = strrchr(parent, '/');
                if(!bs) continue;
                *bs = 0;
                snprintf(ipath, sizeof(ipath), "%s%s", parent, tails[t]);
            } else {
                snprintf(ipath, sizeof(ipath), "%s%s", root, tails[t]);
            }
            FILE* f = fopen(ipath, "rb");
            if(!f) continue;
            fclose(f);
            if(dvd_open_image(ipath)){
                if(!s_disc_present_logged){ fprintf(stderr,"[dvd] image %s\n", ipath); s_disc_present_logged=1; }
                break;
            }
        }
        if(!dvd_image_ready() && !s_disc_present_logged){
            // No plain ISO exists (only .rvz + extracted tree) — but the
            // tree-backed 0xA8 path serves real bytes, so report the disc
            // as present (cover CLOSED). Leaving cover OPEN makes the
            // guest's 177D0 cover poll spin before it ever issues reads.
            fprintf(stderr,"[dvd] no plain ISO image; tree-backed reads active, cover=CLOSED (%s)\n", root);
            s_disc_present_logged = 1;
        }
        dol_di_set_disc_present(&s_di, true);
        dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_DI, dol_di_interrupt_pending(&s_di));
    }
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
    if((addr&0xFF000000u)==0xCC000000u || (addr&0xFF000000u)==0xCD000000u){
        if(addr >= 0xCC006000u && addr < 0xCC006028u){
            static int _n=0;
            if(_n<16){ fprintf(stderr,"[di] read%u 0x%08X -> 0 (pc=0x%08X)\n", size, addr, cpu->pc); _n++; }
        }
        return 0;
    }
    (void)cpu;(void)addr;(void)size;
    return 0;
}
static void hle_external_write(CPUState* cpu, uint32_t addr, uint64_t val, uint8_t size){
    s_mmio_writes++;
    if(dol_mmio_bus_write(&s_mmio_bus, cpu, addr, size, val)) return;
    (void)cpu;(void)addr;(void)val;(void)size;
    // DI command trace: log TSTART + command words so we learn what disc
    // offsets the guest requests (no symbol map needed). Bounded logging.
    if(addr >= 0xCC006000u && addr < 0xCC006028u){
        static int _n=0;
        if(_n<24){ fprintf(stderr,"[di] write%u 0x%08X <- 0x%llX (pc=0x%08X)\n", size, addr, (unsigned long long)val, cpu->pc); _n++; }
    }
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
    // Read-only probe: log 16A38 (DI inquiry wrapper) entry regs to learn
    // the command-block pointer + callback. Touches nothing (returns false).
    if(addr==0x80016A38u){
        static int _n=0;
        if(_n<6){ fprintf(stderr,"[di] 16A38 entry r3=0x%08X r4=0x%08X r5=0x%08X r6=0x%08X r7=0x%08X lr=0x%08X\n",
            cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->gpr[6], cpu->gpr[7], cpu->lr); _n++; }
        return false;
    }
    // NOTE 800102AC is NOT DVDGetFSTLocation: it reads low-mem 0x800000E4
    // (an OSArena/OS-context word) and returns. Keep the FST install here
    // only as a side effect; do NOT skip the body — let it run so r3 and
    // the low-mem word stay guest-coherent.
    if(addr==0x800102ACu){
        // Install the real fst.bin blob at 0x81200000 as a SIDE EFFECT ONLY
        // (first call wins; the blob persists in guest RAM), then let the
        // guest body run so r3/low-mem stay guest-coherent. Skipping the
        // body (blr) was wrong: 102AC is a low-mem reader (0x800000E4),
        // not DVDGetFSTLocation.
        extern int dvd_build_fst_from_tree(uint8_t* ram, unsigned ram_size, unsigned base);
        { static int _done=0;
          if(!_done){
              _done=1;
              if(cpu->ram_size >= 0x1200010u){
                  uint32_t base=0x81200000u; uint32_t off=base - GC_RAM_BASE;
                  if(off+16 <= cpu->ram_size){
                      int n = dvd_build_fst_from_tree(cpu->ram, cpu->ram_size, base);
                      if(n > 0){
                          write_be32(cpu->ram + (0x80000038u - GC_RAM_BASE), base);
                          fprintf(stderr,"[dvd] FST entries=%d base=0x%08X (side effect @0x800102AC)\n", n, base);
                      }
                  }
              }
          } }
        return false; // do NOT skip: run the recompiled body
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
    uint32_t arena_lo = 0;
    {
        arena_lo = (bss_addr + bss_size + 0x20000u + 31u) & ~31u;
        if(arena_lo < GC_RAM_BASE || arena_lo > 0x817FEC60u) arena_lo = bss_addr ? bss_addr : GC_RAM_BASE;
        POKE32(0x80000020u, 0x0D15EA5Eu); POKE32(0x80000024u, 1u); POKE32(0x80000028u, 0x01800000u);
        POKE32(0x8000002Cu, 1u); POKE32(0x80000030u, arena_lo); POKE32(0x80000034u, 0x817FEC60u);
        POKE32(0x80000038u, 0u); POKE32(0x8000003Cu, 0u); POKE32(0x800000CCu, 0u);
        // NOTE: 0x800000D4 is owned by the guest OS (OSContext pointer set by
        // __OSInit / OSInitThreadQueue). Do NOT pre-seed it: 102AC reads this
        // word and the boot PSL would diverge from hardware.
        POKE32(0x800000F8u, 0x09A7EC80u); POKE32(0x800000FCu, 0x1CF7C580u);
        cpu->gpr[1] = 0x817FFF00u; POKE32(0x817FFF00u, 0u); POKE32(0x817FFF04u, 0u);
    }
    #undef POKE32
    // IPL handoff: synthesize the initial thread the apploader hands the
    // DOL. 0x800000E4 (__gCurrentThread, dolsdk2001 os.h:55) must point at
    // a valid OSThread (context @0x0 size 0x2C8, state @0x2C8u16=RUNNING(2),
    // priority @0x2D0, stackEnd @0x308). Placed in the 128KB stack-reserve
    // gap below arena_lo so OSInit's arena clear cannot wipe it; stack top
    // stays 0x817FFF00 above arena_hi as before.
    {
        uint32_t tbase = (arena_lo - 0x1000u) & ~31u; // 4KB thread struct
        uint32_t stack_top = 0x817FFF00u;
        if(tbase >= GC_RAM_BASE && tbase + 0x1000 <= GC_RAM_BASE + cpu->ram_size){
            uint8_t* t = cpu->ram + (tbase - GC_RAM_BASE);
            memset(t, 0, 0x1000);
            // OSContext: srr0=entry (resume here), msr=FP, state=FPSAVED|EXC.
            // state field @0x1A2 is u16: FPSAVED(0x01 per dolsdk2001 OSContext.h:135).
            t[0x198]=(uint8_t)(entry>>24); t[0x199]=(uint8_t)(entry>>16);
            t[0x19A]=(uint8_t)(entry>>8); t[0x19B]=(uint8_t)entry;
            t[0x19C]=0; t[0x19D]=0; t[0x19E]=0x20; t[0x19F]=0; // msr FP
            t[0x1A2]=0; t[0x1A3]=0x01; // FPSAVED
            // gpr1 in context @0x04 = stack top
            t[0x04]=(uint8_t)(stack_top>>24); t[0x05]=(uint8_t)(stack_top>>16);
            t[0x06]=(uint8_t)(stack_top>>8); t[0x07]=(uint8_t)stack_top;
            // OSThread: state=RUNNING(2) @0x2C8, priority @0x2D0/@0x2D4,
            // stackBase @0x304, stackEnd @0x308, queue=NULL @0x2DC.
            t[0x2C8]=0; t[0x2C9]=2;
            t[0x2D0]=0; t[0x2D1]=16; // priority 16 (normal)
            t[0x2D4]=0; t[0x2D5]=16;
            t[0x304]=(uint8_t)(stack_top>>24); t[0x305]=(uint8_t)(stack_top>>16);
            t[0x306]=(uint8_t)(stack_top>>8); t[0x307]=(uint8_t)stack_top;
            t[0x308]=(uint8_t)(stack_top>>24); t[0x309]=(uint8_t)(stack_top>>16);
            t[0x30A]=(uint8_t)(stack_top>>8); t[0x30B]=(uint8_t)stack_top;
            if(0x800000E4u >= GC_RAM_BASE && 0x800000E4u + 4 <= GC_RAM_BASE + cpu->ram_size)
                write_be32(cpu->ram + (0x800000E4u - GC_RAM_BASE), tbase);
            fprintf(stderr,"[boot] initial thread @0x%08X (E4 set, RUNNING prio16)\n", tbase);
        }
    }
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
    // Deterministic timebase: 675000 ticks per VI retrace ONLY (driven from
    // the vi_clock below). The old wall-clock bump (486M/240 per slice) made
    // identical runs diverge by host speed. Removed per PLAN M0.
    (void)0;
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
        // (was: 0x80011424 timebase += 5000 wall-clock hack — removed per M0.)
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
        // (was: 0x8000BE60 spoofed low-mem 0xD4=0x80003000 — removed.
        // BE5C/BE60 reads the OS global at 0x800000D4, owned by the guest;
        // seeding it corrupts the OSContext chain. Let the guest write it.)
        else if(pc==0x80011160u) poke32_set(g_cpu.gpr[13]-31684u, 1u);
        // Bounded DVD state-machine probes (remove once M2 answered):
        // 189FC = completion dispatcher (r3=cmd block, +8=type tag);
        // 16018 = waiter flag check; 18CEC = inquiry re-issue site.
        if(pc==0x800189FCu){ static int _n=0; if(_n<8){ _n++;
            u32 b=g_cpu.gpr[3]; u32 tag=0; if(b>=GC_RAM_BASE && b+12<=GC_RAM_BASE+g_cpu.ram_size) tag=((u32)g_cpu.ram[b-GC_RAM_BASE+8]<<24)|((u32)g_cpu.ram[b-GC_RAM_BASE+9]<<16)|((u32)g_cpu.ram[b-GC_RAM_BASE+10]<<8)|g_cpu.ram[b-GC_RAM_BASE+11];
            u32 r13=g_cpu.gpr[13];
            u32 s48=0,s60=0,cb=0; if(r13-31568u>=GC_RAM_BASE && r13-31448u+4<=GC_RAM_BASE+g_cpu.ram_size){ uint8_t* m=g_cpu.ram+(r13-31568u-GC_RAM_BASE); s48=((u32)m[0]<<24)|((u32)m[1]<<16)|((u32)m[2]<<8)|m[3]; uint8_t* m2=g_cpu.ram+(r13-31460u-GC_RAM_BASE); s60=((u32)m2[0]<<24)|((u32)m2[1]<<16)|((u32)m2[2]<<8)|m2[3]; uint8_t* m3=g_cpu.ram+(r13-31488u-GC_RAM_BASE); cb=((u32)m3[0]<<24)|((u32)m3[1]<<16)|((u32)m3[2]<<8)|m3[3]; }
            fprintf(stderr,"[dvdsm] 189FC block=0x%08X tag=%u st-31568=%u drv-31460=%u cb-31488=0x%08X lr=0x%08X\n", b, tag, s48, s60, cb, g_cpu.lr);
            // Jump-table target: table base 0x80124018 (lis -32750 +16408),
            // entry = mem32(base + tag*4). Tells which case row tag 14 hits.
            { u32 base=0x80124018u, tgt=0; if(base>=GC_RAM_BASE && base+64*4<=GC_RAM_BASE+g_cpu.ram_size){ uint8_t* m=g_cpu.ram+(base-GC_RAM_BASE)+tag*4; tgt=((u32)m[0]<<24)|((u32)m[1]<<16)|((u32)m[2]<<8)|m[3]; }
              static int _k=0; if(_k<2){ _k++; fprintf(stderr,"[dvdsm] 189FC tag=%u -> target=0x%08X\n", tag, tgt); } } } }
        if(pc==0x80016018u){ static int _n=0; if(_n<8){ _n++;
            u32 r13=g_cpu.gpr[13]; u32 fl=0; if(r13-31592u>=GC_RAM_BASE && r13-31592u+4<=GC_RAM_BASE+g_cpu.ram_size){ uint8_t* m=g_cpu.ram+(r13-31592u-GC_RAM_BASE); fl=((u32)m[0]<<24)|((u32)m[1]<<16)|((u32)m[2]<<8)|m[3]; }
            fprintf(stderr,"[dvdsm] 16018 flag-31592=%u DIstatus=0x%08X lr=0x%08X\n", fl, s_di.status, g_cpu.lr); } }

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
                dol_si_latch_poll(&s_si, 0xFu);
                dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_SI, dol_si_interrupt_pending(&s_si));
            }
            chassis_deliver_external();
        }
        // HLE async-callback trampoline: if a queued guest callback (e.g.
        // the DI inquiry completion at 0x80018D1C) is pending, run it with
        // saved context; it returns through HLE_CALLBACK_RETURN.
        if(pc == HLE_CALLBACK_RETURN){
            if(dol_hle_handle_callback_return(&g_cpu, pc)) continue;
        }
        if(dol_hle_poll_callback(&g_cpu)) continue;
        // First-dispatch trace: log each never-before-dispatched pc once.
        // Ring of last 64 + ever-total: early boot saturates any first-N
        // cap (384 unique in minutes), so keep a sliding window over the
        // frontier instead. GX-family entry is the M3 signal.
        { static uint32_t _seen[2048]; static int _nseen=0; static int _logged=0;
          int _f=0; for(int _i=0;_i<_nseen;_i++) if(_seen[_i]==pc){ _f=1; break; }
          if(!_f){ if(_nseen<2048) _seen[_nseen++]=pc;
            if(_logged<384 || (_nseen % 16)==0){ _logged++;
              fprintf(stderr,"[new] pc=0x%08X lr=0x%08X (uniq=%d logged=%d)\n", pc, g_cpu.lr, _nseen, _logged); } } }
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
