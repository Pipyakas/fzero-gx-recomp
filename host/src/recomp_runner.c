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
bool dol_hle_queue_guest_callback(u32 address, u32 r3, u32 r4);
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
static bool guest_read32(uint32_t addr, uint32_t* out); // fwd: def after chassis_init (needs g_cpu)
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
        // EXP fzV: drop the r13-31592=1 flag raise. Rationale: it fires on
        // EVERY command (stale r13 under the trampoline's saved context),
        // so it can short-circuit the 15FC0 waiter before the callback runs.
        // Keep only block-state=END + the slot callback below.
        // SDK completion: DVDLowCallback(result, block). HLE poll sets
        // gpr3=channel, gpr4=result, so queue channel=0 (result OK) and
        // result=block pointer. Block addr from r13-31488 current-block
        // global (0x8015BF20 per 16A38 probe). r3=0 takes the 18D68
        // success path (18D20 early-out is only for r3==0x10 error).
        // Dispatch the LOW-LEVEL callback 18D1C (16A38's r4), NOT the
        // block+40 slot (A374). 18D1C does drive-state bookkeeping
        // (-31456/-31448/-31436, 187CC chain) then invokes the slot itself
        // via its native blrl (18F2C: r3=0, r4=block). Dispatching A374
        // directly skips that bookkeeping => inquiry retried forever
        // (fzS-fzY: inq=100%, tag=14 always). r3=0 (OK), r4=block.
        { uint32_t block = 0x8015BF20u;
          guest_read32(cmd->cpu->gpr[13]-31488u, &block);
          if(block < GC_RAM_BASE) block = 0x8015BF20u;
          uint32_t cb = 0x80018D1Cu;
          // fzCB: do NOT pre-write block+12. The native body files
          // 18DF8 stw r0,12(r30) with r0=10 itself (after setting drive
          // state -31448=7, -31456=0) — our END(0) pre-write is overwritten
          // (b12 flip-flops 0/-1 across callbacks) and may corrupt the
          // state the body expects. Lengths at +28/+32 are untouched.
          // (A374 slot check is moot: body invokes slot itself via blrl.)
          { uint32_t t1 = block + 28u, t2 = block + 32u; uint32_t len = cmd->dma_length;
            if(t1+4u <= GC_RAM_BASE + cmd->cpu->ram_size && t2+4u <= GC_RAM_BASE + cmd->cpu->ram_size){
              uint8_t* q1 = cmd->cpu->ram + (t1 - GC_RAM_BASE);
              uint8_t* q2 = cmd->cpu->ram + (t2 - GC_RAM_BASE);
              q1[0]=(uint8_t)(len>>24); q1[1]=(uint8_t)(len>>16); q1[2]=(uint8_t)(len>>8); q1[3]=(uint8_t)len;
              q2[0]=(uint8_t)(len>>24); q2[1]=(uint8_t)(len>>16); q2[2]=(uint8_t)(len>>8); q2[3]=(uint8_t)len;
            }
            { static int _m=0; if(_m<2){ _m++; fprintf(stderr,"[di] INQUIRY blk=0x%08X len=%u cb=0x%08X (b12 left for native body)\n", block, len, cb); } } }
          dol_hle_queue_guest_callback(cb, 0, block); }
        return DOL_DI_COMMAND_COMPLETE;
    }
    // Motor/stop/reset class (dolsdk2001 DVDLowStopMotor 0xE3, Reset etc.):
    // no payload, no DMA. COMPLETE (not ERROR) so the SDK state machine
    // advances instead of retrying down an error path.
    if((c0 & 0xFF000000u) == 0xE3000000u){
        { static int _n=0; if(_n<3){ fprintf(stderr,"[di] exec STOPMOTOR (complete)\n"); _n++; } }
        // fzCD: STOPMOTOR completion must ALSO mark block+12. The inquiry
        // callback's native body writes b12=-1 on its error path and +10 on
        // success; the stop callback's body reads block+12 the same way. If
        // we leave the inquiry body's stale -1/10 in place, the stop
        // completion branch mis-resolves. Write the SDK END(0) marker: the
        // stop body has no native writer of its own (it only reads).
        // fzCL/fzCP/fzCQ: EXP — set drive-substate m56 (-31456) = 1.
        // fzCQ timing proof: callback #1 (INQUIRY completion) runs BEFORE
        // any STOPMOTOR write, so m56=0 -> body takes 18DD4->18E40 and files
        // b12=-1 (error path, correct: no drive op done yet). Callback #2+
        // (STOPMOTOR completions) see our m56=1 -> body takes 18DD8 path
        // (files b12=10... but entry still shows b12=0/m48=0 because those
        // are read BEFORE the body runs — post-body values unobserved).
        // The park persists through both paths, so the drive-state branch is
        // NOT the park driver. Keep EXP (harmless, matches SDK post-op=1).
        { uint32_t blk=0x8015BF20u; guest_read32(cmd->cpu->gpr[13]-31488u, &blk);
          if(blk < GC_RAM_BASE) blk = 0x8015BF20u;
          uint32_t ba = blk + 12u;
          if(ba >= GC_RAM_BASE && ba + 4u > ba && ba + 4u <= GC_RAM_BASE + cmd->cpu->ram_size){
            uint8_t* p = cmd->cpu->ram + (ba - GC_RAM_BASE);
            p[0]=0; p[1]=0; p[2]=0; p[3]=0; }
          { uint32_t ma = cmd->cpu->gpr[13]-31456u;
            if(ma >= GC_RAM_BASE && ma + 4u > ma && ma + 4u <= GC_RAM_BASE + cmd->cpu->ram_size){
              uint8_t* q = cmd->cpu->ram + (ma - GC_RAM_BASE);
              q[0]=0; q[1]=0; q[2]=0; q[3]=1; } }
          { static int _m=0; if(_m<3){ fprintf(stderr,"[di] STOPMOTOR blk=0x%08X b12=END m56=1 -> 18D1C\n", blk); _m++; } }
          dol_hle_queue_guest_callback(0x80018D1Cu, 0, blk); }
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
    // fzB7: work_units_per_retrace was 1 with advance(1) per dispatch =>
    // EVERY block = a full 675000-tick retrace; timebase raced ~1e5x too
    // fast, so every 1140C mftb latch (AECC r4) was unique and the AC44
    // free-list walk never settled. One retrace per 200k dispatches instead.
    dol_vi_clock_configure(&s_vi_clock, 200000u, 60u, GC_TIMEBASE_HZ);
    // fzAV result: timebase seed REFUTED (node8 went 0->1 = wall-clock
    // read working, but park persists). Seed reverted to 0 to keep M0
    // determinism (retrace-driven only).
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
static bool guest_read32(uint32_t addr, uint32_t* out){
    if(addr < GC_RAM_BASE || addr + 4u > GC_RAM_BASE + g_cpu.ram_size || addr + 4u < addr) return false;
    uint8_t* p = g_cpu.ram + (addr - GC_RAM_BASE);
    *out = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
    return true;
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
        // mtdec (X/O 166) / mfdec (X/O 459): decrementer is SPR 22 in the
        // spr[] file (mtspr/mfspr already route it); the fallback only needs
        // to advance pc. Before this, mtdec raised ILLEGAL and broke the
        // slice out of the AD1C-park allocator path (9FEC call at AD9C/ADC4).
        if(xo==166){ uint32_t rs=(raw>>21)&31u; ppc_mtspr(cpu,22,cpu->gpr[rs],cia); if(cpu->exception==0) cpu->pc=cia+4; return; }
        if(xo==459){ uint32_t rt=(raw>>21)&31u; uint32_t before=cpu->exception; uint32_t v=ppc_mfspr(cpu,22,cia); if(cpu->exception==before){ cpu->gpr[rt]=v; cpu->pc=cia+4; } return; }
    }
    fprintf(stderr,"[hle] fallback raw=0x%08X @0x%08X\n",raw,cia);
    ppc_program_exception(cpu, PPC_PROGRAM_ILLEGAL, cia);
}
static bool hle_host_call(CPUState* cpu, uint32_t addr){
    // fzD5/fzD6: AEDC is `bl AC44` with NO downcount (falls through from
    // AED8, no dispatch point) — like AEC8 it NEVER fires as a probe, healthy
    // or DVD. The chain AECC->AED0->AED4->AED8->AEDC->AC44 runs native inside
    // one dolrecomp_call; only AC44 (downcount) dispatches next. So the DVD
    // request passing AECC but never reaching AC44 means it diverts INSIDE
    // that native chain — candidates: exception in AED0 adde/AED4/AED8, or
    // the chunk RETURNS early (downcount budget) between AECC and AC44.
    if(addr==0x8000AEDCu){
      static unsigned _a=0; if(++_a<=8) fprintf(stderr,"[watch] AEDC r3=0x%08X r6=0x%08X r7=0x%08X r30=0x%08X lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->gpr[6], cpu->gpr[7], cpu->gpr[30], cpu->lr, _a);
      return false; }
    // host_call runs inside dolrecomp_call BEFORE the chunk — but only for
    // the pc that STARTED the call (fzBN: AD60/AD68 never fire because the
    // chunk runs them natively mid-chain). Same visibility as slice-loop
    // if() probes. Mid-chain state is only observable via consulting the
    // post-lap effect at the next dispatched pc (see [ins] post-lap).
    // 16920 = DVD stop-motor wrapper entry (lr=1923C: called from INSIDE
    // the 18D1C completion path at 19238). r30 arrives SET (fzBR) — but the
    // whole 18D1C body runs native from ONE dispatch, so r30 at 16920 entry
    // is whatever 18D1C carried in ITS r30. Probe 18D1C entry (dispatched,
    // chunk_5 entry): r30 there is the callback's incoming r30.
    if(addr==0x80016920u){
      static unsigned _m=0; if(++_m<=6) fprintf(stderr,"[watch] 16920 r3=0x%08X r30=0x%08X r31=0x%08X tb=0x%llX lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->gpr[30], cpu->gpr[31], (unsigned long long)cpu->timebase, cpu->lr, _m);
      return false; }
    // fzCH: 17958-chain (gate writer 1799C inside) — dispatched entry?
    // Dump r3 (the 0x10-compare arg) + r13 vars to see if it ever runs and
    // with what state. Its callers: 18D60 bl 17958 (inside 18D1C body).
    if(addr==0x80017958u){
      static unsigned _h=0; if(++_h<=6){ uint32_t v64=0,v60=0;
        guest_read32(cpu->gpr[13]-31464u,&v64); guest_read32(cpu->gpr[13]-31460u,&v60);
        fprintf(stderr,"[watch] 17958 r3=0x%08X m64=%u m60=%u lr=0x%08X (#%u)\n",
          cpu->gpr[3], v64, v60, cpu->lr, _h); }
      return false; }
    // 18D1C = low-level completion entry (dispatched). fzBT: r30 IDENTICAL
    // at 18D1C and 16920 (0x1823CF40) — rides in on the SAVED slice context
    // (trampoline preempts AC34 with garbage r30). Only r3/r4 are args.
    // Fix (fzBU): zero the callback's non-arg regs at poll time.
    if(addr==0x80018D1Cu){
      static unsigned _d=0; if(++_d<=4||_d%5000000==0){
        uint32_t v60=0,v56=0,v36=0,v32=0,b12=0,v64=0,v48=0; uint32_t blk=cpu->gpr[4];
        guest_read32(cpu->gpr[13]-31460u,&v60); guest_read32(cpu->gpr[13]-31456u,&v56);
        guest_read32(cpu->gpr[13]-31436u,&v36); guest_read32(cpu->gpr[13]-31432u,&v32);
        guest_read32(cpu->gpr[13]-31464u,&v64); guest_read32(cpu->gpr[13]-31448u,&v48);
        if(blk){ guest_read32(blk+12u,&b12); }
        uint32_t b8=0; if(blk) guest_read32(blk+8u,&b8);
        fprintf(stderr,"[watch] 18D1C r3=%u blk=0x%08X b8=%u b12=%u m64=%u m60=%u m56=%u m48=%u m36=%u m32=%u r30=0x%08X r31=0x%08X (#%u)\n",
          cpu->gpr[3], blk, b8, b12, v64, v60, v56, v48, v36, v32, cpu->gpr[30], cpu->gpr[31], _d); }
      return false; }
    // Re-walk key probe (fzBJ): AD60 publishes head=r29 then AD68 rebuilds
    // the search key from the NEW node (r6=[r29+12], r0=[r29+8]).
    if(addr==0x8000AD60u||addr==0x8000AD68u){
      static unsigned _e=0; if(++_e<=8){ uint32_t head=0; guest_read32(cpu->gpr[13]-31800u, &head);
        fprintf(stderr,"[park] %s head=0x%08X r6=0x%08X r0=0x%08X r29=0x%08X r30=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          addr==0x8000AD60u?"AD60(publish)":"AD68(rekey)",
          head, cpu->gpr[6], cpu->gpr[0], cpu->gpr[29], cpu->gpr[30], cpu->gpr[4], cpu->lr, _e); }
      return false; }
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
    for(int f=0; f<8; f++){
        u32 caller = 0, ret = 0;
        if(sp < GC_RAM_BASE || sp + 8u < sp || sp + 8u > GC_RAM_BASE+g_cpu.ram_size) break;
        if(!guest_read32(sp, &caller)) break;
        if(caller<=sp || caller+8u < caller || caller+8u > GC_RAM_BASE+g_cpu.ram_size) break;
        if(!guest_read32(caller+4u, &ret)) break;
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
        // (was: 0x80010718 poke r13-31688=1 — removed. 10718/1071C is a
        // spin-wait on that flag; forcing it defeats the wait it guards.)
        else if(pc==0x80010624u && g_cpu.gpr[6]==0) g_cpu.gpr[6]=1;
        // (was: 0x8000BE60 spoofed low-mem 0xD4=0x80003000 — removed.
        // BE5C/BE60 reads the OS global at 0x800000D4, owned by the guest;
        // seeding it corrupts the OSContext chain. Let the guest write it.)
        else if(pc==0x80011160u) poke32_set(g_cpu.gpr[13]-31684u, 1u);
        // Bounded DVD state-machine probes (remove once M2 answered):
        // 189FC = completion dispatcher (r3=cmd block, +8=type tag);
        // 16018 = waiter flag check; 18CEC = inquiry re-issue site.
        // fzCS: uncap 189FC (was first-8-only): count entries per lap-phase
        // (before/after AC44) to learn whether the sink re-runs per lap.
        if(pc==0x800189FCu){ static unsigned _n=0; _n++;
          if(_n<=8||_n%5000000==0){ uint32_t head=0; guest_read32(g_cpu.gpr[13]-31800u, &head);
            fprintf(stderr,"[dvdsm] 189FC hits=%u head=0x%08X tb=0x%llX\n", _n, head, (unsigned long long)g_cpu.timebase); } }
        if(pc==0x80016018u){ static int _n=0; if(_n<8){ _n++;
            u32 fl=0; guest_read32(g_cpu.gpr[13]-31592u, &fl);
            fprintf(stderr,"[dvdsm] 16018 flag-31592=%u DIstatus=0x%08X lr=0x%08X\n", fl, s_di.status, g_cpu.lr); } }
        // 19690 = parent that calls 19700->187CC sink (dispatch chain
        // 19690->19700->187CC->19FA4?->...->189FC->18CC8->16A38). Read-only:
        // dump block[8] (tag), block[12] (state), r13 vars, and lr.
        // fzBX correction: 1970C/16970 are mid-chain natives (no downcount)
        // that never dispatch — probes removed. 19700 shows r30=0x8015BF00
        // (block ptr, from 19690's r3 arg chain), so r30 ENTERS 18D1C's
        // native body as the block pointer; the 16970 mulli then derives the
        // search key from the 1142C timebase latch. Key question answered:
        // key = f(timebase), not f(block) — timebase cadence sets the key.
        if(pc==0x80019690u||pc==0x80019700u||pc==0x800187CCu){
          static unsigned _w1=0,_w2=0,_w3=0;
          unsigned *c = pc==0x80019690u?&_w1:pc==0x80019700u?&_w2:&_w3; (*c)++;
          if(*c<=4){ u32 tag=0xDEADu,st=0xDEADu,s48=0,s60=0,cb=0;
            guest_read32(0x8015BF20u+8u, &tag); guest_read32(0x8015BF20u+12u, &st);
            guest_read32(g_cpu.gpr[13]-31568u, &s48); guest_read32(g_cpu.gpr[13]-31460u, &s60); guest_read32(g_cpu.gpr[13]-31488u, &cb);
            fprintf(stderr,"[dvdsm] %s r3=0x%08X r30=0x%08X r31=0x%08X tag=%u st=%u s48=%u drv=%u curblk=0x%08X lr=0x%08X (#%u)\n",
              pc==0x80019690u?"19690":pc==0x80019700u?"19700":"187CC",
              g_cpu.gpr[3], g_cpu.gpr[30], g_cpu.gpr[31], tag, st, s48, s60, cb, g_cpu.lr, *c); }
          if((*c%20000)==0) fprintf(stderr,"[dvdsm] sinkchain 19690=%u 19700=%u 187CC=%u\n", _w1,_w2,_w3); }
        // 18F38 = result-bit branch; 18E68 = drive-state branch; 1920C = alt path.
        // Uncapped counters + first-few dumps: which callback path executes?
        // fzC6: ALSO watch the 18D1C return chain 18CF0/18D0C/16A38 — these
        // only dispatch when the native 18D1C body RETURNS to them (blr/blrl
        // landing). If 18CF0/18D0C never dispatch but 18D1C does, the body
        // never returns (it issues 18CC8->16A38 inline INSTEAD of returning).
        if(pc==0x80018CF0u||pc==0x80018D0Cu){
          static unsigned _r1=0,_r2=0; unsigned *c = pc==0x80018CF0u?&_r1:&_r2; (*c)++;
          if(*c<=4||*c%5000000==0) fprintf(stderr,"[dvdsm] %s hit lr=0x%08X (#%u)\n",
            pc==0x80018CF0u?"18CF0":"18D0C", g_cpu.lr, *c); }
        if(pc==0x80018F38u||pc==0x80018E68u||pc==0x8001920Cu||pc==0x80018DB0u||pc==0x80018D1Cu||pc==0x80018CC8u||pc==0x80018D68u){
          static unsigned _c1=0,_c2=0,_c3=0,_c4=0,_c5=0,_c6=0,_c7=0;
          unsigned *c = pc==0x80018F38u?&_c1:pc==0x80018E68u?&_c2:pc==0x8001920Cu?&_c3:pc==0x80018DB0u?&_c4:pc==0x80018D1Cu?&_c5:pc==0x80018CC8u?&_c6:&_c7;
          (*c)++;
          if(*c<=3){
            u32 s56=0,s60=0; guest_read32(g_cpu.gpr[13]-31568u, &s56); guest_read32(g_cpu.gpr[13]-31460u, &s60);
            fprintf(stderr,"[dvdsm] %s r3=%u r4=0x%08X st=%u drv=%u (#%u)\n", pc==0x80018F38u?"18F38":pc==0x80018E68u?"18E68":pc==0x8001920Cu?"1920C":pc==0x80018DB0u?"18DB0":pc==0x80018D1Cu?"18D1C":pc==0x80018CC8u?"18CC8":"18D68", g_cpu.gpr[3], g_cpu.gpr[4], s56, s60, *c); }
          if((*c%2000)==0){ u32 s56=0,s60=0,cb=0,cc=0; guest_read32(g_cpu.gpr[13]-31568u, &s56); guest_read32(g_cpu.gpr[13]-31460u, &s60); guest_read32(g_cpu.gpr[13]-31488u, &cb); guest_read32(g_cpu.gpr[13]-31584u, &cc); } }
        // (fzC7 slice-loop duplicate of the host_call 18D1C probe above —
        // removed: host_call fires first and owns the [watch] tag. Kept the
        // note so nobody re-adds it.)
        // AD1C-park probe (fzAE): guest sits in a 64-bit list walk
        // (AD1C lwz r0,8(r6) / AD20 lwz r5,12(r6), subfc/subfe/neg. on CR0,
        // exit at AD38->ADD8). Dump r6 node ptr, node[8], node[12], and CR —
        // read-only. If r6/node words never change, the list is circular or
        // its successor is never written (missing DMA? missing callback?).
        // AC44-inner watcher (fzAS): ring showed the node+20 write happens
        // with pc-path AECC->AC44->AD1C->AEE0, i.e. INSIDE the AC44 allocator
        // (AD04 stw r31,0(r29) / AD0C stw r30,12(r29) / AD10 stw r25,8(r29)
        // publish block). Log regs at AC44 entry + the publish values, so we
        // learn what key the node is filed under (r30/r25 vs node12 drift).
        // fzD1: AC44 r29 = r3 (AC58 or r29,r3,r3) — the NEW-NODE pointer is
        // the caller's r3. Dump r3's target words (+8/+12 key, +16/+20 links)
        // to see whether the caller passes a FRESH node or the TAIL itself.
        // fzD7: also dump r5 (AED0 adde r5,r29,r3 result) + xer[CA] at AC44
        // entry: DVD path's AECC addc wraps (r6=0x80000000, CA=1?) vs healthy
        // no-carry. If DVD r5 != 0 while healthy r5 == 0, the wrapped carry
        // poisoned the key high word and AC44 files under a corrupt key.
        if(pc==0x8000AC44u){
          static unsigned _v=0; _v++;
          // fzDJ: uncap fully (was first-4 + every-5M) — the DVD request is
          // the 5th AC44 entry and the old cap hid it. Log every entry.
          { uint32_t n12=0xDEADu; guest_read32(0x8015CDD8u+12u, &n12);
            uint32_t t8=0,t12=0,t16=0,t20=0; uint32_t t=g_cpu.gpr[3];
            guest_read32(t+8u,&t8); guest_read32(t+12u,&t12); guest_read32(t+16u,&t16); guest_read32(t+20u,&t20);
            fprintf(stderr,"[watch] AC44 new=r3=0x%08X k=%08X:%08X l16=0x%08X l20=0x%08X r5=0x%08X r6=0x%08X xerCA=%u node12=0x%08X lr=0x%08X tb=0x%llX (#%u)\n",
              t, t8, t12, t16, t20, g_cpu.gpr[5], g_cpu.gpr[6], (g_cpu.xer>>29)&1u, n12, g_cpu.lr, (unsigned long long)g_cpu.timebase, _v); } }
        // AEC8 is `bl 1142C` with NO downcount (falls through from AEC4, no
        // dispatch point) — fzB1 confirms it never fires. Watch AECC instead
        // (addc r6,r28,r4, HAS downcount -=5, dispatched): r6 here is the
        // 64-bit key low word r28+r4 that becomes the AC44 search key.
        if(pc==0x8000AECCu){
          static unsigned _e=0; _e++;
          if(_e<=12){ uint32_t n12=0xDEADu; guest_read32(0x8015CDD8u+12u, &n12);
            fprintf(stderr,"[watch] AECC r6=0x%08X r28=0x%08X r4=0x%08X r27=0x%08X r29=0x%08X r30=0x%08X node12=0x%08X tb=0x%llX lr=0x%08X (#%u)\n",
              g_cpu.gpr[6], g_cpu.gpr[28], g_cpu.gpr[4], g_cpu.gpr[27], g_cpu.gpr[29], g_cpu.gpr[30], n12, (unsigned long long)g_cpu.timebase, g_cpu.lr, _e); } }
        // fzD2/fzD9/fzDA: AEB8 dispatches (downcount); AEDC does NOT
        // (fall-through `bl`, no downcount — same as AEC8). Counts: AEB8 7x
        // (4 healthy + 3 DVD), AECC 7x, AC44 4x (healthy only). Zero [hle]
        // exc/fallback: no exception. fzDA correction: the DVD AECC is
        // followed *in the same slice* by AD1C laps with r29(new)=CDD8 — i.e.
        // the DVD path DOES reach AC44's walk natively (chain AECC->AEDC->
        // AC44 runs without dispatch), files the tail, and parks. The "4 vs
        // 7" gap is probe visibility (AC44 probe capped at first-4), NOT a
        // divert. The walk runs; the SELF tail traps it.
        if(pc==0x8000AEB8u){
          static unsigned _b=0; if(++_b<=8) fprintf(stderr,"[watch] AEB8 lr=0x%08X r27=0x%08X tb=0x%llX (#%u)\n",
            g_cpu.lr, g_cpu.gpr[27], (unsigned long long)g_cpu.timebase, _b); }
        if(pc==0x8000AE94u){
          static unsigned _w=0; _w++;
          if(_w<=12){ uint32_t r5slot=0; guest_read32(g_cpu.gpr[1]+36u+8u, &r5slot);
            fprintf(stderr,"[watch] AE94 r6=0x%08X r3=0x%08X r5=0x%08X r7=0x%08X r30=0x%08X r31=0x%08X spSavedR29(mem)=0x%08X lr=0x%08X tb=0x%llX (#%u)\n",
              g_cpu.gpr[6], g_cpu.gpr[3], g_cpu.gpr[5], g_cpu.gpr[7], g_cpu.gpr[30], g_cpu.gpr[31], r5slot, g_cpu.lr, (unsigned long long)g_cpu.timebase, _w); } }
        // 16990's caller: backchain at AE94 entry shows who called the
        // wrapper. One-shot EABI walk dump.
        if(pc==0x8000AE94u){
          static int _b=0; if(!_b){ _b=1;
            uint32_t sp=g_cpu.gpr[1];
            fprintf(stderr,"[watch] AE94-entry backchain pc=0x%08X lr=0x%08X sp=0x%08X:", pc, g_cpu.lr, sp);
            for(int f=0;f<6;f++){ uint32_t caller=0,ret=0;
              if(!guest_read32(sp,&caller)) break;
              if(caller<=sp||!guest_read32(caller+4u,&ret)) break;
              fprintf(stderr," [0x%08X]", ret); sp=caller; }
            fprintf(stderr,"\n"); } }
        // Insert-effect check (fzBI): the AD44/AD48 stores ([r6+16]=r29,
        // [r29+20]=r6) run native mid-chain so chunk probes can't catch them —
        // but their EFFECT is visible at the next dispatched AD1C: the tail
        // node's [r6+16] should hold the new node, and new[+20] the successor.
        // Track prev-lap (r6,r29) vs current [r6+16]/[r29+20]: SELF-link iff
        // [r6+16]==r6 && [r29+20]==r29 with r6==r29.
        // fzDB: laps #1-#6 walk CA90..CB08 with CB08.next==0 (null tail) yet
        // lap #7 restarts at head INSTEAD of terminating — the null at CB08
        // should end the walk (ADDC->ADE0 falls through to ADE4 insert at
        // the tail, which is correct), but then the NEXT walk should find
        // the lengthened list and make progress, not re-walk identically.
        // The re-walk is identical because the insert's key compare keeps
        // resolving the same way: r30/r4 (search key) never change per lap.
        { static uint32_t _pr6=0,_pr29=0; static int _have=0;
          static unsigned _laps=0; static uint32_t _firstR30=0; static int _haveR30=0;
          if(pc==0x8000AD1Cu){ _laps++;
            // fzDD: capture the walk's search key (r30) on lap 1 vs lap 2+:
            // identical => the SAME request re-walks (caller re-invokes with
            // stale key); drifting => fresh requests per lap.
            // fzDE: ALSO capture r6 (walk position): laps advance CA90..CB08
            // then wrap to head — the +20 chain is walked via ADD8, so the
            // walk DOES advance; the question is what happens AFTER the tail
            // (CB08.next==0): does the next lap start at head (re-walk from
            // AC44 entry, r6=head) or continue from tail (r6=tail)?
            // fzDG finding: lap 5 r6=head AGAIN after CB08.next==0 at lap 4 —
            // and the exit block ADE4->AE80 runs natively (AE80 dispatches but
            // never fires => the chunk returns via budget BEFORE reaching it,
            // then re-dispatches at AD1C). The walk never takes the ADE4 exit
            // because the budget return always lands back at AD1C first.
            // fzDL sequencing: each AC44 walk is preceded by a FRESH 18D1C
            // callback + 16920 + AE94 (#6 follows #2-callback, not #1) — so
            // the caller DOES re-invoke per callback; laps 1-4 within ONE
            // walk are the native ADD8 advance, and the "restart at head" is
            // the NEXT callback's fresh walk, not a re-walk. The SELF tail
            // is written once (insert #5) and re-walked by later callbacks'
            // walks that keep resolving the same way (frozen key).
            if(!_haveR30){ _haveR30=1; _firstR30=g_cpu.gpr[30]; }
            if(_laps<=8||_laps%20000000==0) fprintf(stderr,"[ins] lap=%u r6=0x%08X r30=0x%08X r4=0x%08X firstR30=0x%08X %s tb=0x%llX\n",
              _laps, g_cpu.gpr[6], g_cpu.gpr[30], g_cpu.gpr[4], _firstR30,
              (g_cpu.gpr[30]==_firstR30)?"SAME-KEY":"NEW-KEY", (unsigned long long)g_cpu.timebase);
            if(_laps%5000000==0){ uint32_t head=0; guest_read32(g_cpu.gpr[13]-31800u, &head);
              fprintf(stderr,"[ins] laps=%u head=0x%08X tb=0x%llX\n", _laps, head, (unsigned long long)g_cpu.timebase); }
            if(_have){ uint32_t s16=0,s20=0;
              guest_read32(_pr6+16u,&s16); guest_read32(_pr29+20u,&s20);
              if(_pr6==_pr29){ static unsigned _n=0;
                uint32_t s16b=0; guest_read32(_pr29+16u,&s16b);
                if(++_n<=6) fprintf(stderr,"[ins] post-lap r6==r29==0x%08X [r6+16]=0x%08X [r29+20]=0x%08X [r29+16]=0x%08X %s\n",
                  _pr6, s16, s20, s16b, (s16==_pr6&&s20==_pr29)?"SELF-LINK":"linked-ok"); }
              else { static unsigned _m=0;
                // Log the +20 next field of the WALK node (r6): the insert
                // path's AD48 store ([r29+20]=r6) + AD58 ([r29+20]=r6 into
                // successor) chain should have left [r6+20] pointing at the
                // new node when the lap inserted before r6's successor.
                // (fzC1 correction: the walk link is +20, not +16; +16 is
                // the head-parent back-pointer. The earlier "stale" readings
                // sampled the wrong field.)
                if(++_m<=8){ uint32_t nx=0; guest_read32(_pr6+20u,&nx);
                  fprintf(stderr,"[ins] lap r6=0x%08X r29(new)=0x%08X [r6+20]=0x%08X %s\n",
                    _pr6, _pr29, nx, (nx==_pr29)?"INSERTED":"keep-walking"); } } }
            _pr6=g_cpu.gpr[6]; _pr29=g_cpu.gpr[29]; _have=1; } }
        // fzCV: downcount-expiry probe — log g_cpu.downcount at AD1C hits.
        // If downcount hovers just above -1000 and AD1C re-dispatches, laps
        // are budget returns (walk never exits). If it stays high, the walk
        // DOES exit via AE80 and something re-calls AC44 per lap.
        // fzCW finding: lr at AD1C is ALWAYS 0x8000AEE0 and the live
        // backchain is [AEE0][16990][1923C][7FFF0000][18CF0][189E8]... —
        // the 7FFF0000 sentinel proves the park runs INSIDE the un-returned
        // HLE 18D1C callback (trampoline preempted AC34). Laps are slice-loop
        // resumptions of that same callback body after budget returns, not
        // fresh allocator calls. The walk never reaches AE80 blr.
        if(pc==0x8000AD1Cu){
          static unsigned _p=0; _p++;
          if(_p<=6||_p%20000000==0){ uint32_t w8=0xDEADu,w12=0xDEADu,w20=0xDEADu;
            if(_p==1||_p%20000000==0) fprintf(stderr,"[park] AD1C downcount=%lld (#%u)\n", (long long)g_cpu.downcount, _p);
            guest_read32(g_cpu.gpr[6]+8u, &w8); guest_read32(g_cpu.gpr[6]+12u, &w12); guest_read32(g_cpu.gpr[6]+20u, &w20);
            // Emulate AD24-AD38 128-bit compare. CORRECTED fzBE: subfc r0 =
            // lo = r30-w12; subfe r3 = hi = r4-(w8^0x80000000)+CA; subfe
            // r3,r4,r4 folds the borrow; neg. sets EQ iff that == 0.
            // So: EQ(advance ADD8) iff lo==0 AND hi+CAborrow==0.
            uint32_t r3h = w8 ^ 0x80000000u, r4 = g_cpu.gpr[4], r30 = g_cpu.gpr[30];
            uint64_t lo = (uint64_t)r30 + (uint64_t)(~w12) + 1u; uint32_t ca = (uint32_t)(lo>>32);
            uint64_t hi = (uint64_t)r4 + (uint64_t)(~r3h) + ca;
            uint32_t hfold = (uint32_t)hi + (ca?0u:1u); // subfe r3,r4,r4 + ~CA
            int toADD8 = ((uint32_t)lo==0 && ((hfold==0&&(uint32_t)hi==0)||((uint32_t)hi==0&&(ca==0?1u:0u)==0&&(int32_t)((uint32_t)hi)==0)));
            // Simpler exact form: EQ iff lo==0 && hi==0 && CA==1 (no borrow).
            toADD8 = ((uint32_t)lo==0 && (uint32_t)hi==0 && ca==1);
            fprintf(stderr,"[park] AD1C r6=0x%08X node8=0x%08X node12=0x%08X next20=0x%08X r29(new)=0x%08X r30=0x%08X r4=0x%08X lo=%08X hi=%08X ca=%u cmp=%s lr=0x%08X (#%u)\n",
              g_cpu.gpr[6], w8, w12, w20, g_cpu.gpr[29], r30, r4, (uint32_t)lo, (uint32_t)hi, ca, toADD8?"ADD8":"ADE4", g_cpu.lr, _p); }
          // One-shot collision check: heap head (-31800(r13)) vs our thread.
          { static int _c=0; if(!_c){ _c=1;
            uint32_t head=0,alo=0,ahi=0,e4=0; int i;
            guest_read32(g_cpu.gpr[13]-31800u, &head);
            guest_read32(0x80000030u, &alo); guest_read32(0x80000034u, &ahi);
            guest_read32(0x800000E4u, &e4);
            fprintf(stderr,"[park] heap head=0x%08X r13=0x%08X arena_lo=0x%08X arena_hi=0x%08X E4(thread)=0x%08X r6=0x%08X\n",
              head, g_cpu.gpr[13], alo, ahi, e4, g_cpu.gpr[6]);
            fprintf(stderr,"[park] r6-32..+48:");
            for(i=-32;i<48;i+=4){ uint32_t w=0xDEADu; guest_read32(g_cpu.gpr[6]+(uint32_t)i, &w); fprintf(stderr," %08X", w); }
            fprintf(stderr,"\n"); } } }
        // List-shape dump (fzAT): follow next@+20 up to 8 hops from the head
        // with each node's key words (+8/+12). Circular? Linear? How long?
        // One-shot + repeat every 20M AD1C hits (list may evolve).
        if(pc==0x8000AD1Cu){
          static unsigned _ls=0; _ls++;
          if(_ls==1||_ls%20000000==0){ uint32_t head=0;
            guest_read32(g_cpu.gpr[13]-31800u, &head);
            fprintf(stderr,"[park] listshape head=0x%08X r13=0x%08X tb=0x%llX:", head, g_cpu.gpr[13], (unsigned long long)g_cpu.timebase);
            uint32_t cur=head;
            for(int h=0;h<8&&cur;h++){ uint32_t k8=0,k12=0,nx=0;
              guest_read32(cur+8u,&k8); guest_read32(cur+12u,&k12); guest_read32(cur+20u,&nx);
              fprintf(stderr," [0x%08X k=%08X:%08X nx=0x%08X]", cur, k8, k12, nx);
              if(nx==cur){ fprintf(stderr," SELF"); break; }
              if(nx==head){ fprintf(stderr," CIRCULAR"); break; }
              cur=nx; }
            fprintf(stderr," (#%u)\n", _ls); } }
        // Insert-path pcs (fzBC correction): AD3C/AD54/AD60/AE80 carry
        // downcount and DO dispatch when reached — but they only run after
        // ADE4, and ADE4 is entered ONLY via the ADE0 back-edge (downcount
        // return), never via dispatch. So the whole insert path is native;
        // the walk DOES take it (AD1C cmp=ADE4 x4). Watch ADE0 instead (the
        // back-edge, dispatched): fires once per loop lap.
        // (Re-walk AD60/AD68 probe moved to hle_host_call: those labels are
        // mid-chain natives that never START a dolrecomp_call, so slice-loop
        // if() probes can't see them; host_call runs before every chunk.)
        if(pc==0x8000ADE0u||pc==0x8000AE80u){
          static unsigned _e3=0,_e5=0;
          unsigned *c = pc==0x8000ADE0u?&_e3:&_e5; (*c)++;
          if(*c<=3||*c%5000000==0){ uint32_t head=0; guest_read32(g_cpu.gpr[13]-31800u, &head);
            fprintf(stderr,"[park] %s hit head=0x%08X r6=0x%08X r29=0x%08X r30=0x%08X downcount=%lld (#%u)\n",
              pc==0x8000ADE0u?"ADE0(back-edge)":"AE80(return)",
              head, g_cpu.gpr[6], g_cpu.gpr[29], g_cpu.gpr[30], (long long)g_cpu.downcount, *c); } }
        // 1142C/1140C = sync primitives called INSIDE the park path (fzAN:
        // node12's first write happens with pc=1142C). Dispatched (chunk_3
        // entries). Dump regs + node12 at each hit — which call in the park
        // sequence writes node12, and with what value?
        if(pc==0x8001142Cu||pc==0x8001140Cu||pc==0x8001146Cu){
          static unsigned _s=0; _s++;
          // fzD3: uncap + split by lr — count how many 1142C entries come
          // from AECC (lr=AECC, the wrapper's timebase call) vs elsewhere.
          // AECC fires 7x but no lr=AECC entry seen in first-12 cap.
          { static unsigned _ae=0; if(pc==0x8001142Cu&&g_cpu.lr==0x8000AECCu){ _ae++;
            if(_ae<=6) fprintf(stderr,"[park] 1142C-from-AECC lr=0x%08X (#ae=%u, #s=%u)\n", g_cpu.lr, _ae, _s); } }
          if(_s<=12){ uint32_t n12=0xDEADu; guest_read32(0x8015CDD8u+12u, &n12);
            fprintf(stderr,"[park] %s r3=0x%08X r4=0x%08X r27=0x%08X r29=0x%08X r30=0x%08X r31=0x%08X node12=0x%08X lr=0x%08X (#%u)\n",
              pc==0x8001142Cu?"1142C":pc==0x8001140Cu?"1140C":"1146C",
              g_cpu.gpr[3], g_cpu.gpr[4], g_cpu.gpr[27], g_cpu.gpr[29], g_cpu.gpr[30], g_cpu.gpr[31], n12, g_cpu.lr, _s); } }
        // 14168/1416C/14170/1417C = upper-layer flag check (0x800030CE==0x8200?).
        // 14170 never dispatched in prior runs — read-only dump of the flag +
        // block word0 to learn which side the branch takes.
        if(pc==0x80014168u||pc==0x8001416Cu||pc==0x80014170u||pc==0x8001417Cu||pc==0x80014180u){
          static unsigned _f1=0,_f2=0,_f3=0,_f4=0,_f5=0;
          unsigned *c = pc==0x80014168u?&_f1:pc==0x8001416Cu?&_f2:pc==0x80014170u?&_f3:pc==0x8001417Cu?&_f4:&_f5; (*c)++;
          if(*c<=3){ uint32_t fl=0xDEADu, w0=0xDEADu; guest_read32(0x800030CEu-1u+1u, &fl); guest_read32(0x8015BF20u, &w0);
            // flag is a halfword at 0x800030CE; read enclosing word aligned
            uint32_t alg=0; guest_read32(0x800030CCu, &alg);
            fprintf(stderr,"[dvdsm] %s r0=0x%08X r3=0x%08X r4=0x%08X flag30CC=0x%08X blk0=0x%08X (#%u)\n",
              pc==0x80014168u?"14168":pc==0x8001416Cu?"1416C":pc==0x80014170u?"14170":pc==0x8001417Cu?"1417C":"14180",
              g_cpu.gpr[0], g_cpu.gpr[3], g_cpu.gpr[4], alg, w0, *c); }
          if((*c%20000)==0) fprintf(stderr,"[dvdsm] flagcheck 14168=%u 1416C=%u 14170=%u 1417C=%u 14180=%u\n", _f1,_f2,_f3,_f4,_f5); }

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
        // HLE callback trampoline (fzAD finding): the queued guest callback
        // runs to completion INLINE here — the chunk executes 18D1C fully
        // (downcount-driven return runs the whole native chain including
        // 18D68/18DB0 interior labels). `continue` then resumes the SAVED
        // context after the callback already ran; the next-iteration pc IS
        // the callback only for poll()'s bookkeeping, not for execution.
        // (The old code was correct all along; interior labels never
        // dispatch by design — no bug here.)
        if(dol_hle_poll_callback(&g_cpu)){
          uint32_t cbpc = g_cpu.pc;
          { static unsigned _t=0; if(++_t<=2) fprintf(stderr,"[cb] trampoline cb=0x%08X from pc=0x%08X\n", cbpc, pc); }
          dolrecomp_call(&g_cpu, cbpc);
          continue; }
        // First-dispatch trace: log each never-before-dispatched pc once.
        // Ring of last 64 + ever-total: early boot saturates any first-N
        // cap (384 unique in minutes), so keep a sliding window over the
        // frontier instead. GX-family entry is the M3 signal.
        { static uint32_t _seen[2048]; static int _nseen=0; static int _logged=0;
          int _f=0; for(int _i=0;_i<_nseen && _i<2048;_i++) if(_seen[_i]==pc){ _f=1; break; }
          if(!_f){ if(_nseen<2048) _seen[_nseen++]=pc;
            if((_nseen%16)==1){
              fprintf(stderr,"[new] pc=0x%08X lr=0x%08X (uniq=%d)\n", pc, g_cpu.lr, _nseen); } } }
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
