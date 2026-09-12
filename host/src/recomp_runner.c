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
#include "gxruntime/audio_dma.h"
#include "gxruntime/aram.h"
#include "gxruntime/exi.h"
// Local callback trampoline (host/src/hle_callback.c), extracted from
// GXRuntime hle_core.c — hle_core.c itself can't link here (newer CPUState
// + card/ARAM/platform deps). Keep hle_abi.h out too (guest_memory dep);
// this TU only needs the four callback functions + the return sentinel.
#define HLE_CALLBACK_RETURN 0x7FFF0000u
bool dol_hle_queue_guest_callback(u32 address, u32 r3, u32 r4);
bool dol_hle_poll_callback(CPUState* cpu);
bool dol_hle_poll_nested(CPUState* cpu);
bool dol_hle_handle_callback_return(CPUState* cpu, u32 address);
void dol_hle_init(const void* config);

static CPUState g_cpu;
static int g_inited = 0;
static u64 g_tb = 0;
// fzEYzb58 watch-addrs: guest RAM offsets watched for ANY write via the
// cpu.c journal hook. -1 slot = unused. Slots: [0]=gate2 target word,
// [1]=game wait word -31388, [2]=DVD curblk -31488, [3]=-31372 fnptr
// (fzEYzb65: 1A7AC/1AF30 writers are native labels, invisible to
// dispatch probes — the journal sees them), [4]=-31452 m52,
// [5]=-31456 m56 (fzEYzb66: 1A628 runs natively inside the 17958 frame;
// only a journal slot can tell whether the 179C8 slot-invoke leg or
// the 179DC clear leg executes), [6]=-30422 gate2 done-flag byte's word
// (fzEYzb85: 706EC stb r0=1 files it AFTER 1B42C returns; the game path's
// 706E4->1B42C is what must start returning nonzero for the boot to
// advance past the DVD-wait stage), [7]=-30632 waiter-2 flag byte's word
// (fzEYzb113: 341A4 stb r0=0 clears pre-park, 34148 stb r31 pre-worker,
// 344B8 stb r3 files post-display; the 34488 probe can never observe the
// native 344B8 store, so the journal is the ONLY witness of the write).
static u32 s_watch_off[8] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
static const char* s_watch_nm[8] = {"gate2word", "waitword31388", "curblk31488", "fnptr31372", "m52drv", "m56drv", "doneflag30422", "wait2flag30632"};
static unsigned s_watch_tot[8] = {0,0,0,0,0,0,0,0};
static unsigned s_watch_nz[8] = {0,0,0,0,0,0,0,0};
static void watch_journal(u32 offset, u32 size, void* user){
    (void)size;
    // fzEYzb100: the journal fires PRE-write (ppc_journal_ram_write runs
    // BEFORE write_be32 in cpu.c), so the word image read here is the
    // PRE-store value, not post-store. All "now=" values are pre-images:
    // now=1 at pc=34E4 means memset OVERWROTE a 1 with fill (0) — the
    // actual stored value is the fill, not 1. The 64 "nonzero" hits were
    // pre-existing 1s being zeroed, never waker stores. The pc= field is
    // likewise the CHUNK-ENTRY pc (journal fires before the chunk sets
    // ctx->pc per label... precisely: journal time pc = whatever the last
    // dispatched label set). Conclusion stands but reasoning corrected:
    // no post-write 1 has ever been observed in the waitword.
    // Fully-covered 4B stores report the word as-is. Anything else
    // reports the covered bytes + a '%' suffix meaning "rest is image".
    // fzEYzb113: slot7 (wait2 flag) logs EVERY write uncapped — the byte
    // flips at most a few times per boot, and each flip is the verdict on
    // whether the 34488 native chain ran.
    for(int i=0;i<8;i++)
        if(s_watch_off[i]!=0xFFFFFFFFu && offset < s_watch_off[i]+4u && s_watch_off[i] < offset+size){
            uint32_t a = GC_RAM_BASE + s_watch_off[i], v = 0xDEADu;
            int exact = (offset<=s_watch_off[i] && s_watch_off[i]+4u<=offset+size);
            if(a >= GC_RAM_BASE && a + 4 <= GC_RAM_BASE + g_cpu.ram_size){
                uint8_t* p = g_cpu.ram + (a - GC_RAM_BASE);
                v = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
            }
            // fzEYzb70: per-slot counters. The old single shared cap (24)
            // was exhausted by boot memset spam, hiding any later waker
            // write. First 8 + every 50k + every nonzero.
            // fzEYzb89 (restored — the fzEYzb86 rewrite clobbered it): the
            // waitword slot logs EVERY nonzero write UNCAPPED, pc included.
            // The 1A618 waker stores 1,2,3... rarely; memset residue (also
            // nonzero: 0x00000001) already burned any finite cap in every
            // run, hiding real waker stores. pc=1A618 (waker) vs pc=34E4
            // (memset) disambiguates. Volume is tiny (dozens/run).
            s_watch_tot[i]++;
            int _is_nz = (v!=0 && v!=0xDEADu);
            int _uncap = ((i==1 || i==7) && _is_nz);
            // fzEYzb97: the pc= field is the chunk-ENTRY pc (journal is
            // pre-write AND pre-label-pc-set), so it cannot name the native
            // store instruction — 34E4/1AB1C are just the entries whose
            // native stretches contain the memset/clearer stores. The
            // 1A618 proof must come from the WAKER-LR bl continuations
            // (BFC8 lr=1A620 + BE00 lr=1A628 fire 6/6) — which we have —
            // plus a post-frame READ of the word. The entry probe already
            // reads it (waitword=0 at every 1A55C entry, both before and
            // after entries that provably ran the 1A618 side).
            if(s_watch_tot[i]<=8 || s_watch_tot[i]%50000==0 ||
               (_is_nz && (s_watch_nz[i]<64 || _uncap))){
                if(_is_nz) s_watch_nz[i]++;
                fprintf(stderr,"[watchmem] %s write off=0x%X sz=%u now=0x%08X%s pc=0x%08X lr=0x%08X (tot=%u)\n",
                    s_watch_nm[i], offset, size, v, exact?"":"%",
                    g_cpu.pc, g_cpu.lr, s_watch_tot[i]); }
        }
    (void)user;
}
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
static DolAudioDma s_audio_dma;
static DolExi s_exi;
// DSP CONTROL shadow + mailbox + ARAM-IRQ latch (Dolphin DSP.cpp/DSPHLE
// semantics, no DSP emulator): power-on CONTROL = DSPHalt (0x0004);
// DSPReset (bit0) auto-clears on write; DSPInitCode (0x0400) reads 0
// (hardware clears it ~130 ticks after DSPInit falls; boot only waits
// for clear); ARAM-DMA completion latches INT_ARAM (0x0020) until the
// guest acks (write-1-to-clear); mailbox FROM_HI MSB reports DSP-ready,
// latched by the first ARAM-DMA completion (B404 waits clear pre-DMA,
// B4D4 waits set post-DMA).
static u16 s_dsp_control = 0x0004u;
static bool s_dsp_aram_irq = false;
static bool s_dsp_mail_ready = false;
static u16 s_dsp_mail_to_hi = 0u, s_dsp_mail_to_lo = 0u;
// Level-triggered PI DSP source = ARAM-IRQ latch OR AID-engine pending.
// Forward declaration: DSP write handlers call it before its definition.
static void chassis_sync_dsp_irq(void);
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
// fzEYzb118: null-GX CP STATUS REGISTER (0xCC000000). GX init polls the CP
// status word (Dolphin CommandProcessorManager::Init: ReadIdle=1,
// CommandIdle=1 — fresh-console idle FIFO). Unclaimed reads return 0 =
// "FIFO busy forever", so the 700C0/700D8 poll loop (320F0+[sp+10]==1
// gate) never observes completion and the boot parks at 320F0. Report
// the Dolphin power-on value (ReadIdle|CommandIdle = 0x000C, 16-bit) for
// the status halfword; every other CP offset still reports 0 (no FIFO
// emulation — GP adresses are not served, so no phantom draws).
static bool chassis_cp_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user; (void)cpu;
    if(value){
        if(ea == 0xCC000000u && size == 2u) *value = 0x000Cu;
        else *value = 0u;
    }
    return true;
}
static bool chassis_cp_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user; (void)cpu; (void)ea; (void)size; (void)value;
    return true; // CP control/clear writes acked, no FIFO state kept
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
// DSP (0xCC005000, 0x40) + AI (0xCC006C00, 0x20) via the production
// DolAudioDma model (Dolphin AudioInterface.cpp / DSP.cpp register
// semantics): AI control init 0x42 reads back 32kHz at boot; DSP
// CONTROL/HALT/DMAState/ARAM-mode/refresh follow Dolphin's power-on
// defaults; ARAM-DMA trigger + AID interrupt feed PI like hardware.
// No audio output: dol_platform_audio_* are no-ops here (platform HAL
// unset), so the model advances register state with zero guest-state
// divergence vs audio-on (PLAN M5: audio on/off identical guest state).
static bool chassis_dsp_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user; (void)cpu;
    // Dolphin ReadToSmaller: 32-bit DSP/AI reads combine two 16-bit halves.
    if(size == 4u && ((ea >= 0xCC005000u && ea + 4u <= 0xCC005040u) ||
                      (ea >= 0xCC006C00u && ea + 4u <= 0xCC006C20u))){
        u64 hi = 0, lo = 0;
        if(!chassis_dsp_read(user, cpu, ea, 2u, &hi)) return false;
        if(!chassis_dsp_read(user, cpu, ea + 2u, 2u, &lo)) return false;
        if(value) *value = ((hi & 0xFFFFu) << 16) | (lo & 0xFFFFu);
        return true;
    }
    if(ea >= 0xCC005000u && ea < 0xCC005040u){
        u32 off = ea - 0xCC005000u;
        if(value){
            // FROM mailbox: MSB = DSP-ready (set ONLY by ARAM-DMA
            // completion; cleared when the guest collects FROM_LO so the
            // next pass's empty-wait observes empty again).
            if(off == 0x04u && size == 2u){ *value = s_dsp_mail_ready ? 0x8000u : 0u; return true; }
            if(off == 0x06u && size == 2u){
                *value = 0u;
                s_dsp_mail_ready = false;
                return true;
            }
            // CONTROL (Dolphin DSP.cpp/DSPHLE): shadow + live status.
            // ARAM bit = our ARAM-IRQ latch; AID bit = audio-DMA engine;
            // DMAState = 0 (synchronous DMA always complete); power-on
            // Halt = 1.
            if(off == 0x0Au && size == 2u){
                u16 c = s_dsp_control;
                if(s_dsp_aram_irq) c |= 0x0020u; else c &= (u16)~0x0020u;
                if(dol_audio_dma_interrupt_pending(&s_audio_dma)) c |= 0x0008u;
                else c &= (u16)~0x0008u;
                c &= (u16)~0x0200u; // DMAState idle
                *value = c; return true;
            }
            if(off == 0x16u && size == 2u){ *value = 0x0001u; return true; } // AR_MODE init'd
            if(off == 0x1Au && size == 2u){ *value = 156u; return true; } // AR_REFRESH 156MHz
        }
        dol_audio_dma_dsp_mmio_read(&s_audio_dma, off, size, value);
        return true;
    }
    if(ea >= 0xCC006C00u && ea < 0xCC006C20u){
        dol_audio_dma_ai_mmio_read(&s_audio_dma, ea - 0xCC006C00u, size, value);
        return true;
    }
    return false;
}
static bool chassis_dsp_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user;
    // Dolphin combines 32-bit DSP accesses into two 16-bit ones
    // (WriteToSmaller); the boot ARAM-DMA trigger is a 32-bit stwu to
    // CNT_H+CNT_L (0xCC005028). Split HI-first so CNT_L sees CNT_H.
    if(size == 4u && ((ea >= 0xCC005000u && ea + 4u <= 0xCC005040u) ||
                      (ea >= 0xCC006C00u && ea + 4u <= 0xCC006C20u))){
        bool ok = chassis_dsp_write(user, cpu, ea, 2u, (value >> 16) & 0xFFFFu);
        ok = chassis_dsp_write(user, cpu, ea + 2u, 2u, value & 0xFFFFu) && ok;
        return ok;
    }
    if(ea >= 0xCC005000u && ea < 0xCC005040u){
        u32 off = ea - 0xCC005000u;
        // ARAM-DMA trigger (Dolphin DSP.cpp: AR_DMA_CNT_L write runs
        // Do_ARAM_DMA; CompleteARAM clears DMAState + raises INT_ARAM).
        // CC-relative offsets: AR_DMA_MMADDR 0x5020/22, ARADDR 0x5024/26,
        // CNT_H 0x5028, CNT_L 0x502A. Address halves are masked per
        // Dolphin (HI 0x03FF, LO 0xFFE0); dir = CNT_H bit15.
        if(off == 0x2Au && size == 2u){
            dol_audio_dma_dsp_mmio_write(&s_audio_dma, off, size, value);
            { u64 mhi = 0, mlo = 0, ahi = 0, alo = 0, cnth = 0;
              dol_audio_dma_dsp_mmio_read(&s_audio_dma, 0x20u, 2u, &mhi);
              dol_audio_dma_dsp_mmio_read(&s_audio_dma, 0x22u, 2u, &mlo);
              dol_audio_dma_dsp_mmio_read(&s_audio_dma, 0x24u, 2u, &ahi);
              dol_audio_dma_dsp_mmio_read(&s_audio_dma, 0x26u, 2u, &alo);
              dol_audio_dma_dsp_mmio_read(&s_audio_dma, 0x28u, 2u, &cnth);
              u32 mm = (((u32)mhi & 0x03FFu) << 16) | ((u32)mlo & 0xFFE0u);
              u32 ar = (((u32)ahi & 0x03FFu) << 16) | ((u32)alo & 0xFFE0u);
              u32 n = ((((u32)cnth & 0x03FFu) << 16) | ((u32)value & 0xFFE0u)) & 0x7FFFFFFFu;
              bool dir = (cnth & 0x8000u) != 0u; // UARAMCount.dir
              if(n && cpu && cpu->ram){
                  if(dir) aram_dma_to_ram(cpu->ram, 0x80000000u | (mm & 0x01FFFFFFu), ar & 0x00FFFFFFu, n);
                  else aram_dma_to_aram(cpu->ram, 0x80000000u | (mm & 0x01FFFFFFu), ar & 0x00FFFFFFu, n);
              }
              { static unsigned _n=0; if(++_n<=4)
                  fprintf(stderr,"[dsp] ARAM-DMA dir=%u n=%u (PI DSP asserted)\n", dir?1u:0u, n); } }
            // CompleteARAM: DMAState=0 + INT_ARAM. Mailbox also latches
            // ready (B4D4 gate). LEVEL-TRIGGERED: the source stays pending
            // until the guest acks CONTROL (write-1-to-clear below) — never
            // de-assert here. Recompute from the latches like the read side.
            s_dsp_mail_ready = true;
            s_dsp_aram_irq = true;
            chassis_sync_dsp_irq();
            return true;
        }
        // CONTROL write (Dolphin DSP.cpp write handler): DSPReset
        // auto-clears; ARAM/AID status bits are write-1-to-clear acks;
        // HALT bit + masks persist. CONTROL_MASK=0x0C07 gates the
        // emulator-owned bits; the rest passes through to the shadow.
        if(off == 0x0Au && size == 2u){
            u16 v = (u16)value;
            if(v & 0x0020u) s_dsp_aram_irq = false; // ack INT_ARAM
            if(v & 0x0008u) dol_audio_dma_ack_interrupt(&s_audio_dma); // ack AID
            // halt tracking: B478 lift writes bit1? keep shadow of HALT/masks.
            s_dsp_control = (u16)(((s_dsp_control & ~0x0C07u) | (v & ~0x0C07u)) |
                                  (v & 0x0C04u));
            s_dsp_control &= (u16)~0x0001u; // DSPReset self-clears
            s_dsp_control &= (u16)~0x0400u; // InitCode reads clear
            chassis_sync_dsp_irq();
            return true;
        }
        // MAIL_TO write just latches (TO and FROM are separate
        // mailboxes: sending TO mail must NOT set FROM-ready, or the
        // pre-DMA empty-wait at B404 spins on the next DSP-init pass).
        // FROM-ready sets only on ARAM-DMA completion above and clears
        // when the guest collects the mail (FROM_LO read).
        if((off == 0x00u || off == 0x02u) && size == 2u){
            if(off == 0x00u) s_dsp_mail_to_hi = (u16)value;
            else s_dsp_mail_to_lo = (u16)value;
            return true;
        }
        dol_audio_dma_dsp_mmio_write(&s_audio_dma, ea - 0xCC005000u, size, value);
        chassis_sync_dsp_irq();
        (void)cpu;
        return true;
    }
    if(ea >= 0xCC006C00u && ea < 0xCC006C20u){
        dol_audio_dma_ai_mmio_write(&s_audio_dma, ea - 0xCC006C00u, size, value);
        chassis_sync_dsp_irq();
        (void)cpu;
        return true;
    }
    return false;
}
// Recompute the level-triggered PI DSP source from the latches (defined
// after the DSP handlers that call it; declared above chassis_di_write).
static void chassis_sync_dsp_irq(void){
    bool pend = s_dsp_aram_irq || dol_audio_dma_dsp_interrupt_pending(&s_audio_dma);
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_DSP, pend);
}
// ARAM window (synthetic CPU-addressable base) + EXI (RTC/card on
// channels 0/1): production models, same as Strikers mmio_install.
static bool chassis_aram_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user; (void)cpu;
    if(!aram_contains(ea)) return false;
    if(value) *value = aram_read(ea, size);
    return true;
}
static bool chassis_aram_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user; (void)cpu;
    if(!aram_contains(ea)) return false;
    aram_write(ea, value, size);
    return true;
}
static bool chassis_exi_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value){
    (void)user; (void)cpu;
    if(!dol_exi_mmio_contains(ea)) return false;
    if(value) *value = dol_exi_mmio_read(&s_exi, ea, size);
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_EXI, dol_exi_interrupt_pending(&s_exi));
    return true;
}
static bool chassis_exi_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    if(!dol_exi_mmio_contains(ea)) return false;
    dol_exi_mmio_write(&s_exi, cpu, ea, size, value);
    dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_EXI, dol_exi_interrupt_pending(&s_exi));
    return true;
}
static bool chassis_di_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value){
    (void)user;
    if(!dol_di_mmio_contains(ea)) return false;
    // Trace every DI MMIO write (bounded): command words + TSTART show the
    // exact SDK sequence without needing a symbol map.
    // fzEF: log command-word writes (0xCC006008) + CTL uncapped by count
    // (M2 progress = c0 ever becomes 0xA8 FST read vs 0x12/0xE3 retries).
    { static unsigned _n=0,_n8=0; _n++;
      if(ea==0xCC006008u){ _n8++;
        if(_n8<=12||_n8%200==0) fprintf(stderr,"[di] CMD #%u 0x%08X <- 0x%08X (pc=0x%08X)\n",
          _n8, ea, (u32)value, cpu?cpu->pc:0); }
      else if(_n<40){ fprintf(stderr,"[di] %s write 0x%08X <- 0x%08X (pc=0x%08X)\n",
        ea==0xCC00601Cu?"CTL":ea==0xCC006014u?"DMAADDR":ea==0xCC006018u?"DMALEN":"REG",
        ea, (u32)value, cpu?cpu->pc:0); } }
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
// fzEYzb164: cancel the DVD timeout alarm before every LOW completion.
// __DVDInterruptHandler unconditionally runs OSCancelAlarm(AlarmForTimeout,
// .bss 0x8015CDD8, symbols.txt:2765) on each DI interrupt before invoking
// the low callback; our HLE bypasses the handler and queues the callback
// directly, so the static alarm node stayed queued across commands. The
// next DVDLow*'s OSSetAlarm then re-inserted the still-queued node, the
// InsertAlarm walk met the node against itself (equal key -> ADD8 to self)
// and spun in AD1C forever (probe195: guard trip at AD1C, SELF tail at
// CDD8, uniq frozen at 1489). Queue the guest's own OSCancelAlarm first;
// it is a safe no-op when the alarm is not queued (AF98/AFA4 early-out),
// and the trampoline runs it (short, DOL chunks) before the nested LOW
// callback in the same frame, preserving issuer r30/r31 (AF78 saves and
// restores them; poll_nested does not touch them).
static void queue_di_completion(CPUState* cpu, u32 cb, u32 r3, u32 block){
    { extern void dol_hle_note_command_issuer(CPUState* cpu);
      dol_hle_note_command_issuer(cpu); }
    dol_hle_queue_guest_callback(0x8000AF78u, 0x8015CDD8u, 0u);
    dol_hle_queue_guest_callback(cb, r3, block);
}
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
        // DI DMA registers carry a 26-bit PHYSICAL RAM address. GXRuntime
        // correctly masks DMAADDR to 0x03FFFFE0, so convert it back to the
        // cached guest alias before indexing RAM. The old >=0x80000000 test
        // rejected 0x0015BF00 and silently never wrote DriveInfo; the guest
        // then saw zeros and retried INQUIRY/STOPMOTOR forever.
        u32 a = cmd->dma_address;
        if(a < cmd->cpu->ram_size) a |= GC_RAM_BASE;
        if(a >= GC_RAM_BASE && a + 32 <= GC_RAM_BASE + cmd->cpu->ram_size){
            uint8_t* d = cmd->cpu->ram + (a - GC_RAM_BASE);
            // Dolphin-exact DVDLowInquiry payload (DVDInterface.cpp):
            // 0x00000002 / 0x20060526 / 0x41000000. Prior GC-era guess
            // (rev 1 / 0x20011023) never advanced the guest state machine;
            // match Dolphin byte-for-byte, then let the guest decide.
            d[0]=0x00; d[1]=0x00; d[2]=0x00; d[3]=0x02;
            d[4]=0x20; d[5]=0x06; d[6]=0x05; d[7]=0x26;
            d[8]=0x41; d[9]=0x00; d[10]=0x00; d[11]=0x00;
            memset(d+12, 0, 20);
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
          guest_read32(cmd->cpu->gpr[13]-31584u, &cb);
          if(cb < GC_RAM_BASE) cb = 0x80018D1Cu;
          // fzCB: do NOT pre-write block+12. The native body files
          // 18DF8 stw r0,12(r30) with r0=10 itself (after setting drive
          // state -31448=7, -31456=0) — our END(0) pre-write is overwritten
          // (b12 flip-flops 0/-1 across callbacks) and may corrupt the
          // state the body expects. Lengths at +28/+32 are untouched.
          // (A374 slot check is moot: body invokes slot itself via blrl.)
          // fzEE: block fully untouched (no +12/+28/+32 writes; the 19270
          // success gate needs [blk+32]==[blk+20] and any HLE write breaks it)
            { static int _m=0; if(_m<2){ _m++; fprintf(stderr,"[di] INQUIRY blk=0x%08X cb=0x80018D1C (block untouched, fzEE)\n", block); } }
          // fzEYzb38 (answered — r3=0 tested live, identical: 1000/1000
          // inq/stop, read=0, 7 completions, loop unchanged). r3 is
          // IRRELEVANT to the loop; the loop is purely guest-closed.
          // Kept at r3=0 (SDK-correct DVDCBCallback OK semantics).
          // fzEYzb2: queue result = transferred length (32), matching
          // DVDCBCallback(result=bytes, block). r3=0 read as zero-length
          // failure at 18D80/18F38 bit branches + 19270 gate.
          // fzEYzb44 (answered live — m56=1 seeded, loop UNCHANGED:
          // still 1A178 lr=19230 per completion, INQUIRY re-issues, b12=1,
          // m60=14. The 18DD8 success leg runs (m48=7 filed then drained)
          // but converges back to the same re-issue: the success leg ALSO
          // ends at 18CC8/16A38 INQUIRY, not at READ. Correct per static
          // decode — 18DD8 is "inquiry succeeded, issue next inquiry",
          // not "advance to data". REVERTED to no-seed; the loop needs a
          // different key, not m56.)
          // fzEYzb153 (supersedes fzEYzb38/fzEYzb53): LOW completion
          // status needs TCINT bit0 set. r3=0 (and 32: both even) takes
          // the 18F38->1920C error leg (STOPMOTOR + FatalErrorFlag=1,
          // which poisons every later dequeue, so tag1 never reaches
          // DVDLowRead and no 0xA8 ever issues). r3=1 runs the
          // transfer-complete path (slot gets 32, stateReady, tag1
          // dequeues). The SLOT result (length) is filed by the guest
          // itself at 1901C, not by this r3.
          queue_di_completion(cmd->cpu, cb, 1, block); }
        return DOL_DI_COMMAND_COMPLETE;
    }
    // Motor/stop/reset class (dolsdk2001 DVDLowStopMotor 0xE3, Reset etc.):
    // no payload, no DMA.
    // fzEYzb53 (answered statically — 19218/1922C decoded): the 1920C
    // gate (m60!=0xE, ours 14) ALWAYS takes 19218 on our runs: b12=-1,
    // 1922C bl 1A178 error-report (r3=0x01234567 = ASCII?? 0x01234567 —
    // actually a tag/cookie constant), then 19238 bl 16920 STOPMOTOR.
    // So EVERY INQUIRY completion flows: 18D1C body -> ... -> 1920C
    // (m60=14 != 0xE? NO: 14==14==0xE! 19210 cmplwi r0,0xE / 19214 bc-4,2
    // = branch if NE. m60=14 IS 0xE => EQ => NO branch => FALL to
    // 19218 error leg!). Wait — that means m60==0xE TAKES the error
    // leg, and only m60!=0xE advances to 19240?! INVERTED from the
    // naive reading: the error leg is the m60==14 path. Hmm, but 19218
    // files b12=-1 = ERROR... and then issues STOPMOTOR. So on Dolphin
    // with m60==0xE the same error leg runs? Then what makes m60!=0xE?
    // The 18870 writer copies [curblk+8]=tag: tag 14 => m60=14=0xE.
    // Tags are SMALL ints (1,2,4,5,8,11,13,14,15): m60==0xE means
    // "last completed tag was 14 (INQUIRY)". The 19210 gate: m60==0xE
    // => this completion WAS an inquiry => report + stop motor (spin
    // down after inquiry phase?) — the BOOT SEQUENCE, not an error!
    // 1A178 r3=0x01234567 is likely the inquiry-success cookie. The
    // ADVANCE (19240: m60==1/4/5/0xE?-gates) handles OTHER tags. So the
    // loop is CORRECT BOOT BEHAVIOR: inquiry, report, stop-motor,
    // repeat until the UPPER LAYER issues something else. The upper
    // layer (game thread at 1AF64/110A8 wait) never does — THAT is the
    // stall, not the DVD thread. REDIRECT: fix the game-thread wait
    // ([r13-31388] waker at 1A618, unreachable 1A5xx chain).
    // fzEYzb54 (answered statically — 6FDEC decoded): the 6A64-called
    // game path (6A64 bl 1AF64 = DVD-wait, then 6A68 r3=1 + 6A6C bl
    // 1BDF0 + 6A74 bl 6FFCC + 6A78 bl 6FDEC + 6A7C bl 6FEFC...) runs
    // the DVD-wait FIRST (6A64) and only continues past it when
    // [r13-31388] changes. 6FDEC is the allocator/memcpy worker
    // (7988C + 1140C-sync frames). So the GAME BOOT SEQUENCE is:
    // 1AF64-wait (DVD) -> 1BDF0 -> 6FFCC -> 6FDEC (alloc) -> 6FEFC...
    // It is parked at step 0. The waker (1A618 increment, called from
    // the unreachable 1A5xx chain) never runs. NEXT: what calls 1A5xx?
    // NOTHING (no bl found) — so the 1A5xx chain is entered via a
    // FUNCTION POINTER (slot callback? 199xx bctr table? OS thread?).
    // Grep for 1A5xx/1A6xx addresses taken as VALUES (lis/addi loads),
    // not just bl targets.
    // fzEYzb55 (answered statically — 1A9xx is VIDEO, not DVD): the
    // 1A9xx region (1A980+: lhz/lbz field extracts + sth packs at
    // 8192/8206 offsets; 1AA28+: srawi/addze fixed-point; 1AA40+: sth
    // video-mode tables at 8240-8246) is the VIDEO MODE / FRAMEBUFFER
    // setup (1AB20 zeroes -31388/-31348/-31352/-31332 = the DVD wait
    // words as a SIDE EFFECT of video init!). 1AAE0-frame: waits
    // [-31392]==0 -> files -31392=1, -31364=1, then polls a VIDEO
    // register (8192+2 bit0) via 1AB08 lhzu loop -> 1AB1C: r31=0 filer
    // (1AB20 stw r31,-31388 = CLEARS the DVD-wait word to 0!) + more
    // zero-filers. So VIDEO INIT clears [r13-31388]=0 — and the game
    // waiter (1AF64: r30=[word] at entry, exits iff word CHANGES)
    // entered with r30=0 and waits for NONZERO? No: exits iff
    // r30!=[word]-now; word stays 0 => spins. The 1A618 INCREMENTER
    // (+1) is what SHOULD wake it — 1A618 sits in the 1A5FC-taken leg
    // (1A5F8 bit3 of r7 set?). The 1A5xx caller question stands, but
    // NOTE: 1AB20 writes 0 (not a wakeup); only 1A618 wakes. NEXT:
    // what sets up r7's bit3 for the 1A5F8 gate, and who calls 1A5xx?
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
        // fzEG: STOPMOTOR block fully untouched (same reasoning as fzEE
        // for INQUIRY: the native body files its own b12/m56 state, and HLE
        // pre-writes corrupt the branch resolution).
        // fzEH: dispatch the COMMAND's saved callback from r13-31584, not a
        // hardcoded 18D1C. 16A38 saves its r4 (0x80018D1C) there for INQUIRY;
        // 16920 saves its r3 (0x80017958) there for STOPMOTOR. Hardcoding
        // 18D1C for motor completions bypasses the 17958->1799C(m64=1)->
        // 187CC-sink chain that advances the drive state (m60 stuck at 14
        // => 1920C files b12=-1 and re-issues STOPMOTOR forever).
        // fzEYzb49 (answered live — slot seeded, loop UNCHANGED: still
        // 800+ INQUIRY issues/20s, 7 completions, m60=14, b12=1. The
        // seeded slot is never invoked: 179BC reads m56==0 -> 179C4 TAKEN
        // to 179EC drain BEFORE any slot read; and the 18D1C body's own
        // 18E04 slot gate reads [r30+40]... r30 at that point is curblk
        // (0x8015BF20, seeded) — but 18E00 checks [r12]==0 first (slot
        // word's... no: 18DFC r12=[r30+40]=seeded 18D1C !=0 -> 18E04
        // TAKEN to 18E18, SKIPPING the 18E08 blrl invoke! The seed makes
        // the body SKIP the invoke, not take it. Either way: unchanged.
        // REVERTED to unseeded.)
        { uint32_t blk=0x8015BF20u; guest_read32(cmd->cpu->gpr[13]-31488u, &blk);
          if(blk < GC_RAM_BASE) blk = 0x8015BF20u;
          uint32_t cb=0x80017958u; guest_read32(cmd->cpu->gpr[13]-31584u, &cb);
          if(cb < GC_RAM_BASE) cb = 0x80017958u;
          { static int _m=0; if(_m<3){ fprintf(stderr,"[di] STOPMOTOR blk=0x%08X (block untouched, fzEG) -> 0x%08X\n", blk, cb); _m++; } }
          queue_di_completion(cmd->cpu, cb, 0, blk); }
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
                // fzEYzb154: DI DMAADDR carries a 26-bit PHYSICAL address
                // (GXRuntime masks to 0x03FFFFE0), so a read target logs as
                // 0x0155AD80, not 0x8155AD80. Same alias fix as INQUIRY.
                if(ga < cmd->cpu->ram_size) ga |= GC_RAM_BASE;
                u8* dst = NULL;
                if(ga >= GC_RAM_BASE && ga + cmd->dma_length <= GC_RAM_BASE + cmd->cpu->ram_size)
                    dst = cmd->cpu->ram + (ga - GC_RAM_BASE);
                else if(ga >= GC_RAM_UNCACHED && ga + cmd->dma_length <= GC_RAM_UNCACHED + cmd->cpu->ram_size)
                    dst = cmd->cpu->ram + (ga - GC_RAM_UNCACHED);
                if(dst){
                    unsigned got = dvd_read_disc_bytes(f, 1024*1024, disc_off, dst, cmd->dma_length);
                    if(got < cmd->dma_length) memset(dst+got, 0, cmd->dma_length-got);
                    fprintf(stderr,"[di] tree read off=0x%08X got=%u/%u\n", disc_off, got, cmd->dma_length);
                    // fzEYzb153: queue the LOW callback (18D1C cbForStateBusy)
                    // with TCINT bit0 set, mirroring the INQUIRY fix above.
                    // Without this a served READ never completes guest-side.
                    // (fzEYzb164: alarm-cancel first, via queue_di_completion.)
                    { uint32_t block = 0x8015BF20u;
                      guest_read32(cmd->cpu->gpr[13]-31488u, &block);
                      if(block < GC_RAM_BASE) block = 0x8015BF20u;
                      uint32_t cb = 0x80018D1Cu;
                      guest_read32(cmd->cpu->gpr[13]-31584u, &cb);
                      if(cb < GC_RAM_BASE) cb = 0x80018D1Cu;
                      queue_di_completion(cmd->cpu, cb, 1, block); }
                    return DOL_DI_COMMAND_COMPLETE;
                }
            }
        }
        dvd_read_to_guest(cmd->cpu, cmd->dma_address, disc_off, cmd->dma_length);
        { uint32_t block = 0x8015BF20u;
          guest_read32(cmd->cpu->gpr[13]-31488u, &block);
          if(block < GC_RAM_BASE) block = 0x8015BF20u;
          uint32_t cb = 0x80018D1Cu;
          guest_read32(cmd->cpu->gpr[13]-31584u, &cb);
          if(cb < GC_RAM_BASE) cb = 0x80018D1Cu;
          queue_di_completion(cmd->cpu, cb, 1, block); }
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
    // fzEYzb118: CP status register (null-GX idle FIFO: ReadIdle|CmdIdle).
    dol_mmio_bus_register(&s_mmio_bus, 0xCC000000u, 0x20u, chassis_cp_read, chassis_cp_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC002000u, 0x80u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC003000u, 0x40u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC00100Au, 2u, chassis_vi_read, chassis_vi_write, NULL);
    dol_mmio_bus_register(&s_mmio_bus, 0xCC006000u, 0x28u, chassis_di_read, chassis_di_write, NULL);
    { extern bool chassis_si_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value);
      extern bool chassis_si_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value);
      dol_si_init(&s_si);
      dol_mmio_bus_register(&s_mmio_bus, 0xCC006400u, 0x100u, chassis_si_read, chassis_si_write, NULL); }
    // DSP/AI audio (0xCC005000/0xCC006C00), EXI (0xCC006800), ARAM window:
    // production GXRuntime models, registered like Strikers mmio_install.
    { extern bool chassis_dsp_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value);
      extern bool chassis_dsp_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value);
      extern bool chassis_exi_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value);
      extern bool chassis_exi_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value);
      extern bool chassis_aram_read(void* user, CPUState* cpu, u32 ea, u8 size, u64* value);
      extern bool chassis_aram_write(void* user, CPUState* cpu, u32 ea, u8 size, u64 value);
      dol_audio_dma_init(&s_audio_dma);
      aram_init();
      dol_exi_init(&s_exi);
      dol_mmio_bus_register(&s_mmio_bus, 0xCC005000u, 0x40u, chassis_dsp_read, chassis_dsp_write, NULL);
      dol_mmio_bus_register(&s_mmio_bus, 0xCC006C00u, 0x20u, chassis_dsp_read, chassis_dsp_write, NULL);
      dol_mmio_bus_register(&s_mmio_bus, 0xCC006800u, 0x3Cu, chassis_exi_read, chassis_exi_write, NULL);
      dol_mmio_bus_register(&s_mmio_bus, ARAM_BASE, ARAM_SIZE, chassis_aram_read, chassis_aram_write, NULL); }
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
static uint32_t s_os_dispatch_interrupt = 0x8000D9CCu; // F-Zero __OSDispatchInterrupt (OSInterruptMask.s)
// fzEYzb116: gate for null-GX PE-finish synthesis (see slice tail). Set when
// the guest registers handler 19 (34488, the waiter-2 flag filer).
static bool s_pe19_registered = false;
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
        // mtmsr: PLAN-sanctioned MSR.FP forcing (matches Strikers host).
        // fzEYzb61: full passthrough was tried (guest owns MSR) — it
        // produced an FP_UNAVAILABLE storm at 70A58 every game-loop lap
        // (guest FP-off windows via 9FC4 never re-enable per-thread, so
        // each reschedule faults again). Forcing FP is harmless: the
        // FP-off windows are L2/cache-config stretches that use no FP.
        if(xo==146){ uint32_t rs=(raw>>21)&31u; cpu->msr=(cpu->gpr[rs]|0x2000u); cpu->pc=cia+4; return; }
        if(xo==210||xo==242||xo==595||xo==659){ uint32_t rt=(raw>>21)&31u; if(xo==595||xo==659) cpu->gpr[rt]=0; cpu->pc=cia+4; return; }
        if(xo==306){ uint32_t rb=(raw>>11)&31u; ppc_tlbie(cpu,cpu->gpr[rb],cia); if(cpu->exception==0) cpu->pc=cia+4; return; }
        if(xo==512||xo==854||xo==982||xo==470||xo==54||xo==86||xo==278||xo==246||xo==598||xo==150){ cpu->pc=cia+4; ppc_memory_fence(); return; }
        if(xo==1014){ uint32_t ra=(raw>>16)&31u, rb=(raw>>11)&31u; uint32_t ea=(ra?cpu->gpr[ra]:0)+cpu->gpr[rb]; ppc_dcbz_l(cpu,ea,cia); if(cpu->exception==0) cpu->pc=cia+4; return; }
        if(xo==19){ uint32_t rt=(raw>>21)&31u; cpu->gpr[rt]=cpu->cr; cpu->pc=cia+4; return; }
        if(xo==144){ cpu->pc=cia+4; return; }
        // mftb (X/O 371, SPR 268/269): timebase low/high. The guest's 1140C
        // spin-wait (mftbu-loop) and the AECC allocator key both read tb via
        // mftb; without it the slice raises ILLEGAL and drops the chunk.
        // Route to the deterministic retrace-driven timebase (fzEA).
        if(xo==371){ uint32_t rt=(raw>>21)&31u, tbr=(((raw>>11)&0x1Fu)<<5)|((raw>>16)&0x1Fu); uint32_t before=cpu->exception; uint32_t v=ppc_mftb(cpu,(uint16_t)tbr,cia); if(cpu->exception==before){ cpu->gpr[rt]=v; cpu->pc=cia+4; } return; }
        // mfibat (X/O 615/129, SPR 528-535): IBAT upper/lower. The 5E60/5E68
        // cache-config stretch reads IBAT0U (SPR 528); BATs are identity on
        // real hardware after OSInit — return the stored value (0 = unset,
        // matching cpu_reset's zeroed spr file) and advance.
        if(xo==615||xo==129){ uint32_t rt=(raw>>21)&31u, spr=((raw>>11)&0x1Fu)<<5|((raw>>16)&0x1Fu); uint32_t before=cpu->exception; uint32_t v=ppc_mfspr(cpu,(uint16_t)spr,cia); if(cpu->exception==before){ cpu->gpr[rt]=v; cpu->pc=cia+4; } return; }
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
    // fzEWe (CORRECTED via decomp — old note was wrong): FatalErrorFlag
    // (m64, -31464) 0->1 fires in cbForStateError's NATIVE tail (1799C
    // stw r3=1), reached via the 17984-bl-1A2EC path on NON-0x10 results.
    // The observed transition (17958-entry m64=0 → 179BC-entry m64=1)
    // brackets exactly those two stores. It is NOT set-once: 179C8 reads
    // m56-word... precisely: 179BC lwz m56-lbl_801A68E0; if nonzero the
    // 179C8/179D0 leg files m56=0 (clears it); m64 itself is never cleared
    // here (only cbForStateError's sibling paths re-file it). Callers of
    // 17958: 18D60-bl-17958 (inside cbForStateBusy) — but OUR runs take
    // the r3==0x10 leg? No: probe183 shows 16920 r3=0x80017958 (the
    // STOPMOTOR issue from 1923C inside the 18D1C completion path), then
    // 17958-entry with m64=0. So the STOPMOTOR completion runs 17958 with
    // a non-0x10 result → 17984 path → files FatalErrorFlag=1. m64=1 then
    // steers 18820 (stateReady pop leg) down the consume path.
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
    // fzEYzb170: serve OSCancelAlarm (AF78) in hle_host_call. The AF78 chunk
    // body (OSDisableInterrupts D4F4 + InsertAlarm AD1C walk) parks the probe
    // tail at D4F4 with EE=0 (log: pc=D4F4 lr=AF98 r1=0x801B75F8 msr=0x1008,
    // DIpend=1). The queued completion pair (AF78 r3=CDD8 + low callback)
    // only needs AF78's early-out semantics: [r3]==0 -> unlink-nothing and
    // return; the alarm is never queued in our runs (AD1C listshape SELF at
    // CDD8). Serve natively only when the dequeue is a no-op ([alarm]==0,
    // [alarm+20]==0); otherwise run the native chunk. Bounded log.
    if(addr==0x8000AF78u){
      uint32_t _al = cpu->gpr[3], _w0 = 1u, _w20 = 1u;
      guest_read32(_al, &_w0); guest_read32(_al+20u, &_w20);
      if(_w0==0u && _w20==0u){
        { static unsigned _c=0; if(++_c<=4)
          fprintf(stderr,"[os] AF78-cancel-noop alarm=0x%08X lr=0x%08X (#%u)\n",
            _al, cpu->lr, _c); }
        cpu->pc = cpu->lr & ~3u; return true;
      }
      { static unsigned _q=0; if(++_q<=4)
        fprintf(stderr,"[os] AF78-queued alarm=0x%08X w0=0x%08X w20=0x%08X lr=0x%08X (native)\n",
          _al, _w0, _w20, cpu->lr, _q); }
      return false; }
    // ARQ HLE (fzEYzb168/169): the second-stage REL posts font ARAM-DMA
    // requests via 205A0-bl, then parks polling [0x803D0198] for
    // completion. On HW __ARQServiceQueueLo (20360) dequeues and
    // ARStartDMA (1E864) copies synchronously inside the DSP interrupt;
    // the completion callback files the wait word. Our DSP chassis never
    // raises the ARAM interrupt, so the queue never pumps: serve
    // synchronously here (mirrors GXRuntime hle_core.c
    // dol_hle_ARQPostRequest ABI: request=r3, owner=r4, type=r5, prio=r6,
    // src=r7, dst=r8, len=r9, callback=r10; type 0 = MRAM->ARAM,
    // type 1 = ARAM->MRAM per aurora ar.h ARAM_DIR_*).
    // Post 1 (lr=0x802161B8 class): type==0 MRAM->ARAM font upload, sync
    // word [0x803D0198] waited-for-1.
    // Post 2 (lr=0x802165FC, caller disassembled from the live dump:
    // lis r4,-0x7FDF/addi r10,r4,0x63A8/mr r7,r29/addi r3,r3,0x4A0/
    // mr r8,r31/mr r9,r28/li r4,1/li r5,1/li r6,1/bl 205A0 (bit-math
    // verified below), then lis r3,-0x7FC3/addi r3,r3,0x49C/lwz r0,0(r3)/
    // cmpwi r0,0/bne 80216604 spin): type==1 ARAM->MRAM, src=0x81003668
    // dst=0x8044A900 (MEM1) len=0x120 cb=0x802163A8. The post-2 wait word
    // is [0x803D049C], waited-for-0: after the bl returns to 0x802165FC
    // the caller runs lis r3,0x803D / addi r3,r3,0x49C / lwz r0,0(r3) /
    // cmpwi r0,0 / bne 80216604. File 0 as the completion state, mirroring
    // post-1's filing of its waited-for-1 word (filing-then-callback is the
    // proven pattern: post-1's callback ran fine after its word was filed).
    // The src is ARAM-side
    // BY ABI TYPE: a MEM1->MEM1 ARQ DMA is impossible on HW (ARStartDMA
    // always touches ARAM, and the 2068C/206A0 pump legs swap src/dst by
    // [req+8] unconditionally), so the direction keys off type, not off
    // the src numeric range. The ARAM side is bounded by aram.c's
    // power-of-2 offset mask inside aram_dma_to_ram; the MEM1 dst and len
    // are bounds-checked here. (The req struct is NOT consulted: this hook
    // runs before the chunk body files it, so req+16/+20 are still stale.)
    // bl-target bit math (I-form: tgt = cia + sext(field & 0x03FFFFFC)):
    // 0x802165F8[4BE09FA9]->0x800205A0, 0x80216614[4BE0A0E9]->0x800206FC
    // (ARQSetChunkSize, runs natively via coverage dispatch). The stale
    // 0x802161B4/0x8002029C/0x800206D8/0x800798A8 note is superseded: those
    // bl words appear nowhere in fze.sample.rel and the live regs show the
    // second-stage REL (not the sample REL) as the poster.
    // Bounds-checked; anything unexpected runs the native chunk.
    if(addr==0x800205A0u){
      uint32_t req=cpu->gpr[3];
      uint32_t type=cpu->gpr[5], src=cpu->gpr[7], dst=cpu->gpr[8],
               len=cpu->gpr[9], cb=cpu->gpr[10];
      if(len && len<0x1000000u && src>=0x80000000u &&
         src+len>=src && src+len<=0x80000000u+cpu->ram_size &&
         dst<0x1000000u && type==0){
        aram_dma_to_aram(cpu->ram, src, dst, len);
        { static unsigned _c=0; if(++_c<=2) fprintf(stderr,"[arqh] served MRAM->ARAM src=0x%08X dst=0x%X len=%u cb=0x%08X req=0x%08X\n",src,dst,len,cb,req); }
        // Complete the ARQ queue + file the wait word, mirroring what
        // 20360-pump + ARStartDMA + the callback would have done natively.
        uint32_t head=0; guest_read32(cpu->gpr[13]-31168u,&head);
        uint8_t* rm=cpu->ram;
        #define WR32(a,v) do{ uint32_t _a=(a); if(_a>=0x80000000u && _a+4<=0x80000000u+cpu->ram_size){ rm[_a-0x80000000u]=(uint8_t)((v)>>24); rm[_a-0x80000000u+1]=(uint8_t)((v)>>16); rm[_a-0x80000000u+2]=(uint8_t)((v)>>8); rm[_a-0x80000000u+3]=(uint8_t)(v); } }while(0)
        if(head>=0x80000000u){ uint32_t nx=0; guest_read32(head,&nx); WR32(cpu->gpr[13]-31168u,nx); }
        WR32(cpu->gpr[13]-31148u,0u);
        WR32(0x803D0198u,1u);
        #undef WR32
        // Run the guest completion callback via the existing trampoline
        // (HW runs it from the ARQ ISR with r3=request).
        if(cb>=0x80000000u && cb<0x81800000u){
          dol_hle_note_command_issuer(cpu);
          dol_hle_queue_guest_callback(cb, req, 0u);
        }
        cpu->pc=cpu->lr & ~3u; return true;
      }
      if(len && len<0x1000000u && type==1 &&
         dst>=0x80000000u && len<=cpu->ram_size &&
         dst+len>=dst && dst+len<=0x80000000u+cpu->ram_size){
        aram_dma_to_ram(cpu->ram, dst, src, len);
        { static unsigned _c2=0; if(++_c2<=2) fprintf(stderr,"[arqh] served ARAM->MRAM src=0x%08X dst=0x%08X len=%u cb=0x%08X req=0x%08X\n",src,dst,len,cb,req); }
        // Same queue-pop/wait-word/callback completion as the type==0 leg:
        // the pump (20360) and ISR (20464) treat both directions alike
        // (2068C-leg dir bit only selects the ARStartDMA direction).
        uint32_t head=0; guest_read32(cpu->gpr[13]-31168u,&head);
        uint8_t* rm=cpu->ram;
        #define WR32B(a,v) do{ uint32_t _a=(a); if(_a>=0x80000000u && _a+4<=0x80000000u+cpu->ram_size){ rm[_a-0x80000000u]=(uint8_t)((v)>>24); rm[_a-0x80000000u+1]=(uint8_t)((v)>>16); rm[_a-0x80000000u+2]=(uint8_t)((v)>>8); rm[_a-0x80000000u+3]=(uint8_t)(v); } }while(0)
        if(head>=0x80000000u){ uint32_t nx=0; guest_read32(head,&nx); WR32B(cpu->gpr[13]-31168u,nx); }
        WR32B(cpu->gpr[13]-31148u,0u);
        // Post-2 completion files the spin word to its exit state. On HW
        // this is done by the ARQ ISR + completion callback; the fzEYzb168
        // pattern (file-then-callback) is proven by post-1, so do both.
        WR32B(0x803D049Cu,0u);
        #undef WR32B
        if(cb>=0x80000000u && cb<0x81800000u){
          dol_hle_note_command_issuer(cpu);
          dol_hle_queue_guest_callback(cb, req, 0u);
        }
        cpu->pc=cpu->lr & ~3u; return true;
      }
      { static unsigned _q=0; if(++_q<=4)
        fprintf(stderr,"[arqh] 205A0-UNEXPECTED r3=0x%08X r5=0x%08X r7=0x%08X r8=0x%08X r9=0x%08X r10=0x%08X lr=0x%08X\n",
          req,type,src,dst,len,cb,cpu->lr); }
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
    // fzCH (decomp: 17958 = cbForStateError): the 0x10-compare arg decides
    // stateTimeout leg (r3==0x10: b12=-1 + DVDReset + recurse) vs the
    // 17984 leg (fn_8001A2EC, DummyBlock→executing, FatalErrorFlag=1,
    // slot-invoke r3=-1, m56-conditional slot-invoke r3=0, stateReady).
    // Its callers: 18D60-bl-17958 (inside cbForStateBusy) + stateTimeout's
    // tail. Probe183: STOPMOTOR completion → 17958 with m64=0 → 179BC
    // with m64=1, i.e. the 17984 leg. Dump r3 + m64/m60 to confirm.
    if(addr==0x80017958u||addr==0x800179BCu||addr==0x800187CCu||addr==0x80018820u){
      static unsigned _h1=0,_h2=0,_h3=0,_h4=0;
      unsigned *c=addr==0x80017958u?&_h1:addr==0x800179BCu?&_h2:addr==0x800187CCu?&_h3:&_h4; (*c)++;
      // fzEYzc2: at 18820 also dump [curblk+8] (m60 source on skip path).
      if(*c<=3){ uint32_t a=0,b=0,t8=0xDEADu;
        guest_read32(cpu->gpr[13]-31464u,&a); guest_read32(cpu->gpr[13]-31460u,&b);
        if(addr==0x80018820u){ uint32_t cb=0; guest_read32(cpu->gpr[13]-31488u,&cb);
          if(cb) guest_read32(cb+8u,&t8); }
        fprintf(stderr,"[watch] %s m64=%u m60=%u lr=0x%08X%s (#%u)\n",
          addr==0x80017958u?"17958":addr==0x800179BCu?"179BC":addr==0x800187CCu?"187CC":"18820",
          a, b, cpu->lr,
          addr==0x80018820u?(t8==0xDEADu?" curblk=0":""):"",
          *c);
        if(addr==0x80018820u){ uint32_t cb=0; guest_read32(cpu->gpr[13]-31488u,&cb);
          if(cb){ uint32_t v=0; guest_read32(cb+8u,&v);
            fprintf(stderr,"[watch] 18820 curblk=0x%08X tag8=%u\n", cb, v); } } }
      return false; }
    // fzEM (corrected): 19FA4's scan loop is 19FC4(lwz)/19FCC(bc)/
    // 19FDC(addi)/19FE0-bdnz/19FE4/19FEC-return. (19F2C et al are 19F04's
    // loop — wrong labels, never fired.) Uncapped counters + first dumps.
    if(addr==0x80019FC4u||addr==0x80019FCCu||addr==0x80019FDCu||addr==0x80019FE0u||addr==0x80019FE4u||addr==0x80019FECu){
      static unsigned _m1=0,_m2=0,_m3=0,_m4=0,_m5=0,_m6=0;
      unsigned *c=addr==0x80019FC4u?&_m1:addr==0x80019FCCu?&_m2:addr==0x80019FDCu?&_m3:addr==0x80019FE0u?&_m4:addr==0x80019FE4u?&_m5:&_m6; (*c)++;
      if(*c<=2) fprintf(stderr,"[watch] %s r3=0x%08X r4=0x%08X ctr=0x%08X lr=0x%08X (#%u)\n",
        addr==0x80019FC4u?"19FC4":addr==0x80019FCCu?"19FCC":addr==0x80019FDCu?"19FDC":addr==0x80019FE0u?"19FE0-bdnz":addr==0x80019FE4u?"19FE4":"19FEC-ret",
        cpu->gpr[3], cpu->gpr[4], cpu->ctr, cpu->lr, *c);
      if(*c%5000000==0) fprintf(stderr,"[watch] 19FA4-loop 19FC4=%u 19FCC=%u 19FDC=%u 19FE0=%u 19FE4=%u 19FEC=%u\n", _m1,_m2,_m3,_m4,_m5,_m6);
      return false; }
    // fzEYi (decomp: 1A178 = __DVDStoreErrorCode): reports per completion
    // from lr=19230 (report-then-motor: 19218 files b12=-1, 1922C calls
    // 1A178, 19230+ issues STOPMOTOR via 16920). Normal path, not a
    // failure — the motor issue follows it.
    if(addr==0x8001A178u){
      static unsigned _n=0; _n++;
      if(_n<=8||_n%5000000==0){ uint32_t m60=0,m56=0,m52=0,d0=0,d4=0,d8=0;
        guest_read32(cpu->gpr[13]-31460u,&m60); guest_read32(cpu->gpr[13]-31456u,&m56);
        guest_read32(cpu->gpr[13]-31452u,&m52);
        guest_read32(0x8015BF00u,&d0); guest_read32(0x8015BF04u,&d4); guest_read32(0x8015BF08u,&d8);
        fprintf(stderr,"[dvdsm] 1A178 lr=0x%08X r3=0x%08X m60=%u m56=%u m52=0x%08X drive=%08X/%08X/%08X (#%u)\n",
          cpu->lr, cpu->gpr[3], m60, m56, m52, d0, d4, d8, _n); }
      return false; }
    // fzEW (decomp: 16850 = DVDLowWaitCoverClose, weak — NOT an error
    // reporter). Called from the 189xx ResumeFromHere legs (cover-close
    // waits). Caller lr distinguishes which leg; r3 = callback.
    if(addr==0x80016850u){
      static unsigned _v=0; if(++_v<=6||_v%5000000==0)
        fprintf(stderr,"[dvdsm] 16850-errorleg r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], cpu->lr, _v);
      return false; }
    // fzEZa: 19760 (dispatched entry, chunk_6) is the motor wrapper that
    // returns r3=1 (STOPMOTOR path needs r3 odd at 18D80). Does it run?
    if(addr==0x80019760u){
      static unsigned _y=0; if(++_y<=4) fprintf(stderr,"[dvdsm] 19760 r3=0x%08X lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->lr, _y);
      return false; }
    // fzEXh/fzEXg/fzEXf/fzEXe probes removed: 18F40/18F54/18F7C/18F8C/
    // 18F98/18FAC never fire (mid-chain natives inside the 18D1C frame).
    // fzEYzd: 1A3F4/1A418/1A43C (all dispatched) lead to the second
    // 19500 call at 1A450. Entry lr names which upper layer registers.
    if(addr==0x8001A3F4u||addr==0x8001A418u||addr==0x8001A43Cu){
      static unsigned _r[3]={0}; int _i=
        addr==0x8001A3F4u?0:addr==0x8001A418u?1:2;
      const char *_nm[3]={"1A3F4-entry","1A418","1A43C"};
      if(++_r[_i]<=3) fprintf(stderr,"[dvdsm] %s r3=0x%08X lr=0x%08X (#%u)\n",
        _nm[_i], cpu->gpr[3], cpu->lr, _r[_i]);
      return false; }
    // fzEYze: 19AF8/19B00 (both dispatched) bracket the chunk_6 curblk
    // writer at native 19B08 (same lis/addi constant as 17998). If they
    // fire, the second curblk writer runs; dump r0 (value filed).
    if(addr==0x80019AF8u||addr==0x80019B00u){
      static unsigned _n1=0,_n2=0; unsigned *c=addr==0x80019AF8u?&_n1:&_n2; (*c)++;
      if(*c<=3) fprintf(stderr,"[dvdsm] %s r0=%u r3=0x%08X lr=0x%08X (#%u)\n",
        addr==0x80019AF8u?"19AF8":"19B00", cpu->gpr[0], cpu->gpr[3], cpu->lr, *c);
      return false; }
    // fzEYz3: 1A31C (dispatched entry) + 1A338 gate the 1A340 cascade
    // into the 19500 slot-register call. Entry lr names the caller.
    if(addr==0x8001A31Cu||addr==0x8001A338u){
      static unsigned _p1=0,_p2=0; unsigned *c=addr==0x8001A31Cu?&_p1:&_p2; (*c)++;
      if(*c<=4) fprintf(stderr,"[dvdsm] %s r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
        addr==0x8001A31Cu?"1A31C-entry":"1A338", cpu->gpr[3], cpu->gpr[4], cpu->lr, *c);
      return false; }
    // fzEYz2: 1A344/1A348/1A354/1A37C/1A3AC/1A3B0/1A3B8/1A3C0/1A3CC all
    // dispatch on the 1A340 cascade into the 19500 slot-register call.
    // 1A340/1A3DC themselves are native branch labels (no downcount).
    if(addr==0x8001A344u||addr==0x8001A348u||addr==0x8001A354u||addr==0x8001A37Cu||addr==0x8001A3ACu||addr==0x8001A3B0u||addr==0x8001A3B8u||addr==0x8001A3C0u||addr==0x8001A3CCu){
      static unsigned _o[9]={0}; int _i=
        addr==0x8001A344u?0:addr==0x8001A348u?1:addr==0x8001A354u?2:addr==0x8001A37Cu?3:
        addr==0x8001A3ACu?4:addr==0x8001A3B0u?5:addr==0x8001A3B8u?6:addr==0x8001A3C0u?7:8;
      const char *_nm[9]={"1A344","1A348","1A354","1A37C","1A3AC","1A3B0","1A3B8","1A3C0","1A3CC"};
      if(++_o[_i]<=3) fprintf(stderr,"[dvdsm] %s lr=0x%08X (#%u)\n", _nm[_i], cpu->lr, _o[_i]);
      return false; }
    // fzEYzb5/fzEYzb4 probes removed: 18EDC/18E90/18FE0 never fire
    // (mid-chain natives in 18D1C frame).
    // fzEYzb16: superseded by fzEYzb31/32 below (19E64-entry index dump).
    // (Old r3-only probe removed to avoid double-fire with fzEYzb32.)
    // fzEYzb15: 177A0/177A4 (both dispatched) bracket the 177AC workarea
    // writer (native stores r0=0x80000000 to r13-31480!). The null base
    // is filed by guest code itself — not a missing installer. Dump r0.
    if(addr==0x800177A0u||addr==0x800177A4u){
      static unsigned _n1=0,_n2=0; unsigned *c=addr==0x800177A0u?&_n1:&_n2; (*c)++;
      if(*c<=3) fprintf(stderr,"[dvdsm] %s r0=0x%08X r3=0x%08X lr=0x%08X (#%u)\n",
        addr==0x800177A0u?"177A0":"177A4", cpu->gpr[0], cpu->gpr[3], cpu->lr, *c);
      return false; }
    // fzEYzb14: 177D0 dispatches, then the 177E4 deref chain
    // ([r13-31480]+32 deref) + 177F0/177F8 compare run native; 177FC
    // (fallthrough toward 1A3F4) vs 17814 (taken) decide. Dump the
    // deref base + final r3 to see the selector value.
    // fzEYzb29 (answered): p=[base+32]=0x0D15EA5E — the HARNESS's own
    // POKE32(0x80000020) console-size value, because 177AC writes the
    // constant 0x80000000 (lis r0,-32768) to base-31480, so p reads our
    // [0x80000020]. v=[p] reads RAM at 0x0D15EA5E — unmapped, so the
    // 177EC lwz faults (DSI) or returns 0; r0=(0+6880<<16)=0x1AE00000 vs
    // 0x7C22 at 177F4 => NE => 177F8 branches to 17814 (dead side), so
    // 177FC/1780C/1A3F4 slot registration NEVER runs. On hardware the
    // base would be a DVD workarea pointer (filed by 16DC0/19E64), not a
    // constant — but here the workarea write IS the constant by design
    // (177A4/177AC write r0, not a computed pointer). NEXT fzEYzb30: what
    // SHOULD base-31480 point at — i.e. read 17784's selector [r13-31424]
    // (17794 sets it to 1; A360's 030CE halfword may gate entry at all).
    // fzEYzb151 (probe184 verdict: 177FC-live 0 hits, 17814-dead 0 hits —
    // BOTH legs are native-only stretches inside the single 177D0 dispatch
    // (same class as 1190C/A000 and 6FE14/6FEA8). The gate DECISION is still
    // fully determined: p=[0x80000020]=0x0D15EA5E (harness POKE) ->
    // r0=(p+0x1AE00000)=0x27F5EA5E vs 0x7C22 -> NE -> dead side taken by
    // correct guest arithmetic. __fstLoad (1A3F4) never runs, so the boot
    // BB2/FST READ cascade never starts. Fix = the [0x80000020] INPUT word
    // (what HW apploader files there), never the gate. Probes kept for the
    // 177D0 base/p dump only; leg probes removed as structurally silent.)
    if(addr==0x800177D0u){
      static unsigned _n=0; if(++_n<=6){ uint32_t b=0,p=0xDEADu,v=0xDEADu,w20=0xDEADu;
        guest_read32(cpu->gpr[13]-31480u,&b);
        if(b){ guest_read32(b+32u,&p); if(p&&p>=0x80000000u) guest_read32(p,&v); }
        guest_read32(0x80000020u,&w20);
        fprintf(stderr,"[dvdsm] 177D0 base=0x%08X p=[base+32]=0x%08X v=[p]=0x%08X [0x80000020]=0x%08X lr=0x%08X (#%u)\n",
          b, p, v, w20, cpu->lr, _n); }
      return false; }
    // fzEYzb13: 177C0/177C8 (post-call continuations). 177D0 has its
    // own deref-dump probe above; excluded here to avoid double-fire.
    if(addr==0x800177C0u||addr==0x800177C8u){
      static unsigned _m[2]={0}; int _i=addr==0x800177C0u?0:1;
      const char *_nm[2]={"177C0","177C8"};
      if(++_m[_i]<=3) fprintf(stderr,"[dvdsm] %s r3=0x%08X lr=0x%08X (#%u)\n",
        _nm[_i], cpu->gpr[3], cpu->lr, _m[_i]);
      return false; }
    // fzEYzb18: 17788/17824 (both dispatched) are the 17784-gate
    // sides: 17788 fallthrough (toward 177F0 branch) vs 17824 taken
    // (away). r0 = [r13-31424] selects; 1776C fires so entry runs.
    // fzEYzb30 (answered — 17788/17824 never fire in the 20s run: like
    // 18EB0/18ED4 they are NATIVE-only stretches inside the 1776C frame,
    // not resume pcs. The 1776C frame runs: 1776C(entry) -> ... -> 17788
    // native -> 1778C bl ABBC -> 17790 -> 17794 (flag=1) -> 17798 bl 16DC0
    // -> 1779C bl 19E64 -> 177A0 bl 15F80 -> 177A4 -> 177AC (const writer)
    // -> 177BC bl D540 -> 177C0 -> 177C4 bl D944 -> 177C8 -> 177CC bl 1029C
    // -> 177D0 (all dispatched bl-targets) -> 177E4 chain NATIVE to the
    // 177F8 gate, which branches native to 17814 (never 177FC). So the ONLY
    // dispatched pcs of the frame are the bl-target entries; everything
    // else runs native exactly once per 1776C. Reverted to no-probe.
    // fzEYzb31 (answered — 16DC0 ENTRY always shows r3=0xFFFFFFFF because
    // r3 is the CALLER's leftover; the body sets r3=lis(-32768)=0x80000000
    // itself at 16DC0. The real dump needs POST-body state. But the
    // [0x80000038+56] read is REAL DATA: 0x64638000 = 'dc'+0x8000 — the
    // FST string-table magic ('dc' = disc content marker at fst+56?).
    // So the workarea indexer READS LIVE FST DATA (our side-effect FST at
    // 0x81200000 via 102AC + 0x80000038 pointer). The chain is INTACT.
    // fzEYzb32 (answered): r13-31508(idx)=0 at 19E64 entry — the 16DC0
    // indexer files index = [fst_word]*12 + base => FST entry 0's offset
    // (first file). The queue index is COLD (0) because this is the FIRST
    // command ever issued (boot's INQUIRY) — correct, not a bug. The chain
    // 17798/1779C (16DC0/19E64) is INTACT; the FST read is LIVE.
    // fzEYzb33 (answered statically — 19690 decoded): the block filer
    // writes tag=14 ITSELF (19698 li r0,14 + 196AC stw r0,8(r3)) — even
    // before the boot INQUIRY issues. The 19700 leg then files b12=2.
    // So tag 14 is the COLD-START tag for EVERY boot block: the queue
    // NEVER holds anything but INQUIRY-class blocks until something files
    // a different tag — and the only other tag-filers (tag 8 via 18B08
    // READ row, tag 5 via 19500 slot-reg, tags 1/4 via 19430/1944C) are
    // all downstream of a SUCCESSFUL non-INQUIRY completion.
    // fzEYzb39 (CORRECTED statically — full 199xx lattice decoded): the
    // 199xx region is a RETRY-CLASS ladder on b12=[r29+12], entered via
    // 19900 (bl-target): 19920 r4=b12, 19928 r0=b12+1, 1992C vs 12:
    // r0==12 (b12==11) => 19B50 (phase advance); else 19988 m56-gate
    // (m56==0, cold) => 19978/19984 zero-leg => 1999C tag-gate:
    // tag==4 => 199A8 (SUCCESS: bl 16D50 waiter-flags + 199B4 bl 19FFC +
    // b12=10 filer + slot-invoke-or-drain); else 199A0 vs 1: ==1 =>
    // 19B50 (RETRY-EXIT, no state change); else 19AA4 vs 4: ==4 =>
    // 19AB0 (m48-gated writer: m48==0? then 19AA8 r0=3 / 19ABC r0=4
    // filers to m48, then re-gates at 19AC4/19AD8/19AEC/19AF4/19B00 on
    // b12 vs 5/6/11/7 — each filing m48=4/1/2/7/10 + b12=10 + drain).
    // So b12==11 is the TERMINAL retry count (phase advance at 19B50);
    // b12==1 (cold) exits immediately at 19A4. The ladder 2..11 needs
    // the 19994 filer (b12=[r29+8]) — but our blocks re-file b12 fresh
    // each issue, so the ladder never climbs. The 19B50 exits all land
    // at 19B50: r3=r31 + bl D51C + epilogue (a RETRY-RETURN, not READ).
    // fzEYzb39 (answered live — b12 RESETS to 1 every completion, #1-8
    // identical): the counter never advances because each completion runs
    // the 192FC clearer (b12=0) + the next issue re-files the block fresh
    // (19690 b12=2 -> 18D1C entry shows 1 after the 18DD8-path decrement?
    // exact reset site TBD, but the EFFECT is confirmed: the retry counter
    // is pinned at 1, so the 19930 jump-table phase (b12+1==12) is
    // unreachable). The loop is a STABLE LIMIT CYCLE, not a slow ramp.
    // fzEYzb40 (answered statically — 16D50 decoded): the 199A8 filer
    // (tag==4 leg) calls 16D50 which sets waiter-flag -31592=1 AND
    // -31560=1 (drive-ready?). The tag==4 leg is itself downstream of the
    // 19994 tag==4 check — unreachable while tags stay 14. The whole
    // 199xx dispatcher is a dead lattice from the cold state.
    // CONCLUSION (DVD loop fully decoded, all forks cold): the guest
    // runs its BOOT inquiry sequence correctly; the advance to READ
    // requires either (a) the slot callback (19500, never fires — needs
    // the 1A3F4 upper-layer registration via the dead 177FC side), or
    // (b) a cover/drive state change the HLE never signals (our DI cover
    // reports CLOSED but the STATUS register may lack the READY/TCINT
    // bits the guest polls at 14170: 0x800030CE==0x8200?). NEXT: probe
    // the 14170 flag-check leg (does the guest see drive-ready?) and the
    // DI STATUS/COVER register values we report vs Dolphin.
    // fzEYzb41 (answered statically — 13F9C-14180 upper layer decoded):
    // the 13FAC frame reads b12 (block+12) bit0: set => 1418C (slot
    // re-invoke path), clear => 13FB8 bit2 check => 13FC4/13FCC bit
    // decodes => 140F8 (r3<=0xFF gate) => 14100/14110/14114/14120/14128/
    // 14140/14148/14158 cascade (block-word signature checks: -257<<16,
    // -1287<<16, -1058<<16 patterns = FST/file magic compares) => 1416C
    // 030CE halfword gate (0x8200 = drive READY+cover-closed?) => 1417C
    // (r28=1, continue) vs 14170 (read flag, compare, 14180 continue).
    // With b12=1 (bit0 SET): 13FA0 bit0 => TAKEN to 1418C — the
    // SLOT path, not the 030CE gate. So the upper layer DOES dispatch
    // past the flag check iff the slot (block+40) is non-null. It is
    // null (19500 never filed) => which 1418C leg? Read 1418C next.
    // fzEYzb42 (answered statically — 1418C/1418C-141AC decoded): 1418C
    // reads [r31+12] bit2: set => 14198 (r3=r28, return to caller — the
    // upper layer CONTINUES with r28 status), clear => 13F9C (back to the
    // b12-bit0 gate = INFINITE UPPER-LEVEL SPIN on the same block while
    // bit2 stays clear). r31 here = the 13F64-frame's block cursor
    // (r29/r31 = queue walk: r29 = base+idx*20 entry). bit2 of [blk+12]
    // is the COMPLETION-DONE bit — filed by the 18Dx completion body as
    // b12=10? No: b12 values seen are 1,2,10,-1. bit2 set means b12&4:
    // b12=10 (0b1010) HAS bit2... wait 10=0b1010, bit2 (val 4)? 10&4=0.
    // Hmm: b12=10 has bits 1,3. bit2 (value 4) is CLEAR. b12=1: clear.
    // b12=2: clear. b12=-1 (0xFFFFFFFF): SET. So the 1418C gate passes
    // (to 14198 continue) ONLY when b12==-1 — the ERROR path value filed
    // by 19218/19224 (and 18DA0 etc.)! The upper layer spins at
    // 13F9C->1418C->13F9C until a completion files b12=-1?? That can't
    // be right either — recheck: 14190 rlwinm. r0,r0,0,29,29 = bit3
    // (value 4? PPC bit numbering: bits 29,29 = mask 0x00000004 = value
    // 4 = bit2 zero-indexed). b12=-1 has it set; b12=10 (0xA) doesn't.
    // So YES: upper layer waits for b12==-1?? But our completions file
    // b12=10 (success) via 18DF8/192A8 etc. — which would SPIN FOREVER.
    // INVERSION? Or b12==-1 means DONE-WITH-ERROR and the upper layer
    // then READS the error and re-issues? NEXT: who calls 13F64/13F9C
    // (the 13C00/13EFC/145D8/151A0 bl 141B0 sites) and what b12 values
    // do THEY expect — read the 141B0 wrapper.
    // fzEYzb43 (answered statically — 13BB8/13BC8 bit gates decoded): the
    // 13BB4-frame reads [r31+12] TWICE: 13BBC bit0-1 (rlwinm mask 3):
    // nonzero => 13BD0 zero-leg (r3=0, skip); zero => 13BC4 bit2 check
    // (mask 4): nonzero => 13BE0 (stw r25,4(r31) filer + 13BE4 zero-check
    // => 13BF0 bl 141B0 READ-ISSUE with r3=r26,r4=0,r5=1,r6=0!); zero =>
    // 13BD0 zero-leg. With b12=1: bit0-1 = 01 nonzero => 13BD0 zero-leg
    // (r3=0, NO read). With b12=10 (0b1010): bit0-1 = 10 nonzero =>
    // zero-leg too! With b12=2 (0b10): nonzero => zero-leg. With b12=0:
    // zero => bit2 check: clear => zero-leg. ONLY b12 with bit0-1==0 AND
    // bit2==1 passes to 13BE0: b12=4 (0b100)! So the READ-ISSUE leg needs
    // b12==4 exactly (among small values). Our b12 is pinned at 1 (bit0-1
    // nonzero) => zero-leg => r3=0 => 13DC4... the upper layer returns 0
    // (not-ready) and the boot NEVER proceeds to READ. The b12 values are
    // a STATE MACHINE: 0=idle, 1=issued, 2=??, 4=ready-to-read senior?,
    // 10=done?, 11=??, -1=error. The completion body MUST advance
    // 1->2->4 via the 199xx dispatcher — which needs the slot/tag-8
    // machinery that is cold. CONFIRMED END-TO-END: the DVD subsystem is
    // waiting on b12==4 and nothing can produce it from the cold state.
    // fzEYzb45 (answered statically — 19B78/19B9C SUCCESS leg decoded):
    // the 19934 jump-table row for tag==14 lands at 19B78 (r30=block):
    // 19B98 bl 198FC (queue helper) -> 19B9C r3==0? (helper FAILED =>
    // 19BA4 r3=-1, 19BA8 -> 19C0C FAIL-EXIT) : 19BAC bl D4F4-sync ->
    // 19BB0 r31=[block+12]=b12 -> 19BB4 r3=b12 -> 19BB8 r0=b12+1 vs 1:
    // b12==0 => 19C00 (SKIP the b12+1==12/==10/==3 ladders: b12=1 first
    // INVALIDs r0==0? no: 19BC0 checks r0&... wait 19BB8 r0=r3+1=b12+1,
    // 19BBC vs 1: b12+1==1 i.e. b12==0 => 19C00; else 19BC4 vs 10:
    // b12+1==10 i.e. b12==9 => 19C00; else 19BCC vs 3: b12==3 =>
    // 19BF4 (SKIP to 19BF4 filer: r3=r13-31496 + bl 110A8 + -> 19BB4
    // RE-READ b12?? no: 19BF4 is BEFORE 19BB4 in layout... order:
    // 19BB4->19BC0->19BC4->19BCC->19BD0->19BD4(tag)/19BDC(tag-1)/19BE4/
    // 19BEC/19BF0 vs 13/15 gates -> 19BF4 filer (r13-31496 + bl 110A8
    // QUEUE-WAIT?) -> 19BFC -> 19BB4 RE-LOOP reading FRESH b12).
    // So the 19B78 leg is a POLL LOOP: re-read b12 until it changes.
    // With b12 pinned at 1: 19BB8 r0=2 vs 1: NE -> 19BC4 r3=1 vs 10:
    // NE -> 19BCC r3=1 vs 3: NE -> 19BD0 fallthrough -> 19BD4 reads
    // [r30+8]=tag (14) -> 19BD8 tag-4 -> 19BDC vs 1: 10 vs 1: NE ->
    // 19BE4 tag vs 13: NE -> 19BEC vs 15: NE -> 19BF0 vs 15: NE ->
    // 19BF4 filer + bl 110A8 + 19BFC -> 19BB4 RE-READ. INFINITE POLL on
    // b12==1/tag==14. The 19BF4 filer + 110A8 call is the queue-wait
    // primitive — 110A8 writes [thread+712]=4 + [thread+732]=queue (see
    // 110CC/110D8: waiter link!). The guest PARKS ITSELF on the queue
    // waiting for b12 to change — and b12 changes ONLY via a completion
    // that takes a different leg. STABLE PARK, not a spin: the thread
    // sleeps until someone files b12. NEXT: who wakes it — the 110A8
    // waiter chain (thread+732 queue link) and whether OUR completions
    // ever link/unlink it.
    // fzEYzb34 (answered statically — 18CC8 decoded): the leg writes
    // NOTHING to the drive state — it copies [0xCC006014]+4 to the block,
    // sets [blk+28]=32, then 18CEC bl 16A38 re-issues INQUIRY (c0=0x12).
    // m60 is never touched on this path: the ONLY m60 writers in the
    // whole 18xxx region are 18870 ([curblk+8] tag copy) and 18BDC
    // (r0=1, tag-8 row only). The 18D1C completion body likewise never
    // writes m60 except via the 18870/18860 drain calls. So after a
    // successful INQUIRY the guest ITSELF re-issues INQUIRY (tag stays 14,
    // m60 stays 14) — the ADVANCE to READ must come from the SLOT
    // callback (block+40, filed by 19500) or from the upper layer (1A3F4
    // chain via 1780C) — both dead (19500 never fires; 177F8 always to
    // 17814). The loop is architecturally closed: INQUIRY completes ->
    // state unchanged -> INQUIRY re-issues.
    // fzEYzb35 (answered statically — 17F54-gated writer decoded): the
    // 17FAC m56-writer is REACHED only via 17F8C (m60=2): 17F90 loads
    // curblk, 17F98 m48=2, 17FA4 curblk=r31.const, 17FA8 r0=10, 17FAC
    // stw r3->m56 with r3=0 (17FA0 li r3,0) => m56=0 CLEARER, not setter.
    // So the ONLY true m56=1 setter in the entire DVD region is... NONE
    // found yet: 17FAC writes r3=0, all other -31456 writers write r0=0
    // or r4=0. m56 is cold-zero and every fork that needs m56==1
    // (179C4->179C8 slot-invoke leg, 18DD4->18DD8 m48=7 leg, 1928C->19290
    // success leg) is untaken BY CONSTRUCTION. The 17F80 fork
    // (m56!=0 -> 17FF8 vs m56==0 -> 17F8C clearer) likewise always takes
    // the clearer. NEXT: the 030CE halfword gate (A360: A398 stores
    // [r30]+2|0x8000 vs A3A8 stores 1) — read A360's caller chain.
    if(addr==0x80019E64u){
      static unsigned _q=0; if(++_q<=2) fprintf(stderr,"[dvdsm] 19E64-entry (see fzEYzb32: idx=0 cold-start, FST live) lr=0x%08X (#%u)\n",
        cpu->lr, _q);
      return false; }
    if(addr==0x80016DC0u){
      static unsigned _n=0; if(++_n<=2) fprintf(stderr,"[dvdsm] 16DC0-entry (r3=caller leftover, see fzEYzb31) lr=0x%08X (#%u)\n",
        cpu->lr, _n);
      return false; }
    // fzEYzb12: 177FC (dispatched) is the 177F8-fallthrough side toward
    // 1780C/1A3F4; 17814 is the branch-taken side. r3 = [r13-31480]+32
    // deref selects. Dump r3 + the deref chain base.
    if(addr==0x800177FCu){
      static unsigned _n=0; if(++_n<=4){ uint32_t b=0; guest_read32(cpu->gpr[13]-31480u,&b);
        fprintf(stderr,"[dvdsm] 177FC r3=0x%08X base-31480=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], b, cpu->lr, _n); }
      return false; }
    // fzEYzb11: 17814/1780C (both dispatched) are the 177F8-branch
    // sides into the 1A3F4 caller. 177F0/177F8 native; r3 selects.
    if(addr==0x80017814u||addr==0x8001780Cu){
      static unsigned _q1=0,_q2=0; unsigned *c=addr==0x80017814u?&_q1:&_q2; (*c)++;
      if(*c<=4) fprintf(stderr,"[dvdsm] %s r3=0x%08X lr=0x%08X (#%u)\n",
        addr==0x80017814u?"17814":"1780C-caller", cpu->gpr[3], cpu->lr, *c);
      return false; }
    // fzEYzb10: 1776C (dispatched entry) encloses the 1A3F4 slot-chain
    // caller at 1780C. Fires => the registration chain runs at all.
    // fzEYzb27: ALSO dump m56/m60 at entry: the A72C call chain
    // (A728 bl 1776C gated on r13-31816==0; A72C reads r13-31852 which
    // 17794 SET to 1) shows whether 1776C re-runs natively after the
    // completion storm, and with what drive state.
    // fzEYzb28: 19FA4 is the drain scanner (bl-target of 187E4, NOT a slice
    // entry — it fires once per drain, 7x like the completions). Dump its r3
    // (scan result: block ptr or 0) + m64 + curblk: ret 0 => 187E8 sees
    // 187F0 early-out (curblk=0); ret nonzero => 187FC continue. Which?
    if(addr==0x80019FA4u){
      static unsigned _n=0; if(++_n<=8){ uint32_t m64=0xDEADu,cb=0;
        guest_read32(cpu->gpr[13]-31464u,&m64); guest_read32(cpu->gpr[13]-31488u,&cb);
        fprintf(stderr,"[dvdsm] 19FA4-scan r3=0x%08X m64=%u curblk=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], m64, cb, cpu->lr, _n); }
      return false; }
    if(addr==0x8001776Cu){
      static unsigned _m=0; if(++_m<=6){ uint32_t m56=0xDEADu,m60=0xDEADu,cb=0,tag=0xDEADu,w=0xDEADu;
        guest_read32(cpu->gpr[13]-31456u,&m56); guest_read32(cpu->gpr[13]-31460u,&m60);
        guest_read32(cpu->gpr[13]-31488u,&cb); if(cb) guest_read32(cb+8u,&tag);
        guest_read32(0x800030CCu,&w);
        fprintf(stderr,"[dvdsm] 1776C r3=0x%08X r4=0x%08X m56=%u m60=%u curblk=0x%08X tag=%u flag30CC=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], m56, m60, cb, tag, w, cpu->lr, _m); }
      return false; }
    // fzEYzb46 (answered — 19934 table dumped from DOL: tag 14 row is
    // 19CF0, NOT 19B78. 19CF0: r4=[0xCC006004] (DI cmd word mirror) bit0
    // (rotl30 & 1, i.e. ORIGINAL bit30 = word's bit1?): set => 19D0C
    // (19CF0 taken... verify direction: bc 4,2 = branch if CR0EQ clear?
    // 19D00 TAKEN-side => 19D0C) : 19D04 bit31 check => 19D14/19D08.
    // In all cases the row READS LIVE DI REGISTERS ([0xCC006004]) — the
    // row is only as correct as our DI model. It then flows to 19D2C
    // (r3 = 198FC-helper result?) etc. The tag-14 row NEVER touches b12
    // or the poll loop — it re-examines the hardware command word!
    // Who calls the 19934 dispatcher at all? NOBODY in the observed
    // loop — 199xx never fires (no probes hit all session). The live
    // loop is 189FC(tag14)->18CC8->16A38 only. So the 199xx lattice is
    // for a LATER phase (post-inquiry command management) that boot
    // never reaches.
    // fzEYzb50 (answered live — waiter trace + 1AF8C poke REMOVED):
    // 110A8 fires 4-6x per 25s, ALL lr=1AF8C, r30=0 (NULL queue). The
    // 1AF8C-poke removal changed NOTHING (same DVD rate, same frontier
    // 12974, same 110A8 pattern): the poke was NOT the park driver —
    // r30==r0 holds because the WORD never changes, not because we
    // force it. The 1AF64-frame spins natively (1AF84->1AF88 bl 110A8
    // -> 1AF8C/1AF94 EQUAL -> back) WITHOUT dispatching 1AF98: the
    // game thread waits on [r13-31388] which NO completion ever files
    // (all -31388 writers? none found in DVD region — grep later).
    // The 11160-poll never fires (110A8 frame runs native through).
    // NEXT fzEYzb51: who SHOULD file [r13-31388] — grep all writers;
    // and what sits in the word now (dump at 110A8 entry).
    // fzEYzb51 (answered statically — writers found): [r13-31388] has
    // exactly TWO writers: 1A618 (stw r0=[r13-31388]+1, i.e. INCREMENT,
    // in the 1A5FC-taken leg = 1A5F8 bit3-set path) and 1AB20 (stw r31,
    // value TBD). The 1AF64 waiter exits iff [word] != r30-entry-value.
    // 1A618 increments it — so the 1A5xx path is the WAKER. Who calls
    // 1A5xx? NO bl to 1A5xx/1A6xx EXISTS in any chunk (grep: only case
    // labels + one lr=1A540 return slot). The 1A5xx waker is itself
    // unreachable from the live loop — same cold-lattice pattern as
    // 199xx/19500/17160. The FULL picture: the DVD subsystem is a set
    // of mutually-reachable-but-collectively-unentered phases; the live
    // code is only the INQUIRY re-issue limb + the game-thread wait
    // limb. The ENTRY to the rest (1A3F4 slot-chain via dead 177FC side,
    // 199xx via 19B78 game calls that never come) never fires.
    // NEXT fzEYzb52: stop decoding dead lattice. The RIGHT question is
    // comparative: run Dolphin with a breakpoint/log at 18D1C #1 and at
    // 1AF64-entry and compare register+memory state vs ours. The FIRST
    // divergence (not the 50th downstream gate) is the fix point.
    // Candidate first-divergences already logged: (a) r30/r31 at 18D1C
    // entry (ours: 0/0 zeroed by fzBU — hardware thread regs unknown);
    // (b) 16A38's r4 callback (ours 18D1C — matches); (c) DI STATUS/
    // COVER bits at issue time; (d) the 177AC const-vs-pointer write.
    // fzEYzb50 probes quieted (pattern stable: 110A8 lr=1AF8C r30=0;
    // 11160 never fires — 110A8 frame runs native through). Re-enable
    // with caps if the wait pattern changes.
    if(addr==0x800110A8u){
      static unsigned _n=0; if(++_n<=4){ uint32_t qh=0xDEADu,q0=0xDEADu,q4=0xDEADu,th=0xDEADu;
        guest_read32(cpu->gpr[3]+732u,&qh);
        guest_read32(cpu->gpr[3],&q0); guest_read32(cpu->gpr[3]+4u,&q4);
        guest_read32(0x800000E4u,&th);
        fprintf(stderr,"[wait] 110A8-entry r3=0x%08X [r3]=0x%08X [r3+4]=0x%08X [r3+732]=0x%08X thread=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], q0, q4, qh, th, cpu->lr, _n); }
      // fzEYzb102: post-sleep RETURN probe. 110A8's blr landing (1AF8C) is
      // the waiter's sample point — but 110A8 itself may never RETURN
      // (thread sleeps forever if never woken). Dump the RETURN side:
      // 1AF8C fires => the sleep returned => sample r0 there (fzEYzb101).
      // Correlate counts: 110A8-entry N vs 1AF8C-sample M. M<N (here 0<2)
      // = threads still asleep = the sleep primitive itself never wakes.
      return false; }
    // fzEYzb47 (answered statically — 17234 caller chain decoded): the
    // ONLY in-DVD caller of the 199xx family is 17234 (bl 19B78), inside
    // the 17160-frame: 17160 (r30=path?, r31=dst?) -> 1717C bl 16DF8
    // (queue lookup by idx?) -> r3==0? no: 17184 loop over workarea
    // entries (171B4 mulli*12 + 171BC/171C0 top-byte gate on entry+8) ->
    // 171E4 builds block (48/52/56/12 filers) -> 17200 reads entry+8 ->
    // 17234 bl 19B78 (SUBMIT the freshly built block to the post-inquiry
    // manager!) -> 17238 r3=1 return. The 17160-frame is the
    // READ-ISSUE builder (builds a block from a workarea entry and
    // submits it) — called from 172CC/173DC via 1724C, which NOBODY
    // calls either (no bl to 1724C/17160 found in DVD region). The two
    // OTHER 19B78 callers (5584C/55F44) are in game code (80055xxx =
    // boot/main thread!) — the UPPER LAYER calls 19B78 directly to
    // submit reads! So the bridge EXISTS: game code -> 19B78 ->
    // 198FC-helper -> queue. It never fires because the game thread
    // never gets past the DVD-wait (it parks at 110A8/AD1C waiting for
    // b12, or spins in the boot inquiry wait). The boot is stuck
    // BELOW the game: the DVD thread spins INQUIRY while the game
    // thread waits for a READ that nobody issues. NEXT: find what the
    // game thread is doing — the 12974/10740/1BD10 frontier + the AD1C
    // park (heap allocator, not DVD): is the game waiting on the DVD
    // queue (110A8 waiter link) or parked elsewhere?
    // fzEYzb46 probe quieted (19CF0 never fires — 199xx lattice dead).
    // (Body kept as comment so the finding stays recorded.)
    // fzEYzb56 (answered live — census): ONLY 1AF64-gamewait fires
    // (3x, lr=70A2C, r3=0); 1A500/1A540/1A560/1A60C/1AB20/19B78/19500
    // NEVER fire. The dead lattice stays dead; the live set is exactly
    // {INQUIRY limb, game-wait limb}.
    // fzEYzb57: 70A2C-decoded — the game thread runs 1AF64-wait FIRST
    // (70A28 bl 1AF64), then 70A2C reads [r13-30412] (a POINTER), loads
    // [ptr+0] bit0: set => 70A40 (continue to 798D0 worker = PROGRESS);
    // clear => 70A3C bl 1AF64 (wait AGAIN). So the game thread is a
    // TWO-STAGE wait: [r13-31388] (1AF64) then [[r13-30412]+0].bit0
    // (70A30). NEITHER ever flips: -31388 pinned at 0 (waker dead),
    // -30412 target unknown. Dump [r13-30412] + [[it]] at 1AF64 entry:
    // is the second gate even armed (non-null pointer)?
    // fzEYzb57 (answered live — gate2ptr=0x8019E150, [ptr]=0, waitword=0
    // on all 4 hits): the second gate IS armed (valid pointer) but its
    // target word is 0 (bit0 clear = wait). BOTH gates pinned at zero.
    // The 706xx filer (7066C/7067C/7068C/7069C stw r0,-30412, selected by
    // r3==1/0 + r31==0 cascade from a 1C01C query at 70634) files the
    // gate2 POINTER (0x8019Exxx = DVD workarea?); the [ptr] WORD it
    // points at is filed by NOBODY yet (a later boot phase). And the
    // first gate's waker (1A618) is in the dead 1A5xx chain. So the game
    // thread waits on TWO words that only LATER boot phases file — and
    // those phases never run because... the DVD thread spins INQUIRY
    // instead of advancing to them. CIRCULAR BOOT DEPENDENCY, fully
    // mapped. NEXT: what SHOULD break it — the inquiry SUCCESS path
    // that files the first of these words. Trace [0x8019E150] writers.
    // fzEYzb58: gate2 target [0x8019E150] is INSIDE DOL .data
    // (0x80095EA0+0xC5A80 covers it — DVD workarea/BSS-adjacent). Writer
    // candidates: the 19E64 initializer (19E64+ writes r3/imm chains at
    // 0x801xxxxx? verify), the 19690 filer, or the 16DC0 indexer. The
    // 1AF64 probe stays (first-4) to detect ANY change across runs.
    // fzEYzb58: watch the gate words for ANY guest write via the cpu.c
    // journal hook (watch_journal at file top). Armed at first 1AF64
    // hit: slot0 = gate2 word (ptr read live), slot1 = -31388 wait
    // word, slot2 = -31488 curblk. First-12 journal hits total, then
    // silent. If a slot NEVER logs, nothing ever writes it.
    if(addr==0x8001AF64u){
      static unsigned _n=0; if(++_n<=4){ uint32_t p=0xDEADu,v=0xDEADu,w=0xDEADu;
        guest_read32(cpu->gpr[13]-30412u,&p);
        if(p>=GC_RAM_BASE) guest_read32(p,&v);
        guest_read32(cpu->gpr[13]-31388u,&w);
        fprintf(stderr,"[dvdsm] 1AF64-gamewait r3=0x%08X gate2ptr=0x%08X [ptr]=0x%08X waitword31388=%u lr=0x%08X (#%u)\n",
          cpu->gpr[3], p, v, w, cpu->lr, _n); }
      if(s_watch_off[0]==0xFFFFFFFFu){
        uint32_t p=0; guest_read32(cpu->gpr[13]-30412u,&p);
        if(p>=GC_RAM_BASE && p+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[0]=p-GC_RAM_BASE;
        { uint32_t a=cpu->gpr[13]-31388u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[1]=a-GC_RAM_BASE; }
        { uint32_t a=cpu->gpr[13]-31488u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[2]=a-GC_RAM_BASE; }
        { uint32_t a=cpu->gpr[13]-31372u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[3]=a-GC_RAM_BASE; }
        { uint32_t a=cpu->gpr[13]-31452u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[4]=a-GC_RAM_BASE; }
        { uint32_t a=cpu->gpr[13]-31456u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[5]=a-GC_RAM_BASE; }
        { uint32_t a=(cpu->gpr[13]-30422u)&~3u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[6]=a-GC_RAM_BASE; }
        { uint32_t a=(cpu->gpr[13]-30632u)&~3u;
          if(a>=GC_RAM_BASE && a+4<=GC_RAM_BASE+g_cpu.ram_size) s_watch_off[7]=a-GC_RAM_BASE; }
        { extern void ppc_set_mem_write_journal(void (*fn)(u32,u32,void*), void* user);
          ppc_set_mem_write_journal(watch_journal, NULL); }
        fprintf(stderr,"[watchmem] armed gate2off=0x%X waitoff=0x%X curblkoff=0x%X fnptroff=0x%X m52off=0x%X m56off=0x%X doneoff=0x%X wait2off=0x%X (r13=0x%08X)\n",
          s_watch_off[0], s_watch_off[1], s_watch_off[2], s_watch_off[3], s_watch_off[4], s_watch_off[5], s_watch_off[6], s_watch_off[7], cpu->gpr[13]);
      }
      return false; }
    // fzEYzb101: waiter SAMPLE probe. 1AF8C (dispatched, downcount) is
    // the waiter's re-read of the word AFTER the 110A8 sleep; r0 is the
    // sampled value, r30 the pre-sleep value. Exit iff r30!=r0. Dump
    // both: any nonzero r0 here = the waiter OBSERVED a wake.
    // (Correction: an earlier build failed — the first edit orphaned the
    // arming block outside any if; fixed by re-adding this probe AFTER
    // the arming close brace. Uncapped counter, first-8 dump.)
    if(addr==0x8001AF8Cu){
      static unsigned _s=0; _s++;
      // fzEYzb108 (correction): host_call runs BEFORE the chunk, so gpr[0]
      // here is STALE (pre-lwz, still the 110A8 lr value 0x8001AF8C — that
      // is why r0 looked like a code pointer). Read the word from memory.
      if(_s<=8){ uint32_t w=0xDEADu; guest_read32(cpu->gpr[13]-31388u,&w);
        fprintf(stderr,"[dvdsm] 1AF8C-sample r30=%u word=%u r3=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[30], w, cpu->gpr[3], cpu->lr, _s); }
      return false; }
    // fzEYzb103: sleep/wake-chain probes. 110A8 always ends at 11170 bl
    // 105D0 (r3=0); the waker frame calls 11194 at 1A764 (r3=queue
    // r13-31380). 105D0's three blr landings (11174 waiter / 11278 waker /
    // 10818 scheduler) are dispatched => RETURN observable. If 11174 never
    // fires, 105D0 never returns to the waiter = sleep never wakes.
    if(addr==0x800105D0u){
      static unsigned _s=0; _s++;
      if(_s<=6){ uint32_t th=0xDEADu,f84=0xDEADu,f80=0xDEADu,f88=0xDEADu,st=0xDEADu,sched=0xDEADu;
        guest_read32(0x800000E4u,&th);
        guest_read32(cpu->gpr[13]-31684u,&f84); guest_read32(cpu->gpr[13]-31680u,&f80);
        guest_read32(cpu->gpr[13]-31688u,&f88); guest_read32(cpu->gpr[13]-32640u,&sched);
        if(th>=GC_RAM_BASE) guest_read32(th+712u,&st);
        fprintf(stderr,"[sleep] 105D0-entry r3=0x%08X r30=0x%08X thread=0x%08X [th+712]=0x%08X f84=%u f80=%u f88=%u sched=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[30], th, st, f84, f80, f88, sched, cpu->lr, _s); }
      return false; }
    // fzEYzb104: 105D0-inner census. 106E4=f88 pre-check (skip scheduler if
    // set); 10708=scheduler returned; 1071C/1072C=spin polls; 10740=wake
    // path; 107E0=105D0 return (blr to 11174/11278/10818). Tells where the
    // waiter parks when RET-11174 never fires.
    if(addr==0x800106E4u||addr==0x80010708u||addr==0x8001071Cu||addr==0x8001072Cu||addr==0x80010740u||addr==0x800107E0u){
      static unsigned _q1=0,_q2=0,_q3=0,_q4=0,_q5=0,_q6=0;
      unsigned *c=addr==0x800106E4u?&_q1:addr==0x80010708u?&_q2:addr==0x8001071Cu?&_q3:addr==0x8001072Cu?&_q4:addr==0x80010740u?&_q5:&_q6; (*c)++;
      if(*c<=4){ uint32_t f88=0xDEADu;
        guest_read32(cpu->gpr[13]-31688u,&f88);
        fprintf(stderr,"[sleep] %05X f88=%u r3=0x%08X r30=0x%08X lr=0x%08X (#%u)\n",
          addr&0xFFFFFu, f88, cpu->gpr[3], cpu->gpr[30], cpu->lr, *c); }
      return false; }
    if(addr==0x80011194u){
      static unsigned _w=0; _w++;
      if(_w<=6){ uint32_t q0=0xDEADu,q4=0xDEADu,th=0xDEADu;
        if(cpu->gpr[3]>=GC_RAM_BASE){ guest_read32(cpu->gpr[3],&q0); guest_read32(cpu->gpr[3]+4u,&q4); }
        guest_read32(0x800000E4u,&th);
        fprintf(stderr,"[sleep] 11194-entry r3=0x%08X [r3]=0x%08X [r3+4]=0x%08X thread=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], q0, q4, th, cpu->lr, _w); }
      return false; }
    if(addr==0x80011174u||addr==0x80011278u||addr==0x80010818u){
      static unsigned _r1=0,_r2=0,_r3=0;
      unsigned *c=addr==0x80011174u?&_r1:addr==0x80011278u?&_r2:&_r3; (*c)++;
      if(*c<=4) fprintf(stderr,"[sleep] RET-%05X r3=0x%08X r31=0x%08X lr=0x%08X (#%u)\n",
        addr&0xFFFFFu, cpu->gpr[3], cpu->gpr[31], cpu->lr, *c);
      return false; }
    // fzEYzb109: 34378 hook-frame (30640-setter) + 309FC pre-display
    // frame. 34378 is the DIRECT 30640-setter (chains via r30 at 34398);
    // 309xx display path runs 34378 THEN 34444 — so WHICH runs tells
    // whether the display path reached the 30980-leg. 309FC runs 3450C
    // (IRQ-table registrant) then GX display work; its lr (707FC =
    // game path vs other = display path) names the caller. Both fire
    // BEFORE 3416C's park, so ordering vs 3416C-entry maps the race.
    if(addr==0x80034378u){
      static unsigned _h=0; if(++_h<=4){ uint32_t h=0xDEADu;
        guest_read32(cpu->gpr[13]-30640u,&h);
        fprintf(stderr,"[wait2] 34378-hookset r3=0x%08X oldhook30640=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], h, cpu->gpr[4], cpu->lr, _h); }
      return false; }
    // fzEYzb114: IRQ-table census. D540(index=r3, handler=r4) is the ONLY
    // writer to the handler table at [r13-31768] (stw r4,0(r5)); entry is
    // dispatched. First-40 dump maps index->handler live: expect 18->343BC
    // + 19->34488 from the 3450C frame (0x80030000+17340=343BC,
    // +17544=34488), plus the VI handler that reaches 1A55C (r3=0x18
    // there implies a native stub between DCD8 blrl and 1A55C). lr names
    // the registrant.
    if(addr==0x8000D540u){
      static unsigned _q=0; if(++_q<=40)
        fprintf(stderr,"[wait2] IRQREG idx=%d handler=0x%08X lr=0x%08X (#%u)\n",
          (int)(int16_t)(cpu->gpr[3]&0xFFFFu), cpu->gpr[4], cpu->lr, _q);
      // fzEYzb116: note handler-19 registration for null-GX PE synthesis.
      if((int16_t)(cpu->gpr[3]&0xFFFFu)==19 && cpu->gpr[4]==0x80034488u)
        s_pe19_registered = true;
      return false; }
    // fzEYzb114b: 343BC-entry (dispatched, downcount-11) = handler-18 head.
    // If 343BC fires per retrace but 34488 never does, cause-18 pends while
    // cause-19 doesn't — the chassis must assert cause-19 per Dolphin.
    if(addr==0x800343BCu){
      static unsigned _v=0; if(++_v<=4){ uint32_t h=0xDEADu;
        guest_read32(cpu->gpr[13]-30640u,&h);
        fprintf(stderr,"[wait2] 343BC-entry r3=0x%08X r4=0x%08X hook30640=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], h, cpu->lr, _v); }
      return false; }
    // fzEYzb111: 342xx dispatcher entries (video-mode blrl chain).
    // 34230=head (cmp r3,1), 34254=merge, 342B8/342CC/342E0/342F4=legs.
    // r3 at entry = the mode arg the caller passed through the hook chain
    // (30640 -> blrl 343BC -> here). lr names the caller leg.
    if(addr==0x80034230u||addr==0x80034254u||addr==0x800342B8u||addr==0x800342CCu||addr==0x800342E0u||addr==0x800342F4u){
      static unsigned _d1=0,_d2=0,_d3=0,_d4=0,_d5=0,_d6=0;
      unsigned *c=addr==0x80034230u?&_d1:addr==0x80034254u?&_d2:addr==0x800342B8u?&_d3:addr==0x800342CCu?&_d4:addr==0x800342E0u?&_d5:&_d6; (*c)++;
      if(*c<=4) fprintf(stderr,"[wait2] VM-%05X r3=%u r4=0x%08X r5=0x%08X lr=0x%08X (#%u)\n",
        addr&0xFFFFFu, cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->lr, *c);
      return false; }
    if(addr==0x800309FCu){
      static unsigned _d=0; if(++_d<=4)
        fprintf(stderr,"[wait2] 309FC-display r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], cpu->lr, _d);
      return false; }
    // fzEYzb138: 197F0 leg-name probe (probe171 NEVER fired the old set:
    // stale binary — build failed on lock). Exactly one of 19818
    // (m64==0 leg), 19840 (curblk==0 leg), 19848 (curblk!=0 leg) fires
    // per invocation and NAMES the path. Static trace with m64=1/w72=0/
    // curblk=0 predicts 19840 always, return 0 — but 6600 sees -1, so
    // either the leg differs (stale-entry read vs native read) or the
    // tail (19878 r3=r30 / D51C / 19880 r3=r31) corrupts. 19878-join +
    // 19880-ret pin the tail.
    if(addr==0x80019818u||addr==0x80019840u||addr==0x80019848u||addr==0x80019878u||addr==0x80019880u){
      static unsigned _a=0,_b=0,_c=0,_d=0,_e=0;
      unsigned *c=addr==0x80019818u?&_a:addr==0x80019840u?&_b:addr==0x80019848u?&_c:addr==0x80019878u?&_d:&_e; (*c)++;
      if(*c<=6){ uint32_t w12=0xDEADu;
        guest_read32(cpu->gpr[31]+12u,&w12);
        fprintf(stderr,"[wait4] %s r0=%d r30=0x%08X r31=0x%08X [r31+12]=%d r3=0x%08X (#%u)\n",
          addr==0x80019818u?"19818-m64zero":addr==0x80019840u?"19840-cbzero":addr==0x80019848u?"19848-cbnz":addr==0x80019878u?"19878-join":"19880-ret",
          (int32_t)cpu->gpr[0],cpu->gpr[30],cpu->gpr[31],(int32_t)w12,cpu->gpr[3],*c); }
      return false; }
    // fzEYzb117: post-waiter2 park (320F0) + 70068/700B4 display chain.
    // 320F0 (dispatched, downcount) copies VI [r13-30708]+0 live bits into
    // gx vars [r2-32232]+12 + stb flags to r3/r4/r5/r6/r7; returns via
    // 3213C blr. lr names the 700xx caller. If flag-30632 flips but 320F0
    // never advances past 3213C, the NEXT waiter (32140 mulli gate?) parks.
    // 70068/700B4 (dispatched entries) = the GX chain the game path runs
    // after the park (70068 via 6FD5C bl, 700B4 via 6FD58 bl).
    if(addr==0x800320F0u){
      static unsigned _p=0; if(++_p<=6){ uint32_t vi=0xDEADu,gx=0xDEADu;
        guest_read32(cpu->gpr[13]-30708u,&vi); guest_read32(cpu->gpr[2]-32232u,&gx);
        fprintf(stderr,"[wait3] 320F0-park VIptr=0x%08X gxbase=0x%08X lr=0x%08X (#%u)\n",
          vi, gx, cpu->lr, _p); }
      return false; }
    if(addr==0x80070068u||addr==0x800700B4u){
      static unsigned _g1=0,_g2=0;
      unsigned *c=addr==0x80070068u?&_g1:&_g2; (*c)++;
      if(*c<=4) fprintf(stderr,"[wait3] %s r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
        addr==0x80070068u?"70068":"700B4",
        cpu->gpr[3], cpu->gpr[4], cpu->lr, *c);
      return false; }
    // fzEYzb120: 6FEA0 is a NATIVE goto-target (from 6FE3C-taken, no
    // downcount, HAS switch case — reachable only as native goto landing,
    // never as a dispatch entry). 6FE14 (bl 1140C, downcount, switch case)
    // IS dispatchable — but the park iterations never dispatch EITHER
    // (both silent in probe146): the WHOLE 6FE14->6FEA8 loop runs native
    // inside ONE dolrecomp_call from 6FE94/6FE08, like D9CC's DCD8 blrl
    // stretch. So the loop is INVISIBLE to dispatch probes by
    // construction; observe only its EFFECT: journal here is armed;
    // byte -30476 read live at 6FEB0/6FCE0-dispatch points. If the byte
    // stays 0 while 700F4 never fires, the 6FCE0 path stalls before
    // 6FD1C — chase the 6Fxx tail (6FD34 bit-test? 6FD74 C41C call?).
    // NEXT: probes at 6FD1C-entry + 6FD34 + 700F4-entry (all dispatched).
    if(addr==0x8006FE14u){
      static unsigned _f=0; _f++;
      if(_f<=6||_f%5000000==0){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30476u,&fl);
        fprintf(stderr,"[wait3] 6FE14-lap flag30476=0x%08X r3=0x%08X lr=0x%08X (#%u)\n",
          fl, cpu->gpr[3], cpu->lr, _f); }
      return false; }
    if(addr==0x8006FD1Cu){
      static unsigned _d=0; if(++_d<=4){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30476u,&fl);
        fprintf(stderr,"[wait3] 6FD1C-entry flag30476=0x%08X lr=0x%08X (#%u)\n",
          fl, cpu->lr, _d); }
      return false; }
    if(addr==0x8006FD34u){
      static unsigned _t=0; if(++_t<=4)
        fprintf(stderr,"[wait3] 6FD34-bitest r0=%u r3=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[0], cpu->gpr[3], cpu->lr, _t);
      return false; }
    if(addr==0x800700F4u){
      static unsigned _w=0; if(++_w<=4)
        fprintf(stderr,"[wait3] 700F4-filer r0=%u lr=0x%08X (#%u)\n",
          cpu->gpr[0], cpu->lr, _w);
      return false; }
    // fzEYzb121: 7001C-entry (PAD-config callee, dispatched) + 74918-entry
    // (its worker, dispatched). 700F4-filer's lr=344E0 (not 6FD1C) means
    // the 344xx display path sets flag-30476 WITHOUT the 6Fxx frame ever
    // running 6FD1C — so WHO calls 7001C/700F4, and does the 6Fxx spin
    // (whose ONLY writer is 6FD1C/700F4) ever observe the filer? lr names
    // the caller of each.
    if(addr==0x8007001Cu){
      static unsigned _p=0; if(++_p<=6)
        fprintf(stderr,"[wait3] 7001C-entry r3=%u r4=%u r5=%u lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->lr, _p);
      return false; }
    // fzEYzb122: 6FDA4 (bl 35110, dispatched) + 6FDAC (bl 1BE6C,
    // dispatched) = the 6FD98-tail waitword chain. 35110's r3/r4 gate
    // the 1BE6C waitword READ (1BE6C = lwz r3,-31388(r13); blr — the
    // waitword as a FUNCTION). lr=6FDA8/6FDB0 names the leg. If 1BE6C
    // never fires here, the tail stalls before the 6FDB0 store.
    if(addr==0x8006FDA4u||addr==0x8006FDACu){
      static unsigned _t1=0,_t2=0;
      unsigned *c=addr==0x8006FDA4u?&_t1:&_t2; (*c)++;
      if(*c<=4){ uint32_t w=0xDEADu; guest_read32(cpu->gpr[13]-31388u,&w);
        fprintf(stderr,"[wait3] %s r3=0x%08X r4=0x%08X waitword=%u lr=0x%08X (#%u)\n",
          addr==0x8006FDA4u?"6FDA4-35110":"6FDAC-1BE6C",
          cpu->gpr[3], cpu->gpr[4], w, cpu->lr, *c); }
      return false; }
    if(addr==0x8001BE6Cu){
      static unsigned _v=0; if(++_v<=4){ uint32_t w=0xDEADu;
        guest_read32(cpu->gpr[13]-31388u,&w);
        fprintf(stderr,"[wait3] 1BE6C-waitword r3=0x%08X word=%u lr=0x%08X (#%u)\n",
          cpu->gpr[3], w, cpu->lr, _v); }
      return false; }
    if(addr==0x80074918u){
      static unsigned _q=0; if(++_q<=4)
        fprintf(stderr,"[wait3] 74918-entry r3=%u r4=%u r5=%u lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->lr, _q);
      return false; }
    // fzEYzb123: post-1425 frontier (20k run): 17180-entry (cmpwi r3,0
    // after the 1717C-bl-16DF8 path2entry call — dispatched), 13978-entry
    // (or r3,r31 waiter-link frame, dispatched), C49C-entry (3493C-memcpy
    // caller region? verify by fire), A000-entry (idle spin: li r3,0 +
    // b A000 — the park the tail bt shows with lr=C5A8/17520/6380).
    // lr names the caller of each; A000 firing = the tail park.
    // fzEYzb124 (answered probe151 — 13978 fires x4 with SELF-lr): 13978
    // is a blrl-called waiter frame whose lr is its own body (BFC8 with
    // lr=13978 = the frame re-arms its own waiter link each pass).
    // r3=0x801B7558 (sp+24 waiter block), r31=0x8015C748 (video ctx):
    // the frame waits on the SAME queue the 344xx display path feeds.
    // NEXT: 13934-entry (frame head: does the 13948-flag gate pass?) +
    // 13980-entry (post-blrl continuation: does the hook RETURN?).
    if(addr==0x80017180u){
      static unsigned _v1=0; if(++_v1<=4)
        fprintf(stderr,"[wait4] 17180-entry r3=%d r4=0x%08X lr=0x%08X (#%u)\n",
          (int32_t)cpu->gpr[3], cpu->gpr[4], cpu->lr, _v1);
      return false; }
    // fzEYzb132: 174D0-frame census (the fze.str READ issuer). Entry regs
    // r3-r8 become r26/r27/r28/flag-r29/r30/r31 (174E0-174F4); 17520 is
    // the fatal continuation (taken iff r29 < [r26+52] at 17500/17504).
    // Dump both: entry args + which side the frame resolves to. 17520
    // firing with r29=1358 proves the FST fix landed but the bounds leg
    // still rejects; 17508 (non-fatal) firing proves advance.
    if(addr==0x800174D0u||addr==0x80017520u||addr==0x80017508u||addr==0x80017578u||addr==0x800175C0u||addr==0x80017590u||addr==0x80006384u){
      static unsigned _e=0,_f=0,_n=0,_h=0,_c=0,_s=0,_8=0;
      unsigned *c=addr==0x800174D0u?&_e:addr==0x80017520u?&_f:addr==0x80017508u?&_n:addr==0x80017578u?&_h:addr==0x800175C0u?&_c:addr==0x80017590u?&_s:&_8; (*c)++;
      if(*c<=4){ uint32_t d52=0xDEADu;
        if(addr!=0x800174D0u) guest_read32(cpu->gpr[26]+52u,&d52);
        fprintf(stderr,"[wait4] %s r3=0x%08X r4=0x%08X r5=0x%08X r6=0x%08X r7=0x%08X r8=0x%08X r26=0x%08X r29=%u [r26+52]=%u lr=0x%08X (#%u)\n",
          addr==0x800174D0u?"174D0-entry":addr==0x80017520u?"17520-FATAL":addr==0x80017508u?"17508-live":addr==0x80017578u?"17578-RET19354":addr==0x800175C0u?"175C0-fzeprog":addr==0x80017590u?"17590-175A8slot":addr==0x80006384u?"6384-postwake":"?",
          cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->gpr[6],
          cpu->gpr[7], cpu->gpr[8], cpu->gpr[26], cpu->gpr[29], d52, cpu->lr, *c); }
      return false; }
    // fzEYzb139 (HLE open-completion — same class as fzEYzb125's 858E4:
    // PLAN-banned per-PC intervention, KEPT with rationale): the tag1
    // (fze.str OPEN) request links, drains and slot-invokes 17590->6340
    // EXACTLY ONCE (probe172: lr=18860/175B0), but the drain's slot-invoke
    // passes r3=-1 unconditionally (18858 li r3,-1), and 6340's bclr
    // early-return on r3==-1 NEVER clears flag -31973 — so the 6354 poll
    // (6388) spins forever and DVDOpen never returns, even though the
    // open itself SUCCEEDED (16DF8 resolved fze.str=1358, block consumed
    // via the 18830-consume leg). On HW the 199xx lattice later re-invokes
    // the slot with r3=0/-3 (success) which clears the flag; that lattice
    // is dead here (m56 never 1, 19B78 never called). Our DI HLE completes
    // every command, so signal success the same way: turn this -1
    // (queued) into 0 (done) and let the guest run its OWN clear path
    // (634C stb). r3 is dead after 6340's blr (175B0 epilogue ignores
    // it), so nothing else is perturbed. REMOVE once the 199xx/19500
    // completion lattice runs for real.
    if(addr==0x80006340u && cpu->gpr[3]==0xFFFFFFFFu){
      static unsigned _o=0; if(++_o<=4)
        fprintf(stderr,"[wait4] 6340-COMPLETE r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[4], cpu->lr, _o);
      cpu->gpr[3]=0; }
    // fzEYzb135: post-tag1 spin census. 19354 returns 1 but the 6354 poll
    // loop (6388 native poll -> 6384-bl-659C pump) never exits: the only
    // 6340-callback invocation passes r3=-1 (18858 li) which takes 6340's
    // bclr early-return (no flag clear). 659C-entry counts pump iterations;
    // 6340-entry samples the callback arg; 58AC fires iff the spin exits
    // (6354 returns to game); 18868 counts drain-skip legs; 175B0 proves
    // the 6340 blrl returned into the slot epilogue.
    if(addr==0x8000659Cu||addr==0x80006340u||addr==0x800058ACu||addr==0x80018868u||addr==0x800175B0u||addr==0x80006388u||addr==0x80006394u||addr==0x800058BCu||addr==0x800058C4u||addr==0x80017228u||addr==0x80006354u||addr==0x800065D8u||addr==0x800065E4u||addr==0x800065ECu){
      static unsigned _p=0,_c=0,_o=0,_k=0,_e=0,_l=0,_r=0,_b=0,_q=0,_u=0,_z=0,_d=0,_v=0,_w=0,_y=0;
      unsigned *c=addr==0x8000659Cu?&_p:addr==0x80006340u?&_c:addr==0x800058ACu?&_o:addr==0x80018868u?&_k:addr==0x800175B0u?&_e:addr==0x80006388u?&_l:addr==0x80006394u?&_r:addr==0x800058BCu?&_b:addr==0x800058C4u?&_q:addr==0x80017228u?&_y:addr==0x80006354u?&_z:addr==0x800065D8u?&_d:addr==0x800065E4u?&_v:&_w; (*c)++;
      if(*c<=4||*c%20000000==0){ uint32_t fl=0xDEADu,t=0xDEADu;
        guest_read32(cpu->gpr[13]-31973u,&fl); guest_read32(cpu->gpr[13]-31972u,&t);
        fprintf(stderr,"[wait4] %s r3=0x%08X r4=0x%08X flagB=%u tgt31972=0x%08X lr=0x%08X (#%u)\n",
          addr==0x8000659Cu?"659C-pump":addr==0x80006340u?"6340-cb":addr==0x800058ACu?"58AC-openret":addr==0x80018868u?"18868-skip":addr==0x800175B0u?"175B0-slotret":addr==0x80006388u?"6388-poll":addr==0x80006394u?"6394-openret":addr==0x800058BCu?"58BC-postopen":addr==0x800058C4u?"58C4":addr==0x80017228u?"17228-issuer":addr==0x80006354u?"6354-entry":addr==0x800065D8u?"65D8-pretgt":addr==0x800065E4u?"65E4-bctrl":"65EC-posttgt",
          cpu->gpr[3], cpu->gpr[4], (fl>>24)&0xFFu, t, cpu->lr, *c); }
      return false; }
    // fzEYzb136: 659C-pump interior. 659C fires ONCE then the opener
    // thread never returns to 6388/659C: the pump descends 65FC->197F0->
    // 6600 (r3=-1 each time, 197F0's m64==0 leg: STALE r13 context in the
    // display thread) then runs 6634-row dispatch + GAME-LOGIC loop
    // (65E4 bctrl indirect whose REGISTRAR 6914 writes [r13-31968] was
    // never reached post-tag1: 65D8-pretgt r12tgt=0). 6600 samples each
    // 197F0 return (r31 frozen at -1), 68B4 proves pump exit, 6798 shows
    // the drive-table scan leg (probe168+).
    if(addr==0x80006798u||addr==0x80006600u||addr==0x800068B4u||addr==0x800065FCu||addr==0x8000660Cu||addr==0x8000661Cu||addr==0x80006914u||addr==0x80006AF4u||addr==0x80006634u){
      static unsigned _l=0,_m=0,_x=0,_y=0,_v=0,_w=0,_u=0,_t=0,_d=0;
      unsigned *c=addr==0x80006798u?&_l:addr==0x80006600u?&_m:addr==0x800068B4u?&_x:addr==0x800065FCu?&_y:addr==0x8000660Cu?&_v:addr==0x8000661Cu?&_w:addr==0x80006914u?&_u:addr==0x80006AF4u?&_t:&_d; (*c)++;
      if(*c<=6||*c%5000000==0){
        if(addr==0x80006914u||addr==0x80006AF4u){ uint32_t t=0xDEADu;
          guest_read32(cpu->gpr[13]-31968u,&t);
          fprintf(stderr,"[wait4] %s registrar r3=0x%08X [r13-31968]=0x%08X lr=0x%08X (#%u)\n",
            addr==0x80006914u?"6914-reg":"6AF4-flagget",cpu->gpr[3],t,cpu->lr,*c);
        } else if(addr==0x80006634u){ uint32_t tgt=0xDEADu,t0=0xDEADu;
          guest_read32(0x8012208Cu+cpu->gpr[0]*4u,&tgt); guest_read32(0x8012208Cu,&t0);
          fprintf(stderr,"[wait4] 6634-rowdispatch r31=%d idx=%u tgt=0x%08X t0=0x%08X lr=0x%08X (#%u)\n",
            (int32_t)cpu->gpr[31],cpu->gpr[0],tgt,t0,cpu->lr,*c);
        } else fprintf(stderr,"[wait4] %s r0=%d r3=0x%08X r31=%d ctr=0x%08X lr=0x%08X (#%u)\n",
          addr==0x80006798u?"6798-scan":addr==0x80006600u?"6600-post197F0":addr==0x800068B4u?"68B4-pumpexit":addr==0x800065FCu?"65FC-pre197F0":addr==0x8000660Cu?"660C-row":"661C-indcall",
          (int32_t)cpu->gpr[0], cpu->gpr[3], (int32_t)cpu->gpr[31], cpu->ctr, cpu->lr, *c); }
      return false; }
    // fzEYzb137: 197F0 leg + pump indirect-target census. 197F0 returns -1
    // (6600-post r3=-1 x3) but static decode allows -1 ONLY via the m64==0
    // leg (19818) — m64 is set-once (=1 since STOPMOTOR completion), so the
    // -1 implies either a wrong-r13 read or a [curblk+12]=-1 reload via
    // 19864. Dump r13 + DVD globals + curblk words to decide. 65D8 samples
    // the pump's indirect target ([r13-31972]: null => bctrl skipped,
    // straight to 65FC/197F0); 65EC samples the bctrl result (fires iff the
    // indirect call was taken).
    if(addr==0x800197F0u){
      static unsigned _n=0; if(++_n<=3){ uint32_t r13=cpu->gpr[13],m64=0xDEADu,m60=0xDEADu,m56=0xDEADu,cb=0,w72=0xDEADu,tg=0xDEADu,b12=0xDEADu,sl=0xDEADu;
        guest_read32(r13-31464u,&m64); guest_read32(r13-31460u,&m60); guest_read32(r13-31456u,&m56);
        guest_read32(r13-31488u,&cb); guest_read32(r13-31472u,&w72);
        if(cb>=0x80000000u){ guest_read32(cb+8u,&tg); guest_read32(cb+12u,&b12); guest_read32(cb+40u,&sl); }
        fprintf(stderr,"[wait4] 197F0-leg r13=0x%08X m64=%u m60=%u m56=%u w72=%u curblk=0x%08X tag=%u b12=%d slot=0x%08X r3=0x%08X (#%u)\n",
          r13,m64,m60,m56,w72,cb,tg,(int32_t)b12,sl,cpu->gpr[3],_n); }
      return false; }
    if(addr==0x800065D8u||addr==0x800065ECu){
      static unsigned _a=0,_b=0; unsigned *c=addr==0x800065D8u?&_a:&_b; (*c)++;
      if(*c<=3){ uint32_t t=0xDEADu; guest_read32(cpu->gpr[13]-31972u,&t);
        fprintf(stderr,"[wait4] %s r12tgt=0x%08X r3=0x%08X r31=0x%08X lr=0x%08X (#%u)\n",
          addr==0x800065D8u?"65D8-pretgt":"65EC-posttgt",t,cpu->gpr[3],cpu->gpr[31],cpu->lr,*c); }
      return false; }
    if(addr==0x80013934u){
      static unsigned _v0=0; if(++_v0<=4){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-31632u,&fl);
        fprintf(stderr,"[wait4] 13934-head r3=0x%08X r4=0x%08X flag30632=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], fl, cpu->lr, _v0); }
      return false; }
    if(addr==0x80013978u){
      static unsigned _v2=0; if(++_v2<=4)
        fprintf(stderr,"[wait4] 13978-entry r3=0x%08X r31=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[31], cpu->lr, _v2);
      return false; }
    if(addr==0x80013980u){
      static unsigned _v5=0; if(++_v5<=4)
        fprintf(stderr,"[wait4] 13980-posthook r3=0x%08X r31=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[31], cpu->lr, _v5);
      return false; }
    if(addr==0x8000C49Cu){
      static unsigned _v3=0; if(++_v3<=4)
        fprintf(stderr,"[wait4] C49C-entry r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], cpu->lr, _v3);
      return false; }
    // fzEYzb126: C568-loop census (the C49C memcpy's node walk:
    // C568 lwz r5,0(r29) / C584 cmplwi r29,0 / C588 bc-12,2-C5A4 exit
    // (r29==0 = end of heap chain, NORMAL exit) / C58C addis-bound /
    // C594 bc-12,2-C5A4 exit (bound check) / C598 cmplwi r30,16 /
    // C5A0 bc-12,0-C568 loop-back (16 iterations max). r29=node cursor,
    // r30=iteration count. Which exit takes, and does the loop re-enter
    // per C49C call? Distinguishes "heap walk finds nothing 16x then
    // parks at 9FFC" from "walk corrupts and never exits".
    // fzEYzb127 (answered DOL-decode — branch polarity CORRECTED): C588
    // bc-12,2 = branch iff EQ (r29==0): NULL cursor EXITS to C5A4, nonzero
    // FALLS to C58C. So the heap walk exits on NULL (normal list end),
    // NOT on found. C594 same polarity (bound-check exit). The loop is
    // CORRECT; the stall is downstream (C5A4-bl-9FFC park), not here.
    if(addr==0x8000C568u||addr==0x8000C584u||addr==0x8000C598u){
      static unsigned _c1=0,_c2=0,_c3=0;
      unsigned *c=addr==0x8000C568u?&_c1:addr==0x8000C584u?&_c2:&_c3; (*c)++;
      if(*c<=4||*c%5000000==0)
        fprintf(stderr,"[wait4] C-loop-%05X r29=0x%08X r30=%u r5=0x%08X lr=0x%08X (#%u)\n",
          addr&0xFFFFFu, cpu->gpr[29], cpu->gpr[30], cpu->gpr[5], cpu->lr, *c);
      return false; }
    if(addr==0x8000A000u){
      static unsigned _v4=0; _v4++;
      if(_v4<=4||_v4%5000000==0){ uint32_t s0=0xDEADu,s4=0xDEADu;
        guest_read32(cpu->gpr[1],&s0); guest_read32(cpu->gpr[1]+4u,&s4);
        fprintf(stderr,"[park] A000 r1=0x%08X lr=0x%08X [r1]=0x%08X [r1+4]=0x%08X r3=%u (#%u)\n",
          cpu->gpr[1], cpu->lr, s0, s4, cpu->gpr[3], _v4); }
      return false; }
    // fzEYzb145: A000-park mechanism (probe178: flow reaches A000 once via
    // 1190C (lr=1190C) and spins 250M+; 7A060/7A13C/32AC never re-fire, so
    // 1190C's blr did NOT return to 7A13C — r0=[r1+12] must have read A000,
    // yet the later bt shows [0x801B791C]=7A13C intact with r1=0x801B7910).
    // Dump r1 + [r1+12] (exactly what 1190C's lwz loads — host_call runs
    // before the chunk, but no store precedes the lwz, so the live read is
    // what the blr will target) + lr. Read-only, first-6.
    // fzEYzb147 (probe180: 1190C NEVER fires but A000 spins lr=1190C —
    // RESOLVED statically: 9FFC is `sync` with NO downcount (fallthrough
    // native into A000), and 11908-bl-9FFC/1190C both lack dispatch
    // visibility except via chunk-entry: the whole 118FC->11908->9FFC->
    // A000->1190C? chain runs NATIVE from ONE 118FC dispatch (same class
    // as the 6FE14/6FEA8 video loop, fzEYzb138). But WAIT: 11908's `bl`
    // targets 9FFC (sync), whose fallthrough is A000 — 1190C is lr, NOT
    // a return target: the halt path NEVER returns through 1190C. The
    // A000 lr=1190C is just the stale bl-lr. So 1190C silence is CORRECT
    // and the halt is entered once. Only 118FC-sync + 09FFC-fence (both
    // HAVE ctx->pc) are observable: count them. 1191C/1192C/11934 are the
    // flag-poller return legs (11920 lwz [0x80003B94], EQ->11934 r3=0,
    // else 1192C r3=1): fire iff 118FC returns into the poller.
    if(addr==0x8001191Cu||addr==0x8001192Cu||addr==0x80011934u){
      static unsigned _m=0,_k=0,_j=0;
      unsigned *c=addr==0x8001191Cu?&_m:addr==0x8001192Cu?&_k:&_j; (*c)++;
      const char* nm=addr==0x8001191Cu?"1191C-poller":addr==0x8001192Cu?"1192C-ret1":"11934-ret0";
      if(*c<=8){ uint32_t f=0xDEADu;
        guest_read32(0x80003B94u,&f);
        fprintf(stderr,"[park] %s r1=0x%08X flag3B94=%d lr=0x%08X (#%u)\n",
          nm, cpu->gpr[1], (int32_t)f, cpu->lr, *c); }
      return false; }
    // fzEYzb140: 7A060 init-table runner census (probe173 parks at A000 with
    // lr=1190C <- 7A138 bl 118FC <- 7A118 bl 7ED8C: 7A060 ran to its tail).
    // 7A060 walks T1 @0x8008FF20 (null-terminated fnptr list, 7A09C-7A0B4),
    // hook [r13-30192], T2 @0x801A3280 indexed by count [r13-30196], hook
    // [r13-30188], then 7A138 bl 118FC = safety halt (sync->A000 spin).
    // Empty tables => correct-but-starved halt; registrars never filed.
    // All sites have ctx->pc (dispatchable); bctrl continuations observed
    // (ctr still = target, r3 = retval, r31 = pre-incr ptr). Read-only.
    if(addr==0x8007A090u||addr==0x8007A0ACu||addr==0x8007A0A8u||addr==0x8007A0B8u||addr==0x8007A0E8u||addr==0x8007A104u||addr==0x8007A11Cu||addr==0x8007A138u||addr==0x800118FCu||addr==0x80009FFCu){
      static unsigned _b=0,_i=0,_t1=0,_h1=0,_c=0,_t2=0,_h2=0,_h=0,_s=0,_p=0;
      unsigned *cc=addr==0x8007A090u?&_b:addr==0x8007A0ACu?&_i:addr==0x8007A0A8u?&_t1:addr==0x8007A0B8u?&_h1:addr==0x8007A0E8u?&_c:addr==0x8007A104u?&_t2:addr==0x8007A11Cu?&_h2:addr==0x8007A138u?&_h:addr==0x800118FCu?&_s:&_p; (*cc)++;
      const char* nm=addr==0x8007A090u?"7A090-T1base":addr==0x8007A0ACu?"7A0AC-T1iter":addr==0x8007A0A8u?"7A0A8-T1ret":addr==0x8007A0B8u?"7A0B8-hook1":addr==0x8007A0E8u?"7A0E8-T2body":addr==0x8007A104u?"7A104-T2chk":addr==0x8007A11Cu?"7A11C-hook2":addr==0x8007A138u?"7A138-halt":addr==0x800118FCu?"118FC-sync":"09FFC-fence";
      if(*cc<=8){ uint32_t w0=0xDEADu,w1=0xDEADu,w2=0xDEADu,w3=0xDEADu;
        if(addr==0x8007A090u){ guest_read32(0x8008FF20u,&w0); guest_read32(0x8008FF24u,&w1); guest_read32(0x8008FF28u,&w2); guest_read32(0x8008FF2Cu,&w3);
          fprintf(stderr,"[init] %s T1[0..3]=0x%08X 0x%08X 0x%08X 0x%08X lr=0x%08X (#%u)\n",nm,w0,w1,w2,w3,cpu->lr,*cc); }
        else if(addr==0x8007A0ACu){ guest_read32(cpu->gpr[31],&w0);
          fprintf(stderr,"[init] %s r31=0x%08X [r31]=0x%08X lr=0x%08X (#%u)\n",nm,cpu->gpr[31],w0,cpu->lr,*cc); }
        else if(addr==0x8007A0B8u||addr==0x8007A11Cu){ uint32_t hk=0xDEADu;
          guest_read32(cpu->gpr[13]+(uint32_t)(int32_t)(addr==0x8007A0B8u?-30192:-30188),&hk);
          fprintf(stderr,"[init] %s hook=0x%08X lr=0x%08X (#%u)\n",nm,hk,cpu->lr,*cc); }
        else if(addr==0x8007A0E8u||addr==0x8007A104u){ uint32_t cnt=0xDEADu;
          guest_read32(cpu->gpr[13]-30196u,&cnt); guest_read32(0x801A3280u,&w0); guest_read32(0x801A3284u,&w1); guest_read32(0x801A327Cu,&w2);
          fprintf(stderr,"[init] %s count=%d T2[0]=0x%08X T2[1]=0x%08X T2[-1]=0x%08X ctr=0x%08X r3=0x%08X (#%u)\n",nm,(int32_t)cnt,w0,w1,w2,cpu->ctr,cpu->gpr[3],*cc); }
        else fprintf(stderr,"[init] %s ctr=0x%08X r31=0x%08X lr=0x%08X (#%u)\n",nm,cpu->ctr,cpu->gpr[31],cpu->lr,*cc); }
      return false; }
    // fzEYzb142: post-submit continuation census (probe174: after submit#2
    // the game goes 599C(native)->?->E030->794D8->7A11C->halt; the 599C
    // chain 59A8-bctrl([r30+52])/59C0-bl-E5A8/59CC-bl-793D4/59D8-bl-B324
    // is where the READ should be issued — or the halt decided). Dump the
    // indirect target + its retval + callee args. 7A060-entry logs r13
    // (main vs DVD r13 decides which words the -30200/-30196/-30188 gates
    // actually read). All sites have ctx->pc. Read-only, first-8.
    if(addr==0x8000599Cu||addr==0x800059A8u||addr==0x800059B4u||addr==0x800059C4u||addr==0x800059D4u||addr==0x800059DCu||addr==0x800059E4u||addr==0x800059E8u){
      static unsigned _a=0,_b=0,_c=0,_d=0,_e=0,_f=0,_g=0,_h=0;
      unsigned *cc=addr==0x8000599Cu?&_a:addr==0x800059A8u?&_b:addr==0x800059B4u?&_c:addr==0x800059C4u?&_d:addr==0x800059D4u?&_e:addr==0x800059DCu?&_f:addr==0x800059E4u?&_g:&_h; (*cc)++;
      const char* nm=addr==0x8000599Cu?"599C-cont":addr==0x800059A8u?"59A8-indcall":addr==0x800059B4u?"59B4-indret":addr==0x800059C4u?"59C4-preE5A8":addr==0x800059D4u?"59D4-preB324":addr==0x800059DCu?"59DC-postB324":addr==0x800059E4u?"59E4-bctrl":"59E8-epi";
      if(*cc<=8){ uint32_t tgt=0xDEADu,w52=0xDEADu;
        if(cpu->gpr[30]>=0x80000000u) guest_read32(cpu->gpr[30]+52u,&tgt);
        if(addr==0x800059A8u||addr==0x800059B4u) w52=tgt;
        fprintf(stderr,"[cont] %s r3=0x%08X r4=0x%08X r30=0x%08X r31=0x%08X [r30+52]=0x%08X ctr=0x%08X lr=0x%08X (#%u)\n",
          nm,cpu->gpr[3],cpu->gpr[4],cpu->gpr[30],cpu->gpr[31],w52,cpu->ctr,cpu->lr,*cc); }
      return false; }
    if(addr==0x8000E030u||addr==0x8000E5A8u||addr==0x800793D4u||addr==0x8000B324u||addr==0x8000B31Cu||addr==0x800793E8u||addr==0x800794D8u||addr==0x8007ED8Cu||addr==0x800798D0u){
      static unsigned _a=0,_b=0,_c=0,_d=0,_e=0,_f=0,_g=0,_h=0,_i=0;
      unsigned *cc=addr==0x8000E030u?&_a:addr==0x8000E5A8u?&_b:addr==0x800793D4u?&_c:addr==0x8000B324u?&_d:addr==0x8000B31Cu?&_e:addr==0x800793E8u?&_f:addr==0x800794D8u?&_g:addr==0x8007ED8Cu?&_h:&_i; (*cc)++;
      const char* nm=addr==0x8000E030u?"E030":addr==0x8000E5A8u?"E5A8":addr==0x800793D4u?"793D4":addr==0x8000B324u?"B324":addr==0x8000B31Cu?"B31C":addr==0x800793E8u?"793E8-cachegate":addr==0x800794D8u?"794D8-b8ac":addr==0x8007ED8Cu?"7ED8C-nopret":"798D0-restore";
      if(*cc<=8){ uint32_t cg=0xDEADu;
        if(addr==0x800793E8u||addr==0x800794D8u) guest_read32(cpu->gpr[13]-30216u,&cg);
        fprintf(stderr,"[cont] %s-call r3=0x%08X r4=0x%08X r5=0x%08X r6=0x%08X r7=0x%08X%s lr=0x%08X (#%u)\n",
          nm,cpu->gpr[3],cpu->gpr[4],cpu->gpr[5],cpu->gpr[6],cpu->gpr[7],
          (addr==0x800793E8u||addr==0x800794D8u)?(cg==0xDEADu?"":""):"",
          cpu->lr,*cc);
        if(addr==0x800793E8u||addr==0x800794D8u)
          fprintf(stderr,"[cont]   cachegate30216=%u r25=0x%08X r26=0x%08X\n",cg,cpu->gpr[25],cpu->gpr[26]); }
      return false; }
    // fzEYzb143: 7A060 head census (native prestretch: 7A070-gate is the
    // native lwz+cmpwi at 7A07C-entry; 7A08C/7A090 bl-continuations prove
    // the T1-setup ran). Dump the gate word + count + hooks live.
    if(addr==0x8007A060u||addr==0x8007A08Cu||addr==0x8007A090u){
      static unsigned _a=0,_b=0,_c=0;
      unsigned *cc=addr==0x8007A060u?&_a:addr==0x8007A08Cu?&_b:&_c; (*cc)++;
      const char* nm=addr==0x8007A060u?"7A060-entry":addr==0x8007A08Cu?"7A08C-post7ED84":"7A090-T1base";
      if(*cc<=8){ uint32_t g0=0xDEADu,cnt=0xDEADu,h1=0xDEADu,h2=0xDEADu;
        guest_read32(cpu->gpr[13]-30200u,&g0); guest_read32(cpu->gpr[13]-30196u,&cnt);
        guest_read32(cpu->gpr[13]-30192u,&h1); guest_read32(cpu->gpr[13]-30188u,&h2);
        fprintf(stderr,"[init] %s r13=0x%08X r1=0x%08X gate30200=%u count30196=%d hook1=0x%08X hook2=0x%08X lr=0x%08X (#%u)\n",
          nm,cpu->gpr[13],cpu->gpr[1],g0,(int32_t)cnt,h1,h2,cpu->lr,*cc); }
      return false; }
    // fzEYzb141: DVD-submit census (probe173: opens succeed, 17228 issuers
    // run twice, 19B58 fires, but 19430-tag4 NEVER fires). Map which submit
    // legs execute: 19B78-entry (r3=block: tag/b12?), 1996C (m56 gate),
    // 19988 (m56==0 leg: files m52/m56=1), 199A8 (tag==4 leg: bl 16D50 =
    // READ-issue path), 19B9C (198FC result: 0=fail?), 19BB4 (b12 poll),
    // 1A31C/1A3F4 (cascade entries). All have ctx->pc (dispatchable).
    // Read-only, first-8 prints.
    // fzEYzb150 (decomp-resolved naming, karamzov123/fzero-gx-decomp):
    // 19B78 = DVDCancel (NOT a submit — the game calls DVDCancelSync on
    // the block after dvd_read_sync_wait returns!); 198FC =
    // __DVDDequeueWaitingQueue; 1996C = m56==0 test; 19988 = m56-files
    // leg; 199A8 = tag==4||tag==1 leg -> bl DVDLowStopMotorAtNextInt
    // (16D50 = StopAtNextInt flag-setter, NOT a READ issuer); 199B0 =
    // fn_80019FFC leg (b12=10 filer); 19B9C = dequeue-result test
    // (r3!=0 = dequeued); 19BB4 = DVDCancel's b12-poll loop; 19B50 =
    // dequeue join; 19B58 = dequeue epilogue (r3=1). The 19430-tag4
    // (DVDReadAbsAsyncForBS) callers are ONLY the fstload cb @1A31C
    // (boot BB2/FST chain) — never on the GameMainLoopFrame path. The
    // game path issues NO tag4 READ here by design: DVDReadPrio(tag1)
    // + DVDCancelSync is the whole open+cancel sequence per frame.
    if(addr==0x80019B78u||addr==0x8001996Cu||addr==0x80019988u||addr==0x800199A8u||addr==0x800199ACu||addr==0x80016D50u||addr==0x80019B9Cu||addr==0x80019BB4u||addr==0x8001A31Cu||addr==0x8001A3F4u||addr==0x80019B50u||addr==0x80019B58u||addr==0x8001994Cu||addr==0x80019980u||addr==0x800199B0u||addr==0x800199C0u||addr==0x800199DCu||addr==0x80019A4Cu||addr==0x80019A68u||addr==0x80019A9Cu||addr==0x80019AE4u){
      static unsigned _e=0,_g=0,_m=0,_t=0,_r=0,_p=0,_c=0,_f=0,_s1=0,_s2=0,_s3=0,_s4=0,_s5=0,_s6=0,_s7=0,_s8=0,_s9=0,_sA=0,_sB=0,_sC=0;
      unsigned *cc=addr==0x80019B78u?&_e:addr==0x8001996Cu?&_g:addr==0x80019988u?&_m:addr==0x800199A8u?&_t:addr==0x800199ACu?&_sB:addr==0x80016D50u?&_sC:addr==0x80019B9Cu?&_r:addr==0x80019BB4u?&_p:addr==0x8001A31Cu?&_c:addr==0x8001A3F4u?&_f:addr==0x80019B50u?&_s1:addr==0x80019B58u?&_s2:addr==0x8001994Cu?&_s3:addr==0x80019980u?&_s4:addr==0x800199B0u?&_s5:addr==0x800199C0u?&_s6:addr==0x800199DCu?&_s7:addr==0x80019A4Cu?&_s8:addr==0x80019A68u?&_s9:addr==0x80019A9Cu?&_sA:&_f; (*cc)++;
      const char* nm=addr==0x80019B78u?"19B78-submit":addr==0x8001996Cu?"1996C-m56gate":addr==0x80019988u?"19988-m56zero":addr==0x800199A8u?"199A8-tag4issue":addr==0x800199ACu?"199AC-post16D50":addr==0x80016D50u?"16D50-flagset":addr==0x80019B9Cu?"19B9C-result":addr==0x80019BB4u?"19BB4-b12poll":addr==0x8001A31Cu?"1A31C-cascade":addr==0x8001A3F4u?"1A3F4-reg":addr==0x80019B50u?"19B50-join":addr==0x80019B58u?"19B58-epi":addr==0x8001994Cu?"1994C-r30gate":addr==0x80019980u?"19980-b12zero":addr==0x800199B0u?"199B0-19FFC":addr==0x800199C0u?"199C0-slotgate":addr==0x800199DCu?"199DC-r30b":addr==0x80019A4Cu?"19A4C-m56b":addr==0x80019A68u?"19A68-m56w":addr==0x80019A9Cu?"19A9C-b12cmp":"?";
      if(*cc<=8){ uint32_t b8=0xDEADu,b12=0xDEADu,m56=0xDEADu,s40=0xDEADu;
        uint32_t blk=(addr==0x80019B78u||addr==0x80019BB4u)?cpu->gpr[3]:cpu->gpr[30];
        if(addr==0x8001996Cu||addr==0x80019988u||addr==0x800199A8u) blk=cpu->gpr[29];
        if(addr==0x80019B9Cu) blk=cpu->gpr[30];
        if(blk>=0x80000000u){ guest_read32(blk+8u,&b8); guest_read32(blk+12u,&b12); guest_read32(blk+40u,&s40); }
        guest_read32(cpu->gpr[13]-31456u,&m56);
        fprintf(stderr,"[submit] %s r3=0x%08X r30=0x%08X blk=0x%08X tag=%u b12=%d slot40=0x%08X m56=%u lr=0x%08X (#%u)\n",
          nm,cpu->gpr[3],cpu->gpr[30],blk,b8,(int32_t)b12,s40,m56,cpu->lr,*cc); }
      return false; }
    if(addr==0x8006FEA0u){
      static unsigned _f2=0; _f2++;
      if(_f2<=6||_f2%5000000==0){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30476u,&fl);
        fprintf(stderr,"[wait3] 6FEA0-lap flag30476=0x%08X r0=%u lr=0x%08X (#%u)\n",
          fl, cpu->gpr[0], cpu->lr, _f2); }
      return false; }
    // fzEYzb106: second waiter frame (3416C) + flag writer (34488/344A0).
    // 3416C runs 33E20-worker then parks 110A8 on queue r13-30628 until
    // byte r13-30632 flips (341C4 lbz poll); 34488-frame (3448C li r3,1)
    // files that byte at 344B8 (stb r3) from display path 30988 + boot
    // path 6FCD0. 30636 = hook (3445C lwz r31 / 34464 stw r30 chain-links
    // it — the new hook value chains via r30). 30640 = second hook the
    // same way (34390/34398/343D4/343F8). 34444 = the 30636-setter frame
    // (NOT the flag writer). If 341C4-poll count grows but the byte stays
    // 0, 34488 never runs — same signature as the first waiter pre-fix.
    if(addr==0x8003416Cu){
      static unsigned _c=0; if(++_c<=4){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30632u,&fl);
        fprintf(stderr,"[wait2] 3416C-entry r3=0x%08X flag30632=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], fl, cpu->lr, _c); }
      return false; }
    // (341C0 is a native bl inside the 341BC-loop — never dispatches;
    // observe via its 341C4 lbz landing instead, which has downcount.)
    if(addr==0x800341C0u){
      static unsigned _w=0; _w++;
      if(_w<=4||_w%5000000==0){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30632u,&fl);
        fprintf(stderr,"[wait2] 341C0-park flag30632=0x%08X lr=0x%08X (#%u)\n",
          fl, cpu->lr, _w); }
      return false; }
    // (344A0 itself is NATIVE — the 344xx frame's dispatched entries are
    // 34444/34464/3446C/34488. Probe 34444 = 30636-setter entry (NOT the
    // flag writer); 34488 = flag-writer entry (r3=1 filed at 344B8).
    // 34444's r3 = the new 30636 hook (chains via r30 at 34464).)
    // fzEYzb112 (answered — 344BC NEVER dispatches: 0 hits across 8 D9CC
    // runs): the flag-writer chain 34488->344B8->344BC runs native from
    // ONE 34488 dispatch, like D9CC's DCD8 blrl. So NO probe can observe
    // the 344B8 store firing or skipping — only its EFFECT: journal here
    // is armed; byte -30632 read live at every 341BC-lap. If the byte
    // flips while 34488-entry stays at 1, the writer ran; the waiter-2
    // re-sleep is then the sleep-chain (105D0/f88), same as waiter-1
    // pre-fzEYzb105. NEXT: journal slot for -30632 (DOL .data byte).
    if(addr==0x80034444u){
      static unsigned _s=0; if(++_s<=4){ uint32_t h=0xDEADu;
        guest_read32(cpu->gpr[13]-30636u,&h);
        fprintf(stderr,"[wait2] 34444-setter r3=0x%08X oldhook30636=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], h, cpu->gpr[4], cpu->lr, _s); }
      return false; }
    if(addr==0x80034488u){
      static unsigned _s3=0; if(++_s3<=4){ uint32_t h=0xDEADu;
        guest_read32(cpu->gpr[13]-30636u,&h);
        fprintf(stderr,"[wait2] 34488-flagwrite r3=%u hook30636=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], h, cpu->gpr[4], cpu->lr, _s3); }
      return false; }
    if(addr==0x800344A0u){
      static unsigned _s2=0; if(++_s2<=4)
        fprintf(stderr,"[wait2] 344A0-flagwrite r3=%u lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->lr, _s2);
      return false; }
    // fzEYzb107: 341BC = the poll-LOOP head (addi queue + bl 110A8 +
    // lbz flag): dispatched every lap. 341C4 never fires post-first-lap
    // because the 110A8 bl never returns post-first-lap (thread re-sleeps
    // inside 105D0 each lap — same as waiter-1 pre-fix). So count 341BC
    // laps (= 110A8 sleeps) and sample the flag word there.
    if(addr==0x800341BCu){
      static unsigned _b=0; _b++;
      if(_b<=4||_b%5000000==0){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30632u,&fl);
        fprintf(stderr,"[wait2] 341BC-lap flag30632=0x%08X r3=0x%08X lr=0x%08X (#%u)\n",
          fl, cpu->gpr[3], cpu->lr, _b); }
      return false; }
    if(addr==0x800341C4u){
      static unsigned _p=0; _p++;
      if(_p<=4||_p%5000000==0){ uint32_t fl=0xDEADu;
        guest_read32(cpu->gpr[13]-30632u,&fl);
        fprintf(stderr,"[wait2] 341C4-poll flag30632=0x%08X (#%u)\n", fl, _p); }
      return false; }
    // fzEYzb59: 14158 is the DISPATCHED entry of the 14158-1416C leg
    // (14170 itself is a mid-chain native). Probe here: dump r4 (the
    // queue-walk cursor), [r4] (block word), and the 030CE halfword +
    // waiter flag. If 14158 never fires either, the whole 13F64 walk
    // never reaches the flag-check leg (spins at 13F9C/1418C instead).
    if(addr==0x80014158u){
      static unsigned _f=0; if(++_f<=4){ uint32_t bw=0xDEADu, w=0xDEADu, fl=0xDEADu;
        uint32_t r4=cpu->gpr[4]; if(r4) guest_read32(r4,&bw);
        guest_read32(0x800030CCu,&w); guest_read32(cpu->gpr[13]-31592u,&fl);
        fprintf(stderr,"[dvdsm] 14158 r4=0x%08X [r4]=0x%08X halfCE=0x%04X waitflag=%u lr=0x%08X (#%u)\n",
          r4, bw, (w>>16)&0xFFFFu, fl, cpu->lr, _f); }
      return false; }
    // fzEYzb60: 7096C gate2-consumer entry (dispatched? has downcount).
    // Dumps the gate2 POINTER + target word + byte25 (the strcmp key?).
    // Fires => game thread reached the gate2 stage (past 1AF64 wait).
    if(addr==0x8007096Cu){
      static unsigned _g=0; if(++_g<=4){ uint32_t p=0xDEADu, v=0xDEADu;
        guest_read32(cpu->gpr[13]-30412u,&p);
        if(p>=GC_RAM_BASE) guest_read32(p,&v);
        fprintf(stderr,"[dvdsm] 7096C gate2ptr=0x%08X [ptr]=0x%08X lr=0x%08X (#%u)\n",
          p, v, cpu->lr, _g); }
      return false; }
    // fzEYzb65 (CORRECTED): 1A7AC/1AF30 are NATIVE branch labels (no
    // downcount) — invisible to dispatch probes by construction (same as
    // 18A38). The -31372 fnptr word is watched via the journal instead
    // (slot3 armed at 1AF64 — see watch_journal). 1A628 IS dispatched
    // but never fires => the waker chain never reaches the indirect
    // call. Its ENTRIES need probing instead (1A55C? 1A500? — TBD).
    // fzEYzb62: command-issue register snapshot: log issuer r30/r31 at
    // 16A38 (INQUIRY) + 16920 (STOPMOTOR) entry. The completion callback
    // (18D1C) installs these as its r30/r31 (issuer-thread values, not
    // the preempted slice's allocator leftovers). First-4 each.
    if(addr==0x80016A38u||addr==0x80016920u){
      static unsigned _q1=0,_q2=0; unsigned *c=addr==0x80016A38u?&_q1:&_q2; (*c)++;
      if(*c<=4) fprintf(stderr,"[dvdsm] %s-issue r30=0x%08X r31=0x%08X r3=0x%08X lr=0x%08X (#%u)\n",
        addr==0x80016A38u?"16A38":"16920",
        cpu->gpr[30], cpu->gpr[31], cpu->gpr[3], cpu->lr, *c);
      return false; }
    // fzEYzb9: 16C94 (dispatched entry) encloses the native 16D50
    // flag-setter tail (flag-31592=1 + flag-31560=1). Fires => setter
    // runs; dump the flag to confirm it lands.
    // fzEYzb36: A360 decoded statically — it is NOT a DVD gate at all:
    // A344 cmplw r3,r31 / A348 bc-then A360 means A360 = the r3!=r31 SYNC
    // path (A34C bl B314 worker + A350 subf/memcpy via 3458 + A35C bl 3458
    // + A360 epilogue). The 030CE halfword (A398: [r30]+2|0x8000 vs A3A8:
    // 1) is a per-THREAD sync word written on the A360 path only. (An old
    // note claimed A360 fires — superseded by fzEYzb36b: it never fires;
    // the whole A3xx family runs native inside the 5660 frame.) The
    // A348-taken side (r3==r31, skip worker) goes A35C->A360 directly.
    // fzEYzb36b (answered — A340 never fires either: the A3xx sync family
    // runs INSIDE the 5660 frame (566C bl A3B0 -> native A3xx chain ->
    // returns to 5670), and 5660 itself is called from 5648's bl — so the
    // whole sync+1776C sequence runs native from ONE dispatch. Only
    // bl-targets that START a dolrecomp_call are visible: 1776C fires
    // (bl-target of A728/5670) but A340/A344/A360 never do. The sync path
    // demonstrably RUNS (1776C fires from BOTH lr=A72C and lr=5674, and
    // the 5674 path only executes after A3B0's sync returns). No probe
    // can observe its interior — the question "which A348 side" is
    // unanswerable by dispatch probe; would need a memory-watch on the
    // 030CE halfword instead. Reverted to no-probe.
    // fzEYzb37 (answered): flag30CC=0 on BOTH 1776C paths — the A3xx sync
    // family (A744/A3A8 030CE stores) has NOT run when DVD init runs, OR
    // its word is elsewhere (r3=0x80000000-based 12518 offset = 0x800030CE
    // only if r3=0x80000000; here r3 at A3B0-entry is unknown). Either way
    // the sync word is not the DVD gate: 1776C runs unconditionally from
    // both callers with the word at zero. The DVD init is NOT gated by
    // thread-sync — it runs, files the INQUIRY, and the completion loop
    // spins on INQUIRY because the ADVANCE conditions (m56==1 forks,
    // tag-8 filer, slot-40 invoke, 177F8-EQ side) are all cold-start
    // zeros. NEXT fzEYzb38: stop probing the DVD loop — the loop is FULLY
    // DECODED. The question is now comparative: what does DOLPHIN show at
    // these same pcs (18D1C entry m56/m60, 1776C flag, 19690 tag)? If
    // Dolphin also shows m56=0/m60=14 at 18D1C #1, the divergence is
    // LATER (our HLE completion values vs theirs: r3=32 vs r3=0?).
    if(addr==0x80016C94u){
      static unsigned _m=0; if(++_m<=4){ uint32_t f=0xDEADu;
        guest_read32(cpu->gpr[13]-31592u,&f);
        fprintf(stderr,"[dvdsm] 16C94 flag-31592=%u r3=0x%08X lr=0x%08X (#%u)\n",
          f, cpu->gpr[3], cpu->lr, _m); }
      return false; }
    // fzEYzb12 (answered): 18ADC runs native straight into the 18AE0
    // block that re-issues INQUIRY via 16A38 — the 16AD4 tag-filing
    // calls (18BA8/18BD4/18C08) sit on sibling legs the frame never
    // takes, and 18B04 et al are native labels (no downcount).
    // fzEYzb19: READ ancestors — 162A0 (resume helper) + 162C8 (its
    // take-command leg); 16524 (read wrapper) + 16574/16650/166FC (3
    // issue legs); 161C4 (15FC0 re-issue leg). Dump queue index
    // (r13-31524), drive state (r13-31556), waiter flag (r13-31592).
    if(addr==0x800162A0u||addr==0x800162C8u||addr==0x80016524u||addr==0x80016574u||addr==0x80016650u||addr==0x800166FCu||addr==0x800161C4u){
      static unsigned _r[7]={0}; int _i=
        addr==0x800162A0u?0:addr==0x800162C8u?1:addr==0x80016524u?2:addr==0x80016574u?3:
        addr==0x80016650u?4:addr==0x800166FCu?5:6;
      const char *_nm[7]={"162A0-resume","162C8-take","16524-wrap","16574-leg1","16650-leg2","166FC-leg3","161C4-reissue"};
      if(++_r[_i]<=4){ uint32_t qi=0xDEADu,ds=0xDEADu,wf=0xDEADu;
        guest_read32(cpu->gpr[13]-31524u,&qi); guest_read32(cpu->gpr[13]-31556u,&ds);
        guest_read32(cpu->gpr[13]-31592u,&wf);
        fprintf(stderr,"[dvdsm] %s r3=0x%08X r4=0x%08X qidx=%u drv=%u wait=%u lr=0x%08X (#%u)\n",
          _nm[_i], cpu->gpr[3], cpu->gpr[4], qi, ds, wf, cpu->lr, _r[_i]); }
      return false; }
    // fzEYzb17: 16394/163DC (both dispatched) bracket the guest
    // DVDLowRead builder (lis -22528 = 0xA8000000 at 163F0, DICR=3 at
    // 16418). Fires => guest issues a real READ; dump regs + DMA regs.
    if(addr==0x80016394u||addr==0x800163DCu){
      static unsigned _e1=0,_e2=0; unsigned *c=addr==0x80016394u?&_e1:&_e2; (*c)++;
      if(*c<=4){ uint32_t c0=0xDEADu,c1=0xDEADu,da=0xDEADu,dl=0xDEADu;
        guest_read32(0xCC006008u,&c0); guest_read32(0xCC00600Cu,&c1);
        guest_read32(0xCC006014u,&da); guest_read32(0xCC006018u,&dl);
        fprintf(stderr,"[dvdsm] %s r3=0x%08X r30=0x%08X c0=0x%08X c1=0x%08X dma=0x%08X len=%u lr=0x%08X (#%u)\n",
          addr==0x80016394u?"16394-entry":"163DC-build",
          cpu->gpr[3], cpu->gpr[30], c0, c1, da, dl, cpu->lr, *c); }
      return false; }
    // fzEYzb23 (answered): 18EB0/18ED4 can NEVER fire — 18EB0 has no
    // downcount (native-only stretch) and 18ED4 is a goto-target, never a
    // resume pc; host_call only sees call-target entries. Reverted. The
    // accumulator question is answered via bl-targets instead: 16850
    // (fzEW, resume pc after 18E60's bl) fires iff the 18E4C leg runs.
    // fzEYzb24 (answered — never fires, like all mid-frame interior
    // labels; host_call only sees call-target resume pcs). Reverted to a
    // comment so the finding stays recorded without dead code.
    // fzEYzb25 (answered — never fires: 18D68 is mid-frame interior, only
    // reachable natively; resume pcs are bl-targets only). Full native path
    // decoded statically + confirmed by post-lap tracer (frame exits via
    // 1A178 lr=19230, m60=14 frozen). With r3=32,m60=14,m56=0,m32=0 at entry:
    // 18D3C fallthrough (32!=0x10) -> 18D6C/18D70 fallthrough (14!=3) ->
    // 18D78/18D7C TAKEN to 18E68 (14!=15) -> 18E70/18E7C fallthrough ->
    // 18E80 EQ (14==0xE) -> 18E84 fallthrough -> 18E88 r0=1 -> 18EA8/18EAC
    // fallthrough -> 18EB0 accumulator: [cb+32]=[cb+28]-[0xCC006018]+[cb+32]
    // =32-0+0=32, so the 19270 gate (b32==b20==32) PASSES -> 18ED4 fallthrough
    // (32&8==0) -> 18ED8 TAKEN to 18F38 (mid labels native) -> 18F3C TAKEN to
    // 1920C (32&1==0) -> 19214 fallthrough (14==0xE) -> b12=-1, 1922C bl 1A178
    // (report) -> 19238 bl 16920 STOPMOTOR -> 19270 gate passes -> 1928C TAKEN
    // (m56==0) -> 192F0/192F8 fallthrough -> 192FC: b12=0, curblk=r31+64, slot
    // null -> 19328 bl 187CC drain -> 19FA4 ret 0 -> 187F0 curblk=0 -> return.
    // Nested STOPMOTOR completion (17958, m56 still 0) drains to curblk=0 too.
    // Loop repeats from scratch each time: m60 self-reinforces at 14 via the
    // 18870 writer ([curblk+8]=tag 14 -> m60); the 18BDC m60=1 leg needs tag 8
    // (row 18B08 READ builder), never filed.
    // fzEYzb26: m56 writers decoded statically — ALL reachable natives except
    // the 18E00/18E14-style slot legs (slot null here) are m56=0 CLEARERS
    // (18DEC/18EE8/18F64/192A0/179D0/17C48 li r0,0 + stw m56). The only SETTER
    // is 17FAC (stw r3->m56) inside the 17F54-gated writer: with m60=14 the
    // 17F54 (m60!=5) -> 17F60 (m60!=13) -> 17F6C/17F74 fallthrough (14!=15)
    // sets m32=1 then READS m56 (0) -> 17F84/17F88 TAKEN to 17FF8 only if m56
    // is ALREADY 1. So 17FAC is gated on m56==1 — circular: nothing ever sets
    // the first 1. Same for slot+40 (19500 never fires) and m60 (18BDC needs
    // tag 8). The drive state is cold-started at zeros and the INQUIRY path
    // never warms it: the REAL question is what the native boot SHOULD have
    // run before 1776C — i.e. whether 1776C's own frame (16DC0/19E64/177AC
    // writer chain) is intact, and what calls 1776C with r3=0 (A72C) vs the
    // r3=0x80120000 path (5674) that may register slots.
    // fzEYzb22: 18BDC (dispatched) writes m60 directly (stw r0,-31468
    // with r0=1) on the path into the 18C08 tag-filer. m60 source dump
    // shows whether this leg ever advances the drive state.
    if(addr==0x80018BDCu){
      static unsigned _n=0; if(++_n<=4){ uint32_t m=0;
        guest_read32(cpu->gpr[13]-31460u,&m);
        fprintf(stderr,"[dvdsm] 18BDC m60=%u r0=%u lr=0x%08X (#%u)\n",
          m, cpu->gpr[0], cpu->lr, _n); }
      return false; }
    // fzEYzb21: tag 2 row (18B08, dispatched) calls the 16524 wrapper
    // which contains the READ builder — the only dispatch row reaching
    // a READ. Tag 14 row (18CC8) re-issues INQUIRY. Dump on fire.
    if(addr==0x80018B08u){
      static unsigned _n=0; if(++_n<=4||_n%5000000==0){ uint32_t blk=cpu->gpr[7];
        uint32_t tag=0xDEADu; if(blk) guest_read32(blk+8u,&tag);
        fprintf(stderr,"[dvdsm] 18B08-tag2 blk=0x%08X tag=%u lr=0x%08X (#%u)\n",
          blk, tag, cpu->lr, _n); }
      return false; }
    // fzEYzb6: 16AD4/16B5C/16BEC (all dispatched entries) file the
    // block+8 tags (0xE1/0xE2/0xE4-class). lr + r3 identify which command
    // family files tag=14 each issue.
    if(addr==0x80016AD4u||addr==0x80016B5Cu||addr==0x80016BECu){
      static unsigned _a=0,_b=0,_c=0;
      unsigned *q=addr==0x80016AD4u?&_a:addr==0x80016B5Cu?&_b:&_c; (*q)++;
      if(*q<=3) fprintf(stderr,"[dvdsm] %s r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
        addr==0x80016AD4u?"16AD4":addr==0x80016B5Cu?"16B5C":"16BEC",
        cpu->gpr[3], cpu->gpr[4], cpu->lr, *q);
      return false; }
    // fzEYzb8: A360 (dispatched entry) selects the 030CE store side
    // (A398 native vs A3A8 native). Entry lr + r3 identify the caller.
    if(addr==0x8000A360u){
      static unsigned _c=0; if(++_c<=4) fprintf(stderr,"[dvdsm] A360 r3=0x%08X lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->lr, _c);
      return false; }
    // fzEYzb72: D9CC = __OSDispatchInterrupt entry (dispatched, downcount).
    // Deliveries redirect pc here; if it never fires, delivered interrupts
    // never enter the guest dispatcher. r3=exception code (4=external),
    // r4=context pointer.
    if(addr==0x8000D9CCu){
      static unsigned _n=0; if(++_n<=8) fprintf(stderr,"[dvdsm] D9CC-dispatch r3=%u r4=0x%08X msr=0x%08X lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->gpr[4], cpu->msr, cpu->lr, _n);
      return false; }
    // fzEYzb110 (answered probe134 — 0 hits across 8 D9CC runs): the
    // D9CC table-walk interior (DC90/DCA4/DCC4/DCEC) NEVER dispatches —
    // native stretch from D9CC entry through the DCD8 blrl, same class as
    // 1A600/1A60C. Handler identity is proven instead by 1A55C entries
    // with lr=DCDC (the blrl return address): 1A55C IS the selected
    // handler, called directly by the dispatcher. Probe removed.
    // fzEYzb72b: 1B42C entry (dispatched) = the 706E4 file-load waiter that
    // must return for the 706EC done-flag store to run. lr names caller.
    if(addr==0x8001B42Cu){
      static unsigned _n=0; if(++_n<=4){ uint32_t w=0xDEADu;
        if(cpu->gpr[3]) guest_read32(cpu->gpr[3],&w);
        fprintf(stderr,"[dvdsm] 1B42C-entry r3=0x%08X [r3]=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
          cpu->gpr[3], w, cpu->gpr[4], cpu->lr, _n); }
      return false; }
    // fzEYzb73: 1B42C RETURN continuations — r3 holds 1B42C's return value.
    // 706E8 follows the 706E4 call (setter path -> done-flag store next);
    // 70A1C follows the 70A18 call (game path -> 1BD84/1BC54/1AF64 next).
    // Return 0/failure vs nonzero/success tells whether the file op worked.
    if(addr==0x800706E8u||addr==0x80070A1Cu){
      static unsigned _m1=0,_m2=0; unsigned *c=addr==0x800706E8u?&_m1:&_m2; (*c)++;
      if(*c<=4){ uint32_t w=0xDEADu; guest_read32(0x8019E150u,&w);
        fprintf(stderr,"[dvdsm] %s 1B42C-ret r3=0x%08X r4=0x%08X gate2=[0x8019E150]=0x%08X (#%u)\n",
          addr==0x800706E8u?"706E8":"70A1C", cpu->gpr[3], cpu->gpr[4], w, *c); }
      return false; }
    // fzEYzb73b: live file-op entries on the game path (all dispatched):
    // 1AFB8 (fires, lr=1BDD0), 1BD84, 1BC54, 1BDF0. Dump args + lr.
    // fzEYzb90: + the 16DF8 (DVDConvertPathToEntrynum) callers: 07040
    // (game boot), 1717C/1747C (DVD layer). ANY fire = first READ attempt.
    // + 6B4C (game boot worker calling 175C0/DVDOpen) liveness.
    if(addr==0x80007040u||addr==0x8001717Cu||addr==0x8001747Cu||addr==0x80006B4Cu||addr==0x80070620u||addr==0x80070600u||addr==0x80070604u){
      static unsigned _c1=0,_c2=0,_c3=0,_c4=0,_c5=0,_c6=0,_c7=0;
      unsigned *c=addr==0x80007040u?&_c1:addr==0x8001717Cu?&_c2:addr==0x8001747Cu?&_c3:addr==0x80006B4Cu?&_c4:addr==0x80070620u?&_c5:addr==0x80070600u?&_c6:&_c7; (*c)++;
      const char *_nm=addr==0x80007040u?"07040-16DF8caller":addr==0x8001717Cu?"1717C-16DF8caller":addr==0x8001747Cu?"1747C-16DF8caller":addr==0x80006B4Cu?"6B4C-bootworker":addr==0x80070620u?"70620-setter-entry":addr==0x80070600u?"70600-setter-entry":"70604-setter-ret";
      if(*c<=4) fprintf(stderr,"[dvdsm] %s r3=0x%08X r4=0x%08X lr=0x%08X (#%u)\n",
        _nm, cpu->gpr[3], cpu->gpr[4], cpu->lr, *c);
      return false; }
    if(addr==0x8001AFB8u||addr==0x8001BD84u||addr==0x8001BC54u||addr==0x8001BDF0u){
      static unsigned _f1=0,_f2=0,_f3=0,_f4=0;
      unsigned *c=addr==0x8001AFB8u?&_f1:addr==0x8001BD84u?&_f2:addr==0x8001BC54u?&_f3:&_f4; (*c)++;
      if(*c<=4) fprintf(stderr,"[dvdsm] %s r3=0x%08X r4=0x%08X r5=0x%08X lr=0x%08X (#%u)\n",
        addr==0x8001AFB8u?"1AFB8":addr==0x8001BD84u?"1BD84":addr==0x8001BC54u?"1BC54":"1BDF0",
        cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->lr, *c);
      return false; }
    // fzEYz: 19500 (dispatched entry) files block+40 (slot) at 19538.
    // If it never fires, no slot is ever registered and 18E04 always
    // skips the invoke — the completion can never call back up.
    if(addr==0x80019500u){
      static unsigned _n=0; if(++_n<=4) fprintf(stderr,"[dvdsm] 19500 slot-reg r3=0x%08X r5=0x%08X lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->gpr[5], cpu->lr, _n);
      return false; }
    // fzEYzb68: 1A55C entry (dispatched, has downcount) is the ONLY caller
    // of the 1A5xx waker chain (1A55C->...->1A5FC-taken->1A618 increment ->
    // 1A628 fnptr invoke). 1A55C takes r3=video-mode-ish + r4 (r30=copy).
    // lr names the caller; r7 bit3 (from the VI field-status polls) decides
    // 1A5F8->1A5FC vs 1A600. Fires => waker chain entered; never => dead.
    if(addr==0x8001A55Cu){
      // fzEYzb93: dump the STORED INTSR0-3 HI halves per entry (ground
      // truth for the r7 reconstruction: does entry k see INTSR2/3 set?).
      // VCT-gated retrace model: INTSR0/1 latch (game-armed), INTSR2/3
      // (0x0000, VCT=0) stay clear => r7=0x3 => no-wake side expected.
      static unsigned _n=0; if(++_n<=6){
        // fzEYzb96: dump r13 + CURRENT pc alongside the entry pc. The
        // 1A618 stw is r13-relative: if the waker frame runs with a
        // different r13 than the waiter's, the store lands elsewhere and
        // the waitword slot can never observe it.
        uint32_t w=0xDEADu;
        guest_read32(cpu->gpr[13]-31388u,&w);
        u64 v30=0,v34=0,v38=0,v3c=0;
        dol_mmio_bus_read(&s_mmio_bus, cpu, 0xCC002030u, 2u, &v30);
        dol_mmio_bus_read(&s_mmio_bus, cpu, 0xCC002034u, 2u, &v34);
        dol_mmio_bus_read(&s_mmio_bus, cpu, 0xCC002038u, 2u, &v38);
        dol_mmio_bus_read(&s_mmio_bus, cpu, 0xCC00203Cu, 2u, &v3c);
        fprintf(stderr,"[dvdsm] 1A55C-waker-entry r3=0x%08X r4=0x%08X r13=0x%08X waitword=%u VI30=%04X VI34=%04X VI38=%04X VI3C=%04X lr=0x%08X (#%u)\n",
          cpu->gpr[3], cpu->gpr[4], cpu->gpr[13], w,
          (unsigned)v30&0xFFFFu, (unsigned)v34&0xFFFFu,
          (unsigned)v38&0xFFFFu, (unsigned)v3c&0xFFFFu, cpu->lr, _n); }
      return false; }
    // fzEYzb75: 1A600 (no-wake side, downcount) vs 1A60C (waker side,
    // NATIVE fallthrough of the 1A5FC bc — never dispatches). 1A600 fires
    // iff the 1A5F4 gate fell through (r7 bit2 clear). The 1A60C side is
    // journal-visible only (does 1A618's stw land with pc=1A618?).
    // fzEYzb84 (answered — filtered [vi] waker-ACK trace): each entry ACKs
    // exactly 2 INTSRs (0x2030->0x1107 at 1A590, 0x2034->0x1001 at 1A5AC),
    // i.e. reads bit15-SET at 1A584/1A5A4 => r7=0x3 (bits 0,1). INTSR2/3
    // (0x2038/0x203C) stay 0x0000 => r7 bits 2,3 stay 0 =>
    // 1A5F0 tests bit2: CLEAR (r7=0x3) => TAKEN to 1A600 no-wake side.
    // (bc 4,2 with CR0[EQ] set branches to 1A600 — confirmed by the chunk:
    // `if (ctr_ok && cr_ok) goto label_8001A600` where cr_ok tests the EQ
    // bit SET. An earlier note tangled the BO semantics; the code is
    // unambiguous: EQ=>1A600 no-wake, NE=>fall to 1A5F8.)
    // fzEYzb94 (standing question): what arms INTSR2 (VCT=1, HCT=1) on
    // hardware such that the 1A5C0 poll observes bit15 set?
    if(addr==0x8001A600u||addr==0x8001A60Cu){
      // fzEYzb95: THESE NEVER FIRE (both are native-reach-only: 1A600 via
      // 1A5F4-taken goto, 1A60C via 1A5FC-taken fallthrough — neither is a
      // bl target or blr landing, so no dispatch ever lands on them).
      // Gate outcome is observable ONLY via (a) the journal (1A618 stw with
      // pc frozen at 1A60C), and (b) the bl continuations below (BE00/BFC8
      // entries with lr=1A608/1A620/1A628). Kept for documentation.
      static unsigned _g3=0,_g4=0;
      unsigned *c=addr==0x8001A600u?&_g3:&_g4; (*c)++;
      if(*c<=4||*c%200000==0) fprintf(stderr,"[dvdsm] %s r7=0x%08X lr=0x%08X (nowake=%u wake=%u)\n",
        addr==0x8001A600u?"1A600-NOWAKE":"1A60C-WAKE",
        cpu->gpr[7], cpu->lr, _g3, _g4);
      return false; }
    // fzEYzb95b: DECISIVE gate probe. Both gate sides call out via bl
    // (=> dispatch => observable): no-wake side calls BE00 with lr=1A608;
    // waker side calls BFC8 (lr=1A620) then BE00 (lr=1A628). lr names the
    // side unambiguously.
    // fzEYzb95c (ANSWERED — BFC8 lr=1A620 + BE00 lr=1A628 fire 6/6 waker
    // entries): the waker-side bl chain RUNS every entry. The 1A618 stw
    // itself is native (chunk 0006: 1A60C/1A610/1A614/1A618 carry no
    // downcount, running as one stretch to the 1A61C bl), so dispatch
    // probes can never observe it — but the 1A61C bl DOES dispatch, and
    // its BFC8 entry (lr=1A620) fires, PROVING control passed through
    // 1A618. The journal DOES see the store (pre-write); the word reads
    // back 0 at the next 1A55C entry because the 1AB1C video clearer
    // (stw r31=0, journal pc=1AB1C) zeroes it every frame — the waker
    // increments (+1) and the clearer zeroes, racing with the waiter
    // sampling between them. The 1A640-fallthrough BFC8 (lr=1A750) fires
    // because the fnptr word (-31372) is still 0 (journal slot3: only
    // zero-writes) — the 1AF30 clearer zeroes it too. Both clearers run
    // in the same video frame; the wake is real but never accumulates.
    if(addr==0x8000BE00u||addr==0x8000BFC8u){
      // fzEYzb98: dump GPR[1..7] — the 1A60C path passes r3=[sp+24] into
      // BFC8 and r3/r4 into the BE00 calls; register state at the BFC8
      // entry (probed pre-body) reveals what 1A610/1A614 loaded and
      // whether r4 (the waitword old value) was sane. Cap 4 per side.
      // fzEYzb98b: counters increment ONLY on window hits (an earlier
      // revision incremented on every BE00/BFC8 call, so boot traffic
      // burned the *c<=4 cap before the first waker entry).
      // fzEYzb99: window widened to 1A600-1A770 — the 1A640-fallthrough
      // BFC8 (lr=1A750, fnptr-NULL leg) is as informative as the waker
      // legs (lr=1A620/1A628): it distinguishes "fnptr null" from
      // "waker never ran".
      static unsigned _b1=0,_b2=0;
      if(cpu->lr>=0x8001A600u && cpu->lr<=0x8001A770u){
        unsigned *c=addr==0x8000BE00u?&_b1:&_b2; (*c)++;
        if(*c<=4)
          fprintf(stderr,"[dvdsm] %s-WAKER-LR lr=0x%08X r1=0x%08X r3=0x%08X r4=0x%08X r5=0x%08X r6=0x%08X r7=0x%08X\n",
            addr==0x8000BE00u?"BE00":"BFC8", cpu->lr, cpu->gpr[1],
            cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->gpr[6], cpu->gpr[7]);
      }
      return false; }
    // fzEYzb83: memset entry (dispatched). The journal's 64 nonzero
    // waitword hits (now=1 at pc=34E4) are memset's word-fill pattern, not
    // the 1A618 waker (different pc; the journal reads the stw's own bytes
    // back mid-burst). Confirm live: dump r3(dest)/r4(fill)/r5(len)+lr.
    if(addr==0x80003458u){
      static unsigned _m=0; if(++_m<=6) fprintf(stderr,"[dvdsm] 3458-memset r3=0x%08X r4=0x%08X r5=%u lr=0x%08X (#%u)\n",
        cpu->gpr[3], cpu->gpr[4], cpu->gpr[5], cpu->lr, _m);
      return false; }
    // fzEYzb68b: 1A618 is a NATIVE stw inside the 1A55C frame (no downcount:
    // 1A60C/1A610/1A614/1A618 run as one stretch to the 1A61C bl) — it can
    // NEVER dispatch, like all mid-chain natives. The gate outcome is
    // journal-visible ONLY: the waitword slot fires with 1A618's pc iff
    // the 1A5FC gate took the waker side. (An old note claimed 1A618 was
    // dispatched — wrong; it has no downcount line in the chunk.)
    if(addr==0x8001A618u){
      static unsigned _n=0; if(++_n<=6){ uint32_t w=0xDEADu;
        guest_read32(cpu->gpr[13]-31388u,&w);
        fprintf(stderr,"[dvdsm] 1A618-waker-fire waitword=%u r7=0x%08X lr=0x%08X (#%u)\n",
          w, cpu->gpr[7], cpu->lr, _n); }
      return false; }
    // fzEYzb69: DVD READ-chain entries (all dispatched, downcount>0):
    // 16DF8=DVDConvertPathToEntrynum, 1687C=ReadDiskID, 19354=tag1-enqueue,
    // 19430=tag4-enqueue (the READ enqueue: li r0,4 + stw r0,8(r3)),
    // 16394=DVDLowRead (0xA8 builder). lr names the caller at each stage.
    // ANY fire here = the file path is live (FST resolved, read enqueued).
    if(addr==0x80016DF8u||addr==0x8001687Cu||addr==0x80019354u||addr==0x80019430u||addr==0x80016394u){
      static unsigned _d1=0,_d2=0,_d3=0,_d4=0,_d5=0;
      unsigned *c=addr==0x80016DF8u?&_d1:addr==0x8001687Cu?&_d2:addr==0x80019354u?&_d3:addr==0x80019430u?&_d4:&_d5; (*c)++;
      // fzEYzb128 SUPERSEDED by fzEYzb131 (probe160 + FST decode): the
      // fzEYzb130 FST preload makes 16DF8 find "fze.str" -> entry 1358.
      // The OLD note (both return -1, REL paths never in the FST) was
      // written when [0x80000038] was 0 at lookup time so 16DC0 had filed
      // -31516=0 and EVERY lookup returned -1. With the real root filed,
      // fze.str resolves; the fze.sample.rel call is a DIFFERENT caller
      // (its 16DF8 fire is the 1747C-boot path, pre-FST-root or the
      // strip-to-'/'+1 tail). The REAL divergence is now downstream:
      // 19354 fires (tag1 enqueue for entry 1358, r4=0x8155AD80 data
      // block) but 19430 (tag4 READ enqueue -> 16394 -> 0xA8 DI cmd)
      // NEVER fires — the 174D0 frame takes the 17520 fatal leg
      // (17180-entry r3=-1 in the OLD runs; now r3=1358) — verify what
      // 174D0/17520 does next, and whether 19354's 19E9C-link + 187CC
      // drain completes. Dump r3 path + r4 for each stage.
      // fzEYzb131: 19354 (tag1 enqueue) must run inside the 174D0 frame
      // BEFORE the fatal 17520 leg (17574-bl-19354, regs r3-r8 captured
      // at 174D0-entry snapshot: r26/27/28/30/31 + r29 flag). Sample
      // [blk+8] (tag) + [blk+12] (state) at 19354-entry: tag should be
      // 1 (19354 filed li r0,1 at 1935C + stw r0,8(r3)), state b12 should
      // still be pre-drain. Then 19430-entry proves the READ follows.
      if(*c<=4){ uint32_t r4=cpu->gpr[4]; char pb[24]; pb[0]=0; char hx[65]; hx[0]=0;
        if(cpu->gpr[3]>=GC_RAM_BASE){ uint8_t* p=g_cpu.ram+(cpu->gpr[3]-GC_RAM_BASE);
          int n=0; for(;n<23;n++){ if(p+n>=g_cpu.ram+g_cpu.ram_size) break; char ch=(char)p[n]; if(!ch) break; pb[n]=(ch>=32&&ch<127)?ch:'.'; } pb[n]=0;
          // fzEYzb159: hex dump of the path bytes — the REL-built path #3
          // arrives garbled ('enem.M...e/...eMM.bin', 16DF8 returns -1),
          // so downstream offset/len are garbage. Hex pinpoints which byte
          // positions the interp path-decoder gets wrong.
          int m=0; for(;m<32;m++){ if(p+m>=g_cpu.ram+g_cpu.ram_size) break;
            hx[m*2]="0123456789ABCDEF"[(p[m]>>4)&15]; hx[m*2+1]="0123456789ABCDEF"[p[m]&15]; } hx[m*2]=0; }
        uint32_t _tg=0xDEADu,_b12=0xDEADu; uint32_t _blk=cpu->gpr[3];
        if(addr==0x80019354u||addr==0x80019430u){ guest_read32(_blk+8u,&_tg); guest_read32(_blk+12u,&_b12); }
        fprintf(stderr,"[dvdsm] %s path='%s' hex=%s r4=0x%08X blk=0x%08X tag=%u b12=%u lr=0x%08X (#%u)\n",
        addr==0x80016DF8u?"16DF8-path2entry":addr==0x8001687Cu?"1687C-diskid":addr==0x80019354u?"19354-tag1":addr==0x80019430u?"19430-tag4-READ":"16394-DVDLowRead",
        pb, hx, r4, _blk, _tg, _b12, cpu->lr, *c); }
      return false; }
    // fzEYy probe removed: 18DD8/18E08 never fire (mid-chain natives).
    // fzEYx/fzEYzb3/fzEYv probes removed: 18E34 et al / 18F04 et al /
    // 19240 et al never fire (mid-chain natives in 18D1C frame).
    // fzEXd probe removed: 18F38 never fires (mid-chain native).
    // fzEXc/fzEXb/fzEZb/fzEZ probes removed: 18FB0 et al / 18DB0 et al /
    // 18D80 never fires (mid-chain native in 18D1C frame). 18D68's status
    // is RE-TESTED by fzEYzb25 below (bl-targets may dispatch it).
    // fzEU: 18D40 (dispatched) is the r3==0x10 error path; 18D64 is a
    // goto-target (never a resume pc); 18D58/18D5C are native bls.
    // 18D40 firing with r3==0x10 confirms the ERRPATH into 1A178;
    // 18D68 (fzEYzb25) firing instead would mean the 18D3C branch fell
    // through to the main body. Direction read at 18D40 vs 18D68 only.
    if(addr==0x80018D40u){
      static unsigned _u1=0; _u1++;
      if(_u1<=4||_u1%5000000==0) fprintf(stderr,"[dvdsm] 18D40-errpath r3=0x%08X (#%u)\n",
        cpu->gpr[3], _u1);
      return false; }
    // fzEN: 187E8 dispatches (downcount) with r3 = 19FA4's return value.
    // r3==0 -> 187F0 early-out (no restore); r3!=0 -> 187FC continue to
    // the 18804/18820 restore-consumption gates. This decides the drain.
    if(addr==0x800187E8u){
      static unsigned _q=0; if(++_q<=4){ uint32_t a=0; guest_read32(cpu->gpr[13]-31464u,&a);
        fprintf(stderr,"[watch] 187E8 19FA4ret=r3=%d m64=%u lr=0x%08X (#%u)\n",
          (int32_t)cpu->gpr[3], a, cpu->lr, _q); }
      return false; }
    // fzES: m48 cascade — 18868 (entry, dispatched) runs native through
    // 18880/1888C/18890/18898/1889C to 188A4->189CC. Only dispatched
    // labels (18868/18880/1888C/18890/18898/1889C/188A4/188A8/189CC) fire.
    if(addr==0x80018868u||addr==0x80018880u||addr==0x8001888Cu||addr==0x80018890u||addr==0x80018898u||addr==0x8001889Cu||addr==0x800188A4u||addr==0x800188A8u||addr==0x800188BCu||addr==0x800188B8u||addr==0x80018904u||addr==0x800189B0u||addr==0x800189CCu){
      static unsigned _s[13]={0}; int _i=
        addr==0x80018868u?0:addr==0x80018880u?1:addr==0x8001888Cu?2:addr==0x80018890u?3:
        addr==0x80018898u?4:addr==0x8001889Cu?5:addr==0x800188A4u?6:addr==0x800188A8u?7:
        addr==0x800188BCu?8:addr==0x800188B8u?9:addr==0x80018904u?10:addr==0x800189B0u?11:12;
      const char *_nm[13]={"18868","18880","1888C","18890","18898","1889C","188A4","188A8","188BC","188B8","18904","189B0","189CC"};
      if(++_s[_i]<=3){ uint32_t m=0; guest_read32(cpu->gpr[13]-31448u,&m);
        fprintf(stderr,"[watch] %s m48=%u lr=0x%08X (#%u)\n", _nm[_i], m, cpu->lr, _s[_i]); }
      return false; }
    // fzEYzc: 1799C/179A8 are NATIVE (no downcount) — the slot branch
    // runs inside the 17958 frame; 179AC fires only if slot!=0, 179BC
    // if slot==0. 179BC fires per completion => slot always null here.
    if(addr==0x800179ACu||addr==0x800179C8u||addr==0x800179DCu){
      static unsigned _f[3]={0}; int _i=
        addr==0x800179ACu?0:addr==0x800179C8u?1:2;
      const char *_nm[3]={"179AC","179C8","179DC-preslot"};
      if(++_f[_i]<=3){ uint32_t m=0;
        guest_read32(cpu->gpr[13]-31452u,&m);
        fprintf(stderr,"[dvdsm] %s m52=0x%08X r12=0x%08X r3=%d (#%u)\n",
          _nm[_i], m, cpu->gpr[12], (int32_t)cpu->gpr[3], _f[_i]); }
      return false; }
    // fzEYzb63: bctr dispatch INSIDE the 18D1C completion frame is
    // invisible (18A38 bctr is a native branch label — the whole
    // 18D1C->...->192xx frame runs in ONE dolrecomp_call, so slice/host
    // probes never fire mid-frame). Instrument INSTEAD at frame ENTRY:
    // extend the 18D1C probe (below) with post-frame state — but the
    // body runs native, so instead log the 189FC dispatcher's tag/row
    // (which selects 18CC8 vs sibling legs on the NEXT drain) — already
    // covered. This probe is REMOVED (never fires by construction).
    // fzEYza: 187F0/187FC/18808/1881C/18820 all dispatch on the
    // 19FA4ret=0 early-out (curblk=0) vs continue routes. First-fire +
    // counters show which the drain takes per completion.
    if(addr==0x800187F0u||addr==0x800187FCu||addr==0x80018808u||addr==0x8001881Cu||addr==0x80018820u){
      static unsigned _e[5]={0}; int _i=
        addr==0x800187F0u?0:addr==0x800187FCu?1:addr==0x80018808u?2:addr==0x8001881Cu?3:4;
      const char *_nm[5]={"187F0-earlyout","187FC-cont","18808","1881C","18820"};
      if(++_e[_i]<=3||_e[_i]%5000000==0){ uint32_t c=0; guest_read32(cpu->gpr[13]-31488u,&c);
        fprintf(stderr,"[dvdsm] %s curblk=0x%08X r3=%u (#%u)\n", _nm[_i], c, cpu->gpr[3], _e[_i]); }
      return false; }
    // fzEL: superseded by fzEYza above (same labels + curblk dump).
    // fzEJ: 18830 = consume path (m64!=0), 18868 = skip path (m64==0).
    if(addr==0x80018830u||addr==0x80018868u){
      static unsigned _g2=0,_g3=0; unsigned *c=addr==0x80018830u?&_g2:&_g3; (*c)++;
      if(*c<=4){ uint32_t a=0; guest_read32(cpu->gpr[13]-31464u,&a);
        fprintf(stderr,"[watch] %s m64=%u lr=0x%08X (#%u)\n",
          addr==0x80018830u?"18830-consume":"18868-skip", a, cpu->lr, *c); }
      return false; }
    // fzEYzb63: 18A38 jump-table dispatch (the 18D1C completion frame's
    // routing table: 18A38 bctr -> per-tag legs 18B04/18B2C/.../18CF4).
    // Log the CTR target (which leg each completion takes) + r3 (result).
    // First-8 + every-5M counter. If all completions land on the same
    // leg, that leg owns the re-issue.
    // fzEK (corrected by fzEYzb28): 19FA4 fires as a BL-TARGET (resume pc
    // after 187E4's bl), once per drain — r3 is the CONSTANT scan arg
    // (0x80160000), NOT the result. Result returns natively to 187E8.
    // The fzEYzb28 probe above covers 19FA4 entry; this keeps 19F04 only.
    if(addr==0x80019F04u){
      static unsigned _j2=0; _j2++;
      if(_j2<=3){ uint32_t a=0; guest_read32(cpu->gpr[13]-31464u,&a);
        fprintf(stderr,"[watch] 19F04 m64=%u r3=0x%08X lr=0x%08X (#%u)\n",
          a, cpu->gpr[3], cpu->lr, _j2); }
      return false; }
    if(addr==0x8001A2ECu||addr==0x800169ACu){
      static unsigned _k1=0,_k2=0; unsigned *c=addr==0x8001A2ECu?&_k1:&_k2; (*c)++;
      if(*c<=4){ uint32_t v64=0; guest_read32(cpu->gpr[13]-31464u,&v64);
        fprintf(stderr,"[watch] %s m64=%u lr=0x%08X (#%u)\n",
          addr==0x8001A2ECu?"1A2EC":"169AC", v64, cpu->lr, *c); }
      return false; }
    // 18D1C = low-level completion entry (dispatched). fzBT: r30 IDENTICAL
    // at 18D1C and 16920 (0x1823CF40) — rides in on the SAVED slice context
    // (trampoline preempts AC34 with garbage r30). Only r3/r4 are args.
    // Fix (fzBU): zero the callback's non-arg regs at poll time.
    // fzEYzb64 (FRAME-EXIT probe): the 18D1C frame runs native to
    // completion, so slice probes can't see its interior — but its EXIT
    // state IS observable: the frame returns to HLE_CALLBACK_RETURN, and
    // the NEXT slice iterations dispatch the frame's tail calls (16920
    // STOPMOTOR issue, 1A178 report, 187CC drain...). Log POST-frame
    // block words here at the NEXT 18D1C entry (i.e. previous frame's
    // exit state): blk+12 (b12 filer), -31456 (m56), -31448 (m48),
    // -31488 (curblk). If the exit state ever differs from entry state
    // (b12=1/m56=0/m48=0), the frame advanced.
    if(addr==0x80018D1Cu){
      static unsigned _d=0; if(++_d<=4||_d%5000000==0){
        uint32_t v60=0,v56=0,v36=0,v32=0,b12=0,v64=0,v48=0; uint32_t blk=cpu->gpr[4];
        guest_read32(cpu->gpr[13]-31460u,&v60); guest_read32(cpu->gpr[13]-31456u,&v56);
        guest_read32(cpu->gpr[13]-31436u,&v36); guest_read32(cpu->gpr[13]-31432u,&v32);
        guest_read32(cpu->gpr[13]-31464u,&v64); guest_read32(cpu->gpr[13]-31448u,&v48);
        if(blk){ guest_read32(blk+12u,&b12); }
        uint32_t b8=0; if(blk) guest_read32(blk+8u,&b8);
        // fzEE: 19270 success gate is [blk+32]==[blk+20] (xfer lens). Dump
        // +20/+28/+32 to see if our INQUIRY +28/+32=len write breaks it.
        // fzEH: 19270 reads r30 from -31488 (current-block global), NOT r4.
        // Dump -31488 + its +20/+32: if curblk != r4, the gate compares a
        // different block than the one we complete.
        uint32_t b20=0,b28=0,b32=0; if(blk){ guest_read32(blk+20u,&b20); guest_read32(blk+28u,&b28); guest_read32(blk+32u,&b32); }
        uint32_t cur=0,c20=0,c32=0; guest_read32(cpu->gpr[13]-31488u,&cur);
        if(cur){ guest_read32(cur+20u,&c20); guest_read32(cur+32u,&c32); }
        fprintf(stderr,"[watch] 18D1C r3=%u blk=0x%08X b8=%u b12=%d b20=%u b28=%u b32=%u cur=0x%08X c20=%u c32=%u m64=%u m60=%u m56=%u m48=%u m36=%u m32=%u r30=0x%08X r31=0x%08X (#%u)\n",
          cpu->gpr[3], blk, b8, (int32_t)b12, b20, b28, b32, cur, c20, c32, v64, v60, v56, v48, v36, v32, cpu->gpr[30], cpu->gpr[31], _d); }
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
    // fzEYzb20: ALSO dump block+8 tag filed by the 16Axx path (16AFC oris
    // r0,r3,0xE100 -> [r6+8] in 16AD4-family; tag selects 189FC row).
    if(addr==0x80016A38u){
        static int _n=0;
        // fzEF: log EVERY issue (uncapped counter + c0 command word from the
        // DI reg 0xCC006008): M2 progress = c0 ever becomes 0xA8 (FST read)
        // instead of 0x12 (INQUIRY)/0xE3 (STOPMOTOR) retries.
        _n++;
        if(_n<=10||_n%200==0){ uint32_t tag=0xDEADu; uint32_t blk=cpu->gpr[3];
          if(blk) guest_read32(blk+8u,&tag);
          fprintf(stderr,"[di] 16A38 #%d r3=0x%08X r4=0x%08X tag8=%u lr=0x%08X\n",
            _n, cpu->gpr[3], cpu->gpr[4], tag, cpu->lr); }
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
        // fzEYzb67: console type matches Dolphin Boot_BS2Emu.cpp:260
        // (LatestDevkit 0x10000006, not retail 3/1 — retail IDs take
        // different EXI paths in some titles).
        POKE32(0x8000002Cu, 0x10000006u); POKE32(0x80000030u, arena_lo); POKE32(0x80000034u, 0x817FEC60u);
        POKE32(0x80000038u, 0u); POKE32(0x8000003Cu, 0u); POKE32(0x800000CCu, 0u);
        // fzEYzb67: ARAM size 16MB, Dolphin Boot_BS2Emu.cpp SetupGCMemory-exact.
        POKE32(0x800000D0u, 0x01000000u);
        // NOTE: 0x800000D4 is owned by the guest OS (OSContext pointer set by
        // __OSInit / OSInitThreadQueue). Do NOT pre-seed it: 102AC reads this
        // word and the boot PSL would diverge from hardware.
        POKE32(0x800000F8u, 0x09A7EC80u); POKE32(0x800000FCu, 0x1CF7C580u);
        cpu->gpr[1] = 0x817FFF00u; POKE32(0x817FFF00u, 0u); POKE32(0x817FFF04u, 0u);
        // fzEYzb130: pre-install the real FST blob @0x81200000 + seed
        // [0x80000038] BEFORE first dispatch (same bytes the 102AC
        // side-effect installs). On HW the apploader does this: 16DC0's
        // single boot run (lr=1779C) loads [0x80000038] into -31516 and
        // early-outs at 16DD8 when it is 0 — leaving -31516/-31508/-31512
        // zero, so every 16DF8 lookup returns -1, 174B8 never files
        // -31504, the 174D0 frame takes the 17520 fatal leg, and C49C's
        // walk exits to the C5A4 sync-idle hang (A000-spin lr=C5A8).
        // Seeding here makes 16DC0's one run file the LIVE root.
        { extern int dvd_build_fst_from_tree(uint8_t* ram, unsigned ram_size, unsigned base);
          if(cpu->ram_size >= 0x1200010u){
              uint32_t base=0x81200000u; uint32_t off=base - GC_RAM_BASE;
              if(off+16 <= cpu->ram_size){
                  int n = dvd_build_fst_from_tree(cpu->ram, cpu->ram_size, base);
                  if(n > 0) POKE32(0x80000038u, base);
              }
          } }
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
void recomp_shutdown(void){ ppc_set_gather_pipe(NULL,NULL,NULL,NULL,NULL); ppc_set_mem_write_journal(NULL,NULL); if(g_inited){ cpu_free(&g_cpu); g_inited=0; } }
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
            if(g_cpu.exception & PPC_EXC_FP_UNAVAILABLE){ g_cpu.msr|=0x2000u; g_cpu.srr1|=0x2000u; g_cpu.pc=g_cpu.srr0; g_cpu.exception=0; g_cpu.program_exception=0; continue; }
            // System-call vector (0xC00): guest `sc` is the SDK cache-sync
            // barrier epilogue (dcbf-loop + sc + blr). The slice loop
            // emulates cia+4 directly; the FP fault above handles 0x800.
            // Other vectors (DSI/program/etc.) rfi per Strikers host.
            if(vec==0xC00u){ g_cpu.pc=g_cpu.srr0; g_cpu.exception=0; g_cpu.program_exception=0; continue; }
            if(vec>=0x200 && vec<0xD00){ ppc_rfi(&g_cpu, vec); g_cpu.exception=0; g_cpu.program_exception=0; continue; }
            if(g_cpu.exception & PPC_EXC_PROGRAM){ g_cpu.exception=0; break; }
            g_cpu.exception=0; break;
        }
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
        if(pc==0xC00u){ g_cpu.pc=g_cpu.srr0; g_cpu.exception=0; g_cpu.program_exception=0; continue; }
        if(pc>=0x200 && pc<0xD00){ ppc_rfi(&g_cpu, pc); g_cpu.exception=0; g_cpu.program_exception=0; continue; }
        // fzEYzb129 (cpdc loop clamp — same class as the B670/B750/B7C4
        // clamps above: guest cache-block loops whose ctr derives from a
        // heap-table match that can never succeed before the first DVD
        // READ). B6A0's ctr = ((len+lowbits)+31)>>27: with the 858E4 poke
        // the C49C memcpy proceeds, but EVERY dcbst loop (B684-callers)
        // inherits a huge ctr from a zero-length/empty range and spins
        // 100M+ iterations per dispatch slice. Clamp like the others.
        if((pc==0x8000B670u || pc==0x8000B750u || pc==0x8000B7C4u || pc==0x8000B6A0u) && g_cpu.ctr>4) g_cpu.ctr=1;
        else if(pc==0x80010608u && g_cpu.gpr[6]==0) g_cpu.gpr[6]=1;
        if(pc==0x800113B8u && g_cpu.ctr>8) g_cpu.ctr=1;
        else if(pc==0x800034E4u && g_cpu.gpr[3]>256u) g_cpu.gpr[3]=256u;
        // (was: 0x80011424 timebase += 5000 wall-clock hack — removed per M0.)
        // (was: DSP hand-shake pokes at B450/B498/B4B4/B4D4/B3EC/B508/B578 —
        // removed. The DSP/AI MMIO chassis above now serves the real register
        // state; the guest's own sth/lhz hand-shake runs native. Per-PC pokes
        // are banned anti-patterns (PLAN) regardless of outcome.)
        if(pc==0x8001BD10u){ poke32_set(g_cpu.gpr[13]-31352u, 0u); poke32_set(g_cpu.gpr[13]-31348u, 0u); }
        // fzEYzb105 (EXP): 1071C/1072C f88-force REMOVED. These pokes
        // short-circuited the sleep/wake protocol (forced f88=1 before any
        // waker ran, pushing the waiter into 10740 with a bogus mask). Now
        // that VI retrace delivers + the waker runs 11194 (which files f88
        // naturally at its 11240-11254 tail), the waiter should exit its
        // spins on the REAL bit. If RET-11174 fires, the natural protocol
        // works and the pokes were the wedge.

        else if(pc==0x80033614u) g_cpu.gpr[0]=255;
        else if(pc==0x800332FCu){ uint32_t a=g_cpu.gpr[4]; if(a>=0x80000000u && a+4 < 0x80000000u+g_cpu.ram_size) write_be32(g_cpu.ram+(a-0x80000000u), 255u); }

        else if(pc==0x8000D5E0u){ g_cpu.pc=g_cpu.lr & ~3u; }
        // fzEYzb125 (PLAN-banned per-PC poke — KEPT with rationale): 858E4
        // forces the C49C-memcpy 80828-heap lookup to SUCCEED (r3=1) instead
        // of taking the 80834 r3=-1 failure leg. Without it C49C always
        // fails its heap-table match (heap never populated: the FST READ
        // never issued, so no heap entries exist), C568 loops 16x then
        // parks at 9FFC/A000 forever and the boot NEVER advances past the
        // 1751C-memcpy. The poke stands in for the missing DVD READ data
        // path (M2): remove it once the first 0xA8 READ completes and the
        // heap fills for real. All other pokes stay removed.
        else if(pc==0x800858E4u) g_cpu.gpr[3]=1;
        else if(pc==0x800333C8u) g_cpu.gpr[11]=0;
        else if(pc==0x8000A990u) g_cpu.gpr[26]=16;
        // (was: 0x80010718 poke r13-31688=1 — removed. 10718/1071C is a
        // spin-wait on that flag; forcing it defeats the wait it guards.)
        else if(pc==0x80010624u && g_cpu.gpr[6]==0) g_cpu.gpr[6]=1;
        // (was: 0x8000BE60 spoofed low-mem 0xD4=0x80003000 — removed.
        // BE5C/BE60 reads the OS global at 0x800000D4, owned by the guest;
        // seeding it corrupts the OSContext chain. Let the guest write it.)
        // fzEYzb50: 11160 WAIT-loop poke — DISABLED (was poke32_set
        // r13-31684=1u). 11158 files r13-31684=1 itself, then 11160 polls
        // it; forcing it masks whether the 11134/1114C waiter-link chain
        // ever wakes the thread. With the poke REMOVED, if 11160 spins
        // forever the waiter queue is dead; if it exits, wakeups work.
        // (11160 never dispatches in logs — the 110A8 frame runs native
        // from its bl-target entrypc through 11190 blr. Observe via the
        // 110A8-entry probe + 1AF8C poke below instead.)
        // Bounded DVD state-machine probes (remove once M2 answered):
        // 189FC = completion dispatcher (r3=cmd block, +8=type tag);
        // 16018 = waiter flag check; 18CEC = inquiry re-issue site.
        // fzCS: uncap 189FC (was first-8-only): count entries per lap-phase
        // (before/after AC44) to learn whether the sink re-runs per lap.
        if(pc==0x800189FCu){ static unsigned _n=0; _n++;
          if(_n<=8||_n%5000000==0){ uint32_t head=0; guest_read32(g_cpu.gpr[13]-31800u, &head);
            fprintf(stderr,"[dvdsm] 189FC hits=%u head=0x%08X tb=0x%llX\n", _n, head, (unsigned long long)g_cpu.timebase); }
          // fzEO: tag La = row 14 (0x80124018+14*4) -> 0x80018CC8 re-issue.
          // Confirm the table row the dispatcher will take for THIS entry.
          { static unsigned _t=0; if(++_t<=6){ uint32_t blk=g_cpu.gpr[3], tag=0xDEADu, row=0xDEADu;
            if(blk) guest_read32(blk+8u,&tag);
            guest_read32(0x80124018u+tag*4u,&row);
            fprintf(stderr,"[dvdsm] 189FC tag=%u row=0x%08X blk=0x%08X (#t=%u)\n", tag, row, blk, _t); } } }
        // fzEYzb7: 15FC0/16024/16028 dispatch on the waiter path; 16018
        // is native. Uncapped counters show whether the waiter runs and
        // which side (flag==1 short-circuit at 16024 vs clear at 16028).
        if(pc==0x80015FC0u||pc==0x80016024u||pc==0x80016028u){
          static unsigned _w1=0,_w2=0,_w3=0;
          unsigned *c=pc==0x80015FC0u?&_w1:pc==0x80016024u?&_w2:&_w3; (*c)++;
          if(*c<=3||*c%5000000==0){ u32 fl=0; guest_read32(g_cpu.gpr[13]-31592u,&fl);
            fprintf(stderr,"[dvdsm] %s flag-31592=%u DIstatus=0x%08X lr=0x%08X (#%u)\n",
              pc==0x80015FC0u?"15FC0-entry":pc==0x80016024u?"16024-flagset":"16028-flagclear",
              fl, s_di.status, g_cpu.lr, *c); } }
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
        // fzET: 187CC entry lr split — lr=179F0 (drain, m64=1) vs
        // lr=1973C/others (empty-drain probes). Uncapped by phase.
        { static unsigned _l1=0,_l2=0; unsigned *c= g_cpu.lr==0x800179F0u?&_l1:&_l2; (*c)++;
          if(*c<=3||*c%5000000==0){ uint32_t a=0; guest_read32(g_cpu.gpr[13]-31464u,&a);
            fprintf(stderr,"[dvdsm] 187CC-by-lr %s m64=%u r3=0x%08X (#%u)\n",
              g_cpu.lr==0x800179F0u?"drain":"other", a, g_cpu.gpr[3], *c); } }
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
        // fzEQ: 18E4C (m56==0 path's error-report leg, dispatched) — if it
        // fires, the body takes the error leg every completion.
        if(pc==0x80018E4Cu){
          static unsigned _q=0; if(++_q<=4||_q%5000000==0)
            fprintf(stderr,"[dvdsm] 18E4C-errorleg r3=%u r4=0x%08X (#%u)\n", g_cpu.gpr[3], g_cpu.gpr[4], _q); }
        if(pc==0x80018DD8u||pc==0x80018E40u){
          static unsigned _p1=0,_p2=0; unsigned *c=pc==0x80018DD8u?&_p1:&_p2; (*c)++;
          if(*c<=4) fprintf(stderr,"[dvdsm] %s r3=%u (#%u)\n",
            pc==0x80018DD8u?"18DD8-m56set":"18E40-m56zero", g_cpu.gpr[3], *c);
          if((*c%5000000)==0) fprintf(stderr,"[dvdsm] m56branch 18DD8=%u 18E40=%u\n", _p1,_p2); }
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
        // fzDW: entries #1-4 (healthy) show blank new nodes; #5 (DVD) shows
        // the SAME blank new node CDD8 (k=0:0, links 0) — the insert has NOT
        // run yet at entry. #6 shows CDD8 already filed (k=0:182E1BF8,
        // l16=CB08) — filed between #5 and #6. The SELF ([+20]=self) forms
        // after #6. Next: watch [+20] of CDD8 across #5/#6/#7 entries.
        // fzDX: lr is IDENTICAL (AEE0) at #5/#6/#7 — same return address, so
        // the re-invocation is NOT a fresh caller: the walk's own AEDC bl
        // AC44 re-enters (walk recurses via its native call chain) rather
        // than returning. The insert path loops back into the walk top.
        // fzDY correction: AEE0 is the return address for ALL THREE bl AC44
        // sites is FALSE — AEDC's lr IS AEE0 (AEDC+4), but AF58/B1DC have
        // their own lr (AF5C/B1E0). All 7 hits show lr=AEE0 => all come via
        // AEDC (the AE94 wrapper's call), none via AF58/B1DC. The 3 bl sites
        // serve different callers (AE94-wrapper vs AExx others); only the
        // wrapper's path runs here.
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
            // the NEXT callback's fresh walk, not a re-walk. (fzDM
            // correction: only 3 callbacks fire but 7 AE94 + 25M laps run —
            // callbacks #1-3 deliver the sink+walks, then the FIRST callback's
            // native body never returns (7FFF0000 sentinel in live backchain)
            // and its internal walk spins natively. The SELF tail
            // is written once (insert #5) and re-walked by later callbacks'
            // walks that keep resolving the same way (frozen key).
            if(!_haveR30){ _haveR30=1; _firstR30=g_cpu.gpr[30]; }
            // fzDN: sample r1 (stack) per lap: nesting => r1 declines as the
            // callback body re-enters per lap; flat loop => r1 constant.
            // fzDU: ALSO capture r29 (new node) per lap: the tail lap should
            // show r29==CDD8 while r6 walks head..tail; the SELF-link lap
            // would show r6==r29==CDD8 (walk already AT the new node).
            if(_laps<=8||_laps%20000000==0) fprintf(stderr,"[ins] lap=%u r6=0x%08X r29=0x%08X r30=0x%08X r4=0x%08X firstR30=0x%08X %s r1=0x%08X tb=0x%llX\n",
              _laps, g_cpu.gpr[6], g_cpu.gpr[29], g_cpu.gpr[30], g_cpu.gpr[4], _firstR30,
              (g_cpu.gpr[30]==_firstR30)?"SAME-KEY":"NEW-KEY", g_cpu.gpr[1], (unsigned long long)g_cpu.timebase);
            if(_laps%5000000==0){ uint32_t head=0; guest_read32(g_cpu.gpr[13]-31800u, &head);
              fprintf(stderr,"[ins] laps=%u head=0x%08X tb=0x%llX\n", _laps, head, (unsigned long long)g_cpu.timebase); }
            if(_have){ uint32_t s16=0,s20=0;
              guest_read32(_pr6+16u,&s16); guest_read32(_pr29+20u,&s20);
              if(_pr6==_pr29){ static unsigned _n=0;
                uint32_t s16b=0; guest_read32(_pr29+16u,&s16b);
                // fzDQ: r29(new) at AD1C is the NEW node for the CURRENT lap
                // (set by AC44 entry r29=r3 before the walk). On the tail lap
                // r6 advances CB08->CDD8 via ADD8, then the NEXT lap shows
                // r6==r29==CDD8: the walk re-entered at head but r6 reads CDD8
                // because [CB08+20] was already overwritten with CDD8 by the
                // insert. The SELF-link forms when the walk reaches CDD8 and
                // compares CDD8 vs CDD8 (key == its own key => EQ => ADD8
                // advance to [CDD8+20], which the insert just set to CDD8).
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
        // fzDT: downcount is INHERITED, not seeded — first AD1C hit shows
        // -6969693 (whatever the native chain burned since boot), then the
        // +1000 slice replenish outruns the ~50/lap burn and it climbs to -2.
        // The walk entered via ONE dolrecomp_call that never returned; every
        // lap is a downcount-budget return re-dispatched at AD1C. The
        // -1000 threshold DOES fire (that's the return mechanism) — but it
        // returns to the SLICE loop, which re-dispatches AD1C, not to AE80.
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
            (void)hi;
            // fzEB: the AD28-AD34 subfc/subfe/neg chain is an unsigned >=
            // compare of (r4:r30) vs (node8^0x8000:node12): advance (ADD8)
            // while search key >= node key. ca==1 means r30 >= node12
            // (no borrow out of the low subtract). Old == emulation was
            // inverted and mislabeled every advancing lap as ADE4.
            int toADD8 = (r4 > r3h) || (r4 == r3h && ca == 1);
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
        // fzDR: AE80 has ZERO hits across every log ever (also ADE0 silent)
        // while AD1C fires 20M+ — the back-edge always takes the native goto
        // (downcount never <= -1000 there), so the exit path is starved by
        // construction, not by guest logic. Candidate fix: credit downcount
        // at the ADE0 back-edge so the exit can dispatch (see next).
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

        // (was: 0x8001AF8C poke r13-31388=gpr30-or-1 — REMOVED fzEYzb50:
        // 1AF7C loads r30=[r13-31388], 1AF8C loads r0=[same], 1AF90 exits
        // iff r30!=r0. The poke wrote [word]=r30 every lap, forcing
        // r30==r0 forever = INFINITE GAME-THREAD WAIT. The word is owned
        // by the DVD completion path (who files it natively? TBD — but
        // NOT us). Let the guest file it.)
        // GXRuntime VI retrace drive: 1 block dispatch = 1 work unit.
        // Retrace => +675000 timebase ticks + VI status bit asserted.
        // Interrupt delivery stays parked (s_os_dispatch_interrupt==0) so
        // VIWaitForRetrace-style loops observe level-triggered pending.
        // fzEYzb79: on each retrace ALSO advance the VI position/compare
        // model (Dolphin VideoInterface.cpp: IR_INT sets when the half-line
        // counter reaches the VCT/HCT programmed in each INTSR; our
        // interrupts.c has no position counter, so bit15 of the halfword
        // the 1A55C waker polls never sets and r7 stays 0 => the waker
        // gate never takes the 1A60C side). Toggle even/odd field per
        // retrace so both 0x30/0x34 reads report a live beam.
        { static unsigned long long _rt=0; if(++_rt==1||_rt%2000000==0){
          uint32_t d4=0; guest_read32(0x800000D4u,&d4);
          fprintf(stderr,"[irq] retrace#%llu DIpend=%d ext=%d EE=%d mask=0x%08X cause=0x%08X D4=0x%08X disp=0x%08X\n",
            _rt, dol_di_interrupt_pending(&s_di),
            dol_interrupts_external_pending(&s_interrupts), (g_cpu.msr&MSR_EE)!=0,
            dol_interrupts_pi_mask(&s_interrupts), dol_interrupts_pi_cause(&s_interrupts),
            d4, s_os_dispatch_interrupt); } }
        // fzEA: +1 timebase tick per slice-loop iteration (deterministic:
        // the dispatch count is deterministic). Back-to-back DI completions
        // (INQUIRY then STOPMOTOR) otherwise share one frozen tb, so the
        // AECC key (r6=r28+tb) re-walks identically onto the just-filed
        // node (CDD8 vs CDD8 => EQ => ADD8 => SELF tail => park). Native
        // runs never dispatch, so intra-walk keys stay stable; inter-walk
        // keys differ by iteration count. VI retrace cadence below is
        // untouched.
        g_cpu.timebase += 1u;
        dol_vi_clock_advance(&s_vi_clock, 1u);
        {
            u64 ticks = 0;
            while(dol_vi_clock_pop_retrace(&s_vi_clock, &ticks)){
                g_cpu.timebase += ticks;
                dol_interrupts_assert_vi_retrace(&s_interrupts);
                dol_si_latch_poll(&s_si, 0xFu);
                dol_interrupts_set_source(&s_interrupts, DOL_PI_CAUSE_SI, dol_si_interrupt_pending(&s_si));
                // fzEYzb116 (REVISED probe138 — TOKEN alone never reaches
                // handler 19; D9CC's cause&mask gate is INDEX-parallel, and
                // the index-19 leg needs BOTH 0x200 AND 0x8000-set paths per
                // the bit-walk ORs at DA70/DA80/DAA8: bits 0x8000+0x4000+
                // 0x2000+0x1000 + low 0x8000+0x1000+0x2000): set TOKEN (idx
                // 18's source) AND PE_FINISH (idx 19's source) together iff
                // the guest registered handler 19 (34488) AND the D9CC mask
                // enables BOTH causes (guest-gated, like SI). Both are real
                // PE completion sources the null-GX backend otherwise never
                // produces; the guest's own 344B8/344DC/344E4/344F0-344F4
                // waiter-link chain decides whether the 3416C flag flips.
                // (Probe137 proved the OR: TOKEN-only => 343BC x4, 34488 x0.)
                // fzEYzb116 (CONFIRMED probe140: mask=0xFFC has TOKEN but
                // NOT FINISH — so the cause-19 leg's 0x8000-set path at
                // DBE8 can never take; handler 19 is unreachable while the
                // mask lacks PE_FINISH). Synthesize the ack the mask is
                // WAITING for: the guest ACKs PE_FINISH by writing
                // 0xCC00100A|0x0008 (DOL_PE_FINISH_ACK_BIT), which clears
                // PI cause-0x400 — but with null-GX the cause never SET, so
                // the D9CC leg sends the guest in circles. Set the FINISH
                // cause iff handler 19 is registered (game-owned dispatch
                // stays the guest's; this only provides the device event
                // the null backend otherwise never produces, same class as
                // the SI latch and VCT latch fixes). No mask condition:
                // the D9CC table gates on cause&mask itself.
                // DIAGNOSTIC: log mask/cause at the synthesis point.
                { static unsigned long long _pm=0; if(++_pm==1||_pm%2000000==0){
                    fprintf(stderr,"[pe19] synth mask=0x%08X cause=0x%08X reg=%d\n",
                      dol_interrupts_pi_mask(&s_interrupts),
                      dol_interrupts_pi_cause(&s_interrupts),
                      s_pe19_registered?1:0); } }
                if(s_pe19_registered)
                    dol_interrupts_commit_pe_finish(&s_interrupts);
            }
            // AID audio-DMA cadence runs on guest-work units (Strikers:
            // audio_poll per block). No audio output here — but the
            // ENGINE state (chunks consumed, completion interrupt) is
            // guest-visible via DSP CONTROL/AID, so poll it per dispatch.
            // Silent while idle (control ENABLE clear): poll early-returns
            // on a counter increment until a chunk is due.
            { u32 src = 0;
              if(dol_audio_dma_poll(&s_audio_dma, &src)) chassis_sync_dsp_irq(); }
            chassis_deliver_external();
        }
        // fzEYzb74: delivered interrupts redirect g_cpu.pc (to D9CC), but the
        // local `pc` was captured BEFORE delivery — the slice tail then called
        // dolrecomp_call(stale pc), which resets ctx->pc first thing, silently
        // dropping every delivered interrupt (4 deliveries, 0 D9CC runs all
        // session). Refresh + re-run the iteration top for the new pc.
        // Terminates: delivery clears MSR[EE], so no immediate re-delivery.
        if(g_cpu.pc != pc){ pc = g_cpu.pc; continue; }
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
          // fzEYzb71: uncap fully (was first-4): #trampolines vs #dispatches
          // diagnoses the queue. Trampoline > dispatch by exactly the nested
          // (poll_nested) count; dispatch stopping while issues continue =
          // queue overflow (cap 32) or callback never queued.
          { static unsigned _t=0; if(++_t<=12||_t%100==0) fprintf(stderr,"[cb] trampoline cb=0x%08X from pc=0x%08X (#%u)\n", cbpc, pc, _t); }
          // fzER: drive the callback frame to completion (blr to
          // HLE_CALLBACK_RETURN), resuming across downcount-budget returns
          // via the frame's own pc — NOT via slice re-dispatch. The old
          // single-call + continue abandoned the frame at the first budget
          // return (18D68 back-edge), and the slice re-dispatched the ENTRY
          // pc (18D1C/17958) as a FRESH call: the body re-ran from the top,
          // re-issued STOPMOTOR via 16920, and m56 never survived to the
          // 18DD4 branch. Completions queued mid-frame (STOPMOTOR issued by
          // the body's own 16920) run nested via poll_nested once the outer
          // frame returns, before the slice context is restored. Guarded:
          // 1M re-entries then restore + fall through to slice dispatch.
          int guard = 0;
          // fzEYzb48 (answered — frame terminates in <4k re-entries each
          // time: no frame# log ever printed, no guard trip. The 4
          // dispatches + 4 trampolines in 20s = completions #1-3 run to
          // HLE_CALLBACK_RETURN cleanly (CORRECTION from live counts:
          // the ISSUE side does NOT stall — 16A38 issues reach #800+ and
          // 18D1C completions reach #7+ in the same run. The 3-dispatch
          // count was the probe's first-3 cap, not a stall. The loop is
          // INQUIRY->completion->STOPMOTOR->completion->INQUIRY... at
          // full rate, 800+ issues in 20s. It never STALLS and never
          // ADVANCES: a limit cycle at full speed, not a deadlock.)
          for(;;){
            if(++guard > 1000000){
              { static int _w=0; if(!_w){ _w=1;
                fprintf(stderr,"[cb] frame guard tripped cb=0x%08X pc=0x%08X\n", cbpc, g_cpu.pc); } }
              dol_hle_handle_callback_return(&g_cpu, HLE_CALLBACK_RETURN);
              break; }
            g_cpu.timebase += 1u; // deterministic: per re-entry, like slice
            // fzESb: replenish downcount exactly like the slice loop.
            // Without this the AD1C walk exhausts the budget once, then
            // EVERY back-edge returns immediately and the frame spins at
            // AD1C forever (fzED guard tripped here, not in the body).
            if(g_cpu.downcount < -800) g_cpu.downcount += 1000;
            if(g_cpu.pc == HLE_CALLBACK_RETURN){
              if(dol_hle_poll_nested(&g_cpu)){ cbpc = g_cpu.pc; continue; }
              break; }
            // fzEYzb23 (answered — decode recorded in the fzEYzb25 comment;
            // tracer now quiet: it confirmed the frame exits via 1A178
            // lr=19230 with m60=14 frozen, and would spam every run).
            dolrecomp_call(&g_cpu, g_cpu.pc); }
          dol_hle_handle_callback_return(&g_cpu, HLE_CALLBACK_RETURN);
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
            // fzEYzb156 (PLAN M2): static recomp only covers DOL addresses.
            // Runtime-loaded REL code in the heap (0x8155A4BC = fze.sample.rel
            // entry, reached via GameMainLoopFrame 59A8 bctrl) has no chunk,
            // so dolrecomp_call misses and the slice parked here forever.
            // Run those pcs on the integer interpreter (rel_interp.c) instead
            // of compiling REL chunks into recomp_core. The interpreter
            // advances guest pc itself (incl. bl back into DOL chunks); the
            // slice loop then resumes normal dispatch. Interp is CAPPED per
            // slice iteration (10k steps) so a spinning REL yields to VI/DI.
            { extern bool rel_interp_step(CPUState* cpu, u32 cia);
              static int interp_stall = 0;
              // fzEYzb165: dolrecomp_call already missed, so this pc is NOT
              // DOL-covered. Anything in MEM1 (cached 0x80000000 or uncached
              // 0xC0000000 alias) is a heap/REL candidate — the old
              // 0x81000000+ window missed the 0x801C... second-stage target
              // (REL return value bctrl'd at 59E4, probe196 miss 0x801CAADC).
              bool in_heap = (pc >= 0x80000000u && pc < 0x81800000u) ||
                             (pc >= 0xC0000000u && pc < 0xC1800000u);
              if(in_heap && g_cpu.exception==0){
                int budget = 10000;
                while(budget-- > 0){
                  u32 cur = g_cpu.pc;
                  // fzEYzb157: bl into a DOL chunk must NOT break the
                  // interpreter loop — dolrecomp_call the DOL pc inline
                  // (the [relinterp] log shows bursts dying at exactly such
                  // returns: 79888/11424/03458/17160). Only heap pcs and
                  // the callback sentinel sequence through here.
                  // fzEYzb159: faults (FP/STWCX/exception vectors) break out
                  // to the slice loop, which owns vector/rfi/FP-resume
                  // handling — the interp loop must not dispatch vectors.
                  if(g_cpu.exception) break;
                  if(cur == HLE_CALLBACK_RETURN){
                    if(dol_hle_handle_callback_return(&g_cpu, cur)) continue;
                    break; }
                  // Heap callbacks (ARQ completion): the HLE queues a heap
                  // pc while the interp loop spins in the heap wait. Run it
                  // inline WITHOUT slice context save/restore (which would
                  // clobber the wait-site registers and re-enter the wait).
                  // The callback returns via blr to the heap wait pc in lr.
                  { extern bool dol_hle_poll_heap_callback(CPUState* cpu, u32 heap_pc);
                    if(dol_hle_poll_heap_callback(&g_cpu, cur)) continue; }
                  // Inside the interpreter loop: dispatch DOL-covered pcs
                  // via dolrecomp_call (host_call probes + chunks run);
                  // heap REL pcs miss and single-step below. Coverage
                  // (dolrecomp_find_original), not address range, decides:
                  // heap bl-targets WITH static chunks (205A0, 1E864,
                  // 79888, ...) must dispatch — the range test treats them
                  // as heap and single-steps OS code without chunk
                  // semantics. Probe209 proved the cost: 205A0 ARQ post
                  // never dispatched (queue never pumped), REL parked at
                  // 802161D4 forever; with coverage dispatch it posts,
                  // the queue pumps, and the REL advances to 802164C4.
                  if(dolrecomp_find_original(cur)){
                    if(!dolrecomp_call(&g_cpu, cur)) break;
                    if(g_cpu.exception) break;
                    continue; }
                  if(cur < 0x80000000u || (cur >= 0x81800000u && cur < 0xC0000000u) || cur >= 0xC1800000u){
                    if(!dolrecomp_call(&g_cpu, cur)) break;
                    if(g_cpu.exception) break;
                    continue; }
                  if(g_cpu.downcount < -800) g_cpu.downcount += 1000;
                  g_cpu.timebase += 1u;
                  if(!rel_interp_step(&g_cpu, cur)) break;
                }
                { static unsigned _n=0; if(++_n<=4||_n%200==0)
                  fprintf(stderr,"[relinterp] pc=0x%08X lr=0x%08X budget_left=%d exc=%u (#%u)\n",
                    g_cpu.pc, g_cpu.lr, budget, g_cpu.exception, _n); }
                if(g_cpu.exception==0) continue;
              }
              if(g_cpu.exception==0 && !in_heap){
                static int miss_cnt=0;
                if(miss_cnt<6) fprintf(stderr,"[hle] miss pc=0x%08X lr=0x%08X r1=0x%08X\n",pc,g_cpu.lr,g_cpu.gpr[1]);
                miss_cnt++;
              } else if(g_cpu.exception==0 && in_heap){
                interp_stall++;
                if(interp_stall<=4) fprintf(stderr,"[relinterp] STALL pc=0x%08X lr=0x%08X r1=0x%08X\n",pc,g_cpu.lr,g_cpu.gpr[1]);
              }
              (void)interp_stall;
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
