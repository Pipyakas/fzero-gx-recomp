// host/src/hle_callback.c — minimal async-callback trampoline.
//
// Extracted from GXRuntime hle_core.c's callback queue (queue/poll/return +
// init), which is self-contained: it only touches GPR/FPR/PC/LR/MSR/SRR and
// never calls guest_memory/aram/card/platform. hle_core.c itself can't link
// here (it needs the newer CPUState with exram/external_pointer, plus card,
// ARAM and platform backends). This TU includes DolRecomp's cpu.h, whose
// CPUState prefix is ABI-compatible for the fields used.
#include "../../vendor/RingOut/DolRecomp/src/cpu/cpu.h"
#include <string.h>
#include <stdio.h>

#define HLE_CALLBACK_RETURN 0x7FFF0000u
#define HLE_CALLBACK_QUEUE_CAPACITY 32u

typedef struct {
    u32 address;
    u32 r3;
    u32 r4;
} HlePendingCallback;

typedef struct {
    u64 gpr[32];
    f64 fpr[32];
    f64 ps1[32];
    u32 pc;
    u32 lr;
    u32 ctr;
    u32 cr;
    u32 xer;
    u32 fpscr;
    u32 msr;
    u32 srr0;
    u32 srr1;
    u32 dar;
    u32 dsisr;
    u32 ear;
    u32 hid2;
    u32 sr[16];
    u32 gqr[8];
    u32 exception;
    u32 program_exception;
    u32 reserve_addr;
    bool reserve_valid;
} HleSavedContext;

static HlePendingCallback g_callback_queue[HLE_CALLBACK_QUEUE_CAPACITY];
static u32 g_callback_read;
static u32 g_callback_count;
static bool g_callback_active;
static HleSavedContext g_callback_context;

static void save_callback_context(HleSavedContext* saved, const CPUState* cpu) {
    memcpy(saved->gpr, cpu->gpr, sizeof saved->gpr);
    memcpy(saved->fpr, cpu->fpr, sizeof saved->fpr);
    memcpy(saved->ps1, cpu->ps1, sizeof saved->ps1);
    saved->pc = cpu->pc;
    saved->lr = cpu->lr;
    saved->ctr = cpu->ctr;
    saved->cr = cpu->cr;
    saved->xer = cpu->xer;
    saved->fpscr = cpu->fpscr;
    saved->msr = cpu->msr;
    saved->srr0 = cpu->srr0;
    saved->srr1 = cpu->srr1;
    saved->dar = cpu->dar;
    saved->dsisr = cpu->dsisr;
    saved->ear = cpu->ear;
    saved->hid2 = cpu->hid2;
    memcpy(saved->sr, cpu->sr, sizeof saved->sr);
    memcpy(saved->gqr, cpu->gqr, sizeof saved->gqr);
    saved->exception = cpu->exception;
    saved->program_exception = cpu->program_exception;
    saved->reserve_addr = cpu->reserve_addr;
    saved->reserve_valid = cpu->reserve_valid;
}

static void restore_callback_context(CPUState* cpu, const HleSavedContext* saved) {
    memcpy(cpu->gpr, saved->gpr, sizeof saved->gpr);
    memcpy(cpu->fpr, saved->fpr, sizeof saved->fpr);
    memcpy(cpu->ps1, saved->ps1, sizeof saved->ps1);
    cpu->pc = saved->pc;
    cpu->lr = saved->lr;
    cpu->ctr = saved->ctr;
    cpu->cr = saved->cr;
    cpu->xer = saved->xer;
    cpu->fpscr = saved->fpscr;
    cpu->msr = saved->msr;
    cpu->srr0 = saved->srr0;
    cpu->srr1 = saved->srr1;
    cpu->dar = saved->dar;
    cpu->dsisr = saved->dsisr;
    cpu->ear = saved->ear;
    cpu->hid2 = saved->hid2;
    memcpy(cpu->sr, saved->sr, sizeof saved->sr);
    memcpy(cpu->gqr, saved->gqr, sizeof saved->gqr);
    cpu->exception = saved->exception;
    cpu->program_exception = saved->program_exception;
    cpu->reserve_addr = saved->reserve_addr;
    cpu->reserve_valid = saved->reserve_valid;
}

// Args are raw guest registers: queue(address, r3, r4). (Renamed from
// channel/result: DVD completion needs r3=result + r4=block pointer.)
bool dol_hle_queue_guest_callback(u32 address, u32 r3, u32 r4) {
    if (address == 0)
        return true;
    if (g_callback_count == HLE_CALLBACK_QUEUE_CAPACITY) {
        fprintf(stderr, "[cb] completion callback queue overflow\n");
        return false;
    }
    u32 index = (g_callback_read + g_callback_count) % HLE_CALLBACK_QUEUE_CAPACITY;
    g_callback_queue[index].address = address;
    g_callback_queue[index].r3 = r3;
    g_callback_queue[index].r4 = r4;
    g_callback_count++;
    return true;
}

bool dol_hle_poll_callback(CPUState* cpu) {
    if (cpu == NULL || g_callback_active || g_callback_count == 0)
        return false;

    HlePendingCallback pending = g_callback_queue[g_callback_read];
    g_callback_read = (g_callback_read + 1u) % HLE_CALLBACK_QUEUE_CAPACITY;
    g_callback_count--;

    save_callback_context(&g_callback_context, cpu);
    g_callback_active = true;
    // fzEV: callbacks run on the issuing thread with a FRESH condition
    // register — the preempted slice's CR (e.g. AD1C-walk compare residue)
    // must not leak into the callback body. The 18D3C branch reads CR1
    // (set by 18D20 cmplwi) natively mid-chunk, but any stale CR1 from the
    // preempted context that survives to a later compare corrupts the
    // branch. Reset CR (and XER carried-borrow state) at dispatch.
    cpu->cr = 0;
    cpu->xer &= ~0x20000000u;
    // fzBU: the callback's NON-ARG regs must match what the preempted slice
    // had — EXCEPT r30/r31, which the native 18D1C body reads (16984 r6=r30,
    // 16960 r31=...) as *its own locals* without ever writing them first.
    // In real hardware the callback runs on the thread that issued the
    // command, whose r30/r31 hold that thread's values — not the allocator's
    // leftovers (0x1823CF40) from a preempted AC34 slice. Zero r30/r31 only;
    // everything else restores exactly (post-lap SELF-link came from stale
    // r30; keep the slice's other regs intact, including r1/sp).
    cpu->gpr[30] = 0;
    cpu->gpr[31] = 0;
    cpu->gpr[3] = pending.r3;
    cpu->gpr[4] = pending.r4;
    cpu->pc = pending.address;
    cpu->lr = HLE_CALLBACK_RETURN;
    cpu->exception = 0;
    cpu->program_exception = 0;
    { static unsigned _n=0; if(++_n<=3||_n%200000==0)
      fprintf(stderr, "[cb] dispatch callback=0x%08X r3=0x%08X r4=0x%08X (#%u)\n",
            pending.address, pending.r3, pending.r4, _n); }
    return true;
}

// fzEC: dequeue the next queued callback WITHOUT saving context. Used to
// nest completions issued from inside a running callback body (e.g. the
// STOPMOTOR completion queued by 18D1C's own sink chain) within the OUTER
// callback's frame, before the outer saved context is restored. Keeps
// g_callback_active set (owned by the outer poll); the single
// handle_callback_return at the end restores the preempted slice context.
bool dol_hle_poll_nested(CPUState* cpu) {
    if (cpu == NULL || !g_callback_active || g_callback_count == 0)
        return false;
    HlePendingCallback pending = g_callback_queue[g_callback_read];
    g_callback_read = (g_callback_read + 1u) % HLE_CALLBACK_QUEUE_CAPACITY;
    g_callback_count--;
    cpu->gpr[3] = pending.r3;
    cpu->gpr[4] = pending.r4;
    cpu->pc = pending.address;
    cpu->lr = HLE_CALLBACK_RETURN;
    cpu->exception = 0;
    cpu->program_exception = 0;
    return true;
}

bool dol_hle_handle_callback_return(CPUState* cpu, u32 address) {
    if (address == HLE_CALLBACK_RETURN && g_callback_active) {
        restore_callback_context(cpu, &g_callback_context);
        g_callback_active = false;
        return true;
    }
    return false;
}

void dol_hle_init(const void* config) {
    (void)config;
    g_callback_read = 0;
    g_callback_count = 0;
    g_callback_active = false;
    memset(&g_callback_context, 0, sizeof g_callback_context);
}
