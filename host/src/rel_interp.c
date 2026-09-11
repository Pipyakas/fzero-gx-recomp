// host/src/rel_interp.c — single-step interpreter fallback for runtime-loaded
// REL code executing from the heap (e.g. 0x8155A4BC, fze.sample.rel entry).
//
// The static recomp pipeline only covers DOL addresses: dolrecomp_call() has
// no chunk for heap pcs, so the slice loop parks on [hle] miss forever once
// the guest bctrls into a DVD-loaded REL. PLAN M2 calls for running RELs via
// interpreter, not by compiling REL chunks into recomp_core.
//
// Coverage: full integer ISA (what the opcode census shows all 14 RELs use:
// prims 7,8,10,11,12,13,14,15,16,18,19,20,21,23-29,31,32-38,40-47 + XO31/XO19
// integer subset) + branches + integer loads/stores + SPR LR/CTR/XER + the FP
// side the second-stage REL needs: FP loads/stores (lfs/lfd/stfs/stfd,
// D/X/update forms), scalar single+double arithmetic (fadds..fnmsubs,
// fadd..fnmsub, fctiw[z], fmr/neg/abs/nabs, frsp, fsel, fres/frsqrte),
// fcmpu/fcmpo (CR + FPSCR/FPCC), FPSCR moves (mtfsb0/1, mtfsfi, mtfsf, mffs,
// mcrfs), paired-single ALU (ps_add..ps_sel + merges/sums), psq_l/st
// (D/X/update, via ppc_psq_load/store), stfiwx, dcbz (real zero), stwcx.,
// plus cache/sync no-ops (dcbst/dcbf/dcbi/icbi/sync/eieio/tlbsync/isync) and
// string moves (lswi/lswx/stswi/stswx) + mcrxr. Every emitted line mirrors
// vendor/RingOut/DolRecomp/src/backend/emitter.c semantics (CA/OV/RC/CR
// handling, ps_round, Gekko single-lane broadcast), so interp and recomp
// agree by construction. FPRF uses the same LAZY scheme as recomp (ri_fprf_s/d
// tag g_fprf_value/kind; ri_fcompare and the ppc_f* helpers flush/classify),
// so fpscr-observable state matches a lazy-FPRF recomp build bit-for-bit.
//
// Interop with recomp dispatch: bl to a DOL address sets pc there and returns
// true — the next slice iteration dispatches the existing chunk. blr/bclr
// back to a DOL lr likewise resumes recomp code. Only heap->heap control
// stays in the interpreter.
#include "../../vendor/RingOut/DolRecomp/src/cpu/cpu.h"
#include <stdio.h>
#include <string.h>

// Field extraction (same as decoder.c PPC_* macros).
#define R_RD(w)  (((w) >> 21) & 0x1F)
#define R_RS(w)  (((w) >> 21) & 0x1F)
#define R_RA(w)  (((w) >> 16) & 0x1F)
#define R_RB(w)  (((w) >> 11) & 0x1F)
#define R_RC(w)  (((w) >> 6) & 0x1F)
#define R_BO(w)  (((w) >> 21) & 0x1F)
#define R_BI(w)  (((w) >> 16) & 0x1F)
#define R_XO(w)  (((w) >> 1) & 0x3FF)
#define R_AXO(w) (((w) >> 1) & 0x1F)
#define R_SIMM(w) ((s16)((w) & 0xFFFF))
#define R_UIMM(w) ((u16)((w) & 0xFFFF))
#define R_SPR(w) ((((w) >> 16) & 0x1F) | (((w) >> 6) & 0x3E0))

static s32 ri_sign_ext(u32 v, int bits) {
    u32 m = 1u << (bits - 1);
    return (s32)((v ^ m) - m);
}

// CR0 from a GPR value, including XER[SO] (mirrors emit_set_cr0_from_gpr).
static void ri_set_cr0(CPUState* cpu, u32 val) {
    u32 b = 0;
    s32 s = (s32)val;
    if (s < 0) b |= 0x8u;
    if (s > 0) b |= 0x4u;
    if (s == 0) b |= 0x2u;
    b |= (cpu->xer >> 31) & 1u;
    cpu->cr = (cpu->cr & 0x0FFFFFFFu) | (b << 28);
}

static void ri_cmp_s32(CPUState* cpu, u32 crf, s32 a, s32 b) {
    u32 sh = 4u * (7u - crf), f = 0;
    if (a < b) f |= 0x8u;
    if (a > b) f |= 0x4u;
    if (a == b) f |= 0x2u;
    f |= (cpu->xer >> 31) & 1u;
    cpu->cr = (cpu->cr & ~(0xFu << sh)) | (f << sh);
}

static void ri_cmp_u32(CPUState* cpu, u32 crf, u32 a, u32 b) {
    u32 sh = 4u * (7u - crf), f = 0;
    if (a < b) f |= 0x8u;
    if (a > b) f |= 0x4u;
    if (a == b) f |= 0x2u;
    f |= (cpu->xer >> 31) & 1u;
    cpu->cr = (cpu->cr & ~(0xFu << sh)) | (f << sh);
}

// Branch condition (mirrors emit_branch_condition). Handles ctr--.
static void ri_branch_cond(CPUState* cpu, u8 bo, u8 bi, int* ctr_ok, int* cr_ok) {
    if (bo & 0x04) { *ctr_ok = 1; }
    else {
        cpu->ctr--;
        *ctr_ok = (((cpu->ctr != 0) ? 1u : 0u) ^ ((bo >> 1) & 1u)) != 0;
    }
    if (bo & 0x10) { *cr_ok = 1; }
    else {
        u32 mask = 0x80000000u >> bi;
        *cr_ok = (((cpu->cr & mask) != 0) == (((bo >> 3) & 1u) ? 1 : 0));
    }
}

static u32 ri_mask32(u8 mb, u8 me) {
    u32 m = 0; u8 b = mb;
    for (;;) { m |= 0x80000000u >> b; if (b == me) break; b = (u8)((b + 1) & 31); }
    return m;
}

static u32 ri_rotl32(u32 v, u32 sh) {
    sh &= 31u;
    return sh ? ((v << sh) | (v >> (32u - sh))) : v;
}

static u32 ri_ea_d(CPUState* cpu, u32 ra, s16 simm) {
    return ra == 0 ? (u32)(s32)simm : cpu->gpr[ra] + (u32)(s32)simm;
}

static u32 ri_ea_x(CPUState* cpu, u32 ra, u32 rb) {
    u32 b = cpu->gpr[rb];
    return ra == 0 ? b : cpu->gpr[ra] + b;
}

// --- FP bit-cast helpers (mirror dolrecomp_f32_from_bits etc. in emitter.c,
// which are memcpy-based to avoid aliasing issues). ---
static f32 ri_f32_from_bits(u32 bits) {
    f32 v; memcpy(&v, &bits, sizeof(v)); return v;
}
static u32 ri_f32_to_bits(f32 v) {
    u32 b; memcpy(&b, &v, sizeof(b)); return b;
}
static f64 ri_f64_from_bits(u64 bits) {
    f64 v; memcpy(&v, &bits, sizeof(v)); return v;
}
static u64 ri_f64_to_bits(f64 v) {
    u64 b; memcpy(&b, &v, sizeof(b)); return b;
}
// Paired-single lane round: (f64)(f32)value (emitter's dolrecomp_ps_round).
static f64 ri_ps_round(f64 v) { return (f64)(f32)v; }
// Lazy FPRF pending tag (mirrors RECOMP lazy dolrecomp_fprf_s/d: remember the
// value, classify only when FPSCR becomes visible). Keeps interp and recomp
// fpscr-observable state identical without eager classification cost.
static void ri_fprf_s(CPUState* cpu, f32 v) {
    (void)cpu; g_fprf_value = (f64)v; g_fprf_kind = 1u;
}
static void ri_fprf_d(CPUState* cpu, f64 v) {
    (void)cpu; g_fprf_value = v; g_fprf_kind = 2u;
}
// CR1 from FPSCR (mirrors emit_set_cr1_from_fpscr).
static void ri_set_cr1(CPUState* cpu) {
    cpu->cr = (cpu->cr & 0xF0FFFFFFu) | ((cpu->fpscr >> 4) & 0x0F000000u);
}
// Scalar float compare: CR field + FPSCR/FPCC, with pending-FPRF flush first
// (mirrors emit_fcompare ordering).
static void ri_fcompare(CPUState* cpu, u32 crf, f64 a, f64 b) {
    u32 sh = 4u * (7u - crf), f = 0;
    if (a < b) f |= 0x8u;
    else if (a > b) f |= 0x4u;
    else if (a == b) f |= 0x2u;
    else f |= 0x1u;
    cpu->cr = (cpu->cr & ~(0xFu << sh)) | (f << sh);
    ppc_fprf_flush(cpu);
    cpu->fpscr = (cpu->fpscr & ~(0xFu << 12)) | (f << 12);
}
// Paired-single compare: CR only, no FPSCR touch (mirrors PS_CMPU0 emitter).
static void ri_pscmp(CPUState* cpu, u32 crf, f32 a, f32 b) {
    u32 sh = 4u * (7u - crf), f = 0;
    if (a < b) f |= 0x8u;
    else if (a > b) f |= 0x4u;
    else if (a == b) f |= 0x2u;
    else f |= 0x1u;
    cpu->cr = (cpu->cr & ~(0xFu << sh)) | (f << sh);
}
// PSQ D-form 12-bit signed offset (decoder.c decode_psq_d_rt_ra).
static s32 ri_psq_simm(u32 raw) { return ri_sign_ext(raw & 0x0FFFu, 12); }
static u32 ri_ea_psq(CPUState* cpu, u32 ra, s32 simm) {
    return ra == 0 ? (u32)simm : cpu->gpr[ra] + (u32)simm;
}
// String-op helpers (mirror decoder.c reg_in_wrapped_range/string_register_count).
static bool ri_in_wrapped_range(u32 start, u32 count, u32 reg) {
    for (u32 i = 0; i < count; i++)
        if (((start + i) & 31u) == reg) return true;
    return false;
}
static u32 ri_string_count(u32 nb) { return nb ? (nb + 3u) / 4u : 8u; }
// Sub-decode miss (bounded; own counter so FP-family misses don't burn the
// integer-miss budget). Returns false for direct `return` at invalid encodings.
static bool ri_miss_sub(u32 code, u32 raw, u32 cia, const char* fam) {
    static unsigned _s = 0;
    if (_s < 8) { _s++;
        fprintf(stderr, "[relinterp] no cover %s code=%u raw=0x%08X @0x%08X\n",
                fam, code, raw, cia); }
    return false;
}

// Execute the single instruction at cia. Returns true if executed (cpu->pc
// advanced; or cpu->exception pending, pc already redirected by the fault).
// Returns false if the opcode is outside the integer subset (caller logs miss).
bool rel_interp_step(CPUState* cpu, u32 cia) {
    u8* h = dolrecomp_cached_ram(cpu, cia, 4u);
    u32 raw = h ? read_be32(h) : mem_read32(cpu, cia);
    if (cpu->exception) return true;
    u32 prim = raw >> 26;
    u32 rc = raw & 1u;

    switch (prim) {
    case 3: { // twi
        u32 to = R_RD(raw);
        if (ppc_trap_condition((u8)to, cpu->gpr[R_RA(raw)], (u32)(s32)R_SIMM(raw))) {
            ppc_program_exception(cpu, PPC_PROGRAM_TRAP, cia);
            return true;
        }
        cpu->pc = cia + 4; return true;
    }
    case 4: { // paired-single ALU + psq indexed (decoder XO lattice)
        // dcbz_l (XO 1014) is NOT an FPU op: no fp guard in the emitter,
        // so route it before the fp check (else an FP-disabled MSR window
        // would take the 0x800-vector instead of executing it).
        u32 xo = R_XO(raw);
        if (xo == 1014) { // (decoder: rD==0, rc==0)
            if (R_RD(raw) || rc) break;
            ppc_dcbz_l(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), cia);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        }
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 d = R_RD(raw), a = R_RA(raw), b = R_RB(raw), c = R_RC(raw);
        switch (xo & 0x3Fu) {
        case 6: case 38: { // psq_lx / psq_lux (rc must be 0; lux needs rA)
            if (rc) return ri_miss_sub(xo & 0x3Fu, raw, cia, "psq");
            u32 lux = (xo & 0x3Fu) == 38;
            if (lux && a == 0) return ri_miss_sub(xo & 0x3Fu, raw, cia, "psq");
            u32 ea = ri_ea_x(cpu, a, b);
            if (!ppc_psq_load(cpu, (u8)d, ea, (raw >> 10) & 1u,
                              (u8)((raw >> 7) & 7u), true, cia)) return true;
            if (lux) cpu->gpr[a] = ea;
            cpu->pc = cia + 4; return true;
        }
        case 7: case 39: { // psq_stx / psq_stux (rs in RD field)
            if (rc) return ri_miss_sub(xo & 0x3Fu, raw, cia, "psq");
            u32 stux = (xo & 0x3Fu) == 39;
            if (stux && a == 0) return ri_miss_sub(xo & 0x3Fu, raw, cia, "psq");
            u32 ea = ri_ea_x(cpu, a, b);
            if (!ppc_psq_store(cpu, (u8)d, ea, (raw >> 10) & 1u,
                               (u8)((raw >> 7) & 7u), true, cia)) return true;
            if (stux) cpu->gpr[a] = ea;
            cpu->pc = cia + 4; return true;
        }
        default: break;
        }
        switch (xo) {
        case 0: ri_pscmp(cpu, (raw >> 23) & 7u, (f32)cpu->fpr[a], (f32)cpu->fpr[b]);
            cpu->pc = cia + 4; return true; // ps_cmpu0
        case 32: ri_pscmp(cpu, (raw >> 23) & 7u, (f32)cpu->fpr[a], (f32)cpu->fpr[b]);
            cpu->pc = cia + 4; return true; // ps_cmpo0
        case 64: ri_pscmp(cpu, (raw >> 23) & 7u, (f32)cpu->ps1[a], (f32)cpu->ps1[b]);
            cpu->pc = cia + 4; return true; // ps_cmpu1
        case 96: ri_pscmp(cpu, (raw >> 23) & 7u, (f32)cpu->ps1[a], (f32)cpu->ps1[b]);
            cpu->pc = cia + 4; return true; // ps_cmpo1
        case 18: // ps_div
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] / (f32)cpu->fpr[b]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] / (f32)cpu->ps1[b]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 20: // ps_sub
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] - (f32)cpu->fpr[b]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] - (f32)cpu->ps1[b]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 21: // ps_add
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] + (f32)cpu->fpr[b]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] + (f32)cpu->ps1[b]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 24: // ps_res (rA must be 0)
            if (a) return ri_miss_sub(24, raw, cia, "ps");
            { f64 r0, r1; ppc_ps_res(cpu, cpu->fpr[b], cpu->ps1[b], &r0, &r1);
              cpu->fpr[d] = ri_ps_round(r0); cpu->ps1[d] = ri_ps_round(r1); }
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 26: // ps_rsqrte (rA must be 0)
            if (a) return ri_miss_sub(26, raw, cia, "ps");
            { f64 r0, r1; ppc_ps_rsqrte(cpu, cpu->fpr[b], cpu->ps1[b], &r0, &r1);
              cpu->fpr[d] = ri_ps_round(r0); cpu->ps1[d] = ri_ps_round(r1); }
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 40: // ps_neg
            cpu->fpr[d] = (f64)ri_f32_from_bits(ri_f32_to_bits((f32)cpu->fpr[b]) ^ 0x80000000u);
            cpu->ps1[d] = (f64)ri_f32_from_bits(ri_f32_to_bits((f32)cpu->ps1[b]) ^ 0x80000000u);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 72: // ps_mr
            cpu->fpr[d] = cpu->fpr[b]; cpu->ps1[d] = cpu->ps1[b];
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 136: // ps_nabs
            cpu->fpr[d] = (f64)ri_f32_from_bits(ri_f32_to_bits((f32)cpu->fpr[b]) | 0x80000000u);
            cpu->ps1[d] = (f64)ri_f32_from_bits(ri_f32_to_bits((f32)cpu->ps1[b]) | 0x80000000u);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 264: // ps_abs
            cpu->fpr[d] = (f64)ri_f32_from_bits(ri_f32_to_bits((f32)cpu->fpr[b]) & 0x7FFFFFFFu);
            cpu->ps1[d] = (f64)ri_f32_from_bits(ri_f32_to_bits((f32)cpu->ps1[b]) & 0x7FFFFFFFu);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 528: { f64 t0 = cpu->fpr[a], t1 = cpu->fpr[b]; // ps_merge00 (staged)
            cpu->fpr[d] = ri_ps_round(t0); cpu->ps1[d] = ri_ps_round(t1);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        case 560: { f64 t0 = cpu->fpr[a], t1 = cpu->ps1[b]; // ps_merge01
            cpu->fpr[d] = ri_ps_round(t0); cpu->ps1[d] = ri_ps_round(t1);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        case 592: { f64 t0 = cpu->ps1[a], t1 = cpu->fpr[b]; // ps_merge10
            cpu->fpr[d] = ri_ps_round(t0); cpu->ps1[d] = ri_ps_round(t1);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        case 624: { f64 t0 = cpu->ps1[a], t1 = cpu->ps1[b]; // ps_merge11
            cpu->fpr[d] = ri_ps_round(t0); cpu->ps1[d] = ri_ps_round(t1);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        default: break;
        }
        switch (R_AXO(raw)) { // A-form ps_* (10 sum0 ... 31 nmadd)
        case 10: { f64 t0 = cpu->fpr[a], t1 = cpu->ps1[b], tc = cpu->fpr[c]; // ps_sum0 staged
            cpu->fpr[d] = ri_ps_round((f64)((f32)t0 + (f32)t1));
            cpu->ps1[d] = ri_ps_round((f64)(f32)tc);
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        case 11: { f64 ta = cpu->fpr[a], tb = cpu->ps1[b], tc = cpu->fpr[c]; // ps_sum1
            cpu->fpr[d] = ri_ps_round((f64)(f32)tc);
            cpu->ps1[d] = ri_ps_round((f64)((f32)ta + (f32)tb));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        case 12: // ps_muls0
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] * (f32)cpu->fpr[c]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] * (f32)cpu->fpr[c]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 13: // ps_muls1
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] * (f32)cpu->ps1[c]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] * (f32)cpu->ps1[c]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 14: // ps_madds0
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] * (f32)cpu->fpr[c] + (f32)cpu->fpr[b]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] * (f32)cpu->fpr[c] + (f32)cpu->ps1[b]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 15: // ps_madds1
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] * (f32)cpu->ps1[c] + (f32)cpu->fpr[b]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] * (f32)cpu->ps1[c] + (f32)cpu->ps1[b]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 23: // ps_sel
            cpu->fpr[d] = ((f32)cpu->fpr[a] >= 0.0f) ? cpu->fpr[c] : cpu->fpr[b];
            cpu->ps1[d] = ((f32)cpu->ps1[a] >= 0.0f) ? cpu->ps1[c] : cpu->ps1[b];
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 25: // ps_mul
            cpu->fpr[d] = ri_ps_round((f64)((f32)cpu->fpr[a] * (f32)cpu->fpr[c]));
            cpu->ps1[d] = ri_ps_round((f64)((f32)cpu->ps1[a] * (f32)cpu->ps1[c]));
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 28: case 29: case 30: case 31: { // ps_msub/madd/nmsub/nmadd
            u32 ax = R_AXO(raw);
            bool sub = (ax == 28 || ax == 30), neg = (ax >= 30);
            f32 p0 = (f32)cpu->fpr[a] * (f32)cpu->fpr[c];
            f32 p1 = (f32)cpu->ps1[a] * (f32)cpu->ps1[c];
            if (!sub) { p0 += (f32)cpu->fpr[b]; p1 += (f32)cpu->ps1[b]; }
            else { p0 -= (f32)cpu->fpr[b]; p1 -= (f32)cpu->ps1[b]; }
            if (neg) { p0 = -p0; p1 = -p1; }
            cpu->fpr[d] = ri_ps_round((f64)p0); cpu->ps1[d] = ri_ps_round((f64)p1);
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        default: break;
        }
        return ri_miss_sub(xo, raw, cia, "prim4");
    }
    case 7: // mulli
        cpu->gpr[R_RD(raw)] = (u32)((s64)(s32)cpu->gpr[R_RA(raw)] * (s64)(s32)R_SIMM(raw));
        cpu->pc = cia + 4; return true;
    case 8: { // subfic (sets CA)
        u64 r = (u64)(u32)(s32)R_SIMM(raw) + (u64)(~cpu->gpr[R_RA(raw)]) + 1u;
        cpu->gpr[R_RD(raw)] = (u32)r;
        cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(r >> 32) & 1u) << 29);
        cpu->pc = cia + 4; return true;
    }
    case 10: // cmpli
        if ((raw >> 21) & 1u) break;
        ri_cmp_u32(cpu, (raw >> 23) & 7u, cpu->gpr[R_RA(raw)], R_UIMM(raw));
        cpu->pc = cia + 4; return true;
    case 11: // cmpi
        if ((raw >> 21) & 1u) break;
        ri_cmp_s32(cpu, (raw >> 23) & 7u, (s32)cpu->gpr[R_RA(raw)], (s32)R_SIMM(raw));
        cpu->pc = cia + 4; return true;
    case 12: case 13: { // addic / addic.
        u64 a = cpu->gpr[R_RA(raw)], b = (u32)(s32)R_SIMM(raw), r = a + b;
        cpu->gpr[R_RD(raw)] = (u32)r;
        cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(r >> 32) & 1u) << 29);
        if (prim == 13) ri_set_cr0(cpu, (u32)r);
        cpu->pc = cia + 4; return true;
    }
    case 14: // addi
        cpu->gpr[R_RD(raw)] = R_RA(raw) == 0 ? (u32)(s32)R_SIMM(raw)
                                             : cpu->gpr[R_RA(raw)] + (u32)(s32)R_SIMM(raw);
        cpu->pc = cia + 4; return true;
    case 15: // addis
        cpu->gpr[R_RD(raw)] = R_RA(raw) == 0 ? ((u32)(s32)R_SIMM(raw) << 16)
                                             : cpu->gpr[R_RA(raw)] + ((u32)(s32)R_SIMM(raw) << 16);
        cpu->pc = cia + 4; return true;
    case 16: { // bc/bcl/bca/bcla
        u8 bo = R_BO(raw), bi = R_BI(raw);
        int c_ok, r_ok;
        ri_branch_cond(cpu, bo, bi, &c_ok, &r_ok);
        if (c_ok && r_ok) {
            if (raw & 1u) cpu->lr = cia + 4;
            s32 d = ri_sign_ext(raw & 0xFFFCu, 16);
            cpu->pc = (raw & 2u) ? (u32)d : cia + (u32)d;
        } else cpu->pc = cia + 4;
        return true;
    }
    case 17: // sc — SDK cache-sync barrier epilogue: sync and return
        if (raw == 0x44000002u) { cpu->pc = cia + 4; return true; }
        break;
    case 18: { // b/bl/ba/bla
        if (raw & 1u) cpu->lr = cia + 4;
        s32 d = ri_sign_ext(raw & 0x03FFFFFCu, 26);
        cpu->pc = (raw & 2u) ? (u32)d : cia + (u32)d;
        return true;
    }
    case 19: {
        if (raw == 0x4C000064u) { ppc_rfi(cpu, cia); return true; } // rfi
        if (raw == 0x4C00012Cu) { cpu->pc = cia + 4; return true; } // isync
        u32 xo = R_XO(raw);
        if (xo == 16 || xo == 528) { // bclr / bcctr
            u8 bo = R_BO(raw), bi = R_BI(raw);
            int c_ok, r_ok;
            if (xo == 528) {
                // fzEYzb158: bcctr never decrements CTR (BO2 reserved/ignored).
                // ri_branch_cond would do ctr-- for BO2=0 — wrong target next.
                c_ok = 1;
                if (bo & 0x10) { r_ok = 1; }
                else {
                    u32 mask = 0x80000000u >> bi;
                    r_ok = (((cpu->cr & mask) != 0) == (((bo >> 3) & 1u) ? 1 : 0));
                }
            } else
                ri_branch_cond(cpu, bo, bi, &c_ok, &r_ok);
            if (c_ok && r_ok) {
                if (raw & 1u) cpu->lr = cia + 4;
                cpu->pc = (xo == 16 ? cpu->lr : cpu->ctr) & ~3u;
            } else cpu->pc = cia + 4;
            return true;
        }
        if (xo == 0) { // mcrf
            if (raw & ((3u << 21) | (0x7Fu << 11) | 1u)) break;
            u32 ds = 4u * (7u - ((raw >> 23) & 7u)), ss = 4u * (7u - ((raw >> 18) & 7u));
            cpu->cr = (cpu->cr & ~(0xFu << ds)) | (((cpu->cr >> ss) & 0xFu) << ds);
            cpu->pc = cia + 4; return true;
        }
        if (xo == 512) { // mcrxr (decoder: masked fields must be 0)
            if (raw & ((3u << 21) | (0x1Fu << 16) | (0x1Fu << 11) | 1u)) break;
            u32 ds = 4u * (7u - ((raw >> 23) & 7u));
            cpu->cr = (cpu->cr & ~(0xFu << ds)) | (((cpu->xer >> 28) & 0xFu) << ds);
            cpu->xer &= ~0xE0000000u;
            cpu->pc = cia + 4; return true;
        }
        if (xo == 33 || xo == 129 || xo == 193 || xo == 225 ||
            xo == 257 || xo == 289 || xo == 417 || xo == 449) {
            // cr logicals (33 crnor ... 449 cror); rc must be 0 (checked below)
            if (rc) break;
            u32 d = R_RD(raw), a = R_RA(raw), b = R_RB(raw);
            u32 av = (cpu->cr >> (31u - a)) & 1u, bv = (cpu->cr >> (31u - b)) & 1u, v = 0;
            switch (xo) {
            case 33: v = ~(av | bv); break;
            case 129: v = av & ~bv; break;
            case 193: v = av ^ bv; break;
            case 225: v = ~(av & bv); break;
            case 257: v = av & bv; break;
            case 289: v = ~(av ^ bv); break;
            case 417: v = av | ~bv; break;
            case 449: v = av | bv; break;
            default: v = av ^ bv; break; // crxor shares 193+256 path
            }
            u32 m = 0x80000000u >> d;
            cpu->cr = (cpu->cr & ~m) | ((v & 1u) ? m : 0u);
            cpu->pc = cia + 4; return true;
        }
        break;
    }
    case 20: { // rlwimi
        u32 rot = ri_rotl32(cpu->gpr[R_RS(raw)], (raw >> 11) & 0x1F);
        u32 m = ri_mask32((raw >> 6) & 0x1F, (raw >> 1) & 0x1F);
        cpu->gpr[R_RA(raw)] = (cpu->gpr[R_RA(raw)] & ~m) | (rot & m);
        if (rc) ri_set_cr0(cpu, cpu->gpr[R_RA(raw)]);
        cpu->pc = cia + 4; return true;
    }
    case 21: { // rlwinm
        u32 r = ri_rotl32(cpu->gpr[R_RS(raw)], (raw >> 11) & 0x1F)
                & ri_mask32((raw >> 6) & 0x1F, (raw >> 1) & 0x1F);
        cpu->gpr[R_RA(raw)] = r;
        if (rc) ri_set_cr0(cpu, r);
        cpu->pc = cia + 4; return true;
    }
    case 23: { // rlwnm
        u32 r = ri_rotl32(cpu->gpr[R_RS(raw)], cpu->gpr[R_RB(raw)] & 31u)
                & ri_mask32((raw >> 6) & 0x1F, (raw >> 1) & 0x1F);
        cpu->gpr[R_RA(raw)] = r;
        if (rc) ri_set_cr0(cpu, r);
        cpu->pc = cia + 4; return true;
    }
    case 24: // ori
        cpu->gpr[R_RA(raw)] = cpu->gpr[R_RS(raw)] | R_UIMM(raw);
        cpu->pc = cia + 4; return true;
    case 25: // oris
        cpu->gpr[R_RA(raw)] = cpu->gpr[R_RS(raw)] | ((u32)R_UIMM(raw) << 16);
        cpu->pc = cia + 4; return true;
    case 26: // xori
        cpu->gpr[R_RA(raw)] = cpu->gpr[R_RS(raw)] ^ R_UIMM(raw);
        cpu->pc = cia + 4; return true;
    case 27: // xoris
        cpu->gpr[R_RA(raw)] = cpu->gpr[R_RS(raw)] ^ ((u32)R_UIMM(raw) << 16);
        cpu->pc = cia + 4; return true;
    case 28: // andi.
        cpu->gpr[R_RA(raw)] = cpu->gpr[R_RS(raw)] & R_UIMM(raw);
        ri_set_cr0(cpu, cpu->gpr[R_RA(raw)]);
        cpu->pc = cia + 4; return true;
    case 29: // andis.
        cpu->gpr[R_RA(raw)] = cpu->gpr[R_RS(raw)] & ((u32)R_UIMM(raw) << 16);
        ri_set_cr0(cpu, cpu->gpr[R_RA(raw)]);
        cpu->pc = cia + 4; return true;
    case 31: {
        u32 xo = R_XO(raw), oe = (raw >> 10) & 1u;
        u32 a = cpu->gpr[R_RA(raw)], b = cpu->gpr[R_RB(raw)], s = cpu->gpr[R_RS(raw)];
        switch (xo) {
        case 0: // cmp
            if ((raw >> 21) & 1u) break;
            ri_cmp_s32(cpu, (raw >> 23) & 7u, (s32)a, (s32)b);
            cpu->pc = cia + 4; return true;
        case 4: { // tw
            if (rc) break;
            if (ppc_trap_condition((u8)R_RD(raw), a, b)) {
                ppc_program_exception(cpu, PPC_PROGRAM_TRAP, cia);
                return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 8: case 10: case 40: case 136: case 138: case 200: case 202:
        case 232: case 234: case 235: case 266: case 459: case 491:
        case 520: case 522: case 552: case 648: case 650: case 712: case 714:
        case 744: case 746: case 747: case 778: case 971: case 1003: {
            // arithmetic XO subset (mirrors emitter: CA via bit29, OV via helper)
            // NOTE: OE variants are base+512 (decoder: xo9 = xo & 0x1FF + oe
            // flag), so mask before dispatch — else addo/mullwo/etc miss.
            u64 wide = 0; u32 res = 0, ov = 0;
            u32 xob = xo & 0x1FFu;
            switch (xob) {
            case 266: res = a + b; wide = (u64)a + b;
                // add: NO CA touch (mirrors emitter: plain a+b, no xer write;
                // the interpreter previously cleared CA — wrong).
                ov = ppc_add_overflowed(a, b, res); break;
            case 10: wide = (u64)a + b; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(wide >> 32) & 1u) << 29);
                ov = ppc_add_overflowed(a, b, res); break;
            case 138: { u32 ca = (cpu->xer >> 29) & 1u;
                wide = (u64)a + b + ca; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(wide >> 32) & 1u) << 29);
                ov = ppc_add_overflowed(a, b, res); break; }
            case 202: { u32 ca = (cpu->xer >> 29) & 1u; // addze
                wide = (u64)a + ca; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(wide >> 32) & 1u) << 29);
                ov = ppc_add_overflowed(a, 0u, res); break; }
            case 234: { u32 ca = (cpu->xer >> 29) & 1u; // addme
                wide = (u64)a + 0xFFFFFFFFull + ca; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | ((wide >> 32) ? 0x20000000u : 0u);
                ov = ppc_add_overflowed(a, 0xFFFFFFFFu, res); break; }
            case 40: res = b - a; ov = ppc_add_overflowed(~a, b, res); break; // subf
            case 8: wide = (u64)b + (u64)(~a) + 1u; res = (u32)wide; // subfc
                cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(wide >> 32) & 1u) << 29);
                ov = ppc_add_overflowed(~a, b, res); break;
            case 136: { u32 ca = (cpu->xer >> 29) & 1u; // subfe
                wide = (u64)(~a) + b + ca; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(wide >> 32) & 1u) << 29);
                ov = ppc_add_overflowed(~a, b, res); break; }
            case 200: { u32 ca = (cpu->xer >> 29) & 1u; // subfze
                wide = (u64)(~a) + ca; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | (((u32)(wide >> 32) & 1u) << 29);
                ov = ppc_add_overflowed(~a, 0u, res); break; }
            case 232: { u32 ca = (cpu->xer >> 29) & 1u; // subfme
                wide = (u64)(~a) + 0xFFFFFFFFull + ca; res = (u32)wide;
                cpu->xer = (cpu->xer & ~0x20000000u) | ((wide >> 32) ? 0x20000000u : 0u);
                ov = ppc_add_overflowed(~a, 0xFFFFFFFFu, res); break; }
            case 235: { // mullw rD,rA,rB (fzEYzb160: was s*b using
                // RS==RD old value — must be RA*RB like add/sub/div)
                s64 p = (s64)(s32)a * (s64)(s32)b; res = (u32)p;
                ov = (p < -0x80000000ll || p > 0x7FFFFFFFll); break; }
            case 459: { // divwu (emitter still sets OV on div-by-0, even oe=0 path computes it)
                if (b == 0) res = 0; else res = a / b;
                ov = (b == 0); break; }
            default: { // divw (491)
                bool dov = (b == 0) || (a == 0x80000000u && b == 0xFFFFFFFFu);
                if (b == 0) res = ((s32)a < 0) ? 0xFFFFFFFFu : 0u;
                else if (a == 0x80000000u && b == 0xFFFFFFFFu) res = 0x80000000u;
                else res = (u32)((s32)a / (s32)b);
                ov = dov; break; }
            }
            cpu->gpr[R_RD(raw)] = res;
            if (oe) ppc_set_xer_ov(cpu, ov != 0);
            if (rc) ri_set_cr0(cpu, res);
            cpu->pc = cia + 4; return true;
        }
        case 11: // mulhwu
            cpu->gpr[R_RD(raw)] = (u32)(((u64)a * b) >> 32);
            if (rc) ri_set_cr0(cpu, cpu->gpr[R_RD(raw)]);
            cpu->pc = cia + 4; return true;
        case 19: // mfcr
            if (R_RA(raw) || R_RB(raw) || rc) break;
            cpu->gpr[R_RD(raw)] = cpu->cr;
            cpu->pc = cia + 4; return true;
        case 20: // lwarx
            if (rc) break;
            cpu->gpr[R_RD(raw)] = mem_read32(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)));
            if (cpu->exception) return true;
            cpu->reserve_addr = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            cpu->reserve_valid = true;
            cpu->pc = cia + 4; return true;
        case 23: // lwzx
            cpu->gpr[R_RD(raw)] = mem_read32(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 24: { // slw
            u32 sh = b & 0x3Fu;
            cpu->gpr[R_RA(raw)] = sh > 31 ? 0u : (s << sh);
            if (rc) ri_set_cr0(cpu, cpu->gpr[R_RA(raw)]);
            cpu->pc = cia + 4; return true;
        }
        case 26: { // cntlzw
            u32 v = s, n = 0;
            while (n < 32 && ((v & (0x80000000u >> n)) == 0)) n++;
            cpu->gpr[R_RA(raw)] = n;
            if (rc) ri_set_cr0(cpu, n);
            cpu->pc = cia + 4; return true;
        }
        case 28: // and
            cpu->gpr[R_RA(raw)] = s & b;
            if (rc) ri_set_cr0(cpu, s & b);
            cpu->pc = cia + 4; return true;
        case 32: // cmpl
            if ((raw >> 21) & 1u) break;
            ri_cmp_u32(cpu, (raw >> 23) & 7u, a, b);
            cpu->pc = cia + 4; return true;
        case 54: // dcbst (no-op on host)
            if (R_RD(raw) || rc) break;
            cpu->pc = cia + 4; return true;
        case 55: // lwzux
            if (rc || R_RA(raw) == 0 || R_RA(raw) == R_RD(raw)) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            cpu->gpr[R_RD(raw)] = mem_read32(cpu, cpu->gpr[R_RA(raw)]);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 60: // andc
            cpu->gpr[R_RA(raw)] = s & ~b;
            if (rc) ri_set_cr0(cpu, s & ~b);
            cpu->pc = cia + 4; return true;
        case 75: // mulhw
            cpu->gpr[R_RD(raw)] = (u32)(((s64)(s32)a * (s64)(s32)b) >> 32);
            if (rc) ri_set_cr0(cpu, cpu->gpr[R_RD(raw)]);
            cpu->pc = cia + 4; return true;
        case 83: // mfmsr
            if (R_RA(raw) || R_RB(raw) || rc) break;
            cpu->gpr[R_RD(raw)] = cpu->msr;
            cpu->pc = cia + 4; return true;
        case 86: // dcbf (no-op on host)
            if (R_RD(raw) || rc) break;
            cpu->pc = cia + 4; return true;
        case 87: // lbzx
            cpu->gpr[R_RD(raw)] = mem_read8(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 104: // neg
            if (R_RB(raw)) break;
            cpu->gpr[R_RD(raw)] = (~a) + 1u;
            if (oe) ppc_set_xer_ov(cpu, a == 0x80000000u);
            if (rc) ri_set_cr0(cpu, cpu->gpr[R_RD(raw)]);
            cpu->pc = cia + 4; return true;
        case 119: // lbzux
            if (rc || R_RA(raw) == 0) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            cpu->gpr[R_RD(raw)] = mem_read8(cpu, cpu->gpr[R_RA(raw)]);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 124: // nor
            cpu->gpr[R_RA(raw)] = ~(s | b);
            if (rc) ri_set_cr0(cpu, ~(s | b));
            cpu->pc = cia + 4; return true;
        case 144: { // mtcrf
            if ((raw & ((1u << 20) | 1u))) break;
            u32 m = 0, crm = (raw >> 12) & 0xFFu;
            for (u32 f = 0; f < 8; f++)
                if (crm & (0x80u >> f)) m |= 0xFu << (4u * (7u - f));
            cpu->cr = (cpu->cr & ~m) | (s & m);
            cpu->pc = cia + 4; return true;
        }
        case 146: // mtmsr
            if (R_RA(raw) || R_RB(raw) || rc) break;
            cpu->msr = s;
            cpu->pc = cia + 4; return true;
        case 150: { // stwcx. (decoder: rc must be 1)
            if (!rc) break;
            u32 ea = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            bool ok = cpu->reserve_valid;
            cpu->reserve_valid = false;
            if (ok) { mem_write32(cpu, ea, s); if (cpu->exception) return true; }
            cpu->cr = (cpu->cr & 0x0FFFFFFFu) | ((ok ? 2u : 0u) << 28)
                    | ((cpu->xer >> 3) & 0x10000000u);
            cpu->pc = cia + 4; return true;
        }
        case 151: // stwx
            mem_write32(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), s);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 183: // stwux
            if (R_RA(raw) == 0) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            mem_write32(cpu, cpu->gpr[R_RA(raw)], s);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 215: // stbx
            mem_write8(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), (u8)s);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 246: case 278: // dcbtst / dcbt (no-op)
            if (R_RD(raw) || rc) break;
            cpu->pc = cia + 4; return true;
        case 247: // stbux
            if (R_RA(raw) == 0) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            mem_write8(cpu, cpu->gpr[R_RA(raw)], (u8)s);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 279: // lhzx
            cpu->gpr[R_RD(raw)] = mem_read16(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 284: // eqv
            cpu->gpr[R_RA(raw)] = ~(s ^ b);
            if (rc) ri_set_cr0(cpu, ~(s ^ b));
            cpu->pc = cia + 4; return true;
        case 316: // xor
            cpu->gpr[R_RA(raw)] = s ^ b;
            if (rc) ri_set_cr0(cpu, s ^ b);
            cpu->pc = cia + 4; return true;
        case 339: case 467: { // mfspr / mtspr (LR/CTR/XER modeled; rest via cpu.c)
            u32 spr = R_SPR(raw);
            if (xo == 339) {
                u32 v;
                if (spr == 8) v = cpu->lr;
                else if (spr == 9) v = cpu->ctr;
                else if (spr == 1) v = cpu->xer;
                else { u32 bef = cpu->exception;
                    v = ppc_mfspr(cpu, (u16)spr, cia);
                    if (cpu->exception != bef) return true; }
                cpu->gpr[R_RD(raw)] = v;
            } else {
                if (spr == 8) cpu->lr = s;
                else if (spr == 9) cpu->ctr = s;
                else if (spr == 1) cpu->xer = s;
                else { ppc_mtspr(cpu, (u16)spr, s, cia);
                    if (cpu->exception) return true; }
            }
            cpu->pc = cia + 4; return true;
        }
        case 343: // lhax
            cpu->gpr[R_RD(raw)] = (u32)(s32)(s16)mem_read16(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 371: // mftb
            cpu->gpr[R_RD(raw)] = ppc_mftb(cpu, (u16)R_SPR(raw), cia);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 407: // sthx
            mem_write16(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), (u16)s);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 444: // or
            cpu->gpr[R_RA(raw)] = s | b;
            if (rc) ri_set_cr0(cpu, s | b);
            cpu->pc = cia + 4; return true;
        case 534: // lwbrx
            cpu->gpr[R_RD(raw)] = bswap32(mem_read32(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw))));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 210: // mtsr
            if (((raw >> 20) & 1u) || R_RB(raw) || rc) break;
            cpu->sr[R_RA(raw) & 0xFu] = s;
            cpu->pc = cia + 4; return true;
        case 242: // mtsrin
            if (R_RA(raw) || rc) break;
            cpu->sr[(cpu->gpr[R_RB(raw)] >> 28) & 0xFu] = s;
            cpu->pc = cia + 4; return true;
        case 306: // tlbie
            if (R_RD(raw) || R_RA(raw) || rc) break;
            ppc_tlbie(cpu, b, cia);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 310: // eciwx
            if (rc) break;
            cpu->gpr[R_RD(raw)] = ppc_eciwx(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), cia);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 311: // lhzux
            if (rc || R_RA(raw) == 0 || R_RA(raw) == R_RD(raw)) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            cpu->gpr[R_RD(raw)] = mem_read16(cpu, cpu->gpr[R_RA(raw)]);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 375: // lhaux
            if (rc || R_RA(raw) == 0 || R_RA(raw) == R_RD(raw)) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            cpu->gpr[R_RD(raw)] = (u32)(s32)(s16)mem_read16(cpu, cpu->gpr[R_RA(raw)]);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 412: // orc
            cpu->gpr[R_RA(raw)] = s | ~b;
            if (rc) ri_set_cr0(cpu, s | ~b);
            cpu->pc = cia + 4; return true;
        case 438: // ecowx
            if (rc) break;
            ppc_ecowx(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), s, cia);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 439: // sthux
            if (rc || R_RA(raw) == 0) break;
            cpu->gpr[R_RA(raw)] = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            mem_write16(cpu, cpu->gpr[R_RA(raw)], (u16)s);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 470: // dcbi (no-op on host, mirrors emitter)
            if (R_RD(raw) || rc) break;
            cpu->pc = cia + 4; return true;
        case 476: // nand
            cpu->gpr[R_RA(raw)] = ~(s & b);
            if (rc) ri_set_cr0(cpu, ~(s & b));
            cpu->pc = cia + 4; return true;
        case 533: { // lswx
            if (rc) break;
            if (ri_in_wrapped_range(R_RD(raw), ri_string_count(cpu->xer & 0x7Fu),
                                    R_RA(raw)) ||
                ri_in_wrapped_range(R_RD(raw), ri_string_count(cpu->xer & 0x7Fu),
                                    R_RB(raw))) {
                ppc_program_exception(cpu, PPC_PROGRAM_ILLEGAL, cia);
                return true;
            }
            u32 ea = cpu->gpr[R_RB(raw)] + (R_RA(raw) ? cpu->gpr[R_RA(raw)] : 0u);
            u32 count = cpu->xer & 0x7Fu;
            for (u32 n = 0; n < count; n++) {
                u32 reg = (R_RD(raw) + n / 4u) & 31u;
                if ((n & 3u) == 0) cpu->gpr[reg] = 0;
                cpu->gpr[reg] |= (u32)mem_read8(cpu, ea + n) << (24u - 8u * (n & 3u));
                if (cpu->exception) return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 535: case 599: { // lfsx / lfdx (decoder: rc must be 0)
            if (rc) break;
            if (!ppc_fp_available(cpu, cia)) return true;
            u32 ea = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            if (xo == 535) {
                f64 v = (f64)ri_f32_from_bits(mem_read32(cpu, ea));
                if (cpu->exception) return true;
                cpu->fpr[R_RD(raw)] = v; cpu->ps1[R_RD(raw)] = v;
            } else {
                cpu->fpr[R_RD(raw)] = ri_f64_from_bits(mem_read64(cpu, ea));
                if (cpu->exception) return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 567: case 631: { // lfsux / lfdux (decoder: rc=0, rA!=0)
            if (rc || R_RA(raw) == 0) break;
            if (!ppc_fp_available(cpu, cia)) return true;
            u32 ea = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            if (xo == 567) {
                f64 v = (f64)ri_f32_from_bits(mem_read32(cpu, ea));
                if (cpu->exception) return true;
                cpu->fpr[R_RD(raw)] = v; cpu->ps1[R_RD(raw)] = v;
            } else {
                cpu->fpr[R_RD(raw)] = ri_f64_from_bits(mem_read64(cpu, ea));
                if (cpu->exception) return true;
            }
            cpu->gpr[R_RA(raw)] = ea;
            cpu->pc = cia + 4; return true;
        }
        case 595: // mfsr
            if (((raw >> 20) & 1u) || R_RB(raw) || rc) break;
            cpu->gpr[R_RD(raw)] = cpu->sr[R_RA(raw) & 0xFu];
            cpu->pc = cia + 4; return true;
        case 597: { // lswi
            if (rc) break;
            u32 nb = R_RB(raw), count = nb ? nb : 32u;
            if (ri_in_wrapped_range(R_RD(raw), ri_string_count(nb), R_RA(raw))) {
                ppc_program_exception(cpu, PPC_PROGRAM_ILLEGAL, cia);
                return true;
            }
            u32 ea = R_RA(raw) ? cpu->gpr[R_RA(raw)] : 0u;
            for (u32 n = 0; n < count; n++) {
                u32 reg = (R_RD(raw) + n / 4u) & 31u;
                if ((n & 3u) == 0) cpu->gpr[reg] = 0;
                cpu->gpr[reg] |= (u32)mem_read8(cpu, ea + n) << (24u - 8u * (n & 3u));
                if (cpu->exception) return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 598: // sync
            if (raw != 0x7C0004ACu) break;
            ppc_memory_fence();
            cpu->pc = cia + 4; return true;
        case 659: // mfsrin
            if (R_RA(raw) || rc) break;
            cpu->gpr[R_RD(raw)] = cpu->sr[(cpu->gpr[R_RB(raw)] >> 28) & 0xFu];
            cpu->pc = cia + 4; return true;
        case 661: { // stswx
            if (rc) break;
            u32 ea = cpu->gpr[R_RB(raw)] + (R_RA(raw) ? cpu->gpr[R_RA(raw)] : 0u);
            u32 count = cpu->xer & 0x7Fu;
            for (u32 n = 0; n < count; n++) {
                u32 reg = (R_RS(raw) + n / 4u) & 31u;
                mem_write8(cpu, ea + n,
                           (u8)(cpu->gpr[reg] >> (24u - 8u * (n & 3u))));
                if (cpu->exception) return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 663: case 727: { // stfsx / stfdx (decoder: rc must be 0)
            if (rc) break;
            if (!ppc_fp_available(cpu, cia)) return true;
            u32 ea = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            if (xo == 663)
                mem_write32(cpu, ea, ri_f32_to_bits((f32)cpu->fpr[R_RS(raw)]));
            else
                mem_write64(cpu, ea, ri_f64_to_bits(cpu->fpr[R_RS(raw)]));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        }
        case 695: case 759: { // stfsux / stfdux (decoder: rc=0, rA!=0)
            if (rc || R_RA(raw) == 0) break;
            if (!ppc_fp_available(cpu, cia)) return true;
            u32 ea = ri_ea_x(cpu, R_RA(raw), R_RB(raw));
            if (xo == 695)
                mem_write32(cpu, ea, ri_f32_to_bits((f32)cpu->fpr[R_RS(raw)]));
            else
                mem_write64(cpu, ea, ri_f64_to_bits(cpu->fpr[R_RS(raw)]));
            if (cpu->exception) return true;
            cpu->gpr[R_RA(raw)] = ea;
            cpu->pc = cia + 4; return true;
        }
        case 725: { // stswi
            if (rc) break;
            u32 ea = R_RA(raw) ? cpu->gpr[R_RA(raw)] : 0u;
            u32 nb = R_RB(raw), count = nb ? nb : 32u;
            for (u32 n = 0; n < count; n++) {
                u32 reg = (R_RS(raw) + n / 4u) & 31u;
                mem_write8(cpu, ea + n,
                           (u8)(cpu->gpr[reg] >> (24u - 8u * (n & 3u))));
                if (cpu->exception) return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 854: // eieio
            if (raw != 0x7C0006ACu) break;
            ppc_memory_fence();
            cpu->pc = cia + 4; return true;
        case 566: // tlbsync
            if (raw != 0x7C00046Cu) break;
            ppc_memory_fence();
            cpu->pc = cia + 4; return true;
        case 982: // icbi (no-op-ish: route via fallback like the emitter)
            if (R_RD(raw) || rc) break;
            ppc_fallback_instruction(cpu, raw, cia);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 983: // stfiwx (decoder: rc must be 0)
            if (rc) break;
            if (!ppc_fp_available(cpu, cia)) return true;
            mem_write32(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)),
                        (u32)ri_f64_to_bits(cpu->fpr[R_RS(raw)]));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 1014: { // dcbz (RD==0, rc==0; dcbz_l lives in prim 4 XO 1014)
            if (R_RD(raw) || rc) break;
            u32 ea = ri_ea_x(cpu, R_RA(raw), R_RB(raw)) & ~31u;
            for (u32 i = 0; i < 32; i += 4) {
                mem_write32(cpu, ea + i, 0);
                if (cpu->exception) return true;
            }
            cpu->pc = cia + 4; return true;
        }
        case 918: // sthbrx
            mem_write16(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)),
                        bswap16((u16)s));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 536: { // srw
            u32 sh = b & 0x3Fu;
            cpu->gpr[R_RA(raw)] = sh > 31 ? 0u : (s >> sh);
            if (rc) ri_set_cr0(cpu, cpu->gpr[R_RA(raw)]);
            cpu->pc = cia + 4; return true;
        }
        case 662: // stwbrx
            mem_write32(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw)), bswap32(s));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 790: // lhbrx
            cpu->gpr[R_RD(raw)] = bswap16(mem_read16(cpu, ri_ea_x(cpu, R_RA(raw), R_RB(raw))));
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 792: case 824: { // sraw / srawi
            u32 sh = xo == 824 ? ((raw >> 11) & 0x1F) : (b & 0x3Fu);
            u32 v = (u32)s, ca = 0, r;
            if (sh == 0) r = v;
            else if (sh > 31) { r = (v & 0x80000000u) ? 0xFFFFFFFFu : 0u; ca = (v & 0x80000000u) != 0; }
            else { r = (u32)((s32)v >> sh); ca = (v & 0x80000000u) && ((v << (32u - sh)) != 0); }
            cpu->gpr[R_RA(raw)] = r;
            cpu->xer = (cpu->xer & ~0x20000000u) | (ca ? 0x20000000u : 0u);
            if (rc) ri_set_cr0(cpu, r);
            cpu->pc = cia + 4; return true;
        }
        case 922: // extsh
            cpu->gpr[R_RA(raw)] = (u32)(s32)(s16)s;
            if (rc) ri_set_cr0(cpu, (u32)(s32)(s16)s);
            cpu->pc = cia + 4; return true;
        case 954: // extsb
            cpu->gpr[R_RA(raw)] = (u32)(s32)(s8)s;
            if (rc) ri_set_cr0(cpu, (u32)(s32)(s8)s);
            cpu->pc = cia + 4; return true;
        default: break;
        }
        // XO31 miss falls through to unsupported log below.
        { static unsigned _n = 0;
          if (_n < 8) { _n++;
            fprintf(stderr, "[relinterp] no cover XO31=%u raw=0x%08X @0x%08X\n",
                    R_XO(raw), raw, cia); }
          return false; }
    }
    case 32: // lwz
        cpu->gpr[R_RD(raw)] = mem_read32(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)));
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 33: // lwzu
        if (R_RA(raw) == 0 || R_RA(raw) == R_RD(raw)) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        cpu->gpr[R_RD(raw)] = mem_read32(cpu, cpu->gpr[R_RA(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 34: // lbz
        cpu->gpr[R_RD(raw)] = mem_read8(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)));
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 35: // lbzu
        if (R_RA(raw) == 0) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        cpu->gpr[R_RD(raw)] = mem_read8(cpu, cpu->gpr[R_RA(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 36: // stw
        mem_write32(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)), cpu->gpr[R_RS(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 37: // stwu
        if (R_RA(raw) == 0) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        mem_write32(cpu, cpu->gpr[R_RA(raw)], cpu->gpr[R_RS(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 38: // stb
        mem_write8(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)), (u8)cpu->gpr[R_RS(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 39: // stbu
        if (R_RA(raw) == 0) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        mem_write8(cpu, cpu->gpr[R_RA(raw)], (u8)cpu->gpr[R_RS(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 40: // lhz
        cpu->gpr[R_RD(raw)] = mem_read16(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)));
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 41: // lhzu
        if (R_RA(raw) == 0 || R_RA(raw) == R_RD(raw)) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        cpu->gpr[R_RD(raw)] = mem_read16(cpu, cpu->gpr[R_RA(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 42: // lha
        cpu->gpr[R_RD(raw)] = (u32)(s32)(s16)mem_read16(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)));
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 43: // lhau
        if (R_RA(raw) == 0 || R_RA(raw) == R_RD(raw)) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        cpu->gpr[R_RD(raw)] = (u32)(s32)(s16)mem_read16(cpu, cpu->gpr[R_RA(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 44: // sth
        mem_write16(cpu, ri_ea_d(cpu, R_RA(raw), R_SIMM(raw)), (u16)cpu->gpr[R_RS(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 45: // sthu
        if (R_RA(raw) == 0) break;
        cpu->gpr[R_RA(raw)] = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        mem_write16(cpu, cpu->gpr[R_RA(raw)], (u16)cpu->gpr[R_RS(raw)]);
        if (cpu->exception) return true;
        cpu->pc = cia + 4; return true;
    case 46: { // lmw
        u32 ea = R_RA(raw) == 0 ? (u32)(s32)R_SIMM(raw)
                                : cpu->gpr[R_RA(raw)] + (u32)(s32)R_SIMM(raw);
        for (u32 r = R_RD(raw); r < 32; r++, ea += 4) {
            cpu->gpr[r] = mem_read32(cpu, ea);
            if (cpu->exception) return true;
        }
        cpu->pc = cia + 4; return true;
    }
    case 47: { // stmw
        u32 ea = R_RA(raw) == 0 ? (u32)(s32)R_SIMM(raw)
                                : cpu->gpr[R_RA(raw)] + (u32)(s32)R_SIMM(raw);
        for (u32 r = R_RS(raw); r < 32; r++, ea += 4) {
            mem_write32(cpu, ea, cpu->gpr[r]);
            if (cpu->exception) return true;
        }
        cpu->pc = cia + 4; return true;
    }
    case 48: case 49: { // lfs / lfsu (lfsu: rA!=0 per decoder)
        if (prim == 49 && R_RA(raw) == 0) break;
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 ea = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        f64 v = (f64)ri_f32_from_bits(mem_read32(cpu, ea));
        if (cpu->exception) return true;
        cpu->fpr[R_RD(raw)] = v; cpu->ps1[R_RD(raw)] = v;
        if (prim == 49) cpu->gpr[R_RA(raw)] = ea;
        cpu->pc = cia + 4; return true;
    }
    case 50: case 51: { // lfd / lfdu (lfdu: rA!=0 per decoder)
        if (prim == 51 && R_RA(raw) == 0) break;
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 ea = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        cpu->fpr[R_RD(raw)] = ri_f64_from_bits(mem_read64(cpu, ea));
        if (cpu->exception) return true;
        if (prim == 51) cpu->gpr[R_RA(raw)] = ea;
        cpu->pc = cia + 4; return true;
    }
    case 52: case 53: { // stfs / stfsu (stfsu: rA!=0 per decoder)
        if (prim == 53 && R_RA(raw) == 0) break;
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 ea = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        mem_write32(cpu, ea, ri_f32_to_bits((f32)cpu->fpr[R_RS(raw)]));
        if (cpu->exception) return true;
        if (prim == 53) cpu->gpr[R_RA(raw)] = ea;
        cpu->pc = cia + 4; return true;
    }
    case 54: case 55: { // stfd / stfdu (stfdu: rA!=0 per decoder)
        if (prim == 55 && R_RA(raw) == 0) break;
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 ea = ri_ea_d(cpu, R_RA(raw), R_SIMM(raw));
        mem_write64(cpu, ea, ri_f64_to_bits(cpu->fpr[R_RS(raw)]));
        if (cpu->exception) return true;
        if (prim == 55) cpu->gpr[R_RA(raw)] = ea;
        cpu->pc = cia + 4; return true;
    }
    case 56: case 57: { // psq_l / psq_lu (psq_lu: rA!=0 per decoder)
        if (prim == 57 && R_RA(raw) == 0) break;
        if (!ppc_fp_available(cpu, cia)) return true;
        s32 off = ri_psq_simm(raw);
        u32 ea = ri_ea_psq(cpu, R_RA(raw), off);
        if (!ppc_psq_load(cpu, (u8)R_RD(raw), ea, (raw >> 15) & 1u,
                          (u8)((raw >> 12) & 7u), false, cia)) return true;
        if (prim == 57) cpu->gpr[R_RA(raw)] = ea;
        cpu->pc = cia + 4; return true;
    }
    case 59: { // single-precision scalar ALU (A-form sub-op in bits 1..5)
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 ax = R_AXO(raw);
        u32 d = R_RD(raw), a = R_RA(raw), b = R_RB(raw), c = R_RC(raw);
        switch (ax) {
        case 18: cpu->fpr[d] = cpu->ps1[d] = (f64)(f32)(cpu->fpr[a] / cpu->fpr[b]);
            ri_fprf_s(cpu, (f32)cpu->fpr[d]); break; // fdivs
        case 20: cpu->fpr[d] = cpu->ps1[d] = (f64)(f32)(cpu->fpr[a] - cpu->fpr[b]);
            ri_fprf_s(cpu, (f32)cpu->fpr[d]); break; // fsubs
        case 21: cpu->fpr[d] = cpu->ps1[d] = (f64)(f32)(cpu->fpr[a] + cpu->fpr[b]);
            ri_fprf_s(cpu, (f32)cpu->fpr[d]); break; // fadds
        case 24: // fres (rA==0, rC==0)
            if (a || c) return ri_miss_sub(24, raw, cia, "f59");
            { f64 r; if (!ppc_fres(cpu, cpu->fpr[b], &r)) return true;
              cpu->fpr[d] = cpu->ps1[d] = r; }
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 25: cpu->fpr[d] = cpu->ps1[d] = (f64)(f32)(cpu->fpr[a] * cpu->fpr[c]);
            ri_fprf_s(cpu, (f32)cpu->fpr[d]); break; // fmuls
        case 28: case 29: case 30: case 31: { // fmsubs/fmadds/fnmsubs/fnmadds
            bool sub = (ax == 28 || ax == 30), neg = (ax >= 30);
            f64 r;
            if (!ppc_fma(cpu, cpu->fpr[a], cpu->fpr[c], cpu->fpr[b],
                         true, sub, neg, &r)) return true;
            cpu->fpr[d] = cpu->ps1[d] = r;
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true; }
        default: return ri_miss_sub(ax, raw, cia, "f59");
        }
        cpu->pc = cia + 4; return true;
    }
    case 60: case 61: { // psq_st / psq_stu (psq_stu: rA!=0 per decoder)
        if (prim == 61 && R_RA(raw) == 0) break;
        if (!ppc_fp_available(cpu, cia)) return true;
        s32 off = ri_psq_simm(raw);
        u32 ea = ri_ea_psq(cpu, R_RA(raw), off);
        if (!ppc_psq_store(cpu, (u8)R_RS(raw), ea, (raw >> 15) & 1u,
                           (u8)((raw >> 12) & 7u), false, cia)) return true;
        if (prim == 61) cpu->gpr[R_RA(raw)] = ea;
        cpu->pc = cia + 4; return true;
    }
    case 63: { // double ALU + compares + FPSCR moves (decoder XO lattice)
        if (!ppc_fp_available(cpu, cia)) return true;
        u32 xo = R_XO(raw);
        u32 d = R_RD(raw), a = R_RA(raw), b = R_RB(raw), c = R_RC(raw);
        switch (xo) {
        case 0: ri_fcompare(cpu, (raw >> 23) & 7u, cpu->fpr[a], cpu->fpr[b]);
            cpu->pc = cia + 4; return true; // fcmpu
        case 32: ri_fcompare(cpu, (raw >> 23) & 7u, cpu->fpr[a], cpu->fpr[b]);
            cpu->pc = cia + 4; return true; // fcmpo
        case 12: cpu->fpr[d] = cpu->ps1[d] = (f64)(f32)cpu->fpr[b]; // frsp
            ri_fprf_s(cpu, (f32)cpu->fpr[d]);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 14: // fctiw (rA must be 0)
            if (a) return ri_miss_sub(14, raw, cia, "f63");
            { u64 r; if (!ppc_fctiw(cpu, cpu->fpr[b], false, &r)) return true;
              cpu->fpr[d] = ri_f64_from_bits(r); }
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 15: // fctiwz (rA must be 0)
            if (a) return ri_miss_sub(15, raw, cia, "f63");
            { u64 r; if (!ppc_fctiw(cpu, cpu->fpr[b], true, &r)) return true;
              cpu->fpr[d] = ri_f64_from_bits(r); }
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 38: case 70: { // mtfsb1 / mtfsb0 (rA==0, rB==0 per decoder)
            if (a || b) return ri_miss_sub(xo, raw, cia, "f63");
            if (d >= 15u && d <= 19u) ppc_fprf_drop();
            u32 m = 0x80000000u >> d;
            if (xo == 70) { if (d != 1 && d != 2) cpu->fpscr &= ~m; }
            else { if (d != 1 && d != 2) cpu->fpscr |= m; }
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; }
        case 40: cpu->fpr[d] = ri_f64_from_bits(ri_f64_to_bits(cpu->fpr[b]) ^ 0x8000000000000000ull);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; // fneg
        case 64: // mcrfs (masked fields must be 0)
            if (raw & ((3u << 21) | (0x7Fu << 11) | 1u)) break;
            { u32 ss = 4u * (7u - ((raw >> 18) & 7u)), ds = 4u * (7u - ((raw >> 23) & 7u));
              ppc_fprf_flush(cpu);
              u32 f = (cpu->fpscr >> ss) & 0xFu;
              cpu->fpscr &= ~((0xFu << ss) & 0x83F80700u);
              ppc_fpscr_updated(cpu);
              cpu->cr = (cpu->cr & ~(0xFu << ds)) | (f << ds); }
            cpu->pc = cia + 4; return true;
        case 72: cpu->fpr[d] = cpu->fpr[b]; // fmr
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 134: // mtfsfi (masked fields must be 0)
            if (raw & ((0x7Fu << 16) | (1u << 11))) break;
            { u32 sh = 4u * (7u - d);
              ppc_fprf_drop();
              cpu->fpscr = (cpu->fpscr & ~(0xFu << sh)) | (((raw >> 12) & 0xFu) << sh);
              ppc_fpscr_updated(cpu); }
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 136: cpu->fpr[d] = ri_f64_from_bits(ri_f64_to_bits(cpu->fpr[b]) | 0x8000000000000000ull);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; // fnabs
        case 264: cpu->fpr[d] = ri_f64_from_bits(ri_f64_to_bits(cpu->fpr[b]) & 0x7FFFFFFFFFFFFFFFull);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; // fabs
        case 583: // mffs (rA==0, rB==0)
            if (a || b) return ri_miss_sub(583, raw, cia, "f63");
            ppc_fprf_flush(cpu);
            cpu->fpr[d] = ri_f64_from_bits(0xFFF8000000000000ull | cpu->fpscr);
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        case 711: // mtfsf (masked bits must be 0)
            if (raw & ((1u << 25) | (1u << 16))) break;
            { u32 m = 0, fm = (raw >> 17) & 0xFFu;
              for (u32 f = 0; f < 8; f++)
                  if (fm & (1u << f)) m |= 0xFu << (f * 4);
              u32 src = (u32)ri_f64_to_bits(cpu->fpr[b]);
              ppc_fprf_drop();
              cpu->fpscr = (cpu->fpscr & ~m) | (src & m);
              ppc_fpscr_updated(cpu); }
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true;
        default: break;
        }
        switch (R_AXO(raw)) { // A-form double ALU + fsel
        case 18: cpu->fpr[d] = cpu->fpr[a] / cpu->fpr[b];
            ri_fprf_d(cpu, cpu->fpr[d]); break; // fdiv
        case 20: cpu->fpr[d] = cpu->fpr[a] - cpu->fpr[b];
            ri_fprf_d(cpu, cpu->fpr[d]); break; // fsub
        case 21: cpu->fpr[d] = cpu->fpr[a] + cpu->fpr[b];
            ri_fprf_d(cpu, cpu->fpr[d]); break; // fadd
        case 23: cpu->fpr[d] = (cpu->fpr[a] >= 0.0) ? cpu->fpr[c] : cpu->fpr[b];
            if (rc) ri_set_cr1(cpu);
            cpu->pc = cia + 4; return true; // fsel
        case 25: cpu->fpr[d] = cpu->fpr[a] * cpu->fpr[c];
            ri_fprf_d(cpu, cpu->fpr[d]); break; // fmul
        case 26: // frsqrte (rA==0, rC==0)
            if (a || c) return ri_miss_sub(26, raw, cia, "f63");
            { f64 r; if (!ppc_frsqrte(cpu, cpu->fpr[b], &r)) return true;
              cpu->fpr[d] = r; }
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true;
        case 28: case 29: case 30: case 31: { // fmsub/fmadd/fnmsub/fnmadd
            bool sub = (R_AXO(raw) == 28 || R_AXO(raw) == 30);
            bool neg = (R_AXO(raw) >= 30);
            f64 r;
            if (!ppc_fma(cpu, cpu->fpr[a], cpu->fpr[c], cpu->fpr[b],
                         false, sub, neg, &r)) return true;
            cpu->fpr[d] = r;
            if (rc) ri_set_cr1(cpu);
            if (cpu->exception) return true;
            cpu->pc = cia + 4; return true; }
        default: return ri_miss_sub(R_AXO(raw) | 0x800u, raw, cia, "f63");
        }
        cpu->pc = cia + 4; return true;
    }
    default: break;
    }
    { static unsigned _m = 0;
      if (_m < 8) { _m++;
        fprintf(stderr, "[relinterp] no cover prim=%u raw=0x%08X @0x%08X\n",
                prim, raw, cia); }
      return false; }
}
