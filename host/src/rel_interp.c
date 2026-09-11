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
// integer subset) + branches + integer loads/stores + SPR LR/CTR/XER. Every
// emitted line mirrors vendor/RingOut/DolRecomp/src/backend/emitter.c
// semantics (CA/OV/RC/CR handling), so interp and recomp agree by construction.
// FP/PSQ (prims 48-63, prim 4) are NOT covered: step returns false with a
// bounded log naming the opcode, and the slice falls back to the old miss
// path. fze.sample.rel is integer-only, so it runs today; FP RELs are next.
//
// Interop with recomp dispatch: bl to a DOL address sets pc there and returns
// true — the next slice iteration dispatches the existing chunk. blr/bclr
// back to a DOL lr likewise resumes recomp code. Only heap->heap control
// stays in the interpreter.
#include "../../vendor/RingOut/DolRecomp/src/cpu/cpu.h"
#include <stdio.h>

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
        case 232: case 234: case 235: case 266: case 459: case 491: {
            // arithmetic XO subset (mirrors emitter: CA via bit29, OV via helper)
            u64 wide = 0; u32 res = 0, ov = 0;
            switch (xo) {
            case 266: res = a + b; wide = (u64)a + b;
                cpu->xer = (cpu->xer & ~0x20000000u) | 0u; // add: no CA
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
            case 459: { // divwu
                if (b == 0) res = 0; else res = a / b;
                ov = 0; break; }
            default: { // divw (491)
                if (b == 0) res = ((s32)a < 0) ? 0xFFFFFFFFu : 0u;
                else if (a == 0x80000000u && b == 0xFFFFFFFFu) res = 0x80000000u;
                else res = (u32)((s32)a / (s32)b);
                ov = (b != 0 && a == 0x80000000u && b == 0xFFFFFFFFu) ? 0 : 0; break; }
            }
            if (xo == 235 || xo == 266 || xo == 40) cpu->gpr[R_RD(raw)] = res;
            else if (xo == 459 || xo == 491) cpu->gpr[R_RD(raw)] = res;
            else cpu->gpr[R_RD(raw)] = res;
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
        case 535: case 599: case 631: case 663: case 727: break; // FP — no cover
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
    default: break;
    }
    { static unsigned _m = 0;
      if (_m < 8) { _m++;
        fprintf(stderr, "[relinterp] no cover prim=%u raw=0x%08X @0x%08X\n",
                prim, raw, cia); }
      return false; }
}
