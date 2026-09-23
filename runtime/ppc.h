// wiikit runtime — the Gekko/Broadway CPU as seen by recompiled code.
//
// Recompiled functions have the signature `void f_XXXXXXXX(PPCContext& c)`
// and touch guest state only through this header: registers in PPCContext,
// memory through ld*/st* (big-endian guest byte order, a flat 4 GiB host
// reservation at g_mem), and a handful of out-of-line services the runtime
// provides (indirect calls, system calls, traps, MMIO, time base).
//
// Semantics follow the 750CL manual; where Gekko differs from a plain
// PowerPC 750 (paired singles, single-precision results filling both
// halves of an FPR) the rules are Dolphin's interpreter's.
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>

#if defined(_MSC_VER) && !defined(__clang__)
#include <stdlib.h>
#define PPC_BSWAP16 _byteswap_ushort
#define PPC_BSWAP32 _byteswap_ulong
#define PPC_UNLIKELY(x) (x)
#define PPC_NOINLINE __declspec(noinline)
#else
#define PPC_BSWAP16 __builtin_bswap16
#define PPC_BSWAP32 __builtin_bswap32
#define PPC_UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PPC_NOINLINE __attribute__((noinline))
#endif

struct PPCContext {
    uint32_t r[32];
    double   f[32];     // ps0 (the FPR as a double)
    double   ps1[32];   // paired-single second half
    uint8_t  cr[32];    // one byte per CR bit: 4*field + {0 lt, 1 gt, 2 eq, 3 so}
    uint32_t lr, ctr;
    uint8_t  xer_so, xer_ov, xer_ca, xer_bc;
    uint32_t fpscr, msr;
    uint32_t gqr[8];
    uint32_t sr[16];
    uint32_t spr[1024]; // every other SPR, by number
    uint32_t reserve;   // lwarx reservation address
};

typedef void (*PPCFunc)(PPCContext&);

// ---- services provided by the runtime ------------------------------------------------
extern uint8_t* g_mem;
void     ppc_call_indirect(PPCContext& c, uint32_t addr);   // bctrl, blrl, indirect tail calls
void     ppc_syscall(PPCContext& c, uint32_t addr);
void     ppc_trap(PPCContext& c, uint32_t addr);
void     ppc_unimplemented(PPCContext& c, uint32_t addr, const char* what);
uint32_t ppc_mmio_read(uint32_t addr, int size);
void     ppc_mmio_write(uint32_t addr, uint32_t value, int size);
uint64_t ppc_timebase();
void     ppc_lswx(PPCContext& c, int rd, uint32_t ea, uint32_t n);
void     ppc_stswx(PPCContext& c, int rs, uint32_t ea, uint32_t n);

// ---- memory ---------------------------------------------------------------------
// 0xCC000000-0xCDFFFFFF is hardware (GX FIFO pipe, PI, VI, DSP, DI...).
#define PPC_IS_MMIO(a) (((a) & 0xFE000000u) == 0xCC000000u)

static inline uint8_t ld8(uint32_t a) {
    if (PPC_UNLIKELY(PPC_IS_MMIO(a))) return (uint8_t)ppc_mmio_read(a, 1);
    return g_mem[a];
}
static inline uint16_t ld16(uint32_t a) {
    if (PPC_UNLIKELY(PPC_IS_MMIO(a))) return (uint16_t)ppc_mmio_read(a, 2);
    uint16_t v; std::memcpy(&v, g_mem + a, 2); return PPC_BSWAP16(v);
}
static inline uint32_t ld32(uint32_t a) {
    if (PPC_UNLIKELY(PPC_IS_MMIO(a))) return ppc_mmio_read(a, 4);
    uint32_t v; std::memcpy(&v, g_mem + a, 4); return PPC_BSWAP32(v);
}
static inline uint64_t ld64(uint32_t a) {
    return ((uint64_t)ld32(a) << 32) | ld32(a + 4);
}
static inline void st8(uint32_t a, uint8_t v) {
    if (PPC_UNLIKELY(PPC_IS_MMIO(a))) { ppc_mmio_write(a, v, 1); return; }
    g_mem[a] = v;
}
static inline void st16(uint32_t a, uint16_t v) {
    if (PPC_UNLIKELY(PPC_IS_MMIO(a))) { ppc_mmio_write(a, v, 2); return; }
    v = PPC_BSWAP16(v); std::memcpy(g_mem + a, &v, 2);
}
static inline void st32(uint32_t a, uint32_t v) {
    if (PPC_UNLIKELY(PPC_IS_MMIO(a))) { ppc_mmio_write(a, v, 4); return; }
    v = PPC_BSWAP32(v); std::memcpy(g_mem + a, &v, 4);
}
static inline void st64(uint32_t a, uint64_t v) {
    st32(a, (uint32_t)(v >> 32)); st32(a + 4, (uint32_t)v);
}
static inline void ppc_dcbz(uint32_t ea) {
    ea &= ~31u;
    for (int i = 0; i < 32; i += 4) st32(ea + i, 0);
}

// ---- bit helpers ------------------------------------------------------------------
static inline uint32_t rotl32(uint32_t x, uint32_t n) {
    n &= 31; return n ? (x << n) | (x >> (32 - n)) : x;
}
static inline uint32_t cntlzw(uint32_t x) {
#if defined(_MSC_VER) && !defined(__clang__)
    unsigned long i; return _BitScanReverse(&i, x) ? 31 - i : 32;
#else
    return x ? (uint32_t)__builtin_clz(x) : 32;
#endif
}
static inline double   as_f64(uint64_t u) { double d; std::memcpy(&d, &u, 8); return d; }
static inline uint64_t as_u64(double d)   { uint64_t u; std::memcpy(&u, &d, 8); return u; }
static inline float    as_f32(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
static inline uint32_t as_u32(float f)    { uint32_t u; std::memcpy(&u, &f, 4); return u; }

// ---- condition register -------------------------------------------------------------
static inline void cr_set(PPCContext& c, int field, bool lt, bool gt, bool eq, bool so) {
    uint8_t* b = &c.cr[field * 4];
    b[0] = lt; b[1] = gt; b[2] = eq; b[3] = so;
}
static inline void cr_cmp_s(PPCContext& c, int field, int32_t a, int32_t b) {
    cr_set(c, field, a < b, a > b, a == b, c.xer_so);
}
static inline void cr_cmp_u(PPCContext& c, int field, uint32_t a, uint32_t b) {
    cr_set(c, field, a < b, a > b, a == b, c.xer_so);
}
static inline void cr0_rc(PPCContext& c, uint32_t v) { cr_cmp_s(c, 0, (int32_t)v, 0); }
static inline void cr1_rc(PPCContext& c) {          // FP record form: CR1 = FPSCR[FX FEX VX OX]
    uint32_t f = c.fpscr;
    cr_set(c, 1, f >> 31 & 1, f >> 30 & 1, f >> 29 & 1, f >> 28 & 1);
}
static inline uint32_t mfcr(const PPCContext& c) {
    uint32_t v = 0;
    for (int i = 0; i < 32; ++i) v |= (uint32_t)c.cr[i] << (31 - i);
    return v;
}
static inline void mtcrf(PPCContext& c, uint32_t crm, uint32_t v) {
    for (int fld = 0; fld < 8; ++fld)
        if (crm & (0x80u >> fld))
            for (int k = 0; k < 4; ++k) c.cr[fld * 4 + k] = v >> (31 - (fld * 4 + k)) & 1;
}
static inline uint32_t mfxer(const PPCContext& c) {
    return (uint32_t)c.xer_so << 31 | (uint32_t)c.xer_ov << 30 | (uint32_t)c.xer_ca << 29 | c.xer_bc;
}
static inline void mtxer(PPCContext& c, uint32_t v) {
    c.xer_so = v >> 31 & 1; c.xer_ov = v >> 30 & 1; c.xer_ca = v >> 29 & 1; c.xer_bc = v & 0x7F;
}

// ---- integer arithmetic with carry ------------------------------------------------------
// Carrying forms compute a + b + cin in 64 bits; CA is bit 32.
static inline uint32_t add_ca(PPCContext& c, uint32_t a, uint32_t b, uint32_t cin) {
    uint64_t s = (uint64_t)a + b + cin; c.xer_ca = (uint8_t)(s >> 32); return (uint32_t)s;
}
static inline void set_ov(PPCContext& c, bool ov) { c.xer_ov = ov; if (ov) c.xer_so = 1; }
static inline uint32_t divw(uint32_t a, uint32_t b) {
    int32_t sa = (int32_t)a, sb = (int32_t)b;
    if (sb == 0) return sa < 0 ? 0xFFFFFFFFu : 0;
    if (a == 0x80000000u && sb == -1) return 0xFFFFFFFFu;
    return (uint32_t)(sa / sb);
}
static inline uint32_t divwu(uint32_t a, uint32_t b) { return b ? a / b : 0; }
static inline uint32_t slw(uint32_t a, uint32_t n) { n &= 63; return n > 31 ? 0 : a << n; }
static inline uint32_t srw(uint32_t a, uint32_t n) { n &= 63; return n > 31 ? 0 : a >> n; }
static inline uint32_t sraw(PPCContext& c, uint32_t a, uint32_t n) {
    n &= 63;
    int32_t s = (int32_t)a;
    if (n > 31) { c.xer_ca = s < 0; return s < 0 ? 0xFFFFFFFFu : 0; }
    c.xer_ca = s < 0 && n && (a & ((1u << n) - 1)) != 0;
    return (uint32_t)(s >> n);
}
static inline bool trap_cond(uint32_t to, uint32_t a, uint32_t b) {
    int32_t sa = (int32_t)a, sb = (int32_t)b;
    return ((to & 16) && sa < sb) || ((to & 8) && sa > sb) || ((to & 4) && a == b) ||
           ((to & 2) && a < b) || ((to & 1) && a > b);
}

// ---- floating point -------------------------------------------------------------------
static inline double rs(double x) { return (double)(float)x; }     // round to single
static inline void fp_single(PPCContext& c, int d, double v) {      // single ops fill ps0 and ps1
    double r = rs(v); c.f[d] = r; c.ps1[d] = r;
}
static inline void fcmp(PPCContext& c, int field, double a, double b) {
    bool un = std::isnan(a) || std::isnan(b);
    cr_set(c, field, !un && a < b, !un && a > b, !un && a == b, un);
    c.fpscr = (c.fpscr & ~0xF000u) | ((uint32_t)(un ? 1 : a < b ? 8 : a > b ? 4 : 2) << 12);
}
static inline double fctiw_bits(PPCContext& c, double b, bool truncate) {
    int32_t v;
    if (std::isnan(b)) v = INT32_MIN;
    else if (b >= 2147483648.0) v = INT32_MAX;
    else if (b < -2147483648.0) v = INT32_MIN;
    else if (truncate) v = (int32_t)b;
    else {
        switch (c.fpscr & 3) {                     // RN field
        case 0: v = (int32_t)std::nearbyint(b); break;
        case 1: v = (int32_t)b; break;
        case 2: v = (int32_t)std::ceil(b); break;
        default: v = (int32_t)std::floor(b); break;
        }
    }
    return as_f64(0xFFF8000000000000ull | (uint32_t)v);
}
static inline double fres(double b) { return rs(1.0 / b); }
static inline double frsqrte(double b) { return 1.0 / std::sqrt(b); }
static inline void mtfsf(PPCContext& c, uint32_t fm, double b) {
    uint32_t v = (uint32_t)as_u64(b), m = 0;
    for (int i = 0; i < 8; ++i) if (fm & (0x80u >> i)) m |= 0xF0000000u >> (i * 4);
    c.fpscr = (c.fpscr & ~m) | (v & m);
}

// ---- paired-single quantisation (GQRs) --------------------------------------------------
static inline double ps_dequant(uint32_t type, int scale, uint32_t ea, int* size) {
    const double k = std::ldexp(1.0, -scale);
    switch (type) {
    case 4: *size = 1; return ld8(ea) * k;
    case 5: *size = 2; return ld16(ea) * k;
    case 6: *size = 1; return (int8_t)ld8(ea) * k;
    case 7: *size = 2; return (int16_t)ld16(ea) * k;
    default: *size = 4; return (double)as_f32(ld32(ea));
    }
}
static inline void ps_quant(uint32_t type, int scale, uint32_t ea, double v, int* size) {
    const double x = v * std::ldexp(1.0, scale);
    auto clampi = [](double y, double lo, double hi) { return y < lo ? lo : y > hi ? hi : y; };
    switch (type) {
    case 4: *size = 1; st8(ea, (uint8_t)clampi(x, 0, 255)); break;
    case 5: *size = 2; st16(ea, (uint16_t)clampi(x, 0, 65535)); break;
    case 6: *size = 1; st8(ea, (uint8_t)(int8_t)clampi(x, -128, 127)); break;
    case 7: *size = 2; st16(ea, (uint16_t)(int16_t)clampi(x, -32768, 32767)); break;
    default: *size = 4; st32(ea, as_u32((float)v)); break;
    }
}
static inline int gqr_scale(uint32_t s6) { return (int)(s6 & 0x20 ? s6 | ~0x3Fu : s6); }
static inline void psq_load(PPCContext& c, int d, uint32_t ea, int w, int i) {
    uint32_t g = c.gqr[i], type = g >> 16 & 7; int scale = gqr_scale(g >> 24 & 0x3F), sz;
    c.f[d] = ps_dequant(type, scale, ea, &sz);
    c.ps1[d] = w ? 1.0 : ps_dequant(type, scale, ea + sz, &sz);
}
static inline void psq_store(PPCContext& c, int s, uint32_t ea, int w, int i) {
    uint32_t g = c.gqr[i], type = g & 7; int scale = gqr_scale(g >> 8 & 0x3F), sz;
    ps_quant(type, scale, ea, c.f[s], &sz);
    if (!w) ps_quant(type, scale, ea + sz, c.ps1[s], &sz);
}
static inline void ps_set(PPCContext& c, int d, double a, double b) { c.f[d] = rs(a); c.ps1[d] = rs(b); }
