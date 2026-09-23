// wiikit runtime — minimal services: dispatch, string ops, and an abort for
// everything a hardware-free run must never reach (system calls, traps,
// MMIO). The real runtime replaces this file.
#include "ppc.h"
#include <cstdio>
#include <cstdlib>

struct PPCFuncEntry { uint32_t addr; PPCFunc fn; };
extern const PPCFuncEntry g_ppc_funcs[];
extern const size_t g_ppc_nfuncs;

uint8_t* g_mem = nullptr;

PPCFunc ppc_lookup(uint32_t addr) {
    size_t lo = 0, hi = g_ppc_nfuncs;
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (g_ppc_funcs[mid].addr < addr) lo = mid + 1; else hi = mid;
    }
    return lo < g_ppc_nfuncs && g_ppc_funcs[lo].addr == addr ? g_ppc_funcs[lo].fn : nullptr;
}

[[noreturn]] static void die(const char* what, uint32_t addr) {
    std::fprintf(stderr, "wiikit: %s at %08X\n", what, addr);
    std::exit(1);
}

void ppc_call_indirect(PPCContext& c, uint32_t addr) {
    PPCFunc f = ppc_lookup(addr);
    if (!f) die("indirect call to unknown address", addr);
    f(c);
}
void ppc_syscall(PPCContext&, uint32_t addr) { die("sc", addr); }
void ppc_trap(PPCContext&, uint32_t addr) { die("trap", addr); }
void ppc_unimplemented(PPCContext&, uint32_t addr, const char* what) { die(what, addr); }
uint32_t ppc_mmio_read(uint32_t addr, int) { die("mmio read", addr); }
void ppc_mmio_write(uint32_t addr, uint32_t, int) { die("mmio write", addr); }
uint64_t ppc_timebase() { return 0; }

void ppc_lswx(PPCContext& c, int rd, uint32_t ea, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        int r = (rd + i / 4) & 31;
        if (i % 4 == 0) c.r[r] = 0;
        c.r[r] |= (uint32_t)ld8(ea + i) << (24 - 8 * (i % 4));
    }
}
void ppc_stswx(PPCContext& c, int rs, uint32_t ea, uint32_t n) {
    for (uint32_t i = 0; i < n; ++i)
        st8(ea + i, (uint8_t)(c.r[(rs + i / 4) & 31] >> (24 - 8 * (i % 4))));
}
