// wiikit runtime — the part every build shares: guest memory pointer, the
// address -> function dispatch, hooks, the address space above 0xC0000000,
// symbol names and crash reports.
#include "rt.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

extern const PPCFuncEntry g_ppc_funcs[];
extern const size_t g_ppc_nfuncs;
extern const PPCHook g_ppc_hooks[];
extern const PPCSymbol g_ppc_symbols[];

uint8_t* g_mem = nullptr;
thread_local PPCContext* t_ppc = nullptr;

PPCFunc ppc_lookup(uint32_t addr) {
    size_t lo = 0, hi = g_ppc_nfuncs;
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (g_ppc_funcs[mid].addr < addr) lo = mid + 1; else hi = mid;
    }
    return lo < g_ppc_nfuncs && g_ppc_funcs[lo].addr == addr ? g_ppc_funcs[lo].fn : nullptr;
}

void ppc_call_indirect(PPCContext& c, uint32_t addr) {
    PPCFunc f = ppc_lookup(addr);
    if (!f) {
        std::fprintf(stderr, "wiikit: indirect call to %s, not a function\n", rt_name(addr).c_str());
        rt_backtrace(c, stderr);
        std::exit(1);
    }
    f(c);
}

PPCFunc ppc_hook(const char* name, PPCFunc fn) {
    for (const PPCHook* h = g_ppc_hooks; h->name; ++h)
        if (!std::strcmp(h->name, name)) { *h->slot = fn; return h->orig; }
    return nullptr;
}

uint32_t ppc_symbol(const char* name) {
    for (const PPCSymbol* s = g_ppc_symbols; s->name; ++s)
        if (!std::strcmp(s->name, name)) return s->addr;
    return 0;
}

// ---- above 0xC0000000 ---------------------------------------------------------------------
uint32_t ppc_io_read(uint32_t a, int size) {
    if ((a & 0xFE000000u) == 0xCC000000u) return ppc_mmio_read(a, size);
    uint8_t* p = host(a);
    switch (size) {
    case 1: return *p;
    case 2: { uint16_t v; std::memcpy(&v, p, 2); return PPC_BSWAP16(v); }
    default: { uint32_t v; std::memcpy(&v, p, 4); return PPC_BSWAP32(v); }
    }
}

void ppc_io_write(uint32_t a, uint32_t v, int size) {
    if ((a & 0xFE000000u) == 0xCC000000u) { ppc_mmio_write(a, v, size); return; }
    uint8_t* p = host(a);
    switch (size) {
    case 1: *p = (uint8_t)v; break;
    case 2: { uint16_t x = PPC_BSWAP16((uint16_t)v); std::memcpy(p, &x, 2); break; }
    default: { uint32_t x = PPC_BSWAP32(v); std::memcpy(p, &x, 4); break; }
    }
}

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

std::string guest_cstr(uint32_t addr, size_t max) {
    std::string s;
    for (size_t i = 0; i < max; ++i) {
        char ch = (char)ld8(addr + (uint32_t)i);
        if (!ch) break;
        s += ch;
    }
    return s;
}

// ---- names -----------------------------------------------------------------------------
static std::map<uint32_t, std::pair<uint32_t, std::string>> g_names;   // addr -> (size, name)

void rt_load_symbols(const char* tsv) {
    std::ifstream f(tsv);
    std::string line;
    std::getline(f, line);                         // header
    while (std::getline(f, line)) {
        std::vector<std::string> col;
        std::stringstream ss(line);
        std::string x;
        while (std::getline(ss, x, '\t')) col.push_back(x);
        if (col.size() < 7 || col[3] != "2") continue;   // functions only
        uint32_t a = (uint32_t)std::stoul(col[0], nullptr, 16), sz = (uint32_t)std::stoul(col[1]);
        if (a && sz) g_names[a] = {sz, col[6].empty() ? col[5] : col[6]};
    }
}

std::string rt_name(uint32_t a) {
    char buf[32];
    auto it = g_names.upper_bound(a);
    if (it != g_names.begin()) {
        --it;
        if (a < it->first + it->second.first) {
            std::string n = it->second.second;
            if (n.size() > 90) n = n.substr(0, 87) + "...";
            if (a == it->first) return n;
            std::snprintf(buf, sizeof buf, "+0x%X", a - it->first);
            return n + buf;
        }
    }
    std::snprintf(buf, sizeof buf, "%08X", a);
    return buf;
}

void rt_log(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
}

void rt_backtrace(const PPCContext& c, FILE* out, int max) {
    std::fprintf(out, "  lr  %08X  %s\n", c.lr, rt_name(c.lr).c_str());
    uint32_t sp = c.r[1];
    for (int i = 0; i < max && sp >= 0x80000000u && sp < 0x94000000u; ++i) {
        uint32_t next = ld32(sp);
        if (next <= sp || next >= 0x94000000u) break;
        uint32_t lr = ld32(next + 4);
        std::fprintf(out, "  %08X  %s\n", lr, rt_name(lr).c_str());
        sp = next;
    }
}

[[noreturn]] void rt_die(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "wiikit: ");
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
    if (t_ppc) rt_backtrace(*t_ppc, stderr);
    std::fflush(stderr);
    std::fflush(stdout);
    std::_Exit(1);
}
