// wiikit runtime — guest memory and executable loading.
#include "mem.h"
#include "ppc.h"
#include <cstdio>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/mman.h>
#endif

struct Region { uint32_t base, size; };
static const Region kRegions[] = {
    {0x80000000u, 0x01800000u},   // MEM1, 24 MiB
    {0x90000000u, 0x04000000u},   // MEM2, 64 MiB
    {0xE0000000u, 0x00004000u},   // the locked half of the L1 data cache, 16 KiB
};

bool mem_init() {
    const size_t span = size_t(1) << 32;
#ifdef _WIN32
    void* base = VirtualAlloc(nullptr, span, MEM_RESERVE, PAGE_NOACCESS);
    if (!base) return false;
    g_mem = static_cast<uint8_t*>(base);
    for (const Region& r : kRegions)
        if (!VirtualAlloc(g_mem + r.base, r.size, MEM_COMMIT, PAGE_READWRITE)) return false;
#else
    void* base = mmap(nullptr, span, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (base == MAP_FAILED) return false;
    g_mem = static_cast<uint8_t*>(base);
    for (const Region& r : kRegions)
        if (mprotect(g_mem + r.base, r.size, PROT_READ | PROT_WRITE)) return false;
#endif
    return true;
}

void mem_write(uint32_t addr, const void* src, size_t n) { std::memcpy(g_mem + addr, src, n); }
void mem_read(uint32_t addr, void* dst, size_t n) { std::memcpy(dst, g_mem + addr, n); }

static uint32_t be32(const uint8_t* p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

uint32_t mem_load_dol(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return 0;
    std::vector<uint8_t> d;
    uint8_t buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) d.insert(d.end(), buf, buf + n);
    std::fclose(f);
    if (d.size() < 0x100) return 0;
    for (int i = 0; i < 18; ++i) {
        uint32_t off = be32(&d[i * 4]), addr = be32(&d[0x48 + i * 4]), size = be32(&d[0x90 + i * 4]);
        if (size && off + size <= d.size()) mem_write(addr, &d[off], size);
    }
    uint32_t bss = be32(&d[0xD8]), bss_size = be32(&d[0xDC]);
    // bss covers the small-data sections too; clear only what no section loaded into
    std::vector<uint8_t> zero(bss_size, 0);
    for (uint32_t a = bss; a < bss + bss_size;) {
        uint32_t next = bss + bss_size;
        bool inside = false;
        for (int i = 0; i < 18; ++i) {
            uint32_t addr = be32(&d[0x48 + i * 4]), size = be32(&d[0x90 + i * 4]);
            if (!size) continue;
            if (a >= addr && a < addr + size) { inside = true; next = addr + size; break; }
            if (addr > a && addr < next) next = addr;
        }
        if (!inside) std::memset(g_mem + a, 0, next - a);
        a = next;
    }
    return be32(&d[0xE0]);
}
