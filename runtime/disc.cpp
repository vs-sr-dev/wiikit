// wiikit runtime — the game's DATA partition, served from an extracted tree.
//
// `python -m wiikit.disc GAME.wbfs --extract DIR` writes sys/ (boot.bin,
// bi2.bin, apploader.img, main.dol, fst.bin) and files/. The partition is
// rebuilt from them as a map of regions: the system files at their fixed
// offsets and the DOL and FST where boot.bin puts them, then every file where
// the FST puts it. Reads anywhere else return zeros.
#include "disc.h"
#include "rt.h"
#include <algorithm>
#include <map>
#include <vector>
#ifndef _WIN32
#define _fseeki64 fseeko
#define _ftelli64 ftello
#endif

namespace {

struct Region { uint64_t off, size; std::string path; };
std::vector<Region> g_regions;
std::map<std::string, FILE*> g_files;
std::vector<uint8_t> g_boot, g_fst;

uint32_t be32(const uint8_t* p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }

std::vector<uint8_t> slurp(const std::string& path) {
    std::vector<uint8_t> d;
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return d;
    std::fseek(f, 0, SEEK_END);
    d.resize((size_t)std::ftell(f));
    std::fseek(f, 0, SEEK_SET);
    if (!d.empty() && std::fread(d.data(), 1, d.size(), f) != d.size()) d.clear();
    std::fclose(f);
    return d;
}

uint64_t file_size(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return 0;
    _fseeki64(f, 0, SEEK_END);
    uint64_t n = (uint64_t)_ftelli64(f);
    std::fclose(f);
    return n;
}

void add(uint64_t off, const std::string& path) {
    uint64_t n = file_size(path);
    if (n) g_regions.push_back({off, n, path});
}

// walk the FST: directories hold [first child, one past the last entry]
void walk(const std::string& root) {
    const uint8_t* f = g_fst.data();
    uint32_t total = be32(f + 8);
    const char* names = (const char*)f + total * 12;
    std::vector<std::pair<uint32_t, std::string>> stack{{total, ""}};
    for (uint32_t i = 1; i < total; ++i) {
        const uint8_t* e = f + i * 12;
        while (i >= stack.back().first) stack.pop_back();
        std::string name = names + (be32(e) & 0xFFFFFF);
        if (e[0]) stack.push_back({be32(e + 8), stack.back().second + name + "/"});
        else add((uint64_t)be32(e + 4) << 2, root + "/files/" + stack.back().second + name);
    }
}

}  // namespace

bool disc_open(const char* dir) {
    std::string root = dir;
    g_boot = slurp(root + "/sys/boot.bin");
    g_fst = slurp(root + "/sys/fst.bin");
    if (g_boot.size() < 0x440 || g_fst.size() < 12) return false;
    add(0, root + "/sys/boot.bin");
    add(0x440, root + "/sys/bi2.bin");
    add(0x2440, root + "/sys/apploader.img");
    add((uint64_t)be32(&g_boot[0x420]) << 2, root + "/sys/main.dol");
    add((uint64_t)be32(&g_boot[0x424]) << 2, root + "/sys/fst.bin");
    walk(root);
    std::sort(g_regions.begin(), g_regions.end(), [](const Region& a, const Region& b) { return a.off < b.off; });
    return true;
}

size_t disc_read(uint64_t off, void* dst, size_t n) {
    uint8_t* out = static_cast<uint8_t*>(dst);
    std::memset(out, 0, n);
    auto it = std::upper_bound(g_regions.begin(), g_regions.end(), off,
                               [](uint64_t o, const Region& r) { return o < r.off; });
    if (it != g_regions.begin()) --it;
    for (; it != g_regions.end() && it->off < off + n; ++it) {
        uint64_t a = std::max(off, it->off), b = std::min(off + n, it->off + it->size);
        if (a >= b) continue;
        FILE*& f = g_files[it->path];
        if (!f) f = std::fopen(it->path.c_str(), "rb");
        if (!f) continue;
        _fseeki64(f, (int64_t)(a - it->off), SEEK_SET);
        size_t got = std::fread(out + (a - off), 1, (size_t)(b - a), f);
        (void)got;
    }
    return n;
}

const std::vector<uint8_t>& disc_boot() { return g_boot; }
const std::vector<uint8_t>& disc_fst() { return g_fst; }
size_t disc_regions() { return g_regions.size(); }
