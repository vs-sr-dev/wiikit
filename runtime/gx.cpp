// wiikit runtime — the GX command stream: write-gather pipe, CPU FIFO, CP, PE,
// and the record the renderer draws from.
//
// The SDK writes GX commands to 0xCC008000; the pipe stores them in RAM at
// the PI's FIFO write pointer, wrapping between the FIFO's base and end. When
// that FIFO is linked to the graphics processor (CP control, GP link enable),
// the commands are also parsed here in order: CP and XF register loads (XF
// also from indexed arrays), BP writes, display-list calls (parsed from RAM),
// primitives, whose length comes from the vertex descriptor (VCD) and
// attribute format (VAT).
//
// The GP is always idle: it has consumed everything written. That answers
// what the SDK waits for: PE "draw done" and tokens, with their interrupts,
// and FIFO breakpoints (always set at data already written), reached at once.
// Whatever the renderer needs from guest memory is therefore read here, at
// the moment the game believes the GP read it (video.h): vertices decoded
// through the arrays, textures decoded to RGBA through TMEM's palettes and
// cached by content, and EFB copies remembered by address so that a texture
// read from there is the copy, kept on the host GPU.
#include "rt.h"
#include <chrono>
#include "gxtex.h"
#include "video.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <unordered_map>
#include <vector>

namespace {

uint32_t pi_base, pi_end, pi_wptr;                 // PI CPU FIFO (physical addresses)
uint16_t cp_reg16[0x40];                           // CP MMIO, 0xCC000000
uint16_t pe_ctrl, pe_token;                        // PE MMIO: interrupt control, token
uint32_t cp[0x100];                                // CP internal registers (VCD, VAT, arrays)
uint32_t bp[0x100];
uint32_t xf[0x1058];
uint32_t bp_mask = 0xFFFFFF;
std::vector<uint8_t> stream;                       // bytes not yet parsed
bool desync = false;
std::vector<uint8_t> tmem(1 << 20);                // texture memory: only palettes are loaded here

struct Stats { uint64_t cmds, draws, verts, dls, copies, frames, done, uploads; } st;

// CP control: 0 GP read enable, 1 breakpoint enable, 2/3 overflow/underflow
// interrupt enable, 4 GP link enable, 5 breakpoint interrupt enable
bool linked() { return (cp_reg16[1] & 0x11) == 0x11; }
bool bp_reached() { return (cp_reg16[1] & 0x3) == 0x3; }

uint32_t be32(const uint8_t* p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
uint16_t be16(const uint8_t* p) { return (uint16_t)(p[0] << 8 | p[1]); }

void warn_once(const char* what, uint32_t v) {
    static std::set<std::pair<const char*, uint32_t>> seen;
    if (seen.insert({what, v}).second) rt_log("gx: %s %X", what, v);
}

// guest physical range [a, a + n) is in MEM1 or MEM2
bool mem_ok(uint32_t a, size_t n) {
    return (a + n <= 0x01800000u) || (a >= 0x10000000u && a + n <= 0x14000000u);
}

// ---- the record ------------------------------------------------------------------------------
bool video = false;
std::vector<uint8_t> rec;
int rec_frames = 0;

uint8_t* grow(size_t n) {
    size_t o = rec.size();
    rec.resize(o + n);
    return rec.data() + o;
}
template <class T> void put(T v) { std::memcpy(grow(sizeof v), &v, sizeof v); }
void flush(bool force) {
    if (rec.empty() || (!force && rec.size() < (1u << 20))) return;
    video_submit(rec, rec_frames);
    rec_frames = 0;
}

uint64_t hash_mem(const uint8_t* p, size_t n) {
    uint64_t h = 0x9E3779B97F4A7C15ull ^ n;
    size_t i = 0;
    for (; i + 8 <= n; i += 8) {
        uint64_t w;
        std::memcpy(&w, p + i, 8);
        h = (h ^ w) * 0xFF51AFD7ED558CCDull;
        h ^= h >> 29;
    }
    for (; i < n; ++i) h = (h ^ p[i]) * 0x100000001B3ull;
    return h;
}

// ---- textures --------------------------------------------------------------------------------
// Decoded textures are cached by what names them (address, format, size,
// levels, palette) and re-uploaded when their bytes change. Bytes are hashed
// again only after GXInvalidateTexAll (BP 0x66), as TMEM's cache would load
// them again only then; a map's registers being rewritten makes it look again.
struct TexKey {
    uint32_t addr, shape, levels;
    uint64_t tlut;
    bool operator==(const TexKey& o) const {
        return addr == o.addr && shape == o.shape && levels == o.levels && tlut == o.tlut;
    }
};
struct TexKeyHash {
    size_t operator()(const TexKey& k) const {
        return (size_t)(k.addr * 0x9E3779B1u ^ k.shape * 0x85EBCA77u ^ k.levels ^ k.tlut);
    }
};
struct TexEntry { uint32_t id; uint64_t hash; uint32_t gen; };
std::unordered_map<TexKey, TexEntry, TexKeyHash> tcache;
uint32_t tex_next_id = 1, tex_gen = 1;
uint8_t tex_dirty = 0xFF;                          // maps whose binding must be looked at again
uint64_t map_bound[8];                             // what the renderer has on each map

struct CopyRec { size_t size; uint64_t hash; };
std::unordered_map<uint32_t, CopyRec> copies;      // EFB copies to texture, by address

// texture format an EFB copy format produces in RAM (for its size)
uint32_t copy_tex_fmt(uint32_t cf) {
    static const uint8_t f[16] = {0, 1, 2, 3, 4, 5, 6, 1, 1, 1, 1, 3, 3, 1, 1, 1};
    return f[cf & 15];
}

uint32_t used_maps() {
    uint32_t gen = bp[0x00], used = 0;
    int ntev = (int)(gen >> 10 & 15) + 1, nind = (int)(gen >> 16 & 7);
    for (int s = 0; s < ntev; ++s) {
        uint32_t t = bp[0x28 + s / 2] >> (12 * (s & 1));
        if (t & 0x40) used |= 1u << (t & 7);
    }
    for (int i = 0; i < nind && i < 4; ++i) used |= 1u << (bp[0x27] >> (6 * i) & 7);
    return used;
}

void bind_map(int m) {
    int off = (m & 3) + (m >= 4 ? 0x20 : 0);
    uint32_t mode0 = bp[0x80 + off], mode1 = bp[0x84 + off], img0 = bp[0x88 + off],
             img1 = bp[0x8C + off], img3 = bp[0x94 + off], tlut = bp[0x98 + off];
    int w = (int)(img0 & 0x3FF) + 1, h = (int)(img0 >> 10 & 0x3FF) + 1;
    uint32_t fmt = img0 >> 20 & 15, addr = (img3 & 0xFFFFFF) << 5;
    if (img1 >> 21 & 1) warn_once("texture preloaded in TMEM (not modelled), map", m);

    auto c = copies.find(addr);
    if (c != copies.end() && mem_ok(addr, c->second.size) &&
        hash_mem(host(virt(addr)), c->second.size) == c->second.hash) {
        uint64_t tag = 1ull << 32 | addr;
        if (map_bound[m] != tag) {
            put<uint8_t>(VC_TEXEFB);
            put<uint8_t>((uint8_t)m);
            put<uint32_t>(addr);
            map_bound[m] = tag;
        }
        return;
    }
    if (!gxtex_known(fmt)) { warn_once("unknown texture format", fmt); return; }

    int full = 1;
    while ((w >> full) || (h >> full)) ++full;
    int levels = 1;
    if (mode0 >> 5 & 3) {                          // a mipmapping minification filter
        levels = (int)((mode1 >> 8 & 0xFF) + 15) / 16 + 1;
        if (levels > full) levels = full;
    }
    size_t total = 0;
    for (int l = 0; l < levels; ++l) total += gxtex_size(fmt, std::max(1, w >> l), std::max(1, h >> l));
    if (!addr || !mem_ok(addr, total)) { warn_once("texture outside memory at", addr); return; }
    const uint8_t* src = host(virt(addr));

    const uint8_t* pal = nullptr;
    uint32_t tfmt = 0;
    uint64_t th = 0;
    if (gxtex_is_ci(fmt)) {
        uint32_t toff = (tlut & 0x3FF) << 9;
        size_t n = std::min<size_t>((size_t)gxtex_ci_entries(fmt) * 2, tmem.size() - toff);
        if (n < (size_t)gxtex_ci_entries(fmt) * 2) warn_once("palette past TMEM's end", toff);
        pal = &tmem[toff];
        tfmt = tlut >> 10 & 3;
        th = hash_mem(pal, n) ^ tfmt;
    }
    TexKey key{addr, fmt | (uint32_t)w << 4 | (uint32_t)h << 16, (uint32_t)levels, th};
    auto [it, fresh] = tcache.try_emplace(key, TexEntry{tex_next_id, 0, 0});
    if (fresh) ++tex_next_id;
    TexEntry& e = it->second;
    bool upload = false;
    if (fresh || e.gen != tex_gen) {
        uint64_t hh = hash_mem(src, total);
        upload = fresh || hh != e.hash;
        e.hash = hh;
        e.gen = tex_gen;
    }
    if (upload) {
        ++st.uploads;
        put<uint8_t>(VC_TEXUP);
        put<uint8_t>((uint8_t)m);
        put<uint32_t>(e.id);
        put<uint16_t>((uint16_t)w);
        put<uint16_t>((uint16_t)h);
        put<uint8_t>((uint8_t)levels);
        const uint8_t* s = src;
        size_t at = rec.size();
        for (int l = 0; l < levels; ++l) {
            int lw = std::max(1, w >> l), lh = std::max(1, h >> l);
            gxtex_decode(fmt, lw, lh, s, grow((size_t)lw * lh * 4), pal, tfmt);
            s += gxtex_size(fmt, lw, lh);
        }
        static const char* dump = std::getenv("WIIKIT_TEXDUMP");     // debugging: uploads as PNGs
        static int dumped = 0;
        if (dump && dumped < 400 && w * h >= 4096) {
            char name[512];
            std::snprintf(name, sizeof name, "%s/tex_%04d_%08X_f%u_%dx%d.png", dump, dumped++, addr, fmt, w, h);
            write_png(name, w, h, rec.data() + at);
        }
        map_bound[m] = e.id;
    } else if (map_bound[m] != e.id) {
        put<uint8_t>(VC_TEXBIND);
        put<uint8_t>((uint8_t)m);
        put<uint32_t>(e.id);
        map_bound[m] = e.id;
    }
}

void sync_textures() {
    uint32_t todo = used_maps() & tex_dirty;
    for (int m = 0; m < 8; ++m)
        if (todo >> m & 1) bind_map(m);
    tex_dirty &= (uint8_t)~todo;
}

// ---- vertices ----------------------------------------------------------------------------------
int fmt_size(uint32_t fmt) { return fmt == 0 || fmt == 1 ? 1 : fmt == 2 || fmt == 3 ? 2 : 4; }
int col_size(uint32_t fmt) { static const int s[8] = {2, 3, 4, 2, 3, 4, 4, 4}; return s[fmt & 7]; }

float comp(const uint8_t* q, uint32_t fmt, float sc) {
    switch (fmt) {
    case 0: return q[0] * sc;
    case 1: return (int8_t)q[0] * sc;
    case 2: return be16(q) * sc;
    case 3: return (int16_t)be16(q) * sc;
    default: { uint32_t u = be32(q); float f; std::memcpy(&f, &u, 4); return f; }
    }
}

void read_col(const uint8_t* q, uint32_t fmt, uint8_t* o) {
    switch (fmt) {
    case 0: { uint16_t v = be16(q);
              o[0] = (uint8_t)((v >> 11) << 3 | (v >> 13)); o[1] = (uint8_t)((v >> 5 & 63) << 2 | (v >> 9 & 3));
              o[2] = (uint8_t)((v & 31) << 3 | (v >> 2 & 7)); o[3] = 255; break; }
    case 1: case 2: o[0] = q[0]; o[1] = q[1]; o[2] = q[2]; o[3] = 255; break;
    case 3: { uint16_t v = be16(q);
              o[0] = (uint8_t)((v >> 12) * 17); o[1] = (uint8_t)((v >> 8 & 15) * 17);
              o[2] = (uint8_t)((v >> 4 & 15) * 17); o[3] = (uint8_t)((v & 15) * 17); break; }
    case 4: { uint32_t v = (uint32_t)q[0] << 16 | q[1] << 8 | q[2];
              for (int i = 0; i < 4; ++i) { uint32_t c = v >> (18 - 6 * i) & 63; o[i] = (uint8_t)(c << 2 | c >> 4); }
              break; }
    default: o[0] = q[0]; o[1] = q[1]; o[2] = q[2]; o[3] = q[3]; break;
    }
}

struct VtxFmt {
    uint32_t lo, hi;
    uint32_t pos_fmt, nrm_fmt, col_fmt[2], tc_fmt[8];
    int pos_n, tc_n[8];
    bool nbt, nidx3;
    float pos_sc, nrm_sc, tc_sc[8];
    uint8_t mtx_default[9];
};

VtxFmt vtx_fmt(int v) {
    uint32_t a = cp[0x70 + v], b = cp[0x80 + v], c = cp[0x90 + v];
    VtxFmt f{};
    f.lo = cp[0x50];
    f.hi = cp[0x60];
    f.pos_n = (a & 1) ? 3 : 2;
    f.pos_fmt = a >> 1 & 7;
    f.pos_sc = 1.0f / (float)(1u << (a >> 4 & 31));
    f.nbt = a >> 9 & 1;
    f.nrm_fmt = a >> 10 & 7;
    f.nrm_sc = f.nrm_fmt == 4 ? 1.0f : fmt_size(f.nrm_fmt) == 1 ? 1.0f / 64 : 1.0f / 16384;
    f.nidx3 = a >> 31 & 1;
    f.col_fmt[0] = a >> 14 & 7;
    f.col_fmt[1] = a >> 18 & 7;
    const uint32_t t[8][3] = {{a >> 21 & 1, a >> 22 & 7, a >> 25 & 31}, {b & 1, b >> 1 & 7, b >> 4 & 31},
                              {b >> 9 & 1, b >> 10 & 7, b >> 13 & 31}, {b >> 18 & 1, b >> 19 & 7, b >> 22 & 31},
                              {b >> 27 & 1, b >> 28 & 7, c & 31},      {c >> 5 & 1, c >> 6 & 7, c >> 9 & 31},
                              {c >> 14 & 1, c >> 15 & 7, c >> 18 & 31}, {c >> 23 & 1, c >> 24 & 7, c >> 27 & 31}};
    for (int i = 0; i < 8; ++i) {
        f.tc_n[i] = t[i][0] ? 2 : 1;
        f.tc_fmt[i] = t[i][1];
        f.tc_sc[i] = 1.0f / (float)(1u << t[i][2]);
    }
    uint32_t ma = cp[0x30], mb = cp[0x40];
    f.mtx_default[0] = ma & 63;
    for (int i = 0; i < 4; ++i) f.mtx_default[1 + i] = (uint8_t)(ma >> (6 + 6 * i) & 63);
    for (int i = 0; i < 4; ++i) f.mtx_default[5 + i] = (uint8_t)(mb >> (6 * i) & 63);
    return f;
}

// bytes per vertex for vertex format `v`
int vertex_size(const VtxFmt& f) {
    auto index = [](uint32_t mode) { return mode == 2 ? 1 : mode == 3 ? 2 : 0; };
    int n = 0;
    for (int i = 0; i < 9; ++i) n += f.lo >> i & 1;
    uint32_t pos = f.lo >> 9 & 3, nrm = f.lo >> 11 & 3;
    n += pos == 1 ? f.pos_n * fmt_size(f.pos_fmt) : index(pos);
    if (nrm == 1) n += (f.nbt ? 9 : 3) * fmt_size(f.nrm_fmt);
    else n += index(nrm) * (f.nbt && f.nidx3 ? 3 : 1);
    for (int i = 0; i < 2; ++i) {
        uint32_t m = f.lo >> (13 + 2 * i) & 3;
        n += m == 1 ? col_size(f.col_fmt[i]) : index(m);
    }
    for (int i = 0; i < 8; ++i) {
        uint32_t m = f.hi >> (2 * i) & 3;
        n += m == 1 ? f.tc_n[i] * fmt_size(f.tc_fmt[i]) : index(m);
    }
    return n;
}

// an attribute's data: in the stream (direct) or in its array (indexed)
const uint8_t* fetch(uint32_t mode, int arr, int size, const uint8_t*& p, uint32_t sub = 0) {
    if (mode == 1) { const uint8_t* q = p; p += size; return q; }
    uint32_t idx;
    if (mode == 2) idx = *p++;
    else { idx = be16(p); p += 2; }
    return host(virt(cp[0xA0 + arr] + idx * cp[0xB0 + arr] + sub));
}

uint8_t decode_vertices(const VtxFmt& f, const uint8_t* p, uint32_t count, GVtx* out) {
    uint32_t pos = f.lo >> 9 & 3, nrm = f.lo >> 11 & 3, col[2] = {f.lo >> 13 & 3, f.lo >> 15 & 3};
    uint8_t flags = (col[0] ? VTX_COL0 : 0) | (col[1] ? VTX_COL1 : 0) | (nrm ? VTX_NRM : 0) |
                    (nrm && f.nbt ? VTX_NBT : 0);
    int nsz = fmt_size(f.nrm_fmt);
    for (uint32_t k = 0; k < count; ++k) {
        GVtx& v = out[k];
        std::memset(&v, 0, sizeof v);
        for (int i = 0; i < 9; ++i) v.mtx[i] = (f.lo >> i & 1) ? *p++ : f.mtx_default[i];
        if (pos) {
            const uint8_t* q = fetch(pos, 0, f.pos_n * fmt_size(f.pos_fmt), p);
            for (int i = 0; i < f.pos_n; ++i) v.pos[i] = comp(q + i * fmt_size(f.pos_fmt), f.pos_fmt, f.pos_sc);
        }
        if (nrm) {
            float* dst[3] = {v.nrm, v.bin, v.tan};
            if (f.nbt && f.nidx3 && nrm != 1) {
                for (int j = 0; j < 3; ++j) {
                    const uint8_t* q = fetch(nrm, 1, 0, p, (uint32_t)(j * 3 * nsz));
                    for (int i = 0; i < 3; ++i) dst[j][i] = comp(q + i * nsz, f.nrm_fmt, f.nrm_sc);
                }
            } else {
                int n = f.nbt ? 3 : 1;
                const uint8_t* q = fetch(nrm, 1, 3 * n * nsz, p);
                for (int j = 0; j < n; ++j)
                    for (int i = 0; i < 3; ++i) dst[j][i] = comp(q + (j * 3 + i) * nsz, f.nrm_fmt, f.nrm_sc);
            }
        }
        for (int c = 0; c < 2; ++c) {
            if (col[c]) read_col(fetch(col[c], 2 + c, col_size(f.col_fmt[c]), p), f.col_fmt[c], v.col[c]);
            else std::memset(v.col[c], 255, 4);    // a missing colour reads as white
        }
        for (int t = 0; t < 8; ++t) {
            uint32_t m = f.hi >> (2 * t) & 3;
            if (!m) continue;
            int cs = fmt_size(f.tc_fmt[t]);
            const uint8_t* q = fetch(m, 4 + t, f.tc_n[t] * cs, p);
            for (int i = 0; i < f.tc_n[t]; ++i) v.tc[t][i] = comp(q + i * cs, f.tc_fmt[t], f.tc_sc[t]);
        }
    }
    return flags;
}

// ---- BP ----------------------------------------------------------------------------------------
void efb_copy(uint32_t v) {
    if (v >> 14 & 1) {                                 // to the XFB: the frame is done
        ++st.frames;
        if (video) {
            put<uint8_t>(VC_FRAME);
            ++rec_frames;
            flush(true);
        }
        return;
    }
    uint32_t dest = (bp[0x4B] & 0xFFFFFF) << 5;
    int w = (int)(bp[0x4A] & 0x3FF) + 1, h = (int)(bp[0x4A] >> 10 & 0x3FF) + 1;
    if (v >> 9 & 1) { w = (w + 1) / 2; h = (h + 1) / 2; }
    uint32_t tpf = v >> 3 & 15, real = tpf / 2 + (tpf & 1) * 8;
    size_t size = gxtex_size(copy_tex_fmt(real), w, h);
    if (mem_ok(dest, size)) copies[dest] = CopyRec{size, hash_mem(host(virt(dest)), size)};
    tex_dirty = 0xFF;
}

void bp_write(uint32_t v) {
    uint32_t reg = v >> 24, val = v & 0xFFFFFF;
    if (reg == 0xFE) { bp_mask = val; return; }
    bp[reg] = (bp[reg] & ~bp_mask) | (val & bp_mask);
    bp_mask = 0xFFFFFF;
    switch (reg) {
    case 0x45: pe_ctrl |= 8; ++st.done; os_raise(); return;          // PE_DONE: draw done
    case 0x47: pe_token = (uint16_t)val; return;                      // token
    case 0x48: pe_token = (uint16_t)val; pe_ctrl |= 4; os_raise(); return;   // token + interrupt
    case 0x64: return;                                                // TLUT source address
    case 0x65: {                                                      // load a TLUT into TMEM
        uint32_t src = (bp[0x64] & 0xFFFFFF) << 5, dst = (val & 0x3FF) << 9, n = (val >> 10 & 0x7FF) << 5;
        if (dst + n <= tmem.size() && mem_ok(src, n)) std::memcpy(&tmem[dst], host(virt(src)), n);
        tex_dirty = 0xFF;
        return;
    }
    case 0x66: ++tex_gen; tex_dirty = 0xFF; return;                   // GXInvalidateTexAll
    case 0x63: warn_once("TMEM preload", val); break;
    }
    if ((reg >= 0x80 && reg < 0x9C) || (reg >= 0xA0 && reg < 0xBC))
        tex_dirty |= (uint8_t)(1u << ((reg & 3) + (reg >= 0xA0 ? 4 : 0)));
    if (video) {
        put<uint8_t>(VC_BP);
        put<uint32_t>(reg << 24 | bp[reg]);
    }
    if (reg == 0x52) { ++st.copies; efb_copy(bp[reg]); }
}

void xf_load(uint32_t addr, uint32_t n, const uint8_t* src) {
    if (addr >= 0x1058) return;
    if (addr + n > 0x1058) n = 0x1058 - addr;
    for (uint32_t i = 0; i < n; ++i) xf[addr + i] = be32(src + 4 * i);
    if (video) {
        put<uint8_t>(VC_XF);
        put<uint16_t>((uint16_t)addr);
        put<uint16_t>((uint16_t)n);
        std::memcpy(grow(4 * n), &xf[addr], 4 * n);
    }
}

// parse one command at p (n bytes available); returns its length, 0 if incomplete
size_t parse(const uint8_t* p, size_t n, bool in_dl);

void call_dl(uint32_t addr, uint32_t size) {
    ++st.dls;
    if (!mem_ok(addr & 0x1FFFFFFF, size)) { warn_once("display list outside memory at", addr); return; }
    const uint8_t* p = host(virt(addr));
    size_t off = 0;
    while (off < size) {
        size_t k = parse(p + off, size - off, true);
        if (!k) break;
        off += k;
    }
}

size_t parse(const uint8_t* p, size_t n, bool in_dl) {
    uint8_t op = p[0];
    ++st.cmds;
    switch (op) {
    case 0x00: case 0x48: case 0x44: return 1;
    case 0x08:
        if (n < 6) return 0;
        cp[p[1]] = be32(p + 2);
        return 6;
    case 0x10: {
        if (n < 5) return 0;
        uint32_t h = be32(p + 1), cnt = (h >> 16 & 0xF) + 1, addr = h & 0xFFFF;
        size_t len = 5 + 4 * cnt;
        if (n < len) return 0;
        xf_load(addr, cnt, p + 5);
        return len;
    }
    case 0x20: case 0x28: case 0x30: case 0x38: {       // XF load from an indexed array (A-D)
        if (n < 5) return 0;
        uint32_t d = be32(p + 1), idx = d >> 16, cnt = (d >> 12 & 15) + 1, addr = d & 0xFFF;
        int arr = 12 + (op - 0x20) / 8;
        uint32_t src = cp[0xA0 + arr] + idx * cp[0xB0 + arr];
        if (mem_ok(src & 0x1FFFFFFF, 4 * cnt)) xf_load(addr, cnt, host(virt(src)));
        return 5;
    }
    case 0x40:
        if (n < 9) return 0;
        if (in_dl) rt_log("gx: display list calls a display list");
        else call_dl(be32(p + 1), be32(p + 5));
        return 9;
    case 0x61:
        if (n < 5) return 0;
        bp_write(be32(p + 1));
        return 5;
    }
    if (op >= 0x80 && op < 0xC0) {
        if (n < 3) return 0;
        uint32_t count = (uint32_t)p[1] << 8 | p[2];
        VtxFmt f = vtx_fmt(op & 7);
        size_t len = 3 + (size_t)count * vertex_size(f);
        if (n < len) return 0;
        ++st.draws;
        st.verts += count;
        if (video && count) {
            if (bp[0xF1] >> 21 & 7) warn_once("fog (not drawn), type", bp[0xF1] >> 21 & 7);
            if (bp[0xF4] >> 2 & 3) warn_once("Z texture (not drawn), op", bp[0xF4] >> 2 & 3);
            if (g_vperf_on) {
                auto t0 = std::chrono::steady_clock::now();
                sync_textures();
                g_vperf.tex += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
            } else {
                sync_textures();
            }
            put<uint8_t>(VC_DRAW);
            put<uint8_t>(op & 0xF8);
            size_t at = rec.size();
            put<uint8_t>(0);
            put<uint32_t>(count);
            GVtx* out = reinterpret_cast<GVtx*>(grow(count * sizeof(GVtx)));
            if (g_vperf_on) {
                auto t0 = std::chrono::steady_clock::now();
                rec[at] = decode_vertices(f, p + 3, count, out);
                g_vperf.vtx += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
            } else {
                rec[at] = decode_vertices(f, p + 3, count, out);
            }
            flush(false);
        }
        return len;
    }
    if (!desync) rt_log("gx: unknown command %02X: the stream is lost from here", op);
    desync = true;
    return n;
}

void feed(const uint8_t* b, int n) {
    if (desync) return;
    stream.insert(stream.end(), b, b + n);
    size_t off = 0;
    while (off < stream.size()) {
        size_t k = parse(stream.data() + off, stream.size() - off, false);
        if (!k) { --st.cmds; break; }
        off += k;
    }
    stream.erase(stream.begin(), stream.begin() + (ptrdiff_t)off);
}

}  // namespace

void gx_init() { video = video_enabled(); }

// The write-gather pipe: CPU stores to 0xCC008000 collect in a 32-byte
// buffer, and reach the FIFO in memory as 32-byte bursts, as on the console
// (the SDK's GXFlush pads the last one out). The stores themselves take no
// lock (hw.cpp); a burst does.
void gx_pipe_burst(const uint8_t* b, int n) {
    for (int i = 0; i < n; ++i) {
        uint32_t a = pi_wptr & 0x03FFFFFFu;
        *host(virt(a)) = b[i];
        ++a;
        if (pi_end && a >= (pi_end & 0x03FFFFFFu)) pi_wptr = (pi_base & 0x03FFFFFFu) | 0x20000000u;
        else pi_wptr = (pi_wptr & 0x20000000u) | a;
    }
    if (linked()) feed(b, n);
}

uint32_t gx_pi_fifo_read(uint32_t off) {
    return off == 0x0C ? pi_base : off == 0x10 ? pi_end : pi_wptr;
}
void gx_pi_fifo_write(uint32_t off, uint32_t v) {
    if (off == 0x0C) pi_base = v;
    else if (off == 0x10) pi_end = v;
    else if (off == 0x14) pi_wptr = v;
}

uint32_t gx_cp_read(uint32_t off, int size) {
    auto r16 = [](uint32_t o) -> uint16_t {
        uint32_t wp = pi_wptr & 0x03FFFFFFu;
        switch (o) {
        case 0x00:                                       // underflow, GP read idle, command idle
            return (uint16_t)(0x0E | (bp_reached() ? 0x10 : 0));
        case 0x30: case 0x32: return 0;                  // read-write distance: empty
        case 0x34: case 0x38: return (uint16_t)wp;       // write and read pointers
        case 0x36: case 0x3A: return (uint16_t)(wp >> 16);
        }
        return o / 2 < 0x40 ? cp_reg16[o / 2] : 0;
    };
    if (size == 4) return (uint32_t)r16(off) << 16 | r16(off + 2);
    return r16(off & ~1u);
}

void gx_cp_write(uint32_t off, uint32_t v, int size) {
    if (size == 4) {
        gx_cp_write(off, v >> 16, 2);
        gx_cp_write(off + 2, v & 0xFFFF, 2);
        return;
    }
    if (off / 2 < 0x40) cp_reg16[off / 2] = (uint16_t)v;
    if (off == 0x02) os_raise();
}

uint32_t gx_pe_read(uint32_t off, int) {
    if (off == 0x0A) return pe_ctrl;
    if (off == 0x0E) return pe_token;
    return 0;
}

void gx_pe_write(uint32_t off, uint32_t v, int) {
    if (off == 0x0A) {
        uint16_t clear = (uint16_t)(v & 0xC);            // status bits: write 1 to clear
        pe_ctrl = (uint16_t)((v & 3) | (pe_ctrl & 0xC & ~clear));
    }
}

uint32_t gx_irq() {
    uint32_t c = 0;
    if ((pe_ctrl & 4) && (pe_ctrl & 1)) c |= 0x200;      // PE token
    if ((pe_ctrl & 8) && (pe_ctrl & 2)) c |= 0x400;      // PE finish
    if (bp_reached() && (cp_reg16[1] & 0x20)) c |= 0x800;   // CP breakpoint
    return c;
}

void gx_report() {
    rt_log("gx: %llu commands, %llu draws (%llu vertices), %llu display lists, %llu EFB copies "
           "(%llu frames), %llu draw-done, %llu texture uploads",
           (unsigned long long)st.cmds, (unsigned long long)st.draws, (unsigned long long)st.verts,
           (unsigned long long)st.dls, (unsigned long long)st.copies, (unsigned long long)st.frames,
           (unsigned long long)st.done, (unsigned long long)st.uploads);
}
