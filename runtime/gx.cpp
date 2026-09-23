// wiikit runtime — the GX command stream: write-gather pipe, CPU FIFO, CP, PE.
//
// The SDK writes GX commands to 0xCC008000; the pipe stores them in RAM at
// the PI's FIFO write pointer, wrapping between the FIFO's base and end. When
// that FIFO is linked to the graphics processor (CP control, GP link enable),
// the commands are also parsed here in order: CP and XF register loads, BP
// writes, display-list calls (parsed from RAM), primitives, whose length comes
// from the vertex descriptor (VCD) and attribute format (VAT). Nothing is
// drawn yet: this is the command processor the renderer will sit on. It
// already answers what the SDK waits for: PE "draw done" and tokens, with
// their interrupts, an idle CP, and FIFO breakpoints: GP always idle, it
// has consumed everything written, so an armed breakpoint (always set at
// data already written) is reached at once.
#include "rt.h"
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

struct Stats { uint64_t cmds, draws, verts, dls, copies, done; } st;

// CP control: 0 GP read enable, 1 breakpoint enable, 2/3 overflow/underflow
// interrupt enable, 4 GP link enable, 5 breakpoint interrupt enable
bool linked() { return (cp_reg16[1] & 0x11) == 0x11; }
bool bp_reached() { return (cp_reg16[1] & 0x3) == 0x3; }

uint32_t be32(const uint8_t* p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }

int fmt_size(uint32_t fmt) { return fmt == 0 || fmt == 1 ? 1 : fmt == 2 || fmt == 3 ? 2 : 4; }

// bytes per vertex for vertex format `v`
int vertex_size(int v) {
    uint32_t lo = cp[0x50], hi = cp[0x60], a = cp[0x70 + v], b = cp[0x80 + v], c = cp[0x90 + v];
    auto index = [](uint32_t mode) { return mode == 2 ? 1 : mode == 3 ? 2 : 0; };
    int n = 0;
    for (int i = 0; i < 9; ++i) n += lo >> i & 1;                       // matrix indices
    uint32_t pos = lo >> 9 & 3, nrm = lo >> 11 & 3, col[2] = {lo >> 13 & 3, lo >> 15 & 3};
    if (pos == 1) n += ((a & 1) ? 3 : 2) * fmt_size(a >> 1 & 7);
    else n += index(pos);
    if (nrm) {
        int comps = (a >> 9 & 1) ? 9 : 3;
        if (nrm == 1) n += comps * fmt_size(a >> 10 & 7);
        else n += index(nrm) * ((comps == 9 && (a >> 31 & 1)) ? 3 : 1);
    }
    for (int i = 0; i < 2; ++i) {
        if (col[i] == 1) {
            static const int csz[8] = {2, 3, 4, 2, 3, 4, 4, 4};
            n += csz[(i ? a >> 18 : a >> 14) & 7];
        } else n += index(col[i]);
    }
    // texture coordinates: (count bit, format) for TEX0..TEX7
    const uint32_t tc[8][2] = {{a >> 21 & 1, a >> 22 & 7}, {b & 1, b >> 1 & 7}, {b >> 9 & 1, b >> 10 & 7},
                               {b >> 18 & 1, b >> 19 & 7}, {b >> 27 & 1, b >> 28 & 7}, {c >> 5 & 1, c >> 6 & 7},
                               {c >> 14 & 1, c >> 15 & 7}, {c >> 23 & 1, c >> 24 & 7}};
    for (int i = 0; i < 8; ++i) {
        uint32_t mode = hi >> (2 * i) & 3;
        if (mode == 1) n += (tc[i][0] ? 2 : 1) * fmt_size(tc[i][1]);
        else n += index(mode);
    }
    return n;
}

void bp_write(uint32_t v) {
    uint32_t reg = v >> 24, val = v & 0xFFFFFF;
    if (reg == 0xFE) { bp_mask = val; return; }
    bp[reg] = (bp[reg] & ~bp_mask) | (val & bp_mask);
    bp_mask = 0xFFFFFF;
    switch (reg) {
    case 0x45: pe_ctrl |= 8; ++st.done; os_raise(); break;           // PE_DONE: draw done
    case 0x47: pe_token = (uint16_t)val; break;                       // token
    case 0x48: pe_token = (uint16_t)val; pe_ctrl |= 4; os_raise(); break;   // token + interrupt
    case 0x52: ++st.copies; break;                                    // EFB copy
    }
}

// parse one command at p (n bytes available); returns its length, 0 if incomplete
size_t parse(const uint8_t* p, size_t n, bool in_dl);

void call_dl(uint32_t addr, uint32_t size) {
    ++st.dls;
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
        for (uint32_t i = 0; i < cnt; ++i)
            if (addr + i < sizeof xf / 4) xf[addr + i] = be32(p + 5 + 4 * i);
        return len;
    }
    case 0x20: case 0x28: case 0x30: case 0x38: return n < 5 ? 0 : 5;
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
        size_t len = 3 + (size_t)count * vertex_size(op & 7);
        if (n < len) return 0;
        ++st.draws;
        st.verts += count;
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

void gx_pipe_write(uint32_t v, int size) {
    uint8_t b[4];
    for (int i = 0; i < size; ++i) b[i] = (uint8_t)(v >> (8 * (size - 1 - i)));
    for (int i = 0; i < size; ++i) {
        uint32_t a = pi_wptr & 0x03FFFFFFu;
        *host(virt(a)) = b[i];
        ++a;
        if (pi_end && a >= (pi_end & 0x03FFFFFFu)) pi_wptr = (pi_base & 0x03FFFFFFu) | 0x20000000u;
        else pi_wptr = (pi_wptr & 0x20000000u) | a;
    }
    if (linked()) feed(b, size);
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
    rt_log("gx: %llu commands, %llu draws (%llu vertices), %llu display lists, %llu EFB copies, %llu draw-done",
           (unsigned long long)st.cmds, (unsigned long long)st.draws, (unsigned long long)st.verts,
           (unsigned long long)st.dls, (unsigned long long)st.copies, (unsigned long long)st.done);
}
