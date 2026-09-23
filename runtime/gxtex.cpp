// wiikit runtime — GX texture formats to RGBA8.
//
// Pixels are stored tile by tile in raster order, and in raster order inside
// each tile; every tile is 32 bytes (RGBA8: two 32-byte halves, AR then GB).
// Two details differ from the obvious reading and were measured against
// the hardware (wiikit.gxtex): I4 and I8 put the intensity in alpha too, and
// CMPR weights its interpolated colours 5/8 and 3/8, not 1/3 and 2/3.
#include "gxtex.h"
#include <cstring>

namespace {

struct Block { int w, h, bytes; };

Block block(uint32_t fmt) {
    switch (fmt) {
    case 0: case 8: case 14: return {8, 8, 32};
    case 1: case 2: case 9: return {8, 4, 32};
    case 6: return {4, 4, 64};
    default: return {4, 4, 32};             // IA8, RGB565, RGB5A3, C14X2
    }
}

inline uint8_t c3(uint32_t v) { return (uint8_t)(v << 5 | v << 2 | v >> 1); }
inline uint8_t c4(uint32_t v) { return (uint8_t)(v << 4 | v); }
inline uint8_t c5(uint32_t v) { return (uint8_t)(v << 3 | v >> 2); }
inline uint8_t c6(uint32_t v) { return (uint8_t)(v << 2 | v >> 4); }
inline uint16_t be16(const uint8_t* p) { return (uint16_t)(p[0] << 8 | p[1]); }

inline void rgb565(uint16_t v, uint8_t* o) {
    o[0] = c5(v >> 11 & 31); o[1] = c6(v >> 5 & 63); o[2] = c5(v & 31); o[3] = 255;
}
inline void rgb5a3(uint16_t v, uint8_t* o) {
    if (v & 0x8000) {
        o[0] = c5(v >> 10 & 31); o[1] = c5(v >> 5 & 31); o[2] = c5(v & 31); o[3] = 255;
    } else {
        o[0] = c4(v >> 8 & 15); o[1] = c4(v >> 4 & 15); o[2] = c4(v & 15); o[3] = c3(v >> 12 & 7);
    }
}
inline void ia8(uint16_t v, uint8_t* o) {  // high byte alpha, low byte intensity
    o[0] = o[1] = o[2] = (uint8_t)v; o[3] = (uint8_t)(v >> 8);
}
inline void palette(const uint8_t* tlut, uint32_t tfmt, uint32_t i, uint8_t* o) {
    uint16_t v = be16(tlut + 2 * i);
    if (tfmt == 0) ia8(v, o);
    else if (tfmt == 1) rgb565(v, o);
    else rgb5a3(v, o);
}

}  // namespace

bool gxtex_known(uint32_t fmt) { return fmt <= 6 || (fmt >= 8 && fmt <= 10) || fmt == 14; }
bool gxtex_is_ci(uint32_t fmt) { return fmt >= 8 && fmt <= 10; }
int gxtex_ci_entries(uint32_t fmt) { return fmt == 8 ? 16 : fmt == 9 ? 256 : 16384; }

size_t gxtex_size(uint32_t fmt, int w, int h) {
    Block b = block(fmt);
    return (size_t)((w + b.w - 1) / b.w) * ((h + b.h - 1) / b.h) * b.bytes;
}

void gxtex_decode(uint32_t fmt, int w, int h, const uint8_t* s, uint8_t* out, const uint8_t* tlut,
                  uint32_t tfmt) {
    Block b = block(fmt);
    auto px = [&](int x, int y) -> uint8_t* {
        static uint8_t sink[4];
        return x < w && y < h ? out + ((size_t)y * w + x) * 4 : sink;
    };
    for (int by = 0; by < h; by += b.h) {
        for (int bx = 0; bx < w; bx += b.w) {
            switch (fmt) {
            case 0:                                              // I4
                for (int y = 0; y < 8; ++y)
                    for (int x = 0; x < 8; x += 2, ++s) {
                        uint8_t* p = px(bx + x, by + y);
                        p[0] = p[1] = p[2] = p[3] = c4(*s >> 4);
                        p = px(bx + x + 1, by + y);
                        p[0] = p[1] = p[2] = p[3] = c4(*s & 15);
                    }
                break;
            case 1:                                              // I8
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 8; ++x, ++s) {
                        uint8_t* p = px(bx + x, by + y);
                        p[0] = p[1] = p[2] = p[3] = *s;
                    }
                break;
            case 2:                                              // IA4: alpha in the high nibble
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 8; ++x, ++s) {
                        uint8_t* p = px(bx + x, by + y);
                        p[0] = p[1] = p[2] = c4(*s & 15);
                        p[3] = c4(*s >> 4);
                    }
                break;
            case 3:                                              // IA8
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 4; ++x, s += 2) ia8(be16(s), px(bx + x, by + y));
                break;
            case 4:
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 4; ++x, s += 2) rgb565(be16(s), px(bx + x, by + y));
                break;
            case 5:
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 4; ++x, s += 2) rgb5a3(be16(s), px(bx + x, by + y));
                break;
            case 6:                                              // RGBA8: AR half, then GB half
                for (int i = 0; i < 16; ++i) {
                    uint8_t* p = px(bx + i % 4, by + i / 4);
                    p[3] = s[2 * i]; p[0] = s[2 * i + 1]; p[1] = s[32 + 2 * i]; p[2] = s[33 + 2 * i];
                }
                s += 64;
                break;
            case 8:                                              // C4
                for (int y = 0; y < 8; ++y)
                    for (int x = 0; x < 8; x += 2, ++s) {
                        palette(tlut, tfmt, *s >> 4, px(bx + x, by + y));
                        palette(tlut, tfmt, *s & 15, px(bx + x + 1, by + y));
                    }
                break;
            case 9:                                              // C8
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 8; ++x, ++s) palette(tlut, tfmt, *s, px(bx + x, by + y));
                break;
            case 10:                                             // C14X2
                for (int y = 0; y < 4; ++y)
                    for (int x = 0; x < 4; ++x, s += 2) palette(tlut, tfmt, be16(s) & 0x3FFF, px(bx + x, by + y));
                break;
            case 14:                                             // CMPR: four 4x4 DXT1-like sub-blocks
                for (int sb = 0; sb < 4; ++sb, s += 8) {
                    uint16_t k0 = be16(s), k1 = be16(s + 2);
                    uint8_t c[4][4];
                    rgb565(k0, c[0]);
                    rgb565(k1, c[1]);
                    for (int k = 0; k < 3; ++k) {
                        if (k0 > k1) {
                            c[2][k] = (uint8_t)((5 * c[0][k] + 3 * c[1][k]) >> 3);
                            c[3][k] = (uint8_t)((3 * c[0][k] + 5 * c[1][k]) >> 3);
                        } else {                             // the fourth: the average, transparent
                            c[2][k] = c[3][k] = (uint8_t)((c[0][k] + c[1][k]) / 2);
                        }
                    }
                    c[2][3] = 255;
                    c[3][3] = k0 > k1 ? 255 : 0;
                    uint32_t bits = (uint32_t)s[4] << 24 | s[5] << 16 | s[6] << 8 | s[7];
                    int ox = bx + (sb & 1) * 4, oy = by + (sb >> 1) * 4;
                    for (int i = 0; i < 16; ++i)
                        std::memcpy(px(ox + i % 4, oy + i / 4), c[bits >> (30 - 2 * i) & 3], 4);
                }
                break;
            default: return;
            }
        }
    }
}
