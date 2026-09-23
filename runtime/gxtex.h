// wiikit runtime — GX texture formats to RGBA8 (the C++ side of wiikit.gxtex).
#pragma once
#include <cstddef>
#include <cstdint>

// Formats: 0 I4, 1 I8, 2 IA4, 3 IA8, 4 RGB565, 5 RGB5A3, 6 RGBA8, 8 C4, 9 C8,
// 10 C14X2, 14 CMPR. Palette formats (TLUT): 0 IA8, 1 RGB565, 2 RGB5A3.
bool gxtex_known(uint32_t fmt);
size_t gxtex_size(uint32_t fmt, int w, int h);     // bytes of one level, whole blocks
bool gxtex_is_ci(uint32_t fmt);
int gxtex_ci_entries(uint32_t fmt);                // palette entries a CI format can index
// Decode one level; `tlut` (big-endian 16-bit entries) for the CI formats.
void gxtex_decode(uint32_t fmt, int w, int h, const uint8_t* src, uint8_t* rgba,
                  const uint8_t* tlut = nullptr, uint32_t tlut_fmt = 0);
