// wiikit runtime — the game's DATA partition (a GameCube disc: the whole disc),
// from an extracted tree (disc.cpp).
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

bool disc_open(const char* extract_dir);             // sys/ and files/ from wiikit.disc --extract
size_t disc_read(uint64_t off, void* dst, size_t n); // partition (GameCube: disc) offset; gaps read as zeros
const std::vector<uint8_t>& disc_boot();             // boot.bin: the disc header
bool disc_is_gamecube();                              // the header's GameCube magic (0xC2339F3D at 0x1C)
const std::vector<uint8_t>& disc_fst();
size_t disc_regions();
