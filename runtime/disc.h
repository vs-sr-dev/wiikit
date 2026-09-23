// wiikit runtime — the game's DATA partition, from an extracted tree (disc.cpp).
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

bool disc_open(const char* extract_dir);             // sys/ and files/ from wiikit.disc --extract
size_t disc_read(uint64_t off, void* dst, size_t n); // partition offset; gaps read as zeros
const std::vector<uint8_t>& disc_boot();             // boot.bin: the disc header
const std::vector<uint8_t>& disc_fst();
size_t disc_regions();
