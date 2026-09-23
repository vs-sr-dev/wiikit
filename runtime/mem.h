// wiikit runtime — guest memory and executable loading.
#pragma once
#include <cstdint>
#include <cstddef>

// Reserve 4 GiB of host address space at g_mem and make MEM1 (0x80000000,
// 24 MiB), MEM2 (0x90000000, 64 MiB) and the locked cache (0xE0000000,
// 16 KiB) usable. The uncached mirrors at 0xC0000000 and 0xD0000000 are
// folded onto them by ppc_io_*. Returns false on failure.
bool mem_init();
// Load a DOL's text and data sections at their addresses and clear its bss.
// Returns the entry point, or 0 on failure.
uint32_t mem_load_dol(const char* path);
// Guest <-> host helpers for tests and HLE.
void mem_write(uint32_t addr, const void* src, size_t n);
void mem_read(uint32_t addr, void* dst, size_t n);
