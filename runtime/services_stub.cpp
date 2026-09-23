// wiikit runtime — the no-hardware services, for tests: anything a
// hardware-free run must never reach (system calls, traps, MMIO) aborts,
// time stands still and no interrupt ever arrives. wiikit_hw replaces it.
#include "rt.h"

std::atomic<uint32_t> g_ppc_pending{0};
void ppc_poll(PPCContext&) {}

void ppc_syscall(PPCContext&, uint32_t addr) { rt_die("sc at %08X", addr); }
void ppc_trap(PPCContext&, uint32_t addr) { rt_die("trap at %08X", addr); }
void ppc_unimplemented(PPCContext&, uint32_t addr, const char* what) { rt_die("%s at %08X", what, addr); }
uint32_t ppc_mmio_read(uint32_t addr, int) { rt_die("mmio read %08X", addr); }
void ppc_mmio_write(uint32_t addr, uint32_t, int) { rt_die("mmio write %08X", addr); }
uint64_t ppc_timebase() { return 0; }
void ppc_mttb(int, uint32_t) {}
void ppc_mtdec(uint32_t) {}
uint32_t ppc_mfdec() { return 0x7FFFFFFFu; }
