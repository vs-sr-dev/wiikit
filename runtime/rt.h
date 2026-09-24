// wiikit runtime — what the runtime's own files share (not seen by recompiled code).
#pragma once
#include "ppc.h"
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <string>

// ---- core.cpp ---------------------------------------------------------------------------
PPCFunc ppc_lookup(uint32_t addr);
// Optional: a symbols.tsv (python -m wiikit.dol GAME.elf --symbols) names
// guest addresses in logs and crash reports.
void rt_load_symbols(const char* tsv);
std::string rt_name(uint32_t addr);               // "func+0x12", or the hex address
[[noreturn]] void rt_die(const char* fmt, ...);
void rt_log(const char* fmt, ...);                 // stderr, one line
// The guest call chain from a context: LR, then the saved LRs up the back chain.
void rt_backtrace(const PPCContext& c, FILE* out, int max = 24);

// A port's own layer: code linked into wiiboot next to the runtime (the
// project's WIIKIT_EXTRA adds it) registers an install function with a
// static RtGameLayer; wiiboot runs it after the runtime's own hooks, before
// the game starts. Its hooks come from the recompiler's --hooks file.
struct RtGameLayer { RtGameLayer(const char* name, void (*install)()); };
void rt_game_install();

// Registers of the guest code currently running on this host thread (for
// crash reports and "who touched this register" logs). Set by the OS layer.
extern thread_local PPCContext* t_ppc;

// ---- guest memory from the host side -----------------------------------------------------
static inline uint32_t virt(uint32_t phys) { return phys | 0x80000000u; }  // MEM1 and MEM2
static inline uint8_t* host(uint32_t guest) {
    if (guest >= 0xC0000000u && guest < 0xE0000000u) guest -= 0x40000000u;  // uncached mirrors
    return g_mem + guest;
}
std::string guest_cstr(uint32_t addr, size_t max = 256);

// ---- the OS layer (os.cpp) --------------------------------------------------------------
void os_install();                                // hooks, time
void os_start_main(uint32_t entry);               // run __start on a guest host thread, and return
void os_raise();                                  // a device changed its interrupt line
uint64_t os_tb_now();                             // guest time base now
void os_watch(int seconds);                       // report the running thread periodically
void os_profile();                                // WIIKIT_PROFILE: sample the guest code, list the hot functions

// ---- GX (gx.cpp) -----------------------------------------------------------------
void gx_init();                                   // after video_configure
void gx_report();                                 // command stream statistics

// ---- the hardware (hw.cpp) -----------------------------------------------------------
void hw_init();
bool hw_load_fonts(const char* dir);               // font_japanese.bin, font_western.bin
bool hw_external_pending();                        // PI cause & mask
void hw_vi_retrace();                              // called at each vertical retrace
std::chrono::steady_clock::time_point hw_tick();   // timed device events; returns the next one
extern bool g_mmio_log;                            // log every first access to a register

// ---- the Wii Remote (wpad.cpp) -------------------------------------------------------
void wpad_install();                               // WPAD/KPAD hooks

// ---- audio (ax.cpp, audio.cpp) --------------------------------------------------------
void ax_command_list(uint32_t addr);               // the AX micro-code: mix one frame
bool ax_load_coefs(const char* dir);               // dsp_coef.bin: the polyphase resampler's table
void audio_init(bool enabled);                     // the host's audio device (SDL3); false: none
// A block the AI DMA starts playing: frames of big-endian 16-bit stereo,
// right then left, as the Wii's AI reads them.
void audio_play(const uint8_t* be_rl, uint32_t frames, uint32_t rate);

// ---- IOS (ios.cpp) ------------------------------------------------------------------
void ios_init(const char* disc_root, const char* nand_root);
void ios_ipc_write(uint32_t reg, uint32_t v);      // 0xCD000000-0xCD00000C
uint32_t ios_ipc_read(uint32_t reg);
void ios_irq_flag_clear(uint32_t v);               // 0xCD000030
void ios_irq_mask_write(uint32_t v);               // 0xCD000034
uint32_t ios_irq_flags();
uint32_t ios_irq_mask();
