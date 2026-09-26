// wiikit runtime — the hardware registers at 0xCC000000 and 0xCD000000.
//
// Every register reads back what was last written unless a device below
// gives it behaviour. Devices modelled so far, to the depth the SDK needs:
//   PI   interrupt cause and mask; the CPU FIFO registers (gx.cpp)
//   VI   the retrace interrupts (DI0-DI3), the beam position
//   DSP  reset/halt, mailboxes, ARAM DMA (the GameCube's 16 MB of ARAM; none
//        on the Wii), the micro-codes (ROM, the audio init code, AX: the
//        mixer is ax.cpp) and the AI DMA that paces audio frames and hands
//        each block to the host's audio (audio.cpp)
//   AI   the sample counter
//   EXI  three channels; channel 0 device 1 is the IPL chip: RTC, SRAM, UART,
//        and the boot ROM, which holds the system fonts (hw_load_fonts)
//   DI   the GameCube's disc drive (the Wii's is IOS's): reads by DMA, the
//        drive's inquiry, error and audio-stream commands
//   SI   the GameCube's controllers (a GameCube game: a standard controller
//        on each port the host has a pad for); on the Wii none: every
//        transfer times out
//   CP, PE, the write-gather pipe: gx.cpp
//   Hollywood: IPC and its interrupt (ios.cpp), GPIOs, the I2C bus to the
//        AV encoder (answers ACK)
// With g_mmio_log, the first read and the first write of every register is
// logged with the guest function that made it: the to-do list of a new game.
// WIIKIT_DSPDBG=1 in the environment traces the DSP's mails and control.
#include "disc.h"
#include "rt.h"
#include "video.h"
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <mutex>
#include <set>
#include <vector>

bool g_mmio_log = false;

// gx.cpp
uint32_t gx_cp_read(uint32_t off, int size);
void gx_cp_write(uint32_t off, uint32_t v, int size);
uint32_t gx_pe_read(uint32_t off, int size);
void gx_pe_write(uint32_t off, uint32_t v, int size);
uint32_t gx_pi_fifo_read(uint32_t off);
void gx_pi_fifo_write(uint32_t off, uint32_t v);
void gx_pipe_burst(const uint8_t* b, int n);
uint32_t gx_irq();                                   // PI cause bits 9-11

namespace {

std::recursive_mutex g_hw;
uint8_t g_regs[0x20000];                             // default store: 0xCC00xxxx, 0xCD00xxxx
std::set<uint64_t> g_seen;

uint8_t* reg_ptr(uint32_t a) { return &g_regs[((a >> 24) & 1) << 16 | (a & 0xFFFF)]; }
uint32_t store_read(uint32_t a, int size) {
    uint8_t* p = reg_ptr(a);
    return size == 1 ? p[0] : size == 2 ? (uint32_t)(p[0] << 8 | p[1])
                                        : (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}
void store_write(uint32_t a, uint32_t v, int size) {
    uint8_t* p = reg_ptr(a);
    for (int i = size - 1; i >= 0; --i, v >>= 8) p[i] = (uint8_t)v;
}

void log_access(char rw, uint32_t a, int size, uint32_t v) {
    if (!g_mmio_log) return;
    uint64_t key = (uint64_t)a << 8 | (uint64_t)rw << 1 | (size == 4);
    if (!g_seen.insert(key).second) return;
    rt_log("mmio %c%d %08X = %08X   %s", rw, size * 8, a, v, t_ppc ? rt_name(t_ppc->lr).c_str() : "");
}

// ---- PI -----------------------------------------------------------------------------------
enum : uint32_t {
    PI_DI = 0x4, PI_SI = 0x8, PI_EXI = 0x10, PI_AI = 0x20, PI_DSP = 0x40, PI_VI = 0x100, PI_IPC = 0x4000,
    PI_RSWST = 0x10000,                              // reset button state: 1 = not pressed
};
uint32_t pi_mask = 0;

// ---- VI -----------------------------------------------------------------------------------
uint16_t vi[0x40];                                   // 0xCC002000, 16-bit registers
uint64_t vi_frame_tb = 0;                            // time base at the last retrace
bool vi_irq() {
    for (int i = 0; i < 4; ++i) {
        uint16_t hi = vi[0x18 + 2 * i];
        if ((hi & 0x8000) && (hi & 0x1000)) return true;
    }
    return false;
}
// The timing the game programmed: a half-line lasts HLW samples at 13.5 MHz
// (27 MHz when VICLK selects 54 MHz, progressive), and a field is 3 EQU +
// PRB + 2 ACV + PSB half-lines (VTR, and VTO/VTE for odd and even fields):
// NTSC and EuRGB60 525 x 429 (59.94 Hz), PAL 625 x 432 (50 Hz). Until it is
// programmed, NTSC.
struct ViTiming { uint32_t hlw, halflines; double sample_hz; };
ViTiming vi_timing() {
    uint32_t hlw = vi[0x03] & 0x1FF, equ = vi[0x00] & 0xF, acv = vi[0x00] >> 4 & 0x3FF;
    uint32_t odd = 3 * equ + (vi[0x07] & 0x3FF) + 2 * acv + (vi[0x06] & 0x3FF);
    uint32_t even = 3 * equ + (vi[0x09] & 0x3FF) + 2 * acv + (vi[0x08] & 0x3FF);
    double hz = (vi[0x36] & 1) ? 27e6 : 13.5e6;
    uint32_t hl = (odd + even) / 2;
    double field = hl * (double)hlw / hz;
    if (!hlw || field < 0.010 || field > 0.025) return {429, 525, 13.5e6};
    return {hlw, hl, hz};
}
uint16_t vi_read(uint32_t off) {
    if (off == 0x2C || off == 0x2E) {               // VCT, HCT: the beam, from the clock
        ViTiming t = vi_timing();
        double s = (double)(os_tb_now() - vi_frame_tb) / (double)tb_hz() * t.sample_hz;   // samples into the field
        uint32_t line = (uint32_t)(s / (2 * t.hlw));
        if (off == 0x2C) return (uint16_t)(1 + line % ((t.halflines + 1) / 2));
        return (uint16_t)(1 + (uint64_t)s % (2 * t.hlw));
    }
    if (off == 0x6E) return 0;                       // VISEL: composite cable
    return vi[off / 2];
}
void vi_write(uint32_t off, uint16_t v) {
    if (off == 0x02) v &= ~2;                        // DCR reset bit reads back 0
    static bool dbg = std::getenv("WIIKIT_VIDBG") != nullptr;
    if (dbg && vi[off / 2] != v && off != 0x1C && off != 0x1E && off != 0x24 && off != 0x26 && off < 0x30)
        rt_log("vi: %02X = %04X", off, v);
    vi[off / 2] = v;
    if (off == 0x00 || off == 0x02) {                // VTR's active lines per field, DCR's non-interlaced bit
        uint32_t acv = vi[0x00] >> 4 & 0x3FF;
        if (acv) video_set_lines(acv * (vi[0x01] & 4 ? 1 : 2));
    }
    if (off == 0x1C || off == 0x1E) {                // TFBL: the top field's XFB (POFF: address >> 5)
        uint32_t t = (uint32_t)vi[0x0E] << 16 | vi[0x0F];
        video_set_xfb((t & 0xFFFFFF) << (t >> 28 & 1 ? 5 : 0));
    }
}

// ---- DSP ------------------------------------------------------------------------------------
// DSPCR bits
enum : uint16_t {
    DSP_RES = 1, DSP_PIINT = 2, DSP_HALT = 4, DSP_AIDINT = 8, DSP_AIDINTMSK = 0x10,
    DSP_ARINT = 0x20, DSP_ARINTMSK = 0x40, DSP_DSPINT = 0x80, DSP_DSPINTMSK = 0x100,
    DSP_DMA = 0x200, DSP_INITCODE = 0x400, DSP_INIT = 0x800,
};
// The micro-codes, in high-level emulation:
//   ROM   after reset: announces itself (0x8071FEED), then takes a boot task
//         from the SDK's __DSP_boot_task, ten mails (0x80F3A001, IRAM
//         address, ..., 0x80F3D001, entry point), and starts it: always AX
//         so far
//   INIT  the 128-byte code __OSInitAudioSystem runs once: one mail, done
//   AX    the audio mixer. Init mail 0xDCD10000; then each audio frame the
//         SDK sends 0xBABE0000 | size and a command list address, and AX
//         yields (0xDCD10002) when done, which lets the SDK's task manager
//         call the resume callback that prepares the next frame. ax.cpp
//         mixes each list, the Wii's or the GameCube's.
// Mails the DSP sends to the task manager come with a DSP interrupt.
enum class Ucode { ROM, INIT, AX };
uint16_t dsp_cr = DSP_HALT;
Ucode dsp_ucode = Ucode::ROM;
std::deque<uint32_t> dsp_to_cpu;                     // mails from the DSP
uint32_t dsp_from_cpu = 0;                           // the CPU's mailbox (bit 31: not yet taken)
int dsp_boot_mails = 0;
bool dsp_cmdlist_next = false;
uint64_t dsp_frames = 0;
uint16_t dsp_ar[8];                                  // 0x20-0x2A: ARAM DMA
std::vector<uint8_t> aram;                           // the GameCube's ARAM (hw_init)
bool ar_busy = false;                                // an ARAM DMA runs until ar_done
std::chrono::steady_clock::time_point ar_done;
bool dsp_irq() {
    return ((dsp_cr & DSP_AIDINT) && (dsp_cr & DSP_AIDINTMSK)) ||
           ((dsp_cr & DSP_ARINT) && (dsp_cr & DSP_ARINTMSK)) ||
           ((dsp_cr & DSP_DSPINT) && (dsp_cr & DSP_DSPINTMSK));
}
bool dsp_dbg = std::getenv("WIIKIT_DSPDBG") != nullptr;
void dsp_send(uint32_t mail, bool irq) {
    if (dsp_dbg) rt_log("dsp: -> cpu %08X%s", mail, irq ? " (interrupt)" : "");
    dsp_to_cpu.push_back(mail);
    if (irq) dsp_cr |= DSP_DSPINT;
}
void dsp_start() {                                   // HALT went 1 -> 0: the micro-code runs
    switch (dsp_ucode) {
    case Ucode::ROM: dsp_send(0x8071FEED, false); dsp_boot_mails = 0; break;
    case Ucode::INIT: dsp_send(0x80544348, false); break;
    case Ucode::AX: break;
    }
}
void dsp_receive(uint32_t m) {                       // a mail from the CPU, taken at once
    if (dsp_dbg) rt_log("dsp: <- cpu %08X (micro-code %d)", m, (int)dsp_ucode);
    switch (dsp_ucode) {
    case Ucode::ROM:
        if (++dsp_boot_mails == 10) {                // the boot task is complete: AX starts
            dsp_ucode = Ucode::AX;
            dsp_cmdlist_next = false;
            dsp_send(0xDCD10000, true);
        }
        break;
    case Ucode::AX:
        if (dsp_cmdlist_next) {                      // the command list's address: mix a frame
            dsp_cmdlist_next = false;
            ax_command_list(m);
            ++dsp_frames;
            dsp_send(0xDCD10002, true);
        } else if ((m >> 16) == 0xBABE) {
            dsp_cmdlist_next = true;
        } else if (m != 0xCDD10003) {                // 0xCDD10003: carry on
            rt_log("dsp: AX mail %08X", m);
        }
        break;
    case Ucode::INIT:
        rt_log("dsp: mail %08X to the init code", m);
        break;
    }
}
void dsp_cr_write(uint16_t v) {
    uint16_t old = dsp_cr;
    if (dsp_dbg) rt_log("dsp: control %04X -> %04X", old, v);
    uint16_t keep = (uint16_t)(old & (DSP_AIDINT | DSP_ARINT | DSP_DSPINT) & ~v);  // write 1 to clear
    keep |= old & DSP_DMA;                           // read only: an ARAM DMA runs
    dsp_cr = (uint16_t)((v & ~(DSP_AIDINT | DSP_ARINT | DSP_DSPINT | DSP_RES | DSP_PIINT | DSP_DMA)) | keep);
    if (v & DSP_RES) {
        dsp_ucode = Ucode::ROM;
        dsp_to_cpu.clear();
    }
    if (!(v & DSP_INIT)) {                           // boot the 128 bytes at ARAM 0 (the init code)
        dsp_ucode = Ucode::INIT;
        dsp_cr &= ~DSP_INITCODE;
    }
    if ((old & DSP_HALT) && !(dsp_cr & DSP_HALT)) dsp_start();
    // PIINT (DSPAssertTask: "yield when you can"): AX has always finished already
}
// ARAM DMA: 0x20/0x22 main memory address, 0x24/0x26 ARAM address, 0x28/0x2A
// the count, its top bit the direction (1: ARAM to main memory). ARAM
// addresses wrap at its 16 MB, as on a console with no expansion: that is
// how ARInit's size check finds 16 MB. The bytes move at once, but the
// transfer lasts as long as on the console (about 0.5 us per 32 bytes, as
// Dolphin times it; DSPCR's DMA bit meanwhile) before its interrupt: games
// wait for it, and code that sees it finish too early takes other paths (a
// game whose sound driver reads a stack slot that the wait's own call would
// have written). No ARAM on the Wii: done at once.
void dsp_ar_dma() {
    if (!aram.empty()) {
        uint32_t mm = ((uint32_t)dsp_ar[0] << 16 | dsp_ar[1]) & 0x01FFFFE0u;
        uint32_t ar = (uint32_t)dsp_ar[2] << 16 | dsp_ar[3];
        uint32_t cnt = ((uint32_t)(dsp_ar[4] & 0x7FFF) << 16 | dsp_ar[5]) & ~31u;
        bool to_main = dsp_ar[4] & 0x8000;
        const uint32_t mask = (uint32_t)aram.size() - 1;
        for (uint32_t i = 0; i < cnt && mm + i < 0x01800000u; ++i) {
            uint8_t* m = host(virt(mm + i));
            uint8_t& a = aram[(ar + i) & mask];
            if (to_main) *m = a; else a = *m;
        }
        dsp_ar[4] &= 0x8000;
        dsp_ar[5] = 0;
        ar_busy = true;
        ar_done = std::chrono::steady_clock::now() + std::chrono::nanoseconds((cnt / 32 + 1) * 506);
        dsp_cr |= DSP_DMA;
        return;
    }
    dsp_cr |= DSP_ARINT;
}
// AI DMA (DSP registers 0x30-0x3A): the audio output. Each block of
// (control & 0x7FFF) * 32 bytes of 16-bit stereo plays at the AI's DMA rate;
// the AID interrupt comes as a block starts, the SDK's cue to queue the
// next one (AX mixes a frame per block).
using HostClock = std::chrono::steady_clock;
bool ai_dma_on = false;
HostClock::time_point ai_dma_next, ai_dma_block_start;
std::chrono::nanoseconds ai_dma_period{0};
uint32_t ai_cr = 0;
uint64_t ai_frames_then = ~0ull;                      // AX frames mixed when the last block started
bool ai_frame_ready() { return dsp_ucode != Ucode::AX || dsp_frames != ai_frames_then; }
void ai_dma_block() {                                // a block starts: its samples play, interrupt
    uint16_t ctl = (uint16_t)store_read(0xCC005036, 2);
    uint32_t rate = (ai_cr & 0x40) ? 32000 : 48000;  // AICR bit 6: DMA sample rate
    uint64_t bytes = (uint64_t)(ctl & 0x7FFF) * 32;
    uint32_t addr = (store_read(0xCC005030, 2) << 16 | store_read(0xCC005032, 2)) & 0x1FFFFFE0u;
    // AX mixes one frame per block, in the guest's AI interrupt. The clock
    // waits for it (ai_frame_ready); if the guest stays away past that, the
    // AI would play the previous frame again, a 3 ms buzz: such a block is
    // not played.
    static uint64_t stale = 0;
    bool fresh = ai_frame_ready();
    ai_frames_then = dsp_frames;
    if (fresh) audio_play(host(virt(addr)), (uint32_t)bytes / 4, rate);
    else if (++stale % 50 == 1) rt_log("audio: %llu blocks without a new AX frame", (unsigned long long)stale);
    ai_dma_period = std::chrono::nanoseconds(bytes * 1000000000ull / (4ull * rate));
    ai_dma_block_start = HostClock::now();
    dsp_cr |= DSP_AIDINT;
}
void ai_dma_write(uint16_t v) {
    bool on = v & 0x8000;
    if (on && !ai_dma_on) {
        store_write(0xCC005036, v, 2);
        ai_dma_block();
        ai_dma_next = HostClock::now() + ai_dma_period;
    }
    ai_dma_on = on;
}
uint16_t ai_dma_blocks_left() {
    uint16_t ctl = (uint16_t)store_read(0xCC005036, 2) & 0x7FFF;
    if (!ai_dma_on || ai_dma_period.count() <= 0) return 0;
    auto done = HostClock::now() - ai_dma_block_start;
    int64_t left = (int64_t)ctl - (int64_t)(ctl * done.count() / ai_dma_period.count());
    return (uint16_t)(left < 0 ? 0 : left > ctl ? ctl : left);
}

uint16_t dsp_read(uint32_t off) {
    switch (off) {
    case 0x00: return (uint16_t)(dsp_from_cpu >> 16);
    case 0x02: return (uint16_t)dsp_from_cpu;
    case 0x04: return dsp_to_cpu.empty() ? 0 : (uint16_t)(0x8000 | (dsp_to_cpu.front() >> 16 & 0x7FFF));
    case 0x06: {
        if (dsp_to_cpu.empty()) return 0;
        uint16_t lo = (uint16_t)dsp_to_cpu.front();
        dsp_to_cpu.pop_front();
        return lo;
    }
    case 0x0A: return dsp_cr;
    case 0x16:                                       // AR_MODE: bit 0, the ARAM controller is ready
        return (uint16_t)(store_read(0xCC005016, 2) | (aram.empty() ? 0 : 1));
    case 0x3A: return ai_dma_blocks_left();
    }
    if (off >= 0x20 && off < 0x30) return dsp_ar[(off - 0x20) / 2];
    return (uint16_t)store_read(0xCC005000 + off, 2);
}
void dsp_write(uint32_t off, uint16_t v) {
    switch (off) {
    case 0x00: dsp_from_cpu = (dsp_from_cpu & 0xFFFF) | (uint32_t)v << 16; return;
    case 0x02:                                       // the low half sends the mail
        // bit 31 is the mailbox's "full" flag, set by this write whatever the
        // value; the DSP takes the mail at once and the flag drops
        dsp_from_cpu = (dsp_from_cpu & 0xFFFF0000u) | v;
        dsp_receive(dsp_from_cpu | 0x80000000u);
        dsp_from_cpu &= 0x7FFFFFFFu;
        return;
    case 0x0A: dsp_cr_write(v); return;
    case 0x36: ai_dma_write(v); break;
    }
    if (off >= 0x20 && off < 0x30) {
        dsp_ar[(off - 0x20) / 2] = v;
        if (off == 0x2A) dsp_ar_dma();               // writing the count's low half starts it
        return;
    }
    store_write(0xCC005000 + off, v, 2);
}

// ---- AI -------------------------------------------------------------------------------------
uint32_t ai_count_base = 0;                           // AICR (ai_cr) is with the AI DMA above
uint64_t ai_start_tb = 0;
uint32_t ai_samples() {
    if (!(ai_cr & 1)) return ai_count_base;
    return ai_count_base + (uint32_t)((os_tb_now() - ai_start_tb) * 48000 / tb_hz());
}

// ---- EXI ------------------------------------------------------------------------------------
struct Exi { uint32_t csr, mar, len, cr, data; };
Exi exi[3];
uint8_t sram[64];
std::vector<uint8_t> rom(0x200000);                  // the boot ROM: only its two fonts
uint32_t ipl_cmd = 0;
int ipl_cursor = 0;
std::string uart_line;

void sram_init() {
    std::memset(sram, 0, sizeof sram);
    sram[0x13] = 0x2C;                               // flags as Dolphin's default: sound stereo, etc.
    uint16_t sum = 0, inv = 0;
    for (int i = 0x0C; i < 0x14; i += 2) {
        uint16_t w = (uint16_t)(sram[i] << 8 | sram[i + 1]);
        sum += w;
        inv += (uint16_t)~w;
    }
    sram[0] = sum >> 8; sram[1] = (uint8_t)sum; sram[2] = inv >> 8; sram[3] = (uint8_t)inv;
}

uint32_t rtc_now() { return (uint32_t)(std::time(nullptr) - 946684800); }   // seconds since 2000

// one byte through the IPL chip: 4 command bytes, then data at (cmd >> 6) onwards
uint8_t ipl_byte(uint8_t in) {
    if (ipl_cursor < 4) {
        ipl_cmd = ipl_cmd << 8 | in;
        ++ipl_cursor;
        return 0;
    }
    bool write = ipl_cmd & 0x80000000u;
    uint32_t a = ((ipl_cmd & 0x7FFFFFFFu) >> 6) + (uint32_t)(ipl_cursor++ - 4);
    if (a >= 0x800000 && a < 0x800004) {
        uint32_t t = rtc_now();
        return write ? 0 : (uint8_t)(t >> (8 * (3 - (a - 0x800000))));
    }
    if (a >= 0x800004 && a < 0x800044) {
        if (write) { sram[a - 0x800004] = in; return 0; }
        return sram[a - 0x800004];
    }
    if (a >= 0x800400 && a < 0x800500) {             // the debug UART
        if (write) {
            if (in == '\n' || in == '\r') {
                if (!uart_line.empty()) rt_log("uart: %s", uart_line.c_str());
                uart_line.clear();
            } else if (in) uart_line += (char)in;
        }
        return 0;
    }
    if (a < 0x200000) return write ? 0 : rom[a];     // the boot ROM
    rt_log("exi: IPL %s at %06X", write ? "write" : "read", a);
    return 0;
}

uint8_t exi_byte(int ch, int dev, uint8_t in) {
    if (ch == 0 && dev == 1) return ipl_byte(in);
    return 0xFF;                                     // nothing attached
}

void exi_select(int ch, uint32_t new_csr) {
    uint32_t old_cs = exi[ch].csr >> 7 & 7, cs = new_csr >> 7 & 7;
    if (ch == 0 && (old_cs & 2) && !(cs & 2)) { ipl_cursor = 0; ipl_cmd = 0; }
}

void exi_transfer(int ch) {
    Exi& e = exi[ch];
    int cs = e.csr >> 7 & 7, dev = cs & 1 ? 0 : cs & 2 ? 1 : cs & 4 ? 2 : -1;
    int rw = e.cr >> 2 & 3;
    if (e.cr & 2) {                                  // DMA
        for (uint32_t i = 0; i < e.len; ++i) {
            uint32_t a = virt(e.mar + i);
            if (rw == 1) exi_byte(ch, dev, ld8(a));
            else st8(a, exi_byte(ch, dev, 0));
        }
    } else {                                         // immediate, 1-4 bytes, MSB first
        int n = (e.cr >> 4 & 3) + 1;
        uint32_t out = 0;
        for (int i = 0; i < n; ++i) {
            uint8_t b = exi_byte(ch, dev, (uint8_t)(e.data >> (24 - 8 * i)));
            out |= (uint32_t)b << (24 - 8 * i);
        }
        if (rw != 1) e.data = out;
    }
    e.cr &= ~1u;
    e.csr |= 8;                                      // TCINT
}

bool exi_irq() {
    for (auto& e : exi)
        if (((e.csr & 8) && (e.csr & 4)) || ((e.csr & 2) && (e.csr & 1))) return true;
    return false;
}

uint32_t exi_read(uint32_t off) {
    int ch = off / 0x14;
    if (ch > 2) return store_read(0xCD006800 + off, 4);
    Exi& e = exi[ch];
    switch (off % 0x14) {
    case 0x00: return e.csr;
    case 0x04: return e.mar;
    case 0x08: return e.len;
    case 0x0C: return e.cr;
    default: return e.data;
    }
}

void exi_write(uint32_t off, uint32_t v) {
    int ch = off / 0x14;
    if (ch > 2) { store_write(0xCD006800 + off, v, 4); return; }
    Exi& e = exi[ch];
    switch (off % 0x14) {
    case 0x00: {
        exi_select(ch, v);
        uint32_t clear = v & (2 | 8 | 0x800);        // interrupt status bits: write 1 to clear
        e.csr = (v & ~(2u | 8u | 0x800u)) | (e.csr & (2 | 8 | 0x800) & ~clear);
        break;
    }
    case 0x04: e.mar = v & 0x1FFFFFE0u; break;      // MEM1 or MEM2 (physical 0x10000000)
    case 0x08: e.len = v & 0x03FFFFE0u; break;
    case 0x0C: e.cr = v; if (v & 1) exi_transfer(ch); break;
    default: e.data = v; break;
    }
}

// ---- DI (GameCube) --------------------------------------------------------------------------
// The GameCube's SDK drives the disc drive itself: a command in DICMDBUF0-2
// (the command byte on top), a DMA address and length, and DICR's TSTART.
// A command ends with the transfer interrupt, the data in memory then, after
// the time a drive takes: 1 ms, and reads at 16 MB/s (faster than the
// console's drive, about 3 MB/s). Never at once: games set their "busy" flag
// after DVDReadAsync returns, and a read whose callbacks all ran before that
// leaves them busy for ever. DISR bits: 1/2 the error interrupt's mask and
// status, 3/4 the transfer's, 5/6 the break's; DICVR: 0 the cover (open),
// 1/2 its interrupt's mask and status.
enum { DI_SR, DI_CVR, DI_CMD0, DI_CMD1, DI_CMD2, DI_MAR, DI_LEN, DI_CR, DI_IMM, DI_CFG, DI_REGS };
uint32_t di[DI_REGS];
bool di_busy = false;                                // a command runs until di_done
std::chrono::steady_clock::time_point di_done;
bool di_irq() {
    uint32_t s = di[DI_SR];
    return ((s & 0x06) == 0x06) || ((s & 0x18) == 0x18) || ((s & 0x60) == 0x60) ||
           ((di[DI_CVR] & 0x06) == 0x06);
}
void di_command() {                                  // TSTART: the command starts
    uint32_t len = (di[DI_CMD0] >> 24) == 0xA8 ? di[DI_LEN] & ~31u : 0;
    di_busy = true;
    di_done = std::chrono::steady_clock::now() + std::chrono::microseconds(1000 + len / 16);
}
void di_finish() {                                   // its time has passed: its effect, the interrupt
    uint32_t cmd = di[DI_CMD0] >> 24;
    static const char* dbg = std::getenv("WIIKIT_DIDBG");       // =bt: with the guest's call chain
    if (dbg) {
        rt_log("di: %08X %08X %08X -> %08X +%X", di[DI_CMD0], di[DI_CMD1], di[DI_CMD2], di[DI_MAR], di[DI_LEN]);
        if (!std::strcmp(dbg, "bt") && t_ppc) rt_backtrace(*t_ppc, stderr, 10);
    }
    uint32_t mar = di[DI_MAR] & 0x01FFFFE0u, len = di[DI_LEN] & ~31u;
    switch (cmd) {
    case 0xA8:                                       // read (0xA8000040: the disc id, at 0)
        if (mar + len > 0x01800000u) { rt_log("di: read past MEM1 to %08X", mar); break; }
        disc_read((uint64_t)di[DI_CMD1] << 2, host(virt(mar)), len);
        di[DI_MAR] = mar + len;
        di[DI_LEN] = 0;
        break;
    case 0x12: {                                     // inquiry: the drive's revision, as Dolphin's
        const uint8_t rev[32] = {0, 0, 0, 2, 0x20, 0x06, 0x05, 0x26, 0x41};
        uint32_t n = std::min<uint32_t>(len, 32);
        if (mar + n <= 0x01800000u) std::memcpy(host(virt(mar)), rev, n);
        di[DI_MAR] = mar + n;
        di[DI_LEN] = 0;
        break;
    }
    case 0xE0: di[DI_IMM] = 0; break;               // request error: none
    case 0xE2: di[DI_IMM] = 0; break;               // audio stream status: not playing
    case 0xAB: case 0xE1: case 0xE3: case 0xE4: break;   // seek, audio stream, stop motor, stream buffer
    default: rt_log("di: command %08X %08X %08X", di[DI_CMD0], di[DI_CMD1], di[DI_CMD2]); break;
    }
    di[DI_CR] &= ~1u;
    di[DI_SR] |= 0x10;                               // TCINT
}
uint32_t di_read(uint32_t off) { return off / 4 < DI_REGS ? di[off / 4] : 0; }
void di_write(uint32_t off, uint32_t v) {
    if (off / 4 >= DI_REGS) return;
    switch (off / 4) {
    case DI_SR: di[DI_SR] = (v & 0x2B) | (di[DI_SR] & 0x54 & ~v); break;    // interrupts: write 1 to clear
    case DI_CVR: di[DI_CVR] = (v & 0x02) | (di[DI_CVR] & 0x05 & ~(v & 0x04)); break;
    case DI_CR: di[DI_CR] = v; if ((v & 1) && !di_busy) di_command(); break;
    default: di[off / 4] = v; break;
    }
}

// ---- SI -------------------------------------------------------------------------------------
// Four ports: 0x00 + 12 * port its poll command (OUTBUF), 0x04/0x08 the
// last poll's answer (INBUFH/L); 0x30 SIPOLL (bits 7..4 enable ports 0..3),
// 0x34 COMCSR (TCINT 31, its mask 30, COMERR 29, RDSTINT 28, its mask 27;
// the lengths out 16-22 and in 8-14, 0 meaning 128; the port 1-2; TSTART 0),
// 0x38 SISR (a byte a port, port 0 on top: NOREP 0x08, RDST 0x20), 0x80 the
// transfer buffer. A GameCube game finds a standard controller on each port
// the host has a pad for (video_classic: port 1 is also the keyboard), as
// Dolphin answers for one: its type, its origin, and its state, read by the
// poll at each retrace in analog mode 3. The Wii's SDK finds none.
uint32_t si_comcsr = 0, si_sr = 0, si_poll = 0;
uint32_t si_out[4], si_in[4][2];
bool si_irq() {
    return ((si_comcsr & 0x80000000u) && (si_comcsr & 0x40000000u)) ||
           ((si_comcsr & 0x10000000u) && (si_comcsr & 0x08000000u));
}
// port 1 is also the keyboard: there from the start, before the window has read a pad
bool si_pad(int port) { return g_gamecube && (port == 0 || video_classic(port).connected); }

// The controller's state: buttons (with USE_ORIGIN), the stick, then in
// mode 3 the C stick and the triggers. The Classic's buttons as the pad's:
// + is START, ZL and ZR are Z; L and R press their triggers fully. The
// port's filter sees it first, as a Wii game's Classic.
void si_pad_state(int port, uint32_t& hi, uint32_t& lo) {
    ClassicState s = video_classic(port);
    wpad_filter_classic(port, s);
    static const uint32_t map[][2] = {{0x0010, 0x0100}, {0x0040, 0x0200}, {0x0008, 0x0400}, {0x0020, 0x0800},
                                      {0x0400, 0x1000}, {0x0004, 0x0010}, {0x0080, 0x0010}, {0x2000, 0x0040},
                                      {0x0200, 0x0020}, {0x0001, 0x0008}, {0x4000, 0x0004}, {0x0002, 0x0001},
                                      {0x8000, 0x0002}};
    uint32_t b = 0x0080;
    for (auto& m : map)
        if (s.buttons & m[0]) b |= m[1];
    auto axis = [](float v) { int a = 0x80 + (int)(v * 100.0f); return (uint32_t)(a < 0 ? 0 : a > 255 ? 255 : a); };
    auto trig = [](float v) { int a = (int)(v * 255.0f); return (uint32_t)(a < 0 ? 0 : a > 255 ? 255 : a); };
    hi = b << 16 | axis(s.lx) << 8 | axis(s.ly);
    lo = axis(s.rx) << 24 | axis(s.ry) << 16 | trig(s.lt) << 8 | trig(s.rt);
}

// A transfer: the command's first byte in the buffer, the answer written back there
bool si_dbg = std::getenv("WIIKIT_SIDBG") != nullptr;
void si_transfer(uint32_t csr) {
    int port = csr >> 1 & 3;
    uint32_t in_len = csr >> 8 & 0x7F;
    if (!in_len) in_len = 128;
    uint8_t* buf = reg_ptr(0xCD006480);
    if (si_dbg) rt_log("si: port %d command %02X, %u bytes back%s", port, buf[0], in_len, si_pad(port) ? "" : ", nothing there");
    if (!si_pad(port)) {
        si_sr |= 0x08u << (8 * (3 - port));                       // NOREP for that port
        si_comcsr |= 0x80000000u | 0x20000000u;                   // TCINT, COMERR
        return;
    }
    uint8_t reply[10] = {};
    uint32_t n = 0;
    switch (buf[0]) {
    case 0x00: case 0xFF:                                         // reset, type: a standard controller
        reply[0] = 0x09; n = 3; break;
    case 0x41: case 0x42:                                         // origin, recalibrate: centred sticks
        reply[1] = 0x80; reply[2] = reply[3] = reply[4] = reply[5] = 0x80; n = 10; break;
    case 0x40: {                                                  // the state, directly
        uint32_t hi, lo;
        si_pad_state(port, hi, lo);
        for (int i = 0; i < 4; ++i) { reply[i] = (uint8_t)(hi >> (24 - 8 * i)); reply[4 + i] = (uint8_t)(lo >> (24 - 8 * i)); }
        n = 8;
        break;
    }
    default:
        rt_log("si: command %02X to port %d", buf[0], port);
        break;
    }
    std::memset(buf, 0, std::min<uint32_t>(in_len, 128));
    std::memcpy(buf, reply, std::min(n, in_len));
    si_comcsr = (si_comcsr & ~0x20000000u) | 0x80000000u;        // TCINT, no COMERR
}

// The poll at a retrace: each enabled port's answer, RDST, the interrupt
void si_poll_ports() {
    bool any = false;
    for (int port = 0; port < 4; ++port) {
        if (!(si_poll & (0x80u >> port))) continue;
        uint32_t shift = 8 * (3 - port);
        if (si_pad(port)) {
            si_pad_state(port, si_in[port][0], si_in[port][1]);
            si_sr |= 0x20u << shift;                               // RDST
            any = true;
        } else {
            si_in[port][0] = 0x80000000u;                         // ERRSTAT: nothing there
            si_in[port][1] = 0;
            si_sr |= 0x08u << shift;                               // NOREP
        }
    }
    if (any) si_comcsr |= 0x10000000u;                            // RDSTINT
}

uint32_t si_read(uint32_t off) {
    if (off < 0x30) {
        int port = off / 12;
        switch (off % 12) {
        case 0: return si_out[port];
        case 4:                                                   // reading INBUFH takes the answer
            si_sr &= ~(0x20u << (8 * (3 - port)));
            if (!(si_sr & 0x20202020u)) si_comcsr &= ~0x10000000u;
            return si_in[port][0];
        default: return si_in[port][1];
        }
    }
    if (off == 0x30) return si_poll;
    if (off == 0x34) return si_comcsr;
    if (off == 0x38) return si_sr;
    return store_read(0xCD006400 + off, 4);
}
void si_write(uint32_t off, uint32_t v) {
    if (off < 0x30) {
        if (off % 12 == 0) {
            int port = off / 12;
            if (g_gamecube && (si_out[port] & 3) != (v & 3)) video_set_rumble(port, (v & 3) == 1);
            si_out[port] = v;
        }
        return;
    }
    if (off == 0x30) {
        if (si_dbg && v != si_poll) rt_log("si: poll %08X", v);
        si_poll = v;
        return;
    }
    if (off == 0x34) {
        // TCINT, RDSTINT: write 1 to clear; COMERR is the last transfer's, read only
        uint32_t keep = (si_comcsr & 0x90000000u & ~v) | (si_comcsr & 0x20000000u);
        si_comcsr = (v & ~0xB0000001u) | keep;
        if (v & 1) si_transfer(v);
        return;
    }
    if (off == 0x38) { si_sr &= ~(v & 0x0F0F0F0Fu); return; }    // error bits: write 1 to clear
    store_write(0xCD006400 + off, v, 4);
}

// ---- Hollywood ------------------------------------------------------------------------------
uint32_t gpiob_out = 0;
enum : uint32_t { GPIO_AVE_SCL = 0x4000, GPIO_AVE_SDA = 0x8000 };

uint32_t hollywood_read(uint32_t off) {
    switch (off) {
    case 0x00: case 0x04: case 0x08: case 0x0C: return ios_ipc_read(off / 4);
    case 0x30: return ios_irq_flags();
    case 0x34: return ios_irq_mask();
    case 0xC0: return gpiob_out;
    case 0xC8: return gpiob_out & GPIO_AVE_SCL;     // I2C: SDA low = every byte is ACKed
    }
    return store_read(0xCD000000 + off, 4);
}
void hollywood_write(uint32_t off, uint32_t v) {
    switch (off) {
    case 0x00: case 0x04: case 0x08: case 0x0C: ios_ipc_write(off / 4, v); return;
    case 0x30: ios_irq_flag_clear(v); return;
    case 0x34: ios_irq_mask_write(v); return;
    case 0xC0: gpiob_out = v; return;
    }
    store_write(0xCD000000 + off, v, 4);
}

uint32_t pi_cause() {
    uint32_t c = PI_RSWST | gx_irq();
    if (vi_irq()) c |= PI_VI;
    if (dsp_irq()) c |= PI_DSP;
    if (exi_irq()) c |= PI_EXI;
    if (si_irq()) c |= PI_SI;
    if (g_gamecube && di_irq()) c |= PI_DI;
    if (ios_irq_flags() & ios_irq_mask()) c |= PI_IPC;
    return c;
}

// 16-bit register files accessed at any width
uint32_t read16x(uint16_t (*rd)(uint32_t), uint32_t off, int size) {
    if (size == 4) return (uint32_t)rd(off) << 16 | rd(off + 2);
    uint16_t v = rd(off & ~1u);
    return size == 2 ? v : (off & 1 ? v & 0xFF : v >> 8);
}
void write16x(uint16_t (*rd)(uint32_t), void (*wr)(uint32_t, uint16_t), uint32_t off, uint32_t v, int size) {
    if (size == 4) { wr(off, (uint16_t)(v >> 16)); wr(off + 2, (uint16_t)v); return; }
    if (size == 2) { wr(off, (uint16_t)v); return; }
    uint16_t cur = rd(off & ~1u);
    wr(off & ~1u, off & 1 ? (uint16_t)((cur & 0xFF00) | (v & 0xFF)) : (uint16_t)((cur & 0xFF) | v << 8));
}

uint32_t mmio_read(uint32_t a, int size) {
    uint32_t off = a & 0xFFFF;
    if ((a & 0xFFFF0000u) == 0xCC000000u) {
        switch (off >> 12) {
        case 0x0: return gx_cp_read(off, size);
        case 0x1: return gx_pe_read(off & 0xFFF, size);
        case 0x2: return read16x(vi_read, off & 0xFFF, size);
        case 0x3:
            if ((off & 0xFFF) == 0x00) return pi_cause();
            if ((off & 0xFFF) == 0x04) return pi_mask;
            if ((off & 0xFFF) >= 0x0C && (off & 0xFFF) <= 0x14) return gx_pi_fifo_read(off & 0xFFF);
            if ((off & 0xFFF) == 0x2C) return 0x246500B1;       // Flipper revision C
            break;
        case 0x5: return read16x(dsp_read, off & 0xFFF, size);
        case 0x6: return mmio_read(0xCD000000u | off, size);    // the legacy mirror of DI/SI/EXI/AI
        }
        return store_read(a, size);
    }
    // 0xCD000000: Hollywood; 0xCD8xxxxx mirrors it
    if ((a & 0x00FF0000u) == 0x00800000u) return mmio_read(0xCD000000u | off, size);
    if (off < 0x400) {
        uint32_t w = hollywood_read(off & ~3u);
        if (size == 4) return w;
        if (size == 2) return w >> (16 - 8 * (off & 2)) & 0xFFFF;
        return w >> (24 - 8 * (off & 3)) & 0xFF;
    }
    if (g_gamecube && off >= 0x6000 && off < 0x6040) return di_read(off - 0x6000);
    if (off >= 0x6400 && off < 0x6500) return si_read(off - 0x6400);
    if (off >= 0x6800 && off < 0x6840) return exi_read(off - 0x6800);
    if (off >= 0x6C00 && off < 0x6C20) {
        switch (off - 0x6C00) {
        case 0x00: return ai_cr;
        case 0x08: return ai_samples();
        }
    }
    return store_read(a, size);
}

void mmio_write(uint32_t a, uint32_t v, int size) {
    uint32_t off = a & 0xFFFF;
    if ((a & 0xFFFF0000u) == 0xCC000000u) {
        switch (off >> 12) {
        case 0x0: gx_cp_write(off, v, size); return;
        case 0x1: gx_pe_write(off & 0xFFF, v, size); return;
        case 0x2: write16x(vi_read, vi_write, off & 0xFFF, v, size); return;
        case 0x3:
            if ((off & 0xFFF) == 0x00) return;                  // latched causes: none modelled
            if ((off & 0xFFF) == 0x04) { pi_mask = v; os_raise(); return; }
            if ((off & 0xFFF) >= 0x0C && (off & 0xFFF) <= 0x14) { gx_pi_fifo_write(off & 0xFFF, v); return; }
            break;
        case 0x5: write16x(dsp_read, dsp_write, off & 0xFFF, v, size); os_raise(); return;
        case 0x6: mmio_write(0xCD000000u | off, v, size); return;
        case 0x8: return;                            // the gather pipe: ppc_mmio_write
        }
        store_write(a, v, size);
        return;
    }
    if ((a & 0x00FF0000u) == 0x00800000u) { mmio_write(0xCD000000u | off, v, size); return; }
    if (off < 0x400) {
        if (size != 4) rt_die("%d-bit write to Hollywood register %08X", size * 8, a);
        hollywood_write(off, v);
        os_raise();
        return;
    }
    if (g_gamecube && off >= 0x6000 && off < 0x6040) { di_write(off - 0x6000, v); os_raise(); return; }
    if (off >= 0x6400 && off < 0x6500) { si_write(off - 0x6400, v); os_raise(); return; }
    if (off >= 0x6800 && off < 0x6840) { exi_write(off - 0x6800, v); os_raise(); return; }
    if (off >= 0x6C00 && off < 0x6C20) {
        if (off == 0x6C00) {
            if ((v & 1) && !(ai_cr & 1)) { ai_start_tb = os_tb_now(); }
            if (!(v & 1) && (ai_cr & 1)) ai_count_base = ai_samples();
            if (v & 0x20) { ai_count_base = 0; ai_start_tb = os_tb_now(); }   // SCRESET
            ai_cr = v & ~0x28u;
            return;
        }
    }
    store_write(a, v, size);
}

}  // namespace

uint32_t ppc_mmio_read(uint32_t a, int size) {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    uint32_t v = mmio_read(a, size);
    log_access('R', a, size, v);
    return v;
}

// the write-gather pipe's buffer (gx.cpp): only the running guest thread
// stores into it, so it needs no lock until it bursts
uint8_t gather[64];
int gathered = 0;

// Writing WPAR resets the gather buffer, as on the 750CL (and in Dolphin):
// bytes short of a burst are dropped. GXRedirectWriteGatherPipe pads the pipe
// with zeros, then writes WPAR before pointing it at a buffer in memory; the
// padding's remainder must not land at the start of that buffer.
void ppc_wpar_write(uint32_t) { gathered = 0; }

void ppc_mmio_write(uint32_t a, uint32_t v, int size) {
    if ((a & 0xFFFFF000u) == 0xCC008000u) {
        for (int i = 0; i < size; ++i) gather[gathered++] = (uint8_t)(v >> (8 * (size - 1 - i)));
        if (gathered >= 32) {
            {
                std::lock_guard<std::recursive_mutex> lk(g_hw);
                gx_pipe_burst(gather, 32);
            }
            gathered -= 32;
            std::memmove(gather, gather + 32, (size_t)gathered);
            gx_submit_pending();
        }
        return;
    }
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    if ((a & 0xFFFFF000u) != 0xCC008000u) log_access('W', a, size, v);
    mmio_write(a, v, size);
}

std::chrono::steady_clock::time_point ar_tick();
std::chrono::steady_clock::time_point di_tick();

bool hw_external_pending() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    ios_tick();
    ar_tick();
    di_tick();
    return (pi_cause() & pi_mask & ~PI_RSWST) != 0;
}

std::chrono::steady_clock::time_point ai_tick();
std::chrono::steady_clock::time_point ar_tick();
void ios_idle();

void hw_cpu_idle() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    ios_idle();
}

// the clock thread's device work: IPC replies falling due, AI blocks
std::chrono::steady_clock::time_point hw_tick() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    return std::min({ios_tick(), ai_tick(), ar_tick(), di_tick()});
}

std::chrono::steady_clock::time_point di_tick() {    // a disc command's end
    auto now = HostClock::now();
    if (!di_busy) return now + std::chrono::milliseconds(100);
    if (now < di_done) return di_done;
    di_busy = false;
    di_finish();
    os_raise();
    return now + std::chrono::milliseconds(100);
}

std::chrono::steady_clock::time_point ar_tick() {    // an ARAM DMA's end: its interrupt
    auto now = HostClock::now();
    if (!ar_busy) return now + std::chrono::milliseconds(100);
    if (now < ar_done) return ar_done;
    ar_busy = false;
    dsp_cr = (uint16_t)((dsp_cr & ~DSP_DMA) | DSP_ARINT);
    os_raise();
    return now + std::chrono::milliseconds(100);
}

std::chrono::steady_clock::time_point ai_tick() {
    auto now = HostClock::now();
    if (!ai_dma_on || ai_dma_period.count() <= 0) return now + std::chrono::milliseconds(100);
    if (now >= ai_dma_next) {
        // The next block waits for the guest's mix of it: its AI interrupt
        // can come late (on the console the CPU takes it at once; here the
        // game's thread may be decoding a frame's vertices), and a block
        // without its frame is a gap in the sound. The host's audio queue
        // absorbs the wait.
        if (!ai_frame_ready() && now - ai_dma_next < std::chrono::milliseconds(50))
            return now + std::chrono::microseconds(250);
        ai_dma_block();
        // a late block is followed by the next as soon as it is mixed, back
        // on the schedule, so the AI keeps real time; a long stall (the game
        // stopped mixing) starts it afresh
        ai_dma_next += ai_dma_period;
        if (now - ai_dma_next > std::chrono::milliseconds(100)) ai_dma_next = now + ai_dma_period;
        os_raise();
    }
    return ai_dma_next;
}

void hw_vi_retrace() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    vi_frame_tb = os_tb_now();
    bool any = false;
    for (int i = 0; i < 4; ++i) {
        uint16_t& hi = vi[0x18 + 2 * i];
        if (hi & 0x1000) { hi |= 0x8000; any = true; }
    }
    if (g_gamecube && (si_poll & 0xF0)) {
        si_poll_ports();
        any |= si_irq();
    }
    if (any) os_raise();
    video_retrace();
}

// The fonts OSInitFont reads from the boot ROM, Yay0-compressed: Shift-JIS at
// 0x1AFF00, Windows-1252 at 0x1FCF00. Dolphin's free replacements (Droid Sans,
// in its Sys/GC folder) have the right format.
bool hw_load_fonts(const char* dir) {
    bool ok = true;
    for (auto [name, off] : {std::pair<const char*, uint32_t>{"font_japanese.bin", 0x1AFF00},
                             {"font_western.bin", 0x1FCF00}}) {
        std::string p = std::string(dir) + "/" + name;
        FILE* f = std::fopen(p.c_str(), "rb");
        if (!f) { ok = false; continue; }
        size_t n = std::fread(&rom[off], 1, rom.size() - off, f);
        (void)n;
        std::fclose(f);
    }
    return ok;
}

// The IPL leaves VI running in the console's format; VIInit reads the current
// format from DCR and VIConfigure refuses to switch NTSC <-> PAL. A running
// VI also means VIInit skips its own set-up, display interrupts included, so
// they are preset too, as Dolphin's VideoInterface::Preset does. An NTSC boot
// keeps VI at zero (VIInit then sets everything itself), as before.
void hw_vi_preset(bool pal) {
    if (!pal) return;
    vi[0x00] = 6;                                    // VTR: EQU 6, ACV 0
    vi[0x01] = 1 << 8 | 1;                           // DCR: FMT = PAL, ENB
    vi[0x18] = 0x1000 | 263;                         // DI0: enabled, line 263
    vi[0x19] = 430;                                  //      pixel 430
    vi[0x1A] = 0x1000 | 1;                           // DI1: enabled, line 1
    vi[0x1B] = 1;
}

void hw_run_locked(void (*fn)()) {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    fn();
}

std::chrono::nanoseconds hw_vi_field_period() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    ViTiming t = vi_timing();
    return std::chrono::nanoseconds((int64_t)(t.halflines * (double)t.hlw / t.sample_hz * 1e9));
}

uint8_t aram_read(uint32_t addr) {
    return aram.empty() ? 0 : aram[addr & (aram.size() - 1)];
}

void hw_init() {
    sram_init();
    if (g_gamecube) aram.assign(0x01000000, 0);
    vi_frame_tb = os_tb_now();
}
