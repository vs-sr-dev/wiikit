// wiikit runtime — the hardware registers at 0xCC000000 and 0xCD000000.
//
// Every register reads back what was last written unless a device below
// gives it behaviour. Devices modelled so far, to the depth the SDK needs:
//   PI   interrupt cause and mask; the CPU FIFO registers (gx.cpp)
//   VI   the retrace interrupts (DI0-DI3), the beam position
//   DSP  reset/halt, mailboxes, ARAM DMA, the micro-codes (ROM, the audio
//        init code, AX: silent) and the AI DMA that paces audio frames
//   AI   the sample counter
//   EXI  three channels; channel 0 device 1 is the IPL chip: RTC, SRAM, UART,
//        and the boot ROM, which holds the system fonts (hw_load_fonts)
//   SI   no controllers: every transfer times out
//   CP, PE, the write-gather pipe: gx.cpp
//   Hollywood: IPC and its interrupt (ios.cpp), GPIOs, the I2C bus to the
//        AV encoder (answers ACK)
// With g_mmio_log, the first read and the first write of every register is
// logged with the guest function that made it: the to-do list of a new game.
// WIIKIT_DSPDBG=1 in the environment traces the DSP's mails and control.
#include "rt.h"
#include "video.h"
#include <chrono>
#include <cstdlib>
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
void gx_pipe_write(uint32_t v, int size);
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
    PI_SI = 0x8, PI_EXI = 0x10, PI_AI = 0x20, PI_DSP = 0x40, PI_VI = 0x100, PI_IPC = 0x4000,
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
uint16_t vi_read(uint32_t off) {
    if (off == 0x2C || off == 0x2E) {               // VCT, HCT: the beam, from the clock
        uint64_t ticks = os_tb_now() - vi_frame_tb;
        uint32_t line = (uint32_t)(ticks * 15734 / 60750000);   // NTSC line rate
        if (off == 0x2C) return (uint16_t)(1 + line % 263);
        return (uint16_t)(1 + (ticks * 15734 * 858 / 60750000) % 858);
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
//         call the resume callback that prepares the next frame. The lists
//         are not mixed yet: the frames are silent.
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
        if (dsp_cmdlist_next) {                      // the command list: done
            dsp_cmdlist_next = false;
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
    dsp_cr = (uint16_t)((v & ~(DSP_AIDINT | DSP_ARINT | DSP_DSPINT | DSP_RES | DSP_PIINT)) | keep);
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
void dsp_ar_dma() {                                  // ARAM DMA: no ARAM on the Wii, done at once
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
void ai_dma_block() {                                // a block starts: interrupt
    uint16_t ctl = (uint16_t)store_read(0xCC005036, 2);
    uint32_t rate = (ai_cr & 0x40) ? 32000 : 48000;  // AICR bit 6: DMA sample rate
    uint64_t bytes = (uint64_t)(ctl & 0x7FFF) * 32;
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
    return ai_count_base + (uint32_t)((os_tb_now() - ai_start_tb) * 48000 / 60750000);
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

// ---- SI -------------------------------------------------------------------------------------
uint32_t si_comcsr = 0, si_sr = 0;
bool si_irq() { return (si_comcsr & 0x80000000u) && (si_comcsr & 0x40000000u); }
uint32_t si_read(uint32_t off) {
    if (off == 0x34) return si_comcsr;
    if (off == 0x38) return si_sr;
    return store_read(0xCD006400 + off, 4);
}
void si_write(uint32_t off, uint32_t v) {
    if (off == 0x34) {
        uint32_t keep = si_comcsr & 0x80000000u & ~v;            // TCINT: write 1 to clear
        si_comcsr = (v & ~0x80000001u) | keep;
        if (v & 1) {                                              // TSTART: no device answers
            int ch = v >> 1 & 3;
            si_sr |= 0x08u << (8 * (3 - ch));                     // NOREP for that channel
            si_comcsr |= 0x80000000u | 0x20000000u;               // TCINT, COMERR
        }
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
        case 0x8: gx_pipe_write(v, size); return;
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

void ppc_mmio_write(uint32_t a, uint32_t v, int size) {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    if ((a & 0xFFFFF000u) != 0xCC008000u) log_access('W', a, size, v);
    mmio_write(a, v, size);
}

bool hw_external_pending() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    return (pi_cause() & pi_mask & ~PI_RSWST) != 0;
}

std::chrono::steady_clock::time_point hw_tick() {
    std::lock_guard<std::recursive_mutex> lk(g_hw);
    auto now = HostClock::now();
    if (!ai_dma_on || ai_dma_period.count() <= 0) return now + std::chrono::milliseconds(100);
    if (now >= ai_dma_next) {
        ai_dma_block();
        ai_dma_next += ai_dma_period;
        if (ai_dma_next <= now) ai_dma_next = now + ai_dma_period;
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

void hw_init() {
    sram_init();
    vi_frame_tb = os_tb_now();
}
