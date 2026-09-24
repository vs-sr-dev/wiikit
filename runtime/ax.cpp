// wiikit runtime — the AX micro-code: the DSP's audio mixer, in C++.
//
// The SDK's AX library hands the DSP one command list per audio frame
// (3 ms, 96 samples at 32 kHz): set up the mixing buffers, run the list of
// voice parameter blocks (PBs), pass the aux buses through the CPU's effect
// callbacks, and write the stereo mix where the AI DMA will play it. This
// file does what the micro-code does with that list, as Dolphin's AXWii HLE
// describes it: the "accelerator" decodes DSP-ADPCM and PCM from main memory,
// a 4-tap polyphase (or linear) resampler converts the rate, a volume
// envelope, a one-pole low-pass and a biquad shape the voice, and per-channel
// volume ramps mix it into main, aux A/B/C and the Remotes' speakers.
//
// The PB layout is the one of the Wii SDK of 2009-2010 (checked against
// Victorious's AXSetVoice* stores): no per-millisecond update field, the
// full biquad, 0x140 bytes a PB. Addresses the DSP sees are physical.
//
// The polyphase coefficients live in the DSP's ROM; Dolphin's free
// dsp_coef.bin has them (ax_load_coefs). Without it the resampler is linear.
#include "rt.h"
#include <algorithm>
#include <cstring>

namespace {

constexpr int N = 96;                                // samples per frame
constexpr int NWM = 18;                              // Remote speaker samples per frame (6 kHz)

int16_t coefs[0x800];
bool have_coefs = false;

// mixing buffers, in SETUP's order
enum { ML, MR, MS, AL, AR, AS, BL, BR, BS, CL, CR, CS, NMAIN };
int32_t mix[NMAIN][N];
int32_t wm[8][NWM];                                  // Remote 0 main, aux; Remote 1 main, aux; ...
uint16_t last_main_volume = 0x8000, last_aux_volume[3] = {0x8000, 0x8000, 0x8000};
uint16_t compressor_pos = 0;

uint32_t dsp_virt(uint32_t phys) { return virt(phys & 0x1FFFFFFF); }
int16_t clamp16(int64_t v) { return (int16_t)std::clamp<int64_t>(v, -0x8000, 0x7FFF); }

// ---- the parameter block ------------------------------------------------------------------
// Word offsets (u16) into the PB.
enum : int {
    NEXT = 0, SRC_TYPE = 4, COEF_SELECT = 5, MIXER_CONTROL = 6, RUNNING = 8, IS_STREAM = 9,
    MIXER = 10,              // 12 pairs (volume, delta): main L R, aux A L R, B L R, C L R, then S of main, A, B, C
    ITD_ON = 34,
    DPOP = 41,               // main A B C left, main A B C right, main A B C surround
    VE = 53,                 // volume envelope: current, delta per sample
    LOOPING = 55, FORMAT = 56, LOOP_ADDR = 57, END_ADDR = 59, CUR_ADDR = 61,
    ADPCM_COEFS = 63, GAIN = 79, PRED_SCALE = 80, YN1 = 81, YN2 = 82,
    SRC_RATIO = 83, SRC_FRAC = 85, SRC_LAST = 86,
    LOOP_PRED_SCALE = 90, LOOP_YN1 = 91, LOOP_YN2 = 92,
    LPF = 93,                // on, yn1, a0, b0
    BIQUAD = 97,             // on, xn1, xn2, yn1, yn2, b0, b1, b2, a1, a2
    REMOTE = 107, REMOTE_MIXER_CONTROL = 108,
    REMOTE_MIXER = 109,      // 8 pairs: main0, aux0, main1, aux1, ... main3, aux3
    REMOTE_DPOP = 125,       // main0..3, aux0..3
    REMOTE_SRC_FRAC = 133, REMOTE_SRC_LAST = 134,
    REMOTE_IIR = 138,        // on (0 off, 2 biquad, else low-pass), then the filter's fields
    PB_WORDS = 0x140 / 2,
};
struct PB {
    uint16_t w[PB_WORDS];
    uint32_t u32(int i) const { return (uint32_t)w[i] << 16 | w[i + 1]; }
    void set32(int i, uint32_t v) { w[i] = (uint16_t)(v >> 16); w[i + 1] = (uint16_t)v; }
    int16_t& s(int i) { return reinterpret_cast<int16_t&>(w[i]); }
};
void read_pb(uint32_t addr, PB& pb) { for (int i = 0; i < PB_WORDS; ++i) pb.w[i] = ld16(dsp_virt(addr) + 2 * i); }
void write_pb(uint32_t addr, const PB& pb) { for (int i = 0; i < PB_WORDS; ++i) st16(dsp_virt(addr) + 2 * i, pb.w[i]); }

// ---- the accelerator: sample fetch and decode ------------------------------------------------
// Addresses count samples of the format: nibbles for ADPCM, bytes for 8-bit,
// half-words for 16-bit. Reaching the end address raises an "exception"
// the micro-code answers by looping or by stopping the voice.
struct Accel {
    PB* pb;
    uint32_t start, end, cur;
    uint16_t format, pred_scale;
    int16_t yn1, yn2, gain;
    bool stopped = false;

    explicit Accel(PB& p) : pb(&p) {
        start = p.u32(LOOP_ADDR) & 0x3FFFFFFF;
        end = p.u32(END_ADDR) & 0x3FFFFFFF;
        cur = p.u32(CUR_ADDR) & 0xBFFFFFFF;
        format = p.w[FORMAT];
        yn1 = p.s(YN1);
        yn2 = p.s(YN2);
        gain = p.s(GAIN);
        pred_scale = p.w[PRED_SCALE] & 0x7F;
    }
    uint16_t fetch() const {
        switch (format & 3) {
        case 0: { uint8_t b = ld8(dsp_virt(cur >> 1)); return (cur & 1) ? (b & 0xF) : (b >> 4); }
        case 1: return ld8(dsp_virt(cur));
        case 2: return ld16(dsp_virt(cur * 2));
        default: return 0;
        }
    }
    void end_reached() {
        if (pb->w[LOOPING]) {
            pred_scale = pb->w[LOOP_PRED_SCALE] & 0x7F;
            if (pb->w[IS_STREAM] != 1) { yn1 = pb->s(LOOP_YN1); yn2 = pb->s(LOOP_YN2); }
            stopped = false;                         // the micro-code rewrites YN2, which resumes reads
        } else {
            pb->w[RUNNING] = 0;                      // a one-shot voice is over
        }
    }
    int16_t sample() {
        if (stopped) return 0;
        int16_t raw = (int16_t)fetch();
        int ci = (pred_scale >> 4) & 7;
        int32_t c1 = pb->s(ADPCM_COEFS + 2 * ci), c2 = pb->s(ADPCM_COEFS + 2 * ci + 1);
        int16_t val = 0;
        int step = 2;
        uint16_t decode = (format >> 2) & 3;
        if (decode == 0) {                           // DSP-ADPCM: 4-bit nibbles, a header byte every 8 bytes
            raw &= 0xF;
            if (raw >= 8) raw -= 16;
            int32_t v = (1 << (pred_scale & 0xF)) * raw + ((0x400 + c1 * yn1 + c2 * yn2) >> 11);
            val = clamp16(v);
            yn2 = yn1;
            yn1 = val;
            cur += 1;
            if ((end & 0xF) == 0 && cur == end) cur = start + 1;
            else if ((end & 0xF) == 1 && cur == end - 1) cur = start;
            else if ((cur & 15) == 0) {
                pred_scale = ld8(dsp_virt((cur & ~15u) >> 1)) & 0x7F;
                cur += 2;
                step += 2;
            }
        } else {                                     // PCM, scaled by the gain
            int shift = ((format >> 4) & 3) == 1 ? 0 : ((format >> 4) & 3) == 2 ? 16 : 11;
            int32_t v = (((int32_t)gain * raw) >> shift) + ((c1 * yn1) >> shift) + ((c2 * yn2) >> shift);
            val = (int16_t)v;
            yn2 = yn1;
            yn1 = val;
            if (decode != 1) cur += 1;
        }
        if (cur == end + step - 1) {                 // past the end: back to the loop start
            cur = start;
            stopped = true;
            end_reached();
        }
        cur &= 0xBFFFFFFF;
        return val;
    }
    void store() {
        pb->set32(CUR_ADDR, cur);
        pb->s(YN1) = yn1;
        pb->s(YN2) = yn2;
        pb->w[PRED_SCALE] = pred_scale;
    }
};

// ---- resampling ------------------------------------------------------------------------------
// ratio and pos are 16.16 fixed point; last[4] carries the input history
// from one frame to the next.
template <class In>
uint32_t resample(In in, int16_t* out, int count, int16_t* last, uint32_t pos, uint32_t ratio,
                  int type, const int16_t* c) {
    int16_t t[4];
    uint32_t idx = 0;
    for (int i = 0; i < 4; ++i) t[idx++ & 3] = last[i];
    int read = 0;
    if (type != 0 && type != 1) {                    // no conversion
        for (int i = 0; i < count; ++i) out[i] = in(i);
        if (count >= 4) std::memcpy(last, out + count - 4, 8);
        return pos;
    }
    for (int i = 0; i < count; ++i) {
        pos += ratio;
        while (pos >= 0x10000) { t[idx++ & 3] = in(read++); pos -= 0x10000; }
        if (c && type == 0) {                        // 4-tap polyphase: 64 phases of 4 coefficients
            const int16_t* k = c + (((pos & 0xFFFF) >> 9) << 2);
            int64_t s = 0;
            for (int j = 0; j < 4; ++j) s += (int64_t)t[idx++ & 3] * k[j];
            out[i] = clamp16(s >> 15);
        } else {                                     // linear, between the two newest-but-one samples
            uint16_t f = pos & 0xFFFF;
            if (f) {
                int32_t s0 = t[idx++ & 3], s1 = t[idx++ & 3];
                out[i] = (int16_t)((s0 * (uint16_t)-f + s1 * f) >> 16);
                idx += 2;
            } else {
                out[i] = t[idx++ & 3];
                idx += 3;
            }
        }
    }
    for (int i = 3; i >= 0; --i) last[i] = t[--idx & 3];
    return pos;
}

// ---- one voice -------------------------------------------------------------------------------
void mix_add(int32_t* out, const int16_t* in, int count, PB& pb, int vol, int16_t& dpop, bool ramp) {
    uint16_t& v = pb.w[vol];
    uint16_t d = ramp ? pb.w[vol + 1] : 0;
    for (int i = 0; i < count; ++i) {
        int16_t s = clamp16(((int64_t)in[i] * v) >> 15);
        out[i] += s;
        v += d;
        dpop = s;
    }
}

void low_pass(int16_t* s, int count, PB& pb, int f) {    // on, yn1, a0 (unsigned), b0
    for (int i = 0; i < count; ++i)
        pb.s(f + 1) = s[i] = clamp16(((int32_t)pb.w[f + 2] * s[i] + (int32_t)pb.s(f + 3) * pb.s(f + 1)) >> 15);
}

void biquad(int16_t* s, int count, PB& pb, int f) {      // on, xn1, xn2, yn1, yn2, b0, b1, b2, a1, a2
    for (int i = 0; i < count; ++i) {
        int16_t x = s[i];
        int64_t t = (int64_t)pb.s(f + 5) * x + (int64_t)pb.s(f + 6) * pb.s(f + 1) + (int64_t)pb.s(f + 7) * pb.s(f + 2) +
                    (int64_t)pb.s(f + 8) * pb.s(f + 3) + (int64_t)pb.s(f + 9) * pb.s(f + 4);
        t <<= 2;
        t += (t & 0x10000) ? 0x8000 : 0x7FFF;              // the DSP's rounding
        int16_t y = clamp16(t >> 16);
        pb.s(f + 2) = pb.s(f + 1);
        pb.s(f + 4) = pb.s(f + 3);
        pb.s(f + 1) = x;
        pb.s(f + 3) = y;
        s[i] = y;
    }
}

void process_voice(PB& pb) {
    if (pb.w[RUNNING] != 1) return;
    int16_t s[N];
    Accel acc(pb);
    const int16_t* c = have_coefs ? coefs + (pb.w[COEF_SELECT] & 3) * 0x200 : nullptr;
    uint32_t pos = resample([&](int) { return acc.sample(); }, s, N, &pb.s(SRC_LAST), pb.w[SRC_FRAC],
                            pb.u32(SRC_RATIO), pb.w[SRC_TYPE], c);
    pb.w[SRC_FRAC] = (uint16_t)pos;
    acc.store();

    for (int i = 0; i < N; ++i) {                    // the volume envelope (unsigned on the Wii)
        s[i] = clamp16(((int32_t)s[i] * (int32_t)pb.w[VE]) >> 15);
        pb.w[VE] += pb.w[VE + 1];
    }
    if (pb.w[LPF]) low_pass(s, N, pb, LPF);
    if (pb.w[BIQUAD]) biquad(s, N, pb, BIQUAD);

    // mixer control: per bus, bit "on" and bit "on with ramps" for L+R, then S
    uint32_t mc = pb.u32(MIXER_CONTROL);
    struct Bus { int l, r, s; uint32_t lb, rb, lrramp, sb, sramp; int vl, vr, vs, dl, dr, ds; };
    static const Bus buses[4] = {
        {ML, MR, MS, 1u << 0, 1u << 1, 1u << 2, 1u << 3, 1u << 4, MIXER + 0, MIXER + 2, MIXER + 16, DPOP + 0, DPOP + 4, DPOP + 8},
        {AL, AR, AS, 1u << 16, 1u << 17, 1u << 18, 1u << 19, 1u << 20, MIXER + 4, MIXER + 6, MIXER + 18, DPOP + 1, DPOP + 5, DPOP + 9},
        {BL, BR, BS, 1u << 21, 1u << 22, 1u << 23, 1u << 24, 1u << 25, MIXER + 8, MIXER + 10, MIXER + 20, DPOP + 2, DPOP + 6, DPOP + 10},
        {CL, CR, CS, 1u << 26, 1u << 27, 1u << 28, 1u << 29, 1u << 30, MIXER + 12, MIXER + 14, MIXER + 22, DPOP + 3, DPOP + 7, DPOP + 11},
    };
    for (const Bus& b : buses) {
        bool lr_ramp = mc & b.lrramp, s_ramp = mc & b.sramp;
        if ((mc & b.lb) || lr_ramp) mix_add(mix[b.l], s, N, pb, b.vl, pb.s(b.dl), lr_ramp);
        if ((mc & b.rb) || lr_ramp) mix_add(mix[b.r], s, N, pb, b.vr, pb.s(b.dr), lr_ramp);
        if ((mc & b.sb) || s_ramp) mix_add(mix[b.s], s, N, pb, b.vs, pb.s(b.ds), s_ramp);
    }

    if (pb.w[REMOTE]) {                              // the Remotes' speakers: 6 kHz
        static bool told = false;
        if (!told) { told = true; rt_log("ax: a voice plays on a Remote's speaker"); }
        if (pb.w[REMOTE_IIR] == 2) biquad(s, N, pb, REMOTE_IIR);
        else if (pb.w[REMOTE_IIR]) low_pass(s, N, pb, REMOTE_IIR);
        int16_t w[NWM];
        uint32_t p = resample([&](int i) { return s[i]; }, w, NWM, &pb.s(REMOTE_SRC_LAST), pb.w[REMOTE_SRC_FRAC],
                              0x55555, 0, have_coefs ? coefs : nullptr);
        pb.w[REMOTE_SRC_FRAC] = (uint16_t)p;
        uint16_t rmc = pb.w[REMOTE_MIXER_CONTROL];
        for (int ch = 0; ch < 8; ++ch) {             // main0, aux0, main1, aux1, ...
            uint16_t bits = (rmc >> (2 * ch)) & 3;
            int dpop = REMOTE_DPOP + (ch & 1 ? 4 : 0) + ch / 2;
            if (bits) mix_add(wm[ch], w, NWM, pb, REMOTE_MIXER + 2 * ch, pb.s(dpop), bits & 2);
        }
    }
}

void process_pbs(uint32_t addr) {
    PB pb;
    for (int guard = 0; addr && guard < 1024; ++guard) {
        read_pb(addr, pb);
        process_voice(pb);
        write_pb(addr, pb);
        addr = pb.u32(NEXT);
    }
}

// ---- the other commands -------------------------------------------------------------------
void setup(uint32_t addr) {                          // each buffer: a 32-bit start value and a delta
    uint32_t a = dsp_virt(addr);
    auto init = [&](int32_t* b, int n) {
        int32_t v = (int32_t)ld32(a);
        int16_t d = (int16_t)ld16(a + 4);
        a += 6;
        for (int i = 0; i < n; ++i) b[i] = v ? v + i * d : 0;
    };
    for (int i = 0; i < NMAIN; ++i) init(mix[i], N);
    for (int i = 0; i < 8; ++i) init(wm[i], NWM);
}

void volume_ramp(uint16_t* out, uint16_t from, uint16_t to) {
    float v = from;
    for (int i = 0; i < N; ++i) { v += (float)(to - from) / N; out[i] = (uint16_t)v; }
}

void upload(uint32_t addr, const int32_t* b, int n) {
    for (int i = 0; i < n; ++i) st32(dsp_virt(addr) + 4 * i, (uint32_t)b[i]);
}

void add_to_lr(uint32_t addr, bool neg) {
    for (int i = 0; i < N; ++i) {
        int32_t v = (int32_t)ld32(dsp_virt(addr) + 4 * i);
        if (neg) v = -v;
        mix[ML][i] += v;
        mix[MR][i] += v;
    }
}

void add_sub_to_lr(uint32_t addr) {
    for (int i = 0; i < N; ++i) mix[ML][i] += (int32_t)ld32(dsp_virt(addr) + 4 * i);
    for (int i = 0; i < N; ++i) mix[MR][i] -= (int32_t)ld32(dsp_virt(addr) + 4 * (N + i));
}

// an aux bus goes to the CPU (its effect callback), and last frame's result comes back into main
void mix_aux(int aux, uint32_t write_addr, uint32_t read_addr, uint16_t volume) {
    uint16_t ramp[N];
    volume_ramp(ramp, last_aux_volume[aux], volume);
    last_aux_volume[aux] = volume;
    int base = AL + 3 * aux;
    if (write_addr)
        for (int k = 0; k < 3; ++k) upload(write_addr + k * 4 * N, mix[base + k], N);
    uint32_t a = dsp_virt(read_addr);
    for (int k = 0; k < 3; ++k)
        for (int i = 0; i < N; ++i, a += 4)
            mix[ML + k][i] += (int32_t)(((int64_t)(int32_t)ld32(a) * ramp[i]) >> 15);
}

void upload_aux_mix_lrsc(int aux, const uint32_t* addr, uint16_t volume) {   // Dolby Pro Logic II mode
    int base = aux ? BL : AL;
    upload(addr[0], mix[base], N);
    upload(addr[0] + 4 * N, mix[base + 1], N);
    upload(addr[0] + 8 * N, mix[base + 2], N);
    upload(addr[1], mix[aux ? CS : CR], N);
    uint16_t ramp[N];
    volume_ramp(ramp, last_aux_volume[aux], volume);
    last_aux_volume[aux] = volume;
    int32_t* dst[4] = {mix[ML], mix[MR], mix[MS], mix[CL]};
    for (int k = 0; k < 4; ++k)
        for (int i = 0; i < N; ++i)
            dst[k][i] += (int32_t)(((int64_t)(int32_t)ld32(dsp_virt(addr[2 + k]) + 4 * i) * ramp[i]) >> 15);
}

void compressor(uint16_t threshold, uint16_t release_frames, uint32_t table) {
    bool hit = false;
    for (int i = 0; i < N && !hit; ++i)
        hit = std::abs(mix[ML][i]) > (int)threshold || std::abs(mix[MR][i]) > (int)threshold;
    uint32_t entry;
    if (hit) { entry = compressor_pos; compressor_pos = release_frames; }
    else if (compressor_pos) { --compressor_pos; entry = 11 + compressor_pos; }   // release ramps follow 11 attack ramps
    else return;
    uint32_t a = dsp_virt(table) + entry * N * 2;
    for (int i = 0; i < N; ++i) {
        uint16_t k = ld16(a + 2 * i);
        mix[ML][i] = (int32_t)(((int64_t)mix[ML][i] * k) >> 15);
        mix[MR][i] = (int32_t)(((int64_t)mix[MR][i] * k) >> 15);
    }
}

// the frame's result: surround (and aux C in DPL2 mode) as 32-bit samples,
// and main L/R clamped to 16 bits, interleaved right then left, for the AI
void output(uint32_t lr_addr, uint32_t surround_addr, uint16_t volume, bool dpl2) {
    uint16_t ramp[N];
    volume_ramp(ramp, last_main_volume, volume);
    last_main_volume = volume;
    upload(surround_addr, mix[MS], N);
    if (dpl2) upload(surround_addr + 4 * N, mix[CL], N);
    uint32_t a = dsp_virt(lr_addr);
    for (int i = 0; i < N; ++i) {
        st16(a + 4 * i, (uint16_t)clamp16(((int64_t)mix[MR][i] * ramp[i]) >> 15));
        st16(a + 4 * i + 2, (uint16_t)clamp16(((int64_t)mix[ML][i] * ramp[i]) >> 15));
    }
}

void output_remotes(const uint32_t* addr) {
    for (int r = 0; r < 4; ++r)
        for (int i = 0; i < NWM; ++i) st16(dsp_virt(addr[r]) + 2 * i, (uint16_t)clamp16(wm[2 * r][i]));
}

}  // namespace

bool ax_load_coefs(const char* dir) {
    std::string p = std::string(dir) + "/dsp_coef.bin";
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) return false;
    uint8_t b[sizeof coefs];
    size_t n = std::fread(b, 1, sizeof b, f);
    std::fclose(f);
    if (n != sizeof b) return false;
    for (size_t i = 0; i < 0x800; ++i) coefs[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]);
    have_coefs = true;
    return true;
}

// One command list (the micro-code's "0xBABE" task): runs to the END command.
void ax_command_list(uint32_t addr) {
    uint32_t a = dsp_virt(addr);
    auto next = [&] { uint16_t v = ld16(a); a += 2; return v; };
    auto next32 = [&] { uint32_t hi = next(); return hi << 16 | next(); };
    for (int guard = 0; guard < 256; ++guard) {
        uint16_t cmd = next();
        switch (cmd) {
        case 0x00: setup(next32()); break;
        case 0x01: add_to_lr(next32(), false); break;
        case 0x02: add_to_lr(next32(), true); break;
        case 0x03: add_sub_to_lr(next32()); break;
        case 0x04: process_pbs(next32()); break;
        case 0x05: case 0x06: case 0x07: {
            uint16_t vol = next();
            uint32_t w = next32(), r = next32();
            mix_aux(cmd - 0x05, w, r, vol);
            break;
        }
        case 0x08: case 0x09: {
            uint16_t vol = next();
            uint32_t ad[6];
            for (uint32_t& x : ad) x = next32();
            upload_aux_mix_lrsc(cmd - 0x08, ad, vol);
            break;
        }
        case 0x0A: {
            uint16_t threshold = next(), frames = next();
            compressor(threshold, frames, next32());
            break;
        }
        case 0x0B: case 0x0C: {
            uint16_t vol = next();
            uint32_t surround = next32(), lr = next32();
            output(lr, surround, vol, cmd == 0x0C);
            break;
        }
        case 0x0D: {
            uint32_t ad[4];
            for (uint32_t& x : ad) x = next32();
            output_remotes(ad);
            break;
        }
        case 0x0E: return;
        default:
            rt_log("ax: unknown command %04X in the list at %08X", cmd, addr);
            return;
        }
    }
}
