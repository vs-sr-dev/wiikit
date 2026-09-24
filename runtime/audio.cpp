// wiikit runtime — the host's audio: what the AI DMA plays goes to an SDL3
// audio stream.
//
// The AI's pace comes from the clock thread (hw.cpp: one block per period of
// host time), the device's from its own crystal; the two drift apart slowly,
// and the guest can be late with a frame. The stream starts, and restarts
// after running dry, only with kTarget of sound queued, a cushion against
// that jitter; after that the stream's playback speed is nudged (at most
// 2%, not audible) to hold the queue at kTarget. What is heard stays about
// kTarget behind what the game counts: that matters for the rhythm game,
// whose beats come from the audio clock. Past kMaxQueue a block is dropped.
//
// WIIKIT_AUDIODBG=1 reports the queue and the speed every few seconds.
//
// WIIKIT_AUDIODUMP=file.wav writes everything played, for checking the mix
// without ears.
#include "rt.h"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

constexpr double kTarget = 0.02, kMaxQueue = 0.1;   // seconds
SDL_AudioStream* stream = nullptr;
bool dbg = std::getenv("WIIKIT_AUDIODBG") != nullptr;
double smoothed = 0;                                 // queued seconds, low-passed
bool playing = false;
uint64_t underruns = 0;
uint32_t stream_rate = 0;
uint64_t dropped = 0;
FILE* dump = nullptr;
uint32_t dump_rate = 0, dump_bytes = 0;

void wav_header(FILE* f, uint32_t rate, uint32_t bytes) {
    auto u32 = [&](uint32_t v) { uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)}; std::fwrite(b, 1, 4, f); };
    auto u16 = [&](uint16_t v) { uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)}; std::fwrite(b, 1, 2, f); };
    std::fseek(f, 0, SEEK_SET);
    std::fwrite("RIFF", 1, 4, f); u32(36 + bytes); std::fwrite("WAVEfmt ", 1, 8, f);
    u32(16); u16(1); u16(2); u32(rate); u32(rate * 4); u16(4); u16(16);
    std::fwrite("data", 1, 4, f); u32(bytes);
    std::fseek(f, 0, SEEK_END);
}

void close_dump() {
    if (!dump) return;
    wav_header(dump, dump_rate, dump_bytes);
    std::fclose(dump);
    dump = nullptr;
}

}  // namespace

void audio_init(bool enabled) {
    if (const char* p = std::getenv("WIIKIT_AUDIODUMP")) {
        dump = std::fopen(p, "wb");
        if (dump) { wav_header(dump, 32000, 0); std::atexit(close_dump); }
    }
    if (!enabled) return;
    SDL_SetHint(SDL_HINT_AUDIO_DEVICE_SAMPLE_FRAMES, "256");   // a small device buffer: ~5 ms
    if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) { rt_log("audio: %s", SDL_GetError()); return; }
    SDL_AudioSpec spec = {SDL_AUDIO_S16, 2, 32000};
    stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, nullptr, nullptr);
    if (!stream) { rt_log("audio: no device: %s", SDL_GetError()); return; }
    stream_rate = 32000;                             // paused until kTarget is queued
    rt_log("audio: %s", SDL_GetAudioDeviceName(SDL_GetAudioStreamDevice(stream)));
}

void audio_play(const uint8_t* be_rl, uint32_t frames, uint32_t rate) {
    if (!frames || (!stream && !dump)) return;
    static std::vector<int16_t> lr;
    lr.resize(2 * (size_t)frames);
    for (uint32_t i = 0; i < frames; ++i) {
        const uint8_t* p = be_rl + 4 * i;
        lr[2 * i + 1] = (int16_t)(p[0] << 8 | p[1]);  // right
        lr[2 * i] = (int16_t)(p[2] << 8 | p[3]);      // left
    }
    if (dump) {
        if (!dump_rate) dump_rate = rate;
        std::fwrite(lr.data(), 2, lr.size(), dump);  // little-endian hosts
        dump_bytes += (uint32_t)(lr.size() * 2);
    }
    if (!stream) return;
    if (rate != stream_rate) {
        SDL_AudioSpec spec = {SDL_AUDIO_S16, 2, (int)rate};
        SDL_SetAudioStreamFormat(stream, &spec, nullptr);
        stream_rate = rate;
    }
    int queued = SDL_GetAudioStreamQueued(stream);
    if (playing && queued == 0) {                    // ran dry: wait for the cushion again
        SDL_PauseAudioStreamDevice(stream);
        playing = false;
        if (++underruns % 20 == 1) rt_log("audio: %llu underruns", (unsigned long long)underruns);
    }
    if (queued > (int)(kMaxQueue * rate * 4)) {
        if (++dropped % 100 == 1) rt_log("audio: %llu blocks dropped (the device is behind)", (unsigned long long)dropped);
        return;
    }
    SDL_PutAudioStreamData(stream, lr.data(), (int)(lr.size() * 2));
    double q = (double)queued / (rate * 4.0);
    if (!playing) {
        if (q + (double)frames / rate >= kTarget) {
            SDL_ResumeAudioStreamDevice(stream);
            playing = true;
            smoothed = kTarget;
        }
        return;
    }
    // hold the queue at kTarget: play faster when it grows, slower when it shrinks
    smoothed += (q - smoothed) * 0.01;
    float ratio = (float)std::clamp(1.0 + (smoothed - kTarget) * 0.5, 0.98, 1.02);
    SDL_SetAudioStreamFrequencyRatio(stream, ratio);
    static uint64_t n = 0;
    if (dbg && ++n % 1000 == 0) rt_log("audio: queue %.1f ms (smoothed %.1f), speed %.4f", q * 1000, smoothed * 1000, ratio);
}
