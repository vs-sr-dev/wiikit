// wiikit runtime — what the GX command processor (gx.cpp, guest threads) hands the
// renderer (video.cpp, the host's main thread).
//
// The command processor runs where the game writes the FIFO, and does there
// everything that reads guest memory: vertices are decoded through the vertex
// descriptor and the arrays into one fixed layout (GVtx), XF matrices are
// loaded from indexed arrays, textures are decoded to RGBA through TMEM's
// palettes. What it records reads no guest memory any more: the renderer can
// run behind the game on its own thread, and the game may reuse its buffers
// as soon as it believes the GP has consumed them, as on the console.
//
// The record is a byte stream of commands:
//   BP      u32 (reg << 24 | value): a BP register after its mask
//   XF      u16 addr, u16 n, n x u32
//   DRAW    u8 primitive (0x80..0xB8), u8 flags (VTX_*), u32 count, count x GVtx
//   TEXUP   u8 map, u32 id, u16 w, u16 h, u8 levels, RGBA8 pixels of every level
//   TEXBIND u8 map, u32 id
//   TEXEFB  u8 map, u32 address: the EFB copy made to that address
//   FRAME   (an XFB copy was recorded: one frame)
//   DRAWDONE (the game asked to know when the GP has drawn everything so
//            far: the renderer, reaching it, raises PE's finish interrupt)
#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

struct GVtx {
    float pos[3];
    float nrm[3], bin[3], tan[3];
    float tc[8][2];
    uint8_t col[2][4];
    uint8_t mtx[12];                 // position matrix index, then TEX0..TEX7 matrix indices
};
static_assert(sizeof(GVtx) == 132, "GVtx layout");

enum : uint8_t { VC_BP = 1, VC_XF, VC_DRAW, VC_TEXUP, VC_TEXBIND, VC_TEXEFB, VC_FRAME, VC_DRAWDONE };
enum : uint8_t { VTX_COL0 = 1, VTX_COL1 = 2, VTX_NRM = 4, VTX_NBT = 8 };

// video.cpp
bool video_enabled();                       // false with --no-video: the stream is parsed only
// Takes the record and leaves it empty; waits while the renderer has
// frames_ahead frames queued, at most wait_ms (-1: as long as it takes).
// False if the time ran out: the record is left as it was.
bool video_submit(std::vector<uint8_t>& rec, int frames, int wait_ms = -1);
void video_set_xfb(uint32_t top_field_addr); // VI: the XFB being scanned out (physical)
void video_retrace();                        // VI: a vertical retrace happened
void video_set_lines(uint32_t lines);        // VI: lines of picture scanned out (of 480 for NTSC)

struct VideoOptions {
    bool enabled = true;
    int scale = 0;                           // internal resolution: EFB x scale; 0 = from the window's height
    int frames_ahead = 2;                    // frames the game may record before the renderer draws them
    const char* dump_dir = nullptr;          // PNGs of presented frames
    int dump_every = 60;                     // one PNG every N retraces
    double quit_after = 0;                   // seconds; 0 = run until the window closes
    bool widescreen = false;                 // the console's screen is 16:9 (SYSCONF), else 4:3
    bool fullscreen = false;                 // start fullscreen (borderless, at the desktop's mode)
    int window_w = 0, window_h = 0;          // the window's size; 0 = 720 lines at the screen's shape
    std::string keys;                        // the key file: the Remote's buttons on keys and mouse buttons
};
void video_configure(const VideoOptions& o);
// The host's input as a Wii Remote (read by wpad.cpp): WPAD core button
// bits; the pointer over the picture VI shows, -1..1 with y down, when the
// mouse is inside it; shake while the Remote is to be shaken.
// tilt: 0 level, pointing at the screen; +1 or -1 raised, pointing up (the
// sign of KPAD's acc.z then, while a game's expectation is found out)
struct PadState { uint32_t buttons = 0; float x = 0, y = 0; bool pointer = false, shake = false; int tilt = 0; };
PadState video_pad();
// Relative mouse, for a port that turns the mouse's motion into a stick or a
// view: the cursor is captured while the window has the focus (released for
// the pause box), and the motion is summed until taken.
void video_set_relative_mouse(bool on);
void video_take_mouse_motion(float& dx, float& dy);

// WIIKIT_PERF=1: where a frame's time goes, reported every second by the
// renderer. Nanoseconds, summed since the last report.
struct VideoPerf {
    std::atomic<uint64_t> vtx{0};       // guest side: decoding vertices
    std::atomic<uint64_t> tex{0};       // guest side: decoding textures
    std::atomic<uint64_t> wait{0};      // guest side: waiting for the renderer to take a record
    std::atomic<uint64_t> draw{0};      // the renderer: executing records
    std::atomic<uint64_t> present{0};   // the renderer: presenting (the swap waits for the GPU)
};
extern VideoPerf g_vperf;
extern bool g_vperf_on;

void write_png(const std::string& path, int w, int h, const uint8_t* rgba);   // RGBA8, top row first
// Runs the window and the renderer on the calling thread (the process's main
// thread) until the window is closed; the game runs on its own threads.
void video_run(const char* title);
