// wiikit runtime — the renderer: a window, and the GX record drawn with OpenGL 4.5.
//
// It runs on the host's main thread, which owns the window and the GL
// context; the game runs on its own threads and hands over its GX record
// (video.h) in chunks, at most two frames ahead. The renderer keeps a mirror
// of the BP and XF registers from the record and turns each draw into GL
// state and a program generated from the TEV configuration (gxshader.cpp).
//
// The EFB is a framebuffer of 640 x 528 (times --scale), stored top row
// first as on the console. EFB copies stay on the host GPU: to the XFB they
// become the frames VI shows, by address; to textures they are converted to
// what the target format would decode to, and the game's later binds of that
// address find them (gx.cpp). On each VI retrace the XFB that VI's top-field
// register names is presented, letterboxed to 4:3.
#include "video.h"
#include "gl.h"
#include "rt.h"
#include <SDL3/SDL.h>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#define WIIKIT_GL_DEFINE(T, n) T n = nullptr;
WIIKIT_GL_FUNCS(WIIKIT_GL_DEFINE)
#undef WIIKIT_GL_DEFINE

std::string gx_shader_key(const uint32_t* bp, const uint32_t* xf, uint8_t vflags);
void gx_shader_gen(const uint32_t* bp, const uint32_t* xf, uint8_t vflags, std::string& vs, std::string& fs);

namespace {

VideoOptions opt;
constexpr int EFB_W = 640, EFB_H = 528;
using Clock = std::chrono::steady_clock;

// ---- the queue from the game ------------------------------------------------------------------
struct Chunk { std::vector<uint8_t> data; int frames; };
std::mutex qmx;
std::condition_variable q_space;
std::deque<Chunk> q;
int q_frames = 0;
SDL_Semaphore* wake = nullptr;
std::atomic<uint32_t> xfb_addr{0}, retraces{0}, vi_lines{480};
std::mutex pad_mx;
PadState pad;

// ---- GX state, as the record left it --------------------------------------------------------
uint32_t bp[0x100], xf[0x1058];
uint32_t xf_lo = 0, xf_hi = 0x1058;                // XF words not yet uploaded
int32_t tev_reg[4][4], tev_konst[4][4];

struct PSUniforms {                                // gxshader.cpp's uniform block PS (std140)
    int32_t reg[4][4], konst[4][4], alpha[4];
    float texdim[8][4];
    int32_t indscale[4][4], indmtx[6][4];
};

// ---- GL objects -------------------------------------------------------------------------------
SDL_Window* win = nullptr;
int S = 1;                                         // EFB scale
GLuint efb_fbo, efb_col, efb_dep, copy_fbo, vao, empty_vao, vbo, quad_ibo, xf_ssbo, ps_ubo, samplers[8];
GLuint copy_prog;
GLint copy_rect_loc, copy_mode_loc;
constexpr size_t VBO_CAP = 64u << 20;
size_t vbo_off = 0;
PSUniforms ps_last;
bool ps_valid = false;
uint32_t sampler_mode[8][2];
bool sampler_set[8];

struct Tex { GLuint name = 0; int w = 0, h = 0, levels = 0; };
std::unordered_map<uint32_t, Tex> texs;            // decoded textures, by id
std::unordered_map<uint32_t, Tex> efb_copies;      // EFB copies to texture, by address
std::unordered_map<uint32_t, Tex> xfbs;            // EFB copies to the XFB, by address
uint64_t map_src[8];                               // per map: texture id, or 1 << 32 | EFB copy address
uint32_t last_xfb = 0;
std::unordered_map<std::string, GLuint> programs;

struct Counters { uint64_t frames, presents, draws, programs; } cnt;
bool dump_ps = false;                              // WIIKIT_SHADERDUMP: log the next uniforms

int32_t s11(uint32_t v) { return (int32_t)((v & 0x7FF) << 21) >> 21; }
int32_t s10(uint32_t v) { return (int32_t)((v & 0x3FF) << 22) >> 22; }
float fx(uint32_t a) { float f; std::memcpy(&f, &xf[a], 4); return f; }

// ---- GL helpers ---------------------------------------------------------------------------------
bool gl_load() {
    bool ok = true;
#define WIIKIT_GL_LOAD(T, n)                                                   \
    n = reinterpret_cast<T>(SDL_GL_GetProcAddress(#n));                        \
    if (!n) { rt_log("video: no %s", #n); ok = false; }
    WIIKIT_GL_FUNCS(WIIKIT_GL_LOAD)
#undef WIIKIT_GL_LOAD
    return ok;
}

GLuint compile(GLenum type, const std::string& src) {
    GLuint sh = glCreateShader(type);
    const char* p = src.c_str();
    glShaderSource(sh, 1, &p, nullptr);
    glCompileShader(sh);
    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetShaderInfoLog(sh, sizeof log, nullptr, log);
        rt_log("video: shader does not compile:\n%s\n%s", log, src.c_str());
        glDeleteShader(sh);
        return 0;
    }
    return sh;
}

GLuint link(const std::string& vs, const std::string& fs) {
    GLuint v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) return 0;
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetProgramInfoLog(p, sizeof log, nullptr, log);
        rt_log("video: program does not link: %s", log);
        return 0;
    }
    return p;
}

void ensure_tex(Tex& t, int w, int h, int levels) {
    if (t.name && t.w == w && t.h == h && t.levels == levels) return;
    if (t.name) glDeleteTextures(1, &t.name);
    glCreateTextures(GL_TEXTURE_2D, 1, &t.name);
    glTextureStorage2D(t.name, levels, GL_RGBA8, w, h);
    t.w = w; t.h = h; t.levels = levels;
}

void APIENTRY gl_debug(GLenum, GLenum type, GLuint, GLenum severity, GLsizei, const GLchar* msg, const void*) {
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    rt_log("gl: %s%s", type == GL_DEBUG_TYPE_ERROR ? "error: " : "", msg);
}

// ---- the EFB copy pass --------------------------------------------------------------------------
const char* FULLSCREEN_VS = R"(#version 450 core
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

// what a copy to each texture copy format (Dolphin's EFBCopyFormat numbering)
// reads back as when the game samples it
const char* COPY_FS = R"(#version 450 core
layout(binding = 0) uniform sampler2D efb_color;
layout(binding = 1) uniform sampler2D efb_depth;
uniform ivec4 u_rect;   // source x, y (EFB pixels at scale), half size, -
uniform ivec4 u_mode;   // format, from depth, intensity, EFB has alpha
out vec4 o;
vec4 src(ivec2 p) {
    if (u_mode.y != 0) {
        uint z = uint(clamp(texelFetch(efb_depth, p, 0).r, 0.0, 1.0) * 16777215.0);
        return vec4(float(z >> 16), float((z >> 8) & 255u), float(z & 255u), 255.0) / 255.0;
    }
    vec4 c = texelFetch(efb_color, p, 0);
    if (u_mode.w == 0) c.a = 1.0;
    return c;
}
float q(float v, float n) { return floor(v * n + 0.5) / n; }
void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    vec4 c;
    if (u_rect.z != 0) {
        ivec2 s = u_rect.xy + p * 2;
        c = (src(s) + src(s + ivec2(1, 0)) + src(s + ivec2(0, 1)) + src(s + ivec2(1, 1))) * 0.25;
    } else {
        c = src(u_rect.xy + p);
    }
    float i = (u_mode.z != 0 && u_mode.y == 0) ? clamp(0.257 * c.r + 0.504 * c.g + 0.098 * c.b + 16.0 / 255.0, 0.0, 1.0) : c.r;
    switch (u_mode.x) {
    case 0: o = vec4(q(i, 15.0)); break;                                  // R4 / Z4
    case 1: case 8: o = vec4(i); break;                                   // R8 / Z8
    case 2: o = vec4(vec3(q(i, 15.0)), q(c.a, 15.0)); break;              // RA4
    case 3: o = u_mode.y != 0 ? vec4(c.ggg, c.r) : vec4(vec3(i), c.a); break;   // RA8 / Z16
    case 4: o = vec4(q(c.r, 31.0), q(c.g, 63.0), q(c.b, 31.0), 1.0); break;
    case 5: o = c.a >= 0.875 ? vec4(q(c.r, 31.0), q(c.g, 31.0), q(c.b, 31.0), 1.0)
                             : vec4(q(c.r, 15.0), q(c.g, 15.0), q(c.b, 15.0), q(c.a, 7.0)); break;
    case 7: o = vec4(c.a); break;                                         // A8
    case 9: o = vec4(c.g); break;                                         // G8 / Z8M
    case 10: o = vec4(c.b); break;                                        // B8 / Z8L
    case 11: o = vec4(c.ggg, c.r); break;                                 // RG8
    case 12: o = vec4(c.bbb, c.g); break;                                 // GB8 / Z16L
    default: o = c; break;                                                // RGBA8 / Z24X8
    }
}
)";

// ---- register side effects ----------------------------------------------------------------------
void efb_clear(int x, int y, int w, int h) {
    bool has_alpha = (bp[0x43] & 7) == 1;
    bool cu = bp[0x41] >> 3 & 1, au = bp[0x41] >> 4 & 1, zu = bp[0x40] >> 4 & 1;
    if (!cu && !au && !zu) return;
    glBindFramebuffer(GL_FRAMEBUFFER, efb_fbo);
    glEnable(GL_SCISSOR_TEST);
    glScissor(x * S, y * S, w * S, h * S);
    glColorMask(cu, cu, cu, au || !has_alpha);
    glDepthMask(zu);
    float a = (bp[0x4F] >> 8 & 255) / 255.0f, r = (bp[0x4F] & 255) / 255.0f;
    float g = (bp[0x50] >> 8 & 255) / 255.0f, b = (bp[0x50] & 255) / 255.0f;
    glClearColor(r, g, b, has_alpha ? a : 1.0f);
    glClearDepth((bp[0x51] & 0xFFFFFF) / 16777215.0);
    glClear((cu || au ? GL_COLOR_BUFFER_BIT : 0) | (zu ? GL_DEPTH_BUFFER_BIT : 0));
}

void efb_copy(uint32_t v) {
    int x = (int)(bp[0x49] & 0x3FF), y = (int)(bp[0x49] >> 10 & 0x3FF);
    int w = (int)(bp[0x4A] & 0x3FF) + 1, h = (int)(bp[0x4A] >> 10 & 0x3FF) + 1;
    uint32_t dest = (bp[0x4B] & 0xFFFFFF) << 5;
    glDisable(GL_SCISSOR_TEST);
    if (v >> 14 & 1) {                                           // to the XFB
        Tex& t = xfbs[dest];
        ensure_tex(t, w * S, h * S, 1);
        glNamedFramebufferTexture(copy_fbo, GL_COLOR_ATTACHMENT0, t.name, 0);
        glBlitNamedFramebuffer(efb_fbo, copy_fbo, x * S, y * S, (x + w) * S, (y + h) * S, 0, 0, w * S, h * S,
                               GL_COLOR_BUFFER_BIT, GL_NEAREST);
        last_xfb = dest;
        ++cnt.frames;
    } else {                                                     // to a texture
        bool half = v >> 9 & 1;
        int tw = half ? (w + 1) / 2 : w, th = half ? (h + 1) / 2 : h;
        uint32_t tpf = v >> 3 & 15, fmt = tpf / 2 + (tpf & 1) * 8;
        Tex& t = efb_copies[dest];
        ensure_tex(t, tw * S, th * S, 1);
        glNamedFramebufferTexture(copy_fbo, GL_COLOR_ATTACHMENT0, t.name, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, copy_fbo);
        glViewport(0, 0, tw * S, th * S);
        glDisable(GL_BLEND);
        glDisable(GL_COLOR_LOGIC_OP);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glColorMask(1, 1, 1, 1);
        glUseProgram(copy_prog);
        glProgramUniform4i(copy_prog, copy_rect_loc, x * S, y * S, half, 0);
        glProgramUniform4i(copy_prog, copy_mode_loc, (int)fmt, (bp[0x43] & 7) == 3, (int)(v >> 15 & 1),
                           (bp[0x43] & 7) == 1);
        glBindTextureUnit(0, efb_col);
        glBindTextureUnit(1, efb_dep);
        glBindSampler(0, 0);
        glBindSampler(1, 0);
        glBindVertexArray(empty_vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(vao);
    }
    if (v >> 11 & 1) efb_clear(x, y, w, h);
}

void bp_set(uint32_t v) {
    uint32_t reg = v >> 24, val = v & 0xFFFFFF;
    bp[reg] = val;
    if (reg >= 0xE0 && reg < 0xE8) {                             // TEV colour and konst registers
        int32_t* r = (val >> 23 & 1) ? tev_konst[(reg - 0xE0) / 2] : tev_reg[(reg - 0xE0) / 2];
        if (!(reg & 1)) { r[0] = s11(val); r[3] = s11(val >> 12); }   // RA
        else { r[2] = s11(val); r[1] = s11(val >> 12); }              // BG
    }
    if (reg == 0x52) efb_copy(val);
}

// ---- draws --------------------------------------------------------------------------------------
GLuint program_for(uint8_t vflags) {
    std::string key = gx_shader_key(bp, xf, vflags);
    auto it = programs.find(key);
    if (it != programs.end()) return it->second;
    std::string vs, fs;
    gx_shader_gen(bp, xf, vflags, vs, fs);
    GLuint p = link(vs, fs);
    if (const char* dir = std::getenv("WIIKIT_SHADERDUMP")) {           // debugging: the sources
        std::string path = std::string(dir) + "/prog_" + std::to_string(cnt.programs) + ".glsl";
        if (FILE* f = std::fopen(path.c_str(), "w")) {
            std::fprintf(f, "%s\n// ---- fragment\n%s", vs.c_str(), fs.c_str());
            std::fclose(f);
        }
        dump_ps = true;
    }
    programs.emplace(std::move(key), p);
    ++cnt.programs;
    return p;
}

uint32_t used_maps() {
    uint32_t gen = bp[0x00], used = 0;
    int ntev = (int)(gen >> 10 & 15) + 1, nind = (int)(gen >> 16 & 7);
    for (int s = 0; s < ntev; ++s) {
        uint32_t t = bp[0x28 + s / 2] >> (12 * (s & 1));
        if (t & 0x40) used |= 1u << (t & 7);
    }
    for (int i = 0; i < nind && i < 4; ++i) used |= 1u << (bp[0x27] >> (6 * i) & 7);
    return used;
}

void set_ps_uniforms() {
    PSUniforms u{};
    std::memcpy(u.reg, tev_reg, sizeof u.reg);
    std::memcpy(u.konst, tev_konst, sizeof u.konst);
    u.alpha[0] = (int32_t)(bp[0xF3] & 255);
    u.alpha[1] = (int32_t)(bp[0xF3] >> 8 & 255);
    u.alpha[2] = (int32_t)(bp[0x42] & 255);
    for (int m = 0; m < 8; ++m) {
        uint32_t img0 = bp[0x88 + (m & 3) + (m >= 4 ? 0x20 : 0)];
        u.texdim[m][0] = 1.0f / (float)(((img0 & 0x3FF) + 1) * 128);
        u.texdim[m][1] = 1.0f / (float)(((img0 >> 10 & 0x3FF) + 1) * 128);
        u.texdim[m][2] = (float)(((bp[0x30 + 2 * m] & 0xFFFF) + 1) * 128);
        u.texdim[m][3] = (float)(((bp[0x31 + 2 * m] & 0xFFFF) + 1) * 128);
    }
    for (int i = 0; i < 4; ++i) {
        uint32_t r = bp[0x25 + i / 2] >> (8 * (i & 1));
        u.indscale[i][0] = (int32_t)(r & 15);
        u.indscale[i][1] = (int32_t)(r >> 4 & 15);
    }
    for (int m = 0; m < 3; ++m) {
        uint32_t c0 = bp[0x06 + 3 * m], c1 = bp[0x07 + 3 * m], c2 = bp[0x08 + 3 * m];
        int32_t scale = (int32_t)((c0 >> 22 & 3) | (c1 >> 22 & 3) << 2 | (c2 >> 22 & 3) << 4);
        int32_t r0[4] = {s11(c0), s11(c1), s11(c2), 17 - scale};
        int32_t r1[4] = {s11(c0 >> 11), s11(c1 >> 11), s11(c2 >> 11), 17 - scale};
        std::memcpy(u.indmtx[2 * m], r0, sizeof r0);
        std::memcpy(u.indmtx[2 * m + 1], r1, sizeof r1);
    }
    if (dump_ps) {
        dump_ps = false;
        std::string t = "video: program " + std::to_string(cnt.programs - 1) + " uniforms:";
        for (int i = 0; i < 4; ++i) t += " reg" + std::to_string(i) + "(" + std::to_string(u.reg[i][0]) + "," + std::to_string(u.reg[i][1]) + "," + std::to_string(u.reg[i][2]) + "," + std::to_string(u.reg[i][3]) + ")";
        for (int i = 0; i < 4; ++i) t += " k" + std::to_string(i) + "(" + std::to_string(u.konst[i][0]) + "," + std::to_string(u.konst[i][1]) + "," + std::to_string(u.konst[i][2]) + "," + std::to_string(u.konst[i][3]) + ")";
        for (int i = 0; i < 8; ++i) t += " td" + std::to_string(i) + "(" + std::to_string(1 / u.texdim[i][0] / 128) + "," + std::to_string(1 / u.texdim[i][1] / 128) + "," + std::to_string(u.texdim[i][2] / 128) + "," + std::to_string(u.texdim[i][3] / 128) + ")";
        for (int i = 0; i < 4; ++i) t += " is" + std::to_string(i) + "(" + std::to_string(u.indscale[i][0]) + "," + std::to_string(u.indscale[i][1]) + ")";
        for (int i = 0; i < 6; ++i) t += " im" + std::to_string(i) + "(" + std::to_string(u.indmtx[i][0]) + "," + std::to_string(u.indmtx[i][1]) + "," + std::to_string(u.indmtx[i][2]) + "," + std::to_string(u.indmtx[i][3]) + ")";
        rt_log("%s", t.c_str());
    }
    if (ps_valid && !std::memcmp(&u, &ps_last, sizeof u)) return;
    glNamedBufferSubData(ps_ubo, 0, sizeof u, &u);
    ps_last = u;
    ps_valid = true;
}

void bind_textures() {
    static const GLint wrap[4] = {GL_CLAMP_TO_EDGE, GL_REPEAT, GL_MIRRORED_REPEAT, GL_REPEAT};
    static const GLint minf[8] = {GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST, GL_NEAREST_MIPMAP_LINEAR, GL_NEAREST,
                                  GL_LINEAR, GL_LINEAR_MIPMAP_NEAREST, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR};
    uint32_t used = used_maps();
    for (int m = 0; m < 8; ++m) {
        if (!(used >> m & 1)) continue;
        GLuint name = 0;
        if (map_src[m] >> 32) {
            auto it = efb_copies.find((uint32_t)map_src[m]);
            if (it != efb_copies.end()) name = it->second.name;
        } else {
            auto it = texs.find((uint32_t)map_src[m]);
            if (it != texs.end()) name = it->second.name;
        }
        glBindTextureUnit(m, name);
        int off = (m & 3) + (m >= 4 ? 0x20 : 0);
        uint32_t m0 = bp[0x80 + off], m1 = bp[0x84 + off];
        if (!sampler_set[m] || sampler_mode[m][0] != m0 || sampler_mode[m][1] != m1) {
            GLuint s = samplers[m];
            glSamplerParameteri(s, GL_TEXTURE_WRAP_S, wrap[m0 & 3]);
            glSamplerParameteri(s, GL_TEXTURE_WRAP_T, wrap[m0 >> 2 & 3]);
            glSamplerParameteri(s, GL_TEXTURE_MAG_FILTER, (m0 >> 4 & 1) ? GL_LINEAR : GL_NEAREST);
            glSamplerParameteri(s, GL_TEXTURE_MIN_FILTER, minf[m0 >> 5 & 7]);
            glSamplerParameterf(s, GL_TEXTURE_LOD_BIAS, (float)(int8_t)(m0 >> 9 & 0xFF) / 32.0f);
            glSamplerParameterf(s, GL_TEXTURE_MIN_LOD, (float)(m1 & 0xFF) / 16.0f);
            glSamplerParameterf(s, GL_TEXTURE_MAX_LOD, (float)(m1 >> 8 & 0xFF) / 16.0f);
            sampler_mode[m][0] = m0;
            sampler_mode[m][1] = m1;
            sampler_set[m] = true;
        }
        glBindSampler(m, samplers[m]);
    }
}

void draw(uint8_t prim, uint8_t vflags, const uint8_t* vtx, uint32_t n) {
    uint32_t cull = bp[0x00] >> 14 & 3;
    bool tri = prim < 0xA8;
    if (tri && cull == 3) return;
    GLuint prog = program_for(vflags);
    if (!prog) return;
    ++cnt.draws;
    glBindFramebuffer(GL_FRAMEBUFFER, efb_fbo);
    glUseProgram(prog);

    int offx = s10(bp[0x59]) * 2, offy = s10(bp[0x59] >> 10) * 2;
    float wd = std::fabs(fx(0x101A)), ht = std::fabs(fx(0x101B));
    glViewportIndexedf(0, (fx(0x101D) - wd - (float)offx) * (float)S, (fx(0x101E) - ht - (float)offy) * (float)S,
                       2 * wd * (float)S, 2 * ht * (float)S);
    int sx0 = (int)(bp[0x20] >> 12 & 0x7FF) - offx, sy0 = (int)(bp[0x20] & 0x7FF) - offy;
    int sx1 = (int)(bp[0x21] >> 12 & 0x7FF) - offx + 1, sy1 = (int)(bp[0x21] & 0x7FF) - offy + 1;
    sx0 = std::max(sx0, 0); sy0 = std::max(sy0, 0); sx1 = std::min(sx1, EFB_W); sy1 = std::min(sy1, EFB_H);
    if (sx1 <= sx0 || sy1 <= sy0) return;
    glEnable(GL_SCISSOR_TEST);
    glScissor(sx0 * S, sy0 * S, (sx1 - sx0) * S, (sy1 - sy0) * S);

    static const GLenum zf[8] = {GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL, GL_GREATER, GL_NOTEQUAL, GL_GEQUAL, GL_ALWAYS};
    uint32_t zm = bp[0x40];
    if (zm & 1) { glEnable(GL_DEPTH_TEST); glDepthFunc(zf[zm >> 1 & 7]); }
    else glDisable(GL_DEPTH_TEST);
    glDepthMask(zm >> 4 & 1);

    uint32_t cm = bp[0x41];
    bool has_alpha = (bp[0x43] & 7) == 1;
    glColorMask(cm >> 3 & 1, cm >> 3 & 1, cm >> 3 & 1, (cm >> 4 & 1) && has_alpha);
    if (cm & 1) {
        glDisable(GL_COLOR_LOGIC_OP);
        glEnable(GL_BLEND);
        if (cm >> 11 & 1) {                                      // subtract: dst - src
            glBlendEquation(GL_FUNC_REVERSE_SUBTRACT);
            glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ONE, GL_ONE);
        } else {
            GLenum da = has_alpha ? GL_DST_ALPHA : GL_ONE, ida = has_alpha ? GL_ONE_MINUS_DST_ALPHA : GL_ZERO;
            const GLenum src[8] = {GL_ZERO, GL_ONE, GL_DST_COLOR, GL_ONE_MINUS_DST_COLOR,
                                   GL_SRC1_ALPHA, GL_ONE_MINUS_SRC1_ALPHA, da, ida};
            const GLenum dst[8] = {GL_ZERO, GL_ONE, GL_SRC1_COLOR, GL_ONE_MINUS_SRC1_COLOR,
                                   GL_SRC1_ALPHA, GL_ONE_MINUS_SRC1_ALPHA, da, ida};
            GLenum sf = src[cm >> 8 & 7], df = dst[cm >> 5 & 7];
            glBlendEquation(GL_FUNC_ADD);
            glBlendFuncSeparate(sf, df, sf, df);
        }
    } else if (cm >> 1 & 1) {
        static const GLenum lo[16] = {GL_CLEAR, GL_AND, GL_AND_REVERSE, GL_COPY, GL_AND_INVERTED, GL_NOOP,
                                      GL_XOR, GL_OR, GL_NOR, GL_EQUIV, GL_INVERT, GL_OR_REVERSE,
                                      GL_COPY_INVERTED, GL_OR_INVERTED, GL_NAND, GL_SET};
        glDisable(GL_BLEND);
        glEnable(GL_COLOR_LOGIC_OP);
        glLogicOp(lo[cm >> 12 & 15]);
    } else {
        glDisable(GL_BLEND);
        glDisable(GL_COLOR_LOGIC_OP);
    }
    // GX's front faces are clockwise on screen: counter-clockwise for GL,
    // whose y runs the other way over the same rows
    if (tri && cull) { glEnable(GL_CULL_FACE); glCullFace(cull == 1 ? GL_BACK : GL_FRONT); }
    else glDisable(GL_CULL_FACE);

    if (xf_lo < xf_hi) {
        glNamedBufferSubData(xf_ssbo, xf_lo * 4, (xf_hi - xf_lo) * 4, &xf[xf_lo]);
        xf_lo = 0x1058;
        xf_hi = 0;
    }
    set_ps_uniforms();
    bind_textures();

    size_t bytes = (size_t)n * sizeof(GVtx);
    if (vbo_off + bytes > VBO_CAP) { glInvalidateBufferData(vbo); vbo_off = 0; }
    glNamedBufferSubData(vbo, (GLintptr)vbo_off, (GLsizeiptr)bytes, vtx);
    GLint base = (GLint)(vbo_off / sizeof(GVtx));
    vbo_off += bytes;
    switch (prim) {
    case 0x80: case 0x88:
        glDrawElementsBaseVertex(GL_TRIANGLES, (GLsizei)(n / 4 * 6), GL_UNSIGNED_INT, nullptr, base);
        break;
    case 0x90: glDrawArrays(GL_TRIANGLES, base, (GLsizei)n); break;
    case 0x98: glDrawArrays(GL_TRIANGLE_STRIP, base, (GLsizei)n); break;
    case 0xA0: glDrawArrays(GL_TRIANGLE_FAN, base, (GLsizei)n); break;
    case 0xA8: glDrawArrays(GL_LINES, base, (GLsizei)n); break;
    case 0xB0: glDrawArrays(GL_LINE_STRIP, base, (GLsizei)n); break;
    default: glDrawArrays(GL_POINTS, base, (GLsizei)n); break;
    }
}

// ---- the record -----------------------------------------------------------------------------------
template <class T> T rd(const uint8_t*& p) { T v; std::memcpy(&v, p, sizeof v); p += sizeof v; return v; }

void exec(const std::vector<uint8_t>& data) {
    const uint8_t* p = data.data();
    const uint8_t* end = p + data.size();
    while (p < end) {
        switch (*p++) {
        case VC_BP: bp_set(rd<uint32_t>(p)); break;
        case VC_XF: {
            uint16_t a = rd<uint16_t>(p), n = rd<uint16_t>(p);
            std::memcpy(&xf[a], p, 4u * n);
            p += 4u * n;
            xf_lo = std::min<uint32_t>(xf_lo, a);
            xf_hi = std::max<uint32_t>(xf_hi, a + n);
            break;
        }
        case VC_DRAW: {
            uint8_t prim = rd<uint8_t>(p), fl = rd<uint8_t>(p);
            uint32_t n = rd<uint32_t>(p);
            draw(prim, fl, p, n);
            p += (size_t)n * sizeof(GVtx);
            break;
        }
        case VC_TEXUP: {
            uint8_t m = rd<uint8_t>(p);
            uint32_t id = rd<uint32_t>(p);
            int w = rd<uint16_t>(p), h = rd<uint16_t>(p), levels = rd<uint8_t>(p);
            Tex& t = texs[id];
            ensure_tex(t, w, h, levels);
            for (int l = 0; l < levels; ++l) {
                int lw = std::max(1, w >> l), lh = std::max(1, h >> l);
                glTextureSubImage2D(t.name, l, 0, 0, lw, lh, GL_RGBA, GL_UNSIGNED_BYTE, p);
                p += (size_t)lw * lh * 4;
            }
            map_src[m] = id;
            break;
        }
        case VC_TEXBIND: { uint8_t m = rd<uint8_t>(p); map_src[m] = rd<uint32_t>(p); break; }
        case VC_TEXEFB: { uint8_t m = rd<uint8_t>(p); map_src[m] = 1ull << 32 | rd<uint32_t>(p); break; }
        case VC_FRAME: break;
        default: rt_die("video: bad record byte %02X", p[-1]);
        }
    }
}

// ---- presenting -----------------------------------------------------------------------------------
uint32_t crc_table[256];
uint32_t crc32(const uint8_t* p, size_t n, uint32_t c = 0xFFFFFFFFu) {
    if (!crc_table[1])
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t k = i;
            for (int j = 0; j < 8; ++j) k = k & 1 ? 0xEDB88320u ^ (k >> 1) : k >> 1;
            crc_table[i] = k;
        }
    for (size_t i = 0; i < n; ++i) c = crc_table[(c ^ p[i]) & 255] ^ (c >> 8);
    return c;
}

}  // namespace

// a PNG with stored (uncompressed) deflate blocks
void write_png(const std::string& path, int w, int h, const uint8_t* rgba) {
    std::vector<uint8_t> raw;
    for (int y = 0; y < h; ++y) {
        raw.push_back(0);
        raw.insert(raw.end(), rgba + (size_t)y * w * 4, rgba + (size_t)(y + 1) * w * 4);
    }
    std::vector<uint8_t> z = {0x78, 0x01};
    uint32_t a = 1, b = 0;
    for (uint8_t c : raw) { a = (a + c) % 65521; b = (b + a) % 65521; }
    for (size_t off = 0; off < raw.size() || off == 0;) {
        size_t n = std::min<size_t>(65535, raw.size() - off);
        z.push_back(off + n == raw.size() ? 1 : 0);
        z.push_back((uint8_t)n); z.push_back((uint8_t)(n >> 8));
        z.push_back((uint8_t)~n); z.push_back((uint8_t)(~n >> 8));
        z.insert(z.end(), raw.begin() + (ptrdiff_t)off, raw.begin() + (ptrdiff_t)(off + n));
        off += n;
        if (off == raw.size()) break;
    }
    uint32_t ad = b << 16 | a;
    for (int i = 3; i >= 0; --i) z.push_back((uint8_t)(ad >> (8 * i)));
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    auto be = [&](uint32_t v) { uint8_t t[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
                                std::fwrite(t, 1, 4, f); };
    auto chunk = [&](const char* type, const std::vector<uint8_t>& d) {
        be((uint32_t)d.size());
        std::vector<uint8_t> td(type, type + 4);
        td.insert(td.end(), d.begin(), d.end());
        std::fwrite(td.data(), 1, td.size(), f);
        be(crc32(td.data(), td.size()) ^ 0xFFFFFFFFu);
    };
    std::fwrite("\x89PNG\r\n\x1a\n", 1, 8, f);
    std::vector<uint8_t> ihdr = {(uint8_t)(w >> 24), (uint8_t)(w >> 16), (uint8_t)(w >> 8), (uint8_t)w,
                                 (uint8_t)(h >> 24), (uint8_t)(h >> 16), (uint8_t)(h >> 8), (uint8_t)h,
                                 8, 6, 0, 0, 0};
    chunk("IHDR", ihdr);
    chunk("IDAT", z);
    chunk("IEND", {});
    std::fclose(f);
}

namespace {

// The Wii Remote on the mouse and the keyboard. The mouse over the picture
// is the pointer; left button or Enter = A, right button or Backspace = B;
// WASD or the arrows = the d-pad; Tab = +, Q = -; 1, 2 = 1, 2; Space or the
// middle button = a shake. Home has no key: Esc opens the port's own menu
// (video_run) instead of the Wii's.
void update_pad() {
    struct K { SDL_Scancode k; uint32_t b; };
    static const K keys[] = {{SDL_SCANCODE_RETURN, 0x0800}, {SDL_SCANCODE_KP_ENTER, 0x0800},
                             {SDL_SCANCODE_BACKSPACE, 0x0400},
                             {SDL_SCANCODE_A, 0x0001}, {SDL_SCANCODE_D, 0x0002},
                             {SDL_SCANCODE_S, 0x0004}, {SDL_SCANCODE_W, 0x0008},
                             {SDL_SCANCODE_LEFT, 0x0001}, {SDL_SCANCODE_RIGHT, 0x0002},
                             {SDL_SCANCODE_DOWN, 0x0004}, {SDL_SCANCODE_UP, 0x0008},
                             {SDL_SCANCODE_TAB, 0x0010}, {SDL_SCANCODE_Q, 0x1000},
                             {SDL_SCANCODE_1, 0x0200}, {SDL_SCANCODE_2, 0x0100}};
    const bool* ks = SDL_GetKeyboardState(nullptr);
    PadState p;
    for (const K& k : keys)
        if (ks[k.k]) p.buttons |= k.b;
    p.shake = ks[SDL_SCANCODE_SPACE];
    float mx = 0, my = 0;
    SDL_MouseButtonFlags mb = SDL_GetMouseState(&mx, &my);
    if (mb & SDL_BUTTON_LMASK) p.buttons |= 0x0800;
    if (mb & SDL_BUTTON_RMASK) p.buttons |= 0x0400;
    if (mb & SDL_BUTTON_MMASK) p.shake = true;
    int ww = 0, wh = 0;
    SDL_GetWindowSize(win, &ww, &wh);                        // mouse coordinates are in window units
    if (ww > 0 && wh > 0 && (SDL_GetWindowFlags(win) & SDL_WINDOW_MOUSE_FOCUS)) {
        // -1..1 spans the picture as present() shows it: the 4:3 screen's
        // width, and only the lines VI scans out (360 for a letterboxed 16:9),
        // so that the game's cursor lands under the mouse
        float fw = (float)ww, fh = fw * 3 / 4;
        if (fh > (float)wh) { fh = (float)wh; fw = fh * 4 / 3; }
        fh = fh * (float)std::min<uint32_t>(vi_lines.load(), 480) / 480;
        p.x = (mx - ((float)ww - fw) / 2) / fw * 2 - 1;
        p.y = (my - ((float)wh - fh) / 2) / fh * 2 - 1;
        p.pointer = p.x >= -1 && p.x <= 1 && p.y >= -1 && p.y <= 1;
    }
    // over the picture the game draws its own cursor, as on the Wii
    static bool hidden = false;
    if (p.pointer != hidden) {
        hidden = p.pointer;
        if (hidden) SDL_HideCursor(); else SDL_ShowCursor();
    }
    // WIIKIT_PAD="45:A 50.5:@0.2,-0.1 51:A": at each time (seconds from
    // start) press buttons for 150 ms (A B 1 2 + - H U D L R, X = shake), or
    // move the pointer, which stays: reproducible runs for debugging
    static const char* script = std::getenv("WIIKIT_PAD");
    static const Clock::time_point t0 = Clock::now();
    static float sx = 0, sy = 0;
    static bool spointer = false;
    if (script) {
        double t = std::chrono::duration<double>(Clock::now() - t0).count();
        for (const char* q = script; *q;) {
            char* e;
            double at = std::strtod(q, &e);
            if (e == q || *e != ':') break;
            q = e + 1;
            if (*q == '@') {
                float x = std::strtof(q + 1, &e), y = std::strtof(e + 1, &e);
                if (t >= at) { sx = x; sy = y; spointer = true; }
                q = e;
            } else {
                for (; *q && *q != ' '; ++q) {
                    static const char names[] = "AB12+-HUDLR";
                    static const uint32_t bits[] = {0x0800, 0x0400, 0x0200, 0x0100, 0x0010, 0x1000,
                                                    0x8000, 0x0008, 0x0004, 0x0001, 0x0002};
                    const char* n = std::strchr(names, *q);
                    if (n && t >= at && t < at + 0.15) p.buttons |= bits[n - names];
                    if (*q == 'X' && t >= at && t < at + 0.15) p.shake = true;
                }
            }
            while (*q == ' ') ++q;
        }
        if (spointer && !p.pointer) { p.x = sx; p.y = sy; p.pointer = true; }
    }
    std::lock_guard<std::mutex> lk(pad_mx);
    pad = p;
}

void present() {
    ++cnt.presents;
    int ww = 0, wh = 0;
    SDL_GetWindowSizeInPixels(win, &ww, &wh);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_SCISSOR_TEST);
    glColorMask(1, 1, 1, 1);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    auto it = xfbs.find(xfb_addr.load());
    if (it == xfbs.end()) it = xfbs.find(last_xfb);
    if (it != xfbs.end() && ww > 0 && wh > 0) {
        const Tex& t = it->second;
        // the screen is 4:3 and 480 lines tall (NTSC); VI scans the XFB out
        // over its active lines, centred: 360 of them for a letterboxed 16:9
        int fw = ww, fh = ww * 3 / 4;
        if (fh > wh) { fh = wh; fw = wh * 4 / 3; }
        int lines = (int)std::min<uint32_t>(vi_lines.load(), 480);
        int dw = fw, dh = fh * lines / 480;
        int dx = (ww - dw) / 2, dy = (wh - dh) / 2;
        glNamedFramebufferTexture(copy_fbo, GL_COLOR_ATTACHMENT0, t.name, 0);
        glBlitNamedFramebuffer(copy_fbo, 0, 0, 0, t.w, t.h, dx, dy + dh, dx + dw, dy,   // top row first
                               GL_COLOR_BUFFER_BIT, GL_LINEAR);
    }
    if (opt.dump_dir && cnt.presents % (uint64_t)opt.dump_every == 0 && ww > 0 && wh > 0) {
        std::vector<uint8_t> px((size_t)ww * wh * 4), flip(px.size());   // the window, as shown
        glReadPixels(0, 0, ww, wh, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
        for (int y = 0; y < wh; ++y)
            std::memcpy(&flip[(size_t)y * ww * 4], &px[(size_t)(wh - 1 - y) * ww * 4], (size_t)ww * 4);
        char name[64];
        std::snprintf(name, sizeof name, "/frame_%06llu.png", (unsigned long long)cnt.presents);
        write_png(std::string(opt.dump_dir) + name, ww, wh, flip.data());
    }
    SDL_GL_SwapWindow(win);
}

void setup() {
    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
    glDisable(GL_DITHER);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    glCreateTextures(GL_TEXTURE_2D, 1, &efb_col);
    glTextureStorage2D(efb_col, 1, GL_RGBA8, EFB_W * S, EFB_H * S);
    glCreateTextures(GL_TEXTURE_2D, 1, &efb_dep);
    glTextureStorage2D(efb_dep, 1, GL_DEPTH_COMPONENT32F, EFB_W * S, EFB_H * S);
    for (GLuint t : {efb_col, efb_dep}) {
        glTextureParameteri(t, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTextureParameteri(t, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glCreateFramebuffers(1, &efb_fbo);
    glNamedFramebufferTexture(efb_fbo, GL_COLOR_ATTACHMENT0, efb_col, 0);
    glNamedFramebufferTexture(efb_fbo, GL_DEPTH_ATTACHMENT, efb_dep, 0);
    if (glCheckNamedFramebufferStatus(efb_fbo, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        rt_die("video: the EFB framebuffer is incomplete");
    glCreateFramebuffers(1, &copy_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, efb_fbo);
    glClearColor(0, 0, 0, 1);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glCreateBuffers(1, &vbo);
    glNamedBufferStorage(vbo, VBO_CAP, nullptr, GL_DYNAMIC_STORAGE_BIT);
    std::vector<uint32_t> qi;
    for (uint32_t k = 0; k < 0x10000 / 4; ++k)
        for (uint32_t d : {0u, 1u, 2u, 0u, 2u, 3u}) qi.push_back(4 * k + d);
    glCreateBuffers(1, &quad_ibo);
    glNamedBufferStorage(quad_ibo, qi.size() * 4, qi.data(), 0);
    glCreateVertexArrays(1, &vao);
    glCreateVertexArrays(1, &empty_vao);
    glVertexArrayVertexBuffer(vao, 0, vbo, 0, sizeof(GVtx));
    glVertexArrayElementBuffer(vao, quad_ibo);
    struct A { GLuint loc; int n; GLenum type; bool norm, integer; size_t off; };
    const A attrs[] = {{0, 3, GL_FLOAT, false, false, offsetof(GVtx, pos)},
                       {1, 3, GL_FLOAT, false, false, offsetof(GVtx, nrm)},
                       {2, 3, GL_FLOAT, false, false, offsetof(GVtx, bin)},
                       {3, 3, GL_FLOAT, false, false, offsetof(GVtx, tan)},
                       {4, 4, GL_UNSIGNED_BYTE, true, false, offsetof(GVtx, col)},
                       {5, 4, GL_UNSIGNED_BYTE, true, false, offsetof(GVtx, col) + 4},
                       {6, 4, GL_FLOAT, false, false, offsetof(GVtx, tc)},
                       {7, 4, GL_FLOAT, false, false, offsetof(GVtx, tc) + 16},
                       {8, 4, GL_FLOAT, false, false, offsetof(GVtx, tc) + 32},
                       {9, 4, GL_FLOAT, false, false, offsetof(GVtx, tc) + 48},
                       {10, 4, GL_UNSIGNED_BYTE, false, true, offsetof(GVtx, mtx)},
                       {11, 4, GL_UNSIGNED_BYTE, false, true, offsetof(GVtx, mtx) + 4},
                       {12, 4, GL_UNSIGNED_BYTE, false, true, offsetof(GVtx, mtx) + 8}};
    for (const A& a : attrs) {
        glEnableVertexArrayAttrib(vao, a.loc);
        if (a.integer) glVertexArrayAttribIFormat(vao, a.loc, a.n, a.type, (GLuint)a.off);
        else glVertexArrayAttribFormat(vao, a.loc, a.n, a.type, a.norm, (GLuint)a.off);
        glVertexArrayAttribBinding(vao, a.loc, 0);
    }
    glBindVertexArray(vao);

    glCreateBuffers(1, &xf_ssbo);
    glNamedBufferStorage(xf_ssbo, sizeof xf, nullptr, GL_DYNAMIC_STORAGE_BIT);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, xf_ssbo);
    glCreateBuffers(1, &ps_ubo);
    glNamedBufferStorage(ps_ubo, sizeof(PSUniforms), nullptr, GL_DYNAMIC_STORAGE_BIT);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, ps_ubo);
    glCreateSamplers(8, samplers);

    copy_prog = link(FULLSCREEN_VS, COPY_FS);
    if (!copy_prog) rt_die("video: the EFB copy program does not build");
    copy_rect_loc = glGetUniformLocation(copy_prog, "u_rect");
    copy_mode_loc = glGetUniformLocation(copy_prog, "u_mode");
}

}  // namespace

// ---- the game's side ------------------------------------------------------------------------------
bool video_enabled() { return opt.enabled; }
void video_configure(const VideoOptions& o) { opt = o; S = std::max(1, o.scale); }

void video_submit(std::vector<uint8_t>& rec, int frames) {
    std::unique_lock<std::mutex> lk(qmx);
    q_space.wait(lk, [] { return q_frames < 2 && q.size() < 256; });
    q.push_back(Chunk{std::move(rec), frames});
    q_frames += frames;
    rec = std::vector<uint8_t>();
    rec.reserve(1u << 20);
    lk.unlock();
    if (wake) SDL_SignalSemaphore(wake);
}

void video_set_xfb(uint32_t a) { xfb_addr.store(a); }
void video_set_lines(uint32_t n) { vi_lines.store(n); }
PadState video_pad() {
    std::lock_guard<std::mutex> lk(pad_mx);
    return pad;
}
void video_retrace() {
    retraces.fetch_add(1);
    if (wake) SDL_SignalSemaphore(wake);
}

// ---- the window --------------------------------------------------------------------------------
void video_run(const char* title) {
    if (!SDL_Init(SDL_INIT_VIDEO)) rt_die("video: SDL_Init: %s", SDL_GetError());
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    bool debug = std::getenv("WIIKIT_GLDEBUG") != nullptr;
    if (debug) SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    win = SDL_CreateWindow(title, 960, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!win) rt_die("video: SDL_CreateWindow: %s", SDL_GetError());
    SDL_GLContext ctx = SDL_GL_CreateContext(win);
    if (!ctx) rt_die("video: no OpenGL 4.5 context: %s", SDL_GetError());
    SDL_GL_MakeCurrent(win, ctx);
    SDL_GL_SetSwapInterval(0);                   // VI paces the presents
    if (!gl_load()) rt_die("video: OpenGL functions missing");
    rt_log("video: %s, %s", (const char*)glGetString(GL_RENDERER), (const char*)glGetString(GL_VERSION));
    if (debug) {
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(gl_debug, nullptr);
    }
    wake = SDL_CreateSemaphore(0);
    setup();

    auto t0 = Clock::now(), t_title = t0;
    uint64_t frames_then = 0;
    uint32_t seen = retraces.load();
    std::string base_title = title;
    for (;;) {
        SDL_Event e;
        bool quit = false;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) quit = true;
            // Esc: the port's menu in place of the Wii's Home Button menu.
            // While it is open the renderer stops, and the game with it as
            // soon as its FIFO record queue is full.
            if (e.type == SDL_EVENT_KEY_DOWN && e.key.scancode == SDL_SCANCODE_ESCAPE && !e.key.repeat) {
                const SDL_MessageBoxButtonData buttons[] = {
                    {SDL_MESSAGEBOX_BUTTON_ESCAPEKEY_DEFAULT | SDL_MESSAGEBOX_BUTTON_RETURNKEY_DEFAULT, 0, "Resume"},
                    {0, 1, "Quit"}};
                SDL_MessageBoxData box = {SDL_MESSAGEBOX_INFORMATION, win, base_title.c_str(), "Paused.",
                                          2, buttons, nullptr};
                int choice = 0;
                if (SDL_ShowMessageBox(&box, &choice) && choice == 1) quit = true;
            }
        }
        auto now = Clock::now();
        if (opt.quit_after > 0 && std::chrono::duration<double>(now - t0).count() > opt.quit_after) quit = true;
        if (quit) {
            rt_log("video: %llu frames, %llu presents, %llu draws, %llu programs",
                   (unsigned long long)cnt.frames, (unsigned long long)cnt.presents,
                   (unsigned long long)cnt.draws, (unsigned long long)cnt.programs);
            gx_report();
            std::fflush(stdout);
            std::_Exit(0);
        }
        update_pad();
        if (now - t_title >= std::chrono::seconds(1)) {
            double s = std::chrono::duration<double>(now - t_title).count();
            char buf[256];
            std::snprintf(buf, sizeof buf, "%s  |  %.1f fps", base_title.c_str(), (double)(cnt.frames - frames_then) / s);
            SDL_SetWindowTitle(win, buf);
            frames_then = cnt.frames;
            t_title = now;
        }
        bool busy = false;
        for (int k = 0; k < 16; ++k) {
            Chunk c;
            {
                std::lock_guard<std::mutex> lk(qmx);
                if (q.empty()) break;
                c = std::move(q.front());
                q.pop_front();
            }
            exec(c.data);
            {
                std::lock_guard<std::mutex> lk(qmx);
                q_frames -= c.frames;
            }
            q_space.notify_all();
            busy = true;
            if (retraces.load() != seen) break;
        }
        uint32_t r = retraces.load();
        if (r != seen) {
            seen = r;
            present();
            busy = true;
        }
        if (!busy) SDL_WaitSemaphoreTimeout(wake, 5);
    }
}
