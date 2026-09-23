// wiikit runtime — GX state to GLSL: XF (transform, lighting, texgen) to a
// vertex shader, BP (TEV, indirect textures, alpha test) to a fragment shader.
//
// One program per configuration, cached by the renderer under the key built
// here from the registers that change the code. What is only data (matrices,
// light parameters, colours, texture sizes) is read at run time: the vertex
// shader reads the whole of XF memory from a storage buffer, the fragment
// shader gets a uniform block (video.cpp, PSUniforms).
//
// The TEV works on integers as the hardware does (after Dolphin): inputs A,
// B, C are 8-bit, D and the registers signed 11-bit; the lerp is
// (A * (256 - C) + B * C) >> 8 with C widened to 0..256; results clamp to
// 0..255 or to -1024..1023. Texture coordinates are fixed point with 7
// fractional bits in texel units, which is what indirect texturing works on.
#include "video.h"
#include <cstdarg>
#include <cstdio>
#include <string>

namespace {

void w(std::string& s, const char* f, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    s += buf;
}

// ---- vertex shader ----------------------------------------------------------------------------
const char* VS_HEAD = R"(#version 450 core
layout(std430, binding = 0) readonly buffer XFMem { uint xf[]; };
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_nrm;
layout(location = 2) in vec3 a_bin;
layout(location = 3) in vec3 a_tan;
layout(location = 4) in vec4 a_col0;
layout(location = 5) in vec4 a_col1;
layout(location = 6) in vec4 a_tc01;
layout(location = 7) in vec4 a_tc23;
layout(location = 8) in vec4 a_tc45;
layout(location = 9) in vec4 a_tc67;
layout(location = 10) in uvec4 a_mtx0;
layout(location = 11) in uvec4 a_mtx1;
layout(location = 12) in uvec4 a_mtx2;
out vec4 v_col0;
out vec4 v_col1;
out vec3 v_tc[8];
float X(uint a) { return uintBitsToFloat(xf[a]); }
vec4 R(uint a) { return vec4(X(a), X(a + 1u), X(a + 2u), X(a + 3u)); }
vec3 R3(uint a) { return vec3(X(a), X(a + 1u), X(a + 2u)); }
vec4 C(uint a) {
    uint c = xf[a];
    return vec4(float(c >> 24), float((c >> 16) & 255u), float((c >> 8) & 255u), float(c & 255u)) / 255.0;
}
// light i (XF 0x600 + 16 i): colour at +3, cos attenuation +4, distance
// attenuation +7, position +10, direction +13
vec4 light(uint i, uint attn_fn, uint diff_fn, vec3 pos, vec3 nrm) {
    uint L = 0x600u + i * 16u;
    vec3 lpos = R3(L + 10u), ldir = R3(L + 13u), cosatt = R3(L + 4u), distatt = R3(L + 7u);
    vec3 t = lpos - pos;
    vec3 ld = dot(t, t) > 0.0 ? normalize(t) : nrm;
    float attn = 1.0;
    if (attn_fn == 3u) {                                  // spot
        float d2 = dot(t, t), d = sqrt(d2);
        attn = max(0.0, dot(ld, ldir));
        attn = max(0.0, cosatt.x + cosatt.y * attn + cosatt.z * attn * attn) / dot(distatt, vec3(1.0, d, d2));
    } else if (attn_fn == 1u) {                           // specular
        attn = dot(nrm, ld) >= 0.0 ? max(0.0, dot(nrm, ldir)) : 0.0;
        vec3 da = diff_fn == 0u ? distatt : normalize(distatt);
        attn = max(0.0, dot(cosatt, vec3(1.0, attn, attn * attn))) / dot(da, vec3(1.0, attn, attn * attn));
    }
    float df = diff_fn == 0u ? 1.0 : diff_fn == 1u ? dot(ld, nrm) : max(0.0, dot(ld, nrm));
    return attn * df * C(L + 3u);
}
void main() {
    vec4 p = vec4(a_pos, 1.0);
    uint pm = a_mtx0.x * 4u;
    vec3 pos = vec3(dot(R(pm), p), dot(R(pm + 4u), p), dot(R(pm + 8u), p));
    uint nm = 0x400u + (a_mtx0.x & 31u) * 3u;
    vec3 nrm = vec3(dot(R3(nm), a_nrm), dot(R3(nm + 3u), a_nrm), dot(R3(nm + 6u), a_nrm));
    if (dot(nrm, nrm) > 0.0) nrm = normalize(nrm);
    vec4 clip;
    float p0 = X(0x1020u), p1 = X(0x1021u), p2 = X(0x1022u), p3 = X(0x1023u), p4 = X(0x1024u), p5 = X(0x1025u);
    if (xf[0x1026u] == 0u) clip = vec4(p0 * pos.x + p1 * pos.z, p2 * pos.y + p3 * pos.z, p4 * pos.z + p5, -pos.z);
    else clip = vec4(p0 * pos.x + p1, p2 * pos.y + p3, p4 * pos.z + p5, 1.0);
    // GX clip z runs from -w (near) to 0 (far); the viewport maps it to
    // farZ + zRange * z / w, in units of 2^24
    clip.z = clip.w * (X(0x101Fu) / 16777216.0) + clip.z * (X(0x101Cu) / 16777216.0);
    // the EFB is stored top row first: a negative viewport height (the usual
    // one) means no flip here
    if (X(0x101Au) < 0.0) clip.x = -clip.x;
    if (X(0x101Bu) < 0.0) clip.y = -clip.y;
    gl_Position = clip;
)";

void vs_channel(std::string& s, const uint32_t* xf, int j) {
    uint32_t cc = xf[0x100E + j], ac = xf[0x1010 + j];
    w(s, "    {\n        vec4 vc = a_col%d, mreg = C(0x%Xu), areg = C(0x%Xu);\n", j, 0x100C + j, 0x100A + j);
    w(s, "        vec4 mat = vec4(%s.rgb, %s.a);\n", (cc & 1) ? "vc" : "mreg", (ac & 1) ? "vc" : "mreg");
    const char* part[2] = {"rgb", "a"};
    uint32_t ctl[2] = {cc, ac};
    for (int k = 0; k < 2; ++k) {
        uint32_t c = ctl[k];
        if (!(c >> 1 & 1)) continue;                              // no lighting: the material
        uint32_t mask = (c >> 2 & 15) | (c >> 11 & 15) << 4, diff = c >> 7 & 3, attn = c >> 9 & 3;
        w(s, "        {\n        vec4 lacc = %s;\n", (c >> 6 & 1) ? "vc" : "areg");
        for (int i = 0; i < 8; ++i)
            if (mask >> i & 1) w(s, "        lacc += light(%du, %uu, %uu, pos, nrm);\n", i, attn, diff);
        w(s, "        mat.%s *= clamp(lacc.%s, 0.0, 1.0);\n        }\n", part[k], part[k]);
    }
    w(s, "        v_col%d = mat;\n    }\n", j);
}

std::string vertex_shader(const uint32_t* xf, uint8_t vflags) {
    std::string s = VS_HEAD;
    uint32_t nchan = xf[0x1009] & 3, ntg = xf[0x103F] & 15;
    if (ntg > 8) ntg = 8;
    for (uint32_t j = 0; j < nchan && j < 2; ++j) vs_channel(s, xf, (int)j);
    if (nchan == 0) s += "    v_col0 = a_col0;\n";
    if (nchan < 2) s += (vflags & VTX_COL1) ? "    v_col1 = a_col1;\n" : "    v_col1 = v_col0;\n";
    static const char* tcsrc[8] = {"a_tc01.xy", "a_tc01.zw", "a_tc23.xy", "a_tc23.zw",
                                   "a_tc45.xy", "a_tc45.zw", "a_tc67.xy", "a_tc67.zw"};
    static const char* tmidx[8] = {"a_mtx0.y", "a_mtx0.z", "a_mtx0.w", "a_mtx1.x",
                                   "a_mtx1.y", "a_mtx1.z", "a_mtx1.w", "a_mtx2.x"};
    bool dual = xf[0x1012] & 1;
    for (uint32_t i = 0; i < 8; ++i) {
        if (i >= ntg) { w(s, "    v_tc[%u] = vec3(0.0);\n", i); continue; }
        uint32_t tg = xf[0x1040 + i], proj = tg >> 1 & 1, ab11 = !(tg >> 2 & 1), type = tg >> 4 & 7,
                 src = tg >> 7 & 31, esrc = tg >> 12 & 7;
        w(s, "    {\n        vec4 s = ");
        switch (src) {
        case 0: s += "vec4(a_pos, 1.0);\n"; break;
        case 1: s += "vec4(a_nrm, 1.0);\n"; break;
        case 3: s += "vec4(a_tan, 1.0);\n"; break;
        case 4: s += "vec4(a_bin, 1.0);\n"; break;
        default:
            if (src >= 5 && src <= 12) w(s, "vec4(%s, 1.0, 1.0);\n", tcsrc[src - 5]);
            else s += "vec4(0.0, 0.0, 1.0, 1.0);\n";
        }
        if (ab11) s += "        s.z = 1.0;\n";
        s += "        vec3 t;\n";
        if (type == 0) {
            w(s, "        uint m = %s * 4u;\n", tmidx[i]);
            if (proj) s += "        t = vec3(dot(R(m), s), dot(R(m + 4u), s), dot(R(m + 8u), s));\n";
            else s += "        t = vec3(dot(R(m), s), dot(R(m + 4u), s), 1.0);\n";
            if (dual) {
                uint32_t pt = xf[0x1050 + i];
                if (pt >> 8 & 1) s += "        t = normalize(t);\n";
                w(s, "        uint pm = 0x500u + (xf[0x%Xu] & 63u) * 4u;\n", 0x1050 + i);
                s += "        vec4 t4 = vec4(t, 1.0);\n"
                     "        t = vec3(dot(R(pm), t4), dot(R(pm + 4u), t4), dot(R(pm + 8u), t4));\n";
            }
            // a zero q on the console: coordinates halved and clamped (Dolphin)
            if (proj) s += "        if (t.z == 0.0) t.xy = clamp(t.xy / 2.0, vec2(-1.0), vec2(1.0));\n";
        } else if (type == 1) {
            w(s, "        t = v_tc[%u];\n", esrc < i ? esrc : 0);   // emboss: the offset is not modelled
        } else {
            w(s, "        t = vec3(v_col%d.rg, 1.0);\n", type == 2 ? 0 : 1);
        }
        w(s, "        v_tc[%u] = t;\n    }\n", i);
    }
    s += "}\n";
    return s;
}

// ---- fragment shader --------------------------------------------------------------------------
const char* FS_HEAD = R"(#version 450 core
layout(std140, binding = 1) uniform PS {
    ivec4 u_reg[4];       // PREV, C0, C1, C2 as set by the game
    ivec4 u_konst[4];
    ivec4 u_alpha;        // alpha test references, destination alpha value
    vec4 u_texdim[8];     // xy: 1 / (map size * 128); zw: texcoord scale * 128
    ivec4 u_indscale[4];  // indirect stage: coordinate shift s, t
    ivec4 u_indmtx[6];    // indirect matrices, two rows each: (a, b, c, shift)
};
layout(binding = 0) uniform sampler2D s0;
layout(binding = 1) uniform sampler2D s1;
layout(binding = 2) uniform sampler2D s2;
layout(binding = 3) uniform sampler2D s3;
layout(binding = 4) uniform sampler2D s4;
layout(binding = 5) uniform sampler2D s5;
layout(binding = 6) uniform sampler2D s6;
layout(binding = 7) uniform sampler2D s7;
in vec4 v_col0;
in vec4 v_col1;
in vec3 v_tc[8];
layout(location = 0, index = 0) out vec4 o_col;
layout(location = 0, index = 1) out vec4 o_blend;
ivec4 S(sampler2D s, int map, ivec2 c) {
    // c is in texels * 128; half a unit in, a sample that sits exactly on a
    // texel's edge (common under indirect offsets) picks the texel the
    // hardware's floor would, whatever the float rounding
    return ivec4(round(texture(s, (vec2(c) + 0.5) * u_texdim[map].xy) * 255.0));
}
int idot(ivec3 a, ivec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
void main() {
    ivec4 prev = u_reg[0], c0 = u_reg[1], c1 = u_reg[2], c2 = u_reg[3];
    ivec4 col0 = ivec4(round(v_col0 * 255.0)), col1 = ivec4(round(v_col1 * 255.0));
    int alphabump = 0;
    ivec2 tevcoord = ivec2(0), wrapped;
    ivec4 tex, ras, konst;
)";

const char* CIN[16] = {"prev.rgb", "prev.aaa", "c0.rgb", "c0.aaa", "c1.rgb", "c1.aaa", "c2.rgb", "c2.aaa",
                       "tex.rgb", "tex.aaa", "ras.rgb", "ras.aaa", "ivec3(255)", "ivec3(128)", "konst.rgb",
                       "ivec3(0)"};
const char* AIN[8] = {"prev.a", "c0.a", "c1.a", "c2.a", "tex.a", "ras.a", "konst.a", "0"};
const char* REG[4] = {"prev", "c0", "c1", "c2"};

std::string swizzle(const uint32_t* bp, uint32_t table) {
    const char* ch = "rgba";
    uint32_t a = bp[0xF6 + 2 * table], b = bp[0xF7 + 2 * table];
    return std::string(".") + ch[a & 3] + ch[a >> 2 & 3] + ch[b & 3] + ch[b >> 2 & 3];
}

std::string kcolor(uint32_t sel) {
    static const int frac[8] = {255, 223, 191, 159, 128, 96, 64, 32};
    char b[64];
    if (sel < 8) std::snprintf(b, sizeof b, "ivec3(%d)", frac[sel]);
    else if (sel >= 12 && sel < 16) std::snprintf(b, sizeof b, "u_konst[%u].rgb", sel - 12);
    else if (sel >= 16) std::snprintf(b, sizeof b, "u_konst[%u].%c%c%c", sel & 3, "rgba"[(sel - 16) / 4],
                                      "rgba"[(sel - 16) / 4], "rgba"[(sel - 16) / 4]);
    else std::snprintf(b, sizeof b, "ivec3(0)");
    return b;
}
std::string kalpha(uint32_t sel) {
    static const int frac[8] = {255, 223, 191, 159, 128, 96, 64, 32};
    char b[64];
    if (sel < 8) std::snprintf(b, sizeof b, "%d", frac[sel]);
    else if (sel >= 16) std::snprintf(b, sizeof b, "u_konst[%u].%c", sel & 3, "rgba"[(sel - 16) / 4]);
    else std::snprintf(b, sizeof b, "0");
    return b;
}

// one combiner, color (n = 3) or alpha (n = 1), into `out`
void combiner(std::string& s, uint32_t env, bool alpha, const char* out) {
    uint32_t bias = env >> 16 & 3, op = env >> 18 & 1, clampv = env >> 19 & 1, scale = env >> 20 & 3;
    const char* T = alpha ? "int" : "ivec3";
    const char* c = alpha ? ".a" : ".rgb";
    if (bias != 3) {
        static const char* bv[3] = {"", " + 128", " - 128"};
        int sh = scale == 1 ? 1 : scale == 2 ? 2 : 0;
        w(s, "        %s %s = (((td%s%s) << %d) %s ((((ta%s << 8) + (tb%s - ta%s) * (tc%s + (tc%s >> 7))) << %d) + %d >> 8))%s;\n",
          T, out, c, bv[bias], sh, op ? "-" : "+", c, c, c, c, c, sh, op ? 127 : 128, scale == 3 ? " >> 1" : "");
    } else {
        uint32_t mode = scale << 1 | op;
        const char* cmp = (mode & 1) ? "==" : ">";
        if (mode < 6) {
            static const char* pack[3] = {"%s.r", "(%s.r | %s.g << 8)", "(%s.r | %s.g << 8 | %s.b << 16)"};
            char A[64], B[64];
            std::snprintf(A, sizeof A, pack[mode / 2], "ta", "ta", "ta");
            std::snprintf(B, sizeof B, pack[mode / 2], "tb", "tb", "tb");
            w(s, "        %s %s = td%s + ((%s %s %s) ? tc%s : %s(0));\n", T, out, c, A, cmp, B, c, T);
        } else if (alpha) {
            w(s, "        int %s = td.a + ((ta.a %s tb.a) ? tc.a : 0);\n", out, cmp);
        } else {
            w(s, "        ivec3 %s = td.rgb + ivec3(%s(ta.rgb, tb.rgb)) * tc.rgb;\n", out,
              (mode & 1) ? "equal" : "greaterThan");
        }
    }
    if (clampv) w(s, "        %s = clamp(%s, %s(0), %s(255));\n", out, out, T, T);
    else w(s, "        %s = clamp(%s, %s(-1024), %s(1023));\n", out, out, T, T);
}

const char* acmp(uint32_t c) {
    static const char* t[8] = {"false", "a < %s", "a == %s", "a <= %s", "a > %s", "a != %s", "a >= %s", "true"};
    return t[c & 7];
}

std::string fragment_shader(const uint32_t* bp) {
    std::string s = FS_HEAD;
    uint32_t gen = bp[0x00], ntev = (gen >> 10 & 15) + 1, nind = gen >> 16 & 7, ntg = gen & 15;
    if (nind > 4) nind = 4;
    for (uint32_t i = 0; i < ntg && i < 8; ++i)
        // to fixed point by rounding: the rasterizer's own interpolation
        // lands on exact values where the float one falls just short
        w(s, "    ivec2 uv%u = ivec2(round((v_tc[%u].z == 0.0 ? v_tc[%u].xy : v_tc[%u].xy / v_tc[%u].z) * u_texdim[%u].zw));\n",
          i, i, i, i, i, i);
    if (ntg == 0) s += "    ivec2 uv0 = ivec2(0);\n";
    for (uint32_t i = 0; i < nind; ++i) {
        uint32_t ref = bp[0x27] >> (6 * i), map = ref & 7, tc = ref >> 3 & 7;
        if (tc >= ntg) tc = 0;
        w(s, "    ivec3 ind%u = S(s%u, %u, uv%u >> u_indscale[%u].xy).abg;\n", i, map, map, tc, i);
    }
    for (uint32_t st = 0; st < ntev; ++st) {
        uint32_t tref = bp[0x28 + st / 2] >> (12 * (st & 1)), map = tref & 7, tc = tref >> 3 & 7,
                 en = tref >> 6 & 1, chan = tref >> 7 & 7;
        bool has_tc = tc < ntg;
        if (!has_tc) tc = 0;
        uint32_t cmd = bp[0x10 + st], bt = cmd & 3, ifmt = cmd >> 2 & 3, ibias = cmd >> 4 & 7,
                 bs = cmd >> 7 & 3, mid = cmd >> 9 & 15, sw = cmd >> 13 & 7, tw = cmd >> 16 & 7,
                 addprev = cmd >> 20 & 1;
        w(s, "    // stage %u\n    {\n", st);
        if (bt < nind) {
            static const char* amask[4] = {"248", "224", "240", "248"};
            if (bs) w(s, "        alphabump = ind%u.%c & %s;\n", bt, "xyz"[bs - 1], amask[ifmt]);
            if (mid & 3) {
                static const char* fmask[4] = {"255", "31", "15", "7"};
                static const char* badd[4] = {"-128", "1", "1", "1"};
                w(s, "        ivec3 ic = ind%u & %s;\n", bt, fmask[ifmt]);
                for (int k = 0; k < 3; ++k)
                    if (ibias >> k & 1) w(s, "        ic.%c += %s;\n", "xyz"[k], badd[ifmt]);
                uint32_t mi = 2 * ((mid & 3) - 1), id = mid >> 2;
                if (id == 1) w(s, "        ivec2 tr = (uv%u * ic.xx) >> 8;\n", tc);
                else if (id == 2) w(s, "        ivec2 tr = (uv%u * ic.yy) >> 8;\n", tc);
                else w(s, "        ivec2 tr = ivec2(idot(u_indmtx[%u].xyz, ic), idot(u_indmtx[%u].xyz, ic)) >> 3;\n",
                       mi, mi + 1);
                w(s, "        int sh = u_indmtx[%u].w;\n        tr = sh >= 0 ? tr >> sh : tr << -sh;\n", mi);
            } else {
                s += "        ivec2 tr = ivec2(0);\n";
            }
            static const int wrap[8] = {0, 256, 128, 64, 32, 16, -1, -1};
            uint32_t wv[2] = {sw, tw};
            for (int k = 0; k < 2; ++k) {
                char c = "xy"[k];
                if (wv[k] == 0) w(s, "        wrapped.%c = uv%u.%c;\n", c, tc, c);
                else if (wrap[wv[k]] < 0) w(s, "        wrapped.%c = 0;\n", c);
                else w(s, "        wrapped.%c = uv%u.%c & %d;\n", c, tc, c, (wrap[wv[k]] << 7) - 1);
            }
            w(s, "        tevcoord %s wrapped + tr;\n", addprev ? "+=" : "=");
            s += "        tevcoord = (tevcoord << 8) >> 8;\n";
        } else if (en) {
            if (has_tc) w(s, "        tevcoord = uv%u;\n", tc);
            else s += "        tevcoord = ivec2(0);\n";
        }
        uint32_t cenv = bp[0xC0 + 2 * st], aenv = bp[0xC1 + 2 * st];
        uint32_t rswap = aenv & 3, tswap = aenv >> 2 & 3;
        if (en) w(s, "        tex = S(s%u, %u, tevcoord)%s;\n", map, map, swizzle(bp, tswap).c_str());
        else s += "        tex = ivec4(0);\n";
        switch (chan) {
        case 0: w(s, "        ras = col0%s;\n", swizzle(bp, rswap).c_str()); break;
        case 1: w(s, "        ras = col1%s;\n", swizzle(bp, rswap).c_str()); break;
        case 5: s += "        ras = ivec4(alphabump);\n"; break;
        case 6: s += "        ras = ivec4(alphabump | (alphabump >> 5));\n"; break;
        default: s += "        ras = ivec4(0);\n"; break;
        }
        uint32_t ks = bp[0xF6 + st / 2] >> (10 * (st & 1)), kc = ks >> 4 & 31, ka = ks >> 9 & 31;
        w(s, "        konst = ivec4(%s, %s);\n", kcolor(kc).c_str(), kalpha(ka).c_str());
        w(s, "        ivec4 ta = ivec4(%s, %s) & 255;\n", CIN[cenv >> 12 & 15], AIN[aenv >> 13 & 7]);
        w(s, "        ivec4 tb = ivec4(%s, %s) & 255;\n", CIN[cenv >> 8 & 15], AIN[aenv >> 10 & 7]);
        w(s, "        ivec4 tc = ivec4(%s, %s) & 255;\n", CIN[cenv >> 4 & 15], AIN[aenv >> 7 & 7]);
        w(s, "        ivec4 td = ivec4(%s, %s);\n", CIN[cenv & 15], AIN[aenv >> 4 & 7]);
        combiner(s, cenv, false, "rc");
        combiner(s, aenv, true, "ra");
        w(s, "        %s.rgb = rc;\n        %s.a = ra;\n    }\n", REG[cenv >> 22 & 3], REG[aenv >> 22 & 3]);
    }
    s += "    ivec4 fin = prev & 255;\n";
    uint32_t at = bp[0xF3], c0 = at >> 16 & 7, c1 = at >> 19 & 7, logic = at >> 22 & 3;
    if (!(c0 == 7 && c1 == 7 && logic != 2)) {
        char t0[64], t1[64];
        s += "    int a = fin.a;\n";
        std::snprintf(t0, sizeof t0, acmp(c0), "u_alpha.x");
        std::snprintf(t1, sizeof t1, acmp(c1), "u_alpha.y");
        static const char* lg[4] = {"&&", "||", "!=", "=="};
        w(s, "    if (!((%s) %s (%s))) discard;\n", t0, lg[logic], t1);
    }
    s += "    o_blend = vec4(fin) / 255.0;\n    o_col = o_blend;\n";
    if (bp[0x42] >> 8 & 1) s += "    o_col.a = float(u_alpha.z) / 255.0;\n";
    s += "}\n";
    return s;
}

}  // namespace

// The registers that change the generated code.
std::string gx_shader_key(const uint32_t* bp, const uint32_t* xf, uint8_t vflags) {
    std::string k;
    auto add = [&](uint32_t v) { k.append(reinterpret_cast<const char*>(&v), 4); };
    uint32_t nchan = xf[0x1009] & 3, ntg = xf[0x103F] & 15;
    add(nchan | ntg << 4 | (xf[0x1012] & 1) << 8 | (uint32_t)(vflags & VTX_COL1) << 9);
    for (uint32_t j = 0; j < nchan && j < 2; ++j) { add(xf[0x100E + j]); add(xf[0x1010 + j]); }
    for (uint32_t i = 0; i < ntg && i < 8; ++i) { add(xf[0x1040 + i]); add(xf[0x1050 + i] & 0x100); }
    uint32_t gen = bp[0x00], ntev = (gen >> 10 & 15) + 1;
    add(gen & 0x73C0F);                              // texgens, TEV and indirect stages
    for (uint32_t s = 0; s < ntev; ++s) { add(bp[0xC0 + 2 * s]); add(bp[0xC1 + 2 * s]); add(bp[0x10 + s]); }
    for (uint32_t i = 0; i < (ntev + 1) / 2; ++i) add(bp[0x28 + i]);
    for (int i = 0; i < 8; ++i) add(bp[0xF6 + i]);
    add(bp[0xF3] >> 16);
    add(bp[0x27]);
    add(bp[0x42] >> 8 & 1);
    return k;
}

void gx_shader_gen(const uint32_t* bp, const uint32_t* xf, uint8_t vflags, std::string& vs, std::string& fs) {
    vs = vertex_shader(xf, vflags);
    fs = fragment_shader(bp);
}
