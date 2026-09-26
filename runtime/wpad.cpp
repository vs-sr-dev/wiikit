// wiikit runtime — the Wii Remote, at the WPAD/KPAD layer.
//
// Below WPAD lies Nintendo's Bluetooth stack (WUD, BTA/BTE) talking HCI to
// the USB dongle through IOS. A port has no Remote to pair, so the cut is
// the public WPAD and KPAD API that games, KPAD itself, Wwise's speaker
// manager and the Home Button menu call: the Bluetooth stack never starts.
//
// Channel 0 holds a Remote driven by the host (video.cpp's PadState: the
// mouse as the pointer, keys for the buttons); the other channels are empty.
// The Remote is held still, pointing at the screen, gravity along -y,
// except while it is shaken: then it swings +-2 g along x from one sample to
// the next. acc_speed, the change of acceleration between samples, is what
// games compare with a threshold (Victorious: 0.4, over two frames).
//
// A game that plays with the Classic Controller (wpad_set_classic) finds one
// on each channel that has a Classic from video.cpp (video_classic: channel 0
// the keyboard and the first pad, the others the pads plugged in), in the
// KPAD status's ex_status, the Remote holding it pointing nowhere, and in
// WPAD's own samples (WPADRead, the auto-sampling ring, and the 2007 KPAD's
// copy of them) as a WPADCLStatus.
//
// Connections. A game may learn of Remotes only from the connect callbacks
// (KPAD's or WPAD's), and of extensions from WPAD's extension callback, as
// the Bluetooth stack calls them on a console. For a game that plays with
// the Classic the runtime calls them (a game of the Remote alone runs
// without them, and one of 2007 answers a connection by starting WPAD's own
// sampling, which reads the control blocks of a WPAD never started):
// at a VI retrace (an interrupt, as the stack's own callbacks are), or when
// the game reads or probes a Remote, at the first of these at least 0.2 s
// after the callback was registered (called at once, inside the
// registration, a game's tables may not be ready yet), and again whenever a
// channel's controller comes or goes.
#include "rt.h"
#include "video.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr uint32_t WPAD_STATE_SETUP = 3;
constexpr int32_t WPAD_ERR_NONE = 0, WPAD_ERR_NO_CONTROLLER = -1;
constexpr uint32_t WPAD_DEV_CORE = 0, WPAD_DEV_CLASSIC = 2, WPAD_DEV_NOT_FOUND = 253;
constexpr uint8_t WPAD_FMT_CORE_ACC_DPD = 2, WPAD_FMT_CLASSIC_ACC_DPD = 8;
uint32_t kpad_status_size = 0xF0;                   // the SDK's KPADStatus: 0xF0 in later SDKs, 0x84 in 2006-07's
bool classic_game = false;
WpadClassicFilter classic_filter = nullptr;
bool motor_enabled = true;                          // SYSCONF's BT.MOT: rumble on, as a console ships

void ret(PPCContext& c, uint32_t v) { c.r[3] = v; }
void stf(uint32_t a, float f) { uint32_t u; std::memcpy(&u, &f, 4); st32(a, u); }

// ---- what each channel holds ------------------------------------------------------------------
// A Remote on channel 0 always; with the Classic, a Remote holding one
// wherever video.cpp has a Classic Controller.
bool present(uint32_t chan) {
    if (chan >= 4) return false;
    if (!classic_game) return chan == 0;
    return video_classic((int)chan).connected;
}

// ---- connections, as the Bluetooth stack reports them -------------------------------------------
struct Channel {
    uint32_t kpad_connect_cb = 0, wpad_connect_cb = 0, extension_cb = 0;
    uint64_t since = 0;                              // the time base when a connect callback was registered
    bool told = false, told_present = false;         // what the game was last told
};
Channel chans[4];
constexpr uint64_t kSettle = 243000000 / 4 / 5;      // 0.2 s of the time base

void call(const PPCContext& c, uint32_t cb, uint32_t a, uint32_t b) {
    PPCContext e = c;
    e.r[3] = a;
    e.r[4] = b;
    ppc_call_indirect(e, cb);
}

bool delivering = false;
// Tells the game of every change since it was last told, on its callbacks.
void deliver(const PPCContext& c) {
    if (delivering || !classic_game) return;         // a callback that probes: no nesting
    delivering = true;
    uint64_t now = os_tb_now();
    for (uint32_t i = 0; i < 4; ++i) {
        Channel& ch = chans[i];
        if (!ch.kpad_connect_cb && !ch.wpad_connect_cb) continue;
        if (now - ch.since < kSettle) continue;
        bool here = present(i);
        if (ch.told && ch.told_present == here) continue;
        if (!ch.told && !here) { ch.told = true; ch.told_present = false; continue; }   // nothing to say
        ch.told = true;
        ch.told_present = here;
        int32_t reason = here ? WPAD_ERR_NONE : WPAD_ERR_NO_CONTROLLER;
        rt_log("wpad: channel %u %s", i + 1, here ? "connected, a Classic Controller attached" : "disconnected");
        if (ch.wpad_connect_cb) call(c, ch.wpad_connect_cb, i, (uint32_t)reason);
        if (ch.kpad_connect_cb) call(c, ch.kpad_connect_cb, i, (uint32_t)reason);
        // the extension, as the Remote reports it after connecting (the
        // callback may have been registered by the connect callback)
        if (here && ch.extension_cb) call(c, ch.extension_cb, i, WPAD_DEV_CLASSIC);
    }
    delivering = false;
}

void set_callback(PPCContext& c, uint32_t Channel::*slot, bool connect) {   // (chan, cb) -> the previous one
    uint32_t chan = c.r[3], cb = c.r[4];
    if (chan >= 4) { ret(c, 0); return; }
    Channel& ch = chans[chan];
    uint32_t old = ch.*slot;
    ch.*slot = cb;
    if (connect && cb && cb != old) { ch.since = os_tb_now(); ch.told = false; }
    ret(c, old);
}

PPCFunc vi_retrace = nullptr;

// ---- reading -------------------------------------------------------------------------------------
void hle_WPADProbe(PPCContext& c) {                  // (chan, u32* type)
    deliver(c);
    uint32_t chan = c.r[3];
    bool here = present(chan);
    if (c.r[4]) st32(c.r[4], !here ? WPAD_DEV_NOT_FOUND : classic_game ? WPAD_DEV_CLASSIC : WPAD_DEV_CORE);
    ret(c, here ? 0 : (uint32_t)WPAD_ERR_NO_CONTROLLER);
}

struct Prev { uint32_t hold = 0, cl_hold = 0; float x = 0, y = 0; float acc[3] = {0, -1, 0}; bool swing = false; };
Prev prev[4];

// KPADRead(chan, KPADStatus* samples, len): one sample, newest first.
// KPADReadEx(chan, samples, len, s32* err) is the same with an error out.
void hle_KPADRead(PPCContext& c) {
    deliver(c);
    uint32_t chan = c.r[3], s = c.r[4], len = c.r[5];
    if (!present(chan) || !s || !len) { ret(c, 0); return; }
    Prev& pv = prev[chan];
    // the Remote: the host's on channel 0; one that holds a Classic
    // Controller points nowhere and has no button pressed
    PadState p = chan == 0 && !classic_game ? video_pad() : PadState{};
    for (uint32_t i = 0; i < kpad_status_size; i += 4) st32(s + i, 0);
    st32(s + 0x00, p.buttons);                        // hold
    st32(s + 0x04, p.buttons & ~pv.hold);             // trig
    st32(s + 0x08, pv.hold & ~p.buttons);             // release
    pv.hold = p.buttons;
    float acc[3] = {0, -1, 0};                        // gravity, the Remote level
    if (p.tilt) { acc[1] = 0; acc[2] = (float)p.tilt; p.pointer = false; }   // raised: along its length
    if (p.shake) acc[0] = (pv.swing = !pv.swing) ? 2.0f : -2.0f;
    float d2 = 0, a2 = 0;
    for (int i = 0; i < 3; ++i) {
        stf(s + 0x0C + 4 * i, acc[i]);                // acc
        d2 += (acc[i] - pv.acc[i]) * (acc[i] - pv.acc[i]);
        a2 += acc[i] * acc[i];
        pv.acc[i] = acc[i];
    }
    stf(s + 0x18, std::sqrt(a2));                     // acc_value
    stf(s + 0x1C, std::sqrt(d2));                     // acc_speed
    float x = p.pointer ? p.x : pv.x, y = p.pointer ? p.y : pv.y;
    stf(s + 0x20, x);                                 // pos
    stf(s + 0x24, y);
    stf(s + 0x28, x - pv.x);                          // vec
    stf(s + 0x2C, y - pv.y);
    pv.x = x;
    pv.y = y;
    stf(s + 0x34, 1.0f);                              // horizon: level
    stf(s + 0x48, 2.0f);                              // dist: two metres from the sensor bar
    stf(s + 0x54, acc[0]);                            // acc_vertical
    stf(s + 0x58, acc[1]);
    st8(s + 0x5C, classic_game ? WPAD_DEV_CLASSIC : WPAD_DEV_CORE);   // dev_type
    st8(s + 0x5D, 0);                                 // wpad_err: none
    st8(s + 0x5E, p.pointer ? 2 : 0);                 // dpd_valid_fg: both sensor-bar points seen
    st8(s + 0x5F, classic_game ? WPAD_FMT_CLASSIC_ACC_DPD : WPAD_FMT_CORE_ACC_DPD);   // data_format
    if (classic_game) {                               // ex_status.cl
        ClassicState k = video_classic((int)chan);
        if (classic_filter) classic_filter((int)chan, k);
        st32(s + 0x60, k.buttons);                    // hold, trig, release
        st32(s + 0x64, k.buttons & ~pv.cl_hold);
        st32(s + 0x68, pv.cl_hold & ~k.buttons);
        pv.cl_hold = k.buttons;
        stf(s + 0x6C, k.lx);                          // lstick
        stf(s + 0x70, k.ly);
        stf(s + 0x74, k.rx);                          // rstick
        stf(s + 0x78, k.ry);
        stf(s + 0x7C, k.lt);                          // ltrigger, rtrigger
        stf(s + 0x80, k.rt);
    }
    ret(c, 1);
}

void hle_KPADReadEx(PPCContext& c) {
    uint32_t err = c.r[6];
    hle_KPADRead(c);
    if (err) st32(err, c.r[3] ? 0 : (uint32_t)WPAD_ERR_NO_CONTROLLER);   // KPAD_READ_ERR_NONE / NO_CONTROLLER
}

void hle_KPADGetSensorHeight(PPCContext& c) { c.f[1] = c.ps1[1] = 0.0; }

// ---- WPAD's own samples ----------------------------------------------------------------------------
// Below KPAD a Remote's sample is a WPADStatus (0x2A bytes), a WPADFSStatus
// with the Nunchuk (0x32) or a WPADCLStatus with the Classic (0x36). Games
// read them with WPADRead, from a ring WPAD fills (WPADSetAutoSamplingBuf,
// the newest at WPADGetLatestIndexInBuf), or, in the 2007 KPAD, through
// kpad_wpad_status: KPAD's copy of the samples of one device into the
// caller's ring of 16. Here the newest sample is always at index 0, written
// when read.
constexpr uint32_t kStatusSize[3] = {0x2A, 0x32, 0x36};   // by device: core, Nunchuk, Classic

struct Ring { uint32_t buf = 0, len = 0; };
Ring rings[4];

// A stick's WPAD reading: the 2007 SDK's KPAD reads 60..308 along the radius
// as its travel (clamp_stick_circle), 30..180 as a trigger's; the host's
// deflection is put back where KPAD would read it as the same value.
void st_stick(uint32_t a, float x, float y) {
    float r = std::sqrt(x * x + y * y), k = r > 0 ? (60.0f + 248.0f * std::min(r, 1.0f)) / r : 0;
    st16(a, (uint16_t)(int16_t)std::lround(x * k));
    st16(a + 2, (uint16_t)(int16_t)std::lround(y * k));
}
uint8_t trigger(float t) { return t > 0 ? (uint8_t)std::lround(30.0f + 150.0f * std::min(t, 1.0f)) : 0; }

// The channel's sample as its device gives it: the Remote still, no sensor-bar
// point seen, its buttons the host's on channel 0 unless it holds a Classic.
uint32_t write_status(uint32_t chan, uint32_t s) {
    bool here = present(chan);
    uint32_t dev = !here ? WPAD_DEV_NOT_FOUND : classic_game ? WPAD_DEV_CLASSIC : WPAD_DEV_CORE;
    uint32_t size = kStatusSize[dev == WPAD_DEV_CLASSIC ? 2 : 0];
    for (uint32_t i = 0; i < size; i += 2) st16(s + i, 0);
    if (!here) {
        st8(s + 0x28, (uint8_t)dev);
        st8(s + 0x29, (uint8_t)WPAD_ERR_NO_CONTROLLER);
        return dev;
    }
    if (chan == 0 && !classic_game) st16(s + 0x00, (uint16_t)video_pad().buttons);
    st8(s + 0x28, (uint8_t)dev);                      // dev, err: none
    if (dev == WPAD_DEV_CLASSIC) {
        ClassicState k = video_classic((int)chan);
        if (classic_filter) classic_filter((int)chan, k);
        st16(s + 0x2A, (uint16_t)k.buttons);          // clButton: KPAD's Classic bits are WPAD's
        st_stick(s + 0x2C, k.lx, k.ly);               // clLStickX, Y
        st_stick(s + 0x30, k.rx, k.ry);               // clRStickX, Y
        st8(s + 0x34, trigger(k.lt));                 // clTriggerL, R
        st8(s + 0x35, trigger(k.rt));
    }
    return dev;
}

void hle_WPADRead(PPCContext& c) {                   // (chan, void* status)
    deliver(c);
    if (c.r[3] < 4 && c.r[4]) write_status(c.r[3], c.r[4]);
}

void hle_WPADSetAutoSamplingBuf(PPCContext& c) {     // (chan, void* buf, u32 len)
    if (c.r[3] < 4) rings[c.r[3]] = {c.r[4], c.r[5]};
}

void hle_WPADGetLatestIndexInBuf(PPCContext& c) {    // (chan) -> the newest sample's index
    deliver(c);
    uint32_t chan = c.r[3];
    if (chan < 4 && rings[chan].buf && rings[chan].len) write_status(chan, rings[chan].buf);
    ret(c, 0);
}

// kpad_wpad_status(chan, buf, dev) -> buf: 16 samples of the channel's
// device, if it is `dev`, the newest at WPADGetLatestIndexInBuf (0).
void hle_kpad_wpad_status(PPCContext& c) {
    deliver(c);
    uint32_t chan = c.r[3], buf = c.r[4], want = c.r[5];
    if (chan < 4 && buf && want < 3) {
        uint32_t dev = present(chan) ? (classic_game ? WPAD_DEV_CLASSIC : WPAD_DEV_CORE) : WPAD_DEV_NOT_FOUND;
        if (dev == want)
            for (uint32_t i = 0; i < 16; ++i) write_status(chan, buf + i * kStatusSize[want]);
    }
    ret(c, buf);
}

}  // namespace

void wpad_set_kpad_status_size(uint32_t size) { kpad_status_size = size; }
void wpad_set_classic(bool on) { classic_game = on; }
void wpad_set_classic_filter(WpadClassicFilter f) { classic_filter = f; }
void wpad_filter_classic(int chan, ClassicState& s) { if (classic_filter) classic_filter(chan, s); }

void wpad_install() {
    auto nop = [](PPCContext&) {};
    auto zero = [](PPCContext& c) { c.r[3] = 0; };
    auto no_controller = [](PPCContext& c) { c.r[3] = (uint32_t)WPAD_ERR_NO_CONTROLLER; };
    // set-up and state
    for (const char* n : {"WPADInit", "WPADRegisterAllocator", "WPADDisconnect", "WPADSetCallbackByKPAD", "WPADSetSpeakerVolume", "KPADInit", "KPADInitEx", "KPADReset",
                          "KPADSetPosParam", "KPADSetAccParam", "KPADEnableDPD", "KPADDisableDPD"})
        ppc_hook(n, nop);
    ppc_hook("WPADGetStatus", [](PPCContext& c) { c.r[3] = WPAD_STATE_SETUP; });
    ppc_hook("WPADSaveConfig", [](PPCContext& c) { c.r[3] = 1; });
    ppc_hook("WPADGetSpeakerVolume", [](PPCContext& c) { c.r[3] = 0x40; });
    // the motor: the channel's pad rumbles
    ppc_hook("WPADEnableMotor", [](PPCContext& c) { motor_enabled = c.r[3] != 0; });
    ppc_hook("WPADIsMotorEnabled", [](PPCContext& c) { c.r[3] = motor_enabled; });
    ppc_hook("WPADControlMotor", [](PPCContext& c) {  // (chan, WPAD_MOTOR_STOP 0 / RUMBLE 1)
        static bool seen = false;
        if (!seen && c.r[4] == 1) { seen = true; rt_log("wpad: the motor first on (channel %u)", c.r[3] + 1); }
        video_set_rumble((int)c.r[3], motor_enabled && c.r[4] == 1);
    });
    // connections: the callbacks are kept and called (deliver)
    ppc_hook("KPADSetConnectCallback", [](PPCContext& c) { set_callback(c, &Channel::kpad_connect_cb, true); });
    ppc_hook("WPADSetConnectCallback", [](PPCContext& c) { set_callback(c, &Channel::wpad_connect_cb, true); });
    ppc_hook("WPADSetExtensionCallback", [](PPCContext& c) { set_callback(c, &Channel::extension_cb, false); });
    vi_retrace = ppc_hook("__VIRetraceHandler", [](PPCContext& c) {
        PPCContext saved = c;
        vi_retrace(c);
        deliver(saved);
    });
    // other callbacks: accepted, never called; the previous one returned is none
    for (const char* n : {"WPADSetSimpleSyncCallback", "WPADIsUsedCallbackByKPAD", "WPADIsSpeakerEnabled",
                          "WPADCanSendStreamData", "WPADGetSensorBarPosition", "WPADGetRadioSensitivity",
                          "WPADStartFastSimpleSync", "WPADStopSimpleSync", "KPADIsEnableAimingMode"})
        ppc_hook(n, zero);
    // anything addressed to a Remote's speaker or memory: there is none
    for (const char* n : {"WPADControlSpeaker", "WPADSendStreamData", "WPADGetInfoAsync"})
        ppc_hook(n, no_controller);
    ppc_hook("WPADProbe", hle_WPADProbe);
    ppc_hook("KPADRead", hle_KPADRead);
    ppc_hook("KPADReadEx", hle_KPADReadEx);
    ppc_hook("KPADGetSensorHeight", hle_KPADGetSensorHeight);
    ppc_hook("WPADRead", hle_WPADRead);
    ppc_hook("WPADSetAutoSamplingBuf", hle_WPADSetAutoSamplingBuf);
    ppc_hook("WPADGetLatestIndexInBuf", hle_WPADGetLatestIndexInBuf);
    ppc_hook("kpad_wpad_status", hle_kpad_wpad_status);
}
