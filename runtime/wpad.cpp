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
#include "rt.h"
#include "video.h"
#include <cmath>
#include <cstring>

namespace {

constexpr uint32_t WPAD_STATE_SETUP = 3;
constexpr int32_t WPAD_ERR_NO_CONTROLLER = -1;
constexpr uint32_t WPAD_DEV_CORE = 0, WPAD_DEV_NOT_FOUND = 253;
constexpr uint32_t KPAD_STATUS_SIZE = 0xF0;          // this SDK's KPADStatus

void ret(PPCContext& c, uint32_t v) { c.r[3] = v; }
void stf(uint32_t a, float f) { uint32_t u; std::memcpy(&u, &f, 4); st32(a, u); }

void hle_WPADProbe(PPCContext& c) {                  // (chan, u32* type)
    bool here = c.r[3] == 0;
    if (c.r[4]) st32(c.r[4], here ? WPAD_DEV_CORE : WPAD_DEV_NOT_FOUND);
    ret(c, here ? 0 : (uint32_t)WPAD_ERR_NO_CONTROLLER);
}

uint32_t prev_hold = 0;
float prev_x = 0, prev_y = 0;
float prev_acc[3] = {0, -1, 0};
bool swing = false;

// KPADRead(chan, KPADStatus* samples, len): one sample, newest first
void hle_KPADRead(PPCContext& c) {
    uint32_t chan = c.r[3], s = c.r[4], len = c.r[5];
    if (chan != 0 || !s || !len) { ret(c, 0); return; }
    PadState p = video_pad();
    for (uint32_t i = 0; i < KPAD_STATUS_SIZE; i += 4) st32(s + i, 0);
    st32(s + 0x00, p.buttons);                        // hold
    st32(s + 0x04, p.buttons & ~prev_hold);           // trig
    st32(s + 0x08, prev_hold & ~p.buttons);           // release
    prev_hold = p.buttons;
    float acc[3] = {0, -1, 0};                        // gravity, the Remote level
    if (p.shake) acc[0] = (swing = !swing) ? 2.0f : -2.0f;
    float d2 = 0, a2 = 0;
    for (int i = 0; i < 3; ++i) {
        stf(s + 0x0C + 4 * i, acc[i]);                // acc
        d2 += (acc[i] - prev_acc[i]) * (acc[i] - prev_acc[i]);
        a2 += acc[i] * acc[i];
        prev_acc[i] = acc[i];
    }
    stf(s + 0x18, std::sqrt(a2));                     // acc_value
    stf(s + 0x1C, std::sqrt(d2));                     // acc_speed
    float x = p.pointer ? p.x : prev_x, y = p.pointer ? p.y : prev_y;
    stf(s + 0x20, x);                                 // pos
    stf(s + 0x24, y);
    stf(s + 0x28, x - prev_x);                        // vec
    stf(s + 0x2C, y - prev_y);
    prev_x = x;
    prev_y = y;
    stf(s + 0x34, 1.0f);                              // horizon: level
    stf(s + 0x48, 2.0f);                              // dist: two metres from the sensor bar
    stf(s + 0x54, acc[0]);                            // acc_vertical
    stf(s + 0x58, acc[1]);
    st8(s + 0x5C, WPAD_DEV_CORE);                     // dev_type
    st8(s + 0x5D, 0);                                 // wpad_err: none
    st8(s + 0x5E, p.pointer ? 2 : 0);                 // dpd_valid_fg: both sensor-bar points seen
    st8(s + 0x5F, 2);                                 // data_format: buttons, acceleration, pointer
    ret(c, 1);
}

void hle_KPADGetSensorHeight(PPCContext& c) { c.f[1] = c.ps1[1] = 0.0; }

}  // namespace

void wpad_install() {
    auto nop = [](PPCContext&) {};
    auto zero = [](PPCContext& c) { c.r[3] = 0; };
    auto no_controller = [](PPCContext& c) { c.r[3] = (uint32_t)WPAD_ERR_NO_CONTROLLER; };
    // set-up and state
    for (const char* n : {"WPADInit", "WPADRegisterAllocator", "WPADDisconnect", "WPADSetAutoSamplingBuf",
                          "WPADSetCallbackByKPAD", "WPADSetSpeakerVolume", "WPADEnableMotor",
                          "WPADControlMotor", "KPADInit", "KPADReset", "KPADSetPosParam",
                          "KPADSetAccParam", "KPADEnableDPD", "KPADDisableDPD"})
        ppc_hook(n, nop);
    ppc_hook("WPADGetStatus", [](PPCContext& c) { c.r[3] = WPAD_STATE_SETUP; });
    ppc_hook("WPADSaveConfig", [](PPCContext& c) { c.r[3] = 1; });
    ppc_hook("WPADGetSpeakerVolume", [](PPCContext& c) { c.r[3] = 0x40; });
    // callbacks: accepted, never called; the previous one returned is none
    for (const char* n : {"WPADSetConnectCallback", "WPADSetExtensionCallback", "WPADSetSimpleSyncCallback",
                          "KPADSetConnectCallback", "WPADIsUsedCallbackByKPAD", "WPADIsMotorEnabled",
                          "WPADIsSpeakerEnabled", "WPADCanSendStreamData", "WPADGetSensorBarPosition",
                          "WPADGetRadioSensitivity", "WPADStartFastSimpleSync", "WPADStopSimpleSync",
                          "KPADIsEnableAimingMode"})
        ppc_hook(n, zero);
    // anything addressed to a Remote: there is none
    for (const char* n : {"WPADControlSpeaker", "WPADSendStreamData", "WPADGetInfoAsync"})
        ppc_hook(n, no_controller);
    ppc_hook("WPADProbe", hle_WPADProbe);
    ppc_hook("KPADRead", hle_KPADRead);
    ppc_hook("KPADGetSensorHeight", hle_KPADGetSensorHeight);
}
