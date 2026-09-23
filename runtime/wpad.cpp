// wiikit runtime — the Wii Remote, at the WPAD/KPAD layer.
//
// Below WPAD lies Nintendo's Bluetooth stack (WUD, BTA/BTE) talking HCI to
// the USB dongle through IOS. A port has no Remote to pair, so the cut is
// the public WPAD and KPAD API that games, KPAD itself, Wwise's speaker
// manager and the Home Button menu call: the Bluetooth stack never starts.
//
// For now the answer is "WPAD is ready and no Remote is connected": probes
// fail with WPAD_ERR_NO_CONTROLLER and KPADRead returns no samples. The
// mouse-driven Remote on channel 0 will be built on this file.
#include "rt.h"

namespace {

constexpr uint32_t WPAD_STATE_SETUP = 3;
constexpr int32_t WPAD_ERR_NO_CONTROLLER = -1;
constexpr uint32_t WPAD_DEV_NOT_FOUND = 253;

void ret(PPCContext& c, uint32_t v) { c.r[3] = v; }

void hle_WPADProbe(PPCContext& c) {                  // (chan, u32* type)
    if (c.r[4]) st32(c.r[4], WPAD_DEV_NOT_FOUND);
    ret(c, (uint32_t)WPAD_ERR_NO_CONTROLLER);
}

void hle_KPADRead(PPCContext& c) { ret(c, 0); }      // (chan, KPADStatus*, len): no samples

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
