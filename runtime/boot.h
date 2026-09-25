// wiikit runtime — the state a disc game finds at __start (boot.cpp).
#pragma once
#include <cstdint>

// Load main.dol and set up memory as the system leaves it for a disc game,
// from a tree made by `python -m wiikit.disc GAME --extract DIR`. Returns
// the entry point.
// eurgb60: SYSCONF's IPL.E60, for a PAL disc: the IPL leaves VI in EuRGB60.
uint32_t boot_disc(const char* extract_dir, bool eurgb60 = false);
