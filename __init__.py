"""wiikit — game-agnostic building blocks for Wii reverse engineering and ports.

Each module handles one thing the Wii platform, its SDK or its toolchain
imposes on every game, independent of any particular title:

    disc     disc images (.iso, .wbfs): partitions, decryption, FST, extraction
    aes      AES-128-CBC decryption in pure Python (pycryptodome if present)
    dol      DOL and ELF executables: address map, symbols, comparison, library census
    cw       CodeWarrior C++ name demangling
    ppc      Gekko/Broadway decoding (paired singles included), disassembly, xrefs
    gxtex    GX texture formats to RGBA, PNG writer
    tpl      TPL texture palettes
    u8       U8 archives
    dsp      DSP-ADPCM header and decoder
    recomp   static recompilation of Gekko code to C++ (layer 4)

wiikit/runtime/ holds the C++ side: ppc.h (the CPU model the generated code
runs against), mem.cpp (guest memory, DOL loading), services_stub.cpp.

Layers 1-3 are pure Python 3.8+ with no dependencies. Game-specific knowledge
belongs in the game's own tools/, not here.
"""
