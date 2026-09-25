# wiikit

A game-agnostic toolkit for Wii reverse engineering and native PC ports:
disc images, executables, the Gekko CPU, a static recompiler from Gekko
code to C++, and a runtime that replaces the Wii's hardware under the
recompiled game.

The same idea as [ps2kit](https://github.com/vs-sr-dev/pc-extermination/tree/main/ps2kit),
for the Wii. Each Wii game has its own engine and formats, but a large part
of every port is the *same* work: the same disc encryption, executable
formats, CPU, SDK and GPU. wiikit collects that shared part. It grows
inside the ports: each piece is written because a game needed it, then
kept free of that game's knowledge. Game formats and game fixes live in
the ports.

## Ports built on it

| Port | Game | What it asked of wiikit |
|---|---|---|
| [pc-victorious](https://github.com/vs-sr-dev/pc-victorious) | Victorious: Taking the Lead (2012) | everything so far: the disc, the symbolised ELF, the recompiler, the runtime from `__start` to a played, heard, 16:9 game |

Some pieces were first written for two earlier Wii studies (The Last Story
and Final Fantasy Crystal Chronicles: The Crystal Bearers): the disc
extractor, the GX texture decoder, TPL, U8 and DSP-ADPCM.

## Using it

A port takes wiikit as a git submodule at `wiikit/`, so that
`python -m wiikit.…` works from the port's root and the recompiler finds
`wiikit/runtime` next to itself:

```sh
git submodule add https://github.com/vs-sr-dev/wiikit.git wiikit
git clone --recursive <port>          # or: git submodule update --init
```

Each port pins a wiikit commit and moves it forward deliberately.

```sh
python -m wiikit.disc GAME.wbfs --info           # .iso, .wbfs, .rvz, .wia
python -m wiikit.rvz GAME.rvz                   # its tables and compression
python -m wiikit.disc GAME.wbfs --extract build/extract
python -m wiikit.dol build/extract/sys/main.dol --info
python -m wiikit.ppc build/extract/sys/main.dol --at 80006124
python -m wiikit.cw 'process__19CSongMoveBlockActorFf'
python -m wiikit.u8 ARCHIVE.arc
python -m wiikit.tpl TEXTURE.tpl build/out
python -m wiikit.recomp GAME.elf --out build/recomp [--hooks game-hooks.txt]
python -m wiikit.recomp main.dol --out build/recomp --symbols names.tsv   # stripped
python -m wiikit.profile build/recomp-build/wiiboot.exe run.err [build/symbols.tsv]
```

Building recompiled code needs CMake, Ninja, a C++20 compiler (clang from
MSYS2 so far) and SDL3; running it needs OpenGL 4.5. The generated
project includes `runtime/runtime.cmake`; a port adds its own targets and
its own layer (`RtGameLayer`) with `-DWIIKIT_EXTRA=file.cmake`.

## Layers

| Layer | Question it answers | Now | Next |
|---|---|---|---|
| 1. Recognise | What is on this disc? | `disc --info`: game id, partitions, WBFS usage; `rvz`: RVZ/WIA tables | a `fingerprint`: magics, SDK library dates, middleware found by symbol or string (Scaleform, Wwise, Bink, NW4R, Home Button) |
| 2. Extract | Turn standard formats into standard files | `disc` (ISO, WBFS, RVZ and WIA; AES, FST), `u8`, `tpl`, `gxtex`, `dsp` | palette formats C4/C8/C14X2 in Python (the runtime's C++ `gxtex` has them), BRSTM/BRSAR, THP, BNR |
| 3. Map code | What does the code do, where? | `dol` (DOL and ELF, one address map, symbols, `--same-as`, `--libs`), `cw` (CodeWarrior demangler), `ppc` (Gekko decoder with paired singles, disassembly, callers, lis/addi and SDA xrefs, instruction census) | `ppc --mix` without symbols; `sig`: library functions named by signature in stripped executables (Dolphin's `.dsy`, symbolised ELFs) |
| 4. Translate | Turn Gekko code into C++ | `recomp`: units and entry points to a fixed point, switch tables, one C++ function per entry, dispatch table, CMake project; stripped executables: function discovery (`recomp/discover.py`), switch tables sized from the code, names and hooks from a `symbols.tsv` | faithful single-precision rounding |
| 5. Runtime | Replace the hardware | `ppc.h` (the CPU model), `core`/`mem` (guest space, dispatch, hooks), `os` (guest threads on host threads, interrupts, time), `hw` (PI, VI, DSP micro-codes, AI, EXI, SI, Hollywood), `gx` (FIFO parsing, vertex and texture decoding, the record), `gxtex`, `gxshader` (TEV and XF to GLSL), `video` (SDL3 window, OpenGL 4.5 renderer), `ios` + `disc` (IOS HLE at the IPC registers), `sysconf`, `boot`, `wpad` (the Wii Remote on the mouse and keys; the Classic Controller on keys and SDL gamepads, up to four, in KPAD's status and WPAD's own samples; the connect and extension callbacks), `ax` (the AX micro-code) and `audio` (SDL3 output), a port's own layer (`RtGameLayer`), a sampling profiler, and `wiiboot` | the early (2006–07) AX micro-code, locked-cache DMA, synthetic Remote motion (swing, thrust, shake) from mouse gestures, fog and Z textures, Dolphin as the oracle |

How the recompiler, the runtime, the renderer and the audio work is
written up, with Victorious as the case, in pc-victorious's
[09-recompiler](https://github.com/vs-sr-dev/pc-victorious/blob/main/docs/09-recompiler.md),
[11-runtime](https://github.com/vs-sr-dev/pc-victorious/blob/main/docs/11-runtime.md),
[12-renderer](https://github.com/vs-sr-dev/pc-victorious/blob/main/docs/12-renderer.md) and
[13-audio](https://github.com/vs-sr-dev/pc-victorious/blob/main/docs/13-audio.md).

## Principles

* Pure Python, no dependencies, for layers 1–4; the runtime (layer 5) is
  C++20, and its one dependency is SDL3, for the window and the sound. The
  one exception is speed, not function: `aes` uses pycryptodome when it is
  installed (1.3 MB/s in pure Python, a whole disc in about 18 minutes,
  against seconds), and gives the same bytes either way.
* Every claim is checked on a real disc before it goes in.
* Game knowledge stays out.
* **Every change is checked on every port** before it goes in: each still
  gets as far as it did (Victorious boots, plays and sounds; the others as
  far as they have come).

## Checks behind each module

| Module | Checked by |
|---|---|
| `aes` | the FIPS-197 C.1 vector; equal to pycryptodome on a random cluster and on disc data (`python -m wiikit.aes`) |
| `disc` | Victorious (WBFS): re-extraction equal to the previous extractor for all 46 files. A PAL disc (ISO): 3 751 files extracted |
| `rvz` | a PAL disc as RVZ (zstd 19, 128 KiB chunks): every raw region, junk filler included, equal to DolphinTool's ISO; the whole DATA partition extracted from the RVZ equal to the ISO's, 3 759 of 3 759 files, in 57 s |
| `dol` | `--same-as`: all ten DOL sections equal in Victorious's ELF |
| `cw` | 20 619 of 20 619 function names demangled, including templates, conversion operators and anonymous namespaces |
| `ppc` | Victorious: 1 658 815 instructions, none undecoded; equal to capstone on every non-paired-single instruction up to standard aliases; paired-single fields checked by prologue/epilogue symmetry in 1 393 functions. A stripped 2007 DOL: 744 768 words, 14 non-zero words undecoded (data in text) |
| `gxtex` | The Last Story: byte-identical to textures Dolphin dumped from the running game |
| `tpl`, `u8` | the Home Button archives (105 files; the icon decodes correctly) |
| `dsp` | Crystal Bearers' audio, by ear and spectrogram |
| `recomp/discover` | Victorious's DOL against its ELF: 20 612 of 20 619 function starts found; 0 units cut through a function (no branch of any function crosses a unit start); 353 units of dead code after tail calls, harmless. A stripped 2007 PAL DOL: 8 738 units, 332 switch tables sized from the code (3 unresolved), no branch to an unknown target. Victorious's generated C++ unchanged, 201 of 201 files |
| `recomp` + `runtime` | Victorious: all 20 653 functions compile and link; the game's own `sprintf`, `strtod`, 64-bit division, `sin`/`cos`, `qsort` with game comparators, `PSMTX*` paired-single matrices and `memcpy`/`memset`, run natively, match the host in 15 of 15 tests |
| `runtime` (hardware) | Victorious boots from `__start` to its main loop: the SDK's own `OSInit` report, its anti-modchip device check, the Bink logos, Wwise on AX, frames of GX commands |
| `runtime` (renderer) | Victorious, by eye: the Wii Strap screen, the Bink logos (indirect textures), the Scaleform title and menus, the episodes in 3D, at 30 frames a second |
| `runtime` (audio) | Victorious, by ear: music, voices, effects, the Bink movies' sound, the rhythm games |
| `runtime` (Remote) | Victorious, by hand: its first episode played with the mouse, the pointer under the mouse, the rhythm game's presses, holds and shakes |
| `runtime` (Classic Controller) | a stripped 2010 game that learns of controllers only from the connect callback, by hand: played with an Xbox One pad, and with the keys, the mouse and the pad at once. A stripped 2009 game that reads the Classic from WPAD's raw samples (the 2007 KPAD's copy of them, `WPADGetLatestIndexInBuf`): played with the same pad into its first battle |

## Known gaps

* `ppc.text` renders standard simplified mnemonics (`sub`, `clrrwi`, `mr.`,
  `crclr`…). Tools that need the canonical operation use `decode()`'s
  `op` and fields, never the text.
* Constant tracking in `ppc.Tracker` is linear through a function and
  ignores control flow. That is enough for CodeWarrior's `lis`/`addi` pairs.
* `ppc --mix` needs function symbols: on a stripped DOL it counts nothing.
* Discovery misses assembly reached only by an address built in code
  (exception handlers) and tiny functions reached only by pointer: the
  runtime reports them as unknown targets, to be given as seeds.

## History

wiikit grew inside pc-victorious from its first session to its seventh and
was split out with its history (`git subtree split`) when a second port
began. The commit messages of that period describe Victorious's sessions;
the changes in them are wiikit's.

## Licence

MIT — see [LICENSE](LICENSE). wiikit contains no game data and no Nintendo
code; it reads and replaces, it does not include.
