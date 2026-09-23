"""Recompile a symbolised Wii executable to C++.

    python -m wiikit.recomp GAME.elf --out build/recomp [--per-file 8000] [--no-comments]

Writes, into --out:
    funcs.h          prototypes of every recompiled entry point
    recomp_NNN.cpp   the functions, split by instruction count
    table.cpp        the sorted address -> function table for indirect calls
    CMakeLists.txt   builds the code into a static library, plus the link check
    report.txt       units, entries, switch tables, anything not handled

Only symbolised images for now (ELF with .symtab): the symbols give function
boundaries. The generated code needs wiikit/runtime/ppc.h.
"""
import argparse
import os
import time

from .. import cw
from ..dol import Image
from . import emit as E
from .program import Program

RUNTIME = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "runtime"))


class FnCtx:
    """Control-flow answers for one entry's body."""

    def __init__(self, prog, entry, entries):
        self.prog, self.entry, self.entries = prog, entry, entries
        self.unit, _ = prog.body(entry)
        self.tables = prog.tables

    def jump(self, t):
        if self.entry <= t < self.unit.end:
            return f"goto L_{t:08X};"
        if t in self.entries:
            return f"{{ {E.fname(t)}(c); return; }}"
        return f'{{ ppc_unimplemented(c, 0x{t:08X}u, "branch to unknown target"); return; }}'

    def call(self, t):
        if t in self.entries:
            return f"{E.fname(t)}(c);"
        return f'ppc_unimplemented(c, 0x{t:08X}u, "call to unknown target");'


def generate(img, out, per_file=8000, comments=True, log=print):
    t0 = time.time()
    prog = Program(img, log=log)
    entries = set(prog.entries)
    os.makedirs(out, exist_ok=True)
    unsupported, files, n_ins = {}, [], 0
    buf, count, idx = [], 0, 0

    def flush():
        nonlocal buf, count, idx
        if not buf:
            return
        name = f"recomp_{idx:03d}.cpp"
        with open(os.path.join(out, name), "w", encoding="utf-8", newline="\n") as f:
            f.write('#include "funcs.h"\n\n')
            f.write("\n".join(buf))
        files.append(name)
        buf, count, idx = [], 0, idx + 1

    for e in prog.entries:
        unit, body = prog.body(e)
        fn = FnCtx(prog, e, entries)
        labels = prog.labels(e)
        title = unit.name if e == unit.start else f"{unit.name}+0x{e - unit.start:X}"
        lines = [f"// {cw.demangle(title)}", f"void {E.fname(e)}(PPCContext& c) {{"]
        for a, ins in body:
            if a in labels:
                lines.append(f"L_{a:08X}:")
            try:
                code = E.emit(ins, a, fn)
            except E.Unsupported as ex:
                unsupported[str(ex)] = unsupported.get(str(ex), 0) + 1
                code = [f'ppc_unimplemented(c, 0x{a:08X}u, "{ex}");']
            note = f"  // {a:08X} {_text(ins, a)}" if comments else ""
            lines.append("    " + code[0] + note)
            lines.extend("    " + x for x in code[1:])
        last = body[-1][1] if body else None
        if last is None or not E.terminates(last):
            lines.append("    " + fn.jump(unit.end) if unit.end in entries else
                         f'    ppc_unimplemented(c, 0x{unit.end:08X}u, "fell off the end"); return;')
        lines.append("}\n")
        buf.append("\n".join(lines))
        count += len(body)
        n_ins += len(body)
        if count >= per_file:
            flush()
    flush()

    with open(os.path.join(out, "funcs.h"), "w", encoding="utf-8", newline="\n") as f:
        f.write("#pragma once\n#include \"ppc.h\"\n\n")
        for e in prog.entries:
            f.write(f"void {E.fname(e)}(PPCContext& c);\n")
    with open(os.path.join(out, "table.cpp"), "w", encoding="utf-8", newline="\n") as f:
        f.write('#include "funcs.h"\n#include <cstddef>\n\n')
        f.write("struct PPCFuncEntry { uint32_t addr; PPCFunc fn; };\n")
        f.write("extern const PPCFuncEntry g_ppc_funcs[] = {\n")
        for e in prog.entries:
            f.write(f"    {{0x{e:08X}u, {E.fname(e)}}},\n")
        f.write("};\n")
        f.write(f"extern const size_t g_ppc_nfuncs = {len(prog.entries)};\n")
    with open(os.path.join(out, "CMakeLists.txt"), "w", encoding="utf-8", newline="\n") as f:
        rt = RUNTIME.replace("\\", "/")
        srcs = "\n    ".join(files + ["table.cpp"])
        f.write(f"""cmake_minimum_required(VERSION 3.20)
project(wiikit_recomp CXX)
set(CMAKE_CXX_STANDARD 20)
set(WIIKIT_RUNTIME "{rt}")
add_library(recomp STATIC
    {srcs})
target_include_directories(recomp PUBLIC ${{WIIKIT_RUNTIME}})
target_compile_options(recomp PRIVATE -Wno-unused-label -Wno-unused-variable)
add_library(wiikit_rt STATIC ${{WIIKIT_RUNTIME}}/services_stub.cpp ${{WIIKIT_RUNTIME}}/mem.cpp)
target_include_directories(wiikit_rt PUBLIC ${{WIIKIT_RUNTIME}})
# the generated code calls the runtime services and the runtime reads the
# generated dispatch table: a cycle, which CMake resolves by repeating both
target_link_libraries(recomp PUBLIC wiikit_rt)
target_link_libraries(wiikit_rt PUBLIC recomp)
add_executable(linkcheck ${{WIIKIT_RUNTIME}}/linkcheck.cpp)
target_link_libraries(linkcheck recomp)
# a project can add its own targets (tests, the game) with -DWIIKIT_EXTRA=file.cmake
if(DEFINED WIIKIT_EXTRA)
  include(${{WIIKIT_EXTRA}})
endif()
""")
    extra = [e for e in prog.entries if prog.unit_at(e).start != e]
    report = [
        f"units            {len(prog.units)} ({sum(1 for u in prog.units if u.name.startswith('unk_'))} gaps)",
        f"entries          {len(prog.entries)} ({len(extra)} inside units)",
        f"instructions     {n_ins}",
        f"switch tables    {len(prog.tables)} resolved, {prog.unresolved_tables} unresolved",
        f"bad targets      {len(prog.bad_targets)}",
        f"unsupported      {unsupported or 'none'}",
        f"files            {len(files)}",
        f"time             {time.time() - t0:.1f} s",
    ]
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(report) + "\n")
        for a, t in prog.bad_targets:
            f.write(f"bad target {a:08X} -> {t:08X}\n")
    log("\n".join(report))


def _text(ins, a):
    from .. import ppc
    return ppc.text(ins, a)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image")
    ap.add_argument("--out", required=True)
    ap.add_argument("--per-file", type=int, default=8000)
    ap.add_argument("--no-comments", action="store_true")
    a = ap.parse_args()
    img = Image(a.image)
    if not img.symbols:
        raise SystemExit("needs a symbolised ELF")
    generate(img, a.out, a.per_file, not a.no_comments)


if __name__ == "__main__":
    main()
