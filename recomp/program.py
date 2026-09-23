"""The program model a recompiler needs: units, entry points, labels, switch tables.

Units are the code ranges that become C++: every sized function symbol in a
text segment, plus every non-zero gap between them (hand-written assembly
often has no sized symbol).

Entry points are the addresses a C++ function must exist for. They start as
the unit starts; then every `bl` target is an entry, every `b`/`bc` that
leaves its unit lands on an entry, and every branch that jumps backwards out
of an extra entry's body adds another. This is iterated to a fixed point. An
entry that is not a unit start (CodeWarrior's `__save_gpr+0x34` style entries)
gets its own C++ function, covering the code from the entry to the end of its
unit.

Switch tables: an unconditional `bctr` whose CTR came from `lwzx` off a
lis/addi-built base is a table jump. Its size comes from the data symbol at
the table's address. If the base is lost across a branch, the fallback takes
the one data table whose entries all point into this unit. Every other `bctr`
is an indirect tail call.
"""
import bisect
import struct

from .. import ppc


class Unit:
    __slots__ = ("start", "end", "name", "ins")

    def __init__(self, start, end, name):
        self.start, self.end, self.name = start, end, name
        self.ins = []                     # [(addr, Ins)]

    def __contains__(self, a):
        return self.start <= a < self.end


class Program:
    def __init__(self, img, log=print):
        self.img, self.log = img, log
        self.units = self._units()
        self._starts = [u.start for u in self.units]
        for u in self.units:
            u.ins = [(a, ppc.decode(w)) for a, w in ppc.words(img, u.start, u.end - u.start)]
        self.tables = {}                  # bctr address -> [targets]
        self._resolve_tables()
        self.entries = self._entries()

    # --- units ---------------------------------------------------------------------

    def _units(self):
        img = self.img
        funcs = sorted((f for f in img.functions()
                        if img.segment_at(f.addr) and img.segment_at(f.addr).text),
                       key=lambda f: f.addr)
        units = [Unit(f.addr, f.addr + f.size, f.name) for f in funcs]
        gaps = []
        for seg in img.text_segments():
            a, end = seg.vaddr, seg.vaddr + len(seg.data)
            for u in units:
                if u.start < seg.vaddr or u.start >= end:
                    continue
                if u.start > a:
                    gaps.append((a, u.start))
                a = max(a, u.end)
            if a < end:
                gaps.append((a, end))
        for a, b in gaps:
            data = img.read(a, b - a)
            nz = [i for i in range(0, len(data), 4) if data[i:i + 4] != b"\0\0\0\0"]
            if nz:                         # trim zero padding at both ends
                s, e = a + nz[0], a + nz[-1] + 4
                units.append(Unit(s, e, f"unk_{s:08X}"))
        units.sort(key=lambda u: u.start)
        return units

    def unit_at(self, a):
        i = bisect.bisect_right(self._starts, a) - 1
        return self.units[i] if i >= 0 and a in self.units[i] else None

    # --- switch tables ------------------------------------------------------------------

    def _resolve_tables(self):
        img = self.img
        dsyms = {s.addr: s for s in img.symbols if s.type == 1 and s.size}
        by_unit = {}
        for s in dsyms.values():          # data tables whose words all point into one unit
            if s.size % 4 or img.segment_at(s.addr) is None or img.segment_at(s.addr).text:
                continue
            words = [img.u32(s.addr + 4 * k) for k in range(s.size // 4)]
            u = self.unit_at(words[0]) if words and words[0] else None
            if u and all(w in u and w % 4 == 0 for w in words):
                by_unit.setdefault(u.start, []).append((s.addr, words))
        unresolved = 0
        for u in self.units:
            tr = ppc.Tracker(img)
            cand, via_lwzx, table = {}, set(), None
            used = set()
            for a, i in u.ins:
                if i.op == "lwzx":
                    via_lwzx.add(i.f["D"])
                    if i.f["A"] in tr.regs:
                        cand[i.f["D"]] = tr.regs[i.f["A"]]
                elif i.op == "mtspr" and i.f["spr"] == 9:
                    table = (cand.get(i.f["D"]), i.f["D"] in via_lwzx)
                tr.step(i)
                if i.op == "bcctr" and not i.f["LK"] and i.f["BO"] & 0x14 == 0x14 and table:
                    base, is_table = table
                    targets = None
                    if base in dsyms:
                        n = dsyms[base].size // 4
                        targets = [img.u32(base + 4 * k) for k in range(n)]
                    elif is_table:
                        free = [t for t in by_unit.get(u.start, []) if t[0] not in used]
                        if len(free) == 1:
                            base, targets = free[0]
                    if targets and all(t in u for t in targets):
                        self.tables[a] = targets
                        used.add(base)
                    elif is_table:
                        unresolved += 1
                        self.log(f"  unresolved switch table at {a:08X} in {u.name}")
                    table = None
        self.unresolved_tables = unresolved

    # --- entry points ---------------------------------------------------------------

    def _entries(self):
        entries = {u.start for u in self.units}
        self.bad_targets = []
        work = list(self.units)
        extra_done = set()
        while True:
            new = set()
            for u in self.units:
                for a, i in u.ins:
                    t = ppc.branch_target(i, a)
                    if t is None:
                        continue
                    if i.f.get("LK") or t not in u:
                        if t == a + 4 and i.op == "bc" and i.f.get("LK"):
                            continue          # bcl to next: reads the PC, not a call
                        if self.unit_at(t) is None:
                            self.bad_targets.append((a, t))
                            continue
                        if t not in entries:
                            new.add(t)
            # extra entries: backward branches out of their body need entries too
            for e in sorted(entries | new):
                if e in extra_done:
                    continue
                u = self.unit_at(e)
                if u is None or e == u.start:
                    continue
                extra_done.add(e)
                for a, i in u.ins:
                    if a < e:
                        continue
                    t = ppc.branch_target(i, a)
                    if t is not None and t in u and t < e and t not in entries:
                        new.add(t)
            if not new - entries:
                break
            entries |= new
        return sorted(entries)

    def body(self, entry):
        """(unit, [(addr, Ins)]) of the C++ function for this entry."""
        u = self.unit_at(entry)
        return u, [(a, i) for a, i in u.ins if a >= entry]

    def labels(self, entry):
        """Addresses inside this entry's body that are jumped to."""
        u, ins = self.body(entry)
        out = set()
        for a, i in ins:
            t = ppc.branch_target(i, a)
            if t is not None and not i.f.get("LK") and entry <= t < u.end:
                out.add(t)
            if a in self.tables:
                out.update(x for x in self.tables[a] if x >= entry)
        return out
