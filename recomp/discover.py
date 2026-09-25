"""Function discovery for executables without symbols.

The recompiler's units come from sized function symbols. A stripped DOL has
none; this finds them from the code, for CodeWarrior's output:

1. Seeds, strong evidence of a function start: the entry point; every `bl`
   target; every address given by the caller (a symbols.tsv, the runtime's
   log of unknown targets); and, right after a terminator, a stack-frame
   prologue (`stwu r1,-N(r1)` or `mflr r0`). Measured on Victorious's
   symbolised ELF, both are right 100% of the time after a terminator. (A
   `b` from far away is right 99.3%: the rest are jumps inside functions
   longer than the distance, so tail-call targets are left to step 3.)
2. Units: each seed runs to the next. A unit whose code runs into the next
   seed without a terminator (`blr`, `b`, `bctr`, `rfi`) is not split
   there: the next seed is an entry inside it, as CodeWarrior's
   `__save_gpr+N` entries are. Splitting inside a function would turn its
   loops into host recursion; merging two functions costs nothing.
3. Reachability: from each unit's entries, follow its control flow
   (conditional and unconditional branches inside the unit, switch tables,
   fall-through). Code that nothing reaches, right after a terminator, is a
   function nothing calls directly (a callback, a virtual method, a leaf
   tail-called from nearby) if no branch of the unit crosses that point: a
   branch from before it to beyond it, or from it or after back before it,
   means dead code inside one function instead (a `b` to a known start is
   a tail call and does not count). Data words pointing at code are not
   evidence alone: 30% of those after a terminator are switch cases.
4. Repeat 2-3 to a fixed point.

Switch tables are found as the recompiler finds them (lwzx off a lis/addi
base, then mtctr and bctr); without a data symbol their length is the run
of words that point into the unit, capped by a `cmplwi` bound on the index
when one is found before the jump.
"""
import bisect
import struct

from .. import ppc

BLR = 0x4E800020


def _terminates(i):
    if i.op == "bclr" and i.f["BO"] & 0x14 == 0x14 and not i.f["LK"]:
        return True
    if i.op == "b" and not i.f["LK"]:
        return True
    if i.op == "bcctr" and i.f["BO"] & 0x14 == 0x14 and not i.f["LK"]:
        return True
    return i.op == "rfi"


class Code:
    """The text segments decoded once."""

    def __init__(self, img):
        self.img = img
        self.words, self.ins = {}, {}
        self.ranges = []
        for s in img.text_segments():
            self.ranges.append((s.vaddr, s.vaddr + len(s.data)))
            for k in range(0, len(s.data) - 3, 4):
                a = s.vaddr + k
                w = struct.unpack_from(">I", s.data, k)[0]
                self.words[a] = w
                self.ins[a] = ppc.decode(w)
        self.ranges.sort()

    def segment_end(self, a):
        for lo, hi in self.ranges:
            if lo <= a < hi:
                return hi
        return None

    def is_code(self, a):
        i = self.ins.get(a)
        return i is not None and self.words[a] != 0 and i.op != ".long"


def table_targets(code, bctr, lo, hi, lookback=16):
    """Targets of the switch table jumped through by the bctr at `bctr`, or
    None if it is not a table jump. lo/hi bound the targets (the unit)."""
    img = code.img
    regs = {}                         # reg -> constant, from lis/addi/ori
    ctr_src = None
    idx_reg = base_reg = None
    start = max(bctr - 4 * lookback, lo)
    for a in range(start, bctr, 4):
        i = code.ins.get(a)
        if i is None:
            continue
        op, f = i.op, i.f
        if op == "addis" and f["A"] == 0:
            regs[f["D"]] = (f["simm"] << 16) & 0xFFFFFFFF
        elif op == "addi" and f["A"] in regs:
            regs[f["D"]] = (regs[f["A"]] + f["simm"]) & 0xFFFFFFFF
        elif op == "ori" and f["S"] in regs:
            regs[f["A"]] = regs[f["S"]] | f["uimm"]
        elif op == "lwzx":
            if f["A"] in regs:
                base_reg, idx_reg = f["A"], f["B"]
            elif f["B"] in regs:
                base_reg, idx_reg = f["B"], f["A"]
            ctr_src = (f["D"], regs.get(base_reg))
        elif op == "mtspr" and f["spr"] == 9 and ctr_src and ctr_src[0] == f["D"]:
            pass
        elif "D" in f and op not in ("stw", "stb", "sth", "stwu", "stfs", "stfd", "cmpi", "cmpli", "cmp", "cmpl"):
            regs.pop(f["D"], None)
    if not ctr_src or ctr_src[1] is None:
        return None
    base = ctr_src[1]
    # a bound on the index: cmplwi rX, N shortly before (the index is shifted
    # into idx_reg after the compare, so any cmpli in the window is taken)
    bound = None
    for a in range(start, bctr, 4):
        i = code.ins.get(a)
        if i is not None and i.op == "cmpli" and i.f.get("L", 0) == 0:
            bound = i.f["imm"] + 1
    return scan_table(img, base, lo, hi, bound)


def scan_table(img, base, lo, hi, bound=None):
    """The words of a jump table at `base` while they point into [lo, hi),
    at most `bound` of them."""
    out = []
    while (bound is None or len(out) < bound) and len(out) < 4096:
        raw = img.read(base + 4 * len(out), 4)
        if not raw or len(raw) < 4:
            break
        t = struct.unpack(">I", raw)[0]
        if not (lo <= t < hi and t % 4 == 0):
            break
        out.append(t)
    return out or None


def _data_pointers(img, code):
    out = set()
    for s in img.segments:
        if s.text:
            continue
        d = s.data
        for k in range(0, len(d) - 3, 4):
            t = struct.unpack_from(">I", d, k)[0]
            if t % 4 == 0 and t in code.words and code.is_code(t):
                out.add(t)
    return out


def _after_terminator(code, a):
    p = a - 4
    while p in code.words and code.words[p] == 0:
        p -= 4
    return p in code.ins and _terminates(code.ins[p])


def _is_prologue(i):
    return (i.op == "stwu" and i.f["D"] == 1 and i.f["A"] == 1) or         (i.op == "mfspr" and i.f["spr"] == 8 and i.f["D"] == 0)


def discover(img, seeds=(), log=print):
    """[(start, end)] of the units of a stripped executable, and the set of
    entries found inside units (fall-through merges)."""
    code = Code(img)
    strong = set(a for a in seeds if a in code.words)
    entry = getattr(img, "entry", None)
    if entry in code.words:
        strong.add(entry)
    for a, i in code.ins.items():
        if i.op == "b" and i.f["LK"]:
            t = ppc.branch_target(i, a)
            if t in code.words:
                strong.add(t)
        elif _is_prologue(i) and _after_terminator(code, a):
            strong.add(a)
    for lo, hi in code.ranges:                   # each segment's first code
        a = lo
        while a < hi and not code.is_code(a):
            a += 4
        if a < hi:
            strong.add(a)
    pointers = _data_pointers(img, code)
    rounds, cache = 0, {}
    while True:
        rounds += 1
        units, inner = _units(code, strong)
        new, reached_all = _unreached(code, units, inner, cache)
        if not new - strong:
            break
        strong |= new
    unreached_ptrs = sorted(p for p in pointers if p not in reached_all and p not in strong)
    log(f"discover: {len(units)} units, {len(inner)} inner entries, {len(strong)} seeds, "
        f"{rounds} rounds, {len(pointers)} data pointers into code "
        f"({len(unreached_ptrs)} to unreached code)")
    return units, inner


def _units(code, strong):
    starts = sorted(strong)
    units, inner = [], set()
    i = 0
    while i < len(starts):
        s = starts[i]
        seg_end = code.segment_end(s)
        j = i + 1
        while True:
            end = starts[j] if j < len(starts) and starts[j] < seg_end else seg_end
            # the last code word before `end`
            a = end - 4
            while a >= s and not code.is_code(a):
                a -= 4
            if end == seg_end or a < s or _terminates(code.ins[a]):
                break
            inner.add(starts[j])                 # falls through: merge
            j += 1
        # trim trailing padding
        e = end
        while e - 4 >= s and code.words.get(e - 4, 0) == 0:
            e -= 4
        units.append((s, max(e, s + 4)))
        i = j
    return units, inner


def _unreached(code, units, inner, cache):
    """New seeds: in each unit, the first code word of a run that nothing in
    the unit reaches, provided no branch crosses it (a clean cut). Unreached
    code that branches cross is dead code of the function around it."""
    new, reached_all = set(), set()
    inner_sorted = sorted(inner)
    for s, e in units:
        ent = tuple(inner_sorted[bisect.bisect_left(inner_sorted, s):bisect.bisect_left(inner_sorted, e)])
        key = (s, e, ent)
        if key not in cache:
            cache[key] = _unit_scan(code, s, e, ent)
        cuts, seen = cache[key]
        new |= cuts
        reached_all |= seen
    return new, reached_all


def _unit_scan(code, s, e, ent):
    edges = []                                   # (from, to) inside the unit
    known = {s, *ent}
    for a in range(s, e, 4):
        i = code.ins[a]
        if i.op in ("b", "bc") and not i.f["LK"]:
            t = ppc.branch_target(i, a)
            if s <= t < e and not (i.op == "b" and t in known):   # b to a start: a tail call
                edges.append((a, t))
        elif i.op == "bcctr" and not i.f["LK"] and i.f["BO"] & 0x14 == 0x14:
            for t in table_targets(code, a, s, e) or ():
                edges.append((a, t))
    succ = {}
    for a, t in edges:
        succ.setdefault(a, []).append(t)
    todo, seen = [s, *ent], set()
    while todo:
        a = todo.pop()
        while s <= a < e and a not in seen:
            seen.add(a)
            todo.extend(succ.get(a, ()))
            if _terminates(code.ins[a]):
                break
            a += 4
    cuts = set()
    a = s
    while a < e:
        if a not in seen and code.is_code(a):
            p = a - 4
            while p >= s and not code.is_code(p):
                p -= 4
            after_end = p < s or _terminates(code.ins[p]) or code.words[a - 4] == 0
            crossed = any((f < a < t) or (f >= a > t) for f, t in edges)
            if after_end and not crossed:
                cuts.add(a)
            while a < e and a not in seen:
                a += 4
        else:
            a += 4
    return cuts, seen


def load_names(path):
    """{addr: name} from a symbols.tsv: wiikit.dol's (addr, size, bind, type,
    section, name, demangled) or any tab-separated file whose first column
    is a hex address and whose name column is headed "name"."""
    names = {}
    with open(path, encoding="utf-8") as f:
        head = f.readline().rstrip("\n").split("\t")
        col = head.index("name") if "name" in head else 1
        for line in f:
            p = line.rstrip("\n").split("\t")
            if len(p) > col and p[col]:
                try:
                    names.setdefault(int(p[0], 16), p[col])
                except ValueError:
                    pass
    return names


def symbolise(img, names=None, seeds=(), log=print):
    """Give a stripped image function symbols from discovery, so that the
    recompiler, the hook lists and the tools work on it as on an ELF. Named
    addresses (from `names`) are also seeds; unnamed units are fn_XXXXXXXX."""
    from ..dol import Symbol
    names = names or {}
    units, inner = discover(img, seeds=set(seeds) | set(names), log=log)
    for a, e in units:
        img.symbols.append(Symbol(a, e - a, 1, 2, ".text", names.get(a, f"fn_{a:08X}")))
    img.symbols.sort(key=lambda x: x.addr)
    img._sym_addrs = [x.addr for x in img.symbols]
    img.by_name = {x.name: x for x in img.symbols}
    return units, inner
