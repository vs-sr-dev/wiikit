"""Wii / GameCube executables: DOL and ELF, one address map, symbols.

    python -m wiikit.dol main.dol --info
    python -m wiikit.dol game.elf --same-as main.dol
    python -m wiikit.dol game.elf --symbols symbols.tsv
    python -m wiikit.dol game.elf --libs
    python -m wiikit.dol main.dol --read 0x80004000 64

DOL   0x000 u32[7] text offsets, 0x01C u32[11] data offsets,
      0x048 / 0x064 load addresses, 0x090 / 0x0AC sizes,
      0x0D8 bss address, 0x0DC bss size, 0x0E0 entry point. Big-endian.
ELF   The linker's output before `makedol`: big-endian ELF32, PT_LOAD
      segments; when the developer forgot to strip it, a .symtab.

Some discs ship the ELF next to the DOL. `--same-as` checks that every loaded
byte of one image equals the other's at the same address, which is what makes
the ELF's symbols valid for the retail code.

`--libs` sorts the functions of a symbolised image into the usual Wii
libraries by name (SDK prefixes, namespaces, class prefixes of known
middleware). It is a census by naming convention, good to a few percent.
"""
import argparse
import bisect
import re
import struct

from . import cw


class Segment:
    def __init__(self, vaddr, data, memsz, text, name=""):
        self.vaddr, self.data, self.memsz, self.text, self.name = vaddr, data, memsz, text, name

    @property
    def end(self):
        return self.vaddr + self.memsz

    def __repr__(self):
        return f"Segment({self.name} {self.vaddr:08X}-{self.end:08X} {'text' if self.text else 'data'})"


class Symbol:
    __slots__ = ("addr", "size", "bind", "type", "section", "name")

    def __init__(self, addr, size, bind, typ, section, name):
        self.addr, self.size, self.bind, self.type = addr, size, bind, typ
        self.section, self.name = section, name

    @property
    def is_func(self):
        return self.type == 2


class Image:
    """Loaded segments of a DOL or ELF, addressed by virtual address."""

    def __init__(self, path):
        self.path = path
        raw = open(path, "rb").read()
        self.symbols = []
        if raw[:4] == b"\x7fELF":
            self._elf(raw)
        else:
            self._dol(raw)
        self.segments.sort(key=lambda s: s.vaddr)
        self._starts = [s.vaddr for s in self.segments]
        self.symbols.sort(key=lambda s: s.addr)
        self._sym_addrs = [s.addr for s in self.symbols]
        self.by_name = {s.name: s for s in self.symbols}
        self.sda = self._sym("_SDA_BASE_")
        self.sda2 = self._sym("_SDA2_BASE_")

    def _sym(self, name):
        s = [x for x in self.symbols if x.name == name]
        return s[0].addr if s else None

    def _dol(self, d):
        offs = struct.unpack_from(">18I", d, 0)
        addrs = struct.unpack_from(">18I", d, 0x48)
        sizes = struct.unpack_from(">18I", d, 0x90)
        self.bss, self.bss_size, self.entry = struct.unpack_from(">3I", d, 0xD8)
        self.segments = []
        for i in range(18):
            if sizes[i]:
                name = f"T{i}" if i < 7 else f"D{i - 7}"
                self.segments.append(Segment(addrs[i], d[offs[i]:offs[i] + sizes[i]],
                                             sizes[i], i < 7, name))
        self.kind = "DOL"

    def _elf(self, d):
        if d[4] != 1 or d[5] != 2:
            raise ValueError("expected a big-endian ELF32")
        self.entry, phoff, shoff = struct.unpack_from(">III", d, 0x18)
        phentsize, phnum, shentsize, shnum, shstrndx = struct.unpack_from(">HHHHH", d, 0x2A)
        self.segments = []
        for i in range(phnum):
            typ, off, va, _, filesz, memsz, flags, _ = struct.unpack_from(">8I", d, phoff + i * phentsize)
            if typ == 1 and memsz:
                self.segments.append(Segment(va, d[off:off + filesz], memsz, bool(flags & 1), f"P{i}"))
        sh = [struct.unpack_from(">10I", d, shoff + i * shentsize) for i in range(shnum)]

        def cstr(tab, o):
            a = sh[tab][4] + o
            return d[a:d.index(b"\0", a)].decode("latin1")

        secname = [cstr(shstrndx, s[0]) for s in sh]
        for seg in self.segments:        # name segments after their section
            for i, s in enumerate(sh):
                if s[3] == seg.vaddr and s[5]:
                    seg.name = secname[i]
        bss = [s for i, s in enumerate(sh) if secname[i] in (".bss", ".sbss", ".sbss2")]
        self.bss = min((s[3] for s in bss), default=0)
        self.bss_size = max((s[3] + s[5] for s in bss), default=0) - self.bss
        for i, s in enumerate(sh):
            if s[1] != 2:                  # SHT_SYMTAB
                continue
            for k in range(s[5] // 16):
                n, v, sz, info, _, shn = struct.unpack_from(">IIIBBH", d, s[4] + k * 16)
                sec = secname[shn] if 0 < shn < len(sh) else ("ABS" if shn == 0xFFF1 else "UND")
                self.symbols.append(Symbol(v, sz, info >> 4, info & 15, sec, cstr(s[6], n)))
        self.kind = "ELF"

    # --- memory ----------------------------------------------------------

    def segment_at(self, addr):
        i = bisect.bisect_right(self._starts, addr) - 1
        if i >= 0 and addr < self.segments[i].end:
            return self.segments[i]
        return None

    def read(self, addr, n):
        s = self.segment_at(addr)
        if s is None:
            return None
        o = addr - s.vaddr
        b = s.data[o:o + n]
        return b + bytes(max(0, min(n, s.memsz - o) - len(b)))

    def u32(self, addr):
        b = self.read(addr, 4)
        return struct.unpack(">I", b)[0] if b and len(b) == 4 else None

    def cstring(self, addr, limit=256):
        b = self.read(addr, limit)
        if not b or 0 not in b:
            return None
        s = b[:b.index(0)]
        return s.decode("latin1") if s and all(32 <= c < 127 or c in (9, 10, 13) for c in s) else None

    def text_segments(self):
        return [s for s in self.segments if s.text]

    # --- symbols ---------------------------------------------------------

    def functions(self):
        return [s for s in self.symbols if s.is_func and s.size]

    def symbol_at(self, addr, want=(1, 2)):
        """'name' or 'name+0x..' for the object/function containing addr."""
        i = bisect.bisect_right(self._sym_addrs, addr) - 1
        while i >= 0 and self.symbols[i].addr > addr - 0x100000:
            s = self.symbols[i]
            if s.type in want and (s.addr == addr or s.addr <= addr < s.addr + s.size):
                return s.name if s.addr == addr else f"{s.name}+{addr - s.addr:#x}"
            i -= 1
        return None

    def find(self, query):
        """Functions whose name contains `query`; an exact name wins."""
        if query in self.by_name:
            return [self.by_name[query]]
        return [s for s in self.functions() if query in s.name]


# --- library census --------------------------------------------------------

_SDK = re.compile(r"^_*(GX|OS|DVD|VI|SI|EXI|AI|AX|AXFX|DSP|AR|ARQ|PAD|KPAD|WPAD|WUD|NAND|"
                  r"ISFS|IPC|IOS|ES|SC|CARD|MEM|TPL|DB|PPC|NWC24|WENC|CX|SO|VF|MIX|SYN|"
                  r"SEQ|ENC|NCD|NET|USB|LC|FS|VPAD|THP|Dvd|Os|Gx)[A-Z_a-z0-9]")
_BT = re.compile(r"^(bta|btm|btu|btsnd|bte|BTM|BTA|BTU|BTE|L2CA|l2c|l2cu|gki|GKI|hci|HCI|"
                 r"sdp|SDP|rfc|RFCOMM|port|PORT|avdt|AVDT|btif|uusb|hidh|HID)_")
_MSL = re.compile(r"^(mem\w+|str\w+|__\w+|v?s?n?printf|f\w+|sqrtf?|sinf?|cosf?|tanf?|atan2?f?|"
                  r"expf?|logf?|log10f?|powf?|floorf?|ceilf?|fmodf?|abs|labs|rand|srand|qsort|"
                  r"bsearch|malloc|free|calloc|realloc|ato[ifl]|strto\w+|to(upper|lower)|"
                  r"is(alpha|digit|space|upper|lower|xdigit|alnum|punct)|wcs\w+|mbs\w+|"
                  r"raise|signal|exit|abort|__\w+|_\w+)$")


def library_of(sym):
    """Best guess of the library a function symbol belongs to."""
    name = sym.name
    try:
        scope = cw.split(name)[0]
    except (ValueError, IndexError, KeyError):
        scope = []
    top = scope[0] if scope else ""
    cls = scope[-1] if scope else ""
    base = re.split(r"__[0-9QF]", name[2:] if name.startswith("__") else name)[0]
    if top in ("homebutton", "nw4hbm") or base.startswith(("HBM", "__HBM")):
        return "HomeButton"
    if top == "nw4r":
        return "NW4R"
    if top == "AK" or cls.startswith(("CAk", "Ak")) or base.startswith(("Ak", "CAk")):
        return "Wwise"
    if re.match(r"(GFx|GAS|G[A-Z])", cls) or base.startswith(("GFx", "GAS")):
        return "Scaleform GFx"
    if "Bink" in name or "bink" in name or re.match(r"^(rr|RAD|Rad|radmalloc|radfree)", base):
        return "Bink"
    if cls.startswith("TiXml"):
        return "TinyXML"
    if top == "std" or cls.startswith("_"):
        return "MSL / runtime"
    if not scope:
        if _SDK.match(base):
            return "RVL SDK"
        if _BT.match(base):
            return "Bluetooth stack"
        if _MSL.match(base):
            return "MSL / runtime"
        return "free functions"
    return "classes (engine / game)"


def census(img):
    out = {}
    for f in img.functions():
        lib = library_of(f)
        n, size = out.get(lib, (0, 0))
        out[lib] = (n + 1, size + f.size)
    return dict(sorted(out.items(), key=lambda kv: -kv[1][1]))


# --- command line ------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--same-as", metavar="OTHER")
    ap.add_argument("--symbols", metavar="OUT_TSV")
    ap.add_argument("--libs", action="store_true")
    ap.add_argument("--read", nargs=2, metavar=("ADDR", "N"))
    a = ap.parse_args()
    img = Image(a.image)
    if a.same_as:
        other = Image(a.same_as)
        ok = True
        for s in other.segments:
            mine = img.read(s.vaddr, len(s.data))
            diff = sum(x != y for x, y in zip(mine or b"", s.data)) if mine else len(s.data)
            ok &= diff == 0
            print(f"  {s.name:8} {s.vaddr:08X} {len(s.data):8X}  {'same' if not diff else f'{diff} bytes differ'}")
        print("IDENTICAL" if ok else "DIFFERENT")
        raise SystemExit(0 if ok else 1)
    if a.symbols:
        with open(a.symbols, "w", encoding="utf-8") as f:
            f.write("addr\tsize\tbind\ttype\tsection\tname\tdemangled\n")
            for s in img.symbols:
                f.write(f"{s.addr:08x}\t{s.size}\t{s.bind}\t{s.type}\t{s.section}\t{s.name}\t{cw.demangle(s.name)}\n")
        print(f"{len(img.symbols)} symbols -> {a.symbols}")
        return
    if a.libs:
        total = sum(sz for _, sz in census(img).values())
        for lib, (n, sz) in census(img).items():
            print(f"  {lib:26} {n:6} functions  {sz / 1024:7.0f} KB  {100 * sz / total:5.1f}%")
        return
    if a.read:
        addr, n = int(a.read[0], 0), int(a.read[1], 0)
        b = img.read(addr, n) or b""
        for i in range(0, len(b), 16):
            print(f"{addr + i:08X}  {b[i:i + 16].hex(' ')}")
        return
    print(f"{img.kind}  entry {img.entry:08X}  bss {img.bss:08X}+{img.bss_size:X}")
    for s in img.segments:
        print(f"  {s.name:12} {s.vaddr:08X}-{s.end:08X}  {'text' if s.text else 'data'}")
    if img.symbols:
        funcs = img.functions()
        print(f"  {len(img.symbols)} symbols, {len(funcs)} sized functions"
              f"  _SDA_BASE_ {img.sda or 0:08X}  _SDA2_BASE_ {img.sda2 or 0:08X}")


if __name__ == "__main__":
    main()
