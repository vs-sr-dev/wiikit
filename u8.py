"""Nintendo U8 archives (.arc): list and extract.

    python -m wiikit.u8 homeBtn.arc
    python -m wiikit.u8 homeBtn.arc --extract out/

Layout (big-endian):
    0x00  u32 magic 0x55AA382D
    0x04  u32 root node offset (usually 0x20)
    0x08  u32 size of node table + string table
    0x0C  u32 data offset
    nodes, 12 bytes each: u8 type (0 file, 1 dir), u24 name offset,
        file: u32 data offset, u32 size
        dir:  u32 parent index, u32 index past the last child
    the first node is the root; its size field is the node count, and the
    string table follows the nodes.
"""
import argparse
import os
import struct

MAGIC = 0x55AA382D


def parse(buf, base=0):
    """[(path, absolute offset, size)] for every file in the archive at `base`."""
    magic, root_off, _, _ = struct.unpack_from(">IIII", buf, base)
    if magic != MAGIC:
        raise ValueError(f"bad U8 magic {magic:08x}")
    n0 = base + root_off
    total = struct.unpack_from(">I", buf, n0 + 8)[0]
    strings = n0 + total * 12

    def name(off):
        e = buf.index(b"\0", strings + off)
        return buf[strings + off:e].decode("latin1")

    out, stack = [], [(total, "")]
    for i in range(1, total):
        o = n0 + i * 12
        typ = buf[o]
        noff = struct.unpack_from(">I", buf, o)[0] & 0xFFFFFF
        a, b = struct.unpack_from(">II", buf, o + 4)
        while len(stack) > 1 and i >= stack[-1][0]:
            stack.pop()
        if typ == 1:
            stack.append((b, f"{stack[-1][1]}{name(noff)}/"))
        else:
            out.append((f"{stack[-1][1]}{name(noff)}", base + a, b))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("archive")
    ap.add_argument("--extract", metavar="OUT")
    a = ap.parse_args()
    buf = open(a.archive, "rb").read()
    files = parse(buf)
    for path, off, size in files:
        if a.extract:
            dst = os.path.join(a.extract, *path.split("/"))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            with open(dst, "wb") as f:
                f.write(buf[off:off + size])
        else:
            m = buf[off:off + 4]
            tag = m.decode("latin1") if all(32 <= c < 127 for c in m) else m.hex()
            print(f"  {size:9}  {tag:8}  {path}")
    print(f"{len(files)} files" + (f" -> {a.extract}" if a.extract else ""))


if __name__ == "__main__":
    main()
