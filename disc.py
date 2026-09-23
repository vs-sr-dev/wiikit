"""Wii disc images: .iso and .wbfs, partitions, decryption, file system.

    python -m wiikit.disc GAME.wbfs --info
    python -m wiikit.disc GAME.wbfs --list [--part DATA]
    python -m wiikit.disc GAME.wbfs --extract out/ [--part DATA]

Layers, from the outside in:

WBFS  A sparse container written by USB loaders. The disc is cut into "WBFS
      sectors" (2 MiB on the discs seen so far) and a u16 table maps each disc
      sector to a sector of the file; 0 means not stored (unused disc space,
      reads as zeros). Header (big-endian): 'WBFS', u32 hd-sector count,
      u8 log2 hd-sector size, u8 log2 WBFS-sector size, then one byte per disc
      slot. Slot n's info sits at hd_sector * (1 + n): a 0x100-byte copy of the
      disc header followed by the u16 table. open_disc() turns either format
      into one seek/read/tell object, so nothing further down cares.

Disc  0x000 game id (6 chars), 0x018 Wii magic 0x5D1C9EA3, 0x020 title.
      0x40000: four partition groups (u32 count, u32 table offset >> 2); each
      table entry is (u32 offset >> 2, u32 type), type 0 = DATA, 1 = UPDATE,
      2 = CHANNEL.

Partition
      Starts with its ticket: the encrypted title key at 0x1BF, the title id
      at 0x1DC (IV = title id + 8 zero bytes), the common-key index at 0x1F1.
      0x2A4 / 0x2A8: TMD size and offset (>> 2); the TMD names the IOS the
      game runs on (u64 at 0x184).
      0x2B8 / 0x2BC: data offset and size (>> 2). The data area is a run of
      0x8000-byte clusters, each 0x400 bytes of hash tables followed by 0x7C00
      bytes of payload; the payload IV is bytes 0x3D0-0x3DF of the still
      encrypted hash area. Offsets inside a partition are in payload bytes.

FST   Partition offset 0x424 (>> 2), size 0x428 (>> 2). 12-byte entries:
      u8 is-dir, u24 name offset, then file (offset >> 2, size) or directory
      (parent index, index past the last child). Entry 0 is the root and its
      size is the entry count; the name table follows the entries.
"""
import argparse
import os
import struct

from . import aes

WII_MAGIC = 0x5D1C9EA3
WII_SECTOR = 0x8000
WII_SECTORS_DL = 143432 * 2          # dual-layer disc, in 0x8000 sectors
CLUSTER, HASH_AREA, PAYLOAD = 0x8000, 0x400, 0x7C00
PART_TYPES = {0: "DATA", 1: "UPDATE", 2: "CHANNEL"}

# Public Wii common keys (0 retail, 1 Korean, 2 vWii). Needed to read any disc.
COMMON_KEYS = {
    0: bytes.fromhex("ebe42a225e8593e448d9c5457381aaf7"),
    1: bytes.fromhex("63b82bb4f4614e2e13f2fefbba4c9b7e"),
    2: bytes.fromhex("30bfc76e7c19afbb23163330ced7c28d"),
}


def be32(b, o):
    return struct.unpack_from(">I", b, o)[0]


class Wbfs:
    """A .wbfs file seen as the disc it contains (read-only, file-like)."""

    def __init__(self, path, slot=0):
        self.f = open(path, "rb")
        h = self.f.read(0x100)
        if h[:4] != b"WBFS":
            raise ValueError(f"{path}: not a WBFS file")
        self.hd_sec = 1 << h[8]
        self.sec = 1 << h[9]
        if not h[0xC + slot]:
            raise ValueError(f"{path}: WBFS slot {slot} is empty")
        n = WII_SECTORS_DL // (self.sec // WII_SECTOR)
        self.f.seek(self.hd_sec * (1 + slot) + 0x100)
        self.wlba = struct.unpack(f">{n}H", self.f.read(2 * n))
        self.size = n * self.sec
        self.pos = 0

    def seek(self, off, whence=0):
        base = (0, self.pos, self.size)[whence]
        self.pos = base + off
        return self.pos

    def tell(self):
        return self.pos

    def read(self, size=-1):
        if size < 0:
            size = self.size - self.pos
        out = bytearray()
        while size > 0 and self.pos < self.size:
            sec, within = divmod(self.pos, self.sec)
            n = min(size, self.sec - within)
            phys = self.wlba[sec]
            if phys:
                self.f.seek(phys * self.sec + within)
                out += self.f.read(n)
            else:
                out += bytes(n)
            self.pos += n
            size -= n
        return bytes(out)

    def stored_sectors(self):
        return sum(1 for x in self.wlba if x)

    def close(self):
        self.f.close()


def open_disc(path):
    with open(path, "rb") as f:
        magic = f.read(4)
    return Wbfs(path) if magic == b"WBFS" else open(path, "rb")


def header(disc):
    disc.seek(0)
    h = disc.read(0x100)
    return {
        "game_id": h[:6].decode("ascii", "replace"),
        "disc_no": h[6], "version": h[7],
        "wii": be32(h, 0x18) == WII_MAGIC,
        "title": h[0x20:0x60].split(b"\0")[0].decode("latin1"),
    }


def partitions(disc):
    """[(absolute offset, type name)] in table order."""
    disc.seek(0x40000)
    grp = disc.read(0x20)
    out = []
    for g in range(4):
        count, tbl = be32(grp, g * 8), be32(grp, g * 8 + 4) << 2
        if not count:
            continue
        disc.seek(tbl)
        t = disc.read(count * 8)
        for i in range(count):
            typ = be32(t, i * 8 + 4)
            out.append((be32(t, i * 8) << 2, PART_TYPES.get(typ, str(typ))))
    return out


class Partition:
    """Random access to the decrypted payload of one partition."""

    def __init__(self, disc, offset, pure=False):
        self.disc, self.base, self.pure = disc, offset, pure
        disc.seek(offset)
        t = disc.read(0x2C0)
        iv = t[0x1DC:0x1E4] + bytes(8)
        ck = COMMON_KEYS.get(t[0x1F1], COMMON_KEYS[0])
        self.title_id = t[0x1DC:0x1E4]
        self.title_key = aes.cbc_decrypt(ck, iv, t[0x1BF:0x1CF], pure)
        self.data_off = be32(t, 0x2B8) << 2
        self.data_size = be32(t, 0x2BC) << 2
        self.ticket = t[:0x2A4]
        disc.seek(offset + (be32(t, 0x2A8) << 2))
        self.tmd = disc.read(be32(t, 0x2A4))
        self._idx, self._buf = -1, b""

    def _cluster(self, idx):
        if idx != self._idx:
            self.disc.seek(self.base + self.data_off + idx * CLUSTER)
            raw = self.disc.read(CLUSTER)
            self._buf = aes.cbc_decrypt(self.title_key, raw[0x3D0:0x3E0],
                                        raw[HASH_AREA:], self.pure)
            self._idx = idx
        return self._buf

    def read(self, off, size):
        out = bytearray()
        while size > 0:
            idx, within = divmod(off, PAYLOAD)
            chunk = self._cluster(idx)[within:within + size]
            if not chunk:
                break
            out += chunk
            off += len(chunk)
            size -= len(chunk)
        return bytes(out)

    # --- the partition's own header and file system -------------------------

    def boot(self):
        b = self.read(0, 0x440)
        return {"dol": be32(b, 0x420) << 2, "fst": be32(b, 0x424) << 2,
                "fst_size": be32(b, 0x428) << 2, "raw": b}

    def dol_size(self, dol_off):
        """Size of the DOL from its own section table."""
        h = self.read(dol_off, 0x100)
        offs = struct.unpack_from(">18I", h, 0)
        sizes = struct.unpack_from(">18I", h, 0x90)
        return max(o + s for o, s in zip(offs, sizes) if s)

    def files(self):
        """Yield (path, offset, size) for every file in the FST."""
        b = self.boot()
        yield from walk_fst(self.read(b["fst"], b["fst_size"]))


def walk_fst(fst):
    total = be32(fst, 8)
    names = total * 12

    def name(off):
        return fst[names + off:fst.index(b"\0", names + off)].decode("latin1")

    stack = [(total, "")]
    for i in range(1, total):
        e = i * 12
        while i >= stack[-1][0]:
            stack.pop()
        nm = name(be32(fst, e) & 0xFFFFFF)
        a, b = be32(fst, e + 4), be32(fst, e + 8)
        if fst[e]:
            stack.append((b, stack[-1][1] + nm + "/"))
        else:
            yield stack[-1][1] + nm, a << 2, b


def open_partition(path, want="DATA", pure=False):
    disc = open_disc(path)
    parts = partitions(disc)
    for off, name in parts:
        if name == want.upper():
            return disc, Partition(disc, off, pure)
    if want.isdigit() and int(want) < len(parts):
        return disc, Partition(disc, parts[int(want)][0], pure)
    raise SystemExit(f"no {want} partition; have {[n for _, n in parts]}")


def extract(path, out, want="DATA", pure=False, log=print):
    """Write the partition as `wit extract` lays it out: sys/ and files/."""
    disc, p = open_partition(path, want, pure)
    b = p.boot()
    os.makedirs(os.path.join(out, "sys"), exist_ok=True)
    os.makedirs(os.path.join(out, "disc"), exist_ok=True)
    disc.seek(0)
    parts = {"disc/header.bin": disc.read(0x100),
             "ticket.bin": p.ticket, "tmd.bin": p.tmd,
             "sys/boot.bin": b["raw"],
             "sys/bi2.bin": p.read(0x440, 0x2000),
             "sys/fst.bin": p.read(b["fst"], b["fst_size"]),
             "sys/main.dol": p.read(b["dol"], p.dol_size(b["dol"]))}
    app = p.read(0x2440, 0x20)
    parts["sys/apploader.img"] = p.read(0x2440, 0x20 + be32(app, 0x14) + be32(app, 0x18))
    for rel, data in parts.items():
        with open(os.path.join(out, *rel.split("/")), "wb") as f:
            f.write(data)
    n = total = 0
    for rel, off, size in p.files():
        dst = os.path.join(out, "files", *rel.split("/"))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            while size > 0:
                blk = min(size, 0x3E0000)
                f.write(p.read(off, blk))
                off += blk
                size -= blk
                total += blk
        n += 1
        log(f"  {rel}")
    log(f"{n} files, {total / 1e6:.1f} MB -> {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("disc")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--extract", metavar="OUT")
    ap.add_argument("--part", default="DATA")
    ap.add_argument("--pure", action="store_true", help="never use pycryptodome")
    a = ap.parse_args()
    if a.extract:
        extract(a.disc, a.extract, a.part, a.pure)
        return
    disc = open_disc(a.disc)
    if a.list:
        _, p = open_partition(a.disc, a.part, a.pure)
        for rel, off, size in p.files():
            print(f"{off:10X} {size:10d}  {rel}")
        return
    h = header(disc)
    kind = "WBFS" if isinstance(disc, Wbfs) else "ISO"
    print(f"{kind}  {h['game_id']}  '{h['title']}'  disc {h['disc_no']} v{h['version']}"
          f"  {'Wii' if h['wii'] else 'not Wii'}")
    if isinstance(disc, Wbfs):
        print(f"  WBFS sector {disc.sec:#x}, {disc.stored_sectors()}/{len(disc.wlba)} stored")
    for off, name in partitions(disc):
        p = Partition(disc, off, a.pure)
        b = p.boot()
        print(f"  {name:8} @{off:#011x}  title {p.title_id.hex()}  data {p.data_size / 1e6:.0f} MB"
              f"  dol @{b['dol']:#x}  fst @{b['fst']:#x} ({b['fst_size']} B)")


if __name__ == "__main__":
    main()
