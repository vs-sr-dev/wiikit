"""WIA and RVZ disc images (Dolphin's compressed formats), read-only.

    python -m wiikit.rvz GAME.rvz            # header, tables, compression
    python -m wiikit.disc GAME.rvz --info    # everything in disc.py works on them

Layout (big-endian throughout), as Dolphin writes it (docs/WiaAndRvz.md in
Dolphin's source):

file head, 0x48 bytes
      'WIA\\x01' or 'RVZ\\x01', u32 version, u32 compatible version,
      u32 size of the disc struct, sha1, u64 ISO size (0x24), u64 file size,
      sha1.
disc struct, at 0x48
      u32 disc type (1 GameCube, 2 Wii), u32 compression (0 none, 1 purge,
      2 bzip2, 3 LZMA, 4 LZMA2, 5 zstd), s32 level, u32 chunk size (0x10);
      the disc header's first 0x80 bytes (0x10); u32 partition count, u32
      partition entry size, u64 partition table offset (0x90); sha1;
      u32 raw-data count, u64 offset, u32 compressed size (0xB4);
      u32 group count, u64 offset, u32 compressed size (0xC4);
      u8 compressor-data length, 7 bytes of it (LZMA properties).
partitions, stored as is
      16-byte title key, then two {u32 first sector, u32 sector count,
      u32 first group, u32 group count}: the partition's data area, in
      0x8000-byte disc sectors, cut in two. Its groups hold the *decrypted*
      payload, 0x7C00 bytes a sector, hashes left out (recomputed on
      conversion back, with a list of exceptions where they differ).
raw data, compressed
      {u64 disc offset, u64 size, u32 first group, u32 group count}: every
      disc byte outside the partitions' data areas (headers, partition
      tables, tickets and TMDs, unused space). The offset is taken down to a
      0x8000 boundary before it is cut into groups.
groups, compressed
      WIA {u32 offset >> 2, u32 size}; RVZ {u32 offset >> 2, u32 size (top
      bit: the group is compressed), u32 packed size}. A size of 0 reads as
      zeros. A group covers one chunk: chunk-size bytes of raw data, or
      chunk-size / 0x8000 sectors of partition payload.

A partition group begins with its hash exceptions: one list per 2 MiB of
chunk (at least one), each u16 n then n × {u16 offset, sha1}; stored
uncompressed they are padded to 4 bytes. RVZ then "packs" the data: runs of
{u32 size, bytes}, where a size with the top bit set is junk (the
pseudo-random filler Nintendo's mastering writes) given by a 68-byte seed
for the lagged Fibonacci generator below, not stored.

Only reading is needed: raw data comes back as disc bytes, partition data as
decrypted payload, which is all disc.Partition needs. The encrypted form of
a partition's data area is not rebuilt.
"""
import argparse
import bz2
import lzma
import struct

try:                                   # Python 3.14+
    from compression import zstd as _zstd
except ImportError:                    # pragma: no cover
    _zstd = None

SECTOR, PAYLOAD = 0x8000, 0x7C00
COMPRESSION = {0: "none", 1: "purge", 2: "bzip2", 3: "LZMA", 4: "LZMA2", 5: "zstd"}


class LaggedFibonacci:
    """The junk generator of Wii and GameCube discs (Dolphin's
    LaggedFibonacciGenerator): k = 521, j = 32, seeded with 17 words."""

    K, J, SEED = 521, 32, 17

    def __init__(self, seed_bytes):
        b = list(struct.unpack(">17I", seed_bytes)) + [0] * (self.K - self.SEED)
        for i in range(self.SEED, self.K):
            b[i] = ((b[i - 17] << 23) ^ (b[i - 16] >> 9) ^ b[i - 1]) & 0xFFFFFFFF
        # the output takes bits 18-25 of each word for its third byte: done once here
        self.b = [(x & 0xFF00FFFF) | ((x >> 2) & 0x00FF0000) for x in b]
        self.pos = 0
        for _ in range(4):
            self._forward()
        self._bytes = None

    def _forward(self):
        b, K, J = self.b, self.K, self.J
        for i in range(J):
            b[i] ^= b[i + K - J]
        for i in range(J, K):
            b[i] ^= b[i - J]
        self._bytes = None

    def _block(self):
        if self._bytes is None:
            self._bytes = struct.pack(f">{self.K}I", *self.b)
        return self._bytes

    def skip(self, n):
        self.pos += n
        while self.pos >= 4 * self.K:
            self._forward()
            self.pos -= 4 * self.K

    def read(self, n):
        out = bytearray()
        while n > 0:
            take = min(n, 4 * self.K - self.pos)
            out += self._block()[self.pos:self.pos + take]
            self.pos += take
            n -= take
            if self.pos == 4 * self.K:
                self._forward()
                self.pos = 0
        return bytes(out)


class Rvz:
    """A .wia or .rvz file. read()/seek() give raw disc bytes outside the
    partitions' data areas; payload(partition offset) gives a reader of a
    partition's decrypted payload."""

    def __init__(self, path):
        self.f = open(path, "rb")
        h = self.f.read(0x48)
        self.kind = h[:3].decode()
        if h[:4] not in (b"WIA\x01", b"RVZ\x01"):
            raise ValueError(f"{path}: not a WIA or RVZ file")
        self.rvz = self.kind == "RVZ"
        dsize = struct.unpack_from(">I", h, 0x0C)[0]
        self.size = struct.unpack_from(">Q", h, 0x24)[0]
        d = self.f.read(dsize)
        self.disc_type, self.compression, self.level, self.chunk = struct.unpack_from(">IIiI", d, 0)
        self.dhead = d[0x10:0x90]
        npart, psize, poff = struct.unpack_from(">IIQ", d, 0x90)
        nraw, rawoff, rawsize = struct.unpack_from(">IQI", d, 0xB4)
        ngroup, groupoff, groupsize = struct.unpack_from(">IQI", d, 0xC4)
        self.props = d[0xD5:0xD5 + d[0xD4]]
        if self.compression == 5 and _zstd is None:
            raise SystemExit("RVZ with zstd needs Python 3.14 (compression.zstd)")
        self.f.seek(poff)
        pt = self.f.read(npart * psize)
        self.parts = []                        # (key, [(first sector, count, first group, groups)])
        for i in range(npart):
            e = pt[i * psize:(i + 1) * psize]
            self.parts.append((e[:16], [struct.unpack_from(">IIII", e, 16 + 16 * k) for k in range(2)]))
        self.f.seek(rawoff)
        rt = self._decompress(self.f.read(rawsize))
        self.raw = []                          # (aligned offset, end, first group, groups)
        for i in range(nraw):
            off, size, g, n = struct.unpack_from(">QQII", rt, 24 * i)
            skip = off % SECTOR
            self.raw.append((off - skip, off + size, g, n))
        self.f.seek(groupoff)
        gt = self._decompress(self.f.read(groupsize))
        step = 12 if self.rvz else 8
        self.groups = []
        for i in range(ngroup):
            if self.rvz:
                off, size, packed = struct.unpack_from(">III", gt, i * step)
            else:
                (off, size), packed = struct.unpack_from(">II", gt, i * step), 0
            self.groups.append((off << 2, size, packed))
        self.pos = 0
        self._cache = {}

    # --- decompression ------------------------------------------------------

    def _decompress(self, data):
        c = self.compression
        if c in (0, 1):
            return data
        if c == 2:
            return bz2.decompress(data)
        if c in (3, 4):
            if c == 3:
                filt = lzma._decode_filter_properties(lzma.FILTER_LZMA1, self.props)
            else:
                filt = lzma._decode_filter_properties(lzma.FILTER_LZMA2, self.props)
            return lzma.LZMADecompressor(lzma.FORMAT_RAW, filters=[filt]).decompress(data)
        if c == 5:
            return _zstd.decompress(data)
        raise ValueError(f"compression {c}")

    def _group(self, index, data_offset, sectors):
        """One group's data: bytes of raw disc (sectors = 0) or payload."""
        key = index
        if key in self._cache:
            return self._cache[key]
        off, size, packed = self.groups[index]
        want = sectors * PAYLOAD if sectors else self.chunk
        if size & 0x7FFFFFFF == 0:
            out = bytes(want)
        else:
            compressed = bool(size >> 31) if self.rvz else self.compression > 1
            self.f.seek(off)
            data = self.f.read(size & 0x7FFFFFFF)
            if compressed:
                data = self._decompress(data)
            p = 0
            if sectors:                                  # hash exceptions
                for _ in range(max(1, self.chunk // (SECTOR * 64))):
                    n = struct.unpack_from(">H", data, p)[0]
                    p += 2 + 22 * n
                if not compressed:
                    p = (p + 3) & ~3
            data = data[p:]
            out = self._unpack(data, data_offset) if packed else data
        if len(self._cache) > 64:
            self._cache.clear()
        self._cache[key] = out
        return out

    def _unpack(self, data, data_offset):
        out = bytearray()
        p = 0
        while p < len(data):
            size = struct.unpack_from(">I", data, p)[0]
            p += 4
            if size >> 31:
                size &= 0x7FFFFFFF
                lfg = LaggedFibonacci(data[p:p + 68])
                p += 68
                lfg.skip(data_offset % SECTOR)
                out += lfg.read(size)
            else:
                out += data[p:p + size]
                p += size
            data_offset += size
        return bytes(out)

    # --- raw disc bytes -----------------------------------------------------

    def seek(self, off, whence=0):
        self.pos = (0, self.pos, self.size)[whence] + off
        return self.pos

    def tell(self):
        return self.pos

    def read(self, size=-1):
        if size < 0:
            size = self.size - self.pos
        out = bytearray()
        while size > 0 and self.pos < self.size:
            n = self._read_raw(self.pos, size, out)
            self.pos += n
            size -= n
        return bytes(out)

    def _read_raw(self, pos, size, out):
        if pos < 0x80:
            n = min(size, 0x80 - pos)
            out += self.dhead[pos:pos + n]
            return n
        for start, end, g, ng in self.raw:
            if start <= pos < end:
                i, within = divmod(pos - start, self.chunk)
                blk = self._group(g + i, pos - within, 0)
                n = min(size, end - pos, self.chunk - within)
                out += blk[within:within + n].ljust(n, b"\0")
                return n
        for key, pds in self.parts:
            for first, count, g, ng in pds:
                if first * SECTOR <= pos < (first + count) * SECTOR:
                    raise ValueError(f"{pos:#x}: inside a partition's encrypted data "
                                     "area; read it through Partition")
        # not covered by anything: read as zeros up to the next region
        nxt = min([s for s, _, _, _ in self.raw if s > pos] +
                  [pds[0][0] * SECTOR for _, pds in self.parts if pds[0][0] * SECTOR > pos] +
                  [self.size])
        n = min(size, nxt - pos)
        out += bytes(n)
        return n

    # --- partition payload --------------------------------------------------

    def payload(self, part_offset, data_offset):
        """A reader (off, size) -> bytes of the decrypted payload of the
        partition whose data area starts at disc offset part_offset +
        data_offset, or None if this file has no such partition."""
        start = (part_offset + data_offset) // SECTOR
        for key, pds in self.parts:
            if pds[0][0] == start:
                return lambda off, size, pds=pds: self._read_payload(start, pds, off, size)
        return None

    def _read_payload(self, start, pds, off, size):
        out = bytearray()
        per = self.chunk // SECTOR                       # sectors a group
        while size > 0:
            sector, within = divmod(off, PAYLOAD)
            sector += start
            for first, count, g, ng in pds:
                if first <= sector < first + count:
                    gi, gs = divmod(sector - first, per)
                    sectors = min(per, first + count - (first + gi * per))
                    base = (first + gi * per - start) * PAYLOAD
                    blk = self._group(g + gi, base, sectors)
                    at = gs * PAYLOAD + within
                    n = min(size, sectors * PAYLOAD - at)
                    out += blk[at:at + n]
                    break
            else:
                break                                    # past the data area
            off += n
            size -= n
        return bytes(out)

    def close(self):
        self.f.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image")
    a = ap.parse_args()
    r = Rvz(a.image)
    print(f"{r.kind}  {r.dhead[:6].decode('ascii', 'replace')}  ISO {r.size} bytes  "
          f"{COMPRESSION.get(r.compression, r.compression)} level {r.level}  chunk {r.chunk:#x}  "
          f"{len(r.groups)} groups")
    for i, (key, pds) in enumerate(r.parts):
        print(f"  partition {i}: " + ", ".join(
            f"sectors {f * SECTOR:#x}+{c} groups {g}+{n}" for f, c, g, n in pds))
    for s, e, g, n in r.raw:
        print(f"  raw {s:#011x}-{e:#011x} groups {g}+{n}")


if __name__ == "__main__":
    main()
