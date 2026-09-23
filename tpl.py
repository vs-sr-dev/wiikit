"""TPL texture palettes (GameCube / Wii) to PNG.

    python -m wiikit.tpl homeBtnIcon.tpl out/icon

TPL wraps GX image data (see gxtex) in a small container, big-endian, all
offsets from the start of the file:

    0x00  u32 magic 0x0020AF30
    0x04  u32 image count
    0x08  u32 image-table offset (usually 0x0C)
    table: per image, u32 image-header offset, u32 palette-header offset (0 = none)
    image header: u16 height, u16 width, u32 GX format, u32 data offset, then
                  wrap / filter / LOD fields
"""
import struct
import sys

from . import gxtex

GX_NAME = {0: "I4", 1: "I8", 2: "IA4", 3: "IA8", 4: "RGB565", 5: "RGB5A3",
           6: "RGBA8", 8: "C4", 9: "C8", 10: "C14X2", 0xE: "CMPR"}


def images(buf):
    """Yield (index, width, height, format, palette header offset, data offset)."""
    magic, count, tbl = struct.unpack_from(">III", buf, 0)
    if magic != 0x0020AF30:
        raise ValueError(f"bad TPL magic {magic:08x}")
    for i in range(count):
        img_off, pal_off = struct.unpack_from(">II", buf, tbl + i * 8)
        h, w, fmt, data_off = struct.unpack_from(">HHII", buf, img_off)
        yield i, w, h, fmt, pal_off, data_off


def to_png(path, prefix, log=print):
    buf = open(path, "rb").read()
    n = 0
    for i, w, h, fmt, pal_off, data_off in images(buf):
        name = GX_NAME.get(fmt, f"fmt{fmt}")
        if fmt not in gxtex.BLOCK:
            log(f"  [{i}] {w}x{h} {name}: palette formats not handled yet")
            continue
        out = f"{prefix}_{i}.png"
        gxtex.save_png(out, w, h, gxtex.decode(fmt, w, h, buf[data_off:]))
        log(f"  [{i}] {w}x{h} {name} -> {out}")
        n += 1
    return n


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    print(f"{to_png(sys.argv[1], sys.argv[2])} image(s)")
