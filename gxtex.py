"""GX texture decoding (GameCube / Wii) to RGBA, and a minimal PNG writer.

    from wiikit import gxtex
    rgba = gxtex.decode(fmt, width, height, data)
    gxtex.save_png("out.png", width, height, rgba)

Formats: 0 I4, 1 I8, 2 IA4, 3 IA8, 4 RGB565, 5 RGB5A3, 6 RGBA8, 0xE CMPR
(palette formats 8 C4, 9 C8, 10 C14X2 are not handled yet). Pixels are stored
tile by tile in raster order, and in raster order inside each tile; each
format has its own tile size (BLOCK).

Two details differ from the obvious reading and are measured against the
hardware (textures dumped by Dolphin while the game runs, WII-TLS-RE):
* I4 and I8 put the intensity in alpha too, not 255;
* CMPR interpolates with weights 5/8 and 3/8, not 1/3 and 2/3.
"""
import struct

def _c3to8(v): return (v << 5) | (v << 2) | (v >> 1)
def _c4to8(v): return (v << 4) | v
def _c5to8(v): return (v << 3) | (v >> 2)
def _c6to8(v): return (v << 2) | (v >> 4)

def rgb565(v):
    r = _c5to8((v >> 11) & 0x1f); g = _c6to8((v >> 5) & 0x3f); b = _c5to8(v & 0x1f)
    return (r, g, b, 255)

def rgb5a3(v):
    if v & 0x8000:  # opaque RGB555
        r = _c5to8((v >> 10) & 0x1f); g = _c5to8((v >> 5) & 0x1f); b = _c5to8(v & 0x1f)
        return (r, g, b, 255)
    a = ((v >> 12) & 7); a = (a << 5) | (a << 2) | (a >> 1)
    r = _c4to8((v >> 8) & 0xf); g = _c4to8((v >> 4) & 0xf); b = _c4to8(v & 0xf)
    return (r, g, b, a)

# (block_w, block_h) per format
BLOCK = {0: (8, 8), 1: (8, 4), 2: (8, 4), 3: (4, 4), 4: (4, 4),
         5: (4, 4), 6: (4, 4), 0xE: (8, 8)}

def decode(fmt, w, h, data):
    """Return a w*h*4 RGBA bytearray."""
    out = bytearray(w * h * 4)
    def put(x, y, rgba):
        if x < w and y < h:
            i = (y * w + x) * 4
            out[i:i+4] = bytes(rgba)
    bw, bh = BLOCK[fmt]
    pos = 0
    for by in range(0, h, bh):
        for bx in range(0, w, bw):
            if fmt == 0:  # I4
                for ty in range(bh):
                    for tx in range(0, bw, 2):
                        b = data[pos]; pos += 1
                        for k, v in ((0, b >> 4), (1, b & 0xf)):
                            c = _c4to8(v); put(bx+tx+k, by+ty, (c, c, c, c))
            elif fmt == 1:  # I8
                for ty in range(bh):
                    for tx in range(bw):
                        c = data[pos]; pos += 1; put(bx+tx, by+ty, (c, c, c, c))
            elif fmt == 2:  # IA4
                for ty in range(bh):
                    for tx in range(bw):
                        b = data[pos]; pos += 1
                        a = _c4to8(b >> 4); c = _c4to8(b & 0xf)
                        put(bx+tx, by+ty, (c, c, c, a))
            elif fmt == 3:  # IA8
                for ty in range(bh):
                    for tx in range(bw):
                        a = data[pos]; c = data[pos+1]; pos += 2
                        put(bx+tx, by+ty, (c, c, c, a))
            elif fmt == 4:  # RGB565
                for ty in range(bh):
                    for tx in range(bw):
                        v = struct.unpack('>H', data[pos:pos+2])[0]; pos += 2
                        put(bx+tx, by+ty, rgb565(v))
            elif fmt == 5:  # RGB5A3
                for ty in range(bh):
                    for tx in range(bw):
                        v = struct.unpack('>H', data[pos:pos+2])[0]; pos += 2
                        put(bx+tx, by+ty, rgb5a3(v))
            elif fmt == 6:  # RGBA8: 4x4 blocks in two halves, AR then GB
                ar = data[pos:pos+32]; gb = data[pos+32:pos+64]; pos += 64
                for ty in range(4):
                    for tx in range(4):
                        j = (ty*4+tx)*2
                        a = ar[j]; r = ar[j+1]; g = gb[j]; b = gb[j+1]
                        put(bx+tx, by+ty, (r, g, b, a))
            elif fmt == 0xE:  # CMPR: DXT1-like, an 8x8 tile of four 4x4 sub-blocks
                for sy in range(0, 8, 4):
                    for sx in range(0, 8, 4):
                        c0 = struct.unpack('>H', data[pos:pos+2])[0]
                        c1 = struct.unpack('>H', data[pos+2:pos+4])[0]
                        bits = struct.unpack('>I', data[pos+4:pos+8])[0]; pos += 8
                        cols = []
                        r0, g0, b0, _ = rgb565(c0); r1, g1, b1, _ = rgb565(c1)
                        cols.append((r0, g0, b0, 255)); cols.append((r1, g1, b1, 255))
                        if c0 > c1:
                            # Not 1/3 and 2/3: the GX texture unit weights the two
                            # interpolated colours 5/8 and 3/8 (measured against
                            # textures dumped by Dolphin, WII-TLS-RE session 15)
                            cols.append(((5*r0+3*r1)>>3, (5*g0+3*g1)>>3, (5*b0+3*b1)>>3, 255))
                            cols.append(((3*r0+5*r1)>>3, (3*g0+5*g1)>>3, (3*b0+5*b1)>>3, 255))
                        else:
                            cols.append(((r0+r1)//2, (g0+g1)//2, (b0+b1)//2, 255))
                            # the fourth colour is transparent but not black:
                            # it keeps the average, with alpha 0
                            cols.append(((r0+r1)//2, (g0+g1)//2, (b0+b1)//2, 0))
                        for ty in range(4):
                            for tx in range(4):
                                idx = (bits >> (2*(15-(ty*4+tx)))) & 3
                                put(bx+sx+tx, by+sy+ty, cols[idx])
    return out

def save_png(path, w, h, rgba):
    import zlib
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += rgba[y*w*4:(y+1)*w*4]
    def chunk(typ, data):
        c = struct.pack('>I', len(data)) + typ + data
        return c + struct.pack('>I', zlib.crc32(typ + data) & 0xffffffff)
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0)
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) + \
          chunk(b'IDAT', zlib.compress(bytes(raw), 9)) + chunk(b'IEND', b'')
    open(path, 'wb').write(png)
