"""Nintendo DSP-ADPCM: the standard header and the decoder.

    from wiikit import dsp
    h = dsp.header(buf, off)                       # the 0x60-byte DSP header
    pcm = dsp.decode(buf[off + 0x60:], h["coefs"], h["samples"], h["hist1"], h["hist2"])
    dsp.write_wav("out.wav", [pcm], h["rate"])

The DSP header (big-endian, 0x60 bytes) as the SDK's dsptool writes it:
    0x00 u32 sample count      0x04 u32 nibble count     0x08 u32 sample rate
    0x0C u16 loop flag         0x0E u16 format (0)       0x10 u32 loop start nibble
    0x14 u32 loop end nibble   0x18 u32 current address
    0x1C s16[16] coefficients (8 pairs)
    0x3C u16 gain  0x3E u16 initial predictor/scale  0x40 s16 hist1  0x42 s16 hist2
    0x44 u16 loop predictor/scale  0x46 s16 loop hist1  0x48 s16 loop hist2

Frames are 8 bytes: a predictor/scale byte (high nibble: coefficient pair,
low nibble: log2 scale) and 14 signed 4-bit samples. Containers differ only
in how they wrap and interleave these frames; the containers belong in the
game's own tools.
"""
import struct
import wave


def _clamp(v):
    return -32768 if v < -32768 else 32767 if v > 32767 else v


def header(buf, off=0):
    samples, nibbles, rate = struct.unpack_from(">III", buf, off)
    loop, fmt, loop_start, loop_end, cur = struct.unpack_from(">HHIII", buf, off + 0x0C)
    coefs = struct.unpack_from(">16h", buf, off + 0x1C)
    gain, ps, h1, h2 = struct.unpack_from(">HHhh", buf, off + 0x3C)
    return dict(samples=samples, nibbles=nibbles, rate=rate, loop=loop, format=fmt,
                loop_start=loop_start, loop_end=loop_end, coefs=coefs, ps=ps,
                hist1=h1, hist2=h2)


def decode(data, coefs, nsamples, hist1=0, hist2=0):
    """Decode one channel of raw DSP-ADPCM frames to a list of s16 samples."""
    out = []
    p = 0
    while len(out) < nsamples and p < len(data):
        head = data[p]
        p += 1
        scale = 1 << (head & 0xF)
        c1, c2 = coefs[(head >> 4 & 7) * 2], coefs[(head >> 4 & 7) * 2 + 1]
        for _ in range(7):
            if p >= len(data) or len(out) >= nsamples:
                break
            byte = data[p]
            p += 1
            for nib in (byte >> 4, byte & 0xF):
                if len(out) >= nsamples:
                    break
                s = nib - 16 if nib >= 8 else nib
                v = _clamp(((s * scale) << 11) + c1 * hist1 + c2 * hist2 + 1024 >> 11)
                out.append(v)
                hist2, hist1 = hist1, v
    return out


def write_wav(path, channels, rate):
    n = min(len(c) for c in channels)
    frames = bytearray()
    for i in range(n):
        for c in channels:
            frames += struct.pack("<h", c[i])
    with wave.open(path, "wb") as w:
        w.setnchannels(len(channels))
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(bytes(frames))
