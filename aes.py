"""AES-128-CBC decryption, pure Python.

Wii discs encrypt every partition cluster with AES-128-CBC, and the title key
itself is AES-encrypted with the console's common key. Only decryption is
needed to read a disc, so only decryption is here.

The cipher is the table-driven form of FIPS-197 (four 256-entry T-tables for
the inner rounds, the inverse S-box for the last), with the decryption key
schedule of the "equivalent inverse cipher". All tables are computed at import
from the field arithmetic; nothing is pasted in.

Pure Python runs at roughly half a megabyte per second. When pycryptodome is
installed, cbc_decrypt() hands the work to it instead; the result is the same
and `python -m wiikit.aes` checks that it is.
"""
import struct


def _gmul(a, b):
    r = 0
    while b:
        if b & 1:
            r ^= a
        a = ((a << 1) ^ 0x11B) if a & 0x80 else a << 1
        b >>= 1
    return r


def _tables():
    inv = [0] * 256
    for a in range(1, 256):
        for b in range(1, 256):
            if _gmul(a, b) == 1:
                inv[a] = b
                break
    sbox = [0] * 256
    for x in range(256):
        b = inv[x]
        s = b
        for k in range(1, 5):
            s ^= ((b << k) | (b >> (8 - k))) & 0xFF
        sbox[x] = s ^ 0x63
    isbox = [0] * 256
    for x in range(256):
        isbox[sbox[x]] = x
    td0 = []
    for x in range(256):
        s = isbox[x]
        td0.append((_gmul(s, 14) << 24) | (_gmul(s, 9) << 16) | (_gmul(s, 13) << 8) | _gmul(s, 11))
    ror = lambda v, n: ((v >> n) | (v << (32 - n))) & 0xFFFFFFFF
    td1 = [ror(v, 8) for v in td0]
    td2 = [ror(v, 16) for v in td0]
    td3 = [ror(v, 24) for v in td0]
    return sbox, isbox, td0, td1, td2, td3


SBOX, ISBOX, TD0, TD1, TD2, TD3 = _tables()


def _decrypt_key(key):
    """Round keys for the equivalent inverse cipher, 44 words, first round first."""
    w = list(struct.unpack(">4I", key))
    rcon = 1
    for i in range(4, 44):
        t = w[i - 1]
        if i % 4 == 0:
            t = ((t << 8) | (t >> 24)) & 0xFFFFFFFF
            t = (SBOX[t >> 24] << 24) | (SBOX[(t >> 16) & 255] << 16) | \
                (SBOX[(t >> 8) & 255] << 8) | SBOX[t & 255]
            t ^= rcon << 24
            rcon = _gmul(rcon, 2)
        w.append(w[i - 4] ^ t)
    dk = []
    for r in range(10, -1, -1):
        rk = w[4 * r:4 * r + 4]
        if 0 < r < 10:     # InvMixColumns on the inner round keys
            rk = [TD0[SBOX[v >> 24]] ^ TD1[SBOX[(v >> 16) & 255]] ^
                  TD2[SBOX[(v >> 8) & 255]] ^ TD3[SBOX[v & 255]] for v in rk]
        dk += rk
    return dk


def _cbc_decrypt_py(key, iv, data):
    dk = _decrypt_key(key)
    td0, td1, td2, td3, isb = TD0, TD1, TD2, TD3, ISBOX
    out = bytearray(len(data))
    p0, p1, p2, p3 = struct.unpack(">4I", iv)
    unpack, pack_into = struct.unpack_from, struct.pack_into
    for off in range(0, len(data) - 15, 16):
        c0, c1, c2, c3 = unpack(">4I", data, off)
        s0, s1, s2, s3 = c0 ^ dk[0], c1 ^ dk[1], c2 ^ dk[2], c3 ^ dk[3]
        k = 4
        for _ in range(9):
            t0 = td0[s0 >> 24] ^ td1[(s3 >> 16) & 255] ^ td2[(s2 >> 8) & 255] ^ td3[s1 & 255] ^ dk[k]
            t1 = td0[s1 >> 24] ^ td1[(s0 >> 16) & 255] ^ td2[(s3 >> 8) & 255] ^ td3[s2 & 255] ^ dk[k + 1]
            t2 = td0[s2 >> 24] ^ td1[(s1 >> 16) & 255] ^ td2[(s0 >> 8) & 255] ^ td3[s3 & 255] ^ dk[k + 2]
            t3 = td0[s3 >> 24] ^ td1[(s2 >> 16) & 255] ^ td2[(s1 >> 8) & 255] ^ td3[s0 & 255] ^ dk[k + 3]
            s0, s1, s2, s3 = t0, t1, t2, t3
            k += 4
        o0 = (isb[s0 >> 24] << 24 | isb[(s3 >> 16) & 255] << 16 | isb[(s2 >> 8) & 255] << 8 | isb[s1 & 255]) ^ dk[40]
        o1 = (isb[s1 >> 24] << 24 | isb[(s0 >> 16) & 255] << 16 | isb[(s3 >> 8) & 255] << 8 | isb[s2 & 255]) ^ dk[41]
        o2 = (isb[s2 >> 24] << 24 | isb[(s1 >> 16) & 255] << 16 | isb[(s0 >> 8) & 255] << 8 | isb[s3 & 255]) ^ dk[42]
        o3 = (isb[s3 >> 24] << 24 | isb[(s2 >> 16) & 255] << 16 | isb[(s1 >> 8) & 255] << 8 | isb[s0 & 255]) ^ dk[43]
        pack_into(">4I", out, off, o0 ^ p0, o1 ^ p1, o2 ^ p2, o3 ^ p3)
        p0, p1, p2, p3 = c0, c1, c2, c3
    return bytes(out)


try:                                   # optional accelerator, same result
    from Crypto.Cipher import AES as _AES
except ImportError:                    # pragma: no cover
    _AES = None


def cbc_decrypt(key, iv, data, pure=False):
    """AES-128-CBC decrypt `data` (a multiple of 16 bytes)."""
    if _AES is not None and not pure:
        return _AES.new(key, _AES.MODE_CBC, iv).decrypt(data)
    return _cbc_decrypt_py(key, iv, data)


if __name__ == "__main__":
    import os
    import time
    # FIPS-197 appendix C.1 vector (ECB block = CBC with a zero IV)
    key = bytes(range(16))
    ct = bytes.fromhex("69c4e0d86a7b0430d8cdb78070b4c55a")
    pt = _cbc_decrypt_py(key, bytes(16), ct)
    assert pt == bytes.fromhex("00112233445566778899aabbccddeeff"), pt.hex()
    print("FIPS-197 C.1: ok")
    blob, iv = os.urandom(0x7C00), os.urandom(16)
    t = time.time()
    a = _cbc_decrypt_py(key, iv, blob)
    dt = time.time() - t
    print(f"pure Python: {len(blob) / dt / 1e6:.2f} MB/s")
    if _AES is not None:
        assert a == _AES.new(key, _AES.MODE_CBC, iv).decrypt(blob)
        print("matches pycryptodome on a 31 KiB cluster payload")
