"""Gekko / Broadway (PowerPC 750CL) decoding, disassembly and cross-references.

    python -m wiikit.ppc game.elf --func readController
    python -m wiikit.ppc game.elf --callers bGetShake__5CGameFi
    python -m wiikit.ppc game.elf --xref 0x8079ADA0
    python -m wiikit.ppc game.elf --mix              # instruction census
    python -m wiikit.ppc main.dol --at 0x80004000 --count 20

decode(word) returns an Ins with the *canonical* operation name ("addi",
"rlwinm", "bc", "psq_l", ...) and its raw fields; that is what a recompiler
consumes. text(ins, addr) renders it with the usual simplified mnemonics
(li, lis, mr, slwi, beq, blr, mflr, ...) for reading.

Covered: the whole 750CL user and supervisor set, and Gekko's paired singles
(opcode 4 plus psq_l/psq_lu/psq_st/psq_stu at 56/57/60/61). capstone does not
decode paired singles; this module has no dependencies.

Cross-references work the way CodeWarrior builds addresses: `lis rX, hi` then
`addi`/`ori`/a load or store off rX (tracked per register through a
function, cleared at calls), and small-data accesses off r13 (_SDA_BASE_) and
r2 (_SDA2_BASE_).
"""
import argparse
import collections
import struct

from . import cw
from .dol import Image


def _s16(v):
    return v - 0x10000 if v & 0x8000 else v


def _s12(v):
    return v - 0x1000 if v & 0x800 else v


class Ins:
    __slots__ = ("op", "f", "word")

    def __init__(self, op, word, **f):
        self.op, self.word, self.f = op, word, f

    def __repr__(self):
        return f"Ins({self.op} {self.f})"


# --- opcode tables --------------------------------------------------------------

_D_FORM = {7: "mulli", 8: "subfic", 12: "addic", 13: "addic.", 14: "addi", 15: "addis"}
_LOGIC_IMM = {24: "ori", 25: "oris", 26: "xori", 27: "xoris", 28: "andi.", 29: "andis."}
_LOADSTORE = {32: "lwz", 33: "lwzu", 34: "lbz", 35: "lbzu", 36: "stw", 37: "stwu",
              38: "stb", 39: "stbu", 40: "lhz", 41: "lhzu", 42: "lha", 43: "lhau",
              44: "sth", 45: "sthu", 46: "lmw", 47: "stmw", 48: "lfs", 49: "lfsu",
              50: "lfd", 51: "lfdu", 52: "stfs", 53: "stfsu", 54: "stfd", 55: "stfdu"}
_PSQ = {56: "psq_l", 57: "psq_lu", 60: "psq_st", 61: "psq_stu"}

# opcode 31, XO-form arithmetic (9-bit XO + OE)
_X31_XO = {8: "subfc", 10: "addc", 11: "mulhwu", 40: "subf", 75: "mulhw", 104: "neg",
           136: "subfe", 138: "adde", 200: "subfze", 202: "addze", 232: "subfme",
           234: "addme", 235: "mullw", 266: "add", 459: "divwu", 491: "divw"}
_X31_UNARY = {104, 200, 202, 232, 234}          # rD, rA
# opcode 31, X-form: name -> operand shape
_X31 = {
    0: ("cmp", "cmp"), 32: ("cmpl", "cmp"), 4: ("tw", "tw"),
    19: ("mfcr", "D"), 83: ("mfmsr", "D"), 146: ("mtmsr", "S"), 144: ("mtcrf", "mtcrf"),
    339: ("mfspr", "mfspr"), 467: ("mtspr", "mtspr"), 371: ("mftb", "mftb"),
    210: ("mtsr", "mtsr"), 242: ("mtsrin", "SB"), 595: ("mfsr", "mfsr"), 659: ("mfsrin", "DB"),
    512: ("mcrxr", "crfD"),
    24: ("slw", "ASB"), 26: ("cntlzw", "AS"), 28: ("and", "ASB"), 60: ("andc", "ASB"),
    124: ("nor", "ASB"), 284: ("eqv", "ASB"), 316: ("xor", "ASB"), 412: ("orc", "ASB"),
    444: ("or", "ASB"), 476: ("nand", "ASB"), 536: ("srw", "ASB"), 792: ("sraw", "ASB"),
    824: ("srawi", "srawi"), 922: ("extsh", "AS"), 954: ("extsb", "AS"),
    20: ("lwarx", "DAB"), 23: ("lwzx", "DAB"), 55: ("lwzux", "DAB"), 87: ("lbzx", "DAB"),
    119: ("lbzux", "DAB"), 279: ("lhzx", "DAB"), 311: ("lhzux", "DAB"), 343: ("lhax", "DAB"),
    375: ("lhaux", "DAB"), 534: ("lwbrx", "DAB"), 790: ("lhbrx", "DAB"), 533: ("lswx", "DAB"),
    597: ("lswi", "lswi"), 310: ("eciwx", "DAB"),
    150: ("stwcx.", "SAB"), 151: ("stwx", "SAB"), 183: ("stwux", "SAB"), 215: ("stbx", "SAB"),
    247: ("stbux", "SAB"), 407: ("sthx", "SAB"), 439: ("sthux", "SAB"), 662: ("stwbrx", "SAB"),
    918: ("sthbrx", "SAB"), 661: ("stswx", "SAB"), 725: ("stswi", "stswi"), 438: ("ecowx", "SAB"),
    535: ("lfsx", "fDAB"), 567: ("lfsux", "fDAB"), 599: ("lfdx", "fDAB"), 631: ("lfdux", "fDAB"),
    663: ("stfsx", "fDAB"), 695: ("stfsux", "fDAB"), 727: ("stfdx", "fDAB"),
    759: ("stfdux", "fDAB"), 983: ("stfiwx", "fDAB"),
    54: ("dcbst", "AB"), 86: ("dcbf", "AB"), 246: ("dcbtst", "AB"), 278: ("dcbt", "AB"),
    470: ("dcbi", "AB"), 982: ("icbi", "AB"), 1014: ("dcbz", "AB"), 306: ("tlbie", "B"),
    566: ("tlbsync", ""), 598: ("sync", ""), 854: ("eieio", ""),
}
_X19 = {0: ("mcrf", "mcrf"), 16: ("bclr", "bcr"), 528: ("bcctr", "bcr"), 50: ("rfi", ""),
        150: ("isync", ""), 33: ("crnor", "crb"), 129: ("crandc", "crb"), 193: ("crxor", "crb"),
        225: ("crnand", "crb"), 257: ("crand", "crb"), 289: ("creqv", "crb"),
        417: ("crorc", "crb"), 449: ("cror", "crb")}
# A-form float arithmetic: name -> operand order
_A59 = {18: ("fdivs", "DAB"), 20: ("fsubs", "DAB"), 21: ("fadds", "DAB"), 22: ("fsqrts", "DB"),
        24: ("fres", "DB"), 25: ("fmuls", "DAC"), 28: ("fmsubs", "DACB"), 29: ("fmadds", "DACB"),
        30: ("fnmsubs", "DACB"), 31: ("fnmadds", "DACB")}
_A63 = {18: ("fdiv", "DAB"), 20: ("fsub", "DAB"), 21: ("fadd", "DAB"), 22: ("fsqrt", "DB"),
        23: ("fsel", "DACB"), 25: ("fmul", "DAC"), 26: ("frsqrte", "DB"), 28: ("fmsub", "DACB"),
        29: ("fmadd", "DACB"), 30: ("fnmsub", "DACB"), 31: ("fnmadd", "DACB")}
_X63 = {0: ("fcmpu", "fcmp"), 32: ("fcmpo", "fcmp"), 12: ("frsp", "DB"), 14: ("fctiw", "DB"),
        15: ("fctiwz", "DB"), 40: ("fneg", "DB"), 72: ("fmr", "DB"), 136: ("fnabs", "DB"),
        264: ("fabs", "DB"), 38: ("mtfsb1", "crbD"), 70: ("mtfsb0", "crbD"),
        64: ("mcrfs", "mcrf"), 134: ("mtfsfi", "mtfsfi"), 583: ("mffs", "D"), 711: ("mtfsf", "mtfsf")}
# opcode 4, paired singles
_PS10 = {0: ("ps_cmpu0", "fcmp"), 32: ("ps_cmpo0", "fcmp"), 64: ("ps_cmpu1", "fcmp"),
         96: ("ps_cmpo1", "fcmp"), 40: ("ps_neg", "DB"), 72: ("ps_mr", "DB"),
         136: ("ps_nabs", "DB"), 264: ("ps_abs", "DB"), 528: ("ps_merge00", "DAB"),
         560: ("ps_merge01", "DAB"), 592: ("ps_merge10", "DAB"), 624: ("ps_merge11", "DAB"),
         1014: ("dcbz_l", "AB")}
_PS6 = {6: "psq_lx", 7: "psq_stx", 38: "psq_lux", 39: "psq_stux"}
_PS5 = {10: ("ps_sum0", "DACB"), 11: ("ps_sum1", "DACB"), 12: ("ps_muls0", "DAC"),
        13: ("ps_muls1", "DAC"), 14: ("ps_madds0", "DACB"), 15: ("ps_madds1", "DACB"),
        18: ("ps_div", "DAB"), 20: ("ps_sub", "DAB"), 21: ("ps_add", "DAB"),
        23: ("ps_sel", "DACB"), 24: ("ps_res", "DB"), 25: ("ps_mul", "DAC"),
        26: ("ps_rsqrte", "DB"), 28: ("ps_msub", "DACB"), 29: ("ps_madd", "DACB"),
        30: ("ps_nmsub", "DACB"), 31: ("ps_nmadd", "DACB")}

SPR_NAMES = {1: "xer", 8: "lr", 9: "ctr", 18: "dsisr", 19: "dar", 22: "dec", 25: "sdr1",
             26: "srr0", 27: "srr1", 268: "tbl", 269: "tbu", 272: "sprg0", 273: "sprg1",
             274: "sprg2", 275: "sprg3", 282: "ear", 284: "tbl", 285: "tbu", 287: "pvr",
             **{528 + i: f"ibat{i // 2}{'ul'[i % 2]}" for i in range(8)},
             **{536 + i: f"dbat{i // 2}{'ul'[i % 2]}" for i in range(8)},
             **{560 + i: f"ibat{4 + i // 2}{'ul'[i % 2]}" for i in range(8)},
             **{568 + i: f"dbat{4 + i // 2}{'ul'[i % 2]}" for i in range(8)},
             **{912 + i: f"gqr{i}" for i in range(8)},
             920: "hid2", 921: "wpar", 922: "dmau", 923: "dmal", 936: "ummcr0", 937: "upmc1",
             938: "upmc2", 939: "usia", 940: "ummcr1", 941: "upmc3", 942: "upmc4",
             952: "mmcr0", 953: "pmc1", 954: "pmc2", 955: "sia", 956: "mmcr1", 957: "pmc3",
             958: "pmc4", 1008: "hid0", 1009: "hid1", 1010: "iabr", 1011: "hid4",
             1013: "dabr", 1017: "l2cr", 1019: "ictc", 1020: "thrm1", 1021: "thrm2",
             1022: "thrm3"}


def decode(w):
    """Decode one big-endian instruction word. Unknown encodings -> op '.long'."""
    op = w >> 26
    D, A, B, C = (w >> 21) & 31, (w >> 16) & 31, (w >> 11) & 31, (w >> 6) & 31
    rc = w & 1
    if op in _D_FORM:
        return Ins(_D_FORM[op], w, D=D, A=A, simm=_s16(w & 0xFFFF))
    if op in _LOGIC_IMM:
        return Ins(_LOGIC_IMM[op], w, S=D, A=A, uimm=w & 0xFFFF)
    if op in _LOADSTORE:
        return Ins(_LOADSTORE[op], w, D=D, A=A, d=_s16(w & 0xFFFF))
    if op in _PSQ:
        return Ins(_PSQ[op], w, D=D, A=A, W=(w >> 15) & 1, I=(w >> 12) & 7, d=_s12(w & 0xFFF))
    if op in (10, 11):
        return Ins("cmpli" if op == 10 else "cmpi", w, crfD=(w >> 23) & 7, L=(w >> 21) & 1, A=A,
                   imm=(w & 0xFFFF) if op == 10 else _s16(w & 0xFFFF))
    if op == 3:
        return Ins("twi", w, TO=D, A=A, simm=_s16(w & 0xFFFF))
    if op == 18:
        li = w & 0x03FFFFFC
        return Ins("b", w, LI=li - 0x04000000 if li & 0x02000000 else li, AA=(w >> 1) & 1, LK=rc)
    if op == 16:
        bd = w & 0xFFFC
        return Ins("bc", w, BO=D, BI=A, BD=bd - 0x10000 if bd & 0x8000 else bd,
                   AA=(w >> 1) & 1, LK=rc)
    if op == 17 and (w >> 1) & 1:
        return Ins("sc", w)
    if op in (20, 21, 23):
        name = {20: "rlwimi", 21: "rlwinm", 23: "rlwnm"}[op]
        return Ins(name, w, S=D, A=A, SH=B, MB=C, ME=(w >> 1) & 31, Rc=rc)
    if op == 19:
        xo = (w >> 1) & 0x3FF
        if xo in _X19:
            name, shape = _X19[xo]
            if shape == "bcr":
                return Ins(name, w, BO=D, BI=A, LK=rc)
            if shape == "mcrf":
                return Ins(name, w, crfD=(w >> 23) & 7, crfS=(w >> 18) & 7)
            if shape == "crb":
                return Ins(name, w, crbD=D, crbA=A, crbB=B)
            return Ins(name, w)
    if op == 31:
        xo = (w >> 1) & 0x3FF
        if (xo & 0x1FF) in _X31_XO and not (xo == 11 + 512 or xo == 75 + 512):
            x9 = xo & 0x1FF
            return Ins(_X31_XO[x9], w, D=D, A=A, B=B, OE=(w >> 10) & 1, Rc=rc)
        if xo in _X31:
            name, shape = _X31[xo]
            f = {"Rc": rc}
            if shape == "cmp":
                f = dict(crfD=(w >> 23) & 7, L=(w >> 21) & 1, A=A, B=B)
            elif shape == "tw":
                f = dict(TO=D, A=A, B=B)
            elif shape in ("D", "S"):
                f = {shape: D}
            elif shape == "mtcrf":
                f = dict(S=D, CRM=(w >> 12) & 0xFF)
            elif shape in ("mfspr", "mtspr", "mftb"):
                f = dict(D=D, spr=A | (B << 5))
            elif shape == "mtsr":
                f = dict(S=D, SR=A & 15)
            elif shape == "mfsr":
                f = dict(D=D, SR=A & 15)
            elif shape == "SB":
                f = dict(S=D, B=B)
            elif shape == "DB":
                f = dict(D=D, B=B)
            elif shape == "crfD":
                f = dict(crfD=(w >> 23) & 7)
            elif shape in ("ASB", "AS"):
                f = dict(S=D, A=A, B=B, Rc=rc)
            elif shape == "srawi":
                f = dict(S=D, A=A, SH=B, Rc=rc)
            elif shape in ("DAB", "SAB", "fDAB"):
                f = dict(D=D, A=A, B=B)
            elif shape in ("lswi", "stswi"):
                f = dict(D=D, A=A, NB=B)
            elif shape == "AB":
                f = dict(A=A, B=B)
            elif shape == "B":
                f = dict(B=B)
            elif shape == "":
                f = {}
            return Ins(name, w, **f)
    if op == 59 and ((w >> 1) & 31) in _A59:
        name, shape = _A59[(w >> 1) & 31]
        return Ins(name, w, D=D, A=A, B=B, C=C, shape=shape, Rc=rc)
    if op == 63:
        xo = (w >> 1) & 0x3FF
        if xo in _X63:
            name, shape = _X63[xo]
            if shape == "fcmp":
                return Ins(name, w, crfD=(w >> 23) & 7, A=A, B=B)
            if shape == "mcrf":
                return Ins(name, w, crfD=(w >> 23) & 7, crfS=(w >> 18) & 7)
            if shape == "crbD":
                return Ins(name, w, crbD=D, Rc=rc)
            if shape == "mtfsfi":
                return Ins(name, w, crfD=(w >> 23) & 7, IMM=(w >> 12) & 15, Rc=rc)
            if shape == "mtfsf":
                return Ins(name, w, FM=(w >> 17) & 0xFF, B=B, Rc=rc)
            if shape == "D":
                return Ins(name, w, D=D, Rc=rc)
            return Ins(name, w, D=D, B=B, shape="DB", Rc=rc)
        if ((w >> 1) & 31) in _A63:
            name, shape = _A63[(w >> 1) & 31]
            return Ins(name, w, D=D, A=A, B=B, C=C, shape=shape, Rc=rc)
    if op == 4:
        xo = (w >> 1) & 0x3FF
        if xo in _PS10:
            name, shape = _PS10[xo]
            if shape == "fcmp":
                return Ins(name, w, crfD=(w >> 23) & 7, A=A, B=B)
            if shape == "AB":
                return Ins(name, w, A=A, B=B)
            return Ins(name, w, D=D, A=A, B=B, shape=shape, Rc=rc)
        if ((w >> 1) & 0x3F) in _PS6:
            return Ins(_PS6[(w >> 1) & 0x3F], w, D=D, A=A, B=B, W=(w >> 10) & 1, I=(w >> 7) & 7)
        if ((w >> 1) & 31) in _PS5:
            name, shape = _PS5[(w >> 1) & 31]
            return Ins(name, w, D=D, A=A, B=B, C=C, shape=shape, Rc=rc)
    return Ins(".long", w)


# --- text -------------------------------------------------------------------------

def _imm(v):
    return f"-{-v:#x}" if v < -9 else str(v) if -9 <= v <= 9 else f"{v:#x}"


_COND = {(12, 0): "lt", (12, 1): "gt", (12, 2): "eq", (12, 3): "so",
         (4, 0): "ge", (4, 1): "le", (4, 2): "ne", (4, 3): "ns"}


def _bc_name(bo, bi):
    """Simplified mnemonic stem and whether BI is shown, or None for plain bc."""
    b = bo & 0x1E                          # drop the static prediction bit
    if bo & 0x14 == 0x14:
        return "", False                   # branch always
    if b in (12, 4) or bo in (12, 13, 14, 15, 4, 5, 6, 7):
        return _COND[(12 if bo & 8 else 4, bi & 3)], True
    ctr = {16: "dnz", 18: "dz"}.get(b)
    if ctr:
        return ctr, False
    ctrc = {0: "dnzf", 2: "dzf", 8: "dnzt", 10: "dzt"}.get(b)
    if ctrc:
        return ctrc, "bit"
    return None, None


def text(ins, addr=0):
    """Render with simplified mnemonics, capstone-style operands."""
    f, op = ins.f, ins.op
    r = lambda n: f"r{n}"
    ra0 = lambda n: f"r{n}" if n else "0"      # rA = 0 reads as the value 0
    fr = lambda n: f"f{n}"
    dot = "." if f.get("Rc") else ""
    if op == ".long":
        return f".long {ins.word:#010x}"
    if op in ("addi", "addis"):
        if f["A"] == 0:
            return f"{'li' if op == 'addi' else 'lis'} {r(f['D'])}, {_imm(f['simm'])}"
        return f"{op} {r(f['D'])}, {r(f['A'])}, {_imm(f['simm'])}"
    if op in _D_FORM.values():
        return f"{op} {r(f['D'])}, {r(f['A'])}, {_imm(f['simm'])}"
    if op in _LOGIC_IMM.values():
        if op == "ori" and f["S"] == f["A"] == f["uimm"] == 0:
            return "nop"
        return f"{op} {r(f['A'])}, {r(f['S'])}, {_imm(f['uimm'])}"
    if op in _LOADSTORE.values():
        reg = fr if op.startswith(("lf", "stf")) else r
        return f"{op} {reg(f['D'])}, {_imm(f['d'])}({ra0(f['A'])})"
    if op in _PSQ.values():
        return f"{op} {fr(f['D'])}, {_imm(f['d'])}({ra0(f['A'])}), {f['W']}, {f['I']}"
    if op in _PS6.values():
        return f"{op} {fr(f['D'])}, {ra0(f['A'])}, {r(f['B'])}, {f['W']}, {f['I']}"
    if op in ("cmpi", "cmpli", "cmp", "cmpl"):
        stem = {"cmpi": "cmpwi", "cmpli": "cmplwi", "cmp": "cmpw", "cmpl": "cmplw"}[op]
        cr = f"cr{f['crfD']}, " if f["crfD"] else ""
        last = _imm(f["imm"]) if "imm" in f else r(f["B"])
        return f"{stem} {cr}{r(f['A'])}, {last}"
    if op == "twi":
        return f"twi {f['TO']}, {r(f['A'])}, {_imm(f['simm'])}"
    if op == "tw":
        return "trap" if f["TO"] == 31 and f["A"] == f["B"] == 0 else f"tw {f['TO']}, {r(f['A'])}, {r(f['B'])}"
    if op == "b":
        tgt = f["LI"] if f["AA"] else (addr + f["LI"]) & 0xFFFFFFFF
        return f"b{'l' if f['LK'] else ''}{'a' if f['AA'] else ''} {tgt:#x}"
    if op in ("bc", "bclr", "bcctr"):
        stem, show = _bc_name(f["BO"], f["BI"])
        suffix = {"bc": "", "bclr": "lr", "bcctr": "ctr"}[op] + ("l" if f["LK"] else "")
        tgt = ""
        if op == "bc":
            t = f["BD"] if f["AA"] else (addr + f["BD"]) & 0xFFFFFFFF
            tgt = f"{t:#x}"
        if stem is None:
            ops = [str(f["BO"]), str(f["BI"])] + ([tgt] if tgt else [])
            return f"{op}{'l' if f['LK'] else ''} {', '.join(ops)}"
        ops = []
        if show is True and f["BI"] >> 2:
            ops.append(f"cr{f['BI'] >> 2}")
        elif show == "bit":
            ops.append(str(f["BI"]))
        if tgt:
            ops.append(tgt)
        return f"b{stem}{suffix}" + (" " + ", ".join(ops) if ops else "")
    if op in ("rlwinm", "rlwimi", "rlwnm"):
        S, A, SH, MB, ME = f["S"], f["A"], f["SH"], f["MB"], f["ME"]
        if op == "rlwinm":
            if MB == 0 and ME == 31 - SH:
                return f"slwi{dot} {r(A)}, {r(S)}, {_imm(SH)}"
            if ME == 31 and SH and MB == 32 - SH:
                return f"srwi{dot} {r(A)}, {r(S)}, {_imm(MB)}"
            if SH == 0 and ME == 31:
                return f"clrlwi{dot} {r(A)}, {r(S)}, {_imm(MB)}"
            if SH == 0 and MB == 0:
                return f"clrrwi{dot} {r(A)}, {r(S)}, {_imm(31 - ME)}"
            if MB == 0 and ME == 31:
                return f"rotlwi{dot} {r(A)}, {r(S)}, {_imm(SH)}"
        last = r(SH) if op == "rlwnm" else _imm(SH)
        return f"{op}{dot} {r(A)}, {r(S)}, {last}, {_imm(MB)}, {_imm(ME)}"
    if op in _X31_XO.values():
        o = "o" if f["OE"] else ""
        if op in ("neg", "subfze", "addze", "subfme", "addme"):
            return f"{op}{o}{dot} {r(f['D'])}, {r(f['A'])}"
        if op == "subf":
            return f"sub{o}{dot} {r(f['D'])}, {r(f['B'])}, {r(f['A'])}"
        return f"{op}{o}{dot} {r(f['D'])}, {r(f['A'])}, {r(f['B'])}"
    if op in ("mfspr", "mtspr", "mftb"):
        n = SPR_NAMES.get(f["spr"], str(f["spr"]))
        if op == "mftb":
            return f"mftb{'u' if f['spr'] == 269 else ''} {r(f['D'])}"
        if f["spr"] in (1, 8, 9):
            return f"{op[:2]}{n} {r(f['D'])}"
        return f"mfspr {r(f['D'])}, {n}" if op == "mfspr" else f"mtspr {n}, {r(f['D'])}"
    if op == "or" and f["S"] == f["B"]:
        return f"mr{dot} {r(f['A'])}, {r(f['S'])}"
    if op == "nor" and f["S"] == f["B"]:
        return f"not{dot} {r(f['A'])}, {r(f['S'])}"
    if op in ("slw", "and", "andc", "nor", "eqv", "xor", "orc", "or", "nand", "srw", "sraw"):
        return f"{op}{dot} {r(f['A'])}, {r(f['S'])}, {r(f['B'])}"
    if op in ("cntlzw", "extsh", "extsb"):
        return f"{op}{dot} {r(f['A'])}, {r(f['S'])}"
    if op == "srawi":
        return f"srawi{dot} {r(f['A'])}, {r(f['S'])}, {_imm(f['SH'])}"
    if op in ("mfcr", "mfmsr", "mffs"):
        return f"{op}{dot} {(fr if op == 'mffs' else r)(f['D'])}"
    if op == "mtmsr":
        return f"mtmsr {r(f['S'])}"
    if op == "mtcrf":
        return f"mtcr {r(f['S'])}" if f["CRM"] == 0xFF else f"mtcrf {f['CRM']:#x}, {r(f['S'])}"
    if op in ("mtsr", "mfsr"):
        return f"mtsr {f['SR']}, {r(f['S'])}" if op == "mtsr" else f"mfsr {r(f['D'])}, {f['SR']}"
    if op in ("mtsrin", "mfsrin"):
        return f"{op} {r(f.get('S', f.get('D')))}, {r(f['B'])}"
    if op in ("mcrxr",):
        return f"mcrxr cr{f['crfD']}"
    if op in ("lfsx", "lfsux", "lfdx", "lfdux", "stfsx", "stfsux", "stfdx", "stfdux", "stfiwx"):
        return f"{op} {fr(f['D'])}, {ra0(f['A'])}, {r(f['B'])}"
    if op in ("lswi", "stswi"):
        return f"{op} {r(f['D'])}, {ra0(f['A'])}, {f['NB']}"
    if "D" in f and "A" in f and "B" in f and op in dict(_X31.values()) and not op.startswith("f"):
        return f"{op} {r(f['D'])}, {ra0(f['A'])}, {r(f['B'])}"
    if op in ("dcbst", "dcbf", "dcbtst", "dcbt", "dcbi", "icbi", "dcbz", "dcbz_l"):
        return f"{op} {ra0(f['A'])}, {r(f['B'])}"
    if op == "tlbie":
        return f"tlbie {r(f['B'])}"
    if op in ("mcrf", "mcrfs"):
        return f"{op} cr{f['crfD']}, cr{f['crfS']}"
    if op.startswith("cr"):
        a, b, d = f["crbA"], f["crbB"], f["crbD"]
        if op == "crxor" and a == b == d:
            return f"crclr {_crbit(d)}"
        if op == "creqv" and a == b == d:
            return f"crset {_crbit(d)}"
        if op == "cror" and a == b:
            return f"crmove {_crbit(d)}, {_crbit(a)}"
        if op == "crnor" and a == b:
            return f"crnot {_crbit(d)}, {_crbit(a)}"
        return f"{op} {_crbit(d)}, {_crbit(a)}, {_crbit(b)}"
    if op in ("fcmpu", "fcmpo", "ps_cmpu0", "ps_cmpo0", "ps_cmpu1", "ps_cmpo1"):
        return f"{op} cr{f['crfD']}, {fr(f['A'])}, {fr(f['B'])}"
    if op in ("mtfsb0", "mtfsb1"):
        return f"{op}{dot} {f['crbD']}"
    if op == "mtfsfi":
        return f"mtfsfi{dot} cr{f['crfD']}, {f['IMM']}"
    if op == "mtfsf":
        return f"mtfsf{dot} {f['FM']:#x}, {fr(f['B'])}"
    if "shape" in f:
        regs = {k: f.get(k, 0) for k in "DABC"}
        return f"{op}{dot} " + ", ".join(fr(regs[k]) for k in f["shape"])
    if not f or f == {"Rc": 0}:
        return op
    return f"{op} {f}"


def _crbit(n):
    return f"cr{n >> 2}{('lt', 'gt', 'eq', 'un')[n & 3]}"


def branch_target(ins, addr):
    f = ins.f
    if ins.op == "b":
        return f["LI"] if f["AA"] else (addr + f["LI"]) & 0xFFFFFFFF
    if ins.op == "bc":
        return f["BD"] if f["AA"] else (addr + f["BD"]) & 0xFFFFFFFF
    return None


def words(img, addr, size):
    b = img.read(addr, size) or b""
    for i in range(0, len(b) - 3, 4):
        yield addr + i, struct.unpack_from(">I", b, i)[0]


# --- analysis over an image ------------------------------------------------------

class Tracker:
    """Constant tracking of `lis`-built addresses inside one function."""

    _STORE = {"stw", "stwu", "stb", "stbu", "sth", "sthu", "stfs", "stfsu", "stfd", "stfdu",
              "stmw", "psq_st", "psq_stu"}

    def __init__(self, img):
        self.img, self.regs = img, {}

    def step(self, ins):
        """Return the absolute address this instruction completes, if any."""
        f, op, img = ins.f, ins.op, self.img
        out = None
        if op == "addis" and f["A"] == 0:
            self.regs[f["D"]] = (f["simm"] << 16) & 0xFFFFFFFF
            return None
        base = f.get("A")
        if op in ("addi", "ori") and base in self.regs and (op == "ori" or base != 0):
            v = self.regs[base]
            out = (v + f["simm"]) & 0xFFFFFFFF if op == "addi" else v | f["uimm"]
            dst = f["D"] if op == "addi" else f["A"]
            self.regs[dst] = out
            return out
        if "d" in f and base is not None:
            if base == 13 and img.sda is not None:
                out = (img.sda + f["d"]) & 0xFFFFFFFF
            elif base == 2 and img.sda2 is not None:
                out = (img.sda2 + f["d"]) & 0xFFFFFFFF
            elif base in self.regs and base != 0:
                out = (self.regs[base] + f["d"]) & 0xFFFFFFFF
            if op not in self._STORE and op not in ("lmw",) and "D" in f and not op.startswith(("lf", "psq")):
                self.regs.pop(f["D"], None)
            if op.endswith("u"):
                self.regs.pop(base, None)
            return out
        if op in ("b", "bc", "bclr", "bcctr") and ins.f.get("LK"):
            for k in range(0, 13):
                self.regs.pop(k, None)
            return None
        for k in ("D", "A") if op not in self._STORE else ():
            if k in f and op not in ("cmpi", "cmpli", "cmp", "cmpl", "tw", "twi"):
                self.regs.pop(f[k], None)
        return None


def describe(img, addr):
    s = img.cstring(addr)
    if s is not None and len(s) >= 3:
        return '"' + s.replace("\n", "\\n")[:80] + '"'
    return img.symbol_at(addr) or f"{addr:#010x}"


def disassemble(img, func, demangle=True, out=print):
    out(f"; {cw.demangle(func.name) if demangle else func.name}")
    out(f"; {func.name}  @{func.addr:08X}  size {func.size}")
    tr = Tracker(img)
    for a, w in words(img, func.addr, func.size):
        ins = decode(w)
        note = ""
        ref = tr.step(ins)
        tgt = branch_target(ins, a)
        if ref is not None:
            note = describe(img, ref)
        elif tgt is not None and not (func.addr <= tgt < func.addr + func.size):
            n = img.symbol_at(tgt)
            note = (cw.demangle(n) if demangle and n and "+" not in n else n) or ""
        out(f"  {a:08X}  {w:08X}  {text(ins, a):40}{'; ' + note if note else ''}")


def iter_functions(img):
    for fn in img.functions():
        if img.segment_at(fn.addr) and img.segment_at(fn.addr).text:
            yield fn


def callers(img, target):
    for fn in iter_functions(img):
        for a, w in words(img, fn.addr, fn.size):
            if w >> 26 == 18 and w & 1:
                ins = decode(w)
                if branch_target(ins, a) == target:
                    yield a, fn


def xrefs(img, target, span=1):
    for fn in iter_functions(img):
        tr = Tracker(img)
        for a, w in words(img, fn.addr, fn.size):
            ref = tr.step(decode(w))
            if ref is not None and target <= ref < target + span:
                yield a, fn, ref


def mix(img):
    ops = collections.Counter()
    unknown = []
    for fn in iter_functions(img):
        for a, w in words(img, fn.addr, fn.size):
            ins = decode(w)
            ops[ins.op] += 1
            if ins.op == ".long":
                unknown.append((a, w, fn.name))
    return ops, unknown


def _resolve(img, q):
    if q.lower().startswith("0x"):
        a = int(q, 16)
        name = img.symbol_at(a, want=(2,))
        return a, name
    hits = img.find(q)
    if not hits:
        raise SystemExit(f"no function matches {q!r}")
    if len(hits) > 1:
        for h in hits[:30]:
            print(f"  {h.addr:08X}  {h.name}")
        raise SystemExit(f"{len(hits)} matches; be more specific")
    return hits[0].addr, hits[0].name


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image")
    ap.add_argument("--func", metavar="NAME|ADDR")
    ap.add_argument("--callers", metavar="NAME|ADDR")
    ap.add_argument("--xref", metavar="ADDR")
    ap.add_argument("--span", type=lambda s: int(s, 0), default=1)
    ap.add_argument("--mix", action="store_true")
    ap.add_argument("--at", metavar="ADDR")
    ap.add_argument("--count", type=int, default=16)
    ap.add_argument("--raw", action="store_true", help="do not demangle")
    a = ap.parse_args()
    img = Image(a.image)
    if a.func:
        addr, _ = _resolve(img, a.func)
        fn = next(s for s in img.symbols if s.addr == addr and s.is_func)
        disassemble(img, fn, not a.raw)
    elif a.callers:
        addr, name = _resolve(img, a.callers)
        for site, fn in callers(img, addr):
            print(f"  {site:08X}  {fn.name}+{site - fn.addr:#x}")
    elif a.xref:
        for site, fn, ref in xrefs(img, int(a.xref, 16), a.span):
            print(f"  {site:08X}  {fn.name}+{site - fn.addr:#x}  -> {ref:08X}")
    elif a.mix:
        ops, unknown = mix(img)
        total = sum(ops.values())
        ps = sum(n for o, n in ops.items() if o.startswith(("ps_", "psq_", "dcbz_l")))
        print(f"{total} instructions, {len(ops)} distinct operations, "
              f"{ps} paired-single ({100 * ps / total:.2f}%), {len(unknown)} undecoded")
        for o, n in ops.most_common():
            print(f"  {o:12} {n}")
        for a_, w, n in unknown[:20]:
            print(f"  undecoded {a_:08X} {w:08X} in {n}")
    elif a.at:
        base = int(a.at, 16)
        for addr, w in words(img, base, a.count * 4):
            print(f"  {addr:08X}  {w:08X}  {text(decode(w), addr)}")


if __name__ == "__main__":
    main()
