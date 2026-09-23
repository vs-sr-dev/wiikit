"""Gekko instruction -> C++ statement(s), against wiikit/runtime/ppc.h.

emit(ins, addr, fn) returns a list of C++ lines for one instruction; `fn`
answers the control-flow questions (is this target a label here, what is
the C++ name of the function at an address, the switch table of a bctr).
Every statement is wrapped in its own block so that `goto` never crosses an
initialisation.
"""


class Unsupported(Exception):
    pass


def fname(addr):
    return f"f_{addr:08X}"


def _h(v):
    return f"0x{v & 0xFFFFFFFF:X}u"


def _mask(mb, me):
    m = 0
    i = mb
    while True:
        m |= 1 << (31 - i)
        if i == me:
            break
        i = (i + 1) & 31
    return m


def _r(n):
    return f"c.r[{n}]"


def _ra0(n):                              # rA|0
    return f"c.r[{n}]" if n else "0u"


def _ea_d(f):
    d = f["d"]
    if f["A"] == 0:
        return _h(d)
    return _r(f["A"]) if d == 0 else f"({_r(f['A'])} + {_h(d)})"


def _ea_x(f):
    return f"{_r(f['B'])}" if f["A"] == 0 else f"({_r(f['A'])} + {_r(f['B'])})"


_LOAD = {"lwz": "ld32({ea})", "lbz": "ld8({ea})", "lhz": "ld16({ea})",
         "lha": "(uint32_t)(int32_t)(int16_t)ld16({ea})"}
_STORE = {"stw": "st32({ea}, {v})", "stb": "st8({ea}, (uint8_t){v})", "sth": "st16({ea}, (uint16_t){v})"}
_LOADX = {"lwzx": "ld32(ea)", "lbzx": "ld8(ea)", "lhzx": "ld16(ea)",
          "lhax": "(uint32_t)(int32_t)(int16_t)ld16(ea)", "lwbrx": "PPC_BSWAP32(ld32(ea))",
          "lhbrx": "(uint32_t)PPC_BSWAP16(ld16(ea))"}
_STOREX = {"stwx": "st32(ea, {v})", "stbx": "st8(ea, (uint8_t){v})", "sthx": "st16(ea, (uint16_t){v})",
           "stwbrx": "st32(ea, PPC_BSWAP32({v}))", "sthbrx": "st16(ea, PPC_BSWAP16((uint16_t){v}))"}
_LOGIC = {"and": "{s} & {b}", "andc": "{s} & ~{b}", "or": "{s} | {b}", "orc": "{s} | ~{b}",
          "xor": "{s} ^ {b}", "nor": "~({s} | {b})", "nand": "~({s} & {b})", "eqv": "~({s} ^ {b})",
          "slw": "slw({s}, {b})", "srw": "srw({s}, {b})", "sraw": "sraw(c, {s}, {b})"}
# carrying adds: (x, y, carry-in) of x + y + cin
_ADDS = {"addc": ("a", "b", "0"), "adde": ("a", "b", "c.xer_ca"), "addze": ("a", "0u", "c.xer_ca"),
         "addme": ("a", "0xFFFFFFFFu", "c.xer_ca"), "subfc": ("~a", "b", "1"),
         "subfe": ("~a", "b", "c.xer_ca"), "subfze": ("~a", "0u", "c.xer_ca"),
         "subfme": ("~a", "0xFFFFFFFFu", "c.xer_ca")}
_CROPS = {"crand": "a & b", "crandc": "a & !b", "cror": "a | b", "crorc": "a | !b",
          "crxor": "a ^ b", "crnor": "!(a | b)", "crnand": "!(a & b)", "creqv": "!(a ^ b)"}
_FP2 = {"fadd": "a + b", "fsub": "a - b", "fdiv": "a / b", "fmul": "a * cc",
        "fmadd": "a * cc + b", "fmsub": "a * cc - b", "fnmadd": "-(a * cc + b)",
        "fnmsub": "-(a * cc - b)", "fsel": "a >= 0.0 ? cc : b", "fsqrt": "std::sqrt(b)",
        "frsqrte": "frsqrte(b)"}
_FPS = {"fadds": "a + b", "fsubs": "a - b", "fdivs": "a / b", "fmuls": "a * cc",
        "fmadds": "a * cc + b", "fmsubs": "a * cc - b", "fnmadds": "-(a * cc + b)",
        "fnmsubs": "-(a * cc - b)", "fsqrts": "std::sqrt(b)", "fres": "fres(b)", "frsp": "b"}
_FPMOVE = {"fmr": "b", "fneg": "-b", "fabs": "std::fabs(b)", "fnabs": "-std::fabs(b)"}
# paired singles: lane expressions (ps0, ps1) over a0 a1 b0 b1 c0 c1
_PS = {"ps_add": ("a0 + b0", "a1 + b1"), "ps_sub": ("a0 - b0", "a1 - b1"),
       "ps_mul": ("a0 * c0", "a1 * c1"), "ps_div": ("a0 / b0", "a1 / b1"),
       "ps_madd": ("a0 * c0 + b0", "a1 * c1 + b1"), "ps_msub": ("a0 * c0 - b0", "a1 * c1 - b1"),
       "ps_nmadd": ("-(a0 * c0 + b0)", "-(a1 * c1 + b1)"),
       "ps_nmsub": ("-(a0 * c0 - b0)", "-(a1 * c1 - b1)"),
       "ps_sel": ("a0 >= 0.0 ? c0 : b0", "a1 >= 0.0 ? c1 : b1"),
       "ps_res": ("1.0 / b0", "1.0 / b1"),
       "ps_rsqrte": ("1.0 / std::sqrt(b0)", "1.0 / std::sqrt(b1)"),
       "ps_sum0": ("a0 + b1", "c1"), "ps_sum1": ("c0", "a0 + b1"),
       "ps_muls0": ("a0 * c0", "a1 * c0"), "ps_muls1": ("a0 * c1", "a1 * c1"),
       "ps_madds0": ("a0 * c0 + b0", "a1 * c0 + b1"), "ps_madds1": ("a0 * c1 + b0", "a1 * c1 + b1")}
_PSMOVE = {"ps_merge00": ("a0", "b0"), "ps_merge01": ("a0", "b1"), "ps_merge10": ("a1", "b0"),
           "ps_merge11": ("a1", "b1"), "ps_mr": ("b0", "b1"), "ps_neg": ("-b0", "-b1"),
           "ps_abs": ("std::fabs(b0)", "std::fabs(b1)"), "ps_nabs": ("-std::fabs(b0)", "-std::fabs(b1)")}
_NOP = {"sync", "isync", "eieio", "tlbsync", "tlbie", "dcbst", "dcbf", "dcbt", "dcbtst",
        "dcbi", "icbi"}


def _bc_cond(bo, bi):
    """(pre-statement, condition) for a BO/BI pair."""
    pre, parts = "", []
    if not bo & 4:
        pre = "c.ctr--; "
        parts.append("c.ctr == 0" if bo & 2 else "c.ctr != 0")
    if not bo & 16:
        parts.append(f"c.cr[{bi}]" if bo & 8 else f"!c.cr[{bi}]")
    return pre, " && ".join(parts)


def emit(ins, addr, fn):
    """C++ lines for one instruction. `fn` provides goto/call/table resolution."""
    op, f = ins.op, ins.f
    nxt = addr + 4
    rc = f.get("Rc")

    # ---- branches -------------------------------------------------------------------
    if op == "b":
        t = f["LI"] if f["AA"] else (addr + f["LI"]) & 0xFFFFFFFF
        if f["LK"]:
            return [f"c.lr = {_h(nxt)}; {fn.call(t)}"]
        return [fn.jump(t)]
    if op == "bc":
        t = f["BD"] if f["AA"] else (addr + f["BD"]) & 0xFFFFFFFF
        pre, cond = _bc_cond(f["BO"], f["BI"])
        if f["LK"]:
            act = f"c.lr = {_h(nxt)};" if t == nxt else f"c.lr = {_h(nxt)}; {fn.call(t)}"
        else:
            act = fn.jump(t)
        return [f"{{ {pre}if ({cond}) {{ {act} }} }}" if cond else f"{{ {pre}{act} }}"]
    if op == "bclr":
        pre, cond = _bc_cond(f["BO"], f["BI"])
        if f["LK"]:
            act = f"{{ uint32_t t = c.lr; c.lr = {_h(nxt)}; ppc_call_indirect(c, t); }}"
        else:
            act = "return;"
        return [f"{{ {pre}if ({cond}) {act} }}" if cond else f"{{ {pre}{act} }}"]
    if op == "bcctr":
        pre, cond = _bc_cond(f["BO"], f["BI"])
        if f["LK"]:
            act = f"{{ c.lr = {_h(nxt)}; ppc_call_indirect(c, c.ctr); }}"
        elif not cond and addr in fn.tables:
            cases = " ".join(f"case {_h(t)}: goto L_{t:08X};" for t in sorted(set(fn.tables[addr])))
            return [f"switch (c.ctr) {{ {cases} default: ppc_call_indirect(c, c.ctr); return; }}"]
        else:
            act = "{ ppc_call_indirect(c, c.ctr); return; }"
        return [f"{{ {pre}if ({cond}) {act} }}" if cond else f"{{ {pre}{act} }}"]

    # ---- integer, immediate -----------------------------------------------------------
    if op == "addi":
        return [f"c.r[{f['D']}] = {_ra0(f['A'])} + {_h(f['simm'])};"]
    if op == "addis":
        return [f"c.r[{f['D']}] = {_ra0(f['A'])} + {_h(f['simm'] << 16)};"]
    if op in ("addic", "addic."):
        s = f"c.r[{f['D']}] = add_ca(c, c.r[{f['A']}], {_h(f['simm'])}, 0);"
        return [s + (f" cr0_rc(c, c.r[{f['D']}]);" if op == "addic." else "")]
    if op == "subfic":
        return [f"c.r[{f['D']}] = add_ca(c, ~c.r[{f['A']}], {_h(f['simm'])}, 1);"]
    if op == "mulli":
        return [f"c.r[{f['D']}] = c.r[{f['A']}] * {_h(f['simm'])};"]
    if op in ("ori", "oris", "xori", "xoris", "andi.", "andis."):
        imm = f["uimm"] << 16 if op.endswith(("is", "is.")) else f["uimm"]
        o = {"o": "|", "x": "^", "a": "&"}[op[0]]
        s = f"c.r[{f['A']}] = c.r[{f['S']}] {o} {_h(imm)};"
        return [s + (f" cr0_rc(c, c.r[{f['A']}]);" if op.endswith(".") else "")]
    if op in ("cmpi", "cmpli", "cmp", "cmpl"):
        b = _h(f["imm"]) if "imm" in f else f"c.r[{f['B']}]"
        if op in ("cmpi", "cmp"):
            return [f"cr_cmp_s(c, {f['crfD']}, (int32_t)c.r[{f['A']}], (int32_t){b});"]
        return [f"cr_cmp_u(c, {f['crfD']}, c.r[{f['A']}], {b});"]
    if op in ("twi", "tw"):
        b = _h(f["simm"]) if op == "twi" else f"c.r[{f['B']}]"
        if f["TO"] == 31:
            return [f"ppc_trap(c, {_h(addr)});"]
        return [f"if (trap_cond({f['TO']}, c.r[{f['A']}], {b})) ppc_trap(c, {_h(addr)});"]

    # ---- rotates ------------------------------------------------------------------------
    if op in ("rlwinm", "rlwimi", "rlwnm"):
        m = _h(_mask(f["MB"], f["ME"]))
        sh = f"c.r[{f['SH']}]" if op == "rlwnm" else str(f["SH"])
        rot = f"rotl32(c.r[{f['S']}], {sh})"
        if op == "rlwimi":
            s = f"c.r[{f['A']}] = ({rot} & {m}) | (c.r[{f['A']}] & ~{m});"
        else:
            s = f"c.r[{f['A']}] = {rot} & {m};"
        return [s + (f" cr0_rc(c, c.r[{f['A']}]);" if rc else "")]

    # ---- integer, register ------------------------------------------------------------
    if op in ("add", "subf", "neg", "mullw", "mulhw", "mulhwu", "divw", "divwu") or op in _ADDS:
        D, A, B = f["D"], f["A"], f["B"]
        body = f"uint32_t a = c.r[{A}], b = c.r[{B}]; uint32_t r;"
        ov = None
        if op == "add":
            body += " r = a + b;"
            ov = "((a ^ r) & (b ^ r)) >> 31"
        elif op == "subf":
            body += " r = b - a;"
            ov = "((~a ^ r) & (b ^ r)) >> 31"
        elif op == "neg":
            body += " r = 0u - a;"
            ov = "a == 0x80000000u"
        elif op == "mullw":
            body += " int64_t p = (int64_t)(int32_t)a * (int32_t)b; r = (uint32_t)p;"
            ov = "p != (int64_t)(int32_t)r"
        elif op == "mulhw":
            body += " r = (uint32_t)(((int64_t)(int32_t)a * (int32_t)b) >> 32);"
        elif op == "mulhwu":
            body += " r = (uint32_t)(((uint64_t)a * b) >> 32);"
        elif op == "divw":
            body += " r = divw(a, b);"
            ov = "b == 0 || (a == 0x80000000u && b == 0xFFFFFFFFu)"
        elif op == "divwu":
            body += " r = divwu(a, b);"
            ov = "b == 0"
        else:
            x, y, cin = _ADDS[op]
            body += f" uint32_t x = {x}, y = {y}; r = add_ca(c, x, y, {cin});"
            ov = "((x ^ r) & (y ^ r)) >> 31"
        body += f" c.r[{D}] = r;"
        if f.get("OE"):
            if ov is None:
                raise Unsupported(f"{op}o")
            body += f" set_ov(c, {ov});"
        if rc:
            body += " cr0_rc(c, r);"
        return ["{ " + body + " }"]
    if op in _LOGIC:
        e = _LOGIC[op].format(s=f"c.r[{f['S']}]", b=f"c.r[{f['B']}]")
        return [f"c.r[{f['A']}] = {e};" + (f" cr0_rc(c, c.r[{f['A']}]);" if rc else "")]
    if op == "srawi":
        return [f"c.r[{f['A']}] = sraw(c, c.r[{f['S']}], {f['SH']});" + (f" cr0_rc(c, c.r[{f['A']}]);" if rc else "")]
    if op in ("cntlzw", "extsh", "extsb"):
        e = {"cntlzw": "cntlzw({s})", "extsh": "(uint32_t)(int32_t)(int16_t){s}",
             "extsb": "(uint32_t)(int32_t)(int8_t){s}"}[op].format(s=f"c.r[{f['S']}]")
        return [f"c.r[{f['A']}] = {e};" + (f" cr0_rc(c, c.r[{f['A']}]);" if rc else "")]

    # ---- loads and stores -----------------------------------------------------------------
    base = op.rstrip("u") if op.endswith("u") and op not in ("psq_lu", "psq_stu") else op
    if base in _LOAD and op in (base, base + "u"):
        if op == base:
            return [f"c.r[{f['D']}] = {_LOAD[base].format(ea=_ea_d(f))};"]
        return [f"{{ uint32_t ea = {_ea_d(f)}; c.r[{f['D']}] = {_LOAD[base].format(ea='ea')}; c.r[{f['A']}] = ea; }}"]
    if base in _STORE and op in (base, base + "u"):
        v = f"c.r[{f['D']}]"
        if op == base:
            return [_STORE[base].format(ea=_ea_d(f), v=v) + ";"]
        return [f"{{ uint32_t ea = {_ea_d(f)}; {_STORE[base].format(ea='ea', v=v)}; c.r[{f['A']}] = ea; }}"]
    if op in ("lfs", "lfsu", "lfd", "lfdu", "stfs", "stfsu", "stfd", "stfdu"):
        upd = op.endswith("u")
        k = op.rstrip("u")
        stmt = {"lfs": "{ double v = (double)as_f32(ld32(ea)); c.f[D] = v; c.ps1[D] = v; }",
                "lfd": "c.f[D] = as_f64(ld64(ea));",
                "stfs": "st32(ea, as_u32((float)c.f[D]));",
                "stfd": "st64(ea, as_u64(c.f[D]));"}[k].replace("[D]", f"[{f['D']}]")
        return [f"{{ uint32_t ea = {_ea_d(f)}; {stmt}" + (f" c.r[{f['A']}] = ea;" if upd else "") + " }"]
    if op in ("lfsx", "lfsux", "lfdx", "lfdux", "stfsx", "stfsux", "stfdx", "stfdux", "stfiwx"):
        upd = op.endswith("ux")
        k = op.replace("ux", "x")
        stmt = {"lfsx": "{ double v = (double)as_f32(ld32(ea)); c.f[D] = v; c.ps1[D] = v; }",
                "lfdx": "c.f[D] = as_f64(ld64(ea));",
                "stfsx": "st32(ea, as_u32((float)c.f[D]));",
                "stfdx": "st64(ea, as_u64(c.f[D]));",
                "stfiwx": "st32(ea, (uint32_t)as_u64(c.f[D]));"}[k].replace("[D]", f"[{f['D']}]")
        return [f"{{ uint32_t ea = {_ea_x(f)}; {stmt}" + (f" c.r[{f['A']}] = ea;" if upd else "") + " }"]
    if op in _LOADX or (op.endswith("ux") and op.replace("ux", "x") in _LOADX):
        k = op.replace("ux", "x")
        upd = f" c.r[{f['A']}] = ea;" if op.endswith("ux") else ""
        return [f"{{ uint32_t ea = {_ea_x(f)}; c.r[{f['D']}] = {_LOADX[k]};{upd} }}"]
    if op in _STOREX or (op.endswith("ux") and op.replace("ux", "x") in _STOREX):
        k = op.replace("ux", "x")
        upd = f" c.r[{f['A']}] = ea;" if op.endswith("ux") else ""
        return [f"{{ uint32_t ea = {_ea_x(f)}; {_STOREX[k].format(v=f'c.r[{f['D']}]')};{upd} }}"]
    if op in ("lmw", "stmw"):
        out = [f"{{ uint32_t ea = {_ea_d(f)};"]
        for k, rr in enumerate(range(f["D"], 32)):
            out.append(f" c.r[{rr}] = ld32(ea + {4 * k}u);" if op == "lmw" else f" st32(ea + {4 * k}u, c.r[{rr}]);")
        return ["".join(out) + " }"]
    if op == "lwarx":
        return [f"{{ uint32_t ea = {_ea_x(f)}; c.reserve = ea; c.r[{f['D']}] = ld32(ea); }}"]
    if op == "stwcx.":
        return [f"{{ uint32_t ea = {_ea_x(f)}; st32(ea, c.r[{f['D']}]); cr_set(c, 0, 0, 0, 1, c.xer_so); }}"]
    if op in ("lswi", "stswi"):
        n = f["NB"] or 32
        fn_ = "ppc_lswx" if op == "lswi" else "ppc_stswx"
        return [f"{fn_}(c, {f['D']}, {_ra0(f['A'])}, {n});"]
    if op in ("lswx", "stswx"):
        fn_ = "ppc_lswx" if op == "lswx" else "ppc_stswx"
        return [f"{{ uint32_t ea = {_ea_x(f)}; {fn_}(c, {f['D']}, ea, c.xer_bc); }}"]
    if op in ("dcbz", "dcbz_l"):
        return [f"ppc_dcbz({_ea_x(f)});"]
    if op in _NOP:
        return [f"/* {op} */"]

    # ---- system registers ---------------------------------------------------------------
    if op == "mfspr":
        n, D = f["spr"], f["D"]
        src = {1: "mfxer(c)", 8: "c.lr", 9: "c.ctr", 268: "(uint32_t)ppc_timebase()",
               269: "(uint32_t)(ppc_timebase() >> 32)"}.get(n)
        if src is None:
            src = f"c.gqr[{n - 912}]" if 912 <= n < 920 else f"c.spr[{n}]"
        return [f"c.r[{D}] = {src};"]
    if op == "mtspr":
        n, S = f["spr"], f["D"]
        if n == 1:
            return [f"mtxer(c, c.r[{S}]);"]
        dst = {8: "c.lr", 9: "c.ctr"}.get(n) or (f"c.gqr[{n - 912}]" if 912 <= n < 920 else f"c.spr[{n}]")
        return [f"{dst} = c.r[{S}];"]
    if op == "mftb":
        return [f"c.r[{f['D']}] = (uint32_t)(ppc_timebase() >> {32 if f['spr'] == 269 else 0});"]
    if op == "mfmsr":
        return [f"c.r[{f['D']}] = c.msr;"]
    if op == "mtmsr":
        return [f"c.msr = c.r[{f['S']}];"]
    if op == "mfcr":
        return [f"c.r[{f['D']}] = mfcr(c);"]
    if op == "mtcrf":
        return [f"mtcrf(c, {_h(f['CRM'])}, c.r[{f['S']}]);"]
    if op == "mcrf":
        d, s = f["crfD"] * 4, f["crfS"] * 4
        return ["{ " + " ".join(f"c.cr[{d + k}] = c.cr[{s + k}];" for k in range(4)) + " }"]
    if op == "mcrxr":
        return [f"{{ cr_set(c, {f['crfD']}, c.xer_so, c.xer_ov, c.xer_ca, 0); c.xer_so = c.xer_ov = c.xer_ca = 0; }}"]
    if op in _CROPS:
        e = _CROPS[op]
        e = e.replace("a", "\0A").replace("b", "\0B")
        e = e.replace("\0A", f"c.cr[{f['crbA']}]").replace("\0B", f"c.cr[{f['crbB']}]")
        return [f"c.cr[{f['crbD']}] = (uint8_t)(({e}) & 1);"]
    if op in ("mtsr", "mfsr"):
        return [f"c.sr[{f['SR']}] = c.r[{f['S']}];" if op == "mtsr" else f"c.r[{f['D']}] = c.sr[{f['SR']}];"]
    if op in ("mtsrin", "mfsrin"):
        return [f"c.sr[c.r[{f['B']}] >> 28] = c.r[{f['S']}];" if op == "mtsrin"
                else f"c.r[{f['D']}] = c.sr[c.r[{f['B']}] >> 28];"]
    if op == "sc":
        return [f"ppc_syscall(c, {_h(addr)});"]
    if op in ("rfi", "eciwx", "ecowx"):
        return [f'ppc_unimplemented(c, {_h(addr)}, "{op}");' + (" return;" if op == "rfi" else "")]

    # ---- floating point ---------------------------------------------------------------
    if op in _FP2 or op in _FPS or op in _FPMOVE:
        D = f["D"]
        load = f"double a = c.f[{f.get('A', 0)}], b = c.f[{f.get('B', 0)}], cc = c.f[{f.get('C', 0)}]; (void)a; (void)cc;"
        if op in _FP2:
            s = f"c.f[{D}] = {_FP2[op]};"
        elif op in _FPS:
            s = f"fp_single(c, {D}, {_FPS[op]});"
        else:
            load = f"double b = c.f[{f['B']}];"
            s = f"c.f[{D}] = {_FPMOVE[op]};"
        return ["{ " + load + " " + s + (" cr1_rc(c);" if rc else "") + " }"]
    if op in ("fcmpu", "fcmpo"):
        return [f"fcmp(c, {f['crfD']}, c.f[{f['A']}], c.f[{f['B']}]);"]
    if op in ("fctiw", "fctiwz"):
        return [f"c.f[{f['D']}] = fctiw_bits(c, c.f[{f['B']}], {str(op == 'fctiwz').lower()});" + (" cr1_rc(c);" if rc else "")]
    if op == "mffs":
        return [f"c.f[{f['D']}] = as_f64(0xFFF8000000000000ull | c.fpscr);" + (" cr1_rc(c);" if rc else "")]
    if op == "mtfsf":
        return [f"mtfsf(c, {_h(f['FM'])}, c.f[{f['B']}]);" + (" cr1_rc(c);" if rc else "")]
    if op == "mtfsfi":
        sh = 28 - 4 * f["crfD"]
        return [f"c.fpscr = (c.fpscr & ~{_h(0xF << sh)}) | {_h(f['IMM'] << sh)};" + (" cr1_rc(c);" if rc else "")]
    if op in ("mtfsb0", "mtfsb1"):
        bit = _h(0x80000000 >> f["crbD"])
        s = f"c.fpscr &= ~{bit};" if op == "mtfsb0" else f"c.fpscr |= {bit};"
        return [s + (" cr1_rc(c);" if rc else "")]
    if op == "mcrfs":
        sh = 28 - 4 * f["crfS"]
        return [f"{{ uint32_t v = c.fpscr >> {sh}; cr_set(c, {f['crfD']}, v >> 3 & 1, v >> 2 & 1, v >> 1 & 1, v & 1); }}"]

    # ---- paired singles -------------------------------------------------------------
    if op in _PS or op in _PSMOVE:
        A, B, C, D = f.get("A", 0), f.get("B", 0), f.get("C", 0), f["D"]
        load = (f"double a0 = c.f[{A}], a1 = c.ps1[{A}], b0 = c.f[{B}], b1 = c.ps1[{B}], "
                f"c0 = c.f[{C}], c1 = c.ps1[{C}]; (void)a0; (void)a1; (void)b0; (void)b1; (void)c0; (void)c1;")
        if op in _PS:
            e0, e1 = _PS[op]
            s = f"ps_set(c, {D}, {e0}, {e1});"
        else:
            e0, e1 = _PSMOVE[op]
            s = f"c.f[{D}] = {e0}; c.ps1[{D}] = {e1};"
        return ["{ " + load + " " + s + (" cr1_rc(c);" if rc else "") + " }"]
    if op in ("ps_cmpu0", "ps_cmpo0", "ps_cmpu1", "ps_cmpo1"):
        arr = "c.f" if op.endswith("0") else "c.ps1"
        return [f"fcmp(c, {f['crfD']}, {arr}[{f['A']}], {arr}[{f['B']}]);"]
    if op in ("psq_l", "psq_lu", "psq_st", "psq_stu"):
        fn_ = "psq_load" if op.startswith("psq_l") else "psq_store"
        upd = f" c.r[{f['A']}] = ea;" if op.endswith("u") else ""
        return [f"{{ uint32_t ea = {_ea_d(f)}; {fn_}(c, {f['D']}, ea, {f['W']}, {f['I']});{upd} }}"]
    if op in ("psq_lx", "psq_lux", "psq_stx", "psq_stux"):
        fn_ = "psq_load" if op.startswith("psq_l") else "psq_store"
        upd = f" c.r[{f['A']}] = ea;" if op.endswith("ux") else ""
        return [f"{{ uint32_t ea = {_ea_x(f)}; {fn_}(c, {f['D']}, ea, {f['W']}, {f['I']});{upd} }}"]

    raise Unsupported(op)


def terminates(ins):
    """True if control never falls through to the next instruction."""
    op, f = ins.op, ins.f
    if op == "b" and not f["LK"]:
        return True
    if op in ("bclr", "bcctr") and not f["LK"] and f["BO"] & 0x14 == 0x14:
        return True
    return op == "rfi"
