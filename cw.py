"""CodeWarrior (Metrowerks) C++ name demangling.

    python -m wiikit.cw 'process__19CSongMoveBlockActorFf'
    -> CSongMoveBlockActor::process(float)

The GameCube/Wii toolchain mangles in the old cfront style:

    name__<scope>F<args>        method / function
    name__F<args>               free function
    __ct__ / __dt__             constructor / destructor
    __as__ __eq__ __vc__ ...    operators

A scope is <len><name> or Q<n> followed by n of them; a name may carry a
template argument list in <...> that is kept verbatim. Argument codes:
C const, V volatile, P pointer, R reference, U/S unsigned/signed prefix,
v i s l x c b f d w e (void int short long long-long char bool float double
wchar_t ...), A<n>_ array, F...(_ret) function, M<scope> member pointer,
<len><name> or Q.. a class.

demangle() never raises: a name it cannot parse comes back unchanged.
split() gives the parts; owner() the class a symbol belongs to.
"""
import re
import sys

_OPS = {"ct": None, "dt": None, "as": "operator=", "eq": "operator==",
        "ne": "operator!=", "lt": "operator<", "gt": "operator>",
        "le": "operator<=", "ge": "operator>=", "pl": "operator+",
        "mi": "operator-", "ml": "operator*", "dv": "operator/",
        "md": "operator%", "apl": "operator+=", "ami": "operator-=",
        "amu": "operator*=", "adv": "operator/=", "vc": "operator[]",
        "cl": "operator()", "nw": "operator new", "dl": "operator delete",
        "nwa": "operator new[]", "dla": "operator delete[]", "rf": "operator->",
        "nt": "operator!", "aa": "operator&&", "oo": "operator||",
        "ad": "operator&", "or": "operator|", "er": "operator^",
        "co": "operator~", "ls": "operator<<", "rs": "operator>>",
        "pp": "operator++", "mm": "operator--", "aad": "operator&=",
        "aor": "operator|=", "aer": "operator^=", "als": "operator<<=",
        "ars": "operator>>=", "cm": "operator,", "rm": "operator->*"}
_BASIC = {"v": "void", "i": "int", "s": "short", "l": "long", "x": "long long",
          "c": "char", "b": "bool", "f": "float", "d": "double", "r": "long double",
          "w": "wchar_t", "e": "..."}


class _P:
    def __init__(self, s):
        self.s, self.i = s, 0

    def peek(self):
        return self.s[self.i] if self.i < len(self.s) else ""

    def num(self):
        m = re.match(r"\d+", self.s[self.i:])
        if not m:
            raise ValueError
        self.i += len(m.group())
        return int(m.group())

    def ident(self):
        n = self.num()
        out = self.s[self.i:self.i + n]
        if len(out) != n:
            raise ValueError
        self.i += n
        return out

    def scope(self):
        if self.peek() == "Q":
            self.i += 1
            n = int(self.s[self.i])
            self.i += 1
            return [self.ident() for _ in range(n)]
        return [self.ident()]

    def type(self):
        quals = []
        while self.peek() in ("C", "V"):
            quals.append("const" if self.peek() == "C" else "volatile")
            self.i += 1
        c = self.peek()
        if c == "P" or c == "R":
            self.i += 1
            t = self.type() + ("*" if c == "P" else "&")
        elif c == "U" or c == "S":
            self.i += 1
            t = ("unsigned " if c == "U" else "signed ") + _BASIC[self.s[self.i]]
            self.i += 1
        elif c.isdigit() or c == "Q":
            t = "::".join(self.scope())
        elif c == "A":
            self.i += 1
            n = self.num()
            if self.peek() != "_":
                raise ValueError
            self.i += 1
            t = f"{self.type()}[{n}]"
        elif c == "F":
            self.i += 1
            args = self.args()
            ret = "void"
            if self.peek() == "_":
                self.i += 1
                ret = self.type()
            t = f"{ret} (*)({args})"
        elif c == "M":
            self.i += 1
            cls = "::".join(self.scope())
            t = f"{self.type()} {cls}::*"
        elif c in _BASIC:
            self.i += 1
            t = _BASIC[c]
        else:
            raise ValueError
        return " ".join(quals + [t]) if quals else t

    def args(self):
        out = []
        while self.i < len(self.s) and self.peek() != "_":
            out.append(self.type())
        return ", ".join("" if a == "void" and len(out) == 1 else a for a in out)


def split(sym):
    """(scope list, base name, argument string or None, const method, return type).

    The return type is only encoded for template functions; otherwise None."""
    body, lead, conv, ret = sym, "", None, None
    m = re.match(r"__([a-z]+)__(.*)", body)
    if m and m.group(1) in _OPS:            # __ct__..., __as__...
        lead, body = m.group(1), "__" + m.group(2)
        cut = 0
    elif body.startswith("__op"):           # __op<type>__<scope>: conversion
        p = _P(body[4:])
        conv = p.type()
        if not body[4 + p.i:].startswith("__"):
            raise ValueError
        lead, body, cut = "op", body[4 + p.i:], 0
    else:
        cut = None
        for m in re.finditer(r"__(?=[0-9QF])", body):
            if m.start() > 0 and not (body.startswith("__") and m.start() < 2):
                cut = m.start()
                break
        if cut is None:
            return [], sym, None, False, None
    name = body[:cut]
    p = _P(body[cut + 2:])
    scope = [] if p.peek() == "F" else p.scope()
    const = False
    if p.peek() == "C":
        const = True
        p.i += 1
    args = None
    if p.peek() == "F":
        p.i += 1
        args = p.args()
        if p.peek() == "_":                 # template functions encode the return type
            p.i += 1
            ret = p.type()
    if p.i != len(p.s):
        raise ValueError
    if lead:
        cls = scope[-1].split("<")[0] if scope else "?"
        name = (cls if lead == "ct" else "~" + cls if lead == "dt" else
                f"operator {conv}" if lead == "op" else _OPS.get(lead) or f"operator {lead}")
    return scope, name, args, const, ret


def demangle(sym):
    try:
        scope, name, args, const, ret = split(sym)
    except (ValueError, IndexError, KeyError):
        return sym
    if args is None and not scope:
        return sym
    out = "::".join(scope + [name])
    if ret is not None:
        out = f"{ret} {out}"
    if args is not None:
        out += f"({args})" + (" const" if const else "")
    return out


def owner(sym):
    """The class (innermost scope) a symbol belongs to, or ''."""
    try:
        scope = split(sym)[0]
    except (ValueError, IndexError, KeyError):
        return ""
    return scope[-1] if scope else ""


if __name__ == "__main__":
    for s in sys.argv[1:]:
        print(demangle(s))
