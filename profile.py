"""Name the addresses wiiboot's sampling profiler reports (WIIKIT_PROFILE=1).

    python -m wiikit.profile build/recomp-build/wiiboot.exe run.err [build/symbols.tsv]

Reads the last block of "profile-raw: ADDR COUNT [DLL]" lines, finds the
function holding each address with `nm` (host symbols, runtime included),
turns recompiled functions (f_XXXXXXXX) into guest names, and sums per
function. A line with a DLL is a sample taken inside that DLL, charged to
the executable's function that called into it.
"""
import bisect
import collections
import subprocess
import sys


def main():
    exe, log = sys.argv[1], sys.argv[2]
    guest = {}
    if len(sys.argv) > 3:
        for line in open(sys.argv[3], encoding="utf-8"):
            p = line.rstrip("\n").split("\t")
            if len(p) >= 7 and p[0] != "addr":
                guest.setdefault(int(p[0], 16), p[6])
    out = subprocess.run(["nm", "-C", "--defined-only", exe], capture_output=True, text=True).stdout
    syms = []
    for line in out.splitlines():
        p = line.split(" ", 2)
        if len(p) == 3 and p[1] in "tT":
            syms.append((int(p[0], 16), p[2]))
    syms.sort()
    addrs = [a for a, _ in syms]
    blocks, cur = [], []
    for line in open(log, encoding="utf-8", errors="replace"):
        if line.startswith("profile-raw:"):
            f = line.split()
            cur.append((int(f[1], 16), int(f[2]), f[3] if len(f) > 3 else ""))
        elif cur:
            blocks.append(cur)
            cur = []
    if cur:
        blocks.append(cur)
    if not blocks:
        sys.exit("no profile-raw lines")
    per = collections.Counter()
    total = 0
    for a, n, dll in blocks[-1]:
        i = bisect.bisect_right(addrs, a) - 1
        name = syms[i][1] if i >= 0 else "?"
        if name.startswith("f_") and "(" in name:
            g = int(name[2:10], 16)
            name = f"{guest.get(g, name)}  [guest {g:08X}]"
        if dll:
            name += f"  -> in {dll}"
        per[name] += n
        total += n
    for name, n in per.most_common(40):
        print(f"{100.0 * n / total:5.1f}%  {name}")


if __name__ == "__main__":
    main()
