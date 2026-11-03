#!/usr/bin/env python3
"""Align two command logs and report the first divergence.

Input format: one command per line, e.g.

    03-10 04:57:42 RX=[245] [-165] [200]
    03-10 04:57:48 RX=[CLAW] [530] []

Usage:
    python diff_cmdlog.py good.log regressed.log
"""
import re
import sys

LINE = re.compile(r"RX=\[(\S*)\]\s*\[(\S*)\]\s*\[(\S*)\]")


def load(path):
    out = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = LINE.search(line)
            if not m:
                continue
            a, b, c = m.groups()
            if a == "OSD":            # drop display traffic
                continue
            out.append(" ".join(x for x in (a, b, c) if x))
    return out


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    good, bad = load(sys.argv[1]), load(sys.argv[2])

    n = min(len(good), len(bad))
    for i in range(n):
        if good[i] != bad[i]:
            print("first divergence at command %d" % (i + 1))
            print("  good: %s" % good[i])
            print("  bad : %s" % bad[i])
            lo = max(0, i - 3)
            print("\ncontext:")
            for j in range(lo, min(n, i + 2)):
                mark = ">>" if j == i else "  "
                print("%s %-28s | %s" % (mark, good[j], bad[j]))
            return 0

    if len(good) == len(bad):
        print("no divergence in %d commands" % n)
    else:
        print("streams diverge only in length: %d vs %d" % (len(good), len(bad)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
