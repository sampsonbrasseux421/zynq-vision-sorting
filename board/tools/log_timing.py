#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
log_timing.py -- measure real command round trips and sorting cycles.

The board is driven exactly as the production programs drive it: one line of
ASCII per TCP connection, reply read to EOF. Timestamps bracket the whole
round trip, so the numbers include network and PS processing -- that is the
latency a user actually sees.

Usage
-----
    python log_timing.py --moves 10 --out ../measured/move_log.csv
    python log_timing.py --cycle 10 --out ../measured/cycle_log.csv

Requires nothing but the Python standard library.
"""
from __future__ import annotations

import argparse
import csv
import os
import socket
import sys
import time

BOARD_IP = "192.168.1.10"
BOARD_PORT = 5000
TIMEOUT = 8.0

# A short, safe move used for the "one four-axis move" measurement.
# Adjust to a point you know is reachable on your own rig.
TEST_POINT = (250, 0, 150)
HOME_POINT = (200, 0, 200)


def send(cmd: str) -> tuple[str, float]:
    """Send one command, return (last reply line, elapsed seconds)."""
    t0 = time.perf_counter()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(TIMEOUT)
    try:
        s.connect((BOARD_IP, BOARD_PORT))
        s.sendall((cmd + "\n").encode("ascii"))
        s.shutdown(socket.SHUT_WR)
        chunks = []
        while True:
            b = s.recv(4096)
            if not b:
                break
            chunks.append(b)
    finally:
        s.close()
    dt = time.perf_counter() - t0
    reply = b"".join(chunks).decode("ascii", "replace").strip()
    return (reply.splitlines()[-1] if reply else ""), dt


def preflight() -> bool:
    reply, _ = send("PING")
    if reply != "PONG":
        print("board did not answer PING (got %r)" % reply, file=sys.stderr)
        print("check: cable, 192.168.1.10, port 5000 registered in inetd.conf",
              file=sys.stderr)
        return False
    return True


def measure_moves(n: int, fast: bool):
    """Measure n identical point-to-point moves."""
    rows = []
    if fast:
        send("JOG ON")
        send("GAP 60")
    send("HOME")
    time.sleep(1.5)

    for i in range(n):
        reply, dt = send("%d %d %d" % TEST_POINT)
        ok = reply.startswith("OK")
        rows.append({
            "index": i + 1,
            "mode": "fast" if fast else "full",
            "command": "%d %d %d" % TEST_POINT,
            "reply": reply,
            "elapsed_ms": round(dt * 1000.0, 1),
            "ok": int(ok),
        })
        print("  move %2d  %7.1f ms  %s" % (i + 1, dt * 1000.0, reply[:40]))
        # return home between samples so every measurement starts identically
        send("%d %d %d" % HOME_POINT)
        time.sleep(0.3)

    if fast:
        send("JOG OFF")
        send("GAP 60")
    return rows


def measure_cycles(n: int):
    """
    Measure n full pick-and-place cycles.

    This drives the same five-command sequence the production state machine
    uses: go to the block, close the claw, carry to the box, open, return.
    Box angle and height must match your own rig -- the values below are the
    ones used in the project.
    """
    rows = []
    send("JOG ON")
    send("HOME")
    time.sleep(1.5)

    for i in range(n):
        t0 = time.perf_counter()
        steps = []
        for cmd in ("%d %d %d" % (250, 0, 90),   # above the block
                    "CLAW 530",                   # close
                    "%d %d %d" % (250, 0, 200),   # lift
                    "BOX 19200 PLACE",            # carry + descend
                    "CLAW 1150",                  # open
                    "LIFT"):                      # rise before next move
            reply, _ = send(cmd)
            steps.append("OK" if reply.startswith("OK") else reply[:20])
        dt = time.perf_counter() - t0
        rows.append({
            "index": i + 1,
            "elapsed_s": round(dt, 2),
            "steps_ok": sum(1 for s in steps if s == "OK"),
            "steps_total": len(steps),
        })
        print("  cycle %2d  %6.2f s  %d/%d steps ok"
              % (i + 1, dt, rows[-1]["steps_ok"], len(steps)))

    send("JOG OFF")
    send("GAP 60")
    return rows


def warn_if_short(rows, key, expected, what):
    vals = [r[key] for r in rows]
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    print("\n  %s: min %.1f  max %.1f  jitter %.1f"
          % (what, lo, hi, hi - lo))
    if hi - lo > expected:
        print("  WARNING: spread exceeds %.1f -- check for OS scheduling or"
              " a driver retry" % expected, file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--moves", type=int, default=0,
                    help="measure N point-to-point moves")
    ap.add_argument("--cycle", type=int, default=0,
                    help="measure N full pick-and-place cycles")
    ap.add_argument("--fast", action="store_true",
                    help="enable skip-enable fast mode while measuring moves")
    ap.add_argument("--out", default="measured/log.csv")
    args = ap.parse_args()

    if args.moves == 0 and args.cycle == 0:
        ap.error("give at least one of --moves / --cycle")

    if not preflight():
        return 1

    rows = []
    if args.moves:
        print("measuring %d moves (fast=%s)" % (args.moves, args.fast))
        rows = measure_moves(args.moves, args.fast)
        warn_if_short(rows, "elapsed_ms", 20.0, "move time (ms)")
    if args.cycle:
        print("measuring %d cycles" % args.cycle)
        rows = measure_cycles(args.cycle)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("\nwrote %d rows -> %s" % (len(rows), args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
