#!/usr/bin/env python3
"""Fake operator — stream moving wrist poses over UDP to the Unity TeleopController,
so the arms move with NO Quest and NO bridge. Same wire format as orbit_to_unity.py.

Run it WHILE Unity is in Play mode (TeleopController listening on :9101):
    python3 Tools/fake_operator.py

Both wrists trace a slow circle. The controller auto-calibrates on the first frame
(anchoring at the arms' current pose), then the arms follow the circle via IK — so
you should see both arms gently orbit around their home pose. Ctrl+C to stop.
"""
from __future__ import annotations

import argparse
import math
import socket
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9101)
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--radius", type=float, default=0.10, help="circle radius (m)")
    ap.add_argument("--hands", action="store_true", help="also stream open/close hand keypoints")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.host, args.port)
    t0 = time.time()
    print(f"streaming fake wrists{' + hands' if args.hands else ''} -> "
          f"{args.host}:{args.port}  (Ctrl+C to stop)")
    n = 0
    while True:
        t = time.time() - t0
        r = args.radius
        for side, sgn in (("right", -1.0), ("left", 1.0)):
            px = 0.15 * sgn + r * math.sin(t * 0.8)
            py = 0.10 + r * math.cos(t * 1.0)
            pz = -0.30 + r * 0.5 * math.sin(t * 1.2)
            sock.sendto(f"W,{side},{px:.5f},{py:.5f},{pz:.5f},0,0,0,1".encode(), dst)
            if args.hands:
                sock.sendto(_hand_msg(side, 0.5 - 0.5 * math.cos(t * 1.5)).encode(), dst)
        n += 1
        if n % int(args.rate) == 0:
            print(f"  t={t:5.1f}s  sent {n} frames")
        time.sleep(1.0 / args.rate)


def _hand_msg(side: str, curl: float) -> str:
    """26 ORBIT keypoints (wrist, palm, then the WebXR finger chains) that open/close
    with `curl`. Counts match ORBIT: thumb 4 joints, index/middle/ring/pinky 5 each."""
    pts = [(0.0, 0.0, 0.0), (0.0, 0.01, 0.0)]          # 0 wrist, 1 palm (dropped by C#)
    seg = 0.03
    # (base x across palm, n joints): thumb(4), index/middle/ring/pinky(5)
    fingers = [(0.03, 4), (0.02, 5), (0.0, 5), (-0.02, 5), (-0.04, 5)]
    for bx, nj in fingers:
        x, y, z, ang = bx, 0.02, 0.0, 0.0
        for _ in range(nj):
            ang += curl * 1.1
            y += seg * math.cos(ang)
            z += -seg * math.sin(ang)
            pts.append((x, y, z))
    return f"H,{side}," + ",".join(f"{x:.4f},{y:.4f},{z:.4f}" for x, y, z in pts)


if __name__ == "__main__":
    raise SystemExit(main())
