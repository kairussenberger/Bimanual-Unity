#!/usr/bin/env python3
"""ORBIT (NetMQ) -> Unity (UDP) bridge.

The ORBIT Quest app PUSHes operator tracking over NetMQ/ZMTP to 127.0.0.1 (adb
reverse forwards the headset's sends to this host). Unity can't read ZMTP without
the fiddly NetMQ DLL stack, so this tiny bridge does the one thing Unity can't:
bind the ORBIT PULL sockets and re-emit each newest frame as a plain, tagged UDP
datagram that Unity receives with the built-in UdpClient (zero extra DLLs).

It is a DUMB forwarder on purpose: it strips ORBIT's "relative" marker and re-tags
by channel, but does NOT convert frames or drop the palm joint. All of that
(Unity-LH -> robot, palm drop, retarget, IK) happens on the Unity/C# side
(OrbitUdpReceiver + MjFrame), mirroring the proven vr_zmq.py pipeline, so the wire
stays trivial and there is exactly one place that owns the math.

Wire OUT (UTF-8 text, one datagram per channel, to 127.0.0.1:<--out-port>):
    W,right,px,py,pz,qx,qy,qz,qw        # wrist pose, Unity XR world frame, meters, XYZW
    W,left,...
    H,right,k0x,k0y,k0z,...,k25z        # 26 hand keypoints (Unity frame); C# drops Palm[1]
    H,left,...
    HEAD,px,py,pz,qx,qy,qz,qw

Run it from any Python with pyzmq (e.g. the orca-teleop venv):
    ~/Desktop/orca-teleop/.venv/bin/python Tools/orbit_to_unity.py
"""
from __future__ import annotations

import argparse
import shutil
import socket
import subprocess
import time

import zmq

# ORBIT NetMQ ports (verified GestureDetector.cs). We BIND PULL; ORBIT PUSH-connects.
HAND_PORTS = {"right": 8087, "left": 8088}
WRIST_PORTS = {"right": 8122, "left": 8123}
HEAD_PORT = 8200
DRAIN_PORTS = (8095, 8100)          # must be bound so ORBIT's blocking PUSH never stalls
ALL_PORTS = (*HAND_PORTS.values(), *WRIST_PORTS.values(), HEAD_PORT, *DRAIN_PORTS)


def adb_reverse() -> None:
    if not shutil.which("adb"):
        print("[bridge] adb not found; set `adb reverse tcp:P tcp:P` yourself for", ALL_PORTS)
        return
    ok = 0
    for p in ALL_PORTS:
        try:
            r = subprocess.run(["adb", "reverse", f"tcp:{p}", f"tcp:{p}"],
                               capture_output=True, timeout=5)
            ok += r.returncode == 0
        except (subprocess.SubprocessError, OSError):
            pass
    print(f"[bridge] adb reverse set on {ok}/{len(ALL_PORTS)} ports")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-host", default="127.0.0.1")
    ap.add_argument("--out-port", type=int, default=9101)
    ap.add_argument("--no-adb", action="store_true", help="skip auto adb reverse")
    ap.add_argument("--debug", action="store_true", help="print per-channel forward counts")
    args = ap.parse_args()

    if not args.no_adb:
        adb_reverse()

    ctx = zmq.Context.instance()
    poller = zmq.Poller()
    socks = {}
    for side, p in HAND_PORTS.items():
        socks[_bind(ctx, p)] = ("H", side)
    for side, p in WRIST_PORTS.items():
        socks[_bind(ctx, p)] = ("W", side)
    socks[_bind(ctx, HEAD_PORT)] = ("HEAD", None)
    for p in DRAIN_PORTS:
        socks[_bind(ctx, p)] = ("drain", p)
    for s in socks:
        poller.register(s, zmq.POLLIN)

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.out_host, args.out_port)
    counts = {"W": 0, "H": 0, "HEAD": 0}
    print(f"[bridge] ORBIT NetMQ {sorted(ALL_PORTS)}  ->  UDP {args.out_host}:{args.out_port}")
    print("[bridge] launch com.ORBIT.Teleoperation on the Quest, wear it, set controllers down.")

    last = 0.0
    while True:
        events = dict(poller.poll(timeout=200))
        for sock, (kind, key) in socks.items():
            if sock not in events:
                continue
            msg = _drain(sock)
            if msg is None or kind == "drain":
                continue
            payload = _retag(kind, key, msg)
            if payload is not None:
                out.sendto(payload.encode("utf-8"), dst)
                counts[kind] += 1
        if args.debug and (time.monotonic() - last) > 2.0:
            last = time.monotonic()
            print(f"[bridge] forwarded W={counts['W']} H={counts['H']} HEAD={counts['HEAD']}")


def _retag(kind: str, key, msg: str) -> str | None:
    """ORBIT string -> tagged UDP line. Strips the 'relative' marker / ':' wrappers."""
    if kind == "W":
        t = msg.split(",")
        if len(t) != 8:
            return None
        return f"W,{key}," + ",".join(t[1:])                 # drop marker token[0]
    if kind == "HEAD":
        t = msg.split(",")
        if len(t) != 8:
            return None
        return "HEAD," + ",".join(t[1:])
    if kind == "H":
        body = msg.split(":", 1)[1] if ":" in msg else msg   # 'relative:....:'
        pts = [p for p in body.strip(":").split("|") if p.strip()]
        if not pts:
            return None
        return f"H,{key}," + ",".join(pts)                    # k0x,k0y,k0z,... flattened
    return None


def _bind(ctx, port: int):
    s = ctx.socket(zmq.PULL)
    s.bind(f"tcp://*:{port}")
    return s


def _drain(sock):
    msg = None
    while True:
        try:
            msg = sock.recv_string(zmq.NOBLOCK)
        except zmq.Again:
            return msg


if __name__ == "__main__":
    raise SystemExit(main())
