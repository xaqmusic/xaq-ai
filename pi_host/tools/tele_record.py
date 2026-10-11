#!/usr/bin/env python3
"""tele_record — record benchd telemetry OFF the robot, onto a machine that cannot die with it.

WHY THIS IS NOT ON THE PI.  Twice a brownout has destroyed the record of the brownout.
`/tmp` is tmpfs and lost everything (BOM §3.10); moving the log to `$HOME` was not enough
either, because `fflush()` only reaches the kernel and an unclean power cut drops whatever
the page cache still held -- on 2026-09-07 a stalled leg took the machine down and the last
surviving record was from BEFORE the run, so the one measurement that would have shown the
stall current died with the machine that made it.

The obvious fix -- fsync the record -- was tried and MEASURED on 2026-09-08, and it is not
available: `benchd`'s `record()` runs under the mutex the 50 Hz servo tick needs, and at a
1 s cadence an SD fsync costs ~80 ms.  The loop fell to 35.8 Hz with 54 overruns in 12 s.
Durability bought with the control loop is not a trade that daemon may make.

So the record lives on ANOTHER MACHINE.  `benchd` already PUBs every frame at 10 Hz; this
subscribes and writes to a disk that is not on the thing that browns out.  It costs the
robot nothing -- no extra I/O, no extra thread, no shared fate.

⚠ WHAT IT STILL CANNOT CATCH.  It records what arrived.  A hard power cut still loses
whatever was in the Pi's TCP buffer (benchd sets ZMQ SNDHWM 4), so expect to lose the last
frames rather than the last seconds -- better, not perfect.  And a network drop is NOT the
robot going quiet: `seq` is monotonic from the daemon, so every gap is detected and written
as an explicit `gap` record.  A silent hole that a reader mistakes for a still robot is the
one failure this tool must not have.

    python3 pi_host/tools/tele_record.py [--host picrawler.local] [--out DIR] [--label NAME]
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time

try:
    import zmq
except ImportError:
    sys.exit("tele_record needs pyzmq:  pip install pyzmq")

running = True


def _stop(*_):
    global running
    running = False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="picrawler.local", help="robot hostname (mDNS)")
    ap.add_argument("--port", type=int, default=5591, help="benchd PUB port")
    ap.add_argument("--out", default=".", help="directory for the JSONL")
    ap.add_argument("--label", default="", help="tag folded into the filename")
    ap.add_argument("--fsync-s", type=float, default=2.0,
                    help="how often to force the file onto media (cheap here: no control loop)")
    a = ap.parse_args()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"tele_{stamp}{('_' + a.label) if a.label else ''}.jsonl"
    path = os.path.join(a.out, name)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{a.host}:{a.port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 1000)

    n = gaps = lost = 0
    last_seq = None
    last_sync = 0.0
    print(f"tele_record -> {path}   (subscribed to {a.host}:{a.port}; Ctrl-C to stop)")

    with open(path, "w") as f:
        f.write(json.dumps({"kind": "start", "host": a.host, "port": a.port,
                            "label": a.label, "utc": stamp}) + "\n")
        while running:
            try:
                msg = sub.recv()
            except zmq.Again:
                continue                      # no frame this second; the loop below reports it
            except zmq.ZMQError:
                break
            try:
                frame = json.loads(msg.split(b" ", 1)[1].decode())
            except (ValueError, IndexError):
                continue

            seq = frame.get("seq")
            # A GAP IS DATA.  seq is monotonic from benchd, so a jump means frames existed
            # that we did not receive -- almost always the network, not the robot. Written
            # explicitly so nothing downstream reads a hole as a quiet robot.
            if last_seq is not None and isinstance(seq, int) and seq > last_seq + 1:
                missed = seq - last_seq - 1
                gaps += 1
                lost += missed
                f.write(json.dumps({"kind": "gap", "after_seq": last_seq,
                                    "next_seq": seq, "missed": missed,
                                    "pc_wall": time.time()}) + "\n")
            last_seq = seq if isinstance(seq, int) else last_seq

            # pc_wall rides alongside the daemon's own t_mono_ms: two clocks, so a
            # network stall is distinguishable from the robot's own timeline stalling.
            f.write(json.dumps({"kind": "frame", "pc_wall": time.time(), "frame": frame}) + "\n")
            n += 1

            now = time.monotonic()
            if now - last_sync >= a.fsync_s:
                f.flush()
                os.fsync(f.fileno())          # free here: nothing in this process is a control loop
                last_sync = now
            if n % 100 == 0:
                print(f"\r  {n} frames, {gaps} gaps ({lost} frames missed)", end="", flush=True)

        f.write(json.dumps({"kind": "stop", "frames": n, "gaps": gaps,
                            "missed": lost, "pc_wall": time.time()}) + "\n")
        f.flush()
        os.fsync(f.fileno())

    print(f"\n{n} frames, {gaps} gaps ({lost} frames missed) -> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
