#!/usr/bin/env python3
"""ogma_ctl — benchd's CONTROL channel, from a shell ON THE ROBOT.

The run mode decides whether a brain may drive the servos (pi_host/PROTOCOL.md "Run
modes", port doc SPEC §4.2.1).  It is set here, on the robot, over benchd's loopback-only
control socket (--ctl-port) -- never from the calibration channel the dashboards speak.

    ogma_ctl.py                    # mode, STOP state, brain stream
    ogma_ctl.py mode dev           # bench | dev | autonomous   (a brain mode starts STOPPED)
    ogma_ctl.py stop               # freeze every servo where it is
    ogma_ctl.py resume             # hand the servos back (the dashboards' SPACE does this too)

Entering dev or autonomous LATCHES STOP: the brain gets the servos only when someone
resumes, watching.  Leaving to bench freezes the body and starts the bench deadman.
"""
import argparse
import json
import sys

import zmq


def main() -> int:
    p = argparse.ArgumentParser(description="benchd control channel (loopback only)")
    p.add_argument("--port", type=int, default=5593, help="benchd --ctl-port (default 5593)")
    p.add_argument("verb", nargs="?", default="mode.get", choices=["mode.get", "mode", "stop", "resume", "status"])
    p.add_argument("mode", nargs="?", help="with 'mode': bench | dev | autonomous")
    a = p.parse_args()

    req = {"verb": a.verb}
    if a.verb == "mode":
        if not a.mode:
            req = {"verb": "mode.get"}
        else:
            req = {"verb": "mode.set", "mode": a.mode}
    s = zmq.Context.instance().socket(zmq.REQ)
    s.setsockopt(zmq.RCVTIMEO, 1500)
    s.setsockopt(zmq.LINGER, 0)
    # Loopback by construction: benchd binds this socket to 127.0.0.1 only.
    s.connect(f"tcp://127.0.0.1:{a.port}")
    s.send_string(json.dumps(req))
    try:
        r = json.loads(s.recv_string())
    except zmq.Again:
        print(f"no reply on 127.0.0.1:{a.port} — is benchd running with --ctl-port {a.port}?", file=sys.stderr)
        return 1
    if a.verb == "status":
        print(json.dumps(r, indent=1))
    elif not r.get("ok"):
        print(f"refused: {r.get('error')}", file=sys.stderr)
        return 1
    else:
        stop = f"STOPPED ({r.get('stop_why')})" if r.get("stopped") else "running"
        print(f"mode {r.get('mode')}   {stop}   brain: {r.get('brain_frames')} frames, "
              f"{r.get('brain_applied')} applied, age {r.get('brain_age_ms')} ms"
              + ("   HOLDING (stream lost)" if r.get("holding") else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
