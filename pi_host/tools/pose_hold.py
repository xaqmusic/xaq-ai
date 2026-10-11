#!/usr/bin/env python3
"""Command a saved pose by VALUE and HOLD it, so the operator can work on the robot.

⚠ THIS EXISTS BECAUSE OF THE DEADMAN.  `benchd`'s DEADMAN_MS is 1000: once any channel is
armed, a second with no fresh client verb and it commands the saved RESCUE pose.  This HAT
cannot limp a servo at all, so "safe" is a pose, not slack (calib/sensors.json, servo note).
Holding `stand` while someone slides a shim under the feet therefore needs a keepalive, and
the keepalive is NOT optional -- without it the robot folds mid-placement.

⚠ ON EXIT IT DOES NOT LIMP.  It simply stops pinging, and the deadman pulls to rescue ~1 s
later.  That is the safe default and it is deliberate: there is no exit path that leaves the
robot armed and unattended.

⚠ POSES ARE RECALLED BY VALUE.  `pose.set` has never accepted a name -- `pose.get` first.

Concurrent clients are fine: benchd serves one REP socket in order, so a measurement script
polling at 10 Hz alongside this at 5 Hz just interleaves, and BOTH feed the deadman.

  python3 pose_hold.py stand              # command it, then hold until Ctrl-C
  python3 pose_hold.py stand --no-move    # hold whatever it is already doing
  touch /tmp/pose_hold.stop               # ask it to exit from another shell
"""
import argparse, json, math, os, signal, sys, time, zmq

ENDPOINT  = "tcp://127.0.0.1:5590"
STOP_FILE = "/tmp/pose_hold.stop"
_ctx = zmq.Context()
_sock = None

def sock():
    global _sock
    if _sock is None:
        _sock = _ctx.socket(zmq.REQ)
        _sock.setsockopt(zmq.RCVTIMEO, 8000)
        _sock.setsockopt(zmq.LINGER, 0)
        _sock.connect(ENDPOINT)
    return _sock

def rpc(verb, **kw):
    s = sock()
    s.send_string(json.dumps({"verb": verb, **kw}))
    try:
        r = json.loads(s.recv_string())
    except zmq.Again:
        # A REQ socket is dead after a timeout; drop it so the next ping re-dials.
        global _sock
        _sock.close(); _sock = None
        return {"ok": False, "error": "TIMEOUT"}
    if not r.get("ok"):
        raise RuntimeError(f"{verb} failed: {r.get('error')}  (sent {kw})")
    return r

_running = True
def _stop(*_):
    global _running
    _running = False
signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT, _stop)

ap = argparse.ArgumentParser()
ap.add_argument("pose", help="name of a saved pose (pose.list)")
ap.add_argument("--no-move", action="store_true", help="hold the current position, command nothing")
ap.add_argument("--every", type=float, default=2.0, help="seconds between status lines")
ap.add_argument("--hz", type=float, default=5.0, help="keepalive rate; must stay well inside DEADMAN_MS=1000")
a = ap.parse_args()

if os.path.exists(STOP_FILE): os.remove(STOP_FILE)

if not a.no_move:
    us = rpc("pose.get", name=a.pose)["us"]
    print(f"pose '{a.pose}' = {us}", flush=True)
    r = rpc("pose.set", us=us)
    print(f"pose.set -> staggered={r.get('staggered')} eta_ms={r.get('eta_ms')}", flush=True)

print(f"HOLDING at {a.hz:.0f} Hz.  Ctrl-C, SIGTERM, or `touch {STOP_FILE}` to release "
      f"(the deadman takes it to rescue ~1 s later).", flush=True)

period, t_next_print, warned_rescue = 1.0 / a.hz, 0.0, False
while _running and not os.path.exists(STOP_FILE):
    now = time.time()
    if now >= t_next_print:
        t_next_print = now + a.every
        try:
            s = rpc("status")
        except RuntimeError as e:
            print(f"  status failed: {e}", flush=True); time.sleep(period); continue
        t, imu = s.get("tof") or {}, s.get("imu") or {}
        u = imu.get("up_fused") or [0, 1, 0]
        u = list(u) if isinstance(u, (list, tuple)) else [u.get("x",0), u.get("y",1), u.get("z",0)]
        pitch = math.degrees(math.atan2(-u[2], u[1]))
        roll  = math.degrees(math.atan2(u[0], u[1]))
        mv    = "MOVING" if s.get("pose_move_active") or s.get("pose_queue") else "held  "
        print(f"  {mv}  vbat {s.get('vbat',0):5.2f} V  i {(s.get('ina') or {}).get('i_a',0):5.3f} A  "
              f"5V {s.get('ext5v',0):5.3f}  thr {s.get('pi_throttled')}  |  "
              f"pitch {pitch:+6.2f}°  roll {roll:+6.2f}°  |  "
              f"raw {t.get('raw_mm',0):3d} mm  m {1000*(t.get('m') or 0):6.2f}  "
              f"m_comp {1000*(t.get('m_comp') or 0):6.2f}  d {1000*(t.get('comp_delta') or 0):+6.2f} mm  "
              f"{'valid' if t.get('comp_valid') else '⚠ COMP_INVALID'}  bad {t.get('bad_frac',0):.2f}",
              flush=True)
        if s.get("rescue_active") and not warned_rescue:
            warned_rescue = True
            print("  ⚠⚠ RESCUE IS ACTIVE — the deadman fired. The pose you think you are "
                  "measuring is not the one the robot is in.", flush=True)
        if not s.get("rescue_active"): warned_rescue = False
    else:
        try: rpc("ping")
        except RuntimeError: pass
    time.sleep(period)

print("released — the deadman will command rescue within ~1 s.", flush=True)
