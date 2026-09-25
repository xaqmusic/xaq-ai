#!/usr/bin/env python3
"""What does the guard's OWN RESPONSE cost the rail it is protecting?

⚠ THE PREMISE THIS CHECKS.  benchd's comment says the guard drops the load because "the
servos ARE the load".  It cannot.  There is no limp on this HAT once it is initialized —
the servos are energized and holding no matter what — so rescue() does not de-energize
anything.  It calls begin_pose_move() on twelve channels, which is ADDED current, and a
rescue recall is precisely what took the Pi down on 2026-08-29.

So the guard's response to a sagging rail may be a transient of the same kind that caused
the sag.  This measures it: holding current in `stand`, then the peak drawn while the
guard's rescue move runs.

⚠ THE ROBOT MOVES — stand, then the full rescue recall.
"""
import glob, json, os, statistics as st, sys, time, zmq

ENDPOINT, LOG_DIR = "tcp://127.0.0.1:5590", os.path.expanduser("~/xaq-ai/pi_host/log")
_ctx = zmq.Context()

def rpc(verb, _allow_err=False, **kw):
    """⚠ RAISES on ok:false unless _allow_err.  This is not defensive style, it is the fix for
    the defect that invalidated four measurements: move() called rpc("pose.set", name=...) and
    DISCARDED the reply.  pose.set takes `us` as an array of 12 and has never accepted a name,
    so every call returned {"ok":false,"error":"us must be an array of 12"} and the robot never
    moved -- through a separation test and three ceiling sweeps, all of which reported clean
    results measured on a stationary robot.  A harness that ignores error replies cannot detect
    that it is doing nothing."""
    s = _ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 25000); s.setsockopt(zmq.LINGER, 0)
    s.connect(ENDPOINT); s.send_string(json.dumps({"verb": verb, **kw}))
    try:    r = json.loads(s.recv_string())
    except zmq.Again: r = {"ok": False, "error": "TIMEOUT"}
    finally: s.close()
    if not r.get("ok") and not _allow_err:
        raise RuntimeError(f"{verb} failed: {r.get('error')}  (sent {kw})")
    return r

_pose_cache = {}
def pose_us(name):
    """Named poses are recalled by VALUE: there is no recall-by-name verb."""
    if name not in _pose_cache: _pose_cache[name] = rpc("pose.get", name=name)["us"]
    return _pose_cache[name]

def settle(timeout=25):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = rpc("status")
        if not s.get("pose_move_active") and not s.get("pose_queue"): return True
        time.sleep(0.2)
    return False

path = sorted(glob.glob(os.path.join(LOG_DIR, "benchd_*.jsonl")))[-1]
def trace(pos):
    """(t_mono_ms, i_a, vbat) from telemetry records after `pos`, plus new file position."""
    out = []
    with open(path) as fh:
        fh.seek(pos)
        for line in fh:
            try: r = json.loads(line)
            except Exception: continue
            if r.get("kind") == "telemetry":
                d = r["data"]; ina = d.get("ina") or {}
                if ina.get("ok"): out.append((r["t_mono_ms"], ina["i_a"], d["vbat"]))
        return out, fh.tell()

s0 = rpc("status")
print(f"start: vbat={s0['vbat']:.2f}  rail_events={s0['rail_events']}  mask={s0['pi_throttled']}")
if s0["vbat"] < 7.0: sys.exit(f"vbat {s0['vbat']:.2f} too low for a clean current measurement")

print("-> stand ..."); rpc("pose.set", us=pose_us("stand"))
if not settle(): sys.exit("stand did not settle")
time.sleep(1.0)

pos = os.path.getsize(path)
print("holding in stand for 4 s ...");  time.sleep(4.0)
hold, pos = trace(pos)

base = rpc("status").get("rail_baseline", 0)
bit  = 0x40000 if (base & 0x10000) else 0x10000
print(f"-> injecting 0x{bit:X}; the guard should recall rescue ...")
ev0 = rpc("status")["rail_events"]
rpc("rail.inject", bits=bit, confirm=True)
# ⚠ Wait for the guard to FIRE before waiting for the move to finish.  settle() called
# straight after the inject returns True immediately -- the guard has not fired yet, so
# pose_move_active is still false and "settled" means "has not started".  The first run of
# this script captured 0.9 s that way and reported the response drawing LESS than holding.
t0 = time.time()
while time.time() - t0 < 5.0:
    if rpc("status")["rail_events"] > ev0: break
    time.sleep(0.02)
else: sys.exit("the guard never fired")
t0 = time.time()
while time.time() - t0 < 30.0:                 # now follow the recall to its end
    s_ = rpc("status")
    if not s_.get("pose_move_active") and not s_.get("pose_queue") and not s_.get("rescue_active"): break
    time.sleep(0.2)
time.sleep(0.5)
resp, pos = trace(pos)
rpc("rail.inject", bits=0, confirm=True)

if not hold or not resp: sys.exit("no INA219 samples — is the current sensor live?")
hi = [i for _, i, _ in hold]; ri = [i for _, i, _ in resp]
hv = [v for _, _, v in hold]; rv = [v for _, _, v in resp]
print(f"\n  holding in stand : mean {st.mean(hi):.3f} A   max {max(hi):.3f}   vbat min {min(hv):.2f}  n={len(hi)}")
print(f"  guard's response : mean {st.mean(ri):.3f} A   max {max(ri):.3f}   vbat min {min(rv):.2f}  n={len(ri)}")
print(f"\n  the response added {max(ri)-max(hi):+.3f} A of PEAK over holding "
      f"({max(ri)/max(hi):.2f}x) and dipped the pack {min(hv)-min(rv):+.3f} V further")
print("\n⚠ Read against 2026-08-29: a rescue recall at 2000 us/s took the Pi down in 0.5 s.")
print("   This recall is the STAGGERED, 600 us/s one, which is the mitigation already in place.")
