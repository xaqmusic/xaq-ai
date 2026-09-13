#!/usr/bin/env python3
"""How LATE is the rail guard? — detection latency at random poll phase.

The pass/fail drill proves the guard is wired up.  It does not say how long the rail stays
loaded after the bit appears, and that is the whole question for a fast collapse: the
2026-08-29 brownout took the Pi down 0.5 s after pose.set.  A guard that needs a second to
notice is a guard that never runs in that case.

Injects at a RANDOM offset so the sample covers the full poll phase, and reads both
timestamps from the daemon's own JSONL — its monotonic clock, not the RPC round trip.

⚠ THE ROBOT MOVES: every trial commands the rescue pose.
"""
import glob, json, os, random, statistics as st, sys, time, zmq

ENDPOINT, LOG_DIR = "tcp://127.0.0.1:5590", os.path.expanduser("~/xaq-ai/pi_host/log")
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
_ctx = zmq.Context()

def rpc(verb, **kw):
    s = _ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 8000); s.setsockopt(zmq.LINGER, 0)
    s.connect(ENDPOINT); s.send_string(json.dumps({"verb": verb, **kw}))
    try:    return json.loads(s.recv_string())
    except zmq.Again: return {"ok": False, "error": "TIMEOUT"}
    finally: s.close()

def logfile():
    f = sorted(glob.glob(os.path.join(LOG_DIR, "benchd_*.jsonl")))
    if not f: sys.exit("no benchd JSONL — is the daemon logging?")
    return f[-1]

def scan(path, pos):
    """New rail_inject / rail_undervolt records since `pos`, with the daemon's own clock."""
    out = []
    with open(path) as fh:
        fh.seek(pos)
        for line in fh:
            try: r = json.loads(line)
            except Exception: continue
            if r.get("kind") in ("rail_inject", "rail_undervolt"):
                out.append((r["kind"], r["t_mono_ms"], r.get("data", {})))
        return out, fh.tell()

st0 = rpc("status")
if not st0.get("ok"): sys.exit(f"daemon not answering: {st0}")
print(f"n={N}  poll=~{1000} ms nominal  starting mask={st0['pi_throttled']} "
      f"baseline=0x{st0.get('rail_baseline',0):X}")

path = logfile(); pos = os.path.getsize(path)
lat, misses = [], 0
for i in range(N):
    time.sleep(random.uniform(0.0, 1.0))          # random phase against the poll
    base = rpc("status").get("rail_baseline", 0)
    bit  = 0x40000 if (base & 0x10000) else 0x10000
    rpc("rail.inject", bits=bit, confirm=True)
    t_wait = time.time()
    while time.time() - t_wait < 4.0:             # wait for the guard to fire
        if rpc("status")["rail_guarded"]: break
        time.sleep(0.02)
    time.sleep(0.3)                               # let the records land
    recs, pos = scan(path, pos)
    inj = [t for k, t, _ in recs if k == "rail_inject"]
    evt = [t for k, t, _ in recs if k == "rail_undervolt"]
    if inj and evt: lat.append(evt[-1] - inj[-1]); print(f"  trial {i+1:2d}/{N}  {lat[-1]:5.0f} ms")
    else:           misses += 1; print(f"  trial {i+1:2d}/{N}  NO EVENT")
    rpc("rail.inject", bits=0, confirm=True)
    while rpc("status")["rail_guarded"]: time.sleep(0.2)   # let the back-off clear

if not lat: sys.exit("no latencies captured")
lat.sort()
print(f"\ndetection latency over n={len(lat)} (misses {misses}):")
print(f"  min {lat[0]:.0f}   median {st.median(lat):.0f}   p95 {lat[int(len(lat)*.95)]:.0f}   max {lat[-1]:.0f} ms")
print(f"\n⚠ the 2026-08-29 fast brownout took the Pi down 500 ms after pose.set.")
print(f"   trials slower than that: {sum(1 for x in lat if x > 500)}/{len(lat)}")
