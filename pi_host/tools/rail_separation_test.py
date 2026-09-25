#!/usr/bin/env python3
"""Does the Pi's rail actually stop caring what the servos do?  (Mod A acceptance test.)

Before Mod A the HAT's one 5 V/3 A DC-DC fed the Pi AND all twelve servos, so servo load
pulled the rail the Pi lives on -- §3.8.7 measured the Pi dying from it.  After the mod the
Pi has its own BEC.  The acceptance criterion is NOT "the robot still works": it is that a
heavy servo load leaves EXT5V and get_throttled untouched.

⚠ TWO BUGS THIS FILE EXISTS TO NOT REPEAT.
 1. The first version sampled `vcgencmd pmic_read_adc` from its own thread at 20 Hz while
    benchd polled the same VideoCore mailbox.  One call wedged, subprocess cleanup blocked
    on the uninterruptible child, and the thread died silently -- ONE load sample across
    six pose cycles, reported as a clean PASS.  benchd now owns the sampling (`ext5v.rate`)
    and this script reads the records it writes.
 2. settle() was called immediately after pose.set and returned True because the move had
    not STARTED yet, so "settled" meant "has not begun".  It now waits for the start.

⚠ THE ROBOT MOVES: rescue <-> stand cycles at whatever slew the daemon is running.
"""
import glob, json, os, statistics as st, sys, time, zmq

ENDPOINT, LOG_DIR = "tcp://127.0.0.1:5590", os.path.expanduser("~/xaq-ai/pi_host/log")
CYCLES  = int(sys.argv[1]) if len(sys.argv) > 1 else 6
RATE_MS = 50
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

path = sorted(glob.glob(os.path.join(LOG_DIR, "benchd_*.jsonl")))[-1]
def ext5v_since(pos):
    out = []
    with open(path) as fh:
        fh.seek(pos)
        for line in fh:
            try: r = json.loads(line)
            except Exception: continue
            if r.get("kind") == "ext5v":
                d = r["data"]; out.append((r["t_mono_ms"], d["v"], d.get("vbat"), d.get("i_a")))
        return out, fh.tell()

def move(pose, timeout=40):
    """pose.set, then wait for the move to START and only then to finish."""
    rpc("pose.set", us=pose_us(pose))
    t0 = time.time()
    while time.time() - t0 < 3.0:                       # wait for it to BEGIN
        if rpc("status").get("pose_move_active"): break
        time.sleep(0.02)
    t0 = time.time()
    while time.time() - t0 < timeout:                   # then for it to END
        s = rpc("status")
        if not s.get("pose_move_active") and not s.get("pose_queue"): return s
        time.sleep(0.1)
    return rpc("status")

s0 = rpc("status")
if not s0.get("ok"): sys.exit(f"daemon not answering: {s0}")
r = rpc("ext5v.rate", ms=RATE_MS)
if not r.get("ok"): sys.exit(f"ext5v.rate refused: {r}  (is the daemon rebuilt?)")
print(f"start  vbat={s0['vbat']:.2f}  ext5v={s0['ext5v']}  thr={s0['pi_throttled']}  "
      f"bus_err={s0['bus_errors']}  wd={s0['watchdog_trips']}  |  sampling at {RATE_MS} ms")
try:
    pos = os.path.getsize(path)
    time.sleep(4.0)
    idle, pos = ext5v_since(pos)
    iv = [v for _, v, _, _ in idle]
    if len(iv) < 20: sys.exit(f"only {len(iv)} idle samples in 4 s — benchd is not recording ext5v")
    print(f"idle   n={len(iv)} ({len(iv)/4.0:.0f} Hz)  mean {st.mean(iv):.4f}  min {min(iv):.4f}")

    i_peak, vbat_min = 0.0, 99.0
    for c in range(CYCLES):
        for pose in ("stand", "rescue"):
            s = move(pose)
            ina = s.get("ina") or {}
            i_peak = max(i_peak, ina.get("i_peak", 0) or 0); vbat_min = min(vbat_min, s["vbat"])
        got, _ = ext5v_since(pos)
        print(f"  cycle {c+1}/{CYCLES}  vbat={s['vbat']:.2f}  i_peak={i_peak:.3f}  n={len(got)}")
    load, pos = ext5v_since(pos)
finally:
    rpc("ext5v.rate", ms=1000)

lv = [v for _, v, _, _ in load]
s1 = rpc("status")
print(f"\n--- RESULT over {CYCLES} rescue<->stand cycles ---")
print(f"  ext5v idle : mean {st.mean(iv):.4f}  min {min(iv):.4f}  n={len(iv)}")
if len(lv) < 100:
    print(f"  ext5v load : only {len(lv)} samples\n\nNO VERDICT: a starved sampler reads "
          f"exactly like a quiet rail.  Fix the sampler, not the number.")
    sys.exit(2)
span = (load[-1][0] - load[0][0]) / 1000.0
print(f"  ext5v load : mean {st.mean(lv):.4f}  min {min(lv):.4f}  n={len(lv)} over {span:.1f}s "
      f"({len(lv)/span:.0f} Hz)")
print(f"  DROP under load: {st.mean(iv)-st.mean(lv):+.4f} V mean, {min(iv)-min(lv):+.4f} V worst")
print(f"  throttled (benchd, 100 ms poll): {s0['pi_throttled']} -> {s1['pi_throttled']}")
print(f"  vbat min {vbat_min:.2f}   i_peak {i_peak:.3f} A   "
      f"bus_err {s0['bus_errors']}->{s1['bus_errors']}   wd {s0['watchdog_trips']}->{s1['watchdog_trips']}")
clean = int(s0["pi_throttled"], 0) == 0 and int(s1["pi_throttled"], 0) == 0
drop  = min(iv) - min(lv)
print(f"\n{'PASS' if (clean and drop < 0.10) else 'REVIEW'}: the Pi's rail is "
      + ("independent of servo load" if (clean and drop < 0.10)
         else f"MOVING with servo load (worst drop {drop:+.4f} V)"))
