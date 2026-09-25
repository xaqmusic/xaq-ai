#!/usr/bin/env python3
"""Where is the brownout ceiling NOW that the Pi is off the servo rail?  (Mod A follow-on.)

The old limits -- slew 40 us/tick, 100 ms pose stagger -- were chosen to protect a rail the
Pi lived on.  It no longer does (§3.8.8.1), the servos now have the whole 3 A DC-DC, and the
failure has changed species: servo-rail sag resets the HAT MCU rather than the computer.
That is recoverable, and it means the ceiling can be hunted without losing the instrument.

⚠ WHAT WE CANNOT SEE.  Mod C (§3.8.8.5) is NOT fitted -- no INA219 on the servo rail.  So
there is no voltage on the rail that actually sags, and the failure signatures available are
INDIRECT: HAT MCU resets, I2C bus errors, driver watchdog trips, servos not reaching target.
This finds the cliff by walking up to it, not by watching it approach.

⚠ CONDITION-SPECIFIC.  Records pack voltage and posture with every point, because §3.8.3 is
a sweep that had to be split in two when the surface changed mid-run.  A ceiling found on the
belly at 7.5 V is a FLOOR for a standing robot on a full pack, not the limit.

⚠ THE ROBOT MOVES, faster at every step. Abort with Ctrl-C; the daemon holds its own deadman.
"""
import glob, json, os, statistics as st, sys, time, zmq

ENDPOINT, LOG_DIR = "tcp://127.0.0.1:5590", os.path.expanduser("~/xaq-ai/pi_host/log")
VBAT_FLOOR = 7.00          # stop well above the 6.4 V auto-safe so the pack is not the story
CYCLES     = 2             # per point; the ladder is long and the pack is finite
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
            if r.get("kind") == "ext5v": out.append(r["data"]["v"])
        return out, fh.tell()

def moves_since(pos):
    """(pose.set -> pose.landed) durations on the DAEMON's clock, and the new file position.

    ⚠ WHY NOT TIME IT FROM HERE.  move() used to wait for pose_move_active to go true, with a
    3 s timeout.  At high slew the move finishes before any status poll observes it, so that
    loop timed out EVERY time and every point came back at 12.1 s regardless of slew -- my own
    timeout, reported as the robot's move duration, masking the one signal that would have
    shown the lever was not connected.  The daemon already records pose.set and pose.landed;
    the interval between them is the truth and costs nothing.
    """
    out, pending = [], None
    with open(path) as fh:
        fh.seek(pos)
        for line in fh:
            try: r = json.loads(line)
            except Exception: continue
            if r.get("kind") == "pose.set":    pending = r["t_mono_ms"]
            elif r.get("kind") == "pose.landed" and pending is not None:
                out.append((r["t_mono_ms"] - pending) / 1000.0); pending = None
        return out, fh.tell()

def move(pose, timeout=40):
    """Command a pose and wait for it to land.  Timing comes from moves_since(), not here."""
    rpc("pose.set", us=pose_us(pose))
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = rpc("status")
        if not s.get("pose_move_active") and not s.get("pose_queue"):
            time.sleep(0.25)                      # let pose.landed reach the log
            s2 = rpc("status")
            if not s2.get("pose_move_active"): return s2
        time.sleep(0.05)
    return rpc("status")

def point(slew, stagger, poses):
    """One ladder rung.  Returns (row, failure_reason or None)."""
    # ⚠ pose_slew_us, NOT slew_us.  begin_pose_move overwrites the normal slew on every
    # pose.set, so sweeping slew_us against pose moves sweeps nothing -- the first run of
    # this script did that and produced nine identical rows that read as a clean ceiling.
    r = rpc("limits.set", pose_slew_us=slew, stagger_ms=stagger, confirm=True)
    if not r.get("ok"): return None, f"limits.set refused: {r.get('error')}"
    b = rpc("status")
    pos = os.path.getsize(path)
    i_peak, vbat_min = 0.0, 99.0
    for _ in range(CYCLES):
        for pose in poses:
            s = move(pose)
            ina = s.get("ina") or {}
            i_peak  = max(i_peak, ina.get("i_peak", 0) or 0)
            vbat_min = min(vbat_min, s["vbat"])
    a = rpc("status")
    ev, pos2 = ext5v_since(pos)
    durs, _ = moves_since(pos)
    secs = st.median(durs) if durs else float("nan")
    row = dict(slew=slew, stagger=stagger, secs=secs, nmoves=len(durs), i_peak=i_peak, vbat_min=vbat_min,
               ext5v_min=min(ev) if ev else None, n=len(ev),
               bus_err=a["bus_errors"] - b["bus_errors"],
               wd=a["watchdog_trips"] - b["watchdog_trips"],
               thr=a["pi_throttled"], rail_ev=a["rail_events"] - b["rail_events"])
    why = None
    if row["bus_err"]: why = f"{row['bus_err']} I2C bus errors — the HAT MCU is struggling"
    elif row["wd"]:    why = f"{row['wd']} driver watchdog trips"
    elif row["rail_ev"]: why = "the Pi's OWN rail dipped — Mod A did not hold at this load"
    elif int(row["thr"], 0): why = f"throttle flags appeared ({row['thr']})"
    elif a["vbat"] < VBAT_FLOOR: why = f"pack down to {a['vbat']:.2f} V — stopping on charge, not on a ceiling"
    return row, why

s0 = rpc("status")
if not s0.get("ok"): sys.exit(f"daemon not answering: {s0}")
if s0["vbat"] < VBAT_FLOOR: sys.exit(f"vbat {s0['vbat']:.2f} already below the {VBAT_FLOOR} V floor")
rpc("ext5v.rate", ms=100)
print(f"start vbat={s0['vbat']:.2f}  thr={s0['pi_throttled']}  ext5v={s0['ext5v']}\n"
      f"posture: OPERATOR-REPORTED 'on the floor on its belly' — servos are NOT bearing the chassis\n")
# ⚠ `secs` is the tripwire.  If raising the slew does not shorten the moves, the knob is not
# in the path being driven -- which is how the first run of this sweep fooled itself.
# ⚠ `move_s` is the tripwire and it is measured by the DAEMON.  If raising the slew does not
# shorten the moves, the knob is not in the path being driven -- which fooled this sweep twice:
# once by setting the wrong global, once by reporting my own poll timeout as the move time.
hdr = (f"{'pslew':>6} {'stag':>5} {'move_s':>7} {'n':>3} {'i_peak':>7} {'vbat_lo':>8} "
       f"{'ext5v_lo':>9} {'bus':>4} {'wd':>3} {'thr':>8}")
def show(r):
    print(f"{r['slew']:>6} {r['stagger']:>5} {r['secs']:>7.2f} {r['nmoves']:>3} {r['i_peak']:>7.3f} "
          f"{r['vbat_min']:>8.2f} {(r['ext5v_min'] or 0):>9.4f} {r['bus_err']:>4} "
          f"{r['wd']:>3} {r['thr']:>8}")
rows, stopped = [], None
try:
    print("== ladder 1: POSE slew, stagger held at 100 ms, rescue<->stand ==")
    print(hdr)
    for slew in (12, 20, 30, 40, 60, 100, 150, 250, 400):   # pose slew: 12 = 600 us/s default
        row, why = point(slew, 100, ("stand", "rescue"))
        if row is None: stopped = why; break
        rows.append(row)
        show(row)
        if why: stopped = f"pose_slew {slew}: {why}"; break
    if not stopped:
        print("\n== ladder 2: stagger, at the top slew the ladder reached, X<->rescue (the 2026-08-29 case) ==")
        print(hdr)
        top = rows[-1]["slew"]
        for stag in (100, 60, 40, 20, 0):
            row, why = point(top, stag, ("X", "rescue"))
            if row is None: stopped = why; break
            rows.append(row)
            show(row)
            if why: stopped = f"pose_slew {top} stagger {stag}: {why}"; break
finally:
    # _allow_err: a cleanup exception would replace the real failure with its own
    rpc("limits.set", _allow_err=True, slew_us=40, pose_slew_us=12, stagger_ms=100, confirm=True)
    rpc("ext5v.rate", _allow_err=True, ms=1000)
    rpc("pose.set", _allow_err=True, us=pose_us("rescue"))
    print("\nrestored: slew 40, stagger 100 ms, rescue pose, ext5v 1 s")

s1 = rpc("status")
print(f"\nstopped: {stopped or 'nothing failed — the ladder ran out before the robot did'}")
print(f"end vbat={s1['vbat']:.2f} (started {s0['vbat']:.2f})  thr={s1['pi_throttled']}  "
      f"bus_err={s1['bus_errors']}  wd={s1['watchdog_trips']}  rail_events={s1['rail_events']}")
# ⚠ THE SELF-CHECK.  Two sweeps have now reported "no failure" while the lever was not
# connected.  A clean ladder is only evidence if the ladder did something, so prove it from
# the daemon's own move times before any ceiling is claimed.
l1 = [r for r in rows if r["stagger"] == 100 and r["nmoves"]]
if len(l1) < 2:
    print(f"\n⚠⚠ NO CEILING CLAIMED, AND NO LEVER CHECK POSSIBLE: only {len(l1)} of "
          f"{len([r for r in rows if r['stagger']==100])} ladder-1 points recorded any completed "
          f"move at all.  The robot may not have moved.  Do not read this table.")
elif len(l1) >= 2:
    lo, hi = l1[0], l1[-1]
    ratio = (lo["secs"] / hi["secs"]) if hi["secs"] else 0.0
    moved = ratio > 1.5
    print(f"\nlever check: pose_slew {lo['slew']} -> {hi['slew']} ({hi['slew']/lo['slew']:.0f}x) "
          f"moved the median move time {lo['secs']:.2f}s -> {hi['secs']:.2f}s ({ratio:.2f}x faster)")
    if not moved:
        print("⚠ NO CEILING CLAIMED: the slew lever did not change how long the moves take, so "
              "'nothing failed' says nothing about slew.  Find why the knob is not in the path "
              "before reading anything else in this table.")
if rows:
    best = max(rows, key=lambda r: r["i_peak"])
    print(f"highest current seen: {best['i_peak']:.3f} A at slew {best['slew']}, stagger {best['stagger']} ms")
    print("⚠ belly posture + 7.5 V pack: treat every number here as a FLOOR, not a limit.")
