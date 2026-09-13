#!/usr/bin/env python3
"""Fault-injection drill for RailGuard — tests the WIRING, not the logic.

RailGuard's transition logic has unit tests.  What they cannot reach is whether benchd
actually calls it, whether a fresh bit actually reaches rescue(), and whether the arming
verbs actually refuse during the back-off.  This drives the real daemon over its real
socket and checks each of those against the frame it publishes.

⚠ THE ROBOT MOVES.  A pass commands the rescue pose.  Put it somewhere safe first.
"""
import json, sys, time, zmq

ENDPOINT = "tcp://127.0.0.1:5590"
GUARD_MS = 5000
_ctx = zmq.Context()

def rpc(verb, **kw):
    # A fresh REQ per call: REQ enforces strict send/recv alternation and a single
    # timeout would poison a reused socket for every later check in the drill.
    s = _ctx.socket(zmq.REQ)
    s.setsockopt(zmq.RCVTIMEO, 8000); s.setsockopt(zmq.LINGER, 0)
    s.connect(ENDPOINT)
    s.send_string(json.dumps({"verb": verb, **kw}))
    try:    return json.loads(s.recv_string())
    except zmq.Again: return {"ok": False, "error": "TIMEOUT"}
    finally: s.close()

fails = []
def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail else ""))
    if not cond: fails.append(name)

st = rpc("status")
base_events = st["rail_events"]
print(f"before: throttled={st['pi_throttled']} rail_events={base_events} "
      f"guarded={st['rail_guarded']} ext5v={st.get('ext5v')}")
check("guard is not already latched", not st["rail_guarded"])

# --- 1. the drill needs confirm ------------------------------------------
r = rpc("rail.inject", bits=0x10000)
check("rail.inject refuses without confirm", not r.get("ok", False), r.get("error", ""))

# --- 2. inject a bit the current boot does not have ----------------------
# ⚠ Choose a bit the guard has not ALREADY absorbed.  Its baseline advances every time it
# fires, by design -- so a second drill re-injecting the first drill's bit is correctly a
# non-event, and reads as "the poll never calls update()" when it is nothing of the kind.
# The baseline, not the current mask, is what decides whether a bit can still be an event.
base = st.get("rail_baseline", int(st["pi_throttled"], 0))
CANDIDATES = [(0x10000, "under-voltage has occurred"), (0x40000, "throttling has occurred"),
              (0x20000, "arm frequency capped"),       (0x80000, "soft temp limit")]
BIT, WHY = next(((b, w) for b, w in CANDIDATES if not (base & b)), (0, ""))
if not BIT:
    print(f"! every candidate bit is already in the guard's baseline (0x{base:X}).\n"
          f"  Restart ogma-benchd to re-baseline, then re-run."); sys.exit(2)
print(f"\ninjecting 0x{BIT:X} ({WHY}) — baseline 0x{base:X}, real mask {st['pi_throttled']} ...")
r = rpc("rail.inject", bits=BIT, confirm=True)
check("rail.inject accepted", r.get("ok", False), json.dumps(r.get("result", r)))

# --- 3. the poll must pick it up within ~1 s -----------------------------
t0 = time.time(); fired = None
while time.time() - t0 < 3.0:
    st = rpc("status")
    if st["rail_events"] > base_events: fired = time.time() - t0; break
    time.sleep(0.1)
check("the 1 Hz poll reached the guard", fired is not None,
      f"fired after {fired:.2f}s" if fired else "no event in 3 s — the poll never calls update()")
if fired is None: sys.exit(1)
t_event = t0 + fired          # measure the back-off from HERE, not from the expiry loop

check("rail_events incremented by exactly 1", st["rail_events"] == base_events + 1,
      f"{base_events} -> {st['rail_events']}")
check("the back-off latched", st["rail_guarded"])
check("rescue pose was commanded", st["rescue_active"] or st["rescue_pose"] is not None,
      f"rescue_pose={st['rescue_pose']} active={st['rescue_active']}")
check("the injection is visible in the frame", st.get("rail_inject") == BIT,
      "a consumer can tell this drill from a real dip")

# --- 4. the arming verbs must refuse WHILE guarded -----------------------
for verb, kw in (("servo.set", {"ch": 0, "us": 1500}),
                 ("pose.set",  {"name": "rescue"}),
                 ("cal.begin", {"ch": 0})):
    r = rpc(verb, **kw)
    refused = not r.get("ok", False) and "rail" in json.dumps(r).lower()
    check(f"{verb} refused during back-off", refused, r.get("error", "")[:60])

# --- 5. it must fire ONCE, not every poll --------------------------------
print("\nholding the bit set for 3 polls ...")
n_at = st["rail_events"]; time.sleep(3.2)
st = rpc("status")
check("a held bit does not re-fire every poll", st["rail_events"] == n_at,
      f"still {st['rail_events']} — a re-firing guard would pin the robot in rescue")

# --- 6. the back-off must EXPIRE -----------------------------------------
t0 = time.time()
while time.time() - t0 < GUARD_MS/1000 + 4:
    st = rpc("status")
    if not st["rail_guarded"]: break
    time.sleep(0.2)
held = time.time() - t_event
# ⚠ Measured from the EVENT, not from this loop — the 3 s hold above already spent most
# of the back-off, so timing it from here would report ~1.8 s and prove nothing about the
# configured 5 s.  Both bounds matter: too short re-loads a dipping rail, too long reads
# downstream as a dead servo bus.
check("the back-off expired on its own", not st["rail_guarded"], f"held {held:.1f}s")
check(f"it lasted the configured {GUARD_MS/1000:.0f}s", abs(held - GUARD_MS/1000) < 1.2,
      f"{held:.1f}s vs {GUARD_MS/1000:.0f}s (+/- the 0.2 s poll and 1 s status granularity)")
r = rpc("servo.set", ch=0, us=1500)
check("servo.set works again once clear", r.get("ok", False), r.get("error", ""))
rpc("limp")

# --- 7. clear the injection ----------------------------------------------
rpc("rail.inject", bits=0, confirm=True)
st = rpc("status")
check("injection cleared", st.get("rail_inject") == 0)
check("no spurious event on clearing", st["rail_events"] == base_events + 1,
      "removing an injected bit is not a new event")

print(f"\n{'DRILL PASSED' if not fails else 'DRILL FAILED: ' + ', '.join(fails)}")
sys.exit(1 if fails else 0)
