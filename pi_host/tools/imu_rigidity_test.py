#!/usr/bin/env python3
"""Mount rigidity: does the IMU pick up servo vibration?

Compares accelerometer noise with the body AT REST in every arm, so any increase
is vibration transmitted through the mount rather than real body acceleration.

Safety: uses pose.set at its DEFAULT slew (12 us/tick, 600 us/s, 100 ms stagger)
and never bypasses the slew limiter -- section 3.8.2 puts the duty budget at
slew <= 50 (<= 1.90 A), and ~3.0 A at slew >= 200 which is over the 5 V rail
rating. Aborts on low pack or overcurrent. Exiting stops the deadman pings,
which lands the robot in the rescue pose by design.
"""
import zmq, spidev, time, struct, statistics as st, sys

VBAT_ABORT, I_ABORT = 6.8, 2.5
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
A_LSB, G_LSB = 8192.0, 65.5

ctx = zmq.Context()
def new_req():
    s = ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 800)
    s.setsockopt(zmq.LINGER, 0); s.connect("tcp://127.0.0.1:5590"); return s
req = new_req()

def verb(_v, **kw):
    global req
    try:
        req.send_json(dict(verb=_v, **kw)); return req.recv_json()
    except zmq.Again:
        req.close(); req = new_req(); return {"ok": False, "error": "timeout"}

# --- IMU: configure ONCE, no reset between phases -------------------------
spi = spidev.SpiDev(); spi.open(0, 0); spi.mode = 0; spi.max_speed_hz = 4000000
def wr(r, v): spi.xfer2([r & 0x7F, v]); time.sleep(0.001)
def rd(r, n=1): return spi.xfer2([r | 0x80] + [0]*n)[1:]
def bank(b): wr(0x7F, (b & 3) << 4)
bank(0); wr(0x06, 0x80); time.sleep(0.2)
bank(0); wr(0x06, 0x01); time.sleep(0.05)
wr(0x07, 0x00); wr(0x03, 0x10); time.sleep(0.02)
bank(2); wr(0x00, 0x04); wr(0x01, 0x0B); wr(0x10, 0x00); wr(0x11, 0x04); wr(0x14, 0x0B)
bank(0); time.sleep(0.2)
assert rd(0x00)[0] == 0xEA, "WHO_AM_I lost after config"

class Abort(Exception): pass

def sample(dur, label):
    """Sample the IMU while feeding the deadman and watching the rail."""
    ax=[]; ay=[]; az=[]; gm=[]; imax=0.0; vmin=99.0
    t0=time.monotonic(); nxt_ping=0.0; nxt_stat=0.0
    while time.monotonic() - t0 < dur:
        v = struct.unpack(">hhhhhhh", bytes(rd(0x2D, 14)))
        ax.append(v[0]/A_LSB); ay.append(v[1]/A_LSB); az.append(v[2]/A_LSB)
        gm.append(max(abs(v[3]), abs(v[4]), abs(v[5]))/G_LSB)
        now = time.monotonic() - t0
        if now >= nxt_ping:
            verb("ping"); nxt_ping = now + 0.4
        if now >= nxt_stat:
            r = verb("status"); nxt_stat = now + 1.0
            d = r.get("data", r)
            try:
                i = float(d["ina"]["i_a"]); vb = float(d["vbat"])
                imax = max(imax, i); vmin = min(vmin, vb)
                if vb < VBAT_ABORT or i > I_ABORT:
                    raise Abort("vbat %.2f V, i %.2f A" % (vb, i))
            except (KeyError, TypeError, ValueError):
                pass
        time.sleep(0.02)
    sd = (st.pstdev(ax), st.pstdev(ay), st.pstdev(az))
    tot = (sd[0]**2 + sd[1]**2 + sd[2]**2) ** 0.5
    print("  %-18s n=%4d  sd x %.5f  y %.5f  z %.5f g   |sd| %.5f   peak %.2f A  min %.2f V  max|gyro| %.1f dps"
          % (label, len(ax), sd[0], sd[1], sd[2], tot, imax, vmin, max(gm)))
    return sd

def goto(name):
    r = verb("pose.get", name=name)
    if not r.get("ok"): raise Abort("pose.get %s: %s" % (name, r.get("error")))
    r = verb("pose.set", us=r["us"])
    if not r.get("ok"): raise Abort("pose.set %s: %s" % (name, r.get("error")))
    eta = r.get("eta_ms", 4000)/1000.0
    print("  -> pose %-8s staggered=%s  eta %.1f s" % (name, r.get("staggered"), eta))
    t0 = time.monotonic()
    while time.monotonic() - t0 < eta + 10.0:
        verb("ping")
        d = verb("status").get("data", {})
        if d and not d.get("pose_move_active", True) and not d.get("pose_queue", 0):
            break
        time.sleep(0.25)
    for _ in range(12):
        verb("ping"); time.sleep(0.25)          # settle, deadman fed

try:
    print("PHASE 0  unarmed baseline (servos off, body at rest)")
    base = sample(DUR, "unarmed")
    print("\nPHASE 1  rescue pose, ARMED and holding")
    goto("rescue"); r1 = sample(DUR, "armed @ rescue")
    print("\nPHASE 2  stand pose, ARMED and holding")
    goto("stand");  r2 = sample(DUR, "armed @ stand")
    print("\nRETURN   back to rescue, then releasing the deadman")
    goto("rescue")
    b = (base[0]**2 + base[1]**2 + base[2]**2) ** 0.5
    print("\nRATIO vs unarmed baseline")
    for lab, r in (("rescue", r1), ("stand", r2)):
        m = (r[0]**2 + r[1]**2 + r[2]**2) ** 0.5
        print("  armed @ %-8s  |sd| x %.2f" % (lab, m/b))
except Abort as e:
    print("ABORTED:", e)
    verb("limp")
finally:
    spi.close()
