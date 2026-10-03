#!/usr/bin/env python3
# runlog.py <seconds> <out.jsonl> — benchd telemetry at 5 Hz during a brain run, with a TILT GUARD
# that STOPs via the control socket past 80 deg (operator: 60 was too tight).  ⚠ It REFUSES to start without attitude: on
# 2026-10-03 benchd's frame carried imu:null and a fallback read "upright" on every sample.
import json, sys, time, zmq
dur, out = float(sys.argv[1]), sys.argv[2]
ctx = zmq.Context.instance()
def sock(port):
    s = ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 1000); s.setsockopt(zmq.LINGER, 0)
    s.connect(f"tcp://127.0.0.1:{port}"); return s
bench, ctl = sock(5590), sock(5593)
def status():
    global bench
    try:
        bench.send_string('{"verb":"status"}'); return json.loads(bench.recv_string())
    except Exception:
        bench.close(); bench = sock(5590); return None
f = status()
imu = (f or {}).get("imu") or {}
if not imu.get("ok") or not imu.get("up_fused"):
    print("runlog: REFUSING — benchd frame has no attitude (imu null or not ok); the tilt guard would be blind")
    sys.exit(2)
t0 = time.time(); n = 0; why = "time"
with open(out, "w") as fo:
    while time.time() - t0 < dur:
        f = status()
        if f is None: print("status miss"); time.sleep(0.2); continue
        b = f.get("brain") or {}; imu = f.get("imu") or {}; ina = f.get("ina") or {}
        if not imu.get("ok") or not imu.get("up_fused"):
            ctl.send_string('{"verb":"stop"}'); ctl.recv_string(); why = "attitude lost — stopped (guard would be blind)"; break
        up = imu["up_fused"]
        rec = {"t": round(time.time() - t0, 2), "stopped": f["stopped"], "why": f["stop_why"],
               "applied": b.get("applied"), "hold": b.get("holding"), "clamped": b.get("clamped_mask"),
               "vbat": round(f["vbat"], 2), "i_a": round(ina.get("i_a", 0.0), 3), "up": up,
               "disagree": imu.get("disagree_deg"), "tof": {k: (f.get("tof") or {}).get(k) for k in ("m", "valid", "m_comp", "comp_valid", "status")},
               "cur": [x["current_us"] for x in f["servos"]], "out": [x.get("out_us") for x in f["servos"]],
               "ask": b.get("last_us"), "at_limit": [round(x["at_limit_s"], 1) for x in f["servos"]]}
        fo.write(json.dumps(rec) + "\n"); n += 1
        if f["stopped"]: why = f"stopped by benchd: {f['stop_why']}"; break
        if up[1] < 0.1736:
            ctl.send_string('{"verb":"stop"}'); ctl.recv_string(); why = f"tilt guard (up.y {up[1]:.2f})"; break
        time.sleep(0.2)
print(f"runlog: {n} samples, ended by {why}")
