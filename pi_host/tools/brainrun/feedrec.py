#!/usr/bin/env python3
# feedrec.py <seconds> <out.jsonl> — record benchd's 50 Hz state feed (us = slewed command,
# out = pulse on the line) for the jerk metric.
import json, sys, time, zmq
dur, path = float(sys.argv[1]), sys.argv[2]
s = zmq.Context.instance().socket(zmq.SUB); s.setsockopt(zmq.RCVTIMEO, 500)
s.setsockopt_string(zmq.SUBSCRIBE, "state "); s.connect("tcp://127.0.0.1:5592")
t0 = time.time(); n = 0
with open(path, "w") as f:
    while time.time() - t0 < dur:
        try: m = s.recv_string()
        except zmq.Again: continue
        d = json.loads(m[6:])
        f.write(json.dumps({"seq": d["seq"], "t": d["t"], "us": d["us"], "out": d.get("out"), "stopped": d.get("stopped"), "tof_m": d.get("tof_m"), "tof_valid": d.get("tof_valid"), "fsr": d.get("fsr")}) + "\n"); n += 1
print(f"feedrec: {n} frames")
