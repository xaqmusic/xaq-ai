#!/usr/bin/env python3
# bv.py <port> '<json request>' [repeat]  -- one REQ per request, prints the reply
import json, sys, zmq
port, req = int(sys.argv[1]), json.loads(sys.argv[2])
n = int(sys.argv[3]) if len(sys.argv) > 3 else 1
s = zmq.Context.instance().socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 1500); s.setsockopt(zmq.LINGER, 0)
s.connect(f"tcp://127.0.0.1:{port}")
out = []
for _ in range(n):
    s.send_string(json.dumps(req)); out.append(json.loads(s.recv_string()))
if n == 1: print(json.dumps(out[0])[:600])
else:
    errs = [o.get("error", "")[:40] for o in out if not o.get("ok")]
    print(f"{n} sent; ok={sum(1 for o in out if o.get('ok'))}; errors: " + "; ".join(f"{e!r} x{errs.count(e)}" for e in sorted(set(errs))))
