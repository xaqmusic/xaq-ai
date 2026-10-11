#!/usr/bin/env python3
"""Minimal ogma_benchd verb client — ONE verb per invocation, printed as JSON.

Bench mode only: PROTOCOL.md's calibration channel.  It cannot start a brain and it
cannot issue a brain-rate stream; the daemon refuses both.  Every call prints the verb
it sent as well as the reply, so a bench log says what was commanded and not only what
came back.
"""
import json, sys, zmq

def main():
    verb = sys.argv[1]
    args = {}
    for kv in sys.argv[2:]:
        k, _, v = kv.partition("=")
        try:
            args[k] = json.loads(v)
        except Exception:
            args[k] = v
    req = {"verb": verb}
    req.update(args)
    ctx = zmq.Context()
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.RCVTIMEO, 8000)
    s.setsockopt(zmq.LINGER, 0)
    s.connect("tcp://127.0.0.1:5590")
    s.send_string(json.dumps(req))
    print("-> " + json.dumps(req))
    try:
        print("<- " + s.recv_string())
    except zmq.Again:
        print("<- TIMEOUT (daemon busy or verb refused silently)")
        sys.exit(2)

main()
