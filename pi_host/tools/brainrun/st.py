#!/usr/bin/env python3
# st.py — one-line benchd status summary
import json, zmq
s = zmq.Context.instance().socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 1500); s.setsockopt(zmq.LINGER, 0)
s.connect("tcp://127.0.0.1:5590"); s.send_string('{"verb":"status"}'); f = json.loads(s.recv_string())
b = f.get("brain") or {}
print(f"  mode={f['mode']} stopped={f['stopped']}({f['stop_why']}) deadman={f['deadman_ms_left']} vbat={f['vbat']:.2f} "
      f"armed={sum(1 for x in f['servos'] if x['armed'])} moving={f['pose_move_active']} rescue={f['rescue_active']} "
      f"brain: frames={b.get('frames')} applied={b.get('applied')} blocked={b.get('blocked')} bad={b.get('bad')} "
      f"gaps={b.get('seq_gaps')} have={b.get('have_stream')} hold={b.get('holding')} clamped=0x{b.get('clamped_mask',0):03x}")
print(f"  current_us={[x['current_us'] for x in f['servos']]}")
if b.get('last_us'): print(f"  brain asked={b['last_us']}")
