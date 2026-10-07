#!/usr/bin/env python3
"""tof_reduce.py — reduce the host's full level-2 JSONL to what the ToF studies need.

Usage:  <host ... --log-tof-cloud --log-motor-tle> | python3 tof_reduce.py out.jsonl
Charter: docs/plans-and-designs/microduck/tof_studies.md §2.
Drops qpos (50 floats/tick) but keeps the four free bodies' x,y -- the analysis labels."""
import json, sys
FREE = {'obj_ball0': 22, 'obj_ball1': 29, 'obj_block0': 36, 'obj_block1': 43}
KEEP = ("t","x","y","z","tilt","drive","wall","obj","tofs","tofr","tofz","map","hdg","twist",
        "q","stop","satt","event","mtle","btle","tofp","cld","cldp")
out = open(sys.argv[1], "w")
n = 0
for line in sys.stdin:
    if not line.startswith('{'): continue
    try: r = json.loads(line)
    except ValueError: continue
    d = {k: r[k] for k in KEEP if k in r}
    if "qpos" in r:
        d["free"] = {nm: [round(r["qpos"][a], 4), round(r["qpos"][a+1], 4), round(r["qpos"][a+2], 4)]
                     for nm, a in FREE.items()}
    out.write(json.dumps(d, separators=(',', ':')) + "\n"); n += 1
out.close()
print(f"reduced {n} ticks -> {sys.argv[1]}", file=sys.stderr)
