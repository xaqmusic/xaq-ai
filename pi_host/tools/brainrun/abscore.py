#!/usr/bin/env python3
# abscore.py <dir> -- score arm.sh runs (jerk on the line, current, tilt, inputs). Tags are listed below.
import json, math, statistics as st, sys
S = sys.argv[1]   # a directory holding ab/<tag>/ from arm.sh runs
def feedm(tag):
    F = [json.loads(l) for l in open(f"{S}/ab/{tag}/feed.jsonl")]
    F = [f for f in F if not f["stopped"]]
    o = list(zip(*[f["out"] for f in F])); u = list(zip(*[f["us"] for f in F]))
    T = len(F) / 50.0
    def chan(x):
        d = [x[i+1]-x[i] for i in range(len(x)-1)]
        sg = [1 if v > 0 else -1 for v in d if abs(v) > 2]
        rev = sum(1 for i in range(len(sg)-1) if sg[i] != sg[i+1]) / T
        cap = sum(1 for v in d if abs(v) >= 38) / len(d)
        spd = st.mean(abs(v) for v in d) * 50 / 545.2           # rad/s
        acc = st.mean(abs(d[i+1]-d[i]) for i in range(len(d)-1))  # us/tick^2
        return rev, cap, spd, acc
    co = [chan(x) for x in o]; cu = [chan(x) for x in u]
    m = lambda L, k: st.mean(r[k] for r in L)
    return {"T": T, "rev_out": m(co,0), "cap_out": m(co,1), "spd_out": m(co,2), "jerk_out": m(co,3),
            "rev_cmd": m(cu,0), "spd_cmd": m(cu,2)}
def runm(tag):
    R = [json.loads(l) for l in open(f"{S}/ab/{tag}/run.jsonl")]
    ia = [r["i_a"] for r in R]
    tilt = [math.degrees(math.acos(max(-1, min(1, r["up"][1])))) for r in R]
    return {"i_mean": st.mean(ia), "i_max": max(ia), "vbat_min": min(r["vbat"] for r in R),
            "tilt_max": max(tilt), "tilt_mean": st.mean(tilt), "dis_max": max(r["disagree"] or 0 for r in R)}
def hostm(tag):
    D = []
    for l in open(f"{S}/ab/{tag}/host.log"):
        if '"kind":"brain_inputs"' in l and l.startswith('{'): D.append(json.loads(l))
    up = [d["upright"] for d in D]; sv = [math.hypot(*d["stride_v"]) for d in D]
    return {"upright_min": min(up), "stride_mean": st.mean(sv), "n_dump": len(D)}
print(f"{'arm':4} {'rev/s out':>9} {'rev/s cmd':>9} {'@cap':>6} {'rad/s out':>9} {'rad/s cmd':>9} {'jerk':>6} {'I mean':>7} {'I max':>6} {'Vmin':>5} {'tilt max':>8} {'uprt min':>8} {'|sv|':>6}")
for tag in ("A1", "B1", "B2", "A2"):
    f, r, h = feedm(tag), runm(tag), hostm(tag)
    print(f"{tag:4} {f['rev_out']:9.2f} {f['rev_cmd']:9.2f} {f['cap_out']:6.2f} {f['spd_out']:9.2f} {f['spd_cmd']:9.2f} {f['jerk_out']:6.1f} {r['i_mean']:7.2f} {r['i_max']:6.2f} {r['vbat_min']:5.2f} {r['tilt_max']:8.1f} {h['upright_min']:8.3f} {h['stride_mean']:6.3f}")
