#!/usr/bin/env python3
# current_by_foot.py <benchd_*.jsonl ...> -- S0: servo current against per-foot FSR load and the number of loaded feet.
import json, statistics as st, sys, glob
def corr(xs, ys):
    mx, my = st.mean(xs), st.mean(ys)
    sx = sum((x-mx)**2 for x in xs)**.5; sy = sum((y-my)**2 for y in ys)**.5
    return sum((x-mx)*(y-my) for x, y in zip(xs, ys))/(sx*sy) if sx and sy else float("nan")
I=[]; A=[[] for _ in range(4)]; MX=[]; NC=[]
for p in sys.argv[1:]:
    for line in open(p, errors="replace"):
        if '"kind":"telemetry"' not in line: continue
        try: d=json.loads(line)["data"]
        except ValueError: continue
        if d.get("mode") not in ("dev","autonomous") or d.get("stopped"): continue
        ina=d.get("ina") or {}; adc=d.get("adc") or []
        if not ina.get("ok") or len(adc)<5 or d.get("vbat",0)>9: continue
        I.append(ina["i_a"])
        for k in range(4): A[k].append(adc[k])
        MX.append(max(adc[:4])); NC.append(sum(1 for k in range(4) if adc[k] > 200))
names=["A0 phys FL","A1 phys FR","A2 phys RL","A3 phys RR"]
print(f"n={len(I)}")
for k in range(4): print(f"  {names[k]}  r = {corr(A[k], I):+.2f}   mean counts {st.mean(A[k]):.0f}")
print(f"  max single foot   r = {corr(MX, I):+.2f}")
print(f"  feet in contact   r = {corr(NC, I):+.2f}   (mean {st.mean(NC):.2f})")
# current by number of loaded feet
for n in range(5):
    sel=[i for i, c in zip(I, NC) if c==n]
    if sel: print(f"  {n} feet loaded: n={len(sel):5d}  current mean {st.mean(sel):.2f} A  p95 {sorted(sel)[int(.95*len(sel))]:.2f}")
