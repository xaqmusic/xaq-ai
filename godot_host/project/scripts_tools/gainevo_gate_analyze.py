#!/usr/bin/env python3
"""GainEvolver gate analysis — point it at a seedavg output dir.

    python3 gainevo_gate_analyze.py <SEEDAVG_OUT dir>


Judges the gate, and additionally judges THE FIX ITSELF:
  1. accepts > 0            — tautology check
  2. lockstep               — ga_app == ge_pub * 8, ga_rej == 0
  3. did J fall?            — now compared against the MEASURED noise, not eyeballed
  4. did sigma stay off the ceiling?  (gate 2 pinned 3/4 seeds at sigma_max 0.5)
  5. is the noise actually lower?     (variance decomposition, vs gate 2's 81% falls)
  6. did the vector walk toward the hand point?
"""
import glob, json, math, statistics as st, sys

KEYS = ["rear_land", "rear_knee", "rear_push", "amp_tgt",
        "height_hg", "postural", "coupling", "plan_gain"]
FACTORY = [0.0, 0.2, 0.0, 0.4, 0.0, 0.3, 0.0, 0.0]
HAND    = [0.5, 0.2, 0.5, 0.4, 0.04, 0.7, 1.55, 0.05]
SIGMA_MAX = 0.5
dist = lambda a, b: sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

paths = sorted(glob.glob((sys.argv[1] if len(sys.argv) > 1 else ".") + "/*.log"))
print(f"{len(paths)} seed logs\n")
finals, closer, taut, pooled_terms, pooled_noise, ceil_hits = [], 0, 0, [], [], 0
summary = []

for p in paths:
    seed = p.split("_s")[-1].split(".")[0]
    rows, seen = [], set()
    last = {}
    for ln in open(p, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_gen"' not in ln:
            continue
        try: d = json.loads(ln)
        except json.JSONDecodeError: continue
        last = d
        g = d.get("ge_gen")
        if d.get("ge_ji", -1) > 0 and g not in seen:
            seen.add(g); rows.append(d)
    if not rows:
        print(f"seed {seed}: NO scored windows — EXCLUDE\n"); continue
    rows.sort(key=lambda d: d["ge_gen"])
    gen  = int(last.get("ge_gen", 0)); acc = int(last.get("ge_acc", 0))
    rev  = int(last.get("ge_rev", 0)); pub = int(last.get("ge_pub", 0))
    app  = int(last.get("ga_app", 0)); rej = int(last.get("ga_rej", 0))
    sig  = float(last.get("ge_sig", 0)); nn = int(last.get("ge_nn", 0))
    sige = float(last.get("ge_sig_e", 0)); marg = float(last.get("ge_marg", 0))
    vec  = last.get("ge_vec") or rows[-1].get("ge_vec") or []
    J    = [r["ge_ji"] for r in rows]
    half = max(1, len(J) // 2)
    delta = st.mean(J[half:]) - st.mean(J[:half])

    # pure measurement noise: consecutive incumbent windows across a REVERT
    noise = [b["ge_ji"] - a["ge_ji"] for a, b in zip(rows, rows[1:])
             if int(b.get("ge_rev", 0)) > int(a.get("ge_rev", 0))]
    sd_noise = st.pstdev(noise) / math.sqrt(2) if len(noise) > 1 else float("nan")
    pooled_noise += noise
    for r in rows:
        pooled_terms.append({k: float(r.get(k, 0.0))
                             for k in ("ge_tilt", "ge_unl", "ge_flow", "ge_falls",
                                       "ge_dwell", "ge_energy")})
    if acc > 0: taut += 1
    if sig >= SIGMA_MAX - 1e-9: ceil_hits += 1
    d0, dN = dist(FACTORY, HAND), (dist(vec, HAND) if len(vec) == 8 else float("nan"))
    if dN == dN and dN < d0: closer += 1
    if len(vec) == 8: finals.append(vec)

    lock = "OK" if (app == pub * 8 and rej == 0) else f"** MISMATCH {app} vs {pub*8}, rej {rej} **"
    verdict = ("fell" if delta < -abs(sd_noise or 0) else
               "rose" if delta > abs(sd_noise or 0) else "inside noise")
    print(f"--- seed {seed} ---")
    print(f"  generations {gen}   accepts {acc}  reverts {rev}   lockstep {lock}")
    print(f"  sigma {sig:.3f}{'  ** AT CEILING **' if sig >= SIGMA_MAX-1e-9 else ''}"
          f"   sigma_hat {sige:.4f}  margin {marg:.4f}  (noise samples n={nn})")
    print(f"  J: {len(J)} windows, first-half {st.mean(J[:half]):.4f} -> "
          f"last-half {st.mean(J[half:]):.4f}   delta {delta:+.4f}")
    print(f"     measured noise sd {sd_noise:.4f}  ->  J {verdict}")
    if len(vec) == 8:
        print(f"  dist to HAND: {d0:.3f} -> {dN:.3f}  ({'CLOSER' if dN<d0 else 'FURTHER'})")
        for i, k in enumerate(KEYS):
            a = "  " if abs(vec[i]-FACTORY[i]) < 1e-9 else ("^ " if vec[i] > FACTORY[i] else "v ")
            print(f"    {a}{k:<12}{FACTORY[i]:>7.3f} -> {vec[i]:>7.3f}   (hand {HAND[i]})")
    print()
    summary.append((seed, delta, sd_noise, sig))

print("=== ACROSS SEEDS ===")
if finals:
    print("  gain          mean final     sd    factory    hand")
    for i, k in enumerate(KEYS):
        col = [f[i] for f in finals]
        print(f"    {k:<12}{st.mean(col):>9.3f}{(st.pstdev(col) if len(col)>1 else 0):>7.3f}"
              f"{FACTORY[i]:>10.3f}{HAND[i]:>8.3f}")
print(f"\n  tautology (accepts>0):        {taut}/{len(paths)}")
print(f"  moved closer to hand point:   {closer}/{len(paths)}")
print(f"  sigma AT CEILING ({SIGMA_MAX}):        {ceil_hits}/{len(paths)}"
      f"   [gate 2 was 3/4 — lower is the fix working]")
if len(pooled_noise) > 1:
    sdn = st.pstdev(pooled_noise) / math.sqrt(2)
    print(f"\n  pooled measurement noise sd:  {sdn:.4f}   (gate 2: 0.8765)")
if pooled_terms:
    W = {"ge_tilt": 1.0, "ge_unl": 1.0, "ge_flow": 1.0, "ge_falls": 0.0,
         "ge_dwell": 0.0, "ge_energy": 8.0}
    var = {k: st.pvariance([W[k]*t[k] for t in pooled_terms]) for k in W}
    tot = sum(var.values()) or 1.0
    print("  variance share by weighted term   [gate 2: falls was 80.9%]")
    for k, v in sorted(var.items(), key=lambda kv: -kv[1]):
        print(f"    {k:<10}{100*v/tot:6.1f}%")


# ---- THE DWELL DECISION -------------------------------------------------------
# dwell shipped at weight 0 on a 60-tick-sampled estimate that said it was ~2x
# worse than sd(upright).  This run logs it at FULL per-tick resolution, so the
# same comparison can finally be made on real data.  Noise is measured the only
# honest way available: consecutive incumbent windows across a REVERT scored the
# same vector, so their difference is pure measurement noise.
print("\n=== DWELL vs sd(upright): should dwell get a nonzero weight? ===")
per_seed = {}
for p_ in paths:
    rows, seen = [], set()
    for ln in open(p_, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_gen"' not in ln: continue
        try: d = json.loads(ln)
        except json.JSONDecodeError: continue
        g = d.get("ge_gen")
        if d.get("ge_ji", -1) > 0 and g not in seen:
            seen.add(g); rows.append(d)
    rows.sort(key=lambda d: d["ge_gen"])
    per_seed[p_] = rows
for key, label in (("ge_tilt", "sd(upright)  [live, weight 1]"),
                   ("ge_dwell", "dwell        [instrumented, weight 0]")):
    diffs, vals = [], []
    for rows in per_seed.values():
        for a, b in zip(rows, rows[1:]):
            va, vb = float(a.get(key, 0.0)), float(b.get(key, 0.0))
            vals += [va, vb]
            if int(b.get("ge_rev", 0)) > int(a.get("ge_rev", 0)):
                diffs.append(vb - va)
    if len(diffs) > 1 and st.mean(vals):
        sd = st.pstdev(diffs) / math.sqrt(2); m = st.mean(vals)
        print(f"  {label:<40} mean {m:9.5f}  noise sd {sd:9.5f}  noise/mean {sd/abs(m):7.2f}")
    else:
        print(f"  {label:<40} insufficient data (mean {st.mean(vals) if vals else 0:.5f})")
print("  (60-tick-sampled proxy said: sd(upright) 1.93 vs dwell 5.45-5.91)")
print("  -> lower noise/mean than sd(upright) would justify turning dwell ON.")


# ---- THE TWO NEW MECHANISMS ---------------------------------------------------
print(chr(10)+"=== ENERGY TERM + FALL ALARM ===")
rows_by_seed = {}
for p_ in paths:
    rows, seen = [], set()
    for ln in open(p_, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_gen"' not in ln: continue
        try: d = json.loads(ln)
        except json.JSONDecodeError: continue
        g = d.get("ge_gen")
        if d.get("ge_ji", -1) > 0 and g not in seen:
            seen.add(g); rows.append(d)
    rows.sort(key=lambda d: d["ge_gen"]); rows_by_seed[p_] = rows

# alarm duty: pinned 1 = guard permanently strict; pinned 0 = decoration
for p_, rows in rows_by_seed.items():
    seed = p_.split("_s")[-1].split(".")[0]
    on = [int(r.get("ge_alarm_on", 0)) for r in rows]
    al = [float(r.get("ge_alarm", 0.0)) for r in rows]
    if on:
        print(f"  seed {seed}: alarm tripped {100*sum(on)/len(on):5.1f}% of generations"
              f"   level {min(al):.2f}..{max(al):.2f}")

# THE FREEZE TRAP: does energy fall while locomotion quality degrades?
print(chr(10)+"  freeze-trap check (energy DOWN while flow_term UP = buying cheapness with motion):")
for p_, rows in rows_by_seed.items():
    seed = p_.split("_s")[-1].split(".")[0]
    if len(rows) < 6: continue
    h = len(rows) // 2
    e0 = st.mean(float(r.get("ge_energy", 0)) for r in rows[:h])
    e1 = st.mean(float(r.get("ge_energy", 0)) for r in rows[h:])
    f0 = st.mean(float(r.get("ge_flow", 0)) for r in rows[:h])
    f1 = st.mean(float(r.get("ge_flow", 0)) for r in rows[h:])
    flag = "  ** FREEZE WARNING **" if (e1 < e0 and f1 > f0) else ""
    print(f"    seed {seed}: energy {e0:.4f} -> {e1:.4f} ({e1-e0:+.4f})   "
          f"flow_term {f0:.4f} -> {f1:.4f} ({f1-f0:+.4f}){flag}")
print("  (also read amp_min / step_bal in the seedavg summary — a frozen or dead")
print("   leg is the other way this shows up, and the per-leg guard should block it)")
