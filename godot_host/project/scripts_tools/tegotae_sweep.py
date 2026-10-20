#!/usr/bin/env python3
"""Tegotae replication harness: run arms of the LoadPhaseGait experiments and read the GAIT.

Plan: docs/plans-and-designs/tegotae_replication_plan.md.  The rule under test is Owaki &
Ishiguro's dphi_i/dt = omega - sigma N_i cos phi_i (Sci. Rep. 7:277, 2017).  Their result is
about COORDINATION -- which gait four uncoupled legs settle into, how it changes with omega,
and whether it survives a change of body -- so this harness reads coordination first and
transport second, which no other tool here does:

  * the inter-leg relative phases at TOUCHDOWN (the physics contact flag), relative to the
    front-left foot, per stride, and the gait each stride matches (lateral- and
    diagonal-sequence walk, trot, pace, bound, pronk, or irregular);
  * the same read off the oscillators themselves (lp_ph), so a phase pattern that never
    reaches the feet is visible as a disagreement between the two;
  * the "hold": the fraction of stance ticks a leg's phase is (nearly) stopped by its load --
    Owaki's excitatory regime -- and the retrograde fraction;
  * WHERE A LIFTED FOOT'S WEIGHT GOES: at each liftoff, the change in the other feet's load
    and in the chassis's own ground load (`bload`, a god's-eye instrument).  The rule's
    "physical communication" exists only if the first is large;
  * the belly's share of the body weight, the duty factor, the stride period, speed, the
    Froude number on hip height, falls and tilt.

Arms are written as configs under <out>/configs (never into the project's config dir), each
printed as its diff against the base, and every run is checked for the arm's params having
LANDED (OGMA_DUMP_PARAMS) before its numbers count (CLAUDE.md 3.2 rule 7).

Usage:
  tegotae_sweep.py run --base <config.json> --out <dir> [--gym arena] [--seeds 1-6]
                   [--steps 4500] [--difficulty 0.3] [--jobs 8] [--env K=V ...]
                   --arm NAME[:module.param=json[,module.param=json...]][;ENV=K=V...]  (repeatable)
  tegotae_sweep.py analyse <log> [<log> ...]          # any per-tick log with lp_ph/con
  An arm with no ':' is the base itself.  Arm-level env: NAME:mods;ENV=K=V;ENV=K2=V2.
"""
import argparse, concurrent.futures as cf, copy, glob, json, math, os, pathlib, statistics, subprocess, sys

PROJ = pathlib.Path(__file__).resolve().parents[1]
CFGDIR = PROJ / "addons/ami_ogma/configs"
TAU = 0.02                     # brain tick (s): picrawler_body.gd TAU
G = 9.81
WARMUP = int(os.environ.get("TEGOTAE_WARMUP", 900))
LEGS = ["FL", "FR", "RL", "RR"]
# Relative touchdown phase of (FR, RL, RR) against FL, in cycles.  Owaki's L-S walk is
# RH, RF, LH, LF: against LF = 0 that is RH .25, RF .5, LH .75.
TEMPLATES = {
    "LS-walk": (0.50, 0.75, 0.25),
    "DS-walk": (0.50, 0.25, 0.75),
    "trot":    (0.50, 0.50, 0.00),
    "pace":    (0.50, 0.00, 0.50),
    "bound":   (0.00, 0.50, 0.50),
    "pronk":   (0.00, 0.00, 0.00),
}
MATCH_TOL = 0.125              # every leg within an eighth of a cycle of the template


def cdist(a, b):
    d = (a - b) % 1.0
    return min(d, 1.0 - d)


def classify(rel):
    best, bd = "irregular", 9.0
    for name, t in TEMPLATES.items():
        errs = [cdist(r, x) for r, x in zip(rel, t)]
        if max(errs) <= MATCH_TOL and sum(errs) < bd:
            best, bd = name, sum(errs)
    return best


def circ(vals):
    """mean angle (cycles in [0,1)) and resultant length of phases given in cycles."""
    if not vals:
        return float("nan"), 0.0
    c = sum(math.cos(2 * math.pi * v) for v in vals) / len(vals)
    s = sum(math.sin(2 * math.pi * v) for v in vals) / len(vals)
    return (math.atan2(s, c) / (2 * math.pi)) % 1.0, math.hypot(c, s)


# ---------------------------------------------------------------------------------- analysis
def analyse(path, period_ticks=None):
    rows, params = [], {}
    for line in open(path, errors="replace"):
        if not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if d.get("event") == "PARAMS":
            params[d.get("module")] = d.get("params", {})
            continue
        if "x" in d and "t" in d and "con" in d:     # the diag line proper (a few early lines lack it)
            rows.append(d)
    if period_ticks is None:
        for p in params.values():
            if "period_ticks" in p and "sigma_ratio" in p:
                period_ticks = float(p["period_ticks"])
    out = {"log": os.path.basename(path), "ticks": len(rows)}
    if len(rows) < WARMUP + 200:
        out["error"] = "too short"
        return out
    ar = [r.get("auto_reset_count", 0) for r in rows]
    out["falls"] = max(ar) - ar[WARMUP]
    rows = rows[WARMUP:]
    n = len(rows)
    # -- transport (split at resets so a teleport is not distance)
    path_len, segs, seg0 = 0.0, [], 0
    for i in range(1, n):
        if rows[i].get("auto_reset_count", 0) != rows[i - 1].get("auto_reset_count", 0):
            segs.append((seg0, i - 1)); seg0 = i
            continue
        path_len += math.hypot(rows[i]["x"] - rows[i - 1]["x"], rows[i]["z"] - rows[i - 1]["z"])
    segs.append((seg0, n - 1))
    net = sum(math.hypot(rows[b]["x"] - rows[a]["x"], rows[b]["z"] - rows[a]["z"]) for a, b in segs)
    T = n * TAU
    out["net_disp"] = net
    out["straight"] = net / path_len if path_len > 1e-6 else 0.0
    out["speed"] = net / T
    ys = [r.get("y", 0.0) for r in rows]
    out["hip_h"] = statistics.mean(ys)
    out["froude"] = out["speed"] ** 2 / (G * max(1e-3, out["hip_h"]))
    if "swl_mult" in rows[-1]:
        out["swl_mult"] = rows[-1]["swl_mult"]          # MotorEPMv2's own EMA of the swing-load gate
    tl = [r.get("tilt", 0.0) for r in rows]
    out["tilt_sd"] = statistics.pstdev(tl)
    # -- contact: duty, touchdowns, stride
    have_con = all("con" in r for r in rows)
    if have_con:
        con = [[1 if r["con"][k] > 0.5 else 0 for k in range(4)] for r in rows]
        out["duty"] = [sum(c[k] for c in con) / n for k in range(4)]
        td = [[i for i in range(1, n) if con[i][k] and not con[i - 1][k]] for k in range(4)]
        lo = [[i for i in range(1, n) if not con[i][k] and con[i - 1][k]] for k in range(4)]
        out["touchdowns"] = [len(x) for x in td]
        fl = td[0]
        rels, labels = [], []
        for a, b in zip(fl, fl[1:]):
            L = b - a
            if L < 5 or L > 400:          # a stall or a flicker is not a stride
                labels.append("irregular"); continue
            rel = []
            for k in (1, 2, 3):
                inside = [t for t in td[k] if a <= t < b]
                if len(inside) != 1:
                    rel = None; break
                rel.append((inside[0] - a) / L)
            if rel is None:
                labels.append("irregular"); continue
            rels.append(rel); labels.append(classify(rel))
        out["strides"] = len(labels)
        out["stride_ticks"] = statistics.median([b - a for a, b in zip(fl, fl[1:])]) if len(fl) > 2 else float("nan")
        hist = {}
        for l in labels:
            hist[l] = hist.get(l, 0) + 1
        out["gait_hist"] = {k: v / max(1, len(labels)) for k, v in sorted(hist.items(), key=lambda kv: -kv[1])}
        out["gait"] = max(hist, key=hist.get) if hist else "none"
        out["rel_td"] = [circ([r[j] for r in rels]) for j in range(3)]   # (mean, R) per FR, RL, RR
        out["rel_td_label"] = classify(tuple(m for m, _ in out["rel_td"])) if rels else "none"
        # -- where a lifted foot's weight goes
        if all("fload" in r for r in rows):
            fload = [r["fload"] for r in rows]
            bload = [r.get("bload", 0.0) for r in rows]
            tot = [sum(f) + b for f, b in zip(fload, bload)]
            out["belly_frac"] = statistics.mean(b / t for b, t in zip(bload, tot) if t > 0.05) if any(t > 0.05 for t in tot) else float("nan")
            out["feet_bw"] = statistics.mean(sum(f) for f in fload)
            to_feet, to_belly, lost = 0.0, 0.0, 0.0
            for k in range(4):
                for i in lo[k]:
                    if i < 12 or i + 12 >= n:
                        continue
                    pre = range(i - 10, i - 2); post = range(i + 3, i + 11)
                    dl = statistics.mean(fload[j][k] for j in pre) - statistics.mean(fload[j][k] for j in post)
                    if dl <= 0.02:
                        continue
                    others_pre = statistics.mean(sum(fload[j][m] for m in range(4) if m != k) for j in pre)
                    others_post = statistics.mean(sum(fload[j][m] for m in range(4) if m != k) for j in post)
                    b_pre = statistics.mean(bload[j] for j in pre); b_post = statistics.mean(bload[j] for j in post)
                    lost += dl; to_feet += others_post - others_pre; to_belly += b_post - b_pre
            out["xfer_feet"] = to_feet / lost if lost > 0 else float("nan")
            out["xfer_belly"] = to_belly / lost if lost > 0 else float("nan")
    # -- the hold ceiling: a leg held at the rule's fixed point leaves only when sigma_r * N * cos phi < 1
    #    for the rest of its stance, so 1 / max over stance of mean(N | phi) * cos phi is the strongest
    #    hold this body's own load profile can release (sigma_r in body weights, load_mode 1).
    if all("lp_ph" in r and "fload" in r for r in rows):
        nb = 8
        acc = [[[0.0, 0] for _ in range(nb)] for _ in range(4)]
        for r in rows:
            for k in range(4):
                p = r["lp_ph"][k]
                if math.sin(p) < 0 and r["con"][k] > 0.5:
                    b = min(nb - 1, max(0, int((p - math.pi) / math.pi * nb)))
                    acc[k][b][0] += r["fload"][k]; acc[k][b][1] += 1
        worst = 0.0
        prof = []
        for k in range(4):
            row = []
            for b in range(nb):
                m = acc[k][b][0] / acc[k][b][1] if acc[k][b][1] >= 20 else float("nan")
                row.append(m)
                c = math.cos(math.pi + (b + 0.5) * math.pi / nb)
                if not math.isnan(m) and c > 0:
                    worst = max(worst, m * c)
            prof.append(row)
        out["late_load"] = statistics.mean(x for row in prof for x in row[-2:] if not math.isnan(x)) if any(not math.isnan(x) for row in prof for x in row[-2:]) else float("nan")
        out["sigma_crit"] = 1.0 / worst if worst > 0 else float("nan")
    # -- each leg's OWN corner: e = front*sin(pitch) + left*sin(roll) (MotorEPMv2's level-reflex convention,
    #    code leg names; + = the corner is low), averaged over the leg's own mid-swing and mid-stance.
    if all("lp_ph" in r and "tilt_p" in r for r in rows):
        FRONT, LEFT = (1, 1, -1, -1), (1, -1, 1, -1)
        sw = [[] for _ in range(4)]; stc = [[] for _ in range(4)]
        for r in rows:
            for k in range(4):
                e = FRONT[k] * r["tilt_p"] + LEFT[k] * r["tilt_r"]
                sn = math.sin(r["lp_ph"][k])
                if sn > 0.7: sw[k].append(e)
                elif sn < -0.7: stc[k].append(e)
        out["dip_swing"] = [statistics.mean(x) if x else float("nan") for x in sw]
        out["dip_stance"] = [statistics.mean(x) if x else float("nan") for x in stc]
    # -- the oscillators
    if all("lp_ph" in r for r in rows):
        ph = [r["lp_ph"] for r in rows]
        osc_rel = []
        for j in (1, 2, 3):
            osc_rel.append(circ([((p[j] - p[0]) / (2 * math.pi)) % 1.0 for p in ph]))
        out["rel_osc"] = osc_rel
        out["rel_osc_label"] = classify(tuple(m for m, _ in osc_rel))
        if period_ticks:
            w = 2 * math.pi / period_ticks
            hold = stance = retro = tot = 0
            for i in range(1, n):
                for k in range(4):
                    d = ph[i][k] - ph[i - 1][k]
                    if d > math.pi: d -= 2 * math.pi
                    if d < -math.pi: d += 2 * math.pi
                    tot += 1
                    if d < 0: retro += 1
                    if math.sin(ph[i][k]) < 0:
                        stance += 1
                        if d < 0.25 * w: hold += 1
            out["hold"] = hold / max(1, stance)
            out["retro"] = retro / max(1, tot)
            out["omega_rel"] = None
    return out


SCALARS = ["net_disp", "straight", "speed", "froude", "falls", "tilt_sd", "hip_h", "stride_ticks",
           "strides", "hold", "retro", "late_load", "sigma_crit", "swl_mult", "belly_frac", "feet_bw", "xfer_feet", "xfer_belly"]


def fmt_ms(vals):
    vals = [v for v in vals if isinstance(v, (int, float)) and not math.isnan(v)]
    if not vals:
        return "—"
    m = statistics.mean(vals)
    s = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    return f"{m:.3f} ± {s:.3f}" if abs(m) < 10 else f"{m:.1f} ± {s:.1f}"


def summarise(name, res):
    good = [r for r in res if "error" not in r]
    print(f"\n=== {name}  (n={len(good)}{', ' + str(len(res) - len(good)) + ' bad' if len(res) != len(good) else ''})")
    if not good:
        return {}
    summ = {}
    for k in SCALARS:
        vals = [r.get(k) for r in good]
        summ[k] = vals
        print(f"  {k:13s} {fmt_ms(vals)}")
    if "duty" in good[0]:
        print("  duty FL/FR/RL/RR  " + " / ".join(f"{statistics.mean(r['duty'][k] for r in good):.2f}" for k in range(4)))
        print("  touchdowns        " + " / ".join(f"{statistics.mean(r['touchdowns'][k] for r in good):.0f}" for k in range(4)))
    if "dip_swing" in good[0]:
        print("  corner dip, own swing  (x1000) " + " ".join(f"{1000 * statistics.mean(r['dip_swing'][k] for r in good):+5.0f}" for k in range(4)))
        print("  corner dip, own stance (x1000) " + " ".join(f"{1000 * statistics.mean(r['dip_stance'][k] for r in good):+5.0f}" for k in range(4)))
    agg = {}
    for r in good:
        for g, f in r.get("gait_hist", {}).items():
            agg[g] = agg.get(g, 0.0) + f / len(good)
    if agg:
        print("  strides by gait   " + ", ".join(f"{g} {100 * f:.0f}%" for g, f in sorted(agg.items(), key=lambda kv: -kv[1])))
    for key, lab in (("rel_td", "touchdown"), ("rel_osc", "oscillator")):
        if key in good[0]:
            parts = []
            for j, leg in enumerate(("FR", "RL", "RR")):
                m, R = circ([r[key][j][0] for r in good if not math.isnan(r[key][j][0])])
                parts.append(f"{leg} {m:.2f} (R {statistics.mean(r[key][j][1] for r in good):.2f})")
            labs = [r.get(key + "_label") for r in good]
            print(f"  rel. phase {lab:10s} " + "  ".join(parts) + f"   per-seed: {', '.join(labs)}")
    summ["gaits"] = agg
    return summ


# ------------------------------------------------------------------------------------- runs
def parse_arm(spec):
    env = {}
    if ";" in spec:
        spec, *envs = spec.split(";")
        for e in envs:
            assert e.startswith("ENV="), e
            k, _, v = e[4:].partition("=")
            env[k] = v
    name, _, mods = spec.partition(":")
    sets = []
    if mods:
        parts, depth, cur = [], 0, ""
        for ch in mods:                       # split on commas outside [...] so array values survive
            if ch == "[": depth += 1
            elif ch == "]": depth -= 1
            if ch == "," and depth == 0:
                parts.append(cur); cur = ""
            else:
                cur += ch
        parts.append(cur)
        for kv in parts:
            lhs, _, val = kv.partition("=")
            mod, _, key = lhs.partition(".")
            sets.append((mod, key, json.loads(val)))
    return name, sets, env


def write_arm(base_cfg, name, sets, outdir, drop=()):
    cfg = copy.deepcopy(base_cfg)
    print(f"--- arm {name}")
    for mid in drop:
        if not any(m.get("id") == mid for m in cfg["modules"]):
            print(f"    module {mid}: not in the base (already absent)")
            continue
        cfg["modules"] = [m for m in cfg["modules"] if m.get("id") != mid]
        print(f"    module {mid}: DROPPED")
    ids = {m.get("id"): m for m in cfg["modules"]}
    for mod, key, val in sets:
        if mod not in ids:
            sys.exit(f"arm {name}: no module id '{mod}' in the base (have {sorted(ids)})")
        old = ids[mod].setdefault("params", {}).get(key, "<unset>")
        if old == val:
            print(f"    ⚠ {mod}.{key} already {val!r} in the base — TAUTOLOGY")
        ids[mod]["params"][key] = val
        print(f"    {mod}.{key}: {old!r} -> {val!r}")
    cfg["metadata"]["name"] = f"TEGOTAE arm {name}"
    p = outdir / "configs" / f"{name}.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    json.dump(cfg, open(p, "w"), indent=1)
    return p


def run_one(cfgpath, body_env, arm_env, extra_env, seed, steps, gym, difficulty, logpath, port):
    env = dict(os.environ, OGMA_PICRAWLER_GYM=gym, OGMA_SEED=str(seed), OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty), OGMA_PICRAWLER_CONFIG=str(cfgpath),
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(steps),
               OGMA_PICRAWLER_DIAG_INTERVAL="1", OGMA_PICRAWLER_LOAD_LOG="1", OGMA_DUMP_PARAMS="1")
    for k, v in body_env.items():
        if k not in os.environ:
            env[k] = v
    env.update(extra_env); env.update(arm_env)
    with open(logpath, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return logpath


def check_landed(logpath, sets):
    params = {}
    seed_line = None
    for line in open(logpath, errors="replace"):
        if line.startswith('{"event": "PARAMS"') or line.startswith('{"event":"PARAMS"'):
            try:
                d = json.loads(line); params[d["module"]] = d.get("params", {})
            except Exception:
                pass
        elif '"seed"' in line and '"WORLD_READY"' in line:
            seed_line = line.strip()
    bad = []
    for mod, key, val in sets:
        got = params.get(mod, {}).get(key, "<missing>")
        if isinstance(val, (int, float)) and isinstance(got, (int, float)):
            if abs(float(got) - float(val)) > 1e-9:
                bad.append(f"{mod}.{key}={got!r} (wanted {val!r})")
        elif got != val:
            bad.append(f"{mod}.{key}={got!r} (wanted {val!r})")
    return bad, seed_line


def seeds_arg(s):
    if "-" in s:
        a, b = s.split("-"); return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--base", required=True); r.add_argument("--out", required=True)
    r.add_argument("--gym", default="arena"); r.add_argument("--seeds", default="1-6", type=seeds_arg)
    r.add_argument("--steps", type=int, default=4500); r.add_argument("--difficulty", type=float, default=0.3)
    r.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    r.add_argument("--env", action="append", default=[]); r.add_argument("--arm", action="append", required=True)
    r.add_argument("--drop-module", action="append", default=[],
                   help="remove a module (by config id) from EVERY arm, e.g. gain_evolver")
    a = sub.add_parser("analyse"); a.add_argument("logs", nargs="+")
    args = ap.parse_args()

    if args.cmd == "analyse":
        res = [analyse(p) for g in args.logs for p in sorted(glob.glob(g))]
        summarise("logs", res)
        return

    out = pathlib.Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    base_path = pathlib.Path(args.base)
    if not base_path.is_absolute():
        base_path = CFGDIR / args.base
    base_cfg = json.load(open(base_path))
    body_env = {str(k): str(v) for k, v in base_cfg.get("metadata", {}).get("body_env", {}).items()}
    extra_env = dict(kv.split("=", 1) for kv in args.env)
    print(f"base {base_path.name}\ndropped modules {args.drop_module}\nbody_env {body_env}\nextra env {extra_env}\nseeds {args.seeds}  steps {args.steps}  gym {args.gym}")
    arms = []
    for spec in args.arm:
        name, sets, arm_env = parse_arm(spec)
        arms.append((name, write_arm(base_cfg, name, sets, out, args.drop_module), sets, arm_env))
        if arm_env:
            print(f"    env {arm_env}")
    jobs = []
    port = 7800
    with cf.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for name, cfgp, sets, arm_env in arms:
            (out / "logs" / name).mkdir(parents=True, exist_ok=True)
            for s in args.seeds:
                port += 2          # the inspector binds port and port + 1
                lp = out / "logs" / name / f"s{s}.log"
                jobs.append((name, sets, s, ex.submit(run_one, cfgp, body_env, arm_env, extra_env, s, args.steps,
                                                     args.gym, args.difficulty, lp, port)))
        results = {}
        for name, sets, s, fut in jobs:
            lp = fut.result()
            bad, seed_line = check_landed(lp, sets)
            if bad:
                print(f"  ✗ {name} s{s}: LEVER DID NOT LAND: {bad}")
            res = analyse(lp)
            res["seed"] = s; res["landed"] = not bad; res["world"] = seed_line
            results.setdefault(name, []).append(res)
    summary = {}
    for name, *_ in arms:
        summary[name] = summarise(name, [r for r in results[name] if r["landed"]])
    json.dump({"args": vars(args), "results": results}, open(out / "results.json", "w"), indent=1, default=str)
    print(f"\nresults: {out / 'results.json'}")


if __name__ == "__main__":
    main()
