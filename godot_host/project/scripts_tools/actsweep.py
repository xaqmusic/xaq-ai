#!/usr/bin/env python3
"""Actuator authority sweep — the ledger's ★ open problem, run as a protocol.

The criterion is solved (value = responsiveness/(motor_tle+ε) ranks gaits at
corr +0.996 with net_z, 2026-08-07); what is missing is an ACTUATOR with
authority over it.  This tool sweeps candidate knobs and scores every knob on
corr(knob, hk_value) and corr(knob, net_z) TOGETHER — a knob that moves the
criterion and transport in the same direction is the thing the selector
finally attaches to.  A knob that moves neither has no authority (three of
last session's levers died of that, each measurable in ten minutes); a knob
that moves them in OPPOSITE directions is an amp_target-shaped trap (authority
without cause).

Usage:
  actsweep.py <base_config.json> [n=6] [steps=12000] [diff=0.3] key=v1,v2 [key=v1,v2 ...]

Each knob contributes arms at the listed values; the BASE config's own value
joins the series as the shared control arm (run once, reused for every knob's
correlation).  Arms run sequentially, seeds concurrently, so inspector ports
never collide.  Every run exports OGMA_PICRAWLER_CHASSIS_COLLIDE=1 (§4: any
belly-adjacent mechanism measured on the ghost chassis describes a different
body) and writes a per-seed attribution trace for offline per-leg analysis
(the fl-brake thread) without re-running anything.

Results land in $ACTSWEEP_OUT (default /tmp/xaq_actsweep): per-run body logs,
per-seed traces, arm summaries (arms.jsonl), and the final correlation table.
"""
import json, os, statistics, subprocess, sys, concurrent.futures as cf
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import seedavg                                   # parse() — the one honest metric reader

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
CFG  = os.path.join(PROJ, "addons/ami_ogma/configs")
OUT  = os.environ.get("ACTSWEEP_OUT", "/tmp/xaq_actsweep")
os.makedirs(OUT, exist_ok=True)


def make_arm(base, key, val):
    """mkarm-equivalent: one knob changed, the diff printed, tautologies refused."""
    with open(os.path.join(CFG, base)) as f:
        d = json.load(f)
    mods = [m for m in d["modules"] if m.get("type") in ("MotorEPM", "MotorEPMv2")]
    assert len(mods) == 1, f"expected 1 MotorEPM(v2), found {len(mods)}"
    p = mods[0]["params"]
    old = p.get(key, "<unset>")
    if old == val:
        return None                              # tautology — the control arm covers it
    p[key] = val
    name = f"zz_actsweep__{key}_{str(val).replace('.', 'p').replace('-', 'm')}.json"
    with open(os.path.join(CFG, name), "w") as f:
        json.dump(d, f, indent=2)
    print(f"    arm {name}: {key}: {json.dumps(old)} -> {json.dumps(val)}")
    return name


def run_seed(cfg, seed, steps, diff):
    log   = f"{OUT}/{os.path.splitext(cfg)[0]}_s{seed}.log"
    trace = f"{OUT}/{os.path.splitext(cfg)[0]}_s{seed}.trace.jsonl"
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7400 + seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(diff),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous",
               OGMA_PICRAWLER_MAX_STEPS=str(steps),
               OGMA_PICRAWLER_CHASSIS_COLLIDE="1",
               OGMA_PICRAWLER_TRACE=trace)
    with open(log, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60",
                        "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=f, stderr=subprocess.STDOUT)
    return seedavg.parse(log)


def run_arm(cfg, n, steps, diff):
    seeds = list(range(1, n + 1))
    with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
        res = list(ex.map(lambda s: run_seed(cfg, s, steps, diff), seeds))
    return [r for r in res if r]


def corr(x, y):
    n = len(x)
    if n < 3: return float("nan")
    mx, my = sum(x) / n, sum(y) / n
    sx = sum((v - mx) ** 2 for v in x) ** 0.5
    sy = sum((v - my) ** 2 for v in y) ** 0.5
    return (sum((x[k] - mx) * (y[k] - my) for k in range(n)) / (sx * sy)
            if sx * sy > 1e-12 else float("nan"))


REPORT = ("hk_value", "resp", "motor_tle", "net_z", "straight", "tilt_sd",
          "falls", "bellyc_min", "planted", "step_bal")

def main(argv):
    if len(argv) < 2:
        print(__doc__); return 2
    base = argv[0]
    pos, specs = [], []
    for a in argv[1:]:
        (specs if "=" in a else pos).append(a)
    n     = int(pos[0])   if len(pos) > 0 else 6
    steps = int(pos[1])   if len(pos) > 1 else 12000
    diff  = float(pos[2]) if len(pos) > 2 else 0.3
    knobs = {}                                   # key -> [values to add]
    for s in specs:
        k, _, raw = s.partition("=")
        knobs[k] = [json.loads(v) for v in raw.split(",")]

    with open(os.path.join(CFG, base)) as f:
        bp = [m for m in json.load(f)["modules"]
              if m.get("type") in ("MotorEPM", "MotorEPMv2")][0]["params"]

    arms = [("__control__", None, None, base)]   # (label, key, val, cfg)
    made = []
    print(f"actsweep: base={base}  n={n}  steps={steps}  diff={diff}")
    for k, vals in knobs.items():
        assert k in bp, f"knob {k} not in base config params — refusing a silent default"
        for v in vals:
            cfg = make_arm(base, k, v)
            if cfg is None:
                print(f"    arm {k}={v}: SKIPPED, equals the base (control covers it)")
                continue
            arms.append((f"{k}={v}", k, v, cfg))
            made.append(cfg)

    results = {}                                 # label -> (key, val, arm-mean dict, per-seed)
    armlog = open(f"{OUT}/arms.jsonl", "w")
    try:
        for i, (label, k, v, cfg) in enumerate(arms):
            print(f"\n[{i+1}/{len(arms)}] {label} ({cfg}) ...", flush=True)
            rs = run_arm(cfg, n, steps, diff)
            if not rs:
                print("    !! no valid runs"); continue
            mean = {m: statistics.mean([r[m] for r in rs]) for m in REPORT}
            std  = {m: statistics.pstdev([r[m] for r in rs]) for m in REPORT}
            results[label] = (k, v, mean, rs)
            armlog.write(json.dumps({"label": label, "key": k, "val": v, "n": len(rs),
                                     "mean": mean, "std": std,
                                     "seeds": [{m: r[m] for m in REPORT} for r in rs]}) + "\n")
            armlog.flush()
            print("    " + "  ".join(f"{m}={mean[m]:.3f}±{std[m]:.3f}" for m in
                                     ("hk_value", "net_z", "straight", "tilt_sd", "falls")))
    finally:
        armlog.close()
        for cfg in made:                          # never leave zz_ arms in the configs dir
            try: os.remove(os.path.join(CFG, cfg))
            except OSError: pass

    if "__control__" not in results:
        print("\n!! control arm failed — no correlations computed"); return 1
    _, _, cmean, _ = results["__control__"]

    print("\n" + "=" * 100)
    print(f"{'knob':<18}{'series (knob: hk_value / net_z)':<52}{'corr(k,val)':>12}{'corr(k,net_z)':>14}")
    print("-" * 100)
    for k, vals in knobs.items():
        series = [(bp[k], cmean)]
        for label, (kk, v, mean, _) in results.items():
            if kk == k: series.append((v, mean))
        series.sort(key=lambda t: t[0])
        if len(series) < 3:
            print(f"{k:<18}<3 arms survived — no correlation"); continue
        xs  = [t[0] for t in series]
        cv  = corr(xs, [t[1]["hk_value"] for t in series])
        cz  = corr(xs, [t[1]["net_z"]    for t in series])
        desc = "  ".join(f"{t[0]}: {t[1]['hk_value']:.2f}/{t[1]['net_z']:.2f}" for t in series)
        verdict = ("CANDIDATE" if cv == cv and cz == cz and cv * cz > 0
                   and abs(cv) > 0.7 and abs(cz) > 0.7 else "")
        print(f"{k:<18}{desc:<52}{cv:>12.3f}{cz:>14.3f}  {verdict}")
    print("=" * 100)
    print(f"\nlogs/traces/arms.jsonl in {OUT}")
    print("Screen only (n=%d fixed-seed = signal): a CANDIDATE needs a causal follow-up "
          "sweep before it is a finding — amp_target passed authority and failed cause." % n)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
