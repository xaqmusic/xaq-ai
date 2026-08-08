#!/usr/bin/env python3
"""The (d) test in its sharpest form: DEGRADE A LEG mid-episode and ask whether the body
re-coordinates around it.

`picrawler_active_inference_plan.md` 2.8 and 10 both name this as the gait's locomotor-(d)
bar -- "shove -> gait re-forms; degrade a leg -> re-coordinates" -- and until now the only
perturbation this project could run was a teleport (humpavg / recoveravg), which relocates
the body without changing what the body IS.  A lesion is the stronger test: the world is
unchanged, so nothing external explains a behavioural change, and the agent can only notice
through its OWN prediction error.  The lesioned leg still senses itself perfectly; it simply
cannot push.

WHY THE WINDOWS, AND WHY plv_w RATHER THAN plv.  `interleg_plv` accumulates over the WHOLE
run, so a perturbation at tick 2500 is diluted by 6000 ticks of history and the recovery it
is supposed to score is invisible.  `plv_w` is the trailing-window twin (EMA phasor,
tau ~ 500 ticks) added 2026-08-04 for exactly this.  Measured reference lines, so a reading
can be judged instead of admired:

    plv_w random-phase null  0.090 +- 0.066   (independent legs, 40 sims)
    plv_w locked (a trot)    0.984
    plv_w deployed/nolearn2  0.200 +- 0.010

⚠ ALWAYS read plv_w beside plv_wn.  A frozen or fallen body holds a constant relative phase
trivially; plv_wn is the fraction of the window in which both legs of a pair were genuinely
oscillating, and |plv_w| <= plv_wn by construction.  A low plv_w with a low plv_wn means
"nothing was moving", NOT "the legs are uncoordinated".  This is the third route by which
the frozen-body degeneracy has entered this project's coordination metrics; the first two
both produced published numbers that had to be retracted.

⚠ THE RECOVERY FRACTION IS UNDEFINED IF THE LESION DID NOT BITE.  If plv_w does not fall
between the pre and acute windows there is nothing to recover, and reporting either 1.0
("fully recovered") or 0.0 ("never recovered") would be a fabrication.  Printed as `--`.

Usage: lesionavg.py <config.json> [n_seeds] [max_steps] [difficulty] [at] [leg] [scale]
"""
import json, math, os, statistics, subprocess, sys, concurrent.futures as cf
import pathlib

# Derived from this script's own location so a fresh clone works anywhere
# (was a hardcoded home directory, which broke every non-author checkout).
PROJ = str(pathlib.Path(__file__).resolve().parents[1])
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_lesion")
os.makedirs(SP, exist_ok=True)
WARMUP  = int(os.environ.get("SEEDAVG_WARMUP", 900))
ACUTE   = int(os.environ.get("LESION_ACUTE", 1500))   # ticks after the cut = the acute window
# 60 (the body default) gives ~25 samples per window, too thin for a mean.  Applied
# identically to every arm, so it cannot confound an A/B -- it only affects the JSONL emit.
DIAG_IV = int(os.environ.get("LESION_DIAG_INTERVAL", 20))


def run_one(cfg, seed, max_steps, difficulty, at, leg, scale):
    out = f"{SP}/les_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="corridor",
               OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7700 + seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous",
               OGMA_PICRAWLER_MAX_STEPS=str(max_steps),
               OGMA_PICRAWLER_DIAG_INTERVAL=str(DIAG_IV),
               OGMA_PICRAWLER_LESION_LEG=str(leg),
               OGMA_PICRAWLER_LESION_AT=str(at),
               OGMA_PICRAWLER_LESION_SCALE=str(scale))
    with open(out, "w") as fh:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60",
                        "--quit-after", "4000000", "--path", ".",
                        "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=fh, stderr=subprocess.STDOUT)
    return parse(out, at, leg)


def _win(rows, lo, hi):
    return [d for d in rows if lo <= d.get("t", 0) < hi]


def _mean(rows, key):
    v = [d[key] for d in rows if key in d and d[key] is not None]
    return statistics.mean(v) if v else float("nan")


def _leg_mean(rows, key, i):
    v = [d[key][i] for d in rows
         if isinstance(d.get(key), list) and len(d[key]) > i and d[key][i] >= 0.0]
    return statistics.mean(v) if v else float("nan")


def parse(path, at, leg):
    rows = []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith("{"):
            continue
        try:
            rows.append(json.loads(line))
        except Exception:
            continue
    if len(rows) < 20:
        return None
    wins = {"pre":  _win(rows, WARMUP, at),
            "acute": _win(rows, at, at + ACUTE),
            "recov": _win(rows, at + ACUTE, 10 ** 9)}
    if any(len(w) < 5 for w in wins.values()):
        return None
    r = {}
    for nm, w in wins.items():
        r[f"plv_w_{nm}"]  = _mean(w, "plv_w")
        r[f"plv_wn_{nm}"] = _mean(w, "plv_wn")
        r[f"tilt_{nm}"]   = statistics.pstdev([d.get("tilt", 0.0) for d in w]) if len(w) > 1 else 0.0
        r[f"cw_spr_{nm}"] = _mean(w, "cw_spr")
        # The lesioned leg's OWN residual.  This is the (a) "inferred, not oracle" evidence:
        # nothing tells the brain a leg was cut, so if it responds at all it responds to this.
        r[f"tle_cut_{nm}"]   = _leg_mean(w, "tle_leg", leg)
        others = [_leg_mean(w, "tle_leg", i) for i in range(4) if i != leg]
        others = [v for v in others if not math.isnan(v)]
        r[f"tle_other_{nm}"] = statistics.mean(others) if others else float("nan")
        r[f"amp_cut_{nm}"]   = _leg_mean(w, "amp_leg", leg)
        # Forward progress as a RATE, so windows of different length compare.
        dz = w[-1].get("z", 0.0) - w[0].get("z", 0.0)
        dt = max(1, w[-1].get("t", 1) - w[0].get("t", 1))
        r[f"dz_rate_{nm}"] = dz / dt * 60.0
    # Recovery fraction on the coordination read.  1.0 = fully returned to the pre-lesion
    # level, 0.0 = stayed at its worst.  UNDEFINED (nan) if the lesion produced no drop --
    # see the module docstring; a fabricated 1.0 here would be the most flattering possible
    # way to report a lever that did nothing.
    pre, ac, rc = r["plv_w_pre"], r["plv_w_acute"], r["plv_w_recov"]
    drop = pre - ac
    r["recov"] = ((rc - ac) / drop) if drop > 0.01 else float("nan")
    r["falls"] = max(d.get("auto_reset_count", 0) for d in rows)
    return r


def ms(vals):
    v = [x for x in vals if not math.isnan(x)]
    if not v:
        return float("nan"), float("nan")
    return statistics.mean(v), statistics.pstdev(v)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    cfg = sys.argv[1]
    n     = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 6000
    diff  = sys.argv[4] if len(sys.argv) > 4 else "0.3"
    at    = int(sys.argv[5]) if len(sys.argv) > 5 else 2500
    leg   = int(sys.argv[6]) if len(sys.argv) > 6 else 0
    scale = sys.argv[7] if len(sys.argv) > 7 else "0.2"
    seeds = list(range(1, n + 1))
    with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, steps, diff, at, leg, scale), seeds))
    ok = [r for r in res if r]
    print(f"\n{cfg}  (n={len(ok)}/{n}, {steps} ticks, diff {diff}, "
          f"LESION leg {leg} x{scale} @ tick {at}, acute window {ACUTE})")
    if not ok:
        print("  no valid runs"); sys.exit(1)
    ROWS = (("plv_w",     "coordination (null 0.090, trot 0.984)"),
            ("plv_wn",    "  ^ its support -- read plv_w ONLY beside this"),
            ("tle_cut",   "cut leg's own residual (should RISE)"),
            ("tle_other", "the other three legs' residual"),
            ("amp_cut",   "cut leg's oscillation amplitude"),
            ("cw_spr",    "weight spread (0 = the lever never fired)"),
            ("dz_rate",   "forward progress, m/s"),
            ("tilt",      "tilt sd"))
    print(f"  {'':<11}{'pre':>16}{'acute':>16}{'recovered':>16}   note")
    for key, note in ROWS:
        cells = []
        for nm in ("pre", "acute", "recov"):
            m, s = ms([r[f"{key}_{nm}"] for r in ok])
            cells.append("nan" if math.isnan(m) else f"{m:.3f}+-{s:.3f}")
        print(f"  {key:<11}" + "".join(f"{c:>16}" for c in cells) + f"   {note}")
    m, s = ms([r["recov"] for r in ok])
    nd = sum(1 for r in ok if math.isnan(r["recov"]))
    print(f"\n  ** RECOVERY FRACTION on plv_w: "
          + ("UNDEFINED on every seed -- the lesion produced no coordination drop, so there "
             "was nothing to recover" if math.isnan(m)
             else f"{m:+.3f} +- {s:.3f}   (1.0 = fully returned, 0.0 = stayed at its worst)"))
    if nd and not math.isnan(m):
        print(f"     ({nd}/{len(ok)} seeds undefined -- no drop -- and are EXCLUDED, not "
              f"counted as recoveries)")
    fm, fs = ms([float(r["falls"]) for r in ok])
    print(f"     falls {fm:.2f} +- {fs:.2f}")
