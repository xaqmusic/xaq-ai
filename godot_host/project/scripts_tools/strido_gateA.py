#!/usr/bin/env python3
"""PART V gate A — stride-odometry sweep + analysis.

Measures whether stance-foot FK velocity (the efference-copy half of the planned
stride_v sensor) tracks true forward velocity, across gaits and difficulties, and
whether the residual is gait-phase-locked (structured slip) or white.  The per-tick
sv_* series comes from the body's stride-odometry instrument via OGMA_PICRAWLER_TRACE
(picrawler_body.gd, 2026-08-25).

Gate A (promote-or-kill, from the PART V plan): judged variant is sv_clp (commanded
angles through the first-order servo forward model — what the encoder-less hardware
can build).  Pass = r >= ~0.8 against true forward velocity at criterion timescale
(~1 s windows) AND a structured (phase-locked) slip residual.  sv_meas is the sim-only
ceiling; sv_cmd (raw commands) is kept because its failure is itself a finding.

Usage:
  python3 strido_gateA.py run  [outdir]      # ~27 runs, ~6 concurrent, ~25 min
  python3 strido_gateA.py analyze <outdir>

⚠ Seeds vary via OGMA_SEED (the master override — a config's own `seed` param is
NOT enough; see the 2026-08-24 n=1-disguised-as-n=3 trap).
"""
import concurrent.futures as cf
import json, os, pathlib, statistics, subprocess, sys, time

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
CFG_J1S4 = "the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__j1s4.json"
CFG_C0   = "the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__j1s4_c0.json"
CFG_V3   = "the_picrawler_motor_epm_embed_corridor_v3base.json"

# (arm name, config, difficulty, seeds).  j1s4 = the deployed stack across the
# difficulty range; v3base = a different gait (pre-evolver stack) for gait diversity;
# c0 = coupling zeroed in BOTH motor_epm and the frozen evolver seed = the PART IV
# shuffling basin, i.e. stepping (mostly) in place — the blind-metric immunity probe.
ARMS = [
    ("j1s4_d00", CFG_J1S4, 0.0, range(1, 7)),
    ("j1s4_d03", CFG_J1S4, 0.3, range(1, 7)),
    ("j1s4_d06", CFG_J1S4, 0.6, range(1, 7)),
    ("v3base_d03", CFG_V3, 0.3, range(1, 7)),
    ("c0_d00", CFG_C0, 0.0, range(1, 4)),
]
MAX_STEPS = 12000
TAU = 0.02          # brain tick (s)
WARMUP = 900        # ticks skipped for correlation stats (spawn settle; seedavg convention)
WINDOWS = (1, 10, 25, 50)
PHASE_BINS = 12
CONCURRENCY = 6


def run_one(arm, cfg, diff, seed, outdir, port):
    log = f"{outdir}/{arm}_s{seed}.log"
    trace = f"{outdir}/{arm}_s{seed}.trace.jsonl"
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(diff),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(MAX_STEPS),
               OGMA_PICRAWLER_TRACE=trace)
    t0 = time.time()
    with open(log, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=f, stderr=subprocess.STDOUT)
    # Silent-confound check (§3.2 #7): the run must announce the config it loaded.
    loaded = any(cfg in line for line in open(log, errors="replace"))
    print(f"  {arm} s{seed}: {time.time()-t0:.0f}s  config_confirmed={loaded}", flush=True)
    return loaded


def cmd_run(outdir):
    os.makedirs(outdir, exist_ok=True)
    jobs = [(arm, cfg, diff, s, outdir, 7400 + k)
            for k, (arm, cfg, diff, s) in enumerate(
                (arm, cfg, diff, s) for arm, cfg, diff, seeds in ARMS for s in seeds)]
    print(f"gate A sweep: {len(jobs)} runs x {MAX_STEPS} ticks -> {outdir}")
    with cf.ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        ok = list(ex.map(lambda j: run_one(*j), jobs))
    print(f"done: {sum(ok)}/{len(ok)} config-confirmed")


# ---------------------------------------------------------------------------


def corr(a, b):
    n = len(a)
    if n < 3:
        return float("nan")
    ma, mb = sum(a) / n, sum(b) / n
    sa = (sum((x - ma) ** 2 for x in a) / n) ** 0.5
    sb = (sum((x - mb) ** 2 for x in b) / n) ** 0.5
    if sa == 0 or sb == 0:
        return float("nan")
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / n / (sa * sb)


def slope(fk, tr):
    """OLS slope of fk on true — the calibration factor a consumer would need."""
    n = len(tr)
    mt = sum(tr) / n
    var = sum((y - mt) ** 2 for y in tr) / n
    if var == 0:
        return float("nan")
    mf = sum(fk) / n
    return sum((x - mf) * (y - mt) for x, y in zip(fk, tr)) / n / var


def windowed(pairs, w):
    """Non-overlapping w-tick means over (fk, true) pairs of USABLE ticks."""
    fs, ts = [], []
    cf_, ct, n = 0.0, 0.0, 0
    for f, t in pairs:
        cf_ += f
        ct += t
        n += 1
        if n == w:
            fs.append(cf_ / w)
            ts.append(ct / w)
            cf_ = ct = 0.0
            n = 0
    return fs, ts


def phase_profile(rows, usable_idx):
    """Fraction of slip-residual variance explained by gait phase (FL touchdowns),
    plus a circular-shift null (same tick positions, residual series rotated —
    preserves the marginal distribution, destroys the phase alignment).  Structured
    slip = true ratio well above the null."""
    idx = sorted(usable_idx)
    pos = {t: k for k, t in enumerate(idx)}
    series = [rows[t]["sv_clp"][1] - rows[t]["sv_true"][1] for t in idx]
    # FL touchdown ticks (contact 0 -> 1)
    tds = [i for i in range(1, len(rows)) if rows[i]["c"][0] == 1 and rows[i - 1]["c"][0] == 0]
    strides = [(a, b) for a, b in zip(tds, tds[1:]) if 5 <= b - a <= 400]

    def ratio(shift):
        bins = [[] for _ in range(PHASE_BINS)]
        vals = []
        for a, b in strides:
            for i in range(a, b):
                if i in pos:
                    v = series[(pos[i] + shift) % len(series)]
                    bins[int(PHASE_BINS * (i - a) / (b - a))].append(v)
                    vals.append(v)
        if len(vals) < 50:
            return float("nan")
        mv = sum(vals) / len(vals)
        tot = sum((v - mv) ** 2 for v in vals) / len(vals)
        if tot == 0:
            return float("nan")
        expl = sum(len(b) * (sum(b) / len(b) - mv) ** 2 for b in bins if b) / len(vals)
        return expl / tot

    if not series:
        return float("nan"), float("nan"), len(strides)
    r_true = ratio(0)
    nulls = [ratio(k * len(series) // 7 + 13) for k in range(1, 4)]
    nulls = [x for x in nulls if x == x]
    return r_true, (sum(nulls) / len(nulls) if nulls else float("nan")), len(strides)


def analyze_run(trace_path):
    rows = [json.loads(l) for l in open(trace_path)]
    body = [r for r in rows if r["t"] >= WARMUP]
    out = {"ticks": len(rows)}
    for key, gate in (("sv_clp", 0), ("sv_cmd", 0), ("sv_meas", 0), ("sv_tc", 1)):
        pairs = [(r[key][1], r["sv_true"][1]) for r in body if r["sv_ns"][gate] > 0]
        out[key] = {"n": len(pairs), "mean_fk": statistics.mean(p[0] for p in pairs) if pairs else float("nan"),
                    "mean_true": statistics.mean(p[1] for p in pairs) if pairs else float("nan")}
        for w in WINDOWS:
            fs, ts = windowed(pairs, w)
            out[key][f"r_w{w}"] = corr(fs, ts)
        fs, ts = windowed(pairs, 50)
        out[key]["slope_w50"] = slope(fs, ts)
    usable = [i for i, r in enumerate(rows) if r["t"] >= WARMUP and r["sv_ns"][0] > 0]
    pl, pl_null, n_strides = phase_profile(rows, set(usable))
    out["phase_lock"] = pl
    out["phase_lock_null"] = pl_null
    out["n_strides"] = n_strides
    # Odometer view: integrated FK forward travel vs integrated true forward travel.
    out["odo_fk"] = sum(rows[i]["sv_clp"][1] for i in usable) * TAU
    out["odo_true"] = sum(rows[i]["sv_true"][1] for i in usable) * TAU
    # Step activity (for the c0 arm): sd of FL hip1 command — are the legs moving?
    h1 = [r["a1"][0] for r in body]
    out["h1_sd"] = statistics.pstdev(h1) if len(h1) > 10 else float("nan")
    return out


def agg(vals):
    vals = [v for v in vals if v == v]
    if not vals:
        return "  nan    "
    if len(vals) == 1:
        return f"{vals[0]:+.3f}    "
    return f"{statistics.mean(vals):+.3f}±{statistics.stdev(vals):.3f}"


def cmd_analyze(outdir):
    per_run = {}
    for arm, cfg, diff, seeds in ARMS:
        for s in seeds:
            p = f"{outdir}/{arm}_s{s}.trace.jsonl"
            if os.path.exists(p):
                per_run[(arm, s)] = analyze_run(p)
    json.dump({f"{a}_s{s}": v for (a, s), v in per_run.items()},
              open(f"{outdir}/gateA_analysis.json", "w"), indent=1)
    print(f"per-run detail -> {outdir}/gateA_analysis.json\n")
    hdr = f"{'arm':10s} {'n':>2s}  {'r_w1(clp)':>12s} {'r_w50(clp)':>12s} {'r_w50(cmd)':>12s} {'r_w50(meas)':>12s} {'slope(clp)':>12s} {'phlock':>12s} {'null':>12s}"
    print(hdr)
    for arm, cfg, diff, seeds in ARMS:
        rs = [per_run[(arm, s)] for s in seeds if (arm, s) in per_run]
        if not rs:
            continue
        print(f"{arm:10s} {len(rs):2d}  "
              f"{agg([r['sv_clp']['r_w1'] for r in rs]):>12s} "
              f"{agg([r['sv_clp']['r_w50'] for r in rs]):>12s} "
              f"{agg([r['sv_cmd']['r_w50'] for r in rs]):>12s} "
              f"{agg([r['sv_meas']['r_w50'] for r in rs]):>12s} "
              f"{agg([r['sv_clp']['slope_w50'] for r in rs]):>12s} "
              f"{agg([r['phase_lock'] for r in rs]):>12s} "
              f"{agg([r['phase_lock_null'] for r in rs]):>12s}")
    print("\nodometer view (integrated forward travel, m) + step activity:")
    for arm, cfg, diff, seeds in ARMS:
        rs = [per_run[(arm, s)] for s in seeds if (arm, s) in per_run]
        if not rs:
            continue
        print(f"{arm:10s}  odo_fk {agg([r['odo_fk'] for r in rs])}  "
              f"odo_true {agg([r['odo_true'] for r in rs])}  h1_sd {agg([r['h1_sd'] for r in rs])}")


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "run":
        cmd_run(sys.argv[2] if len(sys.argv) > 2 else
                os.path.expanduser(time.strftime("~/xaq_runs/stridoA_%Y%m%d")))
    elif len(sys.argv) >= 3 and sys.argv[1] == "analyze":
        cmd_analyze(sys.argv[2])
    else:
        print(__doc__)
