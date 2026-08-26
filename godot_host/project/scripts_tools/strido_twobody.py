#!/usr/bin/env python3
"""PART V follow-on — the TWO-BODY study: same criterion, same seeds, different body.

    python3 strido_twobody.py landscape [outdir]           # E1: 12 runs, ~2.5 h
    python3 strido_twobody.py analyze-landscape <outdir>   # E1 gate read
    python3 strido_twobody.py search [outdir]              # E2: 24 runs, ~11 h
                                                           #  (needs make_arms d2 configs)

The embodiment claim made falsifiable: if the evolver's discovered gains are
properties of the BODY, two bodies under an identical criterion, seed stream and
displacement must land in basins that each match their OWN body's landscape.
Bodies: `cad` (the body as designed — every ledger result through PART V ran on
it) and `measured` (the body as built, 2026-08-10: hip_z_span +15%, l1 −14%,
l2 −10%, chassis mass +14%, real CoM + chassis boxes).  Selection is
OGMA_PICRAWLER_BODY per arm; every log carries a GEOMETRY RECEIPT line and the
runner checks it (§3.2 silent-confound rule).

E1 (landscape): the D1 protocol in the ARENA (the search phase's gym — landscape
and search must share an environment): σ=0 observer on `j1s4_c2` (the reference
criterion), SETPARAM_AT steps motor_epm.coupling_gain through 6 levels × 3
scored 12k windows, seeds 1–6 odd-asc/even-desc, per body.  NOTE the other
gains sit at the j1s4 point (tuned on cad), so these are CONDITIONAL landscapes
at the shared operating point — which is exactly the operating point E2 starts
from.  Window terms are attributed to the FIRST diag line after the ge_wt reset
(the 2026-08-25 attribution fix).

E1 GATE (declared before the data): proceed to E2 only if the bodies' level
profiles differ beyond noise — some level's between-body |ΔJ| > 2× the pooled
same-level window noise for that comparison — OR the argmin differs by ≥ 2
levels.  Otherwise the search comparison is unpowered by construction: stop,
report, and consider an amplified third body.

E2 (search): the D2 arms per body — sigma0 (displaced control) + fix020 × seeds
1–6.  fix008 is dropped: measured too slow to climb twice (tsw2 and D2), its
σ-floor re-use context unchanged.  Cross-predictions are registered from the E1
landscapes BEFORE E2 launches.
"""
import concurrent.futures as cf
import glob, json, os, pathlib, statistics as st, subprocess, sys, time

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
CFGDIR = os.path.join(PROJ, "addons", "ami_ogma", "configs")
CFG_C2 = ("the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose"
          "__m1auth__planpull__j1s4_c2.json")
CFG_3D = ("the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose"
          "__m1auth__planpull__j1s4_3d.json")
BODIES = ["cad", "measured"]
LEVELS_ASC = [0.0, 0.4, 0.8, 1.2, 1.6, 2.0]
WARMUP, WINDOW, WPL = 10000, 12000, 3
LAND_STEPS = WARMUP + len(LEVELS_ASC) * WPL * WINDOW + 2000
SEARCH_ARMS = ["sigma0", "fix020"]
SEARCH_STEPS = 600000
SEEDS = range(1, 7)
CONCURRENCY = 6


def schedule(asc):
    levels = LEVELS_ASC if asc else list(reversed(LEVELS_ASC))
    ents = [f"1:motor_epm:coupling_gain:{levels[0]}"]
    for i, lv in enumerate(levels[1:], start=1):
        ents.append(f"{WARMUP + i * WPL * WINDOW}:motor_epm:coupling_gain:{lv}")
    return ";".join(ents)


def run_one(body, tag, cfg, seed, steps, outdir, port, setparam=None):
    log = f"{outdir}/{body}/{tag}_s{seed}.log"
    os.makedirs(os.path.dirname(log), exist_ok=True)
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="arena", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(port), OGMA_PICRAWLER_BODY=body,
               OGMA_PICRAWLER_GYM_DIFFICULTY="0.3",
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(steps))
    if setparam:
        env["OGMA_PICRAWLER_SETPARAM_AT"] = setparam
    t0 = time.time()
    with open(log, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "20000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=f, stderr=subprocess.STDOUT)
    body_ok = any(f"geometry '{body}' loaded" in ln for ln in open(log, errors="replace"))
    cfg_ok = any(cfg in ln for ln in open(log, errors="replace"))
    print(f"  {body}/{tag} s{seed}: {time.time()-t0:.0f}s body_ok={body_ok} cfg_ok={cfg_ok}",
          flush=True)
    return body_ok and cfg_ok


def cmd_landscape(outdir):
    jobs = []
    k = 0
    for body in BODIES:
        for s in SEEDS:
            jobs.append((body, "land", CFG_C2, s, LAND_STEPS, outdir, 7800 + k,
                         schedule(s % 2 == 1)))
            k += 1
    print(f"E1: {len(jobs)} runs x {LAND_STEPS} ticks -> {outdir}")
    with cf.ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        ok = list(ex.map(lambda j: run_one(*j), jobs))
    print(f"done: {sum(ok)}/{len(ok)} confirmed (body + config)")


def make_settle_config():
    """E3a config: BASE3 (native j1s4 3-gain seed) + the C2 criterion + sigma 0.2
    pinned, free search.  Regenerable one-off (not committed), body via env."""
    cfg = json.load(open(os.path.join(CFGDIR, CFG_3D)))
    ge = next(m for m in cfg["modules"] if m.get("type") == "GainEvolver")["params"]
    assert ge["gain_seed"] == [0.385, 1.655, 1.092], ge["gain_seed"]
    ge["mutation_sigma"] = 0.2
    ge["sigma_min"], ge["sigma_max"] = 0.2, 0.2
    ge["target_accept"] = -1.0
    ge["travel_topic"] = "reality.proprio.stride_v"
    ge["flow_min_form"] = 1
    ge["flow_turn_k"] = 4.0
    ge["w_flow"] = 2.0
    ge["w_energy"] = 1.0
    name = "e3_settle.json"
    json.dump(cfg, open(os.path.join(CFGDIR, name), "w"), indent=2)
    return name


def cmd_settle(outdir):
    cfg = make_settle_config()
    jobs = []
    k = 0
    for body in BODIES:
        for s in SEEDS:
            jobs.append((body, "settle", cfg, s, SEARCH_STEPS, outdir, 7900 + k, None))
            k += 1
    print(f"E3a: {len(jobs)} runs x {SEARCH_STEPS} ticks -> {outdir}")
    with cf.ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        ok = list(ex.map(lambda j: run_one(*j), jobs))
    print(f"done: {sum(ok)}/{len(ok)} confirmed (body + config)")


def cmd_analyze_settle(outdir):
    """Apply the REGISTERED selection rule: per body, best last-third mean J among
    runs with falls <= that arm's median; print all endpoints beside the pick."""
    for body in BODIES:
        rows = []
        for p in sorted(glob.glob(f"{outdir}/{body}/settle_s*.log")):
            ws = windows(p)
            if not ws:
                continue
            last = ws[-1]
            third = ws[-(max(1, len(ws) // 3)):]
            rows.append({
                "run": os.path.basename(p),
                "vec": last.get("ge_vec"),
                "J_third": st.mean(float(w["ge_ji"]) for w in third),
                "falls": int(last.get("auto_reset_count", 0)),
                "n_win": len(ws),
            })
        med_falls = st.median(r["falls"] for r in rows)
        viable = [r for r in rows if r["falls"] <= med_falls]
        pick = min(viable, key=lambda r: r["J_third"])
        print(f"\n{body}: median falls {med_falls}")
        for r in rows:
            mark = "  << SELECTED" if r is pick else ("" if r["falls"] <= med_falls else "  (falls-filtered)")
            v = r["vec"]
            print(f"  {r['run']:16s} amp={v[0]:.3f} coup={v[1]:.3f} post={v[2]:.3f} "
                  f"J3={r['J_third']:.4f} falls={r['falls']}{mark}")


def cmd_search(outdir):
    # MEASURED BODY ONLY.  The cad arm of E2 is the D2 corpus reused
    # (~/xaq_runs/stridoD2_20260825): identical configs, seeds, gym, body and
    # criterion, and the sim is deterministic (measured twice this campaign), so a
    # cad re-run would reproduce those logs byte-for-byte.  Registered in the plan
    # doc's E2 block — cad's numbers are post-diction and cited as-is.
    jobs = []
    k = 0
    for arm in SEARCH_ARMS:
        for s in SEEDS:
            jobs.append(("measured", arm, f"d2_{arm}_s{s}.json", s, SEARCH_STEPS,
                         outdir, 7850 + k, None))
            k += 1
    print(f"E2: {len(jobs)} runs x {SEARCH_STEPS} ticks -> {outdir}")
    with cf.ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        ok = list(ex.map(lambda j: run_one(*j), jobs))
    print(f"done: {sum(ok)}/{len(ok)} confirmed (body + config)")


# ---------------------------------------------------------------------------


def windows(path):
    """Corrected attribution: a window's terms are on the FIRST line after the reset."""
    rows = []
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_wt"' not in ln:
            continue
        try:
            rows.append(json.loads(ln))
        except json.JSONDecodeError:
            continue
    out = []
    for i in range(1, len(rows)):
        if (int(rows[i].get("ge_wt", -1)) < int(rows[i - 1].get("ge_wt", -1))
                and rows[i].get("ge_ji", -1) > 0):
            out.append(rows[i])
    return out


def cmd_analyze_landscape(outdir):
    profile = {}
    for body in BODIES:
        per = {lv: [] for lv in LEVELS_ASC}
        noise_diffs = []
        for p in sorted(glob.glob(f"{outdir}/{body}/land_s*.log")):
            seed = int(p.rsplit("_s", 1)[1].split(".")[0])
            ws = windows(p)
            levels = LEVELS_ASC if seed % 2 == 1 else list(reversed(LEVELS_ASC))
            Js = [float(w["ge_ji"]) for w in ws]
            for i, J in enumerate(Js):
                li = i // WPL
                if li < len(levels):
                    per[levels[li]].append(J)
                if i + 1 < len(Js) and (i % WPL) != WPL - 1:
                    noise_diffs.append(Js[i + 1] - Js[i])
        noise_sd = (st.variance(noise_diffs) / 2.0) ** 0.5 if len(noise_diffs) > 1 else float("nan")
        profile[body] = (per, noise_sd)
        means = [st.mean(per[lv]) for lv in LEVELS_ASC]
        print(f"\n{body}: window noise sd {noise_sd:.4f}   "
              f"argmin {LEVELS_ASC[means.index(min(means))]}")
        for lv in LEVELS_ASC:
            print(f"  {lv:.1f}: J {st.mean(per[lv]):.4f} ± {st.stdev(per[lv]):.4f} (n={len(per[lv])})")
    a, na = profile[BODIES[0]]
    b, nb = profile[BODIES[1]]
    print(f"\nBETWEEN-BODY (gate: any level |ΔJ| > 2× pooled comparison noise, or argmin ≥ 2 levels apart):")
    hits = 0
    for lv in LEVELS_ASC:
        d = st.mean(b[lv]) - st.mean(a[lv])
        # noise of a mean-of-n comparison from the pooled window noise
        n_ab = min(len(a[lv]), len(b[lv]))
        cmp_sd = ((na ** 2 + nb ** 2) / max(1, n_ab)) ** 0.5
        flag = "  ** EXCEEDS 2x noise **" if abs(d) > 2 * cmp_sd else ""
        if abs(d) > 2 * cmp_sd:
            hits += 1
        print(f"  {lv:.1f}: ΔJ(measured−cad) {d:+.4f}   2×noise {2*cmp_sd:.4f}{flag}")
    print(f"\nGATE: {'PASS' if hits else 'FAIL'} ({hits}/6 levels exceed)")


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "landscape":
        cmd_landscape(sys.argv[2] if len(sys.argv) > 2 else
                      os.path.expanduser(time.strftime("~/xaq_runs/twobodyE1_%Y%m%d")))
    elif len(sys.argv) >= 3 and sys.argv[1] == "analyze-landscape":
        cmd_analyze_landscape(sys.argv[2])
    elif len(sys.argv) >= 2 and sys.argv[1] == "search":
        cmd_search(sys.argv[2] if len(sys.argv) > 2 else
                   os.path.expanduser(time.strftime("~/xaq_runs/twobodyE2_%Y%m%d")))
    elif len(sys.argv) >= 2 and sys.argv[1] == "settle":
        cmd_settle(sys.argv[2] if len(sys.argv) > 2 else
                   os.path.expanduser(time.strftime("~/xaq_runs/twobodyE3a_%Y%m%d")))
    elif len(sys.argv) >= 3 and sys.argv[1] == "analyze-settle":
        cmd_analyze_settle(sys.argv[2])
    else:
        print(__doc__)
