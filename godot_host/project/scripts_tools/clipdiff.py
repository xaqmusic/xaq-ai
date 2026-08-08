#!/usr/bin/env python3
"""GOOD vs BAD clip differ — turn "that looked right" into a measurement.

THE PROBLEM THIS SOLVES.  The operator can see gait quality in the UI long before any
aggregate metric moves, and describing it in words is lossy in a specific way: this
project has three recorded cases where the reported ACTION was right and the reported
MECHANISM was wrong ("the swing leg spins the chassis" — yaw impulse measured LOWER
during swing; "a vertical shank gives mechanical advantage" — foot radius barely moved;
"shorter steps bring the feet in" — foot radius invariant).  Each cost a build plus an
A/B before the instrument caught it.

So instead of describing: MARK it.  [F1] while it looks right, [F2] while it looks
wrong, and this script reports what actually differs between those windows.  The
operator's judgement stays the authority on what "good" means; the numbers say what good
CONSISTS of, which is the part words keep getting wrong.

  $ clipdiff.py /tmp/xaq_clips/<runid>

IT IS AN INSTRUMENT, NEVER A FITNESS.  Nothing here enters a loop the brain can
optimize — that would be reward shaping, which CLAUDE.md §5.1 prohibits outright.  This
picks which lever to chase; the brain never sees it.

READING THE OUTPUT.  Features are ranked by SEPARATION = |mean_good - mean_bad| / pooled
sd, i.e. how cleanly the feature splits the two sets, not how big the difference looks in
raw units.  A separation above ~1.5 with n>=3 clips per side is worth chasing; below ~0.8
the feature does not distinguish what you saw.  With one clip per side everything is
"separated" and nothing is measured — the script says so rather than pretending.
"""
import json
import math
import os
import statistics
import sys

LEGS = ("FL", "FR", "RL", "RR")


# ---------------------------------------------------------------------------
# Per-clip feature extraction
# ---------------------------------------------------------------------------
def _rising_edges(seq):
    """Indices where a 0/1 sequence goes 0 -> 1 (touchdowns)."""
    return [i for i in range(1, len(seq)) if seq[i] and not seq[i - 1]]


def _cv(xs):
    """Coefficient of variation — the REGULARITY measure.

    This is the one the operator's vocabulary points at most directly ("repetitive"),
    and it is exactly what a mean period cannot express: two gaits can share a 27-tick
    mean step while one metronomes and the other alternates 15 and 39.
    """
    xs = [x for x in xs if x > 0]
    if len(xs) < 2:
        return float("nan")
    m = statistics.fmean(xs)
    return statistics.pstdev(xs) / m if m > 0 else float("nan")


def clip_features(rows):
    """Reduce one clip (list of per-tick dicts) to a flat feature dict."""
    if len(rows) < 20:
        return None
    n = len(rows)
    f = {}

    con = [r["c"] for r in rows]                      # per-tick [4] contact
    h1 = [r["h1"] for r in rows]                      # per-tick [4] hip1 angle

    # ---- support: how much of the body is on the ground, and how evenly shared
    f["contact_duty"] = statistics.fmean(statistics.fmean(c) for c in con)
    f["planted_mean"] = statistics.fmean(sum(c) for c in con)
    duties = [statistics.fmean(c[i] for c in con) for i in range(4)]
    f["duty_spread"] = max(duties) - min(duties)      # >0.3 = a leg is not participating
    # Fraction of time with a statically-stable 3+ support polygon.
    f["support_ge3"] = statistics.fmean(1.0 if sum(c) >= 3 else 0.0 for c in con)
    f["support_le1"] = statistics.fmean(1.0 if sum(c) <= 1 else 0.0 for c in con)

    # ---- stepping: count, balance across legs, and REGULARITY
    tds = [_rising_edges([c[i] for c in con]) for i in range(4)]
    counts = [len(t) for t in tds]
    f["steps"] = sum(counts)
    f["step_bal"] = (min(counts) / max(counts)) if max(counts) > 0 else 0.0
    periods = []
    for t in tds:
        periods.extend(t[k] - t[k - 1] for k in range(1, len(t)))
    f["step_period"] = statistics.fmean(periods) if periods else float("nan")
    f["step_period_cv"] = _cv(periods)                # LOW = metronomic = "repetitive"
    # Per-leg period spread: are the four legs even running at the same rate?
    per_leg = [statistics.fmean([t[k] - t[k - 1] for k in range(1, len(t))])
               for t in tds if len(t) >= 2]
    f["leg_period_spread"] = (max(per_leg) - min(per_leg)) if len(per_leg) >= 2 else float("nan")

    # ---- inter-leg coordination: how consistent is the touchdown ORDER?
    # For each leg pair, the phase of leg j's touchdown within leg i's cycle; a
    # coordinated gait holds that offset, a stumbling one scatters it.  Reported as a
    # phase-locking value so 1 = rigid pattern, 0 = no relation.
    plv_sum, plv_n = 0.0, 0
    for i in range(4):
        if len(tds[i]) < 3:
            continue
        cyc = statistics.fmean([tds[i][k] - tds[i][k - 1] for k in range(1, len(tds[i]))])
        if cyc <= 0:
            continue
        for j in range(4):
            if i == j or not tds[j]:
                continue
            cs, sn, m = 0.0, 0.0, 0
            for a in tds[i]:
                nxt = [b for b in tds[j] if b >= a]
                if not nxt:
                    continue
                ph = 2 * math.pi * ((nxt[0] - a) % cyc) / cyc
                cs += math.cos(ph); sn += math.sin(ph); m += 1
            if m >= 3:
                plv_sum += math.hypot(cs, sn) / m; plv_n += 1
    f["interleg_plv"] = (plv_sum / plv_n) if plv_n else float("nan")

    # ---- the honest propulsion read: does the planted foot travel BACKWARD?
    # Same quantity MotorEPM reports as mv_stance/mv_swing, recomputed here from the
    # body's own record so a clip is self-contained.
    mv_st, mv_sw = [], []
    for k in range(1, n):
        for i in range(4):
            d = h1[k][i] - h1[k - 1][i]
            (mv_st if con[k][i] else mv_sw).append(d if i in (0, 2) else -d)
    f["mv_stance"] = statistics.fmean(mv_st) if mv_st else float("nan")
    f["mv_swing"] = statistics.fmean(mv_sw) if mv_sw else float("nan")
    f["mv_separation"] = f["mv_swing"] - f["mv_stance"]
    f["hip1_range"] = statistics.fmean(
        max(r["h1"][i] for r in rows) - min(r["h1"][i] for r in rows) for i in range(4))

    # ---- what the body actually did
    dt = max(1, rows[-1]["t"] - rows[0]["t"])
    dx = rows[-1]["x"] - rows[0]["x"]
    dz = rows[-1]["z"] - rows[0]["z"]
    path = sum(math.dist((rows[k]["x"], rows[k]["z"]), (rows[k - 1]["x"], rows[k - 1]["z"]))
               for k in range(1, n))
    f["net_disp"] = math.hypot(dx, dz)
    f["speed"] = f["net_disp"] / dt * 60.0            # m per 60 ticks (~1 s)
    f["straight"] = f["net_disp"] / path if path > 1e-6 else 0.0
    f["path_len"] = path
    yaws = [r["yaw"] for r in rows]
    dyaw = []
    for k in range(1, n):
        d = yaws[k] - yaws[k - 1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        dyaw.append(d)
    f["yaw_drift"] = abs(sum(dyaw))
    f["yaw_jitter"] = statistics.pstdev(dyaw) if len(dyaw) > 1 else 0.0
    f["chassis_y"] = statistics.fmean(r["y"] for r in rows)
    f["chassis_y_sd"] = statistics.pstdev([r["y"] for r in rows]) if n > 1 else 0.0
    f["fwd_v"] = statistics.fmean(r.get("fwd_v", 0.0) for r in rows)
    return f


# ---------------------------------------------------------------------------
def load_clip(path):
    rows, meta = [], {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "_meta" in d:
                meta = d["_meta"]
            else:
                rows.append(d)
    return meta, rows


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    d = argv[0]
    if not os.path.isdir(d):
        print(f"ERROR: not a directory: {d}")
        return 1
    files = sorted(f for f in os.listdir(d) if f.endswith(".jsonl"))
    if not files:
        print(f"ERROR: no .jsonl clips in {d}")
        return 1

    groups = {"GOOD": [], "BAD": []}
    skipped = []
    for fn in files:
        meta, rows = load_clip(os.path.join(d, fn))
        label = meta.get("label", "GOOD" if "GOOD" in fn else "BAD" if "BAD" in fn else None)
        if label not in groups:
            skipped.append((fn, "no GOOD/BAD label"))
            continue
        feats = clip_features(rows)
        if feats is None:
            skipped.append((fn, f"only {len(rows)} ticks"))
            continue
        groups[label].append((fn, feats, meta))

    ng, nb = len(groups["GOOD"]), len(groups["BAD"])
    print(f"\n  {d}")
    print(f"  GOOD: {ng} clips     BAD: {nb} clips")
    for fn, why in skipped:
        print(f"    skipped {fn}  ({why})")
    # Name the configs involved — a GOOD clip from one arm and a BAD one from another
    # measures the ARM, not the behaviour, and that confound is invisible in the table.
    cfgs = {m.get("config", "?") for _, _, m in groups["GOOD"] + groups["BAD"]}
    if len(cfgs) > 1:
        print(f"\n  !! clips span {len(cfgs)} configs: {', '.join(sorted(cfgs))}")
        print("     A GOOD clip from one arm vs a BAD clip from another measures the ARM.")
    if ng == 0 or nb == 0:
        print("\n  Need at least one clip on each side.  Mark with [F1] (good) / [F2] (bad).")
        return 1
    if ng < 3 or nb < 3:
        print(f"\n  !! UNDERPOWERED: with {ng} vs {nb} clips every feature will look")
        print("     'separated'.  Treat the ordering as a hint, not a result; 3+ per side")
        print("     is where the separation column starts meaning anything.")

    keys = [k for k in groups["GOOD"][0][1] if all(
        k in f for _, f, _ in groups["GOOD"] + groups["BAD"])]
    rows_out = []
    for k in keys:
        g = [f[k] for _, f, _ in groups["GOOD"] if not math.isnan(f[k])]
        b = [f[k] for _, f, _ in groups["BAD"] if not math.isnan(f[k])]
        if not g or not b:
            continue
        mg, mb = statistics.fmean(g), statistics.fmean(b)
        sg = statistics.pstdev(g) if len(g) > 1 else 0.0
        sb = statistics.pstdev(b) if len(b) > 1 else 0.0
        pooled = math.sqrt((sg ** 2 + sb ** 2) / 2)
        sep = abs(mg - mb) / pooled if pooled > 1e-12 else (float("inf") if mg != mb else 0.0)
        rows_out.append((sep, k, mg, sg, mb, sb))
    rows_out.sort(key=lambda r: (-r[0] if r[0] != float("inf") else -1e18))

    print(f"\n  {'feature':<20}{'GOOD':>18}{'BAD':>18}{'separation':>12}")
    print("  " + "-" * 68)
    for sep, k, mg, sg, mb, sb in rows_out:
        s = "  inf" if sep == float("inf") else f"{sep:5.2f}"
        mark = " <<<" if sep != float("inf") and sep >= 1.5 else ""
        print(f"  {k:<20}{mg:>10.4f} ±{sg:<6.4f}{mb:>10.4f} ±{sb:<6.4f}{s:>12}{mark}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
