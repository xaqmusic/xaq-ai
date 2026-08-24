#!/usr/bin/env python3
"""Do independent random starts converge on the same gain vector?

    python3 gainevo_basin.py <basin dir>

The per-gain landscape sweeps say what the criterion can SEE from one operating
point with everything else held still. This asks the question those cannot: with
all eight gains free and started from random points, does the search find a common
destination, and does it find the SAME one on a body with different geometry?

Everything is reported in NORMALIZED gain units (0 = gain_min, 1 = gain_max) so
gains with wildly different ranges are comparable, and spread is reported as the
mean pairwise distance between runs — start vs end. A search that converges
shrinks it; a search that wanders does not.

The per-gain columns are the payload. A gain the criterion can see should be
TIGHTER at the end than its random start; a gain it cannot see has nothing to pull
it anywhere and should stay as scattered as it began. That is a direct, independent
prediction from the landscape sweeps, made on data they never touched.
"""
import glob, json, math, os, statistics as st, sys

# Search bounds are READ FROM THE CONFIG the runs actually used, never restated
# here.  A hand-copied table drifted from the shipped bounds on two of eight gains
# within a day of being written; per-gain sd RATIOS survive that (they are
# scale-invariant) but every distance in normalized space does not.
BOUNDS = []


def load_bounds(d):
    """Bounds from any basin_*.json the runs were launched with."""
    import glob as _g
    for pat in (os.path.join(d, "basin_*.json"),
                "godot_host/project/addons/ami_ogma/configs/basin_*.json"):
        for f in sorted(_g.glob(pat)):
            cfg = json.load(open(f))
            ge = next((m for m in cfg["modules"] if m.get("type") == "GainEvolver"), None)
            if ge:
                return list(zip(ge["params"]["gain_min"], ge["params"]["gain_max"]))
    raise SystemExit("gainevo_basin: cannot find a basin_*.json to read gain bounds from")
# authority measured by the landscape sweeps, for the cross-check
# the GOOD band each gain's landscape resolved (None = the criterion cannot see it)
GOOD_BAND = {"amp_target": (0.15, 0.41), "coupling_gain": (1.2, 2.0),
             "postural_gain": (0.66, 1.5)}
AUTHORITY = {"amp_target": "STRONG", "coupling_gain": "STRONG", "postural_gain": "STRONG",
             "height_homeo_gain": "weak", "plan_gain": "weak", "rear_push_ext": "weak",
             "rear_land_gain": "FLAT", "rear_knee_plant": "FLAT"}


def norm(v):
    return [(x - lo) / (hi - lo) if hi > lo else 0.0 for x, (lo, hi) in zip(v, BOUNDS)]


def final_vec(path):
    """Last published incumbent vector, plus the generation count that produced it."""
    vec, gen = None, 0
    for ln in open(path, errors="replace"):
        if '"ge_vec"' not in ln:
            continue
        try:
            d = json.loads(ln)
        except json.JSONDecodeError:
            continue
        if d.get("ge_vec"):
            vec, gen = d["ge_vec"], int(d.get("ge_gen", gen))
    return vec, gen


def spread(vs):
    """Mean pairwise Euclidean distance in normalized space."""
    if len(vs) < 2:
        return float("nan")
    ds = [math.dist(vs[i], vs[j]) for i in range(len(vs)) for j in range(i + 1, len(vs))]
    return st.mean(ds)


def main(d):
    meta = json.load(open(os.path.join(d, "starts.json")))
    keys, starts = meta["keys"], meta["starts"]
    global BOUNDS
    BOUNDS = load_bounds(d)
    print("  bounds (from config): " + ", ".join(
        f"{k} [{lo:g},{hi:g}]" for k, (lo, hi) in zip(keys, BOUNDS)) + "\n")
    bodies = [b for b in ("cad", "measured") if os.path.isdir(os.path.join(d, b))]
    print(f"{len(starts)} random starts x {len(bodies)} bodies\n")

    ends = {}
    for b in bodies:
        rows, used_starts = [], []
        for p in sorted(glob.glob(f"{d}/{b}/start*.log")):
            i = int(os.path.basename(p)[5:7])
            v, gen = final_vec(p)
            if v is None:
                print(f"  {b}/{os.path.basename(p)}: no published vector — EXCLUDE")
                continue
            rows.append((i, norm(v), gen, v))
            used_starts.append(norm(starts[i]))
        ends[b] = rows
        s0, s1 = spread(used_starts), spread([r[1] for r in rows])
        gens = [r[2] for r in rows]
        print(f"  {b}: n={len(rows)}, {min(gens)}–{max(gens)} generations")
        print(f"    whole-vector spread   start {s0:.3f}  ->  end {s1:.3f}   "
              f"({'CONVERGED' if s1 < s0 * 0.7 else 'partial' if s1 < s0 else 'DIVERGED'})")
        # SPREAD alone cannot tell a search that moved from one that sat still, so
        # the MEAN is reported beside it.  Selection shows up as the mean travelling
        # toward the landscape's good band; diffusion shows up as spread rising with
        # the mean going nowhere.  Reading only the spread confuses the two.
        print(f"    {'gain':<19}{'authority':>9}{'start mean':>11}{'end mean':>9}"
              f"{'moved':>8}{'sd ratio':>10}   good band")
        for k in range(len(keys)):
            sd0 = st.pstdev([s[k] for s in used_starts])
            sd1 = st.pstdev([r[1][k] for r in rows])
            lo, hi = BOUNDS[k]
            m0 = st.mean([s[k] for s in used_starts]) * (hi - lo) + lo
            m1 = st.mean([r[3][k] for r in rows])
            ratio = sd1 / sd0 if sd0 > 1e-9 else float("nan")
            band = GOOD_BAND.get(keys[k])
            btxt = f"{band[0]:.3g}-{band[1]:.3g}" if band else "(flat)"
            # did the mean move TOWARD the band it should prefer?
            def dist(x):
                if not band: return 0.0
                return max(band[0] - x, 0.0, x - band[1])
            arrow = ""
            if band:
                d0, d1 = dist(m0), dist(m1)
                arrow = ("-> in band" if d1 <= 1e-9 else
                         "-> toward" if d1 < d0 - 1e-9 else
                         "<- away" if d1 > d0 + 1e-9 else "")
            tag = "  scattered" if ratio > 1.1 else ("  tightened" if ratio < 0.85 else "")
            print(f"    {keys[k]:<19}{AUTHORITY.get(keys[k],'?'):>9}{m0:>11.3f}{m1:>9.3f}"
                  f"{(m1-m0):>+8.3f}{ratio:>10.2f}   {btxt:<10}{arrow}{tag}")
        print()

    # Does the criterion's own ranking predict what converged?  This is the check
    # the landscape sweeps cannot make on themselves.
    print("=== CROSS-CHECK: does measured authority predict convergence? ===")
    for b in bodies:
        rows = ends[b]
        used = [norm(starts[i]) for i, _, _, _ in rows]
        byauth = {}
        for k, key in enumerate(keys):
            sd0 = st.pstdev([s[k] for s in used]); sd1 = st.pstdev([r[1][k] for r in rows])
            if sd0 > 1e-9:
                byauth.setdefault(AUTHORITY.get(key, "?"), []).append(sd1 / sd0)
        line = "  ".join(f"{a}: {st.mean(v):.2f}" for a, v in
                         sorted(byauth.items(), key=lambda x: {"STRONG":0,"weak":1,"FLAT":2}.get(x[0],3)))
        print(f"  {b:<10} mean end/start sd ratio by authority — {line}")
    print("  (a working search tightens what its criterion can see and leaves the rest alone,")
    print("   so STRONG should sit BELOW FLAT.  If they match, convergence is not selection.)")

    if len(bodies) == 2:
        print("\n=== TWO BODIES: same search, different geometry and weight ===")
        a = [r[1] for r in ends[bodies[0]]]
        bvs = [r[1] for r in ends[bodies[1]]]
        ca = [st.mean([v[k] for v in a]) for k in range(len(keys))]
        cb = [st.mean([v[k] for v in bvs]) for k in range(len(keys))]
        within = (spread(a) + spread(bvs)) / 2
        sep = math.dist(ca, cb)
        print(f"  centroid separation {sep:.3f}   vs within-body spread {within:.3f}")
        # ⚠ BLIND METRIC GUARD.  "The centroids are close" is satisfied trivially when
        # neither body converged — the within-body cloud swamps any body difference, so
        # a null here is a statement about the SEARCH, not about the two bodies.  This
        # comparison only carries information once at least one body converges.
        if within > sep * 2:
            print("  -> UNINFORMATIVE: within-body spread is more than 2x the separation,")
            print("     so this cannot distinguish 'the bodies agree' from 'neither converged'.")
            print("     Re-run the two-body comparison only after convergence on one body.")
        else:
            print(f"  -> {'DISTINCT basins per body' if sep > within else 'the two bodies land in the SAME region'}")
        print(f"    {'gain':<20}{bodies[0]:>10}{bodies[1]:>11}      shift (normalized)")
        for k in range(len(keys)):
            ra = st.mean([r[3][k] for r in ends[bodies[0]]])
            rb = st.mean([r[3][k] for r in ends[bodies[1]]])
            print(f"    {keys[k]:<20}{ra:>10.3f}{rb:>11.3f}      {cb[k]-ca[k]:+.3f}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
