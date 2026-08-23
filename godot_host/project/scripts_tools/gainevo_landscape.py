#!/usr/bin/env python3
"""Step A — the criterion's landscape for every evolved gain.

    python3 gainevo_landscape.py <sweep dir>

Each gain is swept across its declared range while the other seven are held at a
validated operating point, with the search in observer mode so the sweep is the
only thing moving. This yields two things the project does not otherwise have:

  1. A MEASURED IDEAL per gain — the value the criterion actually prefers, as
     distinct from a hand-found reference point.
  2. An AUTHORITY verdict per gain — whether the criterion can see that gain at
     all. A gain whose landscape is flat relative to measurement noise gives the
     search nothing to climb, and no amount of searching will recover it. Such a
     gain should be excluded from a displacement-recovery test rather than being
     allowed to look like a failure of the search.

Windows map to levels by ORDER (index // windows-per-level), exact because the
schedule is deterministic. Odd seeds ascend and even seeds descend, so a large
ascending/descending gap at the same level indicates hysteresis rather than
landscape.
"""
import glob, json, math, os, statistics as st, sys

WINDOWS_PER_LEVEL = 3
N_LEVELS = 6


def windows(path):
    """Scored windows in order; a boundary is the window-tick counter resetting."""
    out, prev, cur = [], None, None
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_wt"' not in ln:
            continue
        try:
            d = json.loads(ln)
        except json.JSONDecodeError:
            continue
        wt = int(d.get("ge_wt", -1))
        if prev is not None and wt < prev and cur is not None:
            out.append(cur)
        prev = wt
        if d.get("ge_ji", -1) > 0:
            cur = d
    if cur is not None:
        out.append(cur)
    return out


def main(d):
    sched = {}
    sf = os.path.join(d, "schedules.txt")
    if os.path.exists(sf):
        for ln in open(sf):
            parts = ln.strip().split("|")
            if len(parts) == 3:
                sched[parts[0]] = [float(x) for x in parts[2].split(",")]

    gains = sorted({os.path.basename(p).rsplit("_s", 1)[0]
                    for p in glob.glob(d + "/*_s*.log")})
    print(f"{len(gains)} gains swept, {N_LEVELS} levels x {WINDOWS_PER_LEVEL} windows\n")
    summary = []
    for g in gains:
        levels = sched.get(g, list(range(N_LEVELS)))
        bylevel = {i: [] for i in range(N_LEVELS)}
        asc, desc = {i: [] for i in range(N_LEVELS)}, {i: [] for i in range(N_LEVELS)}
        for p in sorted(glob.glob(f"{d}/{g}_s*.log")):
            seed = int(p.rsplit("_s", 1)[1].split(".")[0])
            ws = windows(p)
            if len(ws) < WINDOWS_PER_LEVEL:
                continue
            for i, w in enumerate(ws):
                li = i // WINDOWS_PER_LEVEL
                if li >= N_LEVELS:
                    break
                # even seeds ran the schedule in reverse
                idx = li if seed % 2 == 1 else (N_LEVELS - 1 - li)
                bylevel[idx].append(float(w["ge_ji"]))
                (asc if seed % 2 == 1 else desc)[idx].append(float(w["ge_ji"]))
        means = [st.mean(bylevel[i]) if bylevel[i] else float("nan") for i in range(N_LEVELS)]
        within = [st.pstdev(bylevel[i]) for i in range(N_LEVELS) if len(bylevel[i]) > 1]
        noise = st.mean(within) if within else float("nan")
        ok = [m for m in means if m == m]
        if not ok:
            print(f"  {g}: no data"); continue
        span = max(ok) - min(ok)
        best = means.index(min(ok))
        # authority: does the landscape rise above its own within-level scatter?
        ratio = span / noise if noise and noise == noise else float("nan")
        verdict = ("STRONG" if ratio >= 2.0 else "weak" if ratio >= 1.0 else "FLAT — criterion cannot see it")
        print(f"  {g}")
        print("    level " + "  ".join(f"{levels[i]:>7.3g}" for i in range(N_LEVELS)))
        print("    J     " + "  ".join(f"{means[i]:>7.3f}" for i in range(N_LEVELS)))
        # hysteresis: same level reached from opposite directions
        hys = [abs(st.mean(asc[i]) - st.mean(desc[i]))
               for i in range(N_LEVELS) if asc[i] and desc[i]]
        h = max(hys) if hys else float("nan")
        print(f"    span {span:.3f}   within-level noise {noise:.3f}   span/noise {ratio:.2f}  -> {verdict}")
        print(f"    best J at {levels[best]:.3g}   worst-case asc/desc gap {h:.3f}"
              f"{'   ** exceeds span: order effect, not landscape **' if h == h and h > span else ''}")
        print()
        summary.append((g, levels[best], ratio, verdict))

    print("=== MEASURED IDEALS (for the displacement-recovery test) ===")
    print(f"  {'gain':<20}{'ideal':>9}{'span/noise':>12}   authority")
    for g, best, ratio, verdict in summary:
        print(f"  {g:<20}{best:>9.3g}{ratio:>12.2f}   {verdict}")
    usable = [g for g, _, r, _ in summary if r == r and r >= 1.0]
    print(f"\n  {len(usable)}/{len(summary)} gains have a landscape the search could climb:")
    print(f"    {', '.join(usable) if usable else '(none)'}")
    print("  Gains below that bar are excluded from recovery testing: a flat landscape")
    print("  offers no gradient, so failure to recover would measure the criterion, not")
    print("  the search.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
