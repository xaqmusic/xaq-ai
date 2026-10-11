#!/usr/bin/env python3
"""after_sit -- does the duck get away from where it sat? (design doc §17.115 E1, 2026-10-10)

The operator: "the sit is firing as expected but the robot remains in the corner, which is boring."  From the host's FULL
JSONL, per sit (`event` skill:sit): the farthest the trunk gets from the sit's place within 10 / 20 / 40 s, the time until
it is 0.6 m away, the share of backing in the walk 5-15 s after, the next event, and the sits per run and per seed.
Usage: after_sit.py "glob" [...]
"""
import collections, glob, json, math, statistics, sys


def main():
    D = collections.defaultdict(list); nxt = collections.Counter(); per = []
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        R = [json.loads(l) for l in open(path) if l.startswith('{"t"')]
        n = 0
        for i, r in enumerate(R):
            if r.get("event") != "skill:sit":
                continue
            n += 1; x0, y0 = r["x"], r["y"]
            for secs in (10, 20, 40):
                j = min(len(R) - 1, i + secs * 50)
                D[secs].append(max(math.hypot(R[k]["x"] - x0, R[k]["y"] - y0) for k in range(i, j + 1)))
            out = next((R[k]["t"] - r["t"] for k in range(i, len(R)) if math.hypot(R[k]["x"] - x0, R[k]["y"] - y0) > 0.6), None)
            D["out"].append(out if out is not None else 999.0)
            tw = [x["twist"][0] for x in R[i + 250:i + 750] if not int(x.get("stop", 0))]
            D["back"].append(sum(v < -0.05 for v in tw) / max(1, len(tw)))
            evs = [x.get("event") for x in R[i + 1:i + 1500] if x.get("event") and x.get("event") not in ("skill:sit", "stop:walker")]
            nxt[evs[0] if evs else "-"] += 1
        per.append(n)
    if not per or not sum(per):
        print("no sits"); return
    print(f"{len(per)} runs, {sum(per)} sits ({statistics.mean(per):.1f} a run, max {max(per)})")
    for k in (10, 20, 40):
        v = sorted(D[k]); print(f"  farthest from the sit's place within {k:2d} s: p25 {v[len(v) // 4]:.2f}  p50 {v[len(v) // 2]:.2f} m")
    v = sorted(D["out"]); print(f"  time to 0.6 m away: p25 {v[len(v) // 4]:.0f}  p50 {v[len(v) // 2]:.0f}  p75 {v[3 * len(v) // 4]:.0f} s")
    print(f"  backing share of the walk 5-15 s after: {statistics.mean(D['back']):.2f}")
    print("  next event:", nxt.most_common(6))


if __name__ == "__main__":
    main()
