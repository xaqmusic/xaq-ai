#!/usr/bin/env python3
"""stop_phases -- where a stop's time goes, and how long the walk takes to start again (design doc §17.114 step 3, 2026-10-09).

The operator: "the pause between transitions -- if our standing brain is the root of the pause, experiment with using the
existing RL behaviour to hold the stance."  From the host's FULL JSONL (`stop` the phase: 1 settle, 2 the standing brain,
3 the walker holding; `x`, `y`; `event`), per stop:
  settle      ticks waiting for stillness before the stand (the walker at zero command; the look has not started)
  stand       ticks with the standing brain (the look's sweep runs here)
  walker      ticks the walker holds the stance after a hand-off (or a refusal)
  to walk     from the stop's end until the trunk has moved 5 cm (the walk visibly under way)
  decel       from the stop's start: the trunk's speed over the settle (m/s), how fast it was moving when the stop began
Usage: stop_phases.py "glob" [...]
"""
import glob, json, math, statistics, sys


def main():
    S = {"settle": [], "stand": [], "walker": [], "to walk": [], "stop": []}
    runs = 0
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        runs += 1
        R = [json.loads(l) for l in open(path) if l.startswith('{"t"')]
        k = 0
        while k < len(R):
            if int(R[k].get("stop", 0)) and (k == 0 or not int(R[k - 1].get("stop", 0))):
                j = k; cnt = {1: 0, 2: 0, 3: 0}
                while j < len(R) and int(R[j].get("stop", 0)):
                    cnt[int(R[j]["stop"])] = cnt.get(int(R[j]["stop"]), 0) + 1; j += 1
                if j >= len(R):
                    break
                S["settle"].append(cnt[1] / 50); S["stand"].append(cnt[2] / 50); S["walker"].append(cnt[3] / 50)
                S["stop"].append((j - k) / 50)
                x0, y0 = R[j]["x"], R[j]["y"]; m = j
                while m < len(R) and not int(R[m].get("stop", 0)) and math.hypot(R[m]["x"] - x0, R[m]["y"] - y0) < 0.05:
                    m += 1
                if m < len(R) and not int(R[m].get("stop", 0)):
                    S["to walk"].append((m - j) / 50)
                k = j
            else:
                k += 1
    n = len(S["stop"])
    print(f"{runs} runs, {n} stops ({n / max(1, runs):.1f} a run)")
    for key in ("stop", "settle", "stand", "walker", "to walk"):
        v = S[key]
        if v:
            print(f"  {key:8s} mean {statistics.mean(v):5.2f} s  p50 {statistics.median(v):5.2f}  p90 {sorted(v)[int(0.9 * len(v))]:5.2f}"
                  f"  total {sum(v) / max(1, runs):6.1f} s a run")


if __name__ == "__main__":
    main()
