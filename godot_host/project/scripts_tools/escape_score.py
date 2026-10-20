#!/usr/bin/env python3
"""Escape test scorer: the robot is dropped (OGMA_PICRAWLER_TELEPORT_RAMP_AT + _TELEPORT_XZ) onto a corridor
feature -- a pyramid apex, a side-wall slope, the hump -- and this reads whether and how fast it gets itself
off it.  Operator, 2026-10-19: "while the original was slow, the adaptation would get the robot unstuck from
almost any location in the corridor."

The drop is found as the first tick after 1000 where the chassis jumps > 0.2 m.  From the landing point:
  escaped   the chassis is > ESC_R m (horizontal) from the landing point AND, for a wall placement
            (|x| > 0.6 at landing), back on the channel floor (|x| < 0.6)
  t_esc     ticks from the drop to the first escaped tick (NaN if never)
  moved     horizontal distance from the landing point at the end
  falls     auto-resets after the drop
Usage: escape_score.py NAME=<glob> [NAME=<glob> ...]   (per-tick diag logs)
"""
import glob, json, math, re, statistics as st, sys

ESC_R = 0.40


def one(path):
    rows = []
    for l in open(path, errors="replace"):
        if l.startswith("{") and '"x"' in l and '"t"' in l and '"con"' in l:
            try:
                rows.append(json.loads(l))
            except Exception:
                pass
    drop = None
    tp = None                                   # the drop tick, from the body's own "[teleport] ... tick N" line
    for l in open(path, errors="replace"):
        mm = re.search(r"\[teleport\] dropped.* tick (\d+)", l)
        if mm:
            tp = int(mm.group(1)); break
    for i in range(1, len(rows)):
        if tp is not None:
            if rows[i]["t"] > tp:
                drop = i; break
        elif rows[i]["t"] > 1000 and math.hypot(rows[i]["x"] - rows[i - 1]["x"], rows[i]["z"] - rows[i - 1]["z"]) > 0.2:
            drop = i; break
    if drop is None:
        return None
    land = rows[min(len(rows) - 1, drop + 25)]          # after the fall settles
    x0, z0 = land["x"], land["z"]
    wall = abs(x0) > 0.6
    ar0 = rows[drop]["auto_reset_count"] if "auto_reset_count" in rows[drop] else 0
    t_esc = float("nan")
    for r in rows[drop + 25:]:
        if r.get("auto_reset_count", 0) != ar0:
            break                                        # a fall resets the robot elsewhere: not an escape
        d = math.hypot(r["x"] - x0, r["z"] - z0)
        if d > ESC_R and (not wall or abs(r["x"]) < 0.6):
            t_esc = r["t"] - rows[drop]["t"]; break
    end = rows[-1]
    return {"landing": (round(x0, 2), round(z0, 2)), "escaped": not math.isnan(t_esc), "t_esc": t_esc,
            "moved": math.hypot(end["x"] - x0, end["z"] - z0) if end.get("auto_reset_count", 0) == ar0 else float("nan"),
            "falls": end.get("auto_reset_count", 0) - ar0,
            "after": end["t"] - rows[drop]["t"]}


def main():
    for spec in sys.argv[1:]:
        name, _, pat = spec.partition("=")
        res = [r for r in (one(p) for p in sorted(glob.glob(pat))) if r]
        if not res:
            print(f"  {name:28s} no drop found"); continue
        esc = [r for r in res if r["escaped"]]
        te = [r["t_esc"] for r in esc]
        mv = [r["moved"] for r in res if not math.isnan(r["moved"])]
        print(f"  {name:28s} escaped {len(esc)}/{len(res)}  t_esc median {st.median(te) if te else float('nan'):6.0f} ticks"
              f"  moved {st.mean(mv) if mv else float('nan'):4.2f} m  falls {sum(r['falls'] for r in res)}"
              f"  landing {res[0]['landing']}  window {res[0]['after']} ticks")


if __name__ == "__main__":
    main()
