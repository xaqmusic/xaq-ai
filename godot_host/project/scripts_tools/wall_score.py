#!/usr/bin/env python3
"""Wall-transition scorer: the robot is dropped on the channel floor facing a side wall
(OGMA_PICRAWLER_TELEPORT_RAMP_AT + _TELEPORT_XZ + _TELEPORT_YAW_DEG) and must walk from flat ground onto the 30 deg
slope.  Operator, 2026-10-20: the LPG stack could not make this transition; the original eventually did.

Corridor geometry (picrawler_body.gd _build_corridor): flat floor |x| < 0.75, then the wall face rises at 30 deg.
  onto      the chassis reached |x| > ON_X (about 0.12 m up the face) before any fall
  t_onto    ticks from the drop to that moment
  max_x     the farthest |x| the chassis reached before any fall (0.75 = the wall's foot)
  max_rise  the highest chassis y gained over its landing height before any fall (m)
  falls     auto-resets after the drop
Usage: wall_score.py NAME=<glob> [NAME=<glob> ...]
"""
import glob, json, math, re, statistics as st, sys
from _logio import open_log, log_glob

ON_X = 0.95


def one(path):
    rows = []
    for l in open_log(path):
        if l.startswith("{") and '"x"' in l and '"t"' in l and '"con"' in l:
            try:
                rows.append(json.loads(l))
            except Exception:
                pass
    drop = None
    tp = None                                   # the drop tick, from the body's own "[teleport] ... tick N" line
    for l in open_log(path):
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
    ar0 = rows[drop].get("auto_reset_count", 0)
    land = rows[min(len(rows) - 1, drop + 25)]
    y0 = land["y"]
    t_onto, max_x, max_rise = float("nan"), 0.0, 0.0
    for r in rows[drop + 25:]:
        if r.get("auto_reset_count", 0) != ar0:
            break
        max_x = max(max_x, abs(r["x"])); max_rise = max(max_rise, r["y"] - y0)
        if math.isnan(t_onto) and abs(r["x"]) > ON_X:
            t_onto = r["t"] - rows[drop]["t"]
    return {"onto": not math.isnan(t_onto), "t_onto": t_onto, "max_x": max_x, "max_rise": max_rise,
            "falls": rows[-1].get("auto_reset_count", 0) - ar0, "landing": (round(land["x"], 2), round(land["z"], 2))}


def main():
    for spec in sys.argv[1:]:
        name, _, pat = spec.partition("=")
        res = [r for r in (one(p) for p in log_glob(pat)) if r]
        if not res:
            print(f"  {name:28s} no drop found"); continue
        on = [r for r in res if r["onto"]]
        print(f"  {name:28s} onto the wall {len(on)}/{len(res)}  t_onto median {st.median([r['t_onto'] for r in on]) if on else float('nan'):6.0f}"
              f"  max|x| {st.mean(r['max_x'] for r in res):.2f}  max rise {1000 * st.mean(r['max_rise'] for r in res):4.0f} mm"
              f"  falls {sum(r['falls'] for r in res)}  landing {res[0]['landing']}")


if __name__ == "__main__":
    main()
