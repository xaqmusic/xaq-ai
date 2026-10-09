#!/usr/bin/env python3
"""presence_roc — does the thing belief know the thing is gone? (the chase push, design doc §17.113, 2026-10-09)

From the host's FULL JSONL (the `thb` record, ThingBelief's [presence, confirming, refuting, since, have]) and the scene
manifest (CLOUD_OBJECTS_MANIFEST): at every arrival stop, the target's truth -- 'there' (taken on a movable thing that is
still within 0.15 m of where it was), 'moved' (it is not), 'nothing' (taken on no movable thing) -- against four scores:
  cumulative   ThingBelief's presence (evidence since the target was set)
  since        minus the refuting ticks since the last confirming one
  last         the last evidence (+1 confirming / -1 refuting)
  recent       (1 + confirming) / (2 + all) over the last 50 evidence ticks
Each: the AUC of 'there' against 'moved', 'nothing' and both; and for the cumulative score, what releasing under a
threshold keeps and drops.  And the RELEASE RULES replayed over every target's evidence during the walk (T2): which arrivals
each would have released before they happened --
  ub              Beta(1 + confirming, 1 + refuting) since the target was set: mean + sd < 0.5
  after_confirm   only a thing confirmed at least once: Beta(2, 1 + refuting since the last confirmation): mean + sd < 0.5
  either          whichever fires first
Usage: CLOUD_OBJECTS_MANIFEST=... presence_roc.py "glob" [...]
Scoring only: the truth labels; nothing here is a brain input.
"""
import glob, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_objects as co  # noqa: E402
from moved_targets import world  # noqa: E402


def auc(pos, neg):
    s = n = 0
    for a in pos:
        for b in neg:
            s += 1 if a > b else (0.5 if a == b else 0); n += 1
    return s / n if n else float("nan")


def ub(a, b):
    n = a + b
    return a / n + math.sqrt(a * b / (n * n * (n + 1)))


RULES = {
    "ub": lambda c, r, rs: ub(1 + c, 1 + r) < 0.5,
    "after_confirm": lambda c, r, rs: c >= 1 and ub(2, 1 + rs) < 0.5,
}
RULES["either"] = lambda c, r, rs: RULES["ub"](c, r, rs) or RULES["after_confirm"](c, r, rs)


def main():
    _h, lay, movable, _f = co.load_scene()
    rows = []
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        seek_prev, tgt, prev, seq = 0.0, None, None, []
        fired = {k: False for k in RULES}
        for line in open(path):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line)
            s, ch, tb = r.get("seek"), r.get("chase"), r.get("thb")
            sv = s[0] if s else 0.0
            if tb:
                if prev is None or tb[3] < prev[3] or tb[3] == 1: seq = []
                elif tb[1] > prev[1]: seq.append(1)
                elif tb[2] > prev[2]: seq.append(-1)
                prev = tb
                if tb[4]:
                    rs = 0
                    for x in reversed(seq):
                        if x == 1: break
                        rs += 1
                    for k, rule in RULES.items():
                        if not fired[k] and rule(tb[1], tb[2], rs): fired[k] = True
            if sv > 0 and seek_prev == 0 and ch:
                fired = {k: False for k in RULES}
                wx, wy = world(r, ch[2], ch[3])
                d, k, ox, oy = min((math.hypot(ox - wx, oy - wy) - e, k, ox, oy) for k, ox, oy, e in co.objects_at(r, lay, movable))
                tgt = dict(kind=k if d < 0.2 else None, ox=ox, oy=oy)
            if r.get("event") == "stop:arrive" and tgt and tb:
                if tgt["kind"] is None: lab = "nothing"
                else:
                    now = [(a, b) for k, a, b, _e in co.objects_at(r, lay, movable) if k == tgt["kind"]]
                    lab = "moved" if min(math.hypot(a - tgt["ox"], b - tgt["oy"]) for a, b in now) > 0.15 else "there"
                since = 0
                for x in reversed(seq):
                    if x == 1: break
                    since += 1
                recent = seq[-50:]
                rows.append(dict(lab=lab, cumulative=tb[0], since=-since, last=seq[-1] if seq else 0,
                                 recent=(1 + sum(x == 1 for x in recent)) / (2 + len(recent)), fired=dict(fired)))
            seek_prev = sv
    n = {k: sum(r["lab"] == k for r in rows) for k in ("there", "moved", "nothing")}
    print(f"{len(rows)} arrivals: there {n['there']}, moved {n['moved']}, nothing {n['nothing']}")
    for key in ("cumulative", "since", "last", "recent"):
        th = [r[key] for r in rows if r["lab"] == "there"]
        print(f"  {key:10s} AUC vs moved {auc(th, [r[key] for r in rows if r['lab'] == 'moved']):.2f}  "
              f"vs nothing {auc(th, [r[key] for r in rows if r['lab'] == 'nothing']):.2f}  "
              f"vs both {auc(th, [r[key] for r in rows if r['lab'] != 'there']):.2f}")
    th = [r["cumulative"] for r in rows if r["lab"] == "there"]; bad = [r["cumulative"] for r in rows if r["lab"] != "there"]
    for thr in (0.05, 0.1, 0.2, 0.3):
        print(f"  release under {thr} (at arrival): keeps {100 * sum(p >= thr for p in th) / max(1, len(th)):.0f} % of 'there', "
              f"drops {100 * sum(p < thr for p in bad) / max(1, len(bad)):.0f} % of moved + nothing")

    for k in RULES:
        rel = {lab: sum(r["fired"][k] for r in rows if r["lab"] == lab) for lab in ("there", "moved", "nothing")}
        print(f"  rule {k:13s} during the walk releases: there {rel['there']}/{n['there']} ({100 * rel['there'] / max(1, n['there']):.0f} %), "
              f"moved {rel['moved']}/{n['moved']} ({100 * rel['moved'] / max(1, n['moved']):.0f} %), "
              f"nothing {rel['nothing']}/{n['nothing']} ({100 * rel['nothing'] / max(1, n['nothing']):.0f} %)")


if __name__ == "__main__":
    main()
