#!/usr/bin/env python3
"""structure_rule — tall structure consolidated as one immovable object, offline (ten-minutes phase §4, S0, 2026-10-02).

The operator: tall stacks of voxels along the walls, tables and chairs should read as one large immovable object even
where the sampling leaves gaps, and the small fragments at their feet should not be taken for things; a ball by a wall
stays a thing.  The record (§17.78): 73 % of static targets are 8 cm, two-column pieces of a wall's base band, split
off by the ToF's zone spacing; the neighbour count cannot tell them from a thing (13 vs 18 columns within 0.3 m); a row
rule -- the line -- is the re-use.

The rule, on one filed cloud in its own anchor frame (what a brain could compute):
  1. TALL: the columns holding a voxel whose mean height is at or above the small height (16 cm).
  2. CLOSE the tall footprint: dilate each tall column by a disc of radius r = max(1 voxel, K x its range), then erode
     with the radius at each cell's own range.  K is the ToF's zone spacing (45 deg / 8 zones = 0.098 rad): the gap
     the sampling leaves between returns at that range.  A dotted wall becomes a line; the line does not thicken.
  3. A SMALL cluster (the stack rule, the module's form: top < 16 cm, 8 cm <= extent <= 20 cm) is a FRAGMENT when at
     least FRAC of its columns lie within TOL voxels of the closed structure; one that protrudes from it is a thing.

Scored by truth (scoring only): every small cluster of every filed cloud labelled by the manifest (cloud_objects.label:
within 8 cm of a ball or block -> thing; a wall, chair, table or shelf -> structure); "by structure" = a real thing
whose edge is within 0.3 m of a wall face or a furniture footprint.  The baseline is R91's isolation (§17.58): nothing
at or above 25 cm within 25 cm of the cluster's centroid.

Usage: CLOUD_OBJECTS_MANIFEST=... structure_rule.py "glob" [--k 0.098] [--tol 1] [--frac 0.5] [--sweep]
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_objects as co  # noqa: E402

ZONE = math.radians(45.0 / 8.0)
SMALL_EXT_MIN = 0.08
ISO_H, ISO_R = 0.25, 0.25


def small_clusters(V, h, vm):
    """The stack rule's clusters (cloud_objects.clusters), with their columns, filtered to the module's SMALL."""
    cols = collections.defaultdict(list)
    for (ix, iy, _iz, hits, _mm), hh in zip(V, h):
        if hh >= co.BREAK_LO:
            cols[(int(ix), int(iy))].append((hh, hits))
    seeds = {k for k, v in cols.items() if any(co.BREAK_LO <= hh < co.BREAK_HI for hh, _ in v)}
    seen, out = set(), []
    for k in seeds:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            a = stack.pop()
            comp.append(a)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    b = (a[0] + dx, a[1] + dy)
                    if b in seeds and b not in seen:
                        seen.add(b)
                        stack.append(b)
        xs = np.array([(x + 0.5) * vm for x, _ in comp])
        ys = np.array([(y + 0.5) * vm for _, y in comp])
        cx, cy = float(xs.mean()), float(ys.mean())
        rng = math.hypot(cx, cy)
        ext = max(xs.max() - xs.min(), ys.max() - ys.min()) + vm
        gap = max(co.GAP_MIN, 0.12 * rng)
        dil = {(x + dx, y + dy) for x, y in comp for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
        lo = min(hh for kk in comp for hh, _ in cols[kk] if co.BREAK_LO <= hh < co.BREAK_HI)
        top = lo
        for hh in sorted(hh for kk in dil if kk in cols for hh, _ in cols[kk]):
            if hh < lo:
                continue
            if hh > top + gap:
                break
            top = max(top, hh)
        if top < co.SMALL_TOP and SMALL_EXT_MIN <= ext <= co.SMALL_EXT:
            out.append(dict(cx=cx, cy=cy, rng=rng, ext=float(ext), cols=comp))
    return out


def disc(r):
    k = int(math.ceil(r))
    return [(dx, dy) for dx in range(-k, k + 1) for dy in range(-k, k + 1) if dx * dx + dy * dy <= r * r + 1e-9]


def closed_structure(V, h, vm, K):
    tall = {(int(ix), int(iy)) for (ix, iy, *_r), hh in zip(V, h) if hh >= co.SMALL_TOP}
    if not tall:
        return set(), tall

    def rad(c):
        return max(1.0, K * math.hypot((c[0] + 0.5) * vm, (c[1] + 0.5) * vm) / vm)
    dil = set()
    for c in tall:
        for dx, dy in disc(rad(c)):
            dil.add((c[0] + dx, c[1] + dy))
    closed = set()
    for c in dil:
        if all((c[0] + dx, c[1] + dy) in dil for dx, dy in disc(rad(c))):
            closed.add(c)
    return closed | tall, tall


def near(cols, S, tol):
    if tol <= 0:
        return [c in S for c in cols]
    d = disc(tol)
    return [any((c[0] + dx, c[1] + dy) in S for dx, dy in d) for c in cols]


def isolated(cl, V, h, vm):
    for (ix, iy, *_r), hh in zip(V, h):
        if hh >= ISO_H and math.hypot((ix + 0.5) * vm - cl["cx"], (iy + 0.5) * vm - cl["cy"]) <= ISO_R:
            return False
    return True


def collect(paths):
    """Every small cluster of every filed cloud: (truth, by_structure, cloud data needed by the rule)."""
    half, lay, movable, furniture = co.load_scene()
    inner = half - co.WALL_T

    def struct_d(x, y):
        d = inner - max(abs(x), abs(y))
        for o in furniture:
            d = min(d, math.hypot(x - o["x"], y - o["y"]) - o["extent"])
        return d
    clouds = []
    for p in paths:
        for line in open(p):
            if '"cloudv"' not in line:
                continue
            r = json.loads(line)
            c = r["cloudv"]
            if not c["vox"]:
                continue
            V, h = co.mean_heights(c)
            vm = c["voxel_m"]
            objs = co.objects_at(r, lay, movable)
            tr = r.get("train")
            if tr is not None:
                objs = [o for o in objs if o[0] != "train"] + [("train", tr[0], tr[1], 0.09)]
            cls = small_clusters(V, h, vm)
            rows = []
            for cl in cls:
                wx, wy = co.to_world(c["anchor"], cl["cx"], cl["cy"])
                lab = co.label(wx, wy, objs, half, furniture)
                truth = "thing" if lab in ("ball", "block", "train") else ("structure" if lab in ("wall", "chair", "table", "shelf") else "other")
                bys = truth == "thing" and struct_d(wx, wy) < 0.3 + 0.06
                rows.append((cl, truth, bys, isolated(cl, V, h, vm)))
            clouds.append(dict(run=os.path.basename(p), V=V, h=h, vm=vm, rows=rows, walking=c.get("walking", 0)))
    return clouds


def score(clouds, K, tol, frac, tolk=0.0):
    tab = collections.Counter()
    for cd in clouds:
        if not cd["rows"]:
            continue
        S, _tall = closed_structure(cd["V"], cd["h"], cd["vm"], K)
        for cl, truth, bys, iso in cd["rows"]:
            on = near(cl["cols"], S, max(tol, tolk * cl["rng"] / cd["vm"]))
            frag = sum(on) >= frac * len(on)
            key = "thing@struct" if bys else truth
            tab[(key, "line_refuses")] += frag
            tab[(key, "iso_refuses")] += not iso
            tab[(key, "n")] += 1
    return tab


def report(tab, label):
    def pct(k, w):
        n = tab[(k, "n")]
        return f"{100 * tab[(k, w)] / n:5.1f} % of {n:5d}" if n else "   --"
    print(f"{label}")
    for k in ("structure", "thing", "thing@struct", "other"):
        print(f"  {k:13s} refused by the LINE {pct(k, 'line_refuses')}   by ISOLATION (R91) {pct(k, 'iso_refuses')}")


def score_module(paths, vm=0.04):
    """The MODULE's own verdicts (S1): every small cluster of every filed cloud, from the "things" record's tenth value
    (seen_above, host --tof-free-rays + CloudMap free_rays), labelled by truth; the share whose top was NOT seen (the
    small_needs_top refusal: seen_above < top + one voxel)."""
    half, lay, movable, furniture = co.load_scene()
    inner = half - co.WALL_T

    def struct_d(x, y):
        d = inner - max(abs(x), abs(y))
        for o in furniture:
            d = min(d, math.hypot(x - o["x"], y - o["y"]) - o["extent"])
        return d
    tab = collections.Counter()
    for p in paths:
        for line in open(p):
            if '"cloudv"' not in line:
                continue
            r = json.loads(line)
            c = r["cloudv"]
            objs = co.objects_at(r, lay, movable)
            tr = r.get("train")
            if tr is not None:
                objs = [o for o in objs if o[0] != "train"] + [("train", tr[0], tr[1], 0.09)]
            for th in r.get("things", []):
                if len(th) < 10 or not th[8]:
                    continue
                wx, wy = co.to_world(c["anchor"], th[0], th[1])
                lab = co.label(wx, wy, objs, half, furniture)
                truth = "thing" if lab in ("ball", "block", "train") else ("structure" if lab in ("wall", "chair", "table", "shelf") else "other")
                key = "thing@struct" if truth == "thing" and struct_d(wx, wy) < 0.36 else truth
                w = "walk" if c.get("walking") else "stop"
                for kk in (key, key + "/" + w):
                    tab[(kk, "n")] += 1
                    tab[(kk, "unseen")] += th[9] < th[4] + vm
    print("the module's small clusters whose top was NOT seen (small_needs_top would refuse them)")
    for k in ("structure", "thing", "thing@struct", "other"):
        for kk in (k, k + "/stop", k + "/walk"):
            n = tab[(kk, "n")]
            if n:
                print(f"  {kk:20s} {100 * tab[(kk, 'unseen')] / n:5.1f} % of {n:5d}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("globs", nargs="+")
    ap.add_argument("--k", type=float, default=ZONE)
    ap.add_argument("--tol", type=float, default=1.0)
    ap.add_argument("--frac", type=float, default=0.5)
    ap.add_argument("--tolk", type=float, default=0.0, help="tolerance grows with range: max(tol, TOLK x range) (registration smear)")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--module", action="store_true", help="score the module's seen_above from the filed things record")
    a = ap.parse_args()
    paths = sorted(p for g in a.globs for p in glob.glob(g))
    if a.module:
        score_module(paths)
        return
    clouds = collect(paths)
    print(f"{len(paths)} logs, {len(clouds)} filed clouds, {sum(len(c['rows']) for c in clouds)} small clusters")
    print("  ('thing' = a ball, block or train more than 0.3 m from structure; 'thing@struct' = within 0.3 m)")
    if a.sweep:
        for K in (ZONE / 2, ZONE):
            for tolk in (0.0, 0.03, 0.05, 0.08):
                for frac in (0.5,):
                    report(score(clouds, K, 1, frac, tolk), f"K {K:.3f} tol 1 tolk {tolk} frac {frac}")
    else:
        report(score(clouds, a.k, a.tol, a.frac, a.tolk), f"K {a.k:.3f} tol {a.tol} tolk {a.tolk} frac {a.frac}")


if __name__ == "__main__":
    main()
