#!/usr/bin/env python3
"""M1: does the ToF SEE the floor-level objects?   M2: is a stumble visible in any channel?
Both are go/no-go checks before any object-vocabulary work (the ToF studies, 2026-09-12)."""
import sys, math
import numpy as np
sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
from tof_prep import label_fov, point_heights, EL_DEG, AZ_DEG

def m1(d, name):
    best, dist, bear, names, kinds = label_fov(d, max_range=2.0)
    ci = d["cloud_i"]
    H = point_heights(d)                      # casts x 64, height above floor
    R = d["tofr"][ci]                         # casts x 64, slant range (-1 = none)
    Z = d["tofz"][ci]                         # casts x 64, class
    lab = best[ci]
    dst = dist[ci][np.arange(len(ci)), np.maximum(lab, 0)]
    print(f"\n=== M1  does the ToF see it?  [{name}] ===")
    print(f"  casts {len(ci)};  class mix: Empty {100*(Z==0).mean():.0f}%  TooClose {100*(Z==1).mean():.0f}%"
          f"  Floor {100*(Z==2).mean():.0f}%  Hit {100*(Z==3).mean():.0f}%")
    # the height histogram of every RETURNED point
    h = H[np.isfinite(H)]
    bands = [(-1, .02, "floor  <2cm"), (.02, .10, "low  2-10cm (block/ball)"), (.10, .25, "10-25cm"),
             (.25, .45, "25-45cm (chair/table)"), (.45, 9, ">45cm (wall top/shelf/clock)")]
    print("  height of returned points:")
    for lo, hi, t in bands:
        print(f"    {t:30s} {100*((h>=lo)&(h<hi)).mean():5.1f}%")
    # per object kind, in view and close: is there a return at ITS height, in ITS bearing sector?
    print(f"  {'kind':10s} {'casts in view <1.2m':>19s} {'zones on it':>12s} {'its height band hit':>20s}")
    for kind in ("block", "ball", "chair", "table", "shelf", "rug", "clock"):
        ks = [i for i, k in enumerate(kinds) if k == kind]
        if not ks: continue
        sel = np.isin(lab, ks) & (dst < 1.2)
        if sel.sum() < 20:
            print(f"  {kind:10s} {sel.sum():>19d} {'--':>12s} {'--':>20s}"); continue
        # zones whose azimuth is within the object's angular half-width of its bearing
        b = bear[ci][sel][np.arange(int(sel.sum())), lab[sel]]
        az = np.radians(AZ_DEG)[None, :].repeat(8, 0).ravel()      # zone azimuth, 64
        near_az = np.abs(((az[None, :] - b[:, None]) + np.pi) % (2*np.pi) - np.pi) < math.radians(6.0)
        hs = H[sel]
        got = np.isfinite(hs) & near_az
        # the object's own height band
        top = {"block": 0.10, "ball": 0.12, "chair": 0.50, "table": 0.55, "shelf": 0.90, "rug": 0.02, "clock": 0.55}[kind]
        bot = {"block": 0.015, "ball": 0.015, "chair": 0.05, "table": 0.05, "shelf": 0.05, "rug": -1, "clock": 0.30}[kind]
        band = got & (hs >= bot) & (hs < top)
        print(f"  {kind:10s} {sel.sum():>19d} {got.sum()/max(1,sel.sum()):>12.1f} {band.sum()/max(1,sel.sum()):>20.2f}")

def m2(d, name):
    print(f"\n=== M2  is a stumble visible?  [{name}] ===")
    mt = d["mtle"]; ob = d["obj"].astype(int); wl = d["wall"].astype(int)
    walking = (d["drive"] == 0) & (d["stop"] == 0) & (d["t"] > 700)
    print(f"  motor_tle while walking: mean {np.nanmean(mt[walking]):.4f}  sd {np.nanstd(mt[walking]):.4f}"
          f"  p99 {np.nanpercentile(mt[walking], 99):.4f}")
    for label, sig in (("object contact", ob), ("wall contact", wl)):
        onset = np.where((np.diff(sig) > 0) & walking[1:])[0] + 1
        if len(onset) < 5:
            print(f"  {label}: {len(onset)} onsets while walking — too few"); continue
        base = np.nanmean(mt[walking]); sd = np.nanstd(mt[walking])
        win = np.array([np.nanmax(mt[max(0, o-5):o+50]) for o in onset])
        pre = np.array([np.nanmean(mt[max(0, o-100):o-25]) for o in onset])
        print(f"  {label}: {len(onset)} onsets;  peak motor_tle in the second after "
              f"{np.nanmean(win):.4f} vs {np.nanmean(pre):.4f} before "
              f"({(np.nanmean(win)-base)/sd:+.2f} sd);  fraction above p99 {np.mean(win > np.nanpercentile(mt[walking],99)):.2f}")
    print(f"  tilt at object contact: {np.nanmean(d['tilt'][ob==1]):.2f} deg vs walking {np.nanmean(d['tilt'][walking]):.2f} deg")
    print(f"  ticks past 60 deg (on the floor): {int((d['tilt']>60).sum())}")

if __name__ == "__main__":
    for f in sys.argv[1:]:
        d = np.load(f, allow_pickle=True)
        d = {k: d[k] for k in d.files}
        n = f.split('/')[-1].replace('.npz', '')
        m1(d, n); m2(d, n)
