#!/usr/bin/env python3
"""tof_studies.py — studies 1 and 2 of the ToF work (2026-09-12).

S1  one EPM on the sensor's FULL output: what the 64 dims contain (PCA), what each encoder
    keeps of it, and whether the GNG's vocabulary lines up with the visible structure
    (CLAUDE.md §0 rule 2: if the node count says one thing and the scatter says several,
    believe the scatter).
S2  several EPMs in different ROLES off the same sensor: does each one's vocabulary predict
    a different thing -- object identity vs the duck's own location?

World-derived labels are INSTRUMENTATION: they judge what the sensor and the EPMs separate,
and no brain ever sees them.
"""
import json, math, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from tof_prep import label_fov

def load_epm(path):
    arms, ticks, nodes = None, [], {}
    for line in open(path):
        j = json.loads(line)
        ev = j.get("event")
        if ev == "header": arms = j["arms"]
        elif ev == "tick": ticks.append(j)
        elif ev == "nodes": nodes[j["arm"]] = j["nodes"]
    return arms, ticks, nodes

def pca(X, k=2):
    Xc = X - X.mean(0)
    U, S, Vt = np.linalg.svd(Xc, full_matrices=False)
    var = (S ** 2) / max(1, len(Xc) - 1)
    return Xc @ Vt[:k].T, var / var.sum(), Vt[:k]

def cmi(w, o, l):
    """I(W;O | L): does the vocabulary still say WHICH OBJECT once you know WHERE the duck is?
    A code that is only a place code scores ~0 here however high its raw NMI(W;O) is."""
    tot, n = 0.0, len(w)
    for c in np.unique(l):
        m = l == c
        if m.sum() < 30: continue
        tot += (m.sum() / n) * mi(w[m], o[m])
    return float(tot)


def mi(a, b):
    ua, ia = np.unique(a, return_inverse=True); ub, ib = np.unique(b, return_inverse=True)
    if len(ua) < 2 or len(ub) < 2: return 0.0
    P = np.zeros((len(ua), len(ub))); np.add.at(P, (ia, ib), 1.0); P /= len(a)
    pa = P.sum(1, keepdims=True); pb = P.sum(0, keepdims=True); nz = P > 0
    return float((P[nz] * np.log(P[nz] / (pa @ pb)[nz])).sum())


def kind_f1(w, lab, kind):
    """Best F1 of 'the winner is in S' as a detector of one object kind, S grown greedily by
    precision -- i.e. does the vocabulary contain nodes that MEAN this kind?"""
    y = lab == kind
    if y.sum() < 30: return None
    nodes = np.unique(w)
    prec = [(n, float(y[w == n].mean()), int((w == n).sum())) for n in nodes]
    prec = [p for p in prec if p[2] >= 10]
    prec.sort(key=lambda t: -t[1])
    best, sel = 0.0, set()
    for n, _, _ in prec:
        sel.add(n)
        pred = np.isin(w, list(sel))
        tp = (pred & y).sum()
        if tp == 0: continue
        f1 = 2 * tp / (pred.sum() + y.sum())
        best = max(best, float(f1))
    return round(best, 3), round(float(y.mean()), 3)


def nmi(a, b):
    """normalised mutual information, sqrt form; a and b are integer label arrays"""
    a = np.asarray(a); b = np.asarray(b)
    ua, ia = np.unique(a, return_inverse=True); ub, ib = np.unique(b, return_inverse=True)
    n = len(a)
    if len(ua) < 2 or len(ub) < 2: return 0.0
    P = np.zeros((len(ua), len(ub)))
    np.add.at(P, (ia, ib), 1.0)
    P /= n
    pa = P.sum(1, keepdims=True); pb = P.sum(0, keepdims=True)
    nz = P > 0
    I = (P[nz] * np.log(P[nz] / (pa @ pb)[nz])).sum()
    Ha = -(pa[pa > 0] * np.log(pa[pa > 0])).sum(); Hb = -(pb[pb > 0] * np.log(pb[pb > 0])).sum()
    return float(I / math.sqrt(Ha * Hb)) if Ha > 0 and Hb > 0 else 0.0

def main(npz_path, epm_path, out_json, from_t=700.0):
    d = np.load(npz_path, allow_pickle=True); d = {k: d[k] for k in d.files}
    best, dist, bear, names, kinds = label_fov(d, max_range=2.0)
    arms, ticks, nodes = load_epm(epm_path)
    # the bench's "i" indexes its own DEDUPED frame list, not the host tick; match on time,
    # which both sides carry (§3.2 rule 7 in miniature -- never join two streams by assumption)
    tt = np.array([tk["t"] for tk in ticks])
    idx = np.clip(np.searchsorted(d["t"], tt), 0, len(d["t"]) - 1)
    assert np.abs(d["t"][idx] - tt).max() < 0.03, "time join failed"
    keep = tt >= from_t
    idx_k = idx[keep]
    ticks_k = [tk for tk, kp in zip(ticks, keep) if kp]

    # --- labels on the kept frames ---
    kind_of = {i: k for i, k in enumerate(kinds)}
    lab_obj = np.array([kind_of.get(int(b), "none") if b >= 0 else "none" for b in best[idx_k]])
    lab_obj_i = np.unique(lab_obj, return_inverse=True)[1]
    CELL = 0.5
    lab_loc = (np.floor(d["x"][idx_k] / CELL).astype(int) * 100 + np.floor(d["y"][idx_k] / CELL).astype(int))
    lab_loc_i = np.unique(lab_loc, return_inverse=True)[1]
    lab_loc1 = (np.floor(d["x"][idx_k]).astype(int) * 100 + np.floor(d["y"][idx_k]).astype(int))
    lab_loc1_i = np.unique(lab_loc1, return_inverse=True)[1]   # 1 m cells, for conditioning
    # THE CONTROL THAT DECIDES IT.  "which object is in view" is largely a function of WHERE the
    # duck stands and WHICH WAY it faces, so conditioning on a location cell alone leaves the
    # heading free to carry the label.  Condition on POSE = (1 m cell, heading octant): if the
    # conditional information collapses, the vocabulary is a pose code wearing an object's name.
    oct_ = np.floor(((d["hdg"][idx_k, 0] % (2 * np.pi)) / (2 * np.pi)) * 8).astype(int)
    lab_pose = lab_loc1 * 10 + oct_
    lab_pose_i = np.unique(lab_pose, return_inverse=True)[1]
    standing = d["stop"][idx_k] > 0

    res = {"npz": Path(npz_path).name, "frames": int(keep.sum()), "from_t": from_t,
           "label_mix": {k: int(v) for k, v in Counter(lab_obj).most_common()},
           "n_loc_cells": int(len(np.unique(lab_loc_i))), "arms": {}}

    # --- S1: what the raw 64 contain ---
    raw = d["tofr"][idx_k].copy()
    raw[raw < 0] = 4.0
    raw /= 4.0
    P, ev, _ = pca(raw, 4)
    res["raw64_pca_var"] = [round(float(v), 4) for v in ev[:6]]
    res["raw64_dims_for_90pct"] = int(np.searchsorted(np.cumsum(ev), 0.90) + 1)
    # how separable are the object classes in the raw input? a nearest-centroid readout
    def centroid_acc(X, lab):
        cls = np.unique(lab); cen = np.stack([X[lab == c].mean(0) for c in cls])
        pred = cls[np.argmin(((X[:, None, :] - cen[None]) ** 2).sum(-1), 1)]
        return float((pred == lab).mean()), {str(c): float((pred[lab == c] == c).mean()) for c in cls}
    acc, per = centroid_acc(P[:, :4], lab_obj)
    res["raw64_pca4_centroid_acc"] = round(acc, 3)
    res["raw64_pca4_per_class"] = {k: round(v, 3) for k, v in per.items()}
    res["raw64_majority"] = round(float(Counter(lab_obj).most_common(1)[0][1] / len(lab_obj)), 3)

    for name, meta in arms.items():
        w = np.array([tk[name]["winner"] if tk[name] else -1 for tk in ticks_k])
        tle = np.array([tk[name]["tle"] if tk[name] else np.nan for tk in ticks_k])
        qe = np.array([tk[name]["qe"] if tk[name] else np.nan for tk in ticks_k])
        nn = np.array([tk[name]["nodes"] if tk[name] else 0 for tk in ticks_k])
        bk = np.array([tk[name]["baked"] if tk[name] else 0 for tk in ticks_k])
        ok = w >= 0
        lat = np.array([tk[name + "_lat"] for tk in ticks_k if (name + "_lat") in tk and tk[name]])
        a = {"view": meta["view"], "kind": meta["kind"], "dims": meta["dims"],
             "nodes_end": int(nn[-1]), "baked_end": int(bk[-1]),
             "distinct_winners": int(len(np.unique(w[ok]))),
             "tle_mean": round(float(np.nanmean(tle)), 4), "tle_sd": round(float(np.nanstd(tle)), 4),
             "qe_mean": round(float(np.nanmean(qe)), 4),
             "switch_per_min": round(float((np.diff(w[ok]) != 0).sum() / (len(w[ok]) / 12.5 / 60)), 1),
             "nmi_object": round(nmi(w[ok], lab_obj_i[ok]), 4),
             "nmi_location": round(nmi(w[ok], lab_loc_i[ok]), 4),
             "nmi_standing": round(nmi(w[ok], standing[ok].astype(int)), 4),
             "cmi_object_given_place": round(cmi(w[ok], lab_obj_i[ok], lab_loc1_i[ok]), 4),
             "cmi_object_given_pose": round(cmi(w[ok], lab_obj_i[ok], lab_pose_i[ok]), 4),
             "mi_object": round(mi(w[ok], lab_obj_i[ok]), 4),
             "kind_f1": {k: kind_f1(w[ok], lab_obj[ok], k) for k in
                         ("block", "ball", "chair", "shelf", "none")}}
        if len(lat) > 50:
            Pl, evl, _ = pca(lat, 4)
            a["latent_pca_var"] = [round(float(v), 4) for v in evl[:6]]
            a["latent_dims_for_90pct"] = int(np.searchsorted(np.cumsum(evl), 0.90) + 1)
            a["latent_spread"] = round(float(np.linalg.norm(lat - lat.mean(0), axis=1).mean()), 4)
        if name in nodes and nodes[name]:
            proto = np.array([n["proto"] for n in nodes[name]], dtype=float)
            vis = np.array([n["visits"] for n in nodes[name]])
            a["node_visits_median"] = int(np.median(vis))
            a["node_proto_spread"] = round(float(np.linalg.norm(proto - proto.mean(0), axis=1).mean()), 4)
        res["arms"][name] = a

    json.dump(res, open(out_json, "w"), indent=1)
    # ---- the table ----
    print(f"\n=== S1/S2  {Path(npz_path).name}, {int(keep.sum())} frames from {from_t:.0f} s ===")
    print(f"  raw 64 dims: {res['raw64_dims_for_90pct']} PCs hold 90 % of the variance; "
          f"PC1-4 = {', '.join(f'{v:.2f}' for v in res['raw64_pca_var'][:4])}")
    print(f"  nearest-centroid readout of the object class from 4 raw PCs: {acc:.3f} "
          f"(majority baseline {res['raw64_majority']:.3f})")
    print("    per class: " + "  ".join(f"{k} {v:.2f}" for k, v in per.items()))
    print(f"  label mix: {res['label_mix']}")
    print(f"\n  {'arm':12s} {'view':7s} {'kind':9s} {'dims':>4s} {'nodes':>6s} {'baked':>6s} "
          f"{'winners':>8s} {'sw/min':>7s} {'TLE':>6s} {'NMI obj':>8s} {'NMI loc':>8s} {'lat 90%':>8s} {'spread':>7s}")
    for n, a in res["arms"].items():
        print(f"  {n:12s} {a['view']:7s} {a['kind']:9s} {a['dims']:>4d} {a['nodes_end']:>6d} {a['baked_end']:>6d} "
              f"{a['distinct_winners']:>8d} {a['switch_per_min']:>7.0f} {a['tle_mean']:>6.3f} "
              f"{a['nmi_object']:>8.3f} {a['nmi_location']:>8.3f} "
              f"{a.get('latent_dims_for_90pct', -1):>8d} {a.get('latent_spread', float('nan')):>7.3f}")
    # how many distinct poses does the duck ever view each object from?  A pose-invariant object
    # code cannot be LEARNED from one viewpoint, however good the encoder is.
    print("\n  viewpoint diversity (the data's own ceiling on any object vocabulary):")
    print(f"    {'kind':8s} {'frames':>7s} {'1 m cells':>10s} {'pose cells':>11s} {'range span m':>13s}")
    for kind in ("block", "ball", "chair", "shelf", "rug", "table"):
        m = lab_obj == kind
        if m.sum() < 20: continue
        dd = dist[idx_k][np.arange(len(idx_k)), np.maximum(best[idx_k], 0)][m]
        print(f"    {kind:8s} {int(m.sum()):>7d} {len(np.unique(lab_loc1[m])):>10d} "
              f"{len(np.unique(lab_pose[m])):>11d} {dd.max()-dd.min():>13.2f}")
        res.setdefault("viewpoints", {})[kind] = {
            "frames": int(m.sum()), "loc_cells": int(len(np.unique(lab_loc1[m]))),
            "pose_cells": int(len(np.unique(lab_pose[m]))), "range_span": round(float(dd.max() - dd.min()), 2)}

    print(f"\n  {'arm':12s} {'I(W;O)':>7s} {'I(W;O|place)':>13s} {'I(|pose)':>13s} {'  block F1':>10s} {'ball F1':>8s} {'chair F1':>9s} {'shelf F1':>9s}")
    for n, a in res["arms"].items():
        f1 = a["kind_f1"]
        def g(k):
            v = f1.get(k)
            return f"{v[0]:.2f}/{v[1]:.2f}" if v else "   --   "
        print(f"  {n:12s} {a['mi_object']:>7.3f} {a['cmi_object_given_place']:>13.3f}{a['cmi_object_given_pose']:>13.3f} "
              f"{g('block'):>10s} {g('ball'):>8s} {g('chair'):>9s} {g('shelf'):>9s}")
    print("   (F1 / base rate: the best 'winner in S' detector for that kind — does the vocabulary")
    print("    contain nodes that MEAN it? I(W;O|place) is the same question with location known.)")
    return res

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
