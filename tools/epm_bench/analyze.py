#!/usr/bin/env python3
"""analyze.py — Kalman references and seed-averaged tables for epm_kalman_bench output.

Charter: docs/plans-and-designs/epm_kalman_lessons_plan.md (Stage 0.2).

Usage:
  analyze.py <seed_*.jsonl ...>                       one arm: per-seed metrics, mean ± std
  analyze.py --compare <base_dir> <arm_dir>           paired by seed: arm − base, Student-t 95 % CI

Every file carries a header line naming its scenario; all files of one call must share it.
The references are exact (numpy only):
  S1   sample mean of the samples the baked node actually saw = the Kalman filter for a constant
  S2   scalar steady-state Kalman filter for a random walk observed in noise
  S3   steady-state matrix Kalman filter for the damped-rotation target (per axis, DARE iteration)
  S5   inverse-variance fusion of two sensors
"""
import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

# Student-t 97.5 % quantiles, df 1..30; beyond that the normal 1.96 is used.
_T975 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306,
         9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
         16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086, 21: 2.080, 22: 2.074,
         23: 2.069, 24: 2.064, 25: 2.060, 26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042}


def t975(df):
    return _T975.get(df, 1.96)


# --------------------------------------------------------------------------- loading

def load(path):
    header, ticks, dumps = None, [], []
    with open(path) as f:
        for line in f:
            j = json.loads(line)
            ev = j.get("event")
            if ev == "header":
                header = j
            elif ev == "nodes":
                dumps.append(j)
            else:
                ticks.append(j)
    if header is None:
        raise SystemExit(f"{path}: no header line")
    return header, ticks, dumps


def arr(ticks, key):
    return np.array([t[key] for t in ticks], dtype=float)


def tok(ticks, field, sub="tok"):
    return np.array([(t[sub] or {}).get(field, np.nan) if t.get(sub) is not None else np.nan
                     for t in ticks], dtype=float)


def node_dump_at(dumps, t, epm=None):
    """The node dump emitted at tick t (or the last one before it)."""
    best = None
    for d in dumps:
        if epm is not None and d.get("epm") != epm:
            continue
        if d["t"] <= t:
            best = d
    return best


# --------------------------------------------------------------------------- S1

def metrics_S1(h, ticks, dumps):
    mu = np.array(h["mu"])
    x = arr(ticks, "x")
    winner = tok(ticks, "winner")
    just_baked = np.array([bool((t["tok"] or {}).get("just_baked", False)) for t in ticks])
    out = {"nodes_end": tok(ticks, "nodes")[-1], "baked_end": tok(ticks, "baked")[-1]}
    if not just_baked.any():
        out["bake_tick"] = float("nan")
        return out
    ib = int(np.argmax(just_baked))              # first bake
    bt = int(ticks[ib]["t"])
    nid = int(winner[ib])
    d = node_dump_at(dumps, bt)
    proto = None
    if d is not None:
        for n in d["nodes"]:
            if n["id"] == nid:
                proto = np.array(n["proto"])
    out["bake_tick"] = bt
    if proto is None:
        return out
    # The Kalman-optimal estimate given the SAME data: the mean of the seed
    # sample (bootstrap node id 0 ← x_1, id 1 ← x_2) and every sample that
    # node won up to the bake tick.
    # Tick 2 is the GNG's bootstrap return ({0, 0.0}), not a win: skip t <= 2.
    seen = [x[i] for i in range(ib + 1) if winner[i] == nid and ticks[i]["t"] > 2]
    seed = x[nid] if nid in (0, 1) else None
    samples = ([seed] if seed is not None else []) + seen
    smean = np.mean(np.stack(samples), axis=0)
    out["mse_proto"] = float(np.sum((proto - mu) ** 2))
    out["mse_optimal"] = float(np.sum((smean - mu) ** 2))
    out["n_samples"] = len(samples)
    if seed is not None:
        off = seed - mu
        out["seed_weight"] = float(np.dot(proto - mu, off) / max(np.dot(off, off), 1e-12))
    return out


# --------------------------------------------------------------------------- S1m

def metrics_S1m(h, ticks, dumps):
    k = arr(ticks, "k").astype(int)
    winner = tok(ticks, "winner").astype(int)
    n = len(ticks)
    half = n // 2
    maj = {}
    for w, kk in zip(winner[half:], k[half:]):
        maj.setdefault(int(w), Counter())[int(kk)] += 1
    majority = {w: c.most_common(1)[0][0] for w, c in maj.items()}
    purity = float(np.mean([majority[int(w)] == int(kk) for w, kk in zip(winner[half:], k[half:])]))
    distinct = len(set(winner[-1000:].tolist()))
    out = {"purity": purity, "distinct_winners_last1000": distinct,
           "nodes_end": tok(ticks, "nodes")[-1], "baked_end": tok(ticks, "baked")[-1]}
    mus = np.array(h["mus"])
    d = node_dump_at(dumps, ticks[-1]["t"])
    if d is not None:
        errs = []
        for nd in d["nodes"]:
            if nd["visits"] >= 1 and nd["id"] in majority:
                p = np.array(nd["proto"])
                errs.append(float(np.sum((p - mus[majority[nd["id"]]]) ** 2)))
        if errs:
            out["mse_proto_mean"] = float(np.mean(errs))
    return out


# --------------------------------------------------------------------------- S2

def metrics_S2(h, ticks, dumps):
    dim = h["dim"]
    r = h["sd_per_dim"] ** 2
    q = h["qsd_per_dim"] ** 2
    qe = tok(ticks, "qe")
    half = len(ticks) // 2
    sigma2 = h["sigma"] ** 2
    p_prior = (q + math.sqrt(q * q + 4 * q * r)) / 2.0
    p_post = p_prior * r / (p_prior + r)
    return {
        "epm_track_mse": float(np.nanmean(qe[half:] ** 2) - sigma2),
        "kf_track_mse": dim * p_post,
        "epm_qe2": float(np.nanmean(qe[half:] ** 2)),
        "kf_prefit_resid2": dim * (p_prior + r),
        "nodes_end": tok(ticks, "nodes")[-1], "baked_end": tok(ticks, "baked")[-1],
    }


# --------------------------------------------------------------------------- S3

def kf_innovation_variance(A, Q, r):
    """Steady-state innovation variance of a 2-state linear system observed through H = [1, 0]."""
    A = np.array(A, dtype=float)
    Q = np.array(Q, dtype=float)
    H = np.array([[1.0, 0.0]])
    P = np.eye(2)
    S = None
    for _ in range(5000):
        P = A @ P @ A.T + Q
        S = float((H @ P @ H.T).item() + r)
        K = P @ H.T / S
        P = (np.eye(2) - K @ H) @ P
    return S


def metrics_S3(h, ticks, dumps):
    resid = np.array([t["resid"] if (t.get("resid") is not None and len(t["resid"]) == 2) else [np.nan, np.nan]
                      for t in ticks], dtype=float)
    y = arr(ticks, "y")
    half = len(ticks) // 2
    r2 = np.nansum(resid ** 2, axis=1)
    S = kf_innovation_variance(h["A_axis"], np.eye(2) * h["q_sd"] ** 2, h["r_sd_axis"] ** 2)
    ema, alpha = None, 0.02
    ema_curve = []
    for v in r2:
        if np.isnan(v):
            ema_curve.append(np.nan); continue
        ema = v if ema is None else (1 - alpha) * ema + alpha * v
        ema_curve.append(ema)
    ema_curve = np.array(ema_curve)
    final = float(np.nanmedian(ema_curve[half:]))
    conv = next((int(ticks[i]["t"]) for i in range(len(ticks)) if ema_curve[i] <= 1.5 * final), None)
    persist = np.sum((y[1:] - y[:-1]) ** 2, axis=1)
    return {
        "epm_resid2": float(np.nanmean(r2[half:])),
        "kf_innov_var": 2.0 * S,
        "persistence_resid2": float(np.mean(persist[half:])),
        "conv_tick": conv if conv is not None else float("nan"),
        "dp_err_end": float(ticks[-1]["dp_err"]) if ticks[-1].get("dp_err") is not None else float("nan"),
        "nodes_end": tok(ticks, "nodes")[-1], "baked_end": tok(ticks, "baked")[-1],
    }


# --------------------------------------------------------------------------- S4

def metrics_S4(h, ticks, dumps):
    trans = arr(ticks, "trans").astype(int)
    ts = tok(ticks, "ts")
    tle = tok(ticks, "tle")
    warm = np.array([t["t"] > 1000 for t in ticks])
    exp_m = np.nanmean(ts[warm & (trans == 1)]) if np.any(warm & (trans == 1)) else np.nan
    une_m = np.nanmean(ts[warm & (trans == 2)]) if np.any(warm & (trans == 2)) else np.nan
    none_m = np.nanmean(ts[warm & (trans == 0)])
    return {
        "ts_expected": float(exp_m), "ts_unexpected": float(une_m), "ts_within": float(none_m),
        "ts_ratio_unexp_over_exp": float(une_m / exp_m) if exp_m and not np.isnan(exp_m) and exp_m > 0 else float("nan"),
        "tle_expected": float(np.nanmean(tle[warm & (trans == 1)])) if np.any(warm & (trans == 1)) else np.nan,
        "tle_unexpected": float(np.nanmean(tle[warm & (trans == 2)])) if np.any(warm & (trans == 2)) else np.nan,
        "n_unexpected": int(np.sum(warm & (trans == 2))),
        "nodes_end": tok(ticks, "nodes")[-1], "baked_end": tok(ticks, "baked")[-1],
    }


# --------------------------------------------------------------------------- S5

def metrics_S5(h, ticks, dumps):
    sa2 = h["sigma"] ** 2
    sb2 = (h["sigma"] * h["sigma_b_ratio"]) ** 2
    w_a_opt = (1 / sa2) / (1 / sa2 + 1 / sb2)
    ta = "reality.sensor.a"; tb = "reality.sensor.b"
    trust_a = np.array([t["trust"].get(ta, np.nan) for t in ticks])
    trust_b = np.array([t["trust"].get(tb, np.nan) for t in ticks])
    s = arr(ticks, "s")
    dim = s.shape[1]
    fused = np.array([t["fused"] if (t.get("fused") is not None and len(t["fused"]) == dim) else [np.nan] * dim
                      for t in ticks], dtype=float)
    tt = arr(ticks, "t")
    end = h["dead_at"] if h["dead_at"] >= 0 else h["ticks"]
    win = (tt >= 1000) & (tt < end)
    out = {
        "trust_a_mean": float(np.nanmean(trust_a[win])),
        "trust_a_opt": w_a_opt,
        "trust_a_std": float(np.nanstd(trust_a[win])),
        "fused_mse": float(np.nanmean(np.sum((fused[win] - s[win]) ** 2, axis=1))),
        "fusion_mse_opt": 1.0 / (1 / sa2 + 1 / sb2),
        "best_single_mse": sa2,
        "nodes_a_end": tok(ticks, "nodes", "tok_a")[-1], "baked_a_end": tok(ticks, "baked", "tok_a")[-1],
        "nodes_b_end": tok(ticks, "nodes", "tok_b")[-1], "baked_b_end": tok(ticks, "baked", "tok_b")[-1],
    }
    if h["dead_at"] >= 0:
        after = tt >= h["dead_at"]
        strip = next((int(tt[i]) - h["dead_at"] for i in range(len(tt)) if after[i] and trust_b[i] < 0.1), None)
        out["dead_strip_ticks"] = strip if strip is not None else float("nan")
        late = tt >= h["dead_at"] + 200
        out["trust_b_after_dead"] = float(np.nanmean(trust_b[late]))
        out["fused_mse_after_dead"] = float(np.nanmean(np.sum((fused[late] - s[late]) ** 2, axis=1)))
    if h["placeholder_at"] >= 0:
        i = int(np.argmax(tt == h["placeholder_at"]))
        out["trust_b_at_placeholder"] = float(trust_b[i]) if not np.isnan(trust_b[i]) else 0.0
        out["trust_b_before_placeholder"] = float(trust_b[i - 1])
    return out


# --------------------------------------------------------------------------- S6

def metrics_S6(h, ticks, dumps):
    """Vocabulary turnover on a gait-like ring: whole-run distinct winners against the
    per-quarter count (the picrawler read that separated a churning vocabulary from a
    larger one), plus purity of winner→pose in the last quarter."""
    k = arr(ticks, "k").astype(int)
    winner = tok(ticks, "winner").astype(int)
    n = len(ticks); q = n // 4
    quarters = [len(set(winner[i * q:(i + 1) * q].tolist())) for i in range(4)]
    whole = len(set(winner.tolist()))
    last = slice(3 * q, n)
    maj = {}
    for w, kk in zip(winner[last], k[last]):
        maj.setdefault(int(w), Counter())[int(kk)] += 1
    majority = {w: c.most_common(1)[0][0] for w, c in maj.items()}
    purity = float(np.mean([majority[int(w)] == int(kk) for w, kk in zip(winner[last], k[last])]))
    qe = tok(ticks, "qe")
    return {
        "ids_whole_run": whole,
        "ids_per_quarter": float(np.mean(quarters)),
        "turnover": whole / max(float(np.mean(quarters)), 1.0),
        "purity_last_quarter": purity,
        "qe2_second_half": float(np.nanmean(qe[n // 2:] ** 2)),
        "nodes_end": tok(ticks, "nodes")[-1], "baked_end": tok(ticks, "baked")[-1],
    }


METRICS = {"S1": metrics_S1, "S1m": metrics_S1m, "S2": metrics_S2, "S3": metrics_S3,
           "S4": metrics_S4, "S5": metrics_S5, "S6": metrics_S6}


# --------------------------------------------------------------------------- tables

def per_seed(paths):
    rows, scenario = {}, None
    for p in sorted(paths):
        h, ticks, dumps = load(p)
        scenario = scenario or h["scenario"]
        if h["scenario"] != scenario:
            raise SystemExit(f"mixed scenarios: {scenario} vs {h['scenario']} in {p}")
        rows[int(h["seed"])] = METRICS[scenario](h, ticks, dumps)
    return scenario, rows


def fmt(v):
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return "nan"
    return f"{v:.4g}"


def summarize(scenario, rows):
    keys = sorted({k for r in rows.values() for k in r})
    print(f"### {scenario}  (n = {len(rows)} seeds)\n")
    print("| metric | mean | std | min | max |")
    print("|---|---|---|---|---|")
    for k in keys:
        v = np.array([rows[s][k] for s in sorted(rows) if k in rows[s]], dtype=float)
        v = v[~np.isnan(v)]
        if len(v) == 0:
            print(f"| {k} | nan | | | |"); continue
        print(f"| {k} | {fmt(v.mean())} | {fmt(v.std(ddof=1) if len(v) > 1 else 0.0)} | {fmt(v.min())} | {fmt(v.max())} |")
    print()


def compare(base_dir, arm_dir):
    sb, rb = per_seed(glob.glob(os.path.join(base_dir, "*.jsonl")))
    sa, ra = per_seed(glob.glob(os.path.join(arm_dir, "*.jsonl")))
    if sa != sb:
        raise SystemExit(f"scenario mismatch {sb} vs {sa}")
    seeds = sorted(set(rb) & set(ra))
    keys = sorted({k for r in list(rb.values()) + list(ra.values()) for k in r})
    print(f"### {sb}  paired arm − base  (n = {len(seeds)} shared seeds)\n")
    print("| metric | base mean | arm mean | delta | 95 % CI | ratio arm/base |")
    print("|---|---|---|---|---|---|")
    for k in keys:
        pairs = [(rb[s].get(k, np.nan), ra[s].get(k, np.nan)) for s in seeds]
        b = np.array([p[0] for p in pairs], dtype=float)
        a = np.array([p[1] for p in pairs], dtype=float)
        ok = ~(np.isnan(a) | np.isnan(b))
        if ok.sum() == 0:
            continue
        d = a[ok] - b[ok]
        n = len(d)
        ci = t975(n - 1) * d.std(ddof=1) / math.sqrt(n) if n > 1 else float("nan")
        ratio = a[ok].mean() / b[ok].mean() if b[ok].mean() != 0 else float("nan")
        print(f"| {k} | {fmt(b[ok].mean())} | {fmt(a[ok].mean())} | {fmt(d.mean())} | ± {fmt(ci)} | {fmt(ratio)} |")
    print()


def main(argv):
    if not argv:
        print(__doc__); return 2
    if argv[0] == "--compare":
        if len(argv) != 3:
            print(__doc__); return 2
        compare(argv[1], argv[2]); return 0
    scenario, rows = per_seed(argv)
    summarize(scenario, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
