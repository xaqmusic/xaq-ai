#!/usr/bin/env python3
"""Regenerate the GainEvolver sweep configs from their spec.

    python3 gainevo_make_arms.py <basin|basin3d|winverify|tsweep> [outdir]

PART IV's sweeps each needed 6-27 near-identical configs differing in two or three
fields. Those were originally written into the configs directory and committed,
which left ~78 numbered one-offs sitting beside the ~10 named arms that actually
mean something — and they reproduce nothing on their own, because the runs they
fed wrote their logs to an ephemeral scratch directory.

So the recipe lives here instead. Each sweep is a declarative spec; this emits the
configs plus a `spec.json` carrying gain keys, bounds and start vectors, which is
what the analyzers read for normalization rather than reaching back into a config.

  basin      12 random 8-D starts x 2 bodies  — does the search converge?
  basin3d    the same 12 starts projected onto the 3 gains with authority
  winverify  6 runs at eval_window 12000       — does a longer window change acceptance?
  tsweep     step-size arms from a displaced start — does J fall, and does the
             displaced gain recover?  sigma0 is the no-search control.
"""
import argparse, collections, json, os, random, sys

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "addons", "ami_ogma", "configs")
BASE8 = ("the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose"
         "__m1auth__planpull__gainevo_live.json")
BASE3 = ("the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose"
         "__m1auth__planpull__j1s4_3d.json")

# start displaced: coupling inside its measured BAD band, the other two in theirs
TSWEEP_START = [0.385, 0.30, 1.092]
TSWEEP_ARMS = [                      # label,  sigma, sigma_min, sigma_max, target_accept
    ("sigma0", 0.00, 0.0001, 0.0001, -1.0),    # NO SEARCH — the control
    ("auto",   0.08, 0.08,   0.50,   -1.0),    # what ships today
    ("fix003", 0.03, 0.03,   0.03,   -1.0),    # sigma pinned by clamping min==max,
    ("fix008", 0.08, 0.08,   0.08,   -1.0),    # which neutralizes the anneal without
    ("fix020", 0.20, 0.20,   0.20,   -1.0),    # a code change
    ("fix045", 0.45, 0.45,   0.45,   -1.0),
]


def load(name):
    return json.load(open(os.path.join(CFG, name)), object_pairs_hook=collections.OrderedDict)


def ge_of(cfg):
    return next(m for m in cfg["modules"] if m.get("type") == "GainEvolver")


def write(cfg, path, note):
    cfg.setdefault("metadata", collections.OrderedDict())["note"] = note
    json.dump(cfg, open(path, "w"), indent=2)
    open(path, "a").write("\n")


def random_starts(keys, lo, hi, n, seed):
    """Reproducible random starts. Seeded so the same 12 points come back."""
    rng = random.Random(seed)
    return [[round(rng.uniform(lo[k], hi[k]), 4) for k in range(len(keys))] for _ in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep", choices=["basin", "basin3d", "winverify", "tsweep"])
    ap.add_argument("outdir", nargs="?", default=CFG)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    if a.sweep in ("basin", "basin3d"):
        base = load(BASE8 if a.sweep == "basin" else BASE3)
        ge = ge_of(base)["params"]
        keys, lo, hi = ge["gain_keys"], ge["gain_min"], ge["gain_max"]
        starts = random_starts(keys, lo, hi, 12, seed=20260823)
        for i, sv in enumerate(starts):
            c = json.loads(json.dumps(base))
            p = ge_of(c)["params"]
            p["gain_seed"], p["seed"] = sv, 90000 + i
            write(c, os.path.join(a.outdir, f"{a.sweep}_{i:02d}.json"),
                  f"{a.sweep} random start {i}. Convergence test: do independent starts "
                  f"reach a common vector? Vary OGMA_SEED per replicate — it is the MASTER "
                  f"override and rewrites the module's own seed param.")
        json.dump({"keys": keys, "bounds": list(map(list, zip(lo, hi))), "starts": starts},
                  open(os.path.join(a.outdir, f"{a.sweep}_spec.json"), "w"), indent=1)
        print(f"{len(starts)} configs + {a.sweep}_spec.json")

    elif a.sweep == "winverify":
        base = load(BASE3)
        starts = random_starts(ge_of(base)["params"]["gain_keys"],
                               ge_of(base)["params"]["gain_min"],
                               ge_of(base)["params"]["gain_max"], 12, seed=20260823)[:6]
        for i, sv in enumerate(starts):
            c = json.loads(json.dumps(base))
            p = ge_of(c)["params"]
            p["gain_seed"] = sv
            p["eval_window_ticks"], p["settle_ticks"] = 12000, 6000
            write(c, os.path.join(a.outdir, f"winverify_{i:02d}.json"),
                  "eval_window 12000 vs the shipped 4000. NOTE the measured answer: "
                  "acceptance did NOT move, because the margin is accept_k*sigma_hat and a "
                  "longer window shrinks both together. The noise floor depends on accept_k "
                  "alone.")
        print("6 configs")

    else:
        base = load(BASE3)
        for label, sig, smin, smax, tgt in TSWEEP_ARMS:
            for s in (1, 2, 3, 4, 5, 6):
                c = json.loads(json.dumps(base))
                p = ge_of(c)["params"]
                p["gain_seed"] = TSWEEP_START
                p["mutation_sigma"], p["target_accept"] = sig, tgt
                p["sigma_min"], p["sigma_max"] = smin, smax
                write(c, os.path.join(a.outdir, f"tsw2_{label}_s{s}.json"),
                      f"step-size sweep arm '{label}' seed {s}. Starts displaced with "
                      f"coupling at 0.30, inside its BAD band. A pass needs BOTH halves: J "
                      f"falls AND coupling re-enters 1.2-2.0. sigma0 is the no-search control "
                      f"— without it, J falling is indistinguishable from the body settling.")
        json.dump({"keys": ge_of(base)["params"]["gain_keys"],
                   "bounds": list(map(list, zip(ge_of(base)["params"]["gain_min"],
                                                ge_of(base)["params"]["gain_max"]))),
                   "start": TSWEEP_START,
                   "arms": [list(x) for x in TSWEEP_ARMS]},
                  open(os.path.join(a.outdir, "tsweep_spec.json"), "w"), indent=1)
        print(f"{len(TSWEEP_ARMS) * 6} configs + tsweep_spec.json")
    print("⚠ vary OGMA_SEED between replicates. The module's `seed` param is NOT an")
    print("  independent knob — OGMA_SEED overrides it, which silently made one sweep's")
    print("  n=3 into n=1 (2026-08-24).")


if __name__ == "__main__":
    main()
