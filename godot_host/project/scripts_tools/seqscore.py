#!/usr/bin/env python3
"""S0 — the chunker gate scorer (PART III twin gates).

Reads per-tick body logs (DIAG_INTERVAL=1) carrying the bp_win token stream and
the sg_* SequenceGNG mirror (seq_bodypose in event_mode), and answers the gate:

  1. BAKING — did motifs bake at all?  (`sg_b` trajectory; the RL-era chunker
     never baked one: seqgng_body.baked_count = 0.)  Contract acceptance was
     >=3 motifs; report the trajectory, not just the end.
  2. SELF-LIMITING — does the motif vocabulary plateau under its node cap
     (the support-EPM signature of real structure) or run to the ceiling?
  3. PREDICTIVE LIFT — at each EVENT (bp_win change), score the PRE-event
     motif prediction sg_pn against the event that actually arrived, vs:
       - flat first-order event chain (train first half, test second half)
       - marginal argmax (most frequent next-event overall)
     Lift = motif hit-rate / chain hit-rate on the SAME test events.
  4. CONFIDENCE — mean sg_c on scored events, and hit-rate split by
     baked (sg_bk=1) vs raw motifs — is confidence earned?

Usage: seqscore.py '/tmp/xaq_twin/twin_s*.log' [warmup=3000]
"""
import glob, json, statistics, sys
from collections import defaultdict


def analyze(path, warmup):
    rows = []
    for line in open(path):
        line = line.lstrip()
        if not line.startswith("{") or '"bp_win"' not in line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("bp_win", -1) < 0:
            continue
        rows.append((d.get("t", 0), d["bp_win"], d.get("sg_m", -1),
                     d.get("sg_pn", -1), d.get("sg_c", 0.0), d.get("sg_bk", 0),
                     d.get("sg_n", -1), d.get("sg_b", -1), d.get("sg_ev", 0)))
    if len(rows) < 4000:
        return None
    # --- event extraction: (tick, prev_tok, next_tok, pre-event sg state)
    events = []
    for i in range(1, len(rows)):
        if rows[i][1] != rows[i - 1][1]:
            t, _, _, _, _, _, _, _, _ = rows[i]
            _, prev_tok, sg_m, sg_pn, sg_c, sg_bk, _, _, _ = rows[i - 1]
            events.append((t, prev_tok, rows[i][1], sg_m, sg_pn, sg_c, sg_bk))
    ev_post = [e for e in events if e[0] >= warmup]
    # --- baking / sizing trajectory (quartiles of the run)
    n = len(rows)
    traj = [(rows[int(f * (n - 1))][6], rows[int(f * (n - 1))][7])
            for f in (0.25, 0.5, 0.75, 1.0)]
    # --- baselines on the event sequence (second half = test)
    half = len(ev_post) // 2
    train, test = ev_post[:half], ev_post[half:]
    chain = defaultdict(lambda: defaultdict(int))
    marg = defaultdict(int)
    for _, a, b, *_ in train:
        chain[a][b] += 1
        marg[b] += 1
    chain_argmax = {a: max(nb, key=nb.get) for a, nb in chain.items()}
    marg_argmax = max(marg, key=marg.get) if marg else -1
    hits_m = hits_c = hits_g = tot = 0
    conf, hits_baked, n_baked, hits_raw, n_raw = [], 0, 0, 0, 0
    for _, a, b, sg_m, sg_pn, sg_c, sg_bk in test:
        if sg_pn < 0 or a not in chain_argmax:
            continue        # score only where BOTH predictors commit
        tot += 1
        hm = 1 if sg_pn == b else 0
        hits_m += hm
        hits_c += 1 if chain_argmax[a] == b else 0
        hits_g += 1 if marg_argmax == b else 0
        conf.append(sg_c)
        if sg_bk:
            n_baked += 1; hits_baked += hm
        else:
            n_raw += 1; hits_raw += hm
    if tot < 100:
        return None
    return dict(
        events=len(ev_post), scored=tot, traj=traj,
        motif=hits_m / tot, chain=hits_c / tot, marg=hits_g / tot,
        conf=statistics.mean(conf) if conf else 0.0,
        baked_rate=(hits_baked / n_baked) if n_baked else float("nan"),
        raw_rate=(hits_raw / n_raw) if n_raw else float("nan"),
        n_baked=n_baked, n_raw=n_raw,
        final_nodes=rows[-1][6], final_baked=rows[-1][7], n_ev_mod=rows[-1][8])


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    warmup = int(argv[1]) if len(argv) > 1 else 3000
    per = [r for r in (analyze(f, warmup) for f in sorted(glob.glob(argv[0]))) if r]
    if not per:
        print("no scorable runs (need per-tick logs with bp_win + sg_* mirrors)")
        return 1
    print(f"seqscore: {len(per)} runs   "
          f"events/run {statistics.mean(r['events'] for r in per):.0f}   "
          f"module n_events {statistics.mean(r['n_ev_mod'] for r in per):.0f}")
    print(f"  nodes(q1..end): " + "  ".join(
        "/".join(f"{a}n:{b}b" for a, b in r["traj"]) for r in per))
    m = statistics.mean(per_r["motif"] for per_r in per)
    c = statistics.mean(per_r["chain"] for per_r in per)
    g = statistics.mean(per_r["marg"] for per_r in per)
    print(f"  next-event argmax on shared test events "
          f"(n={statistics.mean(r['scored'] for r in per):.0f}/run):")
    print(f"    motif {m:.3f}   event-chain {c:.3f}   marginal {g:.3f}   "
          f"LIFT motif/chain = {m / c if c else float('nan'):.2f}")
    print(f"  match_conf mean {statistics.mean(r['conf'] for r in per):.2f}   "
          f"baked-motif hit {statistics.mean(r['baked_rate'] for r in per):.3f} "
          f"(n {statistics.mean(r['n_baked'] for r in per):.0f})   "
          f"raw-motif hit {statistics.mean(r['raw_rate'] for r in per):.3f} "
          f"(n {statistics.mean(r['n_raw'] for r in per):.0f})")
    print("\nGATE S0: pass = motifs BAKE, vocabulary self-limits under the cap,")
    print("and motif lift > 1 vs the flat event chain.  A chunker that only ties")
    print("the chain adds no sequence memory beyond first-order.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
