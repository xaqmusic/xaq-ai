#!/usr/bin/env python3
"""Catastrophic-forgetting diagnosis: what does an INVERTED episode permanently destroy?

Operator observation (2026-07-26): placed among the pyramids the robot flips onto its
back, lies tucked like a dead spider, then homeokinesis breaks the lock and it flails
until it RIGHTS ITSELF — a large emergent win. But afterwards the flailing continues as
if still inverted, and the walk never recovers: movements exaggerated, phasing wrong,
traversal gone.

This measures that instead of describing it, and — the point of the tool — attributes it
PER VARIABLE. Five candidates, all verified present in code, with very different
reversibility:

  h_max          chassis_h_max — a MONOTONIC max, no decay, no reset (MotorEPM.cpp:838),
                 and it sets the height setpoint (height_k * h_max). Prime suspect:
                 inverted, the belly rangefinder reads sky, so this ratchets to a value
                 the robot can never reach upright. IRREVERSIBLE by construction.
  h_bias         the height integrator chasing that setpoint (slams to its clamp = tuck)
  amp_gain       amplitude-homeostat integrator; while tucked and barely moving, amp_ema
                 -> 0 so this winds to its rail => "movements are exaggerated"
  coord_best_fit the (1+1) search's STORED winner (decays 0.99/window, tau ~400 s, never
                 cleared on a disruption) => "phasing is wrong"
  motor_tle      self-model error — shows when the HK model has re-adapted at all

The run: walk normally, flip deterministically (OGMA_PICRAWLER_TELEPORT_FLIP=1), let it
self-right, then keep walking. AUTO-RESET ON INVERSION MUST BE OFF or the body teleports
the robot away instead of letting it recover — which is also why this failure is normally
invisible: auto-reset requires chassis_y below a height threshold, and a robot wedged up
on a pyramid is too HIGH to trigger it. That is exactly the operator's scenario.

Reports a recovery CURVE (forward progress per bin of ticks-since-righting, against the
pre-flip baseline) beside each variable's value, so "how many ticks to recover" and "which
variable is stuck" are answered together.

Usage: forgetavg.py <config.json> [n_seeds] [flip_at] [max_steps] [difficulty] [flip_xz]

Default flip_xz targets the corridor hump ridge — an ANGLED surface, which is escapable.
Flipping on flat ground is a different (harder, apparently inescapable) experiment.
"""
import json, os, statistics, subprocess, sys, concurrent.futures as cf

PROJ = "/home/xaqmusic/xaq-ai/godot_host/project"
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_forget")
os.makedirs(SP, exist_ok=True)

INVERTED_TILT = 1.57      # rad — past horizontal = on its back
UPRIGHT_TILT  = 0.60      # rad — recovered
BIN = 1200                # ticks per post-recovery bin (20 s at 60 Hz)
NBINS = 10                # longer post-righting window: a LATE recovery must not be missed

def run_one(cfg, seed, flip_at, max_steps, difficulty, flip_xz):
    out = f"{SP}/fg_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7800+seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_PICRAWLER_TELEPORT_RAMP_AT=str(flip_at),
               OGMA_PICRAWLER_TELEPORT_FLIP="1",
               OGMA_PICRAWLER_TELEPORT_XZ=flip_xz,
               OGMA_PICRAWLER_AUTO_RESET_INVERSION="0",     # let it self-right
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    with open(out, "w") as f:
        subprocess.run(["godot4","--headless","--fixed-fps","60","--quit-after","4000000",
                        "--path",".","res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out, flip_at)

VARS = ("h_max", "h_bias", "amp_gain", "coord_best_fit", "motor_tle")

def parse(path, flip_at):
    rows = []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith('{'): continue
        try: d = json.loads(line)
        except Exception: continue
        if 'x' in d and 't' in d: rows.append(d)
    if len(rows) < 20: return None

    def rate(seg):
        """Forward progress per 1000 ticks — the honest 'is it walking' measure."""
        if len(seg) < 2: return 0.0
        dt = seg[-1]['t'] - seg[0]['t']
        return ((seg[-1]['z'] - seg[0]['z']) / dt * 1000.0) if dt > 0 else 0.0
    def snap(seg, k):
        vals = [r[k] for r in seg if k in r]
        return statistics.mean(vals) if vals else float('nan')

    pre = [r for r in rows if flip_at - 2 * BIN <= r['t'] < flip_at]
    post_flip = [r for r in rows if r['t'] >= flip_at]
    # Did it actually go over, and did it come back?
    inverted = [r for r in post_flip if r.get('tilt', 0.0) > INVERTED_TILT]
    if not inverted:
        return dict(flipped=False)
    t_inv = inverted[0]['t']
    righted_t = None
    run = 0
    for r in post_flip:
        if r['t'] <= t_inv: continue
        if r.get('tilt', 9.9) < UPRIGHT_TILT:
            run += 1
            if run >= 3:                      # sustained, not a transient bounce
                righted_t = r['t']; break
        else:
            run = 0
    inv_seg = [r for r in post_flip if t_inv <= r['t'] <= (righted_t or rows[-1]['t'])]
    res = dict(flipped=True, righted=righted_t is not None,
               right_ticks=(righted_t - t_inv) if righted_t else -1,
               pre_rate=rate(pre), bins=[], pre_vars={k: snap(pre, k) for k in VARS},
               inv_vars={k: snap(inv_seg, k) for k in VARS})
    if righted_t is None:
        return res
    for b in range(NBINS):
        seg = [r for r in post_flip if righted_t + b*BIN <= r['t'] < righted_t + (b+1)*BIN]
        if len(seg) < 2: break
        res["bins"].append(dict(rate=rate(seg), **{k: snap(seg, k) for k in VARS}))
    return res

if __name__ == "__main__":
    cfg   = sys.argv[1]
    n     = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    flip  = int(sys.argv[3]) if len(sys.argv) > 3 else 4000
    steps = int(sys.argv[4]) if len(sys.argv) > 4 else 20000
    diff  = sys.argv[5] if len(sys.argv) > 5 else "0.3"
    # WHERE the flip happens decides whether the episode is escapable at all.  Operator
    # (2026-07-26): the inversions actually observed were on ANGLED / complex surfaces, and
    # those are far easier to escape than inverted on flat ground.  Measured here: flipped
    # on FLAT the robot never rights in 16 000 ticks and motor_tle FALLS 0.24 -> 0.08 —
    # inverted-on-flat is a LOW-SURPRISE attractor, so homeokinesis has no pressure to move.
    # The corridor hump (z~3) is the angled surface, and the flip hook already works there.
    flip_xz = sys.argv[6] if len(sys.argv) > 6 else "0,3.0"
    with cf.ThreadPoolExecutor(max_workers=min(4, n)) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, flip, steps, diff, flip_xz), range(1, n+1)))
    ok = [r for r in res if r and r.get("flipped")]
    print(f"\nFORGETTING DIAGNOSIS  {cfg}")
    print(f"  n={n} seeds · flip@{flip} · {steps} ticks · diff {diff} · auto-reset-on-inversion OFF")
    if not ok:
        print("  the robot never went past horizontal — flip did not take"); sys.exit(1)
    righted = [r for r in ok if r["righted"]]
    print(f"  flipped: {len(ok)}/{n}   SELF-RIGHTED: {len(righted)}/{len(ok)}"
          + (f"   in {statistics.mean(r['right_ticks'] for r in righted):.0f} ticks "
             f"(~{statistics.mean(r['right_ticks'] for r in righted)/60.0:.0f} s)" if righted else ""))
    if not righted:
        print("  never recovered upright — nothing to say about post-recovery walking"); sys.exit(0)

    base = statistics.mean(r["pre_rate"] for r in righted)
    print(f"\n  forward progress, z per 1000 ticks   PRE-FLIP BASELINE = {base:+.3f}")
    print(f"  {'window':<22}{'rate':>8}{'% of pre':>10}   " +
          "".join(f"{k:>15}" for k in VARS))
    print("  " + "-" * (40 + 15*len(VARS)))
    print(f"  {'pre-flip':<22}{base:>8.3f}{100.0:>9.0f}%   " +
          "".join(f"{statistics.mean(r['pre_vars'][k] for r in righted):>15.3f}" for k in VARS))
    inv_rate = float('nan')
    print(f"  {'INVERTED':<22}{'—':>8}{'—':>10}   " +
          "".join(f"{statistics.mean(r['inv_vars'][k] for r in righted):>15.3f}" for k in VARS))
    nb = min(len(r["bins"]) for r in righted)
    for b in range(nb):
        rate_b = statistics.mean(r["bins"][b]["rate"] for r in righted)
        pct = (rate_b / base * 100.0) if abs(base) > 1e-9 else float('nan')
        label = f"+{b*BIN}..{(b+1)*BIN} ticks"
        print(f"  {label:<22}{rate_b:>8.3f}{pct:>9.0f}%   " +
              "".join(f"{statistics.mean(r['bins'][b][k] for r in righted):>15.3f}" for k in VARS))
    print("  " + "-" * (40 + 15*len(VARS)))
    print("  Read DOWN each variable column: one that never returns to its pre-flip value")
    print("  while the rate stays depressed is the mechanism doing the forgetting.")
