# Kalman lessons for the EPM substrate — charter and results

*A lever campaign, 2026-09. The question is what Kalman filter theory knows that the
substrate does not, and which of those lessons survive contact with a body. Every lever is
gain-0-guarded, evaluated first on a synthetic bench where the optimal estimator is known in
closed form, then on a creature, one at a time (CLAUDE.md §3). Results are recorded inline,
stage by stage, in the repo's charter convention. The earlier design note this grew from is
[`../research-summaries/epm_kalman_filter_finding.md`](../research-summaries/epm_kalman_filter_finding.md).*

---

## 0. Why

An EPM's prototype update, its dual TLE, and the LateralVoter's `1/(tle+ε)` trust each have a
Kalman-filter counterpart, and in each case the substrate has either the right form with the
wrong schedule, a lost half, or a known failure that control theory names. The operator's
goal is a more robust substrate, accepting a precision-for-utility trade-off where robotics
demands it, and restoring what the Python → C++ port lost. Nothing here claims the EPM *is* a
Kalman filter: a Kalman filter needs a dynamics model and a noise model, and is optimal only
for a linear-Gaussian world; the EPM is what one builds without either, in a world that is
multimodal. The lessons are about the pieces that do correspond.

| Kalman element | Live C++ substrate (verified 2026-09-05) | Lesson |
|---|---|---|
| Gain on the estimate | winner update `w += g(x−w)`, `g = ε_b(1 − 0.9·visits/N)`, frozen at bake (`gng.cpp` step §3) | Same form as the Kalman filter for a constant; wrong schedule. The Kalman gain for a constant is `1/(n+1)` |
| Process noise Q | baked = frozen; drift met only by insertion / mitosis | a Kalman filter keeps a steady-state gain when the world drifts |
| Innovation, temporal half | C++ `transition_surp` = ‖proto_t − proto_{t−1}‖, a displacement (`EPM.cpp:554`); Python reference used `−log P(cur|prev)` (`python/xaq/xaq/epm.py:284`) | lost in port; a displacement cannot tell "big but expected" from "big and unexpected" |
| Precision for fusion | `1/(tle+ε)` on one tick's realized error (`LateralVoter.cpp:352`) | Kalman weights by *expected* error; per-node `ema_error` is the faithful per-mode innovation variance and is already tracked |
| Observability | baked-count informativeness gate, opt-in; picrawler stack has no voter | dead-sensor over-trust is the innovation-adaptive-Kalman failure; the fix is an observability proxy, not a gain formula |
| Uncertainty growth without a reading | sub-rate republish keeps the old TLE (`EPM.cpp:648`); voter Invariant 7 likewise | a Kalman filter inflates P by Q per unobserved step |
| RLS ≡ Kalman for parameters | `DescendingPredictor` `update_method="rls"` is SGD with a rescaled constant; unused by any config | true RLS is the one place a literal Kalman filter drops in |
| Normalised innovation | Python EPM: `1/(1 + tle/running_avg_tle)`; C++ publishes raw `tle` only | lost in port; it is the standard filter-consistency statistic |
| Placeholder trust | C++ voter drops `winner_id < 0` (`LateralVoter.cpp:250`); Python reference did not | the research note's finding is Python-only |

**Testbeds.** The picrawler stack (`…m1auth__planpull__j1s4_c1.json`) carries two `EPM`s whose
`winner_id` alone feeds `MotorPlanner`: GNG levers are observable there, trust levers are dead
code. The Cell maze fusion (`the_cell_maze_fusion*.json`) carries two EPMs, the voter with the
informativeness gate, and a mid-run vision lesion: that is the voter levers' testbed. Ten
legacy picrawler configs set `epsilon_b: 0.0`; gain levers are `DEAD_CODE` there.

**Staging (operator decision).** Bench + first lever, then its A/B, before the next lever is
coded. Drift tracking of baked nodes is in scope as a default-off option.

---

## Stage 0 — instruments and harness (DONE 2026-09-05, no behaviour change)

### What was built

- **Bench** `cpp_core/bench/epm_kalman_bench.cpp` (target `epm_kalman_bench`): drives the real
  `EPM` / `DescendingPredictor` / `LateralVoter` on an `InProcessBus` with an identity encoder,
  one JSON line per tick. Six scenarios, each with an exact reference computed by
  `tools/epm_bench/analyze.py`; `tools/epm_bench/run.sh <scenario> --seeds N [--arm m.k=v]`
  sweeps seeds and prints the table; `analyze.py --compare <base_dir> <arm_dir>` pairs seeds
  and gives Student-t 95 % CIs.
- **Pin test** `GNG.LinearAnnealSeedWeightAtBake` in `cpp_core/tests/v3/test_gng.cpp`: the
  seed's residual weight at bake equals ∏(1−g_n) = 0.241 (ε_b 0.05, N 50). It proves the
  path Stage 1 changes is live and measured.
- **Instruments** on `EPM::diag_lite()`: `tle_norm = tle/ema_tle` (the normalised innovation
  the Python reference folded into serotonin) and `qe_lag1` (lag-1 autocorrelation of
  `quant_error`, innovation whiteness). Diag-only, not serialised.
- **Harness repairs**: `mkarm.py --module=<id>` targets any module by config id;
  `test_descending_predictor` no longer aborts on `init_noise_scale = 0`; the Python reference
  voter now drops placeholder tokens as the C++ voter always did.

### Three traps surfaced while building it

1. **`baking_threshold` advertised 50, runs 100 when omitted.** The runtime passes config
   params verbatim and never merges schema defaults. 105 EPM instances across 40 configs omit
   the key. Noted in `EPM.md`; the bench pins 50 explicitly.
2. **The predictor's residual-mode update is one tick misaligned.** Feedback delivers
   `reality(t−1)` before `tick(t)`; that residual measures `prediction(t−2)`, but the update
   pairs it with the context cached at `t−1`. Harmless for a slow context (the clock), costly
   for fast dynamics. Owned by Stage 3 (K6) together with true RLS.
3. **A constant-velocity target is unusable as a bench stream**: its position is unbounded
   and the predictor's SGD diverges as the context norm grows. S3 is a damped rotation.

### Baselines, n = 20 seeds, unmodified substrate (documented defaults, bake 50, σ = 0.1)

| Scenario | What the reference says | What the substrate does |
|---|---|---|
| S1 static cluster | seed weight at bake 0.241 (analytic); MSE of the Kalman estimate on the same samples 0.00041 | **seed weight 0.275 ± 0.069**; prototype MSE **0.00084 ± 0.00036**, i.e. **2.0×** the optimum on the same data. The two bootstrap nodes split the cluster's wins, so both estimates carry a selection bias toward their seeds |
| S1m 3 clusters | 3 means | purity 1.00; **7.4 baked nodes** for 3 clusters (over-tiling at this noise), prototype MSE to cluster mean 0.010 |
| S2 drifting mean | Kalman tracking MSE 0.00095 | **0.0059 ± 0.0003, 6.2×**; the drift is tracked by a chain of **18.7** nodes, not by moving one |
| S3 damped rotation, closed pair | Kalman innovation variance 0.0166; persistence 0.0232 | residual² **0.0194 ± 0.0007** (1.17× the floor); the linear pair works, the residual GNG bakes 9.9 nodes |
| S4 cycle + teleports | a surprise should separate expected from unexpected transitions | displacement `transition_surp`: expected 1.29, unexpected 1.34, **ratio 1.03 ± 0.10 — blind** |
| S5 two sensors, R_b = 9 R_a | optimal weight on a 0.90; optimal fused MSE 0.0090; best single 0.0100 | trust on a **0.67**; fused MSE **0.0133 — worse than the better sensor alone**; tick-to-tick trust std 0.054; sensor b grows **157 nodes** (noise above the gate, unbounded insertion) |
| S5 + dead sensor at 2000 | a dead sensor carries no information: weight → 0 | **trust on the dead sensor 0.66**, never stripped; fused MSE after death **0.044** (4.4× the live sensor) |
| S5 + placeholder tick | that tick's weight → 0 | **0.00** (C++ filter works; Python reference now matches) |
| S5 + sensor b at 1/4 rate | stale readings should count less | trust unchanged (0.68); the stale token keeps its old TLE |
| S5 + informativeness gate 1.0 (the Cell config) | weight on a 0.90 | **trust on a 0.48 — worse than legacy**: the noisy sensor bakes more nodes (15 vs 2) and is read as *more* informative. Dead arm: **0.80** on the dead sensor |

The last row is the sharpest Stage-0 result: the baked-count informativeness proxy is fooled by
a channel that tiles its own noise. It works on the Cell because an occluded camera bakes one
degenerate node; a noisy channel is the opposite case, and the gate rewards it. That is the
Stage 2 target.

Reference outputs for the gain-0 byte-identity check are frozen under the session scratchpad
(`epm_bench/ref_stage0/`); Stage 1 re-runs the same seeds at default params and diffs.

---

## Stage 1 — per-node Kalman gain (bench WORKING; picrawler: uncapped `REGRESSION`, cap 0.05 `PARTIAL`, 2026-09-05)

**Mechanism** (`cpp_core/include/v3/gng.hpp`, `cpp_core/src/v3/gng.cpp` step §3):
`gain_kind ∈ {linear (default, byte-identical), kalman}`, `kalman_p0` (1.0), `kalman_q` (0.0),
`kalman_gain_cap` (1.0). Per node a scalar `p`, the prior-variance ratio in units of the node's
own observation noise, so the schedule is dimensionless (doctrine §6). Per win:
`p += q; K = min(cap, p/(p+1)); w += K(x−w); p *= (1−K)`. With p0 = 1, q = 0 that is exactly
`1/(n+1)`: the seed counts as one sample. With q > 0 the gain settles at the random-walk steady
state and baked nodes keep moving by it; with q = 0 baked stays frozen. Visit/health damping is
not applied in kalman mode; the neighbour pull `ε_n` is untouched. `p` is serialised only when
the mode is on.

**Predictions (bench, n = 20, paired by seed).** S1 seed weight 0.275 → ≈ 0.02; prototype MSE
ratio to the same-data optimum 2.0 → ≈ 1; S1m purity and node count must not degrade (the named
risk is a high early gain dragging a young node across a cluster boundary; `kalman_gain_cap`
is the knob); S2 with q > 0: tracking MSE toward 0.00095 and node growth stops.

**Tier B.** Picrawler stack arm on `body_pose` / `body_pose_t` via `mkarm.py --module`,
`seedavg.py` n = 6 as the promote-or-kill signal on the full metric set plus baked fraction,
node count and self-transition mass; operator's eye before promotion. Cell fusion on
`epm_scent` / `epm_vision`. `kalman_q > 0` is a separate later lever.

### Tier A results (bench, n = 20 paired seeds, 2026-09-05)

Shipped: `gain_kind` / `kalman_p0` / `kalman_q` / `kalman_gain_cap` on the GNG and the EPM;
three unit tests (`KalmanGainIsTheFilterForAConstantAndFreezesAtBake`,
`KalmanGainWithProcessNoiseTracksAfterBake`, `KalmanStateSerialisation`). **Gain-0 guard
measured:** all six Stage-0 baselines re-run at default params are byte-identical to the frozen
reference, 120/120 seed files.

| Scenario, arm | Prediction | Measured (arm − base, 95 % CI) | Verdict |
|---|---|---|---|
| S1, `gain_kind=kalman` | seed weight 0.275 → ≈ 0.02 | **0.034** (−0.242 ± 0.035); prototype MSE **÷1.6** (0.00084 → 0.00052 ± 0.00021), now at or below the same-data sample mean (0.00056) | prediction confirmed |
| S1m, kalman | purity and node count must not degrade | purity 1.00 → 1.00; nodes **10.7 → 7.05** (± 1.2), baked 7.4 → 4.95, prototype MSE ÷1.4 | the feared boundary-dragging did not occur; the vocabulary tightened |
| S2, kalman, q = 0 | small | tracking MSE ÷1.15 (± 0.0002); nodes 18.7 → 17.3 | as expected: frozen baked nodes still chain |
| S2, kalman, **q = 0.01** | tracking MSE toward the Kalman 0.00095; node growth stops | tracking MSE **0.0059 → 0.00101** (÷5.9), within 6 % of the Kalman optimum; nodes **18.7 → 2** | **loud**: drift is tracked by moving, not by spawning |
| S4, kalman | no change in surprise metrics | surprise ratio unchanged (1.03 → 1.03); nodes 11.7 → 7.75 for 3 states | orthogonal to K2, as designed |
| S5, kalman | small | fused MSE ÷1.02 (± 0.005), trust on a 0.672 → 0.682; sensor b still 159 nodes | the fusion problem is the gate and the trust rule, not the gain |

The bench verdict for `gain_kind=kalman` (q = 0) is **WORKING** in every scenario, with no
regression anywhere. `kalman_q > 0` is loud on S2 and goes to its own A/B.

### Tier B (creature) — IN FLIGHT

Arms written with `mkarm.py --module`: picrawler
`…planpull__j1s4__kgA__kalman.json` (`body_pose`, `body_pose_t`) and Cell
`the_cell_maze_fusion__kgA__kalman.json` (`epm_scent`, `epm_vision`). The picrawler run is
`seedavg.py` n = 6, corridor, 12 000 ticks, difficulty 0.3, against `…__j1s4.json`. The Cell
run waits on the recovered harness.

**Arm 1, uncapped (`kalman_gain_cap` 1.0), n = 6 paired seeds — a stability `REGRESSION`
signal.** Walkers 6/6 both arms. Speed and progress tie (flat_v 0.05 / 0.05, max_z 7.42 /
7.73), time-to-flat improves (2416 → 2127), scrub improves (0.092 → 0.087), but **falls 1 → 6**
(one seed 4), tilt_sd 0.104 → 0.279, planted 3.65 → 3.55, and one seed collapsed to net_z 0.07
after reaching 7.5. §3.2 checks: the arm loaded (paired seeds show a different winner-id set
on every seed, mean distinct ids 45.8 → 53.0, so the consumer, `MotorPlanner`, fired on a
changed vocabulary); the baseline is healthy; no tautology. Note the direction on the body is
the opposite of the bench: the vocabulary got *larger*, not tighter. The stream is a
non-stationary gait ring rather than static clusters, and a node's first win at gain 0.5
lets young nodes chase the input. That is the risk the bench's S1m did not express and the
cap is the documented knob for it. One seed touched the far wall in both arms (the harness's
gym-boundary warning), which affects the distance ranking only. The extra ids are **turnover,
not size**: distinct winners per run-quarter match the base (21–27 in both arms) while the
whole-run count rises (46 → 53), so the uncapped gain raises node churn (insert, chase, prune,
re-insert) rather than tiling more finely.

**What the cap costs on the bench** (n = 20 paired, kalman vs base): cap 0.2 keeps everything
(S1 seed weight 0.032, MSE ÷1.6; S1m nodes 10.7 → 7.25; S2 ÷1.13). Cap 0.05, i.e. never above
the legacy starting gain, keeps the S1 gain (seed weight 0.108, MSE **÷1.8**) but most of the
S1m tightening goes (nodes 10.7 → 9.45).

**Arm 2, cap 0.05 (never above the legacy starting gain), n = 6 paired seeds — `PARTIAL`
signal.** Walkers 6/6, **falls 0** (base 1, uncapped 6), vocabulary turnover back at the base
level (43 whole-run ids vs 46). Paired deltas (arm − base, 95 % CI, t on 5 df):

| metric | base | uncapped | cap 0.05 | paired Δ cap 0.05 − base |
|---|---|---|---|---|
| flat_v | 0.047 | 0.055 | **0.057** | +0.010 ± 0.009, t 2.7 |
| t_flat | 2416 | 2127 | **2000** | −416 ± 507, t −2.1 |
| brt_err (body-rhythm prediction error) | 1.545 | 1.557 | **1.438** | −0.107 ± 0.084, t −3.3 |
| net_z | 7.39 | 6.44 | 8.52 | +1.14 ± 2.20, t 1.3 |
| straight | 0.62 | 0.50 | 0.66 | +0.045 ± 0.083, t 1.4 |
| planted | 3.65 | 3.55 | 3.41 | −0.24 ± 0.27, t −2.3 |
| unstable | 0.097 | 0.140 | 0.202 | +0.105 ± 0.109, t 2.5 |
| tilt_sd | 0.104 | 0.279 | 0.123 | +0.019 ± 0.051 |
| falls (total) | 1 | 6 | 0 | |

Faster to flat and faster on the flat, with a cleaner body rhythm, at the cost of a doubled
unstable-tick fraction and fewer planted feet, while progress and straightness only trend up.
`unstable` is the fraction of ticks with fewer than three feet planted (`seedavg.py:220`),
which a faster gait raises by construction, so it is not a clean stability read on its own;
falls (0) and tilt_sd (flat) are the honest ones here.
One seed reached the far wall (9.98), so net_z is if anything under-reported for the arm.
This is a signal, not a finding: n ≥ 20 varied seeds, the arena gym for distance, and the
operator's eye are the promotion bar.

**Arm 3, cap 0.2, n = 6 paired seeds — `REGRESSION`, the same signature as uncapped.** Falls
5 (one seed 4), tilt_sd 0.339 (+0.235 ± 0.289), one seed collapsed to net_z 1.96, straight
0.50, whole-run winner ids 53 (the uncapped turnover, not the base's 46 or cap 0.05's 43).

### Cell maze fusion, cap 0.05 — `DEAD_CODE`, and a live instance of the zero-encode trap (2026-09-05)

`cell_coverage.py --vary-world`, n = 20 paired worlds, 240 s: **every per-seed number is
identical between base and arm** (eats 13.9 / 13.9, crossings 10.6 / 10.6, mean food distance
6.5 / 6.5 m). §3.2 diagnosis: the arm loaded (the patched config carries `gain_kind=kalman`
on both EPMs), but the EPMs are dead. Their diagnostics read nodes 2, baked 1, `tle` 0.0, one
winner for 3658 of 3660 ticks and a latent of 32 zeros. `ScentCompass` and `VisualBearing`
emit **three** values with `emit_proximity: true` (`ScentCompass.cpp:227`,
`VisualBearing.cpp:316`); the six maze configs declare `proprio_state_dims: 2`; the RBF
encoder returns a zero vector on the mismatch (`encoder_rbf.cpp:150`). Twelve EPMs across
`the_cell_maze_fusion`, `_dropvision`, `_dropscent`, `_scent_only`, `_vision_only` and the
Kalman arm are fed zeros. The voter's trust is therefore a constant 0.5 / 0.5 and
`BearingFusion` blends at 50/50; the vision-dropout recovery in the Cell report ran through
BearingFusion's own `confidence_floor`, not the EPM → voter path the config's own description
credits. The ami-ogma clone's config carries the same mismatch from its first navigator
commit, so this predates the report. **The Cell A/B for any EPM lever needs a repaired
config, and the repair itself must be measured against the broken baseline.** Whether the
report gets a correction is the operator's call (REPORTS.md).

**Repair:** `the_cell_maze_fusion__dims3.json` = the fusion config with `proprio_state_dims: 3`
and a `[0, 1]` range on the proximity dim, nothing else changed. A 60 s diagnostic run shows
the EPMs alive (scent 16 nodes / 13 baked, `tle` 0.24; vision 7 / 5, `tle` 0.0 with one
winner for 3103 of 3660 ticks, i.e. occluded most of the run) and the voter's trust at
**0.20 scent / 0.80 vision**. That split is the Stage-0 S5 trap on the creature: a channel that
is blind most of the time is trivially predictable, bakes enough distinct nodes to pass the
informativeness gate, and is then the *most* trusted. Stage 2's levers now have a live
consumer and a measured baseline to move.

**Repaired A/B, n = 20 paired worlds, 240 s (2026-09-05):**

| arm | eats | crossings | mean food dist (m) | paired vs its control |
|---|---|---|---|---|
| broken config (EPMs dead, trust 0.5/0.5) | 13.90 | 10.6 | 6.51 | |
| repaired config, `linear` | 13.60 | 10.3 | 6.54 | vs broken: eats −0.30 ± 0.96 (t −0.65), food dist +0.03 ± 0.52 — **a tie** |
| repaired config, `kalman` cap 0.05 | 14.55 | 10.9 | 6.75 | vs repaired: eats +0.95 ± 1.53 (t 1.3, 10+/6−), food dist +0.21 ± 0.57 (t 0.8) — **`NULL`** |

Two readings. First, the Kalman gain on the Cell's two nav EPMs is `NULL` at this power: a
trend on eats inside its interval, and no movement on approach. Second, and the reason
that null is weak evidence about the lever: **the repair itself ties the broken config.**
Bringing the EPMs to life and turning a constant 50/50 into a real 0.2/0.8 precision split
did not change foraging, so on an intact forager the EPM → voter → `BearingFusion` path is
a weak consumer of anything the EPMs do; steering is carried by the compass modules'
own confidences and the reflexes. That is doctrine §2.3's rule in the Cell's own numbers:
precision-weighting buys nothing on a healthy system, measure it under damage. **Re-use
context for every EPM lever on the Cell, this one and Stage 2's: measure under the vision
dropout (`cell_perturbation_d.py`, `_dropvision`), where trust decides who steers, not on
the intact forage.** The Cell report's behavioural results stand; its stated mechanism for
the fusion (EPM bakes a degenerate node → trust → 0) never operated, and the config
descriptions that repeat it are for the operator to correct.

### Cell study environment: the report's two findings reproduced, and the lever against them (2026-09-05)

`the_cell_arbiter_room_pillars_vision.json` (the leave-one-out environment; EPMs alive, dims
match), `cell_coverage.py --vary-world`, n = 20 paired worlds, 240 s; the specialist is
`the_cell_chemotaxis_baseline.json` in the same worlds. Arms: `full` = `vision_weight 1.0`
(all four loops live), `noplay` = `play_weight 0`, each with and without
`EPM.gain_kind=kalman, kalman_gain_cap 0.05` on both EPMs (place and vision).

| arm | eats | food dist (m) | paired vs specialist (eats) | paired vs own control (eats / food dist) |
|---|---|---|---|---|
| specialist (reactive run-and-tumble) | 1.95 | 19.1 | | |
| full composition | 0.60 | 21.9 | **−1.35 ± 0.49, t −5.8** (report: −0.95, t −3.6) | |
| full + kalman | 0.65 | 22.2 | −1.30 ± 0.63 | +0.05 ± 0.42 / +0.27 ± 0.85 — `NULL` |
| composition minus play | 2.00 | 19.5 | **+0.05 ± 0.54, a tie** (report: tie) | |
| minus play + kalman | 1.85 | 18.3 | −0.10 ± 0.59, a tie | −0.15 ± 0.55 / **−1.27 ± 1.78** (13 of 20 worlds closer to food; t −1.5) — `NULL` |

Both findings reproduce with the recovered harness: the full composition loses to the
specialist and the composition without play ties it. The per-node Kalman gain changes
neither. That is the informative part: the composition's shortfall is the arbitration (play
crowding out the pragmatic loops, the report's §4), not the quality of the place or vision
vocabulary, so a better estimator in the EPM has nothing to buy there. The only movement is a
trend toward closer food approach without play (−1.27 m, inside its interval), the direction
a cleaner place vocabulary would push the planner. Re-use context: a regime that *demands* a
map (the report's §9), or the vision-dropout perturbation, where the vocabulary's quality is
load-bearing.

### Picrawler, cap 0.05, the operator's three observations measured (2026-09-05)

The operator's eye in the UI (arena, obstacles): good adaptive behaviour without getting
stuck, a flat-ground gait that "is not good", and faster re-adaptation after a belly-up.
Three harnesses, base vs cap 0.05, n = 6 paired seeds, all defaults.

**Arena (`arenaavg.py`, 6000 ticks, the gym where heading is not corrected by walls):**

| metric | base | cap 0.05 | paired Δ, 95 % CI, t |
|---|---|---|---|
| net_disp / straight / turns | 6.24 / 0.75 / +0.05 | 7.20 / 0.79 / −0.01 | +0.97 ± 2.2 / +0.04 ± 0.11 / −0.06 ± 0.11, trends |
| falls / tilt_sd / planted / unstable | 0 / 0.072 / 3.89 / 0.012 | 0 / 0.066 / 3.92 / 0.000 | no regression; unstable 0+/3− |
| steps (leg-lift events) | 54.3 | 45.3 | **−9.0 ± 8.6, t −2.7, fewer in 6 of 6** |
| step_cv_real (step-clock regularity) | 0.763 | 0.730 | **−0.033 ± 0.039, t −2.2** |
| contact_duty / short_bouts | 0.760 / 0.26 | 0.767 / 0.24 | +0.007 ± 0.009 / −0.02 |
| scrub / belly clearance | 0.096 / 0.037 | 0.103 / 0.038 | +0.007 ± 0.011 / +0.002 ± 0.003 |

On the open floor the corridor's doubled unstable-tick fraction is absent, falls and tilt are
flat, and the gait takes **fewer, more regular steps for the same or more distance**: longer
strides, fewer aborted lifts. Scrub is a hair higher. Nothing here says "worse flat gait";
whatever the operator's eye is reacting to is not in these metrics, and the next step is to
name it (asymmetry, drag, a specific leg) so an instrument can be pointed at it.

**Recovery gate (`recoveravg.py`, 12 000 ticks, teleported onto the hump crest every 2400):**
how far the gait-phase coordination wanders from its start (RMS rad), where it settles, the
fraction it returns, and progress in the last third after every perturbation has hit.

| metric | base | cap 0.05 | paired Δ, 95 % CI, t |
|---|---|---|---|
| gp_peak (max wander) | 1.19 | 0.96 | −0.24 ± 0.51, t −1.2, less in 4 of 6 |
| gp_final (where it settled) | 0.86 | 0.69 | −0.18 ± 0.49 |
| gp_recovered (fraction returned) | 0.29 | 0.30 | +0.01 ± 0.18, tie |
| late_progress (last third) | 0.39 | 0.89 | +0.50 ± 1.06, t 1.2 |
| net_z / falls | 8.69 / 0 | 8.49 / 0 | tie / tie |

Under repeated perturbation the coordination wanders less and settles nearer its start, and
progress after the perturbations trends higher; nothing regresses. That is the direction the
operator's "re-adapts faster after belly-up" observation points, and at n = 6 it is a trend,
not a finding.

**Hump gate (`humpavg.py`, 9000 ticks, teleported onto the crest at 3000; the
obstacle-regression gate every flat-speed lever must clear):** both arms clear it in 6 of 6
seeds. final_z 7.33 vs 7.29 (Δ −0.04 ± 0.77, a tie; the hump base is z = 4), gain after the
teleport 4.73 vs 4.69, belly clearance 0.034 vs 0.034, falls 0 vs 0. **Passed, as a tie**, which
is what the mechanism predicts: obstacle handling runs through the belly rangefinder and the
contact reflexes, upstream of the pose vocabulary.

**The three observations, scored.** Adaptation after a perturbation: direction confirmed
(less wander, more late progress), trend at n = 6. Obstacle handling: unchanged, gate passed.
Flat gait: no metric degrades on the open floor; the gait takes fewer, more regular, longer
steps. The operator's "not good" needs a named feature before an instrument can be pointed
at it. Cap 0.05 now carries: corridor `PARTIAL` (faster, cleaner rhythm, zero falls), arena
no regression, recovery trend in its favour, hump tie. Still one lever at n = 6; the
promotion bar (n ≥ 20 varied seeds, operator's eye) stands.

### The promotion run: arena, n = 20 paired seeds, 12 000 ticks (2026-09-05)

| metric | base | cap 0.05 | paired Δ, 95 % CI, t, sign |
|---|---|---|---|
| net_disp | 10.70 | 10.35 | −0.35 ± 0.51, t −1.4, 7+/13− |
| straight | 0.712 | 0.721 | +0.009 ± 0.026 |
| turns (net) | −0.069 | −0.011 | **+0.059 ± 0.042, t 2.9, 15+/5−** (the base drifts left; the arm does not) |
| falls (total) | 1 | 2 | tie |
| tilt_sd / planted / unstable / scrub / belly | 0.079 / 3.90 / 0.020 / 0.096 / 0.034 | 0.080 / 3.91 / 0.010 / 0.096 / 0.035 | ties |
| steps / step_cv_real | 99.1 / 0.816 | 86.6 / 0.810 | −12 ± 14 / tie |

**Verdict at power: `PARTIAL`, not promoted.** No regression on any metric, but the corridor's
flat-speed gain does not carry to the open floor over 12 000 ticks: progress is a slight
negative trend inside its interval, and the one resolvable change is heading neutrality. By
§3.3 a lever that needs averaging to find is not a capability; `gain_kind=kalman` with
`kalman_gain_cap 0.05` stays a shipped, default-off option (bench WORKING, body harmless,
recovery trend in its favour) and does not enter `j1s4`. Re-use context: a body whose
vocabulary is stationary enough for the uncapped schedule, and any regime where the
recovery-gate trend (less coordination wander after perturbation) is load-bearing.

### Stage 1 verdict (2026-09-05)

| Arm | Bench (n = 20) | Picrawler, n = 6 corridor 12 k | Verdict |
|---|---|---|---|
| `gain_kind=kalman`, cap 1.0 | WORKING everywhere | falls 1 → 6, tilt ×2.7, one collapse; node turnover 46 → 53 | `REGRESSION` on the body |
| cap 0.2 | WORKING (identical to uncapped) | falls 5, tilt ×3.3, one collapse; turnover 53 | `REGRESSION` on the body |
| **cap 0.05** | S1 MSE ÷1.8, seed weight 0.11; S1m tightening mostly gone | falls 0, flat_v +21 % (t 2.7), t_flat −17 %, brt_err −7 % (t −3.3); net_z +15 % and straight +7 % trend; turnover 43 | **`PARTIAL`, promote-or-kill: promote to n ≥ 20 + arena + operator's eye** |

**The mechanism-in-context reading.** The Kalman schedule for a constant has a head and a
tail. Its *tail*, `1/(n+1)` instead of the legacy decay to `0.1·ε_b`, is where the estimate
improves, and the body tolerates it. Its *head*, a gain of 0.5 on a node's first win, is
right for a static cluster and wrong for a gait: on a non-stationary ring of poses a young
node chases the input, the region it left gets re-inserted, the vocabulary churns, and the
planner that reads that vocabulary loses the walk. The dose response (1.0 and 0.2 fall, 0.05
does not) locates the harm in the head. Doctrine §0.5's through-line holds one level down:
the theory-optimal schedule for a *stationary* world, imposed on a moving one, fights.

**Bench gap recorded, then tested.** None of S1–S5 expressed this: they are static clusters
or a slow drift. `S6` was added (a ring of 8 poses visited in order, 20-tick dwell, optionally
rotating at 3e-4 rad/tick; `turnover` = whole-run distinct winners over the per-quarter count)
and run at n = 20 under all four gain settings. **It does not reproduce the churn**: the Kalman
gain *lowers* turnover on the bench (stationary ring 1.59 → 1.37 uncapped, 1.30 at cap 0.05;
rotating ring 1.72 → 1.63 / 1.58) and lowers node count, at unchanged purity; the only hint of
harm is a wider quantisation error on the stationary ring under the uncapped and 0.2 gains
(0.0153 → 0.021 / 0.027, CI touching zero). What the body has that no bench stream has is a
**closed loop**: `body_pose`'s vocabulary feeds `MotorPlanner`, which shapes the poses the
EPM reads next, so a node that chases changes its own input. An open-loop bench cannot
express that; the gap is real and stays open, and the creature run remains the instrument
for it.

**Re-use contexts.** Uncapped / cap 0.2: a stationary vocabulary (a Level-N EPM over a
settled consensus stream, or a place map) is where the head is right; retry there. Cap
0.05 on the picrawler: the promotion run above. `kalman_q > 0` (drift tracking of baked
nodes): its own lever, on the bench loud (S2, ÷5.9); on a body only with the cap.

---

## Stage 2 — precision levers on the voter (Cell fusion testbed)

Order decided 2026-09-05 (operator: proceed as I see fit): **K8 activity term first**, because
the failure both the bench and the repaired Cell config actually show is a blind or dead
channel being the most trusted; K3 expected-error trust second (it addresses the noisy-versus-
clean case the informativeness gate gets backwards); K4 stale inflation waits for the slow loop.
Creature test = the vision dropout on the repaired fusion config, not the intact forage.

### K8 — `activity_gain` (SHIPPED 2026-09-05; bench WORKING)

Per channel the voter keeps an EMA (`activity_alpha` 0.1) of the latent's tick-to-tick
displacement, normalised by its own decaying peak (`activity_peak_decay` 0.999) so the factor is
dimensionless and self-calibrating in [0, 1]; raw trust is multiplied by
`activity^activity_gain`. A channel that stops moving loses trust in EMA time instead of
becoming the most trusted because it is trivially predictable. A republished sub-rate token has
zero displacement, so a stale channel decays the same way (K4's intent, for free). State is
serialised only when the gain is non-zero. Default 0 is byte-identical: 40 of 40 S5 reference
files diff empty. Two unit tests.

| S5 arm (n = 20 paired) | legacy | + `activity_gain` 1 | Δ, 95 % CI |
|---|---|---|---|
| intact pair, trust on the clean sensor | 0.672 | 0.671 | −0.0006 ± 0.0009, unchanged (both channels move; the term is not a noise discriminator) |
| dead sensor at 2000: trust on it afterwards | **0.659** | **0.0022** | −0.656 ± 0.017 |
| dead sensor: fused MSE after death (best single 0.0100) | 0.0443 | **0.0099** | ÷4.4, fusion falls back to the live sensor's floor |
| dead sensor + informativeness gate: trust on it | 0.802 | 0.0048 | −0.797 ± 0.014; fused MSE 0.064 → 0.0099 |
| sensor b at 1/4 rate: fused MSE | 0.01304 | **0.01152** | ÷1.13; trust on the fresh sensor 0.676 → 0.725 |
| wrong sign (gain −1), dead sensor | 0.659 | **0.9994** | fused MSE after death ×2.26 — the sign is the whole effect |

Bench verdict `WORKING` on the failure it targets, neutral where it has no business, and the
wrong-sign arm inverts it.

**Creature test: a stuck camera, not a dropout.** Two things ruled out the planned vision
dropout. The recovered `cell_perturbation_d.py` varies only a PlayLoop seed in a fixed world,
so on the fusion config (no PlayLoop) its twenty runs were one world, and its lesion and
control arms came out identical. And a lesion zeroes the bearing, which `BearingFusion`'s
`confidence_floor` already drops regardless of trust, so a dropout cannot express a trust
lever at all. The failure the activity term targets is a channel that reports a **constant,
plausible** value. `VisualBearing.stick_after_ticks` (default −1, byte-identical) republishes
the last bearing seen with food in view, unchanged, from that tick on (latching the first
sighting after it if there was none before). Sanity run on `the_cell_maze_fusion__dims3.json`,
stick at 1800: the vision EPM's input becomes a new constant (it tiles it down to error
0.005, distinct from its blind state); legacy trust on the frozen channel climbs to **0.84**
by tick 3540, and under `activity_gain 1` it sits at **0.03**, having been below 0.2 all run
because the camera is blind most of the time. `cell_coverage.py` now reports eats and food
distance per run-third so the perturbation reads as pre / mid / post.

**A/B (2026-09-05): `the_cell_maze_fusion__dims3.json`, 20 paired worlds, 240 s, camera stuck
at 4800 (the start of the middle third).**

| arm | eats, total | eats by third (pre / mid / post) | food distance by third (m) |
|---|---|---|---|
| base | 13.6 | 5.55 / 4.10 / 3.95 | 8.28 / 5.64 / 5.70 |
| stuck camera, legacy trust | **7.7** | 5.55 / **1.50 / 0.65** | 8.28 / **10.23 / 10.23** |
| activity term, no perturbation | 13.85 | 5.55 / 4.45 / 3.85 | 7.99 / 5.82 / 5.41 |
| stuck camera + activity term | **12.3** | 5.55 / **3.55 / 3.20** | 7.99 / **5.51 / 5.01** |
| stuck camera + wrong sign (gain −1) | 6.0 | 5.35 / 0.35 / 0.30 | 7.94 / 12.61 / 12.12 |

Paired by world (95 % CI, t on 19 df):

| comparison | eats | food distance |
|---|---|---|
| stuck − base (the perturbation's cost) | −5.90 ± 1.97, t −6.3, 17 of 20 | +3.05 ± 0.98, t 6.5, 20 of 20 |
| **act_stuck − stuck (the lever's claim)** | **+4.60 ± 2.09, t 4.6, 16 of 20** | **−3.42 ± 0.86, t −8.3, 19 of 20** |
| act_stuck − base (what is left of the damage) | −1.30 ± 1.63, t −1.7 | +0.37 ± 0.60, tie |
| act − base (no-perturbation control) | +0.25 ± 0.95, tie | −0.14 ± 0.52, tie |
| wrong_stuck − stuck (sign control) | −1.70 ± 0.92, t −3.9 | +1.31 ± 0.82, t 3.3 |

The pre-stick thirds are identical across arms (5.55 / 8.28), the built-in confound check. A
camera frozen on a plausible bearing takes 43 % of the forager's eats under legacy trust,
because a constant channel is the most predictable one in the room and `1/(err+ε)` hands it
authority. With the activity term the frozen channel loses that authority in EMA time, scent
steers, and 78 % of the lost eats come back; the residual against the intact base is inside
its interval. The term costs nothing when nothing is wrong, and the wrong-sign arm is worse
than doing nothing. **Verdict: `WORKING`, at n = 20 paired worlds, under a (d)-class
perturbation, with an intact control and a sign control.** Re-use context: any fusion where a
channel can freeze or go stale; the picrawler stack has no voter today, so nothing to carry
there. Not a default: the operator's eye and the launcher decide promotion.

### K3 — `trust_source` / `trust_power` (SHIPPED 2026-09-05; bench WORKING)

The bench said the intact-pair bias (0.67 on the clean sensor against the optimal 0.90) is the
*form*, not the timing: a Kalman fusion weights by inverse variance, and `1/(err+ε)`
compresses a 9:1 precision ratio to about 2:1. Two knobs, both default to the legacy
expression bit for bit: `trust_source ∈ {default, tle, quant_error, expected}` and
`trust_power` (precision `= 1/(err+ε)^power`; 1 = legacy, 2 = inverse variance, −1 = the
sign control). `expected` is a new `RealityToken::expected_error` the EPM fills.

**A refutation on the way.** `expected_error` was first the winner node's own running RMS
residual, the per-mode innovation spread. On S5 it barely moved trust (0.672 → 0.688) and
doubled its flicker: a noisy channel tiles its noise finely, so every node's own residual is
small and the per-node value cannot tell noisy from clean. The channel's running expected TLE
(the EPM's `ema_tle`) is the right quantity and is what ships.

| S5 arm (n = 20 paired) | legacy | `expected`, power 2 | Δ, 95 % CI |
|---|---|---|---|
| intact pair: trust on the clean sensor (optimum 0.90) | 0.672 | **0.812** | +0.140 ± 0.001 |
| intact pair: fused MSE (optimum 0.0090, best single 0.0100) | 0.01329 | **0.00976** | ÷1.36 |
| intact pair: tick-to-tick trust std | 0.054 | **0.013** | ÷4.3 (instantaneous `tle` at power 2 gives 0.799 / 0.00938 with std 0.079) |
| dead sensor alone: trust on it after death | 0.659 | 0.786 | worse, as predicted: a dead channel's expected error → 0 |
| dead sensor + activity term: trust on it / fused MSE after death | 0.002 / 0.0099 | 0.005 / 0.0099 | the two levers compose: intact gain kept, dead sensor stripped |
| under the informativeness gate (the Cell config): trust on the clean sensor / fused MSE | 0.478 / 0.0251 | **0.690 / 0.0135** | repairs half of the gate's noisy-channel inversion |
| wrong sign (power −1): trust / fused MSE | 0.672 / 0.0133 | 0.325 / 0.0422 | ×3.2 worse |

Gain-0 byte-identity: a structural comparison of the S5 reference at default params shows
zero differing lines (the new token field is the only textual change). Two unit tests.

**Creature test: Gaussian noise on the scent compass** (operator's choice).
`ScentCompass.noise_after_ticks` / `noise_sd` (default off) add N(0, sd) to the published
direction and proximity from a tick, deterministically. A/B on
`the_cell_maze_fusion__dims3.json`, 20 paired worlds, 240 s, noise sd 0.5 from tick 4800:

| arm | eats | by third (pre / mid / post) | paired vs `noisy` (eats) |
|---|---|---|---|
| base | 13.6 | 5.55 / 4.10 / 3.95 | +3.50 ± 1.76, t 4.2 (the noise's cost, 15 of 20 worlds) |
| noisy scent, legacy trust | 10.1 | 5.55 / 2.75 / 1.80 | |
| `expected` + power 2, no noise | 13.95 | 5.60 / 4.45 / 3.90 | +0.35 ± 1.09 vs base, tie (no cost when nothing is wrong) |
| noisy + `expected` power 2 | 11.6 | 5.60 / 3.30 / 2.70 | **+1.50 ± 3.20, t 1.0, 8 of 20 up, 8 down** |
| noisy + both levers | 11.45 | 5.70 / 2.95 / 2.80 | +1.35 ± 2.11, t 1.3 |
| noisy + wrong sign | 10.75 | 5.40 / 3.00 / 2.35 | +0.65 ± 2.60, tie |

**Verdict on the Cell: `NULL`.** The noise costs a quarter of the eats, the lever recovers
under half of that on the mean with a spread so wide that eight worlds improve and eight get
worse, and the wrong-sign arm is not worse than doing nothing, so the trust weighting is
not load-bearing here in either direction. The reason is the one the repair-ties-broken
result already gave: the Cell's camera is blind most of the run and `BearingFusion` drops it
on its own floor, so most of the time there is no second live channel to move weight
toward. Inverse-variance trust only has work to do when two live channels disagree in
quality, which the bench's intact pair models and this creature rarely offers. Re-use
context: a fusion with two channels alive at once (the stride-odometry complementary
filter on the picrawler is exactly such a pair, fused today by hand-set gains), or the
vision-rich `the_cell_vision_dtest` room where the camera sees food often.

**Stage 2 closes** with two shipped, default-off levers: the activity term (WORKING on bench
and creature) and inverse-variance expected-error trust (WORKING on bench, NULL on the Cell
for lack of a second live channel). K4 stale inflation is subsumed: a republished token has
zero displacement, so the activity term already discounts it (bench sub-rate arm ÷1.13).

## Stage 3 — restore lost concepts

Operator decision 2026-09-05: proceed, and feed the restored transition surprise to the play
loop.

### K2 — `transition_surprise_kind=logprob` (SHIPPED 2026-09-05; bench WORKING)

The C++ port's transition term is a displacement between successive prototypes; the Python
reference's was `−log P(cur|prev)`. Restored as an option on the EPM: the surprise is scored
against the transition table as it stood before the step, Laplace-smoothed, normalised by
`log N` to [0, 1], and **conditioned on a move** (a stay scores 0, a first arrival 1). That last
choice is the design point: with stays in the denominator every real move looks surprising,
and what the play loop wants to know is whether the *move* was expected. Default
`displacement` is byte-identical (S4 reference: zero differing lines). `PlayLoop.novelty_source`
(`tle` default, `transition_surp`, `quant_error`) lets play read it. Unit test: a three-state
cycle with a teleport, expected moves low, the teleport at the cap, the displacement unrelated
to either.

| S4 (n = 20 paired) | displacement | logprob | logprob + `gain_kind=kalman` |
|---|---|---|---|
| surprise on expected moves | 1.29 | 0.68 | 0.54 |
| surprise on teleports | 1.34 | 0.99 | 1.00 |
| **ratio unexpected / expected** | **1.03** | **1.50 ± 0.11** | **2.36 ± 0.83** |
| surprise within a state | 0.015 | 0.065 | 0.039 |
| nodes / baked | 11.7 / 7.1 | 11.7 / 7.1 | 7.75 / 5.1 |

The displacement is blind; the restored surprise separates the cases, and the separation
sharpens when the vocabulary is tighter, because over-tiling spreads each real edge's counts
across several node pairs (the B-gate lesson: finer tiling without transition statistics is
per-token sample starvation). The two levers compose.

**Creature test (2026-09-05): the study environment, 20 paired worlds, 240 s.** The report's
finding is that play is a net cost there; the question was whether a play loop that seeks
unexpected *moves* rather than high TLE stops being one.

| arm | eats | coverage (cells) | crossings | play: climb / wander / stale | paired vs `full` (eats) |
|---|---|---|---|---|---|
| full composition | 0.60 | 152.2 | 9.2 | 0.0 / 0.7 / 35.1 | |
| + logprob surprise, play on TLE | 0.70 | 150.0 | 9.3 | 0.0 / 0.7 / 35.8 | +0.10 ± 0.50 |
| + logprob, **play on the surprise** | 0.70 | 153.1 | 9.4 | 0.0 / 0.7 / 31.5 | +0.10 ± 0.57; vs the row above **+0.00 ± 0.55** |
| + logprob, play on the surprise, `gain_kind=kalman` cap 0.05 | 0.60 | 151.9 | 9.6 | 0.0 / 0.7 / 34.7 | −0.05 ± 0.44 |
| composition minus play | 2.00 | 148.2 | 9.7 | | +1.40 ± 0.38, t 7.6 |

**Verdict: `NULL`**, and the play-state column says why: **play's climb fraction is 0.0 in
every arm**, the original included. In this room the play loop never climbs a novelty
gradient; it wanders (0.7) and occasionally force-wanders (0.2), so what it is handed as
novelty cannot matter. That is the report's §4 mechanism seen from inside the loop, and it is
a finding about the play loop's value field, not about the surprise: the lever has no consumer
here until climb is live. Coverage is unchanged, so the surprise does not change where the
wander goes either. Re-use context: any consumer that acts on the surprise itself (a slow-loop
keyframe trigger, a play loop whose climb mode fires), and a place vocabulary coarse enough
that a transition table has counts to speak with. The one instrument this leaves behind is
the bench's S4 ratio, which is now a real measurement (1.03 → 1.50 → 2.36) instead of a
displacement.

K6 (true RLS + the one-tick alignment fix in the predictor's residual mode) follows; S3 is the
measurement, the picrawler `__pc*` arms the creature readout.

## Stage 4 — drift versus split

Innovation-mean test in `maybe_mitosis`: a biased residual moves the prototype, a spread one
splits it. S2 must drift-track, S4 must split.

---

## Verification

1. `cmake -S cpp_core -B cpp_core/build && cmake --build cpp_core/build -j8`; run `test_gng`,
   `test_epm_module`, `test_lateral_voter`, `test_descending_predictor`.
2. Gain-0 byte-identity: re-run the Stage-0 seeds at default params and `diff` against the
   frozen reference; the diff must be empty.
3. Bench: `tools/epm_bench/run.sh <scenario> --seeds 20 --tag <arm> --arm <m.k=v>` then
   `analyze.py --compare`.
4. Picrawler: `seedavg.py <arm.json> 6 12000 0.3` vs base, full metric set, UI observation.
5. Cell: the report's runners, **recovered 2026-09-05** from the ami-ogma clone
   (`/home/xaqmusic/ami-ogma`, branch `cell-maze`, the only branch carrying them) into
   `godot_host/project/scripts_tools/cell_coverage.py` and `cell_perturbation_d.py`, with the
   project-dir derivation adjusted. Smoke-tested on `the_cell_maze_fusion.json`
   (`--vary-world --fwdlog`, 20 s): per-seed world patched, eats and food-distance parsed, temp
   configs cleaned up. Arms are param overrides by module type, e.g.
   `--arm base --arm kalman:EPM.gain_kind=kalman,EPM.kalman_gain_cap=0.05 --n-explore 20 --vary-world`.
   The UI launcher lists the cap-0.05 arm and its j1s4 control (`launcher.gd` allowlist);
   note `metadata.gym_mode` puts UI runs in the arena while the n = 6 numbers are corridor.
6. Verdicts to the ledger with scenario, power, baseline and re-use context.
