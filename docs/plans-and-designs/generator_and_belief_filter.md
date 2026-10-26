# The generator and the belief filter — the EPM's missing complement

*Design note, 2026-10-24 (operator + Claude). G1-G3 are built as instruments and measured (§8); G4-G5 are not built. It closes a gap that has been open since the
EPM became the substrate: the EPM recognises, and nothing generates. It also states what the substrate needs before
it can claim any parity with a Kalman filter, building on
[`epm_kalman_lessons_plan.md`](epm_kalman_lessons_plan.md). Register entry: O71.*

---

## 1. The gap

An EPM maps an observation to a token: encode, find the winner, publish `winner_id`, `latent`, `tle`, `is_novel`
(`primitives/EPM.md`). That is recognition, with an error that says how well it knows the input. Nothing in the
substrate runs the other direction — from a token to content in some space that can be checked or used — with an
error of its own.

What exists already, and what each lacks:

| Piece | What it does | What it lacks |
|---|---|---|
| `DescendingPredictor` | global linear map consensus → an EPM's next encoder latent, subtracted before the GNG | its prediction lives in encoder space and is never output |
| GNG transition counts, EPM `predicted_pathway`, `GNGRollout` | which node comes next | a prediction of *content*; and it is never fused back into the estimate |
| `MotorPlanner` (v1/M1) | a phase-conditioned cone over future tokens; per-joint marginals decoded and **verified** (authority in k ∈ [8, 34]) | zero authority by design; an instrument |
| `plan_target` / `plan_fade` | an authority path: published joint targets drive the servos | nothing learned feeds it |

So the substrate has token-space prediction, latent-space prediction and an actuator path, and no learned decoder
between them. Every "output" a picrawler module produces today is a hand-written control law reading the EPM's
inputs directly, which is why the operator's question — how should the robot choose its gait parameters for where
it is — has no substrate answer yet.

## 2. The Kalman correspondence, completed

The charter's table (its §0) lists what corresponds today. Completed with what a filter also needs:

| Kalman element | Substrate | Status |
|---|---|---|
| Measurement noise R | per-node `ema_error`; `expected_error` on the token; voter K3/K8 | present |
| Gain K | `gain_kind=kalman` (Stage 1; cap 0.05 `PARTIAL` on the body) | present |
| Innovation | TLE; log-probability transition surprise (K2) | present, but a distance in encoder space |
| Process noise Q | `kalman_q`, drift vs split (Stage 4) | present, default off |
| Dynamics F | GNG transitions; MotorPlanner's phase-conditioned cone | partial: learned, never used to estimate |
| **Measurement model H** (state → predicted observation) | — | **missing → the generator (§3.1)** |
| **Control B** (action → change of state or outcome) | — | **missing → the action-conditioned generator (§3.3)** |
| **Belief as a distribution; prior × likelihood** | one hard winner a tick | **missing → the belief filter (§3.2)** |
| Uncertainty growth with no reading | a sub-rate republish keeps the old TLE | **missing → §3.2's inflation** |

With all four, the substrate is a learned, nonparametric, multimodal Bayes filter: the forward algorithm of a hidden
Markov model whose states are the EPM's earned nodes, whose dynamics, observation model, noise and drift are all
estimated from the stream. That is the claim this note works toward, and §6 says what would and would not count.

## 3. The primitives

### 3.1 Generator (H) — a node's prediction of content, graded by its own error

**Inputs.** One EPM's `RealityToken` (`winner_id`, `pruned_ids`, `just_mitosis`); one or more target streams in their
own units (`ProprioToken`s: a raw sensor, a derived gait statistic, joint angles).
**Per node k.** A running mean μ_k and diagonal variance σ²_k of the target observed `horizon` ticks after k won, and
a count n_k. The update uses the Kalman schedule for a constant, `1/(n+1)`, capped (the charter's Stage 1 lesson: the
uncapped head chases a moving input). A new node starts from the nearest existing node's statistics with a small count;
a mitosis child copies its parent; a pruned node is dropped.
**Outputs** (`prediction.<name>`): the predicted target for the current winner (μ, σ); the last *matured* prediction's
error; the normalised innovation e²/σ² (its expected value is 1 per dimension for an honest generator — the
consistency statistic the port lost, charter §0); the generator's running error, globally and per node.
**Error.** Always computed and published. A generator is never trusted for anything its error does not support
(CLAUDE.md §5.6).
**Defaults.** An instrument: no consumer acts on it. Absent from every config until added; a config without it is
byte-identical.
**Not a clusterer.** It owns no partition of its own; it reads the EPM's (CLAUDE.md §0 rule 1). It is the EPM's
decoder, not a second vocabulary.

### 3.2 Belief filter (F + H fused) — predict, then correct

**State.** A belief b over the EPM's live nodes.
**Predict.** b⁻ = T · b + diffusion. T is the node transition matrix counted from consecutive winners;
phase-conditioned where the body is rhythmic (MotorPlanner M0: an unconditioned chain on a rhythmic body degenerates to
persistence). The diffusion is Q: with no reading, the belief spreads instead of freezing (inflation).
**Correct.** The likelihood of each node is the observation under the generator's decode head (horizon 0): N(obs;
μ_k, σ_k), in observation units. b ∝ b⁻ · likelihood. With no fresh reading the correct step is skipped.
**Outputs.** The MAP node and its probability; the belief's entropy; the filtered estimate E_b[μ] (the analogue of the
Kalman state estimate, in observation units); and the predictive surprise −log p(obs | b⁻), the proper innovation.
**Why it matters on the robot.** Three of the current robot problems are filtering problems: the FR FSR flickers at
twice the others' rate (a contact belief coasts through flicker); the belly ToF arrives every ~30 ms with gaps; and a
situation belief for parameter choice (§3.3) must not switch on one noisy stride.

### 3.3 Action-conditioned generator (B) — predicted outcome of a setting

For the operator's parameter-selection question (2026-10-24). Per node k of a slow gait-context EPM (input: a compact
egocentric summary of gait health — stride regularity, per-leg load shares, early-swing contact, belly, tilt
variability, progress deficit), a local model of how the gait's prediction errors respond to the live handles
(LPG period, swing lift, press target, load hold, learned-layer gain …): o ≈ α_k + β_k · (u − u_k), fitted by the
DescendingPredictor's true RLS (charter K6, bench `WORKING`). **The outcome o is a set of prediction errors, never a
reward:** early-swing contact (the phase predicted the foot in the air), contact/phase disagreement, progress deficit
against the commanded speed (StuckSense: expected versus measured), belly error, the motor forward model's error.
**Selection** — off by default and its own lever: in node k, u* minimises the predicted o plus an epistemic bonus on the
model's own uncertainty, moved toward slowly and held long enough to be measured (one handle at a time, the
GainEvolver's (1+1) discipline with a clean criterion — the 2026-08-23 audit found its old criterion read an oracle
velocity). Per-node storage protects what works: learning the rumble strip cannot overwrite the flat-ground walker.

### 3.4 Growth from generative failure

An EPM grows where its input is novel. A node can be familiar as an input and useless for output — "flat and healthy"
and "a rear leg not bearing load" may summarise alike and need opposite settings. Where a generator's per-node error
is persistent, irreducible and localised after baking, that is doctrine §5's growth criterion, and the node should
split. **Hypothesis**, tied to O4 (mitosis liveness: the restored gatekeeper's absolute threshold never splits): the
generator's error is the split signal the gatekeeper lacks. The vocabulary would then grow where the robot cannot
predict the consequences of acting, not only where its inputs are new.

## 4. Doctrine checks

- **Rewrite rule (CLAUDE.md §1).** The selection in §3.3 implements no behaviour: it descends the gait's own prediction
  errors.
- **EPM first.** No new clusterer; the generator and the filter read the EPM's nodes.
- **No reward (§5.1).** The outcome is prediction error. The blind-metric check is mandatory at each stage: what
  degenerate gait would also lower these errors (standing still lowers the progress deficit only if the commanded speed
  is zero — the deficit is measured against the command, so it is not satisfiable by stopping).
- **Markov blanket (§5.3).** Every target stream must pass the sensor-legitimacy audit
  (`sensor_legitimacy_and_the_feet_y_oracle.md`); no god's-eye quantity as a target or a criterion.
- **Graded error (§5.6).** Each generator publishes its error and consistency statistic; nothing acts on an ungraded
  prediction.
- **Gain-0 (§3).** Every stage defaults off and is byte-identical when off.

## 5. Stages and gates

| Stage | Build | Gate (measured) |
|---|---|---|
| G1 | Generator (§3.1), instrument | bench: decode error approaches each scenario's floor; normalised innovation ≈ 1 (consistency); robot/sim: a gait EPM's generator predicts next-stride early-swing contact and belly within its own stated σ |
| G2 | Belief filter (§3.2), instrument | bench: filtered estimate vs the Kalman floor on S1–S6 (charter Stage 0 baselines; K6 aligned RLS reached 0.029 vs floor 0.018 on the fast rotation); a **dropout** scenario (inflation, then recovery); a **multimodal switching** scenario where a single-Gaussian KF fails; sim: an FSR contact belief through injected flicker |
| G3 | Gait-context EPM + action-conditioned generator (§3.3), instrument | the context EPM separates flat / rough / slope / lesion into distinct baked nodes (conditioning first, CLAUDE.md §0 rule 2); the outcome model's predictions verified against held settings |
| G4 | Selection on (§3.3), one lever | A/B vs the fixed promoted settings, full battery; the (d) test: lesion a leg / change terrain mid-run, the belief switches and the gait recovers; n ≥ 20 varied worlds for a finding |
| G5 | Generator error as the split signal (§3.4) | the split nodes reduce the generator's error where they split, on the bench and the body; O4 |

**Bench additions.** S7 multimodal switching (two regimes, abrupt, where the KF's single Gaussian averages them);
S8 dropout (a channel silent for a run of ticks: the belief's spread must grow, the estimate must not freeze, and the
re-acquisition must be fast). Both close-form-scored like S1–S6.
**First creature instrument (G1).** The picrawler sim, robot-faithful inputs: the existing proprio EPM's generator
predicting per-leg early-swing contact and belly 1 stride ahead. Robot second, from a bench run's brain_diag.

### 5.1 G4 decisions (operator, 2026-10-25)

- **Handle: swing lift**, one handle. Reason: the terrain-arena capability map (ledger 2026-10-25) — both walkers stall
  at 38-50 mm vertical edges while pyramids and slope humps pass; it is also the robot's dragging problem, and it is
  already live-tunable.
- **Situation: terrain edges** (high edges vs flat) first. Depends on G3 separating terrain types on the terrain arena
  (re-test running 2026-10-25, 20 varied worlds); if it does not, G4 would be measuring the vocabulary, not selection.
  The frozen leg (already separated, NMI 0.57) is the natural second lever.
- **Model: continuous, per situation.** A linear outcome model alone always predicts a bound as optimal (no interior
  minimum), so selection is a **local descent**, not an argmin: each node keeps an operating point u_k; trials dither
  u_k ± δ (held, see below); RLS fits the local slope β_k of the outcome; u_k steps downhill by a bounded amount;
  δ anneals with the slope's certainty (count/variance-annealed, CLAUDE.md §5.5). A new node starts from its
  neighbours' operating point (or the global one).
- **Outcome (prediction errors, never reward):** early-swing contact (phase predicted air, the foot touched) and the
  progress deficit (StuckSense, expected vs measured for the commanded speed); tilt variability as the complement
  (blind-metric check: "always lift high" would also lower early-swing contact).
- **Exploration: conservative.** Change only when the belief is confident and stable; hold each trial a few hundred
  ticks; discard a trial whose situation changed mid-way (open question 4, answered); known-safe bounds on u; never
  explore during distress or recovery. Gain-0 / default off.
- **Gate:** as §5 — A/B vs the fixed promoted settings on the terrain arena, n ≥ 20 varied worlds, full battery,
  plus the (d) test (mid-run terrain change / frozen leg: belief switches, setting switches, gait recovers). Sim only
  until it passes.

## 6. What "parity with a Kalman filter" would and would not mean

- **Not optimality.** A Kalman filter is optimal only for a linear-Gaussian world with a known model. The substrate's
  claim would be: **with nothing specified**, it approaches the Kalman floor where that floor exists (S1–S6), stays
  consistent (normalised innovation ≈ 1), and beats the single-Gaussian filter where the world is multimodal (S7).
- **The evidence bar** is the repo's: bench scenarios with a closed-form optimum, n = 20 seeds, paired; on the body,
  signal at n = 6 and a finding at n ≥ 20 with the (d) test.
- **The isomorphism note's own caution stands** (`../research-summaries/epm_kalman_filter_finding.md`): trust from
  realised error is adaptive-Kalman-like and a feature; the GNG's many nodes make it a mixture / particle filter
  analogue, not a single-Gaussian one.

## 7. Open questions

1. Where the filter lives: a separate module (reads the EPM's token and the raw observation; needs only node ids and
   lifecycle events) or inside the EPM (the winner could become the MAP of the belief rather than the nearest prototype —
   which changes the EPM's own learning and is a much larger lever). Start separate.
2. Phase conditioning of T without a hand-made phase binner: MotorPlanner's form, or a smooth phase basis (as the
   learned stance press used, plan 7.31).
3. The DescendingPredictor as the generator's global special case: unify later, or keep both.
4. Credit assignment in §3.3 when the context changes while a setting is being measured: discard the trial, or weight
   it by the belief.
5. Cost on the Pi: a belief over a few hundred nodes is cheap; a generator per target per node is a few floats; the
   per-node RLS of §3.3 is the expensive part (handles × outcomes per node) and should be sized before G3.

## 8. Results so far (2026-10-24)

| Stage | Built | Measured | Status |
|---|---|---|---|
| G1 | `Generator` (+ bench S9) | bench: converged excess over the exact floor 0.9 ± 1.8 % (n = 20); body: foot contact half a stride ahead +0.79..+0.81 vs persistence, NIS 1.2-1.6; belly not predictable from pose | `WORKING` |
| G2 | `BeliefFilter` (+ bench S10 with a true-model forward filter and a best-tuned Kalman filter) | estimate 2-3x better than the EPM winner, 1.6-6x better than the Kalman filter with readings; within 21-25 % of the optimum through silences; gap with readings = an over-tiled vocabulary | `WORKING` (bench); not yet on the body |
| G3 | `FeatureWindow` + slow EPM + Generator | a frozen leg is its own situation (NMI 0.56 vs 0.23); terrain zones weakly (0.31 vs chance 0.22); outcome prediction modest, overconfident | `PARTIAL` |
| G4 | — | — | not built (a behaviour change) |
| G5 | — | — | not built |

Verdicts and details: `docs/reports/picrawler_lever_ledger.md`, entry of 2026-10-24.
