# PART IV — The Adaptive Gains Substrate

**Status: PLANNED (operator-approved direction, 2026-08-17). Successor phase to the
PART III campaign** (`locomotion_substrate_repair_plan.md`, which carries every verdict
this plan builds on). **The operator's framing, which is this phase's charter:** the
high-value sliders (the rear-sequence trio, `amp_target`, `height_homeo_gain`,
`postural_gain`, `coupling_gain`, …) interact with wildly different effects; the
parameter space is huge but *simple compared to the homeostasis maintained by
organisms*; the question is whether these parameters can settle and stay adaptable
**from within the robot's own Markov blanket** — a slow evolution over the robot's
lifetime — rather than being found by hand or frozen by an external search.

---

## 1. The position (argued 2026-08-17, operator-agreed)

**Lifetime-vs-generational is a false dichotomy; the design is the division of labor.**
Biology does not evolve gain values — it evolves *regulatory architectures* whose
values are found and continuously re-found within each lifetime. Mapped here:

| Layer | Owns | Timescale | Status in the stack |
|---|---|---|---|
| Fast homeostats + HK | joint-level errors (drop-press, plant angle, push extension, height, amplitude) | ms–s | live (per-gain conversions pending) |
| **Lifetime parameter search** | the *values* of the high-value gain vector | minutes–hours (“the robot’s lifetime”) | **the v1 build** — precedent: the (1+1) coordination search, already promoted once |
| Generational search (optional) | *structure*: which gains exist, ranges, timescales, homeostat topology | offline, sim | exists in embryo (seedavg + SETPARAM_AT = a generation-runner); use ONLY to produce priors/initializations, never frozen constants |
| Neuromodulation (later) | coherent retuning of gain *families* by global state | s–min | DA/HT substrate exists, unused for this |

**Why lifetime-first:** (i) doctrine — §5 “adapt, don’t tune”; LEARNED cooperates,
IMPOSED fights; (ii) **sim2real** — every self-found gain re-finds itself on the
physical PiCrawler from sensors the hardware has (FSR, encoders, IMU); every
hand-or-evolution-frozen constant inherits the reality gap; (iii) adaptability — a
search that never halts converts the (d)-test from an experiment into a property.

**Where “adversarial” earns its keep:** not a co-evolved antagonist — *environmental
adversity* as selection pressure (terrain randomization, the lesion/slick-foot/damping
machinery, corridor↔arena), evolving parameter **robustness**, not parameter values.

## 2. The v1 build — `GainEvolver`

A module implementing the (1+1)-ES pattern lifted from the coordination search,
generalized to a declared gain vector:

- **Vector (v1, ~8-D):** `rear_land_gain`, `rear_knee_plant`, `rear_push_ext`,
  `amp_target`, `height_homeo_gain`, `postural_gain`, `coupling_gain`,
  `plan_gain`. Declared per-config (topics-style param list); anything not listed
  stays a hand knob.
- **Loop:** hold incumbent → mutate the *vector* (never coordinate descent — the
  cross-couplings are the point) → evaluate candidate over a long window → keep iff
  no worse on the guard AND better on the criterion → else revert. Mutation scale
  self-annealed (1/5th-rule flavor); evaluation windows ≥ 4–6k ticks (our own
  seed-averaging discipline applied inward: short windows are noise).
- **Criterion — intrinsic viability + flow, error-form ONLY (the no-reward rule):**
  distress duty, falls, tilt variance, unloaded-contact rate (foot_load at
  touchdown), post-plant slip (foot drift while planted), flow quality (the
  fwd-flow homeostat form: magnitude × predictability — the one sanctioned
  speed-flavored term). Never raw speed. All egocentric; all sensors the real
  robot has.
- **Guards, designed in from day one (each is a recorded PART III burn):**
  criterion held stationary while candidates vary (the ratchet lesson — the
  evaluator must not be modified by the intervention it scores); a no-regression
  viability guard separate from the improvement criterion (the demo-mask lesson:
  target-wins-body-pays must be rejected); per-leg minima in the criterion (the
  stance-capture lesson: group means hide a dead leg); mutation-rate 0 = frozen
  incumbent = byte-identical (the gain-0 guard).
- **Instruments first:** current vector + incumbent/candidate scores + accept/revert
  history mirrored to the body log and a panel readout — the operator must be able
  to *watch it search*. Consumer-fired counters per §3.2.

## 3. Gates and protocol

1. **Smoke:** evolver ON, mutation 0 → byte-identical to the seeded operating point.
2. **Convergence gate (arena, the operator's protocol):** seeded at factory-default
   gains, does the vector walk toward (or past) the operator's hand-found point, and
   does the criterion trace fall? n=4 seeds; judged on the criterion trace + the
   standard behavioral set (no regression vs the hand point).
3. **The (d)-test — the phase's headline evidence:** mid-run perturbation (slick
   foot / terrain swap / damping shift) → criterion degrades → the vector migrates →
   behavior recovers *without any external input*. This is “settle AND remain
   adaptable” demonstrated in one run.
4. **Operator's eye** remains the promotion gate for any operating point the evolver
   settles on (the by-eye multi-objective stays human).

## 4. Sequencing

1. **Seed the operating point:** the operator's current slider session settles the
   rear-trio + companions; bake as the named scaffold values; n=20 the point (this
   also closes the rear-sequence promotion from PART III).
2. Build `GainEvolver` v1 (vector, loop, criterion, guards, instruments) — one
   lever, gain-0.
3. Gates 1–3 above; ledger every verdict with re-use context.
4. **Only then** consider the generational layer (priors search under environmental
   adversity) and the per-gain fast homeostat conversions (drop-press from
   foot_load, plant angle from slip — designs recorded 2026-08-17 in the PART III
   log) as *subsequent levers* — the evolver is the umbrella that makes each of
   them safe to add.

## 5. Open questions (decision points, not blockers)

- **Context-dependence:** one global vector v1; per-context vectors (terrain-binned,
  via the support/pc vocabularies) are the natural v2 once B's substrate matures.
- **Criterion weighting:** fixed error-form weights v1; neuromodulated weighting
  (DA/HT as global context) is the recorded third act.
- **Interaction with B:** a stride-carving vocabulary (PART III's B thread, self-mass
  0.27) could eventually *predict* criterion moves before they happen — planning in
  gain space. Out of scope until B's planner layer earns a verdict.

---

## 6. The v1 build — BUILT 2026-08-17 (`IN_FLIGHT`, gate 1 PASSED)

### 6.1 Architecture as built (three operator fork decisions)

1. **Separate module + gain topic**, not a search inside MotorEPMv2. `GainEvolver`
   publishes a `GainVector` (parallel `keys`/`values` — the mapping is explicit in every
   message, never config-side agreement) on `gain.motor_epm`; MotorEPMv2 gained a
   `gain_topic` socket (ConstructionOnly, `""` = off = byte-identical). **Why it matters
   beyond tidiness: the evaluator is stationary BY CONSTRUCTION** — the criterion reads
   only body-published `reality.proprio.*` topics and the module's own EMAs, so it
   cannot read a quantity the mutated gains regulate through MotorEPMv2's internals
   (the stationary-evaluator burns: a self-referential threshold is not a sensor; a live
   homeostat silently restores the measured quantity).
2. **Post-plant slip DEFERRED** — no egocentric slip signal exists (`lateral_v` is a
   soft oracle, `_grf_fwd` is declared permanently god's-eye). Ledgered with its re-use
   context; it slots in later as one more `w_*` term with zero loop changes.
3. **Interleaved incumbent re-evaluation** replaces the precedent's stored-winner score.
   Each generation = incumbent window → candidate window → *contemporaneous* compare.
   No stored best, no `×0.99` forget. This kills the residual ratchet shape the ledger
   flags (a lucky escape thrash becoming a permanently-reverted-to incumbent) and makes
   the (d)-test structural: after a perturbation the incumbent's score degrades within
   one window instead of waiting on a decay constant.

### 6.2 The criterion and guards as built

```
J = w_falls·falls + w_tilt_var·var(upright) + w_distress·distress_duty
  + w_unloaded·unloaded_contact_mean + w_flow·(1 − flow_quality)     [lower = better]
```
Defaults 1.0 / 5.0 / 0.5 / 0.5 / 0.5. Continuous terms measure the **back half** of each
window (front half = settling, the coord-search precedent); falls count whole-window.
A leg below `min_touchdowns` scores **fully unloaded** — a non-stepping leg must never
look clean. `flow_quality = clamp(flow_ema,0,0.05)/0.05 / (1+4·flow_vol_ema)`, form
lifted from MotorEPMv2's fwd-flow homeostat; its `fwd_v` input is a soft oracle and is
the charter's one knowing exception.

Guards, evaluated **separately** from J (`accept ⇔ G1 ∧ G2 ∧ J_cand < J_inc`):
- **G1** falls no-regression: `cand.falls ≤ inc.falls + viability_falls_tol`
- **G2** per-leg loaded-contact **minimum**: `min_l(loaded_l)` may not fall more than
  `viability_load_tol` below the incumbent's — never a group mean (the stance-capture
  lesson: the GLOBAL amp homeostat satisfied the group mean by over-driving the living
  legs while one leg was dead).

Falls are counted as **debounced `upright < thresh` EDGES** (25 ticks, deliberately under
the body's 30-tick inversion dwell so the edge fires before auto-reset snaps upright
back) — never the harness's god's-eye `chassis_y` detector, and never a duty.

`mutation_sigma = 0` ⇒ **silent observer**: the evaluator still scores windows for the
instruments (the criterion is watchable on the promoted stack before any search runs),
but nothing publishes, no RNG is drawn, nothing mutates. σ self-anneals (1/5th-success
flavor) between `sigma_min`/`sigma_max`; the search never fully stops, which is the
"settle AND remain adaptable" property stated as a mechanism rather than a hope.

Two collisions refused at construction rather than tuned around: `gain_topic` +
`amp_seek_rate > 0` **throws** (both would own `amp_target`), and a declared vector whose
parallel arrays disagree or whose seed is out of bounds throws.

### 6.3 Files

| Path | What |
|---|---|
| `cpp_core/include/ogma/Topics.hpp` | `GainVector` message |
| `cpp_core/src/ogma/PayloadTypeName.cpp` | type-name entry (else it reports "Unknown") |
| `cpp_core/{include,src}/ogma/modules/GainEvolver.{hpp,cpp}` | the module |
| `cpp_core/src/ogma/ModuleRegistry.cpp`, `cpp_core/CMakeLists.txt` | registration |
| `cpp_core/src/ogma/modules/MotorEPMv2.cpp` (+ hpp) | the gain socket + counters + restore replay |
| `cpp_core/tests/ogma/test_gain_evolver.cpp` | 12 tests, all passing |
| `godot_host/src/OgmaBrain.cpp` | `get_module_metrics` branch (metrics, **not** snapshot — the per-tick lesson) |
| `godot_host/project/scripts/picrawler_body.gd` | `ga_app`/`ga_rej` (inside `_mm`) + the `ge_*` block |
| `godot_host/project/scripts/gain_evolver_panel.gd` + `scenes/the_picrawler.tscn` | the `[U]` panel |
| `configs/…__planpull__gainevo.json`, `…__gainevo_factory.json` | the deployable arm (σ=0) + the gate-2 arm |

**Restore replay is load-bearing:** evolved gains live in param members the instance
snapshot does *not* round-trip (params come from the GraphConfig), so `restore_state`
re-dispatches `applied_gains_` through `on_param_change`. Without it a restored clone
silently reverts to config gains and the clone-determinism test would blame the evolver.

**The applied-counter is read-back verified**, not assumed: MotorEPMv2's
`on_param_change` chain has **no terminal else**, so an unknown key is silently ignored.
The socket therefore re-reads `current_params()` after dispatch and counts a landing only
when the value actually matches — a typo'd key increments `gains_rejected` instead of
vanishing. (§3.2 rule 5: a gate has shipped here as silent dead code before.)

### 6.4 Gate 1 — PASSED (2026-08-17)

Promoted config vs `__gainevo` at σ=0, `OGMA_SEED=7`, corridor, 12 000 ticks. Diff =
body-log JSON lines, `ge_`/`ga_` keys dropped (the instrument delta is by design),
sorted-key re-serialize, exact compare: **251 lines each side, byte-identical.** The
observer instruments were confirmed live in the same run (`ge_ji` 0.4578, `ge_ph` 1,
per-term breakdown populated, `ge_pub` 0) — so this is byte-identity *with the evaluator
running*, not byte-identity because the module was inert.

**Live-search smoke** (σ=0.08 via `SETPARAM_AT` at tick 1, 30 000 ticks, seed 7): the
loop runs — 3 generations, 1 accept / 2 reverts, σ held 0.08, and the vector migrated
`[0.5, 0.2, 0.5, 0.4, 0.04, 0.7, 1.55, 0.05] → [0.415, 0.128, 0.840, 0.378, 0.074,
0.673, 1.321, 0.047]`. **The §3.2 two-sided consumer check passes exactly:
`ga_app` 64 = `ge_pub` 8 × 8 keys, `ga_rej` 0** — every published vector landed, every
key mapped. `rlt`/`rpt` kept climbing (10 609 / 30 100), so the rear-landing consumers
stayed live under evolution. **This is a pipe-integrity result and nothing more:** 3
generations on one seed says nothing about convergence, and the J trace moved *up*
(0.281 → 0.336) across them, which at this power is noise.

### 6.5 Gate 2 — RUN 2026-08-17: mechanism PASSES, convergence NOT DEMONSTRATED

Arm: `…__gainevo_factory.json` (factory seeds `[0, 0.2, 0, 0.4, 0, 0.3, 0, 0]`,
σ 0.08), **arena**, n=4, 256 000 ticks, difficulty 0.3, **solid chassis**
(`OGMA_PICRAWLER_CHASSIS_COLLIDE=1` — seedavg does not set it and the ghost default
would make sagging free while `height_homeo_gain` is under evolution). 31 generations
per seed.

**What passed.**
- **Consumer lockstep, all 4 seeds: `ga_app` 512 = `ge_pub` 64 × 8 keys, `ga_rej` 0.**
  Every published vector landed on every key; the pipe is proven at scale.
- **Tautology check: accepts > 0 on 4/4** (14 / 14 / 10 / 17 accepts against 17 / 17 /
  21 / 14 reverts) — the search discriminates rather than rubber-stamping.
- **Direction is sensible on the structural gains.** Pooled finals vs the hand point:
  `coupling_gain` 0 → **2.06** (hand 1.55) — turned ON in all four seeds from a
  factory default of zero, i.e. the search rediscovered that the legs must be coupled;
  `height_homeo_gain` 0 → **0.039** (hand **0.040**); `rear_push_ext` 0 → 0.63 (hand
  0.5); `postural_gain` 0.3 → 0.57 (hand 0.7); `rear_land_gain` 0 → 0.94 (overshoots
  hand 0.5). 3/4 seeds ended closer to the hand point in L2 (1.75 → 0.81 / 1.65 /
  1.84 / 1.05).

**What did not pass — and why, measured.** The J trace does **not** fall: half-run
deltas were +0.30 / −0.09 / **+0.70** / −0.62. Against a per-seed incumbent-window sd
of **0.54 / 0.88 / 2.08 / 1.21**, every one of those deltas sits *inside the noise*.
Pooled over 128 incumbent windows, **J = 1.009 ± 1.418**.

**The variance decomposition names the culprit exactly** (weighted contributions,
128 windows):

| weighted term | mean | sd | share of J variance |
|---|---|---|---|
| `w_falls·falls` | 0.445 | 1.059 | **80.9 %** |
| `w_tilt·var(upright)` | 0.155 | 0.502 | 18.2 % |
| `w_flow·(1−flow_q)` | 0.386 | 0.082 | 0.5 % |
| `w_dis·distress_duty` | 0.015 | 0.060 | 0.3 % |
| `w_unl·unloaded_mean` | 0.009 | 0.045 | 0.1 % |

**Falls alone is 81 % of the criterion's variance**, and the three gait-quality terms
together are under 1 %. Falls is a *rare discrete count* over a 2 000-tick measured
half-window — its relative variance is enormous, and it drowns the very terms the
criterion was designed to select on.

**Second finding: the 1/5th-rule anneal amplifies this instead of absorbing it.**
Acceptance ran 32–55 %, far above `target_accept` 0.2, so σ was driven **up** — to the
`sigma_max` 0.5 **ceiling on 3 of 4 seeds**. That is the signature of a coin flip: a
noisy criterion yields ~50 % acceptance, the classic 1/5th rule reads that as "I am
succeeding, take bigger steps," and the search widens instead of settling.
**The 1/5th rule assumes a deterministic objective; ours is stochastic.**

**Remedies, in the order they should be tried:**
1. **Lengthen `eval_window_ticks` well past 4 000** — the charter's "≥4 000–6 000" is
   now measured to be far too short; noise sd ~1.4 against a signal ~0.3.
2. **Damp the falls term's variance** — cap it per window, or convert it to a
   fall-*rate* over a much longer horizon, so a Poisson count cannot own 81 % of J.
3. **Make acceptance noise-aware** — require `J_cand < J_inc − k·sd` rather than a bare
   inequality, and/or anneal against an improvement *magnitude* instead of a raw
   accept rate.
4. **Common random numbers** — evaluate incumbent and candidate over the same
   perturbation sequence so their difference cancels shared noise.

**Two context caveats on these numbers.** Over 256 k ticks in the arena all four seeds
reach the floor edge (`max_z` ≈ 9.9–10.1) and its containment ramps, so some of the
falls are boundary events rather than gait failures — which inflates precisely the
dominant term. And the factory body is genuinely unstable as predicted (tilt_sd 0.339,
`bellyc_min` 0.000 — the belly does reach the ground on a solid chassis), so this is
the hard version of the convergence question, not the friendly one.

**Verdict: `PARTIAL`.** The machinery is correct and proven end-to-end, and the vector
moves in defensible directions on the gains that matter most — but *convergence is not
demonstrated*, and by §3.3 the honest response is to fix the instrument (window length
and the falls term) rather than to power the effect with more seeds.

### 6.7 Gate 2 RE-RUN (2026-08-17) — the fix works; convergence still not shown

Criterion repaired per §6.5's remedies (falls → guard-only, var → sd, `w_distress` 0,
window 12 000, acceptance must clear `k·σ̂` estimated from the search's own revert pairs),
plus `auto_reset_on_outer_wall=1` — the operator's "recenter on the wall", which already
existed and simply needed enabling. n=4, arena, solid chassis, 500 k ticks, 20 generations.

**Every axis the fix targeted moved the right way:** pooled measurement noise 0.8765 →
**0.1913**; σ at the ceiling **3/4 → 0/4**; falls' variance share 80.9 % → 0 %; reverts now
outnumber accepts 2:1 (the margin is biting); falls per 100 k ticks 11.43 → 9.50 and
tilt_sd 0.339 → 0.239.

**The gate still fails.** Against each seed's own noise: seed 3 fell −0.348 vs 0.143
(2.4× — the campaign's first *claimable* improvement), seed 4 rose +0.232 vs 0.136, seeds
1–2 sat inside noise. One win, one loss, two nulls.

**The cost, and the lesson.** `coupling_gain` — gate 2's strongest result at 0 → 2.06 in
**4/4** seeds — collapsed to **0.61 ± 0.57 with 2/4 seeds leaving it at zero.** The falls
term was carrying the selection pressure that discovered coupling; `sd(upright)` does not
reward coordination nearly as hard. **A term can be simultaneously the noisiest thing in a
criterion and the sole carrier of a real signal; "the noise went down" is not evidence the
criterion improved.** The next design restores that pressure in a low-variance *form* — a
continuous near-inversion dwell, `mean(max(0, thresh − upright))`, which is sampled every
tick and is the actual pre-fall regime — rather than reinstating the Poisson count.

⚠ `net_disp`/`straight` are void in this arm: recentering teleports the body to the
origin, so displacement cannot accumulate (straight reads 0.01). Falls/tilt are per-tick
normalised across the differing run lengths (256 k vs 500 k).
- **Gate 3 ((d)-test):** `OGMA_PICRAWLER_SLICK_LEG=2 OGMA_PICRAWLER_SLICK_AT=200000`
  (leg 2 = cfg `rl` = anatomical REAR-RIGHT) + σ on at tick 1, 400 k ticks, one run.
  Judge migration + **partial** J recovery; demanding full return would over-claim, as
  the friction loss is permanent.
- **Job #1 (the falls tail):** σ=0.08 from the baked seed, ~200 k × 4 seeds → operator
  picks a settled vector from `[U]` → re-bake → `seedavg.py <baked> 20 12000 0.3`
  (seeds include s3/s12). Accept iff the tail shrinks with `rlt`/`rpt` held and the
  per-leg loaded minima not below baseline − tol. **The operator's eye remains the
  promotion gate.**

### 6.6 Known caveats carried into the gates (not defects of the loop)

- `w_distress` consumes the body's **known-contaminated** distress signal as-is: its
  stall half is a world-frame XZ displacement (audit-illegal) and it carries an apparent
  50 Hz-vs-240 Hz normalisation error that makes it under-fire. Fixing the sensor moves
  the deployed panic + plan-distress-cut and is therefore a **separate lever**. If
  `ge_dis` reads ~0 throughout, that weight is dead — a measurement about the sensor,
  not a verdict on the criterion.
- `coord_reward_drive 0.3` and `coord_adapt_rate 0.001` stay **live** inside every
  window: the inner phase search is part of the plant being evaluated (240-tick probes,
  ~10 per measured half-window — symmetric noise across both windows).
- Window noise is the main threat to a convergence claim. If generation-to-generation
  `ge_ji` spread across *incumbent* windows rivals the candidate deltas, accepts are
  coin flips and "converged" would be false; the remedy is a longer
  `eval_window_ticks`, and the spread is measurable directly from the log.
