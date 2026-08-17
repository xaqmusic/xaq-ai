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
