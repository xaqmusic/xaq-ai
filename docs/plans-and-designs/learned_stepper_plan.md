# The learned careful stepper — de-scaffolding the PoseSequencer

*Plan, 2026-11-14 (operator + Claude). Operator's call: "the pose sequencer is definitely a scaffold we want to
de-scaffold with a fully learned solution leaning on the Bayes filter work; it appears this goal has gotten away from
us; let's get the concept back on track even if it means some major refactoring to do correctly." This plan
supersedes the lever-by-lever work on `PoseSequencer` (ledger 2026-11-08 → 2026-11-14). It builds on
[`generator_and_belief_filter.md`](generator_and_belief_filter.md) (G1 and G2 `WORKING`, G3 `PARTIAL`, G4–G5 not
built; register O71) and on the operator's careful-stepper intent (2026-11-06: *predict each footfall's FSR and IMU
outcome from the terrain model, and reduce that prediction's error*).*

---

## 0. Why this plan exists

The sequencer was built on 2026-11-08 as a **scaffold**: a hand-written pose controller that could hold static
support on a staircase where the trot could not. In six days it became the thing being developed:

- **111 parameters, 1 280 lines, 24 commits in a week.** Each lever fixed something the operator saw (the pole lock,
  the hung toe, the inward stance, the bob, the stretched rear leg). Each was a local rule, and most were a
  hand-written behaviour.
- **The rules override the planner without telling it.** The plan-drift instrument (ledger 2026-11-14) found the
  pose search right in 91 % of its predictions; the misses come from the stepping toe ending up ~6 cm from where the
  search put it, moved by the sequencer's own execution reflexes (the edge rule, the retreat, the hang reflex, the
  clamp). The planner does not model its own body.
- **Limits are stated in the wrong space.** The horizontal reach (equal to a straight leg at the stance depth), the
  inner reach, the sector, the landing radius and the extension margin are five proxies for one quantity — each
  leg's kinematic room — and each produced its own corner.
- **The fixed crawl order is an imposed coordination topology** (CLAUDE.md §5.7). It was introduced because the
  greedy first form starved the rear legs; the operator's "bump the priority of a stretching leg" was that patch
  showing through. Priority should be an output of the cost, not an override.
- **Nothing it predicts is learned.** `shift_realised` is a constant 0.82; the landing offsets, the margins'
  errors and the foothold errors are measured and discarded. The plan-drift counter *is* a prediction error — the
  substrate's universal learning signal — and no module consumes it.
- **The method was applied to the wrong unit.** One lever at a time against ~15 interacting rules produced a run of
  `PARTIAL`s; on 2026-11-14 six arms shipped nothing while each of three instruments overturned the hypothesis
  behind the lever just tested.

The goal is not lost: the pieces it needs mostly exist. `TerrainMemory` predicts footholds, `SupportModel` predicts
the FSR shares and the lift margins, the `Generator` decodes an EPM's nodes into graded predictions, the
`BeliefFilter` carries a belief over them, and the `DescendingPredictor`'s RLS fits action-conditioned local models.
What is missing is the **step-level generative model** that ties them together and a **policy that descends its
errors**, with the scaffold reduced to what it should be: an experience generator and a safety fallback.

## 1. The target — what "fully learned" means here

```
 senses (egocentric, audited)          belief                   generative model                      policy
 ─────────────────────────────    ────────────────────    ─────────────────────────────────    ─────────────────────────
 commanded-FK toes (per leg, in   stance EPM ─► Belief    for each stance node and step       EFE over candidate steps:
 its own kinematic terms: room)   Filter (G2): b over     ACTION (leg, landing offset, body    predicted divergence from
 FSR load shares, contact          stance situations       shift): predicted OUTCOME —         the body's preferred
 up_body / tilt                                            touchdown height error, load        outcomes + epistemic value
 TerrainMemory patch at toes and                           shares after, tilt change, each     (the generator's own σ)
 at the candidate landing                                  leg's room after, egomotion         ─► one step intent
                                                           (action-conditioned Generator, B)   (leg order emerges)
                                                                    ▲ graded by the realised outcome of every step
 floor (reflex, not learned): static support ≥ margin, IK valid within joint limits, tilt disengage
 hand (motor primitive, not a planner): shift → lift → swing → lower-until-FSR to the intent; no silent relocation
```

- **State** is perceived, not computed by a rule: an EPM over a conditioned stance vector (each toe's position in its
  leg's own kinematic coordinates — room to straight, to full fold, to the hip stops — its load share and contact,
  the body's tilt, the memory's heights around each toe). The `BeliefFilter` carries the belief through FSR
  flicker and between decisions.
- **Action** is a step: which leg, where it lands relative to its neutral place, how the body shifts first. A small,
  body-centred vocabulary (§4.4 sizes it); no fixed order.
- **The generative model** is the action-conditioned generator of design note §3.3, extended from gait handles to
  steps: per stance node and leg, a local RLS model from the step's continuous parameters (landing offset, shift) to
  the outcome vector. **Every outcome is a prediction error or a sensed quantity, never a reward.** Its targets are
  the step's realised consequences — honest action-consequence learning (doctrine §4), not a copy of the scaffold's
  choices (CLAUDE.md §5.6: nothing here imitates the scaffold).
- **Preferences** are priors over outcomes, the homeostatic form the substrate already uses: every leg keeps room,
  the body stays level, the support stays static, the landing meets the predicted ground, and the body advances
  along the commanded heading (a progress deficit against the command — the complement that stops "stand still"
  from scoring well on the others).
- **The policy** scores candidate steps by expected free energy — the predicted divergence from those priors plus
  the epistemic value of the generator's own uncertainty — over a short horizon (one step first, two once the
  stance transition model is graded). A leg that is running out of room is expensive to leave standing, so it steps:
  the operator's priority, as an output of the cost.
- **The floor** stays a reflex, because a fall supplies no gradient (memory: *fatal events have no gradient*): a
  step whose shift leaves the weight outside the remaining toes, or whose IK leaves the joint limits, is not
  executed. The floor's metric is how rarely it vetoes; a floor that vetoes often is a performance cap.
- **The hand** is a motor primitive: it executes a step intent and reports what happened. Every correction it makes
  (an FSR touchdown above or below the target, a retreat, a re-seed) is published as part of the outcome, never
  hidden from the model.

## 2. The scaffold's new role

The `PoseSequencer` is **frozen** from today: no new levers. It keeps three jobs until the gate of §5 Phase 7 removes
it:

1. **Experience generator.** Its steps produce the (stance, step, outcome) data the generator learns from. To widen
   coverage beyond its own choices, a gain-guarded epistemic jitter perturbs its chosen landing and shift within the
   floor (exploration as the substrate does it: count-annealed, never during distress).
2. **Fallback by precision.** Per stance node, the learned planner drives only where its graded error beats the
   scaffold's analytic prediction (the plan-drift instrument is that baseline, already built); elsewhere the scaffold
   drives. The learned planner's share of decisions is the de-scaffold metric.
3. **Baseline.** Its pruned, golden-traced form is the control every learned arm is judged against.

## 3. The refactor

The monolith splits along the lines §1 draws. Each split is **byte-identical to the pruned scaffold** on the golden
traces before anything learned is added.

| Component | Role | From |
|---|---|---|
| `body/LegKinematics` + a **room** measure | FK, IK (multistart), and per leg the slack to straight, to fold and to the hip stops at the toe's real depth — one quantity replacing the five proxies | `PoseSequencer` IK, `ext_slack` |
| `StepExecutor` | the hand: executes a step intent (shift, lift, swing, lower-until-FSR), publishes a `step.outcome` per step (planned vs realised landing, touchdown height, timing, every correction it made) | `PoseSequencer` phases |
| `SupportFloor` | the reflex: vetoes an intent whose support or joint limits fail; tilt disengage | `SupportModel` margin + the sequencer's safety checks |
| `ScaffoldPlanner` | the frozen pose search, emitting `step.intent`; fixed order retained here only | `PoseSequencer::plan_step` |
| `StanceFeatures` → `EPM` → `BeliefFilter` | the stance belief | new feature module; existing EPM, BeliefFilter |
| `StepGenerator` | the action-conditioned generator: per (node, leg) RLS from step parameters to the outcome vector, graded (error, NIS) | `Generator` + `DescendingPredictor` RLS |
| `StepPolicy` | EFE over candidate steps from the belief and the generator; emits `step.intent` | new; the `EFEArbiter`'s scoring form |
| `StepArbiter` | per stance node, the intent whose predictions are graded better drives | precision arbitration (doctrine §2.3) |

**As built (Phase 2, 2026-11-15).** The split is **in-process**: the components are C++ classes in
`cpp_core/{include,src}/ogma/modules/stepper/` (`StepExecutor`, `ScaffoldPlanner`, `SupportFloor`, `StepperCommon`: the
inputs, the unloaded-toe book, `StepIntent`, `StepOutcome`) and `ogma/body/LegSolver`, owned by `PoseSequencer`, which is now
the shell (topics, params, diag). Identity on the golden traces rules out bus modules here: every topic hop is a tick of
latency. The executor asks for its next step through `intent_hook_` at exactly the point the monolith decided, so that hook is
the seam where the `StepArbiter` goes; the bus split comes with Phase 5, when a second intent source exists. `step.outcome` is
published when `outcome_topic` is set (on in the stack configs) and the diag carries the latest record (`ps_out`) and the room
(`ps_room`); `scripts_tools/stepper_outcomes.py` reads them back.

Persistence: `snapshot_state` / `restore_state` exist in the module contract and the EPM implements them; the
`Generator`, `BeliefFilter`, `TerrainMemory` and the new modules must too, and the host gains a save / load of brain
state between runs, so learning accumulates over many runs and an evaluation can run on a frozen model.

## 4. Phases and gates

Every phase ends with a ledger entry and a commit. Instruments come before levers; a phase's gate is measured, not
asserted.

### Phase 0 — Freeze and baseline
- Feature-freeze `PoseSequencer`. Golden traces of the observation config on the fixed battery.
- **The battery, fixed for the whole plan:** launcher layout (seed 42, 13 000 ticks); single risers of 38 and 64 mm
  from 0.3 m × 6 seeds (9 000 ticks); the flat corridor × 2 seeds. 6 seeds instead of 4: the 38 mm spread (±0.3 m)
  is unreadable at 4.
- The scorer (`ab_score.py`, which now lives in a scratchpad), the blocker and drift readers, and the battery runner
  move into `scripts_tools/` with gzip-on-write (a full `/tmp` truncated a battery on 2026-11-14).
- **Gate:** the battery reproduces the golden traces byte for byte from the repo's tools.
- **Built (2026-11-14):** `scripts_tools/stepper_battery.sh <config> <arm>` (the battery, build and seed gates, gzip to
  `~/xaq_runs/stepper`), `stepper_score.py` (metrics), `seq_instruments.py` (blocker, drift), `stepper_ident.py` (identity;
  hashes in `scripts_tools/stepper_golden.json`, label `phase0_43811e0d`; golden logs in `~/xaq_runs/stepper/phase0_gold`).

### Phase 1 — Prune the scaffold
- Leave-one-out over the scaffold's reflexes and priors (settle, levelling, heading hold, centre pull, edge move,
  lowering retreat, fast lowering and standoff, hang reflex, IK re-seed, multistart, front drop, landing radius,
  phase timeouts and clamps): each off alone on the battery.
- One tiny instrument first: tag each landing's offset from its plan with the rule that moved it.
- A reflex whose removal ties or improves the full metric set is removed from the scaffold (its ledger verdict
  records the context). The pole lock's prevention stays measured, not assumed.
- **Gate:** a pruned scaffold that ties the current one on the battery with fewer rules; operator review before any
  code is deleted.

### Phase 2 — Split the monolith
- §3's components, byte-identical to the pruned scaffold. The `step.outcome` stream and the room measure are
  published from here on.
- **Gate:** golden-trace identity on the full battery; unit tests per component; no behaviour change.

### Phase 3 — The stance belief (instrument)
- `StanceFeatures` (conditioned: centred, scale-normalised — CLAUDE.md §0 rule 2) → stance EPM → `BeliefFilter`.
- Persistence for the learned modules and the host's save / load.
- **Gate:** the EPM bakes and stays bounded; its nodes separate the situations the instruments found decisive —
  a rear leg short of room, a front toe at its inner reach, the body nose-up on a riser, a hung toe — measured by NMI
  against instrument-derived labels used only for evaluation; the belief is consistent (NIS ≈ 1) through injected
  FSR flicker.

### Phase 4 — The step generator (instrument)
- `StepGenerator` trained on the scaffold's steps plus the epistemic jitter, over many runs with saved state.
- **Gate:** on held-out steps its errors beat the analytic models' — the next leg's liftability (the drift
  instrument's baseline), the landing's realised offset, the touchdown height (TerrainMemory's foothold error), each
  leg's room after the step — with NIS ≈ 1. If it cannot beat them, stop and redesign the representation before any
  authority.

### Phase 5 — The policy in shadow
- `StepPolicy` computes its choice every decision, logged, not executed: its predicted outcomes against the
  scaffold's, where they disagree and why.
- **Gate:** in disagreements, the policy's predicted outcomes are better *and* its predictions there are graded
  (no authority from an ungraded prediction, CLAUDE.md §5.6).

### Phase 6 — Authority by precision (the first lever)
- `StepArbiter` on, gain-0-guarded: per stance node, the better-graded intent drives.
- **Gate:** the battery's full metric set ties or beats the pruned scaffold with fewer floor vetoes; the learned
  share of decisions reported; observed in the UI before promotion.

### Phase 7 — De-scaffold
- The scaffold planner lesioned (fallback: hold the stance under the floor).
- **Gate (a finding):** walking and climbing at or above the scaffold on n ≥ 20 varied worlds; the (d) test —
  mid-run, a riser's height changes and an FSR drops out; the belief switches and the steps adapt. Then the robot,
  through the sim-to-real checklist and the sensor-legitimacy audit.

## 5. Doctrine checks

- **Rewrite rule.** No behaviour is written: leg order, landing and shift fall out of minimising predicted outcome
  errors; the operator's "priority to a stretching leg" is the room prior, not a rule.
- **Sensor first.** Two observations the model needs and the robot lacks are named, not worked around: joint stall
  (the pole lock is invisible without it — the servo rail's current on the robot, a per-servo stall sense in the
  sim) and the toe's true height at contact (commanded-FK only today).
- **EPM first.** The stance vocabulary is an EPM; no bespoke binner. The generator reads the EPM's nodes.
- **No reward.** Outcomes are prediction errors and sensed quantities; the blind-metric complement is the progress
  deficit against the commanded heading.
- **Markov blanket.** Inputs are the commanded FK, the FSRs, `up_body`, the field through `TerrainMemory` — every
  stream through the sensor-legitimacy audit; no `feet_y`.
- **No teacher.** The learned model learns consequences of steps, never the scaffold's choices.
- **No imposed topology.** The fixed order lives only inside the scaffold and leaves with it.
- **Fatal events.** The floor is a reflex, measured by how rarely it fires.

## 6. Risks and open questions

1. **Data.** A 13 000-tick run is ~130 steps. Learning needs thousands, hence persistence and long continuous runs
   in the sim (52 ticks/s, 16 in parallel). A frozen-model evaluation is the only fair A/B.
2. **Action space.** 4 legs × 45 landings × 56 shifts is too large for per-node tables; the generator is a local
   linear model in the step's continuous parameters per (node, leg), as design note §5.1 chose for gait handles.
   A local linear model has no interior optimum, so the policy searches the candidate set the floor admits.
3. **Horizon.** One step reproduces today's myopia if the room prior is weak. Two-step lookahead needs the stance
   transition model graded first (MotorPlanner's verification is the precedent).
4. **Sim fidelity.** The pole lock, slip and servo sag are sim facts with robot counterparts; a measured
   disturbance layer is the precondition for transfer (memory: *the sim is clean in specific ways*).
5. **Compute on the Pi.** A belief over a few hundred nodes is cheap; per-(node, leg) RLS over a 3–5 parameter step
   is a few hundred floats per node. Sized at Phase 4.
6. **Where the room comes from.** The commanded FK knows the target, not the real leg; the measured joint is only
   available in the sim (`HONEST_JOINTS`) and needs the sensor above on the robot.

## 7. Working protocol (autonomous execution)

- One phase at a time, in order; each ends with its gate measured, a ledger entry and a commit (DCO-signed).
- **Stop and ask the operator** at: a failed gate that changes the plan; any deletion of scaffold code (Phase 1's
  list, before deleting); the first authority handover (Phase 6); anything that touches the robot.
- Every behaviour change is gain-0-guarded and byte-identical when off; every refactor is golden-trace identical.
- Batteries never run while the operator is observing in the UI (they rotate the UI's log store); logs are gzipped
  on write.
- The resume point for each session is this file's §8 table and the ledger's latest entry.

## 8. Status

| Phase | State | Date | Ledger |
|---|---|---|---|
| 0 Freeze and baseline | **done** — gate met (15 of 15 runs byte-identical from the repo's tools) | 2026-11-14 | 2026-11-14 "Learned stepper Phase 0" |
| 1 Prune | **done** — one rule of 16 redundant (the IK re-seed, covered by multistart); off in the stack configs, its code marked `PRUNE-P1` and kept until the cleanup pass (operator, 2026-11-15) | 2026-11-14 | 2026-11-14 "Learned stepper Phase 1" |
| 2 Split | **done** — gate met: LegSolver and the planner / executor / floor split byte-identical (15 of 15 runs each), the outcome stream and the room field-identical; unit tests per component | 2026-11-15 | 2026-11-15 "Learned stepper Phase 2" |
| 3 Stance belief | **in flight** — built and measured; separation yes; the operator's two fixes measured over 16 carried runs: the own-sample heads REGRESS against legacy (overconfident, jumpier), spares-baked stops the forgetting but the vocabulary grows ~16–23 baked a run with no plateau. Diagnosed upstream: `StanceFeatures`' normaliser never settles (`n_max` 20 000 ≈ 2.3 runs, bench rotating) and slides the input ~1 quantisation cell a run. Next lever (operator's call): a normaliser that settles and holds | 2026-11-15 | 2026-11-15 "Learned stepper Phase 3" and "Phase 3, continued" |
| 4 Step generator | — | | |
| 5 Policy in shadow | — | | |
| 6 Authority by precision | — | | |
| 7 De-scaffold | — | | |
