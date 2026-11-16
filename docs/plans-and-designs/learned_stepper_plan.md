# The learned careful stepper — de-scaffolding the PoseSequencer

*Plan, 2026-11-14 (operator + Claude). Operator's call: "the pose sequencer is definitely a scaffold we want to
de-scaffold with a fully learned solution leaning on the Bayes filter work; it appears this goal has gotten away from
us; let's get the concept back on track even if it means some major refactoring to do correctly." This plan
supersedes the lever-by-lever work on `PoseSequencer` (ledger 2026-11-08 → 2026-11-14). It builds on
[`generator_and_belief_filter.md`](generator_and_belief_filter.md) (G1 and G2 `WORKING`, G3 `PARTIAL`, G4–G5 not
built; register O71) and on the operator's careful-stepper intent (2026-11-06: *predict each footfall's FSR and IMU
outcome from the terrain model, and reduce that prediction's error*).*

*Re-ordered 2026-11-16 (operator's call, §9): authority comes sooner. Phase 3's gate no longer asks the stance vocabulary
to converge on the stairs; after the percept fix, the next phase is the first closed loop — the learned layer owns one
value of the scaffold's step (the landing's forward/back offset), chosen by expected free energy — and authority then
widens one value at a time. The old Phases 4–5 (generator and policy as instruments) are folded into that ladder.*

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
   floor (exploration as the substrate does it: count-annealed, never during distress). *Since 2026-11-16 the jitter is
   not a separate mechanism: it is the epistemic term of the learned layer's own choice (Phase 4), which explores while
   its model is ungraded and exploits once it is graded.*
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
the seam where the `StepArbiter` goes; the bus split comes when a second whole-intent source exists (Phase 6 since the 2026-11-16 re-order; Phase 4's offset component
sits in-process at this same seam). `step.outcome` is
published when `outcome_topic` is set (on in the stack configs) and the diag carries the latest record (`ps_out`) and the room
(`ps_room`); `scripts_tools/stepper_outcomes.py` reads them back.

**Re-ordered (2026-11-16).** `StepGenerator` and `StepPolicy` are built small and grown, not built whole: their first form is
the Phase 4 landing-offset component (one value, per (node, leg) RLS, an EFE choice over a candidate set), and each value
Phase 5 earns widens their action vector. `StepArbiter` arrives at Phase 6, when whole intents exist.

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
- **What is left (2026-11-16):** the percept fix, needed by every later phase — (1) an instrument first: per-group z energy
  and clipped count in `StanceFeatures`' diag (room / lift / load and contact / ground and relief / tilt); (2) a per-feature
  freeze from the normaliser's own dynamics (a component's statistics stay open until it has varied and settled; also the
  de-scaffold of the `freeze_after` count); (3) re-read the stairs' stance curve on the fixed encoder, as a reading, not a
  gate.
- **Gate (re-scoped 2026-11-16 to what an instrument can show):** the EPM bakes, stays bounded and saturates on the known
  worlds; its nodes separate the situations the instruments found decisive — a rear leg short of room, a front toe at its
  inner reach, the body nose-up on a riser, a hung toe — measured by NMI against instrument-derived labels used only for
  evaluation; the vocabulary is retained across a world change (return ties, reuse kept); the belief is consistent (NIS ≈ 1)
  in a familiar world and through injected FSR flicker; **and no feature is saturated on any bench** (the clipped count on
  the 38 and 64 mm risers near the corridor's). *Dropped:* convergence on the stairs. A vocabulary riding a frozen scaffold
  can only describe that scaffold's outcomes; whether the stairs can be learned is now a question for the closed loop
  (Phase 4), measured on outcome error, not stance QE.

### Phase 4 — The first closed loop: the landing offset by expected free energy (the first lever)
*Replaces the old Phase 4 (generator as an instrument) and Phase 5 (policy in shadow) as the next step. Why sooner: under
the frozen scaffold the step is a near-deterministic function of the stance, so an action-conditioned model trained on its
steps alone cannot separate what the body did from where it was — the old Phase 4 gate would fail for a harness reason. A
learned layer that varies its own action, and learns from the consequences, is the smallest thing that can build an
input–output mapping (operator, 2026-11-16: "with a truly learning brain in control, the robot should be able to build the
i/o mappings that work and build upon them").*
- **Instrument first.** `stepper_learning.py` (or a sibling) re-pointed at outcome error and behaviour: per repetition the
  touchdown error (`h_touch − h_pred`), the hand's correction magnitude (sum of `moves`), the hang rate, floor vetoes,
  progress per minute against the commanded heading, the stall share, falls and risers reached; slopes per doubling of
  practice as for QE. Read on the gain-0 chains before any arm.
- **The one value owned.** The forward/back offset δ of the scaffold's *planned* landing (Plan intents only; Choice and
  Unload intents untouched), along the body's commanded heading, over a small candidate set (about ±3 cm). The scaffold
  keeps the leg order, the body shift and therefore the forward progress, so "stand still" cannot score: the progress
  complement is the scaffold's in this phase. Why this value: the relocation tag (ledger 2026-11-14) finds the edge move
  and the obstacle reflex move 23–29 % of planned stairs landings by more than 2 cm — the scaffold's own rules already
  fight over this coordinate — and it is the operator's careful-stepper question (2026-11-06: *if I place my foot here, my
  FSRs and IMU will report this*).
- **The model.** Per (stance node, leg), a local RLS from [1, δ, the terrain at the candidate landing as `TerrainMemory`
  predicts it] to the step's outcome vector: touchdown error, the hand's correction, hang, (later) the load shares after the
  touch. Its targets are realised consequences (`step.outcome`, paired with the δ chosen at the decision by `seq`), never
  the scaffold's choice; δ = 0 is not a target (no teacher, CLAUDE.md §5.6). Graded by its own error and NIS.
- **The choice.** δ = argmin over the candidates the floor admits of risk − epistemic value: risk is the predicted
  divergence from the outcome priors (touchdown meets the predicted ground, no correction, no hang), scaled by those
  outcomes' own running spread, never a hand-set scale (CLAUDE.md §5.5); epistemic value is the model's predictive
  uncertainty at δ. While a (node, leg) is ungraded the epistemic term dominates and the choice is the jitter of §2.1;
  as it is graded, it exploits. No exploration during distress (floor veto, hang, tilt).
- **The floor.** Re-checks support and IK for the shifted landing (the pose search guaranteed support for its own landing,
  not for δ); a vetoed δ falls back to 0 and is counted.
- **Seams.** In-process, as Phase 2: a component between `ScaffoldPlanner` and `StepExecutor` at `intent_hook_`, editing
  `StepIntent.land`; `StepOutcome` already carries `h_pred`, `h_touch`, `hang`, `moves`. Its choice, predictions and
  grade in the diag. Persistence through the state carry, like the stance modules.
- **Gain-0-guarded:** at gain 0 the landing is the scaffold's, byte-identical on the golden traces (`phase1_pruned`) and
  on the config of record.
- **Where authority is earned, in order** (operator, 2026-11-16): (i) the corridor, as a no-harm gate — on flat ground the
  touchdown error barely depends on δ, so the corridor teaches little about risers, but the loop must not cost anything
  there; (ii) the launcher's graded flight (12 / 19 / 25 / 38 mm from 0.5 m), the first world where the mapping matters
  and the scaffold succeeds; (iii) the 38 and 64 mm benches, started from the carried state (`stepper_chain.sh FROM=`).
- **Gate (a signal: n = 6 chains, gain 0 against gain on, the full metric set):** (a) gain 0 byte-identical; (b) corridor:
  progress, falls, floor vetoes and stall share tie the scaffold and the outcome error does not rise; (c) graded flight: the
  outcome error falls with practice, loudly (a practice-curve slope readable at n = 6, not excavated), and a frozen-model
  evaluation after practice ties or beats gain 0 on risers reached and progress with lower outcome error; (d) the 38 / 64
  mm benches from the carried state, read and reported: does the outcome-error curve bend within the 23 minutes in which
  the instrument-only stance curve did not? A flat curve here does not block (b)–(c), but it is diagnosed (which value,
  which sense, the floor's vetoes) before Phase 5 widens anything; (e) the model is graded (NIS ≈ 1 on its outcome predictions); (f) observed in the UI before promotion.
  Blind-metric complements: a layer that lowers touchdown error by landing short on the tread must lose on risers reached
  and progress per step. **Kill** rather than power up if the effect only shows under heavy averaging (CLAUDE.md §3.3).

### Phase 5 — Widen authority, one value at a time
- The same mechanism takes the next value, each its own gain-guarded lever with its own A/B: the body shift before the
  lift (it touches support directly, so the floor matters more), then the landing's sideways offset, then the choice of
  leg (last: it removes the fixed crawl order, the imposed coordination topology, CLAUDE.md §5.7). The step model's action
  vector grows with each value earned; the room and progress priors enter the risk when the layer owns a value that can
  trade them (the shift and the leg choice — from then on the progress complement is the layer's, not the scaffold's).
- Before a value gets gain, one battery in shadow: the layer's choice and predicted outcome for that value logged beside
  the scaffold's, so no authority comes from an ungraded prediction (the old Phase 5 gate, per value). On-policy held-out
  steps against the analytic models (the drift instrument, `TerrainMemory`'s foothold error, the room after the step) are
  the grading criterion (the old Phase 4 gate, per value).
- **Gate per value:** Phase 4's form — no harm in the corridor, outcome error falling with practice on the graded flight,
  frozen evaluation at or above the previous stack on the battery's full metric set, fewer floor vetoes.

### Phase 6 — Whole-intent authority by precision
- Once the layer owns landing, shift and leg, it emits whole step intents (`StepPolicy`); `StepArbiter` on, gain-0-guarded:
  per stance node, the better-graded intent drives, the scaffold's or the layer's.
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
7. **Identifiability (2026-11-16).** A frozen policy's action is a function of its state, so an action-conditioned
   model learned from its steps alone cannot separate the action's effect from the state's. The reason Phase 4 varies its
   own action before any instrument-only gate on the step model.
8. **The dark room.** A learner in control that minimises its own prediction error can do so by acting so nothing
   surprising happens (standing, shuffling, landing only on flat tread). In Phase 4 the scaffold owns progress; from the
   first value that can trade progress (Phase 5's shift and leg choice) the progress prior must be in the layer's risk,
   and progress per step stays in every gate.
9. **The floor under a moved landing.** The pose search guarantees support for its own landing; any learned change to
   the step must be re-checked by the floor, and the floor's veto rate under the layer is reported (a floor that vetoes
   often is a performance cap, §1).

## 7. Working protocol (autonomous execution)

- One phase at a time, in order; each ends with its gate measured, a ledger entry and a commit (DCO-signed).
- **Stop and ask the operator** at: a failed gate that changes the plan; any deletion of scaffold code (Phase 1's
  list, before deleting); the first authority handover (Phase 4 since 2026-11-16: the operator approved the direction;
  ask again before the first gain-on battery) and each new value Phase 5 hands over; anything that touches the robot.
- Every behaviour change is gain-0-guarded and byte-identical when off; every refactor is golden-trace identical.
- Batteries never run while the operator is observing in the UI (they rotate the UI's log store); logs are gzipped
  on write.
- The resume point for each session is this file's §9 (the latest resume section), its §8 table and the ledger's latest entry.

## 8. Status

| Phase | State | Date | Ledger |
|---|---|---|---|
| 0 Freeze and baseline | **done** — gate met (15 of 15 runs byte-identical from the repo's tools) | 2026-11-14 | 2026-11-14 "Learned stepper Phase 0" |
| 1 Prune | **done** — one rule of 16 redundant (the IK re-seed, covered by multistart); off in the stack configs, its code marked `PRUNE-P1` and kept until the cleanup pass (operator, 2026-11-15) | 2026-11-14 | 2026-11-14 "Learned stepper Phase 1" |
| 2 Split | **done** — gate met: LegSolver and the planner / executor / floor split byte-identical (15 of 15 runs each), the outcome stream and the room field-identical; unit tests per component | 2026-11-15 | 2026-11-15 "Learned stepper Phase 2" |
| 3 Stance belief | **in flight** — built and measured; separation yes. Own-sample heads REGRESS against legacy; spares-baked stops the forgetting; growth traced to baking counting correlated samples. Curriculum (known worlds × 8, stairs × 6, return × 2): `bake_min_entries` 5 PARTIAL; a frozen stance encoder (`freeze_after`, scaffold count) is WORKING for retention (return ties pre-stairs TLE, reuse 0.60, 1 new node). Learning rate (`stepper_learning.py`): the known worlds learn (QE −7 %/doubling) and saturate in 6–8 min; the stairs do not learn at all in 7 min at either height. A long stairs block from the frozen brain (start-from-state, exact): no learning in 23 min (QE +0.2 %/doubling), the vocabulary grows and is slowly reused (0.25); not the stalls. The frozen flat-world encoder SATURATES on the stairs (ten terrain/tilt z clipped, |z| 12 vs 6). Entries under the still encoder: PARTIAL (60 % fewer stairs nodes, 3 % stairs fit cost). Gate re-scoped 2026-11-16 (no convergence on the stairs; no saturated feature on any bench). Next: the z-saturation instrument, then a per-feature freeze from the normaliser's own dynamics | 2026-11-16 | 2026-11-15 "Learned stepper Phase 3", "continued", "the settling normaliser", "entries to bake", "the frozen stance encoder"; 2026-11-16 "a long stairs block", "authority sooner" |
| 4 First closed loop (landing offset by EFE) | — direction approved 2026-11-16; the outcome-and-behaviour instrument is built and its gain-0 baselines read (§9.1); waits on Phase 3's gate, then the lever at gain 0, then the operator's go for the gain-on battery | | 2026-11-16 "authority sooner" |
| 5 Widen authority (shift, sideways, leg) | — | | |
| 6 Whole-intent authority by precision | — | | |
| 7 De-scaffold | — | | |

## 9. ▶ Resume here (2026-11-16, decided): authority sooner — the percept fix, then one learned value by expected free energy

**Read this section, §4 Phases 3–5, then the ledger's latest entries** (2026-11-16 "a long stairs block", "authority
sooner"). §10 keeps Phase 3's state and the question as it was put.

**The decision (operator, 2026-11-16).** The operator's idea — *an instrument-only learner riding a frozen controller can
only plateau at that controller's ceiling; with a learning brain in control, the robot can build the input–output mappings
that work and build on them* — is adopted, with four choices:
1. **The direction:** after the percept fix, one value of the scaffold's step is owned by a learned layer that chooses it
   by expected free energy — exploring while ungraded (this is §2.1's jitter), exploiting once graded. It replaces the old
   Phases 4–5 as the next lever; each further value is earned as its own lever (Phase 5).
2. **The value:** the landing's forward/back offset (about ±3 cm along the commanded heading) on the scaffold's planned
   steps. The scaffold keeps leg order, shift and progress.
3. **Where:** the corridor as a no-harm gate, then the launcher's graded flight (12 / 19 / 25 / 38 mm), then the 38 and 64 mm
   benches from the carried state.
4. **Phase 3:** the gate re-scoped to what an instrument can show (drop convergence on the stairs; add no saturated feature
   on any bench); the percept fixed first.

**The assessment behind it, in short.** "Plateau at the scaffold's ceiling" is true of behaviour by definition; it is not
what the stance-QE curves measure (an observer can predict a poor controller's outcomes well — its ceiling is the
scaffold's unpredictability given the percept). The stairs result of §10.2 is confounded twice: the percept is saturated
there, and the moving-normaliser control did not learn the stairs either. The stronger reason for authority sooner is
identifiability (§6.7): under a frozen scaffold the step is a function of the stance, so no step model can separate the
action's effect from the state's; the old Phase 4 gate would have failed for a harness reason. The risk the idea carries is
the dark room (§6.8), which is why the first value leaves progress to the scaffold.

**Next, in order (each stops for the operator where §7 says):**
1. Phase 3: the z-saturation instrument (per-group z energy and clipped count in `StanceFeatures`' diag), read on the
   `lb38` / `cvf` states and logs; then the per-feature freeze from the normaliser's own dynamics, gain-guarded (operator's
   approval of the form before the battery); then the re-scoped gate, measured.
2. Phase 4's instrument: practice curves of outcome error and behaviour (touchdown error, hand corrections, hang, floor
   vetoes, progress per minute, stall share, risers reached) on gain-0 chains.
3. Phase 4's lever built gain-0-guarded (byte-identity on `phase1_pruned` and the config of record), then the operator's go
   for the first gain-on battery.

Unchanged: the scaffold stays frozen; the `PRUNE-P1` cleanup with its byte-identity check is still pending; the `zz_arm_*`
and `sim60_stairs_seq2__p1*` configs stay uncommitted; godot is killed by PID only. Open: the one heap-corruption abort
(`lb38` c4 r4).

### 9.1 Progress since the decision (2026-11-16, evening)

**Three instruments, no authority** (ledger 2026-11-16 "three instruments"):

*Does the stance predict a step's consequence?* (`scripts_tools/stepper_predict.py <chain dir>...`, on the chains already
run.) Every landing is one sample: its outcome record joined to the stance winner on the first row after the previous
landing, when the scaffold planned it. Predictors are prequential (each step predicted from the steps before it in the
chain's own order, then learned from); skill is the share of squared error removed beyond a running per-leg mean within the
world. Controls: winner labels shuffled across steps never beat the leg mean (−0.01 to −0.17); the stepping leg's own
extension slack on the planning row, an input of the stance vector, is recovered from the latent at +0.45 to +0.50 on the
stairs.

| Question | Answer | Data |
|---|---|---|
| Does a per-(node, leg) table work as a step model's address? | **Not on the stairs as the vocabulary stood.** A step is planned from a (leg, node) cell seen three or more times before in 8 % of stairs steps (61–63 % in the corridor and arena): skill +0.03 to +0.13 on most outcomes | `lb38`, `cvf` |
| Is the information there when the address recurs? | **Yes.** In the cells with three or more earlier visits (about 50 steps a chain; a selected set, the repeated stances) the node removes 33 % of the landing offset's error, 22 % of the touchdown height's, 41 % of the duration's, 62 % of the room after the step, 72–74 % of the miss and hang scores | `lb38` |
| Does the stance latent generalise across nodes? | **Modestly at 38 mm, clearly at 64 mm.** Eight principal components of the winner's prototype, per-leg recursive least squares: 38 mm — offset +0.07 to +0.13, room +0.34, miss +0.22; 64 mm — miss +0.26 to +0.33, duration +0.24 to +0.30, hang +0.18 to +0.22, touchdown height +0.17, offset +0.15 to +0.17 (t 3–10 over 6 chains). Sensitive to regularisation: 16 components with a loose prior diverge | `lb38`, `cvf`, `cvfe` |
| Is the hand's landing relocation predictable? | **Weakly**: 2.2 cm rms at 38 mm, 13 % of its variance at best. It is mostly the obstacle reflex (0.65–0.70 cm of a 0.9 cm mean move), a reaction to a contact the swing did not expect | `lb38` |
| Is there anything to learn in the known worlds? | **Little**: the offset is 0.5 cm rms and nothing beats the leg mean; the touchdown error (0.32 cm after the leg mean) falls 8–14 % with the step's parameters | `cvf` |

**For Phase 4's model:** a per-(stance node, leg) local model alone starves on any world the body does not yet walk
repeatably. Its heads should read the stance **latent** (which generalises across nodes), with the node as a local
refinement where it has data — unless the widened encoder makes stairs stances recur, which the `cvw` battery reads. The
"step parameters" columns of this reader are confounded (the scaffold's landing is a function of the stance), the
identifiability argument of §6.7 with a number on it.

*Where does the frozen encoder saturate?* (`StanceFeatures`' diag: per feature kind the samples at the clip and the largest
|z| before it — `sf_sat`, `sf_zmax`, `sf_satn`; `scripts_tools/stepper_saturation.py`. Chain `sat0`: the `cvf` brain after
run 8 on 38 mm, 64 mm, corridor, arena × 6 chains; identical to `lb38` run 1 in all 5.1 million shared fields.)

| World | pitch | roll | ground | relief | knee | hip1 | hip2 | lift | ext | load |
|---|---|---|---|---|---|---|---|---|---|---|
| corridor | 0.6 % | 0.6 % | 0.0 % | 1.7 % | 0.3 % | 0.1 % | 0.2 % | 0.0 % | 0.0 % | 0.0 % |
| arena | 0.7 % | 0.7 % | 0.6 % | 1.3 % | 0.4 % | 0.3 % | 0.3 % | 0.0 % | 0.0 % | 0.0 % |
| 38 mm | 55 % | 35 % | 43 % | 15 % | 11 % | 2.3 % | 7.4 % | 2.0 % | 0.5 % | 1.5 % |
| 64 mm | 82 % | 7 % | 43 % | 21 % | 9 % | 11 % | 12 % | 8.9 % | 3.9 % | 0.1 % |

Not only the terrain and the tilt: the knee and hip room clip on about a tenth of riser samples (the knee peaks 21 flat-world
sd from its mean — the commanded knee well past straight). The flat worlds are clean but for brief tilt transients (peaks
11–19 sd). The re-scoped gate's last clause ("no feature saturated on any bench") fails on the frozen encoder by this table.

*Phase 4's baseline* (`scripts_tools/stepper_practice.py <chain dir>...`: practice curves of touchdown error, the hand's
correction, hang, miss, skipped turns, steps and progress per minute, stall share, falls, risers reached). On the gain-0
chains every curve is flat, as a frozen scaffold's must be: corridor — touchdown error 0.61 cm, correction 0.12 cm, stall
6 %, 59 cm/min; 38 mm — 1.3–1.8 cm, 0.9–1.6 cm, hang 1–5 %, stall 32–54 %, 19–32 cm/min; 64 mm — 2.5–3.1 cm, 1.4–1.8 cm,
hang 10–19 %, miss 12–20 %, stall 70 %, 13 cm/min, never all four toes on the first tread. Saved beside the chains
(`~/xaq_runs/stepper/{lb38,cvf}_practice.txt`); an arm is read against them.

**The per-feature rule, as approved (operator, 2026-11-16: "I approve the widen rule and form. We will get more insights
when I can observe the behavior in the UI").** `StanceFeatures.widen` (default 0, byte-identical: three stairs runs bytes-same
against `sat0`; 7 of 7 tests): once frozen, the centre stays; a feature kind whose sample reaches the clip on any leg has
its scale stretched, the four legs alike, by 1 / (its ticks at the clip so far + `warmup`) of itself. Self-limiting (it
stops when the excursion fits), so a kind's scale ends at its persistent range and holds; the step does not depend on the
excursion's size, so a glitch moves it half a percent. It does not remove the `freeze_after` count (still a scaffold).
Considered and not built: re-opening a clipped feature's full statistics until it settles (follows mean shifts, re-encodes
more, needs a settling criterion); one shared scale per physical unit (fixes ground and relief, not tilt or the knee).

## 10. Before the decision (2026-11-16): where Phase 3 stood, and the question of authority sooner

*Superseded as the resume point by §9, which records the decision.* The ledger's entries 2026-11-15 "the frozen stance
encoder" and 2026-11-16 "a long stairs block" are the evidence. The operator closed the 2026-11-16 session with an idea to
discuss in a fresh context, recorded at the end.

### 10.1 What is built (all instrument-only; the frozen `PoseSequencer` walks the body)

- `StanceFeatures` → stance EPM (`epm_stance`) → three `BeliefFilter` heads (own, legacy, visit-counted). 42 features: per
  leg room (5), lift, load share, contact, remembered ground and relief under the toe; body pitch and roll. Conditioned by a
  pure 1/n running normaliser (`n_max` 0) with `freeze_after` (a scaffold count, 60 000 samples in the curriculum).
- Persistence: the five stance modules carry across runs (`stepper_chain.sh KEEP=...`); `TerrainMemory` does not.
- GNG options: `health_death_spares_baked`, `bake_min_entries` (entries = winner changes).
- Harness: `stepper_chain.sh` (SCHEDULE for curricula, EXPECT asserts levers, FROM / FROM_RUN starts from a carried state;
  seeds 10c + FROM_RUN + k). Readers: `stepper_curriculum.py` (per run: nodes, baked, reuse, heads), `stepper_learning.py`
  (practice curves: slope per doubling, position-matched slope, last-rep change, return after a gap). Both cut a chain at its
  first broken link.
- Config of record: `sim60_stairs_seq2_belief__p3settle__p3vis__p3frz.json` (+`__p3entry` for entries to bake).

### 10.2 What is measured

| Question | Answer | Data |
|---|---|---|
| Does the vocabulary learn the known worlds? | Yes: QE −7 %/doubling (corridor), −4 % (arena); saturates in 6–8 min | `cvf` |
| Is it retained across a world change? | Yes, with the normaliser frozen: return ties, 60 % reuse, 1 new node | `cvf` vs `cvcb` |
| Does it learn the stairs? | **No**, at 38 or 64 mm, over 7 min (curriculum) or 23 min (long block): QE flat, vocabulary grows (+15–26 baked a run), 25 % cross-run reuse | `cvf`, `lb38` |
| Is that the stalls? | No: QE is flat in moving rows too (the body is stalled at a riser ~50 % of stairs rows) | `lb38` |
| Is the stairs' growth dwells? | ~60 % of it (entries to bake 5: 60 % fewer stairs nodes, +3 % stairs QE) — `PARTIAL` | `cvfe` |
| Is the percept sound on the stairs? | **No**: the frozen flat-world scale (ground sd 2 mm, relief 7 mm, tilt 1°) clips ~10 terrain/tilt z at ±4 on a riser (stance |z| 12 vs 6) | `lb38` state + logs |

Open: one heap-corruption abort in ~500 runs (`lb38` c4 r4). Pending cleanup (operator, 2026-11-15): delete the `PRUNE-P1`
code once the promoted module is final, then `stepper_ident.py --check stepper_golden.json --label phase1_pruned`.

### 10.3 The operator's idea (2026-11-16), to discuss before the next lever

> "If the predictor is not controlling the body, then it seems a plateau is inevitable. With a truly learning brain in
> control, the robot should be able to build the i/o mappings that work and build upon them."

**The case for it, from this phase's own data.** Every stance curve so far measures how repeatable the *scaffold's*
outcomes are, not what a brain can learn: the data distribution is set by a frozen policy that nothing learned feeds back
into. Where the scaffold succeeds (corridor, arena) the vocabulary saturates low within minutes; where it fails (the stairs,
half the time stalled, every attempt different) it plateaus high and no amount of practice moves it, because the learner can
only change its model, never its inputs. Active inference has two ways to reduce error, and Phase 3 has removed one of them.
An instrument-only learner's plateau is the scaffold's ceiling. This is CLAUDE.md's "LEARNED cooperates, IMPOSED fights" seen
from the other side: a learner riding on an imposed controller can only describe it.

**What the plan already says, and where the idea differs.** §1 is the closed-loop target (a step generator graded on the
realised outcomes of its own steps, a policy minimising expected free energy against homeostatic priors, the floor as a
reflex); §2 already gives the scaffold an "experience generator with epistemic jitter" role and a "fallback by precision"
role. The difference is ordering: the plan reaches authority only at Phase 6, after three instrument-only gates, and Phase
3's implicit gate (a vocabulary that converges on the stairs) may be unreachable while the scaffold drives — the same shape
as *an unreachable target winds a learner into a trap*.

**Points to settle in the discussion.**
1. **Re-scope Phase 3's gate** to what an instrument can show: bounded, separating (NMI against the instrument labels),
   retained, consistent on the known worlds — not converging on a world the scaffold cannot walk.
2. **Fix the percept regardless.** A controlling brain needs the riser state visible too; the saturation instrument
   (per-group z energy, clipped count) and a per-feature freeze (each feature's statistics stay open until it has varied)
   are cheap and needed by every path.
3. **The smallest closed loop.** Candidates: (a) the learned layer owns one continuous step parameter (landing offset or body
   shift) within the scaffold's step, graded by its own touchdown/load prediction error — authority over one degree of
   freedom, the scaffold keeping order and the floor; (b) epistemic jitter first (§2.1), so the data is no longer the
   scaffold's alone, then (a); (c) Phases 4–6 merged on-policy: the generator learns from the steps it chose.
4. **Where authority is earned.** In the known worlds first (where mappings that work exist to build on — the operator's
   2026-11-15 "build a performant brain in the corridor or arena, then test it on the stairs"), then carried to the stairs
   with start-from-state.
5. **Safety and the bar.** The floor stays a reflex (*fatal events have no gradient*); authority by precision per stance
   node keeps the scaffold as fallback; gain-0-guarded; the learning-rate tool re-pointed at *behaviour and outcome error*
   (progress against the command, floor vetoes, touchdown error), not only the stance QE. The no-teacher rule holds: the
   learned layer learns consequences of steps, never the scaffold's choices.
