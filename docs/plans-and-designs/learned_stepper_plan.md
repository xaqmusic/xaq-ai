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
| 3 Stance belief | **in flight** — built and measured; separation yes. Own-sample heads REGRESS against legacy; spares-baked stops the forgetting; growth traced to baking counting correlated samples. Curriculum (known worlds × 8, stairs × 6, return × 2): `bake_min_entries` 5 PARTIAL; a frozen stance encoder (`freeze_after`, scaffold count) is WORKING for retention (return ties pre-stairs TLE, reuse 0.60, 1 new node). Learning rate (`stepper_learning.py`): the known worlds learn (QE −7 %/doubling) and saturate in 6–8 min; the stairs do not learn at all in 7 min at either height. A long stairs block from the frozen brain (start-from-state, exact): no learning in 23 min (QE +0.2 %/doubling), the vocabulary grows and is slowly reused (0.25); not the stalls. The frozen flat-world encoder SATURATES on the stairs (ten terrain/tilt z clipped, |z| 12 vs 6). Entries under the still encoder: PARTIAL (60 % fewer stairs nodes, 3 % stairs fit cost). Gate re-scoped 2026-11-16 (no convergence on the stairs; no saturated feature on any bench). The saturation instrument: the knee and hips clip too. `widen` (the approved per-feature rule): WORKING for saturation (≤ 0.5 % at the clip on every bench) and for retention after one re-encoding of the known worlds (second return: reuse 0.74 / 0.62); stairs stances still do not recur (7–9 %). Left: NMI and flicker re-read under the widened encoder | 2026-11-16 | 2026-11-15 "Learned stepper Phase 3", "continued", "the settling normaliser", "entries to bake", "the frozen stance encoder"; 2026-11-16 "a long stairs block", "authority sooner", "three instruments", "the widening scale" |
| 4 First closed loop (landing offset by EFE) | **first form REGRESSION in context** — `stepper/LandingOffset` at the intent hook (`lo_gain`, `lo_learn`); byte-identical off; shadow mode; the model reads δ and the remembered terrain at the candidate, per leg (§9.1). With authority (§9.2): the floor admits backward offsets only; corridor progress −19 % (t −3.8, n = 6 chains), graded flight −16 % (n = 5), no outcome gained; cause: the value sits at the scaffold's limit, the outcome vector has no entry for progress, and a standing −0.43 cm touchdown bias feeds the choice noise. Stays at gain 0. **Waits on the operator: the next owned value** (lift height recommended) | 2026-11-16 | 2026-11-16 "authority sooner", "three instruments", "the landing offset", "the first battery with authority" |
| 5 Widen authority (shift, sideways, leg) | — | | |
| 6 Whole-intent authority by precision | — | | |
| 7 De-scaffold | — | | |

## 9. ▶ Resume here (2026-11-16, decided): authority sooner — the percept fix, then one learned value by expected free energy

**Read this section, §4 Phases 3–5, then the ledger's latest entries** (2026-11-16 "a long stairs block", "authority
sooner"). §10 keeps Phase 3's state and the question as it was put.

**Where it stands (2026-11-17, evening).** The operator's word on §9.4: "go straight to the shadow build. I don't know what
levers will bear fruit so lets investigate." Built and measured the same day (§9.5): the **after-touch record**
(`AfterTouch`, `at_learn`), in shadow on the stack of record, graded; and what it found — the re-stepping follows landings
the executor moved off their plan (the lowering retreat: the same leg steps again after 97 % of them), the plan's own
prediction that the next leg will be liftable fails on 29 % of steps, and three scaffold arms aimed at it leave the stall in
place (two regress, the crouch ties). **The next act is the operator's choice (§9.5, "The proposal"): a learned choice at
the decisions where the wave's turn is skipped.** Nothing with learned authority has been run.

**Where it stood (2026-11-17, afternoon).** The operator watched the pair for 25 minutes of rough terrain in the outer field
(74 960 ticks, no fall, no reset) and **promoted it**: the stack of record is now
`sim60_stairs_seq2_belief__p3settle__p3vis__p3frz__kgpm.json` (the frozen sequencer + `ik_knee_guard` 1 +
`plan_model_margin` 1); `…__p3frz.json` is the pre-promotion control. The one failure they saw: past about 10° of chassis
tilt the stepper "could not figure out the next step" and a front toe re-stepped in place (§9.4). **The next act is a
discussion with the operator: which value the learned layer owns next** — §9.4 changes §9.2's order.

**Where it stood (2026-11-17, midday).** Before the next owned value, the operator's priority was the freeze (§9.3): it was the
simulated body asleep — fixed in the body script — and every stairs number in this plan from before 2026-11-17 includes runs
that were dead from some tick on. Two levers came out of the same day, both at 0 in the configs of record: `ik_knee_guard`
and `plan_model_margin`. **The next acts are the operator's:** watch the pair (`…__p4lo__kgpm`, or the `[M]` panel's two
switches), decide whether they join the config of record, and then the choice of the next owned value (§9.2) — which should
be measured on the corrected simulator, with Phase 3's and Phase 4's stairs numbers re-read there first.

**Where it stood (2026-11-16, night).** Of "Next, in order" below: item 1 is done but for two re-reads (the widened encoder
clears the clip and retains after one re-encoding; NMI and flicker not yet re-read under it), item 2 is done, item 3 is built,
byte-identical off, and **measured with authority (2026-11-16, late; §9.2): the corridor's no-harm gate is failed** — the
landing offset, backward only under the floor, costs 19 % of the corridor's progress and 16 % of the graded flight's, and buys
nothing its outcomes can show. **The next act is the operator's: which value the layer owns next** (§9.2 has the three
candidates; the lift height is recommended).

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

**The widen arm, measured (ledger 2026-11-16 "the widening scale").** `cvw` (the frozen `cvf` brain after run 8, then 38 mm × 3,
64 mm × 3, corridor, arena; paired by seed with `cvf` runs 9–16; the body field-for-field identical to the control) and `cvw2`
(the same arm continued on new seeds: 38 mm, 64 mm, corridor, arena).

| Question | Answer | Data |
|---|---|---|
| Is any feature still saturated? | **No.** At the clip, frozen → widened: pitch 55 → 3 % (38 mm), 82 → 1 % (64 mm); ground 43 → under 1 %; knee 11 → under 1 %; every kind ≤ 0.5 % on every bench in the second pass. The scales grew × 1.7 to × 7 and then held (0–12 % further on new stairs seeds, none in the flat worlds) | `sat0`, `cvw`, `cvw2` |
| Is the known-world vocabulary retained? | **After one re-encoding.** First return: reuse 0.02 (frozen 0.60), 34 new corridor nodes — the rescale strands the old nodes. Second return, the scale settled: reuse 0.74 corridor / 0.62 arena, 4 and 9 new nodes | `cvw`, `cvw2` |
| Do stairs stances recur now? | **No.** A step planned from a (leg, node) cell seen three or more times: 7 % and 9 %, against 7 % and 8 % frozen, on the same steps. Saturation was not the cause | `cvw` vs `cvf` |
| What did the stance gain? | The tilt: the latent's read of pitch +0.22 → +0.37 (38 mm), of the miss +0.26 → +0.45 (64 mm). The landing offset is no more predictable | `cvw` vs `cvf` |

`widen` is `WORKING` for the gate's saturation and retention clauses (a signal, n = 6 chains), at the cost of one re-encoding of
the known worlds. Still to re-read under it before Phase 3 closes: the NMI separation and the belief through flicker. If the
one-off re-encoding must be avoided (the robot), the re-use form is a scale that widens only beyond the clip.

**Phase 4's lever, built at gain 0** (ledger 2026-11-16 "the landing offset"). `stepper/LandingOffset` sits at the intent hook
between `ScaffoldPlanner` and `StepExecutor`; `PoseSequencer` params `lo_gain` (authority: the offsets on offer span
`lo_gain` × `lo_max_m`, read at every decision), `lo_learn` (the model learns and reports; with `lo_gain` 0 it runs in shadow
on the scaffold's own landings), `lo_max_m` 0.03, `lo_cell_m` 0.01. Both at 0 (the default) it is never consulted.

- *The model, as built — one change from §4's wording.* Per **leg**, a recursive least squares from what varies across the
  candidates — δ, δ², and the remembered terrain at the candidate (its rise over the ground under the toe now, the relief
  around it, the memory's uncertainty there, whether the memory knows it) — to the outcome vector [touchdown error, the hand's
  correction, hang, miss]. Not per (stance node, leg): the widen arm shows stance cells do not recur on the stairs even
  unsaturated, and a term that is the same for every candidate (the stance alone) cancels in the choice. The stance enters
  where it can change a choice, as interactions with δ — its own lever after this one, reading the latent.
- *The choice.* G = Σ ŷ²/(2 s²) − (K/2) ln(1 + φᵀPφ) over the candidates the floor admits; s² each outcome's own running mean
  square; ties to the smaller offset. In distress (the last step hung or missed, the search skipped a leg, the body tilted
  half-way to the disengage) the epistemic term is dropped.
- *The floor.* The scaffold's landing is always a candidate; a shifted one must pass the pose search's own landing checks
  (`ScaffoldPlanner::landing_ok`, a query added for this) and leave the next leg no less liftable than the scaffold's did.
- *Carried and logged.* The model rides `PoseSequencer`'s snapshot only while the layer is on (add `pose_sequencer` to `KEEP`);
  the log carries each decision, each update, the counts, the chosen offsets and the model's grade (`ps_lo`, `ps_lou`,
  `ps_lon`, `ps_loh`, `ps_los`); `stepper_practice.py` reads them beside the outcome and behaviour curves.
  `stepper_chain.sh` has a `grad` bench (the launcher's graded flight).
- *Identity.* Off (both 0): the golden battery (15 runs) bytes the same as the post-split traces (`p2d`), the config of record bytes
  the same on three stairs chains (`idl3` against `sat0`). Shadow (`ids3`): 0 of 2.59 million shared fields differ, only the
  new keys added. A first shadow reading, three chains × one 38 mm run, about 85 steps each (a reading, not a result): 93 % of
  planned decisions paired with their landing, 11 % made in distress; from the terrain at the scaffold's own landing the
  model removes 19 ± 8 % of the touchdown error's mean square and 23 ± 11 % of the correction's; its grade is 1.6 and 1.3.
- *The floor on a smoke run* (the authority config, 3 000 ticks on one 38 mm riser and one corridor, read for the floor's
  counts only): **every forward candidate is refused by the pose search's own landing checks** (117 of 117, 84 of 84). The
  scaffold's search rewards advance, so its landing already sits at the forward edge of what the floor admits: as specified, the
  offsets on offer are backward only. Refusals are counted by check and side (`ps_lon` entries 8–11).
- *Configs.* `…__p3frz__p4shadow` (`lo_learn` 1) and `…__p3frz__p4lo` (`lo_learn` 1, `lo_gain` 1; named for the launcher).

**Stops here for the operator (§9 item 3):** the first gain-on battery — and, before it, one question the smoke run raises:
the authority is one-sided (back only). Run it as specified (the corridor gate will price a layer that can only shorten
steps), or first move the owned value to one the scaffold has not pushed to its limit (the lateral offset; the lift
height)? Proposed if run as specified, in the plan's order — (i) corridor, no harm:
6 chains × 4 corridor runs, `p4lo` against `p4shadow` paired by seed, progress / stall / vetoes / falls tie and the outcome
error does not rise; (ii) the graded flight: 6 chains × 6 `grad` runs, the model carried, practice curves of outcome error,
then a frozen-model evaluation; (iii) 38 and 64 mm from the carried state. Also for the operator: `widen` as the encoder of
record under these arms (they are built on the frozen `p3frz` config, since the first model does not read the stance).

### 9.2 The first battery with authority: the corridor gate (2026-11-16, late)

The operator's go: "Continue with option 1" — run the landing offset as specified, the corridor first. Ledger 2026-11-16
"the first battery with authority".

**The battery.** `p4clo` (`lo_gain` 1) against `p4csh` (shadow: the scaffold's body, the model learning), six chains × four
corridor runs, cold start, the model carried, paired by seed; 48 of 48 runs through every gate. Reader: `stepper_pair.py`
(arm minus control per paired run; the chain is the unit).

| | authority | shadow (control) | difference ± sd over chains | t (n = 6) |
|---|---|---|---|---|
| forward progress, cm/min | 47.9 | 59.2 | −11.3 ± 7.2 (−19 %) | −3.8 |
| progress per landing, cm | 1.30 | 1.71 | −0.41 ± 0.25 | −4.0 |
| rows stalled, % | 20 | 7 | +13 ± 9 | +3.5 |
| landings per minute | 37.1 | 34.6 | +2.5 ± 1.8 | +3.5 |
| skipped turns per 100 steps | 1.8 | 0.4 | +1.4 ± 2.0 | +1.7 |
| touchdown error, cm | 0.60 | 0.60 | −0.01 ± 0.06 | −0.3 |
| the hand's correction, cm | 0.14 | 0.13 | +0.01 ± 0.05 | +0.3 |
| hang, miss, falls | 0, 0, 0 | 0, 0, 0 | | |

**The no-harm gate is failed** (`REGRESSION` in context). The arm lost progress in 22 of 24 paired runs; the loss follows
the dose (r = −0.91 between a run's mean offset and its lost progress, 11 cm/min per cm) and does not fade over the four
repetitions. Half the planned landings moved, in two peaks: 49 % at the scaffold's landing, 35 % at the full 3 cm back.
The floor refused 99.5 % of forward candidates, so the authority was backward only, as the smoke run said.

**Why — three separable faults, none of them the choice rule.**
1. *The owned value sits at the scaffold's limit.* Every admissible deviation shortens the step, and the landing's reach is
   where the next shift's room comes from: a shorter landing is spent as progress. §4's argument that the scaffold's shift
   keeps progress safe is refuted.
2. *The outcome vector has no entry that progress reduces.* Touchdown error, correction, hang and miss are all indifferent
   to a short step on flat ground, so the layer cannot see what it spends.
3. *A prior at zero on an outcome with a standing offset.* Every leg model in both arms carries a touchdown intercept of
   −0.43 ± 0.06 cm on the flat floor (the toe stops 4 mm below the memory's ground). No offset removes it; it is the largest
   term of the risk; and the argmin then follows each leg's fitted slope of noise about it — 9 of 24 leg models settle at the
   back limit and stay. To be traced as an instrument fault before a model is graded on touchdown error about zero.

**What carries forward.** The machinery (the outcome model, the choice, the floor, shadow mode, the carried state, the paired
reader) worked as built; what failed is the choice of value and of outcome set. Three rules for the next owned value, now in
the ledger as principles: list what the value spends and check each cost is a predicted outcome; own a value with room on
both sides of the scaffold's choice; judge an outcome about its own running level where it has a standing bias.

**For the operator — the next form.** Candidates, in the order I would take them:
- *The lift height* (recommended): not at a limit; its cost is the step's duration, which the outcome record already carries
  (`ticks`), and its benefit is the hang and the miss, which the stairs produce and the corridor does not. Both sides of the
  trade are observable by the layer.
- *The landing offset again, with the cost in the vector*: add the step's realised advance against the plan's own as an
  outcome. The floor stays one-sided, so the best it can do on flat ground is learn to choose 0.
- *The lateral offset*: two-sided, but its cost is the next leg's support margin, which arrives a step late.

**The graded flight, as a diagnostic** (not a promotion step: does a backward offset remove hangs or misses where the
scaffold has them?). `p4glo` against `p4gsh`, six chains × six `grad` runs, each arm's model carried on from its corridor
runs; chain 4 of the arm aborted in its first run (the open heap fault, below) and is left out with its pair, so n = 5.

| | authority | shadow (control) | difference ± sd over chains | t (n = 5) |
|---|---|---|---|---|
| forward progress, cm/min | 35.1 | 41.8 | −6.7 ± 4.8 (−16 %) | −3.2 |
| progress per landing, cm | 0.95 | 1.22 | −0.28 ± 0.17 | −3.8 |
| rows stalled, % | 31 | 19 | +12 ± 8 | +3.4 |
| highest toe on the flight, cm | 3.2 | 4.4 | −1.2 ± 0.9 | −3.0 |
| hang, % of steps | 0.6 | 0.9 | −0.3 ± 1.6 | −0.4 |
| touchdown error, cm | 0.85 | 0.93 | −0.08 ± 0.14 | −1.3 |
| the hand's correction, cm | 0.45 | 0.75 | −0.29 ± 0.22 | −3.0 |

The corridor's reading holds on the stairs. The one gain, the hand's correction, is all in the move off an obstacle (5.9
against 11.5 % of steps) and is confounded: the arm climbs less, and a body that meets fewer riser faces has fewer landings
to move off them. Hang and touchdown error are not separated, and the model's skill on its outcomes is no higher with
authority than in shadow. No evidence, at this power, that a backward offset pays where the scaffold has errors.

**State.** The landing offset stays in the build at gain 0 (`REGRESSION` in the context above; re-use context in the ledger).
The 38 and 64 mm benches of the proposed order were not run: the gate before them is failed. Open, and now seen twice (`lb38`
c4 r4; `p4glo` c4 r1): a heap-corruption abort, "corrupted size vs. prev_size", both on stairs worlds, both in chain 4
(inspector port 8204), the first before this lever existed.

### 9.3 The freeze (2026-11-17): a sleeping body, a knee past straight, and a plan its own gate refuses

Ledger 2026-11-17 has the measurements; this is what it changes for the plan.

**What the freeze was.** The physics engine put the body to sleep after half a second of stillness, and a sleeping body
ignores its joint motors. The careful stepper stands still while it decides; the fast gait never did. The signature (every
real joint exactly constant while the commands move) is in 6 of the 15 golden traces and in every stairs battery of Phases
1–4. The body script now keeps the powered body awake (`OGMA_PICRAWLER_ALLOW_SLEEP=1` restores the old behaviour); a run that
never slept is identical with the fix, field for field.

**What this does to the record.** The golden traces (Phase 0's gate) remain valid as a byte-identity reference *only with
sleep allowed*; a refactor gate run on the corrected simulator needs new golden traces. Phase 3's and Phase 4's stairs
measurements — progress, frozen windows, hang and miss shares, the step-outcome stream the landing-offset model learned from
— include dead runs (on the 64 mm bench the old simulator's hang share reads 16.0 % against 2.4 % awake, on the same seeds).
The corridor numbers are unaffected (no corridor run slept), so §9.2's corridor verdict stands; its graded-flight numbers
(3 of 72 runs asleep) should be re-read before they are quoted.

**Two levers, built at 0.**

| lever | what it does | measured (n = 6, signals) |
|---|---|---|
| `ik_knee_guard` | a solve that ends with a knee past straight is discarded; the leg is reset to its last pose with the knee turned to the bent side and solved again (the operator's rule) | knees past straight 4.1 % of rows → 0; alone it meets the re-plan loop more often (2 loops, 46 s, in 6 runs) |
| `plan_model_margin` | the pose search lowers its own lift margin for a leg by however much the support model's margin sits below it, so it does not re-issue a zero-shift plan the lift check refuses | no re-plan loop in 12 runs with it |
| both | | terrain: 2 freezes against 3, none a loop or a hyperextension; corridor identical to the control; 38 mm +9.9 cm/min (t +2.3); graded flight +7.9 cm/min (t +2.4); 64 mm mixed (no falls, but more missed landings and stall) |

**Open, in order.** (1) ~~The operator's observation of the pair in the UI~~ done 2026-11-17: promoted (§9.4). (2) Each lever singly at 64 mm (the cost there is
not attributed). (3) New golden traces on the corrected simulator, then the Phase 3 re-reads and the Phase 4 graded-flight
re-read. (4) The host's heap abort (three times in about 700 runs) under a sanitizer build. (5) Why the hang share rises
under the guard.

### 9.4 The pair promoted; what is left is the plan on a tilted body (2026-11-17, afternoon)

**Promotion.** The operator ran `…__p4lo__kgpm` for 25 minutes of rough terrain in the outer field of the terrain arena
(seed 42, 74 960 ticks: no fall, no reset, 100 % upright) and promoted the pair. The stack of record is
`sim60_stairs_seq2_belief__p3settle__p3vis__p3frz__kgpm.json`; it is the control for every lever from here on, and
`…__p3frz.json` stays as the pre-promotion control. The parameters' defaults in the code stay 0. New golden traces for the
refactor gate are owed on this config and the corrected simulator (§9.3 item 3).

**The operator's observation.** "It wasn't until the chassis was forced to exceed 10 degrees that the robot could not figure
out the next step (front right toe tapping in place)."

**What the terrain logs already hold** (`frz5/a_kgpm`: the same config, six 3-minute runs; the UI keeps no per-tick log, so
the operator's own run cannot be read). Reader: one leg landing three or more times in a row.

- It is there, and it is a front leg every time (FL 3, FR 4 episodes): 2–14 s each, 4 % of engaged time, the body moving
  under 2.5 cm. Through each episode the wave's own leg is skipped for want of a lift margin, and a later leg steps without
  taking the wave's turn — so the same leg is chosen again.
- The planner's block instrument names the constraint: in 90 % of the blocked plans inside an episode, the smallest body
  shift that would give the skipped leg its margin takes **another planted toe inside its minimum reach** (60 mm; that toe
  sits at a median 65 mm from its hip — 82 mm with the reader's hip coordinates corrected, §9.5). That toe is one of the two highest of the four 78 % of the time (50 % by chance).
- Blocked plans rise with tilt: 3.2 a minute below 5°, 7.4 at 5–10°, 10.0 at 10–15°, 26 above 15°. At 10–15° nearly half
  (47 %) are a shift that exists by the planner's geometry and is refused by the support model's margin.
- No episode in these runs outlasted 14 s, and the share of time spent re-stepping is flat across the tilt bins (4–5 %): the
  operator's lasting case is not reproduced yet (six 10-minute runs along their path are the next read: `tilt1`).

**The reading (a hypothesis, to be tested by dose-response).** The stance's levelling is proportional only (`level_gain`
0.5, each toe moved at most `level_max_m` 2 cm), so about two thirds of the ground's slope stays in the body. A tilted body
carries its centre of mass downhill of the toes by its height times the tangent of the tilt — about 14 mm at 10°, against a
lift margin of 10 mm asked. The shift that would answer it is uphill, toward toes that already stand close under their
hips. The swing is not where the stepper fails: on the corrected simulator the terrain runs record no missed landing and no
hang in 18 minutes, and the stairs benches 1–3 % of steps each.

**The dose-response, same day (`tilt2`, n = 6 × 3 min, signals; ledger).** Both constants the reading pointed at were
moved by hand on the terrain battery's seeds, and both lengthen the re-stepping: `level_gain` 1.0 gives 256 s of it against
42 s (t +2.5), with episodes up to 63 s at 3–9° of tilt; `land_r_min_m` 0.09 gives 148 s (t +2.1). Distance is not separated
in either. The hypothesis in the form "level harder and the block goes" is refuted: levelling spends the legs' reach, and the
block is a reach limit. What binds is how the body comes to stand over its planted toes — the blocking toe sits 63–65 mm (68–82 mm corrected, §9.5)
from its hip in every arm, wherever the landings are kept — and no single constant moved globally improves it.

**The operator's stall reproduced (`tilt1`, six 10-minute runs along their path; `stepper_tilt.py`).** Re-stepping takes
11–12 % of the time below 10° of chassis tilt and 44–51 % above it, and two episodes last five minutes (a front-left toe,
121 and 225 landings, the body at 15° and 11°, moving 0.5 and 1.4 cm). In them the body stands on three legs; the fourth
toe carries 1–3 % of the weight and its contact flag is on 3–8 % of the time. Its touch is counted a landing, it is the
only leg with a lift margin, and it steps again; the leg whose turn it is cannot be freed, because the shift would take a
bearing toe inside its minimum reach. Every counter reads a normal step: no hang, no miss, no timeout. It is the knot of
2026-11-08 (the unloaded toe re-stepped on the approach), where three local rules failed; with today's two constants, five
imposed forms have now failed at it.

**What it says about the learned layer.** The stepper's record of a step ends at the first contact. The prediction that
fails here is one it does not make — *the toe I have just placed will carry its share* — which is the operator's
2026-11-06 sentence ("if I place my foot here, my FSRs and IMU will report this") and the plan's own deferred outcome
("(later) the load shares after the touch"). The first addition is therefore to the outcome vector, in shadow: whether the
landed toe bears a second after the touch, the body's tilt then, and whether the next leg in turn has a feasible lift. The
value proposed to the operator for the first closed loop on it: what the toe does after first contact (how far it presses
before the step is called landed). Its costs are the tilt, the other toes' load and the leg's remaining extension, all
observed in the same step. Caveat carried from the robot: the FSRs are trusted as thresholds, not magnitudes, so "bears" is
a threshold outcome, not a share regression.

**What it does to §9.2's order.** The lift height's benefit was the hang and the miss; with the dead runs out of the record
those are small. The error left is in the stance — whether the next leg can be lifted — and the outcome vector of Phase 4
has no entry for it (nor for the body's tilt or the load on the other feet after the touch). §9.2's first rule applies
again: the outcome set comes before the value. The candidates and the order are the operator's discussion (2026-11-17).

### 9.5 The after-touch record in shadow, and what it found (2026-11-17, evening)

**Built.** `stepper/AfterTouch.{hpp,cpp}`, switched by `PoseSequencer at_learn` (0 = off, byte-identical). A record per
step, opened at its outcome and closed at the next: the **gap** (the height the stance gives the toe on the tick of its
touch, over the height it touched at), the touch height the executor expected at the go, the contact and load over the
toe's planted ticks, the leg that goes next, whether the search that chose it skipped the wave's turn, the tilt at the
touch and the close. A recursive least squares per leg and per outcome predicts at the touch [the toe will not bear, the
same leg lifts next, the tilt's change, the wave's turn is skipped] from [1, gap, its positive part, tilt, the toe's load];
each outcome keeps its own posterior because a record cannot always give every outcome (a toe lifted again at once has no
bearing to read). It reads and moves nothing — the body with and without it is identical row for row. Config
`…__p3frz__kgpm__aft.json` (in the launcher); reader `scripts_tools/stepper_after.py`; ledger 2026-11-17 (evening).

**The shadow battery** (the stack of record, the six spawns and seeds of `tilt1`, 6 × 10 min, 1615 records). 22 % of
readable landings do not bear; the same leg lifts next after 36 %. The model's skill against each leg's own base rate is
+26 / +34 / +23 / +23 % on the four outcomes, NIS 1.10–1.29. The gate of Phase 5's shadow step (no authority from an
ungraded prediction) is met for these outcomes on this terrain.

**What it found.**

- **The re-stepping is a landing the plan did not get.** 45 % of planned landings end more than 2 cm from their plan (65 %
  at 10–15°). After a lowering retreat of more than 1 cm — the executor walking the landing toward the hip because the toe
  cannot reach the ground where it was sent — **the same leg steps again 97 % of the time** (346 landings), against 16 %
  when no rule moved the landing. Those are front legs on a tilted body, the toe meeting the ground a median 43 mm below
  the ground the executor expected; the memory's height there had been refused by a fixed band (`land_band_lo_m`, 2 cm).
- **The plan's prediction of its own step fails on 29 % of steps** (the drift instrument: predicted the next leg liftable,
  found it not), and in 83 % of those the landed toe alone breaks it — a mean 51 mm from its planned place.
- **A plan made right still stalls.** With `land_band_lo_m` 0.04 the retreat disappears and the prediction fails 5 % of the
  time, and the walk is worse (0.31 m against 1.20 m): the plan now knows its step frees nothing and takes it, because it
  is the only feasible one. Keeping each toe's landed height in the stance is worse again (the plane stance is what holds
  the body parallel to its footholds). The crouch (`crouch_max_m` 0.03) removes 86 % of the retreats, evens the legs' step
  counts and ties on distance and re-stepping (`PARTIAL`).
- **Corrections to §9.4.** In a re-step the fourth toe has almost no stance to be unloaded in; the press after first
  contact proposed there is weakened (its dose-response, the landed-height stance, cost attitude). The blocking toe's
  radius was read with wrong hip coordinates: 68–82 mm, not 56–65.

**The proposal (the operator's decision).** Eight imposed rules or constants have been tried at this knot; each closes one
way into the stall and leaves the stall. What none of them supplies is a choice at the decision where the scaffold has no
good answer. So the value proposed for the first closed loop is not a coordinate of the step but **the step intent at the
decisions where the wave's turn is skipped** (43 % of decisions on this terrain; how rare it is on the corridor is the first thing
to measure, since the layer costs nothing where it never acts): the scaffold enumerates the intents its floor admits there — each other leg's landings, and a
body shift without a step — and the layer chooses among them by expected free energy over the after-touch outcomes (the
wave's turn skipped again, the same leg again, the tilt's change) plus Phase 4's (the hand's correction, the touchdown
error). The gate is the state the scaffold fails in (CLAUDE.md §1 rule 4), the scaffold's choice stays a candidate, and
everywhere else the scaffold walks. This is Phase 5's shift and leg choice taken together, but only where the fixed order
has already broken. Before it gets gain: one shadow battery with the candidates and their predicted outcomes logged beside
the scaffold's choice. Alternatives the operator may prefer: the crouch promoted as a scaffold lever after a look in the
UI; or the landing's depth belief (the band) made the memory's own uncertainty instead of a constant.

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
- Config of record: `sim60_stairs_seq2_belief__p3settle__p3vis__p3frz.json` (+`__p3entry` for entries to bake). *Since 2026-11-17: `…__p3frz__kgpm.json` (§9.4).*

### 10.2 What is measured

| Question | Answer | Data |
|---|---|---|
| Does the vocabulary learn the known worlds? | Yes: QE −7 %/doubling (corridor), −4 % (arena); saturates in 6–8 min | `cvf` |
| Is it retained across a world change? | Yes, with the normaliser frozen: return ties, 60 % reuse, 1 new node | `cvf` vs `cvcb` |
| Does it learn the stairs? | **No**, at 38 or 64 mm, over 7 min (curriculum) or 23 min (long block): QE flat, vocabulary grows (+15–26 baked a run), 25 % cross-run reuse | `cvf`, `lb38` |
| Is that the stalls? | No: QE is flat in moving rows too (the body is stalled at a riser ~50 % of stairs rows) | `lb38` |
| Is the stairs' growth dwells? | ~60 % of it (entries to bake 5: 60 % fewer stairs nodes, +3 % stairs QE) — `PARTIAL` | `cvfe` |
| Is the percept sound on the stairs? | **No**: the frozen flat-world scale (ground sd 2 mm, relief 7 mm, tilt 1°) clips ~10 terrain/tilt z at ±4 on a riser (stance |z| 12 vs 6) | `lb38` state + logs |

Open: a heap-corruption abort, twice in ~630 runs (`lb38` c4 r4; `p4glo` c4 r1 — §9.2). Pending cleanup (operator, 2026-11-15): delete the `PRUNE-P1`
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
