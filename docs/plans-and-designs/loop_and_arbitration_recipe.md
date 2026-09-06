# The loop and arbitration recipe — as built, 2026-09-06

*The one canonical statement of what an active-inference loop is in this project and how
loops are arbitrated, written from the code that runs rather than from the designs that
preceded it. Where the code and the doctrine disagree, this document says so and points at
the [open-items register](open_items_register.md) entry that owns the disagreement. Where a
loop's field is non-compliant with [`CLAUDE.md`](../../CLAUDE.md), it is named, not hidden.
The evidence for each statement is a row in the
[audit register](../reports/cell_system_audit_2026-09_appendix.md).*

**Supersedes, in part:** `cell_efe_arbiter_plan.md` §"EFE formulation" (describes the
`value_race` path no Cell config runs), `cell_nav_strange_loops_plan.md` §"EFE ARBITER" and
its PLAY floor ("when hunger is rising"), `cell_play_loop_plan.md` §5 (the three-way ledger
as designed; §2's shared map was never built). Those documents keep their design rationale;
this one states what is.

---

## 1. The loop unit

A loop is a closed path from a group of sensors to a heading and back:

```
inputs → its own prediction and error → a heading generator
       → (heading, confidence) on percept.<loop>
       → its own HeadingController → one MotorBus channel, gated by arbiter.gain.<loop>
```

Each loop owns its heading controller (a learned advance under a UCB policy, effort-costed)
and one motor-bus channel. The arbiter never touches a loop's internals; it sets the
channel's gain, which the bus uses for both the mix and the loop's learning authority, so a
muted loop's advance-learning pauses without a second mechanism. Reflexes (whisker, stuck)
are bus influencers with no arbiter gain: they act on every winner.

Every loop is specified by five fields. The Cell's four, as built:

| loop | module | infers | sensor(s) | predicts | honest signal | confidence handed to the arbiter |
|---|---|---|---|---|---|---|
| klino (scent) | `RunTumbleNavV2` | which way the scent rises, by running and tumbling | `reality.proprio.scent_max` (a position-only scalar; the eight-nostril ring is never published) | scent keeps rising while running | tumble rate rises when it does not | **non-compliant**: `capability = clamp(scent / EMA of scent-at-eat)`, an EMA of a raw feature on the ground-truth eat event (register V2) |
| planner | `PlaceGraphPlanner` | a route over remembered places to remembered food | `heading`, `vel_ego` (a path integral), the place EPM's TLE, `events.eat` | the next hop's value | `plan_value` (0 when no route, ceded when stalled), disconfirmation while camping | `plan_value ∈ [0,1]` — **but the map it plans over is a grid over the path integral, not the place vocabulary** (register V1) |
| play (explore) | `PlayLoop` | where the map's frontier is | the same path integral and place TLE | more novelty at the next node than here | climb when a strictly more novel neighbour exists and the map grew recently; otherwise wander | **non-compliant as it stands**: `max(climb_value, 1 − habituation)`; with climb latched off it is a recency scalar (register R1) |
| vision | `VisualHomingNav` via `VisualBearing` | the bearing to seen food | `host.video.color` | approach closes the bearing | confidence 0 when occluded | detection confidence in [0,1] (the doctrine's "value in the modality's own units") |

The place EPM (`epm_place`, an RBF encoder over the panorama) supplies a novelty scalar to
the planner and to play. It does not supply the map. That inversion is the audit's central
design finding and round 2's build lever (register O10).

Two scaffolds hold the whole thing up and are named here because no earlier document did:

- **The compass.** The body integrates its own commanded rotation into `heading` and sets
  its orientation to it, so the published heading is the world yaw, drift-free and
  noise-free. It is a legal signal (integrated own-yaw) whose *perfection* is the scaffold;
  the fixed-size grid maps depend on it (register S1, O8). The heading-drift perturbation
  measures the dependence.
- **The eat event.** `events.eat` is the world's collision, not an interoceptive change. It
  bootstraps the planner's food memory, the scent loop's capability, and the vision
  detector's learned appearance (register S3). Whether a homeostatic hit memory is reward
  shaping is the operator's open verdict (O9).

## 2. Signals across the blanket

Legal as published: `scent_max` (frame-free scalar), `vel_ego` (egocentric), `hunger`
(interoceptive), the whiskers and clearance (body-local), the camera frame. World-frame
quantities that a brain module uses only as a magnitude (`imu` velocity, `motor_efference`)
are rotation-invariant and legal. The heading is the named scaffold above.

## 3. Arbitration, as built (`EFEArbiter`, `scoring_mode: efe`)

The module's default is `value_race`, the path the arbiter plan documents as "as built";
**every Cell configuration overrides it to `efe`**, so `efe` is the recipe.

Per tick, with `hunger = 1 − energy`:

```
raw_klino     = hunger · scent                       # the smell-of-food event
v_spike_norm  = z-score(raw_klino) against its own running mean/var, normalised by a
                slow-decaying running peak, clamped to [0,1]

reach_klino   = clamp(capability)      if the loop has reported one, else clamp(scent)
reach_planner = clamp(plan_value)
g_prag_klino    = hunger · reach_klino
g_prag_planner  = hunger · reach_planner
g_prag_vision   = vision_weight · hunger · clamp(vision_value)

gate = clamp(1 − max(g_prag_vision, g_prag_klino, g_prag_planner))   # epistemic_reach_gated, default TRUE
     (legacy form, the ablation: 1 − hunger)

g_epist_klino   = gate · v_spike_norm
g_epist_planner = planner_epistemic ? gate · plan_novelty : 0        # FALSE in every Cell config
G_klino   = g_prag_klino   + g_epist_klino
G_planner = g_prag_planner + g_epist_planner
G_play    = play_weight · (play_hunger_weight ? hunger : gate) · play_value   # sign control = hunger
G_vision  = g_prag_vision
```

The winner keeps the motor until a challenger leads by `margin = hysteresis_k · running_std(top1 − top2)`,
the low-temperature limit of `p(policy) ∝ exp(−G/τ)` with commitment and no dwell constant.
Gains are hard, 1 for the winner and 0 otherwise, on `arbiter.gain.<loop>`.

Three statements the doctrine requires, made explicit:

- **Horizon: one step, greedy.** Every term is an instantaneous scalar. The doctrine (§2.2)
  calls a one-step arbiter greedy and asks that the horizon be stated; it is stated here.
  A rollout horizon is `DEFERRED` (register O2): horizon > 1 measured worse in v3, and the
  slow-loop rollout is the proposed form.
- **Value race or precision weighting?** `OPEN` (O3). The `efe` mode puts every pragmatic
  term in shared units (hunger × a reach in [0,1]) and gates the epistemic terms by need,
  which is the scale-free form §2.2 asks for; whether hunger × reach *is* the precision §2.3
  prescribes, or a proxy for it, is decided by measurement. **Measured 2026-09-06:** the
  reach-gate ablation (A1) is a null under both seedings; the crowding is a *units* defect —
  klino's z-spike and play's value are normalised to their own peaks and sit at 1, while the
  pragmatic reaches are raw (0.04 in the study room; a planner holding a distant route reads
  `plan_value` ≈ 0.15 and never wins). The lever: `pragmatic_norm: planner_peak` divides the
  planner's reach by its own slow-decaying peak (the device play and the z-spike already use;
  no new constant), default `none`, byte-identical. Klino's reach stays eat-calibrated: a
  peak-normalised constant weak scent would read as full reach, which the lever's first form
  demonstrated by failing its own test. The gate's docstring names two pragmatic terms; the
  code takes the max over three, one of which is pre-multiplied by `vision_weight`.
- **The sign of play.** Play is weighted by the *surplus* (the need gate, ≈ 1 − hunger when
  nothing is reachable), never by hunger: a curious bug is one that survives. The
  hunger-weighted form is the shipped wrong-sign control (`play_hunger_weight`).

Three facts about the shipped arbiter the register carries: the planner's epistemic term is
off in every configuration (L1, round 2 A3); the planner's precision channel is consumed by
nothing (L2); with the planner's epistemic term off, the planner needs `hunger · V > 0.5`
to beat a play value near 1, a formal reason it rarely wins that is independent of the room.

## 4. Race or fuse

The rule as built: **fuse redundant estimates of one hidden state; race policies with
different goals.** The maze-fusion configs fuse a scent bearing and a visual bearing of the
same food through the `LateralVoter` (precision `1/(tle+ε)`, the activity term, the
informativeness gate) into `BearingFusion`; the study brain races four loops with different
goals through the arbiter. Whether the two redundant pragmatic loops in the study brain
(scent and sight) should be fused before they race is `OPEN` (O1), to be measured under the
stuck camera, where precision decides who steers.

## 5. The duck mapping

The duck's level-2 action is a twist; `IntentAdapter` already holds a heading reference and
turns `action.vyaw` into a yaw rate. The port keeps the recipe's currency: **a loop emits a
heading and a confidence; the arbiter's winner writes the heading reference (sense slot 10);
the level-2 prior turns the body.** Duck loops on the existing surface: avoid (ToF proximity
→ a heading away), wander (map TLE → a heading toward novelty), seek (an object bearing → a
heading toward it). Which arbitration ports, the `efe` race or precision weighting, is what
round 2 decides (O3); the rung-2 design's §17.4 fork is the same question. No `cpp_core`
module is edited from duck work (gate G5); the loops are reused, and the first duck A/B runs
on a level-2 seed-averaged harness (O12) so it is not a single seed.

## 6. Seek-and-fetch — a design note, not a build

By the rewrite rule: the behaviour is not written; the error it minimises is.

- **The error.** Bearing to an object → 0 and proximity → 1 (the vision loop's own
  predictive error, unchanged), then the bearing to *home* → 0 with the object in hand.
- **The sensor first (doctrine §2.1).** The duck has no camera and no bearing-to-object
  channel; nothing can be sought. The smallest legal sensor is a ToF-derived object bearing:
  a hit in the 8 × 8 depth field nearer than the floor and wall background, reduced to
  `(cx, cy, proximity)` on `percept.object_bearing`, the same token shape `VisualBearing`
  emits, so the `epm_vision → VisualHomingNav` path ports unchanged.
- **Touch as a named scaffold.** The centre columns' TooClose fraction held for K ticks →
  `events.touch`, standing in for `events.eat` until a contact affordance exists.
- **Fetch is the return leg.** The planner routing to the remembered home node under the
  duck's real interoceptive prior (battery, `NeurochemState`) is the far-food relocation
  regime's second half, which is why that regime leads round 2.
- Not round-2 scope (O11).
