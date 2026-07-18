# Cell L2 — EFE Arbiter (policy selection over competent loops)

*Design + build spec. Branch `cell-maze`. Task #34. Doctrine refs: §2 (policy selection into the future), §5 (MotorBus authority-gated learning), §6 (whiten by running scale, no tuning), §8 (default-off, verify the consumer fires).*

## Goal

The Cell now has two individually-competent, individually-honest navigation loops:
- **klino** (`RunTumbleNav`) — the **near-food CLOSER** (methylation chemotaxis; pragmatic).
- **planner** (`PlaceGraphPlanner`) — the **far-field SEARCHER/router** (topological map + value
  iteration + run-tumble explorer; pragmatic route + epistemic explore).

The L2 **EFE arbiter** is the active-inference layer that **chooses which loop drives the body**, by
**Expected Free Energy** — pragmatic (reach food) + epistemic (resolve uncertainty) — computed from
the agent's OWN beliefs (doctrine §2: "the arbiter chooses a pathway INTO THE FUTURE"). It replaces
the planner's old hardcoded scent-vs-plan gate, which was scaffolding for the pre-arbiter era.

## MVP framing (operator decision — documented per request)

The arbiter drives the body by **gating the MotorBus channel gains** (winner ≈ 1, loser ≈ 0),
**not** by collapsing to a single bearing. This is chosen for **LEGIBILITY as a minimum-viable-AI
demonstration, explicitly NOT for efficiency**:
- **Pro:** you can watch the gain faders flip in the inspector and *see* the bug choose a policy;
  both loops' values race live even while only one drives. Transparency is the defensibility bar.
- **Con (documented):** both loops + both HeadingControllers keep computing every tick; the muted
  loop's compute is wasted. An efficient later version would gate the *compute* too. We accept the
  waste for the demo.
- **Winner-take-all** (hard 0/1 muting) follows from "basic channel muting"; chatter is prevented by
  **hysteresis** (below), not by softening the mute.

## EFE formulation (the heart — must be defensible, bar b)

Each loop is a **policy**. The arbiter scores each by a **value** (higher value = lower EFE = better
policy = more pragmatic+epistemic reduction), built from the agent's own inferred/interoceptive
quantities — never a hand-coded rule:

- **klino value:** `raw_klino = hunger × scent_proximity`
  - pragmatic: closing on a *sensed* source reduces hunger. `hunger` = `reality.proprio.hunger`
    (the preference weight); `scent_proximity` = `reality.proprio.scent_max`. epistemic ≈ 0.
  - high ⟺ food is near (scent high) **and** hungry.
- **planner value:** `raw_planner = the food-route value` = `value(next_hop)` **while there is a
  committed route to remembered food, else `0` while merely exploring** (the as-built; the original
  `V[target]` whole-field value is what `raw_planner` *was*).
  - the planner's value field `V[n] = food[n] + tle_gain·node_TLE[n] + γ·max_neighbor` (value
    iteration over the learned map) mixes pragmatic (`food`) + epistemic (`node_TLE`). The planner
    publishes the food-route value on `reality.cognitive.plan_value`.
  - high ⟺ a good food **route** exists; its internal hunger/food-known gating already
    routes-when-hungry / explores-when-sated.

**Normalisation — AS-BUILT (revised, §6 — scale-free, no tuned threshold):** the comparator went
through three iterations. (1) ratio-whitening `v_p = raw_p / (running |raw_p| + eps)` drove both
`v_p → ~1.0` so the incumbent always won by default; (2) a symmetric dual-z-score let a *sustained*
high `raw_planner` pull its own running mean up, so `v_planner` decayed toward 0 and dipped
**NEGATIVE** mid-route → a blind klino (`z≈0`) overtook and **broke** the committed food-route (the
"false-interruption" bug). (3) The final design normalises the two channels **ASYMMETRICALLY**
because they measure different things:
- **klino = a z-SCORE excitement SPIKE** — running baseline mean + variance; `v_klino =
  (raw_klino − mean) / (√var + std_eps)`. A z-score measures CHANGE — correct for klino, because
  smelling food (scent rising above baseline) IS an event. klino SPIKES and wins on approach.
- **planner = a sustained LEVEL** — `plan_peak += max(raw_planner, plan_peak·(1 − plan_peak_decay))`;
  `v_planner = clamp(raw_planner / (plan_peak + eps), 0, 1)`. A valid route is a steady-state
  property, not a change, so it is normalised by its own slow-decaying peak: `~1` while routing to
  good food, `0` while exploring, and **never negative**. A blind klino can no longer overtake it —
  only a real scent z-spike can.

Priority: `smelling-klino(spike) > planner-with-route(level~1) > blind-klino(z≈0) >
planner-exploring(0)`. The route HOLDS unless klino genuinely smells food (clean hand-off to close).

**Winner-take-all + adaptive hysteresis (§2 "a trajectory", §6 "no hardcoded dwell"):** keep the
incumbent winner; switch to the challenger only when `v_challenger − v_incumbent > margin`, where
`margin = hysteresis_k · running_std(v_klino − v_planner)` — the margin adapts to the value-gap's
own scale, so commitment is a property of the dynamics, not a magic dwell-count. Output a hard gain
of `1.0` to the winner's channel and `0.0` to the loser's.

**Policy posterior view:** this is the low-temperature limit of `p(policy) ∝ exp(−G/τ)` with
commitment — a legitimate active-inference policy selection, not a reflex threshold.

## Wiring (gate gains; pause learning via the existing authority mechanism)

```
reality.proprio.hunger ─────────┐
reality.proprio.scent_max ──────┼──► EFEArbiter ──► arbiter.gain.klino  ─┐
reality.cognitive.plan_value ───┘    (id: arbiter)   arbiter.gain.planner ┘
                                                          │
percept.klino_heading ─► HeadingController(klino)  ─► cogk.steer/thrust ─┐
percept.nav_bearing   ─► HeadingController(planner)─► cog.steer/thrust  ─┼─► MotorBus ─► action.left/right
whisker / stuck reflexes ───────────────────────────────────────────────┘
                          MotorBus reads arbiter.gain.<influencer> (default 1.0),
                          effective_gain = base_gain × arbiter_gain  → mix AND authority
```

**Pause-learning is FREE via authority (the reflex analogy is exact, §5):** the MotorBus already
scales each channel's advance-policy learning by its **authority** (share of realized drive). Route
the arbiter gain into the **effective fader** (used for both mix and authority). Then muted →
effective fader 0 → authority 0 → that channel's HeadingController advance learning pauses, exactly
as a reflex taking the bus suppresses cognitive learning. Scope:
- HeadingController **advance** policy: **pauses** when muted ✓ (don't miscredit the other loop's motion).
- The **map** (place-EPM): **keeps learning** — the bug still moves and passively maps while klino
  drives; that is the map-while-foraging win. Not gated.
- **klino**: nothing to pause (methylation is structural/reward-free).
- **Reflexes** (whisker, stuck): never receive an arbiter gain (default 1.0) — the safety layer is
  never muted by the arbiter.

## Per-file changes

### NEW `cpp_core/{include,src}/ogma/modules/EFEArbiter.{hpp,cpp}`
- Inputs: `reality.proprio.hunger`, `reality.proprio.scent_max`, `reality.cognitive.plan_value` (all
  `ProprioToken` scalars). Outputs: `arbiter.gain.klino`, `arbiter.gain.planner` (`ProprioToken`
  scalars). Param `policies` could later be a list; v1 hardcodes the two channels by name params
  (`klino_*`, `planner_*` topic names) so it's config-wired, not name-coupled.
- tick(): compute `raw_klino`, `raw_planner`; normalise asymmetrically (klino z-score, planner
  level); update `running_std(gap)`; winner-take-all with the adaptive `margin`; publish gains
  (winner 1.0 / loser 0.0).
- Params (HotMutable unless noted): topic names (ConstructionOnly), klino z-score `mean_alpha`(0.01),
  `var_alpha`(0.01), `std_eps`(0.02); planner level `plan_peak_decay`(0.0005); `hysteresis_k`(1.0),
  `gap_std_alpha`(0.02). NO tuned thresholds.
- diag_snapshot: `raw_klino, raw_planner, v_klino, v_planner, winner(0/1), gain_klino, gain_planner,
  margin, hunger, scent`. Accessors for tests + telemetry.

### EDIT `cpp_core/{include,src}/ogma/modules/MotorBus.{hpp,cpp}`
- For each influencer, subscribe to optional `arbiter.gain.<influencer_name>` (default 1.0, fresh-
  windowed like other inputs). `effective_gain = base_gain × arbiter_gain`. Use `effective_gain`
  for BOTH the mix contribution AND the per-influencer authority share, so muting zeroes authority.
- New param `gain_mod_prefix` (default `"arbiter.gain."`); empty disables (default-off compatible).
- diag: expose the effective gains so the widget can show the mute.

### EDIT `cpp_core/{include,src}/ogma/modules/PlaceGraphPlanner.{hpp,cpp}`
- **Neutralize the scent-vs-plan gate** → pure searcher: always output the plan/explore bearing
  (route when hungry+route-exists, else explore/wander via the run-tumble explorer). Remove the
  scent-bearing republish branch. Keep the explore-vs-route hunger/food-known logic intact.
- **Publish the food-route value** on `reality.cognitive.plan_value` (`ProprioToken` scalar) =
  `value(next_hop)` while routing to remembered food, else `0` while merely exploring — the value of
  the committed route (NOT the whole-field `V[target]`; exploring publishes 0 so `v_planner` reads 0).

### EDIT registration + build
- `cpp_core/src/ogma/ModuleRegistry.cpp`: register `EFEArbiter`.
- `cpp_core/CMakeLists.txt`: `test_efe_arbiter` executable + gtest_discover.

### NEW config `godot_host/project/addons/ami_ogma/configs/the_cell_arbiter.json`
- Base = `the_cell_place_klino.json` (both loops, both HeadingControllers, reflexes, MotorBus).
- Add `arbiter` (EFEArbiter) before the MotorBus; planner publishes `plan_value`; MotorBus
  `gain_mod_prefix="arbiter.gain."`. Base MotorBus gains for planner/klino set so a 0/1 arbiter gain
  cleanly mutes/passes. food_alternate as in the eval env. Add to launcher allowlist.

### EDIT telemetry + widget
- `godot_host/src/OgmaBrain.cpp` `get_module_metrics`: EFEArbiter block (raw + normalised values
  [`v_klino` z-score, `v_planner` level], `mean_klino`, `plan_peak`, winner, gains, margin) +
  MotorBus effective-gains.
- `godot_host/project/scripts/body_controller.gd`: JSONL emit for `EFEArbiter`.
- NEW `tools/xaq_inspector/widgets/efe_arbiter_inspector.py`: the **value race** — two bars/timeseries
  (v_klino vs v_planner) with the margin band, the current winner highlighted, and the two gain
  faders. This is the demo artifact (watch the bug choose).

### NEW tests
- `test_efe_arbiter.cpp`: klino wins when hunger×scent high; planner wins when plan_value high;
  hysteresis (a tiny challenger lead does NOT flip; a clear lead does); asymmetric normalisation
  (klino z-score scale-free, planner level bounded [0,1] never-negative, a sustained route not broken
  by a blind klino but yields to a smell-spike); gains are a valid 0/1 partition. Ablation hooks: a `force_policy` param for
  the always-klino / always-planner controls.
- MotorBus test: `arbiter.gain.<name>=0` zeroes that influencer's contribution AND its authority.
- Planner test: pure-searcher output (never the scent bearing); `plan_value` published = `V[next]`.

## Defensibility (bars a–d) & gates

- **(b) crux:** `G` is computed from inferred beliefs (`V[n]`, place-TLE, interoceptive hunger/scent),
  not ground truth or a hand rule. The selection is `argmin G` with commitment.
- **(c) ablations:** `force_policy=klino` (always-klino), `force_policy=planner` (always-planner),
  and a `shuffle`/random-arbiter — all ship with the module. Gate: the EFE arbiter must beat the
  random arbiter and at least match the better fixed policy on eats over a foraging run.
- **(d) perturbation:** relocate the food (the env's `food_alternate` hop already does this) → `V[n]`
  re-inflates at the new region → the arbiter re-selects (planner to route there, klino to close).
  This is the sharp test; assert the winner switches appropriately after a relocation.
- **Verify the consumer fires (§8):** confirm the MotorBus actually applies the arbiter gains
  (effective gain of the loser → 0; its authority → 0; its advance learning paused).
- **Behavioral gate:** in the L-bend with food alternating, the arbiter selects **klino when scent is
  high/rising near the source** and **planner when scent is low/stalled** (search/route), visibly in
  the widget; eats ≥ the better fixed policy.

## Open items / future
- **Efficient version:** gate the losing loop's *compute*, not just its gain (post-MVP).
- **Finer hunger weighting:** split the planner's published value into pragmatic (food-route) +
  epistemic (frontier-TLE) so the arbiter can weight the food part by hunger independently
  (sated → explore even when a route exists). v1 uses the planner's food-route value (0 when exploring).
- **N-loop:** the gain-vector output generalizes; add the PLAY loop (#33) as a third policy.
