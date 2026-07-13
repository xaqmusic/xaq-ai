# Cell Vision-Homing Loop — the fourth policy: close on a SEEN source

*Design + build spec. Branch `cell-maze`. The fourth strange loop, arbitrated as a
peer of klino/planner/play. Doctrine refs: §1 (predict-or-die — the loop has its own
predictive component + error), §2.1 (pragmatic foraging), §2.2 (the shared-unit EFE
ledger), §2.3 (precision/gains from own dynamics — no-tuning), §3 (one directive per
loop), §5 (decompose, don't disable), §8 (default-off, verify the consumer, staged
gates). Anticipated in `cell_play_loop_plan.md` §8 ("occlusion-break vision-homing").*

---

## 0. The problem this fixes (grounded in the scoping diagnostic)

`docs/findings/play_frontier_wander_findings_2026-07-10.md` (Addendum) established, by
a closest-approach-to-food diagnostic, that composition **eats are SIGNAL-limited**:

- The **close loop already works** — in a scent-rich env the bug homes all the way to
  **0.16 m (contact)** when it has a scent gradient. Not the bottleneck.
- **Play coverage works** (2.5× maze discovery) but does **not** move eats. Not the lever.
- Eats are bounded by **food DETECTION**: scent reach is maxed (15; >16 flattens the
  field), and where walls block scent the bug has **no food signal at all** — in the
  walled quad its eats are collision luck (closest sampled approach 1.4–2.3 m, never homes).

The gap is a **missing sensory channel for food**, not a policy defect. The bug can
**smell** food (klino) but cannot act on food it can only **see**. This loop adds the
second pragmatic close-channel: **home on a SEEN food source**, complementing klino's
scent-taxis exactly where scent fails (line-of-sight past/around a scent barrier).

**This is the multimodal-redundancy thesis of the whole architecture** (the Reality
Token, cross-modal voting): two sensors for the same pragmatic goal (reach food), the
L2 arbiter selecting whichever is **confident** in context — scent when smellable, sight
when visible.

---

## 1. The directive + the doctrine posture — "klino, but the source is SEEN"

**One directive (§3):** *close on a food source the bug can SEE.* This is the visual
twin of klino (*close on a food source the bug can SMELL*). Same posture:

| loop | directive | sensor | value (pragmatic) |
|---|---|---|---|
| **klino** (RunTumbleNavV2) | close on a *smelled* source | scent scalar | `hunger · scent_confidence` |
| **planner** (PlaceNav) | route to a *remembered* region | place map | `hunger · food_route_value` |
| **play** (PlayLoop) | grow the map (explore) | place-EPM TLE | `energy_surplus · novelty` (epistemic) |
| **vision** (NEW) | close on a *seen* source | food-pixel bearing | `hunger · sight_confidence` |

**Pragmatic, hunger-weighted** (like klino, unlike play): the bug should home to seen
food *when hungry*. When full it should play/explore (the energy-surplus posture already
in the arbiter). No new energy threshold — the arbiter's race sets the crossover (§2.3).

### 1.1 Why it is a LOOP, not a reactive visual servo (§1 predict-or-die)

Wiring `VisualBearing → HeadingController → MotorBus` alone would be a *reactive* servo
(steer at the green blob) — cybernetics, not active inference. The loop earns "AI" by
carrying **its own predictive component with its own error**, exactly as the other loops
do (klino's methylation/belief error; play's novelty-value-iteration on the place-EPM TLE;
HeadingController's learned turn-gain). Here that predictive component is **an EPM** (§2) —
the architecture-native TLE — plus two learned elements on top, no hand-set gains:

0. **The vision-food EPM's TLE is the predictive core (§1, §2).** The food bearing is
   encoded by an EPM that predicts its temporal dynamics; the residual is the canonical
   TLE. As the bug homes, a real food's bearing evolves predictably (it centres) → low
   TLE, high informativeness = a trackable target; an occluded/flickering/false blob →
   degenerate node / low informativeness → the loop's confidence collapses and it cedes.
   This SUBSUMES the hand-rolled "looming forward-model" — temporal prediction of the food
   percept is exactly what the EPM does natively. (Reactive-vs-predictive check, §1/R2:
   the loop acts to make its prediction — "the food will centre and I will eat" — true.)

1. **Learned food-appearance (in `VisualBearing`, `learn_appearance=true`).** The bug does
   **not** know "food = green." It LEARNS food's colour by association: on each real eat,
   the central FPV (food is dead-ahead at the moment of eating) is blended into a food
   prototype; a pixel is "food" if within `color_match_dist` of it. **Scent/eating is the
   teacher, vision the student** — before the first eat there is no visual bearing. Emergent,
   not hardcoded (§1); v1 uses this mode (the green-pixel test is a scaffold, shipped OFF).

2. **Eat-calibrated reach confidence (shared-unit value for the arbiter, mirrors klino).**
   klino folds the scent-at-each-real-hit into `eat_scent_` and reports `cap = smell /
   eat_scent`. **Vision does the same on the food-pixel fraction:** fold `green_frac` at
   each eat into a learned `eat_green_` (how large food looms at the instant of eating);
   `cap_vision = green_frac / eat_green_` → self-calibrated "the seen food is within reach."
   Combined with the EPM confidence, this is the **honest value** the arbiter needs (an
   over-confident vision loop would starve klino/play; cf. the arbiter defensibility audit).

**No-tuning (§2.3):** the EPM learns the food-bearing dynamics; the turn gain is
HeadingController-learned; the food appearance and `eat_green_` are learned from eats;
`cap_vision` is self-normalised. No hand-set "steer at green with gain k."

---

## 2. Substrate — `VisualBearing → EPM (vision-food modality) → the CONTROL loop`

**Perception goes through an EPM, like every pathway in AMI-Ogma** (§1 predict-or-die;
modality parity — color/saliency are already EPM modalities). Two existing pieces + one
thin new policy overlay. **Do not rebuild the perception.**

```
host.video.color (raycast FPV, headless)
  → VisualBearing (learn_appearance=true, emit_proximity=true)
      → percept.visual_bearing = [vx=+right, vy=+forward, proximity] + green_frac
  → EPM (vision-food modality; RBF encoder on the bearing; the fusion-PoC per-modality EPM)
      → reality.<group>.vision = { winner_id, tle, informativeness }
  → VisualHomingNav (thin policy overlay: value + bearing→HeadingController)
```

This is the **same shape as the place loops** — `CylinderBuilder→EPM` for PLACE (planner/
play are thin overlays on `winner_id + tle`); `VisualBearing→EPM` for FOOD-VISION (this
loop is the overlay). One perceptual EPM per modality; a thin policy on top (§3).

**The EPM is the loop's predictive substrate (§1.1):** it clusters the egocentric food
bearing into ANGLE-SELECTIVE nodes and predicts the bearing's temporal dynamics; its
**TLE is the canonical §1 error**, and its **informativeness** (a varying food bearing
grows angle-selective nodes; an occluded/absent one bakes ~1 degenerate node) is the
confidence that gates the loop's value — the Vision+Scent Fusion PoC's "trust tracks
informativeness," here feeding the **arbiter** directly rather than the LateralVoter.
`VisualBearing`'s occluded→[0,0] no-signal state makes the degenerate-node/low-
informativeness case automatic.

The **new module** (`VisualHomingNav`) consumes the EPM output + the raw bearing and owns:
- the **value** = informativeness/TLE-derived confidence × **eat-calibrated `cap_vision`**
  (reach: `green_frac / eat_green_`, subscribing `events.eat` for the teacher), shared-unit;
- the **bearing→motor** dispatch (its own `HeadingController`, learned advance);
- the **published value** `reality.cognitive.vision_value` → the arbiter.

---

## 3. The module — `VisualHomingNav` (name TBD by operator; clean-room, versioned)

Anti-cruft convention (per RunTumbleNavV2 / PlaceNav): a **new** module, not a fork of
klino (klino is scalar run-and-tumble; vision is a *directional* percept — different
mechanism), and not a re-use of the retired `BearingFusion` (which fused vision INTO
scent at the voter and went degenerate; here vision is a **separate arbiter policy**,
§3). Cleanest description: *"klino's value construction on `green_frac` + `VisualBearing`'s
directional bearing instead of run-and-tumble."*

**Inputs:** `vision_epm_topic` (the vision-food EPM's `reality.<group>.vision` = winner_id
+ tle + informativeness — the predictive substrate), `visual_bearing_topic`
(percept.visual_bearing — the continuous steering direction), `heading_topic`, `vel_topic`,
`hunger_topic` (energy), `eat_topic = events.eat` (the `eat_green_` teacher).

**Core (per tick):**
- read the EPM `tle`/`informativeness` + `(vx, vy, proximity)` + `green_frac`; if
  `green_frac < min_conf` (occluded/no food) → no-signal (value 0, bearing [0,0] — cede).
- fold `green_frac` at a real eat into `eat_green_` (EMA); `cap_vision = clamp(green_frac
  / max(eat_green_, eps), 0, 1)` — self-calibrated reach confidence.
- **value** `sight_confidence = epm_informativeness · max( v_spike , cap_vision )`, where
  `v_spike` = z-score of `green_frac` above its own baseline (food ENTERS view = a spike;
  mirrors klino's spike-or-level so the arbiter sees shared units). The EPM informativeness
  factor collapses the value on a degenerate/occluded percept (the predictive gate, §1).
- **bearing out** = the (anti-jittered) `visual_bearing` when confident, else [0,0].

**Outputs:** `percept.vision_bearing` (egocentric, same convention) +
`reality.cognitive.vision_value` (∈[0,1] pragmatic confidence → arbiter). **Default-off
(§8):** empty output/value topics ⇒ no publish; existing configs byte-identical.

**Accessors** for `get_module_metrics` (the recurring telemetry gotcha, §6): `have_food`
(green_frac>min), `cap_vision`, `eat_green`, `vx/vy`, `value`, `v_spike`.

---

## 4. Arbiter — the FOURTH policy (§2.2). The arbiter is currently HARDCODED 3-policy.

The `EFEArbiter` has explicit `G_klino_/G_planner_/G_play_` members (not a generic
N-vector — the header's "N-loop" is aspirational). Add vision the way play (task #33) was
added:

```
G_klino   = hunger        · scent_confidence     (pragmatic close — smell)
G_vision  = hunger        · sight_confidence      (pragmatic close — sight)   ← NEW
G_planner = hunger        · food_route_value      (pragmatic route)
G_play    = energy_surplus · novelty              (epistemic grow)
winner = argmax G   (winner-take-all gain vector, adaptive hysteresis)
```

- Params (mirror `play_value_topic`/`play_weight`): `vision_value_topic` (empty=absent,
  byte-identical 3-policy arbiter) + `vision_weight` (0=inert default; 1=Stage-2 on).
- All terms shared-unit [0,1] (vision via `cap_vision`/`v_spike`, same construction as
  klino) → no cede constant, no scale mismatch.
- **klino vs vision (two pragmatic close-loops) — precision-weighting, not conflict.**
  They mostly AGREE on direction when both sense the same food (both steer at it → cheap
  thrash). The arbiter picks the more *confident* sensor: scent-rich open region → klino's
  `cap` high (vision may tie/agree); scent-blocked but line-of-sight → klino blind
  (`scent≈0` → `G_klino≈0`) while `G_vision` carries → **vision wins where scent fails**,
  the whole point. The existing **adaptive hysteresis** (margin = k·std(gap)) damps
  authority flips; **watch for scent↔sight oscillation** as a Stage-2 gate.
- **Honesty guard (belt-and-suspenders, cf. the planner-cede):** vision value only
  non-zero with food actually in view (`green_frac>min`) — a blind vision loop never
  competes, exactly like a blind klino leaves `v_planner` untouched.

**Motor wiring:** a fourth `HeadingController` (`heading_controller_vision` →
`cogv.steer/thrust`), gated by `arbiter.gain.vision`; MotorBus mixes it as the fifth
cognitive channel (klino/planner/play/vision + whisker/stuck reflexes), `turn_brake=0`.

---

## 5. Telemetry / inspector (the recurring gotcha — §6 of the play plan)

`get_module_metrics` uses **accessors**, not `diag_snapshot` (see
`reference_headless_diag_reformatter_keys`): new `VisualHomingNav` fields + the arbiter's
`G_vision`/`gain.vision` need explicit accessor lines in `OgmaBrain::get_module_metrics`
**and** the `body_controller.gd` reformatter (short keys) or they read 0 in the JSONL.
v4_inspector: extend the arbiter race widget to **four** stacked bars; add a `vision`
widget (food-in-view lamp + `cap_vision` meter + bearing gauge). HUD tug-of-war → 4-way.

---

## 6. Staged plan (fast-fail, promote-or-kill gates)

Primary env = the **walled quad** (`the_cell_arbiter_quad_walled_play_klino2.json`,
`quad_complexity=1`): food behind scent-blocking internal walls **with line-of-sight** —
the scent-degraded regime where vision is the only food signal. Baseline = frontier-play
composition (eats 1.8, the FLAT null from the scoping doc). Metric = **eats** (+ the close
diagnostic: does `min food-dist` now reach contact via SIGHT, and `frac<2m` rise?).

- **Stage 1 — Wiring (pilot).** `VisualHomingNav` compiles; `VisualBearing → food-bearing EPM
  → VisualHomingNav` wired end-to-end; `VisualBearing(learn_appearance)` publishes a bearing
  after the first taught eat, the EPM bakes angle-selective nodes + emits TLE; arbiter adds
  `gain.vision` with `vision_weight=0` (inert). **GATE:** value ∈[0,1]; EPM TLE non-degenerate
  when food varies in view; HeadingController
  produces motion when force-authoritied; 4th channel at 0 authority; scent-rich baseline
  byte-identical (default-off proven); consumer fires (§8). Unit tests (cap_vision calibration,
  occluded→0, spike; arbiter 4-policy race).
- **Stage 2 — Signal.** `vision_weight=1`. Walled quad, structural read. **GATE:** (a)
  vision WINS when hungry + food in view, LOSES when blind/occluded (green_frac→authority
  visible); (b) with vision on, the bug HOMES to seen food — `min food-dist` reaches contact
  where scent alone left it at 1.4–2.3 m; (c) **eats rise above the 1.8 FLAT baseline**; (d)
  scent-rich `route2region_lbend` NOT regressed (vision doesn't out-compete a working klino).
  No scent↔sight authority thrash (hysteresis holds).
- **Stage 3 — Direction.** Confirm the clean 4-way division (klino smells / vision sees /
  planner routes / play grows) + the homeostatic cycle. Verify the **EPM's TLE/informativeness
  gates occlusion honestly** — if a measured see-but-can't-reach failure survives (vision
  over-commits to a blocked blob the EPM still finds informative), escalate the EPM predictor
  (e.g. condition on efference so it predicts the bearing *given the bug's own motion*).
  **GATE:** walled-quad eats up, no regression, crossover emerges from the race.
- **Stage 4 — Powered.** n≥10 by varying the ENV (det-env caveat). **Ablations ship with
  the loop (§2c):** vision-off (dead value topic ⇒ 3-policy baseline); vision-forced-on
  (ignore confidence — should over-commit to occluded blobs = the honesty control);
  **green-scaffold vs learned-appearance** (does the taught model match the hardcoded test);
  **wrong-sign** (weight vision by energy_surplus not hunger — should regress).

---

## 7. Open items / future

- **N-policy arbiter generalization.** Two pragmatic close-loops (`hunger · modality_conf`)
  now beg a generic `pragmatic-close[modality]` vector instead of hardcoded members — a
  clean refactor AFTER vision proves out (don't refactor on spec; anti-cruft).
- **Both-blocked case.** Where a wall blocks scent AND sight, neither pragmatic loop can act
  — that region is genuinely under-sensed; only play (blind coverage) can stumble it. Honest
  limit to document, not paper over.
- **Fusion vs separate-policy.** The retired `BearingFusion` fused vision+scent into one
  bearing (degenerate flat trust). This plan keeps them as SEPARATE arbiter policies (§3).
  If Stage-2 shows they thrash or want a blended bearing when BOTH sense, revisit voter-level
  fusion as a *sensor* feeding a single close-loop — but only on measured need.
- **Two vision EPMs — dedicate vs unify.** This loop adds a **dedicated food-bearing EPM**
  (RBF on `percept.visual_bearing`, angle-selective). The EXISTING vision-EPM encodes the
  raw colour frame and feeds only averaged arousal (the play-plan §8 "give it a job or trim
  it" channel). Keep them separate for v1 (different percepts, different jobs); a later
  unification (one vision-EPM serving both arousal and the food-bearing) is a possible
  simplification once the food-bearing EPM proves out — but only on measured need (anti-cruft).
- **EPM config (modality parity).** The food-bearing EPM follows the place-EPM's shape (RBF
  encoder, small `max_nodes`, `process_every_n_ticks=1`); `dim_min/max` cover the bearing
  `[vx,vy,proximity]` range. It is a normal EPM modality (`modality_group`/`modality_name`),
  exported/loaded like the rest — no special-casing.

---

*Living doc. Fold results back into `docs/findings/` and the memory topic file as each
stage promotes or kills. Pairs with `cell_play_loop_plan.md` (the template) and
`cell_efe_arbiter_plan.md` (the ledger this extends).*
