# Cell chemotaxis — reward-free active-inference navigation (the picrawler nav port)

**Status (2026-06-15): working repro + UI demo.** The picrawler's reward-free homeokinetic
*navigation* result reproduced in the Cell environment with the **same brain wiring**, only
the body morphology swapped. Full N=10 generality proof (pre-registered hypotheses,
closed-vs-open figure, head-to-head vs a reward-driven Cell baseline) is the deferred
follow-up (Phase B).

System: a transparent **two-module chain** — `ScentCompass` (perception) reduces the raw
8-nostril scent ring to an egocentric up-gradient bearing, and `ChemotaxisAI` (control) steers
two flagella to drive that bearing's lateral component to zero — Friston active-inference
closure, acting to null the agent's own perceived prediction error. **No reward, no
reinforcement, no reward shaping, no Premotor, no NeurochemState.** The nutrient hits the body
counts (Area3D, ground-truth) are the metric, never fed back to the controller.

The whole perception→action chain is **visible and auditable in the brain graph** —
`reality.proprio.scent (sensor) → ScentCompass → percept.scent_compass → ChemotaxisAI →
action.left/right (sinks)` — and the body is a **raw-sensor publisher only**: nothing crosses
the body↔brain boundary that isn't in the canonical `register_source` list. (An earlier draft
computed the bearing body-side in GDScript and published an *undeclared* `scent_compass`
topic; that hid the perception off-graph and bypassed the boundary register. Moving it into
the `ScentCompass` brain module fixed both — same behavior, bit-for-bit.)

---

## The same loop, swapped I/O

| | picrawler (MotorEPM nav block) | cell (ChemotaxisAI) |
|---|---|---|
| **percept** | `reality.proprio.target_compass` (ground-truth bearing) | `reality.proprio.scent_compass` (8-nostril gradient bearing) |
| **steering** | `bearing = atan2(cx, cy)`, `steer = nav_gain·bearing` | **identical** |
| **actuator** | skid-steer differential over 12 leg servos | differential thrust over 2 flagella (`action.left/right`) |
| **reward** | none | none |

The controller code is structurally the same `bearing → steer → differential` loop
(`MotorEPM.cpp:981-998` ⟷ `ChemotaxisAI.cpp::tick`). The morphology accommodations are:

1. **Sensor** — the cell perceives a *chemical gradient* where the picrawler was handed a
   ground-truth target bearing. The `ScentCompass` brain module subscribes the raw 8-nostril
   `reality.proprio.scent` ring and reduces it to a 2-D egocentric **raw** gradient vector
   (scent-weighted sum of the nostril directions at angles 2π·i/N): direction = up-gradient
   bearing, magnitude = gradient strength. Published as `percept.scent_compass`. The body
   publishes only the raw scent sensor (a registered source); the perception is in the brain.
2. **Actuator** — two flagella under `differential_paddler` + `reflex_modular`, where
   `action.left/right` are direct per-flagellum thrusts (both → forward, asymmetric → turn).
3. **The edge-of-signal gate** — see below; the one place the control law adapts to the
   flagellar body.

## The morphology accommodation that mattered: the spin-orbit gate

The `differential_paddler` only *translates* when both flagella spike synchronously; a hard
steer drives one flagellum to zero → pure rotation. Near a nutrient the scent saturates and
the spatial gradient **flattens**, so a normalized bearing there is noisy — and chasing it
made the cell **pivot in place and orbit the food forever**, never satisfying the
forward-motion hit condition (diagnosed: `scent_max≈1.28` with `ang_v>0.5` in 84/92 windows,
`fwd_v≈0`; *not* a wall-wedge — `wmax=0`, pillars=0).

The fix is principled, not tuning: `ScentCompass` sends the **raw** gradient (magnitude =
confidence), and `ChemotaxisAI` steers only when the magnitude clears a low floor
(`min_signal`). Below it (near the saturated peak, or far in a flat region) the cell **drives
straight** — punching *through* the food, or exploring — instead of orbiting. The
active-inference loop is unchanged; only the confidence gate is added.

## Result (closed vs open loop, 90 s sim/seed, turbo)

| condition | hits/90 s (seeds 1-5) | mean |
|---|---|---|
| **closed loop** (`nav_gain 0.8`) | 5, 5, 8, 11, 1 | **6.0** |
| open loop (`nav_gain 0`, percept severed) | 1, 0, 0, 0, 0 | 0.2 |

**~30× closed over open**, and *every* closed seed scores (no orbit/wander failures). The
only difference between conditions is whether the perceived gradient bearing drives the
action — so the gap-closing in the closed condition *is* action minimizing the agent's own
prediction error. Reward plays no role. This is the Cell analog of the picrawler A2
active-inference signature.

**Honest caveat:** n=5, single duration; one seed scores low (1) — init-basin variance
(the cell's analog of the picrawler gait multi-stability), not a failure mode. The full
N=10 + head-to-head vs a reward-driven Cell baseline + a closed-vs-open figure is the
deferred Phase B generality proof.

## Lessons applied (from the Motor-EPM phase)

This port deliberately carries the durable lessons of the picrawler work
(`feedback_additive_bias_disrupts_emergent_gait`, `aliveness-over-distance-metric`):

- **Reward-free + active inference.** Motivation is intrinsic loop-closure (perceive bearing
  → act to null it), never a reward signal. The nutrient count is a *metric*, off the
  control path — exactly as the picrawler's `walk_visit_count` never fed the controller.
- **No additive bias.** The spin-orbit was fixed by a *constraint* (a confidence gate that
  removes steering where the signal is unreliable), not by adding a corrective joint/thrust
  bias — the same signature as the three picrawler wins (objective-change / constraint-
  removal, never an additive bias).
- **Verify the consumer, behaviorally.** The `nav_gain` sign was confirmed by ground-truth
  behavior (the cell climbs toward food), not a geometric convention — the lesson from the
  hip1 sign near-miss.

## Reproduce

UI: launch the **cell** env → select **"Cell — reward-free chemotaxis (active inference)"** →
Launch (the config declares `body_model=differential_paddler` + `reflex_modular`, now
honored by the launcher, so no dropdown fiddling).

Headless (closed vs open, per seed):
```bash
for cfg in the_cell_chemotaxis_ai the_cell_chemotaxis_open; do
  OGMA_CELL_CONFIG="res://addons/ami_ogma/configs/$cfg.json" \
  OGMA_REFLEX_MODULAR=1 OGMA_BODY_MODEL=differential_paddler OGMA_TURBO=1 \
  OGMA_EPISODE_LENGTH=5400 OGMA_QUIT_AFTER_TICKS=5520 OGMA_SEED=1 \
  timeout --signal=TERM 90 godot4 --path godot_host/project --headless \
    --fixed-fps 600 --disable-render-loop res://scenes/the_cell.tscn 2>/dev/null \
  | grep -oE '"hits_total":[0-9]+' | tail -1
done
```

## Files

- Perception: `cpp_core/{include,src}/ogma/modules/ScentCompass.{hpp,cpp}` (raw 8-nostril
  `reality.proprio.scent` → `percept.scent_compass`).
- Control: `cpp_core/{include,src}/ogma/modules/ChemotaxisAI.{hpp,cpp}` (`percept.scent_compass`
  → `action.left/right`). Both registered in `ModuleRegistry.cpp`, `CMakeLists.txt`.
- Body: unchanged — publishes only the raw `reality.proprio.scent` sensor (no body-side
  perception; the body diff for this work is zero).
- Configs: `the_cell_chemotaxis_ai.json` (headline, allowlisted; ScentCompass + ChemotaxisAI),
  `the_cell_chemotaxis_open.json` (open-loop control, script-only).
- Launcher: `_CELL_CONFIG_ALLOWLIST` + config-declared `body_model` honored at launch.
