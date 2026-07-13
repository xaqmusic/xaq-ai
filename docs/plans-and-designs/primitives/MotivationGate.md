# MotivationGate — Primitive Contract

**Lineage:** added 2026-06-21 (commit `3404fae`) as Stage 2 of the Cell de-scaffold —
grounding foraging in intrinsic homeostatic need (the project's homeokinetic thesis).
**Header:** `cpp_core/include/ogma/modules/MotivationGate.hpp`
**Impl:** `cpp_core/src/ogma/modules/MotivationGate.cpp`

---

## Purpose

"**Forage because hungry.**" The MotivationGate sits between perception (the food-bearing
`[cx, cy]`) and the action layer (`HeadingController`) and scales the desired-heading
**magnitude** by the homeostatic deficit (hunger):

- **hungry** (energy < setpoint) → full magnitude → the action layer pursues + charges;
- **sated** (energy ≥ setpoint) → magnitude → 0 → `HeadingController.nav_on = false` → the bug
  **idles** instead of seeking food.

The **direction is untouched** (`atan2` is scale-invariant), so the action layer is unchanged —
this is the **clean separation** (decomposition option (i)): *motivation decides WHETHER to
forage; the action layer decides HOW to act on a heading.* It is the in-runtime expression of the
H-JEPA "preferred observation = sated; act to close the deficit" loop, applied at the
*whether-to-forage* level.

**It does not learn.** It is a fixed homeostatic *drive-grounding* — the mechanism that converts a
standing internal need into behavioral modulation. (The learners are upstream perception / nav and
the downstream action layer.) Reward shaping is explicitly avoided: there is no external reward,
only the intrinsic energy deficit.

---

## The mechanism

Read the body's energy directly from `reality.proprio.energy` (a scalar in [0, 1] that drains
passively and is replenished on eating). Compute a pursuit gain:

```
g = clamp((sated_energy − energy) / sated_energy, 0, 1)
```

- `energy ≥ sated_energy` → `g = 0` (sated → idle);
- `energy → 0` → `g → 1` (urgent → full pursuit);
- linear ramp between.

Emit the desired heading scaled by `g`: `out = [cx·g, cy·g, prox]`. Because the input bearing is a
unit vector (with `ScentCompass.normalize_direction`), `|out| = g`, so the downstream
`HeadingController.nav_on` gate (`|heading| > min_signal`) turns pursuit **on below the setpoint,
off above it** → foraging runs in **hunger-gated bursts** (eat → energy jumps → idle → drain →
re-forage).

Energy comes from the **body**, not a `HomeostaticDrive` channel: reading `energy` directly gives a
clean setpoint-relative deficit and avoids the `scent_proximity` channel that would pollute the
combined `HomeostaticDrive.urgency` (that tracks "far from food", not "hungry").

**Ablation:** `freeze_gain ≥ 0` overrides `g` with a constant (e.g. `1.0` = always pursue, hunger
ignored). The A/B (motivated vs `freeze_gain=1`) isolates need-modulation.

---

## Input Topics

| Topic (default) | Kind | Payload | Required | Notes |
|---|---|---|---|---|
| `heading_topic` = `percept.scent_compass` | Direct | `ProprioToken` `[cx, cy, (prox)]` | yes | Food-bearing to be gated. In the Stage-3 stack this is `percept.bearing_inferred`. |
| `energy_topic` = `reality.proprio.energy` | Direct | `ProprioToken` (scalar [0,1]) | yes | Homeostatic state; `g` is computed from its setpoint deficit. |

## Output Topics

| Topic (default) | Payload | Cadence |
|---|---|---|
| `output_topic` = `percept.motivated_heading` | `ProprioToken` `[cx·g, cy·g, prox]` | every tick |

→ `HeadingController.input_topic`.

---

## Parameter Schema

| Key | Mutability | Default | Description |
|---|---|---|---|
| `heading_topic` | ConstructionOnly | `percept.scent_compass` | Food-bearing input to gate. |
| `energy_topic` | ConstructionOnly | `reality.proprio.energy` | Homeostatic scalar. |
| `output_topic` | ConstructionOnly | `percept.motivated_heading` | Gated heading (→ action layer). |
| `cx_index` / `cy_index` | ConstructionOnly | 0 / 1 | Component indices. |
| `sated_energy` | HotMutable | 0.8 | Homeostatic setpoint: `g = 0` at/above it, ramps to 1 at empty. |
| `freeze_gain` | HotMutable | −1.0 | ≥0 → constant gain (ABLATION: 1 = always pursue). <0 → use the homeostatic gain. |

---

## Diagnostics

`gain` (pursuit drive ∝ hunger, → 0 when sated), `energy`.

---

## Invariants

1. Publishes exactly one gated heading per tick.
2. `g ∈ [0, 1]`; direction of the output equals the direction of the input (magnitude only is
   scaled).
3. `freeze_gain < 0` ⇒ behavior is a pure function of `(energy, sated_energy)`; `freeze_gain ≥ 0`
   ⇒ `g` is constant (no dependence on energy).
4. Requires `energy_drain > 0` upstream (body metadata) for hunger to be real — with zero drain the
   bug never crosses the setpoint and the gate is inert.

---

## Status / findings (turn rig, n=3, vs `freeze_gain=1` ablation)

- **Motivated:** all seeds **survive**, energy regulated at the setpoint (μ 0.84, tight
  [0.60, 1.00], sd 0.09), idle 67–69 % when sated, `corr(gain, hunger) = +0.81`, genuine approach
  (inter-hit 3.5 m) — **textbook homeostatic regulation**.
- **Ablation (always pursue):** energy **boom-busts to 0** (homeostatic collapse), idle 0 %, no
  need-modulation — eats *more* but starves.
- **Bonus:** regulation makes the system robust to the imperfect learned-advance orbit — eat
  *enough* to stay sated, so foraging *efficiency* stops mattering. **Regulation over
  maximization** = the homeokinetic thesis (and the aliveness-over-distance principle).

Config: `the_cell_corridor_turn_heading_motiv{,_ablate}.json`. Analyzer: `scripts/head_motiv_analyze.py`.
See `docs/plans-and-designs/cell_descaffolding_plan.md` (Stage 2).
