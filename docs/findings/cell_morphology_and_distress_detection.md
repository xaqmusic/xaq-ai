# Cell — bidirectional morphology + homeokinetic distress detection ("signs of life")

**Date:** 2026-06-16 · **Branch:** gait-ignition-homeokinetic (uncommitted working tree)
**Status:** Three wins landed (morphology + detection + hunger-gated foraging/desperation). The
bug now **survives by foraging** under genuine hunger pressure (drain 0.012/s, no starvation
n=3). Emergent *escape* and *learned* (critic-driven) homing still ahead.

## The milestone (operator-observed)

First time the **cell bug shows aliveness comparable to the picrawler** — varied, closed-loop
locomotion (forward / reverse / turn / pause / drift) instead of the prior "ram forward or
freeze." The substrate is now genuinely alive in the cell. The boredom/stuck mechanics do not
yet produce an *emergent escape*, but the foundation they sit on is solid.

This came from two changes, pursued in dialogue while watching the bug pin in the top-right
corner (7.6, −7.6) within ~30 s on every seed.

---

## Win 1 — Bidirectional + pause morphology

**Problem.** The `differential_paddler` was forward-only and half-wave-rectified
(`body_controller.gd` match block): synchronous spikes only *add* forward speed, single spikes
only rotate; the code comment even reads *"reverse spike isn't a thing in the spike model."* A
body that can only ram forward and spin **physically cannot back out of a corner** — no escape
signal can fix that.

**Change.** New `bidirectional_paddler` body model with **signed** thrust:
- common-mode `(aL+aR)/2` → forward **/ reverse**
- differential `(aL−aR)/2` → turn
- zero command → **pause / drift** (coast on friction)

**The unification that mattered.** Signed/linear thrust *removes the half-wave rectification* —
which was the **root of the Stage-1 phantom-TLE** (a linear forward model can't represent a
rectified actuator, so motor-TLE read non-zero even when frozen). The `motor_baseline_beat` was
a *workaround* for that dead-zone. With signed thrust the dead-zone is gone, so the homeokinetic
forward model is honest **without** the beat hack, and pause becomes a real option.

**Result (seed 1, default params, bidirectional):** the homeokinetic loop produces real
exploratory locomotion — **total path 55 m, reverse on 24 % of ticks (fwd to −1.15), pause/drift
on 43 %, no freeze.** The Stage-1 freeze risk did **not** materialize; removing the rectification
freed the HK to move.

---

## Win 2 — Homeokinetic distress detection (the "I'm stuck" signal)

**The trap.** Motor-TLE is a phantom (above). Worse, the **IMU published motor *intent***
(`velocity = forward × _forward_speed`, an efference copy), so when the bug wedged and kept
beating, perception read "I'm moving at 1.4" and **never froze** — boredom couldn't see the pin.

**The fix (reafference).** Make proprioception **afferent**: the IMU now publishes the *actual*
world velocity (post-`move_and_slide` position delta), and the *efference copy* is published
separately on `reality.proprio.motor_efference`. The brain (`DistressDrive`) then computes two
complementary stuck-signals:

- **reafference mismatch** = `clamp01((efferent − afferent) / efferent)` — *trying but not
  moving*. Catches the **RAM** mode; saturates to ~1.0 at a wedge, ~0 free-swimming.
- **no-progress** = `1 − |EMA(afferent velocity *vector*)| / scale` — consistent travel keeps the
  vector EMA up; jitter/pause cancels it toward 0. Catches the **PARK / idle** mode the mismatch
  misses (bug not trying, just not getting anywhere). A slow EMA means *brief pauses stay valid*
  (energy-conserving locomotion); only a *sustained* park trips it.

**Result.** Boredom now fires for *every* stuck mode. Adding no-progress fixed the park outright:
**total path 19 → 55 m, fraction-parked 84 % → 10 %** — the bug stays active and never idles to
death in place.

The meta-EPM TLE / staleness / pooled-state-motion signals were all noisy or inverted on this
body and are kept only as low-weight corroboration; the **reafference pair is the clean primary.**

---

## Win 3 — Hunger-gated foraging + desperation (the "I'm hungry, find food" drive)

**The seed-5 observations (operator).** With the morphology + detection wins in, the bug was
*alive* but (1) had a tendency to **turn away from food** at the whiskers / never actively
forage, (2) its boredom escape **did not escalate** with how long it had been stuck, and (3)
hunger never **compounded** with boredom into desperation. Diagnosis: the bug had **no
food-attraction drive** — the critic was ~random and curiosity chased openness, which can pull
*away* from a stable scent peak.

**Change — three separable, individually-gateable mechanisms in `MotorEPM` (cell-gated):**
- **Food-attraction (forage).** The existing cell nav-steer (`percept.scent_compass` bearing →
  flagella differential) is now **scaled by hunger** (`reality.proprio.hunger = 1−energy`):
  `st = nav_gain · bearing · conf · hunger`. A **sated** bug ignores food and keeps exploring
  (no pull, no linger); a **starving** bug homes hard. Need-driven (homeostatic), not a fixed
  stimulus→response reflex. `conf = clamp(|grad|·4)` keeps it gentle-far / strong-near.
- **Desperation (escalation).** `boredom_streak` counts ticks boredom stays > 0.5; the escape
  amplitude grows `×(1 + boredom_escalation_rate · streak)` (capped ~5×) and is further
  multiplied by `(1 + hunger)`. "Do more the longer you're stuck, and faster when hungry."
  Resets the instant boredom relaxes (freed / getting warmer) → self-terminating, not a runaway.

**Result — forage ON vs OFF (`nav_gain` 1 vs 0), drain 0.02/s, n=3 seeds (5/6/7):**

| seed | ON hits | OFF hits | ON e_mean | OFF e_mean | OFF starved? |
|------|---------|----------|-----------|------------|--------------|
| 5    | **6**   | 3        | 0.77      | 0.67       | no           |
| 6    | **6**   | 3        | 0.81      | 0.41       | **yes** (e→0)|
| 7    | **3**   | 1        | 0.42      | 0.31       | **yes** (e→0)|

Forage ON ≈ **2× the food contacts** at every seed and **avoids starvation** — the
hunger-gated scent steer is **directed**, not random. (Boredom escalation confirmed live:
`boredom_streak` reaches 25–57, `boredom` peaks 0.99 then resets.)

**Critical calibration — `energy_drain` (the survival difficulty knob).** This was the gating
discovery: at the bootstrap value **0.003/s** the bug stays at ~0.97 energy and **hunger never
exceeds ~0.07**, so the entire forage/desperation stack sits **inert** (each +0.4 hit only needs
repaying every ~130 s). The mechanisms are dead unless the bug is actually hungry. Swept the
drain (n=3 forage-ON):

| drain/s | hngr_max | starved seeds | reading |
|---------|----------|---------------|---------|
| 0.003   | 0.07     | 0/3           | trivial — never hungry, forage inert |
| **0.012** | **0.22–0.40** | **0/3**   | **genuine hunger, survivable by foraging** ✓ |
| 0.02    | 0.66     | 1/3 (s7)      | harsh — starves before the critic learns |

Set the config default to **0.012/s** (which is also the code's own historical default): real
homeostatic pressure that the innate forage drive survives across seeds, leaving headroom for the
cognitive critic to learn to do *better*. Committed config (no env override) reproduces: seed 5 =
4 hits, e_min 0.60, **no starvation**, hunger oscillates to 0.40, critic steer spans ±1.

> **Note.** Forage-as-substrate (innate hunger-gated chemotaxis) is the *fast-path* competence
> that keeps the bug alive long enough; the ActionDecoder critic is still expected to do the
> *learned* homing on top (Stage-A goal). This is the same architecture as the picrawler
> (alive Cruse substrate + cognitive steer), not a return to the pure chemotaxis port.

---

## What is NOT yet emergent (the next build)

The escape is currently an **undirected** held random turn: it keeps the bug moving but has no
reason to pick "away from the wall," so the bug *wall-follows* the bottom edge (z pinned at −7.6,
never enters the interior) and burns energy thrashing → still starves. The signal sees the
stuckness; the *action* isn't yet directed.

**Next: ③ curiosity-directed escape.** Replace the random turn with run-and-tumble toward
`interest = scent-novelty + whisker-clearance` (validated separators: scent novelty 4.2×
open-vs-wall; clearance 0.35 vs 0.64) so the now-active escape is *steered* into open / scent-rich
space. **④ MotorFader fast/slow arbitration** (escape ↔ cognition) is banked behind it.

---

## Reproduce

```bash
OGMA_CELL_CONFIG=res://addons/ami_ogma/configs/the_cell_cognitive.json \
OGMA_REFLEX_MODULAR=1 OGMA_BODY_MODEL=bidirectional_paddler \
OGMA_MOTOR_BASELINE_BEAT=1 OGMA_MOTOR_ENERGY=1 \
OGMA_SEED=1 OGMA_TURBO=1 OGMA_EPISODE_LENGTH=5400 OGMA_QUIT_AFTER_TICKS=5460 \
godot4 --path godot_host/project --headless --fixed-fps 240 --disable-render-loop \
  res://scenes/the_cell.tscn
```
Analyze pins by **net position displacement**, not `fwd_v` (which is local thrust and stays
non-zero while wedged). UI: HUD now shows **energy** and **boredom** live.

## Files

- `godot_host/project/scripts/body_controller.gd` — `bidirectional_paddler` model; afferent IMU
  + efference-copy publish; no-progress diag.
- `cpp_core/.../modules/DistressDrive.{hpp,cpp}` — reafference mismatch + no-progress combiner →
  `cognition.boredom` (registered in ModuleRegistry + CMakeLists).
- `cpp_core/src/ogma/modules/MotorEPM.cpp` — boredom consumer (cog-steer fade + escape turn);
  **hunger-gated forage steer** + **boredom-streak desperation** (Win 3); `hunger`/`boredom_streak`
  diag. Cell-gated, default-off (picrawler/Stage-1 bit-identical).
- `godot_host/project/addons/ami_ogma/configs/the_cell_cognitive.json` — distress + forage stack
  wiring (`nav_gain=1`, `hunger_topic`, `boredom_escalation_rate`, **metadata `energy_drain=0.012`**).
  `the_cell_cognitive_noboredom.json` = escape-off A/B arm; `the_cell_cognitive_noforage.json` =
  forage-off (`nav_gain=0`) A/B arm.
- `godot_host/project/scripts/hud.gd` — energy + boredom readouts.
- Memory: `[[v6-cell-cognitive-stagea]]` (live state), `[[v6-reasoning-breakthrough-resume]]`.
