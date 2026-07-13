# HomeokineticExploration — Primitive Contract

**Phase 1 dependency position:** 10 of 10 (post-Phase-5 dead-zone primitive).
**Header:** `cpp_core/include/ogma/modules/HomeokineticExploration.hpp`.
**Reference impl:** none — new primitive (post-Phase-5, addresses the
dead-zone gap documented in `docs/v4_cell_competence.md`).

---

## Purpose

The five primitives of `v4_algorithmic_gaps.md` produce a **gradient-following**
organism. Outside a detectable gradient — or when the body is mechanically
wedged against a wall — the existing modules degenerate to a fixed point.

HomeokineticExploration is the failure-detector that breaks the lock. It
watches the **observable failure** of the modules whose job it is to act
(the change-rate of urgency over time) and, when current change is
anomalously low compared to recent history, fires a stochastic-motility
episode. ActionDecoder yields its action selection to the directive for
`episode_ticks`; the body moves ballistically along a held heading-rate;
on episode exit, normal control resumes.

This primitive **replaces** the Probe machinery in ActionDecoder.

---

## Design principle: no behavior thresholds, only physical horizons

**Every threshold in this primitive is derived from the system's own running
statistics — not from hand-tuned constants.** This is a homeokinetic
commitment: if a parameter requires per-environment tuning, it doesn't
belong in the contract.

The discriminator is **`current_ratio < anomaly_factor × median(historical_ratios)`**:
- `current_ratio` = mean `|Δurgency|` over the last `window_ticks`, divided
  by the long-running EMA of `|Δurgency|`
- `median(historical_ratios)` = median of the per-tick ratios stored in a
  sliding `long_window_ticks` buffer
- `anomaly_factor` = 0.5 (universal "half-of-typical" anomaly convention)

This formulation is robust at any urgency level (saturated, mid, low) and
self-disengages when stuck becomes the new normal: as the buffer fills with
low ratios, the median drifts down too, and "low" eventually stops being
anomalous. No infinite kinesis spam in true dead zones.

The remaining parameters (`window_ticks`, `long_window_ticks`,
`change_ema_alpha`, `episode_ticks`, `cooldown_ticks`, `accel_jitter`) are
**physical horizons or kinematic limits**, not behavior thresholds. They
correspond to:
- "1–8 second time horizons" at 60 Hz physics (window/long_window/EMA)
- "1 s held heading commitment" (episode_ticks)
- "0.5 s pause" (cooldown_ticks)
- "moderate steering bias" (accel_jitter, bounded by `accel_max=4`).

These are not run-specific tuning targets; they are time-scales of the
underlying physics.

---

## Companion: body-side stuck reflex

The cognitive primitive operates in tandem with a body-side stuck detector
in `body_controller.gd` that uses the **body's own observation of its
typical |Δposition|/tick** to detect mechanical wedges:

- **`deficit = max(0, 1 - actual_displacement / max_possible_displacement)`**
  over a 1-second sliding ring buffer
- When `deficit > 0`, inject a held random rotation pulse (resampled every
  ~0.5 s) scaled by deficit
- No whisker thresholds, no hysteresis math — Godot's `move_and_slide`
  handles wall collision; deficit-driven rotation finds the escape heading
  ballistically.

This split is principled: the body handles physical wedges via local
proprioceptive feedback; the cognitive primitive handles informational
dead-ends (no urgency change) via Bus statistics. Neither has hand-tuned
behavior thresholds.

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `drive.errors` | Direct | `DriveErrors` | HomeostaticDrive | yes | Source of urgency. |
| `motor.chunks` | Direct | `MotorChunks` | MotorRepertoire | no | Top-chunk outcome above `chunk_match_eps` suppresses gate. |
| `action.out` | Direct | `ActionOut` | ActionDecoder | no | If `chunk_remaining_ticks > 0`, gate is suppressed. |

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `exploration.directive` | `ExplorationDirective` | every tick |

---

## Parameter Schema

| Key | Type | Mutability | Default | Notes |
|---|---|---|---|---|
| `window_ticks` | int64 | HotMutable | 120 | Short-window length for current ratio (~2 s). |
| `long_window_ticks` | int64 | HotMutable | 480 | Long-history buffer for percentile baseline (~8 s). |
| `change_ema_alpha` | double | HotMutable | 0.002 | EMA rate of `|Δurgency|`. |
| `anomaly_factor` | double | HotMutable | 0.5 | Gate threshold = factor × median(historical ratios). |
| `chunk_match_eps` | double | HotMutable | 0.01 | Top-chunk outcome threshold for chunk-applicable. |
| `episode_ticks` | int64 | HotMutable | 60 | Episode length (~1 s ballistic motion). |
| `cooldown_ticks` | int64 | HotMutable | 30 | Pause after episode (~0.5 s). |
| `accel_jitter` | double | HotMutable | 1.0 | Random accel range (¼ of `accel_max`). |
| `master_seed` | int64 | ConstructionOnly | 0 | RNG namespace seed. |

---

## VV&A Criteria

### 1. Unit tests (`test_homeokinetic_exploration.cpp`, 14 tests)

- Cold-start guards (window not full, long buffer not full)
- Adaptive arming (anomalous low ratio after history of varied ratios)
- Wall-stuck scenario (saturated flat after foraging — gate fires)
- Episode duration + cooldown
- Failing gating conditions (chunk applicable, in-flight chunk, sustained
  high change rate, average urgency)
- Cooldown enforcement
- Outcome-feedback diagnostics (success_rate tracked)
- Determinism (same seed → bit-identical samples)

### 2. Pair test (`pair_homeokinetic_actiondecoder.cpp`)

ActionDecoder yields to directive when active; resumes EFE on episode end.

### 3. Behavioural validation

| Config | hits/run | seeds | notes |
|---|---|---|---|
| Easy (16 m, 3 nut, σ=4) | 1.40 ± 1.36 | 4/5 | Within [1.0, 3.5] baseline |
| Hard (24 m, 1 nut, σ=4) | 0.20 | 1/5 | Improvement vs 0/5 baseline |
| σ=8 sanity (24 m, 1 nut, σ=8) | 0.20 | 1/5 | Below 0.80 baseline (kinesis intrudes during exploration) |
| **3-min UI run, default seed** | **3 hits, 15 episodes** | n=1 | **No permanent wedges; body covers the full room.** |

The headline acceptance criterion is the UI behavior: previous runs
permanently wedged at corners. The current implementation has the body
auto-extracting from any geometry within ~1 s, with kinesis firing only
when the cognitive failure signal warrants it.

---

## Notes

- **No more probe.** The v3 satiety-curiosity Probe in ActionDecoder was
  removed. Its parameters (`probe_serotonin_threshold`,
  `probe_dopamine_threshold`, `probe_duration_ticks`) are gone.
- **Body backup-on-wedge has no hand-tuned thresholds.** The previous
  hysteresis (0.55 trip, 0.30 release, -0.5 backup) has been replaced with
  a proprioceptive deficit detector that uses the body's own typical
  motion as the reference. See `body_controller.gd` for the
  implementation.
- **Outcome-feedback adaptation is currently passive.** `success_rate_ema`
  is tracked for diagnostics but does not adjust episode parameters —
  initial implementation found that the "did urgency drop" signal is too
  rare in the cell env to drive useful adaptation, and inflating episode
  length on every "failure" pushed the body away from faint gradients in
  the σ=8 case. A Phase-3.5 extension may revisit with a better outcome
  signal (e.g., body-side `Δposition during episode > 0` ⇒ success).
