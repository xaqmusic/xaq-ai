# LateralVoter — Primitive Contract

**Phase 1 dependency position:** 3 of 9 (depends on EPM).
**Header:** `cpp_core/include/ogma/modules/LateralVoter.hpp` (Phase 1 deliverable).
**Reference impl:** `src/ami_ogma_v3/lateral_voter_v3.py:79+`.

---

## Purpose

The LateralVoter aggregates per-tick `RealityToken` payloads from every EPM at one level and publishes a single fused `ConsensusToken`. Three jobs:

1. **Trust-weighted fusion of latents.** Each subscriber's contribution to the fused embedding is weighted by `1/(tle + ε)` (low TLE → high trust), with **modality-group balancing** so a high-cardinality group (e.g. multiple visual EPMs) doesn't drown out a low-cardinality group (e.g. one cochlear EPM).
2. **Active-modality selection.** Reports which group's most-active member won the position-encoding priority used by ActionDecoder. v3's "action_modality" — preserves the proprio-priority pattern that v3 relies on for embodied environments.
3. **Hebbian association across modalities.** Maintains an `association_matrix` keyed `(winner_a, winner_b) → strength` updated by outer products of co-activated winners. v3 status: stub for Phase 5; v4 keeps it stub-implemented but exposes the param schema so Phase 3 can promote it.

Modality-group balancing is the single most load-bearing piece of the voter. v3's `_group_proportional_trust` parses each EPM's modality group from a config map; v4 parses it from the topic name (`reality.<group>.<modality>`) using the hierarchical namespace. This is the one consequential consumer of the namespace decision.

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `reality.` | Direct | `RealityToken` | every EPM | yes (≥1 match) | Wildcard prefix subscription. Each delivery comes with the actual topic name (`reality.video.retinal`, etc.); the voter parses `<group>` from the second segment. |
| `neuro.state` | Direct | `NeuroState` | NeurochemState | no | Optional — if present, modulates the trust softmax temperature via `dopamine`. |
| `prediction.consensus.<level>` | Feedback | `PredictionToken` | DescendingPredictor (Phase 1+) | no | Same prediction-subtraction pattern as EPM, applied to the fused embedding before publish. Off in Phase 2 thin-slice. |

Higher-level voters (`level ≥ 1`) subscribe to `consensus.<level-1>` instead of `reality.` — i.e. the input pattern is `params.input_pattern`, defaulting to `reality.` for level 0.

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `consensus.<level>` (e.g. `consensus.0`) | `ConsensusToken` | every tick |

`ConsensusToken` fields (Topics.hpp):

- `fused_embedding` — weighted sum of inputs' `latent` vectors, all share the same `projection_dim`.
- `fused_tle` — weighted average of inputs' `tle`, using the same trust weights.
- `level` — copy of `params.level`.
- `active_modality` — string identifier of the highest-trust modality group's most-active member.
- `active_winner_id` — that member's `winner_id`.
- `trust_weights` — map of full topic name (e.g. `reality.video.retinal`) to applied trust weight.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `level` | int64 | ConstructionOnly | 0 | [0, 16] | Identifies which `consensus.<n>` topic to publish on. |
| `input_pattern` | string | ConstructionOnly | `reality.` | — | Trailing-dot prefix subscribed for input tokens. Level-1 voters use `consensus.0.`. |
| `trust_mode` | string | HotMutable | `tle_inverse` | — | Currently the only supported mode. Reserved for Phase 3 alternates. |
| `trust_epsilon` | double | HotMutable | 0.05 | (0, 1] | Smoothing in `1/(tle + ε)`. |
| `activity_gain` | double | HotMutable | 0.0 | any | **Activity term** ([Kalman-lessons charter](../epm_kalman_lessons_plan.md), Stage 2). Raw trust is multiplied by `activity^gain`, where activity = EMA of the channel's latent tick-to-tick displacement divided by its own decaying peak, in [0, 1]. A channel that stops moving (dead sensor, frozen camera, stale republished token) loses trust in EMA time instead of becoming the most trusted because it is trivially predictable, the inversion doctrine §2.3 names. 0 = byte-identical (measured, 40/40 bench files). Measured 2026-09-05: bench dead sensor's trust 0.66 → 0.002 and fused error after death ÷4.4; Cell maze fusion with a camera frozen on a plausible bearing, n = 20 worlds: the freeze costs 5.9 eats, gain 1 recovers 4.6 ± 2.1 of them, no cost when nothing is wrong, gain −1 is worse than nothing. It is not a noise discriminator (a noisy channel moves); that is the expected-error lever's job. |
| `activity_alpha` | double | HotMutable | 0.1 | (0, 1] | EMA rate of the displacement; sets the strip time (≈ 2/α ticks, measured 21 ± 4 on the bench). |
| `activity_peak_decay` | double | HotMutable | 0.999 | (0, 1) | Per-tick decay of the running peak the activity is normalised by. |
| `activity_floor` | double | HotMutable | 0.001 | (0, 1] | Lowest activity factor applied; keeps `activity^gain` finite for the wrong-sign arm. |
| `group_balance` | bool | HotMutable | true | — | If true, normalize trust within each modality group, then equally weight groups. If false, plain TLE-inverse softmax. |
| `softmax_temperature` | double | HotMutable | 1.0 | (0, 10] | Trust softmax temperature (lower → sharper). Modulated by `1 + 0.5 * neuro.state.dopamine` when `neuro.state` is present. |
| `priority_group` | string | HotMutable | `proprio` | — | The modality group that wins the active-modality tie. Mirrors v3's "proprio always wins" pattern for embodied benchmarks. |
| `association_enabled` | bool | HotMutable | false | — | Off in Phase 1; enables the Hebbian outer-product update in Phase 3. |
| `association_decay` | double | HotMutable | 0.9999 | [0.99, 1.0) | Per-tick decay on association matrix entries. |
| `association_max_size` | int64 | HotMutable | 10000 | [100, 1e6] | Cap on `(winner_a, winner_b)` entries; LRU evict beyond. |
| `novelty_threshold` | double | HotMutable | 0.35 | (0, 1] | If `fused_tle > novelty_threshold`, the published consensus is flagged as a candidate for higher-level mitosis. (Phase 4 fractal mitosis uses this.) |
| `master_seed` | int64 | ConstructionOnly | 0 | — | RNG namespace `voter.<level>` for the trust-tie-break stochasticity. |

---

## Invariants (per tick)

1. The voter publishes exactly one `ConsensusToken` per tick.
2. `fused_embedding.dim == every_input.latent.dim` — the voter rejects the OgmaInstance at construction if EPM outputs declare mismatched `projection_dim`.
3. `Σ trust_weights[topic] == 1.0` (within `1e-6`).
4. When `group_balance == true`: for every modality group g present in the inputs, `Σ_{topic ∈ g} trust_weights[topic]` is the same across groups (within `1e-6`).
5. `active_modality` belongs to the modality group with the highest within-group trust mass. Ties broken by `priority_group` first, then deterministic seed-based tie-break.
6. `active_winner_id` is the winner_id of the EPM whose `latent` had the smallest `quant_error` in the active modality.
7. If no input was delivered this tick (all EPMs late or absent), the voter republishes the previous tick's `ConsensusToken` with an updated `tick_id`. This avoids gaps in `consensus.<level>` for downstream subscribers.
8. The association_matrix update (when enabled) is symmetric: `assoc[a][b] == assoc[b][a]` after every update.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: no `reality.*` deliveries yet | Publish a placeholder `ConsensusToken` (zero embedding, `fused_tle = 0`, `active_modality = ""`). |
| Mid-run: zero deliveries this tick | Republish previous tick's token with new `tick_id` (Invariant 7). |
| Input dim mismatch | Throw at construction. Mid-run reconfiguration via hot-patch must be vetted by the Scheduler before applying. |
| Duplicate topic delivery in one tick (subscription accidentally registered twice) | Take the latest delivery only; log warning. |
| `dopamine` value out of range from `neuro.state` | Clamp to [0, 1]; do not throw (NeurochemState's invariant should prevent this). |
| Association matrix at `association_max_size` | Drop the lowest-strength entries before insertion. |

---

## Latency Budget (Pi5 Cortex-A76, full module graph)

For 5 EPM inputs at 128-D:

- **Hot path (`tick()`):** ≤ 80 µs. Trust-weight computation is `O(n_inputs)` scalar operations; fused embedding is `O(n_inputs * projection_dim)` floating-point ops.
- **With `association_enabled = true`** and `n_winners = 5`: + ~40 µs for the outer-product update.
- **Construction:** ≤ 100 µs.
- **Memory:** dominated by `association_matrix` when enabled (~`association_max_size * 8 bytes`).

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Trust normalization:** synthesized inputs with TLE = [0.1, 0.2, 0.4] from three modalities yield `Σ trust = 1.0` and the correct relative ordering.
- **Group balancing:** with two `reality.video.*` inputs and one `reality.audio.*` input, `Σ video_trust == Σ audio_trust` (within `1e-6`) when `group_balance == true`, and is proportional to count when `group_balance == false`.
- **Topic parsing:** `parse_modality_group("reality.video.retinal") == "video"`, `parse_modality_group("consensus.0.x") == "consensus"` etc., for all topic patterns the voter encounters.
- **Active modality tie-break:** with two equally-active modalities, `priority_group` decides; with no priority match, the deterministic seed-based tie-break is reproducible across runs with the same `master_seed`.
- **Empty-tick republish:** stepping the voter with no inputs after a populated tick republishes the previous token with new `tick_id`.
- **Association symmetry:** after 1000 stochastic updates, `assoc[a][b] == assoc[b][a]` for every populated pair.

### 2. Pair tests

- **`pair_epm_lateralvoter`** (declared in EPM.md): one EPM publishes `reality.video.retinal`; the voter publishes `consensus.0` with the EPM's latent at trust 1.0 and `active_modality == "video"`.
- **`pair_lateralvoter_actiondecoder`**: voter publishes a synthetic `consensus.0`; ActionDecoder consumes it and produces a deterministic action (verifies the consensus → decoder seam).

### 3. Latency

`tick()` p99 ≤ 80 µs on Pi5 with 5 inputs over 10⁴ runs.

### 4. Determinism

Two runs with the same `master_seed` and identical input streams produce bitwise-identical `ConsensusToken` sequences.

### 5. Modality-group regression

With one `reality.video.retinal`, one `reality.video.saliency`, one `reality.audio.cochlear`, and one `reality.proprio.imu`, no single group's combined trust exceeds 30% across 10⁴ ticks of randomized TLE inputs (with `group_balance == true`).

---

## Notes on the v3 → v4 Port

- **Topic-name parsing replaces `_modality_group_map` config.** v3's voter held a hand-maintained dict mapping each EPM ID to its modality group. v4 derives the group from the topic name via the hierarchical namespace; no separate config.
- **Replaces `MultiEPMAdapter`'s fan-in.** v3's MultiEPMAdapter aggregated `_latest_tokens` and called the voter with the dict. v4 has the voter consume directly from the Bus.
- **Higher-level voters reuse the same code.** A `level=1` LateralVoter subscribes to `consensus.0.` (prefix) and publishes `consensus.1`. Same algorithm.
- **Position-encoding priority.** v3's `action_modality` defaults to proprio (the body-state EPM always wins ties when present). v4 preserves this via `priority_group = "proprio"`. If the body-EPM is absent, the priority falls through to the highest-trust group, then the deterministic seed tie-break.
- **Hebbian association_matrix is stub-only in Phase 1.** The schema is wired so Phase 3 can promote it without a contract amendment.
