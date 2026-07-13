# SequenceGNG — Primitive Contract

**Phase 1 dependency position:** 7 of 9.
**Header:** `cpp_core/include/ogma/modules/SequenceGNG.hpp` (Phase 1 deliverable).
**Reference impl:** none — new primitive (`docs/v4_algorithmic_gaps.md` Primitive 2). Reuses `cpp_core/include/v3/gng.hpp` as the underlying clusterer.

---

## Purpose

A SequenceGNG clusters **n-grams of winner transitions** rather than single states. v3's `RealityToken.history_trace` (last 5 winner IDs) is wasted — used as a bag-of-features key but never compressed. SequenceGNG observes the rolling window and, when a sub-sequence recurs frequently enough, bakes it as a "motif node" whose prototype encodes the transition pattern and whose metadata tracks the predicted next transition.

Used by:

- **GNGRollout** for motif-teleport: when a rollout's current state matches a baked motif at phase k, the rollout can skip to the motif's terminal in O(1).
- **MotorRepertoire** for chunk-boundary detection: action-stream motifs that correlate with drive-error reduction become candidate chunks.
- **ActionDecoder** (future) for chunk chaining via composition of motif IDs.

The encoder is windowed concatenation of recent winners' prototypes, then a JL projection. The clusterer is a regular GNG (reusing `cpp_core/include/v3/gng.hpp`) with motif-specific metadata stored in a side-table keyed on baked node ID.

---

## Input Topics

The SequenceGNG is configurable to wrap *any* stream of `RealityToken` or `ActionOut` payloads. For Phase 1 the typical instances are:

- One per `consensus.<level>` (motif-learn over the voter's output)
- One on `action.out` (motif-learn over the decoder's output, feeding MotorRepertoire)

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `params.source_topic` | Direct | `RealityToken` *or* `ConsensusToken` *or* `ActionOut` | varies | yes | Subscribed exactly. Type is determined by the source; the windowing logic uses the appropriate scalar (winner_id for tokens, accel for actions). |
| `neuro.state` | Direct | `NeuroState` | NeurochemState | no | Optional GNG hyperparameter modulation, identical to EPM. |

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `sequence.motif.<source>` | `SequenceMotif` | every tick |

The trailing `<source>` is `params.id` — typically `consensus.0` or `action.out`, shaped to be human-readable in trace logs.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `source_topic` | string | ConstructionOnly | — | — | The subscribed stream. Required. |
| `source_kind` | string | ConstructionOnly | `winner` | `winner` / `action` | Determines whether the windowed input is winner_id sequences (for tokens) or scalar actions. |
| `window_size` | int64 | ConstructionOnly | 5 | [2, 64] | n-gram length. Resolved Phase-0 default = 5 (matches v3 `history_trace`). |
| `projection_dim` | int64 | ConstructionOnly | 128 | [16, 1024] | JL output dim, also GNG input dim. |
| `prototype_per_winner_dim` | int64 | ConstructionOnly | 32 | [8, 256] | Each winner's prototype is compressed to this dim before concatenation. (For `source_kind = action`, this is ignored — action scalars are concatenated directly.) |
| GNG params (`baking_threshold`, `min_insertion_error`, `lambda_new`, `max_age`, `epsilon_b`, `epsilon_n`, `alpha`, `beta`, `max_nodes`, mitosis params, health params, stale-prune params) | various | HotMutable | match EPM defaults | — | Exact same shape as EPM's GNG configuration. |
| `motif_branching_threshold` | double | HotMutable | 0.4 | (0, 1] | When a baked motif's "predicted_next" entropy exceeds this, mitosis splits it into sub-motifs (matching v3 EPM mitosis but with the entropy criterion instead of mean error). |
| `master_seed` | int64 | ConstructionOnly | 0 | — | RNG namespace `seqgng.<id>` for the JL random matrix. |

---

## Invariants (per tick)

1. SequenceGNG publishes exactly one `SequenceMotif` per tick.
2. After the rolling window has filled (tick ≥ `window_size`), a winner is reported (`motif_id ≥ 0` if a baked motif matches, else `-1`).
3. `phase ∈ [0, motif_length)` when `motif_id ≥ 0`; `phase = 0` when `motif_id = -1`.
4. `match_confidence ∈ [0, 1]`; ≥ 0.5 implies a strong match.
5. `predicted_next_id` is the most-frequent successor winner_id observed for the active motif (or `-1` if no successor data yet).
6. `just_baked = true` exactly on the tick a new motif crystallizes; never repeats for the same motif.
7. The motif metadata side-table is purged consistently when GNG nodes are pruned via the underlying GNG's prune logic.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First `window_size - 1` ticks: window not yet full | Publish placeholder `SequenceMotif{motif_id = -1, phase = 0}`. |
| `source_topic` payload type mismatches `source_kind` | Throw at construction (validate type at subscription registration). |
| Source produces winner_id = -1 (the EPM bootstrap placeholder) | Skip this entry in the window; do not advance the rolling buffer. Window growth pauses until winner_id ≥ 0 arrives. |
| Motif metadata side-table grows beyond max_nodes | LRU evict by `last_match_tick`. |

---

## Latency Budget (Pi5 Cortex-A76)

- **Hot path:** ≤ 200 µs. Window concat + JL projection + GNG step.
- **Construction:** ≤ 50 ms (JL matrix build).
- **Memory:** `max_nodes * projection_dim * 4` bytes for prototypes plus motif metadata side-table.

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Window fill:** ticks 0..`window_size-2` publish placeholders; tick `window_size-1` produces a real motif report.
- **Recurrent motif baking:** a deterministic 100-step sequence with a recurring 5-step pattern produces ≥ 1 baked motif within 2 × `baking_threshold` ticks.
- **Phase tracking:** when the active motif is matched at position 2 of 5, `phase = 2`; advances to 3 on the next match.
- **Predicted next:** for a deterministic ABCDE motif, after baking, `predicted_next_id` for state `D` equals winner_id `E`.
- **Mitosis on branching:** a motif whose successor distribution becomes bimodal triggers mitosis within `mitosis_check_interval` post-bake visits.
- **GNG-prune integration:** when a winner_id in the window is reported as pruned by the source EPM, the motif side-table entries depending on it are evicted.

### 2. Pair tests

- **`pair_seqgng_consensus`**: SequenceGNG subscribed to `consensus.0` from a synthetic LateralVoter producing a known winner sequence; verifies motif baking on the integration seam.
- **`pair_seqgng_actiondecoder`** (Phase 1+, when MotorRepertoire is wired): SequenceGNG over `action.out` produces motifs that downstream MotorRepertoire chunk extraction subscribes to.

### 3. Latency

`tick()` p99 ≤ 200 µs on Pi5 with `window_size = 5`, 128-D projection.

### 4. Determinism

Same `master_seed` + same input stream → same baked motif IDs and prototype values.

### 5. Behavioural target

In a synthetic environment with a clear recurring action sequence (e.g. a wall-following pattern in the maze), SequenceGNG bakes ≥ 3 motifs within 5000 ticks, and the most-active baked motif's `match_confidence` averages ≥ 0.7 over the second half of the run.

---

## Notes

- **Window size = 5 is the resolved Phase 0 default** (Open Question #2 in the planning resolution). Adaptive windows are a Phase 3 experiment.
- **`source_kind = action` is the path used by MotorRepertoire** for chunk extraction. The same primitive serves both purposes — a deliberate consequence of "stack the same algorithm at different scales."
- **No new GNG implementation.** This module wraps the v3 GNG (`cpp_core/include/v3/gng.hpp`) and adds the n-gram windowing in front of it.
