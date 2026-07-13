# EPM — Primitive Contract

**Phase 1 dependency position:** 2 of 9 (depends on NeurochemState).
**Header:** `cpp_core/include/ogma/modules/EPM.hpp` (Phase 1 deliverable).
**Reference impl:** `src/ami_ogma_v3/epm.py:84–312`, `cpp_core/include/v3/epm.hpp`, `cpp_core/src/v3/epm.cpp`.

---

## Purpose

An EPM (Episodic Predictive Module) is the bath's per-modality clusterer. It owns:

1. **A frozen encoder** — JL projection (visual), STFT filterbank (audio), or RBF grid (proprioceptive). Stateless except for the encoder's per-band state. Selected at construction time by `params.modality`.
2. **A GNG topology** — Growing Neural Gas with two-gate baking, mitosis, biological health, and stale-prune. State is per-instance and serializable. Reuses `cpp_core/include/v3/gng.hpp` directly.
3. **A dual-TLE estimator** — `tle = α·quant_error + β·transition_surprise`. The combined error drives mitosis decisions and is the headline scalar published in `RealityToken`.

The EPM's output is one `RealityToken` per tick, published on `reality.<group>.<modality>` (e.g. `reality.video.retinal`). v4 adds two new behaviours absent in v3:

- **Top-down prediction subtraction.** Before encoding, the EPM reads `prediction.<modality>` (Feedback subscription, prior-tick) from a DescendingPredictor and subtracts the predicted latent from its current encoder output. The GNG topologizes surprise, not raw observation.
- **Level-N stacking via input source swap.** A Level-1 EPM is the same code as a Level-0 EPM with `params.input_topic` set to `consensus.0` instead of `reality.proprio.<sensor>`/etc. The encoder is the identity passthrough (or optional 128→128 JL rotation per Open Question #7 in `v4_refactor.md`).

---

## Input Topics

The EPM is the most modality-shaped module — its input subscriptions depend on `params.modality` and `params.input_topic`.

| Pattern (concrete examples) | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `params.input_topic` (e.g. `reality.proprio.imu` or `host:reality.video.retinal`) | Direct | `ProprioToken` *or* `RealityToken` *or* host-specific raw frame | host (Level 0) or LateralVoter (Level ≥1) | yes | The single observation channel. Type is determined by the encoder selected. |
| `neuro.state` | Direct | `NeuroState` | NeurochemState | yes | Current-tick scalings — applied to GNG epsilon_b, min_insertion_error, mitosis threshold, novelty threshold before the GNG step. |
| `prediction.<this-modality>` | **Feedback** | `PredictionToken` | DescendingPredictor (when present) | no | Subtracted from the encoder output before the GNG sees it. Absent in Phase 2 thin-slice (DescendingPredictor lands later in Phase 1). |

Note the asymmetry with `NeurochemState`: EPM reads `neuro.state` as **Direct (current-tick)**, while NeurochemState reads `reality.<modality>` as **Feedback (prior-tick)**. This is the resolution of the cycle (see NeurochemState.md "Why the EPM cycle is Feedback").

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `reality.<group>.<modality>` (e.g. `reality.video.retinal`, `reality.audio.stft`, `reality.proprio.imu`, `consensus.1.<source>`) | `RealityToken` | every tick |

The exact topic name is derived from `params.modality_group` and `params.modality_name` at construction time. For Level-1 EPMs that subscribe to `consensus.0`, the output topic is `consensus.1.<id>` where `<id>` distinguishes Level-1 EPMs that subscribe to the same Level-0 consensus.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `modality_group` | string | ConstructionOnly | — | `video`/`audio`/`proprio` | Determines the topic-name prefix. Required. |
| `modality_name` | string | ConstructionOnly | — | — | Trailing component of the output topic. Required. |
| `encoder_kind` | string | ConstructionOnly | — | `jl`/`stft`/`rbf`/`identity` | Selects the encoder backend. Required. Identity is for Level-N EPMs. |
| `input_topic` | string | ConstructionOnly | — | — | The single subscribed observation topic. Required. |
| `projection_dim` | int64 | ConstructionOnly | 128 | [16, 1024] | Encoder output dim = GNG input dim. |
| `baking_threshold` | int64 | HotMutable | 50 | [10, 500] | Visit count required to bake a node. |
| `min_insertion_error` | double | HotMutable | 0.02 | (0, 1] | Auto-tuned each `lambda_new` ticks; this is the floor. |
| `lambda_new` | int64 | HotMutable | 25 | [5, 200] | Steps between insertion-error reviews. |
| `max_age` | int64 | HotMutable | 88 | [10, 500] | Edge max age before pruning. |
| `epsilon_b` | double | HotMutable | 0.05 | (0, 1] | Winner learning rate (modulated by `neuro.state.epsilon_b_scale`). |
| `epsilon_n` | double | HotMutable | 0.003 | (0, 1] | Neighbour learning rate. |
| `alpha` | double | HotMutable | 0.5 | (0, 1] | Error halving on insertion. |
| `beta` | double | HotMutable | 0.0005 | (0, 1] | Global error decay per step. |
| `max_nodes` | int64 | HotMutable | 2000 | [50, 50000] | GNG capacity. |
| `tle_alpha` | double | HotMutable | 0.7 | [0, 1] | Weight of `quant_error` in dual TLE. |
| `tle_beta` | double | HotMutable | 0.3 | [0, 1] | Weight of `transition_surprise`. (`tle_alpha + tle_beta` need not sum to 1 — they are independent gains.) |
| `mitosis_enabled` | bool | HotMutable | true | — | — |
| `mitosis_error_threshold` | double | HotMutable | 0.30 | (0, 1] | Post-bake mean error to trigger split (modulated by `neuro.state.mitosis_threshold_scale`). |
| `mitosis_check_interval` | int64 | HotMutable | 50 | [5, 1000] | Post-bake visits between checks. |
| `health_base_decay` | double | HotMutable | 0.997 | [0.9, 1.0) | Per-tick health decay multiplicand. |
| `stale_prune_enabled` | bool | HotMutable | true | — | — |
| `stale_window_factor` | double | HotMutable | 12000.0 | [100, 1e6] | Steps before a non-revisited node is prune-eligible. |
| `subtract_descending_prediction` | bool | HotMutable | true | — | If true and `prediction.<modality>` is present, subtract before GNG. |
| `master_seed` | int64 | ConstructionOnly | 0 | — | Seeds the JL random matrix and any GNG stochastic operations. Forwarded via `_rng.derive_rng(seed, "epm.<id>")`. |
| Encoder-specific params (audio `f_min`/`f_max`/`sample_rate`/``; visual `encoder_res`/`inject_centroid`/`centroid_gain`; proprio `proprio_state_dims`/`proprio_dim_ranges`) | various | ConstructionOnly | — | — | Forwarded to the encoder constructor. See `cpp_core/include/v3/types.hpp:EPMConfig`. |

---

## Invariants (per tick)

1. EPM publishes exactly one `RealityToken` per tick (no skips, no duplicates).
2. `RealityToken.tick_id == Scheduler::current_tick()`.
3. `winner_id ≥ 0` after the GNG bootstrap completes (first 2 ticks publish the placeholder `winner_id = -1`; see `_first_tick.md`).
4. GNG node IDs are monotonically allocated; once issued, an ID is never reused even after pruning. `pruned_ids` reports ID numbers, not slot numbers.
5. `tle = tle_alpha * quant_error + tle_beta * transition_surprise`. No other formula.
6. `is_novel = (quant_error > novelty_threshold * neuro.state.novelty_threshold_scale)`. Always.
7. `latent` and `winner_prototype` have identical dimension `params.projection_dim`.
8. When `subtract_descending_prediction == true` and a `prediction.<modality>` payload was delivered (Feedback) for the prior tick, the encoder output passed to the GNG is `encode(observation) - prediction.predicted_latent`. Otherwise the encoder output is `encode(observation)` unmodified.
9. Hot-mutable param updates take effect on the next tick; intra-tick mutation is disallowed (Scheduler enforces the between-tick boundary).
10. The STFT encoder's per-band encoder state is part of the EPM's serializable state; serialization round-trip preserves it bit-exactly.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: GNG has fewer than 2 nodes (bootstrap not done) | Publish a token with `winner_id = -1`, `tle = 0`, `is_novel = false`. No exception. |
| `params.input_topic` missing throughout run | Module logs warning every 1000 ticks, publishes the bootstrap-placeholder token. |
| Encoder dimension mismatch with `params.projection_dim` | Throw `std::invalid_argument` at construction. |
| `prediction.<modality>` arrives with wrong dim | Log warning, ignore the prediction this tick (do not subtract), continue. |
| GNG step throws (e.g. NaN propagation) | Log error, reset GNG to bootstrap state, set EPM to a one-tick "warming" mode where the published token has `tle = 0` and `is_novel = false`. Recovery is per-instance, not fatal. |
| `neuro.state` not yet published (very first tick) | Use scaling factors of 1.0 (defaults). |
| Mitosis check would split a node beyond `max_nodes` | Skip mitosis silently this tick. |

---

## Latency Budget (Pi5 Cortex-A76, full module graph)

| Modality / encoder | Tick budget | Dominated by |
|---|---|---|
| `proprio` / RBF (`proprio_state_dims = 22`, 64 centers) | ≤ 200 µs | RBF eval + GNG step |
| `video` / JL (160×120, 128-D) | ≤ 5 ms | JL projection (matrix multiply) |
| `video` / JL with `inject_centroid` | ≤ 6 ms | + center-of-mass over the resized map |
| `audio` / STFT (RTA mode, 32 bands) | ≤ 3 ms | STFT magnitude + mel |
| `audio` / STFT (FFT mode, 32 bands, 1024-sample chunk) | ≤ 12 ms | Radix-2 FFT |
| Level-1 / Identity (128-D consensus token in) | ≤ 100 µs | GNG step only |

Construction (encoder build + GNG init): ≤ 50 ms for any modality; < 10 ms for identity. Memory: dominated by GNG (~`max_nodes * projection_dim * 4 bytes` per node prototype).

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests (no Bus, no neighbours)

- **Encoder reproducibility:** identical input across two constructed EPMs with the same `master_seed` produces bit-identical `latent` vectors. (Reuse `cpp_core/tests/v3/test_encoder_jl.cpp`'s pattern.)
- **Dual TLE formula:** for synthetic (quant_error, transition_surprise) inputs, the published `tle` matches `tle_alpha * QE + tle_beta * TS` to within `1e-7` over 10⁴ samples.
- **Bootstrap behaviour:** ticks 0 and 1 publish placeholder tokens; tick 2 onward have `winner_id ≥ 0`.
- **Prediction subtraction:** with a synthetic `PredictionToken` matching the encoder output, the published `winner_id` matches the GNG winner of the zero vector. Without prediction subtraction the same input produces a different winner.
- **GNG state-roundtrip:** dump GNG to JSON, reload into a fresh EPM, single tick produces the same RealityToken as the original.
- **STFT state-roundtrip:** same test for the STFT encoder's encoder state.
- **Mitosis trigger:** under a controlled input distribution, `mitosis_count` increases when expected; no mitosis under uniform input.

### 2. Pair tests

- **`pair_neurochem_epm`** (already declared in NeurochemState.md): EPM consumes `neuro.state` (Direct), publishes TLE, NeurochemState consumes the TLE (Feedback) — both directions of the cycle exercised. Trace assertion: tick t's `neuro.state.epsilon_b_scale` reflects events from tick t-1 only, never tick t (because NeurochemState reads tick t-1 TLE).
- **`pair_epm_lateralvoter`**: one EPM's `RealityToken` arrives at LateralVoter's `reality.` subscription; LateralVoter publishes `consensus.0`. Verifies topic-name parsing (the voter must extract modality group from `reality.video.retinal`).

### 3. Latency

`tick()` p99 latency on Pi5 ≤ each modality's budget above, measured over 10⁴ runs. Failure on any modality blocks Phase 2.

### 4. Determinism

Two runs with the same `master_seed`, same observation stream, same `neuro.state` schedule, same prediction stream produce bitwise-identical `RealityToken` sequences. Cross-language parity vs. v3 Python is **not** required (numpy ↔ Eigen FP order differs); behavioural parity (matching node-count growth curves on the Phase 2 replay harness) is required.

---

## Notes on the v3 → v4 Port

- **Reuse:** `cpp_core/src/v3/encoder_jl.cpp`, `encoder_rbf.cpp`, `gng.cpp` are dropped in unchanged. The v4 `EPM` module is a thin wrapper around them that adds the Bus interface.
- **Removed:** v3's `MultiEPMAdapter` per-modality routing logic. v4 has one `EPM` instance per modality declared in the graph config; the Scheduler handles dispatch.
- **Renamed:** `EPMResult` (v3) → `RealityToken` (v4 published payload). Field-by-field compatible; the v4 token adds `producer_id` and `tick_id` from the `Message` base class.
- **New:** prediction subtraction (Feedback subscription on `prediction.<modality>`). v3 had no analog. The math is `gng_input = encode(observation) - prediction.predicted_latent`; if absent, `gng_input = encode(observation)`.
- **Cycle handling:** documented above. The Scheduler levels EPM in the same level as DescendingPredictor (both consume `neuro.state` and the previous tick's `reality.<modality>`/`prediction.<modality>` respectively); they run concurrently, not sequentially.
