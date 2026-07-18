# DescendingPredictor — Primitive Contract

**Phase 1 dependency position:** 6 of 9 (depends on EPM + LateralVoter + ActionDecoder; lands after Phase 2 thin-slice).
**Header:** `cpp_core/include/ogma/modules/DescendingPredictor.hpp` (Phase 1 deliverable).
**Reference impl:** none — new primitive (`docs/v4_algorithmic_gaps.md` Primitive 1).

---

## Purpose

The DescendingPredictor closes the top-down loop. v3 has none; every signal flows up. v4 adds one DescendingPredictor per voter (in the MVP — Open Question #1 in the gap doc allows scaling later) that maintains a parametric predictor of each downstream EPM's expected next-tick latent. The EPM subtracts the prediction before its GNG sees the input, so the GNG topologizes **surprise** rather than raw observation.

MVP form is an **AR(1) linear predictor per subscriber** (Open Question #1 resolved): one `target_dim × source_dim` matrix `W_target` plus an offset `b_target` per downstream EPM. Forward pass: `predicted_latent = W_target · consensus_embedding + b_target`. Loss: squared error against the target EPM's next-tick `RealityToken.latent`. Update: recursive least squares (RLS) or simple gradient descent at construction-configurable learning rate; no autograd, no PyTorch. The choice between RLS and SGD is a Phase 1 implementation detail; both meet the contract.

The recurrent-predictor variant called out in `v4_algorithmic_gaps.md` Open Question #1 is a future extension; this contract specifies AR(1) only.

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `consensus.<level>` | Direct | `ConsensusToken` | LateralVoter | yes | The "deep cortical layer" source for descending prediction. |
| `reality.<group>.<modality>` (per declared target) | Feedback | `RealityToken` | target EPM | yes | Each declared target's actual latent from t-1, used as the supervisory signal for the predictor's update step. |

`params.targets` declares which EPM topic names the predictor outputs predictions for; one Feedback subscription per target.

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `prediction.<modality>` (per target) | `PredictionToken` | every tick, per target |

`PredictionToken.target_modality` mirrors the target's topic suffix (e.g. `prediction.video.retinal` carries `target_modality = "video.retinal"`).

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `consensus_topic` | string | ConstructionOnly | `consensus.0` | — | Source topic. |
| `targets` | vector<string> | ConstructionOnly | `[]` | — | List of EPM topic names this predictor projects to. Required: ≥1. |
| `update_method` | string | ConstructionOnly | `rls` | `rls` / `sgd` | Online learning algorithm. |
| `learning_rate` | double | HotMutable | 0.01 | (0, 1] | SGD step size; ignored under RLS. |
| `rls_forget` | double | HotMutable | 0.99 | [0.9, 1.0) | Forgetting factor for RLS. |
| `init_noise_scale` | double | ConstructionOnly | 0.01 | [0, 1] | Std-dev of zero-mean Gaussian initialization for `W_target`. |
| `freeze_after_ticks` | int64 | HotMutable | 0 | [0, 1e7] | If > 0, stop online updates after this many ticks. 0 = never freeze. |
| `confidence_window` | int64 | HotMutable | 100 | [10, 10000] | Rolling window for self-rated confidence (used in `PredictionToken.confidence`). |
| `master_seed` | int64 | ConstructionOnly | 0 | — | RNG namespace `predictor.<id>` for matrix initialization. |

---

## Invariants (per tick)

1. The predictor publishes one `PredictionToken` per declared target every tick.
2. `PredictionToken.predicted_latent.dim == target_EPM.projection_dim`.
3. The supervisory update for tick t uses `RealityToken_target(t-1)` (Feedback) and `ConsensusToken(t-1)` (held in a one-tick buffer because the consumer of the prediction — the target EPM — needs the prediction in the *current* tick before it has produced its `RealityToken(t)`). The predictor's tick(t):
   1. reads `consensus.<level>(t)` (Direct) for forward pass,
   2. reads `reality.<modality>(t-1)` (Feedback) for supervisory update vs. its previous prediction,
   3. publishes `prediction.<modality>(t)` for the target EPM to consume (Feedback) at tick t+1.
4. `confidence ∈ [0, 1]`, computed as `1 - mean(|prediction_error| / target_norm)` over the last `confidence_window` ticks. NaN-safe; never published as NaN.
5. When `freeze_after_ticks > 0` and reached, `W_target` and `b_target` are no longer updated; predictions continue to be published.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: no `consensus.<level>` yet | Publish a zero-vector `predicted_latent` per target; `confidence = 0`. |
| Target EPM dim changes via hot-patch | Predictor must reinitialize the corresponding `W_target` to match. If reinitialization is forbidden by the patch transaction, the patch is rejected. |
| Numerical instability (NaN in `W_target`) | Reset `W_target` to zero matrix; log error; continue with zero predictions until the next confidence_window completes. |

---

## Latency Budget (Pi5 Cortex-A76)

- **Forward pass per target (128-D source × 128-D target):** ~30 µs (matrix-vector multiply).
- **RLS update per target:** ~500 µs (rank-1 update of P matrix, dim²).
- **SGD update per target:** ~50 µs.
- Total `tick()` for 5 targets under SGD: ≤ 500 µs. Under RLS: ≤ 3 ms.

Memory: `targets * (target_dim * source_dim * 4 bytes)` for the weights; under RLS, additionally `targets * source_dim² * 4 bytes` for the inverse-covariance matrices.

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Forward determinism:** identical `consensus_embedding` produces identical `predicted_latent` after construction (no online updates).
- **Linear regression convergence:** with synthetic data `target = A · consensus + ε`, after 5000 ticks the predictor's `W_target` is within `0.02` (Frobenius norm) of `A` for SGD, `0.005` for RLS.
- **Subtraction integration:** with the predictor's output piped to a stub EPM, the GNG receives `encode(input) - prediction.predicted_latent`.
- **Confidence tracking:** after 1000 ticks of perfect prediction, `confidence > 0.95`. After 1000 ticks of orthogonal noise targets, `confidence < 0.20`.
- **Freeze:** after `freeze_after_ticks`, weights stop changing. Verified by snapshotting `W_target` at the freeze boundary and 100 ticks later.

### 2. Pair tests

- **`pair_descendingpred_epm`**: predictor publishes `prediction.video.retinal`; EPM consumes it (Feedback). Verified by trace assertion that EPM's GNG input equals encoder output minus the previous-tick prediction.

### 3. Latency

`tick()` p99 ≤ 500 µs (SGD, 5 targets) or ≤ 3 ms (RLS, 5 targets) on Pi5.

### 4. Determinism

Identical `master_seed` + input streams produce bitwise-identical predictions across runs.

### 5. Phase 3 behavioural target

When the DescendingPredictor is wired into a Phase-3 integration of the maze benchmark, the EPM's mean `quant_error` over the second half of an episode drops by ≥ 15% vs. the same run with `subtract_descending_prediction = false`. (This is the contract's claim that descending prediction actually denoises upstream streams; if it doesn't, the primitive is wrong.)

---

## Notes

- **Per-voter, not global.** Each LateralVoter that has a paired DescendingPredictor reads from its own `consensus.<level>` and projects to its own targets. There is no cross-level prediction sharing in Phase 1.
- **Recurrent variant deferred.** A small per-voter recurrent predictor (GRU-shaped, but inside the voter, not a global workspace) is the documented next step in `v4_algorithmic_gaps.md`. Phase 1 ships AR(1); promotion happens via a contract amendment when the AR(1) form is shown to have a meaningful capacity gap on a real benchmark.
