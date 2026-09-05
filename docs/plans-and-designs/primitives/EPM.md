# EPM — Primitive Contract

**Phase 1 dependency position:** 2 of 9 (depends on NeurochemState).
**Header:** `cpp_core/include/ogma/modules/EPM.hpp` (Phase 1 deliverable).
**Reference impl:** `src/ami_ogma_v3/epm.py:84–312`, `cpp_core/include/v3/epm.hpp`, `cpp_core/src/v3/epm.cpp`.

---

## Purpose

An EPM (Episodic Predictive Module) is the bath's per-modality clusterer. It owns:

1. **A frozen encoder** — JL projection (visual), Hopf filterbank (cochlear), or RBF grid (proprioceptive). Stateless except for Hopf's per-band MOC EMA. Selected at construction time by `params.modality`.
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
| `reality.<group>.<modality>` (e.g. `reality.video.retinal`, `reality.audio.cochlear`, `reality.proprio.imu`, `consensus.1.<source>`) | `RealityToken` | every tick |

The exact topic name is derived from `params.modality_group` and `params.modality_name` at construction time. For Level-1 EPMs that subscribe to `consensus.0`, the output topic is `consensus.1.<id>` where `<id>` distinguishes Level-1 EPMs that subscribe to the same Level-0 consensus.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `modality_group` | string | ConstructionOnly | — | `video`/`audio`/`proprio` | Determines the topic-name prefix. Required. |
| `modality_name` | string | ConstructionOnly | — | — | Trailing component of the output topic. Required. |
| `encoder_kind` | string | ConstructionOnly | — | `jl`/`hopf`/`rbf`/`identity` | Selects the encoder backend. Required. Identity is for Level-N EPMs. |
| `input_topic` | string | ConstructionOnly | — | — | The single subscribed observation topic. Required. |
| `projection_dim` | int64 | ConstructionOnly | 128 | [16, 1024] | Encoder output dim = GNG input dim. |
| `baking_threshold` | int64 | HotMutable | 50 | [10, 500] | Visit count required to bake a node. **⚠ The 50 is the schema's advertised default only.** The runtime hands a config's params to `on_setup` verbatim (`OgmaInstance.cpp:45`) and never merges schema defaults, so an EPM whose config *omits* this key runs `GNG::Config`'s own default of **100**. Measured 2026-09-05 (Kalman-lessons Stage 0): 105 EPM instances across 40 configs omit it. State it explicitly in every config; the picrawler stack does. |
| `min_insertion_error` | double | HotMutable | 0.02 | (0, 1] | The insertion/consistency gate. **Only a floor when `insertion_autotune` is on** — see the correction below. |
| `insertion_autotune` | bool | ConstructionOnly | false (off) | — | Set the gate from the GNG's own recent squared-TLE distribution; the value above becomes the floor. See "Insertion-gate self-tuning". |
| `insertion_autotune_quantile` | double | ConstructionOnly | 0.30 | (0, 1) | Percentile used. A *rank*, not a scale — dimensionless, hence adaptive rather than tuned. |
| `gain_kind` | string | ConstructionOnly | `linear` | `linear`/`kalman` | **Per-node Kalman gain** ([charter](../epm_kalman_lessons_plan.md), Stage 1). `linear` = the legacy anneal `ε_b(1 − 0.9·visits/N)`, byte-identical. `kalman` = each node runs its own scalar Kalman filter: `p += q; K = min(cap, p/(p+1)); w += K(x−w); p *= 1−K`. With `kalman_p0 = 1`, `kalman_q = 0` that is exactly the filter for a constant, gain `1/(n+1)`, and baked nodes stay frozen. Measured 2026-09-05: the legacy anneal leaves 24 % of a baked prototype on its birth point and 2× the MSE of the same samples' mean. `ε_b` and its neuro scaling are unused in this mode; `ε_n` unchanged. |
| `kalman_p0` | double | ConstructionOnly | 1.0 | (0, ∞) | Initial `p` of every node born; 1 = the seed counts as one sample. |
| `kalman_q` | double | HotMutable | 0.0 | [0, ∞) | Process-noise ratio added to `p` per win. 0 = baked frozen; > 0 = every node, baked included, settles at the random-walk steady-state gain `(q + √(q²+4q))/2` and tracks slow drift instead of waiting for mitosis. A separate lever from `gain_kind`. |
| `kalman_gain_cap` | double | HotMutable | 1.0 | (0, 1] | Upper bound on `K`. Lower it if the high early gain drags a young node across a cluster boundary (the risk the bench's S1m scenario watches). |
| `lambda_new` | int64 | HotMutable | 25 | [5, 200] | Steps between insertion-error reviews. |
| `max_age` | int64 | HotMutable | 88 | [10, 500] | Edge max age before pruning. |
| `epsilon_b` | double | HotMutable | 0.05 | (0, 1] | Winner learning rate (modulated by `neuro.state.epsilon_b_scale`). |
| `epsilon_n` | double | HotMutable | 0.003 | (0, 1] | Neighbour learning rate. |
| `alpha` | double | HotMutable | 0.5 | (0, 1] | Error halving on insertion. |
| `beta` | double | HotMutable | 0.0005 | (0, 1] | Global error decay per step. |
| `max_nodes` | int64 | HotMutable | 2000 | [50, 50000] | GNG capacity. |
| `tle_alpha` | double | HotMutable | 0.7 | [0, 1] | Weight of `quant_error` in dual TLE. |
| `transition_surprise_kind` | string | HotMutable | `displacement` | `displacement`/`logprob` | **Stage 3 restoration** ([charter](../epm_kalman_lessons_plan.md)). `displacement` = `‖proto_t − proto_{t−1}‖`, the C++ port's stand-in, byte-identical. `logprob` = the Python reference's surprise: `−log P(cur|prev)` from the EPM's own transition counts as they stood before the step, Laplace-smoothed, normalised by `log N` to [0, 1], conditioned on a move (a stay scores 0, a first arrival 1). The bench's S4 showed the displacement cannot separate an expected transition from a teleport (ratio 1.03). |
| `tle_beta` | double | HotMutable | 0.3 | [0, 1] | Weight of `transition_surprise`. (`tle_alpha + tle_beta` need not sum to 1 — they are independent gains.) |
| `mitosis_enabled` | bool | HotMutable | true | — | — |
| `mitosis_error_threshold` | double | HotMutable | 0.30 | (0, 1] | Post-bake mean error to trigger split (modulated by `neuro.state.mitosis_threshold_scale`). |
| `mitosis_check_interval` | int64 | HotMutable | 50 | [5, 1000] | Post-bake visits between checks. |
| `health_base_decay` | double | HotMutable | 0.997 | [0.9, 1.0) | Per-tick health decay multiplicand. |
| `stale_prune_enabled` | bool | HotMutable | true | — | — |
| `stale_window_factor` | double | HotMutable | 12000.0 | [100, 1e6] | Steps before a non-revisited node is prune-eligible. |
| `subtract_descending_prediction` | bool | HotMutable | true | — | If true and `prediction.<modality>` is present, subtract before GNG. |
| `master_seed` | int64 | ConstructionOnly | 0 | — | Seeds the JL random matrix and any GNG stochastic operations. Forwarded via `_rng.derive_rng(seed, "epm.<id>")`. |
| `dim_autocal_ticks` | int64 | ConstructionOnly | 0 (off) | [0, ∞) | **Commissioning window**, in input frames. Measure the per-dim input ranges instead of being told them; see "Commissioning" below. RBF only (throws otherwise). Mutually exclusive with `dim_min`/`dim_max` (throws). |
| `dim_autocal_k` | double | ConstructionOnly | 4.0 | (0, ∞) | σ-multiplier for the commissioned range: `intersect(μ ± k·σ, [min_obs, max_obs])`. |
| Encoder-specific params (cochlear `f_min`/`f_max`/`sample_rate`/`cochlear_rta`; visual `encoder_res`/`inject_centroid`/`centroid_gain`; proprio `proprio_state_dims`/`proprio_dim_ranges`) | various | ConstructionOnly | — | — | Forwarded to the encoder constructor. See `cpp_core/include/v3/types.hpp:EPMConfig`. |

---

## Commissioning (`dim_autocal_ticks`)

§0 rule 2 requires an EPM's input be **conditioned** before the GNG discretises
it: a channel whose scale is small relative to its siblings is collapsed by the
insertion gate *while the encoder still shows the structure*. Doing that by hand
means measuring each sensor and writing `dim_min`/`dim_max` into config — a
constant tuned to a signal's scale, which is the smell that names a missing
adaptive mechanism. Commissioning **is** that mechanism.

**Why this does not violate "frozen encoder."** The RBF encoder's centers are
laid out in normalised `[0,1]^d` and its auto-`sigma` is derived from
inter-center distances *in that same space* (`encoder_rbf.cpp`). Neither reads
`dim_ranges`; the ranges are consumed in exactly one place, `normalise_state()`,
as a per-dim affine map plus a clamp. `dim_ranges` is therefore **sensor
conditioning upstream of the frozen encoder**, not part of it. Recalibrating it
leaves centers, sigma and the projection untouched.

**Sequence.** For the first N input frames the EPM runs *normally* (warm start)
while accumulating per-dim statistics. On frame N it derives ranges, installs
them, and **resets the GNG topology**, so the vocabulary is re-earned in the
calibrated space. Frame N+1 is the first encoded in the new space.

- **The reset is mandatory, not a tuning choice.** Every prototype learned during
  the window is expressed in provisional units. Keeping them would leave the map
  moving under a topology that *bakes* — the one thing baking assumes cannot
  happen.
- **Node IDs survive the reset** (`GNG::reset_topology` deliberately does not
  reset `next_id_`), so Invariant 4 holds across the boundary: a consumer holding
  a pre-reset `winner_id` sees an ID that no longer resolves, rather than one
  silently rebound to an unrelated region of a different space.
- **Range rule:** `intersect(μ ± k·σ, [min_obs, max_obs])`. Pure min/max is
  outlier-driven — a single first-tick transient sets a range an order of
  magnitude too wide; pure `μ ± k·σ` can invent range a bounded channel never
  occupies. Degenerate dims (`hi − lo` below a floor) fall back to the default
  and are reported in the `EPM_AUTOCAL` line.
- **Window length is legitimately application-set.** It must cover the body's
  characteristic motion — several stride cycles for a gait, a full sweep for a
  scanning sensor. A range set by a startup transient is worse than the default.

**Scope.** RBF only. JL and STFT **throw**: their dims are homogeneous
pixels/samples and a per-dim rescale would destroy the structure the frozen
encoder exists to preserve. Identity is *deferred, not refused* — stacked
heterogeneous latents plausibly want it, but nothing has measured a need.

**⚠ WHEN NOT TO COMMISSION — measured the hard way, 2026-08-06.** Commissioning
normalises a channel by its **observed range**, and a range says *nothing* about
whether what fills it is signal or noise. Enabled on the picrawler motor layer it
rescaled the three velocity (`delta`) channels — per-tick joint differences at
60 Hz, small **and** noise-dominated — from ~5–12 % of the default span to 100 %,
putting the noise on equal footing with position. Quantisation error rose
everywhere, so nothing reached consistency and everything looked novel: **baked
fraction 43 % → 5 %**, node count 58 → 154 and climbing. It is a `REGRESSION`
there, recorded in the lever ledger.

The unit test is not contradicted by this — it shows commissioning recovers a
small signal that is **real** (winner purity 0.698 → 1.000). What it cannot tell
you is whether *your* small channel is real. **So: commission a channel when you
have reason to believe it carries information the default range is hiding — the
smart-sensor case, where a sensor's operating range genuinely must be discovered.
Do not commission a channel merely because it is small.** The cheap check is the
one that caught it here: run with commissioning off and compare **baked fraction**,
never node count.

**Not in scope, deliberately:** (1) *re-calibration mid-life* — same
moving-map-under-a-baked-vocabulary objection; if a body's dynamics change
permanently the honest answers are mitosis or a new EPM. (2) *common-mode
removal* — §0 rule 2 names both, but common-mode is cross-channel, and the EPM
already has the principled version: descending-prediction subtraction. This is
only the scale half.

---

## Insertion-gate self-tuning (`insertion_autotune`)

⚠ **This schema row was wrong for the entire life of the C++ port**, and the error
is instructive. It read *"Auto-tuned each `lambda_new` ticks; this is the floor"* —
describing the **v3 Python** behaviour (`python/xaq/xaq/gng.py:105-114, 676-685`),
where the GNG picked its own insertion floor from the 30th percentile of its recent
squared-TLE distribution, with `freeze_min_insertion_error=True` documented as the
*"old fixed-threshold behaviour for debugging / regression runs."* **Only the frozen
debug branch survived the port to C++.** So every EPM in this repo has run the
debug path while the contract advertised the adaptive one — a doc that describes a
mechanism the code does not have is worse than no doc, because it stops anyone
looking.

`insertion_autotune` restores it, opt-in and default-off.

**Why the gate matters more than it looks.** `min_insertion_error` gates **two**
things, here and in the reference:

| | condition | effect |
|---|---|---|
| insertion | `ema_error < gate` | converged here — do not insert |
| baking | `ema_error >= gate` | inconsistent — demote instead of bake |

A gate **below the signal's own error floor** therefore means nodes always look
surprising enough to insert *and* never look consistent enough to bake. **Unbounded
growth and near-zero baking are one cause, not two symptoms** — which is why the
honest instrument for this is the **baked fraction**, never the node count. (Node
count moves the *wrong way* under the fix: baked nodes are frozen and immune to
pruning, so earning a vocabulary makes the population bigger.)

- **Effective gate = `max(configured, quantile) * neuro_scale`.** The configured
  value is a **floor** — a deliberate divergence from the reference, which replaced
  the threshold outright and so lets the gate collapse toward zero in a low-error
  regime, where growth runs away again.
- **The neurochemical scale multiplies the auto-tuned value**, so dopamine widens or
  narrows growth relative to the body's *current* typical surprise rather than
  relative to a fixed constant. With autotune off the scale is folded in exactly as
  before, bit-identically.
- **The history round-trips** in `GNG::to_json`. A restored GNG that re-warms from an
  empty history has no gate at all for its warmup window — same class of bug as
  Invariant 11.

**Measured, and it is not a universal win.** On a stream whose typical error exceeds
the configured gate it is decisive: baked **0 % → 96 %**. On the picrawler motor
signal it is a **`NULL`** — the gate lifted (0.0234 vs a 0.020 floor) but a ~1.4x lift
was not enough, and baked fraction moved 40 % → 43 %, i.e. not at all. See the lever
ledger for both verdicts and their re-use contexts.

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
10. The Hopf encoder's per-band MOC EMA state is part of the EPM's serializable state; serialization round-trip preserves it bit-exactly.
11. When `dim_autocal_ticks > 0`, both the accumulator **and** the installed ranges are part of the serializable state — same requirement, and the same reason, as Invariant 10. `on_setup` rebuilds the encoder from params, so an EPM restored after commissioning that did not re-install its ranges would run its baked vocabulary against *default* conditioning: a silent space swap. When `dim_autocal_ticks == 0` no commissioning fields are emitted at all, so the serialized form is byte-identical to a pre-feature snapshot.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: GNG has fewer than 2 nodes (bootstrap not done) | Publish a token with `winner_id = -1`, `tle = 0`, `is_novel = false`. No exception. |
| **Arriving payload length ≠ `proprio_state_dims`** (RBF) | ⚠ **CURRENTLY SILENT — see "The zero-encode trap" below.** `FrozenRBFEncoder::encode` returns `Zero(projection_dim)` on dim mismatch; the EPM's only check is `out.size() == projection_dim`, which a zero vector passes. The GNG then steps on a constant zero vector every tick, forever. The contract row below says this **should** throw; it does not yet. |
| `params.input_topic` missing throughout run | Module logs warning every 1000 ticks, publishes the bootstrap-placeholder token. |
| Encoder dimension mismatch with `params.projection_dim` | Throw `std::invalid_argument` at construction. |
| `prediction.<modality>` arrives with wrong dim | Log warning, ignore the prediction this tick (do not subtract), continue. |
| GNG step throws (e.g. NaN propagation) | Log error, reset GNG to bootstrap state, set EPM to a one-tick "warming" mode where the published token has `tle = 0` and `is_novel = false`. Recovery is per-instance, not fatal. |
| `neuro.state` not yet published (very first tick) | Use scaling factors of 1.0 (defaults). |
| Mitosis check would split a node beyond `max_nodes` | Skip mitosis silently this tick. |

### The zero-encode trap — read this before diagnosing a dead EPM

**Cost: three sessions, 2026-08-06.** Four motor-layer EPMs were configured
without `proprio_state_dims`, so it defaulted to 22 while the bridge published
9-value tokens. `FrozenRBFEncoder::encode` returns a **zero vector of length
`projection_dim`** on dim mismatch instead of throwing, and `encode_pending_input`
validates only the *output* length — which the zero vector satisfies. Result: the
EPM reported `have_input = true` and stepped its GNG on a constant zero vector for
the entire run.

**The diagnostic signature, and the one field that disambiguates it:**

| Symptom | Fed nothing | **Fed zeros (this trap)** |
|---|---|---|
| `gng.last_x` | `null` — `step()` never ran | **all-zeros array** — `step()` ran |
| `history_trace` | empty | one winner, forever |
| `nodes` / `ema_tle` | 2 / exactly 0 | 2 / a tiny decaying value (~4e-06) |

`last_x` is emitted as JSON `null` until the first `step()` (`gng.cpp:567-571`).
**`null` means the input never arrived; all-zeros means it arrived and was
silently zeroed.** These need completely different fixes, and reading
"`|last_x|` = 0" as "nothing arrived" sent a session chasing the subscription
path when the subscription was fine. Check `history_trace` as the cross-check: it
is only appended on the real path, never in `publish_bootstrap_token`.

Two further traps in the same family, both of which cost a session each:

- **An EPM snapshot has no `"module"` wrapper** — its diag fields sit at top
  level, unlike MotorEPM's. A probe looking for one finds nothing and prints
  nothing, making a *dead* EPM indistinguishable from an *unread* one.
- **`modality_group` + `modality_name` ARE the output address**
  (`output_topic = "reality." + group + "." + name`). Naming them after the input
  source makes the EPM publish onto its own input topic. Nothing errors; the only
  symptom is that nothing consumes anything.

**The general rule this class teaches:** a config-shape error must fail loudly at
construction, because every one of these is invisible at runtime and presents as
"the module is running fine and doing nothing."

---

## Latency Budget (Pi5 Cortex-A76, full module graph)

| Modality / encoder | Tick budget | Dominated by |
|---|---|---|
| `proprio` / RBF (`proprio_state_dims = 22`, 64 centers) | ≤ 200 µs | RBF eval + GNG step |
| `video` / JL (160×120, 128-D) | ≤ 5 ms | JL projection (matrix multiply) |
| `video` / JL with `inject_centroid` | ≤ 6 ms | + center-of-mass over the resized map |
| `audio` / Hopf (RTA mode, 32 bands) | ≤ 3 ms | Filterbank IIR + MOC EMA |
| `audio` / Hopf (FFT mode, 32 bands, 1024-sample chunk) | ≤ 12 ms | Radix-2 FFT |
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
- **Hopf state-roundtrip:** same test for the Hopf encoder's MOC EMA state.
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

- **Reuse:** `cpp_core/src/v3/encoder_jl.cpp`, `encoder_hopf.cpp`, `encoder_rbf.cpp`, `gng.cpp` are dropped in unchanged. The v4 `EPM` module is a thin wrapper around them that adds the Bus interface.
- **Removed:** v3's `MultiEPMAdapter` per-modality routing logic. v4 has one `EPM` instance per modality declared in the graph config; the Scheduler handles dispatch.
- **Renamed:** `EPMResult` (v3) → `RealityToken` (v4 published payload). Field-by-field compatible; the v4 token adds `producer_id` and `tick_id` from the `Message` base class.
- **New:** prediction subtraction (Feedback subscription on `prediction.<modality>`). v3 had no analog. The math is `gng_input = encode(observation) - prediction.predicted_latent`; if absent, `gng_input = encode(observation)`.
- **Cycle handling:** documented above. The Scheduler levels EPM in the same level as DescendingPredictor (both consume `neuro.state` and the previous tick's `reality.<modality>`/`prediction.<modality>` respectively); they run concurrently, not sequentially.
