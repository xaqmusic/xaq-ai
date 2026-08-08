# ActionDecoder — Primitive Contract

**Phase 1 dependency position:** 5 of 9 (depends on HomeostaticDrive — drive-grounded form is the only form built).
**Header:** `cpp_core/include/ogma/modules/ActionDecoder.hpp` (Phase 1 deliverable).
**Reference impl:** `src/ami_ogma_v3/action_decoder.py:56+` plus `src/ami_ogma_v3/active_inference.py:EFEPolicy` and `ActionTransitionModel`.

---

## Purpose

The ActionDecoder is the bath's policy: given current state, produce one motor command per tick. v4's decoder is **drive-grounded** by design — its EFE policy reads `drive.errors` from HomeostaticDrive and treats the published `urgency` as the explicit exploit-vs-explore knob. There is no v3-port stage; the v3 valence-map-as-floating-reward is reinterpreted here as a learned *drive-error-reduction predictor* keyed on (state node, proprio node).

Concretely:

1. **Hebbian motor cache** — fast-path `(modality, prev_node, cur_node, proprio_node) → velocity_bias` table, ported from v3. Each tick's emitted action lands in the cache via TD credit assignment against `neuro.state.reward_signal`. This handles "I've been here before, that worked" with no inference.
2. **Drive-grounded EFE policy** — when the cache is uncertain (no high-confidence entry for the current state) or `drive.errors.urgency` is high, the decoder falls through to an EFE rollout over candidate actions. Pragmatic value comes from the drive-predictive valence model (`V(state, proprio) ≈ E[drive_reduction | state, proprio]`); epistemic value comes from `GNGRollout.entropy` (Phase 1 onwards) or a temporary count-table proxy until GNGRollout lands.
3. **Probe machine** — extinction-learning state ported from v3 (`_probe_phase`, `_probe_countdown`, `_aversion_ticks`, `_serotonin_gate`). When serotonin saturates and dopamine is low, the decoder injects a stochastic probe action via `_rng.derive_rng(seed, "decoder.probe")`.
4. **Eligibility trace** — bounded deque of recent (state, proprio, action, reward) tuples for n-step TD credit (default depth 12, matching v3).

Reflex-influence and prediction-confidence gates from v3's GlobalWorkspaceHarness are deliberately not preserved (resolved decision, see `docs/v4_refactor.md` Pitfall #3 / plan resolution table).

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `consensus.<level>` | Direct | `ConsensusToken` | LateralVoter | yes | Whichever level has the lowest `fused_tle` drives behaviour; in Phase 1 the decoder subscribes to `consensus.0` only. Level-selection becomes hot-patchable in Phase 4 (fractal level-N). |
| `reality.proprio.` | Direct | `ProprioToken` | host | yes | Specifically the body-state EPM's winner determines `proprio_node` for the Hebbian key. The decoder pulls the active proprio modality's `RealityToken.winner_id` from `consensus.0.trust_weights`. |
| `neuro.state` | Direct | `NeuroState` | NeurochemState | yes | `reward_signal` for credit assignment; `dopamine`/`serotonin` gate the probe machine. |
| `drive.errors` | Direct | `DriveErrors` | HomeostaticDrive | yes | Pragmatic value source for the EFE policy. `urgency` modulates exploration. |
| `rollout.result` | Direct (REP correlation) | `RolloutResult` | GNGRollout (Phase 1+) | no | When present, replaces the count-table proxy for epistemic value. Off in Phase 2 thin-slice. |
| `motor.play.stream` | Direct (REP correlation) | `MotorPlayStream` | MotorRepertoire (Phase 1+) | no | When the decoder has dispatched a chunk, this is the stream of actions it emits. Off in Phase 2 thin-slice. |

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `action.out` | `ActionOut` (singleton) | every tick |
| `rollout.query` | `RolloutQuery` | when EFE policy needs a rollout (Phase 1+) |
| `motor.play.cmd` | `MotorPlayCmd` | when starting a chunk (Phase 1+) |

`ActionOut` carries either `accel` (scalar, default in Phase 1) or `chunk_id` (when MotorRepertoire is wired in). `probe = true` whenever the action came from the probe RNG.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `consensus_level` | int64 | HotMutable | 0 | [0, 16] | Which `consensus.<n>` topic to consume. |
| `efe_temperature` | double | HotMutable | 1.0 | (0, 10] | Softmax temperature over candidate actions. |
| `pragmatic_gain` | double | HotMutable | 10.0 | [0, 100] | Weight of pragmatic value in EFE = pragmatic_gain * V + epistemic_gain * H. |
| `epistemic_gain` | double | HotMutable | 1.0 | [0, 100] | Weight of epistemic value (entropy term). |
| `urgency_exploit_bias` | double | HotMutable | 1.5 | [0, 5] | Multiplies pragmatic_gain when `drive.errors.urgency > urgency_exploit_threshold`. |
| `urgency_exploit_threshold` | double | HotMutable | 0.6 | [0, 1] | Above this, exploit; below, explore. |
| `probe_serotonin_threshold` | double | HotMutable | 0.85 | [0, 1] | Probe machine arms when serotonin > this and dopamine < `probe_dopamine_threshold`. |
| `probe_dopamine_threshold` | double | HotMutable | 0.20 | [0, 1] | — |
| `probe_duration_ticks` | int64 | HotMutable | 8 | [1, 200] | How long a probe action holds before reverting. |
| `eligibility_depth` | int64 | HotMutable | 12 | [1, 200] | Max length of the eligibility trace deque. |
| `td_lambda` | double | HotMutable | 0.7 | [0, 1] | Eligibility trace decay; matches v3. |
| `valence_decay_pos` | double | HotMutable | 0.99999 | [0.99, 1.0) | Decay on positive valence entries (drive reduction observed). |
| `valence_decay_neg` | double | HotMutable | 0.9999 | [0.99, 1.0) | Decay on negative entries — slower forgetting of "this hurt." |
| `valence_max_size` | int64 | HotMutable | 2000 | [100, 1e6] | LRU cap on the (state_node, proprio_node) → drive-reduction map. |
| `accel_min` | double | HotMutable | -4.0 | — | Output clamp. |
| `accel_max` | double | HotMutable | 4.0 | — | Output clamp. |
| `action_bins` | int64 | ConstructionOnly | 3 | [2, 21] | Discretization of the action axis for EFE rollout (-1, 0, +1 in v3 default). |
| `use_chunks` | bool | HotMutable | false | — | If true and MotorRepertoire is in the graph, emit `chunk_id` selections instead of `accel`. |
| `master_seed` | int64 | ConstructionOnly | 0 | — | RNG namespace `decoder.<id>` for probe + EFE softmax sampling. |

---

## Invariants (per tick)

1. ActionDecoder publishes exactly one `ActionOut` per tick.
2. `accel ∈ [accel_min, accel_max]` always.
3. Hebbian table updates use the **previous** tick's (state, proprio, action) — credit cannot be assigned to the action emitted in the same tick as the consensus that produced it; the reward signal arrives in tick t+1 at earliest. The eligibility trace handles n-step credit.
4. `probe == true` if and only if the action was sampled from the probe RNG; deterministic given `master_seed` and the tick stream.
5. When `use_chunks == false` (Phase 2 thin-slice), `chunk_id == -1` always.
6. The valence map's entries decay every tick by `valence_decay_pos` (positive entries) or `valence_decay_neg` (negative entries) — never both, never neither.
7. EFE policy ties are broken by deterministic seed-based sampling; identical seeds + identical inputs produce identical actions.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: no `consensus.0` yet | Emit `accel = 0`, `probe = false`. |
| `drive.errors` missing throughout the run | Treat `urgency = 0`; pragmatic term uses pure valence map without urgency boost. |
| `neuro.state` missing | Treat `reward_signal = 0`; no Hebbian/valence updates this tick. |
| GNGRollout query times out (Phase 1+) | Fall back to count-table epistemic proxy; log warning. |
| MotorRepertoire chunk dispatch fails | Revert to scalar `accel` for the next tick; `use_chunks` stays true. |
| Action selection produces NaN | Emit `accel = 0`, log error, do not propagate NaN. |

---

## Latency Budget (Pi5 Cortex-A76)

- **Hot path (cache hit):** ≤ 50 µs. Dominated by Hebbian lookup + clamp.
- **Hot path (EFE fallback, no GNGRollout):** ≤ 300 µs. Iterates `action_bins` candidates, evaluates pragmatic + epistemic per candidate.
- **Hot path (EFE with GNGRollout query):** ≤ 1.5 ms total (most of this in GNGRollout itself; ActionDecoder's portion is + ~100 µs for the query + result correlation).
- **Construction:** ≤ 200 µs.
- **Memory:** dominated by Hebbian table (`hebbian_max_size * 16 bytes`) + valence map (`valence_max_size * 12 bytes`). Default ~50 KB total.

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Hebbian cache hit:** with a pre-populated table entry for `(modality, prev=A, cur=B, proprio=P)`, the decoder emits the cached `velocity_bias` exactly (within `1e-7`).
- **EFE selection:** with synthetic valence and entropy values for each action bin, the decoder selects the argmax of `pragmatic_gain * V + epistemic_gain * H` (within softmax sampling determinism).
- **Probe arming:** stepping with serotonin = 0.9, dopamine = 0.1 for 100 ticks engages the probe machine; `probe == true` in those ticks. Reverting serotonin to 0.5 disengages.
- **Eligibility trace decay:** a synthetic 12-step trajectory with terminal reward = 1.0 produces TD updates whose magnitudes follow `λ^k` for k ticks back.
- **Determinism:** identical `master_seed` + input streams → identical action sequences, including probe ticks.
- **Valence pruning:** when GNG nodes reported in `RealityToken.pruned_ids` are removed, all valence entries keyed on those node IDs are evicted (no zombie keys).
- **Drive-grounded selection:** with `urgency = 0.0` vs `urgency = 0.9` and identical state, the actions chosen differ in expected ways (urgency boosts the highest-V action's softmax probability).

### 2. Pair tests

- **`pair_lateralvoter_actiondecoder`** (declared in LateralVoter.md): synthetic `consensus.0` → action emission.
- **`pair_homeostatic_actiondecoder`** (declared in HomeostaticDrive.md): drive errors gate the EFE policy.
- **`pair_actiondecoder_motorrepertoire`** (Phase 1 stretch when MotorRepertoire lands): chunk dispatch round-trip.

### 3. Latency

`tick()` p99 ≤ 300 µs (cache miss + EFE fallback) on Pi5 over 10⁴ runs with `action_bins = 3`.

### 4. Determinism

Identical `master_seed` and identical Bus deliveries produce bitwise-identical `ActionOut` sequences across two independent runs.

### 5. Behavioural sanity

In a synthetic environment where one specific action reduces drive error and others don't, the decoder converges to choosing the correct action ≥ 80% of the time within 2000 ticks, with `urgency_exploit_threshold = 0.6`.

---

## Notes on the v3 → v4 Port

- **Drive-grounded EFE replaces v3 valence-floating EFE.** v3's `EFEPolicy` reads `valence_map[(visual_node, proprio_node)] → valence`. v4 reinterprets the same map as `V(state, proprio) ≈ E[drive_reduction | state, proprio]` — the storage shape is identical, the *semantics* and the *update rule* change. v4 updates V via TD against `drive_reduction = -Δ|drive_errors|` rather than against a free-floating dopamine signal. Detail in implementation: when `events.hit` arrives, both `neuro.state.reward_signal` and `drive.errors.energy` change; the valence update uses the *drive delta* directly so the policy is grounded in the body's actual state, not the chemistry's interpretation of it.
- **No reflex_influence gate.** v3's GlobalWorkspaceHarness multiplied the action by `reflex_influence ∈ [0, 1]` based on a learned prediction-accuracy estimator. Removed; the drive-grounded EFE replaces this purpose (high urgency → exploit known-good actions; low urgency → explore).
- **Probe machine ported faithfully.** Same state machine, same default thresholds, same RNG namespace. Bit-exact reproducibility against the v3 `_rng.derive_rng(seed, "decoder.probe")` path.
- **Action discretization.** v3's action axis has 3 bins (-1, 0, +1) in `action_bins`. v4 keeps this default; richer discretization is a Phase 3 experiment.
- **Hebbian table evicts via GNG pruning.** When `RealityToken.pruned_ids` reports a node death, the decoder's Hebbian + valence entries keyed on that ID must be evicted. v3 did this via `purge_nodes(set)`; v4 makes it explicit in the contract.
