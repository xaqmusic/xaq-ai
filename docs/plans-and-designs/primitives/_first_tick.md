# Cross-Cutting: First-Tick Semantics

**Applies to:** every Phase 1+ Ogma Core module.

---

## The Problem

The Scheduler runs every module exactly once per global tick. At tick 0, no module has yet published anything. A consumer that calls `Bus::last_value(topic)` on a topic whose producer hasn't run yet (or that hasn't been declared in this graph at all) gets `nullptr`. The contract for what the consumer does with that nullptr is what this doc nails down — the alternative is silent NaN propagation or first-tick exceptions, both unacceptable.

Two separate cases:

1. **Producer exists in the graph but hasn't run yet.** The consumer's level executes before the producer's level on the FIRST tick. From tick 1 onwards, this only matters for **Feedback** subscriptions (which always read the previous tick's value).
2. **Producer is absent from the graph.** A module declared its subscription as `required = false`, and the graph author chose not to wire that producer in. The consumer must function without that signal.

---

## Default-Value Index

Every topic has a documented default that subscribers fall back to. Defaults are encoded in each module's `on_setup()` (where the consumer caches "last seen good" payloads); this doc is the canonical source.

| Topic | Producer | First-tick / absent default | Notes |
|---|---|---|---|
| `neuro.state` | NeurochemState | `dopamine = 0.45, serotonin = 0.5, reward_signal = 0.0, all *_scale = 1.0` | EPM uses scale=1.0; ActionDecoder treats reward=0.0 as "no learning signal." |
| `reality.<group>.<modality>` | EPM | `winner_id = -1, latent = zero vector(projection_dim), tle = 0, is_novel = false` | LateralVoter excludes such tokens from the trust softmax; if all tokens this tick have `winner_id = -1`, voter republishes previous consensus or zero. |
| `consensus.<level>` | LateralVoter | `fused_embedding = zero(projection_dim), fused_tle = 0, active_modality = ""` | ActionDecoder emits `accel = 0` if the consensus is zero. |
| `drive.errors` | HomeostaticDrive | `errors = {channel: 0 for each}, urgency = 0` | ActionDecoder treats as "no goal pressure" (full exploration). |
| `prediction.<modality>` | DescendingPredictor | `predicted_latent = zero(target_dim), confidence = 0` | EPM does not subtract zero (no-op); equivalent to "no prediction available." |
| `sequence.motif.<source>` | SequenceGNG | `motif_id = -1, phase = 0, predicted_next_id = -1, match_confidence = 0` | GNGRollout sees no teleport; falls back to pure stochastic. |
| `motor.chunks` | MotorRepertoire | `chunks = []` | ActionDecoder dispatches no chunks. |
| `action.out` | ActionDecoder | `accel = 0, chunk_id = -1, probe = false` | Host emits zero force on first tick. |
| `rollout.result` | GNGRollout | `trajectories = [], terminal_values = [], entropy = 0` | ActionDecoder uses count-table proxy for epistemic value. |
| `motor.play.stream` | MotorRepertoire | `actions = []` | ActionDecoder reverts to scalar emission. |
| `reality.proprio.<sensor>` | host | The host bridges defaults from the body schema (e.g. position = origin, hunger = 1.0). | If a sensor is absent for a body schema, HomeostaticDrive zero-fills its corresponding channel. |
| `events.<name>` | host | No delivery → no effect. | Events are sparse by definition; no first-tick value is required. |
| `fitness.score` | OgmaInstance internal | `cumulative_drive_reduction = 0, ticks_alive = 0` | Phase-6 only. |

---

## Implementation Pattern

Every module that subscribes to topics whose defaults matter caches a "last-seen-good" payload at the START of every tick:

```cpp
void EPM::tick(uint64_t tick_id) {
    // Refresh cached neuro state. nullptr if producer hasn't run yet AND
    // there's no Feedback path ferrying t-1 values into us.
    auto neuro_msg = bus_->last_value(topics::kNeuroState);
    NeuroState const& neuro = neuro_msg
        ? *static_cast<NeuroState const*>(neuro_msg.get())
        : default_neuro_;     // populated in on_setup() with the table above

    // ... use neuro.epsilon_b_scale, etc.
}
```

The `default_*` member is a const POD initialized once in `on_setup()`. No allocation on the hot path. No nullptr propagation.

For Feedback subscriptions specifically, the Bus's `begin_tick(t)` rotation guarantees that when the consumer reads at tick t, it sees the producer's tick t-1 value. If t = 0, the consumer reads `nullptr` and uses the default.

---

## Invariant Per Module

Every Phase 1 module's contract MUST cite this doc in its "First tick" failure-mode row. The invariant: **no Phase 1 module throws or produces NaN on tick 0**, regardless of which optional inputs are absent.

---

## Audit Procedure

The Phase 0 deliverable for this contract is:

1. Each primitive contract under `docs/primitives/<name>.md` has a row in its "Failure Modes" table for "First tick: X not yet published" with the default value taken from the table above.
2. The `pair_*` tests under `cpp_core/tests/ogma/pair/` exercise the first-tick path explicitly: tick 0 is run, `last_value()` returns nullptr, the consumer uses defaults, no exceptions.
3. The Phase 2 thin-slice replay harness asserts that no module emits NaN in any field of any payload on tick 0.
