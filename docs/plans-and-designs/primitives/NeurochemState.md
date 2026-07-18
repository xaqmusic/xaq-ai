# NeurochemState — Primitive Contract

**Phase 1 dependency position:** 1 of 9 (no upstream dependencies — first to implement).
**Header:** `cpp_core/include/ogma/modules/NeurochemState.hpp` (Phase 1 deliverable).
**Reference impl:** `src/ami_ogma_v3/neurochemical.py:30–331`.

---

## Purpose

`NeurochemState` is the broadcast hormone-and-neurotransmitter layer for one OgmaInstance. It integrates per-tick TLE samples, environment events, and proprioceptive scalar streams into a small bundle of dimensionless signals that every other module reads:

- **dopamine / serotonin** — bounded [0, 1] state variables with per-tick decay and event-driven boosts. v3 calls these the "transmitter pair"; the names are biological referents, not literal models.
- **reward_signal** — `dopamine - baseline`, the credit-assignment signal `ActionDecoder` and `MotorRepertoire` consume.
- **scaling factors** — four dimensionless multiplicands (`epsilon_b_scale`, `min_insertion_error_scale`, `mitosis_threshold_scale`, `novelty_threshold_scale`) that shape EPM/GNG learning dynamics across the bath. v3's `NeurochemicalState.epsilon_b_scale()` and friends are the source.

It does **not** own valence or drive: those live in HomeostaticDrive (drive-error setpoints) and the per-instance `valence_map` reinterpreted under HomeostaticDrive (state-conditional drive-reduction estimate). Removing the valence-map ownership from NeurochemState relative to v3 is the single largest semantic change.

---

## Input Topics

NeurochemState is the most input-heavy module in the bath. Every signal that v3's `NeurochemicalState.on_*()` API consumed becomes an explicit Bus subscription here.

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `reality.` | **Feedback** | `RealityToken` | every EPM | yes (≥1 match) | Reads each EPM's TLE from the **previous** tick. Breaks the NeurochemState↔EPM cycle (Pitfall #1 in plan). |
| `consensus.0` | Direct | `ConsensusToken` | LateralVoter | no | When present, `consensus.0.fused_tle` augments per-modality TLE in the dopamine drive. |
| `events.hit` | Direct | `EnvEvent` | host | no | Each delivery boosts dopamine by `da_hit_gain * intensity`. |
| `events.miss` | Direct | `EnvEvent` | host | no | Decreases dopamine by `da_miss_decay * intensity`. |
| `events.brick` | Direct | `EnvEvent` | host | no | Mid-strength dopamine pulse (Breakout-class environments). |
| `events.wall_stuck` | Direct | `EnvEvent` | host | no | Decreases dopamine, raises `mitosis_threshold_scale` (frustration → less plasticity). |
| `events.whisker_bump` | Direct | `EnvEvent` | host | no | Same shape as `wall_stuck` but lighter. |
| `reality.proprio.whisker` | Direct | `ProprioToken` | host | no | Continuous whisker integral; modulates serotonin baseline. |
| `reality.proprio.scent` | Direct | `ProprioToken` | host | no | Scent gradient; positive delta boosts dopamine, mirrors v3 `on_scent`. |
| `reality.proprio.hunger` | Direct | `ProprioToken` | host | no | Hunger level → drives dopamine baseline. |
| `reality.proprio.pheromone` | Direct | `ProprioToken` | host | no | Pheromone cue; boosts dopamine when rising. |
| `reality.proprio.travel` | Direct | `ProprioToken` | host | no | Travel level (cumulative distance); raises serotonin (satiation through motion). |

Subscription wildcard rule: NeurochemState subscribes to `reality.` (prefix) with **Feedback** kind for the EPM TLE channels. Concretely it filters incoming `reality.*` payloads to those whose module produces `RealityToken` (vs. `ProprioToken`); the proprioceptive subscriptions are explicit per-sensor entries above to keep type dispatch trivial.

### Why the EPM cycle is Feedback

EPM's tick reads `neuro.state` (current tick) and writes `reality.<modality>` carrying its TLE (current tick). NeurochemState's tick reads each EPM's TLE (which it just wrote) and updates dopamine/serotonin/scales (publishing `neuro.state`). If NeurochemState ran in the same tick as EPM and consumed current-tick TLE, EPM would already have committed to a winner using a stale `neuro.state` from the previous tick — fine — but NeurochemState's update would lag one tick relative to that winner. Either way, one direction is one tick stale; the question is which.

Choice: **NeurochemState reads EPM(t-1) via Feedback subscription; EPM reads NeurochemState(t) via Direct subscription.** Rationale: the chemistry is biologically a slow integrator over the last tick of activity, so reading the *previous* tick's TLE is correct. EPM consuming current-tick scalings is also correct: a hit event in tick t should immediately affect plasticity in tick t (via the dopamine spike NeurochemState publishes in tick t).

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `neuro.state` | `NeuroState` (singleton) | every tick |

`NeuroState` fields (see `Topics.hpp`):

- `dopamine ∈ [0, 1]`, `serotonin ∈ [0, 1]`
- `reward_signal = dopamine - dopamine_baseline` (typically `[-0.45, +0.8]`)
- `epsilon_b_scale ∈ [0.3, 2.5]`
- `min_insertion_error_scale ∈ [0.5, 1.8]`
- `mitosis_threshold_scale ∈ [0.6, 1.8]`
- `novelty_threshold_scale ∈ [0.5, 1.5]`

The four scaling factors are pure functions of (dopamine, serotonin, recent TLE EMA, event flags), defined identically to v3's `NeurochemicalState.*_scale()` methods. They are not learned and have no state of their own beyond what dopamine/serotonin already encode.

---

## Parameter Schema

Defaults match v3 frozen baseline (`src/ami_ogma_v3/neurochemical.py`); deviations require updating `docs/v3_baseline.md`.

| Key | Type | Mutability | Default | Description |
|---|---|---|---|---|
| `da_baseline` | double | HotMutable | 0.20 | Resting dopamine level. Subtracted from dopamine to form `reward_signal`. |
| `ht_baseline` | double | HotMutable | 0.65 | Resting serotonin level. |
| `da_decay` | double | HotMutable | 0.88 | Per-tick exponential-decay coefficient toward `da_baseline`. |
| `ht_decay` | double | HotMutable | 0.93 | Per-tick exponential-decay coefficient toward `ht_baseline`. |
| `intrinsic_da_gain` | double | HotMutable | 0.05 | Dopamine pulse per unit positive TLE delta (`prev_tle - cur_tle`). |
| `scent_da_rate` | double | HotMutable | 0.25 | Dopamine pulse per unit positive scent delta. Falling scent fires nothing. |
| `travel_da_rate` | double | HotMutable | 0.02 | Dopamine pulse per unit `reality.proprio.travel` (speed × openness). |
| `whisker_ht_rate` | double | HotMutable | 0.02 | Serotonin drain per unit `reality.proprio.whisker`. |
| `hunger_ht_rate` | double | HotMutable | 0.01 | Serotonin drain per unit `reality.proprio.hunger`. |
| `pheromone_ht_rate` | double | HotMutable | 0.005 | Serotonin drain per unit pheromone in excess of `pheromone_threshold`. |
| `pheromone_threshold` | double | HotMutable | 0.30 | Below this, pheromone is ignored (fresh-passing noise). |
| `wall_stuck_da_drain` | double | HotMutable | 0.35 | Dopamine drain per `events.wall_stuck`. |
| `wall_stuck_ht_drain` | double | HotMutable | 0.15 | Serotonin drain per `events.wall_stuck`. |
| `event_coupled_da` | bool | HotMutable | false | When false (default), `events.hit/miss/brick` only update telemetry counters. When true, they pulse dopamine. v3 ablation flag. |
| `event_coupled_ht` | bool | HotMutable | false | When false, `events.miss` does not drain serotonin. When true, it does. |
| `da_hit_gain` | double | HotMutable | 0.45 | Dopamine pulse per `events.hit` when `event_coupled_da` is true. |
| `da_brick_gain` | double | HotMutable | 0.65 | Dopamine pulse per `events.brick` when `event_coupled_da` is true. |
| `da_miss_drop` | double | HotMutable | 0.25 | Dopamine drop per `events.miss` when `event_coupled_da` is true. |
| `ht_miss_drop` | double | HotMutable | 0.30 | Serotonin drop per `events.miss` when `event_coupled_ht` is true. |
| `master_seed` | int64 | ConstructionOnly | 0 | RNG namespace seed. Forward-compat — NeurochemState has no stochastic ops in this contract. |

### Routing Notes (the v3 signal-channel mapping)

- **Dopamine boosts** come from: TLE drop (intrinsic_da_gain), scent rising (scent_da_rate), travel (travel_da_rate). Plus event-coupled hit/brick if the flag is on.
- **Dopamine drops** come from: wall_stuck (wall_stuck_da_drain). Plus event-coupled miss.
- **Serotonin drains** come from: whisker, hunger, pheromone-excess, wall_stuck. Plus event-coupled miss.
- **Serotonin boosts** are absent in v3 — serotonin only decays toward baseline or drains. (Future hormone channels can change this.)
- **Scaling factor formulas** (matching v3 exactly):
  - `epsilon_b_scale = 0.3 + 2.2 * dopamine` → range [0.3, 2.5]
  - `min_insertion_error_scale = 1.8 - 1.3 * serotonin` → range [0.5, 1.8]
  - `mitosis_threshold_scale = 0.6 + 1.2 * serotonin` → range [0.6, 1.8]
  - `novelty_threshold_scale = 0.5 + 1.0 * dopamine` → range [0.5, 1.5] (v4 addition; v3 had no equivalent)

---

## Invariants (per tick)

1. `dopamine, serotonin ∈ [0, 1]` after every update.
2. `reward_signal = dopamine - dopamine_baseline` (always — never store reward separately).
3. All four `*_scale` fields are within their declared ranges.
4. The published `NeuroState.tick_id` equals `Scheduler::current_tick()`.
5. NeurochemState publishes exactly one `neuro.state` per tick — no skips, no duplicates. (First tick publishes default values per `_first_tick.md`.)
6. The TLE EMA decays monotonically when no `reality.*` Feedback delivery occurred this tick (i.e. some EPM was inactive); never grows from inactivity.
7. Hot-mutable param updates do not cause discontinuities greater than one tick's normal decay step; if a decay rate is changed mid-run, the next tick uses the new rate without back-applying it.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: no `reality.*` deliveries yet | Use default state (`dopamine=0.45, serotonin=0.5`, scales=1.0). Publish; no exception. |
| Required input topic missing throughout the run | Module logs a one-shot warning at first tick; defaults remain. (Required is per-payload-type, not per-topic.) |
| Param out of range on `set_param` | Throw `std::out_of_range` from `on_param_change`. Scheduler catches → marks instance fatal. |
| `events.hit` with `intensity ≤ 0` | Treated as no-op (defensive — host bug, but should not crash the tick). |
| Internal NaN in dopamine or serotonin | Reset that variable to its baseline, log error, continue. (Never propagate NaN to other modules.) |

---

## Latency Budget (Pi5 Cortex-A76, full module graph)

- **Hot path (`tick()`):** ≤ 25 µs. Module is bookkeeping-only — a handful of float multiplies and clamps; the only heavyweight operation is iterating subscribed topic payloads, which is bounded by the number of EPMs (~5).
- **Construction:** ≤ 200 µs (param parse + Bus subscribe registration).
- **Memory:** ≤ 4 KB working set per instance (no buffers, no histories beyond the TLE EMA scalar).

---

## VV&A Criteria (Phase 1 acceptance)

A NeurochemState implementation is admitted to Phase 2 integration when **all** of the following pass on three consecutive seeds (42, 43, 44):

### 1. Unit tests (no Bus, no neighbours; pure-function tests against golden tensors)

- **Decay parity:** with no events and no TLE deliveries for 1000 ticks, dopamine and serotonin track v3's `tick({}, 0.0)` outputs to within `1e-6` per-tick.
- **Hit response:** a single `events.hit{intensity=1.0}` raises dopamine by exactly `da_hit_gain` (within `1e-7`).
- **TLE EMA:** stepping with TLE=[0.1, 0.2, 0.3] for 100 ticks produces the same EMA as v3's `_running_avg_tle` updater.
- **Scale function bounds:** every scaling factor stays inside its declared range across 10⁵ random (dopamine, serotonin) pairs sampled from [0,1]².
- **Invariant 1:** dopamine and serotonin remain in [0,1] across 10⁵ pseudo-random ticks (events generated by `derive_rng(seed, "vva.neurochem.events")`).
- **Param hot-mutate:** changing `dopamine_decay` from 0.985 to 0.99 between ticks 100 and 101 produces a one-tick discontinuity exactly equal to the decay-rate difference.

### 2. Pair tests (real Bus; real nearest neighbour — EPM)

The pair test (`pair_neurochem_epm`) wires a `MockEPM` (deterministic TLE schedule) and a real `NeurochemState` through a real `InProcessBus` for 5000 ticks.

- **Cycle correctness:** EPM's published TLE in tick t is observed by NeurochemState as a Feedback delivery in tick t+1, never tick t. (Trace assertion.)
- **End-to-end behavioural parity vs. v3:** running the same TLE schedule through v3's `NeurochemicalState` Python and the C++ port produces dopamine/serotonin trajectories within `1e-4` (Linf norm) over 5000 ticks. Cross-language determinism is feasible here because the math is scalar-only (no Eigen / numpy float-order ambiguity).

### 3. Latency

`tick()` p99 latency ≤ 25 µs on Pi5 over 10⁵ runs with 5 EPM-token deliveries per tick.

### 4. Determinism

Two runs with the same `master_seed`, same input stream, same param schedule produce bitwise-identical `neuro.state` outputs. Test under `pair_neurochem_epm` with `--paired-seed 42`.

---

## Notes on the v3 → v4 Port

- **API change:** v3's `tick(events_dict, tle)` signature is gone. Inputs are now Bus-delivered. The implementation should keep an internal "last sampled values" cache so each tick computes deterministically regardless of subscription delivery order.
- **Eliminated coupling:** v3 `NeurochemicalState.set_active_inference_components(valence_map, transition_model)` is removed. Valence semantics move to HomeostaticDrive; the count-table-based "EFE" of v3 is replaced by `GNGRollout`.
- **Reward routing:** v3 routed `reward_signal` directly through Python attribute access in the action loop. v4 puts `reward_signal` in `NeuroState` as a published field; `ActionDecoder` reads it from the Bus.
- **Hormones (Phase 3 stretch):** the v4 design names additional drive channels (cortisol = sustained urgency, estradiol/testosterone = setpoint shifts) but they are NOT in this contract. They land in HomeostaticDrive as additional setpoints; NeurochemState stays scoped to dopamine + serotonin.
