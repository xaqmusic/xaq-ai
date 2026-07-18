# HomeostaticDrive — Primitive Contract

**Phase 1 dependency position:** 4 of 9 (depends on LateralVoter; precedes ActionDecoder).
**Header:** `cpp_core/include/ogma/modules/HomeostaticDrive.hpp` (Phase 1 deliverable).
**Reference impl:** none — this is one of the five new primitives. Closest v3 analog is `src/ami_ogma_v3/active_inference.py:NodeValenceMap`, which v4 reinterprets as a drive-predictive estimator owned by ActionDecoder.

---

## Purpose

HomeostaticDrive is the bath's **goal-pressure source**. Where v3's "active inference" is post-hoc reward credit assignment, HomeostaticDrive computes setpoint deviations every tick and broadcasts the resulting drive errors before any reward has been observed. The contract is taken from `docs/v4_algorithmic_gaps.md` (Primitive 4).

It exposes:

- **Per-channel drive errors** — one scalar per declared setpoint channel (`energy`, `integrity`, `novelty_satiation`, plus any host-declared body-specific channels). Computed as `error = current - target` after channel-specific scaling.
- **Urgency scalar** — `max(|error|)` across all channels, normalized to [0, 1]. Modulates ActionDecoder's exploit-vs-explore trade-off in the EFE policy.

The published `drive.errors` is what makes the v4 ActionDecoder *drive-grounded* rather than valence-driven. The v4 valence map (which lives **inside ActionDecoder**, not here) is reinterpreted as a learned predictor of which (state, proprio) pairs reduce drive errors — i.e. `V(state, proprio) = expected drive-error reduction from acting in this state`. That re-interpretation is documented in ActionDecoder.md; HomeostaticDrive's job stops at publishing the *current* error vector.

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `reality.proprio.` | Direct | `ProprioToken` | host | yes (≥1 match) | Wildcard prefix subscription. The drive computation expects specific sensor names — see "Channel mapping" below. |
| `events.hit` | Direct | `EnvEvent` | host | no | Resets `energy` toward target by `event.intensity * energy_replenish_per_hit`. |
| `events.miss` | Direct | `EnvEvent` | host | no | Optional integrity penalty; also drains energy. |

The drive's input fan-in is bounded — it's a deliberately small primitive. Anything more is signalling that channels should be added to the schema, not new input topics.

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `drive.errors` | `DriveErrors` (singleton) | every tick |

`DriveErrors` fields:

- `errors: map<string, float>` — channel name → signed deviation `current - target`. Sign convention: positive means "above setpoint" (e.g. too much energy = positive `energy` error; too little = negative).
- `urgency: float ∈ [0, 1]` — `max(|errors[c]| / urgency_normalizer[c])` clamped.

---

## Parameter Schema

The schema is extensible: new channels are declared via the `channels` array, and each channel binds to a specific proprioceptive topic.

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `channels` | vector<string> | ConstructionOnly | `["energy", "integrity", "novelty_satiation"]` | — | Declared drive channels. Each name appears as a key in `drive.errors`. |
| `setpoints` | map<string, double> via `vector<double>` | HotMutable | `[0.8, 1.0, 0.5]` | per-channel | Target values; one entry per channel in declaration order. |
| `urgency_normalizers` | vector<double> | HotMutable | `[1.0, 1.0, 0.5]` | (0, 10] | Per-channel scale factor that converts `|deviation|` into a [0, 1] urgency contribution. |
| `channel_input_topics` | vector<string> | ConstructionOnly | `["reality.proprio.energy", "reality.proprio.integrity", "reality.proprio.scent"]` | — | Per-channel proprioceptive source. The first scalar of the matched `ProprioToken.values` is read as the current value. |
| `energy_replenish_per_hit` | double | HotMutable | 0.4 | [0, 1] | Each `events.hit` reduces the `energy` deficit by this fraction (clamped to setpoint). |
| `energy_drain_per_tick` | double | HotMutable | 0.0005 | [0, 1] | Per-tick passive energy drain (metabolic cost). Independent of motion. |
| `integrity_drain_per_miss` | double | HotMutable | 0.05 | [0, 1] | Each `events.miss` lowers the `integrity` channel. |
| `novelty_satiation_alpha` | double | HotMutable | 0.001 | (0, 1] | EMA factor for novelty satiation: tracks the long-window EMA of `consensus.0.fused_tle` and uses it as the channel's "current value." Saturates over time → encourages re-engagement with novelty. |
| `urgency_clamp_lo` | double | HotMutable | 0.0 | [0, 1] | Lower clamp on the published urgency scalar. |
| `urgency_clamp_hi` | double | HotMutable | 1.0 | [0, 1] | Upper clamp. |

### Channel mapping

The default schema declares three channels and binds each to a specific topic:

| Channel | Source topic | Computation |
|---|---|---|
| `energy` | `reality.proprio.energy` (or hunger inverted) | `current = ProprioToken.values[0]`; modified by `energy_replenish_per_hit` and `energy_drain_per_tick`. |
| `integrity` | `reality.proprio.integrity` | `current = ProprioToken.values[0]`; reduced by `events.miss`. |
| `novelty_satiation` | `consensus.0` (subscription added when channel is in declared list) | `current = EMA(consensus.0.fused_tle, alpha = novelty_satiation_alpha)`. |

Hosts can add channels by editing the graph config — e.g. a Cell-environment robot adds a `temperature` channel with its own thermistor topic. The contract scales with the body schema.

When a channel's source topic is missing, the channel is reported with `error = 0` and `current` held at the previous value — never NaN, never crash.

---

## Invariants (per tick)

1. HomeostaticDrive publishes exactly one `DriveErrors` per tick.
2. Every declared channel has an entry in `errors`.
3. `urgency ∈ [urgency_clamp_lo, urgency_clamp_hi]` always.
4. Channel current-value updates use only the per-channel proprioceptive source declared in `channel_input_topics` plus the per-channel event hooks declared above. No cross-channel coupling.
5. `energy ∈ [0, setpoint + 1.0]` after every update (energy can't go below zero — agent is "out of fuel" but not negative).
6. `novelty_satiation`'s EMA never resets; it persists across episodes within one OgmaInstance.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| First tick: no proprio deliveries yet | Use channel current = setpoint (zero error). Urgency = 0. |
| Channel source topic missing for the entire run | Publish `error = 0` for that channel; log warning every 1000 ticks. |
| `setpoints.size() != channels.size()` at construction | Throw `std::invalid_argument`. |
| Hot-patch attempts to remove a channel mid-run | Throw — channel removal requires `RemoveNodeOp` then `AddNodeOp` with the new schema. |
| `events.hit` with `intensity > 1.0` | Clamp to 1.0; log warning once per session. |

---

## Latency Budget (Pi5 Cortex-A76, full module graph)

- **Hot path:** ≤ 30 µs for the default 3-channel schema; scales linearly with channel count.
- **Construction:** ≤ 100 µs.
- **Memory:** ≤ 1 KB per instance (small map of channel state + EMA storage).

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Setpoint at start:** with no events and current values at setpoint, `errors[c] == 0` and `urgency == 0` for every channel.
- **Energy decay:** with no `events.hit` and no proprio deliveries, `energy` decays by `energy_drain_per_tick` per tick. After 1000 ticks the deficit equals `1000 * energy_drain_per_tick`.
- **Hit replenish:** one `events.hit{intensity = 1.0}` reduces the energy deficit by `energy_replenish_per_hit * deficit_before`. Clamps at setpoint.
- **Urgency monotonicity:** as any single channel's |error| grows, urgency grows monotonically (other channels held constant).
- **Novelty satiation:** stepping with constant `consensus.0.fused_tle = 1.0` for 5000 ticks drives `novelty_satiation` toward 1.0 along the expected EMA curve.
- **Channel addition via hot-patch:** sequence `RemoveNodeOp` + `AddNodeOp{channels: [..., "temperature"]}` produces a `drive.errors` payload with the new key on the next tick.

### 2. Pair tests

- **`pair_homeostatic_actiondecoder`**: HomeostaticDrive publishes `drive.errors`; ActionDecoder reads it (Direct) and computes a different action than when `drive.errors` is absent. Verifies the drive-grounded EFE seam.
- **`pair_homeostatic_lateralvoter`** (only if `novelty_satiation` channel is declared): verifies the `consensus.0.fused_tle` → `novelty_satiation` EMA in integration.

### 3. Latency

`tick()` p99 ≤ 30 µs (3 channels) on Pi5 over 10⁴ runs.

### 4. Determinism

Identical schedules of proprio/event deliveries produce bitwise-identical `DriveErrors` payloads.

### 5. Behavioural target (sanity)

In a synthetic environment where `reality.proprio.energy` decays linearly and `events.hit` is delivered when an action threshold is exceeded, an agent driven solely by `urgency`-modulated random actions reduces mean `energy` deficit by ≥ 30% over 10000 ticks vs. a control with no events. (This is a loose test that the urgency signal is *useful* — Phase 2 thin-slice tightens it.)

---

## Notes vs. the v3 Reference

- **No 1:1 v3 module.** The closest piece is `NodeValenceMap`, which v3 used for credit assignment. v4 splits responsibilities: HomeostaticDrive owns *current* drive errors; the valence-as-predictor lives inside ActionDecoder.
- **No reward signal here.** Reward (dopamine - baseline) lives in `neuro.state`. HomeostaticDrive's `drive.errors` is the *prior* over what states should be visited; reward is the *posterior* signal of which transitions worked. Both are inputs to ActionDecoder.
- **Hand-declared setpoints, not learned.** Per `v4_algorithmic_gaps.md` Open Question #3 (resolved): hand-declared for the MVP. Learned setpoints are post-Phase-6.
- **Hormones are future work.** `v4_refactor.md` mentions cortisol/estradiol/testosterone as future drive channels. These would land here as additional declared channels with their own decay kinetics; out of scope for Phase 1.
