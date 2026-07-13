# KeyframeAverager — Sensor / Motor Downsampler

**Status:** shipped (commit pending — Phase v5.2.4b).
**Cross-cutting topic:** enables multi-rate brain by downsampling fast streams into rolling-window keyframes, paired with EPM `process_every_n_ticks` (Phase v5.2.4a).

---

## 1. Why this exists

The brain's perceptual EPMs all run at the substrate's per-tick rate (~60Hz on Cell). The user observation 2026-05-09: food-approach takes seconds, but SequenceGNG only encodes 5-tick (~83ms) windows. The brain can't see the temporal shape of goal-directed behavior because every signal arrives too fast.

EPMs are nominally allowed to operate at any time horizon. Phase v5.2 builds the missing primitive: a downsampler that buffers high-rate inputs in a rolling window and exposes the per-tick mean, which a sub-rate EPM (`process_every_n_ticks`) can consume.

The first concrete use case (Phase v5.2.4c) is **motor self-monitoring**: average the body's recent motor commands over ~833ms (50 ticks), feed to a slow motor-pattern EPM, let the brain develop a self-model of "what have I been doing." Closes the predictive-coding loop the user articulated: "am I on the correct long-horizon path at this moment given my short and long sensor inputs AND my recent motor pattern?"

Sensor downsampling (vision keyframes) is the symmetric use case.

---

## 2. The contract

```
input_topic (ActionOut or ProprioToken, payload_kind discriminated)
       │
       ├──► handle_input — per-input frame appended to rolling buffer (size window_size)
       │
       ▼
tick(t):
   if buffer non-empty:
       mean = (1/N) · Σ frames                   # per-element mean over buffer
       publish ProprioToken{tick_id=t, sensor=sensor_label or id_, values=mean}
                                                  # to output_topic
```

**Always publishes per tick** (when buffer is non-empty). Downstream slow EPMs use `process_every_n_ticks` to gate consumption — this primitive doesn't gate output itself. The split keeps responsibilities cleanly separated: KeyframeAverager owns the rolling-mean math; EPM owns the consumption rate.

---

## 3. Parameters

| key | mutability | default | meaning |
|---|---|---|---|
| `input_topic` | ConstructionOnly | `"action.out"` | Source topic. Type discriminated by `payload_kind`. |
| `output_topic` | ConstructionOnly | `"reality.proprio.motor_avg"` | Destination — always a `ProprioToken`. |
| `payload_kind` | ConstructionOnly | `"action_out"` | One of `"action_out"` (read `.accel` as 1-element vector) or `"proprio_token"` (read `.values` as N-element vector). |
| `window_size` | HotMutable | `50` | Number of recent frames to average. At 60Hz, 50 ≈ 833ms. |
| `sensor_label` | ConstructionOnly | `""` (uses `id_`) | Optional `ProprioToken.sensor` field for the keyframes. |

`window_size` is HotMutable — tests confirm trim-on-shrink behavior is honored when reduced at runtime.

---

## 4. Wiring example — bilateral motor self-model

The Phase v5.2.4c configuration. Brainless-whiskers + REINFORCE + α=0.5, with two motor-keyframe pipelines:

```json
{
  "id": "keyframe_motor_left",
  "type": "KeyframeAverager",
  "params": {
    "input_topic":  "action.left",
    "output_topic": "reality.proprio.motor_avg_left",
    "payload_kind": "action_out",
    "window_size":  50,
    "sensor_label": "motor_avg_left"
  }
},
{
  "id": "keyframe_motor_right",
  "type": "KeyframeAverager",
  "params": {
    "input_topic":  "action.right",
    "output_topic": "reality.proprio.motor_avg_right",
    "payload_kind": "action_out",
    "window_size":  50,
    "sensor_label": "motor_avg_right"
  }
},
{
  "id": "epm_motor_slow_left",
  "type": "EPM",
  "params": {
    "modality_group": "motor",
    "modality_name":  "pattern_left",
    "encoder_kind":   "rbf",
    "input_topic":    "reality.proprio.motor_avg_left",
    "projection_dim": 32,
    "process_every_n_ticks": 50,
    "subtract_descending_prediction": true
  }
},
{
  "id": "epm_motor_slow_right",
  "type": "EPM",
  "params": {
    "modality_group": "motor",
    "modality_name":  "pattern_right",
    "encoder_kind":   "rbf",
    "input_topic":    "reality.proprio.motor_avg_right",
    "projection_dim": 32,
    "process_every_n_ticks": 50,
    "subtract_descending_prediction": true
  }
}
```

Effect: every tick, KeyframeAverager publishes `reality.proprio.motor_avg_left/right`. Every 50 ticks, the slow-motor EPMs consume the keyframe and run their encoder + GNG step; on the other 49 ticks they republish their last RealityToken (per Phase v5.2.4a). voter_0 sees consistent motor-pattern contributions in the consensus latent, encoded at the second timescale.

For the predictive-coding loop, add `reality.motor.pattern_left` and `reality.motor.pattern_right` to the `descend` module's `targets` list — then surprise EMA on the motor channel measures "is my behavior consistent with my long-horizon plan."

---

## 5. Invariants

1. **First frame establishes payload_dim.** Subsequent frames with mismatched dim are silently dropped (returns from `handle_input` without buffering). Diagnostic: `total_inputs_seen()` will lag relative to publish rate.
2. **Output dim = input dim.** No projection / dimensionality change.
3. **Empty buffer → no publish.** First tick before any input arrives is silent.
4. **`payload_kind` and `input_topic` are ConstructionOnly.** Switching mid-run would break type contracts. Mutability schema enforces.
5. **Producer-id gating** — input handler starts with `if (!input_allowed(payload->producer_id)) return;` per `_aux_send_routing.md` contract.

---

## 6. Tests

`cpp_core/tests/ogma/test_keyframe_averager_streams.cpp`:

- `ActionOutRollingMean` — window=4, scalar inputs 1..4 over 4 ticks, verify rolling mean = 1.0, 1.5, 2.0, 2.5.
- `WindowTrimsOldestFrames` — window=4, send 6 frames, verify final mean = mean of last 4.
- `ProprioTokenElementwiseMean` — window=3, vector inputs (1,2,3), (4,5,6), (7,8,9), verify mean = (4,5,6).

3/3 PASS as of commit.

---

## 7. Verification

When changing this module:

```bash
cd /home/xaqmusic/ami-ogma/cpp_core
cmake --build build --target test_keyframe_averager_streams && ctest -R "KeyframeAverager"
```

Should return 3/3 PASS, 0 FAIL.

---

## 8. Related code

- `cpp_core/include/ogma/modules/KeyframeAverager.hpp`
- `cpp_core/src/ogma/modules/KeyframeAverager.cpp`
- `cpp_core/src/ogma/ModuleRegistry.cpp` — registers `"KeyframeAverager"`
- `cpp_core/src/ogma/modules/EPM.cpp` — Phase v5.2.4a `process_every_n_ticks` is the natural downstream consumer
- `docs/primitives/_module_lifecycle.md` — authoring contract this module follows
- `docs/v5_plan.md` §3 — v5.2 phase context
