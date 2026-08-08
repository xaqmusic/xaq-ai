> **Ported from the pre-split `ami-ogma` repo, 2026-07-25. HISTORICAL — dated 2026-05-04 (v4
> Phase 6.5), i.e. before the Cell rebuild and before the picrawler active-inference port.**
> Superseded on method by [`../brain_building_doctrine.md`](../brain_building_doctrine.md) and on
> procedure by [`../../CLAUDE.md`](../../CLAUDE.md). Kept because it is the most complete single
> statement of the *measurement protocols* — what a claim required, and how the staged
> promote-or-kill discipline was actually run. Claim vocabulary here predates the "no external
> reward" correction. `ami_ogma`/`ogma`/`AMI-Ogma` == xaq.

# AMI-Ogma — Methodology and Empirical Status

**Status:** Phase 6.5.16 (2026-05-04)
**Author:** Joseph Butera III
**Branch:** v4-phase6-5-1
**Scope:** A single coherent picture of what AMI-Ogma is, what claims
it makes, and what protocols produce those claims. Cross-references
the deeper retros for derivations and falsifications.

---

## 1. What AMI-Ogma is

AMI-Ogma is a multi-modal, embodied active-inference substrate. It is
a "System 1" subconscious architecture — gradients flowing through
plasticity in a publish-subscribe module bus, not a transformer
solving objectives.

The substrate has nine reusable cognitive primitives:
`NeurochemState` (dopamine/serotonin gain control), `EPM` (Episodic
Predictive Modules — RBF/JL encoder + GNG topology + predictor per
modality), `LateralVoter` (cross-modal Hebbian consensus),
`HomeostaticDrive` (multi-channel urgency), `ActionDecoder` (Q-axis
expected-free-energy action selection with empirical forward model),
`SequenceGNG` (motif crystallization on consensus and action streams),
`MotorRepertoire` (chunk library — crystallized motor primitives),
`HomeokineticExploration` (kinesis — drive-flatline detector that
arms randomized exploration episodes), `DescendingPredictor`
(top-down feedback when ≥2 sensors).

Each environment is a **body bridge** — a Godot scene that publishes
proprio/events to and reads action.out from an instantiated
`OgmaInstance`. Three bridges are validated as of this writing:
**Cell** (continuous food-foraging in a 3D arena, 9 modalities, spike
actuation), **MountainCar** (Gym v0 sparse-reward classic), and
**CartPole** (Gym v1 PD-control classic).

The substrate is genuinely body-agnostic: same cognitive primitives,
different module configurations and reward shaping per body bridge.
The configuration is **derived** from a body manifest rather than
hand-tuned per environment.

---

## 2. The body-bridge design pattern

A new environment is added in three steps:

### 2.1 Body manifest (declarative, ~80 lines JSON)

`configs/manifests/<env>.json` declares what the body publishes and
consumes — sensors, actuators, events, runtime kind. No brain choices
are made here; the manifest is body-side.

```json
{
  "name": "cell",
  "sensors": [
    {"name": "imu",      "dim": 4, "modality_group": "kinematic"},
    {"name": "scent",    "dim": 8, "modality_group": "proprio"},
    {"name": "whisker_0","dim": 1, "modality_group": "whisker"},
    ... (9 sensors total)
  ],
  "actuators": [{"name": "spike", "bins": 3, "range": [-4, 4]}],
  "events": [
    {"name": "hit",        "density": "sparse_event", "valence": "reward"},
    {"name": "wall_stuck", "density": "sparse_event", "valence": "aversive"}
  ],
  "runtime": {"episodic": false}
}
```

### 2.2 Derivation (`scripts/derive_brain_config.py`)

The script applies inclusion rules (see `docs/v4_brain_derivation.md`
§2 for the full table) to the manifest and emits a complete brain
config. Modules are wired iff their precondition fires. Same
architecture, different surface area per body.

Module inclusion rules (excerpt):
- `EPM`: one per `sensors[]` entry (always).
- `LateralVoter`: always (architecturally required — `ActionDecoder`
  subscribes to `consensus.<level>`).
- `DescendingPredictor`: only when `len(sensors) ≥ 2`.
- `HomeokineticExploration`: NOT for `dense_per_tick` reward
  envs (drive-flatline trigger never fires under dense reward).
- `SequenceGNG` + `MotorRepertoire`: when `chunk_pipeline_useful`
  (continuous + multi-sensor, OR episodic with sufficient episode
  length).
- `GNGRollout`: never (superseded by per-bin forward-model epistemic
  in `ActionDecoder`, Phase 6.5.3.1).

`ActionDecoder` parameter derivation:
- `td_gamma = 0.95` if episodic, else `0.0` (V-table behavior in
  continuous envs).
- `use_chunks = chunk_pipeline_useful`.
- `bins, accel_min/max` from `actuators[0]`.

Validation track record:
- CartPole: derived ≡ hand-tuned minimal (byte-equal).
- Cell: derived = current minus `rollout` and with adaptive baseline
  enabled (caught a real regression — see §5).
- MountainCar: derived = 6-module config; substrate-falsified for
  Gym v0 protocol but works in continuous mode.

### 2.3 Audit checklist (catches what derivation can't)

The derivation rules don't catch every failure mode — there are
body-bridge wiring patterns that compile and run but produce
silent regressions. The audit template (`docs/v4_brain_derivation.md`
§4.4) is run by inspection before launching paired-seed sweeps:

```
[ ] Every published event has a documented consumer module
[ ] Every reward-intent event uses the name `hit` or `miss`
[ ] Event firing rates × NeurochemState gains satisfy the
    saturation invariant (or adaptive baseline is enabled)
[ ] Both events.hit + events.solved fire on goal AND
    events.miss + events.failed fire on failure (the second event
    of each pair triggers NeurochemState's terminal-pulse flush
    so the dopamine/serotonin decay tail does not leak into the
    next episode's TD updates — see Phase 6.5.3.F retro)
[ ] reality.proprio.* topics fire at body tick rate
[ ] Brain perceptual loop forms (EPM nodes grow, visible in 1-Hz diag)
[ ] NeurochemState dopamine variation is non-saturated
[ ] HomeostaticDrive urgency varies meaningfully
```

The fact that the audit is a checklist matters: every "no" is a
probable bug, and several have been historical bugs found this way
(§5).

---

## 3. The body-side reward design pattern

After Phase 6.5.5–6.5.16, a single reward template applies across
all three validated bodies:

```
events.hit FIRES iff:
   short_ema(performance_var) > long_ema(performance_var) × threshold
   AND action causally aligned with desired direction

events.miss FIRES iff:
   undesired_state_var > threshold
   AND refractory has elapsed
```

**The variables and comparison directions differ per body**, but the
shape is the same:

| body | performance_var (rewarded) | aversive_var (penalized) | alignment gate |
|------|----------------------------|--------------------------|----------------|
| MC   | `\|velocity\|`, `\|swing\|` | (no direct miss; goal-only) | `force × velocity > 0` |
| CartPole | `\|theta\|` (lower better) | timeout, fall | `force × theta > 0` |
| Cell | `scent_max` (higher better) | `whisker_max > 0.30` | `forward_speed > 0.5 m/s` |

The two principles that produce the pattern:

1. **Substrate-aligned PE model** (dual-EMA): reward fires only on
   transient outperformance, not constant high state. Naturally
   tracks the substrate's own adaptive-baseline EMA in
   `NeurochemState`. Avoids the saturation invariant violation
   that capped CartPole at random-baseline performance pre-§17
   (`docs/v4_phase6_5_3_action_layer_plan.md` §17).
2. **Causal alignment**: reward fires only when the action was sign-
   aligned with the rewarded state-change. Filters out coasting
   moments where momentum kept the variable high but the action
   itself wasn't building it. Discovered empirically as the single
   biggest behavior improvement in the MC arc (2.6× goals/kt from
   one boolean gate).

See `docs/v4_phase6_5_reward_design.md` for the full iteration arc.

---

## 4. Benchmark methodology — the time-bend

Standard RL benchmarks (Gym v0/v1) assume a frozen policy. AMI-Ogma
is a continuous-learning system; plasticity is always on. Running
the standard protocol against a learning system produces amnesia
(every reset is a discontinuity the brain has no model for).

The user's resolution: **bend time, don't disable systems.** Snapshot
the entire brain state at moment T, run the benchmark protocol with
plasticity ON, then restore the snapshot. The benchmark is still a
frozen-policy evaluation because the brain rolls back; the
substrate's principled dynamics are never compromised.

Realized in `OgmaInstance::clone()` (Phase 6.5.4):
- Each module implements `snapshot_state()` / `restore_state()`
- `clone()` constructs a fresh `InProcessBus`, fresh `OgmaInstance`
  from the same `GraphConfig`, copies the bus topic cache by
  shared_ptr (cheap; Messages are immutable), then applies module
  state snapshots
- Determinism contract verified: `test_clone.cpp` warm-starts a brain
  for 50 ticks, clones it, then runs both for 100 more ticks with
  identical inputs — every tick's action AND dopamine match
  (`ASSERT_DOUBLE_EQ`)

For frozen-eval benchmarking, the body bridges expose:

```
OGMA_*_BENCHMARK=1
OGMA_*_TRAINING_EPISODES=N        # train continuously first
OGMA_*_BENCHMARK_EPISODES=N       # then run N benchmark episodes
OGMA_*_BENCHMARK_MAX_STEPS=N      # each capped at N ticks (200=Gym v0, 500=Gym v1)
```

Lifecycle:
1. Train continuously for `TRAINING_EPISODES`
2. Snapshot brain → `BENCHMARK_START` event
3. Run `BENCHMARK_EPISODES`. **Reload the clone before every
   benchmark episode** so each evaluates the SAME starting brain
   (proper Gym frozen-policy semantics; intra-benchmark plasticity
   is discarded with each restore)
4. Emit `BENCHMARK_END` JSON with `mean_reward / n_full_solves /
   solved_gym_v[01]`
5. Restore snapshot one final time → `BENCHMARK_RESTORED`

See `docs/v4_phase6_5_4_clone_system.md` for the full design and
caveats (Scheduler tick counter not propagated, FrozenJLEncoder
optical-flow state not snapshotted, DescendingPredictor stub).

---

## 5. Empirical results

All results below use the following methodology unless stated:
- Headless Godot with `OGMA_SEED=N`
- `paired_seed_ab.py` for A/B comparisons (deterministic per seed)
- Body bridges: `cell_smoke.py`, `mountain_car_run.py`, `cartpole_run.py`

### 5.1 MountainCar

**Continuous mode (substrate-native, no resets):** seed 44, 20k
ticks:

| variant | goals/kt | best goal time | chunks dispatching |
|---|---|---|---|
| Phase 6.5.9 (dual-EMA + alignment, no chunks) | 0.6 | 253 ticks | 0 |
| Phase 6.5.12 (hybrid trigger + implicit miss) | 0.5 | 319 ticks | yes (276 dispatches in 12k ticks) |

The 253-tick goal is the closest the substrate has come to the Gym
v0 200-tick solve threshold under the substrate-native protocol.

**Gym v0 frozen-eval** (clone-reload between every episode):
```
seed 44, 60-episode train + 100-episode benchmark, 200-tick cap:
  mean_reward     -200.00
  n_full_solves   0/100
  solved_gym_v0   false
```
The substrate's BEST goal-finding episode took 281 ticks; the
200-tick cap chops every successful trajectory. This is the
literature-comparable AMI-Ogma MC number — falsified for Gym v0.
(MC can be solved in continuous mode but not under the cap.)

### 5.2 CartPole

**Standard training** (3 seeds × 200 ep, dual-EMA + force×θ
alignment):
```
μ_eval        27.82  (vs historical 23.32 baseline; +19%)
σ             5.29
max episode   157 ticks
```

**Gym v1 frozen-eval** (seed 42, 200 train + 100 bench, 500-tick cap):
```
mean_reward     27.99
max_reward      87
n_full_solves   0/100
solved_gym_v1   false  (threshold ≥475)
```

The frozen-eval (μ=27.99) tracks training (μ=27.82) — methodology
produces consistent numbers. Gap to solve threshold is ~447 ticks;
analysis in §17 attributes this to policy-machinery limits (Q-table
action resolution insufficient for PD-style fine motor control),
not reward-design issues.

### 5.3 Cell

**Continuous-mode smoke** (10 seeds × 120s, parallel=2, default
world):

| variant | hits/min | seeds w/ hits | chunks (mean) | dispatches (mean) |
|---|---|---|---|---|
| pre-audit | 0.10 | 2/10 | 0.2 | 1.0 |
| Phase 6.5.15 audit alignment | 0.20 | 3/10 | 0.3 | 1.5 |
| Phase 6.5.16 reward refinement | **0.61** | **6/10** | **5.4** | **276.5** |

The 6× lift from pre-audit to Phase 6.5.16 came from two coordinated
changes:
1. **Audit alignment** (Phase 6.5.15): `event_coupled_da: false →
   true` in the live config — the single most-impactful divergence
   from the derivation rules. Cell had been running with anticipatory-
   only reward (scent gradient) for an unknown duration; the
   consummatory dopamine pulse on actual eating was silently
   disabled.
2. **Reward refinement** (Phase 6.5.16): added the dual-EMA scent
   reward + whisker miss event at the body, mirroring the MC
   pattern. This 3× lift over post-audit comes from densifying the
   sparse reward signal — the brain now has gradients in both
   directions (approach food, avoid walls) instead of relying
   entirely on rare consummatory hits.

Best seeds reached 1.5 hits/min, back into the territory where food
navigation is observably happening.

---

## 6. Falsifications and known limits

The substrate is published with what doesn't work alongside what does.

### 6.1 MC under Gym v0 (200-tick cap)

Substrate cannot solve MC under the standard 200-tick cap. Best
goal-finding episode takes 253 ticks; cap chops every success.
Documented as a clean falsification of "substrate solves MC under
literature protocol." Continuous-mode performance (no cap, native
substrate evaluation) is competent.

### 6.2 CartPole at Gym v1 threshold

Substrate stabilizes at ~28-tick survival; Gym v1 threshold is 475.
Gap attributed to Q-table action resolution insufficient for fine
PD-style control, NOT reward shaping (the dual-EMA + alignment
pattern produces +19% over historical baseline but doesn't close
the gap). Open architectural question for future work (see §7).

### 6.3 Chunks-don't-dispatch in high-perceptual-variability tasks

Pre-Phase 6.5.12, chunks crystallized in MC but never dispatched
because consensus motifs at goal-approach were too perceptually
variable to recur. The hybrid trigger (consensus OR drive predicate)
addressed this by adding a drive-state-keyed dispatch path.

The deeper architectural question — should chunks be parameterized
motor primitives with body-state predicates rather than perceptual-
context triggers? — is flagged for the quadruped bridge (Phase 6.6+)
where the same perceptual-variability problem will be more severe.

### 6.4 Cold-start exploration

MC continuous mode shows a significant lull before the first goal-
finding (~1000–20000 ticks depending on seed). A/B testing of
"remove EMA bootstrap" vs "permissive threshold during settled
state" (Phase 6.5.10) showed both options HURT overall performance
relative to the bootstrap+strict baseline. Conclusion: cold-start
delay is a policy-side issue (epsilon-greedy too random/not random),
not a reward-design issue. Reward shaping alone cannot fix it.

### 6.5 Kinesis dormant in MC continuous mode

`HomeokineticExploration` fires when drive change-rate flatlines
relative to its history. In MC continuous mode urgency monotonically
rises and plateaus high — no anomalous drop in change-rate. Gate
self-disengages once "small change" becomes the new normal.
Architecturally correct behavior (per the gate's design) but means
kinesis doesn't help break MC's cold-start lull.

---

## 7. Open architectural questions

Ranked by leverage; each is bigger than any single env's tuning.

### 7.1 State aliasing in EPM (untested)

Does EPM RBF projection capacity cap performance on continuous-state
tasks? Current Cell config: kinematic-group EPMs at projection_dim=32,
whiskers at 16. Testable: bump 2× and re-run paired-seed.

If significant lift, the substrate's perceptual capacity has been
the bottleneck in residual variance — re-prioritizes subsequent
work.

### 7.2 Pre-dispatch prediction gating

The substrate has an empirical forward model in `ActionDecoder`
(`forward_model_: (state, bin) → P(next_state)`). Currently used
for `action_tle` and per-bin epistemic value during action selection,
NOT for chunk dispatch gating.

Could filter chunk dispatches before they fire: "given current state,
what does the forward model predict for chunk's first action? Is
that predicted next-state in the chunk's trigger neighborhood?"
Wilson-CI demotion handles correctness post-hoc; pre-dispatch
filtering is a §16-class follow-up.

### 7.3 Adaptive parameter substrate

Many hand-tuned parameters across the reward design pattern:
`hit_threshold = 1.5`, `miss_threshold = 0.30`, `motion_floor = 0.5
m/s`, EMA alphas, `crystallization_min_observations = 4`,
`drive_tolerance = 0.95`, etc. Per the §16 audit, the principled
answer is "make them adaptive based on the substrate's own
statistics" rather than picking values per env.

This is a substantial architectural change — would need substrate-
internal mechanisms for parameter homeostasis. Larger than this
phase's scope.

### 7.4 Quadruped bridge (Phase 6.6+)

The natural next env. Manifest declares ~20 sensors (joint angles,
IMU, contacts, scent), 12 actuators (joint torques), continuous task
with sparse food rewards.

Will likely surface needs for:
- Parameterized motor primitives (DMP-style) instead of consensus-
  keyed chunks (the chunks-don't-dispatch problem at higher
  perceptual variability)
- Better state aliasing handling (12-D action space requires
  finer Q-table discrimination)
- Coordinated multi-actuator timing (gait emergence)

Whether the body-bridge design pattern survives at this scale is
the next falsification gate the framework can clear or fail.

### 7.5 Cell hierarchical voter

Phase 6.5.0 paired-seed test of `the_cell_flat.json` vs
`the_cell.json` (hierarchical) showed Δhits/min μ=+0.92, p=0.0495,
CI [+0.00, +1.73] — borderline-significant lift from the kinematic
mid-voter. Worth re-validating at the new Phase 6.5.16 baseline
where the substrate is producing 6× more hits.

---

## 8. Reproducing these results

Branch: `v4-phase6-5-1`. All commits have detailed messages.

### Build
```
cd cpp_core/build && cmake --build . -j
cd ../../godot_host/build && cmake --build . -j     # also rebuilds ogma_core
```

### Unit tests (196+ all pass)
```
cd cpp_core/build && ctest -j -R "Motor|Action|Clone|GNG\.|Neurochem|EPM|Lateral|Drive|Sequence|Homeokin|Phase3|HotPatch|ThinSlice"
```

### Cell smoke (10 seeds × 120s)
```
python3 scripts/cell_smoke.py --seeds 10 --start 42 --duration 120 --parallel 2
```

### MC continuous (seed 44, 20k ticks)
```
OGMA_SEED=44 OGMA_MC_CONTINUOUS=1 OGMA_MC_MAX_TICKS=20000 \
  OGMA_MC_CONFIG=res://addons/ami_ogma/configs/the_mountain_car.json \
  godot4 --path godot_host/project --headless res://scenes/the_mountain_car.tscn
```

### MC Gym v0 frozen-eval (seed 44)
```
OGMA_SEED=44 OGMA_MC_BENCHMARK=1 \
  OGMA_MC_TRAINING_EPISODES=60 OGMA_MC_BENCHMARK_EPISODES=100 \
  OGMA_MC_BENCHMARK_MAX_STEPS=200 \
  OGMA_MC_MAX_STEPS=1000 OGMA_MC_MAX_EPISODES=0 \
  godot4 --path godot_host/project --headless res://scenes/the_mountain_car.tscn
```

### CartPole Gym v1 frozen-eval (seed 42)
```
OGMA_SEED=42 OGMA_CP_BENCHMARK=1 \
  OGMA_CP_TRAINING_EPISODES=200 OGMA_CP_BENCHMARK_EPISODES=100 \
  OGMA_CP_BENCHMARK_MAX_STEPS=500 \
  OGMA_CARTPOLE_MAX_STEPS=500 OGMA_CARTPOLE_MAX_EPISODES=0 \
  godot4 --path godot_host/project --headless res://scenes/the_cartpole.tscn
```

### Derive a brain config from a manifest
```
python3 scripts/derive_brain_config.py configs/manifests/<env>.json --summary
python3 scripts/derive_brain_config.py configs/manifests/<env>.json \
  --output godot_host/project/addons/ami_ogma/configs/<env>_derived.json
```

### Audit a config against the derivation
```python
import json
live = json.load(open('godot_host/project/addons/ami_ogma/configs/the_<env>.json'))
derived = json.load(open('/path/to/derived.json'))
# diff modules + params (see scripts/cell_audit.py for the canonical pattern)
```

---

## 9. Glossary of internal references

| Phase tag | Subject | Doc |
|---|---|---|
| 6.5.0  | Paired-seed A/B harness | (in commit msgs) |
| 6.5.2  | CartPole bridge + V-table sign-bias | `docs/v4_phase6_5_3_action_layer_plan.md` §15 |
| 6.5.3  | Action-layer redesign (forward model, chunks) | `docs/v4_phase6_5_3_action_layer_plan.md` |
| 6.5.3.6+ | Body-manifest derivation framework | `docs/v4_brain_derivation.md` |
| 6.5.3.F | Terminal-pulse leakage fix | `docs/v4_phase6_5_3_action_layer_plan.md` §17 |
| 6.5.4  | Brain cloning system + frozen-eval | `docs/v4_phase6_5_4_clone_system.md` |
| 6.5.5–6.5.11 | Body-side reward design arc | `docs/v4_phase6_5_reward_design.md` |
| 6.5.12 | Hybrid chunk triggers + implicit miss | (commit `67ec107`) |
| 6.5.13 | Graph-panel firing indicator | (commit `da0b3fd`) |
| 6.5.14 | Copy-log button | (commit `846a18f`, `8bd9ded`, `73a18be`) |
| 6.5.15 | Cell config audit | (commit `dd27912`) |
| 6.5.16 | Cell body-side reward refinement | (commit `6c7a0d9`) |

---

## 10. Status as of 2026-05-04

What works:
- Three body bridges (MC, CartPole, Cell) at substrate-native baselines
- Body-bridge design pattern validated across all three
- Substrate-aligned reward template generalizing across all three
- Brain cloning + frozen-eval methodology producing literature-
  comparable numbers
- §17 terminal-pulse fix foundational; tests pass; CartPole non-
  regression confirmed
- 196+ unit tests pass; full-suite Cell A/B reproducible

What's pending:
- Quadruped bridge (Phase 6.6+)
- State-aliasing investigation in EPM
- Pre-dispatch prediction gating
- Adaptive parameter substrate (§16-class)
- Cell hierarchical voter re-validation at new baseline

What's falsified (open and honest):
- MC under Gym v0 protocol (cap chops trajectories — substrate-level)
- CartPole at Gym v1 threshold (policy-machinery limits, not reward)

The substrate is published with what doesn't work; falsifications
are first-class citizens in the methodology.
