# Module Primitives — Overview & Taxonomy

_Last updated: 2026-05-23, after the Phase 7.2-EPM hierarchical experiment closed._
_Companion to `docs/phase7_arc_findings.md` (the substrate-architecture lesson)
and `~/.claude/projects/-home-xaqmusic-ami-ogma/memory/project_walking_not_emergent_from_perception.md`._

This doc is the single map of every Module subclass in the substrate, what
it does, what category it belongs to, and what's currently load-bearing vs
dormant vs stubbed.  Read this before adding a new module, deleting an old
one, or designing a new architectural lever.

## How to use this doc

- **Adding a new module?** Find the closest existing category, copy the
  doc template from any existing `docs/primitives/*.md`, and follow
  `docs/primitives/_module_lifecycle.md`.
- **Picking a module to extend?** Use the "Status" column to see what's
  hot-path vs cold-path.  Cold-path modules are fair game to repurpose
  or delete.
- **Debugging a config?** The "Active picrawler configs" column tells
  you which modules a tabula-rasa picrawler run will instantiate.
- **Looking for an architectural gap?** §6 lists known gaps the substrate
  doesn't yet have a primitive for — see also `docs/action_side_plan.md`.

## 1.  Taxonomy at a glance

The substrate has **32 module classes** (`cpp_core/include/ogma/modules/*.hpp`).
They cluster into seven functional categories:

| Category | Role in the brain |
|---|---|
| **Perception** | Encode incoming sensation into discrete winner_id + latent. |
| **Lateral integration** | Fuse multiple perception streams into a single consensus. |
| **Top-down prediction** | Predict expected sensation from consensus, subtract from perception. |
| **Drive / neuromodulation** | Maintain affective state (DA / 5-HT) and homeostatic urgency. |
| **Sequence / motor abstraction** | Cluster temporal patterns in winners or actions into addressable motifs / chunks. |
| **Policy / action selection** | Convert consensus state into motor commands. |
| **Action modulation / reflex** | Modify or override motor commands at sub-policy level. |

A handful of modules also serve as **utility / signal processing**
primitives that don't fit cleanly above.

## 2.  Module catalog

### Status legend
- 🟢 **Hot path** — in every active picrawler config; load-bearing.
- 🟡 **Conditional** — in some configs (chunks pipeline, Cell env, etc.).
- 🔵 **Stubbed / dormant** — shipped but no consumer currently uses it.
- 🔴 **Tested negative** — empirically shown to harm; should not ship by default.
- ⚪ **Cell-only** — for the older Cell env, not active in v6 picrawler.

### Perception

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **EPM** (Episodic Predictive Module) | 🟢 | [EPM.md](EPM.md) | Frozen encoder + GNG cluster + dual TLE; the per-modality "winner" producer. |
| **JointSensorimotorBridge** | 🟡 | — *missing* | Joins per-joint action with proprio bundle into per-joint sensorimotor `ProprioToken`. Used by Phase 7.2-EPM hierarchical configs. |
| **KeyframeAverager** | 🔵 | [KeyframeAverager.md](KeyframeAverager.md) | Averages slow-keyframe RealityTokens; used by entry-context history in chunks. |
| **GNGRollout** | 🔵 | [GNGRollout.md](GNGRollout.md) | Forward-rollout helper for predictive entropy / planning. |
| **EmbeddingRegistry** | 🔵 | — *missing* | Holds shared embeddings (Phase 6.6.E); no live consumer yet. |
| **EpisodicCapture** | 🟡 | — *missing* | Captures episodic chunks from reward events. Currently silent (chunks blocked at consensus-baking, see CPG ablation findings). |

### Lateral integration

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **LateralVoter** | 🟢 | [LateralVoter.md](LateralVoter.md) | Trust-weighted fusion of multiple `RealityToken` streams into one `ConsensusToken` on `consensus.<level>`.  Supports hierarchical level=0/1 patterns + per-modality input excludes. |

### Top-down prediction

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **DescendingPredictor** | 🟢 | [DescendingPredictor.md](DescendingPredictor.md) | Learns to predict each EPM's input from current consensus; publishes `prediction.<modality>` which the EPM subtracts before encoding. The downward arc of predictive coding. |
| **StaleConfidenceDecay** | 🔵 | — *missing* | Bleeds confidence when reality stops moving; signal-processing utility. |

### Drive / neuromodulation

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **NeurochemState** | 🟢 | [NeurochemState.md](NeurochemState.md) | Tracks dopamine + serotonin level + their adaptive baselines.  Drives `reward_signal` (`dopamine − baseline`). |
| **HomeostaticDrive** | 🟢 | [HomeostaticDrive.md](HomeostaticDrive.md) | Maintains drive-level urgencies (alive, food, etc.) that feed into chunk selection + exploration arming. |
| **HomeokineticExploration** | 🟡 | [HomeokineticExploration.md](HomeokineticExploration.md) | Long-window TLE-change detector that arms explore-overrides on Premotors when learning stalls. |

### Sequence / motor abstraction

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **SequenceGNG** | 🟡 | [SequenceGNG.md](SequenceGNG.md) | n-gram clusterer over consensus motifs or action streams; publishes `sequence.motif.*`. |
| **MotorRepertoire** | 🟡 | [MotorRepertoire.md](MotorRepertoire.md) | Stores baked motor chunks; serves them on `motor.chunks` for the ActionDecoder. |
| **EpisodicCapture** | 🟡 | — *missing* (see Perception row) | Crystallises chunks from reward-aligned trajectories. |

### Policy / action selection

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **Premotor** | 🟢 | — **MISSING — see §5 below** | Per-channel REINFORCE policy with Hebbian state→intent associations.  Reads `consensus.0`, samples one of n_intents, publishes `ActionOut`. **The core action primitive.** |
| **ActionDecoder** | 🟡 | [ActionDecoder.md](ActionDecoder.md) | Chunk-replay dispatcher.  Consumes `MotorPlayCmd` requests, plays back baked chunks via `ActionOut`. |
| **PolicyChannelAggregator** | 🟡 | — *missing* | Per-channel radix aggregator for multi-channel chunk replays (Phase 6.18). |

### Action modulation / reflex

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **CPGOscillator** | 🔴 | — *missing* (`docs/phase7_cpg_status.md` covers it) | Spinal Central Pattern Generator; phase-rotated sinusoidal bias per joint.  Tested net-negative in `docs/phase7_cpg_ablation_findings.md`; amplitudes zeroed by default in chunks_compass_cpg config. |
| **ActionGate** | 🔵 | — *missing* | BG-analog go/no-go gate on Premotor outputs (Phase 6.5.26).  Tested null on Cell; not in picrawler configs. |
| **MotorFader** | 🔵 | — *missing* | Crossfades brain action with reflex action (Phase 6.6.F-G).  Cell-era; not in picrawler. |
| **FaderController** | 🔵 | — *missing* | Drives MotorFader from a directive topic. |
| **ChunkAbortGate** | 🔵 | — *missing* | Aborts running chunks on outcome divergence (`docs/phase7_chunk_plan.md` Rung 7.4).  Wired but never fires under current config. |
| **ChunkOutcomeGate** | 🔵 | — *missing* | Post-chunk hit-rate gate. |

### Reflexes (Cell-era, ⚪ all)

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **CellReflex** | ⚪ | — *missing* | Cell-env hand-coded reflex (food approach, wall avoidance). |
| **ForwardDriveReflex** | ⚪ | — *missing* | Forward-drive bias when food drive is high. |
| **ScentGateReflex** | ⚪ | — *missing* | Whisker-gated reflex modulation. |
| **StuckEscapeReflex** | ⚪ | — *missing* | Random-direction kick when chassis is stuck. |
| **WhiskerAversionReflex** | ⚪ | — *missing* | Aversive bias when whisker contact spikes. |
| **WhiskerSteerReflex** | ⚪ | — *missing* | Steering bias from whisker contact gradient. |

### Utility / signal processing

| Module | Status | Doc | One-line role |
|--------|--------|------|---|
| **AdaptiveThresholdTracker** | 🔵 | — *missing* | Maintains adaptive threshold on a topic (used by escape detectors). |
| **DualEMADetector** | 🔵 | — *missing* | Two-timescale EMA divergence detector (used in some Cell metrics). |
| **EventConjunction** | 🔵 | — *missing* | Boolean conjunction of two events for chunk-firing pre-conditions. |

## 3.  Counts

| Category | Total | Hot | Conditional | Dormant | Negative | Cell-only |
|---|---|---|---|---|---|---|
| Perception | 6 | 1 | 2 | 3 | 0 | 0 |
| Lateral integration | 1 | 1 | 0 | 0 | 0 | 0 |
| Top-down prediction | 2 | 1 | 0 | 1 | 0 | 0 |
| Drive / neuromodulation | 3 | 2 | 1 | 0 | 0 | 0 |
| Sequence / motor abstraction | 3 | 0 | 3 | 0 | 0 | 0 |
| Policy / action selection | 3 | 1 | 2 | 0 | 0 | 0 |
| Action modulation / reflex | 6 | 0 | 0 | 4 | 1 | 1 |
| Reflexes (Cell) | 6 | 0 | 0 | 0 | 0 | 6 |
| Utility | 3 | 0 | 0 | 3 | 0 | 0 |
| **Total** | **33** | **6** | **8** | **11** | **1** | **7** |

(The total of 33 here vs 32 headers includes EpisodicCapture once though it
spans two categories — Perception and Sequence/abstraction.)

**Observation**: **6 modules carry every picrawler tabula-rasa run.**
Most of the surface area (~75 %) is conditional, dormant, or Cell-only.
The picrawler standing baseline is much leaner than the module catalog
implies.

## 4.  Documentation gap

15 modules have a primitive doc in `docs/primitives/`.  **18 do not.**

**Highest-priority undocumented modules** (in current load-bearing order):

1. **Premotor** — the load-bearing action primitive.  See §5 below for
   a stub-spec until a full doc lands.
2. **JointSensorimotorBridge** — used by Phase 7.2-EPM hier configs and
   chunks_compass_cpg.  Two phase-7 documents reference it informally.
3. **CPGOscillator** — tested net-negative; `docs/phase7_cpg_status.md`
   covers the architecture, but no formal primitive doc.
4. **PolicyChannelAggregator** — chunks-pipeline-only, but blocks chunk
   debugging if undocumented.
5. **EpisodicCapture** — chunks-pipeline-only; blocks chunk debugging.
6. **ChunkAbortGate / ChunkOutcomeGate** — chunks-pipeline gates.

The 6 Cell reflexes and the dormant `Action*Gate` / `Motor*` modules can
wait — they're not on the v6 critical path.

## 5.  Premotor — stub doc until a full one lands

A minimum spec until someone writes `docs/primitives/Premotor.md`:

**Purpose**: convert each tick's `ConsensusToken.active_winner_id` (state)
into an `ActionOut.accel` (motor command), with REINFORCE policy learning
and Hebbian state→intent associations.

**Topology** (per Premotor):
- Subscribes to `consensus.<level>` (default level=0) → `state_node` per tick.
- Holds an n_intents × pd-dim weight matrix `W` (Hebbian-updated by `apply_reward`).
- Each intent has a fixed `intent_accels[i]` value (bilateral: also `intent_accels_right[i]`).
- Per tick: compute `softmax(W·latent + b)`; sample chosen intent (or argmax);
  publish `ActionOut(accel = intent_accels[chosen])` on `action_output_topic`.

**Learning signals**:
- **Hebbian credit**: when an `events.hit` arrives, `apply_reward(intensity)`
  strengthens `W[chosen_intent][state_node]` (the just-acted state → just-chosen
  intent link).
- **REINFORCE (MC episode)**: at `mc_episode_period` ticks, accumulated
  trajectory rewards drive a policy-gradient update with baseline
  subtraction.

**Override paths**:
- `intent_override_topic` (chunks) — replaces softmax sample for one tick.
- `explore_directive_topic` (HomeokineticExploration) — replaces softmax
  with the intent nearest a directive's accel value.

**Diagnostics**:
- `W_total_norm` (= `pre_w_growth` in JSONL) — the load-bearing learning
  signal we've used across every Phase 7 experiment.
- `last_entropy` (= `pre_H`) — softmax entropy over intents.
- `chosen_intent_counts` — visit histogram per intent.
- `total_explore_overrides_used`, `last_explore_active` — explore counters.

**Key architectural gap (the next experiment's target)**:
Premotors are **independent per channel**.  No mechanism couples one
Premotor's choice to another's.  Multi-joint motor primitives (gait,
synergies) cannot emerge from the current architecture.  See
`docs/action_side_plan.md` for the Hebbian co-activation design that
addresses this.

## 6.  Known architectural gaps

Things the substrate does NOT have a primitive for, that biology / motor-control
literature suggests it should:

| Gap | Biological analog | Why it matters | Proposed mechanism |
|---|---|---|---|
| **Cross-Premotor coupling** | Spinal motor synergies (d'Avella) | Cannot express multi-joint primitives | Hebbian co-activation matrix (see action_side_plan.md) |
| **Inverse motor model** | Cerebellum inverse model (Kawato) | Cannot compute "action → desired state" | Future: pair with forward model from EPM/predicted_pathway |
| **Spike-level temporal coding** | Cortical / spinal STDP | No fine timing relationships between Premotors | Could be retrofit onto Hebbian co-activation as STDP-style |
| **Working memory / context buffer** | Prefrontal sustained activity | Cannot maintain goal across many ticks | KeyframeAverager + entry_history_ partially solve, undermotivated currently |
| **Goal representation** | Prefrontal / parietal goal cells | No structured goal beyond drive urgencies | Future: Level-1 EPM on goal_topic |
| **Action gating by drive / value** | Basal ganglia direct/indirect pathways | ActionGate exists but tested null — undermotivated | Re-examine after action-coordination problem is solved |
| **Predictive dispatch / motor imagery** | Cerebellum forward model | Chunks fire reactively (current state match), not predictively | Future: walk EPM's `predicted_pathway` ahead, dispatch chunks for predicted state |

## 7.  Related docs

- `docs/primitives/_module_lifecycle.md` — authoring contract for new modules.
- `docs/primitives/_first_tick.md` — first-tick / feedback semantics.
- `docs/primitives/_hot_patch.md` — patch-mode (add/remove/connect) API.
- `docs/primitives/_aux_send_routing.md` — per-primitive subscription gate.
- `docs/primitives/_rng.md` — RNG derivation conventions.
- `docs/primitives/_pair_tests.md` — pair-test guidance.
- `docs/primitives/_phase2_replay.md` — Phase 2 replay (Dreamer mode hooks).
- `docs/phase7_arc_findings.md` — the substrate-architecture arc lesson
  this overview was written alongside.
- `docs/action_side_plan.md` — next architectural lever (Hebbian
  co-activation in Premotor outputs).
