# GNGRollout — Primitive Contract

**Phase 1 dependency position:** 8 of 9.
**Header:** `cpp_core/include/ogma/modules/GNGRollout.hpp` (Phase 1 deliverable).
**Reference impl:** none — new primitive (`docs/v4_algorithmic_gaps.md` Primitive 3). Replaces v3's count-table argmax-chained "EFE" rollout.

---

## Purpose

GNGRollout simulates futures. Given a current state and a candidate action, it samples K trajectories M steps forward by **weighted edge traversal** over a GNG topology (treating edge ages/strengths as transition probabilities) and returns an empirical distribution over terminal states with their values.

Why a separate module: v3 chained argmaxes over a frequency-based transition table, which amplifies noise as horizon grows. v3's experimental observation: `horizon > 1` makes performance worse. GNGRollout's stochastic samples produce a real uncertainty estimate (`entropy` over the K trajectories) and a distribution-over-outcomes that ActionDecoder's EFE policy can use principled-ly.

The module exposes a REQ/REP service rather than a periodic publish. Callers (typically ActionDecoder) issue `RolloutQuery` messages tagged with a `request_id`; GNGRollout publishes `RolloutResult` carrying the same `request_id`. ActionDecoder correlates by ID. Multiple in-flight queries are allowed.

When a SequenceGNG is wired into the same OgmaInstance and the rollout's current state matches a baked motif, the rollout teleports to the motif's terminal — collapsing M ticks into one lookup. This is the planning-via-chunks mechanism.

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `rollout.query` | Direct | `RolloutQuery` | ActionDecoder (typically) | yes | The REQ side. `request_id` correlates with the response. |
| `reality.<group>.<modality>` | Direct | `RealityToken` | every EPM whose topology is queryable | yes (≥1) | GNGRollout caches each EPM's GNG node-and-edge state to do its own traversal. (It does NOT query the EPM each rollout — that would be a live coupling and a performance disaster.) |
| `sequence.motif.` (prefix) | Direct | `SequenceMotif` | every SequenceGNG | no | When present, enables motif-teleport. |
| `drive.errors` | Direct | `DriveErrors` | HomeostaticDrive | yes | Used to compute `terminal_values[k]` as expected drive-error reduction at the trajectory's end. |

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `rollout.result` | `RolloutResult` | per query (REP) |

`RolloutResult` carries `trajectories: [K][M] node IDs`, `terminal_values: [K] expected drive reduction`, `entropy: float`. `request_id` matches the requesting query.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `K_default` | int64 | HotMutable | 32 | [1, 1024] | Default number of trajectories sampled if the query doesn't specify. |
| `M_default` | int64 | HotMutable | 5 | [1, 32] | Default forward horizon. |
| `transition_smoothing` | double | HotMutable | 0.01 | [0, 1] | Laplace smoothing applied to per-edge transition probabilities (so unvisited edges have small but non-zero probability). |
| `motif_teleport_enabled` | bool | HotMutable | true | — | If true and a baked motif matches, jump to the motif terminal. |
| `motif_teleport_min_confidence` | double | HotMutable | 0.5 | [0, 1] | Minimum match_confidence required to teleport. |
| `value_window_ticks` | int64 | HotMutable | 100 | [10, 10000] | Window over which `terminal_values` is regressed against actual outcomes for self-calibration (Phase 3 stretch — Phase 1 just emits the raw drive-reduction estimate). |
| `max_concurrent_queries` | int64 | HotMutable | 4 | [1, 64] | Cap to bound per-tick worst-case latency. Excess queries are rejected with `request_id` echoed and an empty result. |
| `master_seed` | int64 | ConstructionOnly | 0 | — | RNG namespace `rollout.<id>` for trajectory sampling. |

---

## Invariants (per tick)

1. Each query received in tick t produces exactly one response in tick t (synchronous service; not deferred to a later tick).
2. `RolloutResult.request_id == RolloutQuery.request_id` always.
3. `trajectories.size() == K_used` and every `trajectories[k].size() == M_used`. K_used and M_used are min(query.K, K_default cap) and min(query.M, M_default cap).
4. `entropy ∈ [0, log(K_used)]`. Computed over the distribution of terminal node IDs.
5. Transition probabilities at every traversal step sum to 1.0 (Laplace-smoothed).
6. When `motif_teleport_enabled = true` and the seed state matches a baked motif at phase 0 with confidence ≥ threshold, exactly one of the K trajectories takes the teleport path; the other K-1 follow stochastic edge traversal. This balances "try the chunked plan" with "explore alternatives."
7. The internal GNG-state cache is updated each tick from incoming `RealityToken` payloads; the cache is read-only during rollout sampling — sampling never sees a partial mid-tick update.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| Query for a `source_modality` whose EPM hasn't published yet | Return empty `trajectories`, `terminal_values = []`, `entropy = 0`. ActionDecoder treats as "no information." |
| `winner_id` in query doesn't exist in the cached GNG | Same as above (empty result). Log warning. |
| `K * M > max_concurrent_queries * K_default * M_default` over a single tick | Reject excess queries with empty results; do not block. |
| Motif teleport with `phase ≠ 0` | Sample without teleport for that trajectory. (Mid-motif teleport is not supported in Phase 1.) |
| `drive.errors` not yet published | Return rollout with `terminal_values` filled with zeros. |

---

## Latency Budget (Pi5 Cortex-A76)

- **Per-query at K=32, M=5:** ≤ 1 ms. Each step is `O(neighbors_per_node)` PRNG draws plus prototype distance comparison.
- **Per-query with motif teleport on a matched chunk:** ≤ 200 µs (one trajectory teleports, K-1 still sampled).
- **Per-tick cache update:** ≤ 200 µs across all subscribed EPMs.
- Memory: dominated by the cached GNG topology (~`max_nodes * projection_dim * 4` per EPM cached).

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Single-step rollout determinism:** with a fixed GNG topology and `master_seed`, a query at the same node + action produces the same K trajectories across two runs.
- **Probability normalization:** sum of transition probs out of every node equals 1.0 (within `1e-6`) for arbitrary topologies.
- **Entropy bounds:** for a fully connected k-node graph with uniform transitions, entropy of K terminal states approaches log(min(K, k)) as M increases.
- **Motif teleport:** with a baked motif starting at the seed node, exactly one trajectory ends at the motif's terminal in one step; others follow the stochastic path.
- **Empty-result safety:** queries for nonexistent winners produce well-formed empty `RolloutResult` payloads, not exceptions.
- **`max_concurrent_queries` cap:** the (max+1)-th query in a tick gets an empty response.

### 2. Pair tests

- **`pair_gngrollout_actiondecoder`**: ActionDecoder issues a query under EFE fallback; GNGRollout returns a result; ActionDecoder uses the result's entropy for the epistemic term. Verifies REQ/REP correlation.

### 3. Latency

`tick()` p99 ≤ 1 ms with K=32, M=5 on Pi5 over 10⁴ runs.

### 4. Determinism

Same `master_seed` + same incoming GNG-state stream + same query schedule → bitwise-identical `RolloutResult` payloads.

### 5. Behavioural target

When integrated into the maze benchmark (Phase 3+), ActionDecoder using GNGRollout's entropy outperforms ActionDecoder using the count-table proxy by ≥ 5% hit rate over 10000 ticks, with horizon `M=5`. This is the contract's claim that GNGRollout actually adds signal where v3 added only noise. (Phase 3 acceptance criterion; Phase 1 just verifies the module ships and the seam works.)

---

## Notes

- **Cache, don't query.** The single most important implementation rule: GNGRollout caches each EPM's topology incrementally from `RealityToken` payloads (using `node_count`, `pruned_ids`, and the implicit "winner_prototype" updates). It never reaches into another module's state. The cache is the GNGRollout's own copy.
- **Sampling, not argmax.** Phase 1 uses pure stochastic edge traversal weighted by transition probability. v3's argmax-chained rollout is what we're explicitly fixing. If a Phase 3 experiment shows a need for hybrid stochastic-argmax, it goes through a contract amendment.
- **Motif teleport collapses planning.** A 5-step motif's terminal is reached in one trajectory step when teleport fires. This is the planning-by-chunks mechanism; without SequenceGNG or with motif teleport disabled, GNGRollout falls back to pure step-by-step sampling.
