# MotorRepertoire — Primitive Contract

**Phase 1 dependency position:** 9 of 9 (last in Phase 1).
**Header:** `cpp_core/include/ogma/modules/MotorRepertoire.hpp` (Phase 1 deliverable).
**Reference impl:** none — new primitive (`docs/v4_algorithmic_gaps.md` Primitive 5).

---

## Purpose

MotorRepertoire is the chunk library. v3 emits a scalar force per tick — every action is independent; there is no representation for multi-tick motor programs. MotorRepertoire fixes that: it observes the action stream, detects sub-sequences correlated with drive-error reduction, and crystallizes them as **chunks** with stable IDs that ActionDecoder can dispatch by name.

The primitive does not own action selection — that's still ActionDecoder's job. MotorRepertoire (a) maintains the library and (b) plays back the dispatched chunk tick-by-tick over a REQ/REP service.

Chunk extraction reuses **SequenceGNG over the action stream**: a separate `SequenceGNG` instance subscribed to `action.out` discovers recurring action motifs; MotorRepertoire snapshots those motifs and tags each with the drive-error trajectory observed during occurrence. When a motif's tagged drive-reduction is statistically positive over its lifetime, it crystallizes as a chunk.

---

## Input Topics

| Pattern | Kind | Payload | Producer | Required | Notes |
|---|---|---|---|---|---|
| `action.out` | Direct | `ActionOut` | ActionDecoder | yes | Observes the decoder's emitted actions to learn motifs. |
| `sequence.motif.action.out` | Direct | `SequenceMotif` | SequenceGNG (configured for action stream) | yes | Source of motif candidates for crystallization. |
| `drive.errors` | Direct | `DriveErrors` | HomeostaticDrive | yes | The outcome signal — drives the chunk's `outcome_drive_delta` tagging. |
| `motor.play.cmd` | Direct | `MotorPlayCmd` | ActionDecoder | yes (for playback) | REQ side: "play chunk_id starting now." |
| `events.hit`, `events.miss` | Direct | `EnvEvent` | host | no | Used as additional outcome signal alongside drive errors. |

---

## Output Topics

| Topic | Payload | Cadence |
|---|---|---|
| `motor.chunks` | `MotorChunks` | published on change (new bake / prune); also at end of episode |
| `motor.play.stream` | `MotorPlayStream` | per `motor.play.cmd` (REP) |

`MotorPlayStream.actions` is the tick-by-tick command list MotorRepertoire commits to. The **host** is responsible for emitting these on subsequent ticks via its `action.out` bridge — MotorRepertoire does NOT publish to `action.out` itself (that's ActionDecoder's topic; multiple producers per topic is forbidden by the Bus's invariants). Instead, ActionDecoder receives the stream and emits individual actions on subsequent ticks while suppressing its own per-tick policy until the chunk completes.

---

## Parameter Schema

| Key | Type | Mutability | Default | Range | Description |
|---|---|---|---|---|---|
| `max_chunks` | int64 | HotMutable | 256 | [10, 10000] | Library cap; LRU evict by `use_count` and `outcome_drive_delta`. |
| `chunk_max_ticks` | int64 | HotMutable | 20 | [2, 200] | Max chunk length; longer motifs are truncated. |
| `crystallization_min_observations` | int64 | HotMutable | 10 | [3, 1000] | Number of motif occurrences before crystallization is considered. |
| `crystallization_min_drive_delta` | double | HotMutable | 0.05 | [0, 1] | Mean `outcome_drive_delta` (negative = drive-reducing) required for crystallization. |
| `crystallization_drive_window_ticks` | int64 | HotMutable | 50 | [10, 1000] | Window after motif end over which to integrate drive change. |
| `interrupt_urgency_threshold` | double | HotMutable | 0.85 | [0, 1] | If `drive.errors.urgency` rises above this during chunk playback, abort the chunk and revert control to ActionDecoder. |
| `interrupt_outcome_divergence` | double | HotMutable | 0.30 | [0, 1] | If observed mid-chunk drive trajectory diverges from expected by more than this fraction, abort the chunk. |
| `master_seed` | int64 | ConstructionOnly | 0 | — | RNG namespace `motor_repertoire.<id>` for tie-break in eviction. |

---

## Invariants (per tick)

1. MotorRepertoire publishes `motor.chunks` only when the library state changes (new chunk crystallized, chunk evicted, chunk metadata updated by ≥ 1% change). Not every tick.
2. Each `MotorPlayCmd` produces exactly one `MotorPlayStream` response on the same tick or the next tick (synchronous within `max_concurrent_queries` budget; only one chunk plays per OgmaInstance at a time).
3. While a chunk is "playing" (after dispatch and before completion or interruption), MotorRepertoire publishes the remaining chunks list normally, but its internal "active chunk" state suppresses re-dispatch until the chunk completes.
4. Chunk crystallization is monotonic — once a chunk is in the library, its ID is stable; eviction does not reuse IDs.
5. `outcome_drive_delta` for each chunk is the mean of observed drive deltas across its `crystallization_drive_window_ticks` history; updated on every observation, never reset.
6. Motif → chunk promotion happens at the between-tick boundary; mid-tick crystallization is forbidden.

---

## Failure Modes

| Trigger | Behaviour |
|---|---|
| `MotorPlayCmd` for a `chunk_id` not in the library | Respond with `MotorPlayStream{actions: []}` and log warning. ActionDecoder must fall back to scalar action emission. |
| Chunk dispatched but interrupted mid-stream by `interrupt_urgency_threshold` | Truncate the response stream at the abort tick; remaining ticks revert to ActionDecoder's normal output. |
| Library at `max_chunks` capacity | Evict the lowest-(`outcome_drive_delta` × `use_count`) chunk before insertion. |
| `sequence.motif.action.out` missing | Library doesn't grow — chunks already crystallized still play back. Log info every 10⁴ ticks. |
| Crystallization triggered for a motif that's been pruned by SequenceGNG | Skip silently. |

---

## Latency Budget (Pi5 Cortex-A76)

- **Hot path (no playback active):** ≤ 100 µs. Receives action observation, possibly increments motif counters, no other work.
- **Crystallization tick:** ≤ 1 ms. Snapshots the motif and computes mean drive delta over the window.
- **Playback dispatch:** ≤ 200 µs. Looks up chunk and assembles the action stream.
- **Memory:** `max_chunks * (chunk_max_ticks * 4 + projection_dim * 4 + 16)` bytes — dominated by the per-chunk action sequences and entry-state prototypes.

---

## VV&A Criteria (Phase 1 acceptance)

### 1. Unit tests

- **Crystallization threshold:** with a synthetic motif source delivering 10 observations of the same motif at `outcome_drive_delta = -0.10`, exactly one chunk crystallizes after the 10th observation.
- **Below-threshold motif rejected:** observing the same motif 10 times with mean `outcome_drive_delta = -0.02` (below threshold) does not crystallize.
- **Playback determinism:** dispatching the same chunk twice produces bit-identical `MotorPlayStream.actions`.
- **Interrupt:** during chunk playback, raising `drive.errors.urgency` above the threshold truncates the stream at the next tick.
- **LRU eviction:** with `max_chunks = 8`, crystallizing a 9th chunk evicts the lowest-product entry.
- **Chunk-ID stability:** a crystallized chunk's ID does not change across library updates; eviction does not reuse IDs.

### 2. Pair tests

- **`pair_motorrepertoire_actiondecoder`**: ActionDecoder issues `motor.play.cmd`; MotorRepertoire returns a stream; ActionDecoder consumes and emits the stream tick-by-tick on `action.out`. Verifies the chunk-dispatch round-trip.
- **`pair_motorrepertoire_seqgng`**: SequenceGNG produces motifs; MotorRepertoire crystallizes them when conditions are met. Integration of the motif-source seam.

### 3. Latency

`tick()` p99 ≤ 100 µs (no playback) or ≤ 200 µs (playback dispatch) on Pi5 over 10⁴ runs.

### 4. Determinism

Same `master_seed` + same input streams → same library + same playback streams.

### 5. Behavioural target

When integrated into a Phase-3 maze benchmark, the agent with MotorRepertoire crystallizes ≥ 4 chunks within 10000 ticks, and chunk-driven action selection accounts for ≥ 25% of emitted actions in the second half of the run.

---

## Notes

- **Reuses SequenceGNG.** Chunks are extracted from `sequence.motif.action.out`. There is no separate motif extractor inside MotorRepertoire — that would duplicate code and split the maintenance burden.
- **ActionDecoder owns playback emission.** This is a deliberate choice to keep `action.out` to a single producer. MotorRepertoire dispatches chunks; ActionDecoder is the one module that publishes `action.out`. Multiple-producer topics are forbidden by the Bus invariants.
- **Cerebellum is out of scope.** A forward motor model (cerebellar analog) for online correction is mentioned in `v4_algorithmic_gaps.md` as a Phase 6+ extension. Phase 1's MotorRepertoire is open-loop chunk dispatch with urgency / divergence interruption.
