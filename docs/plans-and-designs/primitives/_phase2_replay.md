# Cross-Cutting: Phase 2 Thin-Slice Replay Harness

**Applies to:** the Phase 2 acceptance gate for the Bus + Scheduler + first-five-modules integration (NeurochemState + EPM + LateralVoter + HomeostaticDrive + ActionDecoder).

---

## What Phase 2 Validates

Per `docs/v4_refactor.md` (Phase 2) and the planning resolution table, the Phase 2 gate is **integration soundness + competence sanity**, not behavioural parity with v3. The thin-slice runs the first five Phase-1 modules wired by `InProcessBus` against a recorded sensor input stream and asserts:

1. Replay completes without NaN, divergence, or crash.
2. `InProcessBus` dispatch ordering matches DAG levelization (validated by trace assertions).
3. Tick-barrier integrity holds under thread-pool stress at `num_cores` workers.
4. GNG node-count growth curves stay within v3's paired-seed variance band on the same input stream.
5. Hit rate ≥ 50% of v3 baseline on the matched seed (sanity check).

Open-loop replay: the recorded sensor stream is replayed verbatim regardless of the v4 brain's emitted actions. The v4 ActionDecoder still produces actions; those actions are recorded for diagnostic comparison but do NOT drive the (replayed) env. Closed-loop testing is Phase 5 (Godot Host).

---

## Golden Stream File Format

The recorded stream lives at `cpp_core/tests/golden/maze5x5_seed42.ogfs` (Ogma Golden Frame Stream). Versioned binary layout — once shipped this file is immutable per filename; format changes mint a new filename.

### Header (40 bytes, little-endian)

```
offset  size  field
------  ----  -------------------------------------------------------------
  0      4   magic           = "OGFS"   (ASCII)
  4      2   format_version  = 1        (uint16)
  6      2   reserved        = 0        (uint16)
  8      8   tick_count      = N        (uint64)
 16      4   image_width     = W        (uint32)
 20      4   image_height    = H        (uint32)
 24      4   image_channels  = C        (uint32)   3 for saliency RGB; 1 for grayscale
 28      4   proprio_dim     = P        (uint32)   22 for default maze schema
 32      4   master_seed     =          (uint32)
 36      4   header_crc32                          (over bytes 0..35)
```

### Per-tick record (variable size)

```
field                 size            notes
--------------------  --------------  -------------------------------------
tick_id               8 (uint64)      monotonic from 0
image_data            W*H*C bytes     uint8 saliency map; row-major
proprio_data          P * 4 bytes     float32 little-endian
event_count           1 (uint8)
events (event_count of):
    name_len          1 (uint8)
    name              name_len bytes  ASCII, no terminator
    intensity         4 (float32)
record_crc32          4 (uint32)      over the bytes from tick_id through last event
```

A typical 10000-tick maze run with W=160, H=120, C=3 produces a file ~580 MB if saliency is dense. Phase 2 tests use 2000-tick captures (~120 MB) — committed if reasonable, otherwise generated on demand by the capture script.

For very long captures the format MAY be extended in version 2 with PNG-compressed image frames; not Phase 0 scope.

---

## Capture Script

`scripts/capture_golden_frames.py`. Standalone — uses only `MazeEnv`, `MazeAdapter`, and a deterministic random-walk policy seeded from the master seed. **Does not** run the full v3 brain, so the resulting stream is "real input distribution from real maze geometry" rather than "what v3's brain saw."

That distinction matters for the Phase 2 multi-criteria gate (#4 — node-count growth curves vs. v3 paired-seed variance band): the comparison band is computed against a v3 run on the *same* random-walk policy capture, NOT against an arbitrary v3 hit-rate run. This keeps the comparison fair and avoids confounding "v3 brain made better decisions" with "v4 brain failed."

To capture a baseline:

```
conda run -n ami-ogma python scripts/capture_golden_frames.py \
    --seed 42 \
    --ticks 2000 \
    --output cpp_core/tests/golden/maze5x5_seed42.ogfs
```

The capture also emits `cpp_core/tests/golden/maze5x5_seed42.manifest.json` with a SHA-256 of the binary, the maze geometry summary, and the random-walk parameters — so future captures can be compared against the canonical one for regression tracking.

---

## C++ Loader (Phase 1 deliverable)

`cpp_core/include/ogma/golden/Replay.hpp` (ships in Phase 1):

```cpp
namespace ogma::golden {

struct Tick {
    uint64_t                tick_id;
    Eigen::MatrixXf         image;       // H × (W*C); host bridges to reality.video.saliency
    Eigen::VectorXf         proprio;     // P-dim
    std::vector<EnvEvent>   events;
};

class StreamReader {
public:
    explicit StreamReader(std::filesystem::path path);
    bool                next(Tick& out);   // false at EOF
    uint64_t            tick_count() const;
    std::string         file_sha256() const;
};

}  // namespace ogma::golden
```

The Phase 2 thin-slice harness drives the OgmaInstance with one `Tick` per call to `OgmaInstance::tick()`, bridging `image` → `reality.video.saliency`, `proprio` → `reality.proprio.*` topics, and `events` → `events.*` topics. The host adapter is also a Phase 1 deliverable.

---

## Multi-Criteria Phase 2 Gate (Detail)

Each criterion in the multi-criteria gate gets a concrete check in the harness:

### (1) No NaN / divergence / crash

Every published payload is inspected for NaN in `Eigen::VectorXf` fields and `inf` in scalar fields. Any NaN/inf fails the gate. Crash = exception escapes `OgmaInstance::tick()`.

### (2) DAG dispatch order

The harness wraps the InProcessBus with a tracer that records `(tick_id, level, module_id, topic, op)` where op ∈ {publish, subscribe-direct-fire, subscribe-feedback-fire}. After the run, asserts:

- For every Direct subscription: `op = subscribe-fire(tick t)` follows `op = publish(tick t)` in tick t's trace.
- For every Feedback subscription: `op = subscribe-fire(tick t)` reads tick t-1's last value (verified by `tick_id` field on the delivered payload).

### (3) Thread-pool stress

The harness runs the same input stream twice: once with a thread pool of size 1 (sequential), once with size `num_cores`. Asserts the two runs produce bitwise-identical published payloads when the master seed is the same. (Determinism per `_rng.md` requires this; a thread-safety bug shows up here.)

### (4) GNG node-count growth band

The same capture is replayed against (a) v3 Python via `run_e2e_v3.py --replay-frames cpp_core/tests/golden/maze5x5_seed42.ogfs` (a Phase-1 v3 capability we add), and (b) the v4 thin-slice. For each EPM, the per-1000-tick node-count curves are compared. v4's curve must stay within v3's paired-seed variance band (computed from `verify_determinism.py` over five v3 seeds). A 95% confidence interval on the v3 band is the threshold; v4 outside the band on more than 10% of the time-buckets is a failure.

### (5) Hit rate ≥ 50% of v3 baseline

Open-loop replay doesn't have a hit rate of its own — but the recorded events stream does (the v3 capture had hits). The test: count how many of the `events.hit` payloads in the recorded stream the v4 ActionDecoder is "satisfied with" (its action.out at the matching tick has the same sign as v3 would have produced). Threshold: ≥ 50% of the v3 baseline rate. This is intentionally loose; tighter checks are Phase 5's job.

If criteria (1)–(3) fail, the gate fails outright. If (4) or (5) fail, the gate stalls for diagnosis but may proceed if the cause is identified and tracked in `docs/v4_refactor.md`'s open-questions.

---

## Phase 0 Deliverable

This doc + the `scripts/capture_golden_frames.py` script. The actual `.ogfs` capture file is produced by running the script — it's NOT committed to git in Phase 0. (The file is large; Phase 1 decides whether to commit a 2000-tick "ci" capture or regenerate per-CI-run.)

The C++ loader (`Replay.hpp`) is a Phase 1 deliverable; no header is shipped now beyond what's already in the frozen Section B headers.
