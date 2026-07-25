> **Ported from the pre-split `ami-ogma` repo, 2026-07-25.** ⚠️ The **CurriculumManager /
> TrainerPanel machinery described here is from the reward-shaped RL era** that
> [`../the-picrawler-detour.md`](../the-picrawler-detour.md) disowns, and the current
> active-inference stack does not use it. **Read it for the contamination discipline, which is
> permanently relevant**: how a single careless launch silently invalidates weeks of paired-seed
> results. That failure mode recurred after the split (a harness flag that never got set), and is
> now check 7 in [`../../CLAUDE.md`](../../CLAUDE.md) §3.2. `ami_ogma`/`ogma` == xaq.

# PiCrawler Curriculum + Trainer Protocol

This doc defines how the **CurriculumManager** (stage-based goal switching)
and the **TrainerPanel** (live DA / 5-HT pulse injection) interact with
the step-wise A/B methodology.  The substrate is sensitive to silent
contamination — the rules here exist so a single careless launch
doesn't invalidate weeks of paired-seed results.

## 1.  Two systems, kept separate

| System | What it changes | Who fires it | Logged as |
|--------|-----------------|--------------|-----------|
| **CurriculumManager** | The body's reward-knob @export vars (target_height, walk_hit_rate, …) at stage transitions. | Manual: UI buttons / `[` `]` keys.  Auto: `advance_when` rule triggered by body metrics. | `curr_idx`, `curr_name` in each diag JSONL line. |
| **TrainerPanel** | Fires `events.hit` / `events.miss` directly to the brain via `body.publish_trainer_event()`. | Manual only: Good Boy / Bad Boy buttons or `G` / `B` keys. | `trainer_good_count`, `trainer_bad_count` cumulative counters in every diag line. |

They never share state.  A run can use neither, just curriculum, just
trainer, or both — every combination is captured in the JSONL audit
trail.

## 2.  When each is appropriate

### Curriculum — appropriate for
- Multi-stage live demos: "watch the brain learn to stand, then I press
  `]` and it learns to walk".
- Reproducible curricula in headless: `--curriculum FILE` runs each
  stage according to the file's `advance_when` rules.
- Saving brain checkpoints at stage boundaries (future work).

### Curriculum — NOT appropriate for
- Controlled A/B experiments where the *only* difference between arms
  should be the mechanism under test.  If arm A has curriculum and arm B
  doesn't, the reward signal trajectory differs, so any observed Δ
  conflates curriculum effect with mechanism effect.  Rule: **both
  arms must use the same curriculum file** *or* **both arms must use
  none.**

### Trainer — appropriate for
- Live demos with a human in the loop.  E.g., physical PiCrawler eventually:
  experimenter watches behavior, presses `G` when the robot does
  something desirable.
- Operant-conditioning explorations on top of an already-learned skill.

### Trainer — NEVER appropriate for
- Any headless A/B.  Trainer pulses are recorded in the per-tick JSONL
  as `trainer_good_count` / `trainer_bad_count`; the harness aggregator
  marks `ok=False` and stamps `trainer_contaminated: true` if either
  counter is non-zero.  Such runs are excluded from arm-level stats.
- Comparing checkpoints saved with vs without trainer history — the
  weights carry the trainer pulses forward and are no longer
  "tabula-rasa + reward shaping only" runs.

## 3.  Guarantees the code enforces

1. **Single entry point**: trainer pulses must route through
   `picrawler_body.publish_trainer_event()`.  There is no other code
   path that fires `events.hit` / `events.miss` with origin "trainer".
2. **Audit columns**: every diag JSONL line carries
   `trainer_good_count`, `trainer_bad_count`, `curr_idx`, `curr_name`.
   These are emitted even when zero / empty so analysis scripts can
   assert column shape.
3. **Headless contamination check**: `picrawler_run.py` sets
   `ok=False` and `trainer_contaminated=true` on any run where either
   trainer counter exceeded 0.
4. **Whitelist on curriculum overrides**: `_CURRICULUM_ALLOWED_KEYS`
   in `picrawler_body.gd` enumerates the exact @export params a
   curriculum file can override.  Anything else emits a warning and is
   dropped.  Curricula are data, not code — they cannot mutate body
   internals like `_chassis` or `tick_counter`.
5. **Determinism**: A vs A self-check with the same curriculum file on
   both arms returns Δ=0 across every metric (verified `picrawler_ab.py
   --self-check` with `OGMA_PICRAWLER_CURRICULUM`).
6. **Manual default**: `CurriculumManager.auto_advance` is `false` until
   explicitly toggled (UI button, `Shift+]`, or
   `OGMA_PICRAWLER_CURRICULUM_AUTO_ADVANCE=1`).

## 4.  Recommended experiment patterns

### Single-stage A/B (existing methodology, unchanged)
```bash
python scripts/picrawler_ab.py --seeds 42-61 --duration 200 \
  --baseline-config <stand_capped.json> \
  --variant-config  <stand_capped.json> \
  --variant-env "OGMA_PICRAWLER_TARGET_HEIGHT=0.085" \
  --turbo --parallel 6
```
No curriculum, no trainer.  This is the v5-frozen methodology.

### Curriculum A/B: comparing two curricula
```bash
# Compare curriculum A's stage advance schedule vs curriculum B's
python scripts/picrawler_ab.py --seeds 42-61 --duration 600 \
  --baseline-config <same.json> --variant-config <same.json> \
  --baseline-env "OGMA_PICRAWLER_CURRICULUM=res://curricula/A.json,OGMA_PICRAWLER_CURRICULUM_AUTO_ADVANCE=1" \
  --variant-env  "OGMA_PICRAWLER_CURRICULUM=res://curricula/B.json,OGMA_PICRAWLER_CURRICULUM_AUTO_ADVANCE=1"
```
Both arms must have a curriculum (or both must not).  Auto-advance
ensures the same trigger conditions fire identically across seeds.

### Curriculum smoke-test: same curriculum, manual stage skipping
```bash
# Inspect what happens when stage 3 is the start point — does the brain
# learn walking faster if standing is already in the weights?  This
# REQUIRES brain-checkpoint plumbing (deferred) to be meaningful.
python scripts/picrawler_run.py --seeds 42-46 --duration 300 \
  --curriculum res://curricula/picrawler_stand_then_walk.json \
  --start-stage 3
```
This is exploratory, not an A/B.

## 5.  Failure modes to watch for

| Symptom | Likely cause |
|---------|--------------|
| `trainer_contaminated=true` in a headless summary | Someone left a Good Boy keypress hooked, or a script called `publish_trainer_event()` programmatically.  Re-run; check `trainer_good_count` field for which kind fired. |
| Two arms of an A/B diverge huge | Verify both arms have the SAME `OGMA_PICRAWLER_CURRICULUM` env var.  A typo on one side will silently flip the reward signal. |
| Curriculum override silently dropped | Param name not in `_CURRICULUM_ALLOWED_KEYS`.  Check the warning log line (`CurriculumManager: 'X' is not in the curriculum whitelist`). |
| Auto-advance fires immediately | The `advance_when.metric` is already satisfied at tick 0 (e.g. `best_cumulative_alive_ticks >= 0`).  Check the rule's threshold value. |

## 6.  Open issues / future work

- Brain-state checkpoints save/load — needed before `--start-stage` is
  genuinely useful (currently it just starts the brain tabula-rasa with
  stage 3's reward, which won't work for the walking stages).
- Stage-aware aggregator: surface `curriculum_final_stage_idx` and
  `n_transitions` in the per-seed JSON so cross-stage analysis is
  one-line.
- Trainer event tagging in the brain's own event stream — currently
  trainer pulses look identical to body-emitted events.hit at the
  brain layer.  A `trainer_origin: true` flag in the consensus token
  would let the brain treat trainer reinforcement differently (e.g.,
  stronger eligibility-trace decay).
