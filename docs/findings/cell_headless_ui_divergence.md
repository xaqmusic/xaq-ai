# Cell — headless↔UI divergence (RESUME HERE next session)

> **✅ ROOT-CAUSED 2026-06-17 — the render/float hypothesis in this doc is FALSIFIED.** It was never
> float nondeterminism; it's a **config-resolution hole**. `reflex_modular` (and the motor flags) have
> **no headless metadata fallback** — `resolve_reflex_modular()` returns true only for `launched &&
> metadata` (UI) *or* `OGMA_REFLEX_MODULAR=1`. So headless runs (no env var) silently resolved
> `reflex_modular=FALSE` and ran the **old body-reflex forager** (which forages); the UI runs
> `reflex_modular=TRUE` = the **actual cognitive bug**, which routes steering through the brain and has
> **no wall-avoidance + ineffective navigation** (the absent `WhiskerSteerReflex` / never-built
> `StuckSteerReflex`). **Fully reproducible headless via `OGMA_REFLEX_MODULAR=1`** (seed 5 → 0 hits,
> drives into the corner). xvfb-rendered with the render loop ON *forages* → render loop was never the
> cause. The "headless forager" referenced below was never the cognitive bug. Fixes landed (boredom→nav
> gate, `None` ablation default, spinal wedge-escape) keep it alive but it still doesn't forage —
> remaining = the **bar-(b) cognitive-navigation** work. Full thread + next steps in the memory
> `v6-reasoning-breakthrough-resume`. Everything below is the (mistaken) original characterization.

**Date:** 2026-06-16 · **Branch:** gait-ignition-homeokinetic · **Latest commit:** 3a52d98 (+ food
non-blocking 47beae6, runtime marker 3a52d98)
**Status:** The "stuck bug" mystery is characterized but **not closed**. Headless forages robustly;
the UI consistently fails; the gap is a systematic runtime difference, root cause still open.

---

## The one-paragraph state

The cell-cognitive bug (config `the_cell_cognitive.json`, body `differential_paddler`) **forages
robustly in HEADLESS** — n=5 seeds (5/42/7/13/99) all explore the full arena (90% coverage, low
boredom 0.03–0.09) and eat **5–13 times** in ~134 s. But in the **UI it consistently fails** — 0
hits, wall-follows a corner, boredom ~0.7 (every operator snapshot). Same config, seed, body,
params. So it is **not noise, not chaos, not within-runtime non-determinism** — each runtime is
internally deterministic; it's a **systematic headless-vs-UI runtime difference** that breaks
foraging only in the UI.

## How we know (evidence, not assumption)

- **n=5 headless:** seeds 5/42/7/13/99 → hits 7/11/5/13/10, all full-arena, boredom <0.1. Robust.
- **UI:** operator snapshots at 18:01/18:24/18:42 all 0 hits, wall-follow, boredom ~0.7. Consistent.
- **Clean tick-by-tick diff** (operator's `rt:"ui"` log 18:43:27 vs headless, both seed 5): they
  diverge at the **first diag sample (tick 60)** by **float-tiny amounts** — motor-TLE 0.6483 vs
  0.6471, 2 of ~120 spike draws flipped, position off 0.01, heading off 0.2° — while the **brain
  output is identical** (`accel` and `cog_steer` equal). So the divergence is **upstream of the
  brain, in physics/sensor floats**, then amplified.
- Brain is **single-threaded sequential** (Scheduler.cpp:5,62) → not a threading race.

## TWO TRAPS that wasted most of the session (both now fixed — don't re-fall-in)

1. **`body_model` was not config-self-contained.** Body resolved it from ExperimentConfig/env/.tscn
   default but NOT the config metadata → headless silently ran `differential_paddler` (tscn default)
   while the launcher ran the config's `bidirectional_paddler`. Different bodies. FIXED: body reads
   metadata (commit 560c460); default switched to differential (82ea352); launcher dropdown reaches
   the body. **bidirectional_paddler currently STALLS** (pause-baseline) — deferred.
2. **`--headless` writes the SAME `user://logs/godot.log` as the UI.** My headless test runs
   (stdout→/tmp, but Godot still writes godot.log) landed in the file I parsed as "the UI run." I
   repeatedly read my own foraging headless runs and called them the operator's UI. PROVEN: godot.log
   hit-ticks == my /tmp headless exactly. FIXED: every JSONL line now carries `"rt":"headless"|"ui"`
   + a startup `{"event":"RUNTIME",...}` (commit 3a52d98). **ALWAYS filter `rt:"ui"` for UI runs;
   use the TIMESTAMPED logs, not godot.log (I keep clobbering godot.log with headless runs).**
   - World-seed bug also fixed en route (72ae884): the world ignored the launcher seed (used @export
     42); now reads ExperimentConfig.seed_value.
   - Food made NON-BLOCKING (47beae6): was a solid StaticBody obstacle (8 of them) → suspected
     collision-order non-determinism + wedging; now own layer 2, body doesn't collide, front mouth
     (mask 2) still eats it. Did NOT fix the UI (still 0 hits) → food-collision was not the cause.

## THE OPEN ROOT CAUSE (next session starts here)

Headless↔UI diverge at tick 60 by float-tiny amounts with the brain identical → **float-level
physics/sensor non-determinism between the runtimes** (Godot physics/FPU behaves subtly differently
with rendering active vs `--headless`), systematically biasing the UI into wall-following. Open
question: is it (a) genuinely float-nondeterministic and the forager is on a knife-edge, or (b) a
SYSTEMATIC UI-only effect (a render-frame side-effect, a delta difference, a sensor that reads
differently when rendering is on)? n=5 robustness in headless argues the forager is NOT knife-edge
across seeds, so (b) — a consistent UI runtime effect — is more likely. NOT yet isolated.

### Next-session attack order
1. **Get TWO `rt:"ui"` runs at the same seed** and diff them → is the UI itself deterministic? (If
   yes, it's a systematic headless-vs-UI float bias; if no, within-UI non-determinism.) Use the
   timestamped logs filtered by `rt:"ui"`; do NOT run headless in between (it clobbers godot.log).
2. **Find the first divergent TICK** (need per-tick logging, not 1/60) between a `rt:"ui"` and a
   `rt:"headless"` run → which sensor/quantity flips first.
3. Candidate UI-only effects to check: physics delta (is `_physics_process` delta truly 1/60 in the
   UI?), any `_process` (render-frame) code feeding the sim, the FPV Camera3D transform/viewport
   affecting the vision raycast when rendering is on.
4. Pragmatic fallback: **do the science in headless** (where it forages robustly) and treat the UI
   as viz-only, OR make the forager so robust the float bias can't flip it.

## ASSETS to latch onto (operator note)

- **Headless forager works** (n=5, 5–13 hits, full-arena, low boredom) on differential_paddler with
  the current escape/forage/mouth/proximity stack. The Stage-A foraging substrate is real in headless.
- **Bidirectional paddler had genuine ALIVENESS early** (operator: "a few moments of aliveness") —
  varied fwd/back/turn/pause locomotion, the morphology win (removed phantom-TLE). It currently
  STALLS in the cell-cognitive stack (pause-baseline → 0 hits) but the aliveness is a foundation to
  rebuild on. Fix candidate: a forward baseline bias so reverse/pause are active options, not the
  default.
- The **critic-homing learning** (the original Stage-A goal) findings this session were on
  differential (now the default, forages in headless) — see [[v6-cell-cognitive-stagea]]: state
  fragmentation FIXED (coarse bearing+proximity state), but the per-tick steer critic can't extract
  a stable policy from the closing-speed reward (Q-bins stay tied); Mode-2 + mouth gave a first weak
  signal (0.40→0.65). That work is valid for headless.

## Reproduce
```bash
# headless forages (rt:headless):
OGMA_SEED=5 OGMA_TURBO=1 OGMA_EPISODE_LENGTH=8000 OGMA_QUIT_AFTER_TICKS=8060 \
OGMA_CELL_CONFIG=res://addons/ami_ogma/configs/the_cell_cognitive.json \
godot4 --path godot_host/project --headless --fixed-fps 240 --disable-render-loop \
  res://scenes/the_cell.tscn
# UI fails (rt:ui): launch normally, the_cell_cognitive.json, seed 5 → 0 hits, wall-follows.
# Parse ONLY rt:"ui" lines from the TIMESTAMPED logs in
#   ~/.local/share/godot/app_userdata/ami-ogma/logs/  (godot.log gets clobbered by headless runs)
```
The run snapshot (HUD "Copy log") now self-reports body_model / terrain / pillars / drain /
green_gain / elapsed-tick, and every line has `rt`.
