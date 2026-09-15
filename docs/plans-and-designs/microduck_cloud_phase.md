# Microduck: the cloud phase, and where the next one starts

Status: closed, R56 promoted · Dates: 2026-09-12 → 2026-09-15 · Branch: `duck-l2` · Simulation only

*This page is the entry point to the phase, not its record. Every number below is derived, with its
verdict and its caveats, in [`microduck_rung2_regime_design.md`](microduck_rung2_regime_design.md)
§17.28–17.33; the sensor studies that opened it are in [`microduck_tof_studies.md`](microduck_tof_studies.md).
Open items live in the [register](open_items_register.md) (O36, O38, O40–O44).*

---

## 1. Where the duck is

The operator's direction at the start: the duck "stares into space". Every behaviour should go toward
novelty, novelty should be about **things**, and the ToF sensor should be pushed for everything it can
give. At the end, on R56: *"the robot is able to make a very solid map in a very short amount of time."*

**Promoted stacks** (launcher presets, `tools/duck_launcher`):

| star | what | promoted |
|---|---|---|
| `★ STACK` | the R19 stand at the joints: the joint brain takes the legs when the body is still | earlier |
| `★ HEAD` | R34: the head brain owns the head joints, level and steadier than the walker's own | 2026-09-11 |
| **`★ CLOUD`** | **R56: the stop's glance** (§2) | **2026-09-15** |

**The promoted run**, as the harness runs it (from `mj_host/`; the preset uses seed 3):

```sh
./build/ogma_mjhost --level2 models/microduck/scene_playroom.xml --graph configs/a1v2_r46_cloud.json \
  --secs 1500 --seed 3 --noise 0.05 \
  --head-graph configs/head2_h2_level_slow.json --load-head checkpoints/head2j_h1_s2.json --head-joints \
  --stop-from 600 --stop-every 80 --stop-secs 60 --stop-brain configs/a1v2_r19_settle_each.json \
  --stop-load checkpoints/duck_r19_s2.json --stop-keep-head --stop-freeze-head --map-on-stop \
  --stop-gaze 0.35 0.08 1.0 6 6 --stop-gaze-residual 1.0 --stop-gaze-learn 0.5 \
  --cloud --log-cloud-profile \
  --stop-gaze-sweep 0.6 0.7 --stop-gaze-sweep-slow 1 --map-view cloud --stop-cloud-end 0.45
```

For a seed-averaged A/B, pass everything after `--noise 0.05` as `--host-args` to
`python3 mj_host/tools/l2_sweep.py mj_host/configs/a1v2_r46_cloud.json --scene
mj_host/models/microduck/scene_playroom.xml`, and add `--logdir DIR --full-logs` for any cloud analysis.
The graph `a1v2_r46_cloud.json` is R43 plus a `CloudMap` module and an object EPM over its profile.

## 2. What a run does now

- **0–600 s:** the identification babble, walking.
- **The walk:** the twist brain drives, and the play loop navigates over the place map's nodes.
- **From 600 s, a stop every 80 s (a timer, see O43):**
  1. The walker settles, the R19 joint brain takes the legs, and the head brain's learning is frozen.
  2. **The gaze sweeps.** It never holds: it moves at a steady 0.6 rad/s toward whichever angle this stop
     has looked at least, within ±0.7 rad of yaw and a 12° pitch band.
  3. **The cloud accumulates.** `ogma::CloudMap` takes every ToF cast (12.5 Hz), gravity-levelled and
     de-rotated by the duck's own odometry yaw, into 4 cm voxels classified by the mean height of their
     points. It opens on the body's own stillness and files when the body moves.
  4. **The place map learns from the cloud,** not the frame. The map's view slots carry `CloudMap::view()`:
     across ±64°, 8 sectors, each the nearest off-floor return over 4 m. The map learns only at stops, and
     on the walk it holds the last cloud's view.
  5. **The stop ends when the cloud stops growing:** once the new voxels over 2 s have stayed below 45 % of
     this stop's own peak for 2 s more. Median 10.5 s (p10 7.3, p90 15.3).
  6. **The cloud is filed** under the map's place (LRU 8), with a revisit score against the last cloud of
     that place. Its 36-dim floor-break profile feeds the object EPM.

## 3. The phase in one table

n = 6 seeds × 1500 s in the playroom unless stated.

| step | § | what was learned | verdict |
|---|---|---|---|
| The ToF studies | 17.28 | The raw 8×8 frame is two-dimensional (PC1 85 %), and every frame-based view is a **pose code**. A small object is sub-pixel per cast (0.058 points on a block) and multi-point per sweep (43), so **the swept cloud is the only level at which it exists.** | characterisation |
| The cloud in the host; an object vocabulary over it | 17.29 | An EPM over the cloud's break profile holds 16 nodes, all baked, switching 6–7 times a minute (the place map: 60–130). It captures 95 % of the object information that pose leaves (frame views 64–75 %). Aiming the gaze at the floor: `NULL`, and O36 closed by measurement. `--body-predicts` gives a contact channel 2.7× the twist brain's. | `WORKING` / `NULL` / `PARTIAL` |
| `CloudMap` becomes a module | 17.30 | Stillness-opened, filed under the map's place, passive (the walk is byte-identical with it running). Three bugs fixed, with their claims withdrawn: the de-rotation sign, a voxel-centre test that counted the floor as things on it (82 %), and a recorder OOM. | `WORKING` |
| The voxel viewer | 17.30 | `tools/run_voxel_viewer.sh`: see what the duck can perceive. | tool |
| The stack rule | 17.31 | A break-band cluster whose stack tops out below 16 cm (gap max(10 cm, 0.12 × range)) is a small thing; one that keeps rising is an obstacle. Precision 0.11 → 0.53 at full recall, out of sample. | `WORKING` as a reduction (not yet a module output) |
| R52, the gaze sweep | 17.31 | The babble's head moved on 6 % of stop ticks. A gaze that never holds made clouds ×3; objects in reach found 67 → 75 %, balls 21 → 33 %, precision 0.62 → 0.92. The frame-fed map churned. | `WORKING` |
| R53, the map reads the cloud | 17.32 | Winner switches 77 → 25 a minute, walks straighter (both 6+/0−); the map's error rises as the cost. | `WORKING` |
| R54, stops end on growth | 17.32 | Stops 58 → 24 s, stands 66 / 66. | `WORKING` |
| R55, a constant 0.6 rad/s | 17.32 | The novelty slow-down had held the head at 0.075 rad/s on 69 % of ticks. Quality holds to 0.6 rad/s, walls fray above it, and **head speed does not shorten stops.** | `★` 2026-09-15, superseded by R56 |
| **R56, growth threshold 0.45** | 17.33 | Stops p50 20.7 → 10.5 s; walls located 66 → 70 % of the in-reach wall; objects in reach 85 → 72 %, balls 23 → 34 %; walls read as small 2.3 → 4.3 %. | **`★ CLOUD`, 2026-09-15** |

## 4. Findings that should shape the next phase

1. **The frame is a pose code; the cloud is where things exist.** Anything object-shaped should read the
   cloud, or something built on it, never a single frame (§17.28–17.29).
2. **Read an accumulating percept as the accumulation.** Fed a moving frame, the place map churned
   (77 winner switches a minute) and a stop never ran out of novelty. Fed the cloud's view, it steadied
   (25). The same will hold for anything downstream of a sweep.
3. **End a gathering behaviour on the gathered thing's own growth, relative to its own peak.** It is
   scale-free, and it turned tempo into one dial with a measured quality curve (§17.33). Head speed looked
   like a tempo dial and is not one.
4. **As a cloud shortens, the obstacle side thins first.** Walls and furniture misread as small rise from
   1–2 % to about 4 %, while objects in reach and balls hold within seed noise.
5. **Measure a sensor reduction before building on it, and score it with instrumentation the brain never
   reads.** The stack rule was scored against the scene manifest and the simulated object positions, and
   the wall metric against the room's true geometry.
6. **In simulation a held gaze misses what lies between rays,** since each zone is one ray. A moving gaze
   fills the gaps. This is partly a simulator effect (§6).

## 5. Tools and records built this phase

- **`tools/run_voxel_viewer.sh RUN.jsonl`** shows every filed cloud in 3D: World frame (anchored poses,
  instrumentation) or Body frame (what the duck has), height bands or hits, and `--screenshot`. See
  `tools/xaq_inspector/README.md`.
- **`mj_host/tools/cloud_objects.py`:**
  - `rules LOG…`: the stack rule's ladder, by what each cluster really was.
  - `arms NAME=GLOB…`: per arm and seed, head speed and motion, voxels, objects in reach and balls found,
    the rule's precision, obstacles misread as small, and walls located.
- **`mj_host/tools/l2_sweep.py … --logdir DIR --full-logs`** keeps the host's whole JSONL (about 110 MB for
  a 1500 s run).
- **`tools/duck_viewer`** draws filed clouds in live, replay and record modes (`P` toggles, `N` solos a
  place). `record` streams frames and takes `--from / --to / --every`.
- **`ogma::CloudMap`** (`cpp_core`, `test_cloud_map` 8/8) exposes `voxels`, `break_voxels`, `new_fraction`,
  `revisit_change`, `revisit_anchor_dist`, `profile()`, `view()` and `last_filed_voxels()`.
- **Host flags,** each off by default and each checked byte-identical when off: `--cloud`,
  `--log-cloud-profile`, `--log-tof-cloud`, `--body-predicts`, `--stop-gaze-sweep SPEED YAW_MAX`,
  `--stop-gaze-sweep-slow F`, `--map-view cloud`, `--stop-cloud-end F`.
- **JSONL records:**
  - `cld` on every tick the cloud is open: `[voxels, break, new_fraction, revisit, cached]`.
  - `cloudv` on every filed cloud: voxels as `[ix, iy, iz, hits, mean_height_mm]`, the anchor pose,
    `revisit`, `revisit_dist`.
  - `cldp`: the 36-dim profile.
- **The A/B logs** (gitignored, `mj_host/log/`): `sweep_ab` (R52), `cloudview_ab` (R53–R55), `cloudend_f`
  (R56), `tof_study/stack_s*` (the stack rule).

## 6. Traps that cost runs this phase

Check these before trusting a result.

- **Never filter a build's output to "error".** The first round of §17.30 was verified on a stale binary,
  and the unfiltered rebuild showed the warning that was the bug.
- **A flag-off guard compares JSON lines, not whole files.** The host prints two banner lines to stdout when
  the inspector port binds, and a concurrent run's busy port hides them. Use `grep '^{' LOG | md5sum`.
- **The harness's `--logdir` alone writes a compact stream** with no cloud records, qpos or joint positions.
  Add `--full-logs`.
- **Read head motion from the joints** (`q[7]` yaw, `q[6]` pitch), not the command. A babble step changes
  the command on one tick and the head then travels for ten.
- **Parallel shell calls share a working directory.** A `cd` in one breaks relative paths in the others;
  use absolute paths.
- **Memory.** A 1500 s replay recorded to video buffered every frame and was OOM-killed at 23.9 GB. Run
  heavy jobs under `systemd-run --user --scope -p MemoryMax=… -p MemorySwapMax=0`. Write large logs under
  `mj_host/log`, not `/tmp` (tmpfs quota).
- **Arms that differ only at stops are identical until 600 s,** so the first stop is a clean visual A/B in
  the voxel viewer.
- **The EPM's `is_novel`,** which the harness reports as `novel%`, is a percentile, not a novelty measure.

## 7. Sim-to-real caveats carried forward

- The simulated ToF casts **one ray per zone**, while the VL53L8CX integrates each zone's 5.6° cone. The
  sweep's gain on balls probably overstates the hardware's; its coverage gain should transfer.
- The simulator reads the head pose at the instant of each cast. On the robot each frame must be
  **timestamped against its head angle** (30 ms at 0.6 rad/s is 0.9°; at 1.5 rad/s, 2.6°).
- **No MJCF variant has the mouth hinge,** so nothing can be grasped in simulation. Pollen's `ground_pick`
  is a phase-scripted 4 s cycle, and their skill runner (playroom plan X2) is not in the host.
- The simulated ToF runs at 12.5 Hz (every 4 ticks); the sensor's 8×8 mode runs at up to 15 Hz.

## 8. Open for the next phase

Ranked by what the operator has pointed at.

1. **When to stop (O43).** Stops still come on a timer, every 80 s from 600 s, so shorter stops lengthened
   the walks instead of adding glances. What should *start* a stop is the next design question, and by the
   rewrite rule it is an error to find, not a schedule. Candidates the data already carries: the place
   map's error on the walk, a bearing where the held cloud view and the frame disagree, and a small thing
   flagged within reach in the last cloud.
2. **Seeking small things (O44, the operator's idea in §17.31).** Walk toward things smaller than itself and
   interact with them.
   - Measured so far:
     - the stack rule (not yet a `CloudMap` output)
     - small objects are seen at 0.6–2.2 m, and the last ~0.5 m is blind at the stop's gaze
     - R44's approach is a scaffold, and its change detector refuses a sweeping gaze (`--stop-orient` is
       rejected with `--stop-gaze-sweep`)
     - the object vocabulary's pose invariance is untested
     - nothing can be grasped in simulation
   - The framing agreed in discussion: the pull is the object vocabulary's error at a flagged cluster;
     habituation lets the duck move on; and a moved object gives the (d) test for free.
3. **The map's error on the walk.** Holding the last cloud's view raised the map's TLE from 0.17 to
   0.25–0.32. Whether that costs the play loop's navigation is unmeasured.
4. **Walls on longer walks (O35).** The `★ CLOUD` stack walks about three times as far as R52 did, and its
   wall contacts are noisy (single seeds up to 81 a minute). R48's model-implied step (14.8 → 1.8 a minute
   on R43) is not in this stack.
5. **The stop's floor** is about 7–8 s, set by the 2 s judgement windows. Anything shorter needs a shorter
   window, not a higher threshold.
6. **Smaller items:** judging a revisit by registering cloud content instead of dead reckoning (O40,
   §17.30); the stack rule's chair-leg false positives (a leg's footprint is 8 cm against 12–16 cm for the
   objects).
7. **Waiting from before this phase:** Pollen PR #260 (open, awaiting review, as of 2026-09-10); W4 (the
   mux) and W5 (leaving a surface) of the walk-stop-look line; O39 (a speed that means something).
