# Microduck: the chase phase. Homing in on what moves

Status: **stage 0 measured 2026-09-27** (the stimulus, the instrument, the signal); no lever built yet; `★ THINGS` = R83 unchanged · Dates: 2026-09-27 → · Branch: `duck-l2` · Simulation only. **Picking the duck up cold? Start at §6.**

*The phase after the things phase ([`microduck_things_phase.md`](microduck_things_phase.md) §10–13, `★ THINGS`
R83). The operator's direction, the design discussion in the rewrite rule's terms, the stimulus built for it, and
what stage 0 measured. Every verdict goes to the rung-2 design doc §17 (from §17.53) and the
[register](open_items_register.md) (O63, O64). Nothing here is measured unless a section says so.*

---

## 1. The direction

The operator, 2026-09-27: the duck has a good base of behaviours; the next loop's behaviour is chasing moving
objects. The robot must use the time-of-flight sensor **while walking** to tell whether some part of the current
cloud is changing relative to the rest of it — the static world is the reference, and a mover is a cluster whose
position in that frame changes while everything else holds still. The robot should home in on **any cluster of
voxels moving relative to the world frame**, whatever its size.

And, on the stimulus: the ball only moves when the duck hits it, so a chase would start from a successful reach,
the rarest event in a run. "How about a toy train or car on a track that stops and starts at regular intervals?
If we want to be more abstract we could move a child-sized capsule through the space."

## 2. The error, by the rewrite rule

The seek loop already carries a forward model: it dead-reckons a fixed thing under the assumption that things
do not move. A rolling ball, or a train, is the residual of that prediction. The chase error is **the thing's
predicted position against its seen position in the odometry-anchored frame**, and chasing is descending that
residual continuously instead of fixing a position once. The honest signal stays the remaining range.

**The sensor check** (CLAUDE.md §1 step 2), from the code: the ToF is 64 zones over 45° at 12.5 Hz, no noise in
simulation, the robot invisible to itself (`Tof.cpp`, group 0 only); the walking cloud translates each cast by the
odometry and re-anchors every metre (`CloudMap.walk_cloud`, O55). Two things the cloud threw away before this
phase: a zone that returned nothing was discarded on insert (no free-space bookkeeping, T6/O49), and an occupied
voxel was never removed — so a thing that moves leaves a **smear**, which the whole-cloud stack rule reads as one
long obstacle. Each voxel does keep its first- and last-seen ticks and its hit count, which is what stage 0 used.

**Three ways to build it**, discussed and ranked before anything was measured:

1. **A moving fix inside the seek loop** (chosen first). The things reduction on the walking cloud through a
   short recency window; a mover found among its clusters by the cloud's own numbers; a target with a velocity,
   homed to at its predicted position; the reach by the peck from the walk (R82b), the only reach that worked.
2. **A separate chase loop** — the same signal, its own bearing, need and trust channel in the arbiter. Cleaner
   in the recipe's terms; costs a steer code, a competence module and a voter channel before anything is measured.
3. **Free-space bookkeeping along each ray (T6, O49)** — the substrate form of the orienting reflex, and the
   right long-term detector; on the walk it also fires on odometry drift at every edge. Second, once 1 shows the
   thing is trackable.

**The consequence of "any moving cluster"**: the mover gate cannot be the stack rule's `small` verdict. Motion is
what makes a thing a target; the kind vocabulary can label it afterwards so a known kind can lose its pull.

## 3. The stimulus: a toy train on an oval

Built 2026-09-27, off by default, the plain playroom byte-identical (`playroom_gen.py` regenerates it unchanged):

- **`playroom_gen.py --train`** adds `mov_train0` — a box 12 × 5 × 6 cm, inside the stack rule's small-thing
  band, so a *stopped* train is a thing the existing vocabulary sees — on an oval track placed last on the free
  floor (0.35 m from the walls, 0.25 m from everything but the rug, clear of the duck's start), and writes the
  track into the scene as `<custom><numeric name="train_path">` = [cx, cy, a, b, yaw]. The sleepers are
  non-colliding and 4 mm tall: floor to the ToF. The seed-1 room's track: centre (−1.02, +0.72), 0.60 × 0.40 m,
  perimeter 3.17 m (`scene_playroom_train.xml`). With the train the **rug lies in the middle of the floor**, around
  the duck's start (the operator, after stage 0: under the track it z-fought the sleepers); the seed's random draw
  and keep-out for it are kept, so every other placement is the plain room's, and the track is the same one stage 0
  measured.
- **`--train SPEED RUN_S STOP_S`** drives it kinematically (pose and velocity written every tick, so contacts
  meet a mover that does not yield) at SPEED for RUN_S, still for STOP_S, repeating; the phase of the schedule
  and the start along the track follow the **seed** (six seeds meet it at six points of the cycle) unless
  `--train-phase S`. The record carries `"train": [x, y, yaw, vx, vy, moving]` — truth, for the scorer.
- **`--log-movers WINDOW_S`**, the instrument: on every cast tick with a cloud open, the stack rule over only
  the voxels seen in the last WINDOW_S seconds (`CloudMap::cluster_recent`, two unit tests) — `"mvc"`:
  [cx, cy, ext, top, ncols, hits, small, **fresh, age_s, age_w_s**] per cluster in the cloud's frame (fresh: the
  share of its voxels first seen inside the window; age: their mean age; age_w: hit-weighted), `"mva"` the
  anchor's world pose, `"mvw"` 1 for a walking cloud.
- **`mj_host/tools/mover_tracks.py`** scores a full log: labels every cluster by truth (train / obj / static /
  wide — a static cluster wider than 0.25 m, whose *visible* part slides with the field of view), tracks them
  across casts of one cloud, fits a velocity over the window, and reports visibility, speed distributions,
  freshness, voxel age, and what each candidate gate would catch and what it would fire on. `--svg`, `--svg-age`.

A stopped train is a remembered thing (the seek loop handles it today); a moving one is the stimulus; each
transition is a (d) test for free. The capsule — a person-sized mover, what the orienting reflex handled with the
carried chair — is the second stimulus arm, precisely because the chase must not depend on size (unbuilt).

## 4. Stage 0, measured: does a moving cluster separate from the static room? (2026-09-27)

**Run.** R84 = `★ THINGS` R83 + `cloud.walk_cloud true` (`a1v2_r84_train.json`; R74's sensor, its behaviour
ungated), the train room, `--train 0.2 8 8 --log-movers 0.5`, n = 6 × 1500 s, `--full-logs`; preset "R84 · stage 0
of CHASING MOVING THINGS" (seed 1). The walk is R74's: walls 31 ± 24 a minute (3 to 68 across seeds), seek 38 %
of the walk, stands 34 %, arrival stops 7.2 a run. Not a lever; no A/B. Numbers in the design doc §17.53.

**What the sensor sees.** With the train within 2 m and 0.6 rad of where the head looks, it is a cluster of
the walking cloud on **65–80 % of casts at 0.5–1.5 m**, moving or stopped, and on a third at 1.5–2 m. The signal
exists at the range the reach works from.

**The centroid's velocity through the window: `NULL` as a separator.** The moving train's estimated speed
reads 0.09 m/s (p50; truth 0.20) — the window's smear lags the centroid and the tracks break on flicker — while
compact *static* clusters read 0.05 p50 and **0.38 p99**: a wall base's visible part slides with the field of
view, and a sparsely hit leg's centroid jumps between voxels as the sampling wanders. No threshold set from the
static spread catches the train (recall 0 at 1 × p99); subtracting the common mode (median 0.03 m/s) changes
nothing, because the jitter is per cluster, not the odometry's.

**The voxels' own age: the separator.** A thing that moves keeps entering voxels the cloud has never held, so
its voxels are as old as the time it takes to cross one (4 cm at 0.2 m/s = 0.2 s); a static thing's voxels are
re-hit and are as old as the watching. Mean voxel age of the cluster: moving train **0.26 s** (p25 0.13),
stopped train 0.96 s, static **7.1 s** (p25 1.6 s, p10 0.26 s), wide 7.4 s, balls and blocks 6.4 s. A gate at
0.3 s catches the moving train on 55 % of its clusters (the tail of old-voxel train samples is the train
re-entering its own trail on the closed track), the *stopped* train on 20 % (it ages out within a second — a
train that just stopped is still interesting), and fires on **167 static clusters a minute of walking cloud** —
1.7 a minute within 0.5 m, 13 at 0.5–1 m, 36 at 1–1.5 m, 60 at 1.5–2 m; by class 141 compact static, 16 wide,
10 balls and blocks. The false alarms are a static thing *newly entering the view* (its voxels are young until
it has been watched) and well-sampled fragments of big things sliding into view. The moving train's centroid
velocity, on the clusters that pass the age gate, points the right way to **0.31 rad** (p50): usable for a lead.

**Freshness** (the share of voxels first seen inside the window) is the weaker form of the same measure: moving
train 0.50 p50, static 0.09 p50 but 0.71 p90; a gate at 0.7 catches 29 % and fires 158 a minute.

**Verdict: `WORKING` as a signal, `NULL` for the centroid's velocity.** The go/no-go passes on the voxel age:
a moving thing is an order of magnitude apart from the static room on a number the cloud already keeps per
voxel, with a transient (a thing newly in view) that a loop's own persistence removes and a rate at range that
the last-metre gate removes. Stage 1 builds the gate from these, not from a velocity.

**The readout for the eye:** <https://claude.ai/code/artifact/ce14ac2a-3f9f-46de-9e37-f14260c918ee> — the age and
speed scatters and the gate table, six seeds.

## 5. Stage 1's design, as stage 0 leaves it

- **The candidate** is a cluster of the walking cloud through the window whose mean voxel age is small against
  the cloud's own — set from the cloud's running spread of cluster ages, not a constant (CLAUDE.md §5 rule 5);
  0.3 s against 7 s is the measured gap. Any size (§2): `small` is not the gate.
- **The confirmation is the loop's prediction** (§2): the chase holds a candidate's position and velocity (the
  two-sighting difference; direction good to 0.3 rad) and keeps it only while a young cluster is found near the
  predicted position on the next computations. A static thing that just came into view ages out in under a
  second; a fragment sliding along a wall base does not follow the prediction. This is the moving fix of §2 option
  1: the seek loop's held target with a velocity, re-fixed while the prediction holds.
- **Range.** Within 1 m the age gate fires on nothing 15 times a minute; at 1.5–2 m four times as often. The
  chase's reach is from the walk in the last metre (things phase §13, item 1) — gate the candidate to the range
  the reach works from, and let the far ones be what the walk drifts toward, not what it commits to.
- **The reach.** The peck from the walk on the chased thing (R82b's `--skill-now`), so the cycle closes: chase,
  peck, the thing moves again, chase.
- **The (d) tests, all in the harness:** the train stops (the chase should end within a second, the seek loop
  keeping the stopped train as a remembered thing); the train starts again (re-found); `--train-phase` to pin the
  schedule for the eye; the moved-ball tests of the things phase §7.
- **Instruments:** `mover_tracks.py` on the stage's logs (the same labels score a loop's choices: what it chased,
  what it really was); the sweep's `walls`, `seek%`, `stopsArrive`, `spins`; a new column for movers chased and
  reached.
- **Open on the sensor side:** the hit-weighted age (`age_w`) is measured and buys about a sixth fewer false alarms
  at matched recall (§17.53) — the weighting cannot help where a false alarm's voxels are all new; T6's free space
  along each ray (O49) as the detector that would also see a thing *leave*.

## 6. Where the duck is (for a cold start)

`★ THINGS` = R83 is unchanged (things phase §10). This phase has, as of 2026-09-27 evening: the train room and its
generator flag, the host's train driver and the movers instrument, the scorer, the R84 instrument preset, and
stage 0's measurement above. Next: stage 1, the mover gate and the moving fix, off by default, byte-identical when
off, n = 6 in the train room, scored on stops at things, walls a minute, falls, and movers chased and reached.

**Traps of this stage:** the walking cloud is off in R83 (`walk_cloud` false) — a movers instrument on R83's
config logs nothing between stops; the train room is a separate scene file (`scene_playroom_train.xml`), and a
sweep needs `--scene` pointed at it; the plain playroom must regenerate byte-identical after any generator change
(check `playroom_gen.py --seed 1 --out /tmp/x.xml` against the checked-in file); a train on a closed track re-enters
its own trail within a cloud's life, which reads as old voxels on a moving thing (a rolling ball will not do this).

**Register rows:** O63 (the phase), O64 (the mover gate's false alarms: newly-in-view things and sliding
fragments), O55, O61, O49.
