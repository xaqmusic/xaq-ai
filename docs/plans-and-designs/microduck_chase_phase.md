# Microduck: the chase phase. Homing in on what moves

Status: **stage 0 and stage 1 measured 2026-09-27** (the stimulus, the instrument, the signal; the chase built, `WORKING` as a mechanism, `NULL` for the train in this room; the walking cloud's bearing fixed, the loud result); `★ THINGS` = R83 unchanged · Dates: 2026-09-27 → · Branch: `duck-l2` · Simulation only. **Picking the duck up cold? Start at §6, then §7.**

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

- **`playroom_gen.py --train`** adds `mov_train0` on an oval track and writes the track into the scene as
  `<custom><numeric name="train_path">` = [cx, cy, a, b, yaw] (and `train_z`). The sleepers are non-colliding and
  4 mm tall: floor to the ToF. **Two rooms have existed.** Stages 0 and 1 (R84–R89) were measured on the *first*:
  a 12 × 5 × 6 cm train on a 0.60 × 0.40 m oval placed last on the free floor in the room's −x +y corner
  (perimeter 3.17 m), the rug moved to the middle of the floor after stage 0. **The second, from the operator's
  eye on R89 ("the robot is not really interacting with the train at all… make the track larger, wider and longer;
  it can stretch almost the entire distance between the green wall and the wall across from it; make the train
  itself larger, similar to the purple block"): the track is laid FIRST, centred on the room with its long axis
  along y (the green wall is +y), 1.52 × 0.80 m to 0.45 m of the walls, perimeter 7.48 m; the train a block-sized
  18 × 10 × 10 cm box; the furniture and the things are placed clear of the track (0.25 m), so the train room's
  small things land elsewhere than the plain room's; the rug lies inside the oval around the duck's start.** The
  track draws no random numbers, so the plain playroom stays byte-identical. Why the train and not the ball, in
  the operator's words: a regular schedule, a known velocity, and control by the seed.
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

**The walk under the walking cloud (O65).** The operator, watching R84 seed 1: caught in a corner from 893 s with
several falls, the wall hugging worse than R83's. The profile: wall episodes 256 → 944 on seed 1 against R83, most
of them from 800 to 1400 s with play steering; the seek loop took no more targets. R84 and R85 share that walk, so
the chase A/B is fair, but no chase stack is promoted on it; `CloudMap.walk_things false` (things at stops only, the
mover still on the walk) is the first lever against it (R86).

**Traps of this stage:** the walking cloud is off in R83 (`walk_cloud` false) — a movers instrument on R83's
config logs nothing between stops; the train room is a separate scene file (`scene_playroom_train.xml`), and a
sweep needs `--scene` pointed at it; the plain playroom must regenerate byte-identical after any generator change
(check `playroom_gen.py --seed 1 --out /tmp/x.xml` against the checked-in file); a train on a closed track re-enters
its own trail within a cloud's life, which reads as old voxels on a moving thing (a rolling ball will not do this).

**Register rows:** O63 (the phase), O64 (the mover gate's false alarms: newly-in-view things and sliding
fragments), O55, O61, O49.

## 7. Stage 1, built and measured (2026-09-27 evening; design doc §17.54)

**Built:** the mover candidate (`CloudMap.mover_topic`: the young cluster against the oldest one's age, within 1.5 m,
any size) and the chase (`BearingSeekLoop.mover_topic`: a candidate confirmed by its own prediction over 0.5 s, then
the predicted position as the target, need 1, no arrival, forgotten after 1 s into a remembered target); six unit
tests; the `"chase"` record; `mover_tracks.py --chases`. Arms: R85 = R84 + the chase; R86 = R85 + `walk_things false`
(the things reduction at stops only, O65). Presets R85 and R86, the R84 argv.

**The first arm found a bug, not a verdict.** R85 on R84's control chased the room: 338 chases in six runs, one at
the moving train, the chased "movers" reading the body's own speed. The walking cloud's bearing was taken from the
cloud's anchor rather than the body, so every static cluster read as moving at the body's velocity once the body had
walked from the anchor — the same bearing that fed R74's walk re-fix (O61). Fixed with a unit test; R84, R85 and R86
re-measured together on the fixed bearing (§17.54 for the numbers).

**On the fixed bearing (n = 6):** the fix itself is the loud result — R84 walks at 12.4 walls a minute against 31.1
before and R83's 28.3, seed 1 from 68.5 to 6.1; O65 is answered by it, and `walk_things false` (R86) is a
`REGRESSION` (walls 20). The chase on the age gate (R85) is `NULL` for the train and a `REGRESSION` for the walk's
ownership: 447 chases, 4 % at the moving train, seek holding the reference 88 % of the walk. A young cluster that
persists half a second is mostly a static thing whose voxels are being entered as the body turns, and "still near
the prediction" is satisfied by a thing that stays put. The confirmation must ask for the other half of a change:
the voxels the thing LEFT. Stage 1b: the cast carries the sensor origin, every returning ray marks the occupied
off-floor voxels it passes through as vacated, a cluster counts its trail, and the candidate needs one (R87 the
instrument, R88 the chase with a trail; measured in §17.55).

**Stage 1b and 1c, measured (§17.55; the corner track).** The trail at 4 cm voxels is `NULL`: a trail on 68 % of static clusters
and 80 % of the moving train's, because any surface crossing a voxel fills it partly and a ray through the empty
part reaches the floor or wall behind as readily as the floor behind a train that has gone. R89 (the candidate
must have MOVED, be compact and within 1.2 m): chases 447 → 72 in six runs, the walk's ownership back to play
(seek 88 → 58 %), and still **3 chases of the moving train in six runs**. The verdict: the chase is `WORKING` as
a mechanism and `NULL` for the train in this room at this power — the train comes within the chase's reach too
seldom (681 moving-train samples against 200 000 static in six runs), and what survives every gate is a fragment
of the room sliding into view as the body turns.

**On the big track (§17.56):** the opportunity tripled (4 745 walking casts with the moving train in the cone
within 1.5 m); R89 chased the moving train 8 times in six runs (11 % of its 71 chases, from 4 %), the stopped
train 5, static clusters 54; walls 18 ± 20 against R84's 14.7, rescues doubled. `PARTIAL` on the train,
`REGRESSION` on the walk: the false chases are the design's own, not the room's, and the room is now right for
removing them.

**Where this leaves the design (for the discussion).** Keep R89's chase; bring the stimulus to the walk before
touching the gate again (a track through the middle of the room, or the ball rolled across the walk as
`--roll-past` rolls it across a stop); the one confound left is the sliding fragment, whose remover is geometric
(along its own surface, correlated with the body's yaw rate) or T6 at a finer resolution — the real sensor's
multi-target returns would give the trail directly.
