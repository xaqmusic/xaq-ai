# Microduck: the chase phase. Homing in on what moves

Status: **stages 0–1 measured 2026-09-27/29 (R84–R94)** (the stimulus, the instrument, the signal; the chase built, `WORKING` as a mechanism, `NULL` for the train in this room; the walking cloud's bearing fixed, the loud result); `★ THINGS` = R83 unchanged · Dates: 2026-09-27 → · Branch: `duck-l2` · Simulation only. **Picking the duck up cold? Start at §11 (the resting point, 2026-10-01: ★ BIRD, the bird's neck), then §10 (how it was reached) and §9 (the arms before it).**

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
- **The live cloud view** (`--log-cloud-live`, on every preset from R84; `tools/duck_viewer/README.md`): the viewer
  draws the cloud as the module holds it — the open cloud fading with each voxel's age, a walking cloud vanishing
  when filed, remembered places dim until evicted, the chase's candidate and target as markers. Built 2026-09-27
  on the operator's ask ("more similar to what the robot is perceiving; if the robot is forgetting places, work
  that into the UI").
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

**2026-09-29, after the campaign (§8; design doc §17.60–17.64):** the harness is the identification-only loaded
brain, 600 s a run, stops from the first second (the launcher's "on the LOADED brain" presets). The stack that
came out of it — R94 + `--stop-on-stuck 8 --stuck-escape 6 --chase-vx 0.35`, then R99 with the mover's priority on
top (walls 10 a minute, §17.65) — is confirmed on eighteen seeds (the R94 form): no run
rides a wall for minutes (the base: six of eighteen), walls 39 → 22 a minute pooled, rescues 0.38 → 0.24, the
moving train a third of chases, more stops at things and answers; not promoted without the eye (preset "R94 ·
loaded + the STACK"). The leans are `NULL`, coasting permanence a `REGRESSION`, the pull's decay `NULL`;
permanence in recognition (R98) is measured in §17.64. `★ THINGS` = R83 is unchanged (things phase §10). This phase has, as of 2026-09-27 evening: the train room and its
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

**R90, a position belongs to a thing while it is still (§17.57):** the operator watched the duck walk to where the
train had passed and peck at the place. A chase that loses a moving thing is now dropped, not remembered; the things
reduction never attends a young cluster; a stop ends on a confirmed chase. On the big track against R84: walls 14.7
→ **6.5** a minute (R89 18.1), rescues halved, falls a third, stops at things 8.7 → 12.7; skills at nothing 8 → 15 of
29 → 45 — the remaining reaches at places are things seen standing that moved after, the (d) test's own answer.
`WORKING` on the walk, `PARTIAL` on the gap. Next: the voxel age into the thing descriptor, so the kind EPM earns
a moving kind and the outcome loop learns that it answers nothing.

**R91–R93, isolation and the age dim (§17.58):** the operator's isolation idea — a cluster with tall voxels near
it is part of something tall — is the discriminator stage 0 lacked: 95 % of the moving train's clusters are
isolated against 6 % of static ones, and "young and isolated" fires 13 times a minute instead of 128. On the mover
candidate alone (R93) the chase is the phase's cleanest: 25 chases in six runs, 8 at the moving train and 6 at the
stopped one, 7 at static clusters (from 49). The walk ties the control within its spread and loses R90's halved
walls, which turn out to have been false chases becoming stops (an interesting artifact). The age in the
descriptor (R92) is `NULL` at n = 6: one value in nine under the projection; re-use, a vocabulary of its own over
[age ratio, fresh].

**R94, a lost chase starts a look (§17.59):** a stop on the loop's own loss, the sweep centred on where the thing
went; a chase confirmed during it ends it. Fired 8 times in six runs; chases at the moving train 8 → 10 of 23 (43 %,
the phase's best); walls by seed 22 → 7, 37 → 3, two unchanged (identical records), one corner trap (seed 4, 172 a
minute) that the stuck stop and escape exist for. `WORKING` as a mechanism, `PARTIAL` on the chase, a signal on the
walk. Presets R91–R94 in the launcher.

**Where this leaves the design (for the discussion).** Keep R89's chase; bring the stimulus to the walk before
touching the gate again (a track through the middle of the room, or the ball rolled across the walk as
`--roll-past` rolls it across a stop); the one confound left is the sliding fragment, whose remover is geometric
(along its own surface, correlated with the body's yaw rate) or T6 at a finer resolution — the real sensor's
multi-target returns would give the trail directly.

## 8. The campaign harness (2026-09-29): the first ten minutes from a saved brain

The operator, stepping away: an autonomous run toward a duck that pursues moving targets rapidly, avoids being stuck
against walls, and shows more behaviours in the first ten minutes; and four items — the stuck stop and escape into
the chase stack; a look that finds nothing counting as an answer; no babble in every run if a saved brain will do;
the head forward over the centre of gravity as a speed lever; object permanence for a moving target that has left
the field of view. Every lever guarded, six seeds, the record here and in §17.60 on.

**The harness.** `--load-brain F` now restores a level-2 brain saved by `--save-brain` (the body left at its reset;
the head and stand brains load as before; the intent EPM's whole-body babble counter is in the snapshot, so a
restored brain does not babble again). The checkpoint is `checkpoints/duck_r94_s1.brain.json` — R94, seed 1, the
full 1500 s on the big track. A campaign run is then 600 s with stops from the first second; its first two minutes
are a walk at the brain's own pace (vx 0.22 m/s mean, seek and play steering, three stops) where the babble's are
a zero-mean identification (vx 0.00). The launcher's "a saved brain" start now works for level 2 (presets "on the
LOADED brain"). The sweep's judged window starts at 30 s (`--control-from 30`).

**Measured (§17.60–17.62).** Sweep 1 showed the whole restored brain to be the wrong harness (walls 42 ± 46: a saved
map, play field and cache keyed to another odometry frame and another habituation); the identification alone is
restored now. On that harness (sweep 2, eight arms): the stuck stop and escape `WORKING` (falls a tenth, walls −31 %),
the pursuit at speed a signal (the moving train 38 % of chases), the leans `NULL` (a commanded lean is absorbed or
paid in falls), coasting permanence a `REGRESSION` (the prediction leads into walls), the pull's decay and the wider
reach `NULL`. Sweep 3: the stack of the two keepers holds both gains (walls 27 → 19, rescues 0.62 → 0.20, the
train 38 %); body pitch 0.05 on it is the interaction-rich variant for the eye. The confirmation on seeds 7–18 is in
§17.63.

**The readout for the eye:** <https://claude.ai/code/artifact/10ffc001-5858-47af-98a1-973dc9bba9ac> — the four
items' standing, every arm's line, the eighteen-seed confirmation.

**Sweeps 6–9 (§17.65–17.68), after the operator's second look:** the mover's priority (a lost mover in mind bars
new static targets) is the campaign's best walk — R99, walls 18 → 10 a minute, every seed under 24, the train 43 %
of chases; the follow beyond 1.2 m changed nothing; the head turned toward the chase on the walk is `NULL` at three
gains; the confirmation's knobs cannot raise the chase-per-crossing rate above one in four without buying the room
(the gate never binds; the motion test and the timeouts are the losses; a longer watch loses the candidate). The
standing limit is the chase's start, and the next lever is the age gradient across the window's voxels as a
single-cast velocity. **For the eye: the stack (R94 + stuck stop + chase vx) for the walk, R99 for the pursuit, both on the fixed chase.**

**The chase's start had two defects (§17.69), found from the operator's eye on R99 seed 6:** the cloud's mover
bearing was taken as a new sighting every tick (four per recompute, dragging the velocity to zero: a crossing train
read as still), and a per-step speed test read centroid jitter as 1 m/s (a crossing at half a metre replaced every
half second). Fixed: sightings by the cloud's recompute tick, the speed and velocity over a ring of the last 0.8 s.
On the same six seeds R99 goes from 14 to 47 chases, 22 at the moving train, 8 re-acquired, crossings chased 27 →
52 %, the walk unchanged. Small things taken from a walking sighting within a metre (R106, §17.70) is live and a
tie on the outcome. On seeds 7–18 (§17.71) the fix confirms on every set — but the priority's walk reverses: pooled over eighteen
seeds R99 walks at 27 walls a minute against the stack's 19 and pursues half again as much (117 chases against 79,
35 at the moving train, 18 re-acquisitions). The eye decides the trade: the stack for the walk, R99 for the
pursuit; a pursuit that yields near tall structure would reconcile them.

**The yield near tall structure (R107, §17.72):** built as designed — the cloud counts the tall voxels within a
body length of the seek loop's held target, and a chase whose target has one yields. On eighteen seeds it takes the
walls from 27 to 17 a minute (the stack's level, R99's priority kept; both of R99's trapped runs gone) — and destroys
the pursuit: 288 of 365 chases ended on their first tick. The yield went through the loss, left the memory of a
mover, and the next sighting re-acquired and yielded again, a stop and a look per cast; the walls it bought came
with twelve extra standing looks a run. And the yields fire at the chair beside the track's western leg (its back
0.3 m from the passing train), not at the track's ends, which run 0.45 m from the walls and beyond the 0.35 m radius.
`PARTIAL` on the walk (an artifact of the stops), `REGRESSION` on the pursuit; the mechanism as built was a weakened
slice. R108 fixes the build: a yield forgets (no look, no memory) and the yielded place is not-a-mover for 5 s.

**The yield that forgets (R108, §17.73, n = 18):** the churn gone (37 yields, 74 dropped sightings, the median chase
a second long), the yields at walls and furniture and none at the moving train; walls 27 → 19 a minute — the
stack's level with R99's priority kept, R99's two trapped runs gone (one of R108's own trapped by the walk's corner
failure, 3 stuck escapes that did not free it); the pursuit four fifths of R99's (93 chases against 117, the train
27 against 35, the same share) because a silent yield walks on where R99's lost look had found the train again.
`PARTIAL`, the first arm that holds both the walk and the pursuit. Next: the yield that LOOKS (R109), measured in §17.74.

**The yield that looks (R109, §17.74, n = 6):** the campaign's purest pursuit (19 of 32 chases at the moving train,
20 started from the look after a yield) and a `REGRESSION` on the walk (31 walls a minute against 11): two runs of six
trapped by the walk's corner failure from the third minute, their episodes not following the looks. Killed at n = 6;
retry once the corner trap is solved. R108 stays the arm.

**The yield's reach (R110, radius 0.5 m, §17.75, n = 6):** more yields, four times the dropped sightings, the chase
cut to a third of a second, the walk worse (11 → 18) and the yields still not at the train's ends. Killed. **The
campaign's arm is R108**; the floor under every arm is the walk's corner trap (one or two runs in eighteen of each
arm, the stuck escape does not free it), and that is the next walk lever. Readout: the artifact above.

**The walk under the loops (§17.76, the operator's eye on R108 seed 1, 2026-09-29 night):** every "attended but
ignored" case is the body not walking to the target the seek loop held (92 % of the run): the range closes on 32 %
of walking seconds, the yaw command's sign agrees with the reference half the time. The heading reflex (§17.37,
O57) on the campaign's arm: agreement 79 %, the heading error halved, closing 61 % where the field is clear — and
released within a metre of the thing (a ToF hit inside its gate), so the last metre orbits and the walls come
back (19 → 28 a minute). `NULL` on the closing, `REGRESSION` on the walls. Next: the seek gate covering the
reflex's release (§17.77). Instrument: `mj_host/tools/walk_closing.py`.

**The reflex with the seek gate (§17.77, n = 18):** the walk goes where the loops point — closing 32 → 40 %, the
heading error 1.2 → 0.5 rad, arrivals 5.4 → 8.3 a run, stuck stops 36 → 26, no run past 62 walls a minute — and
brushes the walls beside the things it now reaches and en route (median 12 → 30 a minute, 15 s a run in contact).
`PARTIAL`; the arm for the eye on the operator's criterion: preset "R108 · + the heading reflex + the SEEK GATE".
Next levers: a static target that yields near tall structure (one parameter on the published count), the arrival
radius from the thing's edge, the nearer attended thing replacing a held target.

**The static yield (R111, §17.78, n = 6):** the things by the walls are dropped (139 in six runs) and the walls
rise with the rescues (20 → 27 a minute, 0.12 → 0.35): the wall contact was never the wall-adjacent targets, and a
loop without a target hands the walk to play. Killed. The neighbour count cannot tell a wall fragment from a thing
(13 vs 18 columns within 0.3 m); a row rule needs the line.

**The free-space gate on the reference (§17.79, n = 6):** a tie on the walls, a cost on the closing (40 → 33 %); killed.
The long wall bursts are a forward stall at half range the stuck detector does not count (its bar is three quarters
of range); `--stuck-cmd 0.4` is measured in §17.80.
**`--stuck-cmd 0.4` (§17.80):** two more stuck stops in six runs, the walls not better; killed. The long bursts are
a held target beyond the wall; next, the seek loop forgets a target the walk does not close on (§17.81).

**The progress forget (R112, §17.81, n = 18):** the seek loop forgets a target the walk does not close on (half a
metre walked, under 5 cm gained): closing 40 → 48–55 %, opening 8 %, play never holds the heading with a target
held, arrivals up — the loudest move on the operator's criterion. The walls tie with two trapped seeds, traced to
two defects (the renewal re-arming a forgotten place every tick; the stuck detector firing once per stall), both
fixed and measured in §17.82.
**What a forgotten place may refuse (§17.82):** barring the renewal empties the loop (a target held a quarter less,
closing 72 %, walls doubled under play); the renewal is load-bearing for engagement. The as-built progress forget
with the stuck re-arm is the arm (§17.83).

**Where the night ends (§17.83, n = 18):** the arm on the operator's criterion is R112 (the progress forget) on the
reflex and seek gate: closing 48–55 % (R108: 32), opening 8 % (27), play never holds the heading with a target
held, arrivals nearly triple, the heading error a third. Walls tie the eye's arm at 30 a minute (R108: 19) with two
traps in eighteen where the body pushes at a surface its ToF reads as empty at contact. Every escape-side wall
lever was null; the floor is a sensor's (a return at contact), the next lever. Preset "R112 · the PROGRESS forget".

**The contact consumers (§17.84, sweeps 39–52, n = 18 each):** the ToF's too-close share is the contact sense
(1.0 at every trap, the range slots empty) and it is confirmed as an observation; four consequences imposed on it
— the stall stop and escape, the reflex's release at contact, the cloud filing contact, the target dropped at
contact — tie or regress on the pooled walls, each returning the body to the wall by another road. Learned
cooperates, imposed fights: the walker carries the share as sense slot 15 and its restored identification never
saw a push. Next: a babble that includes contact, with the contact stall kept as the instrument. The arm stands:
R112 on the reflex and seek gate.

**The contact regime (§17.85, 2026-09-30, n = 18):** the walker's avoidance is a state prior descended through its
model's authority, and the model babbled in the open never saw a push. A 1 m room, the current identification
loaded, 600 s of 6 s babble pulses with learning on, saved: the contact brain, loaded into the playroom, halves
the time the near field is full (30 → 16 s a run), cuts wall contact 27 → 16 s, the longest burst 77 → 17 s, walls
31.5 → 24 a minute, closing held, arrivals up. The prior on the contact slot (R113) adds a tail and a six-seed
signal; the prior on the unlearned brain doubles the walls; a 1200 s babble loses the walk. `WORKING` — the first
wall lever of the campaign that is not a tie. Arm: preset "R113 · the CONTACT prior on the CONTACT brain".

**The head as the walker's motor (§17.86, 2026-09-30):** the walking policy takes the four head commands and was
trained on them; given to the walker's identification, the head becomes a steering motor: turns toward a large
heading error three to five times faster, closing 52–62 %, a fifth faster, half again the arrivals — the circling
was the three-motor command's radius. Head yaw is held sideways by the controller whatever the prior's mask
(the gaze stays the head brain's); the lean (neck pitch, head pitch, head roll) keeps the gains with the gaze
straight. Every form falls four times as often as the control, backing up with the head pitched; amplitude,
smoothing, a tilt objective and an open-first identification did not move it. `PARTIAL`; the falls are the next
lever. Presets: "BABBLE ROOM · SEVEN motors", "R113 · SEVEN motors". The arm stays R113 on the three-motor
contact brain.

**The hand-off (§17.86, the operator's eye on the seven-motor walk: expressive, alive, the seeking more accurate,
and the head ringing on the pitch axis at the moment of standing with backward falls):** 9.5 of the lean's 11.7
falls a run came within 3 s of a stop's start. A servo-rate slew across the ownership change halves the ringing;
the head coming HOME first at every stop, the gaze taking over from level, returns the falls to the three-motor
walker's rate (2.7 against 2.0 on eighteen seeds) with the lean's gains kept (turn +70 %, arrivals +38 %, closing
53 %) and walls the cost (28 against 21). `WORKING` on the request, `PARTIAL` overall; the eye decides the walls
against the aliveness. Preset "★ R113 · SIX motors, the LEAN, the head home at stops".

**Yaw only (sweep 80):** the head parked sideways on 91 % of the walk, no steering gain, walls doubled (the
walker's ToF slots read the head's frame). Head yaw is not a walker motor. The still three-motor walker is the
arm for the ToF; the lean the expressive alternative; looking around on the walk is a gaze lever that first
needs the walker's ToF slots in the body frame.




**The arms** (R94 on the loaded brain is the base; each arm one lever): `--stop-on-stuck 8 --stuck-escape 6` (the
things phase's corner answer); R95 `chase_permanence_ticks 150` (a lost mover kept moving in mind for 3 s,
re-acquired where predicted, the look when it runs out); R96 `chase_pull_decay 0.6` (a chase's need decays at
every loss); `--head-forward 0.15` and `--body-pitch 0.1` (the speed levers). Measured in §17.60.

## 9. Where we are (2026-09-30 evening, for a cold start)

**Read first:** this section, then design doc §17.72–17.86 (each sweep's record, one lever at a time), then §8 above.

**The arms, and what each is for.** Every arm is a launcher preset that mirrors its harness line; the eye is the
promotion gate.
- **★ R113 · the CONTACT prior on the CONTACT brain** — the campaign's arm on the operator's criterion (a thing of
  interest is walked to, not past) with the still head the ToF wants. Config `a1v2_r113_contact_prior.json`,
  checkpoint `duck_contact_s1.brain.json`, host line = the campaign base (`--stop-on-stuck 8 --stuck-escape 6
  --chase-vx 0.35`) + `--heading-reflex 1.0 0.3 1.0 --seek-gate`. What it carries: the progress forget (R112,
  a target the walk does not close on is forgotten, §17.81–17.83), the heading reflex with the seek gate covering
  the reflex's release (§17.77), the mover's priority and the yield that forgets (R99/R108, §17.65–17.73), the
  contact-babbled identification (§17.85) with the too-close share in the walker's prior.
- **R108 · the yield that FORGETS** — the arm on the wall metric alone (19 walls/min, n = 18).
- **R113 · SIX motors, the LEAN, the head home at stops** — the expressive alternative (§17.86): neck pitch, head
  pitch and head roll as the walker's motors, `--intent-head 0.4 --head-slew 1.0 --head-home 1.5`, checkpoint
  `duck_contact6_s1`. Turns toward a large error +70 %, arrivals +38 %, falls at the three-motor rate, walls +35 %,
  the head pitched or rolled on a quarter of the walk. The BEFORE presets beside it reproduce the resonance.
- **BABBLE ROOM** presets — the contact regime itself (`--rebabble 600` in the 1 m room, `--save-brain`), three
  and seven motors.

**Numbers to know (n = 18 unless said).** R108: walls 19, contact 30 s a run. Reflex + seek gate: closing 32 → 40 %.
R112: closing 48–55 %, play never holds the heading with a target held, arrivals ×2.7 over R108. The contact
brain: contact 30 → 16 s, the longest wall burst 77 → 17 s, walls 31 → 24. The lean, n = 18: turn 0.06 → 0.10
rad/s, arrivals 10.4 → 14.4, falls 2.0 → 2.7, walls 21 → 28.

**Instruments** (all in `mj_host/tools/` or the session scratchpad noted in memory): `walk_closing.py` (does the walk
go where its loops point: closing share, heading error, yaw-command agreement, attended-not-taken, near-misses),
`yield_where.py` (where chases start, in the world), the host's summary lines (chases, yields, forgets, contact
stalls, the walker's authority table over its ToF slots after the restore and at the end); scratch: `turn_readout.py`
(the turn toward a large error, the error closed within 3 s), `stop_readout.py` (falls at stops, the head's ringing),
`head_readout.py` (the head's pose and speed on the walk), `seven_readout.sh`, `contact_readout.sh`.

**Open levers, in the order the record suggests.**
1. *Looking around while walking* (the operator's wish, §17.86 end): rotate the walker's ToF summary slots from the
   head's frame into the body's (the walker's avoidance reads the head's view; a yawed head doubled the walls),
   THEN a head-brain yaw sweep on the walk (the stop's gaze sweep exists; a walk form does not). Head yaw is not a
   walker motor: parked sideways in four forms.
2. *The lean's falls-versus-walls*: the hand-off is solved; the walls it brushes (28 vs 21) are the eye's call.
3. *The room's dose* for the contact regime (600 s in a 1 m room is the dose; 1200 s loses the walk; the room
   erodes the forward row — identify in the open first for a from-scratch brain), and the prior's share at n ≥ 20
   varied worlds.
4. *The pursuit*: the chase's start is the standing limit (§17.68); permanence in pursuit unbuilt (O63).
5. *The walls' floor*: the corner trap where the ToF reads empty at contact is now a learned avoidance's business
   (the contact brain), not an escape's — the four imposed consumers were null (§17.84).

**Traps recorded this campaign (memory has them too).** A forget or yield in the seek loop that also bars the
outcome loop's renewal empties the loop and hands the walk to play (§17.82). `pkill` patterns must be bracketed so
they do not match the calling shell; a killed chain script loses its `sweep done` marker. The host runs from
`mj_host/`, so `--load-brain checkpoints/...`. `--head-rate` is an older head-brain flag; the slew is `--head-slew`.
A restored brain will not babble again unless `--rebabble S` reopens the per-leg counter. The ToF's too-close share
is confounded under a pitching head (the floor enters the near field): read wall contact and bursts for head arms.

**The readout for the eye:** <https://claude.ai/code/artifact/10ffc001-5858-47af-98a1-973dc9bba9ac> — eighty sweeps,
every arm's line, and the seven looks the operator took, each with what it found.

## 10. The head: a lean that settles, a head that holds still when it matters (2026-10-01 →)

**The operator's direction (2026-09-30 night, watching the six-motor lean before the hand-off fix):** the nod as the
duck comes to a stop and the head's motion on the walk are interesting and give the duck personality, and a nod
toward a small thing on the floor fits its interest in it, as long as nothing falls. But the brain must also be
able to hold the head still when it needs a stable voxel cloud on the walk. Build that as a learned lever: the
head free to move and ring while looking around, and able to settle when the percept needs it. Their forks: **the
six-motor lean** carries the free head, **the sensor-realism lever** goes in, and first **the lean's own habit**:
R113's lean walks with the head pitched down going forward and up going backward. An inverted pendulum leans into
an acceleration and comes back once the pace is steady; the head should too — addressed in the small babble room
first, then observed in the playroom. **Grow-on-restore** for the new sense (keep the brain, identify the new rows
in a second babble).

**The plan, in the rewrite rule's terms.**
- *Stability is precision, not a behaviour.* The head holds still to the degree some loop needs its percept: a
  prior on percept surprise whose precision rises while a loop consumes the cloud (a held target, a chase) and falls
  to zero when nothing is attended. The pigeon's head-bob is the biology (the hold phase locks the head in space
  while the body walks under it; the bird sees then).
- *The nod as an epistemic act.* A ToF zone is 5.6°; a 4 cm thing at 0.5 m subtends 4.6°, under a zone. A few degrees
  of pitch across it moves the zone boundaries across the thing — dithering, the peering of birds and insects before
  a strike. The error the nod descends is the attended thing's uncertainty (the kind EPM's TLE).
- *The sensor check changes the order.* In simulation the ToF casts instantly from the forward-kinematic pose into
  the gravity-levelled trunk frame (`Tof.hpp`), so head motion costs the cloud nothing; the real sensor integrates
  over time and its pose lags. The costs that exist now are the walker's ToF slots in the head's frame, the attended
  thing's flicker, and possibly the mover gate's false alarms.
- **Stages.** **A** measure the lean as it is (`mj_host/tools/lean_readout.py`). **B** the head's attitude as a walker
  sense, grown on restore, identified by a rebabble in the room. **C** a head-level prior at a swept precision (the
  mid-ranging trade: the twist carries the steady speed, the head the transients), in the room, then n = 6 in the
  playroom, then the eye. **D** the sensor-realism lever (the cast over the sensor's integration window and the
  pose's lag). Then stability as gated precision, then the nod as dithering at arrival stops, the peck last.

### 10.1 Stage A, measured on the saved sweeps (2026-10-01)

`lean_readout.py` over walking ticks: the head's attitude pitch (−hg[2], + = nose down), forward speed and
acceleration through a 0.5 s box (the stride out). Seeds 1–6, 600 s, the playroom:

| | 3 motors (sweep 63) | the lean, before the fix (73) | the lean, head home at stops (77) |
|---|---|---|---|
| head pitch at a forward cruise (\|a\| < 0.06 m/s² for 1 s) | +0.00 rad | **+0.31** | **+0.52** |
| accelerating · decelerating | +0.01 · −0.01 | +0.36 · +0.18 | +0.55 · +0.35 |
| at 0.2–0.3 m/s, median | — | **+0.79** (45° down) | +0.79 |
| backing (v < −0.03 m/s): median · p10 | −0.01 | +0.10 · **−0.36** (up) | +0.31 · −0.16 |
| pitch = … + b·v + c·a | b 0.02, c 0.03 | **b 1.72**, c −0.01 | **b 1.45**, c 0.14 |

The operator's reading, measured: the lean follows SPEED, not acceleration (0.17 rad of pitch per 0.1 m/s, nothing on
acceleration once speed is in); 45° down at the fast walk; a head-up tail on a fifth of the backing. The still-head
walker holds the head level (sd 0.014 rad).

**The cause, read in the saved brain (`duck_contact6_s1`).** The walker's forward-speed prior (sensed vx → 0.75 of
range, 0.30 m/s) descends through all six motors in proportion to the authority the babble found: head pitch and neck
pitch move forward speed about as much as `vx` does (+0.010, −0.013 against +0.015). The prior's tonic half (`h`) is an
integrator, and the head motors' tonics sit at the rails (neck pitch `h` −2.15, tanh −0.97; head pitch +1.14). Two
reasons nothing takes them back: the speed target is above what the walk reaches (0.30 against 0.20 m/s), so its
error never closes; and **the walker cannot sense its head** — its state is the twist's three joints and sixteen sense
slots, none of them the head. No error anywhere reads "head down". (The head brain's own four-motor H2 showed the same
null-space drift: the two pitch joints wound to their rails, `head2_h2_level_slow.json`'s description.)

**Three instrument and faithfulness catches (§3.2), found reading the state's layout.** The state is
`[pos, act, delta]` per twist joint, then the sense (16 slots from state index 9):
1. **The §17.86 "tilt prior" on state 3 and 4 is sensed lateral speed and the `vy` command**, not the trunk's gravity
   (state 9 and 10). It rides in the six- and seven-motor configs (`a1v2_r113_six.json`, `…_seven_tilt*.json`) and in
   the starred six-motor preset; §17.86's "the tilt prior halves contact" is a lateral-speed prior's result. Kept as
   is in the arms below (one lever at a time); the re-use is a real tilt prior on 9, 10 (11, 12 with the head sense).
2. **The host's authority table printed rows 1 and 2 as "sensed vy" and "sensed wz"**; they are `vx`'s action echo
   and delta (the real rows are 3 and 6). §17.86's room table's "sensed wz" row is therefore `vx`'s delta. Fixed
   (stderr only), with the columns now named from the graph's action topics (the lean's sixth column, head roll,
   printed as `head_y`).
3. **The prior's model-implied step (`state_prior_step_gain`, on in every intent config) ignores
   `state_prior_motors`**: the mask arms of §17.86 (sweeps 70–72) masked the slow descent and not the per-tick step,
   a weakened slice of the mask. "Head yaw held sideways whatever the mask" stands for the descent only; sweep 80
   (yaw the only head motor) is unaffected. Not fixed here (it would move the six-motor arm); a lever of its own.

### 10.2 Built (2026-10-01), all off by default, both guards byte-identical

- `MotorEPMv2.state_prior_weights` — a per-index precision on the prior, in the descent (C and h) and in the
  model-implied step (a weighted least squares, each row and its error by √w). Empty = byte-identical.
- `MotorEPMv2.state_grow_at` — **grow on restore**: a restored module whose state arrives wider than its snapshot
  inserts the new elements at this index (the model's rows, the controller's columns, the state model's rows and
  columns all zero: an unidentified sense) instead of dropping every frame. −1 = off.
- Host `--intent-head-sense` — the head's roll and pitch (head-frame gravity y, z; 0 = level, the head brain's own
  level error) at the FRONT of the walker's sense, so every negative prior index keeps its element; the graph
  declares `load_slots 18`.
- `playroom_gen.py --babble-room --half 1.0 --out scene_babble_room2m.xml` — the 2 m room (a post, a box, a chair) for
  the rebabble and for watching a lean settle (the 1 m contact room gives a forward pulse two seconds).
- Configs `a1v2_contact_room6h` (the rebabble), `a1v2_r113_six_h` (the head sensed, no level prior: the control),
  `a1v2_r113_six_level03 / 10 / 30` (+ the level prior on state 9, 10 at precision 0.3 / 1 / 3 against the speed
  prior's 1).
- Guards: R83 plain (300 s, seed 3) `6b9a0b3a…` as before; the starred six-motor preset (150 s, seed 2) old build
  against new `915e1391…` both.

**The rebabble (stage B):** `duck_contact6_s1` loaded into `a1v2_contact_room6h` with the head sensed: the state grew
25 → 27, then 600 s of the structured babble in the 2 m room (2 rescues), saved as `duck_contact6h_s1`. The head's
pitch row after it: `vx` −0.021, `vyaw` +0.021, neck pitch +0.028, head pitch −0.016, head roll −0.024; the roll row
+0.025 on the head motors. Head pitch's authority over sensed speed fell to zero in the rebabble (neck pitch keeps
−0.012). Preset "LEAN · the REBABBLE with the head SENSED".

### 10.3 Stages C and D, measured (2026-10-01; design doc §17.87)

- **The level prior at 1** (the head's attitude → level, at the speed prior's precision) on the rebabbled brain:
  falls a run 3.3 → 1.1 pooled over eighteen seeds (confirmed on seeds 7–18: 3.5 → 1.2, at stops 2.6 → 1.0), the
  lean off speed and on acceleration (0.39 rad down accelerating, 0.17 at a cruise, 0.18 decelerating; the old
  brain 0.55 / 0.52 / 0.35), the head's motion halved. Costs: closing 52 → 41 %, the turn toward a large error
  slower, the walk a sixth slower — head pitch was a steering motor. `PARTIAL`: the eye decides.
- **The head sense alone** (no prior): `NULL`. **A reachable speed target** (the rail): `NULL` as a probe.
- **The ToF's real timing** (`--tof-real 0.066 0.03`, assumptions: continuous mode at 15 Hz, a 30 ms readback): the
  registration error on the walk is 4.7 cm for the still head, 5.4 for the lean, **10.7 for the level lean**, and it
  follows the head's angular SPEED (2 cm below 0.3 rad/s, 7–13 cm above 2), not its attitude. The level prior holds
  the attitude by moving the head; the still head is carried at 0.6–2 rad/s by the gait's sway.

**What this changes in the plan.** The stability stage's error is the head's angular rate in space (a gaze held
still, the pigeon's hold phase; the head IMU senses it; the head brain's H2 "still" slots and the H line's VOR
exist), with its precision gated by when a loop needs the cloud — not "level". Level is the view's and the balance's
error, and it is what cut the falls. The next levers, one at a time: (1) the level prior's precision gated by the
loop (relaxed while turning toward a target, full at a cruise and at stops) to keep the falls and give the steering
back; (2) the head's angular rate as the walker's sense and its prior, gated by the seek or chase engagement, measured
on `tre` under the real timing; (3) the nod as dithering at small things once (2) exists.

Presets: "LEAN · ROOM · …" (the old brain, the sensed control, level 1, level 3), "LEAN · PLAYROOM · …" (level 1, the
control), "LEAN · the REBABBLE …", "TOF REAL · the still head …", "TOF REAL · the lean + the LEVEL prior …".

### 10.4 The bird's neck (2026-10-01; design doc §17.88)

The operator, watching the level lean: the head nods fore and aft and unsettles the walk; move it over the body with
the two pitch joints like a walking bird instead. The joints tilt the view equally and oppositely, so moving them
together slides the head (0.76 cm of the robot's centre of mass at 0.44 rad) with the view unchanged; the nod was the
level prior chasing the stride's bob. Built: the walker's fourth motor `action.head_fore` (`--intent-head-translate
0.6`, the policy told, the head brain keeping the tilt) and its sense (`--intent-fore-sense`); identified by the recipe
(`duck_fore_s1`, control `duck_ctrl3_s1`). Over eighteen seeds: the view is the still head's and so is the cloud's
registration error under the real timing (4.8 cm against 4.7; the level lean 10.7) — **the nod is gone**. Without a
centring prior the head reaches forward into an acceleration (+0.12 rad) and drifts back at a cruise (−0.38); closing
50 → 60 %, the walk +8 %, falls 2.0 → 2.6 (`PARTIAL`). With the centring prior at 1 the head is centred at a cruise and
everything ties the control (`NULL`). Next: the centring prior gated by the pace — on at a steady pace, off while the
speed error changes — to keep the reach and drop the drift. Presets "BIRD · ROOM · …", "BIRD · PLAYROOM · …".

**The pace-gated centring (§17.88, n = 18):** the centring prior gated by the pace (`state_prior_gated_by`) at 1 keeps
the bird's steering (closing 63 % against the still head's 50) at the still head's falls (1.9 against 2.0), the cruise
drift cut to −0.10 rad, the walk +6 %, walls +5 a minute; the head slides back as the duck brakes and backs, and does
not reach forward into an acceleration (the walker credits a forward head with slowing). The candidate for the eye:
preset "BIRD · PLAYROOM · the bird's neck + the PACE-GATED centring at 1 (the candidate)".

## 11. Where we are (2026-10-01, for a cold start): ★ BIRD, the bird's neck

**Read first:** this section, then §10 (the head's phase in order), then design doc §17.87–17.88.

**★ BIRD — promoted on the operator's eye, 2026-10-01:** "this method is promoted. The robot's use of its neck is an
overall win on multiple fronts, including object seeking, voxel cloud clarity and escapes." Preset "★ BIRD · the
bird's neck + the PACE-GATED centring". What it is: everything ★ R113 carried (the progress forget, the heading reflex
with the seek gate, the mover's priority and the yield that forgets, the contact-babbled identification with the
too-close share in the prior), on a walker with a fourth motor that slides the head fore-aft —
- `action.head_fore` moves neck pitch and head pitch TOGETHER, which leaves the view's attitude unchanged (forward
  kinematics: the two joints tilt the view equally and oppositely) and moves the robot's centre of mass 0.76 cm at 0.44
  rad; host `--intent-head-translate 0.6` (rate-limited at 1 rad/s, centred at stops, the walking policy told), written
  on top of the head brain's joints — the head brain keeps the tilt (`--head-joints`);
- the walker senses where the head sits fore-aft (`--intent-fore-sense`, load_slots 17);
- a centring prior on that position (state 9 → 0) gated by the pace (`MotorEPMv2.state_prior_gated_by`: on while the
  sensed speed holds, off while it changes), config `a1v2_r113_fore_g10.json`;
- identified by the recipe: 600 s of babble on the open playroom, 600 s in the 2 m room (`scene_babble_room2m.xml`),
  checkpoint `duck_fore_s1.brain.json`; the host line = the campaign base + `--stop-on-stuck 8 --stuck-escape 6
  --chase-vx 0.35 --heading-reflex 1.0 0.3 1.0 --seek-gate --intent-head-translate 0.6 --intent-fore-sense`.

**Numbers (n = 18, against the still head identified by the same recipe, `duck_ctrl3_s1`).** Closing on the target 63
% (50), the heading error closed within 3 s 41 % (33), falls 1.9 (2.0), the walk 0.183 m/s (0.173), walls 34 (29) a
minute; the view's pitch sd 0.03 rad (the still head's); the ToF's registration error under the real frame timing
4.8–4.9 cm (4.7; the level lean of §17.87 10.7). The head: centred at a steady pace (−0.10 rad), sliding back as the
body brakes (−0.13) and backs (−0.22); no forward reach into an acceleration — the walker identified a forward head as
slowing this walking policy. Before it: the six-motor lean's head followed speed (45° down at the fast walk), the level
prior cut its falls to a third and nodded at the stride (§17.87).

**Built this phase** (all off by default, guards byte-identical): `MotorEPMv2.state_prior_weights` (per-index
precision), `MotorEPMv2.state_grow_at` (grow on restore), `MotorEPMv2.state_prior_gated_by` (the pace gate); host
`--intent-head-sense`, `--intent-head-translate F [RATE]`, `--intent-fore-sense`, `--tof-real SPREAD LAG` (the ToF's
real frame timing, the `tre` field); `scene_babble_room2m.xml`. Unit tests in `test_state_prior.cpp` (26).

**Instruments.** `mj_host/tools/lean_readout.py` (the view's pitch or, `--channel fore`, the head's fore-aft position
against speed and acceleration, by phase, and the settle after an acceleration), `mj_host/tools/head_modes.py` (the
head's motion split into translation and pitch, each mode's stride-band part: what nods), `mj_host/tools/tre_readout.py`
(the registration error under `--tof-real`, by the head's angular speed), with `walk_closing.py` and the host's summary
lines as before.

**Traps recorded this phase.** The walker's state is `[pos, act, delta]` per twist joint, then the sense from index 9
— the six/seven-motor "tilt prior" on state 3, 4 was lateral speed; the prior's model-implied step ignores
`state_prior_motors`; `--head-forward` moved the head back; the neck servo sags ~0.14 rad under an extended head (the
head brain levels it out); a 150 s guard run diverges from a 600 s sweep log where the host declines a stop with less
than the stop's length left — compare up to that tick; the walker's sense insertions go at the FRONT of the load block
so negative prior indices keep their elements; grow-on-restore handles new senses, not new motors (a new motor set is a
new identification).

**Open levers (the next move is planned from here).** Looking around while walking (the walker's ToF slots in the
body frame first, then a gaze sweep on the walk — the head brain now owns the tilt and yaw on the walk, the walker the
translation); the head still in space for the cloud (the gaze's angular rate, the H line's VOR, judged on `tre`); the
nod or the neck's reach toward a small thing at an arrival (the bird's peck is a neck extension); a sharper pace gate;
the prior's step honouring the motor mask; the pursuit's start (§9).

**The gaze (2026-10-01, design doc §17.89).** The walker's ToF slots taken in the body frame from the last 0.5 s of
returns (`--tof-body 0.5`) let the head look away without turning the avoidance; on a walker identified with them
(`duck_forebody_s1`), the head turning toward the seek loop's target on the walk (`--seek-gaze 1.0`) keeps the target in
view 86 % of seeking time (43), turns toward it five times as fast, closes the error within 3 s 46 % (34), falls 1.7
(2.2), walls 22 (24) — `WORKING` against its own control, n = 18; the candidate for the eye is preset "GAZE · the
body-slot brain + the GAZE leading the turn". On the column-slot brain the same gaze lost (a model mismatch).

**The learned gaze (2026-10-01, design doc §17.90).** The head brain senses its gaze error (where the walk is going
minus its yaw) and a prior drives it to zero: after four diagnosed failures (a feedback gain grown on a large error; a
pure reach that is a velocity under the controller's identity hold; the level prior's tonic leaking into yaw; fixed by
re-identifying with yaw last and masking the level priors), it looks — the target in view 90 % against 45 %, the yaw
smooth — and does not yet help the walk (walls 33 against 24, falls 2.6 against 1.8). `PARTIAL`; the reflex stays the
better walker. Preset "GAZE · LEARNED …".

**The follow-ups (§17.90, n = 18):** the stop slew takes the falls at a stop's start 0.6 → 0.2 (under the control's 0.3)
and lifts arrivals; the proportional term at 0.03 brings walls to the control's level, at 0.1 the yaw thrashes again;
the two together keep neither gain. The learned gaze stays `PARTIAL`; the reflex stays the better walker. Next form to
try: the model's own one-step correction (`state_prior_step_gain`) as the proportional term.

**★ GAZE promoted (2026-10-01):** the learned gaze on ★ BIRD's body-slot walker, on the operator's eye ("working well, an
interesting behaviour"). Preset "★ GAZE · the LEARNED gaze". The next push is the turning radius (design doc §17.91): the
walking policy's in-place yaw deadband, a walker yaw loop that limit-cycles, and a heading tonic that winds.

**The turning radius (design doc §17.91–17.92):** the walking policy has an in-place yaw deadband (no turn below a ~1.25
rad/s command) and the walker's range sat inside it. Calibrating the yaw motor (`--yaw-linearize`: the walker's yaw a
desired rate, the deadband compensated from the measured response) and re-identifying the walker on it (`duck_lin_s1`):
on ★ GAZE's stack, n = 18, the time with the target well off the nose halved, the error closed within 3 s 40 → 55 %,
walls 33 → 23 a minute, falls 2.6 → 2.9 — `WORKING`. The heading as a reach ties it; precision 10 regresses. Next: the
speed prior gated by the heading error, so the walker stops and turns in place. Preset "TURN · ★ GAZE on the LINEAR yaw motor".

**The walk that faces its thing (design doc §17.93):** the operator saw the linear yaw approach well and kick and peck
less accurately — it arrived still turning, the thing 61° off the nose at the skill. The forward-speed prior's target
gated by the heading error (`state_prior_target_gated_by`) makes the walk slow, turn in place and back to face the thing:
n = 18, the thing 22–27° off the nose at the skill (★ GAZE 41–46°), the head touching the thing on 57 % of pecks (32 %),
walls 14 a minute, falls 1.5, the error closed within 3 s 70 %; arrivals −13 %. `WORKING`; the candidate for the eye.
