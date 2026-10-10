# Porting the MicroDuck's autonomous layer to the PiCrawler

**Status: reviewed by the operator 2026-10-10; S0 done (master merged, PR #39). S1 next.**

**What this decides.** How the exploration and wall-avoidance machinery built for the
MicroDuck (branch `master`, `docs/plans-and-designs/microduck/`) becomes a level above the
PiCrawler's walking brain (P-e·h0, branch `picrawler-dev`), without touching the gait that
already walks the robot. The PiCrawler has a working heading hold, a forward ultrasonic
rangefinder, a camera, and a stride odometer. The duck has a measured stack of loops that
decide *where to walk*. The port is the duck's level 2 on the PiCrawler's level 1.

The operator's observation that motivates it: lifted mid-walk and turned 90°, the robot swings
back toward its original heading. That is the heading PD on the gyro-integrated own-yaw
(`heading_bearing_hold_gain 7`, `heading_hold_gain 0.3`), and it is exactly the actuator the
duck's loops needed and never had: the duck's RL walker had a yaw deadband and needed a host
reflex and a calibration table to turn at all. The PiCrawler already has the turn. What it lacks
is anything that *sets the heading*, and anything that sees a wall before the body hits it.

---

## 1. Where the two bodies stand

### 1.1 The PiCrawler (this branch)

- **Level 1 is done and deployed.** P-e·h0 (`…__honest__nohomeo.json`) walks the robot and
  looks like the sim. Its only steering input is `goal_bearing_topic`: an egocentric unit
  vector `[vx, vy]` that replaces the heading PD's error. Forward thrust has no socket at all:
  `fwd` is a constant 1.0 unless the oracle `nav_topic` is on (`MotorEPMv2.cpp:4117`).
- **The forward ultrasonic is fitted, driven and unused.** `pi_host` publishes `sense.range`
  `[distance_m, valid]` at 20 Hz (max 1.5 m, no echo past ~1.3 m or at glancing angles). It
  runs only under the senses service, which `dash_run.py` stops before every brain run, so
  the brain has never seen it. The sim has no model of it. Mount height and pitch are
  unmeasured (BOM §8 item 6).
- **The camera is fitted, driven and unused.** The robot publishes a 32×32 luma crop on
  `sense.camera` at 15 fps; the sim publishes 32×24×3 RGB on `host.video.color` (off by
  default) and computes a depth field it never publishes. Only `picrawler_senses.json`
  consumes either, and that config cannot move the robot.
- **Odometry exists in pieces.** `stride_v` (stance-leg kinematics fused with the IMU) and
  `ego_heading` (gyro dead reckoning) are both egocentric and legal. The sim publishes
  `ego_heading`; the robot does not. No host integrates them into a pose.
- **Neither gym has a vertical wall.** The arena has 45° edge ramps and low pyramids; the
  corridor has 30° self-centring walls. The ledger's `stuck→explore` lever was refuted on
  flat ground with the re-use context "terrain, corridor corners, obstacle contact". That
  scenario does not exist in the sim yet.
- **The modules the duck built are not on this branch**: the state-prior extension of
  MotorEPMv2 (+1,707 lines), CloudMap, BearingSeekLoop, SkillOutcomeLoop, LoopCompetence,
  TofAvoidLoop, MotionField, PlaceVectorBuilder, `brain_builder/`, all of `mj_host/`.
  PlayLoop, EFEArbiter and LateralVoter are on both branches (modified on master).

### 1.2 The MicroDuck (master)

The duck's Track B puts the brain *above* a walker it does not own, commanding a twist. Its
promoted stack (★ F5, `la1_fact_sheet.md`) is, bottom to top:

| layer | what | where |
|---|---|---|
| host reflexes | heading reflex with proximity-weighted share; stuck detector scaled by the body's own stall median; escape to the freest sector; impeded look; arrival stops | `mj_host/src/IntentAdapter.cpp`, `main.cpp` |
| the walker brain | MotorEPMv2 at the intent level: motors = twist, senses = body velocity, IMU, heading error, map TLE, 4 ToF proximity slots. Identified by babble, then driven by **state priors** (speed target, heading error → 0, proximity → 0, contact → 0) with the model-implied step | `motor_epm_intent` |
| the place map | an RBF EPM over `[odom x, y, cos yaw, sin yaw, 8 view slots]`, inserting only at stops | `map_epm` |
| the drives | PlayLoop (climbs the map's own TLE, with habituation); BearingSeekLoop (homes on a remembered thing); SkillOutcomeLoop | `play`, `seek`, `outcome` |
| arbitration | LoopCompetence grades each loop; LateralVoter turns competence into trust; EFEArbiter in precision mode picks the bearing owner, `G = need × trust` | `comp_*`, `voter_loops`, `arbiter` |
| the senses | an 8×8 ToF reprojected into Empty / TooClose / Floor / Hit, body-frame proximity slots carried by odometry, and CloudMap (a voxel cloud built at stops) | `Tof.cpp`, `CloudMap` |

The verdicts that decide what to port (fact sheet §G):

- **Avoidance as learned priors on the walker: WORKING, loud** (R48: walls 14.8 → 1.8 a
  minute). Two prerequisites, each measured: the self-model keeps learning after the babble
  (`babble_owns_a 0`), and contact is *in* the babble. On a brain that never saw a push, the
  same prior doubled the walls.
- **Avoidance as a bearing loop (TofAvoidLoop, R28): REGRESSION** on the duck. Its yaw
  actuator had a deadband; small commanded bearings did nothing. That context does not hold
  on the PiCrawler, so the verdict does not transfer (CLAUDE.md §3.1).
- **The heading reflex with its share: WORKING on the channel, PARTIAL as a behaviour.**
- **Stuck stop + escape: WORKING** in the campaign (falls a tenth, walls −31 %).
  **Backing was the only measured exit** (`--no-backing` REGRESSION).
- **PlayLoop as a direction: PARTIAL** (more cells, fewer walls, re-exploration after a wall
  moved). Wander-by-boredom heading jumps and forced wander: REGRESSION.
- **The raw 8×8 straight into the place map: REGRESSION.** A frame is a pose code. Objects
  exist only in the swept cloud.
- **The camera's case** (§17.106): wide, low-resolution peripheral vision. Coverage comes
  from range and width more than from resolution.

### 1.3 What does not transfer

Pollen's RL walker and its calibration table; the two-leg stander and its hand-offs; the
head and everything on it (★ HEAD, ★ BIRD, ★ GAZE, the stop's gaze sweep, look-around, the
impeded look's head-up); CloudMap's voxel geometry, which needs a 64-zone depth array with a
known pose; the things phase (seek, kick, peck, outcome), which needs the cloud to find a
thing. The PiCrawler is statically stable, headless, and carries one acoustic beam.

What transfers is the **shape**: a level-2 brain whose motors are the gait's intent sockets,
whose senses are odometry plus proximity, that learns what its intents do and descends
priors on proximity and heading; a host reflex layer for the fatal cases; a place map and a
play loop over it; an arbiter that hands the bearing to whichever loop has earned it.

---

## 2. The mapping

| duck | PiCrawler | state |
|---|---|---|
| twist `[vx, vy, vyaw]` into the ONNX walker | `goal_bearing_topic` (exists) + a **`speed_topic`** scaling `fwd` (new, §4 S2) into P-e·h0 | partly built |
| sensed twist `reality.proprio.intent` | `stride_v` (2) and `gyro` yaw rate | exists on both hosts |
| leg-contact odometry `[x, y, yaw]` | a host path integral of `stride_v` rotated by `ego_heading` → `reality.proprio.odom`; `vel_ego` ← `stride_v` (the ledger's one-topic swap that makes PlayLoop legal) | sim partly; robot needs `ego_heading` published |
| heading-deviation sense `(heading − ref)/π`, ref a 60 s average | the PD's own `heading_bearing_` integrator, reference set by the winning loop | exists; see trap T4 |
| ToF 4 proximity slots `[left, ahead, right, too_close]` | ultrasonic `ahead` + `too_close` (range < 0.10 m or no-echo-and-stalled); `left` / `right` from **body-frame memory** of past readings carried by odometry (the duck's `--tof-body` trick, with the body's own yaw wander as the sweep), later from the camera's column cue (S5) | new |
| ToF 8 view slots for the place map | the same body-frame memory binned into 8 sectors; later the camera columns | new |
| CloudMap | none (no depth array) | not ported |
| heading reflex with proximity share | the heading PD *is* the reflex; the share rule moves to the arbiter: as `near` rises the hold yields to the avoid loop's bearing | adapt |
| stuck detector (stall = cmd high, sensed vx low, scaled by the body's own stall median) | identical, with `stride_v.y` as sensed vx; **plus `sense.servo_current`**, a stall observation the duck wanted and never had | new |
| escape to the freest sector; back off | the same, if the gait can reverse (a measurement, S1) | new |
| `map_epm`, PlayLoop, LoopCompetence, LateralVoter, EFEArbiter | the same modules, same configs, `heading_sign` re-measured | on master |
| the intent MotorEPMv2 with state priors | the same module, motors `[turn, speed]`, identified by babble in the walled room | on master |
| `mj_host` stop/skill choreography | none | not ported |

---

## 3. Design rules this port keeps

1. **The gait is substrate.** P-e·h0 is not edited for behaviour. Everything new enters
   through two sockets on MotorEPMv2 (`goal_bearing_topic`, `speed_topic`) and both are
   silent-is-byte-identical.
2. **Rewrite rule at every stage.** No wall-avoidance script. Avoidance is a prior
   "proximity → 0" descended through an identified model, or at the reflex level a bearing
   toward free space that the arbiter must earn trust for. Exploration is the map's own TLE
   climbed, not a random walk.
3. **Sensors enter by the three-stage rule** (memory: sensor-channel-admission): publish
   instrument-only, prove the scatter separates the states, only then wire a consumer. The
   duck's ToF studies are the template.
4. **Egocentric only.** Odometry is dead-reckoned from the robot's own stride and gyro. The
   sim may keep god's-eye truth for *instruments* (walls per minute, coverage), never as an
   input. On the robot the same metrics come from the ultrasonic too-close count, the stuck
   detector and the odom map.
5. **One lever, gain-0-guarded, seed-averaged, full metric set, watched in the UI.** The
   metric set for this layer is in §5.

---

## 4. Stages and gates

Each stage is a promote-or-kill. Nothing in a later stage is started before the earlier
gate passes.

### S0. Merge master into picrawler-dev

The duck's modules arrive by merge, not cherry-pick. The dry run (`git merge-tree`) shows
three conflicts, all append/append: `.gitignore`, the lever ledger, and `launcher.gd`'s
config list. `MotorEPMv2.cpp/.hpp` and `cpp_core/CMakeLists.txt` auto-merge; picrawler-dev's
two MotorEPMv2 levers (`explore_noise_tau`, `height_windup_guard`) survive. `mj_host/` and
`brain_builder/` arrive as pure additions.

*Gate:* **P-e·h0 is byte-identical across the merge.** Run the same seed before and after
on all three deployed configs and diff the JSONL. All 1,707 new MotorEPMv2 lines are
parameters defaulting off, but the gate cannot see what it does not run: this is the one
check that proves it. Then `cpp_core` tests (the 30 state-prior tests included), the three
`tests/body/*_parity_check` oracles, a `godot_host` build, and a `pi_host` build on the Pi.
The pre-commit identifier scan runs over 158k added lines; expect it to be slow.

### S1. The senses, as instruments

**Sim.**
- An ultrasonic model in `picrawler_body.gd`: a cone of rays (7 across ±7.5°) from the
  measured mount, min-range return, 20 Hz, max 1.5 m, `valid 0` when the nearest return's
  incidence is past the glancing limit. Published as `sense.range` in the robot's exact
  format. The glancing-angle confound is deliberate: a no-echo reads the same as open
  floor, and the brain must learn that from the `valid` flag (memory: confounds ride in
  the channel).
- **A walled gym, "the room"**: a 3 × 3 m floor with vertical walls and two or three
  boxes, flat ground (so terrain is not a confound), spawned at the centre. Keyed `[3]`
  beside arena and corridor. God's-eye wall-contact and coverage counters for the harness.
- The camera: luma 32×32 on `sense.camera`, matching the robot. The sim's RGB raycast
  already exists; the reduction to the robot's format is a parity check, not new optics.
- `reality.proprio.odom` `[x, y, unwrapped yaw]` and `vel_ego` ← `stride_v`, as a body
  helper in `cpp_core/include/ogma/body/` with a parity test, so the robot runs the same
  arithmetic.

**Robot.**
- `ogma_host` runs `--range --camera` alongside `--brain-inputs` during a dash run, publish
  only. Measure the tick cost (the I²C feed is 1.6 ms of 20; the camera pipe is a separate
  process; the ultrasonic is GPIO edge timestamps).
- Publish `ego_heading` and `odom` from `BrainInputBuilder`.
- Measure the ultrasonic mount height and pitch and record them in `calib/sensors.json`.

**The separation study** (the duck's `tof_studies.md`, rerun on this body): an EPM
instrument-only over `[range, valid]` and over the body-frame proximity memory, with the
PCA visible, driven by the heading-hold baseline walking in the room. The question is the
duck's: does the scatter separate wall-ahead, wall-left, wall-right and free? If the
single beam cannot separate left from right, S5 (the camera) moves ahead of S3's second
form.

**The actuator measurements** ("find the actuator first", the ledger's most repeated NULL):
- The open-loop turn table: step `goal_bearing` to ±15°, ±45°, ±90°, ±180° and record the
  yaw rate and settling time. The rectified differential loses about half the commanded
  turn authority at the rail (hip1 already at 56 % clip duty); this table is the
  PiCrawler's analogue of the duck's yaw calibration and sets the arbiter's reach constant.
- The stall signature: walk into a wall; record `stroke` command, `stride_v.y`,
  `sense.servo_current` and the ultrasonic. The stuck detector's threshold is the body's
  own stall-length median, not a constant.
- Does the gait reverse? `speed_topic` at −0.5: does the body back up, and at what current?
  If not, the escape is a turn only, and the duck's "backing was the only exit" is the first
  thing to re-test.

*Gate:* separation shown for at least `ahead` vs free; the turn table and stall signature
recorded in the ledger; the robot publishes all channels during a brain run with no
overruns.

### S2. The intent boundary on the gait

One lever: **`speed_topic`** on MotorEPMv2, a 1-vector in [−1, 1] multiplying `fwd` (and
the sign of the stroke if reverse exists). No message = 1.0 = byte-identical. With it the
level-1 blanket is complete: active states `[bearing, speed]`, sensory states
`[stride_v, gyro, odom, range, contact]`.

Plus the heading reference discipline (trap T4): when a loop sets the bearing, the PD's
integrator is re-zeroed to the new reference rather than accumulating toward its ±2-turn
clamp. Silent when no loop publishes.

*Gate:* `speed 0` stands still with the belly up and the CPG running; `speed 1` is
byte-identical to P-e·h0; the turn table from S1 reproduces through the socket.

### S3. Wall avoidance, two forms, A/B'd in the room

**Form A, the reflex layer** (cheap; every module exists after S0):
- A proximity-memory builder (new, small): the last 0.5 s of valid ranges stored in the
  odom frame, re-expressed in the current body frame, binned `[left, ahead, right]` as
  `1 − r_min / 1 m`, plus `too_close`. Published as `reality.proprio.prox` (4).
- `TofAvoidLoop` emits a bearing toward free space from those slots.
- The stuck detector + escape (new module, ported from `IntentAdapter`): on a stall longer
  than 8× the body's own median, stop, back off if reverse exists, hold the bearing at the
  freest sector for 6 s. Sector 0 is on the right: pin the sign with a test, as the duck had
  to.
- EFEArbiter in precision mode between `hold` (the PD alone), `avoid` and later `play`,
  with LoopCompetence grading avoid on "did proximity fall while I drove".

**Form B, the learned form** (the duck's loud result):
- A second MotorEPMv2 at the intent level: motors `[turn, speed]` → the two sockets;
  senses `[stride_v (2), yaw rate, heading error, prox (4), map TLE]` through a
  JointSensorimotorBridge. Identified by a structured babble in the room *with contact in
  it* (held pulses, one axis at a time, 600 s, `babble_owns_a 0`), then driven by state
  priors `speed → 0.75`, `heading error → 0`, `prox → 0`, `too_close → 0` with the
  model-implied step. Learning frozen during escapes and recoveries.
- The arbiter then sits above B as it does above A; B's own avoidance replaces TofAvoidLoop.

*Metrics:* walls per minute (sim god's-eye; robot: too-close count), time stuck, longest
contact burst, coverage (distinct map nodes), straightness between turns, falls, time near
the current ceiling. *Blind metric:* a body that spins at the centre scores zero walls, so
coverage is reported beside it always.

*The (d) test:* move a wall mid-run; the proximity memory must re-learn it and contacts
must fall again within a minute.

*Gate:* either form beats the heading-hold baseline on walls and stuck time at n = 6 with
coverage not down; the better form goes on. If A alone is loud, B is deferred with that
context recorded. If neither moves, the first suspect is S1's separation, not the loops.

### S4. The play loop

- `map_epm`: RBF EPM over `[odom x / L, y / L, cos yaw, sin yaw, prox view (8)]`, `L` the
  room's half-size (the duck's ±1.2 m clamp trap). Insert-on-stop is optional on a body
  with no head: start with insert-while-walking at `process_every_n_ticks 5` and A/B the
  stop rhythm later.
- PlayLoop on the map's TLE with habituation, `heading_sign` measured for this body's
  frame before the first run (the mirrored-frame trap cost the duck R27 to R63).
- `comp_play`, `voter_loops`, and `play` added to the arbiter. Precision mode:
  `G_play = (1 − near) × trust_play` against the avoid loop's `near × trust_avoid`.

*Gate:* coverage up against S3's stack with walls not up; the (d) test is "move the boxes
and show re-exploration". Watched in the UI: the operator's loud signals are heading
re-correction, obstacle-triggered adaptation, and a body that leaves the place it has
been.

### S5. The camera as peripheral vision

Not before S3 has shown the single beam's limit. The frozen encoder is geometric: per
column of the 32×32 luma, the row where the floor's texture statistics end, mapped through
the known camera height and pitch to a range. That gives 8 column proximities, the duck's
`cols8`, at the camera's full width. The ultrasonic is the validation instrument for the
geometry at the centre column, never a teacher the camera copies. Admitted by the S1
separation study rerun on the camera columns; then it feeds the proximity memory's
`left`/`right` directly and the map's view slots.

### S6. The robot

Shadow mode first: the level-2 graph runs on the Pi with its bearing and speed published
but not wired, logged against the ultrasonic and the stuck detector. Then live on carpet in
a boxed-off corner of the floor. The operator's two perturbation tests: lift and turn 90°
(heading), and set a box in the path (walls). Metrics from the robot's own senses only.

---

## 5. The metric set for this layer

| metric | source (sim) | source (robot) | blind to |
|---|---|---|---|
| walls / min | god's-eye contact | ultrasonic too-close count | a body that never moves |
| time stuck, longest burst | stall detector | stall detector + current | — |
| coverage | distinct map nodes, god's-eye cells | distinct map nodes | drift (pair with walls) |
| straightness between turns | net / path | odom net / path | — |
| falls | auto-reset | tilt guard | — |
| time near the current ceiling | sim power model | INA219 | — |

---

## 6. Traps carried over from the duck

- **T1. Read the state layout before indexing a prior.** New senses go at the front of the
  load block so negative indices keep meaning.
- **T2. Mirrored frames.** PlayLoop's `heading_sign`, the escape's sector 0, every bearing's
  `[+right, +fwd]` convention. One unit test per sign.
- **T3. The feedback half grows on step errors; a tonic under a hold acts as a velocity.**
  Anti-windup on C and h; the heading prior as "C balances, h reaches" was NULL on the duck
  and is not the first thing to try here.
- **T4. The heading integrator.** `heading_bearing_` clamps at ±2 turns and the robot never
  sends `events.reset`. A level above that sets references must re-zero it, or an hour of
  right turns rails it.
- **T5. Self-scaled gates tighten themselves.** The RMS-scaled speed gate made the duck
  shuffle; use the geometric `min(cos e, k·r/|e|)` form with k from the measured turn.
- **T6. A no-echo is not open floor.** Keep `valid` in the vector; let the EPM learn it.
- **T7. Contact must be in the babble.** A prior on a slot the model has no row for doubles
  the walls.
- **T8. Judge per seed and on the first minute.** The duck's behaviour was bimodal across
  seeds; averages hid it.

---

## 7. What this does for the power problem

The operator's one hardware issue is current when legs catch on terrain. The stuck detector
in S3 is the brain-side half of that fix: a stall that persists is detected on the body's
own scale and ended by an escape instead of pushed through, and `sense.servo_current` is
the second stall channel. The hardware half (Mod D, the servo supply) proceeds in parallel.
A later lever, an interoceptive prior on servo current (the duck's "energy homeostat" made
real), has its re-use context here but is out of this plan's scope.

---

## 8. Open questions, and the operator's answers (2026-10-10)

1. **The room.** A 3 × 3 m floor with boxes. Agreed.
2. **The ultrasonic mount.** Measured: the two transducer centres sit 11 mm above the bottom
   of the chassis and 25 mm apart, on the front face. Recorded in `pi_host/calib/sensors.json`
   and the sim's "eyes" now sit at those positions (`picrawler_body.gd`), so the S1 range model
   is cast from the right place. Pitch is still unmeasured; the beam is treated as level.
3. **Stop and look.** The stop-and-look rhythm is agreed, and the saccade method is open for
   experiment when S4 reaches it. One ruling: rotating the body with the hip1 joints while in
   the standing pose may work as the quadruped's saccade, but it is to be a **learned skill**,
   not a scripted sweep. In this plan's terms: a prior to fulfil (bring the stalest sector of
   the proximity memory into the beam), the hip1 motion emerging from it, A/B'd against the
   body's own yaw wander as the sweep.
4. **The merge.** Done: S0 landed as PR #39; master and picrawler-dev are one tree.
