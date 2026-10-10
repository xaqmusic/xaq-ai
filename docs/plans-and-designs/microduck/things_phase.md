# Microduck: the things phase. Seeking what is smaller than itself, and being surprised when it answers

Status: **closed 2026-09-27; `★ THINGS` = R83** — the R79 stack (the un-mirrored walk, the seek loop, the arrival stop, the kick / peck / push from standing, the outcome loop keyed by the KIND, the aimed unwind, the gaze at the reached thing) with R80's closer arrival, promoted on the operator's eye; lineage `★` R67 (2026-09-22) → R77 → R79 → R83 (all 2026-09-27) · Dates: 2026-09-15 → 2026-09-27 · Branch: `duck-l2` · Simulation only. **Picking the duck up cold? Start at §10.**

*The phase after the cloud ([`microduck/cloud_phase.md`](cloud_phase.md), `★ CLOUD` R56).
The operator's direction, restated in the rewrite rule's terms, then the loops that read the voxels at
three timescales, then the build order. Every verdict goes to the rung-2 design doc §17 and the
[register](../open_items_register.md) (O43, O44, and O45–O50 below), as before. Nothing here is measured
unless a section says so.*

---

## 1. The direction

The operator, 2026-09-15: the duck now makes a solid map of voxels quickly, but map-making is not the
interesting thing for it to be doing. The robot should seek out the most interesting thing in the room and
interact with it to reduce its error. The map should let it predict the place it is in and be surprised by
changes. It should see a ball as a stack of a few voxels, walk over, kick it by accident, and, standing
still, see the ball's location change and have that register as error. The goal of the phase is
interaction with objects, not navigation, even if that means stepping back from wall avoidance as the
walk's driver.

Two corrections from the discussion that shape the design:

1. **A thing does not grow on approach.** A voxel is 4 cm in the world, so a ball occupies the same voxels
   at 2 m as at 0.5 m. What changes with range is sampling. At 2 m the ToF's rays are about 20 cm apart, so
   one cast lands at most one point on an 11 cm ball and the sweep fills its voxels in over time; at 0.5 m
   every voxel gets a ray on every cast. A cluster converges from under-sampled to complete. The error that
   approach reduces is therefore the cluster's precision (hits per voxel, and whether its height, extent and
   stack read the same from one sweep to the next), never its size. A thing the duck can interact with is a
   stack of up to about five voxels, 20 cm, which is the cloud's upper break band.
2. **The head looks where the thing is.** The head brain holds the head level on the walk, and level is why
   the last half metre is blind. The fix is a gaze error, not a script: keep the attended thing centred in
   the sensor's field, in pitch as well as yaw. As the duck closes, the thing's expected elevation drops and
   the head pitches down on its own. With the sensor about 20 cm up and a 45° field, a level gaze first sees
   the floor at about 0.5 m and a gaze 20° down at about 0.2 m.

## 2. What the map is today, and what it is not

Read before designing on it. All of this is in the cloud phase's record (design doc §17.28–17.33).

- **The cloud belongs to a stop, not to the room.** `CloudMap` ignores translation on purpose (a standing
  trunk holds position to a centimetre). Each filed cloud is one standing pose, de-rotated by the duck's own
  yaw.
- **The room is a cache of eight clouds keyed by the place map's winner.** A revisit is compared through the
  odometry's rigid transform between the two anchors, and the odometry drifts 4–6 % of distance walked, so
  two visits minutes apart are aligned by a pose that may be metres out. Registering by content is open
  (O40).
- **`revisit_change` is one scalar over the whole cloud.** It cannot say which sector changed, so it cannot
  give a bearing.
- **The object vocabulary is an EPM over the whole cloud's 36-dim sector profile.** Its 16 nodes are scene
  codes, mostly pose codes (§17.29: pose alone fixes 93 % of what is in view). Nothing in the graph says
  "there is a thing at that bearing".
- **The stack rule** (§17.31) tells a small thing from a wall base or a chair leg at precision 0.53 out of
  sample, 0.92 with the sweep, at full recall. It runs offline in `cloud_objects.py` and is not a module
  output.
- **The body notices what it walks into** (O41): the joint brain's prediction residual is 2.7× the twist
  brain's on contact onsets, exposed as an EMA that cannot show a 100 ms event.
- **The orienting reflex** (R44, §17.24) turns a change at a still gaze into a stop's end and a walk toward
  it, and refuses a sweeping gaze.
- **Stops start on a timer** (every 80 s from 600 s, O43).

So the room model the direction asks for is not a new voxel world map. The predictive structure already has
the right shape: a place vocabulary plus an expected cloud per place. It lacks two things: an error
localised to a thing and a bearing, and a vocabulary of things rather than of scenes. A metric whole-room
voxel map stays an instrument for the viewer (where the world frame already exists) and not a brain input,
because with odometry registration its error would be mostly drift, not change.

## 3. The voxels at three timescales

Each timescale yields a different error, and each error is a different loop.

| timescale | the prediction | the observation | the error | what it drives |
|---|---|---|---|---|
| **within a stop** (seconds) | this stop's own cloud so far | the next cast | growth: new voxels (ends the stop, shipped); change: a return where this cloud had established free space, or an occupied voxel going empty | "the ball moved while I watched": the orienting reflex recast for a sweeping gaze (§5, T6) |
| **per place** (minutes) | the cached cloud of this place | the live cloud at revisit | the difference, localised to a cluster and a bearing rather than a fraction | "this place is not as I left it": the pull returns to a thing that moved (T5) |
| **the room** (the run) | a slow EPM over thing tokens keyed by place (O5's first live instance) | the thing token at this place | its TLE | ranked last: the per-place cache already does the geometric half, and nothing on the interaction goal waits on it |

In simulation nothing moves unless the duck moved it or the harness does, so the within-stop and per-place
loops only fire once the duck can reach a thing. The interaction loop comes first.

## 4. The interaction cycle, as the rewrite rule writes it

The behaviour is one cycle: see a small cluster, walk to it while looking at it, lose it in the last few
centimetres, bump it, and find it somewhere else at the next stop. Nothing in the cycle is a trajectory.

| step | the error minimised | the sensor | the owning module | the gate |
|---|---|---|---|---|
| **notice** | none: a reduction | the open cloud's break-band clusters, the stack rule in the module | `CloudMap` (T1) | a cluster is a candidate while its stack tops out under `small_top` |
| **attend** | the thing token's own instability across sweeps (a thing EPM's TLE) | the attended cluster's world-sized descriptor: stack top, footprint, mass, density, chain | a thing EPM, the same code as every EPM (T1) | the pull is the TLE at the attended thing; a baked, settled thing has none |
| **approach** | bearing to the attended thing → 0, its proximity → 1 | `percept.thing_bearing`, the shape `VisualBearing` emits | `VisualHomingNav`, ported unchanged, racing play for the heading reference (T2) | confidence 0 when nothing small is in reach, so play still drives an empty view; the avoid prior does not apply to the attended thing |
| **look** | the thing's bearing from the sensor's centre → 0, in yaw and pitch | the same token, plus the thing's elevation | the head loop, tracking a slow reference (T3) | gain-0 until the approach loop feeds it; the arbiter's winner owns the gaze |
| **keep it resolved** | the cluster's density → its ceiling | an approach cloud: the cloud kept open over a short walk, translated by odometry (one voxel of error over a metre) | `CloudMap` (T3) | opens on a flagged thing within `approach_range`, files when the thing is lost or the walk exceeds a metre |
| **touch** | a contact the body did not predict | the joint brain's instantaneous residual (O41) | the host's body channel (T3) | above its own running spread |
| **be surprised** | the cached cloud at this place vs the live one, per cluster | `CloudMap` at revisit (T5); within a stop, free-space bookkeeping (T6) | `CloudMap` | the pull returns to the cluster that changed |

Habituation falls out: a block that does not move settles, bakes and loses its pull; a ball that rolls
changes its cluster and the pull returns. Curiosity concentrates on what answers, without anyone defining
interesting (playroom plan §3, now with the cloud as the sensor instead of a camera).

## 5. The build order

Each item is one lever: gain-0 guarded (a config that does not declare it is byte-identical), seed-averaged
on `l2_sweep.py` at n = 6 as a signal, judged on the full metric set with its blind metric named, and put in
front of the operator's eye as a launcher preset before anything is promoted.

### T1. Things as a `CloudMap` output, and a thing EPM

The stack rule moves into the module and runs on the open cloud every few ticks: break-band voxels grouped
into 8-connected columns, each cluster's stack top the contiguous chain of heights over its dilated footprint
with a gap of max(`gap_min`, `gap_k` × range). Two new topics, both empty by default:

- `things_topic`: a ProprioToken describing the attended thing (the nearest cluster whose stack tops out
  under `small_top`), world-sized and in [0,1]: stack top, footprint, footprint aspect, column count, hits
  per column, chain length, plus a rung for range so the token records how well-sampled it is. Published
  only while a thing is attended; an EPM on it earns the vocabulary of things.
- `thing_bearing_topic`: `[vx = +right, vy = +forward, proximity]` in the body frame, the shape
  `VisualHomingNav` already consumes; proximity 0 when nothing small is in reach.

A frozen geometric reduction over the duck's own returns, so a sensor, named as one (`CLAUDE.md` §0 rule 1
still holds: the vocabulary over it is the EPM's to earn). The host logs the attended thing on every compute
tick and the full cluster list at filing, and the scorer labels both against the manifest.

Verdict criteria: the module's clusters agree with the offline rule on the same filed clouds (a faithfulness
check); the attended thing is a real object at the rule's precision (0.9 with the sweep); the thing EPM's
nodes sort by object kind better than the scene EPM's did, and its winner holds across the poses from which
the same object is seen (the pose-invariance test O40 could not run). Blind metric: a vocabulary of one node
scores as steady; read node count with the PCA scatter (§0 rule 2). Behaviourally free: passive, the walk
byte-identical.

**Measured 2026-09-15 (design doc §17.34; R57–R59, n = 6 × 1500 s).** Built as above, passive on every seed
(byte-identical to `★ CLOUD`), exact against the offline rule (583 of 583). The attended thing is a real object
on 73 % of ticks with no floor and 82 % with a two-column footprint floor (`small_ext_min` 0.08, R59, five of
six seeds up); the misses are one-column wall bases and chair legs, and a stability gate on attention does
nothing about them (offline, not built). The thing EPM sorts kinds: per-run purity 0.87–0.91 against chance
0.43–0.52, blocks and balls under separate nodes, and hits per column is what separates them (a ball's curved
face 8, a block's flat face 18), so the descriptor keeps its sampling dims (R58's shape-only form regressed
the vocabulary). Pose invariance stays untestable at 1–3 poses per object per run. **T1 `WORKING`; R59 is the
config T2 builds on.** Two scorer catches on the way: node ids are per run (pooling them read purity 0.56),
and R58 bundled two levers that disagreed.

### T2. The seek loop, racing play

`VisualHomingNav` on `percept.thing_bearing`, its heading and confidence into the arbiter beside the play
loop's, the winner writing the heading reference as every duck loop does (recipe §5). The pull's magnitude is
the thing EPM's TLE at the attended thing; a settled thing lets play win. The proximity prior and R48's
model-implied step treat every return as a wall and would steer around the very thing seek is aiming at, so
the avoid prior is gated off within the attended thing's sector while seek holds the heading. The gate is the
design; the magnitude is tuning.

Metrics: small things approached to within 0.3 m per ten minutes, ball displacements per run (the manifest's
free bodies, instrumentation), wall contacts a minute unchanged, stands held, the arbiter's winner share.
Blind metric: contacts, which a duck crashing into furniture satisfies; its complement is contacts with small
things against walls, and re-approach after a displacement. The first (d): move a ball while the duck walks;
it should re-find it at its next stop at that place.

**Measured 2026-09-15 (design doc §17.35; R60, n = 6 × 1500 s).** Built as `BearingSeekLoop`, a generic loop
that fixes the thing's position by dead reckoning while the bearing is live and homes to it while the cloud is
closed, on the arbiter's vision channel against play. Loud at the episode level: 21 episodes, 17 at a real
object, blocks approached to a median 0.27 m (7 of 11 within 0.4 m, three into contact), arrival by dead
reckoning ends 20 of 21. Walls fall on four seeds of six (21.7 → 12.4 a minute), coverage and stands tie.
Object displacement is blind here (the base already moves the room's objects 3.5 m a run by stumbling), so
interaction waits on T4. The sector gate on the avoid prior is a `REGRESSION` on walls (28.9 a minute; a target
by a wall is what it hides). **T2 `WORKING` as a mechanism, `PARTIAL` as a behaviour; not promoted; preset R60
for the operator's eye.** Next: T4 before T3.

**The walk between stops (2026-09-17, design doc §17.37).** The operator watched R60a circle. Two causes,
one of them a sign: the play loop's bearing had been mirrored on the duck since R27 (its frame is a
reflection of the body's odometry; `PlayLoop.heading_sign −1` fixes it, register O51), and the twist brain's
yaw channel does not close on a steady reference (a heading reflex on `action.vyaw`, `--heading-reflex`,
does). R64 = R60 + `heading_sign −1` + R48's model-implied step: the reference stands still, coverage and
path up, objects moved doubled, walls 20 → 15 a minute; the reflex on top straightens the walks further and
brings the seek episodes to 7 of 8 blocks within 0.4 m, at the cost of walls. **Presets R64 and R64r for the
operator's eye.** The seek loop (T2) and the arrival stop (T4) stand as measured; the walk they ride on is
now the un-mirrored one.

### T3. Looking, the approach cloud, and touch

Three small levers that make the approach seen rather than blind:

- The head loop's reference follows the attended thing's bearing and elevation; a slow reference, since the
  walker jitters the head joints at a zero command (§17.12) and the head brain's pitch prior lost to the gait
  once (H2). With the head down the walls ahead leave the field, which is the same gate as T2's from the other
  side: seek and avoid cannot share the sensor at one instant, and the arbiter's winner owns the gaze.
- The approach cloud: `CloudMap` stays open while a flagged thing is within `approach_range`, translating
  each cast by the odometry's displacement from the anchor. Over a metre the odometry's error is one voxel.
  Also expected to cut the map's walk-time error, which rose 0.17 → 0.27 when the map began holding the last
  cloud's view.
- Touch: the joint brain's instantaneous residual on the bus, gated on its own running spread, as the event
  the cycle's "bump" needs.

**Measured 2026-09-17 (design doc §17.40).** The gaze at the reached thing: `PARTIAL` on what it is for (5 of
28 arrival stops attend a real object within 0.8 m against 1 of 23) and a `REGRESSION` on the stand (rescues
×4): a thing at 0.2 m lies 45° under the beak. Not adopted; the outcome loop observes from a step back
instead (the daemon's unwind). Touch (the joint brain's residual) sees every fall and few contacts (§17.38);
the approach cloud is unbuilt.

**The interaction, as built (§17.39–17.40, register O54):** `SkillOutcomeLoop` asks for Pollen's kick by name
on `intent.skill` at an uncertain arrival; the host fires it from standing, the body backs off and stops to
look; the thing's displacement is the answer, learned per thing node; unseen is unknown. 27 kicks over six
runs, none fall, 12 answers observed at 3–10 cm, one node known. `WORKING` as a mechanism, `PARTIAL` as a
behaviour: the habituation needs more answers per run than one kick every three minutes gives.

### T4. When to stop (O43)

A stop starts on an error, not a timer: arrival at the attended thing (proximity above a threshold, or the
thing lost into the blind zone with touch pending), or the place map's walk-time error above its own running
spread (the signal O39 measured, 0.30 walking against 0.14 at stops). The 80 s timer is retired in the arm
and kept in the control.

**Measured 2026-09-15 (design doc §17.36; R60a, n = 6 × 1500 s).** Built as `--stop-on-arrive` with the timer
kept as the floor. Loud as a mechanism: 32 arrival stops over six runs, beginning a median 0.21 m from a real
object (29 of 31 within 0.5 m). What T3 predicted then shows: at 12 of 31 the cloud attends nothing and at most
of the rest another thing a metre off, because the thing reached is below a level gaze. Where it was visible
(0.3 m) the duck looped stop-walk-stop at it four times in 30 s: lingering without habituation. Costs: walk
82 → 54 %, path down on every seed, walls back to the base's level, a quarter of stops at the 60 s cap. **T4
`WORKING` as a mechanism, `PARTIAL` as a behaviour; not promoted; preset R60a.** Next, in order: T3's gaze at
the reached thing, then habituation through the thing EPM's error, then the (d) tests.

### T5. The per-place change, localised

`revisit_change` per cluster: at filing, each of the new cloud's clusters is looked up in the cached cloud
through the anchors' transform, and a cluster with no counterpart, or a cached cluster with no successor, is
published as change with a bearing. The pull returns to it. Registration by content (O40) improves this and
is not needed to start it; the cell's width (about 0.25 m) is the alignment error to expect.

### T6. The within-stop change, for a sweeping gaze

Free-space bookkeeping along each ray so that a return landing where the cloud had established free space,
or an occupied voxel going empty, is change rather than growth. This is R44's change detector recast so that
it does not need a still gaze. It pays off when things move by themselves, which in simulation means the
harness's mid-stop move, and on hardware means a hand.

## 6. Instruments

- `mj_host/tools/cloud_objects.py` gains `things`: per run and seed, the attended thing's label by the
  manifest (precision of attention), the thing EPM's winner against the label, approaches, displacements.
- The voxel viewer draws the module's clusters with the rule's verdict, so the operator sees what the duck
  flagged and what it walked toward.
- `l2_sweep.py` gains the seek loop's share of the arbiter, approaches, displacements and re-approaches.

## 7. The (d) tests, all from the harness

1. Move a ball while the duck walks (the existing moved-object flag): re-found at the next stop at that place.
2. Move a ball during a stop: the stop ends and the body turns toward it (T6).
3. Lesion the thing bearing mid-run: the duck falls back to play's coverage without a regression in walls.

**The walk, 2026-09-19 (design doc §17.42).** The operator saw R67 circle again at 1060 s. The mirror is
fixed; a second defect on the same line was found (the reference flips by 2π when a bearing crosses ±π) and
turned out to be the orbit's brake, not its cause: play's bearing rotates with the body (O56, open). The
walking cloud is built and works as a sensor (O55); the loops above it need gating before it is a behaviour.
The roll is not a get-up (11 falls, 0 rises).

**The operator's eye, 2026-09-22 (design doc §17.45): `★ THINGS` = R67.** R67r stares at walls on half the
seeds (the reflex follows its loops into play's wall nodes: seed 1 stops 4 at things / 8 at walls against
R67's 12 / 3; `cloud_objects.py where`), `PARTIAL`, not promoted; R69's corner circling is §17.43's residue;
R70's walls "rotated 45°" were the viewer's anchor (the host latched the cloud's world pose only on an open
edge, and the walking cloud files and reopens on one tick: 48 clouds under one anchor), fixed, the odometry
itself 1° and 0.93 of the truth per 10 s of walking. The circling is "mostly solved" by the operator's eye,
so the R67 stack is promoted and the phase turns to what they asked for: interaction over wandering, a wider
vocabulary of intents, and lingering where there is something to learn. First lever: R71, the linger.

## 8. Traps carried forward

- **A preset must carry the harness argv of the arm it names.** `newtest.py` copies the controls of the
  base config's LAST preset and prints the host args; check them against the arm you measured (R61–R64
  lacked `--stop-on-arrive` for a day; design doc §17.38).
- **`newtest.py` numbers a config by the highest `a1v2_r<nn>` CONFIG, not preset.** A preset-only arm (R77, R78, R81, R82)
  leaves a gap, and the next minted config takes a number a preset already owns (r73 → r74 and r77 → r79 were renamed).
  Check the minted filename.
- **Six `--full-logs` hosts at once trip Claude Code's low-memory watchdog** (six × ~75 MB of buffered stdout); run
  `--jobs 3` under `systemd-run --user --scope -p MemoryMax=14G -p MemorySwapMax=0`.
- **The byte-identity guard strips the record fields the phase added** (`pb`, `pl`, `tkind`, `com`, `thg`…): compare
  `grep '^{' LOG | md5sum` after stripping, against `cb24520c…` (the R46 config, 760 s, seed 3).
- **The eye reads one seed and the walk is bimodal across seeds** (R67r, R80/R83: three seeds at things, three at walls).
  Put the per-seed column of `cloud_objects.py where` beside every mean.
- **A replay shorter than the run is not a replay:** a stop is not started with under 60 s left.
- **The host is deterministic**; when a watched run differs from a measured one, compare the argv and the
  JSONL's `patch:` events before suspecting the physics.

- Arms that differ only at stops are identical until 600 s, so the first stop is a clean visual A/B.
- The EPM's `is_novel` is a percentile, not a novelty measure.
- Read head motion from the joints, not the command.
- Use `grep '^{' LOG | md5sum` for a flag-off guard; the banner lines differ by port.
- The balls are 40 g free bodies with low rolling friction: a walk-through moves them, and the duck cannot
  see the moment of contact. The cycle closes at the next stop, not in the act.

## 9. Register rows

| id | item |
|---|---|
| O45 | Things as a `CloudMap` output and a thing EPM (T1) |
| O46 | The seek loop racing play, with the avoid prior gated off at the attended thing (T2) |
| O47 | The gaze follows the thing; the approach cloud; touch as an event (T3) |
| O48 | Per-place change localised to a cluster and a bearing (T5) |
| O49 | Within-stop change under a sweeping gaze: free-space bookkeeping (T6) |
| O50 | The room as a slow EPM over thing tokens keyed by place (O5's first live instance); deferred behind T1–T5 |

O43 (when to stop) is T4; O44 (seeking small things) is the phase.

---

## 10. Where the duck is (the close, 2026-09-27)

The operator's direction at the start: seek what is smaller than itself, interact with it, be surprised when it
answers. At the end, on R79: "the robot successfully kicked the block several times and lingered to play"; on
R80's closer arrival: "the closer distance for interaction is a win." Every number below is derived, with its
verdict and caveats, in the design doc §17.34–17.52.

**Promoted stacks** (launcher presets, `tools/duck_launcher`; the list was pruned to these, the guard, and three
arms for the eye on 2026-09-27 — the retired presets are in `presets_archive.json`, argv intact):

| star | what | promoted |
|---|---|---|
| `★ STACK` | the R19 stand at the joints: the joint brain takes the legs when the body is still | earlier |
| `★ HEAD` | R34: the head brain owns the head joints | 2026-09-11 |
| `★ CLOUD` | R56: the stop's glance — the head sweeps, the cloud accumulates, the map reads it | 2026-09-15 |
| **`★ THINGS`** | **R83: the interaction cycle** (§11) — R79 with the arrival at 0.15 m | **2026-09-27** |

**The promoted run**, as the harness runs it (from `mj_host/`; the preset uses seed 1):

```sh
./build/ogma_mjhost --level2 models/microduck/scene_playroom.xml --graph configs/a1v2_r83_kindfoot.json \
  --secs 1500 --seed 1 --noise 0.05 \
  --head-graph configs/head2_h2_level_slow.json --load-head checkpoints/head2j_h1_s2.json --head-joints \
  --stop-from 600 --stop-every 80 --stop-secs 60 --stop-brain configs/a1v2_r19_settle_each.json \
  --stop-load checkpoints/duck_r19_s2.json --stop-keep-head --stop-freeze-head --map-on-stop \
  --stop-gaze 0.35 0.08 1.0 6 6 --stop-gaze-residual 1.0 --stop-gaze-learn 0.5 \
  --cloud --log-cloud-profile --stop-gaze-sweep 0.6 0.7 --stop-gaze-sweep-slow 1 --map-view cloud --stop-cloud-end 0.45 \
  --stop-on-arrive --skill-unwind 0.3 1.5 --skill-unwind-aim 1.0 --log-com --stop-gaze-at-thing
```

For a seed-averaged A/B pass everything after `--noise 0.05` as `--host-args` to `l2_sweep.py` (`--seeds 6 --secs 1500
--jobs 3 --full-logs --logdir mj_host/log/things_t3/<arm>`), and read the sweep's printed "host args" line before
trusting the run. The graph is R72's (the push) plus `thing_kind_epm` and the two arrival numbers; R79
(`a1v2_r79_kind.json`, arrival 0.25 m) is the comparison arm, one line below in the launcher.

**Where the eye and the harness disagree.** R83's aggregates at n = 6 are R80's to the decimal (the kind changes no
choice until a cell holds two answers, and none does in 21 skills): against R79, arrival stops 17.8 → 5.8 a run,
stops at things 17 → 7, skills 60 → 21, things moved 9 → 3, answers seen 44 → 8, walls 9.6 → 28 a minute, stands
46 → 24 %. Seed 1, the preset's, is the good seed on both arms (R83: 16 stops at things, 297 s). The operator's eye
is the gate and it chose the closer interaction; the harness verdict stands in the record as `REGRESSION` on the
interesting scale, with its re-use (§13, item 1).

## 11. What a run does now

- **0–600 s:** the identification babble, walking.
- **The walk:** the twist brain drives; the play loop (in the duck's own handedness since R63) and the seek loop race
  in the arbiter for the heading reference. The seek loop fixes an attended small thing's position by dead reckoning
  while the cloud sees it and homes to it while the cloud is closed; the linger renews a dropped target while the
  outcome loop still has intents to learn at it.
- **A stop starts on arrival** (0.15 m by dead reckoning) or on the 80 s timer:
  1. The walker settles, the R19 joint brain takes the legs, the head brain's learning is frozen.
  2. **The gaze goes to the thing.** The sweep is centred on the reached thing's bearing and pitched to its
     elevation; the stand holds it (a head 72° down moves the whole-body CoM 1 cm; O36 dissolved).
  3. **The cloud accumulates**, the things reduction attends the nearest small stack, the thing EPM (fine, ~30
     nodes) and the KIND EPM (4 nodes) read its descriptor.
  4. **The outcome loop asks for an intent by name** on `intent.skill` — the least-known of kick, peck and push for
     this kind, cycling — and the host runs Pollen's network from standing at the hand-back (the kick 0.5 s, the
     peck 2.8 s phase-driven, the push = the walker into the thing for 1.2 s).
  5. **The unwind and the look:** the body backs off 1.5 s with its nose on the thing and stops to look; the thing's
     displacement is the answer; unseen is unknown.
  6. **Habituation:** a kind whose intents are known loses its pull (the need falls to 0), and the seek target is
     no longer renewed there.
- **A stop ends** on six known gazes in a row, on the cloud's growth rule, or at the 60 s cap (late in a run the
  cap, because the map keeps inserting; §17.51).

## 12. The phase in one table

n = 6 seeds × 1500 s in the playroom, against the arm named, unless stated. Verdict vocabulary as CLAUDE.md §3.1.

| step | § | what was learned | verdict |
|---|---|---|---|
| T1 things + a thing EPM (R57–R59) | 17.34 | The stack rule in the module is exact against the offline rule; the attended thing is real 73–82 % of ticks; the thing EPM sorts kinds at purity 0.87 (hits per column separates a ball from a block). | `WORKING` |
| T2 the seek loop (R60) | 17.35 | Blocks approached to a median 0.27 m, arrival by dead reckoning 20/21; walls down on 4 seeds; the avoid-prior gate at the target `REGRESSION`. | `WORKING` / `PARTIAL` |
| T4 the arrival stop (R60a) | 17.36 | 32 arrival stops a median 0.21 m from a real object; the reached thing is under a level gaze at 12/31. | `WORKING` / `PARTIAL` |
| The circling (R61–R64) | 17.37 | Play's bearing was MIRRORED on the duck since R27 (`heading_sign −1`); the twist brain's yaw does not close on a steady reference (`--heading-reflex`). R64: objects moved ×2, walls 20 → 15. | fix + `PARTIAL` |
| Stop triggers from the body (R64s) | 17.38 | Falls loud in the residual, contacts not; a push is a stall duration. | `NULL` behaviour |
| The skill runner; the kick (R64k) | 17.39 | Pollen's one-shot networks run as their daemon runs them; mid-walk kick 28 % falls, from standing 0; the thing answers 19 %. | `WORKING` |
| The outcome loop + the unwind (R65, R65u) | 17.40 | Kick what you don't know; the look after backing off sees 12/27 answers. | `WORKING` / `PARTIAL` |
| The peck; two intents (R67) | 17.41 | 31 requests, 13 answers, 0 falls. | **`★` 2026-09-22** |
| Lookahead / commit_hold retried (R68, R69); `--ref-unwrap`; the walking cloud (R70); roulade as a get-up | 17.42 | The 2π flip was the orbit's brake; the walking cloud is a `WORKING` sensor and a `REGRESSION` behaviour (O55); 11 falls, 0 rises. | `NULL` / `REGRESSION` |
| The orbit resolved; the reflex on R67 (R67r); stuck stop, escape, progress stall | 17.43–17.44 | The yaw column turns the wrong way for large errors; the reflex follows its loops into play's wall nodes on half the seeds (O57). | `PARTIAL` |
| The interesting scale; the anchor bug | 17.45 | `cloud_objects.py where` = stops at a thing / wall / open; R70's 45° walls were the viewer's anchor latch (fixed). | instrument + fix |
| The linger (R71); the aimed unwind (R71a) | 17.45 | Renewals 20/6 runs, nothing learned; aimed: stops at things 9.5 → 10.5. | `NULL` / `PARTIAL` |
| The push (R72) | 17.46 | Stops at things 12/run, the best arm then; the push moved its thing 1/11. | `PARTIAL` |
| The spin as frustration; the roll (R73) | 17.47 | `cloud_objects.py spins`; 13 rolls break the watched spin, the total ties. | `WORKING` mech / `NULL` |
| The walk re-fix (R74) | 17.47 | The re-fix drifts to wall bases and legs; the stop-end rule now ignores a walking cloud (fix stays). | `REGRESSION` |
| Fire at what you see (R75); the intent that reaches (R76) | 17.48 | The gate starves under a level gaze (3/49 stops); the push from the stand travels 0.07 m/s. | `NULL` |
| O36 the stand with the head down | 17.49 | Head to 72° moves the CoM 1.1 cm; 0 rescues. | dissolved |
| The gaze at the reached thing (R77) | 17.49 | Skills 39 → 55, answers seen 16 → 31, things moved 2 → 10. | **`★` 2026-09-27** |
| The reach gate on R77 (R78c) | 17.49 | 36/36 asks answered, stops at things 18.2; things moved 1/35. | `PARTIAL` (linger-and-learn) |
| The KIND keys the outcome table (R79) | 17.50 | Nodes known 14/6 runs (first ever), skills 60, seen 44, stops at things 17, walls 9.6. | **`★` 2026-09-27** |
| The arrival at the foot (R80) | 17.50 | Arrival stops 16.5 → 5.8, walls ×2, stands halved. | `REGRESSION` on the harness |
| The closing step (R81, three forms) | 17.51 | The walker from the stand neither turns nor walks in < 2 s; things moved 0/111. | `REGRESSION` |
| The skill from the walk (R82, R82b) | 17.51 | The peck reaches from the walk 8/28 (1/19 from the stand); kicks 1/18; falls 8/49; the cycle restored in R82b. | `PARTIAL` |
| **R80's arrival on R79 (R83)** | 17.52 | Aggregates = R80's; seed 1 good; the eye chose the closer interaction. | **`★ THINGS`, 2026-09-27** |

## 13. What to carry into the next phase: chasing moving things

The operator's next push, in a fresh context. Findings from this phase that bear on it, ranked:

1. **The reach is by sight and from the walk, not from a stop.** Things moved only when the edge was 0.05–0.08 m
   from the body at a settled stand, or when the body was still closing (the peck from the walk, the push). Every
   form of closing the last 10 cm from the stand failed: a tighter dead-reckoned arrival arrives rarely (R80/R83), a
   step from the stand does not step (R81). A moving thing is met on the move or not at all.
2. **The loops were built for a remembered thing.** The seek loop fixes a position once and dead-reckons; the
   walking cloud sees things on 18 601 walking ticks (O55) but every attempt to let a live sighting steer the walk
   regressed (R70, R74) because the live candidates within 0.5 m are wall bases and legs. The gate the live target
   needs is the vocabulary's (a small thing by the stack rule, within the sweep's ±0.6 rad, in the last metre) and
   a precision-weighted fusion with the remembered position, not a replacement. The bearing token's fourth value
   (a walking sighting) is already on the bus.
3. **Things that answer roll away.** The balls are 40 g with low rolling friction; a kick sends one beyond the 0.6 m
   match radius and the outcome is "unknown" — the skills instrument already labels it "rolled". For chasing, that
   rolling ball is the stimulus: the error is a thing that is not where it was predicted to be.
4. **The ancestor is the orienting reflex** (R44, §17.24): a change at a still gaze ends the stop and the duck walks
   to it, 95 % arrival; it refuses a sweeping gaze. T6 (free-space bookkeeping under a sweeping gaze) and T5
   (per-place change with a bearing) are the unbuilt halves of this plan's §3 and are what "the ball moved while I
   watched" needs.
5. **The eye reads one seed.** The walk is bimodal across seeds (three at things, three at walls) on every arm from
   R67r on; the operator's preset seed is a thing seed. Put the per-seed column beside the mean in every report.
6. **The spin is frustration** (`cloud_objects.py spins`; O60): play's reference rotating with the body (O56). The
   roll breaks the spin it fires in; the arbiter does not yet prefer it.
7. **Falls are an accepted price** for contact ("perturb the environment"); skills from the walk double the
   rescues. The recovery scaffold's rise is ~5 s and 3 hand-offs.
8. **Stops lengthen late** because the map keeps inserting and the "six known gazes" rule stops firing; the knob is
   the cap (`--stop-secs 30`, unmeasured).

**Instruments to reuse:** `cloud_objects.py where | skills | spins | stops | seek | heading | things --field tkind`;
the sweep's `spins`, `spinRolls`, `stopsArrive` columns; `--log-com`; the (d) tests of §7 (a moved ball while
walking, a moved ball during a stop, a lesioned thing bearing) are all in the harness already.

**Open register rows:** O55 (the walking cloud), O56 (play's geometry), O57 (the reflex's bimodality), O59 (the
reach), O60 (the spin), O61 (the approach by sight), O62 (the kind's purity), **O63 (this phase's hand-off)**.

**Questions for the design discussion**, by the rewrite rule: what error does chasing minimise — a thing's
predicted position against its seen one, or a bearing rate; which sensor carries it — the walking cloud at 12.5 Hz
and 4 cm voxels (a ball at 0.3 m/s crosses seven voxels a second; a held cloud smears it, a per-cast bearing does
not); which loop owns it — `BearingSeekLoop` with a live, gated re-fix, or a new loop whose need is the thing's
motion; and how it enters the arbiter beside play and seek.
