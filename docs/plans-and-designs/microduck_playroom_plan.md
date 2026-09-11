# Microduck: the playroom plan — a behaviour set, an arena that can show it, the camera, the voice, and play at the joints

Status: plan · Date: 2026-09-10 · Branch: `duck-l2` · Simulation only

*This is the planning document the operator asked for after observing R26–R29 in the arena
(design doc §17.5–17.7): "a good foundation, but we need to define our behaviour set better and
improve the arena accordingly." It sets the target, records the surfaces on both sides as they
are today, writes the behaviour set as loops in the recipe's five fields, designs the arena
that can make each loop observable, and orders the levers. Nothing here is measured. Every
verdict goes to the rung-2 design doc and the register, as before.*

**The target sentence, Pollen's own** (`docs/project/roadmap.md` M9): *a duck left alone in a
room does something worth watching for ten minutes.* Every arena change and every lever in
this plan is judged against that sentence, and against the doctrine's three loud signs of life
(`CLAUDE.md` §3.3): it re-corrects when disturbed, it steps, and it feels around what it bumps.

Companions: [`microduck_port_plan.md`](microduck_port_plan.md) (the port; "▶ Resume here"),
[`microduck_rung2_regime_design.md`](microduck_rung2_regime_design.md) (every measurement so
far), [`microduck_intent_boundary_design.md`](microduck_intent_boundary_design.md) (levels,
the parity ladder, the reporting rules), [`microduck_outreach_plan.md`](microduck_outreach_plan.md)
(PR-1, PR-2), [`loop_and_arbitration_recipe.md`](loop_and_arbitration_recipe.md) (the loop
unit), [`open_items_register.md`](open_items_register.md) (O25–O29 are this plan's).

---

## ▶ Resume here

**State on 2026-09-11, late — the walk-stop-look line is agreed and planned (§12).** The operator's
read of R26–R38 in the room: the duck always stepping, avoiding and tiling a map is a Roomba, and
the target is Pollen's sentence, ten minutes alone worth watching. The reframing that turns the
rhythm into a brain rather than a timer: **a place is a stop** (§12.2) — the map inserts only while
the body is still, the head saccades to the bearing of highest residual, and the walk begins when
nothing at this stop is left to bake. Three modes with one learner (§12.3: remote control, autonomous,
skill; learning never stops, only driving changes), the sixteen built-ins as regime data rather than
trajectories (§12.4), an expression layer fired from brain events over the catalog that actually
exists on Pollen's wire (§12.5), and a ten-minute instrument whose blind metric is variety and whose
complement is contingency (§12.6). **Build order §12.7: W1 the hand-back at the joints first.**
Decisions taken: the hand-back is competence-gated; the rhythm before skills-as-data; host and
simulator only. Register O31–O34.

**State on 2026-09-11.** The head loop is done and promoted: `★ HEAD` (R34) — the head brain
owns the two head joints (Track A at the head, `--head-joints`), identified standing, acting
walking with a slow level prior on a frozen model; level on every seed, the camera steadier
than the walker's own head (frame difference 12.3 → 9.9), the walk "more birdlike" (the
operator's eye; design doc §17.9–17.14). The ask to Pollen is written (outreach plan §8) but
**PR-2 waits**: the operator wants the behaviour set validated in the simulator first, with
what exists.

**2026-09-11, later — the ToF control arm measured first (operator: lean on the ToF; the camera
stays a demonstration for the ask to Pollen).** Design doc §17.15. The sensor's own 8×8 in the
place map: straight into the RBF place EPM it collapses (R35: the RBF grid's bandwidth at 68 dims
flattens the input — an encoder finding, offline-measured, a new `jl_state` EPM encoder kind is
the fix); stacked as the plan's O10 form (R36: `depth_epm` → latent + pose → `map_epm`, both
`jl_state`) it runs, the map grows to ~2× the nodes and does not stop re-tiling in 1500 s, and
the play loop climbing that TLE drives the duck into surfaces: walls/min +6.6 on six of six seeds,
coverage ties (`REGRESSION`); the adaptive insertion gate makes it worse (45.8/min). **What this
settles for the line below:** the host and the EPM now carry the C2 pipeline (the camera arm is
R36 with `depth_epm`'s input swapped for the frame), and E1 must not climb the raw place TLE of a
vocabulary that is still being tiled — the novelty source (baked nodes, or the transition term)
and the arbitration against avoidance come before the camera is rendered. R35/R36 configs stay,
no presets (refuted arms get none).

**2026-09-11, night — the operator's eye on R36 and the dither measured (design doc §17.16).** The
circling the operator saw is R34's too: 286° turned per metre, the yaw command bang-bang, the
map's current node flickering 130–155 times a minute among nodes closer together than the body's
0.18 m turning radius, the play sub-goal re-chosen several times a second. Holding the sub-goal
(R37, `commit_hold`) turns the dither into an orbit or a wall-ride: `REGRESSION` 0+/6− on cells and
straightness. **The fork for the operator** before E1: a play target beyond the turning radius
(a longer bearing horizon), places at the body's scale, or a proportional heading regulator from
the identified yaw row. The sweep now reports `straight` and `switch/min`.

**2026-09-11, later — fork item 1 tried and refuted (R38, design doc §17.17), and the real defect
found.** A target beyond the turning radius, held: `REGRESSION` 0+/6− on straightness; one seed
reproduces R37's orbit to the metre. With the reference then *still*, the twist brain's yaw command
still saturates and flips (140–190/min) and the heading error sits at two radians: **the level-2
twist brain does not hold a heading** — its learned yaw row answers the flickering reference in
R34 and the gait's 2.4 Hz wobble in R38, with a positive-feedback term on its own yaw-rate copy
(R21) either way. Every steering verdict so far measured that the reference was set, not followed.
**The lever before anything in the line below is the yaw channel of the twist brain**: a lesion
of C's yaw row to the heading column, or the model-implied step `u = −e / A(idx, vyaw)`; the bar
is `straight` well above 0.2 on every seed with the play reference live. The `hdg` field and the
`straight` / `switch/min` columns are the instruments.

**Next: the exploration line, in this order**, each a lever with a preset and n = 6, on the
★ HEAD stack (the steady camera is what makes the appearance map possible):

1. **C1** — the head-camera render in the host (an offscreen EGL render of `head_camera` at
   64 × 48, the ToF's 12.5 Hz, published as `host.video.color`; no consumer → byte-identical;
   the cost gate on the realtime factor). The viewer's camera window already shows the frame.
2. **C2** — the visual EPM and the appearance-based place map (O10 on the duck): a frozen
   projection over the frame, centred and normalised (CLAUDE.md §0 rule 2), the place vector
   [visual latent; odometry pose × repeat] into `epm_place` with `pi_cell_size 0`. Metrics:
   nodes vs places, bake rate on static vs moving objects, TLE, PCA vs nodes; (d) move a ball.
3. **E1** — explore by appearance in R27's slot (the play loop's novelty from the visual map).
   Metrics: coverage, contacts, time near objects vs walls, `down%`.
4. **E2** — approach and poke: the bearing to what is novel and near. Metrics: pushes/min,
   TLE after a push, time within a body length of movers. The table-leg wedge (§17.8) and the
   backing-off behaviour (§17.11: backward commands are how it leaves a wall; `--no-backing`
   is a regression) are the two known traps for these loops.
5. Then the behaviour set of §3 as a whole, judged against Pollen's sentence: ten minutes
   alone in the room, worth watching.

Read first: §3 (the loops), §5 (the room), §6 (the camera path), and design doc §17.8 (the
room's baseline R30: 6.6 walls/min, 151 cells, the movables dribbled at 22/min). Mint every
test with `newtest.py`; refuted arms get no preset; the picture metric is blind to a duck
that stops touring (read it with cells and path).

**Earlier resume notes** (A1, the camera fix, the observation trap, H0–H2) are in the
sections below and in design doc §17.8–17.14.

## 1. The operator's direction (2026-09-10), in the rewrite rule's terms

Four things were asked for. Each is restated here as the error it reduces, because that is
the only form this repo builds (`CLAUDE.md` §1).

1. **More life than the sixteen states.** The observation behind it: the standing duck, under
   our brain, moves in a way that is "more random than just standing still." That movement is
   the exploration dither, measured load-bearing twice (design doc §8; "dither is
   load-bearing"). Life, here, is a body whose model of itself is never finished. The plan
   keeps that dither and gives it somewhere to go (§4).
2. **Our brain takes control as it becomes confident.** The mechanism exists and has a
   verdict: `LoopCompetence` grades a loop by whether its own objective moves as predicted
   while it drives (register O21, `PARTIAL` on the Cell). The duck gives it a cleaner test
   because the hand-off is binary and countable (§8).
3. **Exploration by the camera, not the rangefinder.** The rangefinder keeps the walls; the
   camera carries what is worth approaching. "Curious about things its own size" is not a
   size detector: it is a visual vocabulary in which walls bake and go quiet while
   duck-scale things stay novel because they move and change (§3, §6).
4. **The voice carries the error.** Pollen's synth is a live instrument; our xaq_voice already
   sonifies TLE. Connecting them is one small verb on their side (§7).

And a fifth, from the discussion of play: **the brain should have periods of exploring its
own body** — babbling, rolling on the floor, getting up — so that behaviours emerge from the
body in the room rather than from a list. That is §4, and it is the one item that changes the
intent-boundary doc's scope (getting up was out of scope there, §6 of that doc).

## 2. Ground truth on both sides, 2026-09-10

Read before designing. Anchors are into Pollen's checkout at `~/microduck-pr` (their 0.12.0
main) and into this repo.

### 2.1 Pollen's surface, as a client sees it

| channel | what exists | what does not |
|---|---|---|
| **voice** | `sounds::Stream`: pitch, level, mouth-open, a vowel shift of ±3 harmonics, all slewed (pitch 45 ms, level 30 ms, open 60 ms) at 48 kHz (`sounds/src/stream.rs:255–330`). Bank tags a client may fire: alarm, greet, inquire, peck, chirp, coo, wheee (`sounds/src/lib.rs:49`). | **No client verb drives the live voice.** Only robotd instantiates a `Stream` (theremin, chorale). `robot.sound{tag,hold}` triggers a bank variant; `robot.theremin` hands pitch to the hand distance. Nothing carries hz/level/vowel across the wire. |
| **mouth** | `robot.mouth` 0..1, a continuous 20–50 Hz notification (`duck-ipc-proto/src/lib.rs:556`). | — |
| **camera** | 1920×1080 at 30 fps, ~62° HFOV, over WebRTC only (`mediad/src/camera.rs:72–88`). A one-class duck detector at 2 Hz, boxes in frame pixels on the video datachannel (`mediad/src/detect.rs`). | **No frame or feature reaches a robot client.** Nothing from the camera is in `robot.state`. |
| **ToF** | 8×8 at 15 Hz on its own socket (`tof/src/main.rs:126`); a pure-geometry reprojector any client links: Empty / TooClose (< 0.10 m) / Floor / Hit (`kinematics/src/tof.rs:37–57`). Hand tracker: range, closeness, held (`kinematics/src/hand.rs:88`). | **No extent, no clustering, no size.** A wall and a ball are both `Hit`. |
| **motion** | `robot.move` twist, `robot.head` (4 joints), `robot.pose`, `robot.look`, `robot.do{skill}` with ground_pick, kick_left, kick_right, roulade, sit_toggle (`robotd-params/src/registry.rs:166–172`). | No joint-level intent (Track A; the port plan's separate conversation). |
| **interoception** | `robot.health` at ~1 Hz: battery, servo temperature and load (PR-1 adds velocity and load per tick). | — |
| **presence** | `ChoraleHeard{beacon, from, age}` (`duck-ipc-proto/src/lib.rs:4511`). Petting is an in-daemon audio classifier that plays `coo`. | RSSI stays inside btd; petting is not on the wire. |

Consequences: the voice needs one verb from Pollen (§7); the camera path is simulation-only
until a frame or feature crosses their IPC (§6); size must be inferred from pixels and range,
never read from a classifier (§3).

### 2.2 Our host, as the brain sees it

| piece | state |
|---|---|
| arena | `scene_arena.xml`: four walls, 2 m × 2 m, 0.3 m high, a dark checker floor, nothing in it. `--arena-shift` moves one wall and leaves a gap seeds escape through (the R27 (d) confound). |
| camera | Pollen's MJCF defines `head_camera` (`robot_allcollisions.xml:257`); the arena inherits it; **the host renders nothing** — no offscreen buffer, no frame topic. The machine has an RTX 5070 and EGL. |
| exteroception | the 64-beam ToF raycast against group-0 world geometry, 45°, 4 m, 12.5 Hz (`Tof.hpp`), reduced to four sense slots (12–15); the odometry-only place vector `place_in` (`main.cpp:1313`) and its EPM's TLE on slot 11. |
| level-2 sense | 16 slots (`IntentAdapter.cpp:71–88`): gravity, gyro, accel, sensed vx/vy, heading deviation (slot 10), map TLE (11), ToF (12–15). |
| voice | `tools/xaq_voice` reads the `lite` diag topic over ZMQ 7400/7401 from any brain host, maps TLE to pitch and the novelty ratio to volume, chirps on bake, and plays through ALSA. Its studio publishes the mapped numbers on a ZMQ PUB (`src/control.*`), so a network sink is an addition, not a rewrite. The duck's MJCF has a jaw joint. |
| appearance mapping | **does not exist to port.** The Cell's map is a grid over a path integral; the place EPM was demoted to a surprise supplier; the panorama was a staircase; the vision results were harness-invalid (audit V1, H2, R4, R5). The EPM-native map is register O10, `IN_FLIGHT`. The duck is where it gets built first. |
| arbitration | `LoopCompetence` → `LateralVoter` → `EFEArbiter` precision mode, ported gain-0; on the duck, the arbiter's currency is *hold vs release the heading* (R28/R29). |
| play at the joints | the joint-level brain babbles on a host schedule (`--ident-every`), learning switches off past 25° of lean, the recovery harness stands a fallen duck up, the step hand-off gives the joints to Pollen's walker past 6.5° (§15). Ratchet v3 taxes every rescued fall ×0.37 (§12.3). |

## 3. The behaviour set, as loops

Pollen's sixteen states — Chill, LookAround, Wander, TurnInPlace, Zoomies, Startle, Stretch,
Ruffle, Preen, Sneeze, Dance, GroundPick, Nap, BallPlay, Petted, Held — are named in their
`docs/ideas/autonomous_behavior.md` and defined nowhere public (the prototype runtime is not
readable). They are absorbed below, never reimplemented. A loop is specified by the recipe's
five fields; the last column says which of their states it absorbs. Two rows are not loops and
are marked so.

| loop | infers | sensor(s) | predicts | honest signal | confidence to the arbiter | absorbs |
|---|---|---|---|---|---|---|
| **rest** | that nothing needs doing | every module's TLE; on hardware `robot.health` (servo temperature, battery); in sim the consolidation `c` and ticks since the last surprise | error stays low while still | error rises while still | `c` × (1 − recent TLE) | Chill, Nap |
| **avoid** | free space ahead | ToF slots 12–15 | proximity falls when the heading is released to the priors | proximity rises while released | as R29: the value of *releasing* the heading, not a bearing | TurnInPlace, ToF avoidance |
| **explore by appearance** | where the visual map's frontier is | `host.video.color` → visual EPM → the place EPM (visual latent + odometry pose, O10's form) | more visual novelty at the next node than here | novelty gained per metre falls | novelty gained / expected, on the map's own units | LookAround, Wander, the novelty grid |
| **approach and poke** | the bearing to a thing that fills the frame at close range and changes | the visual EPM's residual by image column + ToF range | approaching closes the bearing and grows the thing; a push changes it | the bearing does not close; the thing does not change when pushed | as `VisualHomingNav`: detection confidence in [0,1], 0 when nothing is seen | BallPlay, GroundPick (the skills stay theirs, via `robot.do`) |
| **play at the joints** (new) | what this body does when moved, in this regime | proprioception, IMU | the self-model's residual falls under babble | it does not | `c` of the regime's model | — (Pollen has no such state) |
| **get up** (new) | the route from this regime to upright | gravity in the body frame, joint positions | the gravity vector turns toward upright under the action | it does not | the regime EPM's TLE | — (their "get-up" is a policy) |
| startle — *a reflex, not a loop* | — | any module's TLE against its running spread | — | the spike is the signal | none; reflexes act on every winner | Startle |
| expression — *an output, not a loop* | — | TLE, bake and prune events, the rest loop's confidence | — | — | none | Zoomies, Stretch, Ruffle, Preen, Sneeze, Dance (their skills, fired by our state), plus the voice (§7) |

Deferred with the sensor they need: **Petted, Held** (petting is not on the wire; the ToF hand
tracker is the smallest legal input), **social** (presence is on the wire, RSSI is not). Not
in this plan.

**Why "own size" needs no size.** A frozen encoder over pixels plus a GNG that bakes what
recurs will, in a room of static furniture and a few things that move, bake the furniture
fast and keep the movers novel. The approach-and-poke loop's error is the bearing to what is
novel *and near* (the ToF range). Chair legs are near and duck-sized and never move, so they
bake; a ball moves when pushed, so it stays interesting exactly as long as it keeps answering.
Curiosity concentrates on what changes without anyone defining interesting. If node count says
"one thing" while the PCA scatter says "several", the input is under-conditioned (`CLAUDE.md`
§0 rule 2), not the idea wrong.

## 4. Play at the joints: body exploration

Play as Pollen would script it teaches the body nothing. Play as this brain does it is the
identification babble made endogenous and allowed into the fallen regime.

**What fences it in today, and the lever for each:**

| fence | today | the lever |
|---|---|---|
| the babble is a host schedule | `--ident-every 6 --ident-until 3000` | a boredom gate: babble amplitude rises when the rest loop's confidence has been high for long and the self-model is stale; on hardware, servo temperature closes it. Play-until-warm, rest-until-cool is a homeostat, not a timer (`CLAUDE.md` §5 rule 5). |
| the ratchet taxes every rescued fall ×0.37 | a playing duck pays for every tumble | a play regime the ratchet does not tax; the unbuilt "lean-aware wake" (design doc §12, fork B) is its re-use context |
| learning is off past 25° of lean; the harness rights the duck | the fallen regime has never been identified | open the fallen regime to learning for a bounded window with the rescue held back; the regime vocabulary (design doc §7–§10) keeps a fallen model separate from the standing one |
| the step hand-off at 6.5° | the brain never feels a large lean it could still act on | unchanged in this plan; it stays the safety floor. Counted as a hand-off (§8). |

**The gradient argument.** The catch appeared only when the babble stopped identifying from a
moving body (§14.4); the push-test lesson was that a reflex needs its gradient — the world must
perturb the learner at the scale the reflex acts (§11). Getting up has the same shape one level
up: floor time is the data, not a cost.

**Said plainly.** The catch is a one-step reflex from a linear model at 50 Hz. Standing up is a
multi-step route through regimes the model has not seen. The candidate machinery is in the
substrate — the regime EPM, `GNGRollout` for a short horizon over learned regimes, the
pipeline's hunt stage (the only measured search from a bad pose, §12.6) — and whether it is
enough is the experiment. It may take more than one session. What will not be done is a
scripted get-up: that is the Pollen state the joints would otherwise be handed to.

**Metrics.** Stance survival after a play window; re-consolidation time after it; self-righting
rate from a fixed set of dropped poses within a bounded window; rescues per hour as the crutch
meter (intent boundary doc §5). The (d) test: drop the duck mid-run.

## 4b. The head loop — the H line (agreed 2026-09-10: head first)

**Why the head, and why first.** The walker's command vector is seven numbers: the twist and
four head targets (neck_pitch, head_pitch, head_yaw, head_roll, deltas from HOME; the host's
`Command::head`, Pollen's `robot.head` on the robot — `Observation.hpp` slots 51–55). So a brain
that owns the head needs **no joint access**: it commands the head through the walker's own
channel and the walker tracks it, identically in the host and on hardware. Nothing is asked
of Pollen; the legs stay theirs; every reporting rule of the ladder applies (the head's driver
is named per joint). And a level, steady head removes most of the nuisance variation the visual
EPM would otherwise absorb, so it belongs **before C2**.

**The error, in the rewrite rule's terms.** Head-frame motion the brain did not command:
head gravity off vertical, head angular rate not zero, both from the head IMU (Pollen's ToF
board carries one, read at 100 Hz by tofd; the vendored model has the `head_imu` site with
no sensor, so the overlay adds a gyro and an orientation there — H0). The behaviour that
falls out is the neck counter-rotating the trunk's gait pitch: the vestibulo-collic reflex,
why a chicken's head stays still while its body walks.

| field | the head loop |
|---|---|
| infers | the trunk's motion one step ahead — what the neck must cancel |
| sensors | head gravity, head gyro, the trunk IMU, the efference copy of its own head command; its "joints" are the four head positions relative to HOME in command units |
| predicts | under its command the head gravity stays vertical and the head rate stays zero |
| honest signal | the head-frame rate itself |
| confidence | the head EPM's TLE |

**Design choices made now.** Pitch and roll only; yaw follows the trunk (a stabilised yaw
fights every turn). Babble amplitude small: the head is 38 % of the mass and the walker's
balance feels it; rescues per minute is the guard. The actuator is a command the walker
tracks with a lag (its head low-pass), so: babble holds long enough for the head to arrive,
and identify from a still body first (the R19 lesson), then walking.

**Metrics, with the blind one named.** Head gyro RMS and head-gravity deviation while walking
at 0.2 m/s, against the walker's own head handling as the gain-0 control; rescues/min
unchanged; and the operator's actual complaint measured directly — the mean frame-to-frame
difference of the 64 × 48 brain frame over the tour (computable from any saved run through
the viewer's renderer). The degenerate behaviour: a head locked rigidly to the trunk scores
perfectly on joint motion and worst on world motion — so the world-frame gyro is the number,
never the joint angles. The (d) test: the standing shove series, the head staying level.

**Machinery.** `mj_host/src/HeadAdapter.*` — the twist adapter's code with four command
dimensions and the head's trained ranges (±1.10, ±1.10, ±1.40, ±0.31 rad); `--head-graph
H.json` on the level-2 host; the head brain freezes and resets with the twist brain on a
rescue; the JSONL gains `head` (the four commands), `hg` (head gravity) and `hw` (head gyro)
only when a head graph is present; the identified rows are printed at the end. Config
`mj_host/configs/head_h1_babble.json`: `motor_epm_head` = MotorEPMv2 over the four commands,
12 load slots = head gravity x, y | head gyro x, y, z | trunk gravity x, y | trunk gyro x, y,
z | 2 spare; the H2 prior is `state_prior_indices [12, 13]` (head gravity x, y → 0).

| # | lever | stimulus | metric | promote if |
|---|---|---|---|---|
| H0 | the head IMU in the overlay; `head_gyro()` in the body | — | byte-identity | arena and playroom runs unchanged. **Built 2026-09-10, `WORKING`** |
| H1 | head babble while the walker stands (`--l2-twist 0 0 0`), 600 s, hold 25, scale 0.3 | the body's own head | the identified rows: position diagonal positive and dominant; both pitches on head-gravity x; roll on head-gravity y; rescues | signs consistent across seeds, no rescues. **Built 2026-09-10, `WORKING`: 5 seeds agree to ±5 %, 0 rescues (§17.9)** |
| H2 | the head prior while walking (level + still), gain-0 = the walker's head | the R30 tour + the shove series | head gyro RMS, gravity deviation, rescues/min, the brain-frame difference | loud: the head visibly steadier, balance untouched. **Measured 2026-09-10 (§17.10): level `WORKING` (pitch dev 0.20 → 0.04, every seed), still `NULL` (gyro +14 %), the picture `REGRESSION` (frame diff 12.3 → 16.4). Not promoted; R32 to watch. Found on the way: the IMU frame, the identity hold, the yaw rail, the pitch pair's null space, the drifting closed-loop model** |
| H3 | the camera-stability number over R30 | the tour | frame-to-frame difference, H2 vs control | the number moves with the eye. **Built (offline from a run's qpos): control 12.3, every H2 arm worse** |

**Hardware check before H2 counts there:** whether the head IMU's readings reach a client over
the ToF socket or stay inside tofd.

## 5. The playroom

The arena is an instrument. Each behaviour needs a stimulus it can be observed against and a
metric it moves. Four walls in an empty square test nothing R26 did not already test.

### 5.1 Scale and viewpoint

The duck is ~25 cm tall with a 62° camera at ~20 cm. A chair is four pillars and a roof; a
table is a ceiling it walks under. Those are good stimuli *because* chair legs look duck-sized
and never move. Room: **4 m × 4 m**, walls 1 m (operator, 2026-09-10: 0.3 m walls left the camera
looking at sky and were partly invisible to the ToF's upper rows; the viewer looks in from
above anyway). At the walker's 0.21 m/s a crossing is ~20 s, so a 1500 s run covers the room several
times, which the map metrics need.

### 5.2 Object classes — sorted by how they change, which is the only sorting the brain sees

| class | examples | what it should do in the vocabulary | metric |
|---|---|---|---|
| immovable, static | walls, table, chairs, a shelf | bake fast, go quiet | node count stabilises; TLE falls; contacts/min stays low; time spent near them falls |
| movable by the duck | balls, light blocks, a toy on wheels — free bodies with mass and friction | the only class where its own action reduces its own error (the duck has kick skills) | approaches and pushes per minute; TLE after a push; time within one body length |
| changes on its own, untouchable | a wall clock (a mocap body the host rotates), a mobile, a blinking lamp | pure visual dynamics; novelty that decays with exposure and never with action | looking time; novelty half-life |
| moved by the operator mid-run | any of the above, relocated at a set time | the (d) test; re-inference in the map | map growth after the move; contacts at the old and new positions; the R27 gap closed (no wall left open) |

### 5.3 Colour, shape, light

Real contrast — patterned textures on a plain floor — because a frozen random projection sees
structure at every scale in a pattern and none in a flat colour. Two rules, both from the
picrawler ledger's loudest entry ("the camera was an oracle, and the optics were arbitrary"):
**the encoder is fixed before the room is decorated, and the room is never adjusted to make a
result appear.** Lighting is fixed and never a cue: one angled sun that casts shadows, a weak fill, the
camera headlight nearly off (it flattens textures); no saturated pixels in either view.

### 5.4 Assets, in the order to use them

1. **Primitives with textures.** Boxes, cylinders and spheres make a chair, a table, a shelf,
   a ball. No collision work. Most of the science runs here.
2. **Google Scanned Objects** for toys and household items (~1000 textured OBJ scans, CC-BY,
   includes toys). Concave meshes need convex decomposition; `obj2mjcf` does it and writes the
   MJCF. Only after the visual EPM has mapped the primitive room.
3. Furniture meshes (RoboCasa has MuJoCo-ready ones) only if primitives look wrong in the
   viewer.

Licensing is not an issue for this repo; none of it goes to Pollen.

### 5.5 Harness rules, all learned the hard way

- **Generate the room from a script with a seed** (`mj_host/tools/playroom_gen.py` →
  `scene_playroom.xml` + a JSON manifest), never a hand-edited XML. The Cell's twenty varied
  worlds shared one pillar layout for two months because nobody printed the layout. The
  generator prints its manifest; the run logs it; `l2_sweep.py` asserts it.
- World geometry stays in **collision group 0** so the ToF sees the room and not the duck.
- **A second scene beside `scene_arena.xml`**, so R26–R29 stay reproducible on the room they
  were measured in. R27 re-run on the playroom is the new baseline.
- **Movables are free bodies** with a keyframe position, reset by the harness; the operator's
  moved-object (d) is a `--move <name> <x> <y> --at <s>` on the host.

## 6. The camera path

1. **C1 — render, no consumer.** An offscreen EGL render of `head_camera` in the host,
   small (start at 64 × 48; choose by measurement), at the ToF rate, published as
   `host.video.color` in the Cell's `RawImageFrame` shape so the Cell's vision modules read it
   unchanged. Gain-0: with no subscriber, logs byte-identical. **Cost gate:** measure the
   throughput hit on the primitive room first; the 80× realtime that makes seed averaging
   affordable is the budget. If it halves, the frame gets smaller or slower before any mesh
   asset is considered.
2. **C2 — the visual EPM and the appearance map.** A frozen projection for the visual
   modality (JL, per `primitives/EPM.md`); **condition the input** — centre out the frame
   mean, normalise scale (`CLAUDE.md` §0 rule 2); the place vector = [visual latent; odometry
   pose × repeat] per `PlaceVectorBuilder`, into `epm_place` with `pi_cell_size 0` — this is
   O10 built on the duck. Diagnostics first-class: nodes vs distinct places, bake rate on
   static vs moving objects, the TLE curve, the PCA scatter against node count.
3. **Legality.** Pixels cross the blanket; object positions and identities never do. The
   size cue is inferred (§3).
4. **Hardware.** Simulation-only until a frame or feature path crosses Pollen's IPC; that is
   a second ask and is not in PR-2's first draft.

## 7. The voice

**V1 — on the simulator, now, no ask.** A `duck` patch for `tools/xaq_voice` (TLE → pitch,
the novelty ratio → level, bake → chirp, prune → blip, a TLE spike above the running spread →
the alarm-shaped voice) and the jaw joint driven by the rest loop's error in the host viewer.
The operator hears and sees the error. The only brain-side change permitted is a one-line
O(1) addition to a module's `diag_lite()` (ledger rule).

**V2 — on hardware, no ask.** `robot.mouth` ← the same scalar; `robot.sound` tags on events
(chirp on a bake, inquire on a novel node, alarm on a spike, coo at rest). Nothing changes at
Pollen.

**PR-3 — the continuous voice.** A streaming verb carrying pitch, level, mouth-open and the
vowel shift, in the exact shape of `robot.mouth` (a 20–50 Hz expiring notification), driving
their `Stream`. It helps every client author, carries no philosophy, and is the ask PR-2 can
point at. Prepared locally, opened at the operator's call after PR-1 is reviewed (REPORTS.md
§9). xaq_voice's studio PUB gains a sink that speaks it.

## 8. The take-over ladder and its meters

The intent-boundary doc's parity ladder stands: standing (done, 6/6) → the catch (half the
scaffold's envelope) → the step (phase 0) → **getting up (now in scope, §4)** → walking. Its
reporting rules bind every result here: name the driver of every joint, count the hand-offs,
rescues per hour is the crutch meter, learning frozen in any non-driving driver.

"Our brain takes over as it becomes confident" is `LoopCompetence` per loop feeding the voter
under `scoring_mode precision`, and at the boundary a competence-gated hand-off: the walker
takes the joints when the brain's own attitude error says it must (phase 1a, already built)
and gives them back when the brain's regime model is confident again. Its meter is hand-offs
per hour falling over a run. O21's `PARTIAL` on the Cell was "eats track play's share, not the
ordering principle"; on the duck the ordering principle is countable.

## 9. The levers, in order

*2026-09-11, late: for the exploration line this order is superseded by §12.7 (the walk-stop-look
line); H, V1, A1 and the camera path stand as written.*

One at a time, gain-0-guarded, n = 6 for promote-or-kill, then the operator's eye, then the
verdict in the design doc and the ledger. R-numbers are minted by `newtest.py` at build time.

| # | lever | stimulus | metric | promote if |
|---|---|---|---|---|
| H0–H3 | **the head loop** (§4b) — head first, agreed 2026-09-10 | | | |
| V1 | xaq_voice `duck` patch + jaw | any run | ear and eye | the operator hears TLE |
| A1 | playroom generator + `scene_playroom.xml`, primitives, four object classes, seeded manifest; R27 re-run on it | the room | R27's own metrics (contacts/min, cells, map nodes) on the new room | R27 is not degenerate here (no orbit, no wall-riding); the manifest is logged. **Built 2026-09-10, `WORKING` as an instrument (§17.8): 5/6 seeds tour, 140 cells, 98 nodes; operator's eye pending** |
| C1 | head-camera render, no consumer | — | throughput; byte-identical logs | ≥ half of today's realtime factor kept |
| C2 | visual EPM + appearance map (O10 on the duck) | the room; one object moved at t | nodes vs places; bake rate static vs movers; TLE; PCA vs nodes | walls bake and movers stay novel; the map grows after the move at the right place |
| E1 | explore-by-appearance in R27's slot | the room | coverage, contacts/min, time near objects vs walls | ≥ R27 on coverage, ≤ R27 on contacts |
| E2 | approach and poke | movables | pushes/min; TLE after a push; time within one body length of movers | it spends more time at what answers than at what does not |
| B1 | boredom-gated babble, untaxed | the standing brain | stance survival; re-consolidation time | the stance survives and re-consolidates |
| B2 | the fallen regime open, rescue held back for a window | N dropped poses **and the table-leg wedge** (design doc §17.8: a watched run stayed wedged 690 s through 83 given-up rescues — the stand policy is the rescue, and it has no "wedged" state) | self-righting rate; rescues/hour; `down%` per seed | loud or nothing (§3.3) — no excavating a marginal rate |
| PR-3 | the voice verb | — | their CI | the operator's call |

Carry-overs folded in: the shifted-scene gap (A1), O24 node persistence (C2/E1: a
prototype the world now contradicts is a moved object), the "map too small to route" liveness
check (A1's re-run of R27), the inert wander rule (E1 replaces it).

## 10. What will not be done

- A scripted get-up, a scripted dance, or any behaviour written as a trajectory (§1 rewrite
  rule). Expressions are Pollen's skills fired by our state, or nothing.
- A size or object classifier, or object positions crossing the blanket (§5 rule 3).
- Adjusting the room, the lighting or the textures to a result.
- Powering a marginal self-righting rate with more seeds (§3.3).
- Opening anything at Pollen without the operator's go (REPORTS.md §9.6).

## 11. Register rows

| id | item | state |
|---|---|---|
| O25 | Head-camera render in `mj_host` (sim only); a frame or feature path across Pollen's IPC is a separate ask | `OPEN` |
| O26 | The appearance-based place map on the duck — O10's first live instance | `OPEN` |
| O27 | Play at the joints: the boredom gate, the untaxed play regime, the fallen regime opened to learning; getting up brought into the ladder's scope | `OPEN` |
| O28 | PR-3, a streaming voice verb at Pollen | `DEFERRED` until PR-1 is reviewed |
| O29 | The playroom generator and its seeded manifest as a harness requirement | `OPEN` |

## 12. The walk-stop-look line (2026-09-11, late — agreed)

*The operator's direction after watching the exploration line (R26–R38): what the duck does now —
always stepping, avoiding, tiling a map — reads as a Roomba, and ten minutes of it is not worth
watching. What was asked for: use the sixteen built-in poses and activities as scaffolding while
leaving room for the body to be learned; a cold-started duck with no memory should stand up and look
around; saccades in the mapping paradigm; walk a little, stop, look around; our brain at the stops,
since it stands dynamically; babble and play, and learn to get up; and a hybrid in which active
inference is a mode beside remote control, the two interchangeable. Each is restated below as the
error it reduces, and the pieces that already exist are named.*

### 12.1 What the Roomba feeling is, measured

Design doc §17.16–17.17: the map's current node flickers 130–155 times a minute among nodes closer
together than the 0.18 m turning radius, the play sub-goal is re-chosen several times a second, the
yaw command is bang-bang, and the twist brain does not hold a bearing. The map is built while walking,
from a wobbling body, against a reference that never settles. Every steering verdict measured a
reference that was set, not followed.

### 12.2 A place is a stop

The place map is only well-conditioned when the body is still: the head brain halves the frame
difference walking but the gait's 2.4 Hz is still in every view, and the dither above is the map
being tiled from motion. So: **the map inserts only while the body is stationary.** At a stop the
head saccades to the bearing where the place EPM's residual is highest, holds until that view bakes,
and jumps to the next; when nothing at this stop is above the running spread, the only remaining
gradient is elsewhere, the duck picks the least-baked bearing, turns in place to face it, and walks.
The walk, stop, look rhythm is the map's own sampling schedule, not a timer. What falls out:

- **Saccades are discrete for free.** The target is a GNG winner, so the head jumps and holds; no
  scan pattern is written. This is E1 with the head as its first actuator.
- **Places land at body scale.** Stops are separated by walks — the fork's item 2 (§17.16) without
  a cell-size constant.
- **Cold start is stand and look.** With an empty map there is no node to walk toward; the explore
  loop's only available action is the head. Standing first is what the arbitration does when the
  map is empty, not a rule.
- **The stop length is the honest test.** A stop at a baked place is short; a stop at a new or moved
  place is long. That is a claim a Roomba cannot make, and it is measurable (§12.6).

**The prerequisite that does not go away.** The twist brain's yaw channel (§17.17) is still the lever
before a chosen bearing can be walked to. This line lowers its bar: a bearing is chosen once per stop
and held over a walk of a metre or two, which is the form that was loud (R22, hold-my-current-heading,
20/20 after a shove). Facing the chosen bearing happens *while stationary*, through the walker's yaw
rate with the view as the homing target (turn until the current view is the chosen node), so the
learned yaw row is not on the path for the turn, only for the hold.

**The head's yaw comes back, gated.** The head loop excluded yaw by design (§4b: a stabilised yaw
fights every turn). Saccades need it, so the gate is stance: head yaw is the saccade's channel while
the body is still and follows the trunk while walking. The promoted ★ HEAD is untouched on the walk.
The gating is the design; the magnitude is tuning (`CLAUDE.md` §1 rule 4).

### 12.3 Modes: three drivers, one learner

The hybrid exists in pieces. Pollen's walker, our joint brain and our head brain each drive
something; the step hand-off (§15) gives the joints to the walker past 6.5° of lean. Missing: the
return trip and a command mux. Three modes, in the host now and at their daemon later, where a mode
is simply which client sends `robot.move`:

| mode | who drives | what learns |
|---|---|---|
| **remote control** | the operator's twist to the walker (a gamepad or the viewer's keys) | the level-2 model identifies from it — the efference copy is the twist whoever authored it. Driving the duck around teaches it its body; that is a community story and costs nothing |
| **autonomous** | the loops. At a stop with the twist at zero the joints hand back to our brain, which stands and catches (R19) | everything |
| **skill** | one of Pollen's one-shots, fired by our state (§12.5) | level 2; the regime EPM (§12.4) |

**The rule that keeps it clean:** learning never stops in any mode, only driving changes (§5 rule 4,
never disable a working loop). One nuance from the H line: the joint-level model identifies only when
it drives, because identifying under the walker's closed loop gave the drifting model of §17.10; level
2 learns in every mode. The mux is gain-0: with no operator input, nothing moves.

**The hand-back is competence-gated** (decision, 2026-09-11). Early in a run the walker stands at
the stops; as `LoopCompetence` on the standing regime rises, our brain does. "It takes over as it
becomes confident" (§8) becomes visible inside the ten minutes — the standing dither the operator
called "more random than standing still" is the visible sign — and the transitions, where the falls
will come from, are counted as hand-offs per hour.

### 12.4 The sixteen states as scaffold: regime data, not trajectories

A skill puts the body into a regime the babble never visits, and that is what it is for here. A
roulade is floor-regime data; sit toggle is a sitting regime and a nap with a scaffolded get-up built
in; ground pick is a deep crouch with the camera at the floor; the rise of the sit-stand network is
the fallen-to-upright route observed proprioceptively. The regime EPM (design doc §7–§10) identifies
in each. What the brain may not do is copy the trajectory (`CLAUDE.md` §5 rule 6): its own get-up
comes from rolling out the model it identified (§4's machinery: the regime EPM, `GNGRollout`, the
hunt), with the scaffold held back and rescuing on failure. Identification, not distillation, is the
line. §4 called floor time the data rather than the cost; the skills get the body there faster.

### 12.5 The expression layer: the catalog, and what fires it

**What exists on Pollen's wire** (their 0.12.0 main, `duck-ipc-proto/src/lib.rs`, `policies/`):

| kind | items | notes |
|---|---|---|
| trained one-shot skills, `robot.do` | `roulade` (forward roll, ends on the floor, rises), `kick_left`, `kick_right`, `ground_pick` (a daemon-driven crouch phase), `sit_toggle` (latched) | each its own `.onnx`; the skill table is open — any network with a name, a duration, a twist and an unwind (`SkillParams`); priority roulade > kicks > ground pick > sit > stand > walk (`robotd/src/control.rs`) |
| body pose, `robot.pose` | z −0.025..+0.010 m, roll and pitch ±0.26 rad, glided or snapped back (`active: false`) | a crouch, a lean, a tilt, a bob; the snap-back is a startle body |
| head | `robot.head` four joints; `robot.look` a trunk-frame gaze point with the daemon's IK | the saccade's channel on hardware |
| mouth, `robot.mouth` | 0..1 | nothing else moves it |
| twist, `robot.move` | turn in place, speed | Zoomies is a twist burst, not a state |
| voice bank, `robot.sound` | alarm, greet, inquire, peck, chirp, coo, wheee (held: loops while the notification keeps arriving) | one random variant per play; refused only by a robot with no voice |

**What does not exist.** Stretch, Ruffle, Preen, Sneeze, Dance, Zoomies, Startle and the rest are
states of the prototype runtime; their own ideas document says that module "exists nowhere in the
daemon", and nothing public defines them. They were compositions of the channels above. There is no
stretch to fire; there is a pose channel, a head, a mouth and a voice, and a stretch is a shape drawn
on them.

**Firing them from brain state.** Expression stays an output, not a loop (§3), and the only rule is
that every firing is driven by a measured quantity — never a timer, never a random draw. The
substrate's events are few:

| brain event | expression |
|---|---|
| a saccade lands on a high-residual bearing | hold the gaze there; `inquire`; lean the pose forward a little; the mouth opens by the residual. If it bakes while looking: `chirp`, move on. If it is near and answers: the approach loop, with the kicks or ground pick as the payload on arrival |
| a TLE spike above its running spread | `alarm`; the pose's snap-back. The spread-relative threshold rate-limits it for free |
| a node bakes | `chirp` (as xaq_voice already does) |
| the rest loop wins | `coo`; crouch the pose; close the mouth. Sit toggle here is a nap with a scaffolded get-up, and regime data (§12.4) |
| walking fast toward something novel | `wheee`, held while the novelty gained per metre stays high, released when it falls |
| our brain takes the joints at a stop | nothing yet; `greet` is tempting, the operator's eye decides |

**The five composite emotes come from the brain, not a script.** Preen and ruffle are what the
standing brain already does — the exploration dither — given the head and pose channels and gated to
boredom (B1). A stretch is a slow pose excursion to the edge of the trained range when the self-model
is stale. The discrete trained skills stay Pollen's; the composite gestures are the brain's own body
exploration made visible; nothing is a trajectory.

**Cost in the simulator.** The host vendors only the stand and walk networks (`scaffolds/README.md`);
the five skills are the same fetch-by-hash and the same runner plus a skill window mirroring their
priority order, off by default. The host's command carries the twist and the head; the pose block is
an addition. Sound in sim is xaq_voice (V1); the bank tags become an event track the viewer draws on
the timeline, so the operator sees when an emote fired and against which signal.

### 12.6 The ten-minute instrument

Before any of it, one instrument: a **behaviour histogram over ten minutes** — time in stop, look,
walk, play, rest and skill; stops per run; saccades per stop; hand-offs; plus the doctrine's three
loud signs (re-corrects when disturbed, steps, feels around what it bumps). **Its blind metric is
variety itself**: a duck firing a random skill every twenty seconds scores varied and is a slot
machine. The complement is **contingency** — stop length tracks the novelty at the stop; saccades per
stop fall as places bake and rise at a moved object; emotes track the events in the table above, and
never fire without one. Both numbers, always together.

### 12.7 The levers, in order

One at a time, gain-0-guarded, n = 6 for promote-or-kill, a preset each, then the operator's eye.
This order supersedes §9's for the exploration line; C1/C2/E1/E2 continue afterwards with the map
built on stops.

| # | lever | stimulus | metric | promote if |
|---|---|---|---|---|
| W0 | the ten-minute instrument (§12.6) in the sweep and the viewer | any run | the histogram; contingency | it reads R34 and R38 as the operator saw them |
| W1 | **the hand-back at the joints**: walker stopped, twist zero, our joint brain takes the joints from whatever pose the walker leaves them in; competence-gated | the R30 tour with stops | survival and catch rate after a hand-back; rescues/hour; hand-offs/hour falling | the stance survives the transition on every seed. Produces walk, stop, stand |
| W2 | stance-gated head yaw: the saccade channel while still, trunk-following while walking | a stop | head yaw excursions at stops, zero on the walk; ★ HEAD's picture number unchanged walking | the walk is byte-identical to ★ HEAD |
| W3 | insert-on-stop for the place map, the saccade target = the max-residual bearing, the stop ends when nothing is above the spread | the room; one object moved | nodes per stop; saccades per stop falling as places bake; stop length vs novelty; `switch/min` | stops are short at baked places and long at new or moved ones — loud or nothing |
| W4 | the command mux and remote-control mode in the host and the viewer | the operator's hands | the level-2 model's identified rows after a driven session vs before; byte-identity with no input | the model improves under driving |
| W5 | the twist brain's yaw channel (§17.17), now with one bearing per stop: turn-in-place under the view as homing target, then the R22 hold | a chosen bearing | `straight` ≥ 0.2 on every seed with the reference live | the duck walks where it looked |
| X1 | expression events: the voice tags and the pose channel fired from the §12.5 table; the event track in the viewer | any run | emotes per event; emotes without an event (must be zero) | the operator hears and sees the error |
| X2 | the skill runner in the host: five networks vendored by hash, a skill window in their priority order, the pose block on the command; off by default | a fired skill | byte-identity off; rescues around a skill | skills run in sim as on the robot |
| then | B1/B2 with skills as regime data (§12.4); C1/C2 with the map on stops; E1/E2 | | | |

### 12.8 Not done, in addition to §10

- A walk/stop cycle on a timer, or a stop length that is a constant.
- An emote from a random draw or a schedule; an emote without a brain event.
- A copy of any skill's trajectory as the brain's own; the get-up is rolled out from an identified
  model or it is Pollen's.
- A daemon-side mode switch before the host's is measured (sim only, as the branch says).

### 12.9 Register rows

| id | item | state |
|---|---|---|
| O31 | A place is a stop: insert-on-stop, the saccade as the map's actuator, the stop ending on the residual; the turn-in-place under a homing target before the R22 hold | `OPEN` — W2, W3, W5 |
| O32 | Modes: the command mux, remote control as identification data, the competence-gated hand-back at the joints | `OPEN` — W1, W4 |
| O33 | The expression layer: events over Pollen's catalog; the skill runner and the pose block in the host | `OPEN` — X1, X2 |
| O34 | The ten-minute instrument: the behaviour histogram and its contingency complement | `OPEN` — W0 |
