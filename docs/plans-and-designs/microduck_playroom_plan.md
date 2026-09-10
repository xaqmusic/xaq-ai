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

**A1 is built (2026-09-10, design doc §17.8): `WORKING` as an instrument, awaiting the
operator's eye.** `mj_host/tools/playroom_gen.py --seed 1` → `scene_playroom.xml` + manifest;
host flags `--move NAME X Y S`; the `obj` field; the sweep reads the manifest. R27's loop on
it, n = 6: 140 cells of 256, 98 map nodes, 33 ± 39 walls/min (one wall-rider), 0.07 objs/min,
no escapes. Launcher: **R30** (seed 6) and its moved-ball (d) twin. Render the room to check
it: the viewer's venv with `MUJOCO_GL=egl` and `mujoco.Renderer`. **The head camera is
re-placed** (`robot_overlay_playroom.xml`, generated: at the lens front, facing the ToF's
forward, upright, fovy 49°); a mid-run render is level and shows the room — and much sky over
the 0.3 m walls, a wall-height decision for the operator before C2.

Next: the operator observes R30; then C1 (the render) or V1 (the voice), whichever they
choose. Mint every test config with `tools/duck_launcher/newtest.py`; every lever ships with
a launcher preset that mirrors the harness line.

---

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

## 5. The playroom

The arena is an instrument. Each behaviour needs a stimulus it can be observed against and a
metric it moves. Four walls in an empty square test nothing R26 did not already test.

### 5.1 Scale and viewpoint

The duck is ~25 cm tall with a 62° camera at ~20 cm. A chair is four pillars and a roof; a
table is a ceiling it walks under. Those are good stimuli *because* chair legs look duck-sized
and never move. Room: **4 m × 4 m**, walls 0.3 m (visible to the ToF; low enough to watch
over). At the walker's 0.21 m/s a crossing is ~20 s, so a 1500 s run covers the room several
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
result appear.** Lighting is fixed and never a cue.

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

One at a time, gain-0-guarded, n = 6 for promote-or-kill, then the operator's eye, then the
verdict in the design doc and the ledger. R-numbers are minted by `newtest.py` at build time.

| # | lever | stimulus | metric | promote if |
|---|---|---|---|---|
| V1 | xaq_voice `duck` patch + jaw | any run | ear and eye | the operator hears TLE |
| A1 | playroom generator + `scene_playroom.xml`, primitives, four object classes, seeded manifest; R27 re-run on it | the room | R27's own metrics (contacts/min, cells, map nodes) on the new room | R27 is not degenerate here (no orbit, no wall-riding); the manifest is logged. **Built 2026-09-10, `WORKING` as an instrument (§17.8): 5/6 seeds tour, 140 cells, 98 nodes; operator's eye pending** |
| C1 | head-camera render, no consumer | — | throughput; byte-identical logs | ≥ half of today's realtime factor kept |
| C2 | visual EPM + appearance map (O10 on the duck) | the room; one object moved at t | nodes vs places; bake rate static vs movers; TLE; PCA vs nodes | walls bake and movers stay novel; the map grows after the move at the right place |
| E1 | explore-by-appearance in R27's slot | the room | coverage, contacts/min, time near objects vs walls | ≥ R27 on coverage, ≤ R27 on contacts |
| E2 | approach and poke | movables | pushes/min; TLE after a push; time within one body length of movers | it spends more time at what answers than at what does not |
| B1 | boredom-gated babble, untaxed | the standing brain | stance survival; re-consolidation time | the stance survives and re-consolidates |
| B2 | the fallen regime open, rescue held back for a window | N dropped poses | self-righting rate; rescues/hour | loud or nothing (§3.3) — no excavating a marginal rate |
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
