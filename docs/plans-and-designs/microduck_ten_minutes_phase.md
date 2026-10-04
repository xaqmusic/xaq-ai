# Microduck: the ten-minutes phase. Structure, the phantom, and a run worth watching to the end

Status: **opened 2026-10-02; ★ TURN 43 % boring → **★ T4** 22 % (n = 18; design doc §17.96–17.99), promoted on the operator's eye; next: the phantom (§5)** · Branch: `duck-l2` · Simulation only · Base: `★ TURN`
(chase phase §12). Every verdict goes to the rung-2 design doc §17 (from §17.96) and the
[register](open_items_register.md) (O68–O70). Nothing here is measured unless a section says so.

*The phase after the chase phase ([`microduck_chase_phase.md`](microduck_chase_phase.md) §12, `★ TURN`). Its end is a
behaviour set to put in front of Pollen Robotics as the plan for the duck's autonomous brain: ten minutes of behaviour
that stays interesting.*

---

## 1. The direction

The operator, 2026-10-02: the duck's behaviour set is compelling; the goal is **ten minutes of interesting behaviour**,
and the run gets boring when the duck **gets stuck in corners and stares at the wall**. Two pushes:

1. **Structure.** The duck must see tall stacks of voxels along the walls, tables and chairs as one large immovable
   object — consolidated even where the sampling leaves gaps — and navigate around it, instead of investigating and
   pecking or kicking at the low fragments of it the stack rule reads as small things.
2. **The phantom.** Chasing moving things, which the navigation work interrupted, with a route to object permanence:
   a slow loop that keeps attention on a thing that has moved and left the view, and drives the duck to reduce its
   error against that phantom; when the duck is confident the thing moved, it keeps looking for it until its interest
   is spent.

**The operator's decisions (2026-10-02):**
- **Balls by walls stay things.** The consolidation must not swallow a ball or block that stands against a wall (the
  failure of R91's `things_isolated`, §17.58).
- **Walls: both.** The geometric consolidation is the sensor; the outcome loop also learns that structure does not
  answer, so a fragment that slips through is tried once and then the whole category habituates.
- **The tunnel first.** The permanence stimulus is a tunnel on the train track; the kicked ball rolling out of view
  is the second.
- **Order:** the instrument, then structure, then the phantom (the phantom's belief needs the structure map).

## 2. What the record already says (read before building)

- **Isolation** (R91–R93, §17.58): "nothing tall within 0.25 m" separates the moving train (95 % isolated) from static
  clusters (6 %), and as a gate on the *thing* it refused the room's real balls and blocks, which stand near furniture
  and walls. `WORKING` on the mover candidate (in `★ TURN`: `mover_isolated`), `REGRESSION` on the thing.
- **The static yield** (R111, §17.78): dropping a held target with tall voxels within a body length raised the walls —
  a loop without a target hands the walk to play, which brushes walls as often. 73 % of static targets are set at
  stops, a median 0.33 m from a wall, a quarter within 3 cm: **8 cm, two-column fragments of a wall's base band, the
  ToF's zone spacing at a metre splitting the band into "small isolated" pieces.** The neighbour count cannot tell a
  fragment (13 columns within 0.3 m) from a thing (18). The record's re-use context: **a row rule — the line — that
  tells a wall's base from a thing at the sighting; then the yield stops the approach rather than the target.**
- **Permanence:** coasting (R95: walk on to the predicted point) is a `REGRESSION` — the prediction leads into walls;
  recognition (R98: a lost mover kept in mind 5 s, re-acquired at a sighting near the prediction) is a safe signal and
  is on in `★ TURN` (`chase_memory_ticks 250`); a lost chase starts a look (R94, `--stop-on-lost`). §17.64: the real
  form "wants a prediction of where a thing *can* go (the map's free space)".
- **The chase's start** is the standing limit: about one crossing in four becomes a chase (§17.68); the next lever named
  there is the age gradient across one cast's voxels as a single-cast velocity.
- **Free space along each ray** at 4 cm voxels (the trail, R87–R88) is `NULL`: any surface crossing a voxel fills it
  partly. Negative evidence at the scale of a whole thing is untried.

## 3. Phase 0 — the boring minutes (the instrument)

Before any lever: every second of `★ TURN`'s 600 s runs (the f12 sweep, n = 18, the train room) sorted into one
category, and every boring episode traced to what started it. The playroom plan's ten-minute histogram (§12.6,
designed, never built) is this instrument; its blind metric is variety itself (a slot machine scores varied), so it
reports contingency beside it.

**Categories** (one per tick, first match wins; truth from the manifest, scoring only):
`down` (the recovery scaffold drives) · `skill` (a kick, peck or push runs) · `chase` (the chase holds a mover) ·
standing: `stand@thing` (a movable within 0.6 m of the trunk) / `stare@structure` (wall or furniture within 0.5 m
along the head's view, nothing movable in the ToF's cone within 1.5 m) / `stand-open` · walking: `pinned` (wall or
furniture contact, or under 3 cm/s for 2 s with structure within 0.3 m) / `seek→thing` / `seek→structure` /
`seek→nothing` (the seek target's believed position put into the world through the odometry's own pose, then labelled)
/ `wander` (play holds the reference).

Boring = `stare@structure` + `pinned` + `seek→structure` + `seek→nothing` + spins. Interesting = `skill` + `chase` +
`stand@thing` + `seek→thing`. Each boring episode over 3 s reports its start: the stop event that began it (`bored`,
`arrive`, `stuck`, `lost`, the timer) or the loop that held the reference, and its seek target's label.

Instrument: `mj_host/tools/ten_minutes.py`.

**Measured on ★ TURN (2026-10-02, f12, n = 18 × 600 s; design doc §17.96).** Boring **43 %** of the run (20–72 % by
seed), interesting 35 %. The two biggest boring categories are the operator's: standing facing structure with nothing
movable in view, **104 s a run**, and walking to a held target that is a wall or furniture face, **80 s**. **38 % of
arrivals (94/248) are at structure.** The longest stares are stops that run to the 60 s cap facing a wall (five seeds
carry them). The chase holds 1.6 s a run — nearly absent on ★ TURN, which phase 2 must address before the phantom.
Phase 1's consumers, ranked by these numbers: the approach to a fragment first, then the stop that does not end facing
structure.

### 3.1 The operator's eye on T1 · TOP SEEN (2026-10-02)

- **Seed 1:** stuck against a wall and staring at it around 291 s — the remaining stare (the stop facing structure,
  ~45 s a run on the sweep) is still there.
- **Seed 2060249272: "much more interesting."** The train passes through the duck's view in the first few seconds and
  the duck tries to chase it a few times. **The test seed for the chase work** (phase 2), alongside the n = 18 sweep.
- **Technical debt paid before continuing:** the live inspector lacked widgets for the duck's modules. Built
  2026-10-02: CloudMap, BearingSeekLoop, SkillOutcomeLoop, LoopCompetence, JointSensorimotorBridge, and a
  self-model & priors tab for MotorEPMv2; each brain (intent, head, stand) now serves its own inspector ports
  (7400 / 7402 / 7404) with a brain selector in the inspector (`tools/xaq_inspector/README.md`).

## 4. Phase 1 — structure

**S0, offline, on filed clouds** (as the stack rule began, §17.31): project every voxel above the small height onto the
floor plane; a morphological **closing** (dilate, then erode) whose radius is the ToF's zone spacing at the range the
voxel was seen from (≈ 0.1 rad × range: 10 cm at 1 m, 20 cm at 2 m — the sensor's geometry, not a tuned constant)
bridges the gaps along a wall without thickening it; table and chair tops seen from afar cover the legs beneath. A
small cluster whose footprint lies **on** the closed structure is a fragment; one that **protrudes** from the line is
a thing. Scored by truth on R91's failure: fragments refused, **balls and blocks against walls kept**.
**S0 and S1, measured (2026-10-02, design doc §17.97).** The line alone refuses 25–47 % of the fragments, because most
fragments are not gaps in a line: **their top was never seen.** At stops the gaze is pitched down at a thing, the field of
view ends below the wall's top, and the stack rule calls the base small. So S1 became a sensor change: the ToF's empty
rays go to the cloud (`--tof-free-rays`), every ray's free space is recorded (`CloudMap.free_rays`), and a cluster is small
only once a ray has passed over its top (`small_needs_top`). The module refuses 65 % of stop fragments and 0 % of open-floor
things. On ★ TURN, n = 18: **walls 32.7 → 15.4 a minute, boring 43 → 25 %, interesting 35 → 47 %, arrivals at structure
94 → 32**, falls 3.2 → 2.1, the first minute a tie. `WORKING`; preset "T1 · TOP SEEN" for the eye. The line (the
consolidation proper) stays the next lever for the walking cloud, where the fragments are registration smear in front of
a seen wall.
**S2c, the stop that stares (2026-10-02, design doc §17.98).** A stop facing a wall ran to the 60 s cap because its own
cloud never opened: the gaze sweep rocks the stand past the 0.15 rad/s stillness bar, so a walking cloud stays open and
the growth rule (which judges only a stop's cloud) never fires. `--stop-is-still` (the stop's standing phases count as
still): capped stops 35 → 0, stares halved, the green block moved in the first minute 10/18 (4/18) — but boring 25 → 31 %,
arrivals at structure 32 → 71: the stop's own cloud now opens every time and hands over the fragments `small_needs_top`
leaves (35 % of a stop cloud's). `WORKING` mechanism, `REGRESSION` on the run at n = 18; preset for the eye. Retry on top
of the next structure lever.
**The levers on the base (T1 + stop-is-still, the operator's eye; design doc §17.99).** The line on a seen wall (S1b,
`line_tol_k` 0.049 = half the zone spacing) `WORKING`: boring 30.5 → 23.9 %. The outcome loop learns that structure does not
answer (S3: a context EPM over [on the line, near tall] keys the table; the cell's answered share is the seek loop's need for
a sighted thing) `NULL` starved, then `PARTIAL` with the context pooled at two answers (S3b): arrivals at structure 71 → 40.
The impeded look (the operator's: seek's range unclosed 3 s → back off, look up, a wall is forgotten and escaped) `PARTIAL`:
arrivals at structure 71 → 31, a fall a run. **The stack T4: boring 22.3 %, worst run 41 %, walls 14.5 a minute** — the
candidate for the eye, tied with the line alone on the mean, tighter in the tail.
**S1 (as first planned):** the reduction in `CloudMap`, off by default, byte-identical when off, published as a topic.
**S2, one consumer at a time:** (a) the things reduction does not attend an on-line fragment; (b) the seek loop yields
its approach to an on-line target (R111's re-use); (c) a look ends when everything in view is known structure.
**S3, the learned half:** "on structure" as a key of the outcome table, so one peck at a fragment habituates the
category.

## 5. Phase 2 — the phantom

Entered only from a **confirmed** chase that is then lost (the operator's "high certainty that it moved").
- **The belief is a set of particles**, propagated with the thing's last velocity and diffusing with time.
- **Structure bounds it** (phase 1's map): a particle does not pass through a wall or furniture.
- **Negative evidence:** a ToF zone whose ray passes a particle's position and returns from beyond it says the thing is
  not there; that particle loses weight. Per particle, not per voxel (the 4 cm trail was `NULL`).
- **It drives the gaze first** (`★ GAZE`'s learned head), and the walk only to where the mass could be *seen* — never to
  the predicted point, which is what took coasting into walls.
- **Interest spends itself:** the loop's need is the belief mass still reachable or viewable; each failed look removes
  some; the loop releases when it is gone. No give-up timer.
- **Stimulus: a tunnel on the train track** (`playroom_gen.py --tunnel`, off by default, the plain and train rooms
  byte-identical). The train enters; the duck should look at the exit. Then the kicked ball rolling out of view.
- **(d) tests:** the train re-emerges early or late (`--train-phase`); the train stops inside the tunnel (the belief
  should spend itself at the mouths); the tunnel lesioned (no occluder: the chase alone).

## 6. Phase 3 — the ten-minute set, and Pollen

The promoted levers stacked on `★ TURN`, phase 0's instrument re-run; the stack at finding level (n ≥ 20 varied worlds
and the (d) tests — `★ TURN` itself is unrun at that level); the write-up for Pollen under REPORTS.md §9 (their
vocabulary, Track B's intent boundary). Sending it is the operator's call.

## 7. Register rows

O68 (structure), O69 (the phantom), O70 (the ten minutes). O63, O64, O67 carried.

## The chase push, 2026-10-03: practice, the learned lead, coverage, looking around

Design doc §17.104–17.107. **The train room** (`playroom_gen.py --train-room`) gives contact, but practice there reshapes the
walker for a world without still things (first minute 0–5/18) and teaches the chase nothing: the walker is not where the chase
lives (the operator: "chasing is a planning loop at the intent level"). **The learned lead** (the seek loop learns where to aim,
per situation, from its own closing; walker frozen in practice, `--freeze-walker`) is built and tested but starves: 11–21
outcomes in 20 min. **Coverage is the limit**: the train is in the 45° cone ~15 % of the time it is near, in every arm, because
every mechanism needs it seen first. **Looking around while walking** (`--look-around 0.9 2.0`, LA1) ties T11 on every number
with a livelier head: the candidate for the eye (preset beside T11). A 120° field would see the train 2.4× as often: **the
case for camera access** is outreach plan §9 (the camera buffer is not in Pollen's API). Next: the operator's eye on LA1;
the camera request travels with or after PR-2, the operator's call.
**★ LA1 PROMOTED 2026-10-03** on the operator's eye (novelty in the behaviour): T11 + `--look-around 0.9 2.0`.

**★ F5 PROMOTED 2026-10-04** (design doc §17.108–17.109): ★ LA1 + four correctness fixes found while documenting the brain —
the escapes and the stop sweep had mirrored signs, the place map baked unchecked on walks, its pose range clamped at ±1.2 m.
n = 24: every number leans right, none beyond the noise; the operator's eye: "smarter short-term decisions (not turning the
wrong way into a wall), more accurate object interactions." ★ F5 is the demo configuration; the companion document "Inside
the MicroDuck Brain" describes it.

## The resource push (2026-10-04) — resume here for the ARM measurements

Design doc §17.110–17.111. The question: would Pollen read the brain as too heavy for the Radxa Zero 3W (RK3566, 4× A55)?
Measured on the desktop: intent 199 µs a tick (1.0 % of a core), head 8 µs, stop 31 µs; the brains-only process 24 MB
resident. The cloud map is the one heavy module (tall index landed, exact; weighted cast-once −21 %, behaviour tie, awaiting the
eye). **Next, in a fresh context on a machine with sudo over ssh to the Pi 5:** record a tape here
(`ogma_mjhost --level2 … --record-brains f5_s1.tape`, ★ F5's preset argv), copy `cpp_core/`, `mj_host/src/BrainTape.*`,
`mj_host/tools/brain_replay.cpp` and the tape to the Pi, build `ogma_brain_replay` against ogma_core there (the picrawler-dev
branch's Pi build notes: `-j2`, ~10 min for cpp_core), and run `ogma_brain_replay f5_s1.tape --repeat 3` under `ulimit -v`, plus
SoC temperature over the replay. Then a Radxa Zero 3W for Pollen's own numbers. After that: the cloud map's dense-grid rewrite
(exact) and the pipelined clustering (behaviour-tested), both measured on the replay.
