# The intent boundary — design for review (2026-09-03)

**Status: DRAFT for the operator's review. Nothing here is built.**

**What this decides.** How the brain that stands and catches (R19,
[`microduck_standing_report.md`](../reports/microduck_standing_report.md)) becomes the
autonomous layer of a Microduck without displacing Pollen's stack, and how the two tracks of
[the port plan](microduck_port_plan.md#two-goals-and-they-put-the-markov-blanket-in-different-places)
run in parallel from one host. The operator's framing, which this document adopts: the community
stack is *always available*, for rescue and for other behaviours, so the brain can occupy the
middle between "full xaq, no community stack" and "xaq only as the autonomous layer above the
community stack" — and both of those remain reachable from the middle.

---

## 1. The layered body

Three levels, and the rule for each is *who owns the joints, and when*.

| level | what it is | owner | learns? |
|---|---|---|---|
| **0 · safety and servos** | `robotd`'s `safety` holds the only IO write handle; the deadman zeroes velocity if intents stop; in sim, `DuckBody` and the harness | Pollen (theirs) | no |
| **1 · drivers** | the things that move the joints: Pollen's trained policies (`alpha_stand` with a twist is stand *and* walk; sit-stand, ground-pick, roulade, ball-kick are behaviours), **and** our joint-level reflex (R19) | shared; one driver at a time, arbitrated | R19 learns only while it drives |
| **2 · the autonomous layer** | intents in, state out: the thing their `autonomous_behavior.md` says no design doc owns | ours, as a client of their sockets | yes — this is where prediction error chooses |

The port plan named two tracks. This document's contribution is the **arbitration rule at
level 1** that makes them one system:

> The reflex drives while its in-place catch suffices. A driver from the community stack takes
> the joints for a **fall** (rescue, as today), for a **step** (when the reflex reports
> saturation), and for any **behaviour** the autonomous layer asks for (walk, look, pick).
> Learning is frozen in every driver that is not driving. Every hand-off is an event, logged
> and counted; rescues per hour is the crutch meter, and a behaviour is at parity when it runs
> with that driver switched off and the count stays zero.

That rule is the recovery harness we already run, generalised from one trigger (a fall) to three
(fall, step, behaviour). The community stack never teaches: the brain learns nothing while it
drives, and no percept is distilled from it (doctrine §5.6). "Don't run our daemon" stays a
supported posture: a duck without level 2 running is a duck exactly as Pollen ships it.

## 2. The blanket at level 2

The brain's sensory and active states at the intent boundary, read from `duck-ipc-proto`:

| | channel | what carries it |
|---|---|---|
| sensory | joints, targets, gravity, gyro, dead-reckoned odometry, the active policy name, safety and loop state | `robot.state` (subscribe) |
| sensory | joint velocities and currents | `robot.state`, **after the prepared upstream change** (§6) |
| sensory | classified trunk-frame obstacle points (Empty / TooClose / Floor / Hit) | `tof.stream` |
| sensory | health, mode, remote-session flag | `robot.health`, `robot.mode` |
| sensory (later) | ambient sound events, voice tags, nearby ducks by id, RSSI, hand distance | already in the daemon, "logged, no consumer" |
| active | a twist (vx, vy, vyaw) | `robot.move` |
| active | head pose, gaze target | `robot.head`, `robot.look` |
| active | a named behaviour, a pose, a sound | `robot.do`, `robot.pose`, `robot.sound` |
| active | stop | `robot.stop` |

Everything on the sensory side is egocentric: odometry is dead-reckoned from the robot's own
contacts, gravity is measured, obstacle points are in the trunk frame. Nothing is an oracle. The
trained policies sit inside the blanket as actuators, the way a servo driver does; the brain must
still predict the consequences of its own intents and be graded by its own error.

**Reflex saturation is a sensory event.** The level-1 reflex publishes one scalar the level-2
brain can read: how far the in-place catch is from its limit. Today that limit is measured
(R19 catches 2 N shoves and 0.8 N sustained pushes, goes over at 3 N front-to-back), so the first
version of the scalar is the attitude prior's instant error against the angle at which the
catch fails. It is the same signal the consolidation gate already reads.

## 3. What the brain does at level 2, in the rewrite rule's terms

Never a behaviour; always the error a behaviour reduces. Pollen's sixteen states, read that way:

| their state | the prediction error underneath | the prior |
|---|---|---|
| Chill, Nap | none — the rest state; consolidation *is* rest | earned stillness (§7 of the design doc) |
| LookAround, Wander | novelty: the EPM over odometry × ToF has an `is_novel` output, which is a learned novelty grid | a curiosity drive, precision-weighted by need (the Cell report's lesson: curiosity must yield to need) |
| TurnInPlace, obstacle avoidance | a TooClose point where the prior says none | `TooClose` count → 0, weighted by the point's bearing |
| Startle | a contrast spike against the sensory EMA | the same surprise term the EPM already carries (transition surprise) |
| Zoomies, Stretch, Dance, Preen, Ruffle, Sneeze | an energy/mood homeostat off its set-point | interoceptive priors on an energy variable (their "energy/mood model", made a prior rather than a mode) |
| Petted, Held, BallPlay, GroundPick | a contact or presence event and the behaviour it affords | the `do` actuator, chosen by the arbiter |

Their own shape note says it: *presence, mood, and the shared beat are inputs to one brain, not
modes beside it.* The FSM's transitions become the LateralVoter's precision-weighted choice among
loops, and "which mode am I in" becomes the regime EPM's winner, learned. The first behaviour is
the pair **Wander / Chill**, because it is the one whose error is measurable in the simulation
today (novelty over a learned map, and the rest that consolidation earns) and because their doc
says exploration memory is what Wander needs.

## 4. Phases and gates

Each phase is one lever, gain-0-guarded, seed-averaged, with the (a)–(d) bar.

**Phase 0 — the middle, in `mj_host` (the step).** Add a `Walk` driver to the harness: the same
`alpha_stand` network, given a twist. Add the saturation scalar to the reflex's diagnostics.
Arbitration: reflex drives; on saturation, hand to Walk with a twist along the fall direction
for a bounded time (one step), then hand back through the existing settle-and-handback path.
*Gates:* (i) with the step disabled, byte-identical to today; (ii) at 3 N front-to-back, where
R19 goes over 23/36 times, the stepped stance stays up in most of them; (iii) the reflex's own
envelope is unchanged (35/36 at 2 N) — the step must not be *taught into* the reflex, only handed
to. *Metric:* rescues per hour with and without the step driver.

**Phase 1 — the intent-level sensory and active surface, in sim.** A simulated `robot.state` /
`tof.stream` / `robot.move` / `robot.look` inside the host (the ToF reprojector is theirs and
egocentric; the sim scene gets a wall or two). An identification episode at the intent level:
babble twists and gazes, watch odometry, gravity and the obstacle field respond, build the
level-2 self-model the way the joints' was built. *Gate:* the identified move → odometry rows
agree with dead-reckoned ground truth in sign, per axis, the way R19's were checked against the
probe.

**Phase 2 — Wander / Chill.** The novelty EPM over odometry × ToF, the TooClose prior, and
consolidation as rest. *Gates:* (a) nothing in the loop reads world coordinates; (b) intents
reduce the brain's own novelty and TooClose errors, measured; (c) lesioning the ToF prior
produces collisions and lesioning novelty produces a duck that never leaves; (d) relocate the
walls mid-episode and show re-exploration. *The blind metric to add first:* distance travelled
rewards dead drift; use coverage of the learned map plus collisions.

**Phase 3 — `ogma_duckd`.** The level-2 brain as a JSON-RPC client of their sockets, in the shape
of `padd` and `btd`: subscribes to state, sends intents as notifications at their rate, off by
default. Sim first, against a simulated `robotd` (a question to raise with Pollen: a MuJoCo
backend for `robotd` would let every client be tested without a robot, §6). Hardware only after
the operator's eye and Pollen's. Its construction is decided in §4.1.

### 4.1 Decision: a Rust shell around a C++ core (2026-09-03)

`ogma_duckd` is two things with one seam between them:

| part | language | why |
|---|---|---|
| the shell — socket client, handshake, subscribe-and-notify, systemd, health | **Rust**, using their `duck-ipc-proto` crate directly | the protocol types, the API-version `Hello` and the notification idiom come from their code rather than from a hand-written JSON layer that drifts; it is the shape their own clients (`padd`, `btd`) have, and the shape a Rust reviewer would write |
| the brain — `ogma_core`: the modules, the bus, the EPM stack | **C++**, unchanged, as a library | it is the existing engine that drives every other body; the duck gets the same core, not a port of it |
| the seam | a small **C ABI** (`ogma_c.h`): create an instance from a graph JSON, publish sensors, tick, read intents and diagnostics, snapshot and restore | bounded, testable in isolation, and wanted anyway for the picrawler host and for Python tooling |

Why this and not a C++ client or a Python one: language is invisible across the socket, but it
is visible at three seams — the question "why not Rust?" from a Rust shop, the trust
conversation about running a third-party binary on the robot, and the bench (cross-compiling
to aarch64, memory and rate on the RK3566). A Rust shell answers the first at the seam, keeps
the second about permissions and the write handle (their answer, not ours), and leaves only
the third, which is measurement we owe regardless. The daemon then talks to their robot the
way theirs do.

Consequences: the C ABI is the first artefact of phase 3 and gets its own byte-identity guard
(the C++ host and the Rust shell driving the same graph must produce the same JSONL); the Rust
shell is cross-compiled in our CI with their target triple; nothing of this enters their tree.

## 5. Reporting rules that keep the blur readable

1. Every result names the driver of every joint and the count of hand-offs. Rescues per hour
   is the crutch meter.
2. The community stack never teaches; learning is frozen in any driver not driving.
3. The parity ladder is explicit, and getting up is on it: standing (done, 6/6), the catch (half
   the scaffold's envelope), the step (phase 0), getting up (route 1, a lesion test first —
   with rescue always available the brain never experiences the ground-up phase with plasticity
   on), walking (route 1, later).
4. Each track gets its own R numbers in the launcher series so the list stays readable.

## 6. Dependencies on Pollen, and what we ask of them

- **Joint velocities and currents on `robot.state`** — prepared, tested, unsubmitted
  (`state-velocities-currents`). The level-2 brain wants both; the level-1 reflex wants current
  as the load observation the picrawler never had.
- **A way to run a client against a simulated robot.** Their `FakeIo` exercises the loop, not a
  body. Whether that is a MuJoCo backend for `robotd` or a sim harness that speaks their
  protocol is their architectural choice; either helps every client author, not only us.
- **The autonomous design doc they say nobody owns.** Offering to draft it, in their format and
  voice, with this architecture as one implementation of it — see the outreach plan.

## 7. Out of scope, for now

Hardware. Social behaviours (BLE presence, chorale). Audio in either direction. Anything that
changes their locomotion policies. Replacing the harness's rescue with a learned get-up (route 1,
later, and only after a lesion test says the brain can).

## 8. Open questions for the review

1. Phase 0's step: a fixed bounded twist along the fall direction is a *scripted* step handed to
   a *trained* walker. It is legitimate only as a hand-off, not as a reflex the brain owns. Is
   that acceptable as the middle's first capability, with the learned step (route 1) explicitly
   on the ladder?
2. The saturation scalar: the instant attitude error against the measured failure angle, or the
   consolidation gate's own signal? The first is honest to the physics; the second is already a
   learned quantity.
3. Phase 1's simulated ToF: a wall or two in the scene, or Pollen's own arena?
4. Which of Pollen's behaviours (`do`) may the autonomous layer invoke in phase 2 — none, or the
   charm ones (stretch, preen) as the energy homeostat's outlets?
5. `ogma_duckd` lives in our repo; their repo receives only the hooks it needs. Agreed (2026-09-03).
6. The C ABI's surface: the five calls above, or also live parameter mutation (the launcher's
   hot params) and the `lite` diagnostics topic for xaq_voice?
