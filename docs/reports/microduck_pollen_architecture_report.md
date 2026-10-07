# How the Microduck moves: Pollen's runtime, its trained skills, and the reach-down

Status: report · Date: 2026-09-19 · Scope: Pollen Robotics' `microduck` runtime as checked out at commit `b1c2475` (their 0.12 line), read for the things a brain above it needs to know

*A companion to the microduck port plan and to the design record in `microduck/rung2_regime_design.md` §17.39–17.41, where the runner described in §5 is measured. Written for collaborators who will guide what the duck does next and want to know what the robot can already do on its own, and how.*

---

## Executive summary

A Microduck is a 25 cm, 800 g two-legged robot whose fifteen motors are driven, fifty times a second, by small neural networks that were trained in a physics simulation. There is no hand-written walking program on the robot. A client, whether a gamepad, a phone, or a brain like ours, never touches a motor: it sends an intent, such as "go this fast", "look there", or "do the kick", and the robot's control loop turns that intent into motor targets through whichever network is in charge that tick.

This report explains that runtime for readers outside robotics and machine learning, because the boundary it draws is the one our work sits on. Three things are worth taking away.

**Every network sees the same picture and answers the same question.** The picture is a list of 61 numbers: how the body is tilting and turning, where each joint is and how fast it is moving, what the network said last time, and a thirteen-number command block that carries the intent. The answer is 14 numbers, one offset per joint from a fixed "home" posture. Walking, standing, kicking, rolling and reaching down are all networks of this one shape; they differ only in what they were trained to do and in what the command block means to them.

**A skill is a window during which one of those networks is put in charge.** The kick network drives the joints for half a second, the forward roll for one second, and then the gait takes over again. The network is told nothing about the ball: its command block is all zeros, and it runs at the softer "standing" tuning it was trained against. Which network is in charge is decided by a fixed priority chain: roll, then kick, then reach-down, then sit and stand, then the walker.

**The reach-down, which we call the peck, is the odd one out.** It is not a window but a phase. The control loop advances a clock from 0 to 0.7 over a four-second period, about 2.8 s, and writes the cosine and sine of that clock into the command block's velocity slots. The network was trained to produce the crouch-and-reach motion as a function of that clock, so the robot's daemon drives it by turning the dial, not by sending a trajectory. Nothing on the robot knows whether a thing was picked up; the motion is open-loop from the daemon's point of view. In simulation nothing can be picked up at all, since no simulated model has the beak's hinge.

Our simulator reproduces this runtime rather than replacing it. Measured over six seeded fifteen-minute runs, a kick requested at standing never topples the duck and moves the thing in front of it on one request in five, and a reach-down requested the same way moves it about as often. The number that matters for guiding development is the shape of the boundary, not those rates: what a brain can ask for is exactly the list in §4, and what it learns is what each request does.

## 1. The robot and the daemons that run it

The Microduck's computer is a small ARM board. Its work is split into seven long-running programs, called daemons, that talk to one another through JSON messages over local sockets. Only one of them, the robot daemon, ever commands a motor. Fifteen servos and the inertial sensor share one serial cable, and the robot daemon's 50 Hz loop owns it. The others are transports and sensors: a gamepad reader, a Bluetooth bridge for the phone app, a camera and audio pipeline that also carries remote sessions, a configuration service, an update service, and a small daemon that reads the head's depth sensor and publishes its 8×8 frames.

Three of the daemons are deliberately kept independent of the robot daemon so that a robot whose control loop will not start can still be reconfigured, updated, or rolled back. Releases are installed as whole directories and swapped by moving one link; an update that fails a health check is put back automatically.

Two sensors matter for what follows. The inertial sensor gives the body's rotation rate and the direction of gravity in the body's own frame. The depth sensor in the head is a time-of-flight array: 64 zones in an 8×8 grid, each reporting the distance to the nearest surface in its cone, at up to 15 Hz. The runtime reduces each zone to one of four classes, empty, too close, floor, or hit, and offers a hand tracker over it; it does not cluster, size, or name anything.

## 2. One network shape for everything

Every trained behaviour on the robot is a network of the same shape, exported from Pollen's training repository, where it was trained with reinforcement learning in a MuJoCo simulation. In this report a network of that kind is a policy.

**What a policy sees.** Its input is 61 numbers, assembled fresh every tick:

| rows | count | what |
|---|---|---|
| 0–2 | 3 | rotation rate, body frame |
| 3–5 | 3 | direction of gravity, body frame (a unit vector) |
| 6–19 | 14 | each joint's position, as an offset from the home posture |
| 20–33 | 14 | each joint's velocity |
| 34–47 | 14 | the policy's own previous output |
| 48–50 | 3 | the command: forward speed, sideways speed, turn rate |
| 51–54 | 4 | the command: four head-joint targets |
| 55–60 | 6 | the command: body x, y (always zero), height, roll, pitch, yaw (always zero) |

The beak's hinge is the fifteenth motor and is excluded from the rows the policies see and act on. The head's targets travel inside the command block; they are not added to the output afterwards, which would bend the head twice. Body x, y and yaw are fixed at zero because the training environment never bound them, so all zeros is the nominal encoding rather than a placeholder.

**What a policy says.** Its output is 14 numbers. The motor target for each joint is the home posture plus an action scale times that number: 0.9 for the walker, 1.0 for the standing policy and the skills. A first-order low-pass filter smooths the targets, at the strength the policies were trained with, since a mismatch there degrades the transfer from simulation to the robot.

**What varies.** Only the training, and what the command block means. The walker was trained to follow the three velocity commands and recovers from falls on its own; the standing policy holds the pose; the kick, roll and reach-down policies were trained to produce one motion each, with the command block used as a clock or left at zero.

## 3. The tick

Fifty times a second the robot daemon runs one tick, in a fixed order:

```text
skill windows   advance or expire (roll window, kick timer, reach-down phase, sit and stand rise)
command         the client's smoothed intent, re-encoded for the active skill
network         roll > kick > reach-down > sit and rise > stand-by-magnitude > walk
action          one forward pass of the chosen policy
targets         home posture + action scale × action
filters         the low-pass on head and legs
safety          the layer that decides what is actually sent to the motors
```

The priority chain is the only scheduler. A roll in flight outranks a kick, a kick outranks a reach-down, and so on down to the walker, which is chosen whenever nothing else is running and the twist command is above a small magnitude; below it, the standing policy holds the body. "Stand-by-magnitude" is that rule.

Two subtleties Pollen carried over from their prototype on purpose, because the skills were tuned against them: a kick runs at the standing policy's tuning, with the softer motor gain and the full action scale, because its command block is all zeros and that is exactly what the standing transition keys on; and the rise out of a sit does the same.

The safety layer sits after the targets. Its late verdict, "fallen", is gravity pointing more than about 60° from upright for 200 ms; its early one, a fall predictor, fires around 26° while still tipping, so the motor gains can be softened before impact. The walker's own network then recovers the body; a client is not involved.

## 4. What a client can ask for

The robot daemon answers a small set of named requests. Those that shape behaviour:

| request | what it does |
|---|---|
| move | a twist: forward speed, sideways speed, turn rate |
| head | four head-joint targets, or **look**, a gaze point in the body's frame resolved by the daemon's own kinematics |
| pose | the standing body pose: height, roll, pitch, within ±0.26 rad and 2.5 cm, glided or snapped back |
| mouth | the beak, 0 to 1 |
| sound | one of seven voice tags (alarm, greet, inquire, peck, chirp, coo, wheee), held while the request keeps arriving |
| do | a skill by name |
| state, health, subscribe | what the robot senses and how it is doing, at about 1 Hz for health |

Requests are intents. The daemon smooths them, re-encodes them for whatever is running, and the safety layer has the last word. There is no request that moves a single joint, and none that streams a trajectory.

## 5. Skills, and the reach-down in particular

A skill is a policy plus a definition: a name, the network file, how many seconds it runs, an optional twist to feed while it runs, an optional "unwind" twist to drive for some seconds afterwards before handing back to the gait, and whether holding the request at the end of the window chains another. The release ships a table of them, and a robot's configuration can override any entry by name or add a new one from any policy file of the same shape. Asking for one is `do` with its name; the daemon answers with the list of names it has.

The shipped set, with their durations:

| skill | runs for | how it is driven |
|---|---|---|
| kick, left or right | 0.5 s | a window; the network sees an all-zero command |
| roulade, the forward roll | 1 s, chains while held | a window; it rolls, ends on the floor and rises |
| ground pick, the reach-down | about 2.8 s | a phase, below |
| sit and stand | until asked again | a posture flag in the command's forward-speed slot |

**The reach-down.** The ground-pick policy is driven by a clock rather than a window. When a client asks for it, the daemon starts a phase at 0 and advances it every tick by the tick length over a four-second period; the pick ends when the phase reaches 0.7. Each tick the command block's three velocity slots carry cos(2π·phase), sin(2π·phase) and 0, with the head and body slots zero. The policy was trained to produce the whole crouch, reach and recover as a function of that angle, so the daemon does not know, and need not know, where in the motion the body is: it turns the dial and the motion follows. The pick runs at its own action scale and gain ratio, both 1.0 by default, and it blocks everything, including the walker, until the phase ends; a pick can even cut in on a kick's tail.

Whether anything is picked up is not part of the mechanism. The beak's hinge is the one motor the policies neither see nor drive, and in simulation there is nothing to close: no simulated model of the duck carries that hinge. A reach-down is best read as a reach, a lean onto whatever is in front of the feet, that ends standing.

**How our simulator runs the same thing.** Our host mirrors the daemon: a requested skill by name opens a window of the skill's duration in which that policy drives every joint at the standing scale with an all-zero command, or, for the reach-down, with the phase encoding above; the gait, or our own standing controller, resumes from wherever the body was left. The policy files are the ones Pollen ships, fetched by hash. The one difference of substance is when a request is honoured: a kick fired into a full-speed walk toppled the simulated duck on 28 % of kicks (n = 25, six seeds), because the window's zero command from a walking state is a stumble, so our host fires a requested skill from standing, after its stop's hand-back, where the kick never falls (n = 27). On the robot the same request is `do`, and the daemon's own tuning decides.

## 6. What this means for guiding the work above it

The boundary is narrow and it is the right one. A brain above this runtime does not learn to walk; it learns what to ask for. Its actions are the requests in §4, its senses are the state the daemon publishes plus the depth frames, and the interesting learning is what each request does to the world: whether a kick moves the thing in front of the feet, whether a reach-down does, whether a walk toward a remembered place ends up there. Measured in our simulator, a thing kicked from standing moves by more than five centimetres after one request in five (blocks one in four; n = 27 kicks over six seeds), and a reach-down about as often (n = 13); those are the rates a learner sees, and they are the rates a learner improves by choosing when and from where to ask.

Three consequences for anyone designing behaviours:

- **Nothing on the robot adapts.** The policies are frozen; the daemon's tuning is a configuration file. Adaptation, if it is to exist, lives in the client.
- **The command block is the whole vocabulary.** Head targets, body pose and twist are the only continuous things a client can shape, and the skills are discrete names. A new motion is a new policy file trained next door, not a new request.
- **The reach-down's clock is the daemon's, not the client's.** A client cannot pause a reach-down halfway or aim it; it can only ask, and ask again.

## 7. Claims we are not making

- We have not measured Pollen's runtime on the physical robot; every number above comes from our MuJoCo host running their exported policies, and the rates quoted are signals from six seeds, not findings.
- We do not claim the reach-down grasps anything, in simulation or on the robot; the beak is outside the policies' reach and outside every simulated model.
- We do not claim to know the training details of the skills beyond what the runtime and its comments state: which command slots each policy reads and the phase convention of the pick.
- The daemon's fall handling on the robot is Pollen's; our simulator's recovery is our own scaffold and stands in for it.

## 8. Next work

- Measure the same requests on the physical robot through `do`, with the depth sensor timestamped against the head's angle, to see whether the simulated rates transfer.
- Give the peck an outcome the learner can read reliably: the thing seen again from a step back, which in simulation raised observed outcomes from five of twenty-six to twelve of twenty-seven.
- Ask Pollen for the one request the robot lacks that a brain would use every tick: a streaming voice verb over their synthesizer, which today only the daemon can drive.

## Appendix: where each fact lives

For a replicator. Paths are in Pollen's repository unless marked ours.

| fact | where |
|---|---|
| the daemons, the sockets, what survives an update | `docs/design/architecture.md` |
| the 61-row observation and the 13-row command block | `duck-control/src/obs.rs` (the layout comment) |
| the tick order, the priority chain, the kick-at-standing-tuning note | `robotd/src/control.rs` (header) |
| the reach-down's phase encoding and its end phase 0.7 over a 4 s period | `robotd/src/control.rs` (the ground-pick branch), `robotd-params/src/lib.rs` |
| the skill definition: name, file, duration, command, unwind, chain | `duck-ipc-proto/src/lib.rs` (`SkillParams`), `robotd-params/src/lib.rs` (the built-ins) |
| the client requests | `duck-ipc-proto/src/lib.rs` |
| the fall verdict and the predictor | `duck-control/src/safety.rs`, `duck-control/src/fall.rs` |
| the shipped policy set | the Hub repository `pollen-robotics/microduck-policies`, tag `v1` |
| our host's runner and its measurements (ours) | `mj_host/src/main.cpp` (the skill window), design record §17.39–17.41 |
