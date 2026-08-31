> **LIVING DOC — the single reference for the Microduck project.** Started 2026-08-30 on branch
> `microduck`, cut from `master`. Update it in place — do not fork a second port doc.
>
> **Status 2026-08-31: their sim runs here, nothing of ours is built yet.** G1/G3/G4 measured
> and passing, G2 restated after it failed and taught us something (§Measured). Upstream is
> cloned *outside* this repo — `/home/xaqmusic/microduck` at `590b986` and
> `/home/xaqmusic/microduck_rl` at `d424a0c` — so it stays a clean base to PR from. The CPU
> sim venv is `microduck_rl/.venv` (mujoco 3.12.0, onnxruntime 1.29.0, numpy 2.5.2; no mjlab,
> no CUDA).
>
> **Scope: simulation only.** No hardware, no `robotd`, no servos, until the controller is
> understood in this body. The picrawler hardware build occupies the bench and runs in parallel
> on `picrawler-dev`; the two **share no files** (§Coordination).

# Microduck port plan

## Two goals, and they put the Markov blanket in different places

**Goal 1 — improve the framework.** Use a body that suits active inference better than the
picrawler does, and find out what the substrate can do when its motor loop actually closes.
Inward-facing; the audience is this repo.

**Goal 2 — give the Microduck community a foundation for autonomous behaviour.** Pollen's
autonomous stack is unwritten and unowned. Their own
[`docs/ideas/autonomous_behavior.md`](https://github.com/pollen-robotics/microduck/blob/main/docs/ideas/autonomous_behavior.md)
opens: *"The brain is the biggest untracked gap in the parity audit… no design doc owns it
yet."* That is an opening to contribute higher-level behaviour built on prediction error,
**to people who may well want to keep their RL stack for locomotion** — and should be free to.

These are not the same project wearing two hats. They sit at different blanket positions:

| | **Track A — framework** | **Track B — contribution** |
|---|---|---|
| Blanket at | the **joints** | the **intent boundary** |
| The body is | servos and physics | servos, physics, **and the trained policies** |
| Sensory states | measured `q`, `q̇`, load, gravity, ToF | `robot.state` + `tof.stream` + `robot.health` |
| Active states | 14 joint targets | `move` / `head` / `look` / `pose` / `do` / `sound` |
| The brain must learn | how to move at all | where to go, what to attend to, what it wants |
| Ships as | `mj_host` experiments | `ogma_duckd`, a client daemon that forks nothing |

**The tension is real and gets named rather than smoothed over.** Doctrine §5.7 forbids
injecting a rhythm or imposing a coordination topology, and a walking policy is exactly an
imposed gait. Track A honours §5.7 at the joints. Track B accepts the gait **as part of the
body**, the way a servo driver is part of the body, and puts the brain above it. Both positions
are defensible; holding them at once without saying so would not be.

What makes Track B legitimate rather than a shortcut: nothing in its sensory surface is an
oracle (odometry is dead-reckoned from the robot's own contacts, gravity is measured, ToF is
egocentric), no reward is shaped, and the brain still has to predict the consequences of its
own intents and be graded by its own error. Riding a policy is not distilling one — §5.6 bans
copying a teacher's percept, not standing on a working actuator.

**Both tracks share everything that matters**: the host, the sensory reduction, the EPM /
consensus substrate, and the diagnostic surface. Track B is not a fork of the work. It is the
same brain with its blanket drawn one layer out.

---

## Context

[Microduck](https://github.com/pollen-robotics/microduck) is a ~800 g, ~25 cm bipedal robot
from Pollen Robotics: fifteen Dynamixel XL330 servos and an IMU board on one 1 Mbps bus,
driven by a 50 Hz Rust control loop on a Rockchip RK3566. Its policies are trained next door
in [`microduck_rl`](https://github.com/pollen-robotics/microduck_rl) — MuJoCo + PPO, exported
to ONNX. Both repos are Apache-2.0.

**The reason to care is not the robot, it is the sensorimotor surface.** Three things this
body has that the picrawler does not, each of which the ledger has been fighting:

1. **Measured joint position and velocity.** `pi_host`'s wire protocol is PWM pulse counts —
   the picrawler has no encoders. MotorEPM's whole premise is
   `ξ(t+1) = x(t+1) − x̂(t+1)` against a forward self-model `x̂ = A·y + b`. With `x` sourced
   from *commanded* angles the self-model learns the identity map and the TLE collapses: a
   §3.2 tautology, and the reason the picrawler's deepest result is currently sim-only for a
   hardware reason rather than a scientific one. The duck returns measured `positions` and
   `velocities` in the same bus transaction as the IMU.
2. **Per-joint current (mA), every tick.** The ledger names this gap explicitly — the
   swing-detector post-mortem ends *"answering 'is this foot loaded' needs a load observation,
   which the bus does not have"*, and §6 names `joint_torque` as "the load observation Walknet
   never had". Here it is a first-class channel. `stroke_load_gain` was measured against a
   *simulated* hip1 load; on this body it would be a real one. **Inside the control loop
   only** — `robot.state` does not publish it, which is a Track B gap and a contribution
   opportunity (§Sensing).
3. **An 8×8 depth matrix with the geometry already done.** `kinematics::tof::Reprojector`
   turns the VL53L8CX's 64 slant ranges into trunk-frame points classified
   `Empty`/`TooClose`/`Floor`/`Hit`, using head FK and projected gravity to reject the floor.
   The belly-rangefinder lesson (doctrine §1: a missing *observation*, not a missing policy)
   generalises straight in, at 64 zones instead of one beam.

Two more, smaller: runtime-settable servo P gain gives the adaptive-gains substrate plan an
actuator it has never had, and contact odometry's anchor-foot switch is a body-derived phase
event — §0 rule 3's "feed it phase", from the body rather than from a clock.

**Timing.** The picrawler hardware bring-up occupies the bench. This work is pure simulation
and needs no hardware, so the two proceed in parallel without contending for the robot.

**And the community half is well-timed too.** The autonomous stack does not exist yet, so
there is nothing to displace and no incumbent to argue with. A contribution arriving after a
16-state machine had shipped would be asking people to throw work away; arriving before it
is offering a foundation.

---

## Decisions taken 2026-08-30

| Decision | Choice | Why |
|---|---|---|
| **Sim engine** | MuJoCo, using Pollen's MJCF unmodified | Their body model is CAD-exported from Onshape, mesh-accurate, and validated by transfer to a real robot. Rebuilding it in Godot would reproduce that badly. [`picrawler_sim2real_port.md`](picrawler_sim2real_port.md) records what happens when a hand-derived body disagrees with the real one |
| **Host** | A new **`mj_host/`**, peer to `godot_host/` and `pi_host/` | `OgmaInstance.hpp:60` already says the Bus is owned by "the host (Godot Host, **HAL Host**, Debug Host)". A second host is the anticipated shape, not a new one |
| **No Godot for the duck** | Confirmed | The Godot dependency is the *picrawler body*, not the brain. `xaq_inspector` and `xaq_voice` reach the brain over ZMQ (`tcp://127.0.0.1:7400/7401`), so the operator's UI-first promotion gate survives the engine change untouched |
| **No OgmaBrain refactor** | `mj_host` links `ogma_core` directly | `OgmaBrain.cpp` is 2 197 lines of Godot `Variant` marshalling that every picrawler A/B depends on. `mj_host` needs none of it: it can hold an `OgmaInstance` and call it in plain C++. Extracting a shared "BrainHost" would be a large refactor of a load-bearing file, for a duplication that is small and honest |
| **Head and neck in the motor loop** | In, from run one — two `MotorEPM` instances (legs 2×5, head 1×4) | The head chain is 38 % of the robot's mass with measured CoM authority, and Pollen's own PPO policy found it as a counterweight unprompted (§"The head is 38 % of the mass") |
| **Standing as scaffolding** | Accepted — `alpha_stand` / StandUp used as the **reset mechanism**, never as a controller (A2) | Gives the brain the continuous-reset harness the picrawler only has in sim, on a body that would otherwise stop at the first fall. Named a scaffold, with a de-scaffolding path |
| **Scope** | Simulation only, until the controller is understood in this body | No hardware contention with the picrawler bring-up, and no transfer claim to defend before there is something to transfer |

### Added 2026-08-31

| Decision | Choice | Why |
|---|---|---|
| **Two tracks, one substrate** | Track A at the joints (framework), Track B at the intent boundary (contribution) | Different audiences, different blanket positions, shared host and shared EPM stack. §"Two goals" |
| **The RL stack is not the adversary** | Track B **augments** it. A user keeping PPO locomotion loses nothing | Their locomotion works and transfers. Competing with it would be picking the one fight we would deserve to lose, and it is not the gap |
| **Track B forks nothing** | `ogma_duckd` is a JSON-RPC client of the existing sockets, in the shape of `padd` and `btd` | Their architecture already says clients send intents and `robotd` stays authoritative. Being a well-behaved client *is* the design, and it makes the contribution reviewable |
| **Off by default** | Not running `ogma_duckd` leaves a duck behaving exactly as today | The gain-0 guard (§3), applied to a community contribution. A brain nobody asked for must be invisible until asked for |

---

## The body of record

Everything here is read from Pollen's own source, not transcribed from documentation.

### Actuation — 14 policy joints, position control

Joint order (`duck-control/src/model.rs`, `JOINT_NAMES`; index 9 `mouth` is excluded from
every policy and from the action vector):

| idx | joint | idx | joint | idx | joint |
|---|---|---|---|---|---|
| 0 | `left_hip_yaw` | 5 | `neck_pitch` | 10 | `right_hip_yaw` |
| 1 | `left_hip_roll` | 6 | `head_pitch` | 11 | `right_hip_roll` |
| 2 | `left_hip_pitch` | 7 | `head_yaw` | 12 | `right_hip_pitch` |
| 3 | `left_knee` | 8 | `head_roll` | 13 | `right_knee` |
| 4 | `left_ankle` | *9* | *mouth (excluded)* | 14 | `right_ankle` |

Home pose (`DEFAULT_POSITION`, radians) is mirrored left/right and must match the sim's
`HOME_FRAME` exactly — the policies observe joint position *relative* to it, and Pollen's own
test `home_pose_legs_are_mirrored` pins the mirror.

### ★ The head is 38 % of the mass, and it is this body's balance actuator

Operator observation 2026-08-30, on watching the robot: *the head appears to act as a
stabiliser.* Checked against the MJCF inertials rather than left as an impression, and it is
correct by a wide margin.

Mass, summed from `robot_walk.xml`'s `<inertial>` elements (737.2 g modelled, ~800 g shipped
with battery):

| Group | Mass | Share |
|---|---|---|
| neck + head chain | **279.9 g** | **38.0 %** |
| both legs, all ten joints | 258.1 g | 35.0 % |
| trunk alone | 199.2 g | 27.0 % |

**The head chain outweighs both legs put together.** The head shell (`jaw_soft`, 188.8 g) is
the second-heaviest single body in the robot, within 10 g of the entire trunk, and it hangs at
the end of a four-DoF articulated boom.

Forward kinematics at the `STAND` keyframe, in the trunk frame:

- head-chain CoM sits **143 mm** from the rest-of-robot CoM;
- swinging `neck_pitch` + `head_pitch` by ±0.5 rad (±29°) moves the **whole-robot CoM by
  8.8 mm horizontally** (+8.7 mm forward / −7.2 mm back).

For scale: Pollen re-baselined their entire home pose — and retrained against it — for a **5 mm**
trunk shift, *"so the CoM is over the ankle axis; the old pose biased the robot backwards"*
(`duck-control/src/model.rs`). **A head-pitch swing has more CoM authority than the pose change
they considered worth a re-baseline.**

**And their own trained policy found it without being told.** The `HOME_FRAME` comment in
`microduck_rl` records that the pre-STAND2 pose *"biased the robot backward **and made the
standup policy droop its head forward as a counterweight**"*. A PPO agent, given no instruction
about the head, discovered it as a balance actuator.

That last paragraph is the ledger's **authority check** — *"before building a lever, measure
whether the actuator has authority over the target variable"* — passing before a line of code
is written, on evidence from someone else's experiment. It settles the head/neck question
(§A1): **the head and neck joints are in the motor loop from the first run.** Excluding
them would hand the brain a body whose largest control authority over its own centre of mass is
held out of reach — the exact failure the ledger records three times over as a lever aimed at an
actuator that could not move its target.

### ⚠ Sensing — and the two surfaces are NOT the same

`robotd` reads more than it publishes. Which channels a brain can have depends on where it
stands, and this was checked against `duck-ipc-proto/src/lib.rs` rather than assumed.

| Channel | Width | Inside the loop (Track A) | On the wire (Track B) |
|---|---|---|---|
| joint position | 15 | ✔ `Sensors.positions`, measured | ✔ `robot.state.joints`, measured |
| joint **target** | 15 | ✔ | ✔ `robot.state.targets` — tracking error for free |
| joint **velocity** | 15 | ✔ `Sensors.velocities` | ✘ **not published** |
| joint **current** | 15 | ✔ `Sensors.currents_ma` | ✘ **not published** |
| projected gravity | 3 | ✔ | ✔ `robot.state.safety.gravity` |
| gyro | 3 | ✔ `ImuData.gyro` | ✘ (quat only, via `theremin`/none) |
| contact odometry | 4 | ✔ in-loop struct | ✔ `robot.state.odom` — position + yaw |
| ToF depth | 8×8 | ✔ | ✔ `tof.stream` → `TofFrame{distance_mm, status}` |
| battery volts | 1 | ✔ `SlowSensors.volts` | ✔ `robot.health.battery`, ~1 Hz poll |
| servo temperature | 15 / hottest | ✔ per-joint | ✔ `robot.health.motors`, hottest only |
| which policy drove | 1 | ✔ | ✔ `robot.state.policy` — `walk`/`stand`/`held` |
| **intent clipping** | — | — | ✔ `robot.state.move.limited_by` |

**Track B loses velocity and load, and that is the honest cost of standing outside the loop.**
Velocity can be differenced from `joints` at 50 Hz. Load cannot be recovered at all: it exists
in `duck_control::Sensors` and stops there. That makes "expose velocities and currents on
`robot.state`" a precise, well-motivated upstream contribution rather than a wish
(§Contributing).

**Two things Track B gets that Track A does not**, and both are gifts:

- **`move.limited_by`** — the brain asks for a twist, and the body reports what it *applied*
  and which safety rule clipped it. That is an action-outcome mismatch delivered on the wire:
  prediction error about one's own agency, already measured by someone else.
- **`policy`** — a discrete label for which network is driving. Free context, and exactly the
  kind of low-cardinality signal a slow EPM wants alongside a continuous stream.

Note what is **absent from both**: there is no god's-eye anything. The picrawler's legal
foot-height signal is `feet_y_gravity_cmd`, FK from *commanded* angles, because there are no
encoders. On the duck the same FK runs from *measured* angles, and the "commanded ≈ actual"
assumption disappears. Odometry is dead-reckoned from the robot's own foot contacts in a frame
chosen at boot, which is the same Markov-compliant status doctrine §1 grants dead-reckoned
own-yaw.

### Interoception is real on this robot, not a modelling choice

Battery voltage and servo case temperature are **physiological state**, not a simulated scalar
standing in for one. A `HomeostaticDrive` here regulates something that genuinely degrades: a
duck that walks all afternoon has a sagging pack and hot knees, and both are measurable.

Pollen's plan calls for an "energy/mood model" driving state choice. They have the energy
already; it is on `robot.health`. This is the doctrine's own §1 step 2 — *check the sensory
channel first* — coming out in our favour for once, because the channel is already there.

### ★ Measured 2026-08-31 — the sim runs, and two premises were wrong

First contact with the real thing. Upstream cloned to `/home/xaqmusic/microduck` (`590b986`)
and `/home/xaqmusic/microduck_rl` (`d424a0c`); CPU MuJoCo 3.12.0 + onnxruntime 1.29.0 in a
venv at `microduck_rl/.venv`. No mjlab, no CUDA, no `uv` — `infer_policy.py` imports only
`numpy`, `mujoco` and `onnxruntime`, so the whole training stack is unnecessary to *run* a duck.

**G1 — PASS.** `scene.xml` and `scene_walk.xml` both load unmodified:

| | `scene.xml` | `scene_walk.xml` |
|---|---|---|
| `nq` / `nv` / `nu` | 21 / 20 / 14 | 21 / 20 / 14 |
| bodies / geoms / sensors / keyframes | 16 / 82 / 6 / 4 | 16 / 76 / 6 / 4 |
| `opt.timestep` | **0.002 s** | 0.002 s |

Two numbers the plan needed and now has. **G3's substep count is 10** (500 Hz physics ÷ 50 Hz
brain). And **G4's mapping is the identity on this model**: `ctrl[i]` drives joint `i`, in
exactly `JOINT_NAMES`-minus-mouth order. The name-lookup requirement stands anyway, because an
identity that holds today is not a contract.

**G2 — the original wording was wrong, and the correction is a finding.** Holding the `STAND`
keyframe's own `ctrl` for 3 s topples the robot to **81° tilt at every noise level including
zero**. That is not a harness fault: tilt reads 0.00° at t=0, the fall is gradual (10° by
0.4 s), and the actuators peak at **0.13 N·m against a 0.96 N·m limit** — they are not
saturating, they are *soft*. At kp×5 the same passive hold stands at 1.6°.

**This body has no passive standing equilibrium. It is always actively balancing.** Which is
consistent with everything else here: a robot carrying 38 % of its mass on a four-DoF boom
(§"The head is 38 % of the mass") is an inverted pendulum that must be ridden, not a tripod
that can be parked.

Consequences, both of which change the plan:

1. **S1's premise is void.** "Hold `key STAND`'s ctrl with no brain" is not a baseline; it is a
   robot falling over. S1 becomes "hold with the *standing policy*", which measures at
   **tilt 0.47°** and is robust to 0.03 rad init noise across seeds.
2. **Track A's brain must balance from tick one.** There is no passive fallback to degrade to
   while the motor loop learns, which sharpens the A1 warning rather than adding a new one.

**And the XML actuator cannot walk, which promotes the BAM question.** Same policy, same
command, only stiffness differing:

| arm | commanded stride (ptp) | joint stride (ptp) | tracking error | displacement, 15 s |
|---|---|---|---|---|
| `alpha_walking`, vx 0.15, **stock kp** | 0.224 rad | 0.118 rad | **0.213 rad** | **0.009 m** |
| same, kp×5 | 1.066 rad | 0.943 rad | 0.326 rad | **1.595 m** (0.106 m/s) |
| stock kp, vx 0.30 | 1.430 rad | 0.627 rad | 0.406 rad | 1.951 m (0.130 m/s) |

The policy *is* commanding a gait at stock gains; the joints deliver about half of it and lag
by 12°, and the duck steps in place. A large enough commanded stride escapes the attractor,
which is why vx 0.30 walks where vx 0.15 does not.

**So option 1 of the BAM spec is adequate for posture and NOT for locomotion**, where that
spec claimed it was adequate for everything through S3. Any locomotion claim needs BAM or a
validated stiffness correction; standing, balance and head-use work can proceed as-is. Record
which arm a result came from, exactly as the picrawler records ghost-vs-solid chassis.

**Their Rust test suite is green here**: `duck-control` 60 passed, `duck-ipc-proto` 49 passed,
0 failed, no warnings, on rustc 1.98.0. That is the baseline the `robot.state` PR moves.

### The sim model — what is actually in the MJCF

Checked against `microduck_rl/src/mjlab_microduck/robot/microduck/`:

- `robot_walk.xml` — 428 lines, 85 geoms, 38 mesh assets (STL, in `assets/`), full inertials
  from Onshape, joint ranges, **14 `position` actuators**. Self-contained.
- `robot_allcollisions.xml` — the same body with the full collision set; the standup /
  ground-pick / roller tasks use it. **This is the one we want**, because a brain that
  destabilises on purpose will put the trunk on the floor and the walk model does not have
  geometry there.
- `scene_walk.xml` / `scene.xml` — floor plane, lighting, and four keyframes: `INIT`,
  `STAND` (the STAND2 pose, CoM over the ankle axis), `SIT`, `FOLD`.
- `sensors.xml` — `framequat`, `gyro`, `velocimeter`, `accelerometer` on the `imu` site,
  plus `subtreeangmom`.
- `joints_properties.xml` — the actuator default classes. The live one is `chosen_actuator`:
  `joint damping=0.053 frictionloss=0.0048 armature=0.0018`,
  `position kp=0.55 kv=0 forcerange=±0.96 ctrlrange=±10`.

**⚠ The BAM actuator model is not in the MJCF.** `microduck_rl`'s headline actuator
fidelity — the voltage-controlled XL330 model with Coulomb / Stribeck / load-dependent
friction — comes from the Python package `better-actuator-models` and is installed onto the
model at runtime by mjlab. What a plain-MuJoCo host gets is the XML `position` actuator above.
Same for the ±1° backlash hinges (a post-processor, `add_backlash.py`, which *does* emit
committed XML — `robot_walk_backlash.xml` — so backlash is available; BAM is not). See
[SPEC — actuator fidelity](#spec--actuator-fidelity-the-bam-question).

---

## What we take from Pollen, and what we refuse

`microduck_rl` is three separable layers. Naming them is what keeps the RL out.

| Layer | Contents | Verdict |
|---|---|---|
| **A — the physics** | MJCF from Onshape, mesh collision, inertials, joint ranges, actuator classes, backlash hinges, the scene and its keyframes | **Take all of it.** Reward-free by construction. It is a body model, and it is the artefact the picrawler port had to derive from a tape measure |
| **B — the deployment contract** | `obs[1,61] → act[1,14]` at 50 Hz, joint order, `HOME_FRAME`, `action_scale`, low-pass α | **Take the joint order, the home pose and the rate. Refuse the observation vector** (below) |
| **C — the RL** | PPO, reward terms, curricula, command sampling, domain randomisation *as a training device* | **Refuse.** §5.1. We do not need it |

### The 13-slot command block: a trap on one side of it, the interface on the other

The 61-D observation is *almost* clean. Gyro (3), projected gravity (3), joint position
relative to home (14), joint velocity (14), previous action (14) are all egocentric and legal.
Everything hinges on the last thirteen slots, `[vx, vy, vyaw, head(4), body(6)]`.

That block is a commanded set-point. **Which direction it points decides which track you are
on**, and the same thirteen numbers are a mistake read one way and the contribution read the
other:

| | Command block as… | Verdict |
|---|---|---|
| **Track A** | an **input** to the brain | ✘ The steering-script instinct from doctrine §1's table, arriving through a door marked "sensor". A set-point is not an observation |
| **Track B** | an **output** of the brain | ✔ Exactly what `robot.move` / `robot.head` / `robot.pose` accept. The brain's active states, in the units the body already speaks |

Practically, for Track A: **`mj_host` publishes the 48 proprioceptive dimensions and never the
command block.**

For Track B the block is the whole point. The brain emits a twist, the body walks; the brain
predicts what walking will do to its ToF, its odometry and its gravity vector, and is graded on
the difference. `robot.move` even returns what was *applied* versus what was *requested*
(§Sensing), so the loop closes without our building anything to close it.

---

## Contributing to Microduck — augment, fill, educate

Nobody has asked us for this, and that shapes the posture. A contribution earns its place by
filling a gap its authors have already named, in the shape their architecture already
prescribes, without asking anyone to give up something that works.

### What we never touch

| Theirs | Why it stays theirs |
|---|---|
| **Locomotion policies and the RL pipeline** | They work, they transfer, and they are the product of a measured sim2real recipe we did not build. Competing here is the one fight we would deserve to lose |
| **The skills** — ground pick, kicks, roulade, sit ↔ stand | Trained, tuned, and exposed through `robot.do`. The brain decides *when*; it does not reimplement *how* |
| **The safety layer** | `robotd` is authoritative on fall detection, thermal limits and joint limits. Their invariant, and the right one. A brain that could bypass it would be a liability, not a feature |
| **`updaterd`, `configd`, `btd`, `mediad`, `padd`, `tofd`** | Transports and infrastructure. Untouched, so a robot running our brain is still a robot you can update and recover |
| **The voice** — sounds, chorale, theremin | Charming, working, and already better than anything we would write |

### The gap we fill, and it is the one they named

Their `autonomous_behavior.md` sketches a 16-state machine (Chill, LookAround, Wander,
TurnInPlace, Zoomies, Startle, Stretch, Ruffle, Preen, Sneeze, Dance, GroundPick, Nap,
BallPlay, Petted, Held) over an energy/mood model, a novelty grid, and a set of bespoke
detectors. Every one of those has a substrate answer that is smaller, self-sizing, and carries
its own confidence:

| Their planned piece | What the substrate offers instead | Why it is better here |
|---|---|---|
| **energy / mood model** driving state choice | `NeurochemState` + `HomeostaticDrive` on battery volts and servo temperature | The interoception is **real** on this robot (§Interoception). Mood stops being a tuned scalar and becomes a body state that genuinely degrades and recovers |
| **novelty-grid exploration memory** for Wander | an **EPM** over ToF + odometry + head pose | §0 rule 1: never hand-roll a clusterer. A grid has a resolution somebody picked; an EPM earns its vocabulary, splits where error persists, and publishes `is_novel` as a measured quantity rather than a grid-cell lookup |
| **16 enumerated states** | attractors of a precision-weighted arbitration (`LateralVoter`, `EFEArbiter`) | States stop being a list to maintain. New behaviour comes from new drives and new sensors, not from a seventeenth `enum` variant and its transition table |
| **contrast-based startle** | a **TLE spike** | Surprise is already the currency the whole stack runs on. A bespoke startle detector is a bespoke confidence scalar, which §0 rule 1 also covers |
| **ToF avoidance with freshness gating** | an EPM over reprojected ToF, precision-weighted | "Freshness gating" *is* precision weighting, `1/(tle+ε)`, arrived at from the other direction. They already found the right idea and would be hand-rolling it |
| **sound reactions with self-audio / self-motion gating** | a `DescendingPredictor`: predict your own noise, react to the residual | Self-gating falls out rather than being maintained. A duck that predicts its own servo whine is not startled by it, and needs no rule saying so |
| **recognition, friendship as a met-count** | an EPM over beacon features | Recognition that generalises, with confidence attached. "Friendship" becomes a baked node with a visit history, which is what a met-count was reaching for |
| **ball play, nap, petting** | drives, with `robot.do` firing their skills | The behaviour is ours, the motion is theirs. Nothing is reimplemented |

The through-line to offer them: **they are one design decision away from hand-rolling six
confidence scalars and a grid.** Prediction error gives all of it from one mechanism, and their
own instincts — freshness gating, self-audio gating, mood driving choice — are already pointing
at it.

### How it ships

`ogma_duckd`: a JSON-RPC client on the existing sockets, in the shape of `padd` and `btd`.
It subscribes to `robot.state` and `tof.stream`, polls `robot.health` at ~1 Hz, and sends the
same intents a gamepad sends. It owns nothing, forks nothing, and needs no change to `robotd`.

Three postures a user can take, and **all three must work**:

1. **Keep RL, add the brain above it.** The default offer, and the most useful one. Locomotion,
   skills and safety unchanged; autonomy is inferred rather than scripted.
2. **Brain to the joints.** Track A, for people who want to research the motor layer. Needs a
   `robot.joints` intent, which is a real change to `robotd` and therefore a separate,
   later conversation with its authors.
3. **No brain.** Do not run the daemon. The duck behaves exactly as it does today.

Posture 3 is the gain-0 guard (§3 rule 2) restated for a community: **a contribution that
cannot be switched off is not a contribution, it is a fork.**

### Educate

The third obligation, and the one most likely to be skipped. Active inference is unfamiliar,
the vocabulary is off-putting, and a contribution nobody can reason about gets merged and then
quietly stops being maintained.

The deliverable is a document written to `REPORTS.md`'s audience — *intelligent, curious,
outside active inference and machine learning* — explaining what an error-minimising brain
offers a duck, in duck terms. Not a paper. The test is whether a Microduck owner who has never
heard of a free energy principle can read it, understand why their duck gets bored, and change
something.

---

## Architecture — `mj_host` is a peer HAL

```text
   ┌──────────────────────── mj_host (one process) ─────────────────────────┐
   │                                                                        │
   │   MuJoCo                    DuckBody                    OgmaInstance   │
   │   mjModel/mjData  ──read──▶ (sensor reduction) ─publish─▶ InProcessBus │
   │        ▲                                                       │       │
   │        │                    action.<joint> ◀──last_value───────┘       │
   │        └──── d->ctrl ◀───── (envelope: clamp · slew)                   │
   │                                                                        │
   │   ControlServer :7400 (JSON-RPC)   DiagPublisher :7401 (ZMQ PUB)       │
   └────────────────┬───────────────────────────┬───────────────────────────┘
                    │                           │
             xaq_inspector                  xaq_voice
        (unchanged — same ports)      (unchanged — diag_lite only)
```

### What `mj_host` owns

- `mjModel` / `mjData` from a vendored `scene_allcollisions.xml`, stepped at the physics rate
  with the brain ticked at 50 Hz (see G3).
- **`DuckBody`** — the analogue of `picrawler_body.gd`, in C++: reads `mjData`, publishes named
  proprio packets, reads the action channels, writes `d->ctrl`. This is the only file that
  knows about ducks.
- `OgmaInstance` + `InProcessBus`, built from a JSON `GraphConfig` exactly as the Godot host
  builds one.
- `ControlServer` on 7400 and `DiagPublisher` on 7401, so `xaq_inspector` and `xaq_voice`
  attach with no change.
- A JSON-per-line diagnostic on stdout, in the shape `seedavg.py` already parses.
- An optional GLFW viewer (MuJoCo ships one), off by default, `--headless` being the norm.

### Track B rehearses in the same host

Track B needs no hardware either. `mj_host` gains an `--intent-mode` in which the ONNX
policies are loaded *inside the host, as part of the body*, and the brain's active states are
intents rather than joint targets:

```text
  brain ──▶ action.twist / action.head ──▶ DuckIntentBody ──▶ obs[61] ──▶ ONNX ──▶ d->ctrl
                                                 │
                          robot.state-equivalent ▼   (joints, targets, gravity, odom,
  brain ◀── reality.proprio.* ◀────────────────────    tof, policy label, limited_by)
```

This is precisely what Pollen's own `scripts/infer_policy.py` does — drive the exported
policies in CPU MuJoCo — with a brain in the loop where the keyboard was. The value is that
**the daemon's contract gets designed and debugged against a simulated duck**, so what
eventually reaches a real one has been exercised rather than merely reviewed.

The host must publish exactly the fields `robot.state` publishes and no more (§Sensing), or
Track B will be developed against a sensory surface the real robot does not have. That is
**G7**.

### ⚠ What `mj_host` must NOT own

- **No reward, no episode return, no fitness.** The `metadata` block in picrawler configs
  carries `reward_shape` / `target_height` / `stability_gain`. Those are picrawler-body scoring
  fields consumed by the Godot body, and none of them come across.
- **No command block as an input, in either mode.** In `--intent-mode` the brain *emits* one;
  it never observes one.
- **No ONNX in Track A.** In joint mode a policy in the process is a policy that gets reached
  for. The one exception is the fall-recovery scaffold (A2), which is gated, named, and
  publishes `events.reset` every time it fires.

---

## Validation gates — checks, not A/Bs

Constraints the port must satisfy independently of any behavioural result. **If one fails, the
port is wrong** — these are not levers and are never seed-averaged.

| Gate | Constraint |
|---|---|
| **G1 — the model loads unmodified** | `mj_loadXML` on the vendored scene succeeds with zero edits to Pollen's XML. Any edit we need is an *overlay* file, recorded as such |
| **G2 — STAND is a stable equilibrium *under active control*** | Hold the standing policy for 3 s from noisy inits: the trunk stays upright. **Check tilt, not height** — `microduck_rl/AGENTS.md` records that a settle test recording only `z` reports fallen states as resting fine. **Restated 2026-08-31 after measurement**: the original wording asked for a *passive* `ctrl` hold, and that fails at 81° tilt even at zero noise. This body has no passive standing equilibrium (§Measured) |
| **G3 — rate fidelity** | The brain ticks at exactly 50 Hz against MuJoCo's timestep, with the substep count stated and constant. The picrawler's `TAU = 0.02` is the same contract; a drifting ratio makes every learning rate meaningless |
| **G4 — joint-order round trip** | `DuckBody` maps `action.<joint_name>` → `d->ctrl[i]` by **name lookup on the model**, never by a transcribed index — and a test asserts the resulting order matches `JOINT_NAMES`. This is the picrawler leg-naming mirror ([`picrawler_sim2real_port.md`](picrawler_sim2real_port.md) §"The leg-naming mirror"), and it is the same trap: silent, behavioural, and only visible as a robot that moves wrong |
| **G5 — the picrawler is byte-identical** | Building `mj_host` changes nothing about `godot_host`. `seedavg.py` on the deployed picrawler config produces the same numbers before and after this branch. Verified once at S0 and once at merge |
| **G6 — no god's-eye channel** | Every topic `DuckBody` publishes is derivable from what the real `robot.state` + `tof.stream` carry. A sensor-legitimacy audit like [`sensor_legitimacy_and_the_feet_y_oracle.md`](sensor_legitimacy_and_the_feet_y_oracle.md), written **before** the first brain runs rather than after |
| **G7 — Track B sees only the wire** | In `--intent-mode` the host publishes exactly the fields `robot.state` / `tof.stream` / `robot.health` carry, and **not one field more** — no velocities, no currents, no ground truth. A test asserts the topic set against the proto. Without this, Track B gets developed against a duck that does not exist |
| **G8 — off is really off** | With `ogma_duckd` not running, a duck is byte-identically the duck it was. No `robotd` change, no config change, no residue. The gain-0 guard, restated for a community (§How it ships) |

---

## Phases

**S** is shared foundation, **A** is Track A, **B** is Track B. S0–S2 serve both and come
first; after that the tracks are independent and can run in either order, or together.

```text
  S0 vendor+build ──▶ S1 body, no brain ──▶ S2 sensory surface
                                                  │
                            ┌─────────────────────┴─────────────────────┐
                            ▼                                           ▼
              A1 MotorEPM at the joints                    B1 intent-mode + first drive
                            │                                           │
              A2 standup-as-reset harness                  B2 ogma_duckd vs the simulator
                            │                                           │
                            └──────────▶ S3 harness parity ◀────────────┘
                                                                        │
                                                             B3 the offer to Pollen
```

### S0 — vendoring and build · *shared* · **NOT STARTED**

- `mj_host/models/microduck/` — the MJCF, `assets/*.stl`, scenes, `joints_properties.xml`,
  `sensors.xml`, copied from `microduck_rl` at a **recorded commit**, unmodified.
  Apache-2.0 both sides; add the attribution row to `THIRD_PARTY_NOTICES.md`.
- `mj_host/CMakeLists.txt` — `add_subdirectory(../cpp_core)`, MuJoCo at a **pinned release**
  via `FetchContent` (prebuilt library; MJCF parsing is version-sensitive, so the version is
  part of the body of record and goes in this doc when chosen).
- A `--load-only` mode that satisfies **G1** and prints the model's joint / actuator / sensor
  tables. That output is the first thing pasted back into this doc.

### S1 — the body, with no brain · *shared* · **NOT STARTED**

`DuckBody` + a run loop, no `OgmaInstance` at all. Steps physics, writes the stdout JSONL,
serves the viewer. Satisfies **G2**, **G3**, **G5**.

**Holds the pose with the *standing policy*, not with a passive `ctrl` hold** — the passive
version falls over (§Measured), so the only honest no-brain baseline on this body is an
actively balanced one. That makes `alpha_stand.onnx` a dependency of S1 rather than of A2,
which is a change from the original phasing and is fine: it is the same named scaffold,
arriving one phase earlier.

This is the gain-0 guard for the whole port: a host that runs the body correctly with no brain
in it is the baseline every later phase is measured against.

### S2 — the sensory surface · *shared* · **NOT STARTED**

Publish, as `reality.proprio.<name>`:

| topic | width | from |
|---|---|---|
| `joints` | 14 | `d->qpos` at the servo joints, minus `HOME_FRAME` |
| `joints_dyn` | 28 | `[q, q̇]` — real `d->qvel`, not the picrawler's synthetic Δq |
| `imu` | 4 | `framequat` + `gyro`, reduced as the picrawler's `imu` is |
| `gravity` | 3 | projected gravity, trunk frame — **the legal replacement for absolute Y** |
| `load` | 14 | `d->actuator_force` → the mA-equivalent channel |
| `foot_contact` | 2 | contact sensors at the foot geoms |
| `tof` | 64 | ray-cast through the `tof` site's 8×8 beam table, then `Reprojector`'s classification ported |

Each one gets a `register_source` line with a plain-language description, exactly as
`picrawler_body.gd:2842` does, so the graph panel shows the full environment↔brain interface.
**G6 is written at the end of this phase, before A1 or B1 starts.**

### A1 — the first brain: MotorEPM on measured proprioception · *Track A* · **NOT STARTED**

The point of the whole exercise: a `JointSensorimotorBridge` + `MotorEPM` graph on a body whose
`x` is *measured*. The picrawler config `motor_epm_pure_hk__inst__stance__c025__lr10.json` is
the shape to start from — the same two module types, re-grouped for this body.

**The head is in the loop from the first run, not an addition later.** It carries 38 % of the
robot's mass on a four-DoF boom with more CoM authority than the trunk-pose shift Pollen
retrained against (§"The head is 38 % of the mass"). Including it is the *smaller* hypothesis;
excluding it asserts that the largest mass in the robot is not part of balance, against
evidence that it is.

**The shape is two `MotorEPM` instances, and this is config-only.** `MotorEPM` validates
`action_topics.size() == n_legs * motor_dim` (`MotorEPM.cpp:853`) — one scalar width for every
group — so a 5/5/4 split cannot live in a single instance:

| instance | `n_legs` | `motor_dim` | joints |
|---|---|---|---|
| `motor_epm_legs` | 2 | 5 | hip_yaw, hip_roll, hip_pitch, knee, ankle × L/R |
| `motor_epm_head` | 1 | 4 | `neck_pitch`, `head_pitch`, `head_yaw`, `head_roll` |

Both shapes already ship: the picrawler runs `n_legs=4, motor_dim=3`, and the Cell runs
`n_legs=1, motor_dim=2`. **No module change is needed** — which matters, because §Coordination
forbids one on this branch.

`JointSensorimotorBridge` carries the same single-scalar constraint (`group_size`), so it pairs
one-to-one: a `group_size=5` bridge feeding the legs and a `group_size=4` bridge feeding the
head. Each group's state vector `x` is `3 × group_size` — the bridge emits `[pos, action, delta]`
per joint (`JointSensorimotorBridge.cpp:123`) — so the legs learn a 15-D loop and the head a
12-D one, against the picrawler's 9-D.

Splitting costs almost nothing behaviourally: each group's `(A, b, C, h)` is *already* seeded
from its own RNG stream (`base_seed ^ leg`) and learned independently, and the inter-group
coupling knobs (`coupling_gain`, `ctrl_symmetry_gain`) are 0 in the deployed picrawler config.
It buys something real — head and legs get their own learning rates and gains, which they
should, since a 280 g boom and a 129 g leg do not share dynamics.

**⚠ The mouth is not simulable.** `robot_walk.xml` declares exactly the 14 policy joints; there
is no mouth hinge in any MJCF variant, though the real robot has the servo and
`robot.mouth` drives it. Anything mouth-shaped is hardware-only and out of scope here.

**⚠ Expect this to be hard in a way the picrawler was not.** A quadruped that flails falls over
and keeps trying; a biped that flails falls over and stops. Homeokinesis destabilises on
purpose, and this body has no static-stability margin to absorb that. A2 exists because of
this, and the honest position is that **bipedal homeokinesis may not bootstrap at all** — which
is a result, gets a ledger entry, and redirects rather than ends the work.

Before any lever: run the authority check the ledger demands
(`corr(actuator, target)` on existing traces — ledger §"THREE LEVERS IN ONE SESSION AIMED AT AN
ACTUATOR WITH NO AUTHORITY").

### A2 — standup-as-reset · *Track A* · **NOT STARTED**

The picrawler harness gets continuous resets by teleporting in sim; on hardware it has none.
Pollen trained `Mjlab-StandUp` and ship `alpha_stand.onnx` / the sitstand net: **a get-up
reflex that works on the real robot.**

Use it as the **reset mechanism, never as a controller**: the brain drives the joints; on fall
detection (`duck-control/src/fall.rs` is the reference implementation) hand to the standup net;
when upright, hand back. It never touches the brain's own error signal, and it is the harness,
not the brain — the same category as the Godot body's auto-reset teleport.

Two things this must carry from the picrawler's own harness bugs (ledger §"Reset artifact"):
publish `events.reset` on every hand-back, and mask learning across the boundary, or **any
TLE or coherence trend across a reset is fake**.

### B1 — intent-mode host, and the first drive · *Track B* · **NOT STARTED**

`--intent-mode`: load `alpha_walking.onnx` + `alpha_stand.onnx` inside `mj_host`, build the
61-D observation from `mjData`, and let the brain's action topics supply the command block.
**G7 gates this phase** — the host's published topic set is checked against the proto, so the
brain is developed against the duck that exists.

The first drive should be the smallest one with a real error to descend, not the most
charming. Candidate: **an EPM over reprojected ToF + odometry, with a homeostatic drive on
battery voltage.** A duck that explores where its depth map is novel and returns toward rest as
its pack sags is already a behaviour their FSM would need four states for, and it comes from
one EPM and one drive.

Ask the authority question first, as always: does the twist the brain emits actually move the
quantity the drive cares about? `corr(commanded twist, Δnovelty)` on a random-walk trace, before
any lever.

### B2 — `ogma_duckd`, against the simulator · *Track B* · **NOT STARTED**

Split the brain out of the host and into a daemon that speaks JSON-RPC over unix sockets,
with `mj_host` serving the `robotd` side of the contract. The daemon does not know whether it
is talking to a simulated duck or a real one, which is the property worth having.

Shape it like `padd`: subscribe, own nothing, time-bound every call, treat a closed socket as a
normal answer. Their `architecture.md` §2.4 is the specification, and following it exactly is
what makes the contribution reviewable by its own authors.

### B3 — the offer · *Track B* · **NOT STARTED**

Three artefacts, in this order:

1. **The gap analysis**, written for Pollen: what `autonomous_behavior.md` plans, what the
   substrate does instead, and what is measurably better rather than merely different
   (§"The gap we fill").
2. **`robot.state` gains velocities and currents** — ✅ **WRITTEN 2026-08-31, not submitted.**
   Branch `state-velocities-currents` in the local clone; **nothing pushed, no fork, no PR** —
   that is the operator's call to make.

   The motivation turned out stronger than the plan assumed. `bus.rs` reads **one contiguous
   twelve-byte block** per tick at register 124 — `present_pwm`, `present_current`,
   `present_velocity`, `present_position` — and unpacks all four into `Sensors`. Only position
   reaches the wire. The other two are already measured and paid for, and are discarded at the
   socket. **Zero extra bus cost, zero new failure mode.**

   Neither is recoverable from outside: subscribers are decimated per connection, so
   differencing `joints` means differencing across five ticks at 10 Hz with any dropped frame
   becoming a spike; and load has no substitute at all.

   | | |
   |---|---|
   | Diff | +199 lines, 0 deletions, 3 files |
   | Wire cost | 722 B → 957 B (+235 B, +33 %); +11.8 KB/s at 50 Hz, +2.4 KB/s at 10 Hz |
   | Back-compat | `skip_serializing_if = "Vec::is_empty"` on the `odom` rule — an older frame parses, and *absent* stays distinguishable from *zero* |
   | Tests | duck-control 60→60, duck-ipc-proto 49→**51**, robotd 96→**97**, 0 failed |
   | Their gates | `cargo fmt --all --check` clean, `clippy -D warnings` clean |

   Validated in sim, which for this change means their own `FakeIo` driving the real
   `control_loop` — the new end-to-end test asserts both blocks reach the stream with values
   distinguishable from each other *and* from the joint angles, so a block landing in the wrong
   field fails in CI rather than on a robot. `FakeIo` gained `set_velocities` /
   `set_currents_ma` mirroring `set_imu`, deliberately **not** derived from what was written:
   positions are echoed back, and deriving these the same way would make the test vacuous.

   **Out of scope, found on the way:** `serde_json` round-trips some `f64` values inexactly
   here (`1.4000000000000001` → `1.4`). It affects every float already on this wire —
   `joints`, `targets`, `gravity` — is ~1e-16 against a sensor resolution of ~1e-3, and is not
   ours to fix in a PR about two new fields. Our test compares approximately, which is the rule
   their own `SafetyState` states when it drops `Eq` because "exact equality on [a measurement]
   is not a comparison anybody should be offered".
3. **`ogma_duckd` itself**, off by default, with the education doc (§Educate).

**Nothing here is offered until it works in sim.** An unproven contribution to someone else's
robot is a cost transferred to them.

### S3 — harness parity · *shared* · **NOT STARTED**

`seedavg.py` currently shells `godot4` with picrawler env vars. Give `mj_host` the same
contract — `OGMA_SEED`, `OGMA_INSPECTOR_PORT`, `OGMA_*_CONFIG`, `OGMA_*_MAX_STEPS`, JSONL on
stdout — and add a duck-side runner beside it rather than branching the picrawler one. The
metric set is the duck's own; `net_z`/`straight`/`turns` are corridor metrics and do not
transfer unexamined. Designing that metric set, and asking §3 rule 4's question of each
candidate ("what degenerate behaviour also scores well here?"), is the work of this phase.

Track B needs its own metrics, and they are harder: "explored the room" and "went home when
tired" have no `net_z`. Expect this phase to be mostly the blind-metric question (§3 rule 4),
not mostly plumbing.

### Hardware · **DEFERRED for both tracks, and out of scope for this branch**

Recorded so the shape stays visible. **The two tracks need very different things, and the
asymmetry is the reason Track B goes first on real hardware:**

| | Track A on hardware | Track B on hardware |
|---|---|---|
| Needs | a **`robot.joints` intent** added to `robotd` | **nothing** — the sockets already carry it |
| Risk to them | a new path into the motor loop, which is their safety-critical surface | a client that can be killed at any moment |
| Conversation required | a design discussion with `robotd`'s authors | a daemon they can ignore |

Both respect the same invariants: `robotd` stays authoritative on safety, the control loop
never blocks on us, and stale intents expire rather than latch.

---

## SPEC — actuator fidelity: the BAM question

**Status: spec only, nothing built.** Written now so the decision is deliberate rather than
discovered at the first transfer test.

Pollen's sim2real recipe rests on BAM — a measured, voltage-controlled model of the XL330 with
Coulomb, Stribeck and load-dependent friction, identified on a test bench
(`xl330_test_bench/`). A plain-MuJoCo host gets the XML `position` actuator instead:
`kp=0.55, kv=0, forcerange=±0.96, ctrlrange=±10`, with `damping/frictionloss/armature` on the
joint.

Three options, in increasing cost:

1. **Ship the XML actuator.** Free, and adequate for **posture, balance and head-use** work.
   **Measured 2026-08-31 to be NOT adequate for locomotion**: the trained gait is attenuated to
   half amplitude and the duck steps in place (§Measured). The original wording here claimed it
   was adequate through S3, and that was falsified within a day of writing it. **Start here for
   anything postural.**
2. **Port BAM's `compute()` to C++.** It is a torque model over `(q, q̇, target, load)`; the
   Python is thin, and Rhoban's `bam` carries the identified parameters. Becomes necessary the
   moment a *hardware* transfer is attempted, and not before.
3. **Drive MuJoCo from Python and keep BAM.** Rejected: it puts the brain behind an FFI
   boundary at 50 Hz and drags mjlab's whole training stack into the runtime.

**The gate that decides it:** any claim involving **locomotion** needs option 2 (or a
stiffness correction validated against it). Any claim about posture, balance or whether the
substrate closes a loop at all needs only option 1. Record which
one a result was measured under, the same way the picrawler records ghost-vs-solid chassis.

---

## Open decisions

1. **MuJoCo version pin.** Must be chosen and recorded here; `mj_loadXML` behaviour on these
   files is version-sensitive, and `microduck_rl` deliberately does not override the `mujoco`
   version mjlab pins.
2. **Whether `robot_walk` or `robot_allcollisions` is the default.** Leaning `allcollisions`,
   per the ghost-chassis lesson: a belly that cannot touch the ground is a different body, and
   every claim measured on it is about that body (ledger §"seedavg.py does not set
   `OGMA_PICRAWLER_CHASSIS_COLLIDE`").
3. **Which track leads.** They share S0–S2, then diverge. Track A is the deeper result and the
   riskier one (bipedal homeokinesis may not bootstrap); Track B is the more certain one and
   the one with an outside audience. Not decided, and it does not need to be until S2 lands.
4. **Whether to talk to Pollen before B3 or at it.** An early note costs nothing and might
   save building the wrong thing; it also commits us in public to work that has not been
   measured. Leaning toward the narrow `robot.state` PR first (B3 step 2) as the introduction,
   since it stands on its own merits whatever happens to the rest.

## Coordination

### With the picrawler branch

**No shared files.** This branch adds `mj_host/` and this doc; the picrawler hardware work
lives in `pi_host/`, `godot_host/project/scripts/picrawler_body.gd`, and its own port doc.
The one shared surface is `cpp_core/`, and the rule is that **this branch does not modify a
module.** If A1 wants a MotorEPM change, it stops and gets its own lever discussion under
the §3 protocol, on its own branch, gain-0-guarded, rather than editing the benchmark every
picrawler A/B depends on (the reason `MotorEPMv2` exists as a copy).

**G5 is the check that keeps this true** and is run at both ends of the branch.

### With Pollen — upstream etiquette

Their repo is not ours, and the difference shows up in small decisions:

- **Vendored files stay unmodified.** The MJCF and its meshes are copied at a recorded commit.
  Anything we need changed is an overlay beside it, so a re-vendor is a copy rather than a merge
  (**G1**).
- **A PR does one thing.** "Expose velocities and currents" is a PR. "Expose velocities and
  currents, and also here is a brain" is a debate.
- **Their conventions win inside their repo**, including the ones we would not have picked.
- **No claim travels upstream that has not been measured here first**, at the power §3 demands.
  Scaling a claim to its evidence matters more, not less, when the reader is a stranger.
- **We do not describe their design as a mistake.** A 16-state machine is a reasonable thing to
  build, it is what most robots ship, and the substrate has to earn the comparison on results.
