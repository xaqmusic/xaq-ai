# mj_host — the Microduck in simulation

This directory runs a [Pollen Robotics Microduck](https://github.com/pollen-robotics/microduck)
in MuJoCo with an autonomous brain on top of it. It is written for someone who has a Microduck,
or works on one at Pollen, and wants to:

1. watch the brain play in a simulated room on their own machine, and
2. help move it toward the real robot.

Four parts:

- **The body.** Pollen's model files, unmodified.
- **The skills.** Pollen's own trained policies: walk, stand, kick, forward roll, sit/stand and
  ground pick. They run through ONNX Runtime at the action scales and filters `robotd` uses.
- **The brain.** It sits above the policies and decides what the duck does: where to look,
  where to walk, what to walk up to, and when to kick, peck, chase or give up.
  - It never writes a leg joint. It sends what a gamepad or the phone app would send: a walking
    velocity, a head command, a skill request.
  - It is not trained with a reward. It predicts what it will sense next and acts to make those
    predictions come true, so it learns its own body and room as it goes.
- **The tools** for watching it and measuring it.

The design behind it is proposed to Pollen in
[pollen-robotics/microduck#381](https://github.com/pollen-robotics/microduck/pull/381). There are
two narrated videos, [the proposal and a demo](https://youtu.be/W2jMf6EhW0k) and
[the interpretability and debugging tools](https://youtu.be/F9kEaafD-vo), and a longer
walkthrough, [Inside the MicroDuck Brain](https://claude.ai/artifact/Dirv1ZWg62pjQVM3SCvkzh).

**Simulation only.** Nothing in this repository talks to a real duck yet.
[Toward the real robot](#toward-the-real-robot) lists what exists and what is missing.

---

## What you will see

The duck is alone for ten minutes in a 4 m playroom with balls, blocks, furniture and a toy
train on a loop. It:

- sees through the 8×8 depth sensor in its head and builds its own map of what is around it;
- tells small things from walls and furniture, walks to the small ones, and kicks or pecks them;
- chases the train when it notices it moving;
- looks around at parts of the room it has not seen;
- when it gets stuck, sits down, looks around, and walks out.

Runs are scored by where the ten minutes go:

- **interesting:** at a thing, walking to a thing, chasing, using a skill;
- **boring:** staring at a wall, pinned against furniture, walking toward nothing.

The current configuration is **★ MV**. The star marks a configuration that was promoted after
being measured and watched. Over 24 ten-minute runs it spends about 21 % of its time bored, and
it falls about 2.3 times a run. Each fall is caught by Pollen's standing policy, which stands the
duck back up and hands it back to the brain.

Falls are the number that most needs to come down before hardware. The full measurements are in
[the design record, §17.118](../docs/plans-and-designs/microduck/rung2_regime_design.md).

---

## Getting it running

### Requirements

**Linux on x86_64.** The build fetches MuJoCo 3.12.0 and ONNX Runtime 1.29.0 as prebuilt
linux-x86_64 archives, pinned by version and SHA-256. macOS and ARM desktops are not supported
by this build. The brain alone does build on ARM: see [Compute](#compute-on-the-robots-board).

On Debian or Ubuntu:

```sh
sudo apt install cmake g++ git pkg-config libzmq3-dev python3-venv python3-tk
```

- `libzmq3-dev` is for the brain's runtime (`cpp_core/`).
- `python3-tk` is for the launcher window.

Everything else is downloaded on first build:

- MuJoCo (31 MB) and ONNX Runtime (11 MB);
- Eigen, nlohmann/json and GoogleTest;
- Pollen's policy networks, from Pollen's repository and Hugging Face at pinned commits, each
  checked by SHA-256 ([`scripts/fetch_scaffolds.sh`](scripts/fetch_scaffolds.sh)).

### Build

From the repository root:

```sh
./mj_host/run.sh build
./mj_host/run.sh gates        # checks the model loads and stands; exits non-zero on any failure
```

### Watch it

```sh
./mj_host/run.sh launcher
```

1. Pick **★ MV** (the first preset).
2. Press **Launch**.

A MuJoCo window opens and the duck starts in the playroom. Seed 7 is a good first watch: the
train comes past at about 2.5 minutes. The first launch creates the viewer's Python environment
in `tools/duck_viewer/.venv` (MuJoCo's Python bindings and imageio).

**Viewer controls.** Drag to orbit, scroll to zoom, right-drag to pan, space to pause. Our keys:

| key | does |
|---|---|
| `V` | shows the depth sensor's 64 beams |
| `C` | the head camera at the brain's resolution (shown for you; the brain does not use it) |
| `O` | recentres the camera on the duck |
| `W` | fades walls and furniture that hide the duck |

[`tools/duck_viewer`](../tools/duck_viewer/README.md) has the rest.

**The command behind the window.** The launcher always shows the exact host command line. Every
run is deterministic, so pasting that line into a shell repeats the run:

```sh
tools/duck_launcher/launcher.py --print     # the command for the current selection
```

**Inside the brain.** While a run is live, press **Inspector** in the launcher, or run
`tools/run_inspector.sh`. It shows each of the brain's modules live. The three brains (below)
serve it on TCP ports 7400, 7402 and 7404.

### Run many seeds and score them

One run is an anecdote. Run a preset over many seeds, headless, in parallel:

```sh
python3 mj_host/tools/preset_sweep.py "★ MV" mytry --seeds 8 --jobs 4
```

Logs go to `mj_host/log/ten/mytry/`, about 70 MB per ten-minute run. Then see where the minutes
went:

```sh
cd mj_host
CLOUD_OBJECTS_MANIFEST=models/microduck/scene_playroom_train.manifest.json \
    python3 tools/ten_minutes.py "log/ten/mytry/*.jsonl"
```

- The manifest tells the scorer which scene the runs used. Without it, the scorer assumes the
  room without the train.
- The scorer reads the simulator's ground truth to label what the duck was doing. That truth
  never reaches the brain.

---

## How the pieces fit

Everything runs in one process, `ogma_mjhost`, at 50 ticks a second:

```text
 MuJoCo body ──► sensors (IMU, joint positions, contact odometry, 8×8 depth sensor)
                    │
                    ▼
     ┌──────────────────────────────────────────────┐
     │  intent brain  what to do: walk where, chase, │──► walking velocity, skill request
     │                stop, look                     │
     │  head brain    where to point the head        │──► head joint targets
     │  stop brain    loaded but idle in ★ MV        │
     └──────────────────────────────────────────────┘
                    │
                    ▼
 Pollen's policies: walker · kicks · roll · sit/stand · ground pick · stand (fall rescue)
                    │
                    ▼
               MuJoCo body
```

**Sensors.** The depth sensor is cast every fourth tick (12.5 Hz) against the room. The cast is
a port of Pollen's own reprojection: empty, too close, floor, or hit.

**Skills.** Skill windows and their priority order follow `robotd`'s.

**Falls.** The recovery harness picks the duck up when it falls (below). Nothing learns while
the harness drives.

**The code boundary.** It is the three adapters' `tick()` calls in `cmd_level2` in
[`src/main.cpp`](src/main.cpp), and `--record-brains` records every call across it to a tape.
That boundary is where a robot client would attach.

How Pollen's runtime works, and why the brain sits where it does:
[`docs/reports/microduck_pollen_architecture_report.md`](../docs/reports/microduck_pollen_architecture_report.md).

---

## Toward the real robot

### What exists

- **Merged upstream.** [pollen-robotics/microduck#260](https://github.com/pollen-robotics/microduck/pull/260)
  puts the servos' velocity and load on `robot.state`.
- **Proposed upstream.** The brain's design: [#381](https://github.com/pollen-robotics/microduck/pull/381)
  (open).
- **Ported Pollen code.** The host reproduces `robotd`'s skill windows, policy filters and fall
  detector, and Pollen's contact odometry and depth-sensor reprojection, so the brain already
  works against the same interfaces it would meet on the robot.
- **A compute benchmark that runs on ARM.** The brains rebuilt from a tape, with no MuJoCo or
  ONNX, measured on a Raspberry Pi 5 (below).

### What does not exist yet

**There is no robot client.** No code here connects to `robotd`. The design is settled
([`intent_boundary_design.md`](../docs/plans-and-designs/microduck/intent_boundary_design.md)):

- a small daemon, shaped like Pollen's `padd` and `btd` clients, written in Rust on Pollen's
  `duck-ipc-proto` crate;
- it wraps the brain's C++ core through a small C interface;
- it subscribes to `robot.state` and the depth-sensor stream and sends `move` / `head` /
  `look` / `do` / `stop`;
- it is off by default, so a robot that doesn't run it loses nothing.

None of it is written.

### The gaps, each with what would close it

| gap | in simulation today | on the robot today | what would close it |
|---|---|---|---|
| **Head control** | the head brain writes the four head joints directly | `robot.head` goes through the walking policy (120–160 ms late); there is no joint-level head intent | a joint-level head intent in `robotd`, or re-measuring the head brain through `robot.head` / `robot.look` (open question 1 in #381) |
| **Accelerometer** | fills three of the intent brain's inputs | the IMU board measures it, but `robotd` reads only the gyro and orientation bytes, so it is not on `robot.state` | read and publish it (`duck-control/src/imu.rs`), or measure the brain without those inputs |
| **Head gyro** | an extra sensor in our scene, placed as if on the sensor board | not present | hardware, or drop the input and measure the cost |
| **Heading** | MuJoCo's exact yaw, no sensor noise anywhere | the IMU, which drifts | add IMU noise and drift to the host and re-measure before trusting any number on hardware |
| **Depth sensor** | 64 exact rays, 4 m range, 12.5 Hz, no noise, no per-zone status | VL53L8CX, 15 Hz, raw ranges with per-zone status, shorter useful range | model noise, status and range; `--tof-real SPREAD LAG` models the sub-frame timing but is off in ★ MV |
| **Camera** | rendered for the viewer only; the brain does not use it | 1080p over WebRTC; no frame reaches a robot client | a frame or feature stream for clients (section 6 of #381) |
| **Falls** | about 2.3 per ten-minute run, each caught by the standing policy | `robotd`'s safety layer owns falls | fewer falls in simulation first, then agree with Pollen how a client should handle a fall |
| **Actuators** | Pollen's MJCF actuators | real servos | a port of the BAM actuator model (`--bam`) was built and changed little, but ★ MV has not been re-measured on it |
| **Policy versions** | networks pinned to the commits in `scripts/fetch_scaffolds.sh` | whatever the current release ships | re-pin to the release and re-run the gates and a sweep |
| **Beak** | no model carries the beak hinge, so a peck grasps nothing | the beak works | a model with the hinge |
| **Host-side helpers** | timed and on-arrival stops, the gaze sweep while stopped, stuck escape, the fall rescue, the detour, all C++ in `main.cpp` | — | move them into the client with the brain (listed in section 4 of #381) |

The brain reads no simulator ground truth. The train's true position, the duck's world pose and
contact flags all go to the log for scoring, never to a brain. That was audited input by input on
an earlier configuration ([`la1_fact_sheet.md`](../docs/plans-and-designs/microduck/la1_fact_sheet.md)).
The inputs added since then have not been re-audited one by one.

### Compute on the robot's board

[`tools/arm_bench`](tools/arm_bench/README.md) records a ten-minute run's brain inputs to a tape,
rebuilds the three brains on a board from that tape alone, checks that the outputs match, and
times every tick:

```sh
mj_host/tools/arm_bench/arm_bench.py all <ssh-host>     # record → stage → build → run → report
```

Measured on an earlier configuration (★ F5, one ten-minute run):

| | intent brain per tick (mean / p99) | all three brains | memory |
|---|---|---|---|
| desktop x86 | 0.2 ms / 1.6 ms | 1 % of a core | 24 MB resident |
| Raspberry Pi 5, measured | 0.5 ms / 4.1 ms | 2.9 % of a core | 12.5 MB peak |
| Radxa Zero 3W, projected | 2–4.5 ms / 15–32 ms | 11–26 % of one A55 core | about 5 MB |

- The map of the room is 83 % of the intent brain's cost.
- On the Radxa projection its occasional spikes would overrun a 20 ms tick, so the brain belongs
  on its own thread, not `robotd`'s control loop.
- ★ MV has more modules than ★ F5 and has not been re-measured.
- **No Radxa has been measured.** If you have one, `arm_bench.py all <your-radxa>` turns the
  projection into a measurement. That would be one of the most useful things anyone could
  contribute right now.

---

## Reference

### Files

| piece | what it is |
|---|---|
| `models/microduck/` | the body, [vendored unmodified](models/microduck/README.md) from `microduck_rl` at `d424a0c`, plus our playroom scenes |
| `models/microduck/scaffolds/` | Pollen's policies (fetched, not committed) and [why they are called scaffolds](models/microduck/scaffolds/README.md) |
| `configs/` | brain configurations (JSON module graphs); ★ MV is `a1v2_st3_vouch.json` |
| `checkpoints/` | saved brains the presets start from |
| `src/main.cpp` | the host: every mode, the skills, the stops, the logging |
| `src/DuckBody.*` | the body as the host sees it: names resolved once, physics stepped, egocentric state out |
| `src/Observation.*` | the policies' 61-number input, with the layout table |
| `src/Policy.*` | one ONNX session, validated at load rather than at inference |
| `src/IntentAdapter.*`, `src/HeadAdapter.*`, `src/OgmaBrainAdapter.*` | the three brains' boundaries: what each reads and what it sends |
| `src/Tof.*` | the depth sensor: 64 rays and Pollen's reprojection |
| `src/Odometry.*` | a port of Pollen's contact odometry |
| `src/Recovery.*` | the fall detector and the rescue hand-off |
| `src/BrainTape.*`, `tools/brain_replay.cpp` | record every call into the brains; replay them anywhere |
| `src/InspectorSurface.*` | the inspector's sockets |
| `tools/` | sweeps, scorers and readouts; `preset_sweep.py` and `ten_minutes.py` are the two to start with |
| `CMakeLists.txt` | MuJoCo 3.12.0 and ONNX Runtime 1.29.0, each pinned by version **and** SHA-256 |

### `run.sh`

```sh
./mj_host/run.sh build                        # build (fetches MuJoCo / ONNX on first run)
./mj_host/run.sh gates                        # every gate, non-zero exit on any failure
./mj_host/run.sh launcher                     # presets, configs, seeds, shoves; watch or headless
./mj_host/run.sh level2 [args]                # watch the brain above Pollen's policies
./mj_host/run.sh watch --secs 30              # watch the standing policy hold the body
./mj_host/run.sh stub --secs 60               # watch the recovery harness catch a brain that falls
./mj_host/run.sh record /tmp/duck.mp4 --secs 8
./mj_host/run.sh hold --secs 60               # headless, JSONL to log/
```

`watch`, `level2` and `record` take the host's own arguments (`--secs`, `--noise`, `--seed`, a
scene path). The binary underneath:

```sh
./mj_host/build/ogma_mjhost --load-only       # G1 / G3 / G4, on models/microduck/scene.xml
./mj_host/build/ogma_mjhost --gate-g2         # the settle sweep: 4 noise levels × 3 seeds
./mj_host/build/ogma_mjhost --hold --secs 5   # the standing policy, JSONL on stdout
./mj_host/build/ogma_mjhost --help            # every mode and flag
```

**The viewer never simulates.** The host writes the full generalized position every tick and
[`tools/duck_viewer`](../tools/duck_viewer) draws it. What is on screen is the run, not a second
copy of the dynamics that could quietly disagree with it, and a saved run replays identically.

Runs are large: about 70 MB of JSONL per ten-minute brain run. Logs stay under `mj_host/log/`,
which is gitignored, rather than `/tmp`.

### The inspector

Each brain serves the inspector's two sockets, control and diagnostics:

| brain | ports |
|---|---|
| intent | 7400 / 7401 |
| head | 7402 / 7403 |
| stop | 7404 / 7405 |

- `OGMA_INSPECTOR_PORT` moves the base port, and 0 disables it. Give each concurrent host its
  own base port.
- A port already taken logs one line, and the run proceeds without an inspector.
- `tools/run_inspector.sh` attaches, with a selector for the brain.
- `tools/xaq_voice` can subscribe to the diagnostics stream and turn it into sound.

### The gates

`--load-only` exits non-zero when a gate fails, so it belongs in CI rather than in a person's
memory.

| Gate | Question | Answer today |
|---|---|---|
| **G1** | does Pollen's MJCF load with zero local edits? | PASS: `scene.xml` and `scene_walk.xml` both |
| **G3** | is the 50 Hz brain tick a whole number of physics steps? | PASS: 0.002 s timestep, **10 substeps** |
| **G4** | does `ctrl[i]` drive the joint this project thinks it does? | PASS: identity, in `JOINT_NAMES`-minus-mouth order |
| **G2** | is the standing pose stable *under active control*? | PASS: tilt 0.46–0.48° across 4 noise levels × 3 seeds |

**G4 earns its place by being able to fail.** Swapping the `left_knee` and `left_ankle`
transmissions in a scratch copy produces:

```
   3  left_knee    -> left_ankle    <-- expected left_knee
   4  left_ankle   -> left_knee     <-- expected left_ankle
  [G4 FAIL — resolve by name, and find out why the order moved]
```

A wrong mapping produces a robot that moves plausibly and incorrectly, and no aggregate metric
catches it. The host resolves joints by name for the same reason.

**G2 is judged on tilt, never height.** A settle test that records only height reports a fallen
robot as resting comfortably, which `microduck_rl`'s own `AGENTS.md` names as a mistake that
cost it days.

G2 is also why there is a standing policy in the loop at all. Holding the `STAND` keyframe's own
`ctrl` with nothing in the loop topples the robot to 81° at every noise level, including zero.
**This body has no passive standing equilibrium**, so the only honest no-brain baseline is an
actively balanced one.

The settling transient is worth watching once: the head is the biggest mover in it. Over the
first second `neck_pitch` swings 146 mrad, `head_pitch` 99 and `head_yaw` 68, each larger than
any leg joint (the largest is 63). A robot carrying 38 % of its mass on a four-joint head should
be expected to balance with it.

### The recovery harness

The brain drives, and when the body ends up on the floor, Pollen's standing policy picks it up
and hands it back. That is a harness, not a controller. It gives a learning brain continuous
operation on a body that cannot stand passively.

| | |
|---|---|
| **Trigger** | projected gravity past **−0.5** (about 60° over) held **200 ms**: the numbers from Pollen's `duck-control` `SafetyConfig`, so the same criterion runs in sim and on the robot |
| **Hand-back** | gravity below −0.95 held 0.4 s, so a tumble passing through upright does not count |
| **Measured** | fires at ~82° tilt, hands back at ~5°, recovery ~0.7 s |

Three choices in it, each for a reason:

- **The trigger is the late detector, on purpose.** `robotd` also ships a `FallPredictor` that
  fires at ~26° so the gains can drop before impact. Using it here would rescue the brain before
  it experienced the fall, and the fall is what the brain needs to learn from.
- **Nothing learns while the harness drives.** A brain still updating on actions it did not
  issue would be learning the standing policy, and would look like it was learning to get up.
- **Both edges are logged** as `reset:handoff` / `reset:handback` in the JSONL, so no trend
  silently spans a fall.

`./mj_host/run.sh stub --secs 60 --stub-amp 0` holds exactly the home pose with no controller
at all. The robot still falls about 44 times in 60 s, and the harness keeps it running
throughout.

### Shoving it

`--push N` shoves the trunk with `N` newtons on a rotating heading every `--push-every` seconds:
the cheapest perturb-and-recover test, and one an eye can judge.

```sh
./mj_host/run.sh watch --secs 20 --push 10 --push-every 4
```

Pollen's standing policy, measured over 24 s with `--push-every 4`:

| shove | peak tilt | outcome |
|---|---|---|
| 2 N | 7.6° | leans and catches itself, 5/5 |
| 5 N | 6.7° | staggers, 5/5 |
| 10 N | 101° | knocked over, 5/5 back up |
| 25 N | 166° | knocked flat, 5/5 back up |
| 60 N | 176° | 4/5; this is where it stays down |

A recovery takes about 1.6–2.4 s. A shove that lands before the last recovery has finished is
reported as **inconclusive**, not as a failure. So is one cut off by the end of the run.

- Exit codes: `0` upright, `1` a real fall, `3` not watched long enough.
- Runs keep a clean tail with no shoves, so a conclusive verdict is the default.

The same schedule can shove a brain that learned to stand on its own
([the standing report](../docs/reports/microduck_standing_report.md)):

```sh
mj_host/build/ogma_mjhost --brain --graph mj_host/configs/a1v2_r12c_whole.json \
    --secs 420 --seed 2 --load-brain mj_host/checkpoints/duck_controlled_brain_s2.json \
    --push 2 --push-every 60 --push-from 30 > run.jsonl
python3 mj_host/tools/push_report.py run.jsonl
```

Each shove is reported as **caught by the brain** or **rescued by the standing policy**. Both
end upright, and only the first says anything about the brain.

### Cross-checked against an independent implementation

The same episode run through this host (MuJoCo C API) and through a Python probe (MuJoCo
bindings), same scene, same policy:

| 5 s hold | `tilt_end` | `z_end` | drift |
|---|---|---|---|
| `ogma_mjhost --hold --secs 5` | 0.48° | 0.1163 m | 0.003 m |
| Python probe | 0.48° | 0.1163 m | 0.003 m |

That is the check worth having on `Observation.cpp`, where a wrong offset produces a plausible
robot rather than an error.

### Notes for maintainers

- **The version pin is part of the body of record.** MJCF parsing is version-sensitive and every
  number in the model README was measured under MuJoCo 3.12.0. Re-pin deliberately, and re-run
  the gates when you do.
- **MuJoCo 3.12 widened the model dimensions to `mjtSize` (`int64_t`).** `nq`, `nu`, `nbody` and
  friends are no longer `int`. Iterate with `mjtSize` and print through `long long`.
- **The host reports library and header version** and flags a skew.
- **The brain modules run in config order.** A module that consumes another's output must come
  after it in the JSON.
- **The working record** is [the port plan](../docs/plans-and-designs/microduck/port_plan.md)
  (start at "Resume here") and the
  [design record](../docs/plans-and-designs/microduck/rung2_regime_design.md). The
  [launcher's README](../tools/duck_launcher/README.md) covers presets and how to add one.
