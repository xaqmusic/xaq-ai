# mj_host — the Microduck MuJoCo host

The third host, beside [`godot_host/`](../godot_host) (the picrawler, in Godot) and
[`pi_host/`](../pi_host) (the picrawler, on real hardware). `OgmaInstance.hpp` already names
*"the host (Godot Host, HAL Host, Debug Host)"* as the owner of the Bus, so this is the
anticipated shape rather than a new one: it holds an `OgmaInstance` in plain C++ and never
touches `OgmaBrain`'s Godot `Variant` marshalling.

Plan and rationale: [`docs/plans-and-designs/microduck_port_plan.md`](../docs/plans-and-designs/microduck_port_plan.md).
**Phases S0 and S1 are done. There is no brain in here yet** — that is S2 and S3.

| piece | what it is |
|---|---|
| `models/microduck/` | the body, [vendored unmodified](models/microduck/README.md) from `microduck_rl` at `d424a0c` |
| `models/microduck/scaffolds/` | [somebody else's controller](models/microduck/scaffolds/README.md), named as a prop, holding the body up |
| `src/DuckBody.*` | the body as the host sees it: names resolved once, physics stepped, egocentric state out |
| `src/Observation.*` | the 61-D vector, with the layout table — the highest-risk file here |
| `src/Policy.*` | one ONNX session, validated at load rather than at inference |
| `src/main.cpp` | the three modes below |
| `CMakeLists.txt` | MuJoCo 3.12.0 and ONNX Runtime 1.29.0, each pinned by version **and** SHA-256; `ogma_core` behind an off-by-default option |

## Build and run

[`run.sh`](run.sh) is the front door:

```sh
./mj_host/run.sh build
./mj_host/run.sh gates                        # every gate, non-zero exit on any failure
./mj_host/run.sh watch --secs 30              # WATCH IT — live window, run saved to log/
./mj_host/run.sh record /tmp/duck.mp4 --secs 8
./mj_host/run.sh hold --secs 60               # headless, JSONL to log/
./mj_host/run.sh stub --secs 60               # the recovery harness (below)
```

`watch` and `record` take the host's own arguments, so `--secs`, `--noise`, `--seed` and a
scene path all pass straight through:

```sh
./mj_host/run.sh watch --secs 10 --noise 0.05 --seed 2
./mj_host/run.sh watch --secs 20 --push 10 --push-every 3     # shove it and watch it recover
```

**Give it something to recover from.** A held pose settles in about a second and then sits
almost still — a millirad of residual correction, which is correct and dull. `--push N` shoves
the trunk with `N` newtons on a rotating heading every `--push-every` seconds, which is the
cheapest form of the perturb-and-recover test and the thing an eye can actually judge.

Measured over 24 s with `--push-every 4`, which leaves every shove room to finish:

| shove | peak tilt | outcome |
|---|---|---|
| 2 N | 7.6° | leans and catches itself, 5/5 |
| 5 N | 6.7° | staggers, 5/5 — the most watchable |
| 10 N | 101° | knocked over, 5/5 back up |
| 25 N | 166° | knocked flat, 5/5 back up |
| 60 N | 176° | 4/5 — this is where it stays down |

**Space the shoves.** A recovery takes about 1.6–2.4 s, so `--push-every` under about 4 s
shoves the robot again while it is still getting up. That is a legitimate thing to test, and it
is reported as INCONCLUSIVE per shove rather than as a failure — but it is not a strength limit,
and reading it as one understates the recovery envelope badly.

### The verdict is three-valued, on purpose

`tilt_end` used to be read at the last tick, so it could not tell UPRIGHT from STILL RECOVERING
and a run that stopped mid-getup reported a fall that never happened. What replaced it:

- **`settled`** — worst tilt across the final window, not one sample's opinion
- **`upright %`** — how much of the run was spent up, which no instant knows
- **per-shove recovery**, each marked recovered / not recovered / **inconclusive**

**Inconclusive is the part that was missing.** A shove that cannot be judged — because the run
ended, or because the next shove landed first — is not a failure, and the two causes are named
separately because one means "run longer" and the other means "shove less often". Exit codes
follow: `0` upright, `1` a real fall, `3` not watched long enough.

Runs reserve a clean tail automatically: nothing is shoved inside the final recovery window, so
a conclusive run is what you get by default and an inconclusive one takes effort.

### The launcher

`./mj_host/run.sh launcher` opens the experiment launcher
([`tools/duck_launcher`](../tools/duck_launcher/README.md)): presets for the experiments
so far, every config, seed or a battery of seeds, from scratch or from a saved brain, the
shove schedule, watch live or headless — and the exact host command on screen to paste
into a shell.

### Shoving the brain

The same schedule runs against a learned stance:

```sh
mj_host/build/ogma_mjhost --brain --graph mj_host/configs/a1v2_r12c_whole.json \
    --secs 420 --seed 2 --load-brain mj_host/checkpoints/duck_controlled_brain_s2.json \
    --push 2 --push-every 60 --push-from 30 > run.jsonl
python3 mj_host/tools/push_report.py run.jsonl          # per-run + per-shove table
python3 mj_host/tools/push_report.py --timeline run.jsonl
```

Two things differ from `--hold`. A shove lands only on a brain-driven tick — one that would
land mid-rescue is skipped and counted, not deferred, so the schedule stays readable — and
each shove is reported as **caught by the brain** or **rescued by the scaffold**, because both
end upright and only the first says anything about the brain. `--push-from S` holds the
shoves off until `S` seconds in, for a brain that has to consolidate first. The JSONL carries
the active force in `push` (literal zeros when nothing is pushing) and each MotorEPM's earned
consolidation in `cons`, which is the trace the perturb-and-recover test is judged on: it must
collapse on a real fall and re-earn itself afterwards.

The binary underneath, if you want it directly:

```sh
./mj_host/build/ogma_mjhost --load-only     # G1 / G3 / G4, on models/microduck/scene.xml
./mj_host/build/ogma_mjhost --gate-g2       # the settle sweep: 4 noise levels x 3 seeds
./mj_host/build/ogma_mjhost --hold --secs 5 # run the scaffold, JSONL on stdout
```

**The viewer never simulates.** The host writes the full generalized position every tick and
[`tools/duck_viewer`](../tools/duck_viewer) draws it — so what is on screen is the run, not a
second copy of the dynamics that could quietly disagree with it. Every `watch` and `hold` keeps
its run under `log/`, and a saved run replays identically forever.

MuJoCo (31 MB) and ONNX Runtime (11 MB) download on the first configure and are cached
thereafter. Nothing else is needed: no Python, no mjlab, no CUDA, and `ogma_core` is not built
unless you ask for it with `-DMJ_HOST_WITH_BRAIN=ON` (S2 onward).

## What `--load-only` is for

It is gate G1 with its working shown, and it exits non-zero when a gate fails, so it belongs
in CI rather than in a person's memory.

| Gate | Question | Answer today |
|---|---|---|
| **G1** | does Pollen's MJCF load with zero local edits? | PASS — `scene.xml` and `scene_walk.xml` both |
| **G3** | is the 50 Hz brain tick a whole number of physics steps? | PASS — 0.002 s timestep, **10 substeps** |
| **G4** | does `ctrl[i]` drive the joint this project thinks it does? | PASS — identity, in `JOINT_NAMES`-minus-mouth order |
| **G2** | is `STAND` a stable equilibrium *under active control*? | PASS — tilt 0.46–0.48° across 4 noise levels × 3 seeds |

The settling transient is worth watching once: **the head is the biggest mover in it.**
Over the first second `neck_pitch` swings 146 mrad, `head_pitch` 99 and `head_yaw` 68 — every
one larger than any leg joint (the largest is 63). When this body settles, the head does most
of the work, which is what a robot carrying 38 % of its mass on a four-DoF boom should be
expected to do.

**G4 earns its place by being able to fail.** Swapping the `left_knee` and `left_ankle`
transmissions in a scratch copy produces:

```
   3  left_knee    -> left_ankle    <-- expected left_knee
   4  left_ankle   -> left_knee     <-- expected left_ankle
  [G4 FAIL — resolve by name, and find out why the order moved]
```

That is the picrawler's leg-naming mirror in a different body: a wrong mapping produces a
robot that moves plausibly and incorrectly, and no aggregate metric catches it. The host
resolves joints by name for the same reason — an identity that holds today is not a contract.

## G2, and why it is judged on tilt

`--gate-g2` holds the standing scaffold for three seconds from twelve perturbed starts and
passes on **tilt**, never height. A settle test that records only `z` reports a fallen robot as
resting comfortably, which `microduck_rl`'s own `AGENTS.md` names as a mistake that cost it
days.

The gate is also the reason there is a scaffold at all. The plan originally had S1 hold the
`STAND` keyframe's own `ctrl` with nothing in the loop; measured, that topples the robot to 81°
at every noise level including zero. **This body has no passive standing equilibrium**, so the
only honest no-brain baseline is an actively balanced one.

## The recovery harness (phase A2, built before A1)

The arrangement the brain will run under: **the brain drives the joints, and when the body
ends up on the floor the scaffold picks it up and hands it back.** That is the harness, not the
controller — the same category as the Godot body's auto-reset teleport — and it is what gives a
learning brain continuous operation on a body with no passive standing equilibrium.

It is testable now because it needs a brain that *fails*, which is far easier to write than one
that works. `--stub` is a random walk around the home pose, and `run.sh stub` watches it.

```sh
./mj_host/run.sh stub --secs 60              # watch it fall and be caught, ~40 times
./mj_host/build/ogma_mjhost --stub --secs 60 --stub-amp 0
```

| | |
|---|---|
| **Trigger** | projected gravity past **−0.5** (about 60° over) held **200 ms** — `duck-control`'s own `SafetyConfig` numbers, so the same criterion runs in sim and on the robot |
| **Hand-back** | gravity below −0.95 held 0.4 s, so a tumble passing through upright does not count |
| **Measured** | fires at ~82° tilt, hands back at ~5°, recovery ~0.7 s; 42–44 rescues per 60 s, stable across seeds |

Three things it gets right, each for a reason that cost somebody time before:

- **The trigger is the LATE detector, deliberately.** `robotd` also ships a `FallPredictor`
  that fires at ~26° so the gains can drop before impact. Using that here would rescue the
  brain before it experienced the fall, and **the fall is the prediction error**. Let it go down.
- **Nothing learns while the scaffold drives.** A controller still updating on actions it did
  not issue is learning the scaffold's policy — doctrine §5.6, and invisible, because the brain
  would look like it was learning to get up. The run reports the frozen fraction, and it must
  equal the scaffold's share of ticks. *(Deferred subtlety: a* forward *model learning from
  those actions is learning the* body*, which is legitimate off-policy data. MotorEPM drives
  both from one TLE, so freezing everything is the safe default and the split becomes its own
  lever.)*
- **Both edges announce themselves** as `reset:handoff` / `reset:handback` in the JSONL. The
  picrawler paid for this: its auto-reset fired no event, MotorEPM's leg-phase and EMAs survived
  fall-plus-respawn, and every trend spanning a reset was fake.

**`--stub-amp 0` is the one to run.** The stub then emits *exactly* the home pose — no
controller at all — and the robot still falls 44 times in 60 s, with the harness keeping it
running the whole time. That is this body's lack of a passive equilibrium, demonstrated rather
than asserted.

⚠ A run against the stub says the **harness** works. It says nothing whatever about the
substrate, and no number from it is a baseline.

## The standing scaffold already recovers from a fall

Measured, not assumed: knocked past 90° at 10 N and flat at 25 N, it stands itself back up
five times out of five, each in about 1.6–2.4 s.

That matters beyond being fun to watch. The plan's phase A2 plans a fall-recovery scaffold as
the reset mechanism for when the brain drives the joints, and expected to need `StandUp` or
`VelStand` for it. **The file already vendored here does that job**, so A2 needs no new
dependency — only the hand-off logic and its `events.reset`.

## Cross-checked against an independent implementation

The same episode was run twice, once through this host (MuJoCo C API) and once through a Python
probe (MuJoCo bindings), on the same scene and the same policy:

| 5 s hold | `tilt_end` | `z_end` | drift |
|---|---|---|---|
| `ogma_mjhost --hold --secs 5` | 0.48° | 0.1163 m | 0.003 m |
| Python probe | 0.48° | 0.1163 m | 0.003 m |

Two implementations, one trajectory. That is the check worth having on
`Observation.cpp`, where a wrong offset produces a plausible robot rather than an error.

## Not here yet, deliberately

**No viewer inside the host.** Watching happens through
[`tools/duck_viewer`](../tools/duck_viewer), which renders the host's own output rather than
linking a windowing toolkit into the process that owns the physics. The host stays headless and
the observation path stays a reader, which is the same separation `xaq_inspector` has from the
picrawler.

## Notes for whoever builds S2 on this

- **MuJoCo 3.12 widened the model dimensions to `mjtSize` (`int64_t`).** `nq`, `nu`, `nbody`
  and friends are no longer `int`. Iterate with `mjtSize` and print through `long long`; a
  blind `%d` truncates silently on a future pin.
- **The version pin is part of the body of record**, not build trivia. MJCF parsing is
  version-sensitive and every number in the model README was measured under 3.12.0. Re-pin
  deliberately, and re-run `--load-only` when you do.
- **The host reports library *and* header version** and flags a skew. A pinned prebuilt should
  make that impossible, which is exactly why it is worth seeing if it ever happens.
