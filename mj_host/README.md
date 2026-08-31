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
```

`watch` and `record` take the host's own arguments, so `--secs`, `--noise`, `--seed` and a
scene path all pass straight through:

```sh
./mj_host/run.sh watch --secs 10 --noise 0.05 --seed 2
```

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
