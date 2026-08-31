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

```sh
cmake -S mj_host -B mj_host/build -DCMAKE_BUILD_TYPE=Release
cmake --build mj_host/build -j8

./mj_host/build/ogma_mjhost --load-only     # G1 / G3 / G4, on models/microduck/scene.xml
./mj_host/build/ogma_mjhost --gate-g2       # the settle sweep: 4 noise levels x 3 seeds
./mj_host/build/ogma_mjhost --hold --secs 5 # run the scaffold, JSONL on stdout
```

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

**No interactive viewer.** MuJoCo's own `simulate` is a separate GLFW application, and
embedding one would add a windowing dependency to a host whose workflow is headless runs plus
the ZMQ inspector (S2). Any trajectory this host produces can be rendered from Python against
the same vendored model when something needs watching. Revisit when there is a brain worth
watching live.

## Notes for whoever builds S2 on this

- **MuJoCo 3.12 widened the model dimensions to `mjtSize` (`int64_t`).** `nq`, `nu`, `nbody`
  and friends are no longer `int`. Iterate with `mjtSize` and print through `long long`; a
  blind `%d` truncates silently on a future pin.
- **The version pin is part of the body of record**, not build trivia. MJCF parsing is
  version-sensitive and every number in the model README was measured under 3.12.0. Re-pin
  deliberately, and re-run `--load-only` when you do.
- **The host reports library *and* header version** and flags a skew. A pinned prebuilt should
  make that impossible, which is exactly why it is worth seeing if it ever happens.
