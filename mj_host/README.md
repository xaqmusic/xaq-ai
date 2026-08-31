# mj_host — the Microduck MuJoCo host

The third host, beside [`godot_host/`](../godot_host) (the picrawler, in Godot) and
[`pi_host/`](../pi_host) (the picrawler, on real hardware). `OgmaInstance.hpp` already names
*"the host (Godot Host, HAL Host, Debug Host)"* as the owner of the Bus, so this is the
anticipated shape rather than a new one: it holds an `OgmaInstance` in plain C++ and never
touches `OgmaBrain`'s Godot `Variant` marshalling.

Plan and rationale: [`docs/plans-and-designs/microduck_port_plan.md`](../docs/plans-and-designs/microduck_port_plan.md).
**Phase S0 is done; there is no brain in here yet.**

| piece | what it is |
|---|---|
| `models/microduck/` | the body, [vendored unmodified](models/microduck/README.md) from `microduck_rl` at `d424a0c` |
| `src/main.cpp` | `--load-only`: loads the model and checks gates G1, G3 and G4 |
| `CMakeLists.txt` | MuJoCo 3.12.0 pinned by version **and** SHA-256; `ogma_core` behind an off-by-default option |

## Build and run

```sh
cmake -S mj_host -B mj_host/build -DCMAKE_BUILD_TYPE=Release
cmake --build mj_host/build -j8
./mj_host/build/ogma_mjhost --load-only              # defaults to models/microduck/scene.xml
./mj_host/build/ogma_mjhost --load-only mj_host/models/microduck/scene_walk.xml
```

MuJoCo downloads on the first configure (31 MB) and is cached thereafter. Nothing else is
needed: no Python, no mjlab, no CUDA, and `ogma_core` is not built unless you ask for it with
`-DMJ_HOST_WITH_BRAIN=ON` (S1 onward).

## What `--load-only` is for

It is gate G1 with its working shown, and it exits non-zero when a gate fails, so it belongs
in CI rather than in a person's memory.

| Gate | Question | Answer today |
|---|---|---|
| **G1** | does Pollen's MJCF load with zero local edits? | PASS — `scene.xml` and `scene_walk.xml` both |
| **G3** | is the 50 Hz brain tick a whole number of physics steps? | PASS — 0.002 s timestep, **10 substeps** |
| **G4** | does `ctrl[i]` drive the joint this project thinks it does? | PASS — identity, in `JOINT_NAMES`-minus-mouth order |

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

## Notes for whoever builds S1 on this

- **MuJoCo 3.12 widened the model dimensions to `mjtSize` (`int64_t`).** `nq`, `nu`, `nbody`
  and friends are no longer `int`. Iterate with `mjtSize` and print through `long long`; a
  blind `%d` truncates silently on a future pin.
- **The version pin is part of the body of record**, not build trivia. MJCF parsing is
  version-sensitive and every number in the model README was measured under 3.12.0. Re-pin
  deliberately, and re-run `--load-only` when you do.
- **The host reports library *and* header version** and flags a skew. A pinned prebuilt should
  make that impossible, which is exactly why it is worth seeing if it ever happens.
