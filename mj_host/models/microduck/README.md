# Vendored Microduck model

**Copied unmodified from [`pollen-robotics/microduck_rl`](https://github.com/pollen-robotics/microduck_rl)
at commit `d424a0c899f6b33cbd3daeb279913134349c0b63`**, path
`src/mjlab_microduck/robot/microduck/`. Apache-2.0, as is this repository; the attribution
is recorded in [`THIRD_PARTY_NOTICES.md`](../../../THIRD_PARTY_NOTICES.md).

**Do not edit these files.** Gate G1 of the
[port plan](../../../docs/plans-and-designs/microduck_port_plan.md) is that the model loads
with zero local changes, so a re-vendor stays a copy rather than a merge. Anything we need
different goes in an overlay file beside them, named as an overlay.

## What is here

| File | Role |
|---|---|
| `scene.xml` | floor, lighting, four keyframes (`INIT`/`STAND`/`SIT`/`FOLD`); includes `robot_allcollisions.xml`. **The default** |
| `scene_walk.xml` | the same scene over the reduced-collision model |
| `robot_allcollisions.xml` | full collision set — the one a destabilising brain needs, since it will put the trunk on the floor |
| `robot_walk.xml` | reduced collision set, as used for the walking task |
| `assets/*.stl` | 38 meshes, the ones the two models reference |

Both robot files are self-contained: `onshape-to-robot` inlines `joints_properties.xml`,
`additional.xml` and `sensors.xml` at export, so those upstream sources are not runtime
inputs and are not copied. The 47 `.part` files beside the meshes upstream are Onshape
intermediates that MuJoCo never reads, and are likewise skipped.

## Verified against MuJoCo 3.12.0

`scene.xml` and `scene_walk.xml` both load unmodified:

| | `scene.xml` | `scene_walk.xml` |
|---|---|---|
| `nq` / `nv` / `nu` | 21 / 20 / 14 | 21 / 20 / 14 |
| bodies / geoms / sensors / keyframes | 16 / 82 / 6 / 4 | 16 / 76 / 6 / 4 |
| `opt.timestep` | 0.002 s | 0.002 s |

`ctrl[i]` drives joint `i`, in exactly `JOINT_NAMES`-minus-mouth order. The host still
resolves the mapping by name (gate G4): an identity that holds today is not a contract.

## What the model does not carry

**The BAM actuator.** `microduck_rl`'s headline sim2real fidelity — the voltage-controlled
XL330 model with Coulomb, Stribeck and load-dependent friction — lives in the Python package
`better-actuator-models` and is installed onto the model at runtime by mjlab. What is in this
XML is the plain `position` actuator (`kp=0.55`, `kv=0`, `forcerange=±0.96`,
`ctrlrange=±10`), with `damping=0.053 frictionloss=0.0048 armature=0.0018` on the joint.

That difference is measurable and it matters. The shipped standing policy holds this model at
0.47° of tilt, but the walking policy stalls: it commands a 0.224 rad stride, the joints
deliver 0.118 rad with 0.213 rad of tracking error, and the robot travels 9 mm in 15 s. At
five times the stiffness the same policy walks 1.595 m. **Posture work is sound on this
actuator; no locomotion claim is** — see the port plan's BAM spec.

**No passive standing equilibrium.** Holding the `STAND` keyframe's own `ctrl` topples the
robot to 81° at every noise level including zero, with the actuators peaking at 0.13 N·m
against a 0.96 N·m limit. They are soft, not saturated. This body is always actively
balancing, which is what a robot carrying 38 % of its mass on a four-DoF head boom should be
expected to do.
