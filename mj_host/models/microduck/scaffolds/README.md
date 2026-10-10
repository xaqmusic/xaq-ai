# Scaffolds

**A scaffold is a prop, named as one.** Everything in this directory is somebody else's
trained controller, used to hold the body up while our own work happens elsewhere. None of it
is the brain, none of it learns, and every use of it is expected to carry a de-scaffolding
path (doctrine §"scaffold / de-scaffold").

The directory is named so that nothing in here can quietly become load-bearing without
somebody reading the word "scaffolds" in the path first.

| File | Provenance | Used by |
|---|---|---|
| `alpha_stand.onnx` | [`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck) at `590b986`, `policies/alpha_stand.onnx`, SHA-256 `15692687…` | phase S1 — holding the pose; the rescue driver |
| `ball_kick_left.onnx`, `ball_kick_right.onnx`, `roulade.onnx` | the same repository at `3954496`, `policies/`, SHA-256 `d6928284…`, `147a32c3…`, `3d60da08…` (the current release seeds them from the `pollen-robotics/microduck-policies` Hub repository instead) | the things phase's skills at the intent boundary (2026-09-17, register O54): one-shot networks run as their daemon runs them (a 0.5 s kick window, a 1 s roll, at standing tuning with a zero command), requested by name |
| `alpha_ground_pick.onnx` | `pollen-robotics/microduck-policies` on the Hub, `v1`, SHA-256 `ffbf5109…` (the set the release seeds a fresh board with) | the peck (2026-09-18, O54): their ground pick, phase-driven as their daemon drives it (2.8 s, the phase in the twist slots); nothing is grasped in simulation, it is a reach-down the outcome loop can choose |
| `alpha_walking.onnx` | the same repository at `3954496`, `policies/alpha_walking.onnx` ("velstand": walking on velocity commands and fall recovery in one network), SHA-256 `e36332d3…` | the intent boundary's walker — steps and walks on request, driven as their runtime drives it (0.9 action scale, 0.7/0.5 target low-pass) |

Neither file is tracked in git (`*.onnx` is ignored); `mj_host/scripts/fetch_scaffolds.sh` fetches both
by pinned commit and checks the hashes, and `run.sh` calls it when one is missing.

Apache-2.0, as is this repository. Attribution is in
[`THIRD_PARTY_NOTICES.md`](../../../THIRD_PARTY_NOTICES.md).

## Why S1 needs one at all

The plan originally had S1 hold the `STAND` keyframe's own `ctrl` with no controller in the
loop, as the honest no-brain baseline. **Measurement killed that.** A passive hold topples the
robot to 81° of tilt at every noise level including zero, with the actuators peaking at
0.13 N·m against a 0.96 N·m limit — soft, not saturated.

This body has no passive standing equilibrium. It is an inverted pendulum carrying 38 % of its
mass on a four-DoF head boom, and something has to ride it every tick. So the only no-brain
baseline available is an *actively balanced* one, and that is what this file is for.

## What it is not

It is not a teacher. Nothing distils it, nothing regresses onto its outputs, and no percept is
copied from it (doctrine §5.6). It holds the body; our own error signals are measured against
what the body then does.

Its second use, later, is as the **reset mechanism** in phase A2: when the brain drives the
joints and falls over, this hands the robot back to its feet and then gets out of the way. That
is the harness, not the controller — the same category as the Godot body's auto-reset teleport,
and it publishes `events.reset` every time it fires so no learning signal spans the boundary.
