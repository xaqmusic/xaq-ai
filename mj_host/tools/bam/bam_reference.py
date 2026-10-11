"""An independent reference for the host's BAM port (design doc §17.113): the duck's MJCF edited at the SPEC level as
bam.mjlab.BamActuator.edit_spec does, the firmware law and the motor torque from Rhoban's own bam classes (XL330, m6), and the
m6 friction budget written from bam/mjlab.py.  Replays the host's per-tick targets (--bam-dump) from the host's first state and
compares the robot's joints tick by tick."""
import sys, numpy as np, mujoco
from bam.model import load_model

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch", "head_pitch", "head_yaw",
          "head_roll", "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]
scene, dump = sys.argv[1], sys.argv[2]
VIN, KP, VIN_MAX = 7.35, 200.0, 8.2

bam = load_model(motor_name="xl330", model="m6"); act = bam.actuator
act.kp, act.vin = KP, VIN
kt, R = bam.kt.value, bam.R.value

spec = mujoco.MjSpec.from_file(scene)
force_limit = VIN_MAX * kt / R
for a in spec.actuators:
    if a.name in JOINTS:
        a.set_to_motor(); a.forcelimited = True; a.forcerange = [-force_limit, force_limit]; a.gear = [1, 0, 0, 0, 0, 0]
        a.ctrllimited = False
for j in spec.joints:
    if j.name in JOINTS:
        j.armature = act.get_extra_inertia(); j.damping = np.zeros((3, 1)); j.frictionloss = 0.0
        j.solref_friction = [-5.0e4, -2.0e2]; j.solimp_friction = [0.99, 0.9999, 0.001, 0.5, 2.0]
m = spec.compile(); d = mujoco.MjData(m)
acts = [m.actuator(n).id for n in JOINTS]
jids = [m.joint(n).id for n in JOINTS]
qadr = np.array([m.jnt_qposadr[j] for j in jids]); dadr = np.array([m.jnt_dofadr[j] for j in jids])

rows = [np.array(l.split()[1:], dtype=float) for l in open(dump)]
nq, nv = m.nq, m.nv
tgt = lambda r: r[:14]; qp = lambda r: r[14:14 + nq]; qv = lambda r: r[14 + nq:14 + nq + nv]
d.qpos[:] = qp(rows[0]); d.qvel[:] = qv(rows[0]); mujoco.mj_forward(m, d)
substeps = int(round(0.02 / m.opt.timestep))

def bam_step(target):
    q, dq = d.qpos[qadr], d.qvel[dadr]
    prev = d.qfrc_actuator[dadr].copy()
    control = act.compute_control(target, q, dq, m.opt.timestep)          # bam's firmware law (+ current limiter, PWM clamp)
    tau = act.compute_torque(control, True, q, dq)                       # bam's DC motor with back-EMF
    d.ctrl[acts] = tau
    fric = np.zeros(nv)
    for r in range(d.nefc):
        if d.efc_type[r] == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF: fric[d.efc_id[r]] += d.efc_force[r]
    ext = -d.qfrc_bias[dadr] + d.qfrc_constraint[dadr] - fric[dadr]
    strib = np.exp(-np.power(np.abs(dq) / bam.dtheta_stribeck.value, bam.alpha.value))
    fl = np.full(14, bam.friction_base.value) + strib * bam.friction_stribeck.value
    fl += np.abs(ext * bam.load_friction_external.value - prev * bam.load_friction_motor.value)
    fl += strib * np.abs(ext * bam.load_friction_external_stribeck.value - prev * bam.load_friction_motor_stribeck.value)
    drive = (np.abs(prev) > np.abs(ext)).astype(float)
    fl += strib * (drive * bam.load_friction_external_quad.value * ext ** 2 + (1 - drive) * bam.load_friction_motor_quad.value * prev ** 2)
    m.dof_frictionloss[dadr] = fl; m.dof_damping[dadr] = bam.friction_viscous.value

err = []
for t in range(len(rows) - 1):
    for _ in range(substeps):
        bam_step(tgt(rows[t])); mujoco.mj_step(m, d)
    host_q = qp(rows[t + 1])[qadr]
    err.append(np.max(np.abs(d.qpos[qadr] - host_q)))
    if t in (0, 1, 4, 9, 24, 49, 99, 149, 199, 298) or t == len(rows) - 2:
        trunk = np.linalg.norm(d.qpos[:3] - qp(rows[t + 1])[:3])
        print(f"tick {t + 1:4d} ({(t + 1) * 0.02:5.2f} s): max joint |python - host| {err[-1]:.2e} rad, trunk position {trunk:.2e} m")

# ---- the per-tick check: from the host's own state at each tick, bam's firmware law and motor torque must give the host's
# first-substep torques (the --bam-dump "T" rows), independent of the replay's drift
if len(sys.argv) > 3:
    lines = open(sys.argv[3]).read().split("\n")
    st = [np.array(l.split()[1:], dtype=float) for l in lines if l and not l.startswith("T")]
    tq = [np.array(l.split()[1:], dtype=float) for l in lines if l.startswith("T")]
    worst = 0.0; worst_rel = 0.0
    for r, tr in zip(st, tq):
        q = r[14:14 + nq][qadr]; dq = r[14 + nq:14 + nq + nv][dadr]
        control = act.compute_control(r[:14], q, dq, m.opt.timestep)
        tau = act.compute_torque(control, True, q, dq)
        diff = np.max(np.abs(tau - tr[:14])); worst = max(worst, diff)
        worst_rel = max(worst_rel, diff / max(1e-9, np.max(np.abs(tr[:14]))))
    print(f"per-tick torque check over {len(tq)} ticks: max |bam - host| {worst:.3e} Nm (relative {worst_rel:.3e})")
