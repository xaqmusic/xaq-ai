extends RefCounted
## ServoPowerModel — what each servo's motor is actually doing, recovered from the physics.
##
## WHY (2026-10-05, power-budget plan S1).  On the robot, the HAT's 5 V regulator caps battery-side
## current at ~3.5 A, and when servo demand exceeds that cap the HAT's own 3.3 V rail droops and the
## servo MCU resets (ledger, "3.3 V RAIL" and "SURFACE A/B").  To let the sim feel that, it needs
## a servo CURRENT, and a hobby servo's current is set by the torque its motor delivers
## (DC motor: I ≈ I_idle + |τ| / k_t), not by commanded motion.  The robot proved it: commanded
## motion and feet loaded predict measured current with test R² 0.06-0.09.
##
## The sim's motors are velocity constraints with a torque cap, and Godot Physics 3D does not
## expose a joint's applied impulse.  So the torque is recovered by NEWTON-EULER ON THE DISTAL
## SUBCHAIN.  For joint j, take every segment beyond it (hip1: coxa + upper + lower; hip2: upper +
## lower; knee: lower) and its anchor p (moving) and axis a:
##     a · τ_motor = a · ( dL_p/dt + v_p × P − τ_gravity − τ_contacts − τ_damping )
## L_p is the subchain's angular momentum about p and P its linear momentum.  The joints inside
## the subchain cancel, and the parent joint's constraint force acts through p, so it has no moment.
## Its constraint torque is perpendicular to a.  What is left along a is the motor, plus the
## joint-limit torque, which on the robot is also the servo.  Contacts on EVERY leg segment count,
## not just the foot: a shin snagged on a rock is exactly the stall that drains the rail.
##
## INSTRUMENT ONLY.  Nothing here writes to the physics.  The one side effect, contact reporting
## on coxa/upper segments, is switched on only when the model is enabled.
##
## Read in _physics_process, BEFORE the step: velocities and contact impulses then both belong to
## the step just completed, so dL/dt over that step pairs with the impulses it applied.

const N_JOINTS := 12                 # index = leg * 3 + jt (jt 0 hip1, 1 hip2, 2 knee)

var enabled := false
var tau := PackedFloat32Array()      # per joint, N·m, along the joint axis (signed)
var omega := PackedFloat32Array()    # per joint, rad/s, child relative to parent along the axis
var _have_prev := false
var _L_prev := PackedVector3Array()
var _parent: Array = []              # RigidBody3D per joint
var _anchor_local := PackedVector3Array()
var _axis_local := PackedVector3Array()
var _chain: Array = []               # per joint: Array of segment indices into _segs
var _child: Array = []               # per joint: the segment the joint drives (for omega)
var _segs: Array = []                # RigidBody3D: [coxa0, upper0, lower0, coxa1, ...]
var _gravity := Vector3(0, -9.8, 0)
var _com_prev := PackedVector3Array()
## A segment that moved further than this in one step was teleported (reset, spawn, live
## body swap): the momentum difference across it is not a torque, so that step is skipped.
const TELEPORT_M := 0.05
var skipped_steps := 0


func setup(chassis: RigidBody3D, coxas: Array, uppers: Array, lowers: Array,
		   hip1_w: Array, hip2_w: Array, knee_w: Array, lateral_axes: Array) -> void:
	tau.resize(N_JOINTS); omega.resize(N_JOINTS); _L_prev.resize(N_JOINTS)
	_anchor_local.resize(N_JOINTS); _axis_local.resize(N_JOINTS)
	_parent.resize(N_JOINTS); _chain.resize(N_JOINTS); _child.resize(N_JOINTS)
	_segs.clear()
	_gravity = float(ProjectSettings.get_setting("physics/3d/default_gravity", 9.8)) \
			 * Vector3(ProjectSettings.get_setting("physics/3d/default_gravity_vector", Vector3.DOWN))
	for i in range(4):
		_segs.append_array([coxas[i], uppers[i], lowers[i]])
		for b in [coxas[i], uppers[i]]:            # lowers already report contacts (foot sensor)
			b.contact_monitor = true
			b.max_contacts_reported = 4
		var parents: Array = [chassis, coxas[i], uppers[i]]
		var anchors: Array = [hip1_w[i], hip2_w[i], knee_w[i]]
		var axes: Array = [Vector3.UP, lateral_axes[i], lateral_axes[i]]
		for jt in range(3):
			var k := i * 3 + jt
			var inv: Transform3D = (parents[jt] as RigidBody3D).global_transform.affine_inverse()
			_parent[k] = parents[jt]
			_anchor_local[k] = inv * (anchors[jt] as Vector3)
			_axis_local[k] = (inv.basis * (axes[jt] as Vector3)).normalized()
			_chain[k] = range(i * 3 + jt, i * 3 + 3)
			_child[k] = i * 3 + jt
	_have_prev = false
	enabled = true


## One physics step.  dt = 1 / physics_hz.
func step(dt: float) -> void:
	if not enabled:
		return
	# Per-segment state, once.
	var n := _segs.size()
	var com := PackedVector3Array(); com.resize(n)
	var mom := PackedVector3Array(); mom.resize(n)          # m v
	var spin := PackedVector3Array(); spin.resize(n)        # I_world ω
	var wv := PackedVector3Array(); wv.resize(n)
	var fg := PackedVector3Array(); fg.resize(n)            # gravity + linear damping force
	var tq_d := PackedVector3Array(); tq_d.resize(n)        # angular damping torque
	var c_pos: Array = []                                    # per segment: contact points
	var c_frc: Array = []                                    # per segment: contact forces
	for s in range(n):
		var b: RigidBody3D = _segs[s]
		var st := PhysicsServer3D.body_get_direct_state(b.get_rid())
		com[s] = st.transform.origin + st.center_of_mass
		mom[s] = b.mass * st.linear_velocity
		wv[s] = st.angular_velocity
		# A frozen body (calibration hold, spawn) reports a zero inverse inertia: it has no
		# spin momentum the solver tracks, and inverting it would raise every step.
		var inv_I: Basis = st.inverse_inertia_tensor
		spin[s] = (inv_I.inverse() * st.angular_velocity) if absf(inv_I.determinant()) > 1e-12 else Vector3.ZERO
		fg[s] = b.mass * b.gravity_scale * _gravity - st.total_linear_damp * mom[s]
		tq_d[s] = -st.total_angular_damp * spin[s]
		var ps := PackedVector3Array(); var fs := PackedVector3Array()
		for j in range(st.get_contact_count()):
			ps.append(st.get_contact_local_position(j))     # global frame, despite the name
			fs.append(st.get_contact_impulse(j) / dt)
		c_pos.append(ps); c_frc.append(fs)
	if _com_prev.size() == n:
		for s in range(n):
			if com[s].distance_to(_com_prev[s]) > TELEPORT_M:
				_have_prev = false
				skipped_steps += 1
				break
	_com_prev = com
	for k in range(N_JOINTS):
		var par: RigidBody3D = _parent[k]
		var pst := PhysicsServer3D.body_get_direct_state(par.get_rid())
		var p: Vector3 = par.global_transform * _anchor_local[k]
		var a: Vector3 = (par.global_transform.basis * _axis_local[k]).normalized()
		var v_p: Vector3 = pst.linear_velocity + pst.angular_velocity.cross(p - (pst.transform.origin + pst.center_of_mass))
		var L := Vector3.ZERO
		var P := Vector3.ZERO
		var t_ext := Vector3.ZERO
		for s in _chain[k]:
			var r: Vector3 = com[s] - p
			L += r.cross(mom[s]) + spin[s]
			P += mom[s]
			t_ext += r.cross(fg[s]) + tq_d[s]
			var ps: PackedVector3Array = c_pos[s]
			var fs: PackedVector3Array = c_frc[s]
			for j in range(ps.size()):
				t_ext += (ps[j] - p).cross(fs[j])
		if _have_prev:
			var dLdt: Vector3 = (L - _L_prev[k]) / dt
			tau[k] = a.dot(dLdt + v_p.cross(P) - t_ext)
		_L_prev[k] = L
		omega[k] = a.dot(wv[_child[k]] - pst.angular_velocity)
	_have_prev = true


## The momentum memory spans one step; after a teleport or reset it is meaningless.
func invalidate() -> void:
	_have_prev = false


# ---------------------------------------------------------------------------------------------
# ELECTRICAL MODEL — servo current, pack voltage and the HAT's 3.3 V rail, from the torques above.
#
# A STEP-FOR-STEP PORT of electrical() in pi_host/tools/brainrun/power_calib.py, which holds the
# fit and its provenance (2026-10-06: K fitted to the robot's carpet mean; holding ~free, 1.2 A
# stall and the 3.3 s stall cut-off from the scale probe and SunFounder's SF006PRO).  Change both
# together; tools/power_parity.py checks them against each other on a power log.
# Shape caveat, from the fit: the sim's current is NARROWER than the robot's (p90 2.2 vs 2.8 A).
# ---------------------------------------------------------------------------------------------
const E_I_IDLE := 0.005
const E_I_NL := 0.05
const E_W_NL := 6.2
const E_I_STALL := 1.2
const E_K := 2.49
const E_H_HOLD := 0.0
const E_P_BLEND := 0.02
const E_TAU_MECH := 0.05
const E_TAU_TQ := 0.02
const E_STALL_CUT_S := 3.3
const E_I_CUT := 0.25
const E_TAU_BUS := 0.004
const E_ETA := 0.85
const E_V_REST := 7.86
const E_R_PACK := 0.245
const E_I_LOGIC := 0.10
const E_I5_LIM := 4.05
const E_GAMMA := 1.0
const E_LDO_DROP := 0.25

var i_servo := PackedFloat32Array()  # per servo, A at 5 V
var i5 := 0.0                        # servo bus demand, A at 5 V
var i_bat := 0.0                     # battery-side HAT current, A (what the INA219 reads)
var v_pack := E_V_REST               # pack voltage, V
var v_rail := 3.3                    # the HAT's 3.3 V rail, V
var stalled := PackedByteArray()     # per servo: saturated long enough to cut its own drive
var _e_init := false
var _ts := PackedFloat32Array()
var _ws := PackedFloat32Array()
var _tq := PackedFloat32Array()
var _stall_run := PackedFloat32Array()


func electrical_step(dt: float) -> void:
	if not enabled:
		return
	if not _e_init:
		_ts = tau.duplicate(); _ws = omega.duplicate(); _tq = tau.duplicate()
		_stall_run.resize(N_JOINTS); _stall_run.fill(0.0)
		i_servo.resize(N_JOINTS); stalled.resize(N_JOINTS); stalled.fill(0)
	var a_m: float = dt / (E_TAU_MECH + dt)
	var a_t: float = dt / (E_TAU_TQ + dt)
	var raw := 0.0
	for k in range(N_JOINTS):
		if _e_init:
			_ts[k] += a_m * (tau[k] - _ts[k])
			_ws[k] += a_m * (omega[k] - _ws[k])
			_tq[k] += a_t * (tau[k] - _tq[k])
		var x: float = clampf(_ts[k] * _ws[k] / E_P_BLEND, 0.0, 1.0)
		var g: float = E_H_HOLD + (1.0 - E_H_HOLD) * x * x * (3.0 - 2.0 * x)
		var drive: float = minf(E_I_STALL, E_K * g * absf(_tq[k]))
		_stall_run[k] = (_stall_run[k] + dt) if drive >= 0.9 * E_I_STALL else 0.0
		stalled[k] = 1 if _stall_run[k] > E_STALL_CUT_S else 0
		if stalled[k] != 0:
			drive = minf(drive, E_I_CUT)
		i_servo[k] = E_I_IDLE + E_I_NL * minf(1.0, absf(omega[k]) / E_W_NL) + drive
		raw += i_servo[k]
	if not _e_init:
		i5 = raw
		_e_init = true
	else:
		i5 += dt / (E_TAU_BUS + dt) * (raw - i5)
	i_bat = minf(i5, E_I5_LIM) * 5.0 / (E_ETA * v_pack) + E_I_LOGIC
	v_pack = E_V_REST - E_R_PACK * i_bat
	var v5: float = 5.0 * pow(E_I5_LIM / maxf(i5, 1e-9), E_GAMMA) if i5 > E_I5_LIM else 5.0
	v_rail = minf(3.3, v5 - E_LDO_DROP)
