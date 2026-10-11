# Emits the GDScript stride-odometry chain's inputs AND outputs as BIT PATTERNS, so
# the C++ port (cpp_core/include/ogma/body/StrideOdometry.hpp) can be checked for
# byte-identity rather than for "close enough".
#
# Every expression in the marked blocks is copied VERBATIM from picrawler_body.gd —
# the oracle has to be the real expressions, because the whole question is rounding.
# Only the INPUTS are synthetic (the sim's toe positions come from FK on a physics
# body; here they come from a deterministic generator), since the FK itself is
# already ported and gated by leg_kinematics_parity.
#
# ⚠ THE STANCE SCHEDULE IS PART OF THE TEST.  StrideV has two branches and the coast
# branch (no planted feet) only fires on full-swing/airborne ticks.  An oracle driven
# by a plausible gait would take it rarely or never and still report "bit-exact" —
# so the load schedule below is built to hit stance counts 0..4, with 0 forced in
# runs long enough to matter.  Counts are asserted at the end, not hoped for.
#
# ⚠ Widths are load-bearing and are NOT uniform.  _strido_lp and the loads are
# Array[float] (DOUBLE); the toes/gyro/accel are Vector3 (float32 storage); slip is a
# bare float (DOUBLE) while est/bias are Vector2 (float32).  The declarations below
# mirror picrawler_body.gd exactly for that reason — do not "tidy" them to one type.
#
#   godot4 --headless --path godot_host/project -s res://scripts_tools/stride_odometry_parity.gd
#   -> writes /tmp/strido_parity.txt
extends SceneTree

const TAU: float = 0.02
const STRIDO_LP_ALPHA: float = 0.2
const STRIDE_V_LOAD_THRESH: float = 0.2
const STRIDE_V_FUSE_BETA: float = 1.0
const STRIDE_V_BIAS_KI: float = 0.1
const STRIDE_V_SLIP_ALPHA: float = 0.05
const STRIDE_V_COAST_LEAK: float = 0.005
const L3: float = 0.0765
const N: int = 900

func f32hex(v: float) -> String:
	var b := PackedFloat32Array([v]).to_byte_array()
	return "%02x%02x%02x%02x" % [b[3], b[2], b[1], b[0]]

func f64hex(v: float) -> String:
	var b := PackedFloat64Array([v]).to_byte_array()
	return "%02x%02x%02x%02x%02x%02x%02x%02x" % [b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]]

func v3hex(v: Vector3) -> String:
	return "%s %s %s" % [f32hex(v.x), f32hex(v.y), f32hex(v.z)]

func v2hex(v: Vector2) -> String:
	return "%s %s" % [f32hex(v.x), f32hex(v.y)]

func _init() -> void:
	var f := FileAccess.open("/tmp/strido_parity.txt", FileAccess.WRITE)

	# ---- state, declared at the widths picrawler_body.gd declares them -------------
	var strido_lp: Array[float] = []
	var prev_cmdlp: Array = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
	var prev_loaded: Array = [false, false, false, false]
	var stridev_est: Vector2 = Vector2.ZERO
	var stridev_bias: Vector2 = Vector2.ZERO
	var stridev_slip: float = 0.0
	var prev_valid: bool = false

	var n_by_stance: Array[int] = [0, 0, 0, 0, 0]

	for step in range(N):
		var t: float = float(step) * 0.02

		# ---- synthetic inputs, deliberately nasty ---------------------------------
		# Toes swing through a stride, the body yaws hard enough that the ω×p term is
		# not a rounding detail, and |a| swings across the attitude estimate.
		var eff: Array[float] = []
		for k in range(12):
			var leg: int = k / 3
			var jnt: int = k % 3
			eff.append(0.37 * sin(t * (1.3 + 0.4 * float(jnt)) + 1.57 * float(leg))
				- 0.11 * float(jnt))
		var gyro_mean := Vector3(0.9 * sin(t * 1.1), 2.3 * cos(t * 0.7), 1.4 * sin(t * 1.9))
		var accel := Vector3(2.6 * sin(t * 1.7), 9.81 + 3.4 * sin(t * 0.9), 2.1 * cos(t * 2.3))
		var up_est := Vector3(0.13 * sin(t * 0.6), 0.98, 0.11 * cos(t * 0.8)).normalized()

		# Loads: a per-foot wave riding a BASELINE that drifts slowly across the whole
		# range, so the number of feet over the 0.2 threshold sweeps 1..4 rather than
		# parking at the 2-3 a fixed baseline gives.  Plus a hard all-swing window
		# every 97 ticks, which is the only way stance 0 (the coast branch) is reached.
		var base: float = 0.42 + 0.45 * sin(t * 0.23)
		var loads: Array[float] = []
		for i in range(4):
			var airborne: bool = (step % 97) < 4
			loads.append(0.0 if airborne
				else base + 0.34 * sin(t * 2.1 + 1.57 * float(i)))

		# Toe positions: a stride ellipse per leg, in the body frame.
		var toe_cmdlp_b: Array = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
		for i in range(4):
			var ph: float = t * 2.4 + 1.57 * float(i)
			toe_cmdlp_b[i] = Vector3(
				(0.055 if i < 2 else -0.055) + 0.006 * cos(ph),
				-0.085 + 0.021 * sin(ph),
				(0.055 if i % 2 == 0 else -0.055) + 0.030 * sin(ph))

		# ---- VERBATIM: servo forward model (picrawler_body.gd ~:6677, ~:6698) -------
		if strido_lp.size() != 12:
			strido_lp.resize(12)
			for k in range(12):
				strido_lp[k] = eff[k]
		else:
			for i in range(4):
				strido_lp[i * 3]     += STRIDO_LP_ALPHA * (eff[i * 3] - strido_lp[i * 3])
				strido_lp[i * 3 + 1] += STRIDO_LP_ALPHA * (eff[i * 3 + 1] - strido_lp[i * 3 + 1])
				strido_lp[i * 3 + 2] += STRIDO_LP_ALPHA * (eff[i * 3 + 2] - strido_lp[i * 3 + 2])
		# ----------------------------------------------------------------------------

		# ---- VERBATIM: feet_y_gravity_cmd_imu (picrawler_body.gd ~:6749) ------------
		var feet_y: Array[float] = []
		for i in range(4):
			feet_y.append((toe_cmdlp_b[i] as Vector3).dot(up_est) - L3 * 0.5)
		# ----------------------------------------------------------------------------

		# ---- VERBATIM: the stance-FK accumulation (picrawler_body.gd ~:6783) --------
		var loaded02_now: Array = [false, false, false, false]
		for i in range(4):
			loaded02_now[i] = loads[i] >= STRIDE_V_LOAD_THRESH
		var sv_sensor := Vector3.ZERO
		var sv_ns_sensor: int = 0
		var v_per_leg: Array = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
		if prev_valid:
			for i in range(4):
				var v_clp_i: Vector3 = -((toe_cmdlp_b[i] - prev_cmdlp[i]) / TAU \
					+ gyro_mean.cross((toe_cmdlp_b[i] + prev_cmdlp[i]) * 0.5))
				v_per_leg[i] = v_clp_i
				if loaded02_now[i] and prev_loaded[i]:
					sv_sensor += v_clp_i
					sv_ns_sensor += 1
		# ----------------------------------------------------------------------------

		# ---- VERBATIM: the stride_v ⊕ slip fusion (picrawler_body.gd ~:6828) --------
		var a_lin: Vector3 = accel - 9.81 * up_est
		var v_pred := Vector2(stridev_est.x + (a_lin.x - stridev_bias.x) * TAU,
							  stridev_est.y + (a_lin.z - stridev_bias.y) * TAU)
		if sv_ns_sensor > 0:
			var v_fk := Vector2(sv_sensor.x / float(sv_ns_sensor),
								sv_sensor.z / float(sv_ns_sensor))
			var innov: Vector2 = v_fk - v_pred
			stridev_est = v_pred + STRIDE_V_FUSE_BETA * innov
			stridev_bias += -STRIDE_V_BIAS_KI * innov
			stridev_slip += STRIDE_V_SLIP_ALPHA * (innov.length() - stridev_slip)
		else:
			stridev_est = v_pred * (1.0 - STRIDE_V_COAST_LEAK)
		# ----------------------------------------------------------------------------

		n_by_stance[sv_ns_sensor] += 1

		# inputs | lp | feet_y | per-leg v | a_lin | est | bias | slip
		var cols: Array[String] = []
		for i in range(4):
			cols.append(v3hex(toe_cmdlp_b[i]))
		cols.append(v3hex(gyro_mean))
		cols.append(v3hex(accel))
		cols.append(v3hex(up_est))
		for k in range(12):
			cols.append(f64hex(eff[k]))
		for i in range(4):
			cols.append(f64hex(loads[i]))
		cols.append("%d" % (1 if prev_valid else 0))
		cols.append("|")
		for k in range(12):
			cols.append(f64hex(strido_lp[k]))
		for i in range(4):
			cols.append(f64hex(feet_y[i]))
		for i in range(4):
			cols.append(v3hex(v_per_leg[i]))
		cols.append("%d" % sv_ns_sensor)
		cols.append(v3hex(a_lin))
		cols.append(v2hex(stridev_est))
		cols.append(v2hex(stridev_bias))
		cols.append(f64hex(stridev_slip))
		f.store_line(" ".join(cols))

		# ---- VERBATIM: the carry-forward (picrawler_body.gd ~:6815) -----------------
		for i in range(4):
			prev_cmdlp[i] = toe_cmdlp_b[i]
			prev_loaded[i] = loaded02_now[i]
		prev_valid = true

	f.close()
	print("wrote /tmp/strido_parity.txt  (", N, " steps)")
	print("stance-count coverage  n=0:", n_by_stance[0], "  n=1:", n_by_stance[1],
		"  n=2:", n_by_stance[2], "  n=3:", n_by_stance[3], "  n=4:", n_by_stance[4])
	var missing: Array[int] = []
	for k in range(5):
		if n_by_stance[k] == 0:
			missing.append(k)
	if missing.is_empty():
		print("COVERAGE OK — every stance count 0..4 exercised, coast branch included")
	else:
		print("⚠ COVERAGE GAP — stance counts never reached: ", missing,
			"  (the gate cannot see what it does not run)")
	quit(0)
