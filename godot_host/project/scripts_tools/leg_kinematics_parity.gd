# Emits _fk_leg's inputs and its three output Transform3Ds as float32 BIT PATTERNS, so
# ogma::body::fk_leg can be checked for byte-identity rather than for "close enough".
#
# The FK block below is copied VERBATIM from picrawler_body.gd::_fk_leg — the oracle has
# to be the real expressions, because the whole question is rounding and evaluation
# order. Anchors are synthetic but realistically shaped (hip rectangle at ±38 mm, the
# CAD segment lengths); the numerics do not care, and using synthetic ones keeps this a
# unit test that needs no body instance.
#
# ⚠ This is the DEBUGGING instrument, not the gate. The real gate is a full-sim
# OGMA_PICRAWLER_TRACE diff before/after the swap, which exercises the genuine anchors.
# This exists so that when the trace differs, the fault can be localised in one run.
#
#   godot4 --headless --path godot_host/project -s res://scripts_tools/leg_kinematics_parity.gd
extends SceneTree

const N: int = 400

func f32(v: float) -> String:
	var b := PackedFloat32Array([v]).to_byte_array()
	return "%02x%02x%02x%02x" % [b[3], b[2], b[1], b[0]]

func f64(v: float) -> String:
	var b := PackedFloat64Array([v]).to_byte_array()
	return "%02x%02x%02x%02x%02x%02x%02x%02x" % [b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]]

func v3(v: Vector3) -> String:
	return "%s %s %s" % [f32(v.x), f32(v.y), f32(v.z)]

func xf(t: Transform3D) -> String:
	return "%s %s %s %s" % [v3(t.basis.x), v3(t.basis.y), v3(t.basis.z), v3(t.origin)]

func _init() -> void:
	var f := FileAccess.open("/tmp/legfk_parity.txt", FileAccess.WRITE)
	for n in range(N):
		var u: float = float(n) * 0.0173
		# Per-case anchors: realistic scale, varied so no accidental symmetry hides a bug.
		var sx: float = 1.0 if (n % 2) == 0 else -1.0
		var sz: float = 1.0 if (n % 4) < 2 else -1.0
		var hip1 := Vector3(sx * 0.038, 0.109 + 0.001 * sin(u), sz * 0.038)
		var hip2 := hip1 + Vector3(sx * 0.032, -0.007, sz * 0.0)
		var knee := hip2 + Vector3(sx * 0.048, 0.0, sz * 0.001 * cos(u))
		var coxa_c  := hip1 + Vector3(0.004 * sin(u), 0.002, 0.003)
		var upper_c := hip2 + Vector3(0.006, 0.001 * cos(u), 0.002)
		var lower_c := knee + Vector3(0.005, -0.030, 0.001)
		var hip2_axis := Vector3(sz * 0.7071, 0.0, -sx * 0.7071).normalized()
		var knee_axis := hip2_axis
		var t1: float = 0.6 * sin(u * 1.3)
		var t2: float = 0.9 * cos(u * 0.7)
		var t3: float = -1.1 * sin(u * 1.9)
		var lift_y: float = 0.01 * sin(u * 0.31)

		# ---- verbatim from _fk_leg ------------------------------------------------------
		var lift: Vector3 = Vector3(0, lift_y, 0)
		var hip1_w: Vector3 = hip1 + lift
		var hip2_w: Vector3 = hip2 + lift
		var knee_w: Vector3 = knee + lift
		var coxa_cc:  Vector3 = coxa_c  + lift
		var upper_cc: Vector3 = upper_c + lift
		var lower_cc: Vector3 = lower_c + lift
		var rot1: Basis = Basis(Quaternion(Vector3.UP, t1))
		var h1: Transform3D = Transform3D(rot1, hip1_w - rot1 * hip1_w)
		var t_coxa: Transform3D = h1 * Transform3D(Basis.IDENTITY, coxa_cc)
		var hip2_w_now: Vector3 = h1 * hip2_w
		var hip2_axis_now: Vector3 = rot1 * hip2_axis
		var rot2: Basis = Basis(Quaternion(hip2_axis_now, t2))
		var h2: Transform3D = Transform3D(rot2, hip2_w_now - rot2 * hip2_w_now)
		var t_upper: Transform3D = h2 * h1 * Transform3D(Basis.IDENTITY, upper_cc)
		var knee_w_now: Vector3 = h2 * h1 * knee_w
		var knee_axis_now: Vector3 = rot2 * rot1 * knee_axis
		var rot3: Basis = Basis(Quaternion(knee_axis_now, t3))
		var h3: Transform3D = Transform3D(rot3, knee_w_now - rot3 * knee_w_now)
		var t_lower: Transform3D = h3 * h2 * h1 * Transform3D(Basis.IDENTITY, lower_cc)
		# ---------------------------------------------------------------------------------

		f.store_line("%s | %s | %s | %s | %s | %s | %s | %s | %s %s %s %s | %s | %s | %s" % [
			v3(hip1), v3(hip2), v3(knee), v3(coxa_c), v3(upper_c), v3(lower_c),
			v3(hip2_axis), v3(knee_axis),
			f64(t1), f64(t2), f64(t3), f64(lift_y),
			xf(t_coxa), xf(t_upper), xf(t_lower)])
	f.close()
	print("wrote /tmp/legfk_parity.txt  (", N, " cases)")
	quit(0)
