extends SceneTree
# export_body_calib.gd — write the body's construction-time FK anchors to JSON for the robot.
#
# The port doc's Phase 0 rule: calibration exported from one side drops straight into the
# other, true BY CONSTRUCTION.  ogma_host runs the same ogma::body::fk_leg as the sim, and
# fk_leg is only the same maths if it gets the same anchors — which the sim builds in
# GDScript (_build_leg) from the body JSON.  Retyping them by hand is how a port turns into
# "the same numbers, not the same maths".  So the sim writes them, and the robot reads them.
#
# Usage (from godot_host/project):
#   OGMA_PICRAWLER_BODY=measured_fsr OUT=/abs/path/body_measured_fsr.json \
#   OGMA_PICRAWLER_CONFIG=res://addons/ami_ogma/configs/<any picrawler config>.json \
#     godot4 --headless --path . -s res://scripts_tools/export_body_calib.gd
# ⚠ A brain config is REQUIRED: without one the body aborts setup before _build_leg runs,
# the anchors stay empty, and this waits forever.
#
# Also written, as a parity reference: foot_b_zero[i] = rest_inv * fk(i, 0, 0, 0).lower.origin,
# computed by the body's OWN _fk_leg.  That is the constant behind feet_y_gravity_cmd_imu
# while cmd_fk_source = 0 (ledger 2026-10-02), so the robot's version can be checked against
# the sim's number rather than against a derivation.
var body: Node = null
var frames := 0

func _initialize() -> void:
	root.add_child(load("res://scenes/the_picrawler.tscn").instantiate())

func _v(v: Vector3) -> Array:
	return [v.x, v.y, v.z]

func _process(_d: float) -> bool:
	frames += 1
	if body == null:
		for n in root.find_children("*", "", true, false):
			if n.get("_hip1_world_c") != null and n.get("_lower_rest_xform") != null:
				body = n
				break
		return false
	if frames < 30 or (body._hip1_world_c as Array).size() < 4:
		return false
	var rest: Transform3D = body._chassis_rest_xform
	var rest_inv: Transform3D = rest.affine_inverse()
	var legs := []
	for i in range(4):
		legs.append({
			"name": ["fl", "fr", "rl", "rr"][i],
			"hip1_world": _v(body._hip1_world_c[i]),
			"hip2_world": _v(body._hip2_world_c[i]),
			"knee_world": _v(body._knee_world_c[i]),
			"coxa_rest_origin": _v((body._coxa_rest_xform[i] as Transform3D).origin),
			"upper_rest_origin": _v((body._upper_rest_xform[i] as Transform3D).origin),
			"lower_rest_origin": _v((body._lower_rest_xform[i] as Transform3D).origin),
			"hip2_axis": _v(body._hip2_axes[i]),
			"knee_axis": _v(body._knee_axes[i]),
			"toe_off_c": _v(body._toe_off_c[i]),
			"foot_b_zero": _v(rest_inv * (body._fk_leg(i, 0.0, 0.0, 0.0)[2] as Transform3D).origin),
			# Non-zero poses exercise the axes and the rotation chain; zero angles do not.
			"fk_check": [
				{"t": [0.3, -0.5, 0.8], "toe": _v(rest_inv * ((body._fk_leg(i, 0.3, -0.5, 0.8)[2] as Transform3D) * body._toe_off_c[i]))},
				{"t": [-0.6, 0.4, -1.2], "toe": _v(rest_inv * ((body._fk_leg(i, -0.6, 0.4, -1.2)[2] as Transform3D) * body._toe_off_c[i]))},
			],
		})
	# The brain's u -> joint target, for the robot's actuation path (pi_host Actuation.hpp).
	# Constants read off the body, and u_check computed by the body's OWN mapping function
	# with the brain path's clamp and splay sign applied, so the port is checked against
	# this code's numbers.  Samples cover both knee branches and both rails.
	var u_check := []
	for i in range(4):
		for u in [[0.0, 0.0, 0.0], [1.0, -1.0, 1.0], [-1.0, 1.0, -1.0], [0.37, -0.52, 0.81],
				  [-0.25, 0.6, -0.4], [1.7, -2.0, 1.3]]:
			var u1: float = clamp(float(u[0]), -1.0, 1.0) * float(body.HIP1_SPLAY_OUT_SIGN[i])
			var u2: float = clamp(float(u[1]), -1.0, 1.0)
			var u3: float = clamp(float(u[2]), -1.0, 1.0)
			u_check.append({"leg": i, "u": u, "t": body._discrete_joint_targets(u1, u2, u3)})
	var action_map := {
		"backend": body.actuation_backend,
		"knee_widening": body.knee_widening_enabled,
		"hip1_range": body.HIP1_TARGET_RANGE, "hip2_range": body.HIP_TARGET_RANGE,
		"knee_fold": body.KNEE_RANGE_FOLD, "knee_hyperext": body.KNEE_RANGE_HYPEREXT,
		"knee_symmetric": body.KNEE_RANGE_SYMMETRIC,
		"splay_out_sign": body.HIP1_SPLAY_OUT_SIGN,
		"u_check": u_check,
	}
	var out := {
		"_comment": "Exported by godot_host/project/scripts_tools/export_body_calib.gd. FK anchors for ogma::body::fk_leg, in the sim's leg order (fl, fr, rl, rr — the sim's names, which are MIRRORED from the physical legs: sim fl = physical FR). Do not hand-edit: re-export when the body JSON changes.",
		"geometry": body._geometry_name,
		"l1": body.L1, "l2": body.L2, "l3": body.L3,
		"hip1_rest": body.HIP1_REST, "hip2_rest": body.HIP2_REST, "knee_rest": body.KNEE_REST,
		"hip1_limit": body.HIP1_LIMIT, "hip2_limit": body.HIP2_LIMIT,
		"total_mass_kg": body._TOTAL_MASS,
		"chassis_rest_origin": _v(rest.origin),
		"chassis_rest_basis": [_v(rest.basis.x), _v(rest.basis.y), _v(rest.basis.z)],
		"legs": legs,
		"action_map": action_map,
	}
	var path := OS.get_environment("OUT")
	if path == "":
		push_error("export_body_calib: set OUT=<absolute path>")
		quit(2)
		return true
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_string(JSON.stringify(out, "  ", false, true))
	f.close()
	print("export_body_calib: wrote %s (geometry '%s')" % [path, body._geometry_name])
	quit(0)
	return true
