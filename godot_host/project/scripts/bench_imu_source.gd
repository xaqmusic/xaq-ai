extends Node
## Adapts ogma_benchd's `imu` telemetry block into the shape imu_scope.gd already reads.
##
## The scope was written as a HARDWARE BRING-UP REFERENCE (see its header) and the sim
## body satisfies it through get_imu_debug().  Rather than fork the panel for the bench,
## this node presents the same method over real telemetry, so the instrument the operator
## learned in sim is literally the same instrument on the robot.
##
## ⚠ UNITS ARE CONVERTED HERE, AND THAT IS THE WHOLE JOB.  benchd publishes what the part
## reports — accel in g, gyro in deg/s — because the record should carry the sensor's own
## units.  imu_scope inherits the SIM body's units: accel in m/s² (it divides by 9.81 for
## the |a| bar) and gyro in rad/s (it calls rad_to_deg for the trace).  Feeding g and
## deg/s straight in draws a plausible, wrong panel: |a| would read 0.1 g at rest and the
## gyro trace would sit flat at ±360°/s full scale.  Convert at the boundary, once.
##
## ⚠ NO GROUND-TRUTH KEYS ARE PUBLISHED, deliberately.  The robot has no `up_exact`, so
## `err_fused_deg` / `err_accel_deg` are absent rather than zero — imu_scope tests for the
## key and labels the row n/a instead of drawing a confident 0.0°.

const G: float = 9.81

var _d: Dictionary = {}          # the latest benchd `imu` block, or empty
var _fresh: bool = false

## Feed one telemetry frame.  Pass the WHOLE frame; the block may be null when the part
## is absent, which must read as "no instrument", not as zeros.
func set_frame(tele: Dictionary) -> void:
	var blk = tele.get("imu", null)
	if blk == null or not (blk is Dictionary) or not bool(blk.get("ok", false)):
		_fresh = false
		return
	_d = blk
	_fresh = true

func has_data() -> bool:
	return _fresh

func _vec(key: String) -> Vector3:
	var a = _d.get(key, null)
	if a == null or not (a is Array) or a.size() < 3:
		return Vector3.ZERO
	return Vector3(float(a[0]), float(a[1]), float(a[2]))

func get_imu_debug() -> Dictionary:
	if not _fresh:
		return {}
	var dt: float = float(_d.get("dt_s", 0.0))
	return {
		# already unit-vectors in the sim body frame (benchd applies the sec 4.1 remap)
		"up_fused": _vec("up_fused"),
		"up_accel": _vec("up_accel"),
		"accel":    _vec("accel_body") * G,                  # g -> m/s²
		"gyro":     _vec("gyro_body") * (PI / 180.0),        # deg/s -> rad/s
		"trust":    float(_d.get("trust", 0.0)),
		"disagree_deg": float(_d.get("disagree_deg", 0.0)),
		"imu_hz":   (1.0 / dt) if dt > 1e-6 else 0.0,
		# extras the bench has and the sim does not — the scope ignores unknown keys, and
		# the bench panel prints them beside it.
		"temp_c":       float(_d.get("temp_c", 0.0)),
		"bias_valid":   bool(_d.get("bias_valid", false)),
		"bias_samples": int(_d.get("bias_samples", 0)),
		"gyro_bias_dps": _vec("gyro_bias_dps"),
		"a_norm_g":     float(_d.get("a_norm_g", 0.0)),
		"errors":       int(_d.get("errors", 0)),
	}
