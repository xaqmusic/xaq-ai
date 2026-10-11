# Emits the GDScript attitude filter's inputs AND outputs as float32 BIT PATTERNS, so
# the C++ port (cpp_core/include/ogma/body/ImuAttitude.hpp) can be checked for
# byte-identity rather than for "close enough".
#
# The filter block below is copied VERBATIM from picrawler_body.gd::_imu_substep — the
# oracle has to be the real expressions, because the whole question is rounding.  Only
# the sim-only half (world-velocity differentiation, DLPF, 4 g clip) is replaced by a
# synthetic input sequence, since that half is not being ported.
#
#   godot4 --headless --path godot_host/project -s res://scripts_tools/imu_attitude_parity.gd
#   -> writes /tmp/imu_parity.txt
extends SceneTree

const IMU_ACC_TRUST: float = 0.02
const IMU_ACC_GATE_FRAC: float = 0.5
const N: int = 800

func f32hex(v: float) -> String:
	var b := PackedFloat32Array([v]).to_byte_array()
	return "%02x%02x%02x%02x" % [b[3], b[2], b[1], b[0]]

func f64hex(v: float) -> String:
	# ⚠ dt must round-trip as a DOUBLE.  GDScript computed 1.0/240.0 in 64-bit and used
	# that value; emitting it as float32 would hand the C++ a different number to replay.
	var b := PackedFloat64Array([v]).to_byte_array()
	return "%02x%02x%02x%02x%02x%02x%02x%02x" % [b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]]

func v3hex(v: Vector3) -> String:
	return "%s %s %s" % [f32hex(v.x), f32hex(v.y), f32hex(v.z)]

func _init() -> void:
	var f := FileAccess.open("/tmp/imu_parity.txt", FileAccess.WRITE)
	var up_est := Vector3.ZERO
	var up_acc := Vector3.ZERO
	for i in range(N):
		# Deterministic, and deliberately nasty: tilts through 40°, spins the gyro hard
		# enough to exercise the rotation, and swings |a| across the trust gate's edge so
		# the clamp boundary is hit from both sides.
		var t: float = float(i) * 0.013
		var accel := Vector3(3.1 * sin(t * 1.7), 9.81 + 4.0 * sin(t * 0.9), 2.7 * cos(t * 2.3))
		var gyro := Vector3(1.3 * sin(t * 0.7), 2.1 * cos(t * 1.1), 0.9 * sin(t * 1.9))
		var dt: float = 1.0 / 240.0

		# ---- verbatim from _imu_substep ------------------------------------------------
		var accel_meas: Vector3 = accel
		up_acc = accel_meas.normalized() if accel_meas.length() > 1e-4 else up_acc
		if up_est.length() < 0.5:
			up_est = up_acc
		var w_mag: float = gyro.length()
		if w_mag > 1e-6:
			up_est = (Basis(gyro / w_mag, -w_mag * dt) * up_est).normalized()
		var acc_dev: float = absf(accel_meas.length() - 9.81) / 9.81
		var trust: float = IMU_ACC_TRUST * clampf(1.0 - acc_dev / IMU_ACC_GATE_FRAC, 0.0, 1.0)
		if trust > 0.0:
			up_est = (up_est * (1.0 - trust) + up_acc * trust).normalized()
		# --------------------------------------------------------------------------------

		f.store_line("%s | %s | %s | %s | %s | %s" % [
			v3hex(accel), v3hex(gyro), f64hex(dt),
			v3hex(up_est), v3hex(up_acc), f32hex(trust)])
	f.close()
	print("wrote /tmp/imu_parity.txt  (", N, " steps)")
	quit(0)
