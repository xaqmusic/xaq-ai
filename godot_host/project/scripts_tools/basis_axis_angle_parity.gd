# Isolates ONE operation: Basis(axis, angle) * v, with float32 inputs emitted as bit
# patterns.  No accumulated state, no filter — if this matches, the port's arithmetic is
# right and any remaining drift is sequencing; if it does not, the difference is in the
# engine's trig or codegen and no amount of restructuring will fix it.
extends SceneTree

func f32hex(v: float) -> String:
	var b := PackedFloat32Array([v]).to_byte_array()
	return "%02x%02x%02x%02x" % [b[3], b[2], b[1], b[0]]

func _init() -> void:
	var f := FileAccess.open("/tmp/basis_parity.txt", FileAccess.WRITE)
	for i in range(500):
		var t: float = float(i) * 0.0137
		# Axis is normalized as a Vector3 (float32), exactly as the filter does it.
		var axis: Vector3 = Vector3(sin(t * 1.3), cos(t * 0.7), sin(t * 2.1)).normalized()
		var ang: float = -0.0131 * (1.0 + sin(t))          # radians, filter-sized
		var v: Vector3 = Vector3(cos(t * 0.5), 0.97, sin(t * 0.9)).normalized()
		var r: Vector3 = Basis(axis, ang) * v
		f.store_line("%s %s %s | %s | %s %s %s | %s %s %s" % [
			f32hex(axis.x), f32hex(axis.y), f32hex(axis.z), f32hex(ang),
			f32hex(v.x), f32hex(v.y), f32hex(v.z),
			f32hex(r.x), f32hex(r.y), f32hex(r.z)])
	f.close()
	print("wrote /tmp/basis_parity.txt")
	quit(0)
