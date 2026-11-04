# The forward 8x8 time-of-flight field (a VL53L5CX / VL53L8CX class part) on the picrawler, 2026-11-04.
#
# 64 zones in a 45 deg square field (zone pitch 5.625 deg), slant range per zone to 4 m, cast at 15 Hz from a mount on
# the chassis's front face whose HEIGHT above the chassis bottom and downward PITCH are live-adjustable (env, hotkeys),
# so the mount can be chosen in the sim and then drilled on the robot.  The duck's model (mj_host/src/Tof.hpp) is the
# precedent: one ray per zone against the world; here the body's own legs are cast too, as an instrument, because on
# this body the front legs swing through the field and the mount must be judged by how much of it they take.
#
# What it publishes (sense.tof8, float32[64], row-major, row 0 = the TOP of the field, column 0 = the LEFT as the robot
# sees it): the slant range in metres, -1 = no return (beyond range).  A leg in the beam is a return at the leg (the
# real part sees it too).  The classification (floor / hit / self / too close / none) is for the rays and the log only
# and is egocentric: the floor is where a beam is expected to meet the ground, from the belly clearance and the body's
# own gravity estimate (the duck's 85 % rule).
#
# Frames: sensor forward = the chassis +Z, right = +X, up = +Y; the pitch turns the field DOWN about the chassis X.

const ROWS: int = 8
const COLS: int = 8
const ZONES: int = 64
const FOV_DEG: float = 45.0
const MAX_RANGE_M: float = 4.0
const MIN_RANGE_M: float = 0.02
const TOO_CLOSE_M: float = 0.10
const FLOOR_SAFETY: float = 0.85       # a downward beam whose return reaches 85 % of the way to the ground is Floor
const HZ: float = 15.0

enum Cls { NONE = 0, FLOOR = 1, HIT = 2, SELF = 3, TOO_CLOSE = 4 }

class Model extends RefCounted:
	var height_above_bottom_m: float = 0.070   # the mount above the chassis bottom (the electronics box top is ~0.10)
	var pitch_deg: float = 30.0                # downward tilt of the field's centre
	var fwd_m: float = 0.0                     # ahead of the front face (a bracket)
	var lateral_m: float = 0.0                 # to the chassis +X (the robot's left) of the face's centre
	var self_occlusion: bool = true            # cast the body's own legs too
	var ranges: PackedFloat64Array = PackedFloat64Array()
	var classes: PackedInt32Array = PackedInt32Array()
	var points: Array = []                     # world hit points (Vector3) or null, per zone
	var origin: Vector3 = Vector3.ZERO         # the sensor, world
	var dirs: Array = []                       # world beam directions, per zone
	var casts: int = 0
	var _beams: Array = []                     # sensor-frame beams at pitch 0

	func _init() -> void:
		ranges.resize(ZONES); classes.resize(ZONES); points.resize(ZONES); dirs.resize(ZONES)
		for z in range(ZONES):
			ranges[z] = -1.0; classes[z] = Cls.NONE; points[z] = null; dirs[z] = Vector3.FORWARD
		var pitch_z: float = FOV_DEG / float(COLS)
		for r in range(ROWS):
			for c in range(COLS):
				var az: float = deg_to_rad((float(c) - 3.5) * pitch_z)   # left negative
				var el: float = deg_to_rad((3.5 - float(r)) * pitch_z)   # row 0 = up
				_beams.append(Vector3(cos(el) * sin(az), sin(el), cos(el) * cos(az)).normalized())

	# The sensor's local transform on the chassis: at the front face's centre, height above the bottom, pitched down.
	func local_origin(chassis_bottom_local: float, chassis_z: float) -> Vector3:
		return Vector3(lateral_m, chassis_bottom_local + height_above_bottom_m, chassis_z * 0.5 + fwd_m)

	func local_dir(zone: int) -> Vector3:
		var b: Vector3 = _beams[zone]
		var p: float = deg_to_rad(pitch_deg)
		# pitch down about +X: y' = y cos p - z sin p ... (forward tilts toward -Y)
		return Vector3(b.x, b.y * cos(p) - b.z * sin(p), b.y * sin(p) + b.z * cos(p)).normalized()

	# Cast all 64 zones.  up_world = the body's OWN gravity estimate expressed in the world (for the floor rule);
	# belly_m = the belly clearance the body reads (its ToF), so the sensor's height above the floor is egocentric.
	func cast(space: PhysicsDirectSpaceState3D, xf: Transform3D, chassis_bottom_local: float, chassis_z: float,
			  chassis_rid: RID, layer_world: int, layer_body: int, up_world: Vector3, belly_m: float) -> void:
		casts += 1
		origin = xf * local_origin(chassis_bottom_local, chassis_z)
		var h_sensor: float = maxf(0.0, belly_m) + height_above_bottom_m
		for z in range(ZONES):
			var d: Vector3 = (xf.basis * local_dir(z)).normalized()
			dirs[z] = d
			var q := PhysicsRayQueryParameters3D.create(origin, origin + d * MAX_RANGE_M, layer_world)
			var hit: Dictionary = space.intersect_ray(q)
			var r: float = -1.0
			var cls: int = Cls.NONE
			var pt = null
			if not hit.is_empty():
				r = origin.distance_to(hit.position); pt = hit.position
			if self_occlusion:
				var qs := PhysicsRayQueryParameters3D.create(origin, origin + d * MAX_RANGE_M, layer_body)
				qs.exclude = [chassis_rid]
				var hs: Dictionary = space.intersect_ray(qs)
				if not hs.is_empty():
					var rs: float = origin.distance_to(hs.position)
					if r < 0.0 or rs < r:
						r = rs; pt = hs.position; cls = Cls.SELF
			if r >= 0.0:
				r = maxf(r, MIN_RANGE_M)
				if cls != Cls.SELF:
					if r < TOO_CLOSE_M:
						cls = Cls.TOO_CLOSE
					else:
						var down: float = -d.dot(up_world)
						var floor_dist: float = h_sensor / down if down > 0.05 else INF
						cls = Cls.FLOOR if r >= FLOOR_SAFETY * floor_dist else Cls.HIT
			ranges[z] = r; classes[z] = cls; points[z] = pt

	func count(cls: int) -> int:
		var n: int = 0
		for z in range(ZONES):
			if classes[z] == cls: n += 1
		return n

	func min_range(cls_filter: int = -1) -> float:
		var m: float = INF
		for z in range(ZONES):
			if ranges[z] >= 0.0 and (cls_filter < 0 or classes[z] == cls_filter): m = minf(m, ranges[z])
		return m

	# The field as text: 8 rows of 8, decimetres (two digits), '--' none, 'LL' a leg, for the HUD and prints.
	func text_grid() -> String:
		var s: String = ""
		for r in range(ROWS):
			var row: String = ""
			for c in range(COLS):
				var z: int = r * COLS + c
				if ranges[z] < 0.0: row += " --"
				elif classes[z] == Cls.SELF: row += " LL"
				else: row += " %2d" % clampi(int(round(ranges[z] * 10.0)), 0, 99)
			s += row + ("\n" if r < ROWS - 1 else "")
		return s

class View extends RefCounted:
	var mi: MeshInstance3D
	var _mesh: ImmediateMesh
	var _mat: StandardMaterial3D
	const COL_FLOOR := Color(0.30, 0.90, 0.45, 0.55)
	const COL_HIT := Color(1.00, 0.55, 0.10, 0.95)
	const COL_SELF := Color(0.95, 0.25, 0.95, 0.90)
	const COL_CLOSE := Color(1.00, 0.15, 0.15, 1.00)
	const COL_NONE := Color(0.70, 0.70, 0.75, 0.10)
	const NONE_DRAW_M: float = 0.8   # a ray that returned nothing is drawn this far, faintly

	func _init(parent: Node3D) -> void:
		_mat = StandardMaterial3D.new()
		_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		_mat.vertex_color_use_as_albedo = true
		_mat.no_depth_test = false
		_mesh = ImmediateMesh.new()
		mi = MeshInstance3D.new(); mi.name = "Tof8Rays"; mi.mesh = _mesh
		mi.top_level = true   # world-space vertices
		parent.add_child(mi)

	func update(m: Model) -> void:
		_mesh.clear_surfaces()
		_mesh.surface_begin(Mesh.PRIMITIVE_LINES, _mat)
		for z in range(ZONES):
			var col: Color
			var end: Vector3
			match m.classes[z]:
				Cls.FLOOR: col = COL_FLOOR
				Cls.HIT: col = COL_HIT
				Cls.SELF: col = COL_SELF
				Cls.TOO_CLOSE: col = COL_CLOSE
				_: col = COL_NONE
			if m.ranges[z] >= 0.0 and m.points[z] != null:
				end = m.points[z]
			else:
				end = m.origin + (m.dirs[z] as Vector3) * NONE_DRAW_M
			_mesh.surface_set_color(col); _mesh.surface_add_vertex(m.origin)
			_mesh.surface_set_color(col); _mesh.surface_add_vertex(end)
			# a short cross at each return
			if m.ranges[z] >= 0.0 and m.points[z] != null:
				var p: Vector3 = m.points[z]
				var s: float = 0.006
				_mesh.surface_set_color(col); _mesh.surface_add_vertex(p + Vector3(-s, 0, 0))
				_mesh.surface_set_color(col); _mesh.surface_add_vertex(p + Vector3(s, 0, 0))
				_mesh.surface_set_color(col); _mesh.surface_add_vertex(p + Vector3(0, 0, -s))
				_mesh.surface_set_color(col); _mesh.surface_add_vertex(p + Vector3(0, 0, s))
		_mesh.surface_end()

	func set_visible(v: bool) -> void:
		if mi != null: mi.visible = v
