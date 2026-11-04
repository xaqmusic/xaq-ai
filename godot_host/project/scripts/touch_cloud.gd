extends RefCounted
## (loaded by preload("res://scripts/touch_cloud.gd"), the repo's convention — no class_name, so
## headless runs need no global class cache)
## THE CLOUD OF TOUCHES — the GDScript twin of ogma::TouchMemory (cpp_core), and its drawing.
##
## Every rising foot-contact edge is one claim about where a surface is: the toe's commanded-FK
## position at that moment.  Hobby servos report nothing back, so each claim carries the
## kinematic error (~32 mm standing, ~50 mm walking, ledger 2026-10-24) and the odometry's
## drift.  A voxel is believable in proportion to the agreement it collects — hits, feet, time —
## which is why the store is evidential rather than a point list.
##
## FRAMES.  Touches are stored in the ODOM frame (DeadReckon: x_o = forward at reset, y_o = body
## +X at reset, yaw from x_o toward y_o) and re-expressed in the current BODY frame on demand
## (fwd, lat = body +X, h = height over the support plane).  A body that walks on leaves its
## footfalls behind; drawn body-locked, they slide as the odometry drifts — which is the point.
## With no odometry (the bench twin) the body frame IS the frame.
##
## HEIGHT is relative to the SUPPORT PLANE (the running median height of the toes that were
## already planted, never the landing foot), so a floor touch is h ~ 0 at any stance height and
## a toe landing on a box reads h > 0.  The sim draws two clouds: this one, always, PURPLE (the
## lower legs' contact colour); and the brain's TouchMemory voxels when the config carries the
## module, LAVENDER — two implementations of one idea on one body, so a disagreement is visible.

var voxel_m: float = 0.02
var max_voxels: int = 4000
var plane_tau_ticks: float = 50.0
## LIFETIME (operator, 2026-11-04): the odometry drifts, so a claim older than this is a claim about a place the
## body can no longer locate; it leaves.  1500 ticks = 30 s at 50 Hz (the bench, at its 10 Hz telemetry, sets 300).
## 0 = forever.
var max_age_ticks: int = 1500
var points: int = 0                # add_point() calls (the field's returns), beside `touches`

var touches: int = 0
var plane_y: float = 0.0
var have_plane: bool = false

var _vox: Dictionary = {}          # Vector3i -> {sx, sy, sh, hits, first, last, mask, load}
var _prev_contact: Array = [0.0, 0.0, 0.0, 0.0]
var _tick: int = 0


func clear() -> void:
	_vox.clear(); touches = 0; have_plane = false; plane_y = 0.0
	_prev_contact = [0.0, 0.0, 0.0, 0.0]


## Take the current contact flags as the baseline without recording touches: a body found
## standing on four loaded feet is a starting state, not four events.
func prime(contact: Array) -> void:
	for i in range(mini(4, contact.size())): _prev_contact[i] = float(contact[i])


## One tick.  contact [4] 0/1; feet_xz [8] per leg [x = body +X, z = forward] (chassis rest
## frame, m); feet_y [4] toe height along gravity (body origin 0); odom [x, y, yaw] or [] for
## none; load [4] optional.  Returns how many touches this tick added.
func step(tick: int, contact: Array, feet_xz: Array, feet_y: Array, odom: Array, load: Array = []) -> int:
	_tick = tick
	if (tick % 25) == 0: prune(tick)
	if contact.size() < 4: return 0
	var rising: Array = [false, false, false, false]
	for i in range(4):
		rising[i] = float(contact[i]) > 0.5 and float(_prev_contact[i]) <= 0.5
	var added := 0
	if feet_y.size() >= 4:
		var planted: Array = []
		for i in range(4):
			if float(contact[i]) > 0.5 and not rising[i]: planted.append(float(feet_y[i]))
		if planted.size() > 0:
			planted.sort()
			var n := planted.size()
			var med: float = planted[n / 2] if (n % 2) == 1 else 0.5 * (planted[n / 2 - 1] + planted[n / 2])
			if not have_plane: plane_y = med; have_plane = true
			else: plane_y += (med - plane_y) / plane_tau_ticks
	if feet_xz.size() >= 8 and feet_y.size() >= 4:
		for i in range(4):
			if not rising[i]: continue
			var h: float = float(feet_y[i]) - plane_y if have_plane else 0.0
			if not have_plane: plane_y = float(feet_y[i]); have_plane = true
			add_touch(i, float(feet_xz[2 * i + 1]), float(feet_xz[2 * i]), h, odom,
				float(load[i]) if load.size() >= 4 else 0.0, tick)
			added += 1
	for i in range(4): _prev_contact[i] = float(contact[i])
	return added


## A touch at body-frame (fwd, lat, h), carried into the odom frame by `odom` ([x, y, yaw] or []).
func add_touch(leg: int, fwd: float, lat: float, h: float, odom: Array, load: float, tick: int) -> void:
	var px := 0.0; var py := 0.0; var c := 1.0; var s := 0.0
	if odom.size() >= 3:
		px = float(odom[0]); py = float(odom[1]); c = cos(float(odom[2])); s = sin(float(odom[2]))
	var X: float = px + fwd * c - lat * s
	var Y: float = py + fwd * s + lat * c
	var key := Vector3i(int(floor(X / voxel_m)), int(floor(Y / voxel_m)), int(floor(h / voxel_m)))
	if not _vox.has(key):
		_evict_if_full()
		_vox[key] = {"sx": 0.0, "sy": 0.0, "sh": 0.0, "hits": 0, "first": tick, "last": tick, "mask": 0, "load": 0.0}
	var v: Dictionary = _vox[key]
	v["sx"] += X; v["sy"] += Y; v["sh"] += h; v["hits"] += 1; v["last"] = tick
	v["mask"] |= (1 << leg); v["load"] = maxf(float(v["load"]), load)
	touches += 1


## Drop every voxel not touched for max_age_ticks (0 = never).
func prune(tick: int) -> int:
	_tick = tick
	if max_age_ticks <= 0: return 0
	var gone: Array = []
	for k in _vox.keys():
		if tick - int(_vox[k]["last"]) > max_age_ticks: gone.append(k)
	for k in gone: _vox.erase(k)
	return gone.size()


## A generic point with a CLASS (the 8x8 field's returns: 1 floor, 2 a thing, 4 too close), at body-frame
## (fwd, lat, h) where h is the height in the level body frame (origin 0), carried into the odom frame.
func add_point(cls: int, fwd: float, lat: float, h: float, odom: Array, tick: int) -> void:
	var px := 0.0; var py := 0.0; var c := 1.0; var s := 0.0
	if odom.size() >= 3:
		px = float(odom[0]); py = float(odom[1]); c = cos(float(odom[2])); s = sin(float(odom[2]))
	var X: float = px + fwd * c - lat * s
	var Y: float = py + fwd * s + lat * c
	var key := Vector3i(int(floor(X / voxel_m)), int(floor(Y / voxel_m)), int(floor(h / voxel_m)))
	if not _vox.has(key):
		_evict_if_full()
		_vox[key] = {"sx": 0.0, "sy": 0.0, "sh": 0.0, "hits": 0, "first": tick, "last": tick, "mask": 0, "load": 0.0, "cls": cls}
	var v: Dictionary = _vox[key]
	v["sx"] += X; v["sy"] += Y; v["sh"] += h; v["hits"] += 1; v["last"] = tick
	v["cls"] = cls   # the latest reading's class wins (a floor that became a thing is a thing)
	points += 1


func _evict_if_full() -> void:
	while _vox.size() >= max_voxels:
		var oldest_k = null; var oldest_t := 1 << 62
		for k in _vox.keys():
			if int(_vox[k]["last"]) < oldest_t: oldest_t = int(_vox[k]["last"]); oldest_k = k
		if oldest_k == null: return
		_vox.erase(oldest_k)


## The voxels in the current body frame, nearest first: Array of
## [fwd, lat, h, hits, mask, age_ticks, load] — the same row TouchMemory's diag lists.
func body_voxels(odom: Array, n: int = 0) -> Array:
	var px := 0.0; var py := 0.0; var c := 1.0; var s := 0.0
	if odom.size() >= 3:
		px = float(odom[0]); py = float(odom[1]); c = cos(float(odom[2])); s = sin(float(odom[2]))
	var out: Array = []
	for k in _vox.keys():
		var v: Dictionary = _vox[k]
		var hits: int = int(v["hits"])
		var dx: float = float(v["sx"]) / hits - px
		var dy: float = float(v["sy"]) / hits - py
		out.append([dx * c + dy * s, -dx * s + dy * c, float(v["sh"]) / hits, hits, int(v["mask"]),
			maxi(0, _tick - int(v["last"])), float(v["load"]), int(v.get("cls", 0))])
	out.sort_custom(func(a, b): return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2])
	if n > 0 and out.size() > n: out.resize(n)
	return out


func count() -> int:
	return _vox.size()


## The yaw-levelled body frame at the chassis origin: x = body +X (left), y = up, z = forward.
## Heights in the cloud are along gravity, so the frame must not pitch or roll with the body.
static func level_frame(chassis_xf: Transform3D) -> Transform3D:
	var f: Vector3 = chassis_xf.basis.z; f.y = 0.0
	if f.length() < 1e-6: f = Vector3(0, 0, 1)
	f = f.normalized()
	var l: Vector3 = Vector3.UP.cross(f).normalized()
	return Transform3D(Basis(l, Vector3.UP, f), chassis_xf.origin)


# ---------------------------------------------------------------------------------------------
## The drawing: one MultiMesh of small cubes, body-locked by the transform handed to show().
## Brightness follows the hit count (one touch dim, many touches bright), so agreement reads.
class View extends RefCounted:
	var mmi: MultiMeshInstance3D = null
	var color: Color
	var size: float
	var _mat: StandardMaterial3D
	var class_colors: Dictionary = {}     # class (row[7]) -> Color; empty = one colour
	var max_age_ticks: int = 0            # > 0: a voxel dims over its lifetime (row[5] = age)

	func _init(parent: Node, col: Color, edge: float, nm: String = "TouchCloud") -> void:
		color = col; size = edge
		_mat = StandardMaterial3D.new()
		_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		_mat.vertex_color_use_as_albedo = true
		_mat.no_depth_test = false
		var mesh := BoxMesh.new(); mesh.size = Vector3(edge, edge, edge); mesh.material = _mat
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_colors = true
		mm.mesh = mesh
		mm.instance_count = 0
		mmi = MultiMeshInstance3D.new(); mmi.name = nm; mmi.multimesh = mm
		mmi.top_level = true       # transforms below are WORLD-space; ignore the parent's motion
		mmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		parent.add_child(mmi)

	## items: rows [fwd, lat, h, hits, ...]; base: the level body frame (TouchCloud.level_frame);
	## plane_y: where the support plane sits below the body origin (the brain's plane_y, or 0 when
	## the rows already carry absolute body-frame heights).
	func show(items: Array, base: Transform3D, plane_y: float) -> void:
		if mmi == null: return
		var mm := mmi.multimesh
		var n := items.size()
		if mm.instance_count != n: mm.instance_count = n
		for i in range(n):
			var r: Array = items[i]
			var local := Vector3(float(r[1]), plane_y + float(r[2]), float(r[0]))
			mm.set_instance_transform(i, Transform3D(Basis(), base * local))
			var hits := int(r[3]) if r.size() > 3 else 1
			var a: float = clampf(0.45 + 0.14 * float(hits), 0.45, 1.0)
			if max_age_ticks > 0 and r.size() > 5:
				a *= 1.0 - 0.6 * clampf(float(r[5]) / float(max_age_ticks), 0.0, 1.0)
			var col: Color = color
			if not class_colors.is_empty() and r.size() > 7 and class_colors.has(int(r[7])):
				col = class_colors[int(r[7])]
			mm.set_instance_color(i, Color(col.r, col.g, col.b, a))

	func set_visible(v: bool) -> void:
		if mmi != null: mmi.visible = v

	func free_node() -> void:
		if mmi != null and is_instance_valid(mmi): mmi.queue_free()
		mmi = null
