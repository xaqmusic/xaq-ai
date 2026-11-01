extends RefCounted
## THE BENCH'S TWO FLOORS (2026-11-01, operator: "two translucent horizontal grids so we can easily
## plot voxels in space … one for the belly ToF with its offset so we always know where it is
## sensing the real floor … a second fixed to the bottom of the chassis; no other geometry").
##
## Both grids are drawn in WORLD space (top_level) from the twin's chassis each frame:
##   CHASSIS — blue, translucent, at the chassis bottom plane (`_chassis_bottom_local` below the chassis
##             origin), yaw-levelled with the body: the belly plane the toes and the brain's
##             support plane are measured against.
##   FLOOR   — a translucent brown PLANE (operator, 2026-11-01), the real floor as the belly ToF
##             reports it: the chassis plane dropped by the clearance (telemetry's `tof.m`, already
##             raw − mount offset), with a ring where the boom's beam meets it (boom_z aft of the
##             origin) and the beam drawn from the belly down to it, in the belly row's yellow.
##             Hidden when the ToF is not reading, never frozen at a stale value.
## The grid's lines are every voxel (2 cm), heavier every 10 cm, ± half_extent.  Pure drawing.

var chassis_mi: MeshInstance3D = null
var floor_mi: MeshInstance3D = null
var half_extent: float = 0.40
var step: float = 0.02
# Operator, 2026-11-01: the ToF's floor in the belly row's own full-opacity yellow, the chassis
# (belly) plane in a darker blue, more transparent, so the sensed floor is the louder of the two.
var chassis_color := Color(0.25, 0.42, 0.95, 0.16)
var floor_color := Color(0.46, 0.31, 0.17, 0.55)       # the plane: translucent brown
var beam_color := Color(0.95, 0.80, 0.25, 1.0)        # the ToF's beam and footprint: the belly row's yellow
var floor_plane: MeshInstance3D = null
var _mat: StandardMaterial3D
var _chassis_mesh: ImmediateMesh
var _floor_mesh: ImmediateMesh
var _visible := true


func _init(parent: Node) -> void:
	_mat = StandardMaterial3D.new()
	_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_mat.vertex_color_use_as_albedo = true
	_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	_chassis_mesh = ImmediateMesh.new()
	_floor_mesh = ImmediateMesh.new()
	chassis_mi = MeshInstance3D.new(); chassis_mi.name = "BenchGridChassis"; chassis_mi.mesh = _chassis_mesh
	floor_mi = MeshInstance3D.new(); floor_mi.name = "BenchFloorBeam"; floor_mi.mesh = _floor_mesh
	var pm := PlaneMesh.new(); pm.size = Vector2(2.0 * half_extent, 2.0 * half_extent)
	# Shaded, so the body's shadow lands on it (an unshaded surface receives no shadow); the depth
	# pre-pass keeps a translucent surface lit and shadowed the way an opaque one is.
	var pmat := StandardMaterial3D.new()
	pmat.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	pmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_DEPTH_PRE_PASS
	pmat.cull_mode = BaseMaterial3D.CULL_DISABLED
	pmat.roughness = 1.0
	pmat.albedo_color = floor_color
	pm.material = pmat
	floor_plane = MeshInstance3D.new(); floor_plane.name = "BenchFloorPlane"; floor_plane.mesh = pm
	for mi in [chassis_mi, floor_mi, floor_plane]:
		mi.top_level = true
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		parent.add_child(mi)
	_build_grid(_chassis_mesh, chassis_color, Vector3.ZERO, false)


## A square grid of lines in the mesh's own frame (x right/left, z fore/aft), at y = 0, plus an
## optional beam marker: a ring at `beam` (x, z) and the beam itself from `beam_from_y` down to 0.
func _build_grid(mesh: ImmediateMesh, col: Color, beam: Vector3, with_beam: bool, beam_from_y: float = 0.0, lines: bool = true, beam_start: Vector3 = Vector3.INF) -> void:
	mesh.clear_surfaces()
	mesh.surface_begin(Mesh.PRIMITIVE_LINES, _mat)
	var n := int(round(half_extent / step)) if lines else -1
	for i in range(-n, n + 1):
		var a: float = float(i) * step
		var heavy: bool = (i % 5) == 0
		var c := Color(col.r, col.g, col.b, minf(1.0, col.a * (1.9 if heavy else 1.0) * (1.4 if i == 0 else 1.0)))
		mesh.surface_set_color(c); mesh.surface_add_vertex(Vector3(a, 0.0, -half_extent))
		mesh.surface_set_color(c); mesh.surface_add_vertex(Vector3(a, 0.0, half_extent))
		mesh.surface_set_color(c); mesh.surface_add_vertex(Vector3(-half_extent, 0.0, a))
		mesh.surface_set_color(c); mesh.surface_add_vertex(Vector3(half_extent, 0.0, a))
	if with_beam:
		var bc := Color(col.r, col.g, col.b, minf(1.0, col.a * 3.0))
		var r := 0.012
		for k in range(24):
			var a0: float = TAU * float(k) / 24.0; var a1: float = TAU * float(k + 1) / 24.0
			mesh.surface_set_color(bc); mesh.surface_add_vertex(beam + Vector3(r * cos(a0), 0.0005, r * sin(a0)))
			mesh.surface_set_color(bc); mesh.surface_add_vertex(beam + Vector3(r * cos(a1), 0.0005, r * sin(a1)))
		var from: Vector3 = beam_start if beam_start.is_finite() else Vector3(beam.x, beam_from_y, beam.z)
		mesh.surface_set_color(bc); mesh.surface_add_vertex(from)
		mesh.surface_set_color(bc); mesh.surface_add_vertex(Vector3(beam.x, 0.0, beam.z))
	mesh.surface_end()


## Per frame.  `chassis_xf`: the twin's chassis transform (tilted as the robot is); `bottom_dy`: the
## chassis bottom below its origin (negative); `clearance_m`: the belly ToF's clearance along the
## boom's axis (< 0 or NAN = not reading); `boom_z`: the boom's position along the body's forward axis.
## The CHASSIS grid rides the chassis, tilt and all.  The FLOOR plane is LEVEL (gravity) through the
## point where the beam, cast down the body's own axis from the boom, meets the floor.
func update(chassis_xf: Transform3D, bottom_dy: float, clearance_m: float, boom_z: float) -> void:
	if not _visible: return
	var belly_xf := Transform3D(chassis_xf.basis, chassis_xf.origin + chassis_xf.basis * Vector3(0.0, bottom_dy, 0.0))
	chassis_mi.global_transform = belly_xf
	var ok := is_finite(clearance_m) and clearance_m >= 0.0 and clearance_m < 0.5
	floor_mi.visible = ok; floor_plane.visible = ok
	if ok:
		var boom_w: Vector3 = belly_xf * Vector3(0.0, 0.0, boom_z)                   # the beam's start, on the belly plane
		var hit_w: Vector3 = boom_w + chassis_xf.basis * Vector3(0.0, -clearance_m, 0.0)   # down the body's axis
		var level := _level_basis(chassis_xf.basis)
		var fxf := Transform3D(level, hit_w)
		var start_local: Vector3 = fxf.affine_inverse() * boom_w
		_build_grid(_floor_mesh, beam_color, Vector3.ZERO, true, 0.0, false, start_local)
		floor_mi.global_transform = fxf; floor_plane.global_transform = fxf


static func _level_basis(b: Basis) -> Basis:
	var f: Vector3 = b.z; f.y = 0.0
	if f.length() < 1e-6: f = Vector3(0, 0, 1)
	f = f.normalized()
	return Basis(Vector3.UP.cross(f).normalized(), Vector3.UP, f)


func set_visible(v: bool) -> void:
	_visible = v
	chassis_mi.visible = v
	if not v: floor_mi.visible = false; floor_plane.visible = false
