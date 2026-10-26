# THE TERRAIN ARENA — gym "terrain" (2026-10-25).  A concentric calibration course for the
# gait-context vocabulary: which terrains does the walker need to treat differently?
#
# Slopes are not the interesting axis — with all four feet on a traversable slope the gait is
# the flat gait under a tilted gravity vector.  What changes footfalls is HEIGHT relative to
# swing clearance, SPACING relative to the stride, REGULARITY, and step DIRECTION (a step up
# lands early, a step down lands late with a missing support).  So:
#
#   centre      r < 2.0 m          flat (spawn)
#   ring k      5 bands, 1.5 m wide, separated by 1.0 m flat buffers; band heights
#               12 / 18 / 25 / 38 / 50 mm (plywood and lumber sizes, so the robot's floor
#               course can be built to match).  Each band is four 90 deg sectors:
#                 steps    two raised plateaus (up, down, up, down), vertical edges
#                 bars     seven tangential bars, trapezoid section, 0.20 m apart
#                 rough    overlapping pyramids up to the band height
#                 control  flat, or a gentle radial hump (4 deg ring 1, 8 deg ring 3) —
#                          the slope check: does the vocabulary split what the gait does not?
#   field       14.5..25 m: THE BOWL — challenge until failure.  The floor rises exponentially
#               (0 -> 1.5 m: ~4 deg at 20 m, ~24 deg at the rim) and the pyramids on it grow
#               exponentially (60 -> 600 mm tall, faces 15 -> 40 deg), ~90 % covered at every
#               radius.  The robot has climbed pyramids taller than its body; this finds where
#               it stops.
#   edge        a raised rim at the floor edge, floor 52 x 52 m
#
# HYBRID BUILD: steps and bars need vertical edges, so they are convex primitives (one body
# per sector, one convex prism per arc segment).  The rough sectors, the slope controls and
# the outer field are ONE procedural heightmap (HeightMapShape3D, 4 cm grid — the Cell world
# is the precedent), which is also the floor.  Everything comes from one seed:
# OGMA_PICRAWLER_TERRAIN_SEED (int, default 0; "seed" = use OGMA_SEED) shuffles which sector
# holds which type in each ring and rotates each ring, so varied seeds meet every type at
# every height in varied order — the corridor's fixed zone order let slow drift pass for
# terrain.  label_at(x, z) answers from the same spec that built the geometry, so the scorer
# never duplicates it.
#
# NOTE: picrawler_body.gd declares `const TAU = 0.02` (the tick constant), which shadows the
# maths TAU.  This file is separate, but uses 2.0 * PI throughout so it stays safe to paste.
extends RefCounted

const CENTRE_R: float = 2.0
const BAND: float = 1.5
const BUFFER: float = 1.0
const HEIGHTS: Array = [0.012, 0.018, 0.025, 0.038, 0.050]
const SLOPE_DEG: Array = [0.0, 4.0, 0.0, 8.0, 0.0]      # control sector: 0 = flat
const TYPES: Array = ["steps", "bars", "rough", "control"]
const FIELD_GAP: float = 1.0
const FIELD_R1: float = 25.0
const FIELD_H: Array = [0.060, 0.600]                   # pyramid height scale at field_r0 / FIELD_R1
const FIELD_ANG: Array = [15.0, 40.0]                   # pyramid face angle (deg), inner / outer
const FIELD_COVER: float = 2.3                          # pyramids per mean footprint (~90 % covered)
const BOWL_TOP: float = 1.5                             # floor elevation at FIELD_R1 (m)
const BOWL_L: float = 3.5                               # e-folding length of the bowl (m)
const RIM_W: float = 0.4                                # rim band at the floor edge (m)
const FLOOR_HALF: float = 26.0
const GRID: float = 0.04
const SEG_CHORD: float = 0.40                           # primitive arc segment length (m)

var seed_used: int = 0
var ring_offset: Array = []      # per ring: rotation of sector 0 (rad)
var ring_types: Array = []       # per ring: [type of sector 0..3]
var nx: int = 0
var heights: PackedFloat32Array = PackedFloat32Array()
var n_prisms: int = 0
var n_pyramids: int = 0

static func ring_r0(k: int) -> float:
	return CENTRE_R + float(k) * (BAND + BUFFER)

static func field_r0() -> float:
	return ring_r0(HEIGHTS.size() - 1) + BAND + FIELD_GAP

func _init(seed_v: int) -> void:
	seed_used = seed_v
	var rng := RandomNumberGenerator.new()
	rng.seed = hash(seed_v) & 0x7fffffff
	for _k in range(HEIGHTS.size()):
		var t: Array = TYPES.duplicate()
		for i in range(t.size() - 1, 0, -1):          # Fisher-Yates on the seeded RNG
			var j: int = rng.randi_range(0, i)
			var tmp = t[i]; t[i] = t[j]; t[j] = tmp
		ring_types.append(t)
		ring_offset.append(rng.randf() * 2.0 * PI)
	_rng = rng

var _rng: RandomNumberGenerator

# ---- labels -------------------------------------------------------------------------------

func _sector_of(k: int, x: float, z: float) -> int:
	var a: float = fposmod(atan2(z, x) - float(ring_offset[k]), 2.0 * PI)
	return clampi(int(a / (0.5 * PI)), 0, 3)

## [type, ring, height_mm] for a point.  Types: flat (centre and buffers), steps, bars, rough,
## slope, field, edge.  ring is -1 outside the bands; height is the band height (the hump's
## peak for slope, the local field height for field).
func label_at(x: float, z: float) -> Array:
	var r: float = Vector2(x, z).length()
	if r < CENTRE_R:
		return ["flat", -1, 0]
	for k in range(HEIGHTS.size()):
		var r0: float = ring_r0(k)
		if r >= r0 and r < r0 + BAND:
			var t: String = ring_types[k][_sector_of(k, x, z)]
			if t == "control":
				if float(SLOPE_DEG[k]) > 0.0:
					return ["slope", k, int(round(0.5 * BAND * tan(deg_to_rad(float(SLOPE_DEG[k]))) * 1000.0))]
				return ["flat", k, 0]
			return [t, k, int(round(float(HEIGHTS[k]) * 1000.0))]
		if r >= r0 + BAND and r < r0 + BAND + BUFFER:
			return ["flat", -1, 0]
	if r >= field_r0() and r < FIELD_R1:
		return ["field", -1, int(round(field_hmax(r) * 1000.0))]
	if r < field_r0():
		return ["flat", -1, 0]
	return ["edge", -1, 0]

func _field_t(r: float) -> float:
	return clampf((r - field_r0()) / (FIELD_R1 - field_r0()), 0.0, 1.0)

## Pyramid height scale at radius r: exponential from FIELD_H[0] to FIELD_H[1].
func field_hmax(r: float) -> float:
	return float(FIELD_H[0]) * pow(float(FIELD_H[1]) / float(FIELD_H[0]), _field_t(r))

## The bowl's floor elevation at radius r: 0 inside the field, exponential to BOWL_TOP at
## FIELD_R1, continuing to the floor edge (the corners are capped there).
static func bowl_at(r: float) -> float:
	var f0: float = field_r0()
	if r <= f0:
		return 0.0
	var b: float = BOWL_TOP / (exp((FIELD_R1 - f0) / BOWL_L) - 1.0)
	return b * (exp((minf(r, FLOOR_HALF) - f0) / BOWL_L) - 1.0)

# ---- heightmap ----------------------------------------------------------------------------

func _hidx(x: float, z: float) -> Vector2i:
	return Vector2i(int(round((x + FLOOR_HALF) / GRID)), int(round((z + FLOOR_HALF) / GRID)))

func _raise_pyramid(cx: float, cz: float, base: float, h: float, yaw: float) -> void:
	var half: float = base * 0.5
	var ext: float = half * 1.4143
	var lo := _hidx(cx - ext, cz - ext)
	var hi := _hidx(cx + ext, cz + ext)
	var c: float = cos(yaw)
	var s: float = sin(yaw)
	for j in range(maxi(lo.y, 0), mini(hi.y, nx - 1) + 1):
		var zz: float = float(j) * GRID - FLOOR_HALF - cz
		for i in range(maxi(lo.x, 0), mini(hi.x, nx - 1) + 1):
			var xx: float = float(i) * GRID - FLOOR_HALF - cx
			var u: float = absf(c * xx + s * zz)
			var v: float = absf(-s * xx + c * zz)
			var d: float = maxf(u, v)
			if d < half:
				var y: float = h * (1.0 - d / half)
				var idx: int = j * nx + i
				if y > heights[idx]:
					heights[idx] = y

func _in_sector(k: int, sec: int, x: float, z: float, margin_arc: float) -> bool:
	var r: float = Vector2(x, z).length()
	if r < 1e-6:
		return false
	var a: float = fposmod(atan2(z, x) - float(ring_offset[k]) - float(sec) * 0.5 * PI, 2.0 * PI)
	return a * r >= margin_arc and (0.5 * PI - a) * r >= margin_arc

func _build_heights() -> void:
	nx = int(round(2.0 * FLOOR_HALF / GRID)) + 1
	heights.resize(nx * nx)
	heights.fill(0.0)
	# Rough sectors: overlapping pyramids up to the band height, kept wholly inside the sector.
	for k in range(HEIGHTS.size()):
		var r0: float = ring_r0(k)
		var hk: float = HEIGHTS[k]
		for sec in range(4):
			var t: String = ring_types[k][sec]
			if t == "rough":
				var area: float = 0.25 * PI * (pow(r0 + BAND, 2) - r0 * r0)
				var n: int = int(area * 40.0)                  # ~85 % of the sector covered
				var placed: int = 0
				for _try in range(n * 3):
					if placed >= n:
						break
					var base: float = _rng.randf_range(0.12, 0.30)
					var rr: float = sqrt(_rng.randf_range(pow(r0 + base * 0.75, 2), pow(r0 + BAND - base * 0.75, 2)))
					var a: float = float(ring_offset[k]) + (float(sec) + _rng.randf()) * 0.5 * PI
					var px: float = rr * cos(a)
					var pz: float = rr * sin(a)
					if not _in_sector(k, sec, px, pz, base * 0.75):
						continue
					_raise_pyramid(px, pz, base, _rng.randf_range(0.4, 1.0) * hk, _rng.randf() * 0.5 * PI)
					placed += 1
				n_pyramids += placed
			elif t == "control" and float(SLOPE_DEG[k]) > 0.0:
				_raise_hump(k, sec, tan(deg_to_rad(float(SLOPE_DEG[k]))))
	# The bowl: pyramids growing exponentially outward, placed shell by shell so the coverage
	# stays ~FIELD_COVER footprints at every radius, then the bowl floor added under everything.
	var f0: float = field_r0()
	var r: float = f0
	while r < FIELD_R1:
		var r1: float = minf(r + 0.5, FIELD_R1)
		var hm: float = field_hmax(0.5 * (r + r1))
		var ang: float = deg_to_rad(lerpf(float(FIELD_ANG[0]), float(FIELD_ANG[1]), _field_t(0.5 * (r + r1))))
		var base_mean: float = 2.0 * 0.65 * hm / tan(ang)
		var n: int = int(PI * (r1 * r1 - r * r) * FIELD_COVER / (base_mean * base_mean))
		for _i in range(n):
			var rr: float = sqrt(_rng.randf_range(r * r, r1 * r1))
			var a: float = _rng.randf() * 2.0 * PI
			var h: float = _rng.randf_range(0.3, 1.0) * field_hmax(rr)
			var face: float = deg_to_rad(lerpf(float(FIELD_ANG[0]), float(FIELD_ANG[1]), _field_t(rr)) + _rng.randf_range(-5.0, 5.0))
			var base: float = 2.0 * h / tan(face)
			if rr - base * 0.75 < f0:
				continue                                 # keep the field's inner edge clean
			_raise_pyramid(rr * cos(a), rr * sin(a), base, h, _rng.randf() * 0.5 * PI)
			n_pyramids += 1
		r = r1
	var rim_h: float = bowl_at(FLOOR_HALF) + 0.5
	for j in range(nx):
		var z: float = float(j) * GRID - FLOOR_HALF
		for i in range(nx):
			var x: float = float(i) * GRID - FLOOR_HALF
			var idx: int = j * nx + i
			if FLOOR_HALF - maxf(absf(x), absf(z)) < RIM_W:
				heights[idx] = rim_h                     # containment rim
				continue
			var rr: float = sqrt(x * x + z * z)
			if rr > f0:
				heights[idx] += bowl_at(rr)

# A radial hump across the band (up over the first half, down over the second), tapered to
# zero over 0.6 m of arc at the sector's sides so there is no lateral cliff.
func _raise_hump(k: int, sec: int, slope: float) -> void:
	var r0: float = ring_r0(k)
	var lo := _hidx(-(r0 + BAND), -(r0 + BAND))
	var hi := _hidx(r0 + BAND, r0 + BAND)
	for j in range(maxi(lo.y, 0), mini(hi.y, nx - 1) + 1):
		var z: float = float(j) * GRID - FLOOR_HALF
		for i in range(maxi(lo.x, 0), mini(hi.x, nx - 1) + 1):
			var x: float = float(i) * GRID - FLOOR_HALF
			var r: float = Vector2(x, z).length()
			if r <= r0 or r >= r0 + BAND:
				continue
			var a: float = fposmod(atan2(z, x) - float(ring_offset[k]) - float(sec) * 0.5 * PI, 2.0 * PI)
			if a > 0.5 * PI:
				continue
			var edge: float = minf(a * r, (0.5 * PI - a) * r)
			var w: float = clampf(edge / 0.6, 0.0, 1.0)
			var y: float = slope * minf(r - r0, r0 + BAND - r) * w
			var idx: int = j * nx + i
			if y > heights[idx]:
				heights[idx] = y

# ---- primitives ---------------------------------------------------------------------------

# One annular-sector prism: radial section (ri0..ro0) at the bottom, (ri1..ro1) at height h,
# between angles a0..a1.  Returns its 8 corners (bottom 0-3, top 4-7).
static func _prism(ri0: float, ro0: float, ri1: float, ro1: float, h: float, a0: float, a1: float) -> PackedVector3Array:
	var c0: float = cos(a0); var s0: float = sin(a0)
	var c1: float = cos(a1); var s1: float = sin(a1)
	var yb: float = -0.005                              # sunk 5 mm: no seam for a toe to catch
	return PackedVector3Array([
		Vector3(ri0 * c0, yb, ri0 * s0), Vector3(ro0 * c0, yb, ro0 * s0),
		Vector3(ro0 * c1, yb, ro0 * s1), Vector3(ri0 * c1, yb, ri0 * s1),
		Vector3(ri1 * c0, h, ri1 * s0), Vector3(ro1 * c0, h, ro1 * s0),
		Vector3(ro1 * c1, h, ro1 * s1), Vector3(ri1 * c1, h, ri1 * s1)])

const _PRISM_FACES: Array = [[4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]

# Appends to the members _pv / _pn (not parameters: never rely on how packed arrays are passed).
var _pv := PackedVector3Array()
var _pn := PackedVector3Array()

func _add_prism_mesh(v: PackedVector3Array) -> void:
	var centre: Vector3 = Vector3.ZERO
	for p in v:
		centre += p
	centre /= 8.0
	for f in _PRISM_FACES:
		var p0: Vector3 = v[f[0]]; var p1: Vector3 = v[f[1]]; var p2: Vector3 = v[f[2]]; var p3: Vector3 = v[f[3]]
		var n: Vector3 = (p1 - p0).cross(p2 - p0).normalized()
		if n.dot((p0 + p2) * 0.5 - centre) < 0.0:
			n = -n                                  # outward; the materials draw both sides
		for q in [p0, p1, p2, p0, p2, p3]:
			_pv.append(q)
			_pn.append(n)

func _sector_body(k: int, sec: int, kind: String, contact_mat: PhysicsMaterial, layer: int, mask: int,
		mat: StandardMaterial3D) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.name = "Ring%d_%s" % [k, kind]
	body.collision_layer = layer
	body.collision_mask = mask
	body.physics_material_override = contact_mat
	var r0: float = ring_r0(k)
	var hk: float = HEIGHTS[k]
	var a_start: float = float(ring_offset[k]) + float(sec) * 0.5 * PI
	var radial: Array = []            # [ri0, ro0, ri1, ro1]
	if kind == "steps":
		for span in [[0.20, 0.70], [0.95, 1.45]]:
			radial.append([r0 + span[0], r0 + span[1], r0 + span[0], r0 + span[1]])
	else:   # bars: 0.05 m base, 0.02 m top
		for b in range(7):
			var rc: float = r0 + 0.10 + float(b) * 0.20
			radial.append([rc - 0.025, rc + 0.025, rc - 0.010, rc + 0.010])
	_pv = PackedVector3Array()
	_pn = PackedVector3Array()
	for rad in radial:
		var n_seg: int = maxi(2, int(ceil(float(rad[1]) * 0.5 * PI / SEG_CHORD)))
		for s in range(n_seg):
			var a0: float = a_start + 0.5 * PI * float(s) / float(n_seg)
			var a1: float = a_start + 0.5 * PI * float(s + 1) / float(n_seg)
			var v: PackedVector3Array = _prism(rad[0], rad[1], rad[2], rad[3], hk, a0, a1)
			var shape := ConvexPolygonShape3D.new()
			shape.points = v
			var cs := CollisionShape3D.new()
			cs.shape = shape
			body.add_child(cs)
			_add_prism_mesh(v)
			n_prisms += 1
	var arr: Array = []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = _pv
	arr[Mesh.ARRAY_NORMAL] = _pn
	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var mi := MeshInstance3D.new()
	mi.mesh = am
	mi.set_surface_override_material(0, mat)
	body.add_child(mi)
	return body

# ---- build --------------------------------------------------------------------------------

## Adds the whole course under `root`.  `visuals` false skips the heightmap mesh (headless).
func build(root: Node3D, contact_mat: PhysicsMaterial, _wedge_mat: PhysicsMaterial, layer: int, mask: int,
		visuals: bool) -> void:
	var t0: int = Time.get_ticks_msec()
	_build_heights()
	# Heightmap = the floor (and the rough sectors, slope controls, outer field).
	var floor_body := StaticBody3D.new()
	floor_body.name = "TerrainFloor"
	floor_body.set_meta("ogma_floor", true)       # instrument tag (load_log `fcol`); no physics effect
	floor_body.collision_layer = layer
	floor_body.collision_mask = mask
	floor_body.physics_material_override = contact_mat
	var hs := HeightMapShape3D.new()
	hs.map_width = nx
	hs.map_depth = nx
	hs.map_data = heights
	var hcs := CollisionShape3D.new()
	hcs.shape = hs
	hcs.scale = Vector3(GRID, 1.0, GRID)          # HeightMapShape3D: 1 unit per cell, centred
	floor_body.add_child(hcs)
	# A catch floor 3 cm below, in case a fast contact ever tunnels the heightmap surface.
	var ccs := CollisionShape3D.new()
	var cb := BoxShape3D.new()
	cb.size = Vector3(2.0 * FLOOR_HALF, 0.1, 2.0 * FLOOR_HALF)
	ccs.shape = cb
	ccs.position = Vector3(0, -0.08, 0)
	floor_body.add_child(ccs)
	root.add_child(floor_body)
	if visuals:
		_add_floor_visuals(floor_body)
	# Primitive sectors.
	var steps_mat := StandardMaterial3D.new()
	steps_mat.albedo_color = Color(0.62, 0.48, 0.30)
	steps_mat.roughness = 0.9
	steps_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	var bars_mat := StandardMaterial3D.new()
	bars_mat.albedo_color = Color(0.40, 0.48, 0.62)
	bars_mat.roughness = 0.8
	bars_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	for k in range(HEIGHTS.size()):
		for sec in range(4):
			var t: String = ring_types[k][sec]
			if t == "steps" or t == "bars":
				root.add_child(_sector_body(k, sec, t, contact_mat, layer, mask, steps_mat if t == "steps" else bars_mat))
	var layout: Array = []
	for k in range(HEIGHTS.size()):
		layout.append("%d mm @ %.0f deg: %s" % [int(round(float(HEIGHTS[k]) * 1000.0)), rad_to_deg(float(ring_offset[k])), ",".join(ring_types[k])])
	print("PicrawlerBody: [gym] TERRAIN seed %d: %d prisms, %d pyramids, heightmap %dx%d @ %.0f mm, built in %d ms" % [
		seed_used, n_prisms, n_pyramids, nx, nx, GRID * 1000.0, Time.get_ticks_msec() - t0])
	for k in range(layout.size()):
		print("PicrawlerBody: [gym] TERRAIN ring %d (r %.1f-%.1f m): %s" % [k, ring_r0(k), ring_r0(k) + BAND, layout[k]])

# Hypsometric ramp by ABSOLUTE height (operator 2026-10-26): the lowest ground is the flat floor's own dark grey, rising
# through olive, browns and tans; only the highest peaks of the bowl reach pale grey and white.  The bright colours are
# at the top, where there is least of them.  Thresholds (m) are compressed low so the inner course still shades.
const _RAMP_H: Array = [0.0, 0.03, 0.10, 0.30, 0.70, 1.20, 1.70, 2.25, 2.50]
const _RAMP_C: Array = [Color(0.30, 0.35, 0.30), Color(0.31, 0.32, 0.25), Color(0.34, 0.30, 0.22), Color(0.38, 0.31, 0.22),
	Color(0.45, 0.36, 0.26), Color(0.52, 0.44, 0.33), Color(0.60, 0.53, 0.43), Color(0.76, 0.74, 0.70), Color(0.95, 0.95, 0.95)]

static func _height_colour(h: float) -> Color:
	for i in range(1, _RAMP_H.size()):
		if h <= float(_RAMP_H[i]):
			return (_RAMP_C[i - 1] as Color).lerp(_RAMP_C[i], (h - float(_RAMP_H[i - 1])) / (float(_RAMP_H[i]) - float(_RAMP_H[i - 1])))
	return _RAMP_C[_RAMP_C.size() - 1]

# Flat floor plane plus indexed meshes of the raised heightmap: the inner course at the full
# 4 cm grid, the bowl at 8 cm (collision stays at 4 cm everywhere).  Coloured by absolute height
# (_RAMP_H / _RAMP_C), dark at the floor, white only at the highest peaks.
func _add_floor_visuals(parent: Node3D) -> void:
	var plane := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(2.0 * FLOOR_HALF, 2.0 * FLOOR_HALF)
	plane.mesh = pm
	plane.position = Vector3(0, -0.0005, 0)
	var fmat := StandardMaterial3D.new()
	fmat.albedo_color = Color(0.3, 0.35, 0.3, 1.0)
	plane.set_surface_override_material(0, fmat)
	parent.add_child(plane)
	var f0: float = field_r0()
	var inner_half: float = f0
	_grid_mesh(parent, 1, inner_half, f0 - 0.2, -1.0)
	_grid_mesh(parent, 2, FLOOR_HALF, 1e9, f0 - 0.2)

# One indexed mesh over the square |x|,|z| <= half at `stride` cells, drawing the cells whose
# centre radius is in [r_min, r_max) and that are raised (any corner above 0).
func _grid_mesh(parent: Node3D, stride: int, half: float, r_max: float, r_min: float) -> void:
	var i0: int = maxi(0, int(floor((FLOOR_HALF - half) / GRID)))
	var i1: int = mini(nx - 1, int(ceil((FLOOR_HALF + half) / GRID)))
	var m: int = (i1 - i0) / stride + 1
	var step: float = GRID * float(stride)
	var verts := PackedVector3Array(); verts.resize(m * m)
	var norms := PackedVector3Array(); norms.resize(m * m)
	var cols := PackedColorArray(); cols.resize(m * m)
	for b in range(m):
		var j: int = i0 + b * stride
		var z: float = float(j) * GRID - FLOOR_HALF
		for a in range(m):
			var i: int = i0 + a * stride
			var x: float = float(i) * GRID - FLOOR_HALF
			var h: float = heights[j * nx + i]
			var hl: float = heights[j * nx + maxi(i - stride, 0)]
			var hr: float = heights[j * nx + mini(i + stride, nx - 1)]
			var hd: float = heights[maxi(j - stride, 0) * nx + i]
			var hu: float = heights[mini(j + stride, nx - 1) * nx + i]
			var v: int = b * m + a
			verts[v] = Vector3(x, h, z)
			norms[v] = Vector3(hl - hr, 2.0 * step, hd - hu).normalized()
			cols[v] = _height_colour(h)
	var idx := PackedInt32Array()
	for b in range(m - 1):
		for a in range(m - 1):
			var v00: int = b * m + a
			var v10: int = v00 + 1
			var v01: int = v00 + m
			var v11: int = v01 + 1
			if verts[v00].y <= 0.0 and verts[v10].y <= 0.0 and verts[v01].y <= 0.0 and verts[v11].y <= 0.0:
				continue
			var cx: float = 0.5 * (verts[v00].x + verts[v11].x)
			var cz: float = 0.5 * (verts[v00].z + verts[v11].z)
			var rc: float = sqrt(cx * cx + cz * cz)
			if rc < r_min or rc >= r_max:
				continue
			idx.append_array([v00, v10, v11, v00, v11, v01])
	if idx.is_empty():
		return
	var arr: Array = []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = norms
	arr[Mesh.ARRAY_COLOR] = cols
	arr[Mesh.ARRAY_INDEX] = idx
	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	var mi := MeshInstance3D.new()
	mi.mesh = am
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.vertex_color_is_srgb = true     # the ramp is written in sRGB like every albedo here; read linear it washed out
	mat.roughness = 0.95
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mi.set_surface_override_material(0, mat)
	parent.add_child(mi)
