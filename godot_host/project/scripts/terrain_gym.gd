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
#   slip        OPTIONAL (2026-10-26, operator: the robot walks off carpet onto slippery vinyl): with
#               OGMA_PICRAWLER_TERRAIN_SLIP=<mu> a flat low-friction annulus, a 2 mm sheet (top +2 mm), replaces
#               the rings it overlaps — by default r 2.0-4.5 m, ring 1 and its buffer, so the walker leaves the
#               grippy spawn straight onto it.  OGMA_PICRAWLER_TERRAIN_SLIP_R="r0,r1" moves it.  The other rings
#               keep their layouts (every spec is still drawn; only the covered ones are not built).
#               ⚠ GODOT'S FRICTION RULE (measured, slide test 2026-10-26): when either material is `rough` the
#               contact takes the rough one's friction (the higher, if both are rough); only two smooth materials
#               take the lower.  The feet were rough 1.5, so a low-friction floor could never be felt.  With the
#               sheet on, the body gives its leg segments SMOOTH materials (picrawler_body.gd), and every floor
#               stays rough: each contact then takes the floor's friction — 1.5 everywhere it was, mu on the sheet.
#
# SOLID BUILD (2026-10-26): every piece of ground is a THICK convex solid — a box floor, convex pyramids, the bowl and
# the slope humps as thick sloped cells (each the convex hull of its corner heights over a bottom far below), steps and
# bars as convex prisms, a box rim.  The first build was a HeightMapShape3D (4 cm grid): a zero-thickness surface a
# foot can cross in one physics step — leg capsules arriving at 3 m/s fell through 6 / 300 times (a box floor: 0 up to
# 5 m/s), and a stumble that does it ejects the leg violently (operator saw the front legs fall through into a physics
# explosion at the bowl's edge).  The visuals are drawn FROM the same solids, so what is seen is what the feet touch.
# Everything comes from one seed:
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
const SEG_CHORD: float = 0.40                           # primitive arc segment length (m)

var seed_used: int = 0
const SLIP_H: float = 0.002                             # the sheet's thickness above the floor (m)
var slip_mu: float = 0.0                                # 0 = no sheet
var slip_r: Vector2 = Vector2.ZERO                      # inner, outer radius (m)

## The sheet's friction from the environment; 0 = off.  Read by the body too (its leg materials).
static func slip_mu_env() -> float:
	var v: String = OS.get_environment("OGMA_PICRAWLER_TERRAIN_SLIP")
	return maxf(0.0, v.to_float()) if v != "" else 0.0

## True when ring k is covered by the sheet (not built; labelled slip).
func ring_covered(k: int) -> bool:
	return slip_mu > 0.0 and ring_r0(k) < slip_r.y and ring_r0(k) + BAND > slip_r.x
var ring_offset: Array = []      # per ring: rotation of sector 0 (rad)
var ring_types: Array = []       # per ring: [type of sector 0..3]
var n_prisms: int = 0
var n_pyramids: int = 0

static func ring_r0(k: int) -> float:
	return CENTRE_R + float(k) * (BAND + BUFFER)

static func field_r0() -> float:
	return ring_r0(HEIGHTS.size() - 1) + BAND + FIELD_GAP

func _init(seed_v: int) -> void:
	seed_used = seed_v
	slip_mu = slip_mu_env()
	if slip_mu > 0.0:
		slip_r = Vector2(CENTRE_R, ring_r0(1))
		var sr: PackedStringArray = OS.get_environment("OGMA_PICRAWLER_TERRAIN_SLIP_R").split(",")
		if sr.size() == 2:
			slip_r = Vector2(sr[0].to_float(), sr[1].to_float())
		slip_r.y = minf(slip_r.y, field_r0())       # the sheet is flat; the bowl is not
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
	if slip_mu > 0.0 and r >= slip_r.x and r < slip_r.y:
		return ["slip", -1, int(round(slip_mu * 100.0))]      # "height" carries mu x 100 for the log
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

# ---- ground specification (solids) ---------------------------------------------------------

const CELL_BOWL: float = 0.5        # bowl cells (m): the exponential is smooth, a 0.5 m plane errs by ~1 mm
const CELL_HUMP: float = 0.25       # slope-hump cells (m)
const SOLID_BOTTOM: float = -0.1    # every solid reaches down to here: thick, so a fast foot cannot cross it in a step

var pyramids: Array = []            # PackedFloat32Array [cx, cz, half, base_y, apex_y, yaw] per pyramid
var humps: Array = []               # [k, sec, slope] per slope-control sector
var n_cells: int = 0

# A square pyramid on the ground: its base plane sits 1 cm below the lowest ground under its footprint (the bowl rises
# outward), its apex `h` above the ground at its centre.
func _add_pyramid(cx: float, cz: float, base: float, h: float, yaw: float) -> void:
	var half: float = base * 0.5
	var r: float = Vector2(cx, cz).length()
	var base_y: float = bowl_at(maxf(0.0, r - half * 1.4143)) - 0.01
	pyramids.append(PackedFloat32Array([cx, cz, half, base_y, bowl_at(r) + h, yaw]))

func _in_sector(k: int, sec: int, x: float, z: float, margin_arc: float) -> bool:
	var r: float = Vector2(x, z).length()
	if r < 1e-6:
		return false
	var a: float = fposmod(atan2(z, x) - float(ring_offset[k]) - float(sec) * 0.5 * PI, 2.0 * PI)
	return a * r >= margin_arc and (0.5 * PI - a) * r >= margin_arc

# The same random draws, in the same order, as the heightmap build: a seed's layout is unchanged.
func _build_specs() -> void:
	pyramids.clear(); humps.clear()
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
					_add_pyramid(px, pz, base, _rng.randf_range(0.4, 1.0) * hk, _rng.randf() * 0.5 * PI)
					placed += 1
				n_pyramids += placed
			elif t == "control" and float(SLOPE_DEG[k]) > 0.0:
				humps.append([k, sec, tan(deg_to_rad(float(SLOPE_DEG[k])))])
	# The bowl: pyramids growing exponentially outward, placed shell by shell so the coverage
	# stays ~FIELD_COVER footprints at every radius.
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
			_add_pyramid(rr * cos(a), rr * sin(a), base, h, _rng.randf() * 0.5 * PI)
			n_pyramids += 1
		r = r1

# A radial hump across the band (up over the first half, down over the second), tapered to zero over 0.6 m of arc at
# the sector's sides so there is no lateral cliff.
func _hump_h(k: int, sec: int, slope: float, x: float, z: float) -> float:
	var r0: float = ring_r0(k)
	var r: float = Vector2(x, z).length()
	if r <= r0 or r >= r0 + BAND:
		return 0.0
	var a: float = fposmod(atan2(z, x) - float(ring_offset[k]) - float(sec) * 0.5 * PI, 2.0 * PI)
	if a > 0.5 * PI:
		return 0.0
	var edge: float = minf(a * r, (0.5 * PI - a) * r)
	return slope * minf(r - r0, r0 + BAND - r) * clampf(edge / 0.6, 0.0, 1.0)

## The smooth ground under the pyramids: the bowl, the slope humps; 0 elsewhere.
func ground_h(x: float, z: float) -> float:
	var h: float = bowl_at(Vector2(x, z).length())
	for hp in humps:
		h = maxf(h, _hump_h(hp[0], hp[1], hp[2], x, z))
	return h

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

const CHUNK: float = 6.5            # solids are grouped into static bodies per CHUNK x CHUNK square
var _chunks: Dictionary = {}
var _root: Node3D
var _layer: int = 1
var _mask: int = 1
var _mat: PhysicsMaterial
var _vis: bool = false
var _tv := PackedVector3Array()     # cell tops for the visual mesh (triangles)
var _tn := PackedVector3Array()
var _tc := PackedColorArray()

func _chunk_body(x: float, z: float) -> StaticBody3D:
	var key := Vector2i(int(floor((x + FLOOR_HALF) / CHUNK)), int(floor((z + FLOOR_HALF) / CHUNK)))
	if not _chunks.has(key):
		var b := StaticBody3D.new()
		b.name = "Ground_%d_%d" % [key.x, key.y]
		b.collision_layer = _layer
		b.collision_mask = _mask
		b.physics_material_override = _mat
		_root.add_child(b)
		_chunks[key] = b
	return _chunks[key]

func _add_solid(points: PackedVector3Array, at: Vector2) -> void:
	var shape := ConvexPolygonShape3D.new()
	shape.points = points
	var cs := CollisionShape3D.new()
	cs.shape = shape
	_chunk_body(at.x, at.y).add_child(cs)

# One ground cell: the convex hull of its four top corners and four corners at SOLID_BOTTOM.  The hull's top is the
# UPPER envelope of the corners, so the visual triangulation takes the diagonal with the higher midpoint to match.
func _add_cell(x0: float, z0: float, x1: float, z1: float, h00: float, h10: float, h11: float, h01: float) -> void:
	var p00 := Vector3(x0, h00, z0); var p10 := Vector3(x1, h10, z0); var p11 := Vector3(x1, h11, z1); var p01 := Vector3(x0, h01, z1)
	_add_solid(PackedVector3Array([p00, p10, p11, p01, Vector3(x0, SOLID_BOTTOM, z0), Vector3(x1, SOLID_BOTTOM, z0),
		Vector3(x1, SOLID_BOTTOM, z1), Vector3(x0, SOLID_BOTTOM, z1)]), Vector2(0.5 * (x0 + x1), 0.5 * (z0 + z1)))
	n_cells += 1
	if not _vis:
		return
	var tris: Array = [[p00, p10, p11], [p00, p11, p01]] if h00 + h11 >= h10 + h01 else [[p00, p10, p01], [p10, p11, p01]]
	for t in tris:
		var n: Vector3 = (t[2] - t[0]).cross(t[1] - t[0]).normalized()
		if n.y < 0.0:
			n = -n
		for q in t:
			_tv.append(q); _tn.append(n); _tc.append(_height_colour(q.y))

func _build_cells() -> void:
	# The bowl, on a square grid inside the rim; cells whose corners are all at floor level are the floor box's.
	var lo: float = -FLOOR_HALF + RIM_W
	var nb: int = int(round(2.0 * (FLOOR_HALF - RIM_W) / CELL_BOWL))
	for j in range(nb):
		var z0: float = lo + float(j) * CELL_BOWL; var z1: float = z0 + CELL_BOWL
		for i in range(nb):
			var x0: float = lo + float(i) * CELL_BOWL; var x1: float = x0 + CELL_BOWL
			var h00: float = bowl_at(Vector2(x0, z0).length()); var h10: float = bowl_at(Vector2(x1, z0).length())
			var h11: float = bowl_at(Vector2(x1, z1).length()); var h01: float = bowl_at(Vector2(x0, z1).length())
			if maxf(maxf(h00, h10), maxf(h11, h01)) < 0.001:
				continue
			_add_cell(x0, z0, x1, z1, h00, h10, h11, h01)
	# The slope humps, finer.
	for hp in humps:
		var r1: float = ring_r0(hp[0]) + BAND
		var nh: int = int(ceil(2.0 * r1 / CELL_HUMP))
		for j in range(nh):
			var z0: float = -r1 + float(j) * CELL_HUMP; var z1: float = z0 + CELL_HUMP
			for i in range(nh):
				var x0: float = -r1 + float(i) * CELL_HUMP; var x1: float = x0 + CELL_HUMP
				var h00: float = _hump_h(hp[0], hp[1], hp[2], x0, z0); var h10: float = _hump_h(hp[0], hp[1], hp[2], x1, z0)
				var h11: float = _hump_h(hp[0], hp[1], hp[2], x1, z1); var h01: float = _hump_h(hp[0], hp[1], hp[2], x0, z1)
				if maxf(maxf(h00, h10), maxf(h11, h01)) < 0.001:
					continue
				_add_cell(x0, z0, x1, z1, h00, h10, h11, h01)

func _pyramid_points(p: PackedFloat32Array) -> PackedVector3Array:
	var rot := Basis(Vector3.UP, p[5])
	var c := Vector3(p[0], 0.0, p[1])
	var pts := PackedVector3Array()
	for o in [Vector3(-p[2], 0, -p[2]), Vector3(p[2], 0, -p[2]), Vector3(p[2], 0, p[2]), Vector3(-p[2], 0, p[2])]:
		var w: Vector3 = c + rot * o
		pts.append(Vector3(w.x, p[3], w.z))
	pts.append(Vector3(p[0], p[4], p[1]))
	return pts

## Adds the whole course under `root`.  `visuals` false skips the meshes (headless).
func build(root: Node3D, contact_mat: PhysicsMaterial, _wedge_mat: PhysicsMaterial, layer: int, mask: int,
		visuals: bool) -> void:
	var t0: int = Time.get_ticks_msec()
	_root = root; _layer = layer; _mask = mask; _mat = contact_mat; _vis = visuals; _chunks.clear()
	_build_specs()
	# The floor: a thick box, top at 0 (the old arena's floor; a fast foot cannot cross 0.2 m in a step).
	var floor_body := StaticBody3D.new()
	floor_body.name = "TerrainFloor"
	floor_body.set_meta("ogma_floor", true)       # instrument tag (load_log `fcol`); no physics effect
	floor_body.collision_layer = layer
	floor_body.collision_mask = mask
	floor_body.physics_material_override = contact_mat
	var fcs := CollisionShape3D.new()
	var fb := BoxShape3D.new()
	fb.size = Vector3(2.0 * FLOOR_HALF, 0.2, 2.0 * FLOOR_HALF)
	fcs.shape = fb
	fcs.position = Vector3(0, -0.1, 0)
	floor_body.add_child(fcs)
	root.add_child(floor_body)
	_tv = PackedVector3Array(); _tn = PackedVector3Array(); _tc = PackedColorArray()
	_build_cells()
	for p in pyramids:
		var pr: float = Vector2(p[0], p[1]).length()
		var covered: bool = false
		for k in range(HEIGHTS.size()):
			if ring_covered(k) and pr >= ring_r0(k) and pr < ring_r0(k) + BAND:
				covered = true
		if not covered:
			_add_solid(_pyramid_points(p), Vector2(p[0], p[1]))
	if slip_mu > 0.0:
		root.add_child(_slip_body(layer, mask))
	# The rim: four thick boxes at the floor edge.
	var rim_h: float = bowl_at(FLOOR_HALF) + 0.5
	var rim_body := StaticBody3D.new()
	rim_body.name = "TerrainRim"
	rim_body.collision_layer = layer; rim_body.collision_mask = mask; rim_body.physics_material_override = contact_mat
	var rims: Array = []
	for e in [[Vector3(FLOOR_HALF - 0.5 * RIM_W, 0, 0), Vector3(RIM_W, 0, 2.0 * FLOOR_HALF)],
			[Vector3(-FLOOR_HALF + 0.5 * RIM_W, 0, 0), Vector3(RIM_W, 0, 2.0 * FLOOR_HALF)],
			[Vector3(0, 0, FLOOR_HALF - 0.5 * RIM_W), Vector3(2.0 * FLOOR_HALF, 0, RIM_W)],
			[Vector3(0, 0, -FLOOR_HALF + 0.5 * RIM_W), Vector3(2.0 * FLOOR_HALF, 0, RIM_W)]]:
		var bs := BoxShape3D.new()
		bs.size = Vector3((e[1] as Vector3).x, rim_h - SOLID_BOTTOM, (e[1] as Vector3).z)
		var rcs := CollisionShape3D.new()
		rcs.shape = bs
		rcs.position = Vector3((e[0] as Vector3).x, 0.5 * (rim_h + SOLID_BOTTOM), (e[0] as Vector3).z)
		rim_body.add_child(rcs)
		rims.append(rcs)
	root.add_child(rim_body)
	if visuals:
		_add_visuals(floor_body, rim_body, rims)
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
			if (t == "steps" or t == "bars") and not ring_covered(k):
				root.add_child(_sector_body(k, sec, t, contact_mat, layer, mask, steps_mat if t == "steps" else bars_mat))
	var layout: Array = []
	for k in range(HEIGHTS.size()):
		layout.append("%d mm @ %.0f deg: %s" % [int(round(float(HEIGHTS[k]) * 1000.0)), rad_to_deg(float(ring_offset[k])), ",".join(ring_types[k])])
	print("PicrawlerBody: [gym] TERRAIN seed %d: %d prisms, %d pyramids, %d ground cells, %d bodies (solid build), built in %d ms" % [
		seed_used, n_prisms, n_pyramids, n_cells, _chunks.size() + 2, Time.get_ticks_msec() - t0])
	for k in range(layout.size()):
		print("PicrawlerBody: [gym] TERRAIN ring %d (r %.1f-%.1f m): %s%s" % [k, ring_r0(k), ring_r0(k) + BAND, layout[k],
			"  — COVERED by the slip sheet, not built" if ring_covered(k) else ""])
	if slip_mu > 0.0:
		print("PicrawlerBody: [gym] TERRAIN SLIP sheet r %.1f-%.1f m, mu %.2f (rough; the legs are smooth, so the floor's mu applies)" % [
			slip_r.x, slip_r.y, slip_mu])

# The slip sheet: annular prism segments, top SLIP_H above the floor, bottom sunk into it (static bodies do not collide
# with each other, and a foot resting on the sheet never reaches the floor's top 2 mm below).  Rough, at slip_mu.
func _slip_body(layer: int, mask: int) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.name = "TerrainSlip"
	body.collision_layer = layer
	body.collision_mask = mask
	var m := PhysicsMaterial.new()
	m.friction = slip_mu
	m.rough = true
	m.bounce = 0.0
	m.absorbent = true
	body.physics_material_override = m
	_pv = PackedVector3Array()
	_pn = PackedVector3Array()
	var n_seg: int = maxi(24, int(ceil(2.0 * PI * slip_r.y / SEG_CHORD)))
	for s in range(n_seg):
		var a0: float = 2.0 * PI * float(s) / float(n_seg)
		var a1: float = 2.0 * PI * float(s + 1) / float(n_seg)
		# Outer radius over cos(half-angle): the chord stays outside slip_r.y, so the polygon covers the annulus.
		var ro: float = slip_r.y / cos(PI / float(n_seg))
		var v: PackedVector3Array = _prism(slip_r.x, ro, slip_r.x, ro, SLIP_H, a0, a1)
		var shape := ConvexPolygonShape3D.new()
		shape.points = v
		var cs := CollisionShape3D.new()
		cs.shape = shape
		body.add_child(cs)
		if _vis:
			_add_prism_mesh(v)
	if _vis:
		var arr: Array = []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = _pv
		arr[Mesh.ARRAY_NORMAL] = _pn
		var am := ArrayMesh.new()
		am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
		var vm := StandardMaterial3D.new()
		vm.albedo_color = Color(0.55, 0.60, 0.66)      # cool grey-blue vinyl, glossy: reads as "slick" at a glance
		vm.roughness = 0.15
		vm.metallic_specular = 0.8
		vm.cull_mode = BaseMaterial3D.CULL_DISABLED
		var mi := MeshInstance3D.new()
		mi.mesh = am
		mi.set_surface_override_material(0, vm)
		body.add_child(mi)
	return body

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

func _vertex_colour_mat() -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.vertex_color_use_as_albedo = true
	m.vertex_color_is_srgb = true     # the ramp is written in sRGB like every albedo here; read linear it washes out
	m.roughness = 0.95
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	return m

# The visuals, drawn from the SAME solids the feet touch: the floor plane, the cell tops, every pyramid (one instanced
# pyramid as its own triangles) and the rim.
func _add_visuals(floor_body: Node3D, rim_body: Node3D, rims: Array) -> void:
	var plane := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(2.0 * FLOOR_HALF, 2.0 * FLOOR_HALF)
	plane.mesh = pm
	plane.position = Vector3(0, -0.0005, 0)
	var fmat := StandardMaterial3D.new()
	fmat.albedo_color = Color(0.3, 0.35, 0.3, 1.0)
	plane.set_surface_override_material(0, fmat)
	floor_body.add_child(plane)
	if not _tv.is_empty():
		var arr: Array = []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = _tv; arr[Mesh.ARRAY_NORMAL] = _tn; arr[Mesh.ARRAY_COLOR] = _tc
		var am := ArrayMesh.new()
		am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
		var mi := MeshInstance3D.new()
		mi.mesh = am
		mi.set_surface_override_material(0, _vertex_colour_mat())
		floor_body.add_child(mi)
	# Every pyramid as real triangles with its own face normals (an instanced unit pyramid under non-uniform scale lit
	# its faces wrongly — half of them rendered near black), coloured per vertex by height: dark at the base, lighter up.
	var pv := PackedVector3Array(); var pn := PackedVector3Array(); var pc := PackedColorArray()
	for p in pyramids:
		var pts: PackedVector3Array = _pyramid_points(p)
		var apex: Vector3 = pts[4]
		var mid: Vector3 = (pts[0] + pts[2]) * 0.5
		for i in range(4):
			var a: Vector3 = pts[i]; var b: Vector3 = pts[(i + 1) % 4]
			var n: Vector3 = (b - a).cross(apex - a).normalized()
			if n.dot((a + b) * 0.5 - mid) < 0.0:
				n = -n
			for q in [a, b, apex]:
				pv.append(q); pn.append(n); pc.append(_height_colour(q.y))
	if not pv.is_empty():
		var parr: Array = []
		parr.resize(Mesh.ARRAY_MAX)
		parr[Mesh.ARRAY_VERTEX] = pv; parr[Mesh.ARRAY_NORMAL] = pn; parr[Mesh.ARRAY_COLOR] = pc
		var pmesh := ArrayMesh.new()
		pmesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, parr)
		var pmi := MeshInstance3D.new()
		pmi.mesh = pmesh
		pmi.set_surface_override_material(0, _vertex_colour_mat())
		floor_body.add_child(pmi)
	var rmat := StandardMaterial3D.new()
	rmat.albedo_color = _height_colour(2.5)
	for rcs in rims:
		var bm := BoxMesh.new()
		bm.size = ((rcs as CollisionShape3D).shape as BoxShape3D).size
		var rmi := MeshInstance3D.new()
		rmi.mesh = bm
		rmi.position = (rcs as CollisionShape3D).position
		rmi.set_surface_override_material(0, rmat)
		rim_body.add_child(rmi)
