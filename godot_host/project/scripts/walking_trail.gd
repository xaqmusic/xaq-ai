extends Node3D
## Visualises the picrawler's walking path by dropping a red X-mark every
## METRE the body travels (cumulative path distance) — anywhere in the world,
## in EITHER gym (arena or corridor).  Consecutive marks are joined by a thin
## line so the whole route is legible from above.  Pure UI; no physics, no
## brain hooks.
##
## Tracking semantics (2026-07-23 rewrite — was ring-crossing, which only
## worked inside the donut's 0.1–3.0 m rings and did nothing in the corridor):
##   - Sample the target body's chassis XZ each _process tick.
##   - Accumulate the per-tick step distance into a running path length.
##   - Each time the running length passes the next whole-metre threshold,
##     drop an X at the interpolated point exactly on that metre and connect
##     it to the previous mark.  Distance TRAVELLED (not net displacement),
##     so wandering-in-place still drops marks (they cluster → you can SEE
##     the robot isn't making progress).
##
## Toggle with the body's [P] hotkey (set_shown / toggle_shown flip `visible`,
## which hides the node + all spawned marks at once).
##
## Marker shape: two thin red box meshes rotated ±45° around Y, flat on the
## floor.  Connector: one thin box spanning consecutive marks.

@export var target_path: NodePath
@export var ring_y: float = 0.0035   # marker height above the floor (both gyms' floor is y=0)

const _MARK_INTERVAL: float = 1.0    # metres of travel between X marks
const _MARK_COLOR: Color = Color(0.95, 0.20, 0.20, 1.0)   # red
const _MARK_ARM_HALF_LEN: float = 0.025   # 5 cm total arm length
const _MARK_ARM_THICK:    float = 0.005   # 5 mm thick
const _MARK_ARM_HEIGHT:   float = 0.003   # 3 mm tall (just above floor)
const _LINE_THICK:        float = 0.004   # connector line width

var _target: Node3D = null
var _prev_pos: Vector3 = Vector3.ZERO     # last sampled chassis position
var _have_prev: bool = false
var _accum: float = 0.0                    # cumulative path length walked (m)
var _next_mark: float = 0.0                # next metre threshold to mark at
var _last_mark_pos: Vector3 = Vector3.ZERO
var _have_first_mark: bool = false
var _ring_mat: StandardMaterial3D = null

func _ready() -> void:
	_ring_mat = StandardMaterial3D.new()
	_ring_mat.albedo_color = _MARK_COLOR
	_ring_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	if target_path != NodePath(""):
		_target = get_node_or_null(target_path)

func set_target(node: Node3D) -> void:
	_target = node

# --- show/hide (body [P] hotkey) --------------------------------------------
func set_shown(on: bool) -> void:
	visible = on

func toggle_shown() -> bool:
	visible = not visible
	return visible

func clear() -> void:
	# Wipe all spawned marks + reset the distance accumulator so the next
	# metre of travel starts a fresh trail.  Called from the body's
	# _do_hard_reset() (SPACE-press / gym-switch) so each run gets a clean floor.
	for child in get_children():
		if child is MeshInstance3D:
			child.queue_free()
	_have_prev = false
	_accum = 0.0
	_next_mark = 0.0
	_last_mark_pos = Vector3.ZERO
	_have_first_mark = false

func _process(_delta: float) -> void:
	if _target == null or not is_instance_valid(_target):
		return
	var pos: Vector3 = _target.global_transform.origin
	if not _have_prev:
		# First sample: mark the start point (0 m) so the route has an origin.
		_prev_pos = pos
		_have_prev = true
		_drop_mark(Vector3(pos.x, ring_y, pos.z))
		_next_mark = _MARK_INTERVAL
		return
	var step: float = Vector2(pos.x - _prev_pos.x, pos.z - _prev_pos.z).length()
	if step <= 1e-6:
		_prev_pos = pos
		return
	var seg_start: float = _accum
	_accum += step
	# Drop a mark for every whole-metre threshold crossed this step (usually
	# 0 or 1; the loop covers a rare large jump / teleport gracefully).
	while _accum >= _next_mark:
		var frac: float = clamp((_next_mark - seg_start) / step, 0.0, 1.0)
		var mark: Vector3 = Vector3(
			lerp(_prev_pos.x, pos.x, frac), ring_y, lerp(_prev_pos.z, pos.z, frac))
		_drop_mark(mark)
		_next_mark += _MARK_INTERVAL
	_prev_pos = pos

func _drop_mark(world_pos: Vector3) -> void:
	if _have_first_mark:
		_spawn_line(_last_mark_pos, world_pos)
	_spawn_marker(world_pos)
	_last_mark_pos = world_pos
	_have_first_mark = true

func _spawn_marker(world_pos: Vector3) -> void:
	# Two crossed box meshes form an X, both flat to the floor.
	for ang_deg in [45.0, -45.0]:
		var m := MeshInstance3D.new()
		var box := BoxMesh.new()
		box.size = Vector3(_MARK_ARM_HALF_LEN * 2.0, _MARK_ARM_HEIGHT, _MARK_ARM_THICK)
		m.mesh = box
		m.set_surface_override_material(0, _ring_mat)
		var b: Basis = Basis().rotated(Vector3.UP, deg_to_rad(ang_deg))
		m.global_transform = Transform3D(b, world_pos)
		add_child(m)

func _spawn_line(a: Vector3, b: Vector3) -> void:
	# One thin box spanning a→b (flat on the floor) — the route connector.
	var delta: Vector3 = b - a
	delta.y = 0.0
	var total_len: float = delta.length()
	if total_len < 1e-4:
		return
	var dir: Vector3 = delta.normalized()
	var yaw: float = atan2(dir.x, dir.z)   # box local +Z → dash direction
	var basis: Basis = Basis().rotated(Vector3.UP, yaw)
	var mid: Vector3 = a + dir * (total_len * 0.5)
	mid.y = ring_y
	var seg := MeshInstance3D.new()
	var sb := BoxMesh.new()
	sb.size = Vector3(_LINE_THICK, _MARK_ARM_HEIGHT, total_len)
	seg.mesh = sb
	seg.set_surface_override_material(0, _ring_mat)
	seg.global_transform = Transform3D(basis, mid)
	add_child(seg)
