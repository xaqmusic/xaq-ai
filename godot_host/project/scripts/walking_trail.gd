extends Node3D
## Visualises the picrawler's walking progress by spawning a red X-mark
## at each floor-ring crossing point and a dashed line connecting
## consecutive crossings.  Pure UI; no physics, no brain hooks.
##
## The ring radii must match picrawler_body.gd::_build_floor_rings — if
## those change, update _RING_RADII here too.  Hardcoded rather than
## introspected from the world because rings are a visual property of
## the scene, not a behavioural one.
##
## Tracking semantics:
##   - We sample the target body's chassis position each _process tick.
##   - For each ring, the FIRST time chassis distance crosses the
##     ring's radius going OUTWARD, we mark that crossing point.
##   - Inward re-crossings (body retreating) do NOT mark — keeps the
##     trail monotonically growing.  If the body crosses outward then
##     inward then outward again, the second outward crossing IS marked
##     (we treat each "outward-first" event per ring as a separate
##     observation).
##
## Marker shape: two thin red box meshes, rotated ±45° around Y, sitting
## just above the floor.  Dashed connector: a chain of 2-cm-long box
## segments along the line between consecutive markers, with 2-cm gaps.

@export var target_path: NodePath
@export var ring_y: float = 0.0035   # match floor rings' y in picrawler_body.gd

# Must mirror picrawler_body.gd::_build_floor_rings.
const _RING_RADII: Array = [0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0]
const _MARK_COLOR: Color = Color(0.95, 0.20, 0.20, 1.0)   # red
const _MARK_ARM_HALF_LEN: float = 0.025   # 5 cm total arm length
const _MARK_ARM_THICK:    float = 0.005   # 5 mm thick
const _MARK_ARM_HEIGHT:   float = 0.003   # 3 mm tall (just above floor)
const _DASH_LEN:  float = 0.02
const _DASH_GAP:  float = 0.02

var _target: Node3D = null
# Distance-from-origin at last sampled tick — used to detect outward
# crossings of each ring radius this tick.
var _prev_dist: float = -1.0
# Per-ring "currently above the radius" state.  True means body is
# currently outside this ring; transitions false→true mark a crossing.
var _outside_ring: Array = []
var _last_mark_pos: Vector3 = Vector3.ZERO   # for connecting dashed line
var _have_first_mark: bool = false
var _ring_mat: StandardMaterial3D = null

func _ready() -> void:
    _outside_ring.resize(_RING_RADII.size())
    for i in range(_RING_RADII.size()):
        _outside_ring[i] = false
    _ring_mat = StandardMaterial3D.new()
    _ring_mat.albedo_color = _MARK_COLOR
    _ring_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    if target_path != NodePath(""):
        _target = get_node_or_null(target_path)

func set_target(node: Node3D) -> void:
    _target = node

func clear() -> void:
    # Wipe all spawned X-marker and dashed-segment MeshInstance3Ds and
    # reset the per-ring crossing state so the next outward crossing of
    # each ring is marked again.  Called from the body's _do_hard_reset()
    # so SPACE-press in the UI gives a fresh floor.
    for child in get_children():
        if child is MeshInstance3D:
            child.queue_free()
    _prev_dist = -1.0
    for i in range(_RING_RADII.size()):
        _outside_ring[i] = false
    _last_mark_pos = Vector3.ZERO
    _have_first_mark = false

func _process(_delta: float) -> void:
    if _target == null:
        return
    var pos: Vector3 = _target.global_transform.origin
    var dist: float = Vector2(pos.x, pos.z).length()
    if _prev_dist < 0.0:
        _prev_dist = dist
        # Initialise per-ring state to match current position.
        for i in range(_RING_RADII.size()):
            _outside_ring[i] = (dist > _RING_RADII[i])
        return
    # Detect outward crossings for each ring.
    for i in range(_RING_RADII.size()):
        var r: float = _RING_RADII[i]
        var was_outside: bool = _outside_ring[i]
        var now_outside: bool = (dist > r)
        if now_outside and not was_outside:
            # Outward crossing — interpolate the crossing point along the
            # chord from prev → current sample so the mark lands ON the
            # ring rather than at the discrete sample point.
            var crossing: Vector3 = _interpolate_crossing(pos, _target.global_transform.origin, r)
            _spawn_marker(crossing)
            if _have_first_mark:
                _spawn_dashed_line(_last_mark_pos, crossing)
            _last_mark_pos = crossing
            _have_first_mark = true
        _outside_ring[i] = now_outside
    _prev_dist = dist

func _interpolate_crossing(curr_pos: Vector3, last_pos: Vector3, r: float) -> Vector3:
    # Approximate: bisect along the chord to find the ring intersection.
    # `curr_pos` and `last_pos` differ negligibly per frame so a linear
    # interpolation by distance fraction is accurate enough for visual
    # purposes.
    var d_curr: float = Vector2(curr_pos.x, curr_pos.z).length()
    var d_prev: float = _prev_dist
    if abs(d_curr - d_prev) < 1e-6:
        return Vector3(curr_pos.x, ring_y, curr_pos.z)
    var t: float = clamp((r - d_prev) / (d_curr - d_prev), 0.0, 1.0)
    var xz_prev: Vector2 = Vector2(last_pos.x, last_pos.z)
    var xz_curr: Vector2 = Vector2(curr_pos.x, curr_pos.z)
    var xz: Vector2 = xz_prev.lerp(xz_curr, t)
    return Vector3(xz.x, ring_y, xz.y)

func _spawn_marker(world_pos: Vector3) -> void:
    # Two crossed box meshes form an X.  Both flat to the floor.
    for ang_deg in [45.0, -45.0]:
        var m := MeshInstance3D.new()
        var box := BoxMesh.new()
        box.size = Vector3(_MARK_ARM_HALF_LEN * 2.0, _MARK_ARM_HEIGHT, _MARK_ARM_THICK)
        m.mesh = box
        m.set_surface_override_material(0, _ring_mat)
        var b: Basis = Basis().rotated(Vector3.UP, deg_to_rad(ang_deg))
        m.global_transform = Transform3D(b, world_pos)
        add_child(m)

func _spawn_dashed_line(a: Vector3, b: Vector3) -> void:
    var delta: Vector3 = b - a
    delta.y = 0.0    # keep flat on the floor
    var total_len: float = delta.length()
    if total_len < _DASH_LEN * 0.5:
        return
    var dir: Vector3 = delta.normalized()
    var step: float = _DASH_LEN + _DASH_GAP
    var n_dashes: int = int(floor(total_len / step)) + 1
    # Yaw so the box's local +Z aligns with the dash direction.
    var yaw: float = atan2(dir.x, dir.z)
    var basis: Basis = Basis().rotated(Vector3.UP, yaw)
    var t: float = 0.0
    while t < total_len:
        var dash_end_t: float = min(t + _DASH_LEN, total_len)
        var seg_len: float = dash_end_t - t
        var seg_mid: Vector3 = a + dir * (t + seg_len * 0.5)
        seg_mid.y = ring_y
        var seg := MeshInstance3D.new()
        var sb := BoxMesh.new()
        sb.size = Vector3(_MARK_ARM_THICK, _MARK_ARM_HEIGHT, seg_len)
        seg.mesh = sb
        seg.set_surface_override_material(0, _ring_mat)
        seg.global_transform = Transform3D(basis, seg_mid)
        add_child(seg)
        t += step
    # Avoid unused warning on n_dashes when not used elsewhere.
    if n_dashes < 0: pass
