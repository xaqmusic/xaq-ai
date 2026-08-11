extends Camera3D
## Orbit camera for 3D body inspection.  Mouse drag (left button) = orbit
## around focal point; wheel = zoom in/out; right-click drag = pan focal
## point (rare; mostly for debugging).  The focal point follows a target
## Node3D each frame, so the camera tracks the body's chassis as it moves.
##
## Reads target via @export `target_path` — assign in scene editor or set
## from a body script at _ready time.

@export var target_path: NodePath
@export var initial_distance: float = 2.5
@export var initial_yaw_deg:   float = 0.0
@export var initial_pitch_deg: float = -25.0
@export var min_distance: float = 0.5
@export var max_distance: float = 20.0
@export var min_pitch_deg: float = -85.0
@export var max_pitch_deg: float = 85.0
@export var orbit_speed: float = 0.35     # deg per pixel
@export var zoom_factor: float = 1.12     # multiplier per wheel notch
@export var pan_speed: float   = 0.005

# When true, the focal point's Y component is locked to `floor_y` (default 0)
# instead of tracking the target's vertical motion.  Useful for jumping /
# bouncing bodies where the chassis Y oscillates a lot and following it
# makes the view nauseating.  Only X/Z follow the target horizontally.
@export var floor_lock: bool   = false
@export var floor_y:    float  = 0.0

# Horizontal tracking smoothing — exponential low-pass on the target's
# X/Z position.  As the body becomes more mobile, raw per-tick tracking
# produces jarring camera jitter; this damps high-frequency wobble while
# still following overall motion.  The value is the time constant in
# seconds: ~63% of the gap to the new target is closed per `time_const`,
# 95% in ~3·time_const.  0 disables smoothing (snap tracking).  Only
# X/Z are smoothed — vertical Y is either floor-locked or follows raw.
@export var follow_smoothing_seconds: float = 0.5

var _target: Node3D
var _distance: float
var _yaw: float
var _pitch: float
var _orbiting: bool = false
var _panning: bool = false
var _focal_offset: Vector3 = Vector3.ZERO
# Low-passed horizontal focal target — only X/Z are smoothed.
var _smoothed_xz: Vector2 = Vector2.ZERO
var _smoothed_xz_init: bool = false

func _ready() -> void:
    _distance = initial_distance
    _yaw     = deg_to_rad(initial_yaw_deg)
    _pitch   = deg_to_rad(initial_pitch_deg)
    if target_path != NodePath(""):
        _target = get_node_or_null(target_path)
    _apply_pose()

func set_target(node: Node3D) -> void:
    _target = node
    # Re-seed the low-pass to the new target's current position so the
    # camera doesn't start by sweeping across the world.
    _smoothed_xz_init = false

func snap_to_target() -> void:
    ## Snap the smoothed focal directly to the target.  Call from a body
    ## hard-reset if you want the camera to track the teleport instantly
    ## rather than slowly sliding back over follow_smoothing_seconds.
    _smoothed_xz_init = false

func _process(_delta: float) -> void:
    # is_instance_valid, NOT `!= null`: a queue_free()d node is not null, it is a
    # FREED INSTANCE, and touching it throws every frame.  That is precisely what
    # happened when the live body swap ([B]) began rebuilding the chassis — input
    # kept updating _orbiting/_distance while _apply_pose() aborted on the dead
    # reference, so the camera looked like it had stopped taking input.
    # The body re-targets us on every rebuild; this is the belt to that braces.
    if not is_instance_valid(_target):
        return
    _apply_pose()

func _apply_pose() -> void:
    var raw_target: Vector3 = _target.global_transform.origin if is_instance_valid(_target) else Vector3.ZERO
    # Low-pass X/Z to damp jarring per-tick jitter as the body becomes
    # more mobile.  Run BEFORE the floor_lock + offset application so
    # those still respond immediately to user input (right-click pan).
    if follow_smoothing_seconds > 0.0 and is_instance_valid(_target):
        var dt: float = get_process_delta_time()
        var target_xz: Vector2 = Vector2(raw_target.x, raw_target.z)
        if not _smoothed_xz_init:
            _smoothed_xz = target_xz
            _smoothed_xz_init = true
        else:
            # alpha = 1 - exp(-dt/τ) — framerate-independent exponential
            # low-pass.  With dt≈0.016 s and τ=0.5 s, α≈0.032 per frame
            # → 95% catch-up in ~1.5 s, smooth but still feels reactive.
            var alpha: float = clamp(1.0 - exp(-dt / follow_smoothing_seconds), 0.0, 1.0)
            _smoothed_xz = _smoothed_xz.lerp(target_xz, alpha)
        raw_target.x = _smoothed_xz.x
        raw_target.z = _smoothed_xz.y
    var focal: Vector3 = raw_target + _focal_offset
    if floor_lock:
        # Project to the floor plane: keep X/Z (horizontal tracking) but
        # replace Y with the floor reference.  Eliminates the up-and-down
        # camera bob that the bouncing-chassis behaviour induces.
        focal.y = floor_y
    # Spherical → cartesian offset
    var cp: float = cos(_pitch)
    var offset: Vector3 = Vector3(
        cp * sin(_yaw),
        sin(_pitch),
        cp * cos(_yaw),
    ) * _distance
    global_transform = Transform3D(Basis(), focal + offset).looking_at(focal, Vector3.UP)

func _unhandled_input(event: InputEvent) -> void:
    if event is InputEventMouseButton:
        var mb := event as InputEventMouseButton
        match mb.button_index:
            MOUSE_BUTTON_LEFT:
                _orbiting = mb.pressed
            MOUSE_BUTTON_RIGHT:
                _panning = mb.pressed
            MOUSE_BUTTON_WHEEL_UP:
                if mb.pressed:
                    _distance = clamp(_distance / zoom_factor, min_distance, max_distance)
            MOUSE_BUTTON_WHEEL_DOWN:
                if mb.pressed:
                    _distance = clamp(_distance * zoom_factor, min_distance, max_distance)
    elif event is InputEventMouseMotion:
        var mm := event as InputEventMouseMotion
        if _orbiting:
            _yaw   -= deg_to_rad(mm.relative.x * orbit_speed)
            _pitch += deg_to_rad(mm.relative.y * orbit_speed)
            _pitch  = clamp(_pitch, deg_to_rad(min_pitch_deg), deg_to_rad(max_pitch_deg))
        elif _panning:
            # Pan focal offset in camera-relative XY.
            var right: Vector3 = global_transform.basis.x
            var up:    Vector3 = global_transform.basis.y
            _focal_offset += (-right * mm.relative.x + up * mm.relative.y) * pan_speed * _distance
