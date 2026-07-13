extends Node3D
## Quadruped body controller for v6.0.a — brain-only standing balance.
##
## Procedurally builds an 8-DOF quadruped at _ready(): RigidBody3D chassis
## with four HingeJoint3D-attached upper legs (hips) and four HingeJoint3D-
## attached lower legs (knees).  All joints rotate about the lateral (X)
## axis — flexion/extension only, no abduction.
##
## Brain control surface (4 channels):
##   action.fl  hip torque on front-left  upper leg, ∈ [-1, +1]
##   action.fr  hip torque on front-right upper leg
##   action.rl  hip torque on rear-left   upper leg
##   action.rr  hip torque on rear-right  upper leg
##
## Knees are NOT under brain control in v6.0.a — each knee is a passive
## proportional-derivative controller tracking a fixed bent-stance target
## angle.  The brain only needs to coordinate hip extension to keep the
## chassis above the ground.
##
## Per-tick proprio published to the brain:
##   imu             : 4-D (sin/cos chassis yaw, normalised forward velocity,
##                          normalised angular velocity about Y)
##   joints          : 8-D (4 hip angles + 4 knee angles, each ∈ [-1, +1])
##   foot_contact    : 4-D (0/1 per leg, true when lower leg's foot collides
##                          with the ground plane)
##   chassis_height  : 1-D (chassis Y position above the ground plane,
##                          normalised by NOMINAL_STAND_HEIGHT)
##
## Events:
##   body_alive  per-tick reward while chassis_height > ALIVE_HEIGHT_THRESHOLD
##               AND chassis tilt < ALIVE_TILT_THRESHOLD.  Pure cartpole-style
##               survival pulse — the brain's only signal that "being up"
##               is good.
##   fell_over   sparse aversive, fires once when chassis_height drops below
##               FAIL_HEIGHT_THRESHOLD OR chassis tilt exceeds FAIL_TILT_THRESHOLD.
##               Ends the episode.
##
## Episode lifecycle mirrors cart_body.gd: fixed-step 50 Hz physics
## accumulator, episodes reset on fell_over or max_steps, JSONL diag stream
## via print(JSON.stringify(...)) for downstream parsing.

# ---------------------------------------------------------------------------
# Exports — configurable per-run via @export defaults or env vars
# ---------------------------------------------------------------------------
@export var config_path: String = "res://addons/ami_ogma/configs/the_quadruped_minimal.json"
@export var max_steps: int           = 6000     # 120 s at 50 Hz
@export var max_episodes: int        = 0        # 0 = unbounded; runner sends quit
@export var diag_interval_ticks: int = 60       # JSONL emit interval; 0 = silent

# v6.0.a.11 — task mode.
#   "balance"       — original task: body starts standing, brain must
#                     keep it upright; reward = body_alive (binary
#                     above ALIVE_HEIGHT_THRESHOLD), terminate on fall.
#   "crouch_extend" — body starts in its natural collapsed crouch
#                     (chassis ~0.105m, knees folded against limit),
#                     brain learns to LIFT chassis upward.  Reward =
#                     body_lift events scaled by chassis_y directly
#                     (every mm up = more DA).  No height-based
#                     termination; only extreme tilt fails.  Best paired
#                     with reset_mode=continuous + mc_episode_period
#                     so MC updates fire without teleporting.
#                     Override via OGMA_QUADRUPED_TASK env var.
@export var task_mode: String = "balance"
@export var mc_episode_period: int = 0          # If > 0 and continuous-mode, fire events.episode_end
                                                # every N body ticks so Premotor MC finalises without
                                                # a body teleport.  0 = legacy (event only fires on
                                                # _finish_episode in non-continuous reset modes).
                                                # Override via OGMA_QUADRUPED_MC_PERIOD.

# Episode-boundary handling.  Three modes — see project plan v6.0 for full
# rationale.  Override at launch time via OGMA_RESET_MODE env var.
#   "continuous"    — no reset on fell_over.  Body lies where it fell;
#                     brain has to actually un-fall via its own actions
#                     to resume earning body_alive rewards.  Default.
#                     Best for biological plausibility; one long "episode".
#   "soft_blink"    — on fell_over: 30-tick brain blackout (zero torque
#                     applied, no body_alive emitted), chassis kinematically
#                     lerped back to standing pose over those ticks, then
#                     physics+brain resumes.  Brain experiences a "sleep
#                     interval" rather than a teleport.
#   "instant_pause" — legacy CartPole-style instant teleport, with a single-
#                     tick brain.tick() skip on the reset tick.  Cheap but
#                     doesn't fix the surprise spike on the tick AFTER.
@export var reset_mode: String = "continuous"
const _RESET_MODE_CHOICES := ["continuous", "soft_blink", "instant_pause"]

# Chassis collision/visual shape.  "half_cylinder" (default) rolls on
# its rounded back so the agent isn't permanently stuck after a fall.
# "box" is the original rectangular chassis — flat-back stable resting
# position (useful as a comparison baseline; the brain can't recover
# via random torques on a box).
@export var chassis_shape: String = "half_cylinder"
const _CHASSIS_SHAPE_CHOICES := ["half_cylinder", "box"]

# Leg configuration.
#   "spider"  — legs splay outward from the chassis corners; feet land
#               in a wide support polygon with chassis COG roughly at
#               leg-attachment height.  Statically stable so the brain
#               can focus on perturbation rejection + locomotion rather
#               than constantly fighting gravity.  DEFAULT.
#   "dog"     — original inverted-pendulum config: legs hang straight
#               down, narrow support polygon, COG well above legs.
#               Kept for regression comparison and morphology A/B.
@export var body_morphology: String = "spider"
const _BODY_MORPHOLOGY_CHOICES := ["spider", "dog"]
# Spider upper-leg slope below horizontal (downward sag from the hip).
# 15° gives ~0.05m elbow drop with the rest of the leg drop from a
# vertical lower segment — chassis settles around y = 0.30 m at rest.
const SPIDER_UPPER_SLOPE: float = 0.262   # radians, ≈ 15°

# Global scalar applied to MAX_HIP_TORQUE and the knee PD gains.  Lets
# the operator dial leg "strength" without recompiling — useful while
# tuning a new body geometry where the static-stand torque (~weight ×
# moment-arm) isn't known yet.  1.0 = constants as-coded; values >1
# strengthen the legs (more authoritative motor response, potentially
# unstable above ~10); values <1 soften them (more compliant, may not
# support body weight).  Resolved through ExperimentConfig (launcher
# > OGMA_LEG_STRENGTH env > @export default).
@export var leg_strength: float = 1.0
const _SOFT_BLINK_TICKS: int = 30
# Height of the sinusoidal Y-bump applied during a soft_blink, peaks at the
# midpoint of the lerp.  Must exceed the leg extent radius from the chassis
# centre (≈ CHASSIS half-height + UPPER_LEG_LEN + LOWER_LEG_LEN = 0.09 +
# 0.18 + 0.18 ≈ 0.45 m) so feet clear the floor while the rotation slerps.
const _BLINK_LIFT_HEIGHT: float = 0.55

# ---------------------------------------------------------------------------
# Physics constants (loose dog-sized quadruped, MKS units)
# ---------------------------------------------------------------------------
const TAU: float                = 0.02            # 50 Hz fixed step

# Physics oversampling — Godot's default 60 Hz × 8 solver iterations is
# insufficient for stiff articulated chains under load (joints accumulate
# constraint error → impulse spikes → "physics explosions" the user
# reported at leg_strength≥2).  We crank the global physics rate up
# without touching brain tick rate: the body's _step_one accumulator
# fires at TAU=50 Hz regardless of physics_hz, so sub-steps happen
# inside what the brain sees as one tick.  Reasonable values:
#   physics_hz=60   solver=8     Godot default — explodes here
#   physics_hz=120  solver=16    decent stability, modest cost
#   physics_hz=240  solver=16    used here; ~4× sub-stepping
#   physics_hz=480+ solver=32    overkill for this body
# Override at launch via OGMA_PHYSICS_HZ / OGMA_SOLVER_ITERATIONS.
@export var physics_hz: int        = 240
@export var solver_iterations: int = 32         # was 16; knees were violating their hinge axis
                                                # under load when the constraint solver ran out of
                                                # iteration budget at saturated joint limits.
const CHASSIS_MASS: float       = 5.0             # kg
const SHOULDER_MASS: float      = 0.2             # kg, each (intermediate body
                                                  # between chassis and upper leg; provides
                                                  # the yaw DOF anchor.  Heavier than
                                                  # initial 0.05 kg so the solver has
                                                  # enough rotational inertia to keep the
                                                  # yaw constraint stable under pitch
                                                  # torque transmitted through it.)
const SHOULDER_RADIUS: float    = 0.025           # m, collision sphere radius
const UPPER_LEG_MASS: float     = 0.5             # kg, each
const LOWER_LEG_MASS: float     = 0.3             # kg, each
# Chassis is a half-cylinder oriented along X with the flat side DOWN
# (legs attach to it) and the curved side UP (so when the dog falls onto
# its back it rolls rather than getting stuck on a flat resting surface).
# Cross-section is a half-disk of radius CHASSIS_RADIUS; the origin is
# at the bounding-box centre, so flat bottom is at chassis-local
# Y = -R/2 and the curve apex is at +R/2.  See _make_half_cylinder_mesh
# / _make_half_cylinder_hull_points for the exact geometry.
const CHASSIS_LENGTH: float     = 0.50            # X span
const CHASSIS_RADIUS: float     = 0.16            # Z spans 2R; curve rises R above flat side
const UPPER_LEG_LEN: float      = 0.18
const LOWER_LEG_LEN: float      = 0.18
const LEG_RADIUS: float         = 0.025
const HIP_OFFSET_X: float       = 0.20            # front/rear from chassis centre
const HIP_OFFSET_Z: float       = 0.13            # left/right from chassis centre
const MAX_BRAIN_TORQUE: float   = 0.6             # Nm — max torque the servo loop
                                                  # can output per joint.  Scales by
                                                  # leg_strength.  This is the upper
                                                  # bound the position-tracking
                                                  # servo can apply when the joint
                                                  # is far from its commanded angle.

# v6.0.a.8 — brain commands POSITION (angle target), not torque.  The
# previous direct-torque path turned every tick of brain noise into a
# joint impulse, which flailed the legs (random Premotor at uniform
# softmax flips intents every 20 ms → torque kicks alternating
# direction → wild leg oscillation, body launched into air).  Real
# hobby servos take an angle command and a built-in PD loop tracks
# it; the SERVO_KP/SERVO_KD constants below approximate that loop.
# Brain output u ∈ [-1, +1] becomes target angle = u × TARGET_RANGE.
# Torque (clamped to MAX_BRAIN_TORQUE × leg_strength) tracks the
# target.  Net effect: high motor authority but bounded slew rate —
# brain noise → smooth target wobble, not joint flailing.
const SERVO_HIP_KP: float       = 4.0
const SERVO_HIP_KD: float       = 0.4
const SERVO_KNEE_KP: float      = 4.0
const SERVO_KNEE_KD: float      = 0.4
const HIP_TARGET_RANGE: float   = 0.50            # rad — brain ±1 → ±0.5 rad target
const KNEE_TARGET_RANGE: float  = 0.80            # rad — knee can reach further bend

# v6.0.a.5 — control allocation swap.  Previously the brain drove hip
# pitch and the knee was a strong PD that ended up doing most of the
# work to keep the body up.  Now the brain drives the KNEE (the
# leg-length axis — bending knees drops the body, straightening lifts
# it).  The hip is locked by a stiff body-side PD at its rest pose so
# the leg's stance angle stays fixed and only knee bend changes leg
# length.  Closer to how a real quadruped balances: knee servos vary
# leg extension to maintain height, hip stays at stance.
const HIP_REST_KP: float        = 0.5             # Mild rest PD on hip pitch.  Just
const HIP_REST_KD: float        = 0.10            # enough to bias the hip back
                                                  # toward 0 when the brain stops
                                                  # commanding it; the hard work of
                                                  # bounding hip excursion is done
                                                  # by the joint's near-rigid soft
                                                  # limit (HIP_LIMIT_SOFTNESS=0.95).
                                                  # Stronger PD (KP=2.0) tested but
                                                  # the per-tick PD impulses
                                                  # destabilised the chassis worse
                                                  # than weak-PD + hard-limit.
const KNEE_REST_KP: float       = 4.0             # Strong rest PD ("muscle tone").
const KNEE_REST_KD: float       = 0.4             # Without it, random brain torques
                                                  # flap the knees between joint
                                                  # limits → body oscillates wildly.
                                                  # Brain authority = brain_torque /
                                                  # KNEE_REST_KP rad of deflection
                                                  # from rest per Nm of command.
const KNEE_REST_ANGLE: float    = -0.6            # MORE bent than the ~-0.28 angle
                                                  # that puts the foot exactly on the
                                                  # floor at chassis height 0.30.
                                                  # Body sinks below alive threshold
                                                  # at rest, so the brain HAS to push
                                                  # knees toward straight (positive
                                                  # direction) to stand.  Clear
                                                  # learnable gradient: brain ↑ →
                                                  # knee less bent → body higher.
# v6.0 third-DOF — hip yaw (rotation around world +Y at the hip).
# Stage A: yaw is essentially LOCKED via tight joint limits with
# nearly-rigid softness.  The joint structure (chassis → yaw hinge →
# shoulder → pitch hinge → upper leg) is in place to match the
# PiCrawler 12-servo hardware, but the yaw DOF stays near 0 so the
# knee bend direction stays anatomically correct (no "twist" that
# would let the lower leg flop sideways through the knee axis).
# Stage B (when the brain has a 12-channel actor) will loosen these
# for walking — at that point yaw becomes a real actuated DOF.
const HIP_YAW_KP: float         = 0.05            # MILD PD; the hard work is done by
const HIP_YAW_KD: float         = 0.005           # the joint's tight limits, not PD.
const HIP_YAW_TARGET_ANGLE: float = 0.0           # radians; default no swivel
const HIP_YAW_LIMIT: float      = 0.04            # ~2.3° each side; effectively locked
const HIP_YAW_LIMIT_SOFTNESS: float = 0.99        # 1.0 = perfectly rigid limit;
                                                  # 0.99 gives just enough compliance
                                                  # to avoid solver oscillation
const LEG_ANGULAR_DAMP: float   = 4.0             # extra drag on leg segments to bleed off
                                                  # excess kinetic energy from joint-solver
                                                  # impulses (default Godot rb angular_damp
                                                  # is project-config dependent; pin a known
                                                  # value here so behaviour is reproducible).
# Collision layers: keep all body parts on a private layer that ONLY
# collides with the world (floor + walls).  Without this, the chassis
# and legs penetrate each other when the joint flexes hard — solver
# applies huge corrective impulses → "physics freakout" the user
# reported at leg_strength=2.0+.
const _LAYER_WORLD: int = 1   # floor + guard walls
const _LAYER_BODY:  int = 2   # chassis + every leg segment
# Anatomical joint ranges.  Tighter than the prior ±90° / ±108° so the
# solver doesn't have to settle wild end-of-range states.  Hip allowed
# ~70° fore/aft; knee biased to fold forward (negative angles = bent).
const HIP_LIMIT_UPPER: float    =  0.60       # v6.0.a.7: hip rotates around leg's
const HIP_LIMIT_LOWER: float    = -0.60       # long axis (twist); ±0.6 rad gives
                                              # brain enough swing to step but not
                                              # enough to fully invert the leg.
const HIP_LIMIT_SOFTNESS: float =  0.95       # near-rigid limit so excursions
                                              # past ±0.6 are bounced back hard.
                                              # PD alone isn't enough — the joint
                                              # solver's hard stop is what stops
                                              # tip-over progressions.
const KNEE_LIMIT_UPPER: float   =  0.10       # don't hyperextend
const KNEE_LIMIT_LOWER: float   = -1.80       # ~103° bend
const KNEE_LIMIT_SOFTNESS: float = 0.95       # near-rigid like hip — under load
                                              # the knee was permitting off-axis
                                              # rotation when the joint solver
                                              # couldn't resolve the constraint
                                              # in time.  0.95 is the same value
                                              # that fixed the hip excursions.
# Soft-limit params — moderate softness + light bias so hitting a limit
# decelerates smoothly rather than ejecting the leg.
const JOINT_LIMIT_SOFTNESS: float = 0.7
const JOINT_LIMIT_BIAS:     float = 0.2
const JOINT_LIMIT_RELAXATION: float = 1.0
# Chassis angular damping — bumped up to keep tip-over progression
# slow enough that brain torques can recover from small disturbances.
# At 0.8 (the previous value) random brain hip-twists at ls=1 dragged
# the chassis from upright to upside-down over ~300 ticks.  At 4.0
# the body still tips under sustained adversarial torques but takes
# 4× longer, giving recovery a chance.
const CHASSIS_ANGULAR_DAMP: float = 4.0
const NOMINAL_STAND_HEIGHT: float = 0.30          # used to normalise chassis_height proprio
const ALIVE_HEIGHT_THRESHOLD: float = 0.20        # m above ground to count "alive"
const ALIVE_TILT_THRESHOLD: float = 0.52          # radians (~30°)
const FAIL_HEIGHT_THRESHOLD: float = 0.18         # m — knee-collapse height; below this, body
                                                  # is propped on bent knees, not standing.
                                                  # Was 0.10 (chassis-flat-on-floor); knee-folded
                                                  # rest at ~0.105 sat in a dead zone where
                                                  # fell_over never fired and the brain spent
                                                  # the whole episode in a non-learning posture.
const FAIL_TILT_THRESHOLD: float = 1.05           # radians (~60°)

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
@onready var brain: OgmaBrain = $Brain

# Procedurally built bodies (populated in _ready)
var _chassis: RigidBody3D
var _upper_legs: Array[RigidBody3D] = []          # [FL, FR, RL, RR]
var _lower_legs: Array[RigidBody3D] = []
var _hip_joints: Array[HingeJoint3D] = []
var _knee_joints: Array[Joint3D] = []        # Generic6DOFJoint3D in spider mode (axis discipline)
                                             # HingeJoint3D in dog mode (legacy).
var _hip_initial_local_angle: Array[float] = []   # baseline for relative-angle proprio
var _knee_initial_local_angle: Array[float] = []
# v6.0 — per-leg hinge axes / rest poses for the 3-DOF leg chain.
# Yaw axis is world UP for every leg; hip pitch + knee axes are the
# leg-specific lateral direction (perpendicular to its outward yaw
# in horizontal plane).  All arrays length 4, indexed by leg index.
var _hip_axes:  Array[Vector3] = []
var _knee_axes: Array[Vector3] = []
var _yaw_axes:  Array[Vector3] = []
var _shoulders: Array[RigidBody3D] = []           # NEW intermediate body per leg
var _yaw_joints: Array[HingeJoint3D] = []         # chassis ↔ shoulder hinge
var _upper_initial_basis: Array[Basis] = []
var _lower_initial_basis: Array[Basis] = []
var _shoulder_initial_basis: Array[Basis] = []
var _previous_yaw_angle: Array[float] = [0.0, 0.0, 0.0, 0.0]
# Cached standing-pose targets used by _teleport_to_standing and the
# soft_blink lerp.  Filled at build time so the reset pose matches
# whatever morphology was selected.
var _upper_rest_xform: Array[Transform3D] = []
var _lower_rest_xform: Array[Transform3D] = []
var _shoulder_rest_xform: Array[Transform3D] = []
var _previous_hip_angle: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _previous_knee_angle: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Brain action channel indices (returned by register_action_channel).
# v6.0.a.6 — 8 channels: 4 hip + 4 knee.  Both joints rotate around the
# leg's lateral axis, so the brain has 2-DOF lift/lower control per leg.
var _action_idx_fl_hip:  int = -1
var _action_idx_fl_knee: int = -1
var _action_idx_fr_hip:  int = -1
var _action_idx_fr_knee: int = -1
var _action_idx_rl_hip:  int = -1
var _action_idx_rl_knee: int = -1
var _action_idx_rr_hip:  int = -1
var _action_idx_rr_knee: int = -1
# Convenience arrays indexed by leg (matches _hip_axes order).
var _action_idx_hip:  Array[int] = [-1, -1, -1, -1]
var _action_idx_knee: Array[int] = [-1, -1, -1, -1]

# Episode bookkeeping
var step_in_episode: int = 0
var episode_index: int   = 0
var episode_alive_ticks: int = 0
var tick_counter: int    = 0
var _accum: float = 0.0
var _done: bool = false
var _episode_fell: bool = false                   # set true the tick fell_over fires

# Soft-blink reset state.  During blink: brain still ticks (so its
# DescendingPredictor can observe the lift) but its motor output is
# discarded (zero torque applied), no body_alive events fire, and the
# chassis is kinematically lerped from its fallen pose toward the
# standing pose.  See reset_mode docs above.
var _blink_active:        bool = false
var _blink_ticks_left:    int  = 0
var _blink_start_pose:    Dictionary = {}         # cached pose at blink start; see _begin_soft_blink
var _instant_pause_tick:  bool = false            # one-shot: skip brain.tick() this tick
var _pending_manual_reset: bool = false           # set from _input; honoured at top of _step_one

# RNGs
var _init_rng: RandomNumberGenerator = RandomNumberGenerator.new()

# Leg labels for indexing + diag
const LEG_NAMES: Array = ["fl", "fr", "rl", "rr"]
const LEG_OFFSETS: Array = [
    Vector3( HIP_OFFSET_X, -CHASSIS_RADIUS * 0.5, -HIP_OFFSET_Z),  # FL: +X front, -Z left
    Vector3( HIP_OFFSET_X, -CHASSIS_RADIUS * 0.5,  HIP_OFFSET_Z),  # FR: +X front, +Z right
    Vector3(-HIP_OFFSET_X, -CHASSIS_RADIUS * 0.5, -HIP_OFFSET_Z),  # RL: -X rear,  -Z left
    Vector3(-HIP_OFFSET_X, -CHASSIS_RADIUS * 0.5,  HIP_OFFSET_Z),  # RR: -X rear,  +Z right
]

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
    if brain == null:
        push_error("QuadrupedBody: $Brain not found.")
        return

    # Physics oversampling — set BEFORE any body construction.  Env-var
    # overrides for headless harnesses (paired-seed sweeps).
    var env_phz: String = OS.get_environment("OGMA_PHYSICS_HZ")
    if env_phz != "":
        physics_hz = max(60, env_phz.to_int())
    var env_si: String = OS.get_environment("OGMA_SOLVER_ITERATIONS")
    if env_si != "":
        solver_iterations = max(4, env_si.to_int())
    Engine.physics_ticks_per_second = physics_hz
    ProjectSettings.set_setting("physics/3d/solver/solver_iterations", solver_iterations)

    # Seed precedence: ExperimentConfig (launcher) > OGMA_SEED env var > randomize
    var resolved_seed: int = ExperimentConfig.resolve_seed()
    var env_seed: String   = OS.get_environment("OGMA_SEED")
    if resolved_seed >= 0:
        _init_rng.seed = resolved_seed ^ 0x717561  # 'qua'
    elif env_seed != "":
        _init_rng.seed = env_seed.hash() ^ 0x717561
    else:
        _init_rng.randomize()

    var env_max_eps: String = OS.get_environment("OGMA_QUADRUPED_MAX_EPISODES")
    if env_max_eps != "":
        max_episodes = max(0, env_max_eps.to_int())
    var env_max_steps: String = OS.get_environment("OGMA_QUADRUPED_MAX_STEPS")
    if env_max_steps != "":
        max_steps = max(1, env_max_steps.to_int())
    var env_diag: String = OS.get_environment("OGMA_QUADRUPED_DIAG_INTERVAL")
    if env_diag != "":
        diag_interval_ticks = max(0, env_diag.to_int())
    var env_task: String = OS.get_environment("OGMA_QUADRUPED_TASK")
    if env_task != "":
        task_mode = env_task
    var env_mcp: String = OS.get_environment("OGMA_QUADRUPED_MC_PERIOD")
    if env_mcp != "":
        mc_episode_period = max(0, env_mcp.to_int())

    # Resolve reset_mode through the standard ExperimentConfig precedence:
    #   launcher selection > OGMA_RESET_MODE env > @export default.
    var resolved_reset: String = ExperimentConfig.resolve_reset_mode(reset_mode)
    if resolved_reset in _RESET_MODE_CHOICES:
        reset_mode = resolved_reset
    else:
        push_warning("QuadrupedBody: reset_mode=%s not in %s — keeping %s" % [
            resolved_reset, str(_RESET_MODE_CHOICES), reset_mode])

    var resolved_shape: String = ExperimentConfig.resolve_chassis_shape(chassis_shape)
    if resolved_shape in _CHASSIS_SHAPE_CHOICES:
        chassis_shape = resolved_shape
    else:
        push_warning("QuadrupedBody: chassis_shape=%s not in %s — keeping %s" % [
            resolved_shape, str(_CHASSIS_SHAPE_CHOICES), chassis_shape])

    leg_strength = ExperimentConfig.resolve_leg_strength(leg_strength)
    if leg_strength <= 0.0:
        push_warning("QuadrupedBody: leg_strength must be > 0, got %.3f — clamping to 0.01" % leg_strength)
        leg_strength = 0.01

    var resolved_morph: String = ExperimentConfig.resolve_body_morphology(body_morphology)
    if resolved_morph in _BODY_MORPHOLOGY_CHOICES:
        body_morphology = resolved_morph
    else:
        push_warning("QuadrupedBody: body_morphology=%s not in %s — keeping %s" % [
            resolved_morph, str(_BODY_MORPHOLOGY_CHOICES), body_morphology])

    # Brain interface contract — declare sources, sinks, events BEFORE setup.
    brain.register_source(
        "IMU", "reality.proprio.imu",
        "float32[4]: sin/cos yaw, fwd_v/3.0, ang_v/π — chassis kinematics", true)
    brain.register_source(
        "Joints", "reality.proprio.joints",
        "float32[12]: 4 yaw + 4 hip pitch + 4 knee angles, each clamped/normalised to [-1,+1]", true)
    brain.register_source(
        "FootContact", "reality.proprio.foot_contact",
        "float32[4]: 0/1 per leg (FL,FR,RL,RR) — true when lower leg touches ground", true)
    brain.register_source(
        "ChassisHeight", "reality.proprio.chassis_height",
        "float32[1]: chassis Y / NOMINAL_STAND_HEIGHT", true)

    # v6.0.a.6 — 8 brain channels (hip + knee per leg)
    _action_idx_fl_hip  = brain.register_action_channel("fl_hip",  "action.fl_hip")
    _action_idx_fl_knee = brain.register_action_channel("fl_knee", "action.fl_knee")
    _action_idx_fr_hip  = brain.register_action_channel("fr_hip",  "action.fr_hip")
    _action_idx_fr_knee = brain.register_action_channel("fr_knee", "action.fr_knee")
    _action_idx_rl_hip  = brain.register_action_channel("rl_hip",  "action.rl_hip")
    _action_idx_rl_knee = brain.register_action_channel("rl_knee", "action.rl_knee")
    _action_idx_rr_hip  = brain.register_action_channel("rr_hip",  "action.rr_hip")
    _action_idx_rr_knee = brain.register_action_channel("rr_knee", "action.rr_knee")
    _action_idx_hip  = [_action_idx_fl_hip,  _action_idx_fr_hip,  _action_idx_rl_hip,  _action_idx_rr_hip]
    _action_idx_knee = [_action_idx_fl_knee, _action_idx_fr_knee, _action_idx_rl_knee, _action_idx_rr_knee]

    brain.register_event("BodyAlive", "events.body_alive", "per-tick survival pulse → drive valence")
    brain.register_event("BodyLift",  "events.body_lift",  "per-tick continuous lift reward (intensity = max(0, chassis_y - 0.05)) → aligned dopamine pulse")
    brain.register_event("FellOver",  "events.fell_over",  "episode terminated by tipping/sinking")
    brain.register_event("EpisodeEnd","events.episode_end","fired by _finish_episode → Premotor MC trajectory finalize")

    # Resolve config: ExperimentConfig > OGMA_QUADRUPED_CONFIG > @export default
    config_path = ExperimentConfig.resolve_config("OGMA_QUADRUPED_CONFIG", config_path)
    # v6.0 — propagate the body's resolved seed into the brain so every
    # module's master_seed is derived from it.  Without this, OGMA_SEED
    # would only seed the body's _init_rng and the brain's stochastic
    # streams would stay config-locked → identical trajectories across
    # seeds.  Pass 0 to disable (keeps config seeds — golden-replay
    # behaviour).
    if _init_rng.seed != 0:
        brain.set_master_seed(int(_init_rng.seed))
    if not brain.setup(config_path):
        push_error("QuadrupedBody: brain.setup() failed: %s" % config_path)
        return

    _build_world()
    _build_body()

    # Re-target the orbit camera (if present) to the procedurally-built
    # chassis so it follows the dog's position as it moves.  Path-based
    # binding in the .tscn would fail because _chassis doesn't exist at
    # scene-load time.
    var cam: Node = get_tree().get_root().find_child("Camera3D", true, false)
    if cam != null and cam.has_method("set_target"):
        cam.call("set_target", _chassis)

    print(JSON.stringify({
        "event": "READY",
        "body": "quadruped",
        "config": config_path,
        "seed": _init_rng.seed,
        "max_steps": max_steps,
        "max_episodes": max_episodes,
        "reset_mode": reset_mode,
        "chassis_shape": chassis_shape,
        "leg_strength": leg_strength,
        "body_morphology": body_morphology,
        "physics_hz": physics_hz,
        "solver_iterations": solver_iterations,
    }))

# ---------------------------------------------------------------------------
# World + body construction (procedural, run once in _ready)
# ---------------------------------------------------------------------------

func _build_world() -> void:
    # Ground plane.  Smaller than before (40 → 8 m on each side) so the
    # bounded play area fits within the camera's natural orbit framing
    # and the guard walls are visible at default zoom.
    var ground_size: float = 8.0
    var ground := StaticBody3D.new()
    var ground_shape := CollisionShape3D.new()
    var box := BoxShape3D.new()
    box.size = Vector3(ground_size, 1.0, ground_size)
    ground_shape.shape = box
    ground.add_child(ground_shape)
    var ground_mesh := MeshInstance3D.new()
    var ground_box_mesh := BoxMesh.new()
    ground_box_mesh.size = box.size
    ground_mesh.mesh = ground_box_mesh
    var ground_mat := StandardMaterial3D.new()
    ground_mat.albedo_color = Color(0.20, 0.22, 0.18, 1.0)
    ground_mesh.set_surface_override_material(0, ground_mat)
    ground.add_child(ground_mesh)
    ground.position = Vector3(0, -0.5, 0)
    add_child(ground)

    # Low guard walls around the play area perimeter.  Tall enough that a
    # rolling chassis can't simply tip over them (~3× chassis radius), thin
    # enough that they don't dominate the visual.
    var wall_h: float = 0.50
    var wall_t: float = 0.10
    var wall_mat := StandardMaterial3D.new()
    wall_mat.albedo_color = Color(0.30, 0.32, 0.36, 1.0)
    var half: float = ground_size * 0.5
    var walls := [
        {"size": Vector3(ground_size + wall_t * 2.0, wall_h, wall_t),
         "pos":  Vector3(0, wall_h * 0.5, +half + wall_t * 0.5)},   # +Z wall
        {"size": Vector3(ground_size + wall_t * 2.0, wall_h, wall_t),
         "pos":  Vector3(0, wall_h * 0.5, -half - wall_t * 0.5)},   # -Z wall
        {"size": Vector3(wall_t, wall_h, ground_size),
         "pos":  Vector3(+half + wall_t * 0.5, wall_h * 0.5, 0)},   # +X wall
        {"size": Vector3(wall_t, wall_h, ground_size),
         "pos":  Vector3(-half - wall_t * 0.5, wall_h * 0.5, 0)},   # -X wall
    ]
    for w in walls:
        var wall := StaticBody3D.new()
        var w_shape := CollisionShape3D.new()
        var w_box := BoxShape3D.new()
        w_box.size = w["size"]
        w_shape.shape = w_box
        wall.add_child(w_shape)
        var w_mesh := MeshInstance3D.new()
        var w_box_mesh := BoxMesh.new()
        w_box_mesh.size = w["size"]
        w_mesh.mesh = w_box_mesh
        w_mesh.set_surface_override_material(0, wall_mat)
        wall.add_child(w_mesh)
        wall.position = w["pos"]
        add_child(wall)

func _build_body() -> void:
    # 1. Chassis RigidBody3D at NOMINAL_STAND_HEIGHT.  Shape selected by
    #    `chassis_shape` (launcher / OGMA_CHASSIS_SHAPE).  Half-cylinder
    #    (flat down, curved up) rolls naturally and self-recovers from
    #    falls; box is the original stable comparison baseline.
    _chassis = RigidBody3D.new()
    _chassis.mass = CHASSIS_MASS
    _chassis.position = Vector3(0, NOMINAL_STAND_HEIGHT, 0)
    _chassis.angular_damp = CHASSIS_ANGULAR_DAMP
    _chassis.collision_layer = _LAYER_BODY    # on the body-private layer
    _chassis.collision_mask  = _LAYER_WORLD   # collides only with floor/walls, not legs
    var collision_node := CollisionShape3D.new()
    var mesh_node := MeshInstance3D.new()
    var chassis_mat := StandardMaterial3D.new()
    chassis_mat.albedo_color = Color(0.65, 0.55, 0.40, 1.0)
    if chassis_shape == "box":
        var box := BoxShape3D.new()
        box.size = Vector3(CHASSIS_LENGTH, CHASSIS_RADIUS, CHASSIS_RADIUS * 2.0)
        collision_node.shape = box
        var bm := BoxMesh.new()
        bm.size = box.size
        mesh_node.mesh = bm
    else:   # half_cylinder (default)
        var hull := ConvexPolygonShape3D.new()
        hull.points = _make_half_cylinder_hull_points(CHASSIS_LENGTH, CHASSIS_RADIUS, 12)
        collision_node.shape = hull
        mesh_node.mesh = _make_half_cylinder_mesh(CHASSIS_LENGTH, CHASSIS_RADIUS, 16)
    mesh_node.set_surface_override_material(0, chassis_mat)
    _chassis.add_child(collision_node)
    _chassis.add_child(mesh_node)
    add_child(_chassis)

    # 2-3. For each leg: upper + lower segments + hip + knee joints.
    # Branch on morphology — both build paths populate the same arrays
    # (_upper_legs, _lower_legs, _hip_joints, _knee_joints, _hip_axes,
    # _knee_axes, _upper_rest_xform, _lower_rest_xform) so the tick
    # loop, torque application, and reset code are morphology-agnostic.
    for i in range(4):
        if body_morphology == "spider":
            _build_spider_leg(i)
        else:
            _build_dog_leg(i)

# Generate the convex hull vertex set for a half-cylinder oriented along
# X with the flat face at chassis-local Y = -R/2 and the curved face
# rising to Y = +R/2.  N is the arc subdivision count — vertices along
# each semicircle (θ ∈ [0, π]) at X = ±L/2.  Total: 2 × (N+1) points,
# which gives a smooth convex shape for collisions.
func _make_half_cylinder_hull_points(L: float, R: float, N: int) -> PackedVector3Array:
    var pts := PackedVector3Array()
    for i in range(N + 1):
        var theta: float = (PI * float(i)) / float(N)
        var y: float = -R * 0.5 + R * sin(theta)
        var z: float = R * cos(theta)
        pts.append(Vector3(+L * 0.5, y, z))
        pts.append(Vector3(-L * 0.5, y, z))
    return pts

# Generate an ArrayMesh visual matching the half-cylinder hull.  Set
# normals explicitly per-vertex instead of relying on SurfaceTool's
# generate_normals() — that path averages normals across shared
# vertex positions and produces back-culled faces on parts of the
# curve.  Explicit normals are unambiguous and let us hand-pick the
# winding to satisfy Godot's CCW-from-outside front-face rule.
func _make_half_cylinder_mesh(L: float, R: float, N: int) -> ArrayMesh:
    var st := SurfaceTool.new()
    st.begin(Mesh.PRIMITIVE_TRIANGLES)
    var half_L: float = L * 0.5
    var bottom_y: float = -R * 0.5

    # Curved top — quad strips between θ_i and θ_{i+1} along each end.
    # The outward normal at parameter θ is (0, sin θ, cos θ) — points
    # straight up at θ=π/2 (apex) and straight ±Z at the diameter edges.
    # CCW winding when viewed from outside (above the curve): a → c → b
    # for the +X side, a → d → c for the -X mirror.
    for i in range(N):
        var t1: float = (PI * float(i))     / float(N)
        var t2: float = (PI * float(i + 1)) / float(N)
        var y1: float = bottom_y + R * sin(t1); var z1: float = R * cos(t1)
        var y2: float = bottom_y + R * sin(t2); var z2: float = R * cos(t2)
        var n1 := Vector3(0.0, sin(t1), cos(t1))
        var n2 := Vector3(0.0, sin(t2), cos(t2))
        var a := Vector3(-half_L, y1, z1)
        var b := Vector3(+half_L, y1, z1)
        var c := Vector3(+half_L, y2, z2)
        var d := Vector3(-half_L, y2, z2)
        st.set_normal(n1); st.add_vertex(a)
        st.set_normal(n2); st.add_vertex(d)
        st.set_normal(n2); st.add_vertex(c)
        st.set_normal(n1); st.add_vertex(a)
        st.set_normal(n2); st.add_vertex(c)
        st.set_normal(n1); st.add_vertex(b)

    # Flat bottom at Y = -R/2, outward normal = -Y.  CCW from below.
    var nb := Vector3(0, -1, 0)
    var p00 := Vector3(-half_L, bottom_y, -R)
    var p10 := Vector3(+half_L, bottom_y, -R)
    var p11 := Vector3(+half_L, bottom_y, +R)
    var p01 := Vector3(-half_L, bottom_y, +R)
    st.set_normal(nb); st.add_vertex(p00)
    st.set_normal(nb); st.add_vertex(p11)
    st.set_normal(nb); st.add_vertex(p10)
    st.set_normal(nb); st.add_vertex(p00)
    st.set_normal(nb); st.add_vertex(p01)
    st.set_normal(nb); st.add_vertex(p11)

    # End-cap semicircles — outward normals = ±X.  CCW from outside.
    for sign in [+1.0, -1.0]:
        var x: float = sign * half_L
        var ne := Vector3(sign, 0, 0)
        var ctr := Vector3(x, bottom_y, 0.0)
        for i in range(N):
            var t1: float = (PI * float(i))     / float(N)
            var t2: float = (PI * float(i + 1)) / float(N)
            var p1 := Vector3(x, bottom_y + R * sin(t1), R * cos(t1))
            var p2 := Vector3(x, bottom_y + R * sin(t2), R * cos(t2))
            if sign > 0.0:
                st.set_normal(ne); st.add_vertex(ctr)
                st.set_normal(ne); st.add_vertex(p1)
                st.set_normal(ne); st.add_vertex(p2)
            else:
                st.set_normal(ne); st.add_vertex(ctr)
                st.set_normal(ne); st.add_vertex(p2)
                st.set_normal(ne); st.add_vertex(p1)
    return st.commit()

# Build the shoulder body + yaw hinge for one leg.  Returns the
# shoulder RigidBody3D so the morphology-specific code can attach
# the hip pitch hinge to it instead of to the chassis.  Yaw axis is
# world +Y for every leg; the body-side PD in _step_one pulls it
# back toward HIP_YAW_TARGET_ANGLE.
func _build_shoulder_and_yaw(leg_index: int, hip_world_pos: Vector3) -> RigidBody3D:
    var shoulder := RigidBody3D.new()
    shoulder.mass = SHOULDER_MASS
    shoulder.position = hip_world_pos
    shoulder.angular_damp = LEG_ANGULAR_DAMP
    shoulder.collision_layer = _LAYER_BODY
    shoulder.collision_mask  = _LAYER_WORLD
    var sh_shape := CollisionShape3D.new()
    var sh_sphere := SphereShape3D.new()
    sh_sphere.radius = SHOULDER_RADIUS
    sh_shape.shape = sh_sphere
    shoulder.add_child(sh_shape)
    var sh_mesh := MeshInstance3D.new()
    var sm := SphereMesh.new()
    sm.radius = SHOULDER_RADIUS
    sm.height = SHOULDER_RADIUS * 2.0
    sh_mesh.mesh = sm
    var sh_mat := StandardMaterial3D.new()
    sh_mat.albedo_color = Color(0.55, 0.50, 0.40, 1.0)
    sh_mesh.set_surface_override_material(0, sh_mat)
    shoulder.add_child(sh_mesh)
    add_child(shoulder)

    # Yaw hinge — parented under the CHASSIS so the joint's frame
    # follows chassis rotation (matches real hardware: hip servo bolted
    # to the body).  Default HingeJoint3D hinge axis = local +X;
    # rotation_degrees=(0,0,90) maps local +X → local +Y = chassis's
    # local +Y = world +Y when chassis is upright.
    var yaw_joint := HingeJoint3D.new()
    _chassis.add_child(yaw_joint)
    yaw_joint.global_transform = Transform3D(Basis().rotated(Vector3.FORWARD, PI * 0.5),
                                              hip_world_pos)
    yaw_joint.set_node_a(_chassis.get_path())
    yaw_joint.set_node_b(shoulder.get_path())
    yaw_joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER,         HIP_YAW_LIMIT)
    yaw_joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER,        -HIP_YAW_LIMIT)
    # Yaw uses harder limit softness than pitch/knee — see HIP_YAW_LIMIT_SOFTNESS
    # docstring for rationale (stage-A lock-out of the yaw DOF).
    yaw_joint.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS,      HIP_YAW_LIMIT_SOFTNESS)
    yaw_joint.set_param(HingeJoint3D.PARAM_LIMIT_BIAS,          JOINT_LIMIT_BIAS)
    yaw_joint.set_param(HingeJoint3D.PARAM_LIMIT_RELAXATION,    JOINT_LIMIT_RELAXATION)
    yaw_joint.set_flag(HingeJoint3D.FLAG_USE_LIMIT,            true)
    yaw_joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR,         false)

    _shoulders.append(shoulder)
    _yaw_joints.append(yaw_joint)
    _yaw_axes.append(Vector3.UP)
    _shoulder_initial_basis.append(Basis.IDENTITY)
    _shoulder_rest_xform.append(Transform3D(Basis.IDENTITY, hip_world_pos))
    return shoulder

func _build_dog_leg(leg_index: int) -> void:
    var hip_world_pos: Vector3 = _chassis.position + LEG_OFFSETS[leg_index]
    var upper_center: Vector3 = hip_world_pos + Vector3(0, -UPPER_LEG_LEN * 0.5, 0)
    var lower_center: Vector3 = hip_world_pos + Vector3(0, -UPPER_LEG_LEN - LOWER_LEG_LEN * 0.5, 0)
    # v6.0.a.7 — hip rotates around the LEG's longitudinal axis (the leg's
    # own length direction), knee around lateral.  For dog mode the leg
    # hangs straight down, so leg's long axis = world DOWN (= -Y).
    var leg_long_axis: Vector3 = Vector3.DOWN          # the leg's own length direction
    var lateral_axis:  Vector3 = Vector3(0, 0, 1)      # perpendicular, in horizontal plane
    var shoulder: RigidBody3D = _build_shoulder_and_yaw(leg_index, hip_world_pos)

    # Upper leg
    var upper := RigidBody3D.new()
    upper.mass = UPPER_LEG_MASS
    upper.position = upper_center
    upper.angular_damp = LEG_ANGULAR_DAMP
    upper.collision_layer = _LAYER_BODY
    upper.collision_mask  = _LAYER_WORLD
    var upper_shape := CollisionShape3D.new()
    var upper_box := BoxShape3D.new()
    upper_box.size = Vector3(LEG_RADIUS * 2.0, UPPER_LEG_LEN, LEG_RADIUS * 2.0)
    upper_shape.shape = upper_box
    upper.add_child(upper_shape)
    var upper_mesh := MeshInstance3D.new()
    var um := BoxMesh.new()
    um.size = upper_box.size
    upper_mesh.mesh = um
    var upper_mat := StandardMaterial3D.new()
    upper_mat.albedo_color = Color(0.45, 0.40, 0.35, 1.0)
    upper_mesh.set_surface_override_material(0, upper_mat)
    upper.add_child(upper_mesh)
    add_child(upper)
    _upper_legs.append(upper)

    # Hip joint — HingeJoint3D rotating about the body's LATERAL axis (world Z)
    # so the leg swings forward/backward in the sagittal X-Y plane.  Godot's
    # HingeJoint3D defaults its hinge axis to local +X; rotating the joint
    # node -90° around Y maps local +X → world +Z, which is what we want.
    # (Pre-fix this was (0,0,90), which made the hinge axis world +Y — legs
    # could only swivel like a swing-out cabinet door and hip torque around
    # X was absorbed by the joint constraint.  Body moved only via contact-
    # force compliance, producing seed-independent drift.)
    # Pitch hinge — parented under the SHOULDER so its hinge axis
    # follows shoulder rotation (so if shoulder yaws, the pitch axis
    # yaws with it — true kinematic chain, matches a servo bolted to
    # the previous servo's output horn).  Without this, the pitch axis
    # was world-fixed even when shoulder yawed, and the solver
    # compromise gave the upper leg a fake twist DOF.
    # Hip joint — hinge axis = leg's longitudinal axis (= world DOWN
    # for the dog).  Build the joint basis explicitly: local +X must
    # equal leg_long_axis in world frame at setup.
    var hip_basis := Basis()
    hip_basis.x = leg_long_axis
    hip_basis.z = lateral_axis
    hip_basis.y = lateral_axis.cross(leg_long_axis).normalized()
    hip_basis = hip_basis.orthonormalized()
    var hip := HingeJoint3D.new()
    shoulder.add_child(hip)
    hip.global_transform = Transform3D(hip_basis, hip_world_pos)
    hip.set_node_a(shoulder.get_path())
    hip.set_node_b(upper.get_path())
    # Anatomical hip range (fore/aft) with soft limits.  Tighter than the
    # original ±90° so the solver doesn't spend cycles resolving wild
    # end-of-range states.  Softness + relaxation makes limit-hit
    # contacts decelerate smoothly rather than eject the leg.
    hip.set_param(HingeJoint3D.PARAM_LIMIT_UPPER,         HIP_LIMIT_UPPER)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_LOWER,         HIP_LIMIT_LOWER)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS,      HIP_LIMIT_SOFTNESS)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_BIAS,          JOINT_LIMIT_BIAS)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_RELAXATION,    JOINT_LIMIT_RELAXATION)
    hip.set_flag(HingeJoint3D.FLAG_USE_LIMIT,            true)
    hip.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR,         false)  # we apply torque directly
    _hip_joints.append(hip)
    _hip_initial_local_angle.append(0.0)

    # Lower leg
    var lower := RigidBody3D.new()
    lower.mass = LOWER_LEG_MASS
    lower.position = lower_center
    lower.angular_damp = LEG_ANGULAR_DAMP
    lower.collision_layer = _LAYER_BODY
    lower.collision_mask  = _LAYER_WORLD
    var lower_shape := CollisionShape3D.new()
    var lower_box := BoxShape3D.new()
    lower_box.size = Vector3(LEG_RADIUS * 2.0, LOWER_LEG_LEN, LEG_RADIUS * 2.0)
    lower_shape.shape = lower_box
    lower.add_child(lower_shape)
    var lower_mesh := MeshInstance3D.new()
    var lm := BoxMesh.new()
    lm.size = lower_box.size
    lower_mesh.mesh = lm
    var lower_mat := StandardMaterial3D.new()
    lower_mat.albedo_color = Color(0.30, 0.28, 0.25, 1.0)
    lower_mesh.set_surface_override_material(0, lower_mat)
    lower.add_child(lower_mesh)
    add_child(lower)
    _lower_legs.append(lower)

    # Knee joint — hinge axis = lateral (perpendicular to leg's long
    # axis, in horizontal plane).  Bends the leg in its sagittal plane.
    var knee_basis := Basis()
    knee_basis.x = lateral_axis
    knee_basis.z = leg_long_axis
    knee_basis.y = leg_long_axis.cross(lateral_axis).normalized()
    knee_basis = knee_basis.orthonormalized()
    var knee := HingeJoint3D.new()
    upper.add_child(knee)
    knee.global_transform = Transform3D(knee_basis,
        hip_world_pos + Vector3(0, -UPPER_LEG_LEN, 0))
    knee.set_node_a(upper.get_path())
    knee.set_node_b(lower.get_path())
    knee.set_param(HingeJoint3D.PARAM_LIMIT_UPPER,  KNEE_LIMIT_UPPER)
    knee.set_param(HingeJoint3D.PARAM_LIMIT_LOWER,  KNEE_LIMIT_LOWER)
    knee.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS,   KNEE_LIMIT_SOFTNESS)
    knee.set_param(HingeJoint3D.PARAM_LIMIT_BIAS,       JOINT_LIMIT_BIAS)
    knee.set_param(HingeJoint3D.PARAM_LIMIT_RELAXATION, JOINT_LIMIT_RELAXATION)
    knee.set_flag(HingeJoint3D.FLAG_USE_LIMIT,            true)
    knee.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR,         false)
    _knee_joints.append(knee)
    _knee_initial_local_angle.append(0.0)

    # Morphology-agnostic per-leg state (used by tick + reset).
    _hip_axes.append(leg_long_axis)
    _knee_axes.append(lateral_axis)
    _upper_initial_basis.append(Basis.IDENTITY)
    _lower_initial_basis.append(Basis.IDENTITY)
    _upper_rest_xform.append(Transform3D(Basis.IDENTITY, upper_center))
    _lower_rest_xform.append(Transform3D(Basis.IDENTITY, lower_center))

# ---------------------------------------------------------------------------
# Spider leg — legs splay outward from chassis corners.  Statically stable
# because the feet form a wide support polygon (≈ 0.7 × 0.45 m at default
# offsets) with the chassis COG roughly at hip height.  Each leg has its
# own hinge axis (perpendicular to the leg's outward direction in the
# horizontal plane), stored per-leg so the same tick/torque/reset code
# handles both morphologies.
# ---------------------------------------------------------------------------
func _build_spider_leg(leg_index: int) -> void:
    var hip_offset: Vector3 = LEG_OFFSETS[leg_index]
    var hip_world_pos: Vector3 = _chassis.position + hip_offset
    # v6.0.a.4 — shoulder + yaw hinge precedes the pitch hinge.
    var shoulder: RigidBody3D = _build_shoulder_and_yaw(leg_index, hip_world_pos)

    # Horizontal outward direction (radial from chassis centre in the
    # X-Z plane).  Independent of the leg's Y offset.
    var outward: Vector3 = Vector3(hip_offset.x, 0, hip_offset.z).normalized()
    # Lateral axis perpendicular to outward, in the horizontal plane.
    # This is the hinge axis for both the hip and knee joints of this
    # leg — rotation about it swings the leg up/down in its own
    # vertical plane.  outward × UP = lateral (right-hand rule).
    var lateral: Vector3 = outward.cross(Vector3.UP).normalized()

    # Upper-leg direction: tilted SPIDER_UPPER_SLOPE radians below
    # horizontal, in the plane spanned by outward + DOWN.
    var upper_dir: Vector3 = (outward * cos(SPIDER_UPPER_SLOPE) \
        + Vector3.DOWN * sin(SPIDER_UPPER_SLOPE)).normalized()
    var upper_center: Vector3 = hip_world_pos + upper_dir * (UPPER_LEG_LEN * 0.5)

    # Upper leg basis: local +Y = leg long axis (matches BoxMesh long Y),
    # local +X = hinge (lateral) axis, local +Z = right-hand complement.
    var upper_basis := Basis()
    upper_basis.x = lateral
    upper_basis.y = upper_dir
    upper_basis.z = lateral.cross(upper_dir).normalized()
    upper_basis = upper_basis.orthonormalized()

    var upper := RigidBody3D.new()
    upper.mass = UPPER_LEG_MASS
    upper.transform = Transform3D(upper_basis, upper_center)
    upper.angular_damp = LEG_ANGULAR_DAMP
    upper.collision_layer = _LAYER_BODY
    upper.collision_mask  = _LAYER_WORLD
    var upper_shape := CollisionShape3D.new()
    var upper_box := BoxShape3D.new()
    upper_box.size = Vector3(LEG_RADIUS * 2.0, UPPER_LEG_LEN, LEG_RADIUS * 2.0)
    upper_shape.shape = upper_box
    upper.add_child(upper_shape)
    var upper_mesh := MeshInstance3D.new()
    var um := BoxMesh.new()
    um.size = upper_box.size
    upper_mesh.mesh = um
    var upper_mat := StandardMaterial3D.new()
    upper_mat.albedo_color = Color(0.45, 0.40, 0.35, 1.0)
    upper_mesh.set_surface_override_material(0, upper_mat)
    upper.add_child(upper_mesh)
    add_child(upper)
    _upper_legs.append(upper)

    # v6.0.a.7 — hip rotates around the LEG'S OWN LONGITUDINAL AXIS
    # (upper_dir = the leg's outward+down direction in spider mode),
    # not the lateral.  Knee stays around lateral.  This matches
    # picrawler hardware: the body-mounted servo's output shaft points
    # along the leg, and the leg can twist around its own length.
    var hip_basis := Basis()
    hip_basis.x = upper_dir            # hinge axis = leg long axis
    hip_basis.z = lateral              # second perpendicular
    hip_basis.y = lateral.cross(upper_dir).normalized()
    hip_basis = hip_basis.orthonormalized()

    var hip := HingeJoint3D.new()
    shoulder.add_child(hip)
    hip.global_transform = Transform3D(hip_basis, hip_world_pos)
    # Kinematic chain: chassis→[yaw]→shoulder→[hip-along-leg]→upper.
    hip.set_node_a(shoulder.get_path())
    hip.set_node_b(upper.get_path())
    hip.set_param(HingeJoint3D.PARAM_LIMIT_UPPER,         HIP_LIMIT_UPPER)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_LOWER,         HIP_LIMIT_LOWER)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS,      HIP_LIMIT_SOFTNESS)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_BIAS,          JOINT_LIMIT_BIAS)
    hip.set_param(HingeJoint3D.PARAM_LIMIT_RELAXATION,    JOINT_LIMIT_RELAXATION)
    hip.set_flag(HingeJoint3D.FLAG_USE_LIMIT,            true)
    hip.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR,         false)
    _hip_joints.append(hip)
    _hip_initial_local_angle.append(0.0)

    # Elbow position at the far end of the upper leg.
    var elbow_world: Vector3 = hip_world_pos + upper_dir * UPPER_LEG_LEN
    var lower_center: Vector3 = elbow_world + Vector3.DOWN * (LOWER_LEG_LEN * 0.5)
    var lower_basis: Basis = Basis.IDENTITY   # lower leg points straight down (world -Y)

    var lower := RigidBody3D.new()
    lower.mass = LOWER_LEG_MASS
    lower.transform = Transform3D(lower_basis, lower_center)
    lower.angular_damp = LEG_ANGULAR_DAMP
    lower.collision_layer = _LAYER_BODY
    lower.collision_mask  = _LAYER_WORLD
    var lower_shape := CollisionShape3D.new()
    var lower_box := BoxShape3D.new()
    lower_box.size = Vector3(LEG_RADIUS * 2.0, LOWER_LEG_LEN, LEG_RADIUS * 2.0)
    lower_shape.shape = lower_box
    lower.add_child(lower_shape)
    var lower_mesh := MeshInstance3D.new()
    var lm := BoxMesh.new()
    lm.size = lower_box.size
    lower_mesh.mesh = lm
    var lower_mat := StandardMaterial3D.new()
    lower_mat.albedo_color = Color(0.30, 0.28, 0.25, 1.0)
    lower_mesh.set_surface_override_material(0, lower_mat)
    lower.add_child(lower_mesh)
    add_child(lower)
    _lower_legs.append(lower)

    # Knee joint — Generic6DOFJoint3D with five axes locked, one (knee
    # bend, around lateral) freed.  HingeJoint3D was failing under load:
    # knees rotated past their hinge axis (knee_a hit ±1.55 vs +0.10
    # limit) because the constraint solver couldn't enforce the
    # axis-perpendicular constraints fast enough.  Generic6DOF gives
    # us per-axis control and Bullet/Jolt enforces the linear/angular
    # locks more rigidly.  Joint basis: x=lateral (free axis), z=upper_dir.
    var knee_basis := Basis()
    knee_basis.x = lateral
    knee_basis.z = upper_dir
    knee_basis.y = upper_dir.cross(lateral).normalized()
    knee_basis = knee_basis.orthonormalized()
    var knee := Generic6DOFJoint3D.new()
    upper.add_child(knee)
    knee.global_transform = Transform3D(knee_basis, elbow_world)
    knee.set_node_a(upper.get_path())
    knee.set_node_b(lower.get_path())
    # All linear axes locked (no translation between upper and lower).
    for ax in [Vector3.AXIS_X, Vector3.AXIS_Y, Vector3.AXIS_Z]:
        knee.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
        knee.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
        knee.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
    knee.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
    knee.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
    knee.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
    knee.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
    knee.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
    knee.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
    # Angular: free X (knee bend), lock Y and Z.
    knee.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
    knee.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
    knee.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
    knee.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, KNEE_LIMIT_LOWER)
    knee.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, KNEE_LIMIT_UPPER)
    knee.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
    knee.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
    knee.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
    knee.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
    _knee_joints.append(knee)
    _knee_initial_local_angle.append(0.0)

    _hip_axes.append(upper_dir)
    _knee_axes.append(lateral)
    _upper_initial_basis.append(upper_basis)
    _lower_initial_basis.append(lower_basis)
    _upper_rest_xform.append(Transform3D(upper_basis, upper_center))
    _lower_rest_xform.append(Transform3D(lower_basis, lower_center))

# ---------------------------------------------------------------------------
# Physics loop
# ---------------------------------------------------------------------------

func _physics_process(delta: float) -> void:
    if brain == null or not brain.is_brain_ready():
        return
    _accum += delta
    if _accum > 0.2:
        _accum = 0.2
    while _accum >= TAU:
        _step_one()
        _accum -= TAU

# v6.0 — manual reset hotkey.  Spacebar snaps the body back to standing
# pose regardless of the active reset_mode.  Primary use case: continuous
# mode, where no auto-reset fires and the user wants to start a new
# trajectory without restarting the scene.  Also useful in soft_blink to
# skip past an in-progress lift, or in instant_pause to re-arm after a
# completed reset.  Brain.tick() is skipped on the snap tick (same one-
# shot pattern instant_pause uses) so the discontinuous proprio jump
# doesn't get folded into a learning step.
func _unhandled_input(event: InputEvent) -> void:
    if not (event is InputEventKey and event.pressed and not event.echo):
        return
    if (event as InputEventKey).keycode == KEY_SPACE:
        request_manual_reset()

func request_manual_reset() -> void:
    # Don't perform the teleport here — _unhandled_input runs in the
    # idle frame, where direct RigidBody3D transform writes get
    # overridden by the next physics tick.  Just queue the request;
    # _step_one consumes the flag at the top of the next physics
    # frame, in the proper freeze/unfreeze context.
    _pending_manual_reset = true

func _step_one() -> void:
    if _done:
        return
    # Manual reset (spacebar): runs at the start of the physics frame so
    # freeze→write→unfreeze actually takes effect.  Skip the blink path
    # if active (manual reset wins over a partial lift).
    if _pending_manual_reset:
        _pending_manual_reset = false
        if _blink_active:
            _end_soft_blink()
        _do_hard_reset()
        _instant_pause_tick = true   # skip the next brain.tick() so the
                                      # discontinuous proprio jump isn't
                                      # folded into a learning step
        step_in_episode = 0
        episode_alive_ticks = 0
        _episode_fell = false
        print(JSON.stringify({
            "event": "MANUAL_RESET",
            "tick":  tick_counter,
            "reset_mode": reset_mode,
        }))
    # Soft-blink mode: hand control over to the lift coroutine until the
    # blink completes.  Brain still ticks inside _step_blink so its
    # DescendingPredictor sees the chassis re-acquire upright pose, but
    # it has no motor authority and no body_alive reward arrives.
    if _blink_active:
        _step_blink()
        return
    tick_counter   += 1
    step_in_episode += 1

    # ---- 1. Read sensors ---------------------------------------------------
    var hip_angles: Array = []
    var hip_vels:   Array = []
    var knee_angles: Array = []
    var knee_vels:  Array = []
    var yaw_angles: Array = []
    var yaw_vels:   Array = []
    for i in range(4):
        # Hip pitch is now measured between SHOULDER and upper leg
        # (the kinematic chain is chassis→shoulder→upper→lower).
        var ha: float = _measure_joint_angle(_shoulders[i], _upper_legs[i],
            _hip_axes[i],
            _shoulder_initial_basis[i].inverse() * _upper_initial_basis[i])
        var ka: float = _measure_joint_angle(_upper_legs[i], _lower_legs[i],
            _knee_axes[i],
            _upper_initial_basis[i].inverse() * _lower_initial_basis[i])
        # Hip yaw — relative rotation between chassis and shoulder.
        var ya: float = _measure_joint_angle(_chassis, _shoulders[i],
            _yaw_axes[i],
            _shoulder_initial_basis[i])   # chassis at identity at rest
        hip_angles.append(ha)
        knee_angles.append(ka)
        yaw_angles.append(ya)
        hip_vels.append((ha - _previous_hip_angle[i]) / TAU)
        knee_vels.append((ka - _previous_knee_angle[i]) / TAU)
        yaw_vels.append((ya - _previous_yaw_angle[i]) / TAU)
        _previous_hip_angle[i] = ha
        _previous_knee_angle[i] = ka
        _previous_yaw_angle[i] = ya

    var chassis_xform: Transform3D = _chassis.global_transform
    var chassis_y: float = chassis_xform.origin.y
    var chassis_tilt: float = _chassis_tilt_from_up(chassis_xform.basis)
    var yaw: float = chassis_xform.basis.get_euler().y
    var fwd_v: float = _chassis.linear_velocity.length()
    var ang_v: float = _chassis.angular_velocity.y

    # ---- 2. Publish proprio -----------------------------------------------
    var imu := PackedFloat64Array()
    imu.append(sin(yaw))
    imu.append(cos(yaw))
    imu.append(clamp(fwd_v / 3.0,  -1.0, 1.0))
    imu.append(clamp(ang_v / PI,   -1.0, 1.0))
    brain.publish_proprio(imu, "imu")

    var joints := PackedFloat64Array()
    for i in range(4):
        joints.append(clamp(yaw_angles[i]  / HIP_YAW_LIMIT,     -1.0, 1.0))
    for i in range(4):
        joints.append(clamp(hip_angles[i]  / (PI * 0.5),        -1.0, 1.0))
    for i in range(4):
        joints.append(clamp(knee_angles[i] / (PI * 0.6),        -1.0, 1.0))
    brain.publish_proprio(joints, "joints")

    var feet := PackedFloat64Array()
    for i in range(4):
        feet.append(1.0 if _foot_grounded(_lower_legs[i]) else 0.0)
    brain.publish_proprio(feet, "foot_contact")

    var ch := PackedFloat64Array()
    ch.append(chassis_y / NOMINAL_STAND_HEIGHT)
    brain.publish_proprio(ch, "chassis_height")

    # ---- 3. Compute reward / failure --------------------------------------
    # In crouch_extend task: alive = upright (no height check), fall = extreme
    # tilt only (chassis on the floor is the START state, not a failure).
    var is_alive: bool
    var fell: bool
    if task_mode == "crouch_extend":
        is_alive = chassis_tilt < ALIVE_TILT_THRESHOLD
        fell     = chassis_tilt > FAIL_TILT_THRESHOLD
    else:
        is_alive = chassis_y > ALIVE_HEIGHT_THRESHOLD and chassis_tilt < ALIVE_TILT_THRESHOLD
        fell     = chassis_y < FAIL_HEIGHT_THRESHOLD   or chassis_tilt > FAIL_TILT_THRESHOLD
    # Binary body_alive — drives HomeostaticDrive's alive_pulse channel
    # (urgency rises when the body isn't above the alive threshold).
    if is_alive:
        brain.publish_event("body_alive", 1.0)
        episode_alive_ticks += 1
    # Continuous body_lift reward.  NeurochemState's aligned-event
    # dopamine pulse is per-EVENT-COUNT, not per-event-intensity.  To
    # get a height gradient, publish N events per tick scaled by
    # chassis height.  In crouch_extend, baseline is 0 so brain gets a
    # nonzero reward in the crouch (~3 events/tick at chassis_y=0.105)
    # and incrementally MORE as it lifts.  All-or-nothing at the natural
    # collapse height stalled the brain — no signal anywhere → W
    # plateaued at 1.32 with no lift.
    var lift_baseline: float = 0.0 if task_mode == "crouch_extend" else 0.05
    var lift: float = clamp(chassis_y - lift_baseline, 0.0, 0.35)
    var n_lift_events: int = int(round(lift * 30.0))   # 0..~10 events/tick
    for n in range(n_lift_events):
        brain.publish_event("body_lift", 1.0)
    if fell and not _episode_fell:
        brain.publish_event("fell_over", 1.0)
        _episode_fell = true

    # ---- 4. Tick the brain ------------------------------------------------
    # In instant_pause mode, the tick AFTER a reset skips brain.tick() so
    # the brain doesn't attribute the teleport-snap to its own action.  One-
    # shot flag set by _reset_episode().
    if _instant_pause_tick:
        _instant_pause_tick = false
    else:
        brain.tick(TAU)

    # ---- 5. Apply brain torques per leg ----------------------------------
    # v6.0.a.6 — 8 brain channels: each leg has independent hip and knee
    # commands, both rotating around the leg's lateral axis.  Combined
    # they give the brain a 2-DOF lift/posture surface per leg
    # (8-dimensional total).  The body now genuinely depends on brain
    # output to remain standing — at zero brain output the rest PD
    # alone is too weak to support the chassis.
    var hip_brain:  Array = [0.0, 0.0, 0.0, 0.0]
    var knee_brain: Array = [0.0, 0.0, 0.0, 0.0]
    for i in range(4):
        hip_brain[i]  = brain.get_action_channel(_action_idx_hip[i])
        knee_brain[i] = brain.get_action_channel(_action_idx_knee[i])
    # Diagnostic override: OGMA_QUADRUPED_FORCE_KNEE=+1.0 (and FORCE_HIP) bypass
    # brain output to test the physical capability ceiling of the body.  Used
    # in v6.0.a.11 to confirm the body cannot lift from chassis_y=0.105 even
    # with all 4 knees commanded to maximum extension — the foot-push needed
    # to lift the chassis isn't expressible by independent angle-PDs.
    var env_fk: String = OS.get_environment("OGMA_QUADRUPED_FORCE_KNEE")
    if env_fk != "":
        var fk: float = env_fk.to_float()
        for i in range(4):
            knee_brain[i] = fk
    var env_fh: String = OS.get_environment("OGMA_QUADRUPED_FORCE_HIP")
    if env_fh != "":
        var fh: float = env_fh.to_float()
        for i in range(4):
            hip_brain[i] = fh
    for i in range(4):
        var hip_axis:  Vector3 = _hip_axes[i]
        var knee_axis: Vector3 = _knee_axes[i]
        var yaw_axis:  Vector3 = _yaw_axes[i]
        var max_torque: float = MAX_BRAIN_TORQUE * leg_strength
        # v6.0.a.8 — position-commanded (servo-style) actuators.
        # Brain output u ∈ [-1,+1] becomes target angle = u × range.
        # PD computes the torque needed to track that target; the
        # torque is CLAMPED to the motor's max so a far-from-target
        # brain command can't deliver an unbounded impulse.
        var u_h: float = clamp(float(hip_brain[i]),  -1.0, 1.0)
        var u_k: float = clamp(float(knee_brain[i]), -1.0, 1.0)
        var hip_target:  float = u_h * HIP_TARGET_RANGE
        var knee_target: float = u_k * KNEE_TARGET_RANGE + KNEE_REST_ANGLE  # bias rest pose
        var hip_torque_raw:  float = SERVO_HIP_KP  * (hip_target  - hip_angles[i])  - SERVO_HIP_KD  * hip_vels[i]
        var knee_torque_raw: float = SERVO_KNEE_KP * (knee_target - knee_angles[i]) - SERVO_KNEE_KD * knee_vels[i]
        var hip_torque:  float = clamp(hip_torque_raw,  -max_torque, max_torque)
        var knee_torque: float = clamp(knee_torque_raw, -max_torque, max_torque)
        _upper_legs[i].apply_torque_impulse(hip_axis  * (hip_torque  * TAU))
        _lower_legs[i].apply_torque_impulse(knee_axis * (knee_torque * TAU))
        # Yaw locked (rigid limits + soft PD; not brain-controlled).
        var yaw_err: float = HIP_YAW_TARGET_ANGLE - yaw_angles[i]
        var yaw_torque_mag: float = HIP_YAW_KP * yaw_err - HIP_YAW_KD * yaw_vels[i]
        _shoulders[i].apply_torque_impulse(yaw_axis * (yaw_torque_mag * TAU))

    # ---- 6. JSONL diag emit (sparse) --------------------------------------
    if diag_interval_ticks > 0 and (tick_counter % diag_interval_ticks) == 0:
        _emit_jsonl(hip_brain, knee_brain, chassis_y, chassis_tilt)

    # ---- 7. Episode termination ------------------------------------------
    # In continuous mode, neither fall nor max_steps triggers an in-place
    # reset — the body lies on the ground and the brain has to wiggle
    # back up.  body_alive simply stops firing while down.  A non-zero
    # max_steps is treated as a hard run-end (no teleport).
    if reset_mode == "continuous":
        # crouch_extend / continuous needs MC trajectory finalisation
        # without a body teleport.  When mc_episode_period > 0, fire
        # events.episode_end every N ticks and reset the alive_ticks
        # accumulator for the diag log; body keeps moving in place.
        if mc_episode_period > 0 and step_in_episode > 0 \
                and (step_in_episode % mc_episode_period) == 0:
            brain.publish_event("episode_end", float(episode_alive_ticks))
            print(JSON.stringify({
                "event": "EPISODE_END",
                "episode": episode_index,
                "alive_ticks": episode_alive_ticks,
                "steps": step_in_episode,
                "reason": "mc_period",
                "reset_mode": reset_mode,
            }))
            episode_index += 1
            episode_alive_ticks = 0
        if max_steps > 0 and step_in_episode >= max_steps:
            _done = true
            print(JSON.stringify({
                "event": "RUN_END",
                "episodes": episode_index + 1,
                "tick": tick_counter,
                "reset_mode": reset_mode,
                "final_alive_ticks": episode_alive_ticks,
            }))
    else:
        if fell or (max_steps > 0 and step_in_episode >= max_steps):
            _finish_episode(fell)

func _finish_episode(fell: bool) -> void:
    var reason: String = "fell" if fell else "max_steps"
    # Phase v5.1 / v6.0 — fire events.episode_end so Premotor's MC mode
    # finalises its trajectory: G_t = Σ γ^k r_{t+k} backwards, advantage
    # normalisation, then the per-step REINFORCE update on W.  Without
    # this event the brain accumulates trajectory forever and W stays
    # frozen at init.  Discovered while debugging: pre_W=0.07 across 200
    # episodes — every Premotor was simply not updating.
    brain.publish_event("episode_end", float(episode_alive_ticks))
    print(JSON.stringify({
        "event": "EPISODE_END",
        "episode": episode_index,
        "alive_ticks": episode_alive_ticks,
        "steps": step_in_episode,
        "reason": reason,
        "reset_mode": reset_mode,
    }))
    episode_index += 1
    if max_episodes > 0 and episode_index >= max_episodes:
        _done = true
        print(JSON.stringify({
            "event": "RUN_END",
            "episodes": episode_index,
            "tick": tick_counter,
        }))
        return
    if reset_mode == "soft_blink":
        _begin_soft_blink()   # tick loop hands off to _step_blink next call
    else:   # "instant_pause" or anything else: legacy teleport, with single-tick brain skip
        _do_hard_reset()
        _instant_pause_tick = true   # _step_one will skip brain.tick() exactly once
        step_in_episode = 0
        episode_alive_ticks = 0
        _episode_fell = false

func _teleport_to_standing() -> void:
    # Inner helper — assumes bodies are already in a state where transform
    # writes will stick (either frozen KINEMATIC or called within a
    # physics frame from a fresh _set_body_state path).  Soft_blink's
    # _end calls this with bodies still frozen from _begin_soft_blink.
    # The hard-reset path (instant_pause, spacebar manual) uses
    # _do_hard_reset which does the freeze cycle itself.
    _chassis.global_transform = Transform3D(Basis.IDENTITY, Vector3(0, NOMINAL_STAND_HEIGHT, 0))
    _chassis.linear_velocity  = Vector3.ZERO
    _chassis.angular_velocity = Vector3.ZERO
    for i in range(4):
        # Reset to the morphology's standing pose, captured at build
        # time.  Dog: upper/lower hang straight down from hip.  Spider:
        # upper splayed outward at SPIDER_UPPER_SLOPE, lower drops
        # vertically from the elbow.  v6.0.a.4 adds the shoulder body
        # (yaw connector); reset it to its rest pose too.
        _shoulders[i].global_transform = _shoulder_rest_xform[i]
        _shoulders[i].linear_velocity  = Vector3.ZERO
        _shoulders[i].angular_velocity = Vector3.ZERO
        _upper_legs[i].global_transform = _upper_rest_xform[i]
        _upper_legs[i].linear_velocity  = Vector3.ZERO
        _upper_legs[i].angular_velocity = Vector3.ZERO
        _lower_legs[i].global_transform = _lower_rest_xform[i]
        _lower_legs[i].linear_velocity  = Vector3.ZERO
        _lower_legs[i].angular_velocity = Vector3.ZERO
        _previous_hip_angle[i]  = 0.0
        _previous_knee_angle[i] = 0.0
        _previous_yaw_angle[i]  = 0.0

# Hard reset used by both instant_pause (auto, on fall) and spacebar
# (manual).  Wraps _teleport_to_standing in a freeze→write→unfreeze
# cycle so the transform writes stick against dynamic-mode body physics
# overrides.  Must run inside a physics frame — see _step_one's
# _pending_manual_reset consumer.
func _do_hard_reset() -> void:
    # Freeze all rigid bodies to KINEMATIC so transform writes are
    # authoritative for this frame.  Godot's freeze toggle is honoured
    # immediately when set inside _physics_process.
    _chassis.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
    _chassis.freeze = true
    for i in range(4):
        _shoulders[i].freeze_mode  = RigidBody3D.FREEZE_MODE_KINEMATIC
        _shoulders[i].freeze       = true
        _upper_legs[i].freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
        _upper_legs[i].freeze      = true
        _lower_legs[i].freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
        _lower_legs[i].freeze      = true
    _teleport_to_standing()
    # Release the freeze so the bodies become dynamic again next frame.
    _chassis.freeze = false
    for i in range(4):
        _shoulders[i].freeze  = false
        _upper_legs[i].freeze = false
        _lower_legs[i].freeze = false

# ---------------------------------------------------------------------------
# Soft-blink reset (reset_mode == "soft_blink")
# ---------------------------------------------------------------------------
# On fell_over: cache the fallen pose, switch chassis + legs to PHYSICS-OFF
# (freeze_mode = STATIC), then over _SOFT_BLINK_TICKS interpolate the
# transform from fallen → standing.  Brain still ticks each step inside
# _step_blink so its DescendingPredictor sees the lift, but no torque is
# applied and no body_alive event fires.  At the end of the blink, freeze
# is released and physics + reward resume.

func _begin_soft_blink() -> void:
    _blink_active     = true
    _blink_ticks_left = _SOFT_BLINK_TICKS
    # Cache the START pose so _step_blink can lerp from it.
    _blink_start_pose = {
        "chassis":   _chassis.global_transform,
        "shoulders": [],
        "uppers":    [],
        "lowers":    [],
    }
    for i in range(4):
        _blink_start_pose["shoulders"].append(_shoulders[i].global_transform)
        _blink_start_pose["uppers"].append(_upper_legs[i].global_transform)
        _blink_start_pose["lowers"].append(_lower_legs[i].global_transform)
    # Freeze every rigid body in the chain so external forces don't
    # fight the kinematic lerp.
    _chassis.freeze = true
    _chassis.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
    _chassis.linear_velocity  = Vector3.ZERO
    _chassis.angular_velocity = Vector3.ZERO
    for i in range(4):
        for rb in [_shoulders[i], _upper_legs[i], _lower_legs[i]]:
            rb.freeze = true
            rb.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
            rb.linear_velocity  = Vector3.ZERO
            rb.angular_velocity = Vector3.ZERO

func _step_blink() -> void:
    # Lerp progress: t=0 at blink start, t=1 at blink end.
    var done_ticks: int = _SOFT_BLINK_TICKS - _blink_ticks_left
    var t: float = float(done_ticks + 1) / float(_SOFT_BLINK_TICKS)
    tick_counter += 1

    # Y-axis "lift bump" applied to chassis and all leg segments during the
    # rotation phase.  Without it, mid-blink the chassis is partway between
    # its fallen height (~0.05 m) and standing height (0.30 m), but the
    # legs are sweeping through full extension as the rotation slerps —
    # feet dip below the floor.  A sin(πt) bump peaks at t=0.5 (the worst
    # rotation moment) and decays to 0 at both ends so we land exactly at
    # the standing pose.  Bump magnitude ≥ leg extent radius (~0.45 m).
    var lift: float = sin(PI * t) * _BLINK_LIFT_HEIGHT

    var target_chassis := Transform3D(Basis.IDENTITY, Vector3(0, NOMINAL_STAND_HEIGHT, 0))
    var interp_chassis: Transform3D = (_blink_start_pose["chassis"] as Transform3D).interpolate_with(target_chassis, t)
    interp_chassis.origin.y += lift
    _chassis.global_transform = interp_chassis
    for i in range(4):
        # Lerp to the morphology's rest xform (captured at build time).
        var sh_target: Transform3D = _shoulder_rest_xform[i]
        var up_target: Transform3D = _upper_rest_xform[i]
        var lo_target: Transform3D = _lower_rest_xform[i]
        var interp_sh:    Transform3D = (_blink_start_pose["shoulders"][i] as Transform3D).interpolate_with(sh_target, t)
        var interp_upper: Transform3D = (_blink_start_pose["uppers"][i]    as Transform3D).interpolate_with(up_target, t)
        var interp_lower: Transform3D = (_blink_start_pose["lowers"][i]    as Transform3D).interpolate_with(lo_target, t)
        interp_sh.origin.y    += lift
        interp_upper.origin.y += lift
        interp_lower.origin.y += lift
        _shoulders[i].global_transform = interp_sh
        _upper_legs[i].global_transform = interp_upper
        _lower_legs[i].global_transform = interp_lower

    # Brain still ticks (proprio observable) but no body_alive fires and
    # no torque is applied.  Republish proprio so the brain sees the lift.
    var hip_angles: Array = []
    var knee_angles: Array = []
    var yaw_angles: Array = []
    for i in range(4):
        hip_angles.append(_measure_joint_angle(_shoulders[i], _upper_legs[i],
            _hip_axes[i],
            _shoulder_initial_basis[i].inverse() * _upper_initial_basis[i]))
        knee_angles.append(_measure_joint_angle(_upper_legs[i], _lower_legs[i],
            _knee_axes[i],
            _upper_initial_basis[i].inverse() * _lower_initial_basis[i]))
        yaw_angles.append(_measure_joint_angle(_chassis, _shoulders[i],
            _yaw_axes[i],
            _shoulder_initial_basis[i]))
    var joints := PackedFloat64Array()
    for v in yaw_angles:  joints.append(clamp(v / HIP_YAW_LIMIT, -1.0, 1.0))
    for v in hip_angles:  joints.append(clamp(v / (PI * 0.5),    -1.0, 1.0))
    for v in knee_angles: joints.append(clamp(v / (PI * 0.6),    -1.0, 1.0))
    brain.publish_proprio(joints, "joints")
    var ch_height := PackedFloat64Array()
    ch_height.append(_chassis.global_transform.origin.y / NOMINAL_STAND_HEIGHT)
    brain.publish_proprio(ch_height, "chassis_height")
    brain.tick(TAU)

    _blink_ticks_left -= 1
    if _blink_ticks_left <= 0:
        _end_soft_blink()

func _end_soft_blink() -> void:
    _teleport_to_standing()        # snap residual drift
    # Restore physics on every body in the chain.
    _chassis.freeze = false
    for i in range(4):
        _shoulders[i].freeze  = false
        _upper_legs[i].freeze = false
        _lower_legs[i].freeze = false
    _blink_active = false
    _blink_ticks_left = 0
    _blink_start_pose.clear()
    step_in_episode = 0
    episode_alive_ticks = 0
    _episode_fell = false

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

func _measure_relative_angle(parent: RigidBody3D, child: RigidBody3D) -> float:
    # Kept for back-compat with the legacy dog code path's call sites
    # (hinge axis = world +Z).  Spider mode uses _measure_joint_angle.
    var rel: Basis = parent.global_transform.basis.inverse() * child.global_transform.basis
    return rel.get_euler().z

# Project the relative rotation between two RigidBody3Ds onto a known
# joint axis, returning the signed angular displacement from the
# initial rest pose.  Works for arbitrary hinge axes (spider mode
# where each leg has its own lateral direction) and reduces to the
# Z-axis case when axis_world = (0,0,1) and rest_basis = identity.
func _measure_joint_angle(parent: RigidBody3D, child: RigidBody3D,
        axis_world: Vector3, rest_basis: Basis) -> float:
    var rel: Basis = parent.global_transform.basis.inverse() * child.global_transform.basis
    # Subtract out the rest-pose relative basis so the result is the
    # deflection from the standing pose (zero when at rest).
    var delta: Basis = rel * rest_basis.inverse()
    var q: Quaternion = Quaternion(delta.orthonormalized())
    # Quaternion.get_axis() / get_angle() are undefined for w≈1 (no
    # rotation); guard against that explicitly.
    if abs(q.w) > 0.9999:
        return 0.0
    var ang: float = q.get_angle()
    var ax: Vector3 = q.get_axis()
    # Project onto the joint axis — for a hinge the rotation lies
    # along this axis (joint constraint), so the dot product gives the
    # signed component.
    return ang * sign(ax.dot(axis_world))

func _chassis_tilt_from_up(b: Basis) -> float:
    # Angle between the chassis's local +Y axis and world +Y.  Zero when level,
    # PI/2 when chassis is lying on its side.
    var up_local: Vector3 = b.y.normalized()
    return acos(clamp(up_local.dot(Vector3.UP), -1.0, 1.0))

func _foot_grounded(lower_leg: RigidBody3D) -> bool:
    # Approximate: lower leg's lowest point is near y=0.  Avoids raycast cost.
    var lowest_y: float = lower_leg.global_transform.origin.y - LOWER_LEG_LEN * 0.5
    return lowest_y <= 0.02

func _emit_jsonl(hip_brain: Array, knee_brain: Array, chassis_y: float, chassis_tilt: float) -> void:
    var metrics: Dictionary = brain.get_module_metrics()
    var feet_y: Array = []
    var hip_ang: Array = []
    var knee_ang: Array = []
    for i in range(4):
        var lowest_y: float = _lower_legs[i].global_transform.origin.y - LOWER_LEG_LEN * 0.5
        feet_y.append(snappedf(lowest_y, 0.001))
        hip_ang.append(snappedf(_previous_hip_angle[i], 0.01))
        knee_ang.append(snappedf(_previous_knee_angle[i], 0.01))
    var hip_cmd: Array = []
    var knee_cmd: Array = []
    for i in range(4):
        hip_cmd.append(snappedf(float(hip_brain[i]), 0.01))
        knee_cmd.append(snappedf(float(knee_brain[i]), 0.01))
    var line := {
        "t": tick_counter,
        "ep": episode_index,
        "ep_step": step_in_episode,
        "ep_alive": episode_alive_ticks,
        "y": snappedf(chassis_y, 0.001),
        "tilt": snappedf(chassis_tilt, 0.001),
        "hip_cmd":  hip_cmd,
        "knee_cmd": knee_cmd,
        "da": snappedf(brain.get_dopamine(),  0.001),
        "ht": snappedf(brain.get_serotonin(), 0.001),
        "feet_y":   feet_y,
        "hip":      hip_ang,
        "knee":     knee_ang,
        "hip_lim":  [HIP_LIMIT_LOWER, HIP_LIMIT_UPPER],
        "knee_lim": [KNEE_LIMIT_LOWER, KNEE_LIMIT_UPPER],
    }
    # Per-Premotor entropy + W norm so we can see whether the 4 legs'
    # policies are differentiating (W_norm diverges across legs) or
    # collapsing to the same uniform output.
    var pre_H: Dictionary = {}
    var pre_HW: Dictionary = {}
    var pre_W: Dictionary = {}
    for mod_id in metrics:
        var m: Dictionary = metrics[mod_id]
        if m.get("type", "") == "Premotor":
            pre_H[mod_id]  = snappedf(float(m.get("last_entropy",  0.0)), 0.001)
            pre_HW[mod_id] = snappedf(float(m.get("chosen_window_entropy", 0.0)), 0.001)
            pre_W[mod_id]  = snappedf(float(m.get("W_total_norm", 0.0)), 0.0001)
    if pre_W.size() > 0:
        line["pre_H"]  = pre_H
        line["pre_HW"] = pre_HW
        line["pre_W"]  = pre_W
    print(JSON.stringify(line))
