extends Node3D
##
## PiCrawler 12-DOF quadruped body (v6.0.b — clean implementation).
##
## Mirrors SunFounder PiCrawler hardware kinematics; geometry from the STEP
## file + Blender measurements (see docs/picrawler_geometry.md).
##
## Per leg (3 servos):
##   chassis ──[hip1 hinge, axis = world +Y (vertical)]── coxa
##           ──[hip2 hinge, axis = leg-local lateral]── upper leg
##           ──[knee  hinge, axis = leg-local lateral]── lower leg ── toe
##
## Standing pose:
##   hip1 = 0 (legs splayed at corner-outward 45°)
##   hip2 = 0 (upper leg parallel to ground)
##   knee = −80° (lower leg ~vertical, toe ~10° from straight down)
##
## Brain action API: 12 channels.
##   action.{leg}_hip1, action.{leg}_hip2, action.{leg}_knee  for leg ∈ {fl, fr, rl, rr}.
##
## Standing-task bridge (v6.0.b.1):
##   Proprio:  reality.proprio.imu    (4-D vestibular: sin/cos yaw, fwd v, ang v)
##             reality.proprio.joints (12-D: 4 hip1 + 4 hip2 + 4 knee, normalised)
##   Events:   events.hit         — graded reward, duty-cycled by chassis_y,
##                                  peak 0.2 hits/tick at standing height.
##                                  Couples to NeurochemState dopamine.
##             events.miss        — fires once on fall (chassis below FAIL_HEIGHT
##                                  or tilt above FAIL_TILT).  DA + 5-HT drop.
##             events.episode_end — trajectory finalise boundary.
##   chassis_height is the REWARD signal — NOT published as a sensor.
##
## Episode lifecycle: same shape as quadruped_body.gd — JSON diag emit,
## reset_mode {continuous, instant_pause, soft_blink}, mc_episode_period
## fires events.episode_end periodically when in continuous mode so
## Premotor MC gets its trajectory boundaries.
##
## Note on conventions: Godot uses Y-up.  All angles in radians.
##   World +Y = up
##   Leg "outward" headings in horizontal X-Z plane:
##     FL = −X, +Z   FR = +X, +Z   RL = −X, −Z   RR = +X, −Z
##   "Leg-local lateral" = world UP × heading (rotated 90° CCW in horizontal).

# ---------------------------------------------------------------------------
# Geometry — the BODY, loaded from a swappable JSON before _build_body()
# ---------------------------------------------------------------------------
# 2026-08-10: these were `const` until the physical robot reached the bench and
# its MEASURED geometry turned out to differ materially from the CAD export
# (see docs/operational/picrawler_geometry.md §"Measured from the built robot"
# and docs/plans-and-designs/picrawler_sim2real_port.md).  They are now `var`s
# populated by _load_geometry() BEFORE _build_body(), which makes the body
# swappable — including mid-run, via _rebuild_body(), for the morphological
# (d) test.
#
# The literals below ARE the CAD body and remain the fallback if the JSON is
# missing or malformed, so a load failure degrades to the historical build
# rather than to something silently wrong.  Rationale for each value stays
# HERE; the JSON carries bare numbers so no documentation is lost.
@export var body_geometry_path: String = "res://addons/ami_ogma/body/cad.json"
var _geometry_name: String = "cad"   # whatever was actually loaded; shown in diagnostics

# The @export literal for target_height / peak_height.  _recompute_derived_
# geometry() treats "still equal to this" as "nobody set it explicitly", and
# only then re-points it at the loaded STANDING_CHASSIS_Y.
const _EXPORT_DEFAULT_HEIGHT: float = 0.082

const TAU: float = 0.02              # 50 Hz brain tick — NOT geometry, stays const
var L1: float = 0.03726              # hip1 → hip2 (coxa segment length)
var L2: float = 0.0536               # hip2 → knee (upper leg)
var L3: float = 0.0755               # knee → toe (lower leg + foot)
var COXA_Z_DROP: float = 0.007       # vertical drop from chassis-mounted hip1 to hip2

var CHASSIS_X: float = 0.098
var CHASSIS_Y: float = 0.042
var CHASSIS_Z: float = 0.098
var HIP_X_SPAN: float = 0.0766       # hip1 mount rectangle width  (76.6 mm)
var HIP_Z_SPAN: float = 0.066        # hip1 mount rectangle length (66 mm)

# = real measured chassis-to-floor.  We spawn at this height so feet START on
# the floor (no fall, no impact).  Previous 0.105 spawn meant 23mm drop on
# launch → asymmetric foot impacts propagated through joints to tilt the chassis.
var STANDING_CHASSIS_Y: float = 0.082

# Standing-pose joint angles for the PD rest targets.  All zero because
# the construction-time relative orientation between adjacent bodies is
# IDENTITY (both built with IDENTITY basis at their world-positioned
# centers), so the joint reports angle 0 at the standing pose.  PD
# targeting 0 keeps the body in its construction pose.
var HIP1_REST: float = 0.0
var HIP2_REST: float = 0.0
# Calibration confirmed (2026-05-16): hip1=0, hip2=0, knee=−1.6 produces
# a perfect X-stance for all four legs.  KNEE_REST is the joint angle
# the motor commands when brain action u_knee=0 — set to standing so
# the brain starts at a stable base pose and learns balance around it.
var KNEE_REST: float = -1.6
# LEG GEOMETRY only: how much to rotate heading around lateral to compute
# the standing-pose lower-leg DIRECTION (not the same as the joint angle).
# −1.396 rad ≈ −80° puts the lower leg almost vertical with toe down.
var LOWER_LEG_DROP_ANGLE: float = -1.396

# Joint angle limits — knee is centered at −1.6 (standing pose) with a
# full ±1.4 rad brain-command range around it, so limits must span
# [−3.0, −0.2] at minimum.  Hip1/hip2 are centered at 0 with ±1.4 range.
var HIP1_LIMIT: float = 1.40         # ±80° around 0  (hip1 yaw)
var HIP2_LIMIT: float = 1.40         # ±80° around 0  (hip2 pitch)

# 2026-06-01: widened from -1.70 to allow deeper bend (deep-tuck / crouch
# postures in the negative u direction).  Symmetric limits were under-utilizing
# the physical range — at rest=-1.6 with old limit -1.70 the brain only reached
# -97° on the bend side.  -2.50 ≈ -143° leaves headroom from the -π = -3.14
# quaternion wrap point while giving Cruse/Walknet-style deep-stance poses a
# place to sit.  Pairs with the asymmetric KNEE_RANGE_HYPEREXT below.
var KNEE_LIMIT_LOW: float = -2.50

# Symmetric with KNEE_LIMIT_LOW around the perpendicular rest pose.  Setting
# UPPER to exactly the rest value (0) made the hinge constraint asymmetric and
# the motor unable to drive past the gravity equilibrium on the negative side.
var KNEE_LIMIT_HIGH: float = 1.70

# ⚠ HARDWARE NOTE (2026-08-10): KNEE_RANGE_FOLD/HYPEREXT below let the brain
# COMMAND a 232° knee span; a 500–2500 µs hobby servo has ~180°.  The deployed
# gait only OCCUPIES 113° (−98…+15), so this is not a porting blocker — but the
# limits are fiction, and hardware must be granted occupancy-plus-margin rather
# than the commanded span.  See picrawler_sim2real_port.md §Phase 2.

# Masses calibrated to SunFounder PiCrawler published 600g total.
# Previous estimate underweighted legs (14g) — actual servo + bracket +
# screws + connectors is closer to 25g per segment.  Heavier legs also
# give the constraint solver more inertia to work with, reducing
# numerical sensitivity when motors apply impulse.
#   Chassis (300g): Pi 5 + battery + base plate + 4 chassis-mounted hip1 servos
#   Coxa (25g):   hip2 servo + coxa arm bracket + connectors
#   Upper (25g):  knee servo + upper-arm bracket + connectors
#   Lower (25g):  lower-arm bracket + foot + extension
# Total: 300 + 4×(25+25+25) = 600g.
# ⚠ The real robot weighs 590 g with 62 g legs ⇒ a 58/42 body/leg split, not the
# 50/50 these values give.  Corrected numbers ship in body/measured.json; these
# stay as the CAD body.  Comments at :1605 and :8714 still assert the 50 % figure.
var CHASSIS_MASS: float = 0.300
var COXA_MASS: float    = 0.025
var UPPER_MASS: float   = 0.025
var LOWER_MASS: float   = 0.025

# Visual/collision sizes
var COXA_RADIUS: float = 0.020       # thicker stub
var LEG_RADIUS:  float = 0.012       # thinner segments

# Servo model — calibrated for sim-to-real fidelity per docs/servo_dynamics.md.
# Metal-gear hobby servo (~MG90S class), heavy gear train, stiff PWM tracker.
#
# Powered mode (normal operation):
#   torque = clamp(Kp * (target - angle) - Kd * ω, ±MAX_SERVO_TORQUE)
#   with deadband around target, and torque-speed dropoff so output
#   tapers to 0 as ω approaches MAX_SERVO_SPEED (realistic motor curve).
# NOTE: with motor-based velocity control, SERVO_KP isn't a torque-PD
# gain anymore — it's a velocity-command gain (rad/s per radian of
# position error).  The doc's Kp=150-300 is meaningful for position-PD
# servos with internal PWM bandwidth limiters.  Our sim's motor target-
# velocity is the bottleneck, so a gentler Kp gives smoother motion.
# Kp_vel=20 means: 0.4 rad error → 8 rad/s command (saturates at max);
# 0.1 rad error → 2 rad/s command.
const SERVO_KP: float = 20.0
const SERVO_KI: float = 0.0
const SERVO_KD: float = 8.0
const MAX_SERVO_TORQUE: float = 0.15      # Nm — gentle enough that motor reactions on the
										  # chassis don't accumulate fast enough to flip the body.
										  # Static gravity load at hip2 is ~0.037 Nm so 0.15
										  # gives ~4× headroom — sufficient for slow standing.
										  # loads.  Spec lists 1.5-3.5 Nm stall, but stall is
										  # only relevant under heavy load; on tiny picrawler
										  # legs the motor's no-load torque (~0.15 Nm class)
										  # dominates and giving 2 Nm produces unphysical
										  # angular accelerations (~150,000 rad/s²) that
										  # exceed Euler stability and look like flailing.
										  # 0.3 Nm still gives ~20× headroom over the
										  # gravitational moment at hip2 (~0.013 Nm).
const MAX_SERVO_SPEED: float = 6.0        # rad/s — matches doc spec (6-10).  Now that joints
										  # use right-handed bases (constraint solver stable)
										  # and chassis is suspended during calibration,
										  # higher speed is safe.  At 6 rad/s a full ±80° throw
										  # takes ~230ms — fits comfortably in the 500ms hold
										  # window with ~270ms visible "at-target" time.
const SERVO_DEADBAND: float = 0.001       # rad — spec says 0.0003-0.0015 (PWM precision)
# Torque rise time — first-order lag emulating real servo's gear inertia +
# motor electrical time constant + PWM cycle (50Hz update rate).  Without
# this, the stiff Kp=150 produces step-impulse torques that exceed what
# Euler integration at 240Hz can resolve → high-frequency oscillation
# visible as "twitching".  τ=30ms gives a realistic torque-onset profile;
# brain-tick alpha = 1 - exp(-TAU/τ) ≈ 0.49 per 20ms tick.
const SERVO_TORQUE_RISE_TAU: float = 0.030
const HIP_TARGET_RANGE:  float = 1.40     # brain/cal u=±1 → ±80° = full servo travel
const HIP1_TARGET_RANGE: float = 1.40     # brain/cal u=±1 → ±80° = full servo travel
# 2026-06-01 → 2026-06-03 — asymmetric knee mapping.  Original labels
# (KNEE_RANGE_EXTEND / _BEND) were geometrically reversed — claimed
# "u=+1 → 0 rad = straight" but C-mode operator verification shows
# t=-1.6 is actually straight, t=0 is the construction pose (~90° bent),
# and t=+1.6 is fully tucked (~170° fold).  Renamed and values updated
# 2026-06-03 to match operator-verified geometry:
#   u=+1 → KNEE_REST + 3.20 = +1.60 rad (~170° tuck — spider stance)
#   u=0  → KNEE_REST        = -1.60 rad (straight, lower colinear with upper)
#   u=-1 → KNEE_REST - 0.85 = -2.45 rad (hyperextended past straight)
# Total reachable brain range ≈ 4.05 rad ≈ 232°, covering full natural
# knee fold + slight hyperextension.  Joint LIMIT_LOW=-2.50 / HIGH=+1.70
# clamp the absolute extremes.
const KNEE_RANGE_FOLD:    float = 3.20    # u=+1 → KNEE_REST + 3.20 = +1.60 rad (max tuck)
const KNEE_RANGE_HYPEREXT: float = 0.85   # u=-1 → KNEE_REST - 0.85 = -2.45 rad (past straight)
# Unpowered (ragdoll) passive resistance: back-EMF + gear-train friction.
# A real powered-off metal-gear servo resists backdrive — not freewheeling.
const UNPOWERED_STATIC_FRICTION: float = 0.10  # Nm — spec says 0.05-0.15 (stiction)
const UNPOWERED_VISCOUS_FRICTION: float = 0.02 # Nm·s/rad — spec says 0.01-0.03

# Damping — bumped to make legs settle smoothly instead of oscillating.
const BODY_ANGULAR_DAMP: float = 8.0      # was 4.0
const BODY_LINEAR_DAMP:  float = 2.0      # damps translational oscillation; helps with
										  # contact-twitching when legs touch the floor.
# Shared physics material for floor + all body parts.  Friction tuned
# for "small plastic foot on smooth surface" — feet grip enough to push
# off but slip a little under sustained sideways load (matches the real
# PiCrawler's behaviour on a desk or floor).  Coefficients reference:
#   rubber on dry concrete: ~0.8 (too sticky for small plastic feet)
#   PiCrawler plastic on smooth surface: ~0.5 (this value)
#   plastic on slick surface: ~0.3
# Previous value was 1.5 — way over rubber-on-concrete — which is why
# feet appeared sticky/bouncy (sudden grip release = visible jolt).
var _contact_mat: PhysicsMaterial = null
func _make_contact_mat() -> PhysicsMaterial:
	if _contact_mat == null:
		_contact_mat = PhysicsMaterial.new()
		_contact_mat.friction = 1.5
		_contact_mat.rough    = true
		_contact_mat.bounce   = 0.0
		_contact_mat.absorbent = true
	return _contact_mat

# Low-friction material used on the lower legs ONLY while motor_test_mode
# is on.  Otherwise foot friction (μ=1.5) pins the leg and the motor can't
# rotate hip1 (purely lateral) at all — the user can't tell whether the
# motor is doing anything because the foot is mechanically stuck.
# Phase 6.14 — sticky material for the outer wedge ring.  Higher friction
# than the default contact_mat (3.0 vs 1.5) makes climbing the slope
# actually feasible instead of the body sliding off and tumbling.  Also
# more rough/absorbent so motor impulse converts more efficiently to
# slope-following motion.  Applied only to the wedge StaticBody3Ds; the
# floor, pyramids, and feet keep _make_contact_mat().
var _wedge_mat: PhysicsMaterial = null
func _make_wedge_mat() -> PhysicsMaterial:
	if _wedge_mat == null:
		_wedge_mat = PhysicsMaterial.new()
		_wedge_mat.friction  = 3.0
		_wedge_mat.rough     = true
		_wedge_mat.bounce    = 0.0
		_wedge_mat.absorbent = true
	return _wedge_mat

# ---- 2026-08-04 · THE CHASSIS IS NOT A FOOT --------------------------------------------
# `_make_contact_mat()` is a SHARED singleton (floor, hump, bumps, pyramids, every leg
# segment, and the chassis), so once the chassis stopped being a ghost it was dragging on
# mu = 1.5 -- rubber-grade grip, the same as the feet.  Operator, from the ablation bench:
# "when I ablate the two rear legs the robot is not able to drag its chassis along the
# ground... the feet should have the highest friction, not the chassis."  Physically right:
# a smooth plastic/aluminium shell is nothing like a rubber foot pad, and a crippled robot
# dragging its belly is exactly the case that separates them.
#
# NOTE this is INERT while `chassis_collides` is false -- a ghost chassis has no contacts
# for a friction coefficient to apply to -- so it cannot disturb any historical run.
# Verified by measurement, not argued.
var _chassis_mat: PhysicsMaterial = null
func _make_chassis_mat() -> PhysicsMaterial:
	if _chassis_mat == null:
		_chassis_mat = PhysicsMaterial.new()
		_chassis_mat.rough     = false
		_chassis_mat.bounce    = 0.0
		_chassis_mat.absorbent = true
	_chassis_mat.friction = chassis_friction   # re-read so the slider works live
	return _chassis_mat

var _limb_mat: PhysicsMaterial = null
func _make_limb_mat() -> PhysicsMaterial:
	if _limb_mat == null:
		_limb_mat = PhysicsMaterial.new()
		_limb_mat.rough     = true
		_limb_mat.bounce    = 0.0
		_limb_mat.absorbent = true
	_limb_mat.friction = limb_friction    # re-read so the slider works live
	return _limb_mat

## Coxa + upper get limb_friction; the LOWER legs (the feet) keep the grippy contact
## material.  Skipped entirely while motor_test_mode owns the leg materials.
func _apply_limb_materials() -> void:
	if _motor_test_mode:
		return
	for b in _coxas:
		if is_instance_valid(b): b.physics_material_override = _make_limb_mat()
	for b in _uppers:
		if is_instance_valid(b): b.physics_material_override = _make_limb_mat()

func set_limb_friction(v: float) -> void:
	limb_friction = clampf(v, 0.0, 3.0)
	if _limb_mat != null:
		_limb_mat.friction = limb_friction

var _slick_mat: PhysicsMaterial = null
func _make_slick_mat() -> PhysicsMaterial:
	if _slick_mat == null:
		_slick_mat = PhysicsMaterial.new()
		_slick_mat.friction = 0.05
		_slick_mat.rough    = false
		_slick_mat.bounce   = 0.0
		_slick_mat.absorbent = true
	return _slick_mat
# Knee rest direction sign — controls which way the lower leg drops
# from the knee in the standing pose.  −1.0 should point lower leg DOWN
# (standing); flip to +1.0 if the body spawns inverted.
const KNEE_DROP_SIGN: float = -1.0

# Per-leg outward heading at hip1 = neutral.  Indices: FL, FR, RL, RR.
# Godot Y-up convention: legs splay in the X-Z horizontal plane.
const NEUTRAL_HEADINGS: Array = [
	Vector3(-0.7071, 0.0, +0.7071),   # FL: -X +Z
	Vector3(+0.7071, 0.0, +0.7071),   # FR: +X +Z
	Vector3(-0.7071, 0.0, -0.7071),   # RL: -X -Z
	Vector3(+0.7071, 0.0, -0.7071),   # RR: +X -Z
]
const LEG_NAMES: Array = ["fl", "fr", "rl", "rr"]
# ⚠⚠ LEG NAMING MIRROR (operator-diagnosed 2026-08-11 on the piano roll).
# These names are anatomically SWAPPED left↔right: the body's TRUE forward is
# +Z (eyes / corridor / fwd_v — see the corridor-gym note ~:3605), so body-left
# = +X — yet leg 0 "fl" is BUILT at x<0, the anatomical FRONT-RIGHT (the names
# were assigned in the default-camera screen frame, the classic mirror illusion
# of labelling a body that faces you).  Every action topic, config, event name,
# instrument, and historical per-leg finding uses THIS body frame consistently
# ("fl" = red = anatomical FR), so the record is coherent and the mirror is
# behaviorally null — DO NOT rename piecemeal; the blast radius is every config
# + topic + the ledger's per-leg history.  MUST be resolved deliberately at the
# sim2real port boundary (the real robot's servo map): see
# docs/plans-and-designs/picrawler_sim2real_port.md.
# Per-leg color so each leg is visually distinguishable during calibration
# (lets the user identify which leg has e.g. a reversed servo direction).
# Robotics convention was INTENDED as FL=red … but see the mirror note: red is
# the anatomical FRONT-RIGHT.
const LEG_COLORS: Array = [
	Color(0.95, 0.20, 0.20, 1.0),   # FL (front-left)  red
	Color(0.20, 0.85, 0.20, 1.0),   # FR (front-right) green
	Color(0.30, 0.50, 0.95, 1.0),   # RL (rear-left)   blue
	Color(0.95, 0.85, 0.20, 1.0),   # RR (rear-right)  yellow
]

# Episode failure thresholds (chassis-COM-relative).  No paired
# "alive" thresholds — the reward is graded by chassis_y via the
# events.hit duty cycle, so there is no binary alive/dead band.
#
# Phase 7.14 — FAIL_TILT is now @export so curricula can relax it for
# walking stages.  Default 1.05 rad (~60°) maintained for backward compat.
# Tighter values (smaller) trigger tipover earlier; looser (larger) give
# the body more tilt headroom during locomotion.  User UI observation
# (seed=50, cruse_v2 + stand_walk_gated, 24min): 58 tipover events ≈
# reward-landscape cliff at 60° preventing walking-tilt from earning
# reward.  Relaxing to ~80° (1.4 rad) in walking stages gives body
# room to lean forward during gait.
const FAIL_HEIGHT: float = 0.025     # below this = collapsed
@export var fail_tilt_rad: float = 1.05  # ~60° default — see comment above

# ---------------------------------------------------------------------------
# Exported runtime config (all overridable via env vars in _ready)
# ---------------------------------------------------------------------------
@export var config_path: String = "res://addons/ami_ogma/configs/the_picrawler_minimal.json"
@export var max_steps: int           = 6000
@export var max_episodes: int        = 0
@export var diag_interval_ticks: int = 60
# 2026-06-14 — verbose per-tick diag JSONL is for HEADLESS trajectory parsing.
# In the UI it just accumulates in Godot's output console (a 10 h run choked to
# ~5 fps).  Default on (headless needs it); the launcher disables it for long
# interactive runs so only meaningful events (episode/run end, target touched,
# falls) are logged.
@export var verbose_logging: bool = true
@export var leg_strength: float      = 1.0
# Stage 3.E (2026-06-01) — joint compliance via motor authority scale.
# Multiplies the effective per-tick motor torque cap (max_torque_powered)
# and motor_max_impulse.  Default 1.0 is bit-identical to pre-3.E.  Values
# <1.0 model SERVO-SAVER compliance: the saver's finite spring rate caps
# transmitted torque, so when external load exceeds motor authority the
# joint deflects against the commanded position — exactly the behavior of
# a real picrawler equipped with savers.  Sim2real: characterise the real
# saver (e.g. spring scale measurement of torque-vs-deflection), then set
# motor_authority_scale = (saver_max_transmissible_torque / MAX_SERVO_TORQUE).
# Joseph 2026-05-31 brainstorm: servo savers are the cheap real-hardware
# compliance retrofit.  This knob is the sim-side counterpart for sim2real
# matching.  See `docs/plans-and-designs/picrawler_diagnostic_calibration_plan.md`
# Stage 3.E.
@export var motor_authority_scale: float = 1.0

# 2026-06-01 — Stage 3.E++ motor-internal damping tuner.  Multiplies SERVO_KD
# inside _powered_torque() to optionally underdamp the motor PD.  Coupled to
# the motor (changes how the motor itself responds).  Default 1.0 = motor PD
# unchanged.
@export var motor_damping_factor:  float = 1.0

# 2026-08-10 — P7 SERVO_KI (body-fidelity candidate; PM's ForceBoostWiring analogue).
# Leaky integral of beyond-deadband servo tracking error, boosting BOTH the raw torque
# and the impulse cap: a stance leg the ground resists accumulates error and pushes
# harder — load-dependent timing, the within-leg thrust↔support coupling.
# ⚠ FRAME MAPPING: the hardware doc's Ki band (0.01–0.05, servo_dynamics.md) lives in
# the real servo's internal-loop units and does NOT transplant.  Here the boost is a
# FRACTION of motor_max_impulse on the real force path (imp_* → _set_motor_vf):
# boost_frac ≈ servo_ki × |beyond-deadband error| at integral saturation (τ = 0.4 s),
# so servo_ki {1, 3, 6} ≈ {10, 30, 60} % boost at a 0.1 rad sustained load error.
# (First build attached to _powered_torque and produced BIT-IDENTICAL trajectories —
# that function is telemetry-only; the lesson is inline there.)
# Guards (three windup precedents + the freeplay drift-twitch limit cycle): integral
# FROZEN inside the deadband (output stays 0 there regardless), leak ALWAYS applied,
# contribution clamped to ≤ SERVO_KI_AUTH_FRAC × max_torque.  0 = off, byte-identical.
@export var servo_ki: float = 0.0
const SERVO_KI_TAU: float = 0.4          # leak time constant, seconds
const SERVO_KI_AUTH_FRAC: float = 0.5    # boost cap, fraction of max_torque
var _ki_int: Array[float] = []           # 12 accumulators, [joint*4 + leg]
var _ki_mag_acc: float = 0.0             # diag: mean |integral|
var _ki_mag_n: int = 0
var _ki_boost_ticks: int = 0             # diag: boost-active duty
var _ki_total_ticks: int = 0

# 2026-06-01 — knee-widening A/B toggle.  Default true preserves the
# 2026-06-01 widening (KNEE_LIMIT_LOW=-2.50, asymmetric KNEE_RANGE_
# EXTEND/BEND mapping from f9ab634).  Set false to restore the pre-
# widening behavior: KNEE_LIMIT_LOW=-1.70 (symmetric ±97°), symmetric
# KNEE_RANGE=1.40 mapping (i.e. brain u→knee target uses the same
# range factor as hip1/hip2).  Used by the n=5 paired A/B validating
# whether the knee widening had behavioral effect.  Custom setter
# re-applies the joint LOWER limit live when the toggle flips so
# curriculum overrides take effect without rebuilding the body.
@export var knee_widening_enabled: bool = true: set = _set_knee_widening_enabled

# 2026-06-03 — Joint backend selector.  Two physically distinct picrawler
# bodies share this script:
#   "hinge" — original HingeJoint3D rigid hobby-servo legs.  Reproduces
#             ALL historical baselines and registry §3 metrics.  Default.
#   "g6dof" — Generic6DOFJoint3D legs with adjustable angular spring +
#             damping for the gait-resonance / passive-compliance line
#             ([[v6-apply_torque_spring_falsified]] redux).  Opt-in via
#             launcher dropdown.
# Live-switchable would require destroying + rebuilding all 12 joints —
# we don't support that; the backend is chosen at body launch and
# persists for the run.  Set via OGMA_PICRAWLER_JOINT_BACKEND env var
# or the launcher's body-backend dropdown.
@export var joint_backend: String = "hinge"

# 2026-08-02 · IMPORT I6 — reduced-gravity SCAFFOLD (see _build_body for the full note).
# Every Playful Machine legged experiment runs at gravity -6 vs Earth's -9.81; all of
# their emergence results are measured under it.  1.0 = off = byte-identical.
# PM-equivalent = 0.61.  Settable per-run via OGMA_PICRAWLER_GRAVITY_SCALE.
@export var scaffold_gravity_scale: float = 1.0

# 2026-08-02 · IMPORT I4 — colored proprioceptive noise (see the publish site for the
# full note).  PM wires every legged controller through ColorUniformNoise(0.1); our
# proprio channel has been noiseless.  sigma = 0 (default) is byte-identical.
# PM-equivalent sigma ≈ 0.1 of range.  tau = correlation length in ticks (1 = white).
# Per-run overrides: OGMA_PICRAWLER_SENSOR_NOISE / OGMA_PICRAWLER_SENSOR_NOISE_TAU.
@export var sensor_noise_sigma: float = 0.0

# 2026-08-03 · COMPLIANCE SCAFFOLD-FREE TEST — global damping multipliers.
# PM's dog sets dampingFactor = 0.0: hip, knee and ankle damping are ALL multiplied by
# zero, on top of backdrivable velocity servos.  Their bodies are essentially undamped.
# Ours is damped at THREE levels — SERVO_KD=8.0 (servo velocity term), BODY_ANGULAR_DAMP
# =8.0 / BODY_LINEAR_DAMP=2.0 (rigid-body), joint_angular_damping=0.3 (constraint).
# Homeokinesis works by finding and amplifying the loop's OWN dynamics; a body that
# dissipates energy this fast has no dynamics left to find.  1.0 = off (byte-identical).
# The SERVO term already has a knob -- `motor_damping_factor` above (multiplies SERVO_KD
# in _powered_torque); this adds the RIGID-BODY term, which had none.  Env overrides:
# OGMA_PICRAWLER_MOTOR_DAMP (existing knob) / OGMA_PICRAWLER_BODY_DAMP_SCALE (this one).
@export var body_damp_scale: float = 1.0: set = _set_body_damp_scale

func _set_body_damp_scale(v: float) -> void:
	body_damp_scale = maxf(0.0, v)
	# Apply to already-built bodies so it is tunable LIVE in the UI ([ and ] keys).
	# Guarded: the export setter can fire before _build_body has run, in which case
	# _build_body applies the value itself.
	if not is_instance_valid(_chassis):
		return
	var parts: Array = [_chassis]
	if _coxas != null:  parts += (_coxas as Array)
	if _uppers != null: parts += (_uppers as Array)
	if _lowers != null: parts += (_lowers as Array)
	for b in parts:
		if is_instance_valid(b):
			b.angular_damp = BODY_ANGULAR_DAMP * body_damp_scale
			b.linear_damp  = BODY_LINEAR_DAMP * body_damp_scale
@export var sensor_noise_tau:   float = 8.0
var _sensor_noise: PackedFloat64Array = PackedFloat64Array()
var _prev_joints_pub: PackedFloat64Array = PackedFloat64Array()  # for the joints_dyn q̇
var _qdot_ema: PackedFloat64Array = PackedFloat64Array()          # M0.d.2 smoothed velocity
## joints_dyn q̇ EMA rate (τ ≈ 1/alpha ticks; 0.3 ≈ 3-tick smoothing, ~servo timescale).
@export var joints_dyn_vel_alpha: float = 0.3
# Ground-force / authority accumulators (see the joint_torque publish site).
var _tq_mag_acc: float = 0.0
var _tq_sat_acc: float = 0.0
var _tq_n: float = 0.0
var _sensor_noise_rng: RandomNumberGenerator = RandomNumberGenerator.new()

# 2026-06-02 — Per-joint-type adjustable suspension (Generic6DOFJoint3D
# spring on the free angular axis).  Joseph's gait-resonance bet:
# passive compliance lets the body discover its own natural stride
# frequency via the active-inference Premotor upgrade downstream.
# All zero by default = rigid hinge behaviour (back-compat).  Units:
# stiffness in Nm/rad, damping in Nm·s/rad, equilibrium fixed at 0
# (the joint's neutral position).  See [[v6-apply_torque_spring_falsified]]
# — this is the physics-solver path that earlier apply_torque attempts
# were the wrong tool for.
@export var hip1_spring_stiffness: float = 0.0: set = _set_hip1_spring_stiffness
@export var hip1_spring_damping:   float = 0.0: set = _set_hip1_spring_damping
@export var hip2_spring_stiffness: float = 0.0: set = _set_hip2_spring_stiffness
@export var hip2_spring_damping:   float = 0.0: set = _set_hip2_spring_damping
@export var knee_spring_stiffness: float = 0.0: set = _set_knee_spring_stiffness
@export var knee_spring_damping:   float = 0.0: set = _set_knee_spring_damping

func _set_hip1_spring_stiffness(v: float) -> void:
	hip1_spring_stiffness = max(0.0, v)
	_apply_joint_springs()
func _set_hip1_spring_damping(v: float) -> void:
	hip1_spring_damping = max(0.0, v)
	_apply_joint_springs()
func _set_hip2_spring_stiffness(v: float) -> void:
	hip2_spring_stiffness = max(0.0, v)
	_apply_joint_springs()
func _set_hip2_spring_damping(v: float) -> void:
	hip2_spring_damping = max(0.0, v)
	_apply_joint_springs()
func _set_knee_spring_stiffness(v: float) -> void:
	knee_spring_stiffness = max(0.0, v)
	_apply_joint_springs()
func _set_knee_spring_damping(v: float) -> void:
	knee_spring_damping = max(0.0, v)
	_apply_joint_springs()

# 2026-06-03 — G6DOF angular constraint params, live-tunable via the
# Godot Inspector during UI runs.  Each setter re-applies the value to
# all 12 leg joints immediately so operators can see physics changes
# without restarting the run.  Defaults chosen to roughly match the
# implicit HingeJoint3D characteristics, but Joseph observed
# rapid-servo / high-frequency oscillation indicating under-damping —
# these knobs let him calibrate live.
@export var joint_angular_damping:        float = 0.3: set = _set_joint_angular_damping
@export var joint_angular_limit_softness: float = 0.95: set = _set_joint_angular_limit_softness
@export var joint_angular_erp:            float = 0.8: set = _set_joint_angular_erp

# Live multiplier on the per-tick motor torque cap (max_torque_powered =
# MAX_SERVO_TORQUE × leg_strength × motor_authority_scale).  Defaults to
# 1.0 in both backends.  Originally added as a unit-conversion diagnostic
# during the G6DOF migration — the backend-aware _set_motor_vf helper now
# does the right per-backend conversion (Hinge divides Nm by physics_hz
# to get MAX_IMPULSE, G6DOF passes Nm through to FORCE_LIMIT), so this
# knob is now just a tuning slider for whichever backend is active.
@export var motor_force_scale:            float = 1.0

func _set_joint_angular_damping(v: float) -> void:
	joint_angular_damping = max(0.0, v)
	_apply_joint_solver_params()

func _set_joint_angular_limit_softness(v: float) -> void:
	# Floor at 0.05 — softness=0 makes Bullet's angular limit constraint
	# infinitely stiff and the solver locks up (Joseph UI observation
	# 2026-06-03: "all motion stops and does not restart without stopping
	# the sim").  0.05 still produces a hard-ish limit but the solver
	# stays numerically alive.
	joint_angular_limit_softness = clamp(v, 0.05, 1.0)
	_apply_joint_solver_params()

func _set_joint_angular_erp(v: float) -> void:
	joint_angular_erp = clamp(v, 0.0, 1.0)
	_apply_joint_solver_params()

func _apply_joint_solver_params() -> void:
	# G6DOF-only params — no-op in hinge mode (which uses Bullet defaults
	# for LIMIT_BIAS/RELAXATION baked at construction in _make_hinge_joint).
	if joint_backend == "hinge":
		return
	for j in (_hip1_joints + _hip2_joints + _knee_joints):
		# Hinge axis lives on local +X in G6DOF (see _make_g6dof_joint remap).
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_DAMPING, joint_angular_damping)
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LIMIT_SOFTNESS, joint_angular_limit_softness)
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_ERP, joint_angular_erp)

var KNEE_LIMIT_LOW_NARROW: float = -1.70  # pre-widening (toggle=false)
var KNEE_RANGE_SYMMETRIC:  float =  1.40  # pre-widening (toggle=false), == HIP_TARGET_RANGE

func _set_knee_widening_enabled(value: bool) -> void:
	var changed: bool = (knee_widening_enabled != value)
	knee_widening_enabled = value
	if changed and not _knee_joints.is_empty():
		var limit_low: float = KNEE_LIMIT_LOW if knee_widening_enabled else KNEE_LIMIT_LOW_NARROW
		for j in _knee_joints:
			if j is HingeJoint3D:
				(j as HingeJoint3D).set_param(HingeJoint3D.PARAM_LIMIT_LOWER, limit_low)
			else:
				# G6DOF hinge axis remapped to X — see _make_g6dof_joint.
				(j as Generic6DOFJoint3D).set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, limit_low)
		print("PicrawlerBody: knee_widening_enabled=%s — knee LOWER limit now %s rad" % [value, limit_low])

# 2026-06-01 — Stage 3.E++ motor FREE-PLAY (mechanical slop zone).  When
# |target_angle - current_angle| < motor_freeplay_rad, the motor disengages
# (target_velocity=0, max_impulse=0): joint is free to drift inside the zone.
# Outside the zone, the motor PD engages normally.  Models real hobby-servo
# mechanics: rigid hold at target, slop within ±N° around it.
#
# NOTE 2026-06-01 — an apply_torque-based PASSIVE SPRING was added alongside
# this to provide a restoring force within the free zone (servo-saver
# analog).  That spring was REVERTED in commit XXXXXXX: explicit-Euler
# integration of K*(angle-target) + D*ω via apply_torque is fundamentally
# unstable for low-inertia leg segments (I≈5e-6 kg·m²) at 60Hz physics.
# The native fix is `Generic6DOFJoint3D` with built-in angular spring
# (constraint-level integration in Bullet).  Deferred to a separate
# migration PR; until then, motor_freeplay_rad creates a deadband but no
# centering force — joint floats freely inside the zone.
#
# ⚠ 2026-08-03 — THAT NOTE IS NOW HALF-STALE.  The native Generic6DOFJoint3D angular
# spring IS enabled on the g6dof backend (FLAG_ENABLE_ANGULAR_SPRING, with the preset's
# stiffness/damping), so there IS a restoring force.  But its equilibrium point was
# hardcoded to 0.0 = mechanical neutral, which makes it a RETURN-TO-NEUTRAL spring
# fighting every commanded angle, not compliance about the command.  See
# spring_follows_target, which parks the equilibrium on the servo target instead.
#
# Recommended starting values:
#   motor_freeplay_rad = 0.087  (~5°  — typical hobby-servo-saver play)
#   motor_freeplay_rad = 0.17   (~10° — generous, lets gravity sag visibly)
# Default 0.0 ⇒ no free play, motor engages at any error > SERVO_DEADBAND.
@export var motor_freeplay_rad:    float = 0.0

# 2026-08-03 — SPRING EQUILIBRIUM TRACKING.  The g6dof angular spring's equilibrium
# point was hardcoded to 0.0 (mechanical neutral) both at construction and in
# _apply_spring_to.  That makes it a SERVO-SAVER RETURN SPRING: it pulls every joint
# toward neutral at all times, fighting whatever the controller commands, and inside
# the freeplay deadband it drags a stance-holding leg back toward zero instead of
# holding it compliantly.  It is almost certainly why the g6dof substrate measured
# WORSE than rigid hinge (net_z 4.49 -> 3.51) — the springs were fighting the gait.
#
# What a springy JOINT should be is a series-elastic element: compliance ABOUT THE
# COMMANDED ANGLE.  With this on, the equilibrium point tracks the servo target each
# tick, so stiffness sets how hard the joint is pulled back to where it was TOLD to
# be — which is the centering force in the freeplay zone that makes freeplay+springs
# behave like a real compliant actuator rather than slop plus a return spring.
#
# DEFAULT CHANGED TO TRUE 2026-08-03 (operator): a spring that does not follow the
# command is not compliance, it is a return-to-neutral bug, so the correct behaviour is
# the default rather than a lever.  The flag is kept so the old behaviour can still be
# reproduced for comparison.
# ⚠ CONSEQUENCE: every g6dof result measured before this date used equilibrium=0 and is
# NOT comparable to anything measured after.  That includes the 2026-08-03 hinge-vs-g6dof
# comparison (net_z 4.49 -> 3.51, PLV 0.138 -> 0.083), which is now known to have been
# measured with springs pulling every joint toward neutral and fighting the gait.  Re-run
# before citing it.  `hinge` is unaffected — it has no springs.
@export var spring_follows_target: bool = true

# Stage 3.D (2026-06-01) — Bernoulli-impulse actuation backend.
# Port of v4 Phase 6.0.a body-as-integrator from the_cell. Replaces the
# default "discrete" target-angle pipeline with a per-joint spike sampler:
# the brain's commanded u in [-1,1] sets a Bernoulli fire rate per tick;
# on fire, the joint's target angle moves by `bri_impulse_per_spike`; each
# tick the integrated offset decays toward rest by `bri_friction_per_tick`.
# Smoothness emerges from the friction-integration of many discrete spikes,
# not from smoothness of the brain's commands. See
# `docs/plans-and-designs/picrawler_diagnostic_calibration_plan.md` Stage 3.D
# and `docs/storytelling/glossary.md` "Bernoulli-Impulse Actuation".
#
# Default `actuation_backend = "discrete"` is bit-identical to pre-3.D.
@export_enum("discrete", "bernoulli_impulse") var actuation_backend: String = "discrete"
@export var bri_base_rate:          float = 0.5    # baseline spike prob per tick per joint
@export var bri_command_bias:       float = 0.5    # |u| asymmetrises rate by this
@export var bri_impulse_per_spike:  float = 0.05   # fraction of TARGET_RANGE moved per spike
@export var bri_friction_per_tick:  float = 0.05   # offset decays toward rest by this fraction

@export var reset_mode: String       = "continuous"
@export var mc_episode_period: int   = 0     # 0 = no periodic MC ep_end in continuous mode
# Auto-reset on inversion (training safety net).  When the chassis is
# both inverted (tilt past auto_reset_tilt_threshold) AND on the floor
# (chassis_y below auto_reset_max_height) for auto_reset_dwell_ticks
# consecutive physics ticks, queue a hard reset to the standing pose.
# Disabled by default — the brain MUST learn to recover from any
# recoverable posture (e.g. laying on its side, tilt ≈ π/2) on its own.
# The thresholds here are intentionally conservative: tilt > 2π/3 ≈ 120°
# AND chassis_y < 0.03 m means the chassis is past on-its-side and the
# body is genuinely belly-up; the legs cannot get purchase from this
# posture and resetting accelerates learning without removing a learnable
# correction.  Logged per-run as auto_reset_count in the JSONL.
@export var auto_reset_on_inversion:    bool  = true
@export var auto_reset_tilt_threshold:  float = 2.094   # ~120°  — past on-its-side
@export var auto_reset_max_height:      float = 0.030   # m       — chassis on floor
# ⚠ auto_reset_max_height is an ORIGIN height, and the origin's height when the
# body is resting on its back depends on the chassis SHAPE.  The 0.030 default
# was calibrated against the legacy single box, whose top sits CHASSIS_Y/2 =
# 21 mm above the node origin — so it really encoded "9 mm of slack above the
# height this body rests at when inverted".
#
# The measured body comes to rest on the RPi / Robot-HAT stack, whose top is
# 77 mm above the origin.  Inverted, chassis_y ≈ 0.077 — never below a fixed
# 0.030 — so the belly-up reset could NEVER fire on the new chassis.
#
# Fix: preserve the SLACK, not the absolute number, keying off _chassis_top_local.
#   cad      → 0.021 + 0.009 = 0.030  (byte-identical to the old behaviour)
#   measured → 0.077 + 0.009 = 0.086  (fires when down on the HAT block)
const _LEGACY_INVERTED_REST_H: float = 0.021   # legacy CHASSIS_Y/2, the calibration ref
@export var auto_reset_dwell_ticks:     int   = 30      # ticks   — ~0.5 s at 60 Hz
# Phase 6.14 — outer-wall reset.  When ON, fires _pending_manual_reset
# the moment chassis horizontal distance from origin exceeds
# AUTO_RESET_OUTER_WALL_RADIUS (default 9.5 m — just inside the wedge
# ring at r=10).  Avoids the wall-climb-then-tumble cycle the user
# observed: body finds the slope fun, struggles to escape it, eventually
# falls off and ends up belly-up several metres outside the floor.
# OFF by default — preserve baseline experimentation surface.
@export var auto_reset_on_outer_wall: bool = false
const AUTO_RESET_OUTER_WALL_RADIUS: float = 9.5  # m
# B3 leg-symmetric weight sharing.  When non-"off", Premotor weight
# matrices (W, b, E) are averaged across mirrored leg pairs every
# mc_episode_period ticks.  Tests whether forcing symmetry breaks the
# observed per-leg policy divergence (W_norm spread 4.3→8.9 in 30-min runs).
#   "off"               — independent legs (default, baseline behaviour)
#   "lr_pairs"          — left/right pairs:  {FL↔FR, RL↔RR} for hip1 + pitch
#   "lr_and_fr_pairs"   — also front/rear: averages all 4 legs together
#                         per joint group (most aggressive symmetry).
# Requires mc_episode_period > 0 — sync fires on the same boundary as
# the MC REINFORCE finalise.  No-op if disabled or if get_module_snapshot
# / set_module_snapshot aren't available (older brain build).
@export var leg_symmetry_mode: String = "off"
# 2026-06-10 E1b — LIGHT front-rear posture coupling.  When > 0 (and mode is
# lr_pairs), after the hard L/R averaging each leg-pair's weights are BLENDED a
# fraction α toward the all-four-leg grand mean: W_pair ← (1−α)·W_pair + α·grand.
# This gently pulls front and rear postures toward symmetry (fixing the seed-
# random asymmetric "sit" where one pair folds and the other extends/drags)
# WITHOUT the hard lr_and_fr_pairs collapse to one identical policy (which would
# forbid the inter-leg phase differences a gait needs).  Per Joseph: "all legs
# could share symmetry if the influence is light enough."  0 = off.  Light = 0.1–0.3.
@export var leg_symmetry_fr_blend: float = 0.0
@export var physics_hz: int          = 240
@export var solver_iterations: int   = 64    # more iterations = constraint chain converges
											  # cleanly under contact load (was 32; with 12-DOF
											  # chain + 4-foot ground contact, 32 left residual
											  # constraint error → ragdoll twitching).

const _RESET_MODE_CHOICES := ["continuous", "soft_blink", "instant_pause"]

# Layers — keep body parts off the world layer so legs don't
# self-collide while still hitting the floor.
const _LAYER_BODY:    int = 1 << 1
const _LAYER_WORLD:   int = 1 << 0
const _LAYER_CHASSIS: int = 1 << 2   # chassis-only layer — floor.mask does
									 # NOT include this, so chassis never
									 # touches floor.  Prevents the box-on-
									 # floor rolling instability that flips
									 # the body upside-down even when
									 # leg_strength=0.

# ---- 2026-08-04 · THE CHASSIS IS A GHOST, AND IT IS LOAD-BEARING FOR THE RECORD ----------
# Operator observation: "when I lesion both rear legs the entire back of the robot sinks into
# the ground, because the chassis itself does not collide — only the legs do."  Correct, and
# `_LAYER_CHASSIS` appears in exactly two places in this file (the const and the assignment):
# NOTHING masks it, in either direction.  The chassis has a real BoxShape3D and passes
# straight through the world.
#
# It was deliberate, not an oversight — see the comment above: it suppresses a box-on-floor
# rolling instability that flipped the body even at leg_strength=0.  So enabling it may bring
# that instability back, and that is itself worth knowing.
#
# ⚠ WHAT IT MEANS FOR THE RECORD.  Every belly/clearance result in the ledger was measured on
# a body that CANNOT touch the ground with its belly.  `bellyc`/`gc_raw` is a downward
# rangefinder, so it reports the gap honestly — but nothing ever stopped the gap going to
# zero, and `bellyc_min` has been sitting at 0.000–0.004 for the whole campaign.  The ledger
# already suspected the consequence and named it as "may be a sim exploit (frictionless belly
# drag)" for hump traversal; the real mechanism is worse than frictionless drag — there is no
# belly contact at all.
#
# DEFAULT OFF, so every historical number stays reproducible and this is a LEVER rather than a
# silent re-basing of the whole campaign (CLAUDE.md §3: gain-0-guarded, A/B'd, then promoted
# on evidence).  Turn it on with the export, OGMA_PICRAWLER_CHASSIS_COLLIDE=1, or [J].
@export var chassis_collides: bool = false
# Sliding friction of the chassis shell against the world, INDEPENDENT of the feet (mu=1.5)
# and the climbing wedges (3.0).  0.20 ~ plastic on concrete.  Raise it to make a downed
# robot stick where it falls; drop it toward 0 for a body that slides freely on its belly.
# Live on the [K] panel and via OGMA_PICRAWLER_CHASSIS_FRICTION.
@export var chassis_friction: float = 0.20
# Sliding friction of the NON-FOOT limb segments (coxa + upper).  Measured 2026-08-04: with
# two dead rear legs the chassis never actually touches the floor (belly gc_raw 0.057) --
# the LEG SEGMENTS are what rests on it, and they were carrying the same mu = 1.5 as the
# feet.  So the operator's principle ("the feet should have the highest friction, not the
# chassis") binds here, not on the shell: a coxa and a femur are plastic linkages, not
# rubber pads.  The lower legs / feet deliberately KEEP _make_contact_mat() at 1.5.
# DEFAULT 1.5 = byte-identical to every historical run; lower it to let a crippled robot
# drag itself.  Live on the [K] panel and via OGMA_PICRAWLER_LIMB_FRICTION.
@export var limb_friction: float = 1.5

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
@onready var brain: OgmaBrain = $Brain
var _chassis: RigidBody3D
# Set by _rebuild_body() so a live body swap reconstructs AT THE CURRENT POSE
# instead of respawning at the origin — otherwise the morphological (d) test is
# confounded by a teleport.  Ignored on the initial _ready() build.
var _body_spawn_xform: Transform3D = Transform3D.IDENTITY
var _body_spawn_valid: bool = false
# Optional multi-box chassis (the measured body: base plate + electronics stack).
# EMPTY means the legacy single-box path runs untouched — that is what keeps
# cad.json byte-identical rather than merely equivalent.
var _chassis_boxes: Array = []
# Explicit chassis centre of mass, in chassis-local coords.  A RigidBody3D has
# ONE mass, so an internal distribution can only be expressed this way.
var _chassis_com: Vector3 = Vector3.ZERO
var _chassis_com_valid: bool = false
# Height of the HIGHEST chassis surface above the node origin — i.e. the surface
# the body comes to rest on when INVERTED.  Legacy single box: CHASSIS_Y/2.
# Measured two-box body: the top of the RPi / Robot-HAT stack.  The belly-up
# auto-reset keys off this; see _LEGACY_INVERTED_REST_H.
var _chassis_top_local: float = 0.021
# Walking-trail UI helper (sibling node found at _build time); cached so
# _do_hard_reset() can wipe its X-markers without re-traversing the tree.
# Burst-onset probe state — see the trigger in _clip_record().
# Scaffold motor-intent (see the publish site).  intent_fwd < 0 disables the publisher, so
# the intent socket is unfed and commit_prec_gain stays inert — the byte-identical default.
var _gng_probe_printed: bool = false
# ---------------------------------------------------------------------------
# 2026-08-06 — PER-TICK ATTRIBUTION TRACE (instrument; OFF unless OGMA_PICRAWLER_TRACE
# names an output path, so the default build is byte-identical).
#
# Answers two questions off one logging pass:
#   (1) does the velocity channel (`delta`) carry stride phase, or is it noise?
#       -> autocorrelation of per-joint d(angle) at the stride lag.
#   (2) WHICH LEG is responsible for a given forward-velocity pulse?
#       -> per-joint mechanical power P = tau * omega, stance-gated by contact,
#          integrated between fwd_v zero crossings and regressed on the pulse impulse.
#
# ⚠ THIS IS A GOD'S-EYE INSTRUMENT AND MUST STAY ONE.  fwd_v and foot contact are
# not egocentric; they are legal here because diagnostics may use god's-eye and
# CONTROL MAY NOT (CLAUDE.md §5.3).  `feet_y` already became a live god's-eye
# dependency in the deployed gait by exactly this route, so: nothing computed from
# this trace may be fed back into the brain.  The egocentric shadow of it already
# exists and is unconsumed -- `reality.proprio.joint_torque`, a 12-D load sensor --
# which is the legal path to an internal mechanism once the attribution is proven.
var _trace_file: FileAccess = null
var _trace_ready: bool = false

@export var intent_fwd: float = -1.0
@export var intent_yaw: float = 0.0
var _burst_probe: bool = false
var _bp_last_lifts: int = 0
var _bp_last_lift_tick: int = 0
const _BURST_GAP_TICKS: int = 40      # silence that qualifies as a real pause (~0.67 s)
# Time-scale ladder for [,] slower / [.] faster / [/] reset.  Below 1.0 is exact; above 1.0
# saturates against max_physics_steps_per_frame (see the key handler).
const _TIME_SCALES: Array = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
var _time_scale_v: float = 1.0
# ---- 2026-08-05 · EXPLICIT TICK WINDOW (operator) --------------------------------------
# "the step pause cycle is about eight hundred ticks long... capture tick 2000-3000 in
# _stroke12; for the first 1k ticks the robot is still learning to walk properly."
#
# This supersedes the auto-trigger for the burst question, and the reason matters: the
# trigger keys on _leg_lifted_count, the SWING-DETECTOR's event, which fires ~8x more often
# than a real step (it reported a 95-tick median gap against the operator's observed ~800).
# So it was slicing the run on the wrong events entirely.  A named window makes no claim
# about what a step IS — it just hands over every tick in the interval and lets the analysis
# find the structure.  OGMA_PICRAWLER_CLIP_WINDOW="2000,3000".
var _cw_from: int = -1
var _cw_to: int = -1
var _cw_saved: bool = false
var _walking_trail: Node = null
# ---- 2026-08-05 · PER-METRE WAYPOINTS (operator) ---------------------------------------
# "the best way to measure the path is to get a time stamped location for every meter the
# robot traverses" — i.e. the same object the red [P] trail draws, but in the log, so the
# headless analysis and the picture the operator trusts are THE SAME MEASUREMENT.
# A whole-run mean hides the thing that actually characterises this gait: it moves in
# BURSTS of 2-4 steps separated by fumbles, and the per-metre TIMING is what exposes that
# (a fast metre = a burst held together; a slow metre = fumbles inside it).
# Emitted as their own JSONL lines so they survive any diag_interval.
var _wp_last_xz: Vector2 = Vector2.ZERO
var _wp_path_len: float = 0.0
var _wp_next_m: float = 1.0
var _wp_last_tick: int = 0

# ---------------------------------------------------------------------------
# GOOD/BAD CLIP MARKER (2026-07-27) — the operator -> measurement channel.
#
# The operator can see gait quality in the UI long before any aggregate metric moves, and
# describing it in words is lossy in a SPECIFIC way: this project has three recorded cases
# where the reported ACTION was right and the reported MECHANISM was wrong ("the swing leg
# spins the chassis" — yaw impulse measured LOWER during swing; "a vertical shank gives
# mechanical advantage" — foot radius barely moved; "shorter steps bring the feet in" —
# foot radius invariant).  Each cost a build plus a seed-averaged A/B before an instrument
# caught it.
#
# So instead of describing it: MARK it.  [F1] while it looks right, [F2] while it looks
# wrong, and `scripts_tools/clipdiff.py` reports what actually differs between those
# windows.  The operator stays the authority on what "good" means; the numbers say what
# good CONSISTS of, which is the part words keep getting wrong.
#
# The existing JSONL is emitted at the diag cadence (~1 Hz) — far too coarse for a 26-tick
# step — so this keeps its own narrow per-tick ring.  Body-side signals only: everything
# else (duty, periods, inter-leg phase, mv_stance) is derivable offline.
#
# ⚠️ THIS IS AN INSTRUMENT, NEVER A FITNESS.  Nothing here enters a loop the brain can
# optimize; that would be reward shaping, which CLAUDE.md §5.1 prohibits outright.  It
# selects which lever to chase.  The brain never sees it.
const _CLIP_RING_LEN: int = 900          # ~15 s at 60 Hz
var _clip_ring: Array = []               # ring of per-tick Dictionaries
var _clip_head: int = 0
var _clip_count: int = 0                 # total ticks recorded (< LEN = partial)
var _clip_seq: int = 0                   # clip index within this run
var _clip_dir: String = ""               # resolved lazily on first save

# Trainer-pulse counters — incremented by publish_trainer_event() and
# emitted in every _emit_jsonl line so the audit trail is complete.
# Headless A/Bs must see 0 for both across every diag line; a non-zero
# count means trainer pulses contaminated the run.  Counters are NOT
# reset between diag emissions — they're cumulative per run so the tail
# of the JSONL stream tells the full story.
var _trainer_good_count: int = 0
var _trainer_bad_count:  int = 0

# Auto-reset-on-inversion dwell tracking.  Counter increments while the
# inverted-on-ground condition holds; resets to 0 when the body leaves
# that posture; on reaching auto_reset_dwell_ticks, queues a hard reset.
# auto_reset_count is cumulative per run for the JSONL audit trail.
var _auto_reset_dwell_counter: int = 0
var auto_reset_count:         int = 0
# Phase 6.7 — events.alive heartbeat toggle.  Set via env
# OGMA_PICRAWLER_ALIVE_HEARTBEAT=1.  When true, every events.hit firing
# also emits events.alive once that tick (replenishing HomeostaticDrive's
# alive_pulse channel so urgency oscillates instead of saturating at 1.0).
# Off by default → baseline picrawler urgency dynamics unchanged from
# pre-Phase-6.7.  Required for HomeokineticExploration's gate to fire on
# this body; baseline runs leave it off to preserve bit-identity with
# Stage B/C reference baselines.
var _publish_alive_heartbeat: bool = false
# Phase 6.7 — body-side entropy-collapse escape detector.  Adaptive
# "stuck-pose breaker": when a Premotor's intent entropy has collapsed
# (pre_H_ema below 50% of its historical peak) AND the body hasn't
# translated (chassis_xz immobile) AND DA has decayed to floor (no
# reward firing), we inject Gaussian noise into that Premotor's W
# matrix via the get_module_snapshot / set_module_snapshot round-trip
# Stage B's leg_symmetry sync uses.  Mechanism-shaped (no static
# thresholds — H_min derived from each Premotor's own historical max).
# Targets the SAME failure mode HomeokineticExploration's gate would
# detect, but fires on the symptom (entropy collapse) we directly
# observed in the user's 1h47m snapshot, not on the proxy (urgency
# variability) that requires the late-stage stuck state to manifest.
#
# Off by default (env OGMA_PICRAWLER_ESCAPE_DETECTOR=1 to enable).
# When off, _check_escape_detector is a no-op and all related JSONL
# columns are 0 — column shape invariant across baseline and variant.
var _escape_detector_enabled: bool = false
# Debug toggle (OGMA_PICRAWLER_ESCAPE_FORCE_FIRE=1).  Bypasses the
# chassis-stuck and DA-floor gates, leaving only entropy-collapse +
# peak-min + cooldown.  Used for smoke-testing the snapshot round-trip
# kick path without waiting for a stuck pose to manifest naturally.
# NEVER set this in an A/B variant — it makes the detector fire on
# any confidently-converged Premotor, kicking healthy policies too.
var _escape_force_fire: bool = false
var _pre_h_peak: Dictionary = {}             # per Premotor id → max H ever seen
var _pre_h_ema:  Dictionary = {}             # per Premotor id → smoothed current H
var _chassis_xz_history: Array = []          # deque of recent Vector2 chassis positions
var _escape_cooldown_until: Dictionary = {}  # per Premotor id → tick to wait until
var _escape_fired_total: int = 0
var _escape_fired_per_pm: Dictionary = {}    # per Premotor id → count
var _escape_active_now_count: int = 0        # # Premotors kicked this diag tick
var _escape_rng: RandomNumberGenerator = RandomNumberGenerator.new()
# Escape-detector tuning parameters (kept as constants so the
# mechanism is reproducible; if these turn out to need tuning the
# right move per `feedback-no-tuning` is making them self-adaptive,
# not bumping the value).
const _ESC_COLLAPSE_FRACTION: float = 0.5    # fire when pre_h_ema < 0.5 * peak
const _ESC_MIN_PEAK_FOR_TRIGGER: float = 1.0 # ignore Premotors that never reached H≥1.0 (still in early training)
const _ESC_H_EMA_ALPHA: float = 0.2          # how fast pre_h_ema tracks raw H (per diag tick).  Set to converge over ~5 s of diag samples — fast enough to catch gradual drift before the chassis fully locks up, slow enough to not fire on per-tick noise.
const _ESC_MOTION_EPS: float = 0.05          # m; chassis must stay within this radius for stuck-ticks
const _ESC_STUCK_DIAG_SAMPLES: int = 10      # number of diag samples chassis must be still for (≈ 10 s @ 60-tick diag)
const _ESC_DA_FLOOR: float = 0.05            # DA below this counts as "no reward signal"
const _ESC_COOLDOWN_TICKS: int = 1500        # one MC episode of cooldown after a kick
const _ESC_KICK_SIGMA: float = 0.1           # absolute Gaussian σ added to each W element

# Phase 6.7++ — EPM memory swap on HomeokineticExploration fire.  The
# user-proposed alternative to motor-intent override: when HK detects
# entropy collapse, restore every EPM's GNG / TLE / health state to
# the pristine snapshot captured at body init (right after brain.setup
# returns).  Premotor weights, LateralVoter trust, NeurochemState, and
# HomeostaticDrive are UNTOUCHED — only the perceptual layer resets.
# Background: motor override (Phase 6.7 wiring + C.5 v5) was shown
# empirically destructive — random-accel coordination breaks picrawler's
# gait.  EPM swap preserves policy while giving the brain fresh
# perception to re-learn from.  See plan
# ~/.claude/plans/i-ve-observed-the-system-proud-lynx.md.
#
# Off by default.  Env OGMA_PICRAWLER_EPM_SWAP_ON_HK=1 to enable.
var _epm_swap_enabled: bool = false
var _epm_pristine_snapshots: Dictionary = {}      # module_id → JSON string
var _epm_swap_total: int = 0                      # monotonic
var _epm_swap_at_tick: int = 0                    # last swap tick (0 = never)
var _last_hk_episodes_armed: int = -1             # delta detector for the gate
const _EPM_SWAP_COOLDOWN_TICKS: int = 18000       # 5 min @ 60 Hz — prevent thrashing under HK's continuous fire
var _coxas:  Array[RigidBody3D] = []
var _uppers: Array[RigidBody3D] = []
var _lowers: Array[RigidBody3D] = []
# Untyped Array — holds HingeJoint3D OR Generic6DOFJoint3D depending on
# joint_backend selection.  Helpers (_set_motor_vf, _apply_joint_springs)
# branch on runtime type.
var _hip1_joints: Array = []
var _hip2_joints: Array = []
var _knee_joints: Array = []
var _hip2_axes:   Array[Vector3] = []   # per-leg lateral, used for torque application
var _knee_axes:   Array[Vector3] = []   # same — knee axis is the same lateral as hip2
# Construction-time world positions of the joint anchors per leg, used
# by the calibration FK to compute body transforms from slider angles.
var _hip1_world_c: Array[Vector3] = []
var _hip2_world_c: Array[Vector3] = []
var _knee_world_c: Array[Vector3] = []
# Y-lift currently applied by suspend (so FK can add it to construction
# positions to land bodies at the lifted-chassis height).
var _suspend_lift_y: float = 0.0

# Torque history for first-order lag (servo rise-time emulation).  Each
# servo's currently-applied torque carries over and the new PD command
# pulls it toward the target with alpha = 1-exp(-TAU/SERVO_TORQUE_RISE_TAU).
# 2026-08-06 — TRUE LOAD PROXY (velocity-tracking deficit).
#
# `_prev_torque_*` is NOT a load signal despite what joint_torque's description
# claimed.  It is the PD value used to set the motor's max IMPULSE CAP (an
# authority budget; the code calls _powered_torque "the telemetry path"), and it
# is tau = Kp*err - Kd*omega with Kp=20, Kd=8.  With |err| ~ 0.18 the Kp term is
# ~3.6 while omega reaches 6 rad/s making the Kd term ~24, so the published
# "torque" is DOMINATED BY VELOCITY DAMPING -- measured corr(tau, dtheta) =
# -0.46..-0.56 on all three joints.  Anything load-gating on it (Cruse Rule 5,
# a future epm_joint_torque) would have been gating on -omega.
#
# The motor is VELOCITY-controlled with an impulse cap, so the honest load
# signal is the deficit between the velocity it commanded and the velocity the
# world allowed: ~0 when free, large when the foot is planted and the motor
# stalls against its cap.  That is a load-cell analogue rather than a damping term.
var _prev_load_hip1: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _prev_load_hip2: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _prev_load_knee: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _prev_torque_hip1: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _prev_torque_hip2: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _prev_torque_knee: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Effective servo target — rate-limited at MAX_SERVO_SPEED.  The brain
# (or calibration) can step its commanded target instantly, but the
# servo's INTERNAL target only changes at most MAX_SERVO_SPEED·TAU per
# brain tick.  PD chases this moving target, so the joint can't
# overshoot its physical max-speed.  Matches how real servos work:
# the PWM cycle (50 Hz) is the bottleneck on how fast the commanded
# position can change.  Without this, with our 14 g legs + 240 Hz
# physics + 0.3 Nm peak torque, one tick produces ω ≈ 390 rad/s
# (vs servo max 8 rad/s) → joint limit overshoot → chassis kick →
# body flips over.
var _eff_target_hip1: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _eff_target_hip2: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _eff_target_knee: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Stage 3.D — Bernoulli-impulse backend state.  Per-joint accumulated offset
# from rest in units of TARGET_RANGE, clamped to [-1, 1].  Per-tick spike
# integration moves offset by `bri_impulse_per_spike × sign(u)`; per-tick
# friction decays offset toward 0 (joint relaxes to rest).  12 independent
# RNG streams (one per servo, deterministic-seeded off OGMA_SEED) so adding
# this backend doesn't perturb existing paired-seed comparisons under the
# default `actuation_backend = "discrete"`.
var _bri_offset_hip1: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _bri_offset_hip2: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _bri_offset_knee: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _bri_rng: Array = []   # populated in _ready with 12 RandomNumberGenerator
var _bri_spike_count_total: int = 0   # cumulative across run, for diag telemetry
var _bri_spike_count_this_diag: int = 0   # reset each diag emit; rate over diag interval

# Per-leg sign for hip1 calibration: rotating the chassis-mounted yaw
# servo by the SAME absolute angle on each leg pushes FL+RR outward and
# FR+RL INWARD (toward the chassis), because each leg's outward-pointing
# direction differs.  During calibration, multiply the commanded hip1_u
# by this sign so "hip1 +max" splays ALL legs outward symmetrically
# (the visual intent of the test).  Indices: FL, FR, RL, RR.
const HIP1_SPLAY_OUT_SIGN: Array = [+1.0, +1.0, +1.0, +1.0]
# 2026-06-07 — Joseph empirically verified via G calibration that flipping
# FR+RL signs (effectively cancelling the prior diagonal compensation) produces
# symmetric leg motion and unlocks walking out of the spider stance.  The
# diagonal compensation [+1,-1,-1,+1] was over-correcting.  Restored uniform +1
# across all 4 legs.  In brain mode the body emits per-Premotor u values that
# already encode any per-leg specialization; the SPLAY_OUT layer doesn't need
# to inject a sign distinction the substrate Premotors haven't learned.
# 2026-06-08 D2 diagnostic confirmed the sign flip alone doesn't change tabula
# rasa standing (D1 new sign / D2 old sign / D3 trot new sign all converged to
# alive_max≈1310 ticks).  Regression is elsewhere.

# Cached rest-pose transforms for teleport_to_standing.
var _coxa_rest_xform:  Array[Transform3D] = []
var _upper_rest_xform: Array[Transform3D] = []
var _lower_rest_xform: Array[Transform3D] = []
var _chassis_rest_xform: Transform3D
# Phase 7.x — auto-reset-in-place outside the central ring.  Pyramid XZ
# positions + bounding radii cached at terrain-build time so the in-place
# reset can nudge the spawn point away from any pyramid the chassis is
# currently inside / next to.  Populated in _build_terrain.
var _pyramid_xz_positions: Array = []   # Vector2 per pyramid (xz)
var _pyramid_xz_radii:     Array = []   # float per pyramid (bounding circle around the square base)
# Ignition-eval: donut radius bounds so a near pyramid is reachable inside a
# short run (default 3.5–9.5 m = legacy donut).  Override via
# OGMA_PICRAWLER_PYRAMID_MIN_R / _MAX_R; set min≈2.0 to bring obstacles close.
@export var pyramid_min_r: float = 3.5
@export var pyramid_max_r: float = 9.5
# 2026-06-13 — number of pyramids placed.  Default 30 = dense legacy donut
# (every non-target pyramid is an obstacle; the robot has no avoidance, so
# targets are frequently blocked).  Set small (e.g. 3) via
# OGMA_PICRAWLER_PYRAMID_COUNT for a sparse arena that isolates the nav loop
# from obstacle-blocking — used to verify indefinite target-to-target walking.
@export var pyramid_count: int = 30
# Heading telemetry cached in the proprio-publish step, emitted in _emit_jsonl:
# chassis yaw + body-frame outward unit vector (radial_compass).  Lets the
# aliveness panel score "faces radial-out and re-corrects against noise".
var _last_yaw: float = 0.0
var _last_lat_v: float = 0.0   # signed lateral (sideways-slip) velocity, for diag
var _last_radial_compass: Vector2 = Vector2.ZERO
# 2026-06-03 — target_compass perceptual stream cache (egocentric body-frame
# unit vector to active pyramid; (0, 0) when no target).
var _last_target_compass: Vector2 = Vector2.ZERO
# 2026-06-06 — TRUE (un-biased) target_compass for honest reward computation.
# `_last_target_compass` stores the cognitive-rotated value (what the brain sees);
# `_last_true_target_compass` stores the unrotated value (what reality says).
# Reward signals (leg_event v_toward_target) must use the TRUE value to avoid the
# self-reinforcing illusion loop where cognitive bias gates its own reward.
var _last_true_target_compass: Vector2 = Vector2.ZERO
# 2026-06-06 — per-pyramid engagement counter.  Sized to _pyramid_xz_positions
# when world is built.  Incremented each diag tick when chassis is within
# PYRAMID_ENGAGEMENT_RADIUS_SURFACE of pyramid i's bounding surface.  Lets
# us identify which non-target pyramids the body interacts with during nav
# experiments (per Joseph 2026-06-06).
var _pyramid_engagement_counts: Array = []
const PYRAMID_ENGAGEMENT_RADIUS_SURFACE: float = 1.0  # m of surface clearance
# Cached per-tick nearest pyramid index (-1 = no pyramids placed)
var _nearest_pyramid_idx: int = -1
# 2026-06-06 — hip1 symmetric yaw probe (diagnostic for what reliably turns the body).
# Original Cruse-asymmetric (hip2+knee) pattern was FALSIFIED on a standing-only body:
# the "outer leg longer reach" trick only produces yaw during gait cycling.  Replaced
# with a hip1 joint-frame differential: LEFT side (FL, RL) gets +delta, RIGHT side
# (FR, RR) gets -delta on the hip1 SHOULDER yaw joint, applied AFTER the
# HIP1_SPLAY_OUT_SIGN flip so the effect is pure body yaw.  Schedule is a 10-phase
# ramp over 1200 sim sec: off / +0.5 / off / -0.5 / off / +1.0 / off / -1.0 / off /
# +2.0.  Phase length = 120 sim sec each.  Brain stays attached; probe is purely
# additive on the brain's u_hip1.
var _yaw_probe_enabled: bool = false
# 2026-06-08 — Cruse-trace V2: periodic textual summary of CruseCoordinator
# internal state.  Toggle via OGMA_PICRAWLER_CRUSE_TRACE=1 or via reward_panel.
# Prints once per `_cruse_trace_interval_ticks` ticks (default 60 = 1 sim sec
# at TAU=0.02) with [CRUSE] prefix for easy grep in the dense console.
var _cruse_trace_enabled: bool = false
const _CRUSE_TRACE_INTERVAL_TICKS: int = 60
# Track last fire counters so we can emit deltas (fires/sec) not cumulative totals.
var _cruse_trace_last_r1: int = 0
var _cruse_trace_last_r2: int = 0
var _cruse_trace_last_r3: int = 0
var _cruse_trace_last_tick: int = 0
var _yaw_probe_delta:   float = 0.0     # set by scheduler each tick
# 2026-06-06 — brain snapshot save / load.  Enables a "pre-train once, probe many
# times" workflow per Joseph's design.  Warmup run sets SAVE path; the snapshot
# is taken on the first curriculum auto-advance from stage 0 → 1 (i.e. when the
# canonical 2-stage stand-only curriculum has decided the brain stands).  Probe
# runs set LOAD path; the snapshot is applied at _ready() end (after all
# action_channel / source / event registrations) so the brain starts with
# already-learned standing weights.
var _brain_snapshot_save_path: String = ""
var _brain_snapshot_save_done: bool   = false
var _brain_snapshot_load_path: String = ""
# 2026-06-10 — curriculum-driven snapshot load.  A curriculum stage may carry a
# "brain_snapshot_load" field (absolute or res:// path); when that stage is
# entered (UI loader or headless startup) the brain restores from it once.  Lets
# a single curriculum reproduce the "standing snapshot + scripted gait" setup in
# the UI without env vars.  Guard so it only fires once per run.
var _curric_snapshot_loaded: bool = false
# 2026-06-06 — alternative save trigger: at a wall-time mark instead of curriculum
# advance.  Used to capture a target-aware snapshot from a single-stage to_target
# curriculum (which never auto-advances).  Empty/<=0 = disabled (trigger remains
# stage advance).
var _brain_snapshot_save_at_sec: float = 0.0
const YAW_PROBE_HIP1_GAIN: float = 0.5  # fraction of delta applied to hip1 u (joint frame, post sign flip)
const YAW_PROBE_PHASE_TICKS: int = 6000 # 120 sim sec at TAU=0.02 (50 Hz physics)
# Per-phase delta values.  index = floor(tick / phase_ticks) % len.
# 2026-06-06 — fine-grained schedule after hip1 ±0.5 destabilized the standing
# stance (141→282 falls vs Cruse run).  Smaller deltas to find the floor at
# which sustained-upright yaw signal emerges.
const YAW_PROBE_SCHEDULE: Array[float] = [
	0.0, +0.1, 0.0, -0.1, 0.0, +0.2, 0.0, -0.2, 0.0, +0.3]
# When a brain snapshot is LOADED (probe runs), the body physics is fresh but the
# brain has stale internal state from the warmup body — give it a few sim sec
# to settle into the new pose before the probe schedule starts.  No effect on
# warmup runs (no LOAD → settle = 0).
const YAW_PROBE_LOAD_SETTLE_TICKS: int = 250   # 5 sim sec at TAU=0.02
var _yaw_probe_settle_offset_ticks: int = 0
# 2026-06-05 — Phase H1 (Hector-grounded hierarchical) leg-event state:
# track per-leg foot-planted bool to detect lift/plant edges and fire
# events.leg_lifted_<leg> / events.leg_planted_<leg>. Intensity uses
# chassis_y_norm × velocity_toward_target × upright_factor (per Cruse
# rule 3 force-feedback flavor — quality reward at each transition).
var _prev_foot_planted: Array = [false, false, false, false]
# 2026-06-11 (a) — per-STEP homeokinetic reward (HK step signal).  Fires on each
# touchdown (a step boundary), NOT per tick — walking lives on a multi-tick
# timescale, so a per-tick signal would favour the standing fixed point and
# punish mid-stride.  reward = homeo_step_gain × step_change × regularity, where
# step_change = foot vertical excursion accumulated over the step (real step =
# large; staying planted = ~0; INTRINSIC sensorimotor change, not distance) and
# regularity = exp(-|period - period_ema| / period_ema) (rhythmic, predictable
# step timing → ~1; an irregular lurch → ~0).  The regularity term is what stops
# this Goodharting into the §3n lurch-for-the-pulse failure: only SUSTAINED
# rhythmic stepping scores.  Gated on upright (chassis_y_norm).  0 = off.
@export var homeo_step_gain: float = 0.0
var _homeo_period_ema:   Array = [0.0, 0.0, 0.0, 0.0]   # EMA of step period (ticks) per leg
var _homeo_last_td_tick: Array = [-1, -1, -1, -1]       # tick of last touchdown per leg
var _homeo_foot_min:     Array = [9.0, 9.0, 9.0, 9.0]   # min foot_y this step (excursion tracking)
var _homeo_foot_max:     Array = [-9.0, -9.0, -9.0, -9.0] # max foot_y this step
var _homeo_n_steps:      Array = [0, 0, 0, 0]           # completed steps per leg (EMA bootstrap)
const HOMEO_PERIOD_EMA_ALPHA: float = 0.25              # step-period EMA rate
# 2026-06-11 — upright-quality term.  The bare excursion×regularity reward was
# satisfied by a rhythmic but WOBBLY/knee-folding pseudo-gait (tilt ~0.40); the
# operator (Joseph) saw a claw, not stepping.  Multiply the homeo reward by
# clamp(1 - tilt/TILT_REF) so a tilted step scores ~0 — only UPRIGHT rhythmic
# stepping pays.  TILT_REF=0.35 rad (~20°): tilt 0 → 1.0, 0.17 → 0.5, ≥0.35 → 0.
const HOMEO_TILT_REF: float = 0.35
# 2026-06-11 — self-calibrating payout normalization.  The multiplicative upright
# term suppressed the knee-fold Goodhart (tilt 0.41→0.25, resets halved) but cut
# the homeo payout ~2× (n=3 homeo_upright), dropping it below the ~30%-of-reward
# ignition share — the rhythm decayed back to baseline (autocorr 0.58→0.41).
# Fix: divide the reward by the running MEAN upright factor (EMA over touchdowns,
# floored), so a step that is cleaner than the robot's recent average pays MORE
# than ignition level and a dirtier one pays less.  The clean/tilted gradient is
# preserved; the absolute payout self-calibrates instead of being de-funded.
# homeo_payout_norm: 0 = off (bit-identical to homeo_upright arm), 1 = full.
@export var homeo_payout_norm: float = 0.0
var _homeo_upright_ema: float = 1.0                     # EMA of upright factor at touchdowns
const HOMEO_UPRIGHT_EMA_ALPHA: float = 0.05             # ~20-step settle
const HOMEO_UPRIGHT_EMA_FLOOR: float = 0.1              # caps amplification at 10x
# 2026-06-11 — (c) cross-leg phase-consistency factor: inter-leg coordination
# in REWARD space, not action space.  E2 logit injection regressed (the brain
# defends its objective); the (a) ignition worked by changing what the brain
# WANTS — this extends the same principle to coordination.  On each touchdown
# of leg i, measure where it lands within every other leg's step cycle
# (Δφ ∈ [0,1)) and score consistency against a per-pair circular EMA of that
# relative phase.  ANY stable phase relation pays (trot, walk, pace — the
# brain picks the gait); only incoherent stepping is taxed.  Neutral (1.0)
# until a pair's EMA concentration R clears the floor — no penalty before
# structure exists.  homeo_phase_couple: 0 = factor forced 1.0 (tracking +
# telemetry still live), 1 = full coupling.
@export var homeo_phase_couple: float = 0.0
# Per ordered pair (i,j): circular EMA of leg i's touchdown phase within
# leg j's cycle, stored as (cos, sin) so wraparound averages correctly.
# Concentration R = |EMA| doubles as "structure exists" gate + telemetry.
var _homeo_phase_ema_x: Array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
var _homeo_phase_ema_y: Array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
var _homeo_phase_factor_last: float = 1.0               # telemetry: last phase factor applied
var _homeo_q_ema: float = 1.0                           # EMA of combined quality (upright×phase) at touchdowns
const HOMEO_PHASE_R_FLOOR: float = 0.1                  # below = no structure yet -> neutral
var _homeo_reward_last: float = 0.0                     # telemetry: last homeo reward fired
# Hysteresis band (±15% of stance_y_threshold) avoids edge chatter when
# foot_y wobbles near the threshold.
const LEG_EVENT_HYSTERESIS_FRAC: float = 0.15
# Cumulative per-leg event counters for diag verification (so we can
# confirm events fire at ~1-4 Hz per leg under R1 Cruse without needing
# a bus-side consumer).
var _leg_lifted_count: Array = [0, 0, 0, 0]
var _leg_planted_count: Array = [0, 0, 0, 0]
var _leg_event_intensity_sum: Array = [0.0, 0.0, 0.0, 0.0]
# Phase H1 V2 — closed-loop cognitive steer.  Body caches the cognitive
# PremotorAI's last_chosen (∈ 0..4) at each diag tick and uses it to
# rotate target_compass before publishing.  Brain perceives the "cognitively-
# steered" target; fast Premotor steers to align with the rotated direction;
# cognitive's reward (events.leg_event quality) depends on the body's actual
# motion under its own bias, closing the loop.  Default 0.0 = no bias (V1
# behaviour preserved when cognitive Premotor's pre_W = 0).
const COGNITIVE_BIAS_MAP: Array[float] = [-0.5, -0.25, 0.0, 0.25, 0.5]   # rad
var _cognitive_bias_rad: float = 0.0
var _cognitive_last_chosen: int = 2  # default = "straight" (intent 2)
# 2026-06-09 — L2 open-loop steerability assay (capability_ladder_scoreboard.md).
# When OGMA_PICRAWLER_FORCE_COGNITIVE_BIAS=<rad> is set, pin _cognitive_bias_rad
# to that constant so target_compass is rotated by a fixed open-loop steer,
# bypassing the cognitive Premotor / population entirely.  This is the
# perception-level twin of the motor-level OGMA_PICRAWLER_YAW_PROBE: it tests
# whether the fast substrate follows a steered PERCEPTION (vs whether the body
# can mechanically turn).  Default-off → bit-identical legacy behaviour.
var _force_cog_bias_enabled: bool = false
var _force_cog_bias_rad: float = 0.0
# 2026-06-07 — F10 Cognitive Premotor Population caches.  Empty when only a
# single premotor_cognitive is in the config (legacy F1-F9 behaviour).  Populated
# when the population variant config (cognitive_left/center/right) is loaded —
# body sums their accels into _cognitive_bias_rad in the post-metrics block.
var _cog_pop_accels: Dictionary = {}    # id -> last_accel
var _cog_pop_chosen: Dictionary = {}    # id -> last_chosen
# 2026-06-07 — F11 Cognitive Lateral Voter (body-side multi-arm-bandit trust).
# Per Fractal JEPA addendum A.4.2-A.4.3 (minimum-scope test).  Each population
# member's accel is weighted by its recent "credit": when a member is dominant
# (largest |accel|) AND a reward event fires within the credit-window, that
# member's credit increases.  Other members lose credit by decay.  Body-side
# weighted sum replaces the naive sum used in F10.  Setting cog_voter_enabled
# to false reverts to F10's naive aggregation (backward-compat).
var _cog_voter_enabled:      bool = true
var _cog_pop_credit_ema:     Dictionary = {}    # id -> credit EMA (init 1.0 → uniform)
var _cog_voter_credit_alpha: float = 0.01       # EMA per credit event
var _cog_voter_decay:        float = 0.0001     # per-tick decay toward 1.0 (uniform prior)
var _cog_voter_temperature:  float = 0.3        # softmax temp for weight computation
var _cog_voter_credit_window_ticks: int = 30    # tick window for "dominant when reward arrived"
var _cog_voter_recent_dominant_id:  String = "" # last dominant member
var _cog_voter_last_dominant_tick:  int = -1    # last tick of dominance change
var _cog_voter_hit_pending_until_tick: int = -1 # window-end tick after which dominance gets credit
var _cog_voter_pending_dominant_id:    String = ""
# Cache the per-tick TRUE v_toward_target signal so the population aggregator
# can attribute credit to whichever member is currently dominant.
var _last_true_v_toward_target: float = 0.0
# Phase H1 V6 — proxy looming detector ("LGMD analog").  Body casts an
# 8×8 grid of forward rays from chassis height and counts how many hit
# the active target pyramid.  Published as reality.proprio.target_loom
# (single float ∈ [0,1] = fraction of forward FOV occluded by target).
# When body approaches target, more rays hit → loom value rises (the
# same integrand that locust LGMD computes from retina, just downsampled
# to 64 channels).  Slow cognitive stream's W matrix learns the
# distance-dependent steering from this signal.
const LOOM_RES_H: int = 24                    # bumped: target subtends ~10° wide
const LOOM_RES_V: int = 12                    # bumped: target subtends ~6° tall
const LOOM_FOV_H_RAD: float = 3.1415926536   # 180° horizontal — fly-style panoramic
const LOOM_FOV_V_RAD: float = 0.7853981634   # 45° vertical
# ⚠ 2026-08-04 — WAS 8.0, justified as "pyramid arena ~5 m diag", which is wrong: the arena
# is radius 9.5 and _select_across_target_idx() deliberately picks the ANTIPODAL hemisphere,
# so 93 % of target legs BEGAN BEYOND THE RAY LENGTH — both loom and the camera were
# identically zero for most of every leg, and any "vision nav" result would have measured the
# last metre only.  Now past the ~19 m arena diagonal.  Cost is per-ray distance, not ray
# count, so this is nearly free.
const LOOM_RAY_LEN: float = 20.0
const GROUND_CLEARANCE_RANGE: float = 0.3    # m — downward belly ToF max range (the physical sensor reach)
const GROUND_CLEARANCE_STAND: float = 0.06   # m — standing belly clearance; the published signal is
                                             # NORMALIZED by this (not by the ToF range) so it SATURATES at
                                             # ~standing, exactly like chassis_y_norm.  Without this the
                                             # height reflex's max-relative target runs away (belly pushed to
                                             # 0.19 m → topple, 2930 falls).  A self-model calibration
                                             # constant (the robot's own standing height) — still compliant.
const LOOM_RECOMPUTE_EVERY: int = 6           # ticks; ~10 Hz at 60 Hz physics
											  # 288 rays × 10 Hz = ~2880 rays/sec, cheap
var _last_target_loom: float = 0.0
var _loom_recompute_counter: int = 0
# Phase 7.x — walk_over_there stage support.  Per-pyramid mesh ref so we
# can recolor a chosen pyramid purple for target indication; per-pyramid
# default material so we can restore color when the target moves on.
var _pyramid_meshes:       Array = []   # MeshInstance3D per pyramid
var _pyramid_default_mats: Array = []   # StandardMaterial3D per pyramid (default brown)
const _TARGET_PYRAMID_COLOR: Color = Color(0.55, 0.22, 0.65, 1.0)  # purple
var _target_pyramid_mat: StandardMaterial3D = null   # shared purple material (lazy-init)
# XZ offset applied to ALL rest transforms during the next _do_hard_reset.
# Zero = spawn at the original origin (legacy behaviour).
var _pending_reset_offset: Vector3 = Vector3.ZERO
# Radius of the outermost concentric floor ring (see _build_floor_rings).
# Auto-reset-in-place fires only when the chassis is strictly OUTSIDE this
# radius — inside it, the legacy center reset still applies.
const RING_OUTERMOST_RADIUS: float = 3.0
# Chassis-bounding-circle radius for pyramid-avoidance during in-place
# reset.  Plus a small safety margin so legs don't immediately collide.
const RESET_CHASSIS_MARGIN:  float = 0.25

# Brain action channel indices (3 per leg = 12 total).
var _idx_hip1: Array[int] = []
var _idx_hip2: Array[int] = []
var _idx_knee: Array[int] = []

# Episode state
var _accum: float = 0.0
var _done: bool = false
var step_in_episode: int = 0
var episode_alive_ticks: int = 0
var tick_counter: int = 0
# Controlled belly-on-ramp test: tick at which to auto-drop the robot onto the
# hump (env OGMA_PICRAWLER_TELEPORT_RAMP_AT; -1 = off).  Lets a headless run
# develop the gait first (e.g. 3600 = 1 min) THEN drop, to test recovery.
var _teleport_ramp_at: int = -1
# Scripted placement schedule from OGMA_PICRAWLER_EVENTS (see the dispatch site).
var _events: Array = []
# Optional repeated re-teleport window (env EVERY/UNTIL) — re-fire every N ticks up
# to UNTIL, to SUSTAIN an anomaly (e.g. keep re-flipping so it can't self-right).
var _teleport_every: int = 0
var _teleport_until: int = -1
# ---- 2026-08-04 · PER-LEG LESION — the locomotor-(d) test the plan has always named
# ("degrade a leg -> re-coordinates", picrawler_active_inference_plan.md 2.8 and 10).
# Every perturbation this project could run until now was a TELEPORT, which relocates the
# body without changing what the body IS; the world explains the disturbance.  A lesion
# leaves the world untouched, so nothing external can account for a behavioural change.
# It scales ONE leg's motor torque ceiling from a given tick.  Deliberately honest: the
# leg still senses itself perfectly, it simply cannot push, so the brain can only notice
# through its OWN rising prediction error — which is the (a) "inferred, not oracle" bar.
# ⚠ It must NOT announce a reset (the teleport does, for Gate-0 masking): a reset here
# would mask the very re-organisation this exists to measure.
# _lesion_leg = -1 (the default) => every scale is 1.0 => byte-identical to no lesion.
var _lesion_leg: int = -1
# 2026-08-06 — PER-FOOT SLICK ABLATION.  The sufficient half of the leg-attribution
# validation: a lesion zeroes a leg's COMMANDS, so the leg stops moving and both its
# attributed share AND its raw sweep collapse together — a metric that merely counted
# MOVEMENT would score identically.  Making one foot frictionless instead leaves the leg
# sweeping normally while removing its ability to transmit thrust, so a propulsion metric
# must lose that leg's share while its raw sweep is untouched.  -1 = off.
# 2026-08-06 — GROUND REACTION IMPULSE per foot, accumulated every PHYSICS step.
#
# The kinematic attribution metric (stance-gated hip1 sweep) FAILED its sufficient
# validation: with FL's foot slicked to mu 0.05 — physically unable to transmit thrust —
# its attributed share did not move (0.230 -> 0.230).  A leg on a frictionless foot still
# sweeps in stance, so stance kinematics cannot distinguish GRIP from SLIP, and the
# correlation with fwd_v may be the body dragging the leg rather than the leg driving the
# body.  Force is the only thing that settles the direction of causation.
#
# The feet already run contact_monitor with 4 contacts reported, so the solver's actual
# contact impulses are available via body_get_direct_state.  Projected on the body-forward
# axis this IS the propulsion each leg delivers, and it passes the slick test by
# construction: a frictionless foot transmits no tangential force.
var _grf_fwd: Array[float] = [0.0, 0.0, 0.0, 0.0]     # accumulated since last trace record
# 2026-08-07 — PER-LEG VERTICAL LOAD.  The missing observation.
#
# To push the CoG past the support polygon onto the foot that is about to plant, the
# body must know HOW ITS WEIGHT IS DISTRIBUTED, and nothing on the bus carries that:
# foot_contact is a binary touch with no magnitude; joint_torque is a PD command budget
# dominated by its -Kd*omega term; joint_load (the velocity-tracking deficit) was
# measured NOT to discriminate stance from swing (0.383 vs 0.257).  Doctrine §1 rule 2:
# when no egocentric observation carries the signal the error needs, the fix is a NEW
# SENSOR, not a smarter policy.
#
# Four load scalars ARE a centre-of-gravity sensor: if fl carries 40% of the weight the
# CoG is over fl.  ⚠ AND UNLIKE fwd_v THIS IS EGOCENTRIC-LEGAL — a real picrawler reads
# it from a foot FSR or servo current sense — so it is a lawful brain input, which the
# god's-eye attribution instrument never was.
#
# EMA rather than a drained accumulator so the published sensor is independent of the
# trace recorder, and because a real load cell has its own time constant anyway.
var _grf_up: Array[float] = [0.0, 0.0, 0.0, 0.0]      # drained by the trace (world-up, for comparison)
var _grf_nrm: Array[float] = [0.0, 0.0, 0.0, 0.0]     # drained by the trace (FSR analogue)
var _foot_load_ema: Array[float] = [0.0, 0.0, 0.0, 0.0]
const _FOOT_LOAD_ALPHA: float = 0.15
# Static weight as a PER-PHYSICS-STEP impulse (N·s), which is the unit
# get_contact_impulse returns.  chassis + 4 legs x (coxa+upper+lower).
# ⚠ Was `const ... = CHASSIS_MASS + ...` — a const folded from consts, which
# would have silently kept the CAD total once the masses became loadable vars.
# Recomputed by _load_geometry(); the literal here is only a pre-load fallback.
var _TOTAL_MASS: float = 0.600
var _slick_leg: int = -1
var _slick_at: int = 0
var _slick_done: bool = false
var _lesion_at: int = -1
var _sched_patches: Array = []   # SETPARAM_AT entries: {t, id, key, val, done}
var _lesion_until: int = 0x7FFFFFFF
var _lesion_scale: float = 0.2
# "action" (default) attenuates the leg's COMMANDED MOTION; "torque" caps its torque
# ceiling.  Kept default-off rather than removed: the torque form is refuted as a
# PERTURBATION on this body but is real infrastructure, and its null is itself a finding.
var _lesion_mode: String = "action"
var _lesion_active: bool = false

# ---- 2026-08-04 · PHYSICAL ABLATION MODEL (the [K] panel) ------------------------------
# The single-leg lesion above answers one question ("degrade a leg"). This is the general
# instrument: ANY of the 12 servos can fail in any of three realistic ways, and ANY joint
# can be broken clean through so the limb below it comes off. That is what a real robot
# does when it breaks, and none of it is signalled to the brain — the only way it can
# notice is its own rising prediction error, which is the (a) "inferred, not oracle" bar.
#
# The three servo failures are genuinely different mechanically and the brain should not
# find them equivalent:
#   DEAD   — unpowered. Motor released (velocity 0, impulse 0): the joint free-swings and
#            is back-driveable, so the limb flops and gravity moves it.
#   SEIZED — jammed gearbox. Velocity target 0 at FULL impulse: the joint fights any
#            motion and holds wherever it happened to be. A rigid strut, not a limp one.
#   WEAK   — a tired/browning-out servo. The commanded motion is attenuated but live.
# DETACH is separate from all three: it breaks the constraint so everything DISTAL to that
# joint separates. Detach hip1 -> the whole leg comes off; hip2 -> upper+lower; knee ->
# the shank and foot. Reversible, so an A/B can be run without restarting.
enum AblKind { OK = 0, DEAD = 1, SEIZED = 2, WEAK = 3 }
const ABL_KIND_NAMES: Array = ["ok", "dead", "seized", "weak"]
const ABL_JOINT_NAMES: Array = ["hip1", "hip2", "knee"]
const ABL_LEG_NAMES: Array = ["FL", "FR", "RL", "RR"]
var _abl_kind: PackedInt32Array = PackedInt32Array()        # 12, index = leg*3 + joint
var _abl_detached: PackedByteArray = PackedByteArray()      # 12
var _abl_weak_scale: float = 0.3
var _abl_any: bool = false                                  # fast path: skip all of it
# Reported angle at the moment of detachment.  A detached segment is frozen in WORLD space
# while the chassis walks away, so the measured relative angle would diverge into garbage
# and flood the brain with a signal no real robot produces.  A real robot's servo keeps
# reporting its own encoder, so the honest reading is the last one before the break.
var _abl_hold_angle: PackedFloat32Array = PackedFloat32Array()
# Chassis-relative transforms saved at detach, so re-attaching puts the limb back in the
# pose it came off in rather than wherever it fell.
var _abl_saved_xform: Array = []
var _abl_saved_nodes: Array = []                            # [node_a, node_b] per joint
var _abl_env_spec: String = ""
var _abl_env_at: int = 0
var _abl_env_done: bool = false
# Deferred teleport target (Vector3 ground point) applied at the TOP of the next
# physics frame — transform writes from _input handlers get clobbered by the
# physics step in the same frame, so KEY_3/KEY_4 must defer.  null = nothing pending.
var _pending_teleport = null
# Per-drop flip intent for the NEXT pending teleport: -1 = fall back to the
# OGMA_PICRAWLER_TELEPORT_FLIP env (scripted runs), 0 = upright, 1 = inverted.  Lets the
# mouse placement pick orientation per click instead of needing an env restart.
var _pending_teleport_flip: int = -1
# Ramp-debug: last raw belly rangefinder reading (metres), cached at publish so the
# diag can log the rangefinder output + homeostat state on the hump.
var _dbg_gc_raw: float = 0.0
var _dbg_contact_swing: float = 0.0   # TRUE swing fraction from the foot-contact sensor
var _dbg_fk_cmd_err: float = 0.0      # mean |commanded-FK − achieved-FK| foot height (m)
var _dbg_fk_valid_err: float = 0.0    # mean |measured-FK − achieved-pose| = FK wiring check
var _dbg_att_err_acc: float = 0.0     # accel-only attitude error vs exact (deg)
var _dbg_att_err_imu: float = 0.0     # gyro-fused attitude error vs exact (deg)
var _dbg_acc_mag: float = 0.0         # |accelerometer| m/s^2 (should hover near 9.81)
var _dbg_acc_trust: float = 0.0       # adaptive correction gain actually applied
var _prev_lin_vel: Vector3 = Vector3.ZERO   # for finite-differencing body acceleration
var _up_est_body: Vector3 = Vector3.ZERO    # complementary-filter gravity-up estimate (body frame)
var _up_acc_last: Vector3 = Vector3.ZERO    # last accel-only gravity-up (body frame)
var _accel_body_last: Vector3 = Vector3.ZERO  # last modelled accelerometer reading
var _gyro_body_last: Vector3 = Vector3.ZERO   # last modelled gyro reading
# 2026-08-04 — DEAD-RECKONED EGO HEADING for the L1 nav loop.  Integrated from the MODELLED
# BODY-FRAME GYRO (_gyro_body_last.y), which is what a real IMU reports — not from the chassis
# world transform, which would be a god's-eye read.  It drifts, exactly as dead reckoning does
# on hardware; that drift is honest and RunTumbleNavV2 only needs the frame to be SELF-
# CONSISTENT (it accumulates run_dir_abs_ in the same frame and outputs an egocentric delta).
var _ego_heading: float = 0.0
var _accel_lp: Vector3 = Vector3.ZERO         # DLPF state
# How much the complementary filter trusts the accelerometer per tick.  Small = trust the
# gyro short-term (rejects bounce contamination) and let the accel correct drift slowly.
const IMU_ACC_TRUST: float = 0.02
# Models the accelerometer's on-chip anti-alias low-pass (datasheet DLPF), which a raw
# finite difference lacks.  ~0.25 at 240 Hz ≈ a few tens of Hz cutoff.
const IMU_DLPF_ALPHA: float = 0.25
# Accelerometer full-scale range (±4 g), as a real part would clip at.
const IMU_RANGE_MS2: float = 4.0 * 9.81
# Accelerometer trust falls to zero once ‖a‖ deviates from g by this FRACTION of g.
# Quasi-static samples correct fully; footfall impacts contribute nothing.
const IMU_ACC_GATE_FRAC: float = 0.5
var episode_index: int = 0
var _episode_fell: bool = false
var _instant_pause_tick: bool = false

# Cumulative standing counter — increments per tick the body is upright
# (not fell), resets ONLY on a fresh catastrophic fall event.  Survives
# mc_episode_period resets so the HUD can show real standing duration
# (the per-episode `episode_alive_ticks` is reset every 1500 ticks by
# the MC trajectory boundary, which makes its counter cap at 30 s
# regardless of actual standing time).
var cumulative_alive_ticks: int = 0
var best_cumulative_alive_ticks: int = 0    # max across the session
# Walking PB: maximum distance from origin (m) reached by the chassis
# during this session.  Monotone non-decreasing — never resets on fall,
# auto-reset, or manual reset.  Mirrors the spirit of
# best_cumulative_alive_ticks for the walking objective.
var max_distance_from_origin: float = 0.0

# Phase 6.9 — speed tracking + PR (personal-record) pulse bonus.
# current_speed   : magnitude of horizontal chassis velocity each tick
# best_speed      : monotone non-decreasing max ever observed (persists
#                   across falls / MC resets / auto-resets, same as
#                   max_distance_from_origin).  When walk_hit_rate>0 and
#                   current_speed > best_speed (and ≥ SPEED_PR_MIN to
#                   avoid celebrating per-tick jitter), fire a burst
#                   events.hit with intensity=min(2.0, 1.0+5.0×delta) —
#                   gives the brain a "I just got faster!" credit pulse
#                   on top of the continuous walk-velocity reward.
# current_sustained_speed_ticks : streak of consecutive ticks with
#                   speed ≥ SPEED_SUSTAINED_THRESHOLD.  Resets when speed
#                   drops below threshold.
# best_sustained_speed_ticks    : monotone max of that streak.  Used by
#                   curriculum stage 2 (learn_to_walk) to advance when
#                   the body has held 0.5 m/s for ≥ 60 ticks (1 s).
# pr_event_count : how many PR bursts have fired (telemetry).
var current_speed: float = 0.0
var best_speed:    float = 0.0
var current_sustained_speed_ticks: int = 0
var best_sustained_speed_ticks:    int = 0
var pr_event_count: int = 0
const SPEED_PR_MIN: float                  = 0.05  # don't celebrate jitter below this
const SPEED_SUSTAINED_THRESHOLD: float     = 0.5   # m/s threshold for sustained-walk counter
# Phase 6.15 — EMA-smoothed speed.  User observation: a brief 1-tick
# velocity lurch (e.g. during a fall/recover transient) could spike
# |v_xz| to 0.9 m/s for a single tick — that's not "walking", it's a
# transient.  Smoothing |v_xz| with α≈0.05 (~1 sec @ 60 Hz) means
# reward only flows for sustained gait-like motion, and PR records
# only lock above what the body can actually reproduce over ~1 sec.
# Affects:
#   - walk_reward_mode=total_speed:  walk_v uses _speed_ema (not raw)
#   - PR detection:                  best_speed climbs on _speed_ema
#   - sustained-speed counter:       gates on _speed_ema ≥ threshold
# Radial / radial_penalize_inward modes are unchanged for now.
var _speed_ema: float = 0.0
const SPEED_EMA_ALPHA: float = 0.05

# Turbo budget — when OGMA_QUIT_AFTER_TICKS is set the body exits cleanly
# after that many physics ticks.  Pattern lifted from body_controller.gd /
# cart_body.gd; needed for the picrawler experiment harness so wall-clock
# isn't wasted in --turbo (--fixed-fps) mode.  0 = no budget.
var _quit_after_ticks: int = 0

# Fractional-duty accumulator for the rate-coded standing reward.
# Each tick we add `chassis_y_norm * HIT_RATE_AT_STANDING` and emit one
# events.hit per whole unit accumulated.  At full standing the rate is
# HIT_RATE_AT_STANDING = 0.2 hits/tick — below the 0.21-hits/tick
# dopamine-saturation invariant in docs/v4_brain_derivation.md §4.2.
const HIT_RATE_AT_STANDING: float = 0.2
var _hit_accumulator: float = 0.0

# Phase 7.5.R+ — per-source reward attribution.
# Tracks how much each reward channel is CONTRIBUTING per tick so the
# operator can see (in HUD + JSONL + snapshot) which path is driving
# the dopamine pulses they observe in the neuro module.  Three sources
# composing into events.hit:
#   standing  = chassis_y_norm × HIT_RATE_AT_STANDING × standing_baseline_factor
#   walking   = v_norm × walk_hit_rate          (legacy radial / penalize / target)
#   gated     = chassis_y_norm × gate_v × gated_walk_bonus_rate  (Phase 7.5.R)
# Per-tick added rate is what the accumulator INTEGRATES; threshold-
# crossing events.hit firings are sporadic at low per-tick rates, so we
# surface the smooth EMA-of-added rate as the diagnostic signal.
var _hit_delta_standing:    float = 0.0
var _hit_delta_walking:     float = 0.0
var _hit_delta_gated:       float = 0.0
var _hit_rate_standing_ema: float = 0.0
var _hit_rate_walking_ema:  float = 0.0
var _hit_rate_gated_ema:    float = 0.0
const HIT_RATE_EMA_ALPHA:   float = 0.05   # ~20-tick window for smoothing
# Cumulative reward-contribution sums (NOT events.hit counts — these
# are the integrated per-source contribution that produced the events).
# Useful in the end-of-run snapshot to see total reward share by source.
var _hit_cum_standing: float = 0.0
var _hit_cum_walking:  float = 0.0
var _hit_cum_gated:    float = 0.0
# Phase 7.10b — inter-diagonal phase-contrast EMA for telemetry.
var _phase_contrast_ema: float = 0.0
# Phase 7.12 — progress-PB reward state.
var _min_dist_to_target_pb: float = 1e9   # reset on target rotation
var _hit_delta_progress:    float = 0.0
var _hit_rate_progress_ema: float = 0.0
var _hit_cum_progress:      float = 0.0
# 2026-06-02 Stage 2 walking paradigm — per-leg step-quality reward.
# v3 reward shape: per-leg correlation of foot_y motion with CPG phase.
# Rewards each leg for being lifted during its CPG swing window AND
# planted during its stance window — couples directly to foundational
# CPG substrate ([[v6-cpg-is-foundation]]).  v1 (boolean lift) and v2
# (lift_ema × plant_ema) both empirically falsified at gain=0.04:
# v2 paired A/B (n=2, 20min) showed −27% longest_upright vs gain=0
# control with no policy-level signal on chassis_y/da/pre_w.
# See docs/plans-and-designs/walking_paradigm_redesign.md.
var _step_phase_match_ema: Array = [0.0, 0.0, 0.0, 0.0]
# Cached CPG state.  Re-synced from brain at every diag tick (line ~4988
# in _emit_jsonl) and advanced locally each physics tick so the per-tick
# reward block has a high-resolution phase value without per-tick metric
# dict cost.  Drift bounded by 2π/3 between syncs (1/3 cycle).
var _cpg_phase_now:        float = 0.0
var _cpg_leg_offsets_now:  Array = [0.0, PI, PI, 0.0]  # default = config's trot offsets
var _cpg_period_ticks:     int   = 60
var _cpg_present:          bool  = false
var _hit_delta_step_quality:    float = 0.0
var _hit_rate_step_quality_ema: float = 0.0
var _hit_cum_step_quality:      float = 0.0

# Target-height reward + over-target penalty (v6.0.b.11).  User observation
# during Stage B-F: the unbounded height reward (chassis_y_norm clamped at
# STANDING_CHASSIS_Y) lets the brain climb past standing without a signal
# to come back down — producing the "test the limit → eventually tip" mode.
# Capping reward at target_height + adding a graded events.miss above
# (target_height + grace) creates a satisfaction basin AND an explicit
# reason not to climb further.  Trapezoid shape: ramp on [0, target],
# flat plateau above target, penalty starts at target+grace.  When
# height_penalty_gain=0 (default), the penalty arm is off and the
# reward simply caps at target_height — preserves byte-identical
# behaviour vs. pre-v6.0.b.11 when target_height = STANDING_CHASSIS_Y.
# ⚠ Literal, not STANDING_CHASSIS_Y: an @export default is evaluated at class
# init, before _load_geometry() runs, so it cannot reference a loadable var.
# _load_geometry() re-points it at the loaded standing height IF it is still
# sitting at this literal — so a swapped body follows, while env
# (OGMA_PICRAWLER_TARGET_HEIGHT) and curriculum overrides still win.
@export var target_height:         float = 0.082   # = CAD STANDING_CHASSIS_Y — back-compat
@export var height_penalty_grace:  float = 0.02                  # m above target before penalty engages
@export var height_penalty_scale:  float = 0.03                  # m above (target+grace) for full intensity
@export var height_penalty_gain:   float = 0.0                   # events.miss intensity at full; 0 = mechanism off
# Phase 6.16 — multiplicative tilt factor on the standing-hit reward.
# When > 0, multiplies chassis_y_norm by tilt_norm = clamp(1 -
# chassis_tilt/tilt_target_rad, 0, 1).  Body must be BOTH at target
# height AND upright (within tilt_target_rad of perfectly vertical)
# to earn full standing reward.  Closes the gap user observed: with
# only chassis_y in the reward, a bowed pose (tilt 30° but chassis at
# target) pays the same as a spider stance (tilt 0° at target).
# tilt_target_rad = 0 disables the mechanism (default = backward compat).
# Recommended starting value: 0.5 rad (~29°) — body within 29° gets
# partial reward, perfectly upright gets full.  Curricula opt in via
# the override list.
@export var tilt_target_rad: float = 0.0

# Reward-curve shape selector (v6.0.b.12).
#   "trapezoid"   (default) — chassis_y_norm = clamp(chassis_y / target_height, 0, 1).
#                  Ramp on [0, target], flat plateau above.  Combine with
#                  height_penalty_gain > 0 to add an over-target events.miss.
#   "inverted_u" — chassis_y_norm = max(0, 1 - |chassis_y - peak_height| / band_width).
#                  Peak at peak_height; drops off in BOTH directions.  Brain
#                  gets a positive gradient toward peak from above OR below.
#                  Tests user hypothesis (Stage C-2): a target with bilateral
#                  pull may suppress the "keep climbing" failure mode the
#                  trapezoid + per-tick penalty couldn't eliminate.
@export var reward_shape: String = "trapezoid"
@export var peak_height:  float  = 0.082   # = CAD STANDING_CHASSIS_Y; used by inverted_u.
										   # Same init-order caveat as target_height above.
@export var band_width:   float  = 0.04                  # m; reward → 0 at peak ± band_width
# V9 spider-stance floor — anti-belly-flop safety bound.  When chassis_y falls
# below this, chassis_y_norm (the reward-side height factor used by standing /
# walking / gated streams) is zeroed.  Keeps the inverted_u low-stance peak from
# being satisfied by belly-flop (chassis_y ≈ 0 → otherwise still inside the
# band).  Default 0.0 = disabled (legacy).  Spider-stance recommended ~0.05.
@export var reward_min_height: float = 0.0
# V9c — peak_height source.  "fixed" (default): use peak_height @export as a
# static reward target.  "knee_relative": dynamic — compute the average knee
# height each tick from the lower-leg origins, use that as the effective peak.
# Captures the biomechanical truth that a spider's stable chassis height is
# always relative to where its knees currently are, not a static absolute.
# Body never reaches above its own knees in any natural pose, so the upper
# bound is intrinsically respected without a hard band cap.
@export var peak_height_mode: String = "fixed"
# V9 — reward-side height reference.  "chassis" (default, legacy): use
# chassis_xform.origin.y (chassis center, rotation-invariant for chassis CoM
# rotation but still wobbles when legs translate the chassis vertically).
# "body_cog": mass-weighted average y of chassis + all 12 leg segments —
# physically the most stable height reference, since internal forces between
# chassis and legs cancel in the body CoG (Newton's 3rd) and only ground
# reaction + gravity change it.  Chassis is 50% of body mass (0.300 of 0.600 kg);
# legs matter a lot for CoG.  Affects the REWARD chassis_y_norm only —
# proprio signal, auto-reset, level-chassis, etc. continue using chassis_y so
# downstream semantics don't shift.  When set to body_cog, the curriculum
# peak_height/target_height/reward_min_height should be tuned for the
# body_cog scale (~0.015 m lower than the equivalent chassis_y target).
@export var height_reference: String = "chassis"

# Walking reward (v6.0.b.13) — events.hit rate proportional to chassis
# velocity magnitude.  Composed with the height reward by accumulating
# into the same `_hit_accumulator`: total per-tick hit rate is
# height_norm × HIT_RATE_AT_STANDING + walk_norm × walk_hit_rate.
# Brain optimises both via the same DA pathway; REINFORCE sorts the
# credit assignment.
#
#   walk_target_velocity  m/s; speed at which the walk reward saturates
#                         (default 0.05 ≈ half body-length per second,
#                         matches the SunFounder PiCrawler moderate walk
#                         speed in spec).  Reward = velocity / target,
#                         clamped to [0, 1].
#   walk_hit_rate         peak hits/tick from the walk component.
#                         Default 0 = mechanism off (preserves byte-
#                         identical behaviour vs. pre-v6.0.b.13).
#                         0.05 ≈ ¼ of the height reward's peak — keeps
#                         standing as the primary signal.
@export var walk_target_velocity: float = 0.05
@export var walk_hit_rate:        float = 0.0
# Phase 7.5.R — multiplicative-gating reward shape (Option 2).
#
# Diagnosed cause of standing-attractor lock-in: standing reward fires
# at HIT_RATE_AT_STANDING (0.20 hits/tick) every tick the body is
# upright; walking reward fires at walk_hit_rate (typically 0.05) only
# when sustained radial-outward velocity is present.  Total reward
# accumulates ~4× higher for standing than walking — brain gradient-
# descends toward the standing pose and cancels any phase-driven crawl
# attempts because they reduce time-weighted reward.
#
# Multiplicative gating breaks the asymmetry:
#   1. `standing_baseline_factor` scales DOWN the pure-standing rate
#      so brain still bootstraps pose acquisition cold-start but the
#      mature standing-only reward is small.  Default 1.0 = legacy
#      behaviour.  Recommended 0.25 (= 0.05 hits/tick standing only).
#   2. `gated_walk_bonus_rate` adds an EXTRA hit stream proportional
#      to chassis_y_norm × v_norm.  Fires densely (every tick) when
#      brain is BOTH upright AND moving radially outward; fails closed
#      if either condition isn't met.  Default 0.0 = mechanism off.
#      Recommended 0.20 (= matches legacy standing density when full
#      walking + full upright achieved).
#
# Net effect at the three regimes:
#   regime              standing_baseline (×0.25)  gated_bonus  total
#   ----------------------------------------------------------------
#   standing only       0.05 hits/tick             0.00         0.05
#   walking only (fall) 0.00                       0.00         0.00
#   both                0.05                       0.20         0.25
#
# Brain experiences a 5× reward gradient between standing-only and
# upright-AND-walking — the multiplicative coupling makes walking
# essential to compound the reward, NOT just additive.
#
# This implements Option 2 from the Phase 7.5 reward shape discussion
# without adding magic numbers — both parameters compose with the
# existing standing + walking machinery, defaults preserve byte-
# identical legacy behaviour, curriculum stages opt in.
@export var standing_baseline_factor: float = 1.0
# Fix 2 — standing reward EMA window.  Smooths chassis_y_norm before the standing
# reward fires, so a brief leg-lift (swing pose: chassis dips for ~0.3-0.5s) doesn't
# instantaneously crash the dominant reward stream against the swing-curl pose the
# insect stride requires.  Default 1.0 = no smoothing = legacy per-tick behavior.
# Recommended 0.05 (~20-tick window ≈ 0.4s) so transient lifts are absorbed but
# sustained collapses still register.
@export var standing_reward_ema_alpha: float = 1.0
var _chassis_y_norm_ema: float = 0.0
# Fix 1 — gated_walk_bonus velocity mode.  "radial" (legacy): gate on outward
# radial velocity; "body_forward": gate on chassis forward velocity in body frame
# (Vector2(sin(yaw),cos(yaw)) · v_xz).  body_forward forces the body to *face where
# it's going* to earn the gated bonus → paddling without hip1 reciprocation can't
# sustain body-forward velocity → distinguishes insect stride from paddle.
@export var gated_walk_velocity_mode: String = "radial"
@export var gated_walk_bonus_rate:    float = 0.0
# Phase 7.7 — per-leg reward decomposition.
#
# Diagnosed bottleneck from 1-hour seed=44 run + 6 Stage-2 sweeps: brain
# has gait (peak |v|=0.88 m/s) but cannot direct it (1.6% navigation
# efficiency).  Substrate has proper score-function REINFORCE +
# advantage_normalization (enabled since v6 frozen baseline), so credit-
# assignment mechanism is fine.  The remaining hypothesis is that GLOBAL
# reward (one DA pulse to all 12 Premotors) gives every leg the same
# credit for outward motion, regardless of which leg actually
# contributed.  REINFORCE then can't differentiate "this leg helped"
# from "this leg flailed" at the per-Premotor level.
#
# Per-leg decomposition: each tick, compute per-leg contribution to
# outward chassis motion = max(0, chassis_v_radial) × stance_factor[i].
# Fire events.hit_leg_<leg> with that intensity.  Premotors of leg i
# set aligned_event_name="hit_leg_<leg>" to ADD this credit to their
# reward bucket (existing aligned_event mechanism, no Premotor code
# changes).  Now each leg's 3 Premotors get differentiated credit:
# legs in stance during outward motion get strong reward, legs
# swinging or in stance during inward/sideways motion get nothing.
#
# Stance factor = 1 when foot is at low Y (planted, supporting
# chassis), 0 when foot is lifted.  Smooth ramp via stance_y_threshold.
#
# Composes additively with global reward — per-leg credit is ON TOP of
# standing + walking + gated bonus.  Default per_leg_credit_gain = 0
# preserves byte-identical behaviour.  Recommended starting value 0.5
# gives ~0.30 hits/tick total per-leg credit at full conditions
# (matches legacy standing density).
@export var per_leg_credit_gain:      float = 0.0
@export var stance_y_threshold:       float = 0.04   # m above ground
# 2026-06-02 Stage 2 walking paradigm — step-quality reward.  Per leg,
# reward = clamp(4 * lift_ema * plant_ema, 0, 1) × upright_factor.
# When 0.0 the channel is OFF and behaviour is byte-identical to prior
# substrate.  At 0.04 the max combined rate is ~0.04 hits/tick = 20%
# of HIT_RATE_AT_STANDING (per docs/plans-and-designs/walking_paradigm_redesign.md).
@export var step_quality_reward_gain: float = 0.0
# Foot-deviation amplitude that gives full phase-match contribution.
# phase_match = clamp(2 × foot_dev/step_lift_scale × cpg_drive, 0, 1)
# where foot_dev = foot_y - stance_y_threshold/2 and cpg_drive =
# sin(cpg_phase + leg_offset).  A foot oscillating between 0 and
# stance_y_threshold (default 0.04) — midline-centred amplitude
# step_lift_scale/2 = 0.02 — in sync with the CPG hits 0.5 average
# match.  Larger oscillation amplitudes saturate at 1.0.
@export var step_lift_scale:          float = 0.04
# EMA window for per-leg phase match.  alpha=0.02 → ~50-tick window
# (≈0.83 s at 60 Hz physics).  Tight enough to track a single CPG
# cycle (60 ticks); long enough to smooth single-tick foot wobble.
@export var step_quality_ema_alpha:   float = 0.02
# Phase 7.10b — inter-diagonal phase-contrast multiplier on gated bonus.
#
# Diagnosed from UI observation of D-config at 25 min: brain learns
# within-diagonal coordination (FL+RR sync, FR+RL sync — trot
# symmetry) but inter-diagonal phase is wrong.  Both diagonals lift
# together (bound/pronk pattern) → body bounces but doesn't translate.
# Current gated bonus is invariant to inter-diagonal phase: brain
# gets same reward whether bound or trot.
#
# Phase-contrast factor multiplies gated bonus by:
#   factor = (1 - phase_contrast_gain) + phase_contrast_gain × inter_diag_contrast
# where inter_diag_contrast = |stance(FL+RR)/2 - stance(FR+RL)/2| ∈ [0, 1]
#
# At gain=0.7 (recommended):
#   contrast=0 (bound):       factor = 0.30 → 30% of baseline
#   contrast=1 (perfect trot): factor = 1.00 → full reward
#   3.3× reward differential → gradient pulls toward trot
#
# Brain still gets reward at any contrast (cold-start preserved) but
# proper trot pays 3.3× more.  No specific phase imposed — any
# anti-phase mapping between diagonals gets the bonus.  Default 0
# preserves byte-identical legacy.
@export var phase_contrast_gain:      float = 0.0
# Phase 7.12 — progress-PB reward (target-relative, can't be Goodharted).
#
# After 11 bias-mechanism nulls confirmed the substrate ceiling is
# distribution-shape (not learning-from-scratch), we move to reward-
# landscape replacement.  Standard velocity-based rewards allow orbital
# drift (body oscillates near target without ever getting closer).
#
# Progress-PB reward fires events.hit ONLY when the body's distance to
# its current walk_target reaches a new minimum (a personal best).
# Brain CANNOT earn this reward by orbiting — it must actually close
# distance beyond what it's achieved before.  PB resets when target
# rotates (arrival) or stage changes.
#
# Requires walk_target_idx >= 0 (i.e., walk_over_there mode with
# target_mode="random_pyramid" or similar).  When no target, no
# progress reward fires.  Composes additively with walking_to_target
# velocity reward and multiplicative gating — they provide per-tick
# gradient toward target, progress reward provides "permanent reward
# for new closer records."
#
# Default progress_reward_gain=0 preserves byte-identical legacy.
@export var progress_reward_gain:     float = 0.0
@export var progress_reward_min_delta: float = 0.005   # min distance reduction (m) to fire
# Phase 6.8 — walking-reward direction mode.
#   "radial"      = current behaviour: only motion that INCREASES distance
#                   from origin gets credit (v · r̂).  Caps the body against
#                   the outer wedge ring once it reaches the boundary —
#                   radial reward dries up while body presses against the
#                   wall, no incentive to redirect, ends up stuck.
#   "total_speed" = |v_xz| magnitude.  Any horizontal motion direction
#                   counts, including circling/weaving/return-toward-centre.
#                   Drives continuous variation; designed to be the stage
#                   5 walk_fast reward shape after stage 4 saturates.
@export var walk_reward_mode: String = "radial"
# 2026-06-07 — body-level target_mode @export so the reward_panel UI can
# manually activate a random_pyramid target without a curriculum (Joseph's
# "Launch experiment → set target manually" use case).  Curriculum stage
# entry also writes this field, so a single source of truth.  Values:
#   "off"            — no target selected; nearest_pyramid_dist still computed
#   "random_pyramid" — pick & glow a target; rotate to a new one on visit
@export var target_mode: String = "off"
# 2026-06-08 — runtime knob to scale CruseCoordinator's bias output.
# Joseph's diagnosis: Cruse rule 3 fires ~100/sec even during pure standing
# (phantom-swing detections on contralateral pairs) → 0.55-magnitude bias
# on knee Premotors during stage 1.  This scales the bias by `cruse_bias_gain`
# so curriculum stage 1 can silence Cruse for standing learning, and stage 2
# restores it for walking coordination.  Setter routes the value into the
# brain's HotMutable cruse_bias_gain via apply_patch — fires on @export
# write or curriculum override write.  Default 1.0 = full bias (legacy).
@export var cruse_bias_gain: float = 1.0:
	set(v):
		cruse_bias_gain = v
		_apply_cruse_bias_gain_to_brain()
# 2026-06-08 — per-joint-kind gain.  Default 0.0 means Cruse does NOT bias
# the knee.  V2 trace exposed knee_bias 0.4-0.5 driving a claw fold during
# stance — Cruse's stance/swing coordination layer is the wrong gradient
# for the knee.  hip2 (lift) stays at full cruse_bias_gain.  Knees learn
# their own timing from foot-clearance + body-height signals.
# Joseph 2026-06-08: "concentrate the Cruse rules on hip1 and hip2 for
# lift and swing.  let the knees learn on their own how to manage the gait."
@export var cruse_bias_gain_knee: float = 0.0:
	set(v):
		cruse_bias_gain_knee = v
		_apply_cruse_bias_gain_knee_to_brain()
# 2026-06-08 Move 2 — Cruse bias gain on HIP1 (yaw / swing direction).  Default
# 0.0 = legacy (hip1 unbiased, no swing-coordination signal — the gating root
# cause Joseph identified for no-gait-emergence in matrix nav nulls).  Stage 2
# curriculum sets to 1.0 to enable: during stance, hip1 biased toward posterior
# (body forward through traction); during swing (Rule 2 release), toward
# anterior (leg lifts forward).  Per-leg direction via premotor_stance_sign
# in graph config — picrawler empirical [+1,-1,+1,-1] for FL/FR/RL/RR hip1.
@export var cruse_bias_gain_hip1: float = 0.0:
	set(v):
		cruse_bias_gain_hip1 = v
		_apply_cruse_bias_gain_hip1_to_brain()
# 2026-06-09 Move 4 — Cruse bias gain on HIP2 (lift / vertical axis).  Default
# 1.0 = legacy (no per-joint scaling — hip2 used the implicit cruse_bias_gain
# alone).  Set > 1.0 to amplify the down-during-stance + up-during-swing
# biases that planted/lifted the leg, without scaling hip1 and knee.
# Joseph 2026-06-09: "a bit of hip2 bias downward (positive) during the
# hip1 swings would help plant the thrusting leg.  a bit more hip2 upward
# (negative) would help with swing forward."
@export var cruse_bias_gain_hip2: float = 1.0:
	set(v):
		cruse_bias_gain_hip2 = v
		_apply_cruse_bias_gain_hip2_to_brain()
# 2026-06-09 Move 5 — position-aware bias gate.  When enabled, Cruse reads
# current joint position and computes productive_score = current_u ×
# bias_direction_sign.  If outside [zone_min, zone_max], bias is zeroed
# (suppresses rudder / saturation failure modes).  Default disabled.
@export var saturation_gate_enabled: bool = false:
	set(v):
		saturation_gate_enabled = v
		_apply_saturation_params_to_brain()
@export_range(-1.0, 1.0, 0.05) var saturation_zone_min: float = 0.0:
	set(v):
		saturation_zone_min = v
		_apply_saturation_params_to_brain()
@export_range(0.0, 1.0, 0.05) var saturation_zone_max: float = 0.9:
	set(v):
		saturation_zone_max = v
		_apply_saturation_params_to_brain()
var _walk_accumulator: float = 0.0
# Phase 6.12 — separate accumulator for inward-motion misses under the
# radial_penalize_inward mode.  Tracks fractional ticks of penalty and
# fires events.miss when ≥ 1.0 — same accumulator pattern as the hit
# side so penalty magnitude is symmetric to reward magnitude.
var _walk_miss_accumulator: float = 0.0
# 2026-06-11 — last walk_reward_mode seen by the miss-drain block.  The
# miss accumulator only DRAINS under the penalize modes, but to_target
# also ACCUMULATES into it; a live mode switch (UI slider / curriculum)
# could otherwise dump a stale residual as a burst of miss events the
# moment a draining mode is entered.  On any mode change the residual
# is zeroed instead — penalties never carry across reward regimes.
var _walk_miss_mode_prev: String = ""
var walk_miss_fired_count: int = 0   # telemetry: drained miss events (session cumulative)

# Phase 7.x — heading-consistency reward.  Penalises wandering by
# pulsing events.hit only when current velocity aligns with the EMA
# of recent velocity (i.e., sustained heading).  Rapid heading
# changes → low dot product → no hit accumulation.  Acts as a
# substrate-derived anti-wander pressure layered on top of the
# radial reward.  The EMA tracks the velocity VECTOR (not normalised
# direction) so it naturally decays toward zero when the body stops,
# giving a clean cold-start where no consistency reward fires until
# the body has actually built up sustained motion.  No new tuned
# threshold — the EMA window itself derives the timescale.
@export var walk_heading_consistency_gain: float = 1.0     # multiplier on walk_hit_rate for the consistency pulse
const _HEADING_EMA_ALPHA: float = 0.05                     # ~1 sec @ 60Hz — short enough to track turns, long enough to detect oscillation
var _heading_v_ema: Vector2 = Vector2.ZERO
var _heading_hit_accumulator: float = 0.0

# Phase 7.x — walk_over_there target tracking.  When the active stage
# spec includes target_mode = "random_pyramid", the body selects a
# random pyramid as the current target, recolors it purple, and uses
# walk_reward_mode = "to_target" semantics (penalize moving away,
# reward moving toward).  Arrival within visit_radius increments
# walk_visit_count and rotates to a new random pyramid.
var walk_target_idx:   int = -1                            # index into _pyramid_xz_positions; -1 = no target
var walk_target_pos:   Vector2 = Vector2.ZERO              # xz of current target apex
var walk_visit_radius: float = 0.5                         # m — "arrived" threshold (legacy reward path)
var walk_visit_count:  int = 0                             # cumulative arrivals (curriculum metric)
# 2026-06-13 — reward-free arrival requires actual CONTACT with the TARGET
# pyramid's surface (operator: "the actual target must be touched", not a radius
# and not a nearby pyramid).  Fire when chassis-to-target SURFACE clearance drops
# below this small chassis-contact margin.  Measured to the target specifically,
# so contact with any other pyramid never triggers a switch.
const TARGET_TOUCH_MARGIN: float = 0.15
@export var walk_visit_bonus_intensity: float = 5.0        # events.hit intensity emitted on arrival
var _walk_target_rng: RandomNumberGenerator = RandomNumberGenerator.new()
var _resolved_seed: int = -1                                # cached from _ready for sub-RNG derivation

# ----------------------------------------------------------------------
# Gait-cycle reward (2026-05-30) — discrete events.hit pulse fired when
# all 4 feet have completed a touchdown within a rolling window AND
# the net directional displacement over the window meets the gait-quality
# bar.  Unlike the per-tick standing / gated_walk / level_chassis streams
# (which reward tick-local state), this reward targets the LIMIT-CYCLE
# structure directly: a cycle is "good" iff it (a) involves all 4 legs
# striking down, (b) produces net forward progress in the active
# walk_reward_mode direction, and (c) contains no significant backward
# excursion (anti-wobble guard against reward-hacking via lateral or
# oscillatory motions).  Direction follows walk_reward_mode:
#   radial / radial_penalize_inward → v · r̂  (signed radial outward)
#   to_target                       → v · (target - r̂)  (signed approach)
#   total_speed                     → |v_xz|  (no direction; backward
#                                              guard inactive)
# Constant-intensity pulse (gait_cycle_reward_gain) per good cycle.
# Disabled by default (gain=0) — opt-in via curriculum or reward_panel.
@export var gait_cycle_reward_gain:   float = 0.0    # 0 = mechanism off
@export var gait_cycle_window_ticks:  int   = 120    # 2 s @60Hz — cycle times out if all 4 feet don't touch down
@export var gait_cycle_min_progress:  float = 0.005  # m — SAFETY FLOOR for min signed progress (effective bar = max(this, K·EMA) when adaptive)
@export var gait_cycle_max_backward:  float = 0.002  # m — SAFETY FLOOR for max backward excursion (effective bar = max(this, K·EMA) when adaptive)

# Adaptive thresholds (2026-05-30) — instead of static numeric thresholds
# the bars float as fractions of EMAs of the body's own observed cycle
# statistics.  Rewards "better than average" cycles, so the target rises
# as the body improves.  Safety floor: effective thresholds are never
# below the static @export values above, so a degenerate (no-cycle)
# state can't make the bar trivially permissive.
@export var gait_cycle_adaptive_thresholds: bool  = false  # opt-in
@export var gait_cycle_progress_K:          float = 0.5    # eff_min_progress = max(static, K·progress_ema)
@export var gait_cycle_wobble_K:            float = 2.0    # eff_max_backward = max(static, K·wobble_ema)
@export var gait_cycle_threshold_ema_alpha: float = 0.02   # ~50-cycle horizon
@export var gait_cycle_warmup_cycles:       int   = 20     # cycles before adaptive kicks in

# Consecutive-cycle gate (2026-05-31) — targets limit-cycle continuity.
# A single 4-foot cycle is easy to fake via a forward lurch (v2 finding:
# 17 good cycles produced 4× the fall rate of control).  Requiring N
# consecutive good cycles with no aborts in between raises the bar to
# something only sustained gait can satisfy: each cycle must clear the
# progress/wobble guards AND no failed cycle may occur between them.
# Reward fires every good cycle once the streak reaches N (so sustained
# gait keeps paying; falling out of cycle stops the pay).  Default 1 is
# back-compat with v1/v2/v3 (fire on every good).  Same-direction
# requirement is implicit: all good cycles have progress >= min_progress
# > 0 in the active walk_v direction, so consecutive good cycles are
# automatically in the same direction.
@export var gait_cycle_consecutive_required: int = 1

var _gait_cycle_active:        bool  = false
var _gait_cycle_start_tick:    int   = 0
var _gait_cycle_displacement:  float = 0.0           # ∫ walk_v · dt since cycle start (signed, active direction)
var _gait_cycle_max_displacement: float = 0.0        # running max for backward-excursion check
var _gait_cycle_max_backward_seen: float = 0.0       # running max of backward excursion this cycle (for wobble EMA)
var _gait_cycle_legs_touched:  Array = [false, false, false, false]
var _foot_was_in_contact:      Array = [false, false, false, false]
var _gait_cycle_good_count:           int = 0       # cumulative successful gait cycle completions (all good cycles, even pre-streak)
var _gait_cycle_pulses_fired:         int = 0       # cumulative events.hit emissions (good cycles >= consecutive_required)
var _gait_cycle_consecutive_good:     int = 0       # current good-cycle streak (reset on any abort)
var _gait_cycle_consecutive_good_max: int = 0       # best streak observed in this run
var _gait_cycle_aborted_wobble:       int = 0       # cumulative aborts: backward excursion exceeded
var _gait_cycle_aborted_timeout:      int = 0       # cumulative aborts: window expired without 4-foot touchdown
var _gait_cycle_aborted_low_progress: int = 0       # cumulative aborts: 4 feet touched down but net progress under bar
var _gait_cycle_attempts_total:       int = 0       # cumulative cycle endings of ANY reason (for warmup gate)
var _gait_cycle_last_net_progress:    float = 0.0   # last completed cycle's net signed progress (good or bad)
var _gait_cycle_progress_ema:         float = 0.0   # EMA of |net_progress| over ended cycles (drives adaptive min_progress)
var _gait_cycle_wobble_ema:           float = 0.0   # EMA of max_backward_seen over ended cycles (drives adaptive max_backward)
var _gait_cycle_eff_min_progress:     float = 0.0   # currently-effective bar (telemetry)
var _gait_cycle_eff_max_backward:     float = 0.0   # currently-effective bar (telemetry)
var _hit_delta_gait_cycle:    float = 0.0
var _hit_cum_gait_cycle:      float = 0.0
var _hit_rate_gait_cycle_ema: float = 0.0

# Stability shaping (v6.0.b.3) — push toward "elevated AND still" by
# emitting a small per-tick events.miss when the body is elevated AND
# moving fast.  Couples to NeurochemState's existing miss handler
# (-da_miss_drop × intensity, -ht_miss_drop × intensity).  Disables the
# reward-hacking "jump up for instant reward, fall, repeat" pattern
# observed in A3 mc_period sweep.  All knobs env-overridable:
#   OGMA_PICRAWLER_STAB_Y_NORM    elevation threshold for stability gate
#                                 (0..1 fraction of STANDING_CHASSIS_Y)
#   OGMA_PICRAWLER_STAB_SPEED     chassis linear speed threshold (m/s)
#   OGMA_PICRAWLER_STAB_GAIN      events.miss intensity per tick when
#                                 stability is violated.  Set 0 to
#                                 disable the mechanism.
@export var stability_y_norm:     float = 0.5
@export var stability_speed:      float = 0.05
@export var stability_gain:       float = 0.0      # 0 = mechanism off (default)

# Anti-rotation shaping (v6.0.b.4) — addresses the long-horizon
# tip-backwards failure mode.  Under the sustained-standing policy
# learned at 18k ticks, the body accumulates small yaw/pitch asymmetry
# over hundreds of cycles and eventually tips.  This per-tick
# events.miss penalises angular velocity above a threshold, giving
# the brain a continuous gradient to correct rotational drift even
# when chassis_y is already maxed (and the height reward has
# saturated through the adaptive baseline EMA).
#
# Intensity scales linearly from 0 (at threshold) to full gain (at
# threshold + scale).  All knobs env-overridable + ExperimentConfig-
# resolvable:
#   OGMA_PICRAWLER_ANTIROT_THRESHOLD  rad/s; below this no penalty
#   OGMA_PICRAWLER_ANTIROT_SCALE      rad/s; full intensity at THRESHOLD+SCALE
#   OGMA_PICRAWLER_ANTIROT_GAIN       events.miss intensity at full scale.
#                                     0 = mechanism off (default).
@export var antirot_threshold: float = 0.5
@export var antirot_scale:     float = 1.0
@export var antirot_gain:      float = 0.0      # 0 = mechanism off (default)

# Energy-cost shaping (v6.0.b.7) — per-tick events.miss intensity
# proportional to total motor mechanical power consumed.  Power per
# joint = |torque| × |angular_velocity|, summed across all 12 servos.
# Unlike antirot/stab which gate on threshold, energy provides a
# SMOOTH continuous gradient — every action sample pays a cost
# proportional to its wastefulness.  Mechanistically distinct from
# the three null Stage B mechanisms (antirot, drive_posture, tilt EPM)
# — those failed because their signals got absorbed into the
# REINFORCE trajectory variance; energy cost shapes EVERY action's
# attribution, not just trajectory-level reward.
#
# Generalises directly to walking: efficient gaits naturally minimise
# Σ|τ|·|ω|.  Standing without twitching is the minimum-energy attractor
# for this body.
#
#   OGMA_PICRAWLER_ENERGY_DEADBAND  W; below this no penalty (allows
#                                   minimum-motion baseline).  Default 0.5 W.
#   OGMA_PICRAWLER_ENERGY_SCALE     W; full intensity at deadband + scale.
#                                   Default 5.0 W (~half of max possible).
#   OGMA_PICRAWLER_ENERGY_GAIN      events.miss intensity at full scale.
#                                   0 = mechanism off (default).
@export var energy_deadband: float = 0.5
@export var energy_scale:    float = 5.0
@export var energy_gain:     float = 0.0      # 0 = mechanism off (default)
var _motor_power_last_tick:  float = 0.0      # written by motor block, read by reward block

# Phase 7.16 — Continuous level-chassis reward (UNGATED).  Fires every tick
# regardless of fall state, giving the body a continuous gradient toward
# upright posture EVEN WHEN FALLEN.  Distinct from Phase 6.16 tilt_target_rad
# (which is a multiplicative factor on the standing reward and is gated by
# `if not fell`).
#
# Motivation: user UI observation (Phase 7.15 lean ablation regression).
# Removing perceptual CPG + rhythm EPMs increased tipover rate from 3.4% to
# 9.1% — the body had no continuous orientation signal pulling it back toward
# level when not actively standing tall.  FAIL_TILT was a binary cliff, not a
# gradient.  This adds the missing gradient.
#
# intensity = tilt_factor × level_chassis_rate
#   tilt_factor = clamp(1 - |chassis_tilt| / level_chassis_tilt_scale, 0, 1)
# A body upright earns full bonus.  A body at PI/2 tilt earns zero.
# Composes additively with standing/walking/gated rewards.
@export var level_chassis_rate:           float = 0.0     # 0 = mechanism off (default)
@export var level_chassis_walking_budget: float = 0.5236  # ~30° — tilt below this earns FULL bonus
														   # (lets walking-tilt earn full reward; only excess tilt is penalised)
@export var level_chassis_tilt_scale:     float = 1.5708  # ~90° — tilt beyond this earns zero bonus
														   # (linear decay between walking_budget and tilt_scale)
var _hit_delta_level_chassis: float = 0.0
var _hit_rate_level_chassis_ema: float = 0.0
var _hit_cum_level_chassis:   float = 0.0

# Ablation overrides (v6.0.b.8) — used by the Stage B ablation lattice
# to falsify alternative hypotheses about what produces the standing
# behaviour.  Both default off; the brain runs normally.
#   brain_off=true       → action channels forced to 0 every tick
#                          (body PD-holds the rest pose; tests "is the
#                          motor architecture alone enough to stand?")
#   random_policy=true   → action channels drawn from uniform[-1, 1]
#                          (tests "does any noise on the motors get
#                          you anywhere?")
# When both are true random_policy wins (random > zero in semantics).
@export var brain_off:     bool = false
@export var random_policy: bool = false
var _rand_policy_rng: RandomNumberGenerator = null
# 2026-06-10 — E0 scripted-gait diagnostic (oscillator ladder, capability_ladder
# plan).  Open-loop per-leg joint drive for driven-dynamics system ID: replaces
# the brain u with a parameterized gait waveform so we can measure whether ANY
# driven oscillation translates this body without tipping.  Ablation-mode sibling
# of random_policy / brain_off; default off → byte-identical legacy behaviour.
# foot traces an ellipse: u_hip1 = A1·sin(phi) (fore-aft), u_hip2 = A2·sin(phi+90°)
# (lift, quarter-cycle ahead), optional knee couples to lift.  Per-leg phase from
# SG_PATTERN.  Env: OGMA_PICRAWLER_SCRIPTED_GAIT=1 + SG_AMP_HIP1/HIP2/KNEE,
# SG_PERIOD (ticks), SG_PATTERN (trot|walk|pace|bound).
var _scripted_gait: bool = false
var _sg_amp_hip1: float = 0.5
var _sg_amp_hip2: float = 0.5
var _sg_amp_knee: float = 0.4
var _sg_period:   float = 60.0
var _sg_hip1_phase: float = PI   # fore-aft phase offset (propulsion timing); sweepable
var _sg_offsets:  Array = [0.0, PI, PI / 2.0, 3.0 * PI / 2.0]   # default = walk (lateral-sequence, canonical)

func _sg_pattern_offsets(name: String) -> Array:
	# Per-leg cycle phase offsets in [FL, FR, RL, RR] order (LEG_NAMES).
	match name:
		"walk":  return [0.0, PI, PI / 2.0, 3.0 * PI / 2.0]   # lateral-sequence
		"pace":  return [0.0, PI, 0.0, PI]                    # ipsilateral pairs
		"bound": return [0.0, 0.0, PI, PI]                    # front pair vs rear pair
		_:       return [0.0, PI, PI, 0.0]                    # trot (default)

# Drive posture channel (v6.0.b.5) — when enabled, the body publishes a
# 1-D `reality.proprio.tilt` topic containing chassis_tilt / FAIL_TILT.
# A HomeostaticDrive channel of kind "proprio_passive" subscribed to
# this topic produces a continuous urgency signal that the brain learns
# to minimise (setpoint=0 → standing level).  Distinct from the
# antirot events.miss mechanism: events.miss couples to dopamine
# (REINFORCE reward signal); drive urgency couples to the
# ActionDecoder's drive_errors topic (a separate gradient path).
#
# Opt-in (default off) so existing baselines reproduce byte-identically.
@export var publish_tilt: bool = false

# Ragdoll mode: when true, all servo torques are disabled.  Brain still
# ticks (proprio published, predictions made), so the perception/learning
# loop continues — only the actuation arm is muted.  Toggle with R key.
# DEFAULT OFF as of v6.0.b.2 — the previous ragdoll-on default was a
# debugging hack from when the construction→standing slew under powered
# servos blew up (knee propellers from limit-wrap, hips snap to limit
# from physics_hz× motor authority).  Both root causes are fixed in
# commit c03d2f6 (symmetric ±1.7 knee limits + impulse÷physics_hz +
# motor-velocity sign negation), so motors-on at boot is now safe.
# Launching with motors enabled mirrors the UI flow "R then SPACE"
# (toggle ragdoll off, then hard-reset to standing pose) and lets the
# headless harness measure brain-driven behaviour from t=0.
var _ragdoll_mode: bool = false

# Spacebar manual reset.  Queued from _input and consumed at the top of
# the next physics step so the freeze→write→unfreeze cycle takes effect.
var _pending_manual_reset: bool = false

# Pending chassis suspend/unfreeze: queued by C-key input, consumed at
# the top of the next physics frame so transform writes actually stick
# (writing chassis.global_transform from the idle-frame input handler
# gets clobbered by physics on the next step — same issue as manual
# reset).  Values: 0 = no change, 1 = suspend, 2 = unfreeze.
var _pending_chassis_freeze: int = 0

# Servo calibration mode (C key).  Cycles all 12 servos through a
# scripted sequence of poses to verify joint range, no explosions, and
# hobby-servo-like response.  When active, brain commands are ignored
# and per-channel targets come from CALIBRATE_SEQUENCE.
# Each entry: [hip1_u, hip2_u, knee_u, hold_ticks, label]
# u values are in brain-command space [-1, +1] which the PD maps to
# the joint's target_range.
var _calibrate_mode: bool = false
# Phase 7.x — when both _calibrate_mode AND _cpg_drive_calibrate are
# true, the C-mode FK chain reads servo targets from the brain's
# action channels (action.<joint> = Premotor + CPG blended) every
# tick instead of from the user's sliders.  Lets the operator
# visualise the live motor commands the body WOULD receive in free
# motion, with the chassis suspended so the gait pattern can be
# inspected without the body falling.
var _cpg_drive_calibrate: bool = false
# Phase 7.x — tracks whether the leg bodies have been switched from
# C-mode's KINEMATIC freeze to DYNAMIC (for CPG-drive visualisation).
# Toggled in lockstep with _cpg_drive_calibrate so the leg dynamics
# only run when the operator opted in.
var _legs_unfrozen_for_cpg_drive: bool = false
var _calibrate_step: int = 0
var _calibrate_step_start_tick: int = 0

# Motor-test mode (G key).  Reuses the servo calibration panel sliders
# but keeps the chassis on the ground and drives joints through the
# real motor PD chain instead of the FK short-circuit.  Lets the user
# verify whether the 0.15 Nm servos can hold the body's weight in any
# given pose — i.e. is the body mechanically standable at all?
# Overrides ragdoll while active so motors are powered.  Brain action
# inputs are ignored; slider values map directly to joint targets via
# t = servo_targets[k] * servo_signs[k] + servo_origins[k].
var _motor_test_mode: bool = false
var _brain_paused_notified: bool = false   # one-shot log when calibration pauses the brain
# ---- 2026-08-03 · GANGED JOINT DRIVE for spring/damping characterisation -------------
# Displacing one joint tells you little; a leg is a coupled chain and the body only
# resonates when a JOINT GROUP moves together.  These drive all four hip2s (or all four
# knees) as one, so the operator can PULSE (step, release, watch the ring-down → damping
# ratio and natural frequency) or SHAKE (sinusoid at a swept frequency → find the
# amplitude peak = resonance).  Active only in G motor-test mode.
var _gang_hip2_base: float = 0.0
var _gang_knee_base: float = 0.0
var _gang_pulse_ticks: int = 0        # >0 = a pulse is being held
var _gang_pulse_amp: float = 0.0
var _gang_pulse_group: int = 1        # 1 = hip2, 2 = knee, 3 = both
var _gang_shake_hz: float = 0.0       # 0 = off
var _gang_shake_amp: float = 0.0
var _gang_shake_phase: float = 0.0
# ring-down readout: peak-to-peak of the measured joint angle over a rolling window
var _gang_pp_hip2: float = 0.0
var _gang_pp_knee: float = 0.0
var _gang_win_hi2: float = -9.0
var _gang_win_lo2: float = 9.0
var _gang_win_hik: float = -9.0
var _gang_win_lok: float = 9.0
var _gang_win_n: int = 0
# 2026-06-09 — HUD visibility toggles bound to T and H hotkeys.  Joseph QoL
# ask: clean visual access to the 3D scene during UI observation.  T hides
# all picrawler HUD panels (curriculum / reward / trainer); H hides the
# main HUD text but keeps the hint line visible (quadruped_hud reads
# hud_hidden to suppress everything else).  Both default false.
var _panels_hidden: bool = false
var hud_hidden:     bool = false
# 2026-06-13 — vision-sensor debug aids.  V = toggle the loom/vision ray overlay
# (3D lines coloured by what each ray hits — RED = self-occlusion, the
# camera-inside-chassis tell); M = toggle the camera+LiDAR pixel panels.  For V1
# vision recon: verify the ray origin clears the chassis + see what the EPM sees.
var _ray_overlay_on:  bool = false
var _ray_overlay_mi:  MeshInstance3D = null
var _ray_overlay_mat: StandardMaterial3D = null
var _vision_panel_on: bool = false
const RAY_OVERLAY_SELF_OCCLUSION_M: float = 0.12   # hit nearer than this → RED (origin inside body)
# 2026-06-13 — vision "brain view": a forward-facing perspective raycast → RGB
# pixel array (the traditional Cell-style image the vision EPM will receive in
# V1), rendered as a HUD panel.  Shown when the top-down map cam is active (same
# UX as the Cell).  Colours: purple = active target, gray = ground, tan = object,
# dark = sky/miss.  This IS the V1 vision sensor; the panel is its viz.
# ---- 2026-08-04 · THE CAMERA IS NOW MODELLED ON THE REAL SENSOR ------------------------
# The previous values (32x32, 90 deg square) carried NO hardware citation. This whole path
# "originated as a HUD debug panel that was promoted to a sensor": the shading exists so the
# EPM has gradients, the capture rate is a CPU budget, the ray length is the arena diagonal,
# and the mount was "high enough not to hit the legs". Compare docs/servo_dynamics.md, which
# gets a cited reference, a conformance table and four argued deviations. The camera was the
# least-specified part of the model.
#
# HARDWARE (operator, 2026-08-04): SunFounder PiCrawler, OmniVision **OV5647**, quoted 65 deg.
# That 65 is the DIAGONAL: the OV5647's published 53.5 x 41.4 H x V has a computed diagonal of
# 64.4 deg, which matches. So the faithful optics are 53.5 x 41.4, 4:3.
# ⚠ If that reading is ever corrected to 65 HORIZONTAL, the model becomes 65 x 51.1 and every
# range figure in the plan shifts -- change it here and re-run the Stage-0 characterisation.
const VISION_FOV_H_RAD: float = 0.9337511          # 53.5 deg horizontal (OV5647)
const VISION_FOV_V_RAD: float = 0.7225663          # 41.4 deg vertical   (OV5647, 4:3)
# RESOLUTION IS THE OPTIC NERVE, NOT THE PHOTORECEPTOR ARRAY -- and it is named as such.
# A GPU render is unavailable (Godot --headless disables the rendering server entirely, so a
# SubViewport readback returns null pixels; the Cell hit this and documented it at
# body_controller.gd:331-339), and 640x480 raycasts is infeasible regardless. So we model what
# the brain RECEIVES, not what the sensor captures, and we are exact about the geometry.
#
# 32x24 keeps the sensor's 4:3 aspect with SQUARE pixels. It is 768 rays against the old 1024
# -- 25% CHEAPER -- and resolves ~3x more beacon area at range, because a narrower FOV over the
# same budget is finer. Measured blob for a 1.0 x 0.36 m pyramid at 8 m: 2.3 px at the old
# 32x32/90deg vs 6.4 px here.
# A VAR, not a const (the Cell's OGMA_VIS_RES precedent) so Stage 0 can raise it to 48x36.
# ---- 2026-08-04 · THE BEACON A/B CONTROL (operator's design) ---------------------------
# The sharpest control for "is nav affecting behaviour at all" is not a module-side ablation
# (which depends on the module honouring a param — and our shuffle arm came back
# byte-identical, so that assumption is exactly what is in doubt).  It is a WORLD-side one:
# run the same seed twice, once with the target pyramid painted and once left neutral.
#
#   visible=1 -> the landmark exists -> beacon carries signal -> nav can steer
#   visible=0 -> the landmark does not exist -> beacon is identically 0 -> nav has nothing
#
# Everything else — layout, seed, target selection, arrival detection, tgt_range logging — is
# untouched, so ANY behavioural difference is attributable to the nav loop and nothing else.
# This is also the honest form of the control: the world genuinely lacks the landmark, rather
# than the perception being lesioned.
@export var beacon_visible: bool = true
@export var vision_res_w: int = 32
@export var vision_res_h: int = 24
const VISION_LIGHT_DIR: Vector3 = Vector3(0.4165, 0.8538, 0.3124)   # key-light dir (unit)
const VISION_AMBIENT: float = 0.35                 # ambient floor so shadowed faces aren't black
# 2026-06-13 — wire the camera/LiDAR raycast into the BRAIN (not just the HUD).
# When publish_vision is on the body captures + publishes host.video.color every
# tick so an epm_color (JL) EPM can encode it into the consensus (V1 vision).
# Capture at a subrate (CPU), publish the cached frame every tick (sub-rate
# publishes get filtered out of the voter trust map).
@export var publish_vision: bool = false
# 2026-06-14 — gaze stabilisation (operator: the camera rocks with the gait, so
# the image→bearing mapping is corrupted by chassis pitch/roll).  When on, the
# camera is gravity-levelled and follows only the chassis YAW (heading) — a gimbal
# / vestibulo-ocular reflex — so a target sits at a stable image column regardless
# of the step bob.  Default on for vision.
@export var vision_stabilized: bool = true
# 2026-06-14 — V2: VISION-DERIVED STEERING.  A fixed linear readout of the
# epm_color latent → (tc_x,tc_y) bearing (fit by scripts/v2_fit_vision_steer.py),
# published as reality.proprio.vision_compass so MotorEPM's nav can steer on
# VISION instead of the ground-truth target_compass oracle.  Implies publish_vision.
@export var vision_steer: bool = false
const VISION_STEER_READOUT_PATH: String = "res://addons/ami_ogma/configs/vision_steer_readout.json"
var _vsteer_wx: PackedFloat64Array = PackedFloat64Array()
var _vsteer_wy: PackedFloat64Array = PackedFloat64Array()
var _vsteer_bx: float = 0.0
var _vsteer_by: float = 0.0
var _vsteer_loaded: bool = false
var _vision_compass: Vector2 = Vector2.ZERO
const VISION_CAPTURE_EVERY: int = 3                # ~20 Hz capture; publish cached every tick
var _vision_capture_counter: int = 0
# One forward raycast grid yields TWO sensor modalities (operator insight
# 2026-06-13): the hit COLOUR/class = a camera (RGB), and the hit DISTANCE = a
# LiDAR/sonar range field.  We keep both — the depth/range modality is the
# sim2real-friendly one (real quadrupeds carry cheap ToF/ultrasonic arrays far
# more often than a calibrated camera).  The loom was already this range sensor,
# collapsed to a scalar; this exposes the full field.
var _last_vision_pixels: PackedByteArray = PackedByteArray()   # RGB (camera)
var _last_vision_depth:  PackedFloat32Array = PackedFloat32Array()  # range/LOOM_RAY_LEN ∈ [0,1] (LiDAR)
var _vision_panel:   PanelContainer = null
var _vision_rgb_rect:   TextureRect = null
var _vision_depth_rect: TextureRect = null
var _vision_rgb_image:   Image = null
var _vision_depth_image: Image = null
var _vision_rgb_texture:   ImageTexture = null
var _vision_depth_texture: ImageTexture = null
# 2026-06-13 — panic pathway GATE 0: distress signal (NO behaviour change yet).
# deficit = chassis horizontal displacement over a window vs walking-speed
# CAPACITY (ported from the Cell's _stuck_severity: 0 = free, 1 = wedged).
# distress = a slow accumulator over the deficit, meant to separate a sustained
# WEDGE from the gait's normal ~10 s pause/resume.  We log deficit + distress +
# tilt and verify on seed 50 (reliably wedges) BEFORE designing the final trigger
# — the picrawler's long pauses may confound displacement alone, in which case a
# pose (tilt/height) term gets OR'd in.  See motor_epm_panic_pathway_plan.md.
const DISTRESS_WINDOW_TICKS: int = 120        # 2 s net-displacement window (jiggle-robust)
const DISTRESS_REF_SPEED: float = 0.08        # m/s walking capacity (deficit normaliser)
const DISTRESS_WARMUP_TICKS: int = 600        # 10 s warmup — don't judge while finding feet
const DISTRESS_TILT_EMA_ALPHA: float = 0.02   # ~0.8 s tilt smoothing (the wedge tilt OSCILLATES)
const DISTRESS_PERCH_LO: float = 0.15         # rad — tilt_ema below this = level (perch=0)
const DISTRESS_PERCH_HI: float = 0.30         # rad — tilt_ema at/above this = full perch (=1)
const DISTRESS_RISE: float = 0.006            # accumulate rate × stuck_score
const DISTRESS_DECAY: float = 0.004           # decay rate × (1 − stuck_score)
var _distress_pos_history: Array = []         # ring of chassis XZ (Vector2)
var _stuck_deficit: float = 0.0               # fast 2 s net-displacement deficit (stall, 0..1)
var _tilt_ema: float = 0.0                    # smoothed |tilt| (perch evidence)
var _distress: float = 0.0                    # slow PERCH×STALL accumulator (the panic signal)
var _distress_hud: Label = null
const CALIBRATE_SEQUENCE: Array = []   # superseded by SERVO_CALIBRATE_SEQUENCE below
# Per-servo calibration sweep — one servo at a time, others held at
# center.  Each servo cycles: center → +limit → center → −limit
# with a 250ms hold (12 brain ticks at 50Hz) between transitions.
# Auto-advancing (no SPACE needed).  If every servo completes without
# physics explosion, the sim is hardware-faithful enough to proceed.
#
# Tuple format: [leg_index (0=FL,1=FR,2=RL,3=RR), servo (0=hip1, 1=hip2,
# 2=knee), command_u (-1 to +1), label]
const SERVO_CALIBRATE_HOLD_TICKS: int = 25   # used for auto-advance when enabled
# If true, calibration auto-advances every HOLD_TICKS.  If false, only
# advances when user presses N (and P goes back one step).  Default
# manual so the user can hold any step indefinitely while describing
# what's wrong with it.
var calibrate_auto_step: bool = false
const SERVO_CALIBRATE_INIT_HOLD_TICKS: int = 50   # 50 * 20ms = 1s initial X-stance hold
const SERVO_NAMES: Array = ["hip1", "hip2", "knee"]
var SERVO_CALIBRATE_SEQUENCE: Array = []     # built lazily in _ready
func _build_servo_calibrate_sequence() -> void:
	SERVO_CALIBRATE_SEQUENCE.clear()
	# Phase 1 — initial "all servos center" hold so the user sees the
	# canonical X-stance (legs splayed at corner-outward 45°, knees bent
	# so lower legs vertical) before any per-servo testing.  leg_index=-1
	# is a sentinel meaning "no servo is active — hold all at center".
	SERVO_CALIBRATE_SEQUENCE.append([-1, -1, 0.0, "INIT: X-stance (all at center)"])
	# Phase 2 — per-servo sweep: center → +max → center → −max for each
	# of the 12 servos in turn.
	for leg in range(4):
		for servo in range(3):
			for u in [0.0, +1.0, 0.0, -1.0]:
				var lbl: String = "%s %s u=%+.1f" % [LEG_NAMES[leg], SERVO_NAMES[servo], u]
				SERVO_CALIBRATE_SEQUENCE.append([leg, servo, u, lbl])

# Public-ish state for the HUD to colour the cal-step display.
var calibrate_current_leg: int = -1     # 0..3 = FL/FR/RL/RR, -1 = init phase
var calibrate_current_label: String = ""
# Set when a step has logged its "entered" validation print, cleared on
# step change so each new step prints once.
var _calibrate_step_logged: bool = false

# Per-servo state for the calibration panel.  Indexed by SERVO_IDX(leg, jt)
# where leg ∈ [0..3] (FL/FR/RL/RR) and jt ∈ [0..2] (hip1/hip2/knee).
# Idx = leg*3 + jt → 12 entries.
#   servo_targets: commanded angle in RADIANS (slider value).
#   servo_signs: ±1, multiplies the slider before adding origin.
#   servo_origins: joint-frame angle at servo center (slider = 0).
# Effective PD target: t = servo_targets[idx] * servo_signs[idx] + servo_origins[idx]
var servo_targets: Array[float] = []
var servo_signs:   Array[float] = []
var servo_origins: Array[float] = []
const SERVO_NAMES_FULL: Array = ["hip1", "hip2", "knee"]
static func servo_idx(leg: int, jt: int) -> int:
	return leg * 3 + jt

# ---------------------------------------------------------------------------
# Boot
# ---------------------------------------------------------------------------
func _ready() -> void:
	# FIRST — the body's own dimensions, before anything reads or builds them.
	# Must precede the env/curriculum resolution further down (_resolve_env,
	# ExperimentConfig.resolve_*) so those overrides still win over the body's
	# defaults rather than being stomped by them.
	_load_geometry()

	# Apply physics oversampling.
	Engine.physics_ticks_per_second = physics_hz
	ProjectSettings.set_setting("physics/3d/solver/solver_iterations", solver_iterations)

	# Turbo mode (OGMA_TURBO=1) — wall-decoupled execution.  Combined with
	# `--fixed-fps <N> --disable-render-loop --headless`, the engine
	# advances physics at sim-rate but never sleeps to wall.  Substrate
	# time constants are unchanged because physics_ticks_per_second stays
	# at `physics_hz` (default 60).  Mirrors body_controller.gd:262-273.
	if OS.get_environment("OGMA_TURBO") == "1":
		Engine.max_physics_steps_per_frame = 64
		print("PicrawlerBody: TURBO mode — max_physics_steps_per_frame=64")
	var quit_env: String = OS.get_environment("OGMA_QUIT_AFTER_TICKS")
	if quit_env != "":
		_quit_after_ticks = quit_env.to_int()
		print("PicrawlerBody: OGMA_QUIT_AFTER_TICKS=%d" % _quit_after_ticks)
	var tramp_env: String = OS.get_environment("OGMA_PICRAWLER_TELEPORT_RAMP_AT")
	if tramp_env != "":
		_teleport_ramp_at = tramp_env.to_int()
		print("PicrawlerBody: OGMA_PICRAWLER_TELEPORT_RAMP_AT=%d" % _teleport_ramp_at)
	var evs: String = OS.get_environment("OGMA_PICRAWLER_EVENTS")
	if evs != "":
		var parsed = JSON.parse_string(evs)
		if parsed is Array:
			_events = parsed
			print("PicrawlerBody: OGMA_PICRAWLER_EVENTS — %d scheduled placement(s)" % _events.size())
		else:
			push_error("OGMA_PICRAWLER_EVENTS is not a JSON array: %s" % evs)
	var tev: String = OS.get_environment("OGMA_PICRAWLER_TELEPORT_EVERY")
	if tev != "": _teleport_every = tev.to_int()
	var tun: String = OS.get_environment("OGMA_PICRAWLER_TELEPORT_UNTIL")
	if tun != "": _teleport_until = tun.to_int()
	# Per-leg lesion (the locomotor-(d) test).  LEG unset or -1 leaves the body untouched.
	var lsl: String = OS.get_environment("OGMA_PICRAWLER_LESION_LEG")
	if lsl != "": _lesion_leg = lsl.to_int()
	var lsa: String = OS.get_environment("OGMA_PICRAWLER_LESION_AT")
	if lsa != "": _lesion_at = lsa.to_int()
	var lsu: String = OS.get_environment("OGMA_PICRAWLER_LESION_UNTIL")
	if lsu != "": _lesion_until = lsu.to_int()
	var lss: String = OS.get_environment("OGMA_PICRAWLER_LESION_SCALE")
	if lss != "": _lesion_scale = lss.to_float()
	var lsm: String = OS.get_environment("OGMA_PICRAWLER_LESION_MODE")
	if lsm != "": _lesion_mode = lsm
	var skl: String = OS.get_environment("OGMA_PICRAWLER_SLICK_LEG")
	if skl != "": _slick_leg = skl.to_int()
	var ska: String = OS.get_environment("OGMA_PICRAWLER_SLICK_AT")
	if ska != "": _slick_at = ska.to_int()
	# 2026-08-14 — SCHEDULED BRAIN-PARAM FLIP (generic perturbation hook, the
	# lesion-AT idiom generalized to any HotMutable module param).  Format:
	#   OGMA_PICRAWLER_SETPARAM_AT="tick:module:key:value[;tick:module:key:value...]"
	# Applied ONCE when tick_counter reaches each entry's tick (tick 1 = an
	# arm-differentiation mechanism without config proliferation; a mid-run
	# tick = the (d)-test's perturbation).  Each application is logged.
	var spa: String = OS.get_environment("OGMA_PICRAWLER_SETPARAM_AT")
	if spa != "":
		for ent in spa.split(";", false):
			var parts: PackedStringArray = ent.split(":", false)
			if parts.size() == 4:
				_sched_patches.append({"t": parts[0].to_int(), "id": parts[1],
					"key": parts[2], "val": parts[3].to_float(), "done": false})
			else:
				push_warning("SETPARAM_AT entry malformed (want tick:module:key:value): " + ent)
	_abl_init()
	_abl_parse_env()
	var ccl: String = OS.get_environment("OGMA_PICRAWLER_CHASSIS_COLLIDE")
	if ccl != "":
		chassis_collides = (ccl == "1" or ccl.to_lower() == "true")
		# Loud, and prefixed with the marker gaitreport.py scrapes for env overlays: this
		# one lives in NO config file, so a run summary that omits it describes a body that
		# is physically different from the one that ran.
		print("PicrawlerBody: \u26a0 OGMA_PICRAWLER_CHASSIS_COLLIDE=%s \u2014 chassis %s" % [
			ccl, "COLLIDES with the world" if chassis_collides else "is a ghost (historical)"])
	var cfr: String = OS.get_environment("OGMA_PICRAWLER_CHASSIS_FRICTION")
	if cfr != "":
		chassis_friction = cfr.to_float()
		print("PicrawlerBody: \u26a0 OGMA_PICRAWLER_CHASSIS_FRICTION=%.3f (feet stay at 1.5)"
			% chassis_friction)
	# Optic-nerve budget. A VAR, per the Cell's OGMA_VIS_RES precedent, so Stage 0 can sweep it.
	# Burst-onset probe: auto-saves a clip at each pause->step transition (and a mid-gap
	# control), so the analysis window is the RUN-UP to a step rather than a guessed interval.
	var cwin: String = OS.get_environment("OGMA_PICRAWLER_CLIP_WINDOW")
	if cwin != "":
		var parts := cwin.split(",")
		if parts.size() == 2:
			_cw_from = parts[0].to_int(); _cw_to = parts[1].to_int()
			print("PicrawlerBody: \u26a0 CLIP_WINDOW %d..%d — per-tick dump of that interval" % [_cw_from, _cw_to])
	if OS.get_environment("OGMA_PICRAWLER_BURST_PROBE") == "1":
		_burst_probe = true
		print("PicrawlerBody: \u26a0 BURST_PROBE on — auto-saving ONSET/GAP clips to /tmp/xaq_clips")
	var ifw: String = OS.get_environment("OGMA_PICRAWLER_INTENT_FWD")
	if ifw != "":
		intent_fwd = ifw.to_float()
		print("PicrawlerBody: \u26a0 MOTOR INTENT scaffold — v_fwd*=%.3f yaw*=%.3f" % [intent_fwd, intent_yaw])
	var bv: String = OS.get_environment("OGMA_PICRAWLER_BEACON_VISIBLE")
	if bv != "":
		beacon_visible = (bv == "1" or bv.to_lower() == "true")
		print("PicrawlerBody: \u26a0 OGMA_PICRAWLER_BEACON_VISIBLE=%s \u2014 target pyramid is %s"
			% [bv, "PAINTED (nav has a landmark)" if beacon_visible else "NEUTRAL (no landmark; beacon == 0)"])
	var vw: String = OS.get_environment("OGMA_PICRAWLER_VIS_W")
	if vw != "": vision_res_w = maxi(2, vw.to_int())
	var vh: String = OS.get_environment("OGMA_PICRAWLER_VIS_H")
	if vh != "": vision_res_h = maxi(2, vh.to_int())
	if vw != "" or vh != "":
		print("PicrawlerBody: \u26a0 OGMA_PICRAWLER_VIS_%dx%d (optic-nerve grid; OV5647 optics unchanged)"
			% [vision_res_w, vision_res_h])
	var lfr: String = OS.get_environment("OGMA_PICRAWLER_LIMB_FRICTION")
	if lfr != "":
		limb_friction = lfr.to_float()
		print("PicrawlerBody: \u26a0 OGMA_PICRAWLER_LIMB_FRICTION=%.3f (coxa+upper; feet stay 1.5)"
			% limb_friction)
	if _lesion_leg >= 0 and _lesion_at > 0:
		print("PicrawlerBody: ⚠ LESION armed — leg %d %s x%.2f from tick %d%s" % [
			_lesion_leg, _lesion_mode, _lesion_scale, _lesion_at,
			("" if _lesion_until >= 0x7FFFFFFF else " until tick %d" % _lesion_until)])
	if OS.get_environment("OGMA_PICRAWLER_YAW_PROBE") == "1":
		_yaw_probe_enabled = true
	var fcb_env: String = OS.get_environment("OGMA_PICRAWLER_FORCE_COGNITIVE_BIAS")
	if fcb_env != "":
		_force_cog_bias_enabled = true
		_force_cog_bias_rad = clampf(fcb_env.to_float(), -1.5708, 1.5708)
		_cognitive_bias_rad = _force_cog_bias_rad
		print("PicrawlerBody: FORCE_COGNITIVE_BIAS=%.4f rad — L2 open-loop perception steer (cognitive Premotor write inert)" % _force_cog_bias_rad)
	if OS.get_environment("OGMA_PICRAWLER_CRUSE_TRACE") == "1":
		_cruse_trace_enabled = true
		print("PicrawlerBody: CRUSE_TRACE enabled — periodic [CRUSE] summary every %d ticks (~%.1f sim sec)" % [
			_CRUSE_TRACE_INTERVAL_TICKS, _CRUSE_TRACE_INTERVAL_TICKS * TAU])
		print("PicrawlerBody: YAW_PROBE enabled — Cruse-asymmetric stride bias, schedule=", YAW_PROBE_SCHEDULE)
	_brain_snapshot_save_path = OS.get_environment("OGMA_PICRAWLER_BRAIN_SNAPSHOT_SAVE")
	_brain_snapshot_load_path = OS.get_environment("OGMA_PICRAWLER_BRAIN_SNAPSHOT_LOAD")
	var save_at_s: String = OS.get_environment("OGMA_PICRAWLER_BRAIN_SNAPSHOT_SAVE_AT_SEC")
	if save_at_s != "":
		_brain_snapshot_save_at_sec = save_at_s.to_float()
	if _brain_snapshot_save_path != "":
		var trigger_desc := "first stage advance 0→1"
		if _brain_snapshot_save_at_sec > 0.0:
			trigger_desc = "wall-time t=%.0fs (single-stage curriculum)" % _brain_snapshot_save_at_sec
		print("PicrawlerBody: BRAIN_SNAPSHOT_SAVE=%s (trigger: %s)" % [_brain_snapshot_save_path, trigger_desc])
	if _brain_snapshot_load_path != "":
		print("PicrawlerBody: BRAIN_SNAPSHOT_LOAD=%s (will restore after registrations)" % _brain_snapshot_load_path)

	_build_servo_calibrate_sequence()
	# Per-servo panel state — 12 servos.  Slider centerpoint (target=0)
	# All servos default to origin 0 — slider value IS the target joint
	# angle (after sign).  Knee slider is asymmetric in the UI so the full
	# 160° physical bend range maps onto the negative half (see panel).
	servo_targets.resize(12)
	servo_signs.resize(12)
	servo_origins.resize(12)
	for k in range(12):
		servo_targets[k] = 0.0
		servo_signs[k]   = 1.0
		servo_origins[k] = 0.0
	_resolve_env()

	# Curriculum: load file if env var or ExperimentConfig points to one,
	# then subscribe to stage_changed so future transitions apply their
	# overrides on top of the params we just resolved.  Precedence:
	#   @export → config metadata → env var → curriculum overrides.
	# Curriculum WINS over env on purpose — if a user opts into a
	# curriculum, that's the explicit goal-direction signal.
	var curr_path: String = OS.get_environment("OGMA_PICRAWLER_CURRICULUM")
	if curr_path == "" and ExperimentConfig.has_method("get") and ExperimentConfig.get("picrawler_curriculum_path") != null:
		curr_path = str(ExperimentConfig.get("picrawler_curriculum_path"))
	# CurriculumManager is an autoload singleton — its `stages` array
	# persists across scene reloads.  If a previous session loaded a
	# curriculum and the user now launches without one ("Launch
	# experiment" instead of "Start curriculum"), the prior stages
	# would linger, has_curriculum() would still return true, and
	# CurriculumPanel would show despite no curriculum being active.
	# Clear it explicitly when this run has no curriculum so the UI
	# falls through to the manual reward panel as expected.
	if curr_path == "":
		CurriculumManager.clear()
	if curr_path != "" and CurriculumManager.load_curriculum_file(curr_path):
		var start_env: String = OS.get_environment("OGMA_PICRAWLER_CURRICULUM_START_STAGE")
		if start_env != "":
			CurriculumManager.goto_stage(start_env.to_int())
		# Auto-advance resolution order: launcher-set ExperimentConfig
		# wins (launched flow); otherwise OGMA_PICRAWLER_CURRICULUM_AUTO_ADVANCE
		# env var (headless A/B flow); otherwise default OFF (manual).
		var auto_on: bool = false
		if ExperimentConfig.launched:
			auto_on = ExperimentConfig.picrawler_curriculum_auto_advance
		else:
			var auto_env: String = OS.get_environment("OGMA_PICRAWLER_CURRICULUM_AUTO_ADVANCE")
			auto_on = (auto_env == "1")
		CurriculumManager.set_auto_advance(auto_on)
		print("PicrawlerBody: curriculum loaded — %d stages, auto_advance=%s, path=%s" %
			  [CurriculumManager.n_stages(), str(auto_on), curr_path])
		# Apply the CURRENT stage's overrides immediately, then connect.
		_apply_curriculum_overrides(CurriculumManager.current_overrides())
	CurriculumManager.connect("stage_changed", _on_curriculum_stage_changed)

	# V2: vision-derived steering needs the camera→epm_color stream + the readout.
	# MUST run AFTER the curriculum applies (it sets vision_steer) and BEFORE the
	# register_source block below (publish_vision gates the Camera registration).
	if vision_steer:
		publish_vision = true
		_load_vision_steer_readout()

	# Brain interface contract: declare sources/sinks/events BEFORE setup.
	# Standing task: IMU + joints proprioception only.  chassis_height is
	# the REWARD signal (rate-coded events.hit), NOT a state input —
	# keeping it out of perception forces the brain to learn that lifting
	# the chassis is rewarding from dopamine alone, rather than from
	# goal-leakage in the sensor.  foot_contact is dropped for the same
	# "very simple brain" reason.
	brain.register_source("IMU",     "reality.proprio.imu",
		"float32[4]: sin(yaw), cos(yaw), forward speed, angular speed", true)
	brain.register_source("Joints",  "reality.proprio.joints",
		"float32[12]: 4 hip1 + 4 hip2 + 4 knee, normalised", true)
	# 2026-08-11 (twin-gate M0.d) — PHASE-SPACE proprio: [q, Δq].  Position-only
	# tokens self-intersect on a limit cycle ("knee at 0.3 going up" ≡ "going
	# down"), which is why the planner's chain degenerates to persistence; the
	# per-tick delta breaks the degeneracy with the body's OWN dynamics (no
	# external clock).  Transparent sensor reduction of the same egocentric
	# stream — no subscribers in existing configs, so behaviorally null there.
	brain.register_source("JointsDyn", "reality.proprio.joints_dyn",
		"float32[24]: [q(12), Δq(12)] — the joints stream + its per-tick delta (phase-space embedding)", true)
	if publish_vision:
		# 2026-06-13 — forward-facing raycast camera (Lambert-shaded RGB) → an
		# epm_color JL EPM (host.video.color → reality.video.color → consensus).
		brain.register_source("Camera", "host.video.color",
			"uint8[%d×%d×3]: forward raycast RGB, OV5647 optics 53.5×41.4°, normal-shaded (beacon-coloured surface=purple, ground=gray, object=tan)" % [vision_res_w, vision_res_h], true)
	# World-frame compass: 2-D unit-circle encoding of chassis yaw,
	# anchored to the world's +X axis (cos(yaw)=1, sin(yaw)=0 → facing +X).
	# Redundant with IMU's first 2 channels but published as its own
	# topic so a dedicated epm_compass module can subscribe without
	# competing with epm_imu's projection_dim.  Always published; configs
	# opt in by instantiating an epm_compass that subscribes here.
	brain.register_source("Compass", "reality.proprio.compass",
		"float32[2]: sin(yaw), cos(yaw) — world-frame heading", true)
	brain.register_source("RadialCompass", "reality.proprio.radial_compass",
		"float32[2]: body-frame outward direction (x, z) — (0,0) when r<1cm", true)
	# 2026-06-03 — P2 navigation sensor.  Egocentric unit vector toward
	# active pyramid target in body frame.  (0, 0) when no target active.
	brain.register_source("TargetCompass", "reality.proprio.target_compass",
		"float32[2]: body-frame direction to active pyramid target — (0,0) when no target", true)
	brain.register_source("TargetLoom", "reality.proprio.target_loom",
		"float32[1]: proxy looming signal — fraction of 8x8 forward-FOV raycast hitting the active target pyramid (∈ [0,1]). LGMD analog at low resolution. Rises monotonically as body approaches target; reaches ~1 when target subtends entire FOV.", true)
	# 2026-06-03 — R1a per-leg foot-contact bucket sources (single-float
	# ProprioTokens, 1.0=stance / 0.0=swing per leg).  Optional consumers
	# (Premotor.bucket_context_topic = "reality.proprio.bucket_<leg>").
	brain.register_source("BucketFL", "reality.proprio.bucket_fl",
		"float32[1]: foot-contact bucket for FL leg (1=stance / 0=swing)", true)
	brain.register_source("BucketFR", "reality.proprio.bucket_fr",
		"float32[1]: foot-contact bucket for FR leg (1=stance / 0=swing)", true)
	brain.register_source("BucketRL", "reality.proprio.bucket_rl",
		"float32[1]: foot-contact bucket for RL leg (1=stance / 0=swing)", true)
	brain.register_source("BucketRR", "reality.proprio.bucket_rr",
		"float32[1]: foot-contact bucket for RR leg (1=stance / 0=swing)", true)
	# Phase A3 — per-leg constant ID sources for bucket_context_topic.
	brain.register_source("LegIdFL", "reality.proprio.leg_id_fl",
		"float32[1]: constant 0.0 — FL leg ID for bucket_bias differentiation", true)
	brain.register_source("LegIdFR", "reality.proprio.leg_id_fr",
		"float32[1]: constant 1.0 — FR leg ID for bucket_bias differentiation", true)
	brain.register_source("LegIdRL", "reality.proprio.leg_id_rl",
		"float32[1]: constant 2.0 — RL leg ID for bucket_bias differentiation", true)
	brain.register_source("LegIdRR", "reality.proprio.leg_id_rr",
		"float32[1]: constant 3.0 — RR leg ID for bucket_bias differentiation", true)
	# Phase R4 — per-leg target alignment buckets.
	brain.register_source("TgtAlignFL", "reality.proprio.tgt_align_fl",
		"float32[1]: bucket 0..3 for FL leg target alignment (creative R4)", true)
	brain.register_source("TgtAlignFR", "reality.proprio.tgt_align_fr",
		"float32[1]: bucket 0..3 for FR leg target alignment (creative R4)", true)
	brain.register_source("TgtAlignRL", "reality.proprio.tgt_align_rl",
		"float32[1]: bucket 0..3 for RL leg target alignment (creative R4)", true)
	brain.register_source("TgtAlignRR", "reality.proprio.tgt_align_rr",
		"float32[1]: bucket 0..3 for RR leg target alignment (creative R4)", true)
	brain.register_source("FeetY", "reality.proprio.feet_y",
		"float32[4]: per-leg foot-Y height (FL, FR, RL, RR) — low = planted, high = lifted. " +
		"WARNING: absolute WORLD-Y = god's-eye (the violation that retired chassis_y_norm). " +
		"Markov-compliant twin below.", true)
	brain.register_source("Imu6", "reality.proprio.imu6",
		"float32[6]: body-frame [ax,ay,az] specific force (accelerometer) + [gx,gy,gz] angular " +
		"rate (gyro) — the full 6-axis IMU the real robot has and which the legacy imu topic omits.", true)
	brain.register_source("FeetYGravityCmdAcc", "reality.proprio.feet_y_gravity_cmd_acc",
		"float32[4]: commanded-FK foot height on an ACCELEROMETER-ONLY gravity estimate — shows " +
		"the linear-acceleration contamination a bare accelerometer suffers.", true)
	brain.register_source("FeetYGravityCmdImu", "reality.proprio.feet_y_gravity_cmd_imu",
		"float32[4]: commanded-FK foot height on a GYRO-FUSED gravity estimate (complementary " +
		"filter) — fully hardware-realizable, no exact attitude anywhere.", true)
	brain.register_source("FeetYGravityCmd", "reality.proprio.feet_y_gravity_cmd",
		"float32[4]: feet_y_gravity computed from COMMANDED servo angles instead of achieved " +
		"pose — what a hobby-servo robot can actually compute (no position feedback).", true)
	brain.register_source("FeetYGravityFk", "reality.proprio.feet_y_gravity_fk",
		"float32[4]: feet_y_gravity via analytic FK from MEASURED angles — validation twin; " +
		"should track feet_y_gravity closely, confirming the FK chain is wired correctly.", true)
	brain.register_source("FeetYGravity", "reality.proprio.feet_y_gravity",
		"float32[4]: per-leg foot height below the chassis measured ALONG GRAVITY " +
		"(FL, FR, RL, RR) = encoder-FK foot position · accelerometer gravity-up. IK ⊕ IMU; " +
		"the only legal signal sharing the god's-eye feet_y's gravity reference.", true)
	brain.register_source("FeetYGround", "reality.proprio.feet_y_ground",
		"float32[4]: per-leg foot height above the ground the body stands on = " +
		"feet_y_body (encoder FK) + belly ToF clearance. Markov-compliant AND " +
		"terrain-relative — the legal reconstruction of what god's-eye feet_y provides.", true)
	brain.register_source("FeetYBody", "reality.proprio.feet_y_body",
		"float32[4]: per-leg foot height relative to the CHASSIS (FL, FR, RL, RR) — the " +
		"Markov-compliant twin of feet_y. Derivable from joint encoders + link lengths by " +
		"forward kinematics, so a real picrawler can compute it.", true)
	# 2026-07-22 — Markov-blanket-compliant posture sensors (replace absolute Y).
	brain.register_source("FootContact", "reality.proprio.foot_contact",
		"float32[4]: per-leg TRUE foot-contact (1=touching ground/obstacle, 0=airborne) from the foot's physics contact monitor — a hardware contact switch, NOT an absolute-Y threshold.", true)
	# 2026-08-04 — the HONEST beacon scalar: the fraction of the camera frame occupied by a
	# beacon-COLOURED surface.  A magnitude, never a bearing — direction has to be inferred
	# through action, which is the whole point (plan.md §4 forbids a blob bearing as a first
	# sensor: "nearly instantaneous ≈ an oracle").  Computed at full ray resolution in
	# _capture_vision(), BEFORE the JL encoder's fixed 24×24 resize, and deliberately not
	# routed through an EPM — a GNG would quantise away the very gradient this feeds.
	brain.register_source("EgoHeading", "reality.proprio.ego_heading",
		"float32[1]: dead-reckoned heading, integrated from the modelled body-frame gyro (drifts, as real dead reckoning does)", true)
	brain.register_source("VelEgo", "reality.proprio.vel_ego",
		"float32[2]: [v_right, v_forward] body-frame velocity. ⚠ SOFT ORACLE (world velocity projected) — see sensor_legitimacy doc", true)
	brain.register_source("Beacon", "reality.proprio.beacon",
		"float32[1]: fraction of the frame that is beacon-coloured (looming/LGMD analogue)", true)
	brain.register_source("GroundClearance", "reality.proprio.ground_clearance",
		"float32[1]: downward belly ToF/ultrasonic — normalized [0,1] distance from the belly to the ground beneath (0 = belly ON a surface / high-centered; higher = held up). Egocentric replacement for absolute chassis world-Y.", true)
	brain.register_source("Upright", "reality.proprio.upright",
		"float32[1]: chassis up-vector alignment with gravity (1 = upright, 0 = on its side, -1 = inverted) from the IMU. Gates keyframe baking on posture validity (don't learn from a flipped body).", true)
	# 2026-06-01 Stage 3.A — per-servo torque proprio. Normalized to [-1, 1]
	# against MAX_SERVO_TORQUE. 12-D vector ordered hip1[0..3], hip2[0..3],
	# knee[0..3] matching the existing `Joints` topic convention. Published
	# every tick by default; no consumer wired until Stage 3.A.2 adds a
	# CruseCoordinator load_topic gate. With no subscribers the channel has
	# zero behavioral effect (bit-identical to pre-3.A runs). Per
	# `docs/plans-and-designs/picrawler_diagnostic_calibration_plan.md`
	# Stage 3.A: enables Cruse Walknet Rule 5 (Coactivation — load extends
	# stance), and a future opt-in epm_joint_torque (Stage 3.A.3).
	brain.register_source("JointTorque", "reality.proprio.joint_torque",
		"float32[12]: per-servo PD COMMAND BUDGET (hip1×4, hip2×4, knee×4), normalized to [-1,1] vs MAX_SERVO_TORQUE. ⚠ NOT applied torque and NOT a load proxy despite its original description: it sets the motor's max-impulse cap, and with Kp=20/Kd=8 it is DOMINATED BY THE -Kd*omega damping term (measured corr with joint motion -0.46..-0.56), i.e. it is mostly a negated velocity copy. Use JointLoad for load.", true)
	brain.register_source("FootLoad", "reality.proprio.foot_load",
		"float32[4]: per-leg FOOT NORMAL FORCE (fl,fr,rl,rr), normalised by static body weight — the CoG sensor. Measured as the contact impulse projected on the CONTACT NORMAL, which is exactly what an FSR / load cell in the foot reads: no world-frame or IMU up-vector is required, so this is buildable on the real picrawler as-is. Four load scalars locate the centre of gravity over the support polygon: if fl reads 0.4 the CoG is over fl. THE MISSING OBSERVATION for dynamic walking, where the body must push its CoG past the stable point onto the foot about to plant. Egocentric and physically realisable (foot FSR or servo current sense), so unlike the god's-eye attribution instrument it is a LAWFUL brain input. Distinct from JointLoad, which is a velocity-tracking deficit and was MEASURED not to discriminate stance from swing (0.383 vs 0.257).", true)
	brain.register_source("JointLoad", "reality.proprio.joint_load",
		"float32[12]: per-servo VELOCITY-TRACKING DEFICIT (commanded omega - achieved omega, angle frame) normalized to [-1,1] vs MAX_SERVO_SPEED — the honest load proxy. ~0 when the joint moves freely; large when the motor stalls against its impulse cap because the foot is planted or the limb is loaded. Egocentric, so unlike the god's-eye attribution instrument it is legal as a brain input.", true)
	if publish_tilt:
		# Opt-in 4-D tilt vector for a vestibular EPM that lets the
		# brain PERCEIVE its own pitch/roll (the existing IMU only has
		# yaw + linear/angular speed magnitudes — no pitch/roll, so the
		# brain currently can't see the lean axis that causes its
		# tip-backwards failure mode).  Encoded as
		# [sin(pitch), cos(pitch), sin(roll), cos(roll)] — unit-circle
		# encoding avoids the wrap-around discontinuity at ±π that
		# plain (pitch, roll) would have.  See docs/picrawler_stand_
		# diagnostic.md §11 for the perception-vs-homeostasis rationale.
		brain.register_source("Tilt", "reality.proprio.tilt",
			"float32[4]: sin(pitch), cos(pitch), sin(roll), cos(roll)", true)

	# Event names follow the v4_brain_derivation §4.1 consumer-intent
	# convention: only "hit"/"miss" reach NeurochemState's dopamine path.
	# Earlier body_lift / fell_over names were silently dropped.
	brain.register_event("Hit",        "events.hit",
		"graded reward — duty-cycled to peak 0.2 hits/tick at standing height")
	brain.register_event("Miss",       "events.miss",
		"fired once when chassis tips or sinks below failure threshold")
	brain.register_event("EpisodeEnd", "events.episode_end",
		"trajectory finalise boundary")
	brain.register_event("Reset",      "events.reset",
		"body teleported to the upright rest pose (hard reset / respawn) — Gate 0 reset-masking signal")

	# Register 12 action channels (3 per leg × 4 legs).
	for i in range(4):
		var nm: String = LEG_NAMES[i]
		_idx_hip1.append(brain.register_action_channel(nm + "_hip1", "action." + nm + "_hip1"))
		_idx_hip2.append(brain.register_action_channel(nm + "_hip2", "action." + nm + "_hip2"))
		_idx_knee.append(brain.register_action_channel(nm + "_knee", "action." + nm + "_knee"))

	# Propagate seed into brain master_seed.  Precedence:
	#   ExperimentConfig (launcher) > OGMA_SEED env var > config defaults.
	# Cached into _resolved_seed (class member) so later code (e.g. the
	# walk_over_there stage handler) can derive deterministic sub-RNGs
	# without re-resolving the env-var chain.
	_resolved_seed = ExperimentConfig.resolve_seed()
	if _resolved_seed < 0:
		var env_seed_s: String = OS.get_environment("OGMA_SEED")
		if env_seed_s != "":
			_resolved_seed = env_seed_s.to_int()
	var resolved_seed: int = _resolved_seed   # local alias preserves existing var usage below
	if resolved_seed >= 0:
		brain.set_master_seed(resolved_seed)
	# 2026-06-06 — apply brain snapshot AFTER master_seed (any state RNG
	# in the snapshot supersedes the seeded init).  Loaded JSON must come
	# from a byte-identical GraphConfig — failure to deserialise throws
	# in C++ and is surfaced here as a printed error + abort.
	if _brain_snapshot_load_path != "":
		var f := FileAccess.open(_brain_snapshot_load_path, FileAccess.READ)
		if f == null:
			push_error("PicrawlerBody: cannot open SNAPSHOT_LOAD path " + _brain_snapshot_load_path)
		else:
			var snap_text: String = f.get_as_text()
			f.close()
			print("PicrawlerBody: BRAIN_SNAPSHOT begin restore (%d bytes)" % snap_text.length())
			brain.restore_state(snap_text)
			print("PicrawlerBody: BRAIN_SNAPSHOT restored from %s" % _brain_snapshot_load_path)
			# Settle window so body can re-stabilize under the restored brain
			# before perturbations start.  Only fires when LOAD was used.
			_yaw_probe_settle_offset_ticks = YAW_PROBE_LOAD_SETTLE_TICKS
			print("PicrawlerBody: probe disabled for first %d brain ticks (~%.1f sim sec) post-restore settle" % [
				YAW_PROBE_LOAD_SETTLE_TICKS, YAW_PROBE_LOAD_SETTLE_TICKS * TAU])
	# Curriculum stage-0 snapshot load (headless startup path).  Only when the
	# env-var path didn't already load one; the UI mid-scene load goes through
	# _on_curriculum_stage_changed instead.
	if _brain_snapshot_load_path == "" and CurriculumManager.has_curriculum():
		_load_brain_snapshot_from(str(CurriculumManager.current_stage().get("brain_snapshot_load", "")))
	# Independent RNG for the random_policy ablation arm, seeded
	# deterministically from the master seed so paired-seed comparisons
	# produce identical random action sequences across runs.
	_rand_policy_rng = RandomNumberGenerator.new()
	_rand_policy_rng.seed = (resolved_seed if resolved_seed >= 0 else 42) ^ 0xA17AB1A7
	# Phase 6.7 escape-detector RNG.  Seeded deterministically off the
	# body's resolved seed so paired-seed A/Bs are reproducible.
	_escape_rng.seed = (resolved_seed if resolved_seed >= 0 else 42) ^ 0xE5CAFE07

	# Stage 3.D — 12 per-servo RNG streams for Bernoulli-impulse spike
	# sampling.  Hash-mixed with a per-stream tag so each servo has an
	# independent deterministic stream.  Constructed regardless of
	# actuation_backend setting (cheap to allocate; the discrete backend
	# never draws from them, preserving bit-identical determinism under
	# the default config).
	var bri_seed_base: int = (resolved_seed if resolved_seed >= 0 else 42) ^ 0xB1CE7BE7
	_bri_rng.clear()
	for _j in range(12):
		var r := RandomNumberGenerator.new()
		# Distinct stream per servo: mix the master seed with the joint index
		# using a 32-bit golden-ratio constant (0x9e3779b9).
		r.seed = bri_seed_base ^ (_j * 0x9e3779b9)
		_bri_rng.append(r)

	if not brain.setup(config_path):
		push_error("PicrawlerBody: brain.setup() failed: %s" % config_path)
		return

	# Phase 7.13 — apply env-var overrides for CruseCoordinator HotMutable params
	# AFTER brain.setup() so the JSON config defaults load first.  This lets the
	# launcher tune Cruse rules without editing JSON.  Safe no-op when env-var
	# unset OR when the brain has no cruse_coordinator module.
	_apply_cruse_env_overrides()
	# 2026-06-08 — push the @export var cruse_bias_gain_knee default into Cruse
	# (the @export setter fired pre-_ready() and was no-op'd by is_brain_ready
	# check; brain is initialized now so push the canonical value).
	_apply_cruse_bias_gain_knee_to_brain()
	_apply_cruse_bias_gain_hip1_to_brain()
	_apply_cruse_bias_gain_hip2_to_brain()
	_apply_saturation_params_to_brain()
	# 2026-06-08 — push the @export var cruse_bias_gain default value into the
	# CruseCoordinator now that the brain is initialized.  The @export setter
	# fired before _ready() with brain==null (no-op); this catches the initial
	# assignment.  Future curriculum overrides hit the setter directly.
	_apply_cruse_bias_gain_to_brain()

	# Phase 6.7++ — capture pristine snapshot of every EPM module IMMEDIATELY
	# after brain.setup() returns and before any physics ticks have run.
	# This is the tabula-rasa state the swap mechanism restores TO.  Captured
	# always (cheap) regardless of _epm_swap_enabled, so toggling the env
	# var at runtime works correctly.
	_capture_epm_pristine_snapshots()

	# Phase 6.10 — optional resume from a saved brain state.  Triggered via
	# OGMA_PICRAWLER_RESUME_STATE env var.  Happens AFTER pristine capture
	# so the EPM swap still has a tabula-rasa target if it fires later in
	# the run (the user's saved state is for the brain's policy/perception,
	# not for the swap's fresh snapshot).
	if _resume_state_path != "":
		var loaded: String = ""
		if _resume_state_path == "most_recent":
			loaded = _load_most_recent_brain_state()
		else:
			if _load_brain_state(_resume_state_path):
				loaded = _resume_state_path
		if loaded != "":
			print("PicrawlerBody: resumed from saved brain state %s" % loaded)
		else:
			push_warning("PicrawlerBody: OGMA_PICRAWLER_RESUME_STATE=%s — load failed; cold-starting" % _resume_state_path)

	# Phase 7.x — curriculum-driven bootstrap-preload.  CurriculumManager's
	# initial stage_changed signal fires while load_curriculum_file is
	# executing — BEFORE the body's _ready connects to the signal.  So
	# the stage-0 resume_state_path won't trigger via the signal path
	# (`_on_curriculum_stage_changed`).  Handle it inline here so the
	# initial-stage preload actually happens.  Subsequent stage
	# transitions go through the signal-driven path as expected.
	if CurriculumManager.has_curriculum():
		var initial_stage: Dictionary = CurriculumManager.current_stage()
		var initial_spec: String = str(initial_stage.get("resume_state_path", ""))
		if initial_spec != "" and not _curriculum_resume_done.has(initial_spec):
			var resolved: String = _resolve_resume_state_path(initial_spec)
			if resolved != "" and _load_brain_state(resolved):
				print("PicrawlerBody: initial stage '%s' preloaded brain state ← %s" % [
					CurriculumManager.current_name(), resolved])
				_curriculum_resume_done[initial_spec] = true
			else:
				push_warning("PicrawlerBody: initial stage '%s' resume_state_path='%s' → resolved='%s' — load failed" % [
					CurriculumManager.current_name(), initial_spec, resolved])

	# IMPORT I6 scaffold override — read BEFORE _build_body so it applies at construction.
	var gscale_env: String = OS.get_environment("OGMA_PICRAWLER_GRAVITY_SCALE")
	if gscale_env != "":
		scaffold_gravity_scale = maxf(0.05, float(gscale_env))
	# IMPORT I4 sensor-noise overrides.  Seeded off OGMA_SEED so the noise VARIES with
	# the seed like every other stochastic element — otherwise every seed would share
	# one noise trace and the seed-average would understate the spread.
	var kd_env: String = OS.get_environment("OGMA_PICRAWLER_MOTOR_DAMP")
	if kd_env != "":
		motor_damping_factor = maxf(0.0, float(kd_env))
	var bd_env: String = OS.get_environment("OGMA_PICRAWLER_BODY_DAMP_SCALE")
	if bd_env != "":
		body_damp_scale = maxf(0.0, float(bd_env))
	if not (is_equal_approx(motor_damping_factor, 1.0) and is_equal_approx(body_damp_scale, 1.0)):
		print("PicrawlerBody: ⚠ COMPLIANCE TEST — motor_damp=%.3f body_damp_scale=%.3f (PM runs dampingFactor=0)"
			% [motor_damping_factor, body_damp_scale])
	var snoise_env: String = OS.get_environment("OGMA_PICRAWLER_SENSOR_NOISE")
	if snoise_env != "":
		sensor_noise_sigma = maxf(0.0, float(snoise_env))
	var stau_env: String = OS.get_environment("OGMA_PICRAWLER_SENSOR_NOISE_TAU")
	if stau_env != "":
		sensor_noise_tau = maxf(1.0, float(stau_env))
	var _seed_env: String = OS.get_environment("OGMA_SEED")
	_sensor_noise_rng.seed = (int(_seed_env) if _seed_env != "" else 0) ^ 0x5E4501
	if sensor_noise_sigma > 0.0:
		print("PicrawlerBody: ⚠ IMPORT I4 ACTIVE — colored proprio noise sigma=%.3f tau=%.1f (PM uses ~0.1)"
			% [sensor_noise_sigma, sensor_noise_tau])
	_build_world()
	_build_body()
	_apply_limb_materials()   # coxa+upper get limb_friction; feet stay grippy

	# Phase 7.x — walk_over_there initial target selection.  Same
	# signal-timing issue as resume_state_path: if curriculum starts
	# at a stage whose spec includes target_mode="random_pyramid",
	# _on_curriculum_stage_changed won't have fired by now.  Handle
	# inline here AFTER _build_world has placed the pyramids.
	if CurriculumManager.has_curriculum():
		var init_stage: Dictionary = CurriculumManager.current_stage()
		var init_target_mode: String = str(init_stage.get("target_mode", ""))
		if init_target_mode == "random_pyramid":
			target_mode = "random_pyramid"     # mirror curriculum into body field
			walk_visit_radius = float(init_stage.get("visit_radius", walk_visit_radius))
			_walk_target_rng.seed = (_resolved_seed if _resolved_seed >= 0 else 42) ^ (0xA17D60 + CurriculumManager.current_idx)
			_select_random_pyramid_target()

	# 2026-06-07 — LOUD startup diagnostic so we can debug "no purple pyramid"
	# reports.  Single print with distinctive prefix easy to grep.
	var curr_loaded: bool = CurriculumManager.has_curriculum()
	var curr_name: String = CurriculumManager.current_name() if curr_loaded else "(none)"
	var tgt_mode: String = "n/a"
	if curr_loaded:
		tgt_mode = str(CurriculumManager.current_stage().get("target_mode", "(none)"))
	print("[TARGET-DEBUG] === BODY READY === curriculum=%s  curric_name=%s  stage_target_mode=%s  walk_target_idx=%d  n_pyramids=%d  env_OGMA_PICRAWLER_CURRICULUM=%s  ExperimentConfig.picrawler_curriculum_path=%s" % [
		str(curr_loaded), curr_name, tgt_mode, walk_target_idx, _pyramid_meshes.size(),
		OS.get_environment("OGMA_PICRAWLER_CURRICULUM"),
		str(ExperimentConfig.get("picrawler_curriculum_path")) if ExperimentConfig.has_method("get") else "(no_get)"])

	_retarget_body_watchers()

	print("PicrawlerBody: built — chassis at y=%.3f, leg_strength=%.2f, reset_mode=%s" % [
		STANDING_CHASSIS_Y, leg_strength, reset_mode])
	print("  _chassis_rest_xform.origin = %v" % _chassis_rest_xform.origin)
	for i in range(4):
		print("  leg %d %s: coxa=%v upper=%v lower=%v" % [
			i, LEG_NAMES[i],
			_coxa_rest_xform[i].origin,
			_upper_rest_xform[i].origin,
			_lower_rest_xform[i].origin])

	# If calibration was enabled via env var (before body existed), apply
	# the chassis-suspended state now that _chassis is built.
	if _calibrate_mode:
		_chassis.global_transform = Transform3D(Basis.IDENTITY, Vector3(0, 0.25, 0))
		_chassis.linear_velocity = Vector3.ZERO
		_chassis.angular_velocity = Vector3.ZERO
		_chassis.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
		_chassis.freeze = true
		print("PicrawlerBody: chassis suspended at y=0.25 for AUTO calibration")

func _resolve_env() -> void:
	# ExperimentConfig (autoload, set by launcher) takes precedence over
	# OGMA_PICRAWLER_* env vars.  This lets the picrawler integrate with
	# the launcher dropdown the same way quadruped does.
	for k in ["OGMA_PHYSICS_HZ", "OGMA_SOLVER_ITERATIONS",
			  "OGMA_PICRAWLER_MAX_STEPS", "OGMA_PICRAWLER_MAX_EPISODES",
			  "OGMA_PICRAWLER_DIAG_INTERVAL", "OGMA_PICRAWLER_MC_PERIOD",
			  "OGMA_LEG_STRENGTH", "OGMA_RESET_MODE", "OGMA_PICRAWLER_CONFIG",
			  "OGMA_PICRAWLER_SERVO_KI",
			  "OGMA_PICRAWLER_STAB_Y_NORM", "OGMA_PICRAWLER_STAB_SPEED",
			  "OGMA_PICRAWLER_STAB_GAIN",
			  "OGMA_PICRAWLER_ANTIROT_THRESHOLD", "OGMA_PICRAWLER_ANTIROT_SCALE",
			  "OGMA_PICRAWLER_ANTIROT_GAIN", "OGMA_PICRAWLER_PUBLISH_TILT",
			  "OGMA_PICRAWLER_PUBLISH_VISION", "OGMA_PICRAWLER_VISION_STABILIZED",
			  "OGMA_PICRAWLER_VISION_STEER", "OGMA_PICRAWLER_VERBOSE_LOG",
			  "OGMA_PICRAWLER_ENERGY_DEADBAND", "OGMA_PICRAWLER_ENERGY_SCALE",
			  "OGMA_PICRAWLER_ENERGY_GAIN",
			  "OGMA_PICRAWLER_BRAIN_OFF", "OGMA_PICRAWLER_RANDOM_POLICY",
			  "OGMA_PICRAWLER_SCRIPTED_GAIT", "OGMA_PICRAWLER_SG_AMP_HIP1",
			  "OGMA_PICRAWLER_SG_AMP_HIP2", "OGMA_PICRAWLER_SG_AMP_KNEE",
			  "OGMA_PICRAWLER_SG_PERIOD", "OGMA_PICRAWLER_SG_PATTERN",
			  "OGMA_PICRAWLER_SG_HIP1_PHASE",
			  "OGMA_PICRAWLER_TARGET_HEIGHT", "OGMA_PICRAWLER_HEIGHT_PENALTY_GRACE",
			  "OGMA_PICRAWLER_HEIGHT_PENALTY_SCALE", "OGMA_PICRAWLER_HEIGHT_PENALTY_GAIN",
			  "OGMA_PICRAWLER_TILT_TARGET_RAD",
			  "OGMA_PICRAWLER_REWARD_SHAPE", "OGMA_PICRAWLER_PEAK_HEIGHT",
			  "OGMA_PICRAWLER_BAND_WIDTH",
			  "OGMA_PICRAWLER_WALK_TARGET_VELOCITY", "OGMA_PICRAWLER_WALK_HIT_RATE",
			  "OGMA_PICRAWLER_STANDING_BASELINE_FACTOR",
			  "OGMA_PICRAWLER_GATED_WALK_BONUS_RATE",
			  "OGMA_PICRAWLER_PER_LEG_CREDIT_GAIN",
			  "OGMA_PICRAWLER_STANCE_Y_THRESHOLD",
			  "OGMA_PICRAWLER_PHASE_CONTRAST_GAIN",
			  "OGMA_PICRAWLER_PROGRESS_REWARD_GAIN",
			  "OGMA_PICRAWLER_PROGRESS_REWARD_MIN_DELTA",
			  "OGMA_PICRAWLER_FAIL_TILT_RAD",
			  "OGMA_PICRAWLER_LEVEL_CHASSIS_RATE",
			  "OGMA_PICRAWLER_LEVEL_CHASSIS_TILT_SCALE",
			  "OGMA_PICRAWLER_LEVEL_CHASSIS_WALKING_BUDGET",
			  "OGMA_PICRAWLER_AUTO_RESET_INVERSION",
			  "OGMA_PICRAWLER_AUTO_RESET_OUTER_WALL",
			  "OGMA_PICRAWLER_AUTO_RESET_TILT_THRESHOLD",
			  "OGMA_PICRAWLER_AUTO_RESET_MAX_HEIGHT",
			  "OGMA_PICRAWLER_AUTO_RESET_DWELL_TICKS",
			  "OGMA_PICRAWLER_LEG_SYMMETRY", "OGMA_PICRAWLER_LEG_SYMMETRY_FR_BLEND",
			  "OGMA_PICRAWLER_HOMEO_STEP_GAIN",
			  "OGMA_PICRAWLER_HOMEO_PAYOUT_NORM",
			  "OGMA_PICRAWLER_HOMEO_PHASE_COUPLE",
			  "OGMA_PICRAWLER_ALIVE_HEARTBEAT",
			  "OGMA_PICRAWLER_ESCAPE_DETECTOR",
			  "OGMA_PICRAWLER_ESCAPE_FORCE_FIRE",
			  "OGMA_PICRAWLER_EPM_SWAP_ON_HK",
			  "OGMA_PICRAWLER_WALK_REWARD_MODE",
			  "OGMA_PICRAWLER_RESUME_STATE",
			  "OGMA_PICRAWLER_PYRAMID_MIN_R",
			  "OGMA_PICRAWLER_PYRAMID_MAX_R",
			  "OGMA_PICRAWLER_PYRAMID_COUNT",
			  "OGMA_PICRAWLER_JOINT_BACKEND",
			  "OGMA_PICRAWLER_JOINT_DAMPING",
			  "OGMA_PICRAWLER_MOTOR_FREEPLAY"]:
		var v: String = OS.get_environment(k)
		if v == "": continue
		match k:
			"OGMA_PHYSICS_HZ":               physics_hz = max(60, v.to_int())
			"OGMA_SOLVER_ITERATIONS":        solver_iterations = max(8, v.to_int())
			"OGMA_PICRAWLER_MAX_STEPS":      max_steps = max(1, v.to_int())
			"OGMA_PICRAWLER_MAX_EPISODES":   max_episodes = max(0, v.to_int())
			"OGMA_PICRAWLER_DIAG_INTERVAL":  diag_interval_ticks = max(0, v.to_int())
			"OGMA_PICRAWLER_MC_PERIOD":      mc_episode_period = max(0, v.to_int())
			"OGMA_LEG_STRENGTH":             leg_strength = max(0.0, v.to_float())
			"OGMA_PICRAWLER_SERVO_KI":       servo_ki = max(0.0, v.to_float())
			"OGMA_RESET_MODE":               if v in _RESET_MODE_CHOICES: reset_mode = v
			"OGMA_PICRAWLER_CONFIG":         config_path = v
			"OGMA_PICRAWLER_STAB_Y_NORM":    stability_y_norm = clamp(v.to_float(), 0.0, 1.0)
			"OGMA_PICRAWLER_STAB_SPEED":     stability_speed  = max(0.0, v.to_float())
			"OGMA_PICRAWLER_STAB_GAIN":      stability_gain   = max(0.0, v.to_float())
			"OGMA_PICRAWLER_ANTIROT_THRESHOLD": antirot_threshold = max(0.0, v.to_float())
			"OGMA_PICRAWLER_ANTIROT_SCALE":     antirot_scale     = max(0.001, v.to_float())
			"OGMA_PICRAWLER_ANTIROT_GAIN":      antirot_gain      = max(0.0, v.to_float())
			"OGMA_PICRAWLER_PUBLISH_TILT":      publish_tilt      = (v != "0" and v != "")
			"OGMA_PICRAWLER_PUBLISH_VISION":    publish_vision    = (v != "0" and v != "")
			"OGMA_PICRAWLER_VISION_STABILIZED": vision_stabilized = (v != "0" and v != "")
			"OGMA_PICRAWLER_VISION_STEER":      vision_steer      = (v != "0" and v != "")
			"OGMA_PICRAWLER_VERBOSE_LOG":       verbose_logging   = (v != "0" and v != "")
			"OGMA_PICRAWLER_ENERGY_DEADBAND":   energy_deadband   = max(0.0, v.to_float())
			"OGMA_PICRAWLER_ENERGY_SCALE":      energy_scale      = max(0.001, v.to_float())
			"OGMA_PICRAWLER_ENERGY_GAIN":       energy_gain       = max(0.0, v.to_float())
			"OGMA_PICRAWLER_BRAIN_OFF":         brain_off         = (v != "0" and v != "")
			"OGMA_PICRAWLER_RANDOM_POLICY":     random_policy     = (v != "0" and v != "")
			"OGMA_PICRAWLER_SCRIPTED_GAIT":     _scripted_gait    = (v != "0" and v != "")
			"OGMA_PICRAWLER_SG_AMP_HIP1":       _sg_amp_hip1      = v.to_float()
			"OGMA_PICRAWLER_SG_AMP_HIP2":       _sg_amp_hip2      = v.to_float()
			"OGMA_PICRAWLER_SG_AMP_KNEE":       _sg_amp_knee      = v.to_float()
			"OGMA_PICRAWLER_SG_PERIOD":         _sg_period        = max(1.0, v.to_float())
			"OGMA_PICRAWLER_SG_HIP1_PHASE":     _sg_hip1_phase    = v.to_float()
			"OGMA_PICRAWLER_SG_PATTERN":        _sg_offsets       = _sg_pattern_offsets(v)
			"OGMA_PICRAWLER_TARGET_HEIGHT":         target_height        = max(0.001, v.to_float())
			"OGMA_PICRAWLER_HEIGHT_PENALTY_GRACE":  height_penalty_grace = max(0.0,   v.to_float())
			"OGMA_PICRAWLER_HEIGHT_PENALTY_SCALE":  height_penalty_scale = max(0.001, v.to_float())
			"OGMA_PICRAWLER_HEIGHT_PENALTY_GAIN":   height_penalty_gain  = max(0.0,   v.to_float())
			"OGMA_PICRAWLER_TILT_TARGET_RAD":       tilt_target_rad      = max(0.0,   v.to_float())
			"OGMA_PICRAWLER_REWARD_SHAPE":          if v in ["trapezoid", "inverted_u"]: reward_shape = v
			"OGMA_PICRAWLER_PEAK_HEIGHT":           peak_height          = max(0.001, v.to_float())
			"OGMA_PICRAWLER_BAND_WIDTH":            band_width           = max(0.001, v.to_float())
			"OGMA_PICRAWLER_REWARD_MIN_HEIGHT":     reward_min_height    = max(0.0,   v.to_float())
			"OGMA_PICRAWLER_PEAK_HEIGHT_MODE":      if v in ["fixed", "knee_relative"]: peak_height_mode = v
			"OGMA_PICRAWLER_HEIGHT_REFERENCE":      if v in ["chassis", "body_cog"]: height_reference = v
			"OGMA_PICRAWLER_WALK_TARGET_VELOCITY":  walk_target_velocity = max(0.001, v.to_float())
			"OGMA_PICRAWLER_WALK_HIT_RATE":         walk_hit_rate        = max(0.0,   v.to_float())
			"OGMA_PICRAWLER_STANDING_BASELINE_FACTOR": standing_baseline_factor = max(0.0, v.to_float())
			"OGMA_PICRAWLER_GATED_WALK_BONUS_RATE":    gated_walk_bonus_rate    = max(0.0, v.to_float())
			"OGMA_PICRAWLER_PER_LEG_CREDIT_GAIN":      per_leg_credit_gain      = max(0.0, v.to_float())
			"OGMA_PICRAWLER_STANCE_Y_THRESHOLD":       stance_y_threshold       = max(0.001, v.to_float())
			"OGMA_PICRAWLER_STEP_QUALITY_REWARD_GAIN": step_quality_reward_gain = max(0.0, v.to_float())
			"OGMA_PICRAWLER_STEP_LIFT_SCALE":          step_lift_scale          = max(0.001, v.to_float())
			"OGMA_PICRAWLER_STEP_QUALITY_EMA_ALPHA":   step_quality_ema_alpha   = clamp(v.to_float(), 0.0001, 1.0)
			"OGMA_PICRAWLER_HIP1_SPRING_STIFFNESS":    hip1_spring_stiffness    = max(0.0, v.to_float())
			"OGMA_PICRAWLER_HIP1_SPRING_DAMPING":      hip1_spring_damping      = max(0.0, v.to_float())
			"OGMA_PICRAWLER_HIP2_SPRING_STIFFNESS":    hip2_spring_stiffness    = max(0.0, v.to_float())
			"OGMA_PICRAWLER_HIP2_SPRING_DAMPING":      hip2_spring_damping      = max(0.0, v.to_float())
			"OGMA_PICRAWLER_KNEE_SPRING_STIFFNESS":    knee_spring_stiffness    = max(0.0, v.to_float())
			"OGMA_PICRAWLER_KNEE_SPRING_DAMPING":      knee_spring_damping      = max(0.0, v.to_float())
			"OGMA_PICRAWLER_JOINT_BACKEND":
				if v in ["hinge", "g6dof"]:
					joint_backend = v
				else:
					push_warning("PicrawlerBody: ignoring OGMA_PICRAWLER_JOINT_BACKEND=%s (expected hinge/g6dof)" % v)
			# 2026-08-03 — joint_angular_damping had NO env override, so it could never be
			# swept headlessly.  The operator found it is the largest behavioural lever on
			# the g6dof substrate (1.5 -> 0.5 makes the robot move "much, much faster"),
			# and it is g6dof-ONLY: it reaches the solver solely via
			# Generic6DOFJoint3D.PARAM_ANGULAR_DAMPING, so hinge runs never saw it.  That
			# is why UI observation and headless measurement disagreed all session.
			"OGMA_PICRAWLER_JOINT_DAMPING":            joint_angular_damping    = max(0.0, v.to_float())
			"OGMA_PICRAWLER_MOTOR_FREEPLAY":           motor_freeplay_rad       = max(0.0, v.to_float())
			"OGMA_PICRAWLER_SPRING_FOLLOWS_TARGET":    spring_follows_target    = (v == "1")
			"OGMA_PICRAWLER_PHASE_CONTRAST_GAIN":      phase_contrast_gain      = clamp(v.to_float(), 0.0, 1.0)
			"OGMA_PICRAWLER_PROGRESS_REWARD_GAIN":      progress_reward_gain      = max(0.0, v.to_float())
			"OGMA_PICRAWLER_PROGRESS_REWARD_MIN_DELTA": progress_reward_min_delta = max(0.0001, v.to_float())
			"OGMA_PICRAWLER_FAIL_TILT_RAD":             fail_tilt_rad             = clamp(v.to_float(), 0.5, PI)
			"OGMA_PICRAWLER_LEVEL_CHASSIS_RATE":           level_chassis_rate           = max(0.0, v.to_float())
			"OGMA_PICRAWLER_LEVEL_CHASSIS_TILT_SCALE":     level_chassis_tilt_scale     = clamp(v.to_float(), 0.1, PI)
			"OGMA_PICRAWLER_LEVEL_CHASSIS_WALKING_BUDGET": level_chassis_walking_budget = clamp(v.to_float(), 0.0, PI)
			"OGMA_PICRAWLER_AUTO_RESET_INVERSION":       auto_reset_on_inversion = (v != "0" and v != "")
			"OGMA_PICRAWLER_AUTO_RESET_OUTER_WALL":      auto_reset_on_outer_wall = (v != "0" and v != "")
			"OGMA_PICRAWLER_AUTO_RESET_TILT_THRESHOLD":  auto_reset_tilt_threshold = max(PI/2, v.to_float())
			"OGMA_PICRAWLER_AUTO_RESET_MAX_HEIGHT":      auto_reset_max_height     = max(0.0,  v.to_float())
			"OGMA_PICRAWLER_AUTO_RESET_DWELL_TICKS":     auto_reset_dwell_ticks    = max(1,    v.to_int())
			"OGMA_PICRAWLER_PYRAMID_MIN_R":              pyramid_min_r             = max(0.5,  v.to_float())
			"OGMA_PICRAWLER_PYRAMID_MAX_R":              pyramid_max_r             = max(0.6,  v.to_float())
			"OGMA_PICRAWLER_PYRAMID_COUNT":             pyramid_count             = max(1,    v.to_int())
			"OGMA_PICRAWLER_LEG_SYMMETRY":
				if v in ["off", "lr_pairs", "lr_and_fr_pairs"]:
					leg_symmetry_mode = v
				else:
					push_warning("PicrawlerBody: ignoring OGMA_PICRAWLER_LEG_SYMMETRY=%s (expected off/lr_pairs/lr_and_fr_pairs)" % v)
			"OGMA_PICRAWLER_LEG_SYMMETRY_FR_BLEND":
				leg_symmetry_fr_blend = clamp(v.to_float(), 0.0, 1.0)
			"OGMA_PICRAWLER_HOMEO_STEP_GAIN":
				homeo_step_gain = max(0.0, v.to_float())
			"OGMA_PICRAWLER_HOMEO_PAYOUT_NORM":
				homeo_payout_norm = clampf(v.to_float(), 0.0, 1.0)
			"OGMA_PICRAWLER_HOMEO_PHASE_COUPLE":
				homeo_phase_couple = clampf(v.to_float(), 0.0, 1.0)
			"OGMA_PICRAWLER_ALIVE_HEARTBEAT":  _publish_alive_heartbeat = (v != "0" and v != "")
			"OGMA_PICRAWLER_ESCAPE_DETECTOR":  _escape_detector_enabled = (v != "0" and v != "")
			"OGMA_PICRAWLER_ESCAPE_FORCE_FIRE": _escape_force_fire     = (v != "0" and v != "")
			"OGMA_PICRAWLER_EPM_SWAP_ON_HK":   _epm_swap_enabled      = (v != "0" and v != "")
			"OGMA_PICRAWLER_WALK_REWARD_MODE":
				if v in ["radial", "radial_penalize_inward", "total_speed"]:
					walk_reward_mode = v
				else:
					push_warning("PicrawlerBody: ignoring OGMA_PICRAWLER_WALK_REWARD_MODE=%s (expected radial/radial_penalize_inward/total_speed)" % v)
			"OGMA_PICRAWLER_RESUME_STATE": _resume_state_path = v

	# Auto-start calibration mode (debug / headless verification).
	# OGMA_PICRAWLER_AUTO_CALIBRATE=1  → enable, start at step 0
	# OGMA_PICRAWLER_AUTO_CALIBRATE=N  → enable, start at step N (lets us
	#                                   headlessly inspect a specific pose)
	var env_calib: String = OS.get_environment("OGMA_PICRAWLER_AUTO_CALIBRATE")
	if env_calib != "" and env_calib != "0":
		_calibrate_mode = true
		_calibrate_step = max(0, env_calib.to_int())
		_calibrate_step_start_tick = 0
		print("PicrawlerBody: AUTO calibrate_mode enabled at step %d" % _calibrate_step)

	# 2026-06-03 — Resolve joint_backend EARLY so we can apply backend-
	# conditional defaults before the rest of the resolver chain.  Picking
	# g6dof at the launcher overwrites the @export defaults with Joseph's
	# handtuned compliant-stand preset; env/config/curriculum overrides
	# still win because they run AFTER this on the resolved baseline.
	# 2026-08-03 — CANONICAL SUBSTRATE = "hinge" (operator decision).  Rationale: hinge
	# beats g6dof on every measured metric (PLV 0.138 vs 0.097 at its best damping,
	# net_z 4.49 vs 2.79, straight 0.70 vs 0.48, 0 falls vs 0.17); it is the closer model
	# of a real hobby servo (a stiff position tracker on a rigid gear train, whose
	# compliance is backlash — not a series-elastic suspension); and every result in the
	# ledger was measured on it, so nothing needs re-running.
	#
	# The split this guards against: the launcher's PERSISTED selection silently won over
	# the default, so UI sessions ran g6dof while every headless run ran hinge — the same
	# config producing two different robots depending on how it was started, which is how
	# operator observation and headless measurement disagreed for a whole session.
	# The resolver order is unchanged (explicit env / launcher choice still wins, so
	# g6dof stays one flag away); what is new is that the substrate is ANNOUNCED, loudly,
	# whenever it differs from canonical.
	var _canonical_backend: String = "hinge"
	joint_backend = ExperimentConfig.resolve_picrawler_joint_backend(joint_backend)
	if joint_backend != _canonical_backend:
		push_warning("PicrawlerBody: NON-CANONICAL SUBSTRATE — joint_backend=%s (canonical is %s). "
			% [joint_backend, _canonical_backend]
			+ "Ledger results are all hinge; numbers from this run are NOT comparable to them.")
		print("PicrawlerBody: ⚠⚠ NON-CANONICAL SUBSTRATE joint_backend=%s (canonical=%s) — "
			% [joint_backend, _canonical_backend]
			+ "results NOT comparable to the ledger")
	else:
		print("PicrawlerBody: substrate = hinge (canonical)")
	# 2026-06-13 — the CURRICULUM is authoritative for the substrate, overriding
	# the launcher's persisted dropdown.  The launcher saves the last selection,
	# so a stale hinge memory would silently run the wrong substrate; a stage-level
	# joint_backend in the curriculum makes "load this curriculum" reproduce the
	# validated substrate freshly, regardless of launcher state.  Same stage-level
	# pattern as target_mode/visit_radius (read direct from the stage dict, not the
	# @export whitelist — joint_backend is construction-time, can't be a live set()).
	if CurriculumManager.n_stages() > 0:
		var curr_backend: String = str(CurriculumManager.current_stage().get("joint_backend", ""))
		if curr_backend in ["hinge", "g6dof"]:
			if curr_backend != joint_backend:
				print("PicrawlerBody: curriculum overrides joint_backend %s → %s (authoritative over launcher)" % [joint_backend, curr_backend])
			joint_backend = curr_backend
	if joint_backend == "g6dof":
		_apply_g6dof_default_preset()

	# Launcher → ExperimentConfig overrides env vars.
	config_path = ExperimentConfig.resolve_config("OGMA_PICRAWLER_CONFIG", config_path)
	var ec_reset: String = ExperimentConfig.resolve_reset_mode(reset_mode)
	if ec_reset in _RESET_MODE_CHOICES:
		reset_mode = ec_reset
	leg_strength = ExperimentConfig.resolve_leg_strength(leg_strength)
	# PiCrawler standing-task knobs — launcher metadata overrides env vars.
	mc_episode_period = ExperimentConfig.resolve_picrawler_mc_period(mc_episode_period)
	stability_gain    = ExperimentConfig.resolve_picrawler_stab_gain(stability_gain)
	stability_y_norm  = ExperimentConfig.resolve_picrawler_stab_y_norm(stability_y_norm)
	stability_speed   = ExperimentConfig.resolve_picrawler_stab_speed(stability_speed)
	antirot_threshold = ExperimentConfig.resolve_picrawler_antirot_threshold(antirot_threshold)
	antirot_scale     = ExperimentConfig.resolve_picrawler_antirot_scale(antirot_scale)
	antirot_gain      = ExperimentConfig.resolve_picrawler_antirot_gain(antirot_gain)
	publish_tilt      = ExperimentConfig.resolve_picrawler_publish_tilt(publish_tilt)
	verbose_logging   = ExperimentConfig.resolve_picrawler_verbose_log(verbose_logging)
	energy_deadband   = ExperimentConfig.resolve_picrawler_energy_deadband(energy_deadband)
	energy_scale      = ExperimentConfig.resolve_picrawler_energy_scale(energy_scale)
	energy_gain       = ExperimentConfig.resolve_picrawler_energy_gain(energy_gain)
	brain_off         = ExperimentConfig.resolve_picrawler_brain_off(brain_off)
	random_policy     = ExperimentConfig.resolve_picrawler_random_policy(random_policy)
	target_height        = ExperimentConfig.resolve_picrawler_target_height(target_height)
	height_penalty_grace = ExperimentConfig.resolve_picrawler_height_penalty_grace(height_penalty_grace)
	height_penalty_scale = ExperimentConfig.resolve_picrawler_height_penalty_scale(height_penalty_scale)
	height_penalty_gain  = ExperimentConfig.resolve_picrawler_height_penalty_gain(height_penalty_gain)
	reward_shape         = ExperimentConfig.resolve_picrawler_reward_shape(reward_shape)
	peak_height          = ExperimentConfig.resolve_picrawler_peak_height(peak_height)
	band_width           = ExperimentConfig.resolve_picrawler_band_width(band_width)
	walk_target_velocity = ExperimentConfig.resolve_picrawler_walk_target_velocity(walk_target_velocity)
	walk_hit_rate        = ExperimentConfig.resolve_picrawler_walk_hit_rate(walk_hit_rate)
	standing_baseline_factor = ExperimentConfig.resolve_picrawler_standing_baseline_factor(standing_baseline_factor)
	gated_walk_bonus_rate    = ExperimentConfig.resolve_picrawler_gated_walk_bonus_rate(gated_walk_bonus_rate)
	per_leg_credit_gain      = ExperimentConfig.resolve_picrawler_per_leg_credit_gain(per_leg_credit_gain)
	stance_y_threshold       = ExperimentConfig.resolve_picrawler_stance_y_threshold(stance_y_threshold)
	step_quality_reward_gain = ExperimentConfig.resolve_picrawler_step_quality_reward_gain(step_quality_reward_gain)
	step_lift_scale          = ExperimentConfig.resolve_picrawler_step_lift_scale(step_lift_scale)
	step_quality_ema_alpha   = ExperimentConfig.resolve_picrawler_step_quality_ema_alpha(step_quality_ema_alpha)
	hip1_spring_stiffness    = ExperimentConfig.resolve_picrawler_hip1_spring_stiffness(hip1_spring_stiffness)
	hip1_spring_damping      = ExperimentConfig.resolve_picrawler_hip1_spring_damping(hip1_spring_damping)
	hip2_spring_stiffness    = ExperimentConfig.resolve_picrawler_hip2_spring_stiffness(hip2_spring_stiffness)
	hip2_spring_damping      = ExperimentConfig.resolve_picrawler_hip2_spring_damping(hip2_spring_damping)
	knee_spring_stiffness    = ExperimentConfig.resolve_picrawler_knee_spring_stiffness(knee_spring_stiffness)
	knee_spring_damping      = ExperimentConfig.resolve_picrawler_knee_spring_damping(knee_spring_damping)
	# (joint_backend resolved earlier; see top of _resolve_env.)
	phase_contrast_gain      = ExperimentConfig.resolve_picrawler_phase_contrast_gain(phase_contrast_gain)
	progress_reward_gain      = ExperimentConfig.resolve_picrawler_progress_reward_gain(progress_reward_gain)
	progress_reward_min_delta = ExperimentConfig.resolve_picrawler_progress_reward_min_delta(progress_reward_min_delta)
	level_chassis_rate           = ExperimentConfig.resolve_picrawler_level_chassis_rate(level_chassis_rate)
	level_chassis_tilt_scale     = ExperimentConfig.resolve_picrawler_level_chassis_tilt_scale(level_chassis_tilt_scale)
	level_chassis_walking_budget = ExperimentConfig.resolve_picrawler_level_chassis_walking_budget(level_chassis_walking_budget)
	leg_symmetry_mode    = ExperimentConfig.resolve_picrawler_leg_symmetry(leg_symmetry_mode)
	if leg_strength <= 0.0:
		leg_strength = 0.01
	if ExperimentConfig.launched:
		if ExperimentConfig.max_episodes > 0:
			max_episodes = ExperimentConfig.max_episodes
		# Launcher's "Max steps/episode" spinbox semantics: any explicit
		# value (including 0) is the user's intent.  0 = "run forever"
		# per the spinbox tooltip ("(0 = continuous, goal-only reset)").
		# Set max_steps unconditionally; the per-tick guard checks
		# `max_steps > 0` so 0 disables the cap.
		max_steps = ExperimentConfig.max_steps_per_episode

# ---------------------------------------------------------------------------
# World — minimal flat floor
# ---------------------------------------------------------------------------
# World geometry lives under a WorldRoot container so the gym can be swapped
# live (KEY_1 = arena / KEY_2 = corridor) — freeing WorldRoot and rebuilding
# drops the SAME robot + brain (an experienced agent) into a new scenario
# WITHOUT a scene reload (the brain stays in memory, fully continuous).
var _world_root: Node3D = null
var _gym_mode_active: String = ""
var _gym_mode_override: String = ""   # set by the KEY_1/KEY_2 hotkeys; wins over config/env

func _build_world() -> void:
	_world_root = Node3D.new()
	_world_root.name = "WorldRoot"
	add_child(_world_root)
	_rebuild_world_contents()

func _rebuild_world_contents() -> void:
	var floor_body := StaticBody3D.new()
	floor_body.collision_layer = _LAYER_WORLD
	floor_body.collision_mask  = _world_collision_mask()
	floor_body.physics_material_override = _make_contact_mat()
	var fcs := CollisionShape3D.new()
	var fb := BoxShape3D.new()
	fb.size = Vector3(20.0, 0.1, 20.0)
	fcs.shape = fb
	fcs.position = Vector3(0, -0.05, 0)
	floor_body.add_child(fcs)
	var fmesh := MeshInstance3D.new()
	var fbm := BoxMesh.new()
	fbm.size = fb.size
	fmesh.mesh = fbm
	fmesh.position = Vector3(0, -0.05, 0)
	var fmat := StandardMaterial3D.new()
	fmat.albedo_color = Color(0.3, 0.35, 0.3, 1.0)
	fmesh.set_surface_override_material(0, fmat)
	floor_body.add_child(fmesh)
	_world_root.add_child(floor_body)

	# Gym mode select.  The live override (KEY_1/KEY_2 hotkeys) wins; else the
	# ExperimentConfig resolution: launcher metadata.gym_mode > OGMA_PICRAWLER_GYM
	# env > "" default.  Empty / "arena" / "donut" = the legacy donut arena below.
	# "corridor" = the +Z trench curriculum (flat runway -> hump -> rumble ->
	# pyramids) inside self-centering 30 deg walls.  See _build_corridor().
	var mode: String = _gym_mode_override if _gym_mode_override != "" \
		else ExperimentConfig.resolve_picrawler_gym_mode("").to_lower()
	_gym_mode_active = mode
	if mode == "corridor":
		_build_corridor()
		return

	# Concentric reference rings on the floor.  Visual-only (no collision)
	# to make body motion legible — distance from origin at a glance.
	# Radii chosen to span body-scale (0.1-0.3 m chassis ≈ 0.2 m) up to
	# walking-experiment range (≈ 3 m).  Slightly tighter spacing near
	# the body so we can see fine-grained position drift during the
	# current standing experiment.
	_build_floor_rings()

	# Terrain — 45° outward-sloping ramps at floor edges (containment +
	# slide-down challenge for explorers) and random low pyramids in the
	# outer donut (terrain to walk around / over).  Both use a fixed
	# terrain seed (0) so all body seeds see the SAME layout — preserves
	# paired-seed A/B comparability.
	_build_terrain()

func _build_floor_rings() -> void:
	var ring_radii: PackedFloat32Array = PackedFloat32Array(
		[0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0])
	var ring_color: Color = Color(0.78, 0.74, 0.55, 1.0)   # warm contrast vs floor
	var tube_r: float = 0.003                              # 3 mm thick rings
	var ring_y: float = tube_r + 0.0005                    # just above floor; +0.5 mm margin to avoid z-fight
	var ring_mat := StandardMaterial3D.new()
	ring_mat.albedo_color = ring_color
	ring_mat.metallic = 0.1
	ring_mat.roughness = 0.6
	for r in ring_radii:
		var torus := TorusMesh.new()
		torus.inner_radius = r - tube_r
		torus.outer_radius = r + tube_r
		torus.rings = 64                                   # angular subdivisions
		torus.ring_segments = 8                            # tube subdivisions
		var m := MeshInstance3D.new()
		m.mesh = torus
		m.position = Vector3(0, ring_y, 0)
		m.set_surface_override_material(0, ring_mat)
		_world_root.add_child(m)
	# Small center marker so the origin is unambiguous at any zoom.
	var center := MeshInstance3D.new()
	var center_mesh := CylinderMesh.new()
	center_mesh.top_radius = 0.02
	center_mesh.bottom_radius = 0.02
	center_mesh.height = 0.001
	center.mesh = center_mesh
	center.position = Vector3(0, 0.001, 0)
	var center_mat := StandardMaterial3D.new()
	center_mat.albedo_color = Color(0.95, 0.55, 0.30, 1.0)   # orange-red origin dot
	center.set_surface_override_material(0, center_mat)
	_world_root.add_child(center)

func _build_terrain() -> void:
	# 45° wedges along the four floor edges (x=±10, z=±10).  Each wedge is
	# a tilted BoxShape3D — half buried below the floor — so the visible
	# surface forms a 45° outward slope: a walker trying to climb out
	# encounters increasing height and (with friction) slides back down.
	# 1.5 m horizontal × 1.5 m vertical at 45° → slope length ≈ 2.12 m.
	var floor_half:  float = 10.0
	var wedge_run:   float = 1.5     # horizontal extent of the slope
	var wedge_thick: float = 0.3     # box thickness perpendicular to slope
	var wedge_len:   float = 20.0    # span of one wedge along its edge
	var wedge_color: Color = Color(0.25, 0.30, 0.25, 1.0)
	var wedge_mat := StandardMaterial3D.new()
	wedge_mat.albedo_color = wedge_color
	wedge_mat.roughness = 0.9
	# Box dimensions: long along edge, thin perpendicular to slope,
	# length-along-slope ≈ wedge_run × sqrt(2).  The box CENTER sits at
	# (edge ± wedge_run/2, wedge_run/2, 0) and we rotate it 45° around
	# the long axis (the axis parallel to the edge) to form the ramp.
	var slope_len: float = wedge_run * sqrt(2.0)
	for edge in [
		# name      axis_dir,    inward_normal_dir,   long_axis
		["+x", Vector3(+1,0,0), Vector3(-1,0,0), Vector3(0,0,1)],
		["-x", Vector3(-1,0,0), Vector3(+1,0,0), Vector3(0,0,1)],
		["+z", Vector3(0,0,+1), Vector3(0,0,-1), Vector3(1,0,0)],
		["-z", Vector3(0,0,-1), Vector3(0,0,+1), Vector3(1,0,0)],
	]:
		var axis_dir:    Vector3 = edge[1]
		var long_axis:   Vector3 = edge[3]
		# Ramp body
		var body := StaticBody3D.new()
		body.collision_layer = _LAYER_WORLD
		body.collision_mask  = _world_collision_mask()
		body.physics_material_override = _make_wedge_mat()
		var cs := CollisionShape3D.new()
		var bs := BoxShape3D.new()
		bs.size = Vector3(slope_len, wedge_thick, wedge_len) if long_axis.z != 0 else Vector3(wedge_len, wedge_thick, slope_len)
		cs.shape = bs
		body.add_child(cs)
		# Mesh mirror of the box (so it's visible in UI)
		var mesh := MeshInstance3D.new()
		var bm := BoxMesh.new()
		bm.size = bs.size
		mesh.mesh = bm
		mesh.set_surface_override_material(0, wedge_mat)
		body.add_child(mesh)
		# Position: shift the wedge along its slope direction so the
		# top-inner corner lands exactly on the floor edge corner (10, 0)
		# — gives a smooth, step-less transition from flat floor to ramp.
		# The rest of the box clips below floor level; the visible surface
		# rises from y=0 at the edge to y≈1.4m at the far end.
		# Offset = wedge_thick/2 * sin(45°) = wedge_thick * sqrt(2) / 4.
		var slope_offset: float = wedge_thick * 0.25 * sqrt(2.0)
		var center: Vector3 = axis_dir * (floor_half + wedge_run * 0.5 + slope_offset)
		center.y = wedge_run * 0.5 - slope_offset
		# Rotate 45° around long_axis so the top face of the box (its
		# +Y direction in box-local) points inward-and-up.  Sign analysis:
		#   Basis(Z, +θ) rotates +Y toward -X  →  inward for +x edge
		#     so for axis_dir.x = +1 we need +45°
		#     for axis_dir.x = -1 we need -45°    → angle = +axis_dir.x * 45°
		#   Basis(X, +θ) rotates +Y toward +Z  →  inward for -z edge
		#     for axis_dir.z = +1 (inward = -Z) we need -45°
		#     for axis_dir.z = -1 (inward = +Z) we need +45°  → angle = -axis_dir.z * 45°
		var t := Transform3D(Basis(), center)
		if long_axis.z != 0:
			# Wedge runs along Z (i.e. it's on a +x or -x floor edge).
			t.basis = Basis(Vector3(0,0,1), axis_dir.x * deg_to_rad(45.0))
		else:
			# Wedge runs along X (i.e. it's on a +z or -z floor edge).
			t.basis = Basis(Vector3(1,0,0), -axis_dir.z * deg_to_rad(45.0))
		body.transform = t
		_world_root.add_child(body)

	# Low pyramids in the outer donut (r between 3.5 m and 9.5 m).
	# Pure-deterministic placement (no Godot RNG dependency — empirically
	# `terrain_rng.randf_range` produced degenerate values that collapsed
	# every theta to ≈0, leaving all pyramids in a radial line on +X).
	# Use a Halton-like quasi-random sequence built from the golden ratio
	# to give pseudo-random-looking but bit-deterministic placements.
	# All body seeds see the SAME layout — paired-seed A/B comparability
	# preserved.
	var pyr_color: Color = Color(0.35, 0.30, 0.22, 1.0)
	# Per-pyramid materials so walk_over_there can recolor one purple
	# without affecting the others.  pyr_color is the shared default;
	# each MeshInstance3D gets its own StandardMaterial3D instance.
	var n_pyramids: int = max(1, pyramid_count)
	# 2026-06-06 — engagement counters per pyramid (one int per pyramid).
	# Index aligned with _pyramid_xz_positions / _pyramid_xz_radii.
	_pyramid_engagement_counts.clear()
	_pyramid_engagement_counts.resize(n_pyramids)
	for _ee in range(n_pyramids):
		_pyramid_engagement_counts[_ee] = 0
	var placed: Array = []
	# NOTE: this file declares `const TAU: float = 0.02` at line 46 as
	# the brain tick time constant — it SHADOWS the math TAU (2π).  We
	# must use 2.0 * PI explicitly here, not TAU, or every theta collapses
	# to ~0 (the bug that produced a single radial row of pyramids).
	var two_pi: float = 2.0 * PI
	var bin_width: float = two_pi / float(n_pyramids)
	# Golden-ratio fractional offsets — low-discrepancy sequence.
	var phi_inv: float = 0.6180339887498949    # 1/φ
	var sqrt2_frac: float = 0.4142135623730951 # √2 − 1
	for i in range(n_pyramids):
		# Angular position: bin center + jitter in [-0.4, +0.4] × bin_width,
		# derived from i × (1/φ) wrapped to [0, 1) and remapped.
		var jitter_t: float = fmod(float(i) * phi_inv, 1.0)
		var theta: float = (float(i) + 0.5 + (jitter_t - 0.5) * 0.8) * bin_width
		# Radial position: 3.5 + (i × (√2 − 1) mod 1) × 6.0  → uniform in [3.5, 9.5]
		var radial_t: float = fmod(float(i) * sqrt2_frac, 1.0)
		var r: float = pyramid_min_r + radial_t * (pyramid_max_r - pyramid_min_r)
		var px: float = r * cos(theta)
		var pz: float = r * sin(theta)
		# Min-gap reject (rare under quasi-random distribution).
		var ok: bool = true
		for p in placed:
			if Vector2(px - p.x, pz - p.y).length() < 1.0:
				ok = false
				break
		if not ok:
			continue
		placed.append(Vector2(px, pz))
		# Base width: i × φ⁻¹ mapped to [0.6, 1.4].
		var bw_t: float = fmod(float(i) * phi_inv * 2.0, 1.0)
		var base_w: float = 0.6 + bw_t * 0.8
		# Track pyramid extent for in-place auto-reset collision avoidance.
		# The pyramid base is a square with corners at (±base_w/2, ±base_w/2);
		# its bounding circle has radius base_w * sqrt(2) / 2.
		_pyramid_xz_positions.append(Vector2(px, pz))
		_pyramid_xz_radii.append(base_w * 0.7071068)
		# Height: another quasi-random offset, in [0.05, 0.36].
		var h_t: float = fmod(float(i) * sqrt2_frac * 3.0, 1.0)
		var height: float = 0.05 + h_t * 0.31
		# Pyramid = CylinderMesh with 4 radial segments + top_radius=0.
		# CollisionShape3D doesn't have a pyramid primitive; use a
		# ConvexPolygonShape3D built from the mesh, or — simpler — use a
		# small box approximation slightly inset from the mesh edges.
		# For physics realism we use a ConvexPolygonShape3D matched to
		# the visual mesh.
		var pyr := StaticBody3D.new()
		pyr.collision_layer = _LAYER_WORLD
		pyr.collision_mask  = _world_collision_mask()
		pyr.physics_material_override = _make_contact_mat()
		# Build square pyramid vertices (4 base corners + apex).
		var verts := PackedVector3Array()
		var half: float = base_w * 0.5
		verts.append(Vector3(-half, 0.0, -half))
		verts.append(Vector3(+half, 0.0, -half))
		verts.append(Vector3(+half, 0.0, +half))
		verts.append(Vector3(-half, 0.0, +half))
		verts.append(Vector3(0.0, height, 0.0))
		var convex := ConvexPolygonShape3D.new()
		convex.points = verts
		var pcs := CollisionShape3D.new()
		pcs.shape = convex
		pyr.add_child(pcs)
		# Visual: 4-radial-segment cone matches the convex hull.
		var pmesh := MeshInstance3D.new()
		var cm := CylinderMesh.new()
		cm.bottom_radius = base_w * 0.5 * sqrt(2.0)   # circumscribed radius
		cm.top_radius    = 0.001
		cm.height        = height
		cm.radial_segments = 4
		cm.rings = 1
		pmesh.mesh = cm
		# Cylinder mesh is centered on Y; shift up by height/2 to align with base.
		pmesh.position = Vector3(0, height * 0.5, 0)
		var this_pyr_mat := StandardMaterial3D.new()
		this_pyr_mat.albedo_color = pyr_color
		this_pyr_mat.roughness = 0.95
		pmesh.set_surface_override_material(0, this_pyr_mat)
		_pyramid_meshes.append(pmesh)
		_pyramid_default_mats.append(this_pyr_mat)
		# Rotate 45° around Y so the square base corners align with the
		# cylinder's 4 radial segments.
		pmesh.rotation = Vector3(0, deg_to_rad(45.0), 0)
		pyr.add_child(pmesh)
		pyr.transform.origin = Vector3(px, 0.0, pz)
		_world_root.add_child(pyr)

	print("PicrawlerBody: terrain built — 4 edge ramps + %d pyramids" % placed.size())

# ---------------------------------------------------------------------------
# Corridor gym (OGMA_PICRAWLER_GYM=corridor) — a directed 1-D curriculum.
#
# Runs along +Z — the robot's TRUE forward: the eyes / front legs are on the +Z
# chassis face and the locomotor forward axis fwd_v = (vx,vz)·(sin yaw, cos yaw)
# is +Z at spawn (yaw=0).  (The IMU comment at ~1998 says "+X"; the locomotion
# math says +Z — this gym follows the BODY, so the robot spawns facing DOWN the
# trench and fwd_v directly measures corridor progress.)  It is FULLY ENCLOSED by
# four 30 deg walls — two along Z forming a self-centering trench, two sealing
# the -Z and +Z ends — so drift into any wall and the slope + gravity nudge the
# body back toward the middle.  That passive centering stands in for the
# not-yet-working heading reflex.  Down the corridor the terrain ramps in
# difficulty: flat runway -> gentle 10 deg hump -> half-buried rumble bumps -> a
# small pyramid field.  Forward distance (body diag `z`) is then a clean 1-D
# capability signal; the zone reached = difficulty conquered.
#
# 2026-07-27 — the two END walls were a VERTICAL seal at -Z and NOTHING at +Z,
# and both corrupted distance: a robot could park against the back wall with no
# escape while still reading fwd_v, and a fast one walked off the +Z edge of the
# world.  See the end-wall block below.  ANY CORRIDOR NUMBER MEASURED BEFORE
# THIS CHANGE IS FROM A DIFFERENT GYM and must be re-measured, not compared
# across — including the deployed baseline (net_z 4.75 +/- 0.29 @ 6000 ticks).
#
# Deterministic placement (no RNG) -> paired-seed A/B parity, as _build_terrain.
# Friction matches the live env: the floor + every obstacle use
# _make_contact_mat() (mu=1.5, as the arena floor + pyramids); the sloped walls
# use _make_wedge_mat() (mu=3.0, as the arena's containment ramps) so a grazing
# body follows the slope instead of tumbling.
# ---------------------------------------------------------------------------
func _build_corridor() -> void:
	var chan_half:    float = 0.75    # half-width of the flat channel floor -> 1.5 m walkable
	var corridor_len: float = 9.5     # +Z extent of the curriculum (fits the 20x20 floor)
	var slope_deg:    float = 30.0    # self-centering trench-wall angle
	var wall_face:    float = 1.2     # sloped-face length (rise = 1.2*sin30 ~ 0.60 m)
	var wall_thick:   float = 0.30    # wall box thickness (buried below the floor)
	var back_wall_z:  float = -0.5    # -Z seal, just behind spawn (sloped since 2026-07-27)

	# Reset pyramid bookkeeping (corridor XOR donut — never both).
	_pyramid_xz_positions.clear()
	_pyramid_xz_radii.clear()
	_pyramid_engagement_counts.clear()
	_pyramid_meshes.clear()
	_pyramid_default_mats.clear()

	var wall_color: Color = Color(0.25, 0.30, 0.25, 1.0)
	var th: float = deg_to_rad(slope_deg)
	var wall_len: float = corridor_len + 1.5           # span in Z (runway -> past pyramids + margin)
	var wall_cz:  float = corridor_len * 0.5           # center of that span
	# The back wall + channel-spanning obstacles (hump, rumble bumps) extend to
	# +/-thru_half in X so they clip ALL THE WAY THROUGH the angled side walls
	# (whose top edge sits at x ~ chan_half + wall_face*cos(slope)).  Without
	# this the obstacle ends flush at the wall's inner base while the wall leans
	# away above it, leaving a corner pocket a foot can wedge into.
	var thru_half: float = chan_half + wall_face * cos(th) + 0.15

	# Difficulty lever (0 = trivial .. 1 = hard) — scales obstacle HEIGHTS so the
	# corridor matches what the current gait can surmount.  Resolved via
	# ExperimentConfig: launcher spinbox > config metadata.gym_difficulty >
	# OGMA_PICRAWLER_GYM_DIFFICULTY env > 0.3 default.  Walls/back-wall unaffected.
	var diff: float = clamp(ExperimentConfig.resolve_picrawler_gym_difficulty(0.3), 0.0, 1.0)

	# --- Two self-centering trench walls at x = +/-chan_half (run along Z) ---
	# Build each from its inner-bottom edge (world x=+/-chan_half, y=0): the top
	# face rises along u_world (up + outward) at slope_deg; the surface normal n
	# points up + inward, so a body on the ramp slides back toward the center.
	for xside in [1.0, -1.0]:
		var n:         Vector3 = Vector3(-xside * sin(th), cos(th), 0.0)   # surface normal (up + inward)
		var u_world:   Vector3 = Vector3( xside * cos(th), sin(th), 0.0)   # up-slope (up + outward)
		var p_edge:    Vector3 = Vector3(xside * chan_half, 0.0, wall_cz)  # inner-bottom edge midpoint
		var center:    Vector3 = p_edge + u_world * (wall_face * 0.5) - n * (wall_thick * 0.5)
		var long_axis: Vector3 = Vector3(0, 0, 1)                          # wall runs along +Z
		var z_axis:    Vector3 = long_axis.cross(n).normalized()
		var wbasis:    Basis   = Basis(long_axis, n, z_axis)   # columns = images of local X/Y/Z
		var body := StaticBody3D.new()
		body.collision_layer = _LAYER_WORLD
		body.collision_mask  = _world_collision_mask()
		body.physics_material_override = _make_wedge_mat()
		var bs := BoxShape3D.new()
		bs.size = Vector3(wall_len, wall_thick, wall_face)   # local X=long(Z), Y=thick, Z=face
		var cs := CollisionShape3D.new()
		cs.shape = bs
		body.add_child(cs)
		var mi := MeshInstance3D.new()
		var bm := BoxMesh.new()
		bm.size = bs.size
		mi.mesh = bm
		var wm := StandardMaterial3D.new()
		wm.albedo_color = wall_color
		wm.roughness = 0.9
		mi.set_surface_override_material(0, wm)
		body.add_child(mi)
		body.transform = Transform3D(wbasis, center)
		_world_root.add_child(body)

	# --- End walls: 30 deg slopes sealing BOTH ends of the trench -------------
	# 2026-07-27 (operator UI observation) — these were the two ways the corridor
	# could corrupt a distance measurement, and they pull in opposite directions:
	#
	#   -Z  the back wall was a VERTICAL seal.  A robot that turned around and
	#       walked into it stayed parked against it, facing the wall, with no
	#       geometry able to nudge it out — while still accumulating fwd_v.  A
	#       trapped body that reads as "walking" is the blind-metric shape
	#       CLAUDE.md 3 rule 4 warns about, and it is invisible in net_z (which
	#       just stops rising) unless someone is watching.
	#   +Z  the far end simply DROPPED OFF: the curriculum runs to 9.5 m on a
	#       20x20 floor, so a fast arm walked off the world.  Already in the
	#       ledger — seed 1 of the load-stroke sweep posted the campaign's best
	#       distance (net_z 10.04) with mean chassis_y -39.29 and was charged a
	#       `fall` for it.  That bias hits the FASTEST arm first, i.e. exactly
	#       the arm a propulsion lever exists to demonstrate.
	#
	# Both are fixed by the geometry the SIDE walls already use: a 30 deg ramp
	# whose normal points up-and-inward, so gravity returns a body that reaches
	# it to the channel.  Same rise as the side walls (1.2*sin30 ~ 0.60 m), far
	# beyond anything this 0.085 m body can climb, so containment is absolute
	# while the trap and the cliff are both gone.
	#
	# The corridor is the OBSTACLE gym; the arena is where flat distance is
	# measured (an open floor has neither failure mode).  Containment here is
	# about not corrupting the obstacle result, not about making this the
	# distance gym.
	for zside in [-1.0, 1.0]:
		# zside = -1 seals the -Z end (behind spawn), +1 the far end.
		# Mirrors the side-wall construction with the long axis along X:
		# n = surface normal (up + INWARD, toward the channel), u_world = up-slope
		# (up + OUTWARD), and the wall is built from its inner-bottom edge.
		var edge_z: float = back_wall_z if zside < 0.0 else corridor_len + 0.3
		var en:        Vector3 = Vector3(0.0, cos(th), -zside * sin(th))   # up + inward
		var eu_world:  Vector3 = Vector3(0.0, sin(th),  zside * cos(th))   # up + outward
		var ep_edge:   Vector3 = Vector3(0.0, 0.0, edge_z)
		var ecenter:   Vector3 = ep_edge + eu_world * (wall_face * 0.5) - en * (wall_thick * 0.5)
		var elong:     Vector3 = Vector3(1, 0, 0)                          # end wall runs along X
		var ez_axis:   Vector3 = elong.cross(en).normalized()
		var ebasis:    Basis   = Basis(elong, en, ez_axis)
		var ewall := StaticBody3D.new()
		ewall.collision_layer = _LAYER_WORLD
		ewall.collision_mask  = _world_collision_mask()
		ewall.physics_material_override = _make_wedge_mat()   # mu=3.0, as the side walls
		var ebs := BoxShape3D.new()
		ebs.size = Vector3(thru_half * 2.0, wall_thick, wall_face)   # clip through both side walls
		var ecs := CollisionShape3D.new()
		ecs.shape = ebs
		ewall.add_child(ecs)
		var emi := MeshInstance3D.new()
		var ebm := BoxMesh.new()
		ebm.size = ebs.size
		emi.mesh = ebm
		var ewm := StandardMaterial3D.new()
		ewm.albedo_color = wall_color
		ewm.roughness = 0.9
		emi.set_surface_override_material(0, ewm)
		ewall.add_child(emi)
		ewall.transform = Transform3D(ebasis, ecenter)
		_world_root.add_child(ewall)

	# --- Zone 1: gentle hump (up then down), spanning the channel.
	# Triangular prism: base z in [2.0, 4.0] (1.0 m run each side), apex height
	# scales with difficulty (0.025..0.16 m -> slope ~1.4..9.1 deg) at z=3.0.
	# PrismMesh visual (rotated 90 deg so its ridge runs across X, slopes face
	# +/-Z) + matching 6-vert convex hull.
	var hump_cz:   float = 3.0
	var hump_half: float = 1.0                          # half base run (1.0 m in Z each side)
	var hump_h:    float = lerp(0.025, 0.16, diff)      # peak; slope = atan(peak/run)
	var hump := StaticBody3D.new()
	hump.collision_layer = _LAYER_WORLD
	hump.collision_mask  = _world_collision_mask()
	hump.physics_material_override = _make_contact_mat()
	var hverts := PackedVector3Array()
	hverts.append(Vector3(-thru_half, -hump_h * 0.5, -hump_half))
	hverts.append(Vector3( thru_half, -hump_h * 0.5, -hump_half))
	hverts.append(Vector3( thru_half, -hump_h * 0.5,  hump_half))
	hverts.append(Vector3(-thru_half, -hump_h * 0.5,  hump_half))
	hverts.append(Vector3(-thru_half,  hump_h * 0.5, 0.0))
	hverts.append(Vector3( thru_half,  hump_h * 0.5, 0.0))
	var hconvex := ConvexPolygonShape3D.new()
	hconvex.points = hverts
	var hcs := CollisionShape3D.new()
	hcs.shape = hconvex
	hump.add_child(hcs)
	var hmi := MeshInstance3D.new()
	var prism := PrismMesh.new()
	prism.size = Vector3(hump_half * 2.0, hump_h, thru_half * 2.0)
	hmi.mesh = prism
	hmi.rotation = Vector3(0, deg_to_rad(90.0), 0)   # ridge across X, slopes face +/-Z
	var hmat := StandardMaterial3D.new()
	hmat.albedo_color = Color(0.30, 0.33, 0.28, 1.0)
	hmat.roughness = 0.9
	hmi.set_surface_override_material(0, hmat)
	hump.add_child(hmi)
	hump.transform.origin = Vector3(0.0, hump_h * 0.5, hump_cz)
	_world_root.add_child(hump)

	# --- Zone 2: rumble strips — half-buried cylinders across the channel.
	# Axis along X (span wall-to-wall).  Exposed height scales with difficulty;
	# the cylinder is sunk so only `bump_exposed` pokes above the floor (well
	# under a radius, so it is actually climbable).  Every 0.5 m in z in
	# [4.25, 6.25] -> 5 bumps.
	var bump_r:       float = 0.06
	var bump_exposed: float = lerp(0.012, 0.065, diff)   # height above the floor
	var bump_cy:      float = bump_exposed - bump_r        # cylinder center Y (< 0 = sunk)
	var n_bumps: int = 0
	var bz: float = 4.25
	while bz <= 6.30:
		var bump := StaticBody3D.new()
		bump.collision_layer = _LAYER_WORLD
		bump.collision_mask  = _world_collision_mask()
		bump.physics_material_override = _make_contact_mat()
		var bcyl := CylinderShape3D.new()
		bcyl.radius = bump_r
		bcyl.height = thru_half * 2.0   # span through both walls
		var bcs2 := CollisionShape3D.new()
		bcs2.shape = bcyl
		bump.add_child(bcs2)
		var bmi2 := MeshInstance3D.new()
		var bcm := CylinderMesh.new()
		bcm.top_radius = bump_r
		bcm.bottom_radius = bump_r
		bcm.height = thru_half * 2.0
		bcm.radial_segments = 12
		bmi2.mesh = bcm
		var rmat := StandardMaterial3D.new()
		rmat.albedo_color = Color(0.40, 0.32, 0.22, 1.0)
		rmat.roughness = 0.95
		bmi2.set_surface_override_material(0, rmat)
		bump.add_child(bmi2)
		# Lay the cylinder on its side (axis Y -> X), sunk so only bump_exposed shows.
		bump.transform = Transform3D(Basis(Vector3(0, 0, 1), deg_to_rad(90.0)), Vector3(0.0, bump_cy, bz))
		_world_root.add_child(bump)
		n_bumps += 1
		bz += 0.5

	# --- Zone 3: pyramid field — small climbable pyramids, z in [6.5, 9.5].
	# Deterministic staggered placement across the channel (golden-ratio
	# quasi-random sizes, as the arena).  Kept clear of the +/-chan_half walls.
	var phi_inv: float = 0.6180339887498949
	var n_pyr: int = 7
	for i in range(n_pyr):
		var pz: float = 6.9 + float(i) * 0.38
		var px: float = 0.35 if (i % 2 == 0) else -0.35
		if i % 3 == 2:
			px = 0.0
		var bw_t: float = fmod(float(i) * phi_inv * 2.0, 1.0)
		var base_w: float = 0.5 + bw_t * 0.35
		var h_t: float = fmod(float(i) * phi_inv * 3.0, 1.0)
		var height: float = (0.10 + h_t * 0.22) * lerp(0.6, 1.25, diff)
		_add_pyramid(px, pz, base_w, height)

	print("PicrawlerBody: corridor gym built (+Z, difficulty=%.2f) — hump %.3f m + %d bumps %.3f m + %d pyramids" % [diff, hump_h, n_bumps, bump_exposed, _pyramid_xz_positions.size()])

# Build one square pyramid (convex-hull collision + 4-radial-segment cone mesh)
# at (px, pz) and register it in the pyramid bookkeeping arrays so nav /
# engagement / reset-avoidance behave the same as the arena.  Factored out of
# the corridor builder; mirrors the inline arena pyramid in _build_terrain.
func _add_pyramid(px: float, pz: float, base_w: float, height: float) -> void:
	var pyr := StaticBody3D.new()
	pyr.collision_layer = _LAYER_WORLD
	pyr.collision_mask  = _world_collision_mask()
	pyr.physics_material_override = _make_contact_mat()
	var verts := PackedVector3Array()
	var half: float = base_w * 0.5
	verts.append(Vector3(-half, 0.0, -half))
	verts.append(Vector3(+half, 0.0, -half))
	verts.append(Vector3(+half, 0.0, +half))
	verts.append(Vector3(-half, 0.0, +half))
	verts.append(Vector3(0.0, height, 0.0))
	var convex := ConvexPolygonShape3D.new()
	convex.points = verts
	var pcs := CollisionShape3D.new()
	pcs.shape = convex
	pyr.add_child(pcs)
	var pmesh := MeshInstance3D.new()
	var cm := CylinderMesh.new()
	cm.bottom_radius = base_w * 0.5 * sqrt(2.0)   # circumscribed radius
	cm.top_radius    = 0.001
	cm.height        = height
	cm.radial_segments = 4
	cm.rings = 1
	pmesh.mesh = cm
	pmesh.position = Vector3(0, height * 0.5, 0)
	var this_pyr_mat := StandardMaterial3D.new()
	this_pyr_mat.albedo_color = Color(0.35, 0.30, 0.22, 1.0)
	this_pyr_mat.roughness = 0.95
	pmesh.set_surface_override_material(0, this_pyr_mat)
	pmesh.rotation = Vector3(0, deg_to_rad(45.0), 0)
	pyr.add_child(pmesh)
	pyr.transform.origin = Vector3(px, 0.0, pz)
	_world_root.add_child(pyr)
	# Bookkeeping parity with the arena donut.
	_pyramid_xz_positions.append(Vector2(px, pz))
	_pyramid_xz_radii.append(base_w * 0.7071068)
	_pyramid_meshes.append(pmesh)
	_pyramid_default_mats.append(this_pyr_mat)
	_pyramid_engagement_counts.append(0)

# Live gym swap (KEY_1 = arena / KEY_2 = corridor).  Frees WorldRoot and rebuilds
# the environment around the SAME robot + brain — no scene reload, so the learned
# model stays fully continuous (an EXPERIENCED agent dropped into a new scenario).
# The robot is teleported to the origin spawn so it starts fresh in the new world.
func _switch_gym(mode: String) -> void:
	if mode == _gym_mode_active:
		_ui_notify("[gym] already in %s" % mode)
		return
	_gym_mode_override = mode
	# Tear down the world (WorldRoot owns floor + rings + all obstacles/pyramids).
	# remove_child detaches it from the tree IMMEDIATELY (no one-frame double world),
	# then queue_free deallocates it safely.
	if _world_root != null:
		remove_child(_world_root)
		_world_root.queue_free()
		_world_root = null
	# Pyramid bookkeeping is rebuilt by the gym builders; clear stale entries.
	_pyramid_xz_positions.clear()
	_pyramid_xz_radii.clear()
	_pyramid_engagement_counts.clear()
	_pyramid_meshes.clear()
	_pyramid_default_mats.clear()
	_nearest_pyramid_idx = -1
	# Rebuild the world in the new mode.
	_world_root = Node3D.new()
	_world_root.name = "WorldRoot"
	add_child(_world_root)
	_rebuild_world_contents()
	# Drop the experienced robot at the new world's origin spawn (brain untouched;
	# _do_hard_reset only publishes events.reset for Gate-0 masking, no relearning).
	_pending_reset_offset = Vector3.ZERO
	_do_hard_reset()
	print("PicrawlerBody: [gym] switched -> %s" % _gym_mode_active)
	_ui_notify("[gym] switched -> %s" % _gym_mode_active)

# Controlled belly-on-ramp test (KEY_3, or headless env OGMA_PICRAWLER_TELEPORT_RAMP_AT=<tick>).
# Drops the EXPERIENCED robot onto the corridor hump (z=3) to test the height
# reflex's high-center recovery WITHOUT relying on the gait to navigate there.
# Rigid-translates ALL body parts (preserves the current developed pose + the
# brain's learned model in memory), zeroes velocity, and positions the body above
# the peak so it drops onto the slope.  Announces a reset for Gate-0 masking only
# (no relearning).  Corridor-only (the hump doesn't exist in the arena).
# Surface height under (x,z) — so a drop lands ON a hump/wall rather than inside it.
func _surface_y_at(x: float, z: float) -> float:
	var ss := get_world_3d().direct_space_state
	if ss == null:
		return 0.0
	var q := PhysicsRayQueryParameters3D.create(Vector3(x, 5.0, z), Vector3(x, -1.0, z))
	var h := ss.intersect_ray(q)
	return float(h.position.y) if not h.is_empty() else 0.0

func _teleport_to_ramp() -> void:
	if _gym_mode_active != "corridor":
		_ui_notify("[teleport] the ramp only exists in the corridor gym (press 2)")
		return
	var tx: float = 0.0
	var tz: float = 3.0   # hump peak by default
	var xz: String = OS.get_environment("OGMA_PICRAWLER_TELEPORT_XZ")   # "x,z" to target a wall etc.
	if xz != "":
		var parts := xz.split(",")
		if parts.size() == 2:
			tx = parts[0].to_float(); tz = parts[1].to_float()
	# Find the surface height at (tx,tz) so tall targets (walls) drop ONTO the surface.
	var surf_y: float = 0.0
	var ss := get_world_3d().direct_space_state
	if ss != null:
		var q := PhysicsRayQueryParameters3D.new()
		q.from = Vector3(tx, 2.0, tz); q.to = Vector3(tx, -1.0, tz); q.collision_mask = _LAYER_WORLD
		var h := ss.intersect_ray(q)
		if not h.is_empty():
			surf_y = h.position.y
	_pending_teleport = Vector3(tx, surf_y, tz)   # applied next physics frame

# Drop the EXPERIENCED robot (brain + current pose preserved) onto a ground point.
# Rigid-translates every part so the chassis lands ~0.30 m ABOVE `ground` (the
# surface height under it, so it clears tall obstacles) and falls onto it; zeroes
# velocity; announces a reset for Gate-0 masking only (no relearning).  MUST be
# called from the physics step (see _pending_teleport) or the write is clobbered.
func _teleport_to(ground: Vector3) -> void:
	# Optional FLIP (env OGMA_PICRAWLER_TELEPORT_FLIP=1): rotate the whole assembly
	# 180° about X → drops it upside-down (a controlled INVALID posture for the
	# keyframe bake-gate A/B).  Rigid transform about the chassis pivot.
	# Per-drop intent (mouse placement) wins; scripted/env runs fall through to the env var.
	var flip: bool = (_pending_teleport_flip == 1) if _pending_teleport_flip >= 0 \
		else OS.get_environment("OGMA_PICRAWLER_TELEPORT_FLIP") == "1"
	var drop: Vector3 = Vector3(ground.x, ground.y + (0.35 if flip else 0.30), ground.z)
	var pivot: Vector3 = _chassis.global_transform.origin
	var rot: Basis = Basis(Vector3(1, 0, 0), PI) if flip else Basis.IDENTITY
	var parts: Array = [_chassis] + (_coxas as Array) + (_uppers as Array) + (_lowers as Array)
	for b in parts:
		b.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
		b.freeze = true
	for b in parts:
		var rel: Vector3 = rot * (b.global_transform.origin - pivot)
		b.global_transform = Transform3D(rot * b.global_transform.basis, drop + rel)
		b.linear_velocity = Vector3.ZERO
		b.angular_velocity = Vector3.ZERO
	for b in parts:
		b.freeze = false
	_last_drop = drop
	_last_drop_flip = flip
	_last_drop_tick = tick_counter
	if brain != null:
		brain.publish_event("reset", 1.0)
	print("PicrawlerBody: [teleport] dropped the experienced robot at (%.2f, %.2f)%s tick %d" % [
		ground.x, ground.z, "  FLIPPED" if flip else "", tick_counter])
	_ui_notify("[teleport] dropped%s at (%.1f, %.1f)" % [" (flipped)" if flip else "", ground.x, ground.z])

# --- Mouse-guided teleport placement (KEY_4) --------------------------------
# Enter placement mode: the mouse projects a marker onto the floor; LEFT-CLICK
# drops the experienced robot there.  A general tool — reposition the agent to
# any spot to probe recovery / behaviour from arbitrary states.
var _place_mode: bool = false
var _place_marker: Node3D = null
var _place_target: Vector3 = Vector3.ZERO
# Last actually-dropped teleport target + whether it was inverted.  Surfaced on the HUD so
# a placement can be reported back verbatim and replayed headless via
# OGMA_PICRAWLER_TELEPORT_XZ / _FLIP — otherwise a UI observation is not reproducible.
var _last_drop: Vector3 = Vector3.ZERO
var _last_drop_flip: bool = false
var _last_drop_tick: int = -1

func _set_place_mode(on: bool) -> void:
	_place_mode = on
	if on:
		_ui_notify("[4] place mode — LEFT-click = drop upright, RIGHT-click = drop INVERTED")
	if _place_marker == null and on:
		_place_marker = _make_place_marker()
		add_child(_place_marker)
	if _place_marker != null:
		_place_marker.visible = on
	_ui_notify("[place] mouse-teleport %s%s" % [
		"ON — move mouse, LEFT-CLICK to drop" if on else "off", "  (4 to toggle)"])

func _make_place_marker() -> Node3D:
	var root := Node3D.new()
	root.name = "TeleportMarker"
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(1.0, 0.85, 0.1, 1.0)
	mat.emission_enabled = true
	mat.emission = Color(1.0, 0.7, 0.0)
	for ang in [45.0, -45.0]:   # two crossed bars = an X on the floor
		var mi := MeshInstance3D.new()
		var bm := BoxMesh.new()
		bm.size = Vector3(0.5, 0.008, 0.06)
		mi.mesh = bm
		mi.rotation = Vector3(0, deg_to_rad(ang), 0)
		mi.set_surface_override_material(0, mat)
		root.add_child(mi)
	return root

func _place_ground_from_mouse() -> Variant:
	# Raycast the mouse into the scene and return the SURFACE point under the cursor
	# (floor OR the top of a ramp/pyramid), so the marker sits ON the geometry and
	# the drop lands on whatever's there.  Falls back to the y=0 plane past geometry.
	var cam := get_viewport().get_camera_3d()
	if cam == null:
		return null
	var mpos: Vector2 = get_viewport().get_mouse_position()
	var from: Vector3 = cam.project_ray_origin(mpos)
	var dir: Vector3 = cam.project_ray_normal(mpos)
	var space := get_world_3d().direct_space_state
	if space != null:
		var query := PhysicsRayQueryParameters3D.new()
		query.from = from
		query.to = from + dir * 100.0
		query.collision_mask = _LAYER_WORLD          # floor + obstacles (not the robot)
		var hit := space.intersect_ray(query)
		if not hit.is_empty():
			return hit.position
	# Nothing hit — project onto the y=0 floor plane.
	if abs(dir.y) < 1e-5:
		return null
	var t: float = -from.y / dir.y
	if t < 0.0:
		return null
	return from + dir * t

# ---------------------------------------------------------------------------
# Body construction — chassis + 4 legs
# ---------------------------------------------------------------------------
## World-body collision mask.  Adding _LAYER_CHASSIS lets the floor/obstacles SEE the
## chassis; the chassis must also be given _LAYER_WORLD in its own mask (see _build_body).
## Both directions are set rather than relying on one-sided matching — a half-wired toggle
## that silently does nothing is this codebase's characteristic failure shape.
## Live chassis-friction setter for the [K] panel.  Mutates the shared chassis material in
## place, so it takes effect on the next contact without a rebuild.
func set_chassis_friction(v: float) -> void:
	chassis_friction = clampf(v, 0.0, 3.0)
	if _chassis_mat != null:
		_chassis_mat.friction = chassis_friction

func _world_collision_mask() -> int:
	return _LAYER_BODY | (_LAYER_CHASSIS if chassis_collides else 0)

## Re-point the chassis + every existing world body at the current toggle state, so [J] can
## flip it live without rebuilding the gym.
func _apply_chassis_collision() -> void:
	if _chassis != null and is_instance_valid(_chassis):
		_chassis.collision_mask = _LAYER_WORLD if chassis_collides else 0
		_chassis.physics_material_override = _make_chassis_mat()
	var wr: Node = get_tree().get_root().find_child("WorldRoot", true, false)
	var n: int = 0
	if wr != null:
		var stack: Array = [wr]
		while not stack.is_empty():
			var nd: Node = stack.pop_back()
			for c in nd.get_children():
				stack.append(c)
			if nd is StaticBody3D or nd is RigidBody3D:
				if (nd as CollisionObject3D).collision_layer & _LAYER_WORLD:
					(nd as CollisionObject3D).collision_mask = _world_collision_mask()
					n += 1
	print("PicrawlerBody: chassis_collides = %s  (%d world bodies re-masked)" % [
		chassis_collides, n])
	_ui_notify("[J] chassis collision: %s" % ("ON" if chassis_collides else "OFF (ghost)"))

# ---------------------------------------------------------------------------
# Geometry loading — populates the body vars from JSON before _build_body()
# ---------------------------------------------------------------------------
# Called first thing in _ready(), and again by _rebuild_body() for a live swap.
#
# A missing or malformed file is deliberately NON-FATAL: the class-level
# literals ARE the CAD body, so a load failure degrades to the historical build
# rather than to something silently wrong.  Returns true iff a file was applied.
#
# OGMA_PICRAWLER_BODY selects the body without editing the scene, so the A/B
# harnesses can swap it per-arm.  Accepts a bare name ("measured") or a full
# res:// path.
func _load_geometry(path_override: String = "") -> bool:
	var p: String = body_geometry_path
	if path_override != "":
		p = path_override
	else:
		var env_body: String = OS.get_environment("OGMA_PICRAWLER_BODY")
		if env_body != "":
			p = env_body if env_body.begins_with("res://") \
				else "res://addons/ami_ogma/body/%s.json" % env_body

	var applied: bool = false
	if not FileAccess.file_exists(p):
		push_warning("PicrawlerBody: geometry '%s' not found — keeping built-in CAD values" % p)
	else:
		var f := FileAccess.open(p, FileAccess.READ)
		if f == null:
			push_warning("PicrawlerBody: cannot open geometry '%s' — keeping built-in CAD values" % p)
		else:
			var txt: String = f.get_as_text()
			f.close()
			var parsed: Variant = JSON.parse_string(txt)
			if not (parsed is Dictionary):
				push_error("PicrawlerBody: geometry '%s' is not a JSON object — keeping built-in CAD values" % p)
			else:
				_apply_geometry(parsed as Dictionary)
				_geometry_name = str((parsed as Dictionary).get("name", p.get_file()))
				applied = true
				print("PicrawlerBody: geometry '%s' loaded from %s" % [_geometry_name, p])

	_recompute_derived_geometry()
	return applied

# Reads one nested float, leaving the current value untouched if absent — so a
# partial JSON overrides only what it names.
func _geom_f(d: Dictionary, group: String, key: String, current: float) -> float:
	if d.has(group) and d[group] is Dictionary:
		var g: Dictionary = d[group]
		if g.has(key):
			return float(g[key])
	return current

func _apply_geometry(d: Dictionary) -> void:
	L1                   = _geom_f(d, "links", "l1", L1)
	L2                   = _geom_f(d, "links", "l2", L2)
	L3                   = _geom_f(d, "links", "l3", L3)
	COXA_Z_DROP          = _geom_f(d, "links", "coxa_z_drop", COXA_Z_DROP)

	CHASSIS_X            = _geom_f(d, "chassis", "x", CHASSIS_X)
	CHASSIS_Y            = _geom_f(d, "chassis", "y", CHASSIS_Y)
	CHASSIS_Z            = _geom_f(d, "chassis", "z", CHASSIS_Z)
	HIP_X_SPAN           = _geom_f(d, "chassis", "hip_x_span", HIP_X_SPAN)
	HIP_Z_SPAN           = _geom_f(d, "chassis", "hip_z_span", HIP_Z_SPAN)
	STANDING_CHASSIS_Y   = _geom_f(d, "chassis", "standing_y", STANDING_CHASSIS_Y)

	HIP1_REST            = _geom_f(d, "rest", "hip1", HIP1_REST)
	HIP2_REST            = _geom_f(d, "rest", "hip2", HIP2_REST)
	KNEE_REST            = _geom_f(d, "rest", "knee", KNEE_REST)
	LOWER_LEG_DROP_ANGLE = _geom_f(d, "rest", "lower_leg_drop_angle", LOWER_LEG_DROP_ANGLE)

	HIP1_LIMIT           = _geom_f(d, "limits", "hip1", HIP1_LIMIT)
	HIP2_LIMIT           = _geom_f(d, "limits", "hip2", HIP2_LIMIT)
	KNEE_LIMIT_LOW       = _geom_f(d, "limits", "knee_low", KNEE_LIMIT_LOW)
	KNEE_LIMIT_HIGH      = _geom_f(d, "limits", "knee_high", KNEE_LIMIT_HIGH)
	KNEE_LIMIT_LOW_NARROW = _geom_f(d, "limits", "knee_low_narrow", KNEE_LIMIT_LOW_NARROW)

	CHASSIS_MASS         = _geom_f(d, "masses", "chassis", CHASSIS_MASS)
	COXA_MASS            = _geom_f(d, "masses", "coxa", COXA_MASS)
	UPPER_MASS           = _geom_f(d, "masses", "upper", UPPER_MASS)
	LOWER_MASS           = _geom_f(d, "masses", "lower", LOWER_MASS)

	COXA_RADIUS          = _geom_f(d, "radii", "coxa", COXA_RADIUS)
	LEG_RADIUS           = _geom_f(d, "radii", "leg", LEG_RADIUS)

	# Optional multi-box chassis + explicit CoM.  Both absent ⇒ legacy path.
	_chassis_boxes.clear()
	_chassis_com_valid = false
	if d.has("chassis") and d["chassis"] is Dictionary:
		var ch: Dictionary = d["chassis"]
		if ch.has("boxes") and ch["boxes"] is Array:
			for b in (ch["boxes"] as Array):
				if b is Dictionary and b.has("size") and b.has("offset"):
					_chassis_boxes.append({
						"name":   str((b as Dictionary).get("name", "box")),
						"size":   _to_vec3((b as Dictionary)["size"]),
						"offset": _to_vec3((b as Dictionary)["offset"]),
					})
		if ch.has("center_of_mass"):
			_chassis_com = _to_vec3(ch["center_of_mass"])
			_chassis_com_valid = true

func _to_vec3(v: Variant) -> Vector3:
	if v is Array and (v as Array).size() >= 3:
		return Vector3(float(v[0]), float(v[1]), float(v[2]))
	return Vector3.ZERO

# Everything downstream of the loaded numbers.  Split out so _rebuild_body()
# and the fallback path both get it.
func _recompute_derived_geometry() -> void:
	# Was `const _TOTAL_MASS = CHASSIS_MASS + ...`, which would have frozen at
	# the CAD total the moment the masses became loadable.
	_TOTAL_MASS = CHASSIS_MASS + 4.0 * (COXA_MASS + UPPER_MASS + LOWER_MASS)

	# Topmost chassis surface = what touches down when the robot is on its back.
	_chassis_top_local = CHASSIS_Y * 0.5
	if not _chassis_boxes.is_empty():
		_chassis_top_local = -INF
		for spec in _chassis_boxes:
			var sz: Vector3 = spec["size"]
			var off: Vector3 = spec["offset"]
			_chassis_top_local = max(_chassis_top_local, off.y + sz.y * 0.5)

	# target_height / peak_height follow the body's standing height, but ONLY
	# while still sitting at the class-level literal.  An explicit scene, env
	# (OGMA_PICRAWLER_TARGET_HEIGHT) or curriculum value must survive a body
	# swap untouched — those resolve later in _ready() and win.
	if is_equal_approx(target_height, _EXPORT_DEFAULT_HEIGHT):
		target_height = STANDING_CHASSIS_Y
	if is_equal_approx(peak_height, _EXPORT_DEFAULT_HEIGHT):
		peak_height = STANDING_CHASSIS_Y

# ---------------------------------------------------------------------------
# Live body swap — the morphological (d) test
# ---------------------------------------------------------------------------
# Rebuilds the PHYSICAL body from a different geometry while leaving the brain
# completely untouched: GNG, EPMs and every learned weight persist across the
# swap.  That is the whole point — what follows is RE-INFERENCE, not
# re-initialisation.  Resetting the brain here would turn the sharpest single
# piece of evidence we have into a demo (picrawler_sim2real_port.md §Phase 5).
#
# To read it as evidence, log TLE through the transition: the spike-then-decay
# IS the re-inference.  Without that trace it is a video.
#
# Mirrors _switch_gym(): remove_child detaches IMMEDIATELY (no one-frame double
# body), queue_free deallocates safely afterwards.
func _rebuild_body(geometry_path: String = "") -> void:
	# 1 — Remember pose and motion, so a body swap is a MORPHOLOGICAL
	#     perturbation and not silently also a teleport.  Confounding the two
	#     would make the (d) test unreadable.
	var had_body: bool = is_instance_valid(_chassis)
	# ⚠ POSITION ONLY — the orientation is deliberately DISCARDED and the new
	# body is rebuilt axis-aligned.
	#
	# _build_leg() constructs the entire limb in WORLD axes: `heading` and
	# `lateral` come from the world-frame NEUTRAL_HEADINGS, hip1_local is added
	# to the chassis origin WITHOUT the basis, coxa_dir uses world Vector3.DOWN,
	# and the hinge frames are built from world RIGHT/UP with hand-verified
	# handedness.  All of that is correct only while the chassis basis is
	# IDENTITY, which it always is at spawn.
	#
	# Restoring a tilted transform here therefore builds the legs on world axes
	# around a rotated chassis, which visibly deforms the robot — front hips ride
	# higher than rear, left hips sit forward of right.  Rebuilding upright is
	# correct; preserving orientation would require rewriting _build_leg() in
	# chassis-local space, including the handedness-sensitive joint frames.
	# Logged as a follow-up in picrawler_sim2real_port.md.
	var keep_origin: Vector3 = _chassis.global_transform.origin if had_body else Vector3.ZERO
	var keep_lin:   Vector3  = _chassis.linear_velocity  if had_body else Vector3.ZERO
	var keep_ang:   Vector3  = _chassis.angular_velocity if had_body else Vector3.ZERO

	# 2 — Tear down.  JOINTS FIRST: a Godot joint outliving its bodies asserts.
	for jarr in [_hip1_joints, _hip2_joints, _knee_joints]:
		for j in jarr:
			if is_instance_valid(j):
				remove_child(j)
				j.queue_free()
	for barr in [_coxas, _uppers, _lowers]:
		for b in barr:
			if is_instance_valid(b):
				remove_child(b)
				b.queue_free()
	if had_body:
		remove_child(_chassis)
		_chassis.queue_free()
	_chassis = null

	# Every array _build_leg() appends to must be cleared, or the rebuilt body
	# indexes into stale entries from the previous morphology.
	_hip1_joints.clear(); _hip2_joints.clear(); _knee_joints.clear()
	_coxas.clear();       _uppers.clear();      _lowers.clear()
	_hip2_axes.clear();   _knee_axes.clear()
	_coxa_rest_xform.clear(); _upper_rest_xform.clear(); _lower_rest_xform.clear()
	_hip1_world_c.clear();    _hip2_world_c.clear();     _knee_world_c.clear()
	_foot_load_ema        = [0.0, 0.0, 0.0, 0.0]
	_foot_was_in_contact  = [false, false, false, false]

	# 3 — New dimensions, then rebuild AT THE REMEMBERED POSE.  _build_leg()
	#     anchors off _chassis.global_transform, so the spawn override has to be
	#     applied inside _build_body() before the legs are placed.
	_load_geometry(geometry_path)
	_body_spawn_xform = Transform3D(Basis.IDENTITY, keep_origin)
	_body_spawn_valid = had_body
	_build_body()
	_body_spawn_valid = false
	_apply_limb_materials()
	# Consumers holding the OLD chassis instance are now dangling — the orbit
	# camera and the walking trail both track by reference, not NodePath.
	_retarget_body_watchers()

	if had_body and is_instance_valid(_chassis):
		_chassis.linear_velocity  = keep_lin
		_chassis.angular_velocity = keep_ang

	print("PicrawlerBody: BODY SWAPPED → '%s'  (brain untouched — TLE trace is the evidence)"
		% _geometry_name)
	_ui_notify("[body] swapped → %s" % _geometry_name)

func _build_body() -> void:
	# Chassis
	_chassis = RigidBody3D.new()
	_chassis.mass = CHASSIS_MASS
	_chassis.position = Vector3(0, STANDING_CHASSIS_Y, 0)
	# Live swap: keep the pose the old body had, so _build_leg() (which anchors
	# off _chassis.global_transform) places the new limbs around the CURRENT
	# location rather than at the origin.
	if _body_spawn_valid:
		_chassis.transform = _body_spawn_xform
	_chassis.collision_layer = _LAYER_CHASSIS   # separate layer; floor.mask
												# doesn't include it, so the
												# chassis never touches floor.
	# 0 = the historical ghost chassis (legs are attached via joint constraints, not
	# collisions, so the chassis needs no mask for the robot to hold together).
	_chassis.collision_mask  = _LAYER_WORLD if chassis_collides else 0
	_chassis.angular_damp = BODY_ANGULAR_DAMP * body_damp_scale
	_chassis.linear_damp  = BODY_LINEAR_DAMP * body_damp_scale
	_chassis.physics_material_override = _make_chassis_mat()
	if _chassis_boxes.is_empty():
		# Legacy single centred box — byte-identical path for cad.json.
		var cs := CollisionShape3D.new()
		var cb := BoxShape3D.new()
		cb.size = Vector3(CHASSIS_X, CHASSIS_Y, CHASSIS_Z)
		cs.shape = cb
		_chassis.add_child(cs)
		var cm := MeshInstance3D.new()
		var cbm := BoxMesh.new()
		cbm.size = cb.size
		cm.mesh = cbm
		var cmat := StandardMaterial3D.new()
		cmat.albedo_color = Color(0.65, 0.55, 0.40, 1.0)
		cm.set_surface_override_material(0, cmat)
		_chassis.add_child(cm)
	else:
		# Multi-box chassis (measured body: 41 mm base plate + 62 mm electronics
		# stack).  The BASE's underside is the belly — the surface that actually
		# contacts the floor — so its size and offset are load-bearing for every
		# clearance metric, not decoration.
		var cmat_multi := StandardMaterial3D.new()
		cmat_multi.albedo_color = Color(0.65, 0.55, 0.40, 1.0)
		for spec in _chassis_boxes:
			var sz:  Vector3 = spec["size"]
			var off: Vector3 = spec["offset"]
			var cs2 := CollisionShape3D.new()
			var cb2 := BoxShape3D.new()
			cb2.size = sz
			cs2.shape = cb2
			cs2.position = off
			_chassis.add_child(cs2)
			var cm2 := MeshInstance3D.new()
			var cbm2 := BoxMesh.new()
			cbm2.size = sz
			cm2.mesh = cbm2
			cm2.position = off
			cm2.set_surface_override_material(0, cmat_multi)
			_chassis.add_child(cm2)

	# A RigidBody3D has one mass, so an internal distribution is expressible
	# ONLY as an explicit centre of mass.  For the measured body this is what
	# reproduces the tape-measured CoG by construction (gate G1) instead of
	# hoping the box geometry happens to imply it.
	if _chassis_com_valid:
		_chassis.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
		_chassis.center_of_mass = _chassis_com

	# Visual "eyes" on the +Z chassis face (front).  Two black half-embedded
	# spheres (visible hemisphere from outside).  Visual-only — no
	# collision, no physics.  Marks the body's front so the operator can
	# read heading at a glance, in advance of the compass EPM and
	# biomimetic-front work.
	var eye_radius: float    = 0.010
	var eye_x_offset: float  = 0.020                  # ±X from chassis centerline
	var eye_y_offset: float  = 0.006                  # slightly above center
	var eye_z: float         = CHASSIS_Z * 0.5        # at +Z face, half-protruding
	var eye_mat := StandardMaterial3D.new()
	eye_mat.albedo_color = Color(0.05, 0.05, 0.05, 1.0)
	for side in [-1.0, 1.0]:
		var eye := MeshInstance3D.new()
		var em := SphereMesh.new()
		em.radius = eye_radius
		em.height = eye_radius * 2.0
		eye.mesh = em
		eye.position = Vector3(side * eye_x_offset, eye_y_offset, eye_z)
		eye.set_surface_override_material(0, eye_mat)
		_chassis.add_child(eye)

	add_child(_chassis)
	_chassis_rest_xform = _chassis.global_transform

	for i in range(4):
		_build_leg(i)
	# 2026-06-03 — verify G6DOF angular params reached the joints.  Bit-
	# identical-across-tweaks calibration sweep raised the question of
	# whether changes are reaching the body — this print is the receipt.
	# --- 2026-08-02 · IMPORT I6: PHYSICAL SCAFFOLD, NAMED AS A SCAFFOLD ---------------
	# Every Playful Machine legged experiment runs at reduced gravity: the dog, the
	# hexapod (zoo) and the humanoid all set ODE gravity to -6 against Earth's -9.81
	# (the snake uses -4), on rubber ground, with compliant passive distal joints.
	# Their emergence results are ALL measured under that scaffold; ours are measured
	# at 1.0 g with rigid legs, and we bought uprightness with control-layer terms
	# instead (postural_gain, height_homeo_gain, stance_lift_gain, balance_gain).
	#
	# This knob is the faithful analogue, and it exists to answer ONE question:
	# does the homeokinetic core produce a gait when the body is as forgiving as the
	# one PM's results come from?  It is a SCAFFOLD (docs: scaffold / de-scaffold) —
	# a diagnostic prop that must be named as such in any claim and removed before
	# any result stands on its own.  PM-equivalent value is 6/9.81 = 0.61.
	#
	# 1.0 = off = byte-identical to every historical run.
	if not is_equal_approx(scaffold_gravity_scale, 1.0):
		for b in ([_chassis] + (_coxas as Array) + (_uppers as Array) + (_lowers as Array)):
			b.gravity_scale = scaffold_gravity_scale
		print("PicrawlerBody: ⚠ SCAFFOLD ACTIVE — gravity_scale=%.3f (PM legged sims run 0.61)"
			% scaffold_gravity_scale)
	if joint_backend == "hinge":
		print("PicrawlerBody: joint_backend=hinge (legacy hobby-servo, historical baselines)  motor_force_scale=%.4f" % motor_force_scale)
	else:
		print("PicrawlerBody: joint_backend=g6dof (suspension)  damping=%.3f softness=%.3f erp=%.3f motor_force_scale=%.4f  springs: hip1=%.2f/%.2f hip2=%.2f/%.2f knee=%.2f/%.2f" % [
			joint_angular_damping, joint_angular_limit_softness, joint_angular_erp, motor_force_scale,
			hip1_spring_stiffness, hip1_spring_damping,
			hip2_spring_stiffness, hip2_spring_damping,
			knee_spring_stiffness, knee_spring_damping])
	_report_geometry()

# Hand the (re)built chassis to everything that tracks it by REFERENCE rather
# than by NodePath.  The chassis is built programmatically, so it isn't
# reachable via NodePath at scene-load time — these consumers are given the
# instance instead.
#
# ⚠ That makes them stale-able.  _rebuild_body() frees the old chassis, and a
# consumer still holding it has a freed instance: the orbit camera silently
# stopped responding to input after the first live body swap for exactly this
# reason.  Anything added here MUST be re-targeted on every rebuild, not just
# at _ready().
func _retarget_body_watchers() -> void:
	var cam: Node = get_tree().get_root().find_child("Camera3D", true, false)
	if cam != null and cam.has_method("set_target"):
		cam.call("set_target", _chassis)

	var trail: Node = get_tree().get_root().find_child("WalkingTrail", true, false)
	if trail != null and trail.has_method("set_target"):
		trail.call("set_target", _chassis)
		_walking_trail = trail

# The geometry receipt.  Printed once per build, right after the legs exist, so
# every run's log carries the evidence for gates G1–G3 rather than requiring a
# bespoke harness to check them (picrawler_sim2real_port.md §Validation gates).
# Same spirit as the G6DOF param print above: a claim you can read back.
func _report_geometry() -> void:
	if not is_instance_valid(_chassis):
		return
	var origin_y: float = _chassis.global_transform.origin.y
	var hip2_y:   float = origin_y - COXA_Z_DROP     # node origin sits at hip1

	# G1 — TWO different CoGs, and conflating them already cost one wrong mass
	# split.  The operator measures the CHASSIS assembly (legs off / not
	# contributing); the whole-body number is DERIVED and sits much lower
	# because 248 g of legs hang at ~-11 mm rel hip2.  Print both, always.
	var cog_y: float = _compute_body_cog_y()          # whole body, derived
	var chassis_cog_y: float = _chassis.global_transform.origin.y
	if _chassis_com_valid:
		chassis_cog_y = (_chassis.global_transform * _chassis_com).y

	# G2 — belly = lowest underside among the chassis boxes.
	var belly_local: float = -CHASSIS_Y * 0.5
	if not _chassis_boxes.is_empty():
		belly_local = INF
		for spec in _chassis_boxes:
			var sz: Vector3 = spec["size"]
			var off: Vector3 = spec["offset"]
			belly_local = min(belly_local, off.y - sz.y * 0.5)

	var reach: float = L1 + L2 + L3
	var knee_span_deg: float = rad_to_deg(KNEE_LIMIT_HIGH - KNEE_LIMIT_LOW)

	print("PicrawlerBody: GEOMETRY RECEIPT '%s'" % _geometry_name)
	print("  reach L1+L2+L3 = %.1f mm   (hip span %.0f x %.0f mm)"
		% [reach * 1000.0, HIP_X_SPAN * 1000.0, HIP_Z_SPAN * 1000.0])
	print("  G1 chassis CoG = %+.1f mm rel hip2   <- the MEASURED quantity"
		% [(chassis_cog_y - hip2_y) * 1000.0])
	print("     whole-body  = %+.1f mm rel hip2   (derived; legs pull it down)"
		% [(cog_y - hip2_y) * 1000.0])
	print("  G2 belly       = %+.1f mm rel hip2   (%.1f mm above floor at spawn)"
		% [(origin_y + belly_local - hip2_y) * 1000.0, (origin_y + belly_local) * 1000.0])
	print("  G3 knee span   = %.0f deg commanded-limit  (servo can do ~180)"
		% knee_span_deg)
	print("  mass total     = %.3f kg  (chassis %.3f + 4x%.3f legs)"
		% [_TOTAL_MASS, CHASSIS_MASS, COXA_MASS + UPPER_MASS + LOWER_MASS])
	print("  belly-up reset = chassis_y < %.3f m  (rests on top surface %+.1f mm rel origin)"
		% [_chassis_top_local + (auto_reset_max_height - _LEGACY_INVERTED_REST_H),
		   _chassis_top_local * 1000.0])

func _build_leg(leg_index: int) -> void:
	var heading: Vector3 = NEUTRAL_HEADINGS[leg_index]
	var lateral: Vector3 = Vector3.UP.cross(heading).normalized()

	# Hip1 anchor: chassis corner at half X-span, half Z-span.
	var sx: float = sign(heading.x)
	var sz: float = sign(heading.z)
	var hip1_local := Vector3(sx * HIP_X_SPAN * 0.5, 0.0, sz * HIP_Z_SPAN * 0.5)
	var hip1_world: Vector3 = _chassis.global_transform.origin + hip1_local

	# Hip2 anchor: from hip1, extend L1 outward in heading direction
	# with a 7mm vertical drop.
	var coxa_horiz: float = sqrt(max(0.0, L1*L1 - COXA_Z_DROP*COXA_Z_DROP))
	var coxa_dir: Vector3 = (heading * coxa_horiz + Vector3.DOWN * COXA_Z_DROP).normalized()
	var hip2_world: Vector3 = hip1_world + coxa_dir * L1

	# Coxa body — sphere at midpoint between hip1 and hip2.  Per-leg
	# color from LEG_COLORS lets the user identify each leg at a glance.
	var coxa_center: Vector3 = (hip1_world + hip2_world) * 0.5
	var leg_color: Color = LEG_COLORS[leg_index]
	var coxa := _make_segment(COXA_MASS, COXA_RADIUS, coxa_center, leg_color)
	_coxas.append(coxa)

	# Hip1 joint — Godot's HingeJoint3D uses local +Z as the hinge axis
	# (NOT local +X as Bullet's default).  Empirically verified: when
	# the Z column points along world UP, hip1 yaws correctly.
	# Right-handed basis with Z=UP, X=RIGHT, Y=BACK (=+Z direction).
	# RIGHT-HANDED basis with Z=UP (hinge axis).  X × Y must equal Z.
	# X=(1,0,0), Z=(0,1,0)=UP → Y must be (0,0,-1) so that X×Y = (0,1,0).
	# Previous Basis(RIGHT, (0,0,+1), UP) gave X×Y = (0,-1,0) = -Z → LEFT-handed.
	# Left-handed joint frames invert the constraint solver's sign on the
	# LOCKED axes (the hinge axis itself works, which is why yaw direction
	# was correct, but the perpendicular locks behave wrong → legs slide
	# in directions they shouldn't = "folding into the body").
	var hip1_basis: Basis = Basis(Vector3.RIGHT, Vector3(0, 0, -1), Vector3.UP)
	# 2026-06-02 — G6DOF migration.  Motor used as a JOINT-LEVEL damper:
	# target_velocity=0 + bounded force_limit means the joint resists
	# relative angular velocity, damping joint-chain resonance independently
	# of rigid-body angular_damp.  motor_force_scale applies to the damper
	# baseline the same way as to the per-tick PD force — keeps the two in
	# the same units regardless of which Bullet semantic FORCE_LIMIT has.
	var _damper_force: float = 0.5 * motor_force_scale
	var hip1 := _make_leg_joint(
			hip1_basis, hip1_world,
			_chassis.get_path(), coxa.get_path(), _chassis,
			-HIP1_LIMIT, HIP1_LIMIT, _damper_force,
			hip1_spring_stiffness, hip1_spring_damping)
	_hip1_joints.append(hip1)

	# Upper leg — capsule from hip2 outward along heading, length L2.
	# Visual capsule oriented along heading (horizontal outward) so it
	# spans hip2→knee instead of being a vertical stub at hip2.
	var upper_center: Vector3 = hip2_world + heading * (L2 * 0.5)
	# Slightly darker shade of leg_color for upper segment (visual depth).
	var upper := _make_capsule(UPPER_MASS, LEG_RADIUS, L2, upper_center,
							   leg_color.darkened(0.25), heading)
	_uppers.append(upper)

	# Hip2 joint — Godot HingeJoint3D hinge = local +Z column.  Set
	# Z = lateral so the hinge axis is the leg-local lateral direction.
	# Other columns: X = heading (leg outward), Y = UP-ish (heading×lateral).
	# Right-handedness check: X × Y must = Z.
	#   heading × (heading × lateral)  by BAC-CAB
	#   = heading*(heading·lateral) − lateral*(heading·heading)
	#   = 0 − lateral*1 = -lateral
	# That gives X × Y = -Z (left-handed).  Need Y = lateral × heading instead.
	var hip2_basis := Basis()
	hip2_basis.x = heading
	hip2_basis.y = lateral.cross(heading).normalized()    # right-handed (X×Y = Z = lateral)
	hip2_basis.z = lateral
	hip2_basis = hip2_basis.orthonormalized()
	var hip2 := _make_leg_joint(
			hip2_basis, hip2_world,
			coxa.get_path(), upper.get_path(), coxa,
			-HIP2_LIMIT, HIP2_LIMIT, _damper_force,
			hip2_spring_stiffness, hip2_spring_damping)
	_hip2_joints.append(hip2)
	_hip2_axes.append(lateral)

	# Lower leg — capsule from knee in lower_dir, length L3.
	# Knee bend takes the upper leg's outward heading and rotates it
	# around the knee axis (= lateral).  Sign of the rotation determines
	# whether the lower leg drops DOWN (standing) or up (inverted spawn).
	# Compute both candidates and pick the one with negative Y component
	# so we always end up with a downward-pointing lower leg.
	var knee_world: Vector3 = hip2_world + heading * L2
	var lower_down: Vector3 = heading.rotated(lateral, LOWER_LEG_DROP_ANGLE).normalized()
	var lower_up:   Vector3 = heading.rotated(lateral, -LOWER_LEG_DROP_ANGLE).normalized()
	var lower_dir: Vector3
	if lower_down.y < lower_up.y:
		lower_dir = lower_down
	else:
		lower_dir = lower_up
	print("PicrawlerBody: leg %d heading=%v lateral=%v lower_dir=%v" % [
		leg_index, heading, lateral, lower_dir])
	var lower_center: Vector3 = knee_world + lower_dir * (L3 * 0.5)
	# Darkest shade of leg_color for lower segment.
	var lower := _make_capsule(LOWER_MASS, LEG_RADIUS * 0.8, L3, lower_center,
							   leg_color.darkened(0.5), lower_dir)
	# Markov-compliant foot-contact sensor: enable true physics contact reporting
	# on the foot (lower leg) so we can sense TOUCH (= a hardware contact switch),
	# not an absolute-Y threshold.  Legs mask _LAYER_WORLD only, so any reported
	# collision is genuine ground/obstacle contact.
	lower.contact_monitor = true
	lower.max_contacts_reported = 4
	_lowers.append(lower)

	# Knee joint — same basis as hip2 (axis = leg-local lateral).
	var knee_lower: float = KNEE_LIMIT_LOW if knee_widening_enabled else KNEE_LIMIT_LOW_NARROW
	if leg_index == 0:
		# Diagnostic, once per body construction (printed for FL only):
		# confirm which limits/backend the knee actually sees.
		print("PicrawlerBody knee construction: backend=%s knee_widening_enabled=%s knee_lower=%.3f knee_upper=%.3f (KNEE_LIMIT_LOW=%.3f KNEE_LIMIT_LOW_NARROW=%.3f KNEE_LIMIT_HIGH=%.3f)" % [
			joint_backend, knee_widening_enabled, knee_lower, KNEE_LIMIT_HIGH,
			KNEE_LIMIT_LOW, KNEE_LIMIT_LOW_NARROW, KNEE_LIMIT_HIGH])
	var knee := _make_leg_joint(
			hip2_basis, knee_world,
			upper.get_path(), lower.get_path(), upper,
			knee_lower, KNEE_LIMIT_HIGH, _damper_force,
			knee_spring_stiffness, knee_spring_damping)
	# Knee-specific override: historical hinge baseline used LIMIT_SOFTNESS=0
	# on the knee (no soft-braking region inside the range, motor can drive
	# all the way to the configured limit).  Hip1/hip2 used the default 0.95.
	# Apply only in hinge mode — G6DOF angular softness is global via
	# joint_angular_limit_softness @export.
	if knee is HingeJoint3D:
		(knee as HingeJoint3D).set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS, 0.0)
		(knee as HingeJoint3D).set_param(HingeJoint3D.PARAM_LIMIT_BIAS, 0.3)
	_knee_joints.append(knee)
	_knee_axes.append(lateral)

	_coxa_rest_xform.append(coxa.global_transform)
	_upper_rest_xform.append(upper.global_transform)
	_lower_rest_xform.append(lower.global_transform)
	# Construction-time joint anchor positions — used by calibration FK.
	_hip1_world_c.append(hip1_world)
	_hip2_world_c.append(hip2_world)
	_knee_world_c.append(knee_world)

# 2026-06-03 — Backend-aware leg-joint factory.  Builds the appropriate
# joint type for the current joint_backend setting:
#
#   "hinge" — HingeJoint3D, original hobby-servo behaviour, used to
#             reproduce all historical (pre-2026-06-02) baselines.
#             motor_force is converted to MAX_IMPULSE (Nm·s/step) by
#             dividing by physics_hz inside this helper, so callers can
#             pass a torque (Nm) in either backend mode and get the
#             same effective motor authority.
#
#   "g6dof" — Generic6DOFJoint3D configured as a hinge around the
#             joint's local +Z axis (5 of 6 DOF locked).  Supports
#             passive angular spring on the free axis.  See
#             [[v6-apply_torque_spring_falsified]] for why we needed
#             the physics-solver path.
#
# spring_stiffness > 0 enables the angular spring (G6DOF only; ignored
# in hinge mode).  Equilibrium fixed at 0 rad = the joint's neutral
# pose.  Backwards-compatible: spring_stiffness=0 → rigid hinge in
# both backends.
func _make_leg_joint(
		basis: Basis,
		world_pos: Vector3,
		node_a_path: NodePath,
		node_b_path: NodePath,
		parent: Node,
		lower: float,
		upper: float,
		motor_force: float,
		spring_stiffness: float,
		spring_damping: float) -> Object:
	if joint_backend == "hinge":
		return _make_hinge_joint(basis, world_pos, node_a_path, node_b_path,
				parent, lower, upper, motor_force)
	return _make_g6dof_joint(basis, world_pos, node_a_path, node_b_path,
			parent, lower, upper, motor_force, spring_stiffness, spring_damping)

func _make_hinge_joint(
		basis: Basis,
		world_pos: Vector3,
		node_a_path: NodePath,
		node_b_path: NodePath,
		parent: Node,
		lower: float,
		upper: float,
		motor_force: float) -> HingeJoint3D:
	var j := HingeJoint3D.new()
	parent.add_child(j)
	j.global_transform = Transform3D(basis, world_pos)
	j.set_node_a(node_a_path)
	j.set_node_b(node_b_path)
	j.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, upper)
	j.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, lower)
	# Bullet defaults that the historical baseline relies on.  SOFTNESS=0
	# was knee-specific (set after construction in the old _build_leg);
	# default 0.95 here matches hip1/hip2 history.
	j.set_param(HingeJoint3D.PARAM_LIMIT_SOFTNESS, 0.95)
	j.set_param(HingeJoint3D.PARAM_LIMIT_BIAS, 0.2)
	j.set_param(HingeJoint3D.PARAM_LIMIT_RELAXATION, 1.0)
	j.set_flag(HingeJoint3D.FLAG_USE_LIMIT, true)
	j.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	j.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, 0.0)
	# HingeJoint3D's MAX_IMPULSE is Nm·s/step — caller passes torque (Nm),
	# we divide by physics_hz to keep effective torque equivalent across
	# backends.
	j.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, motor_force / float(physics_hz))
	return j

func _make_g6dof_joint(
		basis: Basis,
		world_pos: Vector3,
		node_a_path: NodePath,
		node_b_path: NodePath,
		parent: Node,
		lower: float,
		upper: float,
		motor_force: float,
		spring_stiffness: float,
		spring_damping: float) -> Generic6DOFJoint3D:
	# CRITICAL: HingeJoint3D uses local +Z as the hinge axis (per the
	# existing comment in _build_leg's hip1 block), but Bullet's
	# btGeneric6DofConstraint uses local +X as the TWIST axis with Y/Z
	# being SWING axes.  Swing limits in Bullet's angular limit code are
	# mathematically capped near ±π/2 — that's the 90° clamp Joseph
	# observed on the knee in G6DOF mode.  The twist axis (X) has the
	# full ±π range available.
	#
	# Fix: remap the incoming basis (which has the hinge axis on +Z to
	# match HingeJoint3D convention) so that the hinge axis ends up on
	# local +X for G6DOF.  Right-handed permutation:
	#   new_X = old_Z   (was the hinge axis; now twist)
	#   new_Y = old_Y   (preserves one perpendicular)
	#   new_Z = -old_X  (= old_Z × old_Y, keeps X × Y = Z)
	# Then lock Y and Z (the swing axes) at 0, and limit/motor the X
	# (twist) axis.
	var g6_basis: Basis = Basis(basis.z, basis.y, -basis.x)
	var j := Generic6DOFJoint3D.new()
	parent.add_child(j)
	j.global_transform = Transform3D(g6_basis, world_pos)
	j.set_node_a(node_a_path)
	j.set_node_b(node_b_path)
	# Lock all 3 linear DOF (lower=upper=0 with limit enabled).
	j.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	j.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	j.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	j.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	j.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	j.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	j.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	j.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	j.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	# Lock angular Y and Z (swing axes — no rotation around them).
	j.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
	j.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
	j.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	j.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
	j.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
	j.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	# Free angular X axis (twist): limits + motor + optional spring.
	# Twist axis supports the full ±π range without the swing-limit clamp.
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, lower)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, upper)
	j.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LIMIT_SOFTNESS, joint_angular_limit_softness)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_DAMPING, joint_angular_damping)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_RESTITUTION, 0.0)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_ERP, joint_angular_erp)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
	j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, motor_force)
	j.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
	if spring_stiffness > 0.0:
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, spring_stiffness)
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, spring_damping)
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, 0.0)
		j.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_SPRING, true)
	# Diagnostic: confirm the X-axis (twist) limit values that actually
	# reached the constraint solver.  If knee shows ±0.85 / +1.6 here but
	# UI still maxes at ±π/2, the issue is downstream in Bullet's twist
	# handling, not in our limit values.
	print("G6DOF joint: x_lower=%.3f x_upper=%.3f motor_force=%.3f spring_k=%.2f spring_d=%.2f basis.x=%s" % [
		lower, upper, motor_force, spring_stiffness, spring_damping, str(g6_basis.x)])
	return j

# 2026-06-03 — G6DOF backend default preset.  Joseph hand-tuned 2026-06-03:
# first set of values that produce a stable compliant standing pose on
# G6DOF joints (stance similar to original hinge body).  Applied as the
# *default* (not an override) when joint_backend=g6dof — env vars,
# ExperimentConfig launcher metadata, and curriculum overrides all still
# win because they run AFTER this baseline is laid down.
#
# Also persisted as a loadable curriculum at:
#   res://curricula/picrawler_g6dof_compliant_stand_handtuned.json
#
# Hinge mode is untouched by this — its defaults remain the historical
# values so pre-2026-06-02 baselines reproduce.
const _G6DOF_DEFAULT_PRESET: Dictionary = {
	"motor_force_scale":            2.0,
	"motor_authority_scale":        1.5,
	"motor_damping_factor":         1.0,
	"joint_angular_damping":        1.5,
	"joint_angular_erp":            1.0,
	"joint_angular_limit_softness": 1.0,
	"hip1_spring_stiffness":        8.0,
	"hip1_spring_damping":          1.5,
	"hip2_spring_stiffness":        7.0,
	"hip2_spring_damping":          1.5,
	"knee_spring_stiffness":        7.0,
	"knee_spring_damping":          1.5,
	# 2026-06-13 — freeplay deadband pairs with the springs for the compliant
	# stand the Motor-EPM nav was validated on (operator's g6dof + freeplay-0.1
	# substrate).  Part of the preset so a g6dof launch reproduces it.
	"motor_freeplay_rad":           0.1,
}

# Env var that owns each preset key.  If it is set, the preset must NOT overwrite it.
const _G6DOF_PRESET_ENV: Dictionary = {
	"motor_force_scale":            "OGMA_PICRAWLER_MOTOR_FORCE_SCALE",
	"motor_authority_scale":        "OGMA_PICRAWLER_MOTOR_AUTHORITY",
	"motor_damping_factor":         "OGMA_PICRAWLER_MOTOR_DAMP",
	"joint_angular_damping":        "OGMA_PICRAWLER_JOINT_DAMPING",
	"joint_angular_erp":            "OGMA_PICRAWLER_JOINT_ERP",
	"joint_angular_limit_softness": "OGMA_PICRAWLER_JOINT_SOFTNESS",
	"hip1_spring_stiffness":        "OGMA_PICRAWLER_HIP1_SPRING_STIFFNESS",
	"hip1_spring_damping":          "OGMA_PICRAWLER_HIP1_SPRING_DAMPING",
	"hip2_spring_stiffness":        "OGMA_PICRAWLER_HIP2_SPRING_STIFFNESS",
	"hip2_spring_damping":          "OGMA_PICRAWLER_HIP2_SPRING_DAMPING",
	"knee_spring_stiffness":        "OGMA_PICRAWLER_KNEE_SPRING_STIFFNESS",
	"knee_spring_damping":          "OGMA_PICRAWLER_KNEE_SPRING_DAMPING",
	"motor_freeplay_rad":           "OGMA_PICRAWLER_MOTOR_FREEPLAY",
}

func _apply_g6dof_default_preset() -> void:
	# ⚠ 2026-08-03 — the preset used to overwrite EVERY key unconditionally.  Its own
	# comment claimed "env vars still win because they run AFTER this baseline", but the
	# env whitelist in _resolve_env() runs BEFORE this call, so the preset was silently
	# clobbering every env override of a preset key.  Discovered when an
	# OGMA_PICRAWLER_JOINT_DAMPING sweep produced four identical arms — the runs all
	# reported damping=1.500 regardless of what was requested.  Any past headless sweep
	# over a preset key on g6dof measured NOTHING and would have read as a clean null.
	var skipped: Array = []
	for key in _G6DOF_DEFAULT_PRESET:
		var envk: String = str(_G6DOF_PRESET_ENV.get(key, ""))
		if envk != "" and OS.get_environment(envk) != "":
			skipped.append(key)
			continue                       # env owns this one
		set(key, _G6DOF_DEFAULT_PRESET[key])
	if not skipped.is_empty():
		print("PicrawlerBody: G6DOF preset yielded to env for: %s" % ", ".join(skipped))
	print("PicrawlerBody: applied G6DOF default preset (hand-tuned compliant stand) — env / config / curriculum can still override")

# 2026-06-03 — Backend-aware per-tick motor setter.  Caller always
# passes torque (Nm); helper does the per-backend unit conversion.
func _set_motor_vf(j: Object, vel: float, force: float) -> void:
	if j is HingeJoint3D:
		(j as HingeJoint3D).set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, vel)
		(j as HingeJoint3D).set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, force / float(physics_hz))
	else:
		# G6DOF: hinge axis remapped to local +X (twist) in _make_g6dof_joint.
		(j as Generic6DOFJoint3D).set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, vel)
		(j as Generic6DOFJoint3D).set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, force)

# 2026-06-02 — Apply current spring @export values to all 12 leg joints.
# Called at startup (after _build_leg) and from _on_curriculum_stage_changed
# so curriculum overrides take effect without rebuilding the body.
# stiffness=0 → spring disabled (back-compat rigid hinge); stiffness>0 →
# Hooke spring active on the free axis with equilibrium at 0 rad.
func _apply_joint_springs() -> void:
	# Springs only exist on G6DOF joints — no-op in hinge mode.
	if joint_backend == "hinge":
		return
	for j in _hip1_joints:
		_apply_spring_to(j, hip1_spring_stiffness, hip1_spring_damping)
	for j in _hip2_joints:
		_apply_spring_to(j, hip2_spring_stiffness, hip2_spring_damping)
	for j in _knee_joints:
		_apply_spring_to(j, knee_spring_stiffness, knee_spring_damping)

# ---- 2026-08-03 · GANGED JOINT DRIVE (G mode only) ----------------------------------
# Called once per physics tick BEFORE the servo targets are read, so it writes straight
# into servo_targets[] and rides the normal PD/freeplay/spring path — i.e. the operator
# is characterising the REAL actuator chain, not a bypass.
#   PULSE  : step the group by amp, hold N ticks, release → the ring-down after release
#            gives the damping ratio (successive peak ratio) and the natural frequency.
#   SHAKE  : sinusoid at shake_hz → sweep it and watch the peak-to-peak readout; the
#            frequency where pp is maximal for a fixed drive amplitude IS the resonance.
# Indices follow the servo_targets layout: 0-3 hip1, 4-7 hip2, 8-11 knee.
func _tick_gang_drive() -> void:
	if not _motor_test_mode:
		return
	var hip2_extra: float = 0.0
	var knee_extra: float = 0.0
	if _gang_pulse_ticks > 0:
		_gang_pulse_ticks -= 1
		if _gang_pulse_group == 1 or _gang_pulse_group == 3: hip2_extra += _gang_pulse_amp
		if _gang_pulse_group == 2 or _gang_pulse_group == 3: knee_extra += _gang_pulse_amp
	if _gang_shake_hz > 0.0 and _gang_shake_amp > 0.0:
		_gang_shake_phase += 2.0 * PI * _gang_shake_hz * TAU
		if _gang_shake_phase > 2.0 * PI: _gang_shake_phase -= 2.0 * PI
		var sv: float = sin(_gang_shake_phase) * _gang_shake_amp
		if _gang_pulse_group == 1 or _gang_pulse_group == 3: hip2_extra += sv
		if _gang_pulse_group == 2 or _gang_pulse_group == 3: knee_extra += sv
	# ⚠ USE servo_idx().  Two different 12-element layouts live in this file: the proprio
	# publish is JOINT-MAJOR (hip1x4, hip2x4, kneex4) while servo_targets is LEG-MAJOR
	# (leg*3 + joint).  Open-coding "4+i"/"8+i" here drove hip1 on the wrong legs — the
	# operator caught it as "hip2 base is moving rear-right hip1".
	for i in range(4):
		servo_targets[servo_idx(i, 1)] = _gang_hip2_base + hip2_extra   # hip2
		servo_targets[servo_idx(i, 2)] = _gang_knee_base + knee_extra   # knee
	# Rolling peak-to-peak of the MEASURED angles — the response, not the command.
	# Leg 0 is representative; the gang drives all four identically.  Measured from the
	# joint geometry with the same helper the physics step uses (hip2_angles/knee_angles
	# are locals there, not members, so they cannot be read from here).
	if _coxas.size() > 0 and _uppers.size() > 0 and _lowers.size() > 0:
		var a2: float = _relative_angle_world_axis(_coxas[0], _uppers[0], _hip2_axes[0])
		var ak: float = _relative_angle_world_axis(_uppers[0], _lowers[0], _knee_axes[0])
		_gang_win_hi2 = maxf(_gang_win_hi2, a2); _gang_win_lo2 = minf(_gang_win_lo2, a2)
		_gang_win_hik = maxf(_gang_win_hik, ak); _gang_win_lok = minf(_gang_win_lok, ak)
	_gang_win_n += 1
	if _gang_win_n >= 100:                      # ~2 s window at 50 Hz
		_gang_pp_hip2 = _gang_win_hi2 - _gang_win_lo2
		_gang_pp_knee = _gang_win_hik - _gang_win_lok
		_gang_win_hi2 = -9.0; _gang_win_lo2 = 9.0
		_gang_win_hik = -9.0; _gang_win_lok = 9.0
		_gang_win_n = 0

func gang_pulse(amp: float, hold_ticks: int, group: int) -> void:
	_gang_pulse_amp = amp
	_gang_pulse_ticks = max(1, hold_ticks)
	_gang_pulse_group = group
	print("PicrawlerBody: GANG PULSE amp=%+.3f hold=%d group=%d — watch the ring-down"
		% [amp, hold_ticks, group])

# Error BEYOND the freeplay band.  Zero inside it (motor released); outside, the residual
# past the boundary — so the motor arrests the joint AT the slop edge rather than driving
# it back to target and re-entering the band.  freeplay 0 ⇒ returns err unchanged, so the
# no-freeplay path stays bit-identical to the pre-2026-08-03 behaviour.
# Soft restoring command INSIDE the freeplay band — the spring, implemented through the
# motor because the constraint spring is inert in Godot Physics 3D.
#   stiffness → fraction of full servo authority applied toward the commanded angle
#   damping   → fraction of the force limit, so a stiffer-but-undamped joint can ring
# Returns [velocity, max_impulse].  stiffness 0 ⇒ [0, 0] = genuinely free, so the
# all-zero ragdoll case is bit-identical to before this change.
const SPRING_K_FULL: float = 20.0     # slider max == full servo authority
func _soft_spring_cmd(err: float, stiffness: float, damping: float, full_imp: float) -> Array:
	if stiffness <= 0.0:
		return [0.0, 0.0]
	var k: float = clampf(stiffness / SPRING_K_FULL, 0.0, 1.0)
	var v: float = -clamp(SERVO_KP * k * err, -MAX_SERVO_SPEED, MAX_SERVO_SPEED)
	# Damping rides the force limit: low damping = the spring can overshoot and ring,
	# high damping = it settles.  Floor keeps a stiff/undamped spring still able to act.
	var d: float = clampf(0.25 + 0.75 * (damping / 5.0), 0.0, 1.0)
	return [v, full_imp * k * d]

func _freeplay_err(err: float) -> float:
	if motor_freeplay_rad <= 0.0:
		return err
	if absf(err) <= motor_freeplay_rad:
		return 0.0
	return err - signf(err) * motor_freeplay_rad

func _set_spring_equilibrium(j: Object, target_rad: float) -> void:
	if j is Generic6DOFJoint3D:
		(j as Generic6DOFJoint3D).set_param_x(
			Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, target_rad)

func _apply_spring_to(j: Generic6DOFJoint3D, stiffness: float, damping: float) -> void:
	# Spring lives on the X axis (twist) since _make_g6dof_joint remaps the
	# hinge axis there.
	if stiffness > 0.0:
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, stiffness)
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, damping)
		j.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, 0.0)
		j.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_SPRING, true)
	else:
		j.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_SPRING, false)

func _make_segment(mass: float, radius: float, center: Vector3, color: Color) -> RigidBody3D:
	var b := RigidBody3D.new()
	b.mass = mass
	b.transform = Transform3D(Basis.IDENTITY, center)
	b.collision_layer = _LAYER_BODY
	b.collision_mask  = _LAYER_WORLD
	b.angular_damp = BODY_ANGULAR_DAMP * body_damp_scale
	b.linear_damp  = BODY_LINEAR_DAMP * body_damp_scale
	b.physics_material_override = _make_contact_mat()
	var cs := CollisionShape3D.new()
	var sp := SphereShape3D.new()
	sp.radius = radius
	cs.shape = sp
	b.add_child(cs)
	var ms := MeshInstance3D.new()
	var sm := SphereMesh.new()
	sm.radius = radius
	sm.height = radius * 2.0
	ms.mesh = sm
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	ms.set_surface_override_material(0, mat)
	b.add_child(ms)
	add_child(b)
	return b

func _make_capsule(mass: float, radius: float, height: float, center: Vector3,
				   color: Color, long_axis_dir: Vector3 = Vector3.UP) -> RigidBody3D:
	# The rigid body's basis stays IDENTITY (joints depend on it).  We
	# rotate the MESH + COLLISION children so the capsule's long axis
	# visually aligns with `long_axis_dir`.  CapsuleMesh/Shape3D default
	# along local +Y, so we rotate +Y → long_axis_dir.
	var b := RigidBody3D.new()
	b.mass = mass
	b.transform = Transform3D(Basis.IDENTITY, center)
	b.collision_layer = _LAYER_BODY
	b.collision_mask  = _LAYER_WORLD
	b.angular_damp = BODY_ANGULAR_DAMP * body_damp_scale
	b.linear_damp  = BODY_LINEAR_DAMP * body_damp_scale
	b.physics_material_override = _make_contact_mat()
	var orient: Basis = Basis.IDENTITY
	if long_axis_dir.distance_to(Vector3.UP) > 0.001:
		orient = Basis(Quaternion(Vector3.UP, long_axis_dir.normalized()))
	var cs := CollisionShape3D.new()
	var cap := CapsuleShape3D.new()
	cap.radius = radius
	cap.height = height
	cs.shape = cap
	cs.transform.basis = orient
	b.add_child(cs)
	var ms := MeshInstance3D.new()
	var cm := CapsuleMesh.new()
	cm.radius = radius
	cm.height = height
	ms.mesh = cm
	ms.transform.basis = orient
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	ms.set_surface_override_material(0, mat)
	b.add_child(ms)
	add_child(b)
	return b

# Lock 5 of 6 DOFs on a Generic6DOFJoint3D, leaving one angular axis free.
func _lock_six_dof(joint: Generic6DOFJoint3D, free_axis: String,
				   ang_low: float, ang_high: float) -> void:
	# Lock all linear axes
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	# Enable angular limits on all three axes; lock 2, free 1.
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	var lows  := {"X": 0.0, "Y": 0.0, "Z": 0.0}
	var highs := {"X": 0.0, "Y": 0.0, "Z": 0.0}
	lows[free_axis]  = ang_low
	highs[free_axis] = ang_high
	joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, lows["X"])
	joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, highs["X"])
	joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, lows["Y"])
	joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, highs["Y"])
	joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, lows["Z"])
	joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, highs["Z"])

# ---------------------------------------------------------------------------
# Physics loop
# ---------------------------------------------------------------------------
func _input(event: InputEvent) -> void:
	# Mouse-guided teleport placement — handled BEFORE the key-only gate below.
	# Move the mouse to slide the floor marker; left-click drops the robot there.
	if _place_mode:
		if event is InputEventMouseMotion:
			var g = _place_ground_from_mouse()
			if g != null:
				_place_target = g
				if _place_marker != null:
					_place_marker.global_position = g + Vector3(0, 0.012, 0)   # float just above the surface
			get_viewport().set_input_as_handled()
			return
		elif event is InputEventMouseButton and event.pressed \
				and event.button_index in [MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT]:
			# LEFT = drop upright, RIGHT = drop INVERTED.  Inversion recovery is a thing we
			# probe constantly (does it self-right, and does the walk survive afterwards),
			# and it used to require setting an env var and restarting.  Now it is a click.
			# Defer the actual drop to the physics step (input-frame writes get clobbered).
			var inverted: bool = (event.button_index == MOUSE_BUTTON_RIGHT)
			_pending_teleport = _place_target
			_pending_teleport_flip = 1 if inverted else 0
			_ui_notify("[place] dropping %s at (%.1f, %.1f)" % [
				"INVERTED" if inverted else "upright", _place_target.x, _place_target.z])
			_set_place_mode(false)
			get_viewport().set_input_as_handled()
			return
	# R toggle on _input (fires before any Control consumer).  SPACE on
	# _unhandled_input below (matches quadruped's working pattern; R via
	# _input + SPACE via _input together broke after the first reset —
	# something downstream was eating the second SPACE press).
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	var key: int = (event as InputEventKey).keycode
	# 2026-08-03 — live body-damping sweep, [ = less damped, ] = more damped.
	# PM's dog runs dampingFactor=0; ours is damped at three levels.  ⚠ THIS IS A
	# DIAGNOSTIC, NOT A LEVER: the damping models real hobby-servo gear-train inertia
	# that the physical robot genuinely has, so turning it down is a sim cheat of the
	# same class as an external anti-fall prop.  It is here to FEEL the coupling
	# between plant damping and the homeokinetic loop, not to be promoted.
	if key == KEY_BRACKETLEFT or key == KEY_BRACKETRIGHT:
		var steps_: Array = [1.0, 0.5, 0.25, 0.1, 0.05]
		var idx: int = 0
		var best: float = 1e9
		for i in range(steps_.size()):
			var dd: float = absf(float(steps_[i]) - body_damp_scale)
			if dd < best: best = dd; idx = i
		idx = clampi(idx + (1 if key == KEY_BRACKETLEFT else -1), 0, steps_.size() - 1)
		self.body_damp_scale = float(steps_[idx])
		_ui_notify("body_damp x%.2f  (DIAGNOSTIC, not a lever — models real servo inertia)"
			% body_damp_scale)
		return
	if key == KEY_R:
		_ragdoll_mode = not _ragdoll_mode
		# Wake / allow-sleep on all bodies so motors actually respond
		# when ragdoll is disabled (same issue as motor_test: sleeping
		# bodies ignore joint motor commands).  Also reset PD state so
		# the servo loop starts from zero error, not from stale brain-
		# accumulated _eff_target values that would slam joints.
		for b in [_chassis] + (_coxas as Array) + (_uppers as Array) + (_lowers as Array):
			b.can_sleep = _ragdoll_mode      # allow sleep in ragdoll; prevent in powered
			if not _ragdoll_mode:
				b.sleeping = false
		if not _ragdoll_mode:
			for i in range(4):
				_eff_target_hip1[i] = 0.0
				_eff_target_hip2[i] = 0.0
				_eff_target_knee[i] = 0.0
				_prev_torque_hip1[i] = 0.0
				_prev_torque_hip2[i] = 0.0
				_prev_torque_knee[i] = 0.0
				for jt in [_hip1_joints[i], _hip2_joints[i], _knee_joints[i]]:
					jt.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		print("PicrawlerBody: ragdoll_mode = %s  can_sleep=%s" % [_ragdoll_mode, _ragdoll_mode])
	# (N/P/A keys removed — panel sliders replace per-step sweep)
	elif key == KEY_C:
		_calibrate_mode = not _calibrate_mode
		_calibrate_step = 0
		_calibrate_step_start_tick = tick_counter
		print("PicrawlerBody: calibrate_mode = %s (auto-sweeps %d servo positions, ~%.1fs)" % [
			_calibrate_mode, SERVO_CALIBRATE_SEQUENCE.size(),
			SERVO_CALIBRATE_SEQUENCE.size() * SERVO_CALIBRATE_HOLD_TICKS * TAU])
		# Queue chassis state change — actual freeze/unfreeze happens at
		# the top of next physics frame (transform writes from input-event
		# handler get clobbered by the next physics step otherwise).
		_pending_chassis_freeze = 1 if _calibrate_mode else 2
		print("PicrawlerBody: chassis %s queued (will apply on next physics tick)" % [
			"SUSPEND" if _calibrate_mode else "UNFREEZE"])
	elif key == KEY_G:
		# G-mode: chassis DYNAMIC, feet on the floor, servo PD drives
		# each joint toward its slider value.  Sliders behave like real
		# hobby-servo position commands (slew-limited tracking with
		# first-order torque lag).  Difference from C: C freezes the
		# chassis and FK-writes legs for pure geometry validation; G
		# leaves everything dynamic so motors actually fight gravity
		# and floor reaction.
		_motor_test_mode = not _motor_test_mode
		# Slick feet so hip1 yaw isn't blocked by floor friction; zero
		# body damping so motor impulse isn't bled off before it
		# produces angular velocity.
		var mat: PhysicsMaterial = _make_slick_mat() if _motor_test_mode else _make_contact_mat()
		for lower in _lowers:
			lower.physics_material_override = mat
		var damp_value: float = 0.0 if _motor_test_mode else BODY_ANGULAR_DAMP
		var ldamp_value: float = 0.0 if _motor_test_mode else BODY_LINEAR_DAMP
		_chassis.angular_damp = damp_value
		_chassis.linear_damp = ldamp_value
		for b in _coxas + _uppers + _lowers:
			b.angular_damp = damp_value
			b.linear_damp = ldamp_value
		# Keep bodies awake — the constraint solver skips sleeping pairs,
		# which silently kills motor commands.
		for b in [_chassis] + (_coxas as Array) + (_uppers as Array) + (_lowers as Array):
			b.can_sleep = not _motor_test_mode
			if _motor_test_mode:
				b.sleeping = false
		# Sync _eff_target + torque history to the CURRENT measured joint
		# state so the first powered tick has zero PD error.  Without
		# this, stale brain-accumulated targets fire on entry and slam
		# joints into limits — the classic "binary at extremes" symptom.
		if _motor_test_mode:
			for i in range(4):
				_eff_target_hip1[i] = _relative_angle_world_axis(_chassis, _coxas[i], Vector3.UP)
				_eff_target_hip2[i] = _relative_angle_world_axis(_coxas[i], _uppers[i], _hip2_axes[i])
				_eff_target_knee[i] = _relative_angle_world_axis(_uppers[i], _lowers[i], _knee_axes[i])
				_prev_torque_hip1[i] = 0.0
				_prev_torque_hip2[i] = 0.0
				_prev_torque_knee[i] = 0.0
				for jt in [_hip1_joints[i], _hip2_joints[i], _knee_joints[i]]:
					jt.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)
		print("PicrawlerBody: motor_test_mode = %s  (dynamic chassis + servo PD, slick feet, damp=%.1f)" % [
			_motor_test_mode, damp_value])
		# 2026-06-07 — refresh the target pyramid color on mode change so the
		# active target stays purple across calibration ↔ experiment transitions.
		_ensure_target_pyramid_color()
	elif key == KEY_F5:
		# Save full brain state snapshot to user://saved_states/.  Auto-named
		# with timestamp + current config basename so it's easy to find later.
		var info: Dictionary = _save_brain_state_info("")
		if info.get("ok", false):
			var msg: String = "[F5] SAVED %d modules → %s  (%s)" % [
				info["module_count"],
				String(info["path"]).get_file(),
				_fmt_bytes(int(info["size"])),
			]
			print("PicrawlerBody: " + msg)
			_ui_notify(msg)
		else:
			var fail: String = "[F5] SAVE FAILED: %s" % info.get("error", "unknown")
			push_warning("PicrawlerBody: " + fail)
			_ui_notify(fail)
	elif key == KEY_F9:
		# Load the most-recent saved state.  Skips silently if none found.
		var load_info: Dictionary = _load_most_recent_brain_state_info()
		if load_info.get("ok", false):
			var fail_count: int = int(load_info.get("failed_count", 0))
			var warn: String = "" if fail_count == 0 else "  (FAILED %d!)" % fail_count
			var match_suffix: String = ""
			if not load_info.get("config_match", true):
				match_suffix = "  ⚠ CONFIG MISMATCH: saved=%s vs current=%s" % [
					load_info.get("saved_base", "?"),
					load_info.get("current_base", "?"),
				]
			var msg: String = "[F9] LOADED %d modules ← %s%s%s" % [
				int(load_info["restored_count"]),
				String(load_info["path"]).get_file(),
				match_suffix,
				warn,
			]
			print("PicrawlerBody: " + msg)
			_ui_notify(msg, 360)   # 6 sec for mismatch — easy to read
		else:
			var fail: String = "[F9] LOAD FAILED: %s" % load_info.get("error", "no saved states found")
			push_warning("PicrawlerBody: " + fail)
			_ui_notify(fail)
	elif key == KEY_T:
		# 2026-06-09 — toggle the picrawler HUD panels.  Joseph QoL: when
		# observing the body in flight he wants clean visual access to the
		# 3D scene without losing the panels for good.  Toggling is global —
		# re-press T to bring them back.  HUD hint line displays current state.
		# (2026-07-18 — reward / trainer / curriculum panels removed as RL cruft.)
		_panels_hidden = not _panels_hidden
		for panel_name in ["MotorEpmPanel"]:
			var p: Node = get_tree().get_root().find_child(panel_name, true, false)
			if p != null and p is Control:
				(p as Control).visible = not _panels_hidden
		print("PicrawlerBody: [T] panels_hidden = %s" % _panels_hidden)
	elif key == KEY_COMMA or key == KEY_PERIOD or key == KEY_SLASH:
		# ---- 2026-08-05 · TIME SCALE (slow-motion + turbo) ------------------------------
		# Engine.time_scale changes how many FIXED-timestep physics ticks advance per wall
		# second — never what happens inside a tick.  So the robot walks the identical
		# trajectory and a UI observation stays comparable to a headless run.  This is the
		# whole reason to use time_scale and NOT Engine.physics_ticks_per_second, which
		# would change the integration rate and hand you a different body (the same class
		# of mistake as the launcher's g6dof default).
		#
		# ⚠ THE TWO DIRECTIONS ARE NOT EQUALLY SAFE, and the asymmetry is worth knowing:
		#   SLOW (<1) is exact.  Fewer ticks per frame, nothing to drop.
		#   TURBO (>1) needs MORE physics steps per rendered frame, and Godot caps that at
		#     max_physics_steps_per_frame to avoid a spiral of death.  Past the cap the
		#     engine does NOT corrupt physics — it simply stops going faster and the sim
		#     falls behind wall clock.  So turbo has a ceiling, and the honest way to run
		#     the sim "as fast as the system allows" is still headless (what seedavg does).
		#   We raise the cap while in turbo so the knob does something before it saturates.
		var idx: int = _TIME_SCALES.find(_time_scale_v)
		if idx < 0: idx = _TIME_SCALES.find(1.0)
		if key == KEY_SLASH: idx = _TIME_SCALES.find(1.0)
		elif key == KEY_COMMA: idx = maxi(0, idx - 1)
		else: idx = mini(_TIME_SCALES.size() - 1, idx + 1)
		_time_scale_v = _TIME_SCALES[idx]
		Engine.time_scale = _time_scale_v
		Engine.max_physics_steps_per_frame = 8 if _time_scale_v <= 1.0 else int(clampf(_time_scale_v * 8.0, 8.0, 128.0))
		print("PicrawlerBody: time_scale = %.2fx (%s)" % [_time_scale_v,
			"real time" if is_equal_approx(_time_scale_v, 1.0)
			else ("SLOW-MO — exact, nothing dropped" if _time_scale_v < 1.0
				  else "TURBO — capped by max_physics_steps_per_frame")])
		_ui_notify("time %.2fx" % _time_scale_v)
	elif key == KEY_J:
		# 2026-08-04 — flip chassis collision live.  DIAGNOSTIC: every number in the ledger
		# was measured with this OFF, so an A/B taken across a mid-run flip is not comparable
		# to anything historical.  Restart for a clean arm.
		chassis_collides = not chassis_collides
		_apply_chassis_collision()
	elif key == KEY_K:
		# 2026-08-04 — toggle the ABLATION bench (break a servo / take a limb off, live).
		# It is destructive, so it boots COLLAPSED and this is the only way to open it.
		var ap: Node = get_tree().get_root().find_child("AblationPanel", true, false)
		if ap != null and ap.has_method("_on_minimise"):
			ap._on_minimise()
			print("PicrawlerBody: [K] ablation panel toggled")
	elif key == KEY_H:
		# 2026-06-09 — toggle the main HUD (calibration label + diagnostic
		# text) while keeping the hint line visible.  The quadruped_hud
		# script polls body.hud_hidden each frame and renders only the
		# hint line when true.
		hud_hidden = not hud_hidden
		print("PicrawlerBody: [H] hud_hidden = %s" % hud_hidden)
	elif key == KEY_P:
		# 2026-07-23 — toggle the walking-path trail (red X every metre travelled,
		# both gyms).  Node3D.visible hides the node + all spawned marks at once.
		var t := _walking_trail as Node3D
		if t != null and is_instance_valid(t):
			var shown: bool = t.call("toggle_shown")
			print("PicrawlerBody: [P] path_trail = %s" % shown)
	elif key == KEY_F1:
		# 2026-07-27 — mark the last ~15 s as GOOD: "this is the gait I want."
		# F-keys because G/C/T/H/P/M/N/V/R/1-4/SPACE are all taken.
		_clip_save("GOOD")
	elif key == KEY_F2:
		# ...and BAD: "this is the failure."  clipdiff.py then reports what actually
		# differs between the two sets, which is the part words keep getting wrong.
		_clip_save("BAD")
	elif key == KEY_F3:
		# Drop the most recent clip (mis-timed press).  Deletes by index rather than by
		# "latest file" so a concurrent run's directory can never be touched.
		if _clip_seq > 0 and _clip_dir != "":
			var d: DirAccess = DirAccess.open(_clip_dir)
			if d != null:
				for fn in d.get_files():
					if fn.begins_with("clip_%02d_" % (_clip_seq - 1)):
						d.remove(fn)
						_clip_seq -= 1
						print("PicrawlerBody: [F3] dropped clip %s" % fn)
						_ui_notify("clip dropped")
						break
	elif key == KEY_B:
		# Live BODY swap — the morphological (d) test.  Toggles cad ↔ measured
		# under the SAME brain, mid-run.  Watch TLE: a spike that then decays is
		# re-inference against a changed morphology, and is the evidence.  A flat
		# TLE means the swap did not reach anything the brain predicts with.
		var next_body: String = "measured" if _geometry_name == "cad" else "cad"
		var next_path: String = "res://addons/ami_ogma/body/%s.json" % next_body
		if not FileAccess.file_exists(next_path):
			_ui_notify("[body] %s.json not present" % next_body)
		else:
			_rebuild_body(next_path)
	elif key == KEY_1:
		# Live gym swap — drop the experienced robot (brain intact) into the ARENA (donut).
		_switch_gym("arena")
	elif key == KEY_2:
		# Live gym swap — drop the experienced robot (brain intact) into the CORRIDOR trench.
		_switch_gym("corridor")
	elif key == KEY_3:
		# Controlled belly-on-ramp test — drop the experienced robot onto the corridor hump.
		_teleport_to_ramp()
	elif key == KEY_4:
		# Mouse-guided teleport placement — X follows the mouse, left-click to drop.
		_set_place_mode(not _place_mode)
	elif key == KEY_V:
		# 2026-06-13 — toggle the loom/vision ray overlay (camera-placement debug).
		_ray_overlay_on = not _ray_overlay_on
		if _ray_overlay_mi != null:
			_ray_overlay_mi.visible = _ray_overlay_on
		if _ray_overlay_on:
			_update_ray_overlay()   # immediate refresh, don't wait for the loom tick
		print("PicrawlerBody: [V] ray_overlay = %s" % _ray_overlay_on)
	elif key == KEY_M:
		# 2026-07-26 — toggle JUST the MOTOR-EPM slider panel.  It is a full-rect Control
		# that sits over the top-right corner even when collapsed, which covered the IMU
		# scope.  [T] still hides all panels together; this is the single-panel version.
		# (The vision-panel toggle moved from M to N to free this key — the current
		# picrawler configs carry no vision module, so that binding was inert here.)
		var mp: Node = get_tree().get_root().find_child("MotorEpmPanel", true, false)
		if mp != null and mp is Control:
			var c := mp as Control
			c.visible = not c.visible
			print("PicrawlerBody: [M] motor_epm_panel = %s" % c.visible)
	elif key == KEY_N:
		# 2026-06-13 — toggle the camera+LiDAR vision pixel panels (no camera
		# switch — the normal orbit camera is easy to drive by hand).
		# Moved from [M] on 2026-07-26 so M can toggle the MOTOR-EPM slider panel.
		_vision_panel_on = not _vision_panel_on
		_update_vision_panel()
		print("PicrawlerBody: [N] vision_panel = %s" % _vision_panel_on)
	elif key == KEY_O:
		# 2026-08-12 — toggle the PLAN AUTHORITY bench (lever b): the reflex↔plan
		# mix slider, the earned-bands gate override, and the live hand-mask
		# controls.  Its own key because [M] is the MOTOR-EPM panel.
		var pp: Node = get_tree().get_root().find_child("PlanAuthorityPanel", true, false)
		if pp != null and pp is Control:
			var pc := pp as Control
			pc.visible = not pc.visible
			print("PicrawlerBody: [O] plan_authority_panel = %s" % pc.visible)
	elif key == KEY_U:
		# 2026-08-17 (PART IV) — toggle the GAIN EVOLVER panel: live vector +
		# incumbent/candidate criterion scores + accept history + the mutation-σ
		# slider (0 = observer; ADOPT hands a hand-tuned [M] point to the
		# evolver before σ-resume).  Its own key: [M] and [O] are taken.
		var gp: Node = get_tree().get_root().find_child("GainEvolverPanel", true, false)
		if gp != null and gp is Control:
			var gc := gp as Control
			gc.visible = not gc.visible
			print("PicrawlerBody: [U] gain_evolver_panel = %s" % gc.visible)

func _unhandled_input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	if (event as InputEventKey).keycode == KEY_SPACE:
		# SPACE always = reset.  Per-servo calibration auto-advances on
		# SERVO_CALIBRATE_HOLD_TICKS — no manual step needed.
		_done = false
		_pending_manual_reset = true
		print("PicrawlerBody: manual reset queued (SPACE)")

func _physics_process(delta: float) -> void:
	if brain == null or not brain.is_brain_ready():
		return
	_accum_grf()
	# Apply a deferred teleport (KEY_3 / KEY_4-click / env) HERE in the physics step
	# so the transform writes stick — input-frame writes get clobbered by the solver.
	if _pending_teleport != null:
		_teleport_to(_pending_teleport)
		_pending_teleport = null
		_pending_teleport_flip = -1      # consumed — later drops fall back to the env again
	# Turbo budget — exit the process cleanly when either:
	#   (a) OGMA_QUIT_AFTER_TICKS is reached, OR
	#   (b) the run is already _done (max_steps hit + reset_mode=continuous
	#       prints RUN_END and sets _done=true, but tick_counter stops
	#       advancing because _step_one is short-circuited below, so a
	#       tick-based budget would never trigger after RUN_END).
	# Must run BEFORE the _done short-circuit; otherwise the godot process
	# burns wall-clock until the harness's `timeout` SIGTERMs it.
	if _done or (_quit_after_ticks > 0 and tick_counter >= _quit_after_ticks):
		# Phase 6.10 — optional save-on-exit, env-gated.  Lets headless
		# smokes save state so the round-trip can be verified without
		# interactive F5 input.  OGMA_PICRAWLER_SAVE_STATE_AT_EXIT=<name>
		# writes to user://saved_states/<name>.brain_state.json.
		var save_name: String = OS.get_environment("OGMA_PICRAWLER_SAVE_STATE_AT_EXIT")
		if save_name != "":
			var saved: String = _save_brain_state(save_name)
			if saved != "":
				print("PicrawlerBody: SAVE_STATE_AT_EXIT → %s" % saved)
		print("PicrawlerBody: quitting (done=%s, tick=%d, budget=%d)"
				% [_done, tick_counter, _quit_after_ticks])
		get_tree().quit()
		return
	# Sensor rate ≠ control rate: sample + filter the IMU every PHYSICS tick (240 Hz)
	# before the 50 Hz brain steps consume it.  See _imu_substep().
	_imu_substep(delta)
	_accum += delta
	if _accum > 0.2:
		_accum = 0.2
	while _accum >= TAU:
		_accum -= TAU
		if _done: return
		_step_one()

# ---------------------------------------------------------------------------
# IMU substep — runs at the PHYSICS rate (physics_hz, default 240), NOT at the
# brain's TAU rate.  This is what a real IMU does: MPU-6050-class parts sample
# 1-8 kHz, a BNO085 fuses on-chip at ~400 Hz, and attitude filters on legged
# machines run 200-1000 Hz.  Sampling at the 50 Hz control rate instead both
# understates the hardware and starves the filter (aliasing footfall impacts),
# and the physics is ALREADY oversampled here to stop feet tunnelling through
# the floor — so this bandwidth costs nothing.
# ---------------------------------------------------------------------------
# IMU state for the HUD scope.  Split deliberately into signals a REAL PiCrawler can
# observe and signals only the simulator knows: on hardware there is no ground-truth
# attitude, so the honest health check is the DISAGREEMENT between the accelerometer-only
# estimate and the fused one — both computable on-robot.  The scope surfaces that as the
# quantity to watch during bring-up, and marks the sim-only rows as such.
func get_imu_debug() -> Dictionary:
	var up_exact: Vector3 = _chassis.global_transform.basis.transposed() * Vector3.UP \
		if _chassis != null else Vector3.UP
	return {
		# --- hardware-observable ---
		"up_fused": _up_est_body,          # complementary-filter gravity-up (body frame)
		"up_accel": _up_acc_last,          # accelerometer-only gravity-up (body frame)
		"accel":    _accel_body_last,      # modelled accelerometer, m/s^2, body frame
		"gyro":     _gyro_body_last,       # modelled gyro, rad/s, body frame
		"trust":    _dbg_acc_trust,        # adaptive correction gain actually applied
		"disagree_deg": rad_to_deg(_up_est_body.angle_to(_up_acc_last)) \
			if _up_acc_last.length() > 0.5 else 0.0,
		# --- simulator ground truth (NOT available on the real robot) ---
		"up_exact":    up_exact,
		"err_fused_deg": _dbg_att_err_imu,
		"err_accel_deg": _dbg_att_err_acc,
		"imu_hz": float(physics_hz),
	}

func _imu_substep(dt: float) -> void:
	if _chassis == null or dt <= 0.0:
		return
	var basis_w: Basis = _chassis.global_transform.basis
	var w2b: Basis = basis_w.transposed()          # world → body (orthonormal)
	# Accelerometer = proper acceleration in the BODY frame: at rest it points along
	# body-frame up with magnitude g; under acceleration it tilts by −a_world.
	var a_world: Vector3 = (_chassis.linear_velocity - _prev_lin_vel) / dt
	_prev_lin_vel = _chassis.linear_velocity
	var accel_body: Vector3 = w2b * (a_world + Vector3.UP * 9.81)
	# A real accelerometer has an ANALOG anti-alias filter ahead of its ADC; a raw
	# finite difference has none, so model the datasheet DLPF as a first-order low-pass.
	if _accel_lp.length() < 1e-6:
		_accel_lp = accel_body
	_accel_lp = _accel_lp.lerp(accel_body, IMU_DLPF_ALPHA)
	# ...and it CLIPS at its configured full scale (±4 g here) rather than reporting
	# unbounded impulse.
	var accel_meas: Vector3 = _accel_lp
	if accel_meas.length() > IMU_RANGE_MS2:
		accel_meas = accel_meas.normalized() * IMU_RANGE_MS2
	var gyro_body: Vector3 = w2b * _chassis.angular_velocity
	_accel_body_last = accel_meas
	_gyro_body_last  = gyro_body
	# Dead-reckon the ego heading from the gyro's yaw component (this runs at the IMU's own
	# substep rate, so use that dt rather than the brain tick).
	_ego_heading = wrapf(_ego_heading + gyro_body.y * dt, -PI, PI)
	_up_acc_last = accel_meas.normalized() if accel_meas.length() > 1e-4 else _up_acc_last

	# --- complementary filter -------------------------------------------------
	if _up_est_body.length() < 0.5:
		_up_est_body = _up_acc_last
	# Gyro propagation: a WORLD-fixed direction seen from the body frame rotates by
	# −ω·dt.  Use an EXACT rotation — the first-order `v -= ω×v·dt` form leaves
	# O((ω·dt)²) error per step, which integrates to radians over a run.
	var w_mag: float = gyro_body.length()
	if w_mag > 1e-6:
		_up_est_body = (Basis(gyro_body / w_mag, -w_mag * dt) * _up_est_body).normalized()
	# Adaptive-gain correction: the accelerometer only indicates "down" when the body is
	# quasi-static; during a footfall it is measuring the impact.  Weight its trust by how
	# close ‖a‖ is to g rather than accepting/rejecting outright (a hard gate starved it).
	var acc_dev: float = absf(accel_meas.length() - 9.81) / 9.81
	var trust: float = IMU_ACC_TRUST * clampf(1.0 - acc_dev / IMU_ACC_GATE_FRAC, 0.0, 1.0)
	if trust > 0.0:
		_up_est_body = (_up_est_body * (1.0 - trust) + _up_acc_last * trust).normalized()
	_dbg_acc_mag = accel_meas.length()
	_dbg_acc_trust = trust
	var up_exact: Vector3 = w2b * Vector3.UP
	_dbg_att_err_acc = rad_to_deg(_up_acc_last.angle_to(up_exact))
	_dbg_att_err_imu = rad_to_deg(_up_est_body.angle_to(up_exact))

func _step_one() -> void:
	# Pending chassis suspend / unfreeze from C-key (queued in
	# _input).  Must be applied inside a physics frame for the
	# transform write + freeze toggle to actually take effect against
	# the dynamic-body physics update.
	if _pending_chassis_freeze == 1 or _pending_chassis_freeze == 3:   # 1 = air-suspend (C), 3 = ground-freeze (G)
		var is_ground_freeze: bool = _pending_chassis_freeze == 3
		_pending_chassis_freeze = 0
		# CRITICAL: must teleport ALL body parts (chassis + 12 leg
		# segments) together to the suspended position.  Otherwise the
		# legs stay at their construction y≈0.08 while chassis jumps
		# to y=0.25 and the joint constraints violently yank the legs
		# up to chassis, overshooting into joint limits where they get
		# stuck (observed as "legs folded UP into the body with hip2
		# at upmost and knee at downmost").
		# is_ground_freeze=true freezes AT standing pose (G mode); false
		# suspends in air at y=0.25 (C mode).  Same FK direct-write path
		# below, just different chassis height + lift offset.
		var suspend_y: float = STANDING_CHASSIS_Y if is_ground_freeze else 0.25
		var delta_y: float = suspend_y - _chassis_rest_xform.origin.y
		var lift: Vector3 = Vector3(0, delta_y, 0)
		_suspend_lift_y = delta_y
		# Briefly unfreeze chassis to write its transform.
		_chassis.freeze = false
		# Freeze each leg body kinematically so transform writes stick.
		var all_legs: Array = _coxas + _uppers + _lowers
		for b in all_legs:
			b.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
			b.freeze = true
		# Write all body transforms at the lifted position.  Force
		# IDENTITY basis explicitly — the rest_xform was captured at the
		# end of _build_leg, AFTER joint creation, which may have nudged
		# the body basis.  If we restore to a non-IDENTITY basis, the
		# measurement function `_relative_angle_world_axis` reads non-
		# zero joint angles even though the bodies are "at rest" → PD
		# loop fights a phantom error → motor saturates → joint slides
		# to limit on first tick (the "+1.54 hip2 / +1.68 knee stuck"
		# symptom in the calibration log).  IDENTITY is the only basis
		# that makes all 12 joints simultaneously read 0.
		_chassis.global_transform = Transform3D(Basis.IDENTITY,
												_chassis_rest_xform.origin + lift)
		_chassis.linear_velocity = Vector3.ZERO
		_chassis.angular_velocity = Vector3.ZERO
		for i in range(4):
			_coxas[i].global_transform  = Transform3D(Basis.IDENTITY,
													  _coxa_rest_xform[i].origin + lift)
			_uppers[i].global_transform = Transform3D(Basis.IDENTITY,
													  _upper_rest_xform[i].origin + lift)
			_lowers[i].global_transform = Transform3D(Basis.IDENTITY,
													  _lower_rest_xform[i].origin + lift)
			for b in [_coxas[i], _uppers[i], _lowers[i]]:
				b.linear_velocity  = Vector3.ZERO
				b.angular_velocity = Vector3.ZERO
		# Re-freeze chassis (static) so it holds.  Legs STAY frozen
		# kinematic during calibration — the calibrate branch in
		# _step_one writes their transforms each tick via forward
		# kinematics from the slider values.  Bypassing the motor +
		# joint-constraint chain is the only reliable way to make
		# sliders drive joints to the exact commanded angle (motors in
		# Godot drive joints to deterministic limits regardless of
		# target velocity, as observed in earlier calibration logs).
		_chassis.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
		_chassis.freeze = true
		# Legs remain freeze_mode=KINEMATIC, freeze=true.  They will be
		# unfrozen on the corresponding UNFREEZE path (C-key off).
		# Reset slew + effective-target state so PD doesn't react to the
		# teleport as if it were a discontinuity.
		for i in range(4):
			_prev_torque_hip1[i] = 0.0
			_prev_torque_hip2[i] = 0.0
			_prev_torque_knee[i] = 0.0
			_eff_target_hip1[i] = HIP1_REST
			_eff_target_hip2[i] = HIP2_REST
			_eff_target_knee[i] = KNEE_REST
		print("PicrawlerBody: chassis + legs SUSPENDED at y=%.2f (delta=%.3f)" % [
			suspend_y, delta_y])
		# Diagnostic: confirm bodies are at IDENTITY basis and joint
		# angles are all zero immediately after the teleport.  Anything
		# non-zero here indicates a construction-time bug or the joint
		# solver yanking bodies in the same frame the teleport ran in.
		var post_h1: Array = []
		var post_h2: Array = []
		var post_kn: Array = []
		for i in range(4):
			post_h1.append(_relative_angle_world_axis(_chassis, _coxas[i], Vector3.UP))
			post_h2.append(_relative_angle_world_axis(_coxas[i], _uppers[i], _hip2_axes[i]))
			post_kn.append(_relative_angle_world_axis(_uppers[i], _lowers[i], _knee_axes[i]))
		print("PicrawlerBody: post-suspend angles  hip1=[%+.2f,%+.2f,%+.2f,%+.2f]  hip2=[%+.2f,%+.2f,%+.2f,%+.2f]  knee=[%+.2f,%+.2f,%+.2f,%+.2f]" % [
			post_h1[0], post_h1[1], post_h1[2], post_h1[3],
			post_h2[0], post_h2[1], post_h2[2], post_h2[3],
			post_kn[0], post_kn[1], post_kn[2], post_kn[3]])
		for i in range(4):
			print("PicrawlerBody: leg %d bases  coxa.y=%v  upper.y=%v  lower.y=%v" % [
				i,
				_coxas[i].global_transform.basis.y,
				_uppers[i].global_transform.basis.y,
				_lowers[i].global_transform.basis.y])
	elif _pending_chassis_freeze == 2:   # unfreeze
		_pending_chassis_freeze = 0
		_chassis.freeze = false
		_chassis.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC   # default
		# Restore legs to dynamic so motors + gravity take over again.
		for i in range(4):
			for b in [_coxas[i], _uppers[i], _lowers[i]]:
				b.freeze = false
				b.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
		_suspend_lift_y = 0.0
		print("PicrawlerBody: chassis UNFROZEN — dynamic physics resumed")

	# Manual reset (spacebar) — runs first inside the physics frame so
	# the freeze→write→unfreeze cycle in _do_hard_reset actually sticks
	# against dynamic-body transform overrides.  Brain doesn't tick on
	# the snap tick (skipped via _instant_pause_tick).
	if _pending_manual_reset:
		_pending_manual_reset = false
		_do_hard_reset()
		step_in_episode = 0
		episode_alive_ticks = 0
		_episode_fell = false
		_instant_pause_tick = true
		return

	tick_counter += 1
	step_in_episode += 1
	# Controlled belly-on-ramp test: auto-drop onto the hump at the configured tick
	# (headless; e.g. after the gait develops).  One-shot.
	# ---- SCRIPTED EVENT TIMELINE (2026-07-26) ----------------------------------------
	# OGMA_PICRAWLER_EVENTS = JSON array of timed placements, e.g.
	#   [{"at":3600,"xz":[0,3.0]},{"at":7200,"xz":[0.4,2.6],"flip":true}]
	# "let it walk straight for a minute, then drop it HERE inverted, then there" — the
	# thing the UI placement tool does by hand, made reproducible.  Deliberately NOT a
	# revival of CurriculumManager: that carries reward/trainer machinery this substrate
	# does not want.  This is just a schedule of teleports, reusing the same drop path (so
	# the HUD "last drop" line and the replay hint apply identically).
	for ev in _events:
		if int(ev.get("at", -1)) == tick_counter:
			var xz: Array = ev.get("xz", [0.0, 0.0])
			var surf: float = _surface_y_at(float(xz[0]), float(xz[1]))
			_pending_teleport = Vector3(float(xz[0]), surf, float(xz[1]))
			_pending_teleport_flip = 1 if bool(ev.get("flip", false)) else 0
			print("PicrawlerBody: [events] tick %d -> drop at (%.2f, %.2f)%s" % [
				tick_counter, float(xz[0]), float(xz[1]),
				"  INVERTED" if bool(ev.get("flip", false)) else ""])
	# Per-leg lesion window.  Announced on each edge so a run can never silently be the
	# arm you did not intend (§3.2 rule 7); NO reset event is published, on purpose.
	if _lesion_leg >= 0 and _lesion_at > 0:
		var want: bool = tick_counter >= _lesion_at and tick_counter < _lesion_until
		if want != _lesion_active:
			_lesion_active = want
			print("PicrawlerBody: [lesion] leg %d %s at tick %d" % [
				_lesion_leg, ("CUT to x%.2f" % _lesion_scale) if want else "RESTORED",
				tick_counter])
	# Per-foot slick ablation.  Announced once, loudly, for the same reason the lesion is.
	if _slick_leg >= 0 and not _slick_done and tick_counter >= _slick_at:
		_slick_done = true
		if _slick_leg < _lowers.size() and is_instance_valid(_lowers[_slick_leg]):
			_lowers[_slick_leg].physics_material_override = _make_slick_mat()
			print("PicrawlerBody: [slick] leg %d foot -> mu 0.05 at tick %d (leg still driven)" % [
				_slick_leg, tick_counter])
	if _abl_env_spec != "" and not _abl_env_done and tick_counter >= _abl_env_at:
		_abl_env_done = true
		_abl_apply_env_spec()
	# Scheduled brain-param flips (SETPARAM_AT).  Announced loudly per §3.2 rule 7 —
	# a run must never silently be the arm you did not intend.
	for _sp in _sched_patches:
		if not _sp["done"] and tick_counter >= int(_sp["t"]) and brain != null:
			_sp["done"] = true
			var _res = brain.apply_patch({"op": "set_param", "id": _sp["id"],
				"key": _sp["key"], "value": _sp["val"]})
			var _ok: bool = typeof(_res) == TYPE_DICTIONARY and bool(_res.get("success", false))
			print("PicrawlerBody: [setparam_at] %s.%s = %s at tick %d -> %s" % [
				_sp["id"], _sp["key"], str(_sp["val"]), tick_counter,
				("OK" if _ok else "FAILED " + str(_res))])
	if _teleport_ramp_at > 0 and tick_counter == _teleport_ramp_at:
		_teleport_to_ramp()
	elif _teleport_ramp_at > 0 and _teleport_every > 0 and tick_counter > _teleport_ramp_at \
			and tick_counter <= _teleport_until and (tick_counter - _teleport_ramp_at) % _teleport_every == 0:
		_teleport_to_ramp()   # re-fire to sustain the anomaly
	# Keyframe map probe (env OGMA_PICRAWLER_KF_PROBE=1) — measures map drift +
	# bake suppression across an anomaly for the bake-gate A/B.
	if tick_counter % 300 == 0 and brain != null and OS.get_environment("OGMA_PICRAWLER_KF_PROBE") == "1":
		var snap = JSON.parse_string(str(brain.get_module_snapshot("keyframe_gait")))
		if snap is Dictionary:
			var mnorm: float = 0.0
			if snap.has("bins"):
				for bj in snap["bins"]:
					if typeof(bj) == TYPE_DICTIONARY and bj.has("kf"):
						for val in bj["kf"]: mnorm += abs(float(val))
			print("KF_PROBE t=%d up=%.2f keyframe_tle=%.3f suppressed=%s map_norm=%.3f" % [
				tick_counter, _chassis.global_transform.basis.y.y,
				float(snap.get("keyframe_tle", -1.0)), str(snap.get("bake_suppress_count", -1)), mnorm])

	# 2026-06-08 — periodic Cruse trace V2 (toggled via env var or panel).
	if _cruse_trace_enabled and tick_counter % _CRUSE_TRACE_INTERVAL_TICKS == 0:
		_print_cruse_trace()

	# 2026-06-07 — defensive purple-target refresh every 60 ticks (~1 sim sec).
	# Cheap idempotent check that keeps the active target visually highlighted
	# across all entry paths (calibration ↔ experiment transitions, snapshot
	# restore, etc.) — addresses Joseph's "target doesn't turn purple in
	# experiment mode" observation.
	if tick_counter % 60 == 0:
		_ensure_target_pyramid_color()

	# 2026-06-06 — time-mark snapshot trigger (for single-stage curricula that never
	# auto-advance, e.g. target-aware warmup).  Fires once when tick_counter * TAU
	# crosses the save-at threshold.  Reuses the same fire-once logic as the
	# stage-advance trigger; either trigger can fire, whichever comes first.
	if (_brain_snapshot_save_path != "" and not _brain_snapshot_save_done
			and _brain_snapshot_save_at_sec > 0.0
			and float(tick_counter) * TAU >= _brain_snapshot_save_at_sec):
		var snap_text: String = brain.snapshot_state()
		var f := FileAccess.open(_brain_snapshot_save_path, FileAccess.WRITE)
		if f == null:
			push_error("PicrawlerBody: cannot open SNAPSHOT_SAVE path " + _brain_snapshot_save_path)
		else:
			f.store_string(snap_text)
			f.close()
			_brain_snapshot_save_done = true
			print("PicrawlerBody: BRAIN_SNAPSHOT saved (time-mark) → %s (%d bytes, sim_sec=%.0f)" % [
				_brain_snapshot_save_path, snap_text.length(), float(tick_counter) * TAU])
	# 2026-06-06 — yaw probe scheduler.  Advance phase based on tick_counter,
	# minus the settle offset that fires only on snapshot LOAD runs.
	# Off when disabled or still within settle window.
	if _yaw_probe_enabled and tick_counter >= _yaw_probe_settle_offset_ticks:
		var effective_tick: int = tick_counter - _yaw_probe_settle_offset_ticks
		var phase_idx: int = (effective_tick / YAW_PROBE_PHASE_TICKS) % YAW_PROBE_SCHEDULE.size()
		_yaw_probe_delta = YAW_PROBE_SCHEDULE[phase_idx]
	else:
		_yaw_probe_delta = 0.0

	# ---- 1. Read joint angles per leg ----
	var hip1_angles: Array = []
	var hip2_angles: Array = []
	var knee_angles: Array = []
	for i in range(4):
		# Hip1: rotation between chassis and coxa, projected onto world UP.
		hip1_angles.append(_relative_angle_world_axis(_chassis, _coxas[i], Vector3.UP))
		# Hip2: rotation between coxa and upper, projected onto leg-local lateral.
		hip2_angles.append(_relative_angle_world_axis(_coxas[i], _uppers[i], _hip2_axes[i]))
		# Knee: rotation between upper and lower, projected onto same lateral.
		knee_angles.append(_relative_angle_world_axis(_uppers[i], _lowers[i], _knee_axes[i]))
	# A DETACHED segment is frozen in world space while the chassis walks away, so its
	# measured relative angle diverges into garbage.  A real robot's servo keeps reporting
	# its own encoder after the limb below it snaps off, so hold the last honest reading
	# rather than feeding the brain a signal no physical robot could produce.
	# A break is inherited DISTALLY: snapping at hip1 leaves hip2 and the knee with nothing
	# to report either.  Holding only the broken joint is not enough and is actively harmful
	# -- the freed limb is frozen in WORLD space while the chassis keeps walking and turning,
	# so the axis each angle is projected onto sweeps past it and the joint reads a PHANTOM
	# OSCILLATION.  Measured before this fix: a leg detached at hip1 still reported amp_ema
	# 0.670 (vs 0.001 when detached at the knee, where the projection axis stays attached) --
	# i.e. the brain was told a limb lying on the floor was still swinging, and that phantom
	# feeds straight into the precision weight w = amp/(tle+eps).
	if _abl_any:
		for i in range(4):
			var d0: bool = _abl_detached[abl_idx(i, 0)] != 0
			var d1: bool = d0 or _abl_detached[abl_idx(i, 1)] != 0
			var d2: bool = d1 or _abl_detached[abl_idx(i, 2)] != 0
			if d0: hip1_angles[i] = _abl_hold_angle[abl_idx(i, 0)]
			if d1: hip2_angles[i] = _abl_hold_angle[abl_idx(i, 1)]
			if d2: knee_angles[i] = _abl_hold_angle[abl_idx(i, 2)]

	var chassis_xform: Transform3D = _chassis.global_transform
	var chassis_y: float = chassis_xform.origin.y
	var chassis_tilt: float = _chassis_tilt(chassis_xform.basis)
	var yaw: float = chassis_xform.basis.get_euler().y
	# imu[2] = SIGNED forward speed (velocity·heading) — the documented "forward
	# speed".  Was |velocity| (direction-agnostic), which let the agency-reward gait
	# search reward sideways crab as much as forward; signed forward rewards
	# forward-aligned motion (reduces crab + closes a speed-magnitude Goodhart).
	var fwd_v: float = Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z).dot(Vector2(sin(yaw), cos(yaw)))
	var ang_v: float = _chassis.angular_velocity.y

	# ---- Auto-reset on belly-up inversion (opt-in training safety) ----
	# Distinguishes "unrecoverable" (chassis past on-its-side AND touching
	# the floor for a sustained dwell) from "recoverable" (laying on its
	# side, tilt ≈ π/2).  Side-laying postures ARE potentially recoverable
	# via leg torque — the brain must learn that correction itself, so
	# we deliberately do NOT auto-reset at the standard FAIL_TILT.  The
	# dwell counter avoids triggering on a single transient frame.
	if auto_reset_on_inversion:
		# Shape-aware grounding test.  chassis_y is the NODE ORIGIN height, and
		# where the origin sits when the body is on its back is set by the
		# topmost chassis surface — the belly-up contact patch.  Comparing a raw
		# origin height against a fixed constant only works for the one chassis
		# it was tuned on.  See _LEGACY_INVERTED_REST_H.
		# Ordering matters for byte-identity: written this way the correction
		# term is EXACTLY 0.0 for the legacy box (_chassis_top_local is
		# CHASSIS_Y*0.5 = 0.021, halving is exact in IEEE), so the threshold is
		# bit-for-bit the old literal.  The other grouping,
		# _chassis_top_local + (max_height - REF), rounds to 0.030000000000000002.
		var inverted_ground_h: float = auto_reset_max_height \
			+ (_chassis_top_local - _LEGACY_INVERTED_REST_H)
		if chassis_tilt > auto_reset_tilt_threshold and chassis_y < inverted_ground_h:
			_auto_reset_dwell_counter += 1
			if _auto_reset_dwell_counter >= auto_reset_dwell_ticks:
				# If the chassis is OUTSIDE the central ring zone (i.e.
				# exploring the pyramid donut), reset in place so the
				# body keeps trying in that general area instead of being
				# teleported back to the center.  Pyramid-avoidance
				# nudges the spawn outward if the chassis happens to be
				# buried in one.  Inside the ring → legacy center reset.
				var chassis_pos: Vector3 = _chassis.global_transform.origin
				var chassis_xz: Vector2 = Vector2(chassis_pos.x, chassis_pos.z)
				var in_place: bool = false
				if chassis_xz.length() > RING_OUTERMOST_RADIUS:
					var safe_xz: Vector2 = _safe_reset_xz_near(chassis_xz)
					if not is_inf(safe_xz.x):
						_pending_reset_offset = Vector3(safe_xz.x, 0.0,
														safe_xz.y) - Vector3(
														_chassis_rest_xform.origin.x,
														0.0,
														_chassis_rest_xform.origin.z)
						in_place = true
				if in_place:
					print("PicrawlerBody: auto-reset in-place (belly-up: tilt=%.2f, y=%.3f, dwell=%d ticks; spawn xz=(%.2f, %.2f))" % [
						chassis_tilt, chassis_y, _auto_reset_dwell_counter,
						chassis_xz.x + _pending_reset_offset.x,
						chassis_xz.y + _pending_reset_offset.z])
				else:
					print("PicrawlerBody: auto-reset to center (belly-up: tilt=%.2f, y=%.3f, dwell=%d ticks)" % [
						chassis_tilt, chassis_y, _auto_reset_dwell_counter])
				auto_reset_count += 1
				_auto_reset_dwell_counter = 0
				# Queue the hard reset for the next physics frame — same
				# path as SPACE.  Brain skips its tick on the reset frame
				# via _instant_pause_tick (handled in the existing manual-
				# reset block at the top of _physics_process).
				_pending_manual_reset = true
		else:
			_auto_reset_dwell_counter = 0
	# Phase 6.14 — outer-wall reset.  Fires the instant chassis horizontal
	# distance from origin exceeds the configured radius (default 9.5 m,
	# just inside the wedge ring at r=10).  No dwell counter — once the
	# body has reached the wedges, it'll only get further off the floor
	# over time.  _ui_notify so the user sees the trigger fire live.
	if auto_reset_on_outer_wall and not _pending_manual_reset:
		var chassis_pos: Vector3 = _chassis.global_transform.origin
		var chassis_r: float = Vector2(chassis_pos.x, chassis_pos.z).length()
		if chassis_r > AUTO_RESET_OUTER_WALL_RADIUS:
			print("PicrawlerBody: auto-reset (outer wall: r=%.2f m, threshold=%.2f m)" % [
				chassis_r, AUTO_RESET_OUTER_WALL_RADIUS])
			auto_reset_count += 1
			_pending_manual_reset = true
			_ui_notify("[OUTER WALL] auto-reset at r=%.2f m" % chassis_r)

	# ---- 2. Publish proprio ----
	var imu := PackedFloat64Array()
	imu.append(sin(yaw)); imu.append(cos(yaw))
	imu.append(clamp(fwd_v / 1.0, -1.0, 1.0))
	imu.append(clamp(ang_v / PI,  -1.0, 1.0))
	# Per-metre waypoint (see the state block).  Uses the same path-length accumulation the
	# red trail does, so the log and the picture cannot disagree.
	var _wp_now := Vector2(_chassis.global_transform.origin.x, _chassis.global_transform.origin.z)
	if tick_counter <= 1:
		_wp_last_xz = _wp_now
	_wp_path_len += _wp_now.distance_to(_wp_last_xz)
	_wp_last_xz = _wp_now
	while _wp_path_len >= _wp_next_m:
		print(JSON.stringify({"wp": _wp_next_m, "t": tick_counter,
			"dt": tick_counter - _wp_last_tick, "x": snappedf(_wp_now.x, 0.001),
			"z": snappedf(_wp_now.y, 0.001)}))
		_wp_last_tick = tick_counter
		_wp_next_m += 1.0
	# ---- 2026-08-05 · MOTOR INTENT (the higher loop's lever) ----------------------------
	# [v_forward*, yaw_rate*] — what the body is being asked to do RIGHT NOW.  MotorEPM turns
	# commit confidence into "am I achieving this" instead of "how well do I predict myself".
	# ⚠ This constant-forward publisher is a SCAFFOLD, named as one: it stands in for the L1
	# nav / EFE arbiter until that layer exists, so the mechanism can be tested before the
	# thing that will eventually drive it.  A real intent would change with the manoeuvre
	# (forward / turn left / turn right / stop), which is the whole point of the socket.
	if intent_fwd >= 0.0:
		var it := PackedFloat64Array()
		it.append(intent_fwd)
		it.append(intent_yaw)
		brain.publish_proprio(it, "motor_intent")
	brain.publish_proprio(imu, "imu")

	# ---- L1 nav inputs (2026-08-04) -------------------------------------------------------
	# RunTumbleNavV2 wants a scalar heading and an egocentric velocity.  Both are published
	# unconditionally (they are cheap and honest); the nav module is what is gain-guarded.
	var eh := PackedFloat64Array()
	eh.append(_ego_heading)
	brain.publish_proprio(eh, "ego_heading")
	# [v_right, v_forward] — index 1 is forward, which is the index RunTumbleNavV2 reads.
	# ⚠ SOFT ORACLE, recorded as such: this is world-frame chassis velocity projected into the
	# body frame, and sensor_legitimacy_and_the_feet_y_oracle.md flags fwd_v/lateral_v as "not
	# free — worth its own pass".  A real legged robot has no odometry; estimating body speed
	# from an IMU alone is genuinely hard.  RunTumbleNavV2 uses it ONLY for its KF2
	# efference-matched stuck check (achieved vs learned capable speed), never for the
	# gradient, so the oracle does not touch the inference — but it is named, not hidden.
	var ve := PackedFloat64Array()
	ve.append(Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z)
		.dot(Vector2(cos(yaw), -sin(yaw))))
	ve.append(fwd_v)
	brain.publish_proprio(ve, "vel_ego")

	# Signed lateral (sideways-slip) velocity on its OWN topic — kept off the imu
	# vector so epm_imu's projection_dim is untouched.  + = body drifts to its
	# right.  The MotorEPM agency-reward search penalises |lateral_v| (coord_lat_
	# penalty) so it self-discovers a straight, lateral-cancelling gait instead of
	# the rear-fishtail crab the forward-only fitness left unpenalised.
	var lat_v: float = Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z).dot(Vector2(cos(yaw), -sin(yaw)))
	_last_lat_v = lat_v
	var latp := PackedFloat64Array()
	latp.append(clamp(lat_v / 1.0, -1.0, 1.0))
	brain.publish_proprio(latp, "lateral_v")

	# World-frame compass — 2-D sin/cos of chassis yaw.  Cheap (2 floats);
	# only configs that instantiate epm_compass will consume it.
	var compass := PackedFloat64Array()
	compass.append(sin(yaw)); compass.append(cos(yaw))
	brain.publish_proprio(compass, "compass")

	# Phase 7.5.R — body-frame egocentric "outward direction".  Carries
	# the body-relative goal vector for the radial-outward walking task:
	# given chassis world position (x, z) and yaw, project the world-
	# frame outward unit vector r̂ = (x, z) / |(x, z)| into body frame.
	#
	# At chassis-at-origin (r < 1 cm) publish (0, 0) so the brain has no
	# defined direction and explores.  Otherwise the 2-D unit vector
	# points from body-origin toward away-from-world-origin in the
	# body's local frame — invariant under body rotation, so "drive
	# toward this direction" is a stable instruction regardless of
	# current heading.
	#
	# Complements (does not replace) reality.proprio.compass: compass
	# is the world-heading reference, radial_compass is the
	# task-specific egocentric goal vector.  Future walk_reward_mode=
	# "to_target" would publish a different egocentric vector on a
	# different topic (target_compass) without disrupting either.
	var rc := PackedFloat64Array()
	var chassis_xz_local := Vector2(chassis_xform.origin.x, chassis_xform.origin.z)
	var r_dist: float = chassis_xz_local.length()
	if r_dist > 0.01:
		var cy: float = cos(yaw)
		var sy: float = sin(yaw)
		var inv_r: float = 1.0 / r_dist
		# 2-D world→body rotation by −yaw on (x, z).
		rc.append((chassis_xz_local.x * cy - chassis_xz_local.y * sy) * inv_r)
		rc.append((chassis_xz_local.x * sy + chassis_xz_local.y * cy) * inv_r)
	else:
		rc.append(0.0)
		rc.append(0.0)
	brain.publish_proprio(rc, "radial_compass")
	# Cache for _emit_jsonl heading telemetry (aliveness signal #1).
	_last_yaw = yaw
	_last_radial_compass = Vector2(rc[0], rc[1])

	# 2026-06-03 — target_compass perceptual stream (P2 of navigation
	# falsification ladder).  Egocentric 2D unit vector from body to the
	# active pyramid target, rotated into body frame.  When no target is
	# active (walk_target_idx < 0) OR body is exactly at target (within
	# 1 cm) publish (0, 0) so the EPM has a defined input.  Mirrors
	# radial_compass pattern exactly per picrawler_body.gd:3089-3093
	# architect comment.  Sensor only — not a reward shape.
	var tc := PackedFloat64Array()
	if walk_target_idx >= 0:
		var tgt_local := Vector2(
			walk_target_pos.x - chassis_xform.origin.x,
			walk_target_pos.y - chassis_xform.origin.z)
		var tgt_dist: float = tgt_local.length()
		if tgt_dist > 0.01:
			var cy2: float = cos(yaw)
			var sy2: float = sin(yaw)
			var inv_t: float = 1.0 / tgt_dist
			var tc_x: float = (tgt_local.x * cy2 - tgt_local.y * sy2) * inv_t
			var tc_y: float = (tgt_local.x * sy2 + tgt_local.y * cy2) * inv_t
			# 2026-06-06 — snapshot TRUE compass BEFORE cognitive rotation, for honest reward.
			_last_true_target_compass = Vector2(tc_x, tc_y)
			# Phase H1 V2 — apply cognitive steering bias.  Rotate the body-frame
			# target direction by _cognitive_bias_rad before publishing.  Brain
			# perceives the rotated target; fast Premotor steers to align;
			# cognitive's reward depends on the body's motion under its own bias.
			if _force_cog_bias_enabled:
				_cognitive_bias_rad = _force_cog_bias_rad   # L2 open-loop override
			if absf(_cognitive_bias_rad) > 1e-4:
				var cb: float = cos(_cognitive_bias_rad)
				var sb: float = sin(_cognitive_bias_rad)
				var rot_x: float = cb * tc_x - sb * tc_y
				var rot_y: float = sb * tc_x + cb * tc_y
				tc_x = rot_x
				tc_y = rot_y
			tc.append(tc_x)
			tc.append(tc_y)
		else:
			tc.append(0.0)
			tc.append(0.0)
	else:
		tc.append(0.0)
		tc.append(0.0)
	brain.publish_proprio(tc, "target_compass")
	_last_target_compass = Vector2(tc[0], tc[1])

	# 2026-06-13 — REWARD-FREE target arrival.  The visit/switch logic in the
	# reward block (~4446) is nested under `if walk_hit_rate > 0.0`, so a
	# reward-free controller (Motor-EPM perception→steering) reaches the target
	# but never switches.  This ungated check (active only when walk reward is
	# OFF, to avoid double-firing) advances target-to-target on SURFACE touch:
	# center distance < visit_radius + the pyramid's bounding radius, so a solid
	# pyramid bigger than visit_radius still registers when the chassis touches.
	if walk_target_idx >= 0 and walk_hit_rate <= 0.0:
		# Surface clearance to the TARGET specifically.  Switch only on actual
		# contact with the target surface — never on proximity, never on a
		# different pyramid.  The low crouch-gait keeps the chassis CENTRE short
		# of the pyramid (it can't climb), so we trigger on whichever part of the
		# robot reaches first: the nearest FOOT (lower-leg tips splay ~7-10 cm
		# beyond the chassis and can touch the pyramid base) OR the chassis (if a
		# taller stance ever climbs onto it).  min over {chassis, 4 feet}.
		var tgt := Vector2(walk_target_pos.x, walk_target_pos.y)
		var target_radius: float = float(_pyramid_xz_radii[walk_target_idx]) \
			if walk_target_idx < _pyramid_xz_radii.size() else 0.0
		var min_center_dist: float = Vector2(chassis_xform.origin.x,
											 chassis_xform.origin.z).distance_to(tgt)
		for _li in range(_lowers.size()):
			var fo: Vector3 = _lowers[_li].global_transform.origin
			min_center_dist = min(min_center_dist,
								  Vector2(fo.x, fo.z).distance_to(tgt))
		var surface_clearance: float = min_center_dist - target_radius
		if surface_clearance < TARGET_TOUCH_MARGIN:
			walk_visit_count += 1
			# 2026-08-04 — arrival event for the L1 nav loop.  RunTumbleNavV2 uses it to
			# calibrate its self-reported CONFIDENCE only (eat_scent_); the policy stays
			# reward-free, so this is not reward shaping (CLAUDE.md §5.1).
			if brain != null:
				brain.publish_event("beacon_reached", 1.0)
			print("PicrawlerBody: TARGET TOUCHED #%d (pyramid %d, surface clearance %.3f m) → new target" % [
				walk_visit_count, walk_target_idx, surface_clearance])
			_select_random_pyramid_target()

	# Phase H1 V6 — proxy looming detector.  Recompute target_loom every
	# LOOM_RECOMPUTE_EVERY ticks (cheap subsampling) and publish the cached
	# value every tick so the KFA + slow EPM see a continuous stream.
	_loom_recompute_counter += 1
	if _loom_recompute_counter >= LOOM_RECOMPUTE_EVERY:
		_loom_recompute_counter = 0
		_last_target_loom = _compute_target_loom()
		if _ray_overlay_on:
			_update_ray_overlay()   # refresh the debug ray lines at the loom cadence
		_update_vision_panel()      # camera+LiDAR pixel panels (captures only in top-down view)
	var loom_pkt := PackedFloat64Array()
	loom_pkt.append(_last_target_loom)
	brain.publish_proprio(loom_pkt, "target_loom")

	# Panic pathway — distress = slow PERCH-AND-STALL accumulator.  The position-only
	# watchdog false-triggered on slow/wandering LEVEL walking (operator: distress
	# climbed to 1.0 while attempting to walk), because the picrawler genuinely
	# nets little ground when slow.  Translation alone can't separate slow-walking
	# from wedged — POSE can: a level, normal body is NOT stuck even if slow; a
	# PERCHED one (tilt high + not translating) is.  Combine translation (2 s net
	# deficit) AND tilt; accumulate SLOWLY so transient walking tilt-blips stay
	# harmless while a sustained perch climbs to 1.  Warmup skips the first 10 s.
	var ch_xz := Vector2(chassis_xform.origin.x, chassis_xform.origin.z)
	_distress_pos_history.append(ch_xz)
	if _distress_pos_history.size() > DISTRESS_WINDOW_TICKS:
		_distress_pos_history.pop_front()
	if _distress_pos_history.size() == DISTRESS_WINDOW_TICKS:
		var disp: float = (_distress_pos_history[DISTRESS_WINDOW_TICKS - 1] as Vector2).distance_to(
			_distress_pos_history[0] as Vector2)
		var max_disp: float = DISTRESS_REF_SPEED * float(DISTRESS_WINDOW_TICKS) / float(physics_hz)
		_stuck_deficit = clamp(1.0 - disp / max_disp, 0.0, 1.0) if max_disp > 0.0 else 0.0
	# Graded PERCH × STALL score (smoothed tilt, not a flickering hard gate — the
	# wedge tilt oscillates ~0.2-0.39 and dips would wipe a binary accumulator).
	# perch ∈ [0,1] from smoothed |tilt|; stall = the 2 s deficit.  BOTH needed
	# (product) → level slow-walking (perch≈0) does not accumulate, a perched stall
	# does.  Rate scales with severity; slow climb (operator: slow is fine).
	_tilt_ema = (1.0 - DISTRESS_TILT_EMA_ALPHA) * _tilt_ema \
			  + DISTRESS_TILT_EMA_ALPHA * absf(_chassis_tilt(chassis_xform.basis))
	var perch: float = clamp((_tilt_ema - DISTRESS_PERCH_LO) / (DISTRESS_PERCH_HI - DISTRESS_PERCH_LO), 0.0, 1.0)
	var stuck_score: float = perch * _stuck_deficit
	if tick_counter < DISTRESS_WARMUP_TICKS:
		_distress = 0.0
	else:
		_distress = clamp(_distress + DISTRESS_RISE * stuck_score - DISTRESS_DECAY * (1.0 - stuck_score), 0.0, 1.0)
	var distress_pkt := PackedFloat64Array()
	distress_pkt.append(_distress)
	brain.publish_proprio(distress_pkt, "distress")
	_update_distress_hud()

	# Vision → brain: capture the shaded RGB at a subrate, publish the cached frame
	# every tick (sub-rate publishes drop out of the voter trust map) so epm_color
	# encodes a fresh token each tick.
	if publish_vision:
		_vision_capture_counter += 1
		if _vision_capture_counter >= VISION_CAPTURE_EVERY:
			_vision_capture_counter = 0
			_capture_vision()
			# V2 readout: epm_color latent → vision bearing (cached, applied here).
			if vision_steer and _vsteer_loaded and brain.has_method("get_module_metrics"):
				var lat = brain.get_module_metrics().get("epm_color", {}).get("last_latent", [])
				if typeof(lat) == TYPE_ARRAY and lat.size() == 64:
					var vx := _vsteer_bx
					var vy := _vsteer_by
					for i in range(64):
						vx += float(lat[i]) * _vsteer_wx[i]
						vy += float(lat[i]) * _vsteer_wy[i]
					var mag := sqrt(vx * vx + vy * vy)
					if mag > 1e-4:
						_vision_compass = Vector2(vx / mag, vy / mag)
		# ⚠ publish_video's signature is (pixels, HEIGHT, WIDTH, channels, modality) —
		# height FIRST.  Square frames hid that for the whole life of this call; a 4:3
		# frame does not.  A size mismatch push_errors and silently publishes NOTHING.
		if _last_vision_pixels.size() == vision_res_w * vision_res_h * 3:
			brain.publish_video(_last_vision_pixels, vision_res_h, vision_res_w, 3, "color")
		if vision_steer:
			# Publish the VISION-derived bearing every tick (MotorEPM nav steers on it).
			var vcp := PackedFloat64Array()
			vcp.append(_vision_compass.x)
			vcp.append(_vision_compass.y)
			brain.publish_proprio(vcp, "vision_compass")

	var joints := PackedFloat64Array()
	for i in range(4): joints.append(clamp(hip1_angles[i] / HIP1_LIMIT,    -1.0, 1.0))
	for i in range(4): joints.append(clamp(hip2_angles[i] / HIP2_LIMIT,    -1.0, 1.0))
	for i in range(4): joints.append(clamp((knee_angles[i] - KNEE_REST) / 1.0,
											-1.0, 1.0))
	# --- 2026-08-02 · IMPORT I4: COLORED PROPRIOCEPTIVE NOISE --------------------------
	# Every Playful Machine legged experiment wires its controller through
	# ColorUniformNoise(0.1) — ~10% of range, TEMPORALLY CORRELATED — on every sensor,
	# plus ODE-level noise 0.01.  In homeokinesis the loop AMPLIFIES its own sensory
	# noise into behaviour: the noise is the seed of the motion, not a robustness test.
	# Our proprio channel has been noiseless, and our only noise is white, motor-side and
	# post-controller (explore_noise), which servo dynamics largely filter out.
	#
	# Measured motivation: at PM's own learning rate the pure-HK loop's activity peaks
	# around 14-20k ticks and then decays monotonically (12.0 -> 5.2 -> 2.95 steps per
	# 1000 ticks by 40k).  c_init sets where the loop STARTS and ctrl_lr how fast it
	# explores; neither keeps it excited.  A persistent correlated perturbation is what
	# PM's loop has and ours does not.
	#
	# Colored via a first-order (OU-like) filter: n <- (1-a)*n + a*U(-s,s), a = 1/tau.
	# tau=1 degenerates to white noise, so the knob spans both regimes.
	# sensor_noise_sigma = 0 (default) leaves this byte-identical.
	if sensor_noise_sigma > 0.0:
		var a: float = 1.0 / maxf(1.0, sensor_noise_tau)
		if _sensor_noise.size() != joints.size():
			_sensor_noise.resize(joints.size())
			_sensor_noise.fill(0.0)
		for i in range(joints.size()):
			_sensor_noise[i] = (1.0 - a) * _sensor_noise[i] \
				+ a * _sensor_noise_rng.randf_range(-sensor_noise_sigma, sensor_noise_sigma)
			joints[i] = clamp(joints[i] + _sensor_noise[i], -1.0, 1.0)
	brain.publish_proprio(joints, "joints")
	# 2026-08-11 (twin-gate M0.d) — [q, q̇] phase-space stream.  Computed AFTER
	# sensor noise so both halves describe the same egocentric view.
	# M0.d.2 (same day): q̇ is now an EMA of the per-tick delta, NOT the raw
	# delta — the v1 raw single-tick Δq was measured to be a noisy velocity
	# estimator (dyn-token chain WORSE than persistence, lift 0.94→0.64) while
	# the stream's structural effect (self-mass 0.72→0.55) confirmed the
	# phase-space concept.  τ ≈ 1/alpha ticks, matched to servo dynamics.
	var joints_dyn := PackedFloat64Array()
	joints_dyn.append_array(joints)
	if _qdot_ema.size() != joints.size():
		_qdot_ema.resize(joints.size())
		_qdot_ema.fill(0.0)
	if _prev_joints_pub.size() == joints.size():
		for i in range(joints.size()):
			_qdot_ema[i] = (1.0 - joints_dyn_vel_alpha) * _qdot_ema[i] \
				+ joints_dyn_vel_alpha * (joints[i] - _prev_joints_pub[i])
	for i in range(joints.size()):
		joints_dyn.append(_qdot_ema[i])
	_prev_joints_pub = joints.duplicate()
	brain.publish_proprio(joints_dyn, "joints_dyn")

	# Phase 7.9 — per-leg foot-Y for SynergyTimer touchdown detection.
	# Cheap (4 floats); consumers opt in by instantiating SynergyTimer.
	var feet_y_arr := PackedFloat64Array()
	for i in range(4):
		feet_y_arr.append(_lowers[i].global_transform.origin.y - L3 * 0.5)
	brain.publish_proprio(feet_y_arr, "feet_y")

	# ---- Markov-compliant twin of feet_y (2026-07-25) -------------------------
	# feet_y above is ABSOLUTE WORLD-Y of the foot — a god's-eye quantity no real
	# picrawler can sense, and the same violation that retired chassis_y_norm.  It is
	# nonetheless what MotorEPM's swing detector has always consumed (gating stance_lift
	# and every Cruse rule).
	#
	# This is the identical formula evaluated in the CHASSIS frame instead of the world
	# frame.  The resulting quantity — where the foot sits relative to the body — is one
	# the real robot CAN compute: the foot's pose relative to the chassis is fully
	# determined by the three joint encoder angles plus known link lengths (L1/L2/L3), so
	# this is forward kinematics from proprioception.  The chassis-transform inverse is
	# used only as an exact shortcut for that FK chain, not as extra information.
	#
	# Deliberately mirrors feet_y's own toe approximation (centre minus L3*0.5) rather
	# than a better one, so an A/B between the two isolates EXACTLY the god's-eye
	# component and nothing else.  Absolute offset/sign are irrelevant to the consumer:
	# the swing detector only ever compares this signal against its own moving average.
	#
	# CAVEAT: encoders would see commanded joint angles; this sees the achieved rigid-body
	# pose, so it includes joint compliance/slop the real encoders would miss.
	var _ch_inv: Transform3D = _chassis.global_transform.affine_inverse()
	var feet_y_body_arr := PackedFloat64Array()
	for i in range(4):
		feet_y_body_arr.append((_ch_inv * _lowers[i].global_transform.origin).y - L3 * 0.5)
	brain.publish_proprio(feet_y_body_arr, "feet_y_body")

	# ---- feet_y_gravity: IK ⊕ IMU ATTITUDE (2026-07-25) ----------------------------
	# The one property the god's-eye feet_y has that every legal replacement so far lacks:
	# it is GRAVITY-REFERENCED.  feet_y_body above is in the CHASSIS frame, so a foot at a
	# fixed body-frame position sits at different GRAVITATIONAL heights as the chassis
	# pitches and rolls; feet_y_ground inherits the same error because the belly ToF points
	# along the chassis down-axis.  Phase references were measured and came out at chance
	# (see the oracle design note), leaving gravity alignment as the live explanation.
	#
	# Gravity alignment is exactly what an accelerometer measures.  `up_body` is the
	# world-up axis expressed in the CHASSIS frame — i.e. the (negated) gravity vector a
	# body-mounted accelerometer reads directly.  Projecting the encoder-FK foot position
	# onto it gives the foot's height below the body measured ALONG GRAVITY:
	#   feet_y_gravity[i] = (foot position in body frame) · (gravity-up in body frame)
	# Both inputs are physically available (IMU + joint angles + link lengths), so this is
	# IK ⊕ IMU and nothing else.  It differs from the oracle by exactly the ABSOLUTE
	# chassis height, which is the god's-eye part and the only part we drop.
	var up_body: Vector3 = _ch_inv.basis * Vector3.UP
	var feet_y_grav_arr := PackedFloat64Array()
	for i in range(4):
		var foot_body: Vector3 = _ch_inv * _lowers[i].global_transform.origin
		feet_y_grav_arr.append(foot_body.dot(up_body) - L3 * 0.5)
	brain.publish_proprio(feet_y_grav_arr, "feet_y_gravity")

	# ---- feet_y_gravity_cmd / _fk: the SIM-TO-REAL test (2026-07-25) ----------------
	# feet_y_gravity above reads the ACHIEVED rigid-body pose.  The physical PiCrawler uses
	# hobby servos, which take an angle and report NOTHING — so real FK must run on the
	# COMMANDED angles, which are blind to load deflection (a planted leg sags while its
	# command still says "I am here").  The sim signal is therefore better-informed than the
	# hardware signal, in the dangerous direction, and this is what tests whether the win
	# survives what the robot can actually know.
	#
	# Both variants run through the SAME known-correct chain (_fk_leg, which calibrate mode
	# depends on being exact) so the FK's own conventions CANCEL out of the comparison:
	#   _fk  ← measured joint angles  → validation twin; must track feet_y_gravity closely,
	#                                    which is the self-check that the FK is wired right
	#   _cmd ← commanded servo targets → the hardware-honest signal
	# Attitude stays exact in both (modelling accelerometer error is a SEPARATE experiment,
	# kept separate so this is one lever).
	if _chassis_rest_xform != Transform3D():
		var rest_inv: Transform3D = _chassis_rest_xform.affine_inverse()
		var fk_arr := PackedFloat64Array()
		var cmd_arr := PackedFloat64Array()
		for i in range(4):
			# Effective joint-frame target: t = target*sign + origin (see servo_targets doc).
			var c1: float = servo_targets[servo_idx(i, 0)] * servo_signs[servo_idx(i, 0)] \
				+ servo_origins[servo_idx(i, 0)]
			var c2: float = servo_targets[servo_idx(i, 1)] * servo_signs[servo_idx(i, 1)] \
				+ servo_origins[servo_idx(i, 1)]
			var c3: float = servo_targets[servo_idx(i, 2)] * servo_signs[servo_idx(i, 2)] \
				+ servo_origins[servo_idx(i, 2)]
			var lower_fk:  Transform3D = _fk_leg(i, hip1_angles[i], hip2_angles[i], knee_angles[i])[2]
			var lower_cmd: Transform3D = _fk_leg(i, c1, c2, c3)[2]
			fk_arr.append((rest_inv * lower_fk.origin).dot(up_body) - L3 * 0.5)
			cmd_arr.append((rest_inv * lower_cmd.origin).dot(up_body) - L3 * 0.5)
		brain.publish_proprio(fk_arr,  "feet_y_gravity_fk")
		brain.publish_proprio(cmd_arr, "feet_y_gravity_cmd")
		# Mean |commanded − achieved| foot height: the servo tracking error in metres, i.e.
		# exactly the information the hardware does not have.  Diagnostic only.
		var e_sum: float = 0.0
		for i in range(4):
			e_sum += absf(cmd_arr[i] - fk_arr[i])
		_dbg_fk_cmd_err = e_sum / 4.0
		# VALIDATION: the measured-angle FK must reproduce the achieved-pose signal.  If this
		# is NOT small, the FK chain is miswired and fk_cmd_err above is measuring MY error
		# rather than the servos'.  Checked before drawing any conclusion from it.
		var v_sum: float = 0.0
		for i in range(4):
			v_sum += absf(fk_arr[i] - feet_y_grav_arr[i])
		_dbg_fk_valid_err = v_sum / 4.0

		# ---- HONEST IMU ATTITUDE (2026-07-26) ----------------------------------------
		# The attitude estimate is computed in _imu_substep() at the PHYSICS rate (240 Hz),
		# not here.  _step_one runs at TAU=50 Hz, and a real IMU samples far faster than a
		# control loop does — MPU-6050-class parts run 1-8 kHz, BNO085 fuses on-chip at
		# ~400 Hz — so sampling the sensor at the brain's rate both understates the hardware
		# and starves the filter.  The physics is already oversampled to 240 Hz to stop foot
		# tunnelling, and that headroom is free sensor bandwidth: 240 Hz sits squarely in the
		# realistic ODR band.  This block only CONSUMES the estimate.
		#   _cmd      exact attitude       (upper bound; NOT physically available)
		#   _cmd_acc  accelerometer ONLY   (shows the contamination)
		#   _cmd_imu  accel + gyro fused   (what the real robot can actually do)
		var up_acc: Vector3 = _up_acc_last if _up_acc_last.length() > 0.5 else up_body
		var imu6 := PackedFloat64Array()
		imu6.append(_accel_body_last.x); imu6.append(_accel_body_last.y); imu6.append(_accel_body_last.z)
		imu6.append(_gyro_body_last.x);  imu6.append(_gyro_body_last.y);  imu6.append(_gyro_body_last.z)
		brain.publish_proprio(imu6, "imu6")

		var acc_arr := PackedFloat64Array()
		var imu_arr := PackedFloat64Array()
		for i in range(4):
			var d1: float = servo_targets[servo_idx(i, 0)] * servo_signs[servo_idx(i, 0)] \
				+ servo_origins[servo_idx(i, 0)]
			var d2: float = servo_targets[servo_idx(i, 1)] * servo_signs[servo_idx(i, 1)] \
				+ servo_origins[servo_idx(i, 1)]
			var d3: float = servo_targets[servo_idx(i, 2)] * servo_signs[servo_idx(i, 2)] \
				+ servo_origins[servo_idx(i, 2)]
			var foot_b: Vector3 = rest_inv * _fk_leg(i, d1, d2, d3)[2].origin
			acc_arr.append(foot_b.dot(up_acc) - L3 * 0.5)
			imu_arr.append(foot_b.dot(_up_est_body) - L3 * 0.5)
		brain.publish_proprio(acc_arr, "feet_y_gravity_cmd_acc")
		brain.publish_proprio(imu_arr, "feet_y_gravity_cmd_imu")
		# (attitude-error diagnostics are set in _imu_substep, at the sensor's own rate)

	# ---- Markov-blanket-compliant posture sensing (2026-07-22) ----------------
	# The old height reflex read ABSOLUTE chassis world-Y (chassis_y_norm) — god's-
	# eye state no physical picrawler could sense.  These two topics are what the
	# REAL robot has: per-leg foot-contact switches + a downward belly rangefinder.
	# (1) foot_contact: TRUE physics touch per leg from the foot's contact monitor.
	var foot_contact_arr := PackedFloat64Array()
	for i in range(4):
		foot_contact_arr.append(1.0 if not _lowers[i].get_colliding_bodies().is_empty() else 0.0)
	brain.publish_proprio(foot_contact_arr, "foot_contact")
	# Cache the TRUE swing fraction (legs touching nothing) for the diag line.  This is
	# the ground truth for "is the foot on the ground".  It is NOT interchangeable with
	# the two proxies that have been used as if it were: `feet_y < stance_y_threshold` is
	# a WORLD-HEIGHT test (neither terrain- nor chassis-relative), and MotorEPM's
	# foot-height EMA is self-referential.  Any duty-factor claim must be checked here.
	var _n_air: int = 0
	for i in range(4):
		if foot_contact_arr[i] < 0.5: _n_air += 1
	_dbg_contact_swing = float(_n_air) / 4.0
	# (2) ground_clearance: body-down ToF/ultrasonic distance from the belly to the
	# nearest ground surface, normalized [0,1].  ~0 = belly ON a surface (dragging /
	# high-centered); higher = belly held up off the ground.  Replaces absolute Y.
	_dbg_gc_raw = _compute_ground_clearance()   # cache raw metres for the ramp-debug diag
	var clearance_arr := PackedFloat64Array()
	clearance_arr.append(clamp(_dbg_gc_raw / GROUND_CLEARANCE_STAND, 0.0, 1.0))
	brain.publish_proprio(clearance_arr, "ground_clearance")
	# Beacon magnitude.  Published every tick from the cached capture (the capture itself is
	# sub-rated for CPU), matching how ground_clearance and the vision frame are handled.
	# ⚠ Gated on the camera actually running.  Publishing 0.0 with no camera would be an
	# "exactly-round null" — indistinguishable from "the beacon is not visible" — which is
	# the failure shape that has produced false verdicts in this project repeatedly.  No
	# camera ⇒ no topic ⇒ a consumer that needs it fails loudly instead of reading zero.
	if publish_vision:
		var beacon_arr := PackedFloat64Array()
		beacon_arr.append(_beacon_frac)
		brain.publish_proprio(beacon_arr, "beacon")

	# ---- feet_y_ground: the LEGAL reconstruction of what feet_y smuggles in ----------
	# A/B showed the god's-eye feet_y (absolute world-Y) beats its body-relative twin
	# (net_z 3.76 vs 2.52).  The two differ by exactly the chassis's own vertical motion,
	# so what the world-frame signal carries — and the body-frame one loses — is the
	# BODY'S BOUNCE: a whole-body vertical phase reference.  Gating a knee push on that
	# synchronises it with when the body is actually loading its legs.
	#
	# Both halves of this sum are Markov-compliant, so the fusion is too:
	#   feet_y_body   — foot pose relative to the chassis (joint encoders + link lengths)
	#   _dbg_gc_raw   — belly ToF rangefinder, the sensor that already replaced the
	#                   god's-eye chassis_y_norm and solved the hump
	# Sum ≈ foot height above the ground the body is actually standing on — and unlike
	# world-Y it is TERRAIN-RELATIVE, so it should survive a slope where world-Y cannot.
	# Absolute offset is irrelevant: the swing detector only compares against its own mean.
	var feet_y_ground_arr := PackedFloat64Array()
	for i in range(4):
		feet_y_ground_arr.append(_dbg_gc_raw + feet_y_body_arr[i])
	brain.publish_proprio(feet_y_ground_arr, "feet_y_ground")
	# (3) upright: the chassis up-vector's alignment with gravity (basis.y.y): 1 =
	# perfectly upright, 0 = on its side, -1 = inverted.  A real IMU (accelerometer
	# gravity vector) gives this — compliant, always on.  Used to gate keyframe
	# baking on "am I in a valid posture" (don't learn from a flipped body).
	var upright_arr := PackedFloat64Array()
	upright_arr.append(_chassis.global_transform.basis.y.y)
	brain.publish_proprio(upright_arr, "upright")

	# 2026-06-03 — R1a per-leg foot-contact bucket signals (PremotorAI /
	# Premotor bucket_context_topic).  Discrete swing(0)/stance(1) bit per
	# leg, derived from foot_y < stance_y_threshold.  Same single-float
	# ProprioToken contract CruseCoordinator.publish_bucket_signals=true
	# emits, so existing Premotor.bucket_context_topic consumers work
	# unchanged — but without requiring CruseCoordinator in the graph.
	# Cheap (4 single-float ProprioTokens) and additive: consumers opt in
	# via the bucket_context_topic param on each Premotor; non-opted-in
	# Premotors ignore the topic.
	const LEG_BUCKET_NAMES := ["bucket_fl", "bucket_fr", "bucket_rl", "bucket_rr"]
	for i in range(4):
		var bp := PackedFloat64Array()
		bp.append(1.0 if feet_y_arr[i] < stance_y_threshold else 0.0)
		brain.publish_proprio(bp, LEG_BUCKET_NAMES[i])

	# 2026-06-05 — Phase H1 (Hector hierarchical) per-leg lift/plant events.
	# Fire `events.leg_lifted_<leg>` on rising foot crossing (planted→swing)
	# and `events.leg_planted_<leg>` on falling foot crossing (swing→planted).
	# Intensity = chassis_y_norm × max(0, velocity_toward_target) × upright_factor
	# — the "quality of this leg event in the current body context" reward
	# that the slow cognitive Premotor's MC actor-critic will use to learn
	# sequences (per neuroWalknet: events drive transitions, not a clock).
	# Hysteresis band prevents chatter when foot_y wobbles near threshold.
	const LEG_NAMES_H1: Array[String] = ["fl", "fr", "rl", "rr"]
	var stance_th_lo: float = stance_y_threshold * (1.0 - LEG_EVENT_HYSTERESIS_FRAC)
	var stance_th_hi: float = stance_y_threshold * (1.0 + LEG_EVENT_HYSTERESIS_FRAC)
	# Build the per-event intensity from current body state.  Use chassis_y_norm
	# clamped against target_height as the upright factor; project chassis
	# velocity onto target_compass (egocentric) for the "toward target" term.
	var event_chassis_y_norm: float = clamp(chassis_y / target_height, 0.0, 1.0)
	var v_xz_h1: Vector2 = Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z)
	# Note: target_compass is in BODY frame (x=forward, y=left), but chassis
	# velocity v_xz is in WORLD frame.  For "velocity toward target" we want
	# v_world · target_world_direction = v_xz.length() projected onto target
	# direction.  Since target_compass is unit-norm in body frame and yaw
	# rotates body→world, we approximate via cos(angle_between_v_world and
	# target_world).  Simpler: skip yaw rotation and use the body-frame v_xz
	# (which would require rotating v_xz by −yaw, but at low speed the body
	# is roughly aligned with motion direction so the approximation is OK).
	# For an honest dot-product, rotate v_xz into body frame.
	var cy_h1: float = cos(_last_yaw)
	var sy_h1: float = sin(_last_yaw)
	var v_body: Vector2 = Vector2(
		v_xz_h1.x * cy_h1 - v_xz_h1.y * sy_h1,
		v_xz_h1.x * sy_h1 + v_xz_h1.y * cy_h1)
	# 2026-06-06 — dot against TRUE compass: reward depends on ACTUAL motion toward
	# the real target, not toward the cognitive-biased perception.  Closes the
	# self-reinforcing illusion loop where cognitive bias gated its own reward.
	var v_toward_target: float = max(0.0, v_body.dot(_last_true_target_compass))
	# 2026-06-07 — F11 cognitive Lateral Voter feed: cache the signed
	# v_toward_target (NOT clamped at 0) so the population aggregator can
	# attribute credit to whichever member is currently dominant.
	_last_true_v_toward_target = v_body.dot(_last_true_target_compass)
	var event_intensity_base: float = event_chassis_y_norm * v_toward_target
	# Upright factor: penalise events that occur while tilted.
	var upright_factor: float = clamp(1.0 - chassis_tilt / 1.05, 0.0, 1.0)
	event_intensity_base *= upright_factor
	for i in range(4):
		# (a) HK step signal — track per-leg foot-Y excursion (peak-to-peak) over
		# the step; consumed + reset on touchdown as this step's sensorimotor
		# change.  Peak-to-peak (not summed Δ) so it measures lift HEIGHT, not
		# duration — a duration-independent "how real was this step".
		if homeo_step_gain > 0.0:
			if feet_y_arr[i] < float(_homeo_foot_min[i]): _homeo_foot_min[i] = feet_y_arr[i]
			if feet_y_arr[i] > float(_homeo_foot_max[i]): _homeo_foot_max[i] = feet_y_arr[i]
		var planted_now: bool = feet_y_arr[i] < stance_th_lo
		var lifted_now: bool = feet_y_arr[i] > stance_th_hi
		# Only act on definitive transitions (outside hysteresis band).
		# (event firing logic — see lift/plant blocks below)
		if planted_now and not _prev_foot_planted[i]:
			# (a) per-STEP homeokinetic reward on touchdown (step boundary).
			# reward = gain × step_change × regularity × upright.  Fired into the
			# events.hit accumulator so it composes with standing; graded so more
			# rhythmic stepping → more frequent events.hit → more dopamine.
			if homeo_step_gain > 0.0 and int(_homeo_last_td_tick[i]) >= 0:
				var hk_period: float = float(tick_counter - int(_homeo_last_td_tick[i]))
				var hk_reg: float = 1.0
				if int(_homeo_n_steps[i]) >= 2 and float(_homeo_period_ema[i]) > 1.0:
					hk_reg = exp(-absf(hk_period - float(_homeo_period_ema[i])) / float(_homeo_period_ema[i]))
				var hk_change: float = maxf(0.0, float(_homeo_foot_max[i]) - float(_homeo_foot_min[i]))
				var hk_tilt: float = _chassis_tilt(_chassis.global_transform.basis)
				var hk_upright: float = clampf(1.0 - hk_tilt / HOMEO_TILT_REF, 0.0, 1.0)
				# (c) cross-leg phase consistency — see declaration block for design.
				# Tracking always runs (homeo_phase_R telemetry live in OFF arms);
				# the factor only bites when homeo_phase_couple > 0.
				var hk_phase: float = 1.0
				var phase_sum: float = 0.0
				var phase_n: int = 0
				for j in range(4):
					if j == i:
						continue
					if int(_homeo_n_steps[j]) < 2 or float(_homeo_period_ema[j]) <= 1.0 or int(_homeo_last_td_tick[j]) < 0:
						continue
					# NOTE: 2.0*PI, NOT this file's TAU — picrawler_body.gd shadows
					# Godot's circle constant with TAU=0.02 (the 50 Hz brain-tick
					# period, ~line 46).  Using TAU here compressed every phase
					# angle into [0, 0.02) rad → all consistency factors pinned at
					# exactly 1.0 (caught by dbg probe 3, 2026-06-11).
					var dphi_ang: float = 2.0 * PI * fposmod(float(tick_counter - int(_homeo_last_td_tick[j])) / float(_homeo_period_ema[j]), 1.0)
					var pk: int = i * 4 + j
					var pex: float = float(_homeo_phase_ema_x[pk])
					var pey: float = float(_homeo_phase_ema_y[pk])
					var pR: float = sqrt(pex * pex + pey * pey)
					if pR > HOMEO_PHASE_R_FLOOR:
						phase_sum += 0.5 * (1.0 + (cos(dphi_ang) * pex + sin(dphi_ang) * pey) / pR)
						phase_n += 1
					_homeo_phase_ema_x[pk] = (1.0 - HOMEO_PERIOD_EMA_ALPHA) * pex + HOMEO_PERIOD_EMA_ALPHA * cos(dphi_ang)
					_homeo_phase_ema_y[pk] = (1.0 - HOMEO_PERIOD_EMA_ALPHA) * pey + HOMEO_PERIOD_EMA_ALPHA * sin(dphi_ang)
				var hk_phase_raw: float = 1.0
				if phase_n > 0:
					hk_phase_raw = phase_sum / float(phase_n)
				hk_phase = lerpf(1.0, hk_phase_raw, clampf(homeo_phase_couple, 0.0, 1.0))
				# homeo_c2 (2026-06-11) — SINGLE COMBINED quality norm.  c1
				# stacked two independent /EMA norms (upright, phase), each
				# floored at 0.1; compounding allowed 13×-ignition single-step
				# jackpots (max 184 vs ~14) that funded rhythm-at-any-cost
				# (n=3: resets 24→132, clean_gait 0.099→0.031, while
				# diag_coherence showed the FIRST nonzero coordination 0.351
				# s123).  Now: q = upright × phase is normalized by ONE EMA of
				# itself with ONE floor — cleaner/more-coordinated-than-average
				# steps still pay above ignition, but the two quality axes can
				# no longer multiply their amplifications.
				var hk_q: float = hk_upright * hk_phase
				_homeo_q_ema = (1.0 - HOMEO_UPRIGHT_EMA_ALPHA) * _homeo_q_ema + HOMEO_UPRIGHT_EMA_ALPHA * hk_q
				var hk_norm_q: float = hk_q
				if homeo_payout_norm > 0.0:
					hk_norm_q = lerpf(hk_q, hk_q / maxf(_homeo_q_ema, HOMEO_UPRIGHT_EMA_FLOOR), clampf(homeo_payout_norm, 0.0, 1.0))
				_homeo_phase_factor_last = hk_phase
				var hk_r: float = homeo_step_gain * hk_change * hk_reg * event_chassis_y_norm * hk_norm_q
				_homeo_upright_ema = (1.0 - HOMEO_UPRIGHT_EMA_ALPHA) * _homeo_upright_ema + HOMEO_UPRIGHT_EMA_ALPHA * hk_upright
				_hit_accumulator += hk_r
				_homeo_reward_last = hk_r
				if int(_homeo_n_steps[i]) == 0:
					_homeo_period_ema[i] = hk_period
				else:
					_homeo_period_ema[i] = (1.0 - HOMEO_PERIOD_EMA_ALPHA) * float(_homeo_period_ema[i]) + HOMEO_PERIOD_EMA_ALPHA * hk_period
			if homeo_step_gain > 0.0:
				_homeo_n_steps[i] = int(_homeo_n_steps[i]) + 1
				_homeo_last_td_tick[i] = tick_counter
				_homeo_foot_min[i] = feet_y_arr[i]
				_homeo_foot_max[i] = feet_y_arr[i]
			var intensity_p: float = event_intensity_base + 0.01
			brain.publish_event("leg_planted_" + LEG_NAMES_H1[i], intensity_p)
			# Aggregated event for the slow cognitive PremotorAI to consume
			# via aligned_event_name="leg_event" (any leg event = reward).
			brain.publish_event("leg_event", intensity_p)
			_prev_foot_planted[i] = true
			_leg_planted_count[i] += 1
			_leg_event_intensity_sum[i] += intensity_p
		elif lifted_now and _prev_foot_planted[i]:
			var intensity_l: float = event_intensity_base + 0.01
			brain.publish_event("leg_lifted_" + LEG_NAMES_H1[i], intensity_l)
			brain.publish_event("leg_event", intensity_l)
			_prev_foot_planted[i] = false
			_leg_lifted_count[i] += 1
			_leg_event_intensity_sum[i] += intensity_l

	# 2026-06-04 — Phase A3: constant per-leg ID single-float ProprioTokens.
	# Each Premotor opts in via bucket_context_topic=reality.proprio.leg_id_<leg>
	# with n_buckets=4 + bucket_bias_init_alt > 0 to get per-Premotor constant
	# logit bias differentiated by leg position.  Static signals (don't change
	# tick-to-tick) — minimum-viable symmetry break via the existing bucket
	# mechanism.  Cheap (4 single-float publishes); consumers opt in.
	const LEG_ID_NAMES := ["leg_id_fl", "leg_id_fr", "leg_id_rl", "leg_id_rr"]
	for i in range(4):
		var lip := PackedFloat64Array()
		lip.append(float(i))
		brain.publish_proprio(lip, LEG_ID_NAMES[i])

	# 2026-06-04 — Phase R4 creative: per-leg target alignment buckets.
	# Each leg has a body-frame angular position (FL +45°, FR -45°, RL +135°,
	# RR -135°). Compute the dot product of the leg's direction with
	# target_compass to get a per-leg "is the target in your wedge?" scalar.
	# Discretize into n_buckets=4: 0 = target behind, 1 = slightly behind,
	# 2 = slightly toward, 3 = target in your wedge. Each Premotor reads its
	# leg's bucket via bucket_context_topic=reality.proprio.tgt_align_<leg>.
	# Gives per-Premotor differentiation (different per leg AT EVERY TICK
	# because leg angular offsets differ) AND target awareness (signal
	# changes as body's orientation to target changes). The creative
	# combination Phase A/B/R1-3 missed.
	const LEG_BODY_ANGLES := [0.7853981633974483, -0.7853981633974483,
							  2.356194490192345, -2.356194490192345]  # +45/-45/+135/-135 deg
	const TGT_ALIGN_NAMES := ["tgt_align_fl", "tgt_align_fr",
							  "tgt_align_rl", "tgt_align_rr"]
	for i in range(4):
		var leg_dir_x: float = cos(LEG_BODY_ANGLES[i])
		var leg_dir_y: float = sin(LEG_BODY_ANGLES[i])
		var alignment: float = leg_dir_x * _last_target_compass.x \
							 + leg_dir_y * _last_target_compass.y
		# Discretize [-1, 1] → bucket 0..3 (each 0.5 wide)
		var bucket_idx: int = clamp(int((alignment + 1.0) * 2.0), 0, 3)
		var bp := PackedFloat64Array()
		bp.append(float(bucket_idx))
		brain.publish_proprio(bp, TGT_ALIGN_NAMES[i])

	# 2026-06-01 Stage 3.A — per-servo torque proprio. 12 floats normalized
	# to [-1, 1] against MAX_SERVO_TORQUE.  Reads `_prev_torque_*` which is
	# written by the motor block ON THE PREVIOUS PHYSICS TICK (motor runs
	# AFTER perception per tick order), so this is one-tick-delayed load
	# data.  Acceptable — load is integrated over 100s of ms in biology too.
	# Order matches the existing `joints` topic: hip1[0..3], hip2[0..3], knee[0..3].
	# Default-no-consumer: published every tick, but with no subscriber the
	# behavior is bit-identical to pre-3.A.  Stage 3.A.2 wires CruseCoordinator
	# to subscribe via a new `load_topic` param.
	var jtorque := PackedFloat64Array()
	for i in range(4): jtorque.append(clamp(_prev_torque_hip1[i] / MAX_SERVO_TORQUE, -1.0, 1.0))
	for i in range(4): jtorque.append(clamp(_prev_torque_hip2[i] / MAX_SERVO_TORQUE, -1.0, 1.0))
	for i in range(4): jtorque.append(clamp(_prev_torque_knee[i] / MAX_SERVO_TORQUE, -1.0, 1.0))
	brain.publish_proprio(jtorque, "joint_torque")
	var jload := PackedFloat64Array()
	for i in range(4): jload.append(_prev_load_hip1[i])
	for i in range(4): jload.append(_prev_load_hip2[i])
	for i in range(4): jload.append(_prev_load_knee[i])
	brain.publish_proprio(jload, "joint_load")
	# 2026-08-07 — PER-LEG VERTICAL LOAD (the CoG sensor).  Normalised by the body's
	# static weight so 1.0 ~ "this foot carries the whole robot" and the four values sum
	# to ~1 when all the weight is on the feet.  Egocentric and physically realisable
	# (foot FSR / servo current), so unlike fwd_v this is a LAWFUL brain input.
	var fload := PackedFloat64Array()
	for i in range(4): fload.append(clamp(_foot_load_ema[i] / _fl_norm(), -2.0, 2.0))
	brain.publish_proprio(fload, "foot_load")
	# 2026-08-03 — GROUND-FORCE / AUTHORITY instrument.  jtorque is already normalized to
	# +-1 against MAX_SERVO_TORQUE, so |t| -> 1 IS saturation.  Two questions this answers:
	#   tq_mag  — how hard are the legs actually pushing?  (the operator observed the pure-HK
	#             arm exerting far MORE ground force than the deployed gait ever did)
	#   tq_sat  — is the servo pinned?  Distinguishes "the controller is not using available
	#             authority" (our bug, fixable) from "the authority is not there" (hardware).
	for _ti in range(jtorque.size()):
		var _ta: float = absf(jtorque[_ti])
		_tq_mag_acc += _ta
		if _ta > 0.95: _tq_sat_acc += 1.0
		_tq_n += 1.0

	# Phase 7.13 v4.2 — body-state signal for CruseCoordinator gating.
	# chassis_y_norm in [0, 1].  CruseCoordinator subscribes optionally and
	# multiplies effective bias magnitude by this value, so the coordinator goes
	# silent when the body falls (no gait coordination on a belly-down robot).
	# Use trapezoid normalization regardless of reward_shape — we want a smooth
	# "is the body upright" signal, not the inverted-U reward proxy.
	var chassis_y_norm_signal: float = clamp(chassis_y / target_height, 0.0, 1.0)
	var body_state_arr := PackedFloat64Array()
	body_state_arr.append(chassis_y_norm_signal)
	brain.publish_proprio(body_state_arr, "chassis_y_norm")

	if publish_tilt:
		# 4-D tilt vector for the vestibular tilt EPM.  Extract pitch
		# and roll from the chassis basis (level: pitch=roll=0).  Use
		# unit-circle encoding [sin, cos] per axis so the EPM sees a
		# continuous representation across the full angle range
		# (vs raw radians which wrap at ±π).
		var basis: Basis = chassis_xform.basis
		var euler: Vector3 = basis.get_euler()    # (pitch, yaw, roll) per Godot XYZ
		var pitch: float = euler.x
		var roll:  float = euler.z
		var tilt_arr := PackedFloat64Array()
		tilt_arr.append(sin(pitch))
		tilt_arr.append(cos(pitch))
		tilt_arr.append(sin(roll))
		tilt_arr.append(cos(roll))
		brain.publish_proprio(tilt_arr, "tilt")

	# ---- 3. Reward / failure ----
	# Graded standing reward: events.hit fires at a rate proportional to
	# chassis_y via a fractional duty-cycle accumulator.  At full standing
	# height (chassis_y = STANDING_CHASSIS_Y) the rate is 0.2 hits/tick —
	# below the 0.21 hits/tick dopamine-saturation invariant for default
	# gains (docs/v4_brain_derivation §4.2).  The adaptive-baseline EMA
	# (Phase 6.5.3.10, α=0.001) self-zero-centers atop this so learning
	# is driven by RISING chassis_y, not absolute height.
	#
	# 2026-06-02 — advance the local CPG phase cache one tick.  Re-synced
	# from the brain at every diag tick (every 20 physics ticks); between
	# syncs we extrapolate at 2π/period_ticks per tick.  Drift between
	# syncs ≤ 1/3 cycle, EMA reward absorbs it.  No-op until the first
	# diag tick sets _cpg_present=true.
	if _cpg_present:
		# NOTE: this file's TAU=0.02 (brain tick); angular 2π must be 2*PI
		# (see [[picrawler-body-tau-shadow]] memory).
		_cpg_phase_now = fposmod(_cpg_phase_now + (2.0 * PI) / float(_cpg_period_ticks), 2.0 * PI)
	var fell: bool = chassis_y < FAIL_HEIGHT or chassis_tilt > fail_tilt_rad

	# Phase 7.16 — Continuous level-chassis bonus.  Deliberately OUTSIDE the
	# `if not fell:` block so the body has a continuous gradient back to
	# upright even when fallen.  Shape: PLATEAU within walking-tilt budget
	# (full bonus 0-30° by default) then linear DECAY to zero by tilt_scale
	# (90° default), multiplicatively scaled by chassis_y_norm so a body
	# lying belly-flat (y=0) earns zero — prevents the lying-flat local
	# optimum trap discovered in v1 smoke.  Multiplicative chassis_y is a
	# continuous gradient, NOT a binary gate.
	if level_chassis_rate > 0.0 and is_finite(chassis_tilt) and is_finite(chassis_y):
		var tilt_abs: float = abs(chassis_tilt)
		var level_tilt_factor: float
		if tilt_abs <= level_chassis_walking_budget:
			level_tilt_factor = 1.0
		else:
			var span: float = level_chassis_tilt_scale - level_chassis_walking_budget
			level_tilt_factor = clamp(1.0 - (tilt_abs - level_chassis_walking_budget) / max(span, 1e-6), 0.0, 1.0)
		var level_height_factor: float = clamp(chassis_y / target_height, 0.0, 1.0)
		# 2026-05-31 — gate on reward_min_height to match standing /
		# gated_walk channels.  Without this, when chassis_y crashes below
		# reward_min_height (so standing and gated go dark), level_chassis
		# alone keeps paying — body learned a belly-flop attractor that
		# earns ~0.02 hits/tick (saturating DA) by lying flat with low
		# tilt.  Gating closes that trap while preserving the tilt-
		# correction usefulness above the floor.  Diagnosed in overnight
		# UI run 2026-05-31 (handtuned_v1_gaitcycle, 3 sim hours, body at
		# chassis_y≈0.035 with reward_min_height=0.06, 86% of all reward
		# came from level_chassis alone).
		if reward_min_height > 0.0 and chassis_y < reward_min_height:
			level_height_factor = 0.0
		var level_intensity: float = level_tilt_factor * level_height_factor * level_chassis_rate
		# Defensive: clamp + finite-check before publish to survive physics-solver
		# degeneracies during extreme overrotation (e.g., when chassis basis briefly
		# becomes near-singular and tilt computation returns NaN/Inf).
		if is_finite(level_intensity) and level_intensity > 1e-6 and level_intensity < 1.0:
			brain.publish_event("hit", level_intensity)
			_hit_delta_level_chassis += level_intensity
			_hit_cum_level_chassis   += level_intensity

	if not fell:
		# Reward shape — dispatches on `reward_shape` knob (v6.0.b.12).
		# Trapezoid (default): ramp [0, target], flat plateau above.
		# Inverted-U: peak at peak_height, drops off in BOTH directions.
		# See @export comments at top of file for the design rationale.
		var chassis_y_norm: float
		# V9 — resolve height reference (chassis center vs full body CoG).
		# Reward-only; everything else uses chassis_y directly.
		var chassis_y_reward: float = chassis_y
		if height_reference == "body_cog":
			chassis_y_reward = _compute_body_cog_y()
		# V9c — resolve effective peak: static @export or dynamic per-tick mean
		# knee height of PLANTED legs only (foot below stance_y_threshold).
		# Lifted-leg knees are excluded so the brain can't game the reward by
		# raising legs skyward to inflate the mean knee height (an attractor
		# observed in V9c v1: FL+RR push down to lift chassis high, FR+RL
		# dangle in the air with knees sky-high — perverse incentive).  With
		# stance-only knees, peak follows the legs ACTUALLY SUPPORTING the
		# body; lifting a leg removes its knee from the peak calculation.
		# Fallback to static peak_height when no leg is currently planted.
		var eff_peak: float = peak_height
		if peak_height_mode == "knee_relative":
			var stance_knee_sum: float = 0.0
			var stance_count: int = 0
			for _i in range(4):
				var foot_y_lg: float = _lowers[_i].global_transform.origin.y - L3 * 0.5
				if foot_y_lg < stance_y_threshold:
					stance_knee_sum += _lowers[_i].global_transform.origin.y
					stance_count += 1
			if stance_count > 0:
				eff_peak = stance_knee_sum / float(stance_count)
			# else: keep static peak_height as the fallback (all legs lifted)
		if reward_shape == "inverted_u":
			chassis_y_norm = max(0.0, 1.0 - abs(chassis_y_reward - eff_peak) / band_width)
		else:
			chassis_y_norm = clamp(chassis_y_reward / target_height, 0.0, 1.0)
		# V9 — anti-belly-flop floor.  When the inverted_u band centred low
		# (peak_height ~ 0.06) is paired with reward_min_height ~ 0.05, the
		# body can't satisfy the band by collapsing to chassis_y ≈ 0
		# (belly-on-floor).  Default 0.0 = disabled.  Uses chassis_y_reward
		# so under height_reference='body_cog' the floor is on body CoG too.
		if reward_min_height > 0.0 and chassis_y_reward < reward_min_height:
			chassis_y_norm = 0.0
		# Phase 6.16 — multiplicative tilt factor.  When tilt_target_rad>0,
		# gate the standing reward on uprightness too: tilt=0 → full,
		# tilt=tilt_target_rad → zero.  Body must be at target height AND
		# within tilt_target of vertical to earn full reward.  Default
		# tilt_target_rad=0 disables (= pre-6.16 height-only reward).
		if tilt_target_rad > 0.0:
			var tilt_norm: float = clamp(1.0 - chassis_tilt / tilt_target_rad, 0.0, 1.0)
			chassis_y_norm *= tilt_norm
		# Phase 7.5.R — standing_baseline_factor scales down the pure-
		# standing reward density.  Default 1.0 preserves legacy 0.20
		# hits/tick at full upright.  Set < 1 to make standing-only
		# less rewarding without removing it entirely (cold-start
		# bootstrap preserved).  Composes multiplicatively with
		# chassis_y_norm and tilt_norm so the existing pose-shape
		# signals are unchanged.
		# Fix 2 — EMA-smooth chassis_y_norm so transient leg-lift dips (the
		# swing-curl pose that initiates the insect stride) don't instantaneously
		# crash the standing reward stream.  alpha=1.0 (default) → ema follows
		# raw → bit-identical to legacy.
		_chassis_y_norm_ema = (1.0 - standing_reward_ema_alpha) * _chassis_y_norm_ema \
							+ standing_reward_ema_alpha * chassis_y_norm
		var y_norm_for_reward: float = _chassis_y_norm_ema if standing_reward_ema_alpha < 1.0 else chassis_y_norm
		var standing_add: float = y_norm_for_reward * HIT_RATE_AT_STANDING * standing_baseline_factor
		_hit_accumulator    += standing_add
		_hit_delta_standing += standing_add

		# 2026-06-02 Stage 2 walking paradigm — step-quality reward (v3).
		# Per-leg correlation of foot_y motion against the CPG phase.
		# Each leg gets credit for being LIFTED during its CPG swing
		# window AND PLANTED during its stance window.  Couples reward
		# directly to the foundational CPG substrate
		# ([[v6-cpg-is-foundation]]).
		#
		#   foot_dev    = foot_y - stance_y_threshold/2     # midline-centred
		#   cpg_drive   = sin(cpg_phase + leg_offset[i])    # ∈ [-1, +1]
		#   phase_match = clamp(2 × foot_dev/step_lift_scale × cpg_drive, 0, 1)
		#
		# Stuck foot (foot_dev=0)           → match=0
		# Anti-phase cycling                → product < 0, clamp to 0
		# In-phase cycling at amplitude=L/2 → match averages 0.5 over cycle
		# Larger amplitude / cleaner phase  → match saturates at 1.0
		#
		# Composes additively with standing into _hit_accumulator.  Upright
		# factor fades to zero between chassis_y_norm 0.5→0.0 so falls
		# earn nothing.
		#
		# History: v1 (boolean lift_state at threshold=0.08) was
		# unreachable from natural standing wobble (channel cum=0 in
		# 20min pilot).  v2 (continuous lift_ema × plant_ema disjoint
		# bands) fired at 0.3% reward share but paired A/B n=2 showed
		# −27% longest_upright with zero policy-level signal on chassis_y
		# /da/pre_w.  v3 routes the reward against the CPG phase the
		# per-servo Premotors already consume as input — alignment of
		# action with phase is then the *natural* policy gradient.
		if step_quality_reward_gain > 0.0 and _cpg_present:
			var step_quality_sum: float = 0.0
			var midline: float = 0.5 * stance_y_threshold
			var scale: float = max(0.001, step_lift_scale)
			for i_sq in range(4):
				var foot_y_sq: float = _lowers[i_sq].global_transform.origin.y - L3 * 0.5
				var foot_dev: float = foot_y_sq - midline
				var leg_off: float = 0.0
				if i_sq < _cpg_leg_offsets_now.size():
					leg_off = float(_cpg_leg_offsets_now[i_sq])
				var cpg_drive: float = sin(_cpg_phase_now + leg_off)
				var phase_match: float = clamp(2.0 * (foot_dev / scale) * cpg_drive, 0.0, 1.0)
				_step_phase_match_ema[i_sq] = (1.0 - step_quality_ema_alpha) * float(_step_phase_match_ema[i_sq]) + step_quality_ema_alpha * phase_match
				step_quality_sum += float(_step_phase_match_ema[i_sq])
			var upright_sq: float = clamp((chassis_y_norm - 0.5) * 2.0, 0.0, 1.0)
			var step_add: float = (step_quality_sum / 4.0) * upright_sq * step_quality_reward_gain
			_hit_accumulator        += step_add
			_hit_delta_step_quality += step_add

		var hit_fired_this_tick: bool = false
		while _hit_accumulator >= 1.0:
			brain.publish_event("hit", 1.0)
			_hit_accumulator -= 1.0
			hit_fired_this_tick = true
		# Phase 6.7 — alive heartbeat tied to reward firing, gated by env
		# var OGMA_PICRAWLER_ALIVE_HEARTBEAT=1.  Emit events.alive ONCE per
		# tick if any events.hit fired (standing or walking arm), making
		# HomeostaticDrive.alive_pulse oscillate at the rhythm of reward:
		# rich reward periods → frequent replenish → low urgency; quiet
		# stuck periods → no replenish → urgency climbs.  That variability
		# is what HomeokineticExploration's gate consumes (fires when
		# current Δurgency is much smaller than the typical past Δ — the
		# signature of a stuck attractor).
		#
		# Gated on by C.5 variant runs only.  Baseline runs (without
		# HomeokineticExploration in topology) DON'T set the env var,
		# so the alive_pulse channel decays to 0 once and stays there
		# forever — exactly the pre-Phase-6.7 behaviour.  Preserves
		# bit-identity with the Stage B / Stage C baseline.
		if _publish_alive_heartbeat and hit_fired_this_tick:
			brain.publish_event("alive", 1.0)
		episode_alive_ticks += 1
		cumulative_alive_ticks += 1
		if cumulative_alive_ticks > best_cumulative_alive_ticks:
			best_cumulative_alive_ticks = cumulative_alive_ticks

		# Over-target penalty arm of the trapezoid.  Engages when
		# chassis_y exceeds target_height + grace.  Intensity ramps
		# linearly from 0 (at the grace boundary) to height_penalty_gain
		# (at grace + scale).  Uses events.miss, so couples to the
		# existing NeurochemState DA-drop + 5HT-drop pathway.
		if height_penalty_gain > 0.0:
			var penalty_floor: float = target_height + height_penalty_grace
			if chassis_y > penalty_floor:
				var overage:   float = chassis_y - penalty_floor
				var intensity: float = clamp(overage / height_penalty_scale, 0.0, 1.0) \
									   * height_penalty_gain
				brain.publish_event("miss", intensity)

		# Walking reward (v6.0.b.17) — events.hit rate ∝ RADIAL-OUTWARD
		# velocity (chassis_velocity_xz · r̂, where r̂ points from origin
		# to chassis).  Going outward = positive reward; going inward or
		# circling = zero (clamped ≥ 0).  Composed with the height reward
		# via independent fractional accumulator; both fire into
		# events.hit so the brain receives a single combined DA stream.
		#
		# Why radial-outward over velocity magnitude (the v6.0.b.13
		# earlier design): magnitude-only is satisfied by jitter and
		# circling — the brain reward-hacks by oscillating in place
		# instead of translating.  Radial-outward has a directional
		# gradient: only motion that increases distance-from-origin is
		# rewarded.  Below the 0.05 m radius epsilon there's no defined
		# direction, so the brain gets standing reward only and must
		# commit to a direction by accident first.
		# Update walking PB whenever the body is upright.  Done before
		# the gating on walk_hit_rate so the metric tracks even when the
		# walking reward channel is disabled (e.g. standing-only stages
		# in the curriculum).
		var _pos_pb: Vector3 = _chassis.global_transform.origin
		var _d_pb: float = Vector2(_pos_pb.x, _pos_pb.z).length()
		if _d_pb > max_distance_from_origin:
			max_distance_from_origin = _d_pb
		if walk_hit_rate > 0.0:
			var pos: Vector3 = _chassis.global_transform.origin
			var r_xz: Vector2 = Vector2(pos.x, pos.z)
			var r_len: float = r_xz.length()
			var v_xz: Vector2 = Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z)
			# Phase 6.8 — walk reward mode dispatch.
			#   radial:                  v · r̂  (outward → hit, inward → nothing)
			#   radial_penalize_inward:  v · r̂  (outward → hit, inward → MISS, symmetric magnitude)
			#   total_speed:             |v_xz| (any horizontal direction → hit)
			# Switch via the walk_reward_mode @export or curriculum override.
			# The penalize-inward mode forces the body to commit to a heading:
			# idle drift / wandering back toward centre costs DA via events.miss,
			# so REINFORCE pressures the policy to specialise on one direction.
			# That asymmetric pressure is the proposed mechanism for an emergent
			# "front" on the 4-fold-symmetric body.
			# Phase 6.15 — track raw current_speed + EMA-smoothed _speed_ema.
			# _speed_ema is the value reward + PR + sustained-counter use; raw
			# current_speed is kept for HUD/JSONL telemetry so the user can
			# see the transient vs smoothed split.
			current_speed = v_xz.length()
			_speed_ema = (1.0 - SPEED_EMA_ALPHA) * _speed_ema + SPEED_EMA_ALPHA * current_speed

			var walk_v: float = 0.0
			if walk_reward_mode == "total_speed":
				# EMA-smoothed magnitude.  Single-tick spikes from fall
				# transients don't register; sustained gait does.
				walk_v = _speed_ema
			elif walk_reward_mode == "to_target":
				# Phase 7.x — walk_over_there.  Project velocity onto the
				# chassis→target direction.  Positive = approaching →
				# _walk_accumulator (hits).  Negative = receding →
				# _walk_miss_accumulator (misses, same penalize-inward
				# shape but anchored to the target pyramid rather than
				# origin).
				if walk_target_idx >= 0:
					var to_t: Vector2 = walk_target_pos - r_xz
					var d_t: float = to_t.length()
					if d_t > 1e-3:
						walk_v = v_xz.dot(to_t / d_t)
			else:
				# "radial" + "radial_penalize_inward" share the projection.
				# Phase 7.x — dropped the r_len > 0.05 dead zone (was a 5 cm
				# circle around origin where no radial reward fired, letting
				# the body spin freely at centre).  Now uses a tiny epsilon
				# for numerical safety only.
				if r_len > 1e-3:
					walk_v = v_xz.dot(r_xz / r_len)
			if walk_reward_mode == "radial_penalize_inward" or walk_reward_mode == "to_target":
				var v_norm_signed: float = clamp(walk_v / walk_target_velocity, -1.0, 1.0)
				if v_norm_signed > 0.0:
					var walk_add: float = v_norm_signed * walk_hit_rate
					_walk_accumulator  += walk_add
					_hit_delta_walking += walk_add
				elif v_norm_signed < 0.0:
					_walk_miss_accumulator += -v_norm_signed * walk_hit_rate
			else:
				var v_norm: float = clamp(walk_v / walk_target_velocity, 0.0, 1.0)
				var walk_add2: float = v_norm * walk_hit_rate
				_walk_accumulator  += walk_add2
				_hit_delta_walking += walk_add2

			# Phase 7.7 / 7.10b — per-leg stance factors (computed once,
			# reused by per_leg_credit_gain block + phase_contrast factor).
			var stance_factors: Array = [0.0, 0.0, 0.0, 0.0]
			for i_leg in range(4):
				var foot_y_lg: float = _lowers[i_leg].global_transform.origin.y - L3 * 0.5
				stance_factors[i_leg] = clamp(1.0 - foot_y_lg / stance_y_threshold, 0.0, 1.0)

			# 2026-05-30 — Gait-cycle reward detector.  Runs inside the
			# `if not fell:` guard so foot positions + walk_v are valid.
			# When the body falls mid-cycle, the cycle naturally times
			# out (detector pauses, window expires) and resets on next
			# touchdown after recovery.  No-op when gain=0.
			if gait_cycle_reward_gain > 0.0:
				# Effective thresholds — static safety floor OR adaptive
				# (K · EMA of observed cycle statistics) once past warmup,
				# whichever is larger.  Bars "follow the body up" as
				# performance improves but never drop below the user's
				# safety floor.  See @export comments for K and warmup.
				var eff_min_progress: float = gait_cycle_min_progress
				var eff_max_backward: float = gait_cycle_max_backward
				if gait_cycle_adaptive_thresholds and _gait_cycle_attempts_total >= gait_cycle_warmup_cycles:
					eff_min_progress = max(eff_min_progress, gait_cycle_progress_K * _gait_cycle_progress_ema)
					eff_max_backward = max(eff_max_backward, gait_cycle_wobble_K   * _gait_cycle_wobble_ema)
				_gait_cycle_eff_min_progress = eff_min_progress
				_gait_cycle_eff_max_backward = eff_max_backward
				# Integrate signed displacement in the active direction
				# while a cycle is in flight.  walk_v already encodes the
				# walk_reward_mode-dependent sign (radial outward, target
				# approach, or |v|).  For total_speed mode walk_v ≥ 0 so
				# the backward-excursion guard is inert (by design — that
				# mode has no preferred direction).
				if _gait_cycle_active:
					_gait_cycle_displacement += walk_v / float(Engine.physics_ticks_per_second)
					if _gait_cycle_displacement > _gait_cycle_max_displacement:
						_gait_cycle_max_displacement = _gait_cycle_displacement
					var backward: float = _gait_cycle_max_displacement - _gait_cycle_displacement
					if backward > _gait_cycle_max_backward_seen:
						_gait_cycle_max_backward_seen = backward
					if backward > eff_max_backward:
						_gait_cycle_aborted_wobble += 1
						_gait_cycle_consecutive_good = 0
						_end_gait_cycle(false)
					elif tick_counter - _gait_cycle_start_tick > gait_cycle_window_ticks:
						_gait_cycle_aborted_timeout += 1
						_gait_cycle_consecutive_good = 0
						_end_gait_cycle(false)
				# Detect per-leg touchdown via binary contact transition
				# (stance_factor crossing 0.5 from below).  De-bounces
				# naturally on a single threshold cross.
				for i_leg in range(4):
					var in_contact: bool = stance_factors[i_leg] > 0.5
					if in_contact and not _foot_was_in_contact[i_leg]:
						# Touchdown event — start a fresh cycle on the
						# first touchdown after a quiescent period; or
						# mark this leg in the in-flight cycle.
						if not _gait_cycle_active:
							_gait_cycle_active = true
							_gait_cycle_start_tick = tick_counter
							_gait_cycle_displacement = 0.0
							_gait_cycle_max_displacement = 0.0
							_gait_cycle_max_backward_seen = 0.0
							_gait_cycle_legs_touched = [false, false, false, false]
						_gait_cycle_legs_touched[i_leg] = true
						# Cycle complete when all 4 legs have touched
						# down at least once inside the window.
						if _gait_cycle_legs_touched[0] and _gait_cycle_legs_touched[1] \
								and _gait_cycle_legs_touched[2] and _gait_cycle_legs_touched[3]:
							if _gait_cycle_displacement >= eff_min_progress:
								_gait_cycle_good_count += 1
								_gait_cycle_consecutive_good += 1
								if _gait_cycle_consecutive_good > _gait_cycle_consecutive_good_max:
									_gait_cycle_consecutive_good_max = _gait_cycle_consecutive_good
								# Pulse only after the streak reaches the configured
								# limit-cycle-continuity bar.  Once at/above N, every
								# additional good cycle keeps paying — sustained gait
								# earns sustained reward.  Default N=1 fires on every
								# good cycle (v1/v2/v3 back-compat).
								if _gait_cycle_consecutive_good >= gait_cycle_consecutive_required:
									brain.publish_event("hit", gait_cycle_reward_gain)
									_gait_cycle_pulses_fired += 1
									_hit_delta_gait_cycle += gait_cycle_reward_gain
									_hit_cum_gait_cycle   += gait_cycle_reward_gain
								_end_gait_cycle(true)
							else:
								_gait_cycle_aborted_low_progress += 1
								_gait_cycle_consecutive_good = 0
								_end_gait_cycle(false)
					_foot_was_in_contact[i_leg] = in_contact
			# Inter-diagonal contrast.  Diag A = FL+RR (indices 0, 3),
			# Diag B = FR+RL (indices 1, 2).  |a_stance - b_stance| ∈ [0,1].
			var diag_a_stance: float = (stance_factors[0] + stance_factors[3]) * 0.5
			var diag_b_stance: float = (stance_factors[1] + stance_factors[2]) * 0.5
			var inter_diag_contrast: float = abs(diag_a_stance - diag_b_stance)
			# EMA for telemetry/inspector visibility.
			_phase_contrast_ema = (1.0 - 0.05) * _phase_contrast_ema + 0.05 * inter_diag_contrast

			# Phase 7.5.R — multiplicative-gating bonus (Option 2).  Fires
			# an EXTRA hit stream proportional to chassis_y_norm × v_norm,
			# densifying forward-while-upright reward without flat-out
			# erasing the standing baseline.  Both signals must be > 0 to
			# contribute (fails closed if body falls, fails closed if not
			# moving outward).  Uses positive radial-outward component
			# only, matching the asymmetric reward gradient picrawler's
			# walk_reward_mode=radial assumes.
			#
			# Phase 7.10b — inter-diagonal phase-contrast multiplier.
			# factor = (1 - gain) + gain × inter_diag_contrast
			# gain=0: factor=1 (legacy gated bonus unchanged)
			# gain=0.7, contrast=1.0: factor=1.0 (full reward — proper trot)
			# gain=0.7, contrast=0.0: factor=0.3 (30% — bound/pronk gets less)
			if gated_walk_bonus_rate > 0.0:
				# Fix 1 — gate velocity source: radial (legacy) vs body_forward.
				# body_forward = v_xz · Vector2(sin yaw, cos yaw) — chassis-frame
				# forward velocity.  Paddling without hip1 reciprocation produces
				# near-zero sustained body_forward; insect-stride does.
				var gate_velocity: float = walk_v
				if gated_walk_velocity_mode == "body_forward":
					var fwd_world: Vector2 = Vector2(sin(yaw), cos(yaw))
					gate_velocity = v_xz.dot(fwd_world)
				var gate_v: float = clamp(max(0.0, gate_velocity) / walk_target_velocity, 0.0, 1.0)
				var phase_factor: float = 1.0
				if phase_contrast_gain > 0.0:
					phase_factor = (1.0 - phase_contrast_gain) + phase_contrast_gain * inter_diag_contrast
				var gated_add: float = chassis_y_norm * gate_v * phase_factor * gated_walk_bonus_rate
				_hit_accumulator  += gated_add
				_hit_delta_gated  += gated_add

			# Phase 7.7 — per-leg reward decomposition.  Each leg in
			# stance during outward chassis motion fires events.hit_leg_<leg>
			# with intensity proportional to its contribution.  Premotors
			# configured with aligned_event_name="hit_leg_<leg>" receive
			# ONLY their leg's credit — so the brain can differentiate
			# which legs actually helped vs which were swinging or
			# uninvolved.  Cost: one EnvEvent publish per leg per tick
			# (only when intensity > epsilon).
			if per_leg_credit_gain > 0.0:
				var leg_v: float = clamp(max(0.0, walk_v) / walk_target_velocity, 0.0, 1.0)
				# Only fire when chassis is upright AND moving outward.
				if chassis_y_norm > 0.5 and leg_v > 0.01:
					var base_intensity: float = chassis_y_norm * leg_v * per_leg_credit_gain
					for i in range(4):
						# Reuse stance_factors array computed above (Phase 7.10b refactor).
						var leg_intensity: float = base_intensity * float(stance_factors[i])
						if leg_intensity > 0.001:
							brain.publish_event("hit_leg_" + LEG_NAMES[i], leg_intensity)

			# Phase 7.x — walk_over_there arrival check.  When a target
			# is active and the chassis is within visit_radius, fire a
			# bonus events.hit pulse, log the arrival, then rotate to a
			# new random pyramid so the body continues navigating.
			if walk_target_idx >= 0:
				var dist_to_target: float = (walk_target_pos - r_xz).length()

				# Phase 7.12 — progress-PB reward.  Fires events.hit only
				# when distance to target hits a new minimum (a personal
				# best).  Brain cannot Goodhart this by orbiting — it
				# must actually close distance beyond previous record.
				# PB resets when target rotates (in _select_random_pyramid_target).
				# Phase 7.12 v2: tightened uprightness gate + multiplicative scaling.
				# v1 (chassis_y_norm > 0.5 GATE only) Goodharted: body collapsed forward,
				# rode chassis_y to 0.05m, and earned progress reward for falling.  −7.56σ on
				# chassis_y_mean_late at n=5×25min.  v2 requires near-target chassis_y AND
				# multiplies intensity by chassis_y_norm so a half-collapsed body earns near-zero
				# progress even when distance closes.
				if progress_reward_gain > 0.0 and chassis_y_norm > 0.9:
					# 2026-06-09 sentinel-seed fix: when gate opens with PB still
					# at sentinel (1e9) — true at fresh target rotation, at stage-2
					# auto-advance from gain=0, and at live slider movement in
					# experiment mode — seed PB from current distance and skip
					# this tick.  Without this guard delta_pb ≈ 1e9 on the very
					# first eligible tick, which NaNs the Premotor REINFORCE
					# update on the direct-publish path and FREEZES Godot on
					# the accumulator path (drain loop runs ~10¹⁰ iterations).
					if _min_dist_to_target_pb >= 1e8:
						_min_dist_to_target_pb = dist_to_target
					elif dist_to_target < _min_dist_to_target_pb - progress_reward_min_delta:
						var delta_pb: float = _min_dist_to_target_pb - dist_to_target
						_min_dist_to_target_pb = dist_to_target
						var progress_intensity: float = delta_pb * progress_reward_gain * chassis_y_norm
						# Path (a) 2026-06-09: route into _hit_accumulator (matches
						# standing_add / gated_add) instead of direct publish.
						# NeurochemState.cpp:216 ignores EnvEvent::intensity and
						# pulses dopamine per event count — so direct publish made
						# progress_reward_gain a binary NC-layer gate (any nonzero
						# slider → full DA pulse on every new PB tick).  Accumulator
						# path drains at canonical intensity=1.0 (line 3905) so the
						# slider becomes a true frequency knob: small gain → few
						# DA pulses, dose-response restored.  See memory note
						# nc-hit-intensity-discarded.
						_hit_accumulator    += progress_intensity
						_hit_delta_progress += progress_intensity
						_hit_cum_progress   += progress_intensity

				# 2026-06-13 — arrival is SURFACE-relative (switch on touch), not
				# center-relative.  dist_to_target is chassis→center, but the
				# pyramid's solid body (bounding radius) blocks the chassis from
				# ever reaching within walk_visit_radius of the center if the
				# pyramid is larger than that radius — so a center test never
				# fires on a big pyramid even when the robot is touching it.  Add
				# the target's bounding radius so walk_visit_radius is the
				# clearance-from-surface that counts as "arrived".
				var eff_visit_radius: float = walk_visit_radius
				if walk_target_idx < _pyramid_xz_radii.size():
					eff_visit_radius += float(_pyramid_xz_radii[walk_target_idx])
				if dist_to_target < eff_visit_radius:
					walk_visit_count += 1
					brain.publish_event("hit", walk_visit_bonus_intensity)
					print("PicrawlerBody: walk_over_there visit #%d at pyramid %d (dist %.2f < %.2f)" % [
						walk_visit_count, walk_target_idx, dist_to_target, eff_visit_radius])
					_select_random_pyramid_target()

			# Phase 7.x — heading-consistency reward (anti-wander).
			# Computes consistency = current velocity direction · EMA-
			# smoothed past velocity direction.  Positive consistency
			# (sustained heading) accumulates events.hit; rapid
			# direction changes drive consistency toward 0 or negative
			# and no hit fires.  The EMA tracks the velocity vector
			# itself (not its normalised direction) so it decays
			# naturally when the body stops — cold start has zero EMA
			# → consistency = 0 → no pulses until sustained motion
			# builds up.
			var v_mag: float = v_xz.length()
			if v_mag > 0.01:
				var v_dir: Vector2 = v_xz / v_mag
				var ema_mag: float = _heading_v_ema.length()
				var consistency: float = 0.0
				if ema_mag > 1e-3:
					consistency = v_dir.dot(_heading_v_ema / ema_mag)
				# Update EMA (vector form — decays toward zero if body stops).
				_heading_v_ema = (1.0 - _HEADING_EMA_ALPHA) * _heading_v_ema \
								  + _HEADING_EMA_ALPHA * v_xz
				# Only positive consistency contributes — opposing-direction
				# motion has consistency<0 but we don't want to PENALISE
				# genuine direction changes, just stop rewarding wandering.
				if consistency > 0.0:
					_heading_hit_accumulator += consistency \
												 * walk_hit_rate \
												 * walk_heading_consistency_gain

			# Phase 6.9 + 6.15 — speed tracking + PR-pulse bonus on EMA.
			# Sustained-speed counter and PR ladder now both gate on
			# _speed_ema → a brief 0.9 m/s lurch can't lock best_speed
			# above what the body can reproduce over ~1 sec.
			if _speed_ema >= SPEED_SUSTAINED_THRESHOLD:
				current_sustained_speed_ticks += 1
				if current_sustained_speed_ticks > best_sustained_speed_ticks:
					best_sustained_speed_ticks = current_sustained_speed_ticks
			else:
				current_sustained_speed_ticks = 0
			if _speed_ema > best_speed and _speed_ema >= SPEED_PR_MIN:
				var pr_delta: float = _speed_ema - best_speed
				best_speed = _speed_ema
				var pr_intensity: float = min(2.0, 1.0 + 5.0 * pr_delta)
				brain.publish_event("hit", pr_intensity)
				pr_event_count += 1
			var walk_hit_fired: bool = false
			while _walk_accumulator >= 1.0:
				brain.publish_event("hit", 1.0)
				_walk_accumulator -= 1.0
				walk_hit_fired = true
			# Phase 7.x — heading-consistency accumulator → events.hit.
			# Same drain pattern as the radial accumulator; fires the
			# moment 1.0 of accumulated consistency is reached.
			while _heading_hit_accumulator >= 1.0:
				brain.publish_event("hit", 1.0)
				_heading_hit_accumulator -= 1.0
				walk_hit_fired = true
			# Phase 6.12 — inward-motion penalty (radial_penalize_inward mode).
			# Fires events.miss at the same rate the hit-side fires for
			# equivalent outward motion → symmetric DA pressure.  Counter
			# is gated on the mode itself so other modes have zero firing
			# regardless of accumulator residuals.
			# 2026-06-11 — to_target added to the drain gate.  It always
			# accumulated misses (receding from target, above) but the drain
			# here was penalize_inward-only, so to_target was silently pure
			# approach-reward with an unbounded dead accumulator — contrary
			# to its design comment ("same penalize-inward shape but anchored
			# to the target").  Residual is zeroed on live mode switch.
			if walk_reward_mode != _walk_miss_mode_prev:
				_walk_miss_accumulator = 0.0
				_walk_miss_mode_prev = walk_reward_mode
			if walk_reward_mode == "radial_penalize_inward" or walk_reward_mode == "to_target":
				while _walk_miss_accumulator >= 1.0:
					brain.publish_event("miss", 1.0)
					_walk_miss_accumulator -= 1.0
					walk_miss_fired_count += 1
			# Phase 6.7 — walking-reward arm of the alive heartbeat.  Same
			# rationale as the standing-reward arm above: alive_pulse
			# oscillates with reward firing, giving HomeokineticExploration's
			# gate a meaningful urgency variability signal.
			if _publish_alive_heartbeat and walk_hit_fired:
				brain.publish_event("alive", 1.0)
	if fell:
		if not _episode_fell:
			brain.publish_event("miss", 1.0)
			_episode_fell = true
			_hit_accumulator = 0.0    # reset accumulator on fall — no carry-over
		# Cumulative "since last fell" timer zeroes EVERY tick the body
		# is fallen (not gated on _episode_fell) so the HUD reflects the
		# actual upright duration, not just the gap between catastrophic
		# fall events.
		cumulative_alive_ticks = 0

	# Stability shaping (v6.0.b.3) — events.miss pulses when the body
	# is elevated AND moving fast.  Brain's only motivation lever is
	# neurochem, so this is the channel for "push toward less movement
	# when up."  Couples to NeurochemState's existing miss handler
	# (-da_miss_drop * intensity, -ht_miss_drop * intensity).  Distinct
	# from the catastrophic fall-miss above (intensity=1.0) by using a
	# smaller per-tick intensity (default 0.05) — together they shape
	# DA as: high+still → +reward; high+moving → reward suppressed;
	# fallen → catastrophic miss.  See docs/v4_brain_derivation §4.2
	# for the saturation invariant; at stability_gain=0.05 with rate
	# ~1/tick the steady-state DA suppression is ~0.10, leaving net
	# +DA when standing still while keeping the brain from pursuing
	# "jump-for-instant-reward" reward-hacking policies.
	if stability_gain > 0.0 and not fell:
		var chassis_y_norm_s: float = clamp(chassis_y / STANDING_CHASSIS_Y, 0.0, 1.0)
		var chassis_speed:    float = _chassis.linear_velocity.length()
		if chassis_y_norm_s > stability_y_norm and chassis_speed > stability_speed:
			brain.publish_event("miss", stability_gain)

	# Anti-rotation shaping — graded events.miss intensity scales with
	# how much the body's angular velocity exceeds threshold.  Unlike
	# the stability shaping above (which is gated on chassis elevation),
	# this fires at any height — including during the initial "get up"
	# phase — to penalise rotational drift throughout the trajectory.
	# The reasoning: tip-backwards is caused by accumulated yaw/pitch,
	# which can build up even before the body is fully upright if the
	# initial policy is asymmetric.
	if antirot_gain > 0.0 and not fell:
		var ang_speed: float = _chassis.angular_velocity.length()
		if ang_speed > antirot_threshold:
			var overage:  float = ang_speed - antirot_threshold
			var intensity: float = clamp(overage / antirot_scale, 0.0, 1.0) * antirot_gain
			brain.publish_event("miss", intensity)

	# Energy-cost shaping — events.miss intensity proportional to total
	# mechanical power consumed by the motors LAST tick.  1-tick latency
	# because the motor block (where torque is computed) runs after
	# this reward block.  See v6.0.b.7 design note above.
	if energy_gain > 0.0 and not fell:
		if _motor_power_last_tick > energy_deadband:
			var overage:  float = _motor_power_last_tick - energy_deadband
			var intensity_e: float = clamp(overage / energy_scale, 0.0, 1.0) * energy_gain
			brain.publish_event("miss", intensity_e)

	# Phase 7.5.R+ — per-source reward attribution update.  Run every
	# tick so EMAs converge even during fall periods (where deltas are
	# 0 → EMA decays toward 0, correctly reflecting "no reward right
	# now").  Cumulative sums track total contribution per source for
	# the end-of-run snapshot.
	_hit_rate_standing_ema = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_standing_ema + HIT_RATE_EMA_ALPHA * _hit_delta_standing
	_hit_rate_walking_ema  = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_walking_ema  + HIT_RATE_EMA_ALPHA * _hit_delta_walking
	_hit_rate_gated_ema    = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_gated_ema    + HIT_RATE_EMA_ALPHA * _hit_delta_gated
	_hit_rate_progress_ema = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_progress_ema + HIT_RATE_EMA_ALPHA * _hit_delta_progress
	_hit_rate_level_chassis_ema = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_level_chassis_ema + HIT_RATE_EMA_ALPHA * _hit_delta_level_chassis
	_hit_rate_gait_cycle_ema = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_gait_cycle_ema + HIT_RATE_EMA_ALPHA * _hit_delta_gait_cycle
	_hit_rate_step_quality_ema = (1.0 - HIT_RATE_EMA_ALPHA) * _hit_rate_step_quality_ema + HIT_RATE_EMA_ALPHA * _hit_delta_step_quality
	_hit_cum_standing += _hit_delta_standing
	_hit_cum_walking  += _hit_delta_walking
	_hit_cum_gated    += _hit_delta_gated
	_hit_cum_step_quality += _hit_delta_step_quality
	_hit_delta_standing = 0.0
	_hit_delta_walking  = 0.0
	_hit_delta_gated    = 0.0
	_hit_delta_progress = 0.0
	_hit_delta_level_chassis = 0.0
	_hit_delta_gait_cycle = 0.0
	_hit_delta_step_quality = 0.0

	# ---- 4. Tick brain (skip immediately after reset) ----
	# 2026-08-03 — PAUSED IN CALIBRATION MODES.  In C (FK geometry) and G (motor test)
	# the operator owns the servo targets, so a ticking brain is doing two harmful
	# things: fighting the sliders for the actuators, and LEARNING FROM A BODY IT IS NOT
	# DRIVING — its forward model gets trained on motion it did not cause, which is
	# exactly the tautological-channel problem in a new place.  Characterising springs
	# and damping requires the brain quiet.  Rings the pause once so it is visible in
	# the log rather than silent.
	if _calibrate_mode or _motor_test_mode:
		if not _brain_paused_notified:
			_brain_paused_notified = true
			print("PicrawlerBody: BRAIN PAUSED (calibration mode) — operator owns the servos")
			_ui_notify("brain PAUSED — calibration mode")
	elif _instant_pause_tick:
		_instant_pause_tick = false
		_brain_paused_notified = false
	else:
		_brain_paused_notified = false
		brain.tick(TAU)

	_tick_gang_drive()      # G-mode ganged pulse / shake writes servo_targets first

	# ---- 5. Apply servo torques (or calibrate FK) ----
	# In C-calibration mode, bodies are frozen kinematic.  We write
	# their transforms directly from the slider values via forward
	# kinematics — the slider IS the joint angle, no motors or constraint
	# solver in the loop.  G-mode keeps the bodies DYNAMIC and falls
	# through to the powered-PD chain below, with slider-sourced targets.
	if _calibrate_mode:
		# Phase 7.x — when _cpg_drive_calibrate is on, drive the
		# suspended joints from the CPG's PURE BIAS (sine + standing
		# DC, no brain command mixed in).  brain.get_action_channel
		# would return brain + CPG blended, which mixes Premotor
		# sampling noise into the visualisation.  get_cpg_pure_bias()
		# returns just the sine-generator output so the gait waveform
		# appears coherently.
		#
		# Slew-limit the target the same way the runtime PD chain does
		# so visualisation rate matches free-running motion.  Bodies
		# stay frozen kinematic; FK direct-write below renders them
		# at the slewed pure-CPG joint angles.
		#
		# Re-freeze any legs that may have been unfrozen by an earlier
		# (now-obsolete) physics-based path so we have a clean state.
		if _legs_unfrozen_for_cpg_drive:
			_legs_unfrozen_for_cpg_drive = false
			for i in range(4):
				_coxas[i].freeze  = true
				_uppers[i].freeze = true
				_lowers[i].freeze = true
				_coxas[i].linear_velocity   = Vector3.ZERO
				_coxas[i].angular_velocity  = Vector3.ZERO
				_uppers[i].linear_velocity  = Vector3.ZERO
				_uppers[i].angular_velocity = Vector3.ZERO
				_lowers[i].linear_velocity  = Vector3.ZERO
				_lowers[i].angular_velocity = Vector3.ZERO
		if _cpg_drive_calibrate:
			var max_step: float = MAX_SERVO_SPEED * TAU
			var bias: Array = brain.get_cpg_pure_bias()
			# Expected length 12 in JOINT_ORDER (fl_hip1, fl_hip2, fl_knee,
			# fr_hip1, ...).  If the CPG isn't in the config, bias is
			# empty and we fall through to slider-driven targets.
			if bias.size() == 12:
				for li in range(4):
					var u1: float = clamp(float(bias[li*3 + 0]), -1.0, 1.0)
					var u2: float = clamp(float(bias[li*3 + 1]), -1.0, 1.0)
					var u3: float = clamp(float(bias[li*3 + 2]), -1.0, 1.0)
					u1 = u1 * HIP1_SPLAY_OUT_SIGN[li]
					var t1: float = u1 * HIP1_TARGET_RANGE + HIP1_REST
					var t2: float = u2 * HIP_TARGET_RANGE  + HIP2_REST
					var t3: float = u3 * HIP_TARGET_RANGE  + KNEE_REST
					_eff_target_hip1[li] = clamp(t1, _eff_target_hip1[li] - max_step, _eff_target_hip1[li] + max_step)
					_eff_target_hip2[li] = clamp(t2, _eff_target_hip2[li] - max_step, _eff_target_hip2[li] + max_step)
					_eff_target_knee[li] = clamp(t3, _eff_target_knee[li] - max_step, _eff_target_knee[li] + max_step)
					servo_targets[servo_idx(li, 0)] = _eff_target_hip1[li]
					servo_targets[servo_idx(li, 1)] = _eff_target_hip2[li]
					servo_targets[servo_idx(li, 2)] = _eff_target_knee[li]
		for i in range(4):
			var s1: float = servo_targets[servo_idx(i, 0)] * servo_signs[servo_idx(i, 0)] \
						  + servo_origins[servo_idx(i, 0)]
			var s2: float = servo_targets[servo_idx(i, 1)] * servo_signs[servo_idx(i, 1)] \
						  + servo_origins[servo_idx(i, 1)]
			var s3: float = servo_targets[servo_idx(i, 2)] * servo_signs[servo_idx(i, 2)] \
						  + servo_origins[servo_idx(i, 2)]
			var fk: Array = _fk_leg(i, s1, s2, s3)
			_coxas[i].global_transform  = fk[0]
			_uppers[i].global_transform = fk[1]
			_lowers[i].global_transform = fk[2]
		# Skip the diag emit / episode boundary tail too — calibration
		# is not an episode, just an interactive visualization.
		if verbose_logging and diag_interval_ticks > 0 and (tick_counter % diag_interval_ticks) == 0:
			_emit_jsonl(hip1_angles, hip2_angles, knee_angles, chassis_y, chassis_tilt)
		return

	# Main powered-PD chain.  C-mode returned above (FK-direct-write);
	# G-mode falls through here with slider-sourced targets.  Motor-test
	# overrides ragdoll so the servos actually fight gravity in G.
	var unpowered: bool = _ragdoll_mode and not _motor_test_mode
	# Stage 3.E — motor_authority_scale models servo-saver compliance.
	# Default 1.0 → bit-identical to pre-3.E.  See @export docstring.
	var max_torque_powered: float = MAX_SERVO_TORQUE * leg_strength * motor_authority_scale
	# Accumulate mechanical power across all 12 joints this tick.  Read
	# by NEXT tick's reward block via _motor_power_last_tick.
	var power_acc: float = 0.0
	for i in range(4):
		# PER-LEG torque ceiling.  Identical to max_torque_powered unless this leg is
		# currently lesioned, so with no lesion armed the whole chain is byte-identical.
		var leg_tq: float = max_torque_powered
		if _lesion_active and i == _lesion_leg and _lesion_mode == "torque":
			leg_tq *= _lesion_scale
		# Relative angular velocity around each hinge axis — used for both
		# powered Kd damping and unpowered viscous/static friction.
		var omega_hip1: float = (_coxas[i].angular_velocity - _chassis.angular_velocity).dot(Vector3.UP)
		var omega_hip2: float = (_uppers[i].angular_velocity - _coxas[i].angular_velocity).dot(_hip2_axes[i])
		var omega_knee: float = (_lowers[i].angular_velocity - _uppers[i].angular_velocity).dot(_knee_axes[i])

		var tq_hip1: float
		var tq_hip2: float
		var tq_knee: float
		if unpowered:
			# Unpowered metal-gear servo: back-EMF + gear-train friction.
			# Resists motion proportional to ω (viscous), plus static
			# stiction that opposes any motion the bodies still have.
			tq_hip1 = _unpowered_torque(omega_hip1)
			tq_hip2 = _unpowered_torque(omega_hip2)
			tq_knee = _unpowered_torque(omega_knee)
		else:
			# Powered PD.  Target source depends on mode:
			#   G (motor-test):     direct slider value with sign + origin
			#   normal operation:   brain action channel mapped through
			#                       HIP*_TARGET_RANGE + per-joint REST
			var t_hip1_cmd: float
			var t_hip2_cmd: float
			var t_knee_cmd: float
			# Source the per-tick u command from one of three sources, then
			# run the SAME backend-aware pipeline below.  This makes G mode
			# (motor_test_mode) a true remote control representative of
			# whichever actuation backend is active — slider position is
			# interpreted as the user's u-command, then routed through bri
			# spike integration or discrete angle mapping just like brain
			# commands would be.  2026-06-01 Stage 3.D + G-mode unification.
			var u_hip1: float
			var u_hip2: float
			var u_knee: float
			if _motor_test_mode:
				# G mode: invert slider's raw target angle back into u-space.
				# u = (slider_target - REST) / TARGET_RANGE.  Slider already
				# bakes in per-servo calibration sign/origin, so the resulting
				# u is in joint-local space; we then apply HIP1_SPLAY_OUT_SIGN
				# below to match brain's convention so the bri pipeline below
				# integrates identically to a brain command at this u value.
				var t_h1_raw: float = servo_targets[servo_idx(i, 0)] * servo_signs[servo_idx(i, 0)] + servo_origins[servo_idx(i, 0)]
				var t_h2_raw: float = servo_targets[servo_idx(i, 1)] * servo_signs[servo_idx(i, 1)] + servo_origins[servo_idx(i, 1)]
				var t_kn_raw: float = servo_targets[servo_idx(i, 2)] * servo_signs[servo_idx(i, 2)] + servo_origins[servo_idx(i, 2)]
				# Note: HIP1_REST may be 0 (the symmetric splay-zero pose); for
				# both hips/knees the inverse-map cleanly returns u in [-1, 1]
				# given slider ranges align with TARGET_RANGE.  Clamp guards
				# against slider mis-calibration / past-end values.
				# 2026-06-03 — G calibration mode does NOT clamp u to [-1, +1].
				# The clamps are correct for brain inputs (whose u is bounded
				# by softmax), but in G mode the operator's slider is in raw
				# joint-angle space and can legitimately command beyond the
				# u=±1 range that the asymmetric mapping represents.  Without
				# the clamp, the round-trip slider→u→t_cmd in the discrete
				# actuation backend is identity, so the slider value becomes
				# the joint target — matching C-mode FK semantics.  Tested:
				# slider +1.6 → knee_defl=+3.2 → u=+2.0 (no clamp) →
				# t_cmd = +2.0 × 1.6 + (-1.6) = +1.6 ✓ (joint LIMIT_HIGH
				# clamps physically at +1.70).
				u_hip1 = (t_h1_raw - HIP1_REST) / HIP1_TARGET_RANGE
				u_hip2 = (t_h2_raw - HIP2_REST) / HIP_TARGET_RANGE
				# Asymmetric knee: invert with whichever range the deflection sign uses
				var knee_defl: float = t_kn_raw - KNEE_REST
				var knee_inv_range: float
				if knee_widening_enabled:
					knee_inv_range = KNEE_RANGE_FOLD if knee_defl >= 0.0 else KNEE_RANGE_HYPEREXT
				else:
					knee_inv_range = KNEE_RANGE_SYMMETRIC
				u_knee = knee_defl / knee_inv_range
			else:
				# Ablation overrides (v6.0.b.8) — replace the brain's
				# action channel with a fixed/random value before
				# mapping to PD targets.  Lets us run brain-off /
				# random-policy controls without reconfiguring the
				# brain or rewiring the action bus.
				if random_policy:
					u_hip1 = _rand_policy_rng.randf_range(-1.0, 1.0)
					u_hip2 = _rand_policy_rng.randf_range(-1.0, 1.0)
					u_knee = _rand_policy_rng.randf_range(-1.0, 1.0)
				elif brain_off:
					u_hip1 = 0.0
					u_hip2 = 0.0
					u_knee = 0.0
				else:
					u_hip1 = clamp(brain.get_action_channel(_idx_hip1[i]), -1.0, 1.0)
					u_hip2 = clamp(brain.get_action_channel(_idx_hip2[i]), -1.0, 1.0)
					u_knee = clamp(brain.get_action_channel(_idx_knee[i]), -1.0, 1.0)
					# LESION, "action" mode (the default).  Attenuates how far this leg
					# actually MOVES rather than how hard it can push.
					#
					# The torque-ceiling form (mode "torque", below) was built first and
					# MEASURED NOT TO PERTURB: at x0.05 the cut leg's oscillation amplitude
					# was 0.720 vs 0.725 untouched, because tq_sat = 0.009 -- the servos are
					# saturated under 1% of the time, so the ceiling is not the binding
					# constraint and scaling it removes headroom the body never used.  Only
					# at exactly x0.0 did anything happen, and then the body COLLAPSED
					# (dz_rate 0.058 -> 0.006, plv support falling) rather than adapting.
					# A (d) test needs a leg that is degraded but still live, and on this
					# body torque authority has no such regime -- it is nearly binary.
					# Corroborates the ledger twice over: "authority was never the binding
					# constraint" and the gravity-scaffold null ("servos have ~4x headroom").
					if _lesion_active and i == _lesion_leg and _lesion_mode == "action":
						u_hip1 *= _lesion_scale
						u_hip2 *= _lesion_scale
						u_knee *= _lesion_scale
					# WEAK ablation, per joint — a browning-out servo. DEAD and SEIZED are
					# not handled here: they act on the MOTOR (below), because attenuating
					# a command is not the same failure as releasing or jamming a gearbox.
					if _abl_any:
						if _abl_kind[abl_idx(i, 0)] == AblKind.WEAK: u_hip1 *= _abl_weak_scale
						if _abl_kind[abl_idx(i, 1)] == AblKind.WEAK: u_hip2 *= _abl_weak_scale
						if _abl_kind[abl_idx(i, 2)] == AblKind.WEAK: u_knee *= _abl_weak_scale
			# E0/E1 — additive scripted-gait oscillation ON TOP of the base
			# controller (brain holds posture; oscillation injects the gait).
			# Canonical within-leg joint offsets hip1=π, hip2=π/2, knee=0 → foot
			# traces a propulsive ellipse.  u stays in brain splay-out convention;
			# the flip + backend pipeline below handle per-servo geometry.  Off = no-op.
			if _scripted_gait:
				var sg_phi: float = TAU * (float(tick_counter) / _sg_period) + float(_sg_offsets[i])
				u_hip1 += _sg_amp_hip1 * sin(sg_phi + _sg_hip1_phase)
				u_hip2 += _sg_amp_hip2 * sin(sg_phi + PI / 2.0)
				u_knee += _sg_amp_knee * sin(sg_phi)
			# UNIFIED PATH from here.  G-mode produces u in joint-local space
			# already (servo_signs/origins handle per-servo calibration); the
			# brain's u uses "splay-out" convention (u_hip1=+1 means splay
			# outward across all legs).  Apply HIP1_SPLAY_OUT_SIGN ONLY to
			# brain-sourced u to bring it into joint-local space before the
			# unified backend pipeline.  Skipping the flip in G mode preserves
			# the original (pre-unification) slider-to-physics semantic.
			if not _motor_test_mode:
				u_hip1 = u_hip1 * HIP1_SPLAY_OUT_SIGN[i]
			# 2026-06-06 — hip1 symmetric yaw probe (joint frame, post sign flip).
			# Cruse-asymmetric (hip2+knee) was falsified on a standing-only body —
			# it needs a gait to bias.  Pure hip1 differential left-vs-right
			# produces yaw torque on a static stance through ground friction.
			# Pattern: LEFT  (FL, RL) joint hip1 += +delta * gain
			#          RIGHT (FR, RR) joint hip1 += -delta * gain
			if _yaw_probe_delta != 0.0:
				var side_sign: float = +1.0 if (i == 0 or i == 2) else -1.0
				u_hip1 = clamp(u_hip1 + side_sign * _yaw_probe_delta * YAW_PROBE_HIP1_GAIN, -1.0, 1.0)
			if actuation_backend == "bernoulli_impulse":
				# Stage 3.D — Bernoulli-impulse: convert u in [-1,1] to
				# spike rate, sample fire/no-fire, integrate offset.
				# 3 independent RNG streams per leg (one per joint), the
				# 12 streams are deterministic-seeded in _ready().  Each
				# fire moves the joint's accumulated offset by
				# bri_impulse_per_spike (in units of TARGET_RANGE);
				# bri_friction_per_tick decays the offset toward rest.
				# Final t_*_cmd = REST + offset × TARGET_RANGE — same
				# mapping shape as the discrete backend, so the motor
				# block downstream is unchanged.
				var rate_h1: float = clamp(bri_base_rate + bri_command_bias * abs(u_hip1), 0.0, 1.0)
				var rate_h2: float = clamp(bri_base_rate + bri_command_bias * abs(u_hip2), 0.0, 1.0)
				var rate_kn: float = clamp(bri_base_rate + bri_command_bias * abs(u_knee), 0.0, 1.0)
				var rng_h1: RandomNumberGenerator = _bri_rng[i * 3 + 0]
				var rng_h2: RandomNumberGenerator = _bri_rng[i * 3 + 1]
				var rng_kn: RandomNumberGenerator = _bri_rng[i * 3 + 2]
				if rng_h1.randf() < rate_h1:
					_bri_offset_hip1[i] += bri_impulse_per_spike * signf(u_hip1)
					_bri_spike_count_total += 1
					_bri_spike_count_this_diag += 1
				if rng_h2.randf() < rate_h2:
					_bri_offset_hip2[i] += bri_impulse_per_spike * signf(u_hip2)
					_bri_spike_count_total += 1
					_bri_spike_count_this_diag += 1
				if rng_kn.randf() < rate_kn:
					_bri_offset_knee[i] += bri_impulse_per_spike * signf(u_knee)
					_bri_spike_count_total += 1
					_bri_spike_count_this_diag += 1
				# Friction decay toward zero offset (relax to rest).
				_bri_offset_hip1[i] *= (1.0 - bri_friction_per_tick)
				_bri_offset_hip2[i] *= (1.0 - bri_friction_per_tick)
				_bri_offset_knee[i] *= (1.0 - bri_friction_per_tick)
				# Clamp offsets to [-1, 1] — same range as raw u.
				_bri_offset_hip1[i] = clamp(_bri_offset_hip1[i], -1.0, 1.0)
				_bri_offset_hip2[i] = clamp(_bri_offset_hip2[i], -1.0, 1.0)
				_bri_offset_knee[i] = clamp(_bri_offset_knee[i], -1.0, 1.0)
				t_hip1_cmd = _bri_offset_hip1[i] * HIP1_TARGET_RANGE + HIP1_REST
				t_hip2_cmd = _bri_offset_hip2[i] * HIP_TARGET_RANGE  + HIP2_REST
				# 2026-06-03 — asymmetric knee mapping (see KNEE_RANGE_FOLD/HYPEREXT).
				# knee_widening_enabled=false collapses to symmetric KNEE_RANGE_SYMMETRIC.
				var bri_knee_range: float
				if knee_widening_enabled:
					bri_knee_range = KNEE_RANGE_FOLD if _bri_offset_knee[i] >= 0.0 else KNEE_RANGE_HYPEREXT
				else:
					bri_knee_range = KNEE_RANGE_SYMMETRIC
				t_knee_cmd = _bri_offset_knee[i] * bri_knee_range + KNEE_REST
			else:
				# Default "discrete" backend — u maps directly to target angle.
				# Bit-identical to pre-3.D pipeline.  In G mode, this round-
				# trips back to the slider's commanded angle (modulo the
				# SPLAY_OUT_SIGN cancellation), so discrete G mode behaves
				# identically to its pre-unification version.
				t_hip1_cmd = u_hip1 * HIP1_TARGET_RANGE + HIP1_REST
				t_hip2_cmd = u_hip2 * HIP_TARGET_RANGE  + HIP2_REST
				# 2026-06-03 — asymmetric knee mapping (see KNEE_RANGE_FOLD/HYPEREXT).
				# u=+1 → max fold (~170° tuck, spider stance reachable).
				# u=0  → REST = straight leg (KNEE_REST=-1.6 rad).
				# u=-1 → max hyperextension past straight (-2.45 rad).
				# knee_widening_enabled=false collapses to symmetric KNEE_RANGE_SYMMETRIC.
				var discrete_knee_range: float
				if knee_widening_enabled:
					discrete_knee_range = KNEE_RANGE_FOLD if u_knee >= 0.0 else KNEE_RANGE_HYPEREXT
				else:
					discrete_knee_range = KNEE_RANGE_SYMMETRIC
				t_knee_cmd = u_knee * discrete_knee_range + KNEE_REST
			# Rate-limit the EFFECTIVE target (the value the PD chases) to
			# MAX_SERVO_SPEED per brain tick.  This bounds how fast the
			# joint can move regardless of how strong the PD is — exactly
			# the same role a real servo's PWM cycle + gear inertia plays.
			var max_step: float = MAX_SERVO_SPEED * TAU
			_eff_target_hip1[i] = clamp(t_hip1_cmd, _eff_target_hip1[i] - max_step, _eff_target_hip1[i] + max_step)
			_eff_target_hip2[i] = clamp(t_hip2_cmd, _eff_target_hip2[i] - max_step, _eff_target_hip2[i] + max_step)
			_eff_target_knee[i] = clamp(t_knee_cmd, _eff_target_knee[i] - max_step, _eff_target_knee[i] + max_step)
			tq_hip1 = _powered_torque(_eff_target_hip1[i], hip1_angles[i], omega_hip1, leg_tq)
			tq_hip2 = _powered_torque(_eff_target_hip2[i], hip2_angles[i], omega_hip2, leg_tq)
			tq_knee = _powered_torque(_eff_target_knee[i], knee_angles[i], omega_knee, leg_tq)
		# First-order lag — emulates real servo's gear+motor rise time.
		# Without this, stiff Kp produces step-impulses that go unstable.
		var alpha: float = 1.0 - exp(-TAU / SERVO_TORQUE_RISE_TAU)
		tq_hip1 = _prev_torque_hip1[i] + (tq_hip1 - _prev_torque_hip1[i]) * alpha
		tq_hip2 = _prev_torque_hip2[i] + (tq_hip2 - _prev_torque_hip2[i]) * alpha
		tq_knee = _prev_torque_knee[i] + (tq_knee - _prev_torque_knee[i]) * alpha
		_prev_torque_hip1[i] = tq_hip1
		_prev_torque_hip2[i] = tq_hip2
		_prev_torque_knee[i] = tq_knee

		# Mechanical power per joint (Watts) = |τ| × |ω|.  Accumulated
		# across all 3 joints of all 4 legs.  Read by next tick's
		# reward block for energy-cost shaping.
		power_acc += abs(tq_hip1) * abs(omega_hip1) \
				   + abs(tq_hip2) * abs(omega_hip2) \
				   + abs(tq_knee) * abs(omega_knee)

		# Apply via JOINT MOTOR target-velocity, not apply_torque_impulse.
		# apply_torque_impulse delivers an unconstrained ω-impulse that
		# exceeds MAX_SERVO_SPEED in one tick on our small leg masses.
		# HingeJoint3D's motor enforces both:
		#   - target_velocity = clamp(Kp * error, ±MAX_SERVO_SPEED)
		#   - per-physics-step torque bounded by PARAM_MOTOR_MAX_IMPULSE
		# Bullet uses MAX_IMPULSE as an angular-impulse cap per step but
		# the motor only delivers as much as needed to track the velocity
		# target — vel saturation at MAX_SERVO_SPEED is what keeps the
		# motor from blowing through limits during normal operation.  The
		# propeller observed earlier was a limit-WRAP issue on the knee
		# (KNEE_LIMIT_LOW was at -π, the angle-measurement wrap point),
		# not a torque-magnitude issue — fixed by tightening the limit.
		# PARAM_MOTOR_MAX_IMPULSE is angular impulse per physics step
		# (Nm·s), so divide torque by physics_hz to get correct units.
		# Without this, the effective torque is physics_hz× too high
		# (e.g. 0.15 Nm × 240 = 36 Nm), which overwhelms the constraint
		# solver on 25g leg segments.
		# Motor-test boosts the impulse cap so the four hip1 motors —
		# which all share the chassis as body A around the same UP axis
		# — have enough headroom to overcome each other's reactions
		# without stalling at intermediate angles.  Brain operation
		# uses the spec'd 0.15 Nm / physics_hz cap.
		# G6DOF migration units — see motor_force_scale @export comment.
		# If FORCE_LIMIT turns out to be per-substep impulse (matching the
		# old HingeJoint3D semantic), motor_force_scale ≈ 1/physics_hz
		# recovers correct behaviour.  If FORCE_LIMIT is true Nm (Bullet
		# multiplies by dt internally), motor_force_scale = 1.0 is correct.
		# Diagnostic — drag in Inspector to find which.
		var motor_max_impulse: float = ((leg_tq * 10.0) if _motor_test_mode else leg_tq) * motor_force_scale
		# Use the PD torque magnitude to set motor max impulse (so the
		# motor effectively saturates at the PD-computed torque), and
		# convert torque sign back to velocity sign for the target.
		# All three motors negate the velocity command — Godot's
		# HingeJoint3D motor velocity sign is inverted relative to the
		# quaternion projection used by _relative_angle_world_axis.
		# (Removing the knee's negation causes propeller — joint blows
		# through the soft limit, the quaternion measurement wraps near
		# ±π, and the PD reverses direction.)
		# 2026-06-01 — Stage 3.E++ free-play AT THE MOTOR COMMAND LEVEL.
		# Previous bug: motor_freeplay_rad only affected _powered_torque
		# (telemetry path), not the actual motor velocity command, so the
		# motor still tracked aggressively inside the supposed free zone.
		# Operator's G-mode test correctly diagnosed: no posture sag when
		# freeplay raised, no chassis lift when spring stiffness raised.
		#
		# Fix: per-joint error check.  If |target − angle| is inside the
		# effective deadband (max of SERVO_DEADBAND and motor_freeplay_rad),
		# the motor is DISENGAGED: target_velocity=0 AND max_impulse=0.
		# With both zero the joint motor applies no torque and the joint
		# is free to drift under gravity.  (A passive centering spring was
		# attempted via apply_torque but is unstable at picrawler scale;
		# the native fix is Generic6DOFJoint3D, deferred to a separate
		# migration PR.)  Outside the deadband, the normal velocity-
		# tracking command applies with the configured impulse cap.
		var err_h1: float = _eff_target_hip1[i] - hip1_angles[i]
		var err_h2: float = _eff_target_hip2[i] - hip2_angles[i]
		var err_kn: float = _eff_target_knee[i] - knee_angles[i]
		# motor disengage ONLY when freeplay is explicitly enabled (>0).
		# Default 0 ⇒ motor always engages (pre-3.E++ behavior, bit-identical).
		var use_freeplay: bool = motor_freeplay_rad > 0.0
		# 2026-08-03 — BACKLASH, not an on/off deadband.  The old code computed the motor
		# velocity from the FULL error and merely zeroed it inside the band.  So a joint
		# drifting under gravity would leave the band, the motor would engage and drive it
		# all the way back to TARGET, overshooting into the band again, motor off, drift...
		# a limit cycle.  The operator saw it exactly: "legs slowly drift down then twitch
		# back up" with all stiffness and damping at zero, where the expectation is a loose
		# floppy joint.
		#
		# Real mechanical slop rests AT THE EDGE of the slop zone: within the band the gear
		# train is disengaged, and once the slop is taken up the train simply holds.  So the
		# motor must respond only to the error BEYOND the deadband.  err_eff then decays to
		# 0 as the joint reaches the boundary and it settles there instead of snapping back.
		var eh1: float = _freeplay_err(err_h1)
		var eh2: float = _freeplay_err(err_h2)
		var ekn: float = _freeplay_err(err_kn)
		var vel_hip1: float = -clamp(SERVO_KP * eh1, -MAX_SERVO_SPEED, MAX_SERVO_SPEED)
		var vel_hip2: float = -clamp(SERVO_KP * eh2, -MAX_SERVO_SPEED, MAX_SERVO_SPEED)
		var vel_knee: float = -clamp(SERVO_KP * ekn, -MAX_SERVO_SPEED, MAX_SERVO_SPEED)
		# Load = commanded velocity - achieved velocity, in the ANGLE frame.
		# vel_* above is negated because Godot's motor velocity sign is inverted
		# relative to _relative_angle_world_axis; undo that here so the deficit is
		# expressed in the same frame as omega_* and the joint angles.
		_prev_load_hip1[i] = clamp(((-vel_hip1) - omega_hip1) / MAX_SERVO_SPEED, -1.0, 1.0)
		_prev_load_hip2[i] = clamp(((-vel_hip2) - omega_hip2) / MAX_SERVO_SPEED, -1.0, 1.0)
		_prev_load_knee[i] = clamp(((-vel_knee) - omega_knee) / MAX_SERVO_SPEED, -1.0, 1.0)
		var imp_h1: float = motor_max_impulse
		var imp_h2: float = motor_max_impulse
		var imp_kn: float = motor_max_impulse
		# P7 SERVO_KI — the force-boost integral, on the REAL force path (imp_* is what
		# _set_motor_vf hands the joint; _powered_torque is telemetry).  eh* is already
		# the beyond-deadband error (0 inside the band → integral frozen there by
		# construction), the leak always applies, and the boost is a bounded FRACTION of
		# motor_max_impulse.  boost_frac ≈ servo_ki × |err| at integral saturation
		# (τ = 0.4 s), so servo_ki {1, 3, 6} ≈ {10, 30, 60}% boost at a 0.1 rad
		# sustained load error.  0 = off, byte-identical.
		if servo_ki > 0.0:
			if _ki_int.size() != 12:
				_ki_int.resize(12)
				for _k in range(12): _ki_int[_k] = 0.0
			var _ki_leak: float = 1.0 - 1.0 / (SERVO_KI_TAU * float(physics_hz))
			var _ehs: Array = [eh1, eh2, ekn]
			var _imps: Array = [imp_h1, imp_h2, imp_kn]
			for _jn in range(3):
				var _kx: int = _jn * 4 + i
				_ki_int[_kx] = _ki_int[_kx] * _ki_leak + _ehs[_jn] / float(physics_hz)
				var _bf: float = clamp(servo_ki * abs(_ki_int[_kx]) / SERVO_KI_TAU,
									   0.0, SERVO_KI_AUTH_FRAC)
				_imps[_jn] = _imps[_jn] * (1.0 + _bf)
				_ki_mag_acc += abs(_ki_int[_kx]); _ki_mag_n += 1
				_ki_total_ticks += 1
				if _bf > 0.01: _ki_boost_ticks += 1
			imp_h1 = _imps[0]; imp_h2 = _imps[1]; imp_kn = _imps[2]
		if use_freeplay:
			# Inside the band the hard motor is released.  What acts there instead is the
			# SOFT SPRING below — see _soft_spring_cmd().
			if is_zero_approx(eh1): vel_hip1 = 0.0; imp_h1 = 0.0
			if is_zero_approx(eh2): vel_hip2 = 0.0; imp_h2 = 0.0
			if is_zero_approx(ekn): vel_knee = 0.0; imp_kn = 0.0
			# 2026-08-03 — SPRINGS IMPLEMENTED IN THE MOTOR, because the constraint spring
			# does not exist.  MEASURED: knee_spring_stiffness 0 vs 20 gave byte-identical
			# trajectories, so Generic6DOFJoint3D's angular spring is a NO-OP here.  The
			# June note assumed Bullet's constraint-level spring; Godot 4 replaced Bullet
			# with Godot Physics 3D, which does not implement it, and setting the params
			# raises no error.  The springs have therefore never done anything.
			#
			# apply_torque was already tried and reverted (explicit Euler, low-inertia
			# segments).  This instead scales the MOTOR's velocity command inside the
			# deadband: a velocity-target motor is solved at constraint level, so it is
			# stable where an applied torque is not.  Soft near the target, firm past the
			# slop — which IS series-elastic behaviour.  stiffness 0 ⇒ free (unchanged).
			if is_zero_approx(eh1):
				var c1: Array = _soft_spring_cmd(err_h1, hip1_spring_stiffness,
												 hip1_spring_damping, motor_max_impulse)
				vel_hip1 = c1[0]; imp_h1 = c1[1]
			if is_zero_approx(eh2):
				var c2: Array = _soft_spring_cmd(err_h2, hip2_spring_stiffness,
												 hip2_spring_damping, motor_max_impulse)
				vel_hip2 = c2[0]; imp_h2 = c2[1]
			if is_zero_approx(ekn):
				var c3: Array = _soft_spring_cmd(err_kn, knee_spring_stiffness,
												 knee_spring_damping, motor_max_impulse)
				vel_knee = c3[0]; imp_kn = c3[1]
		if _abl_any:
			# DEAD   -> release the motor entirely (vel 0, impulse 0): the joint free-swings
			#           and is back-driveable, so gravity moves the limb. Same mechanism the
			#           freeplay deadband uses to disengage.
			# SEIZED -> velocity target 0 at FULL impulse: the joint fights any motion and
			#           holds wherever it was. A rigid strut, not a limp one.
			# DETACHED-> nothing left to drive; release so no torque is wasted on a stub.
			var vv: Array = [vel_hip1, vel_hip2, vel_knee]
			var ii: Array = [imp_h1, imp_h2, imp_kn]
			for jn in range(3):
				var kk: int = abl_idx(i, jn)
				if _abl_detached[kk] != 0 or _abl_kind[kk] == AblKind.DEAD:
					vv[jn] = 0.0; ii[jn] = 0.0
				elif _abl_kind[kk] == AblKind.SEIZED:
					vv[jn] = 0.0; ii[jn] = motor_max_impulse
			vel_hip1 = vv[0]; vel_hip2 = vv[1]; vel_knee = vv[2]
			imp_h1 = ii[0];   imp_h2 = ii[1];   imp_kn = ii[2]
		_set_motor_vf(_hip1_joints[i], vel_hip1, imp_h1)
		_set_motor_vf(_hip2_joints[i], vel_hip2, imp_h2)
		_set_motor_vf(_knee_joints[i], vel_knee, imp_kn)
		# Series-elastic behaviour: park the spring's equilibrium ON the commanded
		# angle, so within the freeplay deadband the joint is centred where it was
		# told to be rather than dragged to mechanical zero.
		if spring_follows_target and joint_backend != "hinge":
			_set_spring_equilibrium(_hip1_joints[i], _eff_target_hip1[i])
			_set_spring_equilibrium(_hip2_joints[i], _eff_target_hip2[i])
			_set_spring_equilibrium(_knee_joints[i], _eff_target_knee[i])
		# Motor-test diagnostic — one line per leg, twice a second, so the
		# user can see whether sliders are reaching the motor chain.
		# Surfaces: slider value, eff_target, measured joint angle,
		# commanded motor velocity for every hinge.
		if _motor_test_mode and not unpowered and (tick_counter % 25) == 0:
			print("MotorTest leg %d  hip1: slider=%+.2f eff=%+.2f joint=%+.2f vel=%+.2f  hip2: %+.2f %+.2f %+.2f %+.2f  knee: %+.2f %+.2f %+.2f %+.2f" % [
				i,
				servo_targets[servo_idx(i, 0)], _eff_target_hip1[i], hip1_angles[i], vel_hip1,
				servo_targets[servo_idx(i, 1)], _eff_target_hip2[i], hip2_angles[i], vel_hip2,
				servo_targets[servo_idx(i, 2)], _eff_target_knee[i], knee_angles[i], vel_knee,
			])
		# Unpowered (ragdoll) override only — calibration leaves ALL
		# servos powered so the inactive ones hold the X-stance as a
		# visual reference while the active servo moves through its range.
		if unpowered:
			# Static friction caps backdrive torque; motor's vel target = 0
			# brakes any motion within this torque budget.  G6DOF takes
			# torque (Nm) directly so the 1/physics_hz scaling is dropped.
			_set_motor_vf(_hip1_joints[i], 0.0, UNPOWERED_STATIC_FRICTION)
			_set_motor_vf(_hip2_joints[i], 0.0, UNPOWERED_STATIC_FRICTION)
			_set_motor_vf(_knee_joints[i], 0.0, UNPOWERED_STATIC_FRICTION)

	# Stash this tick's total mechanical power for next tick's reward
	# block to read.  See v6.0.b.7 energy-cost mechanism.
	_motor_power_last_tick = power_acc

	# ---- 5.9 Clip ring (per tick, for the [F1]/[F2] GOOD/BAD marker) ----
	_clip_record(hip1_angles, hip2_angles, knee_angles, chassis_y)

	# ---- 6. Diag emit ----
	if verbose_logging and diag_interval_ticks > 0 and (tick_counter % diag_interval_ticks) == 0:
		_emit_jsonl(hip1_angles, hip2_angles, knee_angles, chassis_y, chassis_tilt)

	# ---- 7. Episode boundary ----
	if reset_mode == "continuous":
		if mc_episode_period > 0 and step_in_episode > 0 \
				and (step_in_episode % mc_episode_period) == 0:
			brain.publish_event("episode_end", float(episode_alive_ticks))
			print(JSON.stringify({"event": "EPISODE_END",
								  "episode": episode_index,
								  "alive_ticks": episode_alive_ticks,
								  "steps": step_in_episode,
								  "reason": "mc_period"}))
			episode_index += 1
			episode_alive_ticks = 0
			# B3 leg symmetry: average Premotor weight pairs immediately
			# AFTER episode_end (i.e., after the MC REINFORCE finalise has
			# applied its update for this episode).  This way each pair
			# learns independently within an episode but starts the next
			# episode from a shared point.
			if leg_symmetry_mode != "off":
				_sync_leg_symmetry()
		# 2026-07-27 — CONTINUOUS MODE TERMINATES ON tick_counter, NOT step_in_episode.
		#
		# The auto-reset path (inversion / belly-up) zeroes `step_in_episode` at :4380,
		# so an arm that keeps flipping keeps RESTARTING its own countdown and never
		# reaches max_steps: the run then ends only at `--quit-after`, tens of thousands
		# of ticks later.  Measured on the step-lock p0 arm: seeds ended at tick 13 407
		# and 72 043 against a 6 000-tick protocol, i.e. a 5x spread WITHIN one arm, while
		# the healthy baseline ended at exactly 6 000.
		#
		# Two ways that corrupts a result, and the second is the dangerous one:
		#   * `falls`, `steps` and every other COUNT becomes counts-per-unequal-duration,
		#     so arms are not comparable to the baseline OR to each other;
		#   * it costs wall-clock in proportion to how badly an arm fails, so the worse a
		#     lever is the longer it takes to find out -- which quietly discourages
		#     running the very sweeps that would refute it.
		#
		# `tick_counter` is monotonic (never zeroed anywhere in this file), so it is the
		# honest clock for a mode whose whole premise is ONE continuous run with no
		# episode boundaries.  For any arm that never auto-resets the two counters are
		# equal, so this is byte-identical for the baseline and every promoted lever --
		# verified by measurement, not argument (the baseline still ends at exactly 6 000).
		if max_steps > 0 and tick_counter >= max_steps:
			_done = true
			print(JSON.stringify({"event": "RUN_END",
								  "episodes": episode_index + 1,
								  "tick": tick_counter,
								  "step_in_episode": step_in_episode}))
	else:
		if fell or (max_steps > 0 and step_in_episode >= max_steps):
			_finish_episode(fell)

func _finish_episode(fell: bool) -> void:
	var reason: String = "fell" if fell else "max_steps"
	brain.publish_event("episode_end", float(episode_alive_ticks))
	print(JSON.stringify({"event": "EPISODE_END",
						  "episode": episode_index,
						  "alive_ticks": episode_alive_ticks,
						  "steps": step_in_episode,
						  "reason": reason,
						  "reset_mode": reset_mode}))
	episode_index += 1
	# B3: same as the continuous-mode boundary, sync after the
	# REINFORCE finalise so the symmetric average reflects this episode's
	# learning before the next one starts.
	if leg_symmetry_mode != "off":
		_sync_leg_symmetry()
	if max_episodes > 0 and episode_index >= max_episodes:
		_done = true
		print(JSON.stringify({"event": "RUN_END",
							  "episodes": episode_index,
							  "tick": tick_counter}))
		return
	_do_hard_reset()
	step_in_episode = 0
	episode_alive_ticks = 0
	_episode_fell = false
	_instant_pause_tick = true

# B3 leg-symmetric weight sharing.  Reads each Premotor's snapshot,
# parses W/b/E, averages them across mirrored leg pairs (and optionally
# across front/rear), writes the averaged snapshot back.  Skipped when
# leg_symmetry_mode == "off".
#
# Pairs (always FL↔FR and RL↔RR for lr_pairs):
#   hip1:  {premotor_fl_hip1, premotor_fr_hip1}, {premotor_rl_hip1, premotor_rr_hip1}
#   pitch: {premotor_fl_pitch, premotor_fr_pitch}, {premotor_rl_pitch, premotor_rr_pitch}
#
# lr_and_fr_pairs additionally averages the two pair-results within each
# joint group, yielding one shared policy across all four legs per joint
# group (hip1 group has one mean, pitch group has one mean).
const _LEG_SYM_GROUPS: Dictionary = {
	"hip1":  [
		["premotor_fl_hip1",  "premotor_fr_hip1"],
		["premotor_rl_hip1",  "premotor_rr_hip1"],
	],
	"pitch": [
		["premotor_fl_pitch", "premotor_fr_pitch"],
		["premotor_rl_pitch", "premotor_rr_pitch"],
	],
	# Phase 6.17 — per-servo Premotor variant.  These groups exist in
	# the 12-Premotor topology and don't in the 8-Premotor topologies;
	# missing-module pairs return {} from _average_premotor_pair and
	# the sync silently skips them (warning suppressed below).
	"hip2": [
		["premotor_fl_hip2",  "premotor_fr_hip2"],
		["premotor_rl_hip2",  "premotor_rr_hip2"],
	],
	"knee": [
		["premotor_fl_knee",  "premotor_fr_knee"],
		["premotor_rl_knee",  "premotor_rr_knee"],
	],
	# Phase 6.18 — per-leg N-channel Premotors.  Each Premotor controls
	# one whole leg via 3-channel output.  LR-sym pairs are FL↔FR and
	# RL↔RR at the leg granularity.  Silently skipped in 8/12-Premotor
	# configs where these IDs don't exist.
	"leg": [
		["premotor_fl",       "premotor_fr"],
		["premotor_rl",       "premotor_rr"],
	],
}
# Counters for the JSONL audit trail.  Public so analysis scripts can
# read them via brain state without poking internals; bumped each
# successful sync.
var leg_symmetry_sync_count: int = 0

func _sync_leg_symmetry() -> void:
	# Defensive: skip if the brain build doesn't expose the per-module
	# API (older binary still in the addon).  Without this, the call
	# would error at runtime and abort the episode.
	if not (brain.has_method("get_module_snapshot") \
			and brain.has_method("set_module_snapshot")):
		push_warning("PicrawlerBody: leg_symmetry_mode set but brain lacks get/set_module_snapshot; skipping")
		return
	var fr_pair: bool = (leg_symmetry_mode == "lr_and_fr_pairs")
	for group in _LEG_SYM_GROUPS:
		var pair_means: Array = []
		for pair in _LEG_SYM_GROUPS[group]:
			var avg: Dictionary = _average_premotor_pair(pair[0], pair[1])
			if avg.is_empty():
				continue
			pair_means.append(avg)
			_write_premotor_arrays(pair[0], avg)
			_write_premotor_arrays(pair[1], avg)
		if fr_pair and pair_means.size() == 2:
			# Average the two pair-averages and broadcast to all four legs
			# in this joint group.  Aggressive but the cleanest test of
			# the strongest symmetry hypothesis.
			var grand: Dictionary = _avg_array_dicts(pair_means[0], pair_means[1])
			for pair in _LEG_SYM_GROUPS[group]:
				_write_premotor_arrays(pair[0], grand)
				_write_premotor_arrays(pair[1], grand)
		elif leg_symmetry_fr_blend > 0.0 and pair_means.size() == 2:
			# E1b — LIGHT front-rear coupling: pull each pair's weights a
			# fraction α toward the grand mean, keeping L/R hard within the
			# pair but gently symmetrising front vs rear posture.
			var grand2: Dictionary = _avg_array_dicts(pair_means[0], pair_means[1])
			var front_b: Dictionary = _blend_array_dicts(pair_means[0], grand2, leg_symmetry_fr_blend)
			var rear_b:  Dictionary = _blend_array_dicts(pair_means[1], grand2, leg_symmetry_fr_blend)
			var grp: Array = _LEG_SYM_GROUPS[group]
			_write_premotor_arrays(grp[0][0], front_b)
			_write_premotor_arrays(grp[0][1], front_b)
			_write_premotor_arrays(grp[1][0], rear_b)
			_write_premotor_arrays(grp[1][1], rear_b)
	leg_symmetry_sync_count += 1

func _average_premotor_pair(name_a: String, name_b: String) -> Dictionary:
	var snap_a_s: String = brain.get_module_snapshot(name_a)
	var snap_b_s: String = brain.get_module_snapshot(name_b)
	if snap_a_s == "" or snap_b_s == "":
		# Both missing → modules not in this topology; silently skip.
		# Only-one missing is a real bug (topology mismatch) — warn.
		if snap_a_s == "" and snap_b_s == "":
			return {}
		push_warning("PicrawlerBody: half-missing snapshot for %s or %s (topology mismatch?)" % [name_a, name_b])
		return {}
	var snap_a: Dictionary = JSON.parse_string(snap_a_s)
	var snap_b: Dictionary = JSON.parse_string(snap_b_s)
	if not (snap_a is Dictionary) or not (snap_b is Dictionary):
		return {}
	# Average the three weight arrays only; other state (rng, urgency,
	# counters) stays whatever each Premotor had — that's deliberate, the
	# rng/EMA differences keep the per-leg exploration jitter alive across
	# ticks while the long-horizon learning signal is shared.
	var out: Dictionary = {}
	for k in ["W", "b", "E"]:
		if snap_a.has(k) and snap_b.has(k):
			out[k] = _avg_nested_arrays(snap_a[k], snap_b[k])
	return out

func _write_premotor_arrays(name_: String, fields: Dictionary) -> void:
	var snap_s: String = brain.get_module_snapshot(name_)
	if snap_s == "":
		return
	var snap: Variant = JSON.parse_string(snap_s)
	if not (snap is Dictionary):
		return
	for k in fields:
		snap[k] = fields[k]
	brain.set_module_snapshot(name_, JSON.stringify(snap))

# Phase 6.7 — body-side entropy-collapse escape detector.  Called from
# _emit_jsonl at the diag cadence (default every 60 ticks ≈ 1s).
# Per-Premotor logic:
#   1. Track pre_h_ema (smoothed H) and pre_h_peak (max H ever seen).
#   2. Once a Premotor has reached a meaningful peak (≥ _ESC_MIN_PEAK_FOR_TRIGGER):
#      → trigger when pre_h_ema drops below _ESC_COLLAPSE_FRACTION × peak
#      AND chassis_xz has been within _ESC_MOTION_EPS for ≥ _ESC_STUCK_DIAG_SAMPLES diag-samples
#      AND brain DA is below _ESC_DA_FLOOR (no reward firing)
#      AND tick is past this Premotor's cooldown.
#   3. On trigger: read snapshot, add Gauss(0, _ESC_KICK_SIGMA) to every
#      W element, write back.  Update cooldown to tick + _ESC_COOLDOWN_TICKS.
# Returns nothing.  Updates _escape_fired_total / _escape_fired_per_pm /
# _escape_active_now_count for telemetry.  No-op if not _escape_detector_enabled.
# Phase 7.13 — env-var overrides for CruseCoordinator HotMutable params.
# Lets the launcher tune Cruse rules without editing JSON config.
# Reads OGMA_PICRAWLER_CRUSE_* env vars and applies them via brain.apply_patch.
# Silent no-op when env-vars unset or brain has no cruse_coordinator module.
func _apply_cruse_env_overrides() -> void:
	if brain == null or not brain.has_method("apply_patch"):
		return
	var module_id := "cruse_coordinator"
	var overrides: Array = [
		["OGMA_PICRAWLER_CRUSE_BIAS_GAIN",        "cruse_bias_gain",       "float"],
		["OGMA_PICRAWLER_CRUSE_RULE3_WEIGHT",     "rule3_weight",          "float"],
		["OGMA_PICRAWLER_CRUSE_RHYTHM_INJECT_GAIN", "rhythm_inject_gain",  "float"],
		["OGMA_PICRAWLER_CRUSE_ENABLE_RULE1",     "enable_rule_1",         "bool"],
		["OGMA_PICRAWLER_CRUSE_ENABLE_RULE2",     "enable_rule_2",         "bool"],
		["OGMA_PICRAWLER_CRUSE_ENABLE_RULE3",     "enable_rule_3",         "bool"],
		["OGMA_PICRAWLER_CRUSE_ADAPTIVE",         "adaptive_magnitude",    "bool"],
		["OGMA_PICRAWLER_CRUSE_RULE1_BOOST",      "rule1_violation_boost", "float"],
		["OGMA_PICRAWLER_CRUSE_RULE2_WINDOW",     "rule2_window_ticks",    "int"],
		["OGMA_PICRAWLER_CRUSE_WARMUP_TICKS",     "warmup_ticks",          "int"],
		["OGMA_PICRAWLER_CRUSE_ENABLE_RULE6",     "enable_rule_6",         "bool"],
		["OGMA_PICRAWLER_CRUSE_BODY_GATE_MIN",    "body_state_min_threshold", "float"],
	]
	var applied: Array = []
	for triple in overrides:
		var env_name: String = triple[0]
		var param_key: String = triple[1]
		var kind: String = triple[2]
		var raw := OS.get_environment(env_name)
		if raw == "":
			continue
		var value
		match kind:
			"float": value = raw.to_float()
			"int":   value = int(raw.to_int())
			"bool":
				value = (raw == "1" or raw.to_lower() == "true")
			_: value = raw
		var result: Dictionary = brain.apply_patch({
			"op":    "set_param",
			"id":    module_id,
			"key":   param_key,
			"value": value,
		})
		if bool(result.get("success", false)):
			applied.append("%s=%s" % [param_key, str(value)])
		else:
			push_warning("PicrawlerBody: cruse env override %s=%s failed: %s"
				% [param_key, str(value), String(result.get("error", "?"))])
	if not applied.is_empty():
		print("PicrawlerBody: applied Cruse env overrides → %s" % ", ".join(applied))

# 2026-06-08 — push @export var cruse_bias_gain into the CruseCoordinator's
# HotMutable cruse_bias_gain param via apply_patch.  Called from:
#   1. The @export setter (fires on direct assignment + on curriculum overrides
#      via _body.set(field, v))
#   2. _ready() after brain is initialized (the initial @export-default
#      assignment happens before _ready() and would no-op without this)
# Cheap idempotent — single brain.apply_patch call.  Silently no-ops when
# brain isn't ready yet or doesn't have a cruse_coordinator module.
func _print_cruse_trace() -> void:
	# 2026-06-08 V2 textual Cruse inspector — periodic compact summary.
	# Reads cruse_coordinator's snapshot_state via brain.get_module_snapshot
	# and formats:
	#   planted state per leg (●=stance, ○=swing)
	#   rule fire deltas since last emit (fires/sec normalized)
	#   per-knee bias_norm (the joint type Joseph diagnosed as fold-pressured)
	#   current cruse_bias_gain (curriculum-controlled) + body_state value
	# No-op when CruseCoordinator absent or brain not ready.
	if brain == null or not brain.has_method("get_module_snapshot"):
		return
	if brain.has_method("is_brain_ready") and not brain.is_brain_ready():
		return
	var cs: String = brain.get_module_snapshot("cruse_coordinator")
	if cs == "":
		return
	var cj_var: Variant = JSON.parse_string(cs)
	if not (cj_var is Dictionary):
		return
	var cj: Dictionary = cj_var
	var r1: int = int(cj.get("total_rule1_fires", 0))
	var r2: int = int(cj.get("total_rule2_fires", 0))
	var r3: int = int(cj.get("total_rule3_fires", 0))
	var elapsed_ticks: int = tick_counter - _cruse_trace_last_tick
	if elapsed_ticks <= 0: elapsed_ticks = 1
	var elapsed_sec: float = float(elapsed_ticks) * TAU
	var d_r1: float = float(r1 - _cruse_trace_last_r1) / elapsed_sec
	var d_r2: float = float(r2 - _cruse_trace_last_r2) / elapsed_sec
	var d_r3: float = float(r3 - _cruse_trace_last_r3) / elapsed_sec
	_cruse_trace_last_r1 = r1
	_cruse_trace_last_r2 = r2
	_cruse_trace_last_r3 = r3
	_cruse_trace_last_tick = tick_counter
	# Per-leg planted state (●/○).
	var legs: Array = cj.get("legs", [])
	var planted_str := ""
	var leg_labels := ["FL", "FR", "RL", "RR"]
	for i in range(min(legs.size(), 4)):
		var p: bool = bool(legs[i].get("is_planted", false))
		planted_str += leg_labels[i] + ("●" if p else "○")
		if i < 3: planted_str += " "
	# Per-joint bias norms.  V2 trace shows hip1, hip2, AND knee rows so we
	# can diagnose each channel independently.  hip1 (Move 2): swing-direction
	# bias.  hip2: load-bearing standing scaffold.  knee: gain-gated to 0 by
	# default (Move 1).
	var pms: Array = cj.get("premotors", [])
	var hip1_norms: Dictionary = {}
	var hip2_norms: Dictionary = {}
	var knee_norms: Dictionary = {}
	for pm in pms:
		var pmid: String = str(pm.get("id", ""))
		var jkind: String = str(pm.get("joint_kind", ""))
		var bias_norm: float = float(pm.get("last_bias_norm", 0.0))
		var leg: String = pmid.replace("premotor_", "")
		if jkind == "knee":
			leg = leg.replace("_knee", "")
			knee_norms[leg] = bias_norm
		elif jkind == "hip2":
			leg = leg.replace("_hip2", "")
			hip2_norms[leg] = bias_norm
		elif jkind == "hip1":
			leg = leg.replace("_hip1", "")
			hip1_norms[leg] = bias_norm
	var hip1_str := "fl=%.2f fr=%.2f rl=%.2f rr=%.2f" % [
		hip1_norms.get("fl", 0.0), hip1_norms.get("fr", 0.0),
		hip1_norms.get("rl", 0.0), hip1_norms.get("rr", 0.0),
	]
	var hip2_str := "fl=%.2f fr=%.2f rl=%.2f rr=%.2f" % [
		hip2_norms.get("fl", 0.0), hip2_norms.get("fr", 0.0),
		hip2_norms.get("rl", 0.0), hip2_norms.get("rr", 0.0),
	]
	var knee_str := "fl=%.2f fr=%.2f rl=%.2f rr=%.2f" % [
		knee_norms.get("fl", 0.0), knee_norms.get("fr", 0.0),
		knee_norms.get("rl", 0.0), knee_norms.get("rr", 0.0),
	]
	var body_state: float = float(cj.get("body_state_value", 1.0))
	var gain: float = float(cj.get("cruse_bias_gain", 0.0))
	var gain_hip1: float = float(cj.get("cruse_bias_gain_hip1", 0.0))
	var gain_hip2: float = float(cj.get("cruse_bias_gain_hip2", 1.0))
	var gain_knee: float = float(cj.get("cruse_bias_gain_knee", 0.0))
	print("[CRUSE] t=%d planted:%s  r1/r2/r3=%.0f/%.0f/%.0f/s  hip1_bias[%s]  hip2_bias[%s]  knee_bias[%s]  gain=%.2f hip1=%.2f hip2=%.2f knee=%.2f body_state=%.2f" % [
		tick_counter, planted_str, d_r1, d_r2, d_r3, hip1_str, hip2_str, knee_str, gain, gain_hip1, gain_hip2, gain_knee, body_state])

func _apply_cruse_bias_gain_to_brain() -> void:
	if brain == null or not brain.has_method("apply_patch"):
		return
	# Brain instance exists but its internal `initialized_` may still be false
	# (e.g. when the @export setter fires during scene-load before _ready()
	# has called brain.setup).  Skip silently — _ready() re-pushes the value
	# after brain setup.
	if brain.has_method("is_brain_ready") and not brain.is_brain_ready():
		return
	var result: Dictionary = brain.apply_patch({
		"op":    "set_param",
		"id":    "cruse_coordinator",
		"key":   "cruse_bias_gain",
		"value": cruse_bias_gain,
	})
	if bool(result.get("success", false)):
		print("PicrawlerBody: cruse_bias_gain = %.3f → applied to cruse_coordinator" % cruse_bias_gain)
	else:
		var err: String = String(result.get("error", "?"))
		# Silently swallow "unknown module" errors — config may not include cruse_coordinator.
		if not err.contains("unknown") and not err.contains("not found"):
			push_warning("PicrawlerBody: cruse_bias_gain apply_patch failed: %s" % err)

# 2026-06-08 — push @export var cruse_bias_gain_knee into CruseCoordinator's
# HotMutable cruse_bias_gain_knee param.  Same pattern as _apply_cruse_bias_gain_to_brain.
func _apply_cruse_bias_gain_knee_to_brain() -> void:
	if brain == null or not brain.has_method("apply_patch"):
		return
	if brain.has_method("is_brain_ready") and not brain.is_brain_ready():
		return
	var result: Dictionary = brain.apply_patch({
		"op":    "set_param",
		"id":    "cruse_coordinator",
		"key":   "cruse_bias_gain_knee",
		"value": cruse_bias_gain_knee,
	})
	if bool(result.get("success", false)):
		print("PicrawlerBody: cruse_bias_gain_knee = %.3f → applied to cruse_coordinator" % cruse_bias_gain_knee)
	else:
		var err: String = String(result.get("error", "?"))
		if not err.contains("unknown") and not err.contains("not found"):
			push_warning("PicrawlerBody: cruse_bias_gain_knee apply_patch failed: %s" % err)

# 2026-06-08 Move 2 — push @export var cruse_bias_gain_hip1 into Cruse.
func _apply_cruse_bias_gain_hip1_to_brain() -> void:
	if brain == null or not brain.has_method("apply_patch"):
		return
	if brain.has_method("is_brain_ready") and not brain.is_brain_ready():
		return
	var result: Dictionary = brain.apply_patch({
		"op":    "set_param",
		"id":    "cruse_coordinator",
		"key":   "cruse_bias_gain_hip1",
		"value": cruse_bias_gain_hip1,
	})
	if bool(result.get("success", false)):
		print("PicrawlerBody: cruse_bias_gain_hip1 = %.3f → applied to cruse_coordinator" % cruse_bias_gain_hip1)
	else:
		var err: String = String(result.get("error", "?"))
		if not err.contains("unknown") and not err.contains("not found"):
			push_warning("PicrawlerBody: cruse_bias_gain_hip1 apply_patch failed: %s" % err)

# 2026-06-09 Move 4 — push @export var cruse_bias_gain_hip2 into Cruse.
func _apply_cruse_bias_gain_hip2_to_brain() -> void:
	if brain == null or not brain.has_method("apply_patch"):
		return
	if brain.has_method("is_brain_ready") and not brain.is_brain_ready():
		return
	var result: Dictionary = brain.apply_patch({
		"op":    "set_param",
		"id":    "cruse_coordinator",
		"key":   "cruse_bias_gain_hip2",
		"value": cruse_bias_gain_hip2,
	})
	if bool(result.get("success", false)):
		print("PicrawlerBody: cruse_bias_gain_hip2 = %.3f → applied to cruse_coordinator" % cruse_bias_gain_hip2)
	else:
		var err: String = String(result.get("error", "?"))
		if not err.contains("unknown") and not err.contains("not found"):
			push_warning("PicrawlerBody: cruse_bias_gain_hip2 apply_patch failed: %s" % err)

# 2026-06-09 Move 5 — push all three saturation params at once (single
# apply_patch round trip per setter would multiply network chatter on
# the typical case of dragging the zone sliders).
func _apply_saturation_params_to_brain() -> void:
	if brain == null or not brain.has_method("apply_patch"):
		return
	if brain.has_method("is_brain_ready") and not brain.is_brain_ready():
		return
	for spec in [
		{"key": "saturation_gate_enabled", "value": saturation_gate_enabled},
		{"key": "saturation_zone_min",     "value": saturation_zone_min},
		{"key": "saturation_zone_max",     "value": saturation_zone_max},
	]:
		var result: Dictionary = brain.apply_patch({
			"op":    "set_param",
			"id":    "cruse_coordinator",
			"key":   spec["key"],
			"value": spec["value"],
		})
		if not bool(result.get("success", false)):
			var err: String = String(result.get("error", "?"))
			if not err.contains("unknown") and not err.contains("not found"):
				push_warning("PicrawlerBody: %s apply_patch failed: %s" % [spec["key"], err])
	print("PicrawlerBody: saturation_gate enabled=%s zone=[%.2f, %.2f]" % [
		str(saturation_gate_enabled), saturation_zone_min, saturation_zone_max])

# Phase 6.7++ — capture the GNG / TLE / health state of every EPM at body
# init, before any tick has run.  This pristine snapshot is what the swap
# mechanism restores TO on each HK fire.  Stored as JSON strings keyed by
# module_id (matches brain.set_module_snapshot's expected format).
func _capture_epm_pristine_snapshots() -> void:
	_epm_pristine_snapshots.clear()
	if not (brain.has_method("get_module_metrics") \
			and brain.has_method("get_module_snapshot")):
		push_warning("PicrawlerBody: EPM-swap mechanism requires get_module_metrics + get_module_snapshot; skipping pristine capture")
		return
	var metrics: Dictionary = brain.get_module_metrics()
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		if String(m.get("type", "")) != "EPM":
			continue
		var snap_s: String = brain.get_module_snapshot(mod_id)
		if snap_s == "":
			push_warning("PicrawlerBody: empty pristine snapshot for EPM %s" % mod_id)
			continue
		_epm_pristine_snapshots[mod_id] = snap_s
	print("PicrawlerBody: captured pristine snapshots for %d EPM(s): %s" % [
		_epm_pristine_snapshots.size(),
		_epm_pristine_snapshots.keys()])

# Phase 6.7++ — EPM-swap-on-HK trigger.  Called from _emit_jsonl at diag
# cadence.  Detects HomeokineticExploration's episodes_armed counter
# incrementing (= gate just fired) and, if cooldown allows, restores every
# captured EPM to its pristine snapshot.  Premotor / voter / drive state
# is UNTOUCHED — only the perceptual layer resets.
# ---- Phase 6.10 + 6.13 — UI notification helpers --------------------
# Posts a transient message + sets the expiry tick.  HUD reads it
# per-frame and displays the message until tick_counter >= expiry.
func _ui_notify(msg: String, duration_ticks: int = 180) -> void:
	_ui_notification = msg
	_ui_notification_until_tick = tick_counter + duration_ticks

# Friendly byte size for HUD/log readability.
func _fmt_bytes(n: int) -> String:
	if n >= 1024 * 1024: return "%.1f MB" % (n / 1048576.0)
	if n >= 1024:        return "%.1f KB" % (n / 1024.0)
	return "%d B" % n

# Phase 6.10 — brain state save/load (Scope A: brain-only).
# Loops every module in the topology, calls brain.get_module_snapshot
# on each, bundles into a single JSON file under user://saved_states/.
# Use case: skip the 10-min "learn to stand" each time we want to test
# a new reward policy / curriculum / brain wiring.  Caveat: schema
# changes (different module ids, different latent_dim, different
# n_intents) make the snapshot incompatible — load validates and
# refuses with a clear error.
const _SAVED_STATES_DIR: String = "user://saved_states/"
const _SAVED_STATE_VERSION: int = 1
const _SAVED_STATE_EXT: String  = ".brain_state.json"
# OGMA_PICRAWLER_RESUME_STATE — optional path to a .brain_state.json to
# restore immediately after brain.setup().  Accepts:
#   "" / unset      → cold start (default)
#   "most_recent"   → use _load_most_recent_brain_state()
#   "<filepath>"    → load that specific file
var _resume_state_path: String = ""
# Phase 7.x — bootstrap-preload-as-curriculum-component.  Tracks which
# resume_state_path values have already been loaded this session so
# repeated transitions into the same preloaded stage don't trample
# learning that's happened since the first entry.  Key = the spec
# string from the curriculum stage; value = true once loaded.
var _curriculum_resume_done: Dictionary = {}
# Transient HUD notification — set by F5/F9 (or other significant events).
# The HUD reads these per-frame and displays the message until the tick
# matches _ui_notification_until_tick.  3-second flash default.
var _ui_notification: String = ""
var _ui_notification_until_tick: int = 0

func _save_brain_state(custom_name: String = "") -> String:
	if not (brain.has_method("get_module_metrics") \
			and brain.has_method("get_module_snapshot")):
		push_warning("PicrawlerBody: brain lacks snapshot API; cannot save")
		return ""
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(_SAVED_STATES_DIR))
	var metrics: Dictionary = brain.get_module_metrics()
	var snapshots: Dictionary = {}
	for mod_id in metrics:
		var snap_s: String = brain.get_module_snapshot(mod_id)
		if snap_s == "":
			push_warning("PicrawlerBody: empty snapshot for module '%s' — skipping" % mod_id)
			continue
		snapshots[mod_id] = snap_s
	if snapshots.is_empty():
		push_warning("PicrawlerBody: no module snapshots captured; save aborted")
		return ""
	# Compose filename: <timestamp>_<config-basename>_t<tick>.brain_state.json
	var name: String = custom_name
	if name == "":
		var ts: String = Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
		var cfg_base: String = config_path.get_file().trim_suffix(".json")
		name = "%s_%s_t%d" % [ts, cfg_base, tick_counter]
	if not name.ends_with(_SAVED_STATE_EXT):
		name += _SAVED_STATE_EXT
	var path: String = _SAVED_STATES_DIR + name
	var payload: Dictionary = {
		"version":     _SAVED_STATE_VERSION,
		"timestamp":   Time.get_datetime_string_from_system(),
		"tick":        tick_counter,
		"config_path": config_path,
		"snapshots":   snapshots,
	}
	var f: FileAccess = FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_warning("PicrawlerBody: failed to open %s for write" % path)
		return ""
	f.store_string(JSON.stringify(payload))
	f.close()
	return path

func _load_brain_state(filepath: String) -> bool:
	if not (brain.has_method("set_module_snapshot")):
		push_warning("PicrawlerBody: brain lacks set_module_snapshot; cannot load")
		return false
	if not FileAccess.file_exists(filepath):
		push_warning("PicrawlerBody: saved state not found: %s" % filepath)
		return false
	var f: FileAccess = FileAccess.open(filepath, FileAccess.READ)
	if f == null:
		push_warning("PicrawlerBody: failed to open %s for read" % filepath)
		return false
	var raw: String = f.get_as_text()
	f.close()
	var parsed: Variant = JSON.parse_string(raw)
	if not (parsed is Dictionary):
		push_warning("PicrawlerBody: %s is not a valid brain-state JSON" % filepath)
		return false
	var payload: Dictionary = parsed
	if int(payload.get("version", 0)) != _SAVED_STATE_VERSION:
		push_warning("PicrawlerBody: %s has version %s, expected %d — refusing load" % [
			filepath, payload.get("version"), _SAVED_STATE_VERSION])
		return false
	var saved_cfg: String = String(payload.get("config_path", ""))
	if saved_cfg != "" and saved_cfg != config_path:
		push_warning("PicrawlerBody: brain-state was saved under '%s' but current config is '%s'.  Loading anyway — incompatible module shapes will fail downstream.  Cancel and reload with matching config if this isn't intended." % [saved_cfg, config_path])
	var snapshots_v: Variant = payload.get("snapshots", {})
	if not (snapshots_v is Dictionary):
		push_warning("PicrawlerBody: snapshots block in %s is not a dict" % filepath)
		return false
	var snapshots: Dictionary = snapshots_v
	var restored: Array = []
	var failed: Array = []
	for mod_id in snapshots:
		var ok: bool = brain.set_module_snapshot(mod_id, String(snapshots[mod_id]))
		if ok:
			restored.append(mod_id)
		else:
			failed.append(mod_id)
	print("PicrawlerBody: brain state load — restored %d modules (%s)%s" % [
		restored.size(),
		restored,
		("; FAILED: " + str(failed)) if failed.size() > 0 else ""])
	return failed.size() == 0

# Rich-info wrappers used by the F5/F9 handlers — return Dictionary
# with success flag + counts for HUD notifications.  The plain-string
# _save_brain_state / _load_brain_state remain for env-var / CLI use.

func _save_brain_state_info(custom_name: String = "") -> Dictionary:
	var path: String = _save_brain_state(custom_name)
	if path == "":
		return {"ok": false, "error": "save returned empty path"}
	var module_count: int = 0
	var size: int = 0
	var f: FileAccess = FileAccess.open(path, FileAccess.READ)
	if f != null:
		size = int(f.get_length())
		var raw: String = f.get_as_text()
		f.close()
		var parsed: Variant = JSON.parse_string(raw)
		if parsed is Dictionary:
			var snaps: Variant = parsed.get("snapshots", {})
			if snaps is Dictionary:
				module_count = (snaps as Dictionary).size()
	return {
		"ok": true,
		"path": path,
		"module_count": module_count,
		"size": size,
	}

func _load_brain_state_info(filepath: String) -> Dictionary:
	if not (brain.has_method("set_module_snapshot")):
		return {"ok": false, "error": "brain lacks set_module_snapshot"}
	if not FileAccess.file_exists(filepath):
		return {"ok": false, "error": "file not found"}
	var f: FileAccess = FileAccess.open(filepath, FileAccess.READ)
	if f == null:
		return {"ok": false, "error": "open for read failed"}
	var raw: String = f.get_as_text()
	f.close()
	var parsed: Variant = JSON.parse_string(raw)
	if not (parsed is Dictionary):
		return {"ok": false, "error": "not valid JSON"}
	var payload: Dictionary = parsed
	if int(payload.get("version", 0)) != _SAVED_STATE_VERSION:
		return {"ok": false, "error": "version mismatch"}
	var saved_cfg: String = String(payload.get("config_path", ""))
	# Compare by FILENAME only — full res:// paths can superficially differ
	# (path normalization, rename, dir-move) without indicating an actual
	# module-shape incompatibility.  If the basenames match, the snapshot
	# is structurally valid for this body.  Module-level shape mismatches
	# still get caught by set_module_snapshot returning false per module.
	var saved_base: String = saved_cfg.get_file()
	var cur_base: String   = config_path.get_file()
	var config_match: bool = (saved_cfg == "" or saved_base == cur_base)
	if not config_match:
		push_warning("PicrawlerBody: load config mismatch — saved='%s' current='%s' (comparing basenames)" % [saved_base, cur_base])
	var snapshots_v: Variant = payload.get("snapshots", {})
	if not (snapshots_v is Dictionary):
		return {"ok": false, "error": "snapshots not a dict"}
	var snapshots: Dictionary = snapshots_v
	var restored: Array = []
	var failed: Array = []
	for mod_id in snapshots:
		if brain.set_module_snapshot(mod_id, String(snapshots[mod_id])):
			restored.append(mod_id)
		else:
			failed.append(mod_id)
	return {
		"ok": true,
		"path": filepath,
		"restored_count": restored.size(),
		"failed_count":   failed.size(),
		"restored":       restored,
		"failed":         failed,
		"config_match":   config_match,
		"saved_config":   saved_cfg,
		"saved_base":     saved_base,
		"current_base":   cur_base,
	}

func _load_most_recent_brain_state_info() -> Dictionary:
	var path: String = _find_most_recent_brain_state_path()
	if path == "":
		return {"ok": false, "error": "no .brain_state.json found"}
	return _load_brain_state_info(path)

# Returns the path of the loaded state, or "" if none available.
func _load_most_recent_brain_state() -> String:
	var path: String = _find_most_recent_brain_state_path()
	if path == "" or not _load_brain_state(path):
		return ""
	return path

# Lists every .brain_state.json in the save dir and picks the one with
# the most recent file-modification time.  Robust to mixed filename
# formats (auto-named timestamp + user-named) — earlier we relied on
# alphabetic sort, which broke when a non-timestamped save like
# "same_cfg.brain_state.json" sorted after the timestamped ones and
# was wrongly picked as "most recent".
func _find_most_recent_brain_state_path() -> String:
	return _find_most_recent_brain_state_matching("")

# Same as _find_most_recent_brain_state_path() but restricts to filenames
# containing `name_filter` (substring match).  Pass "" to match everything.
# Used by the bootstrap-preload curriculum feature so a stage spec like
#   "resume_state_path": "most_recent:per_servo"
# resolves to the newest saved-state file whose name contains "per_servo"
# (i.e., the latest baseline 6.17b training run regardless of which
# specific timestamp / tick-id stamp it carries).
func _find_most_recent_brain_state_matching(name_filter: String) -> String:
	var dir: DirAccess = DirAccess.open(_SAVED_STATES_DIR)
	if dir == null:
		return ""
	var best_path: String = ""
	var best_mtime: int = -1
	dir.list_dir_begin()
	var fname: String = dir.get_next()
	while fname != "":
		if not dir.current_is_dir() and fname.ends_with(_SAVED_STATE_EXT):
			if name_filter == "" or fname.find(name_filter) >= 0:
				var full: String = _SAVED_STATES_DIR + fname
				var mtime: int = int(FileAccess.get_modified_time(full))
				if mtime > best_mtime:
					best_mtime = mtime
					best_path = full
		fname = dir.get_next()
	dir.list_dir_end()
	return best_path

# Resolves a curriculum stage's `resume_state_path` spec into an actual
# loadable path.  Accepted forms:
#   ""                          → no preload (caller short-circuits)
#   "most_recent"               → newest saved state in the save dir
#   "most_recent:<substr>"      → newest saved state whose filename
#                                 contains <substr> (e.g. "per_servo"
#                                 to grab the latest baseline run)
#   "user://saved_states/X.json" → direct path, no resolution
#   "<bare filename>"           → resolved to _SAVED_STATES_DIR + filename
func _resolve_resume_state_path(spec: String) -> String:
	if spec == "":
		return ""
	if spec == "most_recent":
		return _find_most_recent_brain_state_matching("")
	if spec.begins_with("most_recent:"):
		var prefix: String = spec.substr("most_recent:".length())
		return _find_most_recent_brain_state_matching(prefix)
	if spec.begins_with("user://") or spec.begins_with("res://") or spec.begins_with("/"):
		return spec
	# Bare filename — treat as relative to the save dir.
	return _SAVED_STATES_DIR + spec

func _check_epm_swap_on_hk(metrics: Dictionary) -> void:
	if not _epm_swap_enabled:
		return
	if _epm_pristine_snapshots.is_empty():
		return
	if not brain.has_method("set_module_snapshot"):
		return
	# Find HomeokineticExploration block in metrics; use episodes_armed
	# as the fire signal.
	var hk_armed: int = -1
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		if String(m.get("type", "")) == "HomeokineticExploration":
			hk_armed = int(m.get("episodes_armed", 0))
			break
	if hk_armed < 0:
		return   # no HK module in topology — nothing to react to
	# First sighting: just record the baseline, don't swap (we don't
	# want to swap on the very first diag tick).
	if _last_hk_episodes_armed < 0:
		_last_hk_episodes_armed = hk_armed
		return
	if hk_armed <= _last_hk_episodes_armed:
		return   # no new fire this diag tick
	var fires_this_diag: int = hk_armed - _last_hk_episodes_armed
	_last_hk_episodes_armed = hk_armed
	# Cooldown gate (5 min default).  Skip if we swapped recently.
	if _epm_swap_at_tick > 0 \
			and tick_counter < _epm_swap_at_tick + _EPM_SWAP_COOLDOWN_TICKS:
		return
	# SWAP: restore every captured EPM to its pristine snapshot.
	var swapped: Array = []
	for mod_id in _epm_pristine_snapshots:
		if brain.set_module_snapshot(mod_id, _epm_pristine_snapshots[mod_id]):
			swapped.append(mod_id)
	if swapped.size() > 0:
		_epm_swap_total += 1
		_epm_swap_at_tick = tick_counter
		print("PicrawlerBody: EPM swap #%d at tick=%d (hk fired %d times this diag); swapped %s" % [
			_epm_swap_total, tick_counter, fires_this_diag, swapped])

func _check_escape_detector(metrics: Dictionary, chassis_pos: Vector3) -> void:
	_escape_active_now_count = 0
	if not _escape_detector_enabled:
		return
	# Update chassis_xz history (deque, capped at _ESC_STUCK_DIAG_SAMPLES).
	_chassis_xz_history.push_back(Vector2(chassis_pos.x, chassis_pos.z))
	while _chassis_xz_history.size() > _ESC_STUCK_DIAG_SAMPLES:
		_chassis_xz_history.pop_front()
	var chassis_stuck: bool = false
	if _chassis_xz_history.size() >= _ESC_STUCK_DIAG_SAMPLES:
		var min_x: float =  INF
		var max_x: float = -INF
		var min_z: float =  INF
		var max_z: float = -INF
		for p in _chassis_xz_history:
			if p.x < min_x: min_x = p.x
			if p.x > max_x: max_x = p.x
			if p.y < min_z: min_z = p.y   # Vector2.y stores z
			if p.y > max_z: max_z = p.y
		chassis_stuck = (max_x - min_x) < _ESC_MOTION_EPS \
					and (max_z - min_z) < _ESC_MOTION_EPS
	var da_at_floor: bool = brain.get_dopamine() < _ESC_DA_FLOOR
	# Per-Premotor trigger evaluation.
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		if String(m.get("type", "")) != "Premotor":
			continue
		var h_now: float = float(m.get("last_entropy", 0.0))
		# EMA update + peak tracking.  Initialise on first sight.
		var ema_prev: float = float(_pre_h_ema.get(mod_id, h_now))
		var ema: float = (1.0 - _ESC_H_EMA_ALPHA) * ema_prev + _ESC_H_EMA_ALPHA * h_now
		_pre_h_ema[mod_id] = ema
		var peak: float = max(float(_pre_h_peak.get(mod_id, 0.0)), ema)
		_pre_h_peak[mod_id] = peak
		# Cannot trigger until peak is meaningful (avoids firing during
		# very early training when entropy is still climbing from init).
		if peak < _ESC_MIN_PEAK_FOR_TRIGGER:
			continue
		# Trigger condition.  Each clause is necessary.
		if ema >= _ESC_COLLAPSE_FRACTION * peak:
			continue
		if not _escape_force_fire:
			if not chassis_stuck:
				continue
			if not da_at_floor:
				continue
		var cd: int = int(_escape_cooldown_until.get(mod_id, 0))
		if tick_counter < cd:
			continue
		# KICK.  Pull snapshot, perturb W, write back.  Snapshot.W is a
		# nested array (n_intents × latent_dim) of floats.  Same
		# JSON-round-trip pattern leg_symmetry uses.
		var snap_s: String = brain.get_module_snapshot(mod_id)
		if snap_s == "":
			continue
		var snap: Variant = JSON.parse_string(snap_s)
		if not (snap is Dictionary):
			continue
		if not snap.has("W"):
			continue
		# Premotor's mat_to_json serializes W as {"rows": N, "cols": M,
		# "data": [flat row-major floats]}.  See Premotor.cpp::mat_to_json.
		# Same structure for E.  We perturb each element of the flat
		# `data` array; rows/cols are unchanged.
		var W: Variant = snap["W"]
		if not (W is Dictionary) or not W.has("data"):
			continue
		var data: Variant = W["data"]
		if not (data is Array):
			continue
		var new_data: Array = []
		new_data.resize(data.size())
		for i in range(data.size()):
			new_data[i] = float(data[i]) + _escape_rng.randfn() * _ESC_KICK_SIGMA
		W["data"] = new_data
		snap["W"] = W
		brain.set_module_snapshot(mod_id, JSON.stringify(snap))
		# Record fire.
		_escape_fired_total += 1
		_escape_active_now_count += 1
		_escape_fired_per_pm[mod_id] = int(_escape_fired_per_pm.get(mod_id, 0)) + 1
		_escape_cooldown_until[mod_id] = tick_counter + _ESC_COOLDOWN_TICKS

func _avg_array_dicts(a: Dictionary, b: Dictionary) -> Dictionary:
	var out: Dictionary = {}
	for k in a:
		if b.has(k):
			out[k] = _avg_nested_arrays(a[k], b[k])
	return out

func _avg_nested_arrays(a: Variant, b: Variant) -> Variant:
	# Handles three shapes:
	#   (1) 1-D vector       — flat Array of floats
	#   (2) 2-D matrix       — Array of Arrays of floats
	#   (3) Eigen flat-data  — Dictionary {"rows", "cols", "data": [flat row-major floats]}
	#                          (the actual serialization Premotor uses for W/b/E,
	#                          per mat_to_json in Premotor.cpp:1089)
	# Returns a structurally-identical value with element-wise means.
	# Returns `a` unchanged if shapes diverge or types mismatch.
	#
	# Phase 6.7 fix: prior implementation only handled (1) and (2),
	# silently no-op'ed on (3) and returned `a` — which made
	# _sync_leg_symmetry a copy of pair[0] onto pair[1] rather than the
	# intended average.  Stage B's headline result still held because
	# symmetry was still imposed (both ended identical), but the
	# mechanism description in docs/picrawler_brain_config_matrix.md
	# called it averaging when it was actually copying.  Now correctly
	# averages by walking the flat `data` array element-wise.
	if a is Dictionary and b is Dictionary \
			and a.has("data") and b.has("data") \
			and (a["data"] is Array) and (b["data"] is Array):
		var da: Array = a["data"]
		var db: Array = b["data"]
		if da.size() != db.size():
			return a
		var out_data: Array = []
		out_data.resize(da.size())
		for i in range(da.size()):
			out_data[i] = (float(da[i]) + float(db[i])) * 0.5
		# Preserve rows/cols (and any other metadata) from a; only data changes.
		var out_dict: Dictionary = a.duplicate()
		out_dict["data"] = out_data
		return out_dict
	if not (a is Array) or not (b is Array):
		return a
	if a.size() != b.size():
		return a
	var out: Array = []
	out.resize(a.size())
	for i in range(a.size()):
		var ai: Variant = a[i]
		var bi: Variant = b[i]
		if ai is Array and bi is Array:
			out[i] = _avg_nested_arrays(ai, bi)
		else:
			out[i] = (float(ai) + float(bi)) * 0.5
	return out

# E1b — weighted blend (1−α)·a + α·b over the same shapes _avg_nested_arrays
# handles.  Used for LIGHT front-rear posture coupling.
func _blend_array_dicts(a: Dictionary, b: Dictionary, alpha: float) -> Dictionary:
	var out: Dictionary = {}
	for k in a:
		if b.has(k):
			out[k] = _blend_nested_arrays(a[k], b[k], alpha)
	return out

func _blend_nested_arrays(a: Variant, b: Variant, alpha: float) -> Variant:
	var ka: float = 1.0 - alpha
	if a is Dictionary and b is Dictionary \
			and a.has("data") and b.has("data") \
			and (a["data"] is Array) and (b["data"] is Array):
		var da: Array = a["data"]
		var db: Array = b["data"]
		if da.size() != db.size():
			return a
		var out_data: Array = []
		out_data.resize(da.size())
		for i in range(da.size()):
			out_data[i] = ka * float(da[i]) + alpha * float(db[i])
		var out_dict: Dictionary = a.duplicate()
		out_dict["data"] = out_data
		return out_dict
	if not (a is Array) or not (b is Array):
		return a
	if a.size() != b.size():
		return a
	var out: Array = []
	out.resize(a.size())
	for i in range(a.size()):
		var ai: Variant = a[i]
		var bi: Variant = b[i]
		if ai is Array and bi is Array:
			out[i] = _blend_nested_arrays(ai, bi, alpha)
		else:
			out[i] = ka * float(ai) + alpha * float(bi)
	return out

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
func _relative_angle_world_axis(parent: RigidBody3D, child: RigidBody3D,
								axis_world: Vector3) -> float:
	# Relative rotation between two bodies, projected onto a world-frame axis.
	# Bodies started at IDENTITY basis at construction, so this measures
	# accumulated rotation from rest pose.
	var rel: Basis = parent.global_transform.basis.inverse() * child.global_transform.basis
	var q: Quaternion = Quaternion(rel.orthonormalized())
	if abs(q.w) > 0.9999:
		return 0.0
	var ang: float = q.get_angle()
	var ax:  Vector3 = q.get_axis()
	var dot: float = ax.dot(axis_world)
	return ang * sign(dot)

func _chassis_tilt(b: Basis) -> float:
	return acos(clamp(b.y.dot(Vector3.UP), -1.0, 1.0))

# Forward kinematics for one leg: given joint angles (hip1, hip2, knee),
# return [T_coxa, T_upper, T_lower] world transforms.  Used by calibrate
# mode to write body transforms directly (bypassing motors + joint
# constraints) so the slider value IS the joint angle, exactly.
func _fk_leg(i: int, t1: float, t2: float, t3: float) -> Array:
	var lift: Vector3 = Vector3(0, _suspend_lift_y, 0)
	var hip1_w: Vector3 = _hip1_world_c[i] + lift
	var hip2_w: Vector3 = _hip2_world_c[i] + lift
	var knee_w: Vector3 = _knee_world_c[i] + lift
	var coxa_c:  Vector3 = _coxa_rest_xform[i].origin  + lift
	var upper_c: Vector3 = _upper_rest_xform[i].origin + lift
	var lower_c: Vector3 = _lower_rest_xform[i].origin + lift
	# Hip1 rotation around world UP at hip1 anchor.
	var rot1: Basis = Basis(Quaternion(Vector3.UP, t1))
	var h1: Transform3D = Transform3D(rot1, hip1_w - rot1 * hip1_w)
	var t_coxa: Transform3D = h1 * Transform3D(Basis.IDENTITY, coxa_c)
	# Hip2 rotation around the leg-local lateral (which has been rotated
	# by hip1) at the hip2 anchor (which has also moved with hip1).
	var hip2_w_now: Vector3 = h1 * hip2_w
	var hip2_axis_now: Vector3 = rot1 * _hip2_axes[i]
	var rot2: Basis = Basis(Quaternion(hip2_axis_now, t2))
	var h2: Transform3D = Transform3D(rot2, hip2_w_now - rot2 * hip2_w_now)
	var t_upper: Transform3D = h2 * h1 * Transform3D(Basis.IDENTITY, upper_c)
	# Knee rotation — knee axis carried by both hip1 and hip2 rotations.
	var knee_w_now: Vector3 = h2 * h1 * knee_w
	var knee_axis_now: Vector3 = rot2 * rot1 * _knee_axes[i]
	var rot3: Basis = Basis(Quaternion(knee_axis_now, t3))
	var h3: Transform3D = Transform3D(rot3, knee_w_now - rot3 * knee_w_now)
	var t_lower: Transform3D = h3 * h2 * h1 * Transform3D(Basis.IDENTITY, lower_c)
	return [t_coxa, t_upper, t_lower]

# Powered-servo torque model (docs/servo_dynamics.md):
#   - PID-like with stiff Kp + heavy Kd, no Ki (matches metal-gear PWM tracker).
#   - Deadband around target → no torque if error is below PWM precision.
#   - Linear torque-speed dropoff: max torque tapers to 0 as ω → MAX_SERVO_SPEED
#     (mimics real motor's stall-torque vs no-load-speed curve).
# ---- ABLATION API (2026-08-04) --------------------------------------------------------
# Driven by the [K] panel and by OGMA_PICRAWLER_ABLATE for headless A/B.  Everything here
# is a no-op until something is actually ablated (`_abl_any`), so an un-ablated body is
# byte-identical — verified by measurement, not by argument.
func _abl_init() -> void:
	if _abl_kind.size() == 12:
		return
	_abl_kind.resize(12); _abl_detached.resize(12); _abl_hold_angle.resize(12)
	for k in range(12):
		_abl_kind[k] = AblKind.OK; _abl_detached[k] = 0; _abl_hold_angle[k] = 0.0
	_abl_saved_xform.resize(12); _abl_saved_nodes.resize(12)

func abl_idx(leg: int, joint: int) -> int:
	return leg * 3 + joint

func _abl_refresh_any() -> void:
	_abl_any = false
	for k in range(12):
		if _abl_kind[k] != AblKind.OK or _abl_detached[k] != 0:
			_abl_any = true; return

func abl_kind_of(leg: int, joint: int) -> int:
	_abl_init(); return _abl_kind[abl_idx(leg, joint)]

func abl_is_detached(leg: int, joint: int) -> bool:
	_abl_init(); return _abl_detached[abl_idx(leg, joint)] != 0

func abl_weak_scale() -> float:
	return _abl_weak_scale

func abl_set_weak_scale(v: float) -> void:
	_abl_weak_scale = clampf(v, 0.0, 1.0)

## Set one servo's failure mode.  Announced, because a knob that can silently do nothing is
## this codebase's characteristic trap (see postural_gain_joints in the ledger).
func abl_set_kind(leg: int, joint: int, kind: int) -> void:
	_abl_init()
	if leg < 0 or leg > 3 or joint < 0 or joint > 2:
		return
	var k: int = abl_idx(leg, joint)
	if _abl_kind[k] == kind:
		return
	_abl_kind[k] = kind
	_abl_refresh_any()
	print("PicrawlerBody: [ablate] %s %s servo -> %s  (tick %d)" % [
		ABL_LEG_NAMES[leg], ABL_JOINT_NAMES[joint], ABL_KIND_NAMES[kind], tick_counter])
	_ui_notify("[ablate] %s %s = %s" % [ABL_LEG_NAMES[leg], ABL_JOINT_NAMES[joint],
									   ABL_KIND_NAMES[kind]])

## Break the limb off at this joint (or put it back).  Everything DISTAL separates.
func abl_set_detached(leg: int, joint: int, on: bool) -> void:
	_abl_init()
	if leg < 0 or leg > 3 or joint < 0 or joint > 2:
		return
	if _coxas.size() <= leg or _uppers.size() <= leg or _lowers.size() <= leg:
		return
	var k: int = abl_idx(leg, joint)
	if (_abl_detached[k] != 0) == on:
		return
	var jt: Node = [_hip1_joints[leg], _hip2_joints[leg], _knee_joints[leg]][joint]
	# Distal chain: hip1 takes the whole leg, hip2 takes upper+lower, knee takes the shank.
	var distal: Array = []
	match joint:
		0: distal = [_coxas[leg], _uppers[leg], _lowers[leg]]
		1: distal = [_uppers[leg], _lowers[leg]]
		_: distal = [_lowers[leg]]
	if on:
		# Hold the reported angle BEFORE the geometry stops meaning anything.
		var meas: Array = [
			_relative_angle_world_axis(_chassis, _coxas[leg], Vector3.UP),
			_relative_angle_world_axis(_coxas[leg], _uppers[leg], _hip2_axes[leg]),
			_relative_angle_world_axis(_uppers[leg], _lowers[leg], _knee_axes[leg])]
		for j in range(joint, 3):
			_abl_hold_angle[abl_idx(leg, j)] = meas[j]
		# Save the chassis-relative pose so re-attachment restores the limb where it broke
		# rather than wherever it happened to fall.
		var saved: Array = []
		for b in distal:
			saved.append(_chassis.global_transform.affine_inverse() * b.global_transform)
		_abl_saved_xform[k] = saved
		# BREAK THE CONSTRAINT FIRST.  Freezing a body that is still jointed to the chassis
		# would pin the chassis to an infinite mass and wreck the whole robot.
		_abl_saved_nodes[k] = [jt.node_a, jt.node_b]
		jt.node_a = NodePath(); jt.node_b = NodePath()
		for b in distal:
			b.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
			b.freeze = true
			b.collision_layer = 0
			b.collision_mask = 0
			b.visible = false
		_abl_detached[k] = 1
	else:
		var saved: Array = _abl_saved_xform[k] if _abl_saved_xform[k] is Array else []
		for bi in range(distal.size()):
			var b: RigidBody3D = distal[bi]
			if bi < saved.size():
				b.global_transform = _chassis.global_transform * saved[bi]
			b.visible = true
			b.collision_layer = _LAYER_BODY     # leg segments live on _LAYER_BODY,
			b.collision_mask  = _LAYER_WORLD    # colliding only with the world (:4223)
			b.freeze = false
			b.linear_velocity = Vector3.ZERO
			b.angular_velocity = Vector3.ZERO
		var nodes: Array = _abl_saved_nodes[k] if _abl_saved_nodes[k] is Array else []
		if nodes.size() == 2:
			jt.node_a = nodes[0]; jt.node_b = nodes[1]
		_abl_detached[k] = 0
	_abl_refresh_any()
	print("PicrawlerBody: [ablate] %s %s %s  (tick %d)" % [
		ABL_LEG_NAMES[leg], ABL_JOINT_NAMES[joint],
		"DETACHED — limb distal to it removed" if on else "RE-ATTACHED", tick_counter])
	_ui_notify("[ablate] %s %s %s" % [ABL_LEG_NAMES[leg], ABL_JOINT_NAMES[joint],
									 "detached" if on else "re-attached"])

## Undo everything.  Restores detachments in DISTAL-FIRST order so a leg detached at hip1
## and again at the knee comes back in one piece.
func abl_clear() -> void:
	_abl_init()
	for leg in range(4):
		for joint in [2, 1, 0]:
			if _abl_detached[abl_idx(leg, joint)] != 0:
				abl_set_detached(leg, joint, false)
		for joint in range(3):
			abl_set_kind(leg, joint, AblKind.OK)
	_abl_refresh_any()
	print("PicrawlerBody: [ablate] ALL CLEARED (tick %d)" % tick_counter)

## Headless entry point: OGMA_PICRAWLER_ABLATE="0:2:dead,1:0:detach,3:1:weak"
## (leg:joint:kind, leg 0-3 = FL FR RL RR, joint 0-2 = hip1 hip2 knee).
## Applied at OGMA_PICRAWLER_ABLATE_AT (default 0 = from the start).
func _abl_parse_env() -> void:
	var spec: String = OS.get_environment("OGMA_PICRAWLER_ABLATE")
	if spec == "":
		return
	_abl_env_spec = spec
	var at: String = OS.get_environment("OGMA_PICRAWLER_ABLATE_AT")
	_abl_env_at = at.to_int() if at != "" else 0
	var ws: String = OS.get_environment("OGMA_PICRAWLER_ABLATE_WEAK_SCALE")
	if ws != "": _abl_weak_scale = ws.to_float()
	print("PicrawlerBody: ⚠ ABLATE armed — \"%s\" at tick %d (weak scale %.2f)" % [
		spec, _abl_env_at, _abl_weak_scale])

func _abl_apply_env_spec() -> void:
	for part in _abl_env_spec.split(",", false):
		var f: PackedStringArray = part.strip_edges().split(":")
		if f.size() != 3:
			push_warning("ABLATE: bad term '%s' (want leg:joint:kind)" % part); continue
		var leg: int = f[0].to_int()
		var joint: int = f[1].to_int()
		var kind: String = f[2].strip_edges().to_lower()
		if kind == "detach":
			abl_set_detached(leg, joint, true)
		elif ABL_KIND_NAMES.has(kind):
			abl_set_kind(leg, joint, ABL_KIND_NAMES.find(kind))
		else:
			push_warning("ABLATE: unknown kind '%s'" % kind)

func _powered_torque(target: float, angle: float, omega: float, max_torque: float) -> float:
	var error: float = target - angle
	# Effective deadband = max(SERVO_DEADBAND, motor_freeplay_rad).  The
	# constant deadband prevents motor chatter near the target; the
	# operator-settable freeplay extends that to a configurable mechanical-
	# slop zone where the motor leaves the joint alone.  In this zone only
	# gravity acts (a passive centering spring was attempted but is unstable
	# at picrawler scale via apply_torque — deferred to G6DOFJoint3D).
	# ⚠ TELEMETRY PATH ONLY (verified 2026-08-10 the hard way: a force boost added
	# here produced BIT-IDENTICAL trajectories — nothing physical reads this value;
	# the joints' impulse caps flow through imp_* → _set_motor_vf).  Any actuation
	# lever must attach THERE, not here.
	if abs(error) < max(SERVO_DEADBAND, motor_freeplay_rad):
		return 0.0
	# 2026-06-01 — motor_damping_factor scales the implicit Kd for tunable
	# springiness.  Default 1.0 = current overdamped behavior; <1 = under-
	# damped = oscillates before settling (springy compliance feel).
	var raw: float = SERVO_KP * error - SERVO_KD * motor_damping_factor * omega
	# Speed-dependent saturation: available torque drops linearly with |ω|.
	var speed_factor: float = clamp(1.0 - abs(omega) / MAX_SERVO_SPEED, 0.0, 1.0)
	var effective_max: float = max_torque * speed_factor
	return clamp(raw, -effective_max, effective_max)

# Unpowered-servo torque model (back-EMF + gear-train friction).
# Real metal-gear servos resist backdrive — viscous damping plus a
# Coulomb-friction floor that opposes any residual motion.
func _unpowered_torque(omega: float) -> float:
	var viscous: float = -UNPOWERED_VISCOUS_FRICTION * omega
	# Static friction opposes motion; the magnitude is bounded by
	# |ω| × some scale so we don't kick the joint backward at near-zero ω.
	var static_fric: float = 0.0
	if abs(omega) > 0.001:
		static_fric = -sign(omega) * min(UNPOWERED_STATIC_FRICTION, abs(omega) * 10.0)
	return viscous + static_fric

func export_servo_calibration() -> Dictionary:
	# Snapshot of all per-servo panel state, ready for JSON export.
	# The user can copy this into the body code as calibrated defaults.
	var out: Dictionary = {
		"servos": [],
		"constants": {
			"HIP1_TARGET_RANGE": HIP1_TARGET_RANGE,
			"HIP_TARGET_RANGE":  HIP_TARGET_RANGE,
			"HIP1_LIMIT": HIP1_LIMIT,
			"HIP2_LIMIT": HIP2_LIMIT,
			"KNEE_LIMIT_LOW":  KNEE_LIMIT_LOW,
			"KNEE_LIMIT_HIGH": KNEE_LIMIT_HIGH,
		},
	}
	for leg in range(4):
		for jt in range(3):
			var k: int = servo_idx(leg, jt)
			out["servos"].append({
				"leg":    LEG_NAMES[leg],
				"joint":  SERVO_NAMES_FULL[jt],
				"target": servo_targets[k],
				"sign":   servo_signs[k],
				"origin": servo_origins[k],
			})
	return out

func _print_calibrate_step_validation(step: Array) -> void:
	var lbl: String = str(step[3])
	var active_leg: int = int(step[0])
	print("\n=== CAL %d/%d: %s ===" % [
		_calibrate_step, SERVO_CALIBRATE_SEQUENCE.size(), lbl])
	# Per-leg state: joint angles (relative-rotation projection on hinge axis)
	# and toe world-position deviation from rest.
	print("  leg | hip1   hip2   knee  | toe Δ (mm from rest)")
	for i in range(4):
		var h1: float = _relative_angle_world_axis(_chassis, _coxas[i], Vector3.UP)
		var h2: float = _relative_angle_world_axis(_coxas[i], _uppers[i], _hip2_axes[i])
		var kn: float = _relative_angle_world_axis(_uppers[i], _lowers[i], _knee_axes[i])
		# Toe world-position now vs construction rest, accounting for suspend lift.
		var lift_y: float = _chassis.global_transform.origin.y - _chassis_rest_xform.origin.y
		var toe_now: Vector3 = _lowers[i].global_transform.origin + Vector3(0, -L3 * 0.5, 0)
		var toe_rest: Vector3 = _lower_rest_xform[i].origin + Vector3(0, -L3 * 0.5 + lift_y, 0)
		var dx_mm: float = (toe_now.x - toe_rest.x) * 1000.0
		var dy_mm: float = (toe_now.y - toe_rest.y) * 1000.0
		var dz_mm: float = (toe_now.z - toe_rest.z) * 1000.0
		var marker: String = "<<<" if i == active_leg else "   "
		print("  %s  %s | %+5.2f %+5.2f %+5.2f | Δ=(%+5.1f, %+5.1f, %+5.1f)" % [
			LEG_NAMES[i].to_upper(), marker, h1, h2, kn, dx_mm, dy_mm, dz_mm])

func _feet_y_array() -> Array:
	var out: Array = []
	for i in range(4):
		out.append(snappedf(_lowers[i].global_transform.origin.y - L3 * 0.5, 0.001))
	return out

# Per-leg foot-tip position in the CHASSIS-LOCAL frame [x=right, z=forward], for
# the hip1-sign / propulsion audit.  During a leg's stance (foot planted), the
# chassis rides forward over the planted foot, so the foot's LOCAL z must DECREASE
# (move posterior).  A planted foot whose local z increases (anterior) is doing
# negative work; large |Δx| during stance means it pushes laterally → yaw, not
# forward pull.  Ground truth, independent of nav intent.
func _foot_local_xz_array() -> Array:
	var inv: Transform3D = _chassis.global_transform.affine_inverse()
	var out: Array = []
	for i in range(4):
		var fl: Vector3 = inv * _lowers[i].global_transform.origin
		out.append([snappedf(fl.x, 0.001), snappedf(fl.z, 0.001)])
	return out

# Called from the gait-cycle detector when an in-flight cycle ends for ANY
# reason (good completion, wobble abort, timeout abort, low-progress abort).
# Updates the adaptive-threshold EMAs from the cycle's observed statistics
# and resets the cycle state.  `fired_reward` is informational only —
# the caller has already done the events.hit publish + counter bookkeeping.
func _end_gait_cycle(fired_reward: bool) -> void:
	# Record final progress for telemetry (good or bad).
	_gait_cycle_last_net_progress = _gait_cycle_displacement
	# Update adaptive-threshold EMAs from THIS cycle's observed statistics
	# regardless of outcome — we want the EMAs to track the body's typical
	# motion scale, not just the rare good cycles.  Use |progress| so a
	# wobble-aborted cycle with negative integration still feeds a positive
	# scale into the progress EMA.
	var a: float = gait_cycle_threshold_ema_alpha
	_gait_cycle_progress_ema = (1.0 - a) * _gait_cycle_progress_ema + a * abs(_gait_cycle_displacement)
	_gait_cycle_wobble_ema   = (1.0 - a) * _gait_cycle_wobble_ema   + a * _gait_cycle_max_backward_seen
	_gait_cycle_attempts_total += 1
	_gait_cycle_active = false

# V9 — full body CoG height (mass-weighted average of chassis + 12 leg segments).
# Chassis is 50% of body mass; leg positions matter a lot for the true CoG.
# Used as the height-reference for the reward when height_reference='body_cog' —
# more stable than chassis_y because internal forces between chassis and legs
# cancel in the body CoG (only gravity + ground reaction move it).
func _compute_body_cog_y() -> float:
	# The chassis CoG equals its node origin only while the chassis is a single
	# centred box.  With a multi-box chassis the CoM is deliberately offset, and
	# reading the origin would silently misreport the very quantity gate G1
	# checks.  cad.json leaves _chassis_com_valid false ⇒ unchanged arithmetic.
	var chassis_cog_y: float = _chassis.global_transform.origin.y
	if _chassis_com_valid:
		chassis_cog_y = (_chassis.global_transform * _chassis_com).y
	var weighted_y: float = CHASSIS_MASS * chassis_cog_y
	var n: int = _coxas.size()
	for i in range(n):
		weighted_y += COXA_MASS  * _coxas[i].global_transform.origin.y
		weighted_y += UPPER_MASS * _uppers[i].global_transform.origin.y
		weighted_y += LOWER_MASS * _lowers[i].global_transform.origin.y
	var total_mass: float = CHASSIS_MASS + n * (COXA_MASS + UPPER_MASS + LOWER_MASS)
	return weighted_y / total_mass

# ---- walk_over_there target helpers -----------------------------------------

func _ensure_target_pyramid_color() -> void:
	# 2026-06-07 — defensive idempotent refresh of the purple target color.
	# Called periodically from _step_one + on motor_test_mode toggle to address
	# Joseph's observation that the target pyramid does not turn purple when
	# entering experiment mode.  Now also AUTO-PICKS a target if curriculum
	# requires random_pyramid but none has been selected (handles launcher
	# timing edge cases + "Launch" vs "Start curriculum" button confusion).
	# Cheap idempotent operation — no-ops when nothing needs doing.
	if _pyramid_meshes.is_empty():
		return
	# Auto-pick: body target_mode demands a target but none chosen yet.
	# Source of truth is the body's target_mode field — set by curriculum stage
	# entry OR by the reward_panel UI dropdown.  This means manual Launch-mode
	# users can enable random_pyramid via the UI without a curriculum.
	if walk_target_idx < 0:
		if target_mode != "random_pyramid":
			return
		_walk_target_rng.seed = (_resolved_seed if _resolved_seed >= 0 else 42) ^ 0xA17D60
		print("[TARGET-DEBUG] defensive target-pick fired (target_mode=random_pyramid but walk_target_idx was -1)")
		_select_random_pyramid_target()
		return   # _select_random_pyramid_target sets the material itself
	# Inverse: target was picked but target_mode was set back to "off".
	if target_mode == "off" and walk_target_idx >= 0:
		print("[TARGET-DEBUG] clearing pyramid target (target_mode=off)")
		_clear_pyramid_target()
		return
	# Material refresh path.
	if walk_target_idx >= _pyramid_meshes.size():
		return
	if _target_pyramid_mat == null:
		_target_pyramid_mat = StandardMaterial3D.new()
		_target_pyramid_mat.albedo_color = _TARGET_PYRAMID_COLOR
		_target_pyramid_mat.roughness = 0.6
		_target_pyramid_mat.emission_enabled = true
		_target_pyramid_mat.emission = _TARGET_PYRAMID_COLOR * 0.4
		_target_pyramid_mat.emission_energy_multiplier = 1.0
	var mesh: MeshInstance3D = _pyramid_meshes[walk_target_idx]
	if mesh == null:
		return
	if mesh.get_surface_override_material(0) != _target_pyramid_mat:
		# beacon_visible=false leaves the target its DEFAULT material: the landmark is absent
		# from the world, so the camera has nothing to see and beacon == 0. The A/B control.
		if beacon_visible:
			mesh.set_surface_override_material(0, _target_pyramid_mat)
		_rebuild_beacon_surface_map()   # the beacon is defined by COLOUR, so recolouring redefines it

func _select_across_target_idx() -> int:
	## Routing policy for the next nav target.  Until the robot has obstacle
	## avoidance (or a gait robust enough to climb pyramids), pick the next
	## target roughly ACROSS the arena from the current one so the straight-line
	## path crosses the open donut centre instead of threading the dense field —
	## minimising the chance another pyramid occludes/blocks the heading.
	## Falls back to uniform random for the first target (no current heading).
	var n: int = _pyramid_xz_positions.size()
	if walk_target_idx < 0 or n <= 1:
		return _walk_target_rng.randi_range(0, n - 1)
	# Direction of the current target from arena centre (origin in xz).
	var cur: Vector2 = _pyramid_xz_positions[walk_target_idx]
	var cur_dir: Vector2 = cur.normalized() if cur.length() > 1e-3 else Vector2(1, 0)
	var antipode: Vector2 = -cur_dir
	# Score every other pyramid by alignment with the antipodal direction;
	# keep those in the opposite hemisphere (alignment > 0) as candidates.
	var candidates: Array[int] = []
	var best_idx: int = -1
	var best_align: float = -2.0
	for i in range(n):
		if i == walk_target_idx:
			continue
		var d: Vector2 = _pyramid_xz_positions[i]
		var dir: Vector2 = d.normalized() if d.length() > 1e-3 else Vector2(1, 0)
		var align: float = dir.dot(antipode)
		if align > best_align:
			best_align = align
			best_idx = i
		if align > 0.0:
			candidates.append(i)
	# Pick randomly among the opposite-hemisphere candidates for variety; if
	# none qualifies (degenerate layout) take the single best-aligned pyramid.
	if candidates.is_empty():
		return best_idx if best_idx >= 0 else (walk_target_idx + 1) % n
	return candidates[_walk_target_rng.randi_range(0, candidates.size() - 1)]

func _select_random_pyramid_target() -> void:
	## Picks a random pyramid (uniform over the placed set), restores
	## the previous target's color if any, sets walk_target_idx +
	## walk_target_pos, and recolors the new target's mesh purple.
	if _pyramid_xz_positions.is_empty():
		push_warning("PicrawlerBody: walk_over_there requested but no pyramids placed")
		return
	if _target_pyramid_mat == null:
		_target_pyramid_mat = StandardMaterial3D.new()
		_target_pyramid_mat.albedo_color = _TARGET_PYRAMID_COLOR
		_target_pyramid_mat.roughness = 0.6
		_target_pyramid_mat.emission_enabled = true
		_target_pyramid_mat.emission = _TARGET_PYRAMID_COLOR * 0.4
		_target_pyramid_mat.emission_energy_multiplier = 1.0
	# Restore previous target's default material so it stops glowing.
	if walk_target_idx >= 0 and walk_target_idx < _pyramid_meshes.size():
		var prev_mesh: MeshInstance3D = _pyramid_meshes[walk_target_idx]
		prev_mesh.set_surface_override_material(0, _pyramid_default_mats[walk_target_idx])
	var new_idx: int = _select_across_target_idx()
	walk_target_idx = new_idx
	walk_target_pos = _pyramid_xz_positions[walk_target_idx]
	# Phase 7.12 — reset progress-PB on target rotation.
	_min_dist_to_target_pb = 1e9
	var new_mesh: MeshInstance3D = _pyramid_meshes[walk_target_idx]
	# beacon_visible=false leaves the target its DEFAULT material: the landmark is absent
	# from the world, so the camera has nothing to see and beacon == 0. The A/B control.
	if beacon_visible:
		new_mesh.set_surface_override_material(0, _target_pyramid_mat)
	_rebuild_beacon_surface_map()   # recolouring redefines what IS a beacon
	print("PicrawlerBody: walk_over_there target → pyramid #%d at xz=(%.2f, %.2f)" % [
		walk_target_idx, walk_target_pos.x, walk_target_pos.y])

func _compute_ground_clearance() -> float:
	# Markov-compliant downward rangefinder (a belly-mounted ToF / ultrasonic).
	# Casts along the chassis's OWN down axis (body-relative, like a real sensor on
	# the tilting belly) from the belly surface to the nearest WORLD surface, and
	# returns the distance in metres (GROUND_CLEARANCE_RANGE if nothing is in range).
	# Masks _LAYER_WORLD only, so it passes THROUGH the robot's own legs (_LAYER_BODY)
	# and reads true ground clearance.  Egocentric — no absolute world coordinate.
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	if space_state == null or _chassis == null:
		return GROUND_CLEARANCE_RANGE
	var down: Vector3 = -_chassis.global_transform.basis.y          # body-down
	# Cast from the chassis CENTRE (above the belly), not the belly itself: if the
	# belly rests ON a ramp/wall the belly-origin ray starts on the surface and
	# intersect_ray misses it (hit_from_inside defaults false) → it read max range
	# and the reflex thought "up high" → stuck on the belly (the observed bug).
	# Origin: chassis centre raised a further 2 cm along body-up, so the ray always
	# starts with clear space above whatever the belly rests on (per the operator's
	# note) and fires down THROUGH the chassis interior — it can never clip into the
	# obstacle.  belly is CHASSIS_Y/2 below centre → total sensor-to-belly = that + 2cm.
	var sensor_up: float = CHASSIS_Y * 0.5 + 0.02
	var origin: Vector3 = _chassis.global_transform.origin + (-down) * 0.02
	var query := PhysicsRayQueryParameters3D.new()
	query.from = origin
	query.to = origin + down * (GROUND_CLEARANCE_RANGE + CHASSIS_Y)
	query.collision_mask = _LAYER_WORLD
	query.hit_from_inside = true   # detect the surface even if the origin is inside/on a collider
	var hit := space_state.intersect_ray(query)
	if hit.is_empty():
		return GROUND_CLEARANCE_RANGE
	# Sensor-to-surface distance minus the sensor-to-belly offset = belly clearance.
	return max(0.0, origin.distance_to(hit.position) - sensor_up)

func _compute_target_loom() -> float:
	# Phase H1 V6 — proxy looming: count rays in a forward FOV grid that
	# hit the active target pyramid.  Returns a single float ∈ [0, 1].
	# Biological analog: locust LGMD integrates looming-edge motion over
	# visual field → single output (firing rate).  We use a simpler
	# downsampled raycast count, but the integrand is the same shape
	# (more rays hit → target subtends more of FOV → arrival cue).
	#
	# Cheap: 64 rays × ~10 Hz = ~640 rays/sec (vs Cell's 1024/frame).
	# Subsamples via LOOM_RECOMPUTE_EVERY tick gate at the caller.
	if walk_target_idx < 0 or walk_target_idx >= _pyramid_meshes.size():
		return 0.0
	var target_mesh: MeshInstance3D = _pyramid_meshes[walk_target_idx]
	if target_mesh == null:
		return 0.0
	var target_collider: Node = target_mesh.get_parent()
	if target_collider == null:
		return 0.0
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	if space_state == null:
		return 0.0
	var chassis_xf: Transform3D = _chassis.global_transform
	# Ray origin: slightly above chassis to avoid hitting own body / legs.
	var origin: Vector3 = chassis_xf.origin + Vector3(0.0, 0.08, 0.0)
	# 2026-06-13 — body forward is +basis.z (matches target_compass: tc_y peaks
	# along (sin yaw, cos yaw) = +basis.z).  The loom previously used −basis.z and
	# cast over the REAR (confirmed by the ray overlay vs the chassis "eyes").
	var forward: Vector3 = chassis_xf.basis.z
	var right: Vector3   = -chassis_xf.basis.x   # = forward × up; +basis.x mirrors L/R when forward=+z
	# With 180° horizontal FOV we sweep the front HEMISPHERE (forward to
	# +right to forward to -right).  Use spherical sweep via per-ray angle.
	var tan_v: float = tan(LOOM_FOV_V_RAD * 0.5)
	var query := PhysicsRayQueryParameters3D.new()
	query.collision_mask = 0xFFFFFFFF
	query.collide_with_areas = false
	query.collide_with_bodies = true
	var hits: int = 0
	for j in range(LOOM_RES_V):
		# Vertical: linear in tan(angle) over ±45°.
		var v: float = -1.0 + 2.0 * (float(j) + 0.5) / float(LOOM_RES_V)
		for i in range(LOOM_RES_H):
			# Horizontal: linear in ANGLE over ±90° (panoramic).
			var ang_h: float = -LOOM_FOV_H_RAD * 0.5 \
							 + LOOM_FOV_H_RAD * (float(i) + 0.5) / float(LOOM_RES_H)
			# Direction = rotate forward by ang_h around up axis, then tilt by v*tan_v.
			var dir_h: Vector3 = forward * cos(ang_h) + right * sin(ang_h)
			var dir: Vector3 = (dir_h + chassis_xf.basis.y * (v * tan_v)).normalized()
			query.from = origin
			query.to   = origin + dir * LOOM_RAY_LEN
			var hit := space_state.intersect_ray(query)
			if not hit.is_empty() and hit.collider == target_collider:
				hits += 1
	return float(hits) / float(LOOM_RES_H * LOOM_RES_V)

func _clear_pyramid_target() -> void:
	if walk_target_idx >= 0 and walk_target_idx < _pyramid_meshes.size():
		var prev_mesh: MeshInstance3D = _pyramid_meshes[walk_target_idx]
		prev_mesh.set_surface_override_material(0, _pyramid_default_mats[walk_target_idx])
		_rebuild_beacon_surface_map()   # recolouring redefines what IS a beacon
	walk_target_idx = -1

func _update_ray_overlay() -> void:
	# 2026-06-13 — debug overlay: draw the loom/vision FOV rays as 3D lines from
	# the SAME origin + geometry the loom uses, coloured by what each ray hits.
	# Purpose: verify the vision ray origin clears the chassis before V1 wiring —
	# RED segments (hit nearer than the chassis half-extent) = the origin is INSIDE
	# the body and perception would be self-occluded.  Toggled by KEY_V; refreshed
	# at the loom cadence.  Off by default (extra raycasts only when inspecting).
	if _chassis == null:
		return
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	if space_state == null:
		return
	if _ray_overlay_mi == null:
		_ray_overlay_mat = StandardMaterial3D.new()
		_ray_overlay_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		_ray_overlay_mat.vertex_color_use_as_albedo = true
		_ray_overlay_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		_ray_overlay_mi = MeshInstance3D.new()
		_ray_overlay_mi.name = "RayOverlay"
		_ray_overlay_mi.top_level = true   # vertices are WORLD-space; ignore body transform
		_ray_overlay_mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(_ray_overlay_mi)
	_ray_overlay_mi.visible = true
	var chassis_xf: Transform3D = _chassis.global_transform
	var origin: Vector3  = chassis_xf.origin + Vector3(0.0, 0.08, 0.0)   # MATCHES _compute_target_loom
	var forward: Vector3 = chassis_xf.basis.z   # body forward = +basis.z (see loom fix)
	var right: Vector3   = -chassis_xf.basis.x  # = forward × up (un-mirror L/R)
	var tan_v: float = tan(LOOM_FOV_V_RAD * 0.5)
	var target_collider: Node = null
	if walk_target_idx >= 0 and walk_target_idx < _pyramid_meshes.size():
		var tm: MeshInstance3D = _pyramid_meshes[walk_target_idx]
		if tm != null:
			target_collider = tm.get_parent()
	var query := PhysicsRayQueryParameters3D.new()
	query.collision_mask = 0xFFFFFFFF
	query.collide_with_areas = false
	query.collide_with_bodies = true
	var c_self   := Color(1.0, 0.15, 0.15, 0.95)   # RED  — origin inside/too close to body
	var c_target := Color(0.75, 0.35, 1.0, 0.95)   # purple — hit the active target
	var c_other  := Color(1.0, 0.6, 0.1, 0.5)      # orange — hit something else
	var c_miss   := Color(0.3, 0.85, 0.9, 0.16)    # faint cyan — no hit
	var im := ImmediateMesh.new()
	im.surface_begin(Mesh.PRIMITIVE_LINES, _ray_overlay_mat)
	for j in range(LOOM_RES_V):
		var v: float = -1.0 + 2.0 * (float(j) + 0.5) / float(LOOM_RES_V)
		for i in range(LOOM_RES_H):
			var ang_h: float = -LOOM_FOV_H_RAD * 0.5 \
							 + LOOM_FOV_H_RAD * (float(i) + 0.5) / float(LOOM_RES_H)
			var dir_h: Vector3 = forward * cos(ang_h) + right * sin(ang_h)
			var dir: Vector3 = (dir_h + chassis_xf.basis.y * (v * tan_v)).normalized()
			query.from = origin
			query.to   = origin + dir * LOOM_RAY_LEN
			var hit := space_state.intersect_ray(query)
			var endp: Vector3
			var col: Color
			if hit.is_empty():
				endp = origin + dir * LOOM_RAY_LEN
				col  = c_miss
			else:
				endp = hit.position
				if origin.distance_to(endp) < RAY_OVERLAY_SELF_OCCLUSION_M:
					col = c_self
				elif target_collider != null and hit.collider == target_collider:
					col = c_target
				else:
					col = c_other
			im.surface_set_color(col); im.surface_add_vertex(origin)
			im.surface_set_color(col); im.surface_add_vertex(endp)
	im.surface_end()
	_ray_overlay_mi.mesh = im

# ---- 2026-08-04 · beacon-surface lookup (the honest replacement for the identity test) --
# Keyed on a collider's OWN albedo, not on which pyramid is the target.  Built once and
# refreshed whenever the purple material moves, so the per-ray cost is a dictionary hit
# rather than a material walk across ~3000 rays a frame.
#
# WHY THIS IS NOT JUST THE IDENTITY TEST WEARING A HAT: the map is colour -> bool, so it
# answers "is the surface at this point purple", which a camera can do.  Paint two pyramids
# and both register; paint none and none do; move the beacon and the purple moves with it.
# The old test answered "is this the node the simulator designated as the goal", which no
# sensor can answer and which no amount of learning would need to.
var _beacon_colliders: Dictionary = {}      # collider instance_id -> true
var _beacon_frac: float = 0.0               # fraction of the frame that is beacon-coloured

func _rebuild_beacon_surface_map() -> void:
	_beacon_colliders.clear()
	for i in range(_pyramid_meshes.size()):
		var mesh: MeshInstance3D = _pyramid_meshes[i]
		if mesh == null or not is_instance_valid(mesh):
			continue
		var mat: Material = mesh.get_surface_override_material(0)
		if mat is StandardMaterial3D:
			var c: Color = (mat as StandardMaterial3D).albedo_color
			# "Beacon-coloured" = close to the beacon hue in RGB.  A real detector would
			# threshold in HSV; RGB distance is adequate for a flat-shaded scene and keeps
			# this cheap.  It is a property of the SURFACE, never of the target index.
			if c.r > 0.35 and c.b > 0.45 and c.g < 0.40 and c.b > c.g * 1.4:
				var body: Node = mesh.get_parent()
				if body != null:
					_beacon_colliders[body.get_instance_id()] = true

func _abl_is_beacon_surface(collider: Object) -> bool:
	if collider == null or _beacon_colliders.is_empty():
		return false
	return _beacon_colliders.has(collider.get_instance_id())

func _capture_vision() -> void:
	# One forward-facing perspective raycast grid → BOTH an RGB image (camera)
	# and a depth/range field (LiDAR/sonar).  Forward = +basis.z (the fixed
	# convention); origin = the loom eye (chassis + 0.08 up, confirmed clear of
	# the chassis by the ray overlay).  Colours: purple=active target, gray=ground
	# (normal up), tan=object side, dark=sky/miss.  Depth: near=bright, far=dark.
	if _chassis == null:
		return
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	if space_state == null:
		return
	var nw: int = maxi(2, vision_res_w)
	var nh: int = maxi(2, vision_res_h)
	if _last_vision_pixels.size() != nw * nh * 3:
		_last_vision_pixels.resize(nw * nh * 3)
	if _last_vision_depth.size() != nw * nh:
		_last_vision_depth.resize(nw * nh)
	var chassis_xf: Transform3D = _chassis.global_transform
	var origin: Vector3  = chassis_xf.origin + Vector3(0.0, 0.08, 0.0)
	var forward: Vector3
	var right: Vector3
	var up: Vector3
	if vision_stabilized:
		# Gravity-levelled, yaw-only gaze (gimbal) — removes the gait rock.
		var yaw_s: float = chassis_xf.basis.get_euler().y
		forward = Vector3(sin(yaw_s), 0.0, cos(yaw_s))    # body heading, horizontal (= +basis.z yaw component)
		up      = Vector3(0.0, 1.0, 0.0)                  # world up (level horizon)
		right   = Vector3(-cos(yaw_s), 0.0, sin(yaw_s))   # forward × up
	else:
		forward = chassis_xf.basis.z
		right   = -chassis_xf.basis.x   # = forward × up; un-mirror L/R
		up      = chassis_xf.basis.y
	# Separate H/V half-angles — the sensor is 4:3, not square.  Using one FOV for both
	# axes (the old behaviour) silently stretches the image and misreports every bearing.
	var tan_half_h: float = tan(VISION_FOV_H_RAD * 0.5)
	var tan_half_v: float = tan(VISION_FOV_V_RAD * 0.5)
	# ⚠ NO target_collider here any more — see the colour test below.
	var q := PhysicsRayQueryParameters3D.new()
	q.collision_mask = 0xFFFFFFFF
	q.collide_with_areas = false
	q.collide_with_bodies = true
	var beacon_hits: int = 0
	for j in range(nh):
		var v: float = 1.0 - 2.0 * (float(j) + 0.5) / float(nh)   # top row first
		for i in range(nw):
			var u: float = -1.0 + 2.0 * (float(i) + 0.5) / float(nw)
			var dir: Vector3 = (forward + right * (u * tan_half_h) + up * (v * tan_half_v)).normalized()
			q.from = origin
			q.to   = origin + dir * LOOM_RAY_LEN
			var hit := space_state.intersect_ray(q)
			var r8: int = 18; var g8: int = 18; var b8: int = 41   # sky/miss
			var depth_norm: float = 1.0                            # miss = far
			if not hit.is_empty():
				depth_norm = clamp(origin.distance_to(hit.position) / LOOM_RAY_LEN, 0.0, 1.0)
				var nrm: Vector3 = hit.get("normal", Vector3.UP)
				var br: int; var bg: int; var bb: int          # base class colour
				# ---- 2026-08-04 · THE HONEST COLOUR TEST -------------------------------
				# WAS: `if hit.collider == target_collider` -- a NODE-IDENTITY comparison
				# against _pyramid_meshes[walk_target_idx].  That is an oracle painted into
				# the raw pixels: the ground-truth answer to "which pyramid is the target"
				# arrived upstream of the encoder, upstream of everything.  No camera can
				# perform an identity test.  It also meant the scene-graph purple material
				# was never read -- the purple was a HUD affordance for the human only.
				# NOW: look up the surface's OWN colour.  The pyramid reads purple BECAUSE
				# IT IS PURPLE, which any camera can do, and it degrades honestly -- paint
				# two pyramids and both show; move the beacon and the purple moves with it.
				var beacon_px: bool = _abl_is_beacon_surface(hit.collider)
				if beacon_px:
					br = 191; bg = 89; bb = 255          # purple — a purple SURFACE
					beacon_hits += 1
				elif nrm.y > 0.7:
					br = 120; bg = 115; bb = 110         # gray — ground
				else:
					br = 200; bg = 150; bb = 90          # tan — object/pyramid side
				# Lambert shading from a fixed key light → facet/edge/shape cues
				# (flat per-class colour gave the EPM no notion of geometry).
				var lambert: float = maxf(0.0, nrm.dot(VISION_LIGHT_DIR))
				var shade: float = VISION_AMBIENT + (1.0 - VISION_AMBIENT) * lambert
				r8 = int(clampf(br * shade, 0.0, 255.0))
				g8 = int(clampf(bg * shade, 0.0, 255.0))
				b8 = int(clampf(bb * shade, 0.0, 255.0))
			var pidx: int = (j * nw + i) * 3
			_last_vision_pixels[pidx]     = r8
			_last_vision_pixels[pidx + 1] = g8
			_last_vision_pixels[pidx + 2] = b8
			_last_vision_depth[j * nw + i] = depth_norm
	# ---- THE BEACON SCALAR -- computed HERE, at full ray resolution ------------------
	# Deliberately NOT routed through the EPM.  A GNG coarse-grains into a discrete,
	# addressable vocabulary, which is exactly right for "what kind of place is this" and
	# exactly wrong for a beacon gradient: run-and-tumble climbs a CONTINUOUS graded
	# magnitude, and node-quantising it destroys the signal it climbs.  This is the
	# looming/LGMD pathway; `epm_color` remains the form pathway on the same image.
	# Also note it is computed BEFORE the JL encoder's fixed 24x24 resize, so raising
	# vision_res_* actually buys the nav loop range (it would not if this went via the EPM).
	_beacon_frac = float(beacon_hits) / float(nw * nh)

func _load_vision_steer_readout() -> void:
	# Load the fixed linear readout (epm_color latent → bearing) for V2 steering.
	var f := FileAccess.open(VISION_STEER_READOUT_PATH, FileAccess.READ)
	if f == null:
		push_warning("PicrawlerBody: vision_steer ON but readout missing: " + VISION_STEER_READOUT_PATH)
		return
	var d = JSON.parse_string(f.get_as_text())
	f.close()
	if typeof(d) != TYPE_DICTIONARY or not d.has("w_x") or not d.has("w_y"):
		push_warning("PicrawlerBody: vision_steer readout malformed")
		return
	_vsteer_wx = PackedFloat64Array(d["w_x"])
	_vsteer_wy = PackedFloat64Array(d["w_y"])
	_vsteer_bx = float(d.get("b_x", 0.0))
	_vsteer_by = float(d.get("b_y", 0.0))
	_vsteer_loaded = (_vsteer_wx.size() == 64 and _vsteer_wy.size() == 64)
	print("PicrawlerBody: vision_steer readout loaded (r2_tcx=%s bearing_r=%s)" % [
		str(d.get("r2_tcx", "?")), str(d.get("bearing_r", "?"))])

func _build_vision_panel() -> void:
	var hud := get_tree().get_root().find_child("HUD", true, false)
	if hud == null:
		return
	var px: int = 168
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 3)
	var t1 := Label.new()
	t1.text = "camera (RGB) — what the EPM sees"
	t1.add_theme_font_size_override("font_size", 10)
	t1.add_theme_color_override("font_color", Color(0.85, 0.92, 1.0, 1))
	_vision_rgb_rect = TextureRect.new()
	_vision_rgb_rect.custom_minimum_size = Vector2(px, px)
	_vision_rgb_rect.expand_mode  = TextureRect.EXPAND_IGNORE_SIZE
	_vision_rgb_rect.stretch_mode = TextureRect.STRETCH_SCALE
	_vision_rgb_rect.texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST
	var t2 := Label.new()
	t2.text = "LiDAR / depth (range) — sim2real"
	t2.add_theme_font_size_override("font_size", 10)
	t2.add_theme_color_override("font_color", Color(0.85, 1.0, 0.9, 1))
	_vision_depth_rect = TextureRect.new()
	_vision_depth_rect.custom_minimum_size = Vector2(px, px)
	_vision_depth_rect.expand_mode  = TextureRect.EXPAND_IGNORE_SIZE
	_vision_depth_rect.stretch_mode = TextureRect.STRETCH_SCALE
	_vision_depth_rect.texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST
	vb.add_child(t1); vb.add_child(_vision_rgb_rect)
	vb.add_child(t2); vb.add_child(_vision_depth_rect)
	_vision_panel = PanelContainer.new()
	_vision_panel.name = "VisionPanel"
	_vision_panel.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	_vision_panel.position = Vector2(-(px + 28), 210)
	_vision_panel.add_child(vb)
	_vision_panel.visible = false
	hud.add_child(_vision_panel)
	_vision_rgb_image = Image.create_empty(vision_res_w, vision_res_h, false, Image.FORMAT_RGB8)
	_vision_rgb_texture = ImageTexture.create_from_image(_vision_rgb_image)
	_vision_rgb_rect.texture = _vision_rgb_texture
	_vision_depth_image = Image.create_empty(vision_res_w, vision_res_h, false, Image.FORMAT_RGB8)
	_vision_depth_texture = ImageTexture.create_from_image(_vision_depth_image)
	_vision_depth_rect.texture = _vision_depth_texture

func _update_vision_panel() -> void:
	# Shown when the vision-panel toggle (M) is on; independent of the camera.
	if _vision_panel == null:
		_build_vision_panel()
		if _vision_panel == null:
			return
	_vision_panel.visible = _vision_panel_on
	if not _vision_panel_on:
		return
	_capture_vision()
	var nw: int = vision_res_w
	var nh: int = vision_res_h
	if _last_vision_pixels.size() == nw * nh * 3:
		_vision_rgb_image.set_data(nw, nh, false, Image.FORMAT_RGB8, _last_vision_pixels)
		_vision_rgb_texture.update(_vision_rgb_image)
	# Depth → grayscale (near bright, far dark) for the panel.
	if _last_vision_depth.size() == nw * nh:
		var gray := PackedByteArray()
		gray.resize(nw * nh * 3)
		for k in range(nw * nh):
			var g: int = int((1.0 - _last_vision_depth[k]) * 255.0)
			gray[k * 3] = g; gray[k * 3 + 1] = g; gray[k * 3 + 2] = g
		_vision_depth_image.set_data(nw, nh, false, Image.FORMAT_RGB8, gray)
		_vision_depth_texture.update(_vision_depth_image)

func _update_distress_hud() -> void:
	# Always-visible distress bar (Cell-style) so the wedge state is watchable live.
	if _distress_hud == null:
		var hud := get_tree().get_root().find_child("HUD", true, false)
		if hud == null:
			return
		_distress_hud = Label.new()
		_distress_hud.name = "DistressBar"
		_distress_hud.add_theme_font_size_override("font_size", 13)
		_distress_hud.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
		_distress_hud.position = Vector2(12, -28)
		hud.add_child(_distress_hud)
	var n: int = 12
	var filled: int = clampi(int(round(_distress * float(n))), 0, n)
	var bar: String = "█".repeat(filled) + "░".repeat(n - filled)
	_distress_hud.text = "distress [%s] %.2f  (deficit %.2f)" % [bar, _distress, _stuck_deficit]
	_distress_hud.add_theme_color_override("font_color",
		Color(0.4 + 0.6 * _distress, 0.9 - 0.7 * _distress, 0.3, 1.0))

# Compute a safe XZ spawn point near `target` for in-place auto-reset.
# Iteratively pushes the spawn point outward from any pyramid the
# chassis bounding circle would intrude on, and clamps inside the
# outer-wall safe zone.  Returns Vector2.INF when no safe spot is
# reachable from `target` (caller falls back to center reset).
func _safe_reset_xz_near(target: Vector2) -> Vector2:
	var safe: Vector2 = target
	var max_r: float = AUTO_RESET_OUTER_WALL_RADIUS - 0.2
	for _iter in range(8):
		var moved: bool = false
		for i in range(_pyramid_xz_positions.size()):
			var p: Vector2  = _pyramid_xz_positions[i]
			var pr: float   = _pyramid_xz_radii[i]
			var min_d: float = pr + RESET_CHASSIS_MARGIN
			var d: float = safe.distance_to(p)
			if d < min_d:
				var dir: Vector2 = (safe - p)
				if dir.length_squared() < 1e-6:
					dir = Vector2(1.0, 0.0)
				safe = p + dir.normalized() * min_d
				moved = true
		# Clamp to outer-wall safe zone every iteration so a pyramid push
		# that lands outside the ring then gets pulled back inward.
		var r: float = safe.length()
		if r > max_r:
			safe = safe.normalized() * max_r
			moved = true
		if not moved:
			break
	# Final collision check — if still inside any pyramid, give up.
	for i in range(_pyramid_xz_positions.size()):
		var p: Vector2  = _pyramid_xz_positions[i]
		var pr: float   = _pyramid_xz_radii[i]
		if safe.distance_to(p) < pr + RESET_CHASSIS_MARGIN:
			return Vector2(INF, INF)
	return safe

func _do_hard_reset() -> void:
	# Gate 0 (L-1a) — announce the teleport/respawn on the bus so brain modules
	# (MotorEPM) can reset-mask their metrics.  A hard reset is otherwise invisible
	# to the brain (no events.* fires), which fakes rhythm continuity across the
	# discontinuity.  Reward-free instrumentation.
	if brain != null:
		brain.publish_event("reset", 1.0)
	# Apply _pending_reset_offset to every cached rest transform.  Zero
	# offset (legacy default) reproduces the original center-spawn
	# behaviour bit-identically.  Non-zero offset translates the entire
	# body in XZ — used by the auto-reset-in-place-outside-the-ring path.
	var offset: Vector3 = _pending_reset_offset
	_pending_reset_offset = Vector3.ZERO   # one-shot; clear immediately
	_chassis.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	_chassis.freeze = true
	for b in _coxas + _uppers + _lowers:
		b.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
		b.freeze = true
	_chassis.global_transform = Transform3D(
		_chassis_rest_xform.basis,
		_chassis_rest_xform.origin + offset)
	_chassis.linear_velocity  = Vector3.ZERO
	_chassis.angular_velocity = Vector3.ZERO
	for i in range(4):
		_coxas[i].global_transform  = Transform3D(_coxa_rest_xform[i].basis,
												  _coxa_rest_xform[i].origin + offset)
		_uppers[i].global_transform = Transform3D(_upper_rest_xform[i].basis,
												  _upper_rest_xform[i].origin + offset)
		_lowers[i].global_transform = Transform3D(_lower_rest_xform[i].basis,
												  _lower_rest_xform[i].origin + offset)
		for b in [_coxas[i], _uppers[i], _lowers[i]]:
			b.linear_velocity  = Vector3.ZERO
			b.angular_velocity = Vector3.ZERO
	_chassis.freeze = false
	for b in _coxas + _uppers + _lowers:
		b.freeze = false
	# Explicitly stop every hinge motor before the post-reset physics
	# frame runs.  Without this, PARAM_MOTOR_TARGET_VELOCITY from the
	# last pre-reset tick (e.g. -6 rad/s if the brain was driving the
	# knee toward standing) stays active, and the solver applies that
	# impulse to the just-teleported body during the SAME physics frame
	# — joints leave construction pose immediately, then later ticks
	# compound the error into the observed "propeller" behavior.
	for i in range(4):
		for jt in [_hip1_joints[i], _hip2_joints[i], _knee_joints[i]]:
			_set_motor_vf(jt, 0.0, 0.0)
	# Reset slew-lag torque history AND effective-target state.  Without
	# this, the freshly-teleported body's first physics tick gets
	# whatever residual torque/target the previous step left behind.
	for i in range(4):
		_prev_torque_hip1[i] = 0.0
		_prev_torque_hip2[i] = 0.0
		_prev_torque_knee[i] = 0.0
		# Match _eff_target to the CONSTRUCTION joint angles (all 0 because
		# adjacent bodies are at IDENTITY basis at construction), NOT to
		# the standing-pose targets.  Otherwise the freshly-reset knee has
		# _eff_target=-1.6 but joint_angle=0, the motor sees 1.6 rad error,
		# saturates velocity, and the joint overshoots its limit — observed
		# as the knee free-spinning ("propeller") after SPACE reset.  The
		# slew limiter in the motor section will ramp these targets toward
		# the brain's commanded values at MAX_SERVO_SPEED·TAU per tick,
		# matching the smooth ramp seen when R-press lifts ragdoll.
		_eff_target_hip1[i] = 0.0
		_eff_target_hip2[i] = 0.0
		_eff_target_knee[i] = 0.0
	# Wipe accumulated walking-trail X-markers + dashed lines so the
	# next attempt starts on a clean floor.  Method-presence check
	# keeps this safe in scenes without a WalkingTrail node.
	if _walking_trail != null and _walking_trail.has_method("clear"):
		_walking_trail.call("clear")

# Sum each foot's contact impulses along the body-forward axis, every physics step.
# Drained by the trace recorder, so the logged value is the impulse delivered over the
# whole brain tick rather than a single-substep snapshot.
func _fl_norm() -> float:
	return max(1e-6, _TOTAL_MASS * 9.81 / float(physics_hz))

func _accum_grf() -> void:
	if _trace_file == null and not _trace_ready:
		pass    # still accumulate before the trace opens; cost is 4 direct-state reads
	if _chassis == null or not is_instance_valid(_chassis):
		return
	var yaw: float = _chassis.global_transform.basis.get_euler().y
	var fwd := Vector3(sin(yaw), 0.0, cos(yaw))
	var up_acc: Array[float] = [0.0, 0.0, 0.0, 0.0]
	var nrm_acc: Array[float] = [0.0, 0.0, 0.0, 0.0]
	for i in range(_lowers.size()):
		if not is_instance_valid(_lowers[i]):
			continue
		var st := PhysicsServer3D.body_get_direct_state(_lowers[i].get_rid())
		if st == null:
			continue
		for j in range(st.get_contact_count()):
			# get_contact_impulse is the impulse the SOLVER applied to this body this
			# step; its forward component is the thrust the ground gave the foot.
			var imp: Vector3 = st.get_contact_impulse(j)
			_grf_fwd[i] += imp.dot(fwd)
			up_acc[i]   += imp.dot(Vector3.UP)
			# HARDWARE-HONEST VARIANT — what an FSR / load cell in the foot actually
			# reads: the NORMAL force at the contact patch.  No world-frame projection,
			# so it needs neither a god's-eye up-vector nor the IMU's estimate of one.
			# This is the version a real picrawler can produce; the world-up projection
			# above is kept only to check the two agree.
			nrm_acc[i]  += absf(imp.dot(st.get_contact_local_normal(j)))
	for i in range(4):
		_grf_up[i]  += up_acc[i]
		_grf_nrm[i] += nrm_acc[i]
		_foot_load_ema[i] = (1.0 - _FOOT_LOAD_ALPHA) * _foot_load_ema[i] + _FOOT_LOAD_ALPHA * nrm_acc[i]

# ---------------------------------------------------------------------------
# Attribution trace — one JSON line per brain tick when OGMA_PICRAWLER_TRACE is set.
# ---------------------------------------------------------------------------
func _trace_record(h1: Array, h2: Array, kn: Array, contact: Array, fwd_v: float, yaw: float, chassis_y: float) -> void:
	if not _trace_ready:
		_trace_ready = true
		var path: String = OS.get_environment("OGMA_PICRAWLER_TRACE")
		if path != "":
			_trace_file = FileAccess.open(path, FileAccess.WRITE)
			if _trace_file == null:
				push_error("PicrawlerBody: could not open trace path " + path)
			else:
				print("PicrawlerBody: attribution trace -> %s" % path)
	if _trace_file == null:
		return
	# Torques are ALREADY normalised to [-1,1] against MAX_SERVO_TORQUE by the
	# joint_torque proprio block, and are ONE TICK DELAYED (the motor runs after
	# perception).  Both facts matter to the offline analysis, so the raw values
	# go out unmodified and the correction is made there, not hidden here.
	var rec := {
		"t": tick_counter,
		"fwd_v": snappedf(fwd_v, 0.00001),
		"yaw": snappedf(yaw, 0.00001),
		# 2026-08-06 — chassis height and belly clearance.  ⚠ WITH chassis_collides=false
		# (the default) the chassis is a GHOST: it passes THROUGH the floor rather than
		# resting on it, so a NEGATIVE-going y is the body intersecting the world, not
		# touching it.  Every claim about belly clearance is meaningless without this.
		"cy": snappedf(chassis_y, 0.00001),
		"gc": snappedf(_dbg_gc_raw, 0.00001),
		"c": contact,
		"h1": h1, "h2": h2, "kn": kn,
		"th1": [_prev_torque_hip1[0], _prev_torque_hip1[1], _prev_torque_hip1[2], _prev_torque_hip1[3]],
		"th2": [_prev_torque_hip2[0], _prev_torque_hip2[1], _prev_torque_hip2[2], _prev_torque_hip2[3]],
		"tkn": [_prev_torque_knee[0], _prev_torque_knee[1], _prev_torque_knee[2], _prev_torque_knee[3]],
		"a1": [_eff_target_hip1[0], _eff_target_hip1[1], _eff_target_hip1[2], _eff_target_hip1[3]],
		"a2": [_eff_target_hip2[0], _eff_target_hip2[1], _eff_target_hip2[2], _eff_target_hip2[3]],
		"akn": [_eff_target_knee[0], _eff_target_knee[1], _eff_target_knee[2], _eff_target_knee[3]],
		"lh1": [_prev_load_hip1[0], _prev_load_hip1[1], _prev_load_hip1[2], _prev_load_hip1[3]],
		"lh2": [_prev_load_hip2[0], _prev_load_hip2[1], _prev_load_hip2[2], _prev_load_hip2[3]],
		"lkn": [_prev_load_knee[0], _prev_load_knee[1], _prev_load_knee[2], _prev_load_knee[3]],
		"grf": [_grf_fwd[0], _grf_fwd[1], _grf_fwd[2], _grf_fwd[3]],
		"grfup": [_grf_up[0], _grf_up[1], _grf_up[2], _grf_up[3]],
		"grfn": [_grf_nrm[0], _grf_nrm[1], _grf_nrm[2], _grf_nrm[3]],
		# the PUBLISHED foot_load channel, exactly as a consumer EPM would receive it
		"fload": [snappedf(_foot_load_ema[0] / _fl_norm(), 0.00001),
				  snappedf(_foot_load_ema[1] / _fl_norm(), 0.00001),
				  snappedf(_foot_load_ema[2] / _fl_norm(), 0.00001),
				  snappedf(_foot_load_ema[3] / _fl_norm(), 0.00001)],
		# 2026-08-07 — THE ACCELEROMETER RESIDUAL.  raw accel-up minus the fused estimate,
		# in BODY frame.  The complementary filter treats this as the thing to reject (and
		# is right to: a footfall impulse reads as a tilt to an accelerometer, so feeding
		# raw accel into attitude would make the body think it pitches on every step).
		# But the rejected part is not noise — the operator observes it scattering in four
		# diagonal directions, i.e. each leg's strike has a characteristic direction.  If
		# that holds, the IMU is already a footfall/load sensor, on hardware the real
		# picrawler HAS, and no FSR is needed.  Logged to test exactly that.
		"ares": [snappedf(_up_acc_last.x - _up_est_body.x, 0.00001),
				 snappedf(_up_acc_last.y - _up_est_body.y, 0.00001),
				 snappedf(_up_acc_last.z - _up_est_body.z, 0.00001)],
	}
	_trace_file.store_line(JSON.stringify(rec))
	# FileAccess buffers and the quit path never closes this file, so an unflushed
	# trace silently loses its TAIL — every banked trace was found truncated at
	# 1.9–4 MB of an expected ~12 MB (2026-08-09).  A truncated instrument is worse
	# than a slow one: it reads as "early-run behavior" without saying so.
	if tick_counter % 200 == 0:
		_trace_file.flush()
	for k in range(4):
		_grf_fwd[k] = 0.0    # drain: each line reports one brain tick
		_grf_up[k]  = 0.0
		_grf_nrm[k] = 0.0

# ---------------------------------------------------------------------------
# Clip ring — one compact record per brain tick.  Cheap by construction: a fixed-size
# Array of small Dictionaries, overwritten in place, never grown, never scanned.
# ---------------------------------------------------------------------------
func _clip_record(h1: Array, h2: Array, kn: Array, chassis_y: float) -> void:
	if _chassis == null or not is_instance_valid(_chassis):
		return
	if _clip_ring.size() != _CLIP_RING_LEN:
		_clip_ring.resize(_CLIP_RING_LEN)
		_clip_head = 0
		_clip_count = 0
	var contact: Array = []
	for i in range(4):
		contact.append(1 if not _lowers[i].get_colliding_bodies().is_empty() else 0)
	var o: Vector3 = _chassis.global_transform.origin
	var yaw: float = _chassis.global_transform.basis.get_euler().y
	# Same locomotor-forward projection the diag path uses (see :4515) — recomputed here
	# rather than cached, so the recorder adds no state the rest of the body must maintain.
	var fwd_v: float = Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z) \
		.dot(Vector2(sin(yaw), cos(yaw)))
	_trace_record(h1, h2, kn, contact, fwd_v, yaw, chassis_y)
	_clip_ring[_clip_head] = {
		"t": tick_counter,
		"x": snappedf(o.x, 0.0001), "y": snappedf(chassis_y, 0.0001), "z": snappedf(o.z, 0.0001),
		"yaw": snappedf(yaw, 0.0001),
		"c": contact,
		"h1": [snappedf(h1[0],0.0001), snappedf(h1[1],0.0001), snappedf(h1[2],0.0001), snappedf(h1[3],0.0001)],
		"h2": [snappedf(h2[0],0.0001), snappedf(h2[1],0.0001), snappedf(h2[2],0.0001), snappedf(h2[3],0.0001)],
		"kn": [snappedf(kn[0],0.0001), snappedf(kn[1],0.0001), snappedf(kn[2],0.0001), snappedf(kn[3],0.0001)],
		"fwd_v": snappedf(fwd_v, 0.0001),
		# 2026-08-05 — added for the BURST-ONSET probe.  The question is what gates a step:
		# the gait moves in bursts of 2-4 steps separated by ~1.6 s silences (measured
		# step-rate CV 5.6 against 1.0 for a memoryless process), and nothing in the
		# 5-tick-resolution signal set predicts onset at +-40 ticks.  These are the
		# body-side candidates, at full tick rate.
		"tilt": snappedf(_chassis_tilt(_chassis.global_transform.basis), 0.0001),
		"fy": _feet_y_array(),
		"gc": snappedf(_dbg_gc_raw, 0.0001),
		"lift": [_leg_lifted_count[0], _leg_lifted_count[1], _leg_lifted_count[2], _leg_lifted_count[3]],
		"lat_v": snappedf(Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z)
			.dot(Vector2(cos(yaw), -sin(yaw))), 0.0001),
	}
	_clip_head = (_clip_head + 1) % _CLIP_RING_LEN
	# ---- AUTO-TRIGGER (2026-08-05) ------------------------------------------------------
	# A ring buffer plus a trigger means the saved window is the RUN-UP to an event rather
	# than a guess about when to press a key.  Two classes, so clipdiff.py has both sides:
	#   ONSET — the first lift after >= _BURST_GAP_TICKS of silence.  The clip therefore
	#           contains the whole preceding pause AND the burst that ends it.
	#   GAP   — a control taken deep inside a silence, to answer "what is different about
	#           the moment stepping resumes" rather than "what does a step look like".
	# ⚠ Keyed on _leg_lifted_count, which is the SWING-DETECTOR's event, not true contact
	# (the detector over-reports swing ~1.8x vs the physics flag).  If the captured windows
	# do not look like pause->step transitions to the operator, this trigger is firing on
	# the wrong events and nothing downstream of it is trustworthy.
	if _cw_from >= 0 and not _cw_saved and tick_counter >= _cw_to:
		_cw_saved = true
		var n_out: int = 0
		for k in range(_CLIP_RING_LEN):
			var e = _clip_ring[(_clip_head + k) % _CLIP_RING_LEN]
			if e is Dictionary and int(e.get("t", -1)) >= _cw_from and int(e.get("t", -1)) <= _cw_to:
				e["cw"] = 1
				print(JSON.stringify(e)); n_out += 1
		print("PicrawlerBody: CLIP_WINDOW emitted %d ticks (%d..%d)" % [n_out, _cw_from, _cw_to])
	if _burst_probe:
		var lifts: int = 0
		for i in range(4): lifts += int(_leg_lifted_count[i])
		if lifts > _bp_last_lifts:
			var gap: int = tick_counter - _bp_last_lift_tick
			if gap >= _BURST_GAP_TICKS and _clip_count >= _CLIP_RING_LEN / 2:
				_clip_save("ONSET")
			_bp_last_lifts = lifts
			_bp_last_lift_tick = tick_counter
		elif tick_counter - _bp_last_lift_tick == _BURST_GAP_TICKS * 2 \
				and _clip_count >= _CLIP_RING_LEN / 2:
			_clip_save("GAP")
	if _clip_count < _CLIP_RING_LEN:
		_clip_count += 1


## Write the ring to /tmp/xaq_clips/<runid>/, labelled GOOD or BAD.  Oldest-first, with a
## `_meta` header line naming the config/gym/seed so clipdiff can refuse to compare clips
## that came from different ARMS (a GOOD clip from one arm vs a BAD clip from another
## measures the arm, and that confound is invisible in the output table).
func _clip_save(label: String) -> void:
	if _clip_count < 30:
		print("PicrawlerBody: clip SKIPPED — only %d ticks buffered" % _clip_count)
		return
	if _clip_dir == "":
		var stamp: String = Time.get_datetime_string_from_system().replace(":", "").replace("-", "")
		_clip_dir = "/tmp/xaq_clips/run_%s" % stamp
		DirAccess.make_dir_recursive_absolute(_clip_dir)
	var path: String = "%s/clip_%02d_%s_t%06d.jsonl" % [_clip_dir, _clip_seq, label, tick_counter]
	var f: FileAccess = FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_error("PicrawlerBody: could not open %s" % path)
		return
	f.store_line(JSON.stringify({"_meta": {
		"label": label,
		"config": OS.get_environment("OGMA_PICRAWLER_CONFIG"),
		"gym": _gym_mode_active,
		"tick": tick_counter,
		"ticks": _clip_count,
	}}))
	# Unroll oldest-first so the file reads forward in time regardless of the ring's head.
	var start: int = (_clip_head - _clip_count + 2 * _CLIP_RING_LEN) % _CLIP_RING_LEN
	for k in range(_clip_count):
		var rec = _clip_ring[(start + k) % _CLIP_RING_LEN]
		if rec != null:
			f.store_line(JSON.stringify(rec))
	f.close()
	_clip_seq += 1
	print("PicrawlerBody: [%s] clip saved -> %s  (%d ticks)" % [label, path, _clip_count])
	_ui_notify("clip %s saved (%d ticks)" % [label, _clip_count])


func _emit_jsonl(h1: Array, h2: Array, kn: Array,
				 chassis_y: float, chassis_tilt: float) -> void:
	var metrics: Dictionary = brain.get_module_metrics()
	var pre_W: Dictionary = {}
	# Per-Premotor intent-distribution entropy (Phase 6.7 observability).
	# Surfaces last_entropy from get_module_metrics — same source the
	# end-of-run snapshot prints.  When a Premotor's policy collapses
	# onto a single intent the entropy drops sharply (e.g. H≈0.62 on
	# n_intents=5 ⇒ one intent fires ~85% of the time = attractor lock-
	# in).  Pairs with pre_W: high W + low H = entrenched policy.
	var pre_H: Dictionary = {}
	var pre_raw: Dictionary = {}
	var pre_held: Dictionary = {}
	var pre_dwell_holds: Dictionary = {}
	var pre_dwell_breaks: Dictionary = {}
	var pre_phase_bin: Dictionary = {}
	var pre_phase_penalties: Dictionary = {}
	var pre_phase_holds: Dictionary = {}
	var pre_phase_bin_changes: Dictionary = {}
	# Advantage-gradient diagnostic (2026-05-29): per-intent W row norms
	# (non-uniform = brain differentiates intents; flat = nothing to commit to)
	# + cumulative chosen-intent histogram.
	var pre_w_rows: Dictionary = {}
	var pre_chosen_counts: Dictionary = {}
	# Gait-bucket bet (2026-05-29): per-Premotor per-bucket bias row norms +
	# current bucket.  Non-uniform across buckets = per-bucket specialization.
	var pre_bucket_bias_norms: Dictionary = {}
	var pre_current_bucket:    Dictionary = {}
	# Phase 6.7 — HomeokineticExploration mechanism counters (summed
	# across all Premotors; per-Premotor breakdown is in the diagnostic
	# if needed).  pre_xov_total is monotonic.  pre_xov_active_now is
	# the count of Premotors currently in an explore-override tick.
	# Both 0 when explore_directive_topic="" on every Premotor (default).
	var pre_xov_total: int = 0
	var pre_xov_active_now: int = 0
	# Phase 6.7 — surface HomeokineticExploration module state so we
	# can see when its gate fires (episodes_armed grows; active goes
	# 0→1).  All four fields are 0/false when the module is not
	# instantiated in the topology, keeping JSONL shape invariant.
	var hk_episodes_armed: int = 0
	var hk_active: bool = false
	var hk_long_change_ema: float = 0.0
	var hk_sample_count: int = 0
	# Phase 6.7 — saturation-streak (consecutive ticks of urgency > clamp).
	# 0 when saturation gate is disabled.
	var hk_saturation_streak: int = 0
	# Per-modality trust shares from every LateralVoter in the brain.
	# B2 observability: surfaces the full trust_weights map so we can see
	# if any modality group (vestibular, proprio, …) is starved.  Keyed
	# voter_id → modality_topic → trust_share.
	var trust_by_voter: Dictionary = {}
	# Hebbian association matrix density + magnitude per voter.  Both
	# 0 when association_enabled=false (B1 baseline) so the column shape
	# is invariant; B1 variant runs read these to confirm the matrix is
	# actually filling.
	var assoc_nnz_by_voter: Dictionary = {}
	var assoc_sum_by_voter: Dictionary = {}
	# Phase 7 Rung 7.1 — bilevel SeqGNG telemetry.  Whole-body SeqGNG over
	# consensus.0 publishes to sequence.motif.body (unconsumed).  Per-tick
	# metrics surface node_count + baked_count + current_motif so analysis
	# scripts can count clusters and build transition matrices without
	# reading the bus directly.  Defaults are zero/-1 when the module is
	# not in the topology, keeping column shape invariant across configs.
	var seqgng_by_id: Dictionary = {}
	# CPG diag — emit competence-gate trajectory + amplitudes for the
	# first CPGOscillator instance in the brain (typical config has at
	# most one).  Empty dict if no CPG module is wired.  Used by
	# scripts/cpg_probe.py to observe gate dynamics during headless
	# tabula-rasa runs.
	var cpg_state: Dictionary = {}
	var epm_tle_by_id: Dictionary = {}   # per-EPM ema_tle (aliveness signal #3)
	var epm_win_by_id: Dictionary = {}   # per-EPM current winner id
	# Reset population caches each diag tick before the metrics loop populates them.
	_cog_pop_accels.clear()
	_cog_pop_chosen.clear()
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		var t: String = m.get("type", "")
		# Phase H1 V2/V3 — cache cognitive Premotor's output for the
		# closed-loop steering bias on target_compass.  V2 used discrete
		# last_chosen → COGNITIVE_BIAS_MAP. V3 prefers continuous
		# last_accel ∈ [-1, +1] scaled by 0.5 (so ±0.5 rad ≈ ±29°).
		# Falls back to discrete map when last_accel is missing (older
		# Premotor without diag wiring).
		if mod_id == "premotor_cognitive" and t == "PremotorAI":
			var has_accel: bool = m.has("last_accel")
			if has_accel:
				# V4 fix 2026-06-05: trust the cognitive Premotor's intent_accels
				# config; just multiply by 0.5 to convert accel → rad.  The
				# earlier defensive clamp(±1.0) limited V4's widened range.
				# Final safety clamp at ±π/2 (≈ ±90°) to avoid pathological
				# bias if a config sets intent_accels above that.
				var accel_now: float = float(m.get("last_accel", 0.0))
				_cognitive_bias_rad    = clamp(accel_now * 0.5, -1.5708, 1.5708)
				_cognitive_last_chosen = int(m.get("last_chosen", _cognitive_last_chosen))
			else:
				var ch_now: int = int(m.get("last_chosen", _cognitive_last_chosen))
				if ch_now >= 0 and ch_now < COGNITIVE_BIAS_MAP.size():
					_cognitive_last_chosen = ch_now
					_cognitive_bias_rad    = COGNITIVE_BIAS_MAP[ch_now]
		# 2026-06-07 — F10: Cognitive Premotor Population (per Fractal JEPA
		# addendum A.4-A.6).  When multiple cognitive PremotorAI modules are
		# present (ids cognitive_left / cognitive_center / cognitive_right),
		# the body sums their last_accel outputs and the resulting bias replaces
		# any single-Premotor bias set above.  Empty/missing IDs are silently
		# skipped — config-only feature.  The summed accel is in the same
		# space as a single Premotor's accel; safety clamp at ±π/2.
		if mod_id in ["cognitive_left", "cognitive_center", "cognitive_right"] and t == "PremotorAI":
			_cog_pop_accels[mod_id] = float(m.get("last_accel", 0.0))
			_cog_pop_chosen[mod_id] = int(m.get("last_chosen", -1))
		if t == "Premotor" or t == "PremotorAI":
			pre_W[mod_id] = snappedf(float(m.get("W_total_norm", 0.0)), 0.0001)
			pre_H[mod_id] = snappedf(float(m.get("last_entropy", 0.0)), 0.0001)
			var wrn: Array = m.get("W_row_norms", [])
			var wrn_s: Array = []
			for wv in wrn: wrn_s.append(snappedf(float(wv), 0.0001))
			pre_w_rows[mod_id] = wrn_s
			pre_chosen_counts[mod_id] = m.get("chosen_intent_counts", [])
			var bbn: Array = m.get("bucket_bias_row_norms", [])
			if bbn.size() > 0:
				var bbn_s: Array = []
				for bv in bbn: bbn_s.append(snappedf(float(bv), 0.0001))
				pre_bucket_bias_norms[mod_id] = bbn_s
				pre_current_bucket[mod_id] = int(m.get("current_bucket", -1))
			pre_raw[mod_id] = int(m.get("raw_chosen", -1))
			pre_held[mod_id] = int(m.get("held_intent", -1))
			pre_dwell_holds[mod_id] = int(m.get("dwell_holds", 0))
			pre_dwell_breaks[mod_id] = int(m.get("dwell_breaks", 0))
			pre_phase_bin[mod_id] = int(m.get("phase_bin", -1))
			pre_phase_penalties[mod_id] = int(m.get("phase_switch_penalties", 0))
			pre_phase_holds[mod_id] = int(m.get("phase_boundary_holds", 0))
			pre_phase_bin_changes[mod_id] = int(m.get("phase_bin_changes", 0))
			# Phase 6.7 — HomeokineticExploration override counters per
			# Premotor.  total_explore_overrides_used is monotonic; it
			# increments any tick the directive overrode the softmax
			# sample.  last_explore_active is the per-tick directive
			# state.  Both are 0/false when explore_directive_topic="".
			pre_xov_total += int(m.get("total_explore_overrides_used", 0))
			if bool(m.get("last_explore_active", false)):
				pre_xov_active_now += 1
		elif t == "HomeokineticExploration":
			hk_episodes_armed = int(m.get("episodes_armed", 0))
			hk_active = bool(m.get("active", false))
			hk_long_change_ema = float(m.get("long_change_ema", 0.0))
			hk_sample_count = int(m.get("sample_count", 0))
			hk_saturation_streak = int(m.get("saturation_streak", 0))
		elif t == "LateralVoter":
			var tw: Dictionary = m.get("trust_weights", {})
			var tw_snapped: Dictionary = {}
			for k in tw:
				tw_snapped[k] = snappedf(float(tw[k]), 0.0001)
			trust_by_voter[mod_id] = tw_snapped
			assoc_nnz_by_voter[mod_id] = int(m.get("assoc_matrix_nnz", 0))
			assoc_sum_by_voter[mod_id] = snappedf(float(m.get("assoc_matrix_sum", 0.0)), 0.0001)
		elif t == "SequenceGNG":
			seqgng_by_id[mod_id] = {
				"node_count":       int(m.get("node_count", 0)),
				"baked_count":      int(m.get("baked_count", 0)),
				"current_motif":    int(m.get("current_motif", -1)),
				"match_confidence": snappedf(float(m.get("match_confidence", 0.0)), 0.0001),
				"phase":            int(m.get("phase", 0)),
			}
		elif t == "CPGOscillator" and cpg_state.is_empty():
			cpg_state = {
				"gate":           snappedf(float(m.get("competence_gate", 0.0)), 0.00001),
				"walk_amp":       snappedf(float(m.get("last_walking_amp", 0.0)), 0.00001),
				"stand_factor":   snappedf(float(m.get("last_standing_factor", 0.0)), 0.00001),
				"ema_reward":     snappedf(float(m.get("ema_reward_signal", 0.0)), 0.00001),
				"ema_tle":        snappedf(float(m.get("ema_fused_tle", 0.0)), 0.00001),
				"peak_tle":       snappedf(float(m.get("max_fused_tle_seen", 0.0)), 0.00001),
				"latest_tle":     snappedf(float(m.get("latest_fused_tle", 0.0)), 0.00001),
				"phase":          snappedf(float(m.get("phase", 0.0)), 0.0001),
				"period_ticks":   int(m.get("period_ticks", 60)),
			}
			# 2026-06-02 — sync per-tick CPG phase cache for the step_quality
			# reward channel.  Reward block advances _cpg_phase_now locally
			# each physics tick; this re-syncs it from the brain's CPG every
			# diag tick (drift ≤ 1.5% of cycle between syncs).  leg_offsets
			# rarely change at runtime so we cache them once after the first
			# read.
			_cpg_phase_now = float(m.get("phase", 0.0))
			_cpg_period_ticks = max(1, int(m.get("period_ticks", 60)))
			var lp_now: Array = m.get("leg_phase_offsets", [])
			if lp_now.size() >= 4:
				_cpg_leg_offsets_now = [
					float(lp_now[0]), float(lp_now[1]),
					float(lp_now[2]), float(lp_now[3]),
				]
			_cpg_present = true
		elif t == "EPM":
			# Per-EPM prediction error (RealityToken.tle, exposed as "tle" by
			# OgmaBrain::get_module_metrics).  Instantaneous TLE — spikes on
			# first obstacle contact for the joints/imu EPMs (aliveness #3).
			epm_tle_by_id[mod_id] = snappedf(float(m.get("tle", 0.0)), 0.00001)
			# 2026-08-07 — the WINNER too, so per-node statistics are computable
			# offline.  The selector's gating question is whether responsiveness
			# varies ACROSS support nodes or only with the coarse count of planted
			# feet; without the winner id that cannot be asked.
			epm_win_by_id[mod_id] = int(m.get("winner_id", -1))
	# 2026-06-07 — F10/F11 Cognitive Premotor Population aggregation.
	# When the population caches are non-empty, body aggregates the population's
	# accels into _cognitive_bias_rad (overrides the single-Premotor path above).
	# F10 mode (_cog_voter_enabled=false): naive sum.
	# F11 mode (_cog_voter_enabled=true): softmax-weighted sum by per-member
	# credit EMA.  Credits update via the multi-arm-bandit attribution logic
	# below (dominant member at hit-fire moment gains credit; all credits
	# decay slowly toward uniform).
	if not _cog_pop_accels.is_empty():
		# Initialize credit EMA for any new members.
		for k in _cog_pop_accels:
			if not _cog_pop_credit_ema.has(k):
				_cog_pop_credit_ema[k] = 1.0
		# Find dominant member this tick (largest |accel|).  Stash for attribution.
		var best_id: String = ""
		var best_abs: float = -1.0
		for k in _cog_pop_accels:
			if abs(_cog_pop_accels[k]) > best_abs:
				best_abs = abs(_cog_pop_accels[k])
				best_id = k
		_cog_voter_recent_dominant_id = best_id
		# F11/F12 credit attribution: every tick, credit each member proportionally
		# to its CONTRIBUTION to the current bias output.  Contribution = |accel[k]|
		# / Σ|accel|.  All members participate in the credit update proportional to
		# how much of the action they produced.  Signal = SIGNED true v_toward_target
		# (positive when body moves toward unbiased true target, negative when away).
		# F11 winner-take-all was structurally biased — disjoint accel ranges meant
		# center was never largest |accel|, so center never gained/lost credit,
		# stayed pinned at 1.0, and dominated the softmax weights.
		if _cog_voter_enabled:
			var total_abs: float = 0.0
			for k in _cog_pop_accels:
				total_abs += abs(_cog_pop_accels[k])
			if total_abs > 1e-6:
				for k in _cog_pop_accels:
					var weight_k: float = abs(_cog_pop_accels[k]) / total_abs
					_cog_pop_credit_ema[k] += _cog_voter_credit_alpha * weight_k * _last_true_v_toward_target
		# Decay all credits toward uniform 1.0 (slow drift back to prior).
		if _cog_voter_enabled:
			for k in _cog_pop_credit_ema:
				_cog_pop_credit_ema[k] += _cog_voter_decay * (1.0 - _cog_pop_credit_ema[k])
		# F13 — Cisek-style biased competition: DISCRETE selection of the
		# highest-credit member, not weighted blending.  Avoids the structural
		# small-member-bias of weighted sum: weighted aggregation of a
		# population with disjoint accel ranges rewards minimum-blame
		# contributions (which is what F11/F12 showed center accumulates most
		# credit because it loses least when things go wrong).  Discrete
		# winner-takes-all means the dominant member fully controls the bias
		# while subordinate members still update credit proportional to their
		# contribution at the time the winner emerged.
		if _cog_voter_enabled:
			var max_c: float = -INF
			var winner_id: String = ""
			for k in _cog_pop_credit_ema:
				if _cog_pop_credit_ema[k] > max_c:
					max_c = _cog_pop_credit_ema[k]
					winner_id = k
			if winner_id != "" and _cog_pop_accels.has(winner_id):
				_cognitive_bias_rad = clamp(_cog_pop_accels[winner_id] * 0.5, -1.5708, 1.5708)
			else:
				_cognitive_bias_rad = 0.0
		else:
			var pop_sum: float = 0.0
			for k in _cog_pop_accels:
				pop_sum += _cog_pop_accels[k]
			_cognitive_bias_rad = clamp(pop_sum * 0.5, -1.5708, 1.5708)
		if best_id != "" and _cog_pop_chosen.has(best_id):
			_cognitive_last_chosen = int(_cog_pop_chosen[best_id])
	# Horizontal chassis position (for walking-distance metrics).  Sampled
	# cheaply per diag tick; harness computes path length + max-distance-
	# from-origin in _aggregate().
	var chassis_pos: Vector3 = _chassis.global_transform.origin
	# Surface-distance to the nearest pyramid (aliveness signal #3).  -1 when
	# no pyramids placed (flat world).  Negative = chassis bounding overlaps.
	var nearest_pyramid_dist: float = -1.0
	_nearest_pyramid_idx = -1
	if not _pyramid_xz_positions.is_empty():
		var best_pyr: float = INF
		var best_idx: int = -1
		for pi in range(_pyramid_xz_positions.size()):
			var pp: Vector2 = _pyramid_xz_positions[pi]
			var pr: float = _pyramid_xz_radii[pi] if pi < _pyramid_xz_radii.size() else 0.0
			var pd: float = Vector2(chassis_pos.x - pp.x, chassis_pos.z - pp.y).length() - pr
			if pd < best_pyr:
				best_pyr = pd
				best_idx = pi
		nearest_pyramid_dist = best_pyr
		_nearest_pyramid_idx = best_idx
		# 2026-06-06 — increment per-pyramid engagement counter when chassis is
		# within the engagement-surface radius.  Lets us see which non-target
		# pyramids the robot interacts with over the run.
		if best_idx >= 0 and best_idx < _pyramid_engagement_counts.size() \
				and best_pyr < PYRAMID_ENGAGEMENT_RADIUS_SURFACE:
			_pyramid_engagement_counts[best_idx] += 1
	var line := {
		"t": tick_counter,
		"ep": episode_index,
		"ep_step": step_in_episode,
		"ep_alive": episode_alive_ticks,
		"y": snappedf(chassis_y, 0.001),
		"x": snappedf(chassis_pos.x, 0.001),
		"z": snappedf(chassis_pos.z, 0.001),
		"tilt": snappedf(chassis_tilt, 0.001),
		"hip1": [snappedf(h1[0], 0.01), snappedf(h1[1], 0.01),
				 snappedf(h1[2], 0.01), snappedf(h1[3], 0.01)],
		"hip2": [snappedf(h2[0], 0.01), snappedf(h2[1], 0.01),
				 snappedf(h2[2], 0.01), snappedf(h2[3], 0.01)],
		"knee": [snappedf(kn[0], 0.01), snappedf(kn[1], 0.01),
				 snappedf(kn[2], 0.01), snappedf(kn[3], 0.01)],
		"feet_y": _feet_y_array(),
		"foot_xz": _foot_local_xz_array(),
		"da": snappedf(brain.get_dopamine(), 0.001),
		"ht": snappedf(brain.get_serotonin(), 0.001),
		# Phase 7.5.R+ — per-source reward attribution (EMA-smoothed
		# hits/tick added by each channel).  Total adds = reward_rate
		# observable to brain; per-source split shows attribution.
		"reward_rate": {
			"standing": snappedf(_hit_rate_standing_ema, 0.0001),
			"walking":  snappedf(_hit_rate_walking_ema,  0.0001),
			"gated":    snappedf(_hit_rate_gated_ema,    0.0001),
			"progress": snappedf(_hit_rate_progress_ema, 0.0001),
			"level_chassis": snappedf(_hit_rate_level_chassis_ema, 0.0001),
			"gait_cycle":   snappedf(_hit_rate_gait_cycle_ema,   0.0001),
			"step_quality": snappedf(_hit_rate_step_quality_ema, 0.0001),
			"total":    snappedf(_hit_rate_standing_ema + _hit_rate_walking_ema + _hit_rate_gated_ema + _hit_rate_progress_ema + _hit_rate_level_chassis_ema + _hit_rate_gait_cycle_ema + _hit_rate_step_quality_ema, 0.0001),
		},
		"reward_cum": {
			"standing": snappedf(_hit_cum_standing, 0.01),
			"walking":  snappedf(_hit_cum_walking,  0.01),
			"gated":    snappedf(_hit_cum_gated,    0.01),
			"progress": snappedf(_hit_cum_progress, 0.01),
			"level_chassis": snappedf(_hit_cum_level_chassis, 0.01),
			"gait_cycle":   snappedf(_hit_cum_gait_cycle, 0.01),
			"step_quality": snappedf(_hit_cum_step_quality, 0.01),
		},
		"gait_cycle": {
			"good":            _gait_cycle_good_count,
			"pulses_fired":    _gait_cycle_pulses_fired,
			"consecutive_good":     _gait_cycle_consecutive_good,
			"consecutive_good_max": _gait_cycle_consecutive_good_max,
			"aborted_wobble":  _gait_cycle_aborted_wobble,
			"aborted_timeout": _gait_cycle_aborted_timeout,
			"aborted_low_progress": _gait_cycle_aborted_low_progress,
			"attempts_total":  _gait_cycle_attempts_total,
			"last_net_progress":    snappedf(_gait_cycle_last_net_progress, 0.0001),
			"progress_ema":    snappedf(_gait_cycle_progress_ema, 0.0001),
			"wobble_ema":      snappedf(_gait_cycle_wobble_ema, 0.0001),
			"eff_min_progress": snappedf(_gait_cycle_eff_min_progress, 0.0001),
			"eff_max_backward": snappedf(_gait_cycle_eff_max_backward, 0.0001),
			"active":          _gait_cycle_active,
		},
		# Stage 3.D — Bernoulli-impulse actuation telemetry.  Under the
		# default "discrete" backend, spike_count_* stay at 0 (zero-cost
		# fields, kept in the dict for shape stability across configs).
		"bri": {
			"backend":            actuation_backend,
			"spike_count_total":  _bri_spike_count_total,
			"spike_count_diag":   _bri_spike_count_this_diag,
			"offset_hip1": [snappedf(_bri_offset_hip1[0], 0.01), snappedf(_bri_offset_hip1[1], 0.01),
							snappedf(_bri_offset_hip1[2], 0.01), snappedf(_bri_offset_hip1[3], 0.01)],
			"offset_hip2": [snappedf(_bri_offset_hip2[0], 0.01), snappedf(_bri_offset_hip2[1], 0.01),
							snappedf(_bri_offset_hip2[2], 0.01), snappedf(_bri_offset_hip2[3], 0.01)],
			"offset_knee": [snappedf(_bri_offset_knee[0], 0.01), snappedf(_bri_offset_knee[1], 0.01),
							snappedf(_bri_offset_knee[2], 0.01), snappedf(_bri_offset_knee[3], 0.01)],
		},
		"progress_pb": snappedf(_min_dist_to_target_pb if _min_dist_to_target_pb < 1e8 else 0.0, 0.001),
		# Phase 7.10b — inter-diagonal phase-contrast EMA.
		# 1.0 = perfect trot (FL+RR opposite phase to FR+RL).
		# 0.0 = bound (all 4 legs in same phase).
		"phase_contrast": snappedf(_phase_contrast_ema, 0.001),
	}
	if pre_W.size() > 0:
		line["pre_W"] = pre_W
		line["pre_w_rows"] = pre_w_rows
		line["pre_chosen_counts"] = pre_chosen_counts
		if pre_bucket_bias_norms.size() > 0:
			line["pre_bucket_bias_norms"] = pre_bucket_bias_norms
			line["pre_current_bucket"] = pre_current_bucket
	if pre_H.size() > 0:
		line["pre_H"] = pre_H
		line["pre_raw"] = pre_raw
		line["pre_held"] = pre_held
		line["pre_dwell_holds"] = pre_dwell_holds
		line["pre_dwell_breaks"] = pre_dwell_breaks
		line["pre_phase_bin"] = pre_phase_bin
		line["pre_phase_penalties"] = pre_phase_penalties
		line["pre_phase_holds"] = pre_phase_holds
		line["pre_phase_bin_changes"] = pre_phase_bin_changes
	# Phase 6.7 — escape detector + HomeokineticExploration counters.
	# Run the detector BEFORE serializing telemetry so the counters
	# reflect any firing this diag-tick.  No-op when env var off.
	_check_escape_detector(metrics, chassis_pos)
	_check_epm_swap_on_hk(metrics)
	line["escape_total"]       = _escape_fired_total
	line["escape_active_now"]  = _escape_active_now_count
	line["epm_swap_total"]     = _epm_swap_total
	line["epm_swap_at_tick"]   = _epm_swap_at_tick
	# Phase 6.9 speed/PR telemetry — emitted unconditionally so column
	# shape is invariant across configs.
	# Aliveness panel telemetry (gait-ignition reframe 2026-05-29):
	#  #1 heading regulation — yaw + body-frame outward unit vector;
	#  #3 obstacle adaptation — nearest-pyramid surface distance + per-EPM TLE.
	line["heading_yaw"]              = snappedf(_last_yaw, 0.001)
	# fwd_v — body-frame forward speed (same projection the IMU feeds the brain,
	# MotorEPM.cpp reads it as the thrust/controllability signal).  Emitted for the
	# seed-avg harness's DIRECT propulsion/anti-scrub metric (mean fwd_v).
	line["fwd_v"]                    = snappedf(
		Vector2(_chassis.linear_velocity.x, _chassis.linear_velocity.z).dot(
			Vector2(sin(_last_yaw), cos(_last_yaw))), 0.001)
	# ---- RAMP DEBUG: belly rangefinder + height-homeostat state (2026-07-23) ----
	# gc_raw = raw belly ToF clearance (m); gc_norm = normalized signal the brain sees
	# (ground_clearance); cy_norm = the OLD god's-eye chassis_y_norm (absolute-Y) for
	# comparison. h_ema/h_max/h_bias pulled from MotorEPM's snapshot = the homeostat's
	# smoothed height, self-discovered ceiling, and the integrated lift bias driving hip2.
	line["gc_raw"]  = snappedf(_dbg_gc_raw, 0.0001)
	line["gc_norm"] = snappedf(clamp(_dbg_gc_raw / GROUND_CLEARANCE_STAND, 0.0, 1.0), 0.001)
	line["cy_norm"] = snappedf(clamp(chassis_y / target_height, 0.0, 1.0), 0.001)
	# TRUE swing fraction from the physics foot-contact sensor — the ground truth against
	# which any duty-factor claim must be checked, and which disagrees with BOTH proxies
	# previously used for it (see where it is computed).
	line["contact_swing"] = snappedf(_dbg_contact_swing, 0.001)
	# Servo tracking error in foot-height metres = the information hobby servos do NOT
	# report. If this is large, commanded-angle FK is a materially different signal.
	line["fk_cmd_err"] = snappedf(_dbg_fk_cmd_err, 0.00001)
	line["fk_valid_err"] = snappedf(_dbg_fk_valid_err, 0.00001)
	line["att_err_acc"] = snappedf(_dbg_att_err_acc, 0.01)
	line["att_err_imu"] = snappedf(_dbg_att_err_imu, 0.01)
	line["acc_mag"] = snappedf(_dbg_acc_mag, 0.01)
	line["acc_trust"] = snappedf(_dbg_acc_trust, 0.0001)
	# 2026-08-06 — motor-layer EPM liveness probe.  A DEAD EPM has a signature the
	# operator named: tle 0, 2 nodes, 1 baked (the seed pair, never grown).  Printing
	# it makes "is anything flowing?" a measurement instead of an inference.
	if brain != null and brain.has_method("get_module_snapshot"):
		for _gid in ["motor_gng_fl", "motor_gng_fr", "motor_gng_rl", "motor_gng_rr", "support_epm"]:
			# ⚠ An EPM snapshot has NO "module" wrapper -- its diag fields sit at top
			# level, unlike MotorEPM's.  Looking for one silently yields nothing, which
			# is how a DEAD EPM and an UNREAD EPM became indistinguishable.
			var _gs = JSON.parse_string(str(brain.get_module_snapshot(_gid)))
			if _gs is Dictionary and _gs.has("gng"):
				var _g = _gs["gng"]
				var _narr: Array = _g.get("nodes", [])
				var _nodes: int = _narr.size() if _g.has("nodes") else -1
				var _lx: Array = _g.get("last_x", [])
				var _mag: float = 0.0
				for _v in _lx: _mag += abs(float(_v))
				# ⚠ last_x is JSON null until the first step().  ALL-ZEROS is a
				# DIFFERENT failure -- it means step() ran on a zero encode (the
				# dim-mismatch trap, EPM.md "The zero-encode trap").  Testing for null
				# distinguishes them; magnitude alone cannot, and reading all-zeros as
				# "nothing arrived" cost a session.
				var _never_stepped: bool = (_g.get("last_x", null) == null)
				# GATE FIELDS (§0 rule 4: never baking / never growing / growing
				# unbounded are CONDITIONING diagnoses, not verdicts on the idea).
				#   baked -- is the vocabulary EARNED, or is it churning?
				#   top1  -- visit share of the single busiest node.  A high top1 next to
				#            a large node count is premature saturation: one word absorbs
				#            the stream while the rest are decoration.  Node count alone
				#            cannot see this (it is the same blindness that made purity,
				#            not node count, the EPM commissioning acceptance test).
				var _bake_thr: int = int(_g.get("baking_threshold", 50))
				var _baked: int = 0
				var _visits_tot: float = 0.0
				var _visits_max: float = 0.0
				for _n in _narr:
					var _v2: float = float(_n.get("visits", 0))
					_visits_tot += _v2
					if _v2 > _visits_max: _visits_max = _v2
					if int(_v2) >= _bake_thr: _baked += 1
				var _top1: float = (_visits_max / _visits_tot) if _visits_tot > 0.0 else -1.0
				line["g_" + (_gid.substr(10) if _gid.begins_with("motor_gng_") else "sup")] = [
					_nodes, _baked, snappedf(float(_gs.get("ema_tle", -1.0)), 0.000001),
					(-1.0 if _never_stepped else snappedf(_mag, 0.0001)),
					int(_g.get("mitosis_count", -1)), snappedf(_top1, 0.001),
						snappedf(float(_g.get("autotune_value", -1.0)), 0.000001),
						snappedf(float(_g.get("min_insertion_error", -1.0)), 0.000001)]
	# 2026-08-10 (P5) — body-pose EPM per-tick token mirror.  get_module_metrics reads the
	# bus last-value token, so at OGMA_PICRAWLER_DIAG_INTERVAL=1 this gives the SAME-tick
	# tle/winner the anticipation analysis joins against the trace's contact events.
	# Absent modules simply add no keys — zero cost on configs without the EPMs.
	if brain != null and brain.has_method("get_module_metrics"):
		var _pm = brain.get_module_metrics()
		for _bpid in ["body_pose", "body_pose_t"]:
			if _pm.has(_bpid):
				var _bp = _pm[_bpid]
				var _tag = "bp" if _bpid == "body_pose" else "bpt"
				line[_tag + "_tle"] = snappedf(float(_bp.get("tle", -1.0)), 0.0001)
				line[_tag + "_win"] = int(_bp.get("winner_id", -1))
				line[_tag + "_n"]   = int(_bp.get("node_count", -1))
				line[_tag + "_b"]   = int(_bp.get("baked_count", -1))
		# 2026-08-11 (twin-gate M0.d) — the phase-space EPM's token mirror, same
		# shape as bp_/bpt_ so planscore/conescore-style tools read it unchanged.
		if _pm.has("body_pose_dyn"):
			var _bd = _pm["body_pose_dyn"]
			line["bd_tle"] = snappedf(float(_bd.get("tle", -1.0)), 0.0001)
			line["bd_win"] = int(_bd.get("winner_id", -1))
			line["bd_n"]   = int(_bd.get("node_count", -1))
			line["bd_b"]   = int(_bd.get("baked_count", -1))
		# 2026-08-14 (option B) — the SURPRISE-vocabulary EPM's token mirror, same
		# shape as bp_/bd_ so planscore/conescore-style tools read it unchanged.
		if _pm.has("body_pose_pc"):
			var _bc = _pm["body_pose_pc"]
			line["pc_tle"] = snappedf(float(_bc.get("tle", -1.0)), 0.0001)
			line["pc_win"] = int(_bc.get("winner_id", -1))
			line["pc_n"]   = int(_bc.get("node_count", -1))
			line["pc_b"]   = int(_bc.get("baked_count", -1))
		# 2026-08-11 (twin-gate S0) — SequenceGNG motif mirror (seq_bodypose).
		# sg_m = active motif, sg_c = match confidence, sg_pn = predicted next
		# WINNER (the successor argmax scored by seqscore.py), sg_n/sg_b/sg_ev =
		# nodes/baked/events.  Absent module → no keys.
		if _pm.has("seq_bodypose"):
			var _sg = _pm["seq_bodypose"]
			line["sg_m"]  = int(_sg.get("motif_id", -1))
			line["sg_c"]  = snappedf(float(_sg.get("match_confidence", 0.0)), 0.001)
			line["sg_pn"] = int(_sg.get("predicted_next_id", -1))
			line["sg_bk"] = 1 if bool(_sg.get("is_baked", false)) else 0
			line["sg_n"]  = int(_sg.get("node_count", -1))
			line["sg_b"]  = int(_sg.get("baked_count", -1))
			line["sg_ev"] = int(_sg.get("n_events", 0))
	# 2026-08-10 (P7) — SERVO_KI consumer check: mean |integral| and boost duty.  Both
	# 0.0 with the lever on = the integral never engaged (deadband too wide, or the
	# body never loads its servos — either way the arm measured nothing).
	if servo_ki > 0.0:
		line["ki_mag"]  = snappedf(_ki_mag_acc / max(1, _ki_mag_n), 0.00001)
		line["ki_duty"] = snappedf(float(_ki_boost_ticks) / max(1, _ki_total_ticks), 0.001)
	# 2026-08-09 (substrate-repair P0) — BodyRhythmTracker lock quality, mirrored so a
	# seedavg arm can read whether the body's own rhythm reference is actually locked
	# (brt_plv near 1 = swing crossings land at one phase) rather than merely warmed up.
	if brain != null and brain.has_method("get_module_snapshot"):
		var _bs = JSON.parse_string(str(brain.get_module_snapshot("body_rhythm_tracker")))
		if _bs is Dictionary and _bs.has("module"):
			var _bm = _bs["module"]
			line["brt_plv"]    = snappedf(float(_bm.get("lock_plv", 0.0)), 0.001)
			line["brt_err"]    = snappedf(float(_bm.get("lock_err_ema", -1.0)), 0.001)
			line["brt_period"] = snappedf(float(_bm.get("period_est", 0.0)), 0.01)
	# 2026-08-10 (PART III / M0.b) — MotorPlanner probability-cone mirror.  The planner's
	# accumulators are RUNNING MEANS over the whole run, so any diag-cadence line carries
	# the cumulative per-depth verification scores; the last line is the run's verdict.
	# Absent module → no key (zero cost on non-planner configs).
	# 2026-08-11 (twin gates) — mirrors BOTH planner instances: "plan" = motor_planner
	# (bodypose control), "pland" = motor_planner_dyn (the [q,dq] phase-space arm).
	if brain != null and brain.has_method("get_module_snapshot"):
		for _plid in [["motor_planner", "plan"], ["motor_planner_dyn", "pland"], ["motor_planner_pc", "planc"]]:
			var _ps = JSON.parse_string(str(brain.get_module_snapshot(_plid[0])))
			if _ps is Dictionary and _ps.has("module"):
				var _pmod = _ps["module"]
				line[_plid[1]] = {
					"d":   _pmod.get("probe_depths", []),
					"t1":  _pmod.get("cone_top1", []),
					"tk":  _pmod.get("cone_topk", []),
					"ms":  _pmod.get("cone_mass", []),
					"en":  _pmod.get("cone_entropy", []),
					"n":   _pmod.get("cone_n", []),
					"pr":  _pmod.get("cone_persist", []),
					"auth": _pmod.get("authority_depth", 0),
					"jauth": _pmod.get("joint_auth", []),
					"jband": _pmod.get("joint_band", []),
					"mg":  _pmod.get("marg_top1", 0.0),
					"obs": _pmod.get("n_obs", 0),
					"mk":  _pmod.get("masked_out", 0),
					"mm":  _pmod.get("mask_mode", 0.0),
				}
				# 2026-08-11 (M1) — mask-AUTHOR mirror: trial state + the earned
				# (kept) mask list, so seed runs record what the author learned.
				# Absent unless author_mode=1 (zero cost on every other config).
				if _pmod.has("author"):
					line[_plid[1]]["au"] = _pmod["author"]
				# 2026-08-12 (M1 rung a) — per-depth×per-joint |err| tables
				# (operating decode vs hold-pose): the author_apply A/B compares
				# these across arms; jband alone is thresholded and would hide
				# sub-0.95 improvements.
				if _pmod.has("jerr"):
					line[_plid[1]]["je"] = _pmod["jerr"]
					line[_plid[1]]["jp"] = _pmod["jpers"]
				# 2026-08-12 (lever b) — planner-side publisher state
				if _pmod.has("plan_pub"):
					line[_plid[1]]["pp"] = _pmod["plan_pub"]
	# 2026-08-14 (option B) — DescendingPredictor health.  In residual mode the
	# published latent IS the error, so err/norm ≡ 1 (a tautology, caught on the
	# first smoke); the honest bite-meter is ‖prediction‖ vs ‖residual‖:
	#   dp_err = EMA ‖residual‖ (must FALL as the predictor absorbs the stride)
	#   dp_pn  = ‖cached_prediction‖ (must GROW ≫ dp_err — the absorbed part)
	# dp_seen guards the §3.2 dead-source trap via cached_consensus_valid
	# (sticky after the first context delivery; consensus_seen is a per-tick
	# freshness flag that reads false between deliveries — the first mirror
	# misread it as liveness).
	if brain != null and brain.has_method("get_module_snapshot"):
		var _dps = JSON.parse_string(str(brain.get_module_snapshot("pc_predictor")))
		if _dps is Dictionary and _dps.has("targets"):
			line["dp_seen"] = 1 if bool(_dps.get("cached_consensus_valid", false)) else 0
			for _tk in (_dps["targets"] as Dictionary):
				var _tg: Dictionary = _dps["targets"][_tk]
				line["dp_err"] = snappedf(float(_tg.get("err_ema", -1.0)), 0.0001)
				var _cp = _tg.get("cached_prediction", [])
				var _pn: float = 0.0
				if _cp is Array:
					for _v in _cp: _pn += float(_v) * float(_v)
				line["dp_pn"] = snappedf(sqrt(_pn), 0.0001)
				break
	if brain != null and brain.has_method("get_module_snapshot"):
		var _ms = JSON.parse_string(str(brain.get_module_snapshot("motor_epm")))
		if _ms is Dictionary and _ms.has("module"):
			var _mm = _ms["module"]
			line["h_ema"]  = snappedf(float(_mm.get("chassis_h_ema", 0.0)), 0.001)
			line["h_max"]  = snappedf(float(_mm.get("chassis_h_max", 0.0)), 0.001)
			line["h_bias"] = snappedf(float(_mm.get("height_bias", 0.0)), 0.001)
			# 2026-08-12 (lever b) — plan-objective CONSUMER telemetry: without
			# these the A/B cannot distinguish "the pull acted" from "the seeds
			# moved" (the consumer-fired check every lever has needed).
			line["pl_pull"] = snappedf(float(_mm.get("plan_pull", 0.0)), 0.00001)
			line["pl_w"]    = snappedf(float(_mm.get("plan_w", 0.0)), 0.0001)
			# 2026-08-14 (rear-knee planting lever) — swing-descent consumer check:
			# the descent branch (hip2 press AND/OR the new knee extension) fires
			# only while this counter grows.
			line["swd"] = int(_mm.get("swd_press_ticks", 0))
			line["swo"] = int(_mm.get("swd_overdue_ticks", 0))
			line["rlt"] = int(_mm.get("rear_land_ticks", 0))
			line["rpt"] = int(_mm.get("rear_push_ticks", 0))
			# 2026-08-17 (PART IV) — gain-socket CONSUMER counters, read-back
			# verified in C++: ga_app counts keys that LANDED (current_params
			# reflects the sent value), ga_rej counts keys the on_param_change
			# dispatch silently ignored (typo'd key = nonzero ga_rej, never
			# silence).  ga_app must track ge_pub × |gain_keys| (the ge_* block
			# below) — the two-sided §3.2 check for the evolver pipe.
			line["ga_app"] = int(_mm.get("gains_applied", 0))
			line["ga_rej"] = int(_mm.get("gains_rejected", 0))
			# Verify the belly-grounding setpoint adaptation actually FIRES.  Without
			# this the A/B cannot distinguish "the mechanism worked" from "the seeds
			# moved" -- the consumer-fired check, which this session has needed twice.
			line["h_keff"] = snappedf(float(_mm.get("height_k_eff", -1.0)), 0.001)
			# 2026-08-07 — homeokinetic support selector telemetry.  Without these the
			# consumer-fired check cannot distinguish "the selector moved the gait" from
			# "the seeds moved" — the check this session has needed at every lever.
			line["sup_bin"]  = int(_mm.get("support_bin", -1))
			line["sup_val"]  = snappedf(float(_mm.get("support_value", 1.0)), 0.001)
			line["sup_mult"] = snappedf(float(_mm.get("support_mult", 1.0)), 0.001)
			# 2026-08-09 — raw |dx|/|du| responsiveness, the numerator of the actuator-search
			# criterion value = responsiveness/(motor_tle+ε).  Unconditional in MotorEPMv2 so
			# EVERY arm can be scored on the criterion, selector present or not.
			line["sup_resp"] = snappedf(float(_mm.get("support_resp", 0.0)), 0.0001)
			# 2026-08-09 — stance_release_frac consumer check: fraction of stance-lift
			# ticks with the release live.  0.0 with the lever on = detector never fired.
			line["sr_duty"] = snappedf(float(_mm.get("sr_duty", 0.0)), 0.001)
			# 2026-08-09 P1 — shadow phases (zero authority), per tick, for offline
			# scoring against the trace's contact ground truth.  STAYS INSIDE the _mm
			# block: a mirror placed after it once dedented the scope and killed the
			# whole body script (see the P0 commit).
			for _shk in ["sh_a", "sh_b", "sh_c", "ph_l"]:
				var _shv = _mm.get(_shk, [])
				if _shv is Array and _shv.size() > 0:
					var _shr: Array = []
					for _v in _shv: _shr.append(snappedf(float(_v), 0.001))
					line[_shk] = _shr
			line["sh_ra"] = snappedf(float(_mm.get("sh_a_retro", -1.0)), 0.001)
			line["sh_rb"] = snappedf(float(_mm.get("sh_b_retro", -1.0)), 0.001)
			line["sh_rc"] = snappedf(float(_mm.get("sh_c_retro", -1.0)), 0.001)
			# Ratchet state for the forgetting diagnosis — the variables suspected of NOT
			# recovering after an inverted episode.  h_max above is the worst: a monotonic
			# max with no decay and no reset, and it sets the height setpoint.
			line["amp_gain"] = snappedf(float(_mm.get("amp_gain_mean", 0.0)), 0.001)
			line["coord_best_fit"] = snappedf(float(_mm.get("coord_best_fitness", 0.0)), 0.0001)
			line["motor_tle"] = snappedf(float(_mm.get("motor_tle", 0.0)), 0.0001)
			# --- 2026-08-02 Phase-0 instruments (report-only).  See
			# docs/reports/playful_machine_source_analysis.md §4.
			# clip_duty: fraction of post-warmup leg-ticks where the ASSEMBLED command
			# (HK + stroke + postural + height + cruse + coupling + noise) exceeded ±1
			# and was flattened by the one output clamp.  The HK loop-Jacobian assumes a
			# tanh; the body applies a hard clip — so a high clip_duty means every timing
			# lever downstream was measured through a saturated actuator.
			# hk_share: HK's share of the pre-clamp command magnitude (is the learned
			# controller driving this gait, or the additive scaffolds?).
			# echo_a: mean self-model gain on the [pos, ACTION, delta] echo channel
			# (→1 = the model has latched a channel it can predict perfectly and the
			# controller can drive perfectly, i.e. HK satisfiable without moving).
			# c_act: share of |C|'s mass sitting on those echo columns.
			# L-1b objective socket: is the posture OBJECTIVE driving the controller, or is
			# postural_gain doing it additively?  obj_w is the mean blend weight actually
			# applied (MotorEPM blends xi_tilde = (1-w)*xi + w*(x - x*)); obj_legs counts
			# how many legs have a live objective.  Both 0 => the socket is not firing and
			# any "objective replaces the additive term" claim is untestable.
			# ★ inter-leg phase coherence — the operator's "are the legs actually working
			# together" number.  Lived in diag_snapshot only until 2026-08-03, so no arm in
			# this campaign had ever reported it.
			# coh is the INSTANTANEOUS Kuramoto order parameter: for four independent legs its
			# distribution is mean 0.450 / sd 0.219, so a single reading is nearly meaningless
			# and a 12-seed sweep of it looks bimodal when nothing is locked.  plv is the
			# honest read — constant RELATIVE phase over the run, null -> 0.
			line["coh"] = snappedf(float(_mm.get("gait_coherence", 0.0)), 0.0001)
			# plv is GATED on both legs of a pair actually oscillating — a frozen body has
			# constant phases and would otherwise score ~1.  plv_n says how many ticks of
			# genuine oscillation back the number; a low plv_n means plv is unsupported,
			# not that the legs are uncoordinated.
			line["plv"]   = snappedf(float(_mm.get("interleg_plv", 0.0)), 0.0001)
			line["plv_n"] = int(_mm.get("plv_support", 0))
			# TRAILING-WINDOW plv (tau ~500 ticks).  `plv` above accumulates over the whole
			# run, so a perturbation at tick 2500 is diluted by 6000 ticks of history and the
			# recovery it is meant to score is invisible.  plv_w CAN express a before/after.
			# plv_wn is its support: |plv_w| <= plv_wn by construction, so a low plv_w with a
			# low plv_wn means "nothing was moving", not "the legs are uncoordinated".
			line["plv_w"]  = snappedf(float(_mm.get("interleg_plv_win", 0.0)), 0.0001)
			line["plv_wn"] = snappedf(float(_mm.get("plv_win_support", 0.0)), 0.0001)
			# Per-pair, so a three-legged gait can be scored among its SURVIVORS instead of
			# being averaged against three dead pairs.  Order (0,1)(0,2)(0,3)(1,2)(1,3)(2,3).
			var _pwp = _mm.get("plv_win_pairs", [])
			var _pws = _mm.get("plv_win_pair_sup", [])
			if _pwp is Array and not _pwp.is_empty(): line["plv_pairs"] = _pwp
			if _pws is Array and not _pws.is_empty(): line["plv_pair_n"] = _pws
			# Per-leg homeokinetic forward-model residual + oscillation amplitude.  The
			# inferential-gain direction turns on whether these DIFFER across legs; the
			# body-level motor_tle collapses exactly that.  −1 = leg not initialised;
			# an exactly-0.0000 residual means a limb that never moved, not a perfect model.
			var _tlg = _mm.get("tle_leg", [])
			var _amg = _mm.get("amp_leg", [])
			if _tlg is Array and not _tlg.is_empty(): line["tle_leg"] = _tlg
			if _amg is Array and not _amg.is_empty(): line["amp_leg"] = _amg
			# Panic override: (1 - panic_eff) already scales the coupling/stroke/rhythm terms,
			# so this is an EXISTING surprise-modulated gain that has never been measured.
			line["panic_eff"] = snappedf(float(_mm.get("panic_eff", 0.0)), 0.0001)
			# Consumer check for couple_prec_gain.  The weights are mean-normalised, so
			# cw_mean must read 1.00 and cw_spr = (max-min)/mean is the read that matters:
			# flat across a gain sweep = the lever never fired, which is a measurement
			# outcome and not a verdict on the idea.
			# 2026-08-04 — the beacon channel, for the Stage-0 sensing-envelope measurement.
			# beacon = fraction of frame that is beacon-coloured (the nav sensor).
			# tgt_range is GOD'S-EYE and DIAGNOSTIC ONLY — it exists so the beacon can be
			# characterised against true range and must never reach the brain.
			line["beacon"] = snappedf(_beacon_frac, 0.000001)
			line["vis_wh"] = [vision_res_w, vision_res_h]
			if walk_target_idx >= 0 and walk_target_idx < _pyramid_xz_positions.size():
				var _tp: Vector2 = _pyramid_xz_positions[walk_target_idx]
				line["tgt_range"] = snappedf(Vector2(_chassis.global_transform.origin.x,
					_chassis.global_transform.origin.z).distance_to(_tp), 0.001)
			# L1 NAV CONSUMER CHECK.  gb_msgs == 0 with goal_bearing_topic configured means the
			# nav module never published and the heading PD is silently still holding the spawn
			# bearing — indistinguishable from "the lever did nothing" without this number.
			# Per-leg hip1 clip duty. LEFT = legs 0,2 / RIGHT = 1,3 (the skid-steer `side`
			# pattern). A RECTIFIED differential shows as one side pinned near its
			# straight-line duty while the other falls — the commanded turn is symmetric but
			# only half of it survives the clamp.
			var _clg = _mm.get("clip_h1_leg", [])
			var _prg = _mm.get("pre_h1_leg", [])
			if _clg is Array and not _clg.is_empty(): line["clip_h1_leg"] = _clg
			if _prg is Array and not _prg.is_empty(): line["pre_h1_leg"] = _prg
			# CONSUMER CHECK for the heading integral term. 0.000 with heading_trim_rate != 0
			# means the integrator never accumulated — and without this line, "inert lever" and
			# "unwired instrument" are indistinguishable, which is exactly how this sweep first
			# reported every arm at trim=0.000.
			line["h_trim"] = snappedf(float(_mm.get("heading_trim", 0.0)), 0.00001)
			# CONSUMER CHECK for commit_prec — MISSING until now, which made a whole 35-run
			# sweep uninterpretable: the analysis read 1.00 from a .get() default and could not
			# tell "the lever ran and did nothing" from "the lever never ran".
			line["cR"]    = snappedf(float(_mm.get("couple_R", -1.0)), 0.001)
			line["pretro"] = snappedf(float(_mm.get("phase_retro", -1.0)), 0.001)
			line["resT"]  = snappedf(float(_mm.get("res_period", -1.0)), 0.1)
			line["resA"]  = snappedf(float(_mm.get("res_amp", -1.0)), 0.001)
			line["resL"]  = snappedf(float(_mm.get("res_lock", -1.0)), 0.001)
			line["fprog"] = snappedf(float(_mm.get("fwd_progress_ema", -99.0)), 0.00001)
			line["ierr"]  = snappedf(float(_mm.get("intent_err", -99.0)), 0.00001)
			line["cprec"] = snappedf(float(_mm.get("commit_prec", -1.0)), 0.0001)
			line["imsgs"] = int(_mm.get("intent_msgs", 0))
			line["cboost"] = snappedf(float(_mm.get("commit_boost", -1.0)), 0.0001)
			line["gb_msgs"] = int(_mm.get("goal_bearing_msgs", 0))
			line["gb_err"]  = snappedf(float(_mm.get("goal_bearing_err", 0.0)), 0.0001)
			line["cw_spr"]  = snappedf(float(_mm.get("couple_w_spr", 0.0)), 0.0001)
			line["cw_mean"] = snappedf(float(_mm.get("couple_w_mean", 0.0)), 0.0001)
			# hip2<->knee command sign agreement — the operator's "for the first time I see
			# hip2 and knee work together to lift the chassis".  0.5 = chance.
			line["hk_agree"] = snappedf(float(_mm.get("hip2_knee_agree", 0.0)), 0.0001)
			line["obj_active"] = 1 if bool(_mm.get("obj_active", false)) else 0
			line["obj_w"]      = snappedf(float(_mm.get("obj_weight", 0.0)), 0.0001)
			line["obj_legs"]   = int(_mm.get("obj_legs", 0))
			line["tq_mag"] = snappedf(_tq_mag_acc / maxf(1.0, _tq_n), 0.0001)
			line["tq_sat"] = snappedf(_tq_sat_acc / maxf(1.0, _tq_n), 0.0001)
			line["clip_duty"] = snappedf(float(_mm.get("clip_duty", 0.0)), 0.0001)
			line["hk_share"]  = snappedf(float(_mm.get("hk_share", 0.0)), 0.0001)
			line["echo_a"]    = snappedf(float(_mm.get("echo_a_gain", 0.0)), 0.0001)
			line["c_pos"]     = snappedf(float(_mm.get("c_mass_pos", 0.0)), 0.0001)
			line["c_act"]     = snappedf(float(_mm.get("c_mass_act", 0.0)), 0.0001)
			line["c_del"]     = snappedf(float(_mm.get("c_mass_del", 0.0)), 0.0001)
			var _cdj: Array = _mm.get("clip_duty_j", [])
			if _cdj is Array and _cdj.size() >= 3:
				line["clip_h1"] = snappedf(float(_cdj[0]), 0.0001)
				line["clip_h2"] = snappedf(float(_cdj[1]), 0.0001)
				line["clip_kn"] = snappedf(float(_cdj[2]), 0.0001)
			var _pmj: Array = _mm.get("pre_mag_j", [])
			if _pmj is Array and _pmj.size() >= 3:
				line["pre_h1"] = snappedf(float(_pmj[0]), 0.001)
				line["pre_h2"] = snappedf(float(_pmj[1]), 0.001)
				line["pre_kn"] = snappedf(float(_pmj[2]), 0.001)
			# swing_frac = fraction of legs MotorEPM's foot-height detector calls
			# "swinging" — the gate stance_lift and the Cruse rules ride on.  Read it
			# against this body's OWN absolute planted test (feet_y < stance_y_threshold):
			# a detector reporting ~0.5 while the feet are genuinely down ~80% of the
			# time is measuring gait phase, not ground contact.
			line["swing_frac"] = snappedf(float(_mm.get("swing_frac", 0.0)), 0.001)
			# cruse_bias = mean |MotorEPM's OWN Cruse contribution|.  EXACTLY 0 means
			# MotorEPM's Cruse block never ran, so any Cruse-looking motion is coming
			# from elsewhere.  NOTE there are TWO Rule-3 knobs: MotorEPM's
			# `cruse_rule3_weight` (inert unless `cruse_gain` != 0 — this is the one the
			# MOTOR-EPM panel writes) and CruseCoordinator's separate `rule3_weight`
			# (gated by `cruse_bias_gain`, which defaults to 1.0 = ON).
			line["cruse_bias"] = snappedf(float(_mm.get("cruse_bias", 0.0)), 0.0001)
			# phase_agree: does a LEGAL body-rhythm phase gate reproduce the god's-eye swing
			# detector? ~0.5 = no information; ~1.0 = the detector is really a phase gate and
			# the oracle can be replaced by a signal a real robot has.
			line["phase_agree"] = snappedf(float(_mm.get("phase_agree", 0.0)), 0.001)
			line["legphase_agree"] = snappedf(float(_mm.get("legphase_agree", 0.0)), 0.001)
			# ---- Phase-0 gait-alignment diagnostic (2026-07-26, `gait_align_diag`) ----
			# Is the propulsive stroke phase-locked to ground contact AT ALL?  The stroke
			# rides L.phase (from the KNEE); the stance gate rides the FOOT-HEIGHT cycle.
			# td_plv = phase-locking value of the stroke waveform at TRUE touchdown:
			#   ~0 = the foot lands at a uniformly random point in the power stroke, so
			#        half the stroke pushes air and half the return swing scrubs.
			#   ~1 = locked (and td_phase then says whether stroke_phase is mis-offset).
			# pos_stance vs pos_swing: fraction of each spent in the stroke's positive
			# half.  Both ~0.5 = no relation.  torque_* answers whether joint_torque can
			# separate stance from swing, which is the prerequisite for a load lever.
			line["td_plv"]     = snappedf(float(_mm.get("stroke_td_plv", 0.0)), 0.0001)
			line["sd_plv"]     = snappedf(float(_mm.get("stroke_sd_plv", 0.0)), 0.0001)
			line["pos_stance"] = snappedf(float(_mm.get("stroke_pos_stance", 0.0)), 0.0001)
			line["pos_swing"]  = snappedf(float(_mm.get("stroke_pos_swing", 0.0)), 0.0001)
			line["contact_duty"] = snappedf(float(_mm.get("contact_duty", 0.0)), 0.0001)
			line["tq_agree"]   = snappedf(float(_mm.get("torque_agree", 0.0)), 0.0001)
			line["tq_stance"]  = snappedf(float(_mm.get("torque_stance", 0.0)), 0.00001)
			line["tq_swing"]   = snappedf(float(_mm.get("torque_swing", 0.0)), 0.00001)
			# tq_agree on hip1 ALONE — the signal a load-derived step clock would actually
			# threshold.  The summed three-servo version dilutes it (per-joint stance/swing
			# ratio is 1.368 on hip1 vs 1.148 summed), so scope a load lever on THIS number.
			line["tq_agree_hip1"] = snappedf(float(_mm.get("torque_agree_hip1", 0.0)), 0.0001)
			# --- stroke-to-step lock.  step_lock is the CONSUMER CHECK; mv_* are the
			# NON-tautological reads (td_plv/pos_stance above are satisfied by construction
			# once the phase is touchdown-referenced, whereas mv_* is computed on ACHIEVED
			# hip1 motion and cannot be faked by re-referencing the command).
			line["step_lock"]   = snappedf(float(_mm.get("step_lock", 0.0)), 0.0001)
			line["step_period"] = snappedf(float(_mm.get("step_period", 0.0)), 0.01)
			line["step_td_err"] = snappedf(float(_mm.get("step_td_err", 0.0)), 0.0001)
			# How often the stroke's phase reference SWAPPED between the step clock and the
			# L.phase fallback.  Intermittent locking is worse than never locking: each swap
			# is a discontinuity in the driven command.
			line["step_flips"]  = int(_mm.get("step_lock_flips", 0))
			# FOOTFALL REGULARITY — cycle-to-cycle CV of the true inter-touchdown interval.
			# The prerequisite for any touchdown-referenced phase: a PLL cannot lock to a
			# rhythm whose period wanders.  Reported for EVERY instrumented arm, including
			# the control, so "is the gait periodic at all?" is answerable on the baseline.
			line["step_cv"]     = snappedf(float(_mm.get("step_cv", 0.0)), 0.0001)
			# CHATTER vs APERIODICITY: mean stance/swing BOUT length and the fraction of
			# bouts too short to be a real gait phase.  A high step_cv with a high
			# short_bout_frac is a contact-signal problem; a high step_cv with normal bouts
			# is a genuinely irregular gait.  Same statistic, opposite next lever.
			line["stance_bout"] = snappedf(float(_mm.get("stance_bout", 0.0)), 0.01)
			line["swing_bout"]  = snappedf(float(_mm.get("swing_bout", 0.0)), 0.01)
			line["short_bouts"] = snappedf(float(_mm.get("short_bout_frac", 0.0)), 0.0001)
			# ...and the interval CV counting ONLY real steps (touchdowns preceded by a
			# swing long enough to be a stride).  If step_cv is ~1.0 because micro-lifts are
			# pooled with real steps, THIS is where the hidden rhythm shows up.
			line["step_cv_real"]  = snappedf(float(_mm.get("step_cv_real", 0.0)), 0.0001)
			line["step_per_real"] = snappedf(float(_mm.get("step_per_real", 0.0)), 0.01)
			line["mv_stance"]   = snappedf(float(_mm.get("mv_stance", 0.0)), 0.000001)
			line["mv_swing"]    = snappedf(float(_mm.get("mv_swing", 0.0)), 0.000001)
			# explore_mult = the progress->commit damping actually applied to the
			# coordination probe sigma.  If this already sits at 0 on flat ground then a
			# precision gate on the same sigma would be a TAUTOLOGY (CLAUDE.md 3.2 r1).
			line["explore_mult"] = snappedf(float(_mm.get("explore_mult", 1.0)), 0.001)
			# Per-leg cycle periods: hip1 = the stride, knee = what the stroke's phase is
			# read from, foot = what the stance gate rides.  Three different numbers means
			# three clocks, and the beat between them is the stumble.
			var _p1: Array = _mm.get("ga_hip1_per", [])
			var _pk: Array = _mm.get("ga_knee_per", [])
			var _pf: Array = _mm.get("ga_foot_per", [])
			if _p1 is Array and not _p1.is_empty(): line["per_hip1"] = _p1
			if _pk is Array and not _pk.is_empty(): line["per_knee"] = _pk
			if _pf is Array and not _pf.is_empty(): line["per_foot"] = _pf
			# per_con = the REAL step period from the physics touch flag.  per_foot is the
			# INCUMBENT detector's cycle; a fast per_foot next to a slow per_con is the
			# detector chattering (it is a self-referential threshold that stance_lift
			# rings), not the body stepping faster.
			var _pc: Array = _mm.get("ga_con_per", [])
			if _pc is Array and not _pc.is_empty(): line["per_con"] = _pc
			# Per-joint stance/swing load ratio [hip1, hip2, knee].  1.0 = that servo
			# reports nothing about whether the foot is bearing weight.
			var _tj: Array = _mm.get("torque_sep_joint", [])
			if _tj is Array and not _tj.is_empty(): line["tq_sep_j"] = _tj
			# PURCHASE GATE (stroke_load_gain): mean and spread of the per-leg stroke
			# gate actually applied.  mean 1.0 with spread EXACTLY 0 means the gate never
			# fired -- either the gain is 0 or torque_topic is unwired.  This is the
			# "did the consumer fire?" number; a gate has shipped as silent dead code here
			# once already (CLAUDE.md 3.2 rule 5).
			line["sgate"]    = snappedf(float(_mm.get("stroke_gate_mean", 1.0)), 0.0001)
			line["sgate_spr"] = snappedf(float(_mm.get("stroke_gate_spread", 0.0)), 0.0001)
			# SWING DYNAMICS.  yaw_swing_excess = mean |yaw rate| while ANY foot is
			# airborne minus the all-four-down reference -- i.e. "does swinging a limb
			# spin the chassis?", which is the operator's UI observation as a number.
			# yaw_per_leg attributes it per limb so "it's the back legs" is checkable.
			# swing_tuck_frac is the did-it-fire number for the swing_tuck lever; 0 with
			# the gains non-zero means contact_topic is unwired.
			line["yaw_allplant"] = snappedf(float(_mm.get("yaw_allplant", 0.0)), 0.00001)
			line["yaw_anyswing"] = snappedf(float(_mm.get("yaw_anyswing", 0.0)), 0.00001)
			line["yaw_swing_excess"] = snappedf(float(_mm.get("yaw_swing_excess", 0.0)), 0.00001)
			line["swing_tuck_frac"] = snappedf(float(_mm.get("swing_tuck_frac", 0.0)), 0.001)
			# Mean |shank off vertical| as the CONTROLLER sees it (radians). The collector
			# also reconstructs this from hip2/knee; they should agree.
			line["tib_off_ctl"] = snappedf(float(_mm.get("tibia_off_mean", 0.0)), 0.0001)
			var _yl: Array = _mm.get("yaw_per_leg", [])
			if _yl is Array and not _yl.is_empty(): line["yaw_per_leg"] = _yl
			# |delta yaw rate| split the same way. THIS is the one that can see a swing
			# reaction torque: mean yaw RATE is dominated by intentional steering (which
			# acts through planted feet), so a limb impulse is invisible under it.
			line["yawd_allplant"] = snappedf(float(_mm.get("yawd_allplant", 0.0)), 0.000001)
			line["yawd_anyswing"] = snappedf(float(_mm.get("yawd_anyswing", 0.0)), 0.000001)
			line["yawd_swing_excess"] = snappedf(float(_mm.get("yawd_swing_excess", 0.0)), 0.000001)
			var _yd: Array = _mm.get("yawd_per_leg", [])
			if _yd is Array and not _yd.is_empty(): line["yawd_per_leg"] = _yd
			# (gait_phase — has the imposed trot [0, pi, pi, 0] drifted? — is already
			#  emitted a few lines below; do not duplicate it here.)
			# gait_phase = the LIVE Kuramoto target offsets [FL,FR,RL,RR].  Constant
			# unless coord_adapt_rate (leaky tracker) or coord_reward_drive (fitness
			# ratchet) is on.  Surfaced so "did the coordination recover after the robot
			# got stuck, or did it lock in a destructive pattern?" is a MEASUREMENT.
			# A leaky tracker must drift under perturbation and RETURN; a ratchet won't.
			var _gp = _mm.get("gait_phase", null)
			if _gp is Array:
				var _gpo: Array = []
				for _v in _gp: _gpo.append(snappedf(float(_v), 0.001))
				line["gait_phase"] = _gpo
	# ---- PART IV GainEvolver mirror (2026-08-17).  Self-guarded: absent module
	# ⇒ no ge_* keys ⇒ promoted-config logs unchanged.  Cheap metrics path, NOT
	# get_module_snapshot (the per-tick full-snapshot lesson).  ge_ji is the
	# incumbent criterion J (error-form, lower = better): THE trace that must
	# FALL during the convergence gate and SPIKE-then-recover at the (d)-test.
	# ge_ph: 0=warmup 1=incumbent 2=candidate.  ge_vec = the ACTIVE vector.
	if brain != null and brain.has_method("get_module_metrics"):
		var _ge: Dictionary = brain.get_module_metrics().get("gain_evolver", {})
		if not _ge.is_empty():
			line["ge_gen"]   = int(_ge.get("generation", 0))
			line["ge_acc"]   = int(_ge.get("accepts", 0))
			line["ge_rev"]   = int(_ge.get("reverts", 0))
			line["ge_pub"]   = int(_ge.get("publishes", 0))
			line["ge_sig"]   = snappedf(float(_ge.get("sigma", 0.0)), 0.0001)
			line["ge_ph"]    = int(_ge.get("phase", 0))
			line["ge_wt"]    = int(_ge.get("win_tick", 0))
			line["ge_ji"]    = snappedf(float(_ge.get("J_inc", -1.0)), 0.0001)
			line["ge_jc"]    = snappedf(float(_ge.get("J_cand", -1.0)), 0.0001)
			# Per-term breakdown of the INCUMBENT window — the dead-term / term-
			# domination check (§3.2): a weight whose term never moves is dead,
			# which is a measurement about the sensor, not the criterion.
			line["ge_falls"] = snappedf(float(_ge.get("falls", 0.0)), 0.01)
			line["ge_tilt"]  = snappedf(float(_ge.get("tilt_sd", 0.0)), 0.000001)
			line["ge_dis"]   = snappedf(float(_ge.get("distress_duty", 0.0)), 0.0001)
			line["ge_unl"]   = snappedf(float(_ge.get("unloaded_mean", 0.0)), 0.0001)
			line["ge_flow"]  = snappedf(float(_ge.get("flow_term", 0.0)), 0.0001)
			line["ge_minld"] = snappedf(float(_ge.get("loaded_min", 0.0)), 0.0001)
			# NEAR-INVERSION DWELL, logged at full per-tick resolution even though it
			# ships at weight 0: the 60-tick body-log proxy that measured it as WORSE
			# than sd(upright) inflated its noise ~1.4x, so this is the data that
			# decides its weight honestly.
			line["ge_dwell"] = snappedf(float(_ge.get("dwell", 0.0)), 0.000001)
			# Noise-aware acceptance (post-gate-2): sigma_hat is estimated from the
			# search's own revert pairs and the margin is what a candidate must
			# actually beat.  ge_sig_e == 0 with ge_nn > 0 would mean the estimator
			# ran but found no noise; ge_nn == 0 late in a run means it never
			# gathered a pair, i.e. nothing has been reverted.
			line["ge_sig_e"] = snappedf(float(_ge.get("sigma_est", 0.0)), 0.00001)
			line["ge_marg"]  = snappedf(float(_ge.get("accept_margin", 0.0)), 0.00001)
			line["ge_nn"]    = int(_ge.get("noise_n", 0))
			var _gev = _ge.get("vec", [])
			if _gev is Array and not _gev.is_empty():
				var _gvo: Array = []
				for _v in _gev: _gvo.append(snappedf(float(_v), 0.0001))
				line["ge_vec"] = _gvo
	line["radial_compass"]           = [snappedf(_last_radial_compass.x, 0.001),
										snappedf(_last_radial_compass.y, 0.001)]
	line["target_compass"]           = [snappedf(_last_target_compass.x, 0.001),
										snappedf(_last_target_compass.y, 0.001)]
	line["walk_visit_count"]         = walk_visit_count
	line["walk_target_idx"]          = walk_target_idx
	line["nearest_pyramid_dist"]     = snappedf(nearest_pyramid_dist, 0.001)
	# 2026-06-13 — distance to the ACTIVE target (chassis→walk_target_pos, XZ).
	# nearest_pyramid_dist is nearest-ANY-pyramid; this is the active goal range
	# needed for honest nav metrics (active-inference signature figure).  -1 = no target.
	var target_dist: float = -1.0
	if walk_target_idx >= 0:
		target_dist = (walk_target_pos - Vector2(_chassis.global_transform.origin.x,
												 _chassis.global_transform.origin.z)).length()
	line["target_dist"]              = snappedf(target_dist, 0.001)
	# 2026-06-13 — panic pathway GATE 0: distress telemetry (observe-only).
	line["stuck_deficit"]            = snappedf(_stuck_deficit, 0.001)
	line["distress"]                 = snappedf(_distress, 0.001)
	if vision_steer:
		line["vision_compass"]       = [snappedf(_vision_compass.x, 0.001), snappedf(_vision_compass.y, 0.001)]
	# V1 predictive-decode probe: log the epm_color latent alongside the
	# ground-truth target bearing (target_compass) so we can test offline whether
	# vision carries bearing.  Only when vision is wired (publish_vision).
	if publish_vision and brain.has_method("get_module_metrics"):
		var _vm: Dictionary = brain.get_module_metrics().get("epm_color", {})
		if _vm.has("last_latent"):
			line["vision_latent"] = _vm["last_latent"]
		# Diagnostic (OGMA_PICRAWLER_LOG_VISION_RAW=1): also log the RAW pre-encoder
		# fields so we can compare JL latent vs raw RGB vs depth decode of bearing.
		if OS.get_environment("OGMA_PICRAWLER_LOG_VISION_RAW") == "1":
			line["vision_rgb"]   = Array(_last_vision_pixels)
			line["vision_depth"] = Array(_last_vision_depth)
	# 2026-06-06 — pyramid indexing for nav-engagement diagnostics.
	line["nearest_pyramid_idx"]      = _nearest_pyramid_idx
	line["pyramid_engagement_counts"] = _pyramid_engagement_counts.duplicate()
	# 2026-06-07 — F11 cognitive Lateral Voter telemetry.
	if not _cog_pop_credit_ema.is_empty():
		var credit_snapshot: Dictionary = {}
		for k in _cog_pop_credit_ema:
			credit_snapshot[k] = snappedf(_cog_pop_credit_ema[k], 0.0001)
		line["cog_voter_credits"] = credit_snapshot
		line["cog_voter_dominant"] = _cog_voter_recent_dominant_id
	# Phase H1 leg-event diag — verifies events fire at expected rate per leg.
	line["leg_lifted_counts"]        = [_leg_lifted_count[0], _leg_lifted_count[1],
										_leg_lifted_count[2], _leg_lifted_count[3]]
	line["leg_planted_counts"]       = [_leg_planted_count[0], _leg_planted_count[1],
										_leg_planted_count[2], _leg_planted_count[3]]
	line["leg_event_intensity_sum"]  = [snappedf(_leg_event_intensity_sum[0], 0.0001),
										snappedf(_leg_event_intensity_sum[1], 0.0001),
										snappedf(_leg_event_intensity_sum[2], 0.0001),
										snappedf(_leg_event_intensity_sum[3], 0.0001)]
	line["cognitive_chosen"]         = _cognitive_last_chosen
	if _force_cog_bias_enabled:
		_cognitive_bias_rad = _force_cog_bias_rad   # L2: honest telemetry of forced value
	line["cognitive_bias_rad"]       = snappedf(_cognitive_bias_rad, 0.0001)
	line["yaw_probe_delta"]          = snappedf(_yaw_probe_delta, 0.001)
	# 2026-06-06 — CruseCoordinator diag snapshot (compact fields only).
	# Reads its existing per-tick snapshot_state via the brain API; bloat-controlled
	# by extracting just rule counters + per-leg plant flags + per-premotor bias
	# magnitudes.  No-op when no cruse_coordinator module in the graph.
	if brain.has_method("get_module_snapshot"):
		var cs: String = brain.get_module_snapshot("cruse_coordinator")
		if cs != "":
			var cj_var: Variant = JSON.parse_string(cs)
			if cj_var is Dictionary:
				var cj: Dictionary = cj_var
				line["cruse_r1_fires"]   = cj.get("total_rule1_fires", 0)
				line["cruse_r2_fires"]   = cj.get("total_rule2_fires", 0)
				line["cruse_r3_fires"]   = cj.get("total_rule3_fires", 0)
				line["cruse_r1_violations"] = cj.get("total_rule1_violations", 0)
				line["cruse_r1_compliant"]  = cj.get("total_rule1_compliant",  0)
				var legs: Array = cj.get("legs", [])
				var planted := []
				var nswing  := []
				var nstance := []
				var ltd     := []  # last_touchdown_tick per leg
				var llo     := []  # last_liftoff_tick per leg
				var swing_ema := []
				var stance_ema := []
				for L in legs:
					planted.append(int(L.get("is_planted", false)))
					nswing.append(L.get("n_swing_samples", 0))
					nstance.append(L.get("n_stance_samples", 0))
					ltd.append(L.get("last_touchdown_tick", -1))
					llo.append(L.get("last_liftoff_tick", -1))
					swing_ema.append(snappedf(L.get("swing_duration_ema", 0.0), 0.1))
					stance_ema.append(snappedf(L.get("stance_duration_ema", 0.0), 0.1))
				line["cruse_legs_planted"] = planted
				line["cruse_n_swing"]  = nswing
				line["cruse_n_stance"] = nstance
				# 2026-06-06 — per-leg phase / cycle fields for gait-family + phase-offset analysis.
				line["cruse_last_td"]      = ltd          # last touchdown tick (per leg)
				line["cruse_last_lo"]      = llo          # last liftoff tick (per leg)
				line["cruse_swing_dur_ema"] = swing_ema    # per-leg swing duration EMA (sim ticks)
				line["cruse_stance_dur_ema"]= stance_ema   # per-leg stance duration EMA (sim ticks)
				var pms: Array = cj.get("premotors", [])
				var bias_norms := []
				for pm in pms:
					bias_norms.append(snappedf(pm.get("last_bias_norm", 0.0), 0.001))
				line["cruse_pm_bias_norms"] = bias_norms
	line["target_loom"]              = snappedf(_last_target_loom, 0.001)
	line["epm_tle"]                  = epm_tle_by_id
	line["epm_win"]                  = epm_win_by_id
	line["lateral_v"]                = snappedf(_last_lat_v, 0.0001)
	line["speed_now"]                = snappedf(current_speed, 0.0001)
	line["speed_ema"]                = snappedf(_speed_ema, 0.0001)
	line["speed_best"]               = snappedf(best_speed, 0.0001)
	line["sustained_speed_ticks"]    = current_sustained_speed_ticks
	line["best_sustained_speed_ticks"] = best_sustained_speed_ticks
	line["pr_event_count"]           = pr_event_count
	line["homeo_reward_last"]        = snappedf(_homeo_reward_last, 0.00001)
	line["walk_miss_count"]          = walk_miss_fired_count
	line["homeo_upright_ema"]        = snappedf(_homeo_upright_ema, 0.001)
	line["homeo_phase_factor_last"]  = snappedf(_homeo_phase_factor_last, 0.001)
	line["homeo_q_ema"]              = snappedf(_homeo_q_ema, 0.001)
	var _phase_R: Array = []
	for pi in range(4):
		var pr_acc: float = 0.0
		for pj in range(4):
			if pi == pj:
				continue
			var pkk: int = pi * 4 + pj
			pr_acc += sqrt(float(_homeo_phase_ema_x[pkk]) ** 2 + float(_homeo_phase_ema_y[pkk]) ** 2)
		_phase_R.append(snappedf(pr_acc / 3.0, 0.001))
	line["homeo_phase_R"]            = _phase_R
	line["homeo_period_ema"]         = [snappedf(float(_homeo_period_ema[0]), 0.1), snappedf(float(_homeo_period_ema[1]), 0.1), snappedf(float(_homeo_period_ema[2]), 0.1), snappedf(float(_homeo_period_ema[3]), 0.1)]
	line["pre_xov_total"]      = pre_xov_total
	line["pre_xov_active_now"] = pre_xov_active_now
	line["hk_episodes_armed"]  = hk_episodes_armed
	line["hk_active"]          = hk_active
	line["hk_long_change_ema"] = snappedf(hk_long_change_ema, 0.00001)
	line["hk_sample_count"]    = hk_sample_count
	line["hk_saturation_streak"] = hk_saturation_streak
	if trust_by_voter.size() > 0:
		line["trust"] = trust_by_voter
		line["assoc_nnz"] = assoc_nnz_by_voter
		line["assoc_sum"] = assoc_sum_by_voter
	# Phase 7 Rung 7.1 — always emit the seqgng dict (empty when no
	# SequenceGNG modules are in the topology).  Analysis scripts can
	# `.get("seqgng", {})` and treat absence as zero clusters.
	line["seqgng"] = seqgng_by_id
	# Phase 7.x — CPG state per diag tick.  Empty dict when no CPG
	# module in topology, keeping JSONL shape stable.
	line["cpg"] = cpg_state
	# Cumulative trainer-pulse counters.  Always emitted (even when 0)
	# so the audit-trail column exists in every JSONL line; analysis
	# scripts can assert == 0 across headless runs in a single check.
	line["trainer_good_count"] = _trainer_good_count
	line["trainer_bad_count"]  = _trainer_bad_count
	# Cumulative auto-reset count.  Always emitted (even when 0 or when
	# auto_reset_on_inversion is off) so the audit-trail column shape is
	# invariant.  Headless A/Bs that compare arms with different
	# auto_reset settings will see non-zero values diverging across arms
	# — by design, since auto-reset is an opt-in experimental variable.
	line["auto_reset_count"] = auto_reset_count
	# B3 leg-symmetry sync counter.  0 when leg_symmetry_mode="off"
	# (baseline).  A/Bs use this to confirm the mechanism fired the
	# expected number of times (≈ duration_s / (mc_episode_period/60)).
	line["leg_symmetry_sync_count"] = leg_symmetry_sync_count
	# Curriculum tracking — emit even when no curriculum is loaded
	# (idx=0, name="") so the column shape is invariant.
	if CurriculumManager.has_curriculum():
		line["curr_idx"]  = CurriculumManager.current_idx
		line["curr_name"] = CurriculumManager.current_name()
		# Feed competence metrics for auto-advance.  The body tracks
		# cumulative_alive_ticks; pass that + chassis_y so curriculum
		# rules can gate on them.
		CurriculumManager.tick_competence(tick_counter, {
			"cumulative_alive_ticks":      cumulative_alive_ticks,
			"best_cumulative_alive_ticks": best_cumulative_alive_ticks,
			"max_distance_from_origin":    max_distance_from_origin,
			"best_speed":                  best_speed,
			"best_sustained_speed_ticks":  best_sustained_speed_ticks,
			"chassis_y":                   chassis_y,
			"tick_counter":                tick_counter,
			"walk_visit_count":            walk_visit_count,
		})
	print(JSON.stringify(line))
	# Stage 3.D — reset per-diag spike counter AFTER emit, so the next
	# diag line captures spikes accumulated over the next diag interval.
	# spike_count_total continues to accumulate across the whole run.
	_bri_spike_count_this_diag = 0

## Whitelist of @export params that a curriculum stage is allowed to
## override.  Anything outside this list is silently dropped (with a
## warning).  Reason: a curriculum file is data, not code — it must
## not be able to mutate body internals like _chassis or tick_counter.
const _CURRICULUM_ALLOWED_KEYS: Array = [
	"publish_vision",   # let a vision curriculum enable the camera→epm_color stream
	"vision_steer",     # V2: let a curriculum enable vision-derived steering
	"target_height", "reward_shape", "height_penalty_grace",
	"height_penalty_scale", "height_penalty_gain",
	"tilt_target_rad",
	"peak_height", "band_width",
	"walk_target_velocity", "walk_hit_rate", "walk_reward_mode",
	"walk_heading_consistency_gain",
	"standing_baseline_factor", "gated_walk_bonus_rate",
	"per_leg_credit_gain", "stance_y_threshold", "phase_contrast_gain",
	"step_quality_reward_gain", "step_lift_scale", "step_quality_ema_alpha",
	"hip1_spring_stiffness", "hip1_spring_damping",
	"hip2_spring_stiffness", "hip2_spring_damping",
	"knee_spring_stiffness", "knee_spring_damping",
	"motor_force_scale", "joint_angular_damping",
	"joint_angular_erp", "joint_angular_limit_softness",
	"standing_reward_ema_alpha", "gated_walk_velocity_mode",
	"reward_min_height", "peak_height_mode", "height_reference",
	"progress_reward_gain", "progress_reward_min_delta",
	"fail_tilt_rad",
	"level_chassis_rate", "level_chassis_tilt_scale", "level_chassis_walking_budget",
	"gait_cycle_reward_gain", "gait_cycle_window_ticks",
	"gait_cycle_min_progress", "gait_cycle_max_backward",
	"gait_cycle_adaptive_thresholds", "gait_cycle_progress_K",
	"gait_cycle_wobble_K", "gait_cycle_threshold_ema_alpha",
	"gait_cycle_warmup_cycles", "gait_cycle_consecutive_required",
	"stability_gain", "stability_y_norm", "stability_speed",
	"antirot_threshold", "antirot_scale", "antirot_gain",
	"energy_deadband", "energy_scale", "energy_gain",
	"auto_reset_on_inversion", "auto_reset_tilt_threshold",
	"auto_reset_max_height", "auto_reset_dwell_ticks",
	# Stage 3.D (Bernoulli-impulse actuation backend)
	"actuation_backend", "bri_base_rate", "bri_command_bias",
	"bri_impulse_per_spike", "bri_friction_per_tick",
	# Stage 3.E (joint compliance via motor authority scale + damping for springiness)
	"motor_authority_scale", "motor_damping_factor",
	# Stage 3.E++ (motor mechanical-slop / servo-saver free-play zone)
	"motor_freeplay_rad",
	# 2026-06-01 knee-widening A/B toggle
	"knee_widening_enabled",
	# 2026-06-08 — runtime knobs for body-level CruseCoordinator gain + target mode.
	# cruse_bias_gain_knee: 2026-06-08 per-joint-kind gain (0.0 = no knee bias).
	# cruse_bias_gain: stage 1 silences Cruse for clean standing learning;
	#                  stage 2 restores it for walking coordination.
	# target_mode:      curriculum stages can flip random_pyramid on/off without
	#                   the stage-level target_mode field (or in addition to it).
	"cruse_bias_gain",
	"cruse_bias_gain_knee",
	"cruse_bias_gain_hip1",
	"cruse_bias_gain_hip2",
	# Move 5 saturation gate (position-aware bias suppression)
	"saturation_gate_enabled",
	"saturation_zone_min",
	"saturation_zone_max",
	"target_mode",
	# 2026-06-10 — E0/E1 scripted-gait diagnostic (oscillator ladder).  Lets a
	# curriculum stage turn on the open-loop gait drive + set its waveform so the
	# UI can observe it via the curriculum loader (no env vars needed).
	"_scripted_gait",
	"_sg_amp_hip1", "_sg_amp_hip2", "_sg_amp_knee",
	"_sg_period", "_sg_hip1_phase", "_sg_offsets",
	# 2026-06-10 E1b — light front-rear posture coupling strength.
	"leg_symmetry_fr_blend",
	# 2026-06-11 (a) — per-step homeokinetic reward gain.
	"homeo_step_gain",
	"homeo_payout_norm",
	# 2026-06-11 (c) — cross-leg phase-consistency coupling (reward-space).
	"homeo_phase_couple",
]

func _load_brain_snapshot_from(path: String) -> void:
	# Curriculum stage-field "brain_snapshot_load" handler.  Fires once per run.
	if path == "" or _curric_snapshot_loaded or brain == null:
		return
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		push_error("PicrawlerBody: curriculum brain_snapshot_load — cannot open " + path)
		return
	var snap_text: String = f.get_as_text()
	f.close()
	brain.restore_state(snap_text)
	_curric_snapshot_loaded = true
	# Settle window so the body re-stabilizes under the restored brain before
	# the scripted gait perturbs it (same as the env-var load path).
	_yaw_probe_settle_offset_ticks = YAW_PROBE_LOAD_SETTLE_TICKS
	print("PicrawlerBody: curriculum restored BRAIN_SNAPSHOT from %s (%d bytes)" % [path, snap_text.length()])

func _apply_curriculum_overrides(overrides: Dictionary) -> void:
	if overrides.is_empty():
		return
	var applied: Array = []
	var spring_touched: bool = false
	for k in overrides.keys():
		var key: String = str(k)
		if not (key in _CURRICULUM_ALLOWED_KEYS):
			push_warning("CurriculumManager: '%s' is not in the curriculum whitelist — skipped" % key)
			continue
		set(key, overrides[k])
		applied.append("%s=%s" % [key, str(overrides[k])])
		if key.ends_with("_spring_stiffness") or key.ends_with("_spring_damping"):
			spring_touched = true
	if not applied.is_empty():
		print("PicrawlerBody: curriculum applied → %s" % ", ".join(applied))
	# Re-apply joint springs if any spring knob was touched.  The @export
	# vars are now updated; this propagates them to all 12 G6DOF joints
	# without rebuilding the body.
	if spring_touched and not _hip1_joints.is_empty():
		_apply_joint_springs()
		print("PicrawlerBody: joint springs re-applied (hip1=%.2f/%.2f hip2=%.2f/%.2f knee=%.2f/%.2f)" % [
			hip1_spring_stiffness, hip1_spring_damping,
			hip2_spring_stiffness, hip2_spring_damping,
			knee_spring_stiffness, knee_spring_damping])

func _on_curriculum_stage_changed(_idx: int, name_: String, overrides: Dictionary) -> void:
	var stage_dict: Dictionary = CurriculumManager.current_stage()
	# Curriculum-driven snapshot load (UI loader + stage transitions).
	_load_brain_snapshot_from(str(stage_dict.get("brain_snapshot_load", "")))
	var skip_reset: bool = bool(stage_dict.get("skip_reset_on_entry", false))
	print("PicrawlerBody: curriculum stage → %s (#%d)%s" % [name_, _idx,
		"  [skip_reset_on_entry]" if skip_reset else ""])
	# 2026-06-06 — brain snapshot capture on first advance to stage ≥1.
	# Joseph's "pre-train standing once, probe many times" workflow: when the
	# canonical 2-stage stand-only curriculum auto-advances from stage 0 →
	# stage 1, the brain has demonstrated 24+ sim-sec sustained standing
	# (best_cumulative_alive_ticks ≥ 1200) — save that brain state.  Fires
	# only once per run; subsequent stage changes are ignored.
	if _brain_snapshot_save_path != "" and not _brain_snapshot_save_done and _idx >= 1:
		var snap_text: String = brain.snapshot_state()
		var f := FileAccess.open(_brain_snapshot_save_path, FileAccess.WRITE)
		if f == null:
			push_error("PicrawlerBody: cannot open SNAPSHOT_SAVE path " + _brain_snapshot_save_path)
		else:
			f.store_string(snap_text)
			f.close()
			_brain_snapshot_save_done = true
			print("PicrawlerBody: BRAIN_SNAPSHOT saved → %s (%d bytes, tick=%d)" % [
				_brain_snapshot_save_path, snap_text.length(), tick_counter])
	_apply_curriculum_overrides(overrides)
	# Phase 7.x — walk_over_there: pick a random pyramid target when
	# the stage spec asks for it; clear the target otherwise so other
	# reward modes don't fight a leftover purple highlight.  Per-stage
	# target_mode + visit_radius live in the stage dict directly (same
	# pattern as skip_reset_on_entry) since they're stage-control
	# fields, not body @export overrides.
	# 2026-06-11 — overrides-level fallback.  target_mode is ALSO in the
	# @export whitelist, so curricula commonly set it inside "overrides".
	# This block previously read ONLY the stage level and force-reset the
	# body field to "off" when absent — silently clearing the pyramid
	# target at every stage transition.  Every whr-probe stage 2 ran
	# target-less because of this (to_target hit/miss streams inert);
	# curriculum-driven to_target runs that set target_mode in overrides
	# are suspect for the same confound.  Now: stage-level key wins if
	# present; else the overrides-applied body field stands.  Stages that
	# want to clear a target must say target_mode "off" explicitly.
	var stage_target_mode: String = str(stage_dict.get("target_mode", ""))
	if stage_target_mode == "":
		stage_target_mode = target_mode   # set by _apply_curriculum_overrides above
	target_mode = stage_target_mode if stage_target_mode != "" else "off"
	if stage_target_mode == "random_pyramid":
		walk_visit_radius = float(stage_dict.get("visit_radius", walk_visit_radius))
		# Seed RNG from body's resolved seed XOR stage idx so paired-
		# seed A/Bs reproduce target sequences.
		_walk_target_rng.seed = (_resolved_seed if _resolved_seed >= 0 else 42) ^ (0xA17D60 + _idx)
		_select_random_pyramid_target()
	elif walk_target_idx >= 0:
		_clear_pyramid_target()
	# Bootstrap-preload: if the stage spec includes a resume_state_path,
	# load that saved brain state into the current brain.  Lets the
	# curriculum bake in "skip cold-start standing" as a stage entry
	# condition without command-line orchestration.  Each preload key
	# fires at most once per session — repeated transitions back into
	# the same preloaded stage don't reload (preserves whatever
	# learning happened since the first entry).
	var stage: Dictionary = CurriculumManager.current_stage()
	var resume_spec: String = str(stage.get("resume_state_path", ""))
	if resume_spec != "" and not _curriculum_resume_done.has(resume_spec):
		var resolved: String = _resolve_resume_state_path(resume_spec)
		if resolved != "" and _load_brain_state(resolved):
			print("PicrawlerBody: stage '%s' preloaded brain state ← %s" % [name_, resolved])
			_curriculum_resume_done[resume_spec] = true
		else:
			push_warning("PicrawlerBody: stage '%s' resume_state_path='%s' → resolved='%s' — load failed" % [name_, resume_spec, resolved])
	# Re-centre the body on every stage transition.  The walking stages
	# use radial-outward velocity (chassis_velocity_xz · r̂) — starting
	# from origin gives that signal a clean baseline.  Standing stages
	# benefit too: any wobble-induced drift from the previous stage
	# doesn't pre-bias the body's starting position for the next.
	# Brain weights + session PBs (best_cumulative_alive_ticks,
	# max_distance_from_origin) are unaffected — only the body's pose
	# and episode counters reset, same as SPACE.  The initial scene-load
	# stage_changed(0, …) is filtered out by signal-timing: the body
	# connects to stage_changed AFTER _ready's load_curriculum_file has
	# already fired the initial emit.  Reaching _idx=0 post-connect
	# means the experimenter pressed Prev or the →0 reset — both of
	# which need the re-centre.
	#
	# Phase 7.x — stages can opt OUT of the re-centre with
	# skip_reset_on_entry=true in their stage spec.  Used when a stage
	# transition shouldn't disturb continuity: e.g., walk_far →
	# walk_fast happens when the body has just reached the edge of the
	# rings heading outward to the pyramid donut.  Resetting to origin
	# would erase the committed heading and force a fresh start; the
	# walk_fast stage just swaps the reward mode and lets motion
	# continue.
	if not skip_reset:
		_pending_manual_reset = true

func publish_trainer_event(kind: String, intensity: float) -> void:
	## Sole entry point for live trainer pulses (Good Boy / Bad Boy
	## buttons).  Forwards the pulse to the brain and increments the
	## per-kind cumulative counter so _emit_jsonl can log the audit
	## trail.  Methodology rule: anything outside this method that
	## bypasses the counter is a methodology bug — there must be no
	## other code path that fires events.hit/miss with origin "trainer".
	if brain == null:
		return
	var clamped: float = clamp(intensity, 0.0, 1.0)
	if kind == "good":
		brain.publish_event("hit", clamped)
		_trainer_good_count += 1
	elif kind == "bad":
		brain.publish_event("miss", clamped)
		_trainer_bad_count  += 1
	else:
		push_warning("publish_trainer_event: unknown kind %s" % kind)
