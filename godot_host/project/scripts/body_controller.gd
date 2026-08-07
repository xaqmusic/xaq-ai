extends CharacterBody3D
## Body controller for the Cell environment.
## Bridges physics sensors to OgmaBrain each physics tick.

# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------
@export var config_path: String = "res://addons/ami_ogma/configs/the_cell.json"
@export var move_speed: float   = 3.0              # reference scale for stuck-detector + IMU normalization
@export var gravity: float      = 9.8
@export var whisker_length: float = 1.5
@export var max_steer_rate: float = 1.5            # legacy; superseded by spike dynamics in actuation
@export var diag_interval_ticks: int = 60          # 0 = silent

# --- Phase 6.0.a: two-flagellum spiking actuation -----------------------------
# The brain emits a scalar steer; the body splits it into per-flagellum
# Bernoulli spike rates, samples a discrete impulse per spike, and integrates
# linear/angular velocity with first-order friction.  Body becomes a physical
# integrator of discrete impulses; proprioception reflects body dynamics, not
# controller smoothing.
#
# Calibration target (matches the previous continuous controller):
#   steady-state forward speed at steer=0   ≈ move_speed (3.0 m/s)
#   steady-state turn rate at full asymmetry ≈ max_steer_rate (1.5 rad/s)
@export var flagellum_base_rate:     float = 0.5   # baseline spike prob per tick per flagellum
@export var flagellum_steer_bias:    float = 0.5   # how much steer asymmetrises rates
@export var flagellum_impulse_lin:   float = 0.15  # m/s added to forward speed per spike
@export var flagellum_impulse_ang:   float = 0.15  # rad/s added per asymmetric spike
@export var friction_lin_per_tick:   float = 0.05  # fractional linear decay per physics tick
@export var friction_ang_per_tick:   float = 0.10  # fractional angular decay per physics tick

# Phase 6.5.6 option A — refractory period.  After a paddle fires, that
# paddle's spike rate is suppressed (forced to 0) for N ticks.  Models
# motor-neuron biological refractory.  Side effects:
#   - Lowers average firing rate by ~1/(1+ticks), creating natural rest
#     periods between firings without changing the steer-rate mapping.
#   - Chunks captured under refractory have implicit rest ticks baked
#     into their action_sequence (action_history accumulates a steady
#     accel even when the body isn't firing — replays produce the same
#     rest pattern via the same suppression at replay time).
#   - At ticks=0 (default), behaviour is identical to pre-6.5.6 sampling
#     so any harness that doesn't set OGMA_REFRACTORY_TICKS sees no
#     change.
@export var flagellum_refractory_ticks: int = 0

# Phase 6.8 — per-paddle leaky energy budget (opt-in via OGMA_MOTOR_ENERGY=1).
# Each flagellum has an energy reservoir in [0,1]: paddling drains it
# (cost × effective rate), resting recharges it (constant trickle toward 1).
# Output thrust is gated by available energy, so sustained firing (e.g. pushing
# into a wall) runs the paddle out → forced coast/drift → recovery: a
# relaxation rhythm of beat/coast/pause.  Published as reality.proprio.motor_energy
# so a Motor-EPM can sense it and learn to PACE its beating (conserve).  Per-paddle,
# so asymmetric depletion yields emergent turning.  Default OFF — existing cell
# configs (reactive baseline, prior experiments) are unchanged.
@export var motor_energy_enabled: bool = false
@export var motor_energy_cost: float = 0.02       # drain per tick at full rate
@export var motor_energy_recharge: float = 0.01   # recharge per tick toward 1.0
# Hysteresis relaxation oscillator: a paddle beats until energy drops below
# motor_energy_low (→ exhausted: full rest, rate 0, coast/drift), then recovers
# at rest until energy exceeds motor_energy_high (→ beat again).  This gives a
# clean beat-coast-pause rhythm instead of a low-energy weak-beating equilibrium.
@export var motor_energy_low:  float = 0.15
@export var motor_energy_high: float = 0.85
var _motor_energy_left: float = 1.0
var _motor_energy_right: float = 1.0
var _motor_exhausted_left:  bool = false
var _motor_exhausted_right: bool = false

# Phase 6.8 — baseline-beat actuation (opt-in via OGMA_MOTOR_BASELINE_BEAT=1).
# Maps the brain's bilateral output to a CENTERED spike rate (output 0 → 0.5
# baseline beat) instead of the half-wave clamp.  Grounds a homeokinetic
# controller's linear forward model (no dead zone → no phantom-TLE freeze).
# Only meaningful in modular-passive bilateral mode; default OFF so the reactive
# baseline (ChemotaxisAI) and prior configs keep the half-wave mapping.
@export var motor_baseline_beat: bool = false

# Phase 6.5.6 body-model selector.  Two spike-to-impulse mappings:
#   "asymmetric_paddler"  (default; current pre-6.5.6 behaviour)
#       single spike → forward + rotation (forward thrust always coupled
#       to rotation impulse).  Both spikes → forward only.  No way to
#       rotate without translating; turn capability bounded by forward
#       drift.  Matches bacterial chemotaxis tumble/run paradigm.
#
#   "differential_paddler"  (Phase 6.5.6 candidate; user's prior-work
#       inner-tube model)
#       single spike → pure rotation (no forward).  Both spikes (same
#       tick, synchronous) → forward synergy bonus, rotations cancel.
#       Sign convention matches user's mental model: left spike alone
#       → CW (right turn), right spike alone → CCW (left turn).
#       Allows in-place rotation, decouples turn capability from
#       forward speed.
#
# Env var override: OGMA_BODY_MODEL.  Default preserves current
# behaviour so any harness that doesn't set it sees no change.
@export var body_model: String = "asymmetric_paddler"

# ---------------------------------------------------------------------------
# Nodes + state
# ---------------------------------------------------------------------------
@onready var brain: OgmaBrain = $Brain
var whiskers: Array[RayCast3D] = []
# Phase 6.2 — 4 downward raycasts at body-relative offsets for the terrain
# probe.  Order: +X, -X, +Z, -Z (right, left, front, back).  Built in _ready.
var terrain_probes: Array[RayCast3D] = []
const _TERRAIN_PROBE_OFFSET := 0.6   # metres from body centre (matches NOSTRIL_RADIUS)
const _TERRAIN_PROBE_LENGTH := 5.0   # ray length downward (room is 4 m tall)

var heading: float = 0.0
var energy:  float = 1.0
var hits_total: int = 0
var _ticks_since_food: int = 0   # Phase 6.9 — survival telemetry (ticks since last food)
var _energy_drain_per_sec: float = 0.012   # Phase 6.9 — passive drain (OGMA_ENERGY_DRAIN)
var lock_heading: bool = false             # 2026-06-19 corridor 1-D probe: zero yaw (forward/reverse only)
@export var eat_forward_min: float = 0.5   # 2026-06-21 head-on gate: min forward speed to score an eat (0 = off)
var _corridor_start: Vector3 = Vector3(0, 1, 0)   # trial reset pose (set on first physics tick)
var _corridor_start_set: bool = false
# Runtime tag stamped into every JSONL diag line ("rt") + a startup RUNTIME event,
# so a log can never again be mistaken for the other runtime: --headless writes
# the SAME user://logs/godot.log as the UI, so headless test runs and UI runs land
# in one file.  "headless" = DisplayServer headless (my test runs); "ui" = rendered.
var _runtime_tag: String = "ui"
# --- UI-only top-down PLACE-MAP debug overlay (KEY_J) -------------------------------
# Visualises the PlaceGraphPlanner's path-integration place-map against the bug's REAL
# trajectory, to diagnose why the planner can route a foraging bug AWAY from food:
#   1) breadcrumb of the bug's real (x,z) coloured by the planner's current place-node
#      (cnode) — a healthy PI shows each colour as ONE tight blob; drift smears it;
#   2) a plan-steering arrow in the planner's chosen world direction (toward/away food);
#   3) fixed food markers.
# All geometry sits on visual layer 3 (1<<2) so ONLY the top-down cam renders it (never
# the FPV).  Default OFF; zero work in headless (never built).  Read-only consumption of
# existing brain metrics + the env's active_food_world_positions(); no logic changes.
var _dbg_overlay: Node3D = null               # parent of all debug meshes (in env space)
var _dbg_overlay_visible: bool = false
var _dbg_crumb_lines: MeshInstance3D = null   # arrow + food crosses (ImmediateMesh)
var _dbg_crumbs: Array = []                    # ring-buffer of crumb MeshInstance3D
var _dbg_crumb_mesh: SphereMesh = null         # shared mesh for crumb markers
var _dbg_crumb_pos: int = 0                    # ring write cursor
const _DBG_CRUMB_MAX: int = 600
const _DBG_CRUMB_EVERY: int = 10               # sample every N physics ticks
const _DBG_Y: float = 0.2                      # draw height above floor
var _dbg_first_cnode_logged: bool = false      # one-time print on first valid cnode
# Phase 6.9.B — vision green-saliency emphasis.  Food is the only strongly GREEN
# thing (greenness = G-(R+B)/2 ~191 for food vs <=20 for wall/floor/sky), but it
# occupies few pixels so the JL random projection barely moves when food enters
# view (low embedding contrast).  This suppresses NON-green pixels by this factor
# so the (few) food pixels DOMINATE the frame vector -> food "looms" in the vision
# embedding.  0 = off (raw frame); ~0.85 = background dimmed to 15%, food full.
var _vision_green_gain: float = 0.0   # OGMA_VISION_GREEN_GAIN / metadata vision_green_gain
var tick_counter: int = 0
# v5.4.L Diagnostic A — raw-signal pre/post-eat capture.
var _last_green_frac: float = 0.0
var _pending_hit_samples: Array = []   # [{target_tick, label, hit_id}, ...]
var last_diag_pos: Vector3 = Vector3.ZERO
var _last_scent: PackedFloat64Array = PackedFloat64Array()
var _last_scent_max: float = 0.0   # heading-INVARIANT centre scent (published scent_max; NOT the rotating nostril-ring max)
# The directional 8-nostril ring (reality.proprio.scent) is a SCAFFOLD / different sensory
# modality (a spatial gradient array that ROTATES with the body), NOT the position-only chemical
# bath the run-and-tumble chemotaxis uses. Default ON for legacy ScentCompass/Klinotaxis/bearing
# configs; metadata publish_scent_ring=false fully bypasses it (defensible position-only sensing).
var _publish_scent_ring: bool = true
var _last_whisker_max: float = 0.0

# Phase v5.1 — Monte-Carlo actor-critic episode boundary.  When
# OGMA_EPISODE_LENGTH > 0, the body publishes events.episode_end every
# N physics ticks.  Premotor's `mc_lr > 0` mode listens for this event
# and finalises the trajectory (computes G_t, applies the per-step
# Hebbian-shaped update, clears the buffer).  Default 0 = no episode
# events (legacy v4 behaviour; Premotor's mc_lr should also be 0).
var _episode_length_ticks: int = 0
var _episode_tick_count: int   = 0
# D-value scent-cut hook (env-driven, reproducible across resets)
var _global_tick: int          = 0
var _scent_cut_init: bool       = false
var _scent_cut_at_tick: int     = -1
var _scent_cut_to: float        = 0.0
var _scent_cut_done: bool       = false

# Turbo: when OGMA_QUIT_AFTER_TICKS > 0, the body calls get_tree().quit()
# once tick_counter reaches the budget so a wall-decoupled run exits
# naturally instead of running until the harness timeout.  0 = disabled.
var _quit_after_ticks: int = 0

# Phase 6.5.16 — body-side dual-EMA + alignment reward (Cell variant
# of the MC Phase 6.5.8/9 design).  Densifies the otherwise-sparse
# reward signal in continuous Cell sessions:
#   (1) events.hit when short-EMA(scent_max) > long-EMA(scent_max) × 1.5
#       AND cart is moving (speed > floor).  Rewards "actively
#       heading toward food."
#   (2) events.miss when max_whisker > 0.30.  Fires earlier than the
#       existing wall_stuck threshold (0.55) so the brain gets a
#       negative reward signal on light contact, not just severe
#       stuck.  events.miss drains da+ht (when event_coupled_da/ht
#       are true, which they are post Phase 6.5.15 audit).
var _scent_short_ema: float = 0.0
var _scent_long_ema:  float = 0.0
const _SCENT_SHORT_ALPHA: float = 0.1     # ~10-tick window
const _SCENT_LONG_ALPHA:  float = 0.001   # ~17s @ 60 Hz
const _SCENT_HIT_THRESHOLD: float = 1.5   # short > long × 1.5 → hit (legacy fallback)
const _SCENT_HIT_MOTION_FLOOR: float = 0.5  # m/s; only fire if cart actually moving
const _WHISKER_MISS_THRESHOLD: float = 0.30  # legacy fallback

# Phase 6.5.19 — statistical adaptive reward thresholds (Tier 2).
# Replace the fixed 1.5× scent and 0.30 whisker constants with
# substrate-derived versions tracking running mean + stddev of the
# relevant signal.  Threshold = mean + N × stddev (N=2 → top ~2.3%
# of natural variance).  Same EMA-based statistics primitive the
# substrate already uses internally (NeurochemState's
# da_baseline_ema, kinesis change_ema_alpha).
#
# Settle-time guard: the EMAs need ~1000 ticks to converge from
# zero-init.  Until then, fall back to the legacy fixed thresholds
# above to prevent spurious early firing (the failure mode that hit
# MC Phase 6.5.10 when bootstrap was removed).
#
# Watchdog: if the adaptive scent threshold produces zero hits for
# WATCHDOG ticks, fall back to fixed for that seed (recovers if
# stddev runs away).
var _scent_diff_mean_ema: float = 0.0
var _scent_diff_var_ema:  float = 0.0
var _whisker_mean_ema:    float = 0.0
var _whisker_var_ema:     float = 0.0
const _ADAPT_ALPHA: float = 0.001    # ~17s tracking
const _ADAPT_N_STDDEV: float = 2.0   # threshold = mean + N×stddev
const _ADAPT_WARMUP_TICKS: int = 1000
const _ADAPT_MIN_STDDEV: float = 1e-6  # NaN/division guard
const _ADAPT_HIT_WATCHDOG: int = 5000  # ticks without hit → fallback for seed
var _ticks_since_last_scent_hit: int = 0
var _adaptive_fallback_active: bool = false   # latched ON if watchdog fires

# Phase 6.5.19 Part A — stuck-baseline diagnosis counters.  Track
# WHY the cart is stuck (deficit > 0.5) so we can identify the
# dominant cause empirically rather than guessing.
var _stuck_total_ticks: int = 0
var _stuck_wall_ticks: int = 0          # also max_w > 0.30
var _stuck_refrac_ticks: int = 0        # both flagella refractory
var _stuck_zero_steer_ticks: int = 0    # |steer_brain| < 0.1
# Phase 6.5.20 — honest pinning ticks (deficit-vs-body-max > 0.5).
# Separate from _stuck_total_ticks (which uses move_speed reference and
# fires during normal motion in differential_paddler).
var _stuck_actual_pin_ticks: int = 0
var _speed_sum_for_avg: float = 0.0
var _angular_sum_for_avg: float = 0.0
var _avg_count: int = 0
var _spike_count_total: int = 0
# 2026-06-30 — metabolic energy model (operator): energy depletes per unit of
# MOTOR EFFORT spent this tick, not on a wall-clock timer, so it falls faster
# when the bug drives hard and not at all when it coasts.  Body-model-agnostic:
# spike-driven models (differential/asymmetric) meter by spike count (0..2);
# the bidirectional paddler meters by command magnitude |al|+|ar| (normalised).
# Set in the motor-apply branch each tick; consumed by the energy drain (one-tick
# lag, negligible).  energy_drain (config) keeps its per-second meaning at unit
# effort (the drain term is energy_drain · effort · delta).
var _motor_effort_last: float = 0.0
# Honest locomotion diagnostics (2026-07-08 instrument audit).  The run
# snapshot's "Stuck %", "Mean fwd speed (target 3.0)" and "Spikes/tick" were
# all miscalibrated for the continuous bidirectional_paddler (spikes decoupled
# from thrust; move_speed=3.0 is the asymmetric-paddler cruise, not this body's;
# stuck measured vs that unreachable reference flagged normal slow/pausing
# motion).  These accumulators are body-model-agnostic — they read the ACTUAL
# commanded effort and the body's own velocity integrator.
var _motor_effort_sum: float = 0.0        # Σ _motor_effort_last → mean commanded effort ∈[0,2]
var _pause_ticks: int = 0                 # ticks with effort < _PAUSE_EFFORT (commanded coast)
var _fwdspeed_history: Array = []         # parallel to _pos_history → efference-matched pinning
# Refractory between miss-events to prevent saturation when the cart
# sits in continuous wall contact for many ticks.  Once a miss fires,
# wait this many ticks before another can fire from the same condition.
var _miss_refractory: int = 0
const _MISS_REFRACTORY_TICKS: int = 30

var _mouth: Area3D = null                 # front eat-zone; polled per tick for overlap
const _EAT_COOLDOWN_TICKS: int = 6        # debounce stale-overlap re-eats after reposition
var _last_eat_tick: int = -1000
# --- Spike actuation state ---------------------------------------------------
var _forward_speed:    float = 0.0        # integrated linear velocity magnitude (m/s)
var _angular_velocity: float = 0.0        # integrated angular velocity around y (rad/s)
var _flagellum_rng:    RandomNumberGenerator = RandomNumberGenerator.new()
var _last_spike_left:  bool  = false
var _last_spike_right: bool  = false
# Refractory countdowns — when > 0 the corresponding paddle's effective
# spike rate is forced to 0 this tick.  Decrement before the spike test;
# set after a successful spike.
var _refractory_left:  int   = 0
var _refractory_right: int   = 0

# Phase 6.5.3 instrumentation — chunk dispatch event tracking.  Track
# the brain's active chunk id per tick; on rising edge (transition from
# -1 to a real chunk id) emit a CHUNK_DISPATCH event line capturing
# perceptual context (whisker max, scent max, heading) at the moment
# the chunk was triggered.  cell_smoke.py post-processes these events
# to test the "whisker contact → food navigation" causal-chain
# hypothesis: how often does a whisker-active state precede a chunk
# dispatch, and how often does that dispatch lead to a hit within K
# ticks?
var _last_chunk_id: int = -1

# --- Stuck detector (homeokinetic — no hand-tuned thresholds) -----------------
# Body keeps a 1-second ring of past positions and compares actual
# displacement to the body's *mechanical maximum* over that interval.  The
# only constants here are physical: 60 Hz tick rate, move_speed.  No
# behavioral thresholds.
#
#   deficit = 1 - actual_displacement / (move_speed * window_ticks * delta)
#
# Free motion ⇒ deficit ≈ 0.  Wedged ⇒ deficit ≈ 1.  Wall collisions are
# left to Godot's move_and_slide; deficit-driven rotation injection breaks
# wedges by spinning the body until it faces open space.
const _STUCK_WINDOW_TICKS: int = 60       # 1 s at 60 Hz — physical horizon
const _STUCK_PULSE_TICKS:  int = 30       # ~0.5 s held pulse — resample interval
const _PAUSE_EFFORT: float = 0.05         # |L|+|R| below this = commanded coast, not driving
const _PIN_MIN_INTENDED: float = 0.03     # min commanded path (m) over window to judge pinning
var _pos_history: Array = []              # ring of last N global_positions
var _prev_imu_pos: Vector3 = Vector3.ZERO # last tick's position, for AFFERENT velocity
var _imu_pos_init: bool = false
var _stuck_rng: RandomNumberGenerator = RandomNumberGenerator.new()
var _stuck_severity: float = 0.0          # 0..1, surfaced to HUD/diagnostics
# Held rotation pulse: a per-tick random walk produces only diffusive heading
# changes (≈ 11°/s at deficit=1) that don't reliably escape corners.  We
# resample a *direction* of rotation every _STUCK_PULSE_TICKS and hold it,
# giving ballistic rotation: at deficit=1, heading sweeps a full circle in
# ~4 s.  Resample is also triggered when deficit transitions through 0.5
# (engaging) so the body commits to a direction immediately on getting stuck.
var _stuck_pulse_held:    float = 0.0
var _stuck_pulse_ticks:   int   = 0
var _prev_deficit:        float = 0.0
# Phase 6.5.20 — accurate pinning metric (vs body model max forward,
# not move_speed).  Diagnostic-only; does NOT gate the stuck-pulse
# (which still uses deficit-vs-move_speed because the brain depends
# on the resulting noise floor for exploration).
var _stuck_severity_actual: float = 0.0

# Phase 6.5.23 — vision pipeline (geometric raycast capture).
# Godot's --headless mode disables the rendering server entirely, so a
# SubViewport-with-texture-readback approach returns null pixels.  Use
# the same primitive the whisker sensors already use: cast a grid of
# rays from the FPV camera's pose into the scene via PhysicsServer3D
# and assign each pixel the colour of whatever the ray hits.  No GPU,
# fully deterministic, headless-safe.  Bonus: the encoder receives
# pure scene-content pixels (no shading / lighting noise / aliasing),
# matching the spirit of the rest of the geometric Ogma stack.
#
# Resolution = JL "color" encoder's native input (24×24×3) so the
# encoder's internal resize is a no-op.  Other envs / encoders can
# bump _vis_res to feed richer encoders without changing this code.
# 2026-06-20 — now a VAR (metadata.vis_res / OGMA_VIS_RES), default 24.
# EVEN grids (24) have NO dead-centre ray → a small dead-ahead food falls in
# the ±(1/N) straddle gap (caught by 0 or 2 px, never 1) and is invisible
# beyond ~14m.  ODD grids (25) place a ray at exactly u=0 → dead-ahead food
# always registers.  Configs WITH a vision EPM must keep vis_res = the JL
# encoder's native res (24); the corridor/nav configs (no vision EPM) are free
# to use 25 to A/B the odd-grid green_fraction against the forward fovea.
var _vis_res: int = 24
# Capture every tick.  Required so the LateralVoter sees a fresh
# RealityToken on reality.video.color each tick — sub-rate publishes get
# filtered out via the winner_id<0 bootstrap-token check, which makes the
# vision modality drop out of the trust map and lose its seat at fusion.
const _VIS_CAPTURE_EVERY: int = 1

# Object-class colour palette — matches the_cell_world.gd albedo values.
# Used by the raycast capture path to assign RGB by hit-collider name.
# Latest raycast frame, cached so the HUD can render it as a "brain view"
# debug panel without re-running the raycast.  Empty until the first
# capture; HUD is responsible for handling the bootstrap case.
var _last_vis_pixels: PackedByteArray = PackedByteArray()

# 2026-06-20 — FORWARD-LOOM FOVEA.  A small ODD ray cluster over a NARROW FOV
# centred dead-ahead (u=0 ray guaranteed) — a high-acuity "fovea" decoupled
# from the wide 24×24 peripheral frame.  Emits a CONTINUOUS angular-size loom
# = food_radius / nearest-fovea-hit-distance (smooth, monotonic in closeness,
# no quantization, gap-free), facing-selective (only rises when food is dead
# ahead).  Published as reality.proprio.forward_loom for the cog's near-field
# (mouth-aware) objective, in place of the flickery frame-derived green_fraction.
@export var fovea_res:     int   = 5      # odd → centre ray at u=0 (5×5 = 25 rays)
@export var fovea_fov_deg: float = 24.0   # narrow total FOV (±12°) → fine centre acuity
var _last_forward_loom: float = 0.0

var _CLR_FOOD:    PackedByteArray = PackedByteArray([26, 230, 51])    # 0.1, 0.9, 0.2
var _CLR_WALL:    PackedByteArray = PackedByteArray([191, 184, 173])  # 0.75, 0.72, 0.68
var _CLR_FLOOR:   PackedByteArray = PackedByteArray([140, 133, 128])  # 0.55, 0.52, 0.50
var _CLR_PILLAR:  PackedByteArray = PackedByteArray([89, 77, 102])    # 0.35, 0.30, 0.40
var _CLR_TERRAIN: PackedByteArray = PackedByteArray([115, 128, 102])  # 0.45, 0.50, 0.40
var _CLR_SKY:     PackedByteArray = PackedByteArray([20, 20, 41])     # background_color

# 2026-06-23 — ZONE-COLOURED WALLS (perceptual distinctiveness for place-coding).
# "Twisty passages all alike" aliases every place onto one map node; colouring each
# world-grid zone's walls/pillars a distinct hue gives the vision EPM a place signature.
# Opt-in (metadata.zone_colors); default off → uniform _CLR_WALL (byte-identical).
# Palette is deliberately NON-food-green (food test = R<128 AND B<128 AND G>128) so a
# coloured wall is never mistaken for food, and distinct enough to separate cleanly.
var _zone_colors: bool = false
var _zone_grid:   int  = 4    # NxN zones across the room
var _room_size:   float = 24.0  # cached from metadata (the helper runs outside _ready scope)
var _last_vis_mean_rgb: Array = [0.0, 0.0, 0.0]   # coarse FPV appearance descriptor (diag)
var _ZONE_PALETTE: Array = [
	PackedByteArray([210,  45,  45]),   # red
	PackedByteArray([225, 130,  30]),   # orange
	PackedByteArray([215, 205,  45]),   # yellow
	PackedByteArray([ 45, 175, 205]),   # cyan
	PackedByteArray([ 55,  75, 210]),   # blue
	PackedByteArray([155,  45, 195]),   # purple
	PackedByteArray([210,  50, 160]),   # magenta
	PackedByteArray([130,  85,  45]),   # brown
]

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	# Turbo mode (OGMA_TURBO=1): wall-decoupled execution for fast A/B and
	# evolutionary sweeps.  Combined with godot CLI flags
	# `--fixed-fps 60 --disable-render-loop --headless`, the engine advances
	# physics at sim-time-locked 60Hz but never sleeps to wall — throughput
	# becomes CPU-bound.  Raising max_physics_steps_per_frame ensures the
	# engine doesn't drop ticks if a render frame straddles many sim ticks.
	# Substrate time constants (EMAs, decays) are unchanged because
	# physics_ticks_per_second stays at 60.
	if OS.get_environment("OGMA_TURBO") == "1":
		Engine.max_physics_steps_per_frame = 64
		print("BodyController: TURBO mode — max_physics_steps_per_frame=64")
	var quit_env: String = OS.get_environment("OGMA_QUIT_AFTER_TICKS")
	if quit_env != "":
		_quit_after_ticks = quit_env.to_int()
		print("BodyController: OGMA_QUIT_AFTER_TICKS=%d" % _quit_after_ticks)
	# Test-instrumentation hook: finer JSONL cadence for per-tick analyses (default 60).
	var diag_env: String = OS.get_environment("OGMA_DIAG_INTERVAL")
	if diag_env != "":
		diag_interval_ticks = max(1, diag_env.to_int())
		print("BodyController: OGMA_DIAG_INTERVAL=%d" % diag_interval_ticks)

	var cam := get_node_or_null("Camera3D") as Camera3D
	if cam:
		cam.make_current()
		# FPV sees world (layer 1) and ceiling (layer 2); the body mesh on
		# layer 3 is hidden so the camera-inside-capsule isn't a problem.
		cam.cull_mask = (1 << 0) | (1 << 1)
	else:
		push_error("BodyController: Camera3D not found.")

	for i in range(6):
		var rc = get_node_or_null("Whisker%d" % i)
		if rc:
			# Single source of truth for whisker reach: rescale each RayCast to
			# whisker_length (preserving its direction) so the PHYSICAL ray, the
			# VISUAL line (drawn to target_position in _build_body_visual), and the
			# contact NORMALIZATION (1 - dist/whisker_length) all match.  The scene
			# authored them at 3.0 m while sensing normalized at 1.5 m → the
			# top-down whiskers looked longer than what the bug actually feels.
			if rc.target_position.length() > 0.0001:
				rc.target_position = rc.target_position.normalized() * whisker_length
			whiskers.append(rc)
		else:
			push_warning("BodyController: Whisker%d not found." % i)

	# 2026-06-18 — food is a SOLID obstacle again.  Food is a StaticBody3D on
	# layer 2; widen the body's move_and_slide mask to layer 1 (walls) | layer 2
	# (food) so the body PHYSICALLY collides with food and can no longer clip
	# through it.  Eating stays gated to the FRONT mouth collider (mask 2), so the
	# bug must DRIVE INTO food face-first to eat — it can't back into food and
	# score via body clip-through.  Whiskers keep mask 1 (walls only) so the
	# escape reflex never turns the bug AWAY from food.  Eaten food drops to
	# layer 0 (the_cell_world) → becomes pass-through until respawn.
	# 2026-06-28 (operator obs): food is NON-BLOCKING — do NOT mask layer 2. The
	# body-food collision (re-added for "face-first eating") made the bug RAM the
	# static food: physics frame-rate drop + the bug blocked/jittering at the food
	# so the mouth poll missed the overlap → "collides repeatedly, never eats". The
	# FRONT-ONLY mouth (z −0.45..−1.05, mask 2) already gates eating to face-first
	# (a back/side approach moves food OUT of the front box), so the body collision
	# is unnecessary; removing it also restores the 47beae6 headless↔UI determinism.
	# (body collision_mask stays layer 1 = walls only.)

	_build_body_visual()
	_build_terrain_probes()
	_build_vision_viewport()
	_build_mouth()

	if brain == null:
		push_error("BodyController: $Brain not found.")
		return

	# Seed precedence: ExperimentConfig (launcher) > OGMA_SEED env var >
	# time-randomized.  Same value of seed → same rotation noise sequence.
	var resolved_seed: int = ExperimentConfig.resolve_seed()
	var env_seed: String   = OS.get_environment("OGMA_SEED")
	if resolved_seed >= 0:
		_stuck_rng.seed     = resolved_seed ^ 0x73746B
		_flagellum_rng.seed = resolved_seed ^ 0x666C67
	elif env_seed != "":
		_stuck_rng.seed = env_seed.hash() ^ 0x73746B
		_flagellum_rng.seed = env_seed.hash() ^ 0x666C67
	else:
		_stuck_rng.randomize()
		_flagellum_rng.randomize()

	# Phase 6.5.5: starting heading override.  All previous runs spawned
	# at heading=0 → the body always drove +X first → always struck the
	# wall the same way → the same whisker side won contact → the same
	# turn-direction bias accumulated through chunk replay.  Verifying
	# this hypothesis means flipping the starting heading and watching
	# the bias flip with it.  OGMA_BODY_HEADING is degrees CCW from +X.
	# OGMA_BODY_HEADING_RANDOMIZE=1 derives heading from OGMA_SEED so
	# each seed gets a different initial direction (paired-seed-safe).
	# Body model precedence: ExperimentConfig (launcher) > OGMA_BODY_MODEL env >
	# config metadata.body_model > @export default.  The metadata read makes
	# HEADLESS (OGMA_CELL_CONFIG set, not launched) use the config's DECLARED body
	# — without it the body silently fell back to the .tscn default
	# (differential_paddler) while the launcher used the config's bidirectional_
	# paddler, so headless tests ran a DIFFERENT body than the UI (a real trap).
	if ExperimentConfig.launched and ExperimentConfig.body_model != "":
		body_model = ExperimentConfig.body_model
	else:
		var env_bm: String = OS.get_environment("OGMA_BODY_MODEL")
		if env_bm != "":
			body_model = env_bm
		else:
			var _bm_meta: Dictionary = ExperimentConfig.read_cell_config_metadata()
			if _bm_meta.has("body_model") and str(_bm_meta["body_model"]) != "":
				body_model = str(_bm_meta["body_model"])
	print("BodyController: body_model = %s" % body_model)
	_runtime_tag = "headless" if DisplayServer.get_name() == "headless" else "ui"
	print("BodyController: runtime = %s (DisplayServer=%s)" % [_runtime_tag, DisplayServer.get_name()])
	print(JSON.stringify({"event": "RUNTIME", "runtime": _runtime_tag, "display": DisplayServer.get_name()}))
	# UI-only planner debug overlay (KEY_J).  Optional auto-on for a non-interactive
	# UI smoke test: OGMA_PLANNER_OVERLAY=1 enables it at startup (no key press).
	# Never fires in headless (the toggle early-returns there).
	if OS.get_environment("OGMA_PLANNER_OVERLAY") == "1":
		call_deferred("_toggle_planner_overlay")

	# Phase v5.1 — episode length for events.episode_end emission.
	# Precedence: ExperimentConfig (launcher) > OGMA_EPISODE_LENGTH env var.
	# 0 = disabled (default; Premotor stays in event-driven Hebbian mode).
	# >0 = body publishes events.episode_end every N physics ticks so
	# Premotor (mc_lr>0) can finalise its trajectory.  Recommended 3600
	# (60s at 60Hz physics).
	_episode_length_ticks = ExperimentConfig.resolve_episode_length()
	if _episode_length_ticks > 0:
		print("BodyController: episode_length_ticks = %d" % _episode_length_ticks)

	# Refractory ticks: ExperimentConfig > env var > @export default.
	if ExperimentConfig.launched and ExperimentConfig.refractory_ticks >= 0:
		flagellum_refractory_ticks = ExperimentConfig.refractory_ticks
	else:
		var env_rt: String = OS.get_environment("OGMA_REFRACTORY_TICKS")
		if env_rt != "":
			flagellum_refractory_ticks = max(0, env_rt.to_int())
	if flagellum_refractory_ticks > 0:
		print("BodyController: flagellum_refractory_ticks = %d" % flagellum_refractory_ticks)

	# Phase 6.8 — per-paddle energy (opt-in; ExperimentConfig metadata OR env).
	if (ExperimentConfig.launched and ExperimentConfig.motor_energy) \
			or OS.get_environment("OGMA_MOTOR_ENERGY") == "1":
		motor_energy_enabled = true
	var _me_c: String = OS.get_environment("OGMA_MOTOR_ENERGY_COST")
	if _me_c != "": motor_energy_cost = _me_c.to_float()
	var _me_r: String = OS.get_environment("OGMA_MOTOR_ENERGY_RECHARGE")
	if _me_r != "": motor_energy_recharge = _me_r.to_float()
	var _me_lo: String = OS.get_environment("OGMA_MOTOR_ENERGY_LOW")
	if _me_lo != "": motor_energy_low = clampf(_me_lo.to_float(), 0.0, 1.0)
	var _me_hi: String = OS.get_environment("OGMA_MOTOR_ENERGY_HIGH")
	if _me_hi != "": motor_energy_high = clampf(_me_hi.to_float(), 0.0, 1.0)
	if motor_energy_enabled:
		print("BodyController: motor_energy ON (cost=%.3f recharge=%.3f low=%.2f high=%.2f)" % [
			motor_energy_cost, motor_energy_recharge, motor_energy_low, motor_energy_high])
	if (ExperimentConfig.launched and ExperimentConfig.motor_baseline_beat) \
			or OS.get_environment("OGMA_MOTOR_BASELINE_BEAT") == "1":
		motor_baseline_beat = true
		print("BodyController: motor_baseline_beat ON (centered spike rate)")
	# Config-metadata default (config-self-contained; env overrides for sweeps).
	var _cell_meta: Dictionary = ExperimentConfig.read_cell_config_metadata()
	# Scent RING scaffold gate (see _publish_scent_ring): position-only chemotaxis configs
	# set publish_scent_ring=false to fully bypass the directional ring for defensibility.
	if _cell_meta.has("publish_scent_ring"):
		_publish_scent_ring = bool(_cell_meta["publish_scent_ring"])
	# QUAD (or any maze): spawn the bug at a metadata position (R-fractions of room_size)
	# instead of the scene origin — the quad's origin is the WALLED cross centre, so the bug
	# must start inside Zone 1 (SW). Default: keep the scene position.
	if _cell_meta.has("spawn_pos"):
		var _rs: float = float(_cell_meta.get("room_size", 16.0))
		var _sp = _cell_meta["spawn_pos"]
		global_position = Vector3(float(_sp[0]) * _rs, global_position.y, float(_sp[1]) * _rs)
		print("BodyController: spawn_pos = (%.2f, %.2f)" % [global_position.x, global_position.z])
	if _cell_meta.has("energy_drain"):
		_energy_drain_per_sec = maxf(0.0, float(_cell_meta["energy_drain"]))
	if ExperimentConfig.launched and ExperimentConfig.cell_energy_drain >= 0.0:
		_energy_drain_per_sec = ExperimentConfig.cell_energy_drain
	var _ed: String = OS.get_environment("OGMA_ENERGY_DRAIN")
	if _ed != "":
		_energy_drain_per_sec = maxf(0.0, _ed.to_float())
	print("BodyController: energy_drain_per_sec = %.4f" % _energy_drain_per_sec)
	# 2026-06-19 — CORRIDOR 1-D probe: lock heading (no yaw) so the bug can ONLY
	# move along its initial forward axis (toward/away from a single food ahead).
	# Isolates temporal credit assignment (sequence forwards) from steering.
	if _cell_meta.has("corridor"):
		lock_heading = bool(_cell_meta["corridor"])
	if OS.get_environment("OGMA_CORRIDOR") == "1":
		lock_heading = true
	# TURN rig: when food is at a random bearing, heading must be FREE (the bug
	# learns to steer toward it).  Only lock for the pure 1-D dead-ahead corridor.
	var _cbm: float = 0.0
	if _cell_meta.has("corridor_bearing_max"):
		_cbm = float(_cell_meta["corridor_bearing_max"])
	var _cbe: String = OS.get_environment("OGMA_CORRIDOR_BEARING")
	if _cbe != "":
		_cbm = _cbe.to_float()
	if _cbm > 0.0:
		lock_heading = false
	if lock_heading:
		print("BodyController: CORRIDOR — heading LOCKED (forward/reverse only)")
	elif _cbm > 0.0:
		print("BodyController: CORRIDOR TURN rig — heading FREE (steer toward food at ±%.0f°)" % _cbm)
	# Vision green-saliency emphasis (config-self-contained; env overrides for sweeps).
	if _cell_meta.has("vision_green_gain"):
		_vision_green_gain = clampf(float(_cell_meta["vision_green_gain"]), 0.0, 1.0)
	var _vg: String = OS.get_environment("OGMA_VISION_GREEN_GAIN")
	if _vg != "":
		_vision_green_gain = clampf(_vg.to_float(), 0.0, 1.0)
	if _vision_green_gain > 0.0:
		print("BodyController: vision_green_gain = %.2f (non-green pixels dimmed to %.0f%%)"
			% [_vision_green_gain, 100.0 * (1.0 - _vision_green_gain)])
	# 2026-06-20 — FPV grid resolution (even 24 vs odd 25 dead-centre ray) +
	# forward-loom fovea params.  Metadata then env override.
	if _cell_meta.has("vis_res"):
		_vis_res = maxi(2, int(_cell_meta["vis_res"]))
	var _vr: String = OS.get_environment("OGMA_VIS_RES")
	if _vr != "": _vis_res = maxi(2, _vr.to_int())
	if _cell_meta.has("fovea_res"):
		fovea_res = maxi(1, int(_cell_meta["fovea_res"]))
	if _cell_meta.has("fovea_fov_deg"):
		fovea_fov_deg = maxf(1.0, float(_cell_meta["fovea_fov_deg"]))
	# Zone-coloured walls (place-coding distinctiveness; default off = uniform walls).
	_room_size = float(_cell_meta.get("room_size", 24.0))
	if _cell_meta.has("zone_colors"):
		_zone_colors = bool(_cell_meta["zone_colors"])
	if _cell_meta.has("zone_grid"):
		_zone_grid = maxi(1, int(_cell_meta["zone_grid"]))
	if _zone_colors:
		print("BodyController: ZONE-COLOURED walls ON (%dx%d zones, %d-hue palette)"
			% [_zone_grid, _zone_grid, _ZONE_PALETTE.size()])
	print("BodyController: vis_res=%d (%s) | forward-loom fovea %dx%d @ %.0f° FOV"
		% [_vis_res, ("ODD: dead-centre ray" if _vis_res % 2 == 1 else "EVEN: centre gap"),
		   fovea_res, fovea_res, fovea_fov_deg])

	# Phase 6.8 — drift tuning: lower friction = longer coast = a continuous
	# IMU signal that keeps the homeokinetic TLE alive (avoids the stop→zero-TLE
	# stall).  env overrides for the linear/angular per-tick decay.
	var _fl: String = OS.get_environment("OGMA_FRICTION_LIN")
	if _fl != "": friction_lin_per_tick = clampf(_fl.to_float(), 0.0, 1.0)
	var _fa: String = OS.get_environment("OGMA_FRICTION_ANG")
	if _fa != "": friction_ang_per_tick = clampf(_fa.to_float(), 0.0, 1.0)
	if _fl != "" or _fa != "":
		print("BodyController: friction lin=%.3f ang=%.3f" % [friction_lin_per_tick, friction_ang_per_tick])

	var _efm: String = OS.get_environment("OGMA_EAT_FORWARD_MIN")
	if _efm != "":
		eat_forward_min = maxf(0.0, _efm.to_float())
		print("BodyController: eat_forward_min=%.3f (head-on eat gate)" % eat_forward_min)

	var env_h: String = OS.get_environment("OGMA_BODY_HEADING")
	var env_hrand: String = OS.get_environment("OGMA_BODY_HEADING_RANDOMIZE")
	if env_h != "":
		heading = fposmod(deg_to_rad(env_h.to_float()), TAU)
		rotation.y = heading
	elif env_hrand != "" and env_seed != "":
		# Deterministic per-seed pseudo-random heading in [0, 2π).
		var h_rng := RandomNumberGenerator.new()
		h_rng.seed = env_seed.hash() ^ 0x686472   # "hdr"
		heading = h_rng.randf() * TAU
		rotation.y = heading
	print("BodyController: starting heading = %.1f°" % rad_to_deg(heading))

	# --- Register the complete environment ↔ brain interface ---
	# This is the canonical record of what signals are allowed to cross the
	# boundary in each direction.  Anything not listed here is NOT wired.
	brain.register_source(
		"IMU", "reality.proprio.imu",
		"float32[4]: sin(h),cos(h),vx,vz — kinematic only (whiskers split off)",
		true)
	brain.register_source(
		"Whiskers", "reality.proprio.whisker_*",
		"float32[1] × 6: per-whisker contact intensity (one EPM per channel)",
		true)
	brain.register_source(
		"Energy", "reality.proprio.energy",
		"float32[1]: internal energy 0→1 (passive drain + hit replenish)",
		true)
	brain.register_source(
		"Scent", "reality.proprio.scent",
		"float32[8]: 8 body-relative nostrils, scalar concentration only",
		true)
	brain.register_source(
		"Terrain", "reality.proprio.terrain",
		"float32[4]: slope_x, slope_z, roughness, height_below_body",
		true)
	brain.register_source(
		"Camera", "host.video.color",
		"uint8[H×W×3]: raw first-person RGB → epm_color JL projection (food is green vs grey/blue scene)",
		true)
	brain.register_source(
		"Motor energy", "reality.proprio.motor_energy",
		"float32[2]: per-flagellum leaky energy budget (drains with paddling, recharges at rest)",
		true)
	brain.register_sink(
		"Motor", "action.out",
		"float32 accel [-4,+4] → per-flagellum spike rates → impulse-integrated body dynamics. Legacy single-channel; superseded when action.left + action.right are both published.")
	# Phase 6.6.D.6 — bilateral motor sinks.  Bodies with paired actuators
	# (paddler L+R today; quadruped legs / octopus arms later) read these
	# differentially.  When both channels are publishing, action.out is
	# silenced and steering emerges from L−R differential thrust.
	brain.register_sink(
		"Motor (left)", "action.left",
		"float32 accel [-4,+4] → left-flagellum spike rate. Paired with action.right for differential steering; falls back to action.out when not present.")
	brain.register_sink(
		"Motor (right)", "action.right",
		"float32 accel [-4,+4] → right-flagellum spike rate. Paired with action.left for differential steering; falls back to action.out when not present.")
	brain.register_event("Hit",       "events.hit",       "reward")
	brain.register_event("Eat",       "events.eat",       "reward")   # GROUND-TRUTH consummatory event (real nutrient collision only)
	brain.register_event("WallStuck", "events.wall_stuck", "aversive")

	# Resolve config_path:  ExperimentConfig (launcher) > OGMA_CELL_CONFIG
	# env var > @export default.  Headless harness path (env-var-driven)
	# is preserved when launched=false.
	config_path = ExperimentConfig.resolve_config("OGMA_CELL_CONFIG", config_path)

	# Live-UI 4-loop ablation: patch the EFE arbiter per the launcher's Ablation
	# dropdown (mirrors the headless harness). Empty overrides -> path unchanged.
	config_path = _apply_arbiter_overrides(config_path, ExperimentConfig.cell_arbiter_overrides)

	# v6.0 — propagate resolved seed into brain master_seed (see
	# OgmaBrain::set_master_seed).  Same rationale as cart_body / mc_body.
	if _stuck_rng.seed != 0:
		brain.set_master_seed(int(_stuck_rng.seed))
	if not brain.setup(config_path):
		push_error("BodyController: brain.setup() failed: %s" % config_path)


# Live-UI 4-loop leave-one-out: patch the loaded config's EFEArbiter params per the
# launcher's Ablation dropdown, write a temp config, and return its path (brain.setup
# reads the FILE, so we cannot mutate in-memory). We TEXT-patch rather than round-trip
# through JSON: Godot's JSON.parse_string returns EVERY number as a float, so a
# re-serialised config turns integer params (e.g. HeadingController.cx_index) into floats
# and the C++ loader rejects them. Overrides map "ModuleType.param" -> value.
func _apply_arbiter_overrides(cfg_path: String, overrides: Dictionary) -> String:
	if overrides.is_empty():
		return cfg_path
	var f := FileAccess.open(cfg_path, FileAccess.READ)
	if f == null:
		push_warning("ablation: cannot read %s -- running unpatched" % cfg_path)
		return cfg_path
	var text := f.get_as_text()
	f.close()
	var patched := _patch_module_params(text, "EFEArbiter", overrides)
	if patched == text:
		push_warning("ablation: no EFEArbiter param patched -- running unpatched")
		return cfg_path
	var out_path := "user://_ablation_active.json"
	var of := FileAccess.open(out_path, FileAccess.WRITE)
	if of == null:
		push_warning("ablation: cannot write temp config -- running unpatched")
		return cfg_path
	of.store_string(patched)
	of.close()
	print("BodyController: ablation applied ", overrides, " -> ", out_path)
	return out_path

# Replace (or insert) each override inside module `mtype`'s "params" object, editing the
# raw JSON TEXT scoped to that module's braces (so a param name shared with another module,
# e.g. scent_topic / plan_value_topic, is not touched elsewhere). String values JSON-quoted;
# float values keep a decimal so integer-typed params are never disturbed.
func _patch_module_params(text: String, mtype: String, overrides: Dictionary) -> String:
	var ti := text.find('"type": "%s"' % mtype)
	if ti < 0:
		return text
	var pi := text.find('"params"', ti)
	if pi < 0:
		return text
	var open_brace := text.find("{", pi)
	if open_brace < 0:
		return text
	var depth := 0
	var close_brace := -1
	for i in range(open_brace, text.length()):
		if text[i] == "{":
			depth += 1
		elif text[i] == "}":
			depth -= 1
			if depth == 0:
				close_brace = i
				break
	if close_brace < 0:
		return text
	var block := text.substr(open_brace, close_brace - open_brace + 1)
	for key in overrides:
		var parts := String(key).split(".", true, 1)
		if parts.size() != 2 or parts[0] != mtype:
			continue
		var pname: String = parts[1]
		var v = overrides[key]
		var vtext: String
		if typeof(v) == TYPE_STRING:
			vtext = '"%s"' % v
		elif typeof(v) == TYPE_FLOAT:
			vtext = str(v)
			if not ("." in vtext or "e" in vtext or "E" in vtext):
				vtext += ".0"
		else:
			vtext = str(v)
		var needle := '"%s":' % pname
		var ki := block.find(needle)
		if ki >= 0:
			var vs := ki + needle.length()
			while vs < block.length() and block[vs] == " ":
				vs += 1
			var ve := vs
			while ve < block.length() and block[ve] != "," and block[ve] != "}" and block[ve] != "\n":
				ve += 1
			block = block.substr(0, vs) + vtext + block.substr(ve)
		else:
			block = '{\n        %s: %s,' % ['"' + pname + '"', vtext] + block.substr(1)
	return text.substr(0, open_brace) + block + text.substr(close_brace + 1)


# ---------------------------------------------------------------------------
# Physics process
# ---------------------------------------------------------------------------

func _physics_process(delta: float) -> void:
	if brain == null or not brain.is_brain_ready():
		return

	# Headless D-value rescue hook: cut the scent field at a known tick (env-driven,
	# reproducible) so the A/B can build the map under full scent, then cut it.
	# OGMA_SCENT_CUT_AT = tick; OGMA_SCENT_CUT_TO = reach value (default 0).
	_global_tick += 1
	if not _scent_cut_init:
		_scent_cut_init = true
		var ca := OS.get_environment("OGMA_SCENT_CUT_AT")
		if ca != "": _scent_cut_at_tick = int(ca.to_float())
		var ct := OS.get_environment("OGMA_SCENT_CUT_TO")
		if ct != "": _scent_cut_to = ct.to_float()
	if _scent_cut_at_tick >= 0 and not _scent_cut_done and _global_tick >= _scent_cut_at_tick:
		var w := get_parent()
		if w != null and w.has_method("set_scent_reach"):
			w.call("set_scent_reach", _scent_cut_to)
		_scent_cut_done = true
		print("BodyController: SCENT CUT to reach=", _scent_cut_to, " at tick ", _global_tick)

	# 1. Phase 6.0.b — kinematic IMU (4 floats) + per-whisker channels (1 each).
	#
	# The 10-float bundled IMU was a single perceptual surface; each whisker
	# now publishes its own topic so each gets its own EPM and the
	# LateralVoter can learn trust per channel.  This is the linear-scaling
	# test from the position paper — adding modules should be additive
	# rather than requiring re-tuning.
	# Phase 6.9.A — AFFERENT IMU velocity: the bug's ACTUAL world motion
	# (post-collision position delta), NOT the commanded velocity (efference
	# copy).  Wedged against a wall the flagella keep firing (_forward_speed
	# stays high) but the body does NOT displace → afferent velocity → 0 and
	# perception correctly FREEZES, so the homeokinetic loop can sense the
	# broken sensorimotor contingency.  Biologically: vestibular / optic flow
	# sense actual motion, not motor intent.
	var afferent_vel: Vector3 = Vector3.ZERO
	if _imu_pos_init and delta > 0.0:
		afferent_vel = (global_position - _prev_imu_pos) / delta
	_prev_imu_pos = global_position
	_imu_pos_init = true
	var imu_kin := PackedFloat64Array()
	imu_kin.append(sin(heading))
	imu_kin.append(cos(heading))
	imu_kin.append(afferent_vel.x / move_speed)
	imu_kin.append(afferent_vel.z / move_speed)
	brain.publish_proprio(imu_kin, "imu")
	# 2026-06-21 — EGOCENTRIC afferent velocity [v_right, v_forward], normalized by
	# move_speed.  Computed by rotating the world velocity into the body's own frame
	# (Godot basis), so it is FRAME-CORRECT by construction — no sin/cos convention
	# guessing.  Local +X = right, −Z = forward (matches _fwd_dir = -basis.z).  This
	# is the food-INDEPENDENT motion signal the learned action layer credits ("am I
	# advancing ALONG the heading I was told to follow" = ego_vel · commanded_dir),
	# and the IMU course-of-travel nav will credit later (achieved heading, not
	# commanded).  Afferent (actual displacement), not efferent intent → wedged ⇒ ~0.
	var _local_vel: Vector3 = global_transform.basis.inverse() * afferent_vel
	brain.publish_proprio(
		PackedFloat64Array([_local_vel.x / move_speed, -_local_vel.z / move_speed]), "vel_ego")
	# Efference copy (reafference): the COMMANDED velocity the bug intended this
	# tick (forward heading × integrated motor drive).  Published so the brain
	# can compare intent (efferent) vs result (afferent IMU) — the literal
	# broken-contingency / homeokinetic distress signal.  Wedged: efference
	# high, afferent ≈ 0 → "I tried but didn't move".
	var _fwd_dir: Vector3 = -global_transform.basis.z
	brain.publish_proprio(
		PackedFloat64Array([_fwd_dir.x * _forward_speed / move_speed,
							 _fwd_dir.z * _forward_speed / move_speed]), "motor_efference")
	# 2026-06-21 — signed yaw rate (rad/tick).  The HeadingController learns its
	# body's turn response k_body = ω/steer from THIS (a clean, purely steer-driven
	# rotational signal — unlike the egocentric bearing, which translation also
	# moves), then inverts it to command a calibrated turn (the no-tuning body model).
	brain.publish_proprio(PackedFloat64Array([_angular_velocity]), "ang_vel")
	# Absolute heading (rad) — proprioception (vestibular/efference, a real self-sense,
	# not an external oracle).  GoalBelief uses it for PATH INTEGRATION: it stores the
	# goal direction in the WORLD frame so a momentary scent occlusion doesn't erase
	# the heading, and ego-rotation is compensated (world_goal − heading) → the belief
	# keeps pointing at the remembered food location as the bug turns.
	brain.publish_proprio(PackedFloat64Array([heading]), "heading")
	# 2026-06-26 — sin/cos heading for the compass-EPM (orientation, no ±π fracture, §7).
	brain.publish_proprio(PackedFloat64Array([sin(heading), cos(heading)]), "heading_vec")
	# Phase 6.8 — per-paddle leaky energy (the homeokinetic controller subscribes
	# this so it can learn to pace its beating).  Always published (stays 1.0 when
	# the dynamics are disabled) so a bundle subscribing it never stalls.
	brain.publish_proprio(
		PackedFloat64Array([_motor_energy_left, _motor_energy_right]), "motor_energy")

	for i in range(whiskers.size()):
		var rc = whiskers[i]
		var w_val := 0.0
		if rc.is_colliding():
			var dist := global_position.distance_to(rc.get_collision_point())
			w_val = clampf(1.0 - dist / whisker_length, 0.0, 1.0)
		brain.publish_proprio(PackedFloat64Array([w_val]), "whisker_%d" % i)

	# Phase 6.9.A (③ curiosity escape) — FORWARD clearance: how OPEN the path
	# ahead is.  1 = clear ahead, 0 = wall/pillar directly ahead.  Max contact
	# among the forward-facing whiskers (body-local −Z), inverted.  The spatial
	# "open vs wall" feel the curiosity escape runs toward.
	var _fwd_contact: float = 0.0
	for rc in whiskers:
		if rc.is_colliding():
			var _tp: Vector3 = rc.target_position
			var _len: float = _tp.length()
			if _len > 0.0 and _tp.z / _len < -0.5:   # forward-pointing ray
				var _d: float = global_position.distance_to(rc.get_collision_point())
				_fwd_contact = maxf(_fwd_contact, clampf(1.0 - _d / whisker_length, 0.0, 1.0))
	brain.publish_proprio(PackedFloat64Array([1.0 - _fwd_contact]), "clearance")

	# 2. Energy channel (HomeostaticDrive)
	brain.publish_proprio(PackedFloat64Array([energy]), "energy")

	# 2a. Hunger sensor → NeurochemState lowers serotonin as hunger grows.
	#     Different consumer than #2: HomeostaticDrive needs the absolute
	#     energy level for its setpoint error; neurochem wants the signed
	#     hunger pressure (1−energy) as a sensory drive.
	brain.publish_proprio(PackedFloat64Array([1.0 - energy]), "hunger")

	# 2b. SCENT — the omnidirectional chemical concentration, virtualized POSITION-ONLY: sampled
	# once at the bug's CENTRE (a "bath", frame-independent), NEVER a directional read of the source.
	# This is the defensible chemotaxis substrate: the bug senses a SCALAR level and the DIRECTION
	# EMERGES from run-and-tumble (temporal comparison as it MOVES), exactly like E. coli.
	# (The old scent_max was the max over an 8-nostril ring offset by the body basis, so it SPUN
	# with the bug: a bug turning in place saw a phantom rising/falling scent that klino's
	# methylation and the L2 arbiter's proximity term chased → the spin-in-place lock at the
	# scent/plan boundary the operator observed. Sampling the centre removes that artifact.)
	var world := get_parent() as Node3D
	if world and (world.has_method("compute_scent_scalar") or world.has_method("compute_scent_vector")):
		var smax := 0.0
		if world.has_method("compute_scent_scalar"):
			smax = world.compute_scent_scalar(global_position)   # POSITION-ONLY centre sample
		else:                                                     # fallback for an older world script
			var _r: PackedFloat64Array = world.compute_scent_vector(global_position, global_transform.basis)
			for v in _r: smax = maxf(smax, v)
		_last_scent_max = smax
		brain.publish_proprio(PackedFloat64Array([smax]), "scent_max")

		# 2b-scaffold. DIRECTIONAL 8-nostril ring — a SEPARATE sensory modality (a spatial gradient
		# array that ROTATES with the body), NOT the position-only chemical bath above. Retained
		# ONLY for legacy ScentCompass / Klinotaxis / bearing-estimator / epm_scent configs that
		# subscribe reality.proprio.scent; publish_scent_ring=false (position-only chemotaxis
		# configs, e.g. the_cell_arbiter) FULLY BYPASSES it so the scent virtualization is
		# unambiguously position-only for defensibility.
		if _publish_scent_ring and world.has_method("compute_scent_vector"):
			var ring: PackedFloat64Array = world.compute_scent_vector(
				global_position, global_transform.basis)
			brain.publish_proprio(ring, "scent")
			_last_scent = ring

		# 2c+. Phase 6.5.16 — dual-EMA scent reward on the POSITION-ONLY scalar.
		# Phase 6.5.19 NOTE: statistical adaptive threshold tested and reverted (38% fewer hits);
		# it has no performance gradient, so the substrate-derived stddev variants are kept
		# diagnostic-only. Fire HIT using the legacy fixed 1.5× rule (proven baseline).
		_scent_short_ema = (1.0 - _SCENT_SHORT_ALPHA) * _scent_short_ema + _SCENT_SHORT_ALPHA * smax
		_scent_long_ema  = (1.0 - _SCENT_LONG_ALPHA)  * _scent_long_ema  + _SCENT_LONG_ALPHA  * smax
		var _scent_diff: float = _scent_short_ema - _scent_long_ema
		var _scent_diff_delta: float = _scent_diff - _scent_diff_mean_ema
		_scent_diff_mean_ema = (1.0 - _ADAPT_ALPHA) * _scent_diff_mean_ema + _ADAPT_ALPHA * _scent_diff
		_scent_diff_var_ema  = (1.0 - _ADAPT_ALPHA) * _scent_diff_var_ema  + _ADAPT_ALPHA * _scent_diff_delta * _scent_diff_delta
		if _scent_long_ema > 0.001 \
				and _scent_short_ema > _scent_long_ema * _SCENT_HIT_THRESHOLD \
				and _forward_speed > _SCENT_HIT_MOTION_FLOOR:
			_publish_reflex_event("hit", 1.0)

	# 2c++. Phase 6.5.23 — first-person color frame.  SubViewport mirrors
	# the FPV camera transform, renders at low resolution; we read the
	# RGB texture and forward to the substrate via brain.publish_video.
	# Sampled every _VIS_CAPTURE_EVERY ticks to bound GPU readback cost.
	if (tick_counter % _VIS_CAPTURE_EVERY) == 0:
		_publish_video_frame()

	# 2d. Phase 6.2 — terrain probe.  4 body-relative downward raycasts
	# resolve into (slope_x, slope_z, roughness, height_below_body).  At
	# terrain_amplitude=0 the floor is flat so the signal is the constant
	# vector and epm_terrain saturates immediately (correct silent-modality
	# fallback).  With terrain active the values vary with body position
	# and orientation, giving epm_terrain something to topologise.
	brain.publish_proprio(_compute_terrain_probe(), "terrain")

	# 3. Metabolic energy drain (2026-06-30 operator) — energy is spent by MOTOR
	# WORK (force×time), not a wall-clock timer: drain ∝ motor effort applied last
	# tick (spike count, or |al|+|ar| for the continuous paddler).  Keeping ·delta
	# makes _energy_drain_per_sec BACKWARD-COMPATIBLE — at unit effort it equals the
	# old per-second drain, so existing configs' energy_drain values keep their
	# meaning; the only change is that motion now scales it (faster when driving hard,
	# nothing while coasting).  An eat resets energy to full (on_nutrient_hit) → a
	# clean homeostatic sawtooth.
	energy = maxf(0.0, energy - _energy_drain_per_sec * _motor_effort_last * delta)
	_ticks_since_food += 1   # survival telemetry (reset in on_nutrient_hit)

	# 4. Wall-stuck aversive event
	var max_w := 0.0
	for rc in whiskers:
		if rc.is_colliding():
			var d := global_position.distance_to(rc.get_collision_point())
			max_w = maxf(max_w, 1.0 - d / whisker_length)
	if max_w > 0.55:
		_publish_reflex_event("wall_stuck", max_w)
	# Phase 6.5.16 — light-contact miss event.  Phase 6.5.19 tested
	# statistical adaptive threshold and found that pure statistical
	# adaptation (mean + N×stddev) FAILS for negative-direction
	# signals when bad-state is majority (cart pinned to walls 68%
	# → mean(whisker)≈0.65 → threshold drifts to 0.96 → miss events
	# almost never fire).  Even with a ceiling-bounded variant, the
	# adaptive form added no value over the fixed 0.30 threshold.
	# Statistics are still tracked for diagnostic purposes.
	var _w_delta: float = max_w - _whisker_mean_ema
	_whisker_mean_ema = (1.0 - _ADAPT_ALPHA) * _whisker_mean_ema + _ADAPT_ALPHA * max_w
	_whisker_var_ema  = (1.0 - _ADAPT_ALPHA) * _whisker_var_ema  + _ADAPT_ALPHA * _w_delta * _w_delta
	var _miss_threshold: float = _WHISKER_MISS_THRESHOLD
	# Phase 6.5.21 — scent-gated whisker aversion.  When the cart is
	# making positive scent progress (short EMA above long EMA), it's
	# in a goal-rich zone — food often spawns near walls so whisker
	# contact there isn't a reason to flee.  Scale the miss-event
	# magnitude (and the directed-escape rectification, below) by
	# (1 - suppress) so contact is gradually neutralised as the cart
	# closes on food.  Disable via OGMA_SCENT_GATE=0.
	var _scent_suppress: float = 0.0
	# Gate enabled by default; ExperimentConfig and env var can disable it.
	# Phase 6.6.D.8 — when the active config asks for graph-resident
	# reflexes, ScentGateReflex computes the suppression as a ReflexGate
	# token the brain consumes; the body-side block here would double-count
	# (and the body's _publish_reflex_event is short-circuited anyway, so
	# scaling it doesn't matter, but the directed-escape rectification
	# below ALSO reads _scent_suppress and that path IS active).  Keep the
	# gate disabled body-side under modular reflexes; the Phase 6.6.D.7
	# bilateral steering already routes through the brain and observes the
	# ScentGateReflex via the modules that consume it.
	var _gate_enabled: bool = true
	if ExperimentConfig.resolve_reflex_modular():
		_gate_enabled = false
	elif ExperimentConfig.launched and ExperimentConfig.scent_gate_off:
		_gate_enabled = false
	elif OS.get_environment("OGMA_SCENT_GATE") == "0":
		_gate_enabled = false
	if _gate_enabled and _scent_long_ema > 0.001:
		# Cap the suppression magnitude.  Empirical sweep at cap ∈
		# {0, 0.25, 0.5, 0.75, 1.0} on 10×120s gave 27/26/31/21/25 hits;
		# 0.5 wins (the cart still treats walls as net-aversive but
		# scent rises can at most halve the aversion).  Precedence:
		# ExperimentConfig > OGMA_SCENT_GATE_CAP > default.
		var _gate_cap: float = 0.5
		if ExperimentConfig.launched and ExperimentConfig.scent_gate_cap >= 0.0:
			_gate_cap = clampf(ExperimentConfig.scent_gate_cap, 0.0, 1.0)
		else:
			var _gate_env: String = OS.get_environment("OGMA_SCENT_GATE_CAP")
			if _gate_env != "":
				_gate_cap = clampf(_gate_env.to_float(), 0.0, 1.0)
		_scent_suppress = clampf((_scent_short_ema - _scent_long_ema) / _scent_long_ema, 0.0, _gate_cap)
	if _miss_refractory > 0:
		_miss_refractory -= 1
	elif max_w > _miss_threshold:
		_publish_reflex_event("miss", max_w * (1.0 - _scent_suppress))
		_miss_refractory = _MISS_REFRACTORY_TICKS
	_last_whisker_max = max_w

	# Phase 6.5.19 Part A — stuck-baseline diagnosis (PRE-spike).
	# Counters using last tick's _stuck_severity (set in the spike
	# section below).  We track wall-contact + refractory state at
	# the time of the stuck condition; zero_steer is logged AFTER
	# the steer is computed (see below).
	if _stuck_severity > 0.5:
		_stuck_total_ticks += 1
		if max_w > 0.30:
			_stuck_wall_ticks += 1
		if _refractory_left > 0 and _refractory_right > 0:
			_stuck_refrac_ticks += 1
	if _stuck_severity_actual > 0.5:
		_stuck_actual_pin_ticks += 1
	# Run-summary averages for forward/angular velocity (logged in
	# the clipboard summary).
	_speed_sum_for_avg += _forward_speed
	_angular_sum_for_avg += absf(_angular_velocity)
	_avg_count += 1
	# Honest effort + duty cycle (uses last tick's committed motor effort,
	# consistent with the stuck counters above).  effort ∈[0,2]; a "pause" is a
	# commanded coast (near-zero paddle magnitude) — the bidirectional paddler's
	# energy-conserving drift, NOT a failure to act.
	_motor_effort_sum += _motor_effort_last
	if _motor_effort_last < _PAUSE_EFFORT:
		_pause_ticks += 1

	# 5. Cognitive tick
	brain.tick(delta)

	# 6. Two-flagellum spiking actuation + stuck detector.
	#
	# Brain emits scalar steer; body splits into per-flagellum Bernoulli rates,
	# samples discrete spikes, and integrates linear/angular velocity with
	# first-order friction.  Body is a physical integrator of impulses.
	#
	# (a) Maintain a 1-second position ring → stuck deficit (unchanged).
	# (b) Stuck-pulse adds to steer (no longer overrides heading directly —
	#     it now biases the spike asymmetry, so the rotation it produces is
	#     mediated by the same physics as cognitive steering).
	# (c) Spike rates: rate_left  = base + bias × steer  (clipped to [0,1])
	#                   rate_right = base − bias × steer
	#     At base=0.5 + bias=0.5, sum is constant → forward thrust invariant
	#     to steer; only the angular impulse is affected.
	# (d) Each spike: forward_impulse_lin to forward_speed; left-spike adds
	#     +impulse_ang to angular_velocity, right-spike adds −impulse_ang.
	# (e) Friction: forward_speed *= (1 − friction_lin); angular *= (1 − friction_ang).
	# Phase 6.6.D.6 — bilateral motor input.  When both action.left and
	# action.right were published this tick the body uses differential
	# (L−R)/8 as the effective steer in [-1,1] (matches the legacy
	# accel∈[-4,4] / 4 normalisation).  Otherwise fall back to the
	# legacy single-channel action.out path so existing configs
	# (the_cell.json, all CartPole/MC presets, the modular-reflex preset
	# without ActionGate) continue to work unchanged.
	#
	# Phase 6.6.G — when bilateral *and* the active config opts into
	# modular-passive (reflex_modular), al/ar are interpreted as direct
	# per-side thrust commands in [-4, +4] that map to per-flagellum
	# spike rates (rate_left=clamp(al/4,0,1), rate_right=clamp(ar/4,0,1)),
	# bypassing the steer-mediation collapse below.  This is the only
	# interpretation that lets differential_paddler honor a
	# (+4,+4)→forward / (+4,-4)→CW rotation policy table; the legacy
	# `accel = al - ar` path turns "both sides max forward" into zero
	# steer and zero motion.  See docs/v4_phase6_6g_multichannel_fader.md.
	var accel: float = 0.0
	var al: float = 0.0
	var ar: float = 0.0
	var bilateral_active: bool = brain.is_action_bilateral()
	if bilateral_active:
		al = brain.get_action_left()
		ar = brain.get_action_right()
		# Steer-equivalent scalar in the legacy accel∈[-4,4] frame.  Sign
		# convention: al > ar → cart turns RIGHT (matches existing
		# rate_left = base + bias·steer → more left spikes → CW rotation →
		# right turn).  Used by all non-modular-passive bilateral paths.
		accel = al - ar
	else:
		accel = brain.get_action()
	# 2026-06-19 — forward-dynamics probe: dense (every 5 ticks) record of
	# egocentric food geometry + scent/green + the motor command (al,ar) + the
	# realized motion, for the offline linear-vs-MLP forward-model test.
	if OS.get_environment("OGMA_FWDLOG") == "1" and tick_counter % 5 == 0:
		var _w := get_parent() as Node3D
		var _fe := PackedFloat64Array([0.0, 0.0, -1.0])
		if _w and _w.has_method("nearest_food_egocentric"):
			_fe = _w.nearest_food_egocentric(global_position, global_transform.basis)
		var _smax := _last_scent_max   # position-only centre scent (matches the published scent_max)
		print(JSON.stringify({"event": "FWDLOG", "t": tick_counter,
			"flat": snappedf(_fe[0], 0.001), "ffwd": snappedf(_fe[1], 0.001), "fdist": snappedf(_fe[2], 0.001),
			"smax": snappedf(_smax, 0.001), "green": snappedf(_last_green_frac, 0.0001),
			"loom": snappedf(_last_forward_loom, 0.0001),
			"al": snappedf(al, 0.001), "ar": snappedf(ar, 0.001),
			"fwdv": snappedf(_forward_speed, 0.001), "angv": snappedf(_angular_velocity, 0.001),
			"hd": snappedf(heading, 0.001)}))
	# Phase 6.5.5 diagnostic — override the decoder's accel with a fixed
	# value (env var, float) to isolate substrate-level (body / spike
	# sampler) bias from brain-driven bias.  Setting OGMA_FORCE_ACCEL=0
	# zeros the steer signal so any heading drift comes from below the
	# decoder.  Empty / unset = use the brain's choice as normal.
	var _env_force: String = OS.get_environment("OGMA_FORCE_ACCEL")
	if _env_force != "":
		accel = _env_force.to_float()
	# Phase 6.5.22 — brain-action ablation.  OGMA_BRAIN_OFF=1 zeros the
	# steer the brain produces but leaves all body-side reflexes intact
	# (stuck-pulse, directed escape, scent gate).  Distinct from
	# OGMA_FORCE_ACCEL (which also suppresses the stuck-pulse below).
	# Used to attribute "what fraction of hits comes from brain steering
	# vs body reflexes alone."
	# Phase 6.5.22 — brain-off precedence: ExperimentConfig > env var.
	if (ExperimentConfig.launched and ExperimentConfig.brain_off) \
			or OS.get_environment("OGMA_BRAIN_OFF") == "1":
		accel = 0.0
	# Phase 6.5.22 — brain-weight default = 0.5.  Brain emits accel ∈
	# [-4, 4] which previously mapped 1:1 to steer ∈ [-1, 1] via /4.0.
	# At full weight the brain dominates: 53% of ticks it picks accel=0
	# (forcing the cart still even when stuck-pulse wants rotation),
	# 44% it saturates to ±4 (overriding reflex direction entirely).
	# Halving the weight makes the brain a co-contributor with stuck-pulse
	# noise rather than a hard override, recovering ~9 hits/10×120s
	# (31 → 40) and matching pure-reflex performance (41) while keeping
	# the brain on the bus so chunks still crystallise from real action
	# variability.  Empirical sweep: 0.0/0.4/0.5/0.75/1.0 → 41/31/40/27/31.
	# Override via OGMA_BRAIN_WEIGHT.
	# Precedence: ExperimentConfig > env var > default (0.5).
	var _brain_weight: float = 0.5
	if ExperimentConfig.launched and ExperimentConfig.brain_weight >= 0.0:
		_brain_weight = clampf(ExperimentConfig.brain_weight, 0.0, 1.0)
	else:
		var _bw_env: String = OS.get_environment("OGMA_BRAIN_WEIGHT")
		if _bw_env != "":
			_brain_weight = clampf(_bw_env.to_float(), 0.0, 1.0)
	var steer_brain: float = clampf(accel / 4.0 * _brain_weight, -1.0, 1.0)

	# (a) stuck deficit
	#
	# Phase 6.5.20: two deficit values now.
	#   - deficit (vs move_speed) — gates the stuck-pulse trigger.
	#     Reads ~0.5+ during normal motion in differential_paddler
	#     (mechanical max ~1.5 m/s vs move_speed=3.0); kept that way
	#     because the brain has empirically come to rely on the
	#     resulting always-on rotation noise as exploration drive.
	#     Removing it cold caused the agent to wall-pin and stop
	#     exploring (Phase 6.5.20 v1 regression).
	#   - actual_pinning (vs body model max) — diagnostic-only,
	#     surfaced via _stuck_severity_actual for HUD/JSONL so we
	#     have an honest pinning metric distinct from the trigger.
	_pos_history.append(global_position)
	_fwdspeed_history.append(_forward_speed)
	if _pos_history.size() > _STUCK_WINDOW_TICKS:
		_pos_history.pop_front()
		_fwdspeed_history.pop_front()
	var deficit: float = 0.0
	if _pos_history.size() == _STUCK_WINDOW_TICKS:
		var actual: float = _pos_history[0].distance_to(_pos_history[_pos_history.size() - 1])
		# (1) Behaviour-gating deficit vs move_speed — UNCHANGED.  The brain has
		#     come to rely on the resulting stuck-pulse as exploration drive
		#     (Phase 6.5.20); this is a TRIGGER, not an honest "wedged" metric.
		var max_disp: float = move_speed * float(_STUCK_WINDOW_TICKS) * delta
		if max_disp > 0.0:
			deficit = clampf(1.0 - actual / max_disp, 0.0, 1.0)
		# (2) HONEST efference-matched pinning (diagnostic; body-model-agnostic).
		#     Compare NET displacement to the path the body's OWN velocity
		#     integrator commanded this window (Σ _forward_speed·δ).  High only
		#     when the body MEANT to travel but didn't translate (wall-pin) or
		#     went nowhere net (circling).  A commanded PAUSE (intended≈0) is
		#     guarded to 0 = NOT stuck.  This replaces the vs-move_speed /
		#     vs-body-max reference that mislabelled normal slow/pausing motion
		#     as stuck on the continuous bidirectional_paddler (no base_rate
		#     cruise → those references never described how it actually moves).
		var intended: float = 0.0
		for s in _fwdspeed_history:
			intended += absf(float(s)) * delta   # |speed|: an in-place oscillation (net≈0) is still commanded travel
		if intended > _PIN_MIN_INTENDED:
			_stuck_severity_actual = clampf(1.0 - actual / intended, 0.0, 1.0)
		else:
			_stuck_severity_actual = 0.0
	_stuck_severity = deficit

	# Held stuck-rotation pulse, resampled every _STUCK_PULSE_TICKS or on
	# deficit transitioning through 0.5.  Magnitude tracks deficit.
	var trigger: bool = (_stuck_pulse_ticks <= 0) \
		or (_prev_deficit <= 0.5 and deficit > 0.5)
	if trigger:
		# Phase 6.5.20 — directed stuck escape via "find the open side".
		# Random pulse direction was half-correct, half-wrong; the first
		# weighted-average attempt collapsed in corners (left/right
		# contributions cancel → net zero → fell through to random).
		#
		# Strategy: split whisker contacts by body-local X side (left
		# vs right) and turn AWAY from the more-blocked side (toward the
		# open side).  Pulse magnitude is committed to ±1.0 so the cart
		# pivots hard rather than oscillating between cancelling
		# half-strength signals.
		#
		# Sign convention (differential_paddler): steer > 0 → more
		# left-flagellum spikes → ω decreases (CW yaw) → cart turns
		# RIGHT.  So steer toward the open side:
		#   right_c > left_c (right blocked) → want to turn LEFT → pulse = -1
		#   left_c  > right_c (left blocked)  → want to turn RIGHT → pulse = +1
		# Phase 6.5.20 — directed stuck escape from contact asymmetry.
		# Sum left/right whisker contacts (by body-local X side), then:
		#   direction = sign(right_c − left_c) — turn AWAY from blocked side
		#   magnitude = lerp(|rand|, 1.0, tanh(|diff|)) — interpolates from
		#     pure noise (at zero asymmetry) to full commitment (at large
		#     asymmetry).  No magnitude thresholds, no committed ±1.0
		#     literals — both axes are derived smoothly from the same
		#     physical signal the body already publishes.
		#
		# Behavioral coverage:
		#   - Open floor / no contact → diff=0 → pure uniform noise (the
		#     exploration-noise floor the brain has come to rely on).
		#   - One side clearly blocked (wall) → tanh saturates → committed
		#     pulse toward the open side.
		#   - Symmetric corner / between two pillars → diff small → mostly
		#     noise with mild rectification → multiple cycles sample
		#     multiple directions and the cart finds an open direction
		#     by trial.
		#
		# Sign convention (differential_paddler): steer > 0 → CW yaw → cart
		# turns RIGHT, so escape from a right-side wall is steer < 0,
		# i.e. direction = -sign(diff) when diff > 0.
		#
		# Ablation that picked this form: A_tanh / B_lerp / C_signed /
		# D_noise across 10 seeds × 120s.  B_lerp won on hits (25 vs
		# 24/11/9), seeds-with-hits (8/10), AND pinning% (31% vs
		# 43/68/69) — best on every axis, no magic numbers.
		var _left_c:  float = 0.0
		var _right_c: float = 0.0
		for _rc in whiskers:
			if _rc.is_colliding():
				var _d := global_position.distance_to(_rc.get_collision_point())
				var _w := clampf(1.0 - _d / whisker_length, 0.0, 1.0)
				if _w > 0.0:
					var _tp: Vector3 = _rc.target_position
					var _len: float  = _tp.length()
					if _len > 0.0:
						var _x_unit: float = _tp.x / _len
						if   _x_unit < 0.0: _left_c  += _w
						elif _x_unit > 0.0: _right_c += _w
		var _diff: float = _right_c - _left_c
		var _rand: float = _stuck_rng.randf_range(-1.0, 1.0)
		# Phase 6.5.21 — scale the rectification factor by (1-suppress)
		# so the cart stops actively fleeing walls when scent is rising.
		# At full suppression the pulse collapses to pure noise; the
		# brain's own steer still drives navigation, body just stops
		# overruling it on contact.
		var _t:    float = tanh(absf(_diff)) * (1.0 - _scent_suppress)
		var _dir:  float = (-1.0 if _diff > 0.0 else (1.0 if _diff < 0.0 else sign(_rand)))
		_stuck_pulse_held = clampf(_dir * lerp(absf(_rand), 1.0, _t), -1.0, 1.0)
		_stuck_pulse_ticks = _STUCK_PULSE_TICKS
	_stuck_pulse_ticks -= 1
	_prev_deficit = deficit
	# Suppress stuck-pulse noise when accel is being externally forced —
	# we want clean body-envelope characterisation without injected
	# rotation noise contaminating the trajectory.
	var stuck_pulse: float = deficit * _stuck_pulse_held
	if _env_force != "":
		stuck_pulse = 0.0
	# Phase 6.6.D.8 honest-wiring — under graph-resident reflexes the body is a
	# passive integrator for FORWARD thrust: the baseline spike rate below is
	# zeroed so an empty graph produces no forward motion (modules drive it).
	var _modular_passive: bool = ExperimentConfig.resolve_reflex_modular()
	# 2026-06-17 (operator decision) — the wedge-escape stuck_pulse is a SPINAL
	# SAFETY REFLEX, not a navigation source, so keep it active even under
	# reflex_modular.  The never-built "StuckSteerReflex" left the cognitive bug
	# with NO escape → it plowed into walls and wedged forever (reproducible via
	# OGMA_REFLEX_MODULAR=1, both paddlers, 0 hits).  Cognition does directed nav;
	# this keeps the bug alive.  Injected into the bilateral-passive rates below
	# (al/ar bypass `steer`).

	# (b) combined steer biases the spike asymmetry
	var steer: float = clampf(steer_brain + stuck_pulse, -1.0, 1.0)

	# Phase 6.5.19 Part A — zero-steer stuck attribution.  If the cart
	# IS stuck (deficit > 0.5) AND the brain's chosen steer is near
	# zero, the brain's "do nothing" was likely a contributor.
	if _stuck_severity > 0.5 and absf(steer_brain) < 0.1:
		_stuck_zero_steer_ticks += 1

	# (c) per-flagellum Bernoulli rates.  In modular-passive mode the base
	# rate is zero — joint-spike forward thrust must come from a module
	# publishing positive thrust on action.left + action.right (e.g.
	# ForwardDriveReflex).  With no such publisher, both rates collapse
	# to 0 and the agent freezes: the test the user asked for.
	#
	# Phase 6.6.G — bilateral per-flagellum interpretation.  When the
	# config opts into modular-passive AND the brain is publishing
	# bilateral actions this tick, al/ar are direct per-side thrust
	# commands.  Map them to spike rates without the steer-mediation
	# collapse so a (+4, +4) Premotor intent lands as joint spikes (→
	# forward under differential_paddler) and (+4, -4) lands as left-
	# only spikes (→ pure rotation).  Negative thrust clamps at the
	# rate floor; "reverse spike" isn't a thing in the spike model.
	var _base: float = 0.0 if _modular_passive else flagellum_base_rate
	var rate_left:  float
	var rate_right: float
	var _bilateral_passive: bool = bilateral_active and _modular_passive
	if _bilateral_passive:
		var al_norm: float = clampf(al / 4.0, -1.0, 1.0)
		var ar_norm: float = clampf(ar / 4.0, -1.0, 1.0)
		if motor_baseline_beat:
			# Phase 6.8 — baseline-beat: map the controller output to a CENTERED
			# spike rate (0 → 0.5 baseline beat, −1 → 0 pause, +1 → 1 full).  The
			# operating point sits inside the firing region, so a homeokinetic
			# controller's linear forward model is grounded (no half-wave dead zone
			# → no phantom-TLE frozen attractor).  The flagella always beat at
			# baseline; the per-paddle energy budget then forces coast/rest.
			rate_left  = clampf(0.5 + 0.5 * al_norm, 0.0, 1.0)
			rate_right = clampf(0.5 + 0.5 * ar_norm, 0.0, 1.0)
		else:
			rate_left  = clampf(al_norm, 0.0, 1.0)
			rate_right = clampf(ar_norm, 0.0, 1.0)
		# 2026-06-17 — spinal wedge-escape: al/ar bypass `steer`, so fold the
		# safety stuck_pulse differential into the bilateral rates → a wedged
		# cognitive bug still gets a body-side escape rotation (operator: keep
		# safety reflexes as spinal scaffolds under reflex_modular).
		if stuck_pulse != 0.0:
			rate_left  = clampf(rate_left  + flagellum_steer_bias * stuck_pulse, 0.0, 1.0)
			rate_right = clampf(rate_right - flagellum_steer_bias * stuck_pulse, 0.0, 1.0)
	else:
		rate_left  = clampf(_base + flagellum_steer_bias * steer, 0.0, 1.0)
		rate_right = clampf(_base - flagellum_steer_bias * steer, 0.0, 1.0)
	# Phase 6.5.6 — refractory: force rate to 0 while a paddle is in its
	# refractory window.  Decrement counters here so a refractory tick
	# is just a "missed firing opportunity" — the rate is computed from
	# steer as usual but the actual sampling is gated.
	if _refractory_left  > 0:
		rate_left  = 0.0
		_refractory_left  -= 1
	if _refractory_right > 0:
		rate_right = 0.0
		_refractory_right -= 1

	# Phase 6.8 — per-paddle leaky energy: gate thrust by available energy, then
	# drain by the effort spent (gated rate) and recharge a constant trickle.
	# Low energy → weaker/zero spikes → forced coast/drift → recovery.
	if motor_energy_enabled:
		# Hysteresis: exhausted paddle fully rests (rate 0 → coast) until recovered.
		if _motor_energy_left  < motor_energy_low:   _motor_exhausted_left  = true
		elif _motor_energy_left  > motor_energy_high: _motor_exhausted_left  = false
		if _motor_energy_right < motor_energy_low:   _motor_exhausted_right = true
		elif _motor_energy_right > motor_energy_high: _motor_exhausted_right = false
		if _motor_exhausted_left:  rate_left  = 0.0
		if _motor_exhausted_right: rate_right = 0.0
		_motor_energy_left  = clampf(_motor_energy_left
			- motor_energy_cost * rate_left  + motor_energy_recharge, 0.0, 1.0)
		_motor_energy_right = clampf(_motor_energy_right
			- motor_energy_cost * rate_right + motor_energy_recharge, 0.0, 1.0)

	# (d) sample spikes and apply impulses — branched by body_model.
	_last_spike_left  = _flagellum_rng.randf() < rate_left
	_last_spike_right = _flagellum_rng.randf() < rate_right
	# Phase 6.5.19 Part A — spike count (sum, /tick aggregate).
	if _last_spike_left:  _spike_count_total += 1
	if _last_spike_right: _spike_count_total += 1
	# Metabolic effort (default for spike-driven models): n_spikes this tick (0..2).
	# The bidirectional branch overrides this with its continuous command magnitude.
	_motor_effort_last = float(int(_last_spike_left) + int(_last_spike_right))
	if flagellum_refractory_ticks > 0:
		if _last_spike_left:  _refractory_left  = flagellum_refractory_ticks
		if _last_spike_right: _refractory_right = flagellum_refractory_ticks
	match body_model:
		"differential_paddler":
			# Single spike: pure rotation (no forward).  Synchronous
			# spikes: forward synergy, rotations cancel.  Sign matches
			# the user's inner-tube paddler intuition: L → CW (right
			# turn), R → CCW (left turn).  In-place rotation is
			# possible — turn capability is decoupled from forward
			# drift, removing the wall-stuck pathology that limited
			# the asymmetric paddler.
			if _last_spike_left and _last_spike_right:
				_forward_speed += 2.0 * flagellum_impulse_lin
			elif _last_spike_left:
				_angular_velocity -= flagellum_impulse_ang
			elif _last_spike_right:
				_angular_velocity += flagellum_impulse_ang
		"bidirectional_paddler":
			# Phase 6.9.A (operator 2026-06-16) — SIGNED bidirectional drive.
			# common-mode = forward/BACK thrust, differential = turn, zero
			# command = PAUSE/drift (coast on friction).  Full signed range:
			# no half-wave rectification, so the bug can REVERSE out of a wedge
			# and PAUSE to conserve energy, and the homeokinetic forward model
			# is honest (no phantom-TLE dead zone).  Applied per tick from the
			# controller command (al/ar) — beat/coast/pause texture emerges from
			# the HK dynamics, not forced spike sampling.
			var bl_norm: float = clampf(al / 4.0, -1.0, 1.0)
			var br_norm: float = clampf(ar / 4.0, -1.0, 1.0)
			# Metabolic effort for the continuous (non-spike) paddler = total commanded
			# paddle magnitude ∈[0,2].  Coasting (al≈ar≈0) costs nothing; hard drive costs most.
			_motor_effort_last = absf(bl_norm) + absf(br_norm)
			var common:  float = 0.5 * (bl_norm + br_norm)   # +forward / -reverse
			var diff_t:  float = 0.5 * (bl_norm - br_norm)   # turn: +=more-left paddle
			_forward_speed    += common * 2.0 * flagellum_impulse_lin
			# 2026-06-19 SIGN FIX (operator audit): more-left-paddle must turn the
			# bug RIGHT (CW), matching differential_paddler + physics (a stronger
			# left paddle propels the left side → rotates CW).  The original `+=`
			# made left-paddle → +ang_v → LEFT turn (CCW), backwards — which
			# silently broke every hand-coded steer on this body (nav_gain forager
			# was disabled because of it; the whisker escape reflex steered INTO
			# walls).  CW = −ang_v here (heading integrates +ang_v as CCW), so:
			_angular_velocity -= diff_t * 2.0 * flagellum_impulse_ang
		_:
			# "asymmetric_paddler" (default; pre-6.5.6 behaviour).
			if _last_spike_left:
				_forward_speed    += flagellum_impulse_lin
				_angular_velocity += flagellum_impulse_ang
			if _last_spike_right:
				_forward_speed    += flagellum_impulse_lin
				_angular_velocity -= flagellum_impulse_ang

	# (e) first-order friction
	_forward_speed    *= (1.0 - friction_lin_per_tick)
	_angular_velocity *= (1.0 - friction_ang_per_tick)

	# Corridor 1-D probe: lock heading (no yaw) so the bug only moves fwd/reverse
	# along its initial axis.  Captured start pose for trial resets.
	if lock_heading:
		_angular_velocity = 0.0
		if not _corridor_start_set:
			_corridor_start = global_position
			_corridor_start_set = true
	# Integrate heading from angular velocity
	heading = fposmod(heading + _angular_velocity * delta, TAU)
	rotation.y = heading

	# Drive linear velocity along forward heading; move_and_slide handles walls.
	var fwd: Vector3 = -global_transform.basis.z
	velocity.x = fwd.x * _forward_speed
	velocity.z = fwd.z * _forward_speed
	if not is_on_floor():
		velocity.y -= gravity * delta
	move_and_slide()

	# 6b. Overlap-based eat: after the body moved, eat any food whose volume now
	# overlaps the front mouth (catches food that SPAWNED inside it — body_entered
	# only fires on the entry edge, so spawn-inside froze the bug; operator obs).
	_poll_mouth_overlap()

	# 7. Phase 6.5.3 — chunk dispatch event on rising edge of active chunk
	# id.  Captures perceptual context (whisker_max, scent_max, heading) at
	# the moment the brain decided to fire a stored chunk.  cell_smoke.py
	# post-processes these to test the "whisker contact precedes chunk
	# dispatch precedes hit" causal-chain hypothesis.
	var cur_chunk: int = brain.get_active_chunk_id()
	if cur_chunk >= 0 and _last_chunk_id < 0:
		var sm := _last_scent_max   # position-only centre scent (matches the published scent_max)
		print(JSON.stringify({
			"t":         tick_counter,
			"event":     "CHUNK_DISPATCH",
			"chunk_id":  cur_chunk,
			"wmax":      snappedf(_last_whisker_max, 0.01),
			"scent_max": snappedf(sm, 0.001),
			"heading":   snappedf(rad_to_deg(heading), 0.1),
		}))
	_last_chunk_id = cur_chunk

	# 8. JSONL diagnostic (machine-readable; also printed as plain-text summary)
	tick_counter += 1
	if diag_interval_ticks > 0 and tick_counter % diag_interval_ticks == 0:
		_emit_jsonl(accel)

	# 8b. UI-only top-down planner debug overlay (KEY_J).  No-op until toggled on,
	# and never anything in headless (the overlay node is never built there).
	if _dbg_overlay_visible:
		_update_planner_overlay()
	# v5.4.L Diagnostic A — fire any pending HIT+N sample emissions due now.
	_drain_pending_hit_samples()

	# Phase v5.1 — Monte-Carlo actor-critic episode boundary.  Fire
	# events.episode_end every _episode_length_ticks ticks.  Premotor
	# (mc_lr > 0) drains its trajectory and applies the per-step
	# Hebbian-shaped update with G_t = Σ γ^k r_{t+k} as credit.
	if _episode_length_ticks > 0:
		_episode_tick_count += 1
		if _episode_tick_count >= _episode_length_ticks:
			brain.publish_event("episode_end", float(hits_total))
			_episode_tick_count = 0

	# Turbo budget: when OGMA_QUIT_AFTER_TICKS is set, quit the
	# subprocess once total ticks reach the budget.  Independent of
	# MC episode mechanics; the harness sets this so wall-clock isn't
	# wasted in turbo (--fixed-fps) mode.
	if _quit_after_ticks > 0 and tick_counter >= _quit_after_ticks:
		print("BodyController: OGMA_QUIT_AFTER_TICKS=%d reached at tick %d → quit"
				% [_quit_after_ticks, tick_counter])
		get_tree().quit()

# ---------------------------------------------------------------------------
# JSONL diagnostic emitter
# ---------------------------------------------------------------------------

func _emit_jsonl(accel: float) -> void:
	var metrics: Dictionary = brain.get_module_metrics()

	# Build flat per-module summary for the JSON line.
	var mods := {}
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		var t: String = m.get("type", "")
		match t:
			"ScentCompass":
				mods[mod_id] = {
					"cx":  snappedf(float(m.get("cx", 0.0)), 0.0001),   # +right (which-way)
					"cy":  snappedf(float(m.get("cy", 0.0)), 0.0001),   # +forward (facing)
					"mag": snappedf(float(m.get("mag", 0.0)), 0.0001),  # raw gradient strength
					"les": bool(m.get("lesioned", false)),              # dropout (perturbation demo)
				}
			"VisualBearing":
				mods[mod_id] = {
					"vcx":   snappedf(float(m.get("cx", 0.0)), 0.0001),         # +right
					"vcy":   snappedf(float(m.get("cy", 0.0)), 0.0001),         # +forward
					"vmag":  snappedf(float(m.get("mag", 0.0)), 0.0001),        # 0=occluded, ~1=in view
					"gfrac": snappedf(float(m.get("green_frac", 0.0)), 0.0001), # food-pixel fraction
					"vles":  bool(m.get("lesioned", false)),                    # dropout
				}
			"BearingFusion":
				mods[mod_id] = {
					"fx": snappedf(float(m.get("fx", 0.0)), 0.0001),         # fused +right
					"fy": snappedf(float(m.get("fy", 0.0)), 0.0001),         # fused +forward
					"ws": snappedf(float(m.get("w_scent", 0.0)), 0.0001),    # applied scent weight
					"wv": snappedf(float(m.get("w_vision", 0.0)), 0.0001),   # applied vision weight
				}
			"SaccadeReflex":
				mods[mod_id] = {
					"sst":   int(m.get("state", 0)),                  # 0 idle / 1 pivot / 2 refractory
					"piv":   bool(m.get("pivoting", false)),
					"sdist": snappedf(float(m.get("dist", 0.0)), 0.01), # distance since last saccade
					"scnt":  int(m.get("count", 0)),                  # saccades fired
					"nov":   snappedf(float(m.get("novelty", 0.0)), 0.001),  # vision surprise (epistemic)
					"prog":  snappedf(float(m.get("progress", 0.0)), 0.0001),# foraging progress (>0 approaching)
					"hung":  snappedf(float(m.get("hunger", 0.0)), 0.001),   # 1−energy
				}
			"CylinderBuilder":
				var pano_raw: Array = m.get("panorama", [])
				var pano: Array = []
				for v in pano_raw: pano.append(snappedf(float(v), 0.01))
				mods[mod_id] = {
					"built":  int(m.get("built", 0)),                 # cylinders finalized
					"bfill":  int(m.get("bins_filled", 0)),           # bins covered last sweep
					"pano":   pano,                                   # [n_bins*3] RGB place-code
				}
			"ColumnBuilder":
				mods[mod_id] = {
					"cdims": int(m.get("dims", 0)),                   # 3*n_strips+4
					"crec":  bool(m.get("recorded", false)),          # emitted this tick (intermittent)
					"s0r":   snappedf(float(m.get("s0r", 0.0)), 0.01),# first strip mean R (HUD sanity)
					"s0g":   snappedf(float(m.get("s0g", 0.0)), 0.01),# first strip mean G
					"s0b":   snappedf(float(m.get("s0b", 0.0)), 0.01),# first strip mean B
				}
			"PlaceGraphPlanner":
				mods[mod_id] = {
					"plan":  bool(m.get("planning", false)),          # plan(map) vs forage(scent)
					"wand":  bool(m.get("wandering", false)),         # wander(explore) when lost
					"hvis": bool(m.get("homing_vision", false)),    # vision final-approach
					"cnode": int(m.get("cur_node", -1)),              # current place
					"nnode": int(m.get("next_node", -1)),             # planned next hop
					"nn":    int(m.get("n_nodes", 0)),                # nodes in the map
					"fx":    snappedf(float(m.get("fx", 0.0)), 0.001),# chosen nav bearing
					"fy":    snappedf(float(m.get("fy", 0.0)), 0.001),
					# HK habituation ("recent = boring") telemetry
					"hcur":  snappedf(float(m.get("hab_cur", 0.0)), 0.0001),   # habituation at the current node
					"hnn":   int(m.get("n_nodes_hab", 0)),                     # nodes with a habituation value
					"hmax":  snappedf(float(m.get("max_hab", 0.0)), 0.0001),   # max habituation over the map
					# desperation ("hungrier → let go of dead caches, search out") = hunger, accelerates disconfirmation
					"desp":  snappedf(float(m.get("desperation", 0.0)), 0.001),
					"esc":   bool(m.get("escaping", false)),           # in a desperation run-out burst (breaking the exhausted-basin orbit)
				}
			"PlaceNav":
				# planner reframed: NAVIGATE the shared map to a LOOSE food-region tag.
				# plan = routing to a fresh reachable tag; wand = explore run-and-tumble (no food route).
				# ced = the committed hop was ruled unreachable (block_cost through a wall).
				mods[mod_id] = {
					"plan":  bool(m.get("planning", false)),
					"wand":  bool(m.get("wandering", false)),
					"cnode": int(m.get("cur_node", -1)),
					"nnode": int(m.get("next_node", -1)),
					"nn":    int(m.get("n_nodes", 0)),
					"val":   snappedf(float(m.get("value", 0.0)), 0.001),          # V at the current place
					"ftag":  snappedf(float(m.get("food_tag", 0.0)), 0.001),       # loose bounded food remembrance in [0,1]
					"ntags": int(m.get("n_tags", 0)),                              # nodes map-wide holding a live food tag
					"hcur":  snappedf(float(m.get("hab_cur", 0.0)), 0.0001),
					"rstall": int(m.get("route_stall", 0)),                        # ticks on a committed hop with no transition
					"ced":   bool(m.get("route_ceded", false)),                    # committed hop ruled unreachable (blocked)
					"blk":   snappedf(float(m.get("block_cost", 0.0)), 0.001),     # reachability cost on the committed hop
					"pval":  snappedf(float(m.get("plan_value", 0.0)), 0.001),     # honest reach-to-region -> arbiter reach_planner
					"pnov":  snappedf(float(m.get("plan_novelty", 0.0)), 0.001),   # coverage need when no food route -> arbiter epistemic
					"erx":   int(m.get("eats_rx", 0)),                             # lifetime eats that reached PlaceNav
				}
			"Klinotaxis":
				mods[mod_id] = {
					"base":   snappedf(float(m.get("base", 0.0)), 0.01),
					"lockin": snappedf(float(m.get("lockin", 0.0)), 0.0001),
					"period": snappedf(float(m.get("period", 0.0)), 0.1),
					"trend":  snappedf(float(m.get("trend", 0.0)), 0.0001),
					"cap":    snappedf(float(m.get("cap", 0.0)), 0.001),    # self-calibrated proximity
					"weff":   snappedf(float(m.get("weff", 0.0)), 0.001),   # applied (shrunk) weave amp
					"align":  snappedf(float(m.get("align", 1.0)), 0.001),  # heading-vs-travel alignment gate ∈[0,1]
				}
			"GradientEPM":
				mods[mod_id] = {
					"nodes": int(m.get("nodes", 0)),
					"baked": int(m.get("baked", 0)),
					"pred":  snappedf(float(m.get("pred", 0.0)), 0.001),
					"dscalar": snappedf(float(m.get("dscalar", 0.0)), 0.001),
					"ghead": snappedf(float(m.get("ghead", 0.0)), 0.01),
				}
			"RunTumbleNav":
				mods[mod_id] = {
					"act":   int(m.get("action", 0)),                  # 0 run/1 tumble
					"ptum":  snappedf(float(m.get("p_tumble", 0.0)), 0.001),  # per-tick tumble probability
					"err":   snappedf(float(m.get("error", 0.0)), 0.001),     # normalised prediction error
					"base":  snappedf(float(m.get("baseline", 0.0)), 0.001),  # methylation level (EMA of scent)
					"runs":  int(m.get("runs", 0)),
					"tumbles": int(m.get("tumbles", 0)),
					"cap":   snappedf(float(m.get("cap", 0.0)), 0.001),       # self-reported capability ∈[0,1] (→0 blind, →~1 in its own eating range)
					"speak": snappedf(float(m.get("speak", 0.0)), 0.001),     # slow-decaying scent-magnitude memory (pre-eat bootstrap denom)
					"eat_scent": snappedf(float(m.get("eat_scent", 0.0)), 0.001),  # EMA of the scent at which it actually EATS (calibrated cap denom)
					# KF0/KF1/KF6 kt-loop health: run-length asymmetry (up≫down = healthy kinesis), run integrity, directional belief
					"rlu":   snappedf(float(m.get("run_len_up", 0.0)), 0.01),      # EMA run length for runs that RAISED scent
					"rld":   snappedf(float(m.get("run_len_down", 0.0)), 0.01),    # EMA run length for runs that LOWERED scent
					"tfrac": snappedf(float(m.get("turn_frac", 0.0)), 0.001),      # EMA fraction of ticks reorienting (K1 turn burn)
					"fit":   int(m.get("forced_in_turn", 0)),                      # forced tumbles fired mid-turn (K2; →0 under run_commit)
					"dR":    snappedf(float(m.get("dir_R", 0.0)), 0.001),          # directional consistency ∈[0,1] = belief precision (0=kinesis)
					"dmu":   snappedf(float(m.get("dir_mu", 0.0)), 0.01),          # believed up-gradient absolute heading (rad)
					"mut":   int(1 if m.get("muted", false) else 0),               # KF3: klino coasting (another loop/reflex has the bus)
				}
			"RunTumbleNavV2":
				mods[mod_id] = {
					"act":   int(m.get("action", 0)),
					"ptum":  snappedf(float(m.get("p_tumble", 0.0)), 0.001),
					"err":   snappedf(float(m.get("error", 0.0)), 0.001),
					"base":  snappedf(float(m.get("baseline", 0.0)), 0.001),
					"runs":  int(m.get("runs", 0)),
					"tumbles": int(m.get("tumbles", 0)),
					"cap":   snappedf(float(m.get("cap", 0.0)), 0.001),
					"eat_scent": snappedf(float(m.get("eat_scent", 0.0)), 0.001),
					"nfloor": snappedf(float(m.get("nfloor", 0.0)), 0.0001),       # KF4 stationary-noise floor
					"vscale": snappedf(float(m.get("vscale", 0.0)), 0.001),        # KF2/KF4 learned speed scale
					"rlu":   snappedf(float(m.get("run_len_up", 0.0)), 0.01),
					"rld":   snappedf(float(m.get("run_len_down", 0.0)), 0.01),
					"tfrac": snappedf(float(m.get("turn_frac", 0.0)), 0.001),
					"fit":   int(m.get("forced_in_turn", 0)),
					"dR":    snappedf(float(m.get("dir_R", 0.0)), 0.001),          # KF6 belief precision (0=kinesis floor)
					"dmu":   snappedf(float(m.get("dir_mu", 0.0)), 0.01),
					"mut":   int(1 if m.get("muted", false) else 0),
				}
			"EFEArbiter":
				# Cell L2 — the value race: klino [MAX(z-spike on rising scent, absolute proximity
				# level clamp(hunger×scent,0,1)) → spike wins the APPROACH, level SUSTAINS the CLOSE]
				# vs planner (food-route value, a sustained LEVEL = raw/plan_peak ∈[0,1], never neg),
				# winner-take-all with an adaptive hysteresis margin. win 0=klino 1=planner.
				# mk = klino z-baseline, ppk = planner slow peak (vp denominator).
				mods[mod_id] = {
					"mode":  String(m.get("scoring_mode", "value_race")),  # value_race | efe
					"win":   int(m.get("winner", 0)),
					"rk":    snappedf(float(m.get("raw_klino", 0.0)), 0.0001),
					"rp":    snappedf(float(m.get("raw_planner", 0.0)), 0.0001),
					"vk":    snappedf(float(m.get("v_klino", 0.0)), 0.0001),    # MAX(z-spike approach, abs level, EAT-calibrated level clamp(hunger×cap) close)
					"vp":    snappedf(float(m.get("v_planner", 0.0)), 0.0001),  # route LEVEL, CEDED ×(1−klino proximity) near food
					"cap_klino": snappedf(float(m.get("cap_klino", 1.0)), 0.001),  # klino self-reported capability ∈[0,1] (BOOSTS vk's level near food)
					"mk":    snappedf(float(m.get("mean_klino", 0.0)), 0.0001),
					"ppk":   snappedf(float(m.get("plan_peak", 0.0)), 0.0001),  # planner slow-decaying peak food-route value (value-race vp denominator)
					# explicit-EFE decomposition (efe mode; 0 in value_race) — the four-term race
					"gpk":   snappedf(float(m.get("g_prag_klino", 0.0)), 0.0001),    # hunger·reach(klino), pragmatic/sensory
					"gpp":   snappedf(float(m.get("g_prag_planner", 0.0)), 0.0001),  # hunger·reach(planner), pragmatic/model
					"gek":   snappedf(float(m.get("g_epist_klino", 0.0)), 0.0001),   # (1−hunger)·z-spike, klino epistemic
					"gep":   snappedf(float(m.get("g_epist_planner", 0.0)), 0.0001), # (1−hunger)·frontier novelty
					"pnov":  snappedf(float(m.get("plan_novelty", 0.0)), 0.001),    # planner frontier novelty ∈[0,1]
					"pprec": snappedf(float(m.get("plan_precision", 0.0)), 0.001),  # planner model precision ∈[0,1] (§2.3)
					"Gk":    snappedf(float(m.get("G_klino", 0.0)), 0.0001),         # g_prag_klino + g_epist_klino
					"Gp":    snappedf(float(m.get("G_planner", 0.0)), 0.0001),       # g_prag_planner + g_epist_planner
					"gk":    snappedf(float(m.get("gain_klino", 0.0)), 0.001),
					"gp":    snappedf(float(m.get("gain_planner", 0.0)), 0.001),
					# play policy (task #33) — the epistemic GROW loop, energy-surplus weighted
					"pact":  bool(m.get("play_active", false)),                     # play in the race (weight>0 && wired && efe)
					"pval":  snappedf(float(m.get("play_value", 0.0)), 0.001),      # PlayLoop frontier value ∈[0,1]
					"gepl":  snappedf(float(m.get("g_epist_play", 0.0)), 0.0001),   # play_weight·(1−hunger)·play_value
					"Gpl":   snappedf(float(m.get("G_play", 0.0)), 0.0001),         # = g_epist_play
					"gpl":   snappedf(float(m.get("gain_play", 0.0)), 0.001),       # winner-take-all gain (0 when inert)
						"vact":  bool(m.get("vision_active", false)),                   # vision in the race (weight>0 && wired && efe)
						"vvis":  snappedf(float(m.get("vision_value", 0.0)), 0.001),    # VisualHomingNav sight-confidence
						"Gvi":   snappedf(float(m.get("G_vision", 0.0)), 0.0001),       # vision_weight·hunger·vision_value
						"gvi":   snappedf(float(m.get("gain_vision", 0.0)), 0.001),     # winner-take-all gain (0 when inert)
					"marg":  snappedf(float(m.get("margin", 0.0)), 0.0001),
					"hung":  snappedf(float(m.get("hunger", 0.0)), 0.001),
					"scent": snappedf(float(m.get("scent", 0.0)), 0.001),
				}
			"VisualHomingNav":
				# Cell loop #4 — CLOSE on a SEEN food source (the scent-independent channel).
				# hfood = food in view; val = sight-confidence → arbiter; cap = eat-calibrated reach;
				# inf = EPM informativeness (food structure); tle = vision-food EPM predictive error.
				mods[mod_id] = {
					"hfood": bool(m.get("have_food", false)),
						"htgt":  bool(m.get("have_target", false)),          # a remembered food target is held
						"pers":  bool(m.get("persisting", false)),           # homing to the remembered target (occluded)
						"tcf":   snappedf(float(m.get("tgt_conf", 0.0)), 0.001),
					"val": snappedf(float(m.get("value", 0.0)), 0.001),
					"cap": snappedf(float(m.get("cap_vision", 0.0)), 0.001),
					"egr": snappedf(float(m.get("eat_green", 0.0)), 0.001),
					"inf": snappedf(float(m.get("informativeness", 0.0)), 0.001),
					"tle": snappedf(float(m.get("epm_tle", 0.0)), 0.0001),
					"nn": int(m.get("node_count", 0)),
					"fx": snappedf(float(m.get("fx", 0.0)), 0.001),
					"fy": snappedf(float(m.get("fy", 0.0)), 0.001),
				}
			"PlayLoop":
				# Cell task #33 — the third policy: GROW the shared place-map (epistemic explore).
				# climb = routing UP the novelty gradient to the frontier; wand = run-and-tumble
				# BEYOND the mapped graph (the discovery the planner structurally can't do).
				mods[mod_id] = {
					"climb":  bool(m.get("climbing", false)),
					"wand":   bool(m.get("wandering", false)),
					"fwand":  bool(m.get("forced_wander", false)),
						"hfront": bool(m.get("have_frontier", false)),
					"stale":  int(m.get("stale_explore", 0)),
					"cnode":  int(m.get("cur_node", -1)),
					"nnode":  int(m.get("next_node", -1)),
					"nn":     int(m.get("n_nodes", 0)),
					"pval":   snappedf(float(m.get("play_value", 0.0)), 0.001),   # frontier value ∈[0,1] → arbiter
					"nov":    snappedf(float(m.get("novelty_cur", 0.0)), 0.001),  # place-EPM TLE at the current node
					"hcur":   snappedf(float(m.get("hab_cur", 0.0)), 0.001),
					"ecr":    snappedf(float(m.get("eat_credit", 0.0)), 0.001),   # EMA of "exploration led to an eat"
					"fx":     snappedf(float(m.get("fx", 0.0)), 0.001),
					"fy":     snappedf(float(m.get("fy", 0.0)), 0.001),
				}
			"MotorBus":
				# Per-influencer effective gains + authority — the muted (arbiter gain 0)
				# channel's authority→0 is visible (its advance learning pauses).
				var mb_eff: Array = []
				for v in m.get("eff_gain", []): mb_eff.append(snappedf(float(v), 0.001))
				var mb_arb: Array = []
				for v in m.get("arb_gain", []): mb_arb.append(snappedf(float(v), 0.001))
				var mb_auth: Array = []
				for v in m.get("authority", []): mb_auth.append(snappedf(float(v), 0.001))
				mods[mod_id] = {
					"names": m.get("names", []),
					"eff":   mb_eff,
					"arb":   mb_arb,
					"auth":  mb_auth,
					"gr":    snappedf(float(m.get("gr", 0.0)), 0.001),
				}
			"ScentHomingLearner":
				mods[mod_id] = {
					"nproto":   int(m.get("n_proto", 0)),                       # learned scent percepts
					"proto":    int(m.get("proto", -1)),                        # current state (VQ winner)
					"asec":     int(m.get("action", -1)),                       # chosen egocentric heading sector
					"abear":    snappedf(float(m.get("abearing", 0.0)), 0.001), # chosen θ/π (corr vs food dir)
					"vspread":  snappedf(float(m.get("vspread", 0.0)), 0.0001), # rising = learning
					"vmax":     snappedf(float(m.get("vmax", 0.0)), 0.0001),
					"dprog":    snappedf(float(m.get("dprog", 0.0)), 0.0001),   # last credited Δscent
					"eps":      bool(m.get("eps", false)),                      # exploration pick
				}
			"HeadingPlanner":
				mods[mod_id] = {
					"bsec":    int(m.get("bsec", -1)),                              # belief-bearing state sector
					"asec":    int(m.get("asec", -1)),                              # chosen heading sector
					"vspread": snappedf(float(m.get("vspread", 0.0)), 0.0001),      # max-mean of V[state] (rising=learning)
					"vmax":    snappedf(float(m.get("vmax", 0.0)), 0.0001),
					"wdprog":  snappedf(float(m.get("win_dprog", 0.0)), 0.0001),    # last window's credited Δscent_max
					"eps":     bool(m.get("eps_pick", false)),                      # exploration pick
					"cov":     snappedf(float(m.get("coverage", 0.0)), 0.001),      # frac of V cells visited
					"abear":   snappedf(float(m.get("abearing", 0.0)), 0.001),      # chosen heading θ/π (corr vs fbear)
				}
			"BearingEstimator":
				mods[mod_id] = {
					"np":   int(m.get("n_proto", 0)),                              # learned ring prototypes
					"win":  int(m.get("winner", -1)),
					"tle":  snappedf(float(m.get("tle", 0.0)), 0.0001),            # ring reconstruction err (inference error)
					"icx":  snappedf(float(m.get("cx", 0.0)), 0.001),             # inferred bearing +right
					"icy":  snappedf(float(m.get("cy", 0.0)), 0.001),             # inferred bearing +forward
					"les":  bool(m.get("lesioned", false)),                       # teacher lesioned (running standalone)
				}
			"MotivationGate":
				mods[mod_id] = {
					"g":   snappedf(float(m.get("gain", 0.0)), 0.001),     # pursuit gain (0=sated→idle, ∝hunger)
					"en":  snappedf(float(m.get("energy", 1.0)), 0.001),
				}
			"HeadingProbe":
				mods[mod_id] = {
					"tgt":  snappedf(float(m.get("target_deg", 0.0)), 0.1),   # commanded world heading (deg)
					"pbear":snappedf(float(m.get("pbearing", 0.0)), 0.001),   # egocentric bearing to it (track err)
					"held": int(m.get("held", 0)),
				}
			"GoalBelief":
				mods[mod_id] = {
					"bx":    snappedf(float(m.get("belief_x", 0.0)), 0.001),    # egocentric belief +right
					"by":    snappedf(float(m.get("belief_y", 0.0)), 0.001),    # +forward
					"conf":  snappedf(float(m.get("confidence", 0.0)), 0.001),  # decays without a fix
					"perc":  bool(m.get("perceiving", false)),                  # confident perception this tick
				}
			"HeadingController":
				mods[mod_id] = {
					"bearing": snappedf(float(m.get("bearing", 0.0)), 0.001),     # egocentric, 0=facing heading
					"gain":    snappedf(float(m.get("learned_gain", 0.0)), 0.001),# effective turn gain
					"kbody":   snappedf(float(m.get("k_body", 0.0)), 0.0001),     # learned |ω|/|steer| (converges)
					"steer":   snappedf(float(m.get("hc_steer", 0.0)), 0.01),
					"thrust":  snappedf(float(m.get("hc_thrust", 0.0)), 0.01),
					"nav_on":  bool(m.get("nav_on", false)),
				}
				if m.get("learn_advance", false):
					mods[mod_id]["ebin"] = int(m.get("err_bin", -1))
					mods[mod_id]["tact"] = int(m.get("thrust_act", -1))
					mods[mod_id]["arew"] = snappedf(float(m.get("adv_reward", 0.0)), 0.0001)
					mods[mod_id]["aspr"] = snappedf(float(m.get("adv_spread", 0.0)), 0.0001)
					mods[mod_id]["acov"] = snappedf(float(m.get("adv_cov", 0.0)), 0.001)
			"NeurochemState":
				mods[mod_id] = {
					"da":     snappedf(float(m.get("dopamine", 0.0)), 0.001),
					"ht":     snappedf(float(m.get("serotonin", 0.0)), 0.001),
					"eps":    snappedf(float(m.get("epsilon_b_scale", 1.0)), 0.01),
					"hits":   int(m.get("total_hits", 0)),
					"miss":   int(m.get("total_misses", 0)),
					"bricks": int(m.get("total_bricks", 0)),
				}
			"EPM":
				var epm_diag := {
					"nodes":  int(m.get("node_count", 0)),
					"baked":  int(m.get("baked_count", 0)),
					"tle":    snappedf(float(m.get("tle", 0.0)), 0.001),
					"novel":  bool(m.get("is_novel", false)),
					"wid":    int(m.get("winner_id", -1)),   # current winner node (loop-closure / place-id)
				}
				# v5.4.L Phase 2a — pass through last_latent for the slow
				# consensus EPM (or any EPM the user wants to inspect
				# variance on).  Filtered to slow modality groups so we
				# don't bloat JSONL with high-dim per-modality latents
				# every tick — `_consensus_long` is the conventional id
				# for the slow EPM in the lifecycle pipeline.
				if "consensus_long" in mod_id and m.has("last_latent"):
					epm_diag["last_latent"] = m["last_latent"]
				# v5.4.L Diagnostic B — winner histogram per EPM.
				# Identifies premature GNG saturation across modalities.
				if m.has("winner_top8"):
					epm_diag["winner_top8"]      = m["winner_top8"]
					epm_diag["winner_total"]     = int(m.get("winner_total_samples", 0))
				mods[mod_id] = epm_diag
			"LateralVoter":
				# Compact trust map: round each weight to 0.001 so JSONL stays
				# scannable; downstream summariser can compute group totals.
				var tw_in: Dictionary = m.get("trust_weights", {})
				var tw_out := {}
				for k in tw_in:
					tw_out[k] = snappedf(float(tw_in[k]), 0.001)
				mods[mod_id] = {
					"tle":    snappedf(float(m.get("fused_tle", 0.0)), 0.001),
					"mod":    str(m.get("active_modality", "")),
					"awin":   int(m.get("active_winner_id", -1)),
					"trust":  tw_out,
					"dim":    int(m.get("latent_dim", 0)),
					"lnorm":  snappedf(float(m.get("latent_norm", 0.0)), 0.01),
					"lnz":    int(m.get("latent_near_zero", 0)),
				}
			"HomeostaticDrive":
				var errs: Dictionary = m.get("errors", {})
				mods[mod_id] = {
					"urg":    snappedf(float(m.get("urgency", 0.0)), 0.001),
					"errors": errs
				}
			"MotorEPM", "MotorEPMv2":
				mods[mod_id] = {
					"mtle":   snappedf(float(m.get("motor_tle", 0.0)), 0.0001),
					"lgain":  snappedf(float(m.get("loop_gain", 0.0)), 0.0001),
					"cogsteer": snappedf(float(m.get("cog_steer", 0.0)), 0.001),
					"cogmsgs":  int(m.get("cog_steer_msgs", 0)),
					"bored":  snappedf(float(m.get("boredom", 0.0)), 0.001),
					"intr":   snappedf(float(m.get("interest", 0.0)), 0.001),
					"hngr":   snappedf(float(m.get("hunger", 0.0)), 0.001),
					"bstreak": int(m.get("boredom_streak", 0)),
					"tcx":    snappedf(float(m.get("tc_x", 0.0)), 0.001),
					"tcy":    snappedf(float(m.get("tc_y", 0.0)), 0.001),
				}
			"DistressDrive":
				# Phase 6.9.A boredom signal + 3 components: confirms the signal
				# is low free-swimming and rises during a pin (vs the stuck telemetry).
				mods[mod_id] = {
					"bored":  snappedf(float(m.get("boredom", 0.0)), 0.001),
					"mis":    snappedf(float(m.get("mismatch", 0.0)), 0.001),
					"noprog": snappedf(float(m.get("no_progress", 0.0)), 0.001),
					"intr":   snappedf(float(m.get("interest", 0.0)), 0.001),
					"clr":    snappedf(float(m.get("clearance", 0.0)), 0.001),
					"grn":    snappedf(float(m.get("green", 0.0)), 0.001),
					"tlesp":  snappedf(float(m.get("tle_spike", 0.0)), 0.001),
					"stale":  snappedf(float(m.get("staleness", 0.0)), 0.001),
					"minv":   snappedf(float(m.get("motion_inv", 0.0)), 0.001),
					"mraw":   snappedf(float(m.get("motion_raw", 0.0)), 0.00001),
					"mema":   snappedf(float(m.get("motion_ema", 0.0)), 0.00001),
					"supp":   snappedf(float(m.get("suppress", 0.0)), 0.001),
				}
			"ActionDecoder":
				mods[mod_id] = {
					"accel":      snappedf(float(m.get("accel", 0.0)), 0.01),
					"greedy":     snappedf(float(m.get("greedy_accel", 0.0)), 0.01),
					# 2026-06-19 corridor probe — committed action this option.
					"auth":       snappedf(float(m.get("authority", 1.0)), 0.001),
					"pH":         snappedf(float(m.get("plan_entropy", 0.0)), 0.001),     # explore(~1)/exploit(~0)
					"pconf":      snappedf(float(m.get("plan_confidence", 1.0)), 0.001),
					# 2026-06-21 corridor-flat diagnosis: pragmatic-landscape liveness.
					"pref_obs":   snappedf(float(m.get("pref_obs", 0.0)), 0.0001),     # cy reaching the planner
					"obs_known":  int(m.get("obs_states_known", 0)),                   # value-table size
					"scr_spread": snappedf(float(m.get("score_spread", 0.0)), 0.0001), # max-min action scores (flat≈0)
					"nval_spread":snappedf(float(m.get("nodeval_spread", 0.0)), 0.0001),# node_value range over states
					"cidx":       int(m.get("commit_idx", -1)),
					"cthr":       snappedf(float(m.get("commit_thrust", 0.0)), 0.01),
					"state":      int(m.get("state_node", -1)),
					"fwd_sz":     int(m.get("fwd_model_size", 0)),
					"probe":      bool(m.get("probe", false)),
					"val_sz":     int(m.get("valence_size", 0)),
					"heb_sz":     int(m.get("hebbian_size", 0)),
					"chunk_id":   int(m.get("active_chunk_id", -1)),
					"chunk_left": int(m.get("chunk_remaining",  0)),
					"chunk_disp": int(m.get("chunk_dispatch_count", 0)),
					# v5.4 Phase A
					"entry_hist": int(m.get("entry_history_size", 0)),
					"entry_in":   int(m.get("entry_history_seen", 0)),
					"em_disp":    int(m.get("entry_match_dispatches", 0)),
					# v5.4 Phase C
					"man_disp":   int(m.get("manual_dispatches", 0)),
					"man_miss":   int(m.get("manual_dispatch_misses", 0)),
					# v5.4 Phase E
					"gate_scr":   int(m.get("dispatches_gated_score", 0)),
					"gate_em":    int(m.get("dispatches_gated_match", 0)),
					# v5.4.J — chunk armed-state Schmitt gate
					"armed":      int(m.get("chunks_armed_count", 0)),
					"unarmed_blk":int(m.get("dispatches_blocked_unarmed", 0)),
					# v5.4.L — chunk dispatch age gate
					"young_blk":  int(m.get("dispatches_blocked_too_young", 0)),
				}
			"SequenceGNG":
				mods[mod_id] = {
					"nodes":  int(m.get("node_count", 0)),
					"baked":  int(m.get("baked_count", 0)),
					"motif":  int(m.get("current_motif", -1))
				}
			"MotorRepertoire":
				# Pass through chunks_summary from the C++ side for offline
				# inspection (length / use_count / drive_delta / accel_head).
				mods[mod_id] = {
					"chunks":      int(m.get("chunk_count", 0)),
					"chunks_lib":  m.get("chunks", []),
					# v5.3 Phase B
					"intent_hist": int(m.get("intent_history_size", 0)),
					"intents_in":  int(m.get("intents_received_total", 0)),
					# v5.4 Phase A
					"epi_in":      int(m.get("episodic_proposals_ingested", 0)),
					# v5.4 Phase G
					"pos_hits":    int(m.get("position_hits_credited", 0)),
					# v5.4 Phase H — chunk lifecycle.
					"pruned":      int(m.get("chunks_pruned_total", 0)),
					"elig_cred":   int(m.get("eligibility_credits_total", 0)),
					"trace_n":     int(m.get("chunk_dispatch_trace_size", 0)),
				}
			"HomeokineticExploration":
				mods[mod_id] = {
					"eps":    int(m.get("episodes_armed", 0)),
					"active": bool(m.get("active", false))
				}
			"DualEMADetector":
				mods[mod_id] = {
					"fires":  int(m.get("fire_count", 0)),
					"sema":   snappedf(float(m.get("short_ema", 0.0)), 0.001),
					"lema":   snappedf(float(m.get("long_ema", 0.0)), 0.001),
				}
			"EventConjunction":
				mods[mod_id] = {
					"fires":  int(m.get("fire_count", 0)),
					"in":     int(m.get("inputs_seen", 0)),
					"last":   m.get("last_fire_ticks", []),
				}
			"ChunkAbortGate":
				mods[mod_id] = {
					"aborts": int(m.get("aborts_total", 0)),
					"base_mean": snappedf(float(m.get("baseline_mean", 0.0)), 0.0001),
					"base_var":  snappedf(float(m.get("baseline_var", 0.0)), 0.0001),
					"last_surp": snappedf(float(m.get("last_surprise", 0.0)), 0.0001),
				}
			"EpisodicCapture":
				mods[mod_id] = {
					"kf":     int(m.get("keyframes_seen", 0)),
					"int_in": int(m.get("intents_seen", 0)),
					"prop":   int(m.get("proposals_emitted", 0)),
					"rew":    int(m.get("rewards_seen", 0)),
					"fill":   int(m.get("buffer_fill", 0)),
					"last":   int(m.get("last_intent_index", -1)),
				}
			"ChunkOutcomeGate":
				mods[mod_id] = {
					"aborts":     int(m.get("aborts_total", 0)),
					"act_in":     int(m.get("action_msgs", 0)),
					"chunk_id":   int(m.get("active_chunk_id", -1)),
					"ticks":      int(m.get("ticks_in_chunk", 0)),
					"start":      snappedf(float(m.get("signal_at_start", 0.0)), 0.001),
					"now":        snappedf(float(m.get("current_signal", 0.0)), 0.001),
				}
			"KeyframeAverager":
				# Phase v5.2 — surface buffer state + I/O counters so the
				# UI inspector can show "what is this primitive doing".
				var mean_arr = m.get("last_mean", PackedFloat64Array())
				var mean_first := 0.0
				if mean_arr.size() > 0:
					mean_first = float(mean_arr[0])
				mods[mod_id] = {
					"fill":     int(m.get("window_fill", 0)),
					"win":      int(m.get("window_size", 0)),
					"dim":      int(m.get("payload_dim", 0)),
					"mean0":    snappedf(mean_first, 0.001),
					"in":       int(m.get("total_inputs_seen", 0)),
					"out":      int(m.get("total_publishes", 0)),
				}
			"FaderController":
				# Phase 6.6.audit — α + its three drivers (surprise, familiarity, boredom).
				mods[mod_id] = {
					"a":     snappedf(float(m.get("alpha", 0.0)), 0.001),
					"a_tgt": snappedf(float(m.get("alpha_target", 0.0)), 0.001),
					"surp":  snappedf(float(m.get("surprise_scalar", 0.0)), 0.001),
					"fam":   snappedf(float(m.get("familiarity_scalar", 0.0)), 0.001),
					"bor":   snappedf(float(m.get("boredom_term", 0.0)), 0.001),
					# v5.4.M premotor_certainty diag
					"pol_in":   int(m.get("policy_msgs_received", 0)),
					"pre_H":    snappedf(float(m.get("last_premotor_entropy", 0.0)), 0.001),
					"pre_N":    int(m.get("last_premotor_n_intents", 0)),
					# v5.4 Phase B
					"act_in":   int(m.get("action_msgs_received", 0)),
					"ck_act":   int(m.get("chunk_active_ticks", 0)),
					"ck_id":    int(m.get("current_chunk_id", -2)),
				}
			"DescendingPredictor":
				# Phase 6.6.audit — per-target predictor health.  err/norm
				# ratio close to 1 = no learning.  Decreasing err_ema means
				# the predictor is fitting.
				var pt_in: Dictionary = m.get("per_target", {})
				var pt_out := {}
				for k in pt_in:
					var v: Dictionary = pt_in[k]
					var n: float = float(v.get("norm", 1.0))
					var e: float = float(v.get("err", 1.0))
					pt_out[k] = {
						"err":  snappedf(e, 0.001),
						"norm": snappedf(n, 0.001),
						"r":    snappedf(e / max(n, 1e-6), 0.001),
						"shape": [int(v.get("W_rows", 0)), int(v.get("W_cols", 0))],
					}
				mods[mod_id] = {
					"n":  int(m.get("target_count", 0)),
					"per": pt_out
				}
			"Premotor":
				# Phase 6.6.O diag — verify BC isn't collapsing to one row.
				var rn_in: Array = m.get("W_row_norms", [])
				var rn_out := []
				for v in rn_in:
					rn_out.append(snappedf(float(v), 0.001))
				mods[mod_id] = {
					"n":          int(m.get("n_intents", 0)),
					"bc_total":   int(m.get("bc_total_updates", 0)),
					"bc_hist":    m.get("bc_intent_counts", []),
					"chosen_hist": m.get("chosen_intent_counts", []),
					"last_chosen": int(m.get("last_chosen", -1)),
					"last_bc":     int(m.get("last_bc_intent", -1)),
					"H":           snappedf(float(m.get("last_entropy", 0.0)), 0.001),
					# v5.4.M Diagnostic B — windowed entropy of CHOSEN
					# intents (rut signal).  Low = stuck on one intent.
					"H_win":       snappedf(float(m.get("chosen_window_entropy", 0.0)), 0.001),
					"H_win_n":     int(m.get("chosen_window_size", 0)),
					"W":           snappedf(float(m.get("W_total_norm", 0.0)), 0.01),
					"W_rows":      rn_out,
					# Phase v5.1 MC actor-critic.
					"mc_eps":     int(m.get("mc_episodes_seen", 0)),
					"mc_traj":    int(m.get("mc_trajectory_size", 0)),
					"mc_ret":     snappedf(float(m.get("mc_last_return", 0.0)), 0.01),
					"mc_mu":      snappedf(float(m.get("mc_return_mean", 0.0)), 0.01),
					"mc_sd":      snappedf(float(m.get("mc_return_std", 0.0)), 0.01),
					# v5.3 Phase B
					"overrides":  int(m.get("total_overrides_used", 0)),
					# v5.3 Phase C
					"aligned":    int(m.get("aligned_rewards_seen", 0)),
				}

	# Scent stats: sum (proxy for distance to nearest nutrient) and spread
	# (max - min across nostrils — the directional differential).
	var s_sum := 0.0
	var s_min := INF
	var s_max := 0.0
	for v in _last_scent:
		s_sum += v
		s_min = minf(s_min, v)
		s_max = maxf(s_max, v)
	if _last_scent.is_empty():
		s_min = 0.0

	# Ground-truth egocentric food bearing (observer-only homing telemetry):
	# +1 = nearest food hard right, -1 = hard left, ~0 = ahead/behind.  Enables
	# corr(fbear, action_decoder.greedy) as the sensitive ignition/homing signal.
	var _diag_world := get_parent() as Node3D
	var _fbear := 0.0
	if _diag_world and _diag_world.has_method("nearest_food_local_lateral"):
		_fbear = _diag_world.nearest_food_local_lateral(global_position, global_transform.basis)
	var _food_left := -1
	if _diag_world and _diag_world.has_method("active_food_count"):
		_food_left = _diag_world.active_food_count()

	var line := {
		"t":       tick_counter,
		"rt":      _runtime_tag,
		"pos":     [snappedf(global_position.x, 0.01), snappedf(global_position.z, 0.01)],
		"heading": snappedf(rad_to_deg(heading), 0.1),
		"vrgb":    [snappedf(_last_vis_mean_rgb[0], 0.1), snappedf(_last_vis_mean_rgb[1], 0.1), snappedf(_last_vis_mean_rgb[2], 0.1)],
		"fbear":   snappedf(_fbear, 0.001),
		"food_left": _food_left,
		"accel":   snappedf(accel, 0.01),
		"fwd_v":   snappedf(_forward_speed, 0.01),
		"ang_v":   snappedf(_angular_velocity, 0.01),
		"energy":  snappedf(energy, 0.001),
		"menergy": [snappedf(_motor_energy_left, 0.01), snappedf(_motor_energy_right, 0.01)],
		"hits":    hits_total,
		"tsf":     _ticks_since_food,
		"scent":   {"sum": snappedf(s_sum, 0.001),
					"max": snappedf(_last_scent_max, 0.001),         # position-only centre (the published scent_max)
					"spread": snappedf(s_max - s_min, 0.001),        # ring directional differential (0 when ring bypassed)
					# v5.4.M intentionality probe — per-tick scent gradient
					# (short_EMA − long_EMA).  Positive = approaching food
					# scent source.  Used by intentionality_probe.py to
					# compute P(chosen_intent | scent_grad) conditional
					# distributions and detect context-dependent action.
					"grad": snappedf(_scent_short_ema - _scent_long_ema, 0.0001),
					"green_frac": snappedf(_last_green_frac, 0.0001)},
		"wmax":    snappedf(_last_whisker_max, 0.01),
		# Phase 6.5.19 Part A — stuck-diagnosis counters (cumulative).
		"stuck": {
			"total": _stuck_total_ticks,
			"wall":  _stuck_wall_ticks,
			"refr":  _stuck_refrac_ticks,
			"zero":  _stuck_zero_steer_ticks,
			"pin":   _stuck_actual_pin_ticks,
			"sev":   snappedf(_stuck_severity, 0.01),
			"sev_a": snappedf(_stuck_severity_actual, 0.01),
		},
		# Honest locomotion (2026-07-08 audit) — body-model-agnostic, cumulative.
		"effort":     snappedf(_motor_effort_sum / maxf(1.0, float(_avg_count)), 0.001),  # mean |L|+|R| ∈[0,2]
		"pause_frac": snappedf(float(_pause_ticks) / maxf(1.0, float(_avg_count)), 0.001),
		"fwd_mean":   snappedf(_speed_sum_for_avg / maxf(1.0, float(_avg_count)), 0.001),
		"body_max":   snappedf(_stuck_reference_speed(), 0.01),
		"n":          _avg_count,   # sample count → lets a parser window the cumulative means over time
		"spikes": _spike_count_total,
		# Phase 6.5.19 Part B — adaptive threshold values (current).
		"adapt": {
			"scent_diff_thr": snappedf(_scent_diff_mean_ema + _ADAPT_N_STDDEV * sqrt(maxf(_scent_diff_var_ema, _ADAPT_MIN_STDDEV * _ADAPT_MIN_STDDEV)), 0.0001),
			"whisker_thr":    snappedf(_whisker_mean_ema + _ADAPT_N_STDDEV * sqrt(maxf(_whisker_var_ema, _ADAPT_MIN_STDDEV * _ADAPT_MIN_STDDEV)), 0.001),
			"fallback":       _adaptive_fallback_active,
		},
		"modules": mods
	}
	print(JSON.stringify(line))

# ---------------------------------------------------------------------------
# Nutrient hit callback (called by the_cell_world.gd)
# ---------------------------------------------------------------------------

# Phase 6.6.D.5 — gates inferential events (scent-progress hit, whisker miss,
# stuck-pin wall_stuck) on whether the active config asks for graph-resident
# reflex modules to emit them instead.  The genuine collision-driven
# events.hit in on_nutrient_hit() bypasses this gate — that's the ground-
# truth reward signal, not an inference, and must always fire.
func _publish_reflex_event(name: String, intensity: float) -> void:
	if ExperimentConfig.resolve_reflex_modular():
		return
	brain.publish_event(name, intensity)

# v5.4.L Diagnostic A — emit a tick-stamped raw-sensor sample so an
# external probe can pair AT_HIT and POST_HIT_60 samples by hit_id and
# compute pre-vs-post distributions of the underlying signal.  All
# values are raw (not encoded) — the question is whether the substrate's
# perceptual inputs THEMSELVES discriminate "approaching food" from
# "just ate, food gone".  If yes, encoder failures are tunable.  If
# no, the architecture needs different sensors, not a different gate.
func _emit_hit_window_sample(label: String, hit_id: int) -> void:
	print(JSON.stringify({
		"t":          tick_counter,
		"event":      "HIT_WINDOW",
		"label":      label,
		"hit_id":     hit_id,
		"scent_max":  snappedf(_last_scent_max, 0.0001),   # position-only centre scent
		"scent_grad": snappedf(_scent_short_ema - _scent_long_ema, 0.0001),
		"green_frac": snappedf(_last_green_frac, 0.0001),
		"fwd_v":      snappedf(_forward_speed, 0.001),
	}))

func _drain_pending_hit_samples() -> void:
	if _pending_hit_samples.is_empty():
		return
	# Walk in reverse so removal-during-iteration is safe.
	for i in range(_pending_hit_samples.size() - 1, -1, -1):
		var entry: Dictionary = _pending_hit_samples[i]
		if tick_counter >= int(entry["target_tick"]):
			_emit_hit_window_sample(String(entry["label"]), int(entry["hit_id"]))
			_pending_hit_samples.remove_at(i)

func on_nutrient_hit() -> void:
	brain.publish_event("hit", 1.0)
	# GROUND-TRUTH consummatory event — fires ONLY on a real nutrient collision, distinct from the
	# events.hit above which is ALSO published on the scent-progress inference (short>long×1.5) all
	# through an approach. Modules that need the real eat scale (e.g. RunTumbleNav's eat_scent
	# self-calibration) subscribe to events.eat, not the overloaded events.hit.
	brain.publish_event("eat", 1.0)
	energy = 1.0   # 2026-06-30 (operator) — an eat refuels to FULL → clean homeostatic sawtooth
	hits_total += 1
	_ticks_since_food = 0   # Phase 6.9 survival telemetry
	print(JSON.stringify({"t": tick_counter, "event": "HIT",
		"hits_total": hits_total, "energy": snappedf(energy, 0.001)}))
	# v5.4.L Diagnostic A — emit raw-sensor triple AT the hit moment,
	# then schedule a follow-up emission 60 ticks later (post-eat,
	# food gone, agent moved past).  Pair-difference of the two
	# samples answers "is the raw signal discriminative for
	# food-approach vs post-eat?"  If pre/post triples are nearly
	# identical, no encoder can fix the gate.
	_emit_hit_window_sample("AT_HIT", hits_total)
	_pending_hit_samples.append({"target_tick": tick_counter + 60,
		"label": "POST_HIT_60", "hit_id": hits_total})

# 2026-06-19 — CORRIDOR 1-D probe: teleport the bug back to the start of the
# corridor, re-face the food (heading 0 = −Z), and zero all momentum, so each
# eat begins a FRESH trial.  A run is then a sequence of trials; time-to-food
# per trial is the learning curve.  Called by the world's consume_nutrient in
# corridor mode.
func corridor_reset() -> void:
	var start: Vector3 = _corridor_start if _corridor_start_set else Vector3(0, global_position.y, 0)
	global_position   = start
	heading           = 0.0
	rotation.y        = 0.0
	velocity          = Vector3.ZERO
	_forward_speed    = 0.0
	_angular_velocity = 0.0
	_ticks_since_food = 0
	print(JSON.stringify({"t": tick_counter, "event": "TRIAL_RESET",
		"trial": hits_total}))

# ---------------------------------------------------------------------------
# Toggle graph panel
# ---------------------------------------------------------------------------

func _input(event: InputEvent) -> void:
	# ` / F1 is now handled universally by ogma_graph_panel.gd::_unhandled_input.
	# Keep V here (Cell-specific top-down/FPV camera toggle).
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_V:
				_toggle_camera_view()
			KEY_C:
				_toggle_chunk_probe()
			KEY_J:
				_toggle_planner_overlay()

# ---------------------------------------------------------------------------
# Manual chunk probe — KEY_C toggles a dialog where you type a chunk_id from
# the MotorRepertoire library.  On confirm, sets action_decoder.manual_chunk_id
# live via brain.apply_patch.  The existing force-dispatch path re-fires the
# chunk every tick (bypassing score gates + Wilson-CI demote) so it loops
# until you press C again, which closes the dialog and resets manual_chunk_id
# to 0 (sentinel: normal scoring dispatch resumes).
#
# Status indicator: when probe is active a small label overlays the top-right
# corner showing "Chunk Probe: id=N" so you can see the probe state without
# needing the graph panel open.
# ---------------------------------------------------------------------------

var _chunk_probe_active:   bool         = false
var _chunk_probe_chunk_id: int          = 0
var _chunk_probe_dialog:   AcceptDialog = null
var _chunk_probe_status_lbl: Label      = null

func _toggle_chunk_probe() -> void:
	if _chunk_probe_active:
		_end_chunk_probe()
	else:
		_open_chunk_probe_dialog()

func _open_chunk_probe_dialog() -> void:
	# Dismiss any half-open dialog from a prior press.
	if _chunk_probe_dialog != null and is_instance_valid(_chunk_probe_dialog):
		_chunk_probe_dialog.queue_free()
		_chunk_probe_dialog = null

	var chunks_summary: String = _chunk_probe_library_summary()
	var dlg := AcceptDialog.new()
	dlg.title = "Chunk Probe — fire a chunk by id"
	dlg.dialog_hide_on_ok = true
	dlg.add_cancel_button("Cancel")
	var vb := VBoxContainer.new()
	var info := Label.new()
	info.text = "Type chunk_id to force-dispatch (loops until C pressed again):"
	vb.add_child(info)
	var edit := LineEdit.new()
	edit.placeholder_text = "e.g. -1 (seed) or 7 (organic)"
	edit.custom_minimum_size = Vector2(360, 0)
	edit.text = str(_chunk_probe_chunk_id) if _chunk_probe_chunk_id != 0 else ""
	vb.add_child(edit)
	var lib := Label.new()
	lib.text = chunks_summary
	lib.add_theme_font_size_override("font_size", 10)
	lib.add_theme_color_override("font_color", Color(0.75, 0.75, 0.75))
	lib.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	lib.custom_minimum_size = Vector2(360, 60)
	vb.add_child(lib)
	var hint := Label.new()
	hint.text = "Hint: 0 = disabled.  Press C again at any time to stop probe."
	hint.add_theme_font_size_override("font_size", 9)
	hint.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	vb.add_child(hint)
	dlg.add_child(vb)
	add_child(dlg)
	dlg.confirmed.connect(func ():
		var s: String = edit.text.strip_edges()
		if s.is_empty():
			return
		var parsed: int = s.to_int()
		_begin_chunk_probe(parsed))
	dlg.canceled.connect(func ():
		# explicit cancel → leave probe inactive
		pass)
	# Auto-focus the LineEdit so the user can just type and press Enter.
	dlg.popup_centered(Vector2(400, 220))
	edit.grab_focus()
	# Pressing Enter inside the LineEdit triggers the dialog's OK action.
	edit.text_submitted.connect(func (_t: String): dlg.get_ok_button().pressed.emit())
	_chunk_probe_dialog = dlg

func _begin_chunk_probe(chunk_id: int) -> void:
	if chunk_id == 0:
		# Sentinel "disabled" — treat as a no-op (don't enter probe mode).
		return
	if brain == null or not brain.has_method("apply_patch"):
		push_error("BodyController: chunk probe unavailable — brain has no apply_patch")
		return
	var result: Dictionary = brain.apply_patch({
		"op":    "set_param",
		"id":    "action_decoder",
		"key":   "manual_chunk_id",
		"value": chunk_id,
	})
	if not bool(result.get("success", false)):
		push_error("BodyController: chunk probe set_param failed: %s"
			% String(result.get("error", "?")))
		return
	_chunk_probe_active   = true
	_chunk_probe_chunk_id = chunk_id
	_show_chunk_probe_status()

func _end_chunk_probe() -> void:
	# Always try to clear manual_chunk_id back to the sentinel 0 even if the
	# probe state was inconsistent — the user pressed C to STOP, intent is
	# unambiguous.
	if brain != null and brain.has_method("apply_patch"):
		brain.apply_patch({
			"op":    "set_param",
			"id":    "action_decoder",
			"key":   "manual_chunk_id",
			"value": 0,
		})
	if _chunk_probe_dialog != null and is_instance_valid(_chunk_probe_dialog):
		_chunk_probe_dialog.queue_free()
		_chunk_probe_dialog = null
	_chunk_probe_active   = false
	_chunk_probe_chunk_id = 0
	_hide_chunk_probe_status()

func _chunk_probe_library_summary() -> String:
	# Best-effort enumeration of the current chunk library so the user can
	# pick an id without alt-tabbing to the graph panel.  Pulls the chunks
	# list from MotorRepertoire's diag (the same dict body_controller emits
	# to JSONL).  Sorted by replay_hits desc so the most-used chunks lead.
	if brain == null or not brain.has_method("get_module_metrics"):
		return "Library not introspectable (older host).  Type any id."
	var metrics: Dictionary = brain.get_module_metrics()
	var repertoire: Dictionary = {}
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		if String(m.get("type", "")) == "MotorRepertoire":
			repertoire = m
			break
	if repertoire.is_empty():
		return "No MotorRepertoire module in graph."
	var entries: Array = repertoire.get("chunks", [])
	if entries.is_empty():
		return "Library is empty.  Let chunks crystallise first."
	# v5.4.K — sort by Beta-prior score, not raw replay_hits.  Score
	# = (hits_during + replay_hits + 1) / (hits_during + replay_hits +
	# replay_misses + 2).  A just-fired chunk (rh=1, rm=0) scores 0.667;
	# never-fired chunks at rh=0 score 0.5 (the prior).  Sorting by rh
	# alone buries fresh-success chunks below years-old high-volume
	# winners — the operator who just SAW a chunk produce a hit then
	# can't find it in the list.  Beta-prior score also tracks the same
	# metric the dispatcher uses, so the operator sees the dispatcher's
	# own ranking.
	var sortable: Array = []
	for c in entries:
		var hd: int   = int(c.get("hits_during", 0))
		var rh: float = float(c.get("replay_hits", 0.0))
		var rm: float = float(c.get("replay_misses", 0.0))
		var score: float = (float(hd) + rh + 1.0) / (float(hd) + rh + rm + 2.0)
		sortable.append({
			"id":    int(c.get("id", 0)),
			"hd":    hd,
			"rh":    rh,
			"rm":    rm,
			"score": score,
			"len":   int(c.get("length", c.get("intent_length", 0))),
		})
	sortable.sort_custom(func (a: Dictionary, b: Dictionary) -> bool:
		return a["score"] > b["score"])
	var rows: Array[String] = []
	for c in sortable.slice(0, 12):
		rows.append("id=%-5d  score=%.3f  rh=%6.1f  rm=%5.1f  len=%d"
			% [c["id"], c["score"], c["rh"], c["rm"], c["len"]])
	var s: String = "Library (%d chunks, sorted by Beta-prior score):\n" % entries.size()
	s += "\n".join(rows)
	if entries.size() > 12:
		s += "\n+%d more" % (entries.size() - 12)
	return s

func _show_chunk_probe_status() -> void:
	if _chunk_probe_status_lbl == null or not is_instance_valid(_chunk_probe_status_lbl):
		var lbl := Label.new()
		lbl.add_theme_font_size_override("font_size", 14)
		lbl.add_theme_color_override("font_color", Color(0.95, 0.85, 0.35))
		lbl.add_theme_color_override("font_outline_color", Color(0, 0, 0))
		lbl.add_theme_constant_override("outline_size", 4)
		lbl.anchor_left  = 1.0
		lbl.anchor_right = 1.0
		lbl.anchor_top   = 0.0
		lbl.anchor_bottom= 0.0
		lbl.offset_left  = -240
		lbl.offset_top   = 12
		lbl.offset_right = -12
		lbl.offset_bottom= 40
		var cl := CanvasLayer.new()
		cl.layer = 50
		cl.name = "ChunkProbeStatusLayer"
		cl.add_child(lbl)
		get_tree().root.add_child(cl)
		_chunk_probe_status_lbl = lbl
	_chunk_probe_status_lbl.text = "Chunk Probe: id=%d  [press C to stop]" \
		% _chunk_probe_chunk_id
	_chunk_probe_status_lbl.visible = true

func _hide_chunk_probe_status() -> void:
	if _chunk_probe_status_lbl != null and is_instance_valid(_chunk_probe_status_lbl):
		_chunk_probe_status_lbl.visible = false

# ---------------------------------------------------------------------------
# Phase 6.2 — terrain probe
#
# Four downward RayCast3Ds at body-relative offsets (right, left, front, back)
# sample ground height each tick.  We resolve them into a 4-vector that the
# brain consumes via reality.proprio.terrain:
#   [0] slope_x  = (h_right - h_left) / (2 * offset)
#   [1] slope_z  = (h_front - h_back) / (2 * offset)
#   [2] roughness = stddev across the 4 samples
#   [3] height_below_body = mean ground height − body.y (always negative when
#                           the body is above the ground)
# ---------------------------------------------------------------------------

func _build_terrain_probes() -> void:
	const _OFFSETS := [
		Vector3( _TERRAIN_PROBE_OFFSET, 0.0, 0.0),
		Vector3(-_TERRAIN_PROBE_OFFSET, 0.0, 0.0),
		Vector3(0.0, 0.0,  _TERRAIN_PROBE_OFFSET),
		Vector3(0.0, 0.0, -_TERRAIN_PROBE_OFFSET),
	]
	for i in range(_OFFSETS.size()):
		var rc := RayCast3D.new()
		rc.name = "TerrainProbe%d" % i
		rc.position = _OFFSETS[i]
		rc.target_position = Vector3(0.0, -_TERRAIN_PROBE_LENGTH, 0.0)
		rc.collide_with_areas  = false
		rc.collide_with_bodies = true
		# Default mask hits floor + terrain.  Body sits on its own
		# CharacterBody3D and isn't on the same collision layer as static
		# geometry, so we don't self-hit.
		rc.enabled = true
		add_child(rc)
		terrain_probes.append(rc)

# Phase 6.9.B — FRONT MOUTH.  A small Area3D box just ahead of the body (forward =
# -Z).  Food is a StaticBody obstacle the bug bumps; it is EATEN only when the
# mouth box touches it (so the bug must FACE the food to eat — couples the eat
# reward to the front-relative scent bearing the critic steers on).  The mouth
# extends past the capsule (radius 0.4) so a head-on approach eats before the body
# bumps; an off-axis approach bumps the body (obstacle) without eating.
func _build_mouth() -> void:
	# Mouth GEOMETRY is metadata-driven (mouth_offset_z / mouth_size), defaulting to the
	# front-only aperture below. A scalar run-and-tumble forager climbs CENTRE scent, so it
	# brings the food to the body CENTRE — not the far-forward mouth — and the narrow front box
	# misses laterally-offset food on the pass (|fbear|~0.85 near food → drive-past). A config
	# can RECENTER the box toward the body (a contact-eat zone matching the honest scalar
	# morphology: body r=0.4 + food r=0.5 ⇒ ~0.9 contact) to land those closes.
	var meta: Dictionary = ExperimentConfig.read_cell_config_metadata()
	var m_off_z: float = float(meta.get("mouth_offset_z", -0.75))
	var m_size: Array = meta.get("mouth_size", [1.2, 1.3, 0.6])
	if m_size.size() < 3:
		m_size = [1.2, 1.3, 0.6]
	var mouth := Area3D.new()
	mouth.name = "Mouth"
	mouth.position = Vector3(0.0, 0.65, m_off_z)     # default −0.75 (front); recenter toward 0 for contact-eat
	mouth.collision_layer = 0                       # nothing needs to detect the mouth
	mouth.collision_mask  = 2                       # detect layer-2 bodies = FOOD (non-blocking)
	mouth.monitoring = true
	var box := BoxShape3D.new()
	# 2026-06-21 — front aperture, kept HITTABLE (a too-small mouth starved the
	# learning bootstrap: 0.3-0.6 wide → ~0 hits → never learns).  Two targeted
	# fixes instead: (1) entirely AHEAD of the body front (z −0.45..−1.05, no
	# longer the old z−0.2 that overlapped the body), and (2) the HEAD-ON eat
	# gate in _on_mouth_body_entered (eat only while moving FORWARD) — together
	# these kill the backing/turning-into-food eats that corrupted the objective,
	# WITHOUT shrinking the aperture below hittability.  Width ±0.5 (≈ original).
	# 2026-06-26 (operator obs) — the box was y 0.6–1.3 but food is radius-0.5 spheres
	# centred at y≈0.5 (span y 0.0–1.0): the mouth only overlapped the food's narrow top
	# cap, so a grazing pass went OVER the food and missed.  Cover the food's full height
	# (y 0.0–1.3) + widen slightly (±0.6) so any FRONT touch scores; z stays front-only
	# (−0.45..−1.05) so facing-the-food is still required (no backing/side eats).
	box.size = Vector3(float(m_size[0]), float(m_size[1]), float(m_size[2]))
	var cs := CollisionShape3D.new()
	cs.shape = box
	mouth.add_child(cs)
	# 2026-06-21 — OVERLAP-based eat (operator obs): body_entered fires only on the
	# ENTRY edge, so food that SPAWNS inside the mouth (respawn in the bug's path)
	# never triggers it → the bug freezes on un-eatable food.  Poll the mouth's
	# overlapping bodies every physics tick instead → eat whenever ANY part of the
	# food volume is in the mouth, however it got there.  The forward-motion gate is
	# dropped: the mouth is now FRONT-ONLY (z −0.45..−1.05), so food in it is always
	# a genuine head-on overlap (backing moves food OUT of the front box) — the gate
	# was only needed for the old body-overlapping mouth.
	mouth.body_entered.connect(_on_mouth_body_entered)
	add_child(mouth)
	_mouth = mouth

func _try_eat(other: Node) -> void:
	if other == null or not String(other.name).begins_with("Nutrient"):
		return
	# Debounce: after an eat, consume_nutrient repositions the food but the mouth
	# Area3D's overlap list is STALE for a tick or two → the per-tick poll would
	# re-eat the same (just-moved) food, rapid-firing at one spot (hits inflated
	# ~5×, inter-hit displacement 0).  A short cooldown kills that; genuine
	# re-approach to a respawned food takes ~45 ticks, far longer, so it's unblocked.
	if tick_counter - _last_eat_tick < _EAT_COOLDOWN_TICKS:
		return
	var w := get_parent()
	if w != null and w.has_method("consume_nutrient"):
		_last_eat_tick = tick_counter
		w.consume_nutrient(self, other)

func _on_mouth_body_entered(other: Node) -> void:
	_try_eat(other)   # fast path; the per-tick overlap poll catches spawn-inside

func _poll_mouth_overlap() -> void:
	# Eat one overlapping Nutrient per tick (consume repositions/removes it, so the
	# same food won't re-trigger).  Catches spawn-inside + fast-approach tunnelling.
	if _mouth == null:
		return
	for b in _mouth.get_overlapping_bodies():
		if String(b.name).begins_with("Nutrient"):
			_try_eat(b)
			return

func _compute_terrain_probe() -> PackedFloat64Array:
	var heights: Array[float] = []
	for rc in terrain_probes:
		if rc.is_colliding():
			heights.append(rc.get_collision_point().y)
		else:
			# No hit — body is above its raycast range.  Use a far-below
			# sentinel so the EPM can learn "no ground" as its own state.
			heights.append(global_position.y - _TERRAIN_PROBE_LENGTH)

	# Indices: 0=right, 1=left, 2=front, 3=back.
	var slope_x: float = (heights[0] - heights[1]) / (2.0 * _TERRAIN_PROBE_OFFSET)
	var slope_z: float = (heights[2] - heights[3]) / (2.0 * _TERRAIN_PROBE_OFFSET)
	var mean_h: float = (heights[0] + heights[1] + heights[2] + heights[3]) * 0.25
	var var_h:  float = 0.0
	for h in heights:
		var_h += (h - mean_h) * (h - mean_h)
	var roughness: float = sqrt(var_h * 0.25)
	var height_below: float = mean_h - global_position.y

	var out := PackedFloat64Array()
	out.append(slope_x)
	out.append(slope_z)
	out.append(roughness)
	out.append(height_below)
	return out

func _build_body_visual() -> void:
	# Capsule mesh matches the ColShape (radius 0.4, height 1.8 at y=0.9).
	# Lives on visual layer 3 so FPV cull_mask hides it but the top-down
	# camera (cull_mask layers 1+3) renders it as the agent's "footprint".
	const BODY_LAYER: int = 1 << 2  # layer 3
	var body_mat := StandardMaterial3D.new()
	body_mat.albedo_color = Color(0.85, 0.30, 0.30)   # red-orange
	body_mat.roughness    = 0.6
	var body_mesh := CapsuleMesh.new()
	body_mesh.radius = 0.4
	body_mesh.height = 1.8
	body_mesh.material = body_mat
	var body_mi := MeshInstance3D.new()
	body_mi.name     = "BodyMesh"
	body_mi.mesh     = body_mesh
	body_mi.position = Vector3(0.0, 0.9, 0.0)
	body_mi.layers   = BODY_LAYER
	add_child(body_mi)

	# Forward marker: a small bright sphere at the body's nose so the
	# top-down view conveys orientation, not just position.
	var nose_mat := StandardMaterial3D.new()
	nose_mat.albedo_color = Color(1.0, 0.9, 0.2)
	nose_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var nose_mesh := SphereMesh.new()
	nose_mesh.radius = 0.12
	nose_mesh.height = 0.24
	nose_mesh.material = nose_mat
	var nose_mi := MeshInstance3D.new()
	nose_mi.name     = "NoseMarker"
	nose_mi.mesh     = nose_mesh
	nose_mi.position = Vector3(0.0, 1.0, -0.4)   # body-forward is -Z
	nose_mi.layers   = BODY_LAYER
	add_child(nose_mi)

	# Whisker lines — ImmediateMesh PRIMITIVE_LINES from each RayCast's
	# origin to its target_position, drawn unshaded so they're visible in
	# any lighting.  1px screen width by virtue of being PRIMITIVE_LINES.
	var line_mat := StandardMaterial3D.new()
	line_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	line_mat.albedo_color = Color(0.95, 0.95, 0.30)
	for rc in whiskers:
		var im := ImmediateMesh.new()
		im.surface_begin(Mesh.PRIMITIVE_LINES, line_mat)
		im.surface_add_vertex(Vector3.ZERO)
		im.surface_add_vertex(rc.target_position)
		im.surface_end()
		var line_mi := MeshInstance3D.new()
		line_mi.name   = "WhiskerLine"
		line_mi.mesh   = im
		line_mi.layers = BODY_LAYER
		rc.add_child(line_mi)

func _build_vision_viewport() -> void:
	# Phase 6.5.23 raycast capture: nothing to construct.  All state
	# lives in the cell world (already physics-instantiated colliders);
	# we cast rays via PhysicsServer3D each tick from the FPV camera's
	# global transform.  Kept as a function to leave a hook for future
	# multi-modality vision setup (depth, motion).
	pass

func _zone_color_for(pos: Vector3) -> PackedByteArray:
	# Map a world hit-point to its grid-zone hue (place-coding signature). The room
	# is centred at origin spanning ~room_size; quantise (x,z) into _zone_grid bins.
	# Index = (gx + 2*gz): horizontal step ±1, vertical step ±2 (mod palette) → every
	# 4- and 8-connected neighbour zone gets a different hue (for grid ≤ palette/2).
	var R: float = _room_size
	var half: float = R * 0.5
	var gx: int = clampi(int((pos.x + half) / max(R, 0.001) * float(_zone_grid)), 0, _zone_grid - 1)
	var gz: int = clampi(int((pos.z + half) / max(R, 0.001) * float(_zone_grid)), 0, _zone_grid - 1)
	var idx: int = (gx + 2 * gz) % _ZONE_PALETTE.size()
	return _ZONE_PALETTE[idx]


func _publish_video_frame() -> void:
	var src_cam := get_node_or_null("Camera3D") as Camera3D
	if src_cam == null:
		return
	var space_state := get_world_3d().direct_space_state
	if space_state == null:
		return
	var cam_xf: Transform3D = src_cam.global_transform
	var origin: Vector3 = cam_xf.origin
	# Camera basis: -Z forward, +X right, +Y up (Godot convention).
	var forward: Vector3 = -cam_xf.basis.z
	var right:   Vector3 =  cam_xf.basis.x
	var up:      Vector3 =  cam_xf.basis.y
	# Field of view: read from camera (degrees) → tan half-angle.
	var fov_rad: float = deg_to_rad(src_cam.fov)
	var tan_half: float = tan(fov_rad * 0.5)
	# Square aspect (capture is square); scale right/up by tan_half so
	# the corner pixel ray is at fov/2 from forward.
	var ray_len: float = 60.0   # cell room is ~24m diagonal — generous
	# Whisker collision mask = default world; we want the same.  Layer
	# 1 (world) + layer 5 (nutrient areas).  Areas need collide_with_areas.
	var mask: int = 0xFFFFFFFF
	# RGB output buffer (row-major, top-row first to match standard image layout).
	var pixels: PackedByteArray = PackedByteArray()
	pixels.resize(_vis_res * _vis_res * 3)
	var query := PhysicsRayQueryParameters3D.new()
	query.collision_mask = mask
	query.collide_with_areas = true   # nutrients are Area3D
	query.collide_with_bodies = true
	for j in range(_vis_res):                          # row (y, top → bottom)
		# Pixel coord [-1, +1] with +1 at the top of the image.
		var v: float = 1.0 - 2.0 * (float(j) + 0.5) / float(_vis_res)
		for i in range(_vis_res):                      # column (x, left → right)
			var u: float = -1.0 + 2.0 * (float(i) + 0.5) / float(_vis_res)
			var dir: Vector3 = (forward + right * (u * tan_half) + up * (v * tan_half)).normalized()
			query.from = origin
			query.to   = origin + dir * ray_len
			var hit := space_state.intersect_ray(query)
			var color: PackedByteArray
			if hit.is_empty():
				color = _CLR_SKY
			else:
				var collider: Object = hit.collider
				var name_str: String = ""
				if collider != null and collider.has_method("get_name"):
					name_str = String(collider.name)
				var hpos: Vector3 = hit.get("position", origin)
				if name_str.begins_with("Nutrient"):
					color = _CLR_FOOD
				elif name_str.begins_with("Pillar"):
					color = _zone_color_for(hpos) if _zone_colors else _CLR_PILLAR
				elif name_str == "Terrain":
					color = _CLR_TERRAIN
				elif name_str == "Room":
					# Room contains both walls and floor.  Discriminate by
					# the hit-point Y: floor is the lowest collider; walls
					# are vertical.  collision normal vertical-up → floor.
					var nrm: Vector3 = hit.get("normal", Vector3.UP)
					if nrm.y > 0.7:
						color = _CLR_FLOOR
					else:
						color = _zone_color_for(hpos) if _zone_colors else _CLR_WALL
				else:
					color = _zone_color_for(hpos) if _zone_colors else _CLR_WALL
			var idx: int = (j * _vis_res + i) * 3
			# Green-saliency emphasis: dim NON-green pixels so the few food
			# pixels dominate the JL embedding ("amplify green in embedding
			# space").  greenness = G-(R+B)/2 is ~191 for food, <=20 otherwise,
			# so the threshold cleanly spares food while suppressing the scene.
			if _vision_green_gain > 0.0:
				var greenness: float = float(color[1]) - 0.5 * (float(color[0]) + float(color[2]))
				if greenness < 40.0:
					var keep: float = 1.0 - _vision_green_gain
					pixels[idx]     = int(round(float(color[0]) * keep))
					pixels[idx + 1] = int(round(float(color[1]) * keep))
					pixels[idx + 2] = int(round(float(color[2]) * keep))
				else:
					pixels[idx]     = color[0]
					pixels[idx + 1] = color[1]
					pixels[idx + 2] = color[2]
			else:
				pixels[idx]     = color[0]
				pixels[idx + 1] = color[1]
				pixels[idx + 2] = color[2]
	brain.publish_video(pixels, _vis_res, _vis_res, 3, "color")
	_last_vis_pixels = pixels

	# v5.3 Phase C — green-fraction signal: count food-saturated pixels
	# (R<128 AND B<128 AND G>128 — filters food-green from sky/wall/floor)
	# and publish the fraction as a 1-element ProprioToken.  Consumed by
	# a DualEMADetector reflex (events.green_visible) and the
	# EventConjunction primitive that ANDs it with scent-rising for the
	# handtuned reward scaffold.  Always published; gated downstream by
	# whether the config wires the consuming reflex/conjunction modules.
	var n_pix: int = _vis_res * _vis_res
	var green_count: int = 0
	var sum_r: int = 0
	var sum_g: int = 0
	var sum_b: int = 0
	for k in range(n_pix):
		var pi: int = k * 3
		if pixels[pi] < 128 and pixels[pi + 2] < 128 and pixels[pi + 1] > 128:
			green_count += 1
		sum_r += pixels[pi]
		sum_g += pixels[pi + 1]
		sum_b += pixels[pi + 2]
	var green_frac: float = float(green_count) / float(n_pix)
	_last_green_frac = green_frac
	# Coarse appearance descriptor — mean RGB of the FPV frame. Cheap, reusable place
	# signal (which zone-colours dominate the surround); diag-only verification that
	# zone-colouring differentiates locations.
	_last_vis_mean_rgb = [float(sum_r) / float(n_pix), float(sum_g) / float(n_pix), float(sum_b) / float(n_pix)]
	brain.publish_proprio(PackedFloat64Array([green_frac]), "green_fraction")

	# v5.4.L Phase 2b — goal-context entry vector: [scent_gradient_delta,
	# green_fraction].  Discriminative across food-approach vs post-eat
	# moments because scent_gradient = short_EMA − long_EMA flips sign on
	# eat (rising before, falling after), and green_fraction transitions
	# from high (food in view) to ~0 (food gone) at the eat.  Consumed by
	# epm_consensus_long with encoder_kind=proprio in lifecycle_v2.json
	# so the RBF projection expands these 2 scalars into a higher-dim
	# latent with built-in variance amplification.  Handtuned scaffold
	# (same class as the_cell_chunks_handtuned configs).
	var _scent_grad: float = _scent_short_ema - _scent_long_ema
	brain.publish_proprio(
		PackedFloat64Array([_scent_grad, green_frac]), "goal_context")

	# 2026-06-20 — FORWARD-LOOM FOVEA.  A small ODD cluster of rays over a NARROW
	# FOV centred dead-ahead (reusing this frame's camera basis).  The odd count
	# guarantees a u=0 ray, so dead-ahead food always registers; the narrow FOV
	# gives fine centre acuity.  Loom = food_radius / (nearest fovea-ray hit
	# distance) — a CONTINUOUS angular-size signal (small-angle proxy), smooth +
	# monotonic in closeness, gap-free, and facing-selective (≈0 unless food is
	# dead ahead).  Decoupled from the 24×24 frame → the vision EPM is untouched.
	var food_r: float = 0.5   # nutrient_radius (the_cell_world default)
	var n_f: int = maxi(1, fovea_res)
	var tan_half_f: float = tan(deg_to_rad(fovea_fov_deg) * 0.5)
	var best_loom: float = 0.0
	for fj in range(n_f):
		var fv: float = 0.0 if n_f == 1 else (1.0 - 2.0 * (float(fj) + 0.5) / float(n_f))
		# centre the grid so an ODD n_f lands a ray at exactly (0,0):
		# for n_f odd, (fj+0.5)/n_f at fj=(n_f-1)/2 = 0.5 → fv=0. good.
		for fi in range(n_f):
			var fu: float = 0.0 if n_f == 1 else (-1.0 + 2.0 * (float(fi) + 0.5) / float(n_f))
			var fdir: Vector3 = (forward + right * (fu * tan_half_f) + up * (fv * tan_half_f)).normalized()
			query.from = origin
			query.to   = origin + fdir * ray_len
			var fhit := space_state.intersect_ray(query)
			if fhit.is_empty():
				continue
			var fcol: Object = fhit.collider
			if fcol != null and String(fcol.name).begins_with("Nutrient"):
				var fdist: float = origin.distance_to(fhit.get("position", origin))
				if fdist > 0.01:
					best_loom = maxf(best_loom, clampf(food_r / fdist, 0.0, 1.0))
	_last_forward_loom = best_loom
	brain.publish_proprio(PackedFloat64Array([best_loom]), "forward_loom")

func _stuck_reference_speed() -> float:
	# The body model's achievable steady-state forward speed — reported in the
	# run snapshot so mean speed is contextualised against what THIS body can do
	# (not against move_speed, an IMU-normalization scale calibrated to the
	# asymmetric paddler's cruise).  As of the 2026-07-08 audit this is
	# reporting-only; the honest stuck metric is efference-matched (see the
	# deficit block), not a fixed reference.
	#
	# asymmetric_paddler: every spike contributes forward → expected
	#   impulse/tick = 2·base_rate·impulse_lin → steady = ÷friction (≈3.0=move_speed).
	# differential_paddler: only synchronous spikes contribute forward →
	#   expected impulse/tick = 2·base_rate²·impulse_lin → steady = ÷friction (≈1.5).
	# bidirectional_paddler: CONTINUOUS drive, no base_rate cruise.  Full forward
	#   command (common=1) adds common·2·impulse_lin/tick → mechanical ceiling
	#   = 2·impulse_lin ÷ friction (≈6.0 at defaults).
	if friction_lin_per_tick <= 0.0:
		return move_speed
	match body_model:
		"differential_paddler":
			var p_both: float = flagellum_base_rate * flagellum_base_rate
			return (2.0 * p_both * flagellum_impulse_lin) / friction_lin_per_tick
		"bidirectional_paddler":
			return (2.0 * flagellum_impulse_lin) / friction_lin_per_tick
		_:
			return (2.0 * flagellum_base_rate * flagellum_impulse_lin) / friction_lin_per_tick

func _toggle_camera_view() -> void:
	var fpv := get_node_or_null("Camera3D") as Camera3D
	var topdown := get_tree().get_root().find_child("TopDownCam", true, false) as Camera3D
	if fpv == null or topdown == null:
		return
	if fpv.current:
		topdown.make_current()
	else:
		fpv.make_current()

# ---------------------------------------------------------------------------
# UI-only top-down PLACE-MAP debug overlay (KEY_J)
#
# Read-only diagnostic that draws, ON VISUAL LAYER 3 (top-down cam ONLY):
#   • a breadcrumb of the bug's REAL world (x,z), one marker every
#     _DBG_CRUMB_EVERY ticks, COLOURED by the planner's current place-node
#     (cnode) via Color.from_hsv(cnode%12 / 12) — coherent PI ⇒ one colour =
#     one tight spatial blob; PI drift ⇒ one colour smeared, or one real spot
#     flickering through colours;
#   • a plan-steering arrow from the bug along the planner's chosen WORLD
#     direction (cyan in PLAN mode, dim grey in forage/wander) — shows whether
#     the planner steers toward or away from the food;
#   • a cross at each fixed food position.
#
# It consumes existing telemetry only: cnode/fx/fy from brain.get_module_metrics()
# (the PlaceGraphPlanner module) and food world positions from the env's
# active_food_world_positions().  No C++ / planner-logic changes.
# ---------------------------------------------------------------------------

func _toggle_planner_overlay() -> void:
	# Never build/show in headless — there is no top-down cam to render it.
	if DisplayServer.get_name() == "headless":
		return
	if _dbg_overlay == null:
		_build_planner_overlay()
		if _dbg_overlay == null:
			return
	_dbg_overlay_visible = not _dbg_overlay_visible
	_dbg_overlay.visible = _dbg_overlay_visible
	print("BodyController: planner debug overlay %s" % ("ON" if _dbg_overlay_visible else "OFF"))

func _build_planner_overlay() -> void:
	# Host the geometry under the ENV (our parent) so its transform is world-space
	# and it survives the body's own local transform / rotation.
	var env := get_parent() as Node3D
	if env == null:
		return
	_dbg_overlay = Node3D.new()
	_dbg_overlay.name = "PlannerDebugOverlay"
	_dbg_overlay.visible = false
	env.add_child(_dbg_overlay)

	# Single ImmediateMesh for the steering arrow + food crosses (rebuilt each frame).
	_dbg_crumb_lines = MeshInstance3D.new()
	_dbg_crumb_lines.name = "PlannerArrowFood"
	_dbg_crumb_lines.mesh = ImmediateMesh.new()
	var line_mat := StandardMaterial3D.new()
	line_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	line_mat.vertex_color_use_as_albedo = true
	_dbg_crumb_lines.material_override = line_mat
	_dbg_crumb_lines.layers = 1 << 2          # visual layer 3 → top-down only
	_dbg_overlay.add_child(_dbg_crumb_lines)

	# Shared little sphere for breadcrumb markers; per-instance colour via
	# per-marker material override.
	_dbg_crumb_mesh = SphereMesh.new()
	_dbg_crumb_mesh.radius = 0.18
	_dbg_crumb_mesh.height = 0.36
	_dbg_crumb_mesh.radial_segments = 6
	_dbg_crumb_mesh.rings = 3
	_dbg_crumbs.clear()
	_dbg_crumb_pos = 0
	print("BodyController: planner debug overlay BUILT (layer 3, top-down only)")

# Hash a place-node id to a hue so each node has a stable distinct colour.
func _cnode_color(cnode: int) -> Color:
	if cnode < 0:
		return Color(0.5, 0.5, 0.5, 1.0)
	return Color.from_hsv(float(cnode % 12) / 12.0, 0.85, 0.95)

func _planner_metrics() -> Dictionary:
	# Pull the live PlaceGraphPlanner metrics dict (cnode/fx/fy/planning/...) out of
	# the brain's per-module metrics.  Empty if the planner isn't present.
	if brain == null or not brain.has_method("get_module_metrics"):
		return {}
	var metrics: Dictionary = brain.get_module_metrics()
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		if String(m.get("type", "")) == "PlaceGraphPlanner":
			return m
	return {}

func _update_planner_overlay() -> void:
	if _dbg_overlay == null or _dbg_crumb_lines == null:
		return
	var pm: Dictionary = _planner_metrics()
	var cnode: int = int(pm.get("cur_node", -1))
	var fx: float = float(pm.get("fx", 0.0))     # egocentric +right
	var fy: float = float(pm.get("fy", 0.0))     # egocentric +forward
	var planning: bool = bool(pm.get("planning", false))

	if not _dbg_first_cnode_logged and cnode >= 0:
		_dbg_first_cnode_logged = true
		print("BodyController: planner overlay first valid cnode=%d (fx=%.3f fy=%.3f plan=%s)"
				% [cnode, fx, fy, str(planning)])

	# --- 1) breadcrumb: real (x,z), coloured by cnode, every N ticks -------------
	if tick_counter % _DBG_CRUMB_EVERY == 0:
		var p := global_position
		_add_crumb(Vector3(p.x, _DBG_Y, p.z), _cnode_color(cnode))

	# --- 2) plan-steering arrow + 3) food crosses (rebuilt each frame) -----------
	var im := _dbg_crumb_lines.mesh as ImmediateMesh
	im.clear_surfaces()
	im.surface_begin(Mesh.PrimitiveType.PRIMITIVE_LINES)

	# Steering arrow from the bug along the planner's chosen WORLD direction.
	# Heading h: body sets rotation.y = heading, forward = -basis.z, giving in (x,z):
	#   world_forward = (-sin h, -cos h),  world_right = (cos h, -sin h).
	# Plan output [fx,fy] is egocentric (fx=right, fy=forward), so:
	#   world = fy*(-sin h, -cos h) + fx*(cos h, -sin h).
	var h: float = heading
	var world := Vector2(
		fy * (-sin(h)) + fx * (cos(h)),     # x component
		fy * (-cos(h)) + fx * (-sin(h)))    # z component
	var arrow_col: Color = Color(0.0, 1.0, 1.0, 1.0) if planning else Color(0.45, 0.45, 0.45, 1.0)
	var origin := Vector3(global_position.x, _DBG_Y, global_position.z)
	if world.length() > 1e-4:
		var dir := world.normalized()
		var arrow_len: float = 3.0
		var tip := origin + Vector3(dir.x, 0.0, dir.y) * arrow_len
		im.surface_set_color(arrow_col); im.surface_add_vertex(origin)
		im.surface_set_color(arrow_col); im.surface_add_vertex(tip)
		# arrowhead (two short barbs)
		var perp := Vector3(-dir.y, 0.0, dir.x) * 0.35
		var back := tip - Vector3(dir.x, 0.0, dir.y) * 0.6
		im.surface_set_color(arrow_col); im.surface_add_vertex(tip)
		im.surface_set_color(arrow_col); im.surface_add_vertex(back + perp)
		im.surface_set_color(arrow_col); im.surface_add_vertex(tip)
		im.surface_set_color(arrow_col); im.surface_add_vertex(back - perp)

	# Food markers — a small + cross at each active food world position.
	var env := get_parent()
	if env != null and env.has_method("active_food_world_positions"):
		var food_col := Color(1.0, 0.25, 0.85, 1.0)   # magenta
		for fp in env.active_food_world_positions():
			var c := Vector3(fp.x, _DBG_Y, fp.z)
			var s: float = 0.5
			im.surface_set_color(food_col); im.surface_add_vertex(c + Vector3(-s, 0, 0))
			im.surface_set_color(food_col); im.surface_add_vertex(c + Vector3(s, 0, 0))
			im.surface_set_color(food_col); im.surface_add_vertex(c + Vector3(0, 0, -s))
			im.surface_set_color(food_col); im.surface_add_vertex(c + Vector3(0, 0, s))

	im.surface_end()

func _add_crumb(pos: Vector3, col: Color) -> void:
	var mi: MeshInstance3D
	if _dbg_crumbs.size() < _DBG_CRUMB_MAX:
		mi = MeshInstance3D.new()
		mi.mesh = _dbg_crumb_mesh
		mi.layers = 1 << 2                  # visual layer 3 → top-down only
		var mat := StandardMaterial3D.new()
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		mi.material_override = mat
		_dbg_overlay.add_child(mi)
		_dbg_crumbs.append(mi)
	else:
		# Ring-buffer reuse: overwrite the oldest marker.
		mi = _dbg_crumbs[_dbg_crumb_pos]
		_dbg_crumb_pos = (_dbg_crumb_pos + 1) % _DBG_CRUMB_MAX
	mi.position = pos
	(mi.material_override as StandardMaterial3D).albedo_color = col
