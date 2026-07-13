extends Node
## Phase 6.5.3.A — autoload singleton holding the user's launcher
## selection across scene transitions.
##
## Body controllers (cart_body.gd, mc_body.gd, body_controller.gd) check
## these fields BEFORE falling back to OGMA_* environment variables, which
## in turn fall back to @export defaults.  Headless harness path
## (env-var-driven) stays untouched — fields default to empty so the
## env-var fallback engages whenever the launcher hasn't set them.
##
## State persistence: the launcher writes user://launcher_state.json on
## every Launch, and reads it on startup so selections persist across
## sessions.
##
## Schema mirrors the launcher dialog's fields, plus runtime overrides
## that some scenes accept via env vars today.

# ---- Selected experiment -----------------------------------------------------

# Empty string → fall through to env var / @export default.
var scene_path: String   = ""        # res://scenes/{the_cell|the_cartpole|the_mountain_car}.tscn
var config_path: String  = ""        # res://addons/ami_ogma/configs/*.json

# ---- Common runtime overrides ------------------------------------------------

# -1 → randomise per launch; otherwise concrete seed.
var seed_value: int = -1

# 0 → unbounded (interactive observation); >0 → episode count for episodic envs.
var max_episodes: int = 0

# ---- Cell-specific extras ----------------------------------------------------

var body_model: String        = ""    # "asymmetric_paddler" / "differential_paddler" / ""
var refractory_ticks: int     = -1    # -1 → use scene default
var obstacle_density: float   = -1.0  # -1 → use env var / scene default
var terrain_amplitude: float  = -1.0
# Cell experiment knobs (launcher spinners).  -1 sentinel → fall through to
# config metadata → env var → scene/@export default.
var cell_nutrient_count: int  = -1
var cell_energy_drain: float  = -1.0
var cell_scent_sigma: float   = -1.0
var room_size: float          = -1.0
# 4-loop leave-one-out ablation (launcher AblationDropdown): "Type.param" -> value patches
# applied to the loaded config EFEArbiter at launch (body_controller._apply_arbiter_overrides).
# Empty = fused (all 4 loops). Cell-only; cleared for non-cell / headless.
var cell_arbiter_overrides: Dictionary = {}

# Phase 6.5.22 ablation knobs.  -1.0 / -1 → fall through to env var /
# scene default, matching the rest of this struct.  See the launcher's
# ablation-preset dropdown for the named combinations.
var brain_weight: float    = -1.0   # 0..1; scales brain.get_action() / 4 before summing with stuck_pulse
var scent_gate_cap: float  = -1.0   # 0..1; max suppression of whisker aversion when scent is rising
var scent_gate_off: bool   = false  # disables the scent gate entirely (overrides cap)
var brain_off: bool        = false  # zero the brain's steer; reflexes alone drive motion

# Phase 6.6.D.5 — when true, the body suppresses its own publishing of
# events.hit / events.miss / events.wall_stuck because graph-resident
# reflex modules (DualEMADetector, WhiskerAversionReflex, StuckEscapeReflex)
# generate them instead.  Auto-set from config metadata.reflex_modular.
var reflex_modular: bool   = false

# Phase 6.8 — homeokinetic cell actuation (auto-set from config metadata).
# motor_baseline_beat: centered spike-rate mapping (grounds the HK forward model).
# motor_energy: per-paddle leaky energy budget (beat-coast-pause + pacing).
var motor_baseline_beat: bool = false
var motor_energy: bool         = false

# Phase v5.1 — opt-in episode-boundary signal for MC actor-critic.
# When >0, body_controller publishes events.episode_end every N physics
# ticks so Premotor (mc_lr>0) finalises its trajectory.  0 = disabled.
# Auto-set from config metadata.episode_length by the launcher; falls
# through to OGMA_EPISODE_LENGTH for headless flows.
var episode_length: int    = 0

func resolve_episode_length() -> int:
	if launched and episode_length > 0:
		return episode_length
	var env_ep := OS.get_environment("OGMA_EPISODE_LENGTH")
	if env_ep != "":
		return max(0, env_ep.to_int())
	return 0

# ---- PiCrawler standing-task knobs -----------------------------------------
# Body-side reward-shaping parameters surfaced through the launcher so the
# user can step through the experiment series interactively.  Each defaults
# to the sentinel value that means "fall through to env var / @export
# default" — sentinel = -1 for ints, < 0 for floats.

# mc_episode_period for the picrawler body (v6.0.b.2).  When >0, body
# publishes events.episode_end every N ticks; required for Premotor MC
# REINFORCE to fire.  -1 → fall through to OGMA_PICRAWLER_MC_PERIOD.
var picrawler_mc_episode_period: int = -1

# Stability shaping (v6.0.b.3) — per-tick events.miss when chassis is
# elevated AND moving fast.  Provides a neurochem "push" toward stillness
# at elevation.  -1.0 → fall through to OGMA_PICRAWLER_STAB_* env vars.
var picrawler_stab_gain:    float = -1.0
var picrawler_stab_y_norm:  float = -1.0
var picrawler_stab_speed:   float = -1.0

# Anti-rotation shaping (v6.0.b.4) — graded per-tick events.miss
# proportional to chassis_angular_velocity overage.  Addresses the
# long-horizon tip-backwards failure mode by giving the brain a
# continuous gradient to suppress rotational drift.
var picrawler_antirot_threshold: float = -1.0
var picrawler_antirot_scale:     float = -1.0
var picrawler_antirot_gain:      float = -1.0

# v6.0.b.5 — when true, body publishes reality.proprio.tilt (1-D)
# for the HomeostaticDrive posture channel.  Opt-in so existing
# baselines reproduce byte-identically.
var picrawler_publish_tilt: bool = false
# 2026-06-14 — verbose per-tick diag JSONL (headless trajectory parsing).  Off for
# UI launches by default so long interactive runs don't choke on console buildup.
var picrawler_verbose_log: bool = false

# v6.0.b.7 — energy-cost shaping.  Per-tick events.miss intensity
# proportional to total motor mechanical power consumed.  -1.0 →
# fall through to OGMA_PICRAWLER_ENERGY_* env vars.
var picrawler_energy_deadband: float = -1.0
var picrawler_energy_scale:    float = -1.0
var picrawler_energy_gain:     float = -1.0

# v6.0.b.8 — ablation overrides for the Stage B falsification lattice.
# brain_off: replace action commands with 0; random_policy: replace
# with uniform[-1,1] (uses an RNG seeded deterministically from
# master_seed).  When both true, random wins.
var picrawler_brain_off:     bool = false
var picrawler_random_policy: bool = false

# v6.0.b.11 — capped-height reward arm.  When height_penalty_gain > 0,
# the body emits events.miss when chassis_y exceeds target_height + grace,
# creating a "no higher than target" satisfaction basin.  -1 sentinels
# fall through to OGMA_PICRAWLER_* env vars → body @export defaults.
var picrawler_target_height:        float = -1.0
var picrawler_height_penalty_grace: float = -1.0
var picrawler_height_penalty_scale: float = -1.0
var picrawler_height_penalty_gain:  float = -1.0

# v6.0.b.12 — reward-curve shape selector + inverted-U params.
# reward_shape="" → fall through to env / @export default ("trapezoid").
var picrawler_reward_shape: String = ""
var picrawler_peak_height:  float  = -1.0
var picrawler_band_width:   float  = -1.0

# v6.0.b.13 — walking reward.  walk_hit_rate=0 (default) preserves
# pre-walking behaviour; setting it > 0 enables velocity-magnitude
# reward composed with the existing height reward.
var picrawler_walk_target_velocity: float = -1.0
var picrawler_walk_hit_rate:        float = -1.0

# Phase 7.5.R — multiplicative reward gating (Option 2).  Sentinel <0
# preserves the body @export default (1.0 baseline / 0.0 gated bonus =
# legacy behaviour).  Curriculum/launcher sets to non-negative values to
# enable the new shape per stage.
var picrawler_standing_baseline_factor: float = -1.0
var picrawler_gated_walk_bonus_rate:    float = -1.0

# Phase 7.7 — per-leg reward decomposition.  Sentinel <0 preserves
# body @export default (0.0 = mechanism off = byte-identical legacy).
var picrawler_per_leg_credit_gain:      float = -1.0
var picrawler_stance_y_threshold:       float = -1.0

# 2026-06-02 Stage 2 walking paradigm — per-leg step-quality reward.
var picrawler_step_quality_reward_gain: float = -1.0
var picrawler_step_lift_scale:          float = -1.0
var picrawler_step_quality_ema_alpha:   float = -1.0

# 2026-06-02 G6DOF adjustable suspension — per-joint-type spring on the
# free angular axis.  All -1.0 (= use body default = 0.0 = off) until set.
var picrawler_hip1_spring_stiffness: float = -1.0
var picrawler_hip1_spring_damping:   float = -1.0
var picrawler_hip2_spring_stiffness: float = -1.0
var picrawler_hip2_spring_damping:   float = -1.0
var picrawler_knee_spring_stiffness: float = -1.0
var picrawler_knee_spring_damping:   float = -1.0

# 2026-06-03 Joint backend selector — "" = use body default (hinge),
# "hinge" or "g6dof" override.  Persisted across launches via the
# launcher dropdown so the operator's last choice is sticky.
var picrawler_joint_backend: String = ""

# Phase 7.10b — inter-diagonal phase-contrast multiplier on gated bonus.
var picrawler_phase_contrast_gain:      float = -1.0

# Phase 7.12 — progress-PB reward.
var picrawler_progress_reward_gain:      float = -1.0
var picrawler_progress_reward_min_delta: float = -1.0

# Curriculum file (CurriculumManager autoload reads this in the body's
# _ready).  Empty string = no curriculum loaded.  Auto-advance defaults
# off — the user toggles it inside the curriculum panel at runtime, or
# pre-sets it via the launcher's "Start curriculum" path.
var picrawler_curriculum_path:           String = ""
# Default the launcher's auto-advance checkbox to ON.  Curricula exist
# to drive a learning sequence — manual stage advancement is the
# exception (e.g. early observation runs), not the rule.  Users can
# uncheck the box at launch time or toggle the curriculum panel's
# "Auto: ON/OFF" button live.
var picrawler_curriculum_auto_advance:   bool   = true

# B3 leg-symmetric weight averaging.  Empty string = "use body @export
# default" (= "off").  Set from config metadata (metadata.leg_symmetry)
# or via OGMA_PICRAWLER_LEG_SYMMETRY env var.  Choices: off / lr_pairs /
# lr_and_fr_pairs.  See picrawler_body.gd::_sync_leg_symmetry.
var picrawler_leg_symmetry:              String = ""

func resolve_picrawler_mc_period(default_v: int) -> int:
	if launched and picrawler_mc_episode_period >= 0:
		return picrawler_mc_episode_period
	var env_v := OS.get_environment("OGMA_PICRAWLER_MC_PERIOD")
	if env_v != "":
		return max(0, env_v.to_int())
	return default_v

func resolve_picrawler_stab_gain(default_v: float) -> float:
	if launched and picrawler_stab_gain >= 0.0:
		return picrawler_stab_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_STAB_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_stab_y_norm(default_v: float) -> float:
	if launched and picrawler_stab_y_norm >= 0.0:
		return picrawler_stab_y_norm
	var env_v := OS.get_environment("OGMA_PICRAWLER_STAB_Y_NORM")
	if env_v != "":
		return clamp(env_v.to_float(), 0.0, 1.0)
	return default_v

func resolve_picrawler_stab_speed(default_v: float) -> float:
	if launched and picrawler_stab_speed >= 0.0:
		return picrawler_stab_speed
	var env_v := OS.get_environment("OGMA_PICRAWLER_STAB_SPEED")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_antirot_threshold(default_v: float) -> float:
	if launched and picrawler_antirot_threshold >= 0.0:
		return picrawler_antirot_threshold
	var env_v := OS.get_environment("OGMA_PICRAWLER_ANTIROT_THRESHOLD")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_antirot_scale(default_v: float) -> float:
	if launched and picrawler_antirot_scale > 0.0:
		return picrawler_antirot_scale
	var env_v := OS.get_environment("OGMA_PICRAWLER_ANTIROT_SCALE")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_antirot_gain(default_v: float) -> float:
	if launched and picrawler_antirot_gain >= 0.0:
		return picrawler_antirot_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_ANTIROT_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_verbose_log(default_v: bool) -> bool:
	if launched:
		return picrawler_verbose_log
	var env_v := OS.get_environment("OGMA_PICRAWLER_VERBOSE_LOG")
	if env_v != "":
		return env_v != "0"
	return default_v

func resolve_picrawler_publish_tilt(default_v: bool) -> bool:
	if launched:
		return picrawler_publish_tilt
	var env_v := OS.get_environment("OGMA_PICRAWLER_PUBLISH_TILT")
	if env_v != "":
		return env_v != "0"
	return default_v

func resolve_picrawler_energy_deadband(default_v: float) -> float:
	if launched and picrawler_energy_deadband >= 0.0:
		return picrawler_energy_deadband
	var env_v := OS.get_environment("OGMA_PICRAWLER_ENERGY_DEADBAND")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_energy_scale(default_v: float) -> float:
	if launched and picrawler_energy_scale > 0.0:
		return picrawler_energy_scale
	var env_v := OS.get_environment("OGMA_PICRAWLER_ENERGY_SCALE")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_energy_gain(default_v: float) -> float:
	if launched and picrawler_energy_gain >= 0.0:
		return picrawler_energy_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_ENERGY_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_brain_off(default_v: bool) -> bool:
	if launched:
		return picrawler_brain_off
	var env_v := OS.get_environment("OGMA_PICRAWLER_BRAIN_OFF")
	if env_v != "":
		return env_v != "0"
	return default_v

func resolve_picrawler_random_policy(default_v: bool) -> bool:
	if launched:
		return picrawler_random_policy
	var env_v := OS.get_environment("OGMA_PICRAWLER_RANDOM_POLICY")
	if env_v != "":
		return env_v != "0"
	return default_v

func resolve_picrawler_target_height(default_v: float) -> float:
	if launched and picrawler_target_height > 0.0:
		return picrawler_target_height
	var env_v := OS.get_environment("OGMA_PICRAWLER_TARGET_HEIGHT")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_height_penalty_grace(default_v: float) -> float:
	if launched and picrawler_height_penalty_grace >= 0.0:
		return picrawler_height_penalty_grace
	var env_v := OS.get_environment("OGMA_PICRAWLER_HEIGHT_PENALTY_GRACE")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_height_penalty_scale(default_v: float) -> float:
	if launched and picrawler_height_penalty_scale > 0.0:
		return picrawler_height_penalty_scale
	var env_v := OS.get_environment("OGMA_PICRAWLER_HEIGHT_PENALTY_SCALE")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_height_penalty_gain(default_v: float) -> float:
	if launched and picrawler_height_penalty_gain >= 0.0:
		return picrawler_height_penalty_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_HEIGHT_PENALTY_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_reward_shape(default_v: String) -> String:
	if launched and picrawler_reward_shape != "":
		return picrawler_reward_shape
	var env_v := OS.get_environment("OGMA_PICRAWLER_REWARD_SHAPE")
	if env_v in ["trapezoid", "inverted_u"]:
		return env_v
	return default_v

func resolve_picrawler_peak_height(default_v: float) -> float:
	if launched and picrawler_peak_height > 0.0:
		return picrawler_peak_height
	var env_v := OS.get_environment("OGMA_PICRAWLER_PEAK_HEIGHT")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_band_width(default_v: float) -> float:
	if launched and picrawler_band_width > 0.0:
		return picrawler_band_width
	var env_v := OS.get_environment("OGMA_PICRAWLER_BAND_WIDTH")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_walk_target_velocity(default_v: float) -> float:
	if launched and picrawler_walk_target_velocity > 0.0:
		return picrawler_walk_target_velocity
	var env_v := OS.get_environment("OGMA_PICRAWLER_WALK_TARGET_VELOCITY")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_walk_hit_rate(default_v: float) -> float:
	if launched and picrawler_walk_hit_rate >= 0.0:
		return picrawler_walk_hit_rate
	var env_v := OS.get_environment("OGMA_PICRAWLER_WALK_HIT_RATE")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_standing_baseline_factor(default_v: float) -> float:
	if launched and picrawler_standing_baseline_factor >= 0.0:
		return picrawler_standing_baseline_factor
	var env_v := OS.get_environment("OGMA_PICRAWLER_STANDING_BASELINE_FACTOR")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_gated_walk_bonus_rate(default_v: float) -> float:
	if launched and picrawler_gated_walk_bonus_rate >= 0.0:
		return picrawler_gated_walk_bonus_rate
	var env_v := OS.get_environment("OGMA_PICRAWLER_GATED_WALK_BONUS_RATE")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_per_leg_credit_gain(default_v: float) -> float:
	if launched and picrawler_per_leg_credit_gain >= 0.0:
		return picrawler_per_leg_credit_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_PER_LEG_CREDIT_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_stance_y_threshold(default_v: float) -> float:
	if launched and picrawler_stance_y_threshold > 0.0:
		return picrawler_stance_y_threshold
	var env_v := OS.get_environment("OGMA_PICRAWLER_STANCE_Y_THRESHOLD")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_step_quality_reward_gain(default_v: float) -> float:
	if launched and picrawler_step_quality_reward_gain >= 0.0:
		return picrawler_step_quality_reward_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_STEP_QUALITY_REWARD_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_step_lift_scale(default_v: float) -> float:
	if launched and picrawler_step_lift_scale > 0.0:
		return picrawler_step_lift_scale
	var env_v := OS.get_environment("OGMA_PICRAWLER_STEP_LIFT_SCALE")
	if env_v != "":
		return max(0.001, env_v.to_float())
	return default_v

func resolve_picrawler_step_quality_ema_alpha(default_v: float) -> float:
	if launched and picrawler_step_quality_ema_alpha > 0.0:
		return picrawler_step_quality_ema_alpha
	var env_v := OS.get_environment("OGMA_PICRAWLER_STEP_QUALITY_EMA_ALPHA")
	if env_v != "":
		return clamp(env_v.to_float(), 0.0001, 1.0)
	return default_v

func resolve_picrawler_hip1_spring_stiffness(default_v: float) -> float:
	if launched and picrawler_hip1_spring_stiffness >= 0.0:
		return picrawler_hip1_spring_stiffness
	var env_v := OS.get_environment("OGMA_PICRAWLER_HIP1_SPRING_STIFFNESS")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_hip1_spring_damping(default_v: float) -> float:
	if launched and picrawler_hip1_spring_damping >= 0.0:
		return picrawler_hip1_spring_damping
	var env_v := OS.get_environment("OGMA_PICRAWLER_HIP1_SPRING_DAMPING")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_hip2_spring_stiffness(default_v: float) -> float:
	if launched and picrawler_hip2_spring_stiffness >= 0.0:
		return picrawler_hip2_spring_stiffness
	var env_v := OS.get_environment("OGMA_PICRAWLER_HIP2_SPRING_STIFFNESS")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_hip2_spring_damping(default_v: float) -> float:
	if launched and picrawler_hip2_spring_damping >= 0.0:
		return picrawler_hip2_spring_damping
	var env_v := OS.get_environment("OGMA_PICRAWLER_HIP2_SPRING_DAMPING")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_knee_spring_stiffness(default_v: float) -> float:
	if launched and picrawler_knee_spring_stiffness >= 0.0:
		return picrawler_knee_spring_stiffness
	var env_v := OS.get_environment("OGMA_PICRAWLER_KNEE_SPRING_STIFFNESS")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_knee_spring_damping(default_v: float) -> float:
	if launched and picrawler_knee_spring_damping >= 0.0:
		return picrawler_knee_spring_damping
	var env_v := OS.get_environment("OGMA_PICRAWLER_KNEE_SPRING_DAMPING")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_joint_backend(default_v: String) -> String:
	if launched and picrawler_joint_backend != "":
		return picrawler_joint_backend
	var env_v := OS.get_environment("OGMA_PICRAWLER_JOINT_BACKEND")
	if env_v in ["hinge", "g6dof"]:
		return env_v
	return default_v

func resolve_picrawler_phase_contrast_gain(default_v: float) -> float:
	if launched and picrawler_phase_contrast_gain >= 0.0:
		return picrawler_phase_contrast_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_PHASE_CONTRAST_GAIN")
	if env_v != "":
		return clamp(env_v.to_float(), 0.0, 1.0)
	return default_v

func resolve_picrawler_progress_reward_gain(default_v: float) -> float:
	if launched and picrawler_progress_reward_gain >= 0.0:
		return picrawler_progress_reward_gain
	var env_v := OS.get_environment("OGMA_PICRAWLER_PROGRESS_REWARD_GAIN")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_progress_reward_min_delta(default_v: float) -> float:
	if launched and picrawler_progress_reward_min_delta > 0.0:
		return picrawler_progress_reward_min_delta
	var env_v := OS.get_environment("OGMA_PICRAWLER_PROGRESS_REWARD_MIN_DELTA")
	if env_v != "":
		return max(0.0001, env_v.to_float())
	return default_v

# Phase 7.16 — continuous level-chassis reward resolvers.
var picrawler_level_chassis_rate:           float = -1.0
var picrawler_level_chassis_tilt_scale:     float = -1.0
var picrawler_level_chassis_walking_budget: float = -1.0

func resolve_picrawler_level_chassis_rate(default_v: float) -> float:
	if launched and picrawler_level_chassis_rate >= 0.0:
		return picrawler_level_chassis_rate
	var env_v := OS.get_environment("OGMA_PICRAWLER_LEVEL_CHASSIS_RATE")
	if env_v != "":
		return max(0.0, env_v.to_float())
	return default_v

func resolve_picrawler_level_chassis_tilt_scale(default_v: float) -> float:
	if launched and picrawler_level_chassis_tilt_scale > 0.0:
		return picrawler_level_chassis_tilt_scale
	var env_v := OS.get_environment("OGMA_PICRAWLER_LEVEL_CHASSIS_TILT_SCALE")
	if env_v != "":
		return clamp(env_v.to_float(), 0.1, PI)
	return default_v

func resolve_picrawler_level_chassis_walking_budget(default_v: float) -> float:
	if launched and picrawler_level_chassis_walking_budget >= 0.0:
		return picrawler_level_chassis_walking_budget
	var env_v := OS.get_environment("OGMA_PICRAWLER_LEVEL_CHASSIS_WALKING_BUDGET")
	if env_v != "":
		return clamp(env_v.to_float(), 0.0, PI)
	return default_v

const _LEG_SYMMETRY_CHOICES: Array = ["off", "lr_pairs", "lr_and_fr_pairs"]

func resolve_picrawler_leg_symmetry(default_v: String) -> String:
	if launched and picrawler_leg_symmetry != "":
		if picrawler_leg_symmetry in _LEG_SYMMETRY_CHOICES:
			return picrawler_leg_symmetry
	var env_v := OS.get_environment("OGMA_PICRAWLER_LEG_SYMMETRY")
	if env_v != "" and env_v in _LEG_SYMMETRY_CHOICES:
		return env_v
	return default_v

# Phase 6.6.G — resolves reflex_modular for both launcher-driven and
# headless flows.  Returns true when the launcher set it AND the active
# config opted in, OR when OGMA_REFLEX_MODULAR=1 was exported (the
# headless A/B path; bilateral crossfade configs require this even
# without ExperimentConfig.launched).
func resolve_reflex_modular() -> bool:
	if launched and reflex_modular:
		return true
	return OS.get_environment("OGMA_REFLEX_MODULAR") == "1"

# ---- CartPole / MountainCar extras -------------------------------------------

var cartpole_max_steps: int = 0       # 0 → use env var / scene default (typically 500)
var mc_max_steps: int       = 0       # 0 → use env var / scene default (typically 200; recommend 1000+ for swing dynamics)
var max_steps_per_episode: int = 0    # generic override (used by both episodic envs); 0 → use env-default

# Phase 6.5.5 — continuous-mode flag for MC.  When true, the cart never
# resets on tick-budget exhaustion; only on goal.  No events.failed/miss
# fire on artificial timeouts.  Headless runs use OGMA_MC_MAX_TICKS as
# the run-wide budget; UI runs ignore it and depend on the user closing
# the run (or external tooling firing _done).
var mc_continuous_mode: bool = false

# v6.0 — episode-boundary handling, selectable per body.  Empty string =
# fall through to OGMA_RESET_MODE env var or the body's @export default.
# Currently consumed by quadruped_body.gd; CartPole and MountainCar can
# adopt the same switch when their bodies migrate to the v6 pattern.
# Valid values: "continuous" | "soft_blink" | "instant_pause".
var reset_mode: String = ""

# v6.0 — chassis shape for the quadruped body.  Empty string falls
# through to OGMA_CHASSIS_SHAPE env or the body's @export default.
# Valid values: "half_cylinder" | "box".
var chassis_shape: String = ""

# v6.0 — global leg-strength scalar applied to hip torque + knee PD
# gains.  -1.0 = "not set; fall through to env or body default" (1.0).
# > 0 = scale factor applied directly.
var leg_strength: float = -1.0

# v6.0 — leg configuration for the quadruped body.  Empty string falls
# through to OGMA_BODY_MORPHOLOGY env or @export default ("spider").
# Valid values: "spider" | "dog".
var body_morphology: String = ""

# ---- Status ------------------------------------------------------------------

# Set to true once the launcher has populated us; body controllers gate
# their fallback chain on this.  When false, the user opened the project
# directly via the_cell.tscn (or similar) without going through the
# launcher — env-var path is authoritative as before.
var launched: bool = false

# ---- Persistence -------------------------------------------------------------

const _STATE_PATH := "user://launcher_state.json"
# State schema version.  Bump when adding a one-shot migration in
# load_state — e.g., changing a field's default that should override
# stale persisted values.  Saved state below this version is
# migrated up; never down.
#   v1: pre-2026-05-22 (no version field stored)
#   v2: picrawler_curriculum_auto_advance default flipped false → true;
#       force-migrate any saved false to true so users coming from v1
#       see the new on-by-default UX without manual checkbox toggling.
const _STATE_SCHEMA_VERSION: int = 2

func save_state() -> void:
	var d := {
		"_schema_version":    _STATE_SCHEMA_VERSION,
		"scene_path":         scene_path,
		"config_path":        config_path,
		"seed_value":         seed_value,
		"max_episodes":       max_episodes,
		"body_model":         body_model,
		"reflex_modular":     reflex_modular,
		"motor_baseline_beat": motor_baseline_beat,
		"motor_energy":       motor_energy,
		"refractory_ticks":   refractory_ticks,
		"obstacle_density":   obstacle_density,
		"terrain_amplitude":  terrain_amplitude,
		"room_size":          room_size,
		"cartpole_max_steps":   cartpole_max_steps,
		"mc_max_steps":         mc_max_steps,
		"max_steps_per_episode": max_steps_per_episode,
		"mc_continuous_mode":   mc_continuous_mode,
		"brain_weight":         brain_weight,
		"scent_gate_cap":       scent_gate_cap,
		"scent_gate_off":       scent_gate_off,
		"brain_off":            brain_off,
		"reset_mode":           reset_mode,
		"chassis_shape":        chassis_shape,
		"leg_strength":         leg_strength,
		"body_morphology":      body_morphology,
		"picrawler_mc_episode_period": picrawler_mc_episode_period,
		"picrawler_stab_gain":         picrawler_stab_gain,
		"picrawler_stab_y_norm":       picrawler_stab_y_norm,
		"picrawler_stab_speed":        picrawler_stab_speed,
		"picrawler_antirot_threshold": picrawler_antirot_threshold,
		"picrawler_antirot_scale":     picrawler_antirot_scale,
		"picrawler_antirot_gain":      picrawler_antirot_gain,
		"picrawler_publish_tilt":      picrawler_publish_tilt,
		"picrawler_verbose_log":       picrawler_verbose_log,
		"picrawler_energy_deadband":   picrawler_energy_deadband,
		"picrawler_energy_scale":      picrawler_energy_scale,
		"picrawler_energy_gain":       picrawler_energy_gain,
		"picrawler_target_height":        picrawler_target_height,
		"picrawler_height_penalty_grace": picrawler_height_penalty_grace,
		"picrawler_height_penalty_scale": picrawler_height_penalty_scale,
		"picrawler_height_penalty_gain":  picrawler_height_penalty_gain,
		"picrawler_reward_shape":         picrawler_reward_shape,
		"picrawler_peak_height":          picrawler_peak_height,
		"picrawler_band_width":           picrawler_band_width,
		"picrawler_walk_target_velocity": picrawler_walk_target_velocity,
		"picrawler_walk_hit_rate":        picrawler_walk_hit_rate,
		"picrawler_standing_baseline_factor": picrawler_standing_baseline_factor,
		"picrawler_gated_walk_bonus_rate":    picrawler_gated_walk_bonus_rate,
		"picrawler_per_leg_credit_gain":      picrawler_per_leg_credit_gain,
		"picrawler_stance_y_threshold":       picrawler_stance_y_threshold,
		"picrawler_phase_contrast_gain":      picrawler_phase_contrast_gain,
		"picrawler_progress_reward_gain":      picrawler_progress_reward_gain,
		"picrawler_progress_reward_min_delta": picrawler_progress_reward_min_delta,
		"picrawler_curriculum_path":         picrawler_curriculum_path,
		"picrawler_curriculum_auto_advance": picrawler_curriculum_auto_advance,
		"picrawler_leg_symmetry":            picrawler_leg_symmetry,
		"picrawler_joint_backend":           picrawler_joint_backend,
	}
	var f := FileAccess.open(_STATE_PATH, FileAccess.WRITE)
	if f:
		f.store_string(JSON.stringify(d, "  "))

func load_state() -> bool:
	if not FileAccess.file_exists(_STATE_PATH):
		return false
	var f := FileAccess.open(_STATE_PATH, FileAccess.READ)
	if f == null:
		return false
	var raw := f.get_as_text()
	var parsed = JSON.parse_string(raw)
	if typeof(parsed) != TYPE_DICTIONARY:
		return false
	var d: Dictionary = parsed
	scene_path         = d.get("scene_path", "")
	config_path        = d.get("config_path", "")
	seed_value         = int(d.get("seed_value", -1))
	max_episodes       = int(d.get("max_episodes", 0))
	body_model         = d.get("body_model", "")
	reflex_modular     = bool(d.get("reflex_modular", false))
	motor_baseline_beat = bool(d.get("motor_baseline_beat", false))
	motor_energy       = bool(d.get("motor_energy", false))
	refractory_ticks   = int(d.get("refractory_ticks", -1))
	obstacle_density   = float(d.get("obstacle_density", -1.0))
	terrain_amplitude  = float(d.get("terrain_amplitude", -1.0))
	room_size          = float(d.get("room_size", -1.0))
	cartpole_max_steps    = int(d.get("cartpole_max_steps", 0))
	mc_max_steps          = int(d.get("mc_max_steps", 0))
	max_steps_per_episode = int(d.get("max_steps_per_episode", 0))
	mc_continuous_mode    = bool(d.get("mc_continuous_mode", false))
	brain_weight          = float(d.get("brain_weight", -1.0))
	scent_gate_cap        = float(d.get("scent_gate_cap", -1.0))
	scent_gate_off        = bool(d.get("scent_gate_off", false))
	brain_off             = bool(d.get("brain_off", false))
	reset_mode            = d.get("reset_mode", "")
	chassis_shape         = d.get("chassis_shape", "")
	leg_strength          = float(d.get("leg_strength", -1.0))
	body_morphology       = d.get("body_morphology", "")
	picrawler_mc_episode_period = int(d.get("picrawler_mc_episode_period", -1))
	picrawler_stab_gain         = float(d.get("picrawler_stab_gain", -1.0))
	picrawler_stab_y_norm       = float(d.get("picrawler_stab_y_norm", -1.0))
	picrawler_stab_speed        = float(d.get("picrawler_stab_speed", -1.0))
	picrawler_antirot_threshold = float(d.get("picrawler_antirot_threshold", -1.0))
	picrawler_antirot_scale     = float(d.get("picrawler_antirot_scale", -1.0))
	picrawler_antirot_gain      = float(d.get("picrawler_antirot_gain", -1.0))
	picrawler_publish_tilt      = bool(d.get("picrawler_publish_tilt", false))
	picrawler_verbose_log       = bool(d.get("picrawler_verbose_log", false))
	picrawler_energy_deadband   = float(d.get("picrawler_energy_deadband", -1.0))
	picrawler_energy_scale      = float(d.get("picrawler_energy_scale", -1.0))
	picrawler_energy_gain       = float(d.get("picrawler_energy_gain", -1.0))
	picrawler_target_height        = float(d.get("picrawler_target_height", -1.0))
	picrawler_height_penalty_grace = float(d.get("picrawler_height_penalty_grace", -1.0))
	picrawler_height_penalty_scale = float(d.get("picrawler_height_penalty_scale", -1.0))
	picrawler_height_penalty_gain  = float(d.get("picrawler_height_penalty_gain", -1.0))
	picrawler_reward_shape         = str(d.get("picrawler_reward_shape", ""))
	picrawler_peak_height          = float(d.get("picrawler_peak_height", -1.0))
	picrawler_band_width           = float(d.get("picrawler_band_width", -1.0))
	picrawler_walk_target_velocity = float(d.get("picrawler_walk_target_velocity", -1.0))
	picrawler_walk_hit_rate        = float(d.get("picrawler_walk_hit_rate", -1.0))
	picrawler_standing_baseline_factor = float(d.get("picrawler_standing_baseline_factor", -1.0))
	picrawler_gated_walk_bonus_rate    = float(d.get("picrawler_gated_walk_bonus_rate", -1.0))
	picrawler_per_leg_credit_gain      = float(d.get("picrawler_per_leg_credit_gain", -1.0))
	picrawler_stance_y_threshold       = float(d.get("picrawler_stance_y_threshold", -1.0))
	picrawler_phase_contrast_gain      = float(d.get("picrawler_phase_contrast_gain", -1.0))
	picrawler_progress_reward_gain      = float(d.get("picrawler_progress_reward_gain", -1.0))
	picrawler_progress_reward_min_delta = float(d.get("picrawler_progress_reward_min_delta", -1.0))
	picrawler_curriculum_path         = str(d.get("picrawler_curriculum_path", ""))
	picrawler_curriculum_auto_advance = bool(d.get("picrawler_curriculum_auto_advance", true))
	picrawler_joint_backend           = str(d.get("picrawler_joint_backend", ""))
	# Schema migration: v1 (no version) → v2 force auto-advance on.
	# v1 saved state has false-default; the launcher's new ON-default
	# UX won't apply unless we migrate stale false values.  Users who
	# legitimately want manual control can untick the launcher
	# checkbox; the v2-or-later save round-trips that choice verbatim.
	var loaded_version: int = int(d.get("_schema_version", 1))
	if loaded_version < 2:
		print("ExperimentConfig: migrating state schema v%d → v2 (auto-advance default ON)" % loaded_version)
		picrawler_curriculum_auto_advance = true
		# Persist the migrated state immediately so we don't re-migrate
		# on every launch.  save_state stamps _schema_version with the
		# current version.
		save_state()
	picrawler_leg_symmetry            = str(d.get("picrawler_leg_symmetry", ""))
	return true

# ---- Helpers used by body controllers ----------------------------------------

# Resolve config_path with the precedence: ExperimentConfig > env var > default.
func resolve_config(env_var: String, default_path: String) -> String:
	if launched and config_path != "":
		return config_path
	var env_val: String = OS.get_environment(env_var)
	if env_val != "":
		return env_val
	return default_path

# Reads the cell config's metadata block, resolving the SAME config the brain
# loads in both UI and headless paths.  Lets the world/body read env-target
# knobs (nutrients, scent_sigma, energy_drain) from the config itself, so
# "select the config → reproduce" works identically in the UI and headless
# (env vars still override for sweeps).  Returns {} on any failure.
func read_cell_config_metadata() -> Dictionary:
	var path := resolve_config("OGMA_CELL_CONFIG", "res://addons/ami_ogma/configs/the_cell.json")
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return {}
	var parsed = JSON.parse_string(f.get_as_text())
	if typeof(parsed) == TYPE_DICTIONARY:
		return (parsed as Dictionary).get("metadata", {})
	return {}

# Returns -1 if no override; caller falls back to its own logic.
func resolve_seed() -> int:
	if launched and seed_value >= 0:
		return seed_value
	var env_seed: String = OS.get_environment("OGMA_SEED")
	if env_seed != "":
		return env_seed.to_int()
	return -1

# v6.0 — episode reset-mode resolution.  Precedence:
#   1. ExperimentConfig.reset_mode  (launcher selection)
#   2. OGMA_RESET_MODE env var      (headless harness)
#   3. supplied default             (body @export)
func resolve_reset_mode(default_mode: String) -> String:
	if launched and reset_mode != "":
		return reset_mode
	var env_val: String = OS.get_environment("OGMA_RESET_MODE")
	if env_val != "":
		return env_val
	return default_mode

# v6.0 — chassis shape resolution (same precedence shape).
func resolve_chassis_shape(default_shape: String) -> String:
	if launched and chassis_shape != "":
		return chassis_shape
	var env_val: String = OS.get_environment("OGMA_CHASSIS_SHAPE")
	if env_val != "":
		return env_val
	return default_shape

# v6.0 — leg-strength scalar resolution (same precedence shape).
func resolve_leg_strength(default_strength: float) -> float:
	if launched and leg_strength > 0.0:
		return leg_strength
	var env_val: String = OS.get_environment("OGMA_LEG_STRENGTH")
	if env_val != "":
		var v: float = env_val.to_float()
		if v > 0.0:
			return v
	return default_strength

# v6.0 — body morphology resolution (same precedence shape).
func resolve_body_morphology(default_morph: String) -> String:
	if launched and body_morphology != "":
		return body_morphology
	var env_val: String = OS.get_environment("OGMA_BODY_MORPHOLOGY")
	if env_val != "":
		return env_val
	return default_morph
