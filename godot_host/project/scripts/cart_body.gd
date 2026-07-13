extends Node3D
## Cart body controller for the CartPole-v1 bridge (Phase 6.5.2).
##
## Owns the 4-D state (x, x_dot, theta, theta_dot) and integrates the Gym
## CartPole-v1 ODE with parameters matched to the reference implementation
## (see docs/v4_phase6_5_2_plan.md §3.1).  Bridges proprio + events to the
## OgmaBrain child each fixed 50 Hz physics step.
##
## Output streams (parsed by scripts/cartpole_run.py):
##   - JSONL diag lines (same shape as body_controller.gd)
##   - {"episode_end": N, "reward": R, "reason": "fail"|"solve"} per episode

# ---------------------------------------------------------------------------
# Exports — parameters matched to Gym CartPole-v1
# ---------------------------------------------------------------------------
@export var config_path: String = "res://addons/ami_ogma/configs/the_cartpole_minimal.json"

# Physics constants (Gym v1 source of truth — do not alter unless syncing
# upstream Gym change).
const GRAVITY: float        = 9.8
const MASS_CART: float      = 1.0
const MASS_POLE: float      = 0.1
const TOTAL_MASS: float     = MASS_CART + MASS_POLE       # 1.1
const POLE_HALF_LEN: float  = 0.5                          # "length" in Gym
const POLEMASS_LENGTH: float= MASS_POLE * POLE_HALF_LEN    # 0.05
const FORCE_MAG: float      = 10.0
const TAU: float            = 0.02                         # 50 Hz
const X_THRESHOLD: float    = 2.4
const THETA_THRESHOLD: float= 0.20943951023931953          # 12° in rad
const MAX_STEPS_V1: int     = 500

@export var max_steps: int           = MAX_STEPS_V1
@export var max_episodes: int        = 0          # 0 = unbounded; runner sends quit
@export var diag_interval_ticks: int = 60         # 0 = silent

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
@onready var brain: OgmaBrain = $Brain

# 4-D physics state
var x: float           = 0.0
var x_dot: float       = 0.0
var theta: float       = 0.0
var theta_dot: float   = 0.0

# Episode bookkeeping
var step_in_episode: int = 0
var episode_index: int   = 0
var episode_reward: float = 0.0
var tick_counter: int    = 0

# Fixed-step accumulator — decouples physics rate from engine tick rate so
# the ODE always integrates at exactly 50 Hz regardless of project setting.
var _accum: float = 0.0

# Random policy override (validation arm A). When OGMA_CARTPOLE_RANDOM_POLICY=1,
# action is sampled per-step from a Bernoulli(0.5) instead of brain.get_action().
# Brain is still ticked so its modules behave normally; only the action gate
# is replaced.  Used for arm-A baseline (~22 mean reward by the Gym standard).
var _random_policy: bool = false
var _policy_rng: RandomNumberGenerator = RandomNumberGenerator.new()

# Last force applied — used when the brain emits accel == 0 ("no preference",
# the architecturally honest tie-break of select_action).  CartPole has no
# no-op action, so the body translates "no preference" as "carry forward
# the last force direction" — matches the spirit of the bias fix
# (5e678d9): zero valence → least-disruptive choice → keep doing what
# you were doing.  Initial force chosen by _policy_rng for symmetric
# starting conditions.
var _last_force: float = 0.0

# RNGs
var _init_rng: RandomNumberGenerator = RandomNumberGenerator.new()

# Visuals
var _cart_mesh: MeshInstance3D
var _pole_pivot: Node3D
var _last_action_force: float = 0.0

# Phase 6.5.3.B — _done is set true when max_episodes reached OR the
# user presses ESC and selects "End run" in the pause overlay.  When
# true, _step_one early-returns so the body freezes; the HUD detects
# _done and shows the end-of-run summary modal.  Replaces the prior
# get_tree().quit() which slammed the app shut without surfacing the
# final stats.
var _done: bool = false

# Phase 6.5.3.C — personal-best reward shaping.  Tracks the longest
# balance the agent has achieved this session.  Each tick where the
# CURRENT episode's step count exceeds that record, the body fires an
# additional events.hit — giving the brain a continuous positive
# learning signal during record-breaking trajectories.  No hand-tuned
# threshold (the bar rises with the agent itself); the substrate's
# adaptive dopamine baseline (da_baseline_ema_alpha) keeps the signal
# from saturating once a new plateau is held.  Mirrors the
# Schultz-style RPE setup: "I'm doing better than I ever have →
# reward_signal positive transiently → TD update reinforces this
# (s,a) chain."
var _session_best_steps: int = 0

# Phase 6.5.11 — dual-EMA reward generalisation from MC (Phase 6.5.8).
# In MC the design was: reward when short-EMA(speed) > long-EMA(speed)
# × 1.5 — "I've been faster than my own recent average" — with a
# force × velocity > 0 alignment gate to filter out coasting moments.
#
# CartPole analog: lower |θ| is the "performance" variable (more
# upright = better).  Reward fires when short-EMA(|θ|) is meaningfully
# BELOW long-EMA(|θ|) — "I've been more upright than my own recent
# average."  Inverse-direction comparison vs MC because the "better"
# direction is opposite, but the principle is the same: substrate-
# aligned PE model on body-side state.
#
# Alignment gate: force × θ > 0 — push the cart toward the pole's
# lean direction; this is the inertial-coupling restoring action
# (the cart "scoots under" the pole, which by inertia uprights it).
# When pole is near-perfectly upright, |θ| ≈ 0 → alignment rarely
# fires AND threshold check rarely fires (short ≈ long ≈ small) →
# reward density naturally low when behaviour is already perfect.
# When pole is leaning, alignment+threshold combine to reward only
# the actively-correcting actions.
var _theta_long_ema:  float = 0.0
var _theta_short_ema: float = 0.0
const _THETA_LONG_ALPHA:    float = 0.001    # ~17s at 50 Hz
const _THETA_SHORT_ALPHA:   float = 0.1      # ~10-tick window
const _THETA_HIT_FRACTION:  float = 0.7      # short < long × 0.7 → fire HIT (more upright than usual)

# Phase 6.5.4 — frozen-eval benchmark mode.  Generalised from mc_body.
# Train continuously for OGMA_CP_TRAINING_EPISODES, snapshot the brain,
# then run OGMA_CP_BENCHMARK_EPISODES at OGMA_CP_BENCHMARK_MAX_STEPS
# cap (200 = Gym v0 standard, 500 = Gym v1 standard).  Brain is
# restored from snapshot BETWEEN every benchmark episode so each
# episode evaluates the SAME starting brain (Gym v1 frozen-policy
# semantics).  Final summary as one BENCHMARK_END JSON line.
var _benchmark_enabled: bool          = false
var _benchmark_training_episodes: int = 100
var _benchmark_episodes_target: int   = 100
var _benchmark_max_steps: int         = 500   # Gym v1 standard
var _benchmark_active: bool           = false
var _benchmark_episode_idx: int       = 0
var _benchmark_rewards: Array[int]    = []
var _benchmark_snapshot: String       = ""
var _benchmark_saved_max_steps: int   = 0

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	if brain == null:
		push_error("CartBody: $Brain not found.")
		return

	# Seed precedence: ExperimentConfig (launcher) > OGMA_SEED env var >
	# time-randomized.
	var resolved_seed: int = ExperimentConfig.resolve_seed()
	var env_seed: String   = OS.get_environment("OGMA_SEED")
	if resolved_seed >= 0:
		_init_rng.seed   = resolved_seed ^ 0x636172
		_policy_rng.seed = resolved_seed ^ 0x706f6c
	elif env_seed != "":
		_init_rng.seed   = env_seed.hash() ^ 0x636172
		_policy_rng.seed = env_seed.hash() ^ 0x706f6c
	else:
		_init_rng.randomize()
		_policy_rng.randomize()

	if OS.get_environment("OGMA_CARTPOLE_RANDOM_POLICY") == "1":
		_random_policy = true
		print(JSON.stringify({"event": "RANDOM_POLICY", "active": true}))

	# Initial force direction: random per seed so paired-seed comparisons
	# don't all start with the same first-action carry-forward.  Without
	# this seeding the brain's first "no preference" tick would always
	# flip to the same default.
	_last_force = FORCE_MAG if _policy_rng.randf() >= 0.5 else -FORCE_MAG

	var env_max_eps: String = OS.get_environment("OGMA_CARTPOLE_MAX_EPISODES")
	if env_max_eps != "":
		max_episodes = max(0, env_max_eps.to_int())

	var env_max_steps: String = OS.get_environment("OGMA_CARTPOLE_MAX_STEPS")
	if env_max_steps != "":
		max_steps = max(1, env_max_steps.to_int())

	# Phase 6.5.4 — frozen-eval benchmark mode (CartPole).  Same shape as
	# mc_body's variant — see Phase 6.5.4 doc.
	if OS.get_environment("OGMA_CP_BENCHMARK") == "1":
		_benchmark_enabled = true
	var env_train_eps: String = OS.get_environment("OGMA_CP_TRAINING_EPISODES")
	if env_train_eps != "":
		_benchmark_training_episodes = max(1, env_train_eps.to_int())
	var env_bench_eps: String = OS.get_environment("OGMA_CP_BENCHMARK_EPISODES")
	if env_bench_eps != "":
		_benchmark_episodes_target = max(1, env_bench_eps.to_int())
	var env_bench_steps: String = OS.get_environment("OGMA_CP_BENCHMARK_MAX_STEPS")
	if env_bench_steps != "":
		_benchmark_max_steps = max(1, env_bench_steps.to_int())

	# Brain interface contract.
	brain.register_source(
		"CartState", "reality.proprio.cart_state",
		"float32[4]: x/2.4, x_dot/3.0, theta/12°, theta_dot/π — normalized cart-pole state",
		true)
	brain.register_sink(
		"Force", "action.out",
		"float32 accel ∈ [-4,+4] → sign maps to ±FORCE_MAG (push left/right)")
	brain.register_event("Alive",  "events.alive",  "per-tick survival pulse → drive valence")
	brain.register_event("Failed", "events.failed", "episode terminated by fall or off-track")
	brain.register_event("Solved", "events.solved", "episode survived max_steps")

	# Resolve config_path: ExperimentConfig > OGMA_CARTPOLE_CONFIG env > @export default.
	config_path = ExperimentConfig.resolve_config("OGMA_CARTPOLE_CONFIG", config_path)
	# Apply launcher's max_episodes override if set.
	if ExperimentConfig.launched and ExperimentConfig.max_episodes > 0:
		max_episodes = ExperimentConfig.max_episodes
	if ExperimentConfig.launched and ExperimentConfig.max_steps_per_episode > 0:
		max_steps = ExperimentConfig.max_steps_per_episode

	# v6.0 — propagate resolved seed into brain master_seed (see
	# OgmaBrain::set_master_seed for the contract).  Without this,
	# Premotor / ActionDecoder / etc. would stay config-seeded
	# regardless of OGMA_SEED and seed-paired A/Bs would all run the
	# same trajectory.
	if _init_rng.seed != 0:
		brain.set_master_seed(int(_init_rng.seed))
	if not brain.setup(config_path):
		push_error("CartBody: brain.setup() failed: %s" % config_path)
		return

	_build_visuals()
	_reset_state()
	print(JSON.stringify({"event": "READY", "config": config_path,
		"seed": env_seed, "max_steps": max_steps, "max_episodes": max_episodes}))

# ---------------------------------------------------------------------------
# Physics process — fixed-step accumulator drives 50 Hz ODE
# ---------------------------------------------------------------------------

func _physics_process(delta: float) -> void:
	if brain == null or not brain.is_brain_ready():
		return

	_accum += delta
	# Cap accumulator so a long stall doesn't trigger a thousand-step burst.
	if _accum > 0.2:
		_accum = 0.2
	while _accum >= TAU:
		_step_one()
		_accum -= TAU

# One 50 Hz physics + cognitive tick.  The MDP loop:
#   observe state_t → publish + brain.tick → action_t → integrate to state_{t+1}
func _step_one() -> void:
	if _done:
		return   # frozen post-run; HUD's summary modal handles next user action
	tick_counter   += 1
	step_in_episode += 1

	# 1. Publish current state (normalized).
	var proprio := PackedFloat64Array()
	proprio.append(x         / X_THRESHOLD)            # ~[-1, +1]
	proprio.append(x_dot     / 3.0)                    # ~[-1, +1] empirically
	proprio.append(theta     / THETA_THRESHOLD)        # ~[-1, +1]
	proprio.append(theta_dot / PI)                     # ~[-1, +1] empirically
	brain.publish_proprio(proprio, "cart_state")

	# Phase 6.5.3.C — personal-best reward.  Fire events.hit each tick
	# we're past the longest balance this session.  Adaptive baseline
	# in NeurochemState ensures this doesn't saturate dopamine — the
	# EMA catches up after sustained record-holding so reward_signal
	# remains a transient deviation.
	if step_in_episode > _session_best_steps:
		brain.publish_event("hit", 1.0)
		_session_best_steps = step_in_episode

	# Phase 6.5.11 — dual-EMA + alignment reward (generalised from MC
	# Phase 6.5.8/9).  Fires events.hit when CURRENT pole behaviour is
	# more upright than its OWN recent average AND the brain's last
	# action was actively correcting toward upright.
	var theta_abs := absf(theta)
	_theta_long_ema  = (1.0 - _THETA_LONG_ALPHA)  * _theta_long_ema  + _THETA_LONG_ALPHA  * theta_abs
	_theta_short_ema = (1.0 - _THETA_SHORT_ALPHA) * _theta_short_ema + _THETA_SHORT_ALPHA * theta_abs
	# Alignment: force × θ > 0.  Pushing in lean direction "scoots cart
	# under pole" → inertial reaction uprights pole.  When pole is
	# perfectly upright, force × θ ≈ 0 → no HIT, but uprightness is
	# already at its target so reward density should be low.
	var aligned_action: bool = (_last_action_force * theta) > 0.0
	if aligned_action and _theta_long_ema > 0.001 \
			and _theta_short_ema < _theta_long_ema * _THETA_HIT_FRACTION:
		brain.publish_event("hit", 1.0)

	# 2. Per-tick survival signal.  events.alive feeds HomeostaticDrive's
	# alive_pulse channel — keeps urgency low while alive, spikes on
	# failure.  This is the URGENCY pathway.
	#
	# Phase 6.5.3.9 — REMOVED per-tick events.hit.  The original wiring
	# also published events.hit per tick to drive the dopamine pathway,
	# but that saturated dopamine to the ceiling within ~2 ticks
	# (da_hit_gain=0.45, da_decay=0.88 → steady state at clamp01 limit).
	# With reward_signal = dopamine - baseline pinned at 0.80, TD update
	# δ collapses to 0 and Q-table values converge to a constant
	# regardless of (s,a) — exactly the failure mode that capped CartPole
	# near random baseline.  events.hit now fires only on episode SOLVE
	# (see _finish_episode), making reward_signal a sparse terminal
	# signal that TD bootstrap propagates backward correctly.
	brain.publish_event("alive", 1.0)

	# 3. Cognitive tick.
	brain.tick(TAU)

	# 4. Read action and discretise.  ActionDecoder emits accel ∈ [-4, +4].
	# With action_bins=3 (= the_cartpole.json config), the bins are
	# {-4, 0, +4}; the bias-fix tie-break (5e678d9) prefers smaller |a|
	# on score ties so the cold-start default is 0 ("no preference"), not
	# accel_min.  CartPole has no no-op action, so we translate accel==0
	# as "carry forward the last force direction" — architecturally honest
	# zero-valence behaviour from the brain, deterministic at the body.
	var accel: float = brain.get_action()
	if _random_policy:
		# Override the brain's choice with a Bernoulli(0.5) coin flip.
		accel = 4.0 if _policy_rng.randf() >= 0.5 else -4.0
	var force: float
	if accel > 0.001:
		force = FORCE_MAG
	elif accel < -0.001:
		force = -FORCE_MAG
	else:
		# "no preference" → keep doing what you were doing
		force = _last_force
	_last_force         = force
	_last_action_force  = force

	# 5. Integrate ODE one step (Euler, matching Gym v1 default).
	var costheta: float = cos(theta)
	var sintheta: float = sin(theta)
	var temp: float = (force + POLEMASS_LENGTH * theta_dot * theta_dot * sintheta) / TOTAL_MASS
	var thetaacc: float = (GRAVITY * sintheta - costheta * temp) \
		/ (POLE_HALF_LEN * (4.0 / 3.0 - MASS_POLE * costheta * costheta / TOTAL_MASS))
	var xacc: float = temp - POLEMASS_LENGTH * thetaacc * costheta / TOTAL_MASS

	# Plain Euler (Gym's default kinematics_integrator="euler").
	x         += TAU * x_dot
	x_dot     += TAU * xacc
	theta     += TAU * theta_dot
	theta_dot += TAU * thetaacc

	# 6. Reward bookkeeping (1.0 per surviving step).
	episode_reward += 1.0

	# 7. Update visuals.
	_update_visuals()

	# 8. Termination check.
	var failed: bool = (absf(x) > X_THRESHOLD) or (absf(theta) > THETA_THRESHOLD)
	var solved: bool = step_in_episode >= max_steps
	if failed or solved:
		_finish_episode(failed, solved)

	# 9. Diagnostic JSONL.
	if diag_interval_ticks > 0 and tick_counter % diag_interval_ticks == 0:
		_emit_jsonl(accel)

# ---------------------------------------------------------------------------
# Episode lifecycle
# ---------------------------------------------------------------------------

func _reset_state() -> void:
	# Gym v1: uniform [-0.05, +0.05] for all 4 components.
	x         = _init_rng.randf_range(-0.05, 0.05)
	x_dot     = _init_rng.randf_range(-0.05, 0.05)
	theta     = _init_rng.randf_range(-0.05, 0.05)
	theta_dot = _init_rng.randf_range(-0.05, 0.05)
	step_in_episode = 0
	episode_reward  = 0.0
	_update_visuals()

func _finish_episode(failed: bool, solved: bool) -> void:
	var reason: String = "fail" if failed else "solve"
	brain.publish_event("failed" if failed else "solved", 1.0)
	# Phase 6.5.3.9 — terminal reward signal.
	#   solve  → events.hit  → dopamine pulse → reward_signal positive →
	#     δ positive → TD update reinforces the (s,a) chain via bootstrap.
	#   failed → events.miss → dopamine drop  → reward_signal negative →
	#     δ negative → TD update penalises the (s,a) chain via bootstrap.
	# Both signals are sparse (one per episode) so reward_signal is
	# meaningful — not saturated.
	if solved:
		brain.publish_event("hit", 1.0)
	if failed:
		brain.publish_event("miss", 1.0)
	print(JSON.stringify({
		"episode_end":  episode_index,
		"reward":       int(episode_reward),
		"steps":        step_in_episode,
		"reason":       reason,
		"final_x":      snappedf(x, 0.001),
		"final_theta":  snappedf(theta, 0.001),
	}))
	episode_index += 1

	# Phase 6.5.4 — benchmark phase boundary (CartPole, mirrors mc_body).
	# 1. Snapshot at end of training_episodes.
	if _benchmark_enabled and not _benchmark_active \
			and episode_index >= _benchmark_training_episodes:
		_benchmark_snapshot = brain.snapshot_state()
		_benchmark_saved_max_steps = max_steps
		max_steps = _benchmark_max_steps
		_benchmark_active = true
		_benchmark_episode_idx = 0
		_benchmark_rewards.clear()
		print(JSON.stringify({
			"event":               "BENCHMARK_START",
			"training_episodes":   _benchmark_training_episodes,
			"benchmark_episodes":  _benchmark_episodes_target,
			"benchmark_max_steps": _benchmark_max_steps,
			"snapshot_bytes":      _benchmark_snapshot.length(),
		}))
		_reset_state()
		return
	# 2. Accumulate benchmark rewards; reload clone between episodes.
	if _benchmark_active:
		_benchmark_rewards.append(int(episode_reward))
		_benchmark_episode_idx += 1
		if _benchmark_episode_idx >= _benchmark_episodes_target:
			var sum_r: int = 0
			var max_r: int = -2147483647
			var min_r: int =  2147483647
			var solves: int = 0
			for r in _benchmark_rewards:
				sum_r += r
				if r > max_r: max_r = r
				if r < min_r: min_r = r
				if r >= _benchmark_max_steps: solves += 1
			var mean_r: float = float(sum_r) / float(_benchmark_rewards.size())
			print(JSON.stringify({
				"event":            "BENCHMARK_END",
				"n_episodes":       _benchmark_rewards.size(),
				"mean_reward":      snappedf(mean_r, 0.01),
				"min_reward":       min_r,
				"max_reward":       max_r,
				"n_full_solves":    solves,
				"solved_gym_v1":    mean_r >= 475.0,
				"all_rewards":      _benchmark_rewards,
			}))
			brain.restore_state(_benchmark_snapshot)
			max_steps = _benchmark_saved_max_steps
			_benchmark_active = false
			print(JSON.stringify({"event": "BENCHMARK_RESTORED"}))
			if not ExperimentConfig.launched:
				get_tree().quit()
				return
		else:
			# Reload the clone before the next benchmark episode.
			brain.restore_state(_benchmark_snapshot)
		_reset_state()
		return

	if max_episodes > 0 and episode_index >= max_episodes:
		print(JSON.stringify({"event": "DONE", "episodes": episode_index}))
		# Phase 6.5.3.B: when running interactively (launched via the
		# launcher), set _done so the HUD shows its summary modal
		# rather than slamming the app shut.  When running headless
		# via cartpole_run.py, keep quit() so the subprocess terminates
		# promptly after emitting DONE.
		if ExperimentConfig.launched:
			_done = true
		else:
			get_tree().quit()
		return
	_reset_state()

# ---------------------------------------------------------------------------
# Visuals (CSG primitives — minimal, top-down/side view)
# ---------------------------------------------------------------------------

func _build_visuals() -> void:
	# Track line.
	var track_mat := StandardMaterial3D.new()
	track_mat.albedo_color = Color(0.2, 0.2, 0.25)
	track_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var track_mesh := BoxMesh.new()
	track_mesh.size = Vector3(2.0 * X_THRESHOLD + 0.4, 0.02, 0.1)
	track_mesh.material = track_mat
	var track_mi := MeshInstance3D.new()
	track_mi.mesh = track_mesh
	track_mi.position = Vector3(0, -0.11, 0)
	add_child(track_mi)

	# Cart (red box).
	var cart_mat := StandardMaterial3D.new()
	cart_mat.albedo_color = Color(0.85, 0.30, 0.30)
	var cart_mesh := BoxMesh.new()
	cart_mesh.size = Vector3(0.5, 0.2, 0.2)
	cart_mesh.material = cart_mat
	_cart_mesh = MeshInstance3D.new()
	_cart_mesh.name = "CartMesh"
	_cart_mesh.mesh = cart_mesh
	add_child(_cart_mesh)

	# Pole pivot at top of cart, with pole as a thin tall box rooted at base.
	_pole_pivot = Node3D.new()
	_pole_pivot.name = "PolePivot"
	add_child(_pole_pivot)

	var pole_mat := StandardMaterial3D.new()
	pole_mat.albedo_color = Color(0.95, 0.85, 0.30)
	var pole_mesh := BoxMesh.new()
	pole_mesh.size = Vector3(0.06, 2.0 * POLE_HALF_LEN, 0.06)
	pole_mesh.material = pole_mat
	var pole_mi := MeshInstance3D.new()
	pole_mi.name = "PoleMesh"
	pole_mi.mesh = pole_mesh
	pole_mi.position = Vector3(0, POLE_HALF_LEN, 0)   # base at pivot
	_pole_pivot.add_child(pole_mi)

func _update_visuals() -> void:
	if _cart_mesh:
		_cart_mesh.position = Vector3(x, 0, 0)
	if _pole_pivot:
		_pole_pivot.position = Vector3(x, 0.1, 0)
		# Gym convention: positive theta tilts pole toward +x.  In our
		# scene the pole rotates around the +z axis; positive z-rotation
		# tips it toward +x as desired (right-hand rule).
		_pole_pivot.rotation = Vector3(0, 0, theta)

# ---------------------------------------------------------------------------
# JSONL diagnostic emitter (mirrors body_controller.gd shape so summarisers
# can reuse the same parser; only the per-tick payload differs).
# ---------------------------------------------------------------------------

func _emit_jsonl(accel: float) -> void:
	var metrics: Dictionary = brain.get_module_metrics()
	var mods := {}
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		var t: String = m.get("type", "")
		match t:
			"NeurochemState":
				mods[mod_id] = {
					"da":  snappedf(float(m.get("dopamine", 0.0)),       0.001),
					"ht":  snappedf(float(m.get("serotonin", 0.0)),      0.001),
					"eps": snappedf(float(m.get("epsilon_b_scale", 1.0)),0.01)
				}
			"EPM":
				mods[mod_id] = {
					"nodes":  int(m.get("node_count", 0)),
					"baked":  int(m.get("baked_count", 0)),
					"tle":    snappedf(float(m.get("tle", 0.0)), 0.001),
					"novel":  bool(m.get("is_novel", false))
				}
			"LateralVoter":
				var tw_in: Dictionary = m.get("trust_weights", {})
				var tw_out := {}
				for k in tw_in:
					tw_out[k] = snappedf(float(tw_in[k]), 0.001)
				mods[mod_id] = {
					"tle":   snappedf(float(m.get("fused_tle", 0.0)), 0.001),
					"trust": tw_out
				}
			"HomeostaticDrive":
				var errs: Dictionary = m.get("errors", {})
				mods[mod_id] = {
					"urg":    snappedf(float(m.get("urgency", 0.0)), 0.001),
					"errors": errs
				}
			"ActionDecoder":
				mods[mod_id] = {
					"accel":      snappedf(float(m.get("accel", 0.0)), 0.01),
					"chunk_id":   int(m.get("active_chunk_id", -1)),
					"chunk_left": int(m.get("chunk_remaining",  0))
				}
			"SequenceGNG":
				mods[mod_id] = {
					"nodes": int(m.get("node_count", 0)),
					"baked": int(m.get("baked_count", 0))
				}
			"MotorRepertoire":
				mods[mod_id] = {"chunks": int(m.get("chunk_count", 0))}
			"HomeokineticExploration":
				mods[mod_id] = {
					"eps":    int(m.get("episodes_armed", 0)),
					"active": bool(m.get("active", false))
				}

	var line := {
		"t":        tick_counter,
		"ep":       episode_index,
		"step":     step_in_episode,
		"x":        snappedf(x, 0.001),
		"x_dot":    snappedf(x_dot, 0.001),
		"theta":    snappedf(theta, 0.001),
		"th_dot":   snappedf(theta_dot, 0.001),
		"accel":    snappedf(accel, 0.01),
		"force":    snappedf(_last_action_force, 0.01),
		"ep_r":     int(episode_reward),
		"modules":  mods
	}
	print(JSON.stringify(line))
