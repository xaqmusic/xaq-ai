extends Node3D
## Mountain Car body controller for the MountainCar-v0 bridge (Phase 6.5.3).
##
## Owns the 2-D state (position, velocity) and integrates the Gym
## MountainCar-v0 physics with parameters matched to the reference impl.
## Bridges proprio + events to the OgmaBrain child each fixed 50 Hz step.
##
## Output streams (parsed by scripts/mountain_car_run.py):
##   - JSONL diag lines (mirrors cart_body.gd shape)
##   - {"episode_end": N, "reward": R, "reason": "goal"|"timeout"} per ep

# ---------------------------------------------------------------------------
# Exports — physics matched to Gym MountainCar-v0
# ---------------------------------------------------------------------------
@export var config_path: String = "res://addons/ami_ogma/configs/the_mountain_car.json"

const MIN_POS: float    = -1.2
const MAX_POS: float    =  0.6
const GOAL_POS: float   =  0.5
const MAX_SPEED: float  =  0.07
const FORCE: float      =  0.001
const GRAVITY: float    =  0.0025
const TAU: float        =  0.02      # 50 Hz to match CartPole pacing
const MAX_STEPS_V0: int =  200

@export var max_steps: int           = 1000   # interactive default; headless harness sets MAX_STEPS_V0=200 via OGMA_MC_MAX_STEPS
@export var max_episodes: int        = 0      # 0 = unbounded; runner sends quit
@export var max_ticks: int           = 0      # continuous-mode run budget (0 = unbounded)
@export var diag_interval_ticks: int = 60     # 0 = silent
# Phase 6.5.5 — continuous mode (goal-only reset).  When true, the cart
# never resets on tick-budget exhaustion — only on goal.  events.failed/
# events.miss are NOT fired (no artificial timeout = no failure signal).
# The substrate sees a continuous trajectory across the entire run; the
# only episode boundaries are real successes.  Metric: goals per N ticks
# (already tracked via _session_goal_hits and HUD line).
# Mirrors the v4 substrate principle: episodic resets are externally
# imposed discontinuities the brain has no model for; a benchmark
# protocol that requires resets should be done on a CLONE (Phase 6.5.4)
# so the live brain never suffers amnesia.
@export var continuous_mode: bool    = false

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
@onready var brain: OgmaBrain = $Brain

# 2-D physics state
var position_x: float = -0.5    # init in [-0.6, -0.4]
var velocity:   float =  0.0

# Episode bookkeeping
var step_in_episode: int   = 0
var episode_index:   int   = 0
var episode_reward:  float = 0.0
var tick_counter:    int   = 0

# Body energy (drains slowly; refills on goal hit) — feeds the brain's
# energy drive channel.  This is a body-side bookkeeping device so the
# substrate's existing energy/novelty_satiation drive shape applies; the
# environment itself doesn't have a notion of "fuel."
var energy: float = 1.0
const ENERGY_DRAIN_PER_TICK: float = 0.0005
const ENERGY_REFILL_ON_GOAL: float = 0.5

# Fixed-step accumulator
var _accum: float = 0.0
var _done: bool   = false   # Phase 6.5.3.B — set when max_episodes hit interactively

# Phase 6.5.3.C/E — personal-best swing/speed (kept for HUD display
# and end-of-run summary; no longer the reward source).  Both metrics
# saturate within ~500 ticks at terminal velocity — adequate for
# "have you ever moved fast" telemetry but useless as a sustained
# reward signal once saturated.  See Phase 6.5.6 below for the
# replacement reward design.
var _session_best_swing: float = 0.0   # tracks max(|position - resting_pos|) seen
var _session_best_speed: float = 0.0   # tracks max(|velocity|) seen — kinetic-energy gradient
var _session_max_pos:    float = -2.0  # per-physics-tick max position (HUD reads for accurate swing display)
var _session_min_pos:    float =  2.0  # per-physics-tick min position
var _session_goal_hits:  int   = 0     # cumulative goal events; HUD divides by elapsed time for goals/min
const _RESTING_POS: float = -0.5       # valley centre

# Phase 6.5.6 — EMA-based intrinsic reward.
#
# The personal-best mechanism (6.5.3.C/E) was ANTI-aligned with
# continuous mode: bests saturate fast, then no events.hit ever
# fires, dopamine returns to baseline, brain has no gradient and
# drifts to "stay at the bottom" (the energetically-cheapest stable
# state under EFE-driven exploration).
#
# Replacement: track a slow EMA of |velocity| AND |swing|; fire
# events.hit when CURRENT exceeds EMA × threshold.  This is a
# body-side analog of NeurochemState's adaptive-baseline reward-
# prediction-error model: "reward when you're doing better than
# your own recent average."  Naturally:
# - Avoids saturation: reward fires only on transient outperformance,
#   not constant high state.
# - No dead zone at terminal velocity: once the cart's EMA reaches
#   terminal speed, the brain still gets reward by maintaining big
#   swings (since swing-EMA tracks separately).
# - Self-adapting: as the brain gets better, EMA rises, threshold
#   for next hit rises — moving target keeps the gradient alive.
var _speed_ema: float = 0.0       # long EMA (~17s)
var _swing_ema: float = 0.0       # long EMA (~17s)
var _speed_short_ema: float = 0.0 # short EMA (~10 ticks)
var _swing_short_ema: float = 0.0 # short EMA (~10 ticks)
const _BODY_EMA_ALPHA: float       = 0.001  # long-EMA rate (~17s @ 50 Hz)
const _BODY_EMA_SHORT_ALPHA: float = 0.1    # short-EMA rate (~10-tick window)
const _BODY_HIT_THRESHOLD: float   = 1.5    # default (active regime)
# Phase 6.5.10 — cold-start mode toggle.
# Default (option 1): no bootstrap, strict 1.5× threshold throughout.
#   EMA grows organically from 0; threshold stays low while EMAs are
#   tiny so any aligned motion can fire HIT.  The alignment gate
#   prevents spurious passive-gravity rewards.
# OGMA_MC_PERMISSIVE_SETTLED=1 (option 2): regime switch.
#   When speed_long < SETTLED_FLOOR, threshold drops to 1.05× —
#   permissive cold-start mode.  When speed_long ≥ floor, threshold
#   reverts to 1.5×.  Tested headless against option 1 for comparison.
@export var _permissive_settled_mode: bool = false
const _BODY_HIT_THRESHOLD_SETTLED: float = 1.05
const _BODY_SETTLED_FLOOR: float         = 0.005

# Phase 6.5.7 — goal-time EMA bonus.
#
# Without this, a 147-tick goal and a 5000-tick goal both fire the
# SAME single events.hit pulse — the brain has no signal for "shorter
# time-to-goal is better."  Result: once a working motif is found,
# the brain exploits it but doesn't refine it.  Goal times random-walk
# around some mean rather than monotonically improving.
#
# Fix: track a slow EMA of ticks-per-goal.  When a new goal beats
# EMA × 0.9 (i.e., is at least 10% faster than recent average), fire
# an EXTRA events.hit on top of the standard one.  Creates a moving-
# target gradient toward faster goals — every improvement gets
# reinforced more strongly than steady-state performance.
var _goal_time_ema: float = 0.0           # 0 = no goals yet (sentinel)
const _GOAL_TIME_EMA_ALPHA: float      = 0.2   # tracks within ~5 goals
const _GOAL_TIME_BONUS_FRACTION: float = 0.9   # current < EMA × 0.9 → bonus
var _last_action_force: float = 0.0
var _init_rng: RandomNumberGenerator = RandomNumberGenerator.new()
var _random_policy: bool = false
var _policy_rng: RandomNumberGenerator = RandomNumberGenerator.new()

# Phase 6.5.4 — frozen-eval benchmark mode (see _ready for env vars).
var _benchmark_enabled: bool          = false
var _benchmark_training_episodes: int = 50
var _benchmark_episodes_target: int   = 100
var _benchmark_max_steps: int         = 200
var _benchmark_active: bool           = false       # true once snapshot taken
var _benchmark_episode_idx: int       = 0           # counter within benchmark phase
var _benchmark_rewards: Array[int]    = []          # accumulated benchmark episode rewards
var _benchmark_snapshot: String       = ""          # saved brain state JSON
var _benchmark_saved_max_steps: int   = 0           # restore at end

# Visuals
var _cart_mesh: MeshInstance3D
var _track_mesh: MeshInstance3D
var _goal_mesh: MeshInstance3D

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	if brain == null:
		push_error("MCBody: $Brain not found.")
		return

	# Seed precedence: ExperimentConfig (launcher) > OGMA_SEED env > random.
	var resolved_seed: int = ExperimentConfig.resolve_seed()
	var env_seed: String   = OS.get_environment("OGMA_SEED")
	if resolved_seed >= 0:
		_init_rng.seed   = resolved_seed ^ 0x6d63
		_policy_rng.seed = resolved_seed ^ 0x6d63706f
	elif env_seed != "":
		_init_rng.seed   = env_seed.hash() ^ 0x6d63
		_policy_rng.seed = env_seed.hash() ^ 0x6d63706f
	else:
		_init_rng.randomize()
		_policy_rng.randomize()

	if OS.get_environment("OGMA_MC_RANDOM_POLICY") == "1":
		_random_policy = true
		print(JSON.stringify({"event": "RANDOM_POLICY", "active": true}))

	var env_max_eps: String = OS.get_environment("OGMA_MC_MAX_EPISODES")
	if env_max_eps != "":
		max_episodes = max(0, env_max_eps.to_int())
	var env_max_steps: String = OS.get_environment("OGMA_MC_MAX_STEPS")
	if env_max_steps != "":
		max_steps = max(1, env_max_steps.to_int())

	# Phase 6.5.5 — continuous mode (goal-only reset).
	if OS.get_environment("OGMA_MC_CONTINUOUS") == "1":
		continuous_mode = true
	# Phase 6.5.10 — option 2 cold-start mode (permissive threshold
	# when cart is settled).  Default off (option 1: no bootstrap,
	# strict threshold throughout).
	if OS.get_environment("OGMA_MC_PERMISSIVE_SETTLED") == "1":
		_permissive_settled_mode = true
	# Launcher override (UI-set toggle takes precedence over env if launched).
	if ExperimentConfig.launched and ExperimentConfig.mc_continuous_mode:
		continuous_mode = true
	var env_max_ticks: String = OS.get_environment("OGMA_MC_MAX_TICKS")
	if env_max_ticks != "":
		max_ticks = max(0, env_max_ticks.to_int())

	# Phase 6.5.4 — frozen-eval benchmark mode.  Train continuously for
	# `OGMA_MC_TRAINING_EPISODES` (no Gym-style reset on timeout — body
	# already does that in launched mode), then snapshot the brain, run
	# `OGMA_MC_BENCHMARK_EPISODES` at `OGMA_MC_BENCHMARK_MAX_STEPS` cap
	# (Gym v0 standard 200-tick episodic protocol), restore the brain,
	# print the benchmark summary as one JSON line, and exit.  This is
	# the "snapshot, run benchmark on a clone, restore" methodology
	# realized in-place (the source brain rolls back rather than running
	# in parallel — single-process variant of clone(), uses the same
	# OgmaInstance::snapshot_state/restore_state primitives).
	var env_bench: String = OS.get_environment("OGMA_MC_BENCHMARK")
	if env_bench == "1":
		_benchmark_enabled = true
	var env_train_eps: String = OS.get_environment("OGMA_MC_TRAINING_EPISODES")
	if env_train_eps != "":
		_benchmark_training_episodes = max(1, env_train_eps.to_int())
	var env_bench_eps: String = OS.get_environment("OGMA_MC_BENCHMARK_EPISODES")
	if env_bench_eps != "":
		_benchmark_episodes_target = max(1, env_bench_eps.to_int())
	var env_bench_steps: String = OS.get_environment("OGMA_MC_BENCHMARK_MAX_STEPS")
	if env_bench_steps != "":
		_benchmark_max_steps = max(1, env_bench_steps.to_int())

	# Brain interface contract.
	brain.register_source(
		"MCState", "reality.proprio.mc_state",
		"float32[2]: pos/MAX_POS, vel/MAX_SPEED — normalised mountain-car state",
		true)
	brain.register_source(
		"Energy", "reality.proprio.energy",
		"float32[1]: body energy [0,1]; drains, refills on goal hit",
		true)
	brain.register_sink(
		"Force", "action.out",
		"float32 accel ∈ [-1,+1] → sign maps to ±FORCE (push left/none/right)")
	brain.register_event("Hit",    "events.hit",
		"goal reached at pos>=0.5 (sparse reward → dopamine pulse)")
	brain.register_event("Failed", "events.failed",
		"episode timed out without reaching goal")

	# Resolve config_path: ExperimentConfig > OGMA_MC_CONFIG env > @export default.
	config_path = ExperimentConfig.resolve_config("OGMA_MC_CONFIG", config_path)
	if ExperimentConfig.launched and ExperimentConfig.max_episodes > 0:
		max_episodes = ExperimentConfig.max_episodes
	if ExperimentConfig.launched and ExperimentConfig.max_steps_per_episode > 0:
		max_steps = ExperimentConfig.max_steps_per_episode
	# v6.0 — propagate resolved seed into brain master_seed (see
	# OgmaBrain::set_master_seed).  Same rationale as cart_body.gd.
	if _init_rng.seed != 0:
		brain.set_master_seed(int(_init_rng.seed))
	if not brain.setup(config_path):
		push_error("MCBody: brain.setup() failed: %s" % config_path)
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
	if _accum > 0.2:
		_accum = 0.2
	while _accum >= TAU:
		_step_one()
		_accum -= TAU

func _step_one() -> void:
	if _done:
		return
	tick_counter   += 1
	step_in_episode += 1

	# 1. Publish current normalised state.
	var proprio := PackedFloat64Array()
	proprio.append(position_x / MAX_POS)         # ~[-1, +1]
	proprio.append(velocity   / MAX_SPEED)       # ~[-1, +1]
	brain.publish_proprio(proprio, "mc_state")

	# 1a. Publish energy proprio for the drive's energy channel.
	brain.publish_proprio(PackedFloat64Array([energy]), "energy")

	# Phase 6.5.6/8 — dual-EMA intrinsic reward.  Single-EMA threshold
	# (Phase 6.5.6) fired HIT whenever current absolute speed/swing
	# exceeded the slow EMA — including coasting moments where the cart
	# was decelerating from a previous burst.  The brain credits whatever
	# action it took at HIT time, so "do nothing while coasting" got
	# rewarded equally with "actively pump the swing."
	#
	# Replacement (Phase 6.5.8): track BOTH a short EMA (~10-tick window)
	# and a long EMA (~17s); fire HIT when short > long × 1.5.  This
	# means "the cart has been sustainedly fast over the last ~10 ticks"
	# rather than "the cart is fast at this single tick."  Single-tick
	# coasting peaks no longer trigger reward; sustained fast periods do.
	# Sharper credit assignment toward genuine momentum-building actions.
	var swing := absf(position_x - _RESTING_POS)
	var speed := absf(velocity)
	# EMAs grow organically from zero — no bootstrap window (Phase 6.5.10 opt 1).
	_speed_ema       = (1.0 - _BODY_EMA_ALPHA)       * _speed_ema       + _BODY_EMA_ALPHA       * speed
	_swing_ema       = (1.0 - _BODY_EMA_ALPHA)       * _swing_ema       + _BODY_EMA_ALPHA       * swing
	_speed_short_ema = (1.0 - _BODY_EMA_SHORT_ALPHA) * _speed_short_ema + _BODY_EMA_SHORT_ALPHA * speed
	_swing_short_ema = (1.0 - _BODY_EMA_SHORT_ALPHA) * _swing_short_ema + _BODY_EMA_SHORT_ALPHA * swing
	# Threshold (default 1.5×).  When permissive_settled mode is on
	# (Phase 6.5.10 option 2) AND cart is settled (speed_long below
	# floor), threshold drops to 1.05× — even small aligned motion
	# fires HIT, breaking the brain out of cold-start rest faster.
	var threshold: float = _BODY_HIT_THRESHOLD
	if _permissive_settled_mode and _speed_ema < _BODY_SETTLED_FLOOR:
		threshold = _BODY_HIT_THRESHOLD_SETTLED
	# Force × velocity > 0 (Phase 6.5.9): only fire HIT when brain's
	# action is actively adding kinetic energy.  Coasting/braking →
	# no reward.
	var aligned_action: bool = (_last_action_force * velocity) > 0.0
	var fired_hit := false
	if aligned_action:
		if _speed_ema > 0.0 and _speed_short_ema > _speed_ema * threshold:
			brain.publish_event("hit", 1.0)
			fired_hit = true
		if not fired_hit and _swing_ema > 0.0 and _swing_short_ema > _swing_ema * threshold:
			brain.publish_event("hit", 1.0)
	# HUD/summary tracking — bests still get recorded for telemetry
	# purposes.  Reward gradient comes from the EMA mechanism above.
	if swing > _session_best_swing: _session_best_swing = swing
	if speed > _session_best_speed: _session_best_speed = speed
	# Per-physics-tick max/min position tracking.  HUD samples per render
	# frame (60 Hz) but physics is 50 Hz with multi-tick accumulator — at
	# velocity ±0.07 the cart can pass the goal between HUD samples.
	# Tracking here ensures the displayed best swing reflects every tick
	# the substrate actually experienced.
	if position_x > _session_max_pos: _session_max_pos = position_x
	if position_x < _session_min_pos: _session_min_pos = position_x

	# 2. Cognitive tick.
	brain.tick(TAU)

	# 3. Read action (3-bin discrete: -1, 0, +1 maps to push left/none/right).
	var accel: float = brain.get_action()
	if _random_policy:
		var r := _policy_rng.randi() % 3
		accel = float(r - 1)  # -1, 0, +1
	# Discretise to {-1, 0, +1}
	var action: int
	if accel > 0.33:        action =  1
	elif accel < -0.33:     action = -1
	else:                   action =  0
	_last_action_force = float(action) * FORCE

	# 4. Integrate ODE one step (Gym MountainCar-v0).
	velocity += float(action) * FORCE - GRAVITY * cos(3.0 * position_x)
	velocity = clampf(velocity, -MAX_SPEED, MAX_SPEED)
	position_x += velocity
	if position_x < MIN_POS:
		position_x = MIN_POS
		velocity   = 0.0
	if position_x > MAX_POS:
		position_x = MAX_POS

	# 5. Reward bookkeeping (Gym: -1 per step until goal).
	episode_reward -= 1.0
	# Energy drains per tick.
	energy = maxf(0.0, energy - ENERGY_DRAIN_PER_TICK)

	# 6. Update visuals.
	_update_visuals()

	# 7. Termination.
	var goal:    bool = position_x >= GOAL_POS
	# Continuous mode (Phase 6.5.5): NEVER timeout — the cart plays forever
	# until either a goal or the run-wide tick budget exhausts.  Episodic
	# mode (Gym v0 semantics): timeout fires events.failed + events.miss
	# and resets position.
	var timeout: bool = (not continuous_mode) and (step_in_episode >= max_steps)
	if goal:
		# Reward signal: episode ends with goal — emit hit event so
		# dopamine pulses, refill energy, then reset.
		brain.publish_event("hit", 1.0)
		# Phase 6.5.7 bonus: if THIS goal was meaningfully faster than
		# the running EMA of past goal-times, fire an EXTRA events.hit
		# on top of the standard one.  First goal seeds the EMA without
		# firing the bonus (no baseline to compare against yet).
		var current_goal_time: float = float(step_in_episode)
		if _goal_time_ema > 0.0 \
				and current_goal_time < _goal_time_ema * _GOAL_TIME_BONUS_FRACTION:
			brain.publish_event("hit", 1.0)
		if _goal_time_ema <= 0.0:
			_goal_time_ema = current_goal_time
		else:
			_goal_time_ema = (1.0 - _GOAL_TIME_EMA_ALPHA) * _goal_time_ema \
					+ _GOAL_TIME_EMA_ALPHA * current_goal_time
		# Phase 6.5.3.F — publish events.solved on success so ActionDecoder
		# clears its eligibility trace and prev_state at the episode
		# boundary.  Without this, the queued dopamine pulse from the goal
		# hit (which NeurochemState processes on the NEXT tick — i.e., the
		# first tick of the new episode after _reset_state) leaks into the
		# new episode's TD updates and credits the random reset-state
		# action with the goal reward.  Symmetric with cart_body.gd:300.
		# Diagnosed seed-44 hot-streak collapse (eps 38-55 → eps 56+).
		brain.publish_event("solved", 1.0)
		energy = minf(1.0, energy + ENERGY_REFILL_ON_GOAL)
		_session_goal_hits += 1
		_finish_episode("goal")
	elif timeout:
		brain.publish_event("failed", 1.0)
		# Phase 6.5.3.9 — also publish events.miss on timeout.  Without
		# this, NeurochemState (which only updates dopamine on hit/miss
		# events) never sees a negative signal, so reward_signal stays at
		# baseline forever and Q-table never gets even a "this was bad"
		# update.  Symmetric with cart_body.gd's failure handling.
		brain.publish_event("miss", 1.0)
		_finish_episode("timeout")
	elif continuous_mode and max_ticks > 0 and tick_counter >= max_ticks:
		# Headless run-wide tick budget exhausted.  Emit a final summary
		# (mirrors the BENCHMARK_END shape so analysis tooling can reuse
		# the same parser) and quit.  No events fired — the substrate
		# isn't being told this is a "failure," just that the wall clock
		# expired.
		print(JSON.stringify({
			"event":          "CONTINUOUS_DONE",
			"total_ticks":    tick_counter,
			"goals_hit":      _session_goal_hits,
			"goals_per_kt":   snappedf(float(_session_goal_hits) * 1000.0 / float(tick_counter), 0.001),
			"final_pos":      snappedf(position_x, 0.001),
			"best_swing":     snappedf(_session_max_pos - _session_min_pos, 0.001),
			"best_speed":     snappedf(_session_best_speed, 0.0001),
			"speed_ema":      snappedf(_speed_ema, 0.0001),
			"speed_short":    snappedf(_speed_short_ema, 0.0001),
			"swing_ema":      snappedf(_swing_ema, 0.001),
			"swing_short":    snappedf(_swing_short_ema, 0.001),
			"goal_time_ema":  snappedf(_goal_time_ema, 0.1),
		}))
		if ExperimentConfig.launched:
			_done = true
		else:
			get_tree().quit()

	# 8. Diagnostic JSONL.
	if diag_interval_ticks > 0 and tick_counter % diag_interval_ticks == 0:
		_emit_jsonl(accel)

# ---------------------------------------------------------------------------
# Episode lifecycle
# ---------------------------------------------------------------------------

func _reset_state() -> void:
	# Gym v0: position uniform [-0.6, -0.4], velocity 0.
	position_x = _init_rng.randf_range(-0.6, -0.4)
	velocity   = 0.0
	step_in_episode = 0
	episode_reward  = 0.0
	_update_visuals()

func _finish_episode(reason: String) -> void:
	# Per-episode brain snapshot for diagnostic analysis (Phase 6.5.3.F).
	# Lightweight enough to leave on always — one line per episode end.
	var brain_snap: Dictionary = {}
	if brain != null and brain.is_brain_ready():
		var m: Dictionary = brain.get_module_metrics()
		var n: Dictionary = m.get("neuro", {})
		var ad: Dictionary = m.get("action_decoder", {})
		var sa: Dictionary = m.get("seq_action", {})
		var rep: Dictionary = m.get("repertoire", {})
		# da_baseline_ema = dopamine - reward_signal (reward_signal is the
		# difference; OgmaBrain doesn't surface ema directly).
		var da_now:    float = float(n.get("dopamine", 0.0))
		var r_signal:  float = float(n.get("reward_signal", 0.0))
		brain_snap = {
			"da":       snappedf(da_now, 0.001),
			"da_ema":   snappedf(da_now - r_signal, 0.001),
			"ht":       snappedf(float(n.get("serotonin", 0.0)), 0.001),
			"r_sig":    snappedf(r_signal, 0.001),
			"q_n":      int(ad.get("valence_size", 0)),
			"q_disp":   int(ad.get("chunk_dispatch_count", 0)),
			"chunks":   int(rep.get("chunk_count", 0)),
			"sa_baked": int(sa.get("baked_count", 0)),
		}
	print(JSON.stringify({
		"episode_end":  episode_index,
		"reward":       int(episode_reward),
		"steps":        step_in_episode,
		"reason":       reason,
		"final_pos":    snappedf(position_x, 0.001),
		"final_vel":    snappedf(velocity,   0.001),
		"brain":        brain_snap,
	}))
	episode_index += 1

	# Phase 6.5.4 — benchmark phase boundary handling.
	# 1. If we're not yet benchmarking and the training-episode budget is
	#    exhausted, snapshot the brain and switch to benchmark mode.
	if _benchmark_enabled and not _benchmark_active \
			and episode_index >= _benchmark_training_episodes:
		_benchmark_snapshot = brain.snapshot_state()
		_benchmark_saved_max_steps = max_steps
		max_steps = _benchmark_max_steps
		_benchmark_active = true
		_benchmark_episode_idx = 0
		_benchmark_rewards.clear()
		print(JSON.stringify({
			"event":             "BENCHMARK_START",
			"training_episodes": _benchmark_training_episodes,
			"benchmark_episodes": _benchmark_episodes_target,
			"benchmark_max_steps": _benchmark_max_steps,
			"snapshot_bytes":    _benchmark_snapshot.length(),
		}))
		# Reset the body state for the first benchmark episode.
		_reset_state()
		return
	# 2. If we're benchmarking, accumulate this episode's reward and check
	#    whether benchmark is complete.  Phase 6.5.4 — Gym v0 frozen-policy
	#    semantics: reload the clone (i.e., restore brain from the
	#    pre-benchmark snapshot) BETWEEN every benchmark episode so each
	#    episode is evaluated against the SAME brain state.  This prevents
	#    intra-benchmark plasticity from making later episodes see a
	#    different brain than earlier ones.  The reset between episodes is
	#    therefore a true "fresh clone" rather than "same brain replayed."
	if _benchmark_active:
		_benchmark_rewards.append(int(episode_reward))
		_benchmark_episode_idx += 1
		if _benchmark_episode_idx >= _benchmark_episodes_target:
			# Compute summary stats.
			var sum_r: int = 0
			var max_r: int = -2147483647
			var min_r: int =  2147483647
			var goals: int = 0
			for r in _benchmark_rewards:
				sum_r += r
				if r > max_r: max_r = r
				if r < min_r: min_r = r
				if r > -_benchmark_max_steps: goals += 1
			var mean_r: float = float(sum_r) / float(_benchmark_rewards.size())
			print(JSON.stringify({
				"event":          "BENCHMARK_END",
				"n_episodes":     _benchmark_rewards.size(),
				"mean_reward":    snappedf(mean_r, 0.01),
				"min_reward":     min_r,
				"max_reward":     max_r,
				"n_goals":        goals,
				"solved_gym_v0":  mean_r >= -110.0,
				"all_rewards":    _benchmark_rewards,
			}))
			# Restore brain state so subsequent training continues from
			# pre-benchmark moment.  In headless mode we then quit; in UI
			# mode we resume training (rare; benchmark is mostly headless).
			brain.restore_state(_benchmark_snapshot)
			max_steps = _benchmark_saved_max_steps
			_benchmark_active = false
			print(JSON.stringify({"event": "BENCHMARK_RESTORED"}))
			if not ExperimentConfig.launched:
				get_tree().quit()
				return
		else:
			# Reload the clone before the next benchmark episode — every
			# episode evaluates the same starting brain state.
			brain.restore_state(_benchmark_snapshot)
		_reset_state()
		return

	if max_episodes > 0 and episode_index >= max_episodes:
		print(JSON.stringify({"event": "DONE", "episodes": episode_index}))
		# Phase 6.5.3.B: same launched-vs-headless split as cart_body —
		# UI mode freezes via _done; headless mode quits cleanly.
		if ExperimentConfig.launched:
			_done = true
		else:
			get_tree().quit()
		return
	# Interactive mode: only reset on goal — let the cart keep playing on
	# timeout so the brain has continuous time to build momentum across
	# what would otherwise be artificial 1000-tick episode boundaries.
	# Headless mode preserves the Gym-comparable cap so paired-seed
	# benchmarks remain valid.
	if ExperimentConfig.launched and reason == "timeout":
		step_in_episode = 0
		episode_reward  = 0.0
		return
	_reset_state()

# ---------------------------------------------------------------------------
# Visuals — minimal side-view of the mountain valley + cart
# ---------------------------------------------------------------------------

func _build_visuals() -> void:
	# Track (mountain valley): a series of small boxes laid out along the
	# valley curve y = sin(3*x).  Quick visual indicator only; not used
	# for physics.
	var track_mat := StandardMaterial3D.new()
	track_mat.albedo_color = Color(0.20, 0.20, 0.25)
	track_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	for i in range(40):
		var x: float = MIN_POS + float(i) / 39.0 * (MAX_POS - MIN_POS)
		var y: float = sin(3.0 * x) * 0.45
		var seg := MeshInstance3D.new()
		var box := BoxMesh.new()
		box.size = Vector3(0.05, 0.02, 0.04)
		box.material = track_mat
		seg.mesh     = box
		seg.position = Vector3(x, y, 0)
		add_child(seg)

	# Goal marker — vertical razor-thin line at exactly GOAL_POS.x so
	# the user has zero visual ambiguity about where the goal is.
	# Previous wider 0.04m-wide box made it look like the cart had
	# crossed when the cart's right edge reached the box's left edge
	# (~0.02 short of actual physics goal).
	var goal_mat := StandardMaterial3D.new()
	goal_mat.albedo_color = Color(0.30, 0.95, 0.40)
	goal_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var goal_box := BoxMesh.new()
	goal_box.size = Vector3(0.005, 0.50, 0.04)   # razor-thin x; tall y for visibility
	goal_box.material = goal_mat
	_goal_mesh = MeshInstance3D.new()
	_goal_mesh.mesh = goal_box
	_goal_mesh.position = Vector3(GOAL_POS, sin(3.0 * GOAL_POS) * 0.45 + 0.25, 0)
	add_child(_goal_mesh)

	# Goal flag triangle on top of the line (purely decorative).
	var flag_mat := StandardMaterial3D.new()
	flag_mat.albedo_color = Color(0.30, 0.95, 0.40)
	flag_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var flag_mesh := BoxMesh.new()
	flag_mesh.size = Vector3(0.05, 0.03, 0.04)
	flag_mesh.material = flag_mat
	var flag := MeshInstance3D.new()
	flag.mesh = flag_mesh
	flag.position = Vector3(GOAL_POS + 0.025, sin(3.0 * GOAL_POS) * 0.45 + 0.48, 0)
	add_child(flag)

	# Cart — small enough that its visual half-width is negligible
	# compared to the position scale, so the user's eye reads
	# "cart center crossed the line" as the trigger condition.
	# Previous 0.08m-wide cart made the right edge appear past the
	# goal when physics center was still ~0.04m short.
	var cart_mat := StandardMaterial3D.new()
	cart_mat.albedo_color = Color(0.85, 0.30, 0.30)
	var cart_box := BoxMesh.new()
	cart_box.size = Vector3(0.04, 0.04, 0.04)
	cart_box.material = cart_mat
	_cart_mesh = MeshInstance3D.new()
	_cart_mesh.name = "CartMesh"
	_cart_mesh.mesh = cart_box
	add_child(_cart_mesh)

	# Centre dot on cart so the user can read its precise center
	# at a glance (the actual physics-relevant point).
	var dot_mat := StandardMaterial3D.new()
	dot_mat.albedo_color = Color(1.0, 1.0, 0.2)
	dot_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var dot_mesh := SphereMesh.new()
	dot_mesh.radius = 0.012
	dot_mesh.height = 0.024
	dot_mesh.material = dot_mat
	var dot := MeshInstance3D.new()
	dot.name = "CartCentre"
	dot.mesh = dot_mesh
	_cart_mesh.add_child(dot)   # parented so it follows cart

func _update_visuals() -> void:
	if _cart_mesh:
		_cart_mesh.position = Vector3(position_x, sin(3.0 * position_x) * 0.45 + 0.04, 0)

# ---------------------------------------------------------------------------
# JSONL diagnostic emitter (mirrors cart_body.gd's compact shape)
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
				}
			"EPM":
				mods[mod_id] = {
					"nodes":  int(m.get("node_count", 0)),
					"baked":  int(m.get("baked_count", 0)),
					"tle":    snappedf(float(m.get("tle", 0.0)), 0.001),
				}
			"LateralVoter":
				mods[mod_id] = {
					"tle":   snappedf(float(m.get("fused_tle", 0.0)), 0.001),
				}
			"HomeostaticDrive":
				var errs: Dictionary = m.get("errors", {})
				mods[mod_id] = {
					"urg":    snappedf(float(m.get("urgency", 0.0)), 0.001),
					"errors": errs,
				}
			"ActionDecoder":
				mods[mod_id] = {
					"accel":      snappedf(float(m.get("accel", 0.0)), 0.01),
					"ad_disp":    int(m.get("chunk_dispatch_count", 0)),
					"chunk_play": int(m.get("active_chunk_id", -1)),
					"chunk_left": int(m.get("chunk_remaining", 0)),
				}
			"HomeokineticExploration":
				mods[mod_id] = {
					"eps":         int(m.get("episodes_armed", 0)),
					"active":      bool(m.get("active", false)),
					# Phase 6.5.7 gate-debug — see why kinesis isn't firing
					# during MC lulls.
					"long_chg":    snappedf(float(m.get("long_change_ema", 0.0)), 0.000001),
					"urg_fill":    int(m.get("urgency_buffer_fill", 0)),
					"ratio_fill":  int(m.get("ratio_buffer_fill", 0)),
					"chunk_blk":   bool(m.get("chunk_blocks", false)),
					"succ_rate":   snappedf(float(m.get("success_rate", 0.0)), 0.01),
				}
			"SequenceGNG":
				mods[mod_id] = {
					"nodes":  int(m.get("node_count", 0)),
					"baked":  int(m.get("baked_count", 0)),
					"motif":  int(m.get("current_motif", -1)),
				}
			"MotorRepertoire":
				mods[mod_id] = {
					"chunks":      int(m.get("chunk_count", 0)),
					"active":      int(m.get("active_chunk_count", 0)),
					"dispatches":  int(m.get("total_dispatch_count", 0)),
				}
	var line := {
		"t":         tick_counter,
		"ep":        episode_index,
		"step":      step_in_episode,
		"pos":       snappedf(position_x, 0.001),
		"vel":       snappedf(velocity,   0.001),
		"accel":     snappedf(accel, 0.01),
		"force":     snappedf(_last_action_force, 0.001),
		"ep_r":      int(episode_reward),
		"energy":    snappedf(energy, 0.001),
		"speed_ema":   snappedf(_speed_ema, 0.0001),
		"speed_short": snappedf(_speed_short_ema, 0.0001),
		"swing_ema":   snappedf(_swing_ema, 0.001),
		"swing_short": snappedf(_swing_short_ema, 0.001),
		"modules":     mods,
	}
	print(JSON.stringify(line))
