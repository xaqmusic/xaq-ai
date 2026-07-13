extends "res://scripts/episodic_hud.gd"
## MountainCar-specific HUD: cart pos/vel/energy + Gym v0 solve threshold
## (mean reward ≥ -110 over 100 episodes; per-episode return ≤ -110 = solved).

func _init() -> void:
	body_node_name  = "Cart"
	solve_threshold = -110            # Gym v0 solve criterion: reward >= -110 = solved
	window_episodes = 100

func _env_lines(body: Node) -> Array[String]:
	var pos:    float = float(body.get("position_x"))
	var vel:    float = float(body.get("velocity"))
	var energy: float = float(body.get("energy"))
	var dist_to_goal: float = 0.5 - pos
	# Read body's per-physics-tick session bests rather than tracking
	# per-render-frame here.  HUD samples at 60 Hz, physics ticks at
	# 50 Hz with multi-tick accumulator — at velocity ±0.07 the cart can
	# pass the goal between HUD samples.  Body sees every tick.
	var max_pos:    float = float(body.get("_session_max_pos"))
	var min_pos:    float = float(body.get("_session_min_pos"))
	var best_speed: float = float(body.get("_session_best_speed"))
	var swing_range: float = max_pos - min_pos
	var goal_hits:   int   = int(body.get("_session_goal_hits"))
	var elapsed_min: float = float(Time.get_ticks_msec() - _start_ms) / 60000.0
	var goals_per_min: float = 0.0
	if elapsed_min > 0.01:
		goals_per_min = float(goal_hits) / elapsed_min
	# Phase 6.5.8/9 — dual-EMA reward + force-velocity alignment gate.
	# HUD's "HIT" indicator matches the body's actual firing condition:
	#   short_ema > long_ema × 1.5  AND  force × velocity > 0
	# The alignment requirement filters out coasting/braking moments
	# (cart fast but action opposite or null) so HIT only shows when
	# the brain is actively adding kinetic energy.
	var speed_long:  float = float(body.get("_speed_ema"))
	var swing_long:  float = float(body.get("_swing_ema"))
	var speed_short: float = float(body.get("_speed_short_ema"))
	var swing_short: float = float(body.get("_swing_short_ema"))
	var last_force:  float = float(body.get("_last_action_force"))
	var aligned:     bool  = (last_force * vel) > 0.0
	var speed_hot:   bool  = aligned and speed_long > 0.001 and speed_short > speed_long * 1.5
	var swing_hot:   bool  = aligned and swing_long > 0.001 and swing_short > swing_long * 1.5
	# Phase 6.5.7 — goal-time EMA shows the brain's running average
	# time-to-goal.  Lower is better.  When a fresh goal arrives at
	# < 0.9 × this EMA, an extra reward pulse fires (visible to the
	# user as a sudden dopamine spike on the brain panel).
	var goal_time_ema: float = float(body.get("_goal_time_ema"))
	var ema_str: String = "—" if goal_time_ema <= 0.0 else ("%.0f ticks" % goal_time_ema)
	return [
		"cart:      pos=%+.3f  vel=%+.4f  goal in %+.2f" % [pos, vel, dist_to_goal],
		"energy:    %.3f" % energy,
		"swing:     min=%+.2f  max=%+.2f  range=%.2f  (goal at +0.50)" % [
			min_pos, max_pos, swing_range,
		],
		"best |v|:  %.4f  (max possible: %.4f)" % [best_speed, 0.07],
		"reward:    spd %.4f/%.4f%s   swg %.2f/%.2f%s   (short/long)" % [
			speed_short, speed_long, "  HIT" if speed_hot else "",
			swing_short, swing_long, "  HIT" if swing_hot else "",
		],
		"goals:     %d  (%.2f/min)   avg time-to-goal: %s" % [
			goal_hits, goals_per_min, ema_str,
		],
	]

var _start_ms: int = 0

func _ready() -> void:
	super._ready()
	_start_ms = Time.get_ticks_msec()

func _env_summary_lines(body: Node) -> Array[String]:
	var max_pos: float = float(body.get("_session_max_pos"))
	var min_pos: float = float(body.get("_session_min_pos"))
	var goal_hits: int = int(body.get("_session_goal_hits"))
	return [
		"MountainCar-v0:",
		"  Solve threshold:    ≤−110 mean reward over 100 episodes",
		"  Best swing range:   %.2f units  (goal needs body to reach +0.50)" % (max_pos - min_pos),
		"  Best position:      %+.2f  (goal at +0.50)" % max_pos,
		"  Total goals hit:    %d" % goal_hits,
	]
