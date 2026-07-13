extends "res://scripts/episodic_hud.gd"
## CartPole-specific HUD: adds cart state line + uses Gym v1 solve
## threshold (≥475 over 100 episodes).

func _init() -> void:
	body_node_name  = "Cart"
	solve_threshold = 475             # Gym v1: reward >= 475 = solved
	window_episodes = 100

const _TAU: float = 0.02   # CartPole physics tau (50 Hz) — matches cart_body.gd::TAU

func _env_lines(body: Node) -> Array[String]:
	var x:        float = float(body.get("x"))
	var x_dot:    float = float(body.get("x_dot"))
	var theta:    float = float(body.get("theta"))
	var th_dot:   float = float(body.get("theta_dot"))
	var th_deg:   float = rad_to_deg(theta)
	var fail_x_pct:  float = abs(x) / 2.4 * 100.0
	var fail_th_pct: float = abs(theta) / 0.20943951 * 100.0  # ±12°

	# Longest balance.  Episode reward = surviving-tick count; each tick
	# = TAU = 20 ms, so best_reward × TAU is the longest pole-balance
	# interval the substrate has held in this session.  Useful as a
	# real-time metric independent of the Gym solve threshold.
	var current_balance_s: float = float(body.get("step_in_episode")) * _TAU
	var best_balance_s:    float = 0.0
	if _best_reward != -2147483647:
		best_balance_s = float(_best_reward) * _TAU
	return [
		"cart:      x=%+.2fm  vx=%+.2fm/s   |x|=%.0f%% of fail" % [x, x_dot, fail_x_pct],
		"pole:      θ=%+.1f°  ω=%+.2frad/s  |θ|=%.0f%% of fail" % [th_deg, th_dot, fail_th_pct],
		"balance:   current=%s   longest=%s" % [_fmt_secs(current_balance_s), _fmt_secs(best_balance_s)],
	]

# Format seconds as "1.24 s" or "843 ms" or "5m 12s" depending on
# magnitude.  Keeps the HUD readable across the full range from a
# half-second cold-start fall to a 10-second mid-training balance to
# the 10-second cap on a Gym solve.
func _env_summary_lines(_body: Node) -> Array[String]:
	# Translate best episode tick count to wall seconds (50 Hz physics).
	var best_s: float = 0.0
	if _best_reward != -2147483647:
		best_s = float(_best_reward) * _TAU
	return [
		"CartPole-v1:",
		"  Solve threshold:    ≥475 ticks (≈9.50 s) over 100 episodes",
		"  Longest balance:    %s (%d ticks)" % [_fmt_secs(best_s), max(0, _best_reward)],
	]

func _fmt_secs(s: float) -> String:
	if s < 1.0:
		return "%d ms" % int(round(s * 1000.0))
	if s < 60.0:
		return "%.2f s" % s
	var m: int = int(s) / 60
	var r: float = s - float(m * 60)
	return "%dm %05.2fs" % [m, r]
