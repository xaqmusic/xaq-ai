extends Control
## Common HUD overlay for episodic environments (CartPole, MountainCar).
##
## Polls the body each frame (no signal wiring) and shows:
##   - Episode counter, current step, current episode reward
##   - Hit/miss tally (solves vs fails) for the session
##   - Best episode reward; mean/max over last N
##   - Brain state: dopamine, urgency, action
##   - Env-specific state lines via _env_lines() override hook
##
## Subclasses set `body_node_name`, `solve_threshold`, and override
## `_env_lines(body)` to print per-env state (cart pos/angle, MC pos/vel).

@export var body_node_name:    String = "Cart"
@export var solve_threshold:   int    = 475          # CartPole-v1 default; MC overrides to -110
@export var window_episodes:   int    = 100          # sliding-window size for solve-rate
@export var label_max_lines:   int    = 14
# Both episodic envs use "reward >= threshold = solved" semantics.
# CartPole: balance reward-of-475 ticks (~9.5s) ≥ 475 = solve.
# MountainCar: terminal reward of -110 ≥ -110 = solve (closer to 0
# is better for negative rewards — fewer alive ticks before goal).
# Earlier "solve_predicate" param was an inverted bug-magnet; removed.

var _label: Label
var _last_episode_seen: int = -1

# Phase 6.6.F — realtime MotorFader α meter (top-right).  Visible only
# when a MotorFader is publishing on motor.fader.alpha; configs without
# crossfade get no stub gauge.  Inherited by cartpole_hud and
# mountain_car_hud automatically.
const _MotorFaderMeterScene: GDScript = preload("res://scripts/motor_fader_meter.gd")
var _fader_meter: Control = null
var _episode_rewards: Array[int] = []   # ring of last N completed-episode returns
var _solves: int = 0
var _fails: int = 0
var _best_reward: int = -2147483647

# Phase 6.5.3.B — end-of-run summary modal.  Shown automatically when
# the body's _done flag flips true (max_episodes reached, OR pause-
# overlay's "End run" button).  Modal blocks until the user clicks
# "Return to launcher" — at which point the scene swaps back.
var _summary_root: Control = null
var _summary_shown: bool   = false

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_label = Label.new()
	_label.add_theme_font_size_override("font_size", 16)
	_label.add_theme_color_override("font_color", Color(1, 1, 1, 1))
	_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	_label.add_theme_constant_override("outline_size", 4)
	_label.position = Vector2(12, 8)
	_label.size     = Vector2(520, 240)
	add_child(_label)
	_fader_meter = _MotorFaderMeterScene.new()
	_fader_meter.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	_fader_meter.position = Vector2(-(_fader_meter.custom_minimum_size.x + 12), 12)
	add_child(_fader_meter)

func _process(_delta: float) -> void:
	var body: Node = get_tree().get_root().find_child(body_node_name, true, false)
	if body == null:
		_label.text = "%s body not found" % body_node_name
		return

	# Phase 6.5.3.B — end-of-run detection.  Body sets _done when
	# max_episodes reached (interactive mode only).  HUD shows the
	# summary modal once and then awaits user action.
	if not _summary_shown and bool(body.get("_done")):
		_summary_shown = true
		_show_summary(body)

	var ep:    int = int(body.get("episode_index"))
	var step:  int = int(body.get("step_in_episode"))
	var reward_now: int = int(body.get("episode_reward"))

	# Record episode boundary: when episode_index advances, snapshot the
	# reward of the just-finished episode (it was reset by body before
	# _process polled, so we use the snapshot we kept of "last reward").
	# Body sets episode_reward = 0 in _reset_state.  We capture by watching
	# for ep transitions and using the prior tick's stored value.
	if _last_episode_seen >= 0 and ep != _last_episode_seen:
		# previous episode's terminal reward was the LAST observed reward_now
		# before this transition.  We approximate by reading the previous
		# tick's reward_now (saved in _prev_reward_at_episode_end).
		var prev_r: int = _prev_reward_at_episode_end
		_record_episode(prev_r)
	_last_episode_seen = ep
	if reward_now != 0:
		_prev_reward_at_episode_end = reward_now

	var window_n: int = mini(_episode_rewards.size(), window_episodes)
	var window_mean: float = 0.0
	var window_max:  int   = -2147483647
	if window_n > 0:
		var tail := _episode_rewards.slice(_episode_rewards.size() - window_n)
		var s: int = 0
		for r in tail:
			s += r
			if r > window_max: window_max = r
		window_mean = float(s) / float(window_n)
	var solve_rate: float = 0.0
	if (_solves + _fails) > 0:
		solve_rate = 100.0 * float(_solves) / float(_solves + _fails)

	# Brain state.
	var brain = body.get("brain")
	if _fader_meter != null and brain != null:
		_fader_meter.set_brain(brain)
	var da_str: String = "—"
	var urg_str: String = "—"
	var accel_str: String = "—"
	if brain != null and brain.has_method("is_brain_ready") and brain.is_brain_ready():
		var metrics: Dictionary = brain.get_module_metrics()
		var n: Dictionary = metrics.get("neuro", {})
		var dr: Dictionary = metrics.get("drive", {})
		var ad: Dictionary = metrics.get("action_decoder", {})
		da_str    = "%.2f" % float(n.get("dopamine", 0.0))
		urg_str   = "%.2f" % float(dr.get("urgency", 0.0))
		accel_str = "%+.1f" % float(ad.get("accel", 0.0))

	# Compose lines.
	var lines: Array[String] = []
	lines.append("episode:   #%d  step %d  reward=%d  best=%d" % [
		ep, step, reward_now, _best_reward if _best_reward != -2147483647 else 0
	])
	lines.append("session:   solves=%d  fails=%d  rate=%.1f%%  (thresh r≥%d)" % [
		_solves, _fails, solve_rate, solve_threshold
	])
	if window_n > 0:
		lines.append("last %d:    μ=%.1f  max=%d  (n=%d)" % [
			window_episodes, window_mean, window_max, window_n
		])
	else:
		lines.append("last %d:    no completed episodes yet" % window_episodes)
	lines.append("brain:     da=%s  urg=%s  accel=%s" % [da_str, urg_str, accel_str])
	# Env-specific lines.
	for ln in _env_lines(body):
		lines.append(ln)
	lines.append("[ESC] pause / return to launcher")
	_label.text = "\n".join(lines)

# -- Subclass override hooks ---------------------------------------------------

# Returns env-specific lines (cart pos/angle, MC pos/vel, etc.).
# Default implementation: empty.
func _env_lines(_body: Node) -> Array[String]:
	return []

# -- Internal -----------------------------------------------------------------

var _prev_reward_at_episode_end: int = 0

# -- End-of-run summary modal -------------------------------------------------

# Public — pause overlay calls this when the user picks "End run early".
# Sets the body's _done flag (which triggers the modal next _process tick).
func request_end_run() -> void:
	var body: Node = get_tree().get_root().find_child(body_node_name, true, false)
	if body and not bool(body.get("_done")):
		body.set("_done", true)

func _show_summary(body: Node) -> void:
	if _summary_root != null:
		return
	_summary_root = Control.new()
	_summary_root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_summary_root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_summary_root)

	# Translucent backdrop.
	var bg := ColorRect.new()
	bg.color = Color(0, 0, 0, 0.7)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_STOP
	_summary_root.add_child(bg)

	# Modal panel.
	var panel := PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_CENTER)
	panel.custom_minimum_size = Vector2(620, 420)
	_summary_root.add_child(panel)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left",   28)
	margin.add_theme_constant_override("margin_right",  28)
	margin.add_theme_constant_override("margin_top",    24)
	margin.add_theme_constant_override("margin_bottom", 24)
	panel.add_child(margin)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 12)
	margin.add_child(v)

	var title := Label.new()
	title.text = "Run finished — summary"
	title.add_theme_font_size_override("font_size", 22)
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	v.add_child(title)

	var report := RichTextLabel.new()
	report.bbcode_enabled = false
	report.selection_enabled = true
	report.focus_mode = Control.FOCUS_ALL
	report.size_flags_vertical = Control.SIZE_EXPAND_FILL
	report.add_theme_font_size_override("normal_font_size", 13)
	report.text = _build_summary_text(body)
	v.add_child(report)

	# Buttons.
	var hb := HBoxContainer.new()
	hb.alignment = BoxContainer.ALIGNMENT_END
	hb.add_theme_constant_override("separation", 12)
	v.add_child(hb)

	# Phase 6.5.14 — clipboard export.  Puts a detailed log on the
	# system clipboard in roughly the same format a headless run would
	# emit (config + per-episode rewards + final brain state + summary
	# stats) so the user can paste it into a chat with Claude when
	# describing observations.
	var copy_btn := Button.new()
	copy_btn.text = "📋 Copy log"
	copy_btn.custom_minimum_size = Vector2(140, 36)
	copy_btn.pressed.connect(func():
		var txt: String = build_clipboard_text(body)
		DisplayServer.clipboard_set(txt)
		copy_btn.text = "✓ Copied"
		# Revert label after a short delay so the user sees the
		# confirmation but the button is still re-clickable.
		var t := Timer.new()
		t.wait_time = 1.5
		t.one_shot = true
		t.timeout.connect(func():
			copy_btn.text = "📋 Copy log"
			t.queue_free()
		)
		copy_btn.add_child(t)
		t.start()
	)
	hb.add_child(copy_btn)

	var quit_btn := Button.new()
	quit_btn.text = "Quit"
	quit_btn.custom_minimum_size = Vector2(120, 36)
	quit_btn.pressed.connect(func(): get_tree().quit())
	hb.add_child(quit_btn)

	var return_btn := Button.new()
	return_btn.text = "Return to launcher"
	return_btn.custom_minimum_size = Vector2(180, 36)
	return_btn.pressed.connect(_on_return_to_launcher)
	hb.add_child(return_btn)

func _build_summary_text(body: Node) -> String:
	var lines: Array[String] = []
	var ep_done: int = int(body.get("episode_index"))
	var window_n: int = mini(_episode_rewards.size(), window_episodes)
	var window_mean: float = 0.0
	var window_max: int    = -2147483647
	if window_n > 0:
		var tail := _episode_rewards.slice(_episode_rewards.size() - window_n)
		var s: int = 0
		for r in tail:
			s += r
			if r > window_max: window_max = r
		window_mean = float(s) / float(window_n)
	var solve_rate: float = 100.0 * float(_solves) / max(1.0, float(_solves + _fails))

	lines.append("Episodes completed:   %d" % ep_done)
	lines.append("Solves / fails:       %d / %d  (%.1f%% solve rate)" % [_solves, _fails, solve_rate])
	if _best_reward != -2147483647:
		lines.append("Best episode reward:  %d" % _best_reward)
	if window_n > 0:
		lines.append("Last %d episodes:    μ=%.1f  max=%d" % [window_episodes, window_mean, window_max])
	# Subclass-supplied env-specific lines.
	var env_summary := _env_summary_lines(body)
	if not env_summary.is_empty():
		lines.append("")
		for ln in env_summary:
			lines.append(ln)
	# Trailing reward distribution snippet.
	lines.append("")
	lines.append("Reward histogram (last %d):" % window_n)
	if window_n > 0:
		var tail2 := _episode_rewards.slice(_episode_rewards.size() - window_n)
		lines.append("  " + _ascii_histogram(tail2))
	return "\n".join(lines)

# Per-env extension hook for the summary modal.  CartPole/MC override.
func _env_summary_lines(_body: Node) -> Array[String]:
	return []

# Phase 6.5.14 — clipboard export of detailed run log.  Format mirrors
# what a headless harness (cartpole_run.py / mountain_car_run.py) would
# emit: env metadata + per-episode rewards + final brain state +
# summary stats.  Paste into a chat / report when describing
# observations from a UI run.
#
# Public so the pause-overlay's "Copy log" button (Phase 6.5.14b) can
# call into this from any tick, not just the end-of-run modal.
func build_clipboard_text(body: Node) -> String:
	var lines: Array[String] = []
	# --- Header ---
	lines.append("=== AMI-Ogma UI run summary ===")
	lines.append("Body:        %s" % body_node_name)
	lines.append("Config:      %s" % str(ExperimentConfig.config_path))
	lines.append("Seed:        %d" % int(ExperimentConfig.seed_value))
	lines.append("Episodes:    %d" % int(body.get("episode_index")))
	if body.has_method("get") and body.get("tick_counter") != null:
		lines.append("Total ticks: %d" % int(body.get("tick_counter")))
	lines.append("")

	# --- Episode stats ---
	var ep_done: int = int(body.get("episode_index"))
	var solve_rate: float = 100.0 * float(_solves) / max(1.0, float(_solves + _fails))
	lines.append("Solves / fails: %d / %d  (%.1f%% solve rate)" % [_solves, _fails, solve_rate])
	if _best_reward != -2147483647:
		lines.append("Best episode reward: %d" % _best_reward)
	var window_n: int = mini(_episode_rewards.size(), window_episodes)
	if window_n > 0:
		var tail := _episode_rewards.slice(_episode_rewards.size() - window_n)
		var sum_r: int = 0
		var max_r: int = -2147483647
		for r in tail:
			sum_r += r
			if r > max_r: max_r = r
		var mean_r: float = float(sum_r) / float(window_n)
		lines.append("Last %d episodes: μ=%.1f  max=%d  (n=%d)" % [
			window_episodes, mean_r, max_r, window_n,
		])
	lines.append("")

	# --- Env-specific lines ---
	var env_lines := _env_summary_lines(body)
	if not env_lines.is_empty():
		for ln in env_lines: lines.append(ln)
		lines.append("")

	# --- Full episode reward sequence ---
	# The histogram on screen is a binned summary; the clipboard log
	# includes EVERY episode's reward so downstream analysis can
	# reconstruct the full trajectory.
	lines.append("=== Episode rewards (all %d) ===" % _episode_rewards.size())
	if _episode_rewards.size() <= 200:
		var line := ""
		for i in range(_episode_rewards.size()):
			line += "%d," % _episode_rewards[i]
			if (i + 1) % 20 == 0:
				lines.append(line.substr(0, line.length() - 1))
				line = ""
		if line.length() > 0:
			lines.append(line.substr(0, line.length() - 1))
	else:
		# Long runs: show first 50 and last 50 only to keep clipboard
		# manageable but still informative.
		lines.append("(first 50)")
		var f_line := ""
		for i in range(50):
			f_line += "%d," % _episode_rewards[i]
			if (i + 1) % 20 == 0:
				lines.append(f_line.substr(0, f_line.length() - 1))
				f_line = ""
		if f_line.length() > 0:
			lines.append(f_line.substr(0, f_line.length() - 1))
		lines.append("...(%d more)..." % (_episode_rewards.size() - 100))
		lines.append("(last 50)")
		var l_line := ""
		for i in range(_episode_rewards.size() - 50, _episode_rewards.size()):
			l_line += "%d," % _episode_rewards[i]
			if (i - (_episode_rewards.size() - 50) + 1) % 20 == 0:
				lines.append(l_line.substr(0, l_line.length() - 1))
				l_line = ""
		if l_line.length() > 0:
			lines.append(l_line.substr(0, l_line.length() - 1))
	lines.append("")

	# --- Final brain state ---
	var brain = body.get("brain")
	if brain != null and brain.has_method("get_module_metrics"):
		lines.append("=== Final brain metrics ===")
		var metrics: Dictionary = brain.get_module_metrics()
		for mod_id in metrics:
			var m: Dictionary = metrics[mod_id]
			var t: String = str(m.get("type", ""))
			# Format each module on its own line for readability;
			# include the fields most useful for diagnosis.
			match t:
				"NeurochemState":
					lines.append("  %s: da=%.3f ht=%.3f r_sig=%.3f" % [
						mod_id, float(m.get("dopamine", 0)),
						float(m.get("serotonin", 0)),
						float(m.get("reward_signal", 0)),
					])
				"EPM":
					lines.append("  %s: nodes=%d baked=%d tle=%.4f" % [
						mod_id, int(m.get("node_count", 0)),
						int(m.get("baked_count", 0)),
						float(m.get("tle", 0)),
					])
				"LateralVoter":
					lines.append("  %s: tle=%.4f modality=%s" % [
						mod_id, float(m.get("fused_tle", 0)),
						str(m.get("active_modality", "")),
					])
				"HomeostaticDrive":
					lines.append("  %s: urg=%.3f" % [
						mod_id, float(m.get("urgency", 0)),
					])
				"ActionDecoder":
					lines.append("  %s: accel=%.2f val_sz=%d active_chunk=%d" % [
						mod_id, float(m.get("accel", 0)),
						int(m.get("valence_size", 0)),
						int(m.get("active_chunk_id", -1)),
					])
				"SequenceGNG":
					lines.append("  %s: nodes=%d baked=%d motif=%d" % [
						mod_id, int(m.get("node_count", 0)),
						int(m.get("baked_count", 0)),
						int(m.get("current_motif", -1)),
					])
				"MotorRepertoire":
					lines.append("  %s: chunks=%d active=%d disp=%d failed=%d" % [
						mod_id, int(m.get("chunk_count", 0)),
						int(m.get("active_chunk_count", 0)),
						int(m.get("total_dispatch_count", 0)),
						int(m.get("failed_dispatch_count", 0)),
					])
				"HomeokineticExploration":
					lines.append("  %s: armed=%d active=%s" % [
						mod_id, int(m.get("episodes_armed", 0)),
						str(m.get("active", false)),
					])
		# --- Chunk library detail (MotorRepertoire chunks array) ---
		for mod_id in metrics:
			var m: Dictionary = metrics[mod_id]
			if str(m.get("type", "")) != "MotorRepertoire": continue
			var chunks = m.get("chunks", null)
			if chunks == null or not (chunks is Array): continue
			if chunks.is_empty(): continue
			lines.append("")
			lines.append("=== Chunk library (%d chunks) ===" % chunks.size())
			for c in chunks:
				var seq: Array = c.get("accel_seq", [])
				var seq_str := ""
				for v in seq:
					seq_str += "%+.1f," % float(v)
				if seq_str.length() > 0:
					seq_str = seq_str.substr(0, seq_str.length() - 1)
				lines.append("  id=%d len=%d use=%d trigger_motif=%d trigger_urg=%.3f hits=%d/%d/%d (during/replay+/replay-)" % [
					int(c.get("id", -1)),
					int(c.get("length", 0)),
					int(c.get("use_count", 0)),
					int(c.get("trigger_motif", -1)),
					float(c.get("trigger_urgency", -1.0)),
					int(c.get("hits_during", 0)),
					int(c.get("replay_hits", 0)),
					int(c.get("replay_misses", 0)),
				])
				lines.append("    seq=[%s]" % seq_str)
	return "\n".join(lines)

func _ascii_histogram(values: Array, width: int = 32) -> String:
	if values.is_empty(): return ""
	var lo: float = float(values[0]); var hi: float = float(values[0])
	for v in values:
		var f := float(v)
		if f < lo: lo = f
		if f > hi: hi = f
	if hi == lo:
		return "[all values = %.0f]" % lo
	var bins: Array[int] = []
	for i in range(10): bins.append(0)
	for v in values:
		var idx: int = int((float(v) - lo) / (hi - lo) * 9.999)
		bins[idx] += 1
	var max_count: int = 0
	for c in bins:
		if c > max_count: max_count = c
	var s := ""
	for c in bins:
		var bar := int(round(float(c) / float(max_count) * width))
		s += "▏"
		s += "█".repeat(bar)
		s += "  %d\n  " % c
	s = s.strip_edges()
	return "min=%.0f max=%.0f\n  %s" % [lo, hi, s]

func _on_return_to_launcher() -> void:
	# Reset launched flag so next launcher cycle reads selections from
	# scratch (lets user pick a different config without persisted-state
	# overrides bleeding through).
	ExperimentConfig.launched = false
	get_tree().change_scene_to_file("res://scenes/launcher.tscn")

func _record_episode(reward: int) -> void:
	_episode_rewards.append(reward)
	if _episode_rewards.size() > window_episodes:
		_episode_rewards = _episode_rewards.slice(_episode_rewards.size() - window_episodes)
	# Solve/fail tally.  For CartPole solve_predicate="ge" with thresh 475,
	# for MountainCar solve_predicate="le" with thresh -110.  Both round-trip
	# the standard Gym solve criterion.
	if reward >= solve_threshold:
		_solves += 1
	else:
		_fails += 1
	if reward > _best_reward:
		_best_reward = reward
