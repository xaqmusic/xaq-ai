extends Control
## Minimal v6.0.c HUD for the quadruped scene.  Polls the body each frame
## (no signal wiring) and shows: episode count, current alive ticks,
## best alive duration so far, brain heartbeat (DA / HT / Premotor H /
## chassis height + tilt).
##
## Intentionally light — no solve-threshold logic (standing doesn't have
## a Gym-style solved threshold), no per-episode reward histogram, no
## end-of-run modal (the body runs in continuous mode for v6.0.c).
## Extension hooks are easy to add once a clearer success metric emerges.

const _BODY_CANDIDATES: Array = ["Quadruped", "Picrawler"]
const _TAU: float = 0.02   # physics tick = 20 ms; matches quadruped_body.gd::TAU

var _label: Label
var _calib_label: Label   # separate label whose font_color tracks the active leg
var _best_alive_ticks: int = 0
var _last_episode_seen: int = -1
var _prev_alive_at_episode_end: int = 0

# End-of-run summary modal state (matches hud.gd's pattern so the pause
# overlay's "End run & show summary" button works the same way as it
# does in Cell).  Mounted as a child of this HUD when the user requests
# it; _summary_shown guards against duplicate creation.
var _summary_root: Control = null
var _summary_shown: bool = false

# Cumulative metrics tracked HUD-side so the summary can report fall
# counts and tipover counts the same way the headless harness does.
# Sampled per _process; matches the harness's diag-tick aggregation
# logic but live during the run.
var _n_fall_events: int = 0
var _n_tipover_events: int = 0
var _prev_y_above_fail: bool = true     # for fall-event edge detection
var _prev_tilt_above_tipover: bool = false  # for tipover-event edge detection
var _below_fail_ticks: int = 0           # cumulative below-FAIL count
var _tipover_ticks: int = 0              # cumulative tilt>π/2 count
var _hud_sample_ticks: int = 0           # increments per _process call

func _as_bool(v: Variant) -> bool:
	if v == null:
		return false
	match typeof(v):
		TYPE_BOOL:
			return v
		TYPE_INT, TYPE_FLOAT:
			return v != 0
		TYPE_STRING:
			var t: String = str(v).to_lower()
			return t == "true" or t == "1" or t == "yes" or t == "on"
		_:
			return false

func _as_int(v: Variant, fallback: int = 0) -> int:
	if v == null:
		return fallback
	match typeof(v):
		TYPE_INT:
			return v
		TYPE_FLOAT:
			return roundi(v)
		TYPE_BOOL:
			return 1 if v else 0
		TYPE_STRING:
			return str(v).to_int()
		_:
			return fallback


# Phase 6.7++ — per-mechanism indicator state.  Tracks when HomeokineticExploration
# fires and how recently the body-side mechanisms (EPM swap, escape detector)
# activated.  Pip text is recomputed each _process tick from brain.get_module_metrics
# and body-side counters.  "Idle" / "ACTIVE" / "off" labels for read-at-a-glance
# visibility — the user observed today's 62-min freeze had no live indicator that
# HK had even fired.
var _last_hk_armed_seen: int = -1            # delta detector for HK fire flash
var _hk_recent_fire_ticks_left: int = 0      # ticks remaining to show [FIRED] flash
var _last_swap_total_seen: int = 0
var _swap_recent_ticks_left: int = 0
var _last_escape_total_seen: int = 0
var _escape_recent_ticks_left: int = 0
var _last_curr_idx_seen: int = -1
var _curr_advance_ticks_ago: int = 0
# Phase 6.9 — PR flash tracker.  Detects when best_speed just bumped up
# and flashes the speed line for ~0.5 s so the user catches the moment.
var _last_best_speed_seen: float = -1.0
var _pr_flash_frames_left: int = 0
const _PIP_FLASH_FRAMES: int = 30            # ~0.5 s flash window after a fire
# Matches picrawler_body.gd LEG_COLORS array.  Index = leg (0=FL, 1=FR, 2=RL, 3=RR).
const _LEG_COLORS: Array = [
	Color(0.95, 0.20, 0.20, 1.0),
	Color(0.20, 0.85, 0.20, 1.0),
	Color(0.30, 0.50, 0.95, 1.0),
	Color(0.95, 0.85, 0.20, 1.0),
]

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_label = Label.new()
	_label.add_theme_font_size_override("font_size", 16)
	_label.add_theme_color_override("font_color", Color(1, 1, 1, 1))
	_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	_label.add_theme_constant_override("outline_size", 4)
	_label.position = Vector2(12, 8)
	_label.size     = Vector2(720, 320)
	add_child(_label)
	# Bigger label for the calibration step display, positioned below.
	_calib_label = Label.new()
	_calib_label.add_theme_font_size_override("font_size", 28)
	_calib_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	_calib_label.add_theme_constant_override("outline_size", 6)
	_calib_label.position = Vector2(12, 220)
	_calib_label.size     = Vector2(540, 60)
	_calib_label.visible  = false
	add_child(_calib_label)

func _process(_delta: float) -> void:
	var body: Node = null
	for nm in _BODY_CANDIDATES:
		body = get_tree().get_root().find_child(nm, true, false)
		if body != null:
			break
	if body == null:
		_label.text = "Body not found (looked for %s)" % str(_BODY_CANDIDATES)
		return

	var ep: int   = _as_int(body.get("episode_index"))
	var step: int = _as_int(body.get("step_in_episode"))
	var alive: int = _as_int(body.get("episode_alive_ticks"))
	# Cumulative-since-last-fall (picrawler v6.0.b.9).  Survives the
	# mc_episode_period boundary resets that cap episode_alive_ticks
	# at mc_period × _TAU (30 s for mc=1500).  Falls back to 0 if the
	# body doesn't expose this field (e.g. quadruped).
	var cumulative: int     = _as_int(body.get("cumulative_alive_ticks") if body.get("cumulative_alive_ticks") != null else 0)
	var best_cumulative: int = _as_int(body.get("best_cumulative_alive_ticks") if body.get("best_cumulative_alive_ticks") != null else 0)

	# Record best across completed episodes — body resets episode_alive_ticks
	# to 0 in _reset_episode(), so we snapshot at the boundary.
	if _last_episode_seen >= 0 and ep != _last_episode_seen:
		if _prev_alive_at_episode_end > _best_alive_ticks:
			_best_alive_ticks = _prev_alive_at_episode_end
	_last_episode_seen = ep
	if alive > 0:
		_prev_alive_at_episode_end = alive

	var current_s:    float = float(alive) * _TAU
	var best_s:       float = float(_best_alive_ticks) * _TAU
	var cumulative_s: float = float(cumulative) * _TAU
	var best_cumul_s: float = float(best_cumulative) * _TAU

	# Brain heartbeat.
	var brain = body.get("brain")
	var da_str: String  = "—"
	var ht_str: String  = "—"
	var preH_str: String = "—"
	if brain != null and brain.has_method("is_brain_ready") and brain.is_brain_ready():
		var metrics: Dictionary = brain.get_module_metrics()
		var n: Dictionary  = metrics.get("neuro", {})
		var pm: Dictionary = metrics.get("premotor", {})
		da_str   = "%.2f" % float(n.get("dopamine",  0.0))
		ht_str   = "%.2f" % float(n.get("serotonin", 0.0))
		preH_str = "%.2f / ln(N)=%.2f" % [
			float(pm.get("last_entropy", 0.0)),
			log(max(1.0, float(pm.get("n_intents", 5))))
		]

	# Chassis kinematics for situational awareness while watching.
	var y_str: String    = "—"
	var tilt_str: String = "—"
	# Picrawler FAIL_HEIGHT=0.025m, FAIL_TILT=1.05rad — matches body constants
	# (harness mirrors these in scripts/picrawler_run.py).
	const _FAIL_HEIGHT: float = 0.025
	const _FAIL_TILT:   float = 1.05
	const _TIPOVER_TILT: float = 1.57   # π/2
	if body.has_method("get") and body.get("_chassis") != null:
		var ch: Node = body.get("_chassis")
		if ch is RigidBody3D:
			var pos: Vector3 = (ch as RigidBody3D).global_transform.origin
			y_str = "%.2fm" % pos.y
			var tilt_rad: float = _tilt(ch.global_transform.basis)
			tilt_str = "%.0f°" % rad_to_deg(tilt_rad)
			# Sample stability metrics for the end-run summary.
			_hud_sample_ticks += 1
			var y_above_fail: bool = pos.y >= _FAIL_HEIGHT and tilt_rad <= _FAIL_TILT
			if not y_above_fail:
				_below_fail_ticks += 1
				if _prev_y_above_fail:
					_n_fall_events += 1
				_prev_y_above_fail = false
			else:
				_prev_y_above_fail = true
			if tilt_rad > _TIPOVER_TILT:
				_tipover_ticks += 1
				if not _prev_tilt_above_tipover:
					_n_tipover_events += 1
				_prev_tilt_above_tipover = true
			else:
				_prev_tilt_above_tipover = false

	var mode: String = str(body.get("reset_mode"))
	# Soft-blink fields exist on quadruped body only; picrawler doesn't
	# implement that reset mode.  body.get(missing) returns null, and
	# _as_bool(null) / _as_int(null) throw "Nonexistent constructor".  Guard.
	var blink_v: Variant = body.get("_blink_active")
	var in_blink: bool = (blink_v != null and _as_bool(blink_v))
	var blink_left_v: Variant = body.get("_blink_ticks_left")
	var blink_remaining: int = (_as_int(blink_left_v) if blink_left_v != null else 0)

	var leg_str: float = float(body.get("leg_strength"))
	# Ragdoll + calibration state — picrawler-only fields.
	var ragdoll: Variant = body.get("_ragdoll_mode")
	var ragdoll_active: bool = (ragdoll != null and _as_bool(ragdoll))
	var calib: Variant = body.get("_calibrate_mode")
	var calib_active: bool = (calib != null and _as_bool(calib))
	var calib_step_v: Variant = body.get("_calibrate_step")
	var calib_step: int = (_as_int(calib_step_v) if calib_step_v != null else 0)
	var calib_leg_v: Variant = body.get("calibrate_current_leg")
	var calib_leg: int = (_as_int(calib_leg_v) if calib_leg_v != null else -1)
	var calib_label_v: Variant = body.get("calibrate_current_label")
	var calib_label: String = (str(calib_label_v) if calib_label_v != null else "")
	# Phase 6.7++ — show which config is loaded as the first HUD line.
	# Strips the res://addons/.../configs/ prefix and .json suffix to
	# keep the line compact.  Helps disambiguate variants when running
	# from the launcher (today: baseline vs lr_symmetric vs swap).
	var cfg_path_v: Variant = body.get("config_path")
	var cfg_name: String = "(unknown config)"
	if cfg_path_v != null:
		cfg_name = String(cfg_path_v).get_file().trim_suffix(".json")
	var lines: Array[String] = []
	lines.append("config:     %s" % cfg_name)
	lines.append("episode:    #%d   step %d   reset_mode=%s%s%s%s" % [
		ep, step, mode,
		("   [BLINK %d]" % blink_remaining) if in_blink else "",
		"   [RAGDOLL]" if ragdoll_active else "",
		("   [CALIBRATE step %d]" % calib_step) if calib_active else "",
	])
	# Two timers:
	#   ep:  per-mc-episode (resets every mc_period × TAU; for MC trajectory accounting)
	#   up:  since last catastrophic fall — what "how long has the brain been standing" actually means
	lines.append("standing:   ep=%s   up=%s   best=%s" % [
		_fmt_secs(current_s), _fmt_secs(cumulative_s), _fmt_secs(best_cumul_s)])
	# Walking PB row — only relevant once the body has actually moved
	# away from the origin.  max_distance_from_origin is body-side and
	# persistent across falls / auto-resets / manual resets (matches
	# the "best=" semantics for standing).  Current distance is computed
	# from the chassis x/z snapshot here so it updates every HUD frame.
	var max_dist_v: Variant = body.get("max_distance_from_origin")
	var max_dist: float = float(max_dist_v) if max_dist_v != null else 0.0
	var chassis_pos_v: Variant = body.get("_chassis")
	var cur_dist: float = 0.0
	if chassis_pos_v != null and chassis_pos_v is Node3D:
		var p: Vector3 = chassis_pos_v.global_transform.origin
		cur_dist = Vector2(p.x, p.z).length()
	if max_dist > 0.001 or cur_dist > 0.001:
		lines.append("walking:    now=%.2fm   best=%.2fm" % [cur_dist, max_dist])
	lines.append("chassis:    y=%s   tilt=%s   leg×=%.2f" % [y_str, tilt_str, leg_str])
	lines.append("brain:      da=%s   ht=%s   H=%s" % [da_str, ht_str, preH_str])

	# Phase 6.9 — speed indicators.  Pulls current/best from body fields,
	# flashes [PR!] for ~0.5 s after a new personal record.  Sustained
	# counter shows how long the body has held ≥0.5 m/s contiguously —
	# directly maps to the stage 2 advance gate.
	var speed_now: float = float(body.get("current_speed") if body.get("current_speed") != null else 0.0)
	var speed_ema: float = float(body.get("_speed_ema") if body.get("_speed_ema") != null else 0.0)
	var speed_best: float = float(body.get("best_speed") if body.get("best_speed") != null else 0.0)
	var sustained: int = _as_int(body.get("current_sustained_speed_ticks") if body.get("current_sustained_speed_ticks") != null else 0)
	var best_sustained: int = _as_int(body.get("best_sustained_speed_ticks") if body.get("best_sustained_speed_ticks") != null else 0)
	var pr_total: int = _as_int(body.get("pr_event_count") if body.get("pr_event_count") != null else 0)
	if _last_best_speed_seen >= 0.0 and speed_best > _last_best_speed_seen + 1e-4:
		_pr_flash_frames_left = _PIP_FLASH_FRAMES
	_last_best_speed_seen = speed_best
	var pr_flash: String = ""
	if _pr_flash_frames_left > 0:
		pr_flash = "  [PR!]"
		_pr_flash_frames_left -= 1
	lines.append("speed:      now=%.2f  ema=%.2f m/s   best=%.2f m/s   sustained=%dt (best=%dt)   PRs=%d%s" % [
		speed_now, speed_ema, speed_best, sustained, best_sustained, pr_total, pr_flash])

	# Phase 6.7++ — per-mechanism indicators.  Built from get_module_metrics
	# (HomeokineticExploration) + body-side state vars (EPM swap, escape
	# detector, curriculum).  Pip labels: [off] = mechanism not in topology
	# or env-gate disabled, [idle] = enabled & quiet, [FIRED] = triggered
	# within the last _PIP_FLASH_FRAMES frames, [ACTIVE] = currently
	# overriding (HK only).  All lines emitted unconditionally so column
	# shape doesn't shift between configs.
	var hk_pip: String = "[off]"
	var hk_armed: int = -1
	if brain != null and brain.has_method("get_module_metrics") \
			and brain.has_method("is_brain_ready") and brain.is_brain_ready():
		var metrics2: Dictionary = brain.get_module_metrics()
		for mod_id in metrics2:
			var md: Dictionary = metrics2[mod_id]
			if String(md.get("type", "")) == "HomeokineticExploration":
				hk_armed = _as_int(md.get("episodes_armed", 0))
				if _as_bool(md.get("active", false)):
					hk_pip = "[ACTIVE]"
				else:
					hk_pip = "[idle]"
				break
	# HK fire flash on episodes_armed delta.
	if hk_armed >= 0:
		if _last_hk_armed_seen >= 0 and hk_armed > _last_hk_armed_seen:
			_hk_recent_fire_ticks_left = _PIP_FLASH_FRAMES
		_last_hk_armed_seen = hk_armed
		if _hk_recent_fire_ticks_left > 0:
			hk_pip = "[FIRED]"
			_hk_recent_fire_ticks_left -= 1

	# Body-side EPM swap mechanism.
	var swap_enabled_v: Variant = body.get("_epm_swap_enabled")
	var swap_enabled: bool = (swap_enabled_v != null and _as_bool(swap_enabled_v))
	var swap_total_v: Variant = body.get("_epm_swap_total")
	var swap_total: int = _as_int(swap_total_v) if swap_total_v != null else 0
	var swap_pip: String = "[off]"
	if swap_enabled:
		swap_pip = "[idle]"
		if swap_total > _last_swap_total_seen:
			_swap_recent_ticks_left = _PIP_FLASH_FRAMES
		if _swap_recent_ticks_left > 0:
			swap_pip = "[FIRED]"
			_swap_recent_ticks_left -= 1
	_last_swap_total_seen = swap_total

	# Body-side escape detector (Phase 6.7 task #48 — currently archived/off).
	var escape_enabled_v: Variant = body.get("_escape_detector_enabled")
	var escape_enabled: bool = (escape_enabled_v != null and _as_bool(escape_enabled_v))
	var escape_total_v: Variant = body.get("_escape_fired_total")
	var escape_total: int = _as_int(escape_total_v) if escape_total_v != null else 0
	var escape_pip: String = "[off]"
	if escape_enabled:
		escape_pip = "[idle]"
		if escape_total > _last_escape_total_seen:
			_escape_recent_ticks_left = _PIP_FLASH_FRAMES
		if _escape_recent_ticks_left > 0:
			escape_pip = "[FIRED]"
			_escape_recent_ticks_left -= 1
	_last_escape_total_seen = escape_total

	lines.append("HK %s arms=%d   SWAP %s n=%d   ESC %s n=%d" % [
		hk_pip, max(0, hk_armed),
		swap_pip, swap_total,
		escape_pip, escape_total,
	])

	# Curriculum stage + last-advance indicator.  Currently CurriculumManager
	# is an autoload; emit even when no curriculum loaded so column shape is
	# stable.  Tracking _curr_advance_ticks_ago is approximate (HUD frames,
	# not physics ticks), but accurate enough to see "stage just advanced."
	var curr_str: String = "curriculum: [none]"
	var cur_mgr: Node = get_node_or_null("/root/CurriculumManager")
	if cur_mgr != null and cur_mgr.has_method("has_curriculum") and cur_mgr.has_method("current_name"):
		if cur_mgr.has_curriculum():
			var idx: int = _as_int(cur_mgr.get("current_idx") if cur_mgr.get("current_idx") != null else 0)
			var nm: String = String(cur_mgr.call("current_name"))
			if _last_curr_idx_seen >= 0 and idx != _last_curr_idx_seen:
				_curr_advance_ticks_ago = 0
			_last_curr_idx_seen = idx
			_curr_advance_ticks_ago += 1
			# Convert HUD-frame count to seconds (frame_time ≈ 1/60 s in continuous,
			# but turbo can run hot; treat as approximate).
			var advance_s: float = _curr_advance_ticks_ago / 60.0
			curr_str = "curriculum: stage_%d %s   (advance %.0fs ago)" % [idx, nm, advance_s]
	lines.append(curr_str)

	# Phase 6.13 — transient HUD notification.  Body sets _ui_notification +
	# _ui_notification_until_tick on F5/F9 (or any other significant event).
	# We display the string until the tick_counter passes the expiry — the
	# HUD frame rate isn't tick-locked but reads the same body field every
	# frame, so the message persists for the configured duration.
	var notif_v: Variant = body.get("_ui_notification")
	var notif_until_v: Variant = body.get("_ui_notification_until_tick")
	var body_tick_v: Variant = body.get("tick_counter")
	if notif_v != null and String(notif_v) != "" \
			and notif_until_v != null and body_tick_v != null \
			and _as_int(body_tick_v) <= _as_int(notif_until_v):
		lines.append(">> %s" % String(notif_v))
	lines.append("")
	var space_hint: String = "[SPACE] reset" if mode == "continuous" else "[SPACE] manual reset"
	var ragdoll_hint: String = ""
	if ragdoll != null:
		ragdoll_hint = "   [R] ragdoll: %s" % ("ON" if ragdoll_active else "off")
	var calib_hint: String = ""
	if calib != null:
		calib_hint = "   [C] calibrate: %s" % ("ON" if calib_active else "off")
		if calib_active:
			calib_hint += "   [N/P] step  [A] auto"
	var mtest_v: Variant = body.get("_motor_test_mode")
	var mtest_active: bool = (mtest_v != null and _as_bool(mtest_v))
	var mtest_hint: String = ""
	if mtest_v != null:
		mtest_hint = "   [G] motor-test: %s" % ("ON" if mtest_active else "off")
	# 2026-06-09 — T (hide panels) + H (hide main HUD) hint lines.
	var panels_v: Variant = body.get("_panels_hidden")
	var panels_hint: String = "   [T] panels: %s" % ("hidden" if _as_bool(panels_v) else "ON")
	var hud_v: Variant = body.get("hud_hidden")
	var hud_is_hidden: bool = _as_bool(hud_v)
	var hud_hint: String = "   [H] hud: %s" % ("hidden" if hud_is_hidden else "ON")
	var hint_line: String = "%s%s%s%s%s%s   [`] or [F1] toggle graph   [ESC] quit" % [
		space_hint, ragdoll_hint, calib_hint, mtest_hint, panels_hint, hud_hint]
	if hud_is_hidden:
		# Hidden mode: render ONLY the hint line so the user keeps the
		# keyboard reference but loses the diagnostic text + notifications.
		_label.text = hint_line
	else:
		lines.append(hint_line)
		_label.text = "\n".join(lines)

	# Calibration step display — big label coloured by the active leg.
	if calib_active and calib_label != "":
		_calib_label.text = "CAL %d: %s" % [calib_step, calib_label]
		var col: Color
		if calib_leg < 0 or calib_leg >= _LEG_COLORS.size():
			col = Color(1, 1, 1, 1)   # init phase / no specific leg
		else:
			col = _LEG_COLORS[calib_leg]
		_calib_label.add_theme_color_override("font_color", col)
		_calib_label.visible = true
	else:
		_calib_label.visible = false

func _tilt(b: Basis) -> float:
	var up_local: Vector3 = b.y.normalized()
	return acos(clamp(up_local.dot(Vector3.UP), -1.0, 1.0))

# Phase 6.0.b.10 — pause-overlay end-run hook.  Called by pause_overlay.gd
# when the user clicks "End run & show summary".  Cell hud.gd has the
# same method; pause_overlay.gd:127 finds the HUD via HUD/Overlay path
# and calls request_end_run() on it.
func request_end_run() -> void:
	if _summary_shown:
		return
	_summary_shown = true
	var body: Node = null
	for nm in _BODY_CANDIDATES:
		body = get_tree().get_root().find_child(nm, true, false)
		if body != null:
			break
	if body != null:
		_show_summary(body)

func _show_summary(body: Node) -> void:
	if _summary_root != null:
		return
	_summary_root = Control.new()
	_summary_root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_summary_root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_summary_root)

	var bg := ColorRect.new()
	bg.color = Color(0, 0, 0, 0.7)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_STOP
	_summary_root.add_child(bg)

	var panel := PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_CENTER)
	panel.custom_minimum_size = Vector2(720, 540)
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
	title.text = "Run snapshot — %s" % str(body.name)
	title.add_theme_font_size_override("font_size", 22)
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	v.add_child(title)

	var report := RichTextLabel.new()
	report.bbcode_enabled = false
	report.selection_enabled = true
	report.focus_mode = Control.FOCUS_ALL
	report.size_flags_vertical = Control.SIZE_EXPAND_FILL
	report.add_theme_font_size_override("normal_font_size", 13)
	report.text = build_clipboard_text(body)
	v.add_child(report)

	var hb := HBoxContainer.new()
	hb.alignment = BoxContainer.ALIGNMENT_END
	hb.add_theme_constant_override("separation", 12)
	v.add_child(hb)

	var copy_btn := Button.new()
	copy_btn.text = "📋 Copy log"
	copy_btn.custom_minimum_size = Vector2(140, 36)
	copy_btn.pressed.connect(func():
		DisplayServer.clipboard_set(build_clipboard_text(body))
		copy_btn.text = "✓ Copied"
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
	return_btn.pressed.connect(func():
		ExperimentConfig.launched = false
		get_tree().change_scene_to_file("res://scenes/launcher.tscn")
	)
	hb.add_child(return_btn)

# Clipboard / summary text builder.  Format mirrors what
# scripts/picrawler_run.py aggregates per-seed (chassis y stats,
# pct_upright, longest_up, tipover counts, brain metrics).  Called
# from the summary modal AND from pause_overlay's "📋 Copy log".
func build_clipboard_text(body: Node) -> String:
	var lines: Array[String] = []

	# --- Header ---
	lines.append("=== AMI-Ogma %s run snapshot ===" % str(body.name))
	lines.append("Config:      %s" % str(ExperimentConfig.config_path))
	lines.append("Seed:        %d" % _as_int(ExperimentConfig.seed_value))
	var ticks: int = _as_int(body.get("tick_counter")) if body.get("tick_counter") != null else 0
	var sim_s:  float = float(ticks) * _TAU
	lines.append("Total ticks: %d   (%.1f s sim)" % [ticks, sim_s])
	var ep: int = _as_int(body.get("episode_index")) if body.get("episode_index") != null else 0
	var mc:  int = _as_int(body.get("mc_episode_period")) if body.get("mc_episode_period") != null else 0
	var mode: String = str(body.get("reset_mode")) if body.get("reset_mode") != null else "?"
	lines.append("Reset mode:  %s   mc_episode_period=%d   episodes_fired=%d" % [mode, mc, ep])
	lines.append("")

	# --- Standing duration ---
	var alive_curr: int = _as_int(body.get("episode_alive_ticks")) if body.get("episode_alive_ticks") != null else 0
	var cumul: int      = _as_int(body.get("cumulative_alive_ticks")) if body.get("cumulative_alive_ticks") != null else 0
	var best:  int      = _as_int(body.get("best_cumulative_alive_ticks")) if body.get("best_cumulative_alive_ticks") != null else 0
	lines.append("=== Standing duration ===")
	lines.append("  per-mc-episode (resets every %d ticks):  %s" % [mc, _fmt_secs(float(alive_curr) * _TAU)])
	lines.append("  since last fall:                          %s" % _fmt_secs(float(cumul)  * _TAU))
	lines.append("  best continuous stand (this session):     %s" % _fmt_secs(float(best)   * _TAU))
	lines.append("")

	# --- Stability metrics (sampled by HUD per _process call) ---
	if _hud_sample_ticks > 0:
		var pct_below: float = 100.0 * float(_below_fail_ticks) / float(_hud_sample_ticks)
		var pct_tip:   float = 100.0 * float(_tipover_ticks)    / float(_hud_sample_ticks)
		var pct_up:    float = 100.0 - pct_below
		lines.append("=== Stability (n=%d HUD samples ≈ %.0fs wall) ===" % [
			_hud_sample_ticks, float(_hud_sample_ticks) / 60.0])
		lines.append("  pct_upright:           %.1f%%" % pct_up)
		lines.append("  pct_below_FAIL_HEIGHT: %.1f%%   (chassis_y < %.3f m)" % [pct_below, 0.025])
		lines.append("  pct_tipover:           %.1f%%   (chassis_tilt > π/2)" % pct_tip)
		lines.append("  fall_events:           %d   (FAIL_HEIGHT crossings)" % _n_fall_events)
		lines.append("  tipover_events:        %d   (π/2 tilt crossings)" % _n_tipover_events)
		lines.append("")

	# --- Navigation ---
	if body.get("walk_visit_count") != null:
		lines.append("=== Navigation ===")
		lines.append("  pyramids visited:      %d" % _as_int(body.get("walk_visit_count")))
		lines.append("")

	# --- Final chassis state ---
	if body.get("_chassis") != null:
		var ch: Node = body.get("_chassis")
		if ch is RigidBody3D:
			var pos: Vector3 = (ch as RigidBody3D).global_transform.origin
			var tilt_now: float = _tilt(ch.global_transform.basis)
			lines.append("=== Final chassis state ===")
			lines.append("  position:  (%+.3f, %+.3f, %+.3f) m" % [pos.x, pos.y, pos.z])
			lines.append("  tilt:      %.3f rad   (%.1f°)" % [tilt_now, rad_to_deg(tilt_now)])
			lines.append("")

	# --- Reward-pathway knobs (picrawler-specific) ---
	var stab_g: Variant = body.get("stability_gain")
	if stab_g != null:
		lines.append("=== Reward-pathway knobs (picrawler) ===")
		lines.append("  HIT rate at standing:  %.2f hits/tick" % (float(body.get("HIT_RATE_AT_STANDING")) if body.get("HIT_RATE_AT_STANDING") != null else 0.0))
		lines.append("  stability gain/y_norm/speed:  %.3f / %.2f / %.2f" % [
			float(stab_g),
			float(body.get("stability_y_norm")) if body.get("stability_y_norm") != null else 0.0,
			float(body.get("stability_speed")) if body.get("stability_speed") != null else 0.0,
		])
		var antirot_g: Variant = body.get("antirot_gain")
		if antirot_g != null:
			lines.append("  antirot gain/thresh/scale:  %.3f / %.2f / %.2f" % [
				float(antirot_g),
				float(body.get("antirot_threshold")) if body.get("antirot_threshold") != null else 0.0,
				float(body.get("antirot_scale")) if body.get("antirot_scale") != null else 1.0,
			])
		var energy_g: Variant = body.get("energy_gain")
		if energy_g != null:
			lines.append("  energy gain/deadband/scale:  %.3f / %.2f / %.2f W" % [
				float(energy_g),
				float(body.get("energy_deadband")) if body.get("energy_deadband") != null else 0.0,
				float(body.get("energy_scale")) if body.get("energy_scale") != null else 5.0,
			])
		# Phase 7.5.R+ — live per-source reward attribution.  Shows WHICH
		# channel is producing the DA pulses observed in the neuro module.
		var stand_rate: Variant = body.get("_hit_rate_standing_ema")
		if stand_rate != null:
			var walk_rate: float = float(body.get("_hit_rate_walking_ema")) if body.get("_hit_rate_walking_ema") != null else 0.0
			var gate_rate: float = float(body.get("_hit_rate_gated_ema"))   if body.get("_hit_rate_gated_ema")   != null else 0.0
			var total_rate: float = float(stand_rate) + walk_rate + gate_rate
			lines.append("  reward rate (hits/tick):  total=%.3f  standing=%.3f  walking=%.3f  gated=%.3f" % [
				total_rate, float(stand_rate), walk_rate, gate_rate])
			var stand_cum: float = float(body.get("_hit_cum_standing")) if body.get("_hit_cum_standing") != null else 0.0
			var walk_cum:  float = float(body.get("_hit_cum_walking"))  if body.get("_hit_cum_walking")  != null else 0.0
			var gate_cum:  float = float(body.get("_hit_cum_gated"))    if body.get("_hit_cum_gated")    != null else 0.0
			var sum_cum: float = stand_cum + walk_cum + gate_cum
			if sum_cum > 1e-3:
				lines.append("  reward share (cumulative):  standing=%.0f%%  walking=%.0f%%  gated=%.0f%%" % [
					100.0 * stand_cum / sum_cum,
					100.0 * walk_cum  / sum_cum,
					100.0 * gate_cum  / sum_cum])
		lines.append("")

	# --- Brain final metrics ---
	var brain = body.get("brain")
	if brain != null and brain.has_method("get_module_metrics"):
		lines.append("=== Final brain metrics ===")
		var metrics: Dictionary = brain.get_module_metrics()
		# Sort module ids so the report layout is stable across runs.
		var ids: Array = metrics.keys()
		ids.sort()
		for mod_id in ids:
			var m: Dictionary = metrics[mod_id]
			var t: String = str(m.get("type", ""))
			match t:
				"NeurochemState":
					lines.append("  %-22s da=%.3f  ht=%.3f  r_sig=%.3f" % [
						mod_id, float(m.get("dopamine", 0)),
						float(m.get("serotonin", 0)),
						float(m.get("reward_signal", 0)),
					])
				"EPM":
					lines.append("  %-22s nodes=%d  baked=%d  tle=%.4f" % [
						mod_id, _as_int(m.get("node_count", 0)),
						_as_int(m.get("baked_count", 0)),
						float(m.get("tle", 0)),
					])
				"LateralVoter":
					var trust: Dictionary = m.get("trust_weights", {})
					var trust_str := ""
					for k in trust:
						trust_str += "%s=%.2f " % [k, float(trust[k])]
					lines.append("  %-22s tle=%.4f  active=%s   trust: %s" % [
						mod_id, float(m.get("fused_tle", 0)),
						str(m.get("active_modality", "")),
						trust_str.strip_edges(),
					])
				"DescendingPredictor":
					lines.append("  %-22s loss=%.4f" % [mod_id, float(m.get("loss", 0))])
				"HomeostaticDrive":
					var errs: Dictionary = m.get("errors", {})
					var errs_str := ""
					for k in errs:
						errs_str += "%s=%.3f " % [k, float(errs[k])]
					lines.append("  %-22s urg=%.3f   %s" % [
						mod_id, float(m.get("urgency", 0)), errs_str.strip_edges(),
					])
				"Premotor":
					lines.append("  %-22s W_norm=%.3f  H=%.2f  last_chosen=%d" % [
						mod_id, float(m.get("W_total_norm", 0)),
						float(m.get("last_entropy", 0)),
						_as_int(m.get("last_chosen_intent", -1)),
					])
				_:
					lines.append("  %-22s %s" % [mod_id, t])
	return "\n".join(lines)

func _fmt_secs(s: float) -> String:
	if s < 1.0:
		return "%d ms" % _as_int(round(s * 1000.0))
	if s < 60.0:
		return "%.2f s" % s
	var m: int = _as_int(s) / 60
	var r: float = s - float(m * 60)
	return "%dm %05.2fs" % [m, r]
