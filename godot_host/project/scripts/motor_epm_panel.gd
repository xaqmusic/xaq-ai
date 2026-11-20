extends Control
## Live tuning panel for the sim: the LoadPhaseGait stack's gait, servo, height and learner controls, and (when the config
## carries a PoseSequencer) the learned stepper's authority.
##
## Unlike reward_panel (which writes body @export fields), this writes BRAIN
## module params via brain.apply_patch({"op":"set_param","id":"motor_epm",...}).
## Every MotorEPM knob is HotMutable, so changes take effect on the next tick.
##
## Use case: the Motor-EPM controller is reward-free; its behaviour (gain,
## exploration, posture, coordination) lives entirely in these knobs, and the
## right regime is read by EYE (smoothness / aliveness / leg strength), not by
## headless stats.  Drag a slider, watch the body, repeat.  Top-RIGHT so it
## does not overlap the HUD (which lives top-left).  T-hotkey hideable like the
## other HUD panels (mouse_filter PASS).
##
## Methodology: SCENE-only (headless runs never instance it).  No body-side
## state — pure remote control of the existing MotorEPM HotMutable params.

const _BODY_CANDIDATES: Array = ["Picrawler", "Quadruped"]
const _MODULE_ID: String = "motor_epm"

# Slider records: {key, label, min, max, step} plus where the value lives: "id" = a brain module (default
# motor_epm), or "body": true = a field on the PicrawlerBody node (read every tick there).  {"section": text}
# draws a heading.  REBUILT 2026-10-24 (operator): the old 45 sliders drove the pre-LPG gait (Cruse coordination,
# the height homeostat, agency, rear landing ...), none of it in the current stack.  These are the controls of
# the LoadPhaseGait stack, for walking-speed and posture experiments.  A key the loaded config does not have
# shows "—" and does nothing.  The old list is in git history (motor_epm_panel.gd before this change).
# "only_if": <module id> on a section or a slider hides it when the loaded config has no such module (2026-11-17,
# operator: the stepper's authority beside the LPG controls) -- the stepper rows show only on a sequencer config.
const _SLIDERS: Array = [
	{"section": "HANDOVER  (walker <-> stepper; [L] flips it now)", "only_if": "pose_sequencer"},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "engage", "label": "legs: 0 walker, 1 stepper", "tip": "who owns the legs (0 = the LPG walker, 1 = the stepper); [L] flips it", "min": 0.0, "max": 1.0, "step": 1.0},
	{"body": true, "only_if": "pose_sequencer", "key": "handover_every_s", "label": "alternate every (s; 0 = off)", "min": 0.0, "max": 120.0, "step": 5.0},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "engage_ramp_ticks", "label": "crossfade (ticks; 50 = 1 s)", "min": 10.0, "max": 300.0, "step": 10.0},
	{"only_if": "pose_sequencer", "key": "homeo_pose_hold", "label": "hold walker's gain (0/1)", "tip": "homeo_pose_hold: the walker's amplitude gain is held while the stepper owns the legs (0/1)", "min": 0.0, "max": 1.0, "step": 1.0},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "handback_stand", "label": "stand before handback (0/1/2)", "tip": "handback_stand (2026-11-19): before the walker takes the legs back the stepper lands its toe and stands -- 0 off (the blend starts from wherever the crawl is), 1 on the stepper's neutral toes, 2 in the walker's own mean pose (learned while the walker drives)", "min": 0.0, "max": 2.0, "step": 1.0},
	{"id": "pose_sequencer", "only_if": "walker_outage", "key": "engage_follow", "label": "automatic selection (0/1)", "tip": "engage_follow (2026-11-20): 1 = the selection owns the legs -- the walker by default, the stepper once the walker has browned out twice inside a minute, the walker tried again after the body has travelled; the row above and [L] are then overridden every tick. 0 = the selection only reports (shadow)", "min": 0.0, "max": 1.0, "step": 1.0},
	{"section": "STEPPER  (PoseSequencer)", "only_if": "pose_sequencer"},
	{"meta": "tempo", "only_if": "pose_sequencer", "key": "tempo", "label": "stepper tempo (x config)", "tip": "the stepper's shift, lift, swing and lowering rates together, as a multiple of the config's own (1 = the config; the 2026-11-18 speed arms were 1.5, 2 and 3)", "min": 0.5, "max": 3.0, "step": 0.1},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "lo_gain", "label": "offset authority (0 = scaffold)", "min": 0.0, "max": 1.0, "step": 0.05},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "ik_knee_guard", "label": "knee guard (0/1)", "tip": "knee guard: 1 = the IK never takes a knee past straight", "min": 0.0, "max": 1.0, "step": 1.0},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "plan_model_margin", "label": "plan predicts lift gate (0/1)", "tip": "the plan predicts its own lift gate (1 = on)", "min": 0.0, "max": 1.0, "step": 1.0},
	{"id": "pose_sequencer", "only_if": "pose_sequencer", "key": "crouch_max_m", "label": "crouch to reach (m; 0 off)", "tip": "the body crouches toward a toe that cannot reach its ground (m; 0 = off)", "min": 0.0, "max": 0.05, "step": 0.005},
	{"section": "PACE  (meta: period + stroke together)"},
	{"meta": "pace", "key": "pace", "label": "pace (0 slow .. 1 fast)", "min": 0.0, "max": 1.0, "step": 0.05},
	{"section": "GAIT  (LoadPhaseGait)"},
	{"id": "load_phase", "key": "period_ticks", "label": "LPG period (ticks / stride)", "min": 20.0, "max": 120.0, "step": 1.0},
	{"id": "load_phase", "key": "sigma_ratio",  "label": "LPG load hold (sigma ratio)",  "min": 0.0,  "max": 4.0,   "step": 0.1},
	{"section": "SERVOS  (sim body)"},
	{"body": true, "key": "MAX_SERVO_SPEED", "label": "servo speed (rad/s; robot 4.22)", "min": 1.5, "max": 8.0, "step": 0.1},
	{"body": true, "key": "leg_strength",    "label": "servo strength (x torque cap)",   "min": 0.1, "max": 1.5, "step": 0.05},
	{"section": "STRIDE"},
	{"key": "stroke_gain",         "label": "stroke (forward push)",      "min": 0.0,  "max": 2.0, "step": 0.05},
	{"key": "leg_phase_lift_hip2", "label": "swing lift, femur (- = up)", "min": -0.9, "max": 0.0, "step": 0.05},
	{"key": "leg_phase_lift_knee", "label": "swing lift, knee (+ = fold)", "min": 0.0, "max": 0.6, "step": 0.05},
	{"key": "stance_lift_gain",    "label": "stance press (fixed)",       "min": 0.0,  "max": 1.5, "step": 0.05},
	{"section": "HEIGHT  (belly ToF learners)"},
	{"key": "press_target_m",      "label": "press target (m)",           "min": 0.03, "max": 0.10, "step": 0.005},
	{"key": "press_learn_rate",    "label": "press learner rate (0 off)", "min": 0.0,  "max": 0.05, "step": 0.005},
	{"key": "press_phase",         "label": "press over phase (0/1)",     "min": 0.0,  "max": 1.0,  "step": 1.0},
	{"key": "press_kin",           "label": "press follows kinematics", "tip": "the press follows the leg's kinematics (0/1)", "min": 0.0, "max": 1.0, "step": 1.0},
	{"key": "tuck_learn_target_m", "label": "femur target (m)",           "min": 0.03, "max": 0.10, "step": 0.005},
	{"key": "tuck_learn_rate",     "label": "femur learner rate (0 off)", "min": 0.0,  "max": 0.05, "step": 0.002},
	{"section": "LEARNERS / LEVELLING"},
	{"key": "swing_learn_rate",    "label": "swing-lift learner (0 off)", "min": 0.0,  "max": 0.03, "step": 0.005},
	{"key": "level_int_gain",      "label": "corner integrator",          "min": 0.0,  "max": 0.02, "step": 0.001},
	{"key": "stroke_level_gain",   "label": "level gate on the stroke",   "min": 0.0,  "max": 8.0,  "step": 0.5},
	{"section": "SEARCH / RECOVERY"},
	{"key": "conf_gain",           "label": "confidence handover",        "min": 0.0,  "max": 1.0,  "step": 0.1},
	{"key": "conf_noise",          "label": "search noise",               "min": 0.0,  "max": 0.5,  "step": 0.05},
	{"key": "panic_push_orient",   "label": "panic push to gravity",      "min": 0.0,  "max": 1.0,  "step": 0.1},
	{"key": "motor_gain",          "label": "learned layer gain",         "min": 0.0,  "max": 5.0,  "step": 0.1},
	{"key": "explore_noise",       "label": "explore noise",              "min": 0.0,  "max": 0.5,  "step": 0.01},
]

var _body: Node = null
# PACE meta-control (operator, 2026-10-31): one slider that moves the LPG period over the span the 2026-10-26 speed
# map swept (90 -> 30 ticks over pace 0 -> 1) and the stroke with it, as a fraction of the config's own stroke, so
# the config's point is exactly reproduced at its own pace.  Until a pace error learns this, it is how a faster or
# slower gait is tried live.  The same mapping is the dash's o/p keys (pi_host/tools/dash_run.py, pace_targets).
const _PACE_PERIOD_SLOW: float = 90.0
const _PACE_PERIOD_FAST: float = 30.0
const _PACE_STROKE_SLOPE: float = 0.5
const _PACE_STROKE_MIN: float = 0.4
const _PACE_STROKE_MAX: float = 1.2
var _pace_period_cfg: float = -1.0    # the config's period and stroke, captured at the first sync
var _pace_stroke_cfg: float = -1.0
# STEPPER TEMPO meta-control (operator, 2026-11-18: "I have not yet seen how the stepper performs at faster rates"): one
# slider over the sequencer's five rates, each a multiple of the config's own value, captured at the first sync.  1.5 / 2 / 3
# are the speed arms of the ledger's 2026-11-18 dose-response (zz_arm_t15 / t20 / t30).  A handle on existing parameters, not
# a lever: nothing in the brain reads it.
const _TEMPO_KEYS: Array = ["shift_v_mps", "lift_v_mps", "swing_v_mps", "lower_v_mps", "lower_fast_v_mps"]
var _tempo_cfg: Dictionary = {}       # rate key -> the config's value
# Row layout (2026-11-18, operator: a long label took the whole row and left its slider no width).  The label has a fixed
# column and is clipped to it (its full text, or the row's "tip", is the tooltip); the slider keeps a minimum width.
const _LABEL_W: float = 168.0
const _SLIDER_MIN_W: float = 100.0
var _content_vb: VBoxContainer = null
var _status: Label = null
var _minimise_btn: Button = null
var _value_labels: Dictionary = {}    # "where/key" -> Label (where = module id, or "body")
var _optional: Array = []             # [module id, Control]: rows and headings shown only when the config has that module
var _synced: bool = false             # initial pull landed (brain ready)
# Minimise/expand: collapse to the header row.  The panel
# is anchored bottom=1.0, so we must flip anchor_bottom to 0.0 and clamp
# offset_bottom to a one-line height (else the empty frame keeps full height).
var _is_minimised: bool = false
var _full_anchor_bottom: float = -1.0
var _full_offset_bottom: float = 0.0
const _COLLAPSED_HEIGHT: float = 38.0   # header row + frame padding

func _ready() -> void:
	# Top-RIGHT anchor — the HUD lives top-left, so anchor to the right edge
	# to avoid overlapping it (the reward/trainer/curriculum panels that used
	# to own top-right are gone).  Panel is 360 wide with a 12px margin.
	anchor_left = 1.0
	anchor_top = 0.0
	anchor_right = 1.0
	anchor_bottom = 1.0
	offset_left = -12.0 - 360.0
	offset_top = 12.0
	offset_right = -12.0
	offset_bottom = -12.0
	mouse_filter = Control.MOUSE_FILTER_PASS
	# 2026-08-12 (operator QoL) — boots HIDDEN: the panel is a tuning bench, not
	# a monitor, and even collapsed its full-rect Control shadows the top-right
	# corner.  [M] unhides it (picrawler_body KEY_M handler).
	visible = false

	var root := PanelContainer.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var bg := StyleBoxFlat.new()
	bg.bg_color = Color(0.05, 0.06, 0.05, 0.82)
	bg.border_color = Color(0.5, 0.65, 0.5, 0.85)
	bg.set_border_width_all(1)
	bg.set_corner_radius_all(4)
	bg.content_margin_left = 10
	bg.content_margin_right = 10
	bg.content_margin_top = 8
	bg.content_margin_bottom = 8
	root.add_theme_stylebox_override("panel", bg)
	add_child(root)

	var outer := VBoxContainer.new()
	outer.add_theme_constant_override("separation", 4)
	root.add_child(outer)

	var header := HBoxContainer.new()
	header.add_theme_constant_override("separation", 4)
	outer.add_child(header)
	var title := Label.new()
	title.text = "SIM TUNING  (stepper, LPG gait, servos, height)"
	title.add_theme_font_size_override("font_size", 13)
	title.add_theme_color_override("font_color", Color(0.9, 1.0, 0.9, 1.0))
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(title)
	_minimise_btn = Button.new()
	_minimise_btn.text = "▼"
	_minimise_btn.custom_minimum_size = Vector2(28, 0)
	_minimise_btn.pressed.connect(_on_minimise)
	header.add_child(_minimise_btn)

	_content_vb = VBoxContainer.new()
	_content_vb.add_theme_constant_override("separation", 4)
	_content_vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	outer.add_child(_content_vb)

	for spec in _SLIDERS:
		if spec.has("section"):
			var sec := Label.new()
			sec.text = str(spec["section"])
			sec.add_theme_font_size_override("font_size", 11)
			sec.add_theme_color_override("font_color", Color(0.75, 0.9, 1.0, 1.0))
			_content_vb.add_child(sec)
			if spec.has("only_if"):
				_optional.append([str(spec["only_if"]), sec])
		else:
			_build_slider_row(_content_vb, spec)

	_status = Label.new()
	_status.add_theme_font_size_override("font_size", 10)
	_status.add_theme_color_override("font_color", Color(0.7, 0.8, 0.7, 1.0))
	_status.text = "waiting for body…"
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_content_vb.add_child(_status)

func _build_slider_row(parent: VBoxContainer, spec: Dictionary) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 6)
	parent.add_child(row)

	var name_label := Label.new()
	name_label.text = str(spec["label"])
	name_label.add_theme_font_size_override("font_size", 11)
	name_label.custom_minimum_size = Vector2(_LABEL_W, 0)
	name_label.clip_text = true                      # with the column fixed, no label can take the slider's width
	name_label.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	name_label.tooltip_text = str(spec.get("tip", spec["label"]))
	name_label.mouse_filter = Control.MOUSE_FILTER_PASS   # a Label ignores the mouse by default: no tooltip
	row.add_child(name_label)

	var slider := HSlider.new()
	slider.min_value = float(spec["min"])
	slider.max_value = float(spec["max"])
	slider.step = float(spec["step"])
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.custom_minimum_size = Vector2(_SLIDER_MIN_W, 0)
	slider.tooltip_text = name_label.tooltip_text
	var key := str(spec["key"])
	var where := "body" if bool(spec.get("body", false)) else str(spec.get("id", _MODULE_ID))
	if spec.has("meta"):
		where = "meta"
	slider.value_changed.connect(func(v: float) -> void: _on_slider(where, key, v))
	row.add_child(slider)

	var value_label := Label.new()
	value_label.add_theme_font_size_override("font_size", 10)
	value_label.custom_minimum_size = Vector2(46, 0)
	value_label.text = "—"
	row.add_child(value_label)
	_value_labels[where + "/" + key] = value_label
	if spec.has("only_if"):
		_optional.append([str(spec["only_if"]), row])
		value_label.set_meta("only_if", str(spec["only_if"]))
	# stash the slider so the initial pull can set it without re-emitting apply
	value_label.set_meta("slider", slider)
	value_label.set_meta("name", name_label)

func _process(_dt: float) -> void:
	# Find the body once, then keep retrying the initial value-pull until it
	# lands — the brain is often not initialized the instant the body node
	# appears, so a one-shot sync would silently get an empty schema.
	if _body == null or not is_instance_valid(_body):
		for cand in _BODY_CANDIDATES:
			var b := get_tree().root.find_child(cand, true, false)
			if b != null:
				_body = b
				break
	if _body != null and not _synced:
		_synced = _sync_from_module()
	# The handover row follows the body: [L] and the timer flip `engage` outside this panel.
	if _synced and _body != null and _value_labels.has("pose_sequencer/engage"):
		var he = _body.get("handover_engage")
		if he != null and float(he) >= 0.0:
			var hl: Label = _value_labels["pose_sequencer/engage"]
			var hs: HSlider = hl.get_meta("slider")
			if not is_equal_approx(hs.value, float(he)):
				hs.set_value_no_signal(float(he))
				hl.text = "%.3f" % float(he)

func _brain() -> Object:
	if _body == null or not is_instance_valid(_body):
		return null
	var br = _body.get("brain")
	return br

func _sync_from_module() -> bool:
	# Pull the current values so the sliders start where the config left them, without firing apply.  Returns
	# false (keep retrying) until the brain is initialized and answers for motor_epm.
	var br = _brain()
	if br == null or not br.has_method("get_module_param_schema"):
		return false
	var cur: Dictionary = {}      # "where/key" -> value
	var ok_main := false
	var present: Dictionary = {}   # module ids the loaded config answers for
	var ids: Dictionary = {}
	for spec in _SLIDERS:
		if spec.has("key") and not bool(spec.get("body", false)):
			ids[str(spec.get("id", _MODULE_ID))] = true
		if spec.has("only_if"):
			ids[str(spec["only_if"])] = true   # a row may hang on a module other than the one it sets (2026-11-20: engage_follow)
	for mid in ids.keys():
		var schema = br.get_module_param_schema(mid)
		if typeof(schema) != TYPE_ARRAY or schema.is_empty():
			continue
		present[mid] = true
		if mid == _MODULE_ID:
			ok_main = true
		for entry in schema:
			if typeof(entry) == TYPE_DICTIONARY and entry.has("key"):
				cur[mid + "/" + str(entry["key"])] = entry.get("current_value", entry.get("default_value", null))
	if not ok_main:
		return false   # brain not initialized yet — retry next frame
	for spec in _SLIDERS:
		if spec.has("key") and bool(spec.get("body", false)):
			var bv = _body.get(str(spec["key"]))
			if bv != null:
				cur["body/" + str(spec["key"])] = bv
	# The pace slider starts at the pace of the config's own period; it needs both handles present.
	if cur.has("load_phase/period_ticks") and cur.has(_MODULE_ID + "/stroke_gain") \
			and cur["load_phase/period_ticks"] != null and cur[_MODULE_ID + "/stroke_gain"] != null:
		_pace_period_cfg = float(cur["load_phase/period_ticks"])
		_pace_stroke_cfg = float(cur[_MODULE_ID + "/stroke_gain"])
		cur["meta/pace"] = _pace_from_period(_pace_period_cfg)
	# The stepper's tempo starts at 1: the config's own rates, whatever they are.
	_tempo_cfg.clear()
	for tk in _TEMPO_KEYS:
		if cur.has("pose_sequencer/" + tk) and cur["pose_sequencer/" + tk] != null:
			_tempo_cfg[tk] = float(cur["pose_sequencer/" + tk])
	if _tempo_cfg.size() == _TEMPO_KEYS.size():
		cur["meta/tempo"] = 1.0
	for opt in _optional:
		(opt[1] as Control).visible = present.has(opt[0])
	var missing := 0
	var hidden := 0
	for k in _value_labels.keys():
		var lbl: Label = _value_labels[k]
		var sld: HSlider = lbl.get_meta("slider")
		if lbl.has_meta("only_if") and not present.has(str(lbl.get_meta("only_if"))):
			hidden += 1   # a row for a module this config does not have: not shown, not counted as missing
			continue
		if cur.has(k) and cur[k] != null and typeof(cur[k]) in [TYPE_FLOAT, TYPE_INT, TYPE_BOOL]:
			var v := float(cur[k])
			sld.set_value_no_signal(v)
			lbl.text = "%.3f" % v
		else:
			sld.editable = false
			lbl.text = "—"
			missing += 1
	_status.text = "live — drag to retune" + (("  (%d controls not in this config)" % missing) if missing > 0 else "")
	print("SimTuningPanel: synced %d controls (%d not in this config)" % [_value_labels.size() - hidden - missing, missing])
	# OGMA_PANEL_SELFTEST=1: turn one gait knob, one body knob and (on a sequencer config) the stepper's authority through
	# the real handler and read them back (a headless run cannot drag a slider).  The read-back waits a few physics frames:
	# a patch lands at the brain's next tick, and a read in the same frame returns the value before it (2026-11-17: the
	# first form of this test printed the config's own 30 after setting 41).
	if OS.get_environment("OGMA_PANEL_SELFTEST") == "1":
		_on_slider("load_phase", "period_ticks", 41.0)
		_on_slider("body", "MAX_SERVO_SPEED", 5.5)
		if present.has("pose_sequencer"):
			_on_slider("pose_sequencer", "lo_gain", 0.35)
			_on_slider("pose_sequencer", "engage_ramp_ticks", 150.0)   # the handover rows (2026-11-18)
			_on_slider(_MODULE_ID, "homeo_pose_hold", 1.0)
			_on_slider("body", "handover_every_s", 20.0)
			_on_slider("meta", "tempo", 2.0)
			_on_slider("pose_sequencer", "handback_stand", 2.0)   # the stand before the hand-back (2026-11-19)
		_selftest_readback(present.has("pose_sequencer"))
	return true

func _schema_value(mid: String, key: String):
	var br = _brain()
	if br == null:
		return null
	for e in br.get_module_param_schema(mid):
		if typeof(e) == TYPE_DICTIONARY and str(e.get("key", "")) == key:
			return e.get("current_value", null)
	return null

func _selftest_readback(has_stepper: bool) -> void:
	for _i in range(10):
		await get_tree().physics_frame
	print("SimTuningPanel selftest: load_phase.period_ticks -> %s (set 41), body MAX_SERVO_SPEED -> %s (set 5.5)" % [str(_schema_value("load_phase", "period_ticks")), str(_body.get("MAX_SERVO_SPEED"))])
	var row_visible := str(_value_labels["pose_sequencer/lo_gain"].get_parent().visible)
	if has_stepper:
		print("SimTuningPanel selftest: pose_sequencer.lo_gain -> %s (set 0.35; row visible %s)" % [str(_schema_value("pose_sequencer", "lo_gain")), row_visible])
		print("SimTuningPanel selftest: handover: engage_ramp_ticks -> %s (set 150), homeo_pose_hold -> %s (set 1), body handover_every_s -> %s (set 20), engage row %s" % [
			str(_schema_value("pose_sequencer", "engage_ramp_ticks")), str(_schema_value(_MODULE_ID, "homeo_pose_hold")), str(_body.get("handover_every_s")),
			("present, at %s" % str((_value_labels["pose_sequencer/engage"].get_meta("slider") as HSlider).value)) if _value_labels.has("pose_sequencer/engage") else "MISSING"])
		var rates: Array = []
		for tk in _TEMPO_KEYS:
			rates.append("%s %s (config %s)" % [tk, str(_schema_value("pose_sequencer", tk)), str(_tempo_cfg.get(tk, "?"))])
		print("SimTuningPanel selftest: stepper tempo set 2.0 -> %s" % ", ".join(rates))
		print("SimTuningPanel selftest: handback_stand -> %s (set 2; row %s)" % [str(_schema_value("pose_sequencer", "handback_stand")), "present" if _value_labels.has("pose_sequencer/handback_stand") else "MISSING"])
	else:
		print("SimTuningPanel selftest: no pose_sequencer in this config (stepper rows visible %s)" % row_visible)
	# The layout, as it is drawn: every shown row's slider width, and the labels the column clips (a headless run cannot look).
	visible = true
	for _i in range(3):
		await get_tree().process_frame
	var narrowest := 1.0e9
	var narrow_key := ""
	var clipped: Array = []
	var rows := 0
	for k in _value_labels.keys():
		var vl: Label = _value_labels[k]
		if not (vl.get_parent() as Control).visible:
			continue
		rows += 1
		var sl: HSlider = vl.get_meta("slider")
		if sl.size.x < narrowest:
			narrowest = sl.size.x
			narrow_key = k
		var nl: Label = vl.get_meta("name")
		var tw: float = nl.get_theme_font("font").get_string_size(nl.text, HORIZONTAL_ALIGNMENT_LEFT, -1, nl.get_theme_font_size("font_size")).x
		if tw > nl.size.x:
			clipped.append("%s (%.0f px in %.0f)" % [nl.text, tw, nl.size.x])
	print("SimTuningPanel selftest: layout: %d rows shown, narrowest slider %.0f px (%s), panel content %.0f px wide, labels clipped: %s" % [
		rows, narrowest, narrow_key, _content_vb.size.x, ("none" if clipped.is_empty() else "; ".join(clipped))])
	visible = false

func _pace_from_period(period: float) -> float:
	return clampf((_PACE_PERIOD_SLOW - period) / (_PACE_PERIOD_SLOW - _PACE_PERIOD_FAST), 0.0, 1.0)

func _on_pace(p: float) -> void:
	# One meta-control, two handles: the LPG period and the stroke (see the constants above).
	if _pace_period_cfg < 0.0 or _pace_stroke_cfg < 0.0:
		if _status != null:
			_status.text = "pace: this config has no LPG period or stroke_gain"
		return
	p = clampf(p, 0.0, 1.0)
	var period := roundf(_PACE_PERIOD_SLOW - (_PACE_PERIOD_SLOW - _PACE_PERIOD_FAST) * p)
	var stroke := clampf(_pace_stroke_cfg * (1.0 + _PACE_STROKE_SLOPE * (p - _pace_from_period(_pace_period_cfg))),
		_PACE_STROKE_MIN, _PACE_STROKE_MAX)
	_on_slider("load_phase", "period_ticks", period)
	_on_slider(_MODULE_ID, "stroke_gain", stroke)
	for lk in ["load_phase/period_ticks", _MODULE_ID + "/stroke_gain"]:
		if _value_labels.has(lk):
			var sld: HSlider = _value_labels[lk].get_meta("slider")
			sld.set_value_no_signal(period if lk.ends_with("period_ticks") else stroke)
	if _value_labels.has("meta/pace"):
		_value_labels["meta/pace"].text = "%.2f" % p
	if _status != null:
		_status.text = "pace %.2f -> period %.0f ticks (%.2f s/stride), stroke %.2f" % [p, period, period / 50.0, stroke]

func _on_tempo(x: float) -> void:
	# One meta-control, five handles: each of the stepper's rates at x times the config's own (a rate the config has at 0,
	# such as an unused fast lowering, stays 0).
	if _tempo_cfg.size() != _TEMPO_KEYS.size():
		if _status != null:
			_status.text = "tempo: this config has no stepper"
		return
	x = clampf(x, 0.5, 3.0)
	for tk in _TEMPO_KEYS:
		_on_slider("pose_sequencer", tk, float(_tempo_cfg[tk]) * x)
	if _value_labels.has("meta/tempo"):
		_value_labels["meta/tempo"].text = "%.2f" % x
	if _status != null:
		_status.text = "stepper tempo x %.2f -> shift %.3f, lift %.3f, swing %.3f, lower %.3f m/s" % [x, float(_tempo_cfg["shift_v_mps"]) * x,
			float(_tempo_cfg["lift_v_mps"]) * x, float(_tempo_cfg["swing_v_mps"]) * x, float(_tempo_cfg["lower_v_mps"]) * x]

func _on_slider(where: String, key: String, v: float) -> void:
	var ok := false
	var err := ""
	if where == "meta":
		if key == "tempo":
			_on_tempo(v)
		else:
			_on_pace(v)
		return
	if where == "body":
		if _body != null and is_instance_valid(_body):
			_body.set(key, v)
			ok = is_equal_approx(float(_body.get(key)), v)
	else:
		var br = _brain()
		if br == null or not br.has_method("apply_patch"):
			return
		var res = br.apply_patch({"op": "set_param", "id": where, "key": key, "value": v})
		ok = typeof(res) == TYPE_DICTIONARY and bool(res.get("success", false))
		if not ok and typeof(res) == TYPE_DICTIONARY:
			err = str(res.get("error", ""))
	if where == "pose_sequencer" and key == "engage" and ok and _body != null and is_instance_valid(_body):
		_body.set("handover_engage", v)   # the body's read-back, so the row does not snap back before its next read
	var lk := where + "/" + key
	if _value_labels.has(lk):
		_value_labels[lk].text = "%.3f" % v
	if _status != null:
		_status.text = "%s.%s = %.3f%s" % [where, key, v, ("" if ok else "  (apply failed " + err + ")")]

func _on_minimise() -> void:
	# Hide the content AND shrink the panel rect to the header row, so the
	# collapsed panel is one line and frees the surrounding HUD area.
	if _content_vb == null or _minimise_btn == null:
		return
	_is_minimised = not _is_minimised
	_content_vb.visible = not _is_minimised
	_minimise_btn.text = "▲" if _is_minimised else "▼"
	if _is_minimised:
		if _full_anchor_bottom < 0:
			_full_anchor_bottom = anchor_bottom
			_full_offset_bottom = offset_bottom
		anchor_bottom = 0.0
		offset_bottom = offset_top + _COLLAPSED_HEIGHT
	else:
		if _full_anchor_bottom >= 0:
			anchor_bottom = _full_anchor_bottom
			offset_bottom = _full_offset_bottom
