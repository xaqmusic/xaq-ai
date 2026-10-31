extends Control
## Live tuning panel for the sim: the LoadPhaseGait stack's gait, servo, height and learner controls.
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
const _SLIDERS: Array = [
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
	{"key": "press_kin",           "label": "press follows leg kinematics (0/1)", "min": 0.0, "max": 1.0, "step": 1.0},
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
var _content_vb: VBoxContainer = null
var _status: Label = null
var _minimise_btn: Button = null
var _value_labels: Dictionary = {}    # "where/key" -> Label (where = module id, or "body")
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
	title.text = "SIM TUNING  (LPG stack: gait, servos, height)"
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
	name_label.custom_minimum_size = Vector2(150, 0)
	row.add_child(name_label)

	var slider := HSlider.new()
	slider.min_value = float(spec["min"])
	slider.max_value = float(spec["max"])
	slider.step = float(spec["step"])
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
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
	# stash the slider so the initial pull can set it without re-emitting apply
	value_label.set_meta("slider", slider)

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
	var ids: Dictionary = {}
	for spec in _SLIDERS:
		if spec.has("key") and not bool(spec.get("body", false)):
			ids[str(spec.get("id", _MODULE_ID))] = true
	for mid in ids.keys():
		var schema = br.get_module_param_schema(mid)
		if typeof(schema) != TYPE_ARRAY or schema.is_empty():
			continue
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
	var missing := 0
	for k in _value_labels.keys():
		var lbl: Label = _value_labels[k]
		var sld: HSlider = lbl.get_meta("slider")
		if cur.has(k) and cur[k] != null and typeof(cur[k]) in [TYPE_FLOAT, TYPE_INT, TYPE_BOOL]:
			var v := float(cur[k])
			sld.set_value_no_signal(v)
			lbl.text = "%.3f" % v
		else:
			sld.editable = false
			lbl.text = "—"
			missing += 1
	_status.text = "live — drag to retune" + (("  (%d controls not in this config)" % missing) if missing > 0 else "")
	print("SimTuningPanel: synced %d controls (%d not in this config)" % [_value_labels.size() - missing, missing])
	# OGMA_PANEL_SELFTEST=1: turn one gait knob and one body knob through the real handler and read them back
	# (a headless run cannot drag a slider).
	if OS.get_environment("OGMA_PANEL_SELFTEST") == "1":
		_on_slider("load_phase", "period_ticks", 41.0)
		_on_slider("body", "MAX_SERVO_SPEED", 5.5)
		var sch = br.get_module_param_schema("load_phase")
		var got = null
		for e in sch:
			if typeof(e) == TYPE_DICTIONARY and str(e.get("key", "")) == "period_ticks":
				got = e.get("current_value", null)
		print("SimTuningPanel selftest: load_phase.period_ticks -> %s, body MAX_SERVO_SPEED -> %s" % [str(got), str(_body.get("MAX_SERVO_SPEED"))])
	return true

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

func _on_slider(where: String, key: String, v: float) -> void:
	var ok := false
	var err := ""
	if where == "meta":
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
