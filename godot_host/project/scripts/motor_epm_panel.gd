extends Control
## Live tuning panel for the MotorEPM homeokinetic controller.
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

# Slider records: {key, label, min, max, step}.  key must be a HotMutable param
# on MotorEPM.  Trimmed 2026-07-18 from 33 -> 15 (dropped panic/escape, deep
# learning-rate internals, and rarely-by-eye fine knobs).  Grouped by function.
const _SLIDERS: Array = [
	# primary "by eye" levers (legs weak -> motor_gain; jerky -> ctrl_lr / noise)
	{"key": "motor_gain",        "label": "motor_gain  (leg strength)", "min": 0.0,  "max": 5.0,  "step": 0.1},
	# 2026-08-02 — ceiling raised 0.10 -> 0.30.  The old max sat exactly at the BOTTOM of
	# the range that works, so this lever could not be found by eye.  Measured on the
	# pure-HK base at 20k, n=4: steps 41 -> 141 -> 163 -> 226 and net_disp 0.56 -> 0.68
	# -> 1.79 -> 2.17 across ctrl_lr 0.01 / 0.05 / 0.10 / 0.20, with falls rising
	# 1.00 -> 1.75.  Playful Machine reference: hexapod epsC=0.1, dog epsC=0.05 -- we had
	# been running 0.01, i.e. 5-10x below every legged experiment of theirs.
	# NOTE the activity PEAKS around 14-20k ticks and then decays, so judge this slider
	# over minutes, not seconds.
	{"key": "ctrl_lr",           "label": "ctrl_lr  (HK drive; PM=0.05-0.1)", "min": 0.0, "max": 0.30, "step": 0.01},
	# The model rate is the other half of the pair and has never been on the panel.
	# PM: hexapod epsA=0.05, dog epsA=0.01.  Ours defaults to 0.02.  A model that learns
	# too slowly makes the loop Jacobian stale, which is what the HK gradient descends.
	{"key": "model_lr",          "label": "model_lr  (self-model; PM=0.01-0.05)", "min": 0.0, "max": 0.20, "step": 0.005},
	{"key": "explore_noise",     "label": "explore_noise  (motion)",    "min": 0.0,  "max": 0.50, "step": 0.01},
	# gait oscillation
	{"key": "amp_target",        "label": "amp_target  (osc size)",     "min": 0.0,  "max": 1.5,  "step": 0.05},
	{"key": "coupling_gain",     "label": "coupling_gain  (Rung3 sync)","min": 0.0,  "max": 2.0,  "step": 0.05},
	{"key": "stroke_gain",       "label": "stroke_gain  (fwd thrust)",  "min": 0.0,  "max": 2.0,  "step": 0.05},
	# balance / posture
	{"key": "balance_gain",      "label": "balance_gain  (vestibular)", "min": -3.0, "max": 3.0,  "step": 0.05},
	{"key": "height_homeo_gain", "label": "height_homeo  (stand tall)", "min": 0.0,  "max": 0.1,  "step": 0.005},
	{"key": "postural_gain",     "label": "postural_gain  (damp/hold)", "min": 0.0,  "max": 2.0,  "step": 0.05},
	# steering
	{"key": "heading_gain",      "label": "heading_gain (go-straight)", "min": -3.0, "max": 3.0,  "step": 0.05},
	{"key": "heading_hold_gain", "label": "heading_hold (yaw-rate)",    "min": -3.0, "max": 3.0,  "step": 0.05},
	{"key": "heading_bearing_hold_gain", "label": "heading→spawn (go-straight, POS)", "min": 0.0, "max": 12.0, "step": 0.5},
	{"key": "stuck_explore_gain", "label": "stuck→explore (propulsion)", "min": 0.0, "max": 5.0,  "step": 0.25},
	{"key": "progress_commit_gain", "label": "progress→commit (drive)", "min": 0.0, "max": 3.0,  "step": 0.25},
	{"key": "forward_flow_gain",  "label": "fwd-flow (homeokinetic)",   "min": 0.0, "max": 2.0,  "step": 0.1},
	{"key": "propulsion_balance_gain", "label": "prop_bal (leg catch-up)", "min": 0.0, "max": 2.0, "step": 0.05},
	# coordination / agency (reward-free homeokinetic drives, not RL)
	{"key": "cruse_gain",        "label": "cruse_gain  (leg coord)",    "min": 0.0,  "max": 0.50, "step": 0.02},
	{"key": "cruse_rule3_weight","label": "cruse_rule3  (contra load)", "min": 0.0,  "max": 2.0,  "step": 0.1},
	{"key": "cruse_rule5_gain",  "label": "cruse_rule5 (load→grip)",    "min": 0.0,  "max": 0.30, "step": 0.02},
	{"key": "stance_lift_gain",  "label": "stance_lift (belly-up knee)", "min": -1.5, "max": 1.5, "step": 0.1},
	# Purchase gate on the power stroke. Needs torque_topic wired (ConstructionOnly, so
	# this slider cannot supply it -- use a loadstroke config arm). 0 = off.
	{"key": "stroke_load_gain",  "label": "stroke_load (purchase gate)", "min": 0.0, "max": 8.0, "step": 0.5},
	# 2026-07-27 — stroke TIMING, live.  `stroke_phase` never had a slider even though it is
	# the sharpest knob on the whole gait: the deployed -2.85 sits in a good window only
	# ~90 deg wide, and both cardinal offsets outside it (0, pi/2) collapse the gait to
	# net_z ~0.  Pair this with the inspector's Gait-raster tab (gait_raster_diag=1) and the
	# push band can be slid into alignment with stance BY EYE, which is the tightest loop
	# available for a timing lever.
	{"key": "stroke_phase",      "label": "stroke_phase (push timing)", "min": -3.14, "max": 3.14, "step": 0.05},
	# Phase SOURCE. 0 = the leg's own oscillator phase (deployed). 1 = a touchdown-referenced
	# step clock, 2 = the same from hip1 load.  1 and 2 are REFUTED as gait levers (net_z
	# 4.58 -> 0.20 at the deployed offset, refuted across the full circle against a matched
	# control row) -- kept on the panel because flipping it live against the raster is the
	# clearest demonstration of WHY: the legs keep stepping and the body stops moving.
	{"key": "stroke_phase_src",  "label": "phase src: 0=leg 1=contact 2=load", "min": 0.0, "max": 2.0, "step": 1.0},
	# Swing tuck: the mirror of stance_lift -- folds hip2+knee on legs that are OFF the
	# ground so sweeping them forward stops spinning the chassis. Needs contact_topic.
	# Tibia-plumb: hip2 nulls the shank's deviation from vertical so the knee's drive
	# translates the foot instead of arcing it. Needs no extra topics.
	{"key": "tibia_plumb_gain",  "label": "tibia_plumb (hip2 -> shank vert)", "min": -1.5, "max": 1.5, "step": 0.1},
	{"key": "swing_tuck_hip2",   "label": "swing_tuck hip2 (fold femur)", "min": -1.5, "max": 1.5, "step": 0.1},
	{"key": "swing_tuck_knee",   "label": "swing_tuck knee (fold shank)", "min": -1.5, "max": 1.5, "step": 0.1},
	{"key": "coord_reward_drive","label": "agency_drive (phase search)","min": 0.0,  "max": 0.60, "step": 0.05},
	{"key": "coord_stab_penalty","label": "agency_stab (tilt guard)",   "min": 0.0,  "max": 1.0,  "step": 0.05},
	{"key": "coord_fitness_mode","label": "fitness: 0=fwd_v REWARD 1=free","min": 0.0, "max": 1.0, "step": 1.0},
	# ---- plasticity / forgetting (2026-07-26) --------------------------------------
	# homeo_leak_cycles is the "learn fast, forget fast" knob: the stride-cycle time
	# constant over which height_bias + amp_gain forget toward minimum authority, so an
	# inverted excursion cannot latch.  ~5 measured free; ~2 is too aggressive (wobble).
	# NOTE it wants rhythm_topic=rhythm.body.gait in the CONFIG to read the body's real
	# stride period; without it the leak falls back to a fixed 70-tick stride.
	{"key": "homeo_leak_cycles", "label": "homeo_leak (forget, strides)", "min": 0.0, "max": 20.0, "step": 1.0},
	{"key": "homeo_upright_gate","label": "upright_gate (FREEZE homeo)",  "min": 0.0, "max": 1.0,  "step": 0.05},
	{"key": "height_unwind_free","label": "height unwind-while-moving",   "min": 0.0, "max": 1.0,  "step": 1.0},
	{"key": "swing_hyst_frac",   "label": "swing deadband (MAD units)",   "min": 0.0, "max": 3.0,  "step": 0.25},
	{"key": "coord_lat_penalty", "label": "agency_lat  (anti-crab)",    "min": 0.0,  "max": 1.0,  "step": 0.05},
]

var _body: Node = null
var _content_vb: VBoxContainer = null
var _status: Label = null
var _minimise_btn: Button = null
var _value_labels: Dictionary = {}    # key -> Label
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
	title.text = "MOTOR-EPM  (live homeokinetic controller knobs)"
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
		_build_slider_row(_content_vb, spec)

	_status = Label.new()
	_status.add_theme_font_size_override("font_size", 10)
	_status.add_theme_color_override("font_color", Color(0.7, 0.8, 0.7, 1.0))
	_status.text = "waiting for body…  (legs weak → raise motor_gain; jerky → lower ctrl_lr/noise)"
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
	slider.value_changed.connect(func(v: float) -> void: _on_slider(key, v))
	row.add_child(slider)

	var value_label := Label.new()
	value_label.add_theme_font_size_override("font_size", 10)
	value_label.custom_minimum_size = Vector2(46, 0)
	value_label.text = "—"
	row.add_child(value_label)
	_value_labels[key] = value_label
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
	# Pull current MotorEPM params from the schema so the sliders + value
	# labels start where the config left them, without firing apply.  Returns
	# false (keep retrying) until the brain is initialized and answers.
	var br = _brain()
	if br == null or not br.has_method("get_module_param_schema"):
		return false
	var schema = br.get_module_param_schema(_MODULE_ID)
	if typeof(schema) != TYPE_ARRAY or schema.is_empty():
		return false   # brain not initialized yet — retry next frame
	var cur: Dictionary = {}
	for entry in schema:
		if typeof(entry) == TYPE_DICTIONARY and entry.has("key"):
			# OgmaBrain returns current_value (falls back to default_value).
			cur[str(entry["key"])] = entry.get("current_value", entry.get("default_value", null))
	for key in _value_labels.keys():
		if cur.has(key) and cur[key] != null:
			var v := float(cur[key])
			var lbl: Label = _value_labels[key]
			var sld: HSlider = lbl.get_meta("slider")
			sld.set_value_no_signal(v)
			lbl.text = "%.3f" % v
	_status.text = "MotorEPM live — drag to retune by eye (legs weak → motor_gain↑)"
	return true

func _on_slider(key: String, v: float) -> void:
	var br = _brain()
	if br == null or not br.has_method("apply_patch"):
		return
	var res = br.apply_patch({"op": "set_param", "id": _MODULE_ID, "key": key, "value": v})
	if _value_labels.has(key):
		_value_labels[key].text = "%.3f" % v
	if _status != null:
		var ok: bool = typeof(res) == TYPE_DICTIONARY and bool(res.get("success", false))
		_status.text = "%s = %.3f%s" % [key, v, ("" if ok else "  (apply failed: " + str(res.get("error", "")) + ")")]

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
