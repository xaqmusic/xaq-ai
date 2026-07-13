extends Control
## Live reward-policy editor for picrawler.  Sliders / dropdowns write
## directly to body.<field>; changes take effect on the next physics
## tick.  Use case: experiment-mode runs where the user wants to feel
## out a curriculum by hand — drag a slider, watch the body's behaviour
## shift, repeat.  When CurriculumManager has an active curriculum
## loaded the panel auto-hides itself so its sliders don't fight the
## curriculum's stage overrides.
##
## Exports the current settings as a curriculum-stage JSON snippet
## (clipboard or file) so promising hand-tuned regimes can become
## new curriculum stages directly.
##
## Methodology: panel is part of the picrawler SCENE only — headless
## runs don't instance it.  No body-side state changes here; just
## remote control of the existing @export reward knobs.

const _BODY_CANDIDATES: Array = ["Picrawler", "Quadruped"]

var _body: Node = null

# Slider control records.  Each entry: {field, label, min, max, step}.
# field must be an @export var on picrawler_body.gd in
# _CURRICULUM_ALLOWED_KEYS (we don't expose arbitrary internals).
const _SLIDERS: Array = [
	# --- HEIGHT REWARD (trapezoid uses target_height; inverted_u uses peak_height + band_width) ---
	{"field": "target_height",            "label": "target_height",            "min": 0.02, "max": 0.15, "step": 0.005},
	{"field": "peak_height",              "label": "peak_height (inverted_u)", "min": 0.02, "max": 0.10, "step": 0.005},
	{"field": "band_width",               "label": "band_width  (inverted_u)", "min": 0.005,"max": 0.10, "step": 0.005},
	{"field": "reward_min_height",        "label": "reward_min_height (V9)",   "min": 0.00, "max": 0.07, "step": 0.005},
	{"field": "height_penalty_grace",     "label": "height_penalty_grace",     "min": 0.00, "max": 0.05, "step": 0.005},
	{"field": "height_penalty_scale",     "label": "height_penalty_scale",     "min": 0.005,"max": 0.10, "step": 0.005},
	{"field": "height_penalty_gain",      "label": "height_penalty_gain",      "min": 0.00, "max": 0.30, "step": 0.005},
	{"field": "standing_baseline_factor", "label": "standing_baseline_factor", "min": 0.00, "max": 1.50, "step": 0.05},
	{"field": "standing_reward_ema_alpha","label": "standing_reward_ema_alpha (V8)", "min": 0.01, "max": 1.00, "step": 0.01},
	# --- WALK / GATED REWARD ---
	{"field": "walk_target_velocity",     "label": "walk_target_velocity",     "min": 0.05, "max": 2.00, "step": 0.05},
	{"field": "walk_hit_rate",            "label": "walk_hit_rate",            "min": 0.00, "max": 0.30, "step": 0.01},
	{"field": "gated_walk_bonus_rate",    "label": "gated_walk_bonus_rate",    "min": 0.00, "max": 0.50, "step": 0.01},
	{"field": "phase_contrast_gain",      "label": "phase_contrast_gain",      "min": 0.00, "max": 1.00, "step": 0.05},
	{"field": "stance_y_threshold",       "label": "stance_y_threshold",       "min": 0.01, "max": 0.10, "step": 0.005},
	# --- GAIT-CYCLE PULSE REWARD (2026-05-30) — fires once per 4-foot cycle with net forward progress ---
	{"field": "gait_cycle_reward_gain",   "label": "gait_cycle_reward_gain",   "min": 0.00, "max": 2.00, "step": 0.05},
	{"field": "gait_cycle_window_ticks",  "label": "gait_cycle_window_ticks",  "min": 30,   "max": 360,  "step": 10},
	{"field": "gait_cycle_min_progress",  "label": "gait_cycle_min_progress (floor)",  "min": 0.001,"max": 0.05, "step": 0.001},
	{"field": "gait_cycle_max_backward",  "label": "gait_cycle_max_backward (floor)",  "min": 0.000,"max": 0.02, "step": 0.001},
	{"field": "gait_cycle_progress_K",    "label": "gait_cycle_progress_K (adaptive)",  "min": 0.05, "max": 2.0,  "step": 0.05},
	{"field": "gait_cycle_wobble_K",      "label": "gait_cycle_wobble_K (adaptive)",    "min": 0.5,  "max": 5.0,  "step": 0.1},
	{"field": "gait_cycle_threshold_ema_alpha","label": "gait_cycle_threshold_ema_alpha","min": 0.001,"max": 0.2,  "step": 0.001},
	{"field": "gait_cycle_warmup_cycles", "label": "gait_cycle_warmup_cycles", "min": 5,   "max": 200, "step": 5},
	{"field": "gait_cycle_consecutive_required","label": "gait_cycle_consecutive_required","min": 1, "max": 10, "step": 1},
	# --- TILT / STABILITY / RECOVERY ---
	{"field": "tilt_target_rad",          "label": "tilt_target_rad",          "min": 0.00, "max": 1.57, "step": 0.05},
	{"field": "fail_tilt_rad",            "label": "fail_tilt_rad",            "min": 0.50, "max": 3.14, "step": 0.05},
	{"field": "level_chassis_rate",       "label": "level_chassis_rate",       "min": 0.00, "max": 1.00, "step": 0.05},
	# --- MISC ---
	{"field": "per_leg_credit_gain",      "label": "per_leg_credit_gain",      "min": 0.00, "max": 1.00, "step": 0.05},
	{"field": "progress_reward_gain",     "label": "progress_reward_gain",     "min": 0.00, "max": 1.00, "step": 0.05},
	# --- STAGE 3.E — joint compliance / motor authority (2026-06-01) ---
	{"field": "motor_authority_scale",    "label": "motor_authority_scale (3.E)",    "min": 0.10, "max": 1.50, "step": 0.05},
	{"field": "motor_damping_factor",     "label": "motor_damping_factor (3.E++)",   "min": 0.00, "max": 1.50, "step": 0.05},
	# joint_spring_stiffness/damping (apply_torque path) was attempted but is
	# numerically unstable at picrawler scale.  Deferred to a future
	# Generic6DOFJoint3D migration which has native angular spring/damping.
	{"field": "motor_freeplay_rad",       "label": "motor_freeplay_rad (3.E++ ±°)",  "min": 0.00, "max": 0.30, "step": 0.005},
	# --- G6DOF JOINT BACKEND (2026-06-03) — only active when joint_backend = "g6dof" ---
	# Diagnostic: motor_force_scale lets us probe whether Bullet's G6DOF
	# FORCE_LIMIT semantic over-feeds motor authority by ~physics_hz (240×).
	# Try 1.0 → 0.01 → 0.00417 (=1/physics_hz) to bracket the unit question.
	{"field": "motor_force_scale",           "label": "motor_force_scale (G6DOF)",        "min": 0.001, "max": 2.00, "step": 0.001},
	{"field": "joint_angular_damping",       "label": "joint_angular_damping (G6DOF)",    "min": 0.00,  "max": 2.00, "step": 0.05},
	{"field": "joint_angular_erp",           "label": "joint_angular_erp (G6DOF)",        "min": 0.00,  "max": 1.00, "step": 0.05},
	{"field": "joint_angular_limit_softness","label": "joint_angular_limit_softness (G6DOF)","min": 0.00, "max": 1.00, "step": 0.05},
	# Per-joint-type spring (G6DOF only) — start at 0 (rigid) then bump
	# knee_spring_stiffness first to introduce knee compliance.
	{"field": "hip1_spring_stiffness",       "label": "hip1_spring_stiffness (G6DOF)",    "min": 0.00, "max": 10.0, "step": 0.1},
	{"field": "hip1_spring_damping",         "label": "hip1_spring_damping (G6DOF)",      "min": 0.00, "max": 2.00, "step": 0.05},
	{"field": "hip2_spring_stiffness",       "label": "hip2_spring_stiffness (G6DOF)",    "min": 0.00, "max": 10.0, "step": 0.1},
	{"field": "hip2_spring_damping",         "label": "hip2_spring_damping (G6DOF)",      "min": 0.00, "max": 2.00, "step": 0.05},
	{"field": "knee_spring_stiffness",       "label": "knee_spring_stiffness (G6DOF)",    "min": 0.00, "max": 10.0, "step": 0.1},
	{"field": "knee_spring_damping",         "label": "knee_spring_damping (G6DOF)",      "min": 0.00, "max": 2.00, "step": 0.05},
	# --- STAGE 3.D — Bernoulli-impulse actuation (2026-06-01). Live tuning. ---
	# Default (impulse=0.05, friction=0.05) traded response speed for smoothness;
	# operator observation: legs smoother but standing degraded. Higher impulse
	# + lower friction = larger sustained motion (good for gait emergence).
	{"field": "bri_base_rate",            "label": "bri_base_rate (3.D)",            "min": 0.00, "max": 1.00, "step": 0.05},
	{"field": "bri_command_bias",         "label": "bri_command_bias (3.D)",         "min": 0.00, "max": 1.00, "step": 0.05},
	{"field": "bri_impulse_per_spike",    "label": "bri_impulse_per_spike (3.D)",    "min": 0.005,"max": 0.20, "step": 0.005},
	{"field": "bri_friction_per_tick",    "label": "bri_friction_per_tick (3.D)",    "min": 0.005,"max": 0.20, "step": 0.005},
	# 2026-06-08 — runtime Cruse bias gain (live coordination strength).
	# 0.0 = Cruse silent; 1.0 = full bias.  Lets you observe Cruse on/off
	# effects mid-experiment and lets curriculum stage 1 silence Cruse for
	# clean standing learning.
	{"field": "cruse_bias_gain",          "label": "cruse_bias_gain",                "min": 0.00, "max": 2.00, "step": 0.05},
	# 2026-06-08 — per-joint-kind gain on top of cruse_bias_gain.  Default 0.0
	# means Cruse does NOT bias the knee (knee learns its own timing from
	# foot-clearance + body-height signals).  Set to 1.0 to restore legacy
	# "Cruse pushes knee toward fold during stance" behavior (drives the claw).
	{"field": "cruse_bias_gain_knee",     "label": "cruse_bias_gain_knee (≥0=on, 0=off)", "min": 0.00, "max": 2.00, "step": 0.05},
	# 2026-06-08 Move 2 — Cruse bias on hip1 (yaw / swing direction).  Default 0.0
	# = legacy (hip1 unbiased, no swing-coordination signal).  Set to 1.0 in walking
	# stages to enable: during stance push body forward, during swing lift leg forward.
	{"field": "cruse_bias_gain_hip1",     "label": "cruse_bias_gain_hip1 (Move 2: swing)", "min": 0.00, "max": 2.00, "step": 0.05},
	# 2026-06-09 Move 4 — Cruse bias on hip2 (lift / vertical).  Default 1.0
	# = legacy.  > 1.0 amplifies down-during-stance + up-during-swing to help
	# plant the thrusting leg + lift the swinging leg as Joseph requested.
	{"field": "cruse_bias_gain_hip2",     "label": "cruse_bias_gain_hip2 (Move 4: lift)",  "min": 0.00, "max": 3.00, "step": 0.05},
	# 2026-06-09 Move 5 — position-aware saturation gate.  zone_min = wrong-side
	# floor (typically 0.0 = require leg past perpendicular before pushing
	# further).  zone_max = saturation ceiling (typically 0.9 = stop pushing
	# when leg is already ≥90% of the way to the bias-direction extreme).
	{"field": "saturation_zone_min",      "label": "saturation_zone_min (Move 5)",         "min": -1.00, "max": 1.00, "step": 0.05},
	{"field": "saturation_zone_max",      "label": "saturation_zone_max (Move 5)",         "min":  0.00, "max": 1.00, "step": 0.05},
]
const _DROPDOWNS: Array = [
	{"field": "reward_shape",            "label": "reward_shape",            "options": ["trapezoid", "inverted_u"]},
	{"field": "peak_height_mode",        "label": "peak_height_mode (V9)",   "options": ["fixed", "knee_relative"]},
	{"field": "height_reference",        "label": "height_reference (V9)",   "options": ["chassis", "body_cog"]},
	{"field": "walk_reward_mode",        "label": "walk_reward_mode",        "options": ["radial", "radial_penalize_inward", "total_speed", "to_target"]},
	# 2026-06-07 — manual target_mode control so Launch-experiment users can
	# enable a random_pyramid target without loading a curriculum.  The body's
	# _ensure_target_pyramid_color() polls this field every 60 ticks and
	# picks/clears the target when it flips.
	{"field": "target_mode",             "label": "target_mode",             "options": ["off", "random_pyramid"]},
	# 2026-06-08 — toggle the periodic [CRUSE] textual trace.  Prints planted/swing
	# state, rule fires/sec, knee bias norms, current gain, body_state value.
	{"field": "_cruse_trace_enabled",    "label": "cruse_trace",             "options": ["off", "on"]},
	# 2026-06-09 Move 5 — saturation gate on/off toggle.  Wraps the bool @export.
	{"field": "saturation_gate_enabled", "label": "saturation_gate (Move 5)", "options": ["off", "on"]},
	{"field": "gated_walk_velocity_mode","label": "gated_walk_velocity_mode (V8)", "options": ["radial", "body_forward"]},
	{"field": "gait_cycle_adaptive_thresholds","label": "gait_cycle_adaptive_thresholds", "options": ["off", "on"]},
	# Stage 3.D — flip actuation pipeline live without restarting Godot.
	# Default "discrete" matches all prior behavior bit-identically.
	{"field": "actuation_backend",       "label": "actuation_backend (3.D)", "options": ["discrete", "bernoulli_impulse"]},
]

# Per-row UI references so _process can update labels with live values.
var _slider_widgets: Dictionary = {}   # field → {slider, value_label}
var _dropdown_widgets: Dictionary = {} # field → OptionButton

var _status: Label = null
# 2026-06-09 — minimise/expand toggle.  Joseph QoL ask: panel takes up
# substantial vertical real estate during UI observation; need to hide
# the sliders without losing the panel entirely.
var _minimise_btn: Button = null
var _content_vb: VBoxContainer = null
var _is_minimised: bool = false
# Cache the un-collapsed layout so we can restore it on expand.  The panel
# uses anchor_bottom=1.0 so we have to cache that too, and flip to 0.0
# while collapsed (otherwise offset_bottom is measured from the BOTTOM of
# the screen and the panel ends up empty at the top).  Initialised lazily.
var _full_anchor_bottom: float = -1.0
var _full_offset_bottom: float = 0.0
const _COLLAPSED_HEIGHT: float = 38.0   # just the header row + frame padding
var _root_panel: PanelContainer = null

func _ready() -> void:
	# 2026-06-09 — TOP-RIGHT anchor with a generous right margin so the
	# panel can't extend past the visible viewport regardless of display
	# quirks.  Joseph confirmed the panel is hideable now (T hotkey),
	# so a wide visible footprint is acceptable.  Width 380 + 200px right
	# margin → left edge at viewport-580, right edge at viewport-200.
	anchor_left    = 1.0
	anchor_top     = 0.0
	anchor_right   = 1.0
	anchor_bottom  = 1.0
	offset_left    = -200.0 - 380.0
	offset_top     = 12.0
	offset_right   = -200.0
	offset_bottom  = -12.0
	mouse_filter  = Control.MOUSE_FILTER_PASS

	_root_panel = PanelContainer.new()
	_root_panel.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var bg_style := StyleBoxFlat.new()
	bg_style.bg_color = Color(0.05, 0.05, 0.07, 0.82)
	bg_style.border_color = Color(0.55, 0.55, 0.6, 0.85)
	bg_style.set_border_width_all(1)
	bg_style.set_corner_radius_all(4)
	bg_style.content_margin_left = 10
	bg_style.content_margin_right = 10
	bg_style.content_margin_top = 8
	bg_style.content_margin_bottom = 8
	_root_panel.add_theme_stylebox_override("panel", bg_style)
	add_child(_root_panel)

	var outer_vb := VBoxContainer.new()
	outer_vb.add_theme_constant_override("separation", 4)
	_root_panel.add_child(outer_vb)

	# 2026-06-09 — header row: title + minimise button.  Collapsed state
	# hides the scrollable param area + status row, keeps the header so the
	# user can re-expand.
	var header_row := HBoxContainer.new()
	header_row.add_theme_constant_override("separation", 4)
	outer_vb.add_child(header_row)
	var title := Label.new()
	title.text = "REWARD POLICY  (live picrawler body @export params; Cruse params via config)"
	title.add_theme_font_size_override("font_size", 13)
	title.add_theme_color_override("font_color", Color(0.95, 0.95, 1.0, 1.0))
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header_row.add_child(title)
	_minimise_btn = Button.new()
	_minimise_btn.text = "▼"
	_minimise_btn.custom_minimum_size = Vector2(28, 0)
	_minimise_btn.pressed.connect(_on_minimise_pressed)
	header_row.add_child(_minimise_btn)

	# Content wrapper — minimise toggles this whole block at once.
	_content_vb = VBoxContainer.new()
	_content_vb.add_theme_constant_override("separation", 4)
	_content_vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_content_vb.size_flags_vertical   = Control.SIZE_EXPAND_FILL
	outer_vb.add_child(_content_vb)

	# Scrollable inner container — expanded V8/V9/V10 param set exceeds
	# available vertical space, so wrap in ScrollContainer to keep panel
	# bounded while exposing all controls.
	var scroll := ScrollContainer.new()
	scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.size_flags_vertical   = Control.SIZE_EXPAND_FILL
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	_content_vb.add_child(scroll)

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 4)
	vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(vb)

	for spec in _SLIDERS:
		_build_slider_row(vb, spec)

	var sep1 := HSeparator.new()
	vb.add_child(sep1)

	for spec in _DROPDOWNS:
		_build_dropdown_row(vb, spec)

	var sep2 := HSeparator.new()
	_content_vb.add_child(sep2)

	# Export / Load buttons + status label pinned at the bottom of the
	# content area (still inside _content_vb so minimise hides them too).
	var btn_row := HBoxContainer.new()
	btn_row.add_theme_constant_override("separation", 6)
	_content_vb.add_child(btn_row)

	var export_btn := Button.new()
	export_btn.text = "Copy JSON"
	export_btn.add_theme_font_size_override("font_size", 11)
	export_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	export_btn.pressed.connect(_on_export_pressed)
	btn_row.add_child(export_btn)

	var load_btn := Button.new()
	load_btn.text = "Load JSON"
	load_btn.add_theme_font_size_override("font_size", 11)
	load_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	load_btn.pressed.connect(_on_load_pressed)
	btn_row.add_child(load_btn)

	_status = Label.new()
	_status.add_theme_font_size_override("font_size", 10)
	_status.add_theme_color_override("font_color", Color(0.75, 0.75, 0.8, 1.0))
	_status.text = "edit sliders/dropdowns to retune reward live (Cruse params via cruse_stance_signs.py or config edit)"
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_content_vb.add_child(_status)

func _build_slider_row(parent: VBoxContainer, spec: Dictionary) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 6)
	parent.add_child(row)

	var name_label := Label.new()
	name_label.text = str(spec["label"])
	name_label.add_theme_font_size_override("font_size", 11)
	name_label.custom_minimum_size = Vector2(130, 0)
	row.add_child(name_label)

	var slider := HSlider.new()
	slider.min_value = float(spec["min"])
	slider.max_value = float(spec["max"])
	slider.step      = float(spec["step"])
	# 2026-06-09 — slider back to SIZE_EXPAND_FILL (Joseph confirmed the
	# adaptive slider widths look better).  With the panel pushed left by
	# the 200px right margin, the panel is fully visible and the slider
	# has plenty of room to grow.
	slider.custom_minimum_size = Vector2(120, 16)
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.value_changed.connect(func(v: float) -> void: _on_slider_changed(str(spec["field"]), v))
	row.add_child(slider)

	var value_label := Label.new()
	value_label.text = "—"
	value_label.add_theme_font_size_override("font_size", 10)
	value_label.custom_minimum_size = Vector2(40, 0)
	row.add_child(value_label)

	_slider_widgets[spec["field"]] = {"slider": slider, "value_label": value_label}

func _build_dropdown_row(parent: VBoxContainer, spec: Dictionary) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 6)
	parent.add_child(row)

	var name_label := Label.new()
	name_label.text = str(spec["label"])
	name_label.add_theme_font_size_override("font_size", 11)
	name_label.custom_minimum_size = Vector2(130, 0)
	row.add_child(name_label)

	var dd := OptionButton.new()
	dd.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	dd.add_theme_font_size_override("font_size", 11)
	for opt in spec["options"]:
		dd.add_item(str(opt))
	dd.item_selected.connect(func(idx: int) -> void:
		_on_dropdown_changed(str(spec["field"]), spec["options"][idx]))
	row.add_child(dd)

	_dropdown_widgets[spec["field"]] = dd

func _process(_delta: float) -> void:
	if _body == null or not is_instance_valid(_body):
		for nm in _BODY_CANDIDATES:
			var b: Node = get_tree().get_root().find_child(nm, true, false)
			if b != null:
				_body = b
				_sync_widgets_from_body()   # initial pull
				# 2026-06-09 — sync sliders on every curriculum stage advance
				# so the panel reflects the active stage's overrides.  Joseph
				# UI QoL ask: want curriculum-driven values visible AND
				# overridable mid-stage (the curriculum re-syncs at the next
				# advance).  signal name: stage_changed(idx, name, overrides).
				var cur_mgr: Node = get_node_or_null("/root/CurriculumManager")
				if cur_mgr != null and cur_mgr.has_signal("stage_changed"):
					if not cur_mgr.is_connected("stage_changed", _on_curriculum_stage_changed):
						cur_mgr.connect("stage_changed", _on_curriculum_stage_changed)
				break
		if _body == null:
			return

	# 2026-06-09 — panel always visible (was: hidden when curriculum loaded).
	# Sliders fight the stage timer briefly but the curriculum re-applies on
	# advance, so user overrides are temporary within a stage.  Joseph's
	# explicit ask: "modify an advancing curriculum on the fly".

	# Live-update the per-slider value labels (in case body fields
	# changed elsewhere, e.g. via env var or another panel).
	for field in _slider_widgets:
		var v: Variant = _body.get(field)
		if v == null:
			continue
		var lbl: Label = _slider_widgets[field]["value_label"]
		lbl.text = "%.3f" % float(v)

# One-shot pull from body when first connected — picks up @export defaults
# / env-var resolved values so the sliders start at the actual settings.
func _sync_widgets_from_body() -> void:
	if _body == null:
		return
	for field in _slider_widgets:
		var v: Variant = _body.get(field)
		if v == null:
			continue
		var slider: HSlider = _slider_widgets[field]["slider"]
		slider.set_block_signals(true)
		slider.value = float(v)
		slider.set_block_signals(false)
		_slider_widgets[field]["value_label"].text = "%.3f" % float(v)
	for field in _dropdown_widgets:
		var v: Variant = _body.get(field)
		if v == null:
			continue
		var dd: OptionButton = _dropdown_widgets[field]
		# bool fields display as "off"/"on" in the dropdown but
		# body holds them as bool — map for sync.
		var match_str: String = str(v)
		if typeof(v) == TYPE_BOOL:
			match_str = "on" if v else "off"
		for i in range(dd.item_count):
			if dd.get_item_text(i) == match_str:
				dd.set_block_signals(true)
				dd.select(i)
				dd.set_block_signals(false)
				break

# 2026-06-09 — re-sync slider positions when curriculum advances stages.
# The body's _apply_curriculum_overrides() has already written the new
# @export values; we just pull them into the slider widgets.
func _on_curriculum_stage_changed(_idx: int, _name: String, _overrides: Dictionary) -> void:
	_sync_widgets_from_body()

func _on_slider_changed(field: String, v: float) -> void:
	if _body == null:
		return
	_body.set(field, v)
	if _status != null:
		_status.text = "%s = %.3f" % [field, v]

func _on_dropdown_changed(field: String, value: Variant) -> void:
	if _body == null:
		return
	# bool fields use "off"/"on" labels — convert before setting.
	var current: Variant = _body.get(field)
	if typeof(current) == TYPE_BOOL:
		_body.set(field, str(value) == "on")
	else:
		_body.set(field, value)
	if _status != null:
		_status.text = "%s = %s" % [field, str(value)]

# Exports the current settings as a curriculum-stage JSON snippet to
# the system clipboard.  User can paste it into a curriculum file and
# it'll work as-is (modulo "name", "description", and "advance_when"
# which are stage metadata, not reward params).
func _on_export_pressed() -> void:
	if _body == null:
		if _status != null:
			_status.text = "no body — cannot export"
		return
	var overrides: Dictionary = {}
	for field in _slider_widgets:
		var v: Variant = _body.get(field)
		if v != null:
			overrides[field] = float(v)
	for field in _dropdown_widgets:
		var v: Variant = _body.get(field)
		if v != null:
			overrides[field] = str(v)
	var stage: Dictionary = {
		"name":        "stage_hand_tuned",
		"description": "Hand-tuned via reward_panel.  Edit name / description / advance_when before adding to a curriculum file.",
		"overrides":   overrides,
	}
	var json_text: String = JSON.stringify(stage, "  ")
	DisplayServer.clipboard_set(json_text)
	if _status != null:
		_status.text = "exported %d settings → clipboard" % overrides.size()
	print("RewardPanel: exported stage JSON to clipboard:\n%s" % json_text)

# Open a file picker → load a JSON file → apply its overrides to the
# live body + sync the UI widgets.  Accepted shapes:
#   {"overrides": {...}}           — stage snippet (from Export above)
#   [{...stage...}, ...]            — full curriculum file: loads first stage's overrides
#   {<field>: <value>, ...}         — bare overrides dict
# Out-of-range numeric values are clamped to the slider's [min, max].
# Fields that aren't on the panel are listed in the status line but not
# applied (we don't bypass the panel whitelist).
func _on_load_pressed() -> void:
	var dlg := FileDialog.new()
	dlg.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	dlg.access = FileDialog.ACCESS_FILESYSTEM
	dlg.use_native_dialog = true
	dlg.add_filter("*.json", "JSON files")
	var curr_dir := ProjectSettings.globalize_path("res://curricula/")
	if DirAccess.dir_exists_absolute(curr_dir):
		dlg.current_dir = curr_dir
	dlg.file_selected.connect(_on_load_file_selected)
	dlg.close_requested.connect(func() -> void: dlg.queue_free())
	add_child(dlg)
	dlg.popup_centered(Vector2(900, 600))

func _on_load_file_selected(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		if _status != null:
			_status.text = "could not open %s" % path.get_file()
		return
	var text := f.get_as_text()
	f.close()
	var parsed: Variant = JSON.parse_string(text)
	if parsed == null:
		if _status != null:
			_status.text = "JSON parse failed: %s" % path.get_file()
		return
	var overrides: Dictionary = {}
	if parsed is Dictionary:
		if parsed.has("overrides") and parsed["overrides"] is Dictionary:
			overrides = parsed["overrides"]
		else:
			overrides = parsed
	elif parsed is Array and parsed.size() > 0 and parsed[0] is Dictionary:
		var first: Dictionary = parsed[0]
		overrides = first.get("overrides", first)
	if overrides.is_empty():
		if _status != null:
			_status.text = "no overrides found in %s" % path.get_file()
		return
	_apply_overrides(overrides, path.get_file())

func _apply_overrides(overrides: Dictionary, src_label: String) -> void:
	if _body == null:
		if _status != null:
			_status.text = "no body — cannot apply"
		return
	var n_applied: int = 0
	var n_clamped: int = 0
	var skipped: Array = []
	var unknown_dd: Array = []
	for field in overrides.keys():
		var val: Variant = overrides[field]
		if _slider_widgets.has(field):
			var slider: HSlider = _slider_widgets[field]["slider"]
			var raw: float = float(val)
			var clamped: float = clamp(raw, slider.min_value, slider.max_value)
			if clamped != raw:
				n_clamped += 1
			slider.set_block_signals(true)
			slider.value = clamped
			slider.set_block_signals(false)
			_body.set(field, clamped)
			_slider_widgets[field]["value_label"].text = "%.3f" % clamped
			n_applied += 1
		elif _dropdown_widgets.has(field):
			var dd: OptionButton = _dropdown_widgets[field]
			# bool fields: JSON has true/false; dropdown options are "on"/"off".
			var current: Variant = _body.get(field)
			var sval: String
			if typeof(current) == TYPE_BOOL:
				# Accept bool true/false OR string "on"/"off"/"true"/"false".
				var as_bool: bool
				if typeof(val) == TYPE_BOOL:
					as_bool = val
				else:
					var sv: String = str(val).to_lower()
					as_bool = (sv == "on" or sv == "true" or sv == "1")
				sval = "on" if as_bool else "off"
			else:
				sval = str(val)
			var matched: bool = false
			for i in range(dd.item_count):
				if dd.get_item_text(i) == sval:
					dd.set_block_signals(true)
					dd.select(i)
					dd.set_block_signals(false)
					if typeof(current) == TYPE_BOOL:
						_body.set(field, sval == "on")
					else:
						_body.set(field, sval)
					n_applied += 1
					matched = true
					break
			if not matched:
				unknown_dd.append("%s=%s" % [field, sval])
		else:
			skipped.append(str(field))
	var msg: String = "loaded %d from %s" % [n_applied, src_label]
	if n_clamped > 0:
		msg += " (%d clamped)" % n_clamped
	if skipped.size() > 0:
		var head: Array = skipped.slice(0, 3)
		msg += "  skipped %d not-on-panel: %s%s" % [
			skipped.size(),
			", ".join(head),
			"…" if skipped.size() > 3 else "",
		]
	if unknown_dd.size() > 0:
		msg += "  bad dropdown val: %s" % ", ".join(unknown_dd)
	if _status != null:
		_status.text = msg
	print("RewardPanel: %s" % msg)

# 2026-06-09 — minimise/expand toggle.  Hides _content_vb AND shrinks the
# panel's offset_bottom so the surrounding HUD area is reclaimed (without
# the offset adjustment, the panel chrome stays the same size with empty
# space below the header — Joseph called that out).
func _on_minimise_pressed() -> void:
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
