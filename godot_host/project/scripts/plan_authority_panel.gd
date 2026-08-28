extends Control
## PLAN AUTHORITY bench — lever (b)'s operator instrument.  [O] toggles.
##
## OPERATOR SCAFFOLD, named as such (a diagnostic lesion bench, never an
## operating mode).  Three controls in one place:
##
##   1. MIX — plan_gain (MotorEPMv2): the reflex↔plan crossfade.  The fused
##      objective is w_eff = w_keyframe + plan_gain·w_plan per joint, so at 0
##      the pull is off (byte-identical) and at 1 a gated joint's position
##      error is fully the PLAN's error.  The motor-failure precedent: mix by
##      hand, feel the effect.
##   2. AUTHORITY — plan_gate_override (MotorPlanner): bypass the earned-bands
##      gate (weight 1 on ALL joints) so unearned authority can be FELT and
##      compared against earned-only.  The distress cut still applies.
##      plan_depth picks which future column the body is asked to fulfil.
##   3. HAND MASK — the mode-2 region mask params, so inhibition experiments
##      ("don't be at rest a quarter-stride out") can be driven live while the
##      pull is on: inhibit a predicted future and watch the motor output.
##
## Writes BRAIN params via brain.apply_patch (all targets HotMutable); reads
## back planner/motor snapshots at ~2 Hz for the gate/pull telemetry line.
## SCENE-only (headless runs never instance it).  Boots HIDDEN — [O] unhides.
## Anchored top-right like MotorEpmPanel ([M]); open one bench at a time.
##
## ⚠ Joint labels are ANATOMICAL, tinted nothing (text panel) but always with
## the cfg name beside them — the sim leg frame is mirrored L↔R (LEG_NAMES
## warning in picrawler_body.gd): cfg'fl' = RED = anatomical FR, etc.

const _PLANNER_ID: String = "motor_planner"
const _MOTOR_ID: String = "motor_epm"
const _PROBES: Array = [1, 3, 5, 8, 13, 21, 34]
# planner joint j → anatomical label: leg = j % 4 in cfg order [fl,fr,rl,rr],
# group = j / 4 in [h1,h2,knee].  Anatomy first, cfg cross-ref kept.
const _LEG_ANAT: Array = ["FR", "FL", "RR", "RL"]
const _LEG_CFG: Array = ["fl", "fr", "rl", "rr"]
const _GRP: Array = ["h1", "h2", "kn"]

var _status: Label = null
var _gate_line: Label = null
var _value_labels: Dictionary = {}      # "module/key" -> value Label (meta: slider)
var _joint_opt: OptionButton = null
var _depth_opt: OptionButton = null
var _override_btn: CheckButton = null
var _poll_accum: int = 0
var _synced: bool = false


func _brain():
	var body: Node = get_tree().get_root().find_child("Picrawler", true, false)
	if body == null:
		return null
	var br = body.get("brain")
	if br == null or not br.has_method("apply_patch"):
		return null
	return br


func _set_param(module: String, key: String, v: float) -> void:
	var br = _brain()
	if br == null:
		return
	var res = br.apply_patch({"op": "set_param", "id": module, "key": key, "value": v})
	var ok: bool = typeof(res) == TYPE_DICTIONARY and bool(res.get("success", false))
	if _status != null:
		_status.text = "%s.%s = %.3f%s" % [module, key, v,
			("" if ok else "  (FAILED: " + str(res.get("error", "")) + ")")]


func _mk_slider(parent: VBoxContainer, module: String, key: String, label: String,
		mn: float, mx: float, step: float) -> void:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = label
	l.custom_minimum_size = Vector2(168, 0)
	l.add_theme_font_size_override("font_size", 11)
	row.add_child(l)
	var s := HSlider.new()
	s.min_value = mn
	s.max_value = mx
	s.step = step
	s.custom_minimum_size = Vector2(110, 0)
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(s)
	var vl := Label.new()
	vl.text = "—"
	vl.custom_minimum_size = Vector2(44, 0)
	vl.add_theme_font_size_override("font_size", 11)
	vl.set_meta("slider", s)
	row.add_child(vl)
	var id: String = module + "/" + key
	_value_labels[id] = vl
	s.value_changed.connect(func(v: float) -> void:
		vl.text = "%.2f" % v
		_set_param(module, key, v))
	parent.add_child(row)


func _mk_section(parent: VBoxContainer, txt: String) -> void:
	var l := Label.new()
	l.text = txt
	l.add_theme_font_size_override("font_size", 12)
	l.add_theme_color_override("font_color", Color(0.95, 0.55, 0.85, 1.0))
	parent.add_child(l)


func _ready() -> void:
	anchor_left = 1.0
	anchor_top = 0.0
	anchor_right = 1.0
	anchor_bottom = 0.0
	offset_left = -12.0 - 340.0
	offset_top = 12.0
	offset_right = -12.0
	offset_bottom = 12.0 + 560.0
	mouse_filter = Control.MOUSE_FILTER_PASS
	visible = false           # bench: opt-in via [O]

	var root := PanelContainer.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var bg := StyleBoxFlat.new()
	bg.bg_color = Color(0.07, 0.04, 0.07, 0.86)
	bg.border_color = Color(0.85, 0.45, 0.75, 0.9)
	bg.set_border_width_all(1)
	bg.set_corner_radius_all(4)
	root.add_theme_stylebox_override("panel", bg)
	add_child(root)

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 3)
	root.add_child(vb)

	var hdr := Label.new()
	hdr.text = "PLAN AUTHORITY bench  [O]  — operator scaffold"
	hdr.add_theme_font_size_override("font_size", 13)
	hdr.add_theme_color_override("font_color", Color(1.0, 0.7, 0.95, 1.0))
	vb.add_child(hdr)

	_mk_section(vb, "MIX — reflex ↔ plan (consumer gain)")
	_mk_slider(vb, _MOTOR_ID, "plan_gain", "plan_gain (0=reflex only)", 0.0, 1.0, 0.01)

	_mk_section(vb, "AUTHORITY — the earned-bands gate")
	_override_btn = CheckButton.new()
	_override_btn.text = "gate OVERRIDE (weight 1 on ALL joints — unearned)"
	_override_btn.add_theme_font_size_override("font_size", 11)
	_override_btn.toggled.connect(func(on: bool) -> void:
		_set_param(_PLANNER_ID, "plan_gate_override", 1.0 if on else 0.0))
	vb.add_child(_override_btn)
	var drow := HBoxContainer.new()
	var dl := Label.new()
	dl.text = "plan_depth (ticks ahead)"
	dl.custom_minimum_size = Vector2(168, 0)
	dl.add_theme_font_size_override("font_size", 11)
	drow.add_child(dl)
	_depth_opt = OptionButton.new()
	for p in _PROBES:
		_depth_opt.add_item(str(p))
	_depth_opt.select(3)      # 8 — the verified band's inner edge
	_depth_opt.item_selected.connect(func(idx: int) -> void:
		_set_param(_PLANNER_ID, "plan_depth", float(_PROBES[idx])))
	drow.add_child(_depth_opt)
	vb.add_child(drow)

	_mk_section(vb, "FADE — reflex ↔ plan puppet (the lesion bench)")
	_mk_slider(vb, _MOTOR_ID, "plan_fade", "plan_fade (1=puppet, no reflex)", 0.0, 1.0, 0.01)
	_mk_slider(vb, _MOTOR_ID, "plan_puppet_gain", "puppet servo gain", 0.0, 8.0, 0.1)

	_mk_section(vb, "HAND MASK — mode-2 region inhibition, live")
	var grow := HBoxContainer.new()
	var gl := Label.new()
	gl.text = "group"
	gl.custom_minimum_size = Vector2(46, 0)
	gl.add_theme_font_size_override("font_size", 11)
	grow.add_child(gl)
	for preset in [["ALL h1", 15.0], ["ALL h2", 240.0], ["ALL knee", 3840.0], ["single", 0.0]]:
		var gb := Button.new()
		gb.text = preset[0]
		gb.add_theme_font_size_override("font_size", 11)
		var gv: float = preset[1]
		gb.pressed.connect(func() -> void:
			_set_param(_PLANNER_ID, "mask_joints", gv))
		grow.add_child(gb)
	vb.add_child(grow)
	var jrow := HBoxContainer.new()
	var jl := Label.new()
	jl.text = "mask_joint (anatomical)"
	jl.custom_minimum_size = Vector2(168, 0)
	jl.add_theme_font_size_override("font_size", 11)
	jrow.add_child(jl)
	_joint_opt = OptionButton.new()
	_joint_opt.add_item("ALL joints (-1)")
	for j in range(12):
		var leg: int = j % 4
		var grp: int = j / 4
		_joint_opt.add_item("%s·%s  (cfg %s, j%d)" % [_LEG_ANAT[leg], _GRP[grp], _LEG_CFG[leg], j])
	_joint_opt.item_selected.connect(func(idx: int) -> void:
		_set_param(_PLANNER_ID, "mask_joint", float(idx - 1)))
	jrow.add_child(_joint_opt)
	vb.add_child(jrow)
	_mk_slider(vb, _PLANNER_ID, "mask_val_lo", "mask_val_lo", -1.5, 1.5, 0.01)
	_mk_slider(vb, _PLANNER_ID, "mask_val_hi", "mask_val_hi", -1.5, 1.5, 0.01)
	_mk_slider(vb, _PLANNER_ID, "mask_depth_lo", "mask_depth_lo", 1.0, 40.0, 1.0)
	_mk_slider(vb, _PLANNER_ID, "mask_depth_hi", "mask_depth_hi", 1.0, 40.0, 1.0)
	_mk_slider(vb, _PLANNER_ID, "mask_strength", "mask_strength (0=off)", 0.0, 1.0, 0.05)

	var brow := HBoxContainer.new()
	var demo := Button.new()
	demo.text = "FR-knee rest demo"
	demo.add_theme_font_size_override("font_size", 11)
	demo.tooltip_text = "j8 (anatomical FR knee), rest region [0.7,1.05], depths 8–21, full strength — 'don't be at rest a quarter-stride out'"
	demo.pressed.connect(func() -> void:
		_apply_demo_mask())
	brow.add_child(demo)
	var off := Button.new()
	off.text = "mask OFF"
	off.add_theme_font_size_override("font_size", 11)
	off.pressed.connect(func() -> void:
		_set_param(_PLANNER_ID, "mask_strength", 0.0)
		_sync_slider(_PLANNER_ID, "mask_strength", 0.0))
	brow.add_child(off)
	vb.add_child(brow)

	_gate_line = Label.new()
	_gate_line.add_theme_font_size_override("font_size", 10)
	_gate_line.add_theme_color_override("font_color", Color(0.8, 0.8, 0.85, 1.0))
	_gate_line.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_gate_line.text = "gate: —"
	vb.add_child(_gate_line)

	_status = Label.new()
	_status.add_theme_font_size_override("font_size", 10)
	_status.add_theme_color_override("font_color", Color(0.72, 0.72, 0.75, 1.0))
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_status.text = "waiting for brain…"
	vb.add_child(_status)


func _apply_demo_mask() -> void:
	# The recorded M1 demo spec (ledger 2026-08-11): inhibit the FR-knee rest.
	for kv in [["mask_joint", 8.0], ["mask_val_lo", 0.7], ["mask_val_hi", 1.05],
			["mask_depth_lo", 8.0], ["mask_depth_hi", 21.0], ["mask_strength", 1.0]]:
		_set_param(_PLANNER_ID, kv[0], kv[1])
		_sync_slider(_PLANNER_ID, kv[0], kv[1])
	if _joint_opt != null:
		_joint_opt.select(9)   # item index = joint 8 + the ALL entry


func _sync_slider(module: String, key: String, v: float) -> void:
	var id: String = module + "/" + key
	if _value_labels.has(id):
		var vl: Label = _value_labels[id]
		var s: HSlider = vl.get_meta("slider")
		s.set_value_no_signal(v)
		vl.text = "%.2f" % v


func _sync_from_schema() -> bool:
	var br = _brain()
	if br == null or not br.has_method("get_module_param_schema"):
		return false
	for module in [_PLANNER_ID, _MOTOR_ID]:
		var schema = br.get_module_param_schema(module)
		if typeof(schema) != TYPE_ARRAY or schema.is_empty():
			return false
		for entry in schema:
			if typeof(entry) != TYPE_DICTIONARY or not entry.has("key"):
				continue
			var key: String = str(entry["key"])
			var cur = entry.get("current_value", entry.get("default_value", null))
			if cur == null or not (cur is float or cur is int):
				continue          # topic lists / string params — not slider material
			_sync_slider(module, key, float(cur))
			if key == "plan_gate_override" and _override_btn != null:
				_override_btn.set_pressed_no_signal(float(cur) >= 0.5)
			elif key == "plan_depth" and _depth_opt != null:
				var idx: int = _PROBES.find(int(cur))
				if idx >= 0:
					_depth_opt.select(idx)
			elif key == "mask_joint" and _joint_opt != null:
				_joint_opt.select(clampi(int(cur) + 1, 0, 12))
	_status.text = "live — mix by hand, feel the difference (earned vs override)"
	return true


func _process(_dt: float) -> void:
	if not visible:
		return
	if not _synced:
		_synced = _sync_from_schema()
		return
	_poll_accum += 1
	if _poll_accum < 30:      # ~2 Hz at 60 fps
		return
	_poll_accum = 0
	var br = _brain()
	if br == null or not br.has_method("get_module_snapshot"):
		return
	var ps = JSON.parse_string(str(br.get_module_snapshot(_PLANNER_ID)))
	var ms = JSON.parse_string(str(br.get_module_snapshot(_MOTOR_ID)))
	var gate_txt: String = "gate: (planner has no plan_pub — is this a planpull config?)"
	if ps is Dictionary and ps.has("module"):
		var pmod: Dictionary = ps["module"]
		if pmod.has("plan_pub"):
			var pp: Dictionary = pmod["plan_pub"]
			var w: Array = pp.get("w", [])
			var by_grp: String = ""
			if w.size() >= 12:
				for g in range(3):
					by_grp += " %s[" % _GRP[g]
					for leg in range(4):
						by_grp += "■" if float(w[g * 4 + leg]) > 0.5 else "·"
					by_grp += "]"
			gate_txt = "gate d%d%s %s  distress %.2f" % [
				int(pp.get("depth", 0)), by_grp,
				("OVERRIDE" if int(pp.get("override", 0)) == 1 else "earned"),
				float(pp.get("distress", 0.0))]
			var au = pmod.get("author", null)
			if au is Dictionary:
				gate_txt += "   kept %d" % [au.get("kept", []).size()]
	if ps is Dictionary and ps.has("module") and (ps["module"] as Dictionary).has("plan_pub"):
		var tug: Array = (ps["module"]["plan_pub"] as Dictionary).get("tug", [])
		if tug.size() >= 12:
			var by_g: String = ""
			for g in range(3):
				var s: float = 0.0
				for leg in range(4):
					s += absf(float(tug[g * 4 + leg]))
				by_g += " %s %.3f" % [_GRP[g], s / 4.0]
			gate_txt += "\ntug mean|Δ|:%s" % by_g
	if ms is Dictionary and ms.has("module"):
		var mmod: Dictionary = ms["module"]
		gate_txt += "\npull %.4f  w %.3f  fade %.2f   (gate-map leg order: FR FL RR RL — anatomical)" % [
			float(mmod.get("plan_pull", 0.0)), float(mmod.get("plan_w", 0.0)),
			float(mmod.get("plan_fade", 0.0))]
	_gate_line.text = gate_txt
