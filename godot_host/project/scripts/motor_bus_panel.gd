extends Control
## MotorBusPanel — per-influencer FADER panel for the MotorBus compressor.
##
## The operator's mixing-console surface: one horizontal fader (HSlider) per
## motor influencer (HK swim, cognitive decoder, whisker reflex, …), an
## activity dot + live L/R contribution readout, and a bus output + gain-
## reduction meter at the bottom.  Moving a fader patches the MotorBus `gains`
## array live (apply_patch set_param).  The panel OWNS the faders (write-only);
## telemetry (activity/contribution/output/GR) is read each frame.
##
## Auto-hides when no MotorBus is in the graph.  Added by hud.gd (Cell), like
## motor_fader_meter.  Construct, call set_brain(b).

const _GAIN_MAX: float = 4.0
const _COL_ACTIVE   := Color(0.30, 0.95, 0.45)
const _COL_IDLE     := Color(0.35, 0.35, 0.35)
const _COL_TITLE    := Color(0.85, 0.92, 1.0, 1)
const _COL_TEXT     := Color(0.92, 0.92, 0.92, 1)

var _brain: OgmaBrain = null
var _bus_id: String = ""
var _built: bool = false
var _names: Array = []
var _user_gains: Array = []           # panel-authoritative fader values

var _root: VBoxContainer = null
var _rows_box: VBoxContainer = null
var _sliders: Array = []
var _val_labels: Array = []
var _act_dots: Array = []
var _contrib_labels: Array = []
var _out_label: Label = null

func set_brain(b: OgmaBrain) -> void:
	_brain = b

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_PASS
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	# Top-right, below the fader meter (~y=90) so they don't overlap.
	panel.position = Vector2(-340, 96)
	panel.custom_minimum_size = Vector2(330, 0)
	add_child(panel)
	_root = VBoxContainer.new()
	_root.add_theme_constant_override("separation", 3)
	panel.add_child(_root)

	var title := Label.new()
	title.text = "MOTOR BUS  (faders → compressor)"
	title.add_theme_font_size_override("font_size", 12)
	title.add_theme_color_override("font_color", _COL_TITLE)
	_root.add_child(title)

	_rows_box = VBoxContainer.new()
	_rows_box.add_theme_constant_override("separation", 2)
	_root.add_child(_rows_box)

	_out_label = Label.new()
	_out_label.add_theme_font_size_override("font_size", 11)
	_out_label.add_theme_color_override("font_color", _COL_TEXT)
	_out_label.text = "—"
	_root.add_child(_out_label)

	visible = false

func _process(_delta: float) -> void:
	if _brain == null or not _brain.is_brain_ready():
		if visible: visible = false
		return
	var st: Dictionary = _brain.get_motor_bus_state()
	if st.is_empty():
		if visible: visible = false
		return
	visible = true
	var names: Array = st.get("names", [])
	if not _built or names != _names:
		_rebuild(st)
	_update(st)

func _rebuild(st: Dictionary) -> void:
	# Tear down existing rows.
	for c in _rows_box.get_children():
		c.queue_free()
	_sliders.clear(); _val_labels.clear(); _act_dots.clear(); _contrib_labels.clear()

	_bus_id = String(st.get("id", ""))
	_names = (st.get("names", []) as Array).duplicate()
	_user_gains = (st.get("gains", []) as Array).duplicate()

	for i in range(_names.size()):
		var row := HBoxContainer.new()
		row.add_theme_constant_override("separation", 4)

		var dot := ColorRect.new()
		dot.custom_minimum_size = Vector2(9, 9)
		dot.color = _COL_IDLE
		# vertical-center the dot
		var dot_wrap := CenterContainer.new()
		dot_wrap.add_child(dot)
		row.add_child(dot_wrap)

		var nl := Label.new()
		nl.text = str(_names[i])
		nl.custom_minimum_size.x = 64
		nl.add_theme_font_size_override("font_size", 11)
		nl.add_theme_color_override("font_color", _COL_TEXT)
		row.add_child(nl)

		var sl := HSlider.new()
		sl.min_value = 0.0
		sl.max_value = _GAIN_MAX
		sl.step = 0.05
		sl.value = float(_user_gains[i])
		sl.custom_minimum_size.x = 110
		sl.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		sl.value_changed.connect(_on_gain_changed.bind(i))
		row.add_child(sl)

		var vl := Label.new()
		vl.text = "%.2f" % float(_user_gains[i])
		vl.custom_minimum_size.x = 34
		vl.add_theme_font_size_override("font_size", 11)
		vl.add_theme_color_override("font_color", _COL_TEXT)
		row.add_child(vl)

		var cl := Label.new()
		cl.add_theme_font_size_override("font_size", 10)
		cl.add_theme_color_override("font_color", Color(0.7, 0.78, 0.9, 1))
		row.add_child(cl)

		_rows_box.add_child(row)
		_sliders.append(sl)
		_val_labels.append(vl)
		_act_dots.append(dot)
		_contrib_labels.append(cl)

	_built = true

func _on_gain_changed(value: float, idx: int) -> void:
	if idx < 0 or idx >= _user_gains.size():
		return
	_user_gains[idx] = value
	_val_labels[idx].text = "%.2f" % value
	_patch_gains()

func _patch_gains() -> void:
	if _brain == null or _bus_id == "":
		return
	var op := {
		"op": "set_param",
		"id": _bus_id,
		"key": "gains",
		"value": _user_gains,
	}
	var result: Dictionary = _brain.apply_patch(op)
	if not bool(result.get("success", false)):
		push_warning("MotorBusPanel: gains patch failed — %s" % String(result.get("error", "?")))

func _update(st: Dictionary) -> void:
	var act: Array = st.get("active", [])
	var cl: Array = st.get("contrib_l", [])
	var cr: Array = st.get("contrib_r", [])
	var n: int = mini(_sliders.size(), act.size())
	for i in range(n):
		_act_dots[i].color = _COL_ACTIVE if bool(act[i]) else _COL_IDLE
		if i < cl.size() and i < cr.size():
			_contrib_labels[i].text = "L%+.1f R%+.1f" % [float(cl[i]), float(cr[i])]
	_out_label.text = "OUT  L%+.2f  R%+.2f   |  GR %d%%" % [
		float(st.get("out_l", 0.0)),
		float(st.get("out_r", 0.0)),
		int(round(float(st.get("gr", 0.0)) * 100.0)),
	]
