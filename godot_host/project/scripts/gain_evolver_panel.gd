extends Control
## GAIN EVOLVER panel — PART IV's operator instrument.  [U] toggles.
##
## The charter requires the operator to be able to WATCH IT SEARCH
## (adaptive_gains_substrate_plan.md §2 "instruments first"): the live vector,
## incumbent/candidate criterion scores with the per-term breakdown, the
## accept/revert history, and the σ anneal — all here, at ~2 Hz.
##
## OWNERSHIP CONTRACT (printed on the panel):
##   σ = 0  ⇒ the evolver is a SILENT OBSERVER — hand-tune gains on the [M]
##   MotorEpmPanel sliders as usual; the criterion trace keeps scoring YOUR
##   point.  On σ-resume the evolver republishes ITS incumbent, overwriting
##   hand tunes — press ADOPT first to hand it your current point
##   (writes motor_epm's live gain values into incumbent_override).
##
## Gain scope labels: the rear trio acts on the REAR PAIR (cfg rl/rr — front/
## rear is NOT L↔R mirrored; the mirror warning applies to left/right only);
## the other five are body-wide.
##
## Writes BRAIN params via brain.apply_patch (HotMutable); SCENE-only
## (headless runs never instance it).  Boots HIDDEN — [U] unhides.
## Anchored top-right like [M]/[O]; open one bench at a time.

const _EVOLVER_ID: String = "gain_evolver"
const _MOTOR_ID: String = "motor_epm"
const _PHASE_NAMES: Array = ["WARMUP", "INCUMBENT", "CANDIDATE"]
# gain key → short scope note for the row label
const _SCOPE: Dictionary = {
	"rear_land_gain": "rear pair",
	"rear_knee_plant": "rear pair",
	"rear_push_ext": "rear pair",
	"amp_target": "body",
	"height_homeo_gain": "body",
	"postural_gain": "body",
	"coupling_gain": "body",
	"plan_gain": "body",
}

var _status: Label = null
var _score_line: Label = null
var _vec_line: Label = null
var _sigma_slider: HSlider = null
var _sigma_label: Label = null
var _gain_keys: Array = []
var _gain_min: Array = []
var _gain_max: Array = []
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


func _set_param(module: String, key: String, v) -> void:
	var br = _brain()
	if br == null:
		return
	var res = br.apply_patch({"op": "set_param", "id": module, "key": key, "value": v})
	var ok: bool = typeof(res) == TYPE_DICTIONARY and bool(res.get("success", false))
	if _status != null:
		_status.text = "%s.%s = %s%s" % [module, key, str(v),
			("" if ok else "  (FAILED: " + str(res.get("error", "")) + ")")]


func _ready() -> void:
	anchor_left = 1.0
	anchor_top = 0.0
	anchor_right = 1.0
	anchor_bottom = 0.0
	offset_left = -12.0 - 360.0
	offset_top = 12.0
	offset_right = -12.0
	offset_bottom = 12.0 + 470.0
	mouse_filter = Control.MOUSE_FILTER_PASS
	visible = false           # bench: opt-in via [U]

	var root := PanelContainer.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var bg := StyleBoxFlat.new()
	bg.bg_color = Color(0.04, 0.07, 0.06, 0.86)
	bg.border_color = Color(0.45, 0.85, 0.65, 0.9)
	bg.set_border_width_all(1)
	bg.set_corner_radius_all(4)
	root.add_theme_stylebox_override("panel", bg)
	add_child(root)

	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 3)
	root.add_child(vb)

	var hdr := Label.new()
	hdr.text = "GAIN EVOLVER  [U]  — PART IV lifetime (1+1)-ES"
	hdr.add_theme_font_size_override("font_size", 13)
	hdr.add_theme_color_override("font_color", Color(0.7, 1.0, 0.85, 1.0))
	vb.add_child(hdr)

	var contract := Label.new()
	contract.text = ("σ=0: OBSERVER — hand-tune on [M]; the criterion still scores you.\n" +
		"σ-resume republishes the EVOLVER's incumbent — press ADOPT first\n" +
		"to hand it your current [M] point.")
	contract.add_theme_font_size_override("font_size", 10)
	contract.add_theme_color_override("font_color", Color(0.85, 0.8, 0.6, 1.0))
	vb.add_child(contract)

	var srow := HBoxContainer.new()
	var sl := Label.new()
	sl.text = "mutation σ (0 = observer)"
	sl.custom_minimum_size = Vector2(150, 0)
	sl.add_theme_font_size_override("font_size", 11)
	srow.add_child(sl)
	_sigma_slider = HSlider.new()
	_sigma_slider.min_value = 0.0
	_sigma_slider.max_value = 0.5
	_sigma_slider.step = 0.005
	_sigma_slider.custom_minimum_size = Vector2(120, 0)
	_sigma_slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	srow.add_child(_sigma_slider)
	_sigma_label = Label.new()
	_sigma_label.text = "—"
	_sigma_label.custom_minimum_size = Vector2(44, 0)
	_sigma_label.add_theme_font_size_override("font_size", 11)
	srow.add_child(_sigma_label)
	_sigma_slider.value_changed.connect(func(v: float) -> void:
		_sigma_label.text = "%.3f" % v
		_set_param(_EVOLVER_ID, "mutation_sigma", v))
	vb.add_child(srow)

	var brow := HBoxContainer.new()
	var adopt := Button.new()
	adopt.text = "ADOPT [M] point → incumbent"
	adopt.add_theme_font_size_override("font_size", 11)
	adopt.tooltip_text = "Reads motor_epm's LIVE values for the evolver's gain_keys and writes them as incumbent_override (clamped to bounds). Do this BEFORE raising σ if you hand-tuned."
	adopt.pressed.connect(func() -> void:
		_adopt_current())
	brow.add_child(adopt)
	var pause := Button.new()
	pause.text = "σ → 0"
	pause.add_theme_font_size_override("font_size", 11)
	pause.pressed.connect(func() -> void:
		_sigma_slider.set_value_no_signal(0.0)
		_sigma_label.text = "0.000"
		_set_param(_EVOLVER_ID, "mutation_sigma", 0.0))
	brow.add_child(pause)
	vb.add_child(brow)

	_score_line = Label.new()
	_score_line.add_theme_font_size_override("font_size", 10)
	_score_line.add_theme_color_override("font_color", Color(0.8, 0.85, 0.8, 1.0))
	_score_line.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_score_line.text = "criterion: —"
	vb.add_child(_score_line)

	_vec_line = Label.new()
	_vec_line.add_theme_font_size_override("font_size", 10)
	_vec_line.add_theme_color_override("font_color", Color(0.75, 0.9, 0.8, 1.0))
	_vec_line.text = "vector: —"
	vb.add_child(_vec_line)

	_status = Label.new()
	_status.add_theme_font_size_override("font_size", 10)
	_status.add_theme_color_override("font_color", Color(0.72, 0.72, 0.75, 1.0))
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_status.text = "waiting for brain…  (is gain_evolver in this config?)"
	vb.add_child(_status)


func _motor_live_values() -> Dictionary:
	# motor_epm's CURRENT values for the evolver's keys, via the schema's
	# current_value (same read the [M] panel syncs from).
	var out: Dictionary = {}
	var br = _brain()
	if br == null or not br.has_method("get_module_param_schema"):
		return out
	var schema = br.get_module_param_schema(_MOTOR_ID)
	if typeof(schema) != TYPE_ARRAY:
		return out
	for entry in schema:
		if typeof(entry) == TYPE_DICTIONARY and entry.has("key"):
			var cur = entry.get("current_value", null)
			if cur is float or cur is int:
				out[str(entry["key"])] = float(cur)
	return out


func _adopt_current() -> void:
	if _gain_keys.is_empty():
		if _status != null:
			_status.text = "ADOPT: gain_keys not synced yet"
		return
	var live: Dictionary = _motor_live_values()
	var vec: Array = []
	for k in _gain_keys:
		if not live.has(k):
			if _status != null:
				_status.text = "ADOPT failed: motor_epm has no '%s'" % k
			return
		vec.append(float(live[k]))
	_set_param(_EVOLVER_ID, "incumbent_override", vec)


func _sync_from_schema() -> bool:
	var br = _brain()
	if br == null or not br.has_method("get_module_param_schema"):
		return false
	var schema = br.get_module_param_schema(_EVOLVER_ID)
	if typeof(schema) != TYPE_ARRAY or schema.is_empty():
		return false
	var sigma: float = 0.0
	for entry in schema:
		if typeof(entry) != TYPE_DICTIONARY or not entry.has("key"):
			continue
		var key: String = str(entry["key"])
		var cur = entry.get("current_value", entry.get("default_value", null))
		# vector params cross the bridge as Packed*Array, not Array
		if key == "gain_keys" and (cur is Array or cur is PackedStringArray):
			_gain_keys = Array(cur)
		elif key == "gain_min" and (cur is Array or cur is PackedFloat64Array):
			_gain_min = Array(cur)
		elif key == "gain_max" and (cur is Array or cur is PackedFloat64Array):
			_gain_max = Array(cur)
		elif key == "mutation_sigma" and (cur is float or cur is int):
			sigma = float(cur)
	if _gain_keys.is_empty():
		return false
	_sigma_slider.set_value_no_signal(sigma)
	_sigma_label.text = "%.3f" % sigma
	_status.text = "live — %d gains under evolution" % _gain_keys.size()
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
	if br == null:
		return
	var ge: Dictionary = {}
	if br.has_method("get_module_metrics"):
		ge = br.get_module_metrics().get(_EVOLVER_ID, {})
	if ge.is_empty():
		_score_line.text = "criterion: (no gain_evolver metrics — wrong config?)"
		return
	var ph: int = clampi(int(ge.get("phase", 0)), 0, 2)
	var wt: int = int(ge.get("win_tick", 0))
	_score_line.text = ("gen %d  %s t%d   J_inc %.4f  J_cand %.4f   acc %d / rev %d   σ %.3f\n" +
		"terms: falls %.0f  tilt %.5f  distress %.3f  unloaded %.3f  flow %.3f  minload %.3f") % [
		int(ge.get("generation", 0)), _PHASE_NAMES[ph], wt,
		float(ge.get("J_inc", -1.0)), float(ge.get("J_cand", -1.0)),
		int(ge.get("accepts", 0)), int(ge.get("reverts", 0)), float(ge.get("sigma", 0.0)),
		float(ge.get("falls", 0.0)), float(ge.get("tilt_var", 0.0)),
		float(ge.get("distress_duty", 0.0)), float(ge.get("unloaded_mean", 0.0)),
		float(ge.get("flow_term", 0.0)), float(ge.get("loaded_min", 0.0))]
	# Full vectors + accept history from the (2 Hz — fine at this rate) snapshot.
	var txt: String = ""
	if br.has_method("get_module_snapshot"):
		var ss = JSON.parse_string(str(br.get_module_snapshot(_EVOLVER_ID)))
		if ss is Dictionary and ss.has("module"):
			var mod: Dictionary = ss["module"]
			var inc: Array = mod.get("incumbent", [])
			var cand: Array = mod.get("candidate", [])
			for i in range(_gain_keys.size()):
				var k: String = _gain_keys[i]
				var iv: float = float(inc[i]) if i < inc.size() else 0.0
				var cv: float = float(cand[i]) if i < cand.size() else 0.0
				var lo: float = float(_gain_min[i]) if i < _gain_min.size() else 0.0
				var hi: float = float(_gain_max[i]) if i < _gain_max.size() else 0.0
				txt += "%-18s %-9s inc %.3f  cand %.3f  [%.2f–%.2f]\n" % [
					k, "(" + str(_SCOPE.get(k, "?")) + ")", iv, cv, lo, hi]
			txt += "history: %s" % str(mod.get("accept_log", ""))
	_vec_line.text = txt
