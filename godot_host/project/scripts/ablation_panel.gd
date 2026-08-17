extends Control
## Physical-ablation bench — break the robot, live, and watch what the brain does about it.
##
## WHY THIS EXISTS.  Every perturbation this project could run until 2026-08-04 was a
## TELEPORT: it relocates the body without changing what the body IS, so the world explains
## the disturbance.  An ablation leaves the world untouched.  Nothing is signalled to the
## brain — no reset event, no flag, no sensor that says "your knee is gone" — so the only
## way it can notice is its own rising prediction error.  That is the (a) "inferred, not
## oracle" bar in `docs/brain_building_doctrine.md` §2, and it is the sharpest form of the
## (d) perturbation test the picrawler plan has always named ("degrade a leg -> the gait
## re-coordinates").
##
## WHAT THE THREE SERVO FAILURES ARE, and why they are not interchangeable:
##   DEAD    unpowered — motor released, joint free-swings and is back-driveable. The limb
##           flops and gravity moves it. This is a servo that lost power.
##   SEIZED  jammed gearbox — velocity target 0 at FULL torque. The joint fights any motion
##           and holds wherever it happened to be. A rigid strut, not a limp one.
##   WEAK    browning out — the commanded motion is attenuated but still live. Slider below.
## DETACH is not a servo failure at all: it breaks the joint clean through, so everything
## DISTAL comes off.  hip1 = the whole leg; hip2 = upper + shank; knee = the shank and foot.
## Reversible, so an A/B runs without a restart — the limb goes back in the pose it broke in.
##
## ⚠ ONE MEASUREMENT NOTE, because it bit the campaign already.  A detached segment is frozen
## in world space while the chassis walks away, so its measured joint angle would diverge into
## garbage.  A real robot's servo keeps reporting its own encoder after the limb below it snaps
## off, so the body HOLDS the last honest reading instead.  See picrawler_body.gd `_abl_*`.
##
## ⚠ AND: torque is NOT a useful damage axis on this body.  A torque-ceiling lesion was built
## first and measured not to perturb at all (`tq_sat` = 0.009 — the servos are saturated under
## 1 % of the time, so cutting the ceiling removes headroom nothing was using). That is why
## these are command/constraint failures rather than strength failures. Ledger, 2026-08-04.
##
## Methodology: SCENE-only (headless runs never instance it; use OGMA_PICRAWLER_ABLATE
## instead, same semantics).  No state of its own — pure remote control of the body's
## `abl_*` API, so the panel can never disagree with what the physics is doing.

const _BODY_CANDIDATES: Array = ["Picrawler", "Quadruped"]
const _LEGS: Array = ["FL", "FR", "RL", "RR"]
const _JOINTS: Array = ["hip1", "hip2", "knee"]
# Kind index -> [button text, colour].  Cycled by clicking the button.
const _KINDS: Array = [
	["ok",     Color(0.55, 0.62, 0.55, 1.0)],
	["dead",   Color(1.00, 0.75, 0.30, 1.0)],
	["seized", Color(1.00, 0.45, 0.45, 1.0)],
	["weak",   Color(0.60, 0.80, 1.00, 1.0)],
]

var _body: Node = null
var _content_vb: VBoxContainer = null
var _status: Label = null
var _minimise_btn: Button = null
var _kind_btns: Array = []        # 12, index = leg*3 + joint
var _det_btns: Array = []         # 12
var _weak_label: Label = null
var _is_minimised: bool = true    # starts collapsed: destructive, so opt-in
var _full_anchor_bottom: float = -1.0
var _full_offset_bottom: float = 0.0
const _COLLAPSED_HEIGHT: float = 38.0
# 2026-08-12 (operator QoL) — the distress bar lives at bottom-left y∈[-28,-8]
# (picrawler_body._update_distress_hud), and a -12 bottom edge sat on top of it.
const _BOTTOM_MARGIN: float = 48.0


func _ready() -> void:
	# Bottom-LEFT, raised clear of the distress bar.  The HUD owns top-left and
	# MOTOR-EPM owns the right edge.
	anchor_left = 0.0
	anchor_top = 1.0
	anchor_right = 0.0
	anchor_bottom = 1.0
	offset_left = 12.0
	offset_top = -_BOTTOM_MARGIN - 360.0
	offset_right = 12.0 + 430.0
	offset_bottom = -_BOTTOM_MARGIN
	mouse_filter = Control.MOUSE_FILTER_PASS

	var root := PanelContainer.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var bg := StyleBoxFlat.new()
	bg.bg_color = Color(0.08, 0.05, 0.05, 0.86)
	bg.border_color = Color(0.75, 0.45, 0.45, 0.9)
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
	title.text = "ABLATION  [K]  — break the robot, live"
	title.add_theme_font_size_override("font_size", 13)
	title.add_theme_color_override("font_color", Color(1.0, 0.85, 0.85, 1.0))
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(title)
	_minimise_btn = Button.new()
	_minimise_btn.text = "▲"
	_minimise_btn.custom_minimum_size = Vector2(28, 0)
	_minimise_btn.pressed.connect(_on_minimise)
	header.add_child(_minimise_btn)

	_content_vb = VBoxContainer.new()
	_content_vb.add_theme_constant_override("separation", 3)
	_content_vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	outer.add_child(_content_vb)

	var legend := Label.new()
	legend.add_theme_font_size_override("font_size", 10)
	legend.add_theme_color_override("font_color", Color(0.72, 0.68, 0.68, 1.0))
	legend.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	legend.text = ("click a servo to cycle  ok → dead (unpowered, flops) → seized (jammed, "
		+ "rigid) → weak (attenuated).   ✂ breaks the joint clean through and removes "
		+ "everything below it.   Nothing is told to the brain.")
	_content_vb.add_child(legend)

	# Column header
	var head := HBoxContainer.new()
	head.add_theme_constant_override("separation", 4)
	_content_vb.add_child(head)
	head.add_child(_mk_label("", 30))
	for jn in _JOINTS:
		head.add_child(_mk_label(jn, 74, Color(0.85, 0.85, 0.85, 1.0)))
		head.add_child(_mk_label("✂", 26, Color(0.85, 0.6, 0.6, 1.0)))

	_kind_btns.resize(12)
	_det_btns.resize(12)
	for leg in range(4):
		var row := HBoxContainer.new()
		row.add_theme_constant_override("separation", 4)
		_content_vb.add_child(row)
		row.add_child(_mk_label(_LEGS[leg], 30, Color(0.9, 0.9, 0.75, 1.0)))
		for joint in range(3):
			var k: int = leg * 3 + joint
			var kb := Button.new()
			kb.text = "ok"
			kb.custom_minimum_size = Vector2(74, 22)
			kb.add_theme_font_size_override("font_size", 11)
			kb.pressed.connect(_on_kind.bind(leg, joint))
			row.add_child(kb)
			_kind_btns[k] = kb
			var db := Button.new()
			db.text = "✂"
			db.tooltip_text = "detach at %s %s — removes everything distal" % [_LEGS[leg], _JOINTS[joint]]
			db.custom_minimum_size = Vector2(26, 22)
			db.add_theme_font_size_override("font_size", 11)
			db.pressed.connect(_on_detach.bind(leg, joint))
			row.add_child(db)
			_det_btns[k] = db

	# weak-scale slider
	var wrow := HBoxContainer.new()
	wrow.add_theme_constant_override("separation", 6)
	_content_vb.add_child(wrow)
	wrow.add_child(_mk_label("weak scale", 88, Color(0.6, 0.8, 1.0, 1.0)))
	var sl := HSlider.new()
	sl.min_value = 0.0
	sl.max_value = 1.0
	sl.step = 0.05
	sl.value = 0.3
	sl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sl.custom_minimum_size = Vector2(150, 18)
	sl.value_changed.connect(_on_weak)
	wrow.add_child(sl)
	_weak_label = _mk_label("0.30", 44)
	wrow.add_child(_weak_label)

	# ---- friction, because "can a broken robot drag itself?" is a friction question -------
	# The chassis, the coxa/femur linkages and the feet were ALL sharing one mu = 1.5
	# material.  A rubber foot pad and a plastic shell are not the same surface, and the
	# difference only shows up once the robot is damaged enough to be lying on something
	# other than its feet.
	# ⚠ MEASURED, so you know what to expect: in the headless poses reachable with two dead
	# rear legs the chassis stays ~57 mm off the floor and the FEET are what rests on it, so
	# neither slider binds there and a 30x sweep of both was byte-identical.  They bind in
	# the poses where the belly is genuinely down -- which is what you were seeing.  Watch
	# the belly readout, not just the robot.
	for fs in [{"label": "chassis friction", "setter": "set_chassis_friction",
				"get": "chassis_friction", "max": 1.5, "col": Color(1.0, 0.85, 0.6, 1.0)},
			   {"label": "limb friction (coxa+femur)", "setter": "set_limb_friction",
				"get": "limb_friction", "max": 3.0, "col": Color(0.8, 0.9, 1.0, 1.0)}]:
		var frow := HBoxContainer.new()
		frow.add_theme_constant_override("separation", 6)
		_content_vb.add_child(frow)
		frow.add_child(_mk_label(fs["label"], 150, fs["col"]))
		var fsl := HSlider.new()
		fsl.min_value = 0.0
		fsl.max_value = fs["max"]
		fsl.step = 0.05
		fsl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		fsl.custom_minimum_size = Vector2(120, 18)
		var b0: Node = _find_body()
		fsl.value = float(b0.get(fs["get"])) if b0 != null else 0.3
		var vlab := _mk_label("%.2f" % fsl.value, 44)
		fsl.value_changed.connect(_on_friction.bind(fs["setter"], vlab))
		frow.add_child(fsl)
		frow.add_child(vlab)
	var feet_note := Label.new()
	feet_note.add_theme_font_size_override("font_size", 10)
	feet_note.add_theme_color_override("font_color", Color(0.65, 0.72, 0.65, 1.0))
	feet_note.text = "   feet stay at mu 1.5 deliberately — they should grip the most"
	_content_vb.add_child(feet_note)

	var brow := HBoxContainer.new()
	brow.add_theme_constant_override("separation", 6)
	_content_vb.add_child(brow)
	var clr := Button.new()
	clr.text = "REPAIR ALL"
	clr.pressed.connect(_on_clear)
	brow.add_child(clr)
	var kill := Button.new()
	kill.text = "kill FL knee"
	kill.tooltip_text = "the exact (d)-test lesion from the 2026-08-04 ledger entry"
	kill.pressed.connect(_on_preset_knee)
	brow.add_child(kill)
	var amp := Button.new()
	amp.text = "amputate FL"
	amp.tooltip_text = "detach the whole front-left leg at hip1"
	amp.pressed.connect(_on_preset_amputate)
	brow.add_child(amp)

	_status = Label.new()
	_status.add_theme_font_size_override("font_size", 10)
	_status.add_theme_color_override("font_color", Color(0.75, 0.72, 0.72, 1.0))
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_status.text = "waiting for body…"
	_content_vb.add_child(_status)

	_full_anchor_bottom = anchor_bottom
	_full_offset_bottom = offset_bottom
	_apply_minimised()


func _mk_label(txt: String, w: float, col: Color = Color(0.8, 0.8, 0.8, 1.0)) -> Label:
	var l := Label.new()
	l.text = txt
	l.custom_minimum_size = Vector2(w, 0)
	l.add_theme_font_size_override("font_size", 11)
	l.add_theme_color_override("font_color", col)
	return l


func _find_body() -> Node:
	if is_instance_valid(_body):
		return _body
	for nm in _BODY_CANDIDATES:
		var n: Node = get_tree().get_root().find_child(nm, true, false)
		if n != null and n.has_method("abl_set_kind"):
			_body = n
			return _body
	return null


func _on_kind(leg: int, joint: int) -> void:
	var b: Node = _find_body()
	if b == null:
		return
	# Cycling through a DETACHED joint is meaningless — there is no servo left to fail.
	if b.abl_is_detached(leg, joint):
		_status.text = "%s %s is detached — nothing there to fail. Re-attach it first." % [
			_LEGS[leg], _JOINTS[joint]]
		return
	b.abl_set_kind(leg, joint, (b.abl_kind_of(leg, joint) + 1) % _KINDS.size())
	_refresh()


func _on_detach(leg: int, joint: int) -> void:
	var b: Node = _find_body()
	if b == null:
		return
	b.abl_set_detached(leg, joint, not b.abl_is_detached(leg, joint))
	_refresh()


func _on_weak(v: float) -> void:
	var b: Node = _find_body()
	if b != null:
		b.abl_set_weak_scale(v)
	_weak_label.text = "%.2f" % v


func _on_friction(v: float, setter: String, lab: Label) -> void:
	var b: Node = _find_body()
	if b != null and b.has_method(setter):
		b.call(setter, v)
	lab.text = "%.2f" % v


func _on_clear() -> void:
	var b: Node = _find_body()
	if b != null:
		b.abl_clear()
	_refresh()


func _on_preset_knee() -> void:
	var b: Node = _find_body()
	if b != null:
		b.abl_set_kind(0, 2, 1)   # FL knee -> dead
	_refresh()


func _on_preset_amputate() -> void:
	var b: Node = _find_body()
	if b != null:
		b.abl_set_detached(0, 0, true)
	_refresh()


func _refresh() -> void:
	var b: Node = _find_body()
	if b == null:
		_status.text = "waiting for body…"
		return
	var broken: int = 0
	for leg in range(4):
		for joint in range(3):
			var k: int = leg * 3 + joint
			var det: bool = b.abl_is_detached(leg, joint)
			var kind: int = b.abl_kind_of(leg, joint)
			var kb: Button = _kind_btns[k]
			if det:
				kb.text = "—"
				kb.add_theme_color_override("font_color", Color(0.45, 0.45, 0.45, 1.0))
			else:
				kb.text = _KINDS[kind][0]
				kb.add_theme_color_override("font_color", _KINDS[kind][1])
			_det_btns[k].add_theme_color_override("font_color",
				Color(1.0, 0.35, 0.35, 1.0) if det else Color(0.6, 0.6, 0.6, 1.0))
			if det or kind != 0:
				broken += 1
	if broken == 0:
		_status.text = ("intact.  The lever this bench was built for is NULL on a healthy "
			+ "body — break something before expecting a difference between arms.")
	else:
		_status.text = ("%d/12 joints ablated.  Watch the footfall raster and plv_w: the "
			+ "question is whether the SURVIVING legs keep oscillating, not whether the "
			+ "gait looks the same.") % broken


func _process(_dt: float) -> void:
	if not _is_minimised and Engine.get_process_frames() % 15 == 0:
		_refresh()


func _on_minimise() -> void:
	_is_minimised = not _is_minimised
	_apply_minimised()


func _apply_minimised() -> void:
	_content_vb.visible = not _is_minimised
	_minimise_btn.text = "▲" if _is_minimised else "▼"
	if _is_minimised:
		offset_top = -_BOTTOM_MARGIN - _COLLAPSED_HEIGHT
	else:
		offset_top = -_BOTTOM_MARGIN - 360.0
