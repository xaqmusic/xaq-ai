extends Node3D
## Bench dashboard — the laptop half of pi_host/PROTOCOL.md.
##
## Talks to ogma_benchd on the Pi through BenchClient (REQ verbs + CONFLATE'd telemetry).
## It speaks the CALIBRATION verb set only: there is no path from here to starting the
## brain (port doc SPEC §1.1) and none may be added.  Everything a servo does goes
## through the daemon's ServoDriver (clamp · slew · watchdog → pulse 0 · time-at-limit).
##
## The 3-D body below is the sim picrawler in calibrate mode, FK-written from the
## COMMANDED pulses — hobby servos report nothing back, so it is labelled as such.
## Sim leg names are anatomically mirrored (sim "fl" = physical front-RIGHT); every
## row shows both names, with the sim colour as the cross-check (SPEC §5.2).

const REP_PORT := 5590
const PUB_PORT := 5591
# ogma_host's video PUB = its inspector control port + 2 (7400 -> 7402).
const VIDEO_PORT := 7402
const PING_S   := 0.3
const SEND_THROTTLE_S := 0.05
const US_PER_RAD := 636.6            # 500–2500 µs ≙ ±π/2
const VBAT_MIN := 6.0
const VBAT_MAX := 8.4

# physical → sim (the mirror), sim → index, sim colours (keyed by SIM leg, as in
# picrawler_servo_panel.gd: fl red · fr green · rl blue · rr yellow).
const MIRROR := {"FL": "fr", "FR": "fl", "RL": "rr", "RR": "rl"}
const SIM_INDEX := {"fl": 0, "fr": 1, "rl": 2, "rr": 3}
const SIM_COLORS := [Color(0.95, 0.20, 0.20), Color(0.20, 0.85, 0.20), Color(0.30, 0.50, 0.95), Color(0.95, 0.85, 0.20)]
const PHYS_LONG := {"FL": "front-left", "FR": "front-right", "RL": "rear-left", "RR": "rear-right"}
const PHYS_OPTIONS := ["?", "FL", "FR", "RL", "RR"]
const JOINT_OPTIONS := ["hip1", "hip2", "knee"]

@onready var _client: Node = $Client
@onready var _body: Node = $Picrawler
@onready var _ui: Control = $HUD/UI

var _connected := false
var _link_fail := 0
var _tele: Dictionary = {}
var _tele_ms: int = -1
var _rows: Array = []
var _pending: Dictionary = {}       # ch -> us awaiting a throttled servo.set
var _ping_acc := 0.0
var _avg_sum: Dictionary = {}          # 1 s box-car of the numeric telemetry
var _avg_n := 0
var _avg_started := -1
var _avg: Dictionary = {}              # last completed 1 s means
const UI_FONT := 12
const TOP_H := 84                      # top bar height the side panels hang from
var _cal_content: Control
var _cal_min_btn: Button
var _cal_min := false
var _tele_content: Control
var _tele_min_btn: Button
var _tele_min := false
var _left_panel: PanelContainer
var _right_panel: PanelContainer
var _pose_name: LineEdit
var _pose_list: OptionButton
var _send_acc := 0.0
var _status_msg := ""

# --- widgets we update -------------------------------------------------------------
var _host_edit: LineEdit
var _link_lbl: Label
var _mode_lbl: Label
var _body_lbl: Label
var _tele_lbls: Dictionary = {}
var _check_lbls: Dictionary = {}
var _status_lbl: Label
var _widen_lbl: Label
var _tick_meter: Control
var _video: Node                     # VideoClient — receive-only, see VideoClient.hpp
var _view_tex: TextureRect           # what the CAMERA sees
var _brain_tex: TextureRect          # what the BRAIN sees (the encoder's actual input)
var _video_lbl: Label


func _ready() -> void:
	_build_ui()
	# Put the sim body in calibrate mode: chassis suspended, legs FK-written from
	# servo_targets each physics tick (picrawler_body.gd, the [C] path).  Deferred so
	# the body's own _ready has run.  Sign/origin are applied HERE (in µs), so the
	# body's own sign/origin arrays are neutralised.
	call_deferred("_enter_calibrate_mode")
	_refresh_all_rows()


func _enter_calibrate_mode() -> void:
	# The body reads hotkeys in _input() — BEFORE any text box sees the event — so typing
	# "rescue" into the pose field toggled ragdoll (R) and calibrate (C) on a suspended body
	# and crashed Godot (2026-08-29, log: "ragdoll_mode = true", "calibrate_mode = false").
	# The bench has no use for the body's keys: the dashboard owns the keyboard here.
	_body.set_process_input(false)
	_body.set_process_unhandled_input(false)
	_body.set_process_unhandled_key_input(false)
	if _body == null: return
	_body.set("_calibrate_mode", true)
	_body.set("_calibrate_step", 0)
	_body.set("_pending_chassis_freeze", 1)
	var signs: Array = _body.get("servo_signs")
	var origins: Array = _body.get("servo_origins")
	for k in range(min(12, signs.size())):
		signs[k] = 1.0
	for k in range(min(12, origins.size())):
		origins[k] = 0.0


# ======================================================================================
# UI
# ======================================================================================
func _build_ui() -> void:
	_ui.mouse_filter = Control.MOUSE_FILTER_IGNORE      # the 3-D view keeps the middle
	# One theme for the whole panel: buttons, dropdowns and fields at the LABEL size
	# (Godot's default 16 px button text was what forced the calibration panel to scroll).
	var th := Theme.new()
	th.default_font_size = UI_FONT
	for cls in ["Button", "OptionButton", "LineEdit", "Label", "CheckBox"]:
		th.set_font_size("font_size", cls, UI_FONT)
	th.set_constant("separation", "HBoxContainer", 4)
	th.set_constant("separation", "VBoxContainer", 1)
	_ui.theme = th

	# ---- top bar: connect / disconnect / link / banners ------------------------------
	var top := PanelContainer.new()
	top.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	_ui.add_child(top)
	var topv := VBoxContainer.new(); top.add_child(topv)
	var row := HBoxContainer.new(); topv.add_child(row)
	row.add_child(_lbl("ogma_benchd @"))
	_host_edit = LineEdit.new(); _host_edit.text = "picrawler.local"; _host_edit.custom_minimum_size.x = 160
	row.add_child(_host_edit)
	row.add_child(_lbl(":%d / :%d" % [REP_PORT, PUB_PORT]))
	var cb := Button.new(); cb.text = "CONNECT"; cb.pressed.connect(_on_connect); row.add_child(cb)
	var db := Button.new(); db.text = "DISCONNECT"; db.pressed.connect(_on_disconnect); row.add_child(db)
	var lb := Button.new(); lb.text = "RESCUE POSE"; lb.pressed.connect(_on_limp); row.add_child(lb)
	lb.add_theme_color_override("font_color", Color(1, 0.8, 0.3))
	_link_lbl = _lbl("link: disconnected"); row.add_child(_link_lbl)
	var banner := HBoxContainer.new(); topv.add_child(banner)
	# --- poses: raw µs per channel, stored on the Pi (pi_host/calib/poses.json).  SAVE takes
	# the twelve sliders as they are; RECALL sets the sliders AND arms all twelve servos.
	var prow := HBoxContainer.new(); topv.add_child(prow)
	prow.add_child(_lbl("POSE"))
	_pose_name = LineEdit.new(); _pose_name.placeholder_text = "name (e.g. storage)"; _pose_name.custom_minimum_size.x = 160; prow.add_child(_pose_name)
	var psb := Button.new(); psb.text = "SAVE POSE"; psb.pressed.connect(_on_pose_save); prow.add_child(psb)
	_pose_list = OptionButton.new(); _pose_list.custom_minimum_size.x = 160; prow.add_child(_pose_list)
	var prb := Button.new(); prb.text = "RECALL (arms 12)"; prb.pressed.connect(_on_pose_recall); prow.add_child(prb)
	prb.add_theme_color_override("font_color", Color(1, 0.8, 0.3))
	var pdb := Button.new(); pdb.text = "delete"; pdb.pressed.connect(_on_pose_delete); prow.add_child(pdb)
	var prf := Button.new(); prf.text = "⟳"; prf.pressed.connect(_refresh_poses); prow.add_child(prf)
	_mode_lbl = _lbl("MODE: —", 15); banner.add_child(_mode_lbl)
	banner.add_child(_lbl("      "))
	_body_lbl = _lbl("BODY: —", 15); banner.add_child(_body_lbl)
	banner.add_child(_lbl("      "))
	_status_lbl = _lbl("", 13); _status_lbl.add_theme_color_override("font_color", Color(1, 0.6, 0.6))
	banner.add_child(_status_lbl)

	# ---- left: telemetry + boot self-check --------------------------------------------
	var left := PanelContainer.new(); _left_panel = left
	left.anchor_top = 0.0; left.anchor_bottom = 1.0; left.anchor_left = 0.0; left.anchor_right = 0.0
	left.offset_top = TOP_H; left.offset_left = 0; left.offset_right = 300; left.offset_bottom = -30
	_ui.add_child(left)
	var lmargin := _margin(); left.add_child(lmargin)
	var lroot := VBoxContainer.new(); lmargin.add_child(lroot)
	var lhdr := HBoxContainer.new(); lroot.add_child(lhdr)
	lhdr.add_child(_lbl("TELEMETRY  (the Pi's log is the record)", 13))
	var lspacer := Control.new(); lspacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL; lhdr.add_child(lspacer)
	_tele_min_btn = Button.new(); _tele_min_btn.text = "▼"; _tele_min_btn.custom_minimum_size.x = 26
	_tele_min_btn.pressed.connect(_on_tele_min); lhdr.add_child(_tele_min_btn)
	var lv := VBoxContainer.new(); lroot.add_child(lv); _tele_content = lv
	for key in ["vbat", "adc", "tick_hz", "cost", "cost_split", "mem", "overruns", "watchdog_trips", "deadman_ms_left", "armed_ch", "cal_ch", "age"]:
		var l := _lbl(key + ": —"); _tele_lbls[key] = l; lv.add_child(l)
		if key == "cost":
			_tick_meter = (load("res://scripts/tick_meter.gd") as Script).new()
			_tick_meter.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			lv.add_child(_tick_meter)
	lv.add_child(_lbl(" "))
	lv.add_child(_lbl("BOOT SELF-CHECK  (SPEC §6)", 13))
	for item in ["0x14 present", "Vbat plausible 6.0–8.4 V", "IMU WHO_AM_I = 0xEA", "INA219 ⟷ A4 agree", "ToF plausible", "FSR sum ≈ 1.0 BW"]:
		var l := _lbl("○ " + item); _check_lbls[item] = l; lv.add_child(l)
		l.add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))

	# ---- the camera, inside the telemetry panel so it minimises with the metrics ------
	# Lives here rather than over the 3-D view: that area already carries the body's own
	# meters and the COMMANDED-pose label, and a floating panel overlapped the distress bar.
	# Both planes side by side, because the PAIR is the diagnostic — a fault between the
	# sensor and the encoder shows up as the two disagreeing. (2026-09-01: the camera read
	# across frame boundaries for a week and looked like a working camera the whole time,
	# because nothing ever displayed what the encoder was actually handed.)
	lv.add_child(_lbl(" "))
	lv.add_child(_lbl("CAMERA  (ogma_host :%d — view only)" % VIDEO_PORT, 13))
	var vrow := HBoxContainer.new(); lv.add_child(vrow)
	var vcol := VBoxContainer.new(); vrow.add_child(vcol)
	vcol.add_child(_lbl("what the camera sees", 11))
	_view_tex = TextureRect.new()
	_view_tex.custom_minimum_size = Vector2(160, 120)
	_view_tex.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT
	_view_tex.texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST
	vcol.add_child(_view_tex)
	vrow.add_child(_lbl(" "))
	var bcol := VBoxContainer.new(); vrow.add_child(bcol)
	var blab := _lbl("the BRAIN sees", 11)
	blab.add_theme_color_override("font_color", Color(1, 0.85, 0.4))
	bcol.add_child(blab)
	_brain_tex = TextureRect.new()
	_brain_tex.custom_minimum_size = Vector2(96, 96)
	_brain_tex.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT
	_brain_tex.texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST   # show the pixels, do not smooth them
	bcol.add_child(_brain_tex)
	_video_lbl = _lbl("not connected", 11); lv.add_child(_video_lbl)

	# ---- bottom-centre: the honesty label on the 3-D view ----------------------------
	var cap := _lbl("3-D view = COMMANDED pose", 14)
	cap.set_anchors_and_offsets_preset(Control.PRESET_CENTER_TOP)
	cap.offset_top = TOP_H + 4; cap.offset_bottom = TOP_H + 26     # under the top bar, clear of the body's own meters
	cap.add_theme_color_override("font_color", Color(1, 0.85, 0.4))
	_ui.add_child(cap)

	# ---- right: calibration panel -------------------------------------------------------
	var right := PanelContainer.new(); _right_panel = right
	right.anchor_top = 0.0; right.anchor_bottom = 1.0; right.anchor_left = 1.0; right.anchor_right = 1.0
	right.offset_top = TOP_H; right.offset_left = -860; right.offset_right = 0; right.offset_bottom = -30
	_ui.add_child(right)
	var rmargin := _margin(); right.add_child(rmargin)
	var rroot := VBoxContainer.new(); rmargin.add_child(rroot)
	var hdr := HBoxContainer.new(); rroot.add_child(hdr)
	hdr.add_child(_lbl("SERVO CALIBRATION — robot on a stand", 13))
	var rv := VBoxContainer.new(); rroot.add_child(rv); _cal_content = rv
	rv.size_flags_vertical = Control.SIZE_EXPAND_FILL
	var sv := Button.new(); sv.text = "SAVE MAP"; sv.pressed.connect(_on_save_map); hdr.add_child(sv)
	var ld := Button.new(); ld.text = "LOAD MAP"; ld.pressed.connect(_on_load_map); hdr.add_child(ld)
	var ce := Button.new(); ce.text = "cal.end"; ce.pressed.connect(_on_cal_end); hdr.add_child(ce)
	_widen_lbl = _lbl("widened: none"); hdr.add_child(_widen_lbl)
	var spacer := Control.new(); spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL; hdr.add_child(spacer)
	_cal_min_btn = Button.new(); _cal_min_btn.text = "▼"; _cal_min_btn.custom_minimum_size.x = 26
	_cal_min_btn.pressed.connect(_on_cal_min); hdr.add_child(_cal_min_btn)
	var scroll := ScrollContainer.new(); scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	rv.add_child(scroll)
	var rows := VBoxContainer.new(); rows.size_flags_horizontal = Control.SIZE_EXPAND_FILL; scroll.add_child(rows)
	rows.add_theme_constant_override("separation", 2)
	for ch in range(12):
		rows.add_child(_build_row(ch))
	# today's finding: P0 = rear-left knee
	_rows[0]["phys"].select(PHYS_OPTIONS.find("RL"))
	_rows[0]["joint"].select(JOINT_OPTIONS.find("knee"))


func _build_row(ch: int) -> Control:
	var box := VBoxContainer.new()
	var r1 := HBoxContainer.new(); box.add_child(r1)
	var r2 := HBoxContainer.new(); box.add_child(r2)
	var d: Dictionary = {"ch": ch, "armed": false}
	r1.add_child(_lbl("P%-2d" % ch, 14))
	var phys := OptionButton.new()
	for o in PHYS_OPTIONS: phys.add_item(o)
	phys.item_selected.connect(func(_i): _refresh_row(ch)); r1.add_child(phys); d["phys"] = phys
	var joint := OptionButton.new()
	for o in JOINT_OPTIONS: joint.add_item(o)
	joint.item_selected.connect(func(_i): _refresh_row(ch)); r1.add_child(joint); d["joint"] = joint
	var sim := _lbl("sim ? → ?", 12); sim.custom_minimum_size.x = 230; r1.add_child(sim); d["sim"] = sim
	var arm := Button.new(); arm.text = "ARM"; arm.pressed.connect(func(): _on_arm(ch)); r1.add_child(arm); d["arm"] = arm
	var limp := Button.new(); limp.text = "LIMP"; limp.pressed.connect(_on_limp); r1.add_child(limp)
	var widen := Button.new(); widen.text = "WIDEN"; widen.pressed.connect(func(): _on_widen(ch)); r1.add_child(widen); d["widen"] = widen
	var rec := Button.new(); rec.text = "record map"; rec.pressed.connect(func(): _on_record(ch)); r1.add_child(rec)

	var slider := HSlider.new(); slider.min_value = 500; slider.max_value = 2500; slider.step = 5; slider.value = 1500
	slider.custom_minimum_size.x = 220; slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.value_changed.connect(func(v): _on_slider(ch, v)); r2.add_child(slider); d["slider"] = slider
	var us := _lbl("1500 µs", 12); us.custom_minimum_size.x = 70; r2.add_child(us); d["us"] = us
	var sign := Button.new(); sign.toggle_mode = true; sign.text = "+"
	sign.toggled.connect(func(p): sign.text = "−" if p else "+"; _refresh_row(ch)); r2.add_child(sign); d["sign"] = sign
	r2.add_child(_lbl("origin", 11))
	var origin := LineEdit.new(); origin.text = "1500"; origin.custom_minimum_size.x = 56; r2.add_child(origin); d["origin"] = origin
	r2.add_child(_lbl("min", 11))
	var mn := LineEdit.new(); mn.text = "900"; mn.custom_minimum_size.x = 52; r2.add_child(mn); d["min"] = mn
	mn.text_submitted.connect(func(_t): _apply_row_limits(ch))
	r2.add_child(_lbl("max", 11))
	var mx := LineEdit.new(); mx.text = "2100"; mx.custom_minimum_size.x = 52; r2.add_child(mx); d["max"] = mx
	mx.text_submitted.connect(func(_t): _apply_row_limits(ch))
	var cur := _lbl("", 11); cur.custom_minimum_size.x = 200; r2.add_child(cur); d["cur"] = cur
	_rows.append(d)
	return box


func _margin(l: int = 10, t: int = 4, r: int = 8, b: int = 4) -> MarginContainer:
	var m := MarginContainer.new()
	m.add_theme_constant_override("margin_left", l); m.add_theme_constant_override("margin_top", t)
	m.add_theme_constant_override("margin_right", r); m.add_theme_constant_override("margin_bottom", b)
	return m

# ---- what the loop costs -------------------------------------------------------------
# CPU is reported as per cent of the TICK BUDGET, not of a wall-clock second: at 50 Hz
# the budget is 20 ms, so 50 % means the tick used half the time it had. The MAX sits
# beside the middle on purpose — a mean is blind to the one tick in forty that blows the
# budget, and that tick is the one that misses a deadline.
func _fmt_ms(ms: float) -> String:
	if ms >= 1.0: return "%.2f ms" % ms
	if ms >= 0.001: return "%.1f µs" % (ms * 1000.0)
	return "%.0f ns" % (ms * 1000000.0)


func _update_cost_rows() -> void:
	var c: Dictionary = _tele.get("cpu", {})
	var m: Dictionary = _tele.get("mem", {})
	if c.is_empty():
		_tele_lbls["cost"].text = "tick cost: —   (daemon predates the cpu/mem telemetry)"
		_tele_lbls["cost_split"].text = ""
		_tele_lbls["mem"].text = ""
		return
	var budget := float(c.get("budget_ms", 0.0))
	var wmax := float(c.get("wall_max", 0.0))
	if _tick_meter: _tick_meter.set_window(float(c.get("wall_p50", 0.0)), float(c.get("wall_p95", 0.0)), wmax)
	# Absolute time first, per cent on the max only. An idle loop and a saturated one are
	# three orders of magnitude apart, and a bare "%.0f%%" prints 0 for everything below
	# 0.5 % — which is the whole healthy range.
	_tele_lbls["cost"].text = "tick cost: p50 %s  p95 %s  max %s = %.1f%% of the %.0f ms budget" % [
		_fmt_ms(float(c.get("wall_p50", 0.0)) * budget / 100.0),
		_fmt_ms(float(c.get("wall_p95", 0.0)) * budget / 100.0),
		_fmt_ms(wmax * budget / 100.0), wmax, budget]
	# Red only at a real overrun; amber is the margin worth watching, not a fault.
	var col := Color(0.9, 0.9, 0.9)
	if wmax >= 100.0: col = Color(1, 0.3, 0.3)
	elif wmax >= 75.0: col = Color(1, 0.85, 0.4)
	_tele_lbls["cost"].add_theme_color_override("font_color", col)

	# wall >> cpu means the tick sat BLOCKED (I²C writes, lock contention, preemption)
	# and more compute is nearly free; wall ≈ cpu means every new module costs budget.
	var wp := float(c.get("wall_p50", 0.0))
	var cp := float(c.get("cpu_p50", 0.0))
	var verdict := "—"
	if wp > 0.5:
		var ratio := cp / wp
		verdict = "BLOCKED (bus/lock)" if ratio < 0.5 else ("compute-bound" if ratio > 0.8 else "mixed")
	_tele_lbls["cost_split"].text = "  wall %s vs cpu %s → %s      proc %.1f%% of a core   %.1f °C" % [
		_fmt_ms(wp * budget / 100.0), _fmt_ms(cp * budget / 100.0), verdict,
		float(c.get("proc_pct", 0.0)), float(c.get("temp_c", 0.0))]
	var temp := float(c.get("temp_c", 0.0))
	_tele_lbls["cost_split"].add_theme_color_override("font_color",
		Color(1, 0.85, 0.4) if temp >= 80.0 else Color(0.75, 0.75, 0.75))

	# Swap is the one to fear on a 2 GB board: once the brain swaps, latency is gone.
	var swap := float(m.get("swap_mb", 0.0))
	_tele_lbls["mem"].text = "mem: rss %.0f MB  swap %.0f MB  avail %.0f MB  %+.2f MB/min  majflt %s" % [
		float(m.get("rss_mb", 0.0)), swap, float(m.get("avail_mb", 0.0)),
		float(m.get("growth_mb_min", 0.0)), str(m.get("majflt", "—"))]
	_tele_lbls["mem"].add_theme_color_override("font_color",
		Color(1, 0.3, 0.3) if swap > 0.0 else Color(0.9, 0.9, 0.9))


func _lbl(text: String, size: int = 12) -> Label:
	var l := Label.new(); l.text = text
	l.add_theme_font_size_override("font_size", size)
	return l


# ======================================================================================
# Row helpers
# ======================================================================================
func _row_phys(d: Dictionary) -> String:
	return PHYS_OPTIONS[d["phys"].selected]

func _row_joint_idx(d: Dictionary) -> int:
	return d["joint"].selected

func _row_sign(d: Dictionary) -> float:
	return -1.0 if d["sign"].button_pressed else 1.0

func _row_origin(d: Dictionary) -> float:
	return float(d["origin"].text.to_float())

func _refresh_all_rows() -> void:
	for ch in range(12): _refresh_row(ch)

func _refresh_row(ch: int) -> void:
	var d: Dictionary = _rows[ch]
	var phys := _row_phys(d)
	if phys == "?":
		d["sim"].text = "sim ?  (unmapped)"
		d["sim"].remove_theme_color_override("font_color")
		return
	var sim: String = MIRROR[phys]
	# PHYSICAL first — this is the robot's panel.  The sim name follows because it is
	# mirrored (sim "rr" IS the physical rear-left) and the colour is the sim leg's.
	d["sim"].text = "PHYSICAL %s %s   ← sim '%s' (%s)" % [PHYS_LONG[phys], JOINT_OPTIONS[_row_joint_idx(d)], sim, ["red", "green", "blue", "yellow"][SIM_INDEX[sim]]]
	d["sim"].add_theme_color_override("font_color", SIM_COLORS[SIM_INDEX[sim]])


func _body_is_measured() -> bool:
	return _tele.has("body") and str(_tele["body"]) == "measured"

func _refuse_if_not_measured(what: String) -> bool:
	if not _connected:
		_set_status("%s refused: not connected" % what); return true
	if not _body_is_measured():
		_set_status("%s REFUSED: daemon body is '%s', calibration requires 'measured' (SPEC §5.2)" % [what, str(_tele.get("body", "?"))])
		return true
	return false

func _set_status(msg: String) -> void:
	_status_msg = msg
	_status_lbl.text = msg
	print("bench: ", msg)


# ======================================================================================
# Verbs
# ======================================================================================
func _call(req: Dictionary) -> Dictionary:
	if not _connected: return {"ok": false, "error": "not connected"}
	var rep: Dictionary = _client.call("request", req)
	if not bool(rep.get("ok", false)):
		_link_fail += 1
		if str(rep.get("error", "")) != "":
			_set_status("%s → %s" % [str(req.get("verb")), str(rep.get("error"))])
	else:
		_link_fail = 0
	return rep


func _on_connect() -> void:
	# The video stream lives on ogma_host, a different daemon from benchd, so it gets its
	# own client. Receive-only: there is no verb path from here to the brain (SPEC §1.1).
	if _video == null:
		_video = ClassDB.instantiate("VideoClient")
		if _video != null: add_child(_video)
	if _video != null and not bool(_video.call("connect_to", _host_edit.text, VIDEO_PORT)):
		_video_lbl.text = "video: %s" % str(_video.call("last_error"))
	var ok: bool = _client.call("connect_to", _host_edit.text, REP_PORT, PUB_PORT)
	_connected = ok
	_link_fail = 0
	if ok:
		var rep := _call({"verb": "ping"})
		_set_status("connected to %s" % _host_edit.text if bool(rep.get("ok", false)) else "socket open, but no reply from daemon: %s" % str(rep.get("error", "")))
	else:
		_set_status("connect failed: %s" % str(_client.call("last_error")))
	if _connected:
		_refresh_poses()

func _on_disconnect() -> void:
	# Disconnect is an ACTION, not a closed window (SPEC §4.2.2).  In bench mode the
	# daemon's deadman limps the robot when the pings stop — say so.
	_client.call("disconnect_from")
	_connected = false
	_tele = {}; _tele_ms = -1
	for d in _rows: d["armed"] = false
	_set_status("disconnected — bench deadman on the Pi sends the rescue pose within ~1 s")

func _on_limp() -> void:
	var rep := _call({"verb": "limp"})
	if bool(rep.get("ok", false)):
		for d in _rows: d["armed"] = false
		_pending.clear()
		_set_status("rescue pose '%s' commanded (this HAT cannot limp a servo)" % str(rep.get("rescue_pose", "NONE SAVED")))

func _on_arm(ch: int) -> void:
	if _refuse_if_not_measured("ARM P%d" % ch): return
	var d: Dictionary = _rows[ch]
	var us := int(d["slider"].value)
	var rep := _call({"verb": "servo.set", "ch": ch, "us": us})
	if bool(rep.get("ok", false)):
		for r in _rows: r["armed"] = false
		d["armed"] = true
		_set_status("P%d armed at %d µs (clamped %s)" % [ch, us, str(rep.get("clamped_us", us))])

func _on_slider(ch: int, v: float) -> void:
	var d: Dictionary = _rows[ch]
	d["us"].text = "%d µs" % int(v)
	if d["armed"] and _connected:
		_pending[ch] = int(v)          # throttled flush in _process, final value always sent

func _on_widen(ch: int) -> void:
	if _refuse_if_not_measured("WIDEN P%d" % ch): return
	var rep := _call({"verb": "cal.begin", "ch": ch})
	if bool(rep.get("ok", false)):
		_set_status("P%d widened to 500–2500 µs (audited; ≤120 s; cal.end restores)" % ch)

func _on_cal_end() -> void:
	var rep := _call({"verb": "cal.end"})
	if bool(rep.get("ok", false)): _set_status("cal.end: operating limits restored")

func _on_record(ch: int) -> void:
	var d: Dictionary = _rows[ch]
	var phys := _row_phys(d)
	if phys == "?":
		_set_status("record P%d: choose the PHYSICAL leg first" % ch); return
	var req := {"verb": "cal.map", "ch": ch, "physical": phys, "joint": JOINT_OPTIONS[_row_joint_idx(d)],
		"sign": int(_row_sign(d)), "origin_us": int(_row_origin(d)),
		"min_us": int(d["min"].text.to_int()), "max_us": int(d["max"].text.to_int())}
	var rep := _call(req)
	if bool(rep.get("ok", false)):
		_set_status("P%d recorded: %s %s (sim %s)" % [ch, phys, req["joint"], MIRROR[phys]])
		_call({"verb": "servo.limits", "ch": ch, "min_us": req["min_us"], "max_us": req["max_us"]})

func _on_save_map() -> void:
	var rep := _call({"verb": "cal.save"})
	if bool(rep.get("ok", false)): _set_status("map saved on the Pi: %s" % str(rep.get("path", "")))

func _on_load_map() -> void:
	var rep := _call({"verb": "cal.load"})
	if not bool(rep.get("ok", false)): return
	var m: Variant = rep.get("map", {})
	var servos: Array = m.get("servos", []) if m is Dictionary else []
	for s in servos:
		if not (s is Dictionary): continue
		var ch := int(s.get("ch", -1))
		if ch < 0 or ch >= 12: continue
		var d: Dictionary = _rows[ch]
		var pi := PHYS_OPTIONS.find(str(s.get("physical", "?")))
		d["phys"].select(max(0, pi))
		var ji := JOINT_OPTIONS.find(str(s.get("joint", "hip1")))
		d["joint"].select(max(0, ji))
		d["sign"].button_pressed = int(s.get("sign", 1)) < 0
		d["sign"].text = "−" if d["sign"].button_pressed else "+"
		d["origin"].text = str(int(s.get("origin_us", 1500)))
		d["min"].text = str(int(s.get("min_us", 900)))
		d["max"].text = str(int(s.get("max_us", 2100)))
		_refresh_row(ch)
	_set_status("map loaded: %d channel(s) from %s" % [servos.size(), str(rep.get("path", ""))])


# ======================================================================================
# Per frame: ping, throttle, telemetry, body
# ======================================================================================
func _update_video() -> void:
	if _video == null or not bool(_video.call("is_connected")):
		return
	# Minimised: skip entirely rather than build an ImageTexture nobody draws. Safe to
	# stop polling because the SUB socket is ZMQ_CONFLATE — at most one frame is ever
	# queued, so nothing backs up while the panel is closed.
	if _tele_content != null and not _tele_content.visible:
		return
	if not bool(_video.call("poll")):
		return
	var vi: Image = _video.call("view_image")
	if vi != null and not vi.is_empty():
		_view_tex.texture = ImageTexture.create_from_image(vi)
	var bi: Image = _video.call("brain_image")
	if bi != null and not bi.is_empty():
		_brain_tex.texture = ImageTexture.create_from_image(bi)
	var info: Dictionary = _video.call("info")
	_video_lbl.text = "frame %s   %dx%d source   %.1f kB" % [
		str(info.get("seq", "—")), int(info.get("src_w", 0)), int(info.get("src_h", 0)),
		float(info.get("bytes", 0)) / 1024.0]


func _process(delta: float) -> void:
	_update_video()
	if _connected:
		_ping_acc += delta
		if _ping_acc >= PING_S:
			_ping_acc = 0.0
			_call({"verb": "ping"})
		_send_acc += delta
		if _send_acc >= SEND_THROTTLE_S and not _pending.is_empty():
			_send_acc = 0.0
			for ch in _pending.keys():
				_call({"verb": "servo.set", "ch": ch, "us": _pending[ch]})
			_pending.clear()
		var t: Dictionary = _client.call("poll_telemetry")
		if not t.is_empty():
			_tele = t
			_tele_ms = Time.get_ticks_msec()
	_update_labels()
	_drive_body()


func _tele_fresh() -> bool:
	return _tele_ms >= 0 and Time.get_ticks_msec() - _tele_ms < 500


func _update_labels() -> void:
	var age_ms: int = (Time.get_ticks_msec() - _tele_ms) if _tele_ms >= 0 else -1
	if not _connected:
		_link_lbl.text = "link: DISCONNECTED"
		_link_lbl.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	elif _link_fail >= 3 or (age_ms < 0 or age_ms > 1500):
		_link_lbl.text = "link: LOST (%s) — daemon deadman → rescue pose" % ("no telemetry" if age_ms < 0 else "%d ms stale" % age_ms)
		_link_lbl.add_theme_color_override("font_color", Color(1, 0.3, 0.3))
	else:
		_link_lbl.text = "link: OK   telemetry age %d ms (1 s mean)" % int(_avg.get("age", age_ms))
		_link_lbl.add_theme_color_override("font_color", Color(0.3, 1, 0.3))
	var mode := str(_tele.get("mode", "—"))
	_mode_lbl.text = "MODE: %s — link loss ⇒ RESCUE POSE (%s)" % [mode, "saved" if _tele.get("rescue_pose") != null else "NONE SAVED — save a pose named rescue"]
	_mode_lbl.add_theme_color_override("font_color", Color(1, 0.85, 0.3))
	if _tele.is_empty():
		_body_lbl.text = "BODY: — (no telemetry)"
		_body_lbl.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	elif _body_is_measured():
		_body_lbl.text = "BODY: measured ✓"
		_body_lbl.add_theme_color_override("font_color", Color(0.3, 1, 0.3))
	else:
		_body_lbl.text = "BODY: %s — CALIBRATION REFUSED (needs measured)" % str(_tele.get("body"))
		_body_lbl.add_theme_color_override("font_color", Color(1, 0.3, 0.3))

	# ---- numeric telemetry: accumulate every frame, display 1 s means (raw values at 10 Hz
	# flicker faster than they can be read).  State fields below stay immediate.
	var now := Time.get_ticks_msec()
	if _avg_started < 0: _avg_started = now
	var vbat := float(_tele.get("vbat", 0.0))            # instantaneous, for the self-check below
	var adc: Array = _tele.get("adc", [])
	var sample := {"vbat": float(_tele.get("vbat", 0.0)), "tick_hz": float(_tele.get("tick_hz", 0.0)),
				   "deadman": float(_tele.get("deadman_ms_left", 0.0)), "age": float(max(age_ms, 0))}
	for i in range(adc.size()): sample["adc%d" % i] = float(adc[i])
	for k in sample: _avg_sum[k] = float(_avg_sum.get(k, 0.0)) + sample[k]
	_avg_n += 1
	if now - _avg_started >= 1000:
		_avg = {}
		for k in _avg_sum: _avg[k] = _avg_sum[k] / float(_avg_n)
		_avg_sum = {}; _avg_n = 0; _avg_started = now
		var vbat_avg := float(_avg.get("vbat", 0.0))
		_tele_lbls["vbat"].text = "Vbat: %.2f V   (min 6.0 V; 1 s mean)" % vbat_avg
		_tele_lbls["vbat"].add_theme_color_override("font_color", Color(1, 0.3, 0.3) if (vbat_avg > 0.0 and vbat_avg < VBAT_MIN) else Color(0.9, 0.9, 0.9))
		var parts: Array = []
		for i in range(adc.size()):
			var a := float(_avg.get("adc%d" % i, 0.0))
			parts.append("A%d %4d/%.2fV" % [i, int(a), a * 3.3 / 4095.0])
		_tele_lbls["adc"].text = "adc: " + ("  ".join(parts) if parts.size() else "—")
		_tele_lbls["tick_hz"].text = "tick_hz: %.2f   (HAT frame 49.95 Hz — a different clock)" % float(_avg.get("tick_hz", 0.0))
		_tele_lbls["deadman_ms_left"].text = "deadman_ms_left: %d" % int(_avg.get("deadman", 0.0))
		_tele_lbls["age"].text = "frame seq %s   age %d ms (1 s mean)" % [str(_tele.get("seq", "—")), int(_avg.get("age", 0.0))]
	_update_cost_rows()
	_tele_lbls["overruns"].text = "overruns: %s   bus_errors: %s" % [str(_tele.get("overruns", "—")), str(_tele.get("bus_errors", "—"))]
	_tele_lbls["watchdog_trips"].text = "watchdog_trips: %s   low_battery: %s   pi_throttled: %s" % [str(_tele.get("watchdog_trips", "—")), str(_tele.get("low_battery", "—")), str(_tele.get("pi_throttled", "—"))]
	_tele_lbls["armed_ch"].text = "armed_ch: %s   rescue_pose: %s" % [str(_tele.get("armed_ch", "—")), str(_tele.get("rescue_pose", "NONE"))]
	var cal_ch = _tele.get("cal_ch", -1)
	_tele_lbls["cal_ch"].text = "cal_ch: %s   cal_ms_left: %s" % [str(cal_ch), str(_tele.get("cal_ms_left", "—"))]
	_widen_lbl.text = "widened: %s" % ("none" if cal_ch < 0 else "P%d (%s ms left)" % [cal_ch, str(_tele.get("cal_ms_left", "?"))])

	# self-check
	_set_check("0x14 present", _tele_fresh(), "live" if _tele_fresh() else "no telemetry")
	_set_check("Vbat plausible 6.0–8.4 V", _tele_fresh() and vbat >= VBAT_MIN and vbat <= VBAT_MAX, "%.2f V" % vbat)
	for item in ["IMU WHO_AM_I = 0xEA", "INA219 ⟷ A4 agree", "ToF plausible", "FSR sum ≈ 1.0 BW"]:
		_check_lbls[item].text = "○ %s — not fitted" % item

	# rows: reflect the daemon's view of each channel
	var servos: Array = _tele.get("servos", [])
	var armed_ch := int(_tele.get("armed_ch", -1))
	for ch in range(12):
		var d: Dictionary = _rows[ch]
		if _tele_fresh():
			d["armed"] = (armed_ch == ch)
		d["arm"].text = "ARMED" if d["armed"] else "ARM"
		d["arm"].add_theme_color_override("font_color", Color(0.3, 1, 0.3) if d["armed"] else Color(1, 1, 1))
		d["widen"].text = "WIDENED" if cal_ch == ch else "WIDEN"
		if ch < servos.size() and servos[ch] is Dictionary:
			var s: Dictionary = servos[ch]
			d["cur"].text = "%d→%d  [%d–%d]  %.1fs at limit" % [int(s.get("current_us", 0)), int(s.get("target_us", 0)), int(s.get("min_us", 0)), int(s.get("max_us", 0)), float(s.get("at_limit_s", 0.0))]
		elif not _connected:
			d["cur"].text = ""


func _set_check(item: String, ok: bool, detail: String) -> void:
	var l: Label = _check_lbls[item]
	l.text = "%s %s — %s" % ["●" if ok else "○", item, detail]
	l.add_theme_color_override("font_color", Color(0.3, 1, 0.3) if ok else Color(1, 0.4, 0.4))


func _drive_body() -> void:
	# COMMANDED pose: the daemon's current pulse for the armed channel (what the
	# hardware is being told, after slew), the slider for the rest.  Only rows with a
	# known physical leg reach the model.
	if _body == null: return
	var targets: Variant = _body.get("servo_targets")
	if not (targets is Array) or targets.size() < 12: return
	var servos: Array = _tele.get("servos", [])
	for ch in range(12):
		var d: Dictionary = _rows[ch]
		var phys := _row_phys(d)
		if phys == "?": continue
		var us: float = d["slider"].value
		if d["armed"] and _tele_fresh() and ch < servos.size() and servos[ch] is Dictionary:
			us = float(servos[ch].get("current_us", us))
		var angle: float = _row_sign(d) * (us - _row_origin(d)) / US_PER_RAD
		var k: int = SIM_INDEX[MIRROR[phys]] * 3 + _row_joint_idx(d)
		targets[k] = angle

# Enter in a min/max field applies that row's OPERATING limits at once (servo.limits);
# "record map" still writes them into the map.  Values are 500..2500 with min < max, and
# the daemon refuses anything else — the reply lands in the status line.
func _apply_row_limits(ch: int) -> void:
	var d: Dictionary = _rows[ch]
	var lo := int(d["min"].text.to_int())
	var hi := int(d["max"].text.to_int())
	var rep: Dictionary = _call({"verb": "servo.limits", "ch": ch, "min_us": lo, "max_us": hi})
	_set_status("P%d limits %d–%d: %s" % [ch, lo, hi, "ok" if rep.get("ok", false) else str(rep.get("error", "?"))])

# ======================================================================================
# Poses
# ======================================================================================
func _slider_us() -> Array:
	var us: Array = []
	for d in _rows: us.append(int(d["slider"].value))
	return us

func _refresh_poses() -> void:
	if not _connected: return
	var rep: Dictionary = _call({"verb": "pose.list"})
	var keep := _pose_list.get_item_text(_pose_list.selected) if _pose_list.selected >= 0 else ""
	_pose_list.clear()
	for n in rep.get("poses", []):
		_pose_list.add_item(str(n))
	for k in range(_pose_list.item_count):
		if _pose_list.get_item_text(k) == keep: _pose_list.select(k)

func _on_pose_save() -> void:
	var name := _pose_name.text.strip_edges()
	if name.is_empty():
		_set_status("pose: give it a name first"); return
	var rep: Dictionary = _call({"verb": "pose.save", "name": name, "us": _slider_us()})
	_set_status("pose '%s' saved (%s poses)" % [name, str(rep.get("count", "?"))] if rep.get("ok", false) else "pose.save: " + str(rep.get("error", "?")))
	_refresh_poses()
	for k in range(_pose_list.item_count):
		if _pose_list.get_item_text(k) == name: _pose_list.select(k)

func _on_pose_recall() -> void:
	if _pose_list.selected < 0:
		_set_status("pose: nothing selected"); return
	var name := _pose_list.get_item_text(_pose_list.selected)
	if _refuse_if_not_measured("RECALL pose '%s'" % name): return
	var rep: Dictionary = _call({"verb": "pose.get", "name": name})
	if not rep.get("ok", false):
		_set_status("pose.get: " + str(rep.get("error", "?"))); return
	var us: Array = rep.get("us", [])
	if us.size() != 12:
		_set_status("pose '%s' is malformed" % name); return
	for ch in range(12):
		_rows[ch]["slider"].set_value_no_signal(float(us[ch]))
		_rows[ch]["us"].text = "%d µs" % int(us[ch])
	var set_rep: Dictionary = _call({"verb": "pose.set", "us": us})
	if set_rep.get("ok", false):
		for d in _rows: d["armed"] = true
		_pending.clear()
		_set_status("pose '%s' recalled — all 12 armed (slewing)" % name)
	else:
		_set_status("pose.set: " + str(set_rep.get("error", "?")))

func _on_pose_delete() -> void:
	if _pose_list.selected < 0: return
	var name := _pose_list.get_item_text(_pose_list.selected)
	var rep: Dictionary = _call({"verb": "pose.delete", "name": name})
	_set_status("pose '%s' deleted" % name if rep.get("ok", false) else "pose.delete: " + str(rep.get("error", "?")))
	_refresh_poses()

# ======================================================================================
# Minimise (the ablation_panel.gd pattern: content hidden, panel shrinks to its header)
# ======================================================================================
func _on_cal_min() -> void:
	_cal_min = not _cal_min
	_apply_min(_right_panel, _cal_content, _cal_min_btn, _cal_min)

func _on_tele_min() -> void:
	_tele_min = not _tele_min
	_apply_min(_left_panel, _tele_content, _tele_min_btn, _tele_min)

func _apply_min(panel: PanelContainer, content: Control, btn: Button, minimised: bool) -> void:
	content.visible = not minimised
	btn.text = "▲" if minimised else "▼"
	if minimised:
		panel.anchor_bottom = 0.0
		panel.offset_bottom = TOP_H + 34
	else:
		panel.anchor_bottom = 1.0
		panel.offset_bottom = -30
