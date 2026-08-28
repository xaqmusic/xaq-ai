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


func _ready() -> void:
	_build_ui()
	# Put the sim body in calibrate mode: chassis suspended, legs FK-written from
	# servo_targets each physics tick (picrawler_body.gd, the [C] path).  Deferred so
	# the body's own _ready has run.  Sign/origin are applied HERE (in µs), so the
	# body's own sign/origin arrays are neutralised.
	call_deferred("_enter_calibrate_mode")
	_refresh_all_rows()


func _enter_calibrate_mode() -> void:
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

	# ---- top bar: connect / disconnect / link / banners ------------------------------
	var top := PanelContainer.new()
	top.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	_ui.add_child(top)
	var topv := VBoxContainer.new(); top.add_child(topv)
	var row := HBoxContainer.new(); topv.add_child(row)
	row.add_child(_lbl("ogma_benchd @"))
	_host_edit = LineEdit.new(); _host_edit.text = "10.0.0.113"; _host_edit.custom_minimum_size.x = 160
	row.add_child(_host_edit)
	row.add_child(_lbl(":%d / :%d" % [REP_PORT, PUB_PORT]))
	var cb := Button.new(); cb.text = "  CONNECT  "; cb.pressed.connect(_on_connect); row.add_child(cb)
	var db := Button.new(); db.text = "  DISCONNECT  "; db.pressed.connect(_on_disconnect); row.add_child(db)
	var lb := Button.new(); lb.text = "  LIMP ALL (pulse 0)  "; lb.pressed.connect(_on_limp); row.add_child(lb)
	lb.add_theme_color_override("font_color", Color(1, 0.8, 0.3))
	_link_lbl = _lbl("link: disconnected"); row.add_child(_link_lbl)
	var banner := HBoxContainer.new(); topv.add_child(banner)
	_mode_lbl = _lbl("MODE: —", 20); banner.add_child(_mode_lbl)
	banner.add_child(_lbl("      "))
	_body_lbl = _lbl("BODY: —", 20); banner.add_child(_body_lbl)
	banner.add_child(_lbl("      "))
	_status_lbl = _lbl("", 14); _status_lbl.add_theme_color_override("font_color", Color(1, 0.6, 0.6))
	banner.add_child(_status_lbl)

	# ---- left: telemetry + boot self-check --------------------------------------------
	var left := PanelContainer.new()
	left.anchor_top = 0.0; left.anchor_bottom = 1.0; left.anchor_left = 0.0; left.anchor_right = 0.0
	left.offset_top = 78; left.offset_left = 0; left.offset_right = 330; left.offset_bottom = -30
	_ui.add_child(left)
	var lv := VBoxContainer.new(); left.add_child(lv)
	lv.add_child(_lbl("TELEMETRY  (Pi log is the record; this is the view)", 13))
	for key in ["vbat", "adc", "tick_hz", "overruns", "watchdog_trips", "deadman_ms_left", "armed_ch", "cal_ch", "age"]:
		var l := _lbl(key + ": —"); _tele_lbls[key] = l; lv.add_child(l)
	lv.add_child(_lbl(" "))
	lv.add_child(_lbl("BOOT SELF-CHECK  (SPEC §6)", 13))
	for item in ["0x14 present", "Vbat plausible 6.0–8.4 V", "IMU WHO_AM_I = 0xEA", "INA219 ⟷ A4 agree", "ToF plausible", "FSR sum ≈ 1.0 BW"]:
		var l := _lbl("○ " + item); _check_lbls[item] = l; lv.add_child(l)
		l.add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))

	# ---- bottom-centre: the honesty label on the 3-D view ----------------------------
	var cap := _lbl("3-D view = COMMANDED pose — not measured (hobby servos report nothing)", 14)
	cap.set_anchors_and_offsets_preset(Control.PRESET_CENTER_BOTTOM)
	cap.offset_top = -28; cap.offset_bottom = -6
	cap.add_theme_color_override("font_color", Color(1, 0.85, 0.4))
	_ui.add_child(cap)

	# ---- right: calibration panel -------------------------------------------------------
	var right := PanelContainer.new()
	right.anchor_top = 0.0; right.anchor_bottom = 1.0; right.anchor_left = 1.0; right.anchor_right = 1.0
	right.offset_top = 78; right.offset_left = -760; right.offset_right = 0; right.offset_bottom = -30
	_ui.add_child(right)
	var rv := VBoxContainer.new(); right.add_child(rv)
	var hdr := HBoxContainer.new(); rv.add_child(hdr)
	hdr.add_child(_lbl("SERVO CALIBRATION — one servo at a time, robot on a stand", 13))
	var sv := Button.new(); sv.text = "SAVE MAP"; sv.pressed.connect(_on_save_map); hdr.add_child(sv)
	var ld := Button.new(); ld.text = "LOAD MAP"; ld.pressed.connect(_on_load_map); hdr.add_child(ld)
	var ce := Button.new(); ce.text = "cal.end"; ce.pressed.connect(_on_cal_end); hdr.add_child(ce)
	_widen_lbl = _lbl("widened: none"); hdr.add_child(_widen_lbl)
	rv.add_child(_lbl("ch · physical · joint · sim name (colour = cross-check) · µs · sign · origin · min · max", 11))
	var scroll := ScrollContainer.new(); scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	rv.add_child(scroll)
	var rows := VBoxContainer.new(); rows.size_flags_horizontal = Control.SIZE_EXPAND_FILL; scroll.add_child(rows)
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
	var sim := _lbl("sim ? → ?", 12); sim.custom_minimum_size.x = 250; r1.add_child(sim); d["sim"] = sim
	var arm := Button.new(); arm.text = "ARM"; arm.pressed.connect(func(): _on_arm(ch)); r1.add_child(arm); d["arm"] = arm
	var limp := Button.new(); limp.text = "LIMP"; limp.pressed.connect(_on_limp); r1.add_child(limp)
	var widen := Button.new(); widen.text = "WIDEN"; widen.pressed.connect(func(): _on_widen(ch)); r1.add_child(widen); d["widen"] = widen
	var rec := Button.new(); rec.text = "record map"; rec.pressed.connect(func(): _on_record(ch)); r1.add_child(rec)

	var slider := HSlider.new(); slider.min_value = 500; slider.max_value = 2500; slider.step = 5; slider.value = 1500
	slider.custom_minimum_size.x = 260; slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
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
	var cur := _lbl("", 11); cur.custom_minimum_size.x = 120; r2.add_child(cur); d["cur"] = cur
	_rows.append(d)
	return box


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
	var ok: bool = _client.call("connect_to", _host_edit.text, REP_PORT, PUB_PORT)
	_connected = ok
	_link_fail = 0
	if ok:
		var rep := _call({"verb": "ping"})
		_set_status("connected to %s" % _host_edit.text if bool(rep.get("ok", false)) else "socket open, but no reply from daemon: %s" % str(rep.get("error", "")))
	else:
		_set_status("connect failed: %s" % str(_client.call("last_error")))

func _on_disconnect() -> void:
	# Disconnect is an ACTION, not a closed window (SPEC §4.2.2).  In bench mode the
	# daemon's deadman limps the robot when the pings stop — say so.
	_client.call("disconnect_from")
	_connected = false
	_tele = {}; _tele_ms = -1
	for d in _rows: d["armed"] = false
	_set_status("disconnected — bench deadman on the Pi limps any armed servo within ~1.5 s")

func _on_limp() -> void:
	var rep := _call({"verb": "limp"})
	if bool(rep.get("ok", false)):
		for d in _rows: d["armed"] = false
		_pending.clear()
		_set_status("limp: all 12 channels → pulse 0")

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
func _process(delta: float) -> void:
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
		_link_lbl.text = "link: LOST (%s) — daemon deadman limps armed servos" % ("no telemetry" if age_ms < 0 else "%d ms stale" % age_ms)
		_link_lbl.add_theme_color_override("font_color", Color(1, 0.3, 0.3))
	else:
		_link_lbl.text = "link: OK   telemetry age %d ms" % age_ms
		_link_lbl.add_theme_color_override("font_color", Color(0.3, 1, 0.3))
	var mode := str(_tele.get("mode", "—"))
	_mode_lbl.text = "MODE: %s — link loss ⇒ LIMP (deadman on the calibration channel)" % mode
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

	var vbat := float(_tele.get("vbat", 0.0))
	_tele_lbls["vbat"].text = "Vbat: %.2f V   (min 6.0 V)" % vbat
	_tele_lbls["vbat"].add_theme_color_override("font_color", Color(1, 0.3, 0.3) if (vbat > 0.0 and vbat < VBAT_MIN) else Color(0.9, 0.9, 0.9))
	var adc: Array = _tele.get("adc", [])
	var parts: Array = []
	for i in range(adc.size()):
		parts.append("A%d %4d/%.2fV" % [i, int(adc[i]), float(adc[i]) * 3.3 / 4095.0])
	_tele_lbls["adc"].text = "adc: " + ("  ".join(parts) if parts.size() else "—")
	_tele_lbls["tick_hz"].text = "tick_hz: %s   (HAT frame 49.95 Hz — a different clock)" % str(_tele.get("tick_hz", "—"))
	_tele_lbls["overruns"].text = "overruns: %s" % str(_tele.get("overruns", "—"))
	_tele_lbls["watchdog_trips"].text = "watchdog_trips: %s" % str(_tele.get("watchdog_trips", "—"))
	_tele_lbls["deadman_ms_left"].text = "deadman_ms_left: %s" % str(_tele.get("deadman_ms_left", "—"))
	_tele_lbls["armed_ch"].text = "armed_ch: %s" % str(_tele.get("armed_ch", "—"))
	var cal_ch := int(_tele.get("cal_ch", -1))
	_tele_lbls["cal_ch"].text = "cal_ch: %s   cal_ms_left: %s" % [str(cal_ch), str(_tele.get("cal_ms_left", "—"))]
	_tele_lbls["age"].text = "frame seq %s   age %s" % [str(_tele.get("seq", "—")), ("%d ms" % age_ms) if age_ms >= 0 else "—"]
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
			d["cur"].text = "now %d→%d  lim %d–%d  at-limit %.1fs" % [int(s.get("current_us", 0)), int(s.get("target_us", 0)), int(s.get("min_us", 0)), int(s.get("max_us", 0)), float(s.get("at_limit_s", 0.0))]
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
