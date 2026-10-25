extends Node3D
## Bench dashboard — the laptop half of pi_host/PROTOCOL.md.
##
## Talks to ogma_benchd on the Pi through BenchClient (REQ verbs + CONFLATE'd telemetry).
## It speaks the CALIBRATION verb set only: there is no path from here to starting the
## brain (port doc SPEC §1.1) and none may be added.  It can SEE the daemon's run mode and
## STOP / resume the robot (SPACE, or the STOP button) — resume lifts a stop and nothing
## else; the mode, and with it whether a brain may drive, is set on the robot.  Everything a servo does goes
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
# MEASURED, not the 636.6 hobby-servo standard (500–2500 µs ≙ ±π/2): pi_host/calib/
# sensors.json servo.us_per_rad, RL knee, 2026-09-13.  At 636.6 the mirrored pose read
# ~14 % small.  One channel measured; part-to-part scale is unchecked.
const US_PER_RAD := 545.2
const VBAT_MIN := 6.0
const VBAT_MAX := 8.4
# The forward ultrasonic is read by ogma_host, not benchd, so it comes from ogma_host's
# control verb host_sensors (plain-TCP newline JSON, port 7400) on a WORKER THREAD: a
# request blocks up to its timeout when ogma_host is down, and that must never freeze this window.
const HOST_CTRL_PORT := 7400
const RANGE_POLL_MS := 100
# The FSRs: HAT ADC A0-A3 = PHYSICAL FL, FR, RL, RR; the sim legs are mirrored, so sim leg i
# reads A[FSR_CH_FOR_SIM[i]] (BrainInputs.hpp kFsrChannelForSimLeg).  Contact at 200 counts,
# grams through the divider's curve, body 598 g — the FsrModel the brain's inputs use.
const FSR_CH_FOR_SIM := [1, 0, 3, 2]
const FSR_CONTACT_COUNTS := 200
const FSR_CURVE := [[60, 0.0], [1536, 30.0], [1920, 50.0], [2118, 100.0], [2362, 175.0], [2603, 300.0], [2925, 500.0]]
const BODY_MASS_G := 598.0
const IMU_WHO_AM_I := 0xEA

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
# IMU instrument.  The SAME panel the sim uses (imu_scope.gd), fed through
# bench_imu_source.gd — see that file for why the units are converted at the boundary.
var _imu_src: Node = null
var _imu_scope: Control = null
var _imu_btn: Button = null
const UI_FONT := 12
const TOP_H := 84                      # top bar height the side panels hang from
const IMU_MARGIN_X := 12               # IMU scope inset from the right edge
const IMU_MARGIN_Y := 38               # ...and from the bottom, clearing the status strip
const INA_A4_TOL_V := 0.15             # BOM 3.4: ~1 % on the 20K/10K divider is ordinary
# Belly geometry (geometry §G2 / picrawler_body.gd GROUND_CLEARANCE_STAND): 56.3 mm
# standing at spawn, 9.5 mm at the crouch gate. A ToF reading outside a generous band
# around that is not a clearance, it is a mount or an offset that has not been fitted.
const TOF_PLAUSIBLE_MIN_M := 0.0
const TOF_PLAUSIBLE_MAX_M := 0.30
# Above this, the part is rejecting so many of its own measurements that the channel is
# not reporting the belly any more. Rate, not distance — the millimetres cannot say it.
const TOF_BAD_FRAC_WARN := 0.25
var _cal_content: Control
var _cal_min_btn: Button
var _cal_min := true          # starts folded: calibration is done, and it is the widest panel
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
var _stop_btn: Button
var _stop_lbl: Label
var _stop_reply_ms: int = -1          # when a stop/resume reply last told us the state
var _stop_reply_state := false
var _widen_lbl: Label
var _tick_meter: Control
var _power_graph: Control
var _power_seq := -1              # _update_labels runs per FRAME; the graph wants per SAMPLE
var _belly_graph: Control
var _belly_seq := -1
var _video: Node                     # VideoClient — receive-only, see VideoClient.hpp

var _brain_tex: TextureRect          # what the BRAIN sees (the encoder's actual input)
var _video_lbl: Label
# Robot POV: the camera in its own movable, resizable window (operator 2026-10-25).
var _pov_win: Window
var _pov_tex: TextureRect
var _pov_lbl: Label
var _pov_btn: Button
# Forward ultrasonic (see HOST_CTRL_PORT): the worker writes, the frame reads, under the mutex.
var _range_graph: Control
var _range_thread: Thread
var _range_mutex := Mutex.new()
var _range_run := false
var _range_host := ""
var _range_reply: Dictionary = {}
var _range_reply_ms := -1
var _range_pushed_ms := -1
# Bottom-right POWER panel, fed from the ROBOT (INA219, A4, rail) — the same lines as
# picrawler-dash's power rows.  In this scene the body's own panel would show the SIM's power
# model of a suspended body, so the body's model is switched off (_enter_calibrate_mode).
var _pw_panel: VBoxContainer
var _pw_lbls: Array = []
var _pw_prev_us: Array = []
var _pw_prev_ms := -1


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
	# The body's power model ticks while its _power object exists (windowed runs create it),
	# and its panel would show the SIM's current for a suspended body.  Drop both, and the
	# label references, so nothing writes into the freed panel.
	_body.set("power_model", false)
	_body.set("_power", null)
	var sim_pw: Variant = _body.get("_pw_panel")
	if sim_pw is Node and is_instance_valid(sim_pw):
		(sim_pw as Node).queue_free()
	_body.set("_pw_panel", null)
	_body.set("_pw_lbl", [])
	_body.set("foot_display_external", true)     # the lower legs show the ROBOT's FSRs
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
	# The instrument is built once and hidden; it costs nothing while invisible because
	# imu_scope only redraws from its own _process, which stops with visibility.
	_imu_src = (load("res://scripts/bench_imu_source.gd") as Script).new()
	add_child(_imu_src)
	_imu_scope = (load("res://scripts/imu_scope.gd") as Script).new()
	_imu_scope.set("body", _imu_src)
	# The shared panel is translucent so it can float over the sim body; over this
	# dashboard it needs to read as a solid instrument.
	_imu_scope.set("bg_alpha", 0.97)
	_imu_scope.visible = false
	# ⚠ ADD TO THE TREE BEFORE READING ITS SIZE.  imu_scope sets custom_minimum_size in
	# _ready(), and _ready does not run until add_child(), so reading it first returns
	# (0,0) -- which pins a zero-size rect at the corner that then grows right and down
	# once the minimum lands, putting all but the top-left corner off-screen.
	_ui.add_child(_imu_scope)
	# Bottom-right, clear of the top bar and of the 30 px status strip the side panels
	# already leave.  It sat top-right and read as dim because the shaded top bar was
	# drawn over it.
	var isz: Vector2 = _imu_scope.custom_minimum_size
	if isz.x < 1.0 or isz.y < 1.0:
		isz = Vector2(250, 330)          # the panel's own W x height, if _ready has not run
	_imu_scope.anchor_left = 1.0; _imu_scope.anchor_right = 1.0
	_imu_scope.anchor_top = 1.0;  _imu_scope.anchor_bottom = 1.0
	_imu_scope.offset_right  = -IMU_MARGIN_X
	_imu_scope.offset_left   = -IMU_MARGIN_X - isz.x
	_imu_scope.offset_bottom = -IMU_MARGIN_Y
	_imu_scope.offset_top    = -IMU_MARGIN_Y - isz.y
	var topv := VBoxContainer.new(); top.add_child(topv)
	var row := HBoxContainer.new(); topv.add_child(row)
	row.add_child(_lbl("ogma_benchd @"))
	_host_edit = LineEdit.new()
	# OGMA_BENCH_HOST overrides the default (a preview or screenshot run points it somewhere
	# that does not exist, so the window cannot reach the robot by accident).
	_host_edit.text = OS.get_environment("OGMA_BENCH_HOST") if OS.get_environment("OGMA_BENCH_HOST") != "" else "picrawler.local"; _host_edit.custom_minimum_size.x = 160
	row.add_child(_host_edit)
	row.add_child(_lbl(":%d / :%d" % [REP_PORT, PUB_PORT]))
	var cb := Button.new(); cb.text = "CONNECT"; cb.pressed.connect(_on_connect); row.add_child(cb)
	var db := Button.new(); db.text = "DISCONNECT"; db.pressed.connect(_on_disconnect); row.add_child(db)
	# STOP first and loudest: the operator's "do no more harm" control.  SPACE does the same
	# from anywhere in this window (see _input).
	_stop_btn = Button.new(); _stop_btn.text = "■ STOP  [space]"; _stop_btn.custom_minimum_size.x = 150
	_stop_btn.add_theme_color_override("font_color", Color(1, 0.35, 0.35))
	_stop_btn.focus_mode = Control.FOCUS_NONE     # SPACE must never also "press" a focused button
	_stop_btn.pressed.connect(_toggle_stop); row.add_child(_stop_btn)
	var lb := Button.new(); lb.text = "RESCUE POSE"; lb.pressed.connect(_on_limp); row.add_child(lb)
	_imu_btn = Button.new()
	_imu_btn.text = "IMU SCOPE"
	_imu_btn.toggle_mode = true
	_imu_btn.pressed.connect(_on_imu_toggle)
	row.add_child(_imu_btn)
	_pov_btn = Button.new()
	_pov_btn.text = "CAMERA POV"
	_pov_btn.toggle_mode = true
	_pov_btn.pressed.connect(_on_pov_toggle)
	row.add_child(_pov_btn)
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
	_stop_lbl = _lbl("", 15); banner.add_child(_stop_lbl)
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
	for key in ["range", "belly", "adc", "tick_hz", "cost", "cost_split", "mem", "overruns", "watchdog_trips", "deadman_ms_left", "armed_ch", "cal_ch", "age"]:
		var l := _lbl(key + ": —"); _tele_lbls[key] = l; lv.add_child(l)
		if key == "cost":
			_tick_meter = (load("res://scripts/tick_meter.gd") as Script).new()
			_tick_meter.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			lv.add_child(_tick_meter)
		# The forward ultrasonic, beside the belly ToF it must never be confused with.  Vbat
		# and current moved to the bottom-right POWER panel (2026-10-25: the two were redundant).
		if key == "range":
			_range_graph = (load("res://scripts/range_graph.gd") as Script).new()
			_range_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			lv.add_child(_range_graph)
		# Belly contact is a TRANSIENT — a chassis that touches down mid-step and lifts
		# again is invisible in a mean. The trace is where that shows.
		if key == "belly":
			_belly_graph = (load("res://scripts/clearance_graph.gd") as Script).new()
			_belly_graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			lv.add_child(_belly_graph)
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
	# What the camera sees is in its own window (CAMERA POV); the brain's plane stays here,
	# small, so the PAIR can still be compared at a glance.
	var vrow := HBoxContainer.new(); lv.add_child(vrow)
	var bcol := VBoxContainer.new(); vrow.add_child(bcol)
	var blab := _lbl("the BRAIN sees", 11)
	blab.add_theme_color_override("font_color", Color(1, 0.85, 0.4))
	bcol.add_child(blab)
	_brain_tex = TextureRect.new()
	_brain_tex.custom_minimum_size = Vector2(96, 96)
	_brain_tex.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT
	_brain_tex.texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST   # show the pixels, do not smooth them
	bcol.add_child(_brain_tex)
	vrow.add_child(_lbl("   the camera itself: CAMERA POV (top bar)", 11))
	_video_lbl = _lbl("not connected", 11); lv.add_child(_video_lbl)

	# The floating "3-D view = COMMANDED pose" caption used to live here, centred over the
	# render area.  Removed 2026-09-11 (operator): it overlapped the calibration panel.
	# ⚠ THE CLAIM IT MADE IS NOT DROPPED, only moved — it is the only statement in the UI
	# that the 3-D body is FK-written from COMMANDED pulses rather than measured (hobby
	# servos report nothing back), and losing it would leave the view looking like
	# feedback.  It now rides in the calibration panel header, beside the sliders that do
	# the commanding.

	# ---- right: calibration panel -------------------------------------------------------
	var right := PanelContainer.new(); _right_panel = right
	right.anchor_top = 0.0; right.anchor_bottom = 1.0; right.anchor_left = 1.0; right.anchor_right = 1.0
	right.offset_top = TOP_H; right.offset_left = -860; right.offset_right = 0; right.offset_bottom = -30
	_ui.add_child(right)
	var rmargin := _margin(); right.add_child(rmargin)
	var rroot := VBoxContainer.new(); rmargin.add_child(rroot)
	var hdr := HBoxContainer.new(); rroot.add_child(hdr)
	hdr.add_child(_lbl("SERVO CALIBRATION — robot on a stand", 13))
	# See the note where the floating caption was removed: this is the UI's only statement
	# that the 3-D body shows what was COMMANDED, not what the servos did.
	var cmd_lbl := _lbl("· 3-D view = COMMANDED pose", 11)
	cmd_lbl.add_theme_color_override("font_color", Color(1, 0.85, 0.4))
	hdr.add_child(cmd_lbl)
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

	# Fold the calibration panel NOW rather than relying on _cal_min alone: the flag only
	# records the state, _apply_min is what moves the panel's anchors and flips the arrow.
	# Setting one without the other ships a panel that claims to be folded and is not.
	_apply_min(_right_panel, _cal_content, _cal_min_btn, _cal_min)
	# Draw the scope LAST so it survives the calibration panel being re-opened: siblings
	# paint in tree order and the scope is created before the panels, so it would
	# otherwise be buried the moment the operator un-folds calibration.
	_ui.move_child(_imu_scope, -1)
	_build_pov_window()
	_build_power_panel()


# The robot's point of view, in a window that moves and resizes inside this one.  Hidden
# until CAMERA POV; the video is only polled while something shows it.
func _build_pov_window() -> void:
	_pov_win = Window.new()
	_pov_win.title = "Robot POV — what the camera sees (ogma_host :%d)" % VIDEO_PORT
	_pov_win.size = Vector2i(640, 520)
	_pov_win.min_size = Vector2i(200, 180)
	_pov_win.position = Vector2i(320, 110)
	_pov_win.visible = false
	_pov_win.close_requested.connect(func(): _pov_win.hide(); _pov_btn.button_pressed = false)
	add_child(_pov_win)
	var bg := ColorRect.new(); bg.color = Color(0.06, 0.06, 0.07)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_pov_win.add_child(bg)
	var vb := VBoxContainer.new()
	vb.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_pov_win.add_child(vb)
	_pov_tex = TextureRect.new()
	_pov_tex.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_pov_tex.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	_pov_tex.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_pov_tex.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vb.add_child(_pov_tex)
	_pov_lbl = _lbl("not connected", 11)
	vb.add_child(_pov_lbl)


func _on_pov_toggle() -> void:
	_pov_win.visible = _pov_btn.button_pressed


# Bottom-right POWER panel from the robot's telemetry (see _pw_panel).
func _build_power_panel() -> void:
	var mono := SystemFont.new()
	mono.font_names = PackedStringArray(["DejaVu Sans Mono", "Liberation Mono", "Monospace"])
	_pw_panel = VBoxContainer.new()
	_pw_panel.name = "RobotPowerPanel"
	_pw_panel.anchor_left = 1.0; _pw_panel.anchor_right = 1.0
	_pw_panel.anchor_top = 1.0; _pw_panel.anchor_bottom = 1.0
	_pw_panel.offset_right = -14; _pw_panel.offset_left = -14 - 560
	_pw_panel.offset_bottom = -38; _pw_panel.offset_top = -38 - 150
	_pw_panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_ui.add_child(_pw_panel)
	var title := _lbl("POWER — the robot (INA219 · A4 pack divider · rail)", 11)
	title.add_theme_color_override("font_color", Color(0.7, 0.7, 0.75))
	_pw_panel.add_child(title)
	for _k in range(3):
		var l := _lbl("", 12)
		l.add_theme_font_override("font", mono)
		_pw_panel.add_child(l)
		_pw_lbls.append(l)
	_power_graph = (load("res://scripts/current_graph.gd") as Script).new()
	_power_graph.custom_minimum_size = Vector2(560, 76)
	_pw_panel.add_child(_power_graph)
	_ui.move_child(_imu_scope, -1)


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


# Whole-robot current (BOM §3), in the bottom-right panel. Instrument only.
func _update_power_row() -> void:
	if _pw_lbls.size() != 3:
		return
	var ina_v: Variant = _tele.get("ina")
	var vbat := float(_tele.get("vbat", 0.0))
	var rail: Dictionary = _tele.get("rail", {}) if _tele.get("rail") is Dictionary else {}
	if not (ina_v is Dictionary) or not bool((ina_v as Dictionary).get("ok", false)):
		_pw_lbls[0].text = ("power —   pack %5.3f V (A4)   %s" % [vbat, "no INA219, or a daemon that predates it" if ina_v == null else "INA219 not reading"]) if _connected else "power —   not connected"
		_pw_lbls[0].add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
		_pw_lbls[1].text = ""; _pw_lbls[2].text = ""
		return
	var ina: Dictionary = ina_v
	var amps := float(ina.get("i_a", 0.0))
	var ema := float(ina.get("i_ema", 0.0))
	var peak := float(ina.get("i_peak", 0.0))
	var charging := bool(ina.get("charging", false))
	var busy := bool(_tele.get("pose_move_active", false)) or bool(_tele.get("rescue_active", false))
	# Push once per telemetry frame, not once per rendered frame: at 60 fps against a
	# 10 Hz daemon the trace would be six copies of every sample and the window would
	# cover 10 s while claiming 60.
	var seq := int(_tele.get("seq", -1))
	var new_frame := seq != _power_seq
	if _power_graph and new_frame:
		_power_seq = seq
		_power_graph.push(amps, ema, peak, charging, busy)
	# Same lines, same formats as picrawler-dash's power rows (and the sim's panel).  Pack is
	# the INA219's bus voltage; A4 (the divider) rides beside it — they are two paths to one
	# number, and the self-check below gates their agreement.
	_pw_lbls[0].text = "power %+6.3f A   pack %5.3f V (A4 %5.3f)   rail %4.2f V (min1s %4.2f)%s" % [
		amps, float(ina.get("v", 0.0)), vbat, float(rail.get("v", 0.0)), float(rail.get("min_1s", 0.0)),
		"   CHARGING" if charging else ""]
	_pw_lbls[1].text = "slow  ema30 %+6.3f A   peak60 %5.3f   max %5.3f   spent %+7.1f A·s" % [
		ema, peak, float(ina.get("i_max", 0.0)), float(ina.get("charge_as", 0.0))]
	# Joint line speed from the daemon's CURRENT pulses (after slew) — the same 10 Hz
	# measure as the sim panel's, here in commanded µs because the servos report nothing.
	var servos: Array = _tele.get("servos", [])
	if new_frame and servos.size() == 12:
		var now := Time.get_ticks_msec()
		var us: Array = []
		var at_lim := 0
		for s in servos:
			var d: Dictionary = s if s is Dictionary else {}
			var c := int(d.get("current_us", 0))
			us.append(c)
			if c > 0 and (c <= int(d.get("min_us", 0)) or c >= int(d.get("max_us", 9999))): at_lim += 1
		if _pw_prev_us.size() == 12 and _pw_prev_ms > 0 and now > _pw_prev_ms:
			var dt := float(now - _pw_prev_ms) / 1000.0
			var spd := 0.0
			var spd_max := 0.0
			for k in range(12):
				var v := absf(float(us[k]) - float(_pw_prev_us[k])) / US_PER_RAD / dt
				spd += v / 12.0
				spd_max = maxf(spd_max, v)
			_pw_lbls[2].text = "joints  line speed %4.2f rad/s mean, %4.2f max (10 Hz)   at limit %d" % [spd, spd_max, at_lim]
		_pw_prev_us = us
		_pw_prev_ms = now
	var col := Color(0.9, 0.9, 0.9)
	if charging: col = Color(0.55, 0.95, 0.55)
	elif amps >= 3.0: col = Color(1, 0.3, 0.3)
	elif amps >= 2.25: col = Color(1, 0.85, 0.4)
	_pw_lbls[0].add_theme_color_override("font_color", col)


# Forward ultrasonic (from ogma_host, see HOST_CTRL_PORT).  The front legs' hip1 rides on the
# same row because the question it serves is a GATE: at what hip1 does a leg enter the beam?
func _update_range_row() -> void:
	_range_mutex.lock()
	var rep: Dictionary = _range_reply
	var rep_ms := _range_reply_ms
	_range_mutex.unlock()
	var lbl: Label = _tele_lbls["range"]
	var r: Variant = rep.get("range")
	if not _range_run:
		lbl.text = "range: —   (connect; read from ogma_host :%d)" % HOST_CTRL_PORT
		lbl.add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
		return
	if not (r is Dictionary):
		lbl.text = "range: no reply from ogma_host :%d (%s)" % [HOST_CTRL_PORT, str(rep.get("error", "waiting"))]
		lbl.add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
		return
	var rd: Dictionary = r
	var m := float(rd.get("m", 0.0))
	var valid := bool(rd.get("valid", false))
	var up := bool(rd.get("up", false))
	var hz := float(rd.get("hz", 0.0))
	var dz := float(rd.get("deadzone_m", 0.0))
	if rep_ms != _range_pushed_ms and _range_graph:
		_range_pushed_ms = rep_ms
		_range_graph.set("deadzone_m", dz)
		_range_graph.push(m, valid and up, hz)
	# The raw reading, always; "→ deadzoned" says what the BRAIN was handed instead.
	lbl.text = "range: %s   %.1f Hz%s" % [("%5.3f m" % m) if valid else "no echo (>%.1f m / glancing)" % m, hz,
		("   deadzone %.3f m%s" % [dz, "  → DEADZONED (brain: no echo)" if bool(rd.get("deadzoned", false)) else ""]) if dz > 0.0 else ""]
	lbl.text += "\n  hip1 " + _front_hip1_text()
	lbl.tooltip_text = "pings %d   no-echo %d   dropped %d   (ogma_host host_sensors)" % [
		int(rd.get("pings", 0)), int(rd.get("timeouts", 0)), int(rd.get("dropped", 0))]
	lbl.mouse_filter = Control.MOUSE_FILTER_PASS
	var col := Color(0.9, 0.9, 0.9)
	if not up: col = Color(1, 0.3, 0.3)
	elif Time.get_ticks_msec() - rep_ms > 1500: col = Color(1, 0.85, 0.4)
	lbl.add_theme_color_override("font_color", col)


# The physical front legs' hip1, from the map rows (LOAD MAP names the channels): the pulse
# the daemon is driving now, and the angle it means through that row's sign and origin.
func _front_hip1_text() -> String:
	var servos: Array = _tele.get("servos", [])
	var parts: Array = []
	for leg in ["FL", "FR"]:
		for ch in range(12):
			var d: Dictionary = _rows[ch]
			if _row_phys(d) == leg and JOINT_OPTIONS[_row_joint_idx(d)] == "hip1":
				var us: float = d["slider"].value
				if ch < servos.size() and servos[ch] is Dictionary and int(servos[ch].get("current_us", 0)) > 0:
					us = float(servos[ch].get("current_us", us))
				parts.append("%s %4dµs %+.2f" % [leg, int(us), _row_sign(d) * (us - _row_origin(d)) / US_PER_RAD])
	return ("  ".join(parts) + " rad") if parts.size() else "— LOAD MAP to name the channels"


func _range_start(host: String) -> void:
	_range_stop()
	_range_host = host
	_range_run = true
	_range_thread = Thread.new()
	_range_thread.start(_range_worker)


func _range_stop() -> void:
	_range_run = false
	if _range_thread != null and _range_thread.is_started():
		_range_thread.wait_to_finish()
	_range_thread = null
	_range_mutex.lock(); _range_reply = {}; _range_reply_ms = -1; _range_mutex.unlock()


# The worker: its own socket, used only on this thread.  ogma_host's ControlServer speaks
# PLAIN TCP, one JSON object per line (picrawler_dash.py's Control) — not ZMQ like benchd.
# host_sensors reads the raw sensor values and changes nothing (SPEC §1.1 holds).  Any
# failure drops the socket and the next poll reconnects, so an ogma_host restart heals.
func _range_worker() -> void:
	var tcp: StreamPeerTCP = null
	var buf := ""
	while _range_run:
		var rep: Dictionary = {}
		if tcp == null:
			tcp = StreamPeerTCP.new()
			buf = ""
			if tcp.connect_to_host(_range_host_ip(), HOST_CTRL_PORT) != OK:
				tcp = null
				rep = {"error": "connect failed"}
		if tcp != null:
			var t0 := Time.get_ticks_msec()
			while tcp.get_status() == StreamPeerTCP.STATUS_CONNECTING and Time.get_ticks_msec() - t0 < 1500:
				tcp.poll(); OS.delay_msec(5)
			if tcp.get_status() != StreamPeerTCP.STATUS_CONNECTED:
				tcp = null
				rep = {"error": "no connection to :%d" % HOST_CTRL_PORT}
			else:
				tcp.put_data(('{"verb":"host_sensors"}\n').to_utf8_buffer())
				t0 = Time.get_ticks_msec()
				while buf.find("\n") < 0 and Time.get_ticks_msec() - t0 < 1000:
					tcp.poll()
					var n := tcp.get_available_bytes()
					if n > 0:
						var got: Array = tcp.get_partial_data(n)
						if int(got[0]) == OK: buf += (got[1] as PackedByteArray).get_string_from_utf8()
					else:
						OS.delay_msec(2)
				var nl := buf.find("\n")
				if nl < 0:
					tcp.disconnect_from_host(); tcp = null
					rep = {"error": "timeout"}
				else:
					var line := buf.substr(0, nl)
					buf = buf.substr(nl + 1)
					var parsed: Variant = JSON.parse_string(line)
					rep = parsed if parsed is Dictionary else {"error": "bad reply"}
		_range_mutex.lock(); _range_reply = rep; _range_reply_ms = Time.get_ticks_msec(); _range_mutex.unlock()
		OS.delay_msec(RANGE_POLL_MS)
	if tcp != null:
		tcp.disconnect_from_host()


# StreamPeerTCP wants an address; resolve the host name once per connection attempt.
func _range_host_ip() -> String:
	if _range_host.is_valid_ip_address():
		return _range_host
	var ip := IP.resolve_hostname(_range_host, IP.TYPE_IPV4)
	return ip if ip != "" else _range_host


func _exit_tree() -> void:
	_range_stop()


# Counts -> grams through FSR_CURVE (BrainInputs FsrModel; below 30 g the curve is unmeasured).
func _fsr_grams(counts: float) -> float:
	if counts <= float(FSR_CURVE[0][0]): return 0.0
	for k in range(1, FSR_CURVE.size()):
		if counts <= float(FSR_CURVE[k][0]):
			var p: Array = FSR_CURVE[k - 1]; var q: Array = FSR_CURVE[k]
			return float(p[1]) + (float(q[1]) - float(p[1])) * (counts - float(p[0])) / (float(q[0]) - float(p[0]))
	var p2: Array = FSR_CURVE[FSR_CURVE.size() - 2]; var q2: Array = FSR_CURVE[FSR_CURVE.size() - 1]
	return float(q2[1]) + (float(q2[1]) - float(p2[1])) * (counts - float(q2[0])) / (float(q2[0]) - float(p2[0]))


# Belly clearance (BOM §2 #4). Instrument only — the height homeostat this will
# eventually feed lives in the brain, not here.
#
# THE STATUS IS NOT DECORATION. A ToF measurement that failed the part's own checks is
# not a large distance or a small one, it is an arbitrary one, and at a consumer it
# looks exactly like a good reading. So the invalid rate gets its own number beside the
# millimetres, and a bad reading colours the row rather than quietly moving it.
func _update_belly_row() -> void:
	var tof_v: Variant = _tele.get("tof")
	if not (tof_v is Dictionary) or not bool((tof_v as Dictionary).get("ok", false)):
		_tele_lbls["belly"].text = ("belly: —   (no VL53L0X, or a daemon that predates it)"
			if tof_v == null else "belly: VL53L0X not reading")
		_tele_lbls["belly"].add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
		return
	var tof: Dictionary = tof_v
	var m := float(tof.get("m", 0.0))
	var valid := bool(tof.get("valid", false))
	var bad := float(tof.get("bad_frac", 0.0))
	var age := int(tof.get("age_ms", -1))
	# raw is shown beside the derived clearance because the offset between them is a
	# FIT, not a fact — seeing both is how a drifting mount gets noticed.
	_tele_lbls["belly"].text = "belly: %5.1f mm  (raw %d mm − %.0f offset)   %s   ema30 %5.1f  worst60 %5.1f  min %5.1f mm" % [
		m * 1000.0, int(tof.get("raw_mm", 0)), float(tof.get("offset_mm", 0.0)),
		str(tof.get("status", "?")),
		float(tof.get("m_ema", 0.0)) * 1000.0, float(tof.get("m_min", 0.0)) * 1000.0,
		float(tof.get("m_min_all", 0.0)) * 1000.0]
	_tele_lbls["belly"].text += "\n       signal %.2f / ambient %.2f Mcps   spads %.0f   invalid %.0f%%   age %s" % [
		float(tof.get("signal_mcps", 0.0)), float(tof.get("ambient_mcps", 0.0)),
		float(tof.get("spads", 0.0)), bad * 100.0,
		"—" if age < 0 else "%d ms" % age]
	# Once per telemetry FRAME, not per rendered frame — same reason as the current
	# graph: at 60 fps against a 10 Hz daemon the window would claim 60 s and hold 10.
	var seq := int(_tele.get("seq", -1))
	if _belly_graph and seq != _belly_seq:
		_belly_seq = seq
		var busy := bool(_tele.get("pose_move_active", false)) or bool(_tele.get("rescue_active", false))
		_belly_graph.push(m, float(tof.get("m_ema", 0.0)), float(tof.get("m_min", 0.0)),
			bad, valid and age >= 0 and age < 2000, busy)
	# COLOUR REPORTS THE INSTRUMENT, NOT THE WORLD. A belly on the ground is a true
	# reading of a real state, and colouring it red says "this number is wrong" about the
	# one moment the number matters most. The trace already carries that: it turns red
	# inside the drag band, which is where a state belongs. So this row stays the same
	# 0.9 grey as vbat and power unless the CHANNEL is in trouble —
	#   red   = ranging has stopped, i.e. the number on screen is stale
	#   amber = the part is rejecting a sustained fraction of its own readings
	# A single invalid sample is deliberately NOT amber: one frame of "saw nothing" is a
	# world state too, and flashing on it would make an open floor look like a fault.
	var col := Color(0.9, 0.9, 0.9)
	if bad > TOF_BAD_FRAC_WARN: col = Color(1, 0.85, 0.4)
	if age > 2000: col = Color(1, 0.3, 0.3)
	_tele_lbls["belly"].add_theme_color_override("font_color", col)


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
		_range_start(_host_edit.text)

func _on_disconnect() -> void:
	# Disconnect is an ACTION, not a closed window (SPEC §4.2.2).  In bench mode the
	# daemon's deadman limps the robot when the pings stop — say so.
	var mode := str(_tele.get("mode", "bench"))     # read BEFORE the frame is cleared
	_client.call("disconnect_from")
	_range_stop()
	_connected = false
	_tele = {}; _tele_ms = -1
	for d in _rows: d["armed"] = false
	if mode == "bench":
		_set_status("disconnected — bench deadman on the Pi sends the rescue pose within ~1 s")
	else:
		_set_status("disconnected — %s mode has NO deadman: the robot carries on (SPEC §4.2)" % mode)

# ======================================================================================
# STOP / resume — SPACE
# ======================================================================================
# _input, not _unhandled_input: it runs BEFORE any control sees the key, so SPACE stops
# the robot even while a text field or slider has focus.  The cost is that the pose-name
# field cannot take a space, which is the right trade for a safety key.
func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo \
			and (event as InputEventKey).keycode == KEY_SPACE:
		get_viewport().set_input_as_handled()
		_toggle_stop()

# Is the robot KNOWN to be stopped right now?  Only fresh evidence counts: a stop/resume
# reply in the last second, or telemetry under a second old.  Unknown is treated as
# "not stopped", so an uncertain press always sends STOP and never RESUME.
func _known_stopped() -> bool:
	var now := Time.get_ticks_msec()
	var tele_age: int = (now - _tele_ms) if _tele_ms >= 0 else 1 << 30
	var reply_age: int = (now - _stop_reply_ms) if _stop_reply_ms >= 0 else 1 << 30
	if reply_age < 1000 and (reply_age <= tele_age):
		return _stop_reply_state
	if tele_age < 1000:
		return bool(_tele.get("stopped", false))
	return false

func _toggle_stop() -> void:
	if not _connected:
		_set_status("⚠ NOT CONNECTED — STOP cannot reach the robot. Connect, or cut power.")
		return
	var resuming := _known_stopped()
	var rep := _call({"verb": "resume" if resuming else "stop"})
	if bool(rep.get("ok", false)):
		_stop_reply_ms = Time.get_ticks_msec()
		_stop_reply_state = bool(rep.get("stopped", not resuming))
		_pending.clear()
		if resuming:
			_set_status("RESUMED (%s mode)" % str(rep.get("mode", "?")))
		else:
			for d in _rows: d["armed"] = false
			_set_status("STOPPED — every servo frozen where it is. SPACE to resume.")
	elif not resuming:
		_set_status("⚠ STOP FAILED: %s — cut power if the robot is at risk" % str(rep.get("error", "no reply")))

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
	var pov_on := _pov_win != null and _pov_win.visible
	if not pov_on and _tele_content != null and not _tele_content.visible:
		return
	if not bool(_video.call("poll")):
		return
	var vi: Image = _video.call("view_image")
	if pov_on and vi != null and not vi.is_empty():
		_pov_tex.texture = ImageTexture.create_from_image(vi)
	var bi: Image = _video.call("brain_image")
	if bi != null and not bi.is_empty():
		_brain_tex.texture = ImageTexture.create_from_image(bi)
	var info: Dictionary = _video.call("info")
	_video_lbl.text = "frame %s   %dx%d source   %.1f kB" % [
		str(info.get("seq", "—")), int(info.get("src_w", 0)), int(info.get("src_h", 0)),
		float(info.get("bytes", 0)) / 1024.0]
	if pov_on:
		_pov_lbl.text = "  " + _video_lbl.text + "   (OV5647 measured 56.2 x 42.1 deg; the brain gets the centre 42.1 deg square)"


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
			if _imu_src != null:
				_imu_src.call("set_frame", t)
	_update_labels()
	_drive_body()


func _on_imu_toggle() -> void:
	if _imu_scope == null:
		return
	_imu_scope.visible = _imu_btn.button_pressed
	# The POWER panel shares the bottom-right corner: step it left while the scope is up.
	if _pw_panel != null:
		var shift := (_imu_scope.offset_right - _imu_scope.offset_left + 12.0) if _imu_scope.visible else 0.0
		_pw_panel.offset_right = -14 - shift
		_pw_panel.offset_left = -14 - 560 - shift
	# Say WHY it is empty rather than drawing an idle panel: an absent part and a
	# disconnected daemon look identical on a scope showing zeros.
	if _imu_btn.button_pressed and (_imu_src == null or not bool(_imu_src.call("has_data"))):
		print("BenchDashboard: IMU scope on, but no imu block in telemetry — ",
			"part absent, daemon predates it, or link down.")


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
	# SPEC §4.2.1: "am I in a mode where closing the laptop kills the robot?" must never be
	# ambiguous.  bench = the deadman sends the rescue pose; dev/autonomous = it does not.
	if mode == "bench" or mode == "—":
		_mode_lbl.text = "MODE: %s — link loss ⇒ RESCUE POSE (%s)" % [mode, "saved" if _tele.get("rescue_pose") != null else "NONE SAVED — save a pose named rescue"]
		_mode_lbl.add_theme_color_override("font_color", Color(1, 0.85, 0.3))
	else:
		var br: Variant = _tele.get("brain")
		var br_txt := "no brain stream"
		if br is Dictionary:
			var d: Dictionary = br
			br_txt = "brain %s, %d applied, age %s ms" % ["HOLDING (stream lost)" if bool(d.get("holding", false)) else ("streaming" if bool(d.get("have_stream", false)) else "silent"),
				int(d.get("applied", 0)), str(d.get("age_ms", "—"))]
		_mode_lbl.text = "MODE: %s — THE BRAIN DRIVES; closing this window does NOT stop it (%s)" % [mode.to_upper(), br_txt]
		_mode_lbl.add_theme_color_override("font_color", Color(1, 0.4, 1))
	var stopped := _known_stopped()
	if not _connected or _tele.is_empty():
		_stop_lbl.text = ""
		_stop_btn.text = "■ STOP  [space]"
	elif stopped:
		_stop_lbl.text = "■ STOPPED (%s, %.0f s) — SPACE to resume      " % [str(_tele.get("stop_why", "?")), float(_tele.get("stopped_ms", 0)) / 1000.0]
		_stop_lbl.add_theme_color_override("font_color", Color(1, 0.3, 0.3))
		_stop_btn.text = "▶ RESUME  [space]"
	else:
		_stop_lbl.text = "● RUNNING      "
		_stop_lbl.add_theme_color_override("font_color", Color(0.3, 1, 0.3))
		_stop_btn.text = "■ STOP  [space]"
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
				   # null in a brain mode: there is no deadman there (SPEC §4.2)
				   "deadman": float(_tele.get("deadman_ms_left", 0.0)) if _tele.get("deadman_ms_left") != null else -1.0,
				   "age": float(max(age_ms, 0))}
	for i in range(adc.size()): sample["adc%d" % i] = float(adc[i])
	for k in sample: _avg_sum[k] = float(_avg_sum.get(k, 0.0)) + sample[k]
	_avg_n += 1
	if now - _avg_started >= 1000:
		_avg = {}
		for k in _avg_sum: _avg[k] = _avg_sum[k] / float(_avg_n)
		_avg_sum = {}; _avg_n = 0; _avg_started = now
		# FSRs by physical leg, counts and grams (the lower legs in the 3-D view show the
		# same thing as colour); A4 is the pack divider.
		var parts: Array = []
		var legs := ["FL", "FR", "RL", "RR"]
		for i in range(adc.size()):
			var a := float(_avg.get("adc%d" % i, 0.0))
			if i < 4:
				parts.append("%s %4d=%3.0fg" % [legs[i], int(a), _fsr_grams(a)])
			else:
				parts.append("A%d %.2fV" % [i, a * 3.3 / 4095.0])
		_tele_lbls["adc"].text = "fsr: " + ("  ".join(parts) if parts.size() else "—") + "   (1 s mean)"
		_tele_lbls["tick_hz"].text = "tick_hz: %.2f   (HAT frame 49.95 Hz — a different clock)" % float(_avg.get("tick_hz", 0.0))
		_tele_lbls["deadman_ms_left"].text = ("deadman_ms_left: %d" % int(_avg.get("deadman", 0.0))) if _tele.get("deadman_ms_left") != null else "deadman: none (brain mode — SPEC §4.2)"
		_tele_lbls["age"].text = "frame seq %s   age %d ms (1 s mean)" % [str(_tele.get("seq", "—")), int(_avg.get("age", 0.0))]
	_update_cost_rows()
	_update_power_row()
	_update_range_row()
	_update_belly_row()
	_tele_lbls["overruns"].text = "overruns: %s   bus_errors: %s" % [str(_tele.get("overruns", "—")), str(_tele.get("bus_errors", "—"))]
	_tele_lbls["watchdog_trips"].text = "watchdog_trips: %s   low_battery: %s   pi_throttled: %s" % [str(_tele.get("watchdog_trips", "—")), str(_tele.get("low_battery", "—")), str(_tele.get("pi_throttled", "—"))]
	_tele_lbls["armed_ch"].text = "armed_ch: %s   rescue_pose: %s" % [str(_tele.get("armed_ch", "—")), str(_tele.get("rescue_pose", "NONE"))]
	var cal_ch = _tele.get("cal_ch", -1)
	_tele_lbls["cal_ch"].text = "cal_ch: %s   cal_ms_left: %s" % [str(cal_ch), str(_tele.get("cal_ms_left", "—"))]
	_widen_lbl.text = "widened: %s" % ("none" if cal_ch < 0 else "P%d (%s ms left)" % [cal_ch, str(_tele.get("cal_ms_left", "?"))])

	# self-check
	_set_check("0x14 present", _tele_fresh(), "live" if _tele_fresh() else "no telemetry")
	_set_check("Vbat plausible 6.0–8.4 V", _tele_fresh() and vbat >= VBAT_MIN and vbat <= VBAT_MAX, "%.2f V" % vbat)
	# INA219 ⟷ A4 — two independent paths to one number, so a disagreement is a real
	# fault rather than a calibration opinion. Same 150 mV gate `hat_tool ina probe`
	# exits non-zero on (BOM §3.4). The other three are still genuinely unfitted.
	var chk_ina: Variant = _tele.get("ina")
	if chk_ina is Dictionary and bool((chk_ina as Dictionary).get("ok", false)):
		var iv := float((chk_ina as Dictionary).get("v", 0.0))
		var dv := iv - vbat
		_set_check("INA219 ⟷ A4 agree", _tele_fresh() and absf(dv) < INA_A4_TOL_V,
			"%.3f V vs %.3f V   Δ%+.3f V" % [iv, vbat, dv])
	else:
		_check_lbls["INA219 ⟷ A4 agree"].text = "○ INA219 ⟷ A4 agree — not fitted"
		_check_lbls["INA219 ⟷ A4 agree"].add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
	# ToF plausible — the tape-measure check's standing equivalent (BOM §6 step 4). It
	# asks two things, because either alone passes on a broken sensor: is the distance
	# in the band a belly can actually be at, and is the part accepting its own
	# measurements? A ToF wedged at a plausible number with a 90 % invalid rate reads as
	# perfect on distance alone.
	var chk_tof: Variant = _tele.get("tof")
	if chk_tof is Dictionary and bool((chk_tof as Dictionary).get("ok", false)):
		var t: Dictionary = chk_tof
		var tm := float(t.get("m", 0.0))
		var tbad := float(t.get("bad_frac", 0.0))
		var tage := int(t.get("age_ms", -1))
		var live := tage >= 0 and tage < 2000
		_set_check("ToF plausible",
			_tele_fresh() and live and bool(t.get("valid", false))
				and tm >= TOF_PLAUSIBLE_MIN_M and tm <= TOF_PLAUSIBLE_MAX_M
				and tbad <= TOF_BAD_FRAC_WARN,
			"%.1f mm   %s   invalid %.0f%%%s" % [tm * 1000.0, str(t.get("status", "?")),
				tbad * 100.0, "" if live else "   STALE"])
	else:
		_check_lbls["ToF plausible"].text = "○ ToF plausible — not fitted"
		_check_lbls["ToF plausible"].add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
	# IMU: the part answered its identity register and the filter is running.  These were
	# hard-wired "not fitted" from before the ICM-20948 and the FSRs were on the robot.
	var chk_imu: Variant = _tele.get("imu")
	if chk_imu is Dictionary:
		var im: Dictionary = chk_imu
		var who := int(im.get("who_am_i", -1))
		_set_check("IMU WHO_AM_I = 0xEA", _tele_fresh() and bool(im.get("ok", false)) and who == IMU_WHO_AM_I,
			"0x%02X   |a| %.3f g   disagree %.1f°   bias %s" % [who & 0xFF, float(im.get("a_norm_g", 0.0)),
				float(im.get("disagree_deg", 0.0)), "ok" if bool(im.get("bias_valid", false)) else "seeding (keep still)"])
	else:
		_check_lbls["IMU WHO_AM_I = 0xEA"].text = "○ IMU WHO_AM_I = 0xEA — no imu block (part absent, or a daemon that predates it)"
		_check_lbls["IMU WHO_AM_I = 0xEA"].add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))
	# FSR: standing still on the floor, the four feet should carry about one body weight.
	# Only meaningful then — on a stand every foot is unloaded, and that is said, not failed.
	if adc.size() >= 4:
		var g := 0.0
		var touching := 0
		for i in range(4):
			g += _fsr_grams(float(adc[i]))
			if int(adc[i]) >= FSR_CONTACT_COUNTS: touching += 1
		var bw := g / BODY_MASS_G
		if touching == 0:
			_check_lbls["FSR sum ≈ 1.0 BW"].text = "○ FSR sum ≈ 1.0 BW — all four feet unloaded (on a stand?)"
			_check_lbls["FSR sum ≈ 1.0 BW"].add_theme_color_override("font_color", Color(0.75, 0.75, 0.75))
		else:
			_set_check("FSR sum ≈ 1.0 BW", _tele_fresh() and bw >= 0.75 and bw <= 1.25,
				"%.2f BW (%.0f g of %.0f), %d feet in contact — valid standing still" % [bw, g, BODY_MASS_G, touching])
	else:
		_check_lbls["FSR sum ≈ 1.0 BW"].text = "○ FSR sum ≈ 1.0 BW — no adc in telemetry"
		_check_lbls["FSR sum ≈ 1.0 BW"].add_theme_color_override("font_color", Color(0.6, 0.6, 0.6))

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
	# The lower legs show the robot's foot sensors: leg colour unloaded, purple at contact
	# (200 counts), brighter with load.  Stale telemetry clears them rather than freezing.
	var adc: Array = _tele.get("adc", []) if _tele_fresh() else []
	if _body.has_method("set_foot_force_display"):
		for i in range(4):
			var counts := float(adc[FSR_CH_FOR_SIM[i]]) if adc.size() >= 4 else 0.0
			_body.call("set_foot_force_display", i, counts >= FSR_CONTACT_COUNTS, _fsr_grams(counts) / BODY_MASS_G)
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
