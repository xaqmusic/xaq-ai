extends Control
## Side panel for per-servo calibration.  12 servo rows, each with:
##   - colored leg indicator
##   - servo name (e.g. "FL hip2")
##   - position slider in radians
##   - sign toggle button
##   - origin (rad) text field
##   - read-back of the actual measured joint angle
## Plus an Export JSON button that copies the full calibration state to
## the clipboard (and prints it to console for headless capture).
##
## Auto-shown when the picrawler body enters calibration mode.

const _LEG_COLORS: Array = [
    Color(0.95, 0.20, 0.20, 1.0),   # FL red
    Color(0.20, 0.85, 0.20, 1.0),   # FR green
    Color(0.30, 0.50, 0.95, 1.0),   # RL blue
    Color(0.95, 0.85, 0.20, 1.0),   # RR yellow
]
const _LEG_NAMES: Array = ["FL", "FR", "RL", "RR"]
const _SERVO_NAMES: Array = ["hip1", "hip2", "knee"]

var _body: Node = null
var _rows: Array = []   # one entry per servo: {slider, sign_btn, origin_edit, actual_label}
var _spring_rows: Array = []      # joint-compliance sliders (g6dof only)
var _springs_synced: bool = false
var _gang_rows: Array = []
var _gang_readout: Label = null
var _sft_check: CheckBox = null # _build_ui runs before _body resolves — sync on first sight
var _spring_header: Label = null
var _export_text: Label
var _info_label: Label
# Phase 7.x — when checked, the C-mode FK chain in picrawler_body
# reads servo targets from the brain action channels (Premotor +
# CPG blended) each tick instead of the user's slider values.  The
# panel mirrors the live values into the sliders so the operator
# can see the gait waveform applied to the suspended body.
var _cpg_chk: CheckBox = null

func _ready() -> void:
    visible = false
    mouse_filter = Control.MOUSE_FILTER_STOP
    _build_ui()

func _process(_delta: float) -> void:
    _refresh_spring_header()
    _sync_spring_sliders()
    _refresh_gang_readout()
    if _body == null:
        _body = get_tree().get_root().find_child("Picrawler", true, false)
        if _body == null:
            return
    # Auto-toggle visibility based on body's _calibrate_mode OR
    # _motor_test_mode (G key) — both modes use these sliders, just
    # with different downstream behavior (FK write vs motor PD).
    var cal: Variant = _body.get("_calibrate_mode")
    var mtest: Variant = _body.get("_motor_test_mode")
    var should_show: bool = (cal != null and bool(cal)) or (mtest != null and bool(mtest))
    if should_show != visible:
        visible = should_show
    if not visible:
        return
    # Phase 7.x — if CPG-drive is on, mirror servo_targets (which the
    # body overwrites each tick from action channels) into the
    # sliders so the user sees live waveform values.  set_value_no_signal
    # avoids triggering _on_slider_changed and bouncing back.
    var cpg_drive: bool = (_cpg_chk != null and _cpg_chk.button_pressed)
    if cpg_drive:
        var live_targets: Array = _body.servo_targets
        if live_targets != null:
            for k in range(12):
                var v: float = float(live_targets[k])
                _rows[k]["slider"].set_value_no_signal(v)
                _rows[k]["val_lbl"].text = "%+.2f" % v
    # Sync slider values FROM body (so external code changing them updates UI).
    # And update "actual" measurements from the joints.
    for k in range(12):
        var row: Dictionary = _rows[k]
        var leg: int = k / 3
        var jt: int = k % 3
        # Read measured joint angle for display.
        var actual: float = 0.0
        match jt:
            0: actual = _body.call("_relative_angle_world_axis",
                                    _body.get("_chassis"),
                                    _body.get("_coxas")[leg],
                                    Vector3.UP)
            1: actual = _body.call("_relative_angle_world_axis",
                                    _body.get("_coxas")[leg],
                                    _body.get("_uppers")[leg],
                                    _body.get("_hip2_axes")[leg])
            2: actual = _body.call("_relative_angle_world_axis",
                                    _body.get("_uppers")[leg],
                                    _body.get("_lowers")[leg],
                                    _body.get("_knee_axes")[leg])
        row["actual_label"].text = "act: %+.2f" % actual

func _on_spring_changed(v: float, key: String) -> void:
    if _body == null:
        return
    _body.set(key, v)              # setter calls _apply_joint_springs() → live joints
    for r in _spring_rows:
        if r["key"] == key:
            r["label"].text = "%.2f" % v

func _on_sft_toggled(pressed: bool) -> void:
    if _body != null:
        _body.set("spring_follows_target", pressed)

func _on_gang_changed(v: float, key: String) -> void:
    if _body != null:
        _body.set(key, v)
    for r in _gang_rows:
        if r["key"] == key:
            r["label"].text = "%.2f" % v

func _on_gang_pulse(group: int) -> void:
    if _body == null:
        return
    var amp: float = float(_body.get("_gang_pulse_amp"))
    if amp <= 0.0:
        amp = 0.35                                  # sensible default if left at 0
        _body.set("_gang_pulse_amp", amp)
    _body.call("gang_pulse", amp, 15, group)        # ~0.3 s step, then release

func _refresh_gang_readout() -> void:
    if _gang_readout == null or _body == null:
        return
    # Peak-to-peak of the MEASURED joint angle over a rolling ~2 s window: the response,
    # not the command.  Sweep shake Hz and watch these — the maximum is the resonance.
    _gang_readout.text = "response pp (2s):  hip2 %+.4f rad   knee %+.4f rad" % [
        float(_body.get("_gang_pp_hip2")), float(_body.get("_gang_pp_knee"))]

func _sync_spring_sliders() -> void:
    # _build_ui() runs from _ready(), but _body is only resolved lazily in _process —
    # so at build time every slider read 0 and the panel LIED about the live values
    # (the g6dof preset sets hip1 8.0/1.5, hip2 7.0/1.5, knee 7.0/1.5, freeplay 0.1).
    # Pull the real values across once the body appears.
    if _springs_synced or _body == null or _spring_rows.is_empty():
        return
    for r in _spring_rows:
        var v: float = float(_body.get(str(r["key"])))
        r["slider"].set_value_no_signal(v)
        r["label"].text = "%.2f" % v
    if _sft_check != null:
        _sft_check.set_pressed_no_signal(bool(_body.get("spring_follows_target")))
    _springs_synced = true

func _refresh_spring_header() -> void:
    if _spring_header == null or _body == null:
        return
    var be: String = str(_body.get("joint_backend"))
    if be == "g6dof":
        _spring_header.text = "JOINT COMPLIANCE  —  backend: g6dof (springs ACTIVE)"
        _spring_header.add_theme_color_override("font_color", Color(0.5, 1.0, 0.6, 1))
    else:
        _spring_header.text = "JOINT COMPLIANCE  —  backend: %s  ⚠ SPRINGS INERT on this backend" % be
        _spring_header.add_theme_color_override("font_color", Color(1.0, 0.55, 0.4, 1))

func _build_ui() -> void:
    # Backdrop panel — wider (right 45% of screen) so sliders get
    # generous click area.
    var bg := ColorRect.new()
    bg.color = Color(0.08, 0.08, 0.10, 0.92)
    bg.anchor_left = 0.55
    bg.anchor_top = 0.0
    bg.anchor_right = 1.0
    bg.anchor_bottom = 1.0
    bg.mouse_filter = Control.MOUSE_FILTER_STOP
    add_child(bg)

    var vbox := VBoxContainer.new()
    vbox.anchor_left = 0.56
    vbox.anchor_top = 0.02
    vbox.anchor_right = 0.99
    vbox.anchor_bottom = 0.98
    vbox.add_theme_constant_override("separation", 4)
    add_child(vbox)

    var title := Label.new()
    title.text = "Servo Calibration Panel"
    title.add_theme_font_size_override("font_size", 18)
    title.add_theme_color_override("font_color", Color.WHITE)
    vbox.add_child(title)

    _info_label = Label.new()
    _info_label.text = "Slider sets joint angle (rad)\nSign flips direction\nOrigin sets center offset"
    _info_label.add_theme_font_size_override("font_size", 11)
    _info_label.add_theme_color_override("font_color", Color(0.7, 0.7, 0.75, 1))
    vbox.add_child(_info_label)

    var sep := HSeparator.new()
    vbox.add_child(sep)

    # 12 servo rows
    _rows.resize(12)
    for k in range(12):
        var leg: int = k / 3
        var jt: int = k % 3
        var row := HBoxContainer.new()
        row.add_theme_constant_override("separation", 6)
        vbox.add_child(row)
        # Color indicator
        var cr := ColorRect.new()
        cr.color = _LEG_COLORS[leg]
        cr.custom_minimum_size = Vector2(12, 22)
        row.add_child(cr)
        # Name
        var name_lbl := Label.new()
        name_lbl.text = "%s %s" % [_LEG_NAMES[leg], _SERVO_NAMES[jt]]
        name_lbl.add_theme_color_override("font_color", _LEG_COLORS[leg])
        name_lbl.add_theme_font_size_override("font_size", 12)
        name_lbl.custom_minimum_size = Vector2(70, 0)
        row.add_child(name_lbl)
        # Slider — set to expand-fill the row so it gets generous click area.
        # All servos use symmetric ±1.6.  Knee joint limits are now
        # symmetric (±1.7 rad) around the perpendicular rest pose, so the
        # slider can drive the joint to either parallel-to-upper extreme
        # without the asymmetric-limit lockup that pinned the negative
        # side at the gravity equilibrium.
        var slider := HSlider.new()
        slider.min_value = -1.6
        slider.max_value = +1.6
        slider.step = 0.01
        slider.value = 0.0
        slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
        slider.custom_minimum_size = Vector2(200, 24)
        slider.value_changed.connect(_on_slider_changed.bind(k))
        row.add_child(slider)
        # Value display
        var val_lbl := Label.new()
        val_lbl.text = "%+.2f" % 0.0
        val_lbl.add_theme_font_size_override("font_size", 11)
        val_lbl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.9, 1))
        val_lbl.custom_minimum_size = Vector2(40, 0)
        row.add_child(val_lbl)
        # Sign toggle
        var sign_btn := Button.new()
        sign_btn.text = "+"
        sign_btn.custom_minimum_size = Vector2(28, 0)
        sign_btn.toggle_mode = true
        sign_btn.pressed.connect(_on_sign_toggled.bind(k, sign_btn))
        row.add_child(sign_btn)
        # Origin LineEdit — all servos default to 0.00, matching the body's
        # servo_origins[k].  Slider value IS the target joint angle (after
        # sign).  Origin field lets the user shift any servo's centerpoint.
        var origin_edit := LineEdit.new()
        origin_edit.text = "0.00"
        origin_edit.custom_minimum_size = Vector2(50, 0)
        origin_edit.text_submitted.connect(_on_origin_submitted.bind(k))
        row.add_child(origin_edit)
        # Actual reading
        var actual_lbl := Label.new()
        actual_lbl.text = "act: +0.00"
        actual_lbl.add_theme_font_size_override("font_size", 10)
        actual_lbl.add_theme_color_override("font_color", Color(0.65, 0.65, 0.7, 1))
        actual_lbl.custom_minimum_size = Vector2(75, 0)
        row.add_child(actual_lbl)
        _rows[k] = {
            "slider": slider,
            "val_lbl": val_lbl,
            "sign_btn": sign_btn,
            "origin_edit": origin_edit,
            "actual_label": actual_lbl,
        }

    # ---- 2026-08-03 · JOINT SPRING / COMPLIANCE SLIDERS -----------------------------
    # These existed as @exports with a hand-tuned g6dof preset since June but had no UI,
    # so the compliance the operator wanted to explore was unreachable without a restart.
    # The body's setters call _apply_joint_springs(), which writes straight to the live
    # Generic6DOFJoint3D — so these are LIVE, no rebuild and no respawn.
    # ⚠ g6dof BACKEND ONLY.  On the `hinge` backend the joints have no spring params and
    # these do nothing; the header says which backend is actually running.
    var sep_spring := HSeparator.new()
    vbox.add_child(sep_spring)
    _spring_header = Label.new()
    _spring_header.add_theme_font_size_override("font_size", 13)
    _spring_header.add_theme_color_override("font_color", Color(1, 0.85, 0.4, 1))
    vbox.add_child(_spring_header)
    var spring_help := Label.new()
    spring_help.text = "Joint compliance — the springiness to explore BEFORE adding a springy foot.\n" \
        + "Stiffness 0 = rigid.  Operator's June preset: hip1 8.0/1.5, hip2 7.0/1.5, knee 7.0/1.5."
    spring_help.add_theme_font_size_override("font_size", 10)
    spring_help.add_theme_color_override("font_color", Color(0.7, 0.7, 0.75, 1))
    vbox.add_child(spring_help)

    # Toggle: does the spring's equilibrium follow the COMMANDED angle (series-elastic,
    # the default) or sit at mechanical zero (the legacy return-to-neutral behaviour)?
    # Off makes the spring fight every command, which is what the old code did.
    var damp_note := Label.new()
    damp_note.text = ("⚠ THREE damping sources act on a leg — the two below are NOT the spring sliders.\n"
        + "A joint can only go floppy when ALL of them are at 0.")
    damp_note.add_theme_font_size_override("font_size", 10)
    damp_note.add_theme_color_override("font_color", Color(1.0, 0.75, 0.45, 1))
    vbox.add_child(damp_note)

    _sft_check = CheckBox.new()
    _sft_check.text = "spring follows target  (off = legacy return-to-neutral)"
    _sft_check.add_theme_font_size_override("font_size", 11)
    _sft_check.toggled.connect(_on_sft_toggled)
    vbox.add_child(_sft_check)

    for spec in [{"key": "joint_angular_damping", "label": "JOINT damping", "max": 4.0},
                 {"key": "body_damp_scale",       "label": "BODY damp scale", "max": 2.0},
                 {"key": "hip1_spring_stiffness", "label": "hip1 stiffness", "max": 20.0},
                 {"key": "hip1_spring_damping",   "label": "hip1 damping",   "max": 5.0},
                 {"key": "hip2_spring_stiffness", "label": "hip2 stiffness", "max": 20.0},
                 {"key": "hip2_spring_damping",   "label": "hip2 damping",   "max": 5.0},
                 {"key": "knee_spring_stiffness", "label": "knee stiffness", "max": 20.0},
                 {"key": "knee_spring_damping",   "label": "knee damping",   "max": 5.0},
                 {"key": "motor_freeplay_rad",    "label": "freeplay (rad)", "max": 0.5}]:
        var srow := HBoxContainer.new()
        srow.add_theme_constant_override("separation", 6)
        vbox.add_child(srow)
        var slbl := Label.new()
        slbl.text = str(spec["label"])
        slbl.custom_minimum_size = Vector2(110, 0)
        slbl.add_theme_font_size_override("font_size", 11)
        slbl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.9, 1))
        srow.add_child(slbl)
        var ss := HSlider.new()
        ss.min_value = 0.0
        ss.max_value = float(spec["max"])
        ss.step = float(spec["max"]) / 200.0
        ss.custom_minimum_size = Vector2(240, 0)
        ss.size_flags_horizontal = Control.SIZE_EXPAND_FILL
        if _body != null:
            ss.set_value_no_signal(float(_body.get(str(spec["key"]))))
        ss.value_changed.connect(_on_spring_changed.bind(str(spec["key"])))
        srow.add_child(ss)
        var svl := Label.new()
        svl.text = "%.2f" % ss.value
        svl.custom_minimum_size = Vector2(46, 0)
        svl.add_theme_font_size_override("font_size", 11)
        svl.add_theme_color_override("font_color", Color.WHITE)
        srow.add_child(svl)
        _spring_rows.append({"slider": ss, "label": svl, "key": str(spec["key"])})

    # ---- 2026-08-03 · GANGED DRIVE — spring / damping characterisation ---------------
    # Displacing ONE joint says little: the leg is a coupled chain and the body only
    # resonates when a joint GROUP moves together.  PULSE gives a ring-down (damping
    # ratio from successive peaks, natural frequency from the period); SHAKE sweeps a
    # sinusoid so the frequency where peak-to-peak is maximal IS the resonance.
    # Writes servo_targets, so it rides the real PD / freeplay / spring chain.
    var sep_gang := HSeparator.new()
    vbox.add_child(sep_gang)
    var gang_title := Label.new()
    gang_title.text = "GANGED DRIVE  —  all four legs together (G mode only)"
    gang_title.add_theme_font_size_override("font_size", 13)
    gang_title.add_theme_color_override("font_color", Color(0.55, 0.8, 1.0, 1))
    vbox.add_child(gang_title)
    var gang_help := Label.new()
    gang_help.text = "PULSE = step + release, watch the ring-down.  SHAKE = sweep hz for the amplitude peak."
    gang_help.add_theme_font_size_override("font_size", 10)
    gang_help.add_theme_color_override("font_color", Color(0.7, 0.7, 0.75, 1))
    vbox.add_child(gang_help)

    for gspec in [{"key": "_gang_hip2_base", "label": "hip2 base (all)", "min": -1.5, "max": 1.5},
                  {"key": "_gang_knee_base", "label": "knee base (all)", "min": -2.5, "max": 1.7},
                  {"key": "_gang_pulse_amp", "label": "pulse / shake amp", "min": 0.0, "max": 1.0},
                  {"key": "_gang_shake_hz",  "label": "shake Hz (0=off)", "min": 0.0, "max": 12.0},
                  {"key": "_gang_shake_amp", "label": "shake amp",       "min": 0.0, "max": 1.0}]:
        var grow := HBoxContainer.new()
        grow.add_theme_constant_override("separation", 6)
        vbox.add_child(grow)
        var gl := Label.new()
        gl.text = str(gspec["label"])
        gl.custom_minimum_size = Vector2(110, 0)
        gl.add_theme_font_size_override("font_size", 11)
        gl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.9, 1))
        grow.add_child(gl)
        var gs := HSlider.new()
        gs.min_value = float(gspec["min"])
        gs.max_value = float(gspec["max"])
        gs.step = (gs.max_value - gs.min_value) / 200.0
        gs.custom_minimum_size = Vector2(240, 0)
        gs.size_flags_horizontal = Control.SIZE_EXPAND_FILL
        gs.value_changed.connect(_on_gang_changed.bind(str(gspec["key"])))
        grow.add_child(gs)
        var gv := Label.new()
        gv.text = "0.00"
        gv.custom_minimum_size = Vector2(46, 0)
        gv.add_theme_font_size_override("font_size", 11)
        gv.add_theme_color_override("font_color", Color.WHITE)
        grow.add_child(gv)
        _gang_rows.append({"slider": gs, "label": gv, "key": str(gspec["key"])})

    var gbtn_row := HBoxContainer.new()
    gbtn_row.add_theme_constant_override("separation", 6)
    vbox.add_child(gbtn_row)
    for b in [{"t": "PULSE hip2", "g": 1}, {"t": "PULSE knee", "g": 2}, {"t": "PULSE both", "g": 3}]:
        var pb := Button.new()
        pb.text = str(b["t"])
        pb.pressed.connect(_on_gang_pulse.bind(int(b["g"])))
        gbtn_row.add_child(pb)
    _gang_readout = Label.new()
    _gang_readout.add_theme_font_size_override("font_size", 12)
    _gang_readout.add_theme_color_override("font_color", Color(0.6, 1.0, 0.7, 1))
    vbox.add_child(_gang_readout)

    var sep2 := HSeparator.new()
    vbox.add_child(sep2)

    # Buttons row
    var btn_row := HBoxContainer.new()
    btn_row.add_theme_constant_override("separation", 6)
    vbox.add_child(btn_row)
    var export_btn := Button.new()
    export_btn.text = "Export JSON → clipboard"
    export_btn.pressed.connect(_on_export_clicked)
    btn_row.add_child(export_btn)
    var reset_btn := Button.new()
    reset_btn.text = "Reset all sliders"
    reset_btn.pressed.connect(_on_reset_sliders)
    btn_row.add_child(reset_btn)
    # Phase 7.x — CPG-drive checkbox.  When enabled, the body's
    # _cpg_drive_calibrate flag is set; the C-mode FK chain reads
    # servo targets from action channels each tick instead of the
    # user's sliders.  Lets the operator visualise the actual gait
    # waveform applied to the suspended body without it falling.
    _cpg_chk = CheckBox.new()
    _cpg_chk.text = "Drive servos from CPG output"
    _cpg_chk.toggled.connect(_on_cpg_drive_toggled)
    btn_row.add_child(_cpg_chk)

    _export_text = Label.new()
    _export_text.text = ""
    _export_text.autowrap_mode = TextServer.AUTOWRAP_WORD
    _export_text.add_theme_font_size_override("font_size", 10)
    _export_text.add_theme_color_override("font_color", Color(0.6, 0.8, 0.6, 1))
    vbox.add_child(_export_text)

func _on_slider_changed(value: float, k: int) -> void:
    if _body == null: return
    var arr: Array = _body.servo_targets   # direct property access (avoids get() copy semantics)
    if arr == null or k >= arr.size(): return
    arr[k] = value
    _rows[k]["val_lbl"].text = "%+.2f" % value
    var leg: int = k / 3
    var jt: int = k % 3
    print("ServoPanel: %s %s → target = %+.3f (k=%d)" % [
        _LEG_NAMES[leg], _SERVO_NAMES[jt], value, k])

func _on_sign_toggled(k: int, btn: Button) -> void:
    if _body == null: return
    var arr: Array = _body.servo_signs
    if arr == null or k >= arr.size(): return
    var new_sign: float = -1.0 if btn.button_pressed else +1.0
    arr[k] = new_sign
    btn.text = "−" if btn.button_pressed else "+"
    print("ServoPanel: k=%d sign = %s" % [k, new_sign])

func _on_origin_submitted(text: String, k: int) -> void:
    if _body == null: return
    var arr: Array = _body.servo_origins
    if arr == null or k >= arr.size(): return
    var v: float = text.to_float()
    arr[k] = v
    print("ServoPanel: k=%d origin = %s" % [k, v])

func _on_export_clicked() -> void:
    if _body == null: return
    var data: Dictionary = _body.call("export_servo_calibration")
    var json_str: String = JSON.stringify(data, "  ")
    DisplayServer.clipboard_set(json_str)
    print("===== SERVO CALIBRATION EXPORT =====")
    print(json_str)
    print("===== (also copied to clipboard) ====")
    _export_text.text = "Exported — see console / clipboard"

func _on_cpg_drive_toggled(pressed: bool) -> void:
    if _body == null: return
    _body.set("_cpg_drive_calibrate", pressed)
    print("ServoPanel: CPG-drive = %s" % str(pressed))

func _on_reset_sliders() -> void:
    if _body == null: return
    for k in range(12):
        _rows[k]["slider"].value = 0.0
    var arr: Array = _body.get("servo_targets")
    if arr != null:
        for k in range(12):
            arr[k] = 0.0
        _body.set("servo_targets", arr)
