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
