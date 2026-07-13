extends Control
## Live training-pulse injector for the picrawler scene.  Two buttons:
##   - Good Boy (green) — publishes `events.hit`  at current intensity.
##   - Bad Boy  (red)   — publishes `events.miss` at current intensity.
##
## Hotkeys: T (treat = good), X (cross = bad).  Intensity slider clamps
## to [0, 1].  We deliberately avoid G — that's the body's motor-test
## toggle and would fire both actions on a single press.
##
## All pulses route through body.publish_trainer_event(kind, intensity)
## which (a) forwards to brain.publish_event so DA / 5-HT respond just
## like any in-game reward, and (b) increments per-diag-tick counters
## that _emit_jsonl() logs as `trainer_good_count` / `trainer_bad_count`.
## Those counters are the audit trail: headless A/Bs must see 0 for
## both, or the run is contaminated and the result is invalid.
##
## Methodology guard: this panel is part of the picrawler SCENE only —
## headless runs don't instance the HUD, so trainer pulses cannot fire
## accidentally.  The JSONL fields are emitted regardless so analysis
## scripts can assert headless cleanliness with a single column check.

const _BODY_CANDIDATES: Array = ["Picrawler", "Quadruped"]
const _FLASH_DURATION: float  = 0.35      # seconds the button glow holds
const _GOOD_COLOR: Color = Color(0.25, 0.85, 0.35, 1.0)
const _BAD_COLOR:  Color = Color(0.90, 0.30, 0.25, 1.0)

var _body: Node = null
var _intensity: float = 1.0

var _intensity_label: Label
var _intensity_slider: HSlider
var _good_btn:  Button
var _bad_btn:   Button
var _status:    Label
var _auto_reset_cb: CheckBox
var _auto_reset_status: Label
var _outer_wall_cb: CheckBox
var _outer_wall_status: Label
# 2026-06-09 — minimise/expand toggle (▼/▲).
var _minimise_btn: Button = null
var _content_vb:   VBoxContainer = null
var _is_minimised: bool = false
# Trainer is bottom-anchored — collapse shrinks the panel by moving
# offset_top DOWN toward offset_bottom, leaving only the header visible
# at the very bottom of the screen.
var _full_offset_top: float = 0.0
var _full_offset_top_set: bool = false
const _COLLAPSED_HEIGHT: float = 38.0   # header + frame padding

# Per-button flash animation state — _process() lerps button modulate
# toward white as _good_flash_t / _bad_flash_t decay.
var _good_flash_t: float = 0.0
var _bad_flash_t:  float = 0.0

# Recent-pulse log used by _status label.
var _last_pulse_kind: String = ""
var _last_pulse_time: float = -1.0
var _good_total: int = 0
var _bad_total:  int = 0

func _as_bool(v: Variant) -> bool:
    if v == null:
        return false
    match typeof(v):
        TYPE_BOOL:
            return v
        TYPE_INT, TYPE_FLOAT:
            return v != 0
        TYPE_STRING:
            var t: String = str(v).to_lower()
            return t == "true" or t == "1" or t == "yes" or t == "on"
        _:
            return false

func _as_int(v: Variant, fallback: int = 0) -> int:
    if v == null:
        return fallback
    match typeof(v):
        TYPE_INT:
            return v
        TYPE_FLOAT:
            return roundi(v)
        TYPE_BOOL:
            return 1 if v else 0
        TYPE_STRING:
            return str(v).to_int()
        _:
            return fallback


func _ready() -> void:
    # 2026-06-09 — anchored to BOTTOM-LEFT with 12px margins.  Curriculum
    # panel can grow to fit many stages; the trainer panel staying at the
    # bottom keeps the two from overlapping regardless of curriculum size.
    anchor_left   = 0.0
    anchor_top    = 1.0
    anchor_right  = 0.0
    anchor_bottom = 1.0
    offset_left   = 12.0
    offset_top    = -280.0
    offset_right  = 12.0 + 360.0
    offset_bottom = -12.0
    mouse_filter  = Control.MOUSE_FILTER_PASS

    var bg := PanelContainer.new()
    bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
    var bg_style := StyleBoxFlat.new()
    bg_style.bg_color = Color(0.05, 0.05, 0.07, 0.82)
    bg_style.border_color = Color(0.55, 0.55, 0.6, 0.85)
    bg_style.set_border_width_all(1)
    bg_style.set_corner_radius_all(4)
    bg_style.content_margin_left = 10
    bg_style.content_margin_right = 10
    bg_style.content_margin_top = 8
    bg_style.content_margin_bottom = 8
    bg.add_theme_stylebox_override("panel", bg_style)
    add_child(bg)

    var vb := VBoxContainer.new()
    vb.add_theme_constant_override("separation", 6)
    bg.add_child(vb)

    # 2026-06-09 — header row with title + minimise button.
    var header_row := HBoxContainer.new()
    header_row.add_theme_constant_override("separation", 4)
    vb.add_child(header_row)
    var title := Label.new()
    title.text = "TRAINER"
    title.add_theme_font_size_override("font_size", 14)
    title.add_theme_color_override("font_color", Color(0.95, 0.95, 1.0, 1.0))
    title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    header_row.add_child(title)
    _minimise_btn = Button.new()
    _minimise_btn.text = "▼"
    _minimise_btn.custom_minimum_size = Vector2(28, 0)
    _minimise_btn.pressed.connect(_on_minimise_pressed)
    header_row.add_child(_minimise_btn)

    # Content wrapper so the minimise toggle hides everything below the
    # header as one unit.
    _content_vb = VBoxContainer.new()
    _content_vb.add_theme_constant_override("separation", 6)
    vb.add_child(_content_vb)

    _intensity_label = Label.new()
    _intensity_label.add_theme_font_size_override("font_size", 12)
    _intensity_label.add_theme_color_override("font_color", Color(0.85, 0.85, 0.9, 1.0))
    _content_vb.add_child(_intensity_label)

    _intensity_slider = HSlider.new()
    _intensity_slider.min_value = 0.0
    _intensity_slider.max_value = 1.0
    _intensity_slider.step      = 0.05
    _intensity_slider.value     = _intensity
    _intensity_slider.custom_minimum_size = Vector2(0, 16)
    _intensity_slider.value_changed.connect(_on_intensity_changed)
    _content_vb.add_child(_intensity_slider)
    _update_intensity_label()

    var hb := HBoxContainer.new()
    hb.add_theme_constant_override("separation", 6)
    _content_vb.add_child(hb)

    _bad_btn = Button.new()
    _bad_btn.text = "Bad [X]"
    _bad_btn.custom_minimum_size = Vector2(0, 28)
    _bad_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _bad_btn.add_theme_color_override("font_color", _BAD_COLOR)
    _bad_btn.pressed.connect(func() -> void: _fire("bad"))
    hb.add_child(_bad_btn)

    _good_btn = Button.new()
    _good_btn.text = "Good [T]"
    _good_btn.custom_minimum_size = Vector2(0, 28)
    _good_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _good_btn.add_theme_color_override("font_color", _GOOD_COLOR)
    _good_btn.pressed.connect(func() -> void: _fire("good"))
    hb.add_child(_good_btn)

    _status = Label.new()
    _status.add_theme_font_size_override("font_size", 11)
    _status.add_theme_color_override("font_color", Color(0.75, 0.75, 0.8, 1.0))
    _status.text = "no pulses fired"
    _content_vb.add_child(_status)

    # Auto-reset-on-inversion checkbox.  Toggles the body's belly-up
    # safety net.  Separate from trainer pulses (different mechanism,
    # different audit column) — co-located here because both are
    # experimenter-controlled training aids.
    var sep := HSeparator.new()
    sep.add_theme_constant_override("separation", 4)
    _content_vb.add_child(sep)

    _auto_reset_cb = CheckBox.new()
    _auto_reset_cb.text = "Auto-reset on flip"
    _auto_reset_cb.add_theme_font_size_override("font_size", 12)
    _auto_reset_cb.toggled.connect(_on_auto_reset_toggled)
    _content_vb.add_child(_auto_reset_cb)

    _auto_reset_status = Label.new()
    _auto_reset_status.add_theme_font_size_override("font_size", 10)
    _auto_reset_status.add_theme_color_override("font_color", Color(0.7, 0.75, 0.85, 1.0))
    _auto_reset_status.text = "fires when belly-up + chassis on ground (>0.5s dwell)"
    _auto_reset_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    _content_vb.add_child(_auto_reset_status)

    # Phase 6.14 outer-wall reset checkbox.  Same pattern as the flip
    # checkbox — toggle writes body.auto_reset_on_outer_wall; per-frame
    # sync below picks up env-var / external changes.
    _outer_wall_cb = CheckBox.new()
    _outer_wall_cb.text = "Auto-reset on outer wall"
    _outer_wall_cb.add_theme_font_size_override("font_size", 12)
    _outer_wall_cb.toggled.connect(_on_outer_wall_toggled)
    _content_vb.add_child(_outer_wall_cb)

    _outer_wall_status = Label.new()
    _outer_wall_status.add_theme_font_size_override("font_size", 10)
    _outer_wall_status.add_theme_color_override("font_color", Color(0.7, 0.75, 0.85, 1.0))
    _outer_wall_status.text = "fires when chassis r > 9.5 m (just inside the wedges)"
    _outer_wall_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    _content_vb.add_child(_outer_wall_status)

func _process(delta: float) -> void:
    # Late-binding body lookup — same pattern as quadruped_hud.gd because
    # the picrawler body is built programmatically and isn't reachable
    # via NodePath at scene-load time.
    if _body == null or not is_instance_valid(_body):
        for nm in _BODY_CANDIDATES:
            var b: Node = get_tree().get_root().find_child(nm, true, false)
            if b != null:
                _body = b
                break

    if _good_flash_t > 0.0:
        _good_flash_t = max(0.0, _good_flash_t - delta)
        _good_btn.self_modulate = Color(1.0, 1.0, 1.0, 1.0).lerp(
            _GOOD_COLOR, _good_flash_t / _FLASH_DURATION)
    elif _good_btn.self_modulate != Color(1, 1, 1, 1):
        _good_btn.self_modulate = Color(1, 1, 1, 1)

    if _bad_flash_t > 0.0:
        _bad_flash_t = max(0.0, _bad_flash_t - delta)
        _bad_btn.self_modulate = Color(1.0, 1.0, 1.0, 1.0).lerp(
            _BAD_COLOR, _bad_flash_t / _FLASH_DURATION)
    elif _bad_btn.self_modulate != Color(1, 1, 1, 1):
        _bad_btn.self_modulate = Color(1, 1, 1, 1)

    if _last_pulse_time > 0.0:
        var dt: float = Time.get_ticks_msec() / 1000.0 - _last_pulse_time
        _status.text = "last: %s @ %.2f  (good=%d bad=%d)" % [
            _last_pulse_kind, _intensity, _good_total, _bad_total]
        # Fade text after 4 s
        var alpha: float = clamp(1.0 - (dt - 1.0) / 3.0, 0.4, 1.0)
        _status.modulate = Color(1, 1, 1, alpha)

    # Sync auto-reset checkbox + status from the body each frame.  Cheap;
    # also picks up any env-var or curriculum-driven changes.
    if _body != null and is_instance_valid(_body):
        var en: bool = _as_bool(_body.get("auto_reset_on_inversion"))
        if en != _auto_reset_cb.button_pressed:
            _auto_reset_cb.set_pressed_no_signal(en)
        var n: int = _as_int(_body.get("auto_reset_count"))
        if en:
            _auto_reset_status.text = "fires belly-up + on ground (>0.5s dwell) — fired %d×" % n
        else:
            _auto_reset_status.text = "fires when belly-up + chassis on ground (>0.5s dwell)"
        # Phase 6.14 — sync outer-wall checkbox + status.
        var ow_en_v: Variant = _body.get("auto_reset_on_outer_wall")
        var ow_en: bool = (ow_en_v != null and _as_bool(ow_en_v))
        if ow_en != _outer_wall_cb.button_pressed:
            _outer_wall_cb.set_pressed_no_signal(ow_en)
        if ow_en:
            _outer_wall_status.text = "fires when chassis r > 9.5 m (auto_reset_count shared with flip)"
        else:
            _outer_wall_status.text = "fires when chassis r > 9.5 m (just inside the wedges)"

func _unhandled_input(event: InputEvent) -> void:
    if not (event is InputEventKey):
        return
    var ke: InputEventKey = event
    if not ke.pressed or ke.echo:
        return
    if ke.keycode == KEY_T:
        _fire("good")
        get_viewport().set_input_as_handled()
    elif ke.keycode == KEY_X:
        _fire("bad")
        get_viewport().set_input_as_handled()

func _fire(kind: String) -> void:
    if _body == null or not is_instance_valid(_body):
        return
    if not _body.has_method("publish_trainer_event"):
        # Body doesn't expose the trainer hook — bail rather than reach
        # past the audit-tagged entry point.  This is the methodology
        # guardrail: trainer pulses must always be traceable.
        print("TrainerPanel: body has no publish_trainer_event method — pulse dropped")
        return
    _body.call("publish_trainer_event", kind, _intensity)
    _last_pulse_kind = kind
    _last_pulse_time = Time.get_ticks_msec() / 1000.0
    if kind == "good":
        _good_flash_t = _FLASH_DURATION
        _good_total += 1
    else:
        _bad_flash_t  = _FLASH_DURATION
        _bad_total  += 1

func _on_intensity_changed(v: float) -> void:
    _intensity = clamp(v, 0.0, 1.0)
    _update_intensity_label()

func _on_auto_reset_toggled(pressed: bool) -> void:
    if _body == null or not is_instance_valid(_body):
        return
    _body.set("auto_reset_on_inversion", pressed)
    print("TrainerPanel: auto_reset_on_inversion = %s" % str(pressed))

func _on_outer_wall_toggled(pressed: bool) -> void:
    if _body == null or not is_instance_valid(_body):
        return
    _body.set("auto_reset_on_outer_wall", pressed)
    print("TrainerPanel: auto_reset_on_outer_wall = %s" % str(pressed))

func _update_intensity_label() -> void:
    _intensity_label.text = "Intensity: %.2f" % _intensity

# 2026-06-09 — minimise/expand toggle.  Trainer is BOTTOM-anchored
# (anchor_top = 1.0) so collapse moves offset_top DOWN toward offset_bottom,
# leaving only the header at the bottom of the screen.
func _on_minimise_pressed() -> void:
    if _content_vb == null or _minimise_btn == null:
        return
    _is_minimised = not _is_minimised
    _content_vb.visible = not _is_minimised
    _minimise_btn.text = "▲" if _is_minimised else "▼"
    if _is_minimised:
        if not _full_offset_top_set:
            _full_offset_top = offset_top
            _full_offset_top_set = true
        offset_top = offset_bottom - _COLLAPSED_HEIGHT
    else:
        if _full_offset_top_set:
            offset_top = _full_offset_top
