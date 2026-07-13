extends Control
## MotorFaderMeter — α gauge (Phase 6.6.F + 6.6.G).
##
## Polls OgmaBrain.get_motor_fader_state() each frame and draws a
## horizontal red→white→blue gradient bar with numeric readout and
## brain/reflex source-seen dots.  Hides itself when neither a
## FaderController nor a MotorFader is in the graph (configs without
## crossfade get no stub gauge).
##
## Phase 6.6.G — manual M/←/→ patches now target the FaderController
## (which owns alpha_source / alpha_fixed in 6.6.G) instead of the
## MotorFader.  When no FaderController is in the graph (legacy
## single-channel configs) the meter falls back to patching the
## MotorFader's alpha_fixed param directly.
##
## Used by hud.gd (Cell), cartpole_hud.gd, mountain_car_hud.gd.  Construct,
## anchor where you want it (top-right is the convention), call set_brain(b).

const _BAR_W:        int = 200
const _BAR_H:        int = 18
const _LABEL_H:      int = 16
const _DOT_R:        float = 4.0
const _PAD:          int = 6
const _COL_REFLEX  := Color(0.95, 0.42, 0.35)   # red
const _COL_MID     := Color(0.95, 0.95, 0.95)
const _COL_BRAIN   := Color(0.40, 0.62, 0.95)   # blue
const _COL_DOT_OFF := Color(0.40, 0.40, 0.40)
const _COL_BG      := Color(0.10, 0.10, 0.10, 0.55)
const _COL_BORDER  := Color(0.85, 0.85, 0.85, 0.85)
const _COL_TEXT    := Color(1, 1, 1, 1)

var _brain: OgmaBrain = null
var _font: Font = null
var _last_state: Dictionary = {}

# Phase 6.6.F.2 — manual override.  Press M to disconnect the fader from
# its surprise-driven α and take over via ←/→ arrow keys.  Press M again
# to release and let the brain's own surprise drive α back.
var _manual_mode: bool = false
var _manual_alpha: float = 0.5
var _fader_module_id: String = ""           # discovered from brain.get_module_list
var _fader_module_type: String = ""          # "FaderController" (6.6.G) or "MotorFader" (legacy)
const _MANUAL_STEP: float = 0.05             # α delta per arrow press
const _COL_MANUAL_BADGE := Color(0.95, 0.85, 0.35)

func _ready() -> void:
    # Two lines of readout above the bar plus the bar + edge labels.
    custom_minimum_size = Vector2(_BAR_W + 2 * _PAD,
                                  2 * _LABEL_H + _BAR_H + 2 * _PAD + 6)
    size = custom_minimum_size
    mouse_filter = Control.MOUSE_FILTER_IGNORE
    _font = ThemeDB.fallback_font

func set_brain(b: OgmaBrain) -> void:
    _brain = b

func _process(_delta: float) -> void:
    if _brain == null or not _brain.is_brain_ready():
        if visible:
            visible = false
        return
    _last_state = _brain.get_motor_fader_state()
    var should_show: bool = not _last_state.is_empty()
    if visible != should_show:
        visible = should_show
    if visible:
        if _fader_module_id == "":
            _resolve_fader_id()
        queue_redraw()

func _resolve_fader_id() -> void:
    if _brain == null or not _brain.has_method("get_module_list"):
        return
    # Phase 6.6.G — prefer FaderController for manual α patches; in 6.6.G
    # graphs that's the module that owns alpha_source / alpha_fixed.
    # Fall back to MotorFader for legacy single-channel configs that
    # haven't been migrated.
    var fader_ctrl_id: String = ""
    var motor_fader_id: String = ""
    for m_v in _brain.get_module_list():
        var m: Dictionary = m_v
        var t: String = String(m.get("type", ""))
        if t == "FaderController" and fader_ctrl_id == "":
            fader_ctrl_id = String(m.get("id", ""))
        elif t == "MotorFader" and motor_fader_id == "":
            motor_fader_id = String(m.get("id", ""))
    if fader_ctrl_id != "":
        _fader_module_id   = fader_ctrl_id
        _fader_module_type = "FaderController"
    elif motor_fader_id != "":
        _fader_module_id   = motor_fader_id
        _fader_module_type = "MotorFader"
    if _fader_module_id != "":
        _manual_alpha = float(_last_state.get("alpha", 0.5))

func _unhandled_input(event: InputEvent) -> void:
    if not visible or _fader_module_id == "" or _brain == null:
        return
    if not (event is InputEventKey) or not event.pressed or event.echo:
        return
    var key_event: InputEventKey = event
    if key_event.keycode == KEY_M:
        _manual_mode = not _manual_mode
        if _manual_mode:
            _manual_alpha = float(_last_state.get("alpha", 0.5))
            _apply_fader_patch("alpha_fixed", _manual_alpha)
            # alpha_source only exists on FaderController in 6.6.G.
            # When the legacy fallback target is a MotorFader, alpha_fixed
            # alone is the operator's lever (MotorFader has no alpha_source).
            if _fader_module_type == "FaderController":
                _apply_fader_patch("alpha_source", "fixed")
        else:
            if _fader_module_type == "FaderController":
                _apply_fader_patch("alpha_source", "surprise")
        get_viewport().set_input_as_handled()
    elif _manual_mode and key_event.keycode == KEY_LEFT:
        _manual_alpha = clampf(_manual_alpha - _MANUAL_STEP, 0.0, 1.0)
        _apply_fader_patch("alpha_fixed", _manual_alpha)
        get_viewport().set_input_as_handled()
    elif _manual_mode and key_event.keycode == KEY_RIGHT:
        _manual_alpha = clampf(_manual_alpha + _MANUAL_STEP, 0.0, 1.0)
        _apply_fader_patch("alpha_fixed", _manual_alpha)
        get_viewport().set_input_as_handled()

func _apply_fader_patch(key: String, value: Variant) -> void:
    var op: Dictionary = {
        "op":    "set_param",
        "id":    _fader_module_id,
        "key":   key,
        "value": value,
    }
    var result: Dictionary = _brain.apply_patch(op)
    if not bool(result.get("success", false)):
        push_warning("MotorFaderMeter: set_param %s failed — %s" %
            [key, String(result.get("error", "?"))])

func _draw() -> void:
    if _last_state.is_empty():
        return
    var alpha:           float  = float(_last_state.get("alpha", 0.0))
    var surp:            float  = float(_last_state.get("surprise_scalar", 0.0))
    var brain_seen:      bool   = bool(_last_state.get("brain_seen", false))
    var reflex_seen:     bool   = bool(_last_state.get("reflex_seen", false))
    var source:          String = str(_last_state.get("source", ""))

    # ---- Background panel ----
    var rect := Rect2(0, 0, size.x, size.y)
    draw_rect(rect, _COL_BG, true)
    draw_rect(rect, _COL_BORDER, false, 1.0)

    # ---- Numeric readout (top line) ----
    var readout: String = "α=%.2f  surp=%.2f  src=%s" % [alpha, surp, source]
    if _manual_mode:
        readout += "  [MAN ←/→]"
    var fsize: int = 11
    var col_text: Color = _COL_MANUAL_BADGE if _manual_mode else _COL_TEXT
    draw_string(_font, Vector2(_PAD, _PAD + fsize),
                readout, HORIZONTAL_ALIGNMENT_LEFT, -1, fsize, col_text)

    # ---- Raw component readout (second line) — Phase 6.6.F.3.
    # Direct visual validation that the fader is the only path to action.out:
    # `out` should equal α·B + (1-α)·R; brain/reflex columns reveal what each
    # side is contributing this tick.  When the brain side is ablated, B
    # should read 0 (or "—" if the brain hasn't published this tick yet).
    var b_acc: float = float(_last_state.get("brain_accel",  0.0))
    var r_acc: float = float(_last_state.get("reflex_accel", 0.0))
    var o_acc: float = float(_last_state.get("output_accel", 0.0))
    var b_str: String = "%+.2f" % b_acc if brain_seen  else "  — "
    var r_str: String = "%+.2f" % r_acc if reflex_seen else "  — "
    var raw_line: String = "B=%s  R=%s  →  out=%+.2f" % [b_str, r_str, o_acc]
    draw_string(_font, Vector2(_PAD, _PAD + 2 * fsize + 2),
                raw_line, HORIZONTAL_ALIGNMENT_LEFT, -1, 10, _COL_TEXT)

    # ---- Bar geometry ----
    var bar_y: int = _PAD + 2 * _LABEL_H + 2
    var bar_x := _PAD
    var bar_rect := Rect2(bar_x, bar_y, _BAR_W, _BAR_H)
    draw_rect(bar_rect, Color(0, 0, 0, 0.4), true)

    # Fill: red→white→blue gradient based on α.
    # Render as N segments so the gradient is smooth without a Gradient resource.
    var fill_w: int = int(round(_BAR_W * clampf(alpha, 0.0, 1.0)))
    var seg_count: int = maxi(1, fill_w)
    for i in range(seg_count):
        var t: float = float(i) / float(seg_count - 1) if seg_count > 1 else 0.0
        var c: Color
        if t < 0.5:
            c = _COL_REFLEX.lerp(_COL_MID, t / 0.5)
        else:
            c = _COL_MID.lerp(_COL_BRAIN, (t - 0.5) / 0.5)
        draw_rect(Rect2(bar_x + i, bar_y, 1, _BAR_H), c, true)

    # Tick mark at α midpoint for visual reference
    var mid_x := bar_x + _BAR_W / 2
    draw_line(Vector2(mid_x, bar_y - 1),
              Vector2(mid_x, bar_y + _BAR_H + 1),
              Color(1, 1, 1, 0.35), 1.0)

    # Border on top of fill
    draw_rect(bar_rect, _COL_BORDER, false, 1.0)

    # ---- Edge labels (REFLEX | BRAIN) ----
    draw_string(_font,
                Vector2(bar_x, bar_y + _BAR_H + 12),
                "REFLEX",
                HORIZONTAL_ALIGNMENT_LEFT, -1, 9,
                _COL_REFLEX if reflex_seen else _COL_DOT_OFF)
    var brain_text := "BRAIN"
    var brain_text_w := _font.get_string_size(brain_text, HORIZONTAL_ALIGNMENT_LEFT, -1, 9).x
    draw_string(_font,
                Vector2(bar_x + _BAR_W - brain_text_w, bar_y + _BAR_H + 12),
                brain_text,
                HORIZONTAL_ALIGNMENT_LEFT, -1, 9,
                _COL_BRAIN if brain_seen else _COL_DOT_OFF)

    # ---- Status dots (just inside the bar edges, vertically centered)
    var dot_y := bar_y + _BAR_H / 2.0
    draw_circle(Vector2(bar_x + 5, dot_y),                  _DOT_R,
                _COL_REFLEX if reflex_seen else _COL_DOT_OFF)
    draw_circle(Vector2(bar_x + _BAR_W - 5, dot_y),         _DOT_R,
                _COL_BRAIN  if brain_seen  else _COL_DOT_OFF)
