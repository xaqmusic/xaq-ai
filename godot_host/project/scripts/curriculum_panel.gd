extends Control
## Curriculum visualisation for the picrawler HUD.
##
##   - Header showing "CURRICULUM (current_idx / total)".
##   - Vertical list of all stages with status markers:
##       ✓  completed (idx < current_idx)
##       ▶  current   (idx == current_idx)
##       ○  upcoming  (idx > current_idx)
##   - For the current stage: live progress bar + "current / threshold"
##     readout pulled from CurriculumManager.current_progress().  The
##     metric (e.g. best_cumulative_alive_ticks, max_distance_from_origin)
##     is rendered in human units (sim seconds or metres).
##   - Bottom row: [Prev] [Next] [Auto ☐] [Reset] buttons.  Hotkeys
##     [ / ] / Shift+] are still active.
##
## Stays hidden when no curriculum is loaded.  The panel resizes itself
## to fit the stage count so curricula of any length show cleanly.
##
## Methodology guard: interactive UI only — no per-tick advance logic
## lives here.  CurriculumManager owns the advance rule + manual/auto
## state; this panel just visualises.

const _FLASH_DURATION: float = 0.6
const _ROW_H: float          = 22.0
const _HEADER_H: float       = 32.0
const _PROGRESS_H: float     = 22.0
const _POLICY_ROW_H: float   = 16.0
const _BUTTON_H: float       = 32.0

# Known metric → human-readable formatter.  Add entries here when new
# metrics appear in advance_when blocks.
const _METRIC_HUMAN: Dictionary = {
    "best_cumulative_alive_ticks": {"unit": "time", "label": "stand"},
    "cumulative_alive_ticks":      {"unit": "time", "label": "stand"},
    "max_distance_from_origin":    {"unit": "m",    "label": "walk"},
    "chassis_y":                   {"unit": "m",    "label": "y"},
}

# Known reward-knob → suffix used when displaying the current stage's
# override values.  Unknown keys still show as `key = value`.
const _POLICY_UNITS: Dictionary = {
    "target_height":           "m",
    "height_penalty_grace":    "m",
    "walk_target_velocity":    "m/s",
    "peak_height":             "m",
    "band_width":              "m",
    "auto_reset_max_height":   "m",
    "auto_reset_tilt_threshold": "rad",
}

var _bg: PanelContainer
var _header: Label
var _stages_vb: VBoxContainer
var _progress_row: HBoxContainer
var _progress_bar: ProgressBar
var _progress_label: Label
var _policy_header: Label
var _policy_vb: VBoxContainer
var _policy_scroll: ScrollContainer    # wraps _policy_vb to cap height
var _policy_row_labels: Array = []
const _POLICY_MAX_VISIBLE_ROWS: int = 12   # row count beyond which scroll engages
var _prev_btn: Button
var _next_btn: Button
var _auto_btn: Button
var _reset_btn: Button
# 2026-06-09 — in-scene curriculum loader so the user can switch curricula
# without bouncing back to the launcher.  Joseph QoL ask.
var _load_dropdown: OptionButton
var _load_btn:      Button
var _auto_load_chk: CheckBox
# 2026-06-09 — minimise/expand toggle (▼/▲).  Collapsed state hides the
# main content VBox but keeps the header visible so the user can re-expand.
var _minimise_btn:  Button
var _content_vb:    VBoxContainer
var _is_minimised:  bool = false

var _last_idx: int = -1
var _last_n: int = 0
var _flash_t: float = 0.0
var _stage_row_labels: Array = []   # parallel to stages

func _ready() -> void:
    anchor_left   = 0.0
    anchor_top    = 0.0
    anchor_right  = 0.0
    anchor_bottom = 0.0
    offset_left   = 12.0
    offset_top    = 300.0
    offset_right  = 12.0 + 280.0    # 2026-06-09 — narrower (was 360); content still fits clearly
    offset_bottom = 540.0   # resized in _process based on stage count
    mouse_filter  = Control.MOUSE_FILTER_PASS

    _bg = PanelContainer.new()
    _bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
    var st := StyleBoxFlat.new()
    st.bg_color = Color(0.05, 0.05, 0.07, 0.82)
    st.border_color = Color(0.55, 0.55, 0.6, 0.85)
    st.set_border_width_all(1)
    st.set_corner_radius_all(4)
    st.content_margin_left = 10
    st.content_margin_right = 10
    st.content_margin_top = 8
    st.content_margin_bottom = 8
    _bg.add_theme_stylebox_override("panel", st)
    add_child(_bg)

    var vb := VBoxContainer.new()
    vb.add_theme_constant_override("separation", 4)
    _bg.add_child(vb)

    # Header row: title + minimise button (▼ when expanded, ▲ when collapsed).
    var header_row := HBoxContainer.new()
    header_row.add_theme_constant_override("separation", 4)
    vb.add_child(header_row)
    _header = Label.new()
    _header.text = "CURRICULUM"
    _header.add_theme_font_size_override("font_size", 14)
    _header.add_theme_color_override("font_color", Color(0.95, 0.95, 1.0, 1.0))
    _header.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    header_row.add_child(_header)
    _minimise_btn = Button.new()
    _minimise_btn.text = "▼"
    _minimise_btn.custom_minimum_size = Vector2(28, 0)
    _minimise_btn.pressed.connect(_on_minimise_pressed)
    header_row.add_child(_minimise_btn)

    # Content VBox below the header — everything else lives in here so the
    # minimise toggle can hide it as a unit.
    _content_vb = VBoxContainer.new()
    _content_vb.add_theme_constant_override("separation", 4)
    vb.add_child(_content_vb)

    # 2026-06-09 — in-scene curriculum loader.
    var load_row := HBoxContainer.new()
    load_row.add_theme_constant_override("separation", 4)
    _content_vb.add_child(load_row)
    _load_dropdown = OptionButton.new()
    _load_dropdown.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _populate_load_dropdown()
    load_row.add_child(_load_dropdown)
    _load_btn = Button.new()
    _load_btn.text = "Load"
    _load_btn.pressed.connect(_on_load_pressed)
    load_row.add_child(_load_btn)
    var auto_row := HBoxContainer.new()
    auto_row.add_theme_constant_override("separation", 4)
    _content_vb.add_child(auto_row)
    _auto_load_chk = CheckBox.new()
    _auto_load_chk.text = "auto-advance on load"
    _auto_load_chk.button_pressed = true
    auto_row.add_child(_auto_load_chk)

    var sep1 := HSeparator.new()
    _content_vb.add_child(sep1)

    _stages_vb = VBoxContainer.new()
    _stages_vb.add_theme_constant_override("separation", 2)
    _content_vb.add_child(_stages_vb)

    _progress_row = HBoxContainer.new()
    _progress_row.add_theme_constant_override("separation", 6)
    _content_vb.add_child(_progress_row)

    _progress_bar = ProgressBar.new()
    _progress_bar.min_value = 0.0
    _progress_bar.max_value = 1.0
    _progress_bar.value = 0.0
    _progress_bar.show_percentage = false
    _progress_bar.custom_minimum_size = Vector2(160, 16)
    _progress_bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _progress_row.add_child(_progress_bar)

    _progress_label = Label.new()
    _progress_label.add_theme_font_size_override("font_size", 11)
    _progress_label.add_theme_color_override("font_color", Color(0.85, 0.85, 0.9, 1.0))
    _progress_label.custom_minimum_size = Vector2(120, 0)
    _progress_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
    _progress_row.add_child(_progress_label)

    var sep2 := HSeparator.new()
    _content_vb.add_child(sep2)

    _policy_header = Label.new()
    _policy_header.text = "REWARD POLICY"
    _policy_header.add_theme_font_size_override("font_size", 12)
    _policy_header.add_theme_color_override("font_color", Color(0.8, 0.85, 1.0, 1.0))
    _content_vb.add_child(_policy_header)

    # 2026-06-09 — wrap policy rows in a ScrollContainer.  Height is
    # sized dynamically in _refresh_policy_rows to min(rows,
    # _POLICY_MAX_VISIBLE_ROWS) × _POLICY_ROW_H so short policies don't
    # waste vertical real estate and long ones cap at 12 rows + scroll.
    _policy_scroll = ScrollContainer.new()
    _policy_scroll.custom_minimum_size = Vector2(0, _POLICY_ROW_H * 4 + 4)
    _policy_scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _policy_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
    _content_vb.add_child(_policy_scroll)

    _policy_vb = VBoxContainer.new()
    _policy_vb.add_theme_constant_override("separation", 1)
    _policy_vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _policy_scroll.add_child(_policy_vb)

    var sep3 := HSeparator.new()
    _content_vb.add_child(sep3)

    var hb := HBoxContainer.new()
    hb.add_theme_constant_override("separation", 4)
    _content_vb.add_child(hb)

    _prev_btn = Button.new()
    _prev_btn.text = "[ Prev"
    _prev_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _prev_btn.pressed.connect(func() -> void: CurriculumManager.prev_stage())
    hb.add_child(_prev_btn)

    _next_btn = Button.new()
    _next_btn.text = "Next ]"
    _next_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _next_btn.pressed.connect(func() -> void: CurriculumManager.next_stage())
    hb.add_child(_next_btn)

    _auto_btn = Button.new()
    _auto_btn.toggle_mode = true
    _auto_btn.text = "Auto"
    _auto_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _auto_btn.toggled.connect(_on_auto_toggled)
    hb.add_child(_auto_btn)

    _reset_btn = Button.new()
    _reset_btn.text = "→0"
    _reset_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    _reset_btn.pressed.connect(func() -> void: CurriculumManager.goto_stage(0))
    hb.add_child(_reset_btn)

    CurriculumManager.connect("stage_changed", _on_stage_changed)
    CurriculumManager.connect("curriculum_loaded", _on_curriculum_loaded)
    # 2026-06-09 — panel starts visible (was: hidden when no curriculum) so
    # the in-scene loader is always reachable.  Joseph QoL ask.
    visible = true

func _rebuild_stage_rows() -> void:
    for c in _stages_vb.get_children():
        c.queue_free()
    _stage_row_labels.clear()
    for i in range(CurriculumManager.n_stages()):
        var row := Label.new()
        row.add_theme_font_size_override("font_size", 12)
        _stages_vb.add_child(row)
        _stage_row_labels.append(row)
    # Resize panel to fit stages + reward-policy block.  Policy lines
    # are sized in _refresh_policy_rows once overrides are known.
    var n: int = CurriculumManager.n_stages()
    # 2026-06-09 — height management lives in _refresh_policy_rows so the
    # offset_bottom reflects the actual (capped) policy row count rather
    # than the maximum.  Skip when minimised — toggle owns that state.

func _on_curriculum_loaded(_path: String, _n: int) -> void:
    _rebuild_stage_rows()

func _process(delta: float) -> void:
    # 2026-06-09 — panel stays visible always so the loader is reachable;
    # when no curriculum is loaded the stage/progress/policy rows just
    # show empty placeholders.
    if not CurriculumManager.has_curriculum():
        _header.text = "CURRICULUM (no curriculum loaded)"
        # Skip the stage/progress rendering when nothing to render.
        return
    var n: int = CurriculumManager.n_stages()
    if n != _last_n or _stage_row_labels.size() != n:
        _rebuild_stage_rows()
        _last_n = n
    var idx: int = CurriculumManager.current_idx
    _header.text = "CURRICULUM (%d/%d)" % [idx + 1, n]

    # Update each stage row.  Append a [M:SS] elapsed timer once a stage
    # has been visited (any time > 0); upcoming stages stay timer-less.
    for i in range(n):
        if i >= _stage_row_labels.size():
            break
        var row: Label = _stage_row_labels[i]
        var name_i: String = str(CurriculumManager.stages[i].get("name", "stage_%d" % i))
        var marker: String
        var col: Color
        if i < idx:
            marker = "✓"
            col = Color(0.45, 0.85, 0.45, 1.0)
        elif i == idx:
            marker = "▶"
            col = Color(1.0, 1.0, 0.7, 1.0)
        else:
            marker = "○"
            col = Color(0.55, 0.55, 0.6, 1.0)
        var elapsed_s: float = 0.0
        if i < CurriculumManager.time_in_stage_ticks.size():
            elapsed_s = CurriculumManager.time_in_stage_ticks[i] / 60.0
        var time_str: String = ""
        if elapsed_s > 0.0 or i == idx:
            time_str = "  [%s]" % _fmt_msec(elapsed_s)
        row.text = "  %s  %d. %s%s" % [marker, i + 1, name_i, time_str]
        row.add_theme_color_override("font_color", col)

    # Progress bar for the current stage.
    var prog: Dictionary = CurriculumManager.current_progress()
    if prog.is_empty():
        _progress_bar.value = 0.0
        _progress_label.text = "(no curriculum)"
    elif prog.get("terminal", false):
        _progress_bar.value = 1.0
        _progress_bar.modulate = Color(0.5, 0.85, 0.5, 1.0)
        _progress_label.text = "(terminal stage)"
    else:
        _progress_bar.value = float(prog.get("ratio", 0.0))
        _progress_bar.modulate = Color(1.0, 1.0, 1.0, 1.0)
        var metric_name: String = str(prog.get("metric", ""))
        var cur: float = float(prog.get("current", 0.0))
        var thr: float = float(prog.get("value", 0.0))
        var hu: Dictionary = _METRIC_HUMAN.get(metric_name, {"unit": "", "label": metric_name})
        var unit: String = str(hu.get("unit", ""))
        var cur_s: String
        var thr_s: String
        if unit == "time":
            # cur/thr are tick counts at 60 Hz — divide to get seconds.
            cur_s = _fmt_msec(cur / 60.0)
            thr_s = _fmt_msec(thr / 60.0)
        elif unit == "m":
            cur_s = "%.2fm" % cur
            thr_s = "%.2fm" % thr
        else:
            cur_s = "%.2f" % cur
            thr_s = "%.2f" % thr
        _progress_label.text = "%s %s/%s" % [str(hu.get("label", "")), cur_s, thr_s]

    _refresh_policy_rows(CurriculumManager.current_overrides())

    _prev_btn.disabled = (idx <= 0)
    _next_btn.disabled = (idx >= n - 1)
    if CurriculumManager.auto_advance != _auto_btn.button_pressed:
        _auto_btn.set_pressed_no_signal(CurriculumManager.auto_advance)
    _auto_btn.text = "Auto: %s" % ("ON" if CurriculumManager.auto_advance else "OFF")
    if _flash_t > 0.0:
        _flash_t = max(0.0, _flash_t - delta)
        if idx < _stage_row_labels.size():
            var r: Label = _stage_row_labels[idx]
            r.modulate = Color(1, 1, 0.6, 1).lerp(
                Color(1, 1, 1, 1), 1.0 - _flash_t / _FLASH_DURATION)

func _refresh_policy_rows(overrides: Dictionary) -> void:
    # Rebuild policy lines whenever the override count changes; otherwise
    # just refresh values in place to avoid creating new Labels every
    # frame.
    var keys: Array = overrides.keys()
    keys.sort()
    if _policy_row_labels.size() != keys.size():
        for c in _policy_vb.get_children():
            c.queue_free()
        _policy_row_labels.clear()
        for _i in keys:
            var lbl := Label.new()
            lbl.add_theme_font_size_override("font_size", 11)
            lbl.add_theme_color_override("font_color", Color(0.78, 0.82, 0.88, 1.0))
            _policy_vb.add_child(lbl)
            _policy_row_labels.append(lbl)
        # 2026-06-09 — size the scroll container to fit min(rows, MAX) so
        # short policies don't waste space and long ones cap at 12 rows.
        var visible_rows: int = min(keys.size(), _POLICY_MAX_VISIBLE_ROWS)
        if visible_rows < 1:
            visible_rows = 1
        if _policy_scroll != null:
            _policy_scroll.custom_minimum_size = Vector2(0, _POLICY_ROW_H * visible_rows + 4)
        # Re-sized outer panel to fit the policy block too.  Use the
        # scroll-capped row count (not the raw key count) so the panel
        # doesn't grow past the left-side budget.
        var n: int = CurriculumManager.n_stages()
        if not _is_minimised:
            offset_bottom = offset_top + _HEADER_H + 60.0 + (_ROW_H * n) \
                            + _PROGRESS_H + 22.0 + (_POLICY_ROW_H * visible_rows + 4) \
                            + 8.0 + _BUTTON_H + 36.0
    if keys.is_empty():
        _policy_header.text = "REWARD POLICY  (no overrides — body defaults)"
        return
    _policy_header.text = "REWARD POLICY"
    for i in range(keys.size()):
        var k: String = str(keys[i])
        var v: Variant = overrides[k]
        var unit: String = str(_POLICY_UNITS.get(k, ""))
        var v_str: String
        if v is bool:
            v_str = "on" if v else "off"
        elif v is float or v is int:
            v_str = _fmt_num(float(v)) + unit
            # Annotate "off" semantics for rates that are sometimes 0.
            if (k == "walk_hit_rate" or k == "height_penalty_scale") \
                    and float(v) <= 0.0:
                v_str += "   (disabled)"
        else:
            v_str = str(v)
        _policy_row_labels[i].text = "  %s = %s" % [k, v_str]

func _fmt_num(v: float) -> String:
    # General-purpose float→string for policy values.  GDScript's
    # `%`-operator does not support %g (only d/o/x/X/e/E/f/v/c/s), so
    # we roll our own: integers render as "0", "1", "60"; floats render
    # at up to 4 decimals with trailing zeros stripped.
    if abs(v - round(v)) < 1e-6 and abs(v) < 1e9:
        return "%d" % int(round(v))
    var s: String = "%.4f" % v
    while s.ends_with("0"):
        s = s.substr(0, s.length() - 1)
    if s.ends_with("."):
        s = s.substr(0, s.length() - 1)
    return s

func _fmt_msec(seconds: float) -> String:
    # Format as H:MM:SS (≥1 h), M:SS (≥1 min), else "Ns".
    var total_s: int = int(round(seconds))
    var h: int = total_s / 3600
    var m: int = (total_s % 3600) / 60
    var s: int = total_s % 60
    if h > 0:
        return "%d:%02d:%02d" % [h, m, s]
    if m > 0:
        return "%d:%02d" % [m, s]
    return "%ds" % s

func _unhandled_input(event: InputEvent) -> void:
    if not (event is InputEventKey):
        return
    var ke: InputEventKey = event
    if not ke.pressed or ke.echo:
        return
    if not CurriculumManager.has_curriculum():
        return
    if ke.keycode == KEY_BRACKETLEFT:
        CurriculumManager.prev_stage()
        get_viewport().set_input_as_handled()
    elif ke.keycode == KEY_BRACKETRIGHT:
        if ke.shift_pressed:
            CurriculumManager.set_auto_advance(not CurriculumManager.auto_advance)
        else:
            CurriculumManager.next_stage()
        get_viewport().set_input_as_handled()

func _on_stage_changed(_idx: int, _name: String, _overrides: Dictionary) -> void:
    _flash_t = _FLASH_DURATION

func _on_auto_toggled(pressed: bool) -> void:
    CurriculumManager.set_auto_advance(pressed)

# 2026-06-09 — populate the in-scene loader dropdown from res://curricula/.
# Same convention as the launcher's curriculum dropdown.
func _populate_load_dropdown() -> void:
    _load_dropdown.clear()
    _load_dropdown.add_item("(pick a curriculum)")
    _load_dropdown.set_item_metadata(0, "")
    var dir := DirAccess.open("res://curricula/")
    if dir == null:
        push_warning("CurriculumPanel: cannot open res://curricula/")
        return
    var entries: Array = []
    dir.list_dir_begin()
    while true:
        var f: String = dir.get_next()
        if f == "":
            break
        if f.ends_with(".json"):
            entries.append(f)
    dir.list_dir_end()
    entries.sort()
    for f in entries:
        _load_dropdown.add_item(f)
        _load_dropdown.set_item_metadata(
            _load_dropdown.item_count - 1, "res://curricula/" + f)

# 2026-06-09 — load handler.  Reads selected path + auto-advance checkbox,
# calls CurriculumManager.load_curriculum_file → emits curriculum_loaded
# and stage_changed(0, ...) → body's _on_curriculum_stage_changed applies
# stage-0 overrides → reward_panel's stage_changed listener refreshes sliders.
func _on_load_pressed() -> void:
    var i: int = _load_dropdown.selected
    if i <= 0:
        return
    var path: String = str(_load_dropdown.get_item_metadata(i))
    if path == "":
        return
    var ok: bool = CurriculumManager.load_curriculum_file(path)
    if not ok:
        push_warning("CurriculumPanel: failed to load %s" % path)
        return
    CurriculumManager.set_auto_advance(_auto_load_chk.button_pressed)
    # Keep the Auto button in the bottom row synced with the new state.
    if _auto_btn != null:
        _auto_btn.set_block_signals(true)
        _auto_btn.button_pressed = _auto_load_chk.button_pressed
        _auto_btn.set_block_signals(false)
    print("CurriculumPanel: loaded %s  auto_advance=%s" %
          [path, str(_auto_load_chk.button_pressed)])

# 2026-06-09 — minimise/expand toggle.  Hides _content_vb but keeps the
# header + button visible.  Toggles glyph between ▼ (expanded) and ▲
# (collapsed).
func _on_minimise_pressed() -> void:
    _is_minimised = not _is_minimised
    _content_vb.visible = not _is_minimised
    _minimise_btn.text = "▲" if _is_minimised else "▼"
    # Shrink the panel height when collapsed so it doesn't eat dead space.
    if _is_minimised:
        offset_bottom = offset_top + _HEADER_H + 8
