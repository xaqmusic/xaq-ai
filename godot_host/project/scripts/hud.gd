extends Control
## Top-left HUD overlay: elapsed time, eats, and the active world config.
## Polls TheCell + Body each frame; no signal wiring required.

const TICKS_PER_SEC: float = 60.0   # matches body_controller's _physics_process rate

var _label: Label

# Phase 6.6.F — realtime MotorFader α meter (top-right).  Visible only
# when a MotorFader is actually publishing on motor.fader.alpha.
const _MotorFaderMeterScene: GDScript = preload("res://scripts/motor_fader_meter.gd")
var _fader_meter: Control = null

# 2026-06-19 — multichannel MotorBus fader panel (top-right, below the meter).
# Per-influencer faders → bus-compressor.  Visible only when a MotorBus is
# in the graph (auto-hides otherwise, like the fader meter).
const _MotorBusPanelScene: GDScript = preload("res://scripts/motor_bus_panel.gd")
var _bus_panel: Control = null

# 2026-06-21 — real-time EXPLORING ↔ HOMING gauge from the coxswain's plan
# entropy.  Visible only when an ActionDecoder reports plan_entropy
# (plan_temperature>0).  Top-left under the main HUD label.
const _ExploreMeterScene: GDScript = preload("res://scripts/explore_meter.gd")
var _explore_meter: Control = null

# v5.4 — brain state indicator (top-left, below the main label).  Shows what
# is driving the motor command this tick: CHUNK vs PREMOTOR vs REFLEX, plus
# the clash metric (intent erased when brain+reflex disagree in sign).
const _BrainStateIndicatorScene: GDScript = preload("res://scripts/brain_state_indicator.gd")
var _brain_state_indicator: Control = null

# Phase 6.5.23 — brain-view panel (raycast frame).  Lives at the
# top-right of the HUD, below the motor fader meter.  Only visible while
# the top-down camera is active so the FPV view stays unobstructed
# during normal play.  Anchoring top-right keeps the panel out of the
# bottom-half top-down map area when the window is sized large.  We
# rebuild the ImageTexture each frame from body._last_vis_pixels.
const _BRAIN_VIEW_PX: int = 192     # 8× scale of the 24×24 raycast grid
const _BRAIN_VIEW_MARGIN: int = 12
# Vertical offset to clear the MOTOR BUS PANEL, which is the top-right occupant
# (motor_bus_panel.gd: anchored top-right at y=96, ~110px tall for the cell's
# cog/whisker/stuck channels + title + out-label).  The motor_fader_meter moved to
# top-CENTRE, so the bus faders are what overlapped the raycast view.  Start the
# brain view at y=230 → clearly below the bus faders with a gap.
const _BRAIN_VIEW_TOP_OFFSET: int = 230
var _brain_view_panel: PanelContainer = null
var _brain_view_rect:  TextureRect    = null
var _brain_view_image: Image          = null
var _brain_view_texture: ImageTexture = null

# Phase 6.5.14c — end-run summary modal for Cell.  Cell is continuous
# (no episodes) so the modal is a snapshot of current state with a
# Copy-log button + Return-to-launcher / Quit.  Triggered by the pause
# overlay's "End run & show summary" button via request_end_run().
var _summary_root: Control = null

# Live scent-field-distance slider (the D-value rescue dial): full → zero mid-run.
var _scent_slider: HSlider = null
var _scent_reach_label: Label = null

# --- live-state meter panel (top-left): MOTOR use, ENERGY, then the L2 ARBITER race ---
# Three stacked bars in a shared visual language so movement, its metabolic cost, and the
# policy that drives it read together. motor & energy are single-fill bars; the arbiter is a
# tug-of-war (klino | planner | play share) with the DRIVING loop bright and the muted ones
# dimmed — mirrors the xaq_inspector EFEArbiter widget. The arbiter row shows only when the
# config has an EFEArbiter.
const _KLINO_COL := Color(0.31, 0.88, 0.44)    # green  — the near-food CLOSER
const _PLANNER_COL := Color(0.47, 0.63, 1.0)   # blue   — the far-field SEARCHER
const _PLAY_COL := Color(0.82, 0.5, 1.0)       # violet — the epistemic GROWER (task #33)
const _MOTOR_COL := Color(1.0, 0.72, 0.2)      # amber — motor work (drives energy drain)
const _ENERGY_COL := Color(0.4, 0.85, 0.5)     # green — survival fuel
const _METER_EMPTY := Color(0.12, 0.12, 0.14, 1.0)
const _MOTOR_EFFORT_MAX := 2.0                 # |al|+|ar| (or spike count) ∈ [0, 2]
const _HUD_FONT := 18                          # one font size for the whole HUD (matches the top status label)
var _meters_box: VBoxContainer = null
var _motor_meter: Dictionary = {}
var _energy_meter: Dictionary = {}
var _arbiter_meter: Dictionary = {}
var _arbiter_row: HBoxContainer = null

# 2026-07-09 — HUD hide toggle (TAB): clears the top-down view. When hidden, only a one-line
# hotkey hint shows at the top (same intent as the picrawler scene's H/T hide behaviour).
var _hud_hidden: bool = false
var _hint_label: Label = null
var _arbiter_play: ColorRect = null            # task #33 — play's share segment in the 3-way arbiter meter
var _summary_shown: bool   = false

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE   # don't steal mouse from the graph panel

	# A simple monospaced label in the top-left.  No background panel — keeps
	# the FPV camera unobstructed.
	_label = Label.new()
	_label.add_theme_font_size_override("font_size", 18)
	_label.add_theme_color_override("font_color", Color(1, 1, 1, 1))
	_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	_label.add_theme_constant_override("outline_size", 4)
	_label.position = Vector2(12, 8)
	_label.size     = Vector2(420, 160)
	add_child(_label)
	_build_brain_view_panel()
	_build_fader_meter()
	_build_bus_panel()
	_build_explore_meter()
	_build_brain_state_indicator()
	_build_scent_slider()
	_build_meters_panel()
	_build_hint_label()

func _build_hint_label() -> void:
	# The lone one-line hotkey hint, top-left. Hidden while the full HUD is shown (the meters
	# panel carries the keys line then); shown alone when the HUD is hidden (TAB).
	_hint_label = Label.new()
	_hint_label.add_theme_font_size_override("font_size", 15)
	_hint_label.add_theme_color_override("font_color", Color(1, 1, 1, 1))
	_hint_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	_hint_label.add_theme_constant_override("outline_size", 4)
	_hint_label.position = Vector2(12, 8)
	_hint_label.text = "TAB show HUD   ·   V view · 1 easy · 2 medium · 3 hard · D drop-connector · H scent-heatmap · J place-map"
	_hint_label.visible = false
	add_child(_hint_label)

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_TAB:
		_set_hud_hidden(not _hud_hidden)

func _set_hud_hidden(h: bool) -> void:
	_hud_hidden = h
	for c in get_children():
		if c == _hint_label:
			c.visible = h                # the lone hint line — only while hidden
		elif c == _brain_view_panel:
			pass                         # driven by _refresh_brain_view (guarded on _hud_hidden)
		else:
			c.visible = not h            # everything else hides

func _build_brain_view_panel() -> void:
	# Anchor to bottom-right of the HUD parent.  Width/height = brain-view
	# px + room for the title label.  Hidden by default; shown only when
	# the top-down camera is current (handled in _process).
	var title := Label.new()
	title.text = "brain view (raycast)"
	title.add_theme_font_size_override("font_size", 11)
	title.add_theme_color_override("font_color", Color(0.85, 0.92, 1.0, 1))
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER

	_brain_view_rect = TextureRect.new()
	_brain_view_rect.custom_minimum_size = Vector2(_BRAIN_VIEW_PX, _BRAIN_VIEW_PX)
	_brain_view_rect.expand_mode    = TextureRect.EXPAND_IGNORE_SIZE
	_brain_view_rect.stretch_mode   = TextureRect.STRETCH_SCALE
	_brain_view_rect.texture_filter = CanvasItem.TEXTURE_FILTER_NEAREST   # crisp pixels

	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 4)
	v.add_child(title)
	v.add_child(_brain_view_rect)

	_brain_view_panel = PanelContainer.new()
	_brain_view_panel.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	_brain_view_panel.position = Vector2(
		-(_BRAIN_VIEW_PX + 2 * _BRAIN_VIEW_MARGIN),
		_BRAIN_VIEW_TOP_OFFSET)
	_brain_view_panel.add_child(v)
	_brain_view_panel.visible = false   # surfaced only in top-down view
	add_child(_brain_view_panel)

	# Pre-allocate the Image / ImageTexture so per-frame updates are just
	# a set_data + update — no allocations in the hot path.
	_brain_view_image = Image.create_empty(24, 24, false, Image.FORMAT_RGB8)
	_brain_view_texture = ImageTexture.create_from_image(_brain_view_image)
	_brain_view_rect.texture = _brain_view_texture

func _build_scent_slider() -> void:
	# Live "scent reach" dial (full → zero) → world.set_scent_reach(). Lets the
	# operator shrink the scent field mid-run and watch the planner take over
	# (the D-value rescue). Top-left, just below the status label.
	_scent_reach_label = Label.new()
	_scent_reach_label.text = "scent reach: 100%"
	_scent_reach_label.add_theme_font_size_override("font_size", 14)
	_scent_reach_label.add_theme_color_override("font_color", Color(0.7, 1.0, 0.7, 1))
	_scent_reach_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	_scent_reach_label.add_theme_constant_override("outline_size", 3)

	_scent_slider = HSlider.new()
	_scent_slider.min_value = 0.0
	_scent_slider.max_value = 1.0
	_scent_slider.step = 0.01
	_scent_slider.value = 1.0
	_scent_slider.custom_minimum_size = Vector2(220, 20)
	_scent_slider.mouse_filter = Control.MOUSE_FILTER_STOP   # capture the drag (HUD root IGNORES)
	_scent_slider.value_changed.connect(_on_scent_reach_changed)

	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 2)
	box.add_child(_scent_reach_label)
	box.add_child(_scent_slider)

	var panel := PanelContainer.new()
	# Anchor bottom-left, grow upward — clear of the top-left status label regardless
	# of how many lines it has.
	panel.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
	panel.grow_vertical = Control.GROW_DIRECTION_BEGIN
	panel.offset_left = 12.0
	panel.offset_bottom = -12.0
	panel.mouse_filter = Control.MOUSE_FILTER_PASS
	panel.add_child(box)
	add_child(panel)

func _on_scent_reach_changed(v: float) -> void:
	if _scent_reach_label != null:
		_scent_reach_label.text = "scent reach: %d%%" % int(round(v * 100.0))
	var world: Node = get_tree().get_root().find_child("TheCell", true, false)
	if world != null and world.has_method("set_scent_reach"):
		world.call("set_scent_reach", v)

# One HUD label style everywhere: the shared font size + outline, colour per call.
func _style_hud_label(lbl: Label, color: Color) -> void:
	lbl.add_theme_font_size_override("font_size", _HUD_FONT)
	lbl.add_theme_color_override("font_color", color)
	lbl.add_theme_color_override("font_outline_color", Color(0, 0, 0, 1))
	lbl.add_theme_constant_override("outline_size", 4)

# A labeled fixed-width bar row: [ name | fill══empty | value ], laid out on an even grid
# row of height `row_h` (matches the top label's line height). fill+empty are ColorRects in a
# fixed-width HBox whose stretch ratios encode the 0..1 level; the bar is vertically centred so
# the text (name/value) reads on the same baseline as every other HUD line.
func _make_meter_row(name: String, row_h: float, fill_col: Color) -> Dictionary:
	var row := HBoxContainer.new()
	row.custom_minimum_size = Vector2(0, row_h)
	row.add_theme_constant_override("separation", 6)
	row.mouse_filter = Control.MOUSE_FILTER_IGNORE

	var lbl := Label.new()
	lbl.text = name
	lbl.custom_minimum_size = Vector2(76, row_h)
	lbl.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_style_hud_label(lbl, Color(1, 1, 1, 1))   # white — same as the top status label
	row.add_child(lbl)

	var bar := HBoxContainer.new()
	bar.custom_minimum_size = Vector2(230, 16)
	bar.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	bar.add_theme_constant_override("separation", 1)
	bar.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var fill := ColorRect.new()
	fill.color = fill_col
	fill.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	fill.custom_minimum_size = Vector2(0, 16)
	var empty := ColorRect.new()
	empty.color = _METER_EMPTY
	empty.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	empty.custom_minimum_size = Vector2(0, 16)
	bar.add_child(fill)
	bar.add_child(empty)
	row.add_child(bar)

	var val := Label.new()
	val.custom_minimum_size = Vector2(100, row_h)
	val.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_style_hud_label(val, Color(1, 1, 1, 1))
	row.add_child(val)
	return {"row": row, "fill": fill, "empty": empty, "val": val}

# Set a single-fill meter's level ∈[0,1] and its numeric readout text.
func _set_meter(meter: Dictionary, level: float, text: String) -> void:
	var v: float = clampf(level, 0.0, 1.0)
	meter["fill"].size_flags_stretch_ratio = maxf(v, 0.001)
	meter["empty"].size_flags_stretch_ratio = maxf(1.0 - v, 0.001)
	meter["val"].text = text

func _build_meters_panel() -> void:
	# One even grid, same font as the top status label.  The label carries 4 rows
	# (config, timer, eats, world); this panel continues the grid directly below with
	# MOTOR → ENERGY → ARBITER (operator order) and the keybind hint last.
	var lh := 24.0
	var f: Font = _label.get_theme_font("font")
	if f != null:
		lh = f.get_height(_HUD_FONT)
	var line_spacing: int = _label.get_theme_constant("line_spacing")   # label's per-line gap
	var pitch := lh + float(line_spacing)                               # one grid row = font height + gap
	_meters_box = VBoxContainer.new()
	# start directly after the 4 status lines (config, timer, eats, world) and keep the SAME row
	# pitch (row height lh + separation = the label's line_spacing) → one continuous even grid.
	_meters_box.position = Vector2(12, 8.0 + 4.0 * pitch)
	_meters_box.add_theme_constant_override("separation", line_spacing)
	_meters_box.mouse_filter = Control.MOUSE_FILTER_IGNORE

	_motor_meter = _make_meter_row("motor", lh, _MOTOR_COL)
	_energy_meter = _make_meter_row("energy", lh, _ENERGY_COL)
	_meters_box.add_child(_motor_meter["row"])
	_meters_box.add_child(_energy_meter["row"])

	# arbiter row: same [ name | bar | value ] shape, but the bar is a TUG-OF-WAR
	# (klino green share vs planner blue share) rather than a single fill; reuse the
	# fill/empty rects as the two sides, and the value label as the winner readout.
	_arbiter_meter = _make_meter_row("arbiter", lh, _KLINO_COL)
	_arbiter_meter["empty"].color = _PLANNER_COL   # the "empty" side is the planner's share
	# task #33 — a THIRD segment (play/GROW) so the meter reads klino | planner | play, not
	# just klino|planner. v_play is 0 while play is inert/absent, so the bar collapses to the
	# original 2-way split until the play policy is wired AND weighted.
	var play_seg := ColorRect.new()
	play_seg.color = _PLAY_COL
	play_seg.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	play_seg.custom_minimum_size = Vector2(0, 16)
	(_arbiter_meter["fill"].get_parent() as HBoxContainer).add_child(play_seg)   # bar = [klino | planner | play]
	_arbiter_play = play_seg
	_arbiter_row = _arbiter_meter["row"]
	_meters_box.add_child(_arbiter_row)

	var keys := Label.new()
	keys.text = "keys:  TAB hide-HUD · V view · 1 easy · 2 medium · 3 hard · D drop-connector · P pillars · T terrain · H scent-heatmap · J place-map · C chunk-probe · ` / F1 graph"
	keys.custom_minimum_size = Vector2(0, lh)
	keys.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_style_hud_label(keys, Color(1, 1, 1, 1))
	_meters_box.add_child(keys)

	add_child(_meters_box)

# Find the EFEArbiter's live diag in the brain metrics (keyed by module id → scan by type).
func _find_arbiter(metrics: Dictionary) -> Dictionary:
	for k in metrics:
		var mm = metrics[k]
		if mm is Dictionary and String(mm.get("type", "")) == "EFEArbiter":
			return mm
	return {}

# Refresh all three meters. motor_effort ∈[0,2] drives energy drain; energy ∈[0,1]; arbiter
# is the value-share tug-of-war with the DRIVING loop bright, the muted one dimmed.
func _update_meters(body: Node, metrics: Dictionary) -> void:
	if _meters_box == null:
		return
	var effort: float = float(body.get("_motor_effort_last"))
	_set_meter(_motor_meter, effort / _MOTOR_EFFORT_MAX, "%.2f" % effort)
	var energy: float = float(body.get("energy"))
	_set_meter(_energy_meter, energy, "%.0f%%" % (energy * 100.0))

	var arb := _find_arbiter(metrics)
	if arb.is_empty():
		_arbiter_row.visible = false
		return
	_arbiter_row.visible = true
	# 3-way value-share tug-of-war: klino (green) | planner (blue) | play (violet).
	var vk: float = maxf(0.0, float(arb.get("v_klino", 0.0)))
	var vp: float = maxf(0.0, float(arb.get("v_planner", 0.0)))
	var vpl: float = maxf(0.0, float(arb.get("v_play", 0.0)))
	var total: float = vk + vp + vpl
	var kshare: float  = (vk / total) if total > 1e-6 else (1.0 / 3.0)
	var pshare: float  = (vp / total) if total > 1e-6 else (1.0 / 3.0)
	var plshare: float = (vpl / total) if total > 1e-6 else (1.0 / 3.0)
	_arbiter_meter["fill"].size_flags_stretch_ratio  = maxf(kshare, 0.001)
	_arbiter_meter["empty"].size_flags_stretch_ratio = maxf(pshare, 0.001)
	if _arbiter_play != null:
		_arbiter_play.size_flags_stretch_ratio = maxf(plshare, 0.001)
	# the winner-take-all loop is bright; the muted ones dim (matches the mute → authority 0)
	var win: int = int(arb.get("winner", 0))
	_arbiter_meter["fill"].color  = _KLINO_COL   if win == 0 else _KLINO_COL.darkened(0.6)
	_arbiter_meter["empty"].color = _PLANNER_COL if win == 1 else _PLANNER_COL.darkened(0.6)
	if _arbiter_play != null:
		_arbiter_play.color = _PLAY_COL if win == 2 else _PLAY_COL.darkened(0.6)
	match win:
		0:
			_arbiter_meter["val"].text = "▶ KLINO"
			_arbiter_meter["val"].add_theme_color_override("font_color", _KLINO_COL)
		2:
			_arbiter_meter["val"].text = "▶ PLAY"
			_arbiter_meter["val"].add_theme_color_override("font_color", _PLAY_COL)
		_:
			_arbiter_meter["val"].text = "▶ PLANNER"
			_arbiter_meter["val"].add_theme_color_override("font_color", _PLANNER_COL)

func _build_fader_meter() -> void:
	_fader_meter = _MotorFaderMeterScene.new()
	# add_child first: motor_fader_meter._ready populates custom_minimum_size
	# (which we read just below).  Order matters.
	add_child(_fader_meter)
	# Float top-center.  Set anchors + offsets directly — Control.position
	# is parent-coord, NOT anchor-relative, so position=(-W/2, 12) puts
	# half the widget off-screen-left when the anchor is at 0.5.  Setting
	# offset_left/right symmetrically around the 0.5/0.5 anchor centres
	# the widget on the parent's mid-line; window resize keeps it centred.
	var w: float = _fader_meter.custom_minimum_size.x
	var h: float = _fader_meter.custom_minimum_size.y
	_fader_meter.anchor_left   = 0.5
	_fader_meter.anchor_right  = 0.5
	_fader_meter.anchor_top    = 0.0
	_fader_meter.anchor_bottom = 0.0
	_fader_meter.offset_left   = -w / 2.0
	_fader_meter.offset_right  =  w / 2.0
	_fader_meter.offset_top    = 12.0
	_fader_meter.offset_bottom = 12.0 + h

func _build_explore_meter() -> void:
	_explore_meter = _ExploreMeterScene.new()
	add_child(_explore_meter)
	# Top-left, below the main HUD label (y≈8..168) — out of the FPV centre.
	_explore_meter.anchor_left  = 0.0
	_explore_meter.anchor_right = 0.0
	_explore_meter.anchor_top   = 0.0
	_explore_meter.anchor_bottom = 0.0
	_explore_meter.offset_left  = 12.0
	_explore_meter.offset_top   = 200.0

func _build_bus_panel() -> void:
	# Top-right (the panel anchors itself in its _ready).  Full-size to the
	# parent so its internal PanelContainer anchor resolves correctly.
	_bus_panel = _MotorBusPanelScene.new()
	_bus_panel.set_anchors_preset(Control.PRESET_FULL_RECT)
	_bus_panel.mouse_filter = Control.MOUSE_FILTER_IGNORE   # children still receive
	add_child(_bus_panel)

func _build_brain_state_indicator() -> void:
	# Top-left, below the main HUD label (which uses y=8..168).  Anchor
	# top-left explicitly so the widget moves with viewport resize.
	_brain_state_indicator = _BrainStateIndicatorScene.new()
	add_child(_brain_state_indicator)
	_brain_state_indicator.anchor_left   = 0.0
	_brain_state_indicator.anchor_right  = 0.0
	_brain_state_indicator.anchor_top    = 0.0
	_brain_state_indicator.anchor_bottom = 0.0
	# Place under the main label (y≈170).  Width matches the indicator's
	# own custom_minimum_size (~280).
	_brain_state_indicator.offset_left   = 12
	_brain_state_indicator.offset_top    = 176
	_brain_state_indicator.offset_right  = 12 + _brain_state_indicator.custom_minimum_size.x
	_brain_state_indicator.offset_bottom = 176 + _brain_state_indicator.custom_minimum_size.y

func _topdown_active() -> bool:
	# The Cell scene mirrors v3's "press V to swap": the FPV Camera3D is on
	# the body and a TopDownCam lives at scene root.  Whichever has
	# Camera3D.current=true is the active view.  Treat unknown state as
	# FPV-active so the brain view defaults to hidden.
	var topdown := get_tree().get_root().find_child("TopDownCam", true, false) as Camera3D
	return topdown != null and topdown.current

func _refresh_brain_view(body: Node) -> void:
	if _brain_view_rect == null or _brain_view_image == null:
		return
	var visible_now := _topdown_active() and not _hud_hidden
	_brain_view_panel.visible = visible_now
	if not visible_now:
		return
	var pixels: PackedByteArray = body.get("_last_vis_pixels")
	if pixels == null or pixels.size() != 24 * 24 * 3:
		return
	_brain_view_image.set_data(24, 24, false, Image.FORMAT_RGB8, pixels)
	_brain_view_texture.update(_brain_view_image)

func _process(_delta: float) -> void:
	var body: Node = get_tree().get_root().find_child("Body", true, false)
	var world: Node = get_tree().get_root().find_child("TheCell", true, false)
	if body == null or world == null:
		return

	var ticks: int = int(body.get("tick_counter"))
	var hits:  int = int(body.get("hits_total"))
	var secs:  float = float(ticks) / TICKS_PER_SEC
	var mm: int = int(secs) / 60
	var ss: int = int(secs) % 60

	# Identify the world config (label-only — exact mode classification).
	var room_size_v: Vector3 = world.get("room_size")
	var room_w: float = room_size_v.x
	var nuts:  int    = int(world.get("nutrient_count"))
	var sigma: float  = float(world.get("scent_sigma"))
	# Prefer the launched config's NAME (the operator picks configs by name from the launcher);
	# fall back to the room/nutrient/σ mode classification when no config is set (headless / unknown).
	var cfg_tag: String = _config_label()
	var mode_tag: String = cfg_tag if cfg_tag != "" else _classify_mode(room_w, nuts, sigma)

	# Feed the side panels + collect brain metrics for the live meters below.
	var brain = body.get("brain")
	if _fader_meter != null and brain != null:
		_fader_meter.set_brain(brain)
	if _bus_panel != null and brain != null:
		_bus_panel.set_brain(brain)
	if _explore_meter != null and brain != null:
		_explore_meter.set_brain(brain)
	if _brain_state_indicator != null and brain != null:
		_brain_state_indicator.set_brain(brain)
	var metrics: Dictionary = {}
	if brain != null and brain.has_method("is_brain_ready") and brain.is_brain_ready():
		metrics = brain.get_module_metrics()

	_refresh_brain_view(body)
	# Live meters (top-left): motor use → energy → arbiter race.  motor+energy come from the
	# body every frame; the arbiter row appears only when the config has an EFEArbiter.
	_update_meters(body, metrics)

	_label.text = "%s\nelapsed:  %02d:%02d  (%ds · tick %d)\neats:     %d\nworld:    %.0fm × %.0fm  ·  %d nutrient%s  ·  σ=%.1f" % [
		mode_tag, mm, ss, int(secs), ticks, hits,
		room_w, room_w, nuts, ("s" if nuts != 1 else ""), sigma,
	]

# Match the smoke harness's mode definitions.
func _classify_mode(room_w: float, nuts: int, sigma: float) -> String:
	if abs(room_w - 16.0) < 0.5 and nuts == 3 and abs(sigma - 4.0) < 0.5:
		return "[ EASY ]"
	if abs(room_w - 24.0) < 0.5 and nuts == 1 and abs(sigma - 4.0) < 0.5:
		return "[ HARD ]"
	if abs(room_w - 24.0) < 0.5 and nuts == 1 and abs(sigma - 8.0) < 0.5:
		return "[ σ=8 SANITY ]"
	return "[ CUSTOM ]"

# The active config's identity for the top HUD line.  Prefer the launched config's file
# name (e.g. "arbiter_quad_play") over the room/nutrient/σ mode classification — the operator
# selects configs by name from the launcher, so the name is what they want to see.  Returns
# "" when no config is set (headless / unknown) so the caller falls back to _classify_mode.
func _config_label() -> String:
	var p := String(ExperimentConfig.config_path)
	if p == "":
		return ""
	var base := p.get_file().get_basename()   # ".../the_cell_arbiter_quad_play.json" → "the_cell_arbiter_quad_play"
	if base.begins_with("the_cell_"):
		base = base.substr(9)                 # strip the shared prefix → "arbiter_quad_play"
	return "[ %s ]" % base

# Phase 6.5.14c — request_end_run.  Called by pause-overlay when the
# user picks "End run & show summary".  Cell has no episode boundary
# so we just trigger the summary modal which shows a snapshot of
# current run state + Copy-log button.  The body keeps physics-ticking
# in the background; the modal blocks UI until the user picks a path.
func request_end_run() -> void:
	if _summary_shown:
		return
	_summary_shown = true
	var body: Node = get_tree().get_root().find_child("Body", true, false)
	if body:
		_show_summary(body)

func _show_summary(body: Node) -> void:
	if _summary_root != null:
		return
	_summary_root = Control.new()
	_summary_root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_summary_root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	# Mount on the same canvas layer as this HUD so it's visible above
	# the 3D scene but at HUD-Z, not on the pause-overlay autoload layer.
	add_child(_summary_root)

	var bg := ColorRect.new()
	bg.color = Color(0, 0, 0, 0.7)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_STOP
	_summary_root.add_child(bg)

	var panel := PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_CENTER)
	panel.custom_minimum_size = Vector2(640, 460)
	_summary_root.add_child(panel)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left",   28)
	margin.add_theme_constant_override("margin_right",  28)
	margin.add_theme_constant_override("margin_top",    24)
	margin.add_theme_constant_override("margin_bottom", 24)
	panel.add_child(margin)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 12)
	margin.add_child(v)

	var title := Label.new()
	title.text = "Run snapshot — Cell"
	title.add_theme_font_size_override("font_size", 22)
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	v.add_child(title)

	var report := RichTextLabel.new()
	report.bbcode_enabled = false
	report.selection_enabled = true
	report.focus_mode = Control.FOCUS_ALL
	report.size_flags_vertical = Control.SIZE_EXPAND_FILL
	report.add_theme_font_size_override("normal_font_size", 13)
	report.text = build_clipboard_text(body)
	v.add_child(report)

	var hb := HBoxContainer.new()
	hb.alignment = BoxContainer.ALIGNMENT_END
	hb.add_theme_constant_override("separation", 12)
	v.add_child(hb)

	var copy_btn := Button.new()
	copy_btn.text = "📋 Copy log"
	copy_btn.custom_minimum_size = Vector2(140, 36)
	copy_btn.pressed.connect(func():
		DisplayServer.clipboard_set(build_clipboard_text(body))
		copy_btn.text = "✓ Copied"
		var t := Timer.new()
		t.wait_time = 1.5
		t.one_shot = true
		t.timeout.connect(func():
			copy_btn.text = "📋 Copy log"
			t.queue_free()
		)
		copy_btn.add_child(t)
		t.start()
	)
	hb.add_child(copy_btn)

	var quit_btn := Button.new()
	quit_btn.text = "Quit"
	quit_btn.custom_minimum_size = Vector2(120, 36)
	quit_btn.pressed.connect(func(): get_tree().quit())
	hb.add_child(quit_btn)

	var return_btn := Button.new()
	return_btn.text = "Return to launcher"
	return_btn.custom_minimum_size = Vector2(180, 36)
	return_btn.pressed.connect(func():
		ExperimentConfig.launched = false
		get_tree().change_scene_to_file("res://scenes/launcher.tscn")
	)
	hb.add_child(return_btn)

# Phase 6.5.14b — clipboard export for Cell.  Cell is continuous (no
# episodes), so the log is a snapshot of the current run state rather
# than an end-of-run summary.  Format mirrors what cell_smoke.py would
# emit: world params + body progress (eats, time, stuck) + final brain
# metrics + chunk library detail.  Called by pause_overlay's Copy-log
# button.
func build_clipboard_text(body: Node) -> String:
	var lines: Array[String] = []
	var world: Node = get_tree().get_root().find_child("TheCell", true, false)

	# --- Header ---
	lines.append("=== xaq Cell run snapshot ===")
	lines.append("Config:      %s" % str(ExperimentConfig.config_path))
	lines.append("Seed:        %d" % int(ExperimentConfig.seed_value))
	if body and body.has_method("get") and body.get("tick_counter") != null:
		var ticks: int = int(body.get("tick_counter"))
		var secs: float = float(ticks) / TICKS_PER_SEC
		lines.append("Total ticks: %d  (%.1f s wall)" % [ticks, secs])

	# --- World mode ---
	if world:
		var room_size_v: Vector3 = world.get("room_size")
		var room_w: float = room_size_v.x
		var nuts:  int    = int(world.get("nutrient_count"))
		var sigma: float  = float(world.get("scent_sigma"))
		lines.append("World:       %s  (%.0fm × %.0fm, %d nutrient%s, σ=%.1f)" % [
			_classify_mode(room_w, nuts, sigma),
			room_w, room_w, nuts, ("s" if nuts != 1 else ""), sigma,
		])
		# Resolved body + difficulty params — so the snapshot self-reports the
		# ACTUAL run config (caught a launcher-vs-headless body_model mismatch).
		lines.append("Terrain/pillars: amp=%.2f  density=%.2f" % [
			float(world.get("terrain_amplitude")), float(world.get("obstacle_density")),
		])
	if body:
		var bm: String = str(body.get("body_model"))
		var drain: float = float(body.get("_energy_drain_per_sec"))
		var vgg: float = float(body.get("_vision_green_gain"))
		lines.append("Body:        model=%s  drain=%.4f/s  green_gain=%.2f" % [bm, drain, vgg])
	lines.append("")

	# --- Body progress ---
	if body:
		var hits: int = int(body.get("hits_total"))
		lines.append("Eats / hits: %d" % hits)
		# --- Honest locomotion block (2026-07-08 instrument audit) ---
		# The old "Stuck %", "Mean fwd speed (target 3.0)" and "Spikes/tick"
		# lines were miscalibrated for the continuous bidirectional_paddler
		# (spikes decoupled from thrust; move_speed=3.0 is the asymmetric
		# cruise, not this body's; stuck vs that reference flagged normal
		# slow/pausing motion).  These read the ACTUAL effort + the body's own
		# velocity integrator, and separate genuine pinning from commanded coast.
		var avg_count: int = int(body.get("_avg_count"))
		if avg_count > 0:
			var speed_sum: float = float(body.get("_speed_sum_for_avg"))
			var ang_sum: float = float(body.get("_angular_sum_for_avg"))
			var effort_sum: float = float(body.get("_motor_effort_sum"))
			var pause_t: int = int(body.get("_pause_ticks"))
			var pin_t: int = int(body.get("_stuck_actual_pin_ticks"))
			var body_max: float = float(body.call("_stuck_reference_speed"))
			var mean_fwd: float = speed_sum / float(avg_count)
			lines.append("Locomotion (honest, %s):" % str(body.get("body_model")))
			lines.append("  Motor effort:  mean %.2f / 2.0   (pause %.0f%% = commanded coast)" % [
				effort_sum / float(avg_count), 100.0 * float(pause_t) / float(avg_count)])
			lines.append("  Fwd speed:     mean %.2f m/s   (body max ~%.1f → %.0f%% of max)" % [
				mean_fwd, body_max, 100.0 * mean_fwd / maxf(body_max, 1e-6)])
			lines.append("  Pinning:       %.0f%% of ticks — net progress < ½ commanded motion (wall/circle)" % [
				100.0 * float(pin_t) / float(avg_count)])
			lines.append("  Mean |ang vel|: %.2f rad/s" % (ang_sum / float(avg_count)))
		# Behaviour-trigger rate (below move_speed ref) — NOT a wedged metric;
		# it drives the stuck-pulse the brain uses as exploration noise.  The
		# wall/refr/zero breakdown attributes the ticks the pulse fired on.
		var stuck: float = float(body.get("_stuck_severity"))
		var stuck_total: int = int(body.get("_stuck_total_ticks"))
		var s_extra := ""
		if stuck_total > 0:
			s_extra = "   [of those: wall %.0f%% refr %.0f%% zero %.0f%%]" % [
				100.0 * float(int(body.get("_stuck_wall_ticks"))) / float(stuck_total),
				100.0 * float(int(body.get("_stuck_refrac_ticks"))) / float(stuck_total),
				100.0 * float(int(body.get("_stuck_zero_steer_ticks"))) / float(stuck_total)]
		lines.append("Pulse trigger (below move_speed ref, NOT wedged): %.0f%%%s" % [stuck * 100.0, s_extra])
		# Phase 6.5.19 Part B — adaptive threshold convergence.
		var scent_thresh: float = float(body.get("_scent_diff_mean_ema")) + 2.0 * sqrt(maxf(float(body.get("_scent_diff_var_ema")), 1e-12))
		var whisk_thresh: float = float(body.get("_whisker_mean_ema")) + 2.0 * sqrt(maxf(float(body.get("_whisker_var_ema")), 1e-12))
		var fallback: bool = bool(body.get("_adaptive_fallback_active"))
		lines.append("Adaptive thresholds:  scent_diff > %.4f   whisker > %.3f%s" % [
			scent_thresh, whisk_thresh,
			"   [fallback active]" if fallback else "",
		])
	lines.append("")

	# --- Brain state ---
	if body:
		var brain = body.get("brain")
		if brain != null and brain.has_method("get_module_metrics"):
			lines.append("=== Final brain metrics ===")
			var metrics: Dictionary = brain.get_module_metrics()
			for mod_id in metrics:
				var m: Dictionary = metrics[mod_id]
				var t: String = str(m.get("type", ""))
				match t:
					"NeurochemState":
						lines.append("  %s: da=%.3f ht=%.3f r_sig=%.3f" % [
							mod_id, float(m.get("dopamine", 0)),
							float(m.get("serotonin", 0)),
							float(m.get("reward_signal", 0)),
						])
					"EPM":
						lines.append("  %s: nodes=%d baked=%d tle=%.4f" % [
							mod_id, int(m.get("node_count", 0)),
							int(m.get("baked_count", 0)),
							float(m.get("tle", 0)),
						])
					"LateralVoter":
						var trust: Dictionary = m.get("trust_weights", {})
						var trust_str := ""
						for k in trust:
							trust_str += "%s=%.2f " % [k, float(trust[k])]
						lines.append("  %s: tle=%.4f modality=%s   trust: %s" % [
							mod_id, float(m.get("fused_tle", 0)),
							str(m.get("active_modality", "")),
							trust_str.strip_edges(),
						])
					"HomeostaticDrive":
						var errs: Dictionary = m.get("errors", {})
						var errs_str := ""
						for k in errs:
							errs_str += "%s=%.3f " % [k, float(errs[k])]
						lines.append("  %s: urg=%.3f   %s" % [
							mod_id, float(m.get("urgency", 0)),
							errs_str.strip_edges(),
						])
					"ActionDecoder":
						lines.append("  %s: accel=%.2f val_sz=%d active_chunk=%d disp=%d" % [
							mod_id, float(m.get("accel", 0)),
							int(m.get("valence_size", 0)),
							int(m.get("active_chunk_id", -1)),
							int(m.get("chunk_dispatch_count", 0)),
						])
					"SequenceGNG":
						lines.append("  %s: nodes=%d baked=%d motif=%d" % [
							mod_id, int(m.get("node_count", 0)),
							int(m.get("baked_count", 0)),
							int(m.get("current_motif", -1)),
						])
					"MotorRepertoire":
						lines.append("  %s: chunks=%d active=%d disp=%d failed=%d" % [
							mod_id, int(m.get("chunk_count", 0)),
							int(m.get("active_chunk_count", 0)),
							int(m.get("total_dispatch_count", 0)),
							int(m.get("failed_dispatch_count", 0)),
						])
					"HomeokineticExploration":
						lines.append("  %s: armed=%d active=%s" % [
							mod_id, int(m.get("episodes_armed", 0)),
							str(m.get("active", false)),
						])
			# --- Chunk library detail ---
			for mod_id in metrics:
				var m: Dictionary = metrics[mod_id]
				if str(m.get("type", "")) != "MotorRepertoire": continue
				var chunks = m.get("chunks", null)
				if chunks == null or not (chunks is Array): continue
				if chunks.is_empty(): continue
				lines.append("")
				lines.append("=== Chunk library (%d chunks) ===" % chunks.size())
				for c in chunks:
					var seq: Array = c.get("accel_seq", [])
					var seq_str := ""
					for v in seq:
						seq_str += "%+.2f," % float(v)
					if seq_str.length() > 0:
						seq_str = seq_str.substr(0, seq_str.length() - 1)
					lines.append("  id=%d len=%d use=%d trigger_motif=%d trigger_urg=%.3f hits=%d/%d/%d" % [
						int(c.get("id", -1)),
						int(c.get("length", 0)),
						int(c.get("use_count", 0)),
						int(c.get("trigger_motif", -1)),
						float(c.get("trigger_urgency", -1.0)),
						int(c.get("hits_during", 0)),
						int(c.get("replay_hits", 0)),
						int(c.get("replay_misses", 0)),
					])
					lines.append("    seq=[%s]" % seq_str)
	return "\n".join(lines)
