extends Control
## BrainStateIndicator (v5.4.I/J — UI #3)
##
## Persistent HUD overlay showing what is *actually* driving the body's motor
## command this tick.  Three motor sources may be active in any blend:
##
##   CHUNK     — ActionDecoder is force-dispatching a chunk; Premotor's
##               chosen intent is overridden by the chunk's intent_sequence.
##   PREMOTOR  — no chunk active; Premotor is sampling/argmaxing its own
##               policy distribution.  This is the case the user calls
##               "circling for no apparent reason" — Premotor is always-on
##               and produces some intent every tick.
##   REFLEX    — α=0 (or close): MotorFader is publishing the reflex pathway
##               only; brain motor is ignored regardless of what Premotor
##               chose.
##
## Also surfaces the v5.4 clash metric — how much intent is being erased
## when brain and reflex contributions disagree in sign.  A high clash
## EMA = "your brain and reflex are constantly fighting each other"; a
## low clash = "the two pathways agree, blend is purely additive".
##
## Polls OgmaBrain.get_motor_fader_state(), get_active_chunk_id(),
## get_module_metrics() per frame.  Hides when fader_state is empty
## (the same condition motor_fader_meter uses).
##
## Convention: rendered as a multi-line Label anchored to the top-left
## corner so it stays clear of motor_fader_meter (top-center) and the
## chunk-probe status (top-right).

const _PAD: int = 8
const _COL_BG     := Color(0.10, 0.10, 0.10, 0.60)
const _COL_TEXT   := Color(1, 1, 1, 1)
const _COL_BORDER := Color(0.85, 0.85, 0.85, 0.70)

const _SOURCE_COL := {
	"CHUNK":    Color(0.40, 0.95, 0.55),   # green
	"PREMOTOR": Color(0.95, 0.85, 0.35),   # amber
	"REFLEX":   Color(0.55, 0.75, 0.95),   # blue
	"IDLE":     Color(0.75, 0.75, 0.75),   # grey
}

# Premotor's default 5-intent table — repeated here for label-display only.
# Matches Premotor.cpp L[5]={+4,+2,+4,0,-4} R[5]={-4,0,+4,+2,+4} (cell config).
const _INTENT_LABELS := [
	"L+4 R-4",   # 0 — hard LEFT pivot
	"L+2 R 0",   # 1 — soft LEFT
	"L+4 R+4",   # 2 — FORWARD
	"L 0 R+2",   # 3 — soft RIGHT
	"L-4 R+4",   # 4 — hard RIGHT pivot
]

var _brain: OgmaBrain = null
var _label: Label = null

func _ready() -> void:
	custom_minimum_size = Vector2(300, 150)
	size = custom_minimum_size
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_label = Label.new()
	_label.position = Vector2(_PAD, _PAD)
	_label.size     = Vector2(custom_minimum_size.x - 2 * _PAD,
							  custom_minimum_size.y - 2 * _PAD)
	_label.add_theme_font_size_override("font_size", 12)
	_label.add_theme_color_override("font_color", _COL_TEXT)
	_label.add_theme_color_override("font_outline_color", Color(0, 0, 0))
	_label.add_theme_constant_override("outline_size", 3)
	add_child(_label)

func set_brain(b: OgmaBrain) -> void:
	_brain = b

func _process(_dt: float) -> void:
	if _brain == null or not _brain.is_brain_ready():
		if visible: visible = false
		return
	var fader: Dictionary = _brain.get_motor_fader_state()
	# Hide when no fader is in the graph — same convention as
	# motor_fader_meter.gd.  Without a fader, the body reads motor inputs
	# from a single channel and "blend" semantics don't apply.
	var should_show: bool = not fader.is_empty()
	if visible != should_show:
		visible = should_show
	if not visible: return

	var alpha: float = float(fader.get("alpha", 0.0))
	var brain_seen: bool   = bool(fader.get("brain_seen",  false))
	var reflex_seen: bool  = bool(fader.get("reflex_seen", false))
	var brain_accel: float = float(fader.get("brain_accel", 0.0))
	var reflex_accel: float= float(fader.get("reflex_accel", 0.0))
	var output_accel: float= float(fader.get("output_accel", 0.0))

	# v5.4 — bilateral clash.  get_motor_fader_state() returns ONE side's
	# numbers (whichever MotorFader the C++ loop hit first).  For an honest
	# read of "are brain and reflex fighting", iterate metrics and find
	# every MotorFader instance — their last_clash + clash_ema are exposed
	# per-module since v5.4.J.  Sum across instances so a high clash on
	# either side surfaces.
	var clash_left: float       = 0.0
	var clash_right: float      = 0.0
	var clash_left_ema: float   = 0.0
	var clash_right_ema: float  = 0.0
	var fader_count: int = 0
	var metrics_for_fader: Dictionary = (_brain.get_module_metrics()
		if _brain.has_method("get_module_metrics") else {})
	for mod_id in metrics_for_fader:
		var fm: Dictionary = metrics_for_fader[mod_id]
		if String(fm.get("type", "")) != "MotorFader": continue
		var c: float = float(fm.get("clash", 0.0))
		var ce: float = float(fm.get("clash_ema", 0.0))
		# Heuristic: id contains "right" → right channel; everything else
		# → left channel.  Single-channel configs land everything in left.
		if String(mod_id).find("right") >= 0:
			clash_right     = c
			clash_right_ema = ce
		else:
			clash_left      = c
			clash_left_ema  = ce
		fader_count += 1
	var clash_total: float     = clash_left + clash_right
	var clash_total_ema: float = clash_left_ema + clash_right_ema

	# Determine motor source.  Order matters: CHUNK > PREMOTOR > REFLEX.
	# A chunk dispatching at α=0 still counts as CHUNK (intent is being
	# computed) but no motor effect — show it as CHUNK and the user can
	# read α to understand the brain has no authority.
	var chunk_id: int = -1
	if _brain.has_method("get_active_chunk_id"):
		chunk_id = int(_brain.get_active_chunk_id())
	var source: String
	if chunk_id != -1:
		source = "CHUNK"
	elif alpha <= 0.001:
		source = "REFLEX"
	elif brain_seen:
		source = "PREMOTOR"
	else:
		source = "IDLE"

	# Premotor last_chosen — peek at the metrics dict.  Cheap: same call
	# body_controller uses each tick for JSONL.
	var chosen_intent: int = -1
	var metrics: Dictionary = (_brain.get_module_metrics()
		if _brain.has_method("get_module_metrics") else {})
	for mod_id in metrics:
		var m: Dictionary = metrics[mod_id]
		if String(m.get("type", "")) == "Premotor":
			chosen_intent = int(m.get("last_chosen", -1))
			break

	# Build the label.  Multi-line monospace so columns align.
	var intent_str: String = "—"
	if chosen_intent >= 0 and chosen_intent < _INTENT_LABELS.size():
		intent_str = "%d (%s)" % [chosen_intent, _INTENT_LABELS[chosen_intent]]
	var chunk_str: String = "(none)"
	if chunk_id != -1:
		chunk_str = "id=%d" % chunk_id
	var bilateral_str: String = "L=%+.2f  R=%+.2f" % [
		_brain.get_action_left() if _brain.has_method("get_action_left") else 0.0,
		_brain.get_action_right() if _brain.has_method("get_action_right") else 0.0,
	]
	# Single-channel fallback values for the legacy meter path.
	var legacy_line: String = "brain=%+.2f  reflex=%+.2f  out=%+.2f" % [
		brain_accel, reflex_accel, output_accel]

	_label.text = ("Motor source: %s\nChunk:   %s\nIntent:  %s\nMotor:   %s\n"
				   + "α: %.2f  (legacy: %s)\n"
				   + "Clash:   L=%.3f  R=%.3f  total=%.3f\n"
				   + "Clash ema: L=%.3f  R=%.3f") % [
		source, chunk_str, intent_str, bilateral_str, alpha, legacy_line,
		clash_left, clash_right, clash_total,
		clash_left_ema, clash_right_ema]

	# Tint the first line ("Motor source: X") via outline-colour swap.
	# Cheap visual cue without per-line font themes.
	var col: Color = _SOURCE_COL.get(source, _COL_TEXT)
	_label.add_theme_color_override("font_outline_color", col)

	queue_redraw()

func _draw() -> void:
	var rect := Rect2(Vector2.ZERO, size)
	draw_rect(rect, _COL_BG, true)
	draw_rect(rect, _COL_BORDER, false, 1.0)
