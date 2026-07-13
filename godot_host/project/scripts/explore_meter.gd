extends Control
## ExploreMeter — real-time EXPLORING ↔ HOMING gauge (2026-06-21).
##
## Reads the coxswain ActionDecoder's plan_entropy (normalized softmax entropy of
## the planner's action distribution) each frame and draws a horizontal bar:
##   entropy ~1 = flat softmax = EXPLORING (probing, uncertain)  → amber/red
##   entropy ~0 = peaked softmax = HOMING (confidently committing) → green
## So the operator can SEE in real time whether the bug is randomly exploring or
## confidently choosing actions that lead to eating.  Also shows plan_confidence
## (softmax prob of the chosen action).  Hides when no ActionDecoder reports it
## (e.g. argmax-only configs, plan_temperature=0 → entropy pinned 0 = always
## "homing", which is correct: no exploration).
##
## Construct, anchor, call set_brain(b).  Used by hud.gd (Cell).

const _BAR_W: int = 200
const _BAR_H: int = 16
const _PAD:   int = 6
const _COL_HOMING  := Color(0.30, 0.90, 0.45)   # green (confident)
const _COL_MID     := Color(0.95, 0.85, 0.35)   # amber
const _COL_EXPLORE := Color(0.95, 0.45, 0.35)   # red (exploring)
const _COL_BG      := Color(0.10, 0.10, 0.10, 0.55)
const _COL_BORDER  := Color(0.85, 0.85, 0.85, 0.85)
const _COL_TEXT    := Color(1, 1, 1, 1)

var _brain: OgmaBrain = null
var _font: Font = null
var _entropy: float = 0.0
var _conf: float = 1.0
var _have: bool = false

func _ready() -> void:
	custom_minimum_size = Vector2(_BAR_W + 2 * _PAD, _BAR_H + 2 * _PAD + 18)
	size = custom_minimum_size
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_font = ThemeDB.fallback_font

func set_brain(b: OgmaBrain) -> void:
	_brain = b

func _process(_delta: float) -> void:
	if _brain == null or not _brain.has_method("is_brain_ready") or not _brain.is_brain_ready():
		if visible: visible = false
		return
	var m: Dictionary = _brain.get_module_metrics()
	# find the ActionDecoder entry (id varies: action_plan / action_decoder).
	var ad: Dictionary = {}
	for mid in m:
		var e: Dictionary = m[mid]
		if String(e.get("type", "")) == "ActionDecoder":
			ad = e
			break
	_have = ad.has("plan_entropy")
	if visible != _have:
		visible = _have
	if _have:
		_entropy = clampf(float(ad.get("plan_entropy", 0.0)), 0.0, 1.0)
		_conf    = clampf(float(ad.get("plan_confidence", 1.0)), 0.0, 1.0)
		queue_redraw()

func _draw() -> void:
	if not _have:
		return
	# label
	var state := "HOMING" if _entropy < 0.25 else ("EXPLORING" if _entropy > 0.6 else "mixed")
	draw_string(_font, Vector2(_PAD, _PAD + 11),
		"%s   H=%.2f  conf=%.2f" % [state, _entropy, _conf],
		HORIZONTAL_ALIGNMENT_LEFT, -1, 11, _COL_TEXT)
	var bar_y: int = _PAD + 16
	var rect := Rect2(_PAD, bar_y, _BAR_W, _BAR_H)
	draw_rect(rect, _COL_BG, true)
	# fill proportional to entropy; colour green(homing)→amber→red(exploring).
	var fill_w: int = int(round(_BAR_W * _entropy))
	for i in range(max(1, fill_w)):
		var t: float = float(i) / float(maxi(1, fill_w - 1)) if fill_w > 1 else 0.0
		var c: Color = _COL_HOMING.lerp(_COL_MID, t / 0.5) if t < 0.5 else _COL_MID.lerp(_COL_EXPLORE, (t - 0.5) / 0.5)
		draw_rect(Rect2(_PAD + i, bar_y, 1, _BAR_H), c, true)
	draw_rect(rect, _COL_BORDER, false, 1.0)
	# edge labels
	draw_string(_font, Vector2(_PAD, bar_y + _BAR_H + 10), "homing",
		HORIZONTAL_ALIGNMENT_LEFT, -1, 9, _COL_HOMING)
	var ex := "exploring"
	var w := _font.get_string_size(ex, HORIZONTAL_ALIGNMENT_LEFT, -1, 9).x
	draw_string(_font, Vector2(_PAD + _BAR_W - w, bar_y + _BAR_H + 10), ex,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 9, _COL_EXPLORE)
