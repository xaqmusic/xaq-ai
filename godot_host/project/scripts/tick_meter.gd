extends Control
# tick_meter.gd — a PPM-style meter for tick-budget utilisation.
#
# The audio analogy is exact and load-bearing, not decoration:
#   0 dB (full scale) = 100 % of the tick budget = a missed deadline. Clipping here
#   means the same thing it means on a converter — the signal did not fit.
#
# Scale is LOGARITHMIC for the same reason audio meters are. The working range spans
# three orders of magnitude (an idle loop sits near 0.005 %, a busy one near 20 %), and
# on a linear 0–100 bar the entire healthy range is a sliver against the left edge.
# In dB the same range is legible: 1 % = −40 dB, 18 % = −15 dB, 50 % = −6 dB.
# Gridlines are labelled in PER CENT so the log spacing never has to be read as dB.
#
# Average and peak share one meter, as on a PPM: the solid bar is the window's p50, the
# lighter extension runs to p95, and the bright needle is a peak-hold at the window MAX
# that holds then falls back. A mean alone is blind to the one tick in forty that blows
# the budget — the needle is what makes that tick visible.

const DB_MIN := -60.0          # 0.1 % of budget
const DB_MAX := 6.0            # 200 % — headroom above clip so an overrun has somewhere to go
const HOLD_S := 1.5            # peak sits still this long...
const FALL_DB_S := 24.0        # ...then falls at a PPM-ish rate
const GRID := [1.0, 5.0, 10.0, 25.0, 50.0, 100.0]

var _p50 := 0.0
var _p95 := 0.0
var _max := 0.0
var _peak := 0.0               # held peak, in per cent
var _peak_age := 0.0
var _clipped := false          # latches; the operator clears it
var _live := false


func _ready() -> void:
	custom_minimum_size = Vector2(0, 34)
	set_process(true)
	mouse_filter = Control.MOUSE_FILTER_STOP
	tooltip_text = "Tick-budget meter. 0 dB = 100 % of the budget = missed deadline.\nBar = p50, light = p95, needle = peak hold. Click to clear the clip latch."


func set_window(p50: float, p95: float, mx: float) -> void:
	_live = true
	_p50 = p50
	_p95 = p95
	_max = mx
	if mx >= _peak:
		_peak = mx
		_peak_age = 0.0
	if mx >= 100.0:
		_clipped = true
	queue_redraw()


func clear_clip() -> void:
	_clipped = false
	_peak = _max
	queue_redraw()


func _gui_input(e: InputEvent) -> void:
	if e is InputEventMouseButton and e.pressed:
		clear_clip()


func _process(dt: float) -> void:
	if _peak <= 0.0:
		return
	_peak_age += dt
	if _peak_age > HOLD_S:
		# Fall in dB, not in per cent: a linear fall would crawl at the top and
		# plummet at the bottom, which is not how a peak meter reads.
		var db := _pct_to_db(_peak) - FALL_DB_S * dt
		_peak = max(_pct_from_db(db), _max)
		queue_redraw()


func _pct_to_db(pct: float) -> float:
	return DB_MIN if pct <= 0.0 else clamp(20.0 * (log(pct / 100.0) / log(10.0)), DB_MIN, DB_MAX)


func _pct_from_db(db: float) -> float:
	return 100.0 * pow(10.0, db / 20.0)


func _x(pct: float) -> float:
	var t: float = (_pct_to_db(pct) - DB_MIN) / (DB_MAX - DB_MIN)
	return clamp(t, 0.0, 1.0) * size.x


func _zone(pct: float) -> Color:
	if pct >= 100.0: return Color(1.0, 0.30, 0.30)
	if pct >= 75.0:  return Color(1.0, 0.55, 0.30)
	if pct >= 50.0:  return Color(1.0, 0.85, 0.40)
	return Color(0.35, 0.85, 0.45)


func _draw() -> void:
	var font := ThemeDB.fallback_font
	var bar_h := 16.0
	var bar := Rect2(0, 0, size.x, bar_h)
	draw_rect(bar, Color(0.12, 0.12, 0.14))

	# the danger band above 75 % of budget, drawn behind the signal
	var x75 := _x(75.0)
	draw_rect(Rect2(x75, 0, size.x - x75, bar_h), Color(0.35, 0.12, 0.12))

	if _live:
		var xa := _x(_p50)
		var xb := _x(_p95)
		draw_rect(Rect2(0, 0, xa, bar_h), _zone(_p50))
		if xb > xa:
			var c := _zone(_p95); c.a = 0.45
			draw_rect(Rect2(xa, 0, xb - xa, bar_h), c)
		var xp := _x(_peak)                                  # the peak-hold needle
		draw_rect(Rect2(max(xp - 1.0, 0.0), -1.0, 2.0, bar_h + 2.0), Color(1, 1, 1, 0.95))

	# gridlines, labelled in per cent because that is the quantity, dB is only the spacing
	for g in GRID:
		var gx := _x(g)
		draw_line(Vector2(gx, 0), Vector2(gx, bar_h), Color(1, 1, 1, 0.18), 1.0)
		draw_string(font, Vector2(gx + 2, bar_h + 11), "%d%%" % int(g),
					HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.6, 0.6, 0.6))

	if _clipped:
		var w := 34.0
		draw_rect(Rect2(size.x - w, 0, w, bar_h), Color(1, 0.2, 0.2))
		draw_string(font, Vector2(size.x - w + 3, bar_h - 4), "CLIP",
					HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(0, 0, 0))
	if _live:
		draw_string(font, Vector2(2, bar_h + 11),
					"p50 %.2f%%   p95 %.2f%%   peak %.1f%%" % [_p50, _p95, _peak],
					HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.85, 0.85, 0.85))
