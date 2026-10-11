extends Control
# current_graph.gd — whole-robot bus current, as a scrolling time series.
#
# The INA219 sits inline on the battery input (BOM §3), so this is the Pi, the 5 V
# regulator and all 12 servos together. One number for the whole machine.
#
# WHY A GRAPH AND NOT A METER. The tick meter next door answers "how loaded is this
# tick"; a scalar is enough for that. This answers a different question — "WHICH
# MOVEMENT cost that" — and a scalar cannot, because the spike and the behaviour that
# caused it are separated in time by the length of the move. So the pose-move band runs
# along the bottom: the correlation IS the instrument, and a current trace with no
# record of what the body was doing is just a wiggle.
#
# SIGN IS LOAD-BEARING. Charge current flows backwards through the shunt, so negative
# means the charger is plugged in — and while it is, every energy number is confounded.
# Zero is drawn, always, rather than autoscaling it away.
#
# The 3 A line is the HAT's 5 V regulator rating, which is a datasheet fact. It is NOT
# the duty budget: BOM §3.9 measured that to be surface-dependent (the same move costs
# 1.90 A on vinyl and 2.62 A on leather), so a fixed budget line here would be a lie
# drawn in a place people trust.

const WINDOW_S := 60.0                 # how much history the width holds
const RAIL_A := 3.0                    # HAT 5 V regulator rating (datasheet, not a budget)
const CAP := 600                       # ring capacity = WINDOW_S at the daemon's 10 Hz

var _i: Array[float] = []              # amps, oldest first
var _busy: Array[bool] = []            # was a pose move active at that sample
var _ema := 0.0
var _peak := 0.0
var _charging := false
var _live := false


func _ready() -> void:
	custom_minimum_size = Vector2(0, 76)
	mouse_filter = Control.MOUSE_FILTER_STOP
	tooltip_text = ("Whole-robot bus current: Pi + 5 V regulator + 12 servos.\n"
		+ "Negative = charging. Red line = the HAT's 3 A rail rating, not a duty budget.\n"
		+ "The band along the bottom marks pose-move activity — read spikes against it.\n"
		+ "Click to clear the peak.")


func push(amps: float, ema: float, peak: float, charging: bool, busy: bool) -> void:
	_live = true
	_ema = ema
	_peak = peak
	_charging = charging
	_i.append(amps)
	_busy.append(busy)
	if _i.size() > CAP:
		_i = _i.slice(_i.size() - CAP)
		_busy = _busy.slice(_busy.size() - CAP)
	queue_redraw()


func clear_peak() -> void:
	_peak = 0.0
	queue_redraw()


func _gui_input(e: InputEvent) -> void:
	if e is InputEventMouseButton and e.pressed:
		clear_peak()


func _y_of(a: float, lo: float, hi: float, h: float) -> float:
	if hi <= lo:
		return h
	return h - clamp((a - lo) / (hi - lo), 0.0, 1.0) * h


func _draw() -> void:
	var font := ThemeDB.fallback_font
	var h := size.y - 12.0                     # leave a strip for the legend
	draw_rect(Rect2(0, 0, size.x, h), Color(0.10, 0.10, 0.12))
	if not _live or _i.is_empty():
		draw_string(font, Vector2(4, h * 0.5), "current: waiting for telemetry",
					HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(0.55, 0.55, 0.55))
		return

	# Autoscale, but ALWAYS keep zero and the rail line on screen: the two references
	# that give the trace meaning are the two an autoscaler would throw away first.
	var lo := 0.0
	var hi := RAIL_A * 1.1
	for a in _i:
		lo = min(lo, a)
		hi = max(hi, a)
	hi *= 1.05

	# the zone above the rail rating, behind the trace
	var y_rail := _y_of(RAIL_A, lo, hi, h)
	draw_rect(Rect2(0, 0, size.x, y_rail), Color(0.32, 0.11, 0.11, 0.55))
	draw_line(Vector2(0, y_rail), Vector2(size.x, y_rail), Color(1.0, 0.35, 0.35, 0.8), 1.0)
	draw_string(font, Vector2(2, y_rail - 2), "%.0f A rail" % RAIL_A,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1.0, 0.5, 0.5))

	# zero, drawn whether or not anything is below it — its absence is what hides charging
	var y0 := _y_of(0.0, lo, hi, h)
	draw_line(Vector2(0, y0), Vector2(size.x, y0), Color(1, 1, 1, 0.25), 1.0)

	# One column per pixel, drawn as the min–max of the samples that fall in it. A mean
	# per column would smooth away the one spike the graph exists to show.
	var n := _i.size()
	var cols := int(max(1.0, size.x))
	var per := float(n) / float(cols)
	var prev_y := -1.0
	for c in range(cols):
		var a0 := int(floor(c * per))
		var a1 := int(min(float(n), max(float(a0 + 1), (c + 1) * per)))
		if a0 >= n:
			break
		var mn := _i[a0]
		var mx := _i[a0]
		var busy := false
		for k in range(a0, a1):
			mn = min(mn, _i[k])
			mx = max(mx, _i[k])
			busy = busy or _busy[k]
		var ymn := _y_of(mn, lo, hi, h)
		var ymx := _y_of(mx, lo, hi, h)
		var col := Color(0.45, 0.80, 1.0)
		if mx >= RAIL_A: col = Color(1.0, 0.35, 0.35)
		elif mx >= RAIL_A * 0.75: col = Color(1.0, 0.80, 0.40)
		if _charging and mx <= 0.0: col = Color(0.55, 0.95, 0.55)
		draw_line(Vector2(c, ymx), Vector2(c, max(ymn, ymx + 1.0)), col, 1.0)
		# join columns so a fast edge reads as an edge, not as two disconnected bars
		if prev_y >= 0.0:
			draw_line(Vector2(c - 1, prev_y), Vector2(c, ymx), col, 1.0)
		prev_y = ymx
		# the behaviour track: WHICH movement cost that spike
		if busy:
			draw_rect(Rect2(c, h - 3.0, 1.0, 3.0), Color(1.0, 0.85, 0.35, 0.9))

	# the slow metric, flat across the window it was measured over
	var y_ema := _y_of(_ema, lo, hi, h)
	draw_line(Vector2(0, y_ema), Vector2(size.x, y_ema), Color(0.9, 0.9, 0.5, 0.5), 1.0)

	var now: float = _i[_i.size() - 1]
	var txt := "%+.3f A   ema30 %+.3f   peak60 %.3f" % [now, _ema, _peak]
	if _charging:
		txt += "   CHARGING (energy numbers confounded)"
	draw_string(font, Vector2(2, size.y - 2), txt,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9,
				Color(0.55, 0.95, 0.55) if _charging else Color(0.85, 0.85, 0.85))
	draw_string(font, Vector2(size.x - 62, size.y - 2), "%.0f s" % WINDOW_S,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.5, 0.5, 0.5))
