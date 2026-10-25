extends Control
# range_graph.gd — the FORWARD ultrasonic, as a scrolling time series (bench dashboard).
#
# The HC-SR04-class module looks forward from the chassis front face (Ultrasonic.hpp). It is
# NOT the belly ToF; clearance_graph.gd is that one, and the two must never be read as one
# channel.
#
# WHY A GRAPH. The question this panel exists for (2026-10-25) is a GATE: at what hip1 angle
# does a front leg swing into the beam? A leg entering the lobe shows as a sudden short
# reading that comes and goes with the stride, which a single number hides and a trace shows.
#
# ⚠ A NO-ECHO PING IS NOT A DISTANCE. The driver reports max_range_m with valid = false there
# (the honest floor on a distance nothing came back from). Plotting it would draw a wall at
# 1.5 m, so the trace BREAKS across no-echo samples and they are marked on their own track.
#
# The reference lines are the brain's own: ProxMemory's 0.5 m horizon (an echo inside it is a
# remembered obstacle) and its 0.15 m "too close" (robot_p1 / sim_s60_p30 configs).

const WINDOW_S := 60.0
const CAP := 600                       # WINDOW_S at the 10 Hz poll
const VIEW_TOP_M := 1.5                # the driver's max_range_m: fixed, so near and far keep their place
const HORIZON_M := 0.5                 # ProxMemory range_m
const TOO_CLOSE_M := 0.15              # ProxMemory too_close_m

var _m: Array[float] = []
var _ok: Array[bool] = []
var _live := false
var _hz := 0.0


func _ready() -> void:
	custom_minimum_size = Vector2(0, 76)
	mouse_filter = Control.MOUSE_FILTER_STOP
	tooltip_text = ("Forward ultrasonic from ogma_host (host_sensors) — NOT the belly ToF.\n"
		+ "Lines: 0.5 m = the avoid reflex's horizon, 0.15 m = 'too close'.\n"
		+ "GAPS ARE REAL: no-echo pings break the trace and are marked on the bottom track;\n"
		+ "drawing them as a distance would show a wall at the far limit.\n"
		+ "A leg in the beam reads as short spikes that come and go with the stride.")


func push(m: float, valid: bool, hz: float) -> void:
	_live = true
	_hz = hz
	_m.append(m)
	_ok.append(valid)
	if _m.size() > CAP:
		_m = _m.slice(_m.size() - CAP)
		_ok = _ok.slice(_ok.size() - CAP)
	queue_redraw()


func _y_of(v: float, h: float) -> float:
	return h - clamp(v / VIEW_TOP_M, 0.0, 1.0) * h


func _draw() -> void:
	var font := ThemeDB.fallback_font
	var h := size.y - 12.0
	draw_rect(Rect2(0, 0, size.x, h), Color(0.10, 0.10, 0.12))
	if not _live or _m.is_empty():
		draw_string(font, Vector2(4, h * 0.5), "range: waiting for ogma_host",
					HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(0.55, 0.55, 0.55))
		return
	var y_close := _y_of(TOO_CLOSE_M, h)
	draw_rect(Rect2(0, y_close, size.x, h - y_close), Color(0.32, 0.11, 0.11, 0.55))
	draw_line(Vector2(0, y_close), Vector2(size.x, y_close), Color(1.0, 0.35, 0.35, 0.8), 1.0)
	var y_hor := _y_of(HORIZON_M, h)
	draw_line(Vector2(0, y_hor), Vector2(size.x, y_hor), Color(1.0, 0.8, 0.4, 0.5), 1.0)
	draw_string(font, Vector2(2, y_hor - 2), "0.5 m reflex horizon", HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1.0, 0.8, 0.4, 0.8))
	# One column per pixel as the min–max of its samples: MIN matters, a leg flicking through
	# the beam for one ping is the sample worth having.
	var n := _m.size()
	var cols := int(max(1.0, size.x))
	var per := float(n) / float(cols)
	var prev_y := -1.0
	var valid_n := 0
	for k in range(n):
		if _ok[k]: valid_n += 1
	for c in range(cols):
		var a0 := int(floor(c * per))
		var a1 := int(min(float(n), max(float(a0 + 1), (c + 1) * per)))
		if a0 >= n:
			break
		var mn := 0.0
		var mx := 0.0
		var any := false
		var miss := false
		for k in range(a0, a1):
			if not _ok[k]:
				miss = true
				continue
			if not any:
				mn = _m[k]; mx = _m[k]; any = true
			else:
				mn = min(mn, _m[k]); mx = max(mx, _m[k])
		if miss:
			draw_rect(Rect2(c, h - 3.0, 1.0, 3.0), Color(0.6, 0.6, 0.7, 0.8))
		if not any:
			prev_y = -1.0
			continue
		var ymn := _y_of(mn, h)
		var ymx := _y_of(mx, h)
		var col := Color(0.55, 0.85, 0.75)
		if mn <= TOO_CLOSE_M: col = Color(1.0, 0.35, 0.35)
		elif mn <= HORIZON_M: col = Color(1.0, 0.80, 0.40)
		draw_line(Vector2(c, ymx), Vector2(c, max(ymn, ymx + 1.0)), col, 1.0)
		if prev_y >= 0.0:
			draw_line(Vector2(c - 1, prev_y), Vector2(c, ymx), col, 1.0)
		prev_y = ymn
	var txt := "echo %.0f%% of pings   %.1f Hz   0–1.5 m" % [100.0 * valid_n / max(1, n), _hz]
	draw_string(font, Vector2(2, size.y - 2), txt, HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.85, 0.85, 0.85))
	draw_string(font, Vector2(size.x - 62, size.y - 2), "%.0f s" % WINDOW_S,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.5, 0.5, 0.5))
