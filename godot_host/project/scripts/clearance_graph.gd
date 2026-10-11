extends Control
# clearance_graph.gd — belly clearance, as a scrolling time series.
#
# The VL53L0X points DOWN at the belly (BOM §2 #4, §7), so this is the gap between the
# chassis underside and whatever is beneath it. It is NOT the forward ultrasonic, and
# the two must never be read as one channel.
#
# WHY A GRAPH. The row above already gives the number, and for "how high is it sitting"
# a number is enough. This answers what a number cannot: belly contact is a TRANSIENT.
# A chassis that touches down for 200 ms during a step and lifts again shows up here as
# a spike into the drag band, and shows up in the mean as nothing at all. `bellyc_min`
# sat at 0.000–0.004 for an entire campaign without anyone being able to say WHEN.
#
# ⚠ AN INVALID READING IS NOT A DISTANCE, SO IT IS NOT DRAWN AS ONE. When the part
# rejects its own measurement, benchd reports the far limit — the honest floor on a
# distance nothing came back from (Vl53l0x.hpp). Plotting that would show the belly
# leaping to a metre in the air, which is the opposite of what happened. So the TRACE
# BREAKS across invalid samples and they are marked on their own track instead. A gap
# in this graph means "the sensor could not see the floor", and that is information
# about the world, not a rendering failure.
#
# The reference lines are the BODY'S OWN GEOMETRY, not taste: 56.3 mm standing belly
# height and the 9.5 mm the crouch gate reaches (geometry §G2), and a drag band below
# them. Zero is always drawn — it is the line the whole channel exists to stay off.

const WINDOW_S := 60.0                 # how much history the width holds
const CAP := 600                       # ring capacity = WINDOW_S at the daemon's 10 Hz
const STAND_M := 0.0563                # CAD: belly 56.3 mm above the floor at spawn
const CROUCH_M := 0.0095               # CAD: what the crouch gate actually reaches
const DRAG_M := 0.005                  # below this the belly is on the ground
const VIEW_TOP_M := 0.080              # default ceiling; autoscales up, never down

var _m: Array[float] = []              # metres, oldest first
var _ok: Array[bool] = []              # was that sample a measurement the part accepted
var _busy: Array[bool] = []            # was a pose move active at that sample
var _ema := 0.0
var _worst := 0.0
var _bad_frac := 0.0
var _live := false


func _ready() -> void:
	custom_minimum_size = Vector2(0, 76)
	mouse_filter = Control.MOUSE_FILTER_STOP
	tooltip_text = ("Belly clearance from the downward VL53L0X — NOT the forward ultrasonic.\n"
		+ "Red band = belly on the ground. Lines: 56.3 mm standing, 9.5 mm crouch gate.\n"
		+ "GAPS ARE REAL: the trace breaks where the sensor rejected its own reading,\n"
		+ "marked on the track just above the activity band. Drawing those as a distance\n"
		+ "would show the belly jumping to the far limit.\n"
		+ "The lower band marks pose-move activity — read touchdowns against it.")


func push(m: float, ema: float, worst: float, bad_frac: float, valid: bool, busy: bool) -> void:
	_live = true
	_ema = ema
	_worst = worst
	_bad_frac = bad_frac
	_m.append(m)
	_ok.append(valid)
	_busy.append(busy)
	if _m.size() > CAP:
		_m = _m.slice(_m.size() - CAP)
		_ok = _ok.slice(_ok.size() - CAP)
		_busy = _busy.slice(_busy.size() - CAP)
	queue_redraw()


func _y_of(v: float, hi: float, h: float) -> float:
	if hi <= 0.0:
		return h
	return h - clamp(v / hi, 0.0, 1.0) * h


func _draw() -> void:
	var font := ThemeDB.fallback_font
	var h := size.y - 12.0                     # leave a strip for the legend
	draw_rect(Rect2(0, 0, size.x, h), Color(0.10, 0.10, 0.12))
	if not _live or _m.is_empty():
		draw_string(font, Vector2(4, h * 0.5), "belly: waiting for telemetry",
					HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(0.55, 0.55, 0.55))
		return

	# Autoscale UPWARD only. The floor and the standing line must never leave the view:
	# a scale that tracked the data would make a body pinned to the ground look like a
	# body ranging freely, because the axis would shrink to fit the dirt.
	var hi := VIEW_TOP_M
	for i in range(_m.size()):
		if _ok[i]:
			hi = max(hi, _m[i] * 1.1)

	# the drag band, behind everything: below this the chassis is grinding
	var y_drag := _y_of(DRAG_M, hi, h)
	draw_rect(Rect2(0, y_drag, size.x, h - y_drag), Color(0.32, 0.11, 0.11, 0.55))
	draw_line(Vector2(0, y_drag), Vector2(size.x, y_drag), Color(1.0, 0.35, 0.35, 0.8), 1.0)

	# the two geometry references
	var y_stand := _y_of(STAND_M, hi, h)
	draw_line(Vector2(0, y_stand), Vector2(size.x, y_stand), Color(0.5, 0.8, 1.0, 0.45), 1.0)
	draw_string(font, Vector2(2, y_stand - 2), "%.0f mm standing" % (STAND_M * 1000.0),
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.5, 0.8, 1.0, 0.8))
	var y_crouch := _y_of(CROUCH_M, hi, h)
	draw_line(Vector2(0, y_crouch), Vector2(size.x, y_crouch), Color(0.8, 0.8, 0.4, 0.35), 1.0)

	# One column per pixel, drawn as the min–max of the samples in it. MIN matters here
	# the way MAX matters on the current graph: a per-column mean would average away the
	# single sample where the belly touched, which is the only sample worth having.
	var n := _m.size()
	var cols := int(max(1.0, size.x))
	var per := float(n) / float(cols)
	var prev_y := -1.0
	for c in range(cols):
		var a0 := int(floor(c * per))
		var a1 := int(min(float(n), max(float(a0 + 1), (c + 1) * per)))
		if a0 >= n:
			break
		var mn := 0.0
		var mx := 0.0
		var any := false
		var bad := false
		var busy := false
		for k in range(a0, a1):
			busy = busy or _busy[k]
			if not _ok[k]:
				bad = true
				continue
			if not any:
				mn = _m[k]; mx = _m[k]; any = true
			else:
				mn = min(mn, _m[k]); mx = max(mx, _m[k])
		# A column with no accepted measurement breaks the trace. Carrying the previous
		# value across would draw a flat line through a hole, which reads as a steady
		# clearance the sensor never actually reported.
		if bad:
			draw_rect(Rect2(c, h - 7.0, 1.0, 3.0), Color(1.0, 0.45, 0.45, 0.9))
		if not any:
			prev_y = -1.0
		else:
			var ymn := _y_of(mn, hi, h)
			var ymx := _y_of(mx, hi, h)
			var col := Color(0.55, 0.85, 0.75)
			if mn <= DRAG_M: col = Color(1.0, 0.35, 0.35)
			elif mn <= CROUCH_M: col = Color(1.0, 0.80, 0.40)
			draw_line(Vector2(c, ymx), Vector2(c, max(ymn, ymx + 1.0)), col, 1.0)
			if prev_y >= 0.0:
				draw_line(Vector2(c - 1, prev_y), Vector2(c, ymx), col, 1.0)
			prev_y = ymn
		# the behaviour track: WHICH movement put the belly down
		if busy:
			draw_rect(Rect2(c, h - 3.0, 1.0, 3.0), Color(1.0, 0.85, 0.35, 0.9))

	# the slow metric, flat across the window it was measured over
	var y_ema := _y_of(_ema, hi, h)
	draw_line(Vector2(0, y_ema), Vector2(size.x, y_ema), Color(0.9, 0.9, 0.5, 0.5), 1.0)

	var txt := "ema30 %.1f mm   worst60 %.1f mm" % [_ema * 1000.0, _worst * 1000.0]
	var col2 := Color(0.85, 0.85, 0.85)
	# The invalid rate belongs on the legend, not the trace: it is a property of the
	# CHANNEL, not of any one sample, and a channel this graph cannot draw is the one
	# thing a reader must not have to infer from the density of the gap marks.
	if _bad_frac > 0.01:
		txt += "   invalid %.0f%%" % (_bad_frac * 100.0)
		if _bad_frac > 0.25:
			col2 = Color(1.0, 0.6, 0.4)
	draw_string(font, Vector2(2, size.y - 2), txt,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9, col2)
	draw_string(font, Vector2(size.x - 62, size.y - 2), "%.0f s" % WINDOW_S,
				HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(0.5, 0.5, 0.5))
