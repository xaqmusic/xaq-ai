extends Control
## IMU scope — a live view of the modelled accelerometer/gyro and the attitude filter
## that rides on them.  Replaces the old single god's-eye "chassis angle" readout, which
## showed a number the real robot cannot measure.
##
## DESIGNED AS A HARDWARE BRING-UP REFERENCE.  On the physical PiCrawler there is no
## ground-truth attitude to check the filter against, so the panel is split:
##
##   * Everything in the ATTITUDE ball, the |a| bar and the gyro trace is computable
##     ON-ROBOT from a real IMU.  The accel-only vs fused DISAGREEMENT is the honest
##     health signal there: when the filter is working the two markers sit close and the
##     fused one is visibly steadier.
##   * The two ground-truth error rows are SIMULATOR ONLY and are labelled as such, so it
##     is obvious which readings survive the port.
##
## Toggle with [I].  Reads the body through get_imu_debug(); draws nothing if absent, so
## it is inert on bodies that do not model an IMU.

var body: Node = null

const W: float = 250.0            # panel width
const BALL_R: float = 58.0        # attitude-ball radius (px)
const BALL_MAX_DEG: float = 30.0  # tilt mapped to the ball edge
const TRACE_N: int = 96           # gyro/history sample count
const HIST_N: int = 40            # attitude-marker trail length

var _hist_fused: Array[Vector2] = []
var _hist_accel: Array[Vector2] = []
var _trace_gyro: Array[Vector3] = []
var _font: Font = null

func _ready() -> void:
	custom_minimum_size = Vector2(W, 330)
	_font = ThemeDB.fallback_font
	set_process(true)

func _process(_dt: float) -> void:
	if body == null or not body.has_method("get_imu_debug"):
		return
	var d: Dictionary = body.get_imu_debug()
	# Project a body-frame gravity-up onto the ball: x = roll-ish, y = pitch-ish.  For a
	# near-upright body the horizontal components of "up" ARE the tilt, in radians, which
	# is exactly what we want to plot and needs no euler decomposition.
	var uf: Vector3 = d.get("up_fused", Vector3.UP)
	var ua: Vector3 = d.get("up_accel", Vector3.UP)
	_hist_fused.push_back(Vector2(uf.x, uf.z))
	_hist_accel.push_back(Vector2(ua.x, ua.z))
	while _hist_fused.size() > HIST_N: _hist_fused.pop_front()
	while _hist_accel.size() > HIST_N: _hist_accel.pop_front()
	_trace_gyro.push_back(d.get("gyro", Vector3.ZERO))
	while _trace_gyro.size() > TRACE_N: _trace_gyro.pop_front()
	queue_redraw()

func _tilt_to_px(v: Vector2) -> Vector2:
	# |horizontal component of up| ≈ sin(tilt); map degrees linearly to the ball radius.
	var deg: float = rad_to_deg(asin(clampf(v.length(), 0.0, 1.0)))
	var dir: Vector2 = v.normalized() if v.length() > 1e-6 else Vector2.ZERO
	return dir * (clampf(deg / BALL_MAX_DEG, 0.0, 1.2) * BALL_R)

func _draw() -> void:
	if body == null or not body.has_method("get_imu_debug"):
		return
	var d: Dictionary = body.get_imu_debug()
	var col_bg   := Color(0.05, 0.06, 0.09, 0.82)
	var col_grid := Color(0.32, 0.36, 0.42, 0.9)
	var col_fuse := Color(0.35, 0.95, 0.55)      # fused = the estimate we actually use
	var col_acc  := Color(0.95, 0.45, 0.35)      # accel-only = what a bare accel would give
	var col_txt  := Color(0.82, 0.86, 0.92)
	var col_dim  := Color(0.55, 0.60, 0.68)
	var col_warn := Color(0.98, 0.78, 0.25)

	draw_rect(Rect2(Vector2.ZERO, Vector2(W, 330)), col_bg, true)
	draw_string(_font, Vector2(8, 15), "IMU SCOPE   %.0f Hz" % float(d.get("imu_hz", 0.0)),
		HORIZONTAL_ALIGNMENT_LEFT, -1, 11, col_txt)

	# ---- attitude ball -------------------------------------------------------------
	var c := Vector2(W * 0.5, 92.0)
	for ring_deg in [10.0, 20.0, 30.0]:
		var r: float = (ring_deg / BALL_MAX_DEG) * BALL_R
		draw_arc(c, r, 0.0, TAU, 48, col_grid, 1.0)
		draw_string(_font, c + Vector2(r + 2, -2), "%.0f°" % ring_deg,
			HORIZONTAL_ALIGNMENT_LEFT, -1, 8, col_dim)
	draw_line(c - Vector2(BALL_R, 0), c + Vector2(BALL_R, 0), col_grid, 1.0)
	draw_line(c - Vector2(0, BALL_R), c + Vector2(0, BALL_R), col_grid, 1.0)

	# Trails first, so the current markers sit on top.  The visual point of this widget:
	# the accel-only trail sprays, the fused trail is a tight cluster.
	for i in range(_hist_accel.size()):
		var a: float = float(i) / float(max(1, HIST_N)) * 0.55
		draw_circle(c + _tilt_to_px(_hist_accel[i]), 1.5, Color(col_acc, a))
	for i in range(_hist_fused.size()):
		var a2: float = float(i) / float(max(1, HIST_N)) * 0.75
		draw_circle(c + _tilt_to_px(_hist_fused[i]), 1.5, Color(col_fuse, a2))
	if _hist_accel.size() > 0:
		draw_circle(c + _tilt_to_px(_hist_accel[-1]), 3.5, col_acc)
	if _hist_fused.size() > 0:
		var p: Vector2 = c + _tilt_to_px(_hist_fused[-1])
		draw_circle(p, 4.0, col_fuse)
		draw_line(c, p, Color(col_fuse, 0.45), 1.0)

	var y: float = 168.0
	draw_string(_font, Vector2(8, y), "● fused", HORIZONTAL_ALIGNMENT_LEFT, -1, 9, col_fuse)
	draw_string(_font, Vector2(64, y), "● accel-only", HORIZONTAL_ALIGNMENT_LEFT, -1, 9, col_acc)

	# ---- |a| bar with the trust window ---------------------------------------------
	# Why the filter ignores the accelerometer during a footfall: it is measuring the
	# impact, not gravity.  The shaded band is where the accelerometer is believed.
	y = 190.0
	var accel: Vector3 = d.get("accel", Vector3.ZERO)
	var g_mag: float = accel.length() / 9.81
	draw_string(_font, Vector2(8, y), "|a|  %.2f g" % g_mag,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 10, col_txt if absf(g_mag - 1.0) < 0.5 else col_warn)
	var bx: float = 8.0
	var bw: float = W - 16.0
	var by: float = y + 6.0
	draw_rect(Rect2(Vector2(bx, by), Vector2(bw, 9)), Color(0.12, 0.14, 0.18), true)
	# trust window = |a| within 0.5 g of gravity (IMU_ACC_GATE_FRAC); scale 0..3 g
	var s: float = bw / 3.0
	draw_rect(Rect2(Vector2(bx + 0.5 * s, by), Vector2(1.0 * s, 9)),
		Color(0.20, 0.45, 0.30, 0.75), true)
	draw_line(Vector2(bx + s, by - 2), Vector2(bx + s, by + 11), col_grid, 1.0)
	draw_rect(Rect2(Vector2(bx, by), Vector2(clampf(g_mag / 3.0, 0.0, 1.0) * bw, 9)),
		Color(col_fuse, 0.55), true)
	draw_string(_font, Vector2(bx + s - 6, by + 22), "1g", HORIZONTAL_ALIGNMENT_LEFT, -1, 8, col_dim)
	var trust: float = float(d.get("trust", 0.0))
	draw_string(_font, Vector2(bx + 30, by + 22),
		"accel trusted: %s (gain %.4f)" % ["YES" if trust > 0.0 else "no", trust],
		HORIZONTAL_ALIGNMENT_LEFT, -1, 9, col_fuse if trust > 0.0 else col_dim)

	# ---- gyro trace ----------------------------------------------------------------
	y = 232.0
	draw_string(_font, Vector2(8, y), "gyro  deg/s", HORIZONTAL_ALIGNMENT_LEFT, -1, 10, col_txt)
	var ty: float = y + 8.0
	var th: float = 44.0
	draw_rect(Rect2(Vector2(bx, ty), Vector2(bw, th)), Color(0.10, 0.12, 0.16), true)
	draw_line(Vector2(bx, ty + th * 0.5), Vector2(bx + bw, ty + th * 0.5), col_grid, 1.0)
	var cols := [Color(0.95, 0.45, 0.45), Color(0.5, 0.85, 0.5), Color(0.5, 0.65, 0.98)]
	const GYRO_FS: float = 360.0        # ±360 deg/s full scale
	for axis in range(3):
		var prev := Vector2.ZERO
		for i in range(_trace_gyro.size()):
			var val: float = rad_to_deg(_trace_gyro[i][axis])
			var px: float = bx + (float(i) / float(max(1, TRACE_N - 1))) * bw
			var py: float = ty + th * 0.5 - clampf(val / GYRO_FS, -1.0, 1.0) * th * 0.5
			var cur := Vector2(px, py)
			if i > 0:
				draw_line(prev, cur, cols[axis], 1.0)
			prev = cur
	draw_string(_font, Vector2(bx + bw - 30, ty + th + 9), "±%.0f" % GYRO_FS,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 8, col_dim)

	# ---- numeric block, split by what survives the port ----------------------------
	y = 300.0
	var dis: float = float(d.get("disagree_deg", 0.0))
	# On real hardware this is THE filter-health number: no ground truth exists, but the
	# accel-vs-fused disagreement is computable on-robot and tracks estimate quality.
	draw_string(_font, Vector2(8, y),
		"accel↔fused disagree: %5.1f°   [on-robot]" % dis,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 9, col_fuse if dis < 15.0 else col_warn)
	draw_string(_font, Vector2(8, y + 12),
		"sim truth err — fused %.1f°  accel %.1f°" % [
			float(d.get("err_fused_deg", 0.0)), float(d.get("err_accel_deg", 0.0))],
		HORIZONTAL_ALIGNMENT_LEFT, -1, 9, col_dim)
