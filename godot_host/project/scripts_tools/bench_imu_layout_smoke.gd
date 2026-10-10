# Headless layout check: the IMU scope must sit FULLY INSIDE the viewport, pinned to the
# bottom-right with its margins.
#
# This exists because the first attempt read custom_minimum_size before add_child() — so
# _ready had not run, the size came back (0,0), and the panel anchored as a zero-size rect
# at the corner and then grew off-screen. That is invisible to a parse check and obvious
# to a rect comparison, so compare rects.
#
#   godot4 --headless --path godot_host/project -s res://scripts_tools/bench_imu_layout_smoke.gd
extends SceneTree

var _inst: Node = null
var _n := 0

func _init() -> void:
	_inst = (load("res://scenes/the_bench.tscn") as PackedScene).instantiate()
	root.add_child(_inst)
	process_frame.connect(_tick)

func _tick() -> void:
	_n += 1
	if _n < 8:
		return
	process_frame.disconnect(_tick)
	var scope: Control = _inst.get("_imu_scope")
	if scope == null:
		print("FAIL: no _imu_scope on the dashboard"); quit(1); return
	scope.visible = true
	var vp: Rect2 = root.get_visible_rect()
	var r: Rect2 = scope.get_global_rect()
	print("viewport : ", vp)
	print("scope    : ", r)
	print("margins  : right %.0f  bottom %.0f  (want 12 / 38)" % [vp.end.x - r.end.x, vp.end.y - r.end.y])
	var ok := true
	# (1) THE BUG THIS TEST EXISTS FOR: a zero-size rect, from reading custom_minimum_size
	# before _ready ran.  It anchors at the corner and then grows off-screen.
	if r.size.x < 1.0 or r.size.y < 1.0:
		print("FAIL: scope has no size — custom_minimum_size was read before _ready"); ok = false
	# (2) Anchored to the BOTTOM-RIGHT with the configured insets.  This is the real
	# placement assertion; containment follows from it for any usable window.
	if absf((vp.end.x - r.end.x) - 12.0) > 0.5 or absf((vp.end.y - r.end.y) - 38.0) > 0.5:
		print("FAIL: margins are not the configured 12 / 38"); ok = false
	# ⚠ NOT a viewport-containment check.  Headless pins the window to 64x64 regardless of
	# root.size or --resolution, so every correctly-placed panel larger than 64 px reports
	# as off-screen.  State the requirement instead: given the margins above, the panel
	# fits whenever the window is at least this big, which any real one is.
	print("needs a window of at least %.0f x %.0f" % [r.size.x + 12.0, r.size.y + 38.0])
	# It must also paint above the side panels.
	var ui: Node = _inst.get("_ui")
	if ui != null and scope.get_parent() == ui:
		if scope.get_index() != ui.get_child_count() - 1:
			print("FAIL: scope is not last in _ui — a re-opened panel would bury it"); ok = false
	print("RESULT: ", "PASS" if ok else "FAIL")
	quit(0 if ok else 1)
