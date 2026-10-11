# Headless check that bench_imu_source.gd converts a REAL ogma_benchd frame into the
# shape and UNITS imu_scope.gd expects.  The conversion is the whole job of that adapter
# (g -> m/s², deg/s -> rad/s), and getting it wrong draws a plausible, wrong panel — so
# it is checked against a live frame rather than a hand-written one.
#
#   godot4 --headless --path godot_host/project -s res://scripts_tools/bench_imu_source_smoke.gd -- picrawler.local
extends SceneTree

func _init() -> void:
	var host := "picrawler.local"
	var args := OS.get_cmdline_user_args()
	if args.size() > 0: host = args[0]
	var c = ClassDB.instantiate("BenchClient")
	if not c.connect_to(host, 5590, 5591):
		print("FAIL connect: ", c.last_error()); quit(1); return
	var st: Dictionary = c.request({"verb": "status"})
	if not st.get("ok", false):
		print("FAIL status"); quit(1); return
	var frame: Dictionary = st.get("data", st)
	var src = (load("res://scripts/bench_imu_source.gd") as Script).new()
	src.call("set_frame", frame)
	if not bool(src.call("has_data")):
		print("FAIL: no usable imu block in telemetry"); quit(1); return
	var d: Dictionary = src.call("get_imu_debug")

	var raw: Dictionary = frame["imu"]
	print("--- raw benchd (sensor units) ---")
	print("  accel_body g    : ", raw["accel_body"])
	print("  gyro_body  deg/s: ", raw["gyro_body"])
	print("--- adapted (imu_scope units) ---")
	print("  accel  m/s^2 : ", d["accel"], "   |a| = %.4f g" % (d["accel"].length() / 9.81))
	print("  gyro   rad/s : ", d["gyro"], "   = %.3f deg/s about up" % rad_to_deg(d["gyro"].y))
	print("  up_fused     : ", d["up_fused"])
	print("  disagree     : %.3f deg   trust %.4f   %.0f Hz" % [d["disagree_deg"], d["trust"], d["imu_hz"]])

	var ok := true
	# |a| must land near 1 g once converted; if the g->m/s² step were missing it reads ~0.1.
	var g_mag: float = d["accel"].length() / 9.81
	if absf(g_mag - 1.0) > 0.10:
		print("FAIL |a| = %.4f g — the accel unit conversion is wrong" % g_mag); ok = false
	# The gyro must round-trip back to the reported deg/s.
	var back: float = rad_to_deg(d["gyro"].y)
	var want: float = float(raw["gyro_body"][1])
	if absf(back - want) > 0.01:
		print("FAIL gyro round-trip: %.4f vs %.4f deg/s" % [back, want]); ok = false
	if not d.has("up_fused") or not d.has("disagree_deg"):
		print("FAIL: missing a key imu_scope reads"); ok = false
	# Ground truth must be ABSENT, not zero (imu_scope tests for the key).
	if d.has("err_fused_deg"):
		print("FAIL: err_fused_deg present — the scope would draw a fake 0.0 deg error"); ok = false
	print("RESULT: ", "PASS" if ok else "FAIL")
	src.free()
	quit(0 if ok else 1)
