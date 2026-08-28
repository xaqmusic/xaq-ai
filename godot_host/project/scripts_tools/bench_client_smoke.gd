# Headless smoke test for BenchClient against a live ogma_benchd:
#   godot4 --headless --path godot_host/project -s res://scripts_tools/bench_client_smoke.gd [host]
extends SceneTree

func _init() -> void:
	var host := "10.0.0.113"
	var args := OS.get_cmdline_user_args()
	if args.size() > 0: host = args[0]
	var c = ClassDB.instantiate("BenchClient")
	print("connect_to: ", c.connect_to(host, 5590, 5591), "  err='", c.last_error(), "'")
	print("ping: ", JSON.stringify(c.request({"verb": "ping"})))
	print("status.ok: ", c.request({"verb": "status"}).get("ok"))
	var got := 0
	var last := {}
	for i in range(30):
		OS.delay_msec(100)
		var t: Dictionary = c.poll_telemetry()
		if not t.is_empty():
			got += 1; last = t
	print("telemetry frames in 3 s: ", got, "  last err='", c.last_error(), "'")
	if not last.is_empty():
		print("last: seq=", last.get("seq"), " mode=", last.get("mode"), " body=", last.get("body"), " vbat=", last.get("vbat"), " armed_ch=", last.get("armed_ch"), " servos=", (last.get("servos", []) as Array).size())
	print("bad verb: ", JSON.stringify(c.request({"verb": "nope"})))
	c.disconnect_from()
	print("disconnected, is_connected=", c.is_connected())
	c.free()
	quit()
