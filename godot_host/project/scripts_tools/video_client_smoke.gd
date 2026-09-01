# Headless smoke test for VideoClient against a live ogma_host --video:
#   godot4 --headless --path godot_host/project -s res://scripts_tools/video_client_smoke.gd [host]
# Saves both planes so the Godot path is verified by looking at the pixels, not by the
# absence of an error — the same standard the Pi-side tools are held to.
extends SceneTree

func _init() -> void:
	var host := "picrawler.local"
	var args := OS.get_cmdline_user_args()
	if args.size() > 0: host = args[0]
	var c = ClassDB.instantiate("VideoClient")
	if c == null:
		print("FAIL: VideoClient not registered — is the GDExtension rebuilt?")
		quit(1); return
	print("connect_to(", host, ", 7402): ", c.connect_to(host, 7402), "  err='", c.last_error(), "'")
	var got := 0
	var t0 := Time.get_ticks_msec()
	while got < 3 and Time.get_ticks_msec() - t0 < 8000:
		if c.poll():
			got += 1
			var info: Dictionary = c.info()
			var bi: Image = c.brain_image()
			var vi: Image = c.view_image()
			print("  frame seq=%s tick=%s  brain=%dx%d  view=%dx%d  %.1f kB" % [
				str(info.get("seq")), str(info.get("tick")),
				bi.get_width(), bi.get_height(), vi.get_width(), vi.get_height(),
				float(info.get("bytes", 0)) / 1024.0])
			if got == 3:
				bi.save_png("/tmp/godot_brain.png")
				vi.save_png("/tmp/godot_view.png")
				print("  saved /tmp/godot_brain.png and /tmp/godot_view.png")
		OS.delay_msec(40)
	print("RESULT: ", "PASS" if got >= 3 else "FAIL — no frames")
	c.disconnect_from()
	c.free()          # never parented, so nothing else will
	quit(0 if got >= 3 else 1)
