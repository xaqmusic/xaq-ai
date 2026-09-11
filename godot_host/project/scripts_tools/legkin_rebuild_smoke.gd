# Does the FK anchor cache survive a LIVE BODY SWAP?
#
# ⚠ THIS IS THE HOLE THE BYTE-IDENTITY GATE LEAVES. A 1200-tick trace diff proves the
# port reproduces the sim exactly — on a run that never rebuilds the body. The FK now
# CACHES construction anchors in C++, and _rebuild_body() (the live [B] morphology swap)
# replaces every one of them. A cache that failed to refresh would not crash: it would
# return confident poses for the PREVIOUS geometry, straight into feet_y_gravity_cmd_imu,
# which is a promoted input. picrawler_body.gd:4670 records the same failure shape biting
# the orbit camera, which "silently stopped responding" after the first swap.
#
# So: same joint angles either side of a cad <-> measured swap MUST give different FK,
# because the leg geometry genuinely changed. Identical output means a stale cache.
#
#   godot4 --headless --path godot_host/project -s res://scripts_tools/legkin_rebuild_smoke.gd
extends SceneTree

var _body: Node = null
var _n := 0

func _init() -> void:
	var scn := load("res://scenes/the_picrawler.tscn") as PackedScene
	root.add_child(scn.instantiate())
	process_frame.connect(_tick)

func _find_body(n: Node) -> Node:
	if n.has_method("_fk_leg") and n.has_method("_legkin_refresh"):
		return n
	for c in n.get_children():
		var r := _find_body(c)
		if r != null: return r
	return null

func _fk_str(b: Node) -> String:
	# A fixed, arbitrary pose — the point is only that it is the SAME pose both times.
	var out := ""
	for i in range(4):
		var t: Array = b.call("_fk_leg", i, 0.21, -0.37, 0.53)
		if t.size() < 3: return "<EMPTY — fk returned nothing>"
		out += "%.7v|%.7v " % [t[2].origin, t[0].origin]
	return out

func _tick() -> void:
	_n += 1
	if _n < 10: return
	process_frame.disconnect(_tick)
	_body = _find_body(root)
	if _body == null:
		print("FAIL: picrawler body not found"); quit(1); return

	var ok := true
	var name_before: String = str(_body.get("_geometry_name"))
	var legs_before: int = int(_body.get("_legkin").call("legs_set"))
	var fk_before := _fk_str(_body)
	print("before swap: geometry='%s'  legs_set=%d" % [name_before, legs_before])
	if legs_before != 4:
		print("FAIL: legs_set=%d before swap — _build_body() did not push anchors" % legs_before); ok = false

	var other: String = "measured" if name_before == "cad" else "cad"
	var path: String = "res://addons/ami_ogma/body/%s.json" % other
	print("swapping to %s ..." % path)
	_body.call("_rebuild_body", path)

	var name_after: String = str(_body.get("_geometry_name"))
	var legs_after: int = int(_body.get("_legkin").call("legs_set"))
	var fk_after := _fk_str(_body)
	print("after  swap: geometry='%s'  legs_set=%d" % [name_after, legs_after])

	if legs_after != 4:
		print("FAIL: legs_set=%d after swap — the cache was not repopulated" % legs_after); ok = false
	if name_after == name_before:
		print("INCONCLUSIVE: geometry name unchanged — the swap did not happen"); ok = false
	elif fk_after == fk_before:
		print("FAIL: FK IDENTICAL across a geometry swap — THE CACHE IS STALE."); ok = false
	else:
		print("FK changed across the swap, as it must — cache refreshed.")

	print("RESULT: ", "PASS" if ok else "FAIL")
	quit(0 if ok else 1)
