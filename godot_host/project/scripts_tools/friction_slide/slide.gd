extends Node3D
# FRICTION-COMBINE SLIDE TEST (2026-10-26).  A 0.5 kg box launched at 2 m/s on a floor; mu_eff = deceleration / g over
# two physics ticks.  Measured on Godot Physics 3D 4.6.2: when either material is `rough` the contact takes the rough
# one's friction (the higher, if both are rough); only two smooth materials take the lower.  So a smooth body on a rough
# floor grips at the FLOOR's mu whatever its own, and a rough body grips at least at its own mu on any floor.
#   godot4 --headless --fixed-fps 60 --path . res://scripts_tools/friction_slide/slide.tscn
var cases := [["floor rough 1.5", true, 1.5, "box rough 1.5", true, 1.5],
	["floor rough 1.5", true, 1.5, "box smooth 0.3", false, 0.3],
	["floor smooth 0.3", false, 0.3, "box rough 1.5", true, 1.5],
	["floor smooth 0.3", false, 0.3, "box smooth 1.5", false, 1.5],
	["floor rough 0.3", true, 0.3, "box rough 1.5", true, 1.5],
	["floor smooth 0.2", false, 0.2, "box smooth 0.5", false, 0.5]]
var boxes := []
var t := 0
func _mat(rough: bool, mu: float) -> PhysicsMaterial:
	var m := PhysicsMaterial.new(); m.rough = rough; m.friction = mu; m.bounce = 0.0; m.absorbent = true; return m
func _ready() -> void:
	for i in cases.size():
		var c = cases[i]
		var f := StaticBody3D.new(); f.physics_material_override = _mat(c[1], c[2])
		var cs := CollisionShape3D.new(); var bs := BoxShape3D.new(); bs.size = Vector3(20, 1, 1.5); cs.shape = bs; f.add_child(cs)
		f.position = Vector3(0, -0.5, i * 3.0); add_child(f)
		var b := RigidBody3D.new(); b.mass = 0.5; b.physics_material_override = _mat(c[4], c[5])
		var cs2 := CollisionShape3D.new(); var bs2 := BoxShape3D.new(); bs2.size = Vector3(0.1, 0.05, 0.1); cs2.shape = bs2; b.add_child(cs2)
		b.position = Vector3(-5, 0.025, i * 3.0); add_child(b)
		boxes.append(b)
func _physics_process(_d: float) -> void:
	t += 1
	if t == 30:
		for b in boxes: b.linear_velocity = Vector3(2.0, 0, 0)
	if t == 31:
		for b in boxes: b.set_meta("v0", b.linear_velocity.x)
	if t == 33:
		for i in boxes.size():
			var b = boxes[i]; var v0: float = b.get_meta("v0"); var v1: float = b.linear_velocity.x
			print("%-18s on %-16s  mu_eff %.2f" % [cases[i][3], cases[i][0], (v0 - v1) / (2.0 / 60.0) / 9.8])
		get_tree().quit()
