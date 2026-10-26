# Offline check of the terrain arena's solid build (no physics stepping): how many pyramids sit in each labelled region
# and how tall they reach, the ground cells, and the build times with and without visuals.
# godot4 --headless --path . -s scripts_tools/terrain_gym_check.gd   (OGMA_PICRAWLER_TERRAIN_SEED picks the layout)
extends SceneTree
func _init() -> void:
	var G = load("res://scripts/terrain_gym.gd")
	var seed_v: int = int(OS.get_environment("OGMA_PICRAWLER_TERRAIN_SEED")) if OS.get_environment("OGMA_PICRAWLER_TERRAIN_SEED") != "" else 0
	var g = G.new(seed_v)
	var root := Node3D.new()
	get_root().add_child(root)
	var pm := PhysicsMaterial.new()
	var t0 := Time.get_ticks_msec()
	g.build(root, pm, pm, 1, 1, false)
	print("build without visuals: %d ms" % (Time.get_ticks_msec() - t0))
	var stats: Dictionary = {}
	for p in g.pyramids:
		var lab: Array = g.label_at(p[0], p[1])
		var key: String = "%s/%d" % [lab[0], lab[1]]
		if not stats.has(key):
			stats[key] = [0, 0.0]
		stats[key][0] += 1
		stats[key][1] = maxf(stats[key][1], p[4] - g.ground_h(p[0], p[1]))
	var keys: Array = stats.keys()
	keys.sort()
	for k in keys:
		print("%-12s pyramids %6d   tallest above the ground %.1f mm" % [k, stats[k][0], stats[k][1] * 1000.0])
	print("ground cells %d, bowl at 20 m %.3f m, at the rim %.3f m" % [g.n_cells, G.bowl_at(20.0), G.bowl_at(25.0)])
	root.free()
	var g2 = G.new(seed_v)
	var root2 := Node3D.new()
	get_root().add_child(root2)
	t0 = Time.get_ticks_msec()
	g2.build(root2, pm, pm, 1, 1, true)
	print("build with visuals: %d ms" % (Time.get_ticks_msec() - t0))
	quit()
