# Offline check of the terrain arena spec (no physics): heightmap coverage and peaks per label,
# and that label_at agrees with where the heights are.  godot4 --headless --path . -s scripts_tools/terrain_gym_check.gd
extends SceneTree
func _init() -> void:
	var G = load("res://scripts/terrain_gym.gd")
	var seed_v: int = int(OS.get_environment("OGMA_PICRAWLER_TERRAIN_SEED")) if OS.get_environment("OGMA_PICRAWLER_TERRAIN_SEED") != "" else 0
	var g = G.new(seed_v)
	g._build_heights()
	var stats: Dictionary = {}
	for j in range(0, g.nx, 2):
		for i in range(0, g.nx, 2):
			var x: float = float(i) * G.GRID - G.FLOOR_HALF
			var z: float = float(j) * G.GRID - G.FLOOR_HALF
			var lab: Array = g.label_at(x, z)
			var key: String = "%s/%d" % [lab[0], lab[1]]
			if not stats.has(key):
				stats[key] = [0, 0, 0.0]
			var h: float = g.heights[j * g.nx + i]
			stats[key][0] += 1
			if h > 0.0:
				stats[key][1] += 1
			stats[key][2] = maxf(stats[key][2], h)
	var keys: Array = stats.keys()
	keys.sort()
	for k in keys:
		print("%-12s cells %7d  raised %5.1f %%  peak %.1f mm" % [k, stats[k][0], 100.0 * stats[k][1] / stats[k][0], stats[k][2] * 1000.0])
	var t0: int = Time.get_ticks_msec()
	var holder := Node3D.new()
	g._add_floor_visuals(holder)
	print("visual mesh built in %d ms" % (Time.get_ticks_msec() - t0))
	holder.free()
	quit()
