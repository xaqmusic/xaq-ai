extends Node3D
## Phase 6.5.3.7 — MountainCar-v0 world.  Hosts the MC body + minimal
## lighting.  MC owns its own physics + ODE; world script is intentionally
## thin.  Mirrors the_cartpole_world.gd structure.

func _ready() -> void:
	var env_seed: String = OS.get_environment("OGMA_SEED")
	print(JSON.stringify({
		"event":     "WORLD_READY",
		"scene":     "the_mountain_car",
		"seed":      env_seed,
		"max_eps":   OS.get_environment("OGMA_MC_MAX_EPISODES"),
		"max_steps": OS.get_environment("OGMA_MC_MAX_STEPS"),
	}))
	# Wire the brain into the graph panel (` / F1 toggles).
	call_deferred("_wire_graph_panel")

func _wire_graph_panel() -> void:
	var panel = get_tree().get_first_node_in_group("ogma_graph_panel")
	if panel == null:
		return
	var brain = get_node_or_null("Cart/Brain")
	if brain:
		panel.set_brain(brain)
