extends Node3D
## Phase 6.5.2 — CartPole-v1 world.  Hosts the CartBody + minimal lighting.
##
## Cart owns its own physics state and ODE; the world script is intentionally
## thin so the bridge stays a pure cognition test (no perception complexity,
## no terrain, no obstacles).  Mirrors the_cell_world.gd's env-var shape so
## paired-seed harnesses can drive it the same way.

func _ready() -> void:
	# OGMA_SEED is consumed by cart_body.gd (init-state RNG).  We just log
	# it here so the run header captures provenance.
	var env_seed: String = OS.get_environment("OGMA_SEED")
	print(JSON.stringify({
		"event":     "WORLD_READY",
		"scene":     "the_cartpole",
		"seed":      env_seed,
		"max_eps":   OS.get_environment("OGMA_CARTPOLE_MAX_EPISODES"),
		"max_steps": OS.get_environment("OGMA_CARTPOLE_MAX_STEPS"),
	}))
	# Wire the brain into the graph panel — deferred so both are ready.
	# Mirrors the_cell_world.gd's wiring.
	call_deferred("_wire_graph_panel")

func _wire_graph_panel() -> void:
	var panel = get_tree().get_first_node_in_group("ogma_graph_panel")
	if panel == null:
		return
	var brain = get_node_or_null("Cart/Brain")
	if brain:
		panel.set_brain(brain)
