extends Node3D
## v6.0 — quadruped world.  Hosts the Quadruped body + Brain.  Mirrors
## the_cartpole_world.gd / the_mountain_car_world.gd shape:
##   - Logs a WORLD_READY line for run provenance.
##   - Deferred call into _wire_graph_panel() so the OgmaGraphPanel
##     (added to its own scene group by its _ready()) gets a pointer to
##     the brain — without this hop the F1/backtick graph view stays
##     blank because set_brain() is never called.

func _ready() -> void:
    var env_seed: String = OS.get_environment("OGMA_SEED")
    print(JSON.stringify({
        "event":      "WORLD_READY",
        "scene":      "the_quadruped",
        "seed":       env_seed,
        "reset_mode": OS.get_environment("OGMA_RESET_MODE"),
        "max_steps":  OS.get_environment("OGMA_QUADRUPED_MAX_STEPS"),
    }))
    call_deferred("_wire_graph_panel")

func _wire_graph_panel() -> void:
    var panel = get_tree().get_first_node_in_group("ogma_graph_panel")
    if panel == null:
        return
    var brain = get_node_or_null("Quadruped/Brain")
    if brain:
        panel.set_brain(brain)
