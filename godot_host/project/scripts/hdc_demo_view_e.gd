extends Control

# View E — "The Hebbian-warped metric"
# Six knowledge-graph nodes sit at fixed positions on a ring around the query W.
# Click a node to add a co-occurrence event (Hebbian affinity += step). Each
# node's EFFECTIVE position (the ring around it) drifts toward W along the
# warped-distance interpolation:
#     ratio = λA / (1 + λA)
#     ghost = lerp(real_pos, W, ratio)
# Real positions never move. Lookup picks the node whose GHOST is nearest to W.
# This is geometry-as-memory: experience reshapes the metric, not the map.

const N_NODES: int = 6
const QUERY_RADIUS: float = 16.0
const NODE_RADIUS: float = 22.0
const CLICK_RADIUS: float = 30.0
const GHOST_RADIUS: float = 11.0
const AFFINITY_STEP: float = 0.5
const AFFINITY_LERP_RATE: float = 6.0

const PREV_SCENE: String = "res://scenes/hdc_demo_view_d.tscn"
const NEXT_SCENE: String = ""

var lambda_strength: float = 1.0
# nodes: Array[Dictionary{pos, color, target_affinity, displayed_affinity}]
var nodes: Array = []
var query_pos: Vector2 = Vector2.ZERO

@onready var map_canvas: Control = $V/H/MapCanvas
@onready var lambda_slider: HSlider = $V/H/PanelV/LambdaRow/LambdaSlider
@onready var lambda_value: Label = $V/H/PanelV/LambdaRow/LambdaValue
@onready var reset_button: Button = $V/H/PanelV/ResetButton
@onready var scenario_button: Button = $V/H/PanelV/ScenarioButton
@onready var stats_label: Label = $V/StatsLabel
@onready var prev_button: Button = $V/Nav/PrevButton
@onready var next_button: Button = $V/Nav/NextButton


func _ready() -> void:
	lambda_slider.min_value = 0.0
	lambda_slider.max_value = 5.0
	lambda_slider.step = 0.1
	lambda_slider.value = lambda_strength

	lambda_slider.value_changed.connect(_on_lambda_changed)
	reset_button.pressed.connect(_on_reset)
	scenario_button.pressed.connect(_on_scenario)
	map_canvas.draw.connect(_draw_map)
	map_canvas.resized.connect(_on_map_resized)
	map_canvas.gui_input.connect(_on_map_input)

	_setup_nav()
	_init_nodes()
	_recompute_layout()
	_update_stats()
	set_process(true)


func _init_nodes() -> void:
	nodes.clear()
	for i in range(N_NODES):
		var hue: float = float(i) / float(N_NODES)
		nodes.append({
			"pos": Vector2.ZERO,
			"color": Color.from_hsv(hue, 0.65, 1.0),
			"target_affinity": 0.0,
			"displayed_affinity": 0.0,
		})


func _on_map_resized() -> void:
	_recompute_layout()


func _recompute_layout() -> void:
	var size: Vector2 = map_canvas.size
	if size.x < 40 or size.y < 40:
		return
	var cx: float = size.x * 0.5
	var cy: float = size.y * 0.5
	query_pos = Vector2(cx, cy)
	var radius: float = minf(size.x, size.y) * 0.36
	for i in range(N_NODES):
		var angle: float = float(i) / float(N_NODES) * TAU - PI * 0.5
		nodes[i]["pos"] = Vector2(cx + cos(angle) * radius, cy + sin(angle) * radius)
	map_canvas.queue_redraw()


func _on_lambda_changed(value: float) -> void:
	lambda_strength = value
	lambda_value.text = "%.1f" % value
	_update_stats()
	map_canvas.queue_redraw()


func _on_reset() -> void:
	for n in nodes:
		n["target_affinity"] = 0.0
	_update_stats()


func _on_scenario() -> void:
	# Pre-built narrative: node 3 has been heavily co-occurring with the
	# current consensus pattern. After this click, watch its ghost surge
	# toward W and likely become the new nearest.
	nodes[2]["target_affinity"] += 4.0
	_update_stats()


func _on_map_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event
		if mb.pressed and mb.button_index == MOUSE_BUTTON_LEFT:
			var p: Vector2 = mb.position
			for i in range(N_NODES):
				if p.distance_to(nodes[i]["pos"]) < CLICK_RADIUS:
					nodes[i]["target_affinity"] += AFFINITY_STEP
					_update_stats()
					return


func _process(delta: float) -> void:
	var any_changed: bool = false
	for n in nodes:
		var diff: float = n["target_affinity"] - n["displayed_affinity"]
		if absf(diff) > 0.001:
			n["displayed_affinity"] += diff * minf(delta * AFFINITY_LERP_RATE, 1.0)
			any_changed = true
		elif diff != 0.0:
			n["displayed_affinity"] = n["target_affinity"]
			any_changed = true
	if any_changed:
		map_canvas.queue_redraw()
		_update_stats()


func _warp_ratio(affinity: float) -> float:
	var la: float = lambda_strength * affinity
	return la / (1.0 + la)


func _ghost_pos(n: Dictionary) -> Vector2:
	return (n["pos"] as Vector2).lerp(query_pos, _warp_ratio(n["displayed_affinity"]))


func _warped_distance(n: Dictionary) -> float:
	var euclidean: float = (n["pos"] as Vector2).distance_to(query_pos)
	return euclidean / (1.0 + lambda_strength * n["displayed_affinity"])


func _nearest_node_idx() -> int:
	var best_idx: int = -1
	var best_d: float = INF
	for i in range(N_NODES):
		var d: float = _warped_distance(nodes[i])
		if d < best_d:
			best_d = d
			best_idx = i
	return best_idx


func _update_stats() -> void:
	var nearest: int = _nearest_node_idx()
	var lines: Array[String] = []
	for i in range(N_NODES):
		var n: Dictionary = nodes[i]
		var euclidean: float = (n["pos"] as Vector2).distance_to(query_pos)
		var warped: float = _warped_distance(n)
		var marker: String = "   ← nearest" if i == nearest else ""
		lines.append("node %d:   Euclidean = %5.1f   affinity A = %4.1f   d_warped = %5.1f%s" % [
			i + 1, euclidean, n["displayed_affinity"], warped, marker
		])
	stats_label.text = "\n".join(lines)


# --- Drawing ---

func _draw_map() -> void:
	var size: Vector2 = map_canvas.size
	if size.x < 80 or size.y < 80:
		return
	map_canvas.draw_rect(Rect2(0, 0, size.x, size.y), Color(0.10, 0.11, 0.14))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12
	var nearest: int = _nearest_node_idx()

	# Faint construction lines from query to each REAL node (the base map)
	for i in range(N_NODES):
		var n: Dictionary = nodes[i]
		var c: Color = n["color"]
		c.a = 0.10
		map_canvas.draw_line(query_pos, n["pos"], c, 1.0)

	# Bold lines from query to each GHOST (the effective metric)
	for i in range(N_NODES):
		var n: Dictionary = nodes[i]
		var ghost: Vector2 = _ghost_pos(n)
		var c: Color = n["color"]
		c.a = 0.40
		var thick: float = 1.8
		if i == nearest:
			c = Color(1.0, 0.78, 0.30, 0.90)
			thick = 3.0
		map_canvas.draw_line(query_pos, ghost, c, thick)

	# Real node positions (the unchanging base map)
	for i in range(N_NODES):
		var n: Dictionary = nodes[i]
		var pos: Vector2 = n["pos"]
		var fill: Color = n["color"]
		map_canvas.draw_circle(pos, NODE_RADIUS, Color(0.0, 0.0, 0.0, 0.45))
		map_canvas.draw_circle(pos, NODE_RADIUS - 2, fill)
		var idx_text: String = "%d" % (i + 1)
		var its: Vector2 = font.get_string_size(idx_text, HORIZONTAL_ALIGNMENT_CENTER, -1, 14)
		map_canvas.draw_string(font,
			Vector2(pos.x - its.x * 0.5, pos.y + its.y * 0.35),
			idx_text, HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color(0.05, 0.05, 0.08))
		# Affinity label above the node
		var aff_text: String = "A=%.1f" % n["displayed_affinity"]
		var ats: Vector2 = font.get_string_size(aff_text, HORIZONTAL_ALIGNMENT_CENTER, -1, fs)
		map_canvas.draw_string(font,
			Vector2(pos.x - ats.x * 0.5, pos.y - NODE_RADIUS - 6),
			aff_text, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.85, 0.86, 0.90))

	# Ghost positions (effective node positions under the warp)
	for i in range(N_NODES):
		var n: Dictionary = nodes[i]
		var ghost: Vector2 = _ghost_pos(n)
		if ghost.distance_to(n["pos"]) > 1.5:
			var c: Color = n["color"]
			c.a = 0.95
			map_canvas.draw_arc(ghost, GHOST_RADIUS, 0.0, TAU, 36, c, 2.5, true)
			if i == nearest:
				map_canvas.draw_arc(ghost, GHOST_RADIUS + 6.0, 0.0, TAU, 36,
					Color(1.0, 0.78, 0.30, 0.85), 2.0, true)
		elif i == nearest:
			# Highlight the nearest even when no warp is applied yet
			var c: Color = n["color"]
			c.a = 0.95
			map_canvas.draw_arc(ghost, GHOST_RADIUS, 0.0, TAU, 36, c, 2.5, true)

	# Query (W) at the centre
	map_canvas.draw_circle(query_pos, QUERY_RADIUS + 4, Color(1.0, 1.0, 1.0, 0.28))
	map_canvas.draw_circle(query_pos, QUERY_RADIUS, Color(1.0, 0.95, 0.55))
	var q_label: String = "W"
	var qls: Vector2 = font.get_string_size(q_label, HORIZONTAL_ALIGNMENT_CENTER, -1, 15)
	map_canvas.draw_string(font,
		Vector2(query_pos.x - qls.x * 0.5, query_pos.y + qls.y * 0.4),
		q_label, HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color(0.05, 0.05, 0.08))

	# Help text
	var help: String = "click any node to add a co-occurrence event  (+%.1f to its Hebbian affinity)" % AFFINITY_STEP
	map_canvas.draw_string(font,
		Vector2(12, size.y - 12),
		help, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.65, 0.67, 0.72))


# --- Slideshow nav ---

func _setup_nav() -> void:
	if PREV_SCENE == "":
		prev_button.disabled = true
		prev_button.text = "—"
	else:
		prev_button.text = "←  " + _short_name(PREV_SCENE)
		prev_button.pressed.connect(_on_prev_pressed)
	if NEXT_SCENE == "":
		next_button.disabled = true
		next_button.text = "—"
	else:
		next_button.text = _short_name(NEXT_SCENE) + "  →"
		next_button.pressed.connect(_on_next_pressed)


func _short_name(path: String) -> String:
	if path.ends_with("a.tscn"):
		return "View A"
	if path.ends_with("b.tscn"):
		return "View B"
	if path.ends_with("c.tscn"):
		return "View C"
	if path.ends_with("d.tscn"):
		return "View D"
	if path.ends_with("e.tscn"):
		return "View E"
	return path.get_file()


func _on_prev_pressed() -> void:
	if PREV_SCENE != "":
		get_tree().change_scene_to_file(PREV_SCENE)


func _on_next_pressed() -> void:
	if NEXT_SCENE != "":
		get_tree().change_scene_to_file(NEXT_SCENE)
