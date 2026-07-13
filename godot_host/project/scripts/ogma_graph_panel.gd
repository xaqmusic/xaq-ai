extends Control
## OgmaGraphPanel — live full-signal-chain view of the Ogma module network.
##
## Shows three columns:
##   SOURCES (cyan)   — game engine inputs into the brain
##   BRAIN  (purple)  — Ogma Core modules (live metrics)
##   SINKS  (orange)  — game engine outputs from the brain
## Plus EVENT nodes (green/red) at the bottom.
##
## Greyed-out nodes show planned but not-yet-wired sensors/sinks.
##
## Toggle with backtick ` or F1.  (Tab was historically eaten by
## focus-traversal in some embedding scenes; backtick + F1 are bound
## here to no Godot UI action so they always reach _input.)

# ---------------------------------------------------------------------------
# Layout constants
# ---------------------------------------------------------------------------
# Sources sit on the far left, sinks on the far right; brain modules are
# laid out across LAYER_X columns by the function each type performs in
# the signal chain (encode → vote/modulate → predict/plan → act).
const SOURCE_X     := 30.0
const SINK_X       := 1670.0
const EVENT_X      := 1670.0
const LAYER_X      := {1: 350.0, 2: 690.0, 3: 1030.0, 4: 1370.0}
const ROW_H        := 110.0       # default vertical stride
const ROW_H_TIGHT  := 90.0        # stride inside a multi-row bucket (whiskers)
const GROUP_GAP    := 30.0        # extra gap between buckets in same layer
const SUB_X_OFFSET := 130.0       # horizontal offset for grid-col-2 within bucket
const NODE_W       := 240.0
const REFRESH      := 0.4         # seconds between metric updates

# Module-type → layer assignment (column index into LAYER_X).
# Unknown types default to layer 3.
const TYPE_LAYER := {
	"KeyframeAverager":        1,   # signal-prep (sensor/motor downsampler) — shares layer-1 column with EPMs but in its own "keyframe" bucket above them
	"EPM":                     1,   # input encoders
	"LateralVoter":            2,   # cross-modal consensus
	"NeurochemState":          2,   # ambient modulator next to voter
	"DescendingPredictor":     3,   # top-down predictor
	"SequenceGNG":             3,   # temporal motif extractor
	"GNGRollout":              3,   # imagined-trajectory cache
	"MotorRepertoire":         3,   # crystallised action chunks
	"HomeostaticDrive":        3,   # drive / urgency
	"HomeokineticExploration": 3,   # exploration scheduler
	"ActionDecoder":           4,   # final action policy
}

# Within a layer, buckets stack top-to-bottom in this order; any bucket
# not listed is appended afterwards in module-config order.
const LAYER_BUCKET_ORDER := {
	1: ["keyframe", "proprio", "whisker"],           # signal-prep on top, then EPMs by modality_group
	2: ["NeurochemState", "LateralVoter"],
	3: ["DescendingPredictor", "SequenceGNG", "GNGRollout",
		"MotorRepertoire", "HomeostaticDrive", "HomeokineticExploration"],
	4: ["ActionDecoder"],
}

# Colour palette (background colours for each node class)
const C_SOURCE := Color(0.15, 0.55, 0.65, 0.92)
const C_SINK   := Color(0.65, 0.40, 0.10, 0.92)
const C_EVENT_REWARD   := Color(0.15, 0.55, 0.20, 0.92)
const C_EVENT_AVERSIVE := Color(0.60, 0.18, 0.18, 0.92)
const C_INACTIVE := Color(0.30, 0.30, 0.30, 0.60)
const C_BRAIN_DEFAULT := Color(0.40, 0.22, 0.60, 0.92)

# Per-module-type colours — used for both node fill and the colour of
# outgoing edges (GraphEdit colours connection lines by the source slot).
const TYPE_COLORS := {
	"NeurochemState":      Color(0.22, 0.45, 0.90, 0.92),   # blue
	"EPM":                 Color(0.55, 0.20, 0.70, 0.92),   # purple
	"LateralVoter":        Color(0.20, 0.65, 0.60, 0.92),   # teal
	"HomeostaticDrive":    Color(0.90, 0.55, 0.10, 0.92),   # amber
	"ActionDecoder":       Color(0.85, 0.20, 0.45, 0.92),   # magenta
	"DescendingPredictor": Color(0.50, 0.55, 0.20, 0.92),   # olive
	"SequenceGNG":         Color(0.25, 0.55, 0.30, 0.92),   # forest
	"GNGRollout":          Color(0.55, 0.30, 0.20, 0.92),   # brown
	"MotorRepertoire":     Color(0.30, 0.30, 0.70, 0.92),   # indigo
	"HomeokineticExploration": Color(0.45, 0.45, 0.55, 0.92), # slate
	"KeyframeAverager":        Color(0.20, 0.40, 0.55, 0.92), # steel-blue (signal prep)
}

# Urgency thresholds for brain node border colour
const URG_AMBER := 0.5
const URG_RED   := 0.8

# Edge categories — used to filter which connection lines are drawn.
# Each edge gets a single category derived from its bus topic prefix.
const CAT_REALITY    := "Reality"      # reality.*  (sensor inputs + EPM outputs)
const CAT_CONSENSUS  := "Consensus"    # consensus.*  (voter outputs)
const CAT_PREDICTION := "Prediction"   # prediction.*  (top-down feedback)
const CAT_NEUROCHEM  := "Neurochem"    # neuro.*, hormone.*  (modulators)
const CAT_ACTION     := "Action"       # action.*, motor.*  (motor output)
const CAT_EVENTS     := "Events"       # events.*  (env hits/aversive)
const CAT_OTHER      := "Other"        # drive.*, sequence.*, rollout.*, kinesis.*, fitness.*, exploration.*

# Toggle order in the checkbox row.
const EDGE_CATEGORIES := [CAT_REALITY, CAT_CONSENSUS, CAT_PREDICTION,
						   CAT_NEUROCHEM, CAT_ACTION, CAT_EVENTS, CAT_OTHER]

# UI-dev W1 — typed channel ports.
#
# Port rendering on each brain module:
#   slot 0  = MetricsLabel row (no ports — keeps the existing per-tick metric
#             readout on its own row).
#   slot 1+ = one row per declared input topic on the LEFT side.
#   then    = one row per declared output topic on the RIGHT side.
#
# Each port's (set_slot type_id, color) comes from the topic's payload_type
# string (RealityToken / ConsensusToken / ...).  GraphEdit only allows
# connecting two ports whose `type` int matches — naturally enforces type
# correctness in Patch Mode drag-connects.
#
# `payload_type` strings come from OgmaBrain.get_module_input_specs /
# get_module_output_specs (UI-dev W1 binding) which use
# cpp_core/src/ogma/PayloadTypeName.cpp as the canonical mapping.
const PAYLOAD_TYPE_IDS := {
	"RealityToken":          1,
	"ConsensusToken":        2,
	"NeuroState":            3,
	"DriveErrors":           4,
	"ActionOut":             5,
	"FaderState":            6,
	"PolicyToken":           7,
	"PredictionToken":       8,
	"SequenceMotif":         9,
	"ExplorationDirective": 10,
	"MotorChunks":          11,
	"MotorPlayCmd":         12,
	"MotorPlayStream":      13,
	"RolloutQuery":         14,
	"RolloutResult":        15,
	"RawImageFrame":        16,
	"RawAudioFrame":        17,
	"ProprioToken":         18,
	"EnvEvent":             19,
	"ReflexGate":           20,
	"HormoneState":         21,
	"FitnessScore":         22,
	"AdaptiveThreshold":    23,
	"Unknown":               0,
}
const PAYLOAD_TYPE_COLORS := {
	"RealityToken":          Color(0.55, 0.20, 0.70),  # purple — sensory
	"ConsensusToken":        Color(0.20, 0.65, 0.60),  # teal   — fused
	"NeuroState":            Color(0.22, 0.45, 0.90),  # blue   — modulator
	"DriveErrors":           Color(0.90, 0.55, 0.10),  # amber
	"ActionOut":             Color(0.85, 0.20, 0.45),  # magenta
	"FaderState":            Color(0.95, 0.75, 0.20),  # yellow
	"PolicyToken":           Color(0.85, 0.40, 0.55),  # rose — pre-action
	"PredictionToken":       Color(0.50, 0.55, 0.20),  # olive — top-down
	"SequenceMotif":         Color(0.30, 0.55, 0.30),  # forest
	"ExplorationDirective":  Color(0.55, 0.55, 0.60),  # slate
	"MotorChunks":           Color(0.45, 0.30, 0.65),  # violet
	"MotorPlayCmd":          Color(0.55, 0.35, 0.55),  # plum
	"MotorPlayStream":       Color(0.65, 0.40, 0.55),  # rose-plum
	"RolloutQuery":          Color(0.40, 0.50, 0.60),  # steel
	"RolloutResult":         Color(0.50, 0.60, 0.65),  # sky-grey
	"RawImageFrame":         Color(0.30, 0.30, 0.55),  # navy
	"RawAudioFrame":         Color(0.55, 0.30, 0.30),  # rust
	"ProprioToken":          Color(0.15, 0.55, 0.65),  # cyan — host I/O
	"EnvEvent":              Color(0.15, 0.70, 0.30),  # green
	"ReflexGate":            Color(0.65, 0.50, 0.20),  # bronze
	"HormoneState":          Color(0.40, 0.30, 0.55),  # indigo-mid
	"FitnessScore":          Color(0.70, 0.65, 0.35),  # ochre
	"AdaptiveThreshold":     Color(0.55, 0.60, 0.55),  # sage
	"Unknown":               Color(0.50, 0.50, 0.50),
}

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
var brain: OgmaBrain = null
var graph: GraphEdit = null
var refresh_timer: float = 0.0

# node_id → GraphNode
var gnode: Dictionary = {}

# UI-dev W1 — per-module port indices.
# module_id → { "input_slots": {topic→idx}, "output_slots": {topic→idx} }
# Built in _apply_typed_slots, consumed by _apply_edge_filter to wire each
# edge to the actual source-port → sink-port pair instead of slot-0-only.
var node_ports: Dictionary = {}

# Emitted when the user clicks a typed port on a brain module's GraphNode.
# direction is "input" or "output".  Wired to the W2 inspector sidecar
# (control-server hint / focus message) once that lands.
signal port_inspected(module_id: String, topic: String, direction: String)

# All edges discovered at populate time: [{from, to, category, topic}, ...]
# Re-applied to GraphEdit whenever the category filter changes.
var edges_all: Array = []

# Per-category enable flag.  All on by default.
var enabled_cats: Dictionary = {}

# ---------------------------------------------------------------------------
# Patch Mode (Phase 6.6.A) — runtime graph editing
# ---------------------------------------------------------------------------
# When patch_mode is true, GraphEdit drag-connects/disconnects route through
# OgmaBrain.apply_patch() instead of being no-ops, the toolbar exposes
# Add/Remove/Edit-Params buttons, and right-click on empty graph opens an
# "Add Module" popup.  When false, the panel behaves exactly as the read-
# only viz it has always been.
var patch_mode: bool = false
var patch_mode_button: CheckButton = null
# Per-graph routing-mode toggle (UI-dev manual-routing feature).  When
# the brain has auto_subscribe=true (default) every implicit topic-match
# delivers; toggling off puts every module into default-deny so only
# explicit edges carry signal.  Mirrored from brain.is_auto_subscribe()
# at population time and on every visibility-change.
var auto_subscribe_button: CheckButton = null
var status_bar: Label = null
var add_node_button: Button = null
var remove_node_button: Button = null
var edit_params_button: Button = null
var add_module_popup: PopupMenu = null
var pending_add_position: Vector2 = Vector2.ZERO  # graph-space position for the next add
var selected_module_id: String = ""
# Currently-selected connection (Dictionary {from_node, from_port, to_node,
# to_port} or null).  Set by clicking on a connection line in patch mode;
# cleared by clicking elsewhere or by deleting the connection.  Used by
# the Delete-key handler in _input.
var selected_connection: Variant = null
# NOTE: hover-based connection tooltip was removed (commit ea61b25
# follow-up).  Mouse-motion would fire `_hit_test_connection` against
# graph.get_connection_list() — which can include stale entries
# referencing queue_freed-but-not-yet-deleted GraphNodes during the
# 150 ms repopulate window after a remove.  Accessing position_offset
# on a half-freed GraphNode crashed Godot's X11 backend.  Connection
# inspection is now click-only.
# Right-click context menu for an individual module.  Single shared instance
# re-targeted per click; module_id is stored as metadata on the popup so the
# id_pressed handler knows which module the action applies to.
var node_context_menu: PopupMenu = null
const NODE_CTX_EDIT_PARAMS    := 0
const NODE_CTX_REMOVE         := 1
const NODE_CTX_INSPECT_CHUNKS := 2
# After a successful patch is enqueued, schedule a re-populate so the panel
# reflects the new topology once the Scheduler has applied it (between
# ticks). 150 ms covers at least 9 ticks at 60 Hz.
var repopulate_timer: Timer = null

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	add_to_group("ogma_graph_panel")
	visible = false           # hidden until ` / F1 pressed
	# Run _input even when get_tree().paused is true.  The pause overlay
	# (ESC) sets the tree paused; under default PROCESS_MODE_INHERIT this
	# blocks every _input on this node, including the panel toggle — at
	# which point the user can't open the graph panel anymore.  ALWAYS
	# decouples panel input from pause state.
	process_mode = Node.PROCESS_MODE_ALWAYS
	for c in EDGE_CATEGORIES:
		enabled_cats[c] = true
	visibility_changed.connect(_on_visibility_changed)
	_build_ui()

# Toggle keys: backtick ` (primary) or F1 (alias).  Both are unbound
# in Godot's default action map, so neither competes with focus
# traversal (which historically ate the original Tab binding).  Per-
# scene wiring isn't needed — the panel handles its own input here so
# every scene that mounts the panel gets the toggle for free.
#
# Implemented in BOTH _input and _unhandled_input as a defence in
# depth.  _input fires first and handles the common case; if any
# upstream node (a focused dialog, a Control with a shortcut) eats
# the event before our _input gets it, _unhandled_input picks it up
# as a fallback so the toggle remains usable.
const _TOGGLE_KEYS := [KEY_QUOTELEFT, KEY_F1]

func _input(event: InputEvent) -> void:
	if _try_handle_toggle_or_delete(event):
		get_viewport().set_input_as_handled()

func _unhandled_input(event: InputEvent) -> void:
	if _try_handle_toggle_or_delete(event):
		get_viewport().set_input_as_handled()

func _try_handle_toggle_or_delete(event: InputEvent) -> bool:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return false
	if event.keycode in _TOGGLE_KEYS:
		visible = not visible
		return true
	# In patch mode, Delete removes whatever the user has selected:
	# a connection (preferred when both selected — connections are the
	# more common click target for delete-key flows) or a node.
	if patch_mode and visible and event.keycode == KEY_DELETE:
		if selected_connection != null:
			_delete_selected_connection()
			return true
		if selected_module_id != "":
			_on_remove_selected_pressed()
			return true
	return false

# Hide the on-screen HUD while the graph panel is up so it doesn't compete
# with the layered module view.  Sibling lookup so it works regardless of
# which scene mounts the UI.
func _on_visibility_changed() -> void:
	var hud: Node = get_node_or_null("../Hud")
	if hud == null:
		hud = get_tree().get_root().find_child("Hud", true, false)
	if hud:
		hud.visible = not visible

func set_brain(b: OgmaBrain) -> void:
	brain = b
	_populate_graph()

# ---------------------------------------------------------------------------
# Build the GraphEdit shell
# ---------------------------------------------------------------------------

func _build_ui() -> void:
	# Floating semi-transparent panel
	var bg := PanelContainer.new()
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(bg)

	var vbox := VBoxContainer.new()
	bg.add_child(vbox)

	var title := Label.new()
	title.text = "Ogma Signal Chain  (` or F1 to hide)"
	title.add_theme_color_override("font_color", Color.WHITE)
	vbox.add_child(title)

	# Edge-category toggle row: one checkbox per CAT_*, all on by default.
	var toggles := HBoxContainer.new()
	toggles.add_theme_constant_override("separation", 12)
	var toggle_label := Label.new()
	toggle_label.text = "Edges:"
	toggle_label.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
	toggles.add_child(toggle_label)
	for cat in EDGE_CATEGORIES:
		var cb := CheckBox.new()
		cb.text = cat
		cb.button_pressed = true
		cb.add_theme_color_override("font_color", _color_for_category(cat))
		cb.toggled.connect(_on_category_toggled.bind(cat))
		toggles.add_child(cb)
	vbox.add_child(toggles)

	# UI-dev W1 — payload-type color legend.  One swatch + label per
	# canonical payload type defined in PAYLOAD_TYPE_COLORS.  Wraps
	# horizontally so it fits at narrow window widths.
	var legend_label := Label.new()
	legend_label.text = "Port types:"
	legend_label.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
	vbox.add_child(legend_label)
	var legend := HFlowContainer.new()
	legend.add_theme_constant_override("h_separation", 10)
	legend.add_theme_constant_override("v_separation", 2)
	for ptype in PAYLOAD_TYPE_COLORS.keys():
		if ptype == "Unknown":
			continue
		var row := HBoxContainer.new()
		row.add_theme_constant_override("separation", 4)
		var swatch := ColorRect.new()
		swatch.custom_minimum_size = Vector2(10, 10)
		swatch.color = PAYLOAD_TYPE_COLORS[ptype]
		row.add_child(swatch)
		var name_lbl := Label.new()
		name_lbl.text = ptype
		name_lbl.add_theme_font_size_override("font_size", 9)
		name_lbl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
		row.add_child(name_lbl)
		legend.add_child(row)
	vbox.add_child(legend)

	# Patch Mode toolbar (Phase 6.6.A).  When the toggle is off the buttons
	# are disabled and the GraphEdit drag-connect handlers no-op exactly
	# like the legacy panel.  When on, drag-connects route through the
	# hot-patch API and the buttons become live.
	var patch_row := HBoxContainer.new()
	patch_row.add_theme_constant_override("separation", 10)
	patch_mode_button = CheckButton.new()
	patch_mode_button.text = "Patch Mode"
	patch_mode_button.add_theme_color_override("font_color", Color(0.95, 0.85, 0.55))
	patch_mode_button.toggled.connect(_on_patch_mode_toggled)
	patch_row.add_child(patch_mode_button)
	# Auto-subscribe toggle.  Active only when patch mode is on (the
	# panel itself is read-only outside patch mode, so the routing-mode
	# decision belongs to the same session-with-intent).  Mirrors the
	# brain's live auto_subscribe state on every populate.
	auto_subscribe_button = CheckButton.new()
	auto_subscribe_button.text = "Auto-subscribe"
	auto_subscribe_button.button_pressed = true
	auto_subscribe_button.disabled = true
	auto_subscribe_button.add_theme_color_override("font_color", Color(0.55, 0.85, 0.95))
	auto_subscribe_button.toggled.connect(_on_auto_subscribe_toggled)
	patch_row.add_child(auto_subscribe_button)
	add_node_button = Button.new()
	add_node_button.text = "Add Module…"
	add_node_button.disabled = true
	add_node_button.pressed.connect(_on_add_module_button_pressed)
	patch_row.add_child(add_node_button)
	remove_node_button = Button.new()
	remove_node_button.text = "Remove Selected"
	remove_node_button.disabled = true
	remove_node_button.pressed.connect(_on_remove_selected_pressed)
	patch_row.add_child(remove_node_button)
	edit_params_button = Button.new()
	edit_params_button.text = "Edit Params…"
	edit_params_button.disabled = true
	edit_params_button.pressed.connect(_on_edit_params_pressed)
	patch_row.add_child(edit_params_button)
	var save_button := Button.new()
	save_button.text = "Save Topology…"
	save_button.pressed.connect(_on_save_topology_pressed)
	patch_row.add_child(save_button)
	var load_button := Button.new()
	load_button.text = "Load Topology…"
	load_button.pressed.connect(_on_load_topology_pressed)
	patch_row.add_child(load_button)
	# UI-dev W3.4 — full brain-state snapshot.  Topology save/load above
	# rewrites the GraphConfig only; these two buttons capture and restore
	# every module's working state (GNG topology, Hebbian weights, EMAs,
	# RNGs, etc.) via OgmaInstance::snapshot_state.  Combined, the four
	# buttons let a user save a brain mid-experiment, quit, relaunch,
	# load topology + snapshot and resume bit-identically.
	var snap_save_button := Button.new()
	snap_save_button.text = "Save Snapshot…"
	snap_save_button.pressed.connect(_on_save_snapshot_pressed)
	patch_row.add_child(snap_save_button)
	var snap_load_button := Button.new()
	snap_load_button.text = "Load Snapshot…"
	snap_load_button.pressed.connect(_on_load_snapshot_pressed)
	patch_row.add_child(snap_load_button)
	status_bar = Label.new()
	status_bar.text = ""
	status_bar.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
	status_bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	patch_row.add_child(status_bar)
	vbox.add_child(patch_row)

	graph = GraphEdit.new()
	# Fill the rest of the panel's vertical space; horizontal already fills
	# via VBoxContainer.  Layered layout spans ~1900 px horizontally — wider
	# than typical windows — so GraphEdit's built-in pan/zoom covers overflow.
	graph.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	graph.size_flags_vertical   = Control.SIZE_EXPAND_FILL
	graph.right_disconnects = false
	graph.connection_request.connect(_on_connection_request)
	graph.disconnection_request.connect(_on_disconnection_request)
	# Click-to-select-connection.  GraphEdit doesn't expose connection
	# clicks natively, so we hit-test bezier paths in _gui_input.  Used
	# by the Delete-key handler in _input to disconnect.
	graph.gui_input.connect(_on_graph_gui_input)
	graph.popup_request.connect(_on_graph_popup_request)
	graph.node_selected.connect(_on_graph_node_selected)
	graph.node_deselected.connect(_on_graph_node_deselected)
	vbox.add_child(graph)

	# Add-Module popup (lazily populated when Patch Mode toggles on).
	add_module_popup = PopupMenu.new()
	add_module_popup.id_pressed.connect(_on_add_module_type_chosen)
	add_child(add_module_popup)

	# Per-node right-click context menu (Edit Params / Remove).
	node_context_menu = PopupMenu.new()
	node_context_menu.add_item("Edit Params…",        NODE_CTX_EDIT_PARAMS)
	node_context_menu.add_item("Inspect chunks…",     NODE_CTX_INSPECT_CHUNKS)
	node_context_menu.add_item("Remove",              NODE_CTX_REMOVE)
	node_context_menu.id_pressed.connect(_on_node_context_chosen)
	add_child(node_context_menu)

	# Re-populate timer used after a successful patch is enqueued.  One-shot;
	# wait_time is set per-fire so the same Timer can be reused.
	repopulate_timer = Timer.new()
	repopulate_timer.one_shot = true
	repopulate_timer.timeout.connect(_populate_graph)
	add_child(repopulate_timer)

func _on_category_toggled(pressed: bool, cat: String) -> void:
	enabled_cats[cat] = pressed
	_apply_edge_filter()

# ---------------------------------------------------------------------------
# Patch Mode handlers  (Phase 6.6.A)
# ---------------------------------------------------------------------------

func _on_patch_mode_toggled(pressed: bool) -> void:
	patch_mode = pressed
	add_node_button.disabled = not pressed
	remove_node_button.disabled = not pressed or selected_module_id == ""
	edit_params_button.disabled = not pressed or selected_module_id == ""
	auto_subscribe_button.disabled = not pressed
	# Mirror the brain's live state when entering patch mode so the
	# checkbox reads correctly even after a config that pinned manual
	# mode at boot (runtime.auto_subscribe=false).
	if pressed and brain != null and brain.has_method("is_auto_subscribe"):
		auto_subscribe_button.set_pressed_no_signal(brain.is_auto_subscribe())
	graph.right_disconnects = pressed
	if pressed:
		_populate_add_module_popup()
		_set_status("patch mode ON — drag node ports to connect; right-click empty area to add", Color(0.95, 0.85, 0.55))
	else:
		_set_status("", Color.WHITE)

# Routing-mode toggle handler.  Off → blank-canvas / aux-send mode:
# every module's input gate goes default-deny, only explicit edges
# carry signal.  On → back-compat: implicit topic-matched delivery
# resumes.  Repopulates the panel so implicit-edge tooltips classify
# correctly in the new mode.
func _on_auto_subscribe_toggled(pressed: bool) -> void:
	if brain == null or not brain.has_method("set_auto_subscribe"):
		return
	brain.set_auto_subscribe(pressed)
	if pressed:
		_set_status("auto-subscribe ON — implicit topic-matches deliver again",
			Color(0.55, 0.85, 0.95))
	else:
		_set_status("auto-subscribe OFF — only explicit edges carry signal",
			Color(0.95, 0.65, 0.35))

func _populate_add_module_popup() -> void:
	if brain == null or add_module_popup == null:
		return
	add_module_popup.clear()
	var types: PackedStringArray = brain.list_module_types()
	var sorted := Array(types)
	sorted.sort()
	for i in sorted.size():
		add_module_popup.add_item(String(sorted[i]), i)
	# Stash the sorted list so id_pressed can map index→name
	add_module_popup.set_meta("types", sorted)

func _on_graph_popup_request(at_pos: Vector2) -> void:
	if not patch_mode:
		return
	# at_pos is relative to the GraphEdit; translate to graph-space for placement.
	pending_add_position = at_pos + graph.scroll_offset
	# Show the popup at screen coordinates: GraphEdit.global_position + at_pos.
	var screen_pos: Vector2 = graph.global_position + at_pos
	add_module_popup.position = Vector2i(int(screen_pos.x), int(screen_pos.y))
	add_module_popup.popup()

func _on_add_module_button_pressed() -> void:
	if not patch_mode:
		return
	# Place new node centred in the visible viewport.
	pending_add_position = graph.scroll_offset + graph.size * 0.5
	add_module_popup.position = Vector2i(int(global_position.x + 200),
		int(global_position.y + 200))
	add_module_popup.popup()

func _on_add_module_type_chosen(idx: int) -> void:
	var meta_v: Variant = add_module_popup.get_meta("types", [])
	var types: Array = meta_v if meta_v is Array else []
	if idx < 0 or idx >= types.size():
		return
	var type_name: String = String(types[idx])
	var unique_id: String = _make_unique_id(type_name)
	# Show a small dialog so the user can rename before applying.
	_prompt_for_id(type_name, unique_id)

func _make_unique_id(type_name: String) -> String:
	var base := type_name.to_lower()
	# Truncate "EPM" → "epm", "ActionDecoder" → "actiondecoder" etc., dedupe by suffix.
	var existing: Dictionary = {}
	for m_v in brain.get_module_list():
		var m: Dictionary = m_v
		existing[String(m.get("id", ""))] = true
	var i := 1
	while true:
		var candidate := "%s_%d" % [base, i]
		if not existing.has(candidate):
			return candidate
		i += 1
	return base + "_x"  # unreachable; keeps GDScript happy

func _prompt_for_id(type_name: String, suggested_id: String) -> void:
	var dlg := AcceptDialog.new()
	dlg.title = "Add %s" % type_name
	dlg.dialog_hide_on_ok = true
	dlg.add_cancel_button("Cancel")
	var vb := VBoxContainer.new()
	var lbl := Label.new()
	lbl.text = "Module ID:"
	vb.add_child(lbl)
	var edit := LineEdit.new()
	edit.text = suggested_id
	edit.custom_minimum_size = Vector2(380, 0)
	vb.add_child(edit)
	# Params JSON entry — required for module types that have no-default
	# fields (DescendingPredictor.targets, EPM.input_topic, etc.).  Without
	# this the previous dialog always submitted params={} and the patch
	# was rejected at next tick, silently from the user's perspective
	# because the rejection logged via push_error to Godot's console only.
	var params_lbl := Label.new()
	params_lbl.text = "Params (JSON):"
	vb.add_child(params_lbl)
	var params_edit := TextEdit.new()
	params_edit.text = "{}"
	params_edit.custom_minimum_size = Vector2(380, 100)
	params_edit.placeholder_text = '{ "input_topic": "reality.proprio.imu", ... }'
	vb.add_child(params_edit)
	var hint := Label.new()
	hint.text = "(Use Load Topology to add modules with non-trivial param trees.)"
	hint.add_theme_font_size_override("font_size", 9)
	hint.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	vb.add_child(hint)
	dlg.add_child(vb)
	add_child(dlg)
	dlg.confirmed.connect(func() -> void:
		var id_text: String = edit.text.strip_edges()
		if id_text == "":
			dlg.queue_free()
			return
		var params: Dictionary = {}
		var raw: String = params_edit.text.strip_edges()
		if raw != "" and raw != "{}":
			var parsed: Variant = JSON.parse_string(raw)
			if not (parsed is Dictionary):
				_set_status("✗ params must be a JSON object",
					Color(0.95, 0.45, 0.45))
				dlg.queue_free()
				return
			params = parsed
		_apply_add_module(id_text, type_name, params)
		dlg.queue_free()
	)
	dlg.canceled.connect(func() -> void: dlg.queue_free())
	dlg.popup_centered()

func _apply_add_module(id_text: String, type_name: String,
						params: Dictionary = {}) -> void:
	var op := {
		"op": "add_node",
		"id": id_text,
		"type": type_name,
		"params": params,
	}
	_apply_and_handle(op, "add %s (%s)" % [id_text, type_name])

func _on_graph_node_selected(node: Node) -> void:
	if node == null:
		selected_module_id = ""
		return
	if node.has_meta("module_id"):
		selected_module_id = String(node.get_meta("module_id"))
	else:
		selected_module_id = ""
	if patch_mode:
		remove_node_button.disabled = selected_module_id == ""
		edit_params_button.disabled = selected_module_id == ""

func _on_graph_node_deselected(_node: Node) -> void:
	selected_module_id = ""
	if patch_mode:
		remove_node_button.disabled = true
		edit_params_button.disabled = true

func _on_remove_selected_pressed() -> void:
	if not patch_mode or selected_module_id == "":
		return
	_apply_and_handle({"op": "remove_node", "id": selected_module_id},
		"remove %s" % selected_module_id)
	selected_module_id = ""
	remove_node_button.disabled = true
	edit_params_button.disabled = true

# Right-click on a brain node → context menu (Edit Params / Remove).  Only
# active in Patch Mode; otherwise the click falls through to GraphEdit's
# default node-selection behavior.
func _on_brain_node_gui_input(event: InputEvent, module_id: String) -> void:
	if event is InputEventMouseButton \
			and event.pressed \
			and event.button_index == MOUSE_BUTTON_RIGHT:
		# Allow right-click context menu regardless of patch_mode so the
		# Inspect-chunks item is available during normal play (read-only
		# inspection doesn't need the live-edit gate).  Destructive items
		# (Edit Params, Remove) are still patch_mode-gated in the handler.
		node_context_menu.set_meta("module_id", module_id)
		# Disable destructive items when patch_mode is off so they're
		# visibly inert rather than silently no-op.
		var edit_idx: int    = node_context_menu.get_item_index(NODE_CTX_EDIT_PARAMS)
		var remove_idx: int  = node_context_menu.get_item_index(NODE_CTX_REMOVE)
		var inspect_idx: int = node_context_menu.get_item_index(NODE_CTX_INSPECT_CHUNKS)
		if edit_idx >= 0:    node_context_menu.set_item_disabled(edit_idx,   not patch_mode)
		if remove_idx >= 0:  node_context_menu.set_item_disabled(remove_idx, not patch_mode)
		# Inspect-chunks only makes sense for MotorRepertoire; check the
		# module's type from the live metrics and disable otherwise so the
		# user sees the option exists but learns it's type-gated.
		if inspect_idx >= 0:
			var metrics: Dictionary = (brain.get_module_metrics()
				if brain != null and brain.has_method("get_module_metrics")
				else {})
			var m: Dictionary = metrics.get(module_id, {})
			var is_repertoire: bool = String(m.get("type", "")) == "MotorRepertoire"
			node_context_menu.set_item_disabled(inspect_idx, not is_repertoire)
		node_context_menu.position = Vector2i(
			int(event.global_position.x),
			int(event.global_position.y))
		node_context_menu.popup()
		# Mark handled so GraphEdit doesn't also try to act on the click.
		get_viewport().set_input_as_handled()

func _on_node_context_chosen(id: int) -> void:
	var module_id_v: Variant = node_context_menu.get_meta("module_id", "")
	var module_id: String = String(module_id_v)
	if module_id == "":
		return
	match id:
		NODE_CTX_EDIT_PARAMS:
			_show_param_editor(module_id)
		NODE_CTX_INSPECT_CHUNKS:
			_show_chunks_inspector(module_id)
		NODE_CTX_REMOVE:
			_apply_and_handle({"op": "remove_node", "id": module_id},
				"remove %s" % module_id)
			if selected_module_id == module_id:
				selected_module_id = ""
				remove_node_button.disabled = true
				edit_params_button.disabled = true

func _on_connection_request(from_node: StringName, from_port: int,
		to_node: StringName, to_port: int) -> void:
	if not patch_mode:
		return
	var from_id := _module_id_for_graph_node(String(from_node))
	var to_id   := _module_id_for_graph_node(String(to_node))
	if from_id == "" or to_id == "":
		_set_status("connect: only brain-module endpoints are supported", Color(0.95, 0.55, 0.30))
		return
	# Optimistic visual edge — Scheduler validation is async.
	graph.connect_node(from_node, from_port, to_node, to_port)
	_apply_and_handle({"op": "connect", "from": from_id, "to": to_id},
		"connect %s → %s" % [from_id, to_id])

func _on_disconnection_request(from_node: StringName, from_port: int,
		to_node: StringName, to_port: int) -> void:
	if not patch_mode:
		return
	var from_id := _module_id_for_graph_node(String(from_node))
	var to_id   := _module_id_for_graph_node(String(to_node))
	if from_id == "" or to_id == "":
		_set_status("disconnect: only brain-module endpoints are supported", Color(0.95, 0.55, 0.30))
		return
	graph.disconnect_node(from_node, from_port, to_node, to_port)
	_apply_and_handle({"op": "disconnect", "from": from_id, "to": to_id},
		"disconnect %s ↛ %s" % [from_id, to_id])

func _module_id_for_graph_node(graph_node_name: String) -> String:
	# gnode maps module_id (or src_/snk_/evt_ prefix) → GraphNode whose
	# .name is auto-generated by Godot.  Walk the dict to find the match,
	# and only return brain-module IDs (no host: prefix).
	for key in gnode:
		var k: String = key
		if k.begins_with("src_") or k.begins_with("snk_") or k.begins_with("evt_"):
			continue
		var n: GraphNode = gnode[key]
		if n != null and String(n.name) == graph_node_name:
			return k
	return ""

func _on_edit_params_pressed() -> void:
	if not patch_mode or selected_module_id == "":
		return
	_show_param_editor(selected_module_id)

func _show_param_editor(module_id: String) -> void:
	# Schema-driven editor: pulls params_schema() + current values from C++,
	# renders one row per param.  HotMutable params are editable; ConstructionOnly
	# are shown read-only so the user can still see what's set.  Apply emits
	# one set_param op per param whose value was actually changed.
	var schema: Array = brain.get_module_param_schema(module_id)
	if schema.is_empty():
		_set_status("no params surfaced for %s (module unknown or schema empty)" % module_id,
			Color(0.95, 0.55, 0.30))
		return

	var dlg := AcceptDialog.new()
	dlg.title = "Edit %s params" % module_id
	dlg.dialog_hide_on_ok = true
	dlg.add_cancel_button("Cancel")
	dlg.min_size = Vector2(540, 480)

	var scroll := ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(520, 420)
	scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	var grid := GridContainer.new()
	grid.columns = 3
	grid.add_theme_constant_override("h_separation", 12)
	grid.add_theme_constant_override("v_separation", 6)
	scroll.add_child(grid)
	dlg.add_child(scroll)
	add_child(dlg)

	# Header row
	var hk := Label.new(); hk.text = "Param";        hk.modulate = Color(0.85, 0.85, 0.95); grid.add_child(hk)
	var hv := Label.new(); hv.text = "Value";        hv.modulate = Color(0.85, 0.85, 0.95); grid.add_child(hv)
	var hm := Label.new(); hm.text = "Mutability";   hm.modulate = Color(0.85, 0.85, 0.95); grid.add_child(hm)

	# field_widgets[key] -> { widget, type, original_variant }
	var field_widgets: Dictionary = {}
	for spec_v in schema:
		var spec: Dictionary = spec_v
		var key:        String = String(spec.get("key", ""))
		var type_tag:   String = String(spec.get("type", "string"))
		var mutability: String = String(spec.get("mutability", "construction_only"))
		var current:    Variant = spec.get("current_value", spec.get("default_value", ""))
		var description:String = String(spec.get("description", ""))
		var hot:        bool   = mutability == "hot_mutable"

		var name_box := VBoxContainer.new()
		var name_lbl := Label.new(); name_lbl.text = key; name_box.add_child(name_lbl)
		if description != "":
			var desc_lbl := Label.new()
			desc_lbl.text = description
			desc_lbl.add_theme_font_size_override("font_size", 10)
			desc_lbl.modulate = Color(0.65, 0.65, 0.65)
			desc_lbl.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			desc_lbl.custom_minimum_size = Vector2(180, 0)
			name_box.add_child(desc_lbl)
		grid.add_child(name_box)

		var widget: Control = _make_param_widget(type_tag, current, not hot)
		grid.add_child(widget)

		var mut_lbl := Label.new()
		mut_lbl.text = "hot" if hot else "const"
		mut_lbl.modulate = Color(0.55, 0.85, 0.55) if hot else Color(0.65, 0.65, 0.65)
		grid.add_child(mut_lbl)

		field_widgets[key] = {
			"widget":   widget,
			"type":     type_tag,
			"original": current,
			"hot":      hot,
		}

	dlg.confirmed.connect(func() -> void:
		_apply_param_edits(module_id, field_widgets)
		dlg.queue_free()
	)
	dlg.canceled.connect(func() -> void: dlg.queue_free())
	dlg.popup_centered()

# Build the right widget for a given param type, prepopulated with the
# current value.  Returns a Control whose value can be read back via
# `_read_widget_value`.
func _make_param_widget(type_tag: String, current: Variant, read_only: bool) -> Control:
	match type_tag:
		"bool":
			var cb := CheckBox.new()
			cb.button_pressed = bool(current) if current != null else false
			cb.disabled = read_only
			return cb
		"int":
			var sb := SpinBox.new()
			sb.step = 1
			sb.min_value = -1e9
			sb.max_value = 1e9
			sb.value = float(int(current)) if current != null else 0.0
			sb.editable = not read_only
			sb.custom_minimum_size = Vector2(150, 0)
			return sb
		"float":
			var sb := SpinBox.new()
			sb.step = 0.001
			sb.min_value = -1e9
			sb.max_value = 1e9
			sb.value = float(current) if current != null else 0.0
			sb.editable = not read_only
			sb.custom_minimum_size = Vector2(150, 0)
			return sb
		"list_float":
			var le := LineEdit.new()
			var parts: PackedStringArray = []
			if current is PackedFloat64Array:
				for v in current as PackedFloat64Array:
					parts.append(str(v))
			elif current is Array:
				for v in current as Array:
					parts.append(str(v))
			le.text = ", ".join(parts)
			le.editable = not read_only
			le.custom_minimum_size = Vector2(220, 0)
			le.placeholder_text = "comma-separated floats"
			return le
		"list_string":
			var le := LineEdit.new()
			var parts: PackedStringArray = []
			if current is PackedStringArray:
				for v in current as PackedStringArray:
					parts.append(String(v))
			elif current is Array:
				for v in current as Array:
					parts.append(String(v))
			le.text = ", ".join(parts)
			le.editable = not read_only
			le.custom_minimum_size = Vector2(220, 0)
			le.placeholder_text = "comma-separated strings"
			return le
		_:
			var le := LineEdit.new()
			le.text = String(current) if current != null else ""
			le.editable = not read_only
			le.custom_minimum_size = Vector2(220, 0)
			return le

func _read_widget_value(widget: Control, type_tag: String) -> Variant:
	match type_tag:
		"bool":
			return (widget as CheckBox).button_pressed
		"int":
			return int((widget as SpinBox).value)
		"float":
			return float((widget as SpinBox).value)
		"list_float":
			var raw: String = (widget as LineEdit).text
			var out := PackedFloat64Array()
			for piece in raw.split(",", false):
				var s: String = piece.strip_edges()
				if s != "":
					out.append(float(s))
			return out
		"list_string":
			var raw_s: String = (widget as LineEdit).text
			var out_s := PackedStringArray()
			for piece in raw_s.split(",", false):
				var s: String = piece.strip_edges()
				if s != "":
					out_s.append(s)
			return out_s
		_:
			return (widget as LineEdit).text

func _apply_param_edits(module_id: String, field_widgets: Dictionary) -> void:
	var changed: int = 0
	var failed: Array = []
	for key in field_widgets:
		var info: Dictionary = field_widgets[key]
		if not bool(info.get("hot", false)):
			continue  # construction-only — skip silently
		var new_val: Variant = _read_widget_value(info["widget"], String(info["type"]))
		var orig:    Variant = info["original"]
		# Compare via stringification — Variant equality across PackedArrays is
		# fiddly; "are the rendered text fields the same" is good enough for
		# detecting user edits.
		if str(new_val) == str(orig):
			continue
		var result: Dictionary = brain.apply_patch({
			"op":    "set_param",
			"id":    module_id,
			"key":   String(key),
			"value": new_val,
		})
		if bool(result.get("success", false)):
			changed += 1
		else:
			failed.append("%s: %s" % [key, String(result.get("error", "?"))])
	if changed > 0:
		_set_status("✓ updated %d param(s) on %s%s" % [
			changed, module_id,
			(" — failed: " + ", ".join(failed)) if not failed.is_empty() else ""],
			Color(0.55, 0.85, 0.55) if failed.is_empty() else Color(0.95, 0.55, 0.30))
		repopulate_timer.start(0.15)
	elif failed.is_empty():
		_set_status("no changes to apply on %s" % module_id, Color(0.85, 0.85, 0.85))
	else:
		_set_status("✗ %s — %s" % [module_id, ", ".join(failed)], Color(0.95, 0.45, 0.45))

# ---------------------------------------------------------------------------
# Chunks inspector — per-chunk read-only view with intent-distribution
# histogram.  Diagnoses chunk content quality: a forward-favouring chunk
# has many intent=2 bars; a rotation-heavy chunk dominates 0/4.
#
# Sorted by replay_hits desc so the most-evidence chunks lead.  No mutation
# from this dialog — pair with the "C" hotkey in body_controller to probe a
# specific chunk by id.
# ---------------------------------------------------------------------------
func _show_chunks_inspector(module_id: String) -> void:
	if brain == null or not brain.has_method("get_module_metrics"):
		_set_status("chunks inspector unavailable — no metrics access",
			Color(0.95, 0.55, 0.30))
		return
	var metrics: Dictionary = brain.get_module_metrics()
	var m: Dictionary = metrics.get(module_id, {})
	if String(m.get("type", "")) != "MotorRepertoire":
		_set_status("inspect chunks: %s is not a MotorRepertoire" % module_id,
			Color(0.95, 0.55, 0.30))
		return
	var entries: Array = m.get("chunks", [])
	if entries.is_empty():
		_set_status("%s: chunk library is empty" % module_id,
			Color(0.85, 0.85, 0.85))
		return

	# Sort by Beta-prior score (proxy via replay_hits relative to misses)
	# so the highest-confidence chunks lead.  Score formula matches
	# ActionDecoder.chunk_score:  (h+1) / (h+m+2)  with h = hits_during
	# + replay_hits.  Lifecycle/freshness decay is already baked into
	# the diag values.
	var sortable: Array = []
	for c in entries:
		var rh: float = float(c.get("replay_hits", 0))
		var rm: float = float(c.get("replay_misses", 0))
		var hd: int   = int(c.get("hits_during", 0))
		var h_total: float = float(hd) + rh
		var n_total: float = h_total + rm
		var score: float = (h_total + 1.0) / (n_total + 2.0)
		sortable.append({
			"id":           int(c.get("id", 0)),
			"len":          int(c.get("length", c.get("intent_length", 0))),
			"hits_during":  hd,
			"replay_hits":  rh,
			"replay_misses":rm,
			"score":        score,
			"intent_seq":   c.get("intent_seq", []),
			"trigger_motif":int(c.get("trigger_motif", -1)),
		})
	sortable.sort_custom(func (a: Dictionary, b: Dictionary) -> bool:
		return a["score"] > b["score"])

	var dlg := AcceptDialog.new()
	dlg.title = "Chunks Inspector — %s (%d chunks)" % [module_id, entries.size()]
	dlg.dialog_hide_on_ok = true
	var scroll := ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(740, 540)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 4)
	scroll.add_child(vb)

	var header := Label.new()
	header.text = "    id  | len | h  | rh   | rm   | score  | trig | intent histogram      | top intent (% of seq)"
	header.add_theme_font_size_override("font_size", 11)
	header.add_theme_color_override("font_color", Color(0.85, 0.85, 0.95))
	vb.add_child(header)
	var rule := HSeparator.new()
	vb.add_child(rule)

	for c in sortable:
		var row := _build_chunk_row(c)
		vb.add_child(row)

	# Hint footer.
	var footer := Label.new()
	footer.text = ("Press C in-game to probe a chunk by id.  "
				   + "Premotor intent table: 0=L+4 R-4  1=L+2 R0  "
				   + "2=L+4 R+4  3=L0 R+2  4=L-4 R+4.")
	footer.add_theme_font_size_override("font_size", 10)
	footer.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	footer.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	vb.add_child(footer)

	dlg.add_child(scroll)
	add_child(dlg)
	dlg.popup_centered(Vector2(800, 620))

func _build_chunk_row(c: Dictionary) -> Control:
	# Renders one chunk as: header (id/len/hits/score) + intent histogram
	# (5 bars rendered as Unicode block characters scaled to count).
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 6)

	var seq: Array = c.get("intent_seq", [])
	var counts: Array[int] = [0, 0, 0, 0, 0]
	for v in seq:
		var idx: int = int(v)
		if idx >= 0 and idx < 5:
			counts[idx] += 1
	var max_count: int = 0
	for k in counts:
		if k > max_count: max_count = k
	# Unicode block characters from lowest to highest density.
	var BLOCKS := ["·", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"]
	var hist_chars: Array[String] = []
	for k in counts:
		var ratio: float = (float(k) / float(max_count)) if max_count > 0 else 0.0
		var bi: int = int(round(ratio * (BLOCKS.size() - 1)))
		hist_chars.append(BLOCKS[bi])
	var hist_str: String = "[%s]" % "".join(hist_chars)
	# Top intent percentage.
	var top_idx: int = 0
	var top_count: int = counts[0]
	for i in range(1, 5):
		if counts[i] > top_count:
			top_count = counts[i]
			top_idx = i
	var top_pct: float = (float(top_count) / float(seq.size()) * 100.0
						   if not seq.is_empty() else 0.0)

	var lbl := Label.new()
	lbl.text = "  %4d  | %3d | %2d | %4.1f | %4.1f | %.3f  | %4d | %s  | int=%d (%4.1f%%)" % [
		int(c["id"]),
		int(c["len"]),
		int(c["hits_during"]),
		float(c["replay_hits"]),
		float(c["replay_misses"]),
		float(c["score"]),
		int(c["trigger_motif"]),
		hist_str,
		top_idx,
		top_pct,
	]
	# Monospace font for column alignment + colour-code score.
	lbl.add_theme_font_size_override("font_size", 11)
	var score: float = float(c["score"])
	if   score > 0.75: lbl.add_theme_color_override("font_color", Color(0.55, 0.95, 0.55))
	elif score > 0.55: lbl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.55))
	elif score > 0.40: lbl.add_theme_color_override("font_color", Color(0.85, 0.65, 0.45))
	else:              lbl.add_theme_color_override("font_color", Color(0.85, 0.45, 0.45))
	row.add_child(lbl)
	return row

func _apply_and_handle(op: Dictionary, label: String) -> void:
	if brain == null:
		return
	var result: Dictionary = brain.apply_patch(op)
	if bool(result.get("success", false)):
		_set_status("✓ %s (batch %d)" % [label, int(result.get("batch_id", 0))],
			Color(0.55, 0.85, 0.55))
		# Repopulate after a brief delay so the panel reflects the
		# applied topology once the Scheduler processes the patch.
		repopulate_timer.start(0.15)
	else:
		_set_status("✗ %s — %s" % [label, String(result.get("error", "unknown"))],
			Color(0.95, 0.45, 0.45))

func _set_status(text: String, color: Color) -> void:
	if status_bar == null:
		return
	status_bar.text = text
	status_bar.add_theme_color_override("font_color", color)

# ---------------------------------------------------------------------------
# Topology save / load  (Phase 6.6.A.3)
# ---------------------------------------------------------------------------
# Save: serialise the live module list and brain-internal edges as a JSON
# blob compatible with cpp_core's GraphConfig schema.  Per-module params
# are NOT round-tripped (the C++ binding does not yet expose them); on
# load the modules are recreated with their type defaults and the user
# re-applies any tuning via Edit Params.

func _on_save_topology_pressed() -> void:
	if brain == null:
		_set_status("save: brain not ready", Color(0.95, 0.45, 0.45))
		return
	var dlg := FileDialog.new()
	dlg.access = FileDialog.ACCESS_FILESYSTEM
	dlg.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	dlg.add_filter("*.json", "Topology JSON")
	dlg.current_file = "topology.json"
	add_child(dlg)
	dlg.file_selected.connect(func(path: String) -> void:
		_save_topology_to(path)
		dlg.queue_free()
	)
	dlg.canceled.connect(func() -> void: dlg.queue_free())
	dlg.popup_centered_ratio(0.6)

func _save_topology_to(path: String) -> void:
	var modules: Array = []
	# Phase 6.6.A.3 fix: pull real params from the live brain's config so
	# save→load preserves what each module needs to start up.  Empty params
	# (the prior behavior) made on_setup throw on most module types,
	# crashing the load.
	for m_v in brain.get_module_specs():
		var m: Dictionary = m_v
		modules.append({
			"id":     String(m.get("id", "")),
			"type":   String(m.get("type", "")),
			"params": m.get("params", {}),
		})
	# get_graph_edges() returns the JSON config edges followed by implicit
	# topology-derived ones; dedupe (from,to,topic) so we don't emit doubles.
	var edges: Array = []
	var seen: Dictionary = {}
	for e_v in brain.get_graph_edges():
		var e: Dictionary = e_v
		var fn := String(e.get("from", ""))
		var tn := String(e.get("to", ""))
		# Skip host:* edges — those are bridged by the body controller, not
		# the saved topology.  Re-emitting them would duplicate the body's
		# wiring.  Keep brain-internal edges only.
		if fn.begins_with("host:") or tn.begins_with("host:"):
			continue
		var topic := String(e.get("topic", ""))
		var key := fn + "→" + tn + "|" + topic
		if seen.has(key):
			continue
		seen[key] = true
		var edge_dict: Dictionary = {"from": fn, "to": tn}
		if topic != "":
			edge_dict["topic"] = topic
		if bool(e.get("feedback", false)):
			edge_dict["feedback"] = true
		edges.append(edge_dict)
	var doc: Dictionary = {
		"version": 1,
		"modules": modules,
		"edges":   edges,
	}
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		_set_status("save failed: cannot open %s" % path, Color(0.95, 0.45, 0.45))
		return
	f.store_string(JSON.stringify(doc, "  "))
	f.close()
	_set_status("✓ saved %d modules, %d edges → %s" % [modules.size(), edges.size(), path],
		Color(0.55, 0.85, 0.55))

func _on_load_topology_pressed() -> void:
	if brain == null:
		_set_status("load: brain not ready", Color(0.95, 0.45, 0.45))
		return
	var dlg := FileDialog.new()
	dlg.access = FileDialog.ACCESS_FILESYSTEM
	dlg.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	dlg.add_filter("*.json", "Topology JSON")
	add_child(dlg)
	dlg.file_selected.connect(func(path: String) -> void:
		_load_topology_from(path)
		dlg.queue_free()
	)
	dlg.canceled.connect(func() -> void: dlg.queue_free())
	dlg.popup_centered_ratio(0.6)

func _load_topology_from(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		_set_status("load failed: cannot open %s" % path, Color(0.95, 0.45, 0.45))
		return
	var text: String = f.get_as_text()
	f.close()
	var parsed: Variant = JSON.parse_string(text)
	if not (parsed is Dictionary):
		_set_status("load failed: not a JSON object", Color(0.95, 0.45, 0.45))
		return
	var doc: Dictionary = parsed
	var modules_v: Variant = doc.get("modules", [])
	var edges_v: Variant   = doc.get("edges",   [])
	if not (modules_v is Array) or not (edges_v is Array):
		_set_status("load failed: missing modules/edges arrays", Color(0.95, 0.45, 0.45))
		return
	# Build a single transactional batch: remove every current module, then
	# add the loaded modules, then connect their edges.  Order within a
	# batch is significant — Scheduler applies ops in sequence.
	var ops: Array = []
	for m_v in brain.get_module_list():
		var m: Dictionary = m_v
		ops.append({"op": "remove_node", "id": String(m.get("id", ""))})
	for m_v in modules_v:
		var m: Dictionary = m_v
		var add_op: Dictionary = {
			"op":   "add_node",
			"id":   String(m.get("id", "")),
			"type": String(m.get("type", "")),
		}
		var pv: Variant = m.get("params", {})
		if pv is Dictionary:
			add_op["params"] = pv
		else:
			add_op["params"] = {}
		ops.append(add_op)
	for e_v in edges_v:
		var e: Dictionary = e_v
		var c_op: Dictionary = {
			"op":   "connect",
			"from": String(e.get("from", "")),
			"to":   String(e.get("to", "")),
		}
		var topic := String(e.get("topic", ""))
		if topic != "":
			c_op["topic"] = topic
		if bool(e.get("feedback", false)):
			c_op["feedback"] = true
		ops.append(c_op)
	var batch: Dictionary = {"ops": ops, "source": "ui:load_topology"}
	var result: Dictionary = brain.apply_patch(batch)
	if bool(result.get("success", false)):
		_set_status("✓ loaded %s (%d ops, batch %d)" % [
			path, ops.size(), int(result.get("batch_id", 0))],
			Color(0.55, 0.85, 0.55))
		repopulate_timer.start(0.20)
	else:
		_set_status("✗ load failed — %s" % String(result.get("error", "unknown")),
			Color(0.95, 0.45, 0.45))

# ---------------------------------------------------------------------------
# UI-dev W3.4 — full brain-state snapshot save/load.
# ---------------------------------------------------------------------------
#
# Save / Load Topology (above) handles only the GraphConfig — the static
# wiring + params each module is constructed from.  The buttons below
# capture and restore EVERY module's working state (GNG topology, Hebbian
# matrices, EMAs, RNG state, eligibility traces, ...) via OgmaInstance::
# snapshot_state which returns a JSON blob the C++ side already proves
# byte-equivalent across disk round-trip (cpp_core test_snapshot_disk_*).
#
# Combined: save topology + save snapshot → quit → relaunch → load
# topology + load snapshot resumes the brain bit-identically.

func _on_save_snapshot_pressed() -> void:
	if brain == null or not brain.is_brain_ready():
		_set_status("save snapshot: brain not ready", Color(0.95, 0.45, 0.45))
		return
	var dlg := FileDialog.new()
	dlg.access = FileDialog.ACCESS_FILESYSTEM
	dlg.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	dlg.add_filter("*.json", "Brain Snapshot JSON")
	# Default filename includes a timestamp so users don't accidentally
	# clobber a recent snapshot when saving multiple in a session.
	var ts := Time.get_datetime_string_from_system().replace(":", "").replace("-", "")
	dlg.current_file = "snapshot_%s.json" % ts
	add_child(dlg)
	dlg.file_selected.connect(func(path: String) -> void:
		_save_snapshot_to(path)
		dlg.queue_free()
	)
	dlg.canceled.connect(func() -> void: dlg.queue_free())
	dlg.popup_centered_ratio(0.6)

func _save_snapshot_to(path: String) -> void:
	var blob: String = brain.snapshot_state()
	if blob == "":
		_set_status("save snapshot: empty payload", Color(0.95, 0.45, 0.45))
		return
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		_set_status("save snapshot failed: cannot open %s" % path,
			Color(0.95, 0.45, 0.45))
		return
	f.store_string(blob)
	f.close()
	# Approximate size for the status line — gives the user immediate
	# feedback that the snapshot is non-trivial.
	_set_status("✓ snapshot saved (%d KB) → %s" %
		[blob.length() / 1024, path], Color(0.55, 0.85, 0.55))

func _on_load_snapshot_pressed() -> void:
	if brain == null or not brain.is_brain_ready():
		_set_status("load snapshot: brain not ready", Color(0.95, 0.45, 0.45))
		return
	var dlg := FileDialog.new()
	dlg.access = FileDialog.ACCESS_FILESYSTEM
	dlg.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	dlg.add_filter("*.json", "Brain Snapshot JSON")
	add_child(dlg)
	dlg.file_selected.connect(func(path: String) -> void:
		_load_snapshot_from(path)
		dlg.queue_free()
	)
	dlg.canceled.connect(func() -> void: dlg.queue_free())
	dlg.popup_centered_ratio(0.6)

func _load_snapshot_from(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		_set_status("load snapshot failed: cannot open %s" % path,
			Color(0.95, 0.45, 0.45))
		return
	var text: String = f.get_as_text()
	f.close()
	if text == "":
		_set_status("load snapshot: empty file", Color(0.95, 0.45, 0.45))
		return
	# OgmaInstance.restore_state throws on schema mismatch (version != 1) and
	# the GDExtension surfaces that as a runtime error; wrap in a label
	# update so the user gets feedback before the error reaches the console.
	brain.restore_state(text)
	_set_status("✓ snapshot loaded (%d KB) ← %s" %
		[text.length() / 1024, path], Color(0.55, 0.85, 0.55))
	# Re-render so per-module metric panels read the restored state on
	# their next refresh tick.
	repopulate_timer.start(0.20)

# ---------------------------------------------------------------------------
# Populate from brain topology (called once after brain is set)
# ---------------------------------------------------------------------------

func _populate_graph() -> void:
	if brain == null:
		return
	gnode.clear()
	node_ports.clear()
	# Selection state references GraphNodes about to be freed.  Clearing it
	# here prevents stale lookups on the next mouse / key event.
	selected_module_id = ""
	selected_connection = null
	# Defensively dismiss any tooltip / popup that might be in flight.  The
	# per-port labels carry tooltip_text (W1 typed-port info), and Godot 4's
	# X11 backend has been observed to log a BadWindow error when a Control
	# bearing a visible tooltip gets queue_freed mid-frame — the tooltip
	# Window outlives the Control by a frame and X11 GetProperty fires on
	# the freed surface.  Clearing every descendant's tooltip_text + telling
	# the viewport to release any focus / hovered Control sidesteps the race.
	get_viewport().gui_release_focus()
	for child in graph.get_children():
		if child is GraphNode:
			_clear_descendant_tooltips(child)
			child.queue_free()
	graph.clear_connections()

	# 1. Source / sink / event nodes (left + right columns)
	var reg: Array = brain.get_sensor_registry()
	var src_idx: int = 0
	var snk_idx: int = 0
	var evt_idx: int = 0
	for entry_v in reg:
		var entry: Dictionary = entry_v
		var kind: String = String(entry.get("kind", ""))
		var active: bool = bool(entry.get("active", true))
		var entry_name: String = String(entry.get("name", ""))
		var entry_topic: String = String(entry.get("topic", ""))
		var entry_desc: String  = String(entry.get("description", ""))
		match kind:
			"source":
				var n := _make_node(entry_name,
									C_SOURCE if active else C_INACTIVE,
									Vector2(SOURCE_X, 30.0 + src_idx * ROW_H))
				_add_label(n, entry_desc, 10)
				_add_label(n, "→ " + entry_topic, 10)
				graph.add_child(n)
				gnode["src_" + entry_name] = n
				src_idx += 1
			"sink":
				var n := _make_node(entry_name,
									C_SINK if active else C_INACTIVE,
									Vector2(SINK_X, 30.0 + snk_idx * ROW_H))
				_add_label(n, "← " + entry_topic, 10)
				_add_label(n, entry_desc, 10)
				graph.add_child(n)
				gnode["snk_" + entry_name] = n
				snk_idx += 1
			"event":
				var ev_type: String = String(entry.get("event_type", ""))
				var col: Color = C_EVENT_REWARD if ev_type == "reward" else C_EVENT_AVERSIVE
				# Events stack below the sinks on the right side.
				var n := _make_node(entry_name + " (" + ev_type + ")",
									col if active else C_INACTIVE,
									Vector2(EVENT_X, 480.0 + evt_idx * 70.0))
				_add_label(n, entry_topic, 10)
				graph.add_child(n)
				gnode["evt_" + entry_name] = n
				evt_idx += 1

	# 2. Brain module nodes — bucketed by layer, then by sub-group.
	var mods: Array = brain.get_module_list()
	var positions: Dictionary = _compute_module_positions(mods)
	for m_v in mods:
		var m: Dictionary = m_v
		var m_id: String   = String(m.get("id", ""))
		var m_type: String = String(m.get("type", ""))
		var brain_col: Color = TYPE_COLORS.get(m_type, C_BRAIN_DEFAULT)
		var pos: Vector2 = positions.get(m_id, Vector2(LAYER_X[3], 30.0))
		var n := _make_node(m_id + "\n" + m_type, brain_col, pos)
		n.set_meta("module_id",   m_id)
		n.set_meta("module_type", m_type)
		n.set_meta("base_color",  brain_col)
		var metrics_label := Label.new()
		metrics_label.name = "MetricsLabel"
		metrics_label.text = "…"
		metrics_label.add_theme_font_size_override("font_size", 10)
		metrics_label.add_theme_color_override("font_color", Color(0.9, 0.9, 0.9))
		n.add_child(metrics_label)
		# UI-dev W1 — replace _make_node's single-slot fallback with
		# per-topic typed ports built from the live module's
		# input_topics / output_topics declarations.
		_apply_typed_slots(n, m_id)
		# Right-click context menu (Patch Mode only).  Bind module_id via
		# Callable.bind so the handler knows which node was clicked.
		n.gui_input.connect(_on_brain_node_gui_input.bind(m_id))
		graph.add_child(n)
		gnode[m_id] = n

	# 3. Collect edges (no connect_node yet — we apply via the category filter).
	edges_all.clear()
	_collect_source_edges(reg, mods)
	_collect_brain_edges()
	_apply_edge_filter()

# Source/sink/event boundary edges: not in get_graph_edges() because
# sensors and event publishers aren't Modules.  We synthesize them from
# topic naming convention + known subscriber types.
func _collect_source_edges(reg: Array, mods: Array) -> void:
	for entry_v in reg:
		var entry: Dictionary = entry_v
		var kind: String = String(entry.get("kind", ""))
		if not bool(entry.get("active", true)):
			continue
		var entry_name: String = String(entry.get("name", ""))
		var entry_topic: String = String(entry.get("topic", ""))
		match kind:
			"source":
				var src_key: String = "src_" + entry_name
				if not gnode.has(src_key):
					continue
				# Draw an edge from the sensor source to a brain module
				# only when the module actually subscribes to that exact
				# topic.  Replaces the prior string-contains heuristic
				# which drew spurious edges from every "proprio" source
				# to every EPM regardless of the EPM's actual input_topic.
				for m_v in mods:
					var m: Dictionary  = m_v
					var m_id: String   = String(m.get("id", ""))
					if not gnode.has(m_id):
						continue
					if _module_subscribes_to_topic(m_id, entry_topic):
						_add_edge(src_key, m_id, entry_topic)
			"event":
				# Event sources publish events.<name> from the body.  Known
				# subscribers (events.* prefix): NeurochemState, MotorRepertoire
				# and HomeokineticExploration (post-Phase-6.1).  Surfacing
				# these makes hit registration visible in the graph.
				var ev_key: String = "evt_" + entry_name
				if not gnode.has(ev_key):
					continue
				for m_v in mods:
					var m: Dictionary  = m_v
					var m_id: String   = String(m.get("id", ""))
					var m_type: String = String(m.get("type", ""))
					if m_type in ["NeurochemState", "MotorRepertoire",
								   "HomeokineticExploration"]:
						if gnode.has(m_id):
							_add_edge(ev_key, m_id, entry_topic)
	# Module → Sink boundary edges (Phase 6.6.D.7+ generalization).
	# Every registered sink has a topic; any brain module that publishes to
	# that exact topic gets an edge to the sink.  Replaces the prior hard-
	# coded "ActionDecoder → Motor" rule, which left ActionGate, the new
	# bilateral Motor (left)/(right) sinks, and any future motor publisher
	# (WhiskerSteerReflex, MotorMix, etc.) drawn-but-disconnected.
	for entry_v in reg:
		var entry: Dictionary = entry_v
		if String(entry.get("kind", "")) != "sink":
			continue
		var sink_name: String  = String(entry.get("name", ""))
		var sink_topic: String = String(entry.get("topic", ""))
		var sink_key: String   = "snk_" + sink_name
		if sink_topic == "" or not gnode.has(sink_key):
			continue
		for m_v in mods:
			var m: Dictionary = m_v
			var m_id: String  = String(m.get("id", ""))
			if not gnode.has(m_id):
				continue
			if _module_publishes_topic(m_id, String(m.get("type", "")), sink_topic):
				_add_edge(m_id, sink_key, sink_topic)

# Phase 6.6.F — live per-instance lookup via OgmaBrain.get_module_output_topics.
# Replaces the prior hardcoded type→topics map which couldn't see per-instance
# `output_topic` params (so e.g. WhiskerSteerReflex configured for single-channel
# was still drawn wired to action.left/right).  When the binding isn't
# available yet (older .so), we fall back to the original conservative map so
# the panel stays usable across binary versions.
func _module_publishes_topic(module_id: String, module_type: String, topic: String) -> bool:
	if brain != null and brain.has_method("get_module_output_topics"):
		var live: PackedStringArray = brain.get_module_output_topics(module_id)
		if live.size() > 0:
			return topic in Array(live)
	# Legacy fallback (pre-6.6.F .so).
	match module_type:
		"ActionDecoder":
			return topic == "action.out"
		"ActionGate":
			return topic in ["action.out", "action.left", "action.right"]
		"WhiskerSteerReflex":
			return topic == "action.left" or topic == "action.right"
	return false

# Live per-instance lookup via OgmaBrain.get_module_input_specs.  Returns
# true when `module_id` actually subscribes to `topic` (exact match, or
# the module declares a prefix subscription that `topic` falls under).
# Used by _collect_source_edges to avoid spurious sensor → module edges
# from the prior string-contains heuristic.
func _module_subscribes_to_topic(module_id: String, topic: String) -> bool:
	if brain == null or not brain.has_method("get_module_input_specs"):
		return false
	var specs: Array = brain.get_module_input_specs(module_id)
	for s_v in specs:
		var s: Dictionary = s_v
		var sname: String = String(s.get("name", ""))
		if sname == "":
			continue
		if sname == topic:
			return true
		# Prefix subscription (e.g. voter "reality." matches "reality.X").
		if sname.ends_with(".") and topic.begins_with(sname):
			return true
	return false

# Brain-internal edges from the OgmaInstance config.
#
# UI-dev W1 — dedupe by (from, to, topic) instead of (from, to).  Each
# topic gets its own port pair on each module, so emitting one edge per
# topic surfaces multi-topic relationships (e.g. an ActionDecoder
# subscribing to BOTH consensus.0 and drive.errors from upstream chains)
# that the legacy collapse-to-one-edge model hid.
func _collect_brain_edges() -> void:
	var edges: Array = brain.get_graph_edges()
	var seen: Dictionary = {}
	for e_v in edges:
		var e: Dictionary = e_v
		var fn: String = String(e.get("from", ""))
		var tn: String = String(e.get("to", ""))
		if not (gnode.has(fn) and gnode.has(tn)):
			continue
		var topic := String(e.get("topic", ""))
		var key := fn + "→" + tn + "::" + topic
		if seen.has(key):
			continue
		seen[key] = true
		# Carry is_implicit through so the edge-hover tooltip can classify.
		# get_graph_edges emits explicit (boot config + scheduler current
		# edges) BEFORE implicit topic-derived edges, so for any (from, to,
		# topic) tuple the explicit version wins the dedup race.
		_add_edge(fn, tn, topic, bool(e.get("is_implicit", false)))

func _add_edge(from_id: String, to_id: String, topic: String,
				is_implicit: bool = true) -> void:
	# is_implicit defaults to true for boundary edges (sensor / event /
	# sink synthesised from sensor_registry — they have no scheduler-side
	# entry).  Brain-internal edges pass the brain's actual classification
	# from get_graph_edges.is_implicit.
	edges_all.append({
		"from":        from_id,
		"to":          to_id,
		"topic":       topic,
		"category":    _classify_topic(topic),
		"is_implicit": is_implicit,
	})

# Topic-prefix → category.  Ordered most-specific-first to avoid a generic
# `reality.` swallowing a more specific match.
func _classify_topic(topic: String) -> String:
	if topic.begins_with("prediction."): return CAT_PREDICTION
	if topic.begins_with("consensus."):  return CAT_CONSENSUS
	if topic.begins_with("neuro.") or topic.begins_with("hormone."): return CAT_NEUROCHEM
	if topic.begins_with("action.") or topic.begins_with("motor."):  return CAT_ACTION
	if topic.begins_with("events."):     return CAT_EVENTS
	if topic.begins_with("reality."):    return CAT_REALITY
	return CAT_OTHER

# Priority for choosing which topic represents a deduped (from→to) pair.
# Higher = wins.  The action/prediction signals are the most informative
# to surface visually; reality is everywhere so deprioritise it.
func _category_priority(cat: String) -> int:
	match cat:
		CAT_EVENTS:     return 7  # rare and informative — surface when present
		CAT_ACTION:     return 6
		CAT_PREDICTION: return 5
		CAT_CONSENSUS:  return 4
		CAT_NEUROCHEM:  return 3
		CAT_OTHER:      return 2
		CAT_REALITY:    return 1
	return 0

# Per-category text/edge tint — used both on the toggle labels and (in
# future) to colour edges if Godot exposes per-connection colour.
func _color_for_category(cat: String) -> Color:
	match cat:
		CAT_REALITY:    return Color(0.55, 0.20, 0.70)   # purple (matches EPM)
		CAT_CONSENSUS:  return Color(0.20, 0.65, 0.60)   # teal   (matches voter)
		CAT_PREDICTION: return Color(0.50, 0.55, 0.20)   # olive  (matches descend)
		CAT_NEUROCHEM:  return Color(0.22, 0.45, 0.90)   # blue   (matches neuro)
		CAT_ACTION:     return Color(0.85, 0.20, 0.45)   # magenta (matches action)
		CAT_EVENTS:     return Color(0.15, 0.70, 0.30)   # green  (matches reward event)
		CAT_OTHER:      return Color(0.75, 0.75, 0.75)   # neutral
	return Color.WHITE

func _apply_edge_filter() -> void:
	if graph == null:
		return
	graph.clear_connections()
	for e_v in edges_all:
		var e: Dictionary = e_v
		var cat: String = String(e.get("category", CAT_OTHER))
		if not bool(enabled_cats.get(cat, true)):
			continue
		var fn: String = String(e.get("from", ""))
		var tn: String = String(e.get("to", ""))
		if not (gnode.has(fn) and gnode.has(tn)):
			continue
		# UI-dev W1 — route to the actual typed port slot when both ends are
		# brain modules with declared topics; fall back to slot 0 (the legacy
		# both-side-port row on synthetic source/sink/event nodes).
		var topic: String = String(e.get("topic", ""))
		var from_slot: int = _resolve_port_slot(fn, topic, "output")
		var to_slot:   int = _resolve_port_slot(tn, topic, "input")
		graph.connect_node(gnode[fn].name, from_slot, gnode[tn].name, to_slot)

# Look up a topic's port slot on a node.  Returns 0 (legacy whole-node port
# on source/sink/event nodes) when the lookup fails — keeps boundary edges
# rendering exactly as they did pre-W1.  For brain modules, prefix-pattern
# inputs (trailing dot like "reality.") match any topic that starts with
# the prefix.
func _resolve_port_slot(node_id: String, topic: String, direction: String) -> int:
	if not node_ports.has(node_id):
		return 0
	var ports: Dictionary = node_ports[node_id]
	var key := "input_slots" if direction == "input" else "output_slots"
	var slots: Dictionary = ports.get(key, {})
	if slots.has(topic):
		return int(slots[topic])
	# Prefix subscriptions: trailing-dot keys catch any matching topic.
	for k_v in slots.keys():
		var k: String = k_v
		if not k.is_empty() and k.ends_with(".") and topic.begins_with(k):
			return int(slots[k])
	return 0

# ---------------------------------------------------------------------------
# Runtime metric updates
# ---------------------------------------------------------------------------

func _process(delta: float) -> void:
	if not visible or brain == null or not brain.is_brain_ready():
		return
	refresh_timer += delta
	if refresh_timer < REFRESH:
		return
	refresh_timer = 0.0
	_update_metrics()

func _update_metrics() -> void:
	var metrics: Dictionary = brain.get_module_metrics()
	# Pre-extract the currently-playing chunk id (if any) from any
	# ActionDecoder so MotorRepertoire's display can surface it.
	# Phase 6.5.13 — repertoire node turns red and shows the chunk id
	# being replayed so a human watching the graph can see the chunk
	# pipeline actively firing.
	var firing_chunk_id: int = -1
	for mid in metrics:
		var mt: String = str(metrics[mid].get("type", ""))
		if mt == "ActionDecoder":
			var cid: int = int(metrics[mid].get("active_chunk_id", -1))
			var rem: int = int(metrics[mid].get("chunk_remaining", 0))
			if cid > 0 and rem > 0:
				firing_chunk_id = cid
				break
	for mod_id in metrics:
		if not gnode.has(mod_id):
			continue
		var n: GraphNode = gnode[mod_id]
		var m: Dictionary = metrics[mod_id]
		var lbl := n.get_node_or_null("MetricsLabel") as Label
		if lbl == null:
			continue
		lbl.text = _format_metrics(m, firing_chunk_id)
		# Update border colour by urgency / health
		_colour_brain_node(n, m, firing_chunk_id)

func _format_metrics(m: Dictionary, firing_chunk_id: int = -1) -> String:
	var t: String = m.get("type", "")
	match t:
		"NeurochemState":
			return "da=%.3f  ht=%.3f\neps_b=%.2f" % [
				float(m.get("dopamine", 0)), float(m.get("serotonin", 0)),
				float(m.get("epsilon_b_scale", 1))]
		"EPM":
			return "nodes=%d baked=%d\ntle=%.4f%s" % [
				int(m.get("node_count", 0)), int(m.get("baked_count", 0)),
				float(m.get("tle", 0)),
				" ★" if bool(m.get("is_novel", false)) else ""]
		"KeyframeAverager":
			# Rolling-mean downsampler.  fill/window indicates buffer
			# occupancy; mean shows the current published value (1-D
			# common case = scalar like averaged motor accel).
			var fill: int   = int(m.get("window_fill", 0))
			var win:  int   = int(m.get("window_size", 0))
			var dim:  int   = int(m.get("payload_dim", 0))
			var mean_arr = m.get("last_mean", PackedFloat64Array())
			var mean_s := ""
			if mean_arr.size() == 1:
				mean_s = "mean=%+.3f" % float(mean_arr[0])
			elif mean_arr.size() > 1:
				mean_s = "mean[%d]=%+.2f…" % [mean_arr.size(), float(mean_arr[0])]
			return "fill=%d/%d  dim=%d\n%s\nin=%d  out=%d" % [
				fill, win, dim, mean_s,
				int(m.get("total_inputs_seen", 0)),
				int(m.get("total_publishes", 0))]
		"LateralVoter":
			return "tle=%.4f  %s" % [
				float(m.get("fused_tle", 0)), str(m.get("active_modality", ""))]
		"HomeostaticDrive":
			var errs: Dictionary = m.get("errors", {})
			var s := "urg=%.3f\n" % float(m.get("urgency", 0))
			for k in errs:
				s += "%s=%.3f  " % [k, float(errs[k])]
			return s.strip_edges()
		"ActionDecoder":
			return "accel=%.2f%s\nval=%d heb=%d" % [
				float(m.get("accel", 0)),
				" [probe]" if bool(m.get("probe", false)) else "",
				int(m.get("val_sz", 0)), int(m.get("heb_sz", 0))]
		"DescendingPredictor":
			return "confidence=%.3f" % float(m.get("confidence", 0))
		"SequenceGNG":
			return "nodes=%d  baked=%d\nmotif=%d" % [
				int(m.get("node_count", 0)), int(m.get("baked_count", 0)),
				int(m.get("current_motif", -1))]
		"GNGRollout":
			return "sources=%d" % int(m.get("known_sources", 0))
		"MotorRepertoire":
			# Phase 6.5.13 — show "FIRING #N" inline when ActionDecoder
			# is currently replaying chunk N.  The titlebar also turns
			# red (see _colour_brain_node).
			var line := "chunks=%d  active=%d\ndisp=%d  failed=%d" % [
				int(m.get("chunk_count", 0)),
				int(m.get("active_chunk_count", 0)),
				int(m.get("total_dispatch_count", 0)),
				int(m.get("failed_dispatch_count", 0)),
			]
			if firing_chunk_id > 0:
				line += "\n▶ FIRING #%d" % firing_chunk_id
			return line
	return ""

func _colour_brain_node(n: GraphNode, m: Dictionary, firing_chunk_id: int = -1) -> void:
	var urg := float(m.get("urgency", -1.0))
	# Only HomeostaticDrive has urgency; for others use tle as proxy
	var stress := urg if urg >= 0.0 else float(m.get("tle", 0.0)) * 2.0
	var base: Color = n.get_meta("base_color", C_BRAIN_DEFAULT)
	var col: Color = base
	if stress > URG_RED:
		col = Color(0.70, 0.15, 0.15, 0.92)
	elif stress > URG_AMBER:
		col = base.lerp(Color(0.85, 0.55, 0.10, 0.92), 0.6)
	# Phase 6.5.13 — MotorRepertoire titlebar goes red when a chunk is
	# currently replaying, so a human watching the graph can see chunk
	# dispatches firing in real time.
	if firing_chunk_id > 0 and str(m.get("type", "")) == "MotorRepertoire":
		col = Color(0.70, 0.15, 0.15, 0.92)
	var sb := StyleBoxFlat.new()
	sb.bg_color = col
	sb.corner_radius_top_left = 6
	sb.corner_radius_top_right = 6
	n.add_theme_stylebox_override("titlebar", sb)
	n.add_theme_stylebox_override("titlebar_selected", sb)

# ---------------------------------------------------------------------------
# Helper: create a styled GraphNode
# ---------------------------------------------------------------------------

func _make_node(title: String, bg_color: Color, pos: Vector2) -> GraphNode:
	var n := GraphNode.new()
	n.title = title
	n.position_offset = pos
	# Width fixed for column alignment; height shrinks to fit content.
	n.custom_minimum_size = Vector2(NODE_W, 0)

	var sb_title := StyleBoxFlat.new()
	sb_title.bg_color = bg_color
	sb_title.corner_radius_top_left = 6
	sb_title.corner_radius_top_right = 6
	n.add_theme_stylebox_override("titlebar", sb_title)

	# Selected variant: brighter titlebar + bright cyan accent border so
	# the user has unambiguous visual feedback on selection.  Without
	# this, GraphEdit's default selection visual was being clobbered by
	# titlebar_selected = titlebar above (the previous code used the
	# same stylebox for both states, so selection had no effect).
	var sb_title_sel := StyleBoxFlat.new()
	sb_title_sel.bg_color = Color(
		min(1.0, bg_color.r + 0.18),
		min(1.0, bg_color.g + 0.18),
		min(1.0, bg_color.b + 0.18),
		bg_color.a)
	sb_title_sel.corner_radius_top_left  = 6
	sb_title_sel.corner_radius_top_right = 6
	sb_title_sel.border_color = Color(0.55, 0.95, 1.0, 1.0)  # bright cyan
	sb_title_sel.border_width_top    = 2
	sb_title_sel.border_width_left   = 2
	sb_title_sel.border_width_right  = 2
	sb_title_sel.border_width_bottom = 0
	n.add_theme_stylebox_override("titlebar_selected", sb_title_sel)

	var sb_body := StyleBoxFlat.new()
	sb_body.bg_color = Color(bg_color.r * 0.5, bg_color.g * 0.5, bg_color.b * 0.5, 0.88)
	sb_body.corner_radius_bottom_left = 6
	sb_body.corner_radius_bottom_right = 6
	n.add_theme_stylebox_override("panel", sb_body)

	var sb_body_sel := StyleBoxFlat.new()
	sb_body_sel.bg_color = Color(
		bg_color.r * 0.55, bg_color.g * 0.55, bg_color.b * 0.55, 0.92)
	sb_body_sel.corner_radius_bottom_left  = 6
	sb_body_sel.corner_radius_bottom_right = 6
	sb_body_sel.border_color = Color(0.55, 0.95, 1.0, 1.0)
	sb_body_sel.border_width_top    = 0
	sb_body_sel.border_width_left   = 2
	sb_body_sel.border_width_right  = 2
	sb_body_sel.border_width_bottom = 2
	n.add_theme_stylebox_override("panel_selected", sb_body_sel)

	# Slots: input (left) + output (right). The output-slot colour is what
	# GraphEdit uses to tint the connection line leaving this node.
	n.set_slot(0, true, 0, bg_color, true, 0, bg_color)
	return n

# ---------------------------------------------------------------------------
# Bucketed layer layout
# ---------------------------------------------------------------------------
#
# Each module gets (layer, bucket).  Layer chooses the X column; bucket
# stacks vertically within the layer.  EPMs sub-bucket by modality_group
# (parsed from id, since the C++ binding doesn't surface params).  All
# other types use type as bucket.  Whisker buckets get a 2-col grid so
# six EPMs don't make a 660-px tall column.
#
func _compute_module_positions(mods: Array) -> Dictionary:
	# layer -> (bucket -> Array of module dicts), preserving config order.
	var layer_buckets: Dictionary = {}
	for m_v in mods:
		var m: Dictionary = m_v
		var m_type: String = String(m.get("type", ""))
		var layer: int = int(TYPE_LAYER.get(m_type, 3))
		var bucket: String = _bucket_for(m)
		if not layer_buckets.has(layer):
			layer_buckets[layer] = {}
		var buckets: Dictionary = layer_buckets[layer]
		if not buckets.has(bucket):
			buckets[bucket] = []
		buckets[bucket].append(m)

	var positions: Dictionary = {}
	var layers_sorted: Array = layer_buckets.keys()
	layers_sorted.sort()
	for layer in layers_sorted:
		var x: float = float(LAYER_X.get(layer, LAYER_X[3]))
		var buckets: Dictionary = layer_buckets[layer]
		var ordered: Array = (LAYER_BUCKET_ORDER.get(layer, []) as Array).duplicate()
		# Append unknown buckets in arrival order.
		for b in buckets.keys():
			if not ordered.has(b):
				ordered.append(b)
		var y: float = 30.0
		for b in ordered:
			if not buckets.has(b):
				continue
			var items: Array = buckets[b]
			# Whisker (or any 4+ same-bucket items) uses a 2-col tight grid.
			var grid_cols: int = 2 if items.size() >= 4 else 1
			var stride: float = ROW_H_TIGHT if grid_cols == 2 else ROW_H
			for i in items.size():
				var col: int = i % grid_cols
				var row: int = i / grid_cols
				var item: Dictionary = items[i]
				positions[String(item.get("id", ""))] = Vector2(
					x + col * SUB_X_OFFSET, y + row * stride)
			var rows_used: int = int(ceil(float(items.size()) / float(grid_cols)))
			y += rows_used * stride + GROUP_GAP
	return positions

# Bucket key: EPMs by modality_group (parsed from id), others by type.
# Convention from the_cell.json: epm_whisker_* → whisker, otherwise proprio.
func _bucket_for(m: Dictionary) -> String:
	var m_type: String = String(m.get("type", ""))
	if m_type == "KeyframeAverager":
		return "keyframe"
	if m_type != "EPM":
		return m_type
	var m_id: String = String(m.get("id", ""))
	if m_id.begins_with("epm_whisker"):
		return "whisker"
	return "proprio"

func _add_label(node: GraphNode, text: String, font_size: int) -> void:
	var lbl := Label.new()
	lbl.text = text
	lbl.add_theme_font_size_override("font_size", font_size)
	lbl.add_theme_color_override("font_color", Color(0.9, 0.9, 0.9))
	lbl.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	node.add_child(lbl)

# ---------------------------------------------------------------------------
# UI-dev W1 — typed channel ports on each brain module.
# ---------------------------------------------------------------------------
#
# Reads the live module's input_topics() / output_topics() via the new
# OgmaBrain bindings and renders one Label child per declared topic, with
# the slot on the appropriate side colour-coded by payload type.  Slot 0
# (the existing MetricsLabel) gets no ports so the metric readout doesn't
# accept stray drag-connects.
#
# Falls back gracefully on older .so binaries that lack the bindings —
# leaves the legacy single both-side slot 0 in place.

func _apply_typed_slots(n: GraphNode, m_id: String) -> void:
	if brain == null:
		return
	if not (brain.has_method("get_module_input_specs")
			and brain.has_method("get_module_output_specs")):
		return  # legacy binary — keep _make_node's both-side slot 0
	var inputs: Array  = brain.get_module_input_specs(m_id)
	var outputs: Array = brain.get_module_output_specs(m_id)
	# Slot 0 = MetricsLabel; no ports, no color.
	n.set_slot(0, false, 0, Color.TRANSPARENT, false, 0, Color.TRANSPARENT)
	# Godot's GraphEdit.connect_node(from, from_port, to, to_port) indexes
	# ports per-side (counting only slots with a port enabled on that
	# side), NOT by slot row.  We track separate input / output port-index
	# counters so the values stored here and passed to connect_node line up
	# with what Godot actually expects.  Storing slot row indices instead
	# (the obvious thing to try) makes outputs render from the node's
	# top-left origin because the "port index" passed to connect_node
	# exceeds the real output port count and Godot silently falls back.
	var input_slots: Dictionary = {}
	var output_slots: Dictionary = {}
	var in_port_idx: int  = 0
	var out_port_idx: int = 0
	for spec_v in inputs:
		var spec: Dictionary = spec_v
		var topic: String       = String(spec.get("name", ""))
		var ptype: String       = String(spec.get("payload_type", "Unknown"))
		var kind:  String       = String(spec.get("kind", "direct"))
		var required: bool      = bool(spec.get("required", true))
		var col: Color   = PAYLOAD_TYPE_COLORS.get(ptype, PAYLOAD_TYPE_COLORS["Unknown"])
		var type_id: int = int(PAYLOAD_TYPE_IDS.get(ptype, 0))
		var idx := n.get_child_count()
		# Both input and output ports use a right-pointing arrow so
		# data-flow direction reads consistently across the panel:
		# inputs  → "→ topic"   (left-aligned; arrow → sits next to
		#                        the port on the left, points into the module)
		# outputs → "topic →"   (right-aligned; arrow → sits next to
		#                        the port on the right, points out of it)
		# Feedback subscriptions get a "(fb)" annotation in the label so
		# their direction symbol stays consistent — the kind is also in
		# the tooltip.
		var fb_tag := "  (fb)" if kind != "direct" else ""
		var lbl := _make_port_label("→ %s%s" % [topic, fb_tag], ptype, col,
									 m_id, topic, "input")
		lbl.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
		n.add_child(lbl)
		# `required` rendered as a thin ring around the port via a slightly
		# darker outline colour; non-required ports get a desaturated tone.
		var port_col: Color = col if required else Color(col.r, col.g, col.b, 0.55)
		n.set_slot(idx, true, type_id, port_col, false, 0, Color.TRANSPARENT)
		input_slots[topic] = in_port_idx
		in_port_idx += 1
	for spec_v in outputs:
		var spec: Dictionary = spec_v
		var topic: String = String(spec.get("name", ""))
		var ptype: String = String(spec.get("payload_type", "Unknown"))
		var col: Color   = PAYLOAD_TYPE_COLORS.get(ptype, PAYLOAD_TYPE_COLORS["Unknown"])
		var type_id: int = int(PAYLOAD_TYPE_IDS.get(ptype, 0))
		var idx := n.get_child_count()
		var lbl := _make_port_label("%s →" % topic, ptype, col,
									 m_id, topic, "output")
		# Right-align so the topic name + arrow sit flush against the
		# output connector on the right side of the slot row.
		lbl.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		n.add_child(lbl)
		n.set_slot(idx, false, 0, Color.TRANSPARENT, true, type_id, col)
		output_slots[topic] = out_port_idx
		out_port_idx += 1
	node_ports[m_id] = {
		"input_slots":  input_slots,
		"output_slots": output_slots,
	}

func _make_port_label(text: String, ptype: String, col: Color,
					   module_id: String, topic: String,
					   direction: String) -> Label:
	var lbl := Label.new()
	lbl.text = text
	lbl.add_theme_font_size_override("font_size", 9)
	# Slight tint of the payload color into the row label so even on a
	# black-and-white screenshot the row category is legible.
	lbl.add_theme_color_override("font_color",
		Color(col.r * 0.6 + 0.4, col.g * 0.6 + 0.4, col.b * 0.6 + 0.4))
	lbl.tooltip_text = "%s\npayload: %s\ndirection: %s" % [topic, ptype, direction]
	lbl.mouse_filter = Control.MOUSE_FILTER_PASS
	lbl.gui_input.connect(_on_port_label_input.bind(module_id, topic, direction))
	return lbl

func _on_port_label_input(event: InputEvent, module_id: String,
						   topic: String, direction: String) -> void:
	if event is InputEventMouseButton and event.pressed \
			and event.button_index == MOUSE_BUTTON_LEFT:
		emit_signal("port_inspected", module_id, topic, direction)


# ---------------------------------------------------------------------------
# Connection click-selection (patch-mode ablation flow)
# ---------------------------------------------------------------------------
#
# GraphEdit lacks a native "connection clicked" signal, so we listen on
# graph.gui_input and run a bezier hit-test against every connection in
# graph.get_connection_list().  Threshold-based — if the click is within
# ~12 px (graph-space) of any connection's bezier, that connection
# becomes selected_connection.  Status bar surfaces the selection
# (GraphEdit can't visually highlight individual connections).  Delete
# key in _input() then routes through apply_patch with a disconnect op.

const _CONN_HIT_TOLERANCE_PX := 12.0
const _CONN_BEZIER_SAMPLES   := 32

func _on_graph_gui_input(event: InputEvent) -> void:
	if not patch_mode:
		return
	if not (event is InputEventMouseButton and event.pressed
			and event.button_index == MOUSE_BUTTON_LEFT):
		return
	# Module windows occlude connection clicks: if the click landed inside
	# any GraphNode's rect (in graph-space), the click belongs to the node,
	# not to a connection that visually passes under it.  Without this
	# guard, clicking on a module while a connection runs behind it would
	# select the connection — and worse, mouse-motion processing during
	# the 150 ms post-remove repopulate window could hit-test against
	# half-freed GraphNodes and crash the X11 backend.
	var zoom: float = max(0.001, graph.zoom)
	var graph_pos: Vector2 = (event.position / zoom) + graph.scroll_offset / zoom
	for m_id in gnode:
		var n: GraphNode = gnode[m_id]
		if n == null:
			continue
		if Rect2(n.position_offset, n.size).has_point(graph_pos):
			return  # click belongs to the module; let GraphEdit handle it
	var hit: Variant = _hit_test_connection(event.position)
	if hit != null:
		selected_connection = hit
		_refresh_status_for_selection()
	elif selected_connection != null:
		selected_connection = null
		_refresh_status_for_selection()

# Re-render the status bar with whichever connection is selected.
# Connection inspection is click-only since mouse-motion hover was
# removed (see _on_graph_gui_input note).  When nothing is selected,
# restore the patch-mode banner.
func _refresh_status_for_selection() -> void:
	if selected_connection != null:
		var info: String = _describe_connection(
			selected_connection,
			"  —  Delete to remove")
		_set_status("selected: %s" % info, Color(0.95, 0.85, 0.55))
		return
	if patch_mode:
		_set_status(
			"patch mode ON — drag node ports to connect; right-click empty area to add",
			Color(0.95, 0.85, 0.55))
	else:
		_set_status("", Color.WHITE)

func _describe_connection(conn: Dictionary, suffix: String) -> String:
	var from_n: GraphNode = graph.get_node_or_null(
		NodePath(String(conn.get("from_node", "")))) as GraphNode
	var to_n: GraphNode = graph.get_node_or_null(
		NodePath(String(conn.get("to_node", "")))) as GraphNode
	var from_id := ""
	var to_id := ""
	if from_n != null and from_n.has_meta("module_id"):
		from_id = String(from_n.get_meta("module_id"))
	if to_n != null and to_n.has_meta("module_id"):
		to_id = String(to_n.get_meta("module_id"))
	var topic: String = _topic_for_connection(conn)
	var topic_str := topic if topic != "" else "?"
	# Lookup the edge in edges_all to classify implicit vs explicit.
	# Match on (from, to, topic) — the same tuple _collect_brain_edges
	# dedupes on.  When unknown (boundary edges, etc.) say so.
	var classification := "?"
	if from_id != "" and to_id != "":
		for e_v in edges_all:
			var e: Dictionary = e_v
			if String(e.get("from", "")) != from_id: continue
			if String(e.get("to",   "")) != to_id:   continue
			var et := String(e.get("topic", ""))
			if topic != "" and et != topic: continue
			classification = "implicit" if bool(e.get("is_implicit", false)) else "explicit"
			break
	return "%s → %s  (topic: %s)  —  %s%s" % [
		from_id if from_id != "" else String(conn.get("from_node", "")),
		to_id   if to_id   != "" else String(conn.get("to_node",   "")),
		topic_str, classification, suffix,
	]

func _hit_test_connection(local_pos: Vector2) -> Variant:
	if graph == null:
		return null
	# Convert from GraphEdit local coords to graph (logical) space.  Zoom
	# scales the rendered curves; account for it so the tolerance stays
	# consistent in screen pixels.
	var zoom: float = max(0.001, graph.zoom)
	var graph_pos: Vector2 = (local_pos / zoom) + graph.scroll_offset / zoom
	var best: Variant = null
	var best_dist: float = INF
	for conn_v in graph.get_connection_list():
		var conn: Dictionary = conn_v
		var from_n: GraphNode = graph.get_node_or_null(
			NodePath(String(conn.get("from_node", "")))) as GraphNode
		var to_n: GraphNode = graph.get_node_or_null(
			NodePath(String(conn.get("to_node", "")))) as GraphNode
		if from_n == null or to_n == null:
			continue
		var from_port_idx: int = int(conn.get("from_port", 0))
		var to_port_idx:   int = int(conn.get("to_port",   0))
		var p0: Vector2 = from_n.position_offset \
			+ from_n.get_output_port_position(from_port_idx)
		var p3: Vector2 = to_n.position_offset \
			+ to_n.get_input_port_position(to_port_idx)
		# GraphEdit draws connections with a horizontal-tangent cubic
		# bezier — control points pulled out by half the x-distance.
		var dx: float = abs(p3.x - p0.x) * 0.5
		var c1: Vector2 = p0 + Vector2(dx, 0.0)
		var c2: Vector2 = p3 - Vector2(dx, 0.0)
		var d: float = _bezier_min_distance(p0, c1, c2, p3, graph_pos,
			_CONN_BEZIER_SAMPLES)
		if d < best_dist:
			best_dist = d
			best = conn
	if best != null and best_dist <= _CONN_HIT_TOLERANCE_PX / zoom:
		return best
	return null

func _bezier_min_distance(p0: Vector2, c1: Vector2, c2: Vector2, p3: Vector2,
						   target: Vector2, samples: int) -> float:
	var min_d: float = INF
	for i in range(samples + 1):
		var t: float = float(i) / float(samples)
		var u: float = 1.0 - t
		var p: Vector2 = (u * u * u) * p0 \
			+ (3.0 * u * u * t) * c1 \
			+ (3.0 * u * t * t) * c2 \
			+ (t * t * t) * p3
		var d: float = p.distance_to(target)
		if d < min_d:
			min_d = d
	return min_d

func _topic_for_connection(conn: Dictionary) -> String:
	# Reverse-lookup a connection's topic via node_ports.  When the from-
	# node is a brain module and the output port maps to a known topic
	# we return its name; otherwise empty (boundary edges to/from
	# synthetic source/sink/event nodes use legacy slot 0 with no
	# per-topic mapping).
	var from_n: GraphNode = graph.get_node_or_null(
		NodePath(String(conn.get("from_node", "")))) as GraphNode
	if from_n == null or not from_n.has_meta("module_id"):
		return ""
	var m_id: String = String(from_n.get_meta("module_id"))
	if not node_ports.has(m_id):
		return ""
	var ports: Dictionary = node_ports[m_id]
	var output_slots: Dictionary = ports.get("output_slots", {})
	var port_idx: int = int(conn.get("from_port", 0))
	for topic_v in output_slots.keys():
		if int(output_slots[topic_v]) == port_idx:
			return String(topic_v)
	return ""

func _delete_selected_connection() -> void:
	if selected_connection == null:
		return
	var conn: Dictionary = selected_connection
	var from_n: GraphNode = graph.get_node_or_null(
		NodePath(String(conn.get("from_node", "")))) as GraphNode
	var to_n: GraphNode = graph.get_node_or_null(
		NodePath(String(conn.get("to_node", "")))) as GraphNode
	if from_n == null or to_n == null:
		selected_connection = null
		return
	# Both endpoints must be brain modules — boundary edges to/from
	# synthetic source/sink/event nodes aren't in the scheduler's edge
	# list, so disconnect would be a no-op.  Surface that explicitly.
	if not (from_n.has_meta("module_id") and to_n.has_meta("module_id")):
		_set_status(
			"✗ disconnect not supported for boundary edges (host I/O)",
			Color(0.95, 0.55, 0.30))
		selected_connection = null
		return
	var from_id: String = String(from_n.get_meta("module_id"))
	var to_id:   String = String(to_n.get_meta("module_id"))
	var topic:   String = _topic_for_connection(conn)
	var op: Dictionary = {"op": "disconnect", "from": from_id, "to": to_id}
	if topic != "":
		op["topic"] = topic
	var label := "disconnect %s → %s" % [from_id, to_id]
	if topic != "":
		label += "  (" + topic + ")"
	_apply_and_handle(op, label)
	selected_connection = null

# Recursively walk a Control subtree and clear tooltip_text on every
# descendant.  Used by _populate_graph before queue_free to suppress any
# in-flight or about-to-display tooltip Window — Godot 4's X11 backend
# throws a BadWindow on GetProperty when a tooltip's owning Control is
# freed in the same frame as the tooltip Window's resolution.
func _clear_descendant_tooltips(node: Node) -> void:
	if node is Control:
		var c: Control = node
		c.tooltip_text = ""
	for child in node.get_children():
		_clear_descendant_tooltips(child)
