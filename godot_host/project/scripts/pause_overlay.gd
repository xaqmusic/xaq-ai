extends CanvasLayer
## Phase 6.5.3.A — global pause overlay.
##
## Shows a translucent overlay with [Resume] / [Return to launcher] /
## [Quit] when the user hits ESC during an experiment.  Loaded as an
## autoload so every scene gets it automatically without per-scene
## wiring.
##
## Pause state is per-tree (get_tree().paused).  Body controllers
## already respect this via _physics_process suspension.

var _root: Control          # parent of both bg and panel; controls overall visibility
var _panel: PanelContainer
var _label: Label
var _resume_btn: Button
var _end_run_btn: Button    # Phase 6.5.3.B — triggers HUD's end-of-run summary
var _copy_btn: Button       # Phase 6.5.14b — clipboard log export (universal)
var _launcher_btn: Button
var _quit_btn: Button

func _ready() -> void:
	layer = 100   # above HUD/graph panel/etc
	_build_ui()
	_root.visible = false   # hide BOTH bg and panel until ESC fires
	process_mode = Node.PROCESS_MODE_ALWAYS  # tick even when tree is paused

func _build_ui() -> void:
	# Single root Control wraps both bg and panel so .visible toggles
	# them together.  Without this wrapping, leaving bg visible (with
	# its mouse_filter=STOP) blocks all clicks on the underlying scene
	# AND tints it 55% dark — the "greyed out" symptom we hit on the
	# launcher.
	_root = Control.new()
	_root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_root)

	var bg := ColorRect.new()
	bg.color = Color(0, 0, 0, 0.55)
	bg.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_STOP
	_root.add_child(bg)

	_panel = PanelContainer.new()
	_panel.set_anchors_and_offsets_preset(Control.PRESET_CENTER)
	_panel.custom_minimum_size = Vector2(360, 220)
	_root.add_child(_panel)
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left",   24)
	margin.add_theme_constant_override("margin_right",  24)
	margin.add_theme_constant_override("margin_top",    20)
	margin.add_theme_constant_override("margin_bottom", 20)
	_panel.add_child(margin)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 12)
	margin.add_child(v)

	_label = Label.new()
	_label.text = "Paused"
	_label.add_theme_font_size_override("font_size", 22)
	_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	v.add_child(_label)

	_resume_btn = Button.new()
	_resume_btn.text = "Resume"
	_resume_btn.custom_minimum_size = Vector2(0, 36)
	v.add_child(_resume_btn)

	_end_run_btn = Button.new()
	_end_run_btn.text = "End run & show summary"
	_end_run_btn.custom_minimum_size = Vector2(0, 36)
	v.add_child(_end_run_btn)

	# Phase 6.5.14b — clipboard export accessible from any env mid-run.
	# Cell has no end-of-run modal so this is the only path; episodic
	# envs (CartPole, MC) have a copy button on the modal too.
	_copy_btn = Button.new()
	_copy_btn.text = "📋 Copy log"
	_copy_btn.custom_minimum_size = Vector2(0, 36)
	v.add_child(_copy_btn)

	_launcher_btn = Button.new()
	_launcher_btn.text = "Return to launcher (no summary)"
	_launcher_btn.custom_minimum_size = Vector2(0, 36)
	v.add_child(_launcher_btn)

	_quit_btn = Button.new()
	_quit_btn.text = "Quit"
	_quit_btn.custom_minimum_size = Vector2(0, 36)
	v.add_child(_quit_btn)

	_resume_btn.pressed.connect(_on_resume)
	_end_run_btn.pressed.connect(_on_end_run)
	_copy_btn.pressed.connect(_on_copy_log)
	_launcher_btn.pressed.connect(_on_launcher)
	_quit_btn.pressed.connect(_on_quit)

func _input(event: InputEvent) -> void:
	# Don't intercept ESC on the launcher itself.
	var current := get_tree().current_scene
	if current and current.name == "Launcher":
		return
	if event is InputEventKey and event.pressed and not event.echo \
			and event.keycode == KEY_ESCAPE:
		_toggle()

func _toggle() -> void:
	_root.visible = not _root.visible
	get_tree().paused = _root.visible

func _on_resume() -> void:
	_root.visible = false
	get_tree().paused = false

func _on_launcher() -> void:
	get_tree().paused = false
	_root.visible = false
	# Reset launched flag so next launcher cycle reads from scratch.
	ExperimentConfig.launched = false
	get_tree().change_scene_to_file("res://scenes/launcher.tscn")

func _on_quit() -> void:
	get_tree().quit()

func _on_end_run() -> void:
	# Phase 6.5.3.B — find the active scene's HUD overlay (Cell, CartPole,
	# or MountainCar) and call request_end_run() if it has that method.
	# The HUD will set body._done which triggers the summary modal next
	# _process tick.  We then close the pause overlay.
	var current := get_tree().current_scene
	if current:
		var hud := _find_hud(current)
		if hud and hud.has_method("request_end_run"):
			hud.request_end_run()
	_root.visible = false
	get_tree().paused = false

func _on_copy_log() -> void:
	# Phase 6.5.14b — universal clipboard log export.  Each HUD provides
	# build_clipboard_text(body); we look it up, call it, push to system
	# clipboard, and show "✓ Copied" briefly so the user gets visual
	# confirmation without having to leave the pause overlay.
	var current := get_tree().current_scene
	if current == null:
		return
	var hud := _find_hud(current)
	if hud == null or not hud.has_method("build_clipboard_text"):
		_copy_btn.text = "(no log available)"
		return
	# Find body the same way the HUD does — episodic envs use Cart, MC
	# uses Cart (yes, same name); Cell uses Body.
	var body: Node = get_tree().get_root().find_child("Cart", true, false)
	if body == null:
		body = get_tree().get_root().find_child("Body", true, false)
	if body == null:
		_copy_btn.text = "(body not found)"
		return
	var txt: String = hud.build_clipboard_text(body)
	DisplayServer.clipboard_set(txt)
	_copy_btn.text = "✓ Copied"
	var t := Timer.new()
	t.wait_time = 1.5
	t.one_shot = true
	t.timeout.connect(func():
		_copy_btn.text = "📋 Copy log"
		t.queue_free()
	)
	_copy_btn.add_child(t)
	t.start()

func _find_hud(node: Node) -> Node:
	# Episodic envs: HUD is at HUD/Overlay (script attached to Overlay).
	# Cell: hud is at UI/Hud (mixed case in ui.tscn).  Try episodic first,
	# then both casings of the Cell path so the lookup is robust to scene
	# renames.
	var n := node.get_node_or_null("HUD/Overlay")
	if n: return n
	n = node.get_node_or_null("UI/Hud")
	if n: return n
	return node.get_node_or_null("UI/HUD")
