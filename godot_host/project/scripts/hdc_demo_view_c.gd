extends Control

# View C — "Building a consensus vector, step by step"
# Walks through the four-step pipeline of docs/active_inference.md §2 and
# docs/hdc_consensus_explainer.md §4: each token's symbol vector is bound to
# its time anchor (⊙), all bound vectors are summed, and the result is
# normalised. Step through manually or auto-play.
#
# Uses d = 64 for visual clarity. Production substrate runs at d = 128.

const DIM: int = 64
const N_TOKENS: int = 4
const TIME_ANCHOR_COUNT: int = 12
const TIME_ANCHOR_SEED: int = 0xDEADBEEF
const SYMBOL_SEED_BASE: int = 0x1234567
const PHI: float = 0.6180339887
const AUTO_PLAY_INTERVAL: float = 1.0  # seconds per step

const PREV_SCENE: String = "res://scenes/hdc_demo_view_b.tscn"
const NEXT_SCENE: String = "res://scenes/hdc_demo_view_d.tscn"

# Final step index = N_TOKENS + 1 (normalise step).
# Step semantics:
#   0          → empty: timeline only
#   1..N       → token i added (its sym/time/bound shown, sum updated)
#   N + 1      → final normalise (consensus shown)

var step: int = 0
var _seed: int = 4004
var _time_anchors: Array = []  # Array[PackedFloat32Array]
var _tokens: Array = []  # Array[Dictionary{symbol_id, t, sym_vec, time_vec, bound, color}]
var _running_sum: PackedFloat32Array = PackedFloat32Array()
var _consensus: PackedFloat32Array = PackedFloat32Array()
var _play_acc: float = 0.0
var _playing: bool = false

@onready var timeline_canvas: Control = $V/TimelineCanvas
@onready var step_label: Label = $V/StepRow/StepLabel
@onready var tokens_canvas: Control = $V/TokensCanvas
@onready var sum_canvas: Control = $V/SumCanvas
@onready var reset_button: Button = $V/Controls/Reset
@onready var step_button: Button = $V/Controls/StepFwd
@onready var play_button: Button = $V/Controls/PlayPause
@onready var regen_button: Button = $V/Controls/Regenerate
@onready var prev_button: Button = $V/Nav/PrevButton
@onready var next_button: Button = $V/Nav/NextButton


func _ready() -> void:
	reset_button.pressed.connect(_on_reset)
	step_button.pressed.connect(_on_step_fwd)
	play_button.pressed.connect(_on_play_toggle)
	regen_button.pressed.connect(_on_regenerate)
	timeline_canvas.draw.connect(_draw_timeline)
	timeline_canvas.resized.connect(timeline_canvas.queue_redraw)
	tokens_canvas.draw.connect(_draw_tokens)
	tokens_canvas.resized.connect(tokens_canvas.queue_redraw)
	sum_canvas.draw.connect(_draw_sum)
	sum_canvas.resized.connect(sum_canvas.queue_redraw)

	_setup_nav()
	_rebuild_time_anchors()
	_regenerate_tokens()
	_set_step(0)
	set_process(true)


func _process(delta: float) -> void:
	if not _playing:
		return
	_play_acc += delta
	if _play_acc >= AUTO_PLAY_INTERVAL:
		_play_acc = 0.0
		if step < N_TOKENS + 1:
			_set_step(step + 1)
		else:
			_playing = false
			_update_play_button_text()


func _on_reset() -> void:
	_set_step(0)


func _on_step_fwd() -> void:
	if step < N_TOKENS + 1:
		_set_step(step + 1)


func _on_play_toggle() -> void:
	_playing = not _playing
	_play_acc = 0.0
	if _playing and step >= N_TOKENS + 1:
		# restart from beginning when pressing play at the end
		_set_step(0)
	_update_play_button_text()


func _on_regenerate() -> void:
	_seed += 1
	_regenerate_tokens()
	_set_step(0)


func _update_play_button_text() -> void:
	play_button.text = "⏸ Pause" if _playing else "▶ Play"


func _set_step(new_step: int) -> void:
	step = clampi(new_step, 0, N_TOKENS + 1)
	_recompute_running()
	_update_step_label()
	timeline_canvas.queue_redraw()
	tokens_canvas.queue_redraw()
	sum_canvas.queue_redraw()


func _update_step_label() -> void:
	var name: String = ""
	if step == 0:
		name = "ready — press Step or Play to begin"
	elif step <= N_TOKENS:
		var tok: Dictionary = _tokens[step - 1]
		name = "add token %d:  symbol=%d  t=%.3f" % [step, tok["symbol_id"], tok["t"]]
	else:
		name = "normalise:  consensus = sum / ‖sum‖"
	step_label.text = "Step %d / %d   —   %s" % [step, N_TOKENS + 1, name]


func _rebuild_time_anchors() -> void:
	_time_anchors.clear()
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = TIME_ANCHOR_SEED
	for i in range(TIME_ANCHOR_COUNT):
		_time_anchors.append(_random_unit_vector(rng, DIM))


func _regenerate_tokens() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = _seed
	_tokens.clear()
	for i in range(N_TOKENS):
		var sym_id: int = rng.randi() % 10000
		var t: float = rng.randf()
		var sym_vec: PackedFloat32Array = _symbol_vector(sym_id)
		var time_vec: PackedFloat32Array = _time_anchor_at(t)
		var bound: PackedFloat32Array = _multiply(sym_vec, time_vec)
		var hue: float = fmod(float(sym_id) * PHI, 1.0)
		_tokens.append({
			"symbol_id": sym_id,
			"t": t,
			"sym_vec": sym_vec,
			"time_vec": time_vec,
			"bound": bound,
			"color": Color.from_hsv(hue, 0.7, 1.0),
		})


func _recompute_running() -> void:
	_running_sum = PackedFloat32Array()
	_running_sum.resize(DIM)
	for j in range(DIM):
		_running_sum[j] = 0.0
	var added: int = min(step, N_TOKENS)
	for i in range(added):
		var b: PackedFloat32Array = _tokens[i]["bound"]
		for j in range(DIM):
			_running_sum[j] += b[j]
	_consensus = PackedFloat32Array()
	_consensus.resize(DIM)
	for j in range(DIM):
		_consensus[j] = _running_sum[j]
	_normalize(_consensus)


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
	return path.get_file()


func _on_prev_pressed() -> void:
	get_tree().change_scene_to_file(PREV_SCENE)


func _on_next_pressed() -> void:
	get_tree().change_scene_to_file(NEXT_SCENE)


# --- Math helpers ---

func _random_unit_vector(rng: RandomNumberGenerator, d: int) -> PackedFloat32Array:
	var v: PackedFloat32Array = PackedFloat32Array()
	v.resize(d)
	var ns: float = 0.0
	var i: int = 0
	while i < d:
		var u1: float = maxf(rng.randf(), 1e-12)
		var u2: float = rng.randf()
		var r: float = sqrt(-2.0 * log(u1))
		var theta: float = TAU * u2
		var x0: float = r * cos(theta)
		v[i] = x0
		ns += x0 * x0
		if i + 1 < d:
			var x1: float = r * sin(theta)
			v[i + 1] = x1
			ns += x1 * x1
		i += 2
	var inv: float = 1.0 / sqrt(maxf(ns, 1e-12))
	for j in range(d):
		v[j] *= inv
	return v


func _symbol_vector(symbol_id: int) -> PackedFloat32Array:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = SYMBOL_SEED_BASE + symbol_id * 31
	return _random_unit_vector(rng, DIM)


func _time_anchor_at(t: float) -> PackedFloat32Array:
	var ct: float = clampf(t, 0.0, 1.0)
	var pos: float = ct * float(TIME_ANCHOR_COUNT - 1)
	var i0: int = int(floor(pos))
	var i1: int = min(i0 + 1, TIME_ANCHOR_COUNT - 1)
	var frac: float = pos - float(i0)
	var a: PackedFloat32Array = _time_anchors[i0]
	var b: PackedFloat32Array = _time_anchors[i1]
	var r: PackedFloat32Array = PackedFloat32Array()
	r.resize(DIM)
	for j in range(DIM):
		r[j] = a[j] * (1.0 - frac) + b[j] * frac
	_normalize(r)
	return r


func _multiply(a: PackedFloat32Array, b: PackedFloat32Array) -> PackedFloat32Array:
	var r: PackedFloat32Array = PackedFloat32Array()
	r.resize(a.size())
	for i in range(a.size()):
		r[i] = a[i] * b[i]
	return r


func _normalize(v: PackedFloat32Array) -> void:
	var ns: float = 0.0
	for i in range(v.size()):
		ns += v[i] * v[i]
	var inv: float = 1.0 / sqrt(maxf(ns, 1e-12))
	for i in range(v.size()):
		v[i] *= inv


# --- Drawing ---

func _value_color(v: float, scale: float) -> Color:
	var t: float = clampf(v / scale, -1.0, 1.0)
	var bg: Color = Color(0.10, 0.11, 0.14)
	if t >= 0.0:
		return bg.lerp(Color(1.00, 0.55, 0.30), t)
	else:
		return bg.lerp(Color(0.30, 0.55, 1.00), -t)


func _draw_strip(c: Control, v: PackedFloat32Array, x: float, y: float, w: float, h: float, faded: bool = false) -> void:
	var n: int = v.size()
	if n == 0:
		c.draw_rect(Rect2(x, y, w, h), Color(0.13, 0.14, 0.17))
		c.draw_rect(Rect2(x, y, w, h), Color(0.30, 0.30, 0.35), false, 1.0)
		return
	var max_abs: float = 1e-12
	for val in v:
		var a: float = absf(val)
		if a > max_abs:
			max_abs = a
	var cw: float = w / float(n)
	for i in range(n):
		var color: Color = _value_color(v[i], max_abs)
		if faded:
			color = color.lerp(Color(0.10, 0.11, 0.14), 0.6)
		c.draw_rect(Rect2(x + i * cw, y, cw, h), color)
	c.draw_rect(Rect2(x, y, w, h), Color(0.35, 0.35, 0.40), false, 1.0)


func _draw_timeline() -> void:
	var size: Vector2 = timeline_canvas.size
	var w: float = size.x
	var h: float = size.y
	if w < 40 or h < 20:
		return
	timeline_canvas.draw_rect(Rect2(0, 0, w, h), Color(0.10, 0.11, 0.14))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12
	var lc: Color = Color(0.78, 0.80, 0.85)

	var margin: float = 80.0
	var strip_x: float = margin + 20.0
	var strip_w: float = w - strip_x - 16.0
	var strip_h: float = 22.0
	var strip_y: float = (h - strip_h) * 0.5
	if strip_w < 40.0:
		return

	timeline_canvas.draw_string(font, Vector2(8, strip_y + strip_h * 0.5 + 4),
		"sync window:", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, lc)
	timeline_canvas.draw_rect(Rect2(strip_x, strip_y, strip_w, strip_h),
		Color(0.16, 0.17, 0.21))
	# Tick marks
	for tick_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
		var tx: float = strip_x + tick_t * strip_w
		timeline_canvas.draw_line(Vector2(tx, strip_y + strip_h),
			Vector2(tx, strip_y + strip_h + 4),
			Color(0.4, 0.4, 0.45), 1.0)
		var lab: String = "%.2f" % tick_t
		var ls: Vector2 = font.get_string_size(lab, HORIZONTAL_ALIGNMENT_CENTER, -1, fs)
		timeline_canvas.draw_string(font,
			Vector2(tx - ls.x * 0.5, strip_y + strip_h + ls.y + 4),
			lab, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.65, 0.65, 0.70))

	# Tokens
	for i in range(N_TOKENS):
		var tok: Dictionary = _tokens[i]
		var tx: float = strip_x + tok["t"] * strip_w
		var ty: float = strip_y + strip_h * 0.5
		var is_added: bool = step >= i + 1
		var is_current: bool = step == i + 1
		var c: Color = tok["color"]
		if not is_added:
			c = c.lerp(Color(0.2, 0.2, 0.25), 0.6)
		if is_current:
			timeline_canvas.draw_circle(Vector2(tx, ty), 11.0,
				Color(1.0, 1.0, 1.0, 0.45))
		timeline_canvas.draw_circle(Vector2(tx, ty), 7.0, c)
		var lbl: String = "%d" % (i + 1)
		var ls: Vector2 = font.get_string_size(lbl, HORIZONTAL_ALIGNMENT_CENTER, -1, fs)
		timeline_canvas.draw_string(font,
			Vector2(tx - ls.x * 0.5, ty - 12),
			lbl, HORIZONTAL_ALIGNMENT_LEFT, -1, fs,
			Color(0.92, 0.93, 0.96) if is_added else Color(0.5, 0.5, 0.55))


func _draw_tokens() -> void:
	var size: Vector2 = tokens_canvas.size
	var w: float = size.x
	var h: float = size.y
	if w < 80 or h < 40:
		return
	tokens_canvas.draw_rect(Rect2(0, 0, w, h), Color(0.08, 0.09, 0.12))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12
	var lc: Color = Color(0.85, 0.86, 0.90)

	# Layout: per row: [color+label] | [sym strip] ⊙ [time strip] = [bound strip]
	var row_h: float = (h - 16.0) / float(N_TOKENS)
	if row_h < 30.0:
		row_h = 30.0
	var strip_h: float = minf(row_h - 12.0, 24.0)
	var label_w: float = 120.0
	var content_x: float = label_w + 8.0
	var content_w: float = w - content_x - 12.0
	if content_w < 200.0:
		return
	# Three strips per row, separated by operator labels
	var op_w: float = 22.0
	var strip_w: float = (content_w - 2.0 * op_w) / 3.0

	for i in range(N_TOKENS):
		var tok: Dictionary = _tokens[i]
		var is_added: bool = step >= i + 1
		var is_current: bool = step == i + 1
		var y_top: float = 8.0 + i * row_h
		var y_mid: float = y_top + row_h * 0.5
		var strip_y: float = y_mid - strip_h * 0.5

		# Highlight current row
		if is_current:
			tokens_canvas.draw_rect(Rect2(0, y_top - 2, w, row_h),
				Color(1.0, 0.78, 0.30, 0.10))

		# Label
		var c: Color = tok["color"] if is_added else tok["color"].lerp(Color(0.3, 0.3, 0.35), 0.7)
		tokens_canvas.draw_circle(Vector2(14, y_mid), 6.0, c)
		var info: String = "Token %d\nid=%d  t=%.2f" % [i + 1, tok["symbol_id"], tok["t"]]
		tokens_canvas.draw_multiline_string(font, Vector2(26, y_mid - 8),
			info, HORIZONTAL_ALIGNMENT_LEFT, -1, fs,
			-1, lc if is_added else Color(0.5, 0.5, 0.55))

		# Strips
		var x: float = content_x
		_draw_strip(tokens_canvas, tok["sym_vec"], x, strip_y, strip_w, strip_h, not is_added)
		_draw_op(tokens_canvas, "⊙", x + strip_w, y_mid, op_w, font, fs)
		_draw_strip(tokens_canvas, tok["time_vec"], x + strip_w + op_w, strip_y, strip_w, strip_h, not is_added)
		_draw_op(tokens_canvas, "=", x + 2 * strip_w + op_w, y_mid, op_w, font, fs)
		# Bound strip: shown only if added
		if is_added:
			_draw_strip(tokens_canvas, tok["bound"], x + 2 * strip_w + 2 * op_w, strip_y, strip_w, strip_h, false)
		else:
			_draw_strip(tokens_canvas, PackedFloat32Array(), x + 2 * strip_w + 2 * op_w, strip_y, strip_w, strip_h, true)

		# Sub-labels on first row only (header for the three strip columns)
		if i == 0:
			var sublabel_y: float = strip_y - 6
			tokens_canvas.draw_string(font, Vector2(x + 4, sublabel_y),
				"Z_sym", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.65, 0.67, 0.72))
			tokens_canvas.draw_string(font, Vector2(x + strip_w + op_w + 4, sublabel_y),
				"T(t)", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.65, 0.67, 0.72))
			tokens_canvas.draw_string(font, Vector2(x + 2 * strip_w + 2 * op_w + 4, sublabel_y),
				"bound = Z_sym ⊙ T(t)", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.65, 0.67, 0.72))


func _draw_op(c: Control, sym: String, x: float, y_mid: float, w: float, font: Font, fs: int) -> void:
	var size: Vector2 = font.get_string_size(sym, HORIZONTAL_ALIGNMENT_CENTER, -1, fs + 4)
	c.draw_string(font,
		Vector2(x + w * 0.5 - size.x * 0.5, y_mid + 6),
		sym, HORIZONTAL_ALIGNMENT_LEFT, -1, fs + 4, Color(0.85, 0.86, 0.90))


func _draw_sum() -> void:
	var size: Vector2 = sum_canvas.size
	var w: float = size.x
	var h: float = size.y
	if w < 80 or h < 40:
		return
	sum_canvas.draw_rect(Rect2(0, 0, w, h), Color(0.08, 0.09, 0.12))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12

	var label_w: float = 120.0
	var content_x: float = label_w + 8.0
	var content_w: float = w - content_x - 12.0
	var strip_h: float = 22.0
	if content_w < 100.0:
		return

	var sum_y: float = 12.0
	var consensus_y: float = sum_y + strip_h + 18.0

	sum_canvas.draw_string(font, Vector2(8, sum_y + strip_h * 0.5 + 4),
		"running sum:", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(1.0, 0.78, 0.30))
	if step >= 1:
		_draw_strip(sum_canvas, _running_sum, content_x, sum_y, content_w, strip_h, false)
	else:
		_draw_strip(sum_canvas, PackedFloat32Array(), content_x, sum_y, content_w, strip_h, true)

	sum_canvas.draw_string(font, Vector2(8, consensus_y + strip_h * 0.5 + 4),
		"normalised\nconsensus:", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.55, 0.85, 1.00))
	if step >= N_TOKENS + 1:
		_draw_strip(sum_canvas, _consensus, content_x, consensus_y, content_w, strip_h, false)
	else:
		_draw_strip(sum_canvas, PackedFloat32Array(), content_x, consensus_y, content_w, strip_h, true)
