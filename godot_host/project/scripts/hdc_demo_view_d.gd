extends Control

# View D — "The k/n property"
# Two sync windows side by side. Window 2 shares k of n (symbol, time) pairs
# with Window 1. Drag k from 0 to n and watch the consensus cosine track k/n.
# The scatter plot accumulates points so the diagonal emerges live.

const DIM_OPTIONS: Array[int] = [16, 32, 64, 128, 256, 512, 768]
const TIME_ANCHOR_COUNT: int = 12
const TIME_ANCHOR_SEED: int = 0xDEADBEEF
const SCATTER_X_MIN: float = 0.0
const SCATTER_X_MAX: float = 1.0
const SCATTER_Y_MIN: float = -0.15
const SCATTER_Y_MAX: float = 1.05
const MAX_SCATTER_POINTS: int = 5000
const SYMBOL_SEED_BASE: int = 0x1234567
const FRESH_SEED_MULT: int = 31
const FRESH_SEED_OFFSET: int = 17
const PHI: float = 0.6180339887

const PREV_SCENE: String = "res://scenes/hdc_demo_view_c.tscn"
const NEXT_SCENE: String = "res://scenes/hdc_demo_view_e.tscn"

var dimension: int = 128
var n_tokens: int = 8
var k_shared: int = 4
var time_jitter: float = 0.0
var rng_seed: int = 1001

var _time_anchors: Array = []  # Array[PackedFloat32Array]
var _w1_tokens: Array = []  # Array[Dictionary{symbol_id, t}]
var _w2_tokens: Array = []
var _w1_consensus: PackedFloat32Array = PackedFloat32Array()
var _w2_consensus: PackedFloat32Array = PackedFloat32Array()
var _cosine: float = 0.0

var _scatter: Array = []  # Array[Vector2]

@onready var n_slider: HSlider = $V/Controls/NRow/NSlider
@onready var n_value: Label = $V/Controls/NRow/NValue
@onready var k_slider: HSlider = $V/Controls/KRow/KSlider
@onready var k_value: Label = $V/Controls/KRow/KValue
@onready var d_slider: HSlider = $V/Controls/DRow/DSlider
@onready var d_value: Label = $V/Controls/DRow/DValue
@onready var jitter_slider: HSlider = $V/Controls/JitterRow/JitterSlider
@onready var jitter_value: Label = $V/Controls/JitterRow/JitterValue
@onready var regen_button: Button = $V/Controls/ButtonsRow/Regenerate
@onready var clear_button: Button = $V/Controls/ButtonsRow/ClearScatter
@onready var windows_canvas: Control = $V/H/WindowsCanvas
@onready var scatter_canvas: Control = $V/H/ScatterCanvas
@onready var stats_label: Label = $V/Stats
@onready var prev_button: Button = $V/Nav/PrevButton
@onready var next_button: Button = $V/Nav/NextButton


func _ready() -> void:
	d_slider.min_value = 0
	d_slider.max_value = DIM_OPTIONS.size() - 1
	d_slider.step = 1
	d_slider.value = max(DIM_OPTIONS.find(dimension), 0)

	n_slider.min_value = 2
	n_slider.max_value = 20
	n_slider.step = 1
	n_slider.value = n_tokens

	k_slider.min_value = 0
	k_slider.max_value = n_tokens
	k_slider.step = 1
	k_slider.value = k_shared

	jitter_slider.min_value = 0.0
	jitter_slider.max_value = 0.5
	jitter_slider.step = 0.01
	jitter_slider.value = time_jitter

	n_slider.value_changed.connect(_on_n_changed)
	k_slider.value_changed.connect(_on_k_changed)
	d_slider.value_changed.connect(_on_d_changed)
	jitter_slider.value_changed.connect(_on_jitter_changed)
	regen_button.pressed.connect(_on_regenerate)
	clear_button.pressed.connect(_on_clear_scatter)
	windows_canvas.draw.connect(_draw_windows)
	windows_canvas.resized.connect(windows_canvas.queue_redraw)
	scatter_canvas.draw.connect(_draw_scatter)
	scatter_canvas.resized.connect(scatter_canvas.queue_redraw)

	_setup_nav()
	_rebuild_time_anchors()
	_regenerate_tokens()
	_recompute()


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
	if PREV_SCENE != "":
		get_tree().change_scene_to_file(PREV_SCENE)


func _on_next_pressed() -> void:
	if NEXT_SCENE != "":
		get_tree().change_scene_to_file(NEXT_SCENE)


func _on_n_changed(value: float) -> void:
	n_tokens = int(value)
	k_shared = min(k_shared, n_tokens)
	k_slider.max_value = n_tokens
	k_slider.set_value_no_signal(k_shared)
	_regenerate_tokens()
	_recompute()


func _on_k_changed(value: float) -> void:
	k_shared = int(value)
	_update_window2_tokens()
	_recompute()


func _on_d_changed(value: float) -> void:
	var idx: int = int(value)
	dimension = DIM_OPTIONS[idx]
	_rebuild_time_anchors()
	_scatter.clear()
	_recompute()


func _on_jitter_changed(value: float) -> void:
	time_jitter = value
	_update_window2_tokens()
	_recompute()


func _on_regenerate() -> void:
	rng_seed += 1
	_regenerate_tokens()
	_recompute()


func _on_clear_scatter() -> void:
	_scatter.clear()
	scatter_canvas.queue_redraw()


func _rebuild_time_anchors() -> void:
	_time_anchors.clear()
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = TIME_ANCHOR_SEED
	for i in range(TIME_ANCHOR_COUNT):
		_time_anchors.append(_random_unit_vector(rng, dimension))


func _regenerate_tokens() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = rng_seed
	_w1_tokens.clear()
	for i in range(n_tokens):
		_w1_tokens.append({
			"symbol_id": rng.randi() % 10000,
			"t": rng.randf(),
		})
	_update_window2_tokens()


func _update_window2_tokens() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = rng_seed * FRESH_SEED_MULT + FRESH_SEED_OFFSET
	_w2_tokens.clear()
	for i in range(k_shared):
		var src: Dictionary = _w1_tokens[i]
		var t: float = src["t"]
		if time_jitter > 0.0:
			var delta: float = rng.randf_range(-time_jitter, time_jitter)
			t = clampf(t + delta, 0.0, 1.0)
		_w2_tokens.append({"symbol_id": src["symbol_id"], "t": t})

	var w1_ids: Dictionary = {}
	for tok in _w1_tokens:
		w1_ids[tok["symbol_id"]] = true

	var added: int = 0
	var safety: int = 0
	while added < n_tokens - k_shared and safety < 10000:
		var sym: int = rng.randi() % 10000
		if not w1_ids.has(sym):
			_w2_tokens.append({"symbol_id": sym, "t": rng.randf()})
			added += 1
		safety += 1


func _recompute() -> void:
	_w1_consensus = _compute_consensus(_w1_tokens)
	_w2_consensus = _compute_consensus(_w2_tokens)
	_cosine = _dot(_w1_consensus, _w2_consensus)

	var kn: float = float(k_shared) / max(float(n_tokens), 1.0)
	_scatter.append(Vector2(kn, _cosine))
	if _scatter.size() > MAX_SCATTER_POINTS:
		_scatter.pop_front()

	d_value.text = "%d" % dimension
	n_value.text = "%d" % n_tokens
	k_value.text = "%d" % k_shared
	jitter_value.text = "%.2f" % time_jitter

	var theo: float = 1.0 / sqrt(float(dimension))
	stats_label.text = (
		"d = %d   n = %d   k = %d   jitter = %.2f\n"
		+ "k/n = %.3f    observed cos(W1, W2) = %+.4f    |cos − k/n| = %.4f\n"
		+ "theoretical noise floor (1/√d) = ±%.4f    scatter points = %d"
	) % [
		dimension, n_tokens, k_shared, time_jitter,
		kn, _cosine, absf(_cosine - kn), theo, _scatter.size(),
	]

	windows_canvas.queue_redraw()
	scatter_canvas.queue_redraw()


func _compute_consensus(tokens: Array) -> PackedFloat32Array:
	var c: PackedFloat32Array = PackedFloat32Array()
	c.resize(dimension)
	for tok in tokens:
		var sym_vec: PackedFloat32Array = _symbol_vector(tok["symbol_id"])
		var time_vec: PackedFloat32Array = _time_anchor_at(tok["t"])
		for i in range(dimension):
			c[i] += sym_vec[i] * time_vec[i]
	_normalize(c)
	return c


func _symbol_vector(symbol_id: int) -> PackedFloat32Array:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = SYMBOL_SEED_BASE + symbol_id * 31
	return _random_unit_vector(rng, dimension)


func _time_anchor_at(t: float) -> PackedFloat32Array:
	var ct: float = clampf(t, 0.0, 1.0)
	var pos: float = ct * float(TIME_ANCHOR_COUNT - 1)
	var i0: int = int(floor(pos))
	var i1: int = min(i0 + 1, TIME_ANCHOR_COUNT - 1)
	var frac: float = pos - float(i0)
	var a: PackedFloat32Array = _time_anchors[i0]
	var b: PackedFloat32Array = _time_anchors[i1]
	var r: PackedFloat32Array = PackedFloat32Array()
	r.resize(dimension)
	for j in range(dimension):
		r[j] = a[j] * (1.0 - frac) + b[j] * frac
	_normalize(r)
	return r


func _normalize(v: PackedFloat32Array) -> void:
	var ns: float = 0.0
	for i in range(v.size()):
		ns += v[i] * v[i]
	var inv: float = 1.0 / sqrt(maxf(ns, 1e-12))
	for i in range(v.size()):
		v[i] *= inv


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


func _dot(a: PackedFloat32Array, b: PackedFloat32Array) -> float:
	var s: float = 0.0
	var n: int = a.size()
	for i in range(n):
		s += a[i] * b[i]
	return s


# --- Drawing ---

func _draw_windows() -> void:
	var size: Vector2 = windows_canvas.size
	var w: float = size.x
	var h: float = size.y
	if w < 40.0 or h < 40.0:
		return

	windows_canvas.draw_rect(Rect2(0, 0, w, h), Color(0.10, 0.11, 0.14))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 13
	var label_color: Color = Color(0.78, 0.80, 0.85)

	var margin: float = 20.0
	var strip_h: float = 32.0
	var strip_spacing: float = 56.0
	var strip_y1: float = 50.0
	var strip_y2: float = strip_y1 + strip_h + strip_spacing
	var strip_x: float = margin + 78.0
	var strip_w: float = w - strip_x - margin
	if strip_w < 40.0:
		return

	var shared_ids: Dictionary = {}
	for i in range(k_shared):
		shared_ids[_w1_tokens[i]["symbol_id"]] = true

	windows_canvas.draw_string(font, Vector2(margin, strip_y1 - 12),
		"Window 1", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color(0.92, 0.93, 0.96))
	windows_canvas.draw_string(font, Vector2(margin, strip_y2 - 12),
		"Window 2", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color(0.92, 0.93, 0.96))

	for sy in [strip_y1, strip_y2]:
		windows_canvas.draw_rect(Rect2(strip_x, sy, strip_w, strip_h),
			Color(0.16, 0.17, 0.21))
		for tick_t in [0.0, 0.25, 0.5, 0.75, 1.0]:
			var tx: float = strip_x + tick_t * strip_w
			windows_canvas.draw_line(Vector2(tx, sy + strip_h),
				Vector2(tx, sy + strip_h + 4),
				Color(0.4, 0.4, 0.45), 1.0)

	_draw_tokens(_w1_tokens, strip_x, strip_y1, strip_w, strip_h, shared_ids)
	_draw_tokens(_w2_tokens, strip_x, strip_y2, strip_w, strip_h, shared_ids)

	var time_label: String = "time within sync window  t ∈ [0, 1]"
	var ts: Vector2 = font.get_string_size(time_label, HORIZONTAL_ALIGNMENT_CENTER, -1, fs)
	windows_canvas.draw_string(font,
		Vector2(strip_x + strip_w * 0.5 - ts.x * 0.5, strip_y2 + strip_h + 22),
		time_label, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, label_color)

	var kn: float = float(k_shared) / max(float(n_tokens), 1.0)
	var headline_y: float = strip_y2 + strip_h + 54
	windows_canvas.draw_string(font,
		Vector2(strip_x, headline_y),
		"cos(W1, W2) = %+.4f      k/n = %.3f" % [_cosine, kn],
		HORIZONTAL_ALIGNMENT_LEFT, -1, 18, Color(0.55, 0.85, 1.00))
	windows_canvas.draw_string(font,
		Vector2(strip_x, headline_y + 26),
		"|cos − k/n| = %.4f" % absf(_cosine - kn),
		HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color(0.75, 0.77, 0.82))

	# Legend
	var legend_y: float = headline_y + 56
	windows_canvas.draw_circle(Vector2(strip_x + 8, legend_y), 7.0, Color(1, 1, 1, 0.5))
	windows_canvas.draw_circle(Vector2(strip_x + 8, legend_y), 5.0, Color(0.5, 0.7, 1.0))
	windows_canvas.draw_string(font, Vector2(strip_x + 22, legend_y + 5),
		"shared (symbol, time) pair", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, label_color)
	windows_canvas.draw_circle(Vector2(strip_x + 220, legend_y), 5.0, Color(0.6, 0.6, 0.6))
	windows_canvas.draw_string(font, Vector2(strip_x + 232, legend_y + 5),
		"unique to its window", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, label_color)


func _draw_tokens(tokens: Array, x_start: float, y: float, w: float, h: float, shared_ids: Dictionary) -> void:
	for tok in tokens:
		var t: float = tok["t"]
		var sym_id: int = tok["symbol_id"]
		var x: float = x_start + t * w
		var is_shared: bool = shared_ids.has(sym_id)
		if is_shared:
			var hue: float = fmod(float(sym_id) * PHI, 1.0)
			var c: Color = Color.from_hsv(hue, 0.7, 1.0)
			windows_canvas.draw_circle(Vector2(x, y + h * 0.5), 9.0,
				Color(1.0, 1.0, 1.0, 0.45))
			windows_canvas.draw_circle(Vector2(x, y + h * 0.5), 6.0, c)
		else:
			windows_canvas.draw_circle(Vector2(x, y + h * 0.5), 5.0, Color(0.55, 0.55, 0.58))


func _draw_scatter() -> void:
	var size: Vector2 = scatter_canvas.size
	var w: float = size.x
	var h: float = size.y
	if w < 80.0 or h < 80.0:
		return

	scatter_canvas.draw_rect(Rect2(0, 0, w, h), Color(0.10, 0.11, 0.14))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12

	var ml: float = 50.0
	var mr: float = 16.0
	var mt: float = 36.0
	var mb: float = 38.0
	var pw: float = w - ml - mr
	var ph: float = h - mt - mb
	if pw < 40.0 or ph < 40.0:
		return

	var xr: float = SCATTER_X_MAX - SCATTER_X_MIN
	var yr: float = SCATTER_Y_MAX - SCATTER_Y_MIN
	var axis_color: Color = Color(0.35, 0.35, 0.40)

	scatter_canvas.draw_line(Vector2(ml, mt), Vector2(ml, mt + ph), axis_color, 1.0)
	var zero_y_screen: float = mt + (SCATTER_Y_MAX - 0.0) / yr * ph
	scatter_canvas.draw_line(
		Vector2(ml, zero_y_screen),
		Vector2(ml + pw, zero_y_screen), axis_color, 1.0
	)

	# ±1/√d band around y=x diagonal
	var noise: float = 1.0 / sqrt(float(dimension))
	var dx0: float = ml + (0.0 - SCATTER_X_MIN) / xr * pw
	var dx1: float = ml + (1.0 - SCATTER_X_MIN) / xr * pw
	var top0: float = mt + (SCATTER_Y_MAX - clampf(0.0 + noise, SCATTER_Y_MIN, SCATTER_Y_MAX)) / yr * ph
	var top1: float = mt + (SCATTER_Y_MAX - clampf(1.0 + noise, SCATTER_Y_MIN, SCATTER_Y_MAX)) / yr * ph
	var bot1: float = mt + (SCATTER_Y_MAX - clampf(1.0 - noise, SCATTER_Y_MIN, SCATTER_Y_MAX)) / yr * ph
	var bot0: float = mt + (SCATTER_Y_MAX - clampf(0.0 - noise, SCATTER_Y_MIN, SCATTER_Y_MAX)) / yr * ph
	var band: PackedVector2Array = PackedVector2Array()
	band.append(Vector2(dx0, top0))
	band.append(Vector2(dx1, top1))
	band.append(Vector2(dx1, bot1))
	band.append(Vector2(dx0, bot0))
	scatter_canvas.draw_colored_polygon(band, Color(0.55, 0.85, 1.00, 0.10))

	# y = x diagonal
	var dy0: float = mt + (SCATTER_Y_MAX - 0.0) / yr * ph
	var dy1: float = mt + (SCATTER_Y_MAX - 1.0) / yr * ph
	scatter_canvas.draw_line(
		Vector2(dx0, dy0), Vector2(dx1, dy1),
		Color(0.55, 0.85, 1.00, 0.55), 2.0
	)

	# Scatter points
	for i in range(max(_scatter.size() - 1, 0)):
		var p: Vector2 = _scatter[i]
		var px: float = ml + (p.x - SCATTER_X_MIN) / xr * pw
		var py: float = mt + (SCATTER_Y_MAX - p.y) / yr * ph
		scatter_canvas.draw_circle(Vector2(px, py), 3.0, Color(0.85, 0.85, 0.95, 0.55))

	# Current point highlighted
	if _scatter.size() > 0:
		var p: Vector2 = _scatter.back()
		var px: float = ml + (p.x - SCATTER_X_MIN) / xr * pw
		var py: float = mt + (SCATTER_Y_MAX - p.y) / yr * ph
		scatter_canvas.draw_circle(Vector2(px, py), 9.0, Color(1.0, 0.78, 0.30, 0.30))
		scatter_canvas.draw_circle(Vector2(px, py), 5.5, Color(1.0, 0.78, 0.30))

	# Axis ticks
	for tick in [0.0, 0.25, 0.5, 0.75, 1.0]:
		var tx: float = ml + (tick - SCATTER_X_MIN) / xr * pw
		scatter_canvas.draw_line(Vector2(tx, mt + ph), Vector2(tx, mt + ph + 4), axis_color, 1.0)
		var lab: String = "%.2f" % tick
		var ls: Vector2 = font.get_string_size(lab, HORIZONTAL_ALIGNMENT_CENTER, -1, fs)
		scatter_canvas.draw_string(font,
			Vector2(tx - ls.x * 0.5, mt + ph + ls.y + 4),
			lab, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.7, 0.7, 0.75))

	for tick in [0.0, 0.25, 0.5, 0.75, 1.0]:
		var ty: float = mt + (SCATTER_Y_MAX - tick) / yr * ph
		scatter_canvas.draw_line(Vector2(ml - 4, ty), Vector2(ml, ty), axis_color, 1.0)
		var lab: String = "%.2f" % tick
		scatter_canvas.draw_string(font,
			Vector2(ml - 38, ty + 4),
			lab, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.7, 0.7, 0.75))

	var xtitle: String = "k / n  (fraction of shared (symbol, time) pairs)"
	var xs: Vector2 = font.get_string_size(xtitle, HORIZONTAL_ALIGNMENT_CENTER, -1, fs)
	scatter_canvas.draw_string(font,
		Vector2(ml + pw * 0.5 - xs.x * 0.5, h - 6),
		xtitle, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.78, 0.78, 0.82))

	scatter_canvas.draw_string(font, Vector2(ml, 16),
		"cos(W1, W2)  vs  k / n", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.78, 0.78, 0.82))
	scatter_canvas.draw_string(font, Vector2(ml + pw - 260, 16),
		"diagonal y = x = predicted cos = k/n",
		HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.55, 0.85, 1.00, 0.85))
	scatter_canvas.draw_string(font, Vector2(ml + pw - 260, 30),
		"shaded band = ±1/√d = ±%.3f" % noise,
		HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(0.55, 0.85, 1.00, 0.55))
