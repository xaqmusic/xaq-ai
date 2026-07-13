extends Control

# View A — "Random vectors in N dimensions"
# Demonstrates that as dimension grows, random unit vectors become
# nearly orthogonal: the distribution of pairwise dot products tightens
# around zero with std dev ~ 1/sqrt(d).

const DIM_OPTIONS: Array[int] = [2, 4, 8, 16, 32, 64, 128, 256, 512, 768, 1024]
const BIN_COUNT: int = 80
const X_MIN: float = -1.0
const X_MAX: float = 1.0

const PREV_SCENE: String = ""
const NEXT_SCENE: String = "res://scenes/hdc_demo_view_b.tscn"

var dimension: int = 128
var sample_count: int = 1000
var rng_seed: int = 12345

var _dot_products: PackedFloat32Array = PackedFloat32Array()
var _bins: PackedInt32Array = PackedInt32Array()
var _mean: float = 0.0
var _std: float = 0.0
var _max_abs: float = 0.0

@onready var dim_slider: HSlider = $V/Controls/DimRow/DimSlider
@onready var dim_value: Label = $V/Controls/DimRow/DimValue
@onready var samples_slider: HSlider = $V/Controls/SamplesRow/SamplesSlider
@onready var samples_value: Label = $V/Controls/SamplesRow/SamplesValue
@onready var regen_button: Button = $V/Controls/ButtonsRow/Regenerate
@onready var stats_label: Label = $V/Stats
@onready var canvas: Control = $V/Canvas
@onready var prev_button: Button = $V/Nav/PrevButton
@onready var next_button: Button = $V/Nav/NextButton


func _ready() -> void:
	dim_slider.min_value = 0
	dim_slider.max_value = DIM_OPTIONS.size() - 1
	dim_slider.step = 1
	var initial_idx: int = DIM_OPTIONS.find(dimension)
	dim_slider.value = max(initial_idx, 0)

	samples_slider.min_value = 100
	samples_slider.max_value = 5000
	samples_slider.step = 100
	samples_slider.value = sample_count

	dim_slider.value_changed.connect(_on_dim_changed)
	samples_slider.value_changed.connect(_on_samples_changed)
	regen_button.pressed.connect(_on_regenerate)
	canvas.draw.connect(_on_canvas_draw)
	canvas.resized.connect(canvas.queue_redraw)

	_setup_nav()
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


func _on_dim_changed(value: float) -> void:
	var idx: int = int(value)
	dimension = DIM_OPTIONS[idx]
	_recompute()


func _on_samples_changed(value: float) -> void:
	sample_count = int(value)
	_recompute()


func _on_regenerate() -> void:
	rng_seed += 1
	_recompute()


func _recompute() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = rng_seed

	_dot_products.resize(sample_count)
	_bins.resize(BIN_COUNT)
	for i in range(BIN_COUNT):
		_bins[i] = 0

	var sum_dot: float = 0.0
	var sum_sq: float = 0.0
	var max_abs: float = 0.0
	var x_range: float = X_MAX - X_MIN

	for i in range(sample_count):
		var a: PackedFloat32Array = _random_unit_vector(rng, dimension)
		var b: PackedFloat32Array = _random_unit_vector(rng, dimension)
		var d: float = _dot(a, b)
		_dot_products[i] = d
		sum_dot += d
		sum_sq += d * d
		var ad: float = absf(d)
		if ad > max_abs:
			max_abs = ad

		var bin_idx: int = int(floor((d - X_MIN) / x_range * BIN_COUNT))
		bin_idx = clamp(bin_idx, 0, BIN_COUNT - 1)
		_bins[bin_idx] += 1

	_mean = sum_dot / float(sample_count)
	var variance: float = (sum_sq / float(sample_count)) - _mean * _mean
	_std = sqrt(maxf(variance, 0.0))
	_max_abs = max_abs

	var theoretical_std: float = 1.0 / sqrt(float(dimension))

	dim_value.text = "%d" % dimension
	samples_value.text = "%d" % sample_count
	stats_label.text = (
		"d = %d    n = %d\n"
		+ "observed:    mean dot = %+.4f    std dev = %.4f    max |dot| = %.3f\n"
		+ "theoretical: mean dot = +0.0000    std dev = %.4f  (1/√d)"
	) % [dimension, sample_count, _mean, _std, _max_abs, theoretical_std]

	canvas.queue_redraw()


func _random_unit_vector(rng: RandomNumberGenerator, d: int) -> PackedFloat32Array:
	# Sample d standard normals via Box-Muller, then normalize.
	var v: PackedFloat32Array = PackedFloat32Array()
	v.resize(d)
	var norm_sq: float = 0.0
	var i: int = 0
	while i < d:
		var u1: float = maxf(rng.randf(), 1e-12)
		var u2: float = rng.randf()
		var r: float = sqrt(-2.0 * log(u1))
		var theta: float = TAU * u2
		var x0: float = r * cos(theta)
		v[i] = x0
		norm_sq += x0 * x0
		if i + 1 < d:
			var x1: float = r * sin(theta)
			v[i + 1] = x1
			norm_sq += x1 * x1
		i += 2
	var inv_norm: float = 1.0 / sqrt(maxf(norm_sq, 1e-12))
	for j in range(d):
		v[j] *= inv_norm
	return v


func _dot(a: PackedFloat32Array, b: PackedFloat32Array) -> float:
	var s: float = 0.0
	var n: int = a.size()
	for i in range(n):
		s += a[i] * b[i]
	return s


func _on_canvas_draw() -> void:
	var size: Vector2 = canvas.size
	var w: float = size.x
	var h: float = size.y
	if w < 40.0 or h < 40.0:
		return

	# Plot background
	canvas.draw_rect(Rect2(0, 0, w, h), Color(0.10, 0.11, 0.14))

	var margin_left: float = 50.0
	var margin_right: float = 16.0
	var margin_top: float = 16.0
	var margin_bottom: float = 38.0
	var plot_w: float = w - margin_left - margin_right
	var plot_h: float = h - margin_top - margin_bottom
	if plot_w < 10.0 or plot_h < 10.0:
		return

	var bar_w: float = plot_w / float(BIN_COUNT)
	var x_range: float = X_MAX - X_MIN

	# Find max bin count for vertical normalization
	var max_count: int = 1
	for c in _bins:
		if c > max_count:
			max_count = c

	# Axis lines
	var axis_color: Color = Color(0.35, 0.35, 0.40)
	canvas.draw_line(
		Vector2(margin_left, margin_top),
		Vector2(margin_left, margin_top + plot_h),
		axis_color, 1.0
	)
	canvas.draw_line(
		Vector2(margin_left, margin_top + plot_h),
		Vector2(margin_left + plot_w, margin_top + plot_h),
		axis_color, 1.0
	)

	# Zero reference line (x = 0)
	var zero_x: float = margin_left + (0.0 - X_MIN) / x_range * plot_w
	canvas.draw_line(
		Vector2(zero_x, margin_top),
		Vector2(zero_x, margin_top + plot_h),
		Color(0.55, 0.55, 0.60, 0.35), 1.0
	)

	# Theoretical std-dev band: ±1/sqrt(d) around zero
	var theo_std: float = 1.0 / sqrt(float(dimension))
	var band_left: float = margin_left + (-theo_std - X_MIN) / x_range * plot_w
	var band_right: float = margin_left + (theo_std - X_MIN) / x_range * plot_w
	band_left = clampf(band_left, margin_left, margin_left + plot_w)
	band_right = clampf(band_right, margin_left, margin_left + plot_w)
	if band_right > band_left:
		canvas.draw_rect(
			Rect2(band_left, margin_top, band_right - band_left, plot_h),
			Color(0.30, 0.65, 1.00, 0.08)
		)

	# Bars
	for i in range(BIN_COUNT):
		var count: int = _bins[i]
		if count <= 0:
			continue
		var bar_h: float = (float(count) / float(max_count)) * plot_h
		var x: float = margin_left + i * bar_w
		var y: float = margin_top + plot_h - bar_h
		var bin_center: float = X_MIN + (i + 0.5) / float(BIN_COUNT) * x_range
		var t: float = clampf(absf(bin_center), 0.0, 1.0)
		# Color: bluish near 0, fading toward muted at ±1
		var bar_color: Color = Color(0.30, 0.65 + 0.25 * (1.0 - t), 1.00 - 0.30 * t, 0.95)
		canvas.draw_rect(Rect2(x, y, maxf(bar_w - 1.0, 1.0), bar_h), bar_color)

	# X-axis ticks
	var font: Font = ThemeDB.fallback_font
	var font_size: int = 12
	var tick_color: Color = Color(0.75, 0.75, 0.80)
	for tick in [-1.0, -0.5, 0.0, 0.5, 1.0]:
		var tx: float = margin_left + (tick - X_MIN) / x_range * plot_w
		var ty: float = margin_top + plot_h
		canvas.draw_line(Vector2(tx, ty), Vector2(tx, ty + 4), axis_color, 1.0)
		var label_text: String = "%+.1f" % tick if tick != 0.0 else "0.0"
		var label_size: Vector2 = font.get_string_size(label_text, HORIZONTAL_ALIGNMENT_CENTER, -1, font_size)
		canvas.draw_string(
			font,
			Vector2(tx - label_size.x * 0.5, ty + label_size.y + 4),
			label_text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, tick_color
		)

	# Y-axis label (max count)
	var max_text: String = "n = %d" % max_count
	canvas.draw_string(
		font,
		Vector2(8.0, margin_top + 12.0),
		max_text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(0.70, 0.70, 0.75)
	)

	# X-axis title
	var xtitle: String = "pairwise dot product  (= cosine similarity for unit vectors)"
	var xsize: Vector2 = font.get_string_size(xtitle, HORIZONTAL_ALIGNMENT_CENTER, -1, font_size)
	canvas.draw_string(
		font,
		Vector2(margin_left + plot_w * 0.5 - xsize.x * 0.5, h - 6.0),
		xtitle, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(0.78, 0.78, 0.82)
	)

	# Theoretical band caption
	var band_caption: String = "shaded band = ±1/√d = ±%.3f" % theo_std
	canvas.draw_string(
		font,
		Vector2(margin_left + 8.0, margin_top + 14.0),
		band_caption, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(0.55, 0.78, 1.00, 0.85)
	)
