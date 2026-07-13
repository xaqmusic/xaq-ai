extends Control

# View B — "The three HDC operations"
# Three stacked panels:
#   Binding (⊙)       — A ⊙ B is dissimilar to both, recoverable via ⊙B
#   Superposition (+) — sum is similar to every input, dissimilar to strangers
#   Permutation (roll) — rolled is dissimilar to original, recoverable by -k

const DIM: int = 128
const SUPER_MAX_K: int = 6
const PERM_MAX_K: int = 64

const PREV_SCENE: String = "res://scenes/hdc_demo_view_a.tscn"
const NEXT_SCENE: String = "res://scenes/hdc_demo_view_c.tscn"

# Binding state
var _bind_seed: int = 1001
var _bind_a: PackedFloat32Array = PackedFloat32Array()
var _bind_b: PackedFloat32Array = PackedFloat32Array()
var _bind_ab: PackedFloat32Array = PackedFloat32Array()
var _bind_recovered: PackedFloat32Array = PackedFloat32Array()

# Superposition state
var _super_seed: int = 2002
var _super_k: int = 4
var _super_inputs: Array = []  # Array[PackedFloat32Array]
var _super_sum: PackedFloat32Array = PackedFloat32Array()
var _super_query: PackedFloat32Array = PackedFloat32Array()

# Permutation state
var _perm_seed: int = 3003
var _perm_k: int = 16
var _perm_a: PackedFloat32Array = PackedFloat32Array()
var _perm_rolled: PackedFloat32Array = PackedFloat32Array()
var _perm_recovered: PackedFloat32Array = PackedFloat32Array()

@onready var bind_canvas: Control = $V/BindPanel/BindCanvas
@onready var bind_stats: Label = $V/BindPanel/BindStats
@onready var bind_regen: Button = $V/BindPanel/BindControl/BindRegen
@onready var super_canvas: Control = $V/SuperPanel/SuperCanvas
@onready var super_stats: Label = $V/SuperPanel/SuperStats
@onready var super_slider: HSlider = $V/SuperPanel/SuperControl/SuperSlider
@onready var super_value: Label = $V/SuperPanel/SuperControl/SuperValue
@onready var super_regen: Button = $V/SuperPanel/SuperControl/SuperRegen
@onready var perm_canvas: Control = $V/PermPanel/PermCanvas
@onready var perm_stats: Label = $V/PermPanel/PermStats
@onready var perm_slider: HSlider = $V/PermPanel/PermControl/PermSlider
@onready var perm_value: Label = $V/PermPanel/PermControl/PermValue
@onready var perm_regen: Button = $V/PermPanel/PermControl/PermRegen
@onready var prev_button: Button = $V/Nav/PrevButton
@onready var next_button: Button = $V/Nav/NextButton
@onready var nav_label: Label = $V/Nav/NavLabel


func _ready() -> void:
	super_slider.min_value = 2
	super_slider.max_value = SUPER_MAX_K
	super_slider.step = 1
	super_slider.value = _super_k

	perm_slider.min_value = 0
	perm_slider.max_value = PERM_MAX_K
	perm_slider.step = 1
	perm_slider.value = _perm_k

	bind_regen.pressed.connect(_on_bind_regen)
	super_slider.value_changed.connect(_on_super_changed)
	super_regen.pressed.connect(_on_super_regen)
	perm_slider.value_changed.connect(_on_perm_changed)
	perm_regen.pressed.connect(_on_perm_regen)

	bind_canvas.draw.connect(_draw_binding)
	bind_canvas.resized.connect(bind_canvas.queue_redraw)
	super_canvas.draw.connect(_draw_super)
	super_canvas.resized.connect(super_canvas.queue_redraw)
	perm_canvas.draw.connect(_draw_perm)
	perm_canvas.resized.connect(perm_canvas.queue_redraw)

	_setup_nav()
	_regenerate_binding()
	_regenerate_super()
	_regenerate_perm()


func _on_bind_regen() -> void:
	_bind_seed += 1
	_regenerate_binding()


func _on_super_changed(value: float) -> void:
	_super_k = int(value)
	_recompute_super()


func _on_super_regen() -> void:
	_super_seed += 1
	_regenerate_super()


func _on_perm_changed(value: float) -> void:
	_perm_k = int(value)
	_recompute_perm()


func _on_perm_regen() -> void:
	_perm_seed += 1
	_regenerate_perm()


func _regenerate_binding() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = _bind_seed
	_bind_a = _random_unit_vector(rng, DIM)
	_bind_b = _random_unit_vector(rng, DIM)
	_recompute_binding()


func _recompute_binding() -> void:
	_bind_ab = _multiply(_bind_a, _bind_b)
	_bind_recovered = _multiply(_bind_ab, _bind_b)
	var dot_a_ab: float = _dot(_bind_a, _bind_ab)
	var dot_b_ab: float = _dot(_bind_b, _bind_ab)
	var dot_rec_a: float = _dot(_bind_recovered, _bind_a)
	bind_stats.text = (
		"dot(A, A⊙B) = %+.4f      → A⊙B is dissimilar to A\n"
		+ "dot(B, A⊙B) = %+.4f      → A⊙B is dissimilar to B\n"
		+ "dot(A, (A⊙B)⊙B) = %+.4f  → multiplying by B again recovers A"
	) % [dot_a_ab, dot_b_ab, dot_rec_a]
	bind_canvas.queue_redraw()


func _regenerate_super() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = _super_seed
	_super_inputs.clear()
	for i in range(SUPER_MAX_K):
		_super_inputs.append(_random_unit_vector(rng, DIM))
	_super_query = _random_unit_vector(rng, DIM)
	_recompute_super()


func _recompute_super() -> void:
	_super_sum = PackedFloat32Array()
	_super_sum.resize(DIM)
	for j in range(DIM):
		_super_sum[j] = 0.0
	for i in range(_super_k):
		var v: PackedFloat32Array = _super_inputs[i]
		for j in range(DIM):
			_super_sum[j] += v[j]
	_normalize(_super_sum)

	var min_dot: float = INF
	var max_dot: float = -INF
	var sum_dot: float = 0.0
	for i in range(_super_k):
		var d: float = _dot(_super_sum, _super_inputs[i])
		if d < min_dot:
			min_dot = d
		if d > max_dot:
			max_dot = d
		sum_dot += d
	var avg_dot: float = sum_dot / float(_super_k)
	var query_dot: float = _dot(_super_sum, _super_query)

	super_value.text = "%d" % _super_k
	super_stats.text = (
		"M = normalize(A₁ + A₂ + ... + A_K),   K = %d\n"
		+ "dot(M, Aᵢ):  min = %+.4f    avg = %+.4f    max = %+.4f   (all positive → M carries every input)\n"
		+ "dot(M, Q) for random unrelated Q = %+.4f   (M does NOT contain Q)"
	) % [_super_k, min_dot, avg_dot, max_dot, query_dot]
	super_canvas.queue_redraw()


func _regenerate_perm() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = _perm_seed
	_perm_a = _random_unit_vector(rng, DIM)
	_recompute_perm()


func _recompute_perm() -> void:
	_perm_rolled = _roll(_perm_a, _perm_k)
	_perm_recovered = _roll(_perm_rolled, -_perm_k)
	var dot_a_rolled: float = _dot(_perm_a, _perm_rolled)
	var dot_a_rec: float = _dot(_perm_a, _perm_recovered)
	perm_value.text = "%d" % _perm_k
	perm_stats.text = (
		"k = %d\n"
		+ "dot(A, roll(A, k))         = %+.4f    → rolled vector is dissimilar to A\n"
		+ "dot(A, roll(roll(A,k), -k)) = %+.4f   → rolling by -k recovers A"
	) % [_perm_k, dot_a_rolled, dot_a_rec]
	perm_canvas.queue_redraw()


# --- Slideshow nav ---

func _setup_nav() -> void:
	nav_label.text = "View B — 2 of 5"
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


func _dot(a: PackedFloat32Array, b: PackedFloat32Array) -> float:
	var s: float = 0.0
	for i in range(a.size()):
		s += a[i] * b[i]
	return s


func _normalize(v: PackedFloat32Array) -> void:
	var ns: float = 0.0
	for i in range(v.size()):
		ns += v[i] * v[i]
	var inv: float = 1.0 / sqrt(maxf(ns, 1e-12))
	for i in range(v.size()):
		v[i] *= inv


func _multiply(a: PackedFloat32Array, b: PackedFloat32Array) -> PackedFloat32Array:
	var r: PackedFloat32Array = PackedFloat32Array()
	r.resize(a.size())
	for i in range(a.size()):
		r[i] = a[i] * b[i]
	return r


func _roll(v: PackedFloat32Array, k: int) -> PackedFloat32Array:
	var n: int = v.size()
	var k_mod: int = ((k % n) + n) % n
	var r: PackedFloat32Array = PackedFloat32Array()
	r.resize(n)
	for i in range(n):
		r[(i + k_mod) % n] = v[i]
	return r


# --- Drawing helpers ---

func _value_color(v: float, scale: float) -> Color:
	var t: float = clampf(v / scale, -1.0, 1.0)
	var bg: Color = Color(0.10, 0.11, 0.14)
	if t >= 0.0:
		return bg.lerp(Color(1.00, 0.55, 0.30), t)
	else:
		return bg.lerp(Color(0.30, 0.55, 1.00), -t)


func _draw_strip(c: Control, v: PackedFloat32Array, x: float, y: float, w: float, h: float) -> void:
	var n: int = v.size()
	if n == 0:
		return
	# Auto-scale to max |value| so every strip is visually vivid
	var max_abs: float = 1e-12
	for val in v:
		var a: float = absf(val)
		if a > max_abs:
			max_abs = a
	var cw: float = w / float(n)
	for i in range(n):
		var color: Color = _value_color(v[i], max_abs)
		c.draw_rect(Rect2(x + i * cw, y, cw, h), color)
	c.draw_rect(Rect2(x, y, w, h), Color(0.35, 0.35, 0.40), false, 1.0)


func _draw_binding() -> void:
	var size: Vector2 = bind_canvas.size
	if size.x < 40 or size.y < 40:
		return
	bind_canvas.draw_rect(Rect2(0, 0, size.x, size.y), Color(0.08, 0.09, 0.12))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12
	var lc: Color = Color(0.85, 0.86, 0.90)

	var label_w: float = 120.0
	var strip_x: float = label_w + 8.0
	var strip_w: float = size.x - strip_x - 16.0
	var strip_h: float = 20.0
	var row_spacing: float = 24.0
	var y0: float = 8.0
	if strip_w < 40.0:
		return

	var labels: Array[String] = ["A", "B", "A ⊙ B", "(A⊙B) ⊙ B"]
	var strips: Array = [_bind_a, _bind_b, _bind_ab, _bind_recovered]
	var colors: Array[Color] = [lc, lc, Color(0.55, 0.85, 1.00), Color(1.0, 0.78, 0.30)]
	for i in range(4):
		var y: float = y0 + i * row_spacing
		bind_canvas.draw_string(font, Vector2(8, y + strip_h * 0.5 + 4),
			labels[i], HORIZONTAL_ALIGNMENT_LEFT, -1, fs, colors[i])
		_draw_strip(bind_canvas, strips[i], strip_x, y, strip_w, strip_h)


func _draw_super() -> void:
	var size: Vector2 = super_canvas.size
	if size.x < 40 or size.y < 40:
		return
	super_canvas.draw_rect(Rect2(0, 0, size.x, size.y), Color(0.08, 0.09, 0.12))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12
	var lc: Color = Color(0.85, 0.86, 0.90)

	var label_w: float = 60.0
	var strip_x: float = label_w + 8.0
	var strip_w: float = size.x - strip_x - 16.0
	var strip_h: float = 16.0
	var row_spacing: float = 20.0
	var y0: float = 8.0
	if strip_w < 40.0:
		return

	# Input strips (K of them)
	for i in range(_super_k):
		var y: float = y0 + i * row_spacing
		super_canvas.draw_string(font, Vector2(8, y + strip_h * 0.5 + 4),
			"A%d" % (i + 1), HORIZONTAL_ALIGNMENT_LEFT, -1, fs, lc)
		_draw_strip(super_canvas, _super_inputs[i], strip_x, y, strip_w, strip_h)

	# Sum strip (with gap)
	var sum_y: float = y0 + _super_k * row_spacing + 10.0
	super_canvas.draw_string(font, Vector2(8, sum_y + strip_h * 0.5 + 4),
		"M = Σ", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(1.0, 0.78, 0.30))
	_draw_strip(super_canvas, _super_sum, strip_x, sum_y, strip_w, strip_h)


func _draw_perm() -> void:
	var size: Vector2 = perm_canvas.size
	if size.x < 40 or size.y < 40:
		return
	perm_canvas.draw_rect(Rect2(0, 0, size.x, size.y), Color(0.08, 0.09, 0.12))

	var font: Font = ThemeDB.fallback_font
	var fs: int = 12
	var lc: Color = Color(0.85, 0.86, 0.90)

	var label_w: float = 150.0
	var strip_x: float = label_w + 8.0
	var strip_w: float = size.x - strip_x - 16.0
	var strip_h: float = 20.0
	var row_spacing: float = 24.0
	var y0: float = 8.0
	if strip_w < 40.0:
		return

	var labels: Array[String] = ["A", "roll(A, k)", "roll(roll(A,k), -k)"]
	var strips: Array = [_perm_a, _perm_rolled, _perm_recovered]
	var colors: Array[Color] = [lc, Color(0.55, 0.85, 1.00), Color(1.0, 0.78, 0.30)]
	for i in range(3):
		var y: float = y0 + i * row_spacing
		perm_canvas.draw_string(font, Vector2(8, y + strip_h * 0.5 + 4),
			labels[i], HORIZONTAL_ALIGNMENT_LEFT, -1, fs, colors[i])
		_draw_strip(perm_canvas, strips[i], strip_x, y, strip_w, strip_h)
