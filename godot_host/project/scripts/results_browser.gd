extends Control
## Phase 6.5.3.B — read-only browser over the project's results/ tree.
##
## Discovers:
##   results/<sweep_dir>/manifest.json   — CartPole / MountainCar sweeps
##                                          (cartpole_run.py / mountain_car_run.py)
##   results/<orch_dir>/manifest.json    — Phase 6.5.1 orchestrator output
##   results/*.json                      — single-file paired-seed
##                                          (paired_seed_ab.py output)
##
## Normalises each into an "ExperimentRecord" with kind/env/headline
## fields, displays them sortable by date.  Click a row to see details
## (per-seed breakdowns, paired-Δ stats, etc.).
##
## Goal: community reproducibility.  Anyone with the repo can launch
## the project, hit "Browse past experiments", and walk every
## experiment we've documented in chronological order.

const _BACK_TO_LAUNCHER := "res://scenes/launcher.tscn"

const _ENV_TO_SCENE := {
	"cell":         "res://scenes/the_cell.tscn",
	"cartpole":     "res://scenes/the_cartpole.tscn",
	"mountain_car": "res://scenes/the_mountain_car.tscn",
}

# Resolve absolute path to the repo's results/ directory.  res:// points
# at godot_host/project/; results/ is two levels up.
var _results_dir: String = ""

@onready var _list:        ItemList         = $H/Left/V/List
@onready var _details:     RichTextLabel    = $H/Right/V/Details
@onready var _env_filter:  OptionButton     = $H/Left/V/Filter/EnvFilter
@onready var _kind_filter: OptionButton     = $H/Left/V/Filter/KindFilter
@onready var _back_btn:    Button           = $TopBar/H/Back
@onready var _refresh_btn: Button           = $TopBar/H/Refresh
@onready var _count_label: Label            = $TopBar/H/CountLabel
# Reproduce panel (lives below the details text in the right column).
@onready var _repro_panel:    VBoxContainer = $H/Right/V/Reproduce
@onready var _repro_seed_box: OptionButton  = $H/Right/V/Reproduce/SeedRow/SeedDropdown
@onready var _repro_status:   Label         = $H/Right/V/Reproduce/Status
@onready var _repro_buttons:  HBoxContainer = $H/Right/V/Reproduce/Buttons

# All discovered records.  Keys: id, kind, env, timestamp, git_sha,
# n_seeds, headline, details_text, sort_key (for date sort).
var _records: Array[Dictionary] = []
var _filtered_indices: Array[int] = []

func _ready() -> void:
	_results_dir = ProjectSettings.globalize_path("res://").path_join("../../results/").simplify_path()
	_populate_filters()
	_back_btn.pressed.connect(_on_back)
	_refresh_btn.pressed.connect(_scan_and_refresh)
	_env_filter.item_selected.connect(_on_filter_changed)
	_kind_filter.item_selected.connect(_on_filter_changed)
	_list.item_selected.connect(_on_item_selected)
	_scan_and_refresh()

func _on_back() -> void:
	get_tree().change_scene_to_file(_BACK_TO_LAUNCHER)

# -- Discovery + normalisation ------------------------------------------------

func _scan_and_refresh() -> void:
	print("results_browser: scanning ", _results_dir)
	_records.clear()
	if not DirAccess.dir_exists_absolute(_results_dir):
		_count_label.text = "results/ not found at " + _results_dir
		print("results_browser: dir does not exist!")
		return
	var entries := DirAccess.get_directories_at(_results_dir)
	print("results_browser: %d subdirs, %d top-level files" % [
		entries.size(), DirAccess.get_files_at(_results_dir).size()
	])
	for sub in entries:
		var sub_path: String = _results_dir.path_join(sub)
		var manifest_path: String = sub_path.path_join("manifest.json")
		if FileAccess.file_exists(manifest_path):
			var rec = _parse_dir_manifest(sub, manifest_path, sub_path)
			if rec != null: _records.append(rec)
		# Also scan paired-seed files INSIDE this subdir — so each cell
		# from a Phase 6.5.1 orchestrator sweep (A_headline.json,
		# B_obstacle.json, ...) becomes individually reproducible.
		for fname in DirAccess.get_files_at(sub_path):
			if not fname.ends_with(".json"): continue
			if fname == "manifest.json": continue
			# Skip per-seed cartpole/MC files (named seedNN_arm.json)
			if fname.begins_with("seed"): continue
			var inner_path: String = sub_path.path_join(fname)
			var rec2 = _parse_top_level_json("%s/%s" % [sub, fname], inner_path)
			if rec2 != null: _records.append(rec2)
	# Top-level paired-seed JSONs (one file per experiment).
	for fname in DirAccess.get_files_at(_results_dir):
		if not fname.ends_with(".json"): continue
		var fpath: String = _results_dir.path_join(fname)
		var rec = _parse_top_level_json(fname, fpath)
		if rec != null: _records.append(rec)
	# Sort by timestamp descending (most recent first).
	_records.sort_custom(_sort_by_ts_desc)
	print("results_browser: %d records discovered" % _records.size())
	_apply_filter()

func _sort_by_ts_desc(a: Dictionary, b: Dictionary) -> bool:
	return str(a.get("sort_key", "")) > str(b.get("sort_key", ""))

func _parse_dir_manifest(id: String, mpath: String, dir_path: String) -> Variant:
	var f := FileAccess.open(mpath, FileAccess.READ)
	if f == null: return null
	var parsed = JSON.parse_string(f.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY: return null
	var d: Dictionary = parsed
	# Detect format:
	# A) cartpole/mc sweep: has 'arm' and 'mean_eval_arm'
	# B) phase6_5_1 orchestrator: has 'cells' (array of cell summaries)
	if d.has("arm") and d.has("mean_eval_arm"):
		return _record_from_sweep(id, d, dir_path)
	if d.has("cells"):
		return _record_from_orchestrator(id, d, dir_path)
	# Unknown manifest format — skip gracefully.
	return null

func _record_from_sweep(id: String, d: Dictionary, dir_path: String) -> Dictionary:
	var arm: String         = str(d.get("arm", ""))
	var env: String         = _env_from_id(id, dir_path, arm)
	var n_seeds: int        = int(d.get("n_seeds", 0))
	var n_solved: int       = int(d.get("n_solved", 0))
	var n_ok: int           = int(d.get("n_ok", 0))
	var mean_eval: float    = float(d.get("mean_eval_arm", 0.0))
	var stdev_eval: float   = float(d.get("stdev_eval_arm", 0.0))
	var max_eval: float     = float(d.get("max_eval_arm", 0.0))
	var min_eval: float     = float(d.get("min_eval_arm", 0.0))
	var ts: String          = str(d.get("timestamp_utc", _ts_from_id(id)))
	var wall_s: float       = float(d.get("wall_s", 0.0))
	var headline := "%s arm=%s  μ=%.2f σ=%.2f  solved=%d/%d" % [
		env, arm, mean_eval, stdev_eval, n_solved, n_ok
	]
	var details := _format_sweep_details(id, d, dir_path)
	# Reproduce metadata: config path + episode params + recorded seeds
	# so the user can re-launch the same experiment from the browser.
	var seeds: Array = []
	for s in d.get("summaries", []):
		seeds.append(int(s.get("seed", 0)))
	var reproduce := [{
		"label":        "Run this %s arm" % arm,
		"scene_path":   _ENV_TO_SCENE.get(env, ""),
		"config_path":  str(d.get("config", "")),
		"max_episodes": int(d.get("total_episodes", 0)),
		"seeds":        seeds,
		"body_env":     {},   # CartPole/MC don't carry per-body env vars
	}]
	return {
		"id":           id,
		"kind":         "sweep",
		"env":          env,
		"timestamp":    ts,
		"sort_key":     ts if ts != "" else _ts_from_id(id),
		"git_sha":      "",
		"n_seeds":      n_seeds,
		"headline":     headline,
		"details_text": details,
		"reproduce":    reproduce,
	}

func _record_from_orchestrator(id: String, d: Dictionary, dir_path: String) -> Dictionary:
	var cells: Array        = d.get("cells", [])
	var git_sha: String     = str(d.get("git_sha", ""))
	var wall_s: float       = float(d.get("wall_s_total", d.get("wall_s_so_far", 0.0)))
	var ts: String          = _ts_from_id(id)
	var lines: Array[String] = []
	lines.append("Phase 6.5.1 orchestrator — Cell paired sweep")
	lines.append("git: %s   wall: %.1f min   cells: %d" % [git_sha, wall_s/60.0, cells.size()])
	lines.append("")
	for c in cells:
		lines.append("  %-14s  seeds=%s  duration=%ss  ok=%s" % [
			str(c.get("id","?")),
			str(c.get("seeds","?")),
			str(c.get("duration","?")),
			str(c.get("ok","?")),
		])
	# Headline: also try to read the per-cell paired Δhits/min from the
	# A_headline.json (most statistically powerful cell typically).
	var headline_extra := ""
	var head_path: String = dir_path.path_join("A_headline.json")
	if FileAccess.file_exists(head_path):
		var pf := FileAccess.open(head_path, FileAccess.READ)
		if pf:
			var pp = JSON.parse_string(pf.get_as_text())
			if typeof(pp) == TYPE_DICTIONARY:
				var dh: Dictionary = pp.get("delta_hits_per_min", {})
				headline_extra = "  A: μΔ=%+.3f p=%.3f n=%d" % [
					float(dh.get("mean", 0.0)),
					float(dh.get("p_normal", 0.0)),
					int(pp.get("n_paired", 0)),
				]
	return {
		"id":           id,
		"kind":         "orchestrator",
		"env":          "cell",
		"timestamp":    ts,
		"sort_key":     ts,
		"git_sha":      git_sha,
		"n_seeds":      0,
		"headline":     "Cell sweep — %d cells %s" % [cells.size(), headline_extra],
		"details_text": "\n".join(lines),
	}

func _parse_top_level_json(fname: String, fpath: String) -> Variant:
	var f := FileAccess.open(fpath, FileAccess.READ)
	if f == null: return null
	var parsed = JSON.parse_string(f.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY: return null
	var d: Dictionary = parsed
	# Paired-seed format detector.
	if not (d.has("baseline_results") and d.has("variant_results")): return null
	return _record_from_paired(fname, d)

func _record_from_paired(fname: String, d: Dictionary) -> Dictionary:
	var label: String       = str(d.get("label", fname.trim_suffix(".json")))
	var ts: String          = str(d.get("timestamp_utc", _ts_from_id(fname)))
	var git_sha: String     = str(d.get("git_sha", ""))
	var n: int              = int(d.get("n_paired", 0))
	var dh: Dictionary      = d.get("delta_hits_per_min", {})
	var mean_d: float       = float(dh.get("mean", 0.0))
	var sd_d: float         = float(dh.get("stdev", 0.0))
	var p: float            = float(dh.get("p_normal", 1.0))
	var ci_lo: float        = float(dh.get("ci_lo", 0.0))
	var ci_hi: float        = float(dh.get("ci_hi", 0.0))
	var lb: String          = str(d.get("label_baseline", "baseline"))
	var lv: String          = str(d.get("label_variant", "variant"))
	var headline := "Cell paired (%s vs %s)  n=%d  μΔ=%+.3f hits/min  p=%.3f  CI=[%+.2f, %+.2f]" % [
		lb, lv, n, mean_d, p, ci_lo, ci_hi
	]
	var details := _format_paired_details(d)
	var seeds: Array = []
	for s in d.get("seeds", []):
		seeds.append(int(s))
	# Both arms get a reproduce entry — baseline first, variant second.
	var dur: int = int(d.get("duration_s", 0))
	var reproduce := [
		{
			"label":        "Run baseline arm  (%s)" % lb,
			"scene_path":   _ENV_TO_SCENE["cell"],
			"config_path":  str(d.get("baseline_config", "")),
			"max_episodes": 0,    # Cell is continuous; user picks Episodes in launcher (=0)
			"duration_s":   dur,
			"seeds":        seeds,
			"body_env":     _parse_env_string(str(d.get("baseline_env", ""))),
		},
		{
			"label":        "Run variant arm  (%s)" % lv,
			"scene_path":   _ENV_TO_SCENE["cell"],
			"config_path":  str(d.get("variant_config", "")),
			"max_episodes": 0,
			"duration_s":   dur,
			"seeds":        seeds,
			"body_env":     _parse_env_string(str(d.get("variant_env", ""))),
		},
	]
	return {
		"id":           fname.trim_suffix(".json"),
		"kind":         "paired",
		"env":          "cell",
		"timestamp":    ts,
		"sort_key":     ts if ts != "" else _ts_from_id(fname),
		"git_sha":      git_sha,
		"n_seeds":      n,
		"headline":     headline,
		"details_text": details,
		"reproduce":    reproduce,
	}

# Parse "OGMA_BODY_MODEL=differential_paddler,OGMA_OBSTACLE_DENSITY=0.3"
# into a dict the launcher can consume.  Used to recover Cell-side
# body params (paddler model, obstacle density, terrain amplitude)
# from a paired-seed JSON's recorded env strings.
func _parse_env_string(s: String) -> Dictionary:
	var out := {}
	if s == "":
		return out
	for kv in s.split(","):
		var parts := kv.strip_edges().split("=", true, 1)
		if parts.size() == 2:
			out[parts[0]] = parts[1]
	return out

# -- Detail formatters --------------------------------------------------------

func _format_sweep_details(id: String, d: Dictionary, dir_path: String) -> String:
	var lines: Array[String] = []
	lines.append("Sweep: %s" % id)
	lines.append("config: %s" % d.get("config", "?"))
	lines.append("arm: %s   total_episodes: %s   eval_window: %s" % [
		d.get("arm","?"), d.get("total_episodes","?"), d.get("eval_window","?")
	])
	lines.append("max_steps: %s   wall: %.1fs" % [
		d.get("max_steps","?"), float(d.get("wall_s", 0.0))
	])
	lines.append("")
	lines.append("Headline:")
	lines.append("  μ=%.2f σ=%.2f  range=[%.2f, %.2f]  solved=%d/%d" % [
		float(d.get("mean_eval_arm",0.0)), float(d.get("stdev_eval_arm",0.0)),
		float(d.get("min_eval_arm",0.0)), float(d.get("max_eval_arm",0.0)),
		int(d.get("n_solved",0)), int(d.get("n_seeds",0)),
	])
	lines.append("")
	lines.append("Per-seed:")
	for s in d.get("summaries", []):
		lines.append("  seed=%-3s  μ_eval=%7.1f  μ_train=%7.1f  max=%-5s  solved=%s" % [
			str(s.get("seed","?")),
			float(s.get("mean_eval", 0.0)),
			float(s.get("mean_train", 0.0)),
			str(s.get("max_eval","?")),
			"Y" if bool(s.get("solved_eval", false)) else "N",
		])
	return "\n".join(lines)

func _format_paired_details(d: Dictionary) -> String:
	var lines: Array[String] = []
	var label: String  = str(d.get("label", "paired"))
	var lb: String     = str(d.get("label_baseline", "baseline"))
	var lv: String     = str(d.get("label_variant", "variant"))
	lines.append("Paired-seed comparison: %s" % label)
	lines.append("baseline: %s" % d.get("baseline_config", "?"))
	lines.append("variant:  %s" % d.get("variant_config", "?"))
	lines.append("git: %s   duration/seed: %ss   wall: %.1fs" % [
		str(d.get("git_sha","")), str(d.get("duration_s","?")),
		float(d.get("wall_seconds", 0.0)),
	])
	lines.append("seeds: %s" % str(d.get("seeds", [])))
	lines.append("")
	var dh: Dictionary = d.get("delta_hits_per_min", {})
	lines.append("Δhits/min: μ=%+.3f σ=%.3f  t=%+.3f  p=%.4f  95%% CI=[%+.3f, %+.3f]" % [
		float(dh.get("mean", 0.0)),
		float(dh.get("stdev", 0.0)),
		float(dh.get("t", 0.0)),
		float(dh.get("p_normal", 0.0)),
		float(dh.get("ci_lo", 0.0)),
		float(dh.get("ci_hi", 0.0)),
	])
	# Per-seed table
	lines.append("")
	lines.append("Per-seed:")
	var br: Array = d.get("baseline_results", [])
	var vr: Array = d.get("variant_results", [])
	var deltas_pm: Array = d.get("deltas_hits_per_min", [])
	for i in range(min(br.size(), vr.size())):
		var b = br[i]; var v = vr[i]
		var seed: String = str(b.get("seed", "?"))
		var bhpm: float = float(b.get("hits_per_min", 0.0))
		var vhpm: float = float(v.get("hits_per_min", 0.0))
		var dpm: float  = (float(deltas_pm[i]) if i < deltas_pm.size() else (vhpm - bhpm))
		lines.append("  seed=%-4s  %s=%5.2f  %s=%5.2f  Δ=%+5.2f hits/min" % [
			seed, lb, bhpm, lv, vhpm, dpm
		])
	return "\n".join(lines)

# -- Helpers ------------------------------------------------------------------

func _env_from_id(id: String, dir_path: String, arm_hint: String) -> String:
	var lid: String = id.to_lower()
	if lid.begins_with("cp_") or lid.begins_with("cartpole"):
		return "cartpole"
	if lid.begins_with("mc_") or lid.begins_with("mountain_car"):
		return "mountain_car"
	if lid.begins_with("phase6_5_1"):
		return "cell"
	# Fallback: peek at config path if present in arm_hint.
	if "cartpole" in arm_hint:
		return "cartpole"
	if "mountain_car" in arm_hint:
		return "mountain_car"
	return "unknown"

# Extract a sortable timestamp from typical id formats:
#   "cp_minimal_221640" → "_221640" (HHMMSS today)
#   "cartpole_20260502_212735" → 20260502_212735
#   "phase6_5_1_20260502_174111" → 20260502_174111
#   "mountain_car_20260503_021719" → 20260503_021719
func _ts_from_id(id: String) -> String:
	# Find first occurrence of an 8-digit YYYYMMDD followed by _HHMMSS.
	var re := RegEx.new()
	re.compile("(\\d{8}_\\d{6}|\\d{6}_\\d{6}|\\d{6})")
	var m := re.search(id)
	if m: return m.get_string()
	return id

# -- Filter + UI -------------------------------------------------------------

func _populate_filters() -> void:
	_env_filter.clear()
	_env_filter.add_item("All envs")
	_env_filter.add_item("Cell")
	_env_filter.set_item_metadata(1, "cell")
	_env_filter.add_item("CartPole")
	_env_filter.set_item_metadata(2, "cartpole")
	_env_filter.add_item("MountainCar")
	_env_filter.set_item_metadata(3, "mountain_car")
	_kind_filter.clear()
	_kind_filter.add_item("All types")
	_kind_filter.add_item("Sweep")
	_kind_filter.set_item_metadata(1, "sweep")
	_kind_filter.add_item("Paired-seed")
	_kind_filter.set_item_metadata(2, "paired")
	_kind_filter.add_item("Orchestrator")
	_kind_filter.set_item_metadata(3, "orchestrator")

func _on_filter_changed(_idx: int) -> void:
	_apply_filter()

func _apply_filter() -> void:
	_filtered_indices.clear()
	_list.clear()
	var env_meta: Variant = _env_filter.get_item_metadata(_env_filter.selected)
	var kind_meta: Variant = _kind_filter.get_item_metadata(_kind_filter.selected)
	for i in range(_records.size()):
		var r: Dictionary = _records[i]
		if typeof(env_meta) == TYPE_STRING and r.get("env","") != env_meta:
			continue
		if typeof(kind_meta) == TYPE_STRING and r.get("kind","") != kind_meta:
			continue
		_filtered_indices.append(i)
		var ts_str: String = str(r.get("sort_key", "")).left(15)
		_list.add_item("%-15s  %s" % [ts_str, r.get("headline", "")])
	_count_label.text = "%d / %d records" % [_filtered_indices.size(), _records.size()]
	_details.text = ""

func _on_item_selected(idx: int) -> void:
	if idx < 0 or idx >= _filtered_indices.size(): return
	var rec: Dictionary = _records[_filtered_indices[idx]]
	_details.text = str(rec.get("details_text", ""))
	_populate_reproduce_panel(rec)

# -- Reproduce panel ---------------------------------------------------------

var _current_seeds: Array = []   # seeds available for the currently-selected record

func _populate_reproduce_panel(rec: Dictionary) -> void:
	# Clear existing buttons (keep seed dropdown / status label).
	for c in _repro_buttons.get_children():
		c.queue_free()

	var repro_list: Array = rec.get("reproduce", [])
	if repro_list.is_empty():
		_repro_panel.visible = false
		return
	_repro_panel.visible = true

	# Populate seed dropdown from any reproduce entry's seeds (they
	# share the same seed list across baseline/variant arms).
	_current_seeds = repro_list[0].get("seeds", [])
	_repro_seed_box.clear()
	_repro_seed_box.add_item("Random per launch")
	_repro_seed_box.set_item_metadata(0, -1)
	for s in _current_seeds:
		_repro_seed_box.add_item("seed = %s" % str(s))
		_repro_seed_box.set_item_metadata(_repro_seed_box.item_count - 1, int(s))
	_repro_seed_box.select(0)

	# Build a Launch button per reproduce entry (1 for sweep, 2 for paired).
	var any_missing := false
	for i in range(repro_list.size()):
		var entry: Dictionary = repro_list[i]
		var btn := Button.new()
		btn.text = entry.get("label", "Launch")
		btn.custom_minimum_size = Vector2(220, 36)
		var cfg: String = entry.get("config_path", "")
		var cfg_exists: bool = cfg != "" and FileAccess.file_exists(cfg)
		btn.disabled = not cfg_exists
		if not cfg_exists:
			any_missing = true
		# Capture entry by value via lambda.
		var entry_copy: Dictionary = entry
		btn.pressed.connect(func(): _on_reproduce_launch(entry_copy))
		_repro_buttons.add_child(btn)

	if any_missing:
		_repro_status.text = "(one or more configs no longer exist on disk — those buttons are disabled)"
		_repro_status.modulate = Color(1.0, 0.7, 0.4, 1.0)
	else:
		_repro_status.text = "Configs present on disk — launch will use the recorded brain config and body env."
		_repro_status.modulate = Color(0.7, 0.85, 0.7, 1.0)

func _on_reproduce_launch(entry: Dictionary) -> void:
	var scene: String = entry.get("scene_path", "")
	if scene == "":
		push_error("results_browser: reproduce entry missing scene_path")
		return
	# Populate ExperimentConfig from the recorded entry.  Mirrors what
	# the launcher does for new experiments — same fall-through chain
	# applies in body controllers (ExperimentConfig > env var > export).
	ExperimentConfig.scene_path  = scene
	ExperimentConfig.config_path = entry.get("config_path", "")
	ExperimentConfig.max_episodes = int(entry.get("max_episodes", 0))

	# Seed: dropdown index 0 is "Random per launch" (-1); anything else
	# is a recorded seed.
	var sel_meta = _repro_seed_box.get_item_metadata(_repro_seed_box.selected)
	if typeof(sel_meta) == TYPE_INT and int(sel_meta) >= 0:
		ExperimentConfig.seed_value = int(sel_meta)
	else:
		ExperimentConfig.seed_value = randi() % 1000000

	# Body env vars (Cell-side: paddler, refractory, obstacle, terrain).
	var body_env: Dictionary = entry.get("body_env", {})
	if body_env.has("OGMA_BODY_MODEL"):
		ExperimentConfig.body_model = str(body_env["OGMA_BODY_MODEL"])
	if body_env.has("OGMA_REFRACTORY_TICKS"):
		ExperimentConfig.refractory_ticks = int(str(body_env["OGMA_REFRACTORY_TICKS"]))
	if body_env.has("OGMA_OBSTACLE_DENSITY"):
		ExperimentConfig.obstacle_density = float(str(body_env["OGMA_OBSTACLE_DENSITY"]))
	if body_env.has("OGMA_TERRAIN_AMPLITUDE"):
		ExperimentConfig.terrain_amplitude = float(str(body_env["OGMA_TERRAIN_AMPLITUDE"]))

	ExperimentConfig.launched = true
	ExperimentConfig.save_state()

	get_tree().change_scene_to_file(scene)
