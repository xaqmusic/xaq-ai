extends Control
## Phase 6.5.3.A — experiment launcher dialog.
##
## Discovers brain configs by scanning addons/ami_ogma/configs/*.json,
## reads the metadata block to populate per-environment dropdowns, and
## writes the user's selection to ExperimentConfig (autoload) before
## change_scene_to_file() into the chosen world scene.
##
## All selections persist across sessions via
## ExperimentConfig.save_state() / load_state().

const _CONFIG_DIR := "res://addons/ami_ogma/configs/"

# Curated picrawler launcher list (2026-06-12, Motor-EPM phase).  The configs dir
# accumulates dozens of historical arms; the launcher shows only this allowlist
# for env_target="picrawler" so the dropdown reflects the CURRENT phase plus a
# couple of reference baselines.  Other env_targets are unfiltered.  To surface
# a config in the launcher, add its exact .json filename here.  Files are NOT
# moved/deleted — every config stays loadable by path for scripts + A/B runs.
const _PICRAWLER_CONFIG_ALLOWLIST: Array = [
	# 2026-07-25 — DEMO-CLEAN: narrowed to the current config + one baseline, so the
	# dropdown is presentable and an A/B is unambiguous.  Everything else stays
	# loadable by path (scripts, seedavg/humpavg/recoveravg arms) and in git —
	# nothing is deleted.  To restore one to the dropdown, uncomment it.
	"the_picrawler_motor_epm_embed_corridor_stancelift.json",  # ── BASELINE ── the deployed stack: heading P+7 · progress-commit · Markov belly-rangefinder height · stance_lift 0.5 (belly-up). Clears the hump; net_z 3.62, straight 0.67, tilt_sd 0.078, 0 falls.
	"the_picrawler_motor_epm_embed_corridor_imufused.json",  # ── CURRENT ── the fully hardware-honest stack (commanded-angle IK ⊕ gyro-fused IMU attitude). net_z 4.75, flat_v 0.05, straight 0.74, tilt_sd 0.068, hump 6.09, 0 falls. KNOWN FLAW, documented and unfixed: an inverted episode latches height_bias and amp_gain, and the walk stays degraded for ~200 s after self-righting (ledger §2).
	# "the_picrawler_motor_epm_embed_corridor_uprightgate.json",  # UNRESOLVED TRADE — the upright gate stops the latch (amp_gain 2.53→0.100, height_bias +1.50→−0.09, normal walking byte-identical) BUT self-righted 0/8 vs ungated 1/8. amp_gain winding 40× IS the flail amplitude that rights the robot, so freezing it may disable the escape. Do not promote until resolved.
	# "the_picrawler_motor_epm_embed_corridor_gravcmd.json",  # same but EXACT sim attitude — uncomment to A/B what the honest IMU model costs (it gains). HARDWARE-HONEST: feet_topic = FK from COMMANDED servo angles ⊕ accelerometer gravity-up. Buildable on the real PiCrawler (hobby servos report no position, so commanded angles are all it has). Servo deflection measured at 22mm mean/38mm max; FK validated to 1.1mm. THE HARDWARE'S POORER INFORMATION IS THE BETTER SIGNAL: net_z 3.76 oracle → 3.95 achieved → 4.36 commanded (±0.28); straight 0.70 → 0.62 → 0.74 (±0.01); hump final_z 5.21 → 5.35 → 6.10; 0 falls. Deflection is noise to the gate. STILL UNMODELLED: attitude uses the exact sim transform, not gyro/accel fusion.
	# "the_picrawler_motor_epm_embed_corridor_gravfeet.json",  # same but FK from ACHIEVED pose (sim-privileged) — uncomment to A/B the sim-to-real gap. ORACLE REMOVED. feet_topic = feet_y_gravity (encoder-FK foot position · accelerometer gravity-up = IK ⊕ IMU) replacing feet_y, which was absolute WORLD-Y and unsensable by a real robot. n=4: net_z 3.76→3.95 and flat_v 0.04→0.05 — the FIRST flat-speed movement across 10+ levers; step_bal 0.25→0.42, steps 74→113. Hump gate improves (5.21→5.35, 0 falls). Costs straightness (0.70→0.62) and wobble (0.067→0.095) — livelier, less controlled. Three other legal signals lost badly and two phase hypotheses died at chance; gravity reference was the missing property.
	# "the_picrawler_motor_epm_embed_corridor_rewardfree.json",  # the ORACLE arm — identical but feet_topic=feet_y (absolute world-Y). Uncomment to A/B the legitimacy question directly. baseline + coord_adapt 0.001 + the coordination search made REWARD-FREE (coord_fitness_mode=1): probes ranked by coherence·activity/(1+tle) instead of forward velocity. No distance/velocity term anywhere. vs the fwd_v-reward arm: net_z 3.52→3.76, straight 0.67→0.70, tilt_sd 0.069→0.067, hump gate IDENTICAL (5.21), and coordination RECOVERS BETTER after perturbation (0.30→0.48) — a thrash can't win, it has high tle and low coherence. Removing the reward cost nothing; removing the search entirely costs net_z→2.36.
	# --- 2026-07-27 UI OBSERVATION SET (arena gym; [1]/[2] swap gyms live) ---
	# Neither lever is promoted; both are here to be EYEBALLED, because the metrics
	# alone are ambiguous and this campaign has repeatedly had metrics certify
	# behaviour the operator rejected on sight.  Load the CONTROL first so the
	# comparison is against a matched arm (same instruments) rather than the
	# corridor-default CURRENT entry.
	"the_picrawler_motor_epm_arena_control.json",  # ── ARENA CONTROL ── deployed stack + measurement instruments, NO lever. Verified behaviourally inert (identical seedavg on all 17 metrics, per seed, at 6000 and 12000 ticks), so any visible difference vs the two arms below is the lever. n=3: net_disp 4.85, straight 0.71, tilt_sd 0.088, steps 25, step_bal 0.07, belly 0.0221, 0 falls. The sprawl to look for: hip2 never leaves neutral, tibia 37.5° off vertical (design rest 10°), feet planted at 170mm against a 166mm total leg reach, and scrub 0.100 vs fwd_v 0.050 (sliding sideways twice as fast as it advances).
	"the_picrawler_motor_epm_arena_ik_plumb.json",  # ── ARENA · IK ── tibia_plumb_gain=0.15. hip2 nulls the shank's deviation from vertical so the knee's drive TRANSLATES the foot instead of arcing it (hip2+knee are a planar 2-link arm; one joint of a 2-DOF pair forces a circular foot path). LARGEST EFFECT MEASURED: net_disp 4.85→6.38 (+32%), straight 0.71→0.82 with std 0.00 across 3 seeds, tilt_sd 0.088→0.069, 0 falls. NOT PROMOTED: belly 0.0221→0.0156 (−29%) and belly-up is a promoted invariant. WATCH: does the shank stay under the knee through the stride, and is the belly scraping? Live slider `tibia_plumb` on [M]; +0.3 goes unstable, NEGATIVE un-plumbs.
	"the_picrawler_motor_epm_arena_short_stride.json",  # ── ARENA · SHORTER STRIDE ── stroke_gain 1.2 (deployed 1.65) — the "propulsion amplitude / stroke operating point" the ledger named and nobody had swept. A SHORTER stroke walks FURTHER: net_disp 4.85→5.45 (+12%), straight 0.71→0.78, BELLY PRESERVED, 0 falls; sweep peaks at 1.2. Corridor is a TIE on progress but step_bal 0.44→0.52; hump gate HOLDS. PARTIAL, not promoted — not LOUD enough on metrics alone, hence the eyeball. NOTE it does NOT move the feet inboard (that mechanism is refuted: hip1's axis is world +Z, a YAW joint that sweeps at CONSTANT radius, so foot_r is invariant across a 1.8× sweep). Live slider `stroke_gain` on [M].
	# --- 2026-08-02 PURE-HK OBSERVATION PAIR (corridor gym; [1]/[2] swap gyms live) ---
	# THE CONTEXT, which is why these two are worth eyeballing at all: ablating EVERY
	# learned component of MotorEPM costs the deployed gait nothing (net_z 4.75 ± 0.48
	# vs baseline 4.58 ± 0.27, n=4).  Locomotion in the CURRENT entry above is the
	# hand-built scaffold stack, not the brain — so every lever this campaign has judged
	# was measured in a context where the mechanism has no authority.  These two arms
	# strip the scaffolds so the homeokinetic controller is the ONLY thing driving the
	# legs.  Load the CONTROL first.  Full record: docs/reports/playful_machine_source_analysis.md
	"motor_epm_pure_hk__inst__stance.json",  # ── PURE-HK CONTROL ── homeokinetic core + the postural stance reflex ONLY. No stroke, no coupling, no gait phase, no heading, no height homeostat, no nav. n=4: net_z +0.13, chassis_y 0.029, steps 0.5, tilt_sd 0.044. Expect a low crouch that barely moves — that is the point of the comparison.
	"motor_epm_pure_hk__inst__stance__c025.json",  # ── PURE-HK · SELF-EXCITE ── the control + ONE parameter: c_init=0.25, the Playful Machine's self-exciting controller init (Sox cInit; ≈ their 0.75 at our motor_gain=3.0). Starts the loop at the edge of instability so HK SHAPES an oscillation instead of manufacturing one. n=4: steps 0.5→21, chassis 0.029→0.040, tilt_sd 0.119, planted 3.89, unstable 0.00, net_z +0.23 (deployed 4.58). ★ THE QUESTION METRICS CANNOT ANSWER: is this a body TRYING TO WALK or twitching in place? Every leg motion here is the learned controller — nothing scripts a gait. §3 rule 5 promote-or-kill gate.
	# --- everything below is parked, not deleted ---
	# "the_picrawler_motor_epm_embed_corridor_coordadapt.json",  # the fwd_v-REWARD comparison arm for the above (coord_fitness_mode=0). Uncomment to A/B the reward question directly in the UI.
	# "the_picrawler_motor_epm_minimal.json",   # L-1a Gate 0 — reward-free upright prior + HK loop (additive baseline)
	# "the_picrawler_motor_epm_objposture.json",# L-1b — postural via the objective.posture socket (PosturalPrior), postural_gain=0
	# "the_picrawler_motor_epm_keyframe.json",  # L-1b — KeyframeGait phase-indexed keyframe on the socket + dialed-down postural (0.3)
	# "the_picrawler_motor_epm_embed.json",     # L-1b — the embed MILESTONE gait (arena): keyframe + CPG-EMBED
	# "the_picrawler_motor_epm_embed_corridor.json",  # embed milestone in the CORRIDOR trench gym
	# "the_picrawler_motor_epm_embed_corridor_propbal.json",  # SEED-REFUTED (2026-07-23): noise + anti-composes
	# "the_picrawler_motor_epm_embed_corridor_bearinghold.json",  # the belly-DRAG predecessor to stance-lift (clearance dips to 0.003 = scraping)
	# "the_picrawler_motor_epm_embed_corridor_explore.json",  # heading-hold + stuck→explore 2.0 (seed-refuted as net-neutral)
	# "the_picrawler_motor_epm_velobj.json",  # REJECTED: phase-indexed VELOCITY objective (Cvel pump) — amplified a gait asymmetry into circling
	# "the_picrawler_motor_epm_velsym.json",  # REJECTED: velobj + LR-symmetry prior — the fix BACKFIRED (worse circling)
	# "the_picrawler_motor_epm_hip2learn.json",  # REJECTED: learned hip2 — no gain + more instability
	# "the_picrawler_motor_epm_cpgwalk.json",  # REJECTED: coherent but an open-loop sequencer — chassis slams the ground (flopping fish)
	# "the_picrawler_motor_epm_energetic.json", # EXPERIMENTAL high-energy variant
	# "the_picrawler_motor_epm_rung0.json",
	# "the_picrawler_motor_epm_vision.json",   # rung0 + epm_color (camera→brain, V1 vision)
	# "the_picrawler_motor_epm_vision_steer.json",  # V2: nav steers on VISION (vision_compass)
]

# 2026-06-13 — curriculum dropdown allowlist (same rationale as the config one).
# curricula/ accumulates dozens of historical probe/stand/whr arms; the launcher
# shows only the working headline modes + the current experiment.  "(none)" is
# always present and IS a headline mode (reward-free open-loop WALK — the
# motor_epm config walks with no curriculum).  Files are NOT deleted; every
# curriculum stays loadable by path for scripts / A/B runs.  Empty = show all.
const _PICRAWLER_CURRICULUM_ALLOWLIST: Array = [
	"picrawler_motor_epm_nav.json",         # current experiment — reward-free nav (g6dof authoritative)
	"picrawler_motor_epm_nav_fixed.json",   # A2 active-inference signature (fixed target)
	"picrawler_motor_epm_vision_nav.json",  # nav + vision wired to the brain (epm_color)
	"picrawler_motor_epm_vision_steer_nav.json",  # V2: nav steers on vision
]

# 2026-06-15 — cell config allowlist (same rationale as the picrawler one).
# The Cell port phase: only the reward-free chemotaxis headline is exposed.
# (The reward-driven Cell baselines live in configs/archive/ — not scanned by
# the non-recursive dir walk — and the full head-to-head is the deferred
# Phase B; copy a baseline up + allowlist it here when that lands.)  Empty =
# show all cell configs.
# 2026-06-20 — PRUNED to a focused watch-list (dead-end exploration arms removed
# from the launcher; findings preserved in commits + memory).  Three groups:
# reference baselines, the live wide-cell target, and the corridor diagnostics/diary.
const _CELL_CONFIG_ALLOWLIST: Array = [
	# 2026-07-12 -- curated to the 4-loop Markov-blanket STUDY demo set. Removed the legacy
	# arbiter/ortho/klino2/play/quad/placenav exploratory arms (still loadable by path + in git).
	# Descriptions + powered results: docs/reports/cell_markov_blanket_loops_report.md
	# --- the two headline demos ---
	"the_cell_arbiter_fused_lbend.json",          # FUSED 4-LOOP -- the full assembly; all four loops win a share. Slide play_weight/vision_weight 0<->1 live.
	"the_cell_vision_demo.json",                  # VISION loop #4 -- forage by SIGHT in a scent-poor pillar room. (+170% was an n=5 signal; powered = +0.35 ns.) Slide vision_weight 0<->1.
	# --- the study env + the loop ablations ---
	"the_cell_arbiter_room_pillars_vision.json",  # THE LEAVE-ONE-OUT ABLATION ENV (powered n=20). Only PLAY is significant -- and NEGATIVE (remove it -> +68% eats).
	"the_cell_play_only_lbend.json",              # PLAY ISOLATED -- 2.5x maze discovery, but discovery is NOT the eats lever (composition eats flat; play a net cost).
	"the_cell_arbiter_open_klino2.json",          # 3-LOOP composition (pre-vision) -- the homeostatic forager play-when-FULL -> klino-when-HUNGRY; the base vision was added onto.
	# --- reference baselines (fixed reflexes, NOT Markov-blanket loops) ---
	"the_cell_maze.json",                         # RING-COMPASS REFLEX BASELINE -- hand-coded directional chemotaxis (~334 hits, easy maze, ring available).
	"the_cell_chemotaxis_baseline.json",          # §6 EXTERNAL BASELINE -- reactive run-and-tumble (SCALAR) in the STUDY room: ~1.9 eats, BEATS the 4-loop composition (~0.9 = random-walk floor). A single reflex, not a loop.
]

const _ENV_TO_SCENE := {
	"cell":         "res://scenes/the_cell.tscn",
	"cartpole":     "res://scenes/the_cartpole.tscn",
	"mountain_car": "res://scenes/the_mountain_car.tscn",
	"quadruped":    "res://scenes/the_quadruped.tscn",
	"picrawler":    "res://scenes/the_picrawler.tscn",
}

const _ENV_LABEL := {
	"cell":         "Cell — embodied 9-modality navigation",
	"cartpole":     "CartPole-v1 — balance task",
	"mountain_car": "MountainCar-v0 — sparse-reward swing momentum",
	"quadruped":    "Quadruped — 8-DOF balance (v6.0)",
	"picrawler":    "PiCrawler — 12-DOF SunFounder hardware match (v6.0.b)",
}

# config registry: env_target → list of {path, name, description, phase_tag}
var _configs_by_env: Dictionary = {}
var _selected_env: String       = "cell"
var _verbose_check: CheckBox    = null   # 2026-06-14 — picrawler verbose-diag toggle (created in code)
var _gym_difficulty_spin: SpinBox = null       # corridor obstacle-height difficulty (created in code)
var _gym_difficulty_row:  HBoxContainer = null

# 2026-07-12 -- 4-LOOP LEAVE-ONE-OUT ablation (replaces the legacy Phase-6.5.22
# brain_weight/scent_gate presets, which were for the OLD reflex cell). Each preset
# patches the loaded config's EFEArbiter params at launch (body_controller.
# _apply_arbiter_overrides, mirroring the headless harness) so the user can deaden one
# Markov-blanket loop live and watch the arbiter re-balance. Contributions cited are the
# powered n=20 leave-one-out; see docs/reports/cell_markov_blanket_loops_report.md.
const _ABLATION_PRESETS: Array = [
	{
		"name": "None -- fused (all 4 loops)",
		"description": "The full 4-loop assembly: klino + planner + play + vision all feed the EFE arbiter. The composition as measured.",
		"overrides": {},
	},
	{
		"name": "minus PLAY (no explore)",
		"description": "Deaden play (arbiter.play_weight=0). The leave-one-out that RAISED eats +68% (powered n=20, the only significant loop): in a scent-poor room play wins ~80% by default and crowds out the planner.",
		"overrides": {"EFEArbiter.play_weight": 0.0},
	},
	{
		"name": "minus VISION (no sight)",
		"description": "Deaden vision (arbiter.vision_weight=0). Powered contribution +0.35 eats, NOT significant (the +170% was an n=5 signal). The loop is genuine -- it passes the (d) sensor-dropout test.",
		"overrides": {"EFEArbiter.vision_weight": 0.0},
	},
	{
		"name": "minus KLINO (no scent-close)",
		"description": "Deaden klino's arbiter input (scent_topic + klino_confidence_topic dead) so it never wins. Powered contribution -0.20 eats, ns in this scent-poor env.",
		"overrides": {"EFEArbiter.scent_topic": "dead", "EFEArbiter.klino_confidence_topic": "dead"},
	},
	{
		"name": "minus PLANNER (no routing)",
		"description": "Deaden planner's arbiter input (plan_value_topic dead) so it never wins. Powered contribution +0.15 eats, ns here.",
		"overrides": {"EFEArbiter.plan_value_topic": "dead"},
	},
]

# UI nodes (assigned in _ready)
@onready var _env_dropdown:        OptionButton = $Margin/V/EnvRow/EnvDropdown
@onready var _config_dropdown:     OptionButton = $Margin/V/ConfigRow/ConfigDropdown
@onready var _config_description:  Label        = $Margin/V/ConfigDescription/Label
@onready var _seed_spin:           SpinBox      = $Margin/V/SeedRow/SeedSpin
@onready var _seed_random:         CheckBox     = $Margin/V/SeedRow/SeedRandom
@onready var _episodes_spin:       SpinBox      = $Margin/V/EpisodesRow/EpisodesSpin
@onready var _max_steps_spin:      SpinBox      = $Margin/V/MaxStepsRow/MaxStepsSpin
@onready var _reset_mode_dropdown: OptionButton = $Margin/V/ResetModeRow/ResetModeDropdown
@onready var _chassis_shape_dropdown: OptionButton = $Margin/V/ChassisShapeRow/ChassisShapeDropdown
@onready var _joint_backend_dropdown: OptionButton = $Margin/V/JointBackendRow/JointBackendDropdown
@onready var _morphology_dropdown:  OptionButton = $Margin/V/MorphologyRow/MorphologyDropdown
@onready var _leg_strength_spin:    SpinBox      = $Margin/V/LegStrengthRow/LegStrengthSpin
@onready var _cell_extras:         VBoxContainer = $Margin/V/CellExtras
@onready var _body_model_dropdown: OptionButton = $Margin/V/CellExtras/BodyModelRow/BodyModelDropdown
@onready var _refractory_spin:     SpinBox      = $Margin/V/CellExtras/RefractoryRow/RefractorySpin
@onready var _obstacle_spin:       SpinBox      = $Margin/V/CellExtras/ObstacleRow/ObstacleSpin
@onready var _terrain_spin:        SpinBox      = $Margin/V/CellExtras/TerrainRow/TerrainSpin
@onready var _nutrients_spin:      SpinBox      = $Margin/V/CellExtras/NutrientsRow/NutrientsSpin
@onready var _energy_drain_spin:   SpinBox      = $Margin/V/CellExtras/EnergyDrainRow/EnergyDrainSpin
@onready var _scent_sigma_spin:    SpinBox      = $Margin/V/CellExtras/ScentSigmaRow/ScentSigmaSpin
@onready var _ablation_dropdown:   OptionButton = $Margin/V/CellExtras/AblationRow/AblationDropdown
@onready var _ablation_description: Label       = $Margin/V/CellExtras/AblationDescription/Label
@onready var _launch_btn:          Button       = $Margin/V/Buttons/Launch
@onready var _quit_btn:            Button       = $Margin/V/Buttons/Quit
@onready var _browse_btn:          Button       = $Margin/V/Buttons/BrowseResults
@onready var _curriculum_dropdown: OptionButton = $Margin/V/CurriculumRow/CurriculumDropdown
@onready var _curriculum_auto:     CheckBox     = $Margin/V/CurriculumRow/CurriculumAutoAdvance
@onready var _curriculum_row:      HBoxContainer = $Margin/V/CurriculumRow
@onready var _start_curr_btn:      Button       = $Margin/V/Buttons/StartCurriculum

func _ready() -> void:
	print("launcher: _ready start")
	_scan_configs()
	print("launcher: scanned %d env(s) of configs" % _configs_by_env.size())
	for k in _configs_by_env:
		print("  ", k, ": ", _configs_by_env[k].size(), " configs")
	_populate_env_dropdown()
	_populate_body_model_dropdown()
	_populate_ablation_dropdown()
	_populate_reset_mode_dropdown()
	_populate_chassis_shape_dropdown()
	_populate_joint_backend_dropdown()
	_populate_morphology_dropdown()
	# Restore previous selections (or defaults).
	ExperimentConfig.load_state()
	# 2026-06-14 — picrawler verbose-diag toggle (created in code; .tscn untouched).
	# Off by default → long interactive runs don't choke on per-tick console buildup
	# (a 10 h soak dropped to ~5 fps).  Check it only when you want the headless
	# trajectory JSONL in the UI for debugging.
	_verbose_check = CheckBox.new()
	_verbose_check.text = "Verbose diag log  (headless trajectory; leave OFF for long UI runs)"
	_verbose_check.button_pressed = ExperimentConfig.picrawler_verbose_log
	$Margin/V.add_child(_verbose_check)
	# 2026-07-22 — corridor gym obstacle-height difficulty (0=trivial .. 1=hard).
	# Created in code (.tscn untouched).  Only meaningful for the corridor gym;
	# the donut ignores it.  Scales hump / rumble-bump / pyramid heights.
	_gym_difficulty_row = HBoxContainer.new()
	var _gd_label := Label.new()
	_gd_label.text = "Corridor difficulty (0 easy .. 1 hard) "
	_gym_difficulty_spin = SpinBox.new()
	_gym_difficulty_spin.min_value = 0.0
	_gym_difficulty_spin.max_value = 1.0
	_gym_difficulty_spin.step = 0.05
	_gym_difficulty_spin.value = 0.3
	_gym_difficulty_row.add_child(_gd_label)
	_gym_difficulty_row.add_child(_gym_difficulty_spin)
	$Margin/V.add_child(_gym_difficulty_row)
	_apply_persisted_state()
	_env_dropdown.item_selected.connect(_on_env_changed)
	_config_dropdown.item_selected.connect(_on_config_changed)
	_seed_random.toggled.connect(_on_seed_random_toggled)
	_ablation_dropdown.item_selected.connect(_on_ablation_changed)
	_populate_curriculum_dropdown()
	_curriculum_dropdown.item_selected.connect(_on_curriculum_changed)
	_launch_btn.pressed.connect(_on_launch)
	_start_curr_btn.pressed.connect(_on_start_curriculum)
	_quit_btn.pressed.connect(_on_quit)
	_browse_btn.pressed.connect(_on_browse_results)
	_on_config_changed(_config_dropdown.selected)
	print("launcher: _ready complete")

# ---- Config discovery -------------------------------------------------------

func _scan_configs() -> void:
	_configs_by_env.clear()
	# DirAccess.get_files_at returns just file names (no dirs) — simpler
	# than the begin/get_next/current_is_dir loop and more robust against
	# Godot 4 quirks where current_is_dir() can mis-report after get_next.
	var fnames := DirAccess.get_files_at(_CONFIG_DIR)
	if fnames.is_empty():
		push_warning("launcher: no files under " + _CONFIG_DIR
			+ "  (DirAccess returned empty — check path resolution)")
	for fname in fnames:
		if not fname.ends_with(".json"):
			continue
		var path: String = _CONFIG_DIR + fname
		var entry = _read_metadata(path)   # Variant — null or Dictionary
		if entry == null:
			print("launcher: skipped (no metadata.env_target): ", path)
			continue
		var env: String = entry["env_target"]
		# Phase curation: for picrawler, only show the allowlisted configs.
		if env == "picrawler" and not _PICRAWLER_CONFIG_ALLOWLIST.has(fname):
			continue
		# Same curation for the cell port (empty allowlist = show all).
		if env == "cell" and not _CELL_CONFIG_ALLOWLIST.is_empty() \
				and not _CELL_CONFIG_ALLOWLIST.has(fname):
			continue
		if not _configs_by_env.has(env):
			_configs_by_env[env] = []
		_configs_by_env[env].append(entry)
		print("launcher: registered ", env, " — ", entry["name"])

func _read_metadata(path: String) -> Variant:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return null
	var raw := f.get_as_text()
	var parsed = JSON.parse_string(raw)
	if typeof(parsed) != TYPE_DICTIONARY:
		return null
	var d: Dictionary = parsed
	var meta: Dictionary = d.get("metadata", {})
	if not meta.has("env_target"):
		return null
	return {
		"path":           path,
		"filename":       path.get_file(),
		"name":           meta.get("name", path.get_file()),
		"description":    meta.get("description", ""),
		"env_target":     meta.get("env_target", ""),
		"phase_tag":      meta.get("phase_tag", ""),
		"derived":        meta.get("derived_from_manifest", false),
		"reflex_modular": bool(meta.get("reflex_modular", false)),
		# Phase 6.7 — a config may DECLARE the body model it requires (e.g. the
		# cell chemotaxis port needs differential_paddler); when present it wins
		# over the UI dropdown at launch so "select the config → reproduce" works.
		"body_model":     str(meta.get("body_model", "")),
		# Phase 6.8 — homeokinetic cell actuation flags (the alive-cell fast path).
		"motor_baseline_beat": bool(meta.get("motor_baseline_beat", false)),
		"motor_energy":        bool(meta.get("motor_energy", false)),
		# Cell experiment knobs — pre-populate the launcher spinners (sentinels
		# = "config doesn't declare it" → keep the spinner's default).
		"cell_nutrients":     int(meta.get("nutrients",    -1)),
		"cell_energy_drain":  float(meta.get("energy_drain", -1.0)),
		"cell_scent_sigma":   float(meta.get("scent_sigma",  -1.0)),
		"cell_obstacle_density":  float(meta.get("obstacle_density",  -1.0)),
		"cell_terrain_amplitude": float(meta.get("terrain_amplitude", -1.0)),
		# Phase v5.1 — opt-in episode boundary signalling for MC actor-
		# critic configs.  >0 → body publishes events.episode_end every N
		# physics ticks; 0 → no episode events (default).
		"episode_length": int(meta.get("episode_length", 0)),
		# Picrawler standing-task knobs (v6.0.b).  Sentinels mean
		# "not set" → ExperimentConfig.resolve_picrawler_* falls through
		# to OGMA_PICRAWLER_* env var → body @export default.
		"picrawler_mc_episode_period": int(meta.get("mc_episode_period",  -1)),
		"picrawler_stab_gain":         float(meta.get("stability_gain",   -1.0)),
		"picrawler_stab_y_norm":       float(meta.get("stability_y_norm", -1.0)),
		"picrawler_stab_speed":        float(meta.get("stability_speed",  -1.0)),
		"picrawler_antirot_threshold": float(meta.get("antirot_threshold", -1.0)),
		"picrawler_antirot_scale":     float(meta.get("antirot_scale",     -1.0)),
		"picrawler_antirot_gain":      float(meta.get("antirot_gain",      -1.0)),
		"picrawler_publish_tilt":      bool(meta.get("publish_tilt",       false)),
		"picrawler_energy_deadband":   float(meta.get("energy_deadband",   -1.0)),
		"picrawler_energy_scale":      float(meta.get("energy_scale",      -1.0)),
		"picrawler_energy_gain":       float(meta.get("energy_gain",       -1.0)),
		"picrawler_target_height":        float(meta.get("target_height",        -1.0)),
		"picrawler_height_penalty_grace": float(meta.get("height_penalty_grace", -1.0)),
		"picrawler_height_penalty_scale": float(meta.get("height_penalty_scale", -1.0)),
		"picrawler_height_penalty_gain":  float(meta.get("height_penalty_gain",  -1.0)),
		"picrawler_reward_shape":         str(meta.get("reward_shape", "")),
		"picrawler_peak_height":          float(meta.get("peak_height", -1.0)),
		"picrawler_band_width":           float(meta.get("band_width",  -1.0)),
		"picrawler_walk_target_velocity": float(meta.get("walk_target_velocity", -1.0)),
		"picrawler_walk_hit_rate":        float(meta.get("walk_hit_rate",        -1.0)),
		# B3 leg-symmetric weight averaging — empty string means "use body
		# default" (= "off"); set in config metadata or via env var.
		"picrawler_leg_symmetry":         str(meta.get("leg_symmetry",            "")),
		# Gym / world select — "" = donut arena (default), "corridor" = trench
		# curriculum gym.  Lets a config pick its world (metadata.gym_mode).
		"picrawler_gym_mode":             str(meta.get("gym_mode",                "")),
		# Corridor obstacle difficulty (0..1); -1 = not declared -> spinbox default.
		"picrawler_gym_difficulty":       float(meta.get("gym_difficulty",       -1.0)),
	}

# ---- Dropdown population ----------------------------------------------------

func _populate_env_dropdown() -> void:
	_env_dropdown.clear()
	for env_id in _ENV_TO_SCENE:
		_env_dropdown.add_item(_ENV_LABEL.get(env_id, env_id))
		_env_dropdown.set_item_metadata(_env_dropdown.item_count - 1, env_id)

func _populate_body_model_dropdown() -> void:
	_body_model_dropdown.clear()
	for label in ["asymmetric_paddler", "differential_paddler", "bidirectional_paddler"]:
		_body_model_dropdown.add_item(label)
		_body_model_dropdown.set_item_metadata(_body_model_dropdown.item_count - 1, label)

# v6.0 — episode reset mode dropdown.  Used by bodies that support
# selectable reset semantics (quadruped today; CartPole/MC pending
# migration).  Bodies that don't read ExperimentConfig.reset_mode just
# ignore the selection — safe to leave the widget visible across envs.
const _RESET_MODE_OPTIONS: Array = [
	{
		"id": "continuous",
		"label": "Continuous — no reset",
		"description": "Brain has to wiggle the body back up via its own torques.  No teleport; world-model continuity preserved.  max_steps becomes a run-end cap (not a teleport).",
	},
	{
		"id": "soft_blink",
		"label": "Soft blink — 30-tick brain blackout",
		"description": "On fall: chassis kinematically lerped back to standing pose over ~0.6 s while brain runs with zero motor authority and no reward.  Sleep/awakening analog.",
	},
	{
		"id": "instant_pause",
		"label": "Instant pause — legacy teleport",
		"description": "Gym/CartPole-style instant teleport with a one-tick brain.tick() skip.  Cheap; surprise spike on the tick AFTER the reset is preserved.",
	},
]

func _populate_reset_mode_dropdown() -> void:
	_reset_mode_dropdown.clear()
	for opt in _RESET_MODE_OPTIONS:
		_reset_mode_dropdown.add_item(str(opt["label"]))
		_reset_mode_dropdown.set_item_metadata(_reset_mode_dropdown.item_count - 1, opt)
	_reset_mode_dropdown.select(0)   # continuous = default

func _on_curriculum_changed(_idx: int) -> void:
	# Re-evaluate the Start-curriculum button's disabled state whenever
	# the dropdown changes.  _update_extras_visibility() is the single
	# source of truth for this gate.
	_update_extras_visibility()

func _populate_curriculum_dropdown() -> void:
	## Scan res://curricula/ for *.json files.  Each entry stores its
	## res:// path in item_metadata so the launch handler can pass it
	## directly to ExperimentConfig.picrawler_curriculum_path.
	_curriculum_dropdown.clear()
	_curriculum_dropdown.add_item("(none)")
	_curriculum_dropdown.set_item_metadata(_curriculum_dropdown.item_count - 1, "")
	var dir := DirAccess.open("res://curricula")
	if dir == null:
		return
	var found: Array = []
	dir.list_dir_begin()
	var fname: String = dir.get_next()
	while fname != "":
		if not dir.current_is_dir() and fname.ends_with(".json"):
			if _PICRAWLER_CURRICULUM_ALLOWLIST.is_empty() or _PICRAWLER_CURRICULUM_ALLOWLIST.has(fname):
				found.append(fname)
		fname = dir.get_next()
	dir.list_dir_end()
	found.sort()
	for f in found:
		var label: String = String(f).replace(".json", "")
		_curriculum_dropdown.add_item(label)
		_curriculum_dropdown.set_item_metadata(
			_curriculum_dropdown.item_count - 1, "res://curricula/" + f)
	_curriculum_dropdown.select(0)

# v6.0 — chassis shape dropdown for the quadruped body.  Bodies that
# don't read chassis_shape (CartPole/MountainCar/Cell) ignore the
# selection.
const _CHASSIS_SHAPE_OPTIONS: Array = [
	{
		"id": "half_cylinder",
		"label": "Half-cylinder — rolls when fallen",
		"description": "Flat side DOWN (legs attach), curved side UP.  Rounded back lets the body roll out of a fall instead of getting stuck flat.  Recommended for continuous mode.",
	},
	{
		"id": "box",
		"label": "Box — flat-back stable",
		"description": "Original rectangular chassis.  Falls onto a flat back and stays there.  Useful as a comparison baseline.",
	},
]

func _populate_chassis_shape_dropdown() -> void:
	_chassis_shape_dropdown.clear()
	for opt in _CHASSIS_SHAPE_OPTIONS:
		_chassis_shape_dropdown.add_item(str(opt["label"]))
		_chassis_shape_dropdown.set_item_metadata(_chassis_shape_dropdown.item_count - 1, opt)
	_chassis_shape_dropdown.select(0)   # half_cylinder = default

# 2026-06-03 — Joint backend selector for picrawler.  Independent of the
# brain config: a given brain stack can run on either hinge (legacy
# hobby-servo, reproduces historical baselines) or g6dof (Generic6DOF
# joints with adjustable angular spring + damping, the suspension line).
const _JOINT_BACKEND_OPTIONS: Array = [
	{
		"id": "hinge",
		"label": "Hinge (legacy hobby-servo)",
		"description": "Original HingeJoint3D rigid legs.  Reproduces all historical baselines and registry §3 metrics.  Default — use this for any A/B against pre-2026-06-02 results.",
	},
	{
		"id": "g6dof",
		"label": "G6DOF (adjustable suspension)",
		"description": "Generic6DOFJoint3D legs with passive angular spring on the free axis.  Per-joint-type stiffness + damping in body @export.  For the gait-resonance / passive-compliance line.",
	},
]

func _populate_joint_backend_dropdown() -> void:
	_joint_backend_dropdown.clear()
	for opt in _JOINT_BACKEND_OPTIONS:
		_joint_backend_dropdown.add_item(str(opt["label"]))
		_joint_backend_dropdown.set_item_metadata(_joint_backend_dropdown.item_count - 1, opt)
	_joint_backend_dropdown.select(0)   # hinge = default (preserves historical baselines)

const _MORPHOLOGY_OPTIONS: Array = [
	{
		"id": "spider",
		"label": "Spider — statically stable",
		"description": "Legs splay outward from the chassis corners; feet form a wide support polygon with COG roughly at hip height.  Brain can stand passively; learning focuses on perturbation rejection + locomotion.",
	},
	{
		"id": "dog",
		"label": "Dog — inverted pendulum (legacy)",
		"description": "Legs hang straight down.  Narrow support polygon, COG above legs.  Requires active balance every tick — brain must continuously fight gravity to stay upright.",
	},
]

func _populate_morphology_dropdown() -> void:
	_morphology_dropdown.clear()
	for opt in _MORPHOLOGY_OPTIONS:
		_morphology_dropdown.add_item(str(opt["label"]))
		_morphology_dropdown.set_item_metadata(_morphology_dropdown.item_count - 1, opt)
	_morphology_dropdown.select(0)   # spider = default

func _populate_ablation_dropdown() -> void:
	_ablation_dropdown.clear()
	for preset in _ABLATION_PRESETS:
		_ablation_dropdown.add_item(preset["name"])
		_ablation_dropdown.set_item_metadata(_ablation_dropdown.item_count - 1, preset)
	_ablation_dropdown.select(0)
	_on_ablation_changed(0)

func _populate_config_dropdown(env_id: String) -> void:
	_config_dropdown.clear()
	var entries: Array = _configs_by_env.get(env_id, [])
	# Sort: derived configs first (they're the "current default"), then by name.
	entries.sort_custom(_compare_config_entries)
	for entry in entries:
		var label := str(entry["name"])
		if entry.get("phase_tag", "") != "":
			label += "  [" + str(entry["phase_tag"]) + "]"
		# Show the JSON filename — titles don't always reflect the file on disk.
		if entry.get("filename", "") != "":
			label += "   —   " + str(entry["filename"])
		_config_dropdown.add_item(label)
		_config_dropdown.set_item_metadata(_config_dropdown.item_count - 1, entry)

func _compare_config_entries(a: Dictionary, b: Dictionary) -> bool:
	# Sort: derived configs first; then by name alphabetical.
	var a_derived: bool = a.get("derived", false)
	var b_derived: bool = b.get("derived", false)
	if a_derived != b_derived:
		return a_derived
	return str(a.get("name", "")) < str(b.get("name", ""))

# ---- State restoration / event handlers -------------------------------------

func _apply_persisted_state() -> void:
	# Default seed if never set.
	if ExperimentConfig.seed_value < 0:
		ExperimentConfig.seed_value = 42
	# Resolve env from persisted scene_path (if any).
	var env_id := "cell"
	for k in _ENV_TO_SCENE:
		if _ENV_TO_SCENE[k] == ExperimentConfig.scene_path:
			env_id = k
			break
	# Select env in dropdown.
	for i in range(_env_dropdown.item_count):
		if _env_dropdown.get_item_metadata(i) == env_id:
			_env_dropdown.select(i)
			break
	_selected_env = env_id
	_populate_config_dropdown(env_id)

	# Select previously-chosen config (if still present).
	var match_idx := 0
	if _config_dropdown.item_count > 0:
		for i in range(_config_dropdown.item_count):
			var meta = _config_dropdown.get_item_metadata(i)
			if typeof(meta) == TYPE_DICTIONARY and meta.get("path", "") == ExperimentConfig.config_path:
				match_idx = i
				break
	_config_dropdown.select(match_idx)

	# Persist the curriculum auto-advance checkbox across launches.
	if _curriculum_auto != null:
		_curriculum_auto.button_pressed = ExperimentConfig.picrawler_curriculum_auto_advance

	# Other widgets.
	_seed_spin.value     = ExperimentConfig.seed_value if ExperimentConfig.seed_value >= 0 else 42
	_seed_random.button_pressed = ExperimentConfig.seed_value < 0
	_seed_spin.editable  = not _seed_random.button_pressed
	_episodes_spin.value  = ExperimentConfig.max_episodes
	_max_steps_spin.value = ExperimentConfig.max_steps_per_episode
	# Body model
	for i in range(_body_model_dropdown.item_count):
		if _body_model_dropdown.get_item_metadata(i) == ExperimentConfig.body_model:
			_body_model_dropdown.select(i)
			break
	_refractory_spin.value = max(0, ExperimentConfig.refractory_ticks)
	_obstacle_spin.value   = max(0.0, ExperimentConfig.obstacle_density)
	_terrain_spin.value    = max(0.0, ExperimentConfig.terrain_amplitude)
	# 4-loop ablation is a per-launch choice (not derived from config metadata);
	# default to index 0 ("None -- fused (all 4 loops)").
	_ablation_dropdown.select(0)
	_on_ablation_changed(0)
	# Reset-mode dropdown.  Empty string in ExperimentConfig (never set OR
	# user cleared) → default to "continuous" (index 0).
	var rm_idx := 0
	var rm: String = ExperimentConfig.reset_mode
	if rm != "":
		for i in range(_reset_mode_dropdown.item_count):
			var opt: Dictionary = _reset_mode_dropdown.get_item_metadata(i)
			if str(opt.get("id", "")) == rm:
				rm_idx = i
				break
	_reset_mode_dropdown.select(rm_idx)
	# Chassis-shape dropdown.  Empty string → default to "half_cylinder"
	# (index 0).
	var cs_idx := 0
	var cs: String = ExperimentConfig.chassis_shape
	if cs != "":
		for i in range(_chassis_shape_dropdown.item_count):
			var opt: Dictionary = _chassis_shape_dropdown.get_item_metadata(i)
			if str(opt.get("id", "")) == cs:
				cs_idx = i
				break
	_chassis_shape_dropdown.select(cs_idx)
	# Joint backend dropdown — restore last selection (picrawler only;
	# other envs ignore the field).
	var jb_idx := 0
	var jb: String = ExperimentConfig.picrawler_joint_backend
	if jb != "":
		for i in range(_joint_backend_dropdown.item_count):
			var opt: Dictionary = _joint_backend_dropdown.get_item_metadata(i)
			if str(opt.get("id", "")) == jb:
				jb_idx = i
				break
	_joint_backend_dropdown.select(jb_idx)
	# Morphology dropdown.
	var mp_idx := 0
	var mp: String = ExperimentConfig.body_morphology
	if mp != "":
		for i in range(_morphology_dropdown.item_count):
			var opt: Dictionary = _morphology_dropdown.get_item_metadata(i)
			if str(opt.get("id", "")) == mp:
				mp_idx = i
				break
	_morphology_dropdown.select(mp_idx)
	# Leg strength spinbox.  -1.0 (sentinel for "never set") → default to 1.0.
	var ls: float = ExperimentConfig.leg_strength
	_leg_strength_spin.value = ls if ls > 0.0 else 1.0
	if _gym_difficulty_spin != null:
		var gdv: float = ExperimentConfig.picrawler_gym_difficulty
		_gym_difficulty_spin.value = gdv if gdv >= 0.0 else 0.3
	_update_extras_visibility()

func _on_env_changed(idx: int) -> void:
	_selected_env = _env_dropdown.get_item_metadata(idx)
	_populate_config_dropdown(_selected_env)
	_update_extras_visibility()
	_on_config_changed(_config_dropdown.selected)

func _on_config_changed(idx: int) -> void:
	if idx < 0 or idx >= _config_dropdown.item_count:
		_config_description.text = ""
		return
	var entry: Dictionary = _config_dropdown.get_item_metadata(idx)
	_config_description.text = entry.get("description", "")
	# Pre-populate the cell experiment spinners from the config's declared
	# defaults so "select config → Launch" reproduces it; the user can then tune.
	var cn := int(entry.get("cell_nutrients", -1))
	if cn >= 0: _nutrients_spin.value = cn
	var cd := float(entry.get("cell_energy_drain", -1.0))
	if cd >= 0.0: _energy_drain_spin.value = cd
	var cs := float(entry.get("cell_scent_sigma", -1.0))
	if cs >= 0.0: _scent_sigma_spin.value = cs
	var co := float(entry.get("cell_obstacle_density", -1.0))
	if co >= 0.0: _obstacle_spin.value = co
	var ct := float(entry.get("cell_terrain_amplitude", -1.0))
	if ct >= 0.0: _terrain_spin.value = ct
	# Pre-select the body-model dropdown to the config's DECLARED model so the UI
	# shows what will run; the user can then override it (launch reads the
	# dropdown, not the raw metadata, so the selection actually reaches the body).
	var cbm := str(entry.get("body_model", ""))
	if cbm != "":
		for i in range(_body_model_dropdown.item_count):
			if str(_body_model_dropdown.get_item_metadata(i)) == cbm:
				_body_model_dropdown.select(i)
				break
	# 2026-06-13 — the Motor-EPM (reward-free) configs were validated on the
	# g6dof + freeplay-0.1 compliant-stand substrate; auto-select g6dof so
	# pressing Launch reproduces it exactly (the body's g6dof preset then carries
	# the springs + freeplay).  Other configs default back to hinge (historical
	# baselines).  The user can still override the dropdown after.
	var fname: String = str(entry.get("filename", "")) + str(entry.get("path", ""))
	var want_backend: String = "g6dof" if fname.contains("motor_epm") else "hinge"
	for i in range(_joint_backend_dropdown.item_count):
		var opt: Dictionary = _joint_backend_dropdown.get_item_metadata(i)
		if str(opt.get("id", "")) == want_backend:
			_joint_backend_dropdown.select(i)
			break
	# Pre-populate the corridor-difficulty spinbox if the config declares one
	# (metadata.gym_difficulty), so "select config -> Launch" reproduces it.
	var cgd := float(entry.get("picrawler_gym_difficulty", -1.0))
	if cgd >= 0.0 and _gym_difficulty_spin != null:
		_gym_difficulty_spin.value = cgd

func _on_seed_random_toggled(pressed: bool) -> void:
	_seed_spin.editable = not pressed

func _on_ablation_changed(idx: int) -> void:
	if idx < 0 or idx >= _ablation_dropdown.item_count:
		_ablation_description.text = ""
		return
	var preset: Dictionary = _ablation_dropdown.get_item_metadata(idx)
	_ablation_description.text = preset.get("description", "")

func _selected_ablation_preset() -> Dictionary:
	var idx := _ablation_dropdown.selected
	if idx < 0 or idx >= _ablation_dropdown.item_count:
		return _ABLATION_PRESETS[0]
	return _ablation_dropdown.get_item_metadata(idx)

func _update_extras_visibility() -> void:
	_cell_extras.visible = (_selected_env == "cell")
	# ChassisShape + Morphology only apply to the quadruped (spider/dog
	# procedural construction).  PiCrawler has its own fixed geometry from
	# the SunFounder hardware spec; these knobs do nothing there.
	var quad_specific: bool = (_selected_env == "quadruped")
	$Margin/V/ChassisShapeRow.visible = quad_specific
	$Margin/V/MorphologyRow.visible   = quad_specific
	# Joint backend (hinge vs G6DOF suspension) only applies to articulated
	# bodies (quadruped + picrawler); the cell paddler has no joints.  This row
	# was previously never gated, so it leaked into the Cell view.
	$Margin/V/JointBackendRow.visible = (_selected_env == "quadruped" or _selected_env == "picrawler")
	# Separator sits directly above CellExtras — only meaningful when those show.
	$Margin/V/Sep2.visible = (_selected_env == "cell")
	# LegStrength applies to both quadruped + picrawler (servo torque cap).
	$Margin/V/LegStrengthRow.visible  = (_selected_env == "quadruped" or _selected_env == "picrawler")
	# ResetMode applies to both quadruped + picrawler (instant_pause / continuous / soft_blink).
	$Margin/V/ResetModeRow.visible    = (_selected_env == "quadruped" or _selected_env == "picrawler")
	# Curriculum row + Start-curriculum button only meaningful for picrawler.
	# (Other envs could grow curricula in future — restrict when they do.)
	_curriculum_row.visible = (_selected_env == "picrawler")
	if _verbose_check != null:
		_verbose_check.visible = (_selected_env == "picrawler")
	if _gym_difficulty_row != null:
		_gym_difficulty_row.visible = (_selected_env == "picrawler")
	_start_curr_btn.visible = (_selected_env == "picrawler")
	if _curriculum_row.visible:
		_start_curr_btn.disabled = (_curriculum_dropdown.get_item_count() == 0
			or _curriculum_dropdown.selected < 0
			or str(_curriculum_dropdown.get_item_metadata(_curriculum_dropdown.selected)) == "")

# ---- Launch -----------------------------------------------------------------

func _on_launch() -> void:
	# Resolve config path.
	var cfg_idx := _config_dropdown.selected
	if cfg_idx < 0:
		push_error("launcher: no config selected")
		return
	var entry: Dictionary = _config_dropdown.get_item_metadata(cfg_idx)
	var scene_path: String = _ENV_TO_SCENE.get(_selected_env, "")
	if scene_path == "":
		push_error("launcher: no scene mapped for env " + _selected_env)
		return

	# Populate ExperimentConfig.
	ExperimentConfig.scene_path     = scene_path
	ExperimentConfig.config_path    = entry.get("path", "")
	# Phase 6.6.D.5 — config metadata flag tells the body to suppress its
	# own events.* publishing in favor of graph-resident reflex modules.
	ExperimentConfig.reflex_modular = bool(entry.get("reflex_modular", false))
	ExperimentConfig.motor_baseline_beat = bool(entry.get("motor_baseline_beat", false))
	ExperimentConfig.motor_energy        = bool(entry.get("motor_energy", false))
	# Phase v5.1 — opt-in episode boundary publish for MC actor-critic.
	ExperimentConfig.episode_length = int(entry.get("episode_length", 0))
	# Picrawler standing-task knobs — sentinels (-1) mean "fall through".
	ExperimentConfig.picrawler_mc_episode_period = int(entry.get("picrawler_mc_episode_period", -1))
	ExperimentConfig.picrawler_stab_gain         = float(entry.get("picrawler_stab_gain", -1.0))
	ExperimentConfig.picrawler_stab_y_norm       = float(entry.get("picrawler_stab_y_norm", -1.0))
	ExperimentConfig.picrawler_stab_speed        = float(entry.get("picrawler_stab_speed", -1.0))
	ExperimentConfig.picrawler_antirot_threshold = float(entry.get("picrawler_antirot_threshold", -1.0))
	ExperimentConfig.picrawler_antirot_scale     = float(entry.get("picrawler_antirot_scale", -1.0))
	ExperimentConfig.picrawler_antirot_gain      = float(entry.get("picrawler_antirot_gain", -1.0))
	ExperimentConfig.picrawler_publish_tilt      = bool(entry.get("picrawler_publish_tilt", false))
	if _verbose_check != null:
		ExperimentConfig.picrawler_verbose_log   = _verbose_check.button_pressed
	ExperimentConfig.picrawler_energy_deadband   = float(entry.get("picrawler_energy_deadband", -1.0))
	ExperimentConfig.picrawler_energy_scale      = float(entry.get("picrawler_energy_scale", -1.0))
	ExperimentConfig.picrawler_energy_gain       = float(entry.get("picrawler_energy_gain", -1.0))
	ExperimentConfig.picrawler_target_height        = float(entry.get("picrawler_target_height", -1.0))
	ExperimentConfig.picrawler_height_penalty_grace = float(entry.get("picrawler_height_penalty_grace", -1.0))
	ExperimentConfig.picrawler_height_penalty_scale = float(entry.get("picrawler_height_penalty_scale", -1.0))
	ExperimentConfig.picrawler_height_penalty_gain  = float(entry.get("picrawler_height_penalty_gain", -1.0))
	ExperimentConfig.picrawler_reward_shape         = str(entry.get("picrawler_reward_shape", ""))
	ExperimentConfig.picrawler_peak_height          = float(entry.get("picrawler_peak_height", -1.0))
	ExperimentConfig.picrawler_band_width           = float(entry.get("picrawler_band_width", -1.0))
	ExperimentConfig.picrawler_walk_target_velocity = float(entry.get("picrawler_walk_target_velocity", -1.0))
	ExperimentConfig.picrawler_walk_hit_rate        = float(entry.get("picrawler_walk_hit_rate",        -1.0))
	ExperimentConfig.picrawler_leg_symmetry         = str(entry.get("picrawler_leg_symmetry",            ""))
	ExperimentConfig.picrawler_gym_mode             = str(entry.get("picrawler_gym_mode",                ""))
	# Corridor difficulty comes from the spinbox (picrawler only); other envs -1.
	ExperimentConfig.picrawler_gym_difficulty       = (_gym_difficulty_spin.value
		if (_selected_env == "picrawler" and _gym_difficulty_spin != null) else -1.0)
	if _seed_random.button_pressed:
		ExperimentConfig.seed_value = randi() % 1000000
	else:
		ExperimentConfig.seed_value = int(_seed_spin.value)
	ExperimentConfig.max_episodes          = int(_episodes_spin.value)
	ExperimentConfig.max_steps_per_episode = int(_max_steps_spin.value)
	# v6.0 — reset_mode: pick from the dropdown's metadata.  Bodies that
	# don't support selectable resets just ignore this field.
	var rm_idx := _reset_mode_dropdown.selected
	if rm_idx >= 0 and rm_idx < _reset_mode_dropdown.item_count:
		var rm_opt: Dictionary = _reset_mode_dropdown.get_item_metadata(rm_idx)
		ExperimentConfig.reset_mode = str(rm_opt.get("id", ""))
	else:
		ExperimentConfig.reset_mode = ""
	var cs_idx := _chassis_shape_dropdown.selected
	if cs_idx >= 0 and cs_idx < _chassis_shape_dropdown.item_count:
		var cs_opt: Dictionary = _chassis_shape_dropdown.get_item_metadata(cs_idx)
		ExperimentConfig.chassis_shape = str(cs_opt.get("id", ""))
	else:
		ExperimentConfig.chassis_shape = ""
	# Joint backend (picrawler) — propagate launcher selection so the body
	# picks the matching joint type at construction.
	var jb_idx := _joint_backend_dropdown.selected
	if jb_idx >= 0 and jb_idx < _joint_backend_dropdown.item_count:
		var jb_opt: Dictionary = _joint_backend_dropdown.get_item_metadata(jb_idx)
		ExperimentConfig.picrawler_joint_backend = str(jb_opt.get("id", ""))
	else:
		ExperimentConfig.picrawler_joint_backend = ""
	ExperimentConfig.leg_strength = float(_leg_strength_spin.value)
	var mp_idx := _morphology_dropdown.selected
	if mp_idx >= 0 and mp_idx < _morphology_dropdown.item_count:
		var mp_opt: Dictionary = _morphology_dropdown.get_item_metadata(mp_idx)
		ExperimentConfig.body_morphology = str(mp_opt.get("id", ""))
	else:
		ExperimentConfig.body_morphology = ""
	# Phase 6.5.5 — max-steps-per-episode == 0 means "no artificial cap":
	# in MC this enables continuous-mode (goal-only reset, no events.failed
	# on timeout).  In other envs the body controller decides what zero
	# means; for MC it specifically maps to continuous_mode.
	ExperimentConfig.mc_continuous_mode    = (int(_max_steps_spin.value) == 0)

	# Cell-specific extras.
	if _selected_env == "cell":
		# Use the body-model DROPDOWN selection — it was pre-set to the config's
		# declared model in _on_config_changed, so the default reproduces the
		# config, but the user's override now actually reaches the body (it was
		# previously clobbered by the raw metadata, so dropdown picks were silently
		# ignored — every cell run was forced to the config's declared model).
		var bm_meta = _body_model_dropdown.get_item_metadata(_body_model_dropdown.selected)
		ExperimentConfig.body_model    = str(bm_meta) if bm_meta != null else ""
		ExperimentConfig.refractory_ticks  = int(_refractory_spin.value)
		ExperimentConfig.obstacle_density  = float(_obstacle_spin.value)
		ExperimentConfig.terrain_amplitude = float(_terrain_spin.value)
		ExperimentConfig.cell_nutrient_count = int(_nutrients_spin.value)
		ExperimentConfig.cell_energy_drain   = float(_energy_drain_spin.value)
		ExperimentConfig.cell_scent_sigma    = float(_scent_sigma_spin.value)
		var ab: Dictionary = _selected_ablation_preset()
		ExperimentConfig.cell_arbiter_overrides = ab.get("overrides", {})
		# legacy brain/scent-gate ablation is inactive for the cognitive cell
		ExperimentConfig.brain_weight   = -1.0
		ExperimentConfig.scent_gate_cap = -1.0
		ExperimentConfig.scent_gate_off = false
		ExperimentConfig.brain_off      = false
	else:
		ExperimentConfig.body_model        = ""
		ExperimentConfig.refractory_ticks  = -1
		ExperimentConfig.obstacle_density  = -1.0
		ExperimentConfig.terrain_amplitude = -1.0
		ExperimentConfig.cell_nutrient_count = -1
		ExperimentConfig.cell_energy_drain   = -1.0
		ExperimentConfig.cell_scent_sigma    = -1.0
		ExperimentConfig.cell_arbiter_overrides = {}
		ExperimentConfig.brain_weight      = -1.0
		ExperimentConfig.scent_gate_cap    = -1.0
		ExperimentConfig.scent_gate_off    = false
		ExperimentConfig.brain_off         = false

	# Default launch path: clear any curriculum field so "Launch experiment"
	# never inherits a stale curriculum from a previous session.  The
	# "Start curriculum" handler re-populates it after this method returns
	# the equivalent setup; here we explicitly null it.
	ExperimentConfig.picrawler_curriculum_path         = ""
	ExperimentConfig.picrawler_curriculum_auto_advance = false

	ExperimentConfig.launched = true
	ExperimentConfig.save_state()

	get_tree().change_scene_to_file(scene_path)

func _on_start_curriculum() -> void:
	## "Start curriculum" path — same as Launch, but additionally sets
	## ExperimentConfig.picrawler_curriculum_path from the curriculum
	## dropdown + the auto-advance flag from the launcher checkbox.
	if _selected_env != "picrawler":
		push_warning("launcher: Start curriculum only supports picrawler env")
		return
	var ci: int = _curriculum_dropdown.selected
	if ci < 0:
		push_warning("launcher: no curriculum selected")
		return
	var curr_path: String = str(_curriculum_dropdown.get_item_metadata(ci))
	if curr_path == "":
		push_warning("launcher: '(none)' selected — pick a curriculum file first")
		return
	# Read the auto-advance checkbox BEFORE _on_launch, so we can inject
	# its value directly into the curriculum fields _on_launch was about
	# to clear.  Also surface the checkbox-resolution to the console so
	# a missing @onready node (e.g. stale scene cache) is immediately
	# visible — `_curriculum_auto` is null in that case.
	var auto_on: bool = false
	if _curriculum_auto != null:
		auto_on = _curriculum_auto.button_pressed
	else:
		push_warning("launcher: CurriculumAutoAdvance @onready resolved null — scene cache may be stale; assuming auto_advance=false")
	# Reuse _on_launch's full setup (config + body knobs + seed + …).
	_on_launch()
	# _on_launch zeroed the curriculum fields.  Re-populate them in-place
	# (change_scene_to_file is deferred to end-of-frame, so these writes
	# land on ExperimentConfig BEFORE the body's _ready runs).
	ExperimentConfig.picrawler_curriculum_path         = curr_path
	ExperimentConfig.picrawler_curriculum_auto_advance = auto_on
	ExperimentConfig.save_state()
	print("launcher: curriculum = %s  auto_advance=%s  (checkbox=%s)" %
		[curr_path, str(auto_on),
		 "null" if _curriculum_auto == null else str(_curriculum_auto.button_pressed)])

func _on_quit() -> void:
	get_tree().quit()

func _on_browse_results() -> void:
	get_tree().change_scene_to_file("res://scenes/results_browser.tscn")
