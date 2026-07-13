extends Node3D
## The Cell world — procedurally builds the room and places nutrients.
## Attach this script to the TheCell root Node3D.

@export var room_size: Vector3    = Vector3(16.0, 4.0, 16.0)
@export var wall_thickness: float = 0.4
@export var nutrient_count: int   = 3
@export var nutrient_radius: float = 0.5
# 2026-06-17 — AI-only foraging arena (Stage-1 ignition): food is center-biased
# (dense at the center spawn, sparse toward the edges) so first meals are
# climbable, and NO food spawns within nutrient_wall_margin of a wall — so a
# wall-sliding reflex can never score an eat (foraging must be directed).
@export var nutrient_center_sigma: float = 4.0   # radial Gaussian σ (m); ≤0 = uniform
@export var nutrient_wall_margin:  float = 1.5    # min food distance from any wall (m)
# 2026-06-19 — CORRIDOR 1-D credit-assignment probe.  When true: ONE food placed
# straight ahead of the bug (forward = −Z) at corridor_distance; the body locks
# heading (forward/reverse only); each eat resets the bug to the start for a
# fresh trial.  Isolates "learn to sequence forwards toward delayed reward."
@export var corridor: bool = false
@export var corridor_distance: float = 6.0   # m ahead of the start (−Z)
# 2026-06-20 — TURN rig: when >0, the corridor food is placed at a RANDOM bearing
# in [−max,+max]° off forward each trial (re-randomized on each eat/reset), and the
# body does NOT lock heading → the bug must LEARN to perceive the bearing and STEER
# toward it (the general "turn toward food" policy), not memorize one angle. 0 =
# pure 1-D dead-ahead corridor.
@export var corridor_bearing_max: float = 0.0
# 2026-06-21 — turn rig: on eat, keep the bug in place and respawn the food
# corridor_distance away at a random heading-relative bearing (±corridor_bearing_max)
# instead of teleporting the bug back to start.  Removes the unnatural reset + its
# reset-aversion confound on the approach/thrust channel.  Default false = legacy reset.
@export var corridor_respawn_far: bool = false
var _corridor_rng := RandomNumberGenerator.new()
@export var seed_value: int       = 42

# Scent: radial chemical gradient around each nutrient.  2026-06-16 — switched
# from a Gaussian e^(-d²/σ²) (flat-zero tail far + saturating flat top near) to a
# realistic diffusion-with-decay EXPONENTIAL envelope e^(-d/σ): smellable at ALL
# locations (long tail), a followable "warmer/cooler" gradient everywhere
# (monotonic, nonzero derivative), directional differential fuzzy far / popping
# near (cusp at the source).  σ = decay length; at distance σ the per-nutrient
# contribution is e^-1≈0.37.  σ=6 fills the 16 m room (16 m → e^-2.7≈0.07 > 0).
@export var scent_sigma: float    = 6.0
const NOSTRIL_COUNT  := 8
const NOSTRIL_RADIUS := 0.6   # body-local ring radius for scent samples

# Phase 6.1 — procedural pillars.  Rejection-sampled cylinders that give
# whiskers real contact signal and partially occlude the scent gradient,
# turning the open Cell into the "obstacle world" rung of the affordance
# ladder.  Density ∈ [0,1] is fraction of OBSTACLE_MAX_COVERAGE; seed is
# independent from `seed_value` so map layout and nutrient placement can
# be varied separately when sweeping.
@export var obstacle_density: float = 0.0
@export var obstacle_seed:    int   = 42
@export var pillar_radius:    float = 0.40
@export var pillar_height:    float = 2.00
# Multiplier on scent contribution when the line from nutrient to sample
# point is broken by a pillar.  0.0 = full block, 1.0 = no occlusion.
@export var obstacle_scent_attenuation: float = 0.30
# 2026-06-22 — MAZE + 2D-GRID DIFFUSION SCENT (docs/plans-and-designs/cell_maze_plan.md).
# When maze=true the scent is a SCREENED-POISSON field relaxed over FREE floor cells
# (walls = barriers, food = Dirichlet sources) → the gradient flows ALONG corridors /
# around corners, never through walls (the geodesic field; the straight-line exp(-d/σ)
# model is its broken approximation).  scent_falloff_radius = the decay length (m) = the
# CURRICULUM dial: long → followable everywhere (foundation); short → local-only →
# forces exploration + memory + planning.
@export var maze: bool = false
@export var maze_layout: String = "lbend"            # minimal: lbend (scale up later)
@export var quad_complexity: int = 0                 # quad: 0 = OPEN zones (structure only), 1 = per-zone internal walls
@export var scent_falloff_radius: float = 12.0        # screened-Poisson decay length (m); medium = followable foundation (too-long → flat field → no gradient)
@export var scent_reach: float = 1.0                   # LIVE field-distance dial (1=full, 0=off); UI slider. See _reach_scaled().
const SCENT_REACH_P_MAX := 24.0                        # reach=0 → exponent 25 (field collapses to the food cell)
const MAZE_CELL_SIZE := 0.5                            # diffusion grid resolution (m)
const MAZE_RELAX_SWEEPS := 600                         # Jacobi sweeps per recompute (cap)
const MAZE_RELAX_TOL := 1.0e-5
var _maze_wall: PackedByteArray = PackedByteArray()    # _grid_n*_grid_n, 1 = wall cell
# Progressive-reveal droppable walls (quad maze): the spiral connectors between zones. Each
# entry = {"body": StaticBody3D, "seg": [cx,cz,sx,sz]}. drop_next_wall() opens them in spiral
# order (SW→SE→NE→NW), clearing geometry + the diffusion grid so the next zone's scent leaks
# through. Triggered by metadata (timer/eats, for headless) AND the UI KEY_D hotkey.
var _droppable_walls: Array = []
var _static_wall_segs: Array = []  # all non-droppable wall segments (for the grid rebuild on each drop)
var _wall_drop_period: int = 0     # >0 → auto-drop the next connector every N sim-ticks (headless demo)
var _wall_drop_on_eats: int = 0    # >0 → auto-drop the next connector after every N cumulative eats
var _walls_dropped: int = 0
var _last_drop_tick: int = 0
var _world_tick: int = 0
var _eats_since_drop: int = 0
# 2026-06-25 — PREDICTABLE measurement env (doctrine §8): FIXED food spawn locations.
# metadata.food_positions = [[fx,fz], ...] in R-fractions [-0.5,0.5]; non-empty overrides
# the maze's random respawn — each nutrient lives at its fixed home and respawns there, so
# the bug can come to KNOW its world (eat-rate rises). L-bend default = one each side of
# the wall: one direct (south, line-of-sight), one occluded (NW, behind the wall).
var _food_positions: Array = []      # Vector2 R-fractions
var _food_homes: Dictionary = {}     # nutrient name → fixed home Vector3
# 2026-06-27 (operator) — ALTERNATING food: ONE nutrient that hops to the NEXT fixed
# position on each eat (instead of N fixed foods respawning in place). Forces the bug to
# TRAVEL between locations (exercises the PLACE loop's routing to the occluded side) AND
# moves the food out of the mouth on eat → one eat per contact (no re-eat inflation).
var _food_alternate: bool = false
var _food_alt_idx: int = 0
var _scent_field: PackedFloat32Array = PackedFloat32Array()  # diffused field (cached)
var _grid_n: int = 0
var _scent_dirty: bool = true
var _scent_cache: Dictionary = {}    # alt_idx → cached scent field; food_alternate cycles 2 FIXED positions, so cache both fields and skip the per-eat GDScript recompute (the UI lag)
var _maze_dumped: bool = false   # dump the INITIAL field only (sanity-viz)
# UI-only top-down scent heatmap overlay (KEY_H).  A floor-plane texture of the current
# diffusion field, refreshed whenever the field recomputes (food spawn).  On visual
# layer 3 → the top-down cam (layers 1+3) renders it, the FPV cam (layers 1+2) does not.
# Never built in headless → zero headless impact.
var _scent_overlay: MeshInstance3D = null
var _scent_overlay_mat: StandardMaterial3D = null

# 2026-06-23 — ZONE-TINTED 3D walls (operator diagnosis aid).  A spatial shader
# colours each wall/pillar fragment by its world-grid zone using the SAME mapping
# as the raycast brain-view (gx + 2*gz → 8-hue palette), so the top-down map's wall
# colours match what the brain perceives.  TINTED (mix into concrete, default 45%)
# so it reads as a muted hint, not jarring saturated blocks.  Floor stays grey (the
# raycast floor is grey too).  Opt-in via metadata.zone_colors (same flag the body
# uses); default off → original grey walls.
var _zone_colors: bool = false
var _zone_grid:   int  = 4
var _zone_tint:   float = 0.45            # 0 = grey, 1 = full zone hue
var _zone_wall_mat: ShaderMaterial = null
const OBSTACLE_MAX_COVERAGE := 0.10  # density=1.0 fills 10% of floor area
const OBSTACLE_MIN_SPACING  := 1.50  # pillar-edge to pillar-edge min gap
const OBSTACLE_ORIGIN_BUFFER := 2.00 # keep body spawn point clear
const OBSTACLE_NUTRIENT_BUFFER := 1.50 # keep nutrients reachable

# Phase 6.2 — procedural terrain.  Heightmap layered on top of the flat
# floor; body capsule physics-interacts with the slopes via move_and_slide.
# At amplitude=0 the terrain mesh is skipped and the world is identical to
# Phase 6.1 (back-compat).  Hills are sin × sin so the field has 4 hilltops
# and 4 valleys per period over the room — symmetric, deterministic,
# seedable.  Nutrients spawn preferentially on hilltops when amplitude > 0
# (gradient-up = food → terrain becomes a navigation cue).
@export var terrain_amplitude: float = 0.0
@export var terrain_period:    float = 8.0    # full sin × sin period in metres

# v5.4.M — randomized food respawn.  When true, a nutrient that was just
# eaten respawns at a NEW random location at least respawn_min_distance
# away from the agent (rather than reappearing at the same spot, which
# was producing seed-dependent feeding-loop variance: agents whose
# dynamics circled back to the eat spot farmed it; agents whose
# dynamics carried them away got penalised).  Each respawn uses a
# deterministic RNG seeded from seed_value + a per-respawn nonce so
# paired-seed A/Bs stay paired.
@export var respawn_randomize:   bool  = true
@export var respawn_min_distance: float = 4.0
# 2026-06-18 — when false, eaten food is GONE for good (no respawn): an efficient
# forager CLEARS the board faster, so ticks-to-clear / food-remaining is a clean
# efficiency metric (the precursor to maze-completion time).  Default true keeps
# the other cell configs (chemotaxis/cognitive survival ramp) unchanged.
# Override: metadata.food_respawn or OGMA_FOOD_RESPAWN=0.
@export var food_respawn: bool = true
# QUAD single-respawn (metadata.quad_respawn_single): with food_respawn=false, the 4 zone foods are
# one-shot (clear the board), and once all 4 are eaten a SINGLE food respawns at a time (at a random
# zone position) — the bug must re-find one moving target across the revealed maze. Default off.
@export var quad_respawn_single: bool = false

# Phase 6.9 — food-scarcity ramp (opt-in via OGMA_SCARCITY=1).  Food starts at
# nutrient_count (abundant); as nutrients are eaten the pool DECLINES — an eaten
# nutrient respawns only with prob scarcity_respawn_ratio (0.5 = 1:2), otherwise
# it's removed — down to a hard floor of scarcity_food_floor.  At the floor,
# exactly the floor count is always respawned, after a 1-5s delay, FAR from the
# bug.  Tabula-rasa survival pressure: the bug must learn to find the dwindling,
# distant food to maintain energy.  Default OFF (existing cell behaviour 1:1).
@export var scarcity_enabled:        bool  = false
@export var scarcity_respawn_ratio:  float = 0.5    # respawn prob while above floor (0.5 = 1:2)
@export var scarcity_food_floor:     int   = 1      # never drop below this many foods
@export var scarcity_floor_distance: float = 10.0   # respawn distance at the floor (far)
@export var scarcity_floor_min_delay: float = 1.0   # respawn delay range at the floor (s)
@export var scarcity_floor_max_delay: float = 5.0
var _active_food: int = 0   # nutrients currently in play (set to nutrient_count on spawn)
const TERRAIN_GRID_STEP := 0.50               # collision/mesh resolution
const TERRAIN_NUTRIENT_PERCENTILE := 0.70     # spawn above this height percentile
var _terrain_body: StaticBody3D = null

# v5.4.M — per-run respawn RNG + nonce.  Seeded from seed_value once at
# start; nonce increments per respawn so repeats within a run produce
# distinct positions but paired-seed runs (same seed, same eat sequence)
# produce identical respawn trajectories.  Determinism preserved.
var _respawn_rng:   RandomNumberGenerator = null
var _respawn_nonce: int                   = 0
var _respawn_count: int                   = 0     # diag counter

@onready var nutrients_node: Node3D = $Nutrients
var _topdown_cam: Camera3D = null
var _pillars: Array[StaticBody3D] = []  # for occlusion raycasts

# KEY_P cycle through obstacle densities for interactive sessions.
const _PILLAR_DENSITY_CYCLE := [0.0, 0.3, 0.7]
var _pillar_cycle_idx: int = 0

# KEY_T cycle through terrain amplitudes.  Same idiom as KEY_P; rebuilds the
# heightmap mesh + collision in place and re-rolls nutrient placement so
# food relocates to hilltops (or back to uniform random when flat).
const _TERRAIN_AMPLITUDE_CYCLE := [0.0, 0.3, 0.6]
var _terrain_cycle_idx: int = 0

# ---------------------------------------------------------------------------
# _ready: build room + spawn nutrients procedurally
# ---------------------------------------------------------------------------

func _ready() -> void:
	# Config-metadata defaults (config-self-contained: UI launch + headless both
	# load the same config, so "select the config → reproduce" matches).  Env
	# vars below still OVERRIDE these for sweeps.
	var _cell_meta: Dictionary = ExperimentConfig.read_cell_config_metadata()
	if _cell_meta.has("nutrients"):
		nutrient_count = int(_cell_meta["nutrients"])
	if _cell_meta.has("scent_sigma"):
		scent_sigma = float(_cell_meta["scent_sigma"])
	if _cell_meta.has("nutrient_center_sigma"):
		nutrient_center_sigma = float(_cell_meta["nutrient_center_sigma"])
	if _cell_meta.has("corridor"):
		corridor = bool(_cell_meta["corridor"])
	if OS.get_environment("OGMA_CORRIDOR") == "1":
		corridor = true
	if _cell_meta.has("corridor_distance"):
		corridor_distance = float(_cell_meta["corridor_distance"])
	var _cd: String = OS.get_environment("OGMA_CORRIDOR_DIST")
	if _cd != "":
		corridor_distance = _cd.to_float()
	if _cell_meta.has("corridor_bearing_max"):
		corridor_bearing_max = float(_cell_meta["corridor_bearing_max"])
	var _cb: String = OS.get_environment("OGMA_CORRIDOR_BEARING")
	if _cb != "":
		corridor_bearing_max = _cb.to_float()
	if corridor:
		nutrient_count = 1   # single food ahead
		# NOTE: _corridor_rng is seeded AFTER seed_value is resolved from
		# OGMA_SEED / launcher below — seeding here would use the @export
		# default (42) for every run → identical food bearings across seeds.
		if corridor_bearing_max > 0.0:
			print("CellWorld: CORRIDOR TURN rig — 1 food %.1fm at random bearing ±%.0f°, episodic." % [corridor_distance, corridor_bearing_max])
		else:
			print("CellWorld: CORRIDOR mode — 1 food %.1fm ahead (−Z), episodic trials." % corridor_distance)
	# 2026-06-21 — on eat, RESPAWN food far from the bug (random heading-relative
	# bearing) instead of teleporting the bug back to start.  A reset is "always
	# unnatural" (operator) + it makes the forward model learn "reach food → low-value
	# reset state" (reset-aversion confound for the approach/thrust channel).  Keeping
	# the bug in place + moving food far = the natural test of turn-toward + approach.
	if _cell_meta.has("corridor_respawn_far"):
		corridor_respawn_far = bool(_cell_meta["corridor_respawn_far"])
	if OS.get_environment("OGMA_CORRIDOR_RESPAWN_FAR") == "1":
		corridor_respawn_far = true
	if _cell_meta.has("room_size"):
		var rs := float(_cell_meta["room_size"])
		if rs > 0.0:
			room_size = Vector3(rs, room_size.y, rs)
	if _cell_meta.has("obstacle_density"):
		obstacle_density = clampf(float(_cell_meta["obstacle_density"]), 0.0, 1.0)
	if _cell_meta.has("terrain_amplitude"):
		terrain_amplitude = float(_cell_meta["terrain_amplitude"])
	# 2026-06-22 — maze mode + diffusion-scent falloff (the curriculum dial).
	if _cell_meta.has("maze"):
		maze = bool(_cell_meta["maze"])
	if OS.get_environment("OGMA_MAZE") == "1":
		maze = true
	if _cell_meta.has("maze_layout"):
		maze_layout = str(_cell_meta["maze_layout"])
	# QUAD progressive reveal: auto-drop the next spiral connector every N sim-ticks
	# (wall_drop_period) or after every N cumulative eats (wall_drop_on_eats). 0 = manual
	# (KEY_D) only. For headless demos set one so the zones open deterministically.
	if _cell_meta.has("wall_drop_period"):
		_wall_drop_period = int(_cell_meta["wall_drop_period"])
	if _cell_meta.has("wall_drop_on_eats"):
		_wall_drop_on_eats = int(_cell_meta["wall_drop_on_eats"])
	if _cell_meta.has("quad_complexity"):
		quad_complexity = int(_cell_meta["quad_complexity"])
	var _env_wdp: String = OS.get_environment("OGMA_WALL_DROP_PERIOD")
	if _env_wdp != "":
		_wall_drop_period = _env_wdp.to_int()
	if _cell_meta.has("food_alternate"):
		_food_alternate = bool(_cell_meta["food_alternate"])
	if _cell_meta.has("food_positions"):
		_food_positions.clear()
		for fp in _cell_meta["food_positions"]:
			_food_positions.append(Vector2(float(fp[0]), float(fp[1])))
		if _food_positions.size() > 0:
			# alternating = ONE food that hops between positions; else one nutrient per home
			nutrient_count = 1 if _food_alternate else _food_positions.size()
	if _cell_meta.has("scent_falloff_radius"):
		scent_falloff_radius = float(_cell_meta["scent_falloff_radius"])
	var env_sfr: String = OS.get_environment("OGMA_SCENT_FALLOFF")
	if env_sfr != "":
		scent_falloff_radius = max(0.5, env_sfr.to_float())
	# Launcher spinner overrides (UI launch) — above config metadata, below env.
	if ExperimentConfig.launched:
		if ExperimentConfig.cell_nutrient_count >= 0:
			nutrient_count = ExperimentConfig.cell_nutrient_count
		if ExperimentConfig.cell_scent_sigma >= 0.0:
			scent_sigma = ExperimentConfig.cell_scent_sigma
		# Obstacle/terrain spinners previously set ExperimentConfig but the world
		# only read env — so the UI spinners never reached the world.  Bridge them.
		if ExperimentConfig.obstacle_density >= 0.0:
			obstacle_density = ExperimentConfig.obstacle_density
		if ExperimentConfig.terrain_amplitude >= 0.0:
			terrain_amplitude = ExperimentConfig.terrain_amplitude
	# Run seed: launcher (ExperimentConfig) > OGMA_SEED env > @export default.
	# Without the ExperimentConfig branch the world IGNORED the launcher's seed
	# and always used its @export 42 — so a UI "seed 5" run had a seed-42 food
	# layout (the world) with a seed-5 brain, and never matched headless
	# (OGMA_SEED=5).  The launcher seed now reaches the world.
	if ExperimentConfig.launched and ExperimentConfig.seed_value >= 0:
		seed_value = ExperimentConfig.seed_value
	# Env-var overrides for the smoke-test driver — let the rig sweep
	# difficulty without editing the scene.
	var env_seed: String = OS.get_environment("OGMA_SEED")
	if env_seed != "":
		seed_value = env_seed.to_int()
	# Now that seed_value is resolved (launcher > OGMA_SEED > default), seed the
	# corridor turn-rig food-bearing RNG so different seeds give different
	# bearing sequences (was seeded above with the default → identical runs).
	_corridor_rng.seed = seed_value
	var env_n: String = OS.get_environment("OGMA_NUTRIENTS")
	if env_n != "":
		nutrient_count = env_n.to_int()
	var env_sigma: String = OS.get_environment("OGMA_SCENT_SIGMA")
	if env_sigma != "":
		scent_sigma = env_sigma.to_float()
	var env_room: String = OS.get_environment("OGMA_ROOM_SIZE")
	if env_room != "":
		var v := env_room.to_float()
		if v > 0.0:
			room_size = Vector3(v, room_size.y, v)
	var env_obs_d: String = OS.get_environment("OGMA_OBSTACLE_DENSITY")
	if env_obs_d != "":
		obstacle_density = clampf(env_obs_d.to_float(), 0.0, 1.0)
	# Phase 6.5.28 — scent-through-pillar attenuation override.  Setting
	# to 0.0 makes pillars fully scent-opaque, forcing the cart to use
	# vision (or wandering) to find food behind cover.  Default 0.30
	# (Phase 6.1) is partial occlusion: cart can still gradient-follow
	# weakly through a pillar.  1.0 = TRANSPARENT (true diffusion — scent
	# flows around obstacles, the physically-correct model; 2026-06-21).
	if _cell_meta.has("obstacle_scent_attenuation"):
		obstacle_scent_attenuation = clampf(float(_cell_meta["obstacle_scent_attenuation"]), 0.0, 1.0)
	var env_obs_a: String = OS.get_environment("OGMA_OBSTACLE_SCENT_ATTENUATION")
	if env_obs_a != "":
		obstacle_scent_attenuation = clampf(env_obs_a.to_float(), 0.0, 1.0)
	var env_obs_s: String = OS.get_environment("OGMA_OBSTACLE_SEED")
	if env_obs_s != "":
		obstacle_seed = env_obs_s.to_int()
	else:
		obstacle_seed = seed_value   # default: track the run seed
	var env_terrain_a: String = OS.get_environment("OGMA_TERRAIN_AMPLITUDE")
	if env_terrain_a != "":
		terrain_amplitude = env_terrain_a.to_float()
	var env_terrain_p: String = OS.get_environment("OGMA_TERRAIN_PERIOD")
	if env_terrain_p != "":
		terrain_period = max(0.5, env_terrain_p.to_float())
	# v5.4.M — randomized respawn knobs.  OGMA_RESPAWN_RANDOMIZE=0 reverts
	# to legacy "respawn at same spot" behaviour (for direct comparison
	# with pre-v5.4.M baseline runs).  OGMA_RESPAWN_MIN_DIST overrides
	# the floor distance from the agent at respawn time.
	var env_resp_r: String = OS.get_environment("OGMA_RESPAWN_RANDOMIZE")
	if env_resp_r != "":
		respawn_randomize = env_resp_r != "0"
	var env_resp_d: String = OS.get_environment("OGMA_RESPAWN_MIN_DIST")
	if env_resp_d != "":
		respawn_min_distance = max(0.5, env_resp_d.to_float())
	# 2026-06-18 — no-respawn (board-clearing) mode: metadata then env override.
	if _cell_meta.has("food_respawn"):
		food_respawn = bool(_cell_meta["food_respawn"])
	var env_fr: String = OS.get_environment("OGMA_FOOD_RESPAWN")
	if env_fr != "":
		food_respawn = env_fr != "0"
	if not food_respawn:
		print("CellWorld: food_respawn OFF — eaten food is removed; board can be cleared (ticks-to-clear metric).")
	# QUAD single-respawn: eat all 4 zone foods first (one-shot, clears the board), THEN respawn ONE
	# food at a time (a single moving target the bug must re-find across the revealed maze). Exercises
	# the 3-loop division: play explores, planner patrols/routes the known zones, klino closes on scent.
	if _cell_meta.has("quad_respawn_single"):
		quad_respawn_single = bool(_cell_meta["quad_respawn_single"])
	if quad_respawn_single:
		print("CellWorld: QUAD single-respawn ON — clear all %d zones, then respawn one food at a time." % nutrient_count)
	# Phase 6.9 — scarcity ramp env overrides.
	if OS.get_environment("OGMA_SCARCITY") == "1":
		scarcity_enabled = true
	var env_sc_r: String = OS.get_environment("OGMA_SCARCITY_RATIO")
	if env_sc_r != "": scarcity_respawn_ratio = clampf(env_sc_r.to_float(), 0.0, 1.0)
	var env_sc_f: String = OS.get_environment("OGMA_SCARCITY_FLOOR")
	if env_sc_f != "": scarcity_food_floor = max(1, env_sc_f.to_int())
	if scarcity_enabled:
		print("CellWorld: scarcity ON (ratio=%.2f floor=%d, far=%.1fm)" % [
			scarcity_respawn_ratio, scarcity_food_floor, scarcity_floor_distance])
	# Zone-tinted walls (operator diagnosis aid) — build the shader material before any
	# walls so _build_room/_build_maze/_spawn_pillars can apply it.  Same flag the body
	# uses for the raycast brain-view, so the colours match.
	if _cell_meta.has("zone_colors"):
		_zone_colors = bool(_cell_meta["zone_colors"])
	if _cell_meta.has("zone_grid"):
		_zone_grid = maxi(1, int(_cell_meta["zone_grid"]))
	if _cell_meta.has("zone_tint"):
		_zone_tint = clampf(float(_cell_meta["zone_tint"]), 0.0, 1.0)
	if _zone_colors:
		_zone_wall_mat = _make_zone_wall_material()
	_enforce_honest_respawn()   # food never respawns in the same place twice
	_build_room()
	if maze:
		_build_maze()          # interior walls + occupancy grid (before nutrients)
		if DisplayServer.get_name() != "headless":
			_build_scent_overlay()   # UI-only top-down heatmap (KEY_H); zero headless impact
	_build_terrain()
	_build_topdown_cam()
	_spawn_nutrients()
	if maze:
		if _food_positions.size() > 0:
			_place_fixed_foods()   # 2-food predictable env (each side of the wall)
		else:
			_place_maze_food()     # move food to the maze goal (out of line-of-sight)
	_spawn_pillars()
	# v5.4.M — initialise respawn RNG.  Seed XOR'd with a constant so the
	# respawn sequence doesn't share state with nutrient layout RNG (which
	# uses raw seed_value above).
	_respawn_rng = RandomNumberGenerator.new()
	_respawn_rng.seed = seed_value ^ 0x52455350    # "RESP"
	print("TheCell: room=%.0fx%.0f, %d nutrients, sigma=%.1f, pillars=%d, terrain=A%.2fL%.1f (seed %d, obs_seed %d)" % [
		room_size.x, room_size.z, nutrient_count, scent_sigma, _pillars.size(),
		terrain_amplitude, terrain_period, seed_value, obstacle_seed])
	# Wire the Brain node into the graph panel (deferred so both are ready).
	call_deferred("_wire_graph_panel")

func _wire_graph_panel() -> void:
	var panel = get_tree().get_first_node_in_group("ogma_graph_panel")
	if panel == null:
		return
	var brain = get_node_or_null("Body/Brain")
	if brain:
		panel.set_brain(brain)

func _make_zone_wall_material() -> ShaderMaterial:
	# Spatial shader: ALBEDO = mix(concrete, zone-hue, tint), zone = (gx + 2*gz) % 8
	# from world (x,z) — the SAME mapping the raycast brain-view uses, so a wall reads
	# the same hue the brain perceives there.  Tinted (muted) so it's not jarring.
	var sh := Shader.new()
	sh.code = """
shader_type spatial;
uniform float u_room_size = 24.0;
uniform int u_zone_grid = 4;
uniform vec3 u_palette[8];
uniform vec3 u_base = vec3(0.75, 0.72, 0.68);
uniform float u_tint = 0.45;
varying vec3 v_world;
void vertex() { v_world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }
void fragment() {
	float h = u_room_size * 0.5;
	float g = float(u_zone_grid);
	float gx = clamp(floor((v_world.x + h) / u_room_size * g), 0.0, g - 1.0);
	float gz = clamp(floor((v_world.z + h) / u_room_size * g), 0.0, g - 1.0);
	int idx = int(mod(gx + 2.0 * gz, 8.0));
	ALBEDO = mix(u_base, u_palette[idx], u_tint);
	ROUGHNESS = 0.9;
}
"""
	var m := ShaderMaterial.new()
	m.shader = sh
	m.set_shader_parameter("u_room_size", room_size.x)
	m.set_shader_parameter("u_zone_grid", _zone_grid)
	m.set_shader_parameter("u_tint", _zone_tint)
	m.set_shader_parameter("u_base", Vector3(0.75, 0.72, 0.68))
	# Same hues as body_controller _ZONE_PALETTE (normalised), muted by u_tint.
	m.set_shader_parameter("u_palette", PackedVector3Array([
		Vector3(0.824, 0.176, 0.176),  # red
		Vector3(0.882, 0.510, 0.118),  # orange
		Vector3(0.843, 0.804, 0.176),  # yellow
		Vector3(0.176, 0.686, 0.804),  # cyan
		Vector3(0.216, 0.294, 0.824),  # blue
		Vector3(0.608, 0.176, 0.765),  # purple
		Vector3(0.824, 0.196, 0.627),  # magenta
		Vector3(0.510, 0.333, 0.176),  # brown
	]))
	return m


func _build_room() -> void:
	var room := StaticBody3D.new()
	room.name = "Room"
	add_child(room)

	# Shared wall material — bright enough to be visible without any specific light rig.
	var wall_mat := StandardMaterial3D.new()
	wall_mat.albedo_color = Color(0.75, 0.72, 0.68)   # warm concrete
	wall_mat.roughness    = 0.85

	var floor_mat := StandardMaterial3D.new()
	floor_mat.albedo_color = Color(0.55, 0.52, 0.50)
	floor_mat.roughness    = 0.95

	# The 4 perimeter walls use the zone-tint shader (if enabled) so the top-down map
	# matches the brain-view; floor + ceiling stay grey (the raycast floor is grey too).
	var wall_render_mat: Material = _zone_wall_mat if _zone_wall_mat != null else wall_mat

	# Floor, Ceiling, 4 walls — each is a CollisionShape3D + MeshInstance3D.
	var panels = [
		# name,      position,                                          size,          material
		["Floor",   Vector3(0, -wall_thickness * 0.5,     0),     Vector3(room_size.x, wall_thickness, room_size.z),  floor_mat],
		["Ceiling", Vector3(0, room_size.y + wall_thickness*0.5, 0), Vector3(room_size.x, wall_thickness, room_size.z), wall_mat],
		["WallN",   Vector3(0, room_size.y*0.5,  room_size.z*0.5 + wall_thickness*0.5),
					Vector3(room_size.x, room_size.y, wall_thickness),  wall_render_mat],
		["WallS",   Vector3(0, room_size.y*0.5, -room_size.z*0.5 - wall_thickness*0.5),
					Vector3(room_size.x, room_size.y, wall_thickness),  wall_render_mat],
		["WallE",   Vector3( room_size.x*0.5 + wall_thickness*0.5, room_size.y*0.5, 0),
					Vector3(wall_thickness, room_size.y, room_size.z),  wall_render_mat],
		["WallW",   Vector3(-room_size.x*0.5 - wall_thickness*0.5, room_size.y*0.5, 0),
					Vector3(wall_thickness, room_size.y, room_size.z),  wall_render_mat],
	]

	for p in panels:
		var panel_name: String      = p[0]
		var pos: Vector3            = p[1]
		var sz: Vector3             = p[2]
		var mat: Material           = p[3]

		var box_shape            := BoxShape3D.new()
		box_shape.size           = sz
		var col_shape            := CollisionShape3D.new()
		col_shape.name           = panel_name + "Col"
		col_shape.shape          = box_shape
		col_shape.position       = pos
		room.add_child(col_shape)

		var box_mesh             := BoxMesh.new()
		box_mesh.size            = sz
		box_mesh.material        = mat
		var mesh_inst            := MeshInstance3D.new()
		mesh_inst.name           = panel_name + "Mesh"
		mesh_inst.mesh           = box_mesh
		mesh_inst.position       = pos
		# Ceiling lives on a dedicated visual layer so the top-down camera can
		# cull it; FPV camera (default cull_mask) still sees it.
		if panel_name == "Ceiling":
			mesh_inst.layers = 1 << 1   # layer 2
		room.add_child(mesh_inst)

	# Central ceiling light — guarantees the room is lit even if the
	# directional light angle is unfortunate.
	var omni             := OmniLight3D.new()
	omni.name            = "RoomLight"
	omni.position        = Vector3(0, room_size.y * 0.85, 0)
	omni.light_energy    = 1.6
	omni.light_color     = Color(1.0, 0.95, 0.85)
	omni.omni_range      = room_size.x * 1.2
	add_child(omni)

# =====================================================================================
# MAZE + 2D-grid DIFFUSION SCENT (2026-06-22, docs/plans-and-designs/cell_maze_plan.md)
# =====================================================================================

# Build the interior maze: physical wall segments (the bug collides) + the occupancy
# grid the diffusion relaxes over.  Minimal "lbend": one wall north of centre spanning
# the west wall to +0.2R, leaving a gap on the east → the straight line from the origin
# (south, free) to the food (northwest, behind the wall) crosses a wall, so the bug must
# route east → north through the gap → west (an L/Z path).  The minimal falsifier.
func _build_maze() -> void:
	var R: float = room_size.x
	_grid_n = max(8, int(ceil(R / MAZE_CELL_SIZE)))
	_maze_wall = PackedByteArray()
	_maze_wall.resize(_grid_n * _grid_n)   # all 0 = free
	_droppable_walls.clear()
	_walls_dropped = 0
	_last_drop_tick = 0
	var segs: Array = []   # STATIC walls [center_x, center_z, size_x, size_z]
	match maze_layout:
		"open", "none":
			pass   # no internal wall — open arena (fixed-food dev env, doctrine §3)
		"quad":
			_build_quad(segs)   # 2x2 spiral, 4 zones, + registers the 3 droppable connectors
		_:  # "lbend" (default)
			var wlen: float = R * 0.70
			var wcx: float = -R * 0.5 + wlen * 0.5    # west-aligned → gap on the east
			segs.append([wcx, R * 0.15, wlen, wall_thickness])
	for s in segs:
		_create_maze_wall(s[0], s[1], s[2], s[3])
		_mark_wall_cells(s[0], s[1], s[2], s[3])
	_scent_dirty = true
	print("CellWorld: MAZE '%s' — grid %dx%d cell %.2fm, falloff %.1fm" % [
		maze_layout, _grid_n, _grid_n, R / float(_grid_n), scent_falloff_radius])

func _create_maze_wall(cx: float, cz: float, sx: float, sz: float) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.name = "MazeWall"
	var box := BoxShape3D.new()
	box.size = Vector3(sx, room_size.y, sz)
	var col := CollisionShape3D.new()
	col.shape = box
	col.position = Vector3(cx, room_size.y * 0.5, cz)
	body.add_child(col)
	var mat: Material
	if _zone_wall_mat != null:
		mat = _zone_wall_mat            # zone-tinted to match the brain-view
	else:
		var sm := StandardMaterial3D.new()
		sm.albedo_color = Color(0.62, 0.46, 0.40)   # maze walls a touch warmer than the room
		sm.roughness = 0.9
		mat = sm
	var mesh := BoxMesh.new()
	mesh.size = Vector3(sx, room_size.y, sz)
	mesh.material = mat
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.position = Vector3(cx, room_size.y * 0.5, cz)
	body.add_child(mi)
	add_child(body)
	return body

# ===========================================================================
# QUAD maze — a 2x2 grid of zones with INCREASING complexity, connected in a
# SPIRAL (SW→SE→NE→NW) by three DROPPABLE walls (progressive reveal + KEY_D).
# A + cross at x=0 / z=0 isolates the quadrants; the three connectors are the
# only openings and start CLOSED, so the bug is boxed into Z1 (SW) until each
# wall drops.  One fixed food per zone (via config food_positions).  Pillars
# and terrain layer on top unchanged.
# ===========================================================================
func _build_quad(segs: Array) -> void:
	var R: float = room_size.x
	var half: float = R * 0.5              # 24 at R=48
	var t: float = wall_thickness
	var g: float = R * 0.052               # connector half-gap (~2.5m → 5m corridor at R=48)
	var qc: float = R * 0.25               # connector centre magnitude (12) — mid of each shared wall

	# --- the + cross: vertical wall x=0 with gaps at z=±qc (SW↔SE south, NE↔NW north) ---
	segs.append([0.0, -(half + (qc + g)) * 0.5, t, half - (qc + g)])   # V1 south of the south connector
	segs.append([0.0, 0.0, t, (qc - g) * 2.0])                          # V2 the central span (crosses z=0)
	segs.append([0.0,  (half + (qc + g)) * 0.5, t, half - (qc + g)])   # V3 north of the north connector
	# --- horizontal wall z=0: WEST half solid (SW|NW), EAST half with the SE↔NE connector gap at x=+qc ---
	segs.append([-half * 0.5, 0.0, half, t])                            # H_W west (solid divider)
	segs.append([(qc - g) * 0.5, 0.0, qc - g, t])                       # H_E1 east, up to the connector
	segs.append([(half + qc + g) * 0.5, 0.0, half - (qc + g), t])      # H_E2 east, past the connector

	# --- the three DROPPABLE connectors, registered in SPIRAL order (SW→SE→NE→NW) ---
	_register_droppable(0.0, -qc, t, g * 2.0)    # 1: SW → SE  (south of the vertical wall)
	_register_droppable( qc, 0.0, g * 2.0, t)    # 2: SE → NE  (east of the horizontal wall)
	_register_droppable(0.0,  qc, t, g * 2.0)    # 3: NE → NW  (north of the vertical wall)

	# --- per-zone internal walls, increasing complexity Z1<Z2<Z3<Z4 ---
	# quad_complexity metadata gates this: 0 = OPEN zones (validate structure), 1 = walled zones.
	if quad_complexity >= 1:
		# Z1 SW (x<0,z<0) — lbend: one wall, gap toward the SW→SE connector.
		segs.append([-half * 0.60, -half * 0.62, half * 0.45, t])
		# Z2 SE (x>0,z<0) — two offset walls (S-path).
		segs.append([ half * 0.40, -half * 0.66, half * 0.45, t])
		segs.append([ half * 0.66, -half * 0.36, half * 0.45, t])
		# Z3 NE (x>0,z>0) — a corridor divider + a dead-end stub.
		segs.append([ half * 0.5,  half * 0.62, half * 0.5, t])
		segs.append([ half * 0.34, half * 0.5, t, half * 0.45])
		# Z4 NW (x<0,z>0) — denser: three walls forming a tight route.
		segs.append([-half * 0.5,  half * 0.36, half * 0.5, t])
		segs.append([-half * 0.66, half * 0.62, half * 0.45, t])
		segs.append([-half * 0.30, half * 0.72, t, half * 0.40])

	_static_wall_segs = segs.duplicate(true)   # remember for the grid rebuild on each wall drop
	print("CellWorld: QUAD maze — 4 zones, 3 droppable connectors (spiral SW→SE→NE→NW), complexity=%d" % quad_complexity)

# Create + track a droppable connector wall (geometry + grid cells).  drop_next_wall()
# removes them in registration (spiral) order.
func _register_droppable(cx: float, cz: float, sx: float, sz: float) -> void:
	var body := _create_maze_wall(cx, cz, sx, sz)
	body.name = "DropWall%d" % _droppable_walls.size()
	_mark_wall_cells(cx, cz, sx, sz)
	_droppable_walls.append({"body": body, "seg": [cx, cz, sx, sz], "dropped": false})

# Rebuild the diffusion wall grid from all STATIC walls + all NON-dropped connectors.
# (Rebuild-from-scratch avoids the shared-padded-cell problem of clearing one wall.)
func _rebuild_wall_grid() -> void:
	for i in range(_maze_wall.size()):
		_maze_wall[i] = 0
	for s in _static_wall_segs:
		_mark_wall_cells(s[0], s[1], s[2], s[3])
	for d in _droppable_walls:
		if not d["dropped"]:
			var s: Array = d["seg"]
			_mark_wall_cells(s[0], s[1], s[2], s[3])

# Open the next spiral connector: remove its geometry, rebuild the grid, recompute scent
# (so the next zone's scent leaks through).  Returns true if a wall was dropped.
func drop_next_wall() -> bool:
	for d in _droppable_walls:
		if not d["dropped"]:
			d["dropped"] = true
			var b: StaticBody3D = d["body"]
			if is_instance_valid(b):
				b.queue_free()
			_rebuild_wall_grid()
			_scent_dirty = true
			_walls_dropped += 1
			print("CellWorld: DROPPED connector %d/%d → zone opened" % [_walls_dropped, _droppable_walls.size()])
			return true
	return false

func _mark_wall_cells(cx: float, cz: float, sx: float, sz: float) -> void:
	var n: int = _grid_n
	var cell: float = room_size.x / float(n)
	var pad: float = cell * 0.6   # inflate so the gradient can't leak through a seam
	var x0: float = cx - sx * 0.5 - pad
	var x1: float = cx + sx * 0.5 + pad
	var z0: float = cz - sz * 0.5 - pad
	var z1: float = cz + sz * 0.5 + pad
	for gz in range(n):
		var wz: float = -room_size.z * 0.5 + (gz + 0.5) * cell
		if wz < z0 or wz > z1:
			continue
		for gx in range(n):
			var wx: float = -room_size.x * 0.5 + (gx + 0.5) * cell
			if wx >= x0 and wx <= x1:
				_maze_wall[gz * n + gx] = 1

# The maze goal: food northwest, behind the lbend wall (out of line-of-sight from origin).
func _maze_food_pos() -> Vector3:
	var R: float = room_size.x
	return Vector3(-R * 0.28, 0.5, R * 0.36)

func _place_maze_food() -> void:
	var pos: Vector3 = _maze_food_pos()
	for nut in nutrients_node.get_children():
		if nut is StaticBody3D:
			nut.position = pos
			break   # single goal (minimal maze)
	_scent_dirty = true

# Predictable env (doctrine §8): position each nutrient at its FIXED home (R-fractions)
# and record it so consume_nutrient respawns it there → the bug can learn its world.
func _place_fixed_foods() -> void:
	var R: float = room_size.x
	_food_homes.clear()
	var i: int = 0
	for nut in nutrients_node.get_children():
		if not (nut is StaticBody3D):
			continue
		if i >= _food_positions.size():
			break
		var home := Vector3(_food_positions[i].x * R, 0.5, _food_positions[i].y * R)
		nut.position = home
		_food_homes[nut.name] = home
		print("CellWorld: FIXED food %s at (%.2f, %.2f)" % [nut.name, home.x, home.z])
		i += 1
	_scent_dirty = true

# Respawn the maze food at a random FREE grid cell at least min_d from the bug, so each
# eat forces a fresh route (and the diffusion always has a valid source cell).
func _maze_respawn_pos(body: Node3D) -> Vector3:
	var n: int = _grid_n
	var cell: float = room_size.x / float(n)
	var bp: Vector3 = body.global_position
	var min_d: float = room_size.x * 0.40
	for _i in range(48):
		var gx: int = _respawn_rng.randi_range(2, n - 3)
		var gz: int = _respawn_rng.randi_range(2, n - 3)
		if _maze_wall[gz * n + gx] == 1:
			continue
		var wx: float = -room_size.x * 0.5 + (gx + 0.5) * cell
		var wz: float = -room_size.z * 0.5 + (gz + 0.5) * cell
		var cand: Vector3 = Vector3(wx, 0.5, wz)
		if cand.distance_to(bp) >= min_d:
			return cand
	return _maze_food_pos()

# Screened-Poisson relaxation of the scent field over FREE cells (walls barrier, food
# pinned to 1.0).  Steady-state decays exponentially with length scent_falloff_radius
# along the free-cell graph → the gradient follows corridors, never crosses walls.
func _recompute_scent_field() -> void:
	var n: int = _grid_n
	if n <= 0:
		return
	var cell: float = room_size.x / float(n)
	var kh2: float = pow(cell / max(0.01, scent_falloff_radius), 2.0)   # κ²h² ; decay len = 1/κ = falloff
	var is_src: PackedByteArray = PackedByteArray()
	is_src.resize(n * n)
	for nut in nutrients_node.get_children():
		if nut is StaticBody3D and nut.visible:
			var gp: Vector3 = nut.global_position
			var gx: int = clampi(int((gp.x + room_size.x * 0.5) / cell), 0, n - 1)
			var gz: int = clampi(int((gp.z + room_size.z * 0.5) / cell), 0, n - 1)
			if _maze_wall[gz * n + gx] == 0:
				is_src[gz * n + gx] = 1
	if _scent_field.size() != n * n:
		_scent_field = PackedFloat32Array()
		_scent_field.resize(n * n)
	for i in range(n * n):
		_scent_field[i] = (1.0 if is_src[i] == 1 else 0.0)
	# Gauss-Seidel + over-relaxation (SOR): in-place, converges in ~O(n) sweeps vs
	# O(n²) for Jacobi, so the field reaches steady state even at long falloff within
	# the cap.  Walls (=0) and source cells (=1.0, set in init) are skipped → held.
	var omega: float = 1.9    # over-relaxation (n≈48 optimal ~1.87)
	# SOR converges in ~O(n) sweeps, so scale the cap with the grid size — the quad's 96×96
	# grid needs ~2× the sweeps of the 48×48 single-zone to propagate the field end to end.
	var max_sweeps: int = MAZE_RELAX_SWEEPS * max(1, int(round(float(n) / 48.0)))
	for _sweep in range(max_sweeps):
		var max_delta: float = 0.0
		for gz in range(n):
			for gx in range(n):
				var idx: int = gz * n + gx
				if _maze_wall[idx] == 1 or is_src[idx] == 1:
					continue
				var acc: float = 0.0
				var cnt: int = 0
				if gx > 0 and _maze_wall[idx - 1] == 0:     acc += _scent_field[idx - 1]; cnt += 1
				if gx < n - 1 and _maze_wall[idx + 1] == 0: acc += _scent_field[idx + 1]; cnt += 1
				if gz > 0 and _maze_wall[idx - n] == 0:     acc += _scent_field[idx - n]; cnt += 1
				if gz < n - 1 and _maze_wall[idx + n] == 0: acc += _scent_field[idx + n]; cnt += 1
				if cnt == 0:
					continue
				var target: float = acc / (float(cnt) + kh2)
				var newv: float = _scent_field[idx] + omega * (target - _scent_field[idx])
				max_delta = max(max_delta, absf(newv - _scent_field[idx]))
				_scent_field[idx] = newv
		if max_delta < MAZE_RELAX_TOL:
			break
	_scent_dirty = false
	if _food_alternate:      # cache this fixed-position field → no recompute (UI lag) when the food cycles back here
		_scent_cache[_food_alt_idx] = _scent_field.duplicate()
	if OS.get_environment("OGMA_MAZE_DUMP") != "":
		_dump_scent_field()
	if _scent_overlay != null:        # UI-only: refresh the heatmap on every field recompute
		_update_scent_heatmap_texture()

func _sample_scent_field(world_pos: Vector3) -> float:
	var n: int = _grid_n
	var cell: float = room_size.x / float(n)
	var fx: float = (world_pos.x + room_size.x * 0.5) / cell - 0.5
	var fz: float = (world_pos.z + room_size.z * 0.5) / cell - 0.5
	var gx0: int = clampi(int(floor(fx)), 0, n - 1)
	var gz0: int = clampi(int(floor(fz)), 0, n - 1)
	var gx1: int = clampi(gx0 + 1, 0, n - 1)
	var gz1: int = clampi(gz0 + 1, 0, n - 1)
	var tx: float = clampf(fx - floor(fx), 0.0, 1.0)
	var tz: float = clampf(fz - floor(fz), 0.0, 1.0)
	var s00: float = _scent_field[gz0 * n + gx0]
	var s10: float = _scent_field[gz0 * n + gx1]
	var s01: float = _scent_field[gz1 * n + gx0]
	var s11: float = _scent_field[gz1 * n + gx1]
	return _reach_scaled(lerp(lerp(s00, s10, tx), lerp(s01, s11, tx), tz))

# Live scent-FIELD-DISTANCE control (the D-value rescue dial).  scent_reach in [0,1]:
# 1.0 = full field (current); <1 shrinks the EFFECTIVE decay length by raising the
# normalized field f∈[0,1] to a power p = 1+(1−reach)·P_MAX (f≈e^{-d/L} → f^p = e^{-d/(L/p)},
# i.e. shorter reach), so scent collapses toward the food and far cells go quiet; 0 = off.
# Hot-mutable mid-run (UI slider) and applied to BOTH the nostril sample and the heatmap.
func _reach_scaled(f: float) -> float:
	if scent_reach >= 0.999:
		return f
	if scent_reach <= 0.0:
		return 0.0
	return pow(clampf(f, 0.0, 1.0), 1.0 + (1.0 - scent_reach) * SCENT_REACH_P_MAX)

func set_scent_reach(v: float) -> void:
	scent_reach = clampf(v, 0.0, 1.0)
	# Live-refresh the top-down heatmap so the field visibly shrinks with the slider.
	if _scent_overlay != null and _scent_overlay.visible:
		_update_scent_heatmap_texture()

func _dump_scent_field() -> void:
	var path: String = OS.get_environment("OGMA_MAZE_DUMP")
	if path == "" or _maze_dumped:
		return
	_maze_dumped = true   # capture the INITIAL field only (the sanity-viz)
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		return
	var walls: Array = []
	for i in range(_maze_wall.size()):
		walls.append(int(_maze_wall[i]))
	var fld: Array = []
	for i in range(_scent_field.size()):
		fld.append(snappedf(_scent_field[i], 0.0001))
	var foods: Array = []
	for nut in nutrients_node.get_children():
		if nut is StaticBody3D and nut.visible:
			foods.append([nut.global_position.x, nut.global_position.z])
	f.store_string(JSON.stringify({
		"n": _grid_n, "cell": room_size.x / float(_grid_n), "room": room_size.x,
		"falloff": scent_falloff_radius, "walls": walls, "field": fld, "food": foods}))
	f.close()
	print("CellWorld: dumped maze scent field → %s" % path)

# --- UI-only top-down scent heatmap overlay (KEY_H) ---------------------------------
func _build_scent_overlay() -> void:
	var plane := PlaneMesh.new()
	plane.size = Vector2(room_size.x, room_size.z)   # XZ plane, +Y normal
	_scent_overlay_mat = StandardMaterial3D.new()
	_scent_overlay_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_scent_overlay_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_scent_overlay_mat.albedo_color = Color(1, 1, 1, 1)
	_scent_overlay_mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR   # smooth the 48² grid
	plane.material = _scent_overlay_mat
	_scent_overlay = MeshInstance3D.new()
	_scent_overlay.name = "ScentHeatmap"
	_scent_overlay.mesh = plane
	_scent_overlay.position = Vector3(0.0, 0.08, 0.0)   # just above the floor
	_scent_overlay.layers = 1 << 2                      # visual layer 3 → top-down only
	_scent_overlay.visible = false                      # toggle on with KEY_H
	add_child(_scent_overlay)

func _scent_heat_color(s: float) -> Color:
	# blue (low) → green → yellow (high); alpha ramps with scent so the floor shows
	# through weak-scent regions.  Walls handled by the caller (transparent).
	s = clampf(s, 0.0, 1.0)
	var r := clampf(2.0 * s - 0.5, 0.0, 1.0)
	var g := clampf(1.5 * s, 0.0, 1.0)
	var b := clampf(0.9 - 1.4 * s, 0.0, 0.9)
	var a := clampf(0.18 + 0.62 * s, 0.0, 0.8)
	return Color(r, g, b, a)

func _update_scent_heatmap_texture() -> void:
	if _scent_overlay == null or _grid_n <= 0:
		return
	var n: int = _grid_n
	var img := Image.create(n, n, false, Image.FORMAT_RGBA8)
	for gz in range(n):
		for gx in range(n):
			var idx: int = gz * n + gx
			# top-down view: screen-up = world −Z; the PlaneMesh+camera map grid row gz
			# directly (no flip) so the heatmap aligns with the food's world position.
			if _maze_wall[idx] == 1:
				img.set_pixel(gx, gz, Color(0, 0, 0, 0))   # wall → transparent (wall mesh shows)
			else:
				img.set_pixel(gx, gz, _scent_heat_color(_reach_scaled(_scent_field[idx])))
	_scent_overlay_mat.albedo_texture = ImageTexture.create_from_image(img)

# CORRIDOR food position: distance corridor_distance from origin, at bearing θ off
# forward (−Z); +θ = to the right (+X).  Random θ∈[−max,+max] when corridor_bearing_max>0
# (the turn rig), else dead-ahead.  Forward=−Z so z=−d·cosθ, x=+d·sinθ.
func _corridor_food_pos() -> Vector3:
	var th: float = 0.0
	if corridor_bearing_max > 0.0:
		th = deg_to_rad(_corridor_rng.randf_range(-corridor_bearing_max, corridor_bearing_max))
	var px: float = corridor_distance * sin(th)
	var pz: float = -corridor_distance * cos(th)
	var py: float = _terrain_height(px, pz) + nutrient_radius + 0.1
	return Vector3(px, py, pz)

# 2026-06-21 — respawn-far variant: food corridor_distance from the BUG's current
# position, at a random bearing ±corridor_bearing_max relative to the bug's CURRENT
# heading (so it must turn) — clamped inside the room.  No bug teleport.
func _corridor_food_pos_near(body: Node3D) -> Vector3:
	var th: float = deg_to_rad(_corridor_rng.randf_range(-corridor_bearing_max, corridor_bearing_max))
	# Bug forward is −Z in its local frame; build the offset in WORLD space using its heading.
	var fwd: Vector3 = -body.global_transform.basis.z
	fwd.y = 0.0
	if fwd.length() < 1e-4:
		fwd = Vector3(0, 0, -1)
	fwd = fwd.normalized()
	var dir: Vector3 = fwd.rotated(Vector3.UP, th)   # +th rotates about UP (left/right)
	# Keep food in the room INTERIOR (clear of walls) so chasing it never lures the
	# bug into a wall-pin — but by REFLECTING the direction off the interior bounds,
	# NOT clamping the distance.  Clamping shortened the offset → food landed on the
	# bug → the overlap-poll ate it instantly (spurious 0-travel eats, hits inflated
	# ~30×).  Reflection preserves the full corridor_distance (genuine turn+approach
	# required) while staying interior.
	var margin: float = maxf(3.0, corridor_distance * 0.75)
	var half_x: float = maxf(corridor_distance, room_size.x * 0.5 - margin)
	var half_z: float = maxf(corridor_distance, room_size.z * 0.5 - margin)
	var bp: Vector3 = body.global_position
	if absf(bp.x + dir.x * corridor_distance) > half_x: dir.x = -dir.x
	if absf(bp.z + dir.z * corridor_distance) > half_z: dir.z = -dir.z
	# final safety clamp (only bites in a tight corner where both reflections fail)
	var px: float = clampf(bp.x + dir.x * corridor_distance, -half_x, half_x)
	var pz: float = clampf(bp.z + dir.z * corridor_distance, -half_z, half_z)
	var py: float = _terrain_height(px, pz) + nutrient_radius + 0.1
	return Vector3(px, py, pz)

func _spawn_nutrients() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = seed_value

	# CORRIDOR probe: one food ahead (bearing 0) or at a random bearing (turn rig).
	if corridor:
		var p: Vector3 = _corridor_food_pos()
		_create_nutrient("Nutrient0", p)
		_active_food = 1
		print("TheCell: CORRIDOR Nutrient0 at (%.2f, %.2f)" % [p.x, p.z])
		return

	# AI-only arena: keep food away from walls (no slide-scoring) and bias it
	# toward the center (dense center → sparse edges).  wall_keepout ≥ mouth-reach
	# + nutrient_radius so a wall-pinned bug can't mouth wall-adjacent food.
	var wall_keepout: float = maxf(nutrient_radius + 1.0, nutrient_wall_margin)
	var lim_x: float = maxf(room_size.x * 0.5 - wall_keepout, 0.1)
	var lim_z: float = maxf(room_size.z * 0.5 - wall_keepout, 0.1)

	# Phase 6.2: when terrain is active, oversample and prefer hilltops (food↔
	# terrain correlation); at amplitude=0 all candidates tie → no sort.
	var oversample: int = nutrient_count * 8 if terrain_amplitude > 0.0 else nutrient_count
	var candidates: Array[Dictionary] = []
	var tries: int = 0
	while candidates.size() < oversample and tries < oversample * 64:
		tries += 1
		var cx: float
		var cz: float
		if nutrient_center_sigma > 0.0:
			var r: float = absf(rng.randfn(0.0, nutrient_center_sigma))   # half-normal radius
			var th: float = rng.randf() * TAU
			cx = r * cos(th)
			cz = r * sin(th)
		else:
			cx = rng.randf_range(-lim_x, lim_x)
			cz = rng.randf_range(-lim_z, lim_z)
		if absf(cx) > lim_x or absf(cz) > lim_z:
			continue   # wall-margin rejection (the anti-slide rule)
		candidates.append({"x": cx, "z": cz, "h": _terrain_height(cx, cz)})
	while candidates.size() < oversample:   # fallback if rejection came up short
		var fx: float = rng.randf_range(-lim_x, lim_x)
		var fz: float = rng.randf_range(-lim_z, lim_z)
		candidates.append({"x": fx, "z": fz, "h": _terrain_height(fx, fz)})
	if terrain_amplitude > 0.0:
		candidates.sort_custom(func(a, b): return float(a["h"]) > float(b["h"]))

	for i in range(nutrient_count):
		var c: Dictionary = candidates[i]
		var px: float = float(c["x"])
		var pz: float = float(c["z"])
		var py: float = _terrain_height(px, pz) + nutrient_radius + 0.1
		_create_nutrient("Nutrient%d" % i, Vector3(px, py, pz))
		print("TheCell: Nutrient%d at (%.2f, %.2f) h=%.2f" % [i, px, pz, c["h"]])
	_active_food = nutrient_count   # Phase 6.9 — scarcity ramp starts here

func _create_nutrient(nutrient_name: String, pos: Vector3) -> void:
	# Phase 6.9.B — food is a StaticBody3D on its OWN layer (2), NON-BLOCKING: the
	# body passes through it and the whiskers (RayCast mask 1 = walls only) do NOT
	# feel it.  Food is EATEN only when the bug's front MOUTH collider (mask 2)
	# touches it (body_controller._build_mouth).  This makes facing-the-food
	# NECESSARY to eat → couples the eat reward to the front-relative scent bearing
	# the critic learns on, while keeping whisker contact a pure WALL signal.
	var nut := StaticBody3D.new()
	nut.name = nutrient_name
	# Layer 2 (NOT the world layer 1): the body's move_and_slide (mask 1) does NOT
	# collide with food, so food is NON-BLOCKING — the bug passes through unless
	# its front MOUTH (mask 2) catches it.  Food was layer 1 (a solid obstacle the
	# bug rammed), but ramming 8 dynamic contacts made Godot's collision-resolution
	# ORDER vary with the UI's object/memory layout → non-deterministic UI runs
	# (headless stayed deterministic; same class of bug as the picrawler never hit
	# because it only collides with walls).  Walls-only collision is deterministic.
	nut.collision_layer = 2     # food layer: mouth detects; body does NOT collide
	nut.collision_mask  = 0     # static — doesn't scan for others

	var sphere_shape       := SphereShape3D.new()
	sphere_shape.radius    = nutrient_radius
	var col_shape          := CollisionShape3D.new()
	col_shape.name         = "Col"
	col_shape.shape        = sphere_shape
	nut.add_child(col_shape)

	var nutrient_mat       := StandardMaterial3D.new()
	nutrient_mat.albedo_color = Color(0.1, 0.9, 0.2)   # bright green
	nutrient_mat.emission_enabled = true
	nutrient_mat.emission = Color(0.05, 0.4, 0.05)      # slight glow

	var sphere_mesh        := SphereMesh.new()
	sphere_mesh.radius     = nutrient_radius
	sphere_mesh.height     = nutrient_radius * 2.0
	sphere_mesh.material   = nutrient_mat
	var mesh_inst          := MeshInstance3D.new()
	mesh_inst.mesh         = sphere_mesh
	nut.add_child(mesh_inst)

	nut.position = pos
	nutrients_node.add_child(nut)
	# No body_entered connection: eating is via the bug's front mouth collider,
	# which calls consume_nutrient() — see body_controller._on_mouth_body_entered.

# ---------------------------------------------------------------------------
# Phase 6.1 — pillar generation
#
# Rejection sampling against three constraints:
#  1. Far enough from origin that the body spawn isn't blocked.
#  2. Far enough from each nutrient that they remain reachable.
#  3. Far enough from prior pillars to leave passable corridors.
# Pillar count is derived from density × max-coverage × floor area, so the
# same density value gives proportionally more pillars in larger rooms.
# ---------------------------------------------------------------------------

func _spawn_pillars() -> void:
	_pillars.clear()
	if obstacle_density <= 0.0:
		return

	var rng := RandomNumberGenerator.new()
	rng.seed = obstacle_seed

	var floor_area: float = room_size.x * room_size.z
	var pillar_area: float = PI * pillar_radius * pillar_radius
	var target_count: int = int((obstacle_density * OBSTACLE_MAX_COVERAGE * floor_area)
								/ max(pillar_area, 0.001))
	if target_count <= 0:
		return

	var margin: float = pillar_radius + wall_thickness * 0.5 + 0.2
	var half_x: float = room_size.x * 0.5 - margin
	var half_z: float = room_size.z * 0.5 - margin

	var nutrient_positions: Array[Vector3] = []
	for n in nutrients_node.get_children():
		if n is StaticBody3D:
			nutrient_positions.append(n.global_position)

	var max_attempts: int = target_count * 20
	var placed: int = 0
	var attempt: int = 0
	while placed < target_count and attempt < max_attempts:
		attempt += 1
		var px: float = rng.randf_range(-half_x, half_x)
		var pz: float = rng.randf_range(-half_z, half_z)
		var p: Vector3 = Vector3(px, 0.0, pz)

		# Constraint 1: keep origin (body spawn) clear.
		if Vector3(0, 0, 0).distance_to(p) < pillar_radius + OBSTACLE_ORIGIN_BUFFER:
			continue
		# Constraint 2: don't bury nutrients.
		var blocked_by_nutrient: bool = false
		for np in nutrient_positions:
			if Vector3(np.x, 0.0, np.z).distance_to(p) < pillar_radius + nutrient_radius + OBSTACLE_NUTRIENT_BUFFER:
				blocked_by_nutrient = true
				break
		if blocked_by_nutrient:
			continue
		# Constraint 3: keep corridors passable between pillars.
		var blocked_by_pillar: bool = false
		for prior in _pillars:
			if Vector3(prior.global_position.x, 0.0, prior.global_position.z).distance_to(p) < pillar_radius * 2.0 + OBSTACLE_MIN_SPACING:
				blocked_by_pillar = true
				break
		if blocked_by_pillar:
			continue

		_create_pillar("Pillar%d" % placed, Vector3(px, pillar_height * 0.5, pz))
		placed += 1

func _create_pillar(pillar_name: String, pos: Vector3) -> void:
	var body := StaticBody3D.new()
	body.name = pillar_name

	var shape := CylinderShape3D.new()
	shape.radius = pillar_radius
	shape.height = pillar_height
	var col := CollisionShape3D.new()
	col.name = "PillarCol"
	col.shape = shape
	body.add_child(col)

	var mat: Material
	if _zone_wall_mat != null:
		mat = _zone_wall_mat            # zone-tinted to match the brain-view
	else:
		var sm := StandardMaterial3D.new()
		sm.albedo_color = Color(0.35, 0.30, 0.40)   # dark stone — visually distinct from walls
		sm.roughness = 0.7
		mat = sm
	var mesh_def := CylinderMesh.new()
	mesh_def.top_radius    = pillar_radius
	mesh_def.bottom_radius = pillar_radius
	mesh_def.height        = pillar_height
	mesh_def.material      = mat
	var mesh_inst := MeshInstance3D.new()
	mesh_inst.name = "PillarMesh"
	mesh_inst.mesh = mesh_def
	body.add_child(mesh_inst)

	body.position = pos
	add_child(body)
	_pillars.append(body)

# ---------------------------------------------------------------------------
# Phase 6.2 — procedural terrain
#
# Heightmap layered on top of the flat floor.  Body capsule physics-interacts
# via move_and_slide over the slopes.  At amplitude=0 the function is a
# constant 0 and _build_terrain() short-circuits — full back-compat.
# ---------------------------------------------------------------------------

func _terrain_height(x: float, z: float) -> float:
	if terrain_amplitude <= 0.0:
		return 0.0
	var k: float = TAU / max(terrain_period, 0.5)
	return terrain_amplitude * sin(k * x) * sin(k * z)

func _build_terrain() -> void:
	if terrain_amplitude <= 0.0:
		return

	# Grid resolution: TERRAIN_GRID_STEP m per cell across the room.  Small
	# enough to resolve the heightmap; large enough that an N×N collision
	# stays cheap.  Default 16/0.5 = 32×32 → 1024 cells.
	var nx: int = int(room_size.x / TERRAIN_GRID_STEP) + 1
	var nz: int = int(room_size.z / TERRAIN_GRID_STEP) + 1
	var origin_x: float = -room_size.x * 0.5
	var origin_z: float = -room_size.z * 0.5

	# Heights packed for HeightMapShape3D: row-major, (nx*nz) floats.
	var heights := PackedFloat32Array()
	heights.resize(nx * nz)
	for j in range(nz):
		for i in range(nx):
			var x: float = origin_x + float(i) * TERRAIN_GRID_STEP
			var z: float = origin_z + float(j) * TERRAIN_GRID_STEP
			heights[j * nx + i] = _terrain_height(x, z)

	_terrain_body = StaticBody3D.new()
	_terrain_body.name = "Terrain"

	var hshape := HeightMapShape3D.new()
	hshape.map_width  = nx
	hshape.map_depth  = nz
	hshape.map_data   = heights
	var col := CollisionShape3D.new()
	col.name  = "TerrainCol"
	col.shape = hshape
	# HeightMapShape3D centres the grid on the node's origin; spans
	# (nx-1)*step in x, (nz-1)*step in z.  Step defaults to 1.0 per cell,
	# so we rescale via the parent transform.
	col.scale = Vector3(TERRAIN_GRID_STEP, 1.0, TERRAIN_GRID_STEP)
	_terrain_body.add_child(col)

	# Visual mesh built from the same height grid.  Matches collision
	# exactly so what the body rolls over is what the player sees.
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for j in range(nz - 1):
		for i in range(nx - 1):
			var x0: float = origin_x + float(i)     * TERRAIN_GRID_STEP
			var x1: float = origin_x + float(i + 1) * TERRAIN_GRID_STEP
			var z0: float = origin_z + float(j)     * TERRAIN_GRID_STEP
			var z1: float = origin_z + float(j + 1) * TERRAIN_GRID_STEP
			var h00: float = heights[j       * nx + i]
			var h10: float = heights[j       * nx + i + 1]
			var h01: float = heights[(j + 1) * nx + i]
			var h11: float = heights[(j + 1) * nx + i + 1]
			# Two triangles per cell.
			st.add_vertex(Vector3(x0, h00, z0))
			st.add_vertex(Vector3(x1, h10, z0))
			st.add_vertex(Vector3(x1, h11, z1))
			st.add_vertex(Vector3(x0, h00, z0))
			st.add_vertex(Vector3(x1, h11, z1))
			st.add_vertex(Vector3(x0, h01, z1))
	st.generate_normals()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.45, 0.50, 0.40)   # mossy green-grey
	mat.roughness    = 0.9
	st.set_material(mat)
	var mesh_inst := MeshInstance3D.new()
	mesh_inst.name = "TerrainMesh"
	mesh_inst.mesh = st.commit()
	_terrain_body.add_child(mesh_inst)

	add_child(_terrain_body)

# Eaten when the bug's front MOUTH collider touches this nutrient
# (body_controller._on_mouth_body_entered → world.consume_nutrient).  Food is a
# StaticBody obstacle otherwise.  _set_nutrient_active toggles BOTH visibility and
# collision so a hidden/eaten nutrient is neither seen nor a phantom obstacle.
func _set_nutrient_active(nutrient: Node3D, active: bool) -> void:
	nutrient.visible = active
	if nutrient is StaticBody3D:
		(nutrient as StaticBody3D).collision_layer = 2 if active else 0

# QUAD progressive reveal: auto-drop the next spiral connector on a fixed sim-tick period.
func _physics_process(_delta: float) -> void:
	if _wall_drop_period > 0 and _walls_dropped < _droppable_walls.size():
		_world_tick += 1
		if _world_tick - _last_drop_tick >= _wall_drop_period:
			_last_drop_tick = _world_tick
			drop_next_wall()

func consume_nutrient(body: Node3D, nutrient: Node3D) -> void:
	if body.has_method("on_nutrient_hit"):
		body.on_nutrient_hit()
	# QUAD progressive reveal: drop the next connector after every N cumulative eats.
	if _wall_drop_on_eats > 0 and _walls_dropped < _droppable_walls.size():
		_eats_since_drop += 1
		if _eats_since_drop >= _wall_drop_on_eats:
			_eats_since_drop = 0
			drop_next_wall()
	if maze:
		# Maze owns its respawn: place the food at a random FREE cell far from the bug
		# (valid diffusion source → the field never goes to zero) → the bug must re-route.
		_scent_dirty = true
		if _food_alternate and _food_positions.size() > 0:
			# hop to the NEXT fixed position (forces travel + moves food out of the mouth)
			_food_alt_idx = (_food_alt_idx + 1) % _food_positions.size()
			var R: float = room_size.x
			nutrient.position = Vector3(_food_positions[_food_alt_idx].x * R, 0.5,
										_food_positions[_food_alt_idx].y * R)
			if _scent_cache.has(_food_alt_idx):
				_scent_field = _scent_cache[_food_alt_idx].duplicate()
				_scent_dirty = false   # cached fixed-position field → skip the GDScript recompute (the per-eat UI lag)
				if _scent_overlay != null: _update_scent_heatmap_texture()
			print("CellWorld: food → position %d (%.2f, %.2f)" % [_food_alt_idx, nutrient.position.x, nutrient.position.z])
		elif _food_homes.has(nutrient.name):
			if food_respawn:
				nutrient.position = _food_homes[nutrient.name]   # FIXED home (predictable env, respawns)
			else:                                                # one-shot per zone (quad): eating empties the zone
				_set_nutrient_active(nutrient, false)
				_active_food = max(0, _active_food - 1)
				# QUAD single-respawn: once ALL zones are cleared, respawn ONE food at a time (a single
				# moving target the bug must re-find across the revealed maze).
				if quad_respawn_single and _active_food == 0:
					_respawn_single_quad_food()
		elif food_respawn and is_instance_valid(nutrient):
			nutrient.position = _maze_respawn_pos(body)
		else:
			_set_nutrient_active(nutrient, false)
			_active_food = max(0, _active_food - 1)
		return
	# CORRIDOR probe: eat = end of a trial.  Reset the bug to the start (re-faced
	# at the food) and keep the food where it is → a fresh trial begins.  The
	# sequence of TRIAL_RESET → next HIT is the learning curve (time-to-food).
	if corridor:
		if corridor_respawn_far and corridor_bearing_max > 0.0 and is_instance_valid(nutrient):
			# 2026-06-21 — NO bug reset: keep the bug where it ate, respawn the food
			# corridor_distance away at a random bearing relative to the bug's CURRENT
			# heading → it must re-perceive + turn + APPROACH from wherever it is.
			# (The body's HIT event marks the trial boundary; no reset emitted.)
			nutrient.position = _corridor_food_pos_near(body)
			return
		if body.has_method("corridor_reset"):
			body.corridor_reset()
		# Turn rig: re-place the food at a NEW random bearing each trial so the bug
		# must re-perceive + re-steer (general policy, not a memorized angle).
		if corridor_bearing_max > 0.0 and is_instance_valid(nutrient):
			nutrient.position = _corridor_food_pos()
		return   # food stays active (re-placed) for the next trial
	_set_nutrient_active(nutrient, false)
	# 2026-06-18 — board-clearing mode: eaten food is GONE for good.  The board
	# starts at nutrient_count and shrinks; ticks-to-clear (or food-remaining at
	# episode end) is the efficiency metric.  Announce the clear once.
	if not food_respawn:
		_active_food = max(0, _active_food - 1)
		if _active_food == 0:
			print(JSON.stringify({"event": "BOARD_CLEARED", "food_total": nutrient_count}))
		return
	# Phase 6.9 — scarcity ramp: shrink the pool toward the floor, then hold the
	# floor with a delayed, far respawn.
	if scarcity_enabled:
		if _active_food > scarcity_food_floor and _respawn_rng.randf() >= scarcity_respawn_ratio:
			_active_food -= 1
			return   # eaten nutrient is removed (stays hidden+un-collidable → out of play)
		var at_floor: bool = _active_food <= scarcity_food_floor
		var delay: float = _respawn_rng.randf_range(scarcity_floor_min_delay, scarcity_floor_max_delay) if at_floor else 1.5
		await get_tree().create_timer(delay).timeout
		if not is_instance_valid(body) or not is_instance_valid(nutrient):
			return
		var dist: float = scarcity_floor_distance if at_floor else respawn_min_distance
		nutrient.position = _pick_respawn_position(body.global_position, dist)
		_respawn_count += 1
		_set_nutrient_active(nutrient, true)
		return
	await get_tree().create_timer(1.5).timeout
	if respawn_randomize:
		var agent_pos: Vector3 = body.global_position
		var new_pos: Vector3 = _pick_respawn_position(agent_pos)
		nutrient.position = new_pos
		_respawn_count += 1
	_set_nutrient_active(nutrient, true)

# QUAD single-respawn: after all 4 zone foods are cleared, reactivate ONE nutrient at a random zone
# position — a single moving target the bug must re-find across the now-revealed maze (exercises the
# 3-loop division: play explores, planner patrols/routes the known zones, klino closes on scent).
func _respawn_single_quad_food() -> void:
	if _food_positions.is_empty():
		return
	var idx: int = _respawn_rng.randi_range(0, _food_positions.size() - 1)
	var R: float = room_size.x
	var pos := Vector3(_food_positions[idx].x * R, 0.5, _food_positions[idx].y * R)
	for nut in nutrients_node.get_children():
		if nut is Node3D and not (nut as Node3D).visible:      # reuse an inactive (eaten) nutrient node
			(nut as Node3D).position = pos
			_set_nutrient_active(nut, true)
			_active_food = 1
			_scent_dirty = true                                # force the scent field to recompute at the new source
			_respawn_count += 1
			print(JSON.stringify({"event": "QUAD_RESPAWN", "zone": idx, "respawn_count": _respawn_count}))
			return

# v5.4.M — choose a respawn location that is:
#   (a) at least respawn_min_distance from the agent
#   (b) within the room interior (margin = nutrient_radius + 1.0)
#   (c) on the terrain surface (Y derived from _terrain_height)
#   (d) deterministic per (seed_value, _respawn_nonce) so paired-seed
#       runs produce identical respawn trajectories.
# Tries up to 16 candidates; if none clear the distance bound (rare in
# small rooms) the last candidate is returned anyway — better to
# respawn close than to crash.
func _pick_respawn_position(agent_pos: Vector3, min_dist: float = -1.0) -> Vector3:
	var margin: float = nutrient_radius + 1.0
	var half_x: float = room_size.x * 0.5 - margin
	var half_z: float = room_size.z * 0.5 - margin
	var eff_min: float = respawn_min_distance if min_dist < 0.0 else min_dist
	var min_d2: float = eff_min * eff_min
	var px: float = 0.0
	var pz: float = 0.0
	for _attempt in range(16):
		_respawn_nonce += 1
		px = _respawn_rng.randf_range(-half_x, half_x)
		pz = _respawn_rng.randf_range(-half_z, half_z)
		var dx: float = px - agent_pos.x
		var dz: float = pz - agent_pos.z
		if dx * dx + dz * dz >= min_d2:
			break
	var py: float = _terrain_height(px, pz) + nutrient_radius + 0.1
	return Vector3(px, py, pz)

# ---------------------------------------------------------------------------
# Top-down camera (orthographic map view).  Sized to the room with a margin;
# rebuilt when the mode changes the room size.
# ---------------------------------------------------------------------------

func _build_topdown_cam() -> void:
	_topdown_cam = Camera3D.new()
	_topdown_cam.name        = "TopDownCam"
	_topdown_cam.projection  = Camera3D.PROJECTION_ORTHOGONAL
	_topdown_cam.size        = max(room_size.x, room_size.z) + 4.0
	_topdown_cam.position    = Vector3(0.0, room_size.y + 6.0, 0.0)
	_topdown_cam.rotation    = Vector3(-PI * 0.5, 0.0, 0.0)
	# Layers: 1 = world (floor/walls/nutrients), 2 = ceiling, 3 = body mesh.
	# Top-down sees world + body but not ceiling.
	_topdown_cam.cull_mask   = (1 << 0) | (1 << 2)
	_topdown_cam.current     = false
	add_child(_topdown_cam)

# ---------------------------------------------------------------------------
# Runtime mode switching.  Pressing 1/2/3 reconfigures the world without
# restarting the host: rooms/nutrients are torn down and rebuilt; the body
# is reset to the origin so it doesn't end up wedged inside a new wall.
# ---------------------------------------------------------------------------

# HONEST RESPAWN — food must NEVER respawn in the same place twice (a recurring fix; see memory
# cell_food_respawn_same_place). A respawning FIXED-position env therefore becomes ONE food that
# CYCLES the positions (food_alternate) — never multiple fixed food that reappear under the mouth
# (instant re-eat: a single arrival inflates the eat count, e.g. 13 "eats" from one eat). Quad
# (food_respawn=false + quad_respawn_single) already respawns a single moving target and is untouched.
func _enforce_honest_respawn() -> void:
	if food_respawn and _food_positions.size() >= 2 and not _food_alternate:
		print("CellWorld: honest-respawn → 1 food cycling %d positions (was %d fixed food; prevents instant re-eat)" % [_food_positions.size(), nutrient_count])
		_food_alternate = true
		nutrient_count = 1

func apply_mode(mode_name: String) -> void:
	# 2026-07-09 — MAP SWITCH (adaptability demo): 1/2/3 hot-swap the whole ENVIRONMENT
	# (maze layout + room + food + scent field) UNDER THE RUNNING BRAIN, so any config can be
	# dropped into easy→medium→hard maps live. The brain/modules are untouched — only the world
	# geometry + scent field rebuild (mirrors _ready's build sequence, minus re-reading the config).
	match mode_name:
		"easy":     # small OPEN cell, 2 food (the the_cell_arbiter_open_klino2 env)
			maze = true; maze_layout = "open"
			room_size = Vector3(24.0, room_size.y, 24.0); scent_falloff_radius = 12.0
			nutrient_count = 1
			_food_positions = [Vector2(0.0, 0.28), Vector2(0.0, -0.28)]
			food_respawn = true; quad_respawn_single = false; _food_alternate = true   # 1 food that HOPS N/S (never re-eat in place)
			_wall_drop_period = 0; _zone_colors = false; _zone_grid = 4
		"medium":   # L-BEND maze, 2 food (one direct, one round-the-bend)
			maze = true; maze_layout = "lbend"
			room_size = Vector3(24.0, room_size.y, 24.0); scent_falloff_radius = 12.0
			nutrient_count = 1
			_food_positions = [Vector2(-0.28, 0.36), Vector2(0.25, -0.2)]
			food_respawn = true; quad_respawn_single = false; _food_alternate = true   # 1 food alternating L-bend sides (never re-eat in place)
			_wall_drop_period = 0; _zone_colors = false; _zone_grid = 4
		"hard":     # QUAD maze: 4 zones, droppable connectors (KEY_D to reveal), one food per zone
			maze = true; maze_layout = "quad"
			room_size = Vector3(48.0, room_size.y, 48.0); scent_falloff_radius = 15.0
			nutrient_count = 4
			_food_positions = [Vector2(-0.25, -0.25), Vector2(0.25, -0.25), Vector2(0.25, 0.25), Vector2(-0.25, 0.25)]
			food_respawn = false; quad_respawn_single = true
			_wall_drop_period = 3000; _food_alternate = false; _zone_colors = true; _zone_grid = 4
		_:
			push_warning("apply_mode: unknown mode '%s'" % mode_name)
			return

	_enforce_honest_respawn()   # food never respawns in the same place twice (safety net)
	# --- tear down ALL world geometry (keep Body + Nutrients holder) ---
	# remove_child() is immediate; queue_free() deletes at end of frame — avoids the rebuild
	# racing the old same-named node's deletion (which auto-renames the new one).
	_detach_and_free(get_node_or_null("Room"))
	_detach_and_free(get_node_or_null("RoomLight"))
	if _topdown_cam:
		_detach_and_free(_topdown_cam)
		_topdown_cam = null
	if _scent_overlay:
		_detach_and_free(_scent_overlay)
		_scent_overlay = null
	for w in get_children():
		if str(w.name).begins_with("MazeWall") or str(w.name).begins_with("DropWall"):
			_detach_and_free(w)
	for child in nutrients_node.get_children():
		nutrients_node.remove_child(child)
		child.queue_free()
	for pillar in _pillars:
		_detach_and_free(pillar)
	_pillars.clear()
	if _terrain_body:
		_detach_and_free(_terrain_body)
		_terrain_body = null

	# --- rebuild (mirrors _ready's build sequence; scent field recomputes lazily via _scent_dirty) ---
	_zone_wall_mat = _make_zone_wall_material() if _zone_colors else null
	_build_room()
	if maze:
		_build_maze()          # interior walls + occupancy grid; sets _scent_dirty
		if DisplayServer.get_name() != "headless":
			_build_scent_overlay()
	_build_terrain()
	_build_topdown_cam()
	_spawn_nutrients()
	if maze and _food_positions.size() > 0:
		_place_fixed_foods()   # move the nutrients onto the fixed maze positions
	_spawn_pillars()

	# --- drop the bug into a SAFE spawn for the new map (never inside a wall) ---
	var body := get_node_or_null("Body")
	if body:
		var R: float = room_size.x
		var spawn: Vector3
		match mode_name:
			"hard":   spawn = Vector3(-0.13 * R, 0.5, -0.13 * R)   # SW zone Z1 (boxed until KEY_D)
			"medium": spawn = Vector3(0.0, 0.5, -R * 0.35)         # south, clear of the L wall
			_:        spawn = Vector3(0.0, 0.5, 0.0)               # open centre
		body.global_position = spawn
		if "heading" in body:
			body.heading = 0.0
		var fpv := body.get_node_or_null("Camera3D") as Camera3D
		if fpv:
			fpv.make_current()
	print("TheCell: MAP → %s  (layout=%s room=%.0f nutrients=%d falloff=%.0f)" % [
		mode_name, maze_layout, room_size.x, nutrient_count, scent_falloff_radius])

func _detach_and_free(n: Node) -> void:
	if n == null:
		return
	if n.get_parent():
		n.get_parent().remove_child(n)
	n.queue_free()

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_1: apply_mode("easy")
			KEY_2: apply_mode("medium")
			KEY_3: apply_mode("hard")
			KEY_P: _cycle_pillar_density()
			KEY_T: _cycle_terrain_amplitude()
			KEY_H: _toggle_scent_heatmap()
			KEY_D: drop_next_wall()   # QUAD: open the next spiral connector (progressive reveal)

# Toggle the UI-only top-down scent heatmap overlay (built only outside headless).
func _toggle_scent_heatmap() -> void:
	if _scent_overlay == null:
		return
	if _scent_dirty:
		_recompute_scent_field()      # ensure the texture reflects the current field
	elif _scent_overlay_mat and _scent_overlay_mat.albedo_texture == null:
		_update_scent_heatmap_texture()
	_scent_overlay.visible = not _scent_overlay.visible
	print("CellWorld: scent heatmap %s" % ("ON" if _scent_overlay.visible else "OFF"))

# Advance through _PILLAR_DENSITY_CYCLE and rebuild pillars in place.
# Used by interactive runs where the OGMA_OBSTACLE_DENSITY env var
# isn't set; the smoke harness keeps using the env var for sweeps.
func _cycle_pillar_density() -> void:
	_pillar_cycle_idx = (_pillar_cycle_idx + 1) % _PILLAR_DENSITY_CYCLE.size()
	obstacle_density = _PILLAR_DENSITY_CYCLE[_pillar_cycle_idx]
	for pillar in _pillars:
		_detach_and_free(pillar)
	_pillars.clear()
	_spawn_pillars()
	print("TheCell: pillar density → %.2f (%d pillars)" % [
		obstacle_density, _pillars.size()])

# KEY_T: cycle the heightmap amplitude.  Tears down the terrain mesh,
# nutrients, and pillars; rebuilds in order so nutrients can relocate to
# hilltops and pillars get re-placed against the new nutrient layout.
# Body is reset to origin (and lifted to the new terrain height there) so
# it doesn't end up wedged inside a freshly-grown hill.
func _cycle_terrain_amplitude() -> void:
	_terrain_cycle_idx = (_terrain_cycle_idx + 1) % _TERRAIN_AMPLITUDE_CYCLE.size()
	terrain_amplitude = _TERRAIN_AMPLITUDE_CYCLE[_terrain_cycle_idx]
	if _terrain_body:
		_detach_and_free(_terrain_body)
		_terrain_body = null
	for child in nutrients_node.get_children():
		nutrients_node.remove_child(child)
		child.queue_free()
	for pillar in _pillars:
		_detach_and_free(pillar)
	_pillars.clear()
	_build_terrain()
	_spawn_nutrients()
	_spawn_pillars()
	# Lift body above whatever terrain is now under origin.  Otherwise a
	# newly-grown hill could phase through the capsule.
	var body := get_node_or_null("Body")
	if body:
		body.global_position = Vector3(0.0, _terrain_height(0.0, 0.0) + 1.0, 0.0)
		if "heading" in body:
			body.heading = 0.0
	print("TheCell: terrain amplitude → %.2f" % terrain_amplitude)

# ---------------------------------------------------------------------------
# Scent — radial gradient sampling
#
# Auditability: the brain only sees scalar concentration at body-relative
# nostril positions; never a vector toward the goal.  The body's heading
# rotates the nostril ring so any direction information is implicit and
# must be learned by correlating proprio (heading) with scent differences.
# ---------------------------------------------------------------------------

func _scent_at(world_pos: Vector3) -> float:
	# MAZE: sample the screened-Poisson diffusion field (gradient flows along corridors,
	# never through walls).  Recompute lazily on the first sample after a food change.
	if maze and _grid_n > 0:
		if _scent_dirty:
			_recompute_scent_field()
		return _sample_scent_field(world_pos)
	var s := 0.0
	var inv_sigma := 1.0 / scent_sigma
	var have_pillars: bool = _pillars.size() > 0
	var space_state: PhysicsDirectSpaceState3D = (
		get_world_3d().direct_space_state if have_pillars else null)
	for n in nutrients_node.get_children():
		if n is StaticBody3D and n.visible:
			var d: float = world_pos.distance_to(n.global_position)
			var contrib: float = exp(-d * inv_sigma)   # diffusion-decay envelope (long tail, cusp near)
			# Phase 6.1 — partial occlusion: a pillar between sample point
			# and nutrient attenuates that nutrient's contribution.  Single
			# raycast per (sample, nutrient); the body itself is on a
			# different collision layer so it doesn't block its own scent.
			if have_pillars:
				var query := PhysicsRayQueryParameters3D.create(
					n.global_position, world_pos, 1)
				query.collide_with_areas  = false
				query.collide_with_bodies = true
				var hit := space_state.intersect_ray(query)
				if hit and hit.collider in _pillars:
					contrib *= obstacle_scent_attenuation
			s += contrib
	return s

## Egocentric geometry of the nearest active nutrient, for the forward-dynamics
## probe: [lateral (+right), forward (+ahead), distance].  Observer-only telemetry.
func nearest_food_egocentric(body_pos: Vector3, body_basis: Basis) -> PackedFloat64Array:
	var best_d := INF
	var best_local := Vector3.ZERO
	for n in nutrients_node.get_children():
		if n is StaticBody3D and n.visible:
			var d: float = body_pos.distance_to(n.global_position)
			if d < best_d:
				best_d = d
				best_local = body_basis.inverse() * (n.global_position - body_pos)
	if best_d == INF:
		return PackedFloat64Array([0.0, 0.0, -1.0])
	return PackedFloat64Array([best_local.x, -best_local.z, best_d])

## Active (un-eaten) food remaining — for the ticks-to-clear / food-remaining
## efficiency metric and live UI readout.  In no-respawn mode this counts down to 0.
func active_food_count() -> int:
	return _active_food

## World positions of every active (visible) nutrient.  Read-only observer telemetry
## consumed by the UI-only top-down planner debug overlay (body_controller KEY_J).
func active_food_world_positions() -> Array:
	var out: Array = []
	for n in nutrients_node.get_children():
		if n is StaticBody3D and n.visible:
			out.append(n.global_position)
	return out

## Ground-truth egocentric bearing to the nearest ACTIVE nutrient, for homing
## telemetry (the corr(food_bearing, brain_steer) ignition metric).  Returns the
## normalized LATERAL component in body-local frame: +1 = food hard right,
## -1 = hard left, ~0 = dead ahead/behind.  No active food -> 0.  This is an
## OBSERVER signal only (logged in diag); it never enters the brain.
func nearest_food_local_lateral(body_pos: Vector3, body_basis: Basis) -> float:
	var best_d := INF
	var best_local := Vector3.ZERO
	for n in nutrients_node.get_children():
		if n is StaticBody3D and n.visible:
			var d: float = body_pos.distance_to(n.global_position)
			if d < best_d:
				best_d = d
				best_local = body_basis.inverse() * (n.global_position - body_pos)
	if best_d == INF:
		return 0.0
	var horiz := sqrt(best_local.x * best_local.x + best_local.z * best_local.z)
	if horiz < 1e-4:
		return 0.0
	return clampf(best_local.x / horiz, -1.0, 1.0)

func compute_scent_vector(body_pos: Vector3, body_basis: Basis) -> PackedFloat64Array:
	var out := PackedFloat64Array()
	for i in range(NOSTRIL_COUNT):
		var ang: float = TAU * float(i) / float(NOSTRIL_COUNT)
		var local := Vector3(cos(ang) * NOSTRIL_RADIUS, 0.0, sin(ang) * NOSTRIL_RADIUS)
		out.append(_scent_at(body_pos + body_basis * local))
	return out

## Heading-INVARIANT scalar scent at the bug's CENTRE (position-only). The 8-nostril ring in
## compute_scent_vector is offset by body_basis, so it ROTATES with the bug — correct for the
## DIRECTIONAL scent compass (left/right gradient), but its per-tick MAX spins with the body:
## a bug turning in place sees a phantom rising/falling scent. The SCALAR scent LEVEL that klino's
## methylation and the L2 arbiter's proximity term consume must be frame-INDEPENDENT (a "bath",
## not a beam), so sample the diffusion field once at the body centre — a ray from the bug's
## centre to the source, never tied to its facing.
func compute_scent_scalar(body_pos: Vector3) -> float:
	return _scent_at(body_pos)
# Phase 6.7 NOTE — the scent→bearing reduction (the "scent compass") now lives
# in the brain as the ScentCompass C++ module, which subscribes the raw
# `reality.proprio.scent` ring above and publishes `percept.scent_compass`.  The
# body stays a raw-sensor publisher (nothing crosses the boundary that isn't in
# the register_source list); the perception is auditable in the brain graph.
