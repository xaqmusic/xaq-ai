extends Node
## CurriculumManager — autoload singleton that chains stages of reward
## shaping for an embodied learner.  The manager only stores the
## CURRENT stage's parameter dict; bodies subscribe to stage_changed
## and apply the overrides to their own @export reward knobs.
##
## Default mode is MANUAL: the manager never advances on its own.
## Pre-conditions for auto-advance are evaluated only when
## set_auto_advance(true) is called, and even then the body must opt-in
## by calling tick_competence(metrics) with current rolling-window
## metrics each diag tick.
##
## Methodology rules (enforced here so an A/B can't accidentally
## contaminate itself):
##   1. The active curriculum file path + current stage index are
##      published into ExperimentConfig.curriculum_metadata so every
##      run logs them in its manifest.
##   2. In headless mode (OGMA_PICRAWLER_HEADLESS=1) the keybindings
##      that drive prev/next are inert; only env-var driven goto_stage
##      and tick_competence (when auto_advance) can change stage.
##   3. Trainer pulses are NOT a curriculum concern — see trainer_panel.gd.
##      The two systems are independent so logs can distinguish them.
##
## Stage JSON schema (list at top level):
##   [
##     {
##       "name": "stage_1_stand_wobbly",
##       "overrides": {
##         "target_height": 0.085,
##         "height_penalty_scale": 0.0,
##         "walk_hit_rate": 0.0
##       },
##       "advance_when": {
##         "metric": "longest_upright_physics_ticks",
##         "op": "ge",
##         "value": 3000,
##         "window_ticks": 5000
##       }
##     },
##     ...
##   ]
##
## advance_when is optional — stages without it never auto-advance.
## Supported ops: ge, le, gt, lt, eq.  The body computes window-based
## metrics on its side and passes them through tick_competence().

signal stage_changed(idx: int, name: String, overrides: Dictionary)
signal curriculum_loaded(path: String, n_stages: int)

const _ALLOWED_OPS: Array = ["ge", "le", "gt", "lt", "eq"]

var stages: Array = []
var curriculum_path: String = ""
var current_idx: int = 0
var auto_advance: bool = false
# Latest metrics dict passed in via tick_competence().  Stored
# regardless of auto_advance state so the UI can show progress toward
# the next transition even when the experimenter is driving stages
# manually.  Keys depend on what the body emits — typically:
#   best_cumulative_alive_ticks, cumulative_alive_ticks, chassis_y,
#   max_distance_from_origin.
var last_metrics: Dictionary = {}
# Per-stage elapsed timer (in body ticks, 60 Hz).  Parallel array to
# `stages`.  Accumulates across visits — re-entering a stage continues
# its counter rather than resetting it.  Cleared only when a new
# curriculum file is loaded.  Tick deltas come from tick_competence
# (every diag interval ≈ 1 s of sim time at default settings).
var time_in_stage_ticks: Array[int] = []
var _last_competence_tick: int = -1

# Diag-tick stamp of the last stage change.  Auto-advance is gated by
# stage_changed_at so a too-eager metric can't bounce between stages
# inside a few ticks.  Tunable but kept conservative (500 ticks ≈ 10 s).
var _stage_changed_at: int = 0
const _MIN_TICKS_BETWEEN_AUTO_ADVANCE: int = 500

# Once-per-stage tracker for the "metric not emitted by body" warning
# so we don't spam the console every diag tick when an advance_when
# rule names a metric the body never publishes.
var _missing_metric_warned: Dictionary = {}

func _ready() -> void:
    # The autoload runs in every scene including the launcher.  Stay
    # silent unless explicitly used.
    process_mode = Node.PROCESS_MODE_ALWAYS

func load_curriculum_file(res_path: String) -> bool:
    if res_path == "":
        return false
    if not FileAccess.file_exists(res_path):
        push_warning("CurriculumManager: file not found: %s" % res_path)
        return false
    var f := FileAccess.open(res_path, FileAccess.READ)
    if f == null:
        push_warning("CurriculumManager: cannot open %s" % res_path)
        return false
    var text: String = f.get_as_text()
    f.close()
    var parsed: Variant = JSON.parse_string(text)
    if not (parsed is Array):
        push_warning("CurriculumManager: %s root must be a JSON array" % res_path)
        return false
    var validated: Array = []
    for i in range(parsed.size()):
        var s: Variant = parsed[i]
        if not (s is Dictionary):
            push_warning("CurriculumManager: stage %d not a dict, skipped" % i)
            continue
        var sd: Dictionary = s
        if not sd.has("overrides"):
            sd["overrides"] = {}
        if not sd.has("name"):
            sd["name"] = "stage_%d" % i
        validated.append(sd)
    if validated.is_empty():
        push_warning("CurriculumManager: %s has no valid stages" % res_path)
        return false
    stages = validated
    curriculum_path = res_path
    current_idx = 0
    _stage_changed_at = 0
    time_in_stage_ticks = []
    time_in_stage_ticks.resize(stages.size())
    for i in range(stages.size()):
        time_in_stage_ticks[i] = 0
    _last_competence_tick = -1
    _missing_metric_warned.clear()
    emit_signal("curriculum_loaded", res_path, stages.size())
    _emit_current()
    return true

func clear() -> void:
    stages = []
    curriculum_path = ""
    current_idx = 0
    auto_advance = false

func has_curriculum() -> bool:
    return stages.size() > 0

func current_stage() -> Dictionary:
    if not has_curriculum():
        return {}
    return stages[current_idx]

func current_overrides() -> Dictionary:
    var s: Dictionary = current_stage()
    return s.get("overrides", {})

func current_name() -> String:
    var s: Dictionary = current_stage()
    return str(s.get("name", ""))

func n_stages() -> int:
    return stages.size()

func next_stage() -> bool:
    if not has_curriculum():
        return false
    if current_idx >= stages.size() - 1:
        return false
    current_idx += 1
    _emit_current()
    return true

func prev_stage() -> bool:
    if not has_curriculum():
        return false
    if current_idx <= 0:
        return false
    current_idx -= 1
    _emit_current()
    return true

func goto_stage(idx: int) -> bool:
    if not has_curriculum():
        return false
    if idx < 0 or idx >= stages.size():
        return false
    if idx == current_idx:
        return false
    current_idx = idx
    _emit_current()
    return true

func set_auto_advance(b: bool) -> void:
    auto_advance = b

func tick_competence(tick: int, metrics: Dictionary) -> void:
    ## Called by the body on every diag tick when curriculum is loaded.
    ## Always stores the latest metrics for UI progress display.  Only
    ## evaluates the advance_when rule when auto_advance is on AND the
    ## current stage has an advance_when block AND the metric satisfies
    ## the condition AND enough ticks have passed since the last
    ## transition.
    last_metrics = metrics
    # Accumulate elapsed time in the current stage (independent of
    # auto_advance — the timer runs whether the experimenter is driving
    # stages manually or letting the rule advance them).
    if has_curriculum() and current_idx < time_in_stage_ticks.size():
        if _last_competence_tick >= 0:
            var dt: int = max(0, tick - _last_competence_tick)
            time_in_stage_ticks[current_idx] += dt
        _last_competence_tick = tick
    if not auto_advance:
        return
    if not has_curriculum():
        return
    var s: Dictionary = current_stage()
    if not s.has("advance_when"):
        return
    var rule: Dictionary = s["advance_when"]
    var metric_name: String = str(rule.get("metric", ""))
    var op: String = str(rule.get("op", "ge"))
    var value: float = float(rule.get("value", 0.0))
    if not metrics.has(metric_name):
        # Emit once-per-stage so the user sees "you're auto-advancing on
        # a metric the body never publishes."  Without this the system
        # silently waits forever.
        if not _missing_metric_warned.has(current_idx):
            _missing_metric_warned[current_idx] = true
            push_warning("CurriculumManager: stage %d (%s) gates on metric '%s' but the body never emits it. Available: %s"
                % [current_idx, current_name(), metric_name, metrics.keys()])
        return
    if not (op in _ALLOWED_OPS):
        push_warning("CurriculumManager: invalid op '%s' in stage %d" % [op, current_idx])
        return
    if tick - _stage_changed_at < _MIN_TICKS_BETWEEN_AUTO_ADVANCE:
        return
    var m_val: float = float(metrics[metric_name])
    var triggered := false
    match op:
        "ge": triggered = m_val >= value
        "le": triggered = m_val <= value
        "gt": triggered = m_val >  value
        "lt": triggered = m_val <  value
        "eq": triggered = abs(m_val - value) < 1e-6
    if triggered:
        if current_idx < stages.size() - 1:
            current_idx += 1
            _emit_current()
            # Stamp the advance tick with the SAME clock the cooldown
            # check reads (the body's tick_counter, passed in here).  This
            # is the same value _emit_current would derive via its
            # fallback path, but writing it here keeps the relationship
            # explicit at the auto-advance call-site.
            _stage_changed_at = tick
            print("CurriculumManager: auto-advanced to stage %d (%s) at tick %d (metric %s=%s %s %s)" % [
                current_idx, current_name(), tick, metric_name, m_val, op, value])

func _emit_current() -> void:
    # Anchor _stage_changed_at to the same body-tick clock that
    # tick_competence uses for the cooldown check (line 215).  Using
    # Engine.get_physics_frames() here was a two-clock mismatch: the
    # engine counter has been running since scene-start (before the
    # body's _ready), so at curriculum-load time it could be 100+
    # frames ahead of the body's tick_counter — making the first
    # auto-advance window negative and silently blocking the rule for
    # the first 10–15 sim seconds.  We use the most-recent body tick
    # we've seen (or 0 if none yet) so cooldown math stays in one clock.
    _stage_changed_at = max(0, _last_competence_tick)
    emit_signal("stage_changed", current_idx, current_name(), current_overrides())

func current_progress() -> Dictionary:
    ## Returns progress info for the current stage:
    ##   { terminal:bool, metric:str, op:str, value:float, current:float, ratio:float }
    ## Terminal stages (no advance_when) return {terminal: true}.
    ## ratio is clamped to [0, 1].  For ge/gt ops, ratio = current/threshold.
    ## For le/lt ops, ratio = threshold/current (smaller-is-better).
    if not has_curriculum():
        return {}
    var s: Dictionary = current_stage()
    if not s.has("advance_when"):
        return {"terminal": true}
    var rule: Dictionary = s["advance_when"]
    var metric_name: String = str(rule.get("metric", ""))
    var op: String = str(rule.get("op", "ge"))
    var thresh: float = float(rule.get("value", 0.0))
    var cur: float = float(last_metrics.get(metric_name, 0.0))
    var ratio: float = 0.0
    if op == "ge" or op == "gt":
        if thresh > 1e-9:
            ratio = clamp(cur / thresh, 0.0, 1.0)
    elif op == "le" or op == "lt":
        if cur > 1e-9:
            ratio = clamp(thresh / cur, 0.0, 1.0)
        else:
            ratio = 1.0   # current already <= threshold trivially
    elif op == "eq":
        ratio = 1.0 if abs(cur - thresh) < 1e-6 else 0.0
    return {
        "terminal": false,
        "metric":   metric_name,
        "op":       op,
        "value":    thresh,
        "current":  cur,
        "ratio":    ratio,
    }
