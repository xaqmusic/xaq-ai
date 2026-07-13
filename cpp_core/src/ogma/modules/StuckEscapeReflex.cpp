#include "ogma/modules/StuckEscapeReflex.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("StuckEscapeReflex: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("StuckEscapeReflex: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("StuckEscapeReflex: param '" + key + "' must be a string");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("StuckEscapeReflex: param '" + key + "' must be bool");
}

} // namespace

StuckEscapeReflex::StuckEscapeReflex()  = default;
StuckEscapeReflex::~StuckEscapeReflex() = default;

std::string_view StuckEscapeReflex::type_name() const { return "StuckEscapeReflex"; }

std::vector<TopicSpec> StuckEscapeReflex::input_topics() const {
    return {
        TopicSpec{imu_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> StuckEscapeReflex::output_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{"events.wall_stuck", std::type_index(typeid(EnvEvent))},
    };
    if (enable_pulse_) {
        std::string l = output_topic_left_.empty()  ? std::string(topics::kActionLeft)
                                                    : output_topic_left_;
        std::string r = output_topic_right_.empty() ? std::string(topics::kActionRight)
                                                    : output_topic_right_;
        v.push_back(TopicSpec{l, std::type_index(typeid(ActionOut))});
        v.push_back(TopicSpec{r, std::type_index(typeid(ActionOut))});
    }
    return v;
}

ParamSchema StuckEscapeReflex::params_schema() const {
    return {
        {"imu_topic", ParamMutability::ConstructionOnly,
            "Proprio topic carrying body velocity (vx_index, vz_index pick the dims)",
            ParamValue{std::string("reality.proprio.imu")}},
        {"vx_index", ParamMutability::ConstructionOnly,
            "Index of forward (or side) velocity component",
            ParamValue{int64_t{2}}},
        {"vz_index", ParamMutability::ConstructionOnly,
            "Index of orthogonal velocity component",
            ParamValue{int64_t{3}}},
        {"move_speed_reference", ParamMutability::HotMutable,
            "Reference speed for severity normalisation (severity = 1 - eff_speed/this)",
            ParamValue{3.0}},
        {"ang_vel_topic", ParamMutability::ConstructionOnly,
            "Body angular-velocity proprio topic. When set, rotation counts as motion "
            "(eff_speed = |v_trans| + ang_weight·|ω|) so turning in place is NOT 'stuck'. "
            "Empty = translation-only (legacy byte-identical).", ParamValue{std::string("")}},
        {"ang_weight", ParamMutability::HotMutable,
            "(legacy velocity mode) Weight on |ω| in the motion measure. 0 = translation-only.",
            ParamValue{0.0}},
        {"efference_topic", ParamMutability::ConstructionOnly,
            "Commanded-velocity (efference copy) proprio topic, e.g. reality.proprio.motor_efference. "
            "When set → MISMATCH mode: stuck = |afferent| << |intended| (self-normalizing, no velocity "
            "threshold). Empty = legacy velocity mode.", ParamValue{std::string("")}},
        {"intent_floor", ParamMutability::HotMutable,
            "Mismatch mode: minimum |intended| (commanded speed) to evaluate stuckness — below it the "
            "bug isn't trying to move, so it can't be 'stuck'.", ParamValue{0.1}},
        {"hunger_topic", ParamMutability::ConstructionOnly,
            "Energy ProprioToken (e.g. reality.proprio.energy). When set, the escape fires only when "
            "HUNGRY × inactive — idle-when-sated is rest, not stuck. Empty = ungated (legacy).",
            ParamValue{std::string("")}},
        {"sated_energy", ParamMutability::HotMutable,
            "Energy setpoint; hunger = clamp((sated_energy − energy)/sated_energy).", ParamValue{0.8}},
        {"hunger_floor", ParamMutability::HotMutable,
            "Hunger must exceed this to fire (so a barely-hungry resting bug isn't perturbed).", ParamValue{0.05}},
        {"noise_mag", ParamMutability::HotMutable,
            "Random ± noise added to the escape pulse — jitters the system out of the trap "
            "rather than a clean rotation only. 0 = no noise (legacy).", ParamValue{0.0}},
        {"severity_threshold", ParamMutability::HotMutable,
            "Window-averaged severity above which events.wall_stuck fires",
            ParamValue{0.5}},
        {"window_ticks", ParamMutability::HotMutable,
            "Sliding window length over which speed is averaged",
            ParamValue{int64_t{60}}},
        {"refractory_ticks", ParamMutability::HotMutable,
            "Ticks of suppression after a stuck event fires",
            ParamValue{int64_t{60}}},
        {"enable_pulse", ParamMutability::ConstructionOnly,
            "Phase 6.6.G: when true, also publish a held bilateral rotation pulse on every wall_stuck event (default false = detection-only legacy behaviour)",
            ParamValue{false}},
        {"output_topic_left",  ParamMutability::ConstructionOnly,
            "Phase 6.6.G: left-channel pulse output (empty = action.left)",
            ParamValue{std::string("")}},
        {"output_topic_right", ParamMutability::ConstructionOnly,
            "Phase 6.6.G: right-channel pulse output (empty = action.right)",
            ParamValue{std::string("")}},
        {"pulse_ticks", ParamMutability::HotMutable,
            "Phase 6.6.G: held duration of the rotation pulse (default 90 ≈ 115° rotation)",
            ParamValue{int64_t{90}}},
        {"pulse_rotation", ParamMutability::HotMutable,
            "Phase 6.6.G: bilateral rotation magnitude (al = +mag, ar = -mag)",
            ParamValue{4.0}},
        {"duration_jitter", ParamMutability::HotMutable,
            "Phase 6.6.G: per-fire jitter applied to pulse_ticks and refractory_ticks (sampled from base * uniform[1 - jitter, 1 + jitter]). 0 = fixed; e.g. 0.4 = ±40%",
            ParamValue{0.0}},
        {"master_seed", ParamMutability::ConstructionOnly,
            "Phase 6.6.G: PRNG seed for pulse direction picks",
            ParamValue{int64_t{0}}},
    };
}

ParamMap StuckEscapeReflex::current_params() const {
    ParamMap m;
    m["imu_topic"]            = ParamValue{imu_topic_};
    m["vx_index"]             = ParamValue{int64_t(vx_index_)};
    m["vz_index"]             = ParamValue{int64_t(vz_index_)};
    m["ang_vel_topic"]        = ParamValue{ang_vel_topic_};
    m["ang_weight"]           = ParamValue{double(ang_weight_)};
    m["efference_topic"]      = ParamValue{efference_topic_};
    m["intent_floor"]         = ParamValue{double(intent_floor_)};
    m["hunger_topic"]         = ParamValue{hunger_topic_};
    m["sated_energy"]         = ParamValue{double(sated_energy_)};
    m["hunger_floor"]         = ParamValue{double(hunger_floor_)};
    m["noise_mag"]            = ParamValue{double(noise_mag_)};
    m["move_speed_reference"] = ParamValue{double(move_speed_reference_)};
    m["severity_threshold"]   = ParamValue{double(severity_threshold_)};
    m["window_ticks"]         = ParamValue{int64_t(window_ticks_)};
    m["refractory_ticks"]     = ParamValue{int64_t(refractory_ticks_)};
    m["enable_pulse"]         = ParamValue{enable_pulse_};
    m["output_topic_left"]    = ParamValue{output_topic_left_};
    m["output_topic_right"]   = ParamValue{output_topic_right_};
    m["pulse_ticks"]          = ParamValue{int64_t(pulse_ticks_)};
    m["pulse_rotation"]       = ParamValue{double(pulse_rotation_)};
    m["duration_jitter"]      = ParamValue{double(duration_jitter_)};
    m["master_seed"]          = ParamValue{int64_t(master_seed_)};
    return m;
}

void StuckEscapeReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("StuckEscapeReflex requires a non-null Bus");

    apply_param(params, "imu_topic",
        [&](auto const& v){ imu_topic_ = get_string(v, "imu_topic"); });
    apply_param(params, "vx_index",
        [&](auto const& v){ vx_index_ = int(get_int(v, "vx_index")); });
    apply_param(params, "vz_index",
        [&](auto const& v){ vz_index_ = int(get_int(v, "vz_index")); });
    apply_param(params, "ang_vel_topic",
        [&](auto const& v){ ang_vel_topic_ = get_string(v, "ang_vel_topic"); });
    apply_param(params, "ang_weight",
        [&](auto const& v){ ang_weight_ = float(get_double(v, "ang_weight")); });
    apply_param(params, "efference_topic",
        [&](auto const& v){ efference_topic_ = get_string(v, "efference_topic"); });
    apply_param(params, "intent_floor",
        [&](auto const& v){ intent_floor_ = float(get_double(v, "intent_floor")); });
    apply_param(params, "hunger_topic",  [&](auto const& v){ hunger_topic_  = get_string(v, "hunger_topic"); });
    apply_param(params, "sated_energy",  [&](auto const& v){ sated_energy_  = float(get_double(v, "sated_energy")); });
    apply_param(params, "hunger_floor",  [&](auto const& v){ hunger_floor_  = float(get_double(v, "hunger_floor")); });
    apply_param(params, "noise_mag",     [&](auto const& v){ noise_mag_     = float(get_double(v, "noise_mag")); });
    apply_param(params, "move_speed_reference",
        [&](auto const& v){ move_speed_reference_ = float(get_double(v, "move_speed_reference")); });
    apply_param(params, "severity_threshold",
        [&](auto const& v){ severity_threshold_ = float(get_double(v, "severity_threshold")); });
    apply_param(params, "window_ticks",
        [&](auto const& v){ window_ticks_ = int(get_int(v, "window_ticks")); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = int(get_int(v, "refractory_ticks")); });
    apply_param(params, "enable_pulse",
        [&](auto const& v){ enable_pulse_ = get_bool(v, "enable_pulse"); });
    apply_param(params, "output_topic_left",
        [&](auto const& v){ output_topic_left_  = get_string(v, "output_topic_left"); });
    apply_param(params, "output_topic_right",
        [&](auto const& v){ output_topic_right_ = get_string(v, "output_topic_right"); });
    apply_param(params, "pulse_ticks",
        [&](auto const& v){ pulse_ticks_ = int(get_int(v, "pulse_ticks")); });
    apply_param(params, "pulse_rotation",
        [&](auto const& v){ pulse_rotation_ = float(get_double(v, "pulse_rotation")); });
    apply_param(params, "duration_jitter",
        [&](auto const& v){ duration_jitter_ = float(get_double(v, "duration_jitter")); });
    apply_param(params, "master_seed",
        [&](auto const& v){ master_seed_ = uint64_t(get_int(v, "master_seed")); });
    pulse_rng_.seed(master_seed_ ? master_seed_ : 0xBADBEEFu);

    sub_ids_.push_back(bus_->subscribe(imu_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_imu(p); }));
    if (!ang_vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(ang_vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_ang_vel(p); }));
    if (!efference_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(efference_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_efference(p); }));
    if (!hunger_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(hunger_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_energy(p); }));
}

void StuckEscapeReflex::handle_energy(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) latest_energy_ = float(pt->values[0]);
}

void StuckEscapeReflex::handle_ang_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) latest_ang_speed_ = std::fabs(float(pt->values[0]));
}

void StuckEscapeReflex::handle_efference(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    float s = 0.0f;
    for (int i = 0; i < int(pt->values.size()); ++i) s += float(pt->values[i]) * float(pt->values[i]);
    latest_intended_ = std::sqrt(s);   // |commanded velocity| (efference copy)
}

void StuckEscapeReflex::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "move_speed_reference")  move_speed_reference_ = float(get_double(value, k));
    else if (k == "severity_threshold") severity_threshold_ = float(get_double(value, k));
    else if (k == "window_ticks")       window_ticks_       = int(get_int(value, k));
    else if (k == "refractory_ticks")   refractory_ticks_   = int(get_int(value, k));
    else if (k == "pulse_ticks")        pulse_ticks_        = int(get_int(value, k));
    else if (k == "pulse_rotation")     pulse_rotation_     = float(get_double(value, k));
    else if (k == "duration_jitter")    duration_jitter_    = float(get_double(value, k));
    else if (k == "ang_weight")         ang_weight_         = float(get_double(value, k));
    else if (k == "intent_floor")       intent_floor_       = float(get_double(value, k));
    else if (k == "sated_energy")       sated_energy_       = float(get_double(value, k));
    else if (k == "hunger_floor")       hunger_floor_       = float(get_double(value, k));
    else if (k == "noise_mag")          noise_mag_          = float(get_double(value, k));
    else throw std::invalid_argument("StuckEscapeReflex: unknown/non-mutable param '" + k + "'");
}

void StuckEscapeReflex::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int max_idx = std::max(vx_index_, vz_index_);
    if (pt->values.size() <= max_idx) return;
    // Phase 6.6.G — don't poison our own input.  While a rotation pulse
    // is in progress, forward velocity is intentionally zero (rotation
    // only — no joint spikes under differential_paddler), so feeding
    // those samples into the speed window would self-trigger the next
    // stuck event and lock the agent into a turn-forward-turn cycle.
    if (pulse_remaining_ > 0) return;
    float vx = pt->values[vx_index_];
    float vz = pt->values[vz_index_];
    latest_actual_ = std::sqrt(vx * vx + vz * vz);   // |afferent velocity|
    // Per-tick "stuckness" ∈ [0,1] pushed to the window (severity = its running mean).
    float stuckness;
    if (!efference_topic_.empty()) {
        // MISMATCH mode (self-normalizing): stuck only if COMMANDING motion that isn't
        // achieved.  Turning/idle command ~0 forward → intended < floor → stuckness 0.
        stuckness = (latest_intended_ > intent_floor_)
            ? std::clamp(1.0f - latest_actual_ / latest_intended_, 0.0f, 1.0f)
            : 0.0f;
    } else {
        // legacy velocity mode: stuck = low absolute speed (+ optional rotation term).
        float eff = latest_actual_ + ang_weight_ * latest_ang_speed_;
        stuckness = std::clamp(1.0f - eff / std::max(1e-3f, move_speed_reference_), 0.0f, 1.0f);
    }
    speed_window_.push_back(stuckness);
    while (int(speed_window_.size()) > window_ticks_) speed_window_.pop_front();
}

void StuckEscapeReflex::tick(uint64_t tick_id) {
    // Pulse actuation runs first so it can fire on the same tick a fresh
    // wall_stuck event is detected below.  When a pulse is in progress
    // we publish bilateral rotation to the configured output topics
    // every tick; severity detection still updates underneath.
    auto publish_pulse = [&]() {
        if (!enable_pulse_ || pulse_remaining_ <= 0) return;
        std::string l = output_topic_left_.empty()  ? std::string(topics::kActionLeft)
                                                    : output_topic_left_;
        std::string r = output_topic_right_.empty() ? std::string(topics::kActionRight)
                                                    : output_topic_right_;
        // NOISE: a random ± perturbation on top of the rotation jitters the system out of
        // the trap (operator: "build in some noise to perturb the system").
        float nl = 0.0f, nr = 0.0f;
        if (noise_mag_ > 0.0f) {
            std::uniform_real_distribution<float> u(-noise_mag_, noise_mag_);
            nl = u(pulse_rng_); nr = u(pulse_rng_);
        }
        auto act_l = std::make_shared<ActionOut>();
        act_l->tick_id     = tick_id;
        act_l->producer_id = id_.empty() ? std::string("stuck_escape") : id_;
        act_l->accel       = float(pulse_dir_) *  pulse_rotation_ + nl;
        act_l->source      = "stuck_escape";
        bus_->publish(l, act_l);
        auto act_r = std::make_shared<ActionOut>();
        act_r->tick_id     = tick_id;
        act_r->producer_id = act_l->producer_id;
        act_r->accel       = float(pulse_dir_) * -pulse_rotation_ + nr;
        act_r->source      = "stuck_escape";
        bus_->publish(r, act_r);
        --pulse_remaining_;
        if (pulse_remaining_ == 0) pulse_dir_ = 0;
    };

    if (refractory_remaining_ > 0) --refractory_remaining_;
    if (speed_window_.empty()) {
        last_severity_ = 0.0f;
        publish_pulse();
        return;
    }
    float sum = 0.0f;
    for (float s : speed_window_) sum += s;
    float severity = sum / float(speed_window_.size());   // window holds per-tick stuckness ∈[0,1]
    last_severity_ = severity;

    // HUNGER GATE: idle-when-sated is rest (don't perturb); idle-when-HUNGRY is the
    // "get off your butt and eat" trigger.  Empty hunger_topic = ungated (legacy).
    float hunger = (sated_energy_ > 1e-6f)
        ? std::clamp((sated_energy_ - latest_energy_) / sated_energy_, 0.0f, 1.0f) : 1.0f;
    bool hungry = hunger_topic_.empty() || (hunger > hunger_floor_);

    bool fire = (refractory_remaining_ == 0)
             && (int(speed_window_.size()) >= window_ticks_)
             && (severity > severity_threshold_)
             && hungry;

    if (fire) {
        auto e = std::make_shared<EnvEvent>();
        e->tick_id   = tick_id;
        e->name      = "wall_stuck";
        e->intensity = severity;
        bus_->publish("events.wall_stuck", e);
        ++stuck_count_;
        // Phase 6.6.G — sample jittered durations.  Same PRNG stream as
        // pulse_dir so the master_seed deterministically reproduces the
        // whole event sequence (direction + arc length + recovery
        // length).  At duration_jitter=0 this collapses to the base
        // values bit-for-bit, preserving legacy behaviour.
        auto sample_jittered = [&](int base) {
            if (duration_jitter_ <= 0.0f || base <= 0) return std::max(1, base);
            std::uniform_real_distribution<float> u(
                1.0f - duration_jitter_, 1.0f + duration_jitter_);
            return std::max(1, int(std::lround(float(base) * u(pulse_rng_))));
        };
        refractory_remaining_ = sample_jittered(refractory_ticks_);
        // Phase 6.6.G — start a new rotation pulse.  Only re-arm the
        // direction when no pulse is currently running so a re-fire
        // mid-pulse extends the same rotation rather than flipping it.
        if (enable_pulse_) {
            if (pulse_remaining_ <= 0) {
                pulse_dir_ = (pulse_rng_() & 1u) ? +1 : -1;
            }
            pulse_remaining_ = sample_jittered(pulse_ticks_);
            // Clear the speed window so the post-pulse re-detection
            // requires a FULL fresh window of samples — pre-pulse
            // stuck samples shouldn't carry over and re-trigger.
            speed_window_.clear();
        }
    }
    publish_pulse();
}

nlohmann::json StuckEscapeReflex::snapshot_state() const {
    std::ostringstream os; os << pulse_rng_;
    nlohmann::json speeds = nlohmann::json::array();
    for (auto v : speed_window_) speeds.push_back(v);
    return nlohmann::json{
        {"version",              1},
        {"pulse_rng",            os.str()},
        {"speed_window",         speeds},
        {"refractory_remaining", refractory_remaining_},
        {"stuck_count",          stuck_count_},
        {"last_severity",        last_severity_},
        {"pulse_remaining",      pulse_remaining_},
        {"pulse_dir",            pulse_dir_},
    };
}

void StuckEscapeReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("StuckEscapeReflex::restore_state: unknown version " +
                                 std::to_string(version));
    }
    if (s.contains("pulse_rng") && s["pulse_rng"].is_string()) {
        std::istringstream is(s["pulse_rng"].get<std::string>()); is >> pulse_rng_;
    }
    speed_window_.clear();
    if (s.contains("speed_window") && s["speed_window"].is_array())
        for (auto const& v : s["speed_window"]) speed_window_.push_back(v.get<float>());
    refractory_remaining_ = s.value("refractory_remaining", refractory_remaining_);
    stuck_count_          = s.value("stuck_count",          stuck_count_);
    last_severity_        = s.value("last_severity",        last_severity_);
    pulse_remaining_      = s.value("pulse_remaining",      pulse_remaining_);
    pulse_dir_            = s.value("pulse_dir",            pulse_dir_);
}

} // namespace ogma
