#include "ogma/modules/CellReflex.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
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
    throw std::invalid_argument("CellReflex: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("CellReflex: param '" + key + "' must be integer");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("CellReflex: param '" + key + "' must be bool");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("CellReflex: param '" + key + "' must be string");
}

std::vector<std::string> get_string_list(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("CellReflex: param '" + key + "' must be a list of strings");
}

} // namespace

CellReflex::CellReflex()  = default;
CellReflex::~CellReflex() = default;

std::string_view CellReflex::type_name() const { return "CellReflex"; }

std::vector<TopicSpec> CellReflex::input_topics() const {
    return {
        TopicSpec{whisker_topic_prefix_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{scent_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{imu_topic_,   std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> CellReflex::output_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{output_topic_left_,  std::type_index(typeid(ActionOut))},
        TopicSpec{output_topic_right_, std::type_index(typeid(ActionOut))},
    };
    if (emit_events_) {
        v.push_back(TopicSpec{"events.miss",       std::type_index(typeid(EnvEvent))});
        v.push_back(TopicSpec{"events.wall_stuck", std::type_index(typeid(EnvEvent))});
    }
    return v;
}

ParamSchema CellReflex::params_schema() const {
    return {
        // Topology
        {"whisker_topic_prefix", ParamMutability::ConstructionOnly,
            "Prefix-pattern subscription for whisker proprio topics",
            ParamValue{std::string("reality.proprio.whisker_")}},
        {"left_suffixes", ParamMutability::ConstructionOnly,
            "Suffixes (after prefix) that identify left-side whiskers",
            ParamValue{std::vector<std::string>{"0", "1", "2"}}},
        {"right_suffixes", ParamMutability::ConstructionOnly,
            "Suffixes (after prefix) that identify right-side whiskers",
            ParamValue{std::vector<std::string>{"3", "4", "5"}}},
        {"scent_topic", ParamMutability::ConstructionOnly,
            "Scent intensity proprio topic",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"scent_index", ParamMutability::ConstructionOnly,
            "Index into scent ProprioToken values",
            ParamValue{int64_t{0}}},
        {"imu_topic", ParamMutability::ConstructionOnly,
            "IMU proprio topic for stuck-velocity detection",
            ParamValue{std::string("reality.proprio.imu")}},
        {"vx_index", ParamMutability::ConstructionOnly,
            "IMU index for x-component velocity",
            ParamValue{int64_t{2}}},
        {"vz_index", ParamMutability::ConstructionOnly,
            "IMU index for z-component velocity",
            ParamValue{int64_t{3}}},
        {"output_topic_left", ParamMutability::ConstructionOnly,
            "Bilateral left ActionOut topic",
            ParamValue{std::string(topics::kActionLeft)}},
        {"output_topic_right", ParamMutability::ConstructionOnly,
            "Bilateral right ActionOut topic",
            ParamValue{std::string(topics::kActionRight)}},
        {"emit_events", ParamMutability::ConstructionOnly,
            "Publish events.miss / events.wall_stuck for brain-side neurochem signaling (default true)",
            ParamValue{true}},

        // Wander
        {"wander_thrust", ParamMutability::HotMutable,
            "Symmetric forward thrust applied each tick.  Maps to body's flagellum_base_rate via thrust/4 (2.0 → rate=0.5).",
            ParamValue{2.0}},
        {"wander_noise_amplitude", ParamMutability::HotMutable,
            "Per-tick uniform(-amp, +amp) added independently to each side's thrust.  Restores per-tick spike-rate variance the body's autonomous base rate provided.",
            ParamValue{0.5}},
        {"steer_amp", ParamMutability::HotMutable,
            "Multiplier on steer ∈ [-1, +1] before adding to al/subtracting from ar.  Maps to body's flagellum_steer_bias.  Default 2.0 (with thrust=2 → al ∈ [0, 4] across full steer range).",
            ParamValue{2.0}},

        // Stuck pulse (legacy semantics)
        {"stuck_window_ticks", ParamMutability::HotMutable,
            "Sliding window length (ticks) for velocity-deficit averaging",
            ParamValue{int64_t{60}}},
        {"stuck_severity_threshold", ParamMutability::HotMutable,
            "Deficit (1 - mean_speed/reference) transition threshold that triggers a fresh stuck-pulse (legacy default 0.5)",
            ParamValue{0.5}},
        {"stuck_move_speed_reference", ParamMutability::HotMutable,
            "Reference 'normal' speed the body should reach when unstuck",
            ParamValue{3.0}},
        {"stuck_pulse_period", ParamMutability::HotMutable,
            "Re-sample interval for the stuck pulse direction + magnitude (legacy default 30 ticks ≈ 0.5s)",
            ParamValue{int64_t{30}}},

        // Miss event (whisker-bump signaling)
        {"miss_threshold", ParamMutability::HotMutable,
            "max_w above which an events.miss is fired",
            ParamValue{0.30}},
        {"miss_refractory_ticks", ParamMutability::HotMutable,
            "Ticks between consecutive events.miss publishes",
            ParamValue{int64_t{30}}},

        // Optional whisker steer
        {"avoid_steer_gain", ParamMutability::HotMutable,
            "Optional differential whisker steer added to the steer signal (0 = disabled, brain handles steering; legacy modular preset used 8.0)",
            ParamValue{0.0}},
        {"avoid_threshold", ParamMutability::HotMutable,
            "max_w above which the optional avoid steer is active",
            ParamValue{0.30}},

        // Scent gating
        {"scent_alpha_short", ParamMutability::HotMutable,
            "EMA decay for the short-window scent average",
            ParamValue{0.1}},
        {"scent_alpha_long", ParamMutability::HotMutable,
            "EMA decay for the long-window scent baseline",
            ParamValue{0.001}},
        {"scent_long_pos_min", ParamMutability::HotMutable,
            "Floor on long-EMA before computing the gate; below this, gate stays 0",
            ParamValue{0.001}},
        {"scent_gate_cap", ParamMutability::HotMutable,
            "Maximum scent suppression factor (legacy default 0.5; gradient = (short - long) / max(long, floor))",
            ParamValue{0.5}},

        // Clamps + seed
        {"accel_min", ParamMutability::HotMutable,
            "Per-side ActionOut clamp minimum",
            ParamValue{-4.0}},
        {"accel_max", ParamMutability::HotMutable,
            "Per-side ActionOut clamp maximum",
            ParamValue{4.0}},
        {"master_seed", ParamMutability::ConstructionOnly,
            "PRNG seed for stuck-pulse rand draws and wander noise",
            ParamValue{int64_t{0}}},
    };
}

ParamMap CellReflex::current_params() const {
    ParamMap m;
    m["whisker_topic_prefix"]       = ParamValue{whisker_topic_prefix_};
    m["left_suffixes"]              = ParamValue{left_suffixes_};
    m["right_suffixes"]             = ParamValue{right_suffixes_};
    m["scent_topic"]                = ParamValue{scent_topic_};
    m["scent_index"]                = ParamValue{int64_t(scent_index_)};
    m["imu_topic"]                  = ParamValue{imu_topic_};
    m["vx_index"]                   = ParamValue{int64_t(vx_index_)};
    m["vz_index"]                   = ParamValue{int64_t(vz_index_)};
    m["output_topic_left"]          = ParamValue{output_topic_left_};
    m["output_topic_right"]         = ParamValue{output_topic_right_};
    m["emit_events"]                = ParamValue{emit_events_};
    m["wander_thrust"]              = ParamValue{double(wander_thrust_)};
    m["wander_noise_amplitude"]     = ParamValue{double(wander_noise_amplitude_)};
    m["steer_amp"]                  = ParamValue{double(steer_amp_)};
    m["stuck_window_ticks"]         = ParamValue{int64_t(stuck_window_ticks_)};
    m["stuck_severity_threshold"]   = ParamValue{double(stuck_severity_threshold_)};
    m["stuck_move_speed_reference"] = ParamValue{double(stuck_move_speed_reference_)};
    m["stuck_pulse_period"]         = ParamValue{int64_t(stuck_pulse_period_)};
    m["miss_threshold"]             = ParamValue{double(miss_threshold_)};
    m["miss_refractory_ticks"]      = ParamValue{int64_t(miss_refractory_ticks_)};
    m["avoid_steer_gain"]           = ParamValue{double(avoid_steer_gain_)};
    m["avoid_threshold"]            = ParamValue{double(avoid_threshold_)};
    m["scent_alpha_short"]          = ParamValue{double(scent_alpha_short_)};
    m["scent_alpha_long"]           = ParamValue{double(scent_alpha_long_)};
    m["scent_long_pos_min"]         = ParamValue{double(scent_long_pos_min_)};
    m["scent_gate_cap"]             = ParamValue{double(scent_gate_cap_)};
    m["accel_min"]                  = ParamValue{double(accel_min_)};
    m["accel_max"]                  = ParamValue{double(accel_max_)};
    m["master_seed"]                = ParamValue{int64_t(master_seed_)};
    return m;
}

void CellReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("CellReflex requires a non-null Bus");

    apply_param(params, "whisker_topic_prefix",
        [&](auto const& v){ whisker_topic_prefix_ = get_string(v, "whisker_topic_prefix"); });
    apply_param(params, "left_suffixes",
        [&](auto const& v){ left_suffixes_  = get_string_list(v, "left_suffixes"); });
    apply_param(params, "right_suffixes",
        [&](auto const& v){ right_suffixes_ = get_string_list(v, "right_suffixes"); });
    apply_param(params, "scent_topic",
        [&](auto const& v){ scent_topic_ = get_string(v, "scent_topic"); });
    apply_param(params, "scent_index",
        [&](auto const& v){ scent_index_ = int(get_int(v, "scent_index")); });
    apply_param(params, "imu_topic",
        [&](auto const& v){ imu_topic_ = get_string(v, "imu_topic"); });
    apply_param(params, "vx_index",
        [&](auto const& v){ vx_index_ = int(get_int(v, "vx_index")); });
    apply_param(params, "vz_index",
        [&](auto const& v){ vz_index_ = int(get_int(v, "vz_index")); });
    apply_param(params, "output_topic_left",
        [&](auto const& v){ output_topic_left_  = get_string(v, "output_topic_left"); });
    apply_param(params, "output_topic_right",
        [&](auto const& v){ output_topic_right_ = get_string(v, "output_topic_right"); });
    apply_param(params, "emit_events",
        [&](auto const& v){ emit_events_ = get_bool(v, "emit_events"); });

    apply_param(params, "wander_thrust",
        [&](auto const& v){ wander_thrust_ = float(get_double(v, "wander_thrust")); });
    apply_param(params, "wander_noise_amplitude",
        [&](auto const& v){ wander_noise_amplitude_ = float(get_double(v, "wander_noise_amplitude")); });
    apply_param(params, "steer_amp",
        [&](auto const& v){ steer_amp_ = float(get_double(v, "steer_amp")); });

    apply_param(params, "stuck_window_ticks",
        [&](auto const& v){ stuck_window_ticks_ = int(get_int(v, "stuck_window_ticks")); });
    apply_param(params, "stuck_severity_threshold",
        [&](auto const& v){ stuck_severity_threshold_ = float(get_double(v, "stuck_severity_threshold")); });
    apply_param(params, "stuck_move_speed_reference",
        [&](auto const& v){ stuck_move_speed_reference_ = float(get_double(v, "stuck_move_speed_reference")); });
    apply_param(params, "stuck_pulse_period",
        [&](auto const& v){ stuck_pulse_period_ = int(get_int(v, "stuck_pulse_period")); });

    apply_param(params, "miss_threshold",
        [&](auto const& v){ miss_threshold_ = float(get_double(v, "miss_threshold")); });
    apply_param(params, "miss_refractory_ticks",
        [&](auto const& v){ miss_refractory_ticks_ = int(get_int(v, "miss_refractory_ticks")); });

    apply_param(params, "avoid_steer_gain",
        [&](auto const& v){ avoid_steer_gain_ = float(get_double(v, "avoid_steer_gain")); });
    apply_param(params, "avoid_threshold",
        [&](auto const& v){ avoid_threshold_ = float(get_double(v, "avoid_threshold")); });

    apply_param(params, "scent_alpha_short",
        [&](auto const& v){ scent_alpha_short_ = float(get_double(v, "scent_alpha_short")); });
    apply_param(params, "scent_alpha_long",
        [&](auto const& v){ scent_alpha_long_  = float(get_double(v, "scent_alpha_long")); });
    apply_param(params, "scent_long_pos_min",
        [&](auto const& v){ scent_long_pos_min_ = float(get_double(v, "scent_long_pos_min")); });
    apply_param(params, "scent_gate_cap",
        [&](auto const& v){ scent_gate_cap_ = float(get_double(v, "scent_gate_cap")); });

    apply_param(params, "accel_min",
        [&](auto const& v){ accel_min_ = float(get_double(v, "accel_min")); });
    apply_param(params, "accel_max",
        [&](auto const& v){ accel_max_ = float(get_double(v, "accel_max")); });
    apply_param(params, "master_seed",
        [&](auto const& v){ master_seed_ = uint64_t(get_int(v, "master_seed")); });
    rng_.seed(master_seed_ ? master_seed_ : 0xCE11C0DEu);

    left_topics_.clear();
    right_topics_.clear();
    for (auto const& s : left_suffixes_)  left_topics_.insert(whisker_topic_prefix_ + s);
    for (auto const& s : right_suffixes_) right_topics_.insert(whisker_topic_prefix_ + s);

    std::string parent_prefix = whisker_topic_prefix_;
    auto last_dot = parent_prefix.rfind('.');
    if (last_dot != std::string::npos) {
        parent_prefix = parent_prefix.substr(0, last_dot + 1);
    } else {
        parent_prefix.push_back('.');
    }
    sub_ids_.push_back(bus_->subscribe(parent_prefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_whisker(t, p); }));
    sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    sub_ids_.push_back(bus_->subscribe(imu_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_imu(p); }));
}

void CellReflex::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "wander_thrust")              wander_thrust_ = float(get_double(value, k));
    else if (k == "wander_noise_amplitude")     wander_noise_amplitude_ = float(get_double(value, k));
    else if (k == "steer_amp")                  steer_amp_ = float(get_double(value, k));
    else if (k == "stuck_window_ticks")         stuck_window_ticks_ = int(get_int(value, k));
    else if (k == "stuck_severity_threshold")   stuck_severity_threshold_ = float(get_double(value, k));
    else if (k == "stuck_move_speed_reference") stuck_move_speed_reference_ = float(get_double(value, k));
    else if (k == "stuck_pulse_period")         stuck_pulse_period_ = int(get_int(value, k));
    else if (k == "miss_threshold")             miss_threshold_ = float(get_double(value, k));
    else if (k == "miss_refractory_ticks")      miss_refractory_ticks_ = int(get_int(value, k));
    else if (k == "avoid_steer_gain")           avoid_steer_gain_ = float(get_double(value, k));
    else if (k == "avoid_threshold")            avoid_threshold_ = float(get_double(value, k));
    else if (k == "scent_alpha_short")          scent_alpha_short_ = float(get_double(value, k));
    else if (k == "scent_alpha_long")           scent_alpha_long_  = float(get_double(value, k));
    else if (k == "scent_long_pos_min")         scent_long_pos_min_ = float(get_double(value, k));
    else if (k == "scent_gate_cap")             scent_gate_cap_ = float(get_double(value, k));
    else if (k == "accel_min")                  accel_min_ = float(get_double(value, k));
    else if (k == "accel_max")                  accel_max_ = float(get_double(value, k));
    else
        throw std::invalid_argument("CellReflex: unknown/non-mutable param '" + k + "'");
}

bool CellReflex::is_left(std::string const& topic) const {
    return left_topics_.count(topic) > 0;
}

bool CellReflex::is_right(std::string const& topic) const {
    return right_topics_.count(topic) > 0;
}

void CellReflex::handle_whisker(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    std::string t(topic);
    if (!is_left(t) && !is_right(t)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    last_whisker_values_[t] = pt->values[0];
}

void CellReflex::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    if (int(pt->values.size()) <= scent_index_) return;
    float s = pt->values[scent_index_];
    if (!scent_initialised_) {
        scent_ema_short_   = s;
        scent_ema_long_    = s;
        scent_initialised_ = true;
    } else {
        scent_ema_short_ = (1.0f - scent_alpha_short_) * scent_ema_short_ + scent_alpha_short_ * s;
        scent_ema_long_  = (1.0f - scent_alpha_long_)  * scent_ema_long_  + scent_alpha_long_  * s;
    }
}

void CellReflex::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int max_idx = std::max(vx_index_, vz_index_);
    if (int(pt->values.size()) <= max_idx) return;
    float vx = pt->values[vx_index_];
    float vz = pt->values[vz_index_];
    float speed = std::sqrt(vx * vx + vz * vz);
    speed_window_.push_back(speed);
    while (int(speed_window_.size()) > stuck_window_ticks_) speed_window_.pop_front();
}

float CellReflex::sample_noise() {
    if (wander_noise_amplitude_ <= 0.0f) return 0.0f;
    std::uniform_real_distribution<float> u(
        -wander_noise_amplitude_, wander_noise_amplitude_);
    return u(rng_);
}

float CellReflex::sample_uniform_signed() {
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    return u(rng_);
}

void CellReflex::tick(uint64_t tick_id) {
    // ------------------------------------------------------------------
    // 1. Read whisker contacts
    // ------------------------------------------------------------------
    float left_max  = 0.0f;
    float right_max = 0.0f;
    float left_sum  = 0.0f;
    float right_sum = 0.0f;
    for (auto const& [topic, v] : last_whisker_values_) {
        if (is_left(topic))  { left_max  = std::max(left_max,  v); left_sum  += v; }
        else if (is_right(topic)) { right_max = std::max(right_max, v); right_sum += v; }
    }
    float diff_max = left_max - right_max;
    float max_w    = std::max(left_max, right_max);

    // ------------------------------------------------------------------
    // 2. Scent gate.  Legacy formula: gate = clamp((short-long)/long, 0, cap)
    //    Floor the denominator at scent_long_pos_min so a pre-warmup zero
    //    long-EMA doesn't produce a divide-by-zero or runaway gate.
    // ------------------------------------------------------------------
    float scent_factor = 0.0f;
    if (scent_initialised_ && scent_ema_long_ > scent_long_pos_min_) {
        float gradient = (scent_ema_short_ - scent_ema_long_) / scent_ema_long_;
        scent_factor = std::clamp(gradient, 0.0f, scent_gate_cap_);
    }
    last_scent_factor_ = scent_factor;

    // ------------------------------------------------------------------
    // 3. Velocity-deficit (legacy 1-second window).
    // ------------------------------------------------------------------
    float deficit = 0.0f;
    if (int(speed_window_.size()) >= stuck_window_ticks_
        && stuck_move_speed_reference_ > 0.0f) {
        float sum = 0.0f;
        for (float s : speed_window_) sum += s;
        float mean_speed = sum / float(speed_window_.size());
        deficit = std::clamp(
            1.0f - mean_speed / stuck_move_speed_reference_, 0.0f, 1.0f);
    }
    last_deficit_ = deficit;

    // ------------------------------------------------------------------
    // 4. Whisker miss event (neurochem signaling — replaces body's
    //    _publish_reflex_event("miss", ...) call).  Refractory keeps the
    //    rate sane.
    // ------------------------------------------------------------------
    if (miss_refractory_remaining_ > 0) --miss_refractory_remaining_;
    if (emit_events_ && miss_refractory_remaining_ == 0
        && max_w > miss_threshold_) {
        auto e = std::make_shared<EnvEvent>();
        e->tick_id   = tick_id;
        e->name      = "miss";
        e->intensity = max_w * (1.0f - scent_factor);
        bus_->publish("events.miss", e);
        miss_refractory_remaining_ = miss_refractory_ticks_;
        ++miss_count_;
    }

    // ------------------------------------------------------------------
    // 5. Stuck-pulse resampling — legacy logic.
    //    Trigger on (a) period elapsed OR (b) deficit transitioning
    //    through stuck_severity_threshold (an edge that signals the agent
    //    just got stuck).  Direction is taken from contact-sum asymmetry
    //    so the pulse rotates AWAY from the more-blocked side; magnitude
    //    blends from pure noise (no informative contact) to ±1 committed
    //    (strong asymmetry).
    // ------------------------------------------------------------------
    bool deficit_transition = (prev_deficit_ <= stuck_severity_threshold_
                            && deficit > stuck_severity_threshold_);
    bool resample = (pulse_ticks_remaining_ <= 0) || deficit_transition;
    if (resample) {
        float diff_sum = right_sum - left_sum;   // legacy convention
        float rand     = sample_uniform_signed();
        float t        = std::tanh(std::abs(diff_sum)) * (1.0f - scent_factor);
        float dir;
        if (diff_sum > 0.0f)      dir = -1.0f;   // right blocked → turn left
        else if (diff_sum < 0.0f) dir = +1.0f;   // left blocked  → turn right
        else                      dir = (rand >= 0.0f ? +1.0f : -1.0f);
        // lerp(|rand|, 1.0, t): pure-noise magnitude when no contact,
        // committed ±1 when strong contact.
        float magnitude = std::abs(rand) * (1.0f - t) + 1.0f * t;
        pulse_held_ = std::clamp(dir * magnitude, -1.0f, 1.0f);
        pulse_ticks_remaining_ = stuck_pulse_period_;

        if (deficit_transition && emit_events_) {
            auto e = std::make_shared<EnvEvent>();
            e->tick_id   = tick_id;
            e->name      = "wall_stuck";
            e->intensity = deficit;
            bus_->publish("events.wall_stuck", e);
            ++stuck_count_;
        }
    }
    if (pulse_ticks_remaining_ > 0) --pulse_ticks_remaining_;
    prev_deficit_ = deficit;

    // ------------------------------------------------------------------
    // 6. Steer signal.  Optional whisker-steer-as-reflex is OFF by default
    //    (legacy premotor config has no reflex modules — the brain
    //    learns to steer).  When enabled, scent-gated like the legacy
    //    WhiskerSteerReflex.
    // ------------------------------------------------------------------
    float whisker_steer = 0.0f;
    if (avoid_steer_gain_ > 0.0f && max_w > avoid_threshold_) {
        float effective_gain = avoid_steer_gain_ * (1.0f - 2.0f * scent_factor);
        whisker_steer = diff_max * effective_gain;
    }
    float stuck_pulse_steer = deficit * pulse_held_;
    float steer = std::clamp(whisker_steer + stuck_pulse_steer, -1.0f, 1.0f);
    last_steer_ = steer;

    // ------------------------------------------------------------------
    // 7. Bilateral output.  Per-tick noise on each side reproduces the
    //    body's per-tick Bernoulli spike noise the autonomous base rate
    //    used to provide.
    // ------------------------------------------------------------------
    float al = wander_thrust_ + sample_noise() + steer_amp_ * steer;
    float ar = wander_thrust_ + sample_noise() - steer_amp_ * steer;
    al = std::clamp(al, accel_min_, accel_max_);
    ar = std::clamp(ar, accel_min_, accel_max_);
    last_al_ = al;
    last_ar_ = ar;

    auto act_l = std::make_shared<ActionOut>();
    act_l->tick_id     = tick_id;
    act_l->producer_id = id_.empty() ? std::string("cell_reflex") : id_;
    act_l->accel       = al;
    act_l->source      = "cell_reflex";
    bus_->publish(output_topic_left_, act_l);

    auto act_r = std::make_shared<ActionOut>();
    act_r->tick_id     = tick_id;
    act_r->producer_id = act_l->producer_id;
    act_r->accel       = ar;
    act_r->source      = "cell_reflex";
    bus_->publish(output_topic_right_, act_r);
}

// ---------------------------------------------------------------------------
// Snapshot / restore (UI-dev W3.2 Tier A)
// ---------------------------------------------------------------------------
//
// left_topics_/right_topics_ are derived in on_setup from suffix params and
// not snapshotted. last_whisker_values_ is captured by topic so the gate
// signal on the first tick after restore matches the source.

nlohmann::json CellReflex::snapshot_state() const {
    std::ostringstream os; os << rng_;
    nlohmann::json whiskers = nlohmann::json::object();
    for (auto const& [k, v] : last_whisker_values_) whiskers[k] = v;
    nlohmann::json speeds = nlohmann::json::array();
    for (auto v : speed_window_) speeds.push_back(v);
    return nlohmann::json{
        {"version",                 1},
        {"rng",                     os.str()},
        {"last_whisker_values",     whiskers},
        {"scent_ema_short",         scent_ema_short_},
        {"scent_ema_long",          scent_ema_long_},
        {"scent_initialised",       scent_initialised_},
        {"speed_window",            speeds},
        {"pulse_held",              pulse_held_},
        {"pulse_ticks_remaining",   pulse_ticks_remaining_},
        {"prev_deficit",            prev_deficit_},
        {"miss_refractory_remaining", miss_refractory_remaining_},
        {"stuck_count",             stuck_count_},
        {"miss_count",              miss_count_},
        {"last_al",                 last_al_},
        {"last_ar",                 last_ar_},
        {"last_deficit",            last_deficit_},
        {"last_scent_factor",       last_scent_factor_},
        {"last_steer",              last_steer_},
    };
}

void CellReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("CellReflex::restore_state: unknown version " +
                                 std::to_string(version));
    }
    if (s.contains("rng") && s["rng"].is_string()) {
        std::istringstream is(s["rng"].get<std::string>()); is >> rng_;
    }
    last_whisker_values_.clear();
    if (s.contains("last_whisker_values") && s["last_whisker_values"].is_object())
        for (auto it = s["last_whisker_values"].begin();
             it != s["last_whisker_values"].end(); ++it)
            last_whisker_values_[it.key()] = it.value().get<float>();
    scent_ema_short_         = s.value("scent_ema_short",         scent_ema_short_);
    scent_ema_long_          = s.value("scent_ema_long",          scent_ema_long_);
    scent_initialised_       = s.value("scent_initialised",       scent_initialised_);
    speed_window_.clear();
    if (s.contains("speed_window") && s["speed_window"].is_array())
        for (auto const& v : s["speed_window"]) speed_window_.push_back(v.get<float>());
    pulse_held_              = s.value("pulse_held",              pulse_held_);
    pulse_ticks_remaining_   = s.value("pulse_ticks_remaining",   pulse_ticks_remaining_);
    prev_deficit_            = s.value("prev_deficit",            prev_deficit_);
    miss_refractory_remaining_ = s.value("miss_refractory_remaining", miss_refractory_remaining_);
    stuck_count_             = s.value("stuck_count",             stuck_count_);
    miss_count_              = s.value("miss_count",              miss_count_);
    last_al_                 = s.value("last_al",                 last_al_);
    last_ar_                 = s.value("last_ar",                 last_ar_);
    last_deficit_            = s.value("last_deficit",            last_deficit_);
    last_scent_factor_       = s.value("last_scent_factor",       last_scent_factor_);
    last_steer_              = s.value("last_steer",              last_steer_);
}

} // namespace ogma
