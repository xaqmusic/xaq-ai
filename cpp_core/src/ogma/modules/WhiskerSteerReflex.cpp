#include "ogma/modules/WhiskerSteerReflex.hpp"

#include <algorithm>
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
    throw std::invalid_argument("WhiskerSteerReflex: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("WhiskerSteerReflex: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("WhiskerSteerReflex: param '" + key + "' must be a string");
}

std::vector<std::string> get_string_list(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("WhiskerSteerReflex: param '" + key + "' must be a list of strings");
}

} // namespace

WhiskerSteerReflex::WhiskerSteerReflex()  = default;
WhiskerSteerReflex::~WhiskerSteerReflex() = default;

std::string_view WhiskerSteerReflex::type_name() const { return "WhiskerSteerReflex"; }

std::vector<TopicSpec> WhiskerSteerReflex::input_topics() const {
    return {
        TopicSpec{whisker_topic_prefix_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> WhiskerSteerReflex::output_topics() const {
    if (!output_topic_.empty()) {
        return { TopicSpec{output_topic_, std::type_index(typeid(ActionOut))} };
    }
    return {
        TopicSpec{output_topic_left_,  std::type_index(typeid(ActionOut))},
        TopicSpec{output_topic_right_, std::type_index(typeid(ActionOut))},
    };
}

ParamSchema WhiskerSteerReflex::params_schema() const {
    return {
        {"whisker_topic_prefix", ParamMutability::ConstructionOnly,
            "Prefix-pattern subscription for whisker proprio topics",
            ParamValue{std::string("reality.proprio.whisker_")}},
        {"left_suffixes", ParamMutability::ConstructionOnly,
            "Suffixes (after prefix) that identify left-side whiskers",
            ParamValue{std::vector<std::string>{"0", "1", "2"}}},
        {"right_suffixes", ParamMutability::ConstructionOnly,
            "Suffixes (after prefix) that identify right-side whiskers",
            ParamValue{std::vector<std::string>{"3", "4", "5"}}},
        {"threshold", ParamMutability::HotMutable,
            "Max-side contact above which steering activates (0..1)",
            ParamValue{0.30}},
        {"base_thrust", ParamMutability::HotMutable,
            "Symmetric thrust applied to both sides while steering. "
            "Body model only uses (al-ar) for steering, so this is largely "
            "diagnostic; bee/quadruped morphologies will use it.",
            ParamValue{0.0}},
        {"steer_gain", ParamMutability::HotMutable,
            "Differential gain: (al-ar) = steer_gain × (left_max − right_max). "
            "Default 8.0 maps full asymmetry (1.0) to full ±4 accel range.",
            ParamValue{8.0}},
        {"accel_min", ParamMutability::HotMutable,
            "Per-side ActionOut clamp minimum",
            ParamValue{-4.0}},
        {"accel_max", ParamMutability::HotMutable,
            "Per-side ActionOut clamp maximum",
            ParamValue{4.0}},
        {"refractory_ticks", ParamMutability::HotMutable,
            "Ticks to suppress further steering after a kick (0 = fire each tick contact persists)",
            ParamValue{int64_t{0}}},
        {"pulse_ticks", ParamMutability::HotMutable,
            "Ticks to HOLD the steer-away kick before the refractory (a 1-tick fire barely "
            "imparts angular impulse). 1 = legacy single-tick fire.", ParamValue{int64_t{1}}},
        {"min_steer_threshold", ParamMutability::HotMutable,
            "Phase 6.6.G: skip publish when |left_max - right_max| < this (lets ForwardDriveReflex thrust survive on near-symmetric contact). 0 = legacy always-publish.",
            ParamValue{0.0}},
        {"head_on_threshold", ParamMutability::HotMutable,
            "Phase 6.6.G: when min(left_max, right_max) >= this, switch to head-on rotation pulse instead of differential steer. 0 = disabled (legacy).",
            ParamValue{0.0}},
        {"head_on_rotation", ParamMutability::HotMutable,
            "Phase 6.6.G: bilateral rotation magnitude in head-on mode (al = +mag, ar = -mag, sign chosen per event)",
            ParamValue{4.0}},
        {"master_seed", ParamMutability::ConstructionOnly,
            "Phase 6.6.G: PRNG seed for head-on direction picks (0 = constant default)",
            ParamValue{int64_t{0}}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Phase 6.6.F: when non-empty, publish a single ActionOut with accel=al-ar to this topic instead of bilateral action.left/right (lets this serve as the reflex side of a single-channel MotorFader). Empty default preserves bilateral pathway.",
            ParamValue{std::string("")}},
        {"output_topic_left",  ParamMutability::ConstructionOnly,
            "Phase 6.6.G: left-side ActionOut topic when in bilateral mode (default action.left; set to action.reflex.left to route through bilateral MotorFader)",
            ParamValue{std::string(topics::kActionLeft)}},
        {"output_topic_right", ParamMutability::ConstructionOnly,
            "Phase 6.6.G: right-side ActionOut topic when in bilateral mode (default action.right; set to action.reflex.right to route through bilateral MotorFader)",
            ParamValue{std::string(topics::kActionRight)}},
        {"suppression_topic", ParamMutability::ConstructionOnly,
            "Phase 6.6.F.1: optional ReflexGate topic (e.g. ScentGateReflex output). Gate value g ∈ [0,1] mediates steer sign: 0 = aversion, 0.5 = no steer, 1.0 = attraction. Lets a high scent gradient flip whisker contact from 'wall' (turn away) to 'food' (charge through). Empty default = legacy aversion only.",
            ParamValue{std::string("")}},
    };
}

ParamMap WhiskerSteerReflex::current_params() const {
    ParamMap m;
    m["whisker_topic_prefix"] = ParamValue{whisker_topic_prefix_};
    m["left_suffixes"]        = ParamValue{left_suffixes_};
    m["right_suffixes"]       = ParamValue{right_suffixes_};
    m["threshold"]            = ParamValue{double(threshold_)};
    m["base_thrust"]          = ParamValue{double(base_thrust_)};
    m["steer_gain"]           = ParamValue{double(steer_gain_)};
    m["accel_min"]            = ParamValue{double(accel_min_)};
    m["accel_max"]            = ParamValue{double(accel_max_)};
    m["refractory_ticks"]     = ParamValue{int64_t(refractory_ticks_)};
    m["pulse_ticks"]          = ParamValue{int64_t(pulse_ticks_)};
    m["min_steer_threshold"]  = ParamValue{double(min_steer_threshold_)};
    m["head_on_threshold"]    = ParamValue{double(head_on_threshold_)};
    m["head_on_rotation"]     = ParamValue{double(head_on_rotation_)};
    m["master_seed"]          = ParamValue{int64_t(master_seed_)};
    m["output_topic"]         = ParamValue{output_topic_};
    m["output_topic_left"]    = ParamValue{output_topic_left_};
    m["output_topic_right"]   = ParamValue{output_topic_right_};
    m["suppression_topic"]    = ParamValue{suppression_topic_};
    return m;
}

void WhiskerSteerReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("WhiskerSteerReflex requires a non-null Bus");

    apply_param(params, "whisker_topic_prefix",
        [&](auto const& v){ whisker_topic_prefix_ = get_string(v, "whisker_topic_prefix"); });
    apply_param(params, "left_suffixes",
        [&](auto const& v){ left_suffixes_  = get_string_list(v, "left_suffixes"); });
    apply_param(params, "right_suffixes",
        [&](auto const& v){ right_suffixes_ = get_string_list(v, "right_suffixes"); });
    apply_param(params, "threshold",
        [&](auto const& v){ threshold_ = float(get_double(v, "threshold")); });
    apply_param(params, "base_thrust",
        [&](auto const& v){ base_thrust_ = float(get_double(v, "base_thrust")); });
    apply_param(params, "steer_gain",
        [&](auto const& v){ steer_gain_ = float(get_double(v, "steer_gain")); });
    apply_param(params, "accel_min",
        [&](auto const& v){ accel_min_ = float(get_double(v, "accel_min")); });
    apply_param(params, "accel_max",
        [&](auto const& v){ accel_max_ = float(get_double(v, "accel_max")); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = int(get_int(v, "refractory_ticks")); });
    apply_param(params, "pulse_ticks",
        [&](auto const& v){ pulse_ticks_ = int(get_int(v, "pulse_ticks")); });
    apply_param(params, "min_steer_threshold",
        [&](auto const& v){ min_steer_threshold_ = float(get_double(v, "min_steer_threshold")); });
    apply_param(params, "head_on_threshold",
        [&](auto const& v){ head_on_threshold_   = float(get_double(v, "head_on_threshold")); });
    apply_param(params, "head_on_rotation",
        [&](auto const& v){ head_on_rotation_    = float(get_double(v, "head_on_rotation")); });
    apply_param(params, "reverse_strength",
        [&](auto const& v){ reverse_strength_    = float(get_double(v, "reverse_strength")); });
    apply_param(params, "master_seed",
        [&](auto const& v){ master_seed_         = uint64_t(get_int(v, "master_seed")); });
    head_on_rng_.seed(master_seed_ ? master_seed_ : 0xC0FFEEu);
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "output_topic_left",
        [&](auto const& v){ output_topic_left_  = get_string(v, "output_topic_left"); });
    apply_param(params, "output_topic_right",
        [&](auto const& v){ output_topic_right_ = get_string(v, "output_topic_right"); });
    apply_param(params, "suppression_topic",
        [&](auto const& v){ suppression_topic_ = get_string(v, "suppression_topic"); });

    // Pre-build the full-topic lookup sets so handle_whisker can do an O(1)
    // membership test instead of two list scans per delivery.
    left_topics_.clear();
    right_topics_.clear();
    for (auto const& s : left_suffixes_)  left_topics_.insert(whisker_topic_prefix_ + s);
    for (auto const& s : right_suffixes_) right_topics_.insert(whisker_topic_prefix_ + s);
    // Per-whisker position weights (outer→inner): a wall AHEAD turns harder than
    // one to the side.  Aligned positionally to the suffix lists; missing → 1.0.
    topic_weight_.clear();
    for (size_t i = 0; i < left_suffixes_.size(); ++i)
        topic_weight_[whisker_topic_prefix_ + left_suffixes_[i]] =
            (i < left_weights_.size()) ? left_weights_[i] : 1.0f;
    for (size_t i = 0; i < right_suffixes_.size(); ++i)
        topic_weight_[whisker_topic_prefix_ + right_suffixes_[i]] =
            (i < right_weights_.size()) ? right_weights_[i] : 1.0f;

    // Subscribe at the parent prefix (the bus only honors trailing-dot
    // patterns) and filter inside handle_whisker.  Same approach as
    // WhiskerAversionReflex.
    std::string parent_prefix = whisker_topic_prefix_;
    auto last_dot = parent_prefix.rfind('.');
    if (last_dot != std::string::npos) {
        parent_prefix = parent_prefix.substr(0, last_dot + 1);
    } else {
        parent_prefix.push_back('.');
    }
    sub_ids_.push_back(bus_->subscribe(parent_prefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_whisker(t, p); }));

    // Phase 6.6.F.1 — optional suppression input (e.g. ScentGateReflex).
    // When present, the gate's value mediates the steer sign so that
    // whisker contact during high scent flips from aversion to attraction.
    if (!suppression_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(suppression_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){
                if (auto g = std::dynamic_pointer_cast<const ReflexGate>(p)) {
                    last_suppression_ = std::clamp(g->value, 0.0f, 1.0f);
                }
            }));
    }
}

void WhiskerSteerReflex::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "threshold")              threshold_           = float(get_double(value, k));
    else if (k == "base_thrust")        base_thrust_        = float(get_double(value, k));
    else if (k == "steer_gain")         steer_gain_         = float(get_double(value, k));
    else if (k == "accel_min")          accel_min_          = float(get_double(value, k));
    else if (k == "accel_max")          accel_max_          = float(get_double(value, k));
    else if (k == "refractory_ticks")   refractory_ticks_   = int(get_int(value, k));
    else if (k == "pulse_ticks")         pulse_ticks_        = int(get_int(value, k));
    else if (k == "min_steer_threshold") min_steer_threshold_ = float(get_double(value, k));
    else if (k == "head_on_threshold")   head_on_threshold_   = float(get_double(value, k));
    else if (k == "head_on_rotation")    head_on_rotation_    = float(get_double(value, k));
    else if (k == "reverse_strength")    reverse_strength_    = float(get_double(value, k));
    else throw std::invalid_argument("WhiskerSteerReflex: unknown/non-mutable param '" + k + "'");
}

bool WhiskerSteerReflex::is_left(std::string const& topic) const {
    return left_topics_.count(topic) > 0;
}

bool WhiskerSteerReflex::is_right(std::string const& topic) const {
    return right_topics_.count(topic) > 0;
}

void WhiskerSteerReflex::handle_whisker(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    std::string t(topic);
    if (!is_left(t) && !is_right(t)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    last_values_[t] = pt->values[0];
}

void WhiskerSteerReflex::tick(uint64_t tick_id) {
    // Per-side WEIGHTED contact: inner (forward-facing) whiskers weigh more than
    // outer (lateral) ones, and contributions are SUMMED → more whiskers engaged
    // = stronger turn-away.  left_max/right_max gate activation per side.
    float left_max = 0.0f, right_max = 0.0f, left_sum = 0.0f, right_sum = 0.0f;
    for (auto const& [topic, v] : last_values_) {
        float w = 1.0f;
        auto wit = topic_weight_.find(topic);
        if (wit != topic_weight_.end()) w = wit->second;
        if (is_left(topic))       { left_max  = std::max(left_max,  v); if (v > threshold_) left_sum  += v * w; }
        else if (is_right(topic)) { right_max = std::max(right_max, v); if (v > threshold_) right_sum += v * w; }
    }
    float max_w = std::max(left_max, right_max);

    last_skipped_     = false;
    last_head_on_     = false;

    // Refractory release: stay silent so the cog resurfaces (drives + LEARNS the turn).
    if (refractory_remaining_ > 0) {
        --refractory_remaining_;
        if (max_w <= threshold_) last_head_on_dir_ = 0;
        last_skipped_ = true;
        return;
    }

    float al, ar;
    if (pulse_remaining_ > 0) {
        // HOLD the kick: re-issue the latched steer-away for the rest of the pulse so it
        // imparts real angular impulse (a 1-tick fire barely moves the bug), THEN drop
        // into the refractory release.
        al = latched_al_;
        ar = latched_ar_;
        last_head_on_ = latched_head_on_;
        if (--pulse_remaining_ == 0) refractory_remaining_ = refractory_ticks_;
    } else if (max_w > threshold_) {
        // ---- Fresh contact → compute the kick, LATCH it, start the held pulse --------
        // Ipsilateral turn-away with a wedge REVERSE.  whiskers see WALLS only, so this
        // is pure obstacle-escape; it subsumes the cog on the bus only while kicking.
        bool any_left  = left_max  > threshold_;
        bool any_right = right_max > threshold_;
        if (any_left && any_right) {
            // WEDGE/CORNER: boxed in → flip sign and BACK OUT (the real emergency reverse).
            last_head_on_ = true;
            al = std::min(-steer_gain_ * left_sum,  -reverse_strength_);
            ar = std::min(-steer_gain_ * right_sum, -reverse_strength_);
        } else {
            // SINGLE SIDE: ipsilateral turn-away (left wall → al>ar → CW, curves away).
            al = steer_gain_ * left_sum;
            ar = steer_gain_ * right_sum;
        }
        al = std::clamp(al, accel_min_, accel_max_);
        ar = std::clamp(ar, accel_min_, accel_max_);
        latched_al_ = al; latched_ar_ = ar; latched_head_on_ = last_head_on_;
        ++fire_count_;
        // this tick is the first of pulse_ticks held ticks; refractory follows the pulse.
        pulse_remaining_ = std::max(0, pulse_ticks_ - 1);
        if (pulse_remaining_ == 0) refractory_remaining_ = refractory_ticks_;
    } else {
        // No contact, not pulsing → silent.
        last_head_on_dir_ = 0;
        last_skipped_     = true;
        return;
    }

    if (!output_topic_.empty()) {
        // Single-channel mode (Phase 6.6.F).  Combine into one ActionOut
        // whose accel is the steering signal (al − ar), which is what
        // the body-side bilateral pathway computes anyway.  Suitable
        // input for a single-channel MotorFader.
        auto act = std::make_shared<ActionOut>();
        act->tick_id     = tick_id;
        act->producer_id = id_.empty() ? std::string("whisker_steer") : id_;
        act->accel       = std::clamp(al - ar, accel_min_, accel_max_);
        act->source      = "whisker_steer";
        bus_->publish(output_topic_, act);
    } else {
        auto act_l = std::make_shared<ActionOut>();
        act_l->tick_id     = tick_id;
        act_l->producer_id = id_.empty() ? std::string("whisker_steer") : id_;
        act_l->accel       = al;
        act_l->source      = "whisker_steer";
        act_l->probe       = false;
        act_l->action_tle  = 0.0f;
        bus_->publish(output_topic_left_, act_l);

        auto act_r = std::make_shared<ActionOut>();
        act_r->tick_id     = tick_id;
        act_r->producer_id = act_l->producer_id;
        act_r->accel       = ar;
        act_r->source      = "whisker_steer";
        act_r->probe       = false;
        act_r->action_tle  = 0.0f;
        bus_->publish(output_topic_right_, act_r);
    }

    last_al_ = al;
    last_ar_ = ar;
}

nlohmann::json WhiskerSteerReflex::snapshot_state() const {
    std::ostringstream os; os << head_on_rng_;
    nlohmann::json values = nlohmann::json::object();
    for (auto const& [k, v] : last_values_) values[k] = v;
    return nlohmann::json{
        {"version",              1},
        {"head_on_rng",          os.str()},
        {"last_values",          values},
        {"refractory_remaining", refractory_remaining_},
        {"fire_count",           fire_count_},
        {"last_al",              last_al_},
        {"last_ar",              last_ar_},
        {"last_skipped",         last_skipped_},
        {"last_head_on",         last_head_on_},
        {"last_head_on_dir",     last_head_on_dir_},
        {"last_suppression",     last_suppression_},
    };
}

void WhiskerSteerReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("WhiskerSteerReflex::restore_state: unknown version " +
                                 std::to_string(version));
    }
    if (s.contains("head_on_rng") && s["head_on_rng"].is_string()) {
        std::istringstream is(s["head_on_rng"].get<std::string>()); is >> head_on_rng_;
    }
    last_values_.clear();
    if (s.contains("last_values") && s["last_values"].is_object())
        for (auto it = s["last_values"].begin(); it != s["last_values"].end(); ++it)
            last_values_[it.key()] = it.value().get<float>();
    refractory_remaining_ = s.value("refractory_remaining", refractory_remaining_);
    fire_count_           = s.value("fire_count",           fire_count_);
    last_al_              = s.value("last_al",              last_al_);
    last_ar_              = s.value("last_ar",              last_ar_);
    last_skipped_         = s.value("last_skipped",         last_skipped_);
    last_head_on_         = s.value("last_head_on",         last_head_on_);
    last_head_on_dir_     = s.value("last_head_on_dir",     last_head_on_dir_);
    last_suppression_     = s.value("last_suppression",     last_suppression_);
}

} // namespace ogma
