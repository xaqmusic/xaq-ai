// =============================================================================
// GoalBelief.cpp  --  persistent goal-direction belief (path integration)
// =============================================================================
#include "ogma/modules/GoalBelief.hpp"

#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("GoalBelief param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("GoalBelief param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("GoalBelief param '" + key + "' must be string");
}
}

GoalBelief::GoalBelief()  = default;
GoalBelief::~GoalBelief() = default;

std::string_view GoalBelief::type_name() const { return "GoalBelief"; }

std::vector<TopicSpec> GoalBelief::input_topics() const {
    return { TopicSpec{scent_topic_,   std::type_index(typeid(ProprioToken))},
             TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken))} };
}
std::vector<TopicSpec> GoalBelief::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema GoalBelief::params_schema() const {
    return {
        {"scent_topic", ParamMutability::ConstructionOnly,
            "Egocentric goal-bearing ProprioToken [cx=+right, cy=+forward] (e.g. percept.scent_compass).",
            ParamValue{std::string("percept.scent_compass")}},
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Absolute body yaw ProprioToken (rad) for path integration.",
            ParamValue{std::string("reality.proprio.heading")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Egocentric belief direction [bx,by] (magnitude = confidence).",
            ParamValue{std::string("percept.goal_belief")}},
        {"cx_index", ParamMutability::ConstructionOnly, "+right index in scent token.", ParamValue{int64_t{0}}},
        {"cy_index", ParamMutability::ConstructionOnly, "+forward index in scent token.", ParamValue{int64_t{1}}},
        {"min_signal", ParamMutability::HotMutable,
            "|compass| above this = confident perception → correct the belief.", ParamValue{0.1}},
        {"update_rate", ParamMutability::HotMutable,
            "EMA correction rate toward the observed world direction when perceiving.", ParamValue{0.3}},
        {"decay", ParamMutability::HotMutable,
            "Per-tick confidence decay when NOT perceiving (bridges occlusion; <1).", ParamValue{0.995}},
    };
}

ParamMap GoalBelief::current_params() const {
    ParamMap m;
    m["scent_topic"]  = ParamValue{scent_topic_};
    m["heading_topic"]= ParamValue{heading_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["cx_index"]     = ParamValue{int64_t(cx_index_)};
    m["cy_index"]     = ParamValue{int64_t(cy_index_)};
    m["min_signal"]   = ParamValue{double(min_signal_)};
    m["update_rate"]  = ParamValue{double(update_rate_)};
    m["decay"]        = ParamValue{double(decay_)};
    return m;
}

void GoalBelief::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("GoalBelief requires a non-null Bus");
    apply_param(params, "scent_topic",   [&](auto const& v){ scent_topic_   = get_string(v, "scent_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v, "heading_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "cx_index",      [&](auto const& v){ cx_index_ = int(get_int(v, "cx_index")); });
    apply_param(params, "cy_index",      [&](auto const& v){ cy_index_ = int(get_int(v, "cy_index")); });
    apply_param(params, "min_signal",    [&](auto const& v){ min_signal_  = float(get_double(v, "min_signal")); });
    apply_param(params, "update_rate",   [&](auto const& v){ update_rate_ = float(get_double(v, "update_rate")); });
    apply_param(params, "decay",         [&](auto const& v){ decay_       = float(get_double(v, "decay")); });

    sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_heading(p); }));
}

void GoalBelief::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "min_signal")  min_signal_  = float(get_double(value, k));
    else if (k == "update_rate") update_rate_ = float(get_double(value, k));
    else if (k == "decay")       decay_       = float(get_double(value, k));
    else throw std::invalid_argument("GoalBelief: param '" + k + "' is construction-only / unknown");
}

void GoalBelief::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    cx_ = (cx_index_ < n) ? float(pt->values[cx_index_]) : 0.0f;
    cy_ = (cy_index_ < n) ? float(pt->values[cy_index_]) : 0.0f;
}

void GoalBelief::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { heading_ = float(pt->values[0]); have_heading_ = true; }
}

void GoalBelief::tick(uint64_t tick_id) {
    float ch = std::cos(heading_), sh = std::sin(heading_);
    float mag = std::sqrt(cx_ * cx_ + cy_ * cy_);
    bool perceiving = have_heading_ && (mag > min_signal_);
    last_perceiving_ = perceiving;

    if (perceiving) {
        // Rotate the egocentric observation [cx,cy] into the WORLD frame by +heading,
        // then EMA-correct the stored world goal toward it. (R(+θ): wx=cx·cosθ−cy·sinθ,
        // wy=cx·sinθ+cy·cosθ.)  Confidence → 1 (we have a fix).
        float ux = cx_ / mag, uy = cy_ / mag;
        float wx = ux * ch - uy * sh;
        float wy = ux * sh + uy * ch;
        world_gx_ += update_rate_ * (wx - world_gx_);
        world_gy_ += update_rate_ * (wy - world_gy_);
        confidence_ = 1.0f;
    } else {
        confidence_ *= decay_;   // hold the world goal; fade trust without a fix
    }

    // Re-project the world goal into the CURRENT egocentric frame by −heading
    // (R(−θ): bx=wx·cosθ+wy·sinθ, by=−wx·sinθ+wy·cosθ).  Scale by confidence so a
    // long signal loss fades the belief (controller's nav-gate then explores).
    float bx = world_gx_ * ch + world_gy_ * sh;
    float by = -world_gx_ * sh + world_gy_ * ch;
    belief_x_ = bx * confidence_;
    belief_y_ = by * confidence_;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id;
    out->producer_id = id_.empty() ? std::string("goal_belief") : id_;
    out->sensor = "goal_belief";
    out->values.resize(2);
    out->values[0] = belief_x_;
    out->values[1] = belief_y_;
    bus_->publish(output_topic_, out);
}

nlohmann::json GoalBelief::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"wx", world_gx_}, {"wy", world_gy_}, {"conf", confidence_}};
}
nlohmann::json GoalBelief::diag_snapshot() const {
    return nlohmann::json{{"belief_x", belief_x_}, {"belief_y", belief_y_},
                          {"confidence", confidence_}, {"perceiving", last_perceiving_}};
}
void GoalBelief::restore_state(nlohmann::json const& s) {
    world_gx_   = s.value("wx", world_gx_);
    world_gy_   = s.value("wy", world_gy_);
    confidence_ = s.value("conf", confidence_);
}

} // namespace ogma
