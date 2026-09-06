// =============================================================================
// TofAvoidLoop.cpp  --  see the header
// =============================================================================
#include "ogma/modules/TofAvoidLoop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("TofAvoidLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("TofAvoidLoop: param '" + k + "' must be a string");
}
} // namespace

TofAvoidLoop::TofAvoidLoop()  = default;
TofAvoidLoop::~TofAvoidLoop() = default;

std::string_view TofAvoidLoop::type_name() const { return "TofAvoidLoop"; }

std::vector<TopicSpec> TofAvoidLoop::input_topics() const {
    return { TopicSpec{prox_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> TofAvoidLoop::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))},
             TopicSpec{value_topic_,  std::type_index(typeid(ProprioToken))} };
}

ParamSchema TofAvoidLoop::params_schema() const {
    return {
        {"prox_topic", ParamMutability::ConstructionOnly,
            "ProprioToken [ahead-left, ahead, ahead-right, too_close], each in [0,1] (1 = touching): the duck's ToF summary.",
            ParamValue{std::string("reality.proprio.tof")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Egocentric bearing AWAY from the nearest obstacle [cx=+right, cy=+forward, prox]; zero when nothing is near.",
            ParamValue{std::string("percept.avoid_bearing")}},
        {"value_topic", ParamMutability::ConstructionOnly,
            "The loop's need/value: the nearest proximity in [0,1] (the arbiter's preference for this loop).",
            ParamValue{std::string("reality.cognitive.avoid_value")}},
        {"floor", ParamMutability::HotMutable,
            "Nearest proximity below which the loop is silent (a zero bearing and value).", ParamValue{0.05}},
        {"emit_bearing", ParamMutability::HotMutable,
            "true: publish the away-bearing. false: publish a ZERO bearing (the value/need unchanged) -- to a heading-reference consumer a winning loop with no bearing means RELEASE the reference so the fast proximity priors act unopposed.", ParamValue{true}},
    };
}

ParamMap TofAvoidLoop::current_params() const {
    ParamMap m;
    m["prox_topic"] = ParamValue{prox_topic_}; m["output_topic"] = ParamValue{output_topic_};
    m["value_topic"] = ParamValue{value_topic_}; m["floor"] = ParamValue{double(floor_)}; m["emit_bearing"] = ParamValue{emit_bearing_};
    return m;
}

void TofAvoidLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("TofAvoidLoop requires a non-null Bus");
    apply_param(params, "prox_topic",   [&](auto const& v){ prox_topic_   = get_string(v,"prox_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v,"output_topic"); });
    apply_param(params, "value_topic",  [&](auto const& v){ value_topic_  = get_string(v,"value_topic"); });
    apply_param(params, "floor",        [&](auto const& v){ floor_        = float(get_double(v,"floor")); });
    apply_param(params, "emit_bearing", [&](auto const& v){ if (auto b = std::get_if<bool>(&v)) emit_bearing_ = *b; });
    sub_ids_.push_back(bus_->subscribe(prox_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_prox(p); }));
}

void TofAvoidLoop::on_param_change(std::string_view key, ParamValue const& value) {
    if (key == "floor") floor_ = float(get_double(value, "floor"));
    else if (key == "emit_bearing") { if (auto b = std::get_if<bool>(&value)) emit_bearing_ = *b; }
    else throw std::invalid_argument("TofAvoidLoop: param '" + std::string(key) + "' is construction-only / unknown");
}

void TofAvoidLoop::handle_prox(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 3) return;
    left_ = float(pt->values[0]); ahead_ = float(pt->values[1]); right_ = float(pt->values[2]);
    tooclose_ = pt->values.size() > 3 ? float(pt->values[3]) : 0.0f;
    have_ = true;
}

void TofAvoidLoop::tick(uint64_t tick_id) {
    const float nearest = have_ ? std::max(left_, std::max(ahead_, right_)) : 0.0f;
    if (nearest < floor_) { cx_ = 0.0f; cy_ = 0.0f; value_ = 0.0f; }
    else {
        // Away from the nearer side; something ahead pushes the bearing sideways (to the freer
        // side) and reduces the forward component; symmetric obstacles resolve to the freer side.
        const float side = right_ - left_;                  // > 0: the right is nearer → bear LEFT (cx < 0)
        float cx = -side;
        if (std::fabs(side) < 1e-3f && ahead_ >= floor_) cx = (left_ <= right_) ? -1.0f : 1.0f;   // tie: turn toward the (weakly) freer side
        float cy = 1.0f - ahead_;                            // free ahead → keep going; blocked ahead → turn in place
        const float n = std::sqrt(cx * cx + cy * cy);
        cx_ = n > 1e-6f ? nearest * cx / n : 0.0f;
        cy_ = n > 1e-6f ? nearest * cy / n : 0.0f;
        value_ = nearest;
    }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("avoid") : id_; out->sensor = "avoid_bearing";
    out->values = Eigen::VectorXf::Zero(3);
    if (emit_bearing_) { out->values[0] = cx_; out->values[1] = cy_; }
    out->values[2] = value_;
    bus_->publish(output_topic_, out);
    auto v = std::make_shared<ProprioToken>();
    v->tick_id = tick_id; v->producer_id = out->producer_id; v->sensor = "avoid_value";
    v->values = Eigen::VectorXf::Constant(1, value_);
    bus_->publish(value_topic_, v);
}

nlohmann::json TofAvoidLoop::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"left", left_}, {"ahead", ahead_}, {"right", right_}, {"tooclose", tooclose_}, {"have", have_}};
}
void TofAvoidLoop::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    left_ = s.value("left", 0.0f); ahead_ = s.value("ahead", 0.0f); right_ = s.value("right", 0.0f);
    tooclose_ = s.value("tooclose", 0.0f); have_ = s.value("have", false);
}
nlohmann::json TofAvoidLoop::diag_snapshot() const {
    return nlohmann::json{{"cx", cx_}, {"cy", cy_}, {"value", value_}, {"left", left_}, {"ahead", ahead_}, {"right", right_}};
}

} // namespace ogma
