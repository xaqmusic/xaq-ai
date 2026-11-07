#include "ogma/modules/FootholdShaper.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

namespace ogma {
namespace {
template <typename Fn> void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key); if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("FootholdShaper: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("FootholdShaper: param '" + k + "' must be a string");
}
} // namespace

FootholdShaper::FootholdShaper()  = default;
FootholdShaper::~FootholdShaper() = default;
std::string_view FootholdShaper::type_name() const { return "FootholdShaper"; }

std::vector<TopicSpec> FootholdShaper::input_topics() const {
    return { TopicSpec{predict_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{contact_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{feet_y_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> FootholdShaper::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema FootholdShaper::params_schema() const {
    return {
        {"predict_topic", ParamMutability::ConstructionOnly, "ProprioToken [h_pred, precision, has, fwd, lat, spread] x 4 (TerrainMemory).", ParamValue{std::string("predict.foothold")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4] per-leg contact: the planted toes define the ground the body stands on.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4] the toes' gravity heights (reality.proprio.toe_y).", ParamValue{std::string("reality.proprio.toe_y")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "ProprioToken [lift_mult, reach_m] x 4 for MotorEPMv2's swing_target_topic.", ParamValue{std::string("swing.targets")}},
        {"lift_gain",     ParamMutability::HotMutable, "Lift multiplier gained per metre of predicted rise above the planted ground (1 + gain x rise); 0 = no lift shaping.", ParamValue{0.0}},
        {"lift_max",      ParamMutability::HotMutable, "The lift multiplier's cap.", ParamValue{2.5}},
        {"rise_min_m",    ParamMutability::HotMutable, "A predicted rise under this is the ground's noise, not a step.", ParamValue{0.01}},
        {"edge_min_m",    ParamMutability::HotMutable, "A prediction whose spread exceeds this is a landing on an edge.", ParamValue{0.02}},
        {"edge_shift_m",  ParamMutability::HotMutable, "How far forward an edge landing is moved (m); 0 = no reach shaping.", ParamValue{0.0}},
        {"prec_full",     ParamMutability::HotMutable, "The prediction precision (1/m^2 summed over the pooled voxels) at which the policy acts in full; below it, in proportion.", ParamValue{2000.0}},
    };
}

ParamMap FootholdShaper::current_params() const {
    return {{"predict_topic", ParamValue{predict_topic_}}, {"contact_topic", ParamValue{contact_topic_}}, {"feet_y_topic", ParamValue{feet_y_topic_}},
            {"output_topic", ParamValue{output_topic_}}, {"lift_gain", ParamValue{lift_gain_}}, {"lift_max", ParamValue{lift_max_}}, {"rise_min_m", ParamValue{rise_min_m_}},
            {"edge_min_m", ParamValue{edge_min_m_}}, {"edge_shift_m", ParamValue{edge_shift_m_}}, {"prec_full", ParamValue{prec_full_}}};
}

void FootholdShaper::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("FootholdShaper requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{{"predict_topic", &predict_topic_}, {"contact_topic", &contact_topic_}, {"feet_y_topic", &feet_y_topic_}, {"output_topic", &output_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    for (auto const& k : {"lift_gain", "lift_max", "rise_min_m", "edge_min_m", "edge_shift_m", "prec_full"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which) {
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which](std::string_view, MessagePtr p){ handle(which, p); }));
    };
    sub(predict_topic_, "pred"); sub(contact_topic_, "contact"); sub(feet_y_topic_, "y");
}

void FootholdShaper::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "lift_gain")    lift_gain_ = std::max(0.0, get_double(value, k));
    else if (k == "lift_max")     lift_max_ = std::max(1.0, get_double(value, k));
    else if (k == "rise_min_m")   rise_min_m_ = std::max(0.0, get_double(value, k));
    else if (k == "edge_min_m")   edge_min_m_ = std::max(0.0, get_double(value, k));
    else if (k == "edge_shift_m") edge_shift_m_ = std::clamp(get_double(value, k), -0.1, 0.1);
    else if (k == "prec_full")    prec_full_ = std::max(1.0, get_double(value, k));
    else throw std::invalid_argument("FootholdShaper: param '" + k + "' is construction-only / unknown");
}

void FootholdShaper::handle(std::string const& which, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if      (which == "pred"    && v.size() >= 24) { for (int i = 0; i < 24; ++i) pred_[size_t(i)] = v[i]; have_pred_ = true; }
    else if (which == "contact" && v.size() >= 4)  { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = v[i]; have_contact_ = true; }
    else if (which == "y"       && v.size() >= 4)  { for (int i = 0; i < 4; ++i) feet_y_[size_t(i)] = v[i]; have_y_ = true; }
}

void FootholdShaper::tick(uint64_t tick_id) {
    // the ground the body stands on: the median gravity height of the planted toes (the fallback reference); and
    // where each toe last stood: its height at its last touchdown (the rise a hung rear leg must make)
    if (have_contact_ && have_y_) {
        float pl[4]; int n = 0;
        for (int i = 0; i < 4; ++i) {
            if (contact_[size_t(i)] > 0.5f) { pl[n++] = feet_y_[size_t(i)]; if (prev_contact_[size_t(i)] <= 0.5f || !have_touch_[size_t(i)]) { touch_y_[size_t(i)] = feet_y_[size_t(i)]; have_touch_[size_t(i)] = true; } }
            prev_contact_[size_t(i)] = contact_[size_t(i)];
        }
        if (n > 0) { std::sort(pl, pl + n); ground_ = (n % 2) ? pl[n / 2] : 0.5 * (pl[n / 2 - 1] + pl[n / 2]); have_ground_ = true; }
    }
    for (int i = 0; i < 4; ++i) {
        float lift = 1.0f, reach = 0.0f; rise_[size_t(i)] = 0.0f;
        if (have_pred_ && pred_[size_t(6 * i + 2)] > 0.5f && have_ground_) {
            const double h = pred_[size_t(6 * i)], prec = pred_[size_t(6 * i + 1)], spread = pred_[size_t(6 * i + 5)];
            const double trust = std::clamp(prec / prec_full_, 0.0, 1.0);
            const double from = have_touch_[size_t(i)] ? double(touch_y_[size_t(i)]) : ground_;
            const double rise = h - from;
            rise_[size_t(i)] = float(rise);
            if (lift_gain_ > 0.0 && rise > rise_min_m_) { lift = float(std::min(lift_max_, 1.0 + lift_gain_ * rise * trust)); if (lift > 1.0f) ++lifts_; }
            if (edge_shift_m_ != 0.0 && spread > edge_min_m_ && rise > rise_min_m_) { reach = float(edge_shift_m_ * trust); if (reach != 0.0f) ++shifts_; }
        }
        out_[size_t(2 * i)] = lift; out_[size_t(2 * i + 1)] = reach;
    }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("foothold") : std::string(id()); out->sensor = "swing_targets";
    out->values = Eigen::VectorXf(8); for (int k = 0; k < 8; ++k) out->values[k] = out_[size_t(k)];
    bus_->publish(output_topic_, out);
}

nlohmann::json FootholdShaper::diag_snapshot() const {
    return nlohmann::json{{"lift", {out_[0], out_[2], out_[4], out_[6]}}, {"reach", {out_[1], out_[3], out_[5], out_[7]}},
                          {"rise", {rise_[0], rise_[1], rise_[2], rise_[3]}}, {"ground", ground_}, {"lift_ticks", lifts_}, {"shift_ticks", shifts_},
                          {"have", {{"pred", have_pred_}, {"contact", have_contact_}, {"y", have_y_}}}};
}

} // namespace ogma
