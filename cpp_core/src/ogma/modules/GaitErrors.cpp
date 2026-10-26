#include "ogma/modules/GaitErrors.hpp"

#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("GaitErrors param '" + key + "' must be a string");
}
}  // namespace

std::vector<TopicSpec> GaitErrors::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : phase_topics_) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    v.push_back(TopicSpec{contact_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!stall_topic_.empty()) v.push_back(TopicSpec{stall_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> GaitErrors::output_topics() const {
    return {TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))}};
}
ParamSchema GaitErrors::params_schema() const {
    return {
        {"phase_topics", ParamMutability::ConstructionOnly, "The gait's per-leg rhythm topics, fl fr rl rr: [cos, sin, swing (1/0), load].",
         ParamValue{phase_topics_}},
        {"contact_topic", ParamMutability::ConstructionOnly, "Per-leg foot contact [4].", ParamValue{contact_topic_}},
        {"stall_topic", ParamMutability::ConstructionOnly, "StuckSense output (value 0 = stall score); empty = publish 0.", ParamValue{stall_topic_}},
        {"output_topic", ParamMutability::ConstructionOnly, "Published [sc_fl, sc_fr, sc_rl, sc_rr, stall].", ParamValue{output_topic_}},
    };
}
ParamMap GaitErrors::current_params() const {
    return {{"phase_topics", ParamValue{phase_topics_}}, {"contact_topic", ParamValue{contact_topic_}},
            {"stall_topic", ParamValue{stall_topic_}}, {"output_topic", ParamValue{output_topic_}}};
}
void GaitErrors::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("GaitErrors requires a non-null Bus");
    if (auto it = params.find("phase_topics"); it != params.end()) {
        if (auto p = std::get_if<std::vector<std::string>>(&it->second)) phase_topics_ = *p;
        else throw std::invalid_argument("GaitErrors param 'phase_topics' must be a list of strings");
    }
    if (auto it = params.find("contact_topic"); it != params.end()) contact_topic_ = get_string(it->second, "contact_topic");
    if (auto it = params.find("stall_topic");   it != params.end()) stall_topic_   = get_string(it->second, "stall_topic");
    if (auto it = params.find("output_topic");  it != params.end()) output_topic_  = get_string(it->second, "output_topic");
    if (phase_topics_.size() != 4) throw std::invalid_argument("GaitErrors: phase_topics must name 4 legs");
    for (size_t i = 0; i < 4; ++i)
        sub_ids_.push_back(bus_->subscribe(phase_topics_[i], SubscriptionKind::Direct, [this, i](std::string_view, MessagePtr p){
            if (!input_allowed(p->producer_id)) return;
            auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (pt && pt->values.size() >= 3) { swing_[i] = pt->values[2]; have_phase_ = true; }
        }));
    sub_ids_.push_back(bus_->subscribe(contact_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){
        if (!input_allowed(p->producer_id)) return;
        auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
        if (pt && pt->values.size() >= 4) { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = pt->values[i]; have_contact_ = true; }
    }));
    if (!stall_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(stall_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){
            if (!input_allowed(p->producer_id)) return;
            auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (pt && pt->values.size() >= 1) stall_ = pt->values[0];
        }));
}
void GaitErrors::on_param_change(std::string_view key, ParamValue const&) {
    throw std::invalid_argument("GaitErrors param '" + std::string(key) + "' is ConstructionOnly or unknown");
}
void GaitErrors::tick(uint64_t) {
    ++ticks_;
    if (!have_phase_ || !have_contact_) return;     // no input yet: publish nothing rather than a plausible zero
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = ticks_; out->producer_id = std::string(id()); out->sensor = "gait_errors";
    out->values.resize(5);
    for (int i = 0; i < 4; ++i) {
        const float sc = (swing_[size_t(i)] > 0.5f && contact_[size_t(i)] > 0.5f) ? 1.0f : 0.0f;
        out->values[i] = sc;
        sc_ema_[i] += 0.01 * (double(sc) - sc_ema_[i]);
    }
    out->values[4] = stall_;
    bus_->publish(output_topic_, out);
}
nlohmann::json GaitErrors::diag_snapshot() const {
    return nlohmann::json{{"sc_ema", {sc_ema_[0], sc_ema_[1], sc_ema_[2], sc_ema_[3]}}, {"stall", stall_}};
}

} // namespace ogma
