// =============================================================================
// ThingBelief.cpp  --  see the header (the duck, design doc §17.113)
// =============================================================================
#include "ogma/modules/ThingBelief.hpp"

#include <algorithm>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ThingBelief: param '" + k + "' must be a string");
}
} // namespace

ThingBelief::ThingBelief()  = default;
ThingBelief::~ThingBelief() = default;

std::string_view ThingBelief::type_name() const { return "ThingBelief"; }

std::vector<TopicSpec> ThingBelief::input_topics() const {
    return { TopicSpec{target_topic_,   std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{presence_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}

std::vector<TopicSpec> ThingBelief::output_topics() const {
    return { TopicSpec{belief_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema ThingBelief::params_schema() const {
    return {
        {"target_topic", ParamMutability::ConstructionOnly, "The seek loop's target_out_topic [have, x, y, set tick, chasing].", std::nullopt},
        {"presence_topic", ParamMutability::ConstructionOnly, "CloudMap's presence_topic [confirmed, refuted, in the cloud, top].", std::nullopt},
        {"belief_topic", ParamMutability::ConstructionOnly,
            "Output [presence, ticks confirming, ticks refuting, ticks since the target was set, have a target].", std::nullopt},
    };
}

ParamMap ThingBelief::current_params() const {
    ParamMap m;
    m["target_topic"] = ParamValue{target_topic_}; m["presence_topic"] = ParamValue{presence_topic_};
    m["belief_topic"] = ParamValue{belief_topic_};
    return m;
}

void ThingBelief::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ThingBelief requires a non-null Bus");
    for (auto const& [k, dst] : {std::pair<const char*, std::string*>{"target_topic", &target_topic_},
                                 {"presence_topic", &presence_topic_}, {"belief_topic", &belief_topic_}}) {
        auto it = params.find(k);
        if (it == params.end()) throw std::invalid_argument(std::string("ThingBelief: ") + k + " is required");
        *dst = get_string(it->second, k);
    }
}

void ThingBelief::on_param_change(std::string_view key, ParamValue const&) {
    throw std::invalid_argument("ThingBelief: param '" + std::string(key) + "' is construction-only / unknown");
}

void ThingBelief::tick(uint64_t tick_id) {
    bool have = false, chasing = false; double set_tick = -1.0;
    if (auto t = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(target_topic_)))
        if (t->values.size() >= 5) { have = t->values[0] > 0.5f; set_tick = double(t->values[3]); chasing = t->values[4] > 0.5f; }
    // a new thing: a target taken, a re-fix elsewhere, or the end of a chase (the stopped thing is a place from here)
    if (have && (!have_ || set_tick != set_tick_ || (chasing_ && !chasing))) { conf_ = 0; refute_ = 0; since_ = 0; }
    have_ = have; chasing_ = chasing; set_tick_ = set_tick;
    if (have) {
        ++since_;
        if (auto p = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(presence_topic_)))
            if (p->values.size() >= 3 && p->values[2] > 0.5f) {
                if (p->values[0] > 0.5f) ++conf_;
                else if (p->values[1] > 0.5f) ++refute_;
            }
    }
    presence_ = chasing ? 1.0 : double(1 + conf_) / double(2 + conf_ + refute_);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("thing_belief") : id_; out->sensor = "thing_belief";
    out->values = Eigen::VectorXf(5);
    out->values[0] = float(presence_); out->values[1] = float(conf_); out->values[2] = float(refute_);
    out->values[3] = float(since_); out->values[4] = have ? 1.0f : 0.0f;
    bus_->publish(belief_topic_, out);
}

nlohmann::json ThingBelief::diag_snapshot() const {
    return nlohmann::json{{"presence", presence_}, {"confirming", conf_}, {"refuting", refute_}, {"since", since_},
                          {"have", have_}, {"chasing", chasing_}};
}

} // namespace ogma
