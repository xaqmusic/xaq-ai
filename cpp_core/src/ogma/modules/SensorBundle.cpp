#include "ogma/modules/SensorBundle.hpp"

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

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("SensorBundle: param '" + key + "' must be a string");
}

std::vector<std::string> get_string_list(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("SensorBundle: param '" + key + "' must be a list of strings");
}

} // namespace

SensorBundle::SensorBundle()  = default;
SensorBundle::~SensorBundle() = default;

std::string_view SensorBundle::type_name() const { return "SensorBundle"; }

std::vector<TopicSpec> SensorBundle::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(input_topics_.size());
    for (auto const& t : input_topics_)
        v.emplace_back(t, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> SensorBundle::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema SensorBundle::params_schema() const {
    return {
        {"input_topics", ParamMutability::ConstructionOnly,
            "Ordered list of ProprioToken topics to concatenate (the bundle is "
            "[topic0.values, topic1.values, ...]).",
            ParamValue{std::vector<std::string>{}}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Topic for the concatenated ProprioToken.",
            ParamValue{std::string("fast.proprio.bundle")}},
        {"sensor_label", ParamMutability::ConstructionOnly,
            "sensor field on the published ProprioToken.",
            ParamValue{std::string("bundle")}},
    };
}

ParamMap SensorBundle::current_params() const {
    ParamMap m;
    m["input_topics"] = ParamValue{input_topics_};
    m["output_topic"] = ParamValue{output_topic_};
    m["sensor_label"] = ParamValue{sensor_label_};
    return m;
}

void SensorBundle::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("SensorBundle requires a non-null Bus");

    apply_param(params, "input_topics",
        [&](auto const& v){ input_topics_ = get_string_list(v, "input_topics"); });
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "sensor_label",
        [&](auto const& v){ sensor_label_ = get_string(v, "sensor_label"); });

    last_values_.assign(input_topics_.size(), {});
    seen_.assign(input_topics_.size(), false);

    for (std::size_t i = 0; i < input_topics_.size(); ++i) {
        sub_ids_.push_back(bus_->subscribe(input_topics_[i], SubscriptionKind::Direct,
            [this, i](std::string_view /*topic*/, MessagePtr p){ handle_input(i, p); }));
    }
}

void SensorBundle::on_param_change(std::string_view key, ParamValue const& /*value*/) {
    throw std::invalid_argument("SensorBundle: param '" + std::string(key) +
                                "' is construction-only / unknown");
}

void SensorBundle::handle_input(std::size_t idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    if (idx >= last_values_.size()) return;
    last_values_[idx].assign(pt->values.begin(), pt->values.end());
    seen_[idx] = true;
}

void SensorBundle::tick(uint64_t tick_id) {
    // Only publish once every input has arrived at least once, so the bundle's
    // dimensionality is stable from the first publish (MotorEPM requires it).
    for (bool s : seen_) if (!s) return;

    int dim = 0;
    for (auto const& buf : last_values_) dim += int(buf.size());
    last_dim_ = dim;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("sensor_bundle") : id_;
    out->sensor      = sensor_label_;
    out->values.resize(dim);
    int k = 0;
    for (auto const& buf : last_values_)
        for (float v : buf) out->values[k++] = v;
    bus_->publish(output_topic_, out);
}

nlohmann::json SensorBundle::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"last_dim", last_dim_}};
}

void SensorBundle::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1)
        throw std::runtime_error("SensorBundle::restore_state: unknown version " +
                                 std::to_string(version));
    last_dim_ = s.value("last_dim", last_dim_);
}

} // namespace ogma
