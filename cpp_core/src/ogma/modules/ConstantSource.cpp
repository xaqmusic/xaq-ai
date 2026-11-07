#include "ogma/modules/ConstantSource.hpp"
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
    throw std::invalid_argument("ConstantSource: param '" + k + "' must be numeric");
}
std::vector<double> get_list(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("ConstantSource: param '" + k + "' must be a list of numbers");
}
} // namespace

ConstantSource::ConstantSource()  = default;
ConstantSource::~ConstantSource() = default;
std::string_view ConstantSource::type_name() const { return "ConstantSource"; }
std::vector<TopicSpec> ConstantSource::output_topics() const { return { TopicSpec{topic_, std::type_index(typeid(ProprioToken))} }; }

ParamSchema ConstantSource::params_schema() const {
    return {
        {"topic",      ParamMutability::ConstructionOnly, "The topic the vector is published on (ProprioToken).", ParamValue{std::string("constant.out")}},
        {"sensor",     ParamMutability::ConstructionOnly, "The token's sensor name.", ParamValue{std::string("constant")}},
        {"values",     ParamMutability::HotMutable, "The vector published inside the window.", ParamValue{std::vector<double>{}}},
        {"off_values", ParamMutability::HotMutable, "The vector published outside the window (empty = nothing published there).", ParamValue{std::vector<double>{}}},
        {"from_tick",  ParamMutability::HotMutable, "The window's first tick.", ParamValue{int64_t{0}}},
        {"to_tick",    ParamMutability::HotMutable, "The window's last tick (0 = forever).", ParamValue{int64_t{0}}},
    };
}
ParamMap ConstantSource::current_params() const {
    return {{"topic", ParamValue{topic_}}, {"sensor", ParamValue{sensor_}}, {"values", ParamValue{values_}}, {"off_values", ParamValue{off_values_}},
            {"from_tick", ParamValue{from_tick_}}, {"to_tick", ParamValue{to_tick_}}};
}
void ConstantSource::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ConstantSource requires a non-null Bus");
    apply_param(params, "topic",  [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) topic_ = *p; });
    apply_param(params, "sensor", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) sensor_ = *p; });
    for (auto const& k : {"values", "off_values", "from_tick", "to_tick"}) apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
}
void ConstantSource::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "values")     values_ = get_list(value, k);
    else if (k == "off_values") off_values_ = get_list(value, k);
    else if (k == "from_tick")  from_tick_ = int64_t(get_double(value, k));
    else if (k == "to_tick")    to_tick_ = int64_t(get_double(value, k));
}
void ConstantSource::tick(uint64_t tick_id) {
    const bool in = int64_t(tick_id) >= from_tick_ && (to_tick_ <= 0 || int64_t(tick_id) <= to_tick_);
    auto const& v = in ? values_ : off_values_;
    if (v.empty()) return;
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("constant_source") : std::string(id()); out->sensor = sensor_;
    out->values = Eigen::VectorXf(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) out->values[i] = float(v[size_t(i)]);
    bus_->publish(topic_, out);
    ++published_; if (in) ++in_window_;
}
nlohmann::json ConstantSource::diag_snapshot() const {
    return nlohmann::json{{"published", published_}, {"in_window", in_window_}, {"values", values_}};
}

} // namespace ogma
