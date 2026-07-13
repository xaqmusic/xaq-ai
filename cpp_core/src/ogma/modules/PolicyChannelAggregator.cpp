#include "ogma/modules/PolicyChannelAggregator.hpp"

#include <algorithm>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PolicyChannelAggregator param '" + key + "' must be string");
}

std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("PolicyChannelAggregator param '" + key + "' must be a string array");
}

std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("PolicyChannelAggregator param '" + key + "' must be a numeric array");
}

} // namespace

PolicyChannelAggregator::PolicyChannelAggregator()  = default;
PolicyChannelAggregator::~PolicyChannelAggregator() = default;

std::string_view PolicyChannelAggregator::type_name() const { return "PolicyChannelAggregator"; }

std::vector<TopicSpec> PolicyChannelAggregator::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(input_topics_.size());
    for (auto const& t : input_topics_) {
        v.emplace_back(t, std::type_index(typeid(PolicyToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    }
    return v;
}

std::vector<TopicSpec> PolicyChannelAggregator::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(PolicyToken))} };
}

ParamSchema PolicyChannelAggregator::params_schema() const {
    return {
        {"input_topics", ParamMutability::ConstructionOnly,
            "Array of N per-channel PolicyToken topics (e.g. [\"policy.intent.fl_hip1\", ...]).  Each upstream Premotor publishes to its own topic via policy_output_topic; this module collects them and packs chosen_intent into one combined index.",
            std::nullopt},
        {"channel_radix", ParamMutability::ConstructionOnly,
            "Array of N integers giving n_intents per channel (e.g. [5,5,5] for 3 joints of 5 intents each).  combined = c[0] + c[1]*R[0] + c[2]*R[0]*R[1] + ...; same array used by ActionDecoder.intent_channel_radix to unpack at dispatch.",
            std::nullopt},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Aggregated PolicyToken output topic (e.g. \"policy.intent.leg.fl\").",
            ParamValue{std::string("policy.intent.aggregated")}},
    };
}

ParamMap PolicyChannelAggregator::current_params() const {
    ParamMap p;
    p["input_topics"] = ParamValue{input_topics_};
    std::vector<double> r;
    r.reserve(channel_radix_.size());
    for (int v : channel_radix_) r.push_back(double(v));
    p["channel_radix"] = ParamValue{r};
    p["output_topic"]  = ParamValue{output_topic_};
    return p;
}

void PolicyChannelAggregator::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("PolicyChannelAggregator requires a non-null Bus");

    apply_param(params, "output_topic", [&](auto const& v){
        output_topic_ = get_string(v, "output_topic"); });

    auto it_t = params.find("input_topics");
    auto it_r = params.find("channel_radix");
    if (it_t == params.end() || it_r == params.end()) {
        throw std::invalid_argument(
            "PolicyChannelAggregator: input_topics and channel_radix are required");
    }
    input_topics_ = get_string_vec(it_t->second, "input_topics");
    auto rd       = get_double_vec(it_r->second, "channel_radix");
    channel_radix_.clear();
    channel_radix_.reserve(rd.size());
    for (double d : rd) channel_radix_.push_back(std::max(1, int(d)));

    if (input_topics_.empty() || input_topics_.size() != channel_radix_.size()) {
        throw std::invalid_argument(
            "PolicyChannelAggregator: input_topics and channel_radix must be non-empty and same length");
    }

    latest_.assign(input_topics_.size(), -1);

    sub_ids_.clear();
    for (int i = 0; i < int(input_topics_.size()); ++i) {
        sub_ids_.push_back(bus_->subscribe(input_topics_[i], SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ this->handle_policy(i, p); }));
    }
}

void PolicyChannelAggregator::on_param_change(std::string_view key, ParamValue const& /*value*/) {
    throw std::invalid_argument(
        "PolicyChannelAggregator: param '" + std::string(key) + "' is ConstructionOnly");
}

void PolicyChannelAggregator::handle_policy(int channel, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!p) return;
    if (channel >= 0 && channel < int(latest_.size())) {
        latest_[channel] = p->chosen_intent;
    }
}

void PolicyChannelAggregator::tick(uint64_t tick_id) {
    int n = int(latest_.size());
    // Skip publish until every channel has reported a non-negative chosen_intent
    // at least once — avoids emitting a degenerate combined index during the
    // bootstrap window when Premotors haven't yet seen a valid latent.
    for (int v : latest_) if (v < 0) return;

    int combined = 0;
    int stride = 1;
    for (int i = 0; i < n; ++i) {
        int c = std::clamp(latest_[i], 0, channel_radix_[i] - 1);
        combined += c * stride;
        stride   *= channel_radix_[i];
    }

    auto pol = std::make_shared<PolicyToken>();
    pol->tick_id        = tick_id;
    pol->producer_id    = id_.empty() ? std::string("policy_aggregator") : id_;
    pol->chosen_intent  = combined;
    // intent_distribution intentionally left default-sized; downstream
    // MotorRepertoire only reads chosen_intent.

    bus_->publish(output_topic_, pol);
    ++total_publishes_;
}

} // namespace ogma
