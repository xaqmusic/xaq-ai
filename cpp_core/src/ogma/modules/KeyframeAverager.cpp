#include "ogma/modules/KeyframeAverager.hpp"

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

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("KeyframeAverager param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("KeyframeAverager param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("KeyframeAverager param '" + key + "' must be string");
}

} // namespace

KeyframeAverager::KeyframeAverager()  = default;
KeyframeAverager::~KeyframeAverager() = default;

std::string_view KeyframeAverager::type_name() const { return "KeyframeAverager"; }

std::vector<TopicSpec> KeyframeAverager::input_topics() const {
    // Type discriminated by payload_kind_; we declare ProprioToken when in
    // proprio_token mode and ActionOut otherwise.  Both paths use Direct
    // subscription — no Feedback semantics needed (averaging over ticks is
    // forward-only).
    if (payload_kind_ == "proprio_token") {
        return { TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)),
                            SubscriptionKind::Direct, /*required=*/false} };
    }
    return { TopicSpec{input_topic_, std::type_index(typeid(ActionOut)),
                        SubscriptionKind::Direct, /*required=*/false} };
}

std::vector<TopicSpec> KeyframeAverager::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamMap KeyframeAverager::current_params() const {
    return {
        {"input_topic",   ParamValue{input_topic_}},
        {"output_topic",  ParamValue{output_topic_}},
        {"payload_kind",  ParamValue{payload_kind_}},
        {"window_size",   ParamValue{int64_t(window_size_)}},
        {"sensor_label",  ParamValue{sensor_label_}},
    };
}

ParamSchema KeyframeAverager::params_schema() const {
    return {
        {"input_topic",   ParamMutability::ConstructionOnly,
            "Source topic to subscribe to.  Payload type discriminated by payload_kind.",
            ParamValue{std::string("action.out")}},
        {"output_topic",  ParamMutability::ConstructionOnly,
            "Destination topic for published ProprioToken keyframes.",
            ParamValue{std::string("reality.proprio.motor_avg")}},
        {"payload_kind",  ParamMutability::ConstructionOnly,
            "Source payload type: 'action_out' (read .accel as 1-element vector) or 'proprio_token' (read .values as N-element vector).  Output is always a ProprioToken.",
            ParamValue{std::string("action_out")}},
        {"window_size",   ParamMutability::HotMutable,
            "Number of recent input frames to average.  At 60Hz, window_size=50 = ~833ms keyframe.  Default 50.",
            ParamValue{int64_t{50}}},
        {"sensor_label",  ParamMutability::ConstructionOnly,
            "Optional ProprioToken.sensor field for the published keyframes.  Empty = use module id.",
            ParamValue{std::string("")}},
    };
}

void KeyframeAverager::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("KeyframeAverager requires a non-null Bus");

    apply_param(params, "input_topic",   [&](auto const& v){ input_topic_   = get_string(v, "input_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "payload_kind",  [&](auto const& v){ payload_kind_  = get_string(v, "payload_kind"); });
    apply_param(params, "window_size",   [&](auto const& v){ window_size_   = std::max(1, int(get_int(v, "window_size"))); });
    apply_param(params, "sensor_label",  [&](auto const& v){ sensor_label_  = get_string(v, "sensor_label"); });

    if (payload_kind_ != "action_out" && payload_kind_ != "proprio_token") {
        throw std::invalid_argument(
            "KeyframeAverager: payload_kind must be 'action_out' or 'proprio_token' (got '" + payload_kind_ + "')");
    }

    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_input(p); }));
}

void KeyframeAverager::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "window_size") {
        window_size_ = std::max(1, int(get_int(value, k)));
        // Trim buffer if it exceeds the new window.
        while (int(buffer_.size()) > window_size_) buffer_.pop_front();
    }
    else if (k == "input_topic" || k == "output_topic"
          || k == "payload_kind" || k == "sensor_label") {
        throw std::invalid_argument("KeyframeAverager param '" + k + "' is ConstructionOnly");
    }
    else {
        throw std::invalid_argument("KeyframeAverager: unknown param '" + k + "'");
    }
}

void KeyframeAverager::handle_input(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;

    std::vector<float> frame;
    if (payload_kind_ == "action_out") {
        auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
        if (!a) return;
        frame.push_back(a->accel);
    } else {
        auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
        if (!p) return;
        frame.assign(p->values.begin(), p->values.end());
    }
    if (frame.empty()) return;

    // Establish payload_dim on first frame; subsequent frames must match.
    if (payload_dim_ == 0) {
        payload_dim_ = int(frame.size());
        last_mean_.assign(payload_dim_, 0.0f);
    } else if (int(frame.size()) != payload_dim_) {
        return;   // dim mismatch — ignore stray frames
    }

    buffer_.push_back(std::move(frame));
    while (int(buffer_.size()) > window_size_) buffer_.pop_front();
    ++total_inputs_seen_;
}

void KeyframeAverager::tick(uint64_t tick_id) {
    if (buffer_.empty() || payload_dim_ == 0) return;

    // Per-tick rolling mean over the buffered frames.  Always publishes —
    // downstream slow-EPMs use process_every_n_ticks to gate consumption.
    std::vector<float> mean(payload_dim_, 0.0f);
    for (auto const& f : buffer_) {
        for (int i = 0; i < payload_dim_; ++i) mean[i] += f[i];
    }
    float inv = 1.0f / float(buffer_.size());
    for (int i = 0; i < payload_dim_; ++i) mean[i] *= inv;
    last_mean_ = mean;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("keyframe_avg") : id_;
    out->sensor      = sensor_label_.empty() ? out->producer_id : sensor_label_;
    out->values.resize(payload_dim_);
    for (int i = 0; i < payload_dim_; ++i) out->values[i] = mean[i];

    bus_->publish(output_topic_, out);
    ++total_publishes_;
}

} // namespace ogma
