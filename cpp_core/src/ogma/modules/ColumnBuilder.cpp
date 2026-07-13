// =============================================================================
// ColumnBuilder.cpp  --  passive place-recorder (column = view-feature + pose)
// =============================================================================
#include "ogma/modules/ColumnBuilder.hpp"

#include <algorithm>
#include <cmath>
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
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("ColumnBuilder: param '" + k + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ColumnBuilder: param '" + k + "' must be a string");
}
inline float clampf(float x, float lo, float hi) { return std::max(lo, std::min(hi, x)); }

} // namespace

ColumnBuilder::ColumnBuilder()  = default;
ColumnBuilder::~ColumnBuilder() = default;

std::string_view ColumnBuilder::type_name() const { return "ColumnBuilder"; }

std::vector<TopicSpec> ColumnBuilder::input_topics() const {
    return {
        TopicSpec{vision_topic_,  std::type_index(typeid(RawImageFrame)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)),  SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,     std::type_index(typeid(ProprioToken)),  SubscriptionKind::Direct, false},
        TopicSpec{ang_topic_,     std::type_index(typeid(ProprioToken)),  SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> ColumnBuilder::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema ColumnBuilder::params_schema() const {
    return {
        {"vision_topic", ParamMutability::ConstructionOnly,
            "FPV colour frame (RawImageFrame) — appearance source for the strip means.",
            ParamValue{std::string("host.video.color")}},
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Heading ProprioToken = [sin(heading), cos(heading)]; copied straight into the column tail.",
            ParamValue{std::string("reality.proprio.heading_vec")}},
        {"vel_topic", ParamMutability::ConstructionOnly,
            "Ego velocity ProprioToken; values[0] = forward velocity (normalized clamp(v/4,-1,1)).",
            ParamValue{std::string("reality.proprio.vel_ego")}},
        {"ang_topic", ParamMutability::ConstructionOnly,
            "Angular velocity ProprioToken; values[0] = yaw rate (normalized clamp(w/2,-1,1)).",
            ParamValue{std::string("reality.proprio.ang_vel")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Column place-code ProprioToken [3*n_strips+4] — INTERMITTENT (only on emit ticks).",
            ParamValue{std::string("percept.column")}},
        {"n_strips", ParamMutability::ConstructionOnly,
            "Vertical strips across the FPV width (appearance resolution). "
            "Output dim = 3*n_strips + 4.",
            ParamValue{int64_t{6}}},
        {"record_every", ParamMutability::HotMutable,
            "Emit cadence in ticks: publish the column only when (tick % record_every)==0 "
            "(intermittent passive recording).",
            ParamValue{int64_t{15}}},
    };
}

ParamMap ColumnBuilder::current_params() const {
    ParamMap m;
    m["vision_topic"]  = ParamValue{vision_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["vel_topic"]     = ParamValue{vel_topic_};
    m["ang_topic"]     = ParamValue{ang_topic_};
    m["output_topic"]  = ParamValue{output_topic_};
    m["n_strips"]      = ParamValue{int64_t(n_strips_)};
    m["record_every"]  = ParamValue{int64_t(record_every_)};
    return m;
}

void ColumnBuilder::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ColumnBuilder requires a non-null Bus");

    apply_param(params, "vision_topic",  [&](auto const& v){ vision_topic_  = get_string(v,"vision_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "ang_topic",     [&](auto const& v){ ang_topic_     = get_string(v,"ang_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "n_strips",      [&](auto const& v){ n_strips_      = int(get_int(v,"n_strips")); });
    apply_param(params, "record_every",  [&](auto const& v){ record_every_  = int(get_int(v,"record_every")); });
    if (n_strips_ < 1)     n_strips_ = 1;
    if (record_every_ < 1) record_every_ = 1;

    column_.assign(size_t(dims()), 0.0f);

    if (!vision_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vision_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_frame(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vel(p); }));
    if (!ang_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(ang_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_ang(p); }));
}

void ColumnBuilder::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "record_every") {
        record_every_ = int(get_int(value, k));
        if (record_every_ < 1) record_every_ = 1;
    } else {
        throw std::invalid_argument("ColumnBuilder: param '" + k +
                                    "' is construction-only / unknown");
    }
}

void ColumnBuilder::handle_frame(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto img = std::dynamic_pointer_cast<const RawImageFrame>(payload);
    if (!img) return;
    pixels_ = img->pixels; width_ = img->width; height_ = img->height; channels_ = img->channels;
}

void ColumnBuilder::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    if (pt->values.size() > 0) heading_sin_ = float(pt->values[0]);
    if (pt->values.size() > 1) heading_cos_ = float(pt->values[1]);
}

void ColumnBuilder::handle_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) vel_fwd_ = float(pt->values[0]);
}

void ColumnBuilder::handle_ang(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) ang_vel_ = float(pt->values[0]);
}

void ColumnBuilder::tick(uint64_t tick_id) {
    // Intermittent passive recording: only assemble + publish on emit ticks.
    recorded_last_tick_ = (tick_id % uint64_t(record_every_)) == 0;
    if (!recorded_last_tick_) return;

    const int D = dims();
    column_.assign(size_t(D), 0.0f);

    // [0 .. 3*n_strips-1] = per-strip mean RGB, normalized to [0,1].
    // Split the FPV WIDTH into n_strips equal vertical strips (left→right); per
    // strip accumulate mean R/G/B over all its pixels. Row-major H×W×C uint8:
    // pixel (row,col) byte channel c lives at ((row*W + col)*C + c).
    if (channels_ >= 3 && width_ > 0 && height_ > 0 &&
        int(pixels_.size()) >= width_ * height_ * channels_) {
        std::vector<double> sum_r(n_strips_, 0.0), sum_g(n_strips_, 0.0), sum_b(n_strips_, 0.0);
        std::vector<long>   cnt(n_strips_, 0);
        for (int row = 0; row < height_; ++row) {
            long base = long(row) * width_ * channels_;
            for (int col = 0; col < width_; ++col) {
                int strip = int(long(col) * n_strips_ / width_);
                if (strip >= n_strips_) strip = n_strips_ - 1;   // guard rounding at width-1
                long idx = base + long(col) * channels_;
                sum_r[strip] += pixels_[idx];
                sum_g[strip] += pixels_[idx + 1];
                sum_b[strip] += pixels_[idx + 2];
                ++cnt[strip];
            }
        }
        for (int s = 0; s < n_strips_; ++s) {
            if (cnt[s] > 0) {
                column_[s * 3 + 0] = float(sum_r[s] / double(cnt[s]) / 255.0);
                column_[s * 3 + 1] = float(sum_g[s] / double(cnt[s]) / 255.0);
                column_[s * 3 + 2] = float(sum_b[s] / double(cnt[s]) / 255.0);
            }
        }
    }

    // Pose tail. [3n] sin(heading), [3n+1] cos(heading) — copied straight from
    // the heading token; [3n+2] forward vel /4; [3n+3] yaw rate /2, both clamped.
    const int base = 3 * n_strips_;
    column_[base + 0] = heading_sin_;
    column_[base + 1] = heading_cos_;
    column_[base + 2] = clampf(vel_fwd_ / 4.0f, -1.0f, 1.0f);
    column_[base + 3] = clampf(ang_vel_ / 2.0f, -1.0f, 1.0f);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("column") : id_;
    out->sensor      = "column";
    out->values.resize(D);
    for (int i = 0; i < D; ++i) out->values[i] = column_[i];
    bus_->publish(output_topic_, out);
}

nlohmann::json ColumnBuilder::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"column", column_}};
}

// Live viz / HUD sanity: dims, whether we recorded this tick, n_strips, and the
// first strip's R/G/B so a widget can sanity-check the appearance encoding.
nlohmann::json ColumnBuilder::diag_snapshot() const {
    float s0r = column_.size() > 0 ? column_[0] : 0.0f;
    float s0g = column_.size() > 1 ? column_[1] : 0.0f;
    float s0b = column_.size() > 2 ? column_[2] : 0.0f;
    return nlohmann::json{
        {"dims", dims()},
        {"recorded", recorded_last_tick_},
        {"n_strips", n_strips_},
        {"s0r", s0r},
        {"s0g", s0g},
        {"s0b", s0b},
        {"column", column_},
    };
}

void ColumnBuilder::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    if (s.value("version", 0) != 1) return;
    if (s.contains("column")) column_ = s["column"].get<std::vector<float>>();
}

} // namespace ogma
