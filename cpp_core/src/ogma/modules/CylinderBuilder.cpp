// =============================================================================
// CylinderBuilder.cpp  --  heading-indexed panorama place-code (Pathway C2)
// =============================================================================
#include "ogma/modules/CylinderBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("CylinderBuilder: param '" + k + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("CylinderBuilder: param '" + k + "' must be a string");
}
} // namespace

CylinderBuilder::CylinderBuilder()  = default;
CylinderBuilder::~CylinderBuilder() = default;

std::string_view CylinderBuilder::type_name() const { return "CylinderBuilder"; }

std::vector<TopicSpec> CylinderBuilder::input_topics() const {
    return {
        TopicSpec{frame_topic_,   std::type_index(typeid(RawImageFrame)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)),  SubscriptionKind::Direct, false},
        TopicSpec{active_topic_,  std::type_index(typeid(ProprioToken)),  SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> CylinderBuilder::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema CylinderBuilder::params_schema() const {
    return {
        {"frame_topic", ParamMutability::ConstructionOnly,
            "FPV colour frame (RawImageFrame) — appearance source.",
            ParamValue{std::string("host.video.color")}},
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Absolute heading ProprioToken (rad) — the panorama index (view-invariant).",
            ParamValue{std::string("reality.proprio.heading")}},
        {"active_topic", ParamMutability::ConstructionOnly,
            "Saccade-active scalar (1 while pivoting) — accumulate only during a sweep.",
            ParamValue{std::string("saccade.active")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Panorama place-code ProprioToken [n_bins*3], 0..1 (→ the place-EPM, C3).",
            ParamValue{std::string("percept.cylinder")}},
        {"n_bins", ParamMutability::ConstructionOnly,
            "Heading bins around the circle (panorama resolution).", ParamValue{int64_t{8}}},
        {"passive_emit_every", ParamMutability::ConstructionOnly,
            "PASSIVE mode: >0 = accumulate every tick (no saccade) and emit a rolling windowed "
            "panorama every N ticks. 0 = saccade-gated (legacy).", ParamValue{int64_t{0}}},
    };
}

ParamMap CylinderBuilder::current_params() const {
    ParamMap m;
    m["frame_topic"]   = ParamValue{frame_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["active_topic"]  = ParamValue{active_topic_};
    m["output_topic"]  = ParamValue{output_topic_};
    m["n_bins"]        = ParamValue{int64_t(n_bins_)};
    m["passive_emit_every"] = ParamValue{int64_t(passive_emit_every_)};
    return m;
}

void CylinderBuilder::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("CylinderBuilder requires a non-null Bus");

    apply_param(params, "frame_topic",   [&](auto const& v){ frame_topic_   = get_string(v,"frame_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "active_topic",  [&](auto const& v){ active_topic_  = get_string(v,"active_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "n_bins",        [&](auto const& v){ n_bins_        = int(get_int(v,"n_bins")); });
    apply_param(params, "passive_emit_every", [&](auto const& v){ passive_emit_every_ = int(get_int(v,"passive_emit_every")); });
    if (n_bins_ < 1) n_bins_ = 1;
    if (passive_emit_every_ < 0) passive_emit_every_ = 0;

    sum_r_.assign(n_bins_, 0.0); sum_g_.assign(n_bins_, 0.0); sum_b_.assign(n_bins_, 0.0);
    count_.assign(n_bins_, 0);
    panorama_.assign(size_t(n_bins_) * 3, 0.0f);

    if (!frame_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(frame_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_frame(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!active_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(active_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_active(p); }));
}

void CylinderBuilder::on_param_change(std::string_view key, ParamValue const& /*value*/) {
    throw std::invalid_argument("CylinderBuilder: param '" + std::string(key) +
                                "' is construction-only / unknown");
}

void CylinderBuilder::handle_frame(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto img = std::dynamic_pointer_cast<const RawImageFrame>(payload);
    if (!img) return;
    pixels_ = img->pixels; width_ = img->width; height_ = img->height; channels_ = img->channels;
}

void CylinderBuilder::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) heading_ = float(pt->values[0]);
}

void CylinderBuilder::handle_active(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) active_ = float(pt->values[0]);
}

int CylinderBuilder::bin_of(float heading_rad) const {
    // Wrap to [0,2π) then bin. Absolute heading → view-invariant index.
    float h = std::fmod(heading_rad, kTwoPi);
    if (h < 0.0f) h += kTwoPi;
    int b = int(h / kTwoPi * float(n_bins_));
    if (b < 0) b = 0; if (b >= n_bins_) b = n_bins_ - 1;
    return b;
}

void CylinderBuilder::finalize_panorama() {
    last_bins_filled_ = 0;
    for (int b = 0; b < n_bins_; ++b) {
        if (count_[b] > 0) {
            panorama_[b * 3 + 0] = float(sum_r_[b] / count_[b] / 255.0);
            panorama_[b * 3 + 1] = float(sum_g_[b] / count_[b] / 255.0);
            panorama_[b * 3 + 2] = float(sum_b_[b] / count_[b] / 255.0);
            ++last_bins_filled_;
        }
        // unfilled bins keep the previous cylinder's value (carry-over)
        sum_r_[b] = sum_g_[b] = sum_b_[b] = 0.0; count_[b] = 0;
    }
    have_panorama_ = true;
    ++cylinders_built_;
}

void CylinderBuilder::tick(uint64_t tick_id) {
    bool const passive = passive_emit_every_ > 0;
    // Accumulate the current FPV mean colour into the current heading bin — passively
    // every tick (from natural turning), or (legacy) only while the saccade sweeps.
    if ((passive || active_ > 0.5f) && channels_ >= 3 && width_ > 0 && height_ > 0 &&
        int(pixels_.size()) >= width_ * height_ * channels_) {
        long n = long(width_) * height_;
        double r = 0, g = 0, b = 0;
        for (long k = 0; k < n; ++k) {
            long idx = k * channels_;
            r += pixels_[idx]; g += pixels_[idx + 1]; b += pixels_[idx + 2];
        }
        int bin = bin_of(heading_);
        sum_r_[bin] += r / double(n);
        sum_g_[bin] += g / double(n);
        sum_b_[bin] += b / double(n);
        count_[bin] += 1;
    }

    // PASSIVE: emit a rolling windowed panorama every N ticks (finalise+reset).
    // LEGACY: finalise on saccade end (active 1→0).
    if (passive) {
        if (tick_id > 0 && (tick_id % uint64_t(passive_emit_every_)) == 0) finalize_panorama();
    } else if (prev_active_ > 0.5f && active_ <= 0.5f) {
        finalize_panorama();
    }
    prev_active_ = active_;

    // Publish the last completed panorama (held), so the place-EPM (C3) sees the
    // current place estimate every tick until the next scan refreshes it.
    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("cylinder") : id_;
    out->sensor      = "cylinder";
    out->values = Eigen::VectorXf::Zero(int(panorama_.size()));
    if (have_panorama_)
        for (int i = 0; i < int(panorama_.size()); ++i) out->values[i] = panorama_[i];
    bus_->publish(output_topic_, out);
}

nlohmann::json CylinderBuilder::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"panorama", panorama_}, {"built", cylinders_built_}};
}

nlohmann::json CylinderBuilder::diag_snapshot() const {
    return nlohmann::json{
        {"n_bins", n_bins_},
        {"built", cylinders_built_},
        {"bins_filled", last_bins_filled_},
        {"panorama", panorama_},
    };
}

void CylinderBuilder::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    if (s.value("version", 0) != 1) return;
    if (s.contains("panorama")) {
        panorama_ = s["panorama"].get<std::vector<float>>();
        have_panorama_ = !panorama_.empty();
    }
}

} // namespace ogma
