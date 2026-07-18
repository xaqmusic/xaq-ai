#include "ogma/modules/ScentCompass.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

constexpr float kTwoPi = 6.283185307179586f;

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("ScentCompass: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ScentCompass: param '" + key + "' must be a string");
}

} // namespace

ScentCompass::ScentCompass()  = default;
ScentCompass::~ScentCompass() = default;

std::string_view ScentCompass::type_name() const { return "ScentCompass"; }

std::vector<TopicSpec> ScentCompass::input_topics() const {
    return {
        TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> ScentCompass::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))},
    };
}

ParamSchema ScentCompass::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "Raw per-nostril scent ProprioToken (scalar concentration per nostril, "
            "ordered around a body-local ring at angles 2π·i/N).",
            ParamValue{std::string("reality.proprio.scent")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "2-D egocentric up-gradient bearing ProprioToken (RAW: direction = "
            "bearing, magnitude = gradient strength; [x=+right, y=+forward]).",
            ParamValue{std::string("percept.scent_compass")}},
        {"nostril_count", ParamMutability::ConstructionOnly,
            "Number of nostrils on the ring (angles 2π·i/N). Must match the body's "
            "scent sensor (8 for the cell).",
            ParamValue{int64_t{8}}},
        {"emit_proximity", ParamMutability::ConstructionOnly,
            "When true, append a 3rd output value = proximity_gain * mean nostril "
            "concentration (PROXIMITY). The ring sum cancels the common-mode, so "
            "[cx,cy] is pure direction; this restores the discarded magnitude so a "
            "downstream EPM/critic state can encode near-vs-far (value gradient). "
            "Default false = 2-D output (legacy).",
            ParamValue{false}},
        {"proximity_gain", ParamMutability::ConstructionOnly,
            "Scale on the appended proximity scalar (relative weight of proximity "
            "vs direction in the output vector). Only used when emit_proximity.",
            ParamValue{1.0}},
        {"normalize_direction", ParamMutability::ConstructionOnly,
            "2026-06-21 — emit [cx,cy] as a UNIT direction (cx/|g|, cy/|g|) instead of "
            "the raw gradient sum.  RATIONALE: the raw gradient is tiny+distance-scaled "
            "(~±0.25), so a downstream RBF-EPM normalised over [-1,1] sees direction in a "
            "thin central sliver and clusters by magnitude/distance, NOT angle → "
            "direction-BLIND states (the bug can't tell food-left from food-right) and a "
            "stalled GNG (errors too small to insert nodes).  Normalising spreads the "
            "direction over the full [-1,1] range → angle-selective nodes + GNG growth.  "
            "Gated by min_signal (below = no confidence → emit [0,0], a clean 'no-signal' "
            "state; without the gate, normalising amplifies near-zero noise into spurious "
            "directions).  Proximity (when emit_proximity) still carries near/far.  "
            "Default false = raw output (bit-identical for existing consumers).",
            ParamValue{false}},
        {"min_signal", ParamMutability::ConstructionOnly,
            "Gradient-magnitude floor for normalize_direction: |[cx,cy]| below this = no "
            "directional confidence → emit [0,0].  Only used when normalize_direction.",
            ParamValue{0.0}},
        {"lesion_after_ticks", ParamMutability::HotMutable,
            "≥0 → from this many ticks on, emit [0,0] (DROPOUT: knock scent out mid-run "
            "for the sensor-fusion perturbation→recovery demo). <0 = never.",
            ParamValue{int64_t{-1}}},
        {"force_lesion", ParamMutability::HotMutable,
            "Immediate lesion — emit [0,0] every tick (the UI 'knock out scent' toggle).",
            ParamValue{false}},
    };
}

ParamMap ScentCompass::current_params() const {
    ParamMap m;
    m["input_topic"]    = ParamValue{input_topic_};
    m["output_topic"]   = ParamValue{output_topic_};
    m["nostril_count"]  = ParamValue{int64_t(nostril_count_)};
    m["emit_proximity"] = ParamValue{emit_proximity_};
    m["proximity_gain"] = ParamValue{double(proximity_gain_)};
    m["normalize_direction"] = ParamValue{normalize_direction_};
    m["min_signal"]     = ParamValue{double(min_signal_)};
    m["lesion_after_ticks"] = ParamValue{int64_t(lesion_after_ticks_)};
    m["force_lesion"]   = ParamValue{force_lesion_};
    return m;
}

void ScentCompass::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ScentCompass requires a non-null Bus");

    apply_param(params, "input_topic",
        [&](auto const& v){ input_topic_ = get_string(v, "input_topic"); });
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "nostril_count",
        [&](auto const& v){ nostril_count_ = int(get_int(v, "nostril_count")); });
    apply_param(params, "emit_proximity",
        [&](auto const& v){ if (auto p = std::get_if<bool>(&v)) emit_proximity_ = *p; });
    apply_param(params, "proximity_gain",
        [&](auto const& v){ if (auto p = std::get_if<double>(&v)) proximity_gain_ = float(*p); });
    apply_param(params, "normalize_direction",
        [&](auto const& v){ if (auto p = std::get_if<bool>(&v)) normalize_direction_ = *p; });
    apply_param(params, "min_signal",
        [&](auto const& v){ if (auto p = std::get_if<double>(&v)) min_signal_ = float(*p); });
    apply_param(params, "lesion_after_ticks",
        [&](auto const& v){ lesion_after_ticks_ = int(get_int(v, "lesion_after_ticks")); });
    apply_param(params, "force_lesion",
        [&](auto const& v){ if (auto p = std::get_if<bool>(&v)) force_lesion_ = *p; });
    if (nostril_count_ < 1) nostril_count_ = 1;

    if (!input_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_scent(p); }));
    }
}

void ScentCompass::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "lesion_after_ticks") {
        lesion_after_ticks_ = int(get_int(value, k));
    } else if (k == "force_lesion") {
        if (auto p = std::get_if<bool>(&value)) force_lesion_ = *p;
        else throw std::invalid_argument("ScentCompass: 'force_lesion' must be bool");
    } else {
        throw std::invalid_argument("ScentCompass: param '" + k +
                                    "' is construction-only / unknown");
    }
}

void ScentCompass::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    scent_.assign(pt->values.begin(), pt->values.end());
}

void ScentCompass::tick(uint64_t tick_id) {
    // DROPOUT (sensor-fusion demo): when lesioned, publish the no-signal bearing
    // [0,0] (+ zero proximity) so the fused agent must ride on vision. force_lesion
    // = immediate (UI toggle); lesion_after_ticks = reproducible headless onset.
    lesioned_ = force_lesion_ ||
                (lesion_after_ticks_ >= 0 && int64_t(tick_count_) >= int64_t(lesion_after_ticks_));
    ++tick_count_;
    if (lesioned_) {
        cx_ = 0.0f; cy_ = 0.0f; mag_ = 0.0f; prox_ = 0.0f;
        auto out = std::make_shared<ProprioToken>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("scent_compass") : id_;
        out->sensor      = "scent_compass";
        out->values.resize(emit_proximity_ ? 3 : 2);
        out->values[0] = 0.0f;
        out->values[1] = 0.0f;
        if (emit_proximity_) out->values[2] = 0.0f;
        bus_->publish(output_topic_, out);
        return;
    }

    // Scent-weighted vector sum of the nostril directions → up-gradient bearing.
    // Ring direction i is (cos θ_i, sin θ_i) in body-local XZ with θ_i = 2π·i/N;
    // forward is −Z, so the forward component negates the Z weight.
    float gx = 0.0f;   // +X (right)
    float gz = 0.0f;   // +Z
    int n = std::min<int>(nostril_count_, int(scent_.size()));
    for (int i = 0; i < n; ++i) {
        float ang = kTwoPi * float(i) / float(nostril_count_);
        gx += std::cos(ang) * scent_[i];
        gz += std::sin(ang) * scent_[i];
    }
    cx_ = gx;          // +right
    cy_ = -gz;         // +forward (cell forward = −Z)
    mag_ = std::sqrt(cx_ * cx_ + cy_ * cy_);   // gradient strength (confidence)
    // 2026-06-21 — confidence-gated UNIT-direction output.  The raw gradient is tiny
    // (~±0.25) and distance-scaled, so a downstream RBF-EPM normalised over [-1,1]
    // clusters by magnitude (distance), not angle → direction-blind states + a
    // stalled GNG.  Normalising spreads the bearing over the full [-1,1] range so the
    // EPM forms angle-selective nodes (and the cy value signal becomes a bounded
    // facing measure ∈[-1,1] instead of ±0.1).  Below min_signal → no confidence →
    // [0,0] (a distinct 'no-signal' node; avoids amplifying near-zero noise).
    if (normalize_direction_) {
        if (mag_ > min_signal_ && mag_ > 1e-9f) { cx_ /= mag_; cy_ /= mag_; }
        else                                    { cx_ = 0.0f; cy_ = 0.0f; }
    }
    // PROXIMITY = mean nostril concentration (the common-mode the ring sum above
    // cancels out).  This is "how close / how much food", as opposed to [cx,cy]
    // which is "which way".  Appended as values[2] when emit_proximity.
    float mean_c = 0.0f;
    for (int i = 0; i < n; ++i) mean_c += scent_[i];
    if (n > 0) mean_c /= float(n);
    prox_ = proximity_gain_ * mean_c;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("scent_compass") : id_;
    out->sensor      = "scent_compass";
    out->values.resize(emit_proximity_ ? 3 : 2);
    out->values[0] = cx_;
    out->values[1] = cy_;
    if (emit_proximity_) out->values[2] = prox_;
    bus_->publish(output_topic_, out);
}

nlohmann::json ScentCompass::snapshot_state() const {
    return nlohmann::json{{"version", 1}};   // stateless (last scent refreshed each tick)
}

// Live viz (xaq_inspector ScentCompass widget): the egocentric gradient bearing
// (cx=+right, cy=+forward), its magnitude (= gradient strength / confidence),
// the discarded common-mode (proximity), and the raw nostril ring so the widget
// can draw a compass needle + per-nostril bars.
nlohmann::json ScentCompass::diag_snapshot() const {
    float mag = std::sqrt(cx_ * cx_ + cy_ * cy_);
    return nlohmann::json{
        {"cx", cx_},
        {"cy", cy_},
        {"mag", mag},
        {"prox", prox_},
        {"nostril_count", nostril_count_},
        {"scent", scent_},
    };
}

void ScentCompass::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ScentCompass::restore_state: unknown version " +
                                 std::to_string(version));
    }
}

} // namespace ogma
