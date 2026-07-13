// =============================================================================
// BearingEstimator.cpp  --  EPM-style inferred bearing (distilled from the compass)
// =============================================================================
#include "ogma/modules/BearingEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("BearingEstimator param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("BearingEstimator param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BearingEstimator param '" + key + "' must be string");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("BearingEstimator param '" + key + "' must be bool");
}
}

BearingEstimator::BearingEstimator()  = default;
BearingEstimator::~BearingEstimator() = default;

std::string_view BearingEstimator::type_name() const { return "BearingEstimator"; }

std::vector<TopicSpec> BearingEstimator::input_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{ring_topic_, std::type_index(typeid(ProprioToken))} };
    if (!teacher_topic_.empty())
        v.push_back(TopicSpec{teacher_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

std::vector<TopicSpec> BearingEstimator::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema BearingEstimator::params_schema() const {
    return {
        {"ring_topic", ParamMutability::ConstructionOnly,
            "Raw nostril-ring ProprioToken (e.g. reality.proprio.scent).",
            ParamValue{std::string("reality.proprio.scent")}},
        {"teacher_topic", ParamMutability::ConstructionOnly,
            "Analytic bearing [cx,cy,(prox)] used as the distillation TEACHER "
            "(e.g. percept.scent_compass). Empty = no teacher (hard ablation).",
            ParamValue{std::string("percept.scent_compass")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Inferred bearing [cx,cy,prox] (→ MotivationGate / HeadingController).",
            ParamValue{std::string("percept.bearing_inferred")}},
        {"max_prototypes", ParamMutability::ConstructionOnly,
            "Cap on learned ring prototypes.", ParamValue{int64_t{24}}},
        {"novelty_thresh", ParamMutability::HotMutable,
            "L2 distance above which a novel ring grows a new prototype.", ParamValue{0.05}},
        {"proto_lr", ParamMutability::HotMutable,
            "Vector-quantization update rate (winner prototype → ring).", ParamValue{0.05}},
        {"bearing_lr", ParamMutability::HotMutable,
            "EMA rate for the per-prototype bearing readout toward the teacher.", ParamValue{0.05}},
        {"lesion_after_ticks", ParamMutability::HotMutable,
            "≥0 → freeze readouts + stop reading the teacher after N ticks (the de-scaffold: "
            "forage on the learned percept ALONE). <0 = keep learning.", ParamValue{int64_t{-1}}},
        {"force_lesion", ParamMutability::ConstructionOnly,
            "true → start with no teacher (hard ablation: can it infer with no compass ever).",
            ParamValue{false}},
    };
}

ParamMap BearingEstimator::current_params() const {
    ParamMap m;
    m["ring_topic"]         = ParamValue{ring_topic_};
    m["teacher_topic"]      = ParamValue{teacher_topic_};
    m["output_topic"]       = ParamValue{output_topic_};
    m["max_prototypes"]     = ParamValue{int64_t(max_prototypes_)};
    m["novelty_thresh"]     = ParamValue{double(novelty_thresh_)};
    m["proto_lr"]           = ParamValue{double(proto_lr_)};
    m["bearing_lr"]         = ParamValue{double(bearing_lr_)};
    m["lesion_after_ticks"] = ParamValue{int64_t(lesion_after_ticks_)};
    m["force_lesion"]       = ParamValue{force_lesion_};
    return m;
}

void BearingEstimator::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BearingEstimator requires a non-null Bus");

    apply_param(params, "ring_topic",      [&](auto const& v){ ring_topic_    = get_string(v, "ring_topic"); });
    apply_param(params, "teacher_topic",   [&](auto const& v){ teacher_topic_ = get_string(v, "teacher_topic"); });
    apply_param(params, "output_topic",    [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "max_prototypes",  [&](auto const& v){ max_prototypes_ = std::max(1, int(get_int(v, "max_prototypes"))); });
    apply_param(params, "novelty_thresh",  [&](auto const& v){ novelty_thresh_ = float(get_double(v, "novelty_thresh")); });
    apply_param(params, "proto_lr",        [&](auto const& v){ proto_lr_       = float(get_double(v, "proto_lr")); });
    apply_param(params, "bearing_lr",      [&](auto const& v){ bearing_lr_     = float(get_double(v, "bearing_lr")); });
    apply_param(params, "lesion_after_ticks", [&](auto const& v){ lesion_after_ticks_ = int(get_int(v, "lesion_after_ticks")); });
    apply_param(params, "force_lesion",    [&](auto const& v){ force_lesion_   = get_bool(v, "force_lesion"); });
    lesioned_ = force_lesion_;

    sub_ids_.push_back(bus_->subscribe(ring_topic_, SubscriptionKind::Direct,
        [this](std::string_view /*topic*/, MessagePtr p){ handle_ring(p); }));
    if (!teacher_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(teacher_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_teacher(p); }));
}

void BearingEstimator::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "novelty_thresh")     novelty_thresh_ = float(get_double(value, k));
    else if (k == "proto_lr")           proto_lr_       = float(get_double(value, k));
    else if (k == "bearing_lr")         bearing_lr_     = float(get_double(value, k));
    else if (k == "lesion_after_ticks") lesion_after_ticks_ = int(get_int(value, k));
    else throw std::invalid_argument("BearingEstimator: param '" + k + "' is construction-only / unknown");
}

void BearingEstimator::handle_ring(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    ring_.assign(n, 0.0f);
    for (int i = 0; i < n; ++i) ring_[i] = float(pt->values[i]);
    have_ring_ = (n > 0);
}

void BearingEstimator::handle_teacher(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    if (n > 0) teach_cx_ = float(pt->values[0]);
    if (n > 1) teach_cy_ = float(pt->values[1]);
    if (n > 2) teach_prox_ = float(pt->values[2]);
    have_teacher_ = (n >= 2);
}

void BearingEstimator::tick(uint64_t tick_id) {
    if (lesion_after_ticks_ >= 0 && tick_count_ >= lesion_after_ticks_) lesioned_ = true;
    ++tick_count_;
    if (!have_ring_) return;

    // --- nearest prototype (online vector quantization of the raw ring) ---------
    int   winner = -1;
    float best   = std::numeric_limits<float>::max();
    for (size_t p = 0; p < protos_.size(); ++p) {
        float d = 0.0f;
        auto const& c = protos_[p];
        size_t m = std::min(c.size(), ring_.size());
        for (size_t i = 0; i < m; ++i) { float e = c[i] - ring_[i]; d += e * e; }
        d = std::sqrt(d);
        if (d < best) { best = d; winner = int(p); }
    }
    // Grow a prototype for a novel ring. The VQ (clustering) is SELF-SUPERVISED, so it
    // keeps adapting/growing even when lesioned (teacher-free) → the percept stays
    // covered wherever the bug drifts. A new prototype's bearing is DISTILLED from the
    // teacher while present, else INHERITED from the nearest existing prototype (so a
    // post-lesion prototype is approximately right, not blank).
    if ((winner < 0 || best > novelty_thresh_) && int(protos_.size()) < max_prototypes_) {
        float init_cx = (!lesioned_ && have_teacher_) ? teach_cx_ : (winner >= 0 ? proto_cx_[size_t(winner)] : 0.0f);
        float init_cy = (!lesioned_ && have_teacher_) ? teach_cy_ : (winner >= 0 ? proto_cy_[size_t(winner)] : 0.0f);
        protos_.push_back(ring_);
        proto_cx_.push_back(init_cx);
        proto_cy_.push_back(init_cy);
        winner = int(protos_.size()) - 1;
        best = 0.0f;
    }
    last_winner_ = winner;
    last_tle_    = (winner >= 0) ? best : 0.0f;

    if (winner >= 0) {
        // VQ: always adapt the clustering toward the observed ring (self-supervised,
        // no teacher) — this is what keeps the percept covered post-lesion.
        auto& c = protos_[size_t(winner)];
        size_t m = std::min(c.size(), ring_.size());
        for (size_t i = 0; i < m; ++i) c[i] += proto_lr_ * (ring_[i] - c[i]);
        // Distill the bearing readout ONLY while the teacher is present (pre-lesion);
        // post-lesion the readouts are frozen (teacher-free).
        if (!lesioned_ && have_teacher_) {
            proto_cx_[size_t(winner)] += bearing_lr_ * (teach_cx_ - proto_cx_[size_t(winner)]);
            proto_cy_[size_t(winner)] += bearing_lr_ * (teach_cy_ - proto_cy_[size_t(winner)]);
        }
        inf_cx_ = proto_cx_[size_t(winner)];
        inf_cy_ = proto_cy_[size_t(winner)];
    } else {
        inf_cx_ = inf_cy_ = 0.0f;
    }

    // proximity proxy from the raw ring (sum), independent of the teacher (HC ignores
    // it; emitted for parity with ScentCompass).
    float ring_sum = 0.0f;
    for (float v : ring_) ring_sum += v;
    float prox = std::min(0.5f, ring_sum * 0.05f);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id;
    out->producer_id = id_.empty() ? std::string("bearing_estimator") : id_;
    out->values = Eigen::VectorXf(3);
    out->values[0] = inf_cx_;
    out->values[1] = inf_cy_;
    out->values[2] = prox;
    bus_->publish(output_topic_, out);
}

nlohmann::json BearingEstimator::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"n_proto", int(protos_.size())},
                          {"proto_cx", proto_cx_}, {"proto_cy", proto_cy_}};
}

nlohmann::json BearingEstimator::diag_snapshot() const {
    return nlohmann::json{
        {"n_proto", int(protos_.size())}, {"winner", last_winner_},
        {"tle", last_tle_}, {"cx", inf_cx_}, {"cy", inf_cy_}, {"lesioned", lesioned_},
    };
}

void BearingEstimator::restore_state(nlohmann::json const& /*s*/) {}

} // namespace ogma
