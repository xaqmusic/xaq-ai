// =============================================================================
// BearingFusion.cpp  --  trust-weighted vision+scent bearing fusion
// =============================================================================
#include "ogma/modules/BearingFusion.hpp"

#include "ogma/Topics.hpp"

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

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("BearingFusion: param '" + key + "' must be integer");
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("BearingFusion: param '" + key + "' must be numeric");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BearingFusion: param '" + key + "' must be a string");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("BearingFusion: param '" + key + "' must be bool");
}

} // namespace

BearingFusion::BearingFusion()  = default;
BearingFusion::~BearingFusion() = default;

std::string_view BearingFusion::type_name() const { return "BearingFusion"; }

std::vector<TopicSpec> BearingFusion::input_topics() const {
    return {
        TopicSpec{consensus_topic_, std::type_index(typeid(ConsensusToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{scent_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{vision_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> BearingFusion::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema BearingFusion::params_schema() const {
    return {
        {"consensus_topic", ParamMutability::ConstructionOnly,
            "LateralVoter ConsensusToken topic (read trust_weights).",
            ParamValue{std::string("consensus.0")}},
        {"scent_topic", ParamMutability::ConstructionOnly,
            "Scent bearing ProprioToken [cx,cy,(prox)].",
            ParamValue{std::string("percept.scent_compass")}},
        {"vision_topic", ParamMutability::ConstructionOnly,
            "Vision bearing ProprioToken [vx,vy,(prox)].",
            ParamValue{std::string("percept.visual_bearing")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Fused bearing (→ HeadingController.input_topic).",
            ParamValue{std::string("percept.fused_bearing")}},
        {"scent_trust_key", ParamMutability::ConstructionOnly,
            "trust_weights key for scent = epm_scent's full output topic "
            "(reality.<group>.<name>). MUST match the EPM config.",
            ParamValue{std::string("reality.nav.scent")}},
        {"vision_trust_key", ParamMutability::ConstructionOnly,
            "trust_weights key for vision = epm_vision's full output topic.",
            ParamValue{std::string("reality.nav.vision")}},
        {"cx_index", ParamMutability::ConstructionOnly, "Index of +right component.",
            ParamValue{int64_t{0}}},
        {"cy_index", ParamMutability::ConstructionOnly, "Index of +forward component.",
            ParamValue{int64_t{1}}},
        {"confidence_floor", ParamMutability::HotMutable,
            "A bearing whose own magnitude is below this is down-weighted to 0 "
            "regardless of trust (instant dropout fallback).", ParamValue{1e-4}},
        {"passthrough_prox", ParamMutability::ConstructionOnly,
            "Carry a 3rd (proximity) value through (trust-weighted) when either input "
            "has one.", ParamValue{true}},
        {"renormalize", ParamMutability::ConstructionOnly,
            "Unit-normalize the fused bearing so disagreement-shortening isn't read as "
            "low confidence. Default true.", ParamValue{true}},
    };
}

ParamMap BearingFusion::current_params() const {
    ParamMap m;
    m["consensus_topic"]  = ParamValue{consensus_topic_};
    m["scent_topic"]      = ParamValue{scent_topic_};
    m["vision_topic"]     = ParamValue{vision_topic_};
    m["output_topic"]     = ParamValue{output_topic_};
    m["scent_trust_key"]  = ParamValue{scent_trust_key_};
    m["vision_trust_key"] = ParamValue{vision_trust_key_};
    m["cx_index"]         = ParamValue{int64_t(cx_index_)};
    m["cy_index"]         = ParamValue{int64_t(cy_index_)};
    m["confidence_floor"] = ParamValue{double(confidence_floor_)};
    m["passthrough_prox"] = ParamValue{passthrough_prox_};
    m["renormalize"]      = ParamValue{renormalize_};
    return m;
}

void BearingFusion::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BearingFusion requires a non-null Bus");

    apply_param(params, "consensus_topic",  [&](auto const& v){ consensus_topic_  = get_string(v, "consensus_topic"); });
    apply_param(params, "scent_topic",      [&](auto const& v){ scent_topic_      = get_string(v, "scent_topic"); });
    apply_param(params, "vision_topic",     [&](auto const& v){ vision_topic_     = get_string(v, "vision_topic"); });
    apply_param(params, "output_topic",     [&](auto const& v){ output_topic_     = get_string(v, "output_topic"); });
    apply_param(params, "scent_trust_key",  [&](auto const& v){ scent_trust_key_  = get_string(v, "scent_trust_key"); });
    apply_param(params, "vision_trust_key", [&](auto const& v){ vision_trust_key_ = get_string(v, "vision_trust_key"); });
    apply_param(params, "cx_index",         [&](auto const& v){ cx_index_         = int(get_int(v, "cx_index")); });
    apply_param(params, "cy_index",         [&](auto const& v){ cy_index_         = int(get_int(v, "cy_index")); });
    apply_param(params, "confidence_floor", [&](auto const& v){ confidence_floor_ = float(get_double(v, "confidence_floor")); });
    apply_param(params, "passthrough_prox", [&](auto const& v){ passthrough_prox_ = get_bool(v, "passthrough_prox"); });
    apply_param(params, "renormalize",      [&](auto const& v){ renormalize_      = get_bool(v, "renormalize"); });

    if (!consensus_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(consensus_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_consensus(p); }));
    if (!scent_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    if (!vision_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vision_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vision(p); }));
}

void BearingFusion::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "confidence_floor") confidence_floor_ = float(get_double(value, k));
    else throw std::invalid_argument("BearingFusion: param '" + k + "' is construction-only / unknown");
}

void BearingFusion::handle_consensus(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!ct) return;
    trust_ = ct->trust_weights;
}

void BearingFusion::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    s_cx_ = (cx_index_ < n) ? float(pt->values[cx_index_]) : 0.0f;
    s_cy_ = (cy_index_ < n) ? float(pt->values[cy_index_]) : 0.0f;
    s_has_prox_ = (n > 2);
    s_prox_ = s_has_prox_ ? float(pt->values[2]) : 0.0f;
}

void BearingFusion::handle_vision(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    v_cx_ = (cx_index_ < n) ? float(pt->values[cx_index_]) : 0.0f;
    v_cy_ = (cy_index_ < n) ? float(pt->values[cy_index_]) : 0.0f;
    v_has_prox_ = (n > 2);
    v_prox_ = v_has_prox_ ? float(pt->values[2]) : 0.0f;
}

float BearingFusion::lookup_trust(std::string const& key) const {
    auto it = trust_.find(key);
    return (it != trust_.end()) ? it->second : 0.0f;
}

void BearingFusion::tick(uint64_t tick_id) {
    float s_mag = std::sqrt(s_cx_ * s_cx_ + s_cy_ * s_cy_);
    float v_mag = std::sqrt(v_cx_ * v_cx_ + v_cy_ * v_cy_);

    float w_s = lookup_trust(scent_trust_key_);
    float w_v = lookup_trust(vision_trust_key_);
    // Confidence-floor gate: a bearing emitting ~[0,0] (occluded / lesioned) is
    // dropped regardless of its (possibly stale) trust weight.
    if (s_mag < confidence_floor_) w_s = 0.0f;
    if (v_mag < confidence_floor_) w_v = 0.0f;
    w_scent_  = w_s;
    w_vision_ = w_v;

    float den = w_s + w_v;
    float fprox = 0.0f;
    bool  has_prox = passthrough_prox_ && (s_has_prox_ || v_has_prox_);

    if (den <= 0.0f) {
        // Neither channel trusted/confident → fall back to the higher own-magnitude
        // raw bearing if any, else a clean no-signal [0,0].
        if (s_mag >= v_mag && s_mag > 0.0f)      { fx_ = s_cx_; fy_ = s_cy_; fprox = s_prox_; }
        else if (v_mag > 0.0f)                   { fx_ = v_cx_; fy_ = v_cy_; fprox = v_prox_; }
        else                                     { fx_ = 0.0f;  fy_ = 0.0f;  fprox = 0.0f; }
    } else {
        fx_ = (w_s * s_cx_ + w_v * v_cx_) / den;
        fy_ = (w_s * s_cy_ + w_v * v_cy_) / den;
        fprox = (w_s * s_prox_ + w_v * v_prox_) / den;
        if (renormalize_) {
            float fm = std::sqrt(fx_ * fx_ + fy_ * fy_);
            if (fm > 1e-9f) { fx_ /= fm; fy_ /= fm; }
        }
    }

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("bearing_fusion") : id_;
    out->sensor      = "fused_bearing";
    out->values.resize(has_prox ? 3 : 2);
    out->values[0] = fx_;
    out->values[1] = fy_;
    if (has_prox) out->values[2] = fprox;
    bus_->publish(output_topic_, out);
}

nlohmann::json BearingFusion::snapshot_state() const {
    return nlohmann::json{{"version", 1}};   // stateless (caches refreshed each tick)
}

nlohmann::json BearingFusion::diag_snapshot() const {
    return nlohmann::json{
        {"fx", fx_},
        {"fy", fy_},
        {"w_scent", w_scent_},
        {"w_vision", w_vision_},
    };
}

void BearingFusion::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("BearingFusion::restore_state: unknown version " +
                                 std::to_string(version));
    }
}

} // namespace ogma
