#include "ogma/modules/MotorFader.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

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
    throw std::invalid_argument("MotorFader param '" + key + "' must be numeric");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("MotorFader param '" + key + "' must be string");
}

} // namespace

MotorFader::MotorFader()  = default;
MotorFader::~MotorFader() = default;

std::string_view MotorFader::type_name() const { return "MotorFader"; }

std::vector<TopicSpec> MotorFader::input_topics() const {
    std::vector<TopicSpec> specs = {
        TopicSpec{brain_topic_, std::type_index(typeid(ActionOut)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{reflex_topic_, std::type_index(typeid(ActionOut)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{alpha_topic_, std::type_index(typeid(FaderState)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
    if (!entropy_topic_.empty()) {
        specs.push_back(TopicSpec{entropy_topic_, std::type_index(typeid(PolicyToken)),
                                   SubscriptionKind::Direct, /*required=*/false});
    }
    return specs;
}

std::vector<TopicSpec> MotorFader::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(ActionOut))},
    };
}

ParamSchema MotorFader::params_schema() const {
    return {
        {"brain_topic",   ParamMutability::ConstructionOnly,
            "Brain-side ActionOut input topic",
            ParamValue{std::string("action.brain")}},
        {"reflex_topic",  ParamMutability::ConstructionOnly,
            "Reflex-side ActionOut input topic",
            ParamValue{std::string("action.reflex")}},
        {"alpha_topic",   ParamMutability::ConstructionOnly,
            "FaderState input topic (FaderController publishes here; "
            "graphs without a FaderController fall back to alpha_fixed)",
            ParamValue{std::string(topics::kMotorFaderAlpha)}},
        {"output_topic",  ParamMutability::ConstructionOnly,
            "Blended ActionOut output topic",
            ParamValue{std::string(topics::kActionOut)}},
        {"alpha_fixed",   ParamMutability::HotMutable,
            "Fallback α used when no FaderState ever arrives "
            "(0 = reflex-only, 1 = brain-only, default 0)",
            ParamValue{0.0}},
        {"idle_reflex_passthrough", ParamMutability::HotMutable,
            "When true and NO reflex ActionOut arrived this tick, pass the "
            "brain through unattenuated (effective α=1) instead of blending "
            "with reflex=0.  For a silent-when-idle reflex (e.g. a contact "
            "whisker) this makes it duck the brain only WHEN it fires "
            "(bus-compressor 'silent channel = no gain reduction').  Default "
            "false = legacy blend (byte-identical).",
            ParamValue{false}},
        {"noise_amplitude", ParamMutability::HotMutable,
            "Phase 6.6.P inverted-babbler motor noise: standard "
            "deviation of Gaussian noise added to the blended action, "
            "scaled by (1 - clamp(surprise_scalar, 0, 1)).  Low surprise "
            "→ loud noise (escapes dark-room attractor); high surprise "
            "→ quiet (decisive action).  Default 0 = bit-identical to "
            "pre-6.6.P.",
            ParamValue{0.0}},
        {"noise_seed",    ParamMutability::ConstructionOnly,
            "Seed for the Gaussian RNG used by noise_amplitude.  "
            "0 = deterministic across runs.",
            ParamValue{int64_t(0)}},
        {"entropy_topic", ParamMutability::ConstructionOnly,
            "v6.0.d inverted-babbler entropy input topic (typically "
            "Premotor's 'policy.intent').  When empty, no subscription "
            "is made and the entropy multiplier stays at 1.  When set, "
            "the most recent PolicyToken.entropy / ln(N) is cached and "
            "amplifies noise gain when entropy_gain > 0.",
            ParamValue{std::string("")}},
        {"entropy_gain",  ParamMutability::HotMutable,
            "v6.0.d Playful Machine #1 — boost noise gain by "
            "entropy_gain * (1 − norm_entropy) when the policy is "
            "committed.  Effective gain = noise_amplitude * (1 − "
            "surprise) * (1 + entropy_gain * (1 − norm_entropy)).  "
            "Default 0 = bit-identical to 6.6.P (surprise-only gating).",
            ParamValue{0.0}},
    };
}

ParamMap MotorFader::current_params() const {
    ParamMap m;
    m["brain_topic"]  = ParamValue{brain_topic_};
    m["reflex_topic"] = ParamValue{reflex_topic_};
    m["alpha_topic"]  = ParamValue{alpha_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["alpha_fixed"]      = ParamValue{double(alpha_fixed_)};
    m["noise_amplitude"]  = ParamValue{double(noise_amplitude_)};
    m["noise_seed"]       = ParamValue{int64_t(noise_seed_)};
    m["entropy_topic"]    = ParamValue{entropy_topic_};
    m["entropy_gain"]     = ParamValue{double(entropy_gain_)};
    m["idle_reflex_passthrough"] = ParamValue{idle_reflex_passthrough_};
    return m;
}

void MotorFader::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotorFader requires a non-null Bus");

    apply_param(params, "brain_topic",  [&](auto const& v){ brain_topic_  = get_string(v, "brain_topic"); });
    apply_param(params, "reflex_topic", [&](auto const& v){ reflex_topic_ = get_string(v, "reflex_topic"); });
    apply_param(params, "alpha_topic",  [&](auto const& v){ alpha_topic_  = get_string(v, "alpha_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "alpha_fixed",     [&](auto const& v){ alpha_fixed_     = float(get_double(v, "alpha_fixed")); });
    apply_param(params, "noise_amplitude", [&](auto const& v){ noise_amplitude_ = float(get_double(v, "noise_amplitude")); });
    apply_param(params, "noise_seed",      [&](auto const& v){
        if (auto p = std::get_if<int64_t>(&v))      noise_seed_ = uint64_t(*p);
        else if (auto p = std::get_if<double>(&v))  noise_seed_ = uint64_t(*p);
        else throw std::invalid_argument("MotorFader param 'noise_seed' must be numeric");
    });
    apply_param(params, "entropy_topic",   [&](auto const& v){ entropy_topic_ = get_string(v, "entropy_topic"); });
    apply_param(params, "entropy_gain",    [&](auto const& v){ entropy_gain_  = float(get_double(v, "entropy_gain")); });
    apply_param(params, "idle_reflex_passthrough", [&](auto const& v){
        if (auto p = std::get_if<bool>(&v)) idle_reflex_passthrough_ = *p;
        else throw std::invalid_argument("MotorFader param 'idle_reflex_passthrough' must be bool");
    });

    if (alpha_fixed_ < 0.0f || alpha_fixed_ > 1.0f)
        throw std::invalid_argument("MotorFader: alpha_fixed must be in [0, 1]");
    if (noise_amplitude_ < 0.0f)
        throw std::invalid_argument("MotorFader: noise_amplitude must be >= 0");
    if (entropy_gain_ < 0.0f)
        throw std::invalid_argument("MotorFader: entropy_gain must be >= 0");

    alpha_ = alpha_fixed_;     // fallback until first FaderState arrives
    rng_.seed(noise_seed_);

    sub_ids_.push_back(bus_->subscribe(brain_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_brain(p); }));
    sub_ids_.push_back(bus_->subscribe(reflex_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_reflex(p); }));
    sub_ids_.push_back(bus_->subscribe(alpha_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_alpha(p); }));
    if (!entropy_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(entropy_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ this->handle_policy(p); }));
    }
}

void MotorFader::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "alpha_fixed")  {
        alpha_fixed_ = float(get_double(value, k));
        if (alpha_fixed_ < 0.0f || alpha_fixed_ > 1.0f)
            throw std::invalid_argument("MotorFader: alpha_fixed must be in [0, 1]");
        // Live-update fallback only when no FaderController has taken over.
        if (!alpha_from_bus_) alpha_ = alpha_fixed_;
    }
    else if (k == "noise_amplitude") {
        float v = float(get_double(value, k));
        if (v < 0.0f) throw std::invalid_argument("MotorFader: noise_amplitude must be >= 0");
        noise_amplitude_ = v;
    }
    else if (k == "entropy_gain") {
        float v = float(get_double(value, k));
        if (v < 0.0f) throw std::invalid_argument("MotorFader: entropy_gain must be >= 0");
        entropy_gain_ = v;
    }
    else if (k == "idle_reflex_passthrough") {
        if (auto p = std::get_if<bool>(&value)) idle_reflex_passthrough_ = *p;
        else throw std::invalid_argument("MotorFader param 'idle_reflex_passthrough' must be bool");
    }
    else if (k == "brain_topic" || k == "reflex_topic"
          || k == "alpha_topic" || k == "output_topic"
          || k == "noise_seed"  || k == "entropy_topic")
        throw std::invalid_argument("MotorFader param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("MotorFader: unknown param '" + k + "'");
}

void MotorFader::handle_brain(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    pending_brain_ = std::dynamic_pointer_cast<const ActionOut>(payload);
}

void MotorFader::handle_reflex(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    pending_reflex_ = std::dynamic_pointer_cast<const ActionOut>(payload);
}

void MotorFader::handle_alpha(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto fs = std::dynamic_pointer_cast<const FaderState>(payload);
    if (!fs) return;
    alpha_          = std::clamp(fs->alpha, 0.0f, 1.0f);
    last_surprise_  = std::clamp(fs->surprise_scalar, 0.0f, 1.0f);
    alpha_from_bus_ = true;
}

void MotorFader::handle_policy(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pol = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!pol) return;
    ++policy_msgs_received_;
    int N = int(pol->intent_distribution.size());
    if (N < 2) return;   // entropy undefined for single-intent policies
    float max_H = std::log(float(N));
    if (max_H <= 0.0f) return;
    last_entropy_normalized_ = std::clamp(pol->entropy / max_H, 0.0f, 1.0f);
}

void MotorFader::tick(uint64_t tick_id) {
    bool brain_seen  = bool(pending_brain_);
    bool reflex_seen = bool(pending_reflex_);

    float brain_accel  = brain_seen  ? pending_brain_ ->accel : 0.0f;
    float reflex_accel = reflex_seen ? pending_reflex_->accel : 0.0f;
    // Contact-subsumption: when the reflex is idle (silent-when-idle reflexes
    // like the contact whisker don't publish), pass the brain through
    // unattenuated rather than blending it with reflex=0.  The reflex ducks
    // the brain only WHEN it actually fires.
    float eff_alpha    = (idle_reflex_passthrough_ && !reflex_seen) ? 1.0f : alpha_;
    float blended      = eff_alpha * brain_accel + (1.0f - eff_alpha) * reflex_accel;

    // Phase 6.6.P inverted-babbler noise, extended in v6.0.d by Playful
    // Machine principle #1: when the policy is COMMITTED (low entropy)
    // AND the world is predictable (low surprise), amplify the noise
    // gain to break the attractor.  Multiplicative coupling:
    //   gain = noise_amplitude * (1 - surprise)             // 6.6.P term
    //                          * (1 + entropy_gain * (1 - norm_entropy))
    // norm_entropy ∈ [0, 1] (entropy / ln N); 1 = uniform softmax
    // (no info → no amplification), 0 = fully committed (max boost).
    // entropy_gain=0 OR no entropy_topic subscribed → bit-identical
    // to pre-v6.0.d behaviour.
    float noise_sample = 0.0f;
    if (noise_amplitude_ > 0.0f) {
        std::normal_distribution<float> nd(0.0f, 1.0f);
        float gain = noise_amplitude_ * (1.0f - last_surprise_);
        if (entropy_gain_ > 0.0f) {
            gain *= (1.0f + entropy_gain_ * (1.0f - last_entropy_normalized_));
        }
        noise_sample = gain * nd(rng_);
        blended += noise_sample;
    }
    last_noise_ = noise_sample;

    auto out = std::make_shared<ActionOut>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("motor_fader") : id_;
    out->accel       = blended;
    out->source      = "fader";
    if (brain_seen) {
        out->chunk_id              = pending_brain_->chunk_id;
        out->chunk_remaining_ticks = pending_brain_->chunk_remaining_ticks;
        out->probe                 = pending_brain_->probe;
    }
    bus_->publish(output_topic_, out);
    ++publish_count_;

    last_brain_accel_  = brain_accel;
    last_reflex_accel_ = reflex_accel;
    last_output_accel_ = blended;
    last_brain_seen_   = brain_seen;
    last_reflex_seen_  = reflex_seen;

    // v5.4 clash metric.  Measures intent erased by the blend when brain
    // and reflex disagree in sign.  Computed on the contributions BEFORE
    // noise injection — clash is about the policy/reflex disagreement,
    // not the noise.  Same-sign contributions add cleanly (clash=0);
    // opposite-sign contributions cancel (clash = 2 · min of magnitudes).
    if (brain_seen && reflex_seen) {
        float bc = alpha_ * brain_accel;
        float rc = (1.0f - alpha_) * reflex_accel;
        float pre_noise_blend = bc + rc;
        last_clash_ = std::max(0.0f,
            std::abs(bc) + std::abs(rc) - std::abs(pre_noise_blend));
    } else {
        // Only one side publishing → no clash possible.
        last_clash_ = 0.0f;
    }
    clash_ema_ = (1.0f - kClashEmaAlpha_) * clash_ema_
                 + kClashEmaAlpha_       * last_clash_;

    pending_brain_.reset();
    pending_reflex_.reset();
}

// ---------------------------------------------------------------------------
// Snapshot / restore (UI-dev W3.2 Tier A)
// ---------------------------------------------------------------------------
//
// pending_brain_ / pending_reflex_ are intra-tick caches cleared at end of
// tick(); they are guaranteed empty at any inter-tick snapshot point and
// are deliberately omitted.

nlohmann::json MotorFader::snapshot_state() const {
    std::ostringstream os; os << rng_;
    return nlohmann::json{
        {"version",            1},
        {"alpha",              alpha_},
        {"alpha_from_bus",     alpha_from_bus_},
        {"last_brain_accel",   last_brain_accel_},
        {"last_reflex_accel",  last_reflex_accel_},
        {"last_output_accel",  last_output_accel_},
        {"last_surprise",      last_surprise_},
        {"last_noise",         last_noise_},
        {"last_brain_seen",    last_brain_seen_},
        {"last_reflex_seen",   last_reflex_seen_},
        {"publish_count",      publish_count_},
        {"rng",                os.str()},
        {"last_entropy_normalized", last_entropy_normalized_},
        {"policy_msgs_received",    policy_msgs_received_},
    };
}

void MotorFader::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("MotorFader::restore_state: unknown version " +
                                 std::to_string(version));
    }
    alpha_              = s.value("alpha",              alpha_);
    alpha_from_bus_     = s.value("alpha_from_bus",     alpha_from_bus_);
    last_brain_accel_   = s.value("last_brain_accel",   last_brain_accel_);
    last_reflex_accel_  = s.value("last_reflex_accel",  last_reflex_accel_);
    last_output_accel_  = s.value("last_output_accel",  last_output_accel_);
    last_surprise_      = s.value("last_surprise",      last_surprise_);
    last_noise_         = s.value("last_noise",         last_noise_);
    last_brain_seen_    = s.value("last_brain_seen",    last_brain_seen_);
    last_reflex_seen_   = s.value("last_reflex_seen",   last_reflex_seen_);
    publish_count_      = s.value("publish_count",      publish_count_);
    last_entropy_normalized_ = s.value("last_entropy_normalized", last_entropy_normalized_);
    policy_msgs_received_    = s.value("policy_msgs_received",    policy_msgs_received_);
    if (s.contains("rng") && s["rng"].is_string()) {
        std::istringstream is(s["rng"].get<std::string>()); is >> rng_;
    }
}

} // namespace ogma
