#include "ogma/modules/SynergyTimer.hpp"

#include <algorithm>
#include <cmath>
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

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("SynergyTimer param '" + key + "' must be string");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("SynergyTimer param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("SynergyTimer param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("SynergyTimer param '" + key + "' must be boolean");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("SynergyTimer param '" + key + "' must be string array");
}

} // namespace

SynergyTimer::SynergyTimer()  = default;
SynergyTimer::~SynergyTimer() = default;

std::string_view SynergyTimer::type_name() const { return "SynergyTimer"; }

std::vector<TopicSpec> SynergyTimer::input_topics() const {
    std::vector<TopicSpec> v;
    v.emplace_back(feet_y_topic_, std::type_index(typeid(ProprioToken)),
                   SubscriptionKind::Direct, /*required=*/true);
    v.emplace_back(topics::kEventsPrefix, std::type_index(typeid(EnvEvent)),
                   SubscriptionKind::Direct, /*required=*/false);
    for (auto const& pm : premotor_state_) {
        v.emplace_back(pm.policy_topic, std::type_index(typeid(PolicyToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    }
    return v;
}

std::vector<TopicSpec> SynergyTimer::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(premotor_state_.size());
    for (auto const& pm : premotor_state_) {
        v.emplace_back(std::string(topics::kRhythmBiasPrefix) + pm.id,
                       std::type_index(typeid(RhythmBiasToken)));
    }
    return v;
}

ParamSchema SynergyTimer::params_schema() const {
    return {
        {"feet_y_topic",        ParamMutability::ConstructionOnly,
            "ProprioToken topic carrying per-leg foot-Y (4 floats: FL, FR, RL, RR).",
            ParamValue{std::string("reality.proprio.feet_y")}},
        {"leg_names",           ParamMutability::ConstructionOnly,
            "Leg names in order matching feet_y vector indices.  Default ['fl','fr','rl','rr'].",
            std::nullopt},
        {"premotor_ids",        ParamMutability::ConstructionOnly,
            "Array of Premotor ids that SynergyTimer drives.  Same length as premotor_leg_assignment + premotor_policy_topics.",
            std::nullopt},
        {"premotor_leg_assignment", ParamMutability::ConstructionOnly,
            "Array mapping each Premotor id to a leg name (one of leg_names).  Same length as premotor_ids.",
            std::nullopt},
        {"premotor_policy_topics", ParamMutability::ConstructionOnly,
            "Array of policy_output_topic strings per Premotor (where SynergyTimer subscribes to read chosen intent).  Same length as premotor_ids.",
            std::nullopt},
        {"n_bins",              ParamMutability::ConstructionOnly,
            "Number of discrete phase bins per cycle.  Default 8.",
            ParamValue{int64_t{8}}},
        {"n_intents",           ParamMutability::ConstructionOnly,
            "Number of intents per Premotor.  Must match Premotor's n_intents.  Default 5.",
            ParamValue{int64_t{5}}},
        {"rhythm_bias_gain",    ParamMutability::HotMutable,
            "Scalar applied to the published bias vector.  0 = output silent (legacy).  Recommended 0.1-0.5; too high overrides brain authority.",
            ParamValue{0.0}},
        {"min_confidence",      ParamMutability::HotMutable,
            "Confidence threshold below which Hebbian updates are skipped.  Default 0.2.",
            ParamValue{0.2}},
        {"alpha_period",        ParamMutability::HotMutable,
            "EMA rate for inter_touchdown_ema update.  Default 0.05.",
            ParamValue{0.05}},
        {"alpha_err",           ParamMutability::HotMutable,
            "EMA rate for predicted_touchdown_error_ema update.  Default 0.05.",
            ParamValue{0.05}},
        {"learning_rate",       ParamMutability::HotMutable,
            "Hebbian update step on B[premotor, bin, intent] when reward fires.  Default 0.01.",
            ParamValue{0.01}},
        {"decay_per_tick",      ParamMutability::HotMutable,
            "Multiplicative decay on B every tick to prevent runaway.  Default 1e-5.",
            ParamValue{1e-5}},
        {"self_supervised_rate", ParamMutability::HotMutable,
            "2026-06-10 E1: when > 0, reinforce B(bin, chosen) every tick by this rate x rhythm_confidence WITHOUT a reward gate — amplifies the body's own phase-correlated intent structure into more coherent stepping.  Phase-binned so it strengthens phase-dependent alternation, not single-intent collapse.  Recommended 0.002-0.02.  0 = off (reward-gated legacy).",
            ParamValue{0.0}},
        {"bias_cap",            ParamMutability::HotMutable,
            "Clamp on |B| cells (both reward-gated and self-supervised paths).  Default 2.0.",
            ParamValue{2.0}},
        {"hysteresis_low_frac", ParamMutability::HotMutable,
            "Fraction of (high_ema - low_ema) above low_ema for plant threshold.  Default 0.25.",
            ParamValue{0.25}},
        {"hysteresis_high_frac",ParamMutability::HotMutable,
            "Fraction of (high_ema - low_ema) above low_ema for lift threshold.  Default 0.50.",
            ParamValue{0.50}},
        {"hysteresis_ema_alpha",ParamMutability::HotMutable,
            "EMA rate for low/high feet-Y trackers.  Default 0.01.",
            ParamValue{0.01}},
        {"max_period_ticks",    ParamMutability::HotMutable,
            "Soft cap on inter_touchdown_ema (ticks).  Default 180 = 3s at 60Hz.",
            ParamValue{int64_t{180}}},
        {"min_period_ticks",    ParamMutability::HotMutable,
            "Soft floor on inter_touchdown_ema (ticks).  Default 10.",
            ParamValue{int64_t{10}}},
        {"publish_when_silent", ParamMutability::HotMutable,
            "If true, publish zero-bias RhythmBiasToken even when confidence=0 (keeps Premotor subscription warm).  Default true.",
            ParamValue{true}},
    };
}

ParamMap SynergyTimer::current_params() const {
    ParamMap p;
    p["feet_y_topic"]         = ParamValue{feet_y_topic_};
    p["n_bins"]               = ParamValue{int64_t(n_bins_)};
    p["n_intents"]            = ParamValue{int64_t(n_intents_)};
    p["rhythm_bias_gain"]     = ParamValue{double(rhythm_bias_gain_)};
    p["min_confidence"]       = ParamValue{double(min_confidence_)};
    p["alpha_period"]         = ParamValue{double(alpha_period_)};
    p["alpha_err"]            = ParamValue{double(alpha_err_)};
    p["learning_rate"]        = ParamValue{double(learning_rate_)};
    p["decay_per_tick"]       = ParamValue{double(decay_per_tick_)};
    p["hysteresis_low_frac"]  = ParamValue{double(hysteresis_low_frac_)};
    p["hysteresis_high_frac"] = ParamValue{double(hysteresis_high_frac_)};
    p["hysteresis_ema_alpha"] = ParamValue{double(hysteresis_ema_alpha_)};
    p["max_period_ticks"]     = ParamValue{int64_t(max_period_ticks_)};
    p["min_period_ticks"]     = ParamValue{int64_t(min_period_ticks_)};
    p["publish_when_silent"]  = ParamValue{publish_when_silent_};
    return p;
}

void SynergyTimer::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("SynergyTimer requires a non-null Bus");

    apply_param(params, "feet_y_topic",         [&](auto const& v){ feet_y_topic_         = get_string(v, "feet_y_topic"); });
    apply_param(params, "n_bins",               [&](auto const& v){ n_bins_               = std::max(2, int(get_int(v, "n_bins"))); });
    apply_param(params, "n_intents",            [&](auto const& v){ n_intents_            = std::max(2, int(get_int(v, "n_intents"))); });
    apply_param(params, "rhythm_bias_gain",     [&](auto const& v){ rhythm_bias_gain_     = std::max(0.0f, float(get_double(v, "rhythm_bias_gain"))); });
    apply_param(params, "min_confidence",       [&](auto const& v){ min_confidence_       = std::clamp(float(get_double(v, "min_confidence")), 0.0f, 1.0f); });
    apply_param(params, "alpha_period",         [&](auto const& v){ alpha_period_         = std::clamp(float(get_double(v, "alpha_period")), 0.0f, 1.0f); });
    apply_param(params, "alpha_err",            [&](auto const& v){ alpha_err_            = std::clamp(float(get_double(v, "alpha_err")), 0.0f, 1.0f); });
    apply_param(params, "learning_rate",        [&](auto const& v){ learning_rate_        = std::max(0.0f, float(get_double(v, "learning_rate"))); });
    apply_param(params, "decay_per_tick",       [&](auto const& v){ decay_per_tick_       = std::clamp(float(get_double(v, "decay_per_tick")), 0.0f, 1.0f); });
    apply_param(params, "self_supervised_rate", [&](auto const& v){ self_supervised_rate_ = std::max(0.0f, float(get_double(v, "self_supervised_rate"))); });
    apply_param(params, "bias_cap",             [&](auto const& v){ bias_cap_             = std::max(0.0f, float(get_double(v, "bias_cap"))); });
    apply_param(params, "hysteresis_low_frac",  [&](auto const& v){ hysteresis_low_frac_  = std::clamp(float(get_double(v, "hysteresis_low_frac")), 0.0f, 1.0f); });
    apply_param(params, "hysteresis_high_frac", [&](auto const& v){ hysteresis_high_frac_ = std::clamp(float(get_double(v, "hysteresis_high_frac")), 0.0f, 1.0f); });
    apply_param(params, "hysteresis_ema_alpha", [&](auto const& v){ hysteresis_ema_alpha_ = std::clamp(float(get_double(v, "hysteresis_ema_alpha")), 0.0f, 1.0f); });
    apply_param(params, "max_period_ticks",     [&](auto const& v){ max_period_ticks_     = std::max(2, int(get_int(v, "max_period_ticks"))); });
    apply_param(params, "min_period_ticks",     [&](auto const& v){ min_period_ticks_     = std::max(1, int(get_int(v, "min_period_ticks"))); });
    apply_param(params, "publish_when_silent",  [&](auto const& v){ publish_when_silent_  = get_bool(v, "publish_when_silent"); });

    // ---- Leg state ----
    std::vector<std::string> leg_names = {"fl", "fr", "rl", "rr"};
    auto ln_it = params.find("leg_names");
    if (ln_it != params.end()) {
        leg_names = get_string_vec(ln_it->second, "leg_names");
    }
    leg_state_.clear();
    leg_state_.resize(leg_names.size());
    for (size_t i = 0; i < leg_names.size(); ++i) {
        leg_state_[i].name = leg_names[i];
    }

    // ---- Premotor state ----
    auto need = [&](char const* k) -> ParamValue const& {
        auto it = params.find(k);
        if (it == params.end())
            throw std::invalid_argument(std::string("SynergyTimer: required param '") + k + "' missing");
        return it->second;
    };
    auto premotor_ids       = get_string_vec(need("premotor_ids"),            "premotor_ids");
    auto premotor_legs      = get_string_vec(need("premotor_leg_assignment"), "premotor_leg_assignment");
    auto premotor_topics    = get_string_vec(need("premotor_policy_topics"),  "premotor_policy_topics");
    if (premotor_ids.size() != premotor_legs.size()
        || premotor_ids.size() != premotor_topics.size()) {
        throw std::invalid_argument("SynergyTimer: premotor_ids, premotor_leg_assignment, premotor_policy_topics must all have same length");
    }

    premotor_state_.clear();
    premotor_state_.resize(premotor_ids.size());
    for (size_t i = 0; i < premotor_ids.size(); ++i) {
        auto& pm = premotor_state_[i];
        pm.id           = premotor_ids[i];
        pm.policy_topic = premotor_topics[i];
        // Resolve leg name → index.
        int leg_idx = -1;
        for (int j = 0; j < int(leg_state_.size()); ++j) {
            if (leg_state_[j].name == premotor_legs[i]) { leg_idx = j; break; }
        }
        if (leg_idx < 0)
            throw std::invalid_argument("SynergyTimer: premotor_leg_assignment[" + std::to_string(i) + "] = '"
                                         + premotor_legs[i] + "' not in leg_names");
        pm.leg_idx = leg_idx;
        pm.B       = Eigen::MatrixXf::Zero(n_bins_, n_intents_);
        pm.last_bias_published = Eigen::VectorXf::Zero(n_intents_);
    }

    // ---- Subscriptions ----
    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(feet_y_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_feet_y(p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ this->handle_event(t, p); }));
    for (int i = 0; i < int(premotor_state_.size()); ++i) {
        int idx = i;
        sub_ids_.push_back(bus_->subscribe(premotor_state_[i].policy_topic,
            SubscriptionKind::Direct,
            [this, idx](std::string_view, MessagePtr p){ this->handle_policy(idx, p); }));
    }
}

void SynergyTimer::handle_feet_y(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p) return;
    if (int(p->values.size()) != int(leg_state_.size())) return;
    for (int i = 0; i < int(leg_state_.size()); ++i) {
        update_leg(i, float(p->values[i]), current_tick_);
    }
}

void SynergyTimer::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;
    std::string name(topic.substr(std::string(topics::kEventsPrefix).size()));
    float r = 0.0f;
    if      (name == "hit")  r =  e->intensity;
    else if (name == "miss") r = -e->intensity;
    else return;     // ignore other events
    if (r == 0.0f) return;
    apply_reward(r, current_tick_);
}

void SynergyTimer::handle_policy(int premotor_idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pol = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!pol) return;
    if (premotor_idx >= 0 && premotor_idx < int(premotor_state_.size())) {
        premotor_state_[premotor_idx].last_chosen_intent = pol->chosen_intent;
    }
}

void SynergyTimer::update_leg(int leg_idx, float foot_y, uint64_t tick_id) {
    auto& L = leg_state_[leg_idx];
    // Init thresholds on first sample.
    if (!L.thresholds_initialised) {
        L.feet_y_low_ema  = foot_y;
        L.feet_y_high_ema = foot_y;
        L.thresholds_initialised = true;
    } else {
        // Track lows (when planted) and highs (when lifted).
        if (L.is_planted) {
            L.feet_y_low_ema = (1.0f - hysteresis_ema_alpha_) * L.feet_y_low_ema
                              + hysteresis_ema_alpha_ * foot_y;
        } else {
            L.feet_y_high_ema = (1.0f - hysteresis_ema_alpha_) * L.feet_y_high_ema
                               + hysteresis_ema_alpha_ * foot_y;
        }
    }
    // Hysteresis thresholds.
    float span = std::max(0.005f, L.feet_y_high_ema - L.feet_y_low_ema);
    float plant_threshold = L.feet_y_low_ema + hysteresis_low_frac_  * span;
    float lift_threshold  = L.feet_y_low_ema + hysteresis_high_frac_ * span;
    // Detect touchdown (lifted → planted via plant_threshold downward).
    if (!L.is_planted && foot_y <= plant_threshold) {
        L.is_planted = true;
        on_touchdown(leg_idx, tick_id);
    } else if (L.is_planted && foot_y >= lift_threshold) {
        L.is_planted = false;
    }
    // Update phase + bin.
    if (L.last_touchdown_tick >= 0 && L.inter_touchdown_ema > 0.0f) {
        float elapsed = float(int64_t(tick_id) - L.last_touchdown_tick);
        L.phase     = std::fmod(elapsed / std::max(L.inter_touchdown_ema, 1.0f), 1.0f);
        if (L.phase < 0.0f) L.phase += 1.0f;
        L.phase_bin = std::clamp(int(std::floor(L.phase * float(n_bins_))), 0, n_bins_ - 1);
    } else {
        L.phase     = 0.0f;
        L.phase_bin = 0;
    }
}

void SynergyTimer::on_touchdown(int leg_idx, uint64_t tick_id) {
    auto& L = leg_state_[leg_idx];
    L.touchdown_count += 1;
    if (L.last_touchdown_tick < 0) {
        // First touchdown — no period to estimate yet, just record.
        L.last_touchdown_tick      = int64_t(tick_id);
        L.predicted_touchdown_tick = -1;
        return;
    }
    float observed_interval = float(int64_t(tick_id) - L.last_touchdown_tick);
    observed_interval       = std::clamp(observed_interval,
                                          float(min_period_ticks_),
                                          float(max_period_ticks_));
    // Prediction error (signed).
    float pred_err_abs = 0.0f;
    if (L.predicted_touchdown_tick >= 0) {
        pred_err_abs = std::fabs(float(int64_t(tick_id) - L.predicted_touchdown_tick));
    } else {
        // No prior prediction — initialise error EMA to a value derived from observed.
        pred_err_abs = observed_interval * 0.5f;
    }
    if (L.inter_touchdown_ema <= 0.0f) {
        L.inter_touchdown_ema = observed_interval;
    } else {
        L.inter_touchdown_ema += alpha_period_ * (observed_interval - L.inter_touchdown_ema);
    }
    if (L.predicted_error_ema <= 0.0f) {
        L.predicted_error_ema = pred_err_abs;
    } else {
        L.predicted_error_ema += alpha_err_ * (pred_err_abs - L.predicted_error_ema);
    }
    L.last_touchdown_tick      = int64_t(tick_id);
    L.predicted_touchdown_tick = int64_t(tick_id) + int64_t(L.inter_touchdown_ema);
    // Confidence: 1 when prediction error is small relative to period.
    L.rhythm_confidence = std::clamp(1.0f - L.predicted_error_ema / std::max(L.inter_touchdown_ema, 1.0f),
                                      0.0f, 1.0f);
}

void SynergyTimer::apply_reward(float reward_signed, uint64_t /*tick_id*/) {
    // Reward-gated Hebbian update on B[bin, intent] for each Premotor
    // whose leg has confident rhythm AND whose last_chosen_intent is known.
    for (auto& pm : premotor_state_) {
        if (pm.leg_idx < 0 || pm.leg_idx >= int(leg_state_.size())) continue;
        if (pm.last_chosen_intent < 0 || pm.last_chosen_intent >= n_intents_) continue;
        auto const& L = leg_state_[pm.leg_idx];
        if (L.rhythm_confidence < min_confidence_) continue;
        if (pm.B.rows() != n_bins_ || pm.B.cols() != n_intents_) continue;
        // Strengthen the (bin, chosen) cell.  Reward sign already
        // included — miss reduces, hit grows.
        pm.B(L.phase_bin, pm.last_chosen_intent) +=
            learning_rate_ * reward_signed * L.rhythm_confidence;
    }
}

void SynergyTimer::publish_biases(uint64_t tick_id) {
    for (auto& pm : premotor_state_) {
        if (pm.leg_idx < 0 || pm.leg_idx >= int(leg_state_.size())) continue;
        auto const& L = leg_state_[pm.leg_idx];
        Eigen::VectorXf bias = Eigen::VectorXf::Zero(n_intents_);
        bool active = (rhythm_bias_gain_ > 0.0f
                        && L.rhythm_confidence >= min_confidence_
                        && pm.B.rows() == n_bins_
                        && pm.B.cols() == n_intents_);
        if (active) {
            bias = (rhythm_bias_gain_ * L.rhythm_confidence) * pm.B.row(L.phase_bin).transpose();
        }
        pm.last_bias_published = bias;
        if (!active && !publish_when_silent_) continue;
        auto msg = std::make_shared<RhythmBiasToken>();
        msg->tick_id     = tick_id;
        msg->producer_id = id_.empty() ? std::string("synergy_timer") : id_;
        msg->bias        = bias;
        msg->confidence  = L.rhythm_confidence;
        msg->phase_bin   = L.phase_bin;
        bus_->publish(std::string(topics::kRhythmBiasPrefix) + pm.id, msg);
    }
}

void SynergyTimer::tick(uint64_t tick_id) {
    current_tick_ = tick_id;
    // Per-tick decay on B (safety against runaway).
    if (decay_per_tick_ > 0.0f) {
        float keep = 1.0f - decay_per_tick_;
        for (auto& pm : premotor_state_) {
            pm.B *= keep;
        }
    }
    // 2026-06-10 E1 — self-supervised rhythm-bias update (no reward gate).
    // Reinforce the (current phase bin, currently-chosen intent) cell for each
    // Premotor whose leg has confident rhythm.  Because the update is keyed on
    // the leg's touchdown-derived phase bin, it strengthens whatever intent the
    // leg consistently chooses AT THAT PHASE — amplifying phase-dependent
    // alternation (= coherent stepping) rather than collapsing to one intent
    // (a phase-invariant choice spreads uniformly across bins → no net bias).
    if (self_supervised_rate_ > 0.0f) {
        for (auto& pm : premotor_state_) {
            if (pm.leg_idx < 0 || pm.leg_idx >= int(leg_state_.size())) continue;
            if (pm.last_chosen_intent < 0 || pm.last_chosen_intent >= n_intents_) continue;
            if (pm.B.rows() != n_bins_ || pm.B.cols() != n_intents_) continue;
            auto const& L = leg_state_[pm.leg_idx];
            if (L.rhythm_confidence < min_confidence_) continue;
            float& cell = pm.B(L.phase_bin, pm.last_chosen_intent);
            cell += self_supervised_rate_ * L.rhythm_confidence;
            if (cell >  bias_cap_) cell =  bias_cap_;
            if (cell < -bias_cap_) cell = -bias_cap_;
        }
    }
    publish_biases(tick_id);
}

void SynergyTimer::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "rhythm_bias_gain")     rhythm_bias_gain_     = std::max(0.0f, float(get_double(value, k)));
    else if (k == "min_confidence")       min_confidence_       = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "alpha_period")         alpha_period_         = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "alpha_err")            alpha_err_            = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "learning_rate")        learning_rate_        = std::max(0.0f, float(get_double(value, k)));
    else if (k == "decay_per_tick")       decay_per_tick_       = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "self_supervised_rate") self_supervised_rate_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "bias_cap")             bias_cap_             = std::max(0.0f, float(get_double(value, k)));
    else if (k == "hysteresis_low_frac")  hysteresis_low_frac_  = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "hysteresis_high_frac") hysteresis_high_frac_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "hysteresis_ema_alpha") hysteresis_ema_alpha_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "max_period_ticks")     max_period_ticks_     = std::max(2, int(get_int(value, k)));
    else if (k == "min_period_ticks")     min_period_ticks_     = std::max(1, int(get_int(value, k)));
    else if (k == "publish_when_silent")  publish_when_silent_  = get_bool(value, k);
    else
        throw std::invalid_argument("SynergyTimer: param '" + k + "' is ConstructionOnly");
}

std::vector<float> SynergyTimer::rhythm_confidence_all() const {
    std::vector<float> out;
    out.reserve(leg_state_.size());
    for (auto const& L : leg_state_) out.push_back(L.rhythm_confidence);
    return out;
}
std::vector<float> SynergyTimer::period_ema_all() const {
    std::vector<float> out;
    out.reserve(leg_state_.size());
    for (auto const& L : leg_state_) out.push_back(L.inter_touchdown_ema);
    return out;
}
std::vector<int> SynergyTimer::phase_bin_all() const {
    std::vector<int> out;
    out.reserve(leg_state_.size());
    for (auto const& L : leg_state_) out.push_back(L.phase_bin);
    return out;
}
std::vector<int> SynergyTimer::touchdown_count_all() const {
    std::vector<int> out;
    out.reserve(leg_state_.size());
    for (auto const& L : leg_state_) out.push_back(L.touchdown_count);
    return out;
}

nlohmann::json SynergyTimer::snapshot_state() const {
    auto leg_arr = nlohmann::json::array();
    for (auto const& L : leg_state_) {
        leg_arr.push_back({
            {"name",                  L.name},
            {"is_planted",            L.is_planted},
            {"feet_y_low_ema",        double(L.feet_y_low_ema)},
            {"feet_y_high_ema",       double(L.feet_y_high_ema)},
            {"last_touchdown_tick",   int64_t(L.last_touchdown_tick)},
            {"inter_touchdown_ema",   double(L.inter_touchdown_ema)},
            {"predicted_error_ema",   double(L.predicted_error_ema)},
            {"predicted_touchdown_tick", int64_t(L.predicted_touchdown_tick)},
            {"rhythm_confidence",     double(L.rhythm_confidence)},
            {"phase",                 double(L.phase)},
            {"phase_bin",             L.phase_bin},
            {"touchdown_count",       L.touchdown_count},
        });
    }
    auto pm_arr = nlohmann::json::array();
    for (auto const& pm : premotor_state_) {
        // L2 norm of B per bin — quick view of synergy crystallisation.
        std::vector<double> b_norm_by_bin(n_bins_, 0.0);
        if (pm.B.rows() == n_bins_ && pm.B.cols() == n_intents_) {
            for (int bin = 0; bin < n_bins_; ++bin) {
                b_norm_by_bin[bin] = double(pm.B.row(bin).norm());
            }
        }
        nlohmann::json bias_arr = nlohmann::json::array();
        for (int i = 0; i < int(pm.last_bias_published.size()); ++i) {
            bias_arr.push_back(double(pm.last_bias_published(i)));
        }
        pm_arr.push_back({
            {"id",                   pm.id},
            {"leg_idx",              pm.leg_idx},
            {"last_chosen_intent",   pm.last_chosen_intent},
            {"B_norm_by_bin",        b_norm_by_bin},
            {"last_bias_published",  bias_arr},
        });
    }
    return nlohmann::json{
        {"version",          1},
        {"n_bins",           n_bins_},
        {"n_intents",        n_intents_},
        {"rhythm_bias_gain", double(rhythm_bias_gain_)},
        {"legs",             leg_arr},
        {"premotors",        pm_arr},
    };
}

} // namespace ogma
