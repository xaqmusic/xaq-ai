// ⚠️ FULLY DEPRECATED / ARCHIVED (2026-07-03) — see the deprecation banner in
// HomeokineticExploration.hpp. Retired motor-repertoire-era stuck-detector
// (predicts nothing, random-accel escape, hand-tuned constants); superseded by
// the cell PlayLoop. Do not wire into new configs; do not extend.
#include "ogma/modules/HomeokineticExploration.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

#include "ogma/Rng.hpp"

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
    throw std::invalid_argument("HomeokineticExploration param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("HomeokineticExploration param '" + key + "' must be integer");
}

} // namespace

HomeokineticExploration::HomeokineticExploration()  = default;
HomeokineticExploration::~HomeokineticExploration() = default;

std::string_view HomeokineticExploration::type_name() const { return "HomeokineticExploration"; }

std::vector<TopicSpec> HomeokineticExploration::input_topics() const {
    return {
        TopicSpec{topics::kDriveErrors, std::type_index(typeid(DriveErrors)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kMotorChunks, std::type_index(typeid(MotorChunks)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kActionOut,   std::type_index(typeid(ActionOut)),
                  SubscriptionKind::Direct, /*required=*/false},
        // Phase-6.1+: count hits during episodes — better outcome signal
        // than the urgency-drop proxy that the prior comment flagged as
        // "too rare."  events.hit is the same signal MotorRepertoire's
        // chunk crystallisation uses, so success now means the same thing
        // across both adaptive systems.
        TopicSpec{topics::kPolicyIntent, std::type_index(typeid(PolicyToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kEventsPrefix, std::type_index(typeid(EnvEvent)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> HomeokineticExploration::output_topics() const {
    return {
        TopicSpec{topics::kExplorationDirective, std::type_index(typeid(ExplorationDirective))},
    };
}

ParamSchema HomeokineticExploration::params_schema() const {
    return {
        // Time-scale parameters (physical horizons).
        {"window_ticks",            ParamMutability::HotMutable,       "Short-window length for current state (~2 s)",       ParamValue{int64_t{120}}},
        {"long_window_ticks",       ParamMutability::HotMutable,       "Long-history buffer length for percentile baseline (~8 s)", ParamValue{int64_t{480}}},
        {"change_ema_alpha",        ParamMutability::HotMutable,       "EMA learning rate for long-running |Δurgency|",      ParamValue{0.002}},
        // Anomaly factor — gate fires below `anomaly_factor × median(history)`.
        {"anomaly_factor",          ParamMutability::HotMutable,       "Fire when current change-rate < factor × median (default 0.5 = 'half of typical')", ParamValue{0.5}},
        {"chunk_match_eps",         ParamMutability::HotMutable,       "Best chunk outcome above this suppresses gate",      ParamValue{0.01}},
        {"episode_ticks",           ParamMutability::HotMutable,       "Initial episode length",                             ParamValue{int64_t{60}}},
        {"cooldown_ticks",          ParamMutability::HotMutable,       "Initial cooldown after episode",                     ParamValue{int64_t{30}}},
        {"accel_jitter",            ParamMutability::HotMutable,       "Initial max |accel| sampled per episode",            ParamValue{1.0}},
        {"master_seed",             ParamMutability::ConstructionOnly, "RNG namespace seed",                                 ParamValue{int64_t{0}}},
        {"saturation_clamp",        ParamMutability::HotMutable,       "Phase 6.7: urgency above this counts as saturated.  Used in combination with saturation_streak_threshold to OR-trigger the gate when long_change_ema has decayed to 0 (the user's stuck-pose state where the ratio gate cannot fire).  Default 0 = disabled.  Recommended 0.98 to catch only true clamp_hi saturation.", ParamValue{0.0}},
        {"saturation_streak_threshold", ParamMutability::HotMutable,   "Phase 6.7: # of consecutive saturated samples required to OR-trigger the gate.  Default 0 = disabled.  Recommended 600 (10 s @ 60 Hz drive ticks).  Pairs with saturation_clamp.", ParamValue{int64_t{0}}},
        {"entropy_collapse_fraction", ParamMutability::HotMutable,     "Phase 6.7+: OR-trigger the gate when any subscribed Premotor's smoothed entropy drops below this fraction of its historical peak.  Adaptive — peak is per-Premotor, no static threshold.  0 = disabled (default).  Recommended 0.5 = 'half of peak entropy'.  Pairs with entropy_min_peak (peak must exceed it to avoid early-training false positives) and entropy_cooldown_ticks (per-Premotor refire spacing).", ParamValue{0.0}},
        {"entropy_min_peak",         ParamMutability::HotMutable,      "Phase 6.7+: minimum historical entropy peak for a Premotor before entropy-collapse trigger can fire on it.  Default 1.0 — must reach H >= 1.0 (= 80% of uniform-prior on n_intents=5).  Filters out Premotors still in early-training startup.", ParamValue{1.0}},
        {"entropy_ema_alpha",        ParamMutability::HotMutable,      "Phase 6.7+: EMA smoothing rate for per-Premotor entropy tracking.  Default 0.05 (~20-tick window).  Higher = tracks raw entropy faster but more noise; lower = smoother but slower to detect drift.", ParamValue{0.05}},
        {"entropy_cooldown_ticks",   ParamMutability::HotMutable,      "Phase 6.7+: # of ticks per Premotor before its entropy-collapse trigger can re-fire.  Default 1500 (one MC episode).  Prevents continuous firing while the trigger condition holds; gives REINFORCE a window to respond between fires.", ParamValue{int64_t{1500}}},
        {"entropy_recovery_window_ticks", ParamMutability::HotMutable, "Phase 6.7++ dynamic escalation: # of ticks after episode end before checking recovery.  Default 60 (1 s).  Pairs with entropy_recover_fraction; together they decide whether the LAST fire on this Premotor 'worked' (resets consecutive_fires) or 'failed' (escalates next fire's duration).", ParamValue{int64_t{60}}},
        {"entropy_recover_fraction",  ParamMutability::HotMutable,     "Phase 6.7++ dynamic escalation: fraction of peak that entropy_ema must exceed at recovery check to count as 'recovered'.  Default 0.7 = 'returned to within 30% of historical peak'.  Above collapse_fraction (0.5) so an intervention can be deemed successful even if entropy hasn't fully recovered.", ParamValue{0.7}},
        {"entropy_max_episode_ticks", ParamMutability::HotMutable,     "Phase 6.7++ dynamic escalation: cap on episode_ticks under repeated-fire escalation.  Each consecutive fire on the same Premotor doubles the episode duration up to this cap.  Default = base episode_ticks (no escalation).  Set higher (e.g. 600 = 10 s) to enable.", ParamValue{int64_t{60}}},
    };
}

void HomeokineticExploration::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("HomeokineticExploration requires a non-null Bus");

    apply_param(params, "window_ticks",            [&](auto const& v){ window_ticks_            = std::max(2, int(get_int(v, "window_ticks"))); });
    apply_param(params, "long_window_ticks",       [&](auto const& v){ long_window_ticks_       = std::max(window_ticks_, int(get_int(v, "long_window_ticks"))); });
    apply_param(params, "change_ema_alpha",        [&](auto const& v){ change_ema_alpha_        = float(get_double(v, "change_ema_alpha")); });
    apply_param(params, "anomaly_factor",          [&](auto const& v){ anomaly_factor_          = std::clamp(float(get_double(v, "anomaly_factor")), 0.0f, 1.0f); });
    apply_param(params, "chunk_match_eps",         [&](auto const& v){ chunk_match_eps_         = float(get_double(v, "chunk_match_eps")); });
    apply_param(params, "episode_ticks",           [&](auto const& v){ episode_ticks_           = std::max(1, int(get_int(v, "episode_ticks"))); });
    apply_param(params, "cooldown_ticks",          [&](auto const& v){ cooldown_ticks_          = std::max(0, int(get_int(v, "cooldown_ticks"))); });
    apply_param(params, "accel_jitter",            [&](auto const& v){ accel_jitter_            = float(get_double(v, "accel_jitter")); });
    apply_param(params, "master_seed",             [&](auto const& v){ master_seed_             = uint64_t(get_int(v, "master_seed")); });
    apply_param(params, "saturation_clamp",        [&](auto const& v){ saturation_clamp_        = float(get_double(v, "saturation_clamp")); });
    apply_param(params, "saturation_streak_threshold", [&](auto const& v){ saturation_streak_threshold_ = int(get_int(v, "saturation_streak_threshold")); });
    apply_param(params, "entropy_collapse_fraction", [&](auto const& v){ entropy_collapse_fraction_   = float(get_double(v, "entropy_collapse_fraction")); });
    apply_param(params, "entropy_min_peak",        [&](auto const& v){ entropy_min_peak_         = float(get_double(v, "entropy_min_peak")); });
    apply_param(params, "entropy_ema_alpha",       [&](auto const& v){ entropy_ema_alpha_        = std::clamp(float(get_double(v, "entropy_ema_alpha")), 0.0f, 1.0f); });
    apply_param(params, "entropy_cooldown_ticks",  [&](auto const& v){ entropy_cooldown_ticks_   = int(get_int(v, "entropy_cooldown_ticks")); });
    apply_param(params, "entropy_recovery_window_ticks", [&](auto const& v){ entropy_recovery_window_ticks_ = int(get_int(v, "entropy_recovery_window_ticks")); });
    apply_param(params, "entropy_recover_fraction", [&](auto const& v){ entropy_recover_fraction_ = float(get_double(v, "entropy_recover_fraction")); });
    apply_param(params, "entropy_max_episode_ticks", [&](auto const& v){ entropy_max_episode_ticks_ = int(get_int(v, "entropy_max_episode_ticks")); });

    episode_rng_ = derive_rng(master_seed_,
        std::string("kinesis.") + (id_.empty() ? std::string("homeokinetic") : id_) + ".episode");

    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(topics::kDriveErrors, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_drive(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kMotorChunks, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_chunks(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kActionOut,   SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_action(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_event(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kPolicyIntent, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_policy_intent(t, p); }));
}

// Phase 6.7+ — per-Premotor entropy tracking via PolicyToken.
// Premotor publishes PolicyToken every tick with `entropy = last_entropy_`
// already populated.  We maintain a peak + EMA per producer_id, with
// the gate consulting these maps in gate_holds().
void HomeokineticExploration::handle_policy_intent(std::string_view /*topic*/, MessagePtr payload) {
    if (entropy_collapse_fraction_ <= 0.0f) return;   // trigger disabled
    if (!input_allowed(payload->producer_id)) return;
    auto t = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!t) return;
    std::string const& pid = payload->producer_id;
    if (pid.empty()) return;
    auto& st = entropy_state_[pid];
    if (!st.initialised) {
        st.ema = t->entropy;
        st.peak = t->entropy;
        st.initialised = true;
    } else {
        float a = entropy_ema_alpha_;
        st.ema = (1.0f - a) * st.ema + a * t->entropy;
        if (st.ema > st.peak) st.peak = st.ema;
    }
    entropy_last_seen_tick_ = t->tick_id;
}

void HomeokineticExploration::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    auto const& name = !ev->name.empty()
                        ? ev->name
                        : std::string(topic.substr(std::min(topic.size(),
                            std::string_view("events.").size())));
    // Only count hits arriving while an episode is active — that's what
    // makes "this episode succeeded" meaningful.  Hits between episodes
    // are MotorRepertoire's concern; we don't credit them to exploration.
    if (name == "hit" && ticks_remaining_ > 0) ++episode_hits_;
}

void HomeokineticExploration::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "window_ticks")            window_ticks_            = std::max(2, int(get_int(value, k)));
    else if (k == "long_window_ticks")       long_window_ticks_       = std::max(window_ticks_, int(get_int(value, k)));
    else if (k == "change_ema_alpha")        change_ema_alpha_        = float(get_double(value, k));
    else if (k == "anomaly_factor")          anomaly_factor_          = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "chunk_match_eps")         chunk_match_eps_         = float(get_double(value, k));
    else if (k == "episode_ticks")           episode_ticks_           = std::max(1, int(get_int(value, k)));
    else if (k == "cooldown_ticks")          cooldown_ticks_          = std::max(0, int(get_int(value, k)));
    else if (k == "accel_jitter")            accel_jitter_            = float(get_double(value, k));
    else if (k == "saturation_clamp")        saturation_clamp_        = float(get_double(value, k));
    else if (k == "saturation_streak_threshold") saturation_streak_threshold_ = int(get_int(value, k));
    else if (k == "entropy_collapse_fraction") entropy_collapse_fraction_ = float(get_double(value, k));
    else if (k == "entropy_min_peak")        entropy_min_peak_        = float(get_double(value, k));
    else if (k == "entropy_ema_alpha")       entropy_ema_alpha_       = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "entropy_cooldown_ticks")  entropy_cooldown_ticks_  = int(get_int(value, k));
    else if (k == "entropy_recovery_window_ticks") entropy_recovery_window_ticks_ = int(get_int(value, k));
    else if (k == "entropy_recover_fraction") entropy_recover_fraction_ = float(get_double(value, k));
    else if (k == "entropy_max_episode_ticks") entropy_max_episode_ticks_ = int(get_int(value, k));
    else if (k == "master_seed")
        throw std::invalid_argument("HomeokineticExploration.master_seed is ConstructionOnly");
    else
        throw std::invalid_argument("HomeokineticExploration: unknown param '" + k + "'");
}

void HomeokineticExploration::handle_drive(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto d = std::dynamic_pointer_cast<const DriveErrors>(payload);
    if (!d) return;
    float u = d->urgency;

    // Phase 6.7 — saturation streak tracking.  Incremented while urgency
    // is above the configured clamp; reset to 0 on first sample below.
    // Disabled when saturation_streak_threshold_=0 (default).
    if (saturation_streak_threshold_ > 0) {
        if (u >= saturation_clamp_) {
            ++saturation_streak_;
        } else {
            saturation_streak_ = 0;
        }
    }

    // Short-window: most recent N samples for current state.
    urgency_window_.push_back(u);
    while (int(urgency_window_.size()) > window_ticks_) urgency_window_.pop_front();

    // Long-window: percentile baseline (sliding window, robust to step
    // changes — old samples fall out cleanly, no transient inflation).
    urgency_long_buffer_.push_back(u);
    while (int(urgency_long_buffer_.size()) > long_window_ticks_)
        urgency_long_buffer_.pop_front();

    float a = change_ema_alpha_;

    if (!drive_seen_) {
        prev_urgency_ = u;
        drive_seen_   = true;
        ++sample_count_;
        return;
    }

    float delta = std::abs(u - prev_urgency_);
    long_change_ema_ = (1.0f - a) * long_change_ema_ + a * delta;
    prev_urgency_ = u;

    // Per-tick short/long ratio — push into long history once long_ema is
    // non-zero (otherwise the ratio is undefined).
    if (long_change_ema_ > 0.0f) {
        float r = delta / long_change_ema_;
        ratio_long_buffer_.push_back(r);
        while (int(ratio_long_buffer_.size()) > long_window_ticks_)
            ratio_long_buffer_.pop_front();
    }

    ++sample_count_;
}

namespace {
// Median of an unsorted snapshot.  O(N log N), trivial at N ≤ 1000 / 60 Hz.
float median(std::deque<float> const& src) {
    if (src.empty()) return 0.0f;
    std::vector<float> s(src.begin(), src.end());
    std::sort(s.begin(), s.end());
    size_t n = s.size();
    if (n & 1u) return s[n / 2];
    return 0.5f * (s[n / 2 - 1] + s[n / 2]);
}
} // namespace

void HomeokineticExploration::handle_chunks(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto c = std::dynamic_pointer_cast<const MotorChunks>(payload);
    if (!c) return;
    float best = -std::numeric_limits<float>::infinity();
    for (auto const& ch : c->chunks)
        if (ch.outcome_drive_delta > best) best = ch.outcome_drive_delta;
    chunk_blocks_ = (best > chunk_match_eps_);
}

void HomeokineticExploration::handle_action(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    action_chunk_left_ = a->chunk_remaining_ticks;
}

bool HomeokineticExploration::gate_holds() const {
    if (chunk_blocks_)                                          return false;
    if (action_chunk_left_ > 0)                                 return false;
    // Phase 6.7+ — entropy-collapse short-circuit (does not require
    // urgency-window data — entropy state comes from PolicyToken stream).
    // When any subscribed Premotor's smoothed entropy has dropped below
    // entropy_collapse_fraction_ × its historical peak (with peak above
    // entropy_min_peak_ to avoid early-training false positives), AND that
    // Premotor is past its cooldown, fire and consume the trigger by
    // updating the cooldown.  Bypasses urgency-based gating since the
    // failure mode this catches doesn't show up in drive_errors.
    if (entropy_collapse_fraction_ > 0.0f && !entropy_state_.empty()) {
        for (auto& [pid, st] : entropy_state_) {
            if (!st.initialised)                                continue;
            if (st.peak < entropy_min_peak_)                    continue;
            if (st.ema >= entropy_collapse_fraction_ * st.peak) continue;
            if (entropy_last_seen_tick_ < st.cooldown_until)    continue;
            // Trigger fires.  Consume by updating cooldown for THIS
            // Premotor; others retain their independent cooldowns.
            // Capture the triggering pid so tick() can size the episode
            // dynamically based on consecutive_fires.
            st.cooldown_until = entropy_last_seen_tick_
                              + static_cast<uint64_t>(entropy_cooldown_ticks_);
            ++entropy_collapse_fires_;
            entropy_last_trigger_pid_ = pid;
            return true;
        }
    }
    if (int(urgency_window_.size()) < window_ticks_)            return false;
    // Phase 6.7 — saturation-streak short-circuit.  When urgency has been
    // saturated above clamp for the configured streak, fire regardless of
    // the ratio-history gate (which structurally cannot fire here since
    // long_change_ema_ has decayed to 0).  Bypasses the ratio_long_buffer
    // size check too — the saturation state has its own threshold.
    if (saturation_streak_threshold_ > 0
        && saturation_streak_ >= saturation_streak_threshold_) {
        return true;
    }
    if (int(ratio_long_buffer_.size()) < long_window_ticks_)    return false;
    if (long_change_ema_ <= 0.0f)                               return false;

    // Sole gate criterion: short/long change ratio is in the bottom-tail
    // of its own historical distribution.  Robust to:
    //  - urgency saturation: ratio is normalized by long_change_ema, not
    //    raw urgency level
    //  - sustained shifts: percentile compares against actual recent
    //    history, not a fixed scale
    //  - persistence at the new normal: once "stuck" has been the norm
    //    for the long_window's duration, the ratio buffer fills with low
    //    values and "low" stops being anomalous — natural disengagement
    double sum_abs_delta = 0.0;
    for (size_t i = 1; i < urgency_window_.size(); ++i) {
        sum_abs_delta += std::abs(double(urgency_window_[i])
                                - double(urgency_window_[i - 1]));
    }
    float current_ratio = float(sum_abs_delta
                              / double(urgency_window_.size() - 1)
                              / double(long_change_ema_));
    // Anomalously low: current change-rate is below half of the recent
    // median.  The factor 0.5 ("half of typical") is a universal anomaly
    // convention — robust to outlier transients (median, not mean) and
    // self-disengaging in sustained dead zones (median drifts toward the
    // new low normal, gate stops being able to fire below 0.5 × it).
    float med = median(ratio_long_buffer_);
    return med > 0.0f && current_ratio < anomaly_factor_ * med;
}

void HomeokineticExploration::publish_directive(uint64_t tick_id) {
    auto out = std::make_shared<ExplorationDirective>();
    out->tick_id         = tick_id;
    out->producer_id     = id_.empty() ? std::string("homeokinetic_exploration") : id_;
    out->active          = ticks_remaining_ > 0;
    out->ticks_remaining = ticks_remaining_;
    out->episode_id      = current_episode_id_;
    out->accel           = held_accel_;
    bus_->publish(topics::kExplorationDirective, out);
}

// Outcome-feedback adaptation.  Phase-6.1+: success is now "did at least
// one events.hit fire during the episode" rather than "did urgency drop"
// (the prior proxy was too rare and decoupled from real reward).  The
// new signal is the same one MotorRepertoire uses for chunk gating, so
// success means the same thing across both adaptive systems.
//
// The success_rate_ema_ is updated in tick() at episode end; effective_*
// still return baseline because the right *shape* of adaptation (longer
// episodes on failure?  more jitter?  different gate threshold?) is
// itself an open question that wants empirical data first.  Switching
// the signal without committing to a shape lets the rate accumulate
// honestly, which we can then read in metrics.
int   HomeokineticExploration::effective_episode_ticks() const {
    return episode_ticks_;
}
float HomeokineticExploration::effective_accel_jitter() const {
    return accel_jitter_;
}

void HomeokineticExploration::tick(uint64_t tick_id) {
    // 1. Drain a tick of any active episode.
    if (ticks_remaining_ > 0) {
        --ticks_remaining_;
        if (ticks_remaining_ == 0) {
            // Outcome: did the agent score during the episode?  events.hit
            // is the ground-truth reward signal.  Sign-only — count vs no
            // count — so we don't smuggle a magnitude threshold in.
            float success  = (episode_hits_ > 0) ? 1.0f : 0.0f;
            float a        = change_ema_alpha_;
            success_rate_ema_ = (1.0f - a) * success_rate_ema_ + a * success;
            episode_hits_  = 0;
            cooldown_remaining_ = cooldown_ticks_;
            // Phase 6.7++ dynamic escalation: if this episode was an
            // entropy-collapse trigger, schedule a recovery check tick
            // recovery_window_ticks after episode end.  The Premotor's
            // consecutive_fires increments at that future check if its
            // entropy_ema hasn't recovered above recover_fraction × peak.
            if (!entropy_last_trigger_pid_.empty()) {
                auto it = entropy_state_.find(entropy_last_trigger_pid_);
                if (it != entropy_state_.end()) {
                    it->second.recovery_check_at =
                        tick_id + static_cast<uint64_t>(entropy_recovery_window_ticks_);
                }
                entropy_last_trigger_pid_.clear();
            }
        }
    }
    else if (cooldown_remaining_ > 0) {
        --cooldown_remaining_;
    }
    // 2. If idle and gate holds, arm a new episode.
    else if (gate_holds()) {
        ++current_episode_id_;
        ++episodes_armed_;
        arm_urgency_     = drive_seen_ ? prev_urgency_ : 0.0f;
        // Phase 6.7++ dynamic episode duration.  If the gate fired on an
        // entropy-collapse trigger (entropy_last_trigger_pid_ is set),
        // size the episode by the triggering Premotor's consecutive_fires
        // count: base_episode_ticks * 2^consecutive_fires, capped at
        // entropy_max_episode_ticks_.  Otherwise fall back to the legacy
        // duration (ratio / saturation gate paths).
        int dur = effective_episode_ticks();
        if (!entropy_last_trigger_pid_.empty()) {
            auto it = entropy_state_.find(entropy_last_trigger_pid_);
            if (it != entropy_state_.end()) {
                int scaled = dur * (1 << std::min(it->second.consecutive_fires, 16));
                dur = std::min(scaled, entropy_max_episode_ticks_);
                dur = std::max(dur, effective_episode_ticks());
            }
        }
        ticks_remaining_ = dur;
        float jitter     = effective_accel_jitter();
        std::uniform_real_distribution<float> dist(-jitter, jitter);
        held_accel_      = dist(episode_rng_);
    }
    // 3. Recovery-check evaluation: every tick, walk entropy_state_ and
    //    for any pid whose recovery_check_at == tick_id, decide whether
    //    the last fire on that pid succeeded or failed.  Updates
    //    consecutive_fires accordingly.
    if (entropy_collapse_fraction_ > 0.0f) {
        for (auto& [pid, st] : entropy_state_) {
            if (st.recovery_check_at == 0)                continue;
            if (st.recovery_check_at != tick_id)          continue;
            st.recovery_check_at = 0;
            if (st.peak > 0.0f && st.ema >= entropy_recover_fraction_ * st.peak) {
                // Recovered — reset escalation.
                st.consecutive_fires = 0;
            } else {
                // Still stuck — escalate.  Also force-expire the cooldown
                // so the gate can re-fire immediately on the next tick.
                ++st.consecutive_fires;
                st.cooldown_until = 0;
            }
        }
    }
    // 4. Always publish — subscribers never need to handle a missing message.
    publish_directive(tick_id);
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

namespace {
std::string rng_to_string(std::mt19937_64 const& rng) {
    std::ostringstream oss;
    oss << rng;
    return oss.str();
}
void rng_from_string(std::mt19937_64& rng, std::string const& s) {
    if (s.empty()) return;
    std::istringstream iss(s);
    iss >> rng;
}
} // namespace

nlohmann::json HomeokineticExploration::snapshot_state() const {
    nlohmann::json urg_win = nlohmann::json::array();
    for (auto v : urgency_window_) urg_win.push_back(v);
    nlohmann::json urg_long = nlohmann::json::array();
    for (auto v : urgency_long_buffer_) urg_long.push_back(v);
    nlohmann::json ratio_long = nlohmann::json::array();
    for (auto v : ratio_long_buffer_) ratio_long.push_back(v);
    return nlohmann::json{
        {"version",             1},
        {"urgency_window",      urg_win},
        {"chunk_blocks",        chunk_blocks_},
        {"action_chunk_left",   action_chunk_left_},
        {"ticks_remaining",     ticks_remaining_},
        {"cooldown_remaining",  cooldown_remaining_},
        {"current_episode_id",  current_episode_id_},
        {"held_accel",          held_accel_},
        {"episodes_armed",      episodes_armed_},
        {"drive_seen",          drive_seen_},
        {"prev_urgency",        prev_urgency_},
        {"long_change_ema",     long_change_ema_},
        {"arm_urgency",         arm_urgency_},
        {"success_rate_ema",    success_rate_ema_},
        {"episode_hits",        episode_hits_},
        {"urgency_long_buffer", urg_long},
        {"ratio_long_buffer",   ratio_long},
        {"sample_count",        sample_count_},
        {"episode_rng",         rng_to_string(episode_rng_)},
        {"saturation_streak",   saturation_streak_},
    };
}

void HomeokineticExploration::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("HomeokineticExploration::restore_state: unknown version " +
                                 std::to_string(version));
    }
    chunk_blocks_         = s.value("chunk_blocks",       chunk_blocks_);
    action_chunk_left_    = s.value("action_chunk_left",  action_chunk_left_);
    ticks_remaining_      = s.value("ticks_remaining",    ticks_remaining_);
    cooldown_remaining_   = s.value("cooldown_remaining", cooldown_remaining_);
    current_episode_id_   = s.value("current_episode_id", current_episode_id_);
    held_accel_           = s.value("held_accel",         held_accel_);
    episodes_armed_       = s.value("episodes_armed",     episodes_armed_);
    drive_seen_           = s.value("drive_seen",         drive_seen_);
    prev_urgency_         = s.value("prev_urgency",       prev_urgency_);
    long_change_ema_      = s.value("long_change_ema",    long_change_ema_);
    arm_urgency_          = s.value("arm_urgency",        arm_urgency_);
    success_rate_ema_     = s.value("success_rate_ema",   success_rate_ema_);
    episode_hits_         = s.value("episode_hits",       episode_hits_);
    saturation_streak_    = s.value("saturation_streak",  saturation_streak_);
    sample_count_         = s.value("sample_count",       sample_count_);
    urgency_window_.clear();
    if (s.contains("urgency_window") && s["urgency_window"].is_array())
        for (auto const& v : s["urgency_window"]) urgency_window_.push_back(v.get<float>());
    urgency_long_buffer_.clear();
    if (s.contains("urgency_long_buffer") && s["urgency_long_buffer"].is_array())
        for (auto const& v : s["urgency_long_buffer"]) urgency_long_buffer_.push_back(v.get<float>());
    ratio_long_buffer_.clear();
    if (s.contains("ratio_long_buffer") && s["ratio_long_buffer"].is_array())
        for (auto const& v : s["ratio_long_buffer"]) ratio_long_buffer_.push_back(v.get<float>());
    rng_from_string(episode_rng_, s.value("episode_rng", std::string{}));
}

} // namespace ogma
