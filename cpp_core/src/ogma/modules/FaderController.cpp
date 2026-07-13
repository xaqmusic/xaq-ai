#include "ogma/modules/FaderController.hpp"

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

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("FaderController param '" + key + "' must be numeric");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("FaderController param '" + key + "' must be string");
}

} // namespace

FaderController::FaderController()  = default;
FaderController::~FaderController() = default;

std::string_view FaderController::type_name() const { return "FaderController"; }

std::vector<TopicSpec> FaderController::input_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{consensus_topic_, std::type_index(typeid(ConsensusToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        // v5.4.M — Premotor PolicyToken stream for premotor_certainty α
        // source.  Declared so manual-routing graphs auto-connect; also
        // needed when input_allowed() is enforced on inline lambdas.
        TopicSpec{topics::kPolicyIntent, std::type_index(typeid(PolicyToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
    // Urgency channel: reserved input; advertise the topic only when
    // configured so the graph editor doesn't surface a phantom edge.
    if (!urgency_topic_.empty()) {
        v.push_back(TopicSpec{urgency_topic_, std::type_index(typeid(DriveErrors)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    return v;
}

std::vector<TopicSpec> FaderController::output_topics() const {
    return {
        TopicSpec{alpha_topic_, std::type_index(typeid(FaderState))},
    };
}

ParamSchema FaderController::params_schema() const {
    return {
        {"consensus_topic",  ParamMutability::ConstructionOnly,
            "ConsensusToken topic source for surprise EMA",
            ParamValue{std::string("consensus.0")}},
        {"alpha_topic",      ParamMutability::ConstructionOnly,
            "FaderState publication topic (MotorFader instances subscribe here)",
            ParamValue{std::string(topics::kMotorFaderAlpha)}},
        {"urgency_topic",    ParamMutability::ConstructionOnly,
            "Reserved DriveErrors topic for the deferred panic channel "
            "(empty = disabled in 6.6.G)",
            ParamValue{std::string("")}},
        {"action_topic",     ParamMutability::ConstructionOnly,
            "v5.4 Phase B: ActionOut topic to read chunk_id from when alpha_source='chunk_gated' (default action.out)",
            ParamValue{std::string("action.out")}},
        {"alpha_chunk_active", ParamMutability::HotMutable,
            "v5.4 Phase B: α applied while a chunk is replaying (ActionOut.chunk_id != -1).  Default 1.0 (full brain authority — suppress reflex during chunk).",
            ParamValue{1.0}},
        {"alpha_chunk_idle",   ParamMutability::HotMutable,
            "v5.4 Phase B: α applied while no chunk is replaying.  Default 0.5 (balanced brain+reflex blend).",
            ParamValue{0.5}},
        {"alpha_quality_k",         ParamMutability::HotMutable,
            "v5.4 Phase F (Proposal B): sigmoid steepness for chunk_quality_sigmoid alpha source.  Default 8.0 (sharp transition near midpoint).  Higher = closer to binary; lower = gentler ramp.",
            ParamValue{8.0}},
        {"alpha_quality_midpoint",  ParamMutability::HotMutable,
            "v5.4 Phase F (Proposal B): sigmoid midpoint quality value for chunk_quality_sigmoid.  Default 0.55 (so a chunk with quality just above 0.5 commit threshold lifts α toward midpoint, below it sinks toward α_min).",
            ParamValue{0.55}},
        {"alpha_quality_min",       ParamMutability::HotMutable,
            "v5.4 Phase F (Proposal B): α value at quality=0 (no chunk).  Default 0.5 (matches alpha_chunk_idle).",
            ParamValue{0.5}},
        {"alpha_quality_max",       ParamMutability::HotMutable,
            "v5.4 Phase F (Proposal B): α value at quality=1 (perfect chunk).  Default 0.95 (near-full brain authority but a sliver of reflex still contributes).",
            ParamValue{0.95}},
        {"alpha_certainty_min",     ParamMutability::HotMutable,
            "v5.4.M (premotor_certainty α source): α value when Premotor softmax entropy is at maximum (fully uncertain policy).  Default 0.30 — reflex chemotaxis carries the agent during uncertain moments.",
            ParamValue{0.30}},
        {"alpha_certainty_max",     ParamMutability::HotMutable,
            "v5.4.M (premotor_certainty α source): α value when Premotor entropy = 0 (fully committed policy).  Default 0.95 — brain dominates when it knows what to do.",
            ParamValue{0.95}},
        {"alpha_source",     ParamMutability::HotMutable,
            "How α is derived: 'fixed' or 'surprise'",
            ParamValue{std::string("surprise")}},
        {"alpha_fixed",      ParamMutability::HotMutable,
            "Blend coefficient when alpha_source='fixed' (clamped strict [0,1])",
            ParamValue{0.0}},
        {"alpha_smoothing",  ParamMutability::HotMutable,
            "EMA rate for α update toward α_target",
            ParamValue{0.05}},
        {"alpha_min",        ParamMutability::HotMutable,
            "Hard floor on α under surprise-driven mode "
            "(operator override in fixed mode bypasses this)",
            ParamValue{0.0}},
        {"alpha_max",        ParamMutability::HotMutable,
            "Hard ceiling on α under surprise-driven mode "
            "(operator override in fixed mode bypasses this)",
            ParamValue{1.0}},
        {"surprise_aggregation", ParamMutability::HotMutable,
            "How per-modality surprise is reduced to a scalar: 'mean' or 'max'",
            ParamValue{std::string("mean")}},
        {"state_visit_alpha", ParamMutability::HotMutable,
            "Phase 6.6.K: per-modality state-visit-EMA decay rate (smaller = longer memory).",
            ParamValue{0.05}},
        {"pathway_alpha_coupling", ParamMutability::HotMutable,
            "Phase 6.6.K: weight in [0,1] applied to predicted-pathway familiarity before max(surprise, familiarity).  Bilateral crossfade configs set 1.0 (full coupling; addresses panic-during-stuck where surprise stays low but agent is wedged via familiar repeated states).  Default 0.0 preserves the 6.6.G surprise-only α path bit-for-bit so isolated surprise-dynamics tests stay valid.  Bypassed entirely when learned_alpha_lr > 0 (the reward correlation subsumes the familiarity heuristic).",
            ParamValue{0.0}},
        {"learned_alpha_lr", ParamMutability::HotMutable,
            "Phase 6.6.N: learning rate for the reward-driven α setpoint.  Per tick, setpoint += lr * (alpha - setpoint) * reward_ema.  When > 0, α_target = setpoint * (1 - surprise) — the setpoint is the slow-time signal, surprise stays as a fast tactical override.  Replaces the 6.6.K familiarity coupling (which drags α to 0 over long runs because every state gets revisited).  Default 0 = off, preserves 6.6.K behaviour.",
            ParamValue{0.0}},
        {"learned_alpha_setpoint_init", ParamMutability::ConstructionOnly,
            "Phase 6.6.N: initial value for the learned α setpoint.  Default 0.5 — a 50/50 mix that lets reward feedback push toward whichever side the env actually rewards.  Set 0.0 for cold-start brain-must-earn-it; 1.0 for brain-by-default-must-prove-otherwise.",
            ParamValue{0.5}},
        {"reward_alpha", ParamMutability::HotMutable,
            "Phase 6.6.N: EMA decay rate for the reward signal driving setpoint updates.",
            ParamValue{0.05}},
        {"alpha_long_alpha", ParamMutability::HotMutable,
            "Phase 6.6.N: EMA decay rate for the slow-time α reference.  The setpoint update is gradient ascent on Cov(α, reward), so we need both α and reward centred at their running means.  Default 0.005 ≈ 200-tick window.",
            ParamValue{0.005}},
        {"reward_weight_hit", ParamMutability::HotMutable,
            "Phase 6.6.N: signed reward credited per events.hit (positive).",
            ParamValue{1.0}},
        {"reward_weight_miss", ParamMutability::HotMutable,
            "Phase 6.6.N: signed reward credited per events.miss (typically negative; less in magnitude than hit since misses are denser than hits).",
            ParamValue{-0.5}},
        {"reward_weight_wall_stuck", ParamMutability::HotMutable,
            "Phase 6.6.N: signed reward credited per events.wall_stuck (typically negative).",
            ParamValue{-0.5}},
        {"boredom_gain", ParamMutability::HotMutable,
            "Phase 6.6.Q boredom-α floor.  Adds boredom_gain * (1 - alpha_long_ema) to the surprise-driven α target.  When α has been chronically low (alpha_long_ema → 0), boredom term → boredom_gain, pushing α up so the brain can act.  When α is healthy, the term decays to ~0.  Self-stabilising; equilibrium α* ≈ (surprise_target + gain) / (1 + gain).  Decoupled from instantaneous surprise — addresses the failure mode where pathway familiarity floors α despite surprise collapsing.  Default 0 = bit-identical to pre-6.6.Q.",
            ParamValue{0.0}},
    };
}

ParamMap FaderController::current_params() const {
    ParamMap m;
    m["consensus_topic"]      = ParamValue{consensus_topic_};
    m["alpha_topic"]          = ParamValue{alpha_topic_};
    m["urgency_topic"]        = ParamValue{urgency_topic_};
    m["alpha_source"]         = ParamValue{alpha_source_};
    m["alpha_fixed"]          = ParamValue{double(alpha_fixed_)};
    m["alpha_smoothing"]      = ParamValue{double(alpha_smoothing_)};
    m["alpha_min"]            = ParamValue{double(alpha_min_)};
    m["alpha_max"]            = ParamValue{double(alpha_max_)};
    m["surprise_aggregation"] = ParamValue{surprise_aggregation_};
    m["state_visit_alpha"]    = ParamValue{double(state_visit_alpha_)};
    m["pathway_alpha_coupling"] = ParamValue{double(pathway_alpha_coupling_)};
    m["learned_alpha_lr"]               = ParamValue{double(learned_alpha_lr_)};
    m["learned_alpha_setpoint_init"]    = ParamValue{double(learned_alpha_setpoint_init_)};
    m["reward_alpha"]                   = ParamValue{double(reward_alpha_)};
    m["reward_weight_hit"]              = ParamValue{double(reward_weight_hit_)};
    m["reward_weight_miss"]             = ParamValue{double(reward_weight_miss_)};
    m["reward_weight_wall_stuck"]       = ParamValue{double(reward_weight_wall_stuck_)};
    m["boredom_gain"]                   = ParamValue{double(boredom_gain_)};
    return m;
}

void FaderController::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("FaderController requires a non-null Bus");

    apply_param(params, "consensus_topic", [&](auto const& v){ consensus_topic_ = get_string(v, "consensus_topic"); });
    apply_param(params, "alpha_topic",     [&](auto const& v){ alpha_topic_     = get_string(v, "alpha_topic"); });
    apply_param(params, "urgency_topic",   [&](auto const& v){ urgency_topic_   = get_string(v, "urgency_topic"); });
    apply_param(params, "alpha_source",    [&](auto const& v){ alpha_source_    = get_string(v, "alpha_source"); });
    apply_param(params, "alpha_fixed",     [&](auto const& v){ alpha_fixed_     = float(get_double(v, "alpha_fixed")); });
    apply_param(params, "action_topic",       [&](auto const& v){ action_topic_       = get_string(v, "action_topic"); });
    apply_param(params, "alpha_chunk_active", [&](auto const& v){ alpha_chunk_active_ = float(get_double(v, "alpha_chunk_active")); });
    apply_param(params, "alpha_chunk_idle",   [&](auto const& v){ alpha_chunk_idle_   = float(get_double(v, "alpha_chunk_idle")); });
    apply_param(params, "alpha_quality_k",        [&](auto const& v){ alpha_quality_k_        = float(get_double(v, "alpha_quality_k")); });
    apply_param(params, "alpha_quality_midpoint", [&](auto const& v){ alpha_quality_midpoint_ = float(get_double(v, "alpha_quality_midpoint")); });
    apply_param(params, "alpha_quality_min",      [&](auto const& v){ alpha_quality_min_      = float(get_double(v, "alpha_quality_min")); });
    apply_param(params, "alpha_quality_max",      [&](auto const& v){ alpha_quality_max_      = float(get_double(v, "alpha_quality_max")); });
    apply_param(params, "alpha_certainty_min",    [&](auto const& v){ alpha_certainty_min_    = std::clamp(float(get_double(v, "alpha_certainty_min")), 0.0f, 1.0f); });
    apply_param(params, "alpha_certainty_max",    [&](auto const& v){ alpha_certainty_max_    = std::clamp(float(get_double(v, "alpha_certainty_max")), 0.0f, 1.0f); });
    apply_param(params, "alpha_smoothing", [&](auto const& v){ alpha_smoothing_ = float(get_double(v, "alpha_smoothing")); });
    apply_param(params, "alpha_min",       [&](auto const& v){ alpha_min_       = float(get_double(v, "alpha_min")); });
    apply_param(params, "alpha_max",       [&](auto const& v){ alpha_max_       = float(get_double(v, "alpha_max")); });
    apply_param(params, "surprise_aggregation", [&](auto const& v){ surprise_aggregation_ = get_string(v, "surprise_aggregation"); });
    apply_param(params, "state_visit_alpha",
        [&](auto const& v){ state_visit_alpha_ = std::clamp(float(get_double(v, "state_visit_alpha")), 0.0f, 1.0f); });
    apply_param(params, "pathway_alpha_coupling",
        [&](auto const& v){ pathway_alpha_coupling_ = std::clamp(float(get_double(v, "pathway_alpha_coupling")), 0.0f, 1.0f); });
    apply_param(params, "learned_alpha_lr",
        [&](auto const& v){ learned_alpha_lr_ = std::max(0.0f, float(get_double(v, "learned_alpha_lr"))); });
    apply_param(params, "learned_alpha_setpoint_init",
        [&](auto const& v){ learned_alpha_setpoint_init_ = std::clamp(float(get_double(v, "learned_alpha_setpoint_init")), 0.0f, 1.0f); });
    apply_param(params, "reward_alpha",
        [&](auto const& v){ reward_alpha_ = std::clamp(float(get_double(v, "reward_alpha")), 0.0f, 1.0f); });
    apply_param(params, "alpha_long_alpha",
        [&](auto const& v){ alpha_long_alpha_ = std::clamp(float(get_double(v, "alpha_long_alpha")), 0.0f, 1.0f); });
    apply_param(params, "reward_weight_hit",
        [&](auto const& v){ reward_weight_hit_ = float(get_double(v, "reward_weight_hit")); });
    apply_param(params, "reward_weight_miss",
        [&](auto const& v){ reward_weight_miss_ = float(get_double(v, "reward_weight_miss")); });
    apply_param(params, "reward_weight_wall_stuck",
        [&](auto const& v){ reward_weight_wall_stuck_ = float(get_double(v, "reward_weight_wall_stuck")); });
    apply_param(params, "boredom_gain",
        [&](auto const& v){ boredom_gain_ = std::max(0.0f, float(get_double(v, "boredom_gain"))); });
    learned_alpha_setpoint_ = learned_alpha_setpoint_init_;
    // alpha_long_ema_ initialised at the setpoint when learned mode is on,
    // else at alpha_min_ so boredom-α immediately credits the bored state at
    // cold start (matches user-observed "α never lifts off floor" failure
    // mode the 6.6.Q gain is designed to fix).
    if (learned_alpha_lr_ > 0.0f)
        alpha_long_ema_ = learned_alpha_setpoint_init_;
    else
        alpha_long_ema_ = alpha_min_;

    if (alpha_source_ != "fixed" && alpha_source_ != "surprise"
        && alpha_source_ != "chunk_gated" && alpha_source_ != "chunk_quality_sigmoid"
        && alpha_source_ != "premotor_certainty")
        throw std::invalid_argument("FaderController: alpha_source must be 'fixed', 'surprise', 'chunk_gated', 'chunk_quality_sigmoid', or 'premotor_certainty'");
    if (surprise_aggregation_ != "mean" && surprise_aggregation_ != "max")
        throw std::invalid_argument("FaderController: surprise_aggregation must be 'mean' or 'max'");
    if (alpha_min_ < 0.0f || alpha_max_ > 1.0f || alpha_min_ > alpha_max_)
        throw std::invalid_argument("FaderController: require 0 ≤ alpha_min ≤ alpha_max ≤ 1");
    if (alpha_smoothing_ < 0.0f || alpha_smoothing_ > 1.0f)
        throw std::invalid_argument("FaderController: alpha_smoothing must be in [0,1]");

    // Initial α: aim at the source's baseline so we don't always start at 0.
    if (alpha_source_ == "fixed") {
        alpha_ = std::clamp(alpha_fixed_, 0.0f, 1.0f);
    } else if (alpha_source_ == "chunk_gated") {
        alpha_ = std::clamp(alpha_chunk_idle_, 0.0f, 1.0f);
    } else if (alpha_source_ == "premotor_certainty") {
        alpha_ = std::clamp(alpha_certainty_min_, 0.0f, 1.0f);   // cold start = uncertain
    } else {
        alpha_ = alpha_min_;   // safe cold-start under surprise mode
    }

    sub_ids_.push_back(bus_->subscribe(consensus_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_consensus(p); }));
    // v5.4.M — Premotor PolicyToken cache for premotor_certainty α source.
    sub_ids_.push_back(bus_->subscribe(topics::kPolicyIntent,
                                       SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_policy(p); }));
    // v5.4 Phase B — chunk-gated α subscribes to ActionOut for chunk_id.
    // Always subscribed (cheap) so alpha_source can be hot-toggled at runtime.
    sub_ids_.push_back(bus_->subscribe(action_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){
            auto a = std::dynamic_pointer_cast<const ActionOut>(p);
            if (!a) return;
            ++action_msgs_received_;
            current_chunk_id_ = a->chunk_id;
            current_chunk_quality_ = a->chunk_quality;
            if (a->chunk_id != -1) ++chunk_active_ticks_;
        }));
    // Phase 6.6.N — events.* subscription drives the learned setpoint
    // via reward correlation.  Subscribed unconditionally (cheap; events
    // are sparse) so the param can be hot-toggled without re-subscribing.
    sub_ids_.push_back(bus_->subscribe("events.", SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ this->handle_event(t, p); }));
    // Urgency subscription is harmless even though tick() doesn't read it
    // yet — keeps the wiring honest for the future panic channel.
    if (!urgency_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(urgency_topic_, SubscriptionKind::Direct,
            [](std::string_view, MessagePtr){ /* reserved */ }));
    }
}

void FaderController::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "alpha_source")        alpha_source_    = get_string(value, k);
    else if (k == "alpha_fixed")         alpha_fixed_     = float(get_double(value, k));
    else if (k == "alpha_chunk_active")  alpha_chunk_active_ = float(get_double(value, k));
    else if (k == "alpha_chunk_idle")    alpha_chunk_idle_   = float(get_double(value, k));
    else if (k == "alpha_quality_k")        alpha_quality_k_        = float(get_double(value, k));
    else if (k == "alpha_quality_midpoint") alpha_quality_midpoint_ = float(get_double(value, k));
    else if (k == "alpha_quality_min")      alpha_quality_min_      = float(get_double(value, k));
    else if (k == "alpha_quality_max")      alpha_quality_max_      = float(get_double(value, k));
    else if (k == "alpha_certainty_min")    alpha_certainty_min_    = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "alpha_certainty_max")    alpha_certainty_max_    = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "alpha_smoothing")     alpha_smoothing_ = float(get_double(value, k));
    else if (k == "alpha_min")           alpha_min_       = float(get_double(value, k));
    else if (k == "alpha_max")           alpha_max_       = float(get_double(value, k));
    else if (k == "surprise_aggregation")surprise_aggregation_ = get_string(value, k);
    else if (k == "state_visit_alpha")   state_visit_alpha_      = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "pathway_alpha_coupling") pathway_alpha_coupling_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "learned_alpha_lr")    learned_alpha_lr_       = std::max(0.0f, float(get_double(value, k)));
    else if (k == "reward_alpha")        reward_alpha_           = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "alpha_long_alpha")    alpha_long_alpha_       = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "reward_weight_hit")        reward_weight_hit_       = float(get_double(value, k));
    else if (k == "reward_weight_miss")       reward_weight_miss_      = float(get_double(value, k));
    else if (k == "reward_weight_wall_stuck") reward_weight_wall_stuck_= float(get_double(value, k));
    else if (k == "boredom_gain")        boredom_gain_           = std::max(0.0f, float(get_double(value, k)));
    else if (k == "consensus_topic" || k == "alpha_topic" || k == "urgency_topic"
          || k == "learned_alpha_setpoint_init")
        throw std::invalid_argument("FaderController param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("FaderController: unknown param '" + k + "'");
}

void FaderController::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;
    // topic shape: "events.<name>" — derive name by stripping prefix.
    constexpr std::string_view prefix = "events.";
    if (topic.size() <= prefix.size()) return;
    std::string_view name = topic.substr(prefix.size());
    float weight = 0.0f;
    if      (name == "hit")        weight = reward_weight_hit_;
    else if (name == "miss")       weight = reward_weight_miss_;
    else if (name == "wall_stuck") weight = reward_weight_wall_stuck_;
    else                            return;   // unknown event class — ignore
    last_reward_signal_ += weight * float(e->intensity);
    // last_reward_signal_ accumulates within a tick; flushed into
    // reward_ema in tick() and zeroed.
}

// v5.4.M — Premotor PolicyToken handler.  Caches last_entropy +
// intent_distribution.size() for the premotor_certainty α source.
void FaderController::handle_policy(MessagePtr payload) {
    ++policy_msgs_received_;   // debug — verify subscription fires
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!pt) return;
    last_premotor_entropy_   = pt->entropy;
    last_premotor_n_intents_ = int(pt->intent_distribution.size());
}

void FaderController::handle_consensus(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto c = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!c) return;
    last_consensus_ = c;
    // Phase 6.6.K — bump state-visit EMA for each (modality, observed
    // winner_id) on the consensus.  Reads are O(modalities) per tick.
    for (auto const& [topic, wid] : c->winner_ids_by_modality) {
        if (wid < 0) continue;
        float& ema = state_visit_ema_[topic][wid];
        ema = (1.0f - state_visit_alpha_) * ema + state_visit_alpha_ * 1.0f;
    }
}

float FaderController::aggregate_surprise(
    std::unordered_map<std::string, float> const& m) const {
    if (m.empty()) return 0.0f;
    if (surprise_aggregation_ == "max") {
        float best = 0.0f;
        for (auto const& [_, v] : m) if (v > best) best = v;
        return best;
    }
    // mean
    float sum = 0.0f;
    for (auto const& [_, v] : m) sum += v;
    return sum / float(m.size());
}

float FaderController::compute_alpha_target() const {
    if (alpha_source_ == "fixed") {
        // Operator override: in fixed mode the user/UI is explicitly
        // setting α, so we clamp to strict [0, 1] rather than the
        // alpha_min/max safety floors (which exist to keep both sides
        // slightly contributing under the auto/surprise regime).
        return std::clamp(alpha_fixed_, 0.0f, 1.0f);
    }
    if (alpha_source_ == "chunk_gated") {
        // v5.4 Phase B — when a chunk is replaying (ActionOut.chunk_id != -1),
        // brain has high-certainty plan → α=alpha_chunk_active (default 1.0,
        // suppress reflex).  Otherwise α=alpha_chunk_idle (default 0.5,
        // balanced blend).  Hand-tuned diagnostic of "suppress reflex when
        // brain knows what it's doing"; informs adaptive design where α
        // would be a continuous function of entry_match_certainty.
        float t = (current_chunk_id_ != -1) ? alpha_chunk_active_ : alpha_chunk_idle_;
        return std::clamp(t, 0.0f, 1.0f);
    }
    if (alpha_source_ == "premotor_certainty") {
        // v5.4.M — α modulated by Premotor's instantaneous softmax
        // peakedness.  When Premotor's policy is committed to one
        // intent (low entropy), give the brain more authority.  When
        // distribution is near-uniform (high entropy = uncertain),
        // let reflex chemotaxis dominate.
        //
        // α = α_min + (α_max - α_min) × (1 - normalized_entropy)
        // normalized_entropy = entropy / ln(N) ∈ [0, 1]
        //
        // Cold start (no Premotor publish yet): treat as max-entropy
        // (uncertain) → α=α_min, reflex-safe.
        if (last_premotor_n_intents_ <= 1) {
            return std::clamp(alpha_certainty_min_, 0.0f, 1.0f);
        }
        float h_max    = std::log(float(last_premotor_n_intents_));
        float h_norm   = (h_max > 1e-6f) ?
                         std::clamp(last_premotor_entropy_ / h_max, 0.0f, 1.0f)
                         : 0.0f;
        float certainty = 1.0f - h_norm;
        float t = alpha_certainty_min_
                  + (alpha_certainty_max_ - alpha_certainty_min_) * certainty;
        return std::clamp(t, 0.0f, 1.0f);
    }
    if (alpha_source_ == "chunk_quality_sigmoid") {
        // v5.4 Phase F (Proposal B) — continuous α modulated by chunk
        // quality.  σ(z) = 1/(1 + exp(-z)).  At quality=midpoint → σ=0.5
        // → α at midpoint of [α_min, α_max].  Steepness k controls how
        // sharply α flips around the midpoint.
        float q     = current_chunk_quality_;
        float z     = alpha_quality_k_ * (q - alpha_quality_midpoint_);
        float sig   = 1.0f / (1.0f + std::exp(-z));
        float t     = alpha_quality_min_ + (alpha_quality_max_ - alpha_quality_min_) * sig;
        return std::clamp(t, 0.0f, 1.0f);
    }
    // surprise mode
    if (!last_consensus_) return alpha_min_;     // cold start = reflex-safe
    // Surprise must be active for α to lift off alpha_min — pathway
    // data alone shouldn't push toward brain when the voter isn't
    // tracking prediction error (surprise_gain == 0 → no evidence the
    // brain's predictions matter).  Preserves the bit-identical
    // safeguard in the 6.6.F E2E tests.
    if (last_consensus_->surprise_ema.empty()) return alpha_min_;
    float s = aggregate_surprise(last_consensus_->surprise_ema);
    // Phase 6.6.N — when learned-α mode is engaged, the slow-time
    // signal is the reward-driven setpoint (subsumes 6.6.K
    // familiarity since reward correlation captures whatever
    // familiarity should mean for THIS env's reward landscape).
    // Surprise stays as a fast tactical override: high surprise
    // briefly drops α even if the setpoint is high.
    if (learned_alpha_lr_ > 0.0f) {
        float target = std::clamp(learned_alpha_setpoint_, 0.0f, 1.0f)
                     * (1.0f - std::clamp(s, 0.0f, 1.0f));
        return std::clamp(target, alpha_min_, alpha_max_);
    }
    // Phase 6.6.K — effective signal is max(surprise, familiarity*coupling)
    // so a stuck-but-accurate-prediction (low surprise, high familiarity)
    // still drops α and lets reflexes break the loop.
    float effective = std::max(s, familiarity_scalar_ * pathway_alpha_coupling_);
    float target = 1.0f - std::clamp(effective, 0.0f, 1.0f);
    // Phase 6.6.Q — boredom-α floor.  When alpha_long_ema is chronically
    // low, lift the target by gain*(1 - alpha_long_ema).  Equilibrium:
    // alpha_long*  ≈ (target_no_boredom + gain) / (1 + gain).  At gain=0
    // this is a no-op (bit-identical pre-6.6.Q).
    target += boredom_gain_ * (1.0f - std::clamp(alpha_long_ema_, 0.0f, 1.0f));
    return std::clamp(target, alpha_min_, alpha_max_);
}

void FaderController::tick(uint64_t tick_id) {
    surprise_scalar_ = (alpha_source_ == "surprise" && last_consensus_)
        ? aggregate_surprise(last_consensus_->surprise_ema) : 0.0f;
    // Phase 6.6.K — compute pathway familiarity from this tick's
    // predicted_pathways + the running state_visit_ema.  Aggregation
    // mirrors surprise (mean by default, max if configured) so the two
    // signals combine consistently.
    familiarity_scalar_ = 0.0f;
    if (alpha_source_ == "surprise"
        && pathway_alpha_coupling_ > 0.0f
        && last_consensus_
        && !last_consensus_->predicted_pathways.empty()) {
        if (surprise_aggregation_ == "max") {
            float best = 0.0f;
            for (auto const& [topic, pathway] : last_consensus_->predicted_pathways) {
                auto vit = state_visit_ema_.find(topic);
                for (int pid : pathway) {
                    float f = 0.0f;
                    if (vit != state_visit_ema_.end()) {
                        auto eit = vit->second.find(pid);
                        if (eit != vit->second.end()) f = eit->second;
                    }
                    if (f > best) best = f;
                }
            }
            familiarity_scalar_ = std::clamp(best, 0.0f, 1.0f);
        } else {
            float sum = 0.0f;
            int   n   = 0;
            for (auto const& [topic, pathway] : last_consensus_->predicted_pathways) {
                auto vit = state_visit_ema_.find(topic);
                for (int pid : pathway) {
                    float f = 0.0f;
                    if (vit != state_visit_ema_.end()) {
                        auto eit = vit->second.find(pid);
                        if (eit != vit->second.end()) f = eit->second;
                    }
                    sum += f;
                    ++n;
                }
            }
            if (n > 0)
                familiarity_scalar_ = std::clamp(sum / float(n), 0.0f, 1.0f);
        }
    }
    // Phase 6.6.N — event-gated setpoint update (corrected from earlier
    // covariance-baseline draft).
    //
    //   alpha_long  ← (1-aα)*alpha_long + aα*α        (every tick)
    //   on event tick:
    //     setpoint += lr * (α - alpha_long) * signed_event_weight
    //   on non-event tick: no update
    //
    // Why event-gated and not per-tick covariance: events are sparse
    // (Cell ~3 hits / 90s), so a leaky reward-EMA baseline turns every
    // non-event tick into a small non-zero gradient sample.  With the
    // overwhelmingly-many non-event ticks integrated, the bias drowns
    // the actual hit-correlated signal.  The honest policy-gradient
    // form is "credit α-deviation only at the moments env signals
    // reward."  alpha_long still updates every tick so the deviation
    // reference is current.
    //
    // reward_ema_ retained as diagnostic (tracks long-run reward
    // density for telemetry) but NOT used in the gradient.
    //
    // Skipped entirely when learned_alpha_lr is 0 — preserves 6.6.K
    // bit-for-bit.
    if (learned_alpha_lr_ > 0.0f) {
        alpha_long_ema_ = (1.0f - alpha_long_alpha_) * alpha_long_ema_
                        + alpha_long_alpha_ * alpha_;
        // Diagnostic only:
        reward_ema_ = (1.0f - reward_alpha_) * reward_ema_
                    + reward_alpha_ * last_reward_signal_;
        if (last_reward_signal_ != 0.0f) {
            float a_dev = alpha_ - alpha_long_ema_;
            float delta = learned_alpha_lr_ * a_dev * last_reward_signal_;
            learned_alpha_setpoint_ = std::clamp(
                learned_alpha_setpoint_ + delta, 0.0f, 1.0f);
        }
    } else if (boredom_gain_ > 0.0f) {
        // 6.6.Q: keep alpha_long_ema fresh even when the 6.6.N learned
        // setpoint is off, so compute_alpha_target's boredom term is well
        // defined.  Same EMA rate (alpha_long_alpha_) as the learned-mode
        // path; sharing the param keeps tunability consistent.
        alpha_long_ema_ = (1.0f - alpha_long_alpha_) * alpha_long_ema_
                        + alpha_long_alpha_ * alpha_;
    }
    last_reward_signal_ = 0.0f;
    last_boredom_ = boredom_gain_ * (1.0f - std::clamp(alpha_long_ema_, 0.0f, 1.0f));

    alpha_target_ = compute_alpha_target();
    alpha_ = (1.0f - alpha_smoothing_) * alpha_ + alpha_smoothing_ * alpha_target_;

    auto fs = std::make_shared<FaderState>();
    fs->tick_id          = tick_id;
    fs->producer_id      = id_.empty() ? std::string("fader_controller") : id_;
    fs->alpha            = alpha_;
    fs->alpha_target     = alpha_target_;
    fs->surprise_scalar  = surprise_scalar_;
    // brain_seen / reflex_seen / *_accel are per-MotorFader instance state
    // and not visible to the controller.  They stay at their default 0 /
    // false and will be sourced directly from MotorFader white-box state
    // in step 6 (meter widget polish).  Preserving the struct shape keeps
    // existing OgmaBrain.get_motor_fader_state() callers ABI-stable.
    fs->source           = alpha_source_;
    bus_->publish(alpha_topic_, fs);
    ++publish_count_;
}

// ---------------------------------------------------------------------------
// Snapshot / restore (UI-dev W3.2 Tier A)
// ---------------------------------------------------------------------------

nlohmann::json FaderController::snapshot_state() const {
    nlohmann::json visit = nlohmann::json::object();
    for (auto const& [mod, inner] : state_visit_ema_) {
        nlohmann::json row = nlohmann::json::object();
        for (auto const& [node, ema] : inner) row[std::to_string(node)] = ema;
        visit[mod] = row;
    }
    nlohmann::json consensus = nullptr;
    if (last_consensus_) {
        nlohmann::json surp = nlohmann::json::object();
        for (auto const& [k, v] : last_consensus_->surprise_ema) surp[k] = v;
        nlohmann::json paths = nlohmann::json::object();
        for (auto const& [k, ids] : last_consensus_->predicted_pathways) {
            nlohmann::json a = nlohmann::json::array();
            for (auto v : ids) a.push_back(v);
            paths[k] = a;
        }
        nlohmann::json winners = nlohmann::json::object();
        for (auto const& [k, id] : last_consensus_->winner_ids_by_modality)
            winners[k] = id;
        consensus = nlohmann::json{
            {"surprise_ema",          surp},
            {"predicted_pathways",    paths},
            {"winner_ids_by_modality",winners},
        };
    }
    return nlohmann::json{
        {"version",                 1},
        {"alpha",                   alpha_},
        {"alpha_target",            alpha_target_},
        {"surprise_scalar",         surprise_scalar_},
        {"familiarity_scalar",      familiarity_scalar_},
        {"publish_count",           publish_count_},
        {"learned_alpha_setpoint",  learned_alpha_setpoint_},
        {"reward_ema",              reward_ema_},
        {"alpha_long_ema",          alpha_long_ema_},
        {"learned_warmed_up",       learned_warmed_up_},
        {"last_reward_signal",      last_reward_signal_},
        {"last_boredom",            last_boredom_},
        {"state_visit_ema",         visit},
        {"last_consensus",          consensus},
    };
}

void FaderController::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("FaderController::restore_state: unknown version " +
                                 std::to_string(version));
    }
    alpha_                   = s.value("alpha",                   alpha_);
    alpha_target_            = s.value("alpha_target",            alpha_target_);
    surprise_scalar_         = s.value("surprise_scalar",         surprise_scalar_);
    familiarity_scalar_      = s.value("familiarity_scalar",      familiarity_scalar_);
    publish_count_           = s.value("publish_count",           publish_count_);
    learned_alpha_setpoint_  = s.value("learned_alpha_setpoint",  learned_alpha_setpoint_);
    reward_ema_              = s.value("reward_ema",              reward_ema_);
    alpha_long_ema_          = s.value("alpha_long_ema",          alpha_long_ema_);
    learned_warmed_up_       = s.value("learned_warmed_up",       learned_warmed_up_);
    last_reward_signal_      = s.value("last_reward_signal",      last_reward_signal_);
    last_boredom_            = s.value("last_boredom",            last_boredom_);
    state_visit_ema_.clear();
    if (s.contains("state_visit_ema") && s["state_visit_ema"].is_object()) {
        for (auto it1 = s["state_visit_ema"].begin(); it1 != s["state_visit_ema"].end(); ++it1) {
            auto& inner = state_visit_ema_[it1.key()];
            for (auto it2 = it1.value().begin(); it2 != it1.value().end(); ++it2)
                inner[std::stoi(it2.key())] = it2.value().get<float>();
        }
    }
    last_consensus_.reset();
    if (s.contains("last_consensus") && !s["last_consensus"].is_null()) {
        auto t = std::make_shared<ConsensusToken>();
        auto const& c = s["last_consensus"];
        if (c.contains("surprise_ema") && c["surprise_ema"].is_object())
            for (auto it = c["surprise_ema"].begin(); it != c["surprise_ema"].end(); ++it)
                t->surprise_ema[it.key()] = it.value().get<float>();
        if (c.contains("predicted_pathways") && c["predicted_pathways"].is_object())
            for (auto it = c["predicted_pathways"].begin(); it != c["predicted_pathways"].end(); ++it) {
                auto& v = t->predicted_pathways[it.key()];
                for (auto const& x : it.value()) v.push_back(x.get<int>());
            }
        if (c.contains("winner_ids_by_modality") && c["winner_ids_by_modality"].is_object())
            for (auto it = c["winner_ids_by_modality"].begin(); it != c["winner_ids_by_modality"].end(); ++it)
                t->winner_ids_by_modality[it.key()] = it.value().get<int>();
        last_consensus_ = t;
    }
}

} // namespace ogma
