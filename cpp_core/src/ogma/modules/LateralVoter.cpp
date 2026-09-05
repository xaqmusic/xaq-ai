#include "ogma/modules/LateralVoter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>
#include <vector>

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
    throw std::invalid_argument("LateralVoter param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("LateralVoter param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("LateralVoter param '" + key + "' must be bool");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("LateralVoter param '" + key + "' must be string");
}

} // namespace

LateralVoter::LateralVoter()  = default;
LateralVoter::~LateralVoter() = default;

std::string_view LateralVoter::type_name() const { return "LateralVoter"; }

std::vector<TopicSpec> LateralVoter::input_topics() const {
    return {
        TopicSpec{input_pattern_, std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kNeuroState, std::type_index(typeid(NeuroState)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> LateralVoter::output_topics() const {
    std::vector<TopicSpec> out{
        TopicSpec{output_topic_, std::type_index(typeid(ConsensusToken))}
    };
    if (!publish_as_reality_topic_.empty()) {
        out.push_back(TopicSpec{publish_as_reality_topic_,
                                 std::type_index(typeid(RealityToken))});
    }
    return out;
}

ParamSchema LateralVoter::params_schema() const {
    return {
        {"level",                ParamMutability::ConstructionOnly, "Level identifier (publishes consensus.<level>)", ParamValue{int64_t{0}}},
        {"input_pattern",        ParamMutability::ConstructionOnly, "Trailing-dot prefix subscribed",                  ParamValue{std::string("reality.")}},
        {"input_exclude",        ParamMutability::ConstructionOnly, "Optional prefix(es) to ignore among matches of input_pattern (e.g. exclude raw EPM outputs feeding mid-voters).  Phase 6.6.M: accepts either a single string (legacy) or a list of strings for multi-exclude (e.g. bilateral whisker split needs two prefixes excluded from the top voter).", ParamValue{std::string("")}},
        {"trust_epsilon",        ParamMutability::HotMutable,       "Smoothing in 1/(tle + ε)",                        ParamValue{0.05}},
        {"group_balance",        ParamMutability::HotMutable,       "Modality-group balanced trust",                   ParamValue{true}},
        {"softmax_temperature",  ParamMutability::HotMutable,       "Trust softmax temperature",                       ParamValue{1.0}},
        {"priority_group",       ParamMutability::HotMutable,       "Tie-break group for active modality selection",   ParamValue{std::string("proprio")}},
        {"association_enabled",  ParamMutability::HotMutable,       "Enable Hebbian association_matrix updates",       ParamValue{false}},
        {"association_decay",    ParamMutability::HotMutable,       "Per-tick decay on association entries",           ParamValue{0.9999}},
        {"association_max_size", ParamMutability::HotMutable,       "LRU cap on association entries",                  ParamValue{int64_t{10000}}},
        {"novelty_threshold",    ParamMutability::HotMutable,       "fused_tle threshold for higher-level mitosis",    ParamValue{0.35}},
        {"informativeness_gain", ParamMutability::HotMutable,       "2026-06-20: gate trust by channel INFORMATIVENESS to escape the trap that fools BOTH reliability measures — a flat/degenerate channel is trivially predictable (TLE≈0) AND trivially reconstructed (quant_error≈0) → it maxes precision → over-trusted. When >0, raw trust = precision(1/(quant_error+ε)) × (info_floor + (1−info_floor)·ci)^gain, where ci = crystallized informativeness from baked_count ((baked−1)/(info_baked_ref−1), clamped): a degenerate channel bakes exactly ONE node → ci=0 → EXACTLY zero trust (precision can't resurrect it); a structured channel bakes many → ci=1. Uses quant_error not dual-TLE (drops the transition_surp term that perversely penalises channels tracking a changing world). 0 = legacy 1/(tle+ε) (bit-identical).", ParamValue{0.0}},
        {"info_floor",           ParamMutability::HotMutable,       "Additive floor on informativeness. 0 (default) = a dead channel (ci=0) gets EXACTLY zero trust. Raise (e.g. 0.1) only to keep a momentarily-uninformative channel partially in play.", ParamValue{0.0}},
        {"info_baked_ref",       ParamMutability::HotMutable,       "baked_count at which informativeness saturates to 1 (ci=(baked−1)/(ref−1)). Default 3: baked≤1→0, baked≥3→1.", ParamValue{3.0}},
        {"surprise_gain",        ParamMutability::HotMutable,       "Phase 6.6.E: scale of predicted_pathway surprise modulation on raw trust (0=off; bit-identical to pre-6.6.E)", ParamValue{0.0}},
        {"surprise_alpha",       ParamMutability::HotMutable,       "Phase 6.6.E: per-modality EMA rate for surprise smoothing", ParamValue{0.1}},
        {"surprise_floor",       ParamMutability::HotMutable,       "Phase 6.6.E: minimum trust-scale floor (avoid zeroing under sustained surprise)", ParamValue{0.05}},
        {"surprise_kind",        ParamMutability::HotMutable,       "Phase 6.6.H: 'binary' (0/1 node-id match; default + same as 6.6.E) or 'embedding' (cosine-distance surprise (1-cos)/2 in [0,1])", ParamValue{std::string("binary")}},
        {"surprise_calibrate",   ParamMutability::HotMutable,       "Phase 6.6.J: when true (default), the surfaced surprise_ema applies Bayesian shrinkage toward a 0.5 prior with strength 1/surprise_alpha so cold-start EPMs don't publish 'perfect predictor' immediately.  Trust modulator still uses the raw EMA.", ParamValue{true}},
        {"activity_gain",        ParamMutability::HotMutable,
            "Kalman-lessons Stage 2: ACTIVITY term (doctrine 2.3's observability proxy).  Raw trust is multiplied by "
            "activity^gain where activity = EMA of the channel's latent displacement / its own decaying peak, in [0,1].  "
            "A channel that stops moving (dead sensor, occluded camera, stale republished token) loses trust in EMA time "
            "instead of becoming the most trusted because it is trivially predictable.  0 (default) = byte-identical; "
            "-1 = the wrong-sign control.", ParamValue{0.0}},
        {"activity_alpha",       ParamMutability::HotMutable, "EMA rate of the per-channel latent displacement.", ParamValue{0.1}},
        {"activity_peak_decay",  ParamMutability::HotMutable, "Per-tick decay of the running peak the activity is normalised by (0.999 = ~1000-tick memory).", ParamValue{0.999}},
        {"activity_floor",       ParamMutability::HotMutable, "Lowest activity factor applied (keeps activity^gain finite for gain < 0).", ParamValue{0.001}},
        {"publish_as_reality_topic", ParamMutability::ConstructionOnly, "Phase-6.0.c: also publish fused output as a RealityToken to this topic (empty=disabled)", ParamValue{std::string("")}},
        {"publish_consensus",        ParamMutability::ConstructionOnly, "Phase-6.0.c: false = suppress consensus.<level> publish (mid-voters whose only consumer is via publish_as_reality_topic)", ParamValue{true}},
        {"master_seed",          ParamMutability::ConstructionOnly, "RNG namespace seed",                              ParamValue{int64_t{0}}},
    };
}

void LateralVoter::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("LateralVoter requires a non-null Bus");

    apply_param(params, "level",               [&](auto const& v){ level_               = int(get_int(v, "level")); });
    apply_param(params, "input_pattern",       [&](auto const& v){ input_pattern_       = get_string(v, "input_pattern"); });
    // Phase 6.6.M: input_exclude accepts either a string (legacy) or a
    // vector<string> (multi-exclude).  Branch on the variant alternative.
    apply_param(params, "input_exclude", [&](ParamValue const& v){
        input_excludes_.clear();
        if (auto p = std::get_if<std::vector<std::string>>(&v)) {
            for (auto const& s : *p) {
                if (!s.empty()) input_excludes_.push_back(s);
            }
        } else if (auto p = std::get_if<std::string>(&v)) {
            if (!p->empty()) input_excludes_.push_back(*p);
        } else {
            throw std::invalid_argument(
                "LateralVoter: input_exclude must be string or list of strings");
        }
    });
    apply_param(params, "trust_epsilon",       [&](auto const& v){ trust_epsilon_       = float(get_double(v, "trust_epsilon")); });
    apply_param(params, "activity_gain",       [&](auto const& v){ activity_gain_       = float(get_double(v, "activity_gain")); });
    apply_param(params, "activity_alpha",      [&](auto const& v){ activity_alpha_      = float(get_double(v, "activity_alpha")); });
    apply_param(params, "activity_peak_decay", [&](auto const& v){ activity_peak_decay_ = float(get_double(v, "activity_peak_decay")); });
    apply_param(params, "activity_floor",      [&](auto const& v){ activity_floor_      = float(get_double(v, "activity_floor")); });
    apply_param(params, "informativeness_gain",[&](auto const& v){ informativeness_gain_= float(get_double(v, "informativeness_gain")); });
    apply_param(params, "info_floor",          [&](auto const& v){ info_floor_          = float(get_double(v, "info_floor")); });
    apply_param(params, "info_baked_ref",      [&](auto const& v){ info_baked_ref_      = float(get_double(v, "info_baked_ref")); });
    apply_param(params, "group_balance",       [&](auto const& v){ group_balance_       = get_bool(v, "group_balance"); });
    apply_param(params, "softmax_temperature", [&](auto const& v){ softmax_temperature_ = float(get_double(v, "softmax_temperature")); });
    apply_param(params, "priority_group",      [&](auto const& v){ priority_group_      = get_string(v, "priority_group"); });
    apply_param(params, "association_enabled", [&](auto const& v){ association_enabled_ = get_bool(v, "association_enabled"); });
    apply_param(params, "association_decay",   [&](auto const& v){ association_decay_   = float(get_double(v, "association_decay")); });
    apply_param(params, "association_max_size",[&](auto const& v){ association_max_size_= get_int(v, "association_max_size"); });
    apply_param(params, "novelty_threshold",   [&](auto const& v){ novelty_threshold_   = float(get_double(v, "novelty_threshold")); });
    apply_param(params, "surprise_gain",       [&](auto const& v){ surprise_gain_       = float(get_double(v, "surprise_gain")); });
    apply_param(params, "surprise_alpha",      [&](auto const& v){ surprise_alpha_      = float(get_double(v, "surprise_alpha")); });
    apply_param(params, "surprise_floor",      [&](auto const& v){ surprise_floor_      = float(get_double(v, "surprise_floor")); });
    apply_param(params, "surprise_kind",       [&](auto const& v){ surprise_kind_       = get_string(v, "surprise_kind"); });
    apply_param(params, "surprise_calibrate",  [&](auto const& v){ surprise_calibrate_  = get_bool(v, "surprise_calibrate"); });
    if (surprise_kind_ != "binary" && surprise_kind_ != "embedding")
        throw std::invalid_argument("LateralVoter: surprise_kind must be 'binary' or 'embedding'");
    apply_param(params, "publish_as_reality_topic", [&](auto const& v){ publish_as_reality_topic_ = get_string(v, "publish_as_reality_topic"); });
    apply_param(params, "publish_consensus",        [&](auto const& v){ publish_consensus_        = get_bool(v, "publish_consensus"); });
    apply_param(params, "master_seed",         [&](auto const& v){ master_seed_         = uint64_t(get_int(v, "master_seed")); });

    output_topic_ = std::string("consensus.") + std::to_string(level_);

    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(input_pattern_, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_input(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_neuro(t, p); }));
}

void LateralVoter::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "trust_epsilon")        trust_epsilon_       = float(get_double(value, k));
    else if (k == "activity_gain")        activity_gain_       = float(get_double(value, k));
    else if (k == "activity_alpha")       activity_alpha_      = float(get_double(value, k));
    else if (k == "activity_peak_decay")  activity_peak_decay_ = float(get_double(value, k));
    else if (k == "activity_floor")       activity_floor_      = float(get_double(value, k));
    else if (k == "informativeness_gain") informativeness_gain_= float(get_double(value, k));
    else if (k == "info_floor")           info_floor_          = float(get_double(value, k));
    else if (k == "info_baked_ref")       info_baked_ref_      = float(get_double(value, k));
    else if (k == "group_balance")        group_balance_       = get_bool(value, k);
    else if (k == "softmax_temperature")  softmax_temperature_ = float(get_double(value, k));
    else if (k == "priority_group")       priority_group_      = get_string(value, k);
    else if (k == "association_enabled")  association_enabled_ = get_bool(value, k);
    else if (k == "association_decay")    association_decay_   = float(get_double(value, k));
    else if (k == "association_max_size") association_max_size_= get_int(value, k);
    else if (k == "novelty_threshold")    novelty_threshold_   = float(get_double(value, k));
    else if (k == "surprise_gain")        surprise_gain_       = float(get_double(value, k));
    else if (k == "surprise_alpha")       surprise_alpha_      = float(get_double(value, k));
    else if (k == "surprise_floor")       surprise_floor_      = float(get_double(value, k));
    else if (k == "surprise_kind")        {
        auto next = get_string(value, k);
        if (next != "binary" && next != "embedding")
            throw std::invalid_argument("LateralVoter: surprise_kind must be 'binary' or 'embedding'");
        surprise_kind_ = next;
    }
    else if (k == "surprise_calibrate")   surprise_calibrate_  = get_bool(value, k);
    else if (k == "level" || k == "input_pattern" || k == "master_seed"
             || k == "publish_as_reality_topic" || k == "input_exclude"
             || k == "publish_consensus")
        throw std::invalid_argument("LateralVoter param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("LateralVoter: unknown param '" + k + "'");
}

void LateralVoter::handle_input(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;
    // Phase-6.0.c / 6.6.M: optional exclusion(s) so a top voter can
    // "see reality.* except the raw EPM outputs that are already
    // feeding mid-voters."  Without this the top voter would double-
    // count those EPMs (raw + via mid-voter fused).  Loops the vector
    // so multiple side-resolved mid-voters can each suppress their
    // own raw inputs (bilateral whisker split: two prefixes both
    // excluded from voter_0).
    for (auto const& excl : input_excludes_) {
        if (topic.size() >= excl.size()
            && topic.compare(0, excl.size(), excl) == 0)
            return;
    }
    auto& slot = pending_[std::string(topic)];
    slot.topic = std::string(topic);
    slot.token = rt;
}

void LateralVoter::handle_neuro(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto n = std::dynamic_pointer_cast<const NeuroState>(payload);
    if (!n) return;
    dopamine_ = n->dopamine;
}

std::pair<std::string, std::string>
LateralVoter::parse_group_member(std::string const& topic) const {
    if (topic.size() < input_pattern_.size() ||
        topic.compare(0, input_pattern_.size(), input_pattern_) != 0)
        return {"", ""};
    std::string rest = topic.substr(input_pattern_.size());
    auto dot = rest.find('.');
    if (dot == std::string::npos) return {rest, ""};
    return {rest.substr(0, dot), rest.substr(dot + 1)};
}

float LateralVoter::channel_informativeness(RealityToken const& tok) const {
    // CRYSTALLIZED informativeness = how many DISTINCT concepts a channel has
    // STABLY learned (baked GNG nodes).  This is the flicker-immune signal that
    // escapes the trap that fools BOTH reliability measures (TLE and QE): a
    // constant/degenerate channel is trivially predictable (TLE≈0) AND trivially
    // reconstructed (QE≈0) → it maxes precision → over-trusted.  But it bakes
    // exactly ONE node ("the constant"), no matter how many transient raw nodes
    // flicker (which fooled a winner-id-entropy measure).  A channel tracking
    // real structure bakes MANY.  (Operator insight, 2026-06-20; confirmed:
    // dead energy channel baked=1 vs scent/loom baked=10-11.)
    //   baked ≤ 1               → 0  (one concept = no discrimination)
    //   baked ≥ info_baked_ref  → 1
    // baked == 0 (not yet crystallized) also → 0; at bootstrap ALL channels are
    // ~0 so the voter falls back to uniform trust (the sum→0 path below), and
    // channels ramp in as they bake.  A dead channel is pinned at baked=1 → 0
    // forever → excluded.
    float ref  = std::max(2.0f, info_baked_ref_);
    float frac = (float(tok.baked_count) - 1.0f) / (ref - 1.0f);
    return std::clamp(frac, 0.0f, 1.0f);
}

void LateralVoter::publish_placeholder(uint64_t tick_id) {
    auto out = std::make_shared<ConsensusToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("voter") : id_;
    out->level       = level_;
    out->fused_tle   = 0.0f;
    if (publish_consensus_)
        bus_->publish(output_topic_, out);
    prev_token_ = out;
}

void LateralVoter::tick(uint64_t tick_id) {
    // Filter pending inputs to those with a real winner (non-bootstrap).
    std::vector<std::pair<std::string, std::shared_ptr<const RealityToken>>> active;
    active.reserve(pending_.size());
    for (auto const& [topic, p] : pending_) {
        if (p.token && p.token->winner_id >= 0 && p.token->latent.size() > 0)
            active.emplace_back(topic, p.token);
    }

    if (active.empty()) {
        // No useful inputs.  Republish previous token with new tick_id
        // (Invariant 7), or placeholder on the very first tick.
        if (prev_token_) {
            auto out = std::make_shared<ConsensusToken>(*prev_token_);
            out->tick_id     = tick_id;
            out->producer_id = id_.empty() ? std::string("voter") : id_;
            if (publish_consensus_)
                bus_->publish(output_topic_, out);
            prev_token_ = out;
        } else {
            publish_placeholder(tick_id);
        }
        pending_.clear();
        return;
    }

    // Phase 6.6.E / 6.6.H: predicted_pathway surprise update.
    //
    // For each modality with a buffered prediction from the previous tick,
    // compare to this tick's observed winner and EMA-smooth a surprise
    // scalar in [0, 1].  Then store this tick's predicted_pathway[0] for
    // next-tick comparison and update the local embedding cache so future
    // 'embedding' lookups can find the just-observed prototype.  Skipped
    // entirely when surprise_gain_ == 0 (preserves bit-identity with
    // pre-6.6.E voter).
    //
    // 6.6.H — embedding-distance surprise.  When surprise_kind_ ==
    // "embedding" the per-tick surprise scalar is computed as
    //   s = clamp((1 - cos(E_pred, E_obs)) / 2, 0, 1)
    // where E_pred is the cached prototype for the predicted node id and
    // E_obs is the current token's winner_prototype.  Falls back to
    // binary surprise = 1.0 when E_pred isn't in the cache (predicted
    // node was pruned or never seen by this voter).  Behaviourally the
    // cosine form gives the FaderController a smooth gradient — α
    // responds to *how wrong* the prediction was (clearly different
    // node → high distance → high surprise; near-identical centroid →
    // low distance → low surprise) instead of a step function on
    // node-id equality.
    if (surprise_gain_ > 0.0f) {
        for (auto const& [topic, tok] : active) {
            auto pit = last_predicted_next_.find(topic);
            if (pit != last_predicted_next_.end() && pit->second >= 0) {
                float s = 1.0f;   // safe-default for missing-cache fallback
                if (surprise_kind_ == "embedding") {
                    auto cit = embedding_cache_.find(topic);
                    if (cit != embedding_cache_.end()) {
                        auto eit = cit->second.find(pit->second);
                        if (eit != cit->second.end()
                            && tok->winner_prototype.size() == eit->second.size()
                            && tok->winner_prototype.size() > 0) {
                            float dot = eit->second.dot(tok->winner_prototype);
                            float n_p = eit->second.norm();
                            float n_o = tok->winner_prototype.norm();
                            if (n_p > 1e-9f && n_o > 1e-9f) {
                                float cos_sim = dot / (n_p * n_o);
                                cos_sim = std::clamp(cos_sim, -1.0f, 1.0f);
                                s = std::clamp((1.0f - cos_sim) * 0.5f, 0.0f, 1.0f);
                            }
                        }
                    }
                } else {
                    // "binary" — the original 6.6.E behaviour.
                    s = (pit->second == tok->winner_id) ? 0.0f : 1.0f;
                }
                float& ema = surprise_ema_[topic];   // zero-initialised on first sight
                ema = (1.0f - surprise_alpha_) * ema + surprise_alpha_ * s;
                ++prediction_counts_[topic];   // 6.6.J: shrinkage sample-count
            }
            // Buffer this tick's first predicted node for next-tick surprise.
            // Empty pathway (EPM doesn't emit predictions) clears the buffer
            // so a stale prediction can't haunt us forever.
            if (!tok->predicted_pathway.empty()) {
                last_predicted_next_[topic] = tok->predicted_pathway.front();
            } else {
                last_predicted_next_[topic] = -1;
            }
            // Update the local embedding cache for future predicted-node
            // lookups.  We populate from this tick's observation (winner_id
            // → winner_prototype) and evict any pruned ids the EPM
            // reported.  Cheap to maintain even when surprise_kind ==
            // "binary" because the structure stays small (at most one entry
            // per active GNG node per modality).
            if (tok->winner_prototype.size() > 0 && tok->winner_id >= 0) {
                embedding_cache_[topic][tok->winner_id] = tok->winner_prototype;
            }
            if (!tok->pruned_ids.empty()) {
                auto cit = embedding_cache_.find(topic);
                if (cit != embedding_cache_.end()) {
                    for (int pid : tok->pruned_ids) cit->second.erase(pid);
                }
            }
        }
    }

    // Compute raw TLE-inverse trust per input.  Apply Phase 6.6.E surprise
    // modulator if active: sustained mismatch between predicted and observed
    // winners attenuates that modality's contribution down to surprise_floor_.
    std::vector<float> raw(active.size(), 0.0f);
    for (size_t i = 0; i < active.size(); ++i) {
        float r;
        if (informativeness_gain_ > 0.0f) {
            // INFORMATIVENESS-GATED trust (escape the low-TLE trap):
            //   precision = 1/(quant_error + ε)   ← reconstruction reliability ONLY;
            //     the dual-TLE's transition_surp term is dropped (it perversely
            //     penalises channels that track a CHANGING world, i.e. the
            //     informative ones).
            //   info = floor + (1−floor)·winner_entropy   ← actual discrimination;
            //     a flat/degenerate channel → entropy 0 → factor = floor → stripped.
            float precision = 1.0f / (std::abs(active[i].second->quant_error) + trust_epsilon_);
            float ci   = channel_informativeness(*active[i].second);   // baked-count based, ∈[0,1]
            float info = info_floor_ + (1.0f - info_floor_) * ci;
            // NO floor on `info` here — a genuinely uninformative channel (ci=0,
            // info_floor=0) gets EXACTLY zero raw trust, so its unbounded
            // precision (QE→0) can't resurrect it.  pow(0, gain) = 0.
            r = precision * std::pow(info, informativeness_gain_);
        } else {
            float tle = active[i].second->tle;   // legacy: dual-TLE inverse (bit-identical)
            r = 1.0f / (std::abs(tle) + trust_epsilon_);
        }
        if (surprise_gain_ > 0.0f) {
            auto sit = surprise_ema_.find(active[i].first);
            if (sit != surprise_ema_.end()) {
                float scale = 1.0f - surprise_gain_ * sit->second;
                if (scale < surprise_floor_) scale = surprise_floor_;
                r *= scale;
            }
        }
        // Stage 2 activity term (see LateralVoter.hpp).  Updated for every active
        // channel each tick; multiplied in only when the gain is non-zero.
        if (activity_gain_ != 0.0f) {
            std::string const& topic = active[i].first;
            Eigen::VectorXf const& L = active[i].second->latent;
            float d = 0.0f;
            auto pit = act_prev_latent_.find(topic);
            if (pit != act_prev_latent_.end() && pit->second.size() == L.size())
                d = (L - pit->second).norm();
            act_prev_latent_[topic] = L;
            float& ema  = act_ema_[topic];
            ema = (1.0f - activity_alpha_) * ema + activity_alpha_ * d;
            float& peak = act_peak_[topic];
            peak = std::max(ema, peak * activity_peak_decay_);
            float activity = peak > 1e-12f ? ema / peak : 0.0f;
            activity = std::max(activity_floor_, std::min(1.0f, activity));
            r *= std::pow(activity, activity_gain_);
        }
        raw[i] = r;
    }

    // Modality-group balancing.  Group inputs by parsed <group>; within each
    // group normalize raw → group-internal trust; then split total mass
    // equally across groups (so high-cardinality groups don't dominate).
    std::vector<float> trust(active.size(), 0.0f);

    if (group_balance_) {
        std::unordered_map<std::string, std::vector<size_t>> by_group;
        for (size_t i = 0; i < active.size(); ++i) {
            auto [g, m] = parse_group_member(active[i].first);
            (void)m;
            by_group[g].push_back(i);
        }
        size_t G = by_group.size();
        if (G == 0) G = 1;
        float per_group_mass = 1.0f / float(G);
        for (auto const& [g, idxs] : by_group) {
            float sum = 0.0f;
            for (auto i : idxs) sum += raw[i];
            if (sum <= 0.0f) {
                float share = per_group_mass / float(idxs.size());
                for (auto i : idxs) trust[i] = share;
            } else {
                for (auto i : idxs) trust[i] = per_group_mass * (raw[i] / sum);
            }
        }
    } else {
        // Plain L1 normalize.
        float sum = 0.0f;
        for (auto v : raw) sum += v;
        if (sum > 0.0f) for (size_t i = 0; i < raw.size(); ++i) trust[i] = raw[i] / sum;
        else            for (size_t i = 0; i < raw.size(); ++i) trust[i] = 1.0f / float(raw.size());
    }

    // Apply softmax temperature (only if it changes the distribution
    // meaningfully — temperature=1.0 leaves the trust unchanged, since the
    // raw vector is already a probability).  Lower T → sharper.
    float T = std::max(0.01f, softmax_temperature_ * (1.0f + 0.5f * dopamine_));
    if (std::abs(T - 1.0f) > 1e-6f) {
        std::vector<float> shaped(trust.size());
        float sum_exp = 0.0f;
        // Exponentiate log(trust)/T = trust^(1/T) (then renormalize).
        for (size_t i = 0; i < trust.size(); ++i) {
            float v = std::pow(std::max(1e-12f, trust[i]), 1.0f / T);
            shaped[i] = v;
            sum_exp  += v;
        }
        if (sum_exp > 0.0f)
            for (size_t i = 0; i < trust.size(); ++i) trust[i] = shaped[i] / sum_exp;
    }

    // Compute fused embedding + fused TLE.
    int dim = int(active.front().second->latent.size());
    Eigen::VectorXf fused = Eigen::VectorXf::Zero(dim);
    float fused_tle = 0.0f;
    for (size_t i = 0; i < active.size(); ++i) {
        if (active[i].second->latent.size() != dim) {
            // Skip mismatched-dim inputs rather than throw mid-tick.
            continue;
        }
        fused.noalias() += trust[i] * active[i].second->latent;
        fused_tle      += trust[i] * active[i].second->tle;
    }

    // Active modality selection: highest-trust group, with priority_group winning ties.
    std::unordered_map<std::string, float> group_mass;
    std::unordered_map<std::string, std::pair<size_t, float>> group_best;  // group -> (index, qe)
    for (size_t i = 0; i < active.size(); ++i) {
        auto [g, m] = parse_group_member(active[i].first);
        (void)m;
        group_mass[g] += trust[i];
        auto it = group_best.find(g);
        float qe = active[i].second->quant_error;
        if (it == group_best.end() || qe < it->second.second)
            group_best[g] = {i, qe};
    }
    std::string winner_group;
    float winner_mass = -1.0f;
    // First pass: priority_group breaks ties even if it isn't strictly highest.
    auto pri_it = group_mass.find(priority_group_);
    bool priority_present = pri_it != group_mass.end();
    for (auto const& [g, mass] : group_mass) {
        if (mass > winner_mass + 1e-6f) {
            winner_mass  = mass;
            winner_group = g;
        } else if (std::abs(mass - winner_mass) <= 1e-6f && priority_present
                   && g == priority_group_) {
            winner_group = g;
        }
    }
    size_t winner_idx = 0;
    int    winner_id  = -1;
    if (!winner_group.empty()) {
        auto it = group_best.find(winner_group);
        if (it != group_best.end()) {
            winner_idx = it->second.first;
            winner_id  = active[winner_idx].second->winner_id;
        }
    }

    // Build trust_weights map.
    std::unordered_map<std::string, float> trust_map;
    for (size_t i = 0; i < active.size(); ++i)
        trust_map[active[i].first] = trust[i];

    auto out = std::make_shared<ConsensusToken>();
    out->tick_id          = tick_id;
    out->producer_id      = id_.empty() ? std::string("voter") : id_;
    out->fused_embedding  = fused;
    out->fused_tle        = fused_tle;
    out->level            = level_;
    if (!winner_group.empty()) {
        auto [g, m] = parse_group_member(active[winner_idx].first);
        out->active_modality = m.empty() ? g : (g + "." + m);
    }
    out->active_winner_id = winner_id;
    out->trust_weights    = trust_map;

    // Phase 6.6.F — surface per-modality surprise EMA so MotorFader (and
    // any other downstream consumer) can drive α from prediction
    // accuracy without poking the voter's internals.  Empty when
    // surprise_gain_ == 0; otherwise contains one entry per modality
    // that has had a prediction-vs-observation comparison this run.
    //
    // 6.6.J — Bayesian shrinkage on the surfaced value when
    // surprise_calibrate_ is on.  With few prediction samples
    // (cold-start EPM) the raw EMA is unreliable; shrink toward 0.5
    // prior with strength 1/surprise_alpha so the FaderController
    // doesn't see "perfect predictor" the moment surprise_ema is
    // initialised.  Trust modulator (above) keeps using the raw EMA
    // so its internal sensitivity isn't affected.
    if (surprise_gain_ > 0.0f && !surprise_ema_.empty()) {
        if (surprise_calibrate_ && surprise_alpha_ > 0.0f) {
            float n_prior = 1.0f / surprise_alpha_;
            for (auto const& [topic, raw_ema] : surprise_ema_) {
                int64_t n = 0;
                auto cit = prediction_counts_.find(topic);
                if (cit != prediction_counts_.end()) n = cit->second;
                float c = float(n) / (float(n) + n_prior);
                out->surprise_ema[topic] =
                    (1.0f - c) * 0.5f + c * raw_ema;
            }
        } else {
            out->surprise_ema = surprise_ema_;
        }
    }

    // Phase 6.6.I — surface per-modality forward rollouts and current
    // winners so Premotor's rollout-aware exploration can read them
    // without re-subscribing to every reality.* topic.  Both maps are
    // populated unconditionally (cheap, small) so the downstream
    // consumer can opt in via its own param without needing a
    // surprise_gain enable on the voter.
    for (auto const& [topic, tok] : active) {
        out->winner_ids_by_modality[topic] = tok->winner_id;
        if (!tok->predicted_pathway.empty()) {
            out->predicted_pathways[topic] = tok->predicted_pathway;
        }
    }

    // Hebbian association update (stub — only when enabled).
    if (association_enabled_) {
        for (auto& [a, row] : assoc_)
            for (auto& [b, w] : row) w *= association_decay_;
        for (size_t i = 0; i < active.size(); ++i) {
            for (size_t j = 0; j < active.size(); ++j) {
                if (i == j) continue;
                std::string a = std::to_string(active[i].second->winner_id);
                std::string b = std::to_string(active[j].second->winner_id);
                assoc_[a][b] += trust[i] * trust[j];
            }
        }
    }

    if (publish_consensus_)
        bus_->publish(output_topic_, out);

    // Phase-6.0.c: hierarchical voting.  Republish the fused output as a
    // RealityToken so a parent voter can consume it through its standard
    // reality.* subscription without us inventing a new module class.  The
    // synthesised token carries the fused embedding as `latent`, fused_tle
    // as `tle`, and the active_winner_id from this tick.  Fields that
    // don't apply at the voter level (just_baked, mitosis_count, etc.)
    // stay default — downstream consumers that require those would refuse
    // a voter-synthesised token, which is the right behaviour.
    if (!publish_as_reality_topic_.empty()) {
        auto rt = std::make_shared<RealityToken>();
        rt->tick_id          = tick_id;
        rt->producer_id      = id_.empty() ? std::string("voter") : id_;
        rt->winner_id        = winner_id;
        rt->tle              = fused_tle;
        rt->latent           = fused;
        bus_->publish(publish_as_reality_topic_, rt);
    }

    prev_token_ = out;
    pending_.clear();
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

// The high-rate payload (xaq_voice).  The consensus's own fused error, plus the two
// per-source maps that say WHICH module the consensus currently believes: trust is the
// precision weight 1/(tle+eps) the fusion actually used, surprise its running prediction
// error.  Both are published as nested objects rather than flattened keys, so a subscriber
// enumerating dotted paths sees `trust.<modality>` without this module having to guess at
// a fixed source order.  The maps are small (one entry per voting modality) and copied as
// floats; nothing here walks an embedding.
nlohmann::json LateralVoter::diag_lite() const {
    nlohmann::json trust = nlohmann::json::object();
    nlohmann::json surp  = nlohmann::json::object();
    if (prev_token_)
        for (auto const& [k, v] : prev_token_->trust_weights) trust[k] = v;
    for (auto const& [k, v] : surprise_ema_) surp[k] = v;
    nlohmann::json j = {
        {"dopamine",  dopamine_},
        {"fused_tle", prev_token_ ? prev_token_->fused_tle : 0.0f},
        {"has_token", bool(prev_token_)},
        {"trust",     std::move(trust)},
        {"surprise",  std::move(surp)},
    };
    if (activity_gain_ != 0.0f) {
        nlohmann::json act = nlohmann::json::object();
        for (auto const& [k, v] : act_ema_) act[k] = activity(k);
        j["activity"] = std::move(act);
    }
    return j;
}

nlohmann::json LateralVoter::snapshot_state() const {
    nlohmann::json prev = nullptr;
    if (prev_token_) {
        nlohmann::json embed = nlohmann::json::array();
        for (int i = 0; i < prev_token_->fused_embedding.size(); ++i)
            embed.push_back(prev_token_->fused_embedding(i));
        nlohmann::json trust = nlohmann::json::object();
        for (auto const& [k, v] : prev_token_->trust_weights) trust[k] = v;
        prev = nlohmann::json{
            {"fused_embedding",   embed},
            {"fused_tle",         prev_token_->fused_tle},
            {"level",             prev_token_->level},
            {"active_modality",   prev_token_->active_modality},
            {"active_winner_id",  prev_token_->active_winner_id},
            {"trust_weights",     trust},
        };
    }
    nlohmann::json assoc = nlohmann::json::object();
    for (auto const& [k1, inner] : assoc_) {
        nlohmann::json row = nlohmann::json::object();
        for (auto const& [k2, v] : inner) row[k2] = v;
        assoc[k1] = row;
    }
    nlohmann::json predicted = nlohmann::json::object();
    for (auto const& [k, v] : last_predicted_next_) predicted[k] = v;
    nlohmann::json surp = nlohmann::json::object();
    for (auto const& [k, v] : surprise_ema_) surp[k] = v;
    nlohmann::json counts = nlohmann::json::object();
    for (auto const& [k, v] : prediction_counts_) counts[k] = v;
    nlohmann::json embed_cache = nlohmann::json::object();
    for (auto const& [topic, inner] : embedding_cache_) {
        nlohmann::json m = nlohmann::json::object();
        for (auto const& [node_id, vec] : inner) {
            nlohmann::json a = nlohmann::json::array();
            for (int i = 0; i < vec.size(); ++i) a.push_back(vec(i));
            m[std::to_string(node_id)] = std::move(a);
        }
        embed_cache[topic] = std::move(m);
    }
    nlohmann::json out = nlohmann::json{
        {"version",              1},
        {"dopamine",             dopamine_},
        {"prev_token",           prev},
        {"assoc",                assoc},
        {"last_predicted_next",  predicted},
        {"surprise_ema",         surp},
        {"prediction_counts",    counts},
        {"embedding_cache",      embed_cache},
    };
    // Stage 2 activity state — emitted ONLY when the term is on, so every
    // snapshot at activity_gain 0 is byte-identical to the pre-feature form.
    if (activity_gain_ != 0.0f) {
        nlohmann::json ema = nlohmann::json::object(), peak = nlohmann::json::object(), prev = nlohmann::json::object();
        for (auto const& [k, v] : act_ema_)  ema[k]  = v;
        for (auto const& [k, v] : act_peak_) peak[k] = v;
        for (auto const& [k, vec] : act_prev_latent_) {
            nlohmann::json arr = nlohmann::json::array();
            for (int i = 0; i < vec.size(); ++i) arr.push_back(vec(i));
            prev[k] = std::move(arr);
        }
        out["activity"] = nlohmann::json{{"ema", ema}, {"peak", peak}, {"prev_latent", prev}};
    }
    return out;
}

void LateralVoter::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("LateralVoter::restore_state: unknown version " +
                                 std::to_string(version));
    }
    dopamine_ = s.value("dopamine", dopamine_);
    prev_token_.reset();
    if (s.contains("prev_token") && !s["prev_token"].is_null()) {
        auto const& pt = s["prev_token"];
        auto out = std::make_shared<ConsensusToken>();
        if (pt.contains("fused_embedding") && pt["fused_embedding"].is_array()) {
            auto const& arr = pt["fused_embedding"];
            out->fused_embedding.resize(int(arr.size()));
            for (size_t i = 0; i < arr.size(); ++i) out->fused_embedding(int(i)) = arr[i].get<float>();
        }
        out->fused_tle        = pt.value("fused_tle",        0.0f);
        out->level            = pt.value("level",            0);
        out->active_modality  = pt.value("active_modality",  std::string{});
        out->active_winner_id = pt.value("active_winner_id", -1);
        if (pt.contains("trust_weights") && pt["trust_weights"].is_object()) {
            for (auto it = pt["trust_weights"].begin(); it != pt["trust_weights"].end(); ++it)
                out->trust_weights[it.key()] = it.value().get<float>();
        }
        prev_token_ = out;
    }
    assoc_.clear();
    if (s.contains("assoc") && s["assoc"].is_object()) {
        for (auto it1 = s["assoc"].begin(); it1 != s["assoc"].end(); ++it1) {
            for (auto it2 = it1.value().begin(); it2 != it1.value().end(); ++it2) {
                assoc_[it1.key()][it2.key()] = it2.value().get<float>();
            }
        }
    }
    last_predicted_next_.clear();
    if (s.contains("last_predicted_next") && s["last_predicted_next"].is_object()) {
        for (auto it = s["last_predicted_next"].begin(); it != s["last_predicted_next"].end(); ++it)
            last_predicted_next_[it.key()] = it.value().get<int>();
    }
    surprise_ema_.clear();
    if (s.contains("surprise_ema") && s["surprise_ema"].is_object()) {
        for (auto it = s["surprise_ema"].begin(); it != s["surprise_ema"].end(); ++it)
            surprise_ema_[it.key()] = it.value().get<float>();
    }
    prediction_counts_.clear();
    if (s.contains("prediction_counts") && s["prediction_counts"].is_object()) {
        for (auto it = s["prediction_counts"].begin(); it != s["prediction_counts"].end(); ++it)
            prediction_counts_[it.key()] = it.value().get<int64_t>();
    }
    embedding_cache_.clear();
    if (s.contains("embedding_cache") && s["embedding_cache"].is_object()) {
        for (auto it1 = s["embedding_cache"].begin(); it1 != s["embedding_cache"].end(); ++it1) {
            auto& inner = embedding_cache_[it1.key()];
            for (auto it2 = it1.value().begin(); it2 != it1.value().end(); ++it2) {
                Eigen::VectorXf v(int(it2.value().size()));
                for (size_t i = 0; i < it2.value().size(); ++i)
                    v(int(i)) = it2.value()[i].get<float>();
                inner[std::stoi(it2.key())] = std::move(v);
            }
        }
    }
    act_ema_.clear(); act_peak_.clear(); act_prev_latent_.clear();
    if (s.contains("activity") && s["activity"].is_object()) {
        auto const& a = s["activity"];
        if (a.contains("ema"))  for (auto it = a["ema"].begin();  it != a["ema"].end();  ++it) act_ema_[it.key()]  = it.value().get<float>();
        if (a.contains("peak")) for (auto it = a["peak"].begin(); it != a["peak"].end(); ++it) act_peak_[it.key()] = it.value().get<float>();
        if (a.contains("prev_latent")) {
            for (auto it = a["prev_latent"].begin(); it != a["prev_latent"].end(); ++it) {
                Eigen::VectorXf v(int(it.value().size()));
                for (size_t k = 0; k < it.value().size(); ++k) v(int(k)) = it.value()[k].get<float>();
                act_prev_latent_[it.key()] = std::move(v);
            }
        }
    }
}

} // namespace ogma
