#include "ogma/modules/EpisodicCapture.hpp"

#include <algorithm>
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
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("EpisodicCapture: param '" + key + "' must be integer");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("EpisodicCapture: param '" + key + "' must be bool");
}

// v5.4 Phase D — bilateral mirror of the 5-intent table.  Maps intent
// indices to their L/R-swapped counterparts: 0↔4 (hard left ↔ hard
// right), 1↔3 (soft left ↔ soft right), 2↔2 (forward straight unchanged).
// If a future config uses a different intent table this stays correct
// because mirror is a per-position swap, not a table-aware transform —
// sequences captured under any 5-intent table get a structurally
// mirrored counterpart.
int mirror_intent(int idx) {
    switch (idx) {
        case 0: return 4;
        case 1: return 3;
        case 2: return 2;
        case 3: return 1;
        case 4: return 0;
        default: return idx;   // pass-through for unknown indices
    }
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("EpisodicCapture: param '" + key + "' must be string");
}

std::vector<std::string> get_string_list(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    if (auto p = std::get_if<std::string>(&v)) return {*p};
    throw std::invalid_argument("EpisodicCapture: param '" + key + "' must be string array");
}

} // namespace

EpisodicCapture::EpisodicCapture()  = default;
EpisodicCapture::~EpisodicCapture() = default;

std::string_view EpisodicCapture::type_name() const { return "EpisodicCapture"; }

std::vector<TopicSpec> EpisodicCapture::input_topics() const {
    return {
        TopicSpec{keyframe_topic_, std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{intent_topic_,   std::type_index(typeid(PolicyToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kEventsPrefix, std::type_index(typeid(EnvEvent)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> EpisodicCapture::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(EpisodicChunkProposal))} };
}

ParamSchema EpisodicCapture::params_schema() const {
    return {
        {"keyframe_topic",       ParamMutability::ConstructionOnly,
            "Slow consensus EPM output (RealityToken).  EpisodicCapture uses each fresh-republish boundary on this topic as one keyframe.",
            ParamValue{std::string("reality.slow.consensus")}},
        {"intent_topic",         ParamMutability::ConstructionOnly,
            "PolicyToken topic Premotor publishes (chosen_intent per tick).  EpisodicCapture stamps the most-recent intent observed at each keyframe boundary into the rolling buffer.",
            ParamValue{std::string("policy.intent")}},
        {"output_topic",         ParamMutability::ConstructionOnly,
            "Where to publish EpisodicChunkProposal on reward.  MotorRepertoire subscribes here when in episodic mode.",
            ParamValue{std::string("motor.episodic_proposal")}},
        {"reward_event_names",   ParamMutability::ConstructionOnly,
            "Array of event names that trigger chunk crystallisation.  Default ['hit', 'scent_aligned_with_green'] catches both actual food eats and handtuned alignment moments.",
            std::nullopt},
        {"total_keyframes",      ParamMutability::HotMutable,
            "Total keyframes captured per chunk (entry_keyframes + body_keyframes).  Default 5.",
            ParamValue{int64_t{5}}},
        {"entry_keyframes",      ParamMutability::HotMutable,
            "Number of keyframes (from buffer head) reserved as entry context for cue-based dispatch matching.  Default 2.  Body keyframes = total_keyframes - entry_keyframes.",
            ParamValue{int64_t{2}}},
        {"playback_ticks_per_intent", ParamMutability::HotMutable,
            "Each captured body keyframe = one intent index sampled at the slow EPM's process_every_n_ticks rate.  ActionDecoder consumes intent_sequence one per tick during chunk replay — so each captured intent must be expanded to repeat playback_ticks_per_intent times in the emitted proposal to span the corresponding keyframe duration.  Default 50 (matches the canonical slow EPM process_every_n_ticks=50 = ~833ms keyframes).  Total replayed chunk length = body_keyframes × playback_ticks_per_intent ticks.",
            ParamValue{int64_t{50}}},
        {"mirror_augment",        ParamMutability::HotMutable,
            "v5.4 Phase D: when true, every reward fires TWO chunk proposals — the original captured trajectory AND a bilateral mirror with intents 0↔4 and 1↔3 swapped (intent 2 stays).  Same entry_embeddings, opposite motor sequence.  Doubles the chunk pool and balances L/R distribution per-reward, addressing the chunk-diversity bias observed at brain-only α where chunks all turn one direction because reflex chemotaxis on a given seed happens to predominantly approach food from one side.  Default false = legacy single-proposal-per-reward.",
            ParamValue{false}},
        {"entry_use_winner_prototype", ParamMutability::HotMutable,
            "v5.4.L final: when true, snapshot the keyframe RealityToken's winner_prototype (GNG centroid of the closest cell) instead of latent (raw encoder output) into the chunk's entry_embeddings.  For identity-encoder EPMs, latent is the raw input passthrough — smooth across moments because consensus.0 is a trust-weighted average.  winner_prototype is the GNG's discriminating cell centroid; with N baked nodes spread across the input manifold, two moments in different cells get distinct prototypes (cosine ≈ inter-cell distance).  Skips keyframes where winner_prototype is empty (winner_id<0).  Default false preserves legacy latent capture.",
            ParamValue{false}},
        {"secondary_keyframe_topic",   ParamMutability::ConstructionOnly,
            "v5.4.M — short+long entry fusion.  When non-empty, subscribe to this additional RealityToken topic and cache its latest src vector (latent or winner_prototype per entry_use_winner_prototype).  At keyframe capture, the chunk's entry_embedding becomes concat(primary || secondary).  Use a per-tick fast EPM output (e.g., reality.scent.scent or reality.video.color) as secondary so the entry context blends slow-coherent state with momentary environment signal that survives the slow-consensus averaging floor.  ActionDecoder must have a matching secondary_entry_topic.  Empty default preserves legacy single-source capture.",
            ParamValue{std::string("")}},
    };
}

ParamMap EpisodicCapture::current_params() const {
    ParamMap m;
    m["keyframe_topic"]      = ParamValue{keyframe_topic_};
    m["intent_topic"]        = ParamValue{intent_topic_};
    m["output_topic"]        = ParamValue{output_topic_};
    m["reward_event_names"]  = ParamValue{reward_event_names_};
    m["total_keyframes"]     = ParamValue{int64_t(total_keyframes_)};
    m["entry_keyframes"]     = ParamValue{int64_t(entry_keyframes_)};
    m["playback_ticks_per_intent"] = ParamValue{int64_t(playback_ticks_per_intent_)};
    m["mirror_augment"]      = ParamValue{mirror_augment_};
    m["entry_use_winner_prototype"] = ParamValue{entry_use_winner_prototype_};
    m["secondary_keyframe_topic"]   = ParamValue{secondary_keyframe_topic_};
    return m;
}

void EpisodicCapture::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("EpisodicCapture requires a non-null Bus");

    apply_param(params, "keyframe_topic",
        [&](auto const& v){ keyframe_topic_ = get_string(v, "keyframe_topic"); });
    apply_param(params, "intent_topic",
        [&](auto const& v){ intent_topic_ = get_string(v, "intent_topic"); });
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "reward_event_names",
        [&](auto const& v){ reward_event_names_ = get_string_list(v, "reward_event_names"); });
    apply_param(params, "total_keyframes",
        [&](auto const& v){ total_keyframes_ = std::max(2, int(get_int(v, "total_keyframes"))); });
    apply_param(params, "entry_keyframes",
        [&](auto const& v){ entry_keyframes_ = std::max(1, int(get_int(v, "entry_keyframes"))); });
    apply_param(params, "playback_ticks_per_intent",
        [&](auto const& v){ playback_ticks_per_intent_ = std::max(1, int(get_int(v, "playback_ticks_per_intent"))); });
    apply_param(params, "mirror_augment",
        [&](auto const& v){ mirror_augment_ = get_bool(v, "mirror_augment"); });
    apply_param(params, "entry_use_winner_prototype",
        [&](auto const& v){ entry_use_winner_prototype_ = get_bool(v, "entry_use_winner_prototype"); });
    apply_param(params, "secondary_keyframe_topic",
        [&](auto const& v){ secondary_keyframe_topic_ = get_string(v, "secondary_keyframe_topic"); });

    if (entry_keyframes_ >= total_keyframes_) {
        throw std::invalid_argument(
            "EpisodicCapture: entry_keyframes (" + std::to_string(entry_keyframes_)
            + ") must be < total_keyframes (" + std::to_string(total_keyframes_) + ")");
    }

    sub_ids_.push_back(bus_->subscribe(keyframe_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_keyframe(p); }));
    sub_ids_.push_back(bus_->subscribe(intent_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_intent(p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_event(t, p); }));
    // v5.4.M — optional secondary entry source for short+long fusion.
    if (!secondary_keyframe_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(secondary_keyframe_topic_,
                                            SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_secondary(p); }));
    }
}

void EpisodicCapture::handle_secondary(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;
    // Match the primary's source choice so concat halves share semantics.
    Eigen::VectorXf const& src =
        (entry_use_winner_prototype_ && rt->winner_prototype.size() > 0)
            ? rt->winner_prototype
            : rt->latent;
    if (src.size() == 0) return;
    last_secondary_src_ = src;
}

void EpisodicCapture::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "total_keyframes") total_keyframes_ = std::max(2, int(get_int(value, k)));
    else if (k == "entry_keyframes") entry_keyframes_ = std::max(1, int(get_int(value, k)));
    else if (k == "playback_ticks_per_intent") playback_ticks_per_intent_ = std::max(1, int(get_int(value, k)));
    else if (k == "mirror_augment") mirror_augment_ = get_bool(value, k);
    else if (k == "entry_use_winner_prototype") entry_use_winner_prototype_ = get_bool(value, k);
    else throw std::invalid_argument("EpisodicCapture: unknown/non-mutable param '" + k + "'");
    if (entry_keyframes_ >= total_keyframes_) {
        throw std::invalid_argument("EpisodicCapture: entry_keyframes must be < total_keyframes");
    }
    // Trim buffer if total_keyframes shrank.
    while (int(keyframe_buffer_.size()) > total_keyframes_) keyframe_buffer_.pop_front();
}

void EpisodicCapture::handle_keyframe(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;
    // v5.4.L final — pick the entry-vector source.  Default (legacy)
    // uses rt->latent (raw encoder output).  When
    // entry_use_winner_prototype_ is true AND winner_prototype is
    // populated, capture that instead — it's the GNG's discriminating
    // cell centroid, which decorrelates moments that fall in distinct
    // cells even when their raw inputs are nearly identical.
    Eigen::VectorXf const& src =
        (entry_use_winner_prototype_ && rt->winner_prototype.size() > 0)
            ? rt->winner_prototype
            : rt->latent;
    if (src.size() == 0) {
        // Bootstrap / no prototype yet — skip this keyframe.
        return;
    }
    // Republish-detection MUST use rt->latent (the raw encoder output),
    // NOT the chosen src.  Slow EPM republishes are bit-identical in
    // `latent`; with winner_prototype mode, two distinct keyframes that
    // happen to fall in the same GNG cell would have identical
    // winner_prototypes — filtering on src would drop them as fake
    // republishes and wreck the buffer.  last_keyframe_latent_ caches
    // the last-seen latent for this comparison.
    if (last_keyframe_latent_.size() == rt->latent.size()
        && last_keyframe_latent_.size() > 0
        && last_keyframe_latent_.isApprox(rt->latent, 1e-6f)) {
        return;   // slow EPM republish, no fresh keyframe boundary
    }
    last_keyframe_latent_ = rt->latent;
    // v5.4.M — concat short+long when secondary topic is configured.
    // If the secondary cache hasn't been populated yet (no message
    // arrived), skip this keyframe — we don't want chunks with
    // half-empty entry vectors that the matching side could never
    // reproduce.
    Eigen::VectorXf entry_vec;
    if (!secondary_keyframe_topic_.empty()) {
        if (last_secondary_src_.size() == 0) return;
        entry_vec.resize(src.size() + last_secondary_src_.size());
        entry_vec.head(src.size())                       = src;
        entry_vec.tail(last_secondary_src_.size())       = last_secondary_src_;
    } else {
        entry_vec = src;
    }
    keyframe_buffer_.push_back(Keyframe{entry_vec, last_intent_index_, rt->tick_id});
    while (int(keyframe_buffer_.size()) > total_keyframes_) keyframe_buffer_.pop_front();
    ++keyframes_seen_;
    last_keyframe_tick_ = rt->tick_id;
}

void EpisodicCapture::handle_intent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!pt) return;
    if (pt->chosen_intent < 0) return;
    last_intent_index_ = pt->chosen_intent;
    ++intents_seen_;
}

void EpisodicCapture::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;
    std::string name = !e->name.empty()
        ? e->name
        : std::string(topic.substr(std::min(topic.size(),
                       std::string_view("events.").size())));
    bool matches = false;
    for (auto const& n : reward_event_names_) {
        if (n == name) { matches = true; break; }
    }
    if (!matches) return;
    ++rewards_seen_;

    // Buffer must be full to crystallise.  Partial-buffer rewards are
    // silently dropped (early-game grace period — substrate hasn't
    // accumulated enough trajectory to extract a useful chunk yet).
    if (int(keyframe_buffer_.size()) < total_keyframes_) return;

    auto prop = std::make_shared<EpisodicChunkProposal>();
    prop->tick_id     = e->tick_id;
    prop->producer_id = id_.empty() ? std::string("episodic_capture") : id_;
    prop->trigger_event = name;
    prop->intensity     = e->intensity;
    // v5.4 Phase G — per-position credit bookkeeping needs to know the
    // body segmentation: how many body keyframes this chunk has and
    // how many ticks each one is replayed for.
    prop->body_keyframes              = total_keyframes_ - entry_keyframes_;
    prop->playback_ticks_per_position = playback_ticks_per_intent_;
    // Entry context = first entry_keyframes_ embeddings.
    for (int i = 0; i < entry_keyframes_; ++i) {
        prop->entry_embeddings.push_back(keyframe_buffer_[i].embedding);
    }
    // Body = remaining intent indices, each repeated playback_ticks_per_intent
    // times so the chunk's per-tick replay (popped one int per tick by
    // ActionDecoder) spans the keyframe duration the intent was actually
    // executed for during capture.
    for (int i = entry_keyframes_; i < int(keyframe_buffer_.size()); ++i) {
        int idx = keyframe_buffer_[i].intent_index;
        if (idx < 0) idx = 0;   // sanitise; intent slot may have been -1 if Premotor not yet sampled
        for (int t = 0; t < playback_ticks_per_intent_; ++t) {
            prop->intent_sequence.push_back(idx);
        }
    }
    bus_->publish(output_topic_, prop);
    ++proposals_emitted_;

    // v5.4 Phase D — also emit a bilateral mirror.  Same entry context
    // (perceptual side-asymmetry of the entry keyframes is opaque to
    // this module's vector-matching dispatch — agnostic to which side
    // the agent was "facing"), opposite motor sequence.  Doubles the
    // chunk pool's L/R coverage per reward.
    if (mirror_augment_) {
        auto mirror = std::make_shared<EpisodicChunkProposal>();
        mirror->tick_id          = e->tick_id;
        mirror->producer_id      = id_.empty() ? std::string("episodic_capture") : id_;
        mirror->trigger_event    = name + "_mirror";
        mirror->intensity        = e->intensity;
        mirror->entry_embeddings = prop->entry_embeddings;   // SAME entry context
        mirror->body_keyframes              = prop->body_keyframes;
        mirror->playback_ticks_per_position = prop->playback_ticks_per_position;
        mirror->intent_sequence.reserve(prop->intent_sequence.size());
        for (int idx : prop->intent_sequence) {
            mirror->intent_sequence.push_back(mirror_intent(idx));
        }
        bus_->publish(output_topic_, mirror);
        ++proposals_emitted_;
    }
}

void EpisodicCapture::tick(uint64_t /*tick_id*/) {
    // No per-tick processing — capture is event-driven via handlers.
}

nlohmann::json EpisodicCapture::snapshot_state() const {
    nlohmann::json j{
        {"version",            1},
        {"last_intent_index",  last_intent_index_},
        {"last_keyframe_tick", last_keyframe_tick_},
        {"keyframes_seen",     keyframes_seen_},
        {"intents_seen",       intents_seen_},
        {"proposals_emitted",  proposals_emitted_},
        {"rewards_seen",       rewards_seen_},
    };
    // Skip embedding buffer in snapshot — it's transient working state and
    // would bloat snapshots.  Restoration starts from empty buffer; new
    // keyframes will refill on the next reward window.
    return j;
}

void EpisodicCapture::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1)
        throw std::runtime_error("EpisodicCapture::restore_state: unknown version "
                                 + std::to_string(version));
    last_intent_index_  = s.value("last_intent_index",  last_intent_index_);
    last_keyframe_tick_ = s.value("last_keyframe_tick", last_keyframe_tick_);
    keyframes_seen_     = s.value("keyframes_seen",     keyframes_seen_);
    intents_seen_       = s.value("intents_seen",       intents_seen_);
    proposals_emitted_  = s.value("proposals_emitted",  proposals_emitted_);
    rewards_seen_       = s.value("rewards_seen",       rewards_seen_);
}

} // namespace ogma
