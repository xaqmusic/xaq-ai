#include "ogma/modules/MotorRepertoire.hpp"

#include <algorithm>
#include <cmath>
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
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("MotorRepertoire param '" + key + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("MotorRepertoire param '" + key + "' must be string");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("MotorRepertoire param '" + key + "' must be bool");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("MotorRepertoire param '" + key + "' must be integer");
}

} // namespace

MotorRepertoire::MotorRepertoire()  = default;
MotorRepertoire::~MotorRepertoire() = default;

std::string_view MotorRepertoire::type_name() const { return "MotorRepertoire"; }

std::vector<TopicSpec> MotorRepertoire::input_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{topics::kActionOut,           std::type_index(typeid(ActionOut)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{"sequence.motif.consensus.0",  std::type_index(typeid(SequenceMotif)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kDriveErrors,          std::type_index(typeid(DriveErrors)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kMotorPlayCmd,         std::type_index(typeid(MotorPlayCmd)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kEventsPrefix,         std::type_index(typeid(EnvEvent)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
    // Action motif source (legacy or v5.3 intent_motif_topic override).
    std::string action_motif =
        intent_motif_topic_.empty() ? std::string("sequence.motif.action.out") : intent_motif_topic_;
    v.push_back(TopicSpec{action_motif, std::type_index(typeid(SequenceMotif)),
                          SubscriptionKind::Direct, /*required=*/true});
    // v5.3 Phase B — intent capture source (PolicyToken or IntentToken).
    if (!intent_source_topic_.empty()) {
        v.push_back(TopicSpec{intent_source_topic_, std::type_index(typeid(PolicyToken)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    return v;
}

std::vector<TopicSpec> MotorRepertoire::output_topics() const {
    return {
        TopicSpec{topics::kMotorChunks,    std::type_index(typeid(MotorChunks))},
        TopicSpec{topics::kMotorPlayResp,  std::type_index(typeid(MotorPlayStream))},
    };
}

ParamSchema MotorRepertoire::params_schema() const {
    return {
        {"max_chunks",                          ParamMutability::HotMutable,        "Library cap (LRU evict beyond)", ParamValue{int64_t{256}}},
        {"chunk_max_ticks",                     ParamMutability::HotMutable,        "Max chunk length (truncated longer motifs)", ParamValue{int64_t{20}}},
        {"crystallization_min_observations",    ParamMutability::HotMutable,        "Required motif occurrences before crystallization", ParamValue{int64_t{10}}},
        {"crystallization_min_drive_delta",     ParamMutability::HotMutable,        "Required mean drive_delta for crystallization", ParamValue{0.05}},
        {"crystallization_drive_window_ticks",  ParamMutability::HotMutable,        "Window over which drive change is integrated", ParamValue{int64_t{50}}},
        {"interrupt_urgency_threshold",         ParamMutability::HotMutable,        "Urgency above this aborts active playback (Phase 3 stretch)", ParamValue{0.85}},
        {"interrupt_outcome_divergence",        ParamMutability::HotMutable,        "Phase 3 stretch", ParamValue{0.30}},
        {"master_seed",                         ParamMutability::ConstructionOnly,  "RNG namespace seed", ParamValue{int64_t{0}}},
        {"intent_source_topic",                 ParamMutability::ConstructionOnly,  "v5.3 Phase B: subscribe here for intent indices (PolicyToken or IntentToken).  When set, captured into intent_history and chunks crystallise as intent_sequence instead of action_sequence.  Empty = legacy action-chunk mode.", ParamValue{std::string("")}},
        {"intent_motif_topic",                  ParamMutability::ConstructionOnly,  "v5.3 Phase B: optional motif source for intent-based crystallisation gates.  Empty = use sequence.motif.action.out (legacy).", ParamValue{std::string("")}},
        {"seed_chunks",                         ParamMutability::ConstructionOnly,  "v5.3 Phase E: array of JSON-string chunks loaded into the library at startup.  Each entry: '{\"trigger_motif\": -1, \"trigger_urgency\": 0.0, \"intent_sequence\": [int, int, ...], \"hits_during\": 2}'.  Negative chunk_ids assigned (distinguish from organic).  hits_during>=2 boots Beta(1,1) score above 0.5 commit threshold so seeds dispatch immediately.", std::nullopt},
        {"accept_episodic",                     ParamMutability::ConstructionOnly,  "v5.4 Phase A: when true, subscribe to motor.episodic_proposal and crystallise EpisodicChunkProposal messages (from EpisodicCapture) directly into chunks.  Each crystallisation copies the proposal's entry_embeddings + intent_sequence + boots replay_hits=1 (Beta-prior score = 0.6, immediately dispatchable).  Bypasses the v5.3 motif-baking-then-coinciding-with-hit gate; reward IS the gate.  Default false = legacy behaviour preserved.", ParamValue{false}},
        // v5.4 Phase H — chunk lifecycle (eligibility trace + freshness decay + hard prune).
        {"chunk_credit_lookback",               ParamMutability::HotMutable,        "v5.4 Phase H: number of most-recent dispatched chunks credited on each events.hit.  Newest gets weight 1.0, prior chunks get decay^position weights.  1 = legacy single-chunk credit.", ParamValue{int64_t{3}}},
        {"chunk_credit_decay",                  ParamMutability::HotMutable,        "v5.4 Phase H: geometric decay applied per position back from newest dispatch in the eligibility trace.  0.5 = each prior chunk gets half the previous weight.  0.0 = single-chunk credit (newest only).", ParamValue{0.5}},
        {"chunk_freshness_decay_per_tick",      ParamMutability::HotMutable,        "v5.4 Phase H: per-tick multiplicative decay of replay_hits/_misses (and position_hits/_misses) of all chunks.  0 = no decay (legacy).  1e-4 = ~115s sim half-life @ 60Hz; chunks slowly forget old evidence so freshness reflects RECENT performance.", ParamValue{0.0001}},
        {"chunk_prune_score_threshold",         ParamMutability::HotMutable,        "v5.4 Phase H: Beta-prior score below which a chunk is hard-erased (not just is_active=false).  0 = no pruning (legacy; only LRU eviction at max_chunks).  0.3 default = clearly-bad chunks pruned after enough observations.", ParamValue{0.3}},
        {"chunk_prune_min_dispatches",          ParamMutability::HotMutable,        "v5.4 Phase H: chunk needs at least this many total observations (replay_hits+replay_misses, rounded) before pruning is considered.  Protects fresh chunks from premature death.  Default 5.", ParamValue{int64_t{5}}},
    };
}

void MotorRepertoire::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotorRepertoire requires a non-null Bus");

    apply_param(params, "max_chunks",                         [&](auto const& v){ max_chunks_                          = std::max(1, int(get_int(v, "max_chunks"))); });
    apply_param(params, "chunk_max_ticks",                    [&](auto const& v){ chunk_max_ticks_                     = std::max(2, int(get_int(v, "chunk_max_ticks"))); });
    apply_param(params, "crystallization_min_observations",   [&](auto const& v){ crystallization_min_observations_    = std::max(1, int(get_int(v, "crystallization_min_observations"))); });
    apply_param(params, "crystallization_min_drive_delta",    [&](auto const& v){ crystallization_min_drive_delta_     = float(get_double(v, "crystallization_min_drive_delta")); });
    apply_param(params, "crystallization_drive_window_ticks", [&](auto const& v){ crystallization_drive_window_ticks_  = std::max(1, int(get_int(v, "crystallization_drive_window_ticks"))); });
    apply_param(params, "interrupt_urgency_threshold",        [&](auto const& v){ interrupt_urgency_threshold_         = float(get_double(v, "interrupt_urgency_threshold")); });
    apply_param(params, "interrupt_outcome_divergence",       [&](auto const& v){ interrupt_outcome_divergence_        = float(get_double(v, "interrupt_outcome_divergence")); });
    apply_param(params, "master_seed",                        [&](auto const& v){ master_seed_                         = uint64_t(get_int(v, "master_seed")); });
    apply_param(params, "intent_source_topic",                [&](auto const& v){ intent_source_topic_                 = get_string(v, "intent_source_topic"); });
    apply_param(params, "intent_motif_topic",                 [&](auto const& v){ intent_motif_topic_                  = get_string(v, "intent_motif_topic"); });
    apply_param(params, "accept_episodic",                    [&](auto const& v){ accept_episodic_                     = get_bool(v, "accept_episodic"); });
    apply_param(params, "chunk_credit_lookback",              [&](auto const& v){ chunk_credit_lookback_               = std::max(1, int(get_int(v, "chunk_credit_lookback"))); });
    apply_param(params, "chunk_credit_decay",                 [&](auto const& v){ chunk_credit_decay_                  = std::clamp(float(get_double(v, "chunk_credit_decay")), 0.0f, 1.0f); });
    apply_param(params, "chunk_freshness_decay_per_tick",     [&](auto const& v){ chunk_freshness_decay_per_tick_      = std::max(0.0f, float(get_double(v, "chunk_freshness_decay_per_tick"))); });
    apply_param(params, "chunk_prune_score_threshold",        [&](auto const& v){ chunk_prune_score_threshold_         = std::max(0.0f, float(get_double(v, "chunk_prune_score_threshold"))); });
    apply_param(params, "chunk_prune_min_dispatches",         [&](auto const& v){ chunk_prune_min_dispatches_          = std::max(1, int(get_int(v, "chunk_prune_min_dispatches"))); });

    // v5.3 Phase E — load hand-authored seed chunks.  Each seed entry is a
    // JSON-string parsed into a MotifTracking struct + assigned a negative
    // chunk_id (so it doesn't collide with organic crystallisations that
    // start at next_chunk_id_=1).  Boots replay_hits=2 so Beta(1,1)-prior
    // score = (2+0+1)/(2+0+0+2) = 0.75 — above the 0.5 commit threshold,
    // immediately dispatchable.
    {
        auto sit = params.find("seed_chunks");
        if (sit != params.end()) {
            auto entries = std::get<std::vector<std::string>>(sit->second);
            int seed_id = -1;
            for (auto const& s : entries) {
                auto j = nlohmann::json::parse(s);
                MotifTracking mt;
                mt.chunk_id                    = seed_id--;
                mt.observations                = std::max(int(crystallization_min_observations_),
                                                          int(j.value("observations", crystallization_min_observations_)));
                mt.hits_during                 = int(j.value("hits_during", 2));
                mt.replay_hits                 = float(j.value("replay_hits", 2.0));
                mt.replay_misses               = float(j.value("replay_misses", 0.0));
                mt.trigger_consensus_motif_id  = int(j.value("trigger_motif", -1));
                mt.trigger_urgency             = float(j.value("trigger_urgency", -1.0));
                mt.is_active                   = true;
                if (j.contains("intent_sequence") && j["intent_sequence"].is_array()) {
                    for (auto const& v : j["intent_sequence"])
                        mt.intent_sequence.push_back(v.get<int>());
                }
                if (j.contains("action_sequence") && j["action_sequence"].is_array()) {
                    for (auto const& v : j["action_sequence"])
                        mt.action_sequence.push_back(v.get<float>());
                }
                chunks_[mt.chunk_id] = std::move(mt);
            }
            library_dirty_ = true;
        }
    }

    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(topics::kActionOut, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_action(t, p); }));
    if (!intent_source_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(intent_source_topic_, SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_intent_source(t, p); }));
    }
    std::string action_motif_topic =
        intent_motif_topic_.empty() ? "sequence.motif.action.out" : intent_motif_topic_;
    sub_ids_.push_back(bus_->subscribe(action_motif_topic, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_motif(t, p); }));
    sub_ids_.push_back(bus_->subscribe("sequence.motif.consensus.0", SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_consensus_motif(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kDriveErrors, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_drive(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kMotorPlayCmd, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_play_cmd(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_event(t, p); }));
    if (accept_episodic_) {
        sub_ids_.push_back(bus_->subscribe(topics::kMotorEpisodicProposal, SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_episodic_proposal(t, p); }));
    }
}

void MotorRepertoire::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "max_chunks")                          max_chunks_                         = std::max(1, int(get_int(value, k)));
    else if (k == "chunk_max_ticks")                     chunk_max_ticks_                    = std::max(2, int(get_int(value, k)));
    else if (k == "crystallization_min_observations")    crystallization_min_observations_   = std::max(1, int(get_int(value, k)));
    else if (k == "crystallization_min_drive_delta")     crystallization_min_drive_delta_    = float(get_double(value, k));
    else if (k == "crystallization_drive_window_ticks")  crystallization_drive_window_ticks_ = std::max(1, int(get_int(value, k)));
    else if (k == "interrupt_urgency_threshold")         interrupt_urgency_threshold_        = float(get_double(value, k));
    else if (k == "interrupt_outcome_divergence")        interrupt_outcome_divergence_       = float(get_double(value, k));
    else if (k == "chunk_credit_lookback")               chunk_credit_lookback_              = std::max(1, int(get_int(value, k)));
    else if (k == "chunk_credit_decay")                  chunk_credit_decay_                 = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "chunk_freshness_decay_per_tick")      chunk_freshness_decay_per_tick_     = std::max(0.0f, float(get_double(value, k)));
    else if (k == "chunk_prune_score_threshold")         chunk_prune_score_threshold_        = std::max(0.0f, float(get_double(value, k)));
    else if (k == "chunk_prune_min_dispatches")          chunk_prune_min_dispatches_         = std::max(1, int(get_int(value, k)));
    else if (k == "master_seed")
        throw std::invalid_argument("MotorRepertoire.master_seed is ConstructionOnly");
    else
        throw std::invalid_argument("MotorRepertoire: unknown param '" + k + "'");
}

// v5.3 Phase B — intent capture path.  Subscribed only when
// intent_source_topic_ is configured.  Accepts both PolicyToken
// (Premotor's chosen_intent) and IntentToken (override stream) so
// MotorRepertoire can capture from either side.
void MotorRepertoire::handle_intent_source(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    int idx = -1;
    if (auto pt = std::dynamic_pointer_cast<const PolicyToken>(payload)) {
        idx = pt->chosen_intent;
    } else if (auto it = std::dynamic_pointer_cast<const IntentToken>(payload)) {
        idx = it->index;
    }
    if (idx < 0) return;
    intent_history_.push_back(idx);
    while (int(intent_history_.size()) > chunk_max_ticks_) intent_history_.pop_front();
    ++intents_received_total_;
}

// v5.4 Phase A — episodic chunk ingestion path.  Bypasses the v5.3 motif-
// baking gate; reward IS the gate (EpisodicCapture only emits proposals
// when a configured reward event fires AND the keyframe buffer is full).
// Each proposal becomes a new chunk with replay_hits=1 boot (Beta-prior
// score 0.6 — above the 0.5 commit threshold so it's immediately
// dispatchable, but below 0.83 so an organic chunk that actually
// out-performs in replay can rise above it).  LRU eviction same as
// motif-baking path.
void MotorRepertoire::handle_episodic_proposal(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const EpisodicChunkProposal>(payload);
    if (!p) return;
    if (p->intent_sequence.empty() || p->entry_embeddings.empty()) return;

    auto score_fn = [](MotifTracking const& mt) -> float {
        float h = float(mt.hits_during + mt.replay_hits) + 1.0f;
        float t = h + float(mt.replay_misses) + 1.0f;
        return h / t;
    };
    if (int(chunks_.size()) >= max_chunks_) {
        auto worst = chunks_.begin();
        for (auto it = chunks_.begin(); it != chunks_.end(); ++it)
            if (score_fn(it->second) < score_fn(worst->second)) worst = it;
        chunks_.erase(worst);
    }

    MotifTracking mt;
    mt.chunk_id          = next_chunk_id_++;
    mt.intent_sequence   = p->intent_sequence;
    mt.entry_embeddings  = p->entry_embeddings;
    // Boot Beta-prior: 1 implicit hit (reward fired), 0 misses.  Score = 0.6.
    mt.hits_during       = 1;
    mt.replay_hits       = 0.0f;
    mt.replay_misses     = 0.0f;
    mt.observations      = std::max(1, crystallization_min_observations_);
    mt.is_active         = true;
    // v5.4.L — stamp creation tick from the proposal's reward-event tick.
    mt.created_tick_id   = p->tick_id;
    // v5.4 Phase G (Proposal C) — per-position credit buckets.  body_keyframes
    // and playback_ticks_per_position come from the proposal; if the
    // proposal didn't set them (legacy), derive from intent_sequence
    // length (single-position fallback).
    mt.body_keyframes              = p->body_keyframes > 0 ? p->body_keyframes : 1;
    mt.playback_ticks_per_position = p->playback_ticks_per_position > 0 ? p->playback_ticks_per_position : int(p->intent_sequence.size());
    mt.position_hits.assign(mt.body_keyframes, 0.0f);
    mt.position_misses.assign(mt.body_keyframes, 0.0f);
    // Episodic chunks fire by entry-embedding match, not consensus motif id.
    // Set trigger_consensus_motif_id = -1 (wildcard) and trigger_urgency = -1
    // (skip drive check) — entry-match in ActionDecoder is the gate.
    mt.trigger_consensus_motif_id = -1;
    mt.trigger_urgency            = -1.0f;
    chunks_[mt.chunk_id] = std::move(mt);
    library_dirty_ = true;
    ++episodic_proposals_ingested_;
}

void MotorRepertoire::handle_action(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    action_history_.push_back(a->accel);
    while (int(action_history_.size()) > chunk_max_ticks_) action_history_.pop_front();
    // v5.4 Phase G — track which chunk + body position is currently
    // being replayed so handle_event can credit the right slot.
    current_replay_chunk_id_ = a->chunk_id;
    current_replay_position_ = a->chunk_position;
    // ActionOut carries the chunk id that produced this command (-1 when
    // the decoder selected via EFE).  Tracking this lets handle_event
    // credit hits/misses to whichever chunk is currently replaying.
    //
    // Phase 6.5.3.3 — dispatch-boundary tracking: increment
    // total_dispatch_count_ on chunk-id rising-edge (a fresh chunk just
    // started); on falling-edge, evaluate whether the just-finished
    // dispatch counts as a failure (Wilson 95% lower CI on hits/(hits+misses)
    // for that chunk falls below 0.5 with sufficient evidence).
    // Phase 6.5.12 — dispatch boundary detection moved from chunk_id-edge
    // to handle_play_cmd (which tracks each dispatch as a unique
    // request_id).  ActionOut.chunk_id stays at the same value when a
    // chunk re-dispatches back-to-back (no -1 gap), so the previous
    // edge-based logic missed transitions in continuous-mode chunk
    // dispatch storms.  active_replay_chunk_id_ is still tracked here
    // for handle_event(hit/miss) to know which chunk is currently
    // replaying for replay_hits/replay_misses crediting.
    active_replay_chunk_id_ = a->chunk_id;
}

void MotorRepertoire::evaluate_dispatch_outcome_(int chunk_id) {
    // Phase 6.5.12 — called from handle_play_cmd at every new dispatch.
    // Evaluates whether the PREVIOUS dispatch (the one whose chunk_id
    // matches `chunk_id` here, just ending now) led to a hit; if not,
    // increments replay_misses (implicit miss).  Then checks Wilson CI
    // for demotion.  Symmetric with the original explicit-miss path,
    // but works in continuous-mode environments where events.miss
    // never fires.
    // v5.3 Phase F: accept any non-zero chunk_id including seeded chunks
    // (negative ids).  Previous `chunk_id <= 0` rejected seeds → seeds
    // never accumulated replay_misses → seeds dominated dispatch forever
    // even when ChunkOutcomeGate aborted them.
    if (chunk_id == 0) return;
    auto it = chunks_.find(chunk_id);
    if (it == chunks_.end()) return;
    auto& mt = it->second;
    // v5.4 Phase H — float-safe "no hit during this dispatch" check.
    // With eligibility-trace credit, replay_hits is fractional; small
    // eligibility crumbs from neighbouring dispatches' hits don't count
    // as "this dispatch produced a hit" — require a meaningful increment
    // (the most-recent dispatch credits with weight 1.0, so any value
    // >0.5 above snapshot indicates a hit during this very dispatch).
    if (mt.replay_hits - replay_hits_at_dispatch_start_ < 0.5f) {
        mt.replay_misses += 1.0f;
        library_dirty_ = true;   // v5.4.K — implicit miss visible.
    }
    float n = mt.replay_hits + mt.replay_misses;
    if (n >= 5.0f) {
        // Wilson score interval lower bound at 95%.
        // p_hat = h/n; z = 1.96 → z² = 3.8416.
        float p_hat = mt.replay_hits / n;
        float z2    = 3.8416f;
        float lower = (p_hat + z2 / (2.0f * n)
                      - 1.96f * std::sqrt(p_hat * (1.0f - p_hat) / n
                                          + z2 / (4.0f * n * n)))
                      / (1.0f + z2 / n);
        if (lower < 0.5f && mt.is_active) {
            mt.is_active   = false;
            library_dirty_ = true;
            ++failed_dispatch_count_;
        }
    }
}

void MotorRepertoire::handle_motif(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto m = std::dynamic_pointer_cast<const SequenceMotif>(payload);
    if (!m || m->motif_id < 0) return;

    // Defer stability validation to SeqGNG's two-gate baking machinery.
    // An unbaked motif id is by definition not yet a stable n-gram —
    // accumulating observations on it would dilute the count across the
    // ~55-70 ephemeral motif ids that flow past in a 5-minute easy run
    // (only 12-21 of which actually bake).  By filtering here, the
    // observations counter measures "occurrences of a stable motif"
    // rather than "occurrences of any winner id ever assigned" — which
    // is what the crystallization gate was always trying to approximate.
    if (!m->is_baked) return;

    auto& mt = motifs_[m->motif_id];
    ++mt.observations;
    last_baked_motif_id_ = m->motif_id;

    // Drive delta = decrease in urgency since last tick (urgency dropped → good).
    // Kept as a richer outcome statistic for LRU tie-breaking; the
    // crystallisation gate now uses the hit-correlation count below.
    float delta = prev_drive_urgency_ - latest_drive_urgency_;
    float a = 1.0f / std::max(1.0f, float(crystallization_drive_window_ticks_));
    mt.drive_delta_ema = (1.0f - a) * mt.drive_delta_ema + a * delta;

    // Crystallisation criterion.  Two gates:
    //  (1) baked-motif observations >= min_observations — sanity check that
    //      the motif has stable presence in the current behaviour, not just
    //      a one-tick fluke.  Trivially cleared once a motif bakes (we're
    //      already inside the !is_baked filter).
    //  (2) hits_during >= 1 — the motif was the active baked winner during
    //      at least one events.hit firing.  This replaces the old
    //      drive_delta_ema >= 0.03 gate, which was an EMA-diluted proxy
    //      for the same outcome signal that events.hit carries directly.
    if (mt.chunk_id < 0 &&
        mt.observations >= crystallization_min_observations_ &&
        mt.hits_during >= 1) {

        // LRU evict if at capacity.  Score = Beta(1,1)-prior success rate
        // over (hits_during + replay_hits) successes vs replay_misses
        // failures.  Starts at 0.5 for fresh chunks; refines toward true
        // hit-vs-miss ratio with replay use.  Bad chunks (more misses
        // than hits) get evicted first.  No magic weighting between
        // pre/post-crystallization signals — both are events.hit pulses.
        auto score = [](MotifTracking const& mt) -> float {
            float h = float(mt.hits_during + mt.replay_hits) + 1.0f;
            float t = h + float(mt.replay_misses) + 1.0f;
            return h / t;
        };
        if (int(chunks_.size()) >= max_chunks_) {
            auto worst = chunks_.begin();
            for (auto it = chunks_.begin(); it != chunks_.end(); ++it)
                if (score(it->second) < score(worst->second)) worst = it;
            chunks_.erase(worst);
        }

        mt.chunk_id            = next_chunk_id_++;
        mt.created_tick_id     = m->tick_id;   // v5.4.L — stamp legacy chunks too
        chunks_[mt.chunk_id]   = mt;
        library_dirty_         = true;
    }
}

void MotorRepertoire::handle_drive(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto d = std::dynamic_pointer_cast<const DriveErrors>(payload);
    if (!d) return;
    if (!drive_seen_) {
        // First delivery: seed both prev and latest so the first computed
        // delta is zero rather than reading the placeholder 0.0 baseline.
        prev_drive_urgency_   = d->urgency;
        latest_drive_urgency_ = d->urgency;
        latest_drive_delta_   = 0.0f;
        drive_seen_           = true;
        return;
    }
    prev_drive_urgency_   = latest_drive_urgency_;
    latest_drive_urgency_ = d->urgency;
    latest_drive_delta_   = prev_drive_urgency_ - latest_drive_urgency_;
}

void MotorRepertoire::handle_play_cmd(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto cmd = std::dynamic_pointer_cast<const MotorPlayCmd>(payload);
    if (!cmd) return;

    // Phase 6.5.12 — every play_cmd is a unique dispatch (request_id
    // increases monotonically from ActionDecoder).  This is the canonical
    // dispatch boundary, replacing the previous chunk_id-edge detection
    // which missed back-to-back re-dispatches of the same chunk.
    // v5.3 Phase F: accept negative chunk_ids (seeded chunks) as valid for
    // outcome evaluation — previous `> 0` filter excluded seeds entirely.
    // v5.4 Phase C: skip outcome eval when this dispatch is a manual probe
    // (cmd->force=true) — the operator is exploring, not training.
    if (!cmd->force
        && last_dispatch_request_id_ > 0 && last_dispatch_chunk_id_ != 0) {
        // The PREVIOUS dispatch is now ending — evaluate it for implicit
        // miss + Wilson CI demotion before starting the new one.
        evaluate_dispatch_outcome_(last_dispatch_chunk_id_);
    }
    ++total_dispatch_count_;
    last_dispatch_request_id_ = cmd->request_id;
    last_dispatch_chunk_id_   = cmd->chunk_id;

    // v5.4 Phase H — push this dispatch onto the eligibility trace.
    // Force-probe dispatches (manual chunk playback) are excluded so
    // operator inspection doesn't pollute training credit.
    if (!cmd->force && cmd->chunk_id != 0) {
        chunk_dispatch_trace_.push_back(cmd->chunk_id);
        while (int(chunk_dispatch_trace_.size()) > chunk_credit_lookback_) {
            chunk_dispatch_trace_.pop_front();
        }
    }

    auto resp = std::make_shared<MotorPlayStream>();
    resp->tick_id     = cmd->tick_id;
    resp->producer_id = id_.empty() ? std::string("motor_repertoire") : id_;
    resp->request_id  = cmd->request_id;
    resp->chunk_id    = cmd->chunk_id;

    auto it = chunks_.find(cmd->chunk_id);
    // v5.4 Phase C: manual force probe ignores the is_active gate so the
    // operator can fire Wilson-CI-demoted chunks for inspection.
    if (it != chunks_.end() && (it->second.is_active || cmd->force)) {
        // v5.3 Phase B — populate either intents or actions depending on
        // chunk format.  Exactly one is non-empty per chunk per the
        // MotorChunk contract.
        if (!it->second.intent_sequence.empty()) {
            resp->intents = it->second.intent_sequence;
        } else {
            resp->actions = it->second.action_sequence;
        }
        // Snapshot the chunk's replay_hits at dispatch start so the next
        // evaluate_dispatch_outcome_() call can detect "no hit during
        // this replay" → implicit miss.
        replay_hits_at_dispatch_start_ = it->second.replay_hits;
    }
    // Phase 6.5.3.3 — inactive chunks (Wilson lower-CI on success-rate
    // dropped below 0.5 with ≥5 dispatches) return an empty action
    // sequence; ActionDecoder treats this as no-chunk-available and
    // falls through to its EFE/Q policy.
    bus_->publish(topics::kMotorPlayResp, resp);
}

void MotorRepertoire::handle_consensus_motif(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto m = std::dynamic_pointer_cast<const SequenceMotif>(payload);
    if (!m || m->motif_id < 0) return;
    // Track only baked consensus motifs — same stability bar as the action
    // side.  Unbaked drift would defeat the purpose of state-conditional
    // dispatch.
    if (m->is_baked) last_baked_consensus_motif_id_ = m->motif_id;
}

void MotorRepertoire::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    auto const& name = !ev->name.empty()
                        ? ev->name
                        : std::string(topic.substr(std::min(topic.size(),
                            std::string_view("events.").size())));
    if (name == "hit") {
        ++pending_hits_;
        // v5.4 Phase H — eligibility-trace credit.  Distributes hit credit
        // across the last N dispatched chunks (chunk_credit_lookback_),
        // with weight = decay^position back from the most recent.  The
        // back of the deque is the newest dispatch.
        //   newest dispatch       → weight 1.0          (full credit)
        //   prior dispatch        → weight decay        (e.g., 0.5)
        //   two prior dispatch    → weight decay^2      (e.g., 0.25)
        // Replaces the legacy single-chunk credit (active_replay_chunk_id_>0)
        // which (a) excluded seeded chunks (negative ids) and (b) didn't
        // model temporal credit assignment — chunks that "set up" the
        // approach got nothing even though they conditioned the success.
        // Per-position credit (Proposal C) only applies to the currently-
        // replaying chunk (most recent, weight 1.0) since we don't track
        // historical positions for prior dispatches in the trace.
        int trace_n = int(chunk_dispatch_trace_.size());
        bool any_credit = false;
        for (int i = 0; i < trace_n; ++i) {
            int cid = chunk_dispatch_trace_[trace_n - 1 - i];   // i=0 → newest
            float w = std::pow(chunk_credit_decay_, float(i));
            if (w <= 0.0f) continue;
            auto it = chunks_.find(cid);
            if (it == chunks_.end()) continue;
            it->second.replay_hits += w;
            ++eligibility_credits_total_;
            any_credit = true;
            // Per-position credit only for the most recent dispatch (i=0)
            // and only if it matches the chunk currently playing in
            // ActionOut (avoids crediting wrong slot if a chunk re-
            // dispatched between the trace push and this hit event).
            if (i == 0
                && current_replay_chunk_id_ == cid
                && current_replay_position_ >= 0
                && current_replay_position_ < int(it->second.position_hits.size())) {
                it->second.position_hits[current_replay_position_] += w;
                ++position_hits_credited_;
            }
        }
        // v5.4.K — credit modifies published library state; flag dirty so
        // tick() republishes.  Without this, replay_hits updates land in
        // chunks_ but the motor.chunks snapshot (and therefore the UI
        // chunks inspector + C-probe sort list) stays at the previous
        // chunk-creation-event values, leading the operator to think the
        // chunk that just produced a hit "doesn't exist".
        if (any_credit) library_dirty_ = true;
        // Pre-crystallization: credit the most recent baked action motif.
        // It owned the winner slot when contact happened, so action_history_
        // is the success-correlated sequence the chunk should replay (the
        // pivot / approach pattern that produced the hit).  Refresh every
        // hit so multi-hit motifs end up with the most recent successful
        // sequence captured.
        if (last_baked_motif_id_ >= 0) {
            auto it = motifs_.find(last_baked_motif_id_);
            if (it != motifs_.end()) {
                ++it->second.hits_during;
                // v5.3 Phase B — when intent capture is enabled, populate
                // intent_sequence INSTEAD OF action_sequence; chunks become
                // intent plans that ActionDecoder replays via intent.override.
                // When intent capture is disabled, fall back to legacy
                // action capture so pre-v5.3 configs work unchanged.
                // Phase 7.2-EPM — per-position EMA accumulation.  Instead
                // of overwriting target.intent_sequence on every hit (which
                // crystallised brittle single-hit-fluke chunks at cold
                // start), we maintain per-position vote histograms and
                // derive intent_sequence as the argmax per position.  This
                // requires multiple recurring hits with consistent intent
                // patterns before a chunk solidifies — single flukes can't
                // dominate, and the mechanism self-gates against firing
                // until Premotors converge on a real recurring pattern.
                //
                // α derived from crystallization_min_observations_ so it's
                // not a free tuned knob (per the no_tuning directive):
                // half-life = crystallization_min_observations_ hits.
                float alpha_decay_ = 0.5f;
                if (crystallization_min_observations_ > 1) {
                    alpha_decay_ = std::pow(0.5f,
                        1.0f / float(crystallization_min_observations_));
                }
                auto populate_seq = [&](MotifTracking& target) {
                    if (!intent_source_topic_.empty() && !intent_history_.empty()) {
                        // Resize histograms if first time or chunk_max_ticks changed.
                        if (int(target.intent_position_votes.size()) != chunk_max_ticks_)
                            target.intent_position_votes.assign(chunk_max_ticks_, {});
                        // Decay all positions' votes before adding new evidence.
                        for (auto& m : target.intent_position_votes)
                            for (auto& [k, v] : m) v *= alpha_decay_;
                        // Increment current intent_history at right-aligned
                        // positions (history is rolling window of latest ticks;
                        // the most recent tick is intent_history_.back()).
                        int n = std::min(int(intent_history_.size()), chunk_max_ticks_);
                        int offset = chunk_max_ticks_ - n;
                        int hist_start = int(intent_history_.size()) - n;
                        for (int i = 0; i < n; ++i) {
                            int idx = intent_history_[hist_start + i];
                            if (idx < 0) continue;
                            target.intent_position_votes[offset + i][idx] += 1.0f;
                        }
                        // Derive intent_sequence = argmax per position.
                        target.intent_sequence.assign(chunk_max_ticks_, -1);
                        for (int i = 0; i < chunk_max_ticks_; ++i) {
                            int best = -1;
                            float best_v = 0.0f;
                            for (auto const& [k, v] : target.intent_position_votes[i]) {
                                if (v > best_v) { best_v = v; best = k; }
                            }
                            target.intent_sequence[i] = best;   // -1 = no consensus / empty slot
                        }
                        target.action_sequence.clear();   // exclusive with intents
                    } else if (!action_history_.empty()) {
                        // Parallel streaming weighted-mean for action-mode chunks.
                        if (int(target.action_position_sum.size()) != chunk_max_ticks_) {
                            target.action_position_sum.assign(chunk_max_ticks_, 0.0f);
                            target.action_position_weight.assign(chunk_max_ticks_, 0.0f);
                        }
                        for (int i = 0; i < chunk_max_ticks_; ++i) {
                            target.action_position_sum[i]    *= alpha_decay_;
                            target.action_position_weight[i] *= alpha_decay_;
                        }
                        int n = std::min(int(action_history_.size()), chunk_max_ticks_);
                        int offset = chunk_max_ticks_ - n;
                        int hist_start = int(action_history_.size()) - n;
                        for (int i = 0; i < n; ++i) {
                            target.action_position_sum[offset + i]    += action_history_[hist_start + i];
                            target.action_position_weight[offset + i] += 1.0f;
                        }
                        target.action_sequence.assign(chunk_max_ticks_, 0.0f);
                        for (int i = 0; i < chunk_max_ticks_; ++i) {
                            if (target.action_position_weight[i] > 0.0f)
                                target.action_sequence[i] = target.action_position_sum[i]
                                                          / target.action_position_weight[i];
                        }
                    }
                };
                populate_seq(it->second);
                // Also update the already-crystallised chunk if this motif
                // has been promoted — otherwise the published MotorChunks
                // stays at the empty pre-hit snapshot.
                if (it->second.chunk_id >= 0) {
                    auto cit = chunks_.find(it->second.chunk_id);
                    if (cit != chunks_.end()) {
                        populate_seq(cit->second);
                        library_dirty_ = true;
                    }
                }
                // Stamp the perceptual context: this motif's chunk will
                // dispatch when ActionDecoder sees the same consensus
                // motif again.  -1 if no baked consensus has been seen,
                // meaning the chunk fires unconditionally.
                it->second.trigger_consensus_motif_id = last_baked_consensus_motif_id_;
                // Phase 6.5.12 — also stamp drive urgency as a SECOND
                // dispatch trigger.  ActionDecoder fires the chunk when
                // either the consensus context recurs OR current urgency
                // reaches this level.  Drive subscription may not have
                // delivered yet at very-early hits → store -1.0 in that
                // case (chunk has consensus-only trigger, back-compat).
                it->second.trigger_urgency = drive_seen_ ? latest_drive_urgency_ : -1.0f;
            }
        }
    }
    else if (name == "miss" || name == "wall_stuck") {
        ++pending_misses_;
        // Post-crystallization: a miss (or wall_stuck) while this chunk is
        // replaying is direct evidence the chunk's pattern isn't matching
        // its trigger context anymore — Phase 6.5.18 routes wall_stuck
        // here too because chunks that fire during stuck states without
        // breaking the cart free SHOULD demote.  Pre-fix, wall_stuck
        // (the strongest "you're stuck" signal in the substrate) only
        // drained dopamine at NeurochemState — never reached chunk
        // demotion.  Now stuck-perpetuating chunks demote within 1-2
        // dispatches via the Wilson CI gate, instead of needing 5+
        // accumulated low-threshold whisker misses.
        // v5.4 Phase H — credit applies to all chunk_ids != -1 (was > 0,
        // which excluded seeded/episodic chunks with negative ids).  Misses
        // are NOT eligibility-traced — only the chunk currently replaying
        // gets the negative credit; prior chunks in the trace are off the
        // hook for this miss (per the v5.4.H design: trace credit is for
        // hits only).
        if (active_replay_chunk_id_ != -1) {
            for (auto& [cid, ct] : chunks_) {
                if (cid == active_replay_chunk_id_) {
                    ct.replay_misses += 1.0f;
                    library_dirty_ = true;   // v5.4.K — miss credit visible.
                    break;
                }
            }
        }
    }
}

void MotorRepertoire::publish_library_snapshot(uint64_t tick_id) {
    auto snap = std::make_shared<MotorChunks>();
    snap->tick_id     = tick_id;
    snap->producer_id = id_.empty() ? std::string("motor_repertoire") : id_;
    snap->chunks.reserve(chunks_.size());
    for (auto const& [id, mt] : chunks_) {
        MotorChunk c;
        c.id                          = id;
        c.action_sequence             = mt.action_sequence;
        c.intent_sequence             = mt.intent_sequence;   // v5.3 Phase B
        c.entry_embeddings            = mt.entry_embeddings;  // v5.4 Phase A
        c.position_hits               = mt.position_hits;     // v5.4 Phase G
        c.position_misses             = mt.position_misses;
        c.body_keyframes              = mt.body_keyframes;
        c.playback_ticks_per_position = mt.playback_ticks_per_position;
        c.outcome_drive_delta         = mt.drive_delta_ema;
        c.use_count                   = mt.observations;
        c.trigger_consensus_motif_id  = mt.trigger_consensus_motif_id;
        c.trigger_urgency             = mt.trigger_urgency;
        c.hits_during                 = mt.hits_during;
        c.replay_hits                 = mt.replay_hits;
        c.replay_misses               = mt.replay_misses;
        c.created_tick_id             = mt.created_tick_id;   // v5.4.L
        snap->chunks.push_back(std::move(c));
    }
    bus_->publish(topics::kMotorChunks, snap);
}

void MotorRepertoire::tick(uint64_t tick_id) {
    // v5.4 Phase H — per-tick freshness decay.  Slowly drifts replay_hits/
    // _misses (and per-position equivalents) toward 0 so chunk scores
    // reflect RECENT performance, not all-time.  Without this a chunk that
    // scored well early but is now stale never gets re-evaluated; a chunk
    // that misses early but isn't dispatched again stays demoted forever.
    // Half-life = ln(2)/r ticks; default r=1e-4 → ~6931 ticks ≈ 115s @ 60Hz.
    if (chunk_freshness_decay_per_tick_ > 0.0f) {
        float keep = 1.0f - chunk_freshness_decay_per_tick_;
        for (auto& [cid, mt] : chunks_) {
            mt.replay_hits   *= keep;
            mt.replay_misses *= keep;
            for (auto& v : mt.position_hits)   v *= keep;
            for (auto& v : mt.position_misses) v *= keep;
            // hits_during NOT decayed — pre-crystallisation gate signal,
            // not subject to ongoing forgetting.  Pruning handles stale
            // chunks via score-threshold check below.
        }
    }

    // v5.4 Phase H — hard prune.  Chunks below score threshold AND with
    // enough observations get erased entirely (not just is_active=false).
    // Frees the library slot for new chunk crystallisation; closes the
    // GNG-analog lifecycle loop (boost → decay → prune → replace).
    if (chunk_prune_score_threshold_ > 0.0f && !chunks_.empty()) {
        std::vector<int> to_erase;
        for (auto const& [cid, mt] : chunks_) {
            // n_observations rounded so fractional eligibility credit still
            // counts toward the min_dispatches gate.
            int n_obs = int(std::round(mt.replay_hits + mt.replay_misses));
            if (n_obs < chunk_prune_min_dispatches_) continue;
            float h = float(mt.hits_during) + mt.replay_hits + 1.0f;
            float t = h + mt.replay_misses + 1.0f;
            float score = h / t;
            if (score < chunk_prune_score_threshold_) to_erase.push_back(cid);
        }
        if (!to_erase.empty()) {
            for (int cid : to_erase) {
                chunks_.erase(cid);
                ++chunks_pruned_total_;
            }
            // Also drop pruned chunks from the eligibility trace so they
            // don't keep receiving phantom credit on subsequent hits.
            std::deque<int> filtered;
            for (int cid : chunk_dispatch_trace_) {
                if (chunks_.count(cid)) filtered.push_back(cid);
            }
            chunk_dispatch_trace_.swap(filtered);
            library_dirty_ = true;
        }
    }

    // v5.4.K — publish on dirty OR on heartbeat.  The heartbeat bounds
    // the staleness of the published motor.chunks snapshot to
    // publish_period_ticks_ (default 60 = 1 sim-sec) so the UI inspector
    // sees credit-event updates (replay_hits/_misses) within at most one
    // sim-second, even if no chunk-creation/pruning event coincides.
    bool heartbeat_due = (tick_id >= last_publish_tick_ + uint64_t(publish_period_ticks_));
    if (library_dirty_ || heartbeat_due) {
        publish_library_snapshot(tick_id);
        library_dirty_     = false;
        last_publish_tick_ = tick_id;
    }
    pending_hits_   = 0;
    pending_misses_ = 0;
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

namespace {
nlohmann::json motif_to_json(MotorRepertoire::MotifTracking const* mt_ptr) {
    auto const& mt = *mt_ptr;
    nlohmann::json seq = nlohmann::json::array();
    for (auto v : mt.action_sequence) seq.push_back(v);
    nlohmann::json intent_seq = nlohmann::json::array();
    for (auto v : mt.intent_sequence) intent_seq.push_back(v);
    nlohmann::json proto = nlohmann::json::array();
    for (int i = 0; i < mt.entry_state_prototype.size(); ++i)
        proto.push_back(mt.entry_state_prototype(i));
    // Phase 7.2-EPM — serialise per-position vote histograms so the
    // EMA accumulator survives clone/snapshot.
    nlohmann::json votes = nlohmann::json::array();
    for (auto const& m : mt.intent_position_votes) {
        nlohmann::json pos = nlohmann::json::object();
        for (auto const& [k, v] : m) pos[std::to_string(k)] = v;
        votes.push_back(std::move(pos));
    }
    nlohmann::json act_sum = nlohmann::json::array();
    for (auto v : mt.action_position_sum) act_sum.push_back(v);
    nlohmann::json act_w = nlohmann::json::array();
    for (auto v : mt.action_position_weight) act_w.push_back(v);
    return nlohmann::json{
        {"chunk_id",                   mt.chunk_id},
        {"observations",               mt.observations},
        {"hits_during",                mt.hits_during},
        {"action_sequence",            seq},
        {"intent_sequence",            intent_seq},
        {"intent_position_votes",      votes},
        {"action_position_sum",        act_sum},
        {"action_position_weight",     act_w},
        {"entry_state_prototype",      proto},
        {"drive_delta_ema",            mt.drive_delta_ema},
        {"trigger_consensus_motif_id", mt.trigger_consensus_motif_id},
        {"trigger_urgency",            mt.trigger_urgency},
        {"replay_hits",                mt.replay_hits},
        {"replay_misses",              mt.replay_misses},
        {"is_active",                  mt.is_active},
        {"created_tick_id",            mt.created_tick_id},
    };
}

void motif_from_json(MotorRepertoire::MotifTracking& mt, nlohmann::json const& j) {
    mt.chunk_id          = j.value("chunk_id",     mt.chunk_id);
    mt.observations      = j.value("observations", mt.observations);
    mt.hits_during       = j.value("hits_during",  mt.hits_during);
    mt.action_sequence.clear();
    if (j.contains("action_sequence") && j["action_sequence"].is_array())
        for (auto const& v : j["action_sequence"]) mt.action_sequence.push_back(v.get<float>());
    mt.intent_sequence.clear();
    if (j.contains("intent_sequence") && j["intent_sequence"].is_array())
        for (auto const& v : j["intent_sequence"]) mt.intent_sequence.push_back(v.get<int>());
    // Phase 7.2-EPM — restore per-position vote histograms (silently
    // skip if absent so older snapshots round-trip; EMA resumes from
    // empty histograms, which is the same as a freshly-built chunk).
    mt.intent_position_votes.clear();
    if (j.contains("intent_position_votes") && j["intent_position_votes"].is_array()) {
        for (auto const& pos : j["intent_position_votes"]) {
            std::unordered_map<int, float> m;
            if (pos.is_object())
                for (auto it = pos.begin(); it != pos.end(); ++it)
                    m[std::stoi(it.key())] = it.value().get<float>();
            mt.intent_position_votes.push_back(std::move(m));
        }
    }
    mt.action_position_sum.clear();
    if (j.contains("action_position_sum") && j["action_position_sum"].is_array())
        for (auto const& v : j["action_position_sum"]) mt.action_position_sum.push_back(v.get<float>());
    mt.action_position_weight.clear();
    if (j.contains("action_position_weight") && j["action_position_weight"].is_array())
        for (auto const& v : j["action_position_weight"]) mt.action_position_weight.push_back(v.get<float>());
    if (j.contains("entry_state_prototype") && j["entry_state_prototype"].is_array()) {
        auto const& a = j["entry_state_prototype"];
        mt.entry_state_prototype.resize(int(a.size()));
        for (size_t i = 0; i < a.size(); ++i) mt.entry_state_prototype(int(i)) = a[i].get<float>();
    }
    mt.drive_delta_ema             = j.value("drive_delta_ema",            mt.drive_delta_ema);
    mt.trigger_consensus_motif_id  = j.value("trigger_consensus_motif_id", mt.trigger_consensus_motif_id);
    mt.trigger_urgency             = j.value("trigger_urgency",            mt.trigger_urgency);
    // v5.4 Phase H — snapshots written before this phase had int values for
    // these fields; nlohmann json.value() will accept either int or float
    // and coerce on read, so old snapshots restore cleanly.
    mt.replay_hits                 = float(j.value("replay_hits",   double(mt.replay_hits)));
    mt.replay_misses               = float(j.value("replay_misses", double(mt.replay_misses)));
    mt.is_active                   = j.value("is_active",                  mt.is_active);
    mt.created_tick_id             = j.value("created_tick_id",            mt.created_tick_id);
}
} // namespace

nlohmann::json MotorRepertoire::snapshot_state() const {
    nlohmann::json motifs = nlohmann::json::object();
    for (auto const& [k, mt] : motifs_) motifs[std::to_string(k)] = motif_to_json(&mt);
    nlohmann::json chunks = nlohmann::json::object();
    for (auto const& [k, mt] : chunks_) chunks[std::to_string(k)] = motif_to_json(&mt);
    nlohmann::json action_hist = nlohmann::json::array();
    for (auto v : action_history_) action_hist.push_back(v);
    return nlohmann::json{
        {"version",                       1},
        {"action_history",                action_hist},
        {"motifs",                        motifs},
        {"chunks",                        chunks},
        {"next_chunk_id",                 next_chunk_id_},
        {"library_dirty",                 library_dirty_},
        {"latest_drive_urgency",          latest_drive_urgency_},
        {"latest_drive_delta",            latest_drive_delta_},
        {"prev_drive_urgency",            prev_drive_urgency_},
        {"drive_seen",                    drive_seen_},
        {"pending_hits",                  pending_hits_},
        {"pending_misses",                pending_misses_},
        {"last_baked_motif_id",           last_baked_motif_id_},
        {"last_baked_consensus_motif_id", last_baked_consensus_motif_id_},
        {"active_replay_chunk_id",        active_replay_chunk_id_},
        {"replay_hits_at_dispatch_start", replay_hits_at_dispatch_start_},
        {"last_dispatch_request_id",      last_dispatch_request_id_},
        {"last_dispatch_chunk_id",        last_dispatch_chunk_id_},
        {"total_dispatch_count",          total_dispatch_count_},
        {"failed_dispatch_count",         failed_dispatch_count_},
        // v5.4.K — last_publish_tick_ persists so a snapshot-restored
        // graph doesn't re-emit duplicate library snapshots on the first
        // resumed tick.  publish_period_ticks_ is ConstructionOnly so we
        // don't persist it.
        {"last_publish_tick",             last_publish_tick_},
    };
}

void MotorRepertoire::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("MotorRepertoire::restore_state: unknown version " +
                                 std::to_string(version));
    }
    action_history_.clear();
    if (s.contains("action_history") && s["action_history"].is_array())
        for (auto const& v : s["action_history"]) action_history_.push_back(v.get<float>());
    motifs_.clear();
    if (s.contains("motifs") && s["motifs"].is_object()) {
        for (auto it = s["motifs"].begin(); it != s["motifs"].end(); ++it) {
            int id = std::stoi(it.key());
            MotifTracking mt;
            motif_from_json(mt, it.value());
            motifs_[id] = std::move(mt);
        }
    }
    chunks_.clear();
    if (s.contains("chunks") && s["chunks"].is_object()) {
        for (auto it = s["chunks"].begin(); it != s["chunks"].end(); ++it) {
            int id = std::stoi(it.key());
            MotifTracking mt;
            motif_from_json(mt, it.value());
            chunks_[id] = std::move(mt);
        }
    }
    next_chunk_id_                  = s.value("next_chunk_id",                 next_chunk_id_);
    // Force library re-publish on next tick so ActionDecoder's latest_chunks_
    // cache (which we deliberately don't snapshot) gets refreshed.
    library_dirty_                  = true;
    latest_drive_urgency_           = s.value("latest_drive_urgency",          latest_drive_urgency_);
    latest_drive_delta_             = s.value("latest_drive_delta",            latest_drive_delta_);
    prev_drive_urgency_             = s.value("prev_drive_urgency",            prev_drive_urgency_);
    drive_seen_                     = s.value("drive_seen",                    drive_seen_);
    pending_hits_                   = s.value("pending_hits",                  pending_hits_);
    pending_misses_                 = s.value("pending_misses",                pending_misses_);
    last_baked_motif_id_            = s.value("last_baked_motif_id",           last_baked_motif_id_);
    last_baked_consensus_motif_id_  = s.value("last_baked_consensus_motif_id", last_baked_consensus_motif_id_);
    active_replay_chunk_id_         = s.value("active_replay_chunk_id",        active_replay_chunk_id_);
    // v5.4 Phase H — float (was int); accept either via double round-trip.
    replay_hits_at_dispatch_start_  = float(s.value("replay_hits_at_dispatch_start", double(replay_hits_at_dispatch_start_)));
    last_dispatch_request_id_       = s.value("last_dispatch_request_id",      last_dispatch_request_id_);
    last_dispatch_chunk_id_         = s.value("last_dispatch_chunk_id",        last_dispatch_chunk_id_);
    total_dispatch_count_           = s.value("total_dispatch_count",          total_dispatch_count_);
    failed_dispatch_count_          = s.value("failed_dispatch_count",         failed_dispatch_count_);
    last_publish_tick_              = s.value("last_publish_tick",             last_publish_tick_);
}

} // namespace ogma
