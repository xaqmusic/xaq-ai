#pragma once

// =============================================================================
// MotorRepertoire.hpp  --  Module 9 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/MotorRepertoire.md
// v3 reference:    none.  Action-stream chunk library with REQ/REP playback.
//
// Observes the ActionDecoder's `action.out` stream and the
// `sequence.motif.action.out` topic produced by a SequenceGNG configured for
// actions.  When a motif recurs frequently AND its surrounding drive
// trajectory is favourable (drive-reducing), the motif crystallises as a
// chunk with a stable chunk_id.  ActionDecoder dispatches chunks via
// `motor.play.cmd` and gets back the chunk's action stream on
// `motor.play.stream`.
//
// Phase 1.9 simplifications:
//   - "Drive trajectory" is approximated by the latest urgency at the time
//     of motif observation (no per-motif post-hoc rolling window — that's
//     a Phase 3 stretch).
//   - ActionDecoder integration (motor.play.cmd issuance + suppression of
//     scalar emission during playback) is Phase 3.  The library + REQ/REP
//     surface ship here so the integration can land without contract churn.

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class MotorRepertoire : public Module {
public:
    MotorRepertoire();
    ~MotorRepertoire() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Public state struct — exposed so the snapshot/restore code and
    // white-box tests can serialise individual chunks without friend-class
    // plumbing.  Defined fully in the private: section below.
    struct MotifTracking {
        int                 chunk_id          = -1;
        int                 observations      = 0;
        int                 hits_during       = 0;
        std::vector<float>  action_sequence;
        std::vector<int>    intent_sequence;   // v5.3 Phase B — exclusive with action_sequence
        // Phase 7.2-EPM — per-position evidence accumulators that
        // crystallise across multiple events.hit firings instead of
        // overwriting on every hit.  intent_sequence/action_sequence
        // remain the canonical chunk content (ActionDecoder reads
        // those); these histograms are the SOURCE the canonical
        // sequence is re-derived from on each hit.
        // intent_position_votes[i] : map intent_idx → accumulated EMA weight at position i
        std::vector<std::unordered_map<int, float>> intent_position_votes;
        // action_position_sum[i] / action_position_weight[i] = streaming weighted mean
        std::vector<float>  action_position_sum;
        std::vector<float>  action_position_weight;
        std::vector<Eigen::VectorXf> entry_embeddings;   // v5.4 Phase A — episodic chunk entry context
        // v5.4 Phase G (Proposal C) — per-position Beta-prior credit.
        // v5.4 Phase H — float to accept eligibility-trace fractional credit.
        std::vector<float>  position_hits;
        std::vector<float>  position_misses;
        int                 body_keyframes              = 0;
        int                 playback_ticks_per_position = 1;
        Eigen::VectorXf     entry_state_prototype;
        float               drive_delta_ema   = 0.0f;
        int                 trigger_consensus_motif_id = -1;
        // Phase 6.5.12 — hybrid trigger: drive urgency stamped at hit time.
        // Mirror of the published MotorChunk.trigger_urgency field; copied
        // into the chunk on crystallisation.  -1.0f = no urgency captured
        // (e.g., drive subscription hasn't delivered yet).
        float               trigger_urgency    = -1.0f;
        // v5.4 Phase H — float to accept eligibility-trace fractional credit
        // and per-tick freshness decay.
        float               replay_hits        = 0.0f;
        float               replay_misses      = 0.0f;
        bool                is_active          = true;
        // v5.4.L — tick at which this chunk first entered chunks_.  Used
        // by ActionDecoder's age gate to suppress eat→replay loops on
        // freshly-captured chunks regardless of the entry-match cosine.
        uint64_t            created_tick_id    = 0;
    };

    // White-box accessors
    size_t chunk_count() const { return chunks_.size(); }
    bool   has_chunk(int chunk_id) const { return chunks_.count(chunk_id) > 0; }
    int    total_dispatch_count() const { return total_dispatch_count_; }
    int    failed_dispatch_count() const { return failed_dispatch_count_; }
    int    intent_history_size() const { return int(intent_history_.size()); }
    int    intents_received_total() const { return intents_received_total_; }
    int    episodic_proposals_ingested() const { return episodic_proposals_ingested_; }
    int    position_hits_credited()      const { return position_hits_credited_; }
    // v5.4 Phase H — chunk lifecycle diag.
    int    chunks_pruned_total()         const { return chunks_pruned_total_; }
    int    eligibility_credits_total()   const { return eligibility_credits_total_; }
    int    chunk_dispatch_trace_size()   const { return int(chunk_dispatch_trace_.size()); }
    int    active_chunk_count() const {
        int n = 0;
        for (auto const& [id, mt] : chunks_) if (mt.is_active) ++n;
        return n;
    }
    float  dispatch_success_rate() const {
        if (total_dispatch_count_ <= 0) return 0.0f;
        return 1.0f - float(failed_dispatch_count_) / float(total_dispatch_count_);
    }

private:
    void handle_action(std::string_view topic, MessagePtr payload);
    void handle_intent_source(std::string_view topic, MessagePtr payload);   // v5.3 Phase B
    void handle_episodic_proposal(std::string_view topic, MessagePtr payload);   // v5.4 Phase A
    void handle_motif(std::string_view topic, MessagePtr payload);
    void handle_consensus_motif(std::string_view topic, MessagePtr payload);
    void handle_drive(std::string_view topic, MessagePtr payload);
    void handle_play_cmd(std::string_view topic, MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);

    void publish_library_snapshot(uint64_t tick_id);

    // Phase 6.5.12 — implicit-miss + Wilson CI demotion at dispatch
    // boundary.  Called from handle_play_cmd before starting the new
    // dispatch to evaluate the just-ending one.
    void evaluate_dispatch_outcome_(int chunk_id);

    // (MotifTracking is declared in the public: section above so snapshot
    // helpers and white-box tests can construct/inspect it without friend
    // declarations.  See the public-section definition for field-level
    // documentation.)

    // Configuration
    int      max_chunks_                          = 256;
    int      chunk_max_ticks_                     = 20;
    int      crystallization_min_observations_    = 10;
    float    crystallization_min_drive_delta_     = 0.05f;
    int      crystallization_drive_window_ticks_  = 50;
    float    interrupt_urgency_threshold_         = 0.85f;
    float    interrupt_outcome_divergence_        = 0.30f;
    uint64_t master_seed_                         = 0;

    // v5.3 Phase B — when intent_source_topic_ is non-empty, MotorRepertoire
    // captures intent indices from that topic (default policy.intent;
    // payload PolicyToken or IntentToken) into intent_history_ and uses it
    // to populate intent_sequence on crystallisation.  When also
    // intent_motif_topic_ is non-empty, motif observations from that topic
    // (instead of sequence.motif.action.out) drive the crystallisation
    // gates.  Empty defaults preserve pre-v5.3 behaviour exactly.
    std::string intent_source_topic_                  = "";
    std::string intent_motif_topic_                   = "";

    // v5.4 Phase H — chunk lifecycle (eligibility trace + freshness decay
    // + hard prune).  All HotMutable so they're tunable from the UI.
    //   chunk_credit_lookback           = N most-recent dispatched chunks
    //                                     to credit on each events.hit;
    //                                     newest gets weight 1.0, prior
    //                                     gets decay^position weights.
    //   chunk_credit_decay              = geometric decay applied per
    //                                     position back from newest.
    //   chunk_freshness_decay_per_tick  = per-tick multiplicative decay
    //                                     applied to replay_hits/_misses
    //                                     (and position_hits/_misses) of
    //                                     ALL chunks.  Half-life =
    //                                     ln(2)/r ticks ≈ 6931 at
    //                                     r=1e-4 (~115s sim @ 60Hz).
    //   chunk_prune_score_threshold     = Beta-prior score below which a
    //                                     chunk is hard-erased from the
    //                                     library (not just demoted).
    //   chunk_prune_min_dispatches      = chunk needs this many total
    //                                     (replay_hits+replay_misses,
    //                                     rounded) before pruning is
    //                                     considered.  Protects fresh
    //                                     chunks from premature death.
    int      chunk_credit_lookback_              = 3;
    float    chunk_credit_decay_                 = 0.5f;
    float    chunk_freshness_decay_per_tick_     = 0.0001f;
    float    chunk_prune_score_threshold_        = 0.3f;
    int      chunk_prune_min_dispatches_         = 5;

    // Working state
    std::deque<float>                                action_history_;
    std::deque<int>                                  intent_history_;   // v5.3 Phase B
    int                                              intents_received_total_ = 0;
    int                                              episodic_proposals_ingested_ = 0;
    // v5.4 Phase A — when true, accept EpisodicChunkProposal messages on
    // motor.episodic_proposal and crystallise them directly into chunks
    // (bypassing the v5.3 motif-baking gate).
    bool                                             accept_episodic_ = false;
    // v5.4 Phase G — currently-replaying chunk + position, observed
    // from ActionOut on action_topic_.  Used to credit position_hits /
    // position_misses on the right chunk + slot when events.hit fires.
    int                                              current_replay_chunk_id_ = -1;
    int                                              current_replay_position_ = -1;
    int                                              position_hits_credited_  = 0;
    std::unordered_map<int, MotifTracking>           motifs_;
    std::unordered_map<int, MotifTracking>           chunks_;   // by chunk_id
    int                                              next_chunk_id_       = 1;
    bool                                             library_dirty_       = true;
    float                                            latest_drive_urgency_= 0.0f;
    float                                            latest_drive_delta_  = 0.0f;
    float                                            prev_drive_urgency_  = 0.0f;
    bool                                             drive_seen_          = false;
    int                                              pending_hits_        = 0;
    int                                              pending_misses_      = 0;
    // Most recently observed BAKED motif id — what a hit event credits.
    int                                              last_baked_motif_id_ = -1;
    // Most recently observed BAKED consensus (perceptual) motif id —
    // stamped on the action motif's tracker at hit time so chunks key
    // on the perceptual context that produced the success.
    int                                              last_baked_consensus_motif_id_ = -1;
    // Currently-replaying chunk id, observed via ActionOut.chunk_id.  When
    // > 0, events.hit / events.miss are also credited to this chunk's
    // replay_hits / replay_misses for post-crystallization outcome tracking.
    int                                              active_replay_chunk_id_ = -1;
    // Phase 6.5.12 — implicit-miss detection (request_id-keyed, since
    // chunk_id-edges miss back-to-back re-dispatches of the same chunk).
    // At each MotorPlayCmd, evaluate the PREVIOUS dispatch: if events.hit
    // didn't fire during it (replay_hits unchanged from dispatch_start),
    // increment replay_misses and check Wilson CI for demotion.  Restores
    // the demotion path in continuous-mode envs where events.miss doesn't
    // fire.
    // v5.4 Phase H — float to track snapshot when replay_hits is float.
    float                                            replay_hits_at_dispatch_start_ = 0.0f;
    uint64_t                                         last_dispatch_request_id_      = 0;
    int                                              last_dispatch_chunk_id_        = -1;

    // v5.4 Phase H — eligibility trace: rolling window of the last
    // chunk_credit_lookback_ dispatched chunk_ids (back = newest).  On
    // events.hit, every entry receives weight = decay^position credit on
    // its replay_hits.  Force-probe dispatches (cmd->force) are excluded
    // — manual probes shouldn't influence training credit.
    std::deque<int>                                  chunk_dispatch_trace_;
    int                                              eligibility_credits_total_ = 0;
    int                                              chunks_pruned_total_       = 0;

    // v5.4.K — periodic library-snapshot heartbeat.  Without this the
    // motor.chunks topic was only republished on chunk creation /
    // pruning / Wilson demote.  Credit events (replay_hits++,
    // replay_misses++) updated chunks_ but the published snapshot
    // stayed stale, so the UI chunks inspector + C-probe sort list
    // showed obsolete replay_hits values — the chunk that just
    // produced a hit was invisible until the next library_dirty event.
    // last_publish_tick_ tracks the tick of the most recent snapshot
    // publish; tick() publishes if (library_dirty_) OR
    // (tick_id - last_publish_tick_ >= publish_period_ticks_), whichever
    // comes first.  Default period 60 ticks = 1 sim-sec.
    uint64_t                                         last_publish_tick_         = 0;
    int                                              publish_period_ticks_      = 60;

    // Phase 6.5.3.3 — substrate-level dispatch metrics.  Incremented at
    // dispatch boundaries (entry into and end of each chunk replay).
    // dispatch_success_rate() returns 1 - failed/total.
    int                                              total_dispatch_count_   = 0;
    int                                              failed_dispatch_count_  = 0;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
