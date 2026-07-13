#pragma once

// =============================================================================
// EpisodicCapture.hpp  --  v5.4 Phase A — goal-directed chunk crystallisation
// =============================================================================
//
// Captures the brain's recent trajectory at the slow consensus EPM's keyframe
// rate.  When a configured reward event fires, snapshots the rolling buffer
// into an EpisodicChunkProposal and publishes it on motor.episodic_proposal
// for MotorRepertoire to ingest.
//
// Architectural intent (per user, 2026-05-10):
//   "at the moment of eating, the NT spike must capture the past long horizon
//    sequence ... there needs to be an entry point aspect to every sequence."
//
// Replaces the v5.3 motif-baking-then-coinciding-with-hit crystallisation
// path.  That path captured action_history snapshots that encoded "what
// reflex was doing" not "what produced the hit" — chunks were noise.
// Reward IS the gate now: only sequences that actually ended at a reward
// get crystallised, with their preceding state context preserved for
// cue-based recall.
//
// Two-part chunk shape (per user spec, default sizes):
//   total_keyframes = 5 (~1.25 sec at 250ms/keyframe)
//   entry_keyframes = 2 (first 2 → entry context for cue-based dispatch)
//   body_keyframes  = 3 (remaining → intent sequence to replay)
//
// The keyframe rate is set by the slow consensus EPM's process_every_n_ticks
// (typically 50 = ~833ms at 60Hz).  EpisodicCapture subscribes to the slow
// EPM's RealityToken output and treats each fresh-republish boundary as
// one keyframe.
//
// Module lifecycle: per docs/primitives/_module_lifecycle.md.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

namespace ogma {

class EpisodicCapture : public Module {
public:
    EpisodicCapture();
    ~EpisodicCapture() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests.
    int    keyframes_seen()       const { return keyframes_seen_; }
    int    intents_seen()         const { return intents_seen_; }
    int    proposals_emitted()    const { return proposals_emitted_; }
    int    rewards_seen()         const { return rewards_seen_; }
    int    buffer_fill()          const { return int(keyframe_buffer_.size()); }
    int    last_intent_index()    const { return last_intent_index_; }

private:
    void handle_keyframe(MessagePtr payload);
    void handle_intent(MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    // v5.4.M — secondary fast topic handler (caches latest src vector).
    void handle_secondary(MessagePtr payload);

    // Configuration
    std::string keyframe_topic_         = "reality.slow.consensus";
    std::string intent_topic_           = "policy.intent";
    std::string output_topic_           = "motor.episodic_proposal";
    std::vector<std::string> reward_event_names_ = {"hit", "scent_aligned_with_green"};
    int         total_keyframes_        = 5;
    int         entry_keyframes_        = 2;
    int         playback_ticks_per_intent_ = 50;   // expand each captured intent during emit
    // v5.4 Phase D — mirror augmentation.  When true, EVERY reward fires
    // TWO proposals: the original captured trajectory AND a bilateral
    // mirror (intents 0↔4, 1↔3 swapped; 2 stays).  Doubles the chunk
    // pool size and balances the L/R distribution per-reward, addressing
    // the chunk-diversity bias observed at brain-only α (chunks all
    // turning one direction because reflex chemotaxis on a given seed
    // happens to predominantly approach food from one side).
    bool        mirror_augment_         = false;
    // v5.4.L final — when true, snapshot the keyframe topic's
    // winner_prototype (the GNG-quantized prototype centroid) instead of
    // latent (raw encoder output).  Diagnostic B showed identity-encoder
    // EPMs publish raw input as latent; the GNG's discriminating power
    // lives in winner_prototype.  Switching to winner_prototype lets
    // chunks captured at distinct GNG cells have meaningfully different
    // entry vectors even when the underlying input signal is smooth.
    // Skips keyframes where winner_prototype is empty (winner_id<0,
    // bootstrap or single-node GNG).  Default false = legacy latent.
    bool        entry_use_winner_prototype_ = false;
    // v5.4.M — short+long entry fusion.  When secondary_keyframe_topic_
    // is non-empty, subscribe to it and cache its latest published
    // latent / winner_prototype (chosen per entry_use_winner_prototype_).
    // At keyframe capture time, concat (primary || secondary) into the
    // chunk's entry_embeddings.  Bridges the slow-consensus averaging
    // floor: primary is the long (smoothed) signal, secondary is a
    // per-tick fast signal (e.g., reality.scent.scent) that reflects
    // momentary environment state without motor-noise averaging.
    // ActionDecoder must have a matching secondary_entry_topic for the
    // chunks' entry_embeddings and the runtime entry_history to share
    // vector space.  Empty default preserves legacy single-source.
    std::string secondary_keyframe_topic_ = "";

    // Rolling buffer of last total_keyframes_ tuples.  Each tuple records
    // the slow-consensus embedding observed at that keyframe boundary plus
    // the most-recent intent index seen in the interval.  The buffer is
    // append-only between captures; a reward event triggers a snapshot
    // (does NOT clear the buffer — overlapping reward events can capture
    // overlapping windows, which is desired).
    struct Keyframe {
        Eigen::VectorXf embedding;
        int             intent_index = -1;
        uint64_t        keyframe_tick_id = 0;
    };
    std::deque<Keyframe> keyframe_buffer_;
    int                  last_intent_index_  = -1;
    uint64_t             last_keyframe_tick_ = uint64_t(-1);
    // v5.4.L final — track the raw latent vector (not the buffer's chosen
    // src) for slow-EPM republish detection.  Independent of whether the
    // buffer captures latent or winner_prototype — republish detection
    // must always use the field that's bit-identical on republish.
    Eigen::VectorXf      last_keyframe_latent_;
    // v5.4.M — cached latest from the secondary entry topic.
    Eigen::VectorXf      last_secondary_src_;

    // Telemetry counters
    int keyframes_seen_    = 0;
    int intents_seen_      = 0;
    int proposals_emitted_ = 0;
    int rewards_seen_      = 0;
};

} // namespace ogma
