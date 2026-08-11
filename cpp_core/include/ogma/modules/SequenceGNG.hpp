#pragma once

// =============================================================================
// SequenceGNG.hpp  --  Module 7 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/SequenceGNG.md
// v3 reference:    none.  Wraps cpp_core/include/v3/gng.hpp as the underlying
//                  clusterer.
//
// Clusters n-grams of winner transitions (or action scalars) instead of
// single states.  The encoding pipeline:
//
//   source_kind = "winner":
//     deterministic hash(winner_id) → R^prototype_per_winner_dim
//     concat N most-recent → R^(N * prototype_per_winner_dim)
//     JL project → R^projection_dim
//     GNG step
//
//   source_kind = "action":
//     window of N most-recent action scalars (each = ActionOut.accel)
//     JL project R^N → R^projection_dim
//     GNG step
//
// Successor tracking: for the active baked motif, maintain count(next_winner)
// across observations and report the argmax as predicted_next_id in the
// published SequenceMotif.

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include "v3/encoder_jl.hpp"
#include "v3/gng.hpp"

namespace ogma {

class SequenceGNG : public Module {
public:
    SequenceGNG();
    ~SequenceGNG() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    enum class SourceKind { Winner, Action, Intent };  // Intent = v5.3 Phase B

    int  node_count()    const { return gng_ ? gng_->node_count() : 0; }
    int  baked_count()   const { return gng_ ? gng_->baked_count() : 0; }
    int  current_motif() const { return current_motif_id_; }
    long n_events()      const { return n_events_; }

private:
    void handle_source(std::string_view topic, MessagePtr payload);
    void handle_neuro(std::string_view topic, MessagePtr payload);

    Eigen::VectorXf hash_winner(int winner_id) const;
    bool            encode_window(Eigen::VectorXf& out) const;
    void            track_successor(int active_motif, int next_winner);

    // Configuration
    std::string  source_topic_;
    SourceKind   source_kind_              = SourceKind::Winner;
    int          window_size_              = 5;
    int          projection_dim_           = 128;
    int          prototype_per_winner_dim_ = 32;
    float        motif_branching_threshold_ = 0.4f;
    uint64_t     master_seed_              = 0;
    std::string  output_topic_;
    // 2026-08-11 (twin-gate S0) — EVENT MODE: push the window only when the
    // winner CHANGES, and step the GNG only when the window changed.  On a
    // dwell-heavy stream (body-pose self-transition mass 0.72) the legacy
    // per-tick windows are dwell runs — one plausible reason the picrawler-era
    // seqgng never baked a motif.  Default false = byte-identical legacy.
    bool         event_mode_               = false;

    // Phase 6.5.3.2 — optional context-stability gate.
    // When `context_topic_` is non-empty, the module subscribes to a
    // SequenceMotif topic (e.g. `sequence.motif.consensus.0`) and only
    // runs GNG updates / publishes a motif when the context's motif_id
    // has been the same for the past `window_size_` ticks.  This
    // ensures action-motif clusters form within stable perceptual
    // contexts rather than across context transitions where pattern
    // recurrence is noise.  Empty string disables the gate (default;
    // back-compat with existing configs).
    std::string  context_topic_;
    std::deque<int> context_history_;
    int          last_context_motif_id_ = -1;

    // Encoders
    std::unique_ptr<ami_ogma::v3::FrozenJLEncoder>  windowed_jl_;
    std::unique_ptr<ami_ogma::v3::GNG>              gng_;

    // Working state
    std::deque<int>           winner_window_;
    std::deque<float>         action_window_;
    int                       last_winner_id_   = -1;
    int                       current_motif_id_ = -1;
    bool                      window_dirty_     = false;
    long                      n_events_         = 0;
    int                       motif_phase_      = 0;
    int                       motif_length_     = 0;
    float                     match_confidence_ = 0.0f;
    bool                      just_baked_       = false;

    // Successor tracking: motif_id → map<next_winner_id, count>.
    std::unordered_map<int, std::unordered_map<int, int>> successor_counts_;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot()  const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
