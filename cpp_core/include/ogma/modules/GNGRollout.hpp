#pragma once

// =============================================================================
// GNGRollout.hpp  --  Module 8 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/GNGRollout.md
// v3 reference:    none (replaces v3's argmax-chained EFE rollout).
//
// REQ/REP rollout service.  Given a query (source_modality, winner_id,
// action, M steps, K samples), samples K stochastic trajectories M steps
// forward via weighted edge traversal over a cached per-source GNG
// transition map and returns the empirical distribution.
//
// Phase 1.8 simplifications:
//   - Transition cache is built incrementally from incoming RealityTokens
//     (consecutive winner_ids in the same source → one increment).
//   - terminal_values are filled from drive.errors urgency (heuristic
//     — Phase 3 wires the v4 valence map directly).
//   - Motif teleport is documented but disabled by default; the SequenceGNG
//     subscription is registered so Phase 3 can flip it on without
//     contract churn.

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class GNGRollout : public Module {
public:
    GNGRollout();
    ~GNGRollout() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Read-only accessors for white-box tests.
    size_t known_sources() const { return per_source_.size(); }
    size_t known_nodes(std::string const& source) const;

private:
    void handle_reality(std::string_view topic, MessagePtr payload);
    void handle_query(std::string_view topic, MessagePtr payload);
    void handle_sequence(std::string_view topic, MessagePtr payload);
    void handle_drive(std::string_view topic, MessagePtr payload);

    struct PerSource {
        // last winner_id seen for this source (used to score transitions)
        int last_winner = -1;
        // counts: from -> map<to, count>
        std::unordered_map<int, std::unordered_map<int, int>> trans;
    };

    std::vector<int>
    sample_trajectory(PerSource const& src, int start, int steps, std::mt19937_64& rng);
    float compute_entropy(std::vector<std::vector<int>> const& trajectories) const;

    // Configuration
    int      K_default_                       = 32;
    int      M_default_                       = 5;
    float    transition_smoothing_            = 0.01f;
    bool     motif_teleport_enabled_          = false;
    float    motif_teleport_min_confidence_   = 0.5f;
    int64_t  value_window_ticks_              = 100;
    int64_t  max_concurrent_queries_          = 4;
    uint64_t master_seed_                     = 0;

    // Working state
    std::unordered_map<std::string, PerSource>    per_source_;
    float                                         latest_drive_urgency_ = 0.0f;

    std::mt19937_64                               rng_;
    int                                           queries_this_tick_    = 0;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
