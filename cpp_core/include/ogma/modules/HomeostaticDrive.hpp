#pragma once

// =============================================================================
// HomeostaticDrive.hpp  --  Module 4 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/HomeostaticDrive.md
// v3 reference:    none (new primitive — closest analog is the v3
//                  NodeValenceMap, which v4 reinterprets and re-homes inside
//                  ActionDecoder).
//
// HomeostaticDrive is the bath's goal-pressure source.  Each tick it
// computes per-channel setpoint deviations and a single urgency scalar,
// publishing them on `drive.errors` for ActionDecoder's drive-grounded EFE
// policy.  The channel set is declarative: the default body schema has
// energy, integrity, and novelty_satiation, but extra channels can be added
// per body via the graph config.
//
// Channel kinds (selected per channel by name → default kind):
//
//   "energy"            internal model: passive drain per tick, replenish
//                        on `events.hit`, optional sync from
//                        reality.proprio.energy.
//   "integrity"         internal model: drain on `events.miss`, optional
//                        sync from reality.proprio.integrity.
//   "novelty_ema"       current = EMA of consensus.<ema_source_level>.fused_tle.
//   "proprio_passive"   current = ProprioToken.values[0]; no internal update.
//   "alive_pulse"       internal model: passive decay per tick, replenish on
//                        `events.alive` (closes deficit toward setpoint), hard
//                        zero on `events.failed`. Designed for CartPole-style
//                        per-tick survival reward — see
//                        docs/v4_phase6_5_2_plan.md §3.3.
//
// Hosts add channels by extending the `channels`, `setpoints`,
// `urgency_normalizers`, `channel_input_topics`, and `channel_kinds`
// parameter arrays in lock-step.

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class HomeostaticDrive : public Module {
public:
    HomeostaticDrive();
    ~HomeostaticDrive() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Read-only accessors for white-box tests.
    float current_value(std::string const& channel) const;
    float urgency() const { return urgency_; }
    int   channel_count() const { return int(channels_.size()); }

private:
    enum class ChannelKind { Energy, Integrity, NoveltyEma, ProprioPassive, AlivePulse };

    struct Channel {
        std::string  name;
        std::string  input_topic;
        ChannelKind  kind                 = ChannelKind::ProprioPassive;
        float        setpoint             = 0.0f;
        float        urgency_normalizer   = 1.0f;
        float        current              = 0.0f;
        bool         proprio_seen         = false;
        bool         consensus_seen       = false;
        float        latest_proprio_value = 0.0f;
        float        latest_consensus_tle = 0.0f;
    };

    void   handle_proprio(std::string_view topic, MessagePtr payload, size_t channel_idx);
    void   handle_consensus(std::string_view topic, MessagePtr payload);
    void   handle_event(std::string_view topic, MessagePtr payload);
    static ChannelKind parse_kind(std::string const& s);
    static ChannelKind default_kind_for_name(std::string const& name);

    // Configuration
    std::vector<Channel>                channels_;
    std::unordered_map<std::string, size_t> channel_idx_by_name_;

    // Behaviour parameters
    double energy_replenish_per_hit_  = 0.4;
    double energy_drain_per_tick_     = 0.0005;
    double integrity_drain_per_miss_  = 0.05;
    double alive_pulse_decay_per_tick_      = 0.005;
    double alive_pulse_replenish_per_event_ = 0.5;
    double novelty_satiation_alpha_   = 0.001;
    double urgency_clamp_lo_          = 0.0;
    double urgency_clamp_hi_          = 1.0;
    int    ema_source_level_          = 0;     // which consensus.<n> drives novelty_ema

    // Working state
    float urgency_         = 0.0f;
    int   pending_hits_    = 0;
    int   pending_misses_  = 0;
    int   pending_alive_   = 0;
    int   pending_failed_  = 0;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).
    std::string subscribed_consensus_topic_;

public:
    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
