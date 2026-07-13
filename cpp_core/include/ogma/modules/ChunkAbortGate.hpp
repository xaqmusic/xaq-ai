#pragma once

// =============================================================================
// ChunkAbortGate.hpp  --  v5.3 Phase D — surprise-driven chunk termination
// =============================================================================
//
// Watches the LateralVoter's `consensus.<level>` for per-modality surprise_ema
// (Phase 6.6.E predicted_pathway error EMA, surfaced in ConsensusToken).
// Maintains a baseline EMA of mean per-tick surprise.  When surprise spikes
// above `baseline + k * sigma` for `min_consecutive_ticks` consecutive ticks,
// publishes `events.chunk_abort`.  ActionDecoder (when configured to listen)
// terminates its current chunk replay → next tick's try_dispatch_chunk runs
// fresh selection.
//
// Architectural role: closes the multi-rate brain story.  Chunks commit the
// substrate to a slow-horizon (~833 ms) plan; the abort gate breaks that
// commitment when fast-horizon prediction error spikes (the world diverges
// from what the chunk's plan assumed).  Without it, chunks run to completion
// even when the brain knows mid-stream the plan no longer fits.
//
// Default-disabled (k_sigma=0 = never fires); explicit opt-in via config.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ChunkAbortGate : public Module {
public:
    ChunkAbortGate();
    ~ChunkAbortGate() override;

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
    int   aborts_total()       const { return aborts_total_; }
    float baseline_mean()      const { return baseline_mean_; }
    float baseline_var()       const { return baseline_var_; }
    float last_surprise_value()const { return last_surprise_; }
    int64_t last_abort_tick()  const { return last_abort_tick_; }

private:
    void handle_consensus(MessagePtr payload);

    // Configuration
    std::string consensus_topic_       = "consensus.0";
    std::string output_event_name_     = "chunk_abort";
    std::string output_topic_          = "events.chunk_abort";   // derived
    float       baseline_alpha_        = 0.005f;   // slow EMA over baseline surprise
    float       k_sigma_               = 2.0f;     // threshold = mean + k*sqrt(var); 0 = disabled
    int         min_consecutive_ticks_ = 5;
    int         refractory_ticks_      = 60;

    // Working state
    bool   baseline_init_       = false;
    float  baseline_mean_       = 0.0f;
    float  baseline_var_        = 0.0f;
    float  last_surprise_       = 0.0f;
    int    above_thresh_streak_ = 0;
    int    refractory_remaining_= 0;
    int    aborts_total_        = 0;
    int64_t last_abort_tick_    = -1;
};

} // namespace ogma
