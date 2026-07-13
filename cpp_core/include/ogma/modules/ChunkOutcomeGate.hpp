#pragma once

// =============================================================================
// ChunkOutcomeGate.hpp  --  v5.3 Phase F — outcome-driven chunk termination
// =============================================================================
//
// Counterpart to ChunkAbortGate: instead of firing on surprise spikes (which
// don't fire when a chunk locks the agent into a predictable behaviour like
// spinning), this gate fires when the configured outcome signal hasn't
// improved in the desired direction over the chunk's recent ticks.
//
// Mechanism:
//   - Subscribe to outcome_topic (e.g., reality.proprio.scent_max).
//   - Subscribe to action_topic (e.g., action.out) for ActionOut.chunk_id.
//   - When chunk_id transitions from -1 → ≥0: a new chunk just started.
//     Record signal_at_start = current_outcome_value.
//   - When chunk_id stays ≥0: every tick, check if (current - start) has
//     moved by improvement_threshold in target_sign direction within
//     min_check_ticks of dispatch.  If not, fire events.<output_event_name>.
//   - When chunk_id transitions back to -1 (chunk done): clear baseline.
//
// Defaults (target_sign=rising, improvement_threshold=0): "if scent hasn't
// risen at all in the last 30 ticks of chunk replay, that chunk isn't
// helping — abort and let dispatcher pick a different one."
//
// Different from ChunkAbortGate: surprise-driven (predicted vs actual
// sensors) vs outcome-driven (target signal not moving).  Both can coexist;
// they fire on different failure modes.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class ChunkOutcomeGate : public Module {
public:
    ChunkOutcomeGate();
    ~ChunkOutcomeGate() override;

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
    int   aborts_total()        const { return aborts_total_; }
    int   action_msgs_seen()    const { return action_msgs_seen_; }
    int   active_chunk_id()     const { return active_chunk_id_; }
    int   ticks_in_chunk()      const { return ticks_in_chunk_; }
    float signal_at_start()     const { return signal_at_start_; }
    float current_signal()      const { return current_signal_; }

private:
    void handle_outcome(MessagePtr payload);
    void handle_action(MessagePtr payload);

    // Configuration
    std::string outcome_topic_         = "reality.proprio.scent_max";
    int         outcome_index_         = 0;
    std::string action_topic_          = "action.out";
    std::string output_event_name_     = "chunk_abort";
    std::string output_topic_          = "events.chunk_abort";  // derived
    std::string target_sign_           = "rising";  // "rising" or "falling"
    int         min_check_ticks_       = 30;
    float       improvement_threshold_ = 0.0f;
    int         refractory_ticks_      = 60;

    // Working state
    int     active_chunk_id_     = -1;
    int     ticks_in_chunk_      = 0;
    float   signal_at_start_     = 0.0f;
    float   current_signal_      = 0.0f;
    bool    signal_seen_         = false;
    int     refractory_remaining_= 0;
    int     aborts_total_        = 0;
    int     action_msgs_seen_    = 0;
    int64_t last_abort_tick_     = -1;
};

} // namespace ogma
