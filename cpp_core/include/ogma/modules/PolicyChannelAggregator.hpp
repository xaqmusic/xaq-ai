#pragma once

// =============================================================================
// PolicyChannelAggregator.hpp  --  Phase 7.2-EPM multi-channel chunk support
// =============================================================================
//
// Packs N per-joint PolicyTokens into one combined PolicyToken so a single
// MotorRepertoire can crystallise multi-joint coordination chunks without
// requiring vector-typed intent_history.  Combined index uses radix encoding:
//
//      combined = c[0] + c[1] * R[0] + c[2] * R[0] * R[1] + ...
//
// where R[i] is `n_intents` per input channel.  Inverse unpack happens in
// ActionDecoder when it dispatches the chunk: the combined index becomes
// IntentToken.indices, and each downstream Premotor reads indices[its_channel].
//
// Lifecycle / Bus contract per docs/primitives/_module_lifecycle.md.

#include <cstdint>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class PolicyChannelAggregator : public Module {
public:
    PolicyChannelAggregator();
    ~PolicyChannelAggregator() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    int  n_channels()      const { return int(input_topics_.size()); }
    int  total_publishes() const { return total_publishes_; }
    std::vector<int> const& channel_radix() const { return channel_radix_; }

private:
    void handle_policy(int channel, MessagePtr payload);

    // Configuration
    std::vector<std::string> input_topics_;        // N per-joint PolicyToken topics
    std::vector<int>         channel_radix_;       // n_intents per channel (e.g., [5,5,5])
    std::string              output_topic_       = "policy.intent.aggregated";

    // Working state
    std::vector<int>         latest_;              // size = N, latest chosen_intent per channel
    int                      total_publishes_     = 0;
};

} // namespace ogma
