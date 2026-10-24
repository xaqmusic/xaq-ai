// =============================================================================
// FeatureWindow.hpp  --  several topics summarised over a sliding window, as one vector (generator_and_belief_filter.md
// §3.3, stage G3)
// =============================================================================
//
// A slow EPM over "how the gait is going" needs one input stream; the evidence lives on several topics at the body's
// own rate.  This module keeps, for each selected channel, the mean and the standard deviation over the last `window`
// ticks, and publishes their concatenation every `publish_every` ticks.  Channels are named `topic` (every value) or
// `topic:i,j,...` (those indices).  Output order: for each source in order, its channels' means, then their sds.
// A sensor reduction, not a learner: it carries no state beyond the window.
// =============================================================================
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class FeatureWindow : public Module {
public:
    FeatureWindow();
    ~FeatureWindow() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    int output_dim() const { return out_dim_; }

private:
    struct Source {
        std::string topic;
        std::vector<int> idx;              // empty = every value
        Eigen::VectorXf last;
        bool seen = false;
        std::deque<Eigen::VectorXf> win;   // selected channels, one entry per tick
    };
    void handle(size_t s, MessagePtr p);

    std::vector<std::string> spec_;
    std::vector<Source> src_;
    std::string output_topic_ = "reality.proprio.gait_window";
    int window_ = 100, publish_every_ = 25;
    int out_dim_ = 0;
    int64_t ticks_ = 0, published_ = 0;
    Eigen::VectorXf last_out_;
};

} // namespace ogma
