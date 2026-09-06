// =============================================================================
// TofAvoidLoop.hpp  --  avoidance as a LOOP with a bearing (the duck, §17.6 → item 2)
// =============================================================================
//
// The duck's avoidance was a prior in the level-2 C matrix: a pull on the twist, not a
// policy with a direction.  When novelty became a direction (R27) the two conflicted
// inside one matrix and the learned direction won: the body drove at a moved wall.  The
// Cell recipe arbitrates POLICIES, each a loop with a bearing and a confidence; this module
// makes avoidance one.  From a proximity vector [ahead-left, ahead, ahead-right, too_close]
// (each in [0,1], 1 = touching) it publishes an egocentric bearing AWAY from the nearest
// obstacle -- cx = +right, cy = +forward, the HeadingController / IntentAdapter contract --
// with magnitude = the nearest proximity, and the same scalar as its value (its NEED: the
// arbiter's preference for this loop).  Nothing near → a zero bearing and a zero value: the
// loop has nothing to say and the arbiter hears nothing.  Module absent = byte-identical.
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class TofAvoidLoop : public Module {
public:
    TofAvoidLoop();
    ~TofAvoidLoop() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;
    void           restore_state(nlohmann::json const& s) override;

    float last_cx() const { return cx_; }
    float last_cy() const { return cy_; }
    float last_value() const { return value_; }

private:
    void handle_prox(MessagePtr p);

    std::string prox_topic_   = "reality.proprio.tof";     // [ahead-left, ahead, ahead-right, too_close]
    std::string output_topic_ = "percept.avoid_bearing";
    std::string value_topic_  = "reality.cognitive.avoid_value";
    float floor_ = 0.05f;      // below this nearest proximity the loop is silent (a zero bearing)
    float left_ = 0.0f, ahead_ = 0.0f, right_ = 0.0f, tooclose_ = 0.0f;
    bool  have_ = false;
    float cx_ = 0.0f, cy_ = 0.0f, value_ = 0.0f;
};

} // namespace ogma
