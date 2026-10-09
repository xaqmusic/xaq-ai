// =============================================================================
// ThingBelief.hpp  --  what the targeted thing is doing, not where it was (the duck, design doc §17.113, 2026-10-09)
// =============================================================================
//
// The seek loop's target is a PLACE: a sighting fixes it by dead reckoning, and the arrival fires the stop and the outcome
// loop's request without anything checking the thing is still there -- one interaction in seven is aimed at a thing that
// had already moved (the train that stands and goes, the balls the duck's own kicks rolled).  This module holds a belief
// about the THING the seek loop is sent to.
//
//   infers         whether the targeted thing is still where the brain believes it is
//   senses         the seek loop's target (target_out_topic: have, x, y, the tick it was set, chasing) and CloudMap's
//                  presence evidence (presence_topic: a small thing at the target this tick; free-ray samples that
//                  crossed the target's footprint below the confirmed thing's top -- the place seen empty)
//   predicts       the thing's presence at its believed place
//   honest signal  the evidence each tick: a confirmation, or the place seen through
//   publishes      belief_topic [presence, ticks confirming, ticks refuting, ticks since the target was set, have a target]
//
// T1 (passive): presence = (1 + confirming ticks) / (2 + confirming + refuting), counted since the target was set; a chased
// thing is present by being chased.  Nothing reads it yet.  The module absent = byte-identical.
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class ThingBelief : public Module {
public:
    ThingBelief();
    ~ThingBelief() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json diag_snapshot() const override;

    double presence()   const { return presence_; }
    int    confirming() const { return conf_; }
    int    refuting()   const { return refute_; }

private:
    std::string target_topic_, presence_topic_, belief_topic_;
    bool     have_ = false, chasing_ = false;
    double   set_tick_ = -1.0;
    int      conf_ = 0, refute_ = 0;
    uint64_t since_ = 0;
    double   presence_ = 1.0;
};

} // namespace ogma
