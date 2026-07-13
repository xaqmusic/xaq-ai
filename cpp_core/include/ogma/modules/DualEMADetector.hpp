#pragma once

// =============================================================================
// DualEMADetector.hpp  --  Phase 6.6.D.2 generic event-from-trend primitive
// =============================================================================
//
// Subscribes to a configurable scalar input topic, tracks both a fast (short-α)
// and slow (long-α) EMA, and publishes `events.<output_event_name>` whenever
// the short EMA exceeds the long EMA by `ratio_threshold`.  Optional motion
// floor: gate the event on a second proprio scalar staying above a minimum
// (mirrors the body-side scent-progress-AND-moving rule).
//
// Replaces the body-side scent → events.hit firing in body_controller.gd.
// Generic so the same module type can be re-used for any "trend acceleration"
// detector (e.g. drive.urgency drops, video novelty bursts).

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <string>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class DualEMADetector : public Module {
public:
    DualEMADetector();
    ~DualEMADetector() override;

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
    int   fire_count()  const { return fire_count_; }
    float short_ema()   const { return short_ema_; }
    float long_ema()    const { return long_ema_; }

private:
    void handle_input(MessagePtr payload);
    void handle_motion(MessagePtr payload);

    std::string input_topic_       = "reality.proprio.scent_max";
    int         input_index_       = 0;
    std::string output_event_name_ = "hit";
    std::string output_topic_      = "events.hit";   // derived
    float       alpha_short_       = 0.1f;
    float       alpha_long_        = 0.001f;
    float       ratio_threshold_   = 1.5f;
    int         refractory_ticks_  = 0;
    bool        require_long_pos_  = true;   // skip until long_ema > 0 (warmup)
    std::string motion_floor_topic_ = "";
    int         motion_floor_index_ = 0;
    float       motion_floor_min_   = 0.0f;

    bool   short_ema_initialized_ = false;
    float  short_ema_   = 0.0f;
    float  long_ema_    = 0.0f;
    float  motion_val_  = 0.0f;
    bool   motion_seen_ = false;
    int    refractory_remaining_ = 0;
    int    fire_count_  = 0;
};

} // namespace ogma
