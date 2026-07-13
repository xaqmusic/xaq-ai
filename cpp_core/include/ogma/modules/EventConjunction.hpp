#pragma once

// =============================================================================
// EventConjunction.hpp  --  v5.3 Phase C — N-event AND primitive
// =============================================================================
//
// Subscribes to N event topics (input_event_topics).  Tracks the most-recent
// fire-tick per topic.  Each tick, if every input event has fired within the
// last `window_ticks` (and the optional motion floor is satisfied), publishes
// `events.<output_event_name>` with intensity = mean of input intensities.
//
// Use case: Phase v5.3 vision+scent reward scaffold.  Two DualEMADetectors
// emit `events.scent_hit` (scent gradient rising AND moving) and
// `events.green_visible` (green-fraction gradient rising).  EventConjunction
// ANDs them within a 30-tick window to emit `events.scent_aligned_with_green`,
// which NeurochemState recognises as a teaching DA pulse (`da_aligned_gain`).
//
// Generic: any number of event topics, configurable window, optional motion
// gating.  No state across runs apart from per-topic last-fire-tick + a
// refractory counter.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class EventConjunction : public Module {
public:
    EventConjunction();
    ~EventConjunction() override;

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
    int   fire_count()   const { return fire_count_; }
    int   inputs_seen()  const { return inputs_seen_total_; }
    std::vector<int64_t> const& last_fire_ticks() const { return last_fire_ticks_; }

private:
    void handle_event(std::size_t input_idx, MessagePtr payload);
    void handle_motion(MessagePtr payload);

    // Configuration
    std::vector<std::string> input_event_topics_;          // ["events.scent_hit", "events.green_visible", ...]
    std::string              output_event_name_   = "conjunction";
    std::string              output_topic_        = "events.conjunction"; // derived
    int                      window_ticks_        = 30;
    int                      refractory_ticks_    = 30;
    std::string              motion_floor_topic_  = "";
    int                      motion_floor_index_  = 0;
    float                    motion_floor_min_    = 0.0f;

    // Working state (per-input)
    std::vector<int64_t> last_fire_ticks_;     // tick_id of last fire per input; -1 = never
    std::vector<float>   last_intensities_;
    int                  refractory_remaining_ = 0;
    int                  fire_count_           = 0;
    int                  inputs_seen_total_    = 0;
    float                motion_val_           = 0.0f;
    bool                 motion_seen_          = false;
};

} // namespace ogma
