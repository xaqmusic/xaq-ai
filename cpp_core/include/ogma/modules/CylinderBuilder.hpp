#pragma once

// =============================================================================
// CylinderBuilder.hpp  --  heading-indexed panorama place-code (Pathway C2)
// =============================================================================
//
// Turns a saccade sweep (C1) into a stable place signature — the insect "snapshot".
// While the saccade is active the bug pivots through its headings; this module bins
// the FPV appearance by ABSOLUTE heading and accumulates it into a panorama (a
// "cylinder of time-smoothed vision"). Indexing by absolute heading makes the
// panorama VIEW-INVARIANT: the colour seen looking north from place P is the same
// however the bug arrived, so the same place → the same cylinder. That fixes the
// per-tick view fragmentation (epm_vision was 58 nodes / 0 baked) — the place-EPM
// (C3) clusters whole cylinders into stable, loop-closing zone nodes.
//
//   host.video.color + reality.proprio.heading + saccade.active
//        → CylinderBuilder → percept.cylinder  ([n_bins × 3] RGB panorama, 0..1)
//
// Appearance = the FPV frame's mean RGB per tick (the zone-coloured walls give a
// distinct colour per zone). Binned into n_bins around the circle by absolute
// heading; averaged within the sweep (time-smoothing). On saccade end the panorama
// is finalised (unfilled bins carry the previous cylinder) and HELD/published every
// tick so C3 sees the current place estimate until the next scan refreshes it.
//
// Default-off / opt-in. C2 gate: same spot scanned twice → same cylinder; two
// different spots → distinguishable cylinders (the anti-aliasing test).

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class CylinderBuilder : public Module {
public:
    CylinderBuilder();
    ~CylinderBuilder() override;

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

    // White-box accessors (tests + metrics).
    int   n_bins()          const { return n_bins_; }
    int   cylinders_built() const { return cylinders_built_; }
    int   bins_filled()     const { return last_bins_filled_; }
    std::vector<float> const& panorama() const { return panorama_; }  // [n_bins*3], 0..1

private:
    void handle_frame(MessagePtr payload);
    void handle_heading(MessagePtr payload);
    void handle_active(MessagePtr payload);
    int  bin_of(float heading_rad) const;
    void finalize_panorama();

    std::string frame_topic_    = "host.video.color";
    std::string heading_topic_  = "reality.proprio.heading";
    std::string active_topic_   = "saccade.active";
    std::string output_topic_   = "percept.cylinder";
    int   n_bins_ = 8;
    // 2026-06-27 (operator) — PASSIVE panorama: when >0, accumulate the FPV into bins
    // EVERY tick (from the bug's natural turning — klino weave + nav turns) and emit a
    // rolling windowed panorama every passive_emit_every ticks (finalise+reset). No
    // saccade needed → view-invariant place-code built passively. 0 = saccade-gated (legacy).
    int   passive_emit_every_ = 0;

    // latest inputs
    std::vector<uint8_t> pixels_;
    int   width_ = 0, height_ = 0, channels_ = 0;
    float heading_ = 0.0f;
    float active_  = 0.0f;
    float prev_active_ = 0.0f;

    // per-sweep accumulators (one panorama being built)
    std::vector<double> sum_r_, sum_g_, sum_b_;
    std::vector<int>    count_;

    // finalised, held panorama (the place code) — [n_bins*3], 0..1
    std::vector<float> panorama_;
    bool  have_panorama_ = false;

    // telemetry
    int   cylinders_built_  = 0;
    int   last_bins_filled_ = 0;
};

} // namespace ogma
