#pragma once

// =============================================================================
// VisualBearing.hpp  --  reward-free VISION food-direction perception
// =============================================================================
//
// The vision analog of ScentCompass.  Subscribes the body's raw first-person
// colour frame (host.video.color, a RawImageFrame of food-green/wall/sky pixels
// produced by the FPV raycast) and reduces it to a 2-D EGOCENTRIC bearing toward
// the food (percept.visual_bearing), in the SAME convention as ScentCompass:
// [x = +right, y = +forward].
//
//     host.video.color (RawImageFrame) → VisualBearing → percept.visual_bearing
//
// The reduction: count the food (green) pixels — the SAME test the body uses for
// green_fraction (R<green_r_max AND B<green_b_max AND G>green_g_min) — and take
// the horizontal CENTROID of their screen columns (u ∈ [-1,+1], +u = right).  The
// raycast builds each ray as forward + right·(u·tanH) + up·(v·tanH), so a food
// blob at column-centroid u sits at egocentric bearing normalize([u·tanH, 1])
// (right, forward).  tanH = tan(fov_deg/2) MUST match the FPV camera's fov (80°
// in the_cell.tscn) so the lateral angle is scaled like the body's geometry.
//
// CONFIDENCE / occlusion: vision is LINE-OF-SIGHT only.  When no food pixel is in
// view (food occluded behind a wall, or out of frame) the bearing is [0,0] with
// magnitude 0 — a clean "no-signal" state, exactly parallel to ScentCompass's
// normalize_direction + min_signal gate.  Downstream, a per-modality EPM sees a
// CONSTANT [0,0] while occluded → bakes ~1 degenerate node → the LateralVoter's
// informativeness gate drives its trust toward 0; BearingFusion's confidence
// floor gives the same down-weight instantly.  When food enters view the bearing
// varies → the EPM grows angle-selective nodes → vision trust rises.  This is the
// fusion proof-of-concept's headline mechanism (trust tracks informativeness).
//
// normalize_direction (default true) emits a UNIT bearing when food is visible so
// the downstream RBF-EPM clusters by ANGLE, not magnitude (the ScentCompass
// direction-blind lesson).  lesion_after_ticks / force_lesion knock vision out
// mid-run (the dropout / perturbation→recovery demo) by emitting [0,0].

#include "ogma/Module.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class VisualBearing : public Module {
public:
    VisualBearing();
    ~VisualBearing() override;

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

    // White-box accessors for tests / metrics.
    float last_vx()         const { return vx_; }          // +right
    float last_vy()         const { return vy_; }          // +forward
    float last_mag()        const { return mag_; }         // 0 = occluded, ~1 = food in view
    float last_green_frac() const { return green_frac_; }  // graded confidence
    bool  lesioned()        const { return lesioned_; }
    bool  have_proto()      const { return have_proto_; }  // learned a food appearance yet?
    float proto_r()         const { return food_proto_[0]; }
    float proto_g()         const { return food_proto_[1]; }
    float proto_b()         const { return food_proto_[2]; }

private:
    void handle_video(MessagePtr payload);
    void handle_hit(MessagePtr payload);

    std::string input_topic_  = "host.video.color";
    std::string output_topic_ = "percept.visual_bearing";
    // SCAFFOLD (default) Food-pixel test — matches body_controller.gd green_fraction
    // (pixels[+0]=R, [+1]=G, [+2]=B; food = R<128 AND B<128 AND G>128).
    int   green_r_max_ = 128;   // R strictly below
    int   green_b_max_ = 128;   // B strictly below
    int   green_g_min_ = 128;   // G strictly above
    // NON-SCAFFOLDED learned-appearance mode: the bug does not know "food = green".
    // It LEARNS food's colour by association — on each eat (events.hit) the central
    // FPV (food is dead-ahead during the final approach) is blended into a learned
    // food prototype; a pixel is "food" if its colour is within color_match_dist of
    // that prototype. Scent/eating is the teacher, vision the student. Before the
    // first eat there is no prototype → no visual bearing (it must be taught first).
    bool  learn_appearance_  = false;          // opt-in; false = green scaffold
    std::string hit_topic_   = "events.hit";   // teacher signal (EnvEvent)
    float appearance_alpha_  = 0.3f;           // EMA: central colour at eat → prototype
    float color_match_dist_  = 60.0f;          // RGB distance (0..255) within which a pixel = food
    float central_ema_rate_  = 0.2f;           // EMA of the central-region mean colour (the approach view)
    float central_frac_      = 0.3f;           // central fraction of the frame sampled as the approach view
    float fov_deg_     = 80.0f; // MUST match the FPV Camera3D.fov (the_cell.tscn = 80)
    bool  normalize_direction_ = true;   // unit bearing when food visible (angle-selective EPM)
    float min_confidence_ = 0.0008f;     // green-fraction floor below which → [0,0] (occluded)
    bool  emit_proximity_ = false;       // append values[2] = proximity_gain·green_frac
    float proximity_gain_ = 1.0f;
    float centroid_ema_alpha_ = 0.0f;    // >0 smooths the column centroid (anti-jitter); 0 = off
    int   lesion_after_ticks_ = -1;      // ≥0 → emit [0,0] from this tick on (dropout demo)
    int   lesion_until_ticks_ = -1;      // ≥0 → lesion ENDS here (vision restored) → a dropout WINDOW
                                         // [after, until) for the (d) perturbation→degradation→RECOVERY test;
                                         // <0 (default) = permanent onset (prior behaviour, byte-identical)
    bool  force_lesion_ = false;         // immediate lesion (UI "knock out vision" toggle)
    // Kalman-lessons Stage 2 perturbation: a STUCK sensor.  From this tick on the
    // module republishes the last bearing it saw food with, unchanged — a constant,
    // plausible reading.  A lesion ([0,0]) is caught downstream by BearingFusion's
    // confidence floor; a stuck sensor is not, and a trivially predictable channel is
    // exactly what 1/(err+ε) trusts most.  The activity term is measured against it.
    int   stick_after_ticks_ = -1;       // <0 (default) = off, byte-identical

    // Cached input frame.
    std::vector<uint8_t> pixels_;
    int height_ = 0, width_ = 0, channels_ = 0;

    // Outputs / working state.
    float vx_ = 0.0f, vy_ = 0.0f, mag_ = 0.0f, green_frac_ = 0.0f;
    float ema_u_ = 0.0f; bool have_ema_ = false;
    bool  lesioned_ = false;
    uint64_t tick_count_ = 0;
    bool  stuck_ = false;
    float last_seen_[3] = {0.0f, 0.0f, 0.0f};   // last bearing published with mag_ > 0
    bool  have_last_seen_ = false;

    // learned-appearance state
    float food_proto_[3]  = {0.0f, 0.0f, 0.0f};
    bool  have_proto_     = false;
    float central_ema_[3] = {0.0f, 0.0f, 0.0f};
    bool  have_central_   = false;
    bool  hit_pending_    = false;
};

} // namespace ogma
