// =============================================================================
// AheadFeatures.hpp  --  the ground in front of the body, as one conditioned vector for a ground-ahead EPM
// =============================================================================
//
// Learned stepper plan section 9.7 item 1 (2026-11-20, operator: "go ahead with the ground-ahead vocabulary").  The walker
// selection in shadow (ledger 2026-11-19) ranks the two walkers right on progress and cannot know that the riser the rhythm
// stalled at is still there; what it lacks is a percept of the ground the front legs are about to meet, in a vocabulary each
// walker's outcome can be remembered against.  This module is the transparent reduction under that vocabulary (a named
// scaffold: CLAUDE.md section 5.2 allows a sensor reduction for perception; what is selected on it is learned).  An INSTRUMENT:
// nothing acts on it.
//
// WHAT IT SEES (egocentric only).  TerrainMemory's ground-ahead grid (terrain.ahead): the remembered heights on rows x cols
// cells in front of the body, row 0 on the front toes' line, fused across frames in the gravity-levelled frame -- so the
// rocking of the rhythm's body, which makes the forward field useless frame by frame (ledger 2026-11-19), is already out.
//
// CONDITIONING (CLAUDE.md section 0 rule 2).
//   the common mode   the ground the front toes stand on: `ref`, the precision-weighted mean of row 0 (the nearest row with
//                     anything remembered when row 0 is empty).  What is featurised is the RELIEF, h - ref: the step the front
//                     legs have to make.  The body's climb, the datum and the standing height all leave with it.
//                     With AheadField as the source the grid arrives already against the front toes' own height
//                     (ref_row0 false): row 0 is then a row like the others' neighbour and nothing is subtracted.
//   the scale         one length for every cell, the body's own standing height (scale_m, 0.085: PoseSequencer's stand_h_m):
//                     a relief of 1 is a step as high as the belly.  The cells are homogeneous, so no per-feature normaliser is
//                     needed -- and none is wanted: an adaptive one is part of the encoder and strands the nodes stored under
//                     it when the world changes (ledger 2026-11-15).  Clipped at +-clip.
//   under the body    (under_rows > 0, AheadField only) the field cannot see the ground the body already stands across, and a
//                     walker stalled with its front legs up a riser faces flat ground.  The grid's sixth header value is the
//                     front toes' height over the rear toes'; the rear toes' ground against the front toes' (its negative, in
//                     body units) is one more feature, weighted sqrt(under_rows x cols): the rows the body covers, read from
//                     the toes, as if they were in the grid.
//   what is unknown   a cell nothing is remembered under reads 0 relief, and each column's unknown share (rows 1..R-1) is a
//                     feature of its own, so "flat" and "not seen" are different vectors.
//   the magnitude     the jl_state encoder L2-normalises its input and keeps only a direction: a 38 mm riser and a 64 mm one
//                     of the same shape would be the SAME vector.  One constant component, bias_ratio x sqrt(cells), keeps the
//                     magnitude as an angle (flat ground is the constant's own direction; a relief of bias_ratio on every cell
//                     sits at 45 degrees to it).  StanceFeatures' device, with the constant set from the width.
//
// OUTPUT.  ProprioToken on output_topic: [(R-1) x C reliefs (row-major, row 1 first), C unknown shares, (the ground under the
// body, when under_rows > 0), the constant].
// Published every tick the grid arrives and the body is upright (up_topic's y >= up_min; "" = always): a body on its side
// reads its own legs as ground.  Module absent = byte-identical.
#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class AheadFeatures : public Module {
public:
    AheadFeatures();
    ~AheadFeatures() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    // ---- for tests ----
    std::vector<float> const& features() const { return feat_; }
    int     dim()       const { return int(feat_.size()); }
    double  ref()       const { return ref_; }
    int64_t published() const { return published_; }
    int64_t held()      const { return held_; }

private:
    std::string ahead_topic_  = "terrain.ahead";
    std::string up_topic_     = "reality.proprio.up_body";
    std::string output_topic_ = "reality.proprio.ahead_features";
    double scale_m_ = 0.085, clip_ = 2.0, bias_ratio_ = 0.5, up_min_ = 0.8;
    int    under_rows_ = 0;    // > 0: the ground UNDER the body is a feature too (AheadField's astride), with the weight of this many rows
    double astride_m_ = 0.0;
    bool   ref_row0_ = true;   // false: the grid's heights are already against the front toes' ground (AheadField), nothing is subtracted

    std::vector<float> grid_; bool fresh_ = false;     // the last grid token, as it arrived
    double up_y_ = 1.0; bool have_up_ = false;
    std::vector<float> feat_, relief_m_, unknown_;      // the output; the reliefs in metres and the unknown shares, for the log
    double ref_ = 0.0, norm_ = 0.0; int ref_row_ = -1, rows_ = 0, cols_ = 0;
    int64_t published_ = 0, held_ = 0, no_ref_ = 0;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
