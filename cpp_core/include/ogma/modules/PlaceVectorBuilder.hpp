// =============================================================================
// PlaceVectorBuilder.hpp  --  the place vector the place-EPM learns the map from
// =============================================================================
//
// Cell round 2, lever A5 (2026-09-06).  The audit found the Cell's "place map" to be a
// hand-rolled grid over a path integral inside PlayLoop and PlaceGraphPlanner, with the
// place-EPM demoted to a novelty supplier (audit V1, CLAUDE.md §0 rule 1).  This module
// builds the vector the EPM should have been learning the map FROM:
//
//     [ panorama (n_pano, 0..1) ; (x/odo_scale, y/odo_scale, cos h, sin h) × pose_repeat ]
//
// The panorama is CylinderBuilder's held place code.  The pose is the SAME dead-reckoned
// path integral the two consumers keep (egocentric velocity rotated by the published
// heading), so nothing new crosses the blanket; the heading's drift-free perfection is the
// named scaffold (audit S1, register O8).  pose_repeat = n_pano / 4 (derived, not tuned)
// repeats the four pose dims so the two groups carry equal weight in the RBF encoder's
// L2 distance -- the group balancing the LateralVoter does across modalities, done by
// dimension count instead of a gain.
//
// With this vector on the place-EPM and the consumers' pi_cell_size at 0, the EPM's
// winner_id IS the node id: the map is the learned vocabulary, and "novel" means a place
// the vocabulary has not seen.  The module absent = byte-identical (the grid stays).
#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class PlaceVectorBuilder : public Module {
public:
    PlaceVectorBuilder();
    ~PlaceVectorBuilder() override;

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

    // white-box accessors (tests + metrics)
    int    out_dims()    const { return n_pano_ + 4 * pose_repeat_; }
    int    pose_repeat() const { return pose_repeat_; }
    double odo_x()       const { return odo_x_; }
    double odo_y()       const { return odo_y_; }
    float  heading()     const { return heading_; }
    bool   have_pano()   const { return have_pano_; }
    std::vector<float> const& last_vector() const { return last_; }

private:
    void handle_pano(MessagePtr p);
    void handle_heading(MessagePtr p);
    void handle_vel(MessagePtr p);

    std::string pano_topic_    = "percept.cylinder";
    std::string heading_topic_ = "reality.proprio.heading";
    std::string vel_topic_     = "reality.proprio.vel_ego";
    std::string output_topic_  = "percept.place_vec";
    int    n_pano_    = 24;
    double odo_scale_ = 240.0;   // odometry units are tick·speed⁻¹ (20 ≈ 1 m at move_speed 3): 240 ≈ 12 m → a 24 m room maps to [-1, 1]
    int    pose_repeat_ = 6;     // derived at setup: max(1, n_pano / 4)

    std::vector<float> pano_;
    bool   have_pano_ = false;
    float  heading_   = 0.0f;
    float  vlat_ = 0.0f, vfwd_ = 0.0f;
    double odo_x_ = 0.0, odo_y_ = 0.0;
    std::vector<float> last_;
};

} // namespace ogma
