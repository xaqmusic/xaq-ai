// =============================================================================
// AheadField.hpp  --  the ground in front of the body, straight from the forward field, against the front toes' own ground
// =============================================================================
//
// Learned stepper plan section 9.7 item 1 (2026-11-20).  The first source tried for the ground-ahead grid was the terrain
// memory (TerrainMemory's ahead_topic): it fuses the field across frames, so the rhythm's rocking is out.  But a remembered
// height is a body-frame height PLUS THE DATUM AT THE TICK IT WAS STORED, and where the two walkers alternate on stairs the datum
// moves by 1.4-4.5 cm (p95) inside two seconds -- the size of the risers -- so two cells stored a second apart can disagree by
// one.  Measured side by side (13 stairs runs, ledger 2026-11-20): under the sequencer the two grids read alike; under the
// rhythm the memory's reads 50 and 64 mm risers at 44 and 61 with a tenth of the rows under 25 and 34, and 12 / 19 / 25 mm
// risers all at 25, where this module reads 53 and 68 (tenth: 48 and 61) and 21 / 29 / 37.
//
// This module needs neither the datum nor the odometry.  Per field frame:
//   1. every return is levelled by gravity (up_body, the fused up vector): its height along gravity and its horizontal place
//      forward and to the side of the body's origin;
//   2. its height is taken against THE PLANTED FRONT TOES' OWN HEIGHT in the same frame at the same tick (toe_y of the front
//      legs in contact): the ground the front legs stand on.  The body's bob, crouch and climb leave with it, by construction;
//   3. it is binned on a BODY-FIXED grid (rows x cols cells in front of the body) and each cell is averaged over tau_s
//      (count-annealed: the first frame of a cell is taken whole).  A cell no return has reached for hold_s is unknown again.
// The average over tau_s (a stride of the rhythm) is what takes the rocking out; the body moves under a body-fixed grid, so a
// step's edge is smeared by the travel in tau_s (6 cm at the rhythm's pace, under a row and a half) -- the price of not
// trusting an odometry that reads 1.1 cm/s on a body making 0.3 (ledger 2026-11-19).
// Checked offline before it was built (`ah_proto.py` on 22 logs, the truth attitude) and then as built, with the fused up
// vector: the highest relief in the centre column reads 21 / 29 / 37 / 44 / 53 / 68 / 89 mm for true risers of 12 / 19 / 25 / 38 /
// 50 / 64 / 89 under the rhythm (the low ones a centimetre high: the reference is the toes' COMMANDED height, and the rhythm's
// planted toes are commanded into the ground) and 38 / 52 / 66 / 91 for 38 / 50 / 64 / 89 under the sequencer.
//
// THE STEP THE BODY STANDS ACROSS (astride).  The field looks forward; a riser the front legs have already climbed is under the
// body, and that is where the walkers spend most of their stalls (measured, ledger 2026-11-20: 56 % of the rhythm's stalled rows
// on stairs are astride a step of 45 mm or more, 30 % have it ahead).  The toes carry it: the planted front toes' height over
// the planted rear toes', averaged over astride_tau_s, in the token's sixth header value (0 until both ends have been planted).
//
// OUTPUT.  The same token as TerrainMemory's ahead grid, so AheadFeatures reads either: ProprioToken [rows, cols, row_m, col_m,
// start_m, astride (m), rows x cols heights (ALREADY against the front toes' ground; kNoHeight where unknown), rows x cols precisions
// (the cell's frame count, capped at tau)].  An INSTRUMENT: a transparent sensor reduction, nothing acts on it.  Module absent
// = byte-identical.
#pragma once

#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"
#include "ogma/body/Tof8Geometry.hpp"

namespace ogma {

class AheadField : public Module {
public:
    AheadField();
    ~AheadField() override;

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
    double  cell(int row, int col, bool& known) const;
    double  ref() const { return ref_; }
    int64_t frames() const { return frames_; }
    int64_t no_ref() const { return no_ref_; }
    double  astride() const { return astride_; }

private:
    std::string field_topic_ = "sense.tof8", up_topic_ = "reality.proprio.up_body", toe_y_topic_ = "reality.proprio.toe_y",
                contact_topic_ = "reality.proprio.foot_contact", output_topic_ = "terrain.ahead";
    double pitch_deg_ = 30.0, sensor_x_m_ = 0.0, sensor_y_m_ = 0.049, sensor_z_m_ = 0.049, max_range_m_ = 2.0;
    int    rows_ = 9, cols_ = 3; double row_m_ = 0.04, col_m_ = 0.08, start_m_ = 0.10;
    double tau_s_ = 0.6, hold_s_ = 1.5, hz_ = 50.0, up_min_ = 0.8, astride_tau_s_ = 1.0;
    int    front_a_ = 0, front_b_ = 1;      // the front legs' indices in toe_y and foot_contact

    std::array<float, body::tof8::kZones> field_{}; bool fresh_ = false;
    std::array<double, 3> up_{0.0, 1.0, 0.0}; bool have_up_ = false;
    std::array<float, 4> toe_y_{}, contact_{}; bool have_toe_ = false, have_contact_ = false;
    std::vector<double> est_, cnt_; std::vector<int64_t> seen_tick_;
    uint64_t last_frame_tick_ = 0; bool have_frame_ = false;
    double ref_ = 0.0; int64_t frames_ = 0, no_ref_ = 0, tilted_ = 0;
    double astride_ = 0.0; int64_t astride_n_ = 0;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
