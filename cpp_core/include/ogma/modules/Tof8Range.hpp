#pragma once
// Tof8Range -- the forward 8x8 time-of-flight field reduced to the sonar's contract: the nearest THING ahead
// (2026-11-05, the terrain plan's step 1).
//
// Input: sense.tof8, 64 slant ranges (m, row-major, row 0 the top, column 0 the left, -1 none) from a mount on the
// chassis front face pitched `pitch_deg` down.  The field's own floor is found from its bottom rows -- a plane fitted
// by least squares to their returns, re-fitted without the outliers -- so no IMU enters and the robot's reading is the
// sim's.  A zone whose return sits more than `thing_min_m` above that plane is a THING (a riser, a box, a wall, a
// pillow), more than that below it a DROP; both are features the body would close on.  The nearest feature's slant
// range goes out as ProprioToken [distance_m, valid] on `output_topic` (sense.range_tof): exactly what StuckSense's
// closing-rate channel and ProxMemory read from the sonar, so the stall sense works through soft, dark and distant
// things the sonar never saw -- and, for a body hung by its REAR legs on an edge with open floor ahead, the next band's
// riser is the thing that stops closing (operator, 2026-11-04: the rear legs swing lower and hang more).
// A second token, `cols_topic` (reality.proprio.tof8_cols, [8]), is the nearest feature per column (max_range none),
// for a loop that wants a bearing.  When the bottom rows do not fit a floor (the body nose-down at a wall), every valid
// return within range counts as a thing.  A sensor reduction, named as a scaffold (CLAUDE.md §5.2): it selects and
// believes nothing; it changes units.
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class Tof8Range : public Module {
public:
    static constexpr int kRows = 8, kCols = 8, kZones = 64;
    static constexpr double kFovDeg = 45.0;
    Tof8Range();
    ~Tof8Range() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    // The beam of a zone in the sensor frame at pitch 0 (x right, y up, z forward), and pitched down by pitch_deg.
    static std::array<double, 3> beam(int zone);
    std::array<double, 3> beam_pitched(int zone) const;
    // ---- for tests ----
    double nearest_m() const { return nearest_; }
    bool   nearest_valid() const { return nearest_valid_; }
    int    n_thing_up() const { return n_up_; }
    int    n_thing_down() const { return n_down_; }
    bool   floor_ok() const { return floor_ok_; }
    std::array<float, kCols> const& cols() const { return cols_; }

private:
    void handle(MessagePtr p);
    void reduce();

    std::string input_topic_  = "sense.tof8";
    std::string output_topic_ = "sense.range_tof";
    std::string cols_topic_   = "reality.proprio.tof8_cols";
    double pitch_deg_   = 30.0;
    double max_range_m_ = 4.0;
    double thing_min_m_ = 0.02;      // a rise (or drop) over the fitted floor that counts as a feature
    int    fit_rows_    = 3;         // the bottom rows the floor is fitted from
    double fit_slope_max_ = 0.55;    // |slope| of a believable floor fit (tan 29 deg) -- beyond it the rows hit a wall
    int    col_lo_ = 0, col_hi_ = 7; // the columns the nearest feature is taken over

    std::array<float, kZones> field_{};
    bool have_field_ = false; uint64_t field_tick_ = 0, last_reduced_tick_ = ~uint64_t(0);
    // the reduction
    double nearest_ = 4.0; bool nearest_valid_ = false; int nearest_zone_ = -1;
    int n_valid_ = 0, n_floor_ = 0, n_up_ = 0, n_down_ = 0; bool floor_ok_ = false;
    double plane_a_ = 0, plane_b_ = 0, plane_c_ = 0;
    std::array<float, kCols> cols_{};
    int64_t frames_ = 0;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
