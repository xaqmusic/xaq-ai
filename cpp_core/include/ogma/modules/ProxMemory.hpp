// =============================================================================
// ProxMemory.hpp  --  the duck's body-frame proximity slots from a SINGLE beam
// =============================================================================
//
// S3 of the MicroDuck port (docs/plans-and-designs/picrawler_microduck_port_plan.md).  The
// duck's walker senses were four ToF slots [left, ahead, right, too_close] from an 8x8 array;
// the PiCrawler has one ultrasonic beam.  The duck's own trick (`--tof-body`) supplies the
// width: every valid ping is a point, stored in the ODOM frame; each tick the stored points
// are re-expressed in the current body frame and binned by azimuth, nearest per bin.  The
// body's own yaw wander is the sweep.  Everything here is egocentric: the odom frame is the
// body's own dead-reckoned pose, and a point is "where my beam said something was, carried
// by my own motion since".
//
// Inputs:  range  ProprioToken [distance_m, valid]   (the robot's sense.range)
//          odom   ProprioToken [x, y, yaw]           (reality.proprio.odom, DeadReckon's frame:
//                                                     x forward at reset, y = body +X at reset)
// Outputs: prox   ProprioToken [minus_x, ahead, plus_x, too_close], each in [0, 1], 1 = touching
//                 (slot 0 = the body's −X side, slot 2 = +X, so a consumer's "cx = +X" bearing
//                 turns AWAY from slot 2 when it is nearer -- TofAvoidLoop's rule)
//          view   ProprioToken [8]: nearest range per 15 deg sector across ±60 deg, over
//                 view_range_m, 1 = nothing (the place map's view slots, S4)
//
// Module absent = byte-identical.  Nothing is learned here: a frozen geometric reduction.
//
// CORROBORATION (2026-10-27, a named sensor-reduction scaffold like the host's deadzone).  The robot's beam returns
// single false close pings -- a front foot swinging across it (7 of 16 in run 164309), cross-talk -- and "nearest
// point per bin" kept each one as the nearest obstacle for the whole memory (3 s), while too_close fired on one ping.
// support_n > 1: a bin's range is its support_n-th nearest point, so a wall (~19 pings/s landing on the same spot)
// is believed at once and a lone point never is; a toe moves with the body, so its blips scatter in the odom frame
// and do not stack.  too_close_pings > 1: contact needs that many consecutive valid pings below too_close_m.
// Both 1 = byte-identical.
#pragma once
#include <deque>
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class ProxMemory : public Module {
public:
    ProxMemory();
    ~ProxMemory() override;
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

    float slot(int i) const { return i >= 0 && i < 4 ? slots_[i] : 0.0f; }
    int   points() const { return int(pts_.size()); }

private:
    void handle_range(MessagePtr p);
    void handle_odom(MessagePtr p);

    std::string range_topic_ = "sense.range";
    std::string odom_topic_  = "reality.proprio.odom";
    std::string prox_topic_  = "reality.proprio.prox";
    std::string view_topic_  = "reality.proprio.prox_view";
    double range_m_      = 1.0;     // proximity = 1 − r / range_m
    double view_range_m_ = 1.5;     // view = r / view_range_m, 1 = nothing
    double sensor_fwd_m_ = 0.049;   // the beam's origin ahead of the odom origin (the chassis front face)
    // 2026-10-28.  How much of the odometry's TRANSLATION carries the stored points.  The robot's
    // stride odometry over-reads 2-3x on its floor (the sonar closes at a third of what the stride
    // says), so a wall pinged at 0.49 m is dragged into the body by phantom progress: in run 223518
    // (_cs) the 14 pings under 0.5 m all read 0.48-0.50 m, none with a toe in the beam, yet the
    // avoid loop reached a proximity of 0.72 and the body circled right through the memory (a right
    // turn rotates every remembered point to the left).  The yaw is the gyro's and is sound; the
    // translation is the suspect channel.  1 = the full odometry (byte-identical); 0 = ROTATION ONLY:
    // a point keeps its measured range until re-pinged and only its azimuth follows the body's yaw
    // (at 2 cm/s over a 3 s memory the true translation is 6 cm); between = a scale on the translation.
    double odom_translation_ = 1.0;
    // 2026-10-28, the narrower form.  A point may come NEARER only by being re-pinged, never by dead
    // reckoning: its range for the slots is max(carried range, the range its ping measured).  Receding,
    // dropping behind and the azimuth sweep keep the full odometry, so with honest odometry (the sim)
    // nothing changes for a wall the beam keeps seeing; on the robot the phantom approach of a far
    // wall is capped at what the beam actually said.  0 = off, byte-identical; 1 = on.
    double pinged_range_floor_ = 0.0;
    double too_close_m_  = 0.15;    // the live ping below this reads as contact (too_close → 1 at 0)
    int    memory_ticks_ = 150;     // a point lives this long (3 s at 50 Hz)
    double behind_m_     = 0.10;    // a point this far behind the sensor is dropped
    double ahead_half_deg_ = 5.625; // |az| ≤ this is "ahead" (the duck's sector)
    double side_max_deg_   = 60.0;  // |az| beyond this is out of the three slots (still in the view)
    int    support_n_       = 1;    // a bin's range = its support_n-th nearest point (1 = nearest)
    int    too_close_pings_ = 1;    // consecutive close pings before too_close reads (1 = the live ping)
    int    close_streak_    = 0;

    struct Pt { double x, y; uint64_t t; double d; };   // d: the range the ping measured (m), for pinged_range_floor
    std::deque<Pt> pts_;
    double ox_ = 0.0, oy_ = 0.0, oyaw_ = 0.0;   // latest odom
    bool   have_odom_ = false;
    double last_d_ = 0.0; bool last_valid_ = false; bool new_ping_ = false;
    uint64_t last_tick_ = 0;
    float  slots_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float  view_[8]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    int64_t pings_ = 0;
};

} // namespace ogma
