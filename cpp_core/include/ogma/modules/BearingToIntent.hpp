// =============================================================================
// BearingToIntent.hpp  --  a loop's bearing becomes the gait's intent: goal bearing + speed
// =============================================================================
//
// S3 of the MicroDuck port.  The level-1 intent boundary on the PiCrawler is two sockets on
// MotorEPMv2: goal_bearing_topic ([vx, vy], an egocentric unit vector, vx = body +X) and
// speed_topic ([s] in [-1, 1]).  The duck's IntentAdapter held a HEADING REFERENCE that the
// winning loop set and the heading reflex held when the loop was silent; this module is that
// reference, written for the two sockets:
//
//   * loop active (|bearing| >= floor): the reference becomes the body's heading plus the
//     bearing, and the bearing is published as the goal;
//   * loop silent: the goal is the egocentric bearing TO the reference, so the gait's PD holds
//     the heading the body had when the loop last spoke -- not the spawn's, not whatever it
//     drifts to.  Optionally the reference leaks toward the current heading (ref_tau_s).
//
// The speed sheds with the turn, the duck's ★ TURN gate in its geometric form: s = speed_max ·
// sgn(cos e) · |cos e|^speed_pow, floored at speed_min while the goal is ahead.  speed_pow 0
// means s = speed_max always (no gating).  The turn table (ledger 2026-10-11 S2) is the
// reason: the body's yaw saturates at ~12 deg/s and it FALLS when it pivots at large errors,
// so a loop that asks for a large bearing must also ask for less speed.
//
// Egocentric throughout: the heading is the body's own dead-reckoned yaw (odom[2]).  Module
// absent = byte-identical (the sockets then never receive a message).
#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class BearingToIntent : public Module {
public:
    BearingToIntent();
    ~BearingToIntent() override;
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

    float  last_vx() const { return vx_; }
    float  last_vy() const { return vy_; }
    float  last_speed() const { return speed_; }
    bool   active() const { return active_; }
    double ref_yaw() const { return ref_yaw_; }

private:
    void handle_bearing(MessagePtr p);
    void handle_odom(MessagePtr p);
    void handle_override(MessagePtr p);

    std::string bearing_topic_ = "percept.avoid_bearing";   // [cx, cy, value]; cx = body +X
    std::string odom_topic_    = "reality.proprio.odom";
    std::string goal_topic_    = "reality.proprio.goal_bearing";
    std::string speed_topic_   = "reality.proprio.speed_cmd";
    // S3 lever 2: an OVERRIDE [cx, cy, value, speed].  While value > 0 it wins: its bearing
    // sets the reference and its speed (negative = back up) goes out unchanged.  The escape
    // reflex speaks here.  Empty (default) = no override = byte-identical.
    std::string override_topic_;
    double floor_      = 0.05;   // |bearing| below this = the loop is silent
    double speed_max_  = 1.0;
    double speed_pow_  = 0.0;    // 0 = no gating (speed_max always)
    double speed_min_  = 0.0;    // floor while the goal is ahead (|e| < 90 deg)
    bool   reverse_behind_ = false;   // cos e < 0: back up (true) or hold speed_min (false)
    double ref_tau_s_  = 0.0;    // > 0: the reference leaks toward the current heading
    // 2026-10-28, lever A (the heading reference walks).  On the robot every short activation
    // of the avoid loop -- most of them one or two false sonar pings -- rewrote the reference
    // to "yaw + away-bearing", and the hold then turned the body 20-60 deg toward it once the
    // loop fell silent (runs 185039 .. 223518: 130-500 deg of reference motion per run from
    // episodes under 2.4 s; the _cs right-hand circle was this feedback closing through the
    // proximity memory).  > 0: while the loop is active the reference MOVES toward yaw + e with
    // this time constant (s) instead of jumping, so authority over the heading is earned by
    // persistence: a 0.2 s blip moves it a few degrees, a 20 s wall episode moves it fully.
    // The bearing published DURING the episode is unchanged (the reflex steers at full
    // strength); only what the hold keeps afterward changes.  0 = the jump, byte-identical.
    double ref_attack_s_ = 0.0;
    // Lever A, second form (2026-10-28, after the sim room refuted the time constant: with a
    // lagging reference the hold pulls the body BACK toward the wall it just turned from, and
    // it tips pivoting against it -- 2.0 falls per 12 000 ticks against 0; every fall after
    // 130-450 ticks of wall contact with the hold's goal 70-85 deg behind).  Duration cannot
    // tell a one-ping blip from a short wall glance; the loop's own EVIDENCE can: the robot's
    // blips carried a proximity value of 0.10-0.35, the real walls 0.63-0.75.  > 0: a loop
    // steers at any value above `floor` (the reflex is untouched), but it REWRITES the held
    // reference only while its value is at least this -- weak evidence costs a wiggle back to
    // the old heading, strong evidence sets a new one.  0 = any value rewrites, byte-identical.
    double ref_value_min_ = 0.0;
    double hz_         = 50.0;

    double cx_ = 0.0, cy_ = 0.0, val_ = 0.0; bool have_bearing_ = false;
    double ocx_ = 0.0, ocy_ = 0.0, oval_ = 0.0, ospeed_ = 0.0; bool override_on_ = false; int64_t override_msgs_ = 0;
    double yaw_ = 0.0; bool have_odom_ = false;
    double ref_yaw_ = 0.0; bool ref_set_ = false;
    bool  active_ = false;
    float vx_ = 0.0f, vy_ = 1.0f, speed_ = 1.0f, err_ = 0.0f;
    int64_t msgs_ = 0;
};

} // namespace ogma
