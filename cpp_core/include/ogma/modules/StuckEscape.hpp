// =============================================================================
// StuckEscape.hpp  --  the duck's stuck-stop and escape, for a body that can back up
// =============================================================================
//
// S3 lever 2 of the MicroDuck port.  The duck's rule (IntentAdapter / main.cpp): a stall that
// persists becomes a stop; the exit is BACKING (the only exit it measured; --no-backing was a
// REGRESSION), then a held heading toward the freest sector for a few seconds, then release.
// This module is that rule on the picrawler's sockets, through BearingToIntent's override:
//
//   IDLE    stuck score >= on for confirm_ticks  ->  BACK
//   BACK    override [0, 1, 1, -back_speed] for back_ticks (reverse, heading held)  ->  TURN
//   TURN    override [sin a, cos a, 1, turn_speed] for turn_ticks, a = the freest sector of
//           the proximity view at the trigger (ties -> alternate +/- fallback_turn_deg,
//           because "straight ahead" is where it just got stuck)  ->  REFRACT
//   REFRACT value 0 for refract_ticks (the loops own the body again; no re-trigger)  ->  IDLE
//
// The view is ProxMemory's 8 sectors across +/-60 deg, sector 0 = the body's -X side, 1 =
// nothing remembered.  The bearing published is egocentric at the moment of choice and is
// then held by BearingToIntent's reference while the body turns; this module republishes
// the bearing TO the chosen heading each tick from odom yaw so a slow turn still ends on it.
// Nothing is learned.  Module absent = byte-identical.
#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class StuckEscape : public Module {
public:
    StuckEscape();
    ~StuckEscape() override;
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

    enum class State { Idle = 0, Back = 1, Turn = 2, Refract = 3 };
    State   state() const { return state_; }
    int64_t episodes() const { return episodes_; }
    double  chosen_turn_rad() const { return turn_rad_; }

private:
    void handle_stuck(MessagePtr p);
    void handle_view(MessagePtr p);
    void handle_odom(MessagePtr p);

    std::string stuck_topic_  = "reality.proprio.stuck";
    std::string view_topic_   = "reality.proprio.prox_view";
    std::string odom_topic_   = "reality.proprio.odom";
    std::string output_topic_ = "percept.escape";
    double on_             = 0.7;
    int    confirm_ticks_  = 50;
    int    back_ticks_     = 75;
    double back_speed_     = 0.5;
    int    turn_ticks_     = 100;
    double turn_speed_     = 0.5;
    int    refract_ticks_  = 150;
    double fallback_turn_deg_ = 90.0;
    double view_free_      = 0.9;    // a sector at or above this is "nothing remembered"

    double score_ = 0.0; float view_[8] = {1,1,1,1,1,1,1,1}; bool have_view_ = false;
    double yaw_ = 0.0; bool have_odom_ = false;
    State  state_ = State::Idle; int counter_ = 0; int above_ = 0;
    double turn_rad_ = 0.0, target_yaw_ = 0.0; int fallback_sign_ = 1;
    int64_t episodes_ = 0; int chosen_sector_ = -1;
    float out_cx_ = 0.0f, out_cy_ = 0.0f, out_val_ = 0.0f, out_speed_ = 0.0f;
};

} // namespace ogma
