// =============================================================================
// BearingSeekLoop.hpp  --  seeking a thing seen only now and then (the duck's things phase, T2)
// =============================================================================
//
// The duck sees a small thing only while it stands: the swept cloud (CloudMap) exists at a stop,
// and its attended thing's bearing reads proximity 0 the moment the body walks.  A loop that
// walked toward things would therefore be silent for the whole walk unless it REMEMBERED where
// the thing was.  This loop does, in the body's own frame of reference: while the bearing is
// live it fixes the thing's position by dead reckoning (its own odometry pose plus the bearing
// and range), and while the bearing is silent it homes to that remembered position, re-aiming
// as the body moves and turns, until it arrives (the remaining range under arrive_m) or forgets
// (its confidence decays to the floor).  The Cell's VisualHomingNav kept an allocentric BEARING
// through an occlusion; the duck has range, so the belief is a POSITION, and arrival is its own.
//
// What it publishes, in the loop unit's five fields (loop_and_arbitration_recipe.md):
//   - the bearing to the thing, [cx = +right, cy = +forward, 0]: the IntentAdapter's heading-
//     reference contract, the same as play's and avoidance's;
//   - its NEED / value in [0,1]: the confidence in the held target (1 while seen, decaying while
//     remembered, 0 when none) -- the arbiter's preference for this loop (hunger_topic);
//   - its HONEST SIGNAL: the remaining range to the target, for LoopCompetence (sign -1: seeking
//     works while the range falls under its own drive).
// Nothing here is a trajectory; the twist prior turns the body toward the reference as it always
// has.  Module absent = byte-identical.
#pragma once

#include <limits>
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class BearingSeekLoop : public Module {
public:
    BearingSeekLoop();
    ~BearingSeekLoop() override;

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
    nlohmann::json diag_lite() const override;
    void           restore_state(nlohmann::json const& s) override;

    bool   have_target() const { return have_target_; }
    bool   seen()        const { return seen_; }
    float  value()       const { return value_; }
    double range_left()  const { return range_left_; }
    int    arrivals()    const { return arrivals_; }
    int    forgets()     const { return forgets_; }
    float  last_cx()     const { return cx_; }
    float  last_cy()     const { return cy_; }
    double target_x()    const { return tx_; }
    double target_y()    const { return ty_; }

private:
    std::string bearing_topic_ = "percept.thing_bearing";      // [vx=+right, vy=+forward, proximity]
    std::string pose_topic_    = "reality.proprio.odom";       // [x, y, yaw] dead-reckoned, the body's own
    std::string output_topic_  = "percept.seek_bearing";
    std::string value_topic_   = "reality.cognitive.seek_value";
    std::string range_topic_   = "reality.cognitive.seek_range";
    double proximity_range_ = 2.5;    // the bearing's proximity is 1 - range / this (CloudMap max_range / things_range)
    float  min_conf_        = 0.02f;  // proximity above this = the thing is in view
    double arrive_m_        = 0.25;   // the remaining range at which the target counts as reached
    double forget_ticks_    = 3000.0; // confidence decays by 1/forget_ticks per tick while the thing is unseen
    float  floor_           = 0.05f;  // ...and the target is dropped below this
    // the renewal (2026-09-22, the linger): a token [need, x, y] from the loop that learns what intents do at
    // the thing (SkillOutcomeLoop need_topic).  After an arrival drops the target, a need above renew_min at a
    // position between 1.5 x arrive_m and renew_range away re-arms it with confidence = need: the duck goes
    // back to a thing it does not yet understand and leaves one it does.  Empty = off (byte-identical).
    std::string renew_topic_;
    float  renew_min_       = 0.25f;
    double renew_range_     = 2.0;
    // the walk re-fix (2026-09-23, §17.47): a bearing flagged as seen from a WALKING cloud (the token's 4th
    // value) only refines a target already held, when its fix lies within walk_refix_m of it; it never sets
    // one.  The approach is then by sight and the arrival is where the thing IS.  0 = walking bearings ignored.
    double walk_refix_m_    = 0.0;
    int    refixes_ = 0;
public:
    int refixes() const { return refixes_; }
    // THE CHASE (the chase phase, stage 1, 2026-09-27): the moving fix.  CloudMap's mover_topic names a cluster whose
    // voxels are young against the cloud's own -- a thing that is not where "things do not move" predicted it
    // (design doc §17.53).  The loop holds such a sighting as a CANDIDATE with a position in the odometry frame;
    // a later sighting within chase_gate_m of where the candidate would now be (its position plus its velocity
    // times the time since) confirms it and updates the velocity; after chase_confirm sightings spread over at
    // least chase_confirm_ticks the candidate is CHASED: the target is its predicted position a chase_lead_s ahead,
    // re-fixed by every confirming sighting, the need 1, and arrival does not drop it (a mover's error is never
    // fulfilled by standing where it was).  With no confirming sighting for chase_forget_ticks the chase ends and
    // the last predicted position stays as an ordinary remembered target: where it stopped is where to go and
    // look.  A static thing newly in view is young for under a second and ages out before it confirms; a fragment
    // sliding along a wall does not follow the prediction.  Empty mover_topic = off, byte-identical.
    bool   chasing()        const { return chasing_; }
    int    chase_n()        const { return cand_n_; }
    int    chases()         const { return chases_; }
    double chase_vx()       const { return cand_vx_; }
    double chase_vy()       const { return cand_vy_; }
    bool   mover_seen()     const { return mover_seen_; }
    bool   have_cand()      const { return have_cand_; }
    double cand_x()         const { return cand_x_; }
    double cand_y()         const { return cand_y_; }
private:
    std::string mover_topic_;
    double chase_gate_m_ = 0.35, chase_lead_s_ = 0.3, chase_v_max_ = 1.0;
    // chase_min_v (2026-09-27 night, §17.55): a candidate is chased only if it has MOVED -- its velocity and its
    // displacement since the first sighting both at least this (m/s, and m per second watched).  A young cluster that
    // stays put is a static thing newly in view (R85 chased 447 of those in six runs); 0 = not required.
    double chase_min_v_ = 0.0;
    double cand_x0_ = 0.0, cand_y0_ = 0.0;
    // chase_stop_v (2026-09-28, the operator's eye: the duck walked to where the train HAD passed and pecked at the
    // place): a chase that ends with the thing still moving (its last velocity above this, m/s) is DROPPED -- the
    // thing left the view, it is not at the predicted place; only a thing that had slowed below this is remembered
    // where it stopped.  A position belongs to a thing while the thing is stationary.  0 = always remembered.
    double chase_stop_v_ = 0.0;
    int    chases_lost_ = 0, chases_stopped_ = 0;
    // the loss, for a host that turns it into a LOOK (--stop-on-lost, 2026-09-29): true on the tick a chase is dropped
    // with the thing still moving, with the bearing (body frame, + = right) and range of where it was last predicted
    bool   lost_now_ = false; double lost_ego_ = 0.0, lost_range_ = 0.0;
    // OBJECT PERMANENCE (2026-09-29, the operator: "some form of object permanence, especially for moving objects"):
    // chase_permanence_ticks -- when the sightings stop with the thing still moving, the target keeps moving at its
    // last velocity for up to this long, the need falling from 1 to 0 over it (COASTING), and a sighting near the
    // prediction takes the chase up again; at the end the loss is reported (the look).  0 = the loss at once.
    // THE PULL'S DECAY (habituation): chase_pull_decay -- every loss multiplies the chase's pull by this, and the pull
    // recovers by 1/chase_pull_recover_ticks per tick; the need while chasing is the pull.  A thing that keeps getting
    // away loses its hold, the way a known kind does.  1 = no decay.
    int    chase_permanence_ticks_ = 0;
    // PERMANENCE IN RECOGNITION (2026-09-29, after coasting regressed -- §17.61): chase_memory_ticks -- a lost mover is
    // kept in mind (its last predicted position and velocity, extrapolated) for this long WITHOUT driving the walk;
    // a single mover sighting within chase_gate_m of where it should now be re-acquires the chase at once, no
    // confirmation wait.  Where it went is what the look (--stop-on-lost) turns the head toward.  0 = off.
    int    chase_memory_ticks_ = 0;
    // chase_memory_holds (2026-09-29, the operator: "we should definitely be prioritizing the moving objects" -- measured:
    // in the five seconds after a chase ends a static target is held on 87 % of ticks, the block beside the track): while
    // the memory of a lost mover lives, no NEW static target is taken; the mover keeps its priority until forgotten.
    bool   chase_memory_holds_ = false;
    bool   have_memory_ = false; double mem_x_ = 0.0, mem_y_ = 0.0, mem_vx_ = 0.0, mem_vy_ = 0.0, mem_dt_ = 0.0; uint64_t mem_tick_ = 0;
    double chase_pull_decay_ = 1.0, chase_pull_recover_ticks_ = 3000.0, pull_ = 1.0;
    bool   coasting_ = false; uint64_t coast_from_ = 0;
    int    chases_reacquired_ = 0;
    // why candidates do not become chases (2026-09-29): replaced (the next sighting missed the prediction by more than the
    // gate), too fast (implied a speed over chase_v_max), still (chase_min_v failed at confirmation), timed out (forgotten
    // unconfirmed).  A crossing became a candidate 9 times in 10 and a chase 1 in 4 (sweep 6); these say which gate.
    int    cand_replaced_ = 0, cand_fast_ = 0, cand_still_ = 0, cand_timeout_ = 0;
public:
    int cand_replaced() const { return cand_replaced_; }
    int cand_fast()     const { return cand_fast_; }
    int cand_still()    const { return cand_still_; }
    int cand_timeout()  const { return cand_timeout_; }
private:
    void   lose(uint64_t tick_id, double c, double s);
public:
    bool   coasting()          const { return coasting_; }
    int    chases_reacquired() const { return chases_reacquired_; }
    double pull()              const { return pull_; }
private:
public:
    int    chases_lost()    const { return chases_lost_; }
    int    chases_stopped() const { return chases_stopped_; }
    bool   chase_lost_now() const { return lost_now_; }
    // THE GAZE (2026-09-29, the operator: "the robot should be able to turn its head while it's walking to try to reacquire
    // the moving target"): the bearing (body frame, + = right) the head should turn to -- the chased or coasted target's,
    // else the lost mover's extrapolated memory's; NaN when there is nothing moving to look for.
    double chase_gaze_ego() const;
    bool   memory_live() const { return have_memory_; }
    double chase_lost_ego() const { return lost_ego_; }
    double chase_lost_range() const { return lost_range_; }
private:
    int    chase_confirm_ = 2, chase_confirm_ticks_ = 25, chase_forget_ticks_ = 50;
    bool   have_cand_ = false, chasing_ = false, mover_seen_ = false;
    double cand_x_ = 0.0, cand_y_ = 0.0, cand_vx_ = 0.0, cand_vy_ = 0.0;
    uint64_t cand_tick_ = 0, cand_first_ = 0;
    int    cand_n_ = 0, chases_ = 0, chase_ticks_ = 0;
    void   chase_tick(uint64_t tick_id, double c, double s);
public:
    int chase_ticks() const { return chase_ticks_; }
private:

    bool   seen_ = false, have_target_ = false;
    double tx_ = 0.0, ty_ = 0.0;          // the remembered position, odometry frame
    double px_ = 0.0, py_ = 0.0, pyaw_ = 0.0;
    bool   have_pose_ = false;
    float  conf_ = 0.0f, value_ = 0.0f, cx_ = 0.0f, cy_ = 0.0f;
    double range_left_ = 0.0;
    int    arrivals_ = 0, forgets_ = 0, renewals_ = 0;
public:
    int renewals() const { return renewals_; }
};

} // namespace ogma
