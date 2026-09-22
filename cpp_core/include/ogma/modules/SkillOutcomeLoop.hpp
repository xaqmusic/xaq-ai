// =============================================================================
// SkillOutcomeLoop.hpp  --  learning what an intent does (the duck's things phase, 2026-09-17, O54)
// =============================================================================
//
// The robot ships pre-built intents (a kick, a roll) that a client fires by name.  The operator's
// framing: our brain LEARNING what those intents do is an error we can reduce.  This loop is that
// error, for one intent, the kick, on one question: does the thing answer?
//
//   - infers:   what a thing of THIS kind does when kicked -- per node of the thing vocabulary (the
//               thing EPM's winner while the thing was attended), a running mean and variance of the
//               displacement the kick produced.
//   - senses:   the attended thing's bearing (CloudMap's thing_bearing_topic, live at stops), the thing
//               EPM's token (the node), the seek loop's need and range (arrival), the odometry pose.
//   - acts:     a request on `intent.skill` -- [id, 1] on one tick -- the intent boundary; the host (or
//               the robot's daemon) runs the kick.  The side follows the thing's bearing.
//   - predicts: the displacement, from the node's statistics.
//   - honest signal: the surprise, |observed - predicted| in the node's own spread, published on
//               `outcome_topic` with the node, the prediction and the observation.
//
// WHEN it kicks (the epistemic rule, doctrine §2.2): at an arrival, when the node's answer is still
// uncertain -- fewer than `min_samples` kicks recorded, or a spread above the running mean spread of
// all nodes.  A node whose answer is known is left alone: that is habituation, and it is what lets a
// duck that has kicked the same block four times walk away from it.
//
// WHAT it observes: the thing's position is fixed in the odometry frame while its bearing is live
// (as BearingSeekLoop does).  After the kick, the next live bearing whose position lies within
// `match_radius` of the fixed one is the same thing, moved; its distance from the fixed position is
// the outcome.  If no bearing lands within the radius before `observe_ticks` pass, the outcome is
// UNKNOWN and nothing is learned -- "not seen" is not "did not move" (a reached thing sits below a
// level gaze; the stop's gaze at the thing is what makes this loop honest).  Module absent = byte-identical.
#pragma once

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class SkillOutcomeLoop : public Module {
public:
    SkillOutcomeLoop();
    ~SkillOutcomeLoop() override;

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

    struct Stat { int n = 0; double mean = 0.0, m2 = 0.0; double var() const { return n > 1 ? m2 / (n - 1) : 0.0; } };
    int    requests()  const { return requests_; }
    int    observed()  const { return observed_; }
    int    unknown()   const { return unknown_; }
    double last_surprise() const { return last_surprise_; }
    double need() const { return need_; }
    const std::unordered_map<int, Stat>& stats() const { return stats_; }
    static constexpr int kMaxIntents = 4;
    static int key_of(int node, int intent) { return node * kMaxIntents + intent; }   // intent 0 = kick, 1 = peck, 2 = push

private:
    std::string bearing_topic_ = "percept.thing_bearing";
    std::string thing_topic_   = "reality.cognitive.thing";
    std::string seek_value_topic_ = "reality.cognitive.seek_value";
    std::string seek_range_topic_ = "reality.cognitive.seek_range";
    std::string pose_topic_    = "reality.proprio.odom";
    std::string skill_topic_   = "intent.skill";
    std::string outcome_topic_ = "reality.cognitive.outcome";
    // the need (2026-09-22, the linger): [need, x, y] -- how much of what the intents do to the last attended
    // thing is still unknown (1 - the known intents' share; 0 while an outcome is in flight or nothing was
    // seen) and the thing's fixed position.  The seek loop's renew_topic reads it.  Empty = not published.
    std::string need_topic_;
    double need_ = 0.0;
    double proximity_range_ = 2.5, arrive_range_ = 0.3, match_radius_ = 0.6;
    int    min_samples_ = 2, observe_ticks_ = 1500, min_conf_ticks_ = 5;
    double explore_gain_ = 1.0;
    int    skill_left_ = 0, skill_right_ = 1;
    // a second intent (the peck, Pollen's ground pick): -1 = the kick only.  With two, the loop asks for the
    // one whose answer for THIS thing it knows least -- fewer recorded outcomes, then the larger spread,
    // then the one it did not try last.  A creature that has kicked a block twice and never pecked it pecks.
    int    peck_id_ = -1; int last_intent_ = 1;   // intent 0 = kick (either side), 1 = peck, 2 = push
    // a third intent (the push: the walker into the thing, §17.46): -1 = absent.  The intents present are
    // the vocabulary the least-known rule cycles through at each thing.
    int    push_id_ = -1;
    std::vector<int> intents() const { std::vector<int> v{0}; if (peck_id_ >= 0) v.push_back(1); if (push_id_ >= 0) v.push_back(2); return v; }

    // the thing as last seen: fixed position, node
    bool   seen_ = false; double tx_ = 0.0, ty_ = 0.0; int node_ = -1; int seen_run_ = 0;
    double px_ = 0.0, py_ = 0.0, pyaw_ = 0.0; bool have_pose_ = false;
    float  seek_prev_ = 0.0f;
    // the kick in flight: the fixed position and node before it, the wait for the outcome
    bool   pending_ = false; double kx_ = 0.0, ky_ = 0.0; int knode_ = -1; int wait_ = 0; uint64_t kicked_tick_ = 0;
    bool   request_now_ = false; int request_id_ = 0;
    std::unordered_map<int, Stat> stats_;          // keyed by key_of(node, intent)
    int    kintent_ = 0;
    int    requests_ = 0, observed_ = 0, unknown_ = 0;
    double last_surprise_ = 0.0, last_pred_ = 0.0, last_obs_ = 0.0; int last_node_ = -1;
};

}  // namespace ogma
