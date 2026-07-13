#pragma once

// =============================================================================
// ActionGate.hpp  --  Phase 6.5.26 basal-ganglia-analog action arbiter
// =============================================================================
//
// Arbitrates between parallel motor proposals and emits the final ActionOut
// the body consumes.  Sits one DAG level downstream of Premotor and
// HomeokineticExploration.  Designed as a thin priority gate first; soft
// learned gating can replace the priority logic in a later phase.
//
// First-cut contract:
//
//   exploration.directive (active) ──┐
//   policy.intent (PolicyToken)    ──┼──► ActionGate ──► action.out
//                                    │
// Hard priority:  explore > policy
//   * If ExplorationDirective.active: emit directive.accel, source="explore"
//   * Otherwise:                       emit policy.weighted_accel, source="premotor"
//   * If neither has been received yet (bootstrap): emit accel=0, source=""
//
// Chunk replay (motor.play.stream) is intentionally out of scope here — its
// state machine is currently coupled to ActionDecoder and refactoring it
// cleanly is a separate phase.  If this gate wins on the explore-only
// arbitration, we add chunk arbitration in Phase 6.5.27.
//
// Telemetry: ActionOut.source is set on every emit so smoke / diag can
// measure how often each pathway drove motion.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class ActionGate : public Module {
public:
    ActionGate();
    ~ActionGate() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    ParamMap current_params() const override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests and diag.
    int   policy_count()    const { return policy_count_; }
    int   explore_count()   const { return explore_count_; }
    float last_accel()      const { return last_accel_; }
    std::string const& last_source() const { return last_source_; }

private:
    void handle_policy(std::string_view topic, MessagePtr payload);
    void handle_exploration(std::string_view topic, MessagePtr payload);

    // NOTE: Bus* bus_ is inherited from Module base.  Do not redeclare —
    // shadowing it leaves Module::bus_ stuck at nullptr, which makes
    // Module::on_teardown early-return WITHOUT unsubscribing from the
    // bus, leaving stale `this`-capturing lambdas in the dispatch list
    // after the module is destroyed.  See Premotor.hpp for the same
    // note and the use-after-free post-mortem.
    float       accel_min_          = -4.0f;
    float       accel_max_          =  4.0f;
    // Phase 6.6.D.6 — output channel.  Defaults to "action.out" for back-
    // compat; configs that want bilateral motor sinks instantiate two gates,
    // one with output_topic="action.left" and one with "action.right".
    std::string output_topic_       = "action.out";

    std::shared_ptr<const PolicyToken>          last_policy_;
    std::shared_ptr<const ExplorationDirective> last_explore_;

    int   policy_count_         = 0;
    int   explore_count_        = 0;
    float last_accel_           = 0.0f;
    std::string last_source_;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).
};

} // namespace ogma
