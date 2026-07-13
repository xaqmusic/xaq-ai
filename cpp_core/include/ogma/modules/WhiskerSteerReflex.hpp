#pragma once

// =============================================================================
// WhiskerSteerReflex.hpp  --  Phase 6.6.D.7 directed-escape steering reflex
// =============================================================================
//
// Sibling to WhiskerAversionReflex.  Subscribes to a prefix of whisker
// proprio topics, classifies each incoming sensor as left-side or right-side
// based on a configured ID-suffix list, and per-tick when max contact >
// threshold publishes ActionOut on action.left and action.right with thrust
// biased AWAY from the more-contacting side.
//
// Body convention reminder (body_controller.gd):
//   rate_left  = base + bias × steer
//   rate_right = base − bias × steer
//   positive steer → CW rotation → right turn
//   bilateral mapping (Phase 6.6.D.6): accel = al − ar
//
// So contact on LEFT whiskers (whisker_0..2 by default) → we want a RIGHT
// turn (steer away) → accel positive → al > ar.  Contact on RIGHT
// whiskers → al < ar → left turn.  Symmetric (frontal) contact → no
// steering bias, both sides emit equal thrust and the body just waits out
// the refractory.
//
// Output rule:
//   al = base_thrust + steer_gain · (left_max − right_max)
//   ar = base_thrust − steer_gain · (left_max − right_max)
//
// Bypasses ActionGate by publishing directly to action.left / action.right
// — the body's bilateral pathway (Phase 6.6.D.6) treats simultaneous L+R
// publications as the active source, naturally overriding any ActionGate
// output on action.out for the duration of contact.  This is the reflex-
// priority hack we needed without requiring a new ExplorationDirective
// payload variant.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class WhiskerSteerReflex : public Module {
public:
    WhiskerSteerReflex();
    ~WhiskerSteerReflex() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests.
    int   fire_count()    const { return fire_count_; }
    float last_left()     const { return last_al_; }
    float last_right()    const { return last_ar_; }
    bool  last_skipped()  const { return last_skipped_; }
    bool  last_head_on()  const { return last_head_on_; }
    int   last_head_dir() const { return last_head_on_dir_; }

private:
    void handle_whisker(std::string_view topic, MessagePtr payload);
    bool is_left(std::string const& topic) const;
    bool is_right(std::string const& topic) const;

    std::string whisker_topic_prefix_  = "reality.proprio.whisker_";
    // Suffixes that follow whisker_topic_prefix_ for left/right side.  e.g.
    // for prefix "reality.proprio.whisker_" with suffix "0" the full topic
    // matched is "reality.proprio.whisker_0".  Allows configurable mapping
    // without baking the body geometry into the module.
    std::vector<std::string> left_suffixes_  = {"0", "1", "2"};
    std::vector<std::string> right_suffixes_ = {"3", "4", "5"};
    // 2026-06-18 — refined rules: per-whisker position weights (outer→inner), so
    // a wall straight AHEAD (inner whiskers) turns harder than one to the SIDE
    // (outer).  Aligned to left_suffixes_ (0,1,2 = outer→inner) and right_suffixes_
    // (3,4,5 = inner→outer).  Per side the contributions are SUMMED → more whiskers
    // engaged = stronger turn-away.
    std::vector<float> left_weights_  = {0.5f, 0.75f, 1.0f};
    std::vector<float> right_weights_ = {1.0f, 0.75f, 0.5f};
    std::unordered_map<std::string, float> topic_weight_;
    // Wedge/corner REVERSE: when BOTH sides are in contact the turn-away signal is
    // ambiguous (boxed in), so flip to a strong common-mode BACK-OUT (both paddles
    // reverse), guaranteed at least this magnitude.  bidirectional_paddler only.
    float reverse_strength_  = 4.0f;

    float threshold_         = 0.30f;
    float base_thrust_       = 0.0f;
    float steer_gain_        = 10.0f;  // accel ∈ [-4,4]; weighted-sum × gain saturates fast → decisive turn
    float accel_min_         = -4.0f;
    float accel_max_         =  4.0f;
    int   refractory_ticks_  = 0;      // 0 = fire every tick contact persists
    // 2026-06-22 — a HELD kick.  A 1-tick fire barely imparts angular impulse; hold the
    // steer-away (latched) for pulse_ticks ticks so the bug actually turns, THEN start the
    // refractory release.  pulse_ticks 1 = legacy single-tick fire.
    int   pulse_ticks_       = 1;
    // Phase 6.6.G — symmetric-contact + head-on handling.  These three
    // params close the bilateral regression where:
    //  (a) on near-symmetric whisker contact (left_max ≈ right_max),
    //      WhiskerSteer used to publish (0, 0), clobbering any
    //      ForwardDriveReflex thrust on the same action.reflex.{l,r}
    //      topics — leaving the modular-passive body with no spike
    //      drive at all.  `min_steer_threshold` lets the module skip
    //      its publish when the differential signal is tiny so the
    //      forward thrust survives.
    //  (b) on head-on contact (both sides above head_on_threshold)
    //      the differential vanishes but the agent IS stuck — the
    //      body needs a rotation kick.  WhiskerSteer now picks a
    //      direction (deterministic, master-seeded) on each fresh
    //      head-on event and publishes ±head_on_rotation per side
    //      until contact subsides.  Direction is held across ticks
    //      so the agent doesn't oscillate.
    // Defaults preserve legacy behaviour: min_steer=0 means "always
    // publish"; head_on_threshold=0 means "never enter head-on mode".
    float    min_steer_threshold_ = 0.0f;
    float    head_on_threshold_   = 0.0f;
    float    head_on_rotation_    = 4.0f;
    uint64_t master_seed_         = 0;
    // Phase 6.6.F — when non-empty, publish a single ActionOut with
    // accel = (al − ar) (steering signal collapsed into one channel)
    // to this topic instead of the bilateral pair.  Lets the module
    // serve as the reflex-side input to a single-channel MotorFader
    // without first lifting the fader to bilateral.  Empty default
    // preserves the original bilateral pathway.
    std::string output_topic_  = "";
    // Phase 6.6.G — bilateral output topic redirects.  Defaults preserve
    // the existing action.left / action.right pathway.  Setting these to
    // e.g. action.reflex.left / action.reflex.right routes the steering
    // pair through a bilateral MotorFader.
    std::string output_topic_left_  = std::string(topics::kActionLeft);
    std::string output_topic_right_ = std::string(topics::kActionRight);
    // Phase 6.6.F.1 — optional suppression input.  When set to a
    // ReflexGate topic (e.g. ScentGateReflex's output), the steering
    // sign is mediated by the gate's value g ∈ [0, cap]:
    //
    //   effective_gain = steer_gain * (1 - 2 * g)
    //
    // Semantics:
    //   g = 0     → full aversion (away from contact); legacy behaviour
    //   g = 0.5   → no steering (let the body coast through contact)
    //   g = 1.0   → full attraction (toward contact, e.g. into food)
    //
    // Empirically, ScentGateReflex caps g around 0.5, which collapses
    // aversion when scent is rising — the body charges toward food
    // through whiskers instead of being deflected away.  Empty default
    // disables the gate (legacy aversion behaviour preserved).
    std::string suppression_topic_ = "";
    float       last_suppression_  = 0.0f;

    // sensor_topic -> latest scalar
    std::unordered_map<std::string, float> last_values_;
    // pre-built fast-path lookups
    std::unordered_set<std::string> left_topics_;
    std::unordered_set<std::string> right_topics_;

    int   refractory_remaining_ = 0;
    int   pulse_remaining_      = 0;       // ticks left in the current held kick
    float latched_al_           = 0.0f;    // kick commands held across the pulse
    float latched_ar_           = 0.0f;
    bool  latched_head_on_      = false;
    int   fire_count_           = 0;
    float last_al_              = 0.0f;
    float last_ar_              = 0.0f;
    bool  last_skipped_         = false;
    bool  last_head_on_         = false;
    int   last_head_on_dir_     = 0;       // 0 = not committed, ±1 = CW/CCW held direction
    std::mt19937 head_on_rng_;
};

} // namespace ogma
