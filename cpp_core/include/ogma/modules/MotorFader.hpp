#pragma once

// =============================================================================
// MotorFader.hpp  --  Brain↔reflex blender (Phase 6.6.G refactor)
// =============================================================================
//
// History:
//   - 6.6.F: single-channel module with α-computation built in.
//   - 6.6.G: α-computation extracted into FaderController.  MotorFader is
//            now a pure blender — one per body actuator.  Multiple
//            instances read the same FaderState off motor.fader.alpha
//            and apply that α to their own (brain, reflex) ActionOut
//            pair, publishing the blend on output_topic.
//
// Backwards compatibility: a graph that does NOT include a
// FaderController is supported.  When no FaderState ever arrives,
// MotorFader falls back to alpha_fixed (default 0 → reflex-only).
// This preserves existing single-channel behavior bit-for-bit when
// alpha_fixed is set explicitly and no controller is wired in.
//
// MotorFader does NOT publish FaderState in 6.6.G; the FaderController
// owns that topic.  Per-instance brain/reflex/output accel telemetry is
// available via white-box accessors so the HUD meter can read each
// channel directly (see step 6 of the 6.6.G plan).

#include <memory>
#include <random>
#include <string>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class MotorFader : public Module {
public:
    MotorFader();
    ~MotorFader() override;

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

    // White-box accessors for tests + HUD meter (step 6 will read these).
    float alpha()         const { return alpha_; }
    float brain_accel()   const { return last_brain_accel_; }
    float reflex_accel()  const { return last_reflex_accel_; }
    float output_accel()  const { return last_output_accel_; }
    bool  brain_seen()    const { return last_brain_seen_; }
    bool  reflex_seen()   const { return last_reflex_seen_; }
    bool  alpha_from_bus()const { return alpha_from_bus_; }
    int   publish_count() const { return publish_count_; }
    float last_surprise() const { return last_surprise_; }
    float last_noise()    const { return last_noise_; }
    // v6.0.d — inverted-babbler entropy state.  Returns 1.0 (max-entropy
    // proxy = "no info, no amplification") when no PolicyToken has been
    // seen yet, so a config with entropy_gain > 0 but no Premotor in
    // the graph behaves like baseline 6.6.P noise.
    float last_entropy_normalized() const { return last_entropy_normalized_; }
    int   policy_msgs_received() const { return policy_msgs_received_; }
    // v5.4 — per-tick clash measurement.  Quantifies how much intent each
    // contributor *lost* when the brain and reflex commands disagreed in
    // sign.  clash = max(0, |α·brain| + |(1-α)·reflex| - |output|).  When
    // signs agree → 0 (purely additive); when signs oppose → 2·min of the
    // two contributions (the magnitude of intent erased on both sides of
    // the tug-of-war).  last_clash is this tick's value; clash_ema is an
    // EMA so a HUD widget can show "are signals usually fighting?"
    float last_clash()    const { return last_clash_; }
    float clash_ema()     const { return clash_ema_; }

private:
    void handle_brain (MessagePtr payload);
    void handle_reflex(MessagePtr payload);
    void handle_alpha (MessagePtr payload);
    void handle_policy(MessagePtr payload);   // v6.0.d — inverted babbler

    // Configuration
    std::string brain_topic_   = "action.brain";
    std::string reflex_topic_  = "action.reflex";
    std::string alpha_topic_   = topics::kMotorFaderAlpha;
    std::string output_topic_  = topics::kActionOut;
    // Fallback α used when no FaderController is wired into the graph.
    // Once any FaderState arrives on alpha_topic_ this value is ignored.
    float       alpha_fixed_   = 0.0f;
    // 2026-06-19 — contact-subsumption blend for a SILENT-when-idle reflex
    // (e.g. the cell whisker only publishes on wall contact).  The blend
    // α·brain + (1−α)·reflex attenuates the brain to α·brain whenever the
    // reflex channel is idle (reflex_accel defaults to 0), which would
    // cripple free-swim at any α<1.  When this is true and no reflex
    // ActionOut arrived this tick, the brain passes through UNATTENUATED
    // (effective α=1); the reflex only ducks the brain WHEN it actually
    // fires.  This is the "silent channel = no gain reduction" behaviour of
    // a bus compressor.  Default false → pre-existing configs byte-identical.
    bool        idle_reflex_passthrough_ = false;
    // Phase 6.6.P — inverted-babbler motor noise.  Gaussian noise added to
    // the blended action, gain scaled by (1 - clamp(surprise_scalar, 0, 1)).
    // When the predictor is well-calibrated (low surprise), noise is loud —
    // breaks the dark-room attractor.  When something is genuinely
    // surprising, noise drops and the brain can act decisively.
    // Default 0 → behaviorally identical to pre-6.6.P.
    float       noise_amplitude_ = 0.0f;
    uint64_t    noise_seed_      = 0;
    // v6.0.d — inverted-babbler entropy coupling (Playful Machine #1).
    // When the brain's policy is committed (low entropy → policy locked in)
    // AND the world is predictable (low surprise), inject MORE noise to
    // break the attractor.  Effective noise gain becomes:
    //   gain = noise_amplitude * (1 − surprise) * (1 + entropy_gain * (1 − H/ln(N)))
    // entropy_gain=0 (default) keeps the legacy 6.6.P behaviour bit-for-bit.
    // entropy_topic empty (default) means "don't subscribe"; the entropy
    // input then stays at its "no info" value (norm_entropy=1, multiplier=1).
    std::string entropy_topic_  = "";
    float       entropy_gain_   = 0.0f;
    float       last_entropy_normalized_ = 1.0f;  // 1.0 ≡ max entropy / no info
    int         policy_msgs_received_    = 0;

    // Working state
    float alpha_              = 0.0f;
    bool  alpha_from_bus_     = false;
    float last_brain_accel_   = 0.0f;
    float last_reflex_accel_  = 0.0f;
    float last_output_accel_  = 0.0f;
    float last_surprise_      = 0.0f;
    float last_noise_         = 0.0f;
    bool  last_brain_seen_    = false;
    bool  last_reflex_seen_   = false;
    int   publish_count_      = 0;
    // v5.4 clash tracking.
    float last_clash_         = 0.0f;
    float clash_ema_          = 0.0f;
    static constexpr float kClashEmaAlpha_ = 0.02f;  // ~50-tick half-life

    std::mt19937 rng_;

    // Per-tick input cache (cleared at end of tick()).
    std::shared_ptr<const ActionOut> pending_brain_;
    std::shared_ptr<const ActionOut> pending_reflex_;
};

} // namespace ogma
