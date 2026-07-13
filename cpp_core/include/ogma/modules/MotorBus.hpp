#pragma once

// =============================================================================
// MotorBus.hpp  --  multichannel mixer → bus-compressor for motor influencers
// =============================================================================
//
// Operator's mixing-console model (2026-06-19).  A POPULATION of motor
// "influencers" (HK swim, cognitive decoder, whisker reflex, chemotaxis, …)
// each drive the SAME M motor channels (the 2 cell paddles: left, right).
//
// GAIN STAGING (input trim → fader → bus → compressor):
//   * each influencer is first NORMALIZED to ±1 by its per-channel `scale`
//     (its native full-scale; the ±4 accel convention by default) and CLAMPED
//     to ±1 — so every effector reads on a common 0..1 level and no channel
//     (e.g. the steer+thrust decode, or a saturating reflex) runs past unity;
//   * the FADER (per-channel gain) then scales that normalized signal;
//   * the channels SUM on the bus and pass through a soft COMPRESSOR (tanh)
//     with unity (1.0) as the knee; the result is scaled to the body's accel
//     range by `limit` (the output full-scale):
//
//     out_m = limit · tanh( Σ_i gain_i · clamp(contrib_{i,m} / scale_i, ±1) )
//
// Because the inputs are normalized first, a fader of 1.0 means the same thing
// on every channel and the meter's 0 dB = unity for all of them.
//
// Why a compressor, not the 2-way MotorFader crossfade: the crossfade
// (α·brain + (1−α)·reflex) DILUTES — at α=0.5 it averages two intents into
// mush.  The bus instead SUMS and soft-limits, so:
//   * a single influencer at unity passes through ~linearly (no dilution);
//   * a LOUD channel (whisker fader pegged high + its large contact signal)
//     pushes the bus into tanh SATURATION → it dominates the output and the
//     quiet channels barely move the already-saturated bus = MASKED;
//   * when the loud channel goes SILENT (whisker off-contact → no publish →
//     contributes 0), the bus drops out of saturation and the quiet channels
//     SURFACE again = RELEASE.
// That masking+release is exactly the operator's bus-compressor intuition, and
// it falls straight out of the sum→tanh — no explicit ducking logic.
//
// Influencer kinds (how a source maps to the M motor channels):
//   * "lr"           — topic_a = left ActionOut, topic_b = right ActionOut.
//   * "steer_thrust" — topic_a = steer (differential), topic_b = thrust
//                      (common-mode); decoded to (L,R) = (thrust+steer,
//                      thrust−steer), matching the MotorEPM cell convention.
//
// FRESHNESS / release: an influencer contributes only if it published within
// `active_window` ticks (default 2).  Event-driven sources (the contact
// whisker, silent off-contact) therefore drop to 0 contribution when idle —
// which is what releases the bus.  Always-on sources (HK, cog) are always live.
//
// White-box accessors feed get_motor_bus_state() for the HUD fader panel.

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class MotorBus : public Module {
public:
    MotorBus();
    ~MotorBus() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;   // live viz: meter bridge
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors (tests + HUD fader panel).
    int          n_influencers()        const { return int(chans_.size()); }
    std::string  influencer_name(int i) const { return (i>=0 && i<int(chans_.size())) ? chans_[i].name : std::string(); }
    float        gain(int i)            const { return (i>=0 && i<int(chans_.size())) ? chans_[i].gain : 0.0f; }
    float        eff_gain(int i)        const { return (i>=0 && i<int(chans_.size())) ? chans_[i].eff_gain : 0.0f; }
    float        arb_gain(int i)        const { return (i>=0 && i<int(chans_.size())) ? chans_[i].arb_gain : 1.0f; }
    float        authority(int i)       const { return (i>=0 && i<int(chans_.size())) ? chans_[i].authority : 0.0f; }
    float        mod_value()            const { return mod_value_; }
    float        contrib_l(int i)       const { return (i>=0 && i<int(chans_.size())) ? chans_[i].last_l : 0.0f; }
    float        contrib_r(int i)       const { return (i>=0 && i<int(chans_.size())) ? chans_[i].last_r : 0.0f; }
    bool         active(int i)          const { return (i>=0 && i<int(chans_.size())) ? chans_[i].last_active : false; }
    float        out_l()                const { return out_l_; }   // published (±limit, body accel)
    float        out_r()                const { return out_r_; }
    float        out_norm_l()           const { return out_norm_l_; }   // ±1 (meter)
    float        out_norm_r()           const { return out_norm_r_; }
    float        sum_l()                const { return sum_l_; }
    float        sum_r()                const { return sum_r_; }
    float        limit()               const { return limit_; }
    // Bus gain reduction this tick (0 = none, →1 = heavy compression); the
    // headroom the tanh removed from the raw sum, the meter's "GR" readout.
    float        gain_reduction()       const { return gr_; }

private:
    struct Channel {
        std::string name;
        std::string kind        = "lr";   // "lr" | "steer_thrust"
        std::string topic_a;               // left  | steer
        std::string topic_b;               // right | thrust
        float       scale       = 4.0f;    // input gain-stage: native full-scale → unity (±1)
        float       gain        = 1.0f;    // the fader (static base)
        // 2026-06-20 — SIDECHAIN: dynamic gain response to a modulation signal m∈[0,1]
        // (e.g. boredom).  effective_gain = gain · max(0, 1 + boredom_response·m).
        //   −1 → DUCK to 0 at m=1 (e.g. cog yields when the bug is stuck),
        //   +k → BOOST by (1+k) at m=1 (e.g. hk/escape gets authority when stuck),
        //    0 → no response (default; e.g. a contact reflex independent of boredom).
        float       boredom_response = 0.0f;
        float       eff_gain    = 1.0f;    // gain after sidechain + arbiter (diag/meter)
        // 2026-06-29 — ARBITER GAIN: an optional L2 EFE-arbiter gain on
        // <gain_mod_prefix><name> (ProprioToken scalar, default 1.0 = pass).  Folded
        // into effective_gain for BOTH the mix AND the authority share, so a muted
        // channel (arbiter gain 0) → 0 authority → its advance learning pauses, exactly
        // as a reflex taking the bus suppresses cognitive learning.  Fresh-windowed like
        // the action inputs: stale → reverts to 1.0 (pass), never silently mutes.
        float       arb_gain    = 1.0f;    // latest arbiter gain on this channel
        int64_t     arb_tick    = -1000;   // last tick the arbiter gain published
        float       a_val       = 0.0f;    // latest ActionOut.accel on topic_a
        float       b_val       = 0.0f;    // latest ActionOut.accel on topic_b
        int64_t     a_tick      = -1000;   // last tick topic_a published
        int64_t     b_tick      = -1000;
        float       last_l      = 0.0f;    // contribution to L this tick (post-gain)
        float       last_r      = 0.0f;
        bool        last_active = false;
        // 2026-06-20 — AUTHORITY: this channel's EMA share of the realized bus
        // drive ∈[0,1] (its |contribution| / Σ|contributions|).  Published so a
        // learner on this channel can scale its LEARNING RATE by how much it
        // actually drove the body — credit-by-authority: a muted/ducked/masked
        // channel must not learn from motion it didn't cause (operator insight).
        float       authority   = 0.0f;
    };
    std::vector<Channel> chans_;

    void handle_a(int idx, MessagePtr payload);
    void handle_b(int idx, MessagePtr payload);
    void handle_mod(MessagePtr payload);
    void handle_arb(int idx, MessagePtr payload);

    float       limit_         = 4.0f;
    int64_t     active_window_ = 2;        // ticks of silence before a channel drops to 0
    // 2026-06-22 — PROPORTIONAL mixing.  Default (false): per-channel clamp to ±1 +
    // tanh bus-compressor — both SATURATE, so at high common-mode (thrust) the
    // differential (steer) is clipped away → no turn authority while going fast →
    // the agent must brake to turn.  When true: skip the per-channel clamp and replace
    // tanh with a differential-PRESERVING normalize (scale the L/R pair by its own max),
    // so a turn always expresses — forward yields headroom instead of erasing the turn.
    bool        proportional_mix_ = false;
    // 2026-06-22 — turn-priority brake gain.  head = max(0, 1 − turn_brake·|diff|): how hard
    // the forward (common-mode) yields to the turn (differential).  1 = forward gone only at
    // full steer; >1 = forward yields faster → the inside paddle pivots negative on a moderate
    // turn → tight radius (converge to food) instead of a fixed-radius orbit.  Only used when
    // proportional_mix is on.
    float       turn_brake_       = 1.0f;
    std::string output_left_   = "action.left";
    std::string output_right_  = "action.right";
    // 2026-06-20 — sidechain modulation input (e.g. cognition.boredom, a ReflexGate
    // value ∈[0,1]); drives per-channel boredom_response.  Empty = no sidechain.
    std::string mod_topic_     = "";
    float       mod_value_     = 0.0f;
    // 2026-06-20 — authority publish: per-channel share-of-drive on
    // <authority_prefix><name> (a ProprioToken scalar).  Empty = no publish.
    std::string authority_prefix_  = "";
    float       authority_alpha_   = 0.02f;   // EMA rate for the authority share
    // 2026-06-29 — ARBITER GAIN prefix: subscribe each channel to
    // <gain_mod_prefix><name> (an L2 arbiter's winner-take-all gain).  Empty = no
    // arbiter coupling (default-off compatible; the channel runs at base gain).
    std::string gain_mod_prefix_   = "";

    // working / telemetry
    uint64_t    last_tick_ = 0;
    float       sum_l_ = 0.0f, sum_r_ = 0.0f;        // normalized weighted sum (pre-compress, ±1 units)
    float       out_norm_l_ = 0.0f, out_norm_r_ = 0.0f;  // tanh(sum) ∈ ±1 (compressor output, normalized)
    float       out_l_ = 0.0f, out_r_ = 0.0f;        // published = limit·out_norm (±limit, body accel)
    float       gr_    = 0.0f;
};

} // namespace ogma
