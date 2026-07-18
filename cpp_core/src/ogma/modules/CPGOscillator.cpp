#include "ogma/modules/CPGOscillator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("CPGOscillator param '" + key + "' must be string");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("CPGOscillator param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("CPGOscillator param '" + key + "' must be integer");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("CPGOscillator param '" + key + "' must be string array");
}
std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("CPGOscillator param '" + key + "' must be numeric array");
}

} // namespace

CPGOscillator::CPGOscillator()  = default;
CPGOscillator::~CPGOscillator() = default;

std::string_view CPGOscillator::type_name() const { return "CPGOscillator"; }

std::vector<TopicSpec> CPGOscillator::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(input_topics_.size() + 1);
    for (auto const& t : input_topics_) {
        v.emplace_back(t, std::type_index(typeid(ActionOut)),
                       SubscriptionKind::Direct, /*required=*/false);
    }
    // Phase 7.x — competence-gate input.  neuro.state's reward_signal
    // field (dopamine - adaptive_baseline) drives the slow EMA that
    // scales CPG amplitude.
    v.emplace_back(topics::kNeuroState, std::type_index(typeid(NeuroState)),
                   SubscriptionKind::Direct, /*required=*/false);
    // Phase 7.x — consensus.0 carries fused_tle (the TLE-based
    // competence signal, prop-decoupled measure of brain's predictive
    // accuracy across modalities).
    v.emplace_back(std::string(topics::kConsensusPrefix) + "0",
                   std::type_index(typeid(ConsensusToken)),
                   SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> CPGOscillator::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(output_topics_.size() + perceptual_output_topics_.size());
    for (auto const& t : output_topics_) {
        v.emplace_back(t, std::type_index(typeid(ActionOut)));
    }
    // Phase 7.5 — perceptual-CPG channels emit ProprioToken (consumable by
    // EPMs configured with proprio_state_dims=2).  Declared optional so
    // upstream wiring doesn't break when this mode is disabled.
    for (auto const& t : perceptual_output_topics_) {
        v.emplace_back(t, std::type_index(typeid(ProprioToken)));
    }
    return v;
}

ParamSchema CPGOscillator::params_schema() const {
    return {
        {"input_topics",         ParamMutability::ConstructionOnly,
            "N ActionOut topics this CPG subscribes to (Premotor brain command per joint).",
            std::nullopt},
        {"output_topics",        ParamMutability::ConstructionOnly,
            "N ActionOut topics this CPG publishes to (body's expected action.<joint> topics).  Length must equal input_topics.",
            std::nullopt},
        {"leg_phase_offsets",    ParamMutability::ConstructionOnly,
            "N per-joint leg-cycle phase offsets in radians (which leg-position-in-gait this joint belongs to).  Lateral-sequence walk default = [0, π/2, π, 3π/2] per leg, repeated across joints in a leg.",
            std::nullopt},
        {"joint_phase_offsets",  ParamMutability::ConstructionOnly,
            "N per-joint waveform offsets within the leg's cycle (knee=0, hip2=π/2, hip1=π).",
            std::nullopt},
        {"period_ticks",         ParamMutability::HotMutable,
            "Cycle length in ticks.  Default 60 ≈ 1 sim sec on the 60Hz tick (slow walk for picrawler scale).",
            ParamValue{int64_t{60}}},
        {"amplitude",            ParamMutability::HotMutable,
            "Peak waveform amplitude when competence_gate is fully open.  Effective amp = amplitude_floor + (amplitude − amplitude_floor) * gate.  Default 0.3.",
            ParamValue{0.3}},
        {"amplitude_floor",      ParamMutability::HotMutable,
            "Minimum amplitude when competence_gate = 0 (cold start, no sustained dopamine excess).  Default 0.1 — provides baseline exploration impulse so a tabula-rasa brain has some leg rhythm to learn standing from.  Set to 0 for hard-gated CPG that's fully off at cold start.",
            ParamValue{0.1}},
        {"standing_bias_amplitude", ParamMutability::HotMutable,
            "Magnitude of the cold-start DC standing-bias term: bias_standing = standing_bias_amplitude * (1 − gate) * standing_sign[i].  Fades out as competence_gate opens.  Default 0.3 — strong enough to co-contract knee+hip2 toward chassis-lift at cold start while not overriding learned policy.  Set to 0 to disable the standing-bias mode (walking-rhythm-only behaviour).",
            ParamValue{0.3}},
        {"standing_signs", ParamMutability::ConstructionOnly,
            "Per-joint signed direction toward chassis-lift: array of N floats in {-1, 0, +1} matching input_topics length.  Typical picrawler value: [0, +1, +1] repeated per leg = no yaw bias on hip1, lift-direction bias on hip2 + knee.  Empty (default) = all zeros = no standing bias regardless of standing_bias_amplitude.",
            std::nullopt},
        {"gate_ema_alpha",       ParamMutability::HotMutable,
            "Legacy symmetric EMA rate for the competence gate.  When BOTH gate_ema_alpha_climb and gate_ema_alpha_decline are at their defaults, this value is used in both directions (symmetric EMA, original behaviour).  Specify the directional params instead for asymmetric (hysteresis) behaviour.  Default 0.001 = ~17 sec window @ 60Hz.",
            ParamValue{0.001}},
        {"gate_ema_alpha_climb", ParamMutability::HotMutable,
            "Asymmetric EMA rate when reward_signal is rising (substrate accumulating standing competence).  Larger = gate climbs faster.  Default 0.001 ≈ 17 sec to 63% of incoming positive reward — matches da_baseline_ema_alpha (the substrate's own self-zeroing dopamine baseline rate).  Earlier draft used 0.005 (~3s) but that accumulated brief tabula-rasa standing spikes too aggressively; combined with the slow decline, gate climbed even though body wasn't competent yet.",
            ParamValue{0.001}},
        {"gate_ema_alpha_decline", ParamMutability::HotMutable,
            "Asymmetric EMA rate when reward_signal is falling (substrate losing competence).  Smaller = gate falls slower → competence is remembered longer.  Default 0.0003 ≈ 55 sec to 63% decay.  Provides the hysteresis that lets a brief wobble not collapse competence the substrate took tens of seconds to build up.",
            ParamValue{0.0003}},
        {"gate_scale",           ParamMutability::HotMutable,
            "Dopamine-excess value at which the gate fully opens (gate = clamp(ema_reward_signal / gate_scale, 0, 1)).  Default 0.3 — substrate-tuneable but starts at half the expected steady-state reward_signal range.",
            ParamValue{0.3}},
        {"accel_min",            ParamMutability::HotMutable,
            "Clamp lower bound for the combined (brain + CPG) accel.",
            ParamValue{-1.0}},
        {"accel_max",            ParamMutability::HotMutable,
            "Clamp upper bound for the combined (brain + CPG) accel.",
            ParamValue{1.0}},
        // Phase 7.5 — perceptual-CPG channels.
        {"perceptual_output_topics", ParamMutability::ConstructionOnly,
            "Optional array of M ProprioToken topics (typically one per leg).  When set, CPG publishes a 2-D [cos(phi), sin(phi)] token per channel each tick.  Empty (default) = no perceptual emission (legacy motor-only mode).  Lets the brain perceive gait phase through the perception stream instead of motor injection.",
            std::nullopt},
        {"perceptual_leg_phase_offsets", ParamMutability::ConstructionOnly,
            "Array of M floats (length must equal perceptual_output_topics): per-channel phase offsets in radians.  Trot pattern = [0, π, π, 0] (diagonal pairs in phase); walk = [0, π/2, π, 3π/2]; pace = [0, π, 0, π].",
            std::nullopt},
        {"perceptual_sensor_label", ParamMutability::ConstructionOnly,
            "String written into ProprioToken.sensor for all perceptual emissions.  Default 'cpg'.",
            ParamValue{std::string("cpg")}},
    };
}

ParamMap CPGOscillator::current_params() const {
    ParamMap p;
    p["input_topics"]  = ParamValue{input_topics_};
    p["output_topics"] = ParamValue{output_topics_};
    std::vector<double> lp(leg_phase_offsets_.begin(), leg_phase_offsets_.end());
    std::vector<double> jp(joint_phase_offsets_.begin(), joint_phase_offsets_.end());
    p["leg_phase_offsets"]   = ParamValue{lp};
    p["joint_phase_offsets"] = ParamValue{jp};
    p["period_ticks"]        = ParamValue{int64_t(period_ticks_)};
    p["amplitude"]           = ParamValue{double(base_amplitude_)};
    p["amplitude_floor"]     = ParamValue{double(amplitude_floor_)};
    p["standing_bias_amplitude"] = ParamValue{double(standing_bias_amplitude_)};
    std::vector<double> ss(standing_signs_.begin(), standing_signs_.end());
    p["standing_signs"]      = ParamValue{ss};
    std::vector<double> ws(joint_waveform_signs_.begin(), joint_waveform_signs_.end());
    p["joint_waveform_signs"] = ParamValue{ws};
    p["gate_ema_alpha"]          = ParamValue{double(gate_ema_alpha_)};
    p["gate_ema_alpha_climb"]    = ParamValue{double(gate_ema_alpha_climb_)};
    p["gate_ema_alpha_decline"]  = ParamValue{double(gate_ema_alpha_decline_)};
    p["gate_scale"]          = ParamValue{double(gate_scale_)};
    p["accel_min"]           = ParamValue{double(accel_min_)};
    p["accel_max"]           = ParamValue{double(accel_max_)};
    p["perceptual_output_topics"] = ParamValue{perceptual_output_topics_};
    std::vector<double> ppo(perceptual_leg_phase_offsets_.begin(), perceptual_leg_phase_offsets_.end());
    p["perceptual_leg_phase_offsets"] = ParamValue{ppo};
    p["perceptual_sensor_label"]      = ParamValue{perceptual_sensor_label_};
    return p;
}

void CPGOscillator::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("CPGOscillator requires a non-null Bus");

    auto need = [&](char const* k) -> ParamValue const& {
        auto it = params.find(k);
        if (it == params.end())
            throw std::invalid_argument(std::string("CPGOscillator: required param '") + k + "' missing");
        return it->second;
    };
    // Phase 7.5 — motor side becomes optional so CPG can operate in
    // "pure clock" (perceptual-only) mode for the rhythm-as-perception
    // experiment.  When all four motor-side arrays are absent, the
    // CPG publishes nothing on the motor topics — only the perceptual
    // ProprioToken stream.  When any motor param is present, all four
    // must be present and length-matched (legacy behaviour).
    auto take_strings = [&](char const* k) -> std::vector<std::string> {
        auto it = params.find(k);
        if (it == params.end()) return {};
        return get_string_vec(it->second, k);
    };
    auto take_doubles = [&](char const* k) -> std::vector<double> {
        auto it = params.find(k);
        if (it == params.end()) return {};
        return get_double_vec(it->second, k);
    };
    input_topics_  = take_strings("input_topics");
    output_topics_ = take_strings("output_topics");
    auto lp = take_doubles("leg_phase_offsets");
    auto jp = take_doubles("joint_phase_offsets");
    leg_phase_offsets_.assign(lp.begin(), lp.end());
    joint_phase_offsets_.assign(jp.begin(), jp.end());

    apply_param(params, "period_ticks",   [&](auto const& v){ period_ticks_     = std::max(2, int(get_int(v, "period_ticks"))); });
    apply_param(params, "amplitude",      [&](auto const& v){ base_amplitude_   = std::max(0.0f, float(get_double(v, "amplitude"))); });
    apply_param(params, "amplitude_floor",[&](auto const& v){ amplitude_floor_  = std::max(0.0f, float(get_double(v, "amplitude_floor"))); });
    apply_param(params, "standing_bias_amplitude", [&](auto const& v){ standing_bias_amplitude_ = std::max(0.0f, float(get_double(v, "standing_bias_amplitude"))); });
    apply_param(params, "gate_ema_alpha",         [&](auto const& v){ gate_ema_alpha_         = std::clamp(float(get_double(v, "gate_ema_alpha")), 0.0f, 1.0f); });
    apply_param(params, "gate_ema_alpha_climb",   [&](auto const& v){ gate_ema_alpha_climb_   = std::clamp(float(get_double(v, "gate_ema_alpha_climb")), 0.0f, 1.0f); });
    apply_param(params, "gate_ema_alpha_decline", [&](auto const& v){ gate_ema_alpha_decline_ = std::clamp(float(get_double(v, "gate_ema_alpha_decline")), 0.0f, 1.0f); });
    apply_param(params, "gate_scale",     [&](auto const& v){ gate_scale_       = std::max(1e-6f, float(get_double(v, "gate_scale"))); });
    apply_param(params, "accel_min",      [&](auto const& v){ accel_min_        = float(get_double(v, "accel_min")); });
    apply_param(params, "accel_max",      [&](auto const& v){ accel_max_        = float(get_double(v, "accel_max")); });

    size_t N = input_topics_.size();
    bool motor_any = !input_topics_.empty()
                  || !output_topics_.empty()
                  || !leg_phase_offsets_.empty()
                  || !joint_phase_offsets_.empty();
    if (motor_any) {
        if (N == 0
            || output_topics_.size()       != N
            || leg_phase_offsets_.size()   != N
            || joint_phase_offsets_.size() != N) {
            throw std::invalid_argument(
                "CPGOscillator: when motor side is configured, input_topics, output_topics, leg_phase_offsets, joint_phase_offsets must all be non-empty and same length");
        }
    }

    latest_brain_accel_.assign(N, 0.0f);
    latest_brain_tick_.assign(N, -1);

    // Standing-signs default = all zeros (no standing bias).  Override
    // by config — typical picrawler value [0, +1, +1] repeated per leg.
    standing_signs_.assign(N, 0.0f);
    auto sit = params.find("standing_signs");
    if (sit != params.end()) {
        if (auto p = std::get_if<std::vector<double>>(&sit->second)) {
            if (int(p->size()) != int(N))
                throw std::invalid_argument(
                    "CPGOscillator: standing_signs length must equal input_topics length");
            for (int i = 0; i < int(N); ++i) standing_signs_[i] = float((*p)[i]);
        }
    }
    // Per-joint waveform sign default = all +1.  Override by config —
    // typical picrawler value [+1]*6 + [-1]*6 = front legs same sign,
    // rear legs mirror (so foot-velocity projects to consistent body-
    // relative direction on a 4-fold-symmetric chassis).
    joint_waveform_signs_.assign(N, 1.0f);
    auto wsit = params.find("joint_waveform_signs");
    if (wsit != params.end()) {
        if (auto p = std::get_if<std::vector<double>>(&wsit->second)) {
            if (int(p->size()) != int(N))
                throw std::invalid_argument(
                    "CPGOscillator: joint_waveform_signs length must equal input_topics length");
            for (int i = 0; i < int(N); ++i) joint_waveform_signs_[i] = float((*p)[i]);
        }
    }

    // Phase 7.5 — perceptual-CPG channels (optional).  When present, CPG
    // additionally publishes a ProprioToken per channel each tick carrying
    // [cos(phi_c), sin(phi_c)].  Length consistency enforced.
    auto pit = params.find("perceptual_output_topics");
    if (pit != params.end()) {
        if (auto p = std::get_if<std::vector<std::string>>(&pit->second)) {
            perceptual_output_topics_ = *p;
        }
    }
    auto poit = params.find("perceptual_leg_phase_offsets");
    if (poit != params.end()) {
        if (auto p = std::get_if<std::vector<double>>(&poit->second)) {
            perceptual_leg_phase_offsets_.assign(p->begin(), p->end());
        }
    }
    auto psit = params.find("perceptual_sensor_label");
    if (psit != params.end()) {
        if (auto p = std::get_if<std::string>(&psit->second)) {
            perceptual_sensor_label_ = *p;
        }
    }
    if (!perceptual_output_topics_.empty()
        && perceptual_leg_phase_offsets_.size() != perceptual_output_topics_.size()) {
        throw std::invalid_argument(
            "CPGOscillator: perceptual_output_topics and perceptual_leg_phase_offsets must have equal length");
    }
    if (!motor_any && perceptual_output_topics_.empty()) {
        throw std::invalid_argument(
            "CPGOscillator: at least one of (motor side: input_topics+output_topics+...) or (perceptual side: perceptual_output_topics+perceptual_leg_phase_offsets) must be configured");
    }

    sub_ids_.clear();
    for (int i = 0; i < int(N); ++i) {
        sub_ids_.push_back(bus_->subscribe(input_topics_[i], SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ this->handle_brain_action(i, p); }));
    }
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_neuro(p); }));
    // Phase 7.x — TLE-based competence gate.  Subscribe to consensus.0
    // for the trust-weighted fused prediction error across modalities.
    sub_ids_.push_back(bus_->subscribe(std::string(topics::kConsensusPrefix) + "0",
        SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_consensus(p); }));
}

void CPGOscillator::handle_consensus(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto c = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!c) return;
    latest_fused_tle_ = c->fused_tle;
    // Skip zero-valued observations — LateralVoter initialises
    // ConsensusToken.fused_tle to 0 in its first emission, before any
    // RealityToken with a meaningful TLE has arrived.  If we accept
    // that as the first observation, consensus_seen_ flips true with
    // ema=max=0, and the NEXT observation (real TLE) only updates max
    // (since ema goes through the slow-EMA path) — resulting in
    // gate=1 the moment a real TLE arrives, even though the substrate
    // has no competence yet.  Wait for the first meaningful TLE so
    // ema and max can be initialised TOGETHER, giving gate=0 at the
    // true cold start.
    if (latest_fused_tle_ <= 1e-6f) return;
    if (!consensus_seen_) {
        ema_fused_tle_      = latest_fused_tle_;
        max_fused_tle_seen_ = latest_fused_tle_;
        consensus_seen_     = true;
        return;
    }
    // Symmetric EMA at climb rate.  We want both directions to track at
    // the same gradual pace — competence is earned patiently and lost
    // patiently.  A single TLE spike shouldn't collapse competence; a
    // single low-TLE sample shouldn't mint competence.
    float alpha = gate_ema_alpha_climb_;
    ema_fused_tle_ = (1.0f - alpha) * ema_fused_tle_ + alpha * latest_fused_tle_;
    // Peak tracks ema with hard ratchet — single-tick TLE spikes don't
    // propagate to ema (ema is already slow-smoothed), so the ratchet
    // captures only sustained ema highs.
    if (ema_fused_tle_ > max_fused_tle_seen_)
        max_fused_tle_seen_ = ema_fused_tle_;
}

void CPGOscillator::handle_neuro(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto n = std::dynamic_pointer_cast<const NeuroState>(payload);
    if (!n) return;
    // Always start ema_reward_signal_ at 0 (the member default).  Earlier
    // code snapshotted the first observation to "avoid warm-up delay",
    // but at scene start the body's initial pose is at standing height
    // and the height-reward block fires events.hit before gravity drops
    // the chassis.  That brief startup burst drove the first NeuroState's
    // reward_signal high — the CPG then initialised its EMA to that
    // value, the hysteresis-decline rate (~55s to 63% decay) kept the
    // gate stuck near 1 for nearly a minute even after the body fell.
    //
    // Initialising at 0 instead lets the gate warm up over ~3 sec via
    // gate_ema_alpha_climb (0.005), driven by actual sustained
    // standing competence rather than the artificial startup-pose
    // standing-reward spike.
    neuro_seen_ = true;
    // Hysteresis: climb fast (substrate quickly recognises competence
    // when reward_signal is above the EMA), fall slow (competence is
    // remembered for ~tens of seconds before the standing-bias prop
    // re-engages).  Resolves the user-observed "robot stood well for
    // 1 min but standing_factor is still 0.3" lag.
    float alpha = (n->reward_signal > ema_reward_signal_)
                   ? gate_ema_alpha_climb_
                   : gate_ema_alpha_decline_;
    ema_reward_signal_ = (1.0f - alpha) * ema_reward_signal_
                          + alpha * n->reward_signal;
}

float CPGOscillator::competence_gate() const {
    // Phase 7.x — competence has two evidence paths combined via
    // geometric mean (both must agree), then accumulated via a
    // ratchet so demonstrated competence is remembered:
    //
    //   (a) Predictive: ema_fused_tle dropping below its historical
    //       peak = brain's predictor is improving (Playful Machine
    //       measure).  Robust to prop-contamination — prediction
    //       error is independent of who's driving the body.
    //
    //   (b) Behavioural: ema_reward_signal sustained above baseline =
    //       substrate is consistently above-baseline rewarded,
    //       whatever the source.  This is NOT prop "contamination":
    //       if the scaffold helps, rewards sustain, scaffold fades,
    //       and the brain has to take over — that IS the test.
    //
    // Geometric mean sqrt(g_tle * g_reward) demands joint evidence:
    //   - Cold start: g_reward ≈ 0 → gate ≈ 0 (no behavioural ev)
    //   - Mid: CPG firing rewards but no brain learning → g_tle ≈ 0
    //     → gate ≈ 0 (no predictive ev)
    //   - Both climb → gate opens
    //
    // The RATCHET then preserves the highest joint-evidence value
    // seen, with slow decay (~17 min half-life).  Without the ratchet,
    // TLE-spike noise was snapping standing-factor back to max — user
    // observed the scaffold remaining "pegged at max even after 3 min
    // of continuous standing" because momentary high-TLE events
    // ratcheted peak and collapsed instant gate to 0.
    float g_tle = 0.0f;
    if (consensus_seen_ && max_fused_tle_seen_ > 1e-6f) {
        g_tle = std::clamp(1.0f - ema_fused_tle_ / max_fused_tle_seen_,
                           0.0f, 1.0f);
    }
    float g_reward = 0.0f;
    if (neuro_seen_)
        g_reward = std::clamp(ema_reward_signal_ / gate_scale_, 0.0f, 1.0f);
    float instant = std::sqrt(g_tle * g_reward);
    if (instant > gate_ratchet_) {
        gate_ratchet_ = instant;          // climb fast — accept any new high
    } else {
        gate_ratchet_ *= (1.0f - gate_decay_per_tick_);  // slow decay
    }
    return gate_ratchet_;
}

void CPGOscillator::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "period_ticks")   period_ticks_   = std::max(2, int(get_int(value, k)));
    else if (k == "amplitude")      base_amplitude_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "amplitude_floor")amplitude_floor_= std::max(0.0f, float(get_double(value, k)));
    else if (k == "standing_bias_amplitude") standing_bias_amplitude_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "gate_ema_alpha")         gate_ema_alpha_         = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "gate_ema_alpha_climb")   gate_ema_alpha_climb_   = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "gate_ema_alpha_decline") gate_ema_alpha_decline_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "gate_scale")     gate_scale_     = std::max(1e-6f, float(get_double(value, k)));
    else if (k == "accel_min")      accel_min_      = float(get_double(value, k));
    else if (k == "accel_max")      accel_max_      = float(get_double(value, k));
    else
        throw std::invalid_argument("CPGOscillator: param '" + k + "' is ConstructionOnly");
}

void CPGOscillator::handle_brain_action(int joint_idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    if (joint_idx >= 0 && joint_idx < int(latest_brain_accel_.size())) {
        latest_brain_accel_[joint_idx] = a->accel;
        latest_brain_tick_[joint_idx]  = int64_t(a->tick_id);
    }
}

void CPGOscillator::tick(uint64_t tick_id) {
    // Advance phase.
    const float TWO_PI = 6.28318530717958647692f;
    phase_ += TWO_PI / float(period_ticks_);
    while (phase_ >= TWO_PI) phase_ -= TWO_PI;

    // Phase 7.x — competence-gated amplitude with cold-start floor.
    // effective = amplitude_floor + (amplitude − amplitude_floor) * gate
    // so cold start (gate=0) still emits a baseline exploration
    // impulse (tabula-rasa crawl rhythm) while sustained dopamine
    // excess (gate→1) ramps amplitude up to the configured peak for
    // walking dynamics.
    //
    // Phase 7.x — additive standing-bias term that's strong at gate=0
    // and fades to 0 as gate opens.  Provides the DC co-contraction
    // signal a tabula-rasa body needs to bootstrap chassis-lift
    // (walking-rhythm-alone alternates legs and cancels chassis lift).
    // Phase 7.x — three-phase progression via bell-curve walking amp:
    //
    //   gate=0   (cold start, brain hasn't learned body):
    //       standing_factor = standing_bias_amplitude (max prop)
    //       walking_amp     = amplitude_floor (≈ 0 — no walking rhythm)
    //
    //   gate=0.5 (learning in progress, prop fading, rhythm peak):
    //       standing_factor = standing_bias_amplitude / 2
    //       walking_amp     = base_amplitude (peak)
    //
    //   gate=1   (confident, brain models body without help):
    //       standing_factor = 0
    //       walking_amp     = amplitude_floor (≈ 0 — CPG retires)
    //
    // Walking uses sin(π*gate) shape so it ramps up THEN ramps down as
    // competence saturates — the CPG is a SCAFFOLD that fully retires
    // once the brain has learned, not a permanent rhythm driver.
    // Matches the Playful Machine "competence-based fade-out" idea.
    float gate = competence_gate();
    const float PI = 3.14159265358979323846f;
    float walk_bell = std::sin(PI * gate);    // 0 at gate=0, 1 at gate=0.5, 0 at gate=1
    float effective_amp = amplitude_floor_ + (base_amplitude_ - amplitude_floor_) * walk_bell;
    if (effective_amp < 0.0f) effective_amp = 0.0f;
    // Standing factor uses quadratic falloff (1-gate)^2 instead of linear
    // (1-gate).  At small gate values the linear mapping barely moved
    // standing_factor — user observed it pegged at max even after several
    // minutes of standing-like behaviour with gate at ~0.07.  Quadratic
    // makes small gate gains visible: gate=0.1 → 0.81×max (vs 0.90×max
    // linear); gate=0.3 → 0.49×max (vs 0.70× linear).  The shape
    // preserves the boundary conditions (max at gate=0, zero at gate=1)
    // and matches the bell-curve walking amp's softer-than-linear
    // character.
    float stand_fade  = 1.0f - gate;
    float standing_factor = standing_bias_amplitude_ * stand_fade * stand_fade;
    last_walking_amp_     = effective_amp;
    last_standing_factor_ = standing_factor;

    int N = int(input_topics_.size());
    if (int(last_bias_walking_.size())  != N) last_bias_walking_.assign(N, 0.0f);
    if (int(last_bias_standing_.size()) != N) last_bias_standing_.assign(N, 0.0f);
    if (int(last_blended_.size())       != N) last_blended_.assign(N, 0.0f);
    for (int i = 0; i < N; ++i) {
        float phi = phase_ + leg_phase_offsets_[i] + joint_phase_offsets_[i];
        float bias_w = effective_amp * std::sin(phi) * joint_waveform_signs_[i];
        float bias_s = standing_factor * standing_signs_[i];
        float bias = bias_w + bias_s;
        last_bias_walking_[i]  = bias_w;
        last_bias_standing_[i] = bias_s;

        // Use the LATEST brain command we've observed, regardless of
        // tick_id.  If the scheduler runs Premotor before CPG in the
        // same tick we get the current command; otherwise we get
        // last-tick's command (1-tick lag = efferent copy delay; matches
        // the biological spinal-CPG-reads-descending-command pattern).
        // Was: strict (tick_id == tick_id) freshness check, which
        // dropped brain commands entirely when CPG ticked first and
        // produced a CPG-only motor output that couldn't stand.
        float brain = latest_brain_accel_[i];
        float blended = std::clamp(brain + bias, accel_min_, accel_max_);
        last_blended_[i] = blended;

        auto out = std::make_shared<ActionOut>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("cpg") : id_;
        out->accel       = blended;
        out->source      = "cpg";
        bus_->publish(output_topics_[i], out);
        ++total_outputs_;
    }

    // Phase 7.5 — perceptual-CPG broadcast.  Independent of motor side
    // (no clamps, no brain-blend) — pure clock signal.  Uses the same
    // advanced phase_ so motor + perceptual streams are phase-aligned.
    int M = int(perceptual_output_topics_.size());
    for (int c = 0; c < M; ++c) {
        float phi = phase_ + perceptual_leg_phase_offsets_[c];
        auto pt = std::make_shared<ProprioToken>();
        pt->tick_id     = tick_id;
        pt->producer_id = id_.empty() ? std::string("cpg") : id_;
        pt->sensor      = perceptual_sensor_label_;
        pt->values.resize(2);
        pt->values(0) = std::cos(phi);
        pt->values(1) = std::sin(phi);
        bus_->publish(perceptual_output_topics_[c], pt);
    }
}

nlohmann::json CPGOscillator::snapshot_state() const {
    // Surface per-tick cached state + config params so the xaq_inspector
    // CPGInspector widget can chart competence gate, amplitude blend,
    // and per-joint bias breakdown live.  Not used for clone/restore —
    // CPG is a stateless-modulo-phase oscillator and a future restore
    // would just re-derive phase from the tick clock.
    auto to_array = [](std::vector<float> const& v) {
        nlohmann::json a = nlohmann::json::array();
        for (auto x : v) a.push_back(double(x));
        return a;
    };
    auto to_str_array = [](std::vector<std::string> const& v) {
        nlohmann::json a = nlohmann::json::array();
        for (auto const& s : v) a.push_back(s);
        return a;
    };
    return nlohmann::json{
        {"version",                 1},
        {"competence_gate",         double(competence_gate())},
        {"ema_reward_signal",       double(ema_reward_signal_)},
        {"ema_fused_tle",           double(ema_fused_tle_)},
        {"max_fused_tle_seen",      double(max_fused_tle_seen_)},
        {"latest_fused_tle",        double(latest_fused_tle_)},
        {"phase",                   double(phase_)},
        {"last_walking_amp",        double(last_walking_amp_)},
        {"last_standing_factor",    double(last_standing_factor_)},
        {"base_amplitude",          double(base_amplitude_)},
        {"amplitude_floor",         double(amplitude_floor_)},
        {"standing_bias_amplitude", double(standing_bias_amplitude_)},
        {"gate_ema_alpha",          double(gate_ema_alpha_)},
        {"gate_ema_alpha_climb",    double(gate_ema_alpha_climb_)},
        {"gate_ema_alpha_decline",  double(gate_ema_alpha_decline_)},
        {"gate_scale",              double(gate_scale_)},
        {"period_ticks",            period_ticks_},
        {"n_joints",                n_joints()},
        {"total_outputs",           total_outputs_},
        {"last_bias_walking",       to_array(last_bias_walking_)},
        {"last_bias_standing",      to_array(last_bias_standing_)},
        {"last_blended",            to_array(last_blended_)},
        {"standing_signs",          to_array(standing_signs_)},
        {"joint_waveform_signs",    to_array(joint_waveform_signs_)},
        {"leg_phase_offsets",       to_array(leg_phase_offsets_)},
        {"joint_phase_offsets",     to_array(joint_phase_offsets_)},
        {"output_topics",           to_str_array(output_topics_)},
        {"perceptual_output_topics",   to_str_array(perceptual_output_topics_)},
        {"perceptual_leg_phase_offsets", to_array(perceptual_leg_phase_offsets_)},
        {"perceptual_sensor_label",    perceptual_sensor_label_},
        {"n_perceptual_channels",      n_perceptual_channels()},
    };
}

} // namespace ogma
