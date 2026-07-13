// PremotorAI.cpp — Premotor active-inference upgrade (2026-06-03).
//
// STATUS: renamed clone of Premotor.cpp. NO behavioural changes yet.
// Currently bit-equivalent to Premotor. See PremotorAI.hpp top comment for
// why the EFE additions are pending the R1/R2 substrate-repair design.

#include "ogma/modules/PremotorAI.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("PremotorAI param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("PremotorAI param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("PremotorAI param '" + key + "' must be bool");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PremotorAI param '" + key + "' must be string");
}

void validate_phase_commit_mode(std::string const& mode) {
    if (mode == "none" || mode == "switch_penalty" || mode == "bin_boundary") return;
    throw std::invalid_argument(
        "PremotorAI param 'phase_commit_mode' must be one of: none, switch_penalty, bin_boundary");
}

} // namespace

PremotorAI::PremotorAI()  = default;
PremotorAI::~PremotorAI() = default;

std::string_view PremotorAI::type_name() const { return "PremotorAI"; }

std::vector<TopicSpec> PremotorAI::input_topics() const {
    // We resolve the consensus topic at on_setup time using `level`; the
    // contract here is "subscribe to consensus.<level>".  The Bus does
    // exact-match on a non-trailing-dot pattern, so we can declare it.
    std::string consensus_topic = std::string(topics::kConsensusPrefix) + std::to_string(level_);
    std::vector<TopicSpec> v{
        TopicSpec{consensus_topic,        std::type_index(typeid(ConsensusToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kDriveErrors,   std::type_index(typeid(DriveErrors)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kNeuroState,    std::type_index(typeid(NeuroState)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kEventsPrefix,  std::type_index(typeid(EnvEvent)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
    if (!phase_context_topic_.empty()) {
        v.push_back(TopicSpec{phase_context_topic_, std::type_index(typeid(ProprioToken)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    if (!bucket_context_topic_.empty() && n_buckets_ > 0) {
        v.push_back(TopicSpec{bucket_context_topic_, std::type_index(typeid(ProprioToken)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    if (!leg_phase_input_topic_.empty() && leg_phase_gain_ > 0.0f) {
        v.push_back(TopicSpec{leg_phase_input_topic_, std::type_index(typeid(ProprioToken)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    if (!intent_override_topic_.empty()) {
        v.push_back(TopicSpec{intent_override_topic_, std::type_index(typeid(IntentToken)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    if (!explore_directive_topic_.empty()) {
        v.push_back(TopicSpec{explore_directive_topic_, std::type_index(typeid(ExplorationDirective)),
                              SubscriptionKind::Direct, /*required=*/false});
    }
    return v;
}

std::vector<TopicSpec> PremotorAI::output_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{policy_output_topic_, std::type_index(typeid(PolicyToken))},
    };
    if (multi_enabled_) {
        // Phase 6.18 — N-channel publish.  One TopicSpec per configured
        // output topic.  bilateral_enabled_ is guaranteed false here.
        for (auto const& t : output_topics_multi_) {
            v.push_back(TopicSpec{t, std::type_index(typeid(ActionOut))});
        }
    } else if (bilateral_enabled_) {
        v.push_back(TopicSpec{output_topic_left_,  std::type_index(typeid(ActionOut))});
        v.push_back(TopicSpec{output_topic_right_, std::type_index(typeid(ActionOut))});
    } else {
        v.push_back(TopicSpec{action_output_topic_, std::type_index(typeid(ActionOut))});
    }
    return v;
}

ParamSchema PremotorAI::params_schema() const {
    return {
        {"level",            ParamMutability::ConstructionOnly, "consensus.<level> to read",       ParamValue{int64_t{0}}},
        {"n_intents",        ParamMutability::ConstructionOnly, "Number of motor intents (≥2)",   ParamValue{int64_t{5}}},
        {"accel_min",        ParamMutability::ConstructionOnly, "Lowest intent accel",            ParamValue{-4.0}},
        {"accel_max",        ParamMutability::ConstructionOnly, "Highest intent accel",           ParamValue{4.0}},
        {"gain",             ParamMutability::HotMutable,       "Pre-softmax score multiplier",   ParamValue{1.0}},
        {"learning_rate",    ParamMutability::HotMutable,       "Hebbian update step size",       ParamValue{0.01}},
        {"temperature_base", ParamMutability::HotMutable,       "Softmax temperature baseline",   ParamValue{1.0}},
        {"temperature_da_gain", ParamMutability::HotMutable,    "DA modulation: T = T_base/(1+DA*gain)", ParamValue{0.5}},
        {"sample_action",    ParamMutability::HotMutable,       "true=sample chosen intent; false=argmax", ParamValue{true}},
        {"use_weighted_accel", ParamMutability::HotMutable,     "true=emit Σ p_i·a_i; false=emit a[chosen]", ParamValue{true}},
        {"intent_dwell_ticks", ParamMutability::HotMutable,     "Phase 8/P1: minimum ticks to hold an ordinary sampled/argmax intent before accepting small flips.  0=off.  Hard chunk/explore overrides bypass the dwell.", ParamValue{int64_t{0}}},
        {"intent_dwell_break_bias", ParamMutability::HotMutable, "Phase 8/P1: allow early dwell break only when p[new] >= p[held] + this margin.  Default 0.0.  Recommended sweep 0.10-0.25 with dwell 4/8/12.", ParamValue{0.0}},
        {"phase_context_topic", ParamMutability::ConstructionOnly, "Phase-viscosity P2: ProprioToken topic carrying [cos(phi), sin(phi)] CPG phase for this Premotor. Empty = disabled.", ParamValue{std::string("")}},
        {"phase_bins", ParamMutability::HotMutable, "Phase-viscosity P2: number of CPG phase bins per cycle. 0 disables phase commitment. Recommended first probe: 8.", ParamValue{int64_t{0}}},
        {"phase_commit_mode", ParamMutability::HotMutable, "Phase-viscosity P2: none | switch_penalty | bin_boundary. switch_penalty subtracts phase_switch_penalty from non-current intents inside the same phase bin; bin_boundary hard-holds until bin changes.", ParamValue{std::string("none")}},
        {"phase_switch_penalty", ParamMutability::HotMutable, "Phase-viscosity P2: pre-softmax logit penalty for switching intents within the same CPG phase bin. 0 = no penalty.", ParamValue{0.0}},
        {"bucket_context_topic", ParamMutability::ConstructionOnly, "2026-05-29 gait-bucket bet: ProprioToken topic carrying this leg's current discrete bucket (single float, 0..n_buckets-1).  Typical: 'bucket.fl' from CruseCoordinator publish_bucket_signals.  Empty = disabled.", ParamValue{std::string("")}},
        {"n_buckets",           ParamMutability::ConstructionOnly, "2026-05-29 gait-bucket bet: number of discrete buckets (e.g. 2 for swing/stance).  When >0 AND bucket_context_topic set, allocate bucket_bias_(n_buckets, n_intents) and add bucket_bias.row(current_bucket) to logits each tick; REINFORCE updates bucket_bias per step.  0 = disabled.", ParamValue{int64_t{0}}},
        {"bucket_bias_lr",      ParamMutability::HotMutable, "Separate learning rate for bucket_bias REINFORCE updates.  Default 0 = use mc_lr (legacy bucket v1 behavior; produced bucket_bias norms 6-9 that overpowered W·context).  Set lower (e.g. 0.005, 10× under mc_lr=0.05) to keep bucket_bias on the same scale as W contributions so per-bucket bias composes with compass/heading-driven steering instead of dominating it.", ParamValue{0.0}},
        {"bucket_bias_init_alt", ParamMutability::ConstructionOnly, "V8 Fix 3 — anti-symmetric bucket_bias init magnitude.  When > 0, initialize bucket_bias so bucket 0 favors intent 0 (and disfavors intent n-1), and bucket 1 vice-versa: bias[0,0]=+init, bias[0,last]=-init, bias[1,0]=-init, bias[1,last]=+init.  Breaks the symmetry that leaves hip1 (the locomotion axis) unused, by entering every Premotor with a bimodal preference between extreme intents per bucket.  REINFORCE refines which extreme corresponds to which bucket-role per leg.  Recommended ~0.5 (smaller = washed out faster; larger = dominates longer).  Default 0 = no prior.", ParamValue{0.0}},
        {"leg_phase_input_topic", ParamMutability::ConstructionOnly, "2026-06-04 Phase A2 — per-leg phase latent augmentation. Topic publishing ProprioToken with [cos(φ_leg), sin(φ_leg)] (typically rhythm.cpg.<leg>). When non-empty AND leg_phase_gain>0, PremotorAI allocates a separate (n_intents × 2) W_leg matrix and adds leg_phase_gain*(W_leg*[cos,sin]) to scores each tick. W_leg trains under MC REINFORCE alongside W_ on the per-step phase. Genuinely differentiates per-Premotor logits when each Premotor reads its own leg's phase. Empty = disabled.", ParamValue{std::string("")}},
        {"leg_phase_gain", ParamMutability::HotMutable, "2026-06-04 Phase A2 — magnitude of the W_leg contribution to scores. 0 = off (default, W_leg never allocated, bit-identical legacy). Recommended 0.5-1.0 (same scale as bucket_bias_init_alt) to give per-leg phase a comparable logit budget.", ParamValue{0.0}},
        {"leg_phase_lr", ParamMutability::HotMutable, "2026-06-04 Phase A2 — learning rate for W_leg under MC REINFORCE. Default 0.01 (10× under mc_lr=0.05 to keep W_leg on the same logit scale as W contributions).", ParamValue{0.01}},
        {"eligibility_lambda", ParamMutability::HotMutable,     "0=one-step credit; 0..1 traces decay rate (Williams REINFORCE-with-eligibility)", ParamValue{0.0}},
        {"drive_reward_gain", ParamMutability::HotMutable,      "0=off; >0 = continuous reward from drive urgency drops (sparse-reward envs)", ParamValue{0.0}},
        {"epistemic_gain",   ParamMutability::HotMutable,       "0=off; >0 adds (1-visit_ema_i)*gain to score before softmax (directed exploration)", ParamValue{0.0}},
        {"epistemic_alpha",  ParamMutability::HotMutable,       "Visit-EMA decay rate (smaller = longer memory)", ParamValue{0.05}},
        {"baseline_lr",      ParamMutability::HotMutable,       "0=off; >0 enables V(s) baseline (advantage-actor-critic Hebbian)", ParamValue{0.0}},
        {"value_gamma",      ParamMutability::HotMutable,       "Discount factor for TD-error δ = r + γV(s')−V(s)", ParamValue{0.95}},
        {"target_tau",       ParamMutability::HotMutable,       "EMA rate for target V head (smaller = slower; decouples V(s') from V(s) per Phase 6.5.36 fix)", ParamValue{0.005}},
        {"value_head_gain",      ParamMutability::HotMutable,   "Phase 7.8: weight on per-intent predictive value head V[i,:]·latent added to softmax logits.  0=off (default, byte-identical legacy).  >0 makes intent selection FORWARD-LOOKING: bias toward intents with high predicted future reward.  Recommended 0.5-2.0.", ParamValue{0.0}},
        {"value_head_lr",        ParamMutability::HotMutable,   "Phase 7.8: learning rate for V[chosen] TD update when lookahead window closes.  Default 0.01.", ParamValue{0.01}},
        {"value_head_lookahead",  ParamMutability::HotMutable,  "Phase 7.8: lookahead window in ticks for reward accumulation against V prediction.  Default 30 (≈0.5s at 60Hz).", ParamValue{int64_t{30}}},
        {"rhythm_bias_topic",     ParamMutability::ConstructionOnly, "Phase 7.9: subscribe to RhythmBiasToken on this topic (typically rhythm.bias.<premotor_id>).  When non-empty, adds the received bias vector to softmax scores pre-decision.  Empty = disabled (default).", ParamValue{std::string("")}},
        {"entropy_anneal_gain",      ParamMutability::HotMutable,    "Phase 7.10: adaptive entropy anneal.  When entropy_ema exceeds entropy_anneal_threshold (Premotor stuck near uniform), softmax T is multiplied by (1 - gain × stuck_factor) so the distribution sharpens and the Premotor commits.  Self-correcting: committed Premotors (low entropy) get unaffected T.  0 = off (default, byte-identical legacy).  Recommended 0.4-0.6.", ParamValue{0.0}},
        {"entropy_anneal_threshold", ParamMutability::HotMutable,    "Phase 7.10: entropy_ema above which Premotor counts as 'stuck'.  Default 1.2 (for n_intents=5, max entropy = log(5) ≈ 1.61; threshold catches policies still near uniform but lets converged-uniform-by-design ones below threshold escape).", ParamValue{1.2}},
        {"entropy_anneal_alpha",     ParamMutability::HotMutable,    "Phase 7.10: EMA rate for entropy_ema update.  Default 0.005 ≈ 200-tick window ≈ 3 sec at 60Hz.", ParamValue{0.005}},
        {"alpha_topic",      ParamMutability::ConstructionOnly, "Phase 6.6.F: MotorFader α telemetry topic (FaderState)", ParamValue{std::string(topics::kMotorFaderAlpha)}},
        {"update_alpha_threshold", ParamMutability::HotMutable, "Phase 6.6.F: skip Hebbian update when MotorFader α < this (off-policy guard). 0.0 = off (always update)", ParamValue{0.0}},
        {"action_output_topic", ParamMutability::ConstructionOnly, "Phase 6.6.F: where to publish ActionOut (default action.out; set to action.brain to route through MotorFader)", ParamValue{std::string(topics::kActionOut)}},
        {"intent_override_topic", ParamMutability::ConstructionOnly, "v5.3 Phase B: subscribe to this topic for IntentToken overrides (chunk replay).  Empty = disabled (default).", ParamValue{std::string("")}},
        {"policy_output_topic",   ParamMutability::ConstructionOnly, "Phase 7.2-EPM: per-instance PolicyToken publish topic.  Default 'policy.intent' preserves legacy single-policy behaviour; set to e.g. 'policy.intent.fl_hip1' so per-leg MotorRepertoires can subscribe to their leg's joint policies independently.", ParamValue{std::string(topics::kPolicyIntent)}},
        {"intent_channel",        ParamMutability::ConstructionOnly, "Phase 7.2-EPM: index into IntentToken.indices for multi-channel chunk overrides.  -1 = legacy (use IntentToken.index).  0/1/2 = read indices[N] when ActionDecoder publishes a vector intent for a multi-channel chunk.", ParamValue{int64_t{-1}}},
        {"explore_directive_topic", ParamMutability::ConstructionOnly, "Phase 6.7: subscribe to this topic for ExplorationDirective from HomeokineticExploration.  When directive.active, the intent index whose intent_accels_[i] is nearest directive.accel overrides the softmax sample (lower priority than intent_override_topic chunk dispatches).  Empty = disabled (default).  Set to 'exploration.directive' to consume the standard HomeokineticExploration output.", ParamValue{std::string("")}},
        {"aligned_event_name",    ParamMutability::ConstructionOnly, "v5.3 Phase C: when an event with this name arrives, treat it as +intensity * aligned_reward_gain in the credit-assignment path.  Empty = disabled (legacy).  Used for handtuned reward scaffolding so EventConjunction firings drive Premotor's REINFORCE just like hits do.", ParamValue{std::string("")}},
        {"aligned_reward_gain",   ParamMutability::HotMutable,       "v5.3 Phase C: multiplier applied to aligned_event intensity (default 1.0).", ParamValue{1.0}},
        {"pathway_temp_gain",  ParamMutability::HotMutable,       "Phase 6.6.I: scale of predicted-pathway familiarity → softmax temperature modulation. T = T_base * (1 + this * familiarity) / (1 + DA * t_da_gain).  0 = off, default. Familiar predicted trajectory → higher T → exploration of alternatives.", ParamValue{0.0}},
        {"output_noise_amplitude", ParamMutability::HotMutable,   "Phase 6.6.L: per-tick uniform(-amp, +amp) added independently to each published side's accel.  Gives the brain's bilateral output the same per-tick coordination jitter the reflex's wander noise provides — without it, the softmax-weighted mix is deterministic given the latent and joint-spike timing locks into a single phase.  Recommended ~0.4 on the default 4.0 accel scale.  0 = off (default).", ParamValue{0.0}},
        {"lr_bc",            ParamMutability::HotMutable,         "Phase 6.6.O: behavioral-cloning learning rate.  When > 0, Premotor finds the intent whose accel(s) best match the observed action.reflex.* stream and updates that row of W_ toward the current latent.  Update is α-weighted by (1-α) when bc_alpha_weighting=true, so high-α (brain-led) periods don't BC; low-α (reflex-led) periods imitate the reflex demonstration.  Default 0 = off (preserves pre-6.6.O behaviour bit-for-bit).", ParamValue{0.0}},
        {"bc_alpha_weighting", ParamMutability::HotMutable,       "Phase 6.6.O: when true (default), BC update gates by (1 - last_alpha) so BC only fires when reflex was driving.  Ablation: false → flat update regardless of α.", ParamValue{true}},
        {"bc_weight_decay",   ParamMutability::HotMutable,        "Phase 6.6.O: optional multiplicative shrinkage applied to W_ on each BC update tick (per-row, only the matched row).  Bounds long-run growth; default 0 = no homeostasis.", ParamValue{0.0}},
        {"bc_reflex_topic",      ParamMutability::ConstructionOnly, "Phase 6.6.O: unilateral reflex demonstration topic (used when Premotor is in single-channel mode).", ParamValue{std::string("action.reflex")}},
        {"bc_reflex_left_topic", ParamMutability::ConstructionOnly, "Phase 6.6.O: bilateral left reflex demonstration topic.", ParamValue{std::string("action.reflex.left")}},
        {"bc_reflex_right_topic",ParamMutability::ConstructionOnly, "Phase 6.6.O: bilateral right reflex demonstration topic.", ParamValue{std::string("action.reflex.right")}},
        {"state_visit_alpha",  ParamMutability::HotMutable,       "Phase 6.6.I: per-modality state visit-EMA decay rate (smaller = longer memory)", ParamValue{0.05}},
        {"output_topic_left",  ParamMutability::ConstructionOnly, "Phase 6.6.G: bilateral mode left-side ActionOut topic (empty = bilateral disabled)",  ParamValue{std::string("")}},
        {"output_topic_right", ParamMutability::ConstructionOnly, "Phase 6.6.G: bilateral mode right-side ActionOut topic (empty = bilateral disabled)", ParamValue{std::string("")}},
        {"bilateral_table",    ParamMutability::ConstructionOnly, "Phase 6.6.G: JSON [[L0,R0],...] aligned with intents (empty + n_intents=5 = default table).  Phase 6.18: when output_topics is set (multi-channel mode), must be an n_intents × M matrix where M = output_topics.size().", ParamValue{std::string("")}},
        {"output_topics",      ParamMutability::ConstructionOnly, "Phase 6.18: JSON array of M ActionOut topic strings for N-channel output (e.g. [\"action.fl_hip1\",\"action.fl_hip2\",\"action.fl_knee\"]).  When non-empty, Premotor publishes M ActionOut messages per tick using per-intent values from bilateral_table.  Overrides output_topic_left/right.  Designed for per-leg variant where one Premotor co-learns 3 servo channels.", ParamValue{std::string("")}},
        {"mc_lr",                ParamMutability::HotMutable,     "Phase v5.1 Monte-Carlo actor-critic learning rate.  When > 0, defers per-event Hebbian updates from apply_reward to per-episode finalize_mc_episode.  Per tick, append (latent, distribution, chosen, accumulated_reward) to mc_trajectory_; on events.episode_end compute G_t = r_t + γ G_{t+1} backwards, optionally normalise via running mean/std over advantage_window_ episodes, then apply Hebbian-shaped update W += mc_lr * advantage_t * latent_t * distribution_t per step.  Default 0 → MC mode off; bit-identical to v4 Premotor under all v4 configs.", ParamValue{0.0}},
        {"mc_gamma",             ParamMutability::HotMutable,     "Phase v5.1: discount factor for Monte-Carlo return computation.  Default 0.99.", ParamValue{0.99}},
        {"advantage_normalization", ParamMutability::HotMutable,  "Phase v5.1: when true, normalise per-step return by running (mean, std) of recent episode returns before applying the Hebbian-shaped update.  Eliminates the long-run weight divergence identified at the Phase 6.5.35 ceiling (Hebbian-without-baseline drifts unboundedly under sustained dense reward).  Default false (legacy unbounded growth path; rely on bc_weight_decay or external shrinkage if needed).", ParamValue{false}},
        {"advantage_window",     ParamMutability::HotMutable,     "Phase v5.1: size of rolling buffer of recent episode returns used for advantage normalisation.  Default 100.", ParamValue{int64_t{100}}},
        {"mc_episode_topic",     ParamMutability::ConstructionOnly, "Phase v5.1: bus topic that signals episode boundary; on receipt, finalize_mc_episode is invoked.  Default events.episode_end.", ParamValue{std::string("events.episode_end")}},
        {"mc_reinforce",         ParamMutability::HotMutable,     "Phase v5.1: when true, MC's per-step update is the score-function REINFORCE form W[i] += scale·advantage·(𝟙{i=chosen}−p_i)·latent — chosen intent gets positive credit ∝ (1−p_chosen), non-chosen get negative credit ∝ p_i.  Naturally breaks symmetry from a uniform prior (no BC required).  Default false = legacy Hebbian-distribution-weighted form (structurally symmetric; needs BC bootstrap).  Phase 6.5.33 tested REINFORCE-without-baseline and saw regression; v5.1 pairs it with within-episode advantage normalisation, which should bound the variance.", ParamValue{false}},
        {"master_seed",      ParamMutability::ConstructionOnly, "RNG seed",                       ParamValue{int64_t{0}}},
    };
}

// v5.4.M Diagnostic B — Shannon entropy of the CHOSEN intent histogram
// over the rolling window.  Low entropy = Premotor stuck on one intent
// (rut signal — playful-machine principle 1 says this is when to
// inject exploration).  High entropy = exploring across multiple
// intents.  Returns 0 if window is empty.
float PremotorAI::chosen_window_entropy() const {
    if (chosen_window_.empty()) return 0.0f;
    int N = int(intent_accels_.size());
    if (N <= 0) return 0.0f;
    std::vector<int> counts(N, 0);
    for (int idx : chosen_window_) {
        if (idx >= 0 && idx < N) ++counts[idx];
    }
    int total = int(chosen_window_.size());
    float H = 0.0f;
    for (int n : counts) {
        if (n <= 0) continue;
        float p = float(n) / float(total);
        H -= p * std::log(p);
    }
    return H;   // in nats; max = ln(N) when uniform
}

void PremotorAI::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("Premotor requires a non-null Bus");

    apply_param(params, "level",                [&](auto const& v){ level_                = int(get_int(v, "level")); });
    int n_intents = 5;
    apply_param(params, "n_intents",            [&](auto const& v){ n_intents             = std::max(2, int(get_int(v, "n_intents"))); });
    apply_param(params, "accel_min",            [&](auto const& v){ accel_min_            = float(get_double(v, "accel_min")); });
    apply_param(params, "accel_max",            [&](auto const& v){ accel_max_            = float(get_double(v, "accel_max")); });
    apply_param(params, "gain",                 [&](auto const& v){ gain_                 = float(get_double(v, "gain")); });
    apply_param(params, "learning_rate",        [&](auto const& v){ lr_                   = float(get_double(v, "learning_rate")); });
    apply_param(params, "temperature_base",     [&](auto const& v){ t_base_               = std::max(0.01f, float(get_double(v, "temperature_base"))); });
    apply_param(params, "temperature_da_gain",  [&](auto const& v){ t_da_gain_            = float(get_double(v, "temperature_da_gain")); });
    apply_param(params, "sample_action",        [&](auto const& v){ sample_action_        = get_bool(v, "sample_action"); });
    apply_param(params, "use_weighted_accel",   [&](auto const& v){ use_weighted_accel_   = get_bool(v, "use_weighted_accel"); });
    apply_param(params, "intent_dwell_ticks",   [&](auto const& v){ intent_dwell_ticks_   = std::max(0, int(get_int(v, "intent_dwell_ticks"))); });
    apply_param(params, "intent_dwell_break_bias", [&](auto const& v){ intent_dwell_break_bias_ = std::max(0.0f, float(get_double(v, "intent_dwell_break_bias"))); });
    apply_param(params, "phase_context_topic", [&](auto const& v){ phase_context_topic_ = get_string(v, "phase_context_topic"); });
    apply_param(params, "phase_bins", [&](auto const& v){ phase_bins_ = std::max(0, int(get_int(v, "phase_bins"))); });
    apply_param(params, "phase_commit_mode", [&](auto const& v){ phase_commit_mode_ = get_string(v, "phase_commit_mode"); });
    validate_phase_commit_mode(phase_commit_mode_);
    apply_param(params, "phase_switch_penalty", [&](auto const& v){ phase_switch_penalty_ = std::max(0.0f, float(get_double(v, "phase_switch_penalty"))); });
    apply_param(params, "bucket_context_topic", [&](auto const& v){ bucket_context_topic_ = get_string(v, "bucket_context_topic"); });
    apply_param(params, "n_buckets",            [&](auto const& v){ n_buckets_ = std::max(0, int(get_int(v, "n_buckets"))); });
    apply_param(params, "bucket_bias_lr",       [&](auto const& v){ bucket_bias_lr_ = std::max(0.0f, float(get_double(v, "bucket_bias_lr"))); });
    apply_param(params, "bucket_bias_init_alt", [&](auto const& v){ bucket_bias_init_alt_ = std::max(0.0f, float(get_double(v, "bucket_bias_init_alt"))); });
    // Phase A2 — per-leg phase latent augmentation params
    apply_param(params, "leg_phase_input_topic", [&](auto const& v){ leg_phase_input_topic_ = get_string(v, "leg_phase_input_topic"); });
    apply_param(params, "leg_phase_gain", [&](auto const& v){ leg_phase_gain_ = std::max(0.0f, float(get_double(v, "leg_phase_gain"))); });
    apply_param(params, "leg_phase_lr",   [&](auto const& v){ leg_phase_lr_   = std::max(0.0f, float(get_double(v, "leg_phase_lr"))); });
    if (leg_phase_gain_ > 0.0f && !leg_phase_input_topic_.empty()) {
        int n_intents_init = int(intent_accels_.size());
        if (n_intents_init > 0) {
            W_leg_ = Eigen::MatrixXf::Zero(n_intents_init, 2);
        }
    }
    if (n_buckets_ > 0 && !bucket_context_topic_.empty()) {
        bucket_bias_ = Eigen::MatrixXf::Zero(n_buckets_, n_intents);
        // V8 Fix 3 — anti-symmetric init: bucket b favors intent 0 when b is
        // even, intent (n_intents-1) when b is odd, and vice-versa for the
        // opposite extreme.  For n_buckets=2 (swing=0, stance=1): swing gets
        // +init on i=0 and -init on i=last; stance gets the opposite.
        if (bucket_bias_init_alt_ > 0.0f && n_intents >= 2) {
            int last = n_intents - 1;
            for (int b = 0; b < n_buckets_; ++b) {
                float sign = (b % 2 == 0) ? +1.0f : -1.0f;
                bucket_bias_(b, 0)    = +sign * bucket_bias_init_alt_;
                bucket_bias_(b, last) = -sign * bucket_bias_init_alt_;
            }
        }
    }
    apply_param(params, "eligibility_lambda",   [&](auto const& v){ eligibility_lambda_   = std::clamp(float(get_double(v, "eligibility_lambda")), 0.0f, 1.0f); });
    apply_param(params, "drive_reward_gain",    [&](auto const& v){ drive_reward_gain_    = float(get_double(v, "drive_reward_gain")); });
    apply_param(params, "epistemic_gain",       [&](auto const& v){ epistemic_gain_       = float(get_double(v, "epistemic_gain")); });
    apply_param(params, "epistemic_alpha",      [&](auto const& v){ epistemic_alpha_      = std::clamp(float(get_double(v, "epistemic_alpha")), 0.0f, 1.0f); });
    apply_param(params, "baseline_lr",          [&](auto const& v){ baseline_lr_          = float(get_double(v, "baseline_lr")); });
    apply_param(params, "value_gamma",          [&](auto const& v){ value_gamma_          = std::clamp(float(get_double(v, "value_gamma")), 0.0f, 1.0f); });
    apply_param(params, "target_tau",           [&](auto const& v){ target_tau_           = std::clamp(float(get_double(v, "target_tau")), 0.0f, 1.0f); });
    apply_param(params, "value_head_gain",      [&](auto const& v){ value_head_gain_      = std::max(0.0f, float(get_double(v, "value_head_gain"))); });
    apply_param(params, "value_head_lr",        [&](auto const& v){ value_head_lr_        = std::max(0.0f, float(get_double(v, "value_head_lr"))); });
    apply_param(params, "value_head_lookahead", [&](auto const& v){ value_head_lookahead_ = std::max(1, int(get_int(v, "value_head_lookahead"))); });
    apply_param(params, "rhythm_bias_topic",    [&](auto const& v){ rhythm_bias_topic_    = get_string(v, "rhythm_bias_topic"); });
    apply_param(params, "entropy_anneal_gain",      [&](auto const& v){ entropy_anneal_gain_      = std::max(0.0f, float(get_double(v, "entropy_anneal_gain"))); });
    apply_param(params, "entropy_anneal_threshold", [&](auto const& v){ entropy_anneal_threshold_ = std::max(0.0f, float(get_double(v, "entropy_anneal_threshold"))); });
    apply_param(params, "entropy_anneal_alpha",     [&](auto const& v){ entropy_anneal_alpha_     = std::clamp(float(get_double(v, "entropy_anneal_alpha")), 0.0f, 1.0f); });
    apply_param(params, "alpha_topic",          [&](auto const& v){ alpha_topic_          = get_string(v, "alpha_topic"); });
    apply_param(params, "update_alpha_threshold", [&](auto const& v){ update_alpha_threshold_ = std::clamp(float(get_double(v, "update_alpha_threshold")), 0.0f, 1.0f); });
    apply_param(params, "action_output_topic",  [&](auto const& v){ action_output_topic_  = get_string(v, "action_output_topic"); });
    apply_param(params, "intent_override_topic",[&](auto const& v){ intent_override_topic_= get_string(v, "intent_override_topic"); });
    apply_param(params, "policy_output_topic",  [&](auto const& v){ policy_output_topic_  = get_string(v, "policy_output_topic"); });
    apply_param(params, "intent_channel",       [&](auto const& v){ intent_channel_       = int(get_int(v, "intent_channel")); });
    apply_param(params, "explore_directive_topic",[&](auto const& v){ explore_directive_topic_= get_string(v, "explore_directive_topic"); });
    apply_param(params, "aligned_event_name",   [&](auto const& v){ aligned_event_name_   = get_string(v, "aligned_event_name"); });
    apply_param(params, "aligned_reward_gain",  [&](auto const& v){ aligned_reward_gain_  = float(get_double(v, "aligned_reward_gain")); });
    apply_param(params, "pathway_temp_gain",    [&](auto const& v){ pathway_temp_gain_    = float(get_double(v, "pathway_temp_gain")); });
    apply_param(params, "state_visit_alpha",    [&](auto const& v){ state_visit_alpha_    = std::clamp(float(get_double(v, "state_visit_alpha")), 0.0f, 1.0f); });
    apply_param(params, "output_noise_amplitude", [&](auto const& v){ output_noise_amplitude_ = std::max(0.0f, float(get_double(v, "output_noise_amplitude"))); });
    apply_param(params, "lr_bc",                [&](auto const& v){ lr_bc_                = std::max(0.0f, float(get_double(v, "lr_bc"))); });
    apply_param(params, "bc_alpha_weighting",   [&](auto const& v){ bc_alpha_weighting_   = get_bool(v, "bc_alpha_weighting"); });
    apply_param(params, "bc_weight_decay",      [&](auto const& v){ bc_weight_decay_      = std::max(0.0f, float(get_double(v, "bc_weight_decay"))); });
    apply_param(params, "bc_reflex_topic",      [&](auto const& v){ bc_reflex_topic_      = get_string(v, "bc_reflex_topic"); });
    apply_param(params, "bc_reflex_left_topic", [&](auto const& v){ bc_reflex_left_topic_ = get_string(v, "bc_reflex_left_topic"); });
    apply_param(params, "bc_reflex_right_topic",[&](auto const& v){ bc_reflex_right_topic_= get_string(v, "bc_reflex_right_topic"); });
    apply_param(params, "output_topic_left",    [&](auto const& v){ output_topic_left_    = get_string(v, "output_topic_left"); });
    apply_param(params, "output_topic_right",   [&](auto const& v){ output_topic_right_   = get_string(v, "output_topic_right"); });
    apply_param(params, "bilateral_table",      [&](auto const& v){ bilateral_table_      = get_string(v, "bilateral_table"); });
    apply_param(params, "output_topics",        [&](auto const& v){ output_topics_param_  = get_string(v, "output_topics"); });
    apply_param(params, "mc_lr",                [&](auto const& v){ mc_lr_                = std::max(0.0f, float(get_double(v, "mc_lr"))); });
    apply_param(params, "mc_gamma",             [&](auto const& v){ mc_gamma_             = std::clamp(float(get_double(v, "mc_gamma")), 0.0f, 1.0f); });
    apply_param(params, "advantage_normalization", [&](auto const& v){ advantage_normalization_ = get_bool(v, "advantage_normalization"); });
    apply_param(params, "advantage_window",     [&](auto const& v){ advantage_window_     = std::max(int64_t{1}, get_int(v, "advantage_window")); });
    apply_param(params, "mc_episode_topic",     [&](auto const& v){ mc_episode_topic_     = get_string(v, "mc_episode_topic"); });
    apply_param(params, "mc_reinforce",         [&](auto const& v){ mc_reinforce_         = get_bool(v, "mc_reinforce"); });
    visit_ema_ = Eigen::VectorXf::Constant(n_intents, 1.0f / float(n_intents));
    apply_param(params, "master_seed",          [&](auto const& v){ master_seed_          = uint64_t(get_int(v, "master_seed")); });

    // Phase 6.6.G — bilateral output configuration.  Both topics must be
    // set for bilateral mode to engage; either one alone is a config
    // error.  When engaged we either parse the supplied JSON table or
    // fall back to the design-doc default for n_intents == 5.
    bilateral_enabled_ = !output_topic_left_.empty() && !output_topic_right_.empty();
    if (output_topic_left_.empty() != output_topic_right_.empty()) {
        throw std::invalid_argument(
            "Premotor: output_topic_left and output_topic_right must both be set (or both empty)");
    }
    if (bilateral_enabled_) {
        intent_accels_left_.assign(n_intents, 0.0f);
        intent_accels_right_.assign(n_intents, 0.0f);
        if (bilateral_table_.empty()) {
            if (n_intents != 5) {
                throw std::invalid_argument(
                    "Premotor: bilateral_output requires n_intents=5 when bilateral_table is empty "
                    "(supply an explicit JSON table for other sizes)");
            }
            // Design-doc default 5-intent table.
            const float L[5] = { +4.0f, +2.0f, +4.0f,  0.0f, -4.0f };
            const float R[5] = { -4.0f,  0.0f, +4.0f, +2.0f, +4.0f };
            for (int i = 0; i < 5; ++i) {
                intent_accels_left_[i]  = L[i];
                intent_accels_right_[i] = R[i];
            }
        } else {
            try {
                auto j = nlohmann::json::parse(bilateral_table_);
                if (!j.is_array() || int(j.size()) != n_intents) {
                    throw std::invalid_argument(
                        "Premotor: bilateral_table length must equal n_intents=" +
                        std::to_string(n_intents));
                }
                for (int i = 0; i < n_intents; ++i) {
                    auto const& row = j[i];
                    if (!row.is_array() || row.size() != 2) {
                        throw std::invalid_argument(
                            "Premotor: bilateral_table[" + std::to_string(i) +
                            "] must be a [left, right] pair");
                    }
                    intent_accels_left_[i]  = float(row[0].get<double>());
                    intent_accels_right_[i] = float(row[1].get<double>());
                }
            } catch (nlohmann::json::exception const& e) {
                throw std::invalid_argument(
                    std::string("Premotor: bilateral_table JSON parse failed — ") + e.what());
            }
        }
    }

    // Phase 6.18 — N-channel multi-output mode.  When output_topics is a
    // non-empty JSON array of M topic strings, parse:
    //   1. output_topics_multi_  (vector<string> of M topics)
    //   2. intent_accels_per_channel_  (n_intents × M matrix from
    //      bilateral_table_).  bilateral_table_ is REQUIRED here —
    //      multi-channel mode has no default coupling.
    // multi_enabled_ supersedes bilateral_enabled_'s 2-channel publish
    // path; BC bilateral plumbing stays disabled in this mode.
    if (!output_topics_param_.empty()) {
        try {
            auto j = nlohmann::json::parse(output_topics_param_);
            if (!j.is_array() || j.empty()) {
                throw std::invalid_argument(
                    "Premotor: output_topics must be a non-empty JSON array of strings");
            }
            for (auto const& el : j) {
                if (!el.is_string()) {
                    throw std::invalid_argument(
                        "Premotor: output_topics elements must be strings");
                }
                output_topics_multi_.push_back(el.get<std::string>());
            }
        } catch (nlohmann::json::exception const& e) {
            throw std::invalid_argument(
                std::string("Premotor: output_topics JSON parse failed — ") + e.what());
        }
    }
    multi_enabled_ = !output_topics_multi_.empty();
    if (multi_enabled_) {
        if (bilateral_enabled_) {
            throw std::invalid_argument(
                "Premotor: output_topics (multi-channel) and output_topic_left/right "
                "(bilateral) are mutually exclusive — pick one");
        }
        int M = int(output_topics_multi_.size());
        intent_accels_per_channel_.assign(n_intents, std::vector<float>(M, 0.0f));
        if (bilateral_table_.empty()) {
            throw std::invalid_argument(
                "Premotor: multi-channel output requires bilateral_table — "
                "no default coupling defined for N>2");
        }
        try {
            auto j = nlohmann::json::parse(bilateral_table_);
            if (!j.is_array() || int(j.size()) != n_intents) {
                throw std::invalid_argument(
                    "Premotor: bilateral_table must have n_intents=" +
                    std::to_string(n_intents) + " rows in multi-channel mode");
            }
            for (int i = 0; i < n_intents; ++i) {
                auto const& row = j[i];
                if (!row.is_array() || int(row.size()) != M) {
                    throw std::invalid_argument(
                        "Premotor: bilateral_table row " + std::to_string(i) +
                        " must have width = output_topics.size() = " +
                        std::to_string(M));
                }
                for (int c = 0; c < M; ++c) {
                    intent_accels_per_channel_[i][c] = float(row[c].get<double>());
                }
            }
        } catch (nlohmann::json::exception const& e) {
            throw std::invalid_argument(
                std::string("Premotor: bilateral_table parse (multi-channel) failed — ")
                + e.what());
        }
    }

    // Build the named-intent table.  Linearly spaced from accel_min to
    // accel_max, with cosmetic names so logs/diag are interpretable.
    intent_accels_.resize(n_intents);
    intent_names_.resize(n_intents);
    bc_intent_counts_.assign(n_intents, 0);
    chosen_intent_counts_.assign(n_intents, 0);
    bc_total_updates_ = 0;
    if (n_intents == 1) {
        intent_accels_[0] = 0.5f * (accel_min_ + accel_max_);
        intent_names_[0]  = "neutral";
    } else {
        float step = (accel_max_ - accel_min_) / float(n_intents - 1);
        for (int i = 0; i < n_intents; ++i) {
            intent_accels_[i] = accel_min_ + step * float(i);
            // Names: hard_left, slow_left, ..., neutral, ..., slow_right, hard_right.
            float frac = float(i) / float(n_intents - 1);   // 0..1
            if (frac < 0.25f)        intent_names_[i] = "hard_left";
            else if (frac < 0.45f)   intent_names_[i] = "slow_left";
            else if (frac <= 0.55f)  intent_names_[i] = "neutral";
            else if (frac <= 0.75f)  intent_names_[i] = "slow_right";
            else                      intent_names_[i] = "hard_right";
        }
    }

    rng_.seed(master_seed_ ? master_seed_ : 0xC0FFEEu);

    // Subscribe.
    std::string consensus_topic = std::string(topics::kConsensusPrefix) + std::to_string(level_);
    sub_ids_.push_back(bus_->subscribe(consensus_topic, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_consensus(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kDriveErrors, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_drive(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_neuro(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_event(t, p); }));

    // Phase 6.6.F — listen for MotorFader α to gate the Hebbian update.
    // Optional: when MotorFader is absent, last_alpha_ stays at its
    // initial 1.0 (full authority) and the gate is a no-op.
    sub_ids_.push_back(bus_->subscribe(alpha_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){
            if (auto fs = std::dynamic_pointer_cast<const FaderState>(p)) {
                last_alpha_ = fs->alpha;
            }
        }));

    // Phase 6.6.O — behavioral cloning subscriptions.  Wired
    // unconditionally (cheap; topics are tiny) so lr_bc can be
    // hot-toggled without requiring re-construction.  Bilateral mode
    // subscribes to both sides; unilateral subscribes to the single
    // reflex topic.
    if (bilateral_enabled_) {
        sub_ids_.push_back(bus_->subscribe(bc_reflex_left_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_bc_reflex_left(t, p); }));
        sub_ids_.push_back(bus_->subscribe(bc_reflex_right_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_bc_reflex_right(t, p); }));
    } else {
        sub_ids_.push_back(bus_->subscribe(bc_reflex_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_bc_reflex_uni(t, p); }));
    }
    // v5.3 Phase B — chunk-replay intent override.  Subscribe only when
    // configured; absence keeps pre-v5.3 behaviour exact.
    if (!intent_override_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(intent_override_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_intent_override(t, p); }));
    }
    // Phase 6.7 — HomeokineticExploration directive override.  Subscribe
    // only when configured; absence keeps pre-Phase-6.7 behaviour exact.
    if (!explore_directive_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(explore_directive_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_explore_directive(t, p); }));
    }
    if (!bucket_context_topic_.empty() && n_buckets_ > 0) {
        sub_ids_.push_back(bus_->subscribe(bucket_context_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*t*/, MessagePtr p){
                auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
                if (!pt || pt->values.size() < 1) return;
                int b = int(std::lround(pt->values(0)));
                current_bucket_ = std::clamp(b, 0, n_buckets_ - 1);
                bucket_seen_ = true;
            }));
    }
    if (!phase_context_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(phase_context_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_phase_context(t, p); }));
    }
    // Phase A2 — per-leg phase latent augmentation subscription.
    if (!leg_phase_input_topic_.empty() && leg_phase_gain_ > 0.0f) {
        sub_ids_.push_back(bus_->subscribe(leg_phase_input_topic_, SubscriptionKind::Direct,
            [this](std::string_view t, MessagePtr p){ handle_leg_phase(t, p); }));
    }

    // Phase 7.9 — SynergyTimer rhythm-bias subscription.  Only subscribe
    // when configured; absence keeps pre-7.9 behaviour exact.  Handler
    // latches latest_rhythm_bias_ for the next tick's softmax.
    if (!rhythm_bias_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(rhythm_bias_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){
                if (!input_allowed(p->producer_id)) return;
                auto rb = std::dynamic_pointer_cast<const RhythmBiasToken>(p);
                if (!rb) return;
                latest_rhythm_bias_ = rb->bias;
                rhythm_bias_seen_   = true;
            }));
    }
}

void PremotorAI::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "gain")                  gain_                = float(get_double(value, k));
    else if (k == "learning_rate")    lr_                  = float(get_double(value, k));
    else if (k == "temperature_base") t_base_              = std::max(0.01f, float(get_double(value, k)));
    else if (k == "temperature_da_gain") t_da_gain_        = float(get_double(value, k));
    else if (k == "sample_action")    sample_action_       = get_bool(value, k);
    else if (k == "use_weighted_accel") use_weighted_accel_ = get_bool(value, k);
    else if (k == "intent_dwell_ticks") intent_dwell_ticks_ = std::max(0, int(get_int(value, k)));
    else if (k == "intent_dwell_break_bias") intent_dwell_break_bias_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "phase_bins") phase_bins_ = std::max(0, int(get_int(value, k)));
    else if (k == "phase_commit_mode") {
        auto mode = get_string(value, k);
        validate_phase_commit_mode(mode);
        phase_commit_mode_ = mode;
    }
    else if (k == "phase_switch_penalty") phase_switch_penalty_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "eligibility_lambda") eligibility_lambda_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "drive_reward_gain") drive_reward_gain_ = float(get_double(value, k));
    else if (k == "epistemic_gain") epistemic_gain_ = float(get_double(value, k));
    else if (k == "epistemic_alpha") epistemic_alpha_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "baseline_lr") baseline_lr_ = float(get_double(value, k));
    else if (k == "value_gamma") value_gamma_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "target_tau") target_tau_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "value_head_gain") value_head_gain_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "value_head_lr") value_head_lr_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "value_head_lookahead") value_head_lookahead_ = std::max(1, int(get_int(value, k)));
    else if (k == "entropy_anneal_gain") entropy_anneal_gain_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "entropy_anneal_threshold") entropy_anneal_threshold_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "entropy_anneal_alpha") entropy_anneal_alpha_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "bucket_bias_lr")    bucket_bias_lr_    = std::max(0.0f, float(get_double(value, k)));
    else if (k == "update_alpha_threshold") update_alpha_threshold_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "pathway_temp_gain") pathway_temp_gain_ = float(get_double(value, k));
    else if (k == "state_visit_alpha") state_visit_alpha_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "output_noise_amplitude") output_noise_amplitude_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "lr_bc")             lr_bc_              = std::max(0.0f, float(get_double(value, k)));
    else if (k == "bc_alpha_weighting") bc_alpha_weighting_ = get_bool(value, k);
    else if (k == "bc_weight_decay")    bc_weight_decay_    = std::max(0.0f, float(get_double(value, k)));
    else if (k == "mc_lr")              mc_lr_              = std::max(0.0f, float(get_double(value, k)));
    else if (k == "mc_gamma")           mc_gamma_           = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "advantage_normalization") advantage_normalization_ = get_bool(value, k);
    else if (k == "advantage_window")   advantage_window_   = std::max(int64_t{1}, get_int(value, k));
    else if (k == "mc_reinforce")       mc_reinforce_       = get_bool(value, k);
    else if (k == "level" || k == "n_intents" || k == "accel_min"
          || k == "accel_max" || k == "master_seed" || k == "alpha_topic"
          || k == "action_output_topic" || k == "output_topic_left"
          || k == "output_topic_right" || k == "bilateral_table"
          || k == "bc_reflex_topic" || k == "bc_reflex_left_topic"
          || k == "bc_reflex_right_topic"
          || k == "mc_episode_topic" || k == "phase_context_topic"
          || k == "bucket_context_topic" || k == "n_buckets"
          || k == "bucket_bias_init_alt")
        throw std::invalid_argument("Premotor: param '" + k + "' is construction-only");
    else
        throw std::invalid_argument("Premotor: unknown param '" + k + "'");
}

void PremotorAI::handle_consensus(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!ct) return;
    // Late-init weights when we see the first real token (so we know
    // latent_dim).  Bootstrap tokens may have empty fused_embedding;
    // skip those for init.
    if (!weights_initialised_ && ct->fused_embedding.size() > 0) {
        latent_dim_ = int(ct->fused_embedding.size());
        int N = int(intent_accels_.size());
        // Small symmetric init so initial distribution is near-uniform.
        std::normal_distribution<float> nd(0.0f, 0.01f);
        W_.resize(N, latent_dim_);
        b_.resize(N);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < latent_dim_; ++j) W_(i, j) = nd(rng_);
            b_[i] = 0.0f;
        }
        E_ = Eigen::MatrixXf::Zero(N, latent_dim_);
        value_w_        = Eigen::VectorXf::Zero(latent_dim_);
        value_w_target_ = Eigen::VectorXf::Zero(latent_dim_);
        // Phase 7.8 — predictive value head V: zero init so value_head_gain
        // applied to V·latent contributes zero bias at cold start.  Brain
        // builds up value predictions as it observes (chosen, reward) pairs.
        V_ = Eigen::MatrixXf::Zero(N, latent_dim_);
        value_traj_.clear();
        weights_initialised_ = true;
    }
    // Cache the latent for the upcoming tick().  Reset to zero-length on
    // bootstrap/no-fusion ticks so we know to skip.
    last_latent_ = ct->fused_embedding;

    // Phase 6.6.I — rollout-aware exploration bookkeeping.  Walk the
    // ConsensusToken's per-modality winner ids and bump the state-visit
    // EMA for each.  Decay only the seen modalities — others' EMAs are
    // left to age implicitly via not being incremented (visit-EMA
    // semantics: 0 → 1 toward seen states).  Cache the predicted
    // pathways for tick() to consume.  Both maps are populated
    // unconditionally by LateralVoter so this runs even when
    // pathway_temp_gain_ is 0 (cheap, keeps state ready if the gain is
    // hot-toggled on).
    if (!ct->winner_ids_by_modality.empty()) {
        for (auto const& [topic, wid] : ct->winner_ids_by_modality) {
            if (wid < 0) continue;
            float& ema = state_visit_ema_[topic][wid];
            ema = (1.0f - state_visit_alpha_) * ema + state_visit_alpha_ * 1.0f;
        }
    }
    last_predicted_pathways_ = ct->predicted_pathways;
}

void PremotorAI::handle_drive(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto d = std::dynamic_pointer_cast<const DriveErrors>(payload);
    if (!d) return;
    float new_urgency = float(d->urgency);
    // Phase 6.5.30 — synthetic continuous reward from urgency change.
    // Drop in urgency = good (positive reward); rise = bad.  Combined
    // with eligibility traces (lambda > 0) this gives Premotor a
    // continuous gradient signal in sparse-reward envs.  Skip the very
    // first sample (no baseline yet) and only fire on meaningful deltas.
    if (drive_reward_gain_ > 0.0f && have_prev_urgency_) {
        float delta = prev_urgency_ - new_urgency;
        if (std::abs(delta) > 1e-5f) {
            apply_reward(delta * drive_reward_gain_);
        }
    }
    prev_urgency_       = new_urgency;
    have_prev_urgency_  = true;
    urgency_            = new_urgency;
}

void PremotorAI::handle_neuro(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto n = std::dynamic_pointer_cast<const NeuroState>(payload);
    if (!n) return;
    dopamine_ = float(n->dopamine);
}

void PremotorAI::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;

    std::string name(topic.substr(std::string(topics::kEventsPrefix).size()));

    // Phase v5.1 — episode_end signal.  When mc_lr > 0, finalise the
    // trajectory: walk backwards computing G_t = r_t + γ G_{t+1},
    // optionally normalise via running stats, apply Hebbian-shaped
    // update per step.  Otherwise (legacy mode), ignore.
    if (name == "episode_end") {
        if (mc_lr_ > 0.0f) finalize_mc_episode();
        return;
    }

    // Convert event names to signed valence:
    //   "hit"           → +intensity   (reward)
    //   "miss"          → -intensity
    //   "wall_stuck"    → -intensity
    //   any other event → ignored (no credit assignment)
    float r = 0.0f;
    if (name == "hit")              r =  e->intensity;
    else if (name == "miss"
          || name == "wall_stuck"
          || name == "failed")      r = -e->intensity;
    else if (!aligned_event_name_.empty() && name == aligned_event_name_) {
        // v5.3 Phase C — handtuned-scaffold teaching signal.
        r = e->intensity * aligned_reward_gain_;
        ++aligned_rewards_seen_;
    }
    else                            return;

    // Phase 7.8 — accumulate reward into every open value-head trajectory
    // entry.  Fires regardless of mc_lr_ mode so the value head learns
    // from the same reward stream the policy gradient sees.  Positive r
    // contributes to predicted_reward target; negative r reduces it.
    if (value_head_gain_ > 0.0f) {
        for (auto& step : value_traj_) {
            step.accumulated_reward += r;
        }
    }

    // Phase v5.1 — when MC mode is active, defer credit to episode end.
    // Reward signal is accumulated into the current tick's bucket; the
    // tick handler appends it to the trajectory.
    if (mc_lr_ > 0.0f) {
        accumulated_reward_this_tick_ += r;
        return;
    }

    apply_reward(r);
}

void PremotorAI::handle_bc_reflex_left(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    last_reflex_left_      = float(a->accel);
    last_reflex_left_tick_ = int64_t(a->tick_id);
}

void PremotorAI::handle_bc_reflex_right(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    last_reflex_right_      = float(a->accel);
    last_reflex_right_tick_ = int64_t(a->tick_id);
}

void PremotorAI::handle_bc_reflex_uni(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    last_reflex_uni_      = float(a->accel);
    last_reflex_uni_tick_ = int64_t(a->tick_id);
}

// v5.3 Phase B — intent override handler.  Stores the override for the
// upcoming tick(); checked by tick() right before the softmax sample.
// We also use tick_id matching so a stale override (from a previous tick
// whose tick() already ran) doesn't leak into the next tick's selection.
void PremotorAI::handle_intent_override(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto t = std::dynamic_pointer_cast<const IntentToken>(payload);
    if (!t) return;
    // Phase 7.2-EPM: multi-channel chunk override.  When intent_channel_ >= 0
    // AND the publisher populated indices[], read this Premotor's assigned
    // slice.  Otherwise fall back to the legacy single-channel index field.
    int idx;
    if (intent_channel_ >= 0 && intent_channel_ < int(t->indices.size())) {
        idx = t->indices[intent_channel_];
    } else {
        idx = t->index;
    }
    if (idx < 0) return;   // sentinel "no override"; ignore
    pending_override_idx_  = idx;
    pending_override_tick_ = t->tick_id;
}

// Phase 6.7 — HomeokineticExploration directive handler.  Stores the
// latest directive's active/accel/tick; tick() consults it after the
// intent-override check.  HomeokineticExploration publishes every tick
// (active=false when idle, active=true during an exploration episode),
// so unlike intent_override we DO NOT consume-once — every tick reads
// the latest state.
void PremotorAI::handle_explore_directive(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto d = std::dynamic_pointer_cast<const ExplorationDirective>(payload);
    if (!d) return;
    last_explore_active_ = d->active;
    last_explore_accel_  = d->accel;
    last_explore_tick_   = d->tick_id;
}

void PremotorAI::handle_leg_phase(std::string_view /*topic*/, MessagePtr payload) {
    // Phase A2 — latch per-leg phase [cos(φ), sin(φ)] for the next tick's
    // score block. Same payload contract as phase_context_topic but used
    // as additive W_leg input rather than for bin commitment.
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || p->values.size() < 2) return;
    leg_phase_s_[0] = float(p->values(0));
    leg_phase_s_[1] = float(p->values(1));
    leg_phase_seen_ = true;
}

void PremotorAI::handle_phase_context(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || p->values.size() < 2) return;

    constexpr float TWO_PI = 6.28318530717958647692f;
    float phi = std::atan2(float(p->values(1)), float(p->values(0)));
    if (phi < 0.0f) phi += TWO_PI;
    phase_value_ = phi;

    int next_bin = -1;
    if (phase_bins_ > 0) {
        next_bin = std::clamp(int(std::floor((phi / TWO_PI) * float(phase_bins_))),
                              0, phase_bins_ - 1);
    }
    if (phase_context_seen_ && next_bin >= 0 && phase_bin_ >= 0 && next_bin != phase_bin_) {
        ++phase_bin_changes_;
    }
    phase_bin_ = next_bin;
    phase_context_seen_ = true;
}

void PremotorAI::apply_bc_update(uint64_t tick_id) {
    // Phase 6.6.O — reflex imitation gradient.  No-op when lr_bc=0.
    if (lr_bc_ <= 0.0f) return;
    if (!weights_initialised_) return;
    if (last_latent_.size() != latent_dim_) return;
    int N = int(intent_accels_.size());
    if (N <= 0) return;

    int   matched = -1;
    float best    = std::numeric_limits<float>::infinity();
    // 6.6.O.1 fix: scheduler ordering puts Premotor's tick() before the
    // reflex modules's tick() each scheduler step, so the reflex's tick
    // stamp lags by one tick.  Strict tick equality made BC silent on
    // every diagnostic seed.  Relax to "we have ever seen reflex" — the
    // cached value persists across ticks and represents the most-recent
    // reflex command.  Fairer gate: require both sides for bilateral.
    if (bilateral_enabled_) {
        if (last_reflex_left_tick_  < 0) return;
        if (last_reflex_right_tick_ < 0) return;
        float Lobs = std::clamp(last_reflex_left_,  accel_min_, accel_max_);
        float Robs = std::clamp(last_reflex_right_, accel_min_, accel_max_);
        for (int i = 0; i < N; ++i) {
            float dL = intent_accels_left_[i]  - Lobs;
            float dR = intent_accels_right_[i] - Robs;
            float d2 = dL*dL + dR*dR;
            if (d2 < best) { best = d2; matched = i; }   // first-index tiebreak
        }
    } else {
        if (last_reflex_uni_tick_ < 0) return;
        float Aobs = std::clamp(last_reflex_uni_, accel_min_, accel_max_);
        for (int i = 0; i < N; ++i) {
            float d = intent_accels_[i] - Aobs;
            float d2 = d*d;
            if (d2 < best) { best = d2; matched = i; }
        }
    }
    if (matched < 0) return;
    last_bc_intent_ = matched;

    float gate = bc_alpha_weighting_ ? std::max(0.0f, 1.0f - last_alpha_) : 1.0f;
    if (gate <= 0.0f) return;
    if (matched >= 0 && matched < int(bc_intent_counts_.size())) {
        bc_intent_counts_[matched] += 1;
    }
    bc_total_updates_ += 1;

    // Optional homeostasis: shrink the matched row before adding the
    // new credit, bounding long-run growth.
    if (bc_weight_decay_ > 0.0f) {
        W_.row(matched) *= (1.0f - bc_weight_decay_);
    }
    W_.row(matched).noalias() += (lr_bc_ * gate) * last_latent_.transpose();
}

void PremotorAI::apply_reward(float intensity) {
    if (!weights_initialised_) return;
    // Phase 6.6.F — off-policy contamination guard.  When a MotorFader
    // is in the graph and Premotor's intent had less than this fraction
    // of motor authority on the recent action, skip the Hebbian update
    // — the reward outcome is more attributable to reflexes than to
    // Premotor's policy.  Threshold == 0 disables the gate (legacy
    // behavior preserved).
    if (update_alpha_threshold_ > 0.0f && last_alpha_ < update_alpha_threshold_) {
        return;
    }
    int N = int(intent_accels_.size());
    if (eligibility_lambda_ > 0.0f) {
        // Williams REINFORCE-with-eligibility-traces: credit the entire
        // accumulated history, decayed by lambda each tick.  Reward
        // arriving N ticks late still propagates to the policy/state
        // active N ticks ago.  Required for momentum-class problems.
        if (E_.rows() != N || E_.cols() != latent_dim_) return;
        W_.noalias() += (lr_ * intensity) * E_;
        return;
    }
    // Phase 6.5.36 — value-baseline TD error.  When baseline_lr_ > 0,
    // replace raw `intensity` with the TD error δ = r + γV(s') − V(s)
    // where V(s) = value_w_ · prev_latent and V(s') = value_w_ · latent.
    // Negative δ shrinks weights, positive δ grows them — bounding
    // long-training divergence (Phase 6.5.35 ceiling).  Then update
    // value_w_ toward target r + γV(s').
    float effective_intensity = intensity;
    if (baseline_lr_ > 0.0f
        && weights_initialised_
        && have_prev_value_
        && prev_value_latent_.size() == latent_dim_
        && last_latent_.size() == latent_dim_) {
        // Phase 6.5.37 — use target network for V(s'), online for V(s).
        // Decouples the bootstrap target from the value being learned,
        // breaking the V(s_curr) ≈ V(s_prev) collapse on smoothly-varying
        // continuous latents (Phase 6.5.36 finding).
        float v_curr_target = value_w_target_.dot(last_latent_);
        float td_error = intensity + value_gamma_ * v_curr_target - prev_value_;
        effective_intensity = td_error;
        // Update online V_w toward target r + γV_target(s').
        value_w_.noalias() += baseline_lr_ * td_error * prev_value_latent_;
        // Slow EMA: target tracks online with delay τ.
        if (target_tau_ > 0.0f) {
            value_w_target_.noalias() = (1.0f - target_tau_) * value_w_target_
                                      + target_tau_ * value_w_;
        }
    }

    // λ=0: original Premotor "Hebbian state→action × reward" credit.
    // Δw_i = lr · r · latent · p_i.  NOT strict REINFORCE — that
    // (𝟙{chosen} − p_i × latent · r) was tested in Phase 6.5.33 and
    // empirically REGRESSED on both Cell (37→32 hits) and CartPole
    // (45→9.4 mean reward).  The Hebbian co-activation × reward form
    // is what was working — it's robust to both sparse-reward and
    // dense-reward regimes because it grows weights smoothly in the
    // direction of "this latent + this distribution = good," whereas
    // unbiased REINFORCE-without-baseline is too high-variance for
    // short-failing episodes (CartPole) and too noisy for sparse-event
    // ones (Cell).  Keeping the Hebbian form by deliberate decision.
    if (last_latent_.size() != latent_dim_) return;
    if (last_distribution_.size() != N) return;
    float scale = lr_ * effective_intensity;
    for (int i = 0; i < N; ++i) {
        float p_i = last_distribution_[i];
        W_.row(i).noalias() += (scale * p_i) * last_latent_.transpose();
    }
    // Latch current latent as the "previous" state for next reward's TD.
    // Use ONLINE V_w (not target) for prev_value_, so the next tick's
    // δ has the right "online minus γ·target" structure.
    if (baseline_lr_ > 0.0f && last_latent_.size() == latent_dim_) {
        prev_value_latent_  = last_latent_;
        prev_value_         = value_w_.dot(last_latent_);
        have_prev_value_    = true;
    }
}

void PremotorAI::finalize_mc_episode() {
    // Phase v5.1 — Monte-Carlo actor-critic episode close.
    //
    // 1. Compute returns G_t backwards: G_T = r_T, G_t = r_t + γ G_{t+1}.
    // 2. Optionally normalise: advantage_t = (G_t - μ) / max(σ, ε), where
    //    (μ, σ) come from the rolling buffer of episode totals over the
    //    last `advantage_window_` episodes.  When the buffer is empty or
    //    advantage_normalization_=false, advantage_t = G_t (raw return).
    // 3. Apply Hebbian-shaped per-step update:
    //       W += mc_lr * advantage_t * latent_t * distribution_t
    //    Same form as apply_reward's λ=0 path but with the full-trajectory
    //    return instead of a single reward event.  Eliminates the bootstrap-
    //    correlation collapse identified in Phase 6.5.36/37 because no
    //    bootstrap V is involved.
    // 4. Push episode total return to recent_returns_ for the next
    //    normalisation window; drop oldest if window full.
    // 5. Clear trajectory + reset accumulator.
    if (mc_trajectory_.empty()) return;
    if (!weights_initialised_) {
        mc_trajectory_.clear();
        accumulated_reward_this_tick_ = 0.0f;
        return;
    }

    int  T = int(mc_trajectory_.size());
    int  N = int(intent_accels_.size());
    std::vector<float> G(T, 0.0f);
    float run_return = 0.0f;
    for (int t = T - 1; t >= 0; --t) {
        run_return = mc_trajectory_[t].reward + mc_gamma_ * run_return;
        G[t] = run_return;
    }
    float total_return = G[0];      // sum of γ-discounted rewards from t=0
    mc_last_return_ = total_return;

    // Within-episode advantage normalisation (standard PPO/A2C form):
    // subtract per-episode mean of G_t, divide by per-episode std.
    // Rolling cross-episode stats (recent_returns_) are kept for
    // diagnostics only — using episode-total μ/σ to normalise per-step
    // G_t produced unbounded advantages on late-trajectory steps because
    // G_T can be much smaller than G_0 in a long episode.
    float ep_mean = 0.0f;
    float ep_std  = 1.0f;
    if (advantage_normalization_) {
        double sum = 0.0, sq = 0.0;
        for (int t = 0; t < T; ++t) { sum += G[t]; sq += double(G[t]) * G[t]; }
        double n = double(T);
        double m = sum / n;
        double var = sq / n - m * m;
        ep_mean = float(m);
        ep_std  = float(std::sqrt(std::max(var, 1e-12)));
        if (ep_std < 1e-3f) ep_std = 1.0f;     // degenerate → no scaling
    }

    // Per-step weight update.  Two forms gated by mc_reinforce_:
    //
    //   FALSE (default, Hebbian-distribution):
    //     W[i] += scale · advantage · p_i · latent
    //   Every intent gets credit proportional to its policy probability.
    //   STRUCTURALLY SYMMETRIC: at uniform p (e.g. p_i = 1/N for all i),
    //   all rows receive the same gradient and W stays uniform forever.
    //   Requires an asymmetric prior (BC, custom init) to bootstrap a
    //   policy.  Empirical finding (Phase v5.1 BC-disabled trace,
    //   2026-05-09): with BC off, MC alone keeps W uniform across
    //   intents — no symmetry break, no policy learned.
    //
    //   TRUE (Phase v5.1 score-function REINFORCE):
    //     W[i] += scale · advantage · (𝟙{i = chosen} − p_i) · latent
    //   This is the standard policy-gradient score function.  Chosen
    //   intent gets POSITIVE update proportional to (1 − p_chosen);
    //   non-chosen intents get NEGATIVE update proportional to p_i.
    //   Naturally breaks symmetry: from uniform p, even one rewarded
    //   episode shifts mass toward the chosen intent at the expense of
    //   the others.  Phase 6.5.33 tested this without an advantage
    //   baseline and saw regression (variance dominated); v5.1 pairs
    //   it with within-episode advantage normalisation, which should
    //   bound the variance and let the gradient land cleanly.
    float scale = mc_lr_;
    for (int t = 0; t < T; ++t) {
        float advantage = advantage_normalization_
                              ? (G[t] - ep_mean) / ep_std
                              : G[t];
        Eigen::VectorXf const& latent = mc_trajectory_[t].latent;
        Eigen::VectorXf const& dist   = mc_trajectory_[t].distribution;
        int   chosen = mc_trajectory_[t].chosen;
        if (latent.size() != latent_dim_) continue;
        if (dist.size() != N) continue;
        if (mc_reinforce_) {
            for (int i = 0; i < N; ++i) {
                float indicator = (i == chosen) ? 1.0f : 0.0f;
                float gradient_i = indicator - dist[i];
                W_.row(i).noalias() += (scale * advantage * gradient_i) * latent.transpose();
            }
        } else {
            for (int i = 0; i < N; ++i) {
                float p_i = dist[i];
                W_.row(i).noalias() += (scale * advantage * p_i) * latent.transpose();
            }
        }
        // 2026-05-29 gait-bucket bet — apply the same REINFORCE update to the
        // bucket bias row for the bucket active at this step.  Uses the same
        // mc_lr scale; the bias is small (n_buckets × n_intents) so it grows
        // proportionally with W_.  Score-function form regardless of mc_reinforce_
        // (the bucket bias is purely a per-context-per-intent term, naturally
        // policy-gradient).
        int b_t = mc_trajectory_[t].bucket;
        if (b_t >= 0 && b_t < bucket_bias_.rows()
            && bucket_bias_.cols() == N) {
            // Use bucket_bias_lr_ when set (>0); else fall back to mc_lr.
            float bias_scale = (bucket_bias_lr_ > 0.0f) ? bucket_bias_lr_ : mc_lr_;
            for (int i = 0; i < N; ++i) {
                float indicator = (i == chosen) ? 1.0f : 0.0f;
                float gradient_i = indicator - dist[i];
                bucket_bias_(b_t, i) += bias_scale * advantage * gradient_i;
            }
        }

        // Phase A2 — same REINFORCE update form on W_leg, using the
        // per-step phase [cos, sin] as the "latent" for this contribution.
        if (leg_phase_gain_ > 0.0f
            && W_leg_.rows() == N && W_leg_.cols() == 2) {
            Eigen::Vector2f const& phase_s = mc_trajectory_[t].leg_phase_s;
            float leg_scale = (leg_phase_lr_ > 0.0f) ? leg_phase_lr_ : mc_lr_;
            for (int i = 0; i < N; ++i) {
                float indicator = (i == chosen) ? 1.0f : 0.0f;
                float gradient_i = indicator - dist[i];
                W_leg_.row(i).noalias() += (leg_scale * advantage * gradient_i) * phase_s.transpose();
            }
        }
    }

    // Diagnostic rolling stats over episode totals (G_0 per episode).
    recent_returns_.push_back(total_return);
    while (int(recent_returns_.size()) > advantage_window_)
        recent_returns_.pop_front();
    if (recent_returns_.size() >= 2) {
        double sum = 0.0, sq = 0.0;
        for (float v : recent_returns_) { sum += v; sq += double(v) * v; }
        double n = double(recent_returns_.size());
        double m = sum / n;
        double var = sq / n - m * m;
        mc_return_mean_ = float(m);
        mc_return_std_  = float(std::sqrt(std::max(var, 1e-12)));
    } else if (recent_returns_.size() == 1) {
        mc_return_mean_ = recent_returns_.front();
        mc_return_std_  = 1.0f;
    }

    mc_episodes_seen_ += 1;
    mc_trajectory_.clear();
    accumulated_reward_this_tick_ = 0.0f;
}

void PremotorAI::tick(uint64_t tick_id) {
    int N = int(intent_accels_.size());
    auto pol = std::make_shared<PolicyToken>();
    pol->tick_id          = tick_id;
    pol->producer_id      = id_.empty() ? std::string("premotor") : id_;
    pol->intent_names     = intent_names_;
    pol->intent_accels    = intent_accels_;
    pol->intent_distribution.setConstant(N, 1.0f / float(N));
    pol->weighted_accel   = 0.0f;
    pol->entropy          = std::log(float(N));   // uniform-prior entropy
    pol->temperature      = t_base_;
    pol->chosen_intent    = -1;

    if (!weights_initialised_ || last_latent_.size() != latent_dim_) {
        // No real latent seen yet → publish uniform / zero-action on
        // whichever channel(s) are configured.
        bus_->publish(policy_output_topic_, pol);
        auto make_zero = [&](){
            auto a = std::make_shared<ActionOut>();
            a->tick_id     = tick_id;
            a->producer_id = id_.empty() ? std::string("premotor") : id_;
            a->accel       = 0.0f;
            a->source      = "premotor";
            return a;
        };
        if (multi_enabled_) {
            for (auto const& t : output_topics_multi_) {
                bus_->publish(t, make_zero());
            }
        } else if (bilateral_enabled_) {
            bus_->publish(output_topic_left_,  make_zero());
            bus_->publish(output_topic_right_, make_zero());
        } else {
            bus_->publish(action_output_topic_, make_zero());
        }
        return;
    }

    // 1. Score = (W · latent) + b, scaled by gain.
    Eigen::VectorXf scores = (W_ * last_latent_) + b_;
    // 2026-05-29 gait-bucket bet — add per-bucket bias to logits.  This is the
    // per-leg-role context the per-joint-class standing-reward landscape lacks:
    // REINFORCE updates bucket_bias_(bucket, intent) per step so the policy
    // can differentiate "swing-bucket intents" from "stance-bucket intents."
    if (n_buckets_ > 0 && bucket_seen_
        && current_bucket_ >= 0 && current_bucket_ < bucket_bias_.rows()
        && bucket_bias_.cols() == scores.size()) {
        scores.noalias() += bucket_bias_.row(current_bucket_).transpose();
    }
    scores *= gain_;

    // 1b. Phase 6.5.31 — epistemic novelty bonus.  Add (1 - visit_ema)
    // × epistemic_gain to each intent's pre-softmax score, pushing the
    // distribution toward intents that haven't been chosen lately.
    if (epistemic_gain_ > 0.0f && visit_ema_.size() == N) {
        scores.noalias() += epistemic_gain_ * (Eigen::VectorXf::Ones(N) - visit_ema_);
    }

    // 1c. Phase 7.8 — predictive value head.  V[i, :] · latent predicts
    // accumulated reward in the next value_head_lookahead_ ticks if intent
    // i is chosen.  Forward-looking bias on softmax logits: intents with
    // high predicted future reward get more probability mass.  V starts at
    // zero so cold-start adds no bias; as the trajectory accumulates
    // (chosen, observed_reward) pairs, V learns which intents lead to
    // reward.  Composes additively with the policy gradient W.
    if (value_head_gain_ > 0.0f
        && V_.rows() == N && V_.cols() == latent_dim_
        && last_latent_.size() == latent_dim_) {
        Eigen::VectorXf predicted_v = V_ * last_latent_;
        scores.noalias() += value_head_gain_ * predicted_v;
    }

    // 1c2. Phase A2 — per-leg phase latent augmentation.  When configured,
    // each PremotorAI adds W_leg * leg_phase_s_ to its scores. W_leg learns
    // under MC REINFORCE alongside W_ (see finalize_mc_episode). The leg
    // phase signal is genuinely per-Premotor different when each Premotor
    // is configured with its own rhythm.cpg.<leg> topic.
    if (leg_phase_gain_ > 0.0f && leg_phase_seen_
        && W_leg_.rows() == N && W_leg_.cols() == 2) {
        Eigen::VectorXf leg_contrib = W_leg_ * leg_phase_s_;
        scores.noalias() += leg_phase_gain_ * leg_contrib;
        last_leg_phase_contribution_ = float(leg_contrib.norm());
    }

    // 1d. Phase 7.9 — SynergyTimer rhythm bias.  Pre-multiplied by
    // rhythm_bias_gain × confidence in the publisher; Premotor just
    // adds it.  Silent when no rhythm detected (bias=0).
    if (rhythm_bias_seen_ && latest_rhythm_bias_.size() == N) {
        scores.noalias() += latest_rhythm_bias_;
    }

    // 1e. Phase-viscosity P2 — phase-bin switch penalty.  Within one CPG
    // bin, bias the policy toward continuing the previously expressed intent.
    // A strong Cruse/rhythm/W bias can still overcome this because the penalty
    // is just another pre-softmax logit term, not a downstream servo filter.
    bool phase_active = phase_context_seen_ && phase_bins_ > 0 && phase_bin_ >= 0
                     && phase_commit_mode_ != "none";
    bool same_phase_bin = phase_active && last_phase_commit_bin_ >= 0
                       && phase_bin_ == last_phase_commit_bin_;
    if (same_phase_bin && phase_commit_mode_ == "switch_penalty"
        && phase_switch_penalty_ > 0.0f
        && last_chosen_intent_ >= 0 && last_chosen_intent_ < N) {
        for (int i = 0; i < N; ++i) {
            if (i != last_chosen_intent_) scores[i] -= phase_switch_penalty_;
        }
        ++phase_switch_penalties_;
    }

    // 2. Temperature: DA tightens (low T = exploit), HT could loosen later.
    // Phase 6.6.I — rollout-aware temperature modulation.  When
    // pathway_temp_gain_ > 0, compute the average state-visit EMA over
    // every (modality, predicted_id) currently in the consensus
    // pathways.  Familiar predicted trajectory (high mean) → push T up
    // → entropic distribution → exploration of alternative intents.
    // Novel trajectory (low mean) → keep T at the dopamine baseline.
    float pathway_familiarity = 0.0f;
    if (pathway_temp_gain_ > 0.0f && !last_predicted_pathways_.empty()) {
        float sum = 0.0f;
        int   n   = 0;
        for (auto const& [topic, pathway] : last_predicted_pathways_) {
            auto vit = state_visit_ema_.find(topic);
            if (vit == state_visit_ema_.end()) {
                // Unknown modality → unseen → contributes 0 (low familiarity).
                n += int(pathway.size());
                continue;
            }
            for (int pid : pathway) {
                auto eit = vit->second.find(pid);
                sum += (eit == vit->second.end()) ? 0.0f : eit->second;
                ++n;
            }
        }
        if (n > 0) pathway_familiarity = std::clamp(sum / float(n), 0.0f, 1.0f);
    }
    last_pathway_familiarity_ = pathway_familiarity;

    float T = t_base_ * (1.0f + pathway_temp_gain_ * pathway_familiarity)
            / (1.0f + dopamine_ * t_da_gain_);
    T = std::max(0.05f, T);

    // Phase 7.10 — adaptive entropy anneal.  Uses entropy_ema_ from
    // previous tick (updated end-of-tick).  When entropy stays above
    // threshold, multiply T DOWN so the softmax sharpens → Premotor
    // commits → effective accel becomes non-zero → REINFORCE gets a
    // discriminative gradient.  Self-correcting: once entropy falls,
    // T returns to baseline.  No noise injection.
    float anneal_t_mult = 1.0f;
    if (entropy_anneal_gain_ > 0.0f && entropy_ema_ >= 0.0f) {
        float max_h = std::log(float(std::max(2, N)));
        float denom = std::max(0.01f, max_h - entropy_anneal_threshold_);
        float stuck = std::clamp((entropy_ema_ - entropy_anneal_threshold_) / denom,
                                   0.0f, 1.0f);
        anneal_t_mult = std::max(0.10f, 1.0f - entropy_anneal_gain_ * stuck);
        T *= anneal_t_mult;
        T = std::max(0.05f, T);
    }
    last_anneal_t_mult_ = anneal_t_mult;

    scores /= T;

    // 3. Softmax.  Subtract max for numerical stability.
    float smax = scores.maxCoeff();
    Eigen::VectorXf dist(N);
    float sum = 0.0f;
    for (int i = 0; i < N; ++i) { dist[i] = std::exp(scores[i] - smax); sum += dist[i]; }
    if (sum <= 0.0f) sum = 1.0f;
    dist /= sum;

    // 4. Compute weighted-mean accel and entropy.
    float w_accel = 0.0f, ent = 0.0f;
    for (int i = 0; i < N; ++i) {
        w_accel += dist[i] * intent_accels_[i];
        if (dist[i] > 1e-12f) ent -= dist[i] * std::log(dist[i]);
    }

    // Phase 7.10 — entropy_ema_ update.  Used by NEXT tick's anneal
    // calculation.  Seed with first observation; thereafter EMA-smooth.
    if (entropy_anneal_gain_ > 0.0f) {
        if (entropy_ema_ < 0.0f) {
            entropy_ema_ = ent;
        } else {
            entropy_ema_ = (1.0f - entropy_anneal_alpha_) * entropy_ema_
                          + entropy_anneal_alpha_ * ent;
        }
    }

    // 5. Sample chosen intent (for telemetry; can also drive accel choice).
    //    v5.3 Phase B: if an IntentToken arrived on intent_override_topic_
    //    since our last tick (consume-once), use its index instead of
    //    sampling.  We DON'T require tick_id == current tick because the
    //    scheduler may run Premotor before ActionDecoder; ActionDecoder's
    //    override published this tick will arrive in our queue and be
    //    consumed at the NEXT tick.  One-cycle latency is fine — chunks
    //    are 50-tick units; one-tick alignment doesn't change semantics.
    //    Both the motor emission and the REINFORCE crediting downstream
    //    use this `chosen`, so chunk-driven intents naturally feed the
    //    policy gradient identically to self-sampled intents.
    int chosen = 0;
    bool hard_override = false;
    if (pending_override_idx_ >= 0 && pending_override_idx_ < N) {
        chosen = pending_override_idx_;
        hard_override = true;
        ++total_overrides_used_;
        // Consume-once: clear pending so a stale override doesn't leak
        // into ticks where the chunk dispatcher has stopped publishing.
        pending_override_idx_  = -1;
        pending_override_tick_ = uint64_t(-1);
    } else if (last_explore_active_) {
        // Phase 6.7 — HomeokineticExploration override.  Map the directive's
        // held accel to the nearest intent index; that intent becomes the
        // chosen action for this tick.  REINFORCE credits this chosen exactly
        // like a self-sampled one, so explore episodes still drive policy
        // learning (e.g. if the babble accidentally produces a hit, the
        // explore-chosen intent gets the positive update).  No consume-once:
        // HomeokineticExploration publishes every tick with active toggling
        // on/off as its gate fires.
        int best = 0;
        float best_d = std::fabs(intent_accels_[0] - last_explore_accel_);
        for (int i = 1; i < N; ++i) {
            float d = std::fabs(intent_accels_[i] - last_explore_accel_);
            if (d < best_d) { best_d = d; best = i; }
        }
        chosen = best;
        hard_override = true;
        ++total_explore_overrides_used_;
    } else if (sample_action_) {
        std::uniform_real_distribution<float> ud(0.0f, 1.0f);
        float r = ud(rng_);
        float c = 0.0f;
        for (int i = 0; i < N; ++i) {
            c += dist[i];
            if (r <= c) { chosen = i; break; }
        }
    } else {
        // argmax
        int best = 0; float bv = dist[0];
        for (int i = 1; i < N; ++i) if (dist[i] > bv) { bv = dist[i]; best = i; }
        chosen = best;
    }

    raw_chosen_intent_ = chosen;
    if (!hard_override && same_phase_bin && phase_commit_mode_ == "bin_boundary"
        && last_chosen_intent_ >= 0 && last_chosen_intent_ < N
        && chosen != last_chosen_intent_) {
        chosen = last_chosen_intent_;
        ++phase_boundary_holds_;
    }

    if (intent_dwell_ticks_ > 0 && chosen >= 0 && chosen < N) {
        if (hard_override || held_intent_ < 0 || held_intent_ >= N) {
            held_intent_ = chosen;
            held_intent_ticks_left_ = intent_dwell_ticks_;
        } else if (chosen == held_intent_) {
            held_intent_ticks_left_ = intent_dwell_ticks_;
        } else if (held_intent_ticks_left_ > 0) {
            float proposed_p = dist[chosen];
            float held_p = dist[held_intent_];
            if (proposed_p >= held_p + intent_dwell_break_bias_) {
                held_intent_ = chosen;
                held_intent_ticks_left_ = intent_dwell_ticks_;
                ++dwell_breaks_;
            } else {
                chosen = held_intent_;
                --held_intent_ticks_left_;
                ++dwell_holds_;
            }
        } else {
            held_intent_ = chosen;
            held_intent_ticks_left_ = intent_dwell_ticks_;
        }
    } else {
        held_intent_ = -1;
        held_intent_ticks_left_ = 0;
    }

    float emitted_accel = use_weighted_accel_ ? w_accel : intent_accels_[chosen];
    emitted_accel = std::clamp(emitted_accel, accel_min_, accel_max_);
    if (chosen >= 0 && chosen < int(chosen_intent_counts_.size())) {
        chosen_intent_counts_[chosen] += 1;
    }

    // 6. Cache for next-tick credit assignment.
    last_distribution_   = dist;
    last_chosen_intent_  = chosen;
    if (phase_active) {
        last_phase_commit_bin_ = phase_bin_;
    } else {
        last_phase_commit_bin_ = -1;
    }

    // Phase 7.8 — record value-head trajectory entry for this tick, and
    // close out any old entries whose lookahead window has expired.
    if (value_head_gain_ > 0.0f && chosen >= 0
        && V_.rows() == N && V_.cols() == latent_dim_
        && last_latent_.size() == latent_dim_) {
        // Append current step.
        value_traj_.push_back(ValueStep{chosen, last_latent_, tick_id, 0.0f});
        // Pop entries with elapsed lookahead window — apply TD update.
        while (!value_traj_.empty()
               && tick_id >= value_traj_.front().tick_id + uint64_t(value_head_lookahead_)) {
            auto step = value_traj_.front();
            value_traj_.pop_front();
            if (step.chosen >= 0 && step.chosen < N
                && step.latent.size() == latent_dim_) {
                float predicted = V_.row(step.chosen).dot(step.latent);
                float td_error  = step.accumulated_reward - predicted;
                V_.row(step.chosen).noalias() +=
                    (value_head_lr_ * td_error) * step.latent.transpose();
            }
        }
        // Bound deque size as a safety cap (lookahead+10 max).
        while (int(value_traj_.size()) > value_head_lookahead_ + 10) {
            value_traj_.pop_front();
        }
    }
    // v5.4.M Diagnostic B — rolling window of chosen intents for entropy.
    if (chosen >= 0) {
        chosen_window_.push_back(chosen);
        while (int(chosen_window_.size()) > chosen_window_size_max_)
            chosen_window_.pop_front();
    }
    last_accel_          = emitted_accel;
    last_entropy_        = ent;

    // 6c. Phase 6.5.31 — update visit-EMA so the epistemic bonus tracks
    // recent intent usage.  Decays toward the current distribution.
    if (visit_ema_.size() == N) {
        visit_ema_ = (1.0f - epistemic_alpha_) * visit_ema_ + epistemic_alpha_ * dist;
    }

    // 6b. Update eligibility trace (Phase 6.5.29).  When λ=0 this is a
    // no-op since the trace decays to 0 each tick and apply_reward()
    // takes the one-step-credit path.  When λ>0 the trace accumulates
    // (p_i × latent) decayed by λ each tick — Hebbian co-activation
    // accumulator that matches apply_reward's update form.
    if (eligibility_lambda_ > 0.0f
        && E_.rows() == N
        && E_.cols() == latent_dim_) {
        E_ *= eligibility_lambda_;
        for (int i = 0; i < N; ++i) {
            E_.row(i).noalias() += dist[i] * last_latent_.transpose();
        }
    }

    // 7. Publish.
    pol->intent_distribution = dist;
    pol->weighted_accel      = w_accel;
    pol->entropy             = ent;
    pol->temperature         = T;
    pol->chosen_intent       = chosen;
    bus_->publish(policy_output_topic_, pol);

    auto make_act = [&](float accel){
        auto a = std::make_shared<ActionOut>();
        a->tick_id     = tick_id;
        a->producer_id = id_.empty() ? std::string("premotor") : id_;
        a->accel       = accel;
        a->probe       = false;
        a->action_tle  = ent;       // proxy — high entropy = high "surprise" candidate
        a->source      = "premotor";
        return a;
    };
    // Phase 6.6.L — independent per-side coordination noise.  Drawn
    // post-clamp so the noise is on top of the policy command, not
    // hiding inside the clamp range; final clamp re-applies after the
    // noise is added.
    auto sample_motor_noise = [&]() {
        if (output_noise_amplitude_ <= 0.0f) return 0.0f;
        std::uniform_real_distribution<float> u(
            -output_noise_amplitude_, output_noise_amplitude_);
        return u(rng_);
    };
    if (multi_enabled_) {
        // Phase 6.18 — N-channel publish.  Same chosen-intent semantics
        // as the bilateral path below: the discrete intent rows of the
        // bilateral_table aren't on a convex manifold, so weighted mean
        // would produce motion no intent commands.  Each output channel
        // gets the chosen intent's value + per-channel motor noise.
        int M = int(output_topics_multi_.size());
        for (int c = 0; c < M; ++c) {
            float chan_accel = intent_accels_per_channel_[chosen][c];
            chan_accel = std::clamp(chan_accel + sample_motor_noise(),
                                    accel_min_, accel_max_);
            bus_->publish(output_topics_multi_[c], make_act(chan_accel));
        }
    } else if (bilateral_enabled_) {
        // v5.4.I fix: bilateral output ALWAYS uses chosen-intent row.
        //
        // Pre-v5.4.I the bilateral path mirrored the scalar use_weighted_accel
        // convention, emitting (Σ pᵢ·Lᵢ, Σ pᵢ·Rᵢ) — the expected value over
        // the policy distribution.  Two reasons this was wrong:
        //
        // (1) The bilateral intent table is 5 discrete rows of (L,R) pairs.
        //     Weighted-mean across rows produces a "polite default" that no
        //     single intent prescribes — for the default 5-row table the
        //     uniform-distribution mean is (1.2, 1.2), giving spike rates
        //     of (0.3, 0.3) and joint-spike probability 0.09.  Bilateral
        //     table rows aren't on a convex motion manifold; mixing them
        //     produces motion no intent commands.
        //
        // (2) chunk_id-driven intent.override changes `chosen` but does NOT
        //     change `dist` (the softmax over Hebbian scores).  Under the
        //     pre-fix weighted-mean bilateral, the chunk override changed
        //     only telemetry (chosen_intent_counts_, PolicyToken.chosen_intent)
        //     — the motors received the same (Σ pᵢ·Lᵢ, Σ pᵢ·Rᵢ) regardless
        //     of which chunk played.  Trajectory probe (2026-05-10): intent 0
        //     (L=+4, R=-4) and intent 4 (L=-4, R=+4) produced bit-identical
        //     body trajectories at every α — chunks were theatre.
        //
        // The chosen-intent row IS the chunk's commanded action and IS the
        // policy's MAP action; it's what the body should actually execute.
        // use_weighted_accel still controls the scalar accel path
        // (PolicyToken.weighted_accel, ActionOut.accel for single-channel
        // configs) where weighted-mean of a 1D accel range has a sensible
        // interpretation ("graded commitment to a forward direction").
        float left_accel  = intent_accels_left_[chosen];
        float right_accel = intent_accels_right_[chosen];
        left_accel  = std::clamp(left_accel  + sample_motor_noise(), accel_min_, accel_max_);
        right_accel = std::clamp(right_accel + sample_motor_noise(), accel_min_, accel_max_);
        bus_->publish(output_topic_left_,  make_act(left_accel));
        bus_->publish(output_topic_right_, make_act(right_accel));
    } else {
        float noisy = std::clamp(emitted_accel + sample_motor_noise(),
                                  accel_min_, accel_max_);
        bus_->publish(action_output_topic_, make_act(noisy));
    }

    // Phase 6.6.O — behavioral cloning from reflex.  Runs at end of
    // tick so this tick's action.reflex.* publishes have already
    // arrived.  Hebbian credit on the closest-intent row, gated by
    // (1 - α) so high-α (brain-led) periods don't BC.
    apply_bc_update(tick_id);

    // Phase v5.1 — Monte-Carlo trajectory append.  When mc_lr_ > 0 and
    // we have a valid policy sample this tick, snapshot (latent,
    // distribution, chosen, accumulated_reward) and reset the per-tick
    // reward accumulator.  finalize_mc_episode walks this buffer
    // backwards on events.episode_end.
    if (mc_lr_ > 0.0f
        && weights_initialised_
        && last_latent_.size() == latent_dim_
        && chosen >= 0) {
        MCStep step;
        step.latent       = last_latent_;
        step.distribution = dist;
        step.chosen       = chosen;
        step.reward       = accumulated_reward_this_tick_;
        step.bucket       = (bucket_seen_ && n_buckets_ > 0) ? current_bucket_ : -1;
        step.leg_phase_s  = leg_phase_s_;  // Phase A2 — per-step phase for W_leg update
        mc_trajectory_.push_back(std::move(step));
        accumulated_reward_this_tick_ = 0.0f;
    }
}

// ---------------------------------------------------------------------------
// Snapshot / restore (UI-dev W3.2 Tier A)
// ---------------------------------------------------------------------------

namespace {

nlohmann::json vec_to_json(Eigen::VectorXf const& v) {
    nlohmann::json a = nlohmann::json::array();
    for (int i = 0; i < v.size(); ++i) a.push_back(v(i));
    return a;
}
void json_to_vec(nlohmann::json const& j, Eigen::VectorXf& out) {
    if (!j.is_array()) { out = Eigen::VectorXf(); return; }
    out.resize(int(j.size()));
    for (size_t i = 0; i < j.size(); ++i) out(int(i)) = j[i].get<float>();
}
nlohmann::json mat_to_json(Eigen::MatrixXf const& m) {
    nlohmann::json data = nlohmann::json::array();
    for (int r = 0; r < m.rows(); ++r)
        for (int c = 0; c < m.cols(); ++c) data.push_back(m(r, c));
    return nlohmann::json{{"rows", int(m.rows())}, {"cols", int(m.cols())}, {"data", data}};
}
void json_to_mat(nlohmann::json const& j, Eigen::MatrixXf& out) {
    if (!j.is_object() || !j.contains("rows")) { out = Eigen::MatrixXf(); return; }
    int r = j.value("rows", 0), c = j.value("cols", 0);
    out.resize(r, c);
    auto const& data = j["data"];
    for (int i = 0; i < r * c; ++i) out(i / c, i % c) = data[i].get<float>();
}
std::string rng_to_string(std::mt19937 const& rng) {
    std::ostringstream os; os << rng; return os.str();
}
void string_to_rng(std::string const& s, std::mt19937& rng) {
    std::istringstream is(s); is >> rng;
}

}  // namespace

nlohmann::json PremotorAI::snapshot_state() const {
    nlohmann::json bc_counts = nlohmann::json::array();
    for (auto v : bc_intent_counts_) bc_counts.push_back(v);
    nlohmann::json chosen_counts = nlohmann::json::array();
    for (auto v : chosen_intent_counts_) chosen_counts.push_back(v);
    nlohmann::json visit = nlohmann::json::object();
    for (auto const& [mod, inner] : state_visit_ema_) {
        nlohmann::json row = nlohmann::json::object();
        for (auto const& [node, ema] : inner) row[std::to_string(node)] = ema;
        visit[mod] = row;
    }
    nlohmann::json paths = nlohmann::json::object();
    for (auto const& [mod, ids] : last_predicted_pathways_) {
        nlohmann::json a = nlohmann::json::array();
        for (auto v : ids) a.push_back(v);
        paths[mod] = a;
    }
    return nlohmann::json{
        {"version",                   1},
        {"latent_dim",                latent_dim_},
        {"weights_initialised",       weights_initialised_},
        {"W",                         mat_to_json(W_)},
        {"b",                         vec_to_json(b_)},
        {"E",                         mat_to_json(E_)},
        {"last_latent",               vec_to_json(last_latent_)},
        {"last_distribution",         vec_to_json(last_distribution_)},
        {"dopamine",                  dopamine_},
        {"urgency",                   urgency_},
        {"prev_urgency",              prev_urgency_},
        {"have_prev_urgency",         have_prev_urgency_},
        {"visit_ema",                 vec_to_json(visit_ema_)},
        {"value_w",                   vec_to_json(value_w_)},
        {"value_w_target",            vec_to_json(value_w_target_)},
        {"prev_value",                prev_value_},
        {"prev_value_latent",         vec_to_json(prev_value_latent_)},
        {"have_prev_value",           have_prev_value_},
        {"last_accel",                last_accel_},
        {"last_entropy",              last_entropy_},
        {"last_chosen_intent",        last_chosen_intent_},
        {"raw_chosen_intent",         raw_chosen_intent_},
        {"held_intent",               held_intent_},
        {"held_intent_ticks_left",    held_intent_ticks_left_},
        {"dwell_holds",               dwell_holds_},
        {"dwell_breaks",              dwell_breaks_},
        {"phase_context_seen",        phase_context_seen_},
        {"phase_value",               phase_value_},
        {"phase_bin",                 phase_bin_},
        {"last_phase_commit_bin",     last_phase_commit_bin_},
        {"phase_switch_penalties",    phase_switch_penalties_},
        {"phase_boundary_holds",      phase_boundary_holds_},
        {"phase_bin_changes",         phase_bin_changes_},
        {"last_alpha",                last_alpha_},
        {"state_visit_ema",           visit},
        {"last_predicted_pathways",   paths},
        {"last_pathway_familiarity",  last_pathway_familiarity_},
        {"last_reflex_left",          last_reflex_left_},
        {"last_reflex_right",         last_reflex_right_},
        {"last_reflex_uni",           last_reflex_uni_},
        {"last_reflex_left_tick",     last_reflex_left_tick_},
        {"last_reflex_right_tick",    last_reflex_right_tick_},
        {"last_reflex_uni_tick",      last_reflex_uni_tick_},
        {"last_bc_intent",            last_bc_intent_},
        {"bc_intent_counts",          bc_counts},
        {"chosen_intent_counts",      chosen_counts},
        {"bc_total_updates",          bc_total_updates_},
        {"rng",                       rng_to_string(rng_)},
    };
}

void PremotorAI::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("PremotorAI::restore_state: unknown version " +
                                 std::to_string(version));
    }
    latent_dim_           = s.value("latent_dim",          latent_dim_);
    weights_initialised_  = s.value("weights_initialised", weights_initialised_);
    if (s.contains("W")) json_to_mat(s["W"], W_);
    if (s.contains("b")) json_to_vec(s["b"], b_);
    if (s.contains("E")) json_to_mat(s["E"], E_);
    if (s.contains("last_latent"))       json_to_vec(s["last_latent"],       last_latent_);
    if (s.contains("last_distribution")) json_to_vec(s["last_distribution"], last_distribution_);
    dopamine_           = s.value("dopamine",           dopamine_);
    urgency_            = s.value("urgency",            urgency_);
    prev_urgency_       = s.value("prev_urgency",       prev_urgency_);
    have_prev_urgency_  = s.value("have_prev_urgency",  have_prev_urgency_);
    if (s.contains("visit_ema")) json_to_vec(s["visit_ema"], visit_ema_);
    if (s.contains("value_w"))         json_to_vec(s["value_w"],         value_w_);
    if (s.contains("value_w_target"))  json_to_vec(s["value_w_target"],  value_w_target_);
    prev_value_         = s.value("prev_value",         prev_value_);
    if (s.contains("prev_value_latent")) json_to_vec(s["prev_value_latent"], prev_value_latent_);
    have_prev_value_    = s.value("have_prev_value",    have_prev_value_);
    last_accel_         = s.value("last_accel",         last_accel_);
    last_entropy_       = s.value("last_entropy",       last_entropy_);
    last_chosen_intent_ = s.value("last_chosen_intent", last_chosen_intent_);
    raw_chosen_intent_  = s.value("raw_chosen_intent",  raw_chosen_intent_);
    held_intent_        = s.value("held_intent",        held_intent_);
    held_intent_ticks_left_ = s.value("held_intent_ticks_left", held_intent_ticks_left_);
    dwell_holds_        = s.value("dwell_holds",        dwell_holds_);
    dwell_breaks_       = s.value("dwell_breaks",       dwell_breaks_);
    phase_context_seen_ = s.value("phase_context_seen", phase_context_seen_);
    phase_value_        = s.value("phase_value",        phase_value_);
    phase_bin_          = s.value("phase_bin",          phase_bin_);
    last_phase_commit_bin_ = s.value("last_phase_commit_bin", last_phase_commit_bin_);
    phase_switch_penalties_ = s.value("phase_switch_penalties", phase_switch_penalties_);
    phase_boundary_holds_ = s.value("phase_boundary_holds", phase_boundary_holds_);
    phase_bin_changes_  = s.value("phase_bin_changes",  phase_bin_changes_);
    last_alpha_         = s.value("last_alpha",         last_alpha_);
    state_visit_ema_.clear();
    if (s.contains("state_visit_ema") && s["state_visit_ema"].is_object()) {
        for (auto it1 = s["state_visit_ema"].begin(); it1 != s["state_visit_ema"].end(); ++it1) {
            auto& inner = state_visit_ema_[it1.key()];
            for (auto it2 = it1.value().begin(); it2 != it1.value().end(); ++it2)
                inner[std::stoi(it2.key())] = it2.value().get<float>();
        }
    }
    last_predicted_pathways_.clear();
    if (s.contains("last_predicted_pathways") && s["last_predicted_pathways"].is_object()) {
        for (auto it = s["last_predicted_pathways"].begin();
             it != s["last_predicted_pathways"].end(); ++it) {
            auto& v = last_predicted_pathways_[it.key()];
            for (auto const& x : it.value()) v.push_back(x.get<int>());
        }
    }
    last_pathway_familiarity_ = s.value("last_pathway_familiarity", last_pathway_familiarity_);
    last_reflex_left_         = s.value("last_reflex_left",         last_reflex_left_);
    last_reflex_right_        = s.value("last_reflex_right",        last_reflex_right_);
    last_reflex_uni_          = s.value("last_reflex_uni",          last_reflex_uni_);
    last_reflex_left_tick_    = s.value("last_reflex_left_tick",    last_reflex_left_tick_);
    last_reflex_right_tick_   = s.value("last_reflex_right_tick",   last_reflex_right_tick_);
    last_reflex_uni_tick_     = s.value("last_reflex_uni_tick",     last_reflex_uni_tick_);
    last_bc_intent_           = s.value("last_bc_intent",           last_bc_intent_);
    bc_intent_counts_.clear();
    if (s.contains("bc_intent_counts"))
        for (auto const& x : s["bc_intent_counts"]) bc_intent_counts_.push_back(x.get<int>());
    chosen_intent_counts_.clear();
    if (s.contains("chosen_intent_counts"))
        for (auto const& x : s["chosen_intent_counts"]) chosen_intent_counts_.push_back(x.get<int>());
    bc_total_updates_ = s.value("bc_total_updates", bc_total_updates_);
    if (s.contains("rng") && s["rng"].is_string())
        string_to_rng(s["rng"].get<std::string>(), rng_);
}

} // namespace ogma
