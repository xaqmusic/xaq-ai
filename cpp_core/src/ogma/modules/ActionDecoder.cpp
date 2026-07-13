#include "ogma/modules/ActionDecoder.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <typeindex>
#include <unordered_set>
#include <variant>

#include <nlohmann/json.hpp>

#include "ogma/Rng.hpp"

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
    throw std::invalid_argument("ActionDecoder param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("ActionDecoder param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ActionDecoder param '" + key + "' must be string");
}

std::string hebb_key(std::string const& modality, int prev, int cur, int proprio) {
    return modality + "|" + std::to_string(prev) + "|"
                          + std::to_string(cur)  + "|"
                          + std::to_string(proprio);
}
std::string val_key(int state, int proprio, int bin) {
    return std::to_string(state) + "|" + std::to_string(proprio)
           + "|" + std::to_string(bin);
}
std::string fwd_key(int state, int bin) {
    return std::to_string(state) + "|" + std::to_string(bin);
}

} // namespace

ActionDecoder::ActionDecoder()  = default;
ActionDecoder::~ActionDecoder() = default;

std::string_view ActionDecoder::type_name() const { return "ActionDecoder"; }

std::vector<TopicSpec> ActionDecoder::input_topics() const {
    std::vector<TopicSpec> out{
        TopicSpec{std::string("consensus.") + std::to_string(consensus_level_),
                  std::type_index(typeid(ConsensusToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kDriveErrors,
                  std::type_index(typeid(DriveErrors)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kNeuroState,
                  std::type_index(typeid(NeuroState)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{proprio_topic_,
                  std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
    if (!interest_topic_.empty()) {
        out.push_back(TopicSpec{interest_topic_,
                                 std::type_index(typeid(ReflexGate)),
                                 SubscriptionKind::Direct, /*required=*/false});
    }
    if (use_rollout_) {
        out.push_back(TopicSpec{topics::kRolloutResult,
                                 std::type_index(typeid(RolloutResult)),
                                 SubscriptionKind::Direct, /*required=*/false});
    }
    if (use_chunks_) {
        out.push_back(TopicSpec{topics::kMotorChunks,
                                 std::type_index(typeid(MotorChunks)),
                                 SubscriptionKind::Direct, /*required=*/false});
        out.push_back(TopicSpec{topics::kMotorPlayResp,
                                 std::type_index(typeid(MotorPlayStream)),
                                 SubscriptionKind::Direct, /*required=*/false});
        // Consensus motif gates dispatch — chunks fire only when their
        // stored trigger_consensus_motif_id matches the current winner.
        out.push_back(TopicSpec{"sequence.motif.consensus.0",
                                 std::type_index(typeid(SequenceMotif)),
                                 SubscriptionKind::Direct, /*required=*/false});
    }
    // Phase 6.5.3.8 — when eligibility traces are enabled, subscribe to
    // events.* so we can clear the trace on episode boundaries.  Without
    // boundary clearing, an episode-end failure δ propagates back through
    // the trace into pre-failure (s,a) pairs that didn't cause the
    // failure, and the trace bridges across the body's reset into the
    // next episode's fresh start — both contaminate learning.
    if (eligibility_lambda_ > 0.0f) {
        out.push_back(TopicSpec{topics::kEventsPrefix,
                                 std::type_index(typeid(EnvEvent)),
                                 SubscriptionKind::Direct, /*required=*/false});
    }
    return out;
}

std::vector<TopicSpec> ActionDecoder::output_topics() const {
    std::vector<TopicSpec> out{
        TopicSpec{output_topic_, std::type_index(typeid(ActionOut))}
    };
    if (use_rollout_) {
        out.push_back(TopicSpec{topics::kRolloutQuery,
                                 std::type_index(typeid(RolloutQuery))});
    }
    if (use_chunks_) {
        out.push_back(TopicSpec{topics::kMotorPlayCmd,
                                 std::type_index(typeid(MotorPlayCmd))});
    }
    if (!intent_override_topic_.empty()) {
        out.push_back(TopicSpec{intent_override_topic_,
                                 std::type_index(typeid(IntentToken))});
    }
    return out;
}

ParamSchema ActionDecoder::params_schema() const {
    return {
        {"consensus_level",          ParamMutability::HotMutable,        "Which consensus.<n> to consume", ParamValue{int64_t{0}}},
        {"output_topic",             ParamMutability::ConstructionOnly,  "Phase 6.6.F: where to publish ActionOut (default action.out; set to action.brain to route through MotorFader)", ParamValue{std::string(topics::kActionOut)}},
        {"intent_override_topic",    ParamMutability::ConstructionOnly,  "v5.3 Phase B: when set, intent-sequence chunks (MotorPlayStream.intents non-empty) are replayed by publishing IntentToken on this topic instead of ActionOut.  Premotor reads it for chunk-driven intent overrides.  Empty = legacy float-action chunks only.", ParamValue{std::string("")}},
        {"intent_channel_radix",     ParamMutability::ConstructionOnly,  "Phase 7.2-EPM: per-channel radix array (e.g. [5,5,5]) matching PolicyChannelAggregator on the upstream side.  When non-empty, each chunk-replay tick UNPACKS the combined intent into IntentToken.indices so multi-channel chunks coordinate multiple Premotors.  Empty (default) preserves legacy single-channel intent dispatch.", std::nullopt},
        {"entry_topic",              ParamMutability::ConstructionOnly,  "v5.4 Phase A: slow-keyframe topic (RealityToken from slow consensus EPM) for entry-context history.  When set, episodic chunks (those with non-empty entry_embeddings) only dispatch when the brain's last entry_keyframes embeddings cosine-match the chunk's entry_embeddings above entry_match_threshold.  Empty = pure Beta-prior dispatch (legacy).", ParamValue{std::string("")}},
        {"entry_keyframes",          ParamMutability::HotMutable,        "v5.4 Phase A: how many trailing entry-context embeddings to track + match against chunk entry.  Must equal EpisodicCapture.entry_keyframes.  Default 2.", ParamValue{int64_t{2}}},
        {"entry_match_threshold",    ParamMutability::HotMutable,        "v5.4 Phase A: minimum cosine similarity (0..1) at EACH entry position required for entry-match dispatch.  Default 0.70.", ParamValue{0.70}},
        {"entry_use_winner_prototype", ParamMutability::HotMutable,    "v5.4.L final: when true, populate entry_history from keyframe RealityToken.winner_prototype instead of .latent.  MUST match EpisodicCapture.entry_use_winner_prototype or chunks' entry_embeddings won't share vector space with current entry_history.  Default false = legacy latent.", ParamValue{false}},
        {"secondary_entry_topic",      ParamMutability::ConstructionOnly, "v5.4.M — short+long entry fusion.  When non-empty, subscribe to this additional RealityToken topic; entry_history vectors become concat(primary || secondary).  MUST match EpisodicCapture.secondary_keyframe_topic so chunks' entry_embeddings and runtime entry_history share vector space.  Empty = legacy single-source.", ParamValue{std::string("")}},
        {"manual_chunk_id",          ParamMutability::HotMutable,        "v5.4 Phase C probe: when != 0, every try_dispatch_chunk force-dispatches THIS chunk id (bypassing all scoring + threshold gates).  0 = sentinel 'disabled' (no organic chunk uses id=0; organic starts at 1, seeds at -1 and below).  Set from UI inspector to fire any chunk in the library on demand to inspect playback behaviour.  Set back to 0 to resume normal scoring dispatch.  manual_dispatches counter ticks each successful force-dispatch; manual_dispatch_misses counts attempts when the chunk id wasn't in the current library.", ParamValue{int64_t{0}}},
        {"min_chunk_score",          ParamMutability::HotMutable,        "v5.4 Phase E (Proposal A): minimum Beta(1,1)-prior success rate required to dispatch any chunk.  Default 0.5 = legacy commit threshold.  Raising (e.g., 0.75) filters fresh chunks until they accumulate replay history.  dispatches_gated_score counter ticks each rejection.", ParamValue{0.5}},
        {"min_entry_match_product",  ParamMutability::HotMutable,        "v5.4 Phase E (Proposal A): for episodic chunks, the PRODUCT of per-position cosine similarities must clear this.  Default 0.0 = off (only per-position entry_match_threshold applies).  Raising (e.g., 0.85) demands tight overall context match.  dispatches_gated_match counter ticks each rejection.", ParamValue{0.0}},
        {"chunk_rearm_threshold",    ParamMutability::HotMutable,        "v5.4.J: per-chunk armed-state Schmitt trigger lower threshold.  Episodic chunks (with entry_embeddings) start armed=false on arrival and after dispatch; they re-arm when current entry-match drops BELOW this value, then can fire again when match rises above entry_match_threshold.  Default 0.40 gives a hysteresis band against entry_match_threshold=0.70.  Prevents the 'eat→immediate replay' pattern where a just-captured chunk fires before the agent has left the captured context.  dispatches_blocked_unarmed counter ticks each rejection.  Set to 0 to disable the gate (chunks can always fire).", ParamValue{0.40}},
        {"chunk_dispatch_min_age_ticks", ParamMutability::HotMutable,    "v5.4.L: minimum age (in ticks) a chunk must have before it is eligible for dispatch.  Computed as (current_tick - chunk.created_tick_id).  Default 60 = 1 sim-sec at 60Hz, which keeps a just-captured chunk from firing on the very next tick after its reward event (the eat→replay loop the Schmitt rearm gate alone can't prevent when the slow consensus encoding is degenerate and chunks re-arm immediately).  Set to 0 to disable.  Manual probes (manual_chunk_id != 0) bypass this gate.  dispatches_blocked_too_young counter ticks each rejection.", ParamValue{int64_t{60}}},
        {"proprio_topic",            ParamMutability::ConstructionOnly,  "RealityToken topic for proprio_node lookup", ParamValue{std::string("reality.proprio.imu")}},
        {"action_bins",              ParamMutability::ConstructionOnly,  "Action discretization (default 3 = -1/0/+1)", ParamValue{int64_t{3}}},
        {"accel_min",                ParamMutability::HotMutable,        "Output clamp lower bound",      ParamValue{-4.0}},
        {"accel_max",                ParamMutability::HotMutable,        "Output clamp upper bound",      ParamValue{4.0}},
        {"efe_temperature",          ParamMutability::HotMutable,        "EFE softmax temperature",       ParamValue{1.0}},
        {"pragmatic_gain",           ParamMutability::HotMutable,        "Weight of pragmatic value",     ParamValue{10.0}},
        {"epistemic_gain",           ParamMutability::HotMutable,        "Weight of epistemic value",     ParamValue{1.0}},
        {"efe_select",               ParamMutability::ConstructionOnly,  "Stage-1 active inference: select by expected free energy over the learned forward model (pragmatic 1-step lookahead + interest-scaled epistemic entropy) instead of TD-value argmax+ε-greedy.  Default false = legacy value-RL (CartPole/MountainCar/picrawler unchanged).", ParamValue{false}},
        {"interest_topic",           ParamMutability::ConstructionOnly,  "ReflexGate topic (DistressDrive cognition.interest) feeding the epistemic/curiosity drive into EFE selection.  Empty = off.", ParamValue{std::string("")}},
        {"pref_obs_topic",           ParamMutability::ConstructionOnly,  "ProprioToken scalar topic (e.g. reality.proprio.scent_max) that grounds the EFE pragmatic term in OBSERVATIONS: prefer actions whose predicted next-state has the highest expected preferred-obs (per-state EMA), not the slow learned value.  Empty = legacy E[V].", ParamValue{std::string("")}},
        {"obs_value_alpha",          ParamMutability::HotMutable,        "EMA rate for the per-state preferred-observation estimate (grounded pragmatic).", ParamValue{0.05}},
        {"authority_topic",          ParamMutability::ConstructionOnly,  "2026-06-20 CREDIT-BY-AUTHORITY: ProprioToken scalar ∈[0,1] (e.g. MotorBus motor.bus.authority.cog) = this actor's share of the realized body drive. When set, the forward-model transition learning is gated PROBABILISTICALLY by it — the actor only credits a (state,action)→next transition in proportion to how much it actually DROVE the body, so it never learns from motion caused by other channels (the bus mix / a ducked or masked channel). Empty = full learning (bit-identical).", ParamValue{std::string("")}},
        {"pref_obs_index",           ParamMutability::ConstructionOnly,  "Which value of the pref_obs ProprioToken to use: 0 = scalar (scent_max) or cx; 1 = cy (FORWARD/food-ahead component of percept.scent_compass) — ground a TURN actor on this so facing food is immediately preferred.", ParamValue{int64_t{0}}},
        {"joint_action",             ParamMutability::ConstructionOnly,  "v1 coxswain: true → ONE actor selects a JOINT (turn,thrust) action (index = turn_bin*thrust_bins + thrust_bin), publishing cog.steer + cog.thrust together (kills the two-actor mirror collapse). Default false = legacy 1-D.", ParamValue{false}},
        {"thrust_bins",              ParamMutability::ConstructionOnly,  "Joint action: number of thrust (common-mode) bins.", ParamValue{int64_t{3}}},
        {"thrust_accel_min",         ParamMutability::ConstructionOnly,  "Joint action: min thrust accel.", ParamValue{-4.0}},
        {"thrust_accel_max",         ParamMutability::ConstructionOnly,  "Joint action: max thrust accel.", ParamValue{4.0}},
        {"thrust_output_topic",      ParamMutability::ConstructionOnly,  "Joint action: ActionOut topic for the thrust channel (e.g. cog.thrust); output_topic carries the turn channel.", ParamValue{std::string("")}},
        {"plan_horizon",             ParamMutability::HotMutable,        "Receding-horizon depth: 1 = legacy 1-step argmax; >1 = plan the H-step (turn,thrust) sequence that maximizes cumulative predicted target (fixes 1-step myopia).", ParamValue{int64_t{1}}},
        {"plan_gamma",               ParamMutability::HotMutable,        "Discount across the planning horizon.", ParamValue{0.9}},
        {"green_obs_topic",          ParamMutability::ConstructionOnly,  "Second target gauge (long-range visual 'green loom'), e.g. reality.proprio.green_fraction. Empty = scent only.", ParamValue{std::string("")}},
        {"w_scent",                  ParamMutability::HotMutable,        "Plan target weight on the scent gauge.", ParamValue{1.0}},
        {"w_green",                  ParamMutability::HotMutable,        "Plan target weight on the green-loom gauge.", ParamValue{1.0}},
        {"stay_penalty",             ParamMutability::HotMutable,        "2026-06-20 anti-freeze: penalty subtracted in the planner from a LEARNED staying-put action (a visited (s,a) whose forward model predicts s'==s). Prevents the planner locking into a zero-motion fixed point (the dark-room freeze) once exploration decays. 0 = off (bit-identical); ~node_value scale (e.g. 1-3) to dominate a high-value pause.", ParamValue{0.0}},
        {"plan_temperature",         ParamMutability::HotMutable,        "2026-06-20 PERSISTENT EXPLORATION (coxswain): when >0, SAMPLE the planned action from softmax(plan_scores/T) instead of pure argmax, so exploration never fully decays — escapes the bootstrap-failure (stumble onto food) AND the facing-freeze degeneration. ~0.3-1 of the node_value scale. 0 = argmax (bit-identical).", ParamValue{0.0}},
        {"urgency_exploit_threshold",ParamMutability::HotMutable,        "Urgency above this → exploit",  ParamValue{0.6}},
        {"urgency_exploit_bias",     ParamMutability::HotMutable,        "Pragmatic gain multiplier when exploit", ParamValue{1.5}},
        {"td_lambda",                ParamMutability::HotMutable,        "TD value-update smoothing (per-entry mixing rate)", ParamValue{0.7}},
        {"td_gamma",                 ParamMutability::HotMutable,        "TD bootstrap discount (0 disables; ~0.95 enables Q-learning)", ParamValue{0.0}},
        {"eligibility_lambda",       ParamMutability::HotMutable,        "TD(λ) trace decay; 0 disables (pure TD(0)); ~0.7 propagates δ back through recent (s,a) trail", ParamValue{0.0}},
        {"eligibility_max_len",      ParamMutability::HotMutable,        "eligibility trace depth (ticks); raise (e.g. 64) to bridge a long action→reward delay; only used when eligibility_lambda>0", ParamValue{int64_t{12}}},
        {"commit_ticks",             ParamMutability::HotMutable,        "Mode-2 temporal abstraction: hold each selected action for K ticks and do ONE SMDP TD update on the reward integrated over the commitment. 1 = legacy per-tick TD. >1 (e.g. 30) gives the action a multi-tick reward consequence so its Q separates from neighbours (fixes the tied-Q-bins homing failure).", ParamValue{int64_t{1}}},
        {"valence_decay_pos",        ParamMutability::HotMutable,        "Decay on positive valence entries", ParamValue{0.99999}},
        {"valence_decay_neg",        ParamMutability::HotMutable,        "Decay on negative valence entries", ParamValue{0.9999}},
        {"valence_max_size",         ParamMutability::HotMutable,        "LRU cap on valence map",        ParamValue{int64_t{2000}}},
        {"hebbian_max_size",         ParamMutability::HotMutable,        "LRU cap on Hebbian table",      ParamValue{int64_t{2000}}},
        {"use_rollout",              ParamMutability::HotMutable,        "Issue rollout.query for epistemic value (Phase 3)", ParamValue{false}},
        {"rollout_K",                ParamMutability::HotMutable,        "Trajectory samples per query",  ParamValue{int64_t{16}}},
        {"rollout_M",                ParamMutability::HotMutable,        "Forward horizon per trajectory",ParamValue{int64_t{4}}},
        {"use_chunks",               ParamMutability::HotMutable,        "Dispatch chunks via motor.play.cmd (Phase 3)", ParamValue{false}},
        {"master_seed",              ParamMutability::ConstructionOnly,  "RNG namespace seed",            ParamValue{int64_t{0}}},
    };
}

void ActionDecoder::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ActionDecoder requires a non-null Bus");

    apply_param(params, "consensus_level",          [&](auto const& v){ consensus_level_           = int(get_int(v, "consensus_level")); });
    apply_param(params, "output_topic",             [&](auto const& v){ output_topic_              = get_string(v, "output_topic"); });
    apply_param(params, "intent_override_topic",    [&](auto const& v){ intent_override_topic_     = get_string(v, "intent_override_topic"); });
    {
        auto it = params.find("intent_channel_radix");
        if (it != params.end()) {
            if (auto p = std::get_if<std::vector<double>>(&it->second)) {
                intent_channel_radix_.clear();
                intent_channel_radix_.reserve(p->size());
                for (double d : *p) intent_channel_radix_.push_back(std::max(1, int(d)));
            }
        }
    }
    apply_param(params, "entry_topic",              [&](auto const& v){ entry_topic_               = get_string(v, "entry_topic"); });
    apply_param(params, "entry_keyframes",          [&](auto const& v){ entry_keyframes_           = std::max(1, int(get_int(v, "entry_keyframes"))); });
    apply_param(params, "entry_match_threshold",    [&](auto const& v){ entry_match_threshold_     = std::clamp(float(get_double(v, "entry_match_threshold")), 0.0f, 1.0f); });
    apply_param(params, "entry_use_winner_prototype", [&](auto const& v){ entry_use_winner_prototype_ = std::get<bool>(v); });
    apply_param(params, "secondary_entry_topic",       [&](auto const& v){ secondary_entry_topic_     = get_string(v, "secondary_entry_topic"); });
    apply_param(params, "manual_chunk_id",          [&](auto const& v){ manual_chunk_id_           = int(get_int(v, "manual_chunk_id")); });
    apply_param(params, "min_chunk_score",          [&](auto const& v){ min_chunk_score_           = std::clamp(float(get_double(v, "min_chunk_score")), 0.0f, 1.0f); });
    apply_param(params, "min_entry_match_product",  [&](auto const& v){ min_entry_match_product_   = std::clamp(float(get_double(v, "min_entry_match_product")), 0.0f, 1.0f); });
    apply_param(params, "chunk_rearm_threshold",    [&](auto const& v){ chunk_rearm_threshold_     = std::clamp(float(get_double(v, "chunk_rearm_threshold")), 0.0f, 1.0f); });
    apply_param(params, "chunk_dispatch_min_age_ticks", [&](auto const& v){ chunk_dispatch_min_age_ticks_ = std::max(0, int(get_int(v, "chunk_dispatch_min_age_ticks"))); });
    apply_param(params, "proprio_topic",            [&](auto const& v){ proprio_topic_             = get_string(v, "proprio_topic"); });
    apply_param(params, "action_bins",              [&](auto const& v){ action_bins_               = std::max(2, int(get_int(v, "action_bins"))); });
    apply_param(params, "accel_min",                [&](auto const& v){ accel_min_                 = float(get_double(v, "accel_min")); });
    apply_param(params, "accel_max",                [&](auto const& v){ accel_max_                 = float(get_double(v, "accel_max")); });
    apply_param(params, "efe_temperature",          [&](auto const& v){ efe_temperature_           = float(get_double(v, "efe_temperature")); });
    apply_param(params, "pragmatic_gain",           [&](auto const& v){ pragmatic_gain_            = float(get_double(v, "pragmatic_gain")); });
    apply_param(params, "epistemic_gain",           [&](auto const& v){ epistemic_gain_            = float(get_double(v, "epistemic_gain")); });
    apply_param(params, "efe_select",               [&](auto const& v){ efe_select_                = std::get<bool>(v); });
    apply_param(params, "interest_topic",           [&](auto const& v){ interest_topic_            = get_string(v, "interest_topic"); });
    apply_param(params, "pref_obs_topic",           [&](auto const& v){ pref_obs_topic_            = get_string(v, "pref_obs_topic"); });
    apply_param(params, "authority_topic",          [&](auto const& v){ authority_topic_           = get_string(v, "authority_topic"); });
    apply_param(params, "pref_obs_index",           [&](auto const& v){ pref_obs_index_            = int(get_int(v, "pref_obs_index")); });
    apply_param(params, "joint_action",             [&](auto const& v){ joint_action_              = std::get<bool>(v); });
    apply_param(params, "thrust_bins",              [&](auto const& v){ thrust_bins_               = int(get_int(v, "thrust_bins")); });
    apply_param(params, "thrust_accel_min",         [&](auto const& v){ thrust_accel_min_          = float(get_double(v, "thrust_accel_min")); });
    apply_param(params, "thrust_accel_max",         [&](auto const& v){ thrust_accel_max_          = float(get_double(v, "thrust_accel_max")); });
    apply_param(params, "thrust_output_topic",      [&](auto const& v){ thrust_output_topic_       = get_string(v, "thrust_output_topic"); });
    apply_param(params, "plan_horizon",             [&](auto const& v){ plan_horizon_              = int(get_int(v, "plan_horizon")); });
    apply_param(params, "plan_gamma",               [&](auto const& v){ plan_gamma_                = float(get_double(v, "plan_gamma")); });
    apply_param(params, "green_obs_topic",          [&](auto const& v){ green_obs_topic_           = get_string(v, "green_obs_topic"); });
    apply_param(params, "w_scent",                  [&](auto const& v){ w_scent_                   = float(get_double(v, "w_scent")); });
    apply_param(params, "w_green",                  [&](auto const& v){ w_green_                   = float(get_double(v, "w_green")); });
    apply_param(params, "stay_penalty",             [&](auto const& v){ stay_penalty_              = float(get_double(v, "stay_penalty")); });
    apply_param(params, "plan_temperature",         [&](auto const& v){ plan_temperature_          = float(get_double(v, "plan_temperature")); });
    apply_param(params, "obs_value_alpha",          [&](auto const& v){ obs_value_alpha_           = float(get_double(v, "obs_value_alpha")); });
    apply_param(params, "urgency_exploit_threshold",[&](auto const& v){ urgency_exploit_threshold_ = float(get_double(v, "urgency_exploit_threshold")); });
    apply_param(params, "urgency_exploit_bias",     [&](auto const& v){ urgency_exploit_bias_      = float(get_double(v, "urgency_exploit_bias")); });
    apply_param(params, "td_lambda",                [&](auto const& v){ td_lambda_                 = float(get_double(v, "td_lambda")); });
    apply_param(params, "td_gamma",                 [&](auto const& v){ td_gamma_                  = float(get_double(v, "td_gamma")); });
    apply_param(params, "eligibility_lambda",       [&](auto const& v){ eligibility_lambda_        = float(get_double(v, "eligibility_lambda")); });
    apply_param(params, "eligibility_max_len",       [&](auto const& v){ eligibility_max_len_       = int(get_int(v, "eligibility_max_len")); });
    apply_param(params, "commit_ticks",              [&](auto const& v){ commit_ticks_              = int(get_int(v, "commit_ticks")); });
    apply_param(params, "valence_decay_pos",        [&](auto const& v){ valence_decay_pos_         = float(get_double(v, "valence_decay_pos")); });
    apply_param(params, "valence_decay_neg",        [&](auto const& v){ valence_decay_neg_         = float(get_double(v, "valence_decay_neg")); });
    apply_param(params, "valence_max_size",         [&](auto const& v){ valence_max_size_          = get_int(v, "valence_max_size"); });
    apply_param(params, "hebbian_max_size",         [&](auto const& v){ hebbian_max_size_          = get_int(v, "hebbian_max_size"); });
    apply_param(params, "use_rollout",              [&](auto const& v){ use_rollout_               = std::get<bool>(v); });
    apply_param(params, "rollout_K",                [&](auto const& v){ rollout_K_                 = std::max(1, int(get_int(v, "rollout_K"))); });
    apply_param(params, "rollout_M",                [&](auto const& v){ rollout_M_                 = std::max(1, int(get_int(v, "rollout_M"))); });
    apply_param(params, "use_chunks",               [&](auto const& v){ use_chunks_                = std::get<bool>(v); });
    apply_param(params, "master_seed",              [&](auto const& v){ master_seed_               = uint64_t(get_int(v, "master_seed")); });

    // Seed the softmax-sampling RNG.  master_seed_ is set by the host
    // (paired-seed harness) so this stays deterministic per OGMA_SEED.
    // XOR with a constant so the action sampler doesn't share a sequence
    // with any other module that might seed from master_seed_.
    rng_.seed(master_seed_ ^ 0x6F676D6173616D70ULL /* "ogmasamp" */);

    sub_ids_.clear();
    // Phase 6.9 — consensus subscription is optional: consensus_level < 0 means
    // "no upstream voter" (proprio-only state for the bundle nav path).
    if (consensus_level_ >= 0) {
        sub_ids_.push_back(bus_->subscribe(std::string("consensus.") + std::to_string(consensus_level_),
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_consensus(t, p); }));
    }
    sub_ids_.push_back(bus_->subscribe(topics::kDriveErrors,
                                        SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_drive(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState,
                                        SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_neuro(t, p); }));
    // Phase 6.9 — proprio subscription is optional: empty proprio_topic means
    // "no proprio EPM" (consensus-only state for the voter nav path).
    if (!proprio_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(proprio_topic_,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_proprio_token(t, p); }));
    }
    if (!interest_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(interest_topic_,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_interest(t, p); }));
    }
    if (!authority_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(authority_topic_,
            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_authority(t, p); }));
    }
    if (!pref_obs_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(pref_obs_topic_,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_pref_obs(t, p); }));
    }
    if (!green_obs_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(green_obs_topic_,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_green_obs(t, p); }));
    }

    if (use_rollout_) {
        sub_ids_.push_back(bus_->subscribe(topics::kRolloutResult,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_rollout_result(t, p); }));
    }
    if (use_chunks_) {
        sub_ids_.push_back(bus_->subscribe(topics::kMotorChunks,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_motor_chunks(t, p); }));
        sub_ids_.push_back(bus_->subscribe(topics::kMotorPlayResp,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_motor_stream(t, p); }));
        // Track the current baked consensus motif so try_dispatch_chunk can
        // gate on perceptual context.  Lambda captures consensus_motif_id_
        // directly — only baked motifs are recorded so chunk dispatch
        // doesn't chase drifting unbaked ids.
        sub_ids_.push_back(bus_->subscribe("sequence.motif.consensus.0",
                                            SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p) {
                auto m = std::dynamic_pointer_cast<const SequenceMotif>(p);
                if (m && m->motif_id >= 0 && m->is_baked)
                    current_consensus_motif_id_ = m->motif_id;
            }));
        // v5.4 Phase A — entry-context history.  Subscribe to slow-keyframe
        // RealityToken topic; on each fresh keyframe (deduplicated by
        // approximate-equality of the latent vector vs the previous one),
        // append to the rolling history buffer of size entry_keyframes_.
        // Empty entry_topic_ disables the path = legacy Beta-only dispatch.
        if (!entry_topic_.empty()) {
            sub_ids_.push_back(bus_->subscribe(entry_topic_,
                                                SubscriptionKind::Direct,
                [this](std::string_view, MessagePtr p){
                    auto rt = std::dynamic_pointer_cast<const RealityToken>(p);
                    if (!rt) return;
                    // v5.4.L final — pick entry-vector source.  MUST match
                    // EpisodicCapture.entry_use_winner_prototype (the
                    // chunk's stored entry_embeddings and current
                    // entry_history must share a vector space).
                    Eigen::VectorXf const& src =
                        (entry_use_winner_prototype_ && rt->winner_prototype.size() > 0)
                            ? rt->winner_prototype
                            : rt->latent;
                    if (src.size() == 0) return;  // bootstrap / no prototype
                    // Republish-detection MUST use rt->latent (raw),
                    // not src — winner_prototype mode would otherwise
                    // drop legitimate fresh keyframes that happen to win
                    // the same GNG cell as the prior keyframe.
                    if (last_entry_raw_.size() == rt->latent.size()
                        && last_entry_raw_.size() > 0
                        && last_entry_raw_.isApprox(rt->latent, 1e-6f)) {
                        return;   // slow EPM republish
                    }
                    last_entry_raw_ = rt->latent;
                    // v5.4.M — concat secondary src when fusion is on.
                    // Drop the keyframe if secondary hasn't been seen
                    // yet (vector dim would not match captured chunks).
                    Eigen::VectorXf push_vec;
                    if (!secondary_entry_topic_.empty()) {
                        if (last_secondary_src_.size() == 0) return;
                        push_vec.resize(src.size() + last_secondary_src_.size());
                        push_vec.head(src.size())                = src;
                        push_vec.tail(last_secondary_src_.size()) = last_secondary_src_;
                    } else {
                        push_vec = src;
                    }
                    entry_history_.push_back(push_vec);
                    while (int(entry_history_.size()) > entry_keyframes_)
                        entry_history_.pop_front();
                    ++entry_history_seen_;
                }));
        }
        // v5.4.M — optional secondary subscription.  Same source-choice
        // rule as the primary (latent vs winner_prototype, per the
        // entry_use_winner_prototype flag).
        if (!secondary_entry_topic_.empty()) {
            sub_ids_.push_back(bus_->subscribe(secondary_entry_topic_,
                                                SubscriptionKind::Direct,
                [this](std::string_view, MessagePtr p){
                    auto rt = std::dynamic_pointer_cast<const RealityToken>(p);
                    if (!rt) return;
                    Eigen::VectorXf const& src =
                        (entry_use_winner_prototype_ && rt->winner_prototype.size() > 0)
                            ? rt->winner_prototype
                            : rt->latent;
                    if (src.size() == 0) return;
                    last_secondary_src_ = src;
                }));
        }
    }

    // Phase 6.5.3.8 — eligibility-trace boundary clearing.  When TD(λ)
    // is enabled, subscribe to events.* and clear the trace on
    // events.failed / events.solved so episode-end δ doesn't propagate
    // backward into the next episode's fresh start.
    if (eligibility_lambda_ > 0.0f) {
        sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_event(t, p); }));
    }

    // Exploration directive (HomeokineticExploration).  Always subscribed —
    // the topic carries `active=false` when no episode is armed, so missing
    // publishers degrade gracefully (latest_exploration_ stays null →
    // override never fires).
    sub_ids_.push_back(bus_->subscribe(topics::kExplorationDirective,
                                        SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_exploration(t, p); }));
}

void ActionDecoder::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "proprio_topic" || k == "action_bins" || k == "master_seed" || k == "output_topic")
        throw std::invalid_argument("ActionDecoder param '" + k + "' is ConstructionOnly");
    else if (k == "consensus_level")          consensus_level_           = int(get_int(value, k));
    else if (k == "accel_min")                accel_min_                 = float(get_double(value, k));
    else if (k == "accel_max")                accel_max_                 = float(get_double(value, k));
    else if (k == "efe_temperature")          efe_temperature_           = float(get_double(value, k));
    else if (k == "pragmatic_gain")           pragmatic_gain_            = float(get_double(value, k));
    else if (k == "epistemic_gain")           epistemic_gain_            = float(get_double(value, k));
    else if (k == "urgency_exploit_threshold") urgency_exploit_threshold_= float(get_double(value, k));
    else if (k == "urgency_exploit_bias")     urgency_exploit_bias_      = float(get_double(value, k));
    else if (k == "td_lambda")                td_lambda_                 = float(get_double(value, k));
    else if (k == "td_gamma")                 td_gamma_                  = float(get_double(value, k));
    else if (k == "eligibility_lambda")       eligibility_lambda_        = float(get_double(value, k));
    else if (k == "eligibility_max_len")       eligibility_max_len_       = int(get_int(value, k));
    else if (k == "commit_ticks")              commit_ticks_              = int(get_int(value, k));
    else if (k == "valence_decay_pos")        valence_decay_pos_         = float(get_double(value, k));
    else if (k == "valence_decay_neg")        valence_decay_neg_         = float(get_double(value, k));
    else if (k == "valence_max_size")         valence_max_size_          = get_int(value, k);
    else if (k == "hebbian_max_size")         hebbian_max_size_          = get_int(value, k);
    else if (k == "use_rollout")              use_rollout_               = std::get<bool>(value);
    else if (k == "rollout_K")                rollout_K_                 = std::max(1, int(get_int(value, k)));
    else if (k == "rollout_M")                rollout_M_                 = std::max(1, int(get_int(value, k)));
    else if (k == "use_chunks")               use_chunks_                = std::get<bool>(value);
    else if (k == "entry_keyframes")          entry_keyframes_           = std::max(1, int(get_int(value, k)));
    else if (k == "entry_match_threshold")    entry_match_threshold_     = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "entry_use_winner_prototype") entry_use_winner_prototype_ = std::get<bool>(value);
    else if (k == "manual_chunk_id")          manual_chunk_id_           = int(get_int(value, k));
    else if (k == "min_chunk_score")          min_chunk_score_           = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "min_entry_match_product")  min_entry_match_product_   = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "chunk_rearm_threshold")    chunk_rearm_threshold_     = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "chunk_dispatch_min_age_ticks") chunk_dispatch_min_age_ticks_ = std::max(0, int(get_int(value, k)));
    else if (k == "plan_horizon")             plan_horizon_              = std::max(1, int(get_int(value, k)));
    else if (k == "plan_gamma")               plan_gamma_                = float(get_double(value, k));
    else if (k == "w_scent")                  w_scent_                   = float(get_double(value, k));
    else if (k == "w_green")                  w_green_                   = float(get_double(value, k));
    else if (k == "stay_penalty")             stay_penalty_              = float(get_double(value, k));
    else if (k == "plan_temperature")         plan_temperature_          = float(get_double(value, k));
    else
        throw std::invalid_argument("ActionDecoder: unknown param '" + k + "'");
}

void ActionDecoder::handle_consensus(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    latest_consensus_ = std::dynamic_pointer_cast<const ConsensusToken>(payload);
}
void ActionDecoder::handle_drive(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    latest_drive_ = std::dynamic_pointer_cast<const DriveErrors>(payload);
    if (latest_drive_) current_drive_urgency_ = latest_drive_->urgency;
}
void ActionDecoder::handle_neuro(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    latest_neuro_ = std::dynamic_pointer_cast<const NeuroState>(payload);
}
void ActionDecoder::handle_interest(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto g = std::dynamic_pointer_cast<const ReflexGate>(payload);
    if (g) latest_interest_ = g->active ? std::clamp(g->value, 0.0f, 1.0f) : 0.0f;
}
void ActionDecoder::handle_proprio_token(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (rt) latest_proprio_node_ = rt->winner_id;
}
void ActionDecoder::handle_authority(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0)
        latest_authority_ = std::clamp(float(pt->values[0]), 0.0f, 1.0f);
}

void ActionDecoder::handle_pref_obs(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > pref_obs_index_)
        latest_pref_obs_ = float(pt->values[pref_obs_index_]);
}
void ActionDecoder::handle_green_obs(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) latest_green_obs_ = float(pt->values[0]);
}

// ---- v1 "coxswain" planning helpers ---------------------------------------
// node_value: the target a state offers = w_scent*scent + w_green*green-loom,
// from the per-state EMAs (fall back to the current obs for unvisited states so
// the plan doesn't treat the unknown as worthless).
float ActionDecoder::node_value(int s) const {
    float sc, gr;
    auto it = obs_value_.find(s);
    sc = (it != obs_value_.end()) ? it->second : latest_pref_obs_;
    auto ig = obs_value_green_.find(s);
    gr = (ig != obs_value_green_.end()) ? ig->second : latest_green_obs_;
    return w_scent_ * sc + w_green_ * gr;
}
// predict_next: the most-likely next state after taking joint action a in s
// (argmax of the empirical forward model).  Unseen (s,a) → assume we stay put.
int ActionDecoder::predict_next(int s, int a) const {
    auto fit = forward_model_.find(fwd_key(s, a));
    if (fit == forward_model_.end() || fit->second.empty()) return s;
    int best = s, bc = -1;
    for (auto const& [s_next, c] : fit->second)
        if (c > bc) { bc = c; best = s_next; }
    return best;
}
// plan_value: best cumulative (discounted) target reachable from s within `depth`
// more steps — the receding-horizon look-ahead that lets "aim now, approach
// later" earn value (depth-H argmax tree over the joint action space).
float ActionDecoder::plan_value(int s, int depth) const {
    if (depth <= 0) return 0.0f;
    int N = n_actions();
    float best = -std::numeric_limits<float>::infinity();
    for (int a = 0; a < N; ++a) {
        int s2 = predict_next(s, a);
        float v = node_value(s2) + plan_gamma_ * plan_value(s2, depth - 1);
        if (v > best) best = v;
    }
    return (best == -std::numeric_limits<float>::infinity()) ? 0.0f : best;
}
// plan_first_action: the first joint action of the best H-step plan from s.
// argmax when plan_temperature_==0 (default, bit-identical); otherwise SAMPLE
// from softmax(scores / plan_temperature_) — persistent exploration that never
// fully decays (escapes the bootstrap-failure + facing-freeze degenerations).
int ActionDecoder::plan_first_action(int s) const {
    int N = n_actions();
    std::vector<float> scores(N);
    int best_a = 0; float best = -std::numeric_limits<float>::infinity();
    for (int a = 0; a < N; ++a) {
        int s2 = predict_next(s, a);
        // small curiosity bonus for under-tried transitions (info gain)
        auto fit = forward_model_.find(fwd_key(s, a));
        int visits = 0;
        if (fit != forward_model_.end())
            for (auto const& [sn, c] : fit->second) visits += c;
        float epi = epistemic_gain_ * (1.0f / (1.0f + float(visits)));
        // Anti-freeze: a LEARNED staying-put action (visited, predicts s'==s) is
        // the dark-room fixed point — penalize so a zero-motion action can't be
        // the argmax when a state-changing one exists.  UNSEEN (s,a) (visits==0,
        // predict_next defaults to s) is NOT penalized — it gets the epi bonus.
        float stay = (stay_penalty_ > 0.0f && visits > 0 && s2 == s) ? stay_penalty_ : 0.0f;
        float v = node_value(s2) + plan_gamma_ * plan_value(s2, plan_horizon_ - 1) + epi - stay;
        scores[a] = v;
        if (v > best) { best = v; best_a = a; }
    }
    // diag: spread of the decision landscape (≈0 ⇒ flat ⇒ uniform choice).
    { float lo = scores[0], hi = scores[0];
      for (int a = 1; a < N; ++a) { if (scores[a] < lo) lo = scores[a]; if (scores[a] > hi) hi = scores[a]; }
      last_score_spread_ = hi - lo; }
    if (plan_temperature_ <= 0.0f) {                 // argmax (default)
        last_plan_entropy_ = 0.0f; last_plan_confidence_ = 1.0f;
        return best_a;
    }
    // Softmax sample (subtract max for numerical stability).
    float Z = 0.0f;
    std::vector<float> p(N);
    for (int a = 0; a < N; ++a) { p[a] = std::exp((scores[a] - best) / plan_temperature_); Z += p[a]; }
    if (Z <= 0.0f) { last_plan_entropy_ = 0.0f; last_plan_confidence_ = 1.0f; return best_a; }
    // Explore/exploit readout: normalized softmax entropy (~1 flat/explore,
    // ~0 peaked/confident) + the chosen action's probability.
    float H = 0.0f;
    for (int a = 0; a < N; ++a) { float pa = p[a] / Z; if (pa > 1e-9f) H -= pa * std::log(pa); }
    last_plan_entropy_ = (N > 1) ? std::clamp(H / std::log(float(N)), 0.0f, 1.0f) : 0.0f;
    std::uniform_real_distribution<float> u(0.0f, Z);
    float r = u(rng_), c = 0.0f;
    for (int a = 0; a < N; ++a) {
        c += p[a];
        if (r <= c) { last_plan_confidence_ = p[a] / Z; return a; }
    }
    last_plan_confidence_ = p[best_a] / Z;
    return best_a;
}
// decode_joint: split a joint index into a turn accel (output_topic) and a
// thrust accel (thrust_output_topic).
void ActionDecoder::decode_joint(int idx, float& turn_accel, float& thrust_accel) const {
    int tb = (action_bins_ > 0) ? idx / thrust_bins_ : 0;   // turn bin
    int hb = (thrust_bins_ > 0) ? idx % thrust_bins_ : 0;   // thrust bin
    float tstep = (action_bins_ > 1) ? (accel_max_ - accel_min_) / float(action_bins_ - 1) : 0.0f;
    float hstep = (thrust_bins_ > 1) ? (thrust_accel_max_ - thrust_accel_min_) / float(thrust_bins_ - 1) : 0.0f;
    turn_accel   = accel_min_        + tstep * float(tb);
    thrust_accel = thrust_accel_min_ + hstep * float(hb);
}

void ActionDecoder::handle_rollout_result(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto r = std::dynamic_pointer_cast<const RolloutResult>(payload);
    if (!r) return;
    if (r->request_id != pending_rollout_request_) return;   // stale or unrelated
    pending_rollout_entropy_  = r->entropy;
    pending_rollout_filled_   = true;
    last_rollout_request_id_  = r->request_id;
}

void ActionDecoder::handle_motor_chunks(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto c = std::dynamic_pointer_cast<const MotorChunks>(payload);
    if (c) {
        latest_chunks_ = c;
        // v5.4.J — prune chunk_armed_ entries for chunks that no longer
        // exist in the library (MotorRepertoire hard-pruned them, or LRU
        // evicted).  Without this, the map grows unbounded over a long
        // run as chunks come and go.  Newly-arrived chunks get their
        // armed=false default when try_dispatch_chunk first encounters
        // them via the chunk_armed_.emplace(id, false) call.
        std::unordered_set<int> live_ids;
        live_ids.reserve(c->chunks.size());
        for (auto const& ch : c->chunks) live_ids.insert(ch.id);
        for (auto it = chunk_armed_.begin(); it != chunk_armed_.end(); ) {
            if (!live_ids.count(it->first)) it = chunk_armed_.erase(it);
            else ++it;
        }
    }
}

void ActionDecoder::handle_motor_stream(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto s = std::dynamic_pointer_cast<const MotorPlayStream>(payload);
    if (!s) return;
    if (s->request_id != pending_chunk_request_) return;

    // v5.3 Phase B — intent chunks take priority when present.  Exactly
    // one of intents / actions is populated per stream (MotorRepertoire
    // contract).  Queue whichever is non-empty.
    if (!s->intents.empty()) {
        intent_chunk_queue_.assign(s->intents.begin(), s->intents.end());
        chunk_queue_.clear();
        chunk_remaining_ = int(intent_chunk_queue_.size());
    } else if (!s->actions.empty()) {
        chunk_queue_.assign(s->actions.begin(), s->actions.end());
        intent_chunk_queue_.clear();
        chunk_remaining_ = int(chunk_queue_.size());
    } else {
        return;   // nothing to play
    }
    active_chunk_id_ = s->chunk_id;
    ++chunk_dispatch_count_;
    // v5.4 Phase G — capture body segmentation from the dispatched chunk
    // (looked up from latest_chunks_) so we can stamp ActionOut.chunk_position
    // each tick of replay.  Falls back to "single position spans the whole
    // chunk" when the chunk doesn't have positional bookkeeping.
    current_chunk_total_intents_  = chunk_remaining_;
    current_chunk_body_keyframes_ = 1;
    current_chunk_playback_per_   = std::max(1, chunk_remaining_);
    if (latest_chunks_) {
        for (auto const& c : latest_chunks_->chunks) {
            if (c.id == s->chunk_id) {
                if (c.body_keyframes > 0 && c.playback_ticks_per_position > 0) {
                    current_chunk_body_keyframes_ = c.body_keyframes;
                    current_chunk_playback_per_   = c.playback_ticks_per_position;
                }
                break;
            }
        }
    }
}

void ActionDecoder::handle_exploration(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const ExplorationDirective>(payload);
    if (!e) return;
    exploration_active_           = e->active;
    exploration_accel_            = e->accel;
    exploration_ticks_remaining_  = e->ticks_remaining;
}

void ActionDecoder::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    // Phase 6.5.3.8 — clear eligibility trace on episode boundaries.
    // events.failed (episode terminated by failure) and events.solved
    // (episode terminated by success) both signal that the trace is no
    // longer causally connected to upcoming TD updates.  Also clear
    // prev_state_ / prev_proprio_ / prev_action_ so the first td_update
    // of the next episode doesn't bridge the boundary.
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    auto const& name = !ev->name.empty()
                        ? ev->name
                        : std::string(topic.substr(std::min(topic.size(),
                            std::string_view("events.").size())));
    if (name == "failed" || name == "solved") {
        // Phase 6.5.3.8 — clear eligibility trace at episode boundaries so
        // TD(λ) does not bridge episode boundaries via stale traces.
        eligibility_.clear();
        // Phase 6.5.3.F — DELIBERATELY do NOT clear prev_state_/prev_action_
        // here.  The queued hit/miss pulse from the terminal event will be
        // processed by NeurochemState on the NEXT tick, and the next
        // td_update needs prev_state_ to still hold the final state-of-
        // episode so the goal-causing (or failure-causing) action gets
        // credited.  Leakage into subsequent ticks is prevented by
        // NeurochemState clearing dopamine to baseline at the end of the
        // same tick (see NeurochemState.cpp Phase 6.5.3.F block).
    }
    // v5.3 Phase D — chunk_abort: terminate any active chunk replay.
    // Drains both queues so the next tick's try_dispatch_chunk runs fresh
    // selection; safe when no chunk is playing (queues already empty).
    if (name == "chunk_abort") {
        chunk_queue_.clear();
        intent_chunk_queue_.clear();
        chunk_remaining_ = 0;
        active_chunk_id_ = -1;
    }
}

float ActionDecoder::request_epistemic_entropy(int state_node, std::string const& modality, uint64_t tick_id) {
    if (!use_rollout_ || state_node < 0 || modality.empty()) return 0.0f;

    auto q = std::make_shared<RolloutQuery>();
    q->tick_id          = tick_id;
    q->producer_id      = id_.empty() ? std::string("action_decoder") : id_;
    q->source_modality  = modality;        // GNGRollout accepts short form too
    q->winner_id        = state_node;
    q->action           = 0.0f;
    q->K_samples        = rollout_K_;
    q->M_steps          = rollout_M_;
    q->request_id       = ++next_rollout_request_id_;

    pending_rollout_request_ = q->request_id;
    pending_rollout_filled_  = false;
    pending_rollout_entropy_ = 0.0f;

    bus_->publish(topics::kRolloutQuery, q);
    // Dispatch is synchronous in InProcessBus, so the result handler has
    // already populated pending_rollout_entropy_ by the time we return.
    return pending_rollout_filled_ ? pending_rollout_entropy_ : 0.0f;
}

bool ActionDecoder::try_dispatch_chunk(int state_node, uint64_t tick_id) {
    if (!use_chunks_) return false;

    // v5.4 Phase C — manual chunk playback probe (checked BEFORE the
    // library-empty / state-node guards so misses are still counted in
    // diag, including during cold start).
    if (manual_chunk_id_ != 0) {
        if (!latest_chunks_) { ++manual_dispatch_misses_; return false; }
        bool found = false;
        for (auto const& c : latest_chunks_->chunks) {
            if (c.id == manual_chunk_id_) { found = true; break; }
        }
        if (!found) {
            ++manual_dispatch_misses_;
            return false;
        }
        auto cmd = std::make_shared<MotorPlayCmd>();
        cmd->tick_id     = tick_id;
        cmd->producer_id = id_.empty() ? std::string("action_decoder") : id_;
        cmd->chunk_id    = manual_chunk_id_;
        cmd->request_id  = ++next_chunk_request_id_;
        cmd->force       = true;   // bypass MotorRepertoire's is_active gate
        pending_chunk_request_ = cmd->request_id;
        bus_->publish(topics::kMotorPlayCmd, cmd);
        if (chunk_remaining_ > 0) {
            ++manual_dispatches_;
            return true;
        }
        ++manual_dispatch_misses_;
        return false;
    }

    if (!latest_chunks_ || latest_chunks_->chunks.empty()) return false;

    if (state_node < 0) return false;

    // State-conditional dispatch: a chunk fires only when the current
    // baked consensus motif matches the chunk's trigger_consensus_motif_id
    // stamped at hit time.  Wildcards (trigger_id == -1) match anything.
    //
    // Among matching chunks, pick the one with the highest Beta(1,1)-prior
    // success rate over its lifetime hits / misses.  This is the SAME
    // metric MotorRepertoire uses for LRU eviction, so a chunk's standing
    // in dispatch ranking and in eviction risk move together.  A chunk
    // whose replays start to miss more than hit drops in score and is
    // both less likely to fire and more likely to be evicted — the
    // post-crystallization demotion path closes here.
    auto chunk_score = [](MotorChunk const& c) -> float {
        // v5.4 Phase G (Proposal C) — when per-position credit is
        // populated (episodic chunk crystallised with body_keyframes>0),
        // chunk score = MEAN of per-position Beta(1,1) priors.  Each
        // position's score = (hits[k]+1)/(hits[k]+misses[k]+2).  Chunks
        // with at least one productive position get partial credit;
        // chunks where ALL positions are stale stay at 0.5.  Falls back
        // to chunk-wide Beta when positional data isn't populated.
        if (!c.position_hits.empty()
            && c.position_hits.size() == c.position_misses.size()) {
            float sum = 0.0f;
            int   n   = int(c.position_hits.size());
            for (int k = 0; k < n; ++k) {
                float h = float(c.position_hits[k]) + 1.0f;
                float t = h + float(c.position_misses[k]) + 1.0f;
                sum += h / t;
            }
            // Blend per-position score with chunk-wide prior so freshly-
            // crystallised chunks (all positions zero) inherit the
            // boot-Beta of 0.6 instead of starting at flat 0.5.
            float per_pos_mean = sum / float(n);
            float chunk_wide   = (float(c.hits_during + c.replay_hits) + 1.0f)
                                 / (float(c.hits_during + c.replay_hits + c.replay_misses) + 2.0f);
            return 0.5f * per_pos_mean + 0.5f * chunk_wide;
        }
        float h = float(c.hits_during + c.replay_hits) + 1.0f;
        float t = h + float(c.replay_misses) + 1.0f;
        return h / t;
    };
    int   best_id     = 0;     // arbitrary; gated by best_id_valid below
    bool  best_id_valid = false;
    float best_score  = -std::numeric_limits<float>::infinity();
    bool  best_via_entry_match = false;
    float best_entry_match_product = 1.0f;   // for Proposal A min_entry_match_product gate
    // v5.4 Phase A — entry-match dispatch.  Episodic chunks (those with
    // non-empty entry_embeddings) are gated by cosine similarity of the
    // brain's entry_history_ to chunk.entry_embeddings — BOTH (or all K)
    // positions must clear entry_match_threshold_.  When an entry-match
    // chunk dispatches, its score = product of cosines × Beta-prior.
    // When entry_topic_ is empty OR entry_history_ insufficient, episodic
    // chunks fall through to legacy Beta-only behaviour (any entry chunk
    // can dispatch).
    bool entry_history_ready = !entry_topic_.empty()
        && int(entry_history_.size()) >= entry_keyframes_;
    // Phase 6.5.12 — hybrid trigger.  A chunk matches when EITHER its
    // consensus context recurs (existing) OR the substrate is in a drive
    // state at-or-beyond what the chunk crystallized in.  Drive predicate
    // uses 0.95× tolerance to widen-permissively; chunks with the
    // legacy default (-1) skip the drive check entirely.  Beta-score
    // ranking + Wilson-CI demotion (post-dispatch) handle low-quality
    // dispatches the same way regardless of which trigger fired.
    constexpr float kDriveTriggerTolerance = 0.95f;
    for (auto const& c : latest_chunks_->chunks) {
        // v5.4.L — age gate.  Applied BEFORE entry-match computation so we
        // don't waste cosine ops on chunks that aren't dispatch-eligible
        // anyway.  Compares (current_tick - chunk.created_tick_id) against
        // chunk_dispatch_min_age_ticks_.  Seeded chunks (created_tick_id=0)
        // pass automatically as long as tick_id ≥ age threshold — i.e.,
        // after the first sim-second.  Manual probes already bypassed
        // before reaching this loop.
        if (chunk_dispatch_min_age_ticks_ > 0
            && tick_id >= c.created_tick_id
            && (tick_id - c.created_tick_id) < uint64_t(chunk_dispatch_min_age_ticks_)) {
            ++dispatches_blocked_too_young_;
            continue;
        }
        bool consensus_match = (c.trigger_consensus_motif_id < 0)  // wildcard
                            || (c.trigger_consensus_motif_id == current_consensus_motif_id_);
        bool drive_match     = (c.trigger_urgency >= 0.0f)
                            && (current_drive_urgency_ >= c.trigger_urgency * kDriveTriggerTolerance);
        // v5.4 Phase A — episodic chunk entry-match gate.  When the chunk
        // carries entry_embeddings, it bypasses the legacy consensus_match
        // / drive_match predicates (those are for v5.3 Beta-only chunks)
        // and instead requires entry-history cosine similarity ≥ threshold
        // at every entry position.  Then the chunk score is the product
        // of cosines × Beta-prior.  Episodic chunks dispatch ONLY when
        // entry_history_ready (otherwise the brain has no recent state
        // memory to match against — wait until enough keyframes have
        // accumulated).
        bool is_episodic = !c.entry_embeddings.empty();
        if (is_episodic) {
            if (!entry_history_ready) continue;
            if (int(c.entry_embeddings.size()) != entry_keyframes_) continue;
            // v5.4.J — compute min cosine across positions FIRST so we can
            // arm/disarm the chunk's Schmitt-trigger gate regardless of
            // whether the per-position threshold passed.  When match drops
            // below chunk_rearm_threshold_, the chunk re-arms; the
            // armed-gate is then checked against the higher
            // entry_match_threshold_ for dispatch.  Pre-fix the chunk's
            // entry-match was only computed when ALL positions cleared the
            // dispatch threshold — so a chunk just captured at the
            // current context (cosine ≈ 1) sailed through to dispatch on
            // the very next tick.
            float min_cs = 1.0f;
            float entry_match = 1.0f;     // product of cosines
            bool  all_above   = true;
            bool  computed_all = true;
            for (int k = 0; k < entry_keyframes_; ++k) {
                auto const& a = entry_history_[k];
                auto const& b = c.entry_embeddings[k];
                if (a.size() != b.size() || a.size() == 0) {
                    computed_all = false; all_above = false; break;
                }
                float na = a.norm(), nb = b.norm();
                if (na < 1e-6f || nb < 1e-6f) {
                    computed_all = false; all_above = false; break;
                }
                float cs = a.dot(b) / (na * nb);
                entry_match *= cs;
                if (cs < min_cs) min_cs = cs;
                if (cs < entry_match_threshold_) all_above = false;
            }
            // Schmitt-trigger arm.  Insert chunk into chunk_armed_ if
            // unseen so newly-arrived episodic chunks default to armed=false
            // — they must SEE their entry context leave before they fire.
            auto armed_it = chunk_armed_.emplace(c.id, false).first;
            if (computed_all && min_cs < chunk_rearm_threshold_) {
                armed_it->second = true;     // re-armed: context has left
            }
            if (!all_above) continue;
            if (!armed_it->second) {
                // Match above dispatch threshold but chunk hasn't re-armed
                // since last fire / since capture.  Treat as a deliberate
                // skip (not a hard rejection — the chunk just isn't ready
                // yet) and count it for diag.
                ++dispatches_blocked_unarmed_;
                continue;
            }
            // Snapshot the per-chunk entry_match product for the Proposal-A
            // gate later; entry_match was multiplied position-by-position
            // above so it's the final product across all entry positions.
            float this_entry_match = entry_match;
            // Entry-match is the GATE; chunk_score is the SCORE.
            // Multiplying score×entry_match was pushing fresh episodic
            // chunks (Beta=0.667) × even a tight 0.7-product match to
            // ~0.47 — below the 0.5 commit threshold → 99% of valid
            // entry-matches were silently rejected by the score gate.
            // Now: matched chunks compete on Beta-prior alone.  Tied
            // chunks resolve via the same per-tick noise tiebreak as
            // legacy chunks below.
            uint64_t emix = uint64_t(tick_id) * 2654435761ull
                          ^ uint64_t(c.id < 0 ? -c.id : c.id) * 40503ull
                          ^ 0xE0F1ull;
            float enoise = (float((emix * 0x100000001b3ull) >> 32)
                           / float(uint32_t(-1))) * 2.0f - 1.0f;
            float s = chunk_score(c) + 1e-4f * enoise;
            if (s > best_score) {
                best_score    = s;
                best_id       = c.id;
                best_id_valid = true;
                best_via_entry_match = true;
                best_entry_match_product = this_entry_match;
            }
            continue;   // skip legacy predicates for episodic chunks
        }

        if (!consensus_match && !drive_match) continue;
        float s = chunk_score(c);
        // v5.3 Phase F — tick-hash tiebreak.  When multiple chunks tie on
        // Beta(1,1) score, the previous std::unordered_map iteration order
        // picked one deterministically and stuck with it forever (chunk
        // lockup observed at α=1.0 brain-only with 4 tied seed chunks).
        // Add a tiny per-(tick, chunk) noise (deterministic, seed-stable)
        // so tied chunks rotate across ticks; preserves seed determinism.
        // Magnitude (~1e-4) is small enough not to override real score
        // differences but enough to break ties.
        uint64_t mix = uint64_t(tick_id) * 2654435761ull
                     ^ uint64_t(c.id < 0 ? -c.id : c.id) * 40503ull
                     ^ (c.id < 0 ? 0xA5A5ull : 0ull);
        // Hash to [0, 1) then to [-eps, +eps] with eps=1e-4.
        float noise = (float((mix * 0x100000001b3ull) >> 32) / float(uint32_t(-1))) * 2.0f - 1.0f;
        s += 1e-4f * noise;
        if (s > best_score) {
            best_score    = s;
            best_id       = c.id;
            best_id_valid = true;
        }
    }
    // Beta(1,1) prior puts new chunks at 0.5; only dispatch when the best
    // matching chunk is at least break-even (replay hits ≥ misses).
    // v5.3 Phase E: track validity separately so seeded chunks (negative
    // ids) can dispatch — pre-fix `best_id < 0` rejected any seed chunk.
    if (!best_id_valid) return false;
    // v5.4 Phase E (Proposal A) — quality gates.  min_chunk_score
    // replaces the legacy hard 0.5 commit threshold (default = 0.5 to
    // preserve legacy behaviour).  min_entry_match_product is a
    // separate gate applied only to episodic chunks (default = 0 = off).
    if (best_score < min_chunk_score_) {
        ++dispatches_gated_score_;
        return false;
    }
    if (best_via_entry_match && best_entry_match_product < min_entry_match_product_) {
        ++dispatches_gated_match_;
        return false;
    }
    if (best_via_entry_match) ++entry_match_dispatches_;
    // v5.4 Phase F (Proposal B) — stash quality of dispatched chunk for
    // FaderController to read via ActionOut.chunk_quality.
    current_chunk_quality_ = best_score;
    // v5.4.J — disarm the dispatched chunk so it can't immediately re-fire
    // on the next tick (still-matching context).  It must wait for the
    // agent to leave the entry context (match drops below
    // chunk_rearm_threshold_) before it's eligible again.  Only episodic
    // chunks have armed-state entries; seeded chunks (no entry_embeddings)
    // never inserted into chunk_armed_ so this is a no-op for them.
    if (best_via_entry_match) {
        auto armed_it = chunk_armed_.find(best_id);
        if (armed_it != chunk_armed_.end()) armed_it->second = false;
    }

    auto cmd = std::make_shared<MotorPlayCmd>();
    cmd->tick_id     = tick_id;
    cmd->producer_id = id_.empty() ? std::string("action_decoder") : id_;
    cmd->chunk_id    = best_id;
    cmd->request_id  = ++next_chunk_request_id_;
    pending_chunk_request_ = cmd->request_id;
    bus_->publish(topics::kMotorPlayCmd, cmd);
    // Synchronous; handle_motor_stream populated chunk_queue_ if available.
    return chunk_remaining_ > 0;
}

std::pair<int,int> ActionDecoder::resolve_state() const {
    bool have_cons = (consensus_level_ >= 0);
    bool have_prop = !proprio_topic_.empty();
    int cons = have_cons ? (latest_consensus_ ? latest_consensus_->active_winner_id : -1) : -1;
    int prop = have_prop ? latest_proprio_node_ : -1;
    if (have_cons && have_prop) return { cons, prop };                 // legacy joint
    if (!have_cons)             return { prop, prop >= 0 ? 0 : -1 };    // proprio-only (bundle)
    return { cons, cons >= 0 ? 0 : -1 };                               // consensus-only (voter)
}

float ActionDecoder::select_action(int state_node, int proprio_node, std::string const& modality) {
    // Phase 6.5.2.2 — Q-axis on valence_.
    //
    // Was: V-table keyed on (state, proprio).  All bins shared the same
    //      `v` and the score was `pragmatic_gain * v * tanh(a)` — a sign
    //      bias on actions, not a state-action policy.  CartPole falsified
    //      this (docs/v4_phase6_5_2_plan.md §9): with dense reward, every
    //      visited (s, p) had v > 0 → max-positive a always won.
    // Now: Q-table keyed on (state, proprio, bin).  Each bin has its own
    //      learned value; `select_action` argmaxes across bins.  TD update
    //      writes only the bin that was actually taken (see td_update).
    //      Cold-start q is 0 for all bins → ties → min-|a| tiebreak picks
    //      the safe centre bin (or accel_min for symmetric 2-bin
    //      configurations) deterministically.
    //
    // The Hebbian fast-path read is intentionally removed: with a real
    // per-action Q lookup, the fast-path would just lock in cold-start
    // choices (the diagnostic from §9.1).  The Hebbian write is preserved
    // (td_update) for backward compat with the test contract and as a
    // future hook — a downstream consumer could re-introduce it when there
    // is a meaningful reward-conditioning at the write site.

    float gain = pragmatic_gain_;
    if (latest_drive_ && latest_drive_->urgency > urgency_exploit_threshold_)
        gain *= urgency_exploit_bias_;

    // Phase 6.5.3.1 — per-bin epistemic from the empirical forward
    // model.  Was: bin-independent rollout entropy that shifted every
    // bin's score equally → couldn't differentiate exploration targets.
    // Now: for each bin b, count visits to (state, b) in
    // forward_model_; bonus = 1 / (1 + visits).  Under-visited bins
    // get a high exploration prior; well-visited bins drop to ~0.
    // GNGRollout still queried for back-compat side-effects on
    // request_id bookkeeping, but its return is no longer used.
    if (use_rollout_) {
        uint64_t tid = latest_consensus_ ? latest_consensus_->tick_id : 0;
        (void)request_epistemic_entropy(state_node, modality, tid);
    }

    // Score each bin from its Q value plus the per-bin epistemic
    // bonus.  ε-greedy exploration with ε derived from serotonin:
    //   high serotonin (stable, satisfied)         → low ε → exploit
    //   low serotonin (recent failures stressed)   → high ε → explore
    // This is the adaptive-mechanism replacement for static ε: the
    // substrate already produces a stress signal via NeurochemState's
    // event-coupled ht_miss_drop, so we feed it back into the policy
    // without adding hyperparameters beyond a [min, max] safety clamp.
    // Softmax was tried first (efe_temperature_ scaled by 1/serotonin)
    // but with bootstrapping Q values saturate and the relative score
    // gap exceeds any reasonable T, collapsing softmax to argmax.
    // ε-greedy preserves exploration regardless of Q magnitude.
    float bin_step = (accel_max_ - accel_min_) / float(action_bins_ - 1);
    std::vector<float> scores(action_bins_);
    std::vector<float> accels(action_bins_);
    // 2026-06-18 — EFE mode (opt-in): curiosity-modulated epistemic weight + a
    // cold-transition entropy prior.  Legacy mode is the original value-RL path.
    float epi_scale = epistemic_gain_ * (0.5f + latest_interest_);
    float max_ent   = std::log(float(action_bins_) + 1e-6f);
    for (int b = 0; b < action_bins_; ++b) {
        accels[b] = accel_min_ + bin_step * float(b);
        auto fit = forward_model_.find(fwd_key(state_node, b));
        if (efe_select_) {
            // EXPECTED FREE ENERGY over the learned action-conditioned model
            // P(s'|state,b): pragmatic = E[V(s')] (predicted value of where b
            // leads → act toward preferred/fed states); epistemic = H(s'|state,b)
            // (info gain → resolve uncertain transitions).  No ε-greedy below:
            // the epistemic term IS the principled, interest-driven exploration.
            float pragmatic, epistemic;
            if (fit != forward_model_.end() && !fit->second.empty()) {
                int total = 0;
                for (auto const& [s, c] : fit->second) total += c;
                float ev = 0.0f, ent = 0.0f;
                for (auto const& [s_next, c] : fit->second) {
                    float p = float(c) / float(total);
                    float node_val;
                    if (!pref_obs_topic_.empty()) {
                        // GROUNDED: prefer next-states whose expected PREFERRED
                        // OBSERVATION (EMA scent proximity) is high — act to fulfil
                        // the preference over observations (the spark).  Cold s' =
                        // assume the current obs (don't penalise the unknown).
                        auto oit = obs_value_.find(s_next);
                        node_val = (oit != obs_value_.end()) ? oit->second : latest_pref_obs_;
                    } else {
                        // legacy: E[V] = max_b' Q(s', proprio, b')
                        float vmax = 0.0f; bool any = false;
                        for (int bb = 0; bb < action_bins_; ++bb) {
                            auto vit = valence_.find(val_key(s_next, proprio_node, bb));
                            if (vit == valence_.end()) continue;
                            if (!any || vit->second > vmax) { vmax = vit->second; any = true; }
                        }
                        node_val = vmax;
                    }
                    ev  += p * node_val;
                    ent += -p * std::log(p + 1e-6f);
                }
                pragmatic = ev;
                epistemic = ent;
            } else {
                // cold transition: no model yet → max info gain (explore).  Pragmatic
                // falls back to the current state's preferred-obs estimate (grounded)
                // or the cached Q (legacy).
                if (!pref_obs_topic_.empty()) {
                    auto oit = obs_value_.find(state_node);
                    pragmatic = (oit != obs_value_.end()) ? oit->second : latest_pref_obs_;
                } else {
                    auto qit = valence_.find(val_key(state_node, proprio_node, b));
                    pragmatic = (qit != valence_.end()) ? qit->second : 0.0f;
                }
                epistemic = max_ent;
            }
            scores[b] = gain * pragmatic + epi_scale * epistemic;
        } else {
            // Legacy value-RL (unchanged): TD value + count-based explore bonus.
            auto qit = valence_.find(val_key(state_node, proprio_node, b));
            float q  = (qit != valence_.end()) ? qit->second : 0.0f;
            int visits = 0;
            if (fit != forward_model_.end()) {
                for (auto const& [s, c] : fit->second) visits += c;
            }
            float bin_epistemic = 1.0f / (1.0f + float(visits));
            scores[b] = gain * q + epistemic_gain_ * bin_epistemic;
        }
    }

    // ε-greedy is gated on Q-mode (td_gamma_ > 0): when bootstrapping is
    // off, the substrate keeps its V-table semantics with deterministic
    // argmax — preserves Cell-environment behaviour byte-identical to
    // pre-Q-axis runs.  When bootstrapping is on, ε-greedy with
    // serotonin-driven ε replaces argmax.  ε capped at 0.30 so the
    // policy has enough committed exploitation to consolidate learning
    // even under steady-state stress (low ht).
    int chosen;
    int   best    = 0;
    float best_abs = std::abs(accels[0]);
    for (int b = 1; b < action_bins_; ++b) {
        bool better = (scores[b] > scores[best])
                   || (scores[b] == scores[best] && std::abs(accels[b]) < best_abs);
        if (better) {
            best     = b;
            best_abs = std::abs(accels[b]);
        }
    }
    last_greedy_accel_ = accels[best];   // learned policy (pre-ε) for diag
    if (td_gamma_ > 0.0f && !efe_select_) {   // EFE mode: epistemic term replaces ε-greedy
        float ht      = latest_neuro_ ? latest_neuro_->serotonin : 0.5f;
        float epsilon = std::clamp(1.0f - ht, 0.05f, 0.30f);
        std::uniform_real_distribution<float> coin(0.0f, 1.0f);
        if (coin(rng_) < epsilon) {
            std::uniform_int_distribution<int> idist(0, action_bins_ - 1);
            chosen = idist(rng_);
        } else {
            chosen = best;
        }
    } else {
        chosen = best;
    }
    return std::clamp(accels[chosen], accel_min_, accel_max_);
}

void ActionDecoder::td_update(float reward_signal) {
    if (prev_state_ < 0 || prev_proprio_ < 0) return;

    // Phase 6.5.2.2 — Q-axis on valence_.  TD update writes only the bin
    // that was actually taken last tick.  Other bins for this (state,
    // proprio) keep their prior Q values, so select_action's argmax can
    // compare an updated bin against unchanged neighbours.  This is what
    // makes revision possible — the V-table version updated a single
    // value at (s, p) that biased every bin's score equally, so reward
    // could not differentiate actions.
    //
    // Snap prev_action_ → bin index.  Sources of prev_action_ that aren't
    // exactly bin_to_accel(b) (chunks, exploration directives, Hebbian
    // legacy) get attributed to their closest bin so the Q-table still
    // accumulates evidence for those choices.
    int prev_bin = 0;
    if (action_bins_ > 1) {
        float bin_step = (accel_max_ - accel_min_) / float(action_bins_ - 1);
        if (bin_step > 0.0f) {
            int b = int(std::round((prev_action_ - accel_min_) / bin_step));
            prev_bin = std::clamp(b, 0, action_bins_ - 1);
        }
    }

    // Phase 6.5.3.1 — empirical forward model T[(s, a)] → P(s').
    // Compute action_tle BEFORE updating the model: 1 - P_old(s' | prev_state,
    // prev_bin), so the published TLE measures how surprised the model was
    // by the realized transition (not how surprised it would be after
    // assimilating it).  Range [0, 1].  Mirror of EPM TLE on the action
    // side; closes the substrate's TLE asymmetry and gives downstream
    // consumers (NeurochemState, HomeokineticExploration, voter) a real
    // action-side prediction-error signal to gate on.
    {
        int s_next = resolve_state().first;   // Phase 6.9 — honours proprio/consensus-only modes
        if (s_next >= 0) {
            auto fk = fwd_key(prev_state_, prev_bin);
            auto fit = forward_model_.find(fk);
            float p_old = 0.0f;
            if (fit != forward_model_.end()) {
                int total = 0;
                for (auto const& [s, c] : fit->second) total += c;
                if (total > 0) {
                    auto sit = fit->second.find(s_next);
                    int hits = (sit != fit->second.end()) ? sit->second : 0;
                    p_old = float(hits) / float(total);
                }
            }
            last_action_tle_ = 1.0f - p_old;
            // Now update counts.
            forward_model_[fk][s_next] += 1;
            // LRU evict if oversize: drop the (state, bin) key whose total
            // visit count is smallest.
            if (int64_t(forward_model_.size()) > forward_model_max_size_) {
                auto worst = forward_model_.begin();
                int worst_total = std::numeric_limits<int>::max();
                for (auto it = forward_model_.begin(); it != forward_model_.end(); ++it) {
                    int t = 0;
                    for (auto const& [s, c] : it->second) t += c;
                    if (t < worst_total) { worst_total = t; worst = it; }
                }
                forward_model_.erase(worst);
            }
        }
    }

    auto vk = val_key(prev_state_, prev_proprio_, prev_bin);
    float& v = valence_[vk];
    float v_old = v;

    // TD target.  Without bootstrapping (td_gamma_ == 0) the target is the
    // immediate reward — backwards-compatible with V-table behaviour.  With
    // td_gamma_ > 0 the target is r + γ * max_a' Q(s', a') so the failure
    // signal at episode-end states propagates backward through visited
    // (state, action) pairs, distinguishing "this action led to a state
    // with low return potential" from "this action led to a state with
    // high return potential."  Without bootstrapping, dense per-tick
    // reward (CartPole) is uninformative for action attribution.
    float td_target = reward_signal;
    if (td_gamma_ > 0.0f) {
        auto [s_next, p_next] = resolve_state();   // Phase 6.9 — bootstrap next-state
        if (s_next >= 0 && p_next >= 0) {
            float max_q_next = 0.0f;
            bool  any_q      = false;
            for (int b = 0; b < action_bins_; ++b) {
                auto it = valence_.find(val_key(s_next, p_next, b));
                if (it == valence_.end()) continue;
                if (!any_q || it->second > max_q_next) {
                    max_q_next = it->second;
                    any_q      = true;
                }
            }
            td_target = reward_signal + td_gamma_ * max_q_next;
        }
    }
    // 1-step TD update on the most-recent (prev_state, prev_proprio, prev_bin):
    // v ← v + α·δ where δ = target − v_old, α = td_lambda_ (smoothing).  This
    // is identical in effect to v = (1−α)·v + α·target.
    float delta = td_target - v_old;
    v = v_old + td_lambda_ * delta;

    // Phase 6.5.3.8 — TD(λ) eligibility-trace propagation.  When
    // eligibility_lambda_ > 0, propagate δ backward through the most-
    // recent (s, a) trail in eligibility_, with weight (γλ)^age.  Each
    // trace entry's Q gets nudged by α·(γλ)^age·δ — accelerating credit
    // assignment over multi-step trajectories without requiring a full
    // n-step return computation.  Standard TD(λ); λ=0 → pure TD(0)
    // (current behavior); λ=1 → Monte Carlo-style backward pass over
    // the entire trail.
    //
    // eligibility_ stores entries pushed at the END of each prior tick,
    // so back() = the (state, proprio, action) used at tick T-1 (which
    // IS the prev_* tuple just updated above with weight 1).  Walk back
    // from the second-to-last entry; weight starts at γλ for age=1.
    if (eligibility_lambda_ > 0.0f && eligibility_.size() >= 2) {
        float decay   = td_gamma_ * eligibility_lambda_;
        float weight  = decay;
        float bin_step = (action_bins_ > 1)
                       ? (accel_max_ - accel_min_) / float(action_bins_ - 1)
                       : 0.0f;
        for (int i = int(eligibility_.size()) - 2;
             i >= 0 && weight > 1e-3f; --i) {
            auto const& tr = eligibility_[i];
            if (tr.state < 0 || tr.proprio < 0) { weight *= decay; continue; }
            int trace_bin = 0;
            if (bin_step > 0.0f) {
                int b = int(std::round((tr.action - accel_min_) / bin_step));
                trace_bin = std::clamp(b, 0, action_bins_ - 1);
            }
            auto k = val_key(tr.state, tr.proprio, trace_bin);
            valence_[k] += td_lambda_ * weight * delta;
            weight *= decay;
        }
    }

    // Decay pass over the valence map (asymmetric).
    for (auto& [k, val] : valence_)
        val *= (val >= 0.0f ? valence_decay_pos_ : valence_decay_neg_);

    // LRU evict if oversize.
    if (int64_t(valence_.size()) > valence_max_size_) {
        // Drop the lowest-magnitude entry.
        auto worst = valence_.begin();
        for (auto it = valence_.begin(); it != valence_.end(); ++it)
            if (std::abs(it->second) < std::abs(worst->second)) worst = it;
        valence_.erase(worst);
    }

    // Record (modality, prev, cur, proprio) → action in the Hebbian table.
    if (!prev_modality_.empty() && prev_state_ >= 0) {
        HebbKey hk = hebb_key(prev_modality_, prev_state_, resolve_state().first, prev_proprio_);
        float& bias = hebbian_[hk];
        // Phase 6.5.2-followup: TD-residual ("advantage") weighting on the
        // Hebbian write.  Was: `bias = 0.7*bias + 0.3*prev_action_`
        // (reward-blind average → cements whatever cold-start bias
        // select_action emitted).  Now: imprint prev_action when the
        // observed reward EXCEEDED the prior valence estimate (positive
        // surprise → that action was better than expected); anti-imprint
        // when it FELL SHORT (negative surprise → worse than expected).
        // Raw reward sign would saturate positive in dense-reward tasks
        // (CartPole: every tick alive → reward ≈ 0.8 always); the residual
        // ≈ 0 at steady state and ≈ −1 only on failure ticks, giving the
        // failure tick proper credit-assignment weight.  This is the
        // sign-of-advantage policy gradient applied to the existing
        // substrate machinery.  See docs/v4_phase6_5_2_plan.md §9.
        float td_residual = reward_signal - v_old;
        float r_sign = (td_residual >  1e-3f) ?  1.0f
                     : (td_residual < -1e-3f) ? -1.0f
                                              :  0.0f;
        bias = 0.7f * bias + 0.3f * r_sign * prev_action_;
        if (int64_t(hebbian_.size()) > hebbian_max_size_) {
            auto worst = hebbian_.begin();
            for (auto it = hebbian_.begin(); it != hebbian_.end(); ++it)
                if (std::abs(it->second) < std::abs(worst->second)) worst = it;
            hebbian_.erase(worst);
        }
    }
}

void ActionDecoder::td_update_option(float reward_integrated, int s_end, int p_end) {
    // SMDP update for one completed K-tick option: treat the option as a single
    // macro-step.  Q(s0, a0) <- R_integrated + td_gamma * max_a' Q(s_end, a').
    if (commit_start_state_ < 0 || commit_start_proprio_ < 0) return;
    int bin = 0;
    if (action_bins_ > 1) {
        float bin_step = (accel_max_ - accel_min_) / float(action_bins_ - 1);
        if (bin_step > 0.0f)
            bin = std::clamp(int(std::round((commit_accel_ - accel_min_) / bin_step)),
                             0, action_bins_ - 1);
    }
    // Empirical forward model at OPTION granularity → drives the per-bin epistemic
    // exploration bonus in select_action and the action_tle telemetry.
    if (s_end >= 0) {
        auto fk = fwd_key(commit_start_state_, bin);
        auto fit = forward_model_.find(fk);
        float p_old = 0.0f;
        if (fit != forward_model_.end()) {
            int total = 0; for (auto const& [s, c] : fit->second) total += c;
            if (total > 0) {
                auto sit = fit->second.find(s_end);
                p_old = float(sit != fit->second.end() ? sit->second : 0) / float(total);
            }
        }
        last_action_tle_ = 1.0f - p_old;
        forward_model_[fk][s_end] += 1;
    }
    float max_q_next = 0.0f; bool any_q = false;
    if (td_gamma_ > 0.0f && s_end >= 0 && p_end >= 0) {
        for (int b = 0; b < action_bins_; ++b) {
            auto it = valence_.find(val_key(s_end, p_end, b));
            if (it == valence_.end()) continue;
            if (!any_q || it->second > max_q_next) { max_q_next = it->second; any_q = true; }
        }
    }
    float target = reward_integrated + (any_q ? td_gamma_ * max_q_next : 0.0f);
    auto vk = val_key(commit_start_state_, commit_start_proprio_, bin);
    float& v = valence_[vk];
    v += td_lambda_ * (target - v);
    // Asymmetric decay + LRU evict (mirror td_update housekeeping).
    for (auto& [kk, val] : valence_)
        val *= (val >= 0.0f ? valence_decay_pos_ : valence_decay_neg_);
    if (int64_t(valence_.size()) > valence_max_size_) {
        auto worst = valence_.begin();
        for (auto it = valence_.begin(); it != valence_.end(); ++it)
            if (std::abs(it->second) < std::abs(worst->second)) worst = it;
        valence_.erase(worst);
    }
}

void ActionDecoder::tick(uint64_t tick_id) {
    // Read most recent inputs (current-tick last_value pulls — useful when
    // the producer hasn't fired Direct yet at this point in the level).
    if (!latest_consensus_) {
        if (auto p = std::dynamic_pointer_cast<const ConsensusToken>(
                bus_->last_value(std::string("consensus.") + std::to_string(consensus_level_))))
            latest_consensus_ = p;
    }
    if (!latest_drive_) {
        if (auto p = std::dynamic_pointer_cast<const DriveErrors>(
                bus_->last_value(topics::kDriveErrors)))
            latest_drive_ = p;
    }
    if (!latest_neuro_) {
        if (auto p = std::dynamic_pointer_cast<const NeuroState>(
                bus_->last_value(topics::kNeuroState)))
            latest_neuro_ = p;
    }

    auto [state_node, proprio_node] = resolve_state();   // Phase 6.9 — proprio/consensus-only aware
    std::string modality = latest_consensus_ ? latest_consensus_->active_modality : "";

    // 2026-06-18 — grounded pragmatic: track a per-state EMA of the preferred
    // observation (e.g. scent proximity) so the EFE pragmatic term can prefer
    // next-states that historically smell stronger (preference over observations).
    if (!pref_obs_topic_.empty() && state_node >= 0) {
        auto& ov = obs_value_[state_node];
        ov += obs_value_alpha_ * (latest_pref_obs_ - ov);
    }
    if (!green_obs_topic_.empty() && state_node >= 0) {
        auto& gv = obs_value_green_[state_node];
        gv += obs_value_alpha_ * (latest_green_obs_ - gv);
    }

    // ---- v1 COXSWAIN: joint (turn,thrust) action + receding-horizon plan ------
    // One actor plans an H-step (turn,thrust) sequence over the option-level
    // forward model (each step = one commit window = "a stroke"), commits the
    // first action's strokes, then re-plans — burst-and-coast.  Publishes BOTH
    // cog.steer and cog.thrust from one decision.  Gated on joint_action_ so the
    // legacy 1-D path below is byte-identical when off.
    if (joint_action_ && commit_ticks_ > 1) {
        if (commit_remaining_ <= 0) {
            // Learn the realized option transition (start_state --action--> now),
            // then plan a fresh joint action from the current state.  CREDIT-BY-
            // AUTHORITY: when an authority signal is wired, count this transition
            // only in proportion to how much THIS actor drove the body over the
            // option (probabilistic) — so it never learns from motion caused by
            // other bus channels (the mix / a ducked or masked channel).
            bool credit = true;
            if (!authority_topic_.empty()) {
                std::uniform_real_distribution<float> u(0.0f, 1.0f);
                credit = (u(rng_) < latest_authority_);
            }
            if (credit && commit_active_ && commit_start_state_ >= 0 && commit_action_idx_ >= 0 && state_node >= 0)
                forward_model_[fwd_key(commit_start_state_, commit_action_idx_)][state_node] += 1;
            commit_action_idx_  = (state_node >= 0) ? plan_first_action(state_node) : -1;
            commit_start_state_ = state_node;
            commit_active_      = (state_node >= 0);
            commit_remaining_   = commit_ticks_;
        }
        commit_remaining_ -= 1;
        float turn_a = 0.0f, thrust_a = 0.0f;
        if (commit_action_idx_ >= 0) decode_joint(commit_action_idx_, turn_a, thrust_a);
        last_greedy_accel_ = turn_a;   // diag: the steering the plan chose
        auto out = std::make_shared<ActionOut>();
        out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("action_decoder") : id_;
        out->accel = turn_a; out->action_tle = last_action_tle_;
        bus_->publish(output_topic_, out);
        if (!thrust_output_topic_.empty()) {
            auto ot = std::make_shared<ActionOut>();
            ot->tick_id = tick_id; ot->producer_id = out->producer_id;
            ot->accel = thrust_a; ot->action_tle = last_action_tle_;
            bus_->publish(thrust_output_topic_, ot);
        }
        return;
    }

    // ---- Mode-2 temporal abstraction (commit_ticks_ > 1) ----------------------
    // Hold the selected action for K ticks; learn from the reward INTEGRATED over
    // the commitment (SMDP).  Bypasses the per-tick TD + chunk/exploration path.
    if (commit_ticks_ > 1) {
        float r = latest_neuro_ ? latest_neuro_->reward_signal : 0.0f;
        commit_reward_accum_ += r;
        if (commit_remaining_ <= 0) {
            // Close the previous option, then open a new one from the current state.
            if (commit_active_)
                td_update_option(commit_reward_accum_, state_node, proprio_node);
            commit_accel_ = (state_node >= 0 && proprio_node >= 0)
                          ? select_action(state_node, proprio_node, modality)
                          : 0.0f;
            commit_start_state_    = state_node;
            commit_start_proprio_  = proprio_node;
            commit_start_modality_ = modality;
            commit_active_         = (state_node >= 0 && proprio_node >= 0);
            commit_reward_accum_   = 0.0f;
            commit_remaining_      = commit_ticks_;
        }
        commit_remaining_ -= 1;
        auto out = std::make_shared<ActionOut>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("action_decoder") : id_;
        out->accel       = commit_accel_;
        out->action_tle  = last_action_tle_;
        bus_->publish(output_topic_, out);
        return;
    }
    // --------------------------------------------------------------------------

    // TD update against the previous tick's (state, proprio, action).
    if (latest_neuro_) td_update(latest_neuro_->reward_signal);

    // Decide action.  Priority: in-progress chunk → ExplorationDirective →
    // fresh chunk dispatch → EFE policy.  Bootstrap state (-1) emits zero.
    float accel        = 0.0f;
    bool  probe        = false;
    int   active_chunk = -1;
    int   chunk_left   = 0;

    // v5.3 Phase B — intent-chunk replay: publish IntentToken on
    // intent_override_topic_ (Premotor consumes it and emits the
    // corresponding bilateral motor commands).  ActionDecoder still
    // publishes ActionOut below for telemetry continuity, but with
    // accel=0 since the actual motor signal flows through Premotor.
    if (chunk_remaining_ > 0 && !intent_chunk_queue_.empty()
            && !intent_override_topic_.empty()) {
        int idx = intent_chunk_queue_.front();
        intent_chunk_queue_.pop_front();
        --chunk_remaining_;
        active_chunk = active_chunk_id_;
        chunk_left   = chunk_remaining_;

        auto tok = std::make_shared<IntentToken>();
        tok->tick_id     = tick_id;
        tok->producer_id = id_.empty() ? std::string("action_decoder") : id_;
        tok->index       = idx;
        // Phase 7.2-EPM: when configured with a radix AND idx is valid,
        // unpack combined index into per-channel indices so multi-channel
        // chunks coordinate >1 Premotor per dispatch.  Skip unpack when
        // idx<0 (empty-position slot from EMA-crystallised chunks) so
        // Premotor falls through to its legacy idx<0 = no-override path.
        if (idx >= 0 && !intent_channel_radix_.empty()) {
            tok->indices.resize(intent_channel_radix_.size());
            int rem = idx;
            for (size_t i = 0; i < intent_channel_radix_.size(); ++i) {
                int R = intent_channel_radix_[i];
                tok->indices[i] = rem % R;
                rem /= R;
            }
        }
        bus_->publish(intent_override_topic_, tok);
    }
    else if (chunk_remaining_ > 0 && !chunk_queue_.empty()) {
        accel = std::clamp(chunk_queue_.front(), accel_min_, accel_max_);
        chunk_queue_.pop_front();
        --chunk_remaining_;
        active_chunk = active_chunk_id_;
        chunk_left   = chunk_remaining_;
    }
    else if (exploration_active_) {
        // HomeokineticExploration has armed an episode; hold the directive's
        // sampled accel.  Do not dispatch chunks or run EFE during the
        // episode — the gate fired precisely because both were unable to
        // produce useful action.
        accel = std::clamp(exploration_accel_, accel_min_, accel_max_);
        probe = true;   // ActionOut.probe carries "non-EFE override" semantics
    }
    else if (state_node >= 0 && proprio_node >= 0) {
        // Try to start a new chunk.  If successful, it queued actions OR
        // intents for the next tick(s) AND also gives us the head-of-stream
        // now.  v5.3 Phase B: handle both queues (float / intent).
        if (try_dispatch_chunk(state_node, tick_id)) {
            if (!intent_chunk_queue_.empty() && !intent_override_topic_.empty()) {
                int idx = intent_chunk_queue_.front();
                intent_chunk_queue_.pop_front();
                --chunk_remaining_;
                active_chunk = active_chunk_id_;
                chunk_left   = chunk_remaining_;
                auto tok = std::make_shared<IntentToken>();
                tok->tick_id     = tick_id;
                tok->producer_id = id_.empty() ? std::string("action_decoder") : id_;
                tok->index       = idx;
                if (idx >= 0 && !intent_channel_radix_.empty()) {
                    tok->indices.resize(intent_channel_radix_.size());
                    int rem = idx;
                    for (size_t i = 0; i < intent_channel_radix_.size(); ++i) {
                        int R = intent_channel_radix_[i];
                        tok->indices[i] = rem % R;
                        rem /= R;
                    }
                }
                bus_->publish(intent_override_topic_, tok);
            } else if (!chunk_queue_.empty()) {
                accel = std::clamp(chunk_queue_.front(), accel_min_, accel_max_);
                chunk_queue_.pop_front();
                --chunk_remaining_;
                active_chunk = active_chunk_id_;
                chunk_left   = chunk_remaining_;
            } else {
                accel = select_action(state_node, proprio_node, modality);
            }
        } else {
            accel = select_action(state_node, proprio_node, modality);
        }
    }

    auto out = std::make_shared<ActionOut>();
    out->tick_id               = tick_id;
    out->producer_id           = id_.empty() ? std::string("action_decoder") : id_;
    out->accel                 = accel;
    out->probe                 = probe;
    out->chunk_id              = active_chunk;
    out->chunk_remaining_ticks = chunk_left;
    // v5.4 Phase F (Proposal B): chunk quality on every tick.  Non-zero
    // while a chunk is replaying (we have either active_chunk set or
    // chunk_remaining_>0 for the current step we just popped from);
    // 0 when no chunk active.  Reset stash when chunk fully drains so
    // the next quality reads start fresh.
    bool chunk_active_now = (active_chunk != -1) || chunk_remaining_ > 0;
    out->chunk_quality    = chunk_active_now ? current_chunk_quality_ : 0.0f;
    // v5.4 Phase G — body-position derived from how many intents have
    // been popped so far: ticks_consumed = total - remaining (after this
    // tick's pop).  position = ticks_consumed / playback_ticks_per_position.
    if (chunk_active_now && current_chunk_total_intents_ > 0
        && current_chunk_playback_per_ > 0) {
        int consumed = current_chunk_total_intents_ - chunk_remaining_;
        if (consumed < 0) consumed = 0;
        int pos = consumed / current_chunk_playback_per_;
        if (pos >= current_chunk_body_keyframes_) pos = current_chunk_body_keyframes_ - 1;
        out->chunk_position = pos;
    } else {
        out->chunk_position = -1;
    }
    if (chunk_remaining_ == 0 && active_chunk == -1) current_chunk_quality_ = 0.0f;
    out->action_tle            = last_action_tle_;
    bus_->publish(output_topic_, out);

    // Roll forward state for the next tick's TD update.
    prev_state_     = state_node;
    prev_proprio_   = proprio_node;
    prev_action_    = accel;
    prev_modality_  = modality;

    // Eligibility trace bookkeeping (length-bounded; not used in update for
    // Phase 1.5 — single-step TD only).  Kept so Phase 3 can extend to
    // n-step returns without an interface change.
    eligibility_.push_back(Trace{state_node, proprio_node, accel, modality});
    while (int(eligibility_.size()) > eligibility_max_len_) eligibility_.pop_front();

    // Clear caches so a missing publish next tick doesn't replay stale data.
    latest_consensus_.reset();
    latest_drive_.reset();
    latest_neuro_.reset();
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------
//
// Bus-message caches (latest_consensus_, latest_drive_, latest_neuro_) are
// reset() at end of every tick — no need to serialise them.  latest_chunks_
// persists across ticks, but on restore MotorRepertoire's library_dirty_ is
// forced true so the clone receives a fresh chunk-library publish on tick
// T+1 anyway.

nlohmann::json ActionDecoder::snapshot_state() const {
    nlohmann::json hebb = nlohmann::json::object();
    for (auto const& [k, v] : hebbian_) hebb[k] = v;
    nlohmann::json val = nlohmann::json::object();
    for (auto const& [k, v] : valence_) val[k] = v;
    nlohmann::json fwd = nlohmann::json::object();
    for (auto const& [k, dist] : forward_model_) {
        nlohmann::json d = nlohmann::json::object();
        for (auto const& [next, count] : dist) d[std::to_string(next)] = count;
        fwd[k] = d;
    }
    nlohmann::json elig = nlohmann::json::array();
    for (auto const& tr : eligibility_) {
        elig.push_back({
            {"state",    tr.state},
            {"proprio",  tr.proprio},
            {"action",   tr.action},
            {"modality", tr.modality},
        });
    }
    nlohmann::json cqueue = nlohmann::json::array();
    for (auto v : chunk_queue_) cqueue.push_back(v);
    std::ostringstream rng_ss; rng_ss << rng_;
    return nlohmann::json{
        {"version",                       1},
        {"latest_proprio_node",           latest_proprio_node_},
        {"hebbian",                       hebb},
        {"valence",                       val},
        {"forward_model",                 fwd},
        {"last_action_tle",               last_action_tle_},
        {"eligibility",                   elig},
        {"prev_state",                    prev_state_},
        {"prev_proprio",                  prev_proprio_},
        {"prev_action",                   prev_action_},
        {"prev_modality",                 prev_modality_},
        {"exploration_active",            exploration_active_},
        {"exploration_accel",             exploration_accel_},
        {"exploration_ticks_remaining",   exploration_ticks_remaining_},
        {"chunk_queue",                   cqueue},
        {"chunk_remaining",               chunk_remaining_},
        {"active_chunk_id",               active_chunk_id_},
        {"chunk_dispatch_count",          chunk_dispatch_count_},
        {"current_consensus_motif_id",    current_consensus_motif_id_},
        {"current_drive_urgency",         current_drive_urgency_},
        {"next_chunk_request_id",         next_chunk_request_id_},
        {"pending_chunk_request",         pending_chunk_request_},
        {"next_rollout_request_id",       next_rollout_request_id_},
        {"pending_rollout_request",       pending_rollout_request_},
        {"last_rollout_request_id",       last_rollout_request_id_},
        {"rng",                           rng_ss.str()},
    };
}

// Live viz (v4_inspector ActionDecoder widget): the coxswain's decision state —
// which belief node it's in, the preferred-observation targets it scores plans by
// (scent + green loom), the action it's currently committing, and the action its
// H-step plan recommends.  Lets the operator SEE whether the brain is aiming at
// the target it should be, and whether commit ≠ plan (mid-option hold).
nlohmann::json ActionDecoder::diag_snapshot() const {
    nlohmann::json j;
    // mode flags so the widget knows which fields are meaningful
    j["efe_select"]   = efe_select_;
    j["joint_action"] = joint_action_;
    j["plan_horizon"] = plan_horizon_;
    j["action_bins"]  = action_bins_;
    j["thrust_bins"]  = thrust_bins_;
    j["n_actions"]    = n_actions();

    // resolved belief node (the consensus/proprio winner the actor is reasoning over)
    auto [state_node, proprio_node] = resolve_state();
    j["state_node"]   = state_node;
    j["proprio_node"] = proprio_node;

    // preferred-observation targets (active-inference C prior the plan optimizes)
    j["pref_obs"]   = latest_pref_obs_;     // scent proximity / bearing the brain wants to raise
    j["green_obs"]  = latest_green_obs_;    // green-loom long-range food gauge
    j["w_scent"]    = w_scent_;
    j["w_green"]    = w_green_;
    // per-state learned target estimate + plan value (how good this state looks)
    j["node_value"] = (state_node >= 0) ? node_value(state_node) : 0.0f;
    j["plan_value"] = (state_node >= 0 && plan_horizon_ > 1)
                          ? plan_value(state_node, plan_horizon_) : 0.0f;

    // committed action (held this option) decoded into turn + thrust
    j["commit_action_idx"] = commit_action_idx_;
    if (joint_action_ && commit_action_idx_ >= 0) {
        float t = 0.0f, h = 0.0f;
        decode_joint(commit_action_idx_, t, h);
        j["commit_turn"]   = t;
        j["commit_thrust"] = h;
    } else {
        j["commit_turn"]   = commit_accel_;   // 1-D legacy: the held steer
        j["commit_thrust"] = 0.0f;
    }

    // the action the H-step plan recommends RIGHT NOW from the current state
    if (state_node >= 0) {
        int pa = plan_first_action(state_node);
        j["plan_action_idx"] = pa;
        if (joint_action_) {
            float t = 0.0f, h = 0.0f;
            decode_joint(pa, t, h);
            j["plan_turn"]   = t;
            j["plan_thrust"] = h;
        }
    } else {
        j["plan_action_idx"] = -1;
    }

    // learning health
    j["greedy_accel"]      = last_greedy_accel_;
    j["action_tle"]        = last_action_tle_;    // forward-model surprise (epistemic signal)
    j["interest"]          = latest_interest_;
    j["authority"]         = latest_authority_;   // credit-by-authority gate value
    j["plan_entropy"]      = last_plan_entropy_;     // ~1 exploring (flat softmax), ~0 confident (peaked)
    j["plan_confidence"]   = last_plan_confidence_;  // softmax prob of the chosen action
    j["fwd_model_size"]    = int(forward_model_.size());
    j["obs_states_known"]  = int(obs_value_.size());
    return j;
}

void ActionDecoder::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ActionDecoder::restore_state: unknown version " +
                                 std::to_string(version));
    }
    latest_proprio_node_ = s.value("latest_proprio_node", latest_proprio_node_);
    hebbian_.clear();
    if (s.contains("hebbian") && s["hebbian"].is_object())
        for (auto it = s["hebbian"].begin(); it != s["hebbian"].end(); ++it)
            hebbian_[it.key()] = it.value().get<float>();
    valence_.clear();
    if (s.contains("valence") && s["valence"].is_object())
        for (auto it = s["valence"].begin(); it != s["valence"].end(); ++it)
            valence_[it.key()] = it.value().get<float>();
    forward_model_.clear();
    if (s.contains("forward_model") && s["forward_model"].is_object()) {
        for (auto it = s["forward_model"].begin(); it != s["forward_model"].end(); ++it) {
            std::unordered_map<int, int> dist;
            for (auto it2 = it.value().begin(); it2 != it.value().end(); ++it2)
                dist[std::stoi(it2.key())] = it2.value().get<int>();
            forward_model_[it.key()] = std::move(dist);
        }
    }
    last_action_tle_ = s.value("last_action_tle", last_action_tle_);
    eligibility_.clear();
    if (s.contains("eligibility") && s["eligibility"].is_array()) {
        for (auto const& je : s["eligibility"]) {
            Trace tr;
            tr.state    = je.value("state",    -1);
            tr.proprio  = je.value("proprio",  -1);
            tr.action   = je.value("action",   0.0f);
            tr.modality = je.value("modality", std::string{});
            eligibility_.push_back(std::move(tr));
        }
    }
    prev_state_                  = s.value("prev_state",                  prev_state_);
    prev_proprio_                = s.value("prev_proprio",                prev_proprio_);
    prev_action_                 = s.value("prev_action",                 prev_action_);
    prev_modality_               = s.value("prev_modality",               prev_modality_);
    exploration_active_          = s.value("exploration_active",          exploration_active_);
    exploration_accel_           = s.value("exploration_accel",           exploration_accel_);
    exploration_ticks_remaining_ = s.value("exploration_ticks_remaining", exploration_ticks_remaining_);
    chunk_queue_.clear();
    if (s.contains("chunk_queue") && s["chunk_queue"].is_array())
        for (auto const& v : s["chunk_queue"]) chunk_queue_.push_back(v.get<float>());
    chunk_remaining_             = s.value("chunk_remaining",             chunk_remaining_);
    active_chunk_id_             = s.value("active_chunk_id",             active_chunk_id_);
    chunk_dispatch_count_        = s.value("chunk_dispatch_count",        chunk_dispatch_count_);
    current_consensus_motif_id_  = s.value("current_consensus_motif_id",  current_consensus_motif_id_);
    current_drive_urgency_       = s.value("current_drive_urgency",       current_drive_urgency_);
    next_chunk_request_id_       = s.value("next_chunk_request_id",       next_chunk_request_id_);
    pending_chunk_request_       = s.value("pending_chunk_request",       pending_chunk_request_);
    next_rollout_request_id_     = s.value("next_rollout_request_id",     next_rollout_request_id_);
    pending_rollout_request_     = s.value("pending_rollout_request",     pending_rollout_request_);
    last_rollout_request_id_     = s.value("last_rollout_request_id",     last_rollout_request_id_);
    std::string rng_s = s.value("rng", std::string{});
    if (!rng_s.empty()) {
        std::istringstream iss(rng_s);
        iss >> rng_;
    }
}

} // namespace ogma
