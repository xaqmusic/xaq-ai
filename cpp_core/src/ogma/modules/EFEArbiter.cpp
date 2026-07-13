#include "ogma/modules/EFEArbiter.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("EFEArbiter: param '" + k + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("EFEArbiter: param '" + k + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("EFEArbiter: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    if (auto p = std::get_if<double>(&v))  return *p != 0.0;
    throw std::invalid_argument("EFEArbiter: param '" + k + "' must be boolean");
}
}  // namespace

std::string_view EFEArbiter::type_name() const { return "EFEArbiter"; }

std::vector<TopicSpec> EFEArbiter::input_topics() const {
    return {
        TopicSpec{hunger_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{scent_topic_,      std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{plan_value_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{klino_confidence_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{plan_novelty_topic_,   std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{plan_precision_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> EFEArbiter::output_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{klino_gain_topic_,   std::type_index(typeid(ProprioToken))},
        TopicSpec{planner_gain_topic_, std::type_index(typeid(ProprioToken))},
    };
    if (!play_gain_topic_.empty())   // task #33 — only when the play channel is wired (byte-identical otherwise)
        v.push_back(TopicSpec{play_gain_topic_, std::type_index(typeid(ProprioToken))});
    if (!vision_gain_topic_.empty()) // loop #4 — only when the vision channel is wired (byte-identical otherwise)
        v.push_back(TopicSpec{vision_gain_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema EFEArbiter::params_schema() const {
    return {
        {"hunger_topic",     ParamMutability::ConstructionOnly, "klino preference weight (interoceptive hunger).", ParamValue{std::string("reality.proprio.hunger")}},
        {"scent_topic",      ParamMutability::ConstructionOnly, "klino scent proximity (SCALAR scent_max).", ParamValue{std::string("reality.proprio.scent_max")}},
        {"plan_value_topic", ParamMutability::ConstructionOnly, "planner FOOD-ROUTE value (value(next_hop) while routing to remembered food, else 0).", ParamValue{std::string("reality.cognitive.plan_value")}},
        {"klino_confidence_topic", ParamMutability::ConstructionOnly, "klino's SELF-REPORTED capability ∈[0,1] (BOOSTS v_klino's proximity level near food: level=clamp(hunger·cap); a MAX so it never silences a far/blind klino). Absent → the boost is inert (behavior unchanged when not wired).", ParamValue{std::string("percept.klino_confidence")}},
        {"plan_novelty_topic",   ParamMutability::ConstructionOnly, "efe: planner FRONTIER NOVELTY ∈[0,1] (the map-uncertainty a hop would resolve; ~0 while exploiting a food route) → g_epist_planner=(1−hunger)·plan_novelty. Absent → 0 (planner has no epistemic term).", ParamValue{std::string("reality.cognitive.plan_novelty")}},
        {"plan_precision_topic", ParamMutability::ConstructionOnly, "efe: planner MODEL PRECISION ∈[0,1] (sharpness of the food belief) → the optional klino undirected-search floor (klino_search_floor). Absent → 0.", ParamValue{std::string("reality.cognitive.plan_precision")}},
        {"klino_gain_topic",   ParamMutability::ConstructionOnly, "winner-take-all gain → MotorBus (klino channel).", ParamValue{std::string("arbiter.gain.klino")}},
        {"planner_gain_topic", ParamMutability::ConstructionOnly, "winner-take-all gain → MotorBus (planner channel).", ParamValue{std::string("arbiter.gain.planner")}},
        {"play_value_topic",   ParamMutability::ConstructionOnly, "task #33: PlayLoop FRONTIER VALUE ∈[0,1] (epistemic map-growth potential) → G_play. Empty (default) = play absent (byte-identical 2-policy arbiter).", ParamValue{std::string("")}},
        {"play_gain_topic",    ParamMutability::ConstructionOnly, "task #33: winner-take-all gain → MotorBus (play channel). Empty (default) = no publish.", ParamValue{std::string("")}},
        {"play_weight",        ParamMutability::HotMutable, "task #33: play epistemic gain. G_play = play_weight·(1−hunger=energy surplus)·play_value (curiosity is instrumental → play most when FULL). 0 (default) = play INERT (never wins; 2-policy race byte-identical). 1 = the third policy (Stage 2+). efe mode only.", ParamValue{0.0}},
        {"play_hunger_weight", ParamMutability::HotMutable, "task #33 ABLATION (wrong-sign control): weight play by HUNGER instead of (1−hunger) energy surplus. true → the bug explores when HUNGRY (should REGRESS: wanders off known food and starves), proving the play-when-FULL sign is correct. Default false.", ParamValue{false}},
        {"vision_value_topic", ParamMutability::ConstructionOnly, "loop #4: VisualHomingNav sight-confidence ∈[0,1] (pragmatic close on a SEEN source) → G_vision. Empty (default) = vision absent (byte-identical arbiter).", ParamValue{std::string("")}},
        {"vision_gain_topic",  ParamMutability::ConstructionOnly, "loop #4: winner-take-all gain → MotorBus (vision channel). Empty (default) = no publish.", ParamValue{std::string("")}},
        {"vision_weight",      ParamMutability::HotMutable, "loop #4: vision pragmatic gain. G_vision = vision_weight·hunger·vision_value (CLOSE on a seen source, hunger-weighted like klino). 0 (default) = vision INERT (never wins; 2/3-policy race byte-identical). 1 = the fourth policy (Stage 2+). efe mode only.", ParamValue{0.0}},
        {"epistemic_reach_gated", ParamMutability::HotMutable, "R1 (2026-07-09): gate the epistemic (explore) terms by (1 − max(g_prag_klino, g_prag_planner)) = 1 − hunger·max_reach, NOT the blanket (1−hunger). Breaks the starvation deadlock (§2.1): a HUNGRY-but-BLIND agent (no reach) is in MAX uncertainty → epistemic stays high → play pushes the frontier to find food, instead of (1−hunger)→0 silencing exploration exactly when it is needed. full → gate≈1 (play when full, preserved); hungry+reachable → gate≈0 (exploit). No new constant. false = legacy (1−hunger) gate (ablation baseline).", ParamValue{true}},
        {"scoring_mode",  ParamMutability::HotMutable, "'value_race' (legacy value race — DEFAULT, byte-identical to prior builds) | 'efe' (explicit-EFE precision scoring: each policy G = pragmatic hunger·reach-prob (SHARED UNITS, scale mismatch gone by construction — no cede/plan_peak/max-of-three) + epistemic (1−hunger)·uncertainty-reduction; hunger sets the exploit/explore balance).", ParamValue{std::string("value_race")}},
        {"z_peak_decay",  ParamMutability::HotMutable, "efe: SLOW decay of klino's z-spike running peak (the epistemic normaliser z_ref, §6 — derived from the signal, not hand-set).", ParamValue{0.0005}},
        {"planner_epistemic",  ParamMutability::HotMutable, "efe ABLATION: include the planner's epistemic term g_epist_planner=(1−hunger)·plan_novelty (true) or zero it (false, Stage-3 coverage A/B).", ParamValue{true}},
        {"klino_search_floor", ParamMutability::HotMutable, "efe: add g_epist_klino += (1−hunger)·(1−plan_precision) — an UNDIRECTED klino search drive when the planner's model is imprecise (§1.4 blind-forager floor; keep OFF unless eats regress below the value-race baseline).", ParamValue{false}},
        {"mean_alpha",    ParamMutability::HotMutable, "EMA rate of KLINO's running baseline (z-score mean, §6). Slow → klino stays EXCITED while inside the scent field (~100 ticks).", ParamValue{0.01}},
        {"var_alpha",     ParamMutability::HotMutable, "EMA rate of KLINO's running variance (z-score scale, §6).", ParamValue{0.01}},
        {"std_eps",       ParamMutability::HotMutable, "Floor on KLINO's z-score denominator so a FLAT klino signal can't blow up its z-score.", ParamValue{0.02}},
        {"plan_peak_decay", ParamMutability::HotMutable, "SLOW decay of the planner's food-route PEAK (the level denominator: v_planner = raw_planner/plan_peak). Remembers a typical good-route value so a sustained route reads ~1; never lets v_planner decay/go negative (the false-interruption fix). ~1/plan_peak_decay-tick memory.", ParamValue{0.0005}},
        {"hysteresis_k",  ParamMutability::HotMutable, "margin = hysteresis_k · running_std(v_klino − v_planner): adaptive dwell, no magic count.", ParamValue{1.0}},
        {"gap_std_alpha", ParamMutability::HotMutable, "EMA rate of the running mean/variance of the value gap (the hysteresis scale).", ParamValue{0.02}},
        {"force_policy",  ParamMutability::HotMutable, "ABLATION: '' = live EFE arbiter; 'klino'/'planner' = always that policy; 'shuffle' = random winner each tick (rng seeded by master_seed, varied by tick).", ParamValue{std::string("")}},
        {"master_seed",   ParamMutability::ConstructionOnly, "RNG seed (shuffle control).", ParamValue{int64_t{11}}},
    };
}

ParamMap EFEArbiter::current_params() const {
    ParamMap m;
    m["hunger_topic"]     = ParamValue{hunger_topic_};
    m["scent_topic"]      = ParamValue{scent_topic_};
    m["plan_value_topic"] = ParamValue{plan_value_topic_};
    m["klino_confidence_topic"] = ParamValue{klino_confidence_topic_};
    m["klino_gain_topic"]   = ParamValue{klino_gain_topic_};
    m["planner_gain_topic"] = ParamValue{planner_gain_topic_};
    m["play_value_topic"]   = ParamValue{play_value_topic_};
    m["play_gain_topic"]    = ParamValue{play_gain_topic_};
    m["play_weight"]        = ParamValue{double(play_weight_)};
    m["play_hunger_weight"] = ParamValue{play_hunger_weight_};
    m["vision_value_topic"] = ParamValue{vision_value_topic_};
    m["vision_gain_topic"]  = ParamValue{vision_gain_topic_};
    m["vision_weight"]      = ParamValue{double(vision_weight_)};
    m["epistemic_reach_gated"] = ParamValue{epistemic_reach_gated_};
    m["scoring_mode"]  = ParamValue{scoring_mode_};
    m["z_peak_decay"]  = ParamValue{double(z_peak_decay_)};
    m["planner_epistemic"]  = ParamValue{planner_epistemic_};
    m["klino_search_floor"] = ParamValue{klino_search_floor_};
    m["mean_alpha"]    = ParamValue{double(mean_alpha_)};
    m["var_alpha"]     = ParamValue{double(var_alpha_)};
    m["std_eps"]       = ParamValue{double(std_eps_)};
    m["plan_peak_decay"] = ParamValue{double(plan_peak_decay_)};
    m["hysteresis_k"]  = ParamValue{double(hysteresis_k_)};
    m["gap_std_alpha"] = ParamValue{double(gap_std_alpha_)};
    m["force_policy"]  = ParamValue{force_policy_};
    m["master_seed"]   = ParamValue{int64_t(master_seed_)};
    return m;
}

void EFEArbiter::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "scoring_mode")  { scoring_mode_ = get_string(value, k);
                                     play_active_ = (play_weight_ > 0.0f) && !play_value_topic_.empty() && scoring_mode_ == "efe"; }
    else if (k == "play_weight")   { play_weight_ = float(get_double(value, k));
                                     play_active_ = (play_weight_ > 0.0f) && !play_value_topic_.empty() && scoring_mode_ == "efe"; }
    else if (k == "play_hunger_weight") play_hunger_weight_ = get_bool(value, k);
    else if (k == "vision_weight") { vision_weight_ = float(get_double(value, k));
                                     vision_active_ = (vision_weight_ > 0.0f) && !vision_value_topic_.empty() && scoring_mode_ == "efe"; }
    else if (k == "epistemic_reach_gated") epistemic_reach_gated_ = get_bool(value, k);
    else if (k == "z_peak_decay")  z_peak_decay_  = float(get_double(value, k));
    else if (k == "planner_epistemic")  planner_epistemic_  = get_bool(value, k);
    else if (k == "klino_search_floor") klino_search_floor_ = get_bool(value, k);
    else if (k == "mean_alpha")    mean_alpha_    = float(get_double(value, k));
    else if (k == "var_alpha")     var_alpha_     = float(get_double(value, k));
    else if (k == "std_eps")       std_eps_       = float(get_double(value, k));
    else if (k == "plan_peak_decay") plan_peak_decay_ = float(get_double(value, k));
    else if (k == "hysteresis_k")  hysteresis_k_  = float(get_double(value, k));
    else if (k == "gap_std_alpha") gap_std_alpha_ = float(get_double(value, k));
    else if (k == "force_policy")  force_policy_  = get_string(value, k);
    else throw std::invalid_argument("EFEArbiter: param '" + k + "' is construction-only / unknown");
}

void EFEArbiter::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "hunger_topic",     [&](auto const& v){ hunger_topic_     = get_string(v,"hunger_topic"); });
    apply_param(params, "scent_topic",      [&](auto const& v){ scent_topic_      = get_string(v,"scent_topic"); });
    apply_param(params, "plan_value_topic", [&](auto const& v){ plan_value_topic_ = get_string(v,"plan_value_topic"); });
    apply_param(params, "klino_confidence_topic", [&](auto const& v){ klino_confidence_topic_ = get_string(v,"klino_confidence_topic"); });
    apply_param(params, "klino_gain_topic",   [&](auto const& v){ klino_gain_topic_   = get_string(v,"klino_gain_topic"); });
    apply_param(params, "planner_gain_topic", [&](auto const& v){ planner_gain_topic_ = get_string(v,"planner_gain_topic"); });
    apply_param(params, "play_value_topic",   [&](auto const& v){ play_value_topic_   = get_string(v,"play_value_topic"); });
    apply_param(params, "play_gain_topic",    [&](auto const& v){ play_gain_topic_    = get_string(v,"play_gain_topic"); });
    apply_param(params, "play_weight",        [&](auto const& v){ play_weight_        = float(get_double(v,"play_weight")); });
    apply_param(params, "play_hunger_weight", [&](auto const& v){ play_hunger_weight_ = get_bool(v,"play_hunger_weight"); });
    apply_param(params, "vision_value_topic", [&](auto const& v){ vision_value_topic_ = get_string(v,"vision_value_topic"); });
    apply_param(params, "vision_gain_topic",  [&](auto const& v){ vision_gain_topic_  = get_string(v,"vision_gain_topic"); });
    apply_param(params, "vision_weight",      [&](auto const& v){ vision_weight_      = float(get_double(v,"vision_weight")); });
    apply_param(params, "epistemic_reach_gated", [&](auto const& v){ epistemic_reach_gated_ = get_bool(v,"epistemic_reach_gated"); });
    apply_param(params, "scoring_mode",  [&](auto const& v){ scoring_mode_  = get_string(v,"scoring_mode"); });
    apply_param(params, "z_peak_decay",  [&](auto const& v){ z_peak_decay_  = float(get_double(v,"z_peak_decay")); });
    apply_param(params, "planner_epistemic",  [&](auto const& v){ planner_epistemic_  = get_bool(v,"planner_epistemic"); });
    apply_param(params, "klino_search_floor", [&](auto const& v){ klino_search_floor_ = get_bool(v,"klino_search_floor"); });
    apply_param(params, "plan_novelty_topic",   [&](auto const& v){ plan_novelty_topic_   = get_string(v,"plan_novelty_topic"); });
    apply_param(params, "plan_precision_topic", [&](auto const& v){ plan_precision_topic_ = get_string(v,"plan_precision_topic"); });
    apply_param(params, "mean_alpha",    [&](auto const& v){ mean_alpha_    = float(get_double(v,"mean_alpha")); });
    apply_param(params, "var_alpha",     [&](auto const& v){ var_alpha_     = float(get_double(v,"var_alpha")); });
    apply_param(params, "std_eps",       [&](auto const& v){ std_eps_       = float(get_double(v,"std_eps")); });
    apply_param(params, "plan_peak_decay", [&](auto const& v){ plan_peak_decay_ = float(get_double(v,"plan_peak_decay")); });
    apply_param(params, "hysteresis_k",  [&](auto const& v){ hysteresis_k_  = float(get_double(v,"hysteresis_k")); });
    apply_param(params, "gap_std_alpha", [&](auto const& v){ gap_std_alpha_ = float(get_double(v,"gap_std_alpha")); });
    apply_param(params, "force_policy",  [&](auto const& v){ force_policy_  = get_string(v,"force_policy"); });
    apply_param(params, "master_seed",   [&](auto const& v){ master_seed_   = uint64_t(get_int(v,"master_seed")); });

    rng_.seed(master_seed_);

    if (!hunger_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(hunger_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_hunger(p); }));
    if (!scent_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    if (!plan_value_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(plan_value_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_plan_value(p); }));
    if (!klino_confidence_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(klino_confidence_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_klino_confidence(p); }));
    if (!plan_novelty_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(plan_novelty_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_plan_novelty(p); }));
    if (!plan_precision_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(plan_precision_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_plan_precision(p); }));
    if (!play_value_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(play_value_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_play_value(p); }));
    if (!vision_value_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vision_value_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vision_value(p); }));

    // task #33 — play joins the winner race only when wired AND weighted AND in efe mode.
    // Otherwise the 2-policy klino/planner logic below is byte-identical (play stays inert).
    play_active_ = (play_weight_ > 0.0f) && !play_value_topic_.empty() && scoring_mode_ == "efe";
    // loop #4 — vision joins the race only when wired AND weighted AND in efe mode (else byte-identical).
    vision_active_ = (vision_weight_ > 0.0f) && !vision_value_topic_.empty() && scoring_mode_ == "efe";
}

void EFEArbiter::handle_hunger(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) hunger_ = float(pt->values[0]);
}
void EFEArbiter::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) scent_ = float(pt->values[0]);
}
void EFEArbiter::handle_plan_value(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) plan_value_ = float(pt->values[0]);
}
void EFEArbiter::handle_klino_confidence(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { cap_klino_ = float(pt->values[0]); have_cap_ = true; }
}
void EFEArbiter::handle_plan_novelty(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) plan_novelty_ = float(pt->values[0]);
}
void EFEArbiter::handle_plan_precision(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) plan_precision_ = float(pt->values[0]);
}
void EFEArbiter::handle_play_value(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) play_value_ = float(pt->values[0]);
}
void EFEArbiter::handle_vision_value(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) vision_value_ = float(pt->values[0]);
}

void EFEArbiter::tick(uint64_t tick_id) {
    // ---- shared: raw EFE inputs from the agent's OWN beliefs (bar b) ----
    // klino proximity drive (hunger·scent); planner food-route value (0 while exploring).
    raw_klino_   = hunger_ * scent_;
    raw_planner_ = plan_value_;
    // task #33 — play terms reset each tick; set only in efe mode (0 in value_race / when inert).
    g_epist_play_ = 0.0f; G_play_ = 0.0f; v_play_ = 0.0f;
    // loop #4 — vision terms reset each tick; set only in efe mode (0 in value_race / when inert).
    g_prag_vision_ = 0.0f; G_vision_ = 0.0f; v_vision_ = 0.0f;

    if (scoring_mode_ == "efe") {
        // =====================================================================
        // EXPLICIT-EFE SCORING (doctrine §2.2 / §2.3) — the precision refactor.
        // Each policy scored as an expected free energy  G = pragmatic + epistemic.
        // The two PRAGMATIC terms share units (hunger × reach-probability) so the
        // klino/planner scale mismatch is gone BY CONSTRUCTION — no cede, no
        // plan_peak self-norm, no max-of-three. `hunger` (preference precision)
        // sets the exploit↔explore balance; no tuned λ.
        // =====================================================================

        // --- klino epistemic = the z-SPIKE (scent rising = evidence "food is this way") ---
        // Reuse the running baseline; normalise the z-score to [0,1] by its OWN running peak
        // (z_ref, §6 — derived from the signal) so it is commensurable with the pragmatic terms.
        mean_klino_ += mean_alpha_ * (raw_klino_ - mean_klino_);
        float d = raw_klino_ - mean_klino_;
        var_klino_ += var_alpha_ * (d * d - var_klino_);
        float sd = std::sqrt(std::max(var_klino_, 0.0f));
        float v_spike = d / (sd + std_eps_);
        z_peak_ = std::max(v_spike, z_peak_ * (1.0f - z_peak_decay_));
        float v_spike_norm = std::clamp(v_spike / (z_peak_ + 1e-4f), 0.0f, 1.0f);

        // --- reach-probabilities P(reach food | policy) ∈[0,1], hunger-FREE (sensory precision) ---
        // klino: eat-calibrated capability (→1 in its own eating range); pre-report fallback = raw
        //   scent proximity (so a blind klino reads ~0 and relies on the z-spike epistemic term).
        // planner: discounted route value (food_reward=1 ⇒ V≈γ^hops ∈(0,1]); clamp guards food_>1.
        float reach_klino   = have_cap_ ? std::clamp(cap_klino_, 0.0f, 1.0f)
                                        : std::clamp(scent_,     0.0f, 1.0f);
        float reach_planner = std::clamp(plan_value_, 0.0f, 1.0f);

        // --- pragmatic (exploit): hunger × reach-prob — SHARED UNITS = expected hunger-reduction ---
        g_prag_klino_   = hunger_ * reach_klino;
        g_prag_planner_ = hunger_ * reach_planner;
        // loop #4 — VISION is a pragmatic reach means (CLOSE on a SEEN source); its reach-prob is
        // VisualHomingNav's detection/direction confidence (vision_value ∈[0,1], distance-independent).
        // Computed HERE (before the gate) so it enters the reach-gate max below: when vision can reach
        // food it can SEE, exploration should back off just as it does for a smelling klino.
        g_prag_vision_ = vision_weight_ * hunger_ * std::clamp(vision_value_, 0.0f, 1.0f);
        // --- R1: epistemic precision = 1 − max reach (NOT the blanket 1−hunger) ---
        // The epistemic (explore) terms should be gated by how UNRESOLVED the pragmatic goal is, not
        // by energy surplus alone. max(g_prag) = hunger·max_reach is exactly "how well can I already
        // reach food"; 1 − that is the residual uncertainty the explore drive should resolve (§2.1).
        //   full            → g_prag≈0 → gate≈1 → play/explore wins (play-when-full preserved);
        //   hungry+reachable → g_prag≈1 → gate≈0 → pragmatic close dominates (correct exploit);
        //   hungry+BLIND     → g_prag≈0 → gate≈1 → play pushes the frontier to FIND food (deadlock
        //                      broken; the old 1−hunger gate silenced exactly this case).
        // No new constant — derived from the two pragmatic terms already computed. Legacy 1−hunger
        // available for the ablation baseline.
        float epistemic_gate = epistemic_reach_gated_
            ? std::clamp(1.0f - std::max(g_prag_vision_, std::max(g_prag_klino_, g_prag_planner_)), 0.0f, 1.0f)
            : (1.0f - hunger_);
        //   klino  = the normalised z-spike (approach / chemotactic evidence gathering).
        //   planner = its FRONTIER NOVELTY (plan_novelty = map-uncertainty a hop would resolve,
        //             ~0 while exploiting a food route) — the curiosity that maps new ground.
        g_epist_klino_   = epistemic_gate * v_spike_norm;
        g_epist_planner_ = planner_epistemic_ ? epistemic_gate * plan_novelty_ : 0.0f;
        // OPTIONAL undirected klino search floor (§1.4): when the planner's model is IMPRECISE
        // (plan_precision low ⇒ it can't point anywhere), let klino search undirected — expressed
        // from the dynamics (1−plan_precision), never a bare constant. Off by default (the blind
        // forager is already preserved by the z-spike); enable only if eats regress.
        if (klino_search_floor_)
            g_epist_klino_ += epistemic_gate * (1.0f - plan_precision_);

        G_klino_   = g_prag_klino_   + g_epist_klino_;
        G_planner_ = g_prag_planner_ + g_epist_planner_;

        // --- task #33: PLAY = epistemic GROW, energy-surplus weighted (§2.1 curiosity is instrumental) ---
        //   G_play = play_weight · (1−hunger) · play_value.  The pragmatic term is ≈0 (play does not
        //   reach food); (1−hunger) = ENERGY SURPLUS, so play scores highest when FULL — the map is
        //   built before hunger forces a return (plan §1). play_weight=0 ⇒ 0 ⇒ inert (never wins).
        // R1: play's epistemic gate = the reach-gated epistemic_gate (play explores when full OR when
        // hungry-and-blind). play_hunger_weight flips it to hunger = the WRONG-SIGN ablation.
        float play_gate = play_hunger_weight_ ? hunger_ : epistemic_gate;
        g_epist_play_ = play_weight_ * play_gate * play_value_;
        G_play_ = g_epist_play_;

        // --- loop #4: VISION = pragmatic CLOSE on a SEEN source (g_prag_vision computed above, in the
        //   reach-gate). Purely pragmatic (no epistemic term): vision homes to food it can SEE where
        //   scent is blind. vision_weight=0 ⇒ g_prag_vision=0 ⇒ inert (never wins; 2/3-policy byte-identical).
        G_vision_ = g_prag_vision_;
        v_vision_ = G_vision_;

        // Feed the EFE scores into the SHARED winner-take-all below via v_klino_/v_planner_.
        // Near food cap_klino→1 ⇒ g_prag_klino→hunger, which necessarily exceeds the planner's
        // discounted hunger·γ^hops to any REMEMBERED cache → klino owns the close, no cede.
        v_klino_   = G_klino_;
        v_planner_ = G_planner_;
        v_play_    = G_play_;
    } else {
    // =====================================================================
    // VALUE-RACE (legacy — DEFAULT). Byte-identical to prior builds.
    // =====================================================================
    // ---- klino = MAX( z-SCORE excitement , proximity LEVEL ) ----
    // The z-SCORE (§6: SD above klino's own ~100-tick baseline) SPIKES when scent RISES (food
    // smelled) → it wins the APPROACH. But a z-score measures CHANGE, so it goes QUIET when the bug
    // sits in STABLE-high scent — right AT the food, exactly when klino should dominate to make the
    // close (the planner's level otherwise saturates it). So we add a proximity LEVEL that SUSTAINS
    // the close. Two forms, taken as a MAX so the level can only ever RISE toward 1 (a BOOST near
    // food, never a silence far from it):
    //   (a) absolute proximity = clamp(raw_klino=hunger·scent, 0, 1): HIGH near a real source AND
    //       hungry, ~0 far/sated/flat. Always available. But the scent FIELD is source-normalised
    //       (source cell = 1.0) while the bug samples a sub-1 scent at its CLOSEST approach (it eats
    //       before reaching the source cell), so this level is structurally capped (~0.73) BELOW the
    //       planner's self-normalised ≈1 → klino ties/loses the close → they oscillate (operator obs).
    //   (b) EAT-CALIBRATED proximity = clamp(hunger · cap_klino, 0, 1), where cap_klino = klino's
    //       self-reported confidence = current-smell / its own learned EAT-scent (RunTumbleNav folds
    //       the scent at each real hit into eat_scent_). cap→1 the instant the bug is in its OWN
    //       eating range, so this level reaches ≈hunger (≈1 when hungry) AT the close — lifting the
    //       ~0.73 cap to the planner's scale so klino OWNS the close on its own merit. Only active
    //       once klino reports (have_cap_); the abs form covers the pre-report / unwired case.
    // v_klino = MAX(spike, abs-level, calibrated-level): spike wins the approach, the level SUSTAINS
    // the close. Route-hold holds: a BLIND klino (low scent → cap→0) reads low on ALL THREE — the
    // calibration BOOSTS near food but cannot lift a far/blind klino (cap's numerator is 0).
    {
        mean_klino_ += mean_alpha_ * (raw_klino_ - mean_klino_);
        float d = raw_klino_ - mean_klino_;
        var_klino_  += var_alpha_  * (d * d - var_klino_);
        float sd = std::sqrt(std::max(var_klino_, 0.0f));
        float v_spike     = d / (sd + std_eps_);                          // z-score: wins the APPROACH
        float v_lvl_abs   = std::clamp(raw_klino_, 0.0f, 1.0f);           // absolute proximity (~0.73-capped at food)
        float v_lvl_cal   = have_cap_ ? std::clamp(hunger_ * cap_klino_, 0.0f, 1.0f)
                                      : 0.0f;                             // EAT-calibrated proximity (→1 at the eat)
        v_klino_ = std::max(v_spike, std::max(v_lvl_abs, v_lvl_cal));     // BOOST near food, never silence far
    }

    // ---- planner = a SUSTAINED LEVEL normalised by its own slow-decaying peak (the FIX) ----
    // A plan is VALID for as long as it routes to food — a steady-state property, NOT a change.
    // plan_peak_ remembers the typical good-route value (decays SLOWLY); v_planner is the route's
    // raw value as a fraction of that peak, clamped to [0,1]. So a sustained route reads ~1 the
    // whole time and v_planner NEVER goes negative — a blind klino (z≈0) can no longer falsely
    // overtake a steady route. raw_planner_=0 (exploring) → v_planner=0, leaving klino free to
    // forage. (Was a z-score: a sustained-high raw pulled its mean up so v_planner decayed
    // negative mid-route and a blind klino crossed it → the false interruption. No more.)
    plan_peak_ = std::max(raw_planner_, plan_peak_ * (1.0f - plan_peak_decay_));
    v_planner_ = std::clamp(raw_planner_ / (plan_peak_ + 1e-4f), 0.0f, 1.0f);

    // ---- PLANNER CEDES to DIRECT SENSING (precision-weighting; 2026-06-30 operator obs) — belt-and-suspenders ----
    // When the goal is DIRECTLY SENSED, the belief-based ROUTE is less precise than the observation, so
    // DOWN-WEIGHT the planner by the ABSOLUTE sensed proximity (hunger·scent). A blind klino (low scent)
    // leaves v_planner untouched, so the route-hold / false-interruption guard is preserved. This works
    // WITH the eat-calibrated klino level above: the calibration lifts v_klino to the planner's scale (a
    // decisive win on its own merit), and this cede lowers v_planner near food (a margin against scent
    // jitter at the eat) — the two are complementary and BOTH only fire near real food. (Alone, this
    // absolute cede was only MARGINALLY decisive: at the eat scent ~0.5-0.6 the UNCALIBRATED klino level
    // ~0.5 barely lost to the ~0.5-ceded planner; the calibration is what makes the pairing decisive.
    // The RELATIVE form ÷scent_peak was decisive but BROKE route-hold — do NOT reintroduce it.)
    float klino_prox = std::clamp(raw_klino_, 0.0f, 1.0f);   // = hunger·scent, klino's absolute proximity (≈0.73-capped at food)
    v_planner_ *= (1.0f - klino_prox);

    // ---- (i) CAPABILITY as a BOOST (above), NOT a multiplicative silence GATE (that was REVERTED) ----
    // cap_klino now BOOSTS klino's proximity level near food (the v_lvl_cal term above) — it can only
    // RAISE v_klino toward 1, never lower it. The old `v_klino_ = cap_klino_ * v_klino_` SILENCE gate is
    // permanently retired: multiplying the z-score by cap zeroed a BLIND klino, but klino's blind
    // run-and-tumble IS the forager/bootstrap (it wanders until it smells, then closes) and the planner
    // cannot close — so silencing it collapsed eats (100%: 31→5, 50%: 20→0, n=5 seeds). The boost keeps
    // the forager alive (cap→0 far just means no boost, the z-spike still stands) while fixing the close.
    // v_klino_ = cap_klino_ * v_klino_;   // <-- DO NOT re-enable: this SILENCED the forager.
    }  // end value_race

    // ---- adaptive hysteresis margin (§2/§6): the gap's own running std ----
    // Shared across scoring modes: operates on v_klino_/v_planner_ (value-race values, or the
    // EFE scores mirrored into them above). The mechanism is orthogonal and proven — unchanged.
    // EMA mean + variance of the value gap so margin = hysteresis_k · std(gap). No magic
    // dwell-count — commitment scales with the value-gap's own dynamics.
    // task #33: when play is active the live competition is 3-way → use the top-two gap; otherwise
    // the 2-policy gap is unchanged (byte-identical).
    float gap;
    if (vision_active_) {
        // loop #4 — 4-way live competition → top1 − top2 over {klino, planner, play, vision}.
        float vs4[4] = { v_klino_, v_planner_, v_play_, v_vision_ };
        float t1 = -1e30f, t2 = -1e30f;
        for (float x : vs4) { if (x > t1) { t2 = t1; t1 = x; } else if (x > t2) { t2 = x; } }
        gap = t1 - t2;
    } else if (play_active_) {
        float hi = std::max(v_klino_, std::max(v_planner_, v_play_));
        float lo = std::min(v_klino_, std::min(v_planner_, v_play_));
        float mid = v_klino_ + v_planner_ + v_play_ - hi - lo;
        gap = hi - mid;   // top1 − top2
    } else {
        gap = v_klino_ - v_planner_;
    }
    if (!have_gap_ema_) { gap_mean_ = gap; gap_var_ = 0.0f; have_gap_ema_ = true; }
    else {
        float d = gap - gap_mean_;
        gap_mean_ += gap_std_alpha_ * d;
        gap_var_  += gap_std_alpha_ * (d * d - gap_var_);   // EMA of squared deviation
    }
    float gap_std = std::sqrt(std::max(0.0f, gap_var_));
    margin_ = hysteresis_k_ * gap_std;

    // ---- winner-take-all + commitment (low-T policy posterior) ----
    int new_winner;
    if (force_policy_ == "klino") {
        new_winner = 0;
    } else if (force_policy_ == "planner") {
        new_winner = 1;
    } else if (force_policy_ == "play") {   // task #33 ablation: force the play policy
        new_winner = 2;
    } else if (force_policy_ == "vision") { // loop #4 ablation: force the vision policy
        new_winner = 3;
    } else if (force_policy_ == "shuffle") {
        // random winner each tick — the FAIR ablation null. rng seeded by master_seed,
        // re-seeded per tick so the stream varies with the tick id (reproducible per seed).
        // 2-way klino/planner, +play when active, +vision when active (byte-identical when inert).
        std::mt19937 r(master_seed_ ^ (tick_id * 0x9E3779B97F4A7C15ull + 0x1234567ull));
        int n = vision_active_ ? 4 : (play_active_ ? 3 : 2);
        new_winner = (std::uniform_int_distribution<int>(0, n - 1)(r));
    } else if (vision_active_) {
        // LIVE EFE arbiter, 4-policy (loop #4): keep the incumbent unless the BEST challenger leads
        // it by more than the adaptive margin. Reduces to the 2/3-policy cases when those are inert
        // (v_play_/v_vision_ = 0), so this branch only runs when vision is genuinely active.
        float vs[4] = { v_klino_, v_planner_, v_play_, v_vision_ };
        int inc = (winner_ >= 0 && winner_ <= 3) ? winner_ : 0;
        int best_chl = -1; float best_chl_v = -1e30f;
        for (int i = 0; i < 4; ++i)
            if (i != inc && vs[i] > best_chl_v) { best_chl_v = vs[i]; best_chl = i; }
        if (best_chl >= 0 && best_chl_v - vs[inc] > margin_) new_winner = best_chl;
        else                                                 new_winner = inc;
    } else if (!play_active_) {
        // LIVE EFE arbiter, 2-policy (byte-identical): keep the incumbent; switch only when the
        // challenger leads by more than the adaptive margin. Hysteresis stops chatter without
        // softening the hard 0/1 mute.
        float v_inc = (winner_ == 0) ? v_klino_ : v_planner_;
        float v_chl = (winner_ == 0) ? v_planner_ : v_klino_;
        if (v_chl - v_inc > margin_) new_winner = (winner_ == 0) ? 1 : 0;
        else                          new_winner = winner_;
    } else {
        // LIVE EFE arbiter, 3-policy (task #33): keep the incumbent unless the BEST challenger
        // leads it by more than the adaptive margin.
        float vs[3] = { v_klino_, v_planner_, v_play_ };
        int inc = (winner_ >= 0 && winner_ <= 2) ? winner_ : 0;
        int best_chl = -1; float best_chl_v = -1e30f;
        for (int i = 0; i < 3; ++i)
            if (i != inc && vs[i] > best_chl_v) { best_chl_v = vs[i]; best_chl = i; }
        if (best_chl >= 0 && best_chl_v - vs[inc] > margin_) new_winner = best_chl;
        else                                                 new_winner = inc;
    }
    winner_ = new_winner;

    // hard gains: winner 1.0, loser 0.0 (basic channel muting; chatter prevented by hysteresis)
    gain_klino_   = (winner_ == 0) ? 1.0f : 0.0f;
    gain_planner_ = (winner_ == 1) ? 1.0f : 0.0f;
    gain_play_    = (winner_ == 2) ? 1.0f : 0.0f;
    gain_vision_  = (winner_ == 3) ? 1.0f : 0.0f;

    auto pub = [&](std::string const& topic, float v){
        auto out = std::make_shared<ProprioToken>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("arbiter") : id_;
        out->sensor      = "arbiter_gain";
        out->values.resize(1);
        out->values[0]   = v;
        bus_->publish(topic, out);
    };
    pub(klino_gain_topic_,   gain_klino_);
    pub(planner_gain_topic_, gain_planner_);
    if (!play_gain_topic_.empty()) pub(play_gain_topic_, gain_play_);  // task #33 — 0 when inert/lost (consumer fires)
    if (!vision_gain_topic_.empty()) pub(vision_gain_topic_, gain_vision_);  // loop #4 — 0 when inert/lost (consumer fires)
}

nlohmann::json EFEArbiter::diag_snapshot() const {
    return nlohmann::json{
        {"scoring_mode", scoring_mode_},   // "value_race" (legacy) | "efe" (explicit-EFE precision scoring)
        {"raw_klino", raw_klino_},
        {"raw_planner", raw_planner_},     // planner's food-route value (0 while exploring)
        {"v_klino", v_klino_},             // selected klino score (value-race MAX, or G_klino in efe mode)
        {"v_planner", v_planner_},         // selected planner score (value-race LEVEL, or G_planner in efe mode)
        {"cap_klino", cap_klino_},         // klino's self-reported capability ∈[0,1]
        {"mean_klino", mean_klino_},       // klino's running raw baseline (legible value race)
        {"plan_peak", plan_peak_},         // planner's slow-decaying peak food-route value (value-race level denominator)
        // ---- explicit-EFE decomposition (efe mode; 0 in value_race) ----
        {"g_prag_klino", g_prag_klino_},   // hunger · reach-prob(klino)   — pragmatic, sensory precision
        {"g_prag_planner", g_prag_planner_}, // hunger · reach-prob(planner) — pragmatic, model precision
        {"g_epist_klino", g_epist_klino_}, // (1−hunger) · normalised z-spike — klino approach/epistemic
        {"g_epist_planner", g_epist_planner_}, // (1−hunger) · planner frontier novelty
        {"G_klino", G_klino_},             // g_prag_klino + g_epist_klino
        {"G_planner", G_planner_},         // g_prag_planner + g_epist_planner
        {"plan_novelty", plan_novelty_},   // planner frontier novelty ∈[0,1] (model epistemic input)
        {"plan_precision", plan_precision_}, // planner model precision ∈[0,1] (§2.3 controlled precision)
        // ---- play policy (task #33) ----
        {"play_active", play_active_},     // play participates in the race (weight>0 && wired && efe)
        {"play_value", play_value_},       // PlayLoop frontier value ∈[0,1] (epistemic map-growth potential)
        {"g_epist_play", g_epist_play_},   // play_weight · (1−hunger) · play_value
        {"v_play", v_play_},               // selected play score (= G_play)
        {"gain_play", gain_play_},
        // ---- vision policy (loop #4) ----
        {"vision_active", vision_active_}, // vision participates in the race (weight>0 && wired && efe)
        {"vision_value", vision_value_},   // VisualHomingNav sight-confidence ∈[0,1] (pragmatic close)
        {"g_prag_vision", g_prag_vision_}, // vision_weight · hunger · vision_value
        {"v_vision", v_vision_},           // selected vision score (= G_vision)
        {"gain_vision", gain_vision_},
        {"winner", winner_},               // 0 = klino, 1 = planner, 2 = play, 3 = vision
        {"gain_klino", gain_klino_},
        {"gain_planner", gain_planner_},
        {"margin", margin_},
        {"hunger", hunger_},
        {"scent", scent_},
    };
}

}  // namespace ogma
