#include "ogma/modules/VisualHomingNav.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }

template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("VisualHomingNav: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("VisualHomingNav: param '" + k + "' must be a string");
}
}  // namespace

std::string_view VisualHomingNav::type_name() const { return "VisualHomingNav"; }

std::vector<TopicSpec> VisualHomingNav::input_topics() const {
    return {
        TopicSpec{vision_epm_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
        TopicSpec{bearing_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{eat_topic_,        std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> VisualHomingNav::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
    if (!value_topic_.empty())
        v.push_back(TopicSpec{value_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema VisualHomingNav::params_schema() const {
    return {
        {"vision_epm_topic", ParamMutability::ConstructionOnly, "The vision-food EPM RealityToken (winner_id/tle/node_count) — the predictive substrate + informativeness.", ParamValue{std::string("reality.cognitive.vision")}},
        {"bearing_topic",    ParamMutability::ConstructionOnly, "VisualBearing egocentric food bearing ProprioToken [vx=+right, vy=+forward, proximity=green_frac].", ParamValue{std::string("percept.visual_bearing")}},
        {"heading_topic",    ParamMutability::ConstructionOnly, "Absolute heading (ProprioToken[0], rad) → ego↔allocentric target reprojection for persistence.", ParamValue{std::string("reality.proprio.heading")}},
        {"eat_topic",        ParamMutability::ConstructionOnly, "GROUND-TRUTH eat (EnvEvent) → eat-calibration teacher (fold green_frac at the eat into eat_green_).", ParamValue{std::string("events.eat")}},
        {"output_topic",     ParamMutability::ConstructionOnly, "Chosen vision bearing → HeadingController (vision channel).", ParamValue{std::string("percept.vision_bearing")}},
        {"value_topic",      ParamMutability::ConstructionOnly, "Publish vision_value ∈[0,1] (pragmatic sight-confidence) → the L2 EFE arbiter (weighted by hunger·vision_weight). Empty = no publish (default-off).", ParamValue{std::string("")}},
        {"min_conf",         ParamMutability::HotMutable, "Proximity (green_frac) floor below which NO food is in view (occluded) → value 0, bearing [0,0] (cede).", ParamValue{0.02}},
        {"node_ref",         ParamMutability::HotMutable, "node_count at which the EPM informativeness saturates to 1 (developed food-bearing structure). Derived scale, not a behaviour knob.", ParamValue{4.0}},
        {"eat_alpha",        ParamMutability::HotMutable, "EMA rate folding green_frac at each real eat into eat_green_ (the reach scale).", ParamValue{0.2}},
        {"green_bootstrap",  ParamMutability::HotMutable, "Pre-calibration reach scale (before the first eat teaches eat_green_) so vision participates while being taught by scent/collision eats.", ParamValue{0.15}},
        {"centroid_ema",     ParamMutability::HotMutable, "EMA smoothing of the output bearing (anti-jitter); 0 = pass-through.", ParamValue{0.0}},
        {"persist_decay",    ParamMutability::HotMutable, "VISUAL TARGET PERSISTENCE (object permanence): keep homing to the remembered food's ALLOCENTRIC bearing while occluded, decaying confidence at this per-tick rate (~1/persist_decay ticks of memory). Spans a pillar's occlusion shadow so vision sustains authority through it. 0 = OFF (per-tick reactive value).", ParamValue{0.0}},
        {"persist_floor",    ParamMutability::HotMutable, "Drop the remembered target when its confidence decays below this.", ParamValue{0.05}},
    };
}

ParamMap VisualHomingNav::current_params() const {
    ParamMap m;
    m["vision_epm_topic"] = ParamValue{vision_epm_topic_};
    m["bearing_topic"]    = ParamValue{bearing_topic_};
    m["heading_topic"]    = ParamValue{heading_topic_};
    m["eat_topic"]        = ParamValue{eat_topic_};
    m["output_topic"]     = ParamValue{output_topic_};
    m["value_topic"]      = ParamValue{value_topic_};
    m["min_conf"]         = ParamValue{double(min_conf_)};
    m["node_ref"]         = ParamValue{double(node_ref_)};
    m["eat_alpha"]        = ParamValue{double(eat_alpha_)};
    m["green_bootstrap"]  = ParamValue{double(green_bootstrap_)};
    m["centroid_ema"]     = ParamValue{double(centroid_ema_)};
    m["persist_decay"]    = ParamValue{double(persist_decay_)};
    m["persist_floor"]    = ParamValue{double(persist_floor_)};
    return m;
}

void VisualHomingNav::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "min_conf")        min_conf_        = float(get_double(value, k));
    else if (k == "node_ref")        node_ref_        = float(get_double(value, k));
    else if (k == "eat_alpha")       eat_alpha_       = float(get_double(value, k));
    else if (k == "green_bootstrap") green_bootstrap_ = float(get_double(value, k));
    else if (k == "centroid_ema")    centroid_ema_    = float(get_double(value, k));
    else if (k == "persist_decay")   persist_decay_   = float(get_double(value, k));
    else if (k == "persist_floor")   persist_floor_   = float(get_double(value, k));
}

void VisualHomingNav::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "vision_epm_topic", [&](auto const& v){ vision_epm_topic_ = get_string(v,"vision_epm_topic"); });
    apply_param(params, "bearing_topic",    [&](auto const& v){ bearing_topic_    = get_string(v,"bearing_topic"); });
    apply_param(params, "heading_topic",    [&](auto const& v){ heading_topic_    = get_string(v,"heading_topic"); });
    apply_param(params, "eat_topic",        [&](auto const& v){ eat_topic_        = get_string(v,"eat_topic"); });
    apply_param(params, "output_topic",     [&](auto const& v){ output_topic_     = get_string(v,"output_topic"); });
    apply_param(params, "value_topic",      [&](auto const& v){ value_topic_      = get_string(v,"value_topic"); });
    apply_param(params, "min_conf",        [&](auto const& v){ min_conf_        = float(get_double(v,"min_conf")); });
    apply_param(params, "node_ref",        [&](auto const& v){ node_ref_        = float(get_double(v,"node_ref")); });
    apply_param(params, "eat_alpha",       [&](auto const& v){ eat_alpha_       = float(get_double(v,"eat_alpha")); });
    apply_param(params, "green_bootstrap", [&](auto const& v){ green_bootstrap_ = float(get_double(v,"green_bootstrap")); });
    apply_param(params, "centroid_ema",    [&](auto const& v){ centroid_ema_    = float(get_double(v,"centroid_ema")); });
    apply_param(params, "persist_decay",   [&](auto const& v){ persist_decay_   = float(get_double(v,"persist_decay")); });
    apply_param(params, "persist_floor",   [&](auto const& v){ persist_floor_   = float(get_double(v,"persist_floor")); });

    if (!eat_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(eat_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_eat(p); }));
}

void VisualHomingNav::handle_eat(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (std::dynamic_pointer_cast<const EnvEvent>(payload)) eat_pending_ = true;
}

void VisualHomingNav::tick(uint64_t tick_id) {
    // ---- pull inputs by value (robust to gate + DAG order) ----
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(heading_topic_)))
        if (pt->values.size() > 0) cur_heading_ = float(pt->values[0]);
    float vx = 0.0f, vy = 0.0f, prox = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(bearing_topic_))) {
        if (pt->values.size() > 0) vx   = float(pt->values[0]);
        if (pt->values.size() > 1) vy   = float(pt->values[1]);
        if (pt->values.size() > 2) prox = float(pt->values[2]);   // proximity = green_frac
    }
    epm_tle_ = 0.0f; node_count_ = 0;
    if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(vision_epm_topic_))) {
        epm_tle_    = rt->tle;            // §1 predictive error on the food bearing
        node_count_ = rt->node_count;     // EPM food-structure informativeness proxy
    }

    // ---- occlusion gate: food in view this tick? (VisualBearing → [0,0], prox 0 when occluded) ----
    have_food_ = (prox > min_conf_);

    // ---- eat-calibration: fold green_frac at a real eat into eat_green_ (the reach scale) ----
    bool ate_this_tick = false;
    if (eat_pending_) {
        eat_pending_ = false;
        ate_this_tick = true;
        if (prox > 0.0f) {
            if (!have_eat_green_) { eat_green_ = prox; have_eat_green_ = true; }
            else                  eat_green_ += eat_alpha_ * (prox - eat_green_);
        }
        // the remembered food was REACHED (belief fulfilled) — and food_alternate moves it — so drop
        // the target; vision cedes to play/klino until the bug turns and re-detects the next food.
        have_target_ = false; tgt_conf_ = 0.0f;
    }
    float reach_scale = have_eat_green_ ? std::max(eat_green_, 1e-4f)
                                        : std::max(green_bootstrap_, 1e-4f);
    cap_vision_ = std::clamp(prox / reach_scale, 0.0f, 1.0f);

    // ---- EPM informativeness (developed food-bearing structure vs degenerate occluded node) ----
    informativeness_ = std::clamp(float(node_count_) / std::max(node_ref_, 1e-3f), 0.0f, 1.0f);

    // ---- pragmatic VALUE → arbiter (∈[0,1]); arbiter applies hunger·vision_weight ----
    // DETECTION/DIRECTION confidence, DISTANCE-INDEPENDENT: high whenever food is clearly SEEN
    // (green_gate) AND the EPM has learned a food-bearing structure (informativeness). NOT scaled by
    // cap_vision (reach): the eat-calibrated cap is a proximity signal (big blob = close), which is
    // high only when klino already SMELLS the food — squeezing vision out of its own regime (seen but
    // not smelled). klino's scent already encodes range; vision's bearing is a DIRECTION, so its value
    // is "I can lead you to food," not "am I about to eat." cap_vision is kept for telemetry / the
    // arbiter's own close-vs-explore logic. (Measured: cap-scaled value never won even seeing food 10%
    // of ticks at value 0.8, because far food — where vision is needed — has a small blob → low cap.)
    float detect_value = have_food_ ? informativeness_ : 0.0f;

    // ---- VISUAL TARGET PERSISTENCE (object permanence) + bearing out ----
    // have_food → refresh the target belief (remember the ALLOCENTRIC bearing = ego + heading, invariant
    //   under the bug's own rotation) and home to the live bearing.
    // occluded but a target is remembered → KEEP HOMING to it (reproject allocentric → egocentric each
    //   tick), decaying the confidence — so vision sustains arbiter authority across a pillar's shadow.
    // A real eat drops the target (above). This is a predictive belief (§1): act to confirm "food is
    // still there." persist_decay=0 ⇒ OFF (per-tick reactive value, prior behaviour).
    float ox = 0.0f, oy = 0.0f;
    persisting_ = false;
    if (ate_this_tick) {
        // just reached food → belief fulfilled + dropped above; cede this tick (don't re-home a stale frame)
        value_ = 0.0f; have_ema_ = false;
    } else if (have_food_) {
        float ego = std::atan2(vx, vy);                       // egocentric bearing (0 = forward, + = right)
        tgt_world_bearing_ = wrap_pi(ego + cur_heading_);     // → allocentric (rotation-invariant)
        have_target_ = true;
        tgt_conf_    = detect_value;
        value_       = detect_value;
        if (centroid_ema_ > 0.0f) {
            if (!have_ema_) { ema_vx_ = vx; ema_vy_ = vy; have_ema_ = true; }
            else { ema_vx_ += centroid_ema_ * (vx - ema_vx_); ema_vy_ += centroid_ema_ * (vy - ema_vy_); }
            ox = ema_vx_; oy = ema_vy_;
        } else { ox = vx; oy = vy; }
    } else if (have_target_ && persist_decay_ > 0.0f) {
        tgt_conf_ *= (1.0f - persist_decay_);
        if (tgt_conf_ < persist_floor_) {
            have_target_ = false; tgt_conf_ = 0.0f; value_ = 0.0f; have_ema_ = false;
        } else {
            persisting_ = true;
            float ego = wrap_pi(tgt_world_bearing_ - cur_heading_);   // reproject allocentric → egocentric
            ox = std::sin(ego); oy = std::cos(ego);
            value_ = tgt_conf_;
        }
    } else {
        value_ = 0.0f; have_ema_ = false;
    }
    out_vx_ = ox; out_vy_ = oy;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("vision_homing") : id_;
    out->sensor      = "vision_bearing";
    out->values.resize(3);
    out->values[0] = ox; out->values[1] = oy; out->values[2] = 0.0f;
    bus_->publish(output_topic_, out);

    if (!value_topic_.empty()) {
        auto vout = std::make_shared<ProprioToken>();
        vout->tick_id     = tick_id;
        vout->producer_id = id_.empty() ? std::string("vision_homing") : id_;
        vout->sensor      = "vision_value";
        vout->values.resize(1);
        vout->values[0]   = value_;
        bus_->publish(value_topic_, vout);
    }
}

nlohmann::json VisualHomingNav::diag_snapshot() const {
    return nlohmann::json{
        {"have_food", have_food_},
        {"have_target", have_target_},         // a remembered food target is held (persistence)
        {"persisting", persisting_},           // homing to the remembered target this tick (food occluded)
        {"tgt_conf", tgt_conf_},               // decaying confidence in the remembered target
        {"value", value_},                     // vision_value ∈[0,1] → arbiter
        {"cap_vision", cap_vision_},           // eat-calibrated reach confidence
        {"eat_green", eat_green_},             // learned green_frac at the eat
        {"informativeness", informativeness_}, // EPM food-structure trust
        {"epm_tle", epm_tle_},                 // vision-food EPM TLE (§1 predictive error)
        {"node_count", node_count_},
        {"vx", out_vx_}, {"vy", out_vy_},
    };
}

}  // namespace ogma
