// =============================================================================
// SaccadeReflex.cpp  --  saccadic learning-walk (Pathway C1)
// =============================================================================
#include "ogma/modules/SaccadeReflex.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("SaccadeReflex: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("SaccadeReflex: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("SaccadeReflex: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("SaccadeReflex: param '" + k + "' must be bool");
}
} // namespace

SaccadeReflex::SaccadeReflex()  = default;
SaccadeReflex::~SaccadeReflex() = default;

std::string_view SaccadeReflex::type_name() const { return "SaccadeReflex"; }

std::vector<TopicSpec> SaccadeReflex::input_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{vel_topic_, std::type_index(typeid(ProprioToken)),
                              SubscriptionKind::Direct, /*required=*/false} };
    if (!boredom_topic_.empty())
        v.push_back(TopicSpec{boredom_topic_, std::type_index(typeid(ReflexGate)), SubscriptionKind::Direct, false});
    if (!hunger_topic_.empty())
        v.push_back(TopicSpec{hunger_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!scent_topic_.empty())
        v.push_back(TopicSpec{scent_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!novelty_topic_.empty())
        v.push_back(TopicSpec{novelty_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> SaccadeReflex::output_topics() const {
    return {
        TopicSpec{output_topic_left_,  std::type_index(typeid(ActionOut))},
        TopicSpec{output_topic_right_, std::type_index(typeid(ActionOut))},
        TopicSpec{active_topic_,       std::type_index(typeid(ProprioToken))},
    };
}

ParamSchema SaccadeReflex::params_schema() const {
    return {
        {"vel_topic", ParamMutability::ConstructionOnly,
            "Egocentric velocity ProprioToken [v_right,v_fwd] — distance-travelled trigger.",
            ParamValue{std::string("reality.proprio.vel_ego")}},
        {"output_topic_left", ParamMutability::ConstructionOnly,
            "Left-paddle ActionOut (→ MotorBus 'saccade' lr channel).",
            ParamValue{std::string("saccade.left")}},
        {"output_topic_right", ParamMutability::ConstructionOnly,
            "Right-paddle ActionOut.", ParamValue{std::string("saccade.right")}},
        {"active_topic", ParamMutability::ConstructionOnly,
            "Scalar ProprioToken = 1.0 while pivoting (the cylinder builder's gate).",
            ParamValue{std::string("saccade.active")}},
        {"enable", ParamMutability::HotMutable, "Master enable.", ParamValue{true}},
        {"trigger_mode", ParamMutability::HotMutable,
            "'distance' = BOOTSTRAP (fire after travelling travel_trigger). 'epistemic' = "
            "Pathway B: explore = NO FORAGING PROGRESS × HUNGER, scan only in a novel place.",
            ParamValue{std::string("distance")}},
        {"scent_topic", ParamMutability::ConstructionOnly,
            "reality.proprio.scent_max → the foraging-progress EMA (short−long). Empty = no "
            "progress gate (always 'not progressing').", ParamValue{std::string("")}},
        {"hunger_topic", ParamMutability::ConstructionOnly,
            "reality.proprio.hunger (1−energy) → explore only when hungry. Empty = no hunger gate.",
            ParamValue{std::string("")}},
        {"short_alpha", ParamMutability::HotMutable, "Fast EMA rate on scent_max.", ParamValue{0.05}},
        {"long_alpha",  ParamMutability::HotMutable, "Slow EMA rate on scent_max.", ParamValue{0.005}},
        {"progress_gate", ParamMutability::HotMutable,
            "Explore when (short−long scent EMA) < this (scent not rising = stalled).", ParamValue{0.001}},
        {"hunger_gate", ParamMutability::HotMutable,
            "Explore only when hunger > this.", ParamValue{0.3}},
        {"boredom_topic", ParamMutability::ConstructionOnly,
            "OPTIONAL physical-stuck gate (DistressDrive cognition.boredom ReflexGate); empty = off.",
            ParamValue{std::string("")}},
        {"boredom_gate", ParamMutability::HotMutable,
            "(optional) physical-stuck threshold.", ParamValue{0.3}},
        {"novelty_topic", ParamMutability::ConstructionOnly,
            "Perceptual-novelty signal — a vision-EPM RealityToken (reads .tle); empty = no "
            "novelty requirement.", ParamValue{std::string("")}},
        {"scent_gate", ParamMutability::HotMutable,
            "Epistemic mode fires only when scent_max < this (the bug is not homing).",
            ParamValue{0.05}},
        {"novelty_threshold", ParamMutability::HotMutable,
            "Epistemic mode fires only when the vision-EPM TLE > this (unfamiliar place).",
            ParamValue{0.3}},
        {"travel_trigger", ParamMutability::HotMutable,
            "Accumulated |vel_ego| before a saccade fires (distance mode).",
            ParamValue{3.0}},
        {"pivot_ticks", ParamMutability::HotMutable,
            "Sweep duration in ticks (≈ one clock period so phase ≈ sweep heading).",
            ParamValue{int64_t{120}}},
        {"spin_rate", ParamMutability::HotMutable,
            "Differential-rotation magnitude during the pivot (pure in-place spin).",
            ParamValue{4.0}},
        {"refractory_ticks", ParamMutability::HotMutable,
            "Min ticks after a saccade before the next can fire.", ParamValue{int64_t{120}}},
    };
}

ParamMap SaccadeReflex::current_params() const {
    ParamMap m;
    m["vel_topic"]         = ParamValue{vel_topic_};
    m["output_topic_left"] = ParamValue{output_topic_left_};
    m["output_topic_right"]= ParamValue{output_topic_right_};
    m["active_topic"]      = ParamValue{active_topic_};
    m["enable"]            = ParamValue{enable_};
    m["trigger_mode"]      = ParamValue{std::string(trigger_mode_ == 1 ? "epistemic" : "distance")};
    m["scent_topic"]       = ParamValue{scent_topic_};
    m["hunger_topic"]      = ParamValue{hunger_topic_};
    m["short_alpha"]       = ParamValue{double(short_alpha_)};
    m["long_alpha"]        = ParamValue{double(long_alpha_)};
    m["progress_gate"]     = ParamValue{double(progress_gate_)};
    m["hunger_gate"]       = ParamValue{double(hunger_gate_)};
    m["boredom_topic"]     = ParamValue{boredom_topic_};
    m["boredom_gate"]      = ParamValue{double(boredom_gate_)};
    m["novelty_topic"]     = ParamValue{novelty_topic_};
    m["scent_gate"]        = ParamValue{double(scent_gate_)};
    m["novelty_threshold"] = ParamValue{double(novelty_threshold_)};
    m["travel_trigger"]    = ParamValue{double(travel_trigger_)};
    m["pivot_ticks"]       = ParamValue{int64_t(pivot_ticks_)};
    m["spin_rate"]         = ParamValue{double(spin_rate_)};
    m["refractory_ticks"]  = ParamValue{int64_t(refractory_ticks_)};
    return m;
}

void SaccadeReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("SaccadeReflex requires a non-null Bus");

    apply_param(params, "vel_topic",         [&](auto const& v){ vel_topic_          = get_string(v,"vel_topic"); });
    apply_param(params, "output_topic_left", [&](auto const& v){ output_topic_left_  = get_string(v,"output_topic_left"); });
    apply_param(params, "output_topic_right",[&](auto const& v){ output_topic_right_ = get_string(v,"output_topic_right"); });
    apply_param(params, "active_topic",      [&](auto const& v){ active_topic_       = get_string(v,"active_topic"); });
    apply_param(params, "enable",            [&](auto const& v){ enable_             = get_bool(v,"enable"); });
    apply_param(params, "trigger_mode",      [&](auto const& v){ trigger_mode_       = (get_string(v,"trigger_mode") == "epistemic") ? 1 : 0; });
    apply_param(params, "scent_topic",       [&](auto const& v){ scent_topic_        = get_string(v,"scent_topic"); });
    apply_param(params, "hunger_topic",      [&](auto const& v){ hunger_topic_       = get_string(v,"hunger_topic"); });
    apply_param(params, "short_alpha",       [&](auto const& v){ short_alpha_        = float(get_double(v,"short_alpha")); });
    apply_param(params, "long_alpha",        [&](auto const& v){ long_alpha_         = float(get_double(v,"long_alpha")); });
    apply_param(params, "progress_gate",     [&](auto const& v){ progress_gate_      = float(get_double(v,"progress_gate")); });
    apply_param(params, "hunger_gate",       [&](auto const& v){ hunger_gate_        = float(get_double(v,"hunger_gate")); });
    apply_param(params, "boredom_topic",     [&](auto const& v){ boredom_topic_      = get_string(v,"boredom_topic"); });
    apply_param(params, "boredom_gate",      [&](auto const& v){ boredom_gate_       = float(get_double(v,"boredom_gate")); });
    apply_param(params, "novelty_topic",     [&](auto const& v){ novelty_topic_      = get_string(v,"novelty_topic"); });
    apply_param(params, "scent_gate",        [&](auto const& v){ scent_gate_         = float(get_double(v,"scent_gate")); });
    apply_param(params, "novelty_threshold", [&](auto const& v){ novelty_threshold_  = float(get_double(v,"novelty_threshold")); });
    apply_param(params, "travel_trigger",    [&](auto const& v){ travel_trigger_     = float(get_double(v,"travel_trigger")); });
    apply_param(params, "pivot_ticks",       [&](auto const& v){ pivot_ticks_        = int(get_int(v,"pivot_ticks")); });
    apply_param(params, "spin_rate",         [&](auto const& v){ spin_rate_          = float(get_double(v,"spin_rate")); });
    apply_param(params, "refractory_ticks",  [&](auto const& v){ refractory_ticks_   = int(get_int(v,"refractory_ticks")); });

    if (pivot_ticks_ < 1) pivot_ticks_ = 1;

    if (!vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vel(p); }));
    if (!scent_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    if (!novelty_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(novelty_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_novelty(p); }));
}

void SaccadeReflex::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "enable")           enable_           = get_bool(value, k);
    else if (k == "trigger_mode")     trigger_mode_     = (get_string(value, k) == "epistemic") ? 1 : 0;
    else if (k == "short_alpha")      short_alpha_      = float(get_double(value, k));
    else if (k == "long_alpha")       long_alpha_       = float(get_double(value, k));
    else if (k == "progress_gate")    progress_gate_    = float(get_double(value, k));
    else if (k == "hunger_gate")      hunger_gate_      = float(get_double(value, k));
    else if (k == "boredom_gate")     boredom_gate_     = float(get_double(value, k));
    else if (k == "scent_gate")       scent_gate_       = float(get_double(value, k));
    else if (k == "novelty_threshold")novelty_threshold_= float(get_double(value, k));
    else if (k == "travel_trigger")   travel_trigger_   = float(get_double(value, k));
    else if (k == "pivot_ticks")      pivot_ticks_      = std::max(1, int(get_int(value, k)));
    else if (k == "spin_rate")        spin_rate_        = float(get_double(value, k));
    else if (k == "refractory_ticks") refractory_ticks_ = int(get_int(value, k));
    else throw std::invalid_argument("SaccadeReflex: param '" + k + "' is construction-only / unknown");
}

void SaccadeReflex::handle_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    float vr = float(pt->values[0]);
    float vf = float(pt->values[1]);
    speed_ = std::sqrt(vr * vr + vf * vf);
}

void SaccadeReflex::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) scent_max_ = float(pt->values[0]);
}

void SaccadeReflex::handle_novelty(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (rt) novelty_tle_ = float(rt->tle);   // vision-EPM prediction error = perceptual surprise
}

void SaccadeReflex::tick(uint64_t tick_id) {
    // Pull the latest scent / novelty by value (robust to input-gate + DAG-order
    // timing — the push handlers can be gated out for conditionally-declared inputs).
    if (trigger_mode_ == 1) {
        if (!scent_topic_.empty()) {
            if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(scent_topic_)))
                if (pt->values.size() > 0) scent_max_ = float(pt->values[0]);
        }
        // Foraging-progress EMAs: short − long > 0 = scent rising = approaching food.
        if (!have_scent_ema_) { scent_short_ = scent_long_ = scent_max_; have_scent_ema_ = true; }
        else {
            scent_short_ += short_alpha_ * (scent_max_ - scent_short_);
            scent_long_  += long_alpha_  * (scent_max_ - scent_long_);
        }
        if (!hunger_topic_.empty()) {
            if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(hunger_topic_)))
                if (pt->values.size() > 0) hunger_ = float(pt->values[0]);
        }
        if (!novelty_topic_.empty()) {
            if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(novelty_topic_)))
                novelty_tle_ = float(rt->tle);
        }
        if (!boredom_topic_.empty()) {
            if (auto rg = std::dynamic_pointer_cast<const ReflexGate>(bus_->last_value(boredom_topic_)))
                boredom_ = float(rg->value);
        }
    }

    // --- FSM advance ---
    if (!enable_) { state_ = 0; ticks_left_ = 0; }
    else {
        switch (state_) {
            case 0: {  // IDLE — decide whether to fire a saccade
                bool fire = false;
                if (trigger_mode_ == 1) {
                    // EPISTEMIC explore = NO FORAGING PROGRESS × HUNGER, in a NOVEL place.
                    bool stalled = (scent_short_ - scent_long_) < progress_gate_;   // scent not rising
                    bool hungry  = hunger_topic_.empty()  || hunger_     > hunger_gate_;
                    bool novel   = novelty_topic_.empty() || novelty_tle_ > novelty_threshold_;
                    bool not_phys_stuck_ok = boredom_topic_.empty() || boredom_ > boredom_gate_;  // optional
                    fire = stalled && hungry && novel && not_phys_stuck_ok;
                } else {
                    // DISTANCE bootstrap: fire after travelling travel_trigger.
                    dist_accum_ += speed_;
                    fire = dist_accum_ >= travel_trigger_;
                }
                if (fire) {
                    state_ = 1; ticks_left_ = pivot_ticks_; dist_accum_ = 0.0f;
                    ++saccade_count_;
                }
                break;
            }
            case 1:  // PIVOT — spin in place for pivot_ticks
                if (--ticks_left_ <= 0) { state_ = 2; ticks_left_ = refractory_ticks_; }
                break;
            case 2:  // REFRACTORY
                if (--ticks_left_ <= 0) { state_ = 0; }
                break;
        }
    }

    // --- output ---
    // PIVOT: pure differential rotation (left=+spin, right=−spin → common-mode 0 →
    // zero forward → spins in place). Idle/refractory: silent (forager drives).
    float spin = (state_ == 1) ? spin_rate_ : 0.0f;
    last_spin_ = spin;

    auto al = std::make_shared<ActionOut>();
    al->tick_id = tick_id; al->producer_id = id_.empty() ? std::string("saccade") : id_;
    al->accel = +spin;
    bus_->publish(output_topic_left_, al);

    auto ar = std::make_shared<ActionOut>();
    ar->tick_id = tick_id; ar->producer_id = al->producer_id;
    ar->accel = -spin;
    bus_->publish(output_topic_right_, ar);

    auto act = std::make_shared<ProprioToken>();
    act->tick_id = tick_id; act->producer_id = al->producer_id; act->sensor = "saccade_active";
    act->values.resize(1);
    act->values[0] = (state_ == 1) ? 1.0f : 0.0f;
    bus_->publish(active_topic_, act);
}

nlohmann::json SaccadeReflex::snapshot_state() const {
    return nlohmann::json{{"version", 1}};
}

nlohmann::json SaccadeReflex::diag_snapshot() const {
    return nlohmann::json{
        {"state", state_},                 // 0 idle / 1 pivot / 2 refractory
        {"pivoting", state_ == 1},
        {"dist", dist_accum_},
        {"ticks_left", ticks_left_},
        {"count", saccade_count_},
        {"mode", trigger_mode_},                       // 0 distance / 1 epistemic
        {"novelty", novelty_tle_},                     // vision-EPM surprise
        {"progress", scent_short_ - scent_long_},      // foraging progress (>0 approaching)
        {"hunger", hunger_},                           // 1−energy
        {"boredom", boredom_},                         // optional physical-stuck
    };
}

void SaccadeReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    if (s.value("version", 0) != 1)
        throw std::runtime_error("SaccadeReflex::restore_state: unknown version");
}

} // namespace ogma
