#include "ogma/modules/PlaceGraphPlanner.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("PlaceGraphPlanner: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("PlaceGraphPlanner: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PlaceGraphPlanner: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("PlaceGraphPlanner: param '" + k + "' must be bool");
}
}  // namespace

float PlaceGraphPlanner::Edge::heading() const {
    if (count == 0) return 0.0f;
    return std::atan2(sum_sin, sum_cos);   // circular mean of unit vectors
}

std::string_view PlaceGraphPlanner::type_name() const { return "PlaceGraphPlanner"; }

std::vector<TopicSpec> PlaceGraphPlanner::input_topics() const {
    std::vector<TopicSpec> v{
        TopicSpec{place_topic_,         std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_,       std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{hunger_topic_,        std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{eat_topic_,           std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,           std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
    };
    if (!vision_topic_.empty())
        v.push_back(TopicSpec{vision_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> PlaceGraphPlanner::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
    if (!plan_value_topic_.empty())
        v.push_back(TopicSpec{plan_value_topic_, std::type_index(typeid(ProprioToken))});
    if (!plan_precision_topic_.empty())
        v.push_back(TopicSpec{plan_precision_topic_, std::type_index(typeid(ProprioToken))});
    if (!plan_novelty_topic_.empty())
        v.push_back(TopicSpec{plan_novelty_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema PlaceGraphPlanner::params_schema() const {
    return {
        {"place_topic",         ParamMutability::ConstructionOnly, "Current place (RealityToken.winner_id).", ParamValue{std::string("reality.cognitive.place")}},
        {"heading_topic",       ParamMutability::ConstructionOnly, "Absolute heading (ProprioToken[0]).",     ParamValue{std::string("reality.proprio.heading")}},
        {"vision_topic", ParamMutability::ConstructionOnly, "Visual bearing — fine line-of-sight final approach; empty = no vision.", ParamValue{std::string("")}},
        {"vision_floor", ParamMutability::HotMutable, "Home on vision when |visual bearing| > this (food in view).", ParamValue{0.05}},
        {"hunger_topic",        ParamMutability::ConstructionOnly, "hunger (1−energy) → plan gate.",            ParamValue{std::string("reality.proprio.hunger")}},
        {"eat_topic",           ParamMutability::ConstructionOnly, "GROUND-TRUTH eat event (EnvEvent, e.g. events.eat) → tag the current place with food memory. NOT events.hit: in the Cell that topic is ALSO published on the scent-progress inference (short>long×1.5) all through an approach, which pumps food_reward every tick and inflates phantom caches the planner then obsessively circles.", ParamValue{std::string("events.eat")}},
        {"vel_topic",           ParamMutability::ConstructionOnly, "Forward speed (ProprioToken[0]) → path-integration odometry.", ParamValue{std::string("reality.proprio.vel_ego")}},
        {"output_topic",        ParamMutability::ConstructionOnly, "Chosen nav bearing → HeadingController.",   ParamValue{std::string("percept.nav_bearing")}},
        {"plan_value_topic",    ParamMutability::ConstructionOnly, "Publish the FOOD-ROUTE value (value(next_node) when food_known && route_exists, else 0) as a ProprioToken scalar → the L2 EFE arbiter. Sustained-high while routing to remembered food, 0 while merely exploring. Empty = no publish.", ParamValue{std::string("")}},
        {"plan_precision_topic", ParamMutability::ConstructionOnly, "§2.3 MODEL PRECISION → explicit-EFE arbiter: sharpness of the food belief = 1 − H(food_dist)/log N (single known cache = 1, food spread over many caches → lower, empty map → next-hop value margin). Publishes a ProprioToken scalar ∈[0,1]. Empty = no publish.", ParamValue{std::string("")}},
        {"plan_novelty_topic",   ParamMutability::ConstructionOnly, "§2.2 EPISTEMIC term → explicit-EFE arbiter: the frontier TLE the planner routes toward when there is NO committed food route (normalised by its own running peak, §6), ~0 while EXPLOITING a food route. Publishes a ProprioToken scalar ∈[0,1] → the arbiter's g_epist_planner. Empty = no publish.", ParamValue{std::string("")}},
        {"wander_thrust", ParamMutability::HotMutable, "Forward bearing magnitude while exploring (B.2 wander).", ParamValue{1.0}},
        {"gamma",         ParamMutability::HotMutable, "Value-iteration discount.",                 ParamValue{0.85}},
        {"vi_sweeps",     ParamMutability::HotMutable, "Value-iteration sweeps per tick.",          ParamValue{int64_t{8}}},
        {"food_reward",   ParamMutability::HotMutable, "Value added to the current place on a hit.", ParamValue{1.0}},
        {"food_decay",    ParamMutability::HotMutable, "Per-tick food-value decay.",                ParamValue{0.9995}},
        {"tle_gain",      ParamMutability::HotMutable, "Epistemic weight: β·(place-EPM TLE) added to V[n] → explore the least-modelled frontier (0 = pure exploit).", ParamValue{0.5}},
        {"tle_ema_alpha", ParamMutability::HotMutable, "EMA rate for the per-node TLE novelty signal.", ParamValue{0.1}},
        {"tle_peak_decay", ParamMutability::HotMutable, "Slow decay of the node-TLE running peak (the plan_novelty normaliser, §6 — derived from the signal, not hand-set).", ParamValue{0.0005}},
        {"hab_rise",      ParamMutability::HotMutable, "HK habituation rise rate while dwelling on a node (recent = boring; fast). Structural timescale knob, like the methylation rate. 0 disables habituation.", ParamValue{0.1}},
        {"hab_decay",     ParamMutability::HotMutable, "HK habituation per-tick recovery (slow; ~1/hab_decay ticks). Sets how long the planner sweeps other ground before a region becomes interesting again.", ParamValue{0.002}},
        {"disconfirm",  ParamMutability::HotMutable, "DESPERATION disconfirmation rate (0 = off): a remembered cache the bug DWELLS on but does NOT eat at has its food belief decayed by disconfirm·hunger·habituation — faster the longer it camps and the hungrier it is. An empty cache the bug keeps returning to fades, the value field FLATTENS, and the planner drops into its fast run-and-tumble WANDER branch to search new ground instead of circling a dead cache until it starves. A real cache (where it eats) is re-reinforced on the hit → productive caches persist.", ParamValue{0.0}},
        {"escape_gain", ParamMutability::HotMutable, "CONFINEMENT STEER (0 = off): late in a run the dense value field always has an uphill neighbour (stale food, TLE, or γ-propagation), so route_exists never clears and the planner ORBITS a few habituated nodes until it starves even while BORED + HUNGRY. Instead of wandering, STEER the route toward FRESH ground: each neighbour's value is penalised by escape_gain·hunger·stale·habituation, where stale=clamp(ticks-since-the-map-last-grew · hab_decay) rises only when the bug RE-TREADS known ground (≈0 while it forages/discovers new nodes → the steered value IS the raw value → routing unchanged → genuine hungry routes to real food are never disrupted, no foraging regression). The bug keeps ROUTING (graph-native, scales to complex mazes) but climbs OUT of the exhausted basin toward the least-re-tread neighbour → the boundary → fresh ground. Self-limiting (new ground → stale resets → steer→0).", ParamValue{0.0}},
        {"patrol_mode", ParamMutability::HotMutable, "COVERAGE PATROLLER (0/false = the food-memory planner, default). The planner's job = build + TRAVERSE its map (visit all known places, prefer least-recently-visited), NOT remember food (a liability where the source relocates → stale route → dither → starve). true: drop food+TLE from the value; V[n] = coverage_gain·(1−hab[n]) + γ·maxV routes to the least-recently-visited known node (the habituation field IS the coverage need), always flowing to a reachable adjacent node → never fixates → never stuck. plan_value → the frontier coverage need (1−hab[next]) ∈[0,1] (hunger-gated in the arbiter: patrol when hungry, yield to klino on scent / play when full). The 3-loop division: play GROWS the map (novelty), klino CLOSES on scent, planner PATROLS known ground so klino keeps getting scent chances.", ParamValue{false}},
        {"patrol_fallback", ParamMutability::HotMutable, "HYBRID PATROLLER (opt-in, default false = byte-identical; needs planner_epistemic=true in the arbiter to score). Keeps food memory AND makes the planner PATROL known ground when it has no food route. Two parts: (1) a coverage_gain baseline in the value field so the planner ROUTES toward the least-recently-visited known node (never silent/stuck); (2) the frontier COVERAGE NEED (1−hab[next]) is published on the EPISTEMIC channel (plan_novelty), so the arbiter weights it by (1−hunger)=energy surplus — the planner patrols when FULL (alongside play), routes to remembered food when hungry (plan_value, pragmatic), and YIELDS to klino's foraging when hungry. A hunger-gated PRAGMATIC patrol instead crowds klino → starve; the epistemic split is the fix. The 3-loop division: play GROWS the map, planner PATROLS it, klino CLOSES on scent. Unlike patrol_mode this does NOT drop food memory.", ParamValue{false}},
        {"coverage_gain", ParamMutability::HotMutable, "patrol_mode / patrol_fallback: the uniform per-node 'want to visit' drive (the value scale). 1.0 default.", ParamValue{1.0}},
        {"stall_factor", ParamMutability::HotMutable, "ROUTE-EXECUTION ROBUSTNESS (0 = off): a capable planner never DITHERS on a route it can't execute (a committed hop pointing through a wall → the bug spins in place → starves). When the current hop takes more than stall_factor × the bug's OWN typical hop-traversal time (no node transition), the route is blocked → CEDE (plan_value→0) so the arbiter hands authority to klino (forage) / play (explore), which move the bug off the stuck spot; a transition resets it and routing resumes. The timeout is derived from the running EMA of successful hop durations (no fixed tick constant). ~4 = cede after 4× the normal traversal time.", ParamValue{0.0}},
        {"explore_cycle",        ParamMutability::HotMutable, "Run-and-tumble explore: run length (ticks) before a tumble decision.", ParamValue{int64_t{30}}},
        {"explore_tumble_range", ParamMutability::HotMutable, "Run-and-tumble explore: max reorient per tumble (rad).", ParamValue{1.5708}},
        {"explore_seed",         ParamMutability::ConstructionOnly, "Run-and-tumble explore RNG seed.", ParamValue{int64_t{11}}},
        {"pi_cell_size",  ParamMutability::HotMutable, "Path-integration place-code: >0 = place node IS the odometry grid cell (metres); 0 = use place_topic (panorama).", ParamValue{0.0}},
    };
}

ParamMap PlaceGraphPlanner::current_params() const {
    ParamMap m;
    m["place_topic"]         = ParamValue{place_topic_};
    m["heading_topic"]       = ParamValue{heading_topic_};
    m["vision_topic"]        = ParamValue{vision_topic_};
    m["vision_floor"]        = ParamValue{double(vision_floor_)};
    m["hunger_topic"]        = ParamValue{hunger_topic_};
    m["eat_topic"]           = ParamValue{eat_topic_};
    m["vel_topic"]           = ParamValue{vel_topic_};
    m["output_topic"]        = ParamValue{output_topic_};
    m["plan_value_topic"]    = ParamValue{plan_value_topic_};
    m["plan_precision_topic"] = ParamValue{plan_precision_topic_};
    m["plan_novelty_topic"]   = ParamValue{plan_novelty_topic_};
    m["wander_thrust"] = ParamValue{double(wander_thrust_)};
    m["gamma"]         = ParamValue{double(gamma_)};
    m["vi_sweeps"]     = ParamValue{int64_t(vi_sweeps_)};
    m["food_reward"]   = ParamValue{double(food_reward_)};
    m["food_decay"]    = ParamValue{double(food_decay_)};
    m["tle_gain"]      = ParamValue{double(tle_gain_)};
    m["tle_ema_alpha"] = ParamValue{double(tle_ema_alpha_)};
    m["tle_peak_decay"] = ParamValue{double(tle_peak_decay_)};
    m["hab_rise"]      = ParamValue{double(hab_rise_)};
    m["hab_decay"]     = ParamValue{double(hab_decay_)};
    m["disconfirm"]  = ParamValue{double(disconfirm_)};
    m["escape_gain"] = ParamValue{double(escape_gain_)};
    m["patrol_mode"] = ParamValue{patrol_mode_};
    m["patrol_fallback"] = ParamValue{patrol_fallback_};
    m["coverage_gain"] = ParamValue{double(coverage_gain_)};
    m["stall_factor"] = ParamValue{double(stall_factor_)};
    m["explore_cycle"]        = ParamValue{int64_t(explore_cycle_)};
    m["explore_tumble_range"] = ParamValue{double(explore_tumble_range_)};
    m["explore_seed"]         = ParamValue{int64_t(explore_seed_)};
    m["pi_cell_size"]  = ParamValue{double(pi_cell_size_)};
    return m;
}

void PlaceGraphPlanner::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "vision_floor")  vision_floor_  = float(get_double(value, k));
    else if (k == "wander_thrust") wander_thrust_ = float(get_double(value, k));
    else if (k == "gamma")         gamma_         = float(get_double(value, k));
    else if (k == "vi_sweeps")     vi_sweeps_     = int(get_int(value, k));
    else if (k == "food_reward")   food_reward_   = float(get_double(value, k));
    else if (k == "food_decay")    food_decay_    = float(get_double(value, k));
    else if (k == "tle_gain")      tle_gain_      = float(get_double(value, k));
    else if (k == "tle_ema_alpha") tle_ema_alpha_ = float(get_double(value, k));
    else if (k == "tle_peak_decay") tle_peak_decay_ = float(get_double(value, k));
    else if (k == "hab_rise")      hab_rise_      = float(get_double(value, k));
    else if (k == "hab_decay")     hab_decay_     = float(get_double(value, k));
    else if (k == "disconfirm")  disconfirm_  = float(get_double(value, k));
    else if (k == "escape_gain") escape_gain_ = float(get_double(value, k));
    else if (k == "patrol_mode") patrol_mode_ = get_bool(value, k);
    else if (k == "patrol_fallback") patrol_fallback_ = get_bool(value, k);
    else if (k == "coverage_gain") coverage_gain_ = float(get_double(value, k));
    else if (k == "stall_factor") stall_factor_ = float(get_double(value, k));
    else if (k == "explore_cycle")        explore_cycle_        = int(get_int(value, k));
    else if (k == "explore_tumble_range") explore_tumble_range_ = float(get_double(value, k));
    else if (k == "pi_cell_size")  pi_cell_size_  = float(get_double(value, k));
}

void PlaceGraphPlanner::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "place_topic",         [&](auto const& v){ place_topic_         = get_string(v,"place_topic"); });
    apply_param(params, "heading_topic",       [&](auto const& v){ heading_topic_       = get_string(v,"heading_topic"); });
    apply_param(params, "vision_topic",        [&](auto const& v){ vision_topic_        = get_string(v,"vision_topic"); });
    apply_param(params, "vision_floor",        [&](auto const& v){ vision_floor_        = float(get_double(v,"vision_floor")); });
    apply_param(params, "hunger_topic",        [&](auto const& v){ hunger_topic_        = get_string(v,"hunger_topic"); });
    apply_param(params, "eat_topic",           [&](auto const& v){ eat_topic_           = get_string(v,"eat_topic"); });
    apply_param(params, "vel_topic",           [&](auto const& v){ vel_topic_           = get_string(v,"vel_topic"); });
    apply_param(params, "output_topic",        [&](auto const& v){ output_topic_        = get_string(v,"output_topic"); });
    apply_param(params, "plan_value_topic",    [&](auto const& v){ plan_value_topic_    = get_string(v,"plan_value_topic"); });
    apply_param(params, "plan_precision_topic", [&](auto const& v){ plan_precision_topic_ = get_string(v,"plan_precision_topic"); });
    apply_param(params, "plan_novelty_topic",   [&](auto const& v){ plan_novelty_topic_   = get_string(v,"plan_novelty_topic"); });
    apply_param(params, "wander_thrust", [&](auto const& v){ wander_thrust_ = float(get_double(v,"wander_thrust")); });
    apply_param(params, "gamma",         [&](auto const& v){ gamma_         = float(get_double(v,"gamma")); });
    apply_param(params, "vi_sweeps",     [&](auto const& v){ vi_sweeps_     = int(get_int(v,"vi_sweeps")); });
    apply_param(params, "food_reward",   [&](auto const& v){ food_reward_   = float(get_double(v,"food_reward")); });
    apply_param(params, "food_decay",    [&](auto const& v){ food_decay_    = float(get_double(v,"food_decay")); });
    apply_param(params, "tle_gain",      [&](auto const& v){ tle_gain_      = float(get_double(v,"tle_gain")); });
    apply_param(params, "tle_ema_alpha", [&](auto const& v){ tle_ema_alpha_ = float(get_double(v,"tle_ema_alpha")); });
    apply_param(params, "tle_peak_decay", [&](auto const& v){ tle_peak_decay_ = float(get_double(v,"tle_peak_decay")); });
    apply_param(params, "hab_rise",      [&](auto const& v){ hab_rise_      = float(get_double(v,"hab_rise")); });
    apply_param(params, "hab_decay",     [&](auto const& v){ hab_decay_     = float(get_double(v,"hab_decay")); });
    apply_param(params, "disconfirm",  [&](auto const& v){ disconfirm_  = float(get_double(v,"disconfirm")); });
    apply_param(params, "escape_gain", [&](auto const& v){ escape_gain_ = float(get_double(v,"escape_gain")); });
    apply_param(params, "patrol_mode", [&](auto const& v){ patrol_mode_ = get_bool(v,"patrol_mode"); });
    apply_param(params, "patrol_fallback", [&](auto const& v){ patrol_fallback_ = get_bool(v,"patrol_fallback"); });
    apply_param(params, "coverage_gain", [&](auto const& v){ coverage_gain_ = float(get_double(v,"coverage_gain")); });
    apply_param(params, "stall_factor", [&](auto const& v){ stall_factor_ = float(get_double(v,"stall_factor")); });
    apply_param(params, "explore_cycle",        [&](auto const& v){ explore_cycle_        = int(get_int(v,"explore_cycle")); });
    apply_param(params, "explore_tumble_range", [&](auto const& v){ explore_tumble_range_ = float(get_double(v,"explore_tumble_range")); });
    apply_param(params, "explore_seed",         [&](auto const& v){ explore_seed_         = uint64_t(get_int(v,"explore_seed")); });
    explore_rng_.seed(explore_seed_);
    apply_param(params, "pi_cell_size",  [&](auto const& v){ pi_cell_size_  = float(get_double(v,"pi_cell_size")); });

    if (!eat_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(eat_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_hit(p); }));
}

void PlaceGraphPlanner::handle_hit(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    // Any EnvEvent on the configured eat_topic_ is the ground-truth food event (the topic is
    // subscription-specific — no name filter, which previously rejected the "eat" event name).
    if (std::dynamic_pointer_cast<const EnvEvent>(payload)) hit_in_window_ = true;
}

void PlaceGraphPlanner::run_value_iteration() {
    // Gauss-Seidel sweeps over the observed graph: V[n] = food[n] + gamma·max_m V[n→m].
    for (int s = 0; s < vi_sweeps_; ++s) {
        for (auto& [n, vn] : value_) {
            float best = 0.0f;
            auto it = edges_.find(n);
            if (it != edges_.end())
                for (auto const& [m, e] : it->second) best = std::max(best, value_[m]);
            auto tit = node_tle_.find(n);
            float ep = (tit != node_tle_.end()) ? tit->second : 0.0f;
            // HK habituation + hunger gating folded into the value field:
            //  • hunger weights the pragmatic (food) term → sated bug explores, hungry bug homes.
            //  • habituation suppresses the LOCAL term ONLY (food+TLE), not the propagated γ·best —
            //    a recently-visited node can still be a through-WAYPOINT, but it stops being a
            //    DESTINATION. Propagation then routes the gradient to the least-recent frontier.
            auto hit2 = hab_.find(n);
            float h = (hit2 != hab_.end()) ? hit2->second : 0.0f;
            // PATROL: a uniform "want to visit" drive per node → V = coverage_gain·(1−hab) + γ·maxV
            // routes toward the LEAST-RECENTLY-VISITED known node (visit all, prefer least-recent),
            // no food memory (which goes stale where the source relocates). Else the food-memory planner.
            // patrol_mode: pure coverage (no food). patrol_fallback: food+TLE PLUS a coverage baseline so
            // the field is never flat → the planner always has a least-recently-visited node to route to
            // (patrol) while a remembered food node still stands out by hunger·food (exploit when hungry).
            float local = patrol_mode_ ? coverage_gain_
                                       : (hunger_ * food_[n] + tle_gain_ * ep
                                          + (patrol_fallback_ ? coverage_gain_ : 0.0f));
            vn = local * (1.0f - h) + gamma_ * best;             // recent = boring; propagation routes to the least-recent frontier
        }
    }
}

float PlaceGraphPlanner::geo_bearing(int from, int to) const {
    // GEOMETRIC edge direction from path-integrated place centroids — robust to the
    // conflicting heading-of-travel (a place is entered from many directions).
    auto fit = place_pos_.find(from), tit = place_pos_.find(to);
    if (fit == place_pos_.end() || tit == place_pos_.end() || fit->second.n == 0 || tit->second.n == 0)
        return std::numeric_limits<float>::quiet_NaN();
    double fx = fit->second.sx / fit->second.n, fy = fit->second.sy / fit->second.n;
    double tx = tit->second.sx / tit->second.n, ty = tit->second.sy / tit->second.n;
    double dx = tx - fx, dy = ty - fy;
    if (dx == 0.0 && dy == 0.0) return std::numeric_limits<float>::quiet_NaN();
    // forward dir(h) = (-sin h, -cos h), so the heading to face toward (dx,dy) is atan2(-dx,-dy)
    return float(std::atan2(-dx, -dy));
}

void PlaceGraphPlanner::tick(uint64_t tick_id) {
    // ---- pull inputs by value (robust to gate + DAG order) ----
    int new_node = cur_node_;
    float place_tle = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(place_topic_))) {
        new_node = pt->winner_id;
        place_tle = pt->tle;   // world-model surprise at the current place → epistemic novelty signal
    }
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(heading_topic_)))
        if (pt->values.size() > 0) cur_heading_ = float(pt->values[0]);
    float vlat = 0.0f, vfwd = 0.0f;   // vel_ego = [local-right, local-forward] (÷move_speed)
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(vel_topic_))) {
        if (pt->values.size() > 0) vlat = float(pt->values[0]);
        if (pt->values.size() > 1) vfwd = float(pt->values[1]);
    }
    // path-integration: integrate the WORLD displacement. Body convention (rotation.y=heading,
    // forward = -basis.z): forward dir = (-sin h, -cos h), right dir = (cos h, -sin h).
    odo_x_ += -double(vfwd) * std::sin(cur_heading_) + double(vlat) * std::cos(cur_heading_);
    odo_y_ += -double(vfwd) * std::cos(cur_heading_) - double(vlat) * std::sin(cur_heading_);
    // PI place-code: the place node IS the fresh odometry grid cell (no panorama lag →
    // food tagged where the bug actually ate, geometric edges between cell centroids).
    if (pi_cell_size_ > 0.0f) {
        int gx = int(std::floor(odo_x_ / double(pi_cell_size_))) + 512;
        int gy = int(std::floor(odo_y_ / double(pi_cell_size_))) + 512;
        new_node = gx * 1024 + gy;
    }
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(hunger_topic_)))
        if (pt->values.size() > 0) hunger_ = float(pt->values[0]);

    // ---- map building: register the node + learn the edge-heading on a transition ----
    ++stale_ticks_;                                  // ticks since the map last grew (the confinement clock)
    bool node_transitioned = false;                  // did cur_node change this tick? (route-execution stall clock)
    if (new_node >= 0) {
        auto [vit, vins] = value_.emplace(new_node, 0.0f);
        if (vins) stale_ticks_ = 0;                  // discovered NEW ground → not confined, reset the clock
        food_.emplace(new_node, 0.0f);
        if (cur_node_ >= 0 && new_node != cur_node_) {
            node_transitioned = true;
            Edge& e = edges_[cur_node_][new_node];
            e.sum_sin += std::sin(cur_heading_);   // heading travelled cur_node_ → new_node
            e.sum_cos += std::cos(cur_heading_);
            e.count   += 1;
        }
        cur_node_ = new_node;
    }
    // accumulate the current place's position centroid (path-integration → geometric edges)
    if (cur_node_ >= 0) {
        PlacePos& pp = place_pos_[cur_node_];
        pp.sx += odo_x_; pp.sy += odo_y_; pp.n += 1;
    }
    // EPISTEMIC novelty: EMA the place-EPM's TLE onto the current node. Dynamic — high where the
    // world-model is worst, falls as the EPM LEARNS the place (not a count). Drives exploration
    // toward the least-modelled frontier via the value sweep below.
    if (cur_node_ >= 0) {
        auto [it, ins] = node_tle_.try_emplace(cur_node_, place_tle);
        if (!ins) it->second += tle_ema_alpha_ * (place_tle - it->second);
    }
    float local_tle = (cur_node_ >= 0 && node_tle_.count(cur_node_)) ? node_tle_.at(cur_node_) : 0.0f;

    // DESPERATION = hunger: how long since the bug last fed (energy depletes since the eat). It drives
    // the disconfirmation below (a hungrier bug abandons an unproductive cache faster) and is exposed
    // as telemetry so the demo can show "hunger rising → the planner lets go and searches out".
    desperation_ = hunger_;

    // HK HABITUATION ("recent = boring"): decay ALL nodes (slow recovery) then RISE the current
    // node (fast). A node the bug is sitting on saturates toward 1 (its local value is suppressed
    // in run_value_iteration → it stops being a destination); nodes it left recover over ~1/hab_decay
    // ticks. This is the de-oscillation drive: it pushes the routed value gradient toward the
    // least-recently-visited frontier so the planner SWEEPS the map instead of ping-ponging.
    for (auto& [n, hh] : hab_) hh *= (1.0f - hab_decay_);
    if (cur_node_ >= 0) { float& hc = hab_[cur_node_]; hc += hab_rise_ * (1.0f - hc); }

    // ---- food memory: reinforce on a real eat, DISCONFIRM a cache camped without one ----
    // On a hit the current place gets +food_reward. Otherwise, DISCONFIRMATION (active-inference belief
    // update): a remembered cache the bug dwells on but does NOT eat at is evidence the food is gone
    // (consumed / moved to the alternate spot). Decay its food belief by disconfirm_·desperation_·hab —
    // i.e. faster the longer it camps (hab) and the HUNGRIER it is (desperation). This is what breaks
    // the "circle a stale cache until you starve" cycle: an empty cache the bug keeps returning to fades,
    // the value field FLATTENS there, route_exists goes false, and the planner drops into its fast
    // run-and-tumble WANDER branch → it searches new ground for food instead of orbiting a dead cache.
    // A real cache (where it actually eats) is re-reinforced on the hit, so productive caches persist.
    // disconfirm_=0 → off (the exact prior food memory).
    if (hit_in_window_ && cur_node_ >= 0) {
        food_[cur_node_] += food_reward_; hit_in_window_ = false;
    } else if (disconfirm_ > 0.0f && cur_node_ >= 0) {
        auto fit = food_.find(cur_node_);
        auto hit3 = hab_.find(cur_node_);
        float hc = (hit3 != hab_.end()) ? hit3->second : 0.0f;
        if (fit != food_.end())
            fit->second *= (1.0f - std::clamp(disconfirm_ * desperation_ * hc, 0.0f, 1.0f));
    }
    for (auto& [n, f] : food_) f *= food_decay_;

    run_value_iteration();

    // ---- CONFINEMENT STEER (2026-07-02, operator option 2): route toward FRESH ground when stuck ----
    // Late in a run the value field is dense — food/TLE/γ-propagation leave an uphill neighbour at
    // EVERY node, so route_exists never clears and the planner ORBITS a handful of HABITUATED nodes
    // (the observed lock), BORED (habituation saturated) + HUNGRY, until it starves. Rather than a
    // blunt wander (which abandons routing → misses real food routes), STEER the route itself: subtract
    // a habituation penalty from each neighbour's value while the bug is CONFINED, so it keeps ROUTING
    // (graph-native — scales to complex mazes) but climbs OUT of the exhausted basin toward the
    // least-re-tread neighbour → the boundary → fresh ground.
    //   escape_bias = escape_gain · desperation · stale, where stale = clamp(ticks-since-the-map-grew ·
    //   hab_decay) rises ONLY when the bug re-treads known ground (≈0 while it forages/discovers new
    //   nodes → escape_bias≈0 → the steered value IS the raw value, so routing is IDENTICAL to before
    //   and genuine hungry routes to real food are never disrupted — no foraging regression by
    //   construction). steered(n) = value[n] − escape_bias·hab[n]. Interior of a uniformly-habituated
    //   basin: the penalty cancels (all hab≈1) so it still climbs the value gradient; at the BOUNDARY a
    //   fresh neighbour's small penalty wins → the bug routes out. Self-limiting: fresh ground grows the
    //   map → stale resets → escape_bias→0 → normal routing resumes. escape_gain=0 → off.
    float stale = (escape_gain_ > 0.0f) ? std::clamp(float(stale_ticks_) * hab_decay_, 0.0f, 1.0f) : 0.0f;
    float escape_bias = escape_gain_ * desperation_ * stale;
    steer_bias_ = escape_bias;   // telemetry: how hard the confinement steer is pulling toward fresh ground
    auto steered = [&](int n) -> float {
        auto vit = value_.find(n); float v = (vit != value_.end()) ? vit->second : 0.0f;
        if (escape_bias <= 0.0f) return v;                      // not confined → raw value (unchanged)
        auto hit = hab_.find(n);  float h = (hit != hab_.end()) ? hit->second : 0.0f;
        return v - escape_bias * h;                             // confined → prefer LESS-habituated (fresher) neighbours
    };

    // ---- policy: SUB-GOAL COMMITMENT. Hold the chosen next hop until the bug reaches it (or it
    // stops being a valid uphill neighbour). Re-picking the argmax neighbour every tick makes the
    // bug dither between similar-value neighbours → the bearing never settles → turn-in-place with
    // no forward progress. Commit to one hop, face it, drive there, then choose again. (Uses the
    // STEERED value so the commitment/route logic all agree under the confinement steer.)
    bool keep_goal = (committed_next_ >= 0) && (cur_node_ >= 0) && (committed_next_ != cur_node_);
    if (keep_goal) {
        auto it = edges_.find(cur_node_);
        keep_goal = (it != edges_.end()) && (it->second.count(committed_next_) > 0) &&
                    (steered(committed_next_) > steered(cur_node_) + 1e-4f);   // still a valid uphill neighbour
    }
    if (keep_goal) {
        next_node_ = committed_next_;
    } else {
        next_node_ = -1;
        float best_v = -1e30f;                                  // steered value may be negative under the bias
        if (cur_node_ >= 0) {
            auto it = edges_.find(cur_node_);
            if (it != edges_.end())
                for (auto const& [m, e] : it->second)
                    if (steered(m) > best_v) { best_v = steered(m); next_node_ = m; }
        }
        committed_next_ = next_node_;   // commit to the freshly chosen sub-goal
    }
    // THRESHOLD-FREE plan↔forage (2026-06-30, defensibility): the planner ROUTES whenever there is
    // a strictly-UPHILL (steered) neighbour to move to — a pure COMPARISON, no magnitude constant:
    // plan when somewhere is strictly better than staying; explore when the field is flat (empty/all-
    // habituated map → nothing to climb) or at a local peak. The +1e-4 is float-equality tolerance.
    // PATROL: always route to a reachable adjacent node (there is a least-recently-visited neighbour
    // essentially always, since the current node's own hab is high) → the bug keeps FLOWING through
    // known ground and NEVER drops into the run-tumble wander (that is play's job) or dithers on a
    // stuck target. Food-memory planner: route only when strictly uphill (else it forages/wanders).
    bool route_exists = patrol_mode_
        ? (next_node_ >= 0 && cur_node_ >= 0)
        : (next_node_ >= 0 && cur_node_ >= 0 && steered(next_node_) > steered(cur_node_) + 1e-4f);

    // ---- EXPLORE → EXPLOIT on the UNIFIED value field. The value sweep carries hunger-gated food +
    // epistemic TLE − habituation, so the SAME gradient handles exploit (route to remembered food)
    // and explore (route to the least-recent / highest-TLE frontier). The planner ROUTES whenever an
    // uphill gradient exists; the run-and-tumble EXPLORE branch is the BOOTSTRAP for a flat map. No
    // scent dependence → a competent standalone loop the arbiter can pick.
    float max_food = 0.0f;
    for (auto const& [n, f] : food_) max_food = std::max(max_food, f);
    bool food_known = max_food > 0.0f;   // a remembered food cache exists (gates the plan_value level)
    planning_ = route_exists;

    // ---- ROUTE-EXECUTION STALL (never dither on an unexecutable hop) ----
    // Count ticks on the current committed hop; a node transition RESETS it and trains the timeout scale
    // (EMA of successful hop durations). If a hop takes more than stall_factor × the bug's own normal
    // hop time, the route is BLOCKED (pointing through a wall / unreachable) → cede below (plan_value=0)
    // so klino/play move the bug; the resulting transition clears the stall and routing resumes.
    route_ceded_ = false;
    if (stall_factor_ > 0.0f) {
        if (node_transitioned) {
            if (route_stall_ > 0)
                hop_ema_ = (hop_ema_ <= 0.0f) ? float(route_stall_)
                                              : hop_ema_ + 0.1f * (float(route_stall_) - hop_ema_);
            route_stall_ = 0;
        } else if (planning_ && next_node_ >= 0 && next_node_ != cur_node_) {
            ++route_stall_;
        }
        route_ceded_ = (hop_ema_ > 0.5f) && (float(route_stall_) > stall_factor_ * hop_ema_);
    }

    // ---- §2.3 MODEL PRECISION: sharpness of the food belief ∈[0,1] ----
    // Treat the food_ map as a distribution over the caches it believes in; precision =
    // 1 − H(p)/log N (belief-entropy normalised). One known cache ⇒ p is a point mass ⇒ precision 1
    // (the planner is certain where food is). Food spread over many equal caches ⇒ high entropy ⇒
    // low precision. Empty map ⇒ fall back to the next-hop VALUE margin (how decisively the best
    // neighbour beats the runner-up = how sharply the value field points). This is what §2.3 means
    // by making precision a CONTROLLED variable: near a known source the belief is sharp, and the
    // arbiter can compare it against klino's SENSORY precision (cap) rather than a static 1/(tle+ε).
    {
        float sumf = 0.0f; int nf = 0;
        for (auto const& [n, f] : food_) if (f > 1e-4f) { sumf += f; ++nf; }
        if (nf >= 2 && sumf > 1e-6f) {
            float H = 0.0f;
            for (auto const& [n, f] : food_) if (f > 1e-4f) { float p = f / sumf; H -= p * std::log(p); }
            last_plan_precision_ = std::clamp(1.0f - H / std::log(float(nf)), 0.0f, 1.0f);
        } else if (nf == 1) {
            last_plan_precision_ = 1.0f;                       // a single known cache = maximally sharp belief
        } else {
            float best = -1e30f, second = -1e30f;              // empty map → next-hop value margin
            if (cur_node_ >= 0) {
                auto it = edges_.find(cur_node_);
                if (it != edges_.end())
                    for (auto const& [m, e] : it->second) {
                        float v = value(m);
                        if (v > best) { second = best; best = v; }
                        else if (v > second) second = v;
                    }
            }
            last_plan_precision_ = (best > -1e29f && second > -1e29f)
                                   ? std::clamp(best - second, 0.0f, 1.0f) : 0.0f;
        }
    }

    // ---- §2.2 EPISTEMIC term: the frontier novelty the planner routes toward ∈[0,1] ----
    // The per-node TLE (place-EPM world-model surprise) at the node the planner is heading to —
    // "how much map-uncertainty this hop would resolve" — normalised by its own slow running peak
    // (§6, derived not hand-set). ~0 while EXPLOITING a committed food route (the planner is not
    // gathering map-evidence then); high while routing up a pure-TLE gradient or wandering the
    // frontier. The arbiter weights it by (1−hunger) → curiosity rises as the bug sates.
    {
        float novelty;
        if (patrol_fallback_) {
            // PATROL as an EPISTEMIC term: the frontier COVERAGE NEED (1−hab[next]) — reduce state-
            // uncertainty about the least-recently-visited known ground. The arbiter weights this by
            // (1−hunger) = energy surplus, so the planner PATROLS known ground when FULL (alongside play
            // growing the map) and yields to klino's foraging when hungry — the clean pragmatic/epistemic
            // split (a hunger-gated pragmatic patrol crowds klino → starve). Coverage is robust where the
            // place-EPM TLE is too sparse to drive exploration.
            auto hit = hab_.find(next_node_);
            float hnext = (next_node_ >= 0 && hit != hab_.end()) ? hit->second : 0.0f;
            novelty = (next_node_ >= 0) ? std::clamp(1.0f - hnext, 0.0f, 1.0f) : 0.0f;
        } else {
            float frontier_tle = (next_node_ >= 0 && node_tle_.count(next_node_))
                                 ? node_tle_.at(next_node_) : local_tle;
            tle_peak_ = std::max(frontier_tle, tle_peak_ * (1.0f - tle_peak_decay_));
            novelty = std::clamp(frontier_tle / (tle_peak_ + 1e-6f), 0.0f, 1.0f);
        }
        last_plan_novelty_ = (food_known && route_exists) ? 0.0f : novelty;   // 0 while exploiting food
    }

    if (std::getenv("OGMA_PLANNER_DEBUG") && planning_ && (tick_id % 300 == 0)) {
        float vmax = 0.0f, fmax = 0.0f; int fnode = -1, nfood = 0, nout = 0;
        for (auto const& [n, v] : value_) vmax = std::max(vmax, v);
        for (auto const& [n, f] : food_) { if (f > fmax) { fmax = f; fnode = n; } if (f > 1e-3f) ++nfood; }
        auto it = edges_.find(cur_node_); if (it != edges_.end()) nout = int(it->second.size());
        std::fprintf(stderr, "PLDBG t=%llu cur=%d V[cur]=%.4f next=%d V[next]=%.4f nout=%d Vmax=%.4f "
                     "food_nodes=%d Fmax=%.4f fnode=%d V[fnode]=%.4f\n",
                     (unsigned long long)tick_id, cur_node_, value_[cur_node_], next_node_,
                     next_node_ >= 0 ? value_[next_node_] : -1.0f, nout, vmax, nfood, fmax,
                     fnode, fnode >= 0 ? value_[fnode] : -1.0f);
    }

    // VISION final-approach (highest priority): when the learned visual bearing sees food
    // in line of sight, home straight to it — this is the fine "last meter" the coarse map
    // route can't do and short-range scent can't reach.
    float vis_x = 0.0f, vis_y = 0.0f, vis_mag = 0.0f;
    if (!vision_topic_.empty()) {
        if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(vision_topic_))) {
            if (pt->values.size() > 0) vis_x = float(pt->values[0]);
            if (pt->values.size() > 1) vis_y = float(pt->values[1]);
            vis_mag = std::sqrt(vis_x * vis_x + vis_y * vis_y);
        }
    }

    float fx, fy, prox = 0.0f;
    wandering_ = false;
    homing_vision_ = false;
    if (vis_mag > vision_floor_) {
        homing_vision_ = true; planning_ = false; explore_active_ = false;
        fx = vis_x; fy = vis_y;
    } else if (planning_) {
        explore_active_ = false;                                      // EXPLOIT: route to remembered food
        float target = geo_bearing(cur_node_, next_node_);            // GEOMETRIC place→place bearing (path-integration)
        if (std::isnan(target)) target = edges_[cur_node_][next_node_].heading();  // fallback before centroids accumulate
        float delta  = wrap_pi(target - cur_heading_);   // how much to turn toward the next hop
        // TURN-COMMIT: a behind target (|delta|→π) dithers the HeadingController's steer
        // (sin(±π)=0 → no turn direction). Latch a direction and cap under π so the bug
        // commits to turning around instead of oscillating in place.
        constexpr float kPiF = 3.14159265f;
        if (!turning_ && std::fabs(delta) > 1.5708f)        { turning_ = true; turn_dir_ = (delta >= 0.0f) ? 1.0f : -1.0f; }
        else if (turning_ && std::fabs(delta) < 0.6283f)    { turning_ = false; }   // exit at ~36°
        if (turning_) delta = turn_dir_ * std::min(std::fabs(delta), 0.92f * kPiF);
        fx = std::sin(delta);   // egocentric: +right (matches the scent compass convention)
        fy = std::cos(delta);   // +forward
    } else {
        // EXPLORE: run-and-tumble climbing the LOCAL place-novelty (TLE). Run in a committed
        // heading; when a run cycle ends without novelty rising (entered already-learned ground),
        // tumble to a new heading. Forward run = motion (no crawl); novelty-climb = directed at the
        // unmodelled frontier. Homeokinetic exploration, no scent — the planner's own bootstrap so
        // it maps the maze without depending on klino. (RunTumbleNav mechanism on the map's TLE.)
        wandering_ = true; prox = 0.0f;
        if (!explore_active_) {                      // (re)entering explore → start a fresh forward run
            explore_active_ = true; explore_dir_ = cur_heading_;
            explore_run_ticks_ = 0; explore_tle_start_ = local_tle;
        }
        if (++explore_run_ticks_ >= explore_cycle_) {
            if ((local_tle - explore_tle_start_) <= 0.0f) {     // novelty didn't rise → tumble to new ground
                std::uniform_real_distribution<float> u(-explore_tumble_range_, explore_tumble_range_);
                explore_dir_ = wrap_pi(cur_heading_ + u(explore_rng_));
            }
            explore_tle_start_ = local_tle; explore_run_ticks_ = 0;
        }
        float ed = wrap_pi(explore_dir_ - cur_heading_);
        fx = std::sin(ed); fy = std::cos(ed);        // egocentric bearing toward the committed run heading
    }
    out_fx_ = fx; out_fy_ = fy;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("place_planner") : id_;
    out->sensor      = "nav_bearing";
    out->values.resize(3);
    out->values[0] = fx; out->values[1] = fy; out->values[2] = prox;
    bus_->publish(output_topic_, out);

    // ---- L2 EFE arbiter input: the FOOD-ROUTE value, published as a SUSTAINED LEVEL.
    // pv = value(next_node) ONLY when there is a committed route to REMEMBERED FOOD
    // (food_known && route_exists); 0 while merely EXPLORING (no food route).
    // Rationale (the false-interruption fix): the arbiter normalises this level by its own
    // slow peak (NOT a z-score), so a sustained high pv reads ~1 the whole time the planner
    // routes to food and never decays/goes negative as the running mean catches up. A blind
    // klino (z≈0) can no longer "overtake" a steady route; only a real scent z-spike does.
    // 0 while exploring means klino is free to forage when there is no food route to hold.
    if (!plan_value_topic_.empty()) {
        // PATROL: the arbiter "reach" is the frontier COVERAGE NEED (1−hab[next]) ∈[0,1] — high when
        // there is under-covered known ground to sweep, ~0 when everything is freshly covered. Hunger-
        // gated in the arbiter → the planner patrols when hungry (revisit known places where relocated
        // food may be), yields to klino when it smells food (cap→1 > coverage need) and to play when
        // full. Food-memory planner: value(next) while committed to a REMEMBERED FOOD route, else 0.
        float pv;
        if (patrol_mode_) {
            auto hit = hab_.find(next_node_);
            float hnext = (next_node_ >= 0 && hit != hab_.end()) ? hit->second : 0.0f;
            pv = (next_node_ >= 0) ? std::clamp(1.0f - hnext, 0.0f, 1.0f) : 0.0f;
        } else {
            // Food route stays PRAGMATIC (hunger-gated) — sustained while routing to REMEMBERED FOOD, else 0.
            // The patrol (coverage) is EPISTEMIC and rides the plan_novelty channel instead (see below), so
            // it does NOT compete with klino's hungry foraging here (a pragmatic patrol crowds klino → starve).
            pv = (food_known && route_exists) ? value(next_node_) : 0.0f;
        }
        if (route_ceded_) pv = 0.0f;   // route is blocked (dither guard) → cede so klino/play move the bug off the stuck spot
        last_plan_value_ = pv;
        auto vout = std::make_shared<ProprioToken>();
        vout->tick_id     = tick_id;
        vout->producer_id = id_.empty() ? std::string("place_planner") : id_;
        vout->sensor      = "plan_value";
        vout->values.resize(1);
        vout->values[0]   = pv;
        bus_->publish(plan_value_topic_, vout);
    }
    // §2.3 model precision + §2.2 epistemic frontier novelty → the explicit-EFE arbiter.
    auto pub_scalar = [&](std::string const& topic, char const* sensor, float v){
        if (topic.empty()) return;
        auto o = std::make_shared<ProprioToken>();
        o->tick_id     = tick_id;
        o->producer_id = id_.empty() ? std::string("place_planner") : id_;
        o->sensor      = sensor;
        o->values.resize(1);
        o->values[0]   = v;
        bus_->publish(topic, o);
    };
    pub_scalar(plan_precision_topic_, "plan_precision", last_plan_precision_);
    pub_scalar(plan_novelty_topic_,   "plan_novelty",   last_plan_novelty_);
}

float PlaceGraphPlanner::food_value(int n) const {
    auto it = food_.find(n); return it == food_.end() ? 0.0f : it->second;
}
float PlaceGraphPlanner::value(int n) const {
    auto it = value_.find(n); return it == value_.end() ? 0.0f : it->second;
}
float PlaceGraphPlanner::edge_heading(int from, int to) const {
    auto fit = edges_.find(from);
    if (fit == edges_.end()) return std::numeric_limits<float>::quiet_NaN();
    auto tit = fit->second.find(to);
    if (tit == fit->second.end()) return std::numeric_limits<float>::quiet_NaN();
    return tit->second.heading();
}
int PlaceGraphPlanner::edge_count(int from, int to) const {
    auto fit = edges_.find(from);
    if (fit == edges_.end()) return 0;
    auto tit = fit->second.find(to);
    return tit == fit->second.end() ? 0 : tit->second.count;
}
float PlaceGraphPlanner::hab_cur() const {
    if (cur_node_ < 0) return 0.0f;
    auto it = hab_.find(cur_node_);
    return it == hab_.end() ? 0.0f : it->second;
}
float PlaceGraphPlanner::max_hab() const {
    float m = 0.0f;
    for (auto const& [n, h] : hab_) m = std::max(m, h);
    return m;
}

nlohmann::json PlaceGraphPlanner::diag_snapshot() const {
    // Node-parallel arrays + edge list → the v4_inspector place-graph widget.
    std::vector<int>   nodes;   std::vector<double> food, val;
    nlohmann::json node_pos = nlohmann::json::array();   // path-integration centroids (true geometry)
    for (auto const& [n, v] : value_) {
        nodes.push_back(n); val.push_back(v);
        auto fit = food_.find(n); food.push_back(fit == food_.end() ? 0.0 : fit->second);
        auto pit = place_pos_.find(n);
        if (pit != place_pos_.end() && pit->second.n > 0)
            node_pos.push_back({pit->second.sx / pit->second.n, pit->second.sy / pit->second.n});
        else
            node_pos.push_back({0.0, 0.0});
    }
    nlohmann::json edges = nlohmann::json::array();
    for (auto const& [from, tos] : edges_)
        for (auto const& [to, e] : tos)
            edges.push_back({from, to, double(e.heading()), e.count});
    return nlohmann::json{
        {"cur_node", cur_node_},
        {"next_node", next_node_},
        {"planning", planning_},
        {"wandering", wandering_},
        {"patrol_mode", patrol_mode_},
        {"patrol_fallback", patrol_fallback_},
        {"route_ceded", route_ceded_},   // ceding a blocked route this tick (dither guard)
        {"route_stall", route_stall_},   // ticks on the current hop without a transition
        {"homing_vision", homing_vision_},
        {"cur_heading", cur_heading_},
        {"n_nodes", int(value_.size())},
        {"nodes", nodes},
        {"node_pos", node_pos},
        {"food", food},
        {"value", val},
        {"edges", edges},
        {"fx", out_fx_}, {"fy", out_fy_},
        {"hunger", hunger_},
        // value-race transparency (mirrors the EFE arbiter): V at the current node and the chosen
        // next hop, the map's best value, and the FOOD-ROUTE value this planner publishes to the L2
        // arbiter (value(next_hop) while routing to remembered food, else 0 — its raw_planner term).
        {"v_cur",  (cur_node_  >= 0 && value_.count(cur_node_))  ? value_.at(cur_node_)  : 0.0f},
        {"v_next", (next_node_ >= 0 && value_.count(next_node_)) ? value_.at(next_node_) : 0.0f},
        {"v_max",  value_.empty() ? 0.0f
                     : std::max_element(value_.begin(), value_.end(),
                         [](auto const& a, auto const& b){ return a.second < b.second; })->second},
        {"plan_value", last_plan_value_},
        // §2.3 model precision (belief sharpness) + §2.2 frontier novelty → the explicit-EFE arbiter
        {"plan_precision", last_plan_precision_},
        {"plan_novelty",   last_plan_novelty_},
        // HK habituation telemetry ("recent = boring")
        {"hab_cur", hab_cur()},
        {"n_nodes_hab", int(hab_.size())},
        {"max_hab", max_hab()},
        // desperation-driven exploration telemetry (the "hungrier → widen the search" drive)
        {"desperation", desperation_}, // = hunger → accelerates disconfirmation (let go of unproductive caches, search out)
        {"steer_bias", steer_bias_},     // confinement-steer strength (escape_gain·desperation·stale)
        {"escaping", steer_bias_ > 0.01f}, // the steer is meaningfully pulling the route toward fresh ground
    };
}

}  // namespace ogma
