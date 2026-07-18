#include "ogma/modules/PlaceNav.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }
template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("PlaceNav: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("PlaceNav: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PlaceNav: param '" + k + "' must be a string");
}
PlaceNav::Ablation parse_ablation(std::string const& s) {
    if (s.empty() || s == "none")   return PlaceNav::Ablation::None;
    if (s == "no_forget")           return PlaceNav::Ablation::NoForget;
    if (s == "no_reach_cost")       return PlaceNav::Ablation::NoReachCost;
    if (s == "shuffle_edges")       return PlaceNav::Ablation::ShuffleEdges;
    throw std::invalid_argument("PlaceNav: unknown ablation '" + s + "'");
}
const char* ablation_name(PlaceNav::Ablation a) {
    switch (a) {
        case PlaceNav::Ablation::None:         return "none";
        case PlaceNav::Ablation::NoForget:     return "no_forget";
        case PlaceNav::Ablation::NoReachCost:  return "no_reach_cost";
        case PlaceNav::Ablation::ShuffleEdges: return "shuffle_edges";
    }
    return "none";
}
constexpr float kTagFloor = 0.05f;   // a tag below this is "gone" (not a food route)
}  // namespace

float PlaceNav::Edge::heading() const {
    if (count == 0) return std::numeric_limits<float>::quiet_NaN();
    return std::atan2(sum_sin, sum_cos);
}

std::string_view PlaceNav::type_name() const { return "PlaceNav"; }

std::vector<TopicSpec> PlaceNav::input_topics() const {
    return {
        TopicSpec{place_topic_,   std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{hunger_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{eat_topic_,     std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> PlaceNav::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
    if (!plan_value_topic_.empty())   v.push_back(TopicSpec{plan_value_topic_,   std::type_index(typeid(ProprioToken))});
    if (!plan_novelty_topic_.empty()) v.push_back(TopicSpec{plan_novelty_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema PlaceNav::params_schema() const {
    return {
        {"place_topic",   ParamMutability::ConstructionOnly, "Current place (RealityToken.winner_id + tle).", ParamValue{std::string("reality.cognitive.place")}},
        {"heading_topic", ParamMutability::ConstructionOnly, "Absolute heading (ProprioToken[0]).", ParamValue{std::string("reality.proprio.heading")}},
        {"hunger_topic",  ParamMutability::ConstructionOnly, "Hunger (1-energy), weights the food term in V.", ParamValue{std::string("reality.proprio.hunger")}},
        {"eat_topic",     ParamMutability::ConstructionOnly, "GROUND-TRUTH eat -> SET the current place's food tag.", ParamValue{std::string("events.eat")}},
        {"vel_topic",     ParamMutability::ConstructionOnly, "Egomotion vel [right,fwd] -> path-integration odometry.", ParamValue{std::string("reality.proprio.vel_ego")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "Egocentric bearing [fx,fy,prox] -> HeadingController.", ParamValue{std::string("percept.nav_bearing")}},
        {"plan_value_topic",   ParamMutability::ConstructionOnly, "HONEST reach-to-region value -> EFEArbiter: value(next) ONLY for a FRESH tag + an executable (non-stalled) route, else 0. Empty = no publish.", ParamValue{std::string("")}},
        {"plan_novelty_topic", ParamMutability::ConstructionOnly, "Coverage need (1-hab[next]) when there is NO food route -> EFEArbiter g_epist_planner. Empty = no publish.", ParamValue{std::string("")}},
        {"gamma",         ParamMutability::HotMutable, "Value-iteration discount.", ParamValue{0.85}},
        {"vi_sweeps",     ParamMutability::HotMutable, "Value-iteration sweeps per tick.", ParamValue{int64_t{8}}},
        {"food_decay",    ParamMutability::HotMutable, "Slow passive per-tick fade of a food tag (loose remembrance).", ParamValue{0.999}},
        {"arrival_forget", ParamMutability::HotMutable, "R1 HONEST DISCONFIRM: multiply a tag by this when the bug ARRIVES at the tagged place and no eat fires within arrival_window (or leaves without eating) -> the food is gone. Collapses stale caches (§8 downvote-on-own-consequence).", ParamValue{0.1}},
        {"arrival_window", ParamMutability::HotMutable, "Ticks on a tagged node to wait for an eat before collapsing the tag.", ParamValue{int64_t{30}}},
        {"block_gain",    ParamMutability::HotMutable, "R2 reachability: block_cost added to a committed hop that STALLS (points through a wall).", ParamValue{0.5}},
        {"block_decay",   ParamMutability::HotMutable, "Per-tick decay of a hop's block_cost (a dropped wall re-opens the edge).", ParamValue{0.001}},
        {"stall_factor",  ParamMutability::HotMutable, "A committed hop is BLOCKED after this x (EMA hop duration) with no transition -> cost the edge + cede.", ParamValue{4.0}},
        {"hab_rise",      ParamMutability::HotMutable, "Habituation rise rate while dwelling (coverage / anti-oscillation).", ParamValue{0.1}},
        {"hab_decay",     ParamMutability::HotMutable, "Per-tick habituation recovery (slow).", ParamValue{0.002}},
        {"coverage_gain", ParamMutability::HotMutable, "Per-node 'want to visit' drive when there is NO food route (routes to least-recent; -> plan_novelty).", ParamValue{0.3}},
        {"pi_cell_size",  ParamMutability::HotMutable, ">0: node IS the odometry grid cell (fresh, geometric); 0 = use place_topic winner_id.", ParamValue{0.0}},
        {"explore_cycle", ParamMutability::HotMutable, "Explore run length (ticks) before a tumble decision.", ParamValue{int64_t{30}}},
        {"explore_tumble_range", ParamMutability::HotMutable, "Max reorient per explore tumble (rad).", ParamValue{1.5708}},
        {"ablation",      ParamMutability::HotMutable, "Validation control. none=full navigator; no_forget=keep stale tags (the R1 baseline, should REGRESS on relocation); no_reach_cost=ignore wall stalls (R2 baseline); shuffle_edges=randomise edge headings (breaks routing, must regress).", ParamValue{std::string("none")}},
        {"master_seed",   ParamMutability::ConstructionOnly, "RNG seed (explore tumbles).", ParamValue{int64_t{11}}},
    };
}

ParamMap PlaceNav::current_params() const {
    ParamMap m;
    m["place_topic"] = ParamValue{place_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["hunger_topic"] = ParamValue{hunger_topic_};
    m["eat_topic"] = ParamValue{eat_topic_};
    m["vel_topic"] = ParamValue{vel_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["plan_value_topic"] = ParamValue{plan_value_topic_};
    m["plan_novelty_topic"] = ParamValue{plan_novelty_topic_};
    m["gamma"] = ParamValue{double(gamma_)};
    m["vi_sweeps"] = ParamValue{int64_t(vi_sweeps_)};
    m["food_decay"] = ParamValue{double(food_decay_)};
    m["arrival_forget"] = ParamValue{double(arrival_forget_)};
    m["arrival_window"] = ParamValue{int64_t(arrival_window_)};
    m["block_gain"] = ParamValue{double(block_gain_)};
    m["block_decay"] = ParamValue{double(block_decay_)};
    m["stall_factor"] = ParamValue{double(stall_factor_)};
    m["hab_rise"] = ParamValue{double(hab_rise_)};
    m["hab_decay"] = ParamValue{double(hab_decay_)};
    m["coverage_gain"] = ParamValue{double(coverage_gain_)};
    m["pi_cell_size"] = ParamValue{double(pi_cell_size_)};
    m["explore_cycle"] = ParamValue{int64_t(explore_cycle_)};
    m["explore_tumble_range"] = ParamValue{double(explore_tumble_range_)};
    m["ablation"] = ParamValue{std::string(ablation_name(ablation_))};
    m["master_seed"] = ParamValue{int64_t(master_seed_)};
    return m;
}

void PlaceNav::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "gamma")          gamma_          = float(get_double(value, k));
    else if (k == "vi_sweeps")      vi_sweeps_      = std::max(1, int(get_int(value, k)));
    else if (k == "food_decay")     food_decay_     = float(get_double(value, k));
    else if (k == "arrival_forget") arrival_forget_ = float(get_double(value, k));
    else if (k == "arrival_window") arrival_window_ = std::max(1, int(get_int(value, k)));
    else if (k == "block_gain")     block_gain_     = float(get_double(value, k));
    else if (k == "block_decay")    block_decay_    = float(get_double(value, k));
    else if (k == "stall_factor")   stall_factor_   = float(get_double(value, k));
    else if (k == "hab_rise")       hab_rise_       = float(get_double(value, k));
    else if (k == "hab_decay")      hab_decay_      = float(get_double(value, k));
    else if (k == "coverage_gain")  coverage_gain_  = float(get_double(value, k));
    else if (k == "pi_cell_size")   pi_cell_size_   = float(get_double(value, k));
    else if (k == "explore_cycle")  explore_cycle_  = std::max(1, int(get_int(value, k)));
    else if (k == "explore_tumble_range") explore_tumble_range_ = float(get_double(value, k));
    else if (k == "ablation")       ablation_       = parse_ablation(get_string(value, k));
    else throw std::invalid_argument("PlaceNav: param '" + k + "' is construction-only / unknown");
}

void PlaceNav::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "place_topic",   [&](auto const& v){ place_topic_   = get_string(v,"place_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "hunger_topic",  [&](auto const& v){ hunger_topic_  = get_string(v,"hunger_topic"); });
    apply_param(params, "eat_topic",     [&](auto const& v){ eat_topic_     = get_string(v,"eat_topic"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "plan_value_topic",   [&](auto const& v){ plan_value_topic_   = get_string(v,"plan_value_topic"); });
    apply_param(params, "plan_novelty_topic", [&](auto const& v){ plan_novelty_topic_ = get_string(v,"plan_novelty_topic"); });
    apply_param(params, "gamma",         [&](auto const& v){ gamma_         = float(get_double(v,"gamma")); });
    apply_param(params, "vi_sweeps",     [&](auto const& v){ vi_sweeps_     = std::max(1, int(get_int(v,"vi_sweeps"))); });
    apply_param(params, "food_decay",    [&](auto const& v){ food_decay_    = float(get_double(v,"food_decay")); });
    apply_param(params, "arrival_forget",[&](auto const& v){ arrival_forget_= float(get_double(v,"arrival_forget")); });
    apply_param(params, "arrival_window",[&](auto const& v){ arrival_window_= std::max(1, int(get_int(v,"arrival_window"))); });
    apply_param(params, "block_gain",    [&](auto const& v){ block_gain_    = float(get_double(v,"block_gain")); });
    apply_param(params, "block_decay",   [&](auto const& v){ block_decay_   = float(get_double(v,"block_decay")); });
    apply_param(params, "stall_factor",  [&](auto const& v){ stall_factor_  = float(get_double(v,"stall_factor")); });
    apply_param(params, "hab_rise",      [&](auto const& v){ hab_rise_      = float(get_double(v,"hab_rise")); });
    apply_param(params, "hab_decay",     [&](auto const& v){ hab_decay_     = float(get_double(v,"hab_decay")); });
    apply_param(params, "coverage_gain", [&](auto const& v){ coverage_gain_ = float(get_double(v,"coverage_gain")); });
    apply_param(params, "pi_cell_size",  [&](auto const& v){ pi_cell_size_  = float(get_double(v,"pi_cell_size")); });
    apply_param(params, "explore_cycle", [&](auto const& v){ explore_cycle_ = std::max(1, int(get_int(v,"explore_cycle"))); });
    apply_param(params, "explore_tumble_range", [&](auto const& v){ explore_tumble_range_ = float(get_double(v,"explore_tumble_range")); });
    apply_param(params, "ablation",      [&](auto const& v){ ablation_      = parse_ablation(get_string(v,"ablation")); });
    apply_param(params, "master_seed",   [&](auto const& v){ master_seed_   = uint64_t(get_int(v,"master_seed")); });

    explore_rng_.seed(master_seed_);
    // The ground-truth eat is delivered via a Direct subscription (the bus dispatches every publish to
    // all Direct subscribers). We do NOT gate the handler on input_allowed: the eat is a ground-truth
    // env signal, always from the host, and gating it serves no routing purpose.
    if (!eat_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(eat_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){
                if (std::dynamic_pointer_cast<const EnvEvent>(p)) { eat_in_window_ = true; ++eats_received_; }
            }));
}

float PlaceNav::geo_bearing(int from, int to) const {
    auto af = place_pos_.find(from), at = place_pos_.find(to);
    if (af == place_pos_.end() || at == place_pos_.end() || af->second.n == 0 || at->second.n == 0)
        return std::numeric_limits<float>::quiet_NaN();
    double fx = af->second.sx / af->second.n, fy = af->second.sy / af->second.n;
    double tx = at->second.sx / at->second.n, ty = at->second.sy / at->second.n;
    double dx = tx - fx, dy = ty - fy;
    if (dx == 0.0 && dy == 0.0) return std::numeric_limits<float>::quiet_NaN();
    return std::atan2(float(dx), float(dy));   // absolute bearing (same [sin,cos] convention as edge headings)
}

void PlaceNav::run_value_iteration() {
    // V[n] = local[n]*(1-hab[n]) + gamma*max_m (V[m] - block[n][m]).
    // local = hunger*food_tag (exploit) + coverage_gain*(1-hab) when there is no food route (patrol).
    bool any_food = food_tag_count_ > 0;
    for (int s = 0; s < vi_sweeps_; ++s) {
        for (auto& [n, v] : value_) {
            float hab = hab_.count(n) ? hab_.at(n) : 0.0f;
            float tag = food_tag_.count(n) ? food_tag_.at(n) : 0.0f;
            float local = hunger_ * tag;
            if (!any_food) local += coverage_gain_;   // no food known -> cover least-recent ground
            float best = 0.0f;
            auto eit = edges_.find(n);
            if (eit != edges_.end())
                for (auto& [m, e] : eit->second)
                    if (value_.count(m)) best = std::max(best, value_.at(m) - e.block);
            v = local * (1.0f - hab) + gamma_ * best;
        }
    }
}

void PlaceNav::tick(uint64_t tick_id) {
    // ---- pull inputs by value ----
    int new_node = cur_node_;
    float place_tle = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(place_topic_))) {
        new_node = pt->winner_id;
        place_tle = pt->tle;
    }
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(heading_topic_)))
        if (pt->values.size() > 0) cur_heading_ = float(pt->values[0]);
    float vlat = 0.0f, vfwd = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(vel_topic_))) {
        if (pt->values.size() > 0) vlat = float(pt->values[0]);
        if (pt->values.size() > 1) vfwd = float(pt->values[1]);
    }
    odo_x_ += -double(vfwd) * std::sin(cur_heading_) + double(vlat) * std::cos(cur_heading_);
    odo_y_ += -double(vfwd) * std::cos(cur_heading_) - double(vlat) * std::sin(cur_heading_);
    if (pi_cell_size_ > 0.0f) {
        int gx = int(std::floor(odo_x_ / double(pi_cell_size_))) + 512;
        int gy = int(std::floor(odo_y_ / double(pi_cell_size_))) + 512;
        new_node = gx * 1024 + gy;
    }
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(hunger_topic_)))
        if (pt->values.size() > 0) hunger_ = float(pt->values[0]);

    // ---- map building: register node + learn edge heading on a transition ----
    bool node_transitioned = false;
    if (new_node >= 0) {
        value_.emplace(new_node, 0.0f);
        food_tag_.emplace(new_node, 0.0f);
        if (cur_node_ >= 0 && new_node != cur_node_) {
            node_transitioned = true;
            Edge& e = edges_[cur_node_][new_node];
            e.sum_sin += std::sin(cur_heading_);
            e.sum_cos += std::cos(cur_heading_);
            e.count   += 1;
        }
        prev_node_ = cur_node_;
        cur_node_  = new_node;
    }
    if (cur_node_ >= 0) {
        PlacePos& pp = place_pos_[cur_node_];
        pp.sx += odo_x_; pp.sy += odo_y_; pp.n += 1;
        auto [it, ins] = node_tle_.try_emplace(cur_node_, place_tle);
        if (!ins) it->second += 0.1f * (place_tle - it->second);
    }
    float local_tle = (cur_node_ >= 0 && node_tle_.count(cur_node_)) ? node_tle_.at(cur_node_) : 0.0f;

    // ---- LOOSE HONEST FOOD TAG ----
    // eat -> SET (bounded) the current place's tag; a productive cache re-arms on every eat.
    bool ate_this_tick = false;
    if (eat_in_window_ && cur_node_ >= 0) {
        food_tag_[cur_node_] = 1.0f;
        ate_since_arrival_ = true;
        arrival_node_ = -1;               // productive -> not a stale arrival
        ate_this_tick = true;
    }
    eat_in_window_ = false;
    // open a forage window on ARRIVING at a tagged region we did NOT just eat at (a return visit to
    // check remembered food). Eating on arrival keeps the belief; only a foodless arrival is on trial.
    if (!ate_this_tick && node_transitioned && cur_node_ >= 0 &&
        (food_tag_.count(cur_node_) && food_tag_.at(cur_node_) > kTagFloor)) {
        arrival_node_ = cur_node_; arrival_ticks_ = 0; ate_since_arrival_ = false;
    }
    // R1 honest disconfirm (§8): the belief "food is in this region" collapses only when a HUNGRY
    // forage of the region FAILS -- the bug arrived at the tagged place, ate nothing, and has now
    // COMMITTED to leaving (gone a sustained arrival_window spell). Camping a productive region
    // (periodic eats re-arm) or leaving while FULL never disconfirms it, so a live food-region stays
    // remembered across the whole forage cycle (routable back to when hunger returns) instead of dying
    // between a slow closer's sparse eats. This is the scent-blind agent's only honest negative signal:
    // "I remembered food here, came back hungry, tried, and it's gone."
    if (ablation_ != Ablation::NoForget && arrival_node_ >= 0) {
        if (ate_since_arrival_) {
            arrival_node_ = -1; arrival_ticks_ = 0;        // productive visit -> re-armed
        } else if (cur_node_ == arrival_node_) {
            arrival_ticks_ = 0;                            // still foraging the region -> hold the belief
        } else if (++arrival_ticks_ >= arrival_window_) {  // committed departure without an eat
            if (hunger_ > 0.3f && food_tag_.count(arrival_node_))
                food_tag_[arrival_node_] *= arrival_forget_;   // a HUNGRY forage of the region failed -> gone
            arrival_node_ = -1; arrival_ticks_ = 0;
        }
    }
    // slow passive fade + recount live tags.
    food_tag_count_ = 0;
    for (auto& [n, f] : food_tag_) { f *= food_decay_; if (f > kTagFloor) ++food_tag_count_; }

    // ---- habituation ----
    for (auto& [n, h] : hab_) h *= (1.0f - hab_decay_);
    if (cur_node_ >= 0) { float& hc = hab_[cur_node_]; hc += hab_rise_ * (1.0f - hc); }

    // ---- reachability: block_cost on a stalled committed hop ----
    for (auto& [f, tos] : edges_) for (auto& [t, e] : tos) e.block *= (1.0f - block_decay_);
    route_ceded_ = false;
    if (node_transitioned) {
        hop_ema_ += 0.1f * (float(route_stall_) - hop_ema_);   // learn the normal hop duration
        route_stall_ = 0;
        committed_next_ = -1;                                  // reached a node -> re-plan
    } else if (committed_next_ >= 0) {
        ++route_stall_;
        float cede_at = stall_factor_ * std::max(hop_ema_, 8.0f);
        if (float(route_stall_) > cede_at) {                   // the committed hop is BLOCKED (wall)
            if (ablation_ != Ablation::NoReachCost && cur_node_ >= 0)
                edges_[cur_node_][committed_next_].block += block_gain_;
            route_ceded_ = true;
            committed_next_ = -1;
            route_stall_ = 0;
        }
    }

    // ---- value iteration + policy ----
    run_value_iteration();
    bool any_food = food_tag_count_ > 0;
    next_node_ = -1;
    float best_score = -std::numeric_limits<float>::infinity();
    if (cur_node_ >= 0 && edges_.count(cur_node_)) {
        // hold the committed sub-goal while it is still a strictly-uphill reachable neighbour.
        if (committed_next_ >= 0) {
            auto& row = edges_.at(cur_node_);
            auto ceit = row.find(committed_next_);
            if (ceit != row.end() && value_.count(committed_next_)) {
                float cs = value_.at(committed_next_) - ceit->second.block;
                float here = value_.count(cur_node_) ? value_.at(cur_node_) : 0.0f;
                if (cs > here + 1e-4f) { next_node_ = committed_next_; best_score = cs; }
            }
        }
        if (next_node_ < 0)
            for (auto& [m, e] : edges_.at(cur_node_)) {
                if (!value_.count(m)) continue;
                float sc = value_.at(m) - e.block;
                if (sc > best_score) { best_score = sc; next_node_ = m; }
            }
    }
    float here_v = (cur_node_ >= 0 && value_.count(cur_node_)) ? value_.at(cur_node_) : 0.0f;
    bool route_exists = (next_node_ >= 0) && (best_score > here_v + 1e-4f);
    planning_ = any_food && route_exists && !route_ceded_;
    wandering_ = !planning_;
    if (planning_) committed_next_ = next_node_;

    // ---- bearing ----
    float delta;
    if (planning_ && next_node_ >= 0) {
        // steer on the heading that ACTUALLY worked (edge), fall back to the geometric bearing.
        float tgt = edge_heading(cur_node_, next_node_);
        if (std::isnan(tgt)) tgt = geo_bearing(cur_node_, next_node_);
        if (std::isnan(tgt)) tgt = cur_heading_;
        delta = wrap_pi(tgt - cur_heading_);
        // turn-commit latch (avoid antipode dither)
        if (!turning_ && std::fabs(delta) > 1.5708f)     { turning_ = true; turn_dir_ = (delta >= 0.0f) ? 1.0f : -1.0f; }
        else if (turning_ && std::fabs(delta) < 0.6283f) { turning_ = false; }
        if (turning_) delta = turn_dir_ * std::min(std::fabs(delta), 0.92f * kPi);
    } else {
        // EXPLORE: run-and-tumble climbing local place-TLE (hand-off; play/klino also drive here).
        if (explore_run_ticks_ == 0) { explore_dir_ = cur_heading_; explore_tle_start_ = local_tle; }
        ++explore_run_ticks_;
        if (explore_run_ticks_ >= explore_cycle_) {
            explore_run_ticks_ = 0;
            if (local_tle <= explore_tle_start_ + 1e-4f) {   // novelty didn't rise -> tumble to new ground
                std::uniform_real_distribution<float> ut(-explore_tumble_range_, explore_tumble_range_);
                explore_dir_ = wrap_pi(cur_heading_ + ut(explore_rng_));
            }
        }
        delta = wrap_pi(explore_dir_ - cur_heading_);
        turning_ = false;
    }
    out_fx_ = std::sin(delta);
    out_fy_ = std::cos(delta);
    float prox = std::clamp(here_v, 0.0f, 1.0f);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("placenav") : id_;
    out->sensor      = "nav_bearing";
    out->values.resize(3);
    out->values[0] = out_fx_; out->values[1] = out_fy_; out->values[2] = prox;
    bus_->publish(output_topic_, out);

    // ---- HONEST plan_value (reach-to-region) + plan_novelty (coverage need) ----
    last_plan_value_ = planning_ ? std::clamp(value_.count(next_node_) ? value_.at(next_node_) : 0.0f, 0.0f, 1.0f) : 0.0f;
    last_plan_novelty_ = (!planning_ && next_node_ >= 0)
        ? std::clamp(1.0f - (hab_.count(next_node_) ? hab_.at(next_node_) : 0.0f), 0.0f, 1.0f) : 0.0f;
    if (!plan_value_topic_.empty()) {
        auto v = std::make_shared<ProprioToken>();
        v->tick_id = tick_id; v->producer_id = id_.empty() ? std::string("placenav") : id_;
        v->sensor = "plan_value"; v->values.resize(1); v->values[0] = last_plan_value_;
        bus_->publish(plan_value_topic_, v);
    }
    if (!plan_novelty_topic_.empty()) {
        auto v = std::make_shared<ProprioToken>();
        v->tick_id = tick_id; v->producer_id = id_.empty() ? std::string("placenav") : id_;
        v->sensor = "plan_novelty"; v->values.resize(1); v->values[0] = last_plan_novelty_;
        bus_->publish(plan_novelty_topic_, v);
    }
}

float PlaceNav::food_tag(int n) const { auto it = food_tag_.find(n); return it == food_tag_.end() ? 0.0f : it->second; }
float PlaceNav::value(int n)    const { auto it = value_.find(n);    return it == value_.end()    ? 0.0f : it->second; }
float PlaceNav::edge_heading(int from, int to) const {
    auto f = edges_.find(from); if (f == edges_.end()) return std::numeric_limits<float>::quiet_NaN();
    auto t = f->second.find(to); if (t == f->second.end()) return std::numeric_limits<float>::quiet_NaN();
    return t->second.heading();
}
float PlaceNav::block_cost(int from, int to) const {
    auto f = edges_.find(from); if (f == edges_.end()) return 0.0f;
    auto t = f->second.find(to); if (t == f->second.end()) return 0.0f;
    return t->second.block;
}
float PlaceNav::hab_cur() const { return (cur_node_ >= 0 && hab_.count(cur_node_)) ? hab_.at(cur_node_) : 0.0f; }

nlohmann::json PlaceNav::diag_snapshot() const {
    // Node-parallel arrays + edge list -> the xaq_inspector place-nav widget (mirrors the
    // PlaceGraphPlanner snapshot so the map/route render is reused; `food` = the LOOSE food tag).
    std::vector<int>    nodes;   std::vector<double> food, val, hab;
    nlohmann::json node_pos = nlohmann::json::array();   // path-integration centroids (geometry)
    for (auto const& [n, v] : value_) {
        nodes.push_back(n); val.push_back(v);
        auto fit = food_tag_.find(n); food.push_back(fit == food_tag_.end() ? 0.0 : fit->second);
        auto hit = hab_.find(n);      hab.push_back(hit == hab_.end() ? 0.0 : hit->second);
        auto pit = place_pos_.find(n);
        if (pit != place_pos_.end() && pit->second.n > 0)
            node_pos.push_back({pit->second.sx / pit->second.n, pit->second.sy / pit->second.n});
        else
            node_pos.push_back({0.0, 0.0});
    }
    nlohmann::json edges = nlohmann::json::array();
    for (auto const& [from, tos] : edges_)
        for (auto const& [to, e] : tos)
            edges.push_back({from, to, double(e.heading()), e.count, double(e.block)});  // [from,to,heading,count,block]
    float v_max = value_.empty() ? 0.0f
                    : std::max_element(value_.begin(), value_.end(),
                        [](auto const& a, auto const& b){ return a.second < b.second; })->second;
    float max_hab = hab_.empty() ? 0.0f
                    : std::max_element(hab_.begin(), hab_.end(),
                        [](auto const& a, auto const& b){ return a.second < b.second; })->second;
    return nlohmann::json{
        {"cur_node", cur_node_}, {"next_node", next_node_},
        {"planning", planning_}, {"wandering", wandering_},
        {"homing_vision", false},                    // PlaceNav is scent/vision-blind by design
        {"route_ceded", route_ceded_}, {"route_stall", route_stall_},
        {"cur_heading", cur_heading_},
        {"n_nodes", int(value_.size())},
        {"nodes", nodes}, {"node_pos", node_pos},
        {"food", food},                              // the loose bounded food TAG per node
        {"value", val}, {"hab", hab},
        {"edges", edges},
        {"fx", out_fx_}, {"fy", out_fy_},
        {"hunger", hunger_},
        {"v_cur",  (cur_node_  >= 0 && value_.count(cur_node_))  ? value_.at(cur_node_)  : 0.0f},
        {"v_next", (next_node_ >= 0 && value_.count(next_node_)) ? value_.at(next_node_) : 0.0f},
        {"v_max",  v_max},
        {"plan_value",   last_plan_value_},          // honest reach-to-region -> arbiter reach_planner
        {"plan_precision", 0.0f},                    // PlaceNav does not publish a precision term
        {"plan_novelty", last_plan_novelty_},        // coverage need when no food route
        {"hab_cur", hab_cur()}, {"max_hab", max_hab}, {"n_nodes_hab", int(hab_.size())},
        // --- PlaceNav reframe extras (the region-navigator signals) ---
        {"n_food_tags", food_tag_count_},            // # of nodes holding a live food tag (map-wide)
        {"ablation", ablation_name(ablation_)},
    };
}

}  // namespace ogma
