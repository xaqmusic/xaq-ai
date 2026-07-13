#include "ogma/modules/PlayLoop.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }

// Circular blend of two headings: weight (1−w) on a, w on c (w∈[0,1]). w=0 returns a EXACTLY (Δ=0).
inline float circ_blend(float a, float c, float w) {
    float s = (1.0f - w) * std::sin(a) + w * std::sin(c);
    float k = (1.0f - w) * std::cos(a) + w * std::cos(c);
    if (s == 0.0f && k == 0.0f) return a;   // antipodal at equal weight → degenerate; keep a
    return std::atan2(s, k);
}

template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("PlayLoop: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("PlayLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PlayLoop: param '" + k + "' must be a string");
}
}  // namespace

float PlayLoop::Edge::heading() const {
    if (count == 0) return 0.0f;
    return std::atan2(sum_sin, sum_cos);   // circular mean of unit vectors
}

std::string_view PlayLoop::type_name() const { return "PlayLoop"; }

std::vector<TopicSpec> PlayLoop::input_topics() const {
    return {
        TopicSpec{place_topic_,   std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{eat_topic_,     std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> PlayLoop::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
    if (!play_value_topic_.empty())
        v.push_back(TopicSpec{play_value_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema PlayLoop::params_schema() const {
    return {
        {"place_topic",   ParamMutability::ConstructionOnly, "Current place (RealityToken.winner_id) — the SHARED place-map both play and planner read.", ParamValue{std::string("reality.cognitive.place")}},
        {"heading_topic", ParamMutability::ConstructionOnly, "Absolute heading (ProprioToken[0]).", ParamValue{std::string("reality.proprio.heading")}},
        {"vel_topic",     ParamMutability::ConstructionOnly, "Forward speed (ProprioToken) → path-integration odometry.", ParamValue{std::string("reality.proprio.vel_ego")}},
        {"eat_topic",     ParamMutability::ConstructionOnly, "GROUND-TRUTH eat event (EnvEvent, e.g. events.eat) → eat-credit success telemetry (did exploration lead to a real eat).", ParamValue{std::string("events.eat")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "Chosen play bearing → HeadingController (play channel).", ParamValue{std::string("percept.play_bearing")}},
        {"play_value_topic", ParamMutability::ConstructionOnly, "Publish the FRONTIER NOVELTY ∈[0,1] (the epistemic value) as a ProprioToken scalar → the L2 EFE arbiter (weighted by energy surplus). NOT zeroed while a route exists (play has no food route). Empty = no publish (default-off).", ParamValue{std::string("")}},
        {"gamma",         ParamMutability::HotMutable, "Novelty value-iteration discount.", ParamValue{0.85}},
        {"vi_sweeps",     ParamMutability::HotMutable, "Value-iteration sweeps per tick.", ParamValue{int64_t{8}}},
        {"tle_ema_alpha", ParamMutability::HotMutable, "EMA rate for the per-node TLE novelty signal.", ParamValue{0.1}},
        {"tle_peak_decay", ParamMutability::HotMutable, "Slow decay of the node-TLE running peak (the play_value normaliser, §6 — derived from the signal, not hand-set).", ParamValue{0.0005}},
        {"hab_rise",      ParamMutability::HotMutable, "HK habituation rise rate while dwelling (recent = boring; fast). 0 disables habituation.", ParamValue{0.1}},
        {"hab_decay",     ParamMutability::HotMutable, "HK habituation per-tick recovery (slow; ~1/hab_decay ticks). Sets how long play sweeps other ground before a region is interesting again.", ParamValue{0.002}},
        {"explore_cycle",        ParamMutability::HotMutable, "Run-and-tumble wander: run length (ticks) before a tumble decision.", ParamValue{int64_t{30}}},
        {"explore_tumble_range", ParamMutability::HotMutable, "Run-and-tumble wander: max reorient per tumble (rad).", ParamValue{1.5708}},
        {"wander_stall_ticks",   ParamMutability::HotMutable, "WANDER-BEYOND fix: force the run-tumble WANDER (override the climb) when the map has not grown for this many ticks — the bug has mapped this region, so push PAST the frontier into unmapped ground instead of climbing freshly-baked nodes forever. A new node resets the counter → back to climb. 0 = off (climb-only). ~explore_cycle is the natural scale.", ParamValue{int64_t{0}}},
        {"frontier_bias",        ParamMutability::HotMutable, "FRONTIER-DIRECTED WANDER: bias the run-tumble wander toward UNEXPLORED ground (away from the habituation-weighted centroid of visited places) instead of a memoryless random walk — the maze-discovery fix. Effective pull = frontier_bias·max_hab (magnitude DERIVED from the explored-core confidence, no-tuning §6). 0 = off (diffusive run-and-tumble, prior behaviour, Δ=0); 1 = full outward (beat 0.5/0 monotonically, A/B lbend).", ParamValue{0.0}},
        {"explore_seed",         ParamMutability::ConstructionOnly, "Run-and-tumble wander RNG seed.", ParamValue{int64_t{11}}},
        {"pi_cell_size",  ParamMutability::HotMutable, "Path-integration place-code: >0 = place node IS the odometry grid cell (metres); 0 = use place_topic (panorama).", ParamValue{0.0}},
        {"eat_credit_alpha", ParamMutability::HotMutable, "EMA rate for the eat-credit success signal (telemetry).", ParamValue{0.01}},
    };
}

ParamMap PlayLoop::current_params() const {
    ParamMap m;
    m["place_topic"]   = ParamValue{place_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["vel_topic"]     = ParamValue{vel_topic_};
    m["eat_topic"]     = ParamValue{eat_topic_};
    m["output_topic"]  = ParamValue{output_topic_};
    m["play_value_topic"] = ParamValue{play_value_topic_};
    m["gamma"]         = ParamValue{double(gamma_)};
    m["vi_sweeps"]     = ParamValue{int64_t(vi_sweeps_)};
    m["tle_ema_alpha"] = ParamValue{double(tle_ema_alpha_)};
    m["tle_peak_decay"] = ParamValue{double(tle_peak_decay_)};
    m["hab_rise"]      = ParamValue{double(hab_rise_)};
    m["hab_decay"]     = ParamValue{double(hab_decay_)};
    m["explore_cycle"]        = ParamValue{int64_t(explore_cycle_)};
    m["explore_tumble_range"] = ParamValue{double(explore_tumble_range_)};
    m["wander_stall_ticks"]   = ParamValue{int64_t(wander_stall_ticks_)};
    m["frontier_bias"]        = ParamValue{double(frontier_bias_)};
    m["explore_seed"]         = ParamValue{int64_t(explore_seed_)};
    m["pi_cell_size"]  = ParamValue{double(pi_cell_size_)};
    m["eat_credit_alpha"] = ParamValue{double(eat_credit_alpha_)};
    return m;
}

void PlayLoop::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "gamma")         gamma_         = float(get_double(value, k));
    else if (k == "vi_sweeps")     vi_sweeps_     = int(get_int(value, k));
    else if (k == "tle_ema_alpha") tle_ema_alpha_ = float(get_double(value, k));
    else if (k == "tle_peak_decay") tle_peak_decay_ = float(get_double(value, k));
    else if (k == "hab_rise")      hab_rise_      = float(get_double(value, k));
    else if (k == "hab_decay")     hab_decay_     = float(get_double(value, k));
    else if (k == "explore_cycle")        explore_cycle_        = int(get_int(value, k));
    else if (k == "explore_tumble_range") explore_tumble_range_ = float(get_double(value, k));
    else if (k == "wander_stall_ticks")   wander_stall_ticks_   = int(get_int(value, k));
    else if (k == "frontier_bias") frontier_bias_ = float(get_double(value, k));
    else if (k == "pi_cell_size")  pi_cell_size_  = float(get_double(value, k));
    else if (k == "eat_credit_alpha") eat_credit_alpha_ = float(get_double(value, k));
}

void PlayLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "place_topic",   [&](auto const& v){ place_topic_   = get_string(v,"place_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "eat_topic",     [&](auto const& v){ eat_topic_     = get_string(v,"eat_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "play_value_topic", [&](auto const& v){ play_value_topic_ = get_string(v,"play_value_topic"); });
    apply_param(params, "gamma",         [&](auto const& v){ gamma_         = float(get_double(v,"gamma")); });
    apply_param(params, "vi_sweeps",     [&](auto const& v){ vi_sweeps_     = int(get_int(v,"vi_sweeps")); });
    apply_param(params, "tle_ema_alpha", [&](auto const& v){ tle_ema_alpha_ = float(get_double(v,"tle_ema_alpha")); });
    apply_param(params, "tle_peak_decay", [&](auto const& v){ tle_peak_decay_ = float(get_double(v,"tle_peak_decay")); });
    apply_param(params, "hab_rise",      [&](auto const& v){ hab_rise_      = float(get_double(v,"hab_rise")); });
    apply_param(params, "hab_decay",     [&](auto const& v){ hab_decay_     = float(get_double(v,"hab_decay")); });
    apply_param(params, "explore_cycle",        [&](auto const& v){ explore_cycle_        = int(get_int(v,"explore_cycle")); });
    apply_param(params, "explore_tumble_range", [&](auto const& v){ explore_tumble_range_ = float(get_double(v,"explore_tumble_range")); });
    apply_param(params, "wander_stall_ticks",   [&](auto const& v){ wander_stall_ticks_   = int(get_int(v,"wander_stall_ticks")); });
    apply_param(params, "frontier_bias",        [&](auto const& v){ frontier_bias_        = float(get_double(v,"frontier_bias")); });
    apply_param(params, "explore_seed",         [&](auto const& v){ explore_seed_         = uint64_t(get_int(v,"explore_seed")); });
    explore_rng_.seed(explore_seed_);
    apply_param(params, "pi_cell_size",  [&](auto const& v){ pi_cell_size_  = float(get_double(v,"pi_cell_size")); });
    apply_param(params, "eat_credit_alpha", [&](auto const& v){ eat_credit_alpha_ = float(get_double(v,"eat_credit_alpha")); });

    if (!eat_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(eat_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_eat(p); }));
}

void PlayLoop::handle_eat(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (std::dynamic_pointer_cast<const EnvEvent>(payload)) eat_in_window_ = true;
}

void PlayLoop::run_value_iteration() {
    // Gauss-Seidel sweeps on NOVELTY (not food): V[n] = novelty[n]·(1−hab[n]) + γ·max_m V[n→m].
    // Habituation suppresses the LOCAL novelty term only (recent = boring) so propagation routes the
    // gradient toward the least-recent / highest-TLE frontier and play SWEEPS instead of orbiting.
    for (int s = 0; s < vi_sweeps_; ++s) {
        for (auto& [n, vn] : value_) {
            float best = 0.0f;
            auto it = edges_.find(n);
            if (it != edges_.end())
                for (auto const& [m, e] : it->second) best = std::max(best, value_[m]);
            auto tit = node_tle_.find(n);
            float nov = (tit != node_tle_.end()) ? tit->second : 0.0f;
            auto hit = hab_.find(n);
            float h = (hit != hab_.end()) ? hit->second : 0.0f;
            vn = nov * (1.0f - h) + gamma_ * best;
        }
    }
}

float PlayLoop::geo_bearing(int from, int to) const {
    auto fit = place_pos_.find(from), tit = place_pos_.find(to);
    if (fit == place_pos_.end() || tit == place_pos_.end() || fit->second.n == 0 || tit->second.n == 0)
        return std::numeric_limits<float>::quiet_NaN();
    double fx = fit->second.sx / fit->second.n, fy = fit->second.sy / fit->second.n;
    double tx = tit->second.sx / tit->second.n, ty = tit->second.sy / tit->second.n;
    double dx = tx - fx, dy = ty - fy;
    if (dx == 0.0 && dy == 0.0) return std::numeric_limits<float>::quiet_NaN();
    return float(std::atan2(-dx, -dy));  // forward dir(h) = (-sin h, -cos h)
}

void PlayLoop::tick(uint64_t tick_id) {
    // ---- pull inputs by value (robust to gate + DAG order) ----
    int new_node = cur_node_;
    float place_tle = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(place_topic_))) {
        new_node = pt->winner_id;
        place_tle = pt->tle;   // world-model surprise at the current place → novelty
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

    // ---- map building: register the node + learn the edge-heading on a transition ----
    bool grew = false;   // did NEW ground bake this tick? (resets the stall-wander clock)
    if (new_node >= 0) {
        auto [vit, vins] = value_.emplace(new_node, 0.0f);
        if (vins) grew = true;
        if (cur_node_ >= 0 && new_node != cur_node_) {
            Edge& e = edges_[cur_node_][new_node];
            e.sum_sin += std::sin(cur_heading_);
            e.sum_cos += std::cos(cur_heading_);
            e.count   += 1;
        }
        cur_node_ = new_node;
    }
    stale_explore_ = grew ? 0 : (stale_explore_ + 1);   // ticks since the map last grew
    if (cur_node_ >= 0) {
        PlacePos& pp = place_pos_[cur_node_];
        pp.sx += odo_x_; pp.sy += odo_y_; pp.n += 1;
    }
    // NOVELTY: EMA the place-EPM's TLE onto the current node (dynamic; falls as the EPM learns).
    if (cur_node_ >= 0) {
        auto [it, ins] = node_tle_.try_emplace(cur_node_, place_tle);
        if (!ins) it->second += tle_ema_alpha_ * (place_tle - it->second);
    }
    float local_tle = (cur_node_ >= 0 && node_tle_.count(cur_node_)) ? node_tle_.at(cur_node_) : 0.0f;

    // HK HABITUATION ("recent = boring"): decay all (slow), rise current (fast).
    for (auto& [n, hh] : hab_) hh *= (1.0f - hab_decay_);
    if (cur_node_ >= 0) { float& hc = hab_[cur_node_]; hc += hab_rise_ * (1.0f - hc); }

    // EAT-CREDIT (success telemetry): EMA of "a real eat happened this tick".
    float ate = eat_in_window_ ? 1.0f : 0.0f; eat_in_window_ = false;
    eat_credit_ += eat_credit_alpha_ * (ate - eat_credit_);

    run_value_iteration();

    // ---- policy: SUB-GOAL COMMITMENT on the NOVELTY value field (climb to the frontier) ----
    bool keep_goal = (committed_next_ >= 0) && (cur_node_ >= 0) && (committed_next_ != cur_node_);
    if (keep_goal) {
        auto it = edges_.find(cur_node_);
        keep_goal = (it != edges_.end()) && (it->second.count(committed_next_) > 0) &&
                    (value(committed_next_) > value(cur_node_) + 1e-4f);
    }
    if (keep_goal) {
        next_node_ = committed_next_;
    } else {
        next_node_ = -1;
        float best_v = -1e30f;
        if (cur_node_ >= 0) {
            auto it = edges_.find(cur_node_);
            if (it != edges_.end())
                for (auto const& [m, e] : it->second)
                    if (value(m) > best_v) { best_v = value(m); next_node_ = m; }
        }
        committed_next_ = next_node_;
    }
    // THRESHOLD-FREE climb↔wander: CLIMB whenever a strictly-more-novel neighbour exists; WANDER
    // (run-and-tumble beyond the frontier) when the local field is flat (at a novelty peak / edge of
    // the mapped graph). The +1e-4 is float-equality tolerance, not a knob.
    bool route_exists = (next_node_ >= 0) && (cur_node_ >= 0)
                        && (value(next_node_) > value(cur_node_) + 1e-4f);
    // WANDER-BEYOND: if the map has not grown for wander_stall_ticks, the bug has mapped this region
    // (climbing freshly-baked nodes is treadmilling) → FORCE the run-tumble wander to push PAST the
    // frontier into unmapped ground. A new node (grew → stale_explore_=0) drops it back to climb.
    forced_wander_ = (wander_stall_ticks_ > 0) && (stale_explore_ >= wander_stall_ticks_);
    climbing_ = route_exists && !forced_wander_;

    // ---- §2.2 epistemic value → the L2 arbiter: the frontier VALUE ∈[0,1] ----
    // V_play propagates downstream novelty (γ·max), so V_play(next_node) is HIGH when a novel frontier
    // is reachable (even through a low-novelty intermediate hop) and → 0 when the whole map is explored
    // (novelty has decayed everywhere). Normalised by its own slow running peak (§6, derived not hand-
    // set). NOT zeroed while climbing — play has no food route to exploit, so its value is always its
    // epistemic (map-growth) potential. (This mirrors the planner publishing value(next_node), the
    // propagated food value, rather than the raw food at the next hop.)
    {
        // ABSOLUTE exploration potential ∈[0,1] — NOT self-normalised. The old form divided the frontier
        // value by its own slow-decaying running peak (val_peak_), so play_value COLLAPSED to ~0 the moment
        // the frontier node was reached and flickered thereafter (an early TLE spike then suppressed it for
        // ~1400 ticks) → play only won in brief bursts. Two honest, absolute referents instead:
        //  • CLIMB (a more-novel neighbour exists): the propagated frontier novelty relative to the scale of
        //    FRESHLY-discovered ground (novel_ref_ = EMA of the node-TLE at map growth — "what new ground is
        //    worth," self-calibrated from the EPM's own surprise, §6, not a hand-set constant).
        //  • WANDER / at a leaf edge (no mapped novel neighbour → about to push into UNMAPPED ground): the
        //    local UN-habituation (1−hab_cur). Fresh ground (hab→0, e.g. at spawn or on entering new
        //    territory) is maximally worth growing into; well-trodden ground (hab→1) yields to klino/planner.
        // This keeps play's epistemic value HIGH while it is actually expanding the map and lets it fall only
        // when the reachable ground is mapped — the energy-surplus gate in the arbiter (1−hunger) still makes
        // play defer to foraging as the bug empties (§2.1: curiosity is instrumental).
        if (grew) novel_ref_ += 0.1f * (std::max(local_tle, novel_ref_) - novel_ref_);   // rise to the fresh-place scale
        novel_ref_ *= (1.0f - tle_peak_decay_);                                           // slow fade if no new ground bakes (world mapped)
        val_peak_ = std::max(novel_ref_, val_peak_ * (1.0f - tle_peak_decay_));           // (telemetry: value_peak)
        float ref = std::max(novel_ref_, 1e-3f);
        float climb_v  = (next_node_ >= 0) ? std::clamp(value(next_node_) / ref, 0.0f, 1.0f) : 0.0f;
        float wander_v = std::clamp(1.0f - hab_cur(), 0.0f, 1.0f);
        last_play_value_ = std::max(climb_v, wander_v);
    }

    // ---- bearing: CLIMB the novelty gradient, else WANDER beyond the frontier ----
    float fx, fy;
    wandering_ = false;
    if (climbing_) {
        explore_active_ = false;                                     // EXPLORE-CLIMB: route to the frontier node
        float target = geo_bearing(cur_node_, next_node_);
        if (std::isnan(target)) target = edges_[cur_node_][next_node_].heading();
        float delta  = wrap_pi(target - cur_heading_);
        constexpr float kPiF = 3.14159265f;
        if (!turning_ && std::fabs(delta) > 1.5708f)     { turning_ = true; turn_dir_ = (delta >= 0.0f) ? 1.0f : -1.0f; }
        else if (turning_ && std::fabs(delta) < 0.6283f) { turning_ = false; }
        if (turning_) delta = turn_dir_ * std::min(std::fabs(delta), 0.92f * kPiF);
        fx = std::sin(delta); fy = std::cos(delta);
    } else {
        // WANDER: run-and-tumble climbing the LOCAL novelty. Run a committed heading; when a run cycle
        // ends without novelty rising (entered already-learned ground), tumble to new ground. Forward
        // run = motion (no crawl); this is the branch that LEAVES the mapped graph into unmapped ground.
        wandering_ = true;
        // FRONTIER-DIRECTED base heading: normally the current heading (memoryless run-and-tumble). When
        // frontier_bias>0, blend in the OUTWARD bearing — from the habituation-weighted centroid of
        // visited places toward here — so the wander pushes toward unexplored ground instead of diffusing.
        // The centroid is in odometry space; it and odo_[xy]_ share the SAME drift, so the outward vector
        // is drift-robust (common-mode cancels). Magnitude derived (eff = frontier_bias·max_hab): no pull
        // on a fresh map, full pull once a core is well-known (§6/no-tuning, Playful-Machine self-avoidance).
        have_frontier_ = false;
        float base_dir = cur_heading_;
        if (frontier_bias_ > 0.0f) {
            double cx = 0.0, cy = 0.0, wsum = 0.0;
            for (auto const& [n, h] : hab_) {
                auto pit = place_pos_.find(n);
                if (pit == place_pos_.end() || pit->second.n == 0) continue;
                double px = pit->second.sx / pit->second.n, py = pit->second.sy / pit->second.n;
                cx += double(h) * px; cy += double(h) * py; wsum += double(h);
            }
            if (wsum > 1e-6) {
                cx /= wsum; cy /= wsum;
                double dx = odo_x_ - cx, dy = odo_y_ - cy;   // outward vector: visited-centroid → here
                if (dx * dx + dy * dy > 1e-6) {
                    frontier_bearing_ = float(std::atan2(-dx, -dy));   // heading whose forward dir = (dx,dy)
                    // DERIVED magnitude (no-tuning §6): the outward pull = frontier_bias · max_hab — a
                    // confidence ramp that RISES with the explored core (no pull on a fresh map — wsum-guarded
                    // — full pull once any ground is well-known). frontier_bias is the 0..1 enable/ceiling; 1.0
                    // (full outward) BEAT 0.5 and 0 monotonically on discovery (A/B lbend) — steer as hard as
                    // possible toward the frontier, no interior optimum to tune. A per-place hab_cur gate was
                    // tried and LOST (it relaxed the pull on fresh ground → the bug curled back; first-far
                    // 3468 vs 1188): the SUSTAINED always-on drive is what carries the bug across a maze.
                    float eff = std::clamp(frontier_bias_ * max_hab(), 0.0f, 1.0f);
                    base_dir = circ_blend(cur_heading_, frontier_bearing_, eff);
                    have_frontier_ = true;
                }
            }
        }
        if (!explore_active_) {
            explore_active_ = true; explore_dir_ = base_dir;
            explore_run_ticks_ = 0; explore_tle_start_ = local_tle;
        }
        if (++explore_run_ticks_ >= explore_cycle_) {
            if ((local_tle - explore_tle_start_) <= 0.0f) {
                std::uniform_real_distribution<float> u(-explore_tumble_range_, explore_tumble_range_);
                explore_dir_ = wrap_pi(base_dir + u(explore_rng_));
            }
            explore_tle_start_ = local_tle; explore_run_ticks_ = 0;
        }
        float ed = wrap_pi(explore_dir_ - cur_heading_);
        fx = std::sin(ed); fy = std::cos(ed);
    }
    out_fx_ = fx; out_fy_ = fy;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("play_loop") : id_;
    out->sensor      = "play_bearing";
    out->values.resize(3);
    out->values[0] = fx; out->values[1] = fy; out->values[2] = 0.0f;
    bus_->publish(output_topic_, out);

    if (!play_value_topic_.empty()) {
        auto vout = std::make_shared<ProprioToken>();
        vout->tick_id     = tick_id;
        vout->producer_id = id_.empty() ? std::string("play_loop") : id_;
        vout->sensor      = "play_value";
        vout->values.resize(1);
        vout->values[0]   = last_play_value_;
        bus_->publish(play_value_topic_, vout);
    }
}

float PlayLoop::value(int n) const {
    auto it = value_.find(n); return it == value_.end() ? 0.0f : it->second;
}
float PlayLoop::novelty(int n) const {
    auto it = node_tle_.find(n); return it == node_tle_.end() ? 0.0f : it->second;
}
float PlayLoop::novelty_cur() const {
    if (cur_node_ < 0) return 0.0f;
    auto it = node_tle_.find(cur_node_);
    return it == node_tle_.end() ? 0.0f : it->second;
}
float PlayLoop::hab_cur() const {
    if (cur_node_ < 0) return 0.0f;
    auto it = hab_.find(cur_node_);
    return it == hab_.end() ? 0.0f : it->second;
}
float PlayLoop::max_hab() const {
    float m = 0.0f;
    for (auto const& [n, h] : hab_) m = std::max(m, h);
    return m;
}

nlohmann::json PlayLoop::diag_snapshot() const {
    std::vector<int> nodes; std::vector<double> nov, val;
    nlohmann::json node_pos = nlohmann::json::array();
    for (auto const& [n, v] : value_) {
        nodes.push_back(n); val.push_back(v);
        auto tit = node_tle_.find(n); nov.push_back(tit == node_tle_.end() ? 0.0 : tit->second);
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
        {"climbing", climbing_},
        {"wandering", wandering_},
        {"have_frontier", have_frontier_},        // frontier bearing defined + biasing the wander this tick
        {"frontier_bearing", frontier_bearing_},  // outward heading (away from the visited centroid)
        {"forced_wander", forced_wander_},   // stall-wander overriding the climb (pushing beyond the frontier)
        {"stale_explore", stale_explore_},    // ticks since the map last grew
        {"cur_heading", cur_heading_},
        {"n_nodes", int(value_.size())},
        {"nodes", nodes},
        {"node_pos", node_pos},
        {"novelty", nov},
        {"value", val},
        {"edges", edges},
        {"fx", out_fx_}, {"fy", out_fy_},
        {"play_value", last_play_value_},   // frontier VALUE ∈[0,1] → arbiter (propagated novelty, epistemic value)
        {"novelty_cur", novelty_cur()},
        {"value_peak", val_peak_},
        {"hab_cur", hab_cur()},
        {"n_nodes_hab", int(hab_.size())},
        {"max_hab", max_hab()},
        {"eat_credit", eat_credit_},        // EMA of "exploration led to a real eat"
    };
}

}  // namespace ogma
