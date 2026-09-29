// =============================================================================
// BearingSeekLoop.cpp  --  see the header
// =============================================================================
#include "ogma/modules/BearingSeekLoop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    if (auto p = std::get_if<bool>(&v))    return *p ? 1.0 : 0.0;
    throw std::invalid_argument("BearingSeekLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BearingSeekLoop: param '" + k + "' must be a string");
}
} // namespace

BearingSeekLoop::BearingSeekLoop()  = default;
BearingSeekLoop::~BearingSeekLoop() = default;

std::string_view BearingSeekLoop::type_name() const { return "BearingSeekLoop"; }

std::vector<TopicSpec> BearingSeekLoop::input_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{bearing_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
                              TopicSpec{pose_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
    if (!renew_topic_.empty()) v.push_back(TopicSpec{renew_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!mover_topic_.empty()) v.push_back(TopicSpec{mover_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!yield_topic_.empty()) v.push_back(TopicSpec{yield_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> BearingSeekLoop::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))},
                              TopicSpec{value_topic_,  std::type_index(typeid(ProprioToken))} };
    if (!range_topic_.empty()) v.push_back(TopicSpec{range_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema BearingSeekLoop::params_schema() const {
    return {
        {"bearing_topic", ParamMutability::ConstructionOnly,
            "ProprioToken [vx=+right, vy=+forward, proximity] to the attended thing (CloudMap thing_bearing_topic); proximity 0 = nothing seen.",
            ParamValue{std::string("percept.thing_bearing")}},
        {"pose_topic", ParamMutability::ConstructionOnly,
            "ProprioToken [x, y, yaw]: the body's own dead-reckoned pose, the frame the remembered position lives in.",
            ParamValue{std::string("reality.proprio.odom")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Bearing to the thing (live, or remembered and re-aimed) [cx=+right, cy=+forward, 0]; zero when none.",
            ParamValue{std::string("percept.seek_bearing")}},
        {"value_topic", ParamMutability::ConstructionOnly,
            "The loop's need in [0,1]: confidence in the held target (1 seen, decaying remembered, 0 none) -- the arbiter's preference.",
            ParamValue{std::string("reality.cognitive.seek_value")}},
        {"range_topic", ParamMutability::ConstructionOnly,
            "The loop's honest signal for LoopCompetence (sign -1): the remaining range to the target, metres; the last value when none. Empty = not published.",
            ParamValue{std::string("reality.cognitive.seek_range")}},
        {"proximity_range", ParamMutability::HotMutable,
            "The range the bearing's proximity is scaled by: proximity = 1 - range / this (CloudMap's things_range, else its max_range).",
            ParamValue{2.5}, ParamValue{0.1}, ParamValue{10.0}},
        {"min_conf", ParamMutability::HotMutable,
            "Proximity above which the thing counts as in view this tick.", ParamValue{0.02}},
        {"arrive_m", ParamMutability::HotMutable,
            "Remaining range (metres, by dead reckoning) at which the target is reached and dropped.", ParamValue{0.25}},
        {"forget_ticks", ParamMutability::HotMutable,
            "While the thing is unseen the confidence decays by 1/this per tick (about this many ticks of memory).", ParamValue{3000.0}},
        {"floor", ParamMutability::HotMutable,
            "Confidence below which the remembered target is dropped.", ParamValue{0.05}},
        {"renew_topic", ParamMutability::ConstructionOnly,
            "ProprioToken [need, x, y] (SkillOutcomeLoop need_topic): what is still unknown about the last attended thing and where it is. "
            "After an arrival, a need above renew_min re-arms the target there with confidence = need (the linger). Empty = off.",
            ParamValue{std::string("")}},
        {"renew_min", ParamMutability::HotMutable,
            "The need above which a dropped target is renewed.", ParamValue{0.25}},
        {"renew_range", ParamMutability::HotMutable,
            "A renewal only within this range (metres) of the thing; never within 1.5 x arrive_m (that would be an arrival without a walk).",
            ParamValue{2.0}},
        {"walk_refix_m", ParamMutability::HotMutable,
            "A bearing flagged as seen from a WALKING cloud (the token's 4th value) refines a held target when its fix lies within this of it, "
            "and never sets a new one; 0 = walking bearings are ignored.  The approach is then by sight.",
            ParamValue{0.0}},
        {"walk_take_range", ParamMutability::HotMutable,
            "A bearing seen from a WALKING cloud may start a target when its fix is within this range (m), no target is held and no "
            "lost mover is in mind.  0 = never (R74's rule, made for a bearing that was wrong).", ParamValue{0.0}},
        {"mover_topic", ParamMutability::ConstructionOnly,
            "THE CHASE: ProprioToken [vx=+right, vy=+forward, proximity, age, oldest] (CloudMap mover_topic) -- a cluster whose voxels are "
            "young against the cloud's own.  Sightings that follow their own prediction become a chased target with a velocity; "
            "see the header.  Empty = off.",
            ParamValue{std::string("")}},
        {"chase_gate_m", ParamMutability::HotMutable,
            "A sighting confirms the candidate when its fix lies within this (metres) of the candidate's predicted position.", ParamValue{0.35}},
        {"chase_confirm", ParamMutability::HotMutable,
            "Sightings (including the first) before the candidate is chased.", ParamValue{int64_t{2}}},
        {"chase_confirm_ticks", ParamMutability::HotMutable,
            "...spread over at least this many ticks: a static thing newly in view is young for under a second and must age out first.",
            ParamValue{int64_t{25}}},
        {"chase_forget_ticks", ParamMutability::HotMutable,
            "No confirming sighting for this many ticks ends the chase; the last predicted position stays as a remembered target.",
            ParamValue{int64_t{50}}},
        {"chase_lead_s", ParamMutability::HotMutable,
            "The target is the candidate's predicted position this far ahead (seconds of its velocity).", ParamValue{0.3}},
        {"chase_v_max", ParamMutability::HotMutable,
            "A fix implying a faster mover than this (m/s) is another cluster, not a confirmation.", ParamValue{1.0}},
        {"chase_stop_v", ParamMutability::HotMutable,
            "A chase that ends with the thing still moving faster than this (m/s) is dropped, not remembered: the thing left the "
            "view.  Below it the thing stopped and its place is remembered.  0 = always remembered (the first form).", ParamValue{0.0}},
        {"chase_permanence_ticks", ParamMutability::HotMutable,
            "OBJECT PERMANENCE: when a chased thing's sightings stop, its predicted position keeps moving at the last velocity for up "
            "to this many ticks (the need falling 1 -> 0), a sighting near the prediction resumes the chase, and the loss is reported "
            "at the end.  0 = the loss at once.", ParamValue{int64_t{0}}},
        {"yield_topic", ParamMutability::ConstructionOnly,
            "THE YIELD: ProprioToken [tall_count, range] (CloudMap target_tall_topic) -- the tall voxels around this loop's held target.  Empty = off.",
            ParamValue{std::string("")}},
        {"chase_yield_tall", ParamMutability::HotMutable,
            "A chase or coast whose target has at least this many tall voxels within a body length yields (lost: the memory, the look).  0 = off.",
            ParamValue{int64_t{0}}},
        {"chase_memory_ticks", ParamMutability::HotMutable,
            "PERMANENCE IN RECOGNITION: a lost mover is kept in mind (position and velocity, extrapolated) for this many ticks without "
            "driving the walk; one mover sighting within chase_gate_m of where it should now be re-acquires the chase at once.  0 = off.",
            ParamValue{int64_t{0}}},
        {"chase_memory_holds", ParamMutability::HotMutable,
            "While the memory of a lost mover lives (chase_memory_ticks), no new static target is taken: the moving thing keeps its "
            "priority over the block beside it until it is forgotten.", ParamValue{false}},
        {"chase_pull_decay", ParamMutability::HotMutable,
            "Every loss multiplies the chase's pull (its need while chasing) by this; 1 = no decay.", ParamValue{1.0}},
        {"chase_pull_recover_ticks", ParamMutability::HotMutable,
            "The pull recovers toward 1 by 1/this per tick.", ParamValue{3000.0}},
        {"chase_min_v", ParamMutability::HotMutable,
            "A candidate is chased only if it has moved: its velocity and its displacement per second since the first sighting at "
            "least this (m/s).  A young cluster that stays put is a thing newly in view.  0 = not required.", ParamValue{0.0}},
    };
}

ParamMap BearingSeekLoop::current_params() const {
    ParamMap m;
    m["bearing_topic"] = ParamValue{bearing_topic_}; m["pose_topic"] = ParamValue{pose_topic_};
    m["output_topic"] = ParamValue{output_topic_}; m["value_topic"] = ParamValue{value_topic_};
    m["range_topic"] = ParamValue{range_topic_};
    m["proximity_range"] = ParamValue{proximity_range_}; m["min_conf"] = ParamValue{double(min_conf_)};
    m["arrive_m"] = ParamValue{arrive_m_}; m["forget_ticks"] = ParamValue{forget_ticks_}; m["floor"] = ParamValue{double(floor_)};
    m["renew_topic"] = ParamValue{renew_topic_}; m["renew_min"] = ParamValue{double(renew_min_)}; m["renew_range"] = ParamValue{renew_range_};
    m["walk_refix_m"] = ParamValue{walk_refix_m_}; m["walk_take_range"] = ParamValue{walk_take_range_};
    m["mover_topic"] = ParamValue{mover_topic_}; m["chase_gate_m"] = ParamValue{chase_gate_m_};
    m["chase_confirm"] = ParamValue{int64_t(chase_confirm_)}; m["chase_confirm_ticks"] = ParamValue{int64_t(chase_confirm_ticks_)};
    m["chase_forget_ticks"] = ParamValue{int64_t(chase_forget_ticks_)}; m["chase_lead_s"] = ParamValue{chase_lead_s_};
    m["chase_v_max"] = ParamValue{chase_v_max_}; m["chase_min_v"] = ParamValue{chase_min_v_}; m["chase_stop_v"] = ParamValue{chase_stop_v_};
    m["chase_permanence_ticks"] = ParamValue{int64_t(chase_permanence_ticks_)}; m["chase_memory_ticks"] = ParamValue{int64_t(chase_memory_ticks_)};
    m["chase_memory_holds"] = ParamValue{chase_memory_holds_};
    m["yield_topic"] = ParamValue{yield_topic_}; m["chase_yield_tall"] = ParamValue{int64_t(chase_yield_tall_)}; m["chase_pull_decay"] = ParamValue{chase_pull_decay_};
    m["chase_pull_recover_ticks"] = ParamValue{chase_pull_recover_ticks_};
    return m;
}

void BearingSeekLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BearingSeekLoop requires a non-null Bus");
    apply_param(params, "bearing_topic",   [&](auto const& v){ bearing_topic_ = get_string(v,"bearing_topic"); });
    apply_param(params, "pose_topic",      [&](auto const& v){ pose_topic_    = get_string(v,"pose_topic"); });
    apply_param(params, "output_topic",    [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "value_topic",     [&](auto const& v){ value_topic_   = get_string(v,"value_topic"); });
    apply_param(params, "range_topic",     [&](auto const& v){ range_topic_   = get_string(v,"range_topic"); });
    apply_param(params, "proximity_range", [&](auto const& v){ proximity_range_ = get_double(v,"proximity_range"); });
    apply_param(params, "min_conf",        [&](auto const& v){ min_conf_      = float(get_double(v,"min_conf")); });
    apply_param(params, "arrive_m",        [&](auto const& v){ arrive_m_      = get_double(v,"arrive_m"); });
    apply_param(params, "forget_ticks",    [&](auto const& v){ forget_ticks_  = std::max(1.0, get_double(v,"forget_ticks")); });
    apply_param(params, "floor",           [&](auto const& v){ floor_         = float(get_double(v,"floor")); });
    apply_param(params, "renew_topic",     [&](auto const& v){ renew_topic_   = get_string(v,"renew_topic"); });
    apply_param(params, "renew_min",       [&](auto const& v){ renew_min_     = float(get_double(v,"renew_min")); });
    apply_param(params, "renew_range",     [&](auto const& v){ renew_range_   = get_double(v,"renew_range"); });
    apply_param(params, "walk_refix_m",    [&](auto const& v){ walk_refix_m_  = get_double(v,"walk_refix_m"); });
    apply_param(params, "walk_take_range", [&](auto const& v){ walk_take_range_ = get_double(v,"walk_take_range"); });
    apply_param(params, "mover_topic",     [&](auto const& v){ mover_topic_   = get_string(v,"mover_topic"); });
    apply_param(params, "chase_gate_m",    [&](auto const& v){ chase_gate_m_  = get_double(v,"chase_gate_m"); });
    apply_param(params, "chase_confirm",   [&](auto const& v){ chase_confirm_ = std::max(1, int(get_double(v,"chase_confirm"))); });
    apply_param(params, "chase_confirm_ticks", [&](auto const& v){ chase_confirm_ticks_ = std::max(0, int(get_double(v,"chase_confirm_ticks"))); });
    apply_param(params, "chase_forget_ticks", [&](auto const& v){ chase_forget_ticks_ = std::max(1, int(get_double(v,"chase_forget_ticks"))); });
    apply_param(params, "chase_lead_s",    [&](auto const& v){ chase_lead_s_  = get_double(v,"chase_lead_s"); });
    apply_param(params, "chase_v_max",     [&](auto const& v){ chase_v_max_   = get_double(v,"chase_v_max"); });
    apply_param(params, "chase_min_v",     [&](auto const& v){ chase_min_v_   = get_double(v,"chase_min_v"); });
    apply_param(params, "chase_stop_v",    [&](auto const& v){ chase_stop_v_  = get_double(v,"chase_stop_v"); });
    apply_param(params, "chase_permanence_ticks", [&](auto const& v){ chase_permanence_ticks_ = std::max(0, int(get_double(v,"chase_permanence_ticks"))); });
    apply_param(params, "yield_topic",     [&](auto const& v){ yield_topic_   = get_string(v,"yield_topic"); });
    apply_param(params, "chase_yield_tall", [&](auto const& v){ chase_yield_tall_ = std::max(0, int(get_double(v,"chase_yield_tall"))); });
    apply_param(params, "chase_memory_holds", [&](auto const& v){ chase_memory_holds_ = get_double(v,"chase_memory_holds") > 0.5; });
    apply_param(params, "chase_memory_ticks", [&](auto const& v){ chase_memory_ticks_ = std::max(0, int(get_double(v,"chase_memory_ticks"))); });
    apply_param(params, "chase_pull_decay", [&](auto const& v){ chase_pull_decay_ = std::clamp(get_double(v,"chase_pull_decay"), 0.0, 1.0); });
    apply_param(params, "chase_pull_recover_ticks", [&](auto const& v){ chase_pull_recover_ticks_ = std::max(1.0, get_double(v,"chase_pull_recover_ticks")); });
}

void BearingSeekLoop::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "proximity_range") proximity_range_ = get_double(value, k);
    else if (k == "min_conf")        min_conf_ = float(get_double(value, k));
    else if (k == "arrive_m")        arrive_m_ = get_double(value, k);
    else if (k == "forget_ticks")    forget_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "floor")           floor_ = float(get_double(value, k));
    else if (k == "renew_min")       renew_min_ = float(get_double(value, k));
    else if (k == "renew_range")     renew_range_ = get_double(value, k);
    else if (k == "walk_refix_m")    walk_refix_m_ = get_double(value, k);
    else if (k == "walk_take_range") walk_take_range_ = get_double(value, k);
    else if (k == "chase_gate_m")    chase_gate_m_ = get_double(value, k);
    else if (k == "chase_confirm")   chase_confirm_ = std::max(1, int(get_double(value, k)));
    else if (k == "chase_confirm_ticks") chase_confirm_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "chase_forget_ticks") chase_forget_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "chase_lead_s")    chase_lead_s_ = get_double(value, k);
    else if (k == "chase_v_max")     chase_v_max_ = get_double(value, k);
    else if (k == "chase_min_v")     chase_min_v_ = get_double(value, k);
    else if (k == "chase_stop_v")    chase_stop_v_ = get_double(value, k);
    else if (k == "chase_permanence_ticks") chase_permanence_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "chase_yield_tall") chase_yield_tall_ = std::max(0, int(get_double(value, k)));
    else if (k == "chase_memory_holds") chase_memory_holds_ = get_double(value, k) > 0.5;
    else if (k == "chase_memory_ticks") chase_memory_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "chase_pull_decay") chase_pull_decay_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "chase_pull_recover_ticks") chase_pull_recover_ticks_ = std::max(1.0, get_double(value, k));
    else throw std::invalid_argument("BearingSeekLoop: param '" + k + "' is construction-only / unknown");
}

void BearingSeekLoop::tick(uint64_t tick_id) {
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(pose_topic_)))
        if (pt->values.size() >= 3) { px_ = pt->values[0]; py_ = pt->values[1]; pyaw_ = pt->values[2]; have_pose_ = true; }
    float vx = 0.0f, vy = 0.0f, prox = 0.0f; bool walking = false;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(bearing_topic_))) {
        if (pt->values.size() > 0) vx   = float(pt->values[0]);
        if (pt->values.size() > 1) vy   = float(pt->values[1]);
        if (pt->values.size() > 2) prox = float(pt->values[2]);
        if (pt->values.size() > 3) walking = pt->values[3] > 0.5f;
    }
    seen_ = prox > min_conf_ && (vx * vx + vy * vy) > 1e-6f;
    const double c = std::cos(pyaw_), s = std::sin(pyaw_);
    // a walking bearing: only a re-fix of a held target within walk_refix_m; otherwise as if unseen
    if (seen_ && walking) {
        bool refix = false;
        const double n = std::sqrt(double(vx) * vx + double(vy) * vy);
        const double range = std::max(0.0, 1.0 - double(prox)) * proximity_range_;
        if (walk_refix_m_ > 0.0 && have_target_ && have_pose_) {
            const double fwd = vy / n, left = -vx / n;
            const double bx = fwd * range, by = left * range;
            const double fx = px_ + c * bx - s * by, fy = py_ + s * bx + c * by;
            refix = std::hypot(fx - tx_, fy - ty_) <= walk_refix_m_;
            if (refix) ++refixes_;
        }
        // the take from the walk: a small thing close by, nothing held, nothing moving in mind
        const bool take = walk_take_range_ > 0.0 && !have_target_ && !chasing_ && !coasting_ && !have_memory_ && range <= walk_take_range_;
        if (take) ++walk_takes_;
        if (!refix && !take) seen_ = false;
    }
    // the renewal: with no target held (the arrival's tick has passed -- the value read 0 for one tick, which
    // is the arrival the outcome loop sees) and nothing in view, a need still open at the thing re-arms it
    if (!renew_topic_.empty() && !have_target_ && !seen_ && have_pose_) {
        if (auto rt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(renew_topic_))) {
            if (rt->values.size() >= 3 && rt->values[0] > renew_min_) {
                const double rr = std::hypot(double(rt->values[1]) - px_, double(rt->values[2]) - py_);
                if (rr > 1.5 * arrive_m_ && rr < renew_range_) {
                    tx_ = rt->values[1]; ty_ = rt->values[2]; have_target_ = true;
                    conf_ = std::clamp(rt->values[0], 0.0f, 1.0f); ++renewals_;
                }
            }
        }
    }
    // the chase: a mover sighting becomes a candidate, confirms by its own prediction, and preempts the static fix
    if (!mover_topic_.empty() && have_pose_) chase_tick(tick_id, c, s);
    pull_ = std::min(1.0, pull_ + 1.0 / chase_pull_recover_ticks_);
    if (have_memory_) {
        if (tick_id - mem_tick_ > uint64_t(chase_memory_ticks_)) have_memory_ = false;
        else mem_dt_ = double(tick_id - mem_tick_) / 50.0;
    }
    if (have_yield_ && tick_id - yield_tick_ > uint64_t(chase_memory_ticks_)) have_yield_ = false;
    if (chasing_ || coasting_) {
        const double dt = double(tick_id - cand_tick_) / 50.0 + chase_lead_s_;
        tx_ = cand_x_ + cand_vx_ * dt; ty_ = cand_y_ + cand_vy_ * dt;
        double need = pull_;
        if (coasting_) need *= std::max(0.0, 1.0 - double(tick_id - coast_from_) / double(std::max(1, chase_permanence_ticks_)));
        have_target_ = true; conf_ = float(need); ++chase_ticks_;
        const double dx = tx_ - px_, dy = ty_ - py_;
        range_left_ = std::hypot(dx, dy);
        const double bx = c * dx + s * dy, by = -s * dx + c * dy;
        if (range_left_ > 1e-6) { cx_ = float(-by / range_left_); cy_ = float(bx / range_left_); }
        else { cx_ = 0.0f; cy_ = 0.0f; }
    } else if (seen_ && have_pose_ && !(chase_memory_holds_ && have_memory_ && tick_id - mem_tick_ <= uint64_t(chase_memory_ticks_))) {
        // fix the thing's position: the body's pose plus the bearing (body frame: +x forward, +y left)
        // (not while a lost mover is still in mind: the moving thing keeps its priority)
        // at the range the proximity encodes
        const double n = std::sqrt(double(vx) * vx + double(vy) * vy);
        const double fwd = vy / n, left = -vx / n;                       // cx = +right → left = -cx
        const double range = std::max(0.0, 1.0 - double(prox)) * proximity_range_;
        const double bx = fwd * range, by = left * range;                // body frame
        tx_ = px_ + c * bx - s * by;
        ty_ = py_ + s * bx + c * by;
        have_target_ = true;
        conf_ = 1.0f;
        range_left_ = range;
        cx_ = vx / float(n); cy_ = vy / float(n);
    } else if (have_target_) {
        // home to the remembered position: re-aim from the body's current pose
        const double dx = tx_ - px_, dy = ty_ - py_;
        range_left_ = std::hypot(dx, dy);
        const double bx =  c * dx + s * dy;                               // world → body
        const double by = -s * dx + c * dy;
        if (range_left_ < arrive_m_) {
            have_target_ = false; conf_ = 0.0f; ++arrivals_;              // reached: the belief is fulfilled
            cx_ = 0.0f; cy_ = 0.0f;
        } else {
            conf_ *= float(1.0 - 1.0 / forget_ticks_);
            if (conf_ < floor_) { have_target_ = false; conf_ = 0.0f; ++forgets_; cx_ = 0.0f; cy_ = 0.0f; }
            else { cx_ = float(-by / range_left_); cy_ = float(bx / range_left_); }
        }
    } else {
        cx_ = 0.0f; cy_ = 0.0f; conf_ = 0.0f;
    }
    value_ = have_target_ ? conf_ : 0.0f;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("seek") : id_; out->sensor = "seek_bearing";
    out->values = Eigen::VectorXf::Zero(3);
    out->values[0] = cx_; out->values[1] = cy_; out->values[2] = value_;
    bus_->publish(output_topic_, out);
    auto v = std::make_shared<ProprioToken>();
    v->tick_id = tick_id; v->producer_id = out->producer_id; v->sensor = "seek_value";
    v->values = Eigen::VectorXf::Constant(1, value_);
    bus_->publish(value_topic_, v);
    if (!range_topic_.empty()) {
        auto r = std::make_shared<ProprioToken>();
        r->tick_id = tick_id; r->producer_id = out->producer_id; r->sensor = "seek_range";
        r->values = Eigen::VectorXf::Constant(1, float(range_left_));
        bus_->publish(range_topic_, r);
    }
}

void BearingSeekLoop::chase_tick(uint64_t tick_id, double c, double s) {
    lost_now_ = false; last_decision_ = 0;
    // the yield: the pursuit does not run into tall structure the thing turned away from
    if ((chasing_ || coasting_) && chase_yield_tall_ > 0 && !yield_topic_.empty()) {
        if (auto yt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(yield_topic_)))
            if (yt->values.size() >= 1 && int(yt->values[0]) >= chase_yield_tall_) {
                ++chases_yielded_;
                yield_to_structure(tick_id);
                return;
            }
    }
    float mx = 0.0f, my = 0.0f, mprox = 0.0f; bool fresh = true;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(mover_topic_))) {
        if (pt->tick_id == tick_id && pt->values.size() >= 3) {
            mx = pt->values[0]; my = pt->values[1]; mprox = pt->values[2];
            // a sighting is new only when the cloud recomputed its clusters (the token's sixth value); the bearing alone
            // is re-aimed every tick and would otherwise count four times over
            if (pt->values.size() >= 6) { fresh = double(pt->values[5]) != last_seq_; if (fresh) last_seq_ = double(pt->values[5]); }
        }
    }
    mover_seen_ = mprox > min_conf_ && (mx * mx + my * my) > 1e-6f;
    if (mover_seen_ && !fresh) mover_seen_ = false;   // the same sighting again: nothing new to judge
    if (mover_seen_) {
        const double n = std::sqrt(double(mx) * mx + double(my) * my);
        const double fwd = my / n, left = -mx / n, range = std::max(0.0, 1.0 - double(mprox)) * proximity_range_;
        const double bx = fwd * range, by = left * range;
        const double fx = px_ + c * bx - s * by, fy = py_ + s * bx + c * by;   // the sighting, odometry frame
        if (have_yield_ && tick_id - yield_tick_ <= uint64_t(chase_memory_ticks_) && std::hypot(fx - yield_x_, fy - yield_y_) <= chase_gate_m_) {
            ++yield_drops_;   // a sighting at the yielded place: part of the tall structure there, not a mover (sweep 13: the same sighting re-started the chase every tick)
        } else if (have_cand_ && tick_id > cand_tick_) {
            const double dt = double(tick_id - cand_tick_) / 50.0;
            const double ex = cand_x_ + cand_vx_ * dt, ey = cand_y_ + cand_vy_ * dt;   // where the candidate should be
            const double miss = std::hypot(fx - ex, fy - ey);
            const double vx = (fx - cand_x_) / dt, vy = (fy - cand_y_) / dt;
            // the speed test over the WATCH, not the last step: a centroid's jitter of 8 cm between casts 80 ms apart reads
            // as 1 m/s and replaced a crossing train at half a metre (2026-09-29); under 0.2 s of watching the gate alone judges
            const double watched = double(tick_id - cand_first_) / 50.0;
            // the speed over the ring of recent sightings (chase_v_window_s), not one step: a centroid's per-step jitter is 1 m/s
            while (!sight_.empty() && double(tick_id) - sight_.front()[0] > chase_v_window_s_ * 50.0) sight_.pop_front();
            double speed_w = 0.0, rvx = vx, rvy = vy;
            if (!sight_.empty()) {
                const double rdt = double(tick_id - uint64_t(sight_.front()[0])) / 50.0;
                if (rdt >= 0.2) { rvx = (fx - sight_.front()[1]) / rdt; rvy = (fy - sight_.front()[2]) / rdt; speed_w = std::hypot(rvx, rvy); }
            }
            last_miss_ = miss; last_speed_ = speed_w;
            const bool ok = miss <= chase_gate_m_ && speed_w <= chase_v_max_;
            last_decision_ = ok ? 1 : (miss <= chase_gate_m_ ? 3 : 2);
            if (ok) {
                // a confirmation: the velocity is the displacement since the FIRST sighting over the watch once there is
                // 0.2 s of it (a centroid's per-step jitter is 1 m/s; over a second it is 0.04), the first steps' own before
                if (speed_w > 0.0 || (!sight_.empty() && double(tick_id - uint64_t(sight_.front()[0])) / 50.0 >= 0.2)) { cand_vx_ = rvx; cand_vy_ = rvy; }
                else { const double a = cand_n_ >= 2 ? 0.5 : 1.0; cand_vx_ = (1.0 - a) * cand_vx_ + a * vx; cand_vy_ = (1.0 - a) * cand_vy_ + a * vy; }
                sight_.push_back({double(tick_id), fx, fy});
                cand_x_ = fx; cand_y_ = fy; cand_tick_ = tick_id; ++cand_n_;
                if (coasting_) { coasting_ = false; chasing_ = true; ++chases_reacquired_; }   // found where predicted: the chase resumes
            } else if (!chasing_ && !coasting_) {
                // an unconfirmed candidate that did not follow: this sighting is the new candidate
                if (miss <= chase_gate_m_) ++cand_fast_; else ++cand_replaced_;
                cand_x_ = cand_x0_ = fx; cand_y_ = cand_y0_ = fy; cand_vx_ = 0.0; cand_vy_ = 0.0; cand_tick_ = cand_first_ = tick_id; cand_n_ = 1;
                sight_.clear(); sight_.push_back({double(tick_id), fx, fy});
            }
            // (a chased target ignores a stray sighting; it is another thing)
        } else if (!have_cand_) {
            have_cand_ = true; last_decision_ = 4; last_miss_ = 0.0; last_speed_ = 0.0;
            cand_x_ = cand_x0_ = fx; cand_y_ = cand_y0_ = fy; cand_vx_ = 0.0; cand_vy_ = 0.0; cand_tick_ = cand_first_ = tick_id; cand_n_ = 1;
            sight_.clear(); sight_.push_back({double(tick_id), fx, fy});
            // the memory of a lost mover: a sighting where it should now be is the same thing, back in view
            if (have_memory_ && tick_id > mem_tick_ && tick_id - mem_tick_ <= uint64_t(chase_memory_ticks_)) {
                const double dt = double(tick_id - mem_tick_) / 50.0;
                if (std::hypot(fx - (mem_x_ + mem_vx_ * dt), fy - (mem_y_ + mem_vy_ * dt)) <= chase_gate_m_) {
                    cand_vx_ = mem_vx_; cand_vy_ = mem_vy_; cand_n_ = chase_confirm_;
                    cand_first_ = tick_id >= uint64_t(chase_confirm_ticks_) ? tick_id - uint64_t(chase_confirm_ticks_) : 0;
                    chasing_ = true; ++chases_; ++chases_reacquired_; have_memory_ = false;
                }
            }
        }
        if (!chasing_ && cand_n_ >= chase_confirm_ && tick_id - cand_first_ >= uint64_t(chase_confirm_ticks_)) {
            bool moved = true;
            if (chase_min_v_ > 0.0) {
                const double watched = double(tick_id - cand_first_) / 50.0;
                const double disp = std::hypot(cand_x_ - cand_x0_, cand_y_ - cand_y0_);
                moved = std::hypot(cand_vx_, cand_vy_) >= chase_min_v_ && watched > 0.0 && disp / watched >= chase_min_v_;
            }
            if (moved) { chasing_ = true; ++chases_; last_decision_ = 5; } else { ++cand_still_; last_decision_ = 6; }
        }
    }
    if (coasting_ && tick_id - coast_from_ >= uint64_t(chase_permanence_ticks_)) {
        lose(tick_id, c, s);                                          // the permanence ran out: the loss, the look
        return;
    }
    if (have_cand_ && !coasting_ && tick_id - cand_tick_ > uint64_t(chase_forget_ticks_)) {
        if (chasing_) {
            const bool stopped = chase_stop_v_ <= 0.0 || std::hypot(cand_vx_, cand_vy_) < chase_stop_v_;
            if (stopped) {
                // the thing stopped: where it was last seen is an ordinary remembered target from here
                tx_ = cand_x_; ty_ = cand_y_; have_target_ = true; conf_ = 1.0f; ++chases_stopped_;
                chasing_ = false; have_cand_ = false; cand_n_ = 0; cand_vx_ = 0.0; cand_vy_ = 0.0;
            } else if (chase_permanence_ticks_ > 0) {
                // the thing left the view still moving: keep it moving in mind (coasting), the candidate kept for a re-sighting
                chasing_ = false; coasting_ = true; coast_from_ = tick_id;
            } else {
                lose(tick_id, c, s);
            }
        } else {
            have_cand_ = false; cand_n_ = 0; cand_vx_ = 0.0; cand_vy_ = 0.0; ++cand_timeout_;   // an unconfirmed candidate, forgotten
        }
    }
}

double BearingSeekLoop::chase_gaze_ego() const {
    if (!have_pose_) return std::numeric_limits<double>::quiet_NaN();
    const double c = std::cos(pyaw_), s = std::sin(pyaw_);
    double gx, gy;
    if (chasing_ || coasting_) { gx = tx_; gy = ty_; }
    else if (have_memory_ && chase_memory_ticks_ > 0) {
        // where the lost thing should be by now (the memory's own clock is the loop's last tick: use its extrapolation as stored)
        gx = mem_x_ + mem_vx_ * mem_dt_; gy = mem_y_ + mem_vy_ * mem_dt_;
    } else return std::numeric_limits<double>::quiet_NaN();
    const double dx = gx - px_, dy = gy - py_;
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;
    return std::atan2(-by, bx);
}

void BearingSeekLoop::lose(uint64_t tick_id, double c, double s) {
    // the thing left the view still moving: it is not at the place; nothing to walk to -- but where it was last
    // predicted to be is where to LOOK (the host may start a stop on lost_now_); and the pull decays
    const double dt = double(tick_id - cand_tick_) / 50.0;
    const double lx = cand_x_ + cand_vx_ * dt, ly = cand_y_ + cand_vy_ * dt;
    const double dx = lx - px_, dy = ly - py_;
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;          // body frame: x forward, y left
    lost_ego_ = std::atan2(-by, bx); lost_range_ = std::hypot(bx, by); lost_now_ = true;
    have_target_ = false; conf_ = 0.0f; cx_ = 0.0f; cy_ = 0.0f; ++chases_lost_;
    pull_ *= chase_pull_decay_;
    if (chase_memory_ticks_ > 0) { have_memory_ = true; mem_x_ = lx; mem_y_ = ly; mem_vx_ = cand_vx_; mem_vy_ = cand_vy_; mem_tick_ = tick_id; }
    chasing_ = false; coasting_ = false; have_cand_ = false; cand_n_ = 0; cand_vx_ = 0.0; cand_vy_ = 0.0;
}

void BearingSeekLoop::yield_to_structure(uint64_t tick_id) {
    // the thing stands at the foot of tall structure: it is part of that structure, not a mover.  Nothing to look for
    // and no mover to keep in mind (sweep 13, 2026-09-29: a yield that went through lose() left the memory, the next
    // sighting re-acquired it and yielded again, 288 of 365 chases ending on their first tick).  The PLACE is remembered
    // as not-a-mover for the memory's lifetime: a sighting within the gate of it is dropped, one further off is a fresh
    // candidate again.
    if (chase_memory_ticks_ > 0) { have_yield_ = true; yield_x_ = tx_; yield_y_ = ty_; yield_tick_ = tick_id; }
    have_target_ = false; conf_ = 0.0f; cx_ = 0.0f; cy_ = 0.0f;
    chasing_ = false; coasting_ = false; have_cand_ = false; cand_n_ = 0; cand_vx_ = 0.0; cand_vy_ = 0.0;
}

nlohmann::json BearingSeekLoop::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"have_target", have_target_}, {"tx", tx_}, {"ty", ty_}, {"conf", conf_},
                          {"arrivals", arrivals_}, {"forgets", forgets_}, {"renewals", renewals_}};
}
void BearingSeekLoop::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    have_target_ = s.value("have_target", false); tx_ = s.value("tx", 0.0); ty_ = s.value("ty", 0.0);
    conf_ = s.value("conf", 0.0f); arrivals_ = s.value("arrivals", 0); forgets_ = s.value("forgets", 0); renewals_ = s.value("renewals", 0);
}
nlohmann::json BearingSeekLoop::diag_lite() const {
    return nlohmann::json{{"seen", seen_}, {"target", have_target_}, {"value", value_}, {"range", range_left_},
                          {"arrivals", arrivals_}, {"forgets", forgets_}, {"renewals", renewals_}, {"refixes", refixes_}};
}
nlohmann::json BearingSeekLoop::diag_snapshot() const {
    nlohmann::json j = diag_lite();
    j["cx"] = cx_; j["cy"] = cy_; j["tx"] = tx_; j["ty"] = ty_;
    return j;
}

} // namespace ogma
