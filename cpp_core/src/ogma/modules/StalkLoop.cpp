// =============================================================================
// StalkLoop.cpp  --  see the header (the duck, design doc §17.113)
// =============================================================================
#include "ogma/modules/StalkLoop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <vector>

namespace ogma {

namespace {
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("StalkLoop: param '" + k + "' must be a string");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("StalkLoop: param '" + k + "' must be numeric");
}
} // namespace

double StalkLoop::Place::expected() const {
    if (intervals.empty()) return 0.0;
    std::vector<double> v(intervals.begin(), intervals.end());
    std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
    return v[v.size() / 2];
}

StalkLoop::StalkLoop()  = default;
StalkLoop::~StalkLoop() = default;

std::string_view StalkLoop::type_name() const { return "StalkLoop"; }

std::vector<TopicSpec> StalkLoop::input_topics() const {
    return { TopicSpec{mover_pos_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{map_topic_,       std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
             TopicSpec{target_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{mind_topic_,      std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{pose_topic_,      std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}

std::vector<TopicSpec> StalkLoop::output_topics() const {
    return { TopicSpec{ambush_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema StalkLoop::params_schema() const {
    return {
        {"mover_pos_topic", ParamMutability::ConstructionOnly, "The seek loop's mover_pos_topic [x/2, y/2] (confirmed mover sightings).", std::nullopt},
        {"map_topic", ParamMutability::ConstructionOnly, "The motion map EPM's RealityToken over those sightings (its winner = the passing place).", std::nullopt},
        {"target_topic", ParamMutability::ConstructionOnly, "The seek loop's target_out_topic [have, x, y, set tick, chasing, source].", std::nullopt},
        {"mind_topic", ParamMutability::ConstructionOnly, "The seek loop's mind_topic [a mover in mind]: the ambush waits for it to empty.", std::nullopt},
        {"pose_topic", ParamMutability::ConstructionOnly, "The odometry [x, y, yaw].", std::nullopt},
        {"ambush_topic", ParamMutability::ConstructionOnly, "Output [have, x, y, need, waiting] -- the seek loop's ambush_topic.", std::nullopt},
        {"pass_gap_ticks", ParamMutability::HotMutable, "Sightings further apart than this start a new pass (the mover-in-mind span).", ParamValue{int64_t{250}}},
        {"min_passes", ParamMutability::HotMutable, "A passing place is one the mover came back through at least this often.", ParamValue{int64_t{2}}},
        {"arrive_m", ParamMutability::HotMutable, "Within this of the place the wait begins (metres).", ParamValue{0.25}},
    };
}

ParamMap StalkLoop::current_params() const {
    ParamMap m;
    m["mover_pos_topic"] = ParamValue{mover_pos_topic_}; m["map_topic"] = ParamValue{map_topic_};
    m["target_topic"] = ParamValue{target_topic_}; m["mind_topic"] = ParamValue{mind_topic_};
    m["pose_topic"] = ParamValue{pose_topic_}; m["ambush_topic"] = ParamValue{ambush_topic_};
    m["pass_gap_ticks"] = ParamValue{int64_t(pass_gap_ticks_)}; m["min_passes"] = ParamValue{int64_t(min_passes_)};
    m["arrive_m"] = ParamValue{arrive_m_};
    return m;
}

void StalkLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("StalkLoop requires a non-null Bus");
    for (auto const& [k, dst] : {std::pair<const char*, std::string*>{"mover_pos_topic", &mover_pos_topic_}, {"map_topic", &map_topic_},
                                 {"target_topic", &target_topic_}, {"mind_topic", &mind_topic_}, {"pose_topic", &pose_topic_},
                                 {"ambush_topic", &ambush_topic_}}) {
        auto it = params.find(k);
        if (it == params.end()) throw std::invalid_argument(std::string("StalkLoop: ") + k + " is required");
        *dst = get_string(it->second, k);
    }
    if (auto it = params.find("pass_gap_ticks"); it != params.end()) pass_gap_ticks_ = std::max(1, int(get_double(it->second, "pass_gap_ticks")));
    if (auto it = params.find("min_passes"); it != params.end()) min_passes_ = std::max(1, int(get_double(it->second, "min_passes")));
    if (auto it = params.find("arrive_m"); it != params.end()) arrive_m_ = get_double(it->second, "arrive_m");
}

void StalkLoop::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "pass_gap_ticks") pass_gap_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "min_passes")     min_passes_ = std::max(1, int(get_double(value, k)));
    else if (k == "arrive_m")       arrive_m_ = get_double(value, k);
    else throw std::invalid_argument("StalkLoop: param '" + k + "' is construction-only / unknown");
}

void StalkLoop::tick(uint64_t tick_id) {
    if (auto p = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(pose_topic_)))
        if (p->values.size() >= 2) { px_ = p->values[0]; py_ = p->values[1]; }
    bool chasing = false, have = false; int src = 0;
    if (auto t = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(target_topic_)))
        if (t->values.size() >= 5) { have = t->values[0] > 0.5f; chasing = t->values[4] > 0.5f; src = t->values.size() >= 6 ? int(t->values[5]) : 0; }
    double mind = 0.0;
    if (auto m = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(mind_topic_)))
        if (m->values.size() >= 1) mind = m->values[0];

    // the passing places: each confirmed sighting into its motion-map node
    if (auto mp = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(mover_pos_topic_)))
        if (mp->values.size() >= 2 && mp->tick_id != mp_seen_) {
            mp_seen_ = mp->tick_id;
            if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(map_topic_)))
                if (rt->winner_id >= 0) {
                    Place& pl = nodes_[rt->winner_id];
                    pl.sx += 2.0 * mp->values[0]; pl.sy += 2.0 * mp->values[1]; ++pl.n;
                    if (!pl.seen || mp->tick_id - pl.last > uint64_t(pass_gap_ticks_)) {
                        if (pl.seen) { pl.intervals.push_back(double(mp->tick_id - pl.pass_start)); if (pl.intervals.size() > 16) pl.intervals.pop_front(); }
                        ++pl.passes; pl.pass_start = mp->tick_id;
                    }
                    pl.last = mp->tick_id; pl.seen = true;
                }
        }

    // a chase: the mover is the chase's; a wait that sees one has paid
    if (chasing) {
        if (state_ == State::Waiting && node_ >= 0) { nodes_[node_].a += 1.0; ++paid_; }
        chased_ = true; state_ = State::Idle; node_ = -1; last_failed_ = -1;
    }
    chasing_prev_ = chasing;

    if (state_ != State::Idle && have && src != 4) { state_ = State::Idle; node_ = -1; }   // the seek took another target (a thing seen)
    if (state_ == State::Going && node_ >= 0) {
        const Place& pl = nodes_[node_];
        if (std::hypot(pl.x() - px_, pl.y() - py_) <= arrive_m_) { state_ = State::Waiting; wait_start_ = tick_id; }
    }
    if (state_ == State::Waiting && node_ >= 0) {
        Place& pl = nodes_[node_];
        const double expect = pl.expected();
        if (expect > 0.0 && double(tick_id - wait_start_) > expect) {   // outlasted the place's own return: nothing came
            pl.b += 1.0; ++failed_; last_failed_ = node_; state_ = State::Idle; node_ = -1;
        }
    }
    // the choice: after a chase, the mover out of mind, nothing else held -- the place most likely to pay, nearest first
    if (state_ == State::Idle && chased_ && !chasing && mind <= 0.0 && !have) {
        int best = -1; double bs = -1.0;
        for (auto const& [w, pl] : nodes_) {
            if (pl.passes < min_passes_ || pl.expected() <= 0.0 || w == last_failed_) continue;
            const double s = pl.p() / (1.0 + std::hypot(pl.x() - px_, pl.y() - py_));
            if (s > bs) { bs = s; best = w; }
        }
        if (best >= 0) { node_ = best; state_ = State::Going; ++ambushes_; }
    }
    need_ = (state_ != State::Idle && node_ >= 0) ? nodes_[node_].p() : 0.0;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("stalk") : id_; out->sensor = "ambush";
    out->values = Eigen::VectorXf::Zero(5);
    if (state_ != State::Idle && node_ >= 0) {
        const Place& pl = nodes_[node_];
        out->values[0] = 1.0f; out->values[1] = float(pl.x()); out->values[2] = float(pl.y());
        out->values[3] = float(need_); out->values[4] = state_ == State::Waiting ? 1.0f : 0.0f;
    }
    bus_->publish(ambush_topic_, out);
}

nlohmann::json StalkLoop::diag_snapshot() const {
    nlohmann::json places = nlohmann::json::array();
    for (auto const& [w, pl] : nodes_)
        places.push_back({w, pl.n, pl.passes, pl.expected() / 50.0, pl.p(), pl.x(), pl.y()});
    return nlohmann::json{{"state", int(state_)}, {"node", node_}, {"need", need_}, {"ambushes", ambushes_},
                          {"waits_paid", paid_}, {"waits_failed", failed_}, {"places", places}};
}

} // namespace ogma
