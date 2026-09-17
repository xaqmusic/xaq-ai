// =============================================================================
// SkillOutcomeLoop.cpp  --  see the header
// =============================================================================
#include "ogma/modules/SkillOutcomeLoop.hpp"

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
    throw std::invalid_argument("SkillOutcomeLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("SkillOutcomeLoop: param '" + k + "' must be a string");
}
}  // namespace

SkillOutcomeLoop::SkillOutcomeLoop()  = default;
SkillOutcomeLoop::~SkillOutcomeLoop() = default;

std::string_view SkillOutcomeLoop::type_name() const { return "SkillOutcomeLoop"; }

std::vector<TopicSpec> SkillOutcomeLoop::input_topics() const {
    return { TopicSpec{bearing_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{thing_topic_,      std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false},
             TopicSpec{seek_value_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{seek_range_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{pose_topic_,       std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> SkillOutcomeLoop::output_topics() const {
    return { TopicSpec{skill_topic_,   std::type_index(typeid(ProprioToken))},
             TopicSpec{outcome_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema SkillOutcomeLoop::params_schema() const {
    return {
        {"bearing_topic",    ParamMutability::ConstructionOnly, "The attended thing's [vx=+right, vy=+forward, proximity] (CloudMap thing_bearing_topic).", ParamValue{std::string("percept.thing_bearing")}},
        {"thing_topic",      ParamMutability::ConstructionOnly, "The thing EPM's RealityToken: its winner is the node the outcome is learned for.", ParamValue{std::string("reality.cognitive.thing")}},
        {"seek_value_topic", ParamMutability::ConstructionOnly, "The seek loop's need; its fall to 0 with the range under arrive_range is the arrival.", ParamValue{std::string("reality.cognitive.seek_value")}},
        {"seek_range_topic", ParamMutability::ConstructionOnly, "The seek loop's range left.", ParamValue{std::string("reality.cognitive.seek_range")}},
        {"pose_topic",       ParamMutability::ConstructionOnly, "The body's dead-reckoned [x, y, yaw].", ParamValue{std::string("reality.proprio.odom")}},
        {"skill_topic",      ParamMutability::ConstructionOnly, "The intent boundary: ProprioToken [skill id, request] with request 1 on the tick a skill is asked for.", ParamValue{std::string("intent.skill")}},
        {"outcome_topic",    ParamMutability::ConstructionOnly, "The loop's honest signal: [node, predicted displacement, observed displacement, surprise, samples] on the tick an outcome is observed; zeros otherwise.", ParamValue{std::string("reality.cognitive.outcome")}},
        {"proximity_range",  ParamMutability::HotMutable, "The bearing's proximity scale (metres): range = (1 - proximity) x this.", ParamValue{2.5}},
        {"arrive_range",     ParamMutability::HotMutable, "The seek range under which a need falling to 0 is an arrival (metres).", ParamValue{0.3}},
        {"match_radius",     ParamMutability::HotMutable, "A bearing after the kick whose fixed position lies within this of the kicked thing's is the same thing, moved (metres).", ParamValue{0.6}},
        {"min_samples",      ParamMutability::HotMutable, "A node with fewer recorded outcomes than this is always worth a kick.", ParamValue{int64_t{2}}},
        {"observe_ticks",    ParamMutability::HotMutable, "How long after a kick to wait for the thing to be seen again before the outcome is unknown.", ParamValue{int64_t{1500}}},
        {"min_conf_ticks",   ParamMutability::HotMutable, "A bearing must be live this many ticks in a row before its position is fixed.", ParamValue{int64_t{5}}},
        {"explore_gain",     ParamMutability::HotMutable, "Kick a known node when its outcome spread exceeds this x the mean spread over nodes; 0 = only unknown nodes.", ParamValue{1.0}},
        {"skill_left",       ParamMutability::ConstructionOnly, "The id of the left kick on the boundary.", ParamValue{int64_t{0}}},
        {"skill_right",      ParamMutability::ConstructionOnly, "The id of the right kick.", ParamValue{int64_t{1}}},
    };
}

ParamMap SkillOutcomeLoop::current_params() const {
    ParamMap m;
    m["bearing_topic"] = ParamValue{bearing_topic_}; m["thing_topic"] = ParamValue{thing_topic_};
    m["seek_value_topic"] = ParamValue{seek_value_topic_}; m["seek_range_topic"] = ParamValue{seek_range_topic_};
    m["pose_topic"] = ParamValue{pose_topic_}; m["skill_topic"] = ParamValue{skill_topic_}; m["outcome_topic"] = ParamValue{outcome_topic_};
    m["proximity_range"] = ParamValue{proximity_range_}; m["arrive_range"] = ParamValue{arrive_range_}; m["match_radius"] = ParamValue{match_radius_};
    m["min_samples"] = ParamValue{int64_t{min_samples_}}; m["observe_ticks"] = ParamValue{int64_t{observe_ticks_}}; m["min_conf_ticks"] = ParamValue{int64_t{min_conf_ticks_}};
    m["explore_gain"] = ParamValue{explore_gain_}; m["skill_left"] = ParamValue{int64_t{skill_left_}}; m["skill_right"] = ParamValue{int64_t{skill_right_}};
    return m;
}

void SkillOutcomeLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("SkillOutcomeLoop requires a non-null Bus");
    apply_param(params, "bearing_topic",    [&](auto const& v){ bearing_topic_ = get_string(v,"bearing_topic"); });
    apply_param(params, "thing_topic",      [&](auto const& v){ thing_topic_ = get_string(v,"thing_topic"); });
    apply_param(params, "seek_value_topic", [&](auto const& v){ seek_value_topic_ = get_string(v,"seek_value_topic"); });
    apply_param(params, "seek_range_topic", [&](auto const& v){ seek_range_topic_ = get_string(v,"seek_range_topic"); });
    apply_param(params, "pose_topic",       [&](auto const& v){ pose_topic_ = get_string(v,"pose_topic"); });
    apply_param(params, "skill_topic",      [&](auto const& v){ skill_topic_ = get_string(v,"skill_topic"); });
    apply_param(params, "outcome_topic",    [&](auto const& v){ outcome_topic_ = get_string(v,"outcome_topic"); });
    apply_param(params, "proximity_range",  [&](auto const& v){ proximity_range_ = get_double(v,"proximity_range"); });
    apply_param(params, "arrive_range",     [&](auto const& v){ arrive_range_ = get_double(v,"arrive_range"); });
    apply_param(params, "match_radius",     [&](auto const& v){ match_radius_ = get_double(v,"match_radius"); });
    apply_param(params, "min_samples",      [&](auto const& v){ min_samples_ = int(get_double(v,"min_samples")); });
    apply_param(params, "observe_ticks",    [&](auto const& v){ observe_ticks_ = int(get_double(v,"observe_ticks")); });
    apply_param(params, "min_conf_ticks",   [&](auto const& v){ min_conf_ticks_ = int(get_double(v,"min_conf_ticks")); });
    apply_param(params, "explore_gain",     [&](auto const& v){ explore_gain_ = get_double(v,"explore_gain"); });
    apply_param(params, "skill_left",       [&](auto const& v){ skill_left_ = int(get_double(v,"skill_left")); });
    apply_param(params, "skill_right",      [&](auto const& v){ skill_right_ = int(get_double(v,"skill_right")); });
}

void SkillOutcomeLoop::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "proximity_range") proximity_range_ = get_double(value, k);
    else if (k == "arrive_range")    arrive_range_ = get_double(value, k);
    else if (k == "match_radius")    match_radius_ = get_double(value, k);
    else if (k == "min_samples")     min_samples_ = int(get_double(value, k));
    else if (k == "observe_ticks")   observe_ticks_ = int(get_double(value, k));
    else if (k == "min_conf_ticks")  min_conf_ticks_ = int(get_double(value, k));
    else if (k == "explore_gain")    explore_gain_ = get_double(value, k);
    else throw std::invalid_argument("SkillOutcomeLoop: param '" + k + "' is construction-only / unknown");
}

void SkillOutcomeLoop::tick(uint64_t tick_id) {
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(pose_topic_)))
        if (pt->values.size() >= 3) { px_ = pt->values[0]; py_ = pt->values[1]; pyaw_ = pt->values[2]; have_pose_ = true; }
    float vx = 0.0f, vy = 0.0f, prox = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(bearing_topic_)))
        if (pt->values.size() >= 3) { vx = pt->values[0]; vy = pt->values[1]; prox = pt->values[2]; }
    int node = -1;
    if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(thing_topic_))) node = rt->winner_id;
    float seek_v = 0.0f, seek_r = 9.0f;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(seek_value_topic_))) if (pt->values.size() > 0) seek_v = pt->values[0];
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(seek_range_topic_))) if (pt->values.size() > 0) seek_r = pt->values[0];

    // the thing as seen now: fix its position once the bearing has been live for a few ticks
    const bool live = prox > 0.02f && (vx * vx + vy * vy) > 1e-6f && have_pose_;
    seen_run_ = live ? seen_run_ + 1 : 0;
    double sx = 0.0, sy = 0.0; bool fixed = false;
    if (live && seen_run_ >= min_conf_ticks_) {
        const double n = std::sqrt(double(vx) * vx + double(vy) * vy);
        const double fwd = vy / n, left = -vx / n, range = std::max(0.0, 1.0 - double(prox)) * proximity_range_;
        const double bx = fwd * range, by = left * range, c = std::cos(pyaw_), s = std::sin(pyaw_);
        sx = px_ + c * bx - s * by; sy = py_ + s * bx + c * by; fixed = true;
        seen_ = true; tx_ = sx; ty_ = sy; if (node >= 0) node_ = node;
    }

    // an outcome in flight: the same thing seen again within the radius is the answer
    bool outcome_now = false;
    if (pending_) {
        ++wait_;
        if (fixed && std::hypot(sx - kx_, sy - ky_) < match_radius_ && wait_ > 25) {
            const double disp = std::hypot(sx - kx_, sy - ky_);
            Stat& st = stats_[knode_];
            const double pred = st.n > 0 ? st.mean : 0.0, sd = std::sqrt(st.var());
            last_pred_ = pred; last_obs_ = disp; last_node_ = knode_;
            last_surprise_ = std::fabs(disp - pred) / (sd + 0.02);
            st.n += 1; const double d = disp - st.mean; st.mean += d / st.n; st.m2 += d * (disp - st.mean);
            ++observed_; pending_ = false; outcome_now = true;
        } else if (wait_ > observe_ticks_) { ++unknown_; pending_ = false; }
    }

    // the arrival: the seek need falls to 0 with the range under arrive_range -- kick if the answer is uncertain
    request_now_ = false;
    if (seek_prev_ > 0.0f && seek_v == 0.0f && seek_r < arrive_range_ && seen_ && !pending_) {
        const Stat& st = stats_[node_ < 0 ? 0 : node_];
        double mean_sd = 0.0; int nn = 0;
        for (auto const& [k, s] : stats_) if (s.n >= min_samples_) { mean_sd += std::sqrt(s.var()); ++nn; }
        mean_sd = nn ? mean_sd / nn : 0.0;
        const bool uncertain = st.n < min_samples_ || (explore_gain_ > 0.0 && std::sqrt(st.var()) > explore_gain_ * mean_sd);
        if (uncertain) {
            // the side: the thing's bearing from the body now (+ left in the body frame => the left foot)
            const double dx = tx_ - px_, dy = ty_ - py_, c = std::cos(pyaw_), s = std::sin(pyaw_);
            const double by = -s * dx + c * dy;
            request_id_ = by >= 0.0 ? skill_left_ : skill_right_;
            request_now_ = true; ++requests_;
            pending_ = true; kx_ = tx_; ky_ = ty_; knode_ = node_ < 0 ? 0 : node_; wait_ = 0; kicked_tick_ = tick_id;
        }
    }
    seek_prev_ = seek_v;

    auto sk = std::make_shared<ProprioToken>();
    sk->tick_id = tick_id; sk->producer_id = id_.empty() ? std::string("outcome") : id_; sk->sensor = "skill_request";
    sk->values = Eigen::VectorXf(2); sk->values[0] = float(request_id_); sk->values[1] = request_now_ ? 1.0f : 0.0f;
    bus_->publish(skill_topic_, sk);
    auto oc = std::make_shared<ProprioToken>();
    oc->tick_id = tick_id; oc->producer_id = sk->producer_id; oc->sensor = "outcome";
    oc->values = Eigen::VectorXf::Zero(5);
    if (outcome_now) { oc->values[0] = float(last_node_); oc->values[1] = float(last_pred_); oc->values[2] = float(last_obs_); oc->values[3] = float(last_surprise_); oc->values[4] = float(stats_[last_node_].n); }
    bus_->publish(outcome_topic_, oc);
}

nlohmann::json SkillOutcomeLoop::snapshot_state() const {
    nlohmann::json st = nlohmann::json::object();
    for (auto const& [k, s] : stats_) st[std::to_string(k)] = {{"n", s.n}, {"mean", s.mean}, {"m2", s.m2}};
    return nlohmann::json{{"version", 1}, {"stats", st}, {"requests", requests_}, {"observed", observed_}, {"unknown", unknown_}};
}
void SkillOutcomeLoop::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    stats_.clear();
    for (auto const& [k, v] : s.value("stats", nlohmann::json::object()).items())
        stats_[std::stoi(k)] = Stat{v.value("n", 0), v.value("mean", 0.0), v.value("m2", 0.0)};
    requests_ = s.value("requests", 0); observed_ = s.value("observed", 0); unknown_ = s.value("unknown", 0);
}
nlohmann::json SkillOutcomeLoop::diag_lite() const {
    return nlohmann::json{{"requests", requests_}, {"observed", observed_}, {"unknown", unknown_}, {"pending", pending_},
                          {"node", node_}, {"surprise", last_surprise_}, {"nodes_known", int(std::count_if(stats_.begin(), stats_.end(), [&](auto const& kv){ return kv.second.n >= min_samples_; }))}};
}
nlohmann::json SkillOutcomeLoop::diag_snapshot() const {
    nlohmann::json j = diag_lite();
    nlohmann::json st = nlohmann::json::object();
    for (auto const& [k, s] : stats_) st[std::to_string(k)] = {{"n", s.n}, {"mean", s.mean}, {"sd", std::sqrt(s.var())}};
    j["stats"] = st; j["tx"] = tx_; j["ty"] = ty_;
    return j;
}

}  // namespace ogma
