// =============================================================================
// HeadingPlanner.cpp  --  learned heading SELECTION
// =============================================================================
#include "ogma/modules/HeadingPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("HeadingPlanner param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("HeadingPlanner param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("HeadingPlanner param '" + key + "' must be string");
}
float wrap_pi(float a) {
    while (a >  kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}
}

HeadingPlanner::HeadingPlanner()  = default;
HeadingPlanner::~HeadingPlanner() = default;

std::string_view HeadingPlanner::type_name() const { return "HeadingPlanner"; }

std::vector<TopicSpec> HeadingPlanner::input_topics() const {
    return { TopicSpec{belief_topic_,   std::type_index(typeid(ProprioToken))},
             TopicSpec{progress_topic_, std::type_index(typeid(ProprioToken))},
             TopicSpec{heading_topic_,  std::type_index(typeid(ProprioToken))} };
}
std::vector<TopicSpec> HeadingPlanner::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema HeadingPlanner::params_schema() const {
    return {
        {"belief_topic", ParamMutability::ConstructionOnly,
            "Egocentric goal belief [bx,by] (mag=confidence), e.g. percept.goal_belief.",
            ParamValue{std::string("percept.goal_belief")}},
        {"progress_topic", ParamMutability::ConstructionOnly,
            "Scalar proximity truth whose Δ over a commit window is the reward (e.g. reality.proprio.scent_max).",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Absolute body yaw (rad) for achieved-heading credit.", ParamValue{std::string("reality.proprio.heading")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Chosen heading [hx,hy]·confidence → HeadingController.", ParamValue{std::string("plan.heading")}},
        {"bx_index", ParamMutability::ConstructionOnly, "+right index in belief token.", ParamValue{int64_t{0}}},
        {"by_index", ParamMutability::ConstructionOnly, "+forward index in belief token.", ParamValue{int64_t{1}}},
        {"n_state_sectors",  ParamMutability::ConstructionOnly, "Belief-bearing state bins.",  ParamValue{int64_t{8}}},
        {"n_action_sectors", ParamMutability::ConstructionOnly, "Candidate heading bins.",     ParamValue{int64_t{8}}},
        {"commit_ticks",  ParamMutability::HotMutable, "Hold a chosen heading K ticks (short → less braking-fight).", ParamValue{int64_t{6}}},
        {"value_lr",      ParamMutability::HotMutable, "EMA rate of V toward Δprogress.", ParamValue{0.1}},
        {"epistemic_gain",ParamMutability::HotMutable, "Exploration bonus weight (1/(1+visits)).", ParamValue{0.5}},
        {"temperature",   ParamMutability::HotMutable, "Softmax sampling temperature (<=0 = argmax).", ParamValue{1.0}},
        {"min_signal",    ParamMutability::HotMutable, "Belief-confidence floor for a valid state.", ParamValue{0.1}},
        {"credit_achieved", ParamMutability::HotMutable, "Credit the ACHIEVED heading (Δyaw) instead of the commanded one.", ParamValue{false}},
        {"hit_drop_thresh", ParamMutability::HotMutable, "One-tick scent_max drop > this = hit-teleport in window.", ParamValue{0.2}},
        {"r_hit",         ParamMutability::HotMutable, "Reward credited on a hit-in-window (strong positive).", ParamValue{1.0}},
        {"allow_resector_abort", ParamMutability::HotMutable, "Abort a stale commit if belief sector shifts >1.", ParamValue{false}},
        {"shuffle",       ParamMutability::HotMutable, "ABLATION: ignore V, pick a random action each commit.", ParamValue{false}},
        {"master_seed",   ParamMutability::ConstructionOnly, "RNG seed for exploration sampling.", ParamValue{int64_t{7}}},
    };
}

ParamMap HeadingPlanner::current_params() const {
    ParamMap m;
    m["belief_topic"]   = ParamValue{belief_topic_};
    m["progress_topic"] = ParamValue{progress_topic_};
    m["heading_topic"]  = ParamValue{heading_topic_};
    m["output_topic"]   = ParamValue{output_topic_};
    m["bx_index"]       = ParamValue{int64_t(bx_index_)};
    m["by_index"]       = ParamValue{int64_t(by_index_)};
    m["n_state_sectors"]  = ParamValue{int64_t(n_state_sectors_)};
    m["n_action_sectors"] = ParamValue{int64_t(n_action_sectors_)};
    m["commit_ticks"]   = ParamValue{int64_t(commit_ticks_)};
    m["value_lr"]       = ParamValue{double(value_lr_)};
    m["epistemic_gain"] = ParamValue{double(epistemic_gain_)};
    m["temperature"]    = ParamValue{double(temperature_)};
    m["min_signal"]     = ParamValue{double(min_signal_)};
    m["credit_achieved"]= ParamValue{credit_achieved_};
    m["hit_drop_thresh"]= ParamValue{double(hit_drop_thresh_)};
    m["r_hit"]          = ParamValue{double(r_hit_)};
    m["allow_resector_abort"] = ParamValue{allow_resector_abort_};
    m["shuffle"]        = ParamValue{shuffle_};
    m["master_seed"]    = ParamValue{int64_t(master_seed_)};
    return m;
}

void HeadingPlanner::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("HeadingPlanner requires a non-null Bus");

    apply_param(params, "belief_topic",   [&](auto const& v){ belief_topic_   = get_string(v, "belief_topic"); });
    apply_param(params, "progress_topic", [&](auto const& v){ progress_topic_ = get_string(v, "progress_topic"); });
    apply_param(params, "heading_topic",  [&](auto const& v){ heading_topic_  = get_string(v, "heading_topic"); });
    apply_param(params, "output_topic",   [&](auto const& v){ output_topic_   = get_string(v, "output_topic"); });
    apply_param(params, "bx_index", [&](auto const& v){ bx_index_ = int(get_int(v, "bx_index")); });
    apply_param(params, "by_index", [&](auto const& v){ by_index_ = int(get_int(v, "by_index")); });
    apply_param(params, "n_state_sectors",  [&](auto const& v){ n_state_sectors_  = std::max(2, int(get_int(v, "n_state_sectors"))); });
    apply_param(params, "n_action_sectors", [&](auto const& v){ n_action_sectors_ = std::max(2, int(get_int(v, "n_action_sectors"))); });
    apply_param(params, "commit_ticks",   [&](auto const& v){ commit_ticks_   = std::max(1, int(get_int(v, "commit_ticks"))); });
    apply_param(params, "value_lr",       [&](auto const& v){ value_lr_       = float(get_double(v, "value_lr")); });
    apply_param(params, "epistemic_gain", [&](auto const& v){ epistemic_gain_ = float(get_double(v, "epistemic_gain")); });
    apply_param(params, "temperature",    [&](auto const& v){ temperature_    = float(get_double(v, "temperature")); });
    apply_param(params, "min_signal",     [&](auto const& v){ min_signal_     = float(get_double(v, "min_signal")); });
    apply_param(params, "credit_achieved",[&](auto const& v){ if (auto p = std::get_if<bool>(&v)) credit_achieved_ = *p; });
    apply_param(params, "hit_drop_thresh",[&](auto const& v){ hit_drop_thresh_= float(get_double(v, "hit_drop_thresh")); });
    apply_param(params, "r_hit",          [&](auto const& v){ r_hit_          = float(get_double(v, "r_hit")); });
    apply_param(params, "allow_resector_abort", [&](auto const& v){ if (auto p = std::get_if<bool>(&v)) allow_resector_abort_ = *p; });
    apply_param(params, "shuffle",        [&](auto const& v){ if (auto p = std::get_if<bool>(&v)) shuffle_ = *p; });
    apply_param(params, "master_seed",    [&](auto const& v){ master_seed_    = uint64_t(get_int(v, "master_seed")); });

    V_.assign(size_t(n_state_sectors_) * size_t(n_action_sectors_), 0.0f);
    visits_.assign(V_.size(), 0);
    rng_.seed(master_seed_);

    sub_ids_.push_back(bus_->subscribe(belief_topic_,   SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_belief(p); }));
    sub_ids_.push_back(bus_->subscribe(progress_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_progress(p); }));
    sub_ids_.push_back(bus_->subscribe(heading_topic_,  SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_heading(p); }));
}

void HeadingPlanner::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "commit_ticks")   commit_ticks_   = std::max(1, int(get_int(value, k)));
    else if (k == "value_lr")       value_lr_       = float(get_double(value, k));
    else if (k == "epistemic_gain") epistemic_gain_ = float(get_double(value, k));
    else if (k == "temperature")    temperature_    = float(get_double(value, k));
    else if (k == "min_signal")     min_signal_     = float(get_double(value, k));
    else if (k == "credit_achieved")     { if (auto p = std::get_if<bool>(&value)) credit_achieved_ = *p; }
    else if (k == "hit_drop_thresh")hit_drop_thresh_= float(get_double(value, k));
    else if (k == "r_hit")          r_hit_          = float(get_double(value, k));
    else if (k == "allow_resector_abort"){ if (auto p = std::get_if<bool>(&value)) allow_resector_abort_ = *p; }
    else if (k == "shuffle")        { if (auto p = std::get_if<bool>(&value)) shuffle_ = *p; }
    else throw std::invalid_argument("HeadingPlanner: param '" + k + "' is construction-only / unknown");
}

void HeadingPlanner::handle_belief(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    bx_ = (bx_index_ < n) ? float(pt->values[bx_index_]) : 0.0f;
    by_ = (by_index_ < n) ? float(pt->values[by_index_]) : 0.0f;
}
void HeadingPlanner::handle_progress(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) smax_ = float(pt->values[0]);
}
void HeadingPlanner::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { yaw_ = float(pt->values[0]); have_heading_ = true; }
}

int HeadingPlanner::sector_of(float angle) const {
    int s = int(std::floor((wrap_pi(angle) + kPi) / kTwoPi * float(n_state_sectors_)));
    return std::clamp(s, 0, n_state_sectors_ - 1);
}
int HeadingPlanner::action_sector_of(float angle) const {
    int s = int(std::floor((wrap_pi(angle) + kPi) / kTwoPi * float(n_action_sectors_)));
    return std::clamp(s, 0, n_action_sectors_ - 1);
}
float HeadingPlanner::action_center(int sector) const {
    return -kPi + (float(sector) + 0.5f) * (kTwoPi / float(n_action_sectors_));
}
float HeadingPlanner::coverage() const {
    if (visits_.empty()) return 0.0f;
    int seen = 0; for (int v : visits_) if (v > 0) ++seen;
    return float(seen) / float(visits_.size());
}
float HeadingPlanner::action_bearing() const {
    if (last_asec_ < 0) return 0.0f;
    return action_center(last_asec_) / kPi;   // normalized [-1,1] (for corr vs fbear)
}

namespace { void publish_dir(Bus* bus, std::string const& topic, uint64_t tick, std::string const& pid,
                             float x, float y) {
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick; out->producer_id = pid; out->sensor = "plan.heading";
    out->values.resize(2); out->values[0] = x; out->values[1] = y;
    bus->publish(topic, out);
} }

void HeadingPlanner::tick(uint64_t tick_id) {
    std::string pid = id_.empty() ? std::string("heading_planner") : id_;
    float conf = std::sqrt(bx_ * bx_ + by_ * by_);

    // No confident belief → emit nothing (HeadingController nav-gate stops/explores);
    // abort any open window (its outcome is ambiguous without a goal).
    if (conf <= min_signal_) {
        publish_dir(bus_, output_topic_, tick_id, pid, 0.0f, 0.0f);
        ticks_left_ = 0; have_committed_ = false; last_committed_ = false; last_bsec_ = -1;
        return;
    }

    float belief_angle = std::atan2(bx_, by_);   // SAME convention as HeadingController
    int   bsec = sector_of(belief_angle);
    last_bsec_ = bsec;

    // --- mid-commit: hold the chosen heading, watch for a hit-teleport ----------
    if (ticks_left_ > 0) {
        bool abort = allow_resector_abort_ &&
                     std::abs(bsec - committed_state_) > 1 &&
                     std::abs(bsec - committed_state_) < n_state_sectors_ - 1;
        if ((window_start_smax_ - smax_) > hit_drop_thresh_) { hit_in_window_ = true; ticks_left_ = 0; }
        else if (abort) { ticks_left_ = 0; }   // re-select against a stale commit
        else {
            ticks_left_--;
            if (ticks_left_ > 0) {
                float a = action_center(committed_action_);
                publish_dir(bus_, output_topic_, tick_id, pid,
                            std::sin(a) * conf, std::cos(a) * conf);
                last_committed_ = true;
                return;   // still committed
            }
        }
    }

    // --- credit the window that just closed (if any) ---------------------------
    if (have_committed_) {
        float reward = hit_in_window_ ? r_hit_ : (smax_ - window_start_smax_);
        int credit_action = committed_action_;
        if (credit_achieved_ && have_heading_) {
            // achieved egocentric heading = how far the bug actually turned toward the
            // commanded direction.  Turning toward +right food is a CW (yaw-decreasing)
            // move, so the achieved egocentric angle = (start_yaw − end_yaw).
            credit_action = action_sector_of(wrap_pi(window_start_yaw_ - yaw_));
        }
        int idx = committed_state_ * n_action_sectors_ + credit_action;
        V_[idx]      += value_lr_ * (reward - V_[idx]);
        visits_[idx] += 1;
        last_win_dprog_ = reward;
        hit_in_window_  = false;
    }

    // --- select a new heading for this state -----------------------------------
    int row = bsec * n_action_sectors_;
    int asec;
    if (shuffle_) {                                   // ABLATION: ignore V
        std::uniform_int_distribution<int> u(0, n_action_sectors_ - 1);
        asec = u(rng_);
        last_eps_pick_ = true;
    } else {
        // score = V + epistemic bonus; softmax-sample (or argmax if temperature<=0).
        std::vector<float> score(n_action_sectors_);
        int best = 0; float bestv = -1e30f;
        for (int a = 0; a < n_action_sectors_; ++a) {
            score[a] = V_[row + a] + epistemic_gain_ / (1.0f + float(visits_[row + a]));
            if (score[a] > bestv) { bestv = score[a]; best = a; }
        }
        if (temperature_ <= 0.0f) {
            asec = best;
        } else {
            float Z = 0.0f; std::vector<float> p(n_action_sectors_);
            for (int a = 0; a < n_action_sectors_; ++a) { p[a] = std::exp((score[a] - bestv) / temperature_); Z += p[a]; }
            std::uniform_real_distribution<float> u(0.0f, Z);
            float r = u(rng_), c = 0.0f; asec = n_action_sectors_ - 1;
            for (int a = 0; a < n_action_sectors_; ++a) { c += p[a]; if (r <= c) { asec = a; break; } }
        }
        last_eps_pick_ = (asec != best);
        // value-spread diag (max−mean of this state's row) → rising = learning
        float mx = V_[row], mn = V_[row], sum = 0.0f;
        for (int a = 0; a < n_action_sectors_; ++a) { float v = V_[row + a]; mx = std::max(mx, v); mn = std::min(mn, v); sum += v; }
        last_vspread_ = mx - (sum / float(n_action_sectors_));
        last_vmax_    = mx;
    }

    committed_state_  = bsec;
    committed_action_ = asec;
    have_committed_   = true;
    ticks_left_       = commit_ticks_;
    window_start_smax_ = smax_;
    window_start_yaw_  = yaw_;
    last_asec_        = asec;
    last_committed_   = true;

    float a = action_center(asec);
    publish_dir(bus_, output_topic_, tick_id, pid, std::sin(a) * conf, std::cos(a) * conf);
}

nlohmann::json HeadingPlanner::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"V", V_}, {"visits", visits_}};
}
nlohmann::json HeadingPlanner::diag_snapshot() const {
    return nlohmann::json{
        {"bsec", last_bsec_}, {"asec", last_asec_}, {"vspread", last_vspread_},
        {"vmax", last_vmax_}, {"win_dprog", last_win_dprog_}, {"eps_pick", last_eps_pick_},
        {"coverage", coverage()}, {"committed", last_committed_},
    };
}
void HeadingPlanner::restore_state(nlohmann::json const& s) {
    if (s.contains("V"))      V_      = s.at("V").get<std::vector<float>>();
    if (s.contains("visits")) visits_ = s.at("visits").get<std::vector<int>>();
}

} // namespace ogma
