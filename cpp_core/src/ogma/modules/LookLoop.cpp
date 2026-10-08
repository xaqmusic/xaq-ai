// =============================================================================
// LookLoop.cpp  --  see the header (the duck, design doc §17.112)
// =============================================================================
#include "ogma/modules/LookLoop.hpp"

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
    throw std::invalid_argument("LookLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("LookLoop: param '" + k + "' must be a string");
}
float scalar(Bus* bus, std::string const& topic, int index, float fallback) {
    if (topic.empty()) return fallback;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus->last_value(topic));
    return (pt && pt->values.size() > index) ? pt->values[index] : fallback;
}
} // namespace

LookLoop::LookLoop()  = default;
LookLoop::~LookLoop() = default;

std::string_view LookLoop::type_name() const { return "LookLoop"; }

std::vector<TopicSpec> LookLoop::input_topics() const {
    std::vector<TopicSpec> v;
    v.push_back(TopicSpec{state_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!context_topic_.empty())      v.push_back(TopicSpec{context_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false});
    if (!active_topic_.empty())       v.push_back(TopicSpec{active_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!place_topic_.empty())        v.push_back(TopicSpec{place_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false});
    if (!seek_value_topic_.empty())   v.push_back(TopicSpec{seek_value_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!outcome_need_topic_.empty()) v.push_back(TopicSpec{outcome_need_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!skill_topic_.empty())        v.push_back(TopicSpec{skill_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> LookLoop::output_topics() const {
    std::vector<TopicSpec> v;
    v.push_back(TopicSpec{value_topic_, std::type_index(typeid(ProprioToken))});
    if (!driving_topic_.empty())    v.push_back(TopicSpec{driving_topic_, std::type_index(typeid(ProprioToken))});
    if (!descriptor_topic_.empty()) v.push_back(TopicSpec{descriptor_topic_, std::type_index(typeid(ProprioToken))});
    if (!trust_topic_.empty())      v.push_back(TopicSpec{trust_topic_, std::type_index(typeid(RealityToken))});
    if (!commit_topic_.empty())     v.push_back(TopicSpec{commit_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema LookLoop::params_schema() const {
    return {
        {"state_topic", ParamMutability::ConstructionOnly,
            "CloudMap's state_topic [open, walking, voxels, new_fraction]: a stop is a stop's cloud open (open, not walking); "
            "new_fraction is the cloud's surprise.", std::nullopt},
        {"value_topic", ParamMutability::ConstructionOnly,
            "Output [need, prediction / peak, in a stop, the last stop's realized surprise (s), its prediction (s)]; values[0] is "
            "the need the arbiter reads.", std::nullopt},
        {"context_topic", ParamMutability::ConstructionOnly,
            "A context EPM's RealityToken: its winner keys the prediction table.  Empty = one cell.", ParamValue{std::string("")}},
        {"descriptor_topic", ParamMutability::ConstructionOnly,
            "Output [place TLE / its running peak, seek need, outcome need] for the context EPM.  Empty = not published.",
            ParamValue{std::string("")}},
        {"place_topic", ParamMutability::ConstructionOnly, "The place EPM's RealityToken (its TLE: have I stood here?).", ParamValue{std::string("")}},
        {"seek_value_topic", ParamMutability::ConstructionOnly, "The seek loop's need [v].", ParamValue{std::string("")}},
        {"outcome_need_topic", ParamMutability::ConstructionOnly, "The outcome loop's need [v].", ParamValue{std::string("")}},
        {"active_topic", ParamMutability::ConstructionOnly,
            "The arbiter's gain for the look (arbiter.gain.look): a stop counts as the look's only while it is above 0.5.  "
            "Empty = passive: every stop's cloud is watched.", ParamValue{std::string("")}},
        {"driving_topic", ParamMutability::ConstructionOnly,
            "Output [1 in a stop, else 0]: a LoopCompetence's gain while the loop is passive.  Empty = not published.",
            ParamValue{std::string("")}},
        {"need_mode", ParamMutability::ConstructionOnly,
            "'surprise' (S1: the stop cloud's new_fraction integral) | 'yield' (lever 2: P(a stop pays -- a seek target appears or a "
            "skill is requested), and during a stop the probability a payoff is still to come).", ParamValue{std::string("surprise")}},
        {"skill_topic", ParamMutability::ConstructionOnly, "yield: intent.skill [id, request]; a request during a stop is a payoff.", ParamValue{std::string("")}},
        {"pending_skill_pays", ParamMutability::ConstructionOnly,
            "yield: a skill requested on the walk (deferred by the host to the next stop's hand-back) is that stop's payoff at t = 0.  "
            "false = only a request inside the stop counts (lever 2 as first built).", ParamValue{false}},
        {"per_context_times", ParamMutability::ConstructionOnly,
            "yield: the time to a payoff is learned per context (each cell's own payoff times, the all-stops distribution as a "
            "prior worth one observation).  false = one distribution over all stops (lever 2b).", ParamValue{false}},
        {"episode_start", ParamMutability::ConstructionOnly,
            "'cloud' (default): a stop is the stop's cloud being open.  'hold' (lever 3): also while the trunk is still (state_topic "
            "values[4]) or the look holds the body (hold_topic) -- the payoff window opens when the stop does.", ParamValue{std::string("cloud")}},
        {"commit_topic", ParamMutability::ConstructionOnly,
            "lever 4: output [1 while the look holds a stop it took, unpaid, and under half of this context's payoffs would have "
            "come by now (the learned median time to a payoff)]; the arbiter's look_commit_topic.  Empty = not published.",
            ParamValue{std::string("")}},
        {"hold_topic", ParamMutability::ConstructionOnly, "episode_start hold: the arbiter's gain for the look.", ParamValue{std::string("")}},
        {"trust_topic", ParamMutability::ConstructionOnly,
            "yield: a RealityToken for the voter (reality.loop.look): expected_error = the mean Brier score of the look's payoff "
            "predictions (0.25 before any).  Empty = not published.", ParamValue{std::string("")}},
        {"optimism", ParamMutability::HotMutable,
            "The prediction's credit to the unproven: mean + optimism x sd / sqrt(n) (an untried cell reads the peak).", ParamValue{1.0}},
        {"peak_decay", ParamMutability::HotMutable,
            "Per stop, the running peak of realized surprise decays this fraction toward the latest (its own scale, adapted).",
            ParamValue{0.02}},
        {"min_stop_ticks", ParamMutability::HotMutable, "A stop's cloud open fewer ticks than this is not a stop (a twitch).", ParamValue{int64_t{25}}},
        {"tick_hz", ParamMutability::ConstructionOnly, "The brain's tick rate (the integral is in seconds).", ParamValue{50.0}},
    };
}

ParamMap LookLoop::current_params() const {
    ParamMap m;
    m["state_topic"] = ParamValue{state_topic_}; m["value_topic"] = ParamValue{value_topic_};
    m["context_topic"] = ParamValue{context_topic_}; m["descriptor_topic"] = ParamValue{descriptor_topic_};
    m["place_topic"] = ParamValue{place_topic_}; m["seek_value_topic"] = ParamValue{seek_value_topic_};
    m["outcome_need_topic"] = ParamValue{outcome_need_topic_}; m["active_topic"] = ParamValue{active_topic_};
    m["driving_topic"] = ParamValue{driving_topic_};
    m["need_mode"] = ParamValue{need_mode_}; m["skill_topic"] = ParamValue{skill_topic_}; m["trust_topic"] = ParamValue{trust_topic_}; m["pending_skill_pays"] = ParamValue{pending_counts_}; m["per_context_times"] = ParamValue{per_context_times_};
    m["episode_start"] = ParamValue{episode_start_}; m["hold_topic"] = ParamValue{hold_topic_}; m["commit_topic"] = ParamValue{commit_topic_};
    m["optimism"] = ParamValue{optimism_}; m["peak_decay"] = ParamValue{peak_decay_};
    m["min_stop_ticks"] = ParamValue{int64_t(min_stop_ticks_)}; m["tick_hz"] = ParamValue{tick_hz_};
    return m;
}

void LookLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("LookLoop requires a non-null Bus");
    apply_param(params, "state_topic",        [&](auto const& v){ state_topic_ = get_string(v, "state_topic"); });
    apply_param(params, "value_topic",        [&](auto const& v){ value_topic_ = get_string(v, "value_topic"); });
    apply_param(params, "context_topic",      [&](auto const& v){ context_topic_ = get_string(v, "context_topic"); });
    apply_param(params, "descriptor_topic",   [&](auto const& v){ descriptor_topic_ = get_string(v, "descriptor_topic"); });
    apply_param(params, "place_topic",        [&](auto const& v){ place_topic_ = get_string(v, "place_topic"); });
    apply_param(params, "seek_value_topic",   [&](auto const& v){ seek_value_topic_ = get_string(v, "seek_value_topic"); });
    apply_param(params, "outcome_need_topic", [&](auto const& v){ outcome_need_topic_ = get_string(v, "outcome_need_topic"); });
    apply_param(params, "active_topic",       [&](auto const& v){ active_topic_ = get_string(v, "active_topic"); });
    apply_param(params, "driving_topic",      [&](auto const& v){ driving_topic_ = get_string(v, "driving_topic"); });
    apply_param(params, "optimism",           [&](auto const& v){ optimism_ = get_double(v, "optimism"); });
    apply_param(params, "need_mode",          [&](auto const& v){ need_mode_ = get_string(v, "need_mode"); });
    apply_param(params, "skill_topic",        [&](auto const& v){ skill_topic_ = get_string(v, "skill_topic"); });
    apply_param(params, "trust_topic",        [&](auto const& v){ trust_topic_ = get_string(v, "trust_topic"); });
    apply_param(params, "episode_start",      [&](auto const& v){ episode_start_ = get_string(v, "episode_start"); });
    apply_param(params, "hold_topic",         [&](auto const& v){ hold_topic_ = get_string(v, "hold_topic"); });
    apply_param(params, "commit_topic",       [&](auto const& v){ commit_topic_ = get_string(v, "commit_topic"); });
    apply_param(params, "per_context_times", [&](auto const& v){
        if (auto b = std::get_if<bool>(&v)) per_context_times_ = *b; else per_context_times_ = get_double(v, "per_context_times") > 0.5; });
    apply_param(params, "pending_skill_pays", [&](auto const& v){
        if (auto b = std::get_if<bool>(&v)) pending_counts_ = *b; else pending_counts_ = get_double(v, "pending_skill_pays") > 0.5; });
    apply_param(params, "peak_decay",         [&](auto const& v){ peak_decay_ = get_double(v, "peak_decay"); });
    apply_param(params, "min_stop_ticks",     [&](auto const& v){ min_stop_ticks_ = std::max(1, int(get_double(v, "min_stop_ticks"))); });
    apply_param(params, "tick_hz",            [&](auto const& v){ tick_hz_ = std::max(1.0, get_double(v, "tick_hz")); });
    if (state_topic_.empty()) throw std::invalid_argument("LookLoop: state_topic is required");
    if (value_topic_.empty()) throw std::invalid_argument("LookLoop: value_topic is required");
    if (need_mode_ != "surprise" && need_mode_ != "yield") throw std::invalid_argument("LookLoop: need_mode is 'surprise' or 'yield'");
    if (episode_start_ != "cloud" && episode_start_ != "hold") throw std::invalid_argument("LookLoop: episode_start is 'cloud' or 'hold'");
}

void LookLoop::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "optimism")       optimism_ = get_double(value, k);
    else if (k == "peak_decay")     peak_decay_ = get_double(value, k);
    else if (k == "min_stop_ticks") min_stop_ticks_ = std::max(1, int(get_double(value, k)));
    else throw std::invalid_argument("LookLoop: param '" + k + "' is construction-only / unknown");
}

int LookLoop::context_now() const {
    if (context_topic_.empty()) return 0;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(context_topic_));
    return rt ? std::max(0, rt->winner_id) : 0;
}

double LookLoop::predict(int cell) const {
    auto it = cells_.find(cell);
    if (it == cells_.end() || it->second.n == 0) return peak_;                       // untried: the best seen
    const Cell& c = it->second;
    const double sd = c.n >= 2 ? std::sqrt(c.m2 / double(c.n - 1)) : peak_;
    return c.mean + optimism_ * sd / std::sqrt(double(c.n));
}

double LookLoop::pay_mean(int cell) const {
    auto it = cells_.find(cell);
    const double a = it == cells_.end() ? 1.0 : it->second.a, b = it == cells_.end() ? 1.0 : it->second.b;
    return a / (a + b);
}

double LookLoop::pay_optimistic(int cell) const {
    auto it = cells_.find(cell);
    const double a = it == cells_.end() ? 1.0 : it->second.a, b = it == cells_.end() ? 1.0 : it->second.b, n = a + b;
    return std::clamp(a / n + optimism_ * std::sqrt(a * b / (n * n * (n + 1.0))), 0.0, 1.0);
}

void LookLoop::record_pay(int cell, double secs) {
    pay_times_.push_back(secs); if (pay_times_.size() > 400) pay_times_.pop_front();
    auto& ct = cell_times_[cell]; ct.push_back(secs); if (ct.size() > 200) ct.pop_front();
}

double LookLoop::still_to_come(double p, double tau, int cell) const {
    // P(a payoff is still to come | none in the first tau s) = P S(tau) / (1 - P + P S(tau))
    const double surv = survival(tau, cell);
    const double num = p * surv, den = 1.0 - p + num;
    return den > 1e-12 ? num / den : 0.0;
}

double LookLoop::survival(double tau, int cell) const {
    double surv = 1.0;
    if (!pay_times_.empty()) {
        const bool hold = episode_start_ == "hold";        // hold: a payoff AT tau is still to come (continuity at t = 0)
        int later = 0; for (double x : pay_times_) later += hold ? x >= tau : x > tau;
        surv = double(later) / double(pay_times_.size());
    }
    if (per_context_times_) {
        // this context's own times to a payoff, the all-stops survival as a prior worth one observation
        auto it = cell_times_.find(cell);
        if (it != cell_times_.end() && !it->second.empty()) {
            int later = 0; for (double x : it->second) later += episode_start_ == "hold" ? x >= tau : x > tau;
            surv = (double(later) + surv) / (double(it->second.size()) + 1.0);
        }
    }
    return surv;
}

void LookLoop::tick(uint64_t tick_id) {
    const bool open    = scalar(bus_, state_topic_, 0, 0.0f) > 0.5f;
    const bool walking = scalar(bus_, state_topic_, 1, 0.0f) > 0.5f;
    nf_ = std::clamp(double(scalar(bus_, state_topic_, 3, 0.0f)), 0.0, 1.0);
    const bool active = active_topic_.empty() || scalar(bus_, active_topic_, 0, 0.0f) > 0.5f;
    bool in_now = open && !walking && active;
    if (episode_start_ == "hold") {
        // lever 3: a stop starts when the body does -- the look's own stop when it takes the body (the arbiter's gain), any
        // stop when the trunk is still (the stillness the cloud's input carries) -- not when the stop's cloud opens
        const bool still = scalar(bus_, state_topic_, 4, 0.0f) > 0.5f;
        const bool held  = !hold_topic_.empty() && scalar(bus_, hold_topic_, 0, 0.0f) > 0.5f;
        in_now = in_now || still || held;
    }

    // the context: the descriptor for the context EPM (published every tick), and the winner it last returned
    if (!descriptor_topic_.empty()) {
        float place = 0.0f;
        if (!place_topic_.empty())
            if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(place_topic_))) {
                place_peak_ = std::max(double(rt->tle), place_peak_ * (1.0 - 1e-4));
                place = place_peak_ > 0.0 ? float(std::clamp(double(rt->tle) / place_peak_, 0.0, 1.0)) : 0.0f;
            }
        auto d = std::make_shared<ProprioToken>();
        d->tick_id = tick_id; d->producer_id = id_.empty() ? std::string("look") : id_; d->sensor = "look_context";
        d->values = Eigen::VectorXf(3);
        d->values[0] = place;
        d->values[1] = std::clamp(scalar(bus_, seek_value_topic_, 0, 0.0f), 0.0f, 1.0f);
        d->values[2] = std::clamp(scalar(bus_, outcome_need_topic_, 0, 0.0f), 0.0f, 1.0f);
        bus_->publish(descriptor_topic_, d);
    }
    const int cell = context_now();

    if (in_now && !in_ep_) {                                   // a stop's cloud opens: the prediction is made now
        in_ep_ = true; ep_ticks_ = 0; ep_gain_ = 0.0; ep_nf_peak_ = 0.0; ep_cell_ = cell; ep_pred_ = predict(cell);
    }
    if (in_ep_ && in_now) {
        ++ep_ticks_; ep_gain_ += nf_ / tick_hz_; ep_nf_peak_ = std::max(ep_nf_peak_, nf_);
    }
    if (in_ep_ && !in_now) {                                   // the stop ended: the honest signal
        in_ep_ = false;
        if (ep_ticks_ >= min_stop_ticks_) {
            Cell& c = cells_[ep_cell_];
            ++c.n; const double d = ep_gain_ - c.mean; c.mean += d / double(c.n); c.m2 += d * (ep_gain_ - c.mean);
            peak_ = peak_ <= 0.0 ? ep_gain_ : std::max(ep_gain_, peak_ + peak_decay_ * (ep_gain_ - peak_));
            last_realized_ = ep_gain_; last_predicted_ = ep_pred_;
            abs_err_sum_ += std::fabs(ep_gain_ - ep_pred_);
            ++episodes_;
        }
    }

    if (need_mode_ == "yield") {
        // the payoff: the seek need rising from 0, or a skill requested, during the stop
        const double sv = scalar(bus_, seek_value_topic_, 0, 0.0f);
        bool skill = false;
        if (!skill_topic_.empty())
            if (auto sk = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(skill_topic_)))
                if (sk->tick_id != skill_seen_tick_) {                 // a token not seen before (the outcome loop may run
                    skill_seen_tick_ = sk->tick_id;                    // after this module: its request is a tick old here)
                    skill = sk->values.size() >= 2 && sk->values[1] > 0.5f;
                }
        // a skill requested on the walk is deferred by the host to the next stop's hand-back (it fires from standing, before
        // the stop's cloud opens): that stop is what serves it -- its payoff at t = 0 (2026-10-08: without this the look
        // never saw an arrival stop's skill, 304 of 306)
        if (skill && !in_ep_ && pending_counts_) skill_pending_ = true;
        if (in_ep_ && ep_ticks_ == 1) {
            ep_paid_ = false; ep_p_ = pay_mean(ep_cell_);
            if (skill_pending_) {
                ep_paid_ = true; skill_pending_ = false;
                // hold: a deferred skill is a certain payoff, served at once -- not a time the look waits for
                if (episode_start_ != "hold") record_pay(ep_cell_, 0.0);
            }
        }
        if (in_ep_ && !ep_paid_ && ((sv > 0.0 && seek_prev_ <= 0.0) || skill)) {
            ep_paid_ = true;
            record_pay(ep_cell_, double(ep_ticks_) / tick_hz_);
        }
        seek_prev_ = sv;
        if (!in_ep_ && ep_ticks_ > 0) {                                   // the stop that just ended: its outcome
            if (ep_ticks_ >= min_stop_ticks_) {
                Cell& c = cells_[ep_cell_];
                if (ep_paid_) { c.a += 1.0; ++paid_; } else c.b += 1.0;
                const double y = ep_paid_ ? 1.0 : 0.0;
                brier_sum_ += (y - ep_p_) * (y - ep_p_);
                last_realized_ = y; last_predicted_ = ep_p_;
            }
            ep_ticks_ = 0;
        }
        pred_norm_ = pay_optimistic(cell);
        if (!in_ep_) need_ = pred_norm_;
        else need_ = ep_paid_ ? 0.0 : still_to_come(pay_optimistic(ep_cell_), double(ep_ticks_) / tick_hz_, ep_cell_);
        if (!commit_topic_.empty()) {
            // lever 4: the look commits to a stop it took while under half of this context's payoffs would have come by now
            const bool held = !hold_topic_.empty() && scalar(bus_, hold_topic_, 0, 0.0f) > 0.5f;
            commit_ = in_ep_ && held && !ep_paid_ && survival(double(ep_ticks_) / tick_hz_, ep_cell_) >= 0.5;
            auto cm = std::make_shared<ProprioToken>();
            cm->tick_id = tick_id; cm->producer_id = id_.empty() ? std::string("look") : id_; cm->sensor = "look_commit";
            cm->values = Eigen::VectorXf::Constant(1, commit_ ? 1.0f : 0.0f);
            bus_->publish(commit_topic_, cm);
        }
        if (!trust_topic_.empty()) {
            const double err = episodes_ ? brier_sum_ / double(episodes_) : 0.25;
            auto tok = std::make_shared<RealityToken>();
            tok->tick_id = tick_id; tok->producer_id = id_.empty() ? std::string("look") : id_;
            tok->winner_id = 0; tok->latent = Eigen::VectorXf::Constant(1, float(1.0 - err)); tok->winner_prototype = tok->latent;
            tok->quant_error = float(err); tok->expected_error = float(err); tok->tle = float(err); tok->transition_surp = 0.0f;
            tok->node_count = 1; tok->baked_count = 3;
            bus_->publish(trust_topic_, tok);
        }
    } else {
        pred_norm_ = peak_ > 0.0 ? std::clamp(predict(cell) / peak_, 0.0, 1.0) : 1.0;   // nothing seen yet: untried
        need_ = (in_ep_ && ep_nf_peak_ > 0.0) ? std::clamp(nf_ / ep_nf_peak_, 0.0, 1.0) : pred_norm_;
    }

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("look") : id_; out->sensor = "look";
    out->values = Eigen::VectorXf(5);
    out->values[0] = float(need_); out->values[1] = float(pred_norm_); out->values[2] = in_ep_ ? 1.0f : 0.0f;
    out->values[3] = float(last_realized_); out->values[4] = float(last_predicted_);
    bus_->publish(value_topic_, out);
    if (!driving_topic_.empty()) {
        auto g = std::make_shared<ProprioToken>();
        g->tick_id = tick_id; g->producer_id = out->producer_id; g->sensor = "look_driving";
        g->values = Eigen::VectorXf::Constant(1, in_ep_ ? 1.0f : 0.0f);
        bus_->publish(driving_topic_, g);
    }
}

nlohmann::json LookLoop::cell_times_json() const {
    nlohmann::json j = nlohmann::json::array();
    for (auto const& [k, v] : cell_times_) j.push_back({k, std::vector<double>(v.begin(), v.end())});
    return j;
}

nlohmann::json LookLoop::snapshot_state() const {
    nlohmann::json cells = nlohmann::json::array();
    for (auto const& [k, c] : cells_) cells.push_back({k, c.n, c.mean, c.m2});
    nlohmann::json pay = nlohmann::json::array();
    for (auto const& [k, c] : cells_) pay.push_back({k, c.a, c.b});
    return nlohmann::json{{"version", 1}, {"cells", cells}, {"peak", peak_}, {"place_peak", place_peak_},
                          {"episodes", episodes_}, {"abs_err_sum", abs_err_sum_},
                          {"pay", pay}, {"pay_times", std::vector<double>(pay_times_.begin(), pay_times_.end())},
                          {"brier_sum", brier_sum_}, {"paid", paid_}, {"cell_times", cell_times_json()}};
}

void LookLoop::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    cells_.clear();
    for (auto const& c : s.value("cells", nlohmann::json::array()))
        if (c.is_array() && c.size() == 4) cells_[c[0].get<int>()] = Cell{c[1].get<int>(), c[2].get<double>(), c[3].get<double>()};
    peak_ = s.value("peak", 0.0); place_peak_ = s.value("place_peak", 0.0);
    episodes_ = s.value("episodes", uint64_t{0}); abs_err_sum_ = s.value("abs_err_sum", 0.0);
    for (auto const& c : s.value("pay", nlohmann::json::array()))
        if (c.is_array() && c.size() == 3) { Cell& x = cells_[c[0].get<int>()]; x.a = c[1].get<double>(); x.b = c[2].get<double>(); }
    pay_times_.clear(); for (double x : s.value("pay_times", std::vector<double>{})) pay_times_.push_back(x);
    brier_sum_ = s.value("brier_sum", 0.0); paid_ = s.value("paid", uint64_t{0});
    cell_times_.clear();
    for (auto const& c : s.value("cell_times", nlohmann::json::array()))
        if (c.is_array() && c.size() == 2) for (double x : c[1].get<std::vector<double>>()) cell_times_[c[0].get<int>()].push_back(x);
    in_ep_ = false;
}

nlohmann::json LookLoop::diag_snapshot() const {
    nlohmann::json cells = nlohmann::json::object();
    for (auto const& [k, c] : cells_) cells[std::to_string(k)] = {{"n", c.n}, {"mean", c.mean}, {"paid_a", c.a}, {"unpaid_b", c.b}};
    return nlohmann::json{{"need", need_}, {"prediction", pred_norm_}, {"in_stop", in_ep_}, {"new_fraction", nf_},
                          {"peak", peak_}, {"episodes", episodes_}, {"last_realized", last_realized_},
                          {"last_predicted", last_predicted_},
                          {"mean_abs_error", episodes_ ? abs_err_sum_ / double(episodes_) : 0.0}, {"cells", cells},
                          {"need_mode", need_mode_}, {"paid", paid_}, {"brier", episodes_ ? brier_sum_ / double(episodes_) : 0.25},
                          {"in_stop_paid", ep_paid_}, {"commit", commit_}};
}

} // namespace ogma
