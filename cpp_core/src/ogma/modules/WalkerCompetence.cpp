// =============================================================================
// WalkerCompetence.cpp  --  see the header (learned stepper plan 9.6 item 2, 2026-11-19)
// =============================================================================
#include "ogma/modules/WalkerCompetence.hpp"

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
    throw std::invalid_argument("WalkerCompetence: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("WalkerCompetence: param '" + k + "' must be a string");
}
} // namespace

WalkerCompetence::WalkerCompetence()  = default;
WalkerCompetence::~WalkerCompetence() = default;

std::string_view WalkerCompetence::type_name() const { return "WalkerCompetence"; }

std::vector<TopicSpec> WalkerCompetence::input_topics() const {
    std::vector<TopicSpec> v;
    const auto pt = std::type_index(typeid(ProprioToken));
    v.push_back(TopicSpec{stride_topic_, pt, SubscriptionKind::Direct, false});
    if (!stuck_topic_.empty()) v.push_back(TopicSpec{stuck_topic_, pt, SubscriptionKind::Direct, false});
    if (!share_topic_.empty()) v.push_back(TopicSpec{share_topic_, pt, SubscriptionKind::Direct, false});
    if (!speed_topic_.empty()) v.push_back(TopicSpec{speed_topic_, pt, SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> WalkerCompetence::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema WalkerCompetence::params_schema() const {
    return {
        {"stride_topic", ParamMutability::ConstructionOnly, "ProprioToken: the stride odometry's velocity (m/s); stride_index is the forward component.", ParamValue{std::string("reality.proprio.stride_v")}},
        {"stride_index", ParamMutability::ConstructionOnly, "Index of the forward speed in stride_topic.", ParamValue{int64_t{1}}},
        {"stuck_topic",  ParamMutability::ConstructionOnly, "StuckSense's output; the value at stuck_index (its range channel) discounts the stride's claim. \"\" = no discount.", ParamValue{std::string("")}},
        {"stuck_index",  ParamMutability::ConstructionOnly, "Index in stuck_topic (4 = the range channel alone, smoothed).", ParamValue{int64_t{4}}},
        {"share_topic",  ParamMutability::ConstructionOnly, "ProprioToken carrying a walker's share of the command at share_index (the pose socket: pose.targets, value 0 = the sequencer's). \"\" = this walker always drives.", ParamValue{std::string("")}},
        {"share_index",  ParamMutability::ConstructionOnly, "Index of the share in share_topic.", ParamValue{int64_t{0}}},
        {"share_invert", ParamMutability::ConstructionOnly, "true: this walker's share is 1 - the value (the rhythm, which owns what the pose socket does not).", ParamValue{false}},
        {"speed_topic",  ParamMutability::ConstructionOnly, "ProprioToken [speed]: the forward command; no grading below cmd_min. \"\" = always commanding.", ParamValue{std::string("")}},
        {"output_topic", ParamMutability::ConstructionOnly, "ProprioToken [est, sd, graded, rate, mean, n, switch_cost_s, p_same, settling, idle_m].", std::nullopt},
        {"drive_min",    ParamMutability::HotMutable, "The walker holds the legs when its share is at least this.", ParamValue{0.99}},
        {"cmd_min",      ParamMutability::HotMutable, "No grading while the forward command is below this.", ParamValue{0.05}},
        {"tau_s",        ParamMutability::HotMutable, "The rate's EMA, in graded seconds (the walker's own cadence).", ParamValue{2.0}},
        {"mean_tau_s",   ParamMutability::HotMutable, "The memory of the walker's long-run mean and variance, in graded seconds.", ParamValue{600.0}},
        {"relax_m",      ParamMutability::HotMutable, "SCAFFOLD. While idle the estimate returns to the walker's mean over this many metres of the body's travel (0 = travel changes nothing).", ParamValue{0.25}},
        {"relax_s",      ParamMutability::HotMutable, "SCAFFOLD. While idle the estimate returns to the walker's mean over this many seconds (0 = time changes nothing).", ParamValue{0.0}},
        {"settle_cap_s", ParamMutability::HotMutable, "A take-over that has not reached half the walker's mean rate after this long is no longer a settle.", ParamValue{30.0}},
        {"hz",           ParamMutability::ConstructionOnly, "Tick rate.", ParamValue{50.0}},
    };
}

ParamMap WalkerCompetence::current_params() const {
    ParamMap m;
    m["stride_topic"] = ParamValue{stride_topic_}; m["stride_index"] = ParamValue{int64_t(stride_index_)};
    m["stuck_topic"] = ParamValue{stuck_topic_};   m["stuck_index"] = ParamValue{int64_t(stuck_index_)};
    m["share_topic"] = ParamValue{share_topic_};   m["share_index"] = ParamValue{int64_t(share_index_)};
    m["share_invert"] = ParamValue{share_invert_}; m["speed_topic"] = ParamValue{speed_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["drive_min"] = ParamValue{drive_min_}; m["cmd_min"] = ParamValue{cmd_min_}; m["tau_s"] = ParamValue{tau_s_};
    m["mean_tau_s"] = ParamValue{mean_tau_s_}; m["relax_m"] = ParamValue{relax_m_}; m["relax_s"] = ParamValue{relax_s_};
    m["settle_cap_s"] = ParamValue{settle_cap_s_}; m["hz"] = ParamValue{hz_};
    return m;
}

void WalkerCompetence::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("WalkerCompetence requires a non-null Bus");
    apply_param(params, "stride_topic", [&](auto const& v){ stride_topic_ = get_string(v, "stride_topic"); });
    apply_param(params, "stride_index", [&](auto const& v){ stride_index_ = int(get_double(v, "stride_index")); });
    apply_param(params, "stuck_topic",  [&](auto const& v){ stuck_topic_  = get_string(v, "stuck_topic"); });
    apply_param(params, "stuck_index",  [&](auto const& v){ stuck_index_  = int(get_double(v, "stuck_index")); });
    apply_param(params, "share_topic",  [&](auto const& v){ share_topic_  = get_string(v, "share_topic"); });
    apply_param(params, "share_index",  [&](auto const& v){ share_index_  = int(get_double(v, "share_index")); });
    apply_param(params, "share_invert", [&](auto const& v){ share_invert_ = get_double(v, "share_invert") != 0.0; });
    apply_param(params, "speed_topic",  [&](auto const& v){ speed_topic_  = get_string(v, "speed_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "hz",           [&](auto const& v){ hz_ = std::max(1.0, get_double(v, "hz")); });
    for (auto const& k : {"drive_min", "cmd_min", "tau_s", "mean_tau_s", "relax_m", "relax_s", "settle_cap_s"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    if (stride_topic_.empty()) throw std::invalid_argument("WalkerCompetence: stride_topic is required");
    if (output_topic_.empty()) throw std::invalid_argument("WalkerCompetence: output_topic is required");
    if (stride_index_ < 0 || stuck_index_ < 0 || share_index_ < 0) throw std::invalid_argument("WalkerCompetence: an index is negative");

    auto scalar = [this](std::string const& topic, int index, double* dst, bool* have) {
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct,
            [this, index, dst, have](std::string_view, MessagePtr p){
                if (!input_allowed(p->producer_id)) return;
                auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
                if (!pt || index >= pt->values.size()) return;
                *dst = double(pt->values[index]); if (have) *have = true;
            }));
    };
    scalar(stride_topic_, stride_index_, &vfwd_, nullptr);
    if (!stuck_topic_.empty()) scalar(stuck_topic_, stuck_index_, &stuck_, nullptr);
    if (!share_topic_.empty()) scalar(share_topic_, share_index_, &share_, &have_share_);
    if (!speed_topic_.empty()) scalar(speed_topic_, 0, &speed_, &have_speed_);
}

void WalkerCompetence::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "drive_min")    drive_min_    = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "cmd_min")      cmd_min_      = get_double(value, k);
    else if (k == "tau_s")        tau_s_        = std::max(1.0 / hz_, get_double(value, k));
    else if (k == "mean_tau_s")   mean_tau_s_   = std::max(1.0, get_double(value, k));
    else if (k == "relax_m")      relax_m_      = std::max(0.0, get_double(value, k));
    else if (k == "relax_s")      relax_s_      = std::max(0.0, get_double(value, k));
    else if (k == "settle_cap_s") settle_cap_s_ = std::max(0.0, get_double(value, k));
    else throw std::invalid_argument("WalkerCompetence: param '" + k + "' is construction-only / unknown");
}

void WalkerCompetence::tick(uint64_t tick_id) {
    const double dt = 1.0 / hz_;
    // this walker's share of the command.  No word from the socket yet = the rhythm has the legs.
    double own = 1.0;
    if (!share_topic_.empty()) { const double s = have_share_ ? std::clamp(share_, 0.0, 1.0) : 0.0; own = share_invert_ ? 1.0 - s : s; }
    const bool drive_now  = own >= drive_min_;
    const bool commanding = speed_topic_.empty() || !have_speed_ || speed_ >= cmd_min_;

    if (drive_now) {
        if (!driving_) {                       // the take-over
            ++takeovers_; since_ = 0; live_ = 0.0;
            handed_est_ = est_; handed_sd_ = sd_;
            settling_ = n_ > 0 && mean_ > 0.0; // a walker with no history has nothing to settle against
        }
        if (commanding) {
            const double realised = vfwd_ * (1.0 - std::clamp(stuck_, 0.0, 1.0));
            const double a = std::min(1.0, dt / tau_s_);
            ++since_;
            if (settling_) {
                live_ += a * (realised - live_);
                if (live_ >= 0.5 * mean_) {    // at half its own pace: the settle is over, and it took this long
                    const double c = double(since_) * dt;
                    ++cost_n_; cost_s_ += (c - cost_s_) / double(std::min<int64_t>(cost_n_, 10));
                    settling_ = false; rate_ = live_;
                } else if (double(since_) * dt >= settle_cap_s_) {
                    settling_ = false; rate_ = live_;   // it did not get going: the live rate is the report
                }
            } else {
                rate_ += a * (realised - rate_);
                ++n_;
                const double am = 1.0 / std::min(double(n_), mean_tau_s_ * hz_);
                const double d = rate_ - mean_;
                mean_ += am * d;
                var_  += am * (d * (rate_ - mean_) - var_);
            }
        }
        if (settling_) { est_ = handed_est_; sd_ = handed_sd_; }
        else           { est_ = std::max(0.0, rate_); sd_ = 0.0; }
        p_same_ = 1.0; idle_net_ = 0.0; idle_m_ = 0.0; idle_ticks_ = 0;
    } else {
        if (driving_) {                        // the hand-off: what it knew of this ground when it let go
            hold_ = settling_ ? handed_est_ : std::max(0.0, rate_);   // handed away mid-settle: nothing was learned
            settling_ = false; idle_net_ = 0.0; idle_m_ = 0.0; idle_ticks_ = 0;
        }
        ++idle_ticks_; idle_net_ += vfwd_ * dt; idle_m_ = std::fabs(idle_net_);   // net travel: rocking in place is the same ground
        double e = 0.0;
        if (relax_m_ > 0.0) e += idle_m_ / relax_m_;
        if (relax_s_ > 0.0) e += double(idle_ticks_) * dt / relax_s_;
        p_same_ = std::exp(-e);
        if (n_ == 0) { est_ = 0.0; sd_ = 0.0; }   // never driven: the count says so, and a selector tries it
        else {
            est_ = std::max(0.0, p_same_ * hold_ + (1.0 - p_same_) * mean_);
            sd_  = (1.0 - p_same_) * std::sqrt(std::max(0.0, var_));
        }
    }
    driving_ = drive_now;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? output_topic_ : id_; out->sensor = "walker_competence";
    out->values = Eigen::VectorXf(10);
    out->values[0] = float(est_); out->values[1] = float(sd_); out->values[2] = (driving_ && !settling_) ? 1.0f : 0.0f;
    out->values[3] = float(rate_); out->values[4] = float(mean_); out->values[5] = float(n_);
    out->values[6] = float(cost_s_); out->values[7] = float(p_same_); out->values[8] = settling_ ? 1.0f : 0.0f;
    out->values[9] = float(idle_m_);
    bus_->publish(output_topic_, out);
}

nlohmann::json WalkerCompetence::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"rate", rate_}, {"mean", mean_}, {"var", var_}, {"n", n_}, {"driving", driving_},
                          {"settling", settling_}, {"live", live_}, {"since", since_}, {"handed_est", handed_est_}, {"handed_sd", handed_sd_},
                          {"cost_s", cost_s_}, {"cost_n", cost_n_}, {"takeovers", takeovers_}, {"hold", hold_}, {"idle_m", idle_m_}, {"idle_net", idle_net_},
                          {"idle_ticks", idle_ticks_}, {"p_same", p_same_}, {"est", est_}, {"sd", sd_}};
}
void WalkerCompetence::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    rate_ = s.value("rate", 0.0); mean_ = s.value("mean", 0.0); var_ = s.value("var", 0.0); n_ = s.value("n", int64_t{0});
    driving_ = s.value("driving", false); settling_ = s.value("settling", false); live_ = s.value("live", 0.0); since_ = s.value("since", int64_t{0});
    handed_est_ = s.value("handed_est", 0.0); handed_sd_ = s.value("handed_sd", 0.0);
    cost_s_ = s.value("cost_s", 0.0); cost_n_ = s.value("cost_n", int64_t{0}); takeovers_ = s.value("takeovers", int64_t{0});
    hold_ = s.value("hold", 0.0); idle_m_ = s.value("idle_m", 0.0); idle_net_ = s.value("idle_net", idle_m_); idle_ticks_ = s.value("idle_ticks", int64_t{0});
    p_same_ = s.value("p_same", 1.0); est_ = s.value("est", 0.0); sd_ = s.value("sd", 0.0);
}
nlohmann::json WalkerCompetence::diag_snapshot() const {
    return nlohmann::json{{"est", est_}, {"sd", sd_}, {"graded", driving_ && !settling_}, {"driving", driving_}, {"settling", settling_},
                          {"rate", rate_}, {"mean", mean_}, {"var", var_}, {"n", n_}, {"switch_cost_s", cost_s_}, {"costs", cost_n_},
                          {"takeovers", takeovers_}, {"p_same", p_same_}, {"idle_m", idle_m_}, {"hold", hold_}, {"live", live_},
                          {"vfwd", vfwd_}, {"stuck", stuck_}, {"share", share_}};
}

} // namespace ogma
