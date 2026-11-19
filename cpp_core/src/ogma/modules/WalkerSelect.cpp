// =============================================================================
// WalkerSelect.cpp  --  see the header (learned stepper plan 9.6 item 2, 2026-11-19)
// =============================================================================
#include "ogma/modules/WalkerSelect.hpp"

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
    throw std::invalid_argument("WalkerSelect: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("WalkerSelect: param '" + k + "' must be a string");
}
} // namespace

WalkerSelect::WalkerSelect()  = default;
WalkerSelect::~WalkerSelect() = default;

std::string_view WalkerSelect::type_name() const { return "WalkerSelect"; }

std::vector<TopicSpec> WalkerSelect::input_topics() const {
    const auto pt = std::type_index(typeid(ProprioToken));
    std::vector<TopicSpec> v;
    for (auto const& t : report_topics_) if (!t.empty()) v.push_back(TopicSpec{t, pt, SubscriptionKind::Direct, false});
    if (!share_topic_.empty()) v.push_back(TopicSpec{share_topic_, pt, SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> WalkerSelect::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema WalkerSelect::params_schema() const {
    return {
        {"rhythm_report_topic",    ParamMutability::ConstructionOnly, "The rhythm walker's WalkerCompetence output.", std::nullopt},
        {"sequencer_report_topic", ParamMutability::ConstructionOnly, "The pose sequencer's WalkerCompetence output.", std::nullopt},
        {"share_topic",  ParamMutability::ConstructionOnly, "ProprioToken: the sequencer's share of the command at share_index (the pose socket); it has the legs at >= 0.5.", ParamValue{std::string("pose.targets")}},
        {"share_index",  ParamMutability::ConstructionOnly, "Index of the share in share_topic.", ParamValue{int64_t{0}}},
        {"output_topic", ParamMutability::ConstructionOnly, "ProprioToken [want, lead, lead_s, G_rhythm, G_sequencer, incumbent, agree]. Nothing consumes it: the selection is in shadow.", ParamValue{std::string("select.walker")}},
        {"optimism",     ParamMutability::HotMutable, "G = est + optimism x sd: what is not known about a walker's progress counts for it.", ParamValue{1.0}},
        {"horizon_s",    ParamMutability::HotMutable, "SCAFFOLD. The stretch a hand-over is priced against: the challenger's value is scaled by 1 - its switching cost / horizon_s.", ParamValue{30.0}},
        {"dwell_min_s",  ParamMutability::HotMutable, "The challenger must lead for as long as the hand-over to it takes (its measured switching cost), and at least this long.", ParamValue{2.0}},
        {"hz",           ParamMutability::ConstructionOnly, "Tick rate.", ParamValue{50.0}},
    };
}

ParamMap WalkerSelect::current_params() const {
    ParamMap m;
    m["rhythm_report_topic"] = ParamValue{report_topics_[0]}; m["sequencer_report_topic"] = ParamValue{report_topics_[1]};
    m["share_topic"] = ParamValue{share_topic_}; m["share_index"] = ParamValue{int64_t(share_index_)}; m["output_topic"] = ParamValue{output_topic_};
    m["optimism"] = ParamValue{optimism_}; m["horizon_s"] = ParamValue{horizon_s_}; m["dwell_min_s"] = ParamValue{dwell_min_s_};
    m["hz"] = ParamValue{hz_};
    return m;
}

void WalkerSelect::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("WalkerSelect requires a non-null Bus");
    apply_param(params, "rhythm_report_topic",    [&](auto const& v){ report_topics_[0] = get_string(v, "rhythm_report_topic"); });
    apply_param(params, "sequencer_report_topic", [&](auto const& v){ report_topics_[1] = get_string(v, "sequencer_report_topic"); });
    apply_param(params, "share_topic",  [&](auto const& v){ share_topic_  = get_string(v, "share_topic"); });
    apply_param(params, "share_index",  [&](auto const& v){ share_index_  = int(get_double(v, "share_index")); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "hz",           [&](auto const& v){ hz_ = std::max(1.0, get_double(v, "hz")); });
    for (auto const& k : {"optimism", "horizon_s", "dwell_min_s"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    if (report_topics_[0].empty() || report_topics_[1].empty()) throw std::invalid_argument("WalkerSelect: both report topics are required");
    if (share_topic_.empty() || share_index_ < 0) throw std::invalid_argument("WalkerSelect: share_topic is required");
    if (output_topic_.empty()) throw std::invalid_argument("WalkerSelect: output_topic is required");

    for (int i = 0; i < 2; ++i)
        sub_ids_.push_back(bus_->subscribe(report_topics_[size_t(i)], SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){
                if (!input_allowed(p->producer_id)) return;
                auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
                if (!pt || pt->values.size() < 7) return;
                Report& r = r_[size_t(i)];
                r.est = double(pt->values[0]); r.sd = double(pt->values[1]); r.n = double(pt->values[5]); r.cost = double(pt->values[6]); r.have = true;
            }));
    sub_ids_.push_back(bus_->subscribe(share_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){
            if (!input_allowed(p->producer_id)) return;
            auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (!pt || share_index_ >= pt->values.size()) return;
            share_ = double(pt->values[share_index_]); have_share_ = true;
        }));
}

void WalkerSelect::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "optimism")     optimism_     = std::max(0.0, get_double(value, k));
    else if (k == "horizon_s")    horizon_s_    = std::max(1e-3, get_double(value, k));
    else if (k == "dwell_min_s")  dwell_min_s_  = std::max(0.0, get_double(value, k));
    else throw std::invalid_argument("WalkerSelect: param '" + k + "' is construction-only / unknown");
}

void WalkerSelect::tick(uint64_t tick_id) {
    cur_ = (have_share_ && share_ >= 0.5) ? 1 : 0;
    if (ticks_ > 0 && cur_ != prev_cur_) { lead_ticks_ = 0; calling_ = false; }   // a hand-over: the count starts afresh
    const int j = 1 - cur_;
    for (int i = 0; i < 2; ++i) g_[size_t(i)] = r_[size_t(i)].est + optimism_ * r_[size_t(i)].sd;
    const Report& rc = r_[size_t(cur_)]; const Report& rj = r_[size_t(j)];
    int want = cur_;
    if (rc.have && rj.have) {
        const double value_j = g_[size_t(j)] * std::max(0.0, 1.0 - rj.cost / horizon_s_);
        lead_ = value_j - g_[size_t(cur_)];
        const bool leading = rc.n > 0.0 && (rj.n <= 0.0 || lead_ > 0.0);   // a walker never driven leads once the incumbent has a grade
        lead_ticks_ = leading ? lead_ticks_ + 1 : 0;
        if (double(lead_ticks_) >= std::max(dwell_min_s_, rj.cost) * hz_) want = j;
    } else lead_ticks_ = 0;
    const bool calling = want != cur_;
    if (calling && !calling_) ++calls_;                         // a new call for the other walker
    calling_ = calling; want_ = want; prev_cur_ = cur_;
    ++ticks_; ++want_ticks_[size_t(want_)]; if (!calling) ++agree_;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? output_topic_ : id_; out->sensor = "walker_select";
    out->values = Eigen::VectorXf(7);
    out->values[0] = float(want_); out->values[1] = float(lead_); out->values[2] = float(double(lead_ticks_) / hz_);
    out->values[3] = float(g_[0]); out->values[4] = float(g_[1]); out->values[5] = float(cur_); out->values[6] = calling ? 0.0f : 1.0f;
    bus_->publish(output_topic_, out);
}

nlohmann::json WalkerSelect::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"lead_ticks", lead_ticks_}, {"calling", calling_}, {"want", want_}, {"cur", cur_},
                          {"prev_cur", prev_cur_}, {"ticks", ticks_}, {"agree", agree_}, {"calls", calls_}, {"want_ticks", {want_ticks_[0], want_ticks_[1]}}};
}
void WalkerSelect::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    lead_ticks_ = s.value("lead_ticks", int64_t{0}); calling_ = s.value("calling", false);
    want_ = s.value("want", 0); cur_ = s.value("cur", 0); prev_cur_ = s.value("prev_cur", 0);
    ticks_ = s.value("ticks", int64_t{0}); agree_ = s.value("agree", int64_t{0}); calls_ = s.value("calls", int64_t{0});
    if (s.contains("want_ticks") && s["want_ticks"].size() == 2) { want_ticks_[0] = s["want_ticks"][0].get<int64_t>(); want_ticks_[1] = s["want_ticks"][1].get<int64_t>(); }
}
nlohmann::json WalkerSelect::diag_snapshot() const {
    return nlohmann::json{{"want", want_}, {"incumbent", cur_}, {"lead", lead_}, {"lead_s", double(lead_ticks_) / hz_}, {"g_rhythm", g_[0]}, {"g_sequencer", g_[1]},
                          {"agree", want_ == cur_}, {"ticks", ticks_}, {"agree_ticks", agree_}, {"calls", calls_},
                          {"want_rhythm_ticks", want_ticks_[0]}, {"want_sequencer_ticks", want_ticks_[1]},
                          {"cost_rhythm_s", r_[0].cost}, {"cost_sequencer_s", r_[1].cost}};
}

} // namespace ogma
