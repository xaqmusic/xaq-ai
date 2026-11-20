// =============================================================================
// WalkerOutage.cpp  --  see the header (learned stepper plan 9.10, 2026-11-20)
// =============================================================================
#include "ogma/modules/WalkerOutage.hpp"

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
    throw std::invalid_argument("WalkerOutage: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("WalkerOutage: param '" + k + "' must be a string");
}
} // namespace

WalkerOutage::WalkerOutage()  = default;
WalkerOutage::~WalkerOutage() = default;

std::string_view WalkerOutage::type_name() const { return "WalkerOutage"; }

std::vector<TopicSpec> WalkerOutage::input_topics() const {
    const auto pt = std::type_index(typeid(ProprioToken));
    std::vector<TopicSpec> v;
    v.push_back(TopicSpec{reset_topic_, pt, SubscriptionKind::Direct, false});
    if (!rail_topic_.empty())   v.push_back(TopicSpec{rail_topic_, pt, SubscriptionKind::Direct, false});
    if (!share_topic_.empty())  v.push_back(TopicSpec{share_topic_, pt, SubscriptionKind::Direct, false});
    if (!stride_topic_.empty()) v.push_back(TopicSpec{stride_topic_, pt, SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> WalkerOutage::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema WalkerOutage::params_schema() const {
    return {
        {"reset_topic",  ParamMutability::ConstructionOnly, "ProprioToken [count, ...]: the supply's outages (HAT resets) since the host started; a rise is an outage.", ParamValue{std::string("sense.hat_reset")}},
        {"rail_topic",   ParamMutability::ConstructionOnly, "ProprioToken [V]: the HAT's 3.3 V rail. \"\" = no dips.", ParamValue{std::string("sense.hat_rail")}},
        {"share_topic",  ParamMutability::ConstructionOnly, "ProprioToken: the sequencer's share of the command at share_index (the pose socket). \"\" = the rhythm always drives.", ParamValue{std::string("pose.targets")}},
        {"share_index",  ParamMutability::ConstructionOnly, "Index of the share in share_topic.", ParamValue{int64_t{0}}},
        {"stride_topic", ParamMutability::ConstructionOnly, "ProprioToken: the stride odometry's velocity (m/s); stride_index is forward. \"\" = travel changes nothing.", ParamValue{std::string("reality.proprio.stride_v")}},
        {"stride_index", ParamMutability::ConstructionOnly, "Index of the forward speed in stride_topic.", ParamValue{int64_t{1}}},
        {"output_topic", ParamMutability::ConstructionOnly, "ProprioToken [est_r, est_s, veto_r, veto_s, dips_r, dips_s, driver, resets, p_same_r, p_same_s, backoff_r, backoff_s].", ParamValue{std::string("select.outage")}},
        {"window_s",     ParamMutability::HotMutable, "An outage counts for this many seconds of its walker's driving (the daemon's window is 60).", ParamValue{60.0}},
        {"veto_n",       ParamMutability::HotMutable, "SCAFFOLD. Outages in a window at which a walker is vetoed (the daemon stops recovering on its own past 3).", ParamValue{2.0}},
        {"release",      ParamMutability::HotMutable, "The veto drops when the walker's estimate is at or under release x veto_n.", ParamValue{0.5}},
        {"relax_m",      ParamMutability::HotMutable, "SCAFFOLD. While idle a walker's estimate decays over this many metres of the body's net travel, doubled per veto (0 = held).", ParamValue{0.25}},
        {"backoff_max",  ParamMutability::HotMutable, "The most doublings of relax_m.", ParamValue{int64_t{4}}},
        {"dip_v",        ParamMutability::HotMutable, "A rail dip is an entry below this (V).", ParamValue{3.0}},
        {"drive_min",    ParamMutability::HotMutable, "A walker holds the legs when its share is at least this.", ParamValue{0.99}},
        {"hz",           ParamMutability::ConstructionOnly, "Tick rate.", ParamValue{50.0}},
    };
}

ParamMap WalkerOutage::current_params() const {
    ParamMap m;
    m["reset_topic"] = ParamValue{reset_topic_}; m["rail_topic"] = ParamValue{rail_topic_};
    m["share_topic"] = ParamValue{share_topic_}; m["share_index"] = ParamValue{int64_t(share_index_)};
    m["stride_topic"] = ParamValue{stride_topic_}; m["stride_index"] = ParamValue{int64_t(stride_index_)};
    m["output_topic"] = ParamValue{output_topic_};
    m["window_s"] = ParamValue{window_s_}; m["veto_n"] = ParamValue{veto_n_}; m["release"] = ParamValue{release_};
    m["relax_m"] = ParamValue{relax_m_}; m["backoff_max"] = ParamValue{int64_t(backoff_max_)}; m["dip_v"] = ParamValue{dip_v_};
    m["drive_min"] = ParamValue{drive_min_}; m["hz"] = ParamValue{hz_};
    return m;
}

void WalkerOutage::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("WalkerOutage requires a non-null Bus");
    apply_param(params, "reset_topic",  [&](auto const& v){ reset_topic_  = get_string(v, "reset_topic"); });
    apply_param(params, "rail_topic",   [&](auto const& v){ rail_topic_   = get_string(v, "rail_topic"); });
    apply_param(params, "share_topic",  [&](auto const& v){ share_topic_  = get_string(v, "share_topic"); });
    apply_param(params, "share_index",  [&](auto const& v){ share_index_  = int(get_double(v, "share_index")); });
    apply_param(params, "stride_topic", [&](auto const& v){ stride_topic_ = get_string(v, "stride_topic"); });
    apply_param(params, "stride_index", [&](auto const& v){ stride_index_ = int(get_double(v, "stride_index")); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "hz",           [&](auto const& v){ hz_ = std::max(1.0, get_double(v, "hz")); });
    for (auto const& k : {"window_s", "veto_n", "release", "relax_m", "backoff_max", "dip_v", "drive_min"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    if (reset_topic_.empty())  throw std::invalid_argument("WalkerOutage: reset_topic is required");
    if (output_topic_.empty()) throw std::invalid_argument("WalkerOutage: output_topic is required");
    if (share_index_ < 0 || stride_index_ < 0) throw std::invalid_argument("WalkerOutage: an index is negative");

    auto scalar = [this](std::string const& topic, int index, double* dst, bool* have) {
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct,
            [this, index, dst, have](std::string_view, MessagePtr p){
                if (!input_allowed(p->producer_id)) return;
                auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
                if (!pt || index >= pt->values.size()) return;
                *dst = double(pt->values[index]); if (have) *have = true;
            }));
    };
    scalar(reset_topic_, 0, &reset_count_, &have_reset_);
    if (!rail_topic_.empty())   scalar(rail_topic_, 0, &rail_, &have_rail_);
    if (!share_topic_.empty())  scalar(share_topic_, share_index_, &share_, &have_share_);
    if (!stride_topic_.empty()) scalar(stride_topic_, stride_index_, &vfwd_, nullptr);
}

void WalkerOutage::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "window_s")    window_s_    = std::max(1.0, get_double(value, k));
    else if (k == "veto_n")      veto_n_      = std::max(0.0, get_double(value, k));
    else if (k == "release")     release_     = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "relax_m")     relax_m_     = std::max(0.0, get_double(value, k));
    else if (k == "backoff_max") backoff_max_ = std::clamp(int(get_double(value, k)), 0, 16);
    else if (k == "dip_v")       dip_v_       = get_double(value, k);
    else if (k == "drive_min")   drive_min_   = std::clamp(get_double(value, k), 0.5, 1.0);
    else throw std::invalid_argument("WalkerOutage: param '" + k + "' is construction-only / unknown");
}

void WalkerOutage::tick(uint64_t tick_id) {
    const double dt = 1.0 / hz_;
    const double s = have_share_ ? std::clamp(share_, 0.0, 1.0) : 0.0;
    const std::array<double, 2> own{1.0 - s, s};
    const int larger = s >= 0.5 ? 1 : 0;
    driver_ = own[1] >= drive_min_ ? 1 : own[0] >= drive_min_ ? 0 : -1;

    // the events of this tick, attributed to the walker with the larger share
    int new_resets = 0;
    if (have_reset_) {
        if (primed_ && reset_count_ > last_count_) new_resets = int(std::lround(reset_count_ - last_count_));
        last_count_ = reset_count_; primed_ = true;       // a count that falls is the host restarting: taken as the new zero
    }
    bool new_dip = false;
    if (have_rail_) { const bool low = rail_ < dip_v_; new_dip = low && !in_dip_; in_dip_ = low; }

    for (int w = 0; w < 2; ++w) {
        const size_t i = size_t(w);
        const bool drive = own[i] >= drive_min_;
        if (drive && !driving_[i]) {                       // the take-over: start on what it was believed to cost here
            for (auto& e : ev_[i])  e.w *= p_same_[i];
            for (auto& e : dip_[i]) e.w *= p_same_[i];
            idle_net_[i] = 0.0; p_same_[i] = 1.0; clean_s_[i] = 0.0;
        }
        if (!drive && driving_[i]) idle_net_[i] = 0.0;     // the hand-off
        if (drive) {                                       // its clock runs only at the legs
            for (auto& e : ev_[i])  e.age_s += dt;
            for (auto& e : dip_[i]) e.age_s += dt;
            auto old = [this](Ev const& e){ return e.age_s > window_s_ || e.w < 1e-3; };
            ev_[i].erase(std::remove_if(ev_[i].begin(), ev_[i].end(), old), ev_[i].end());
            dip_[i].erase(std::remove_if(dip_[i].begin(), dip_[i].end(), old), dip_[i].end());
            clean_s_[i] += dt;
            if (clean_s_[i] >= window_s_ && ev_[i].empty()) backoff_[i] = 0;   // a clean window at the legs
        } else if (own[i] < 1.0 - drive_min_) {            // idle: the body's net travel is what makes the ground another
            idle_net_[i] += vfwd_ * dt;
            const double scale = relax_m_ * double(1 << std::min(backoff_[i], backoff_max_));
            p_same_[i] = scale > 0.0 ? std::exp(-std::fabs(idle_net_[i]) / scale) : 1.0;
        }
        bool hit = false;
        if (w == larger) {
            for (int k = 0; k < new_resets; ++k) { ev_[i].push_back(Ev{}); clean_s_[i] = 0.0; hit = true; }
            if (new_dip) dip_[i].push_back(Ev{});
        }
        est_[i] = (drive ? 1.0 : p_same_[i]) * sum(ev_[i]);
        // on probation (vetoed before, no clean window since) one more outage is enough: a retry costs one brownout, not veto_n
        if (!veto_[i] && veto_n_ > 0.0 && (est_[i] >= veto_n_ || (hit && backoff_[i] > 0))) {
            veto_[i] = true; ++vetoes_[i]; backoff_[i] = std::min(backoff_[i] + 1, backoff_max_);
        } else if (veto_[i] && !drive && est_[i] <= release_ * veto_n_) veto_[i] = false;   // released by travel under the other walker, never at the legs
        driving_[i] = drive;
    }
    resets_seen_ += new_resets; if (new_dip) ++dips_seen_; ++ticks_;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? output_topic_ : id_; out->sensor = "walker_outage";
    out->values = Eigen::VectorXf(12);
    out->values[0] = float(est_[0]); out->values[1] = float(est_[1]); out->values[2] = veto_[0] ? 1.0f : 0.0f; out->values[3] = veto_[1] ? 1.0f : 0.0f;
    out->values[4] = float(sum(dip_[0])); out->values[5] = float(sum(dip_[1])); out->values[6] = float(driver_); out->values[7] = float(resets_seen_);
    out->values[8] = float(p_same_[0]); out->values[9] = float(p_same_[1]); out->values[10] = float(backoff_[0]); out->values[11] = float(backoff_[1]);
    bus_->publish(output_topic_, out);
}

namespace {
nlohmann::json evs(std::vector<std::pair<double, double>> const& v) { nlohmann::json a = nlohmann::json::array(); for (auto const& e : v) a.push_back({e.first, e.second}); return a; }
}

nlohmann::json WalkerOutage::snapshot_state() const {
    auto pack = [](std::vector<Ev> const& v){ std::vector<std::pair<double, double>> o; for (auto const& e : v) o.push_back({e.age_s, e.w}); return evs(o); };
    return nlohmann::json{{"version", 1}, {"ev", {pack(ev_[0]), pack(ev_[1])}}, {"dip", {pack(dip_[0]), pack(dip_[1])}},
                          {"p_same", {p_same_[0], p_same_[1]}}, {"idle_net", {idle_net_[0], idle_net_[1]}}, {"clean_s", {clean_s_[0], clean_s_[1]}},
                          {"veto", {veto_[0], veto_[1]}}, {"driving", {driving_[0], driving_[1]}}, {"backoff", {backoff_[0], backoff_[1]}},
                          {"vetoes", {vetoes_[0], vetoes_[1]}}, {"resets_seen", resets_seen_}, {"dips_seen", dips_seen_}, {"in_dip", in_dip_}};
}
void WalkerOutage::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    auto unpack = [](nlohmann::json const& a){ std::vector<Ev> v; for (auto const& e : a) v.push_back(Ev{e[0].get<double>(), e[1].get<double>()}); return v; };
    for (size_t i = 0; i < 2; ++i) {
        ev_[i] = unpack(s["ev"][i]); dip_[i] = unpack(s["dip"][i]);
        p_same_[i] = s["p_same"][i].get<double>(); idle_net_[i] = s["idle_net"][i].get<double>(); clean_s_[i] = s["clean_s"][i].get<double>();
        veto_[i] = s["veto"][i].get<bool>(); driving_[i] = s["driving"][i].get<bool>(); backoff_[i] = s["backoff"][i].get<int>();
        vetoes_[i] = s["vetoes"][i].get<int64_t>();
        est_[i] = (driving_[i] ? 1.0 : p_same_[i]) * sum(ev_[i]);
    }
    resets_seen_ = s.value("resets_seen", int64_t{0}); dips_seen_ = s.value("dips_seen", int64_t{0}); in_dip_ = s.value("in_dip", false);
    primed_ = false;   // the host's count starts again with the host
}
nlohmann::json WalkerOutage::diag_snapshot() const {
    return nlohmann::json{{"est", {est_[0], est_[1]}}, {"n", {sum(ev_[0]), sum(ev_[1])}}, {"dips", {sum(dip_[0]), sum(dip_[1])}},
                          {"veto", {veto_[0] ? 1 : 0, veto_[1] ? 1 : 0}}, {"vetoes", {vetoes_[0], vetoes_[1]}}, {"backoff", {backoff_[0], backoff_[1]}},
                          {"p_same", {p_same_[0], p_same_[1]}}, {"driver", driver_}, {"resets", resets_seen_}, {"dips_seen", dips_seen_}, {"rail", rail_}};
}

} // namespace ogma
