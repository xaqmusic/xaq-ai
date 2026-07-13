// =============================================================================
// ScentHomingLearner.cpp  --  LEARNED scent-homing (Pathway A)
// =============================================================================
#include "ogma/modules/ScentHomingLearner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("ScentHomingLearner: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("ScentHomingLearner: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ScentHomingLearner: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("ScentHomingLearner: param '" + k + "' must be bool");
}
} // namespace

ScentHomingLearner::ScentHomingLearner()  = default;
ScentHomingLearner::~ScentHomingLearner() = default;

std::string_view ScentHomingLearner::type_name() const { return "ScentHomingLearner"; }

std::vector<TopicSpec> ScentHomingLearner::input_topics() const {
    return {
        TopicSpec{ring_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{progress_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{hit_topic_,      std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> ScentHomingLearner::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema ScentHomingLearner::params_schema() const {
    return {
        {"ring_topic", ParamMutability::ConstructionOnly,
            "Raw per-nostril scent ring ProprioToken (NOT a computed bearing).",
            ParamValue{std::string("reality.proprio.scent")}},
        {"progress_topic", ParamMutability::ConstructionOnly,
            "Scalar proximity truth (Δ over the commit window = progress reward).",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"hit_topic", ParamMutability::ConstructionOnly,
            "Hit event (EnvEvent) — the agent's OWN reward (it ate). Authoritative "
            "credit for the heading that reached food (robust to scent scale).",
            ParamValue{std::string("events.hit")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Learned egocentric heading [hx,hy] → HeadingController.input_topic.",
            ParamValue{std::string("percept.scent_homing")}},
        {"bootstrap_topic", ParamMutability::ConstructionOnly,
            "Teacher bearing (analytic compass) that GUIDES exploration while V learns the "
            "action-consequence; dropped after lesion_after_ticks. Empty = self-drive (cold start).",
            ParamValue{std::string("")}},
        {"lesion_after_ticks", ParamMutability::HotMutable,
            "≥0 → drop the bootstrap after N ticks; foraging then runs on the learned V alone.",
            ParamValue{int64_t{-1}}},
        {"ring_dim", ParamMutability::ConstructionOnly, "Nostril count.", ParamValue{int64_t{8}}},
        {"max_prototypes", ParamMutability::ConstructionOnly,
            "Max VQ prototypes (learned categorical scent percepts).", ParamValue{int64_t{24}}},
        {"novelty_thresh", ParamMutability::ConstructionOnly,
            "L2 distance above which a new prototype is grown (unit-pattern scale).", ParamValue{0.30}},
        {"proto_lr", ParamMutability::ConstructionOnly, "VQ winner→ring update rate.", ParamValue{0.05}},
        {"center_ring", ParamMutability::ConstructionOnly,
            "Subtract the per-tick nostril mean before VQ (remove common-mode so the VQ "
            "clusters by ANGULAR pattern, not concentration). Whitening, not bearing-compute.",
            ParamValue{true}},
        {"normalize_ring", ParamMutability::ConstructionOnly,
            "L2-normalize the centered ring before VQ (scale-independent angular state).",
            ParamValue{true}},
        {"n_action_sectors", ParamMutability::ConstructionOnly,
            "Egocentric heading sectors (sector 0 = straight ahead).", ParamValue{int64_t{8}}},
        {"commit_ticks", ParamMutability::HotMutable,
            "Hold a chosen heading this many ticks, then credit Δscent + reselect.", ParamValue{int64_t{6}}},
        {"value_lr", ParamMutability::HotMutable, "EMA rate toward Δprogress.", ParamValue{0.1}},
        {"reward_norm", ParamMutability::HotMutable,
            "Whiten Δscent by its running magnitude so the value table lands in O(1) "
            "std-of-progress units (the tabula-rasa value-scale fix). Hits credit r_hit "
            "directly in those units.", ParamValue{true}},
        {"reward_scale_lr", ParamMutability::ConstructionOnly,
            "EMA rate of the running |Δscent| scale (reward whitening).", ParamValue{0.02}},
        {"epistemic_gain", ParamMutability::HotMutable,
            "Count-based exploration bonus weight 1/(1+visits).", ParamValue{0.5}},
        {"temperature", ParamMutability::HotMutable,
            "Softmax sampling temperature (persistent exploration; <=0 = argmax).", ParamValue{1.0}},
        {"signal_floor", ParamMutability::ConstructionOnly,
            "Ring-sum below this = no scent → emit [0,0], don't learn.", ParamValue{1e-4}},
        {"hit_drop_thresh", ParamMutability::ConstructionOnly,
            "One-tick scent_max drop above this = hit-teleport in window.", ParamValue{0.2}},
        {"r_hit", ParamMutability::ConstructionOnly,
            "Reward credited on a hit-in-window (strong positive).", ParamValue{1.0}},
        {"shuffle", ParamMutability::HotMutable,
            "ABLATION: ignore V, pick a random heading each commit (learning control).", ParamValue{false}},
        {"master_seed", ParamMutability::ConstructionOnly, "RNG seed.", ParamValue{int64_t{7}}},
    };
}

ParamMap ScentHomingLearner::current_params() const {
    ParamMap m;
    m["ring_topic"]       = ParamValue{ring_topic_};
    m["progress_topic"]   = ParamValue{progress_topic_};
    m["hit_topic"]        = ParamValue{hit_topic_};
    m["bootstrap_topic"]  = ParamValue{bootstrap_topic_};
    m["lesion_after_ticks"] = ParamValue{int64_t(lesion_after_ticks_)};
    m["output_topic"]     = ParamValue{output_topic_};
    m["ring_dim"]         = ParamValue{int64_t(ring_dim_)};
    m["max_prototypes"]   = ParamValue{int64_t(max_prototypes_)};
    m["novelty_thresh"]   = ParamValue{double(novelty_thresh_)};
    m["proto_lr"]         = ParamValue{double(proto_lr_)};
    m["center_ring"]      = ParamValue{center_ring_};
    m["normalize_ring"]   = ParamValue{normalize_ring_};
    m["n_action_sectors"] = ParamValue{int64_t(n_action_sectors_)};
    m["commit_ticks"]     = ParamValue{int64_t(commit_ticks_)};
    m["value_lr"]         = ParamValue{double(value_lr_)};
    m["reward_norm"]      = ParamValue{reward_norm_};
    m["reward_scale_lr"]  = ParamValue{double(reward_scale_lr_)};
    m["epistemic_gain"]   = ParamValue{double(epistemic_gain_)};
    m["temperature"]      = ParamValue{double(temperature_)};
    m["signal_floor"]     = ParamValue{double(signal_floor_)};
    m["hit_drop_thresh"]  = ParamValue{double(hit_drop_thresh_)};
    m["r_hit"]            = ParamValue{double(r_hit_)};
    m["shuffle"]          = ParamValue{shuffle_};
    m["master_seed"]      = ParamValue{int64_t(master_seed_)};
    return m;
}

void ScentHomingLearner::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ScentHomingLearner requires a non-null Bus");

    apply_param(params, "ring_topic",     [&](auto const& v){ ring_topic_     = get_string(v,"ring_topic"); });
    apply_param(params, "progress_topic", [&](auto const& v){ progress_topic_ = get_string(v,"progress_topic"); });
    apply_param(params, "hit_topic",      [&](auto const& v){ hit_topic_      = get_string(v,"hit_topic"); });
    apply_param(params, "output_topic",   [&](auto const& v){ output_topic_   = get_string(v,"output_topic"); });
    apply_param(params, "bootstrap_topic",[&](auto const& v){ bootstrap_topic_= get_string(v,"bootstrap_topic"); });
    apply_param(params, "lesion_after_ticks",[&](auto const& v){ lesion_after_ticks_ = int(get_int(v,"lesion_after_ticks")); });
    apply_param(params, "ring_dim",       [&](auto const& v){ ring_dim_       = int(get_int(v,"ring_dim")); });
    apply_param(params, "max_prototypes", [&](auto const& v){ max_prototypes_ = int(get_int(v,"max_prototypes")); });
    apply_param(params, "novelty_thresh", [&](auto const& v){ novelty_thresh_ = float(get_double(v,"novelty_thresh")); });
    apply_param(params, "proto_lr",       [&](auto const& v){ proto_lr_       = float(get_double(v,"proto_lr")); });
    apply_param(params, "center_ring",    [&](auto const& v){ center_ring_    = get_bool(v,"center_ring"); });
    apply_param(params, "normalize_ring", [&](auto const& v){ normalize_ring_ = get_bool(v,"normalize_ring"); });
    apply_param(params, "n_action_sectors",[&](auto const& v){ n_action_sectors_ = int(get_int(v,"n_action_sectors")); });
    apply_param(params, "commit_ticks",   [&](auto const& v){ commit_ticks_   = int(get_int(v,"commit_ticks")); });
    apply_param(params, "value_lr",       [&](auto const& v){ value_lr_       = float(get_double(v,"value_lr")); });
    apply_param(params, "reward_norm",    [&](auto const& v){ reward_norm_    = get_bool(v,"reward_norm"); });
    apply_param(params, "reward_scale_lr",[&](auto const& v){ reward_scale_lr_= float(get_double(v,"reward_scale_lr")); });
    apply_param(params, "epistemic_gain", [&](auto const& v){ epistemic_gain_ = float(get_double(v,"epistemic_gain")); });
    apply_param(params, "temperature",    [&](auto const& v){ temperature_    = float(get_double(v,"temperature")); });
    apply_param(params, "signal_floor",   [&](auto const& v){ signal_floor_   = float(get_double(v,"signal_floor")); });
    apply_param(params, "hit_drop_thresh",[&](auto const& v){ hit_drop_thresh_= float(get_double(v,"hit_drop_thresh")); });
    apply_param(params, "r_hit",          [&](auto const& v){ r_hit_          = float(get_double(v,"r_hit")); });
    apply_param(params, "shuffle",        [&](auto const& v){ shuffle_        = get_bool(v,"shuffle"); });
    apply_param(params, "master_seed",    [&](auto const& v){ master_seed_    = uint64_t(get_int(v,"master_seed")); });

    if (max_prototypes_ < 1)   max_prototypes_ = 1;
    if (n_action_sectors_ < 1) n_action_sectors_ = 1;
    if (commit_ticks_ < 1)     commit_ticks_ = 1;

    V_.assign(size_t(max_prototypes_) * n_action_sectors_, 0.0f);
    visits_.assign(size_t(max_prototypes_) * n_action_sectors_, 0);
    rng_.seed(master_seed_);

    if (!ring_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(ring_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_ring(p); }));
    if (!progress_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(progress_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_progress(p); }));
    if (!hit_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(hit_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_hit(p); }));
    if (!bootstrap_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(bootstrap_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_bootstrap(p); }));
}

void ScentHomingLearner::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "commit_ticks")   commit_ticks_   = int(get_int(value,k));
    else if (k == "value_lr")       value_lr_       = float(get_double(value,k));
    else if (k == "reward_norm")    reward_norm_    = get_bool(value,k);
    else if (k == "epistemic_gain") epistemic_gain_ = float(get_double(value,k));
    else if (k == "temperature")    temperature_    = float(get_double(value,k));
    else if (k == "shuffle")        shuffle_        = get_bool(value,k);
    else if (k == "lesion_after_ticks") lesion_after_ticks_ = int(get_int(value,k));
    else throw std::invalid_argument("ScentHomingLearner: param '" + k + "' is construction-only / unknown");
}

void ScentHomingLearner::handle_ring(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    ring_.assign(pt->values.begin(), pt->values.end());
    have_ring_ = true;
}

void ScentHomingLearner::handle_progress(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) smax_ = float(pt->values[0]);
}

void ScentHomingLearner::handle_bootstrap(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() >= 2) {
        boot_cx_ = float(pt->values[0]); boot_cy_ = float(pt->values[1]);
        have_boot_ = (boot_cx_ * boot_cx_ + boot_cy_ * boot_cy_) > 1e-8f;
    }
}

void ScentHomingLearner::handle_hit(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    // The agent ate during this commit window → the committed heading reached food.
    // Authoritative reward (interoceptive energy gain), robust to scent scale.
    hit_in_window_ = true;
}

int ScentHomingLearner::vq_winner() {
    // VQ over the CONDITIONED ring (vq_in_), so clustering is by angular pattern.
    if (protos_.empty()) { protos_.push_back(vq_in_); return 0; }
    int   best = 0;
    float bestd = std::numeric_limits<float>::infinity();
    for (int i = 0; i < int(protos_.size()); ++i) {
        float d = 0.0f;
        int n = std::min<int>(int(protos_[i].size()), int(vq_in_.size()));
        for (int k = 0; k < n; ++k) { float e = protos_[i][k] - vq_in_[k]; d += e*e; }
        if (d < bestd) { bestd = d; best = i; }
    }
    if (std::sqrt(bestd) > novelty_thresh_ && int(protos_.size()) < max_prototypes_) {
        protos_.push_back(vq_in_);
        return int(protos_.size()) - 1;
    }
    int n = std::min<int>(int(protos_[best].size()), int(vq_in_.size()));
    for (int k = 0; k < n; ++k) protos_[best][k] += proto_lr_ * (vq_in_[k] - protos_[best][k]);
    return best;
}

int ScentHomingLearner::select_action(int proto) {
    int n = n_action_sectors_;
    int base = proto * n;
    if (shuffle_) {
        std::uniform_int_distribution<int> u(0, n - 1);
        last_eps_pick_ = true;
        return u(rng_);
    }
    // argmax of V (the exploit choice), for the explore-pick flag
    int   v_arg = 0; float v_best = V_[base];
    for (int a = 1; a < n; ++a) if (V_[base + a] > v_best) { v_best = V_[base + a]; v_arg = a; }

    std::vector<float> score(n);
    for (int a = 0; a < n; ++a)
        score[a] = V_[base + a] + epistemic_gain_ / (1.0f + float(visits_[base + a]));

    int chosen;
    if (temperature_ > 0.0f) {
        float mx = *std::max_element(score.begin(), score.end());
        std::vector<float> w(n);
        float sum = 0.0f;
        for (int a = 0; a < n; ++a) { w[a] = std::exp((score[a] - mx) / temperature_); sum += w[a]; }
        std::uniform_real_distribution<float> u(0.0f, sum);
        float r = u(rng_), acc = 0.0f;
        chosen = n - 1;
        for (int a = 0; a < n; ++a) { acc += w[a]; if (r <= acc) { chosen = a; break; } }
    } else {
        chosen = 0; float best = score[0];
        for (int a = 1; a < n; ++a) if (score[a] > best) { best = score[a]; chosen = a; }
    }
    last_eps_pick_ = (chosen != v_arg);
    return chosen;
}

float ScentHomingLearner::action_center(int sector) const {
    float a = float(sector) * (kTwoPi / float(n_action_sectors_));
    if (a > kPi) a -= kTwoPi;
    return a;   // sector 0 = 0 rad = straight ahead
}

int ScentHomingLearner::sector_of(float cx, float cy) const {
    // inverse of action_center: egocentric bearing [cx=+right, cy=+forward] → sector.
    float theta = std::atan2(cx, cy);          // angle from forward, +right (matches publish_heading)
    if (theta < 0.0f) theta += kTwoPi;
    int s = int(std::lround(theta / (kTwoPi / float(n_action_sectors_)))) % n_action_sectors_;
    return s < 0 ? s + n_action_sectors_ : s;
}

void ScentHomingLearner::tick(uint64_t tick_id) {
    auto publish_heading = [&](float hx, float hy) {
        auto out = std::make_shared<ProprioToken>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("scent_homing") : id_;
        out->sensor      = "scent_homing";
        out->values.resize(2);
        out->values[0] = hx;   // +right
        out->values[1] = hy;   // +forward
        bus_->publish(output_topic_, out);
    };

    if (lesion_after_ticks_ >= 0 && tick_count_ >= uint64_t(lesion_after_ticks_)) lesioned_ = true;
    ++tick_count_;

    if (!have_ring_) { publish_heading(0.0f, 0.0f); return; }

    float ring_sum = 0.0f;
    for (float v : ring_) ring_sum += std::fabs(v);
    if (ring_sum < signal_floor_) {
        // No scent here — this pathway has nothing to home on (Pathway B will
        // drive exploration). Reset the commit so a stale heading isn't held.
        have_committed_ = false;
        prev_smax_ = smax_; have_prev_smax_ = true;
        last_proto_ = -1;
        publish_heading(0.0f, 0.0f);
        return;
    }

    // Condition the ring → vq_in_: center (remove common-mode = proximity) + L2-normalize
    // (scale-independent angular pattern). This is what lets the VQ cluster by DIRECTION
    // instead of collapsing to one concentration-blob node. Not a bearing computation —
    // the cluster→heading mapping is still learned.
    vq_in_.assign(ring_.begin(), ring_.end());
    if (center_ring_ && !vq_in_.empty()) {
        float mean = 0.0f;
        for (float v : vq_in_) mean += v;
        mean /= float(vq_in_.size());
        for (float& v : vq_in_) v -= mean;
    }
    if (normalize_ring_) {
        float m = 0.0f;
        for (float v : vq_in_) m += v * v;
        m = std::sqrt(m);
        if (m > 1e-9f) for (float& v : vq_in_) v /= m;
    }

    int winner = vq_winner();

    // Hit-teleport: a one-tick collapse of scent_max during a commit = ate + respawn far.
    if (have_committed_ && have_prev_smax_ && (prev_smax_ - smax_) > hit_drop_thresh_)
        hit_in_window_ = true;

    auto start_commit = [&](int proto) {
        committed_proto_  = proto;
        // BOOTSTRAP: while the compass teacher is present (pre-lesion), commit to ITS
        // sector — the bug forages via the compass AND V[proto][that-sector] learns the
        // real Δscent. Post-lesion the compass is gone: commit to argmax V (learned-alone).
        committed_action_ = (!lesioned_ && have_boot_) ? sector_of(boot_cx_, boot_cy_)
                                                       : select_action(proto);
        ticks_left_       = commit_ticks_;
        window_start_smax_ = smax_;
        hit_in_window_    = false;
        have_committed_   = true;
    };

    if (!have_committed_) {
        start_commit(winner);
    } else {
        --ticks_left_;
        if (ticks_left_ <= 0) {
            float raw_dprog = smax_ - window_start_smax_;   // gradient progress this window
            float dprog;
            if (hit_in_window_) {
                dprog = r_hit_;   // a hit credits r_hit directly (already in normalized units)
            } else if (reward_norm_) {
                // Track the running |Δ| scale and whiten → O(1) std-of-progress units.
                if (reward_scale_ <= 0.0f) reward_scale_ = std::max(std::fabs(raw_dprog), 1e-6f);
                else reward_scale_ += reward_scale_lr_ * (std::fabs(raw_dprog) - reward_scale_);
                dprog = raw_dprog / (reward_scale_ + 1e-6f);
            } else {
                dprog = raw_dprog;
            }
            int idx = committed_proto_ * n_action_sectors_ + committed_action_;
            V_[idx]      += value_lr_ * (dprog - V_[idx]);
            visits_[idx] += 1;
            last_win_dprog_ = dprog;
            start_commit(winner);   // credit done → reselect for the current state
        }
    }

    float theta = action_center(committed_action_);
    publish_heading(std::sin(theta), std::cos(theta));

    // telemetry over the current state's row
    last_proto_  = winner;
    last_action_ = committed_action_;
    int base = winner * n_action_sectors_;
    float vmax = V_[base], vsum = V_[base];
    for (int a = 1; a < n_action_sectors_; ++a) { vmax = std::max(vmax, V_[base+a]); vsum += V_[base+a]; }
    last_vmax_    = vmax;
    last_vspread_ = vmax - vsum / float(n_action_sectors_);

    prev_smax_ = smax_; have_prev_smax_ = true;
}

float ScentHomingLearner::action_bearing() const {
    if (last_action_ < 0) return 0.0f;
    return action_center(last_action_) / kPi;
}

float ScentHomingLearner::value_at(int proto, int action) const {
    int idx = proto * n_action_sectors_ + action;
    if (idx < 0 || idx >= int(V_.size())) return 0.0f;
    return V_[idx];
}

nlohmann::json ScentHomingLearner::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"V", V_}, {"visits", visits_}, {"n_proto", int(protos_.size())}};
}

nlohmann::json ScentHomingLearner::diag_snapshot() const {
    return nlohmann::json{
        {"n_proto",  int(protos_.size())},
        {"proto",    last_proto_},
        {"action",   last_action_},
        {"abearing", action_bearing()},
        {"vspread",  last_vspread_},
        {"vmax",     last_vmax_},
        {"dprog",    last_win_dprog_},
        {"eps",      last_eps_pick_},
    };
}

void ScentHomingLearner::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    if (s.value("version", 0) != 1) return;
    if (s.contains("V"))      V_      = s["V"].get<std::vector<float>>();
    if (s.contains("visits")) visits_ = s["visits"].get<std::vector<int>>();
}

} // namespace ogma
