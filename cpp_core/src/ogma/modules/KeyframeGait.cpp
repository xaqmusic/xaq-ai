#include "ogma/modules/KeyframeGait.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;

std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("KeyframeGait param '" + key + "' must be a string array");
}
std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("KeyframeGait param '" + key + "' must be a numeric array");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("KeyframeGait param '" + key + "' must be string");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("KeyframeGait param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("KeyframeGait param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("KeyframeGait param '" + key + "' must be bool");
}
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
} // namespace

KeyframeGait::KeyframeGait()  = default;
KeyframeGait::~KeyframeGait() = default;

std::string_view KeyframeGait::type_name() const { return "KeyframeGait"; }

std::vector<TopicSpec> KeyframeGait::input_topics() const {
    std::vector<TopicSpec> v;
    v.emplace_back(cpg_topic_, std::type_index(typeid(ProprioToken)),
                   SubscriptionKind::Direct, /*required=*/false);
    for (auto const& t : proprio_topics_)
        v.emplace_back(t, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> KeyframeGait::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(objective_output_topics_.size() + velocity_output_topics_.size());
    for (auto const& t : objective_output_topics_)
        v.emplace_back(t, std::type_index(typeid(PredictionToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    for (auto const& t : velocity_output_topics_)
        v.emplace_back(t, std::type_index(typeid(PredictionToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

ParamSchema KeyframeGait::params_schema() const {
    return {
        {"cpg_topic", ParamMutability::ConstructionOnly,
         "CPG phase-clock topic (ProprioToken [cos φ, sin φ]); φ = atan2(sin,cos) indexes the keyframe.",
         std::nullopt, std::nullopt, std::nullopt},
        {"proprio_topics", ParamMutability::ConstructionOnly,
         "Per-leg ProprioToken input topics ([pos,act,delta]×motor_dim).",
         std::nullopt, std::nullopt, std::nullopt},
        {"objective_output_topics", ParamMutability::ConstructionOnly,
         "Per-leg PredictionToken output topics (objective.posture.<leg>); the current bin's keyframe posture is published here.",
         std::nullopt, std::nullopt, std::nullopt},
        {"velocity_output_topics", ParamMutability::ConstructionOnly,
         "Optional per-leg PredictionToken output topics (objective.velocity.<leg>); the current bin's phase-indexed VELOCITY target (EMA of proprio delta) is published here — the propulsive trajectory MotorEPM's velocity feed-forward (Cvel) descends. Empty = velocity socket OFF (map still accumulates but is not published).",
         std::nullopt, std::nullopt, std::nullopt},
        {"motor_dim", ParamMutability::ConstructionOnly, "joints per leg",
         ParamValue{int64_t(3)}, ParamValue{int64_t(1)}, ParamValue{int64_t(12)}},
        {"n_bins", ParamMutability::ConstructionOnly, "number of CPG-phase bins",
         ParamValue{int64_t(16)}, ParamValue{int64_t(2)}, ParamValue{int64_t(256)}},
        {"keyframe_alpha", ParamMutability::HotMutable,
         "Cross-cycle EMA rate for the keyframe + its TLE (slow = crystallize over cycles).",
         ParamValue{0.02}, ParamValue{0.0}, ParamValue{1.0}},
        {"gain", ParamMutability::HotMutable,
         "Base objective weight (policy). Published confidence w = gain · self_precision(bin), clamped [0,1]. Soft — a strong objective over-constrains the HK loop.",
         ParamValue{0.3}, ParamValue{0.0}, ParamValue{1.0}},
        {"warmup_visits", ParamMutability::HotMutable,
         "Per-bin visits before it may drive (crystallize-then-drive: prevents driving on an unproven bin whose deviation-EMA hasn't converged).",
         ParamValue{int64_t(24)}, ParamValue{int64_t(0)}, ParamValue{int64_t(100000)}},
        {"precision_scale", ParamMutability::HotMutable,
         "Self-precision softness: precision = warmup_ramp · exp(−bin_dev/scale). Smaller = only very consistent bins drive.",
         ParamValue{0.6}, ParamValue{0.01}, ParamValue{100.0}},
        {"shuffle_phase", ParamMutability::HotMutable,
         "Ablation: index the keyframe by a RANDOM bin instead of the CPG phase → the map washes out. Proves the phase-index does the work.",
         ParamValue{false}, std::nullopt, std::nullopt},
        {"freeze_map", ParamMutability::HotMutable,
         "Ablation: stop updating the keyframe (keep publishing the frozen map). Proves improvement is learning, not mechanics.",
         ParamValue{false}, std::nullopt, std::nullopt},
        {"publish", ParamMutability::HotMutable,
         "false = accumulate the map but do NOT drive the objective socket (ablate the descending closure).",
         ParamValue{true}, std::nullopt, std::nullopt},
        {"symmetry_pairs", ParamMutability::ConstructionOnly,
         "Flat list of left-right leg-index pairs for the velocity-symmetry prior, e.g. [0,1, 2,3] = pair leg0<->leg1 and leg2<->leg3. Empty = no symmetry. Only meaningful with vel_symmetry_gain > 0.",
         std::nullopt, std::nullopt, std::nullopt},
        {"vel_symmetry_gain", ParamMutability::HotMutable,
         "Anti-circling prior: per-tick pull equalizing the RMS push magnitude of paired legs' joints in the VELOCITY map (posture map untouched), so the propulsive pump can't grow a yaw bias into circling. Preserves each leg's own sign/phase. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"seed", ParamMutability::ConstructionOnly, "RNG seed for the shuffle_phase ablation",
         ParamValue{int64_t(12345)}, std::nullopt, std::nullopt},
    };
}

ParamMap KeyframeGait::current_params() const {
    ParamMap m;
    m["cpg_topic"]               = cpg_topic_;
    m["proprio_topics"]          = std::vector<std::string>(proprio_topics_);
    m["objective_output_topics"] = std::vector<std::string>(objective_output_topics_);
    m["velocity_output_topics"]  = std::vector<std::string>(velocity_output_topics_);
    m["motor_dim"]      = int64_t(motor_dim_);
    m["n_bins"]         = int64_t(n_bins_);
    m["keyframe_alpha"] = keyframe_alpha_;
    m["gain"]           = gain_;
    m["warmup_visits"]  = int64_t(warmup_visits_);
    m["precision_scale"]= precision_scale_;
    m["shuffle_phase"]  = shuffle_phase_;
    m["freeze_map"]     = freeze_map_;
    m["publish"]        = publish_;
    { std::vector<double> sp(symmetry_pairs_.begin(), symmetry_pairs_.end());
      m["symmetry_pairs"] = sp; }
    m["vel_symmetry_gain"] = vel_symmetry_gain_;
    return m;
}

void KeyframeGait::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("KeyframeGait requires a non-null Bus");

    int64_t seed = 12345;
    apply_param(params, "seed",           [&](auto const& v){ seed = get_int(v, "seed"); });
    apply_param(params, "cpg_topic",      [&](auto const& v){ cpg_topic_ = get_string(v, "cpg_topic"); });
    apply_param(params, "motor_dim",      [&](auto const& v){ motor_dim_ = int(get_int(v, "motor_dim")); });
    apply_param(params, "n_bins",         [&](auto const& v){ n_bins_    = int(get_int(v, "n_bins")); });
    apply_param(params, "keyframe_alpha", [&](auto const& v){ keyframe_alpha_ = get_double(v, "keyframe_alpha"); });
    apply_param(params, "gain",           [&](auto const& v){ gain_      = get_double(v, "gain"); });
    apply_param(params, "warmup_visits",  [&](auto const& v){ warmup_visits_   = int(get_int(v, "warmup_visits")); });
    apply_param(params, "precision_scale",[&](auto const& v){ precision_scale_ = get_double(v, "precision_scale"); });
    apply_param(params, "shuffle_phase",  [&](auto const& v){ shuffle_phase_ = get_bool(v, "shuffle_phase"); });
    apply_param(params, "freeze_map",     [&](auto const& v){ freeze_map_    = get_bool(v, "freeze_map"); });
    apply_param(params, "publish",        [&](auto const& v){ publish_       = get_bool(v, "publish"); });
    apply_param(params, "vel_symmetry_gain", [&](auto const& v){ vel_symmetry_gain_ = get_double(v, "vel_symmetry_gain"); });
    apply_param(params, "symmetry_pairs", [&](auto const& v){
        auto d = get_double_vec(v, "symmetry_pairs");
        symmetry_pairs_.clear();
        for (double x : d) symmetry_pairs_.push_back(int(std::lround(x)));
    });
    apply_param(params, "proprio_topics", [&](auto const& v){ proprio_topics_ = get_string_vec(v, "proprio_topics"); });
    apply_param(params, "objective_output_topics",
                [&](auto const& v){ objective_output_topics_ = get_string_vec(v, "objective_output_topics"); });
    apply_param(params, "velocity_output_topics",
                [&](auto const& v){ velocity_output_topics_ = get_string_vec(v, "velocity_output_topics"); });

    if (proprio_topics_.empty())
        throw std::invalid_argument("KeyframeGait: proprio_topics must be non-empty");
    if (objective_output_topics_.size() != proprio_topics_.size())
        throw std::invalid_argument("KeyframeGait: objective_output_topics length must equal proprio_topics");
    if (!velocity_output_topics_.empty() && velocity_output_topics_.size() != proprio_topics_.size())
        throw std::invalid_argument("KeyframeGait: velocity_output_topics length must equal proprio_topics (or be empty)");
    if (n_bins_ < 2) throw std::invalid_argument("KeyframeGait: n_bins must be >= 2");
    if (symmetry_pairs_.size() % 2 != 0)
        throw std::invalid_argument("KeyframeGait: symmetry_pairs must have an even length (flat [legA,legB,...])");
    for (int idx : symmetry_pairs_)
        if (idx < 0 || idx >= int(proprio_topics_.size()))
            throw std::invalid_argument("KeyframeGait: symmetry_pairs leg index out of range");

    int nl = int(proprio_topics_.size());
    int wb = nl * motor_dim_;
    posture_.assign(nl, Eigen::VectorXf());
    vel_.assign(nl, Eigen::VectorXf());
    posture_seen_.assign(nl, 0);
    keyframe_.assign(n_bins_, Eigen::VectorXf::Zero(wb));
    vel_keyframe_.assign(n_bins_, Eigen::VectorXf::Zero(wb));
    bin_dev_ema_.assign(n_bins_, 0.0f);
    bin_count_.assign(n_bins_, 0);
    keyframe_tle_ema_ = 0.0f;
    vel_keyframe_tle_ema_ = 0.0f;
    rng_.seed(uint32_t(seed));

    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(cpg_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_cpg(p); }));
    for (int leg = 0; leg < nl; ++leg)
        sub_ids_.push_back(bus_->subscribe(proprio_topics_[leg], SubscriptionKind::Direct,
            [this, leg](std::string_view, MessagePtr p){ handle_proprio(leg, p); }));
}

void KeyframeGait::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "keyframe_alpha") keyframe_alpha_ = get_double(value, k);
    else if (k == "gain")           gain_          = get_double(value, k);
    else if (k == "warmup_visits")  warmup_visits_   = int(get_int(value, k));
    else if (k == "precision_scale")precision_scale_ = get_double(value, k);
    else if (k == "shuffle_phase")  shuffle_phase_ = get_bool(value, k);
    else if (k == "freeze_map")     freeze_map_    = get_bool(value, k);
    else if (k == "publish")        publish_       = get_bool(value, k);
    else if (k == "vel_symmetry_gain") vel_symmetry_gain_ = get_double(value, k);
    else throw std::invalid_argument("KeyframeGait: param '" + k + "' is not HotMutable");
}

float KeyframeGait::self_precision(int b) const {
    // Objective SELF-precision (no cross-objective logic — the arbiter owns that).
    //   warmup_ramp : 0 until a bin is seen enough that its deviation-EMA is meaningful,
    //                 ramping to 1 (crystallize-then-drive; an unproven bin has dev≈0 and
    //                 would otherwise look falsely confident).
    //   consistency : exp(−bin_dev/scale) — a bin whose posture recurs tightly = high precision.
    if (b < 0 || b >= n_bins_ || bin_count_[b] <= 0) return 0.0f;
    float warmup = std::clamp(float(bin_count_[b]) / float(std::max(1, warmup_visits_)), 0.0f, 1.0f);
    float cons   = std::exp(-bin_dev_ema_[b] / float(precision_scale_));
    return std::clamp(warmup * cons, 0.0f, 1.0f);
}

int KeyframeGait::bin_of(float phi) const {
    float h = std::fmod(phi, kTwoPi);
    if (h < 0.0f) h += kTwoPi;
    int b = int(h / kTwoPi * float(n_bins_));
    if (b < 0) b = 0;
    if (b >= n_bins_) b = n_bins_ - 1;
    return b;
}

void KeyframeGait::handle_cpg(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || p->values.size() < 2) return;
    float phi = std::atan2(p->values[1], p->values[0]);   // [-π, π]
    if (phi < 0.0f) phi += kTwoPi;
    phi_ = phi;
    phi_seen_ = true;
}

void KeyframeGait::handle_proprio(int leg, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || leg < 0 || leg >= int(posture_.size())) return;
    if (p->values.size() >= 3 * motor_dim_) {
        if (posture_[leg].size() != motor_dim_) posture_[leg] = Eigen::VectorXf::Zero(motor_dim_);
        if (vel_[leg].size()     != motor_dim_) vel_[leg]     = Eigen::VectorXf::Zero(motor_dim_);
        for (int j = 0; j < motor_dim_; ++j) {
            posture_[leg][j] = p->values[3 * j];      // position at index 3j
            vel_[leg][j]     = p->values[3 * j + 2];  // delta (velocity proxy) at index 3j+2
        }
        posture_seen_[leg] = 1;
    }
}

void KeyframeGait::tick(uint64_t tick_id) {
    int nl = int(proprio_topics_.size());
    if (!phi_seen_) return;
    for (int leg = 0; leg < nl; ++leg) if (!posture_seen_[leg]) return;

    // Whole-body posture + velocity vectors (concat of per-leg positions / deltas).
    int wb_dim = nl * motor_dim_;
    Eigen::VectorXf wb(wb_dim), wv(wb_dim);
    for (int leg = 0; leg < nl; ++leg) {
        wb.segment(leg * motor_dim_, motor_dim_) = posture_[leg];
        wv.segment(leg * motor_dim_, motor_dim_) = vel_[leg];
    }

    // Index by CPG phase (or a random bin under the shuffle-phase ablation).
    int b = shuffle_phase_
            ? std::uniform_int_distribution<int>(0, n_bins_ - 1)(rng_)
            : bin_of(phi_);

    const float a = float(keyframe_alpha_);
    // Deviation of the current posture from the accumulated keyframe = the crystallization
    // signal (falls as a recurring phase-posture crystallizes; stays high if it washes out).
    // The POSTURE deviation alone feeds bin_dev_ema_/self_precision (the established gate); the
    // velocity map rides that same gate and tracks its own aggregate TLE for observability only.
    if (bin_count_[b] > 0) {
        float dev = (wb - keyframe_[b]).norm();
        keyframe_tle_ema_ = (1.0f - a) * keyframe_tle_ema_ + a * dev;
        bin_dev_ema_[b]   = (1.0f - a) * bin_dev_ema_[b]   + a * dev;
        float vdev = (wv - vel_keyframe_[b]).norm();
        vel_keyframe_tle_ema_ = (1.0f - a) * vel_keyframe_tle_ema_ + a * vdev;
    }
    // Cross-cycle EMA at matching phase (init on first visit).  Bake on RECURRENCE only.
    if (!freeze_map_) {
        if (bin_count_[b] == 0) { keyframe_[b] = wb; vel_keyframe_[b] = wv; }
        else {
            keyframe_[b]     = (1.0f - a) * keyframe_[b]     + a * wb;
            vel_keyframe_[b] = (1.0f - a) * vel_keyframe_[b] + a * wv;
        }
    }
    ++bin_count_[b];

    // Left-right symmetry prior on the VELOCITY map (anti-circling): equalize the per-joint
    // RMS push magnitude of paired legs so the propulsive pump can't grow a yaw bias that Cvel
    // would amplify into circling.  Preserves each leg's own sign/phase (magnitude-only); pulls
    // gently each tick — stable, since at equal energy the factor is 1 (a no-op).  Posture map
    // untouched, so posture-only configs stay byte-identical.
    if (vel_symmetry_gain_ > 0.0 && !symmetry_pairs_.empty()) {
        const float g = float(vel_symmetry_gain_);
        float imb_sum = 0.0f; int imb_n = 0;
        for (size_t p = 0; p + 1 < symmetry_pairs_.size(); p += 2) {
            int A = symmetry_pairs_[p], B = symmetry_pairs_[p + 1];
            for (int j = 0; j < motor_dim_; ++j) {
                double sa = 0.0, sb = 0.0; int cnt = 0;
                for (int bb = 0; bb < n_bins_; ++bb) if (bin_count_[bb] > 0) {
                    float va = vel_keyframe_[bb][A * motor_dim_ + j];
                    float vb = vel_keyframe_[bb][B * motor_dim_ + j];
                    sa += double(va) * va; sb += double(vb) * vb; ++cnt;
                }
                if (cnt == 0) continue;
                float eA = std::sqrt(float(sa / cnt)), eB = std::sqrt(float(sb / cnt));
                float e = 0.5f * (eA + eB);
                imb_sum += std::abs(eA - eB) / (eA + eB + 1e-6f); ++imb_n;
                float fA = (eA > 1e-6f) ? (1.0f + g * (e / eA - 1.0f)) : 1.0f;
                float fB = (eB > 1e-6f) ? (1.0f + g * (e / eB - 1.0f)) : 1.0f;
                for (int bb = 0; bb < n_bins_; ++bb) {
                    vel_keyframe_[bb][A * motor_dim_ + j] *= fA;
                    vel_keyframe_[bb][B * motor_dim_ + j] *= fB;
                }
            }
        }
        last_lr_imbalance_ = imb_n ? imb_sum / float(imb_n) : 0.0f;
    }

    // Publish the current bin's keyframe posture, sliced per leg, as the soft objective.
    // Confidence = base gain × SELF-precision: a smeared/unproven bin drives weakly or not at
    // all (§2.5 "drive on consistency"), so the map crystallizes on the body's own gait before
    // it takes over — the premature-drive fix, and the precision the arbiter will later scale.
    if (publish_) {
        float w = std::clamp(float(gain_) * self_precision(b), 0.0f, 1.0f);
        last_drive_w_ = w;
        for (int leg = 0; leg < nl; ++leg) {
            auto out = std::make_shared<PredictionToken>();
            out->tick_id          = tick_id;
            out->producer_id      = id_.empty() ? std::string("keyframe_gait") : id_;
            out->target_modality  = "posture." + std::to_string(leg);
            out->predicted_latent = keyframe_[b].segment(leg * motor_dim_, motor_dim_);
            out->confidence       = w;
            bus_->publish(objective_output_topics_[leg], out);
        }
        // Phase-indexed VELOCITY objective (the propulsive push): same bin, same self-precision
        // gate, so it drives only on consistent bins.  Consumed by MotorEPM's Cvel feed-forward,
        // which descends the velocity error (v*−ẋ) → keeps moving THROUGH the pose, not stopping
        // AT it.  Off when the socket is unwired (the map still accumulated above).
        if (!velocity_output_topics_.empty()) {
            for (int leg = 0; leg < nl; ++leg) {
                auto out = std::make_shared<PredictionToken>();
                out->tick_id          = tick_id;
                out->producer_id      = id_.empty() ? std::string("keyframe_gait") : id_;
                out->target_modality  = "velocity." + std::to_string(leg);
                out->predicted_latent = vel_keyframe_[b].segment(leg * motor_dim_, motor_dim_);
                out->confidence       = w;
                bus_->publish(velocity_output_topics_[leg], out);
            }
        }
    }
}

int KeyframeGait::bins_filled() const {
    int c = 0; for (int64_t n : bin_count_) if (n > 0) ++c; return c;
}

// ---- diag (live viz) ----
nlohmann::json KeyframeGait::diag_snapshot() const {
    nlohmann::json j;
    j["n_bins"]       = n_bins_;
    j["bins_filled"]  = bins_filled();
    j["keyframe_tle"]     = keyframe_tle_ema_;      // Gate 1 signal: FALLS as the posture map crystallizes
    j["vel_keyframe_tle"] = vel_keyframe_tle_ema_;  // velocity map crystallization (observability)
    j["vel_publish"]      = !velocity_output_topics_.empty();
    j["vel_symmetry_gain"] = vel_symmetry_gain_;
    j["vel_lr_imbalance"]  = last_lr_imbalance_;    // paired-leg RMS-energy imbalance (falls as the symmetry prior balances L-R)
    // Mean per-bin deviation over filled bins (lower = more cross-cycle consistent).
    float s = 0.0f; int c = 0;
    for (int b = 0; b < n_bins_; ++b) if (bin_count_[b] > 0) { s += bin_dev_ema_[b]; ++c; }
    j["mean_bin_dev"]     = c ? s / float(c) : 0.0f;
    j["mean_consistency"] = c ? 1.0f / (1.0f + s / float(c)) : 0.0f;
    // Self-precision drive gate: mean over filled bins + the last published confidence.
    float ps = 0.0f; int pc = 0;
    for (int b = 0; b < n_bins_; ++b) if (bin_count_[b] > 0) { ps += self_precision(b); ++pc; }
    j["mean_precision"] = pc ? ps / float(pc) : 0.0f;
    j["drive_w"]        = last_drive_w_;
    j["phi"]     = phi_;
    j["gain"]    = gain_;
    j["shuffle_phase"] = shuffle_phase_;
    j["freeze_map"]    = freeze_map_;
    j["publish"]       = publish_;
    return j;
}

// ---- snapshot / restore (persist the learned map) ----
nlohmann::json KeyframeGait::snapshot_state() const {
    nlohmann::json bins = nlohmann::json::array();
    for (int b = 0; b < n_bins_; ++b) {
        nlohmann::json bj;
        bj["kf"]    = std::vector<float>(keyframe_[b].data(), keyframe_[b].data() + keyframe_[b].size());
        bj["vkf"]   = std::vector<float>(vel_keyframe_[b].data(), vel_keyframe_[b].data() + vel_keyframe_[b].size());
        bj["dev"]   = bin_dev_ema_[b];
        bj["count"] = bin_count_[b];
        bins.push_back(std::move(bj));
    }
    return nlohmann::json{{"version", 2}, {"n_bins", n_bins_},
                          {"keyframe_tle", keyframe_tle_ema_},
                          {"vel_keyframe_tle", vel_keyframe_tle_ema_}, {"bins", bins}};
}

void KeyframeGait::restore_state(nlohmann::json const& s) {
    if (!s.contains("bins")) return;
    keyframe_tle_ema_     = s.value("keyframe_tle", keyframe_tle_ema_);
    vel_keyframe_tle_ema_ = s.value("vel_keyframe_tle", vel_keyframe_tle_ema_);
    auto const& bins = s.at("bins");
    for (int b = 0; b < n_bins_ && b < int(bins.size()); ++b) {
        auto const& bj = bins[b];
        if (bj.contains("kf")) {
            auto v = bj["kf"].get<std::vector<float>>();
            keyframe_[b] = Eigen::Map<const Eigen::VectorXf>(v.data(), int(v.size()));
        }
        if (bj.contains("vkf")) {   // legacy (v1) snapshots have no velocity map → keep zero-init
            auto v = bj["vkf"].get<std::vector<float>>();
            vel_keyframe_[b] = Eigen::Map<const Eigen::VectorXf>(v.data(), int(v.size()));
        }
        bin_dev_ema_[b] = bj.value("dev", bin_dev_ema_[b]);
        bin_count_[b]   = bj.value("count", bin_count_[b]);
    }
}

} // namespace ogma
