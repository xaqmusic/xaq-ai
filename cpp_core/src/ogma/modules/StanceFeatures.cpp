// StanceFeatures -- see the header.
#include "ogma/modules/StanceFeatures.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

namespace ogma {
namespace {
template <typename Fn> void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key); if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("StanceFeatures: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("StanceFeatures: param '" + k + "' must be a string");
}
} // namespace

StanceFeatures::StanceFeatures() = default;
StanceFeatures::~StanceFeatures() = default;
std::string_view StanceFeatures::type_name() const { return "StanceFeatures"; }

std::vector<TopicSpec> StanceFeatures::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : {model_topic_, pose_topic_, load_topic_, contact_topic_, up_topic_, patch_topic_})
        if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> StanceFeatures::output_topics() const {
    std::vector<TopicSpec> v{TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))}};
    if (!obs_topic_.empty()) v.push_back(TopicSpec{obs_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema StanceFeatures::params_schema() const {
    return {
        {"model_topic",   ParamMutability::ConstructionOnly, "ProprioToken [116]: the body's leg model (the stepper's LegSolver is built from it).", ParamValue{std::string("reality.proprio.leg_model")}},
        {"pose_topic",    ParamMutability::ConstructionOnly, "ProprioToken [engage, u x 12]: the stepper's commanded pose (PoseSequencer output_topic).", ParamValue{std::string("pose.targets")}},
        {"load_topic",    ParamMutability::ConstructionOnly, "ProprioToken [4]: the FSR load shares.", ParamValue{std::string("reality.proprio.foot_load")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4]: the contact flags.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"up_topic",      ParamMutability::ConstructionOnly, "ProprioToken [3]: the body's gravity-up in the body frame.", ParamValue{std::string("reality.proprio.up_body")}},
        {"patch_topic",   ParamMutability::ConstructionOnly, "ProprioToken: TerrainMemory's patch (the remembered ground around each toe).", ParamValue{std::string("terrain.patch")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "ProprioToken [43]: the conditioned stance (z, clipped) plus one constant component sqrt(42) -- the stance EPM's input (jl_state).", ParamValue{std::string("reality.proprio.stance_features")}},
        {"obs_topic",     ParamMutability::ConstructionOnly, "ProprioToken [42]: the conditioned stance alone -- the BeliefFilter's observation. \"\" = not published.", ParamValue{std::string("reality.proprio.stance_obs")}},
        {"engage_min",    ParamMutability::HotMutable, "Features are taken only while the stepper's engage is at least this (the stance is the stepper's).", ParamValue{0.99}},
        {"n_max",         ParamMutability::HotMutable, "The running statistics' rate anneals as 1 / n down to 1 / n_max (samples). 0 = no floor: a pure 1 / n, the statistics of every sample ever seen, which settle and hold across carried runs (Phase 3 continued, 2026-11-15).", ParamValue{int64_t{20000}}},
        {"freeze_after",  ParamMutability::HotMutable, "THE FROZEN ENCODER: after this many samples the running statistics stop updating, so the stance EPM's coordinates hold still across worlds (the EPM contract's frozen encoder). The count rides in the snapshot, so a carried state stays frozen. 0 = off (byte-identical). Measured 2026-11-15: a pure 1 / n absorbed six stairs runs (mean 0.4 sigma, sigma x1.5) and re-encoded the corridor.", ParamValue{int64_t{0}}},
        {"warmup",        ParamMutability::HotMutable, "Nothing is published for the first this many samples (the statistics are not yet a scale).", ParamValue{int64_t{200}}},
        {"clip",          ParamMutability::HotMutable, "Each z is clipped to +- this.", ParamValue{4.0}},
        {"widen",         ParamMutability::HotMutable, "THE SCALE MAY ONLY WIDEN (with freeze_after): once frozen, a feature kind (ext, knee to straight, knee to fold, hip1, hip2, lift, load, contact, ground, relief over the four legs; pitch; roll) whose sample reaches the clip has its scale stretched by 1 / (ticks at the clip so far + warmup) of itself, all four legs alike; the centre stays frozen. Self-limiting: it stops when the excursion fits inside the clip, so the scale ends at the kind's range and holds. A step's size does not depend on the excursion's, so a glitch moves it half a percent. 0 = off (byte-identical). Measured 2026-11-16: the scale frozen in flat worlds held the ground at 2 mm and the pitch at 1 degree, and a riser clipped them.", ParamValue{0.0}},
        {"flicker_p",     ParamMutability::HotMutable, "THE FLICKER TEST: each contact flag is flipped with this probability per tick before it is featurised; 0 = off (byte-identical).", ParamValue{0.0}},
        {"flicker_seed",  ParamMutability::ConstructionOnly, "The flicker's random stream.", ParamValue{int64_t{1}}},
    };
}

ParamMap StanceFeatures::current_params() const {
    return {{"model_topic", ParamValue{model_topic_}}, {"pose_topic", ParamValue{pose_topic_}}, {"load_topic", ParamValue{load_topic_}},
            {"contact_topic", ParamValue{contact_topic_}}, {"up_topic", ParamValue{up_topic_}}, {"patch_topic", ParamValue{patch_topic_}},
            {"output_topic", ParamValue{output_topic_}}, {"obs_topic", ParamValue{obs_topic_}}, {"engage_min", ParamValue{engage_min_}},
            {"n_max", ParamValue{int64_t(n_max_)}}, {"freeze_after", ParamValue{int64_t(freeze_after_)}}, {"warmup", ParamValue{int64_t(warmup_)}}, {"clip", ParamValue{clip_}}, {"widen", ParamValue{widen_}},
            {"flicker_p", ParamValue{flicker_p_}}, {"flicker_seed", ParamValue{int64_t(flicker_seed_)}}};
}

void StanceFeatures::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("StanceFeatures requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"model_topic", &model_topic_}, {"pose_topic", &pose_topic_}, {"load_topic", &load_topic_}, {"contact_topic", &contact_topic_},
            {"up_topic", &up_topic_}, {"patch_topic", &patch_topic_}, {"output_topic", &output_topic_}, {"obs_topic", &obs_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    apply_param(params, "flicker_seed", [&](auto const& v){ flicker_seed_ = uint32_t(get_double(v, "flicker_seed")); });
    rng_ = flicker_seed_ * 2654435761u + 1u;
    for (auto const& k : {"engage_min", "n_max", "freeze_after", "warmup", "clip", "widen", "flicker_p"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which) {
        if (topic.empty()) return;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which](std::string_view, MessagePtr p){ handle(which, p); }));
    };
    sub(model_topic_, "model"); sub(pose_topic_, "pose"); sub(load_topic_, "load"); sub(contact_topic_, "contact"); sub(up_topic_, "up"); sub(patch_topic_, "patch");
}

void StanceFeatures::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "engage_min") engage_min_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "n_max")      n_max_ = std::max(0, int(get_double(value, k)));
    else if (k == "freeze_after") freeze_after_ = std::max<int64_t>(0, int64_t(get_double(value, k)));
    else if (k == "warmup")     warmup_ = std::max(0, int(get_double(value, k)));
    else if (k == "clip")       clip_ = std::max(0.5, get_double(value, k));
    else if (k == "widen")      widen_ = std::max(0.0, get_double(value, k));
    else if (k == "flicker_p")  flicker_p_ = std::clamp(get_double(value, k), 0.0, 1.0);
}

void StanceFeatures::handle(std::string const& which, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if (which == "model") kin_.set_model(v.data(), std::size_t(v.size()));
    else if (which == "pose" && v.size() >= 13) { for (int i = 0; i < 13; ++i) pose_[size_t(i)] = v[i]; have_pose_ = true; }
    else if (which == "load" && v.size() >= 4) { for (int i = 0; i < 4; ++i) load_[size_t(i)] = v[i]; have_load_ = true; }
    else if (which == "contact" && v.size() >= 4) { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = v[i]; have_contact_ = true; }
    else if (which == "up" && v.size() >= 3) {
        const double n = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]);
        if (n > 0.5) { up_ = {v[0] / n, v[1] / n, v[2] / n}; have_up_ = true; }
    }
    else if (which == "patch" && v.size() >= 4) {
        const int N = int(v[0]); if (N < 2 || v.size() < 4 + 2 * N * N) return;
        patch_.N_ = N; patch_.cell_ = std::max(1e-3, double(v[1])); patch_.patch_.assign(v.data(), v.data() + 4 + 2 * N * N); patch_.have_patch_ = true;
    }
}

bool StanceFeatures::raw(Eigen::VectorXd& x) const {
    if (!kin_.ready() || !have_pose_) return false;
    x = Eigen::VectorXd::Zero(kRaw);
    std::array<body::Vec3f, 4> toe{}; std::array<std::array<double, 3>, 4> ang{};
    for (int l = 0; l < 4; ++l) {
        ang[size_t(l)] = kin_.angles_of_u(l, {pose_[size_t(1 + 3 * l)], pose_[size_t(2 + 3 * l)], pose_[size_t(3 + 3 * l)]});
        toe[size_t(l)] = kin_.toe_of(l, ang[size_t(l)]);
    }
    double gh[4]; bool gok[4]; double gsum = 0.0; int gn = 0;
    for (int l = 0; l < 4; ++l) { gok[l] = patch_.patch_h(toe[size_t(l)].z, toe[size_t(l)].x, gh[l]); if (gok[l]) { gsum += gh[l]; ++gn; } }
    for (int l = 0; l < 4; ++l) {
        const int b = 10 * l;
        const auto r = kin_.room(l, ang[size_t(l)]);
        x[b + 0] = r.ext_m; x[b + 1] = r.knee_straight; x[b + 2] = r.knee_fold; x[b + 3] = r.hip1; x[b + 4] = r.hip2;
        double oy = 0.0; for (int j = 0; j < 4; ++j) if (j != l) oy += toe[size_t(j)].y / 3.0;
        x[b + 5] = double(toe[size_t(l)].y) - oy;
        x[b + 6] = have_load_ ? double(load_[size_t(l)]) : 0.0;
        x[b + 7] = have_contact_ ? (((contact_[size_t(l)] > 0.5f) != flip_[size_t(l)]) ? 1.0 : 0.0) : 0.0;
        x[b + 8] = (gok[l] && gn >= 2) ? gh[l] - gsum / gn : 0.0;
        double rel = 0.0; patch_.patch_edge(toe[size_t(l)].z, toe[size_t(l)].x, rel); x[b + 9] = rel;
    }
    x[40] = std::atan2(up_[2], std::max(0.2, up_[1]));   // pitch, nose-up positive (the stepper's front drop reads the same)
    x[41] = std::atan2(up_[0], std::max(0.2, up_[1]));   // roll
    return true;
}

void StanceFeatures::tick(uint64_t tick_id) {
    if (flicker_p_ > 0.0 && have_contact_) {   // the flicker test: flags flipped where they are featurised (the stored reading kept)
        for (int l = 0; l < 4; ++l) {
            rng_ = rng_ * 1664525u + 1013904223u;
            flip_[size_t(l)] = double(rng_ >> 8) / 16777216.0 < flicker_p_;
            if (flip_[size_t(l)]) ++flips_;
        }
    }
    if (!have_pose_ || double(pose_[0]) < engage_min_) return;
    Eigen::VectorXd x; if (!raw(x)) return;
    ++n_;
    const double a = n_max_ > 0 ? std::max(1.0 / double(n_), 1.0 / double(n_max_)) : 1.0 / double(n_);
    if (n_ == 1) { mean_ = x; var_.setZero(); }
    else if (freeze_after_ > 0 && n_ > freeze_after_) {   // frozen: the coordinates hold still
        if (widen_ > 0.0) {   // ...except a kind whose samples reach the clip: its scale stretches, the centre stays
            for (int k = 0; k < kKinds; ++k) {
                const int i0 = k < 10 ? k : 40 + (k - 10), step = k < 10 ? 10 : 1, cnt = k < 10 ? 4 : 1;
                bool at_clip = false;
                for (int j = 0; j < cnt; ++j) { const int i = i0 + j * step; if (std::abs(x[i] - mean_[i]) / (std::sqrt(var_[i]) + 1e-4) >= clip_) at_clip = true; }
                if (!at_clip) continue;
                const double f = 1.0 + 1.0 / double(++widen_n_[size_t(k)] + warmup_);   // count-annealed: one frozen sd per `warmup` ticks at the clip
                for (int j = 0; j < cnt; ++j) { const int i = i0 + j * step; const double sd = (std::sqrt(var_[i]) + 1e-4) * f - 1e-4; var_[i] = sd * sd; }
            }
        }
    }
    else {
        const Eigen::VectorXd d = x - mean_;
        mean_ += a * d;
        var_ = (1.0 - a) * (var_ + a * d.cwiseProduct(d));
    }
    if (n_ <= warmup_) return;
    const Eigen::ArrayXd zr = (x - mean_).array() / (var_.array().sqrt() + 1e-4);   // before the clip: what the scale failed to hold
    const Eigen::VectorXd z = zr.max(-clip_).min(clip_).matrix();
    last_z_ = z; ++published_;
    {   // the saturation instrument (always on, deciding nothing): per feature kind, the samples at the clip and the largest |z|
        ++sat_samples_; double s2 = 0.0;
        for (int i = 0; i < kRaw; ++i) {
            const size_t k = size_t(i < 40 ? i % 10 : 10 + (i - 40)); const double az = std::abs(zr[i]);
            if (az >= clip_) ++sat_n_[k];
            if (az > sat_zmax_[k]) sat_zmax_[k] = az;
            s2 += zr[i] * zr[i];
        }
        last_zr_norm_ = std::sqrt(s2);
    }
    const std::string pid = id().empty() ? std::string("stance_features") : std::string(id());
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = pid; out->sensor = "stance_features";
    out->values = Eigen::VectorXf(kRaw + 1);
    for (int i = 0; i < kRaw; ++i) out->values[i] = float(z[i]);
    out->values[kRaw] = float(std::sqrt(double(kRaw)));
    bus_->publish(output_topic_, out);
    if (!obs_topic_.empty()) {
        auto ob = std::make_shared<ProprioToken>();
        ob->tick_id = tick_id; ob->producer_id = pid; ob->sensor = "stance_obs";
        ob->values = z.cast<float>();
        bus_->publish(obs_topic_, ob);
    }
}

nlohmann::json StanceFeatures::diag_snapshot() const {
    return nlohmann::json{{"samples", n_}, {"published", published_}, {"flips", flips_}, {"ready", kin_.ready() && have_pose_},
                          {"z_norm", last_z_.norm()}, {"frozen", freeze_after_ > 0 && n_ > freeze_after_},
                          {"zr_norm", last_zr_norm_}, {"sat_samples", sat_samples_}, {"sat_n", sat_n_}, {"sat_zmax", sat_zmax_}, {"widen_n", widen_n_}};
}

nlohmann::json StanceFeatures::snapshot_state() const {
    nlohmann::json j{{"version", 1}, {"n", n_}, {"published", published_}, {"flips", flips_}, {"rng", rng_},
                     {"mean", std::vector<double>(mean_.data(), mean_.data() + mean_.size())},
                     {"var", std::vector<double>(var_.data(), var_.data() + var_.size())}};
    if (widen_ > 0.0) j["widen_n"] = widen_n_;   // only when the lever is on: a state saved with it off is unchanged
    return j;
}

void StanceFeatures::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    if (s.value("version", 0) != 1) throw std::invalid_argument("StanceFeatures::restore_state: unknown version");
    const auto m = s.at("mean").get<std::vector<double>>(), v = s.at("var").get<std::vector<double>>();
    if (int(m.size()) != kRaw || int(v.size()) != kRaw) throw std::invalid_argument("StanceFeatures::restore_state: width mismatch");
    for (int i = 0; i < kRaw; ++i) { mean_[i] = m[size_t(i)]; var_[i] = v[size_t(i)]; }
    if (s.contains("widen_n")) { const auto w = s.at("widen_n").get<std::vector<int64_t>>(); if (int(w.size()) == kKinds) for (int k = 0; k < kKinds; ++k) widen_n_[size_t(k)] = w[size_t(k)]; }
    n_ = s.at("n").get<int64_t>(); published_ = s.value("published", int64_t{0}); flips_ = s.value("flips", int64_t{0}); rng_ = s.value("rng", rng_);
}

} // namespace ogma
