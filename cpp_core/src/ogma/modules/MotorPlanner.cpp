// MotorPlanner v1 — the probability cone as a shadow instrument.  See the header.
#include "ogma/modules/MotorPlanner.hpp"
#include "ogma/Topics.hpp"
#include "ogma/Bus.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>

namespace ogma {

namespace {
double pget(ParamMap const& p, std::string const& k, double dflt) {
    auto it = p.find(k);
    if (it == p.end()) return dflt;
    if (auto* d = std::get_if<double>(&it->second)) return *d;
    if (auto* i = std::get_if<int64_t>(&it->second)) return double(*i);
    return dflt;
}
std::string sget(ParamMap const& p, std::string const& k, std::string dflt) {
    auto it = p.find(k);
    if (it == p.end()) return dflt;
    if (auto* s = std::get_if<std::string>(&it->second)) return *s;
    return dflt;
}
constexpr float kTwoPi = 6.28318530718f;
} // namespace

constexpr std::array<int, MotorPlanner::kNumProbes> MotorPlanner::kProbes;

std::vector<TopicSpec> MotorPlanner::input_topics() const {
    return {
        TopicSpec{state_topic_,  std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{rhythm_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

ParamSchema MotorPlanner::params_schema() const {
    return {
        {"state_topic", ParamMutability::ConstructionOnly,
         "RealityToken stream whose winner_id is the body-state vocabulary the cone is built over "
         "(default reality.bodypose.pose).", std::nullopt, std::nullopt, std::nullopt},
        {"rhythm_topic", ParamMutability::ConstructionOnly,
         "ProprioToken [cos φ, sin φ, ω] body-rhythm reference; the cone's transition model is "
         "PHASE-CONDITIONED on it (the M0 diagnosis: unconditioned chains degenerate to persistence).",
         std::nullopt, std::nullopt, std::nullopt},
        {"horizon", ParamMutability::HotMutable,
         "Cone depth in ticks.", ParamValue{40.0}, ParamValue{4.0}, ParamValue{120.0}},
        {"beam_k", ParamMutability::HotMutable,
         "Per-row sparsity cap (top-K tokens kept per depth).", ParamValue{12.0}, ParamValue{2.0}, ParamValue{64.0}},
        {"phase_bins", ParamMutability::ConstructionOnly,
         "Phase bins conditioning the transition model.", ParamValue{8.0}, ParamValue{2.0}, ParamValue{16.0}},
        {"mask_mode", ParamMutability::HotMutable,
         "Cone masking experiment: 0 = none (baseline cone); 1 = PHASE-AFFINITY mask — zero mass on "
         "tokens whose learned phase occupancy at the target bin is below mask_floor (inhibition as "
         "precision withdrawal; the operator's E/I proposal, measured on cone quality only — v1 has "
         "no behavioral authority).", ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"mask_floor", ParamMutability::HotMutable,
         "Affinity floor for mask_mode 1.", ParamValue{0.02}, ParamValue{0.0}, ParamValue{0.5}},
    };
}

ParamMap MotorPlanner::current_params() const {
    ParamMap m;
    m["state_topic"] = state_topic_; m["rhythm_topic"] = rhythm_topic_;
    m["horizon"] = horizon_; m["beam_k"] = beam_k_;
    m["phase_bins"] = phase_bins_; m["mask_mode"] = mask_mode_; m["mask_floor"] = mask_floor_;
    return m;
}

void MotorPlanner::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    state_topic_  = sget(params, "state_topic",  state_topic_);
    rhythm_topic_ = sget(params, "rhythm_topic", rhythm_topic_);
    horizon_    = pget(params, "horizon",    horizon_);
    beam_k_     = pget(params, "beam_k",     beam_k_);
    phase_bins_ = std::clamp(pget(params, "phase_bins", phase_bins_), 2.0, double(kMaxBins));
    mask_mode_  = pget(params, "mask_mode",  mask_mode_);
    mask_floor_ = pget(params, "mask_floor", mask_floor_);
    subs_.push_back(bus_->subscribe(state_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){
            if (auto t = std::dynamic_pointer_cast<const RealityToken>(p))
                cur_tok_ = t->winner_id;
        }));
    subs_.push_back(bus_->subscribe(rhythm_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){
            auto t = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (t && t->values.size() >= 2) {
                float phi = std::atan2(t->values[1], t->values[0]);
                if (phi < 0.0f) phi += kTwoPi;
                if (t->values.size() >= 3 && t->values[2] > 1e-4f) {
                    omega_ = t->values[2];               // reference carries its own rate
                } else if (phi_seen_) {
                    // 2-D clock (e.g. rhythm.cpg.body): estimate the rate from the
                    // clock's own advance — forward steps only, EMA-smoothed.
                    float dphi = phi - phi_;
                    if (dphi < -3.14159265f) dphi += kTwoPi;
                    if (dphi >  3.14159265f) dphi -= kTwoPi;
                    if (dphi > 1e-5f && dphi < 1.0f)
                        omega_ += 0.02f * (dphi - omega_);
                }
                phi_ = phi;
                phi_seen_ = true;
            }
        }));
}

void MotorPlanner::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    auto d = [&](double dflt){ if (auto* v = std::get_if<double>(&value)) return *v; return dflt; };
    if      (k == "horizon")    horizon_    = d(horizon_);
    else if (k == "beam_k")     beam_k_     = d(beam_k_);
    else if (k == "mask_mode")  mask_mode_  = d(mask_mode_);
    else if (k == "mask_floor") mask_floor_ = d(mask_floor_);
}

int MotorPlanner::phase_bin(float phi) const {
    int b = int(phi / kTwoPi * float(phase_bins_));
    return std::clamp(b, 0, int(phase_bins_) - 1);
}

void MotorPlanner::apply_mask(Dist& d, int bin) const {
    if (mask_mode_ < 0.5) return;
    for (auto it = d.begin(); it != d.end(); ) {
        auto tp = tok_phase_.find(it->first);
        float aff = 0.0f;
        if (tp != tok_phase_.end()) {
            float tot = 0.0f;
            for (int b = 0; b < int(phase_bins_); ++b) tot += tp->second[b];
            if (tot > 0.0f) aff = tp->second[bin] / tot;
        }
        if (aff < float(mask_floor_)) { it = d.erase(it); ++const_cast<long&>(masked_out_); }
        else ++it;
    }
    float tot = 0.0f;
    for (auto const& kv : d) tot += kv.second;
    if (tot > 1e-9f) for (auto& kv : d) kv.second /= tot;
}

MotorPlanner::Dist MotorPlanner::propagate(Dist const& d, int bin) const {
    Dist out;
    for (auto const& [tok, p] : d) {
        auto it = trans_.find(tok * kMaxBins + bin);
        if (it == trans_.end()) { out[tok] += p; continue; }   // no data: hold state
        float tot = 0.0f;
        for (auto const& kv : it->second) tot += kv.second;
        if (tot <= 0.0f) { out[tok] += p; continue; }
        for (auto const& [nxt, c] : it->second) out[nxt] += p * (c / tot);
    }
    // top-K truncation + renormalize (the beam; the mask acts before truncation)
    if (int(out.size()) > int(beam_k_)) {
        std::vector<std::pair<int, float>> v(out.begin(), out.end());
        std::nth_element(v.begin(), v.begin() + int(beam_k_), v.end(),
                         [](auto const& a, auto const& b){ return a.second > b.second; });
        v.resize(size_t(beam_k_));
        out.clear();
        for (auto const& kv : v) out.insert(kv);
    }
    float tot = 0.0f;
    for (auto const& kv : out) tot += kv.second;
    if (tot > 1e-9f) for (auto& kv : out) kv.second /= tot;
    return out;
}

void MotorPlanner::tick(uint64_t tick_id) {
    if (cur_tok_ < 0 || !phi_seen_) return;
    const int bin = phase_bin(phi_);

    // ---- verify: score any pending predictions that are due NOW (before learning,
    // so the scored model never saw this tick).
    for (auto it = pending_.begin(); it != pending_.end(); ) {
        if (it->due == tick_id) {
            int slot = -1;
            for (int i = 0; i < kNumProbes; ++i) if (kProbes[i] == it->depth) slot = i;
            if (slot >= 0 && !it->dist.empty()) {
                int argmax = -1; float best = -1.0f, ent = 0.0f, mass = 0.0f;
                for (auto const& [tok, p] : it->dist) {
                    if (p > best) { best = p; argmax = tok; }
                    if (p > 1e-9f) ent -= p * std::log(p);
                    if (tok == cur_tok_) mass = p;
                }
                acc_top1_[slot] += (argmax == cur_tok_) ? 1.0 : 0.0;
                acc_topk_[slot] += (mass > 0.0f) ? 1.0 : 0.0;
                acc_mass_[slot] += mass;
                acc_ent_[slot]  += ent;
                ++acc_n_[slot];
            }
            it = pending_.erase(it);
        } else ++it;
    }
    // marginal baseline at depth 1
    if (marg_n_ > 0 && !marginal_.empty()) {
        int mtok = -1; float mb = -1.0f;
        for (auto const& kv : marginal_) if (kv.second > mb) { mb = kv.second; mtok = kv.first; }
        marg_top1_ += (mtok == cur_tok_) ? 1.0 : 0.0;
    }
    ++marg_n_;
    marginal_[cur_tok_] += 1.0f;

    // ---- learn: per-tick phase-conditioned transition from the PREVIOUS observation.
    static thread_local int prev_tok = -1; static thread_local int prev_bin = 0;
    if (prev_tok >= 0) { trans_[prev_tok * kMaxBins + prev_bin][cur_tok_] += 1.0f; ++n_obs_; }
    tok_phase_[cur_tok_][bin] += 1.0f;
    prev_tok = cur_tok_; prev_bin = bin;

    // ---- roll the cone from t0 (the reflexes' row: the actual present) outward.
    Dist row; row[cur_tok_] = 1.0f;
    int next_probe = 0;
    for (int n = 1; n <= int(horizon_) && next_probe < kNumProbes; ++n) {
        const int b = phase_bin(std::fmod(phi_ + float(n) * omega_, kTwoPi));
        row = propagate(row, b);
        apply_mask(row, b);
        if (row.empty()) break;
        if (n == kProbes[next_probe]) {
            pending_.push_back(Pending{tick_id + uint64_t(n), n, row});
            ++next_probe;
        }
    }
    if (pending_.size() > 512) pending_.erase(pending_.begin(), pending_.begin() + 128);
}

nlohmann::json MotorPlanner::snapshot_state() const {
    nlohmann::json mod;
    nlohmann::json d = nlohmann::json::array(), t1 = nlohmann::json::array(),
                   tk = nlohmann::json::array(), ms = nlohmann::json::array(),
                   en = nlohmann::json::array(), nn = nlohmann::json::array();
    for (int i = 0; i < kNumProbes; ++i) {
        d.push_back(kProbes[i]);
        double n = double(std::max<long>(1, acc_n_[i]));
        t1.push_back(acc_top1_[i] / n); tk.push_back(acc_topk_[i] / n);
        ms.push_back(acc_mass_[i] / n); en.push_back(acc_ent_[i] / n);
        nn.push_back(acc_n_[i]);
    }
    mod["probe_depths"] = d; mod["cone_top1"] = t1; mod["cone_topk"] = tk;
    mod["cone_mass"] = ms; mod["cone_entropy"] = en; mod["cone_n"] = nn;
    mod["marg_top1"] = marg_n_ ? marg_top1_ / double(marg_n_) : 0.0;
    mod["n_obs"] = n_obs_; mod["masked_out"] = masked_out_;
    mod["mask_mode"] = mask_mode_;
    return nlohmann::json{{"version", 1}, {"module", mod}};
}

void MotorPlanner::restore_state(nlohmann::json const&) {}

nlohmann::json MotorPlanner::diag_snapshot() const { return snapshot_state()["module"]; }

} // namespace ogma
