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
        TopicSpec{pose_topic_,   std::type_index(typeid(ProprioToken)),
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
        {"pose_topic", ParamMutability::ConstructionOnly,
         "12-D joint-pose ProprioToken (4 hip1 + 4 hip2 + 4 knee, normalised) used ONLY for the "
         "per-token pose readout that decodes cone rows to joint space for the piano-roll display. "
         "An instrument readout, not a percept — nothing downstream consumes it.",
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
    m["pose_topic"] = pose_topic_;
    m["horizon"] = horizon_; m["beam_k"] = beam_k_;
    m["phase_bins"] = phase_bins_; m["mask_mode"] = mask_mode_; m["mask_floor"] = mask_floor_;
    return m;
}

void MotorPlanner::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    state_topic_  = sget(params, "state_topic",  state_topic_);
    rhythm_topic_ = sget(params, "rhythm_topic", rhythm_topic_);
    pose_topic_   = sget(params, "pose_topic",   pose_topic_);
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
    subs_.push_back(bus_->subscribe(pose_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){
            auto t = std::dynamic_pointer_cast<const ProprioToken>(p);
            if (t && t->values.size() >= kJoints) {
                for (int j = 0; j < kJoints; ++j) cur_pose_[j] = t->values[j];
                pose_seen_ = true;
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

// Cone row → joint space: probability-weighted mixture over the per-token pose
// readouts.  mean = Σ p·μ_tok ; var = Σ p·(σ²_tok + μ²_tok) − mean²  (law of total
// variance — the fan width carries BOTH within-token spread and across-token
// disagreement).  Tokens with no readout yet fall back to the current pose.
void MotorPlanner::decode_row(Dist const& d, float* mean_out, float* sd_out) const {
    for (int j = 0; j < kJoints; ++j) {
        float m = 0.0f, s = 0.0f;
        for (auto const& [tok, p] : d) {
            auto it = tok_pose_.find(tok);
            if (it == tok_pose_.end() || it->second.n < 2) {
                m += p * cur_pose_[j];
                s += p * cur_pose_[j] * cur_pose_[j];
            } else {
                float mu  = it->second.mean[j];
                float var = it->second.m2[j] / float(it->second.n - 1);
                m += p * mu;
                s += p * (var + mu * mu);
            }
        }
        mean_out[j] = m;
        float v = s - m * m;
        sd_out[j] = v > 0.0f ? std::sqrt(v) : 0.0f;
    }
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
                acc_pers_[slot] += (it->tok0 == cur_tok_) ? 1.0 : 0.0;
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

    // ---- pose readout + past ring (the roll's decoder and its left half)
    if (pose_seen_) {
        auto& ps = tok_pose_[cur_tok_];
        ++ps.n;
        for (int j = 0; j < kJoints; ++j) {           // Welford
            float d1 = cur_pose_[j] - ps.mean[j];
            ps.mean[j] += d1 / float(ps.n);
            ps.m2[j]   += d1 * (cur_pose_[j] - ps.mean[j]);
        }
        past_[past_head_] = cur_pose_;
        past_head_ = (past_head_ + 1) % kPastRing;
        if (past_n_ < kPastRing) ++past_n_;
    }

    // ---- roll the cone from t0 (the reflexes' row: the actual present) outward,
    // decoding EVERY depth to joint space for the piano roll.
    const int H = int(horizon_);
    roll_mean_.assign(size_t(H) * kJoints, 0.0f);
    roll_sd_.assign(size_t(H) * kJoints, 0.0f);
    roll_len_ = 0;
    Dist row; row[cur_tok_] = 1.0f;
    int next_probe = 0;
    for (int n = 1; n <= H; ++n) {
        const int b = phase_bin(std::fmod(phi_ + float(n) * omega_, kTwoPi));
        row = propagate(row, b);
        apply_mask(row, b);
        if (row.empty()) break;
        decode_row(row, &roll_mean_[size_t(n - 1) * kJoints],
                        &roll_sd_[size_t(n - 1) * kJoints]);
        roll_len_ = n;
        if (next_probe < kNumProbes && n == kProbes[next_probe]) {
            pending_.push_back(Pending{tick_id + uint64_t(n), n, cur_tok_, row});
            ++next_probe;
        }
    }
    if (pending_.size() > 512) pending_.erase(pending_.begin(), pending_.begin() + 128);
}

nlohmann::json MotorPlanner::snapshot_state() const {
    nlohmann::json mod;
    nlohmann::json d = nlohmann::json::array(), t1 = nlohmann::json::array(),
                   tk = nlohmann::json::array(), ms = nlohmann::json::array(),
                   en = nlohmann::json::array(), nn = nlohmann::json::array(),
                   pr = nlohmann::json::array();
    for (int i = 0; i < kNumProbes; ++i) {
        d.push_back(kProbes[i]);
        double n = double(std::max<long>(1, acc_n_[i]));
        t1.push_back(acc_top1_[i] / n); tk.push_back(acc_topk_[i] / n);
        ms.push_back(acc_mass_[i] / n); en.push_back(acc_ent_[i] / n);
        pr.push_back(acc_pers_[i] / n);
        nn.push_back(acc_n_[i]);
    }
    mod["probe_depths"] = d; mod["cone_top1"] = t1; mod["cone_topk"] = tk;
    mod["cone_mass"] = ms; mod["cone_entropy"] = en; mod["cone_n"] = nn;
    mod["cone_persist"] = pr;
    mod["marg_top1"] = marg_n_ ? marg_top1_ / double(marg_n_) : 0.0;
    mod["n_obs"] = n_obs_; mod["masked_out"] = masked_out_;
    mod["mask_mode"] = mask_mode_;
    // THE AUTHORITY DEPTH — the deepest probe (in ticks) such that every probe up
    // to it has enough verdicts (n≥200) and the cone's argmax beats the persistence
    // baseline scored under the identical pending protocol, with a 5% margin.
    // 0 = the planner has earned no authority anywhere; reflexes own the roll.
    int authority = 0;
    for (int i = 0; i < kNumProbes; ++i) {
        if (acc_n_[i] < 200) break;
        double n = double(acc_n_[i]);
        if (acc_top1_[i] / n <= 1.05 * (acc_pers_[i] / n)) break;
        authority = kProbes[i];
    }
    mod["authority_depth"] = authority;
    return nlohmann::json{{"version", 1}, {"module", mod}};
}

void MotorPlanner::restore_state(nlohmann::json const&) {}

nlohmann::json MotorPlanner::diag_snapshot() const {
    // The piano-roll payload: everything snapshot_state carries, plus this tick's
    // decoded cone (roll_mean/roll_sd, roll_len × 12) and the past-pose ring —
    // shipped WHOLE, not accumulated client-side (DiagPublisher throttles to the
    // subscription hz; a client accumulator would alias, the gait-raster lesson).
    nlohmann::json mod = snapshot_state()["module"];
    mod["joints"]   = kJoints;
    mod["horizon"]  = int(horizon_);
    mod["roll_len"] = roll_len_;
    nlohmann::json rm = nlohmann::json::array(), rs = nlohmann::json::array();
    for (int i = 0; i < roll_len_ * kJoints; ++i) {
        rm.push_back(std::round(roll_mean_[i] * 1000.0f) / 1000.0f);
        rs.push_back(std::round(roll_sd_[i]   * 1000.0f) / 1000.0f);
    }
    mod["roll_mean"] = std::move(rm);
    mod["roll_sd"]   = std::move(rs);
    nlohmann::json past = nlohmann::json::array();
    for (int i = 0; i < past_n_; ++i) {           // oldest → newest
        int idx = (past_head_ - past_n_ + i + kPastRing) % kPastRing;
        for (int j = 0; j < kJoints; ++j)
            past.push_back(std::round(past_[idx][j] * 1000.0f) / 1000.0f);
    }
    mod["past"]     = std::move(past);
    mod["past_len"] = past_n_;
    mod["cur_tok"]  = cur_tok_;
    mod["phi"]      = std::round(phi_ * 1000.0f) / 1000.0f;
    mod["omega"]    = omega_;
    return mod;
}

} // namespace ogma
