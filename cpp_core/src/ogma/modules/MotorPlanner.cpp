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
        {"mask_joint", ParamMutability::HotMutable,
         "mode 2: joint whose READOUT pose is tested (0..11), or -1 = all joints.",
         ParamValue{-1.0}, ParamValue{-1.0}, ParamValue{11.0}},
        {"mask_val_lo", ParamMutability::HotMutable,
         "mode 2: inhibited pose-value range, low edge.", ParamValue{0.0}, ParamValue{-1.5}, ParamValue{1.5}},
        {"mask_val_hi", ParamMutability::HotMutable,
         "mode 2: inhibited pose-value range, high edge.", ParamValue{0.0}, ParamValue{-1.5}, ParamValue{1.5}},
        {"mask_depth_lo", ParamMutability::HotMutable,
         "mode 2: first cone depth (ticks ahead) the mask acts on.", ParamValue{1.0}, ParamValue{1.0}, ParamValue{120.0}},
        {"mask_depth_hi", ParamMutability::HotMutable,
         "mode 2: last cone depth the mask acts on.", ParamValue{40.0}, ParamValue{1.0}, ParamValue{120.0}},
        {"mask_strength", ParamMutability::HotMutable,
         "mode 2: suppression factor (1 = full inhibition of matching mass).",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"author_mode", ParamMutability::HotMutable,
         "M1 MASK AUTHOR: 0 = off (inert, byte-identical); 1 = the authoring slow loop OWNS the "
         "region-mask params — proposes candidate masks (residual-guided + random), trials each for "
         "author_period ticks, and KEEPS only candidates whose verified final-vs-raw error ratio "
         "beats author_keep_ratio.  Requires mask_mode=2.  Kept masks are recorded, not re-applied (v1).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"author_period", ParamMutability::HotMutable,
         "Ticks per candidate trial.", ParamValue{800.0}, ParamValue{200.0}, ParamValue{6000.0}},
        {"author_warmup", ParamMutability::HotMutable,
         "Ticks before the first trial (model + residual tracker need material).",
         ParamValue{3000.0}, ParamValue{500.0}, ParamValue{20000.0}},
        {"author_min_n", ParamMutability::HotMutable,
         "Minimum per-slot trial verdicts before a slot contributes to the judgment.",
         ParamValue{300.0}, ParamValue{50.0}, ParamValue{5000.0}},
        {"author_keep_ratio", ParamMutability::HotMutable,
         "Keep threshold on the SCORING final/raw verified-error ratio (0.95 = the mask must "
         "reduce error ≥5%, mirroring the authority-band margin).",
         ParamValue{0.95}, ParamValue{0.5}, ParamValue{1.0}},
        {"author_score", ParamMutability::HotMutable,
         "Scoring altitude: 0 = whole-body ratio (v1.0 — measured near-null: dilution over 12 "
         "joints hides target-side wins); 1 = TARGET-JOINT ratio with the whole-body ratio as a "
         "no-damage guard (author_guard) — the ledger's re-use context: verified material lives "
         "in the per-joint marginals.", ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"author_guard", ParamMutability::HotMutable,
         "score=1: the whole-body final/raw ratio must stay below this (1.0 = the mask may not "
         "damage the body's roll at all, point-estimate).",
         ParamValue{1.0}, ParamValue{0.8}, ParamValue{1.2}},
        {"author_depth_min", ParamMutability::HotMutable,
         "Shallowest cone depth the author may mask (ledger re-use context: k<5 is reflex territory).",
         ParamValue{5.0}, ParamValue{1.0}, ParamValue{40.0}},
        {"author_depth_max", ParamMutability::HotMutable,
         "Deepest cone depth the author may mask.", ParamValue{34.0}, ParamValue{5.0}, ParamValue{120.0}},
        {"author_max_kept", ParamMutability::HotMutable,
         "Capacity of the earned-mask set (worst evicted).", ParamValue{8.0}, ParamValue{1.0}, ParamValue{32.0}},
        {"seed", ParamMutability::ConstructionOnly,
         "Author RNG seed (rewritten per-run by the OGMA_SEED master override, like every 'seed' param).",
         ParamValue{1234.0}, std::nullopt, std::nullopt},
    };
}

ParamMap MotorPlanner::current_params() const {
    ParamMap m;
    m["state_topic"] = state_topic_; m["rhythm_topic"] = rhythm_topic_;
    m["pose_topic"] = pose_topic_;
    m["horizon"] = horizon_; m["beam_k"] = beam_k_;
    m["phase_bins"] = phase_bins_; m["mask_mode"] = mask_mode_; m["mask_floor"] = mask_floor_;
    m["mask_joint"] = mask_joint_; m["mask_val_lo"] = mask_val_lo_; m["mask_val_hi"] = mask_val_hi_;
    m["mask_depth_lo"] = mask_depth_lo_; m["mask_depth_hi"] = mask_depth_hi_;
    m["mask_strength"] = mask_strength_;
    m["author_mode"] = author_mode_; m["author_period"] = author_period_;
    m["author_warmup"] = author_warmup_; m["author_min_n"] = author_min_n_;
    m["author_keep_ratio"] = author_keep_ratio_;
    m["author_score"] = author_score_; m["author_guard"] = author_guard_;
    m["author_depth_min"] = author_depth_min_; m["author_depth_max"] = author_depth_max_;
    m["author_max_kept"] = author_max_kept_; m["seed"] = seed_;
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
    mask_joint_    = pget(params, "mask_joint",    mask_joint_);
    mask_val_lo_   = pget(params, "mask_val_lo",   mask_val_lo_);
    mask_val_hi_   = pget(params, "mask_val_hi",   mask_val_hi_);
    mask_depth_lo_ = pget(params, "mask_depth_lo", mask_depth_lo_);
    mask_depth_hi_ = pget(params, "mask_depth_hi", mask_depth_hi_);
    mask_strength_ = pget(params, "mask_strength", mask_strength_);
    author_mode_       = pget(params, "author_mode",       author_mode_);
    author_period_     = pget(params, "author_period",     author_period_);
    author_warmup_     = pget(params, "author_warmup",     author_warmup_);
    author_min_n_      = pget(params, "author_min_n",      author_min_n_);
    author_keep_ratio_ = pget(params, "author_keep_ratio", author_keep_ratio_);
    author_score_      = pget(params, "author_score",      author_score_);
    author_guard_      = pget(params, "author_guard",      author_guard_);
    author_depth_min_  = pget(params, "author_depth_min",  author_depth_min_);
    author_depth_max_  = pget(params, "author_depth_max",  author_depth_max_);
    author_max_kept_   = pget(params, "author_max_kept",   author_max_kept_);
    seed_              = pget(params, "seed",              seed_);
    // xorshift32 init: mix the (possibly OGMA_SEED-namespaced) seed, never zero
    rng_state_ = uint32_t(uint64_t(seed_) * 2654435761u ^ 0x9E3779B9u);
    if (rng_state_ == 0) rng_state_ = 0x1234567u;
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
    auto d = [&](double dflt){
        if (auto* v = std::get_if<double>(&value)) return *v;
        if (auto* i = std::get_if<int64_t>(&value)) return double(*i);   // set_param sends ints as int64
        return dflt;
    };
    if      (k == "horizon")    horizon_    = d(horizon_);
    else if (k == "beam_k")     beam_k_     = d(beam_k_);
    else if (k == "mask_mode")  mask_mode_  = d(mask_mode_);
    else if (k == "mask_floor") mask_floor_ = d(mask_floor_);
    else if (k == "mask_joint")    mask_joint_    = d(mask_joint_);
    else if (k == "mask_val_lo")   mask_val_lo_   = d(mask_val_lo_);
    else if (k == "mask_val_hi")   mask_val_hi_   = d(mask_val_hi_);
    else if (k == "mask_depth_lo") mask_depth_lo_ = d(mask_depth_lo_);
    else if (k == "mask_depth_hi") mask_depth_hi_ = d(mask_depth_hi_);
    else if (k == "mask_strength") mask_strength_ = d(mask_strength_);
    else if (k == "author_mode")       author_mode_       = d(author_mode_);
    else if (k == "author_period")     author_period_     = d(author_period_);
    else if (k == "author_warmup")     author_warmup_     = d(author_warmup_);
    else if (k == "author_min_n")      author_min_n_      = d(author_min_n_);
    else if (k == "author_keep_ratio") author_keep_ratio_ = d(author_keep_ratio_);
    else if (k == "author_score")      author_score_      = d(author_score_);
    else if (k == "author_guard")      author_guard_      = d(author_guard_);
    else if (k == "author_depth_min")  author_depth_min_  = d(author_depth_min_);
    else if (k == "author_depth_max")  author_depth_max_  = d(author_depth_max_);
    else if (k == "author_max_kept")   author_max_kept_   = d(author_max_kept_);
}

int MotorPlanner::phase_bin(float phi) const {
    int b = int(phi / kTwoPi * float(phase_bins_));
    return std::clamp(b, 0, int(phase_bins_) - 1);
}

void MotorPlanner::apply_mask(Dist& d, int bin) const {
    // MODE 1 ONLY (phase-affinity, the recorded refuted arm).  The guard must
    // exclude mode 2 — a `>= 0.5` test silently ran this mask under mode 2,
    // emptying rows and truncating the roll (operator-reported flicker).
    if (mask_mode_ < 0.5 || mask_mode_ > 1.5) return;
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

// mode 2 — continuous region inhibition: suppress mass of tokens whose READOUT
// pose falls inside the masked value range on the masked joint(s).  Inhibition
// acts on the CONTINUOUS substrate through the mixture; tokens are carriers.
// Returns true if any mass was suppressed.
bool MotorPlanner::apply_region_mask(Dist& d, int depth) {
    if (mask_mode_ < 1.5 || mask_strength_ <= 0.0) return false;
    if (depth < int(mask_depth_lo_) || depth > int(mask_depth_hi_)) return false;
    const float lo = float(std::min(mask_val_lo_, mask_val_hi_));
    const float hi = float(std::max(mask_val_lo_, mask_val_hi_));
    if (hi <= lo) return false;
    const int jsel = int(mask_joint_);
    // INHIBITION MAY REROUTE, NEVER ANNIHILATE.  If suppression would zero the
    // whole row (every option inside the masked region), the mask is
    // uninformative at this depth — there is nothing to reroute TO.  The row
    // passes unmasked and the saturation counter records it (total-inhibition
    // rows were truncating the roll and flickering the widget's future).
    Dist backup = d;
    bool any = false;
    for (auto& [tok, pmass] : d) {
        auto it = tok_pose_.find(tok);
        if (it == tok_pose_.end() || it->second.n < 2) continue;
        bool inside = false;
        if (jsel >= 0 && jsel < kJoints) {
            float v = it->second.mean[jsel];
            inside = (v >= lo && v <= hi);
        } else {
            for (int j = 0; j < kJoints && !inside; ++j)
                inside = (it->second.mean[j] >= lo && it->second.mean[j] <= hi);
        }
        if (inside) { pmass *= float(1.0 - mask_strength_); any = true; ++masked_out_; }
    }
    if (!any) return false;
    float tot = 0.0f;
    for (auto const& kv : d) tot += kv.second;
    if (tot <= 1e-9f) {           // saturated: total inhibition — revert
        d = std::move(backup);
        ++mask_saturated_;
        return false;
    }
    for (auto it = d.begin(); it != d.end(); )
        it = (it->second <= 1e-9f) ? d.erase(it) : std::next(it);
    for (auto& kv : d) kv.second /= tot;
    return true;
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
                if (pose_seen_) {
                    for (int j = 0; j < kJoints; ++j) {
                        acc_jerr_[slot][j]     += std::fabs(it->pred_pose[j]     - cur_pose_[j]);
                        acc_jerr_raw_[slot][j] += std::fabs(it->pred_pose_raw[j] - cur_pose_[j]);
                        acc_jpers_[slot][j]    += std::fabs(it->pose0[j]         - cur_pose_[j]);
                        // SIGNED raw residual (Welford) — the author's proposal gradient:
                        // a persistent sign here means the un-inhibited decode
                        // systematically hallucinates mass on that side of reality.
                        double r  = double(it->pred_pose_raw[j]) - double(cur_pose_[j]);
                        long&  rn = res_n_[slot][j];
                        double d1 = r - res_mean_[slot][j];
                        res_mean_[slot][j] += d1 / double(++rn);
                        res_m2_[slot][j]   += d1 * (r - res_mean_[slot][j]);
                        if (it->trial != 0 && it->trial == trial_serial_) {
                            tacc_jerr_[slot][j]     += std::fabs(it->pred_pose[j]     - cur_pose_[j]);
                            tacc_jerr_raw_[slot][j] += std::fabs(it->pred_pose_raw[j] - cur_pose_[j]);
                        }
                    }
                    if (it->trial != 0 && it->trial == trial_serial_) ++tacc_n_[slot];
                }
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
    if (prev_tok_ >= 0) { trans_[prev_tok_ * kMaxBins + prev_bin_][cur_tok_] += 1.0f; ++n_obs_; }
    tok_phase_[cur_tok_][bin] += 1.0f;
    prev_tok_ = cur_tok_; prev_bin_ = bin;

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

    // ---- M1 mask author: advance the trial state machine BEFORE the roll so a
    // trial's mask params shape this tick's cone (inert unless author_mode=1).
    if (author_mode_ >= 0.5 && mask_mode_ > 1.5) author_step();

    // ---- roll the cone from t0 (the reflexes' row: the actual present) outward,
    // decoding EVERY depth to joint space for the piano roll.
    const int H = int(horizon_);
    roll_mean_.assign(size_t(H) * kJoints, 0.0f);
    roll_sd_.assign(size_t(H) * kJoints, 0.0f);
    roll_raw_mean_.assign(size_t(H) * kJoints, 0.0f);
    roll_raw_sd_.assign(size_t(H) * kJoints, 0.0f);
    roll_len_ = 0;
    mask_applied_ = false;
    // While a region mask is live, a TRUE second unmasked cone runs beside the
    // final one, so raw-vs-final is a genuine counterfactual at EVERY depth.
    // (The old single-cone "raw" was pre-this-row's-mask only: past the first
    // masked depth it inherited upstream rerouting and understated the mask's
    // real effect — the author's keep-rule needs the honest with/without.)
    const bool masking = mask_mode_ > 1.5 && mask_strength_ > 0.0;
    Dist row; row[cur_tok_] = 1.0f;
    Dist row_raw; if (masking) row_raw[cur_tok_] = 1.0f;
    int next_probe = 0;
    for (int n = 1; n <= H; ++n) {
        const int b = phase_bin(std::fmod(phi_ + float(n) * omega_, kTwoPi));
        row = propagate(row, b);
        apply_mask(row, b);                     // mode 1 (kept as the recorded arm)
        if (row.empty()) break;
        if (masking) {
            row_raw = propagate(row_raw, b);
            decode_row(row_raw, &roll_raw_mean_[size_t(n - 1) * kJoints],
                                &roll_raw_sd_[size_t(n - 1) * kJoints]);
        } else {
            decode_row(row, &roll_raw_mean_[size_t(n - 1) * kJoints],
                            &roll_raw_sd_[size_t(n - 1) * kJoints]);
        }
        // mode 2: the mask REROUTES the row itself, so suppression at depth n
        // propagates into every deeper row — the future influencing what the
        // near rows can still believe.
        if (apply_region_mask(row, n)) mask_applied_ = true;
        if (row.empty()) break;
        decode_row(row, &roll_mean_[size_t(n - 1) * kJoints],
                        &roll_sd_[size_t(n - 1) * kJoints]);
        roll_len_ = n;
        if (next_probe < kNumProbes && n == kProbes[next_probe]) {
            Pending pd{tick_id + uint64_t(n), n, cur_tok_, row, {}, {}, {},
                       (author_phase_ == 1 && masking) ? trial_serial_ : 0};
            for (int j = 0; j < kJoints; ++j) {
                pd.pred_pose[j]     = roll_mean_[size_t(n - 1) * kJoints + j];
                pd.pred_pose_raw[j] = roll_raw_mean_[size_t(n - 1) * kJoints + j];
                pd.pose0[j]         = cur_pose_[j];
            }
            pending_.push_back(std::move(pd));
            ++next_probe;
        }
    }
    if (pending_.size() > 512) pending_.erase(pending_.begin(), pending_.begin() + 128);
}

// ---------------------------------------------------------------------------
// The M1 MASK AUTHOR — the first slow loop over the continuous substrate.
// Propose (residual-guided + random) → trial (author_period ticks of dual
// verification) → judge (keep iff the whole-body final/raw verified-error
// ratio < author_keep_ratio).  Keep-rights are EARNED through the meters.
// ---------------------------------------------------------------------------

uint32_t MotorPlanner::rng_next() {
    uint32_t x = rng_state_;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return rng_state_ = x;
}

void MotorPlanner::author_step() {
    ++author_t_;
    switch (author_phase_) {
    case 0:   // warmup: mask held inert until the model has material
        mask_strength_ = 0.0;
        if (author_t_ >= long(author_warmup_) && pose_seen_ && n_obs_ >= 2000)
            author_start_trial();
        break;
    case 1:   // trial live
        if (author_t_ >= long(author_period_)) {
            // drain: mask off, let the trial's in-flight predictions verify.
            // New Pendings carry trial=0 (mask inactive), so nothing dilutes.
            mask_strength_ = 0.0;
            author_phase_ = 2;
            author_t_ = 0;
        }
        break;
    case 2:   // drain, then judge and immediately propose the next candidate
        if (author_t_ >= long(horizon_) + 8) {
            author_judge_trial();
            author_start_trial();
        }
        break;
    }
}

void MotorPlanner::author_start_trial() {
    // Recent joint-value envelope from the past ring (≈2 strides) — regions are
    // placed relative to the body's own running statistics, never absolute
    // constants (doctrine §5: adapt to the system's dynamics).
    std::array<float, kJoints> mu{}, sd{};
    const int N = past_n_;
    if (N < 16) { mask_strength_ = 0.0; author_phase_ = 0; author_t_ = 0; return; }
    for (int i = 0; i < N; ++i) {
        int idx = (past_head_ - N + i + kPastRing) % kPastRing;
        for (int j = 0; j < kJoints; ++j) mu[j] += past_[idx][j];
    }
    for (int j = 0; j < kJoints; ++j) mu[j] /= float(N);
    for (int i = 0; i < N; ++i) {
        int idx = (past_head_ - N + i + kPastRing) % kPastRing;
        for (int j = 0; j < kJoints; ++j) {
            float d = past_[idx][j] - mu[j];
            sd[j] += d * d;
        }
    }
    for (int j = 0; j < kJoints; ++j)
        sd[j] = std::max(0.05f, std::sqrt(sd[j] / float(N - 1)));

    // probe slots the author may target
    std::vector<int> slots;
    for (int i = 0; i < kNumProbes; ++i)
        if (kProbes[i] >= int(author_depth_min_) && kProbes[i] <= int(author_depth_max_))
            slots.push_back(i);
    if (slots.empty()) { mask_strength_ = 0.0; author_phase_ = 0; author_t_ = 0; return; }

    // GUIDED candidate: the (slot, joint) cell with the strongest standardized
    // signed residual (|mean|/stderr ≥ 3) not already trialed this run.  The
    // t-statistic is self-normalizing — no constant tuned to the signal's scale.
    int g_slot = -1, g_joint = -1, g_sign = 0; double g_t = 3.0;
    for (int i : slots) {
        for (int j = 0; j < kJoints; ++j) {
            long n = res_n_[i][j];
            if (n < 400) continue;
            double var = res_m2_[i][j] / double(n - 1);
            if (var <= 1e-12) continue;
            double t = std::fabs(res_mean_[i][j]) / std::sqrt(var / double(n));
            if (t <= g_t) continue;
            int sign = res_mean_[i][j] > 0 ? 1 : -1;
            int sig = (i * kJoints + j) * 2 + (sign > 0 ? 1 : 0);
            bool tried = false;
            for (int k = 0; k < tried_n_ && k < int(tried_.size()); ++k)
                if (tried_[k] == sig) { tried = true; break; }
            if (tried) continue;
            g_t = t; g_slot = i; g_joint = j; g_sign = sign;
        }
    }

    MaskSpec c;
    if (g_slot >= 0) {
        cand_guided_ = true;
        c.joint = g_joint;
        // the decode hallucinates on side g_sign of reality: inhibit the slab
        // beyond the body's recent envelope on that side
        float a = mu[g_joint] + float(g_sign) * 0.5f * sd[g_joint];
        float b = mu[g_joint] + float(g_sign) * 3.0f * sd[g_joint];
        c.lo = std::min(a, b); c.hi = std::max(a, b);
        c.dlo = (g_slot > 0) ? kProbes[g_slot - 1] + 1 : int(author_depth_min_);
        c.dlo = std::max(c.dlo, int(author_depth_min_));
        c.dhi = kProbes[g_slot];
        if (tried_n_ < int(tried_.size()))
            tried_[tried_n_++] = (g_slot * kJoints + g_joint) * 2 + (g_sign > 0 ? 1 : 0);
    } else {
        cand_guided_ = false;
        // RANDOM exploration: h1/knee-biased joint (the verified-material tracks),
        // random allowed slot, random slab from the joint's own envelope
        uint32_t r = rng_next();
        static constexpr int kPreferred[8] = {0, 1, 2, 3, 8, 9, 10, 11};
        c.joint = (r % 10 < 8) ? kPreferred[(r / 16) % 8] : int((r / 16) % kJoints);
        int slot = slots[(rng_next()) % slots.size()];
        c.dlo = (slot > 0) ? kProbes[slot - 1] + 1 : int(author_depth_min_);
        c.dlo = std::max(c.dlo, int(author_depth_min_));
        c.dhi = kProbes[slot];
        float u = (float(rng_next() % 4001) / 1000.0f) - 2.0f;   // [-2, 2]
        float w = 0.5f + float(rng_next() % 2001) / 1000.0f;     // [0.5, 2.5]
        float ctr = mu[c.joint] + u * sd[c.joint];
        c.lo = ctr - 0.5f * w * sd[c.joint];
        c.hi = ctr + 0.5f * w * sd[c.joint];
    }
    c.lo = std::clamp(c.lo, -1.5f, 1.5f);
    c.hi = std::clamp(c.hi, -1.5f, 1.5f);
    cand_ = c;

    // the author OWNS the mask params during a trial
    mask_joint_    = double(c.joint);
    mask_val_lo_   = double(c.lo);
    mask_val_hi_   = double(c.hi);
    mask_depth_lo_ = double(c.dlo);
    mask_depth_hi_ = double(c.dhi);
    mask_strength_ = 1.0;
    for (auto& a : tacc_jerr_)     a.fill(0.0);
    for (auto& a : tacc_jerr_raw_) a.fill(0.0);
    tacc_n_.fill(0);
    ++trial_serial_;
    author_phase_ = 1;
    author_t_ = 0;
}

void MotorPlanner::author_judge_trial() {
    // Judge on every probe at or beyond the mask's first depth: rerouting
    // propagates deeper, so the meter must capture the whole downstream roll.
    double num_all = 0.0, den_all = 0.0, num_tgt = 0.0, den_tgt = 0.0;
    long n_tot = 0;
    for (int i = 0; i < kNumProbes; ++i) {
        if (kProbes[i] < cand_.dlo) continue;
        if (tacc_n_[i] < long(author_min_n_)) continue;
        n_tot += tacc_n_[i];
        for (int j = 0; j < kJoints; ++j) {
            num_all += tacc_jerr_[i][j];
            den_all += tacc_jerr_raw_[i][j];
            if (j == cand_.joint) {
                num_tgt += tacc_jerr_[i][j];
                den_tgt += tacc_jerr_raw_[i][j];
            }
        }
    }
    ++trials_done_;
    last_ratio_all_ = den_all > 1e-9 ? num_all / den_all : -1.0;
    last_ratio_tgt_ = den_tgt > 1e-9 ? num_tgt / den_tgt : -1.0;
    // Two scoring altitudes.  Whole-body (score=0, v1.0) was measured near-null:
    // a good target-side mask improves its joint 8–25% while the other 11 joints
    // dilute the pooled ratio to ~1.  Per-joint (score=1) is the ledger's
    // altitude — the verified material lives in the joint marginals — with the
    // whole-body ratio as a no-damage guard (the demo-mask failure mode).
    bool keep;
    if (author_score_ >= 0.5) {
        keep = last_ratio_tgt_ > 0.0 && last_ratio_tgt_ < author_keep_ratio_ &&
               last_ratio_all_ > 0.0 && last_ratio_all_ < author_guard_;
    } else {
        keep = last_ratio_all_ > 0.0 && last_ratio_all_ < author_keep_ratio_;
    }
    if (n_tot > 0 && keep) {
        kept_.push_back(Kept{cand_, last_ratio_all_, last_ratio_tgt_, n_tot,
                             trial_serial_, cand_guided_});
        const bool by_tgt = author_score_ >= 0.5;
        std::sort(kept_.begin(), kept_.end(), [by_tgt](Kept const& a, Kept const& b){
            return (by_tgt ? a.ratio_tgt : a.ratio_all) < (by_tgt ? b.ratio_tgt : b.ratio_all); });
        if (int(kept_.size()) > int(author_max_kept_)) kept_.resize(size_t(author_max_kept_));
    }
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
    // PER-JOINT authority: same protocol, continuous space — deepest probe where
    // the decoded joint prediction's mean |error| beats hold-pose by ≥5%.  The
    // token chain is one whole-body model; these are its per-joint marginals,
    // and a joint can carry earned authority the global argmax does not.
    nlohmann::json jauth = nlohmann::json::array();
    for (int j = 0; j < kJoints; ++j) {
        int a = 0;
        for (int i = 0; i < kNumProbes; ++i) {
            if (acc_n_[i] < 200) break;
            double n = double(acc_n_[i]);
            double je = acc_jerr_[i][j] / n, pe = acc_jpers_[i][j] / n;
            if (je >= 0.95 * pe) break;
            a = kProbes[i];
        }
        jauth.push_back(a);
    }
    mod["joint_auth"] = std::move(jauth);
    // PER-JOINT authority BAND [lo, hi]: the longest contiguous run of probe
    // depths where the joint marginal wins.  Measured 2026-08-11: hip1/knee
    // marginals lose 3× at k=1 (the jump to token mean vs a barely-moving body)
    // yet win 12–23% at k=8–34 — authority need not start at the present.
    // Reflexes own the near field; a planner earns a BAND of the future.
    nlohmann::json jband = nlohmann::json::array();
    for (int j = 0; j < kJoints; ++j) {
        int best_lo = 0, best_hi = 0, best_len = 0, cur_lo = 0, cur_hi = 0, run = 0;
        for (int i = 0; i < kNumProbes; ++i) {
            double n = double(std::max<long>(1, acc_n_[i]));
            bool win = acc_n_[i] >= 200 &&
                       (acc_jerr_[i][j] / n) < 0.95 * (acc_jpers_[i][j] / n);
            if (win) {
                if (run == 0) cur_lo = kProbes[i];
                cur_hi = kProbes[i];
                if (++run > best_len) { best_len = run; best_lo = cur_lo; best_hi = cur_hi; }
            } else run = 0;
        }
        jband.push_back(best_lo);
        jband.push_back(best_hi);
    }
    mod["joint_band"] = std::move(jband);
    // M1 MASK AUTHOR state — present only when the loop is on.  kept masks are
    // RECORDED (earned through the meters), not re-applied in v1: the block says
    // what the author has learned, not what is currently shaping the roll.
    if (author_mode_ >= 0.5) {
        nlohmann::json au;
        au["phase"]  = author_phase_;          // 0 warmup · 1 trial · 2 drain
        au["trial"]  = trial_serial_;
        au["trials"] = trials_done_;
        au["last_r"]     = std::round(last_ratio_all_ * 10000.0) / 10000.0;
        au["last_r_tgt"] = std::round(last_ratio_tgt_ * 10000.0) / 10000.0;
        if (author_phase_ == 1) {
            au["cand"] = {{"j", cand_.joint}, {"lo", std::round(cand_.lo * 1000.0f) / 1000.0f},
                          {"hi", std::round(cand_.hi * 1000.0f) / 1000.0f},
                          {"dlo", cand_.dlo}, {"dhi", cand_.dhi},
                          {"guided", cand_guided_ ? 1 : 0}};
        }
        nlohmann::json kl = nlohmann::json::array();
        for (auto const& k : kept_) {
            kl.push_back({{"j", k.spec.joint},
                          {"lo", std::round(k.spec.lo * 1000.0f) / 1000.0f},
                          {"hi", std::round(k.spec.hi * 1000.0f) / 1000.0f},
                          {"dlo", k.spec.dlo}, {"dhi", k.spec.dhi},
                          {"r", std::round(k.ratio_all * 10000.0) / 10000.0},
                          {"rt", std::round(k.ratio_tgt * 10000.0) / 10000.0},
                          {"n", k.n}, {"tr", k.trial}, {"g", k.guided ? 1 : 0}});
        }
        au["kept"] = std::move(kl);
        mod["author"] = std::move(au);
    }
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
    // The operator's three-layer debugging contract: ORIGINAL motion (raw,
    // pre-mask), THE MASK, FINAL motion.  Raw roll ships only while a mask is
    // actually suppressing mass (payload stays lean otherwise).
    mod["mask_applied"] = mask_applied_;
    mod["mask_saturated"] = mask_saturated_;
    if (mask_mode_ > 1.5 && mask_strength_ > 0.0) {
        nlohmann::json rrm = nlohmann::json::array(), rrs = nlohmann::json::array();
        for (int i = 0; i < roll_len_ * kJoints; ++i) {
            rrm.push_back(std::round(roll_raw_mean_[i] * 1000.0f) / 1000.0f);
            rrs.push_back(std::round(roll_raw_sd_[i]   * 1000.0f) / 1000.0f);
        }
        mod["roll_raw_mean"] = std::move(rrm);
        mod["roll_raw_sd"]   = std::move(rrs);
    }
    if (mask_mode_ > 1.5) {
        mod["mask_spec"] = {
            {"joint", int(mask_joint_)},
            {"val_lo", mask_val_lo_}, {"val_hi", mask_val_hi_},
            {"depth_lo", int(mask_depth_lo_)}, {"depth_hi", int(mask_depth_hi_)},
            {"strength", mask_strength_},
        };
        // the inhibition damage/benefit meter: per-depth mean |err| of the
        // FINAL (masked) vs RAW decode, averaged over joints
        nlohmann::json em = nlohmann::json::array(), er = nlohmann::json::array();
        for (int i = 0; i < kNumProbes; ++i) {
            double n = double(std::max<long>(1, acc_n_[i]));
            double a = 0.0, b = 0.0;
            for (int j = 0; j < kJoints; ++j) {
                a += acc_jerr_[i][j] / n;
                b += acc_jerr_raw_[i][j] / n;
            }
            em.push_back(std::round(a / kJoints * 10000.0) / 10000.0);
            er.push_back(std::round(b / kJoints * 10000.0) / 10000.0);
        }
        mod["masked_err"] = std::move(em);
        mod["raw_err"]    = std::move(er);
    }
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
    // per-depth × per-joint mean |error| pairs (cone decode vs hold-pose) —
    // the raw material behind joint_auth, for the widget's certainty readout
    nlohmann::json je = nlohmann::json::array(), jp = nlohmann::json::array();
    for (int i = 0; i < kNumProbes; ++i) {
        double n = double(std::max<long>(1, acc_n_[i]));
        for (int j = 0; j < kJoints; ++j) {
            je.push_back(std::round(acc_jerr_[i][j] / n * 10000.0) / 10000.0);
            jp.push_back(std::round(acc_jpers_[i][j] / n * 10000.0) / 10000.0);
        }
    }
    mod["jerr"]  = std::move(je);
    mod["jpers"] = std::move(jp);
    // the author's proposal gradient, top standardized residual cells — the
    // widget's "where does the raw decode hallucinate" readout
    if (author_mode_ >= 0.5) {
        std::vector<std::array<double, 4>> cells;   // t, depth, joint, mean
        for (int i = 0; i < kNumProbes; ++i)
            for (int j = 0; j < kJoints; ++j) {
                long n = res_n_[i][j];
                if (n < 400) continue;
                double var = res_m2_[i][j] / double(n - 1);
                if (var <= 1e-12) continue;
                double t = std::fabs(res_mean_[i][j]) / std::sqrt(var / double(n));
                cells.push_back({t, double(kProbes[i]), double(j), res_mean_[i][j]});
            }
        std::sort(cells.begin(), cells.end(),
                  [](auto const& a, auto const& b){ return a[0] > b[0]; });
        nlohmann::json rt = nlohmann::json::array();
        for (size_t c = 0; c < cells.size() && c < 5; ++c)
            rt.push_back({{"t", std::round(cells[c][0] * 10.0) / 10.0},
                          {"d", int(cells[c][1])}, {"j", int(cells[c][2])},
                          {"m", std::round(cells[c][3] * 1000.0) / 1000.0}});
        mod["res_top"] = std::move(rt);
    }
    return mod;
}

} // namespace ogma
