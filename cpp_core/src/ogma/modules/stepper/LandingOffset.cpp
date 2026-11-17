// LandingOffset -- see the header: the landing's forward/back offset chosen by expected free energy (learned stepper plan Phase 4).
#include "ogma/modules/stepper/LandingOffset.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"

namespace ogma::stepper {

void LandingOffset::reset() {
    for (auto& l : legs_) { l.P = Eigen::Matrix<double, kF, kF>::Identity() * p0_; l.W.setZero(); l.n = 0; }
    s2_.setZero(); r2_.setZero(); v2_.setZero(); nis_num_.setZero(); nis_den_.setZero(); n_out_ = 0; pending_ = Pending{}; last_bad_ = false;
}

// what varies across the candidates: the offset (in units of the full range) and the remembered terrain at the candidate, in cm
LandingOffset::Feat LandingOffset::features(double d_unit, double rise_m, double relief_m, double sigma_m, bool known) {
    Feat f; f << 1.0, d_unit, d_unit * d_unit, known ? 100.0 * rise_m : 0.0, known ? 100.0 * relief_m : 0.0, known ? 100.0 * sigma_m : 0.0, known ? 0.0 : 1.0;
    return f;
}

// the realised consequence: touchdown error (cm, signed: the toe's height where it stopped against the ground expected there),
// the summed length of the hand's tagged corrections of the landing (cm), hang, miss
LandingOffset::Out LandingOffset::outcome_vector(const StepOutcome& o) {
    double corr = 0.0; for (auto const& m : o.moves) corr += std::hypot(m[0], m[1]);
    Out y; y << 100.0 * (o.h_touch - o.h_pred), 100.0 * corr, o.hang ? 1.0 : 0.0, o.touched ? 0.0 : 1.0;
    return y;
}

LandingOffset::Choice LandingOffset::choose(int leg, const std::vector<Candidate>& cands, bool distress) const {
    const Leg& L = legs_[size_t(std::clamp(leg, 0, 3))];
    Choice best; double best_g = 1e300; bool any = false;
    for (int i = 0; i < int(cands.size()); ++i) {
        const Candidate& c = cands[size_t(i)]; if (!c.admitted) continue;
        const Out yhat = L.W * c.phi; const double u = c.phi.dot(L.P * c.phi);
        double risk = 0.0; for (int k = 0; k < kY; ++k) if (s2_[k] > 0.0) risk += yhat[k] * yhat[k] / (2.0 * s2_[k]);
        const double epi = distress ? 0.0 : 0.5 * kY * std::log1p(std::max(0.0, u));
        const double g = risk - epi;
        const bool better = !any || g < best_g - 1e-12 || (std::abs(g - best_g) <= 1e-12 && std::abs(c.delta) < std::abs(best.delta) - 1e-12);
        if (better) { best.index = i; best.delta = c.delta; best.risk = risk; best.epi = epi; best.u = u; best.yhat = yhat; best_g = g; any = true; }
    }
    return best;
}

void LandingOffset::update(int leg, const Feat& phi, const Out& y) {
    Leg& L = legs_[size_t(std::clamp(leg, 0, 3))];
    const Out e = y - L.W * phi;                         // prequential: predicted before this step is learned
    const Feat Pp = L.P * phi; const double u = phi.dot(Pp);
    for (int k = 0; k < kY; ++k)                         // the grade: the residual against the spread the model itself has shown
        if (v2_[k] > 0.0) { nis_num_[k] += e[k] * e[k]; nis_den_[k] += v2_[k] * (1.0 + u); }
    ++n_out_;
    for (int k = 0; k < kY; ++k) { s2_[k] += (y[k] * y[k] - s2_[k]) / double(n_out_); r2_[k] += (e[k] * e[k] - r2_[k]) / double(n_out_); v2_[k] += (e[k] * e[k] / (1.0 + u) - v2_[k]) / double(n_out_); }
    const Feat gain = Pp / (1.0 + u);
    L.W += e * gain.transpose();
    L.P -= gain * Pp.transpose();
    L.P = 0.5 * (L.P + L.P.transpose()).eval();
    ++L.n; last_err_ = e;
}

void LandingOffset::absorb(const StepExecutor& ex) {
    if (ex.outcome_seq_ == seen_seq_) return;
    seen_seq_ = ex.outcome_seq_;
    const StepOutcome& o = ex.outcome_;
    last_bad_ = o.hang || !o.touched;
    if (!(pending_.valid && o.kind == 1 && o.leg == pending_.leg)) { if (o.kind == 1) ++unpaired_; return; }   // not this decision's step (unplanned, or the leg switched)
    update(pending_.leg, pending_.phi, outcome_vector(o));
    ++updates_; last_upd_delta_ = pending_.delta; last_upd_leg_ = pending_.leg; last_upd_seq_ = o.seq;
    pending_.valid = false;
}

StepIntent LandingOffset::decide(StepIntent in, const StepperInputs& inp, const SupportFloor& floor, ScaffoldPlanner& planner, const StepExecutor& ex) {
    absorb(ex);
    if (pending_.valid) { ++abandoned_; pending_.valid = false; }   // a decision supersedes a step that never landed
    if (in.kind != StepIntent::Plan || in.leg < 0 || in.leg > 3) return in;
    ++decisions_;
    const int leg = in.leg; const Vec3f toe = ex.toe_tgt_[size_t(leg)];
    double h0 = 0.0; const bool k0 = inp.patch_h(toe.z, toe.x, h0);
    const double range = std::clamp(gain_, 0.0, 1.0) * max_m_;
    const int n = cell_m_ > 0.0 ? std::min(6, int(std::floor(range / cell_m_ + 1e-9))) : 0;
    const double need = floor.margin_min_m_ + floor.margin_extra_m_;
    const bool have_pred = planner.pred_leg_ >= 0 && planner.pred_step_ == leg;
    std::vector<Candidate> cands; std::vector<double> margins;
    for (int i = 0; i <= 2 * n; ++i) {   // 0, +1, -1, +2, -2, ...: ties go to the smaller offset, forward first
        const int cell = i == 0 ? 0 : ((i % 2) ? (i + 1) / 2 : -(i / 2));
        Candidate c; c.delta = cell * cell_m_;
        Vec3f L = in.land; L.z = float(double(in.land.z) + c.delta);
        double m = have_pred ? planner.pred_m_ : 0.0;
        if (cell != 0) {   // the floor: the search's own landing checks, and the next leg no less liftable than the scaffold left it
            c.admitted = planner.landing_ok(leg, L);
            if (!c.admitted) ++veto_by_[size_t(cell > 0 ? 0 : 1)];
            if (c.admitted && have_pred) {
                std::array<Vec3f, 4> landed = planner.pred_toes_; landed[size_t(leg)] = Vec3f(L.x, landed[size_t(leg)].y, L.z);
                double sx, sz; m = planner.best_margin_over_shifts(planner.pred_leg_, landed, planner.pred_cx_, planner.pred_cz_, sx, sz, false);
                c.admitted = m >= need || m >= planner.pred_m_ - 1e-9;
                if (!c.admitted) ++veto_by_[size_t(cell > 0 ? 2 : 3)];
            }
            if (!c.admitted) ++vetoed_;
        }
        double h = 0.0, sg = 0.0, rel = 0.0; const bool known = inp.patch_h(L.z, L.x, h, sg) && k0; inp.patch_edge(L.z, L.x, rel);
        c.phi = features(max_m_ > 0.0 ? c.delta / max_m_ : 0.0, h - h0, rel, sg, known);
        cands.push_back(c); margins.push_back(m);
    }
    const bool tilted = inp.have_up_ && inp.up_[1] < 0.5 * (1.0 + floor.tilt_disengage_);
    const bool distress = last_bad_ || planner.last_plan_skips_ > 0 || tilted;
    if (distress) ++distress_;
    const Choice ch = choose(leg, cands, distress);
    int admitted = 0; for (auto const& c : cands) admitted += c.admitted ? 1 : 0;
    last_ = ch; last_leg_ = leg; last_admitted_ = admitted; last_distress_ = distress;
    const int cell = int(std::lround(ch.delta / std::max(1e-9, cell_m_)));
    ++hist_[size_t(std::clamp(cell + 6, 0, 12))];
    if (ch.delta != 0.0) {
        ++shifted_;
        in.land.z = float(double(in.land.z) + ch.delta);
        if (have_pred) {   // the plan drift instrument checks the landing the body is now asked for
            planner.pred_toes_[size_t(leg)].z = in.land.z; planner.pred_m_ = margins[size_t(ch.index)];
        }
    }
    pending_.valid = true; pending_.leg = leg; pending_.delta = ch.delta; pending_.phi = cands[size_t(ch.index)].phi; pending_.yhat = ch.yhat; pending_.u = ch.u; pending_.seq = decisions_;
    return in;
}

nlohmann::json LandingOffset::diag() const {
    auto v = [](const Out& o) { return std::vector<double>{o[0], o[1], o[2], o[3]}; };
    return nlohmann::json{
        {"gain", gain_}, {"learn", learn_},
        // the latest decision: [its count, leg, delta m, risk, epistemic value, candidates admitted, distress, leverage u, yhat x 4]
        {"dec", {double(decisions_), double(last_leg_), last_.delta, last_.risk, last_.epi, double(last_admitted_), last_distress_ ? 1.0 : 0.0, last_.u, last_.yhat[0], last_.yhat[1], last_.yhat[2], last_.yhat[3]}},
        // the latest update: [the outcome's seq, leg, delta m, the prequential error x 4]
        {"upd", {double(last_upd_seq_), double(last_upd_leg_), last_upd_delta_, last_err_[0], last_err_[1], last_err_[2], last_err_[3]}},
        {"n", {decisions_, shifted_, vetoed_, distress_, updates_, abandoned_, unpaired_, veto_by_[0], veto_by_[1], veto_by_[2], veto_by_[3]}},
        {"leg_n", {legs_[0].n, legs_[1].n, legs_[2].n, legs_[3].n}},
        {"hist", hist_}, {"s2", v(s2_)}, {"r2", v(r2_)}, {"nis", {nis(0), nis(1), nis(2), nis(3)}}};
}

nlohmann::json LandingOffset::snapshot() const {
    nlohmann::json j{{"version", 1}, {"n_out", n_out_}, {"legs", nlohmann::json::array()}};
    auto vec = [](const double* p, int n) { return std::vector<double>(p, p + n); };
    j["s2"] = vec(s2_.data(), kY); j["r2"] = vec(r2_.data(), kY); j["v2"] = vec(v2_.data(), kY); j["nis_num"] = vec(nis_num_.data(), kY); j["nis_den"] = vec(nis_den_.data(), kY);
    for (auto const& l : legs_) j["legs"].push_back({{"n", l.n}, {"P", vec(l.P.data(), kF * kF)}, {"W", vec(l.W.data(), kY * kF)}});
    return j;
}

void LandingOffset::restore(const nlohmann::json& s) {
    if (s.value("version", 0) != 1) throw std::invalid_argument("LandingOffset::restore: unknown version");
    const auto& lg = s.at("legs"); if (!lg.is_array() || lg.size() != 4) throw std::invalid_argument("LandingOffset::restore: four legs expected");
    auto load = [](const nlohmann::json& a, double* p, int n) { const auto v = a.get<std::vector<double>>(); if (int(v.size()) != n) throw std::invalid_argument("LandingOffset::restore: width mismatch"); std::copy(v.begin(), v.end(), p); };
    for (int i = 0; i < 4; ++i) { legs_[size_t(i)].n = lg[size_t(i)].at("n").get<int64_t>(); load(lg[size_t(i)].at("P"), legs_[size_t(i)].P.data(), kF * kF); load(lg[size_t(i)].at("W"), legs_[size_t(i)].W.data(), kY * kF); }
    load(s.at("s2"), s2_.data(), kY); load(s.at("r2"), r2_.data(), kY); load(s.at("v2"), v2_.data(), kY); load(s.at("nis_num"), nis_num_.data(), kY); load(s.at("nis_den"), nis_den_.data(), kY);
    n_out_ = s.at("n_out").get<int64_t>();
    pending_ = Pending{}; last_bad_ = false;   // a decision does not carry across a run: the body starts fresh
}

} // namespace ogma::stepper
