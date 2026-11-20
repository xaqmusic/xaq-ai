// AfterTouch -- see the header: the step's record past first contact, and a model of it in shadow (learned stepper plan 9.5).
#include "ogma/modules/stepper/AfterTouch.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"

namespace ogma::stepper {

void AfterTouch::reset() {
    for (auto& l : legs_) { for (auto& p : l.P) p = Eigen::Matrix<double, kF, kF>::Identity() * p0_; l.W.setZero(); l.n = {}; l.mean = {}; }
    n_ = {}; mean_ = {}; r2_ = {}; b2_ = {}; v2_ = {}; nis_num_ = {}; nis_den_ = {};
    rec_ = Record{}; last_ = Record{}; prev_phase_ = Phase::Idle; go_leg_ = -1; go_fresh_ = false;
}

// what is known at the touch: the gap (cm) and its positive part (the toe held ABOVE its ground is the case that matters; a toe
// held below it presses), the body's tilt, and what the toe carried as it touched
AfterTouch::Feat AfterTouch::features(double gap_m, double tilt_deg, double load) {
    Feat f; f << 1.0, 100.0 * gap_m, 100.0 * std::max(0.0, gap_m), 0.1 * tilt_deg, load;
    return f;
}

AfterTouch::Out AfterTouch::predict(int leg, const Feat& phi) const {
    const Leg& L = legs_[size_t(std::clamp(leg, 0, 3))]; Out y{};
    for (int k = 0; k < kY; ++k) y[size_t(k)] = L.W.row(k).dot(phi);
    return y;
}

void AfterTouch::update(int leg, const Feat& phi, const Out& y, const Has& has) {
    Leg& L = legs_[size_t(std::clamp(leg, 0, 3))];
    for (int k = 0; k < kY; ++k) {
        if (!has[size_t(k)]) continue;
        const size_t q = size_t(k);
        const double e = y[q] - L.W.row(k).dot(phi);               // prequential: predicted before this record is learned
        const double eb = y[q] - L.mean[q];                         // the base rate: this leg's own running mean, before the update
        const Feat Pp = L.P[q] * phi; const double u = phi.dot(Pp);
        if (v2_[q] > 0.0) { nis_num_[q] += e * e; nis_den_[q] += v2_[q] * (1.0 + u); }
        ++n_[q]; const double a = 1.0 / double(n_[q]);
        mean_[q] += a * (y[q] - mean_[q]); r2_[q] += a * (e * e - r2_[q]); b2_[q] += a * (eb * eb - b2_[q]); v2_[q] += a * (e * e / (1.0 + u) - v2_[q]);
        ++L.n[q]; L.mean[q] += (y[q] - L.mean[q]) / double(L.n[q]);
        const Feat gain = Pp / (1.0 + u);
        L.W.row(k) += e * gain.transpose();
        L.P[q] -= gain * Pp.transpose();
        L.P[q] = 0.5 * (L.P[q] + L.P[q].transpose()).eval();
    }
}

double AfterTouch::tilt_deg(const StepperInputs& in) {
    return in.have_up_ ? std::acos(std::clamp(in.up_[1], -1.0, 1.0)) * 180.0 / M_PI : 0.0;
}

void AfterTouch::close(int reason, const StepExecutor&, const StepperInputs& in) {
    Record& r = rec_;
    r.open = false; r.reason = reason; r.tilt1 = tilt_deg(in);
    if (reason == ByDisengage) { ++count_[2]; last_ = r; return; }   // the body left the stepper: nothing to learn from this one
    ++count_[1]; if (reason == ByCap) ++count_[3];
    r.has = {false, false, false, false}; r.y = {-1.0, -1.0, 0.0, -1.0};
    if (r.planted >= min_ticks_ && in.have_contact_) { r.has[0] = true; r.y[0] = 2 * r.contact < r.planted ? 1.0 : 0.0; ++count_[4]; if (r.y[0] > 0.5) ++count_[5]; }
    if (r.went) { r.has[1] = true; r.y[1] = r.go_leg == r.leg ? 1.0 : 0.0; ++count_[6]; if (r.y[1] > 0.5) ++count_[7];
                  r.has[3] = true; r.y[3] = r.skip ? 1.0 : 0.0; if (r.skip) ++count_[8]; }
    if (in.have_up_) { r.has[2] = true; r.y[2] = r.tilt1 - r.tilt0; }
    update(r.leg, r.phi, r.y, r.has);
    last_ = r;
}

void AfterTouch::observe(const StepExecutor& ex, const ScaffoldPlanner& pl, const StepperInputs& in, const LoadBook& book) {
    const Phase ph = ex.phase_;
    if (ph == Phase::Idle || ph == Phase::Return) {   // disengaged (or on its way out): an open record is dropped, and a step in the air is no longer this stepper's
        if (rec_.open) close(ByDisengage, ex, in);
        seen_seq_ = ex.outcome_seq_; prev_phase_ = ph; go_fresh_ = false;
        return;
    }
    // a go: a leg leaves the ground (the shift's last tick became the lift's first)
    if (ph == Phase::Lift && prev_phase_ == Phase::Shift && ex.leg_ >= 0 && ex.leg_ < 4) {
        go_leg_ = ex.leg_; go_fresh_ = true;
        // the touch height the executor expects, in the stance's own terms (the lowering ends where the command's height meets land_h_)
        go_y_pred_ = ex.land_h_ - in.gravity_h(Vec3f(ex.land_.x, float(-ex.stand_h_m_), ex.land_.z));
        if (rec_.open && !rec_.went) { rec_.went = true; rec_.go_leg = ex.leg_; rec_.skip = pl.last_plan_skips_ > 0; }
    }
    // a step's outcome: the open record closes on it, and its own opens
    if (ex.outcome_seq_ != seen_seq_) {
        seen_seq_ = ex.outcome_seq_;
        if (rec_.open) close(ByLanding, ex, in);
        const StepOutcome& o = ex.outcome_;
        if (o.leg >= 0 && o.leg < 4) {
            Record r; r.open = true; r.seq = o.seq; r.leg = o.leg; r.touched = o.touched; r.h_touch = o.h_touch; r.h_pred = o.h_pred;
            const size_t q = size_t(o.leg);
            r.y_touch = ex.elev_landed_[q];                                  // the command's body-frame height at the touch, over the standing plane
            r.y_hold0 = double(ex.toe_tgt_[q].y) + ex.stand_h_m_;            // what the stance has made of it on this same tick
            r.gap = r.y_hold0 - r.y_touch;
            r.have_pred = go_fresh_ && go_leg_ == o.leg; r.y_pred = r.have_pred ? go_y_pred_ : r.y_touch; go_fresh_ = false;
            r.load0 = in.have_load_ ? double(in.load_[q]) : 0.0; r.tilt0 = tilt_deg(in);
            r.phi = features(r.gap, r.tilt0, r.load0); r.yhat = predict(o.leg, r.phi);
            rec_ = r; ++count_[0]; if (!o.touched) ++count_[9];
        }
    }
    if (rec_.open) {   // the toe's stance since its touch
        Record& r = rec_; const size_t q = size_t(r.leg);
        ++r.ticks;
        const bool stepping = ex.leg_ == r.leg && (ph == Phase::Lift || ph == Phase::Swing || ph == Phase::Lower);
        if (!stepping) {
            ++r.planted;
            if (in.have_contact_ && in.contact_[q] > 0.5f) ++r.contact;
            if (in.have_load_) { r.load_sum += double(in.load_[q]); if (double(in.load_[q]) >= book.unloaded_share_) ++r.loaded; }
            r.hold_sum += double(ex.toe_tgt_[q].y) + ex.stand_h_m_; r.creep = ex.creep_[q];
        }
        if (r.ticks >= cap_ticks_) close(ByCap, ex, in);
    }
    prev_phase_ = ph;
}

std::vector<double> AfterTouch::Record::floats() const {
    const double np = double(std::max(1, planted));
    std::vector<double> v{double(seq), double(leg), touched ? 1.0 : 0.0, y_touch, y_pred, y_hold0, gap, h_touch, h_pred, load0, tilt0, tilt1,
                          double(ticks), double(planted), double(contact), double(loaded), load_sum / np, hold_sum / np, creep, double(go_leg), skip ? 1.0 : 0.0, double(reason)};
    for (int k = 0; k < kY; ++k) v.push_back(has[size_t(k)] ? y[size_t(k)] : -1.0);
    for (int k = 0; k < kY; ++k) v.push_back(yhat[size_t(k)]);
    v.push_back(have_pred ? 1.0 : 0.0); v.push_back(0.0);
    return v;
}

nlohmann::json AfterTouch::diag() const {
    std::vector<double> g;   // per outcome [n, mean, the model's residual mean square, the base rate's, NIS]
    for (int k = 0; k < kY; ++k) { const size_t q = size_t(k); g.push_back(double(n_[q])); g.push_back(mean_[q]); g.push_back(r2_[q]); g.push_back(b2_[q]); g.push_back(nis(k)); }
    return nlohmann::json{{"learn", learn_}, {"rec", last_.floats()}, {"n", count_}, {"grade", g},
                          {"open", {double(rec_.open ? rec_.seq : 0), double(rec_.open ? rec_.leg : -1), double(rec_.ticks), double(rec_.planted), double(rec_.contact)}}};
}

nlohmann::json AfterTouch::snapshot() const {
    auto vec = [](const double* p, int n) { return std::vector<double>(p, p + n); };
    nlohmann::json j{{"version", 1}, {"n", n_}, {"mean", mean_}, {"r2", r2_}, {"b2", b2_}, {"v2", v2_}, {"nis_num", nis_num_}, {"nis_den", nis_den_}, {"legs", nlohmann::json::array()}};
    for (auto const& l : legs_) {
        nlohmann::json P = nlohmann::json::array(); for (auto const& p : l.P) P.push_back(vec(p.data(), kF * kF));
        j["legs"].push_back({{"n", l.n}, {"mean", l.mean}, {"P", P}, {"W", vec(l.W.data(), kY * kF)}});
    }
    return j;
}

void AfterTouch::restore(const nlohmann::json& s) {
    if (s.value("version", 0) != 1) throw std::invalid_argument("AfterTouch::restore: unknown version");
    const auto& lg = s.at("legs"); if (!lg.is_array() || lg.size() != 4) throw std::invalid_argument("AfterTouch::restore: four legs expected");
    auto load = [](const nlohmann::json& a, double* p, int n) { const auto v = a.get<std::vector<double>>(); if (int(v.size()) != n) throw std::invalid_argument("AfterTouch::restore: width mismatch"); std::copy(v.begin(), v.end(), p); };
    for (int i = 0; i < 4; ++i) {
        Leg& L = legs_[size_t(i)]; const auto& j = lg[size_t(i)];
        L.n = j.at("n").get<std::array<int64_t, kY>>(); L.mean = j.at("mean").get<std::array<double, kY>>();
        if (j.at("P").size() != size_t(kY)) throw std::invalid_argument("AfterTouch::restore: one posterior per outcome expected");
        for (int k = 0; k < kY; ++k) load(j.at("P")[size_t(k)], L.P[size_t(k)].data(), kF * kF);
        load(j.at("W"), L.W.data(), kY * kF);
    }
    n_ = s.at("n").get<std::array<int64_t, kY>>(); mean_ = s.at("mean").get<std::array<double, kY>>(); r2_ = s.at("r2").get<std::array<double, kY>>(); b2_ = s.at("b2").get<std::array<double, kY>>();
    v2_ = s.at("v2").get<std::array<double, kY>>(); nis_num_ = s.at("nis_num").get<std::array<double, kY>>(); nis_den_ = s.at("nis_den").get<std::array<double, kY>>();
    rec_ = Record{}; last_ = Record{}; prev_phase_ = Phase::Idle; go_fresh_ = false;   // a record does not carry across a run: the body starts fresh
}

} // namespace ogma::stepper
