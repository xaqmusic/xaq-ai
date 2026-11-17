// ScaffoldPlanner -- see the header.  Split out of PoseSequencer byte-identical (learned stepper plan Phase 2, 2026-11-15): every function body is the
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include <algorithm>
#include <cmath>

namespace ogma::stepper {

// the next step (moved from PoseSequencer::decide_next; the executor clears the plan's state before asking)
StepIntent ScaffoldPlanner::decide() {
    if (book_.unloaded_shift_m_ > 0.0) {   // a toe that touches but carries nothing: the body goes over it first
        for (int k = 0; k < 5; ++k) {
            // first the leg that just landed and never came to bear (the loop's signature: it would be lifted again at once), then any toe unloaded for unloaded_ticks
            int i; bool fire;
            if (k == 0) {
                i = book_.last_landed_; fire = false;
                if (i >= 0 && in_.have_load_) {
                    if (book_.land_max_load_[size_t(i)] < book_.unloaded_share_) ++book_.unloaded_landings_[size_t(i)]; else book_.unloaded_landings_[size_t(i)] = 0;
                    fire = book_.unloaded_landings_[size_t(i)] >= book_.unloaded_landings_min_ && book_.unload_cool_[size_t(i)] <= 0;
                    book_.land_max_load_[size_t(i)] = 0.0;
                }
                book_.last_landed_ = -1;
            }
            else { i = k - 1; fire = book_.unloaded_run_[size_t(i)] >= book_.unloaded_ticks_ && book_.unload_cool_[size_t(i)] <= 0 && last_plan_skips_ >= book_.unloaded_skips_min_; }
            if (!fire) continue;
            double mx = 0, mz = 0; for (int j = 0; j < 4; ++j) { mx += ex_.toe_tgt_[size_t(j)].x * 0.25; mz += ex_.toe_tgt_[size_t(j)].z * 0.25; }
            const double dx = ex_.toe_tgt_[size_t(i)].x - mx, dz = ex_.toe_tgt_[size_t(i)].z - mz, dn = std::max(1e-6, std::hypot(dx, dz));
            StepIntent out; out.kind = StepIntent::Unload; out.leg = i; out.sx = dx / dn * book_.unloaded_shift_m_; out.sz = dz / dn * book_.unloaded_shift_m_;
            book_.unload_cool_[size_t(i)] = 4 * book_.unloaded_ticks_; book_.unloaded_run_[size_t(i)] = 0; ++unload_shifts_;
            return out;
        }
    }
    if (plan_ > 0.0) {
        int l; Vec3f L; double sx, sz, sc;
        if (plan_step(l, L, sx, sz, sc)) { StepIntent out; out.kind = StepIntent::Plan; out.leg = l; out.land = L; out.sx = sx; out.sz = sz; out.score = sc; ++plans_; return out; }
        ++plan_none_;
    }
    StepIntent out; out.kind = StepIntent::Choice; out.leg = choose_leg(); return out;
}

// the leg whose toe is furthest behind its neutral place steps next (the crawl's order emerges); among equals, the model's
// safest
int ScaffoldPlanner::choose_leg() {
    if (in_.have_contact_) for (int i = 0; i < 4; ++i) if (!ex_.bearing(i) && ex_.air_ticks_[size_t(i)] >= ex_.air_ticks_min_) return i;   // a toe the creep could not bring to ground is placed first
    if (!order_.empty()) {   // the fixed order: the next leg whose turn it is
        order_pos_ = (order_pos_ + 1) % int(order_.size());
        return std::clamp(int(order_[size_t(order_pos_)]), 0, 3);
    }
    // the toe furthest from its neutral place steps next -- behind it (the body has advanced), or tucked in or out
    // (the body has swayed); the crawl's order emerges from the geometry, and the model's margin breaks ties
    int best = -1; double best_score = -1e9;
    for (int i = 0; i < 4; ++i) {
        const double dx = double(ex_.neutral_[size_t(i)].x - ex_.toe_tgt_[size_t(i)].x), dz = double(ex_.neutral_[size_t(i)].z - ex_.toe_tgt_[size_t(i)].z);
        (void)dx;
        const double away = std::max(0.0, dz);   // behind its place (a sway moves every toe sideways alike; a toe AHEAD of its place is fine where it is)
        const double margin = in_.have_support_ ? double(in_.support_[size_t(10 + 5 * i)]) : 0.0;
        const double age = double(int64_t(ex_.tick_now_) - ex_.last_step_tick_[size_t(i)]) * 1e-6;   // among equals, the leg that stepped longest ago
        const double score = away + 0.2 * margin + age;
        score_[size_t(i)] = score;
        if (score > best_score) { best_score = score; best = i; }
    }
    return best;
}
// is the body shift (sx, sz) feasible with `lifting` in the air: every other toe stays within its reach and sector
bool ScaffoldPlanner::shift_feasible(int lifting, const std::array<Vec3f, 4>& toes, double sx, double sz) const {
    for (int i = 0; i < 4; ++i) {
        if (i == lifting) {   // the lifting toe's reach and sector do not block (it is about to leave the ground); under ext_lifting its extension does
            if (ext_lifting_ > 0.0 && ext_margin_m_ > 0.0) {
                const Vec3f q(float(toes[size_t(i)].x - sx), toes[size_t(i)].y, float(toes[size_t(i)].z - sz));
                const double sa = kin_.ext_slack(i, q);
                const double floor_m = ext_lifting_ * ext_margin_m_;
                if (sa < floor_m && sa < kin_.ext_slack(i, toes[size_t(i)]) - 1e-9) { ++ext_lift_rejects_; return false; }
            }
            continue;
        }
        Vec3f q(float(toes[size_t(i)].x - sx), toes[size_t(i)].y, float(toes[size_t(i)].z - sz));
        const Vec3f hip = kin_.hip1(i); const double r = std::hypot(q.x - hip.x, q.z - hip.z);
        if (r < kin_.p.reach_min_m || r > kin_.p.reach_max_m) return false;
        if (i == protect_ && kin_.ext_slack(i, q) < kin_.ext_slack(i, toes[size_t(i)]) - 1e-4) { ++overdue_rejects_; return false; }   // the overdue leg is not stretched further
        Vec3f probe = q; if (kin_.sector_clamp(i, probe)) return false;
        if (ext_margin_m_ > 0.0) {   // nor closer to its full extension than the margin, unless it was already and the shift does not stretch it further
            const double sa = kin_.ext_slack(i, q);
            if (sa < ext_margin_m_ && sa < kin_.ext_slack(i, toes[size_t(i)]) - 1e-9) { ++ext_rejects_; return false; }
        }
    }
    return true;
}
// the shift test's first refusal, in shift_feasible's own order and without its counters (the blocker instrument): 0 = feasible
int ScaffoldPlanner::shift_refusal(int lifting, const std::array<Vec3f, 4>& toes, double sx, double sz, int& toe) const {
    for (int i = 0; i < 4; ++i) {
        toe = i;
        const Vec3f q(float(toes[size_t(i)].x - sx), toes[size_t(i)].y, float(toes[size_t(i)].z - sz));
        if (i == lifting) {
            if (ext_lifting_ > 0.0 && ext_margin_m_ > 0.0) { const double sa = kin_.ext_slack(i, q); if (sa < ext_lifting_ * ext_margin_m_ && sa < kin_.ext_slack(i, toes[size_t(i)]) - 1e-9) return 6; }
            continue;
        }
        const Vec3f hip = kin_.hip1(i); const double r = std::hypot(q.x - hip.x, q.z - hip.z);
        if (r > kin_.p.reach_max_m) return 1;
        if (r < kin_.p.reach_min_m) return 2;
        if (i == protect_ && kin_.ext_slack(i, q) < kin_.ext_slack(i, toes[size_t(i)]) - 1e-4) return 3;
        Vec3f probe = q; if (kin_.sector_clamp(i, probe)) return 4;
        if (ext_margin_m_ > 0.0) { const double sa = kin_.ext_slack(i, q); if (sa < ext_margin_m_ && sa < kin_.ext_slack(i, toes[size_t(i)]) - 1e-9) return 5; }
    }
    toe = -1; return 0;
}
// the blocker instrument: of the shifts that would give `leg` its lift margin, the smallest one's refusal (or "grid")
void ScaffoldPlanner::diagnose_block(int leg, double cx, double cz, double need) {
    const int nl = int(std::floor(plan_shift_m_ / plan_cell_m_ + 1e-9)), nf = int(std::floor(plan_shift_fwd_m_ / plan_cell_m_ + 1e-9));
    double best_d = 1e9, mmax = -1e9; int best_toe = -1, best_r = 0;
    for (int ix = -nl; ix <= nl; ++ix) for (int iz = -nl; iz <= nf; ++iz) {
        const double tx = ix * plan_cell_m_, tz = iz * plan_cell_m_;
        std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
        for (int i = 0; i < 4; ++i) if (i != leg && kk < 3) { t3[size_t(kk)] = {double(ex_.toe_tgt_[size_t(i)].x) - tx, double(ex_.toe_tgt_[size_t(i)].z) - tz}; ++kk; }
        double a, b; const double m = tri_margin(t3, cx, cz, a, b);
        mmax = std::max(mmax, m);
        if (m < need) continue;
        const double d = std::hypot(tx, tz); if (d >= best_d) continue;
        int toe; const int r = shift_refusal(leg, ex_.toe_tgt_, tx, tz, toe);
        best_d = d; best_r = r; best_toe = r == 0 ? leg : toe;
    }
    if (best_toe < 0) { ++block_[size_t(leg)][28]; block_short_[size_t(leg)] += need - mmax; return; }
    ++block_[size_t(leg)][size_t(best_toe * 7 + best_r)]; block_d_[size_t(leg)] += best_d;
}
// the best lift margin `leg` can be given by a feasible shift of the body over `toes` (the centre of mass at (cx, cz) in the
// body frame moves with the body; the toes move the other way); forward_first prefers, among shifts that give the margin
// asked, the most forward one, else the largest margin
bool ScaffoldPlanner::landing_ok(int c, const Vec3f& L) const {
    const Vec3f hip = kin_.hip1(c);
    const double r = std::hypot(L.x - hip.x, L.z - hip.z); if (r < kin_.p.reach_min_m || r > kin_.p.reach_max_m - 0.01) return false;
    if (land_r_min_m_ > 0.0 && r < land_r_min_m_) return false;
    { Vec3f probe = L; if (kin_.sector_clamp(c, probe, std::min(kin_.p.sector_half_rad, ex_.land_sector_rad_))) return false; }
    const Vec3f deep(L.x, float(-ex_.stand_h_m_ - ex_.land_band_lo_m_), L.z); auto ang = ex_.angles_[size_t(c)]; bool ok = false;
    for (int it = 0; it < 8 && !ok; ++it) ok = kin_.ik_step(c, deep, ang);
    return ok || len(sub(kin_.toe_of(c, ang), deep)) <= 0.008;
}
// how far the model's lift margin for `leg` sits BELOW the geometric one at the present pose (<= 0; 0 when off, or with a toe not bearing: the model vetoes only with all four down)
double ScaffoldPlanner::model_bias(int leg, double cx, double cz) {
    if (plan_model_margin_ <= 0.0 || !in_.have_support_ || int(in_.support_[1] + 0.5f) < 4) return 0.0;
    std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
    for (int i = 0; i < 4; ++i) if (i != leg && kk < 3) { t3[size_t(kk)] = {double(ex_.toe_tgt_[size_t(i)].x), double(ex_.toe_tgt_[size_t(i)].z)}; ++kk; }
    double a, b; const double m_geo = tri_margin(t3, cx, cz, a, b);
    const double bias = plan_model_margin_ * std::min(0.0, double(in_.support_[size_t(10 + 5 * leg)]) - m_geo);
    if (bias < -1e-3) { ++model_bias_n_; model_bias_sum_ -= bias; }
    return bias;
}
double ScaffoldPlanner::best_margin_over_shifts(int leg, const std::array<Vec3f, 4>& toes, double cx, double cz, double& sx, double& sz, bool forward_first, double bias) const {
    double best = -1e9, best_key = -1e9; sx = 0.0; sz = 0.0;
    const double need = floor_.margin_min_m_ + floor_.margin_extra_m_;
    const int nl = int(std::floor(plan_shift_m_ / plan_cell_m_ + 1e-9)), nf = int(std::floor(plan_shift_fwd_m_ / plan_cell_m_ + 1e-9));
    for (int ix = -nl; ix <= nl; ++ix) for (int iz = -nl; iz <= nf; ++iz) {
        const double tx = ix * plan_cell_m_, tz = iz * plan_cell_m_;
        if (!shift_feasible(leg, toes, tx, tz)) continue;
        std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
        for (int i = 0; i < 4; ++i) if (i != leg && kk < 3) { t3[size_t(kk)] = {double(toes[size_t(i)].x) - tx, double(toes[size_t(i)].z) - tz}; ++kk; }
        double a, b; double m = tri_margin(t3, cx, cz, a, b);
        if (bias != 0.0) m += bias;   // the model's margin, predicted (plan_model_margin)
        // the key: with the margin met, forward progress then the shortest shift; without it, the margin itself
        const double key = forward_first ? (m >= need ? 10.0 + tz - 0.1 * std::hypot(tx, tz) : m) : m;
        if (key > best_key) { best_key = key; best = m; sx = tx; sz = tz; }
    }
    return best;
}
void ScaffoldPlanner::check_drift(double cx, double cz, double need) {
    if (pred_leg_ < 0) return;
    const int p = pred_leg_, st = pred_step_; pred_leg_ = -1;
    // the extra searches must not move the shift test's own counters
    const int64_t c1 = ext_rejects_, c2 = ext_lift_rejects_, c3 = overdue_rejects_;
    double sx, sz;
    auto margin = [&](const std::array<Vec3f, 4>& toes, double x, double z) { return best_margin_over_shifts(p, toes, x, z, sx, sz, false); };
    const double m_act = margin(ex_.toe_tgt_, cx, cz);
    double dl = 0.0, doth = 0.0; int no = 0;
    for (int i = 0; i < 4; ++i) {
        const double d = std::hypot(double(ex_.toe_tgt_[size_t(i)].x) - pred_toes_[size_t(i)].x, double(ex_.toe_tgt_[size_t(i)].z) - pred_toes_[size_t(i)].z);
        if (i == st) dl = d; else if (i != p) { doth += d; ++no; }
    }
    doth /= std::max(1, no); const double dc = std::hypot(cx - pred_cx_, cz - pred_cz_);
    ++drift_[0]; drift_mag_all_[0] += dl; drift_mag_all_[1] += doth; drift_mag_all_[2] += dc;
    if (pred_m_ < need) ++drift_[3];
    else {
        ++drift_[1];
        if (m_act < need) {
            ++drift_[2]; drift_mag_[0] += dl; drift_mag_[1] += doth; drift_mag_[2] += dc;
            auto swapped = [&](bool land, bool others) { std::array<Vec3f, 4> t = pred_toes_; for (int i = 0; i < 4; ++i) { if (i == p) continue; if ((i == st && land) || (i != st && others)) t[size_t(i)] = ex_.toe_tgt_[size_t(i)]; } return t; };
            const bool by_land = margin(swapped(true, false), pred_cx_, pred_cz_) < need;
            const bool by_oth = margin(swapped(false, true), pred_cx_, pred_cz_) < need;
            const bool by_com = margin(pred_toes_, cx, cz) < need;
            if (by_land) ++drift_cause_[0];
            if (by_oth) ++drift_cause_[1];
            if (by_com) ++drift_cause_[2];
            if (!by_land && !by_oth && !by_com) ++drift_cause_[3];
        }
    }
    ext_rejects_ = c1; ext_lift_rejects_ = c2; overdue_rejects_ = c3;
}
bool ScaffoldPlanner::plan_step(int& leg_out, Vec3f& land_out, double& sx_out, double& sz_out, double& score_out) {
    const double cx = in_.have_support_ ? double(in_.support_[3]) : 0.0, cz = in_.have_support_ ? double(in_.support_[2]) : 0.0;
    const double need = floor_.margin_min_m_ + floor_.margin_extra_m_;
    const int n_order = order_.empty() ? 4 : int(order_.size());
    double best_score = -1e9, best_mnext = -1e9; int best_nleg = -1; std::array<Vec3f, 4> best_landed{}; leg_out = -1;
    const int64_t skips0 = plan_skips_;
    const int nland = int(std::floor(plan_land_m_ / plan_cell_m_ + 1e-9));
    protect_ = -1;
    check_drift(cx, cz, need);
    const int wleg0 = order_.empty() ? 0 : std::clamp(int(order_[size_t((order_pos_ + 1) % n_order)]), 0, 3);
    int urgent = -1;   // the overdue leg's queue jump: the planted leg with the least slack below ext_urgent_m
    if (ext_urgent_m_ > 0.0 && kin_.ext_ready()) { double lo = ext_urgent_m_; for (int i = 0; i < 4; ++i) { const double sl = kin_.ext_slack(i, ex_.toe_tgt_[size_t(i)]); if (sl < lo) { lo = sl; urgent = i; } } }
    if (urgent == wleg0 || jumped_) urgent = -1;   // it is the wave's leg already, or a leg has jumped this turn
    for (int k = (urgent >= 0 ? -1 : 0); k < n_order; ++k) {   // the wave's order first (an urgent leg before it); a later leg only when an earlier one has no feasible step
        const bool jump = k < 0;
        const int pos = jump ? -1 : (order_.empty() ? k : (order_pos_ + 1 + k) % n_order);
        const int c = jump ? urgent : (order_.empty() ? k : std::clamp(int(order_[size_t(pos)]), 0, 3));
        if (!jump && c == urgent) continue;   // tried first, and infeasible
        if (in_.have_contact_ && !ex_.bearing(c) && ex_.air_ticks_[size_t(c)] < ex_.air_ticks_min_ && ex_.air_ticks_[size_t(c)] > 0) {}   // (a toe in the air is still a candidate: placing it is a step)
        auto skipped = [&]{   // the leg has no feasible step: under overdue the steps after it serve it
            if (jump) { ++urgent_fails_; if (overdue_ > 0.0) protect_ = c; return; }
            ++plan_skips_;
            if (k == 0 && overdue_ > 0.0 && protect_ < 0) protect_ = c;
        };
        double sx, sz; const double m_now = best_margin_over_shifts(c, ex_.toe_tgt_, cx, cz, sx, sz, true, model_bias(c, cx, cz));
        if (m_now < need) { ++skip_nomargin_[size_t(c)]; diagnose_block(c, cx, cz, need); skipped(); continue; }
        if (ex_.miss_reacquire_ > 0.0 && ex_.miss_count_[size_t(c)] >= 2 && k < n_order - 1) { ++miss_skips_; ++skip_miss_[size_t(c)]; continue; }   // let the others step first; the last candidate is never skipped
        // the leg the landing must leave liftable: the wave's own next leg (the one being skipped when c is a later leg --
        // the first form protected only the leg after c and marched two legs of one side forward while the other side's
        // never became feasible, 89 / 3 / 90 / 2); when c IS the wave's next leg, the one after it
        const int wpos = order_.empty() ? 0 : (order_pos_ + 1) % n_order;
        const int wleg = order_.empty() ? 0 : std::clamp(int(order_[size_t(wpos)]), 0, 3);
        const int npos = order_.empty() ? (c + 1) % 4 : (pos + 1) % n_order;
        int nleg = (c == wleg) ? (order_.empty() ? npos : std::clamp(int(order_[size_t(npos)]), 0, 3)) : wleg;
        if (protect_ >= 0 && c != protect_) nleg = protect_;   // under overdue the landing must leave the overdue leg liftable
        // the toes after the shift, c still to land
        std::array<Vec3f, 4> after = ex_.toe_tgt_; for (int i = 0; i < 4; ++i) { after[size_t(i)].x -= float(sx); after[size_t(i)].z -= float(sz); }
        const Vec3f hip = kin_.hip1(c), h2 = kin_.hip2(c);
        double hx = h2.x - hip.x, hz = h2.z - hip.z; const double hn = std::max(1e-6, std::hypot(hx, hz)); hx /= hn; hz /= hn;
        const double lx = -hz, lz = hx;   // the heading's lateral
        double leg_best = -1e9, leg_mnext = -1e9; Vec3f leg_land{}; std::array<Vec3f, 4> leg_landed{};
        double jx = 0.0, jz = 0.0;   // re-acquisition: a leg that missed searches a grid shifted by up to 2 cm, and is penalised by 1 cm of margin per miss
        if (ex_.miss_reacquire_ > 0.0 && ex_.miss_count_[size_t(c)] > 0) { auto nz = [&]{ rng_ = rng_ * 1664525u + 1013904223u; return (double(rng_ >> 8) / 16777216.0 - 0.5) * 0.04; }; jx = nz(); jz = nz(); }
        const double miss_pen = ex_.miss_reacquire_ > 0.0 ? 0.01 * ex_.miss_count_[size_t(c)] : 0.0;
        for (int ia = -nland; ia <= nland; ++ia) for (int il = -nland / 2; il <= nland / 2; ++il) {
            const double da = ia * plan_cell_m_ + jz, dl = il * plan_cell_m_ + jx;
            Vec3f L(float(ex_.neutral_[size_t(c)].x + hx * da + lx * dl), ex_.toe_tgt_[size_t(c)].y, float(ex_.neutral_[size_t(c)].z + hz * da + lz * dl));
            const double r = std::hypot(L.x - hip.x, L.z - hip.z); if (r < kin_.p.reach_min_m || r > kin_.p.reach_max_m - 0.01) continue;
            if (land_r_min_m_ > 0.0 && r < land_r_min_m_) { ++land_r_rejects_; continue; }   // the base is kept wide enough that a shift stays feasible
            { Vec3f probe = L; if (kin_.sector_clamp(c, probe, std::min(kin_.p.sector_half_rad, ex_.land_sector_rad_))) continue; }
            {   // the landing must be reachable at the stance's depth plus the landing band (a toe at the horizontal reach cannot lower: 7 of 8 misses sank at r 0.12-0.13)
                const Vec3f deep(L.x, float(-ex_.stand_h_m_ - ex_.land_band_lo_m_), L.z); auto ang = ex_.angles_[size_t(c)]; bool ok = false;
                for (int it = 0; it < 8 && !ok; ++it) ok = kin_.ik_step(c, deep, ang);
                if (!ok && len(sub(kin_.toe_of(c, ang), deep)) > 0.008) continue;
            }
            std::array<Vec3f, 4> landed = after; landed[size_t(c)] = L;
            double nsx, nsz; const double m_next = best_margin_over_shifts(nleg, landed, cx, cz, nsx, nsz, false);
            const double fwd = double(L.z) - double(ex_.neutral_[size_t(c)].z);   // progress: the landing ahead of the neutral place
            double score = std::min(m_next, 0.03) + plan_adv_w_ * fwd - miss_pen;   // a margin beyond 3 cm buys nothing more
            if (protect_ >= 0 && c != protect_ && m_next >= need) score += 0.1;   // a landing that leaves the overdue leg liftable ranks above any that does not
            if (plan_next_first_ > 0.0 && m_next >= need) score += 0.1;   // next leg liftable first: the advance chooses only among landings that leave it liftable
            if (face_gap_m_ > 0.0) {   // lever C: a face just ahead of the landing (a riser the next lift must clear from too close)
                double h0, h1; if (in_.patch_h(L.z, L.x, h0) && in_.patch_h(L.z + face_gap_m_, L.x, h1) && h1 - h0 >= ex_.land_edge_m_) { score -= face_penalty_m_; ++face_penalties_; }
            }
            if (score > leg_best) { leg_best = score; leg_land = L; leg_mnext = m_next; leg_landed = landed; }
        }
        if (leg_best <= -1e8) { ++skip_noland_[size_t(c)]; skipped(); continue; }
        if (jump) { best_score = leg_best; leg_out = c; land_out = leg_land; sx_out = sx; sz_out = sz; best_mnext = leg_mnext; best_nleg = nleg; best_landed = leg_landed; ++urgent_jumps_; jumped_ = true; break; }   // the urgent leg steps; the wave's turn is not consumed
        if (leg_best > best_score) { best_score = leg_best; leg_out = c; land_out = leg_land; sx_out = sx; sz_out = sz; best_mnext = leg_mnext; best_nleg = nleg; best_landed = leg_landed; if (k == 0) { order_pos_ = pos; jumped_ = false; } }   // a later leg's step does not consume the wave's turn
        if (k == 0) break;   // the wave's own next leg is feasible: take it (the search ranks its landing and shift, not its turn)
    }
    if (protect_ >= 0) ++overdue_decisions_;
    if (leg_out >= 0) ++next_lift_[best_mnext >= need ? 0 : 1];
    if (leg_out >= 0) { pred_leg_ = best_nleg; pred_step_ = leg_out; pred_m_ = best_mnext; pred_toes_ = best_landed; pred_cx_ = cx; pred_cz_ = cz; } else pred_leg_ = -1;
    protect_ = -1;
    score_out = best_score; last_plan_skips_ = int(plan_skips_ - skips0);
    return leg_out >= 0;
}
bool ScaffoldPlanner::plan_relief(int stuck, double cx, double cz, int bearing, int& leg_out, Vec3f& land_out, double& margin_out) const {
    double best = -1e9; leg_out = -1;
    for (int c = 0; c < 4; ++c) {
        if (c == stuck) continue;
        {   // c must be safe to lift now, by the geometry and (with all four bearing) by the model
            std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
            for (int i = 0; i < 4; ++i) if (i != c && kk < 3) { t3[size_t(kk)] = {double(ex_.toe_tgt_[size_t(i)].x), double(ex_.toe_tgt_[size_t(i)].z)}; ++kk; }
            double a, b; const double ml = tri_margin(t3, cx, cz, a, b);
            const double mm = in_.have_support_ ? double(in_.support_[size_t(10 + 5 * c)]) : 1.0;
            if (ml < floor_.margin_min_m_ || (bearing >= 4 && mm < floor_.margin_min_m_)) continue;
        }
        const Vec3f hip = kin_.hip1(c), h2 = kin_.hip2(c);
        double hx = h2.x - hip.x, hz = h2.z - hip.z; const double hn = std::max(1e-6, std::hypot(hx, hz)); hx /= hn; hz /= hn;
        for (int k = -relief_cells_; k <= relief_cells_; ++k) {
            Vec3f cand(float(ex_.neutral_[size_t(c)].x + hx * k * relief_step_m_), ex_.toe_tgt_[size_t(c)].y, float(ex_.neutral_[size_t(c)].z + hz * k * relief_step_m_));
            const double r = std::hypot(cand.x - hip.x, cand.z - hip.z);
            if (r < kin_.p.reach_min_m || r > kin_.p.reach_max_m - 0.01) continue;
            Vec3f probe = cand; if (kin_.sector_clamp(c, probe)) continue;
            std::array<std::array<double, 2>, 3> t3{}; int kk = 0;   // the stuck leg's support after c has landed there
            for (int i = 0; i < 4; ++i) if (i != stuck && kk < 3) { const Vec3f& q = (i == c) ? cand : ex_.toe_tgt_[size_t(i)]; t3[size_t(kk)] = {double(q.x), double(q.z)}; ++kk; }
            double a, b; const double m_after = tri_margin(t3, cx, cz, a, b);
            if (m_after > best) { best = m_after; leg_out = c; land_out = cand; }
        }
    }
    margin_out = best;
    return leg_out >= 0 && best >= floor_.margin_min_m_ + floor_.margin_extra_m_;
}

} // namespace ogma::stepper
