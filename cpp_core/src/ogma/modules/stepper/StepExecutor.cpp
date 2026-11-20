// StepExecutor -- see the header.  Split out of PoseSequencer byte-identical (learned stepper plan Phase 2, 2026-11-15): every function body is the
#include "ogma/modules/stepper/StepExecutor.hpp"
#include <algorithm>
#include <cmath>

namespace ogma::stepper {

void StepExecutor::engage_now(uint64_t tick_id) {
    for (int i = 0; i < 4; ++i) {
        std::array<double, 3> u{0.0, 0.0, 0.0};
        if (in_.have_gait_) u = {in_.gait_u_[size_t(3 * i)], in_.gait_u_[size_t(3 * i + 1)], in_.gait_u_[size_t(3 * i + 2)]};
        angles_[size_t(i)] = kin_.angles_of_u(i, u);
        const Vec3f toe = kin_.toe_of(i, angles_[size_t(i)]);
        // the stance: this toe's place in the plane, kept within reach of its hip, at the standing height
        Vec3f t(toe.x, float(-stand_h_m_), toe.z);
        if (kin_.sector_clamp(i, t)) {   // a toe at the edge of its reach gets a centimetre of room, so the first shift is not blocked by it
            const Vec3f hip = kin_.hip1(i); const double dx = hip.x - t.x, dz = hip.z - t.z, dn = std::max(1e-6, std::hypot(dx, dz));
            t.x += float(dx / dn * 0.01); t.z += float(dz / dn * 0.01);
        }
        toe_tgt_[size_t(i)] = t;
        // the neutral place: neutral_r along the leg's heading from its hip, at the standing height
        const Vec3f hip = kin_.hip1(i), h2 = kin_.hip2(i);
        const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::max(1e-6, std::hypot(hx, hz));
        neutral_[size_t(i)] = Vec3f(float(hip.x + hx / hn * neutral_r_m_), float(-stand_h_m_), float(hip.z + hz / hn * neutral_r_m_));
        elev_[size_t(i)] = 0.0; elev_landed_[size_t(i)] = 0.0; ground_h_[size_t(i)] = -stand_h_m_; creep_[size_t(i)] = 0.0; air_ticks_[size_t(i)] = 0;
    }
    reloc_active_ = false;   // (the planner's drift prediction is reset by the shell at engage)
    shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ticks_ = 0; leg_ = -1; phase_ = Phase::Settle; engaged_at_ = int64_t(tick_id); crouch_ = 0.0; ret_h_ = 0.0;
    yaw_ref_ = (heading_ref_ == "engage" && in_.have_odom_in_) ? in_.odom_in_[2] : 0.0; yaw_err_ = 0.0;
}

// the hand-back: from wherever the crawl is to a stand (see the header)
void StepExecutor::return_now(const std::array<Vec3f, 4>& places, double h_m) {
    if (phase_ == Phase::Shift || phase_ == Phase::Settle) leg_ = -1;   // all four are planted (Shift's leg_ is only the next to lift)
    ret_place_ = places; ret_h_goal_m_ = h_m; plan_active_ = false; plan_pending_ = false; relief_for_ = -1; free_pending_ = 0; reloc_active_ = false;
    touch_run_ = 0; phase_ = Phase::Return; phase_ticks_ = 0; ++returns_;
}

// the planted toes' heights: stand_h below the body plus each toe's relative elevation, recentred on their mean
void StepExecutor::hold_stance() {
    // each toe's ground elevation: where it landed (elev_source landed), or from the memory under its place
    if (elev_source_ == "landed") { for (int i = 0; i < 4; ++i) elev_[size_t(i)] = elev_landed_[size_t(i)]; }
    else if (in_.have_patch_) {
        double hs[4]; bool ok[4]; int n_ok = 0; double sum = 0.0;
        for (int i = 0; i < 4; ++i) { ok[i] = in_.patch_h(toe_tgt_[size_t(i)].z, toe_tgt_[size_t(i)].x, hs[i]); if (ok[i]) { sum += hs[i]; ++n_ok; } }
        if (n_ok >= 2) {
            const double a = 1.0 / elev_tau_ticks_;
            for (int i = 0; i < 4; ++i) {
                if (!ok[i]) continue;
                const double others = (sum - hs[i]) / std::max(1, n_ok - 1);
                double e = hs[i] - others; if (std::fabs(e) < elev_deadband_m_) e = 0.0;
                elev_[size_t(i)] += a * (std::clamp(e, -0.1, 0.15) - elev_[size_t(i)]);
            }
        }
    }
    double mean = 0.0; int n = 0;
    for (int i = 0; i < 4; ++i) if (i != leg_ || phase_ == Phase::Shift) { mean += elev_[size_t(i)]; ++n; }
    if (n > 0) mean /= n;
    // the levelling corrections, ZERO-MEAN over the planted toes (an extend-only term ratcheted the body up 2 cm a minute
    // until the legs stood near full extension and a step tipped it, 2026-11-08), each bounded
    if (front_drop_gain_ > 0.0 && in_.have_up_) {   // lever 3: the nose-up pitch beyond the deadband drops the front, through a first-order filter
        const double nose_up = std::atan2(double(in_.up_[2]), std::max(0.2, double(in_.up_[1])));
        const double target = std::clamp(front_drop_gain_ * std::max(0.0, nose_up - front_drop_deadband_), 0.0, front_drop_max_m_);
        front_drop_ += (target - front_drop_) / front_drop_tau_ticks_;
    } else front_drop_ = 0.0;
    std::array<double, 4> lev{}; double lev_mean = 0.0; int n_lev = 0;
    if (in_.have_up_) {
        for (int i = 0; i < 4; ++i) {
            lev[size_t(i)] = std::clamp(level_gain_ * (toe_tgt_[size_t(i)].x * in_.up_[0] + toe_tgt_[size_t(i)].z * in_.up_[2]) / std::max(0.2, in_.up_[1]), -level_max_m_, level_max_m_);
            if (i != leg_ || phase_ == Phase::Shift) { lev_mean += lev[size_t(i)]; ++n_lev; }
        }
        if (n_lev > 0) lev_mean /= n_lev;
    }
    if (crouch_ > 0.0 && phase_ != Phase::Lower) {   // the crouch relaxes while every toe bears and nothing is lowering
        bool all_bear = true; for (int i = 0; i < 4; ++i) if (!bearing(i)) all_bear = false;
        if (all_bear) crouch_ = std::max(0.0, crouch_ - crouch_relax_v_ * dt_);
    }
    for (int i = 0; i < 4; ++i) {
        if (i == leg_ && phase_ != Phase::Shift) { creep_[size_t(i)] = 0.0; air_ticks_[size_t(i)] = 0; continue; }   // the stepping toe is the machine's
        // the load-seeking creep: down while the flag is off, back while it bears
        if (book_.unloaded_shift_m_ > 0.0 && in_.have_contact_ && in_.have_load_) {   // the unloaded toe: touching (the flag on) but carrying nothing
            if (double(in_.load_[size_t(i)]) < book_.unloaded_share_) ++book_.unloaded_run_[size_t(i)]; else book_.unloaded_run_[size_t(i)] = 0;   // the flag may flicker on an unloaded toe; the load is the fact
            book_.land_max_load_[size_t(i)] = std::max(book_.land_max_load_[size_t(i)], double(in_.load_[size_t(i)]));
            if (book_.unload_cool_[size_t(i)] > 0) --book_.unload_cool_[size_t(i)];
        }
        if (in_.have_contact_) {
            if (!bearing(i)) ++air_ticks_[size_t(i)]; else air_ticks_[size_t(i)] = 0;   // airborne by the flag (the re-step's rule)
            if (creep_load_min_ > 0.0 && in_.have_load_) {   // by the LOAD: down while unloaded, back only while it carries its share
                if (double(in_.load_[size_t(i)]) < creep_load_min_) { creep_[size_t(i)] = std::min(creep_max_m_, creep_[size_t(i)] + creep_v_ * dt_); ++creep_load_ticks_; }
                else if (double(in_.load_[size_t(i)]) > creep_load_relax_) creep_[size_t(i)] = std::max(0.0, creep_[size_t(i)] - creep_relax_v_ * dt_);
            } else {
                if (!bearing(i)) creep_[size_t(i)] = std::min(creep_max_m_, creep_[size_t(i)] + creep_v_ * dt_);
                else creep_[size_t(i)] = std::max(0.0, creep_[size_t(i)] - creep_relax_v_ * dt_);
            }
        }
        // BODY-frame heights (equal legs = a level body on flat ground) plus a LEVELLING term from the IMU: the side the
        // body's up vector leans away from is low, and its toes are sent deeper by level_gain of the geometric correction
        // (x ux + z uz) / uy.  (Equal GRAVITY heights were tried, 2026-11-08: four toes on a flat floor share one gravity
        // height at any roll, so that stance held whatever attitude the body had and it rolled to the disengage limit.)
        toe_tgt_[size_t(i)].y = float(-(stand_h_m_ - crouch_ - ret_h_) + std::clamp(elev_[size_t(i)] - mean, -elev_max_m_, elev_max_m_) - creep_[size_t(i)] + (lev[size_t(i)] - lev_mean));
        if (front_drop_ > 0.0 && toe_tgt_[size_t(i)].z > 0.0f) toe_tgt_[size_t(i)].y += float(front_drop_);   // the front of the chassis comes down
        if (depth_max_m_ > 0.0 && toe_tgt_[size_t(i)].y < float(-depth_max_m_)) { toe_tgt_[size_t(i)].y = float(-depth_max_m_); ++depth_caps_; }
    }
}

void StepExecutor::step_machine() {
    ++phase_ticks_;
    const double adv = shift_v_ * dt_;
    switch (phase_) {
    case Phase::Idle: break;
    case Phase::Settle: {   // the toes slide to their neutral places; then the crawl begins
        bool all_there = true; const double step = settle_v_ * dt_;
        for (int i = 0; i < 4 && settle_ > 0.0; ++i) {
            Vec3f& tg = toe_tgt_[size_t(i)]; const double dx = neutral_[size_t(i)].x - tg.x, dz = neutral_[size_t(i)].z - tg.z, dn = std::hypot(dx, dz);
            if (dn <= step) { tg.x = neutral_[size_t(i)].x; tg.z = neutral_[size_t(i)].z; } else { tg.x += float(dx / dn * step); tg.z += float(dz / dn * step); all_there = false; }
        }
        ++settle_ticks_;
        if (all_there && phase_ticks_ > 10) { decide_next(tick_now_); shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ = Phase::Shift; phase_ticks_ = 0; }
        break; }
    case Phase::Return: {   // the hand-back: the stepping toe down, then the toes to their places and the body to its height; then Idle
        ++return_ticks_;
        if (leg_ >= 0) {
            Vec3f& tg = toe_tgt_[size_t(leg_)];
            const Vec3f cmd = kin_.toe_of(leg_, angles_[size_t(leg_)]);
            tg.y -= float(lower_v_ * dt_);
            if (depth_max_m_ > 0.0 && tg.y < float(-depth_max_m_)) tg.y = float(-depth_max_m_);
            const bool raw_touch = in_.have_contact_ && in_.contact_[size_t(leg_)] > 0.5f && (!in_.have_load_ || in_.load_[size_t(leg_)] >= float(touch_load_));
            touch_run_ = raw_touch ? touch_run_ + 1 : 0;
            const bool touched = touch_run_ >= touch_ticks_ && phase_ticks_ > 3;
            const bool low = double(tg.y) <= -(stand_h_m_ - crouch_) - sink_max_m_ || phase_ticks_ > 4 * phase_max_ticks_;
            if (touched || low) { elev_landed_[size_t(leg_)] = double(cmd.y) + stand_h_m_; tg = cmd; leg_ = -1; phase_ticks_ = 0; ++return_lowered_; }   // the stance takes it from here
            break;
        }
        bool all_there = true; const double step = settle_v_ * dt_;
        for (int i = 0; i < 4; ++i) {
            Vec3f& tg = toe_tgt_[size_t(i)]; const double dx = ret_place_[size_t(i)].x - tg.x, dz = ret_place_[size_t(i)].z - tg.z, dn = std::hypot(dx, dz);
            if (dn <= step) { tg.x = ret_place_[size_t(i)].x; tg.z = ret_place_[size_t(i)].z; } else { tg.x += float(dx / dn * step); tg.z += float(dz / dn * step); all_there = false; }
        }
        if (ret_h_goal_m_ > 0.0) {   // the body to the height asked for (the crouch, relaxing on its own, is counted in)
            const double goal = std::clamp(stand_h_m_ - crouch_ - ret_h_goal_m_, -ret_h_max_m_, ret_h_max_m_), dh = goal - ret_h_, hs = crouch_v_ * dt_;
            if (std::fabs(dh) <= hs) ret_h_ = goal; else { ret_h_ += dh > 0.0 ? hs : -hs; all_there = false; }
        }
        const bool late = phase_ticks_ > return_max_ticks_;
        if ((all_there && phase_ticks_ > 10) || late) { if (late) ++return_timeouts_; phase_ = Phase::Idle; leg_ = -1; }
        break; }
    case Phase::Shift: {
        // the body's move: into the triangle the other toes make, by the margin still missing, and toward the toes' centroid
        const double cx = in_.have_support_ ? double(in_.support_[3]) : 0.0, cz = in_.have_support_ ? double(in_.support_[2]) : 0.0;   // the model's centre of mass (lat = x, fwd = z)
        std::array<std::array<double, 2>, 3> tri{}; int k = 0; double mx = 0, mz = 0;
        for (int i = 0; i < 4; ++i) { mx += toe_tgt_[size_t(i)].x * 0.25; mz += toe_tgt_[size_t(i)].z * 0.25; if (i != leg_ && k < 3) { tri[size_t(k)] = {double(toe_tgt_[size_t(i)].x), double(toe_tgt_[size_t(i)].z)}; ++k; } }
        double nx = 0, nz = 0; const double m_local = tri_margin(tri, cx, cz, nx, nz);
        const double model_margin = in_.have_support_ ? double(in_.support_[size_t(10 + 5 * leg_)]) : 1.0;
        const int bearing = in_.have_support_ ? int(in_.support_[1] + 0.5f) : 4;
        // the margin still missing, by the local geometry and -- when all four toes bear and the model disagrees -- by the model
        const double m_req = floor_.margin_min_m_ + floor_.margin_extra_m_ + (free_form_ == 3 && free_pending_ > 0 ? free_margin_m_ : 0.0);   // a freeing lift asks for more
        double need = std::max(0.0, m_req - m_local);
        if (bearing >= 4) need = std::max(need, m_req - model_margin);
        double fwd_left = std::max(0.0, advance_m_ - fwd_acc_);
        if (advance_reach_m_ > 0.0) {   // a planted toe near its reach limit: the body does not walk away from it
            bool stretched = false;
            for (int i = 0; i < 4; ++i) { if (i == leg_) continue; const Vec3f hip = kin_.hip1(i); if (std::hypot(toe_tgt_[size_t(i)].x - hip.x, toe_tgt_[size_t(i)].z - hip.z) >= kin_.p.reach_max_m - advance_reach_m_) stretched = true; }
            if (stretched && fwd_left > 0.0) { fwd_left = 0.0; ++advance_yields_; }
        }
        const double cg = centre_off_ ? 0.0 : centre_gain_;   // the centroid's pull yields when it has cancelled the margin's (stagnation)
        double dx = need * nx + cg * (mx - cx), dz = need * nz + cg * (mz - cz) + std::min(adv, fwd_left);
        bool plan_reached = false;
        if (plan_active_) {   // the planned shift, walked at the shift's rate; the margin controller's move is replaced, its safety check kept
            const double lx = plan_sx_ - plan_done_x_, lz = plan_sz_ - plan_done_z_, ln = std::hypot(lx, lz);
            if (ln <= adv) { dx = lx; dz = lz; plan_reached = true; } else { dx = lx / ln * adv; dz = lz / ln * adv; }
            plan_done_x_ += dx; plan_done_z_ += dz;
        }
        {   // the advance yields to the margin: if the move would leave the weight short of the support that will remain, drop it
            double dn0 = std::hypot(dx, dz); double sx = dx, sz = dz; if (dn0 > adv) { sx *= adv / dn0; sz *= adv / dn0; }
            std::array<std::array<double, 2>, 3> tri2 = tri; for (auto& q : tri2) { q[0] -= sx; q[1] -= sz; }
            double n2x, n2z; const double m_after = tri_margin(tri2, cx, cz, n2x, n2z);
            if (fwd_left > 0.0 && m_after < m_local && m_after < floor_.margin_min_m_ + floor_.margin_extra_m_) { dz -= std::min(adv, fwd_left); fwd_blocked_ = true; }
        }
        const double dn = std::hypot(dx, dz);
        if (dn > adv) { dx *= adv / dn; dz *= adv / dn; }
        shift_dx_ = dx; shift_dz_ = dz; fwd_acc_ += std::max(0.0, dz);
        {   // the sequencer's odometry: the body moved (dx, dz) in its own frame; forward along the dead-reckoning frame's heading
            const double yaw = in_.have_odom_in_ ? in_.odom_in_[2] : 0.0, c = std::cos(yaw), sn = std::sin(yaw);
            px_ += shift_realised_ * (dz * c - dx * sn); py_ += shift_realised_ * (dz * sn + dx * c);
        }
        if (heading_hold_ > 0.0 && in_.have_odom_in_) {   // the heading hold: turn the body back by rotating the planted toes the other way
            double e = in_.odom_in_[2] - yaw_ref_; while (e > M_PI) e -= 2 * M_PI; while (e < -M_PI) e += 2 * M_PI;
            yaw_err_ = e;
            const double a = std::clamp(heading_hold_ * heading_gain_ * e, -yaw_rate_max_, yaw_rate_max_) * dt_;
            const double ca = std::cos(a), sa = std::sin(a);
            for (int i = 0; i < 4; ++i) { Vec3f& q = toe_tgt_[size_t(i)]; const double x = q.x, z = q.z; q.x = float(x * ca + z * sa); q.z = float(-x * sa + z * ca); }
        }
        // a shift that has stopped moving (the margin's pull cancels the advance) has no advance to give this leg
        if (std::hypot(dx, dz) < 0.1 * adv) { if (++still_ticks_ >= 10) fwd_blocked_ = true; } else still_ticks_ = 0;
        bool blocked = false; blocked_leg_ = -1; blocked_toe_ = {false, false, false, false};
        for (int i = 0; i < 4; ++i) {
            toe_tgt_[size_t(i)].x -= float(dx); toe_tgt_[size_t(i)].z -= float(dz);
            const bool clamped = kin_.sector_clamp(i, toe_tgt_[size_t(i)]);
            if (clamped && !(plan_active_ && i == leg_)) { blocked = true; blocked_toe_[size_t(i)] = true; if (blocked_leg_ < 0) blocked_leg_ = i; }   // under a plan the lifting toe's own clamp does not block: it is about to leave the ground (it blocked 9 of 10 planned shifts)
        }   // the toes move the other way, as far as they reach
        shift_acc_ += std::hypot(dx, dz); ++shift_ticks_;
        if (blocked) ++shift_blocked_;
        // may the leg go?  By the geometry the shift is steering on; the model vetoes only when all four toes bear (with
        // fewer, a flickering flag makes every other leg's margin the floor, and the shift would run to its bound)
        const bool safe_now = m_local >= floor_.margin_min_m_ && (bearing < 4 || model_margin >= floor_.margin_min_m_);
        if (unload_shift_only_ && plan_active_ && (plan_reached || blocked)) {   // the body has gone over the unloaded toe: decide afresh
            unload_shift_only_ = false; decide_next(tick_now_); shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ticks_ = 0;
            break;
        }
        bool go = safe_now && (fwd_acc_ >= advance_m_ || fwd_blocked_ || blocked || (plan_active_ && plan_reached));
        if (plan_active_ && plan_reached) { ++plan_reached_; if (!safe_now) fwd_blocked_ = true; }   // arrived without the margin: the spent logic below takes over
        if (go && fwd_acc_ < advance_m_) ++steps_no_advance_;
        // a shift standing still without the leg being safe: first let the margin's pull act alone, then treat as spent
        bool spent = blocked || shift_acc_ >= shift_max_m_;
        if (!go && !safe_now && still_ticks_ >= 10) { if (!centre_off_) { centre_off_ = true; still_ticks_ = 0; } else spent = true; }
        if (!go && spent) {
            // the body can move no further for this leg: take the toe that blocks it if ITS margin allows, else the leg the
            // model rates safest if that one's does; a leg without a margin is never lifted -- better to stand than to fall
            auto margin_of_leg = [&](int i) { return in_.have_support_ ? double(in_.support_[size_t(10 + 5 * i)]) : 1.0; };
            auto local_margin_of_leg = [&](int c) {
                std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
                for (int i = 0; i < 4; ++i) if (i != c && kk < 3) { t3[size_t(kk)] = {double(toe_tgt_[size_t(i)].x), double(toe_tgt_[size_t(i)].z)}; ++kk; }
                double a, b; return tri_margin(t3, cx, cz, a, b);
            };
            int cand = (blocked && blocked_leg_ >= 0) ? blocked_leg_ : -1;
            if (cand >= 0 && local_margin_of_leg(cand) < floor_.margin_min_m_) cand = -1;
            if (cand < 0 || margin_of_leg(cand) < floor_.margin_min_m_) {
                // the safest leg among those that NEED a step (behind their place): a toe already ahead has nothing to gain
                double bm = -1e9; cand = -1;
                for (int i = 0; i < 4; ++i) { const bool needs = neutral_[size_t(i)].z - toe_tgt_[size_t(i)].z > 0.0; if (needs && margin_of_leg(i) > bm) { bm = margin_of_leg(i); cand = i; } }
                if (cand < 0) for (int i = 0; i < 4; ++i) if (margin_of_leg(i) > bm) { bm = margin_of_leg(i); cand = i; }
                if (cand >= 0 && bm < floor_.margin_min_m_) cand = -1;
            }
            if (!order_ref_->empty()) {   // under the fixed order the turn passes to the next leg; a toe at its edge takes it if it is safe
                int rl = -1; Vec3f rland; double rm = 0.0;
                if (cand >= 0 && cand == blocked_leg_ && cand != leg_) { leg_ = cand; ++shift_exhausted_; go = true; }
                else if (relief_ > 0.0 && blocked_toe_[size_t(leg_)] && relief_for_ < 0 && relief_hook_(leg_, cx, cz, bearing, rl, rland, rm)) {
                    relief_for_ = leg_; leg_ = rl; relief_land_ = rland; relief_pending_ = true; ++relief_steps_; go = true;   // the stuck leg's turn is retried when the relief leg lands
                } else {
                    if (relief_ > 0.0 && blocked_toe_[size_t(leg_)]) ++relief_none_;
                    ++turns_passed_; decide_next(tick_now_); shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ticks_ = 0;
                }
            }
            else if (cand >= 0) { if (cand != leg_) leg_ = cand; ++shift_exhausted_; go = true; }
            else ++deadlock_ticks_;
        }
        if (go) {
            const Vec3f cur = toe_tgt_[size_t(leg_)];
            // the landing point: the neutral place plus half a stride ahead, kept within reach of the hip
            const bool was_planned = plan_pending_ && !relief_pending_;   // (the tag's: read before the landing consumes it)
            if (relief_pending_) { land_ = Vec3f(relief_land_.x, cur.y, relief_land_.z); relief_pending_ = false; }
            else if (plan_pending_) { land_ = Vec3f(plan_land_.x, cur.y, plan_land_.z); plan_pending_ = false; }
            else land_ = Vec3f(neutral_[size_t(leg_)].x, cur.y, float(neutral_[size_t(leg_)].z + stride_m_ * 0.5));
            if (!reloc_active_) {   // the tag starts at the landing the step sets out for (a re-go after a hang keeps its track)
                reloc_active_ = true; reloc_vec_ = {}; reloc_from_ = land_; reloc_hang_ = false; go_tick_ = tick_now_;
                reloc_planned_ = was_planned && leg_ == reloc_plan_leg_; reloc_switched_ = was_planned && leg_ != reloc_plan_leg_;
            }
            {   // a centimetre inside the reach, so the swing can finish and the toe can still lower
                const Vec3f hip = kin_.hip1(leg_); const double dx = land_.x - hip.x, dz = land_.z - hip.z, r = std::hypot(dx, dz), rm = kin_.p.reach_max_m - 0.01;
                const Vec3f b0 = land_;
                if (land_clamp_ > 0.0 && r > rm) { land_.x = float(hip.x + dx * rm / r); land_.z = float(hip.z + dz * rm / r); }
                reloc_add(RelReach, b0, land_); const Vec3f b1 = land_;
                if (land_clamp_ > 0.0) kin_.sector_clamp(leg_, land_, std::min(kin_.p.sector_half_rad, land_sector_rad_));
                reloc_add(RelSector, b1, land_);
            }
            // the ground: under the toe now and at the landing, from the memory where it has it, else the stance's own height
            const double stance_h = in_.gravity_h(Vec3f(cur.x, float(-stand_h_m_), cur.z));
            const double h_here = in_.gravity_h(cur);                   // the toe stands here: the ground under it is where it is
            if (land_edge_m_ > 0.0) {   // a riser's edge under the landing: move it along the heading, a cell at a time, to flat remembered ground
                double relief; const Vec3f hip = kin_.hip1(leg_); Vec3f best = land_; bool moved = false;
                if (in_.patch_edge(land_.z, land_.x, relief) && relief >= land_edge_m_) {
                    for (int dir : {+1, -1}) {   // forward first (onto the tread), then back
                        for (int k = 1; k * in_.cell_ <= land_edge_max_m_ + 1e-9; ++k) {
                            Vec3f cand(land_.x, land_.y, float(land_.z + dir * k * in_.cell_));
                            if (std::hypot(cand.x - hip.x, cand.z - hip.z) > kin_.p.reach_max_m - 0.01) break;
                            double r2; if (in_.patch_edge(cand.z, cand.x, r2) && r2 < land_edge_m_) { best = cand; moved = true; break; }
                        }
                        if (moved) break;
                    }
                    if (moved) { reloc_add(RelEdge, land_, best); land_ = best; ++edge_moves_; } else ++edge_stuck_;
                }
            }
            double h_land = stance_h, hp, sg = 0.0; land_sigma_ = 0.0;
            if (in_.patch_h(land_.z, land_.x, hp, sg)) {                 // the patch's frame: fwd = z, lat = x, as TerrainMemory stores the toes
                if (hp >= stance_h - land_band_lo_m_ && hp <= stance_h + land_band_hi_m_) { h_land = hp; land_sigma_ = sg; } else ++land_h_rejected_;
            }
            land_h_ = h_land; lift_h_ = std::max(h_here, h_land) + lift_clear_m_ + (free_form_ == 3 && free_pending_ > 0 ? free_extra_m_ + free_out_dy_ : 0.0);
            lift_dy_ = lift_h_ - stance_h;                           // the lift as a BODY-frame rise over the stance (a gravity conversion on a rolling body ran a target up and the leg into its stops, 2026-11-08)
            if (miss_reacquire_ > 0.0 && miss_count_[size_t(leg_)] > 0) { const double drop = std::min(0.01 * miss_count_[size_t(leg_)], std::max(0.0, lift_clear_m_ - 0.015)); lift_h_ -= drop; lift_dy_ -= drop; }   // a re-attempt lifts lower
            phase_ = Phase::Lift; phase_ticks_ = 0; left_ground_ = false; obstacle_run_ = 0; hang_run_ = 0; swing_v_fwd_ = 0.0; swing_d_fwd_ = 0.0; clip_run_ = 0; lower_retreats_ = 0; lift_from_ = cur;
        }
        break; }
    case Phase::Lift:
    case Phase::Swing:
        if (hang_ticks_ > 0 && in_.have_contact_) {   // THE HUNG TOE: asked up but never off the ground, or the body dragged back by it
            const bool up_asked = phase_ == Phase::Swing || double(toe_tgt_[size_t(leg_)].y) + stand_h_m_ >= 0.02;
            const bool on = in_.contact_[size_t(leg_)] > 0.5f;
            if (up_asked && on && !left_ground_) ++hang_run_; else if (!on) hang_run_ = 0;
            if (in_.have_imu_ && in_.have_up_) {   // the forward linear acceleration (gravity removed by the body's up), integrated over this lift+swing
                const double a_lin_z = double(in_.imu_[2]) - 9.81 * in_.up_[2]; a_lin_z_ema_ += 0.01 * (a_lin_z - a_lin_z_ema_);
                swing_v_fwd_ += (a_lin_z - a_lin_z_ema_) * dt_; swing_d_fwd_ += swing_v_fwd_ * dt_;
            }
            const bool caught = hang_run_ >= hang_ticks_, dragged = drag_back_m_ > 0.0 && swing_d_fwd_ < -drag_back_m_;
            if ((caught || dragged) && free_pending_ < 3) {
                ++hangs_; reloc_hang_ = reloc_active_; if (caught) ++hangs_by_contact_; if (dragged) ++hangs_by_drag_; ++hang_count_[size_t(leg_)];
                free_pending_ = std::min(2, hang_count_[size_t(leg_)]);
                Vec3f& tg = toe_tgt_[size_t(leg_)]; tg.y = float(-stand_h_m_);
                if (free_form_ == 3) {
                    // the toe set down where it is; the body unloads it by the margin shift; then a full lift, higher and outward (noise on the second attempt)
                    auto noise = [&]{ rng_ = rng_ * 1664525u + 1013904223u; return (double(rng_ >> 8) / 16777216.0 - 0.5) * 2.0 * free_noise_m_; };
                    const double out_sign = (leg_ == 1 || leg_ == 3) ? 1.0 : -1.0;
                    free_out_dx_ = out_sign * free_out_m_ * free_pending_ + (free_pending_ >= 2 ? noise() : 0.0);
                    free_out_dy_ = free_pending_ >= 2 ? noise() : 0.0;
                    plan_active_ = false; plan_pending_ = true; plan_land_ = land_;   // the landing kept; the margin controller steers the shift
                    shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ = Phase::Shift; phase_ticks_ = 0; hang_run_ = 0;
                } else {
                    // the toe dragged toward the hip along the ground, then lifted higher
                    free_retract_left_ = free_retract_m_ * free_pending_;
                    lift_h_ += free_extra_m_; lift_dy_ += free_extra_m_;
                    phase_ = Phase::Lift; phase_ticks_ = 0; hang_run_ = 0; left_ground_ = false; obstacle_run_ = 0; swing_v_fwd_ = 0.0; swing_d_fwd_ = 0.0;
                }
                if (free_pending_ >= 2) free_pending_ = 3;   // the second attempt on this leg is the last: a third hang lets the timeouts run (no loop)
                break;
            }
        }
        if (in_.have_contact_ && double(toe_tgt_[size_t(leg_)].y) + stand_h_m_ >= 0.02) {   // the obstacle reflex, armed once the toe is asked 2 cm up: it left the ground and something stopped it
            const bool on = in_.contact_[size_t(leg_)] > 0.5f;
            if (!on) { left_ground_ = true; obstacle_run_ = 0; }
            else if (left_ground_ && obstacle_ticks_ > 0 && ++obstacle_run_ >= obstacle_ticks_) {   // sustained a tenth of a second: a brush of the floor is briefer
                ++obstacle_hits_; reloc_add(RelObstacle, land_, toe_tgt_[size_t(leg_)]); land_ = toe_tgt_[size_t(leg_)]; land_h_ = -stand_h_m_;
                phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; left_ground_ = false; obstacle_run_ = 0;
                break;
            }
        }
        if (phase_ == Phase::Lift) {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        if (free_retract_left_ > 0.0) {   // freeing: first drag the toe along the ground toward the hip, away from the face
            const Vec3f hip = kin_.hip1(leg_); const double dx = hip.x - tg.x, dz = hip.z - tg.z, dn = std::max(1e-6, std::hypot(dx, dz)), st = std::min(free_retract_left_, swing_v_ * dt_);
            tg.x += float(dx / dn * st); tg.z += float(dz / dn * st); free_retract_left_ -= st;
            break;
        }
        const bool freeing3 = free_form_ == 3 && free_pending_ > 0;
        Vec3f top(float(tg.x + (freeing3 ? free_out_dx_ : 0.0)), float(-stand_h_m_ + lift_dy_), tg.z);
        if (freeing3) { Vec3f probe = top; if (kin_.sector_clamp(leg_, probe)) top.x = probe.x; }   // outward only as far as the leg's sector allows
        const double dy = double(top.y - tg.y), dx = double(top.x - tg.x), step = lift_v_ * dt_;
        tg.y += float(std::fabs(dy) <= step ? dy : (dy > 0 ? step : -step));
        if (freeing3) tg.x += float(std::fabs(dx) <= step ? dx : (dx > 0 ? step : -step));
        const bool arrived = std::fabs(dy) <= step && (!freeing3 || std::fabs(dx) <= step);
        if (clip_ticks_ > 0) {   // the leg cannot follow: take the part of the arc it reached and move on
            const Vec3f cmd_now = kin_.toe_of(leg_, angles_[size_t(leg_)]);
            if (len(sub(cmd_now, tg)) > unreach_tol_m_) ++clip_run_; else clip_run_ = 0;
            if (clip_run_ >= clip_ticks_) { tg = cmd_now; lift_dy_ = double(tg.y) + stand_h_m_; ++clips_; clip_run_ = 0; phase_ = Phase::Swing; phase_ticks_ = 0; break; }
        }
        if (arrived && len(sub(kin_.toe_of(leg_, angles_[size_t(leg_)]), tg)) < 2.0 * kin_.p.ik_tol_m) { phase_ = Phase::Swing; phase_ticks_ = 0; }
        else if (phase_ticks_ > phase_max_ticks_ * (freeing3 ? 2 : 1)) { ++phase_timeouts_; phase_ = Phase::Swing; phase_ticks_ = 0; }
        } else {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f goal(land_.x, float(-stand_h_m_ + lift_dy_), land_.z);
        const Vec3f d = sub(goal, tg); const double dn = len(d), step = swing_v_ * dt_;
        if (dn <= step) tg = goal; else tg = add(tg, Vec3f(float(d.x / dn * step), float(d.y / dn * step), float(d.z / dn * step)));
        if (clip_ticks_ > 0) {
            const Vec3f cmd_now = kin_.toe_of(leg_, angles_[size_t(leg_)]);
            if (len(sub(cmd_now, tg)) > unreach_tol_m_) ++clip_run_; else clip_run_ = 0;
            if (clip_run_ >= clip_ticks_) { reloc_add(RelClip, land_, cmd_now); tg = cmd_now; ++clips_; clip_run_ = 0; phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; break; }
        }
        if (dn <= step && len(sub(kin_.toe_of(leg_, angles_[size_t(leg_)]), tg)) < 2.0 * kin_.p.ik_tol_m) { phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; }
        else if (phase_ticks_ > phase_max_ticks_) { ++phase_timeouts_; tg = goal; phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; }
        }
        break;
    case Phase::Lower: {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f cmd = kin_.toe_of(leg_, angles_[size_t(leg_)]);
        const bool unreachable = len(sub(cmd, tg)) > unreach_tol_m_;
        double v = lower_v_;
        if (lower_fast_v_ > 0.0 && land_sigma_ > 0.0) {            // the memory knows the ground at the landing: fast to a standoff above it, careful beneath
            const double h_tg = double(tg.y) + stand_h_m_ + in_.gravity_h(Vec3f(tg.x, float(-stand_h_m_), tg.z));
            const double standoff = lower_standoff_m_ + lower_standoff_k_ * std::max(land_sigma_, std::sqrt(land_err_ms_));   // the memory's own uncertainty, or the sequencer's running landing error where the memory is overconfident (sigma_touch 5 mm against 1-3 cm of landing error)
            if (h_tg > land_h_ + standoff) { v = lower_fast_v_; ++fast_ticks_; } else ++slow_ticks_;
        }
        const bool cant_reach_down = unreachable && double(cmd.y) > double(tg.y) + 0.005;   // the leg is ABOVE its target and cannot get down
        if (unreachable && !cant_reach_down) ++lower_fold_ticks_;    // the leg is below a target it cannot fold up to: descending helps (a 50 mm riser left a front leg at both its stops with its target above the tread, 2026-11-09)
        if (cant_reach_down && crouch_max_m_ > 0.0) { crouch_ = std::min(crouch_max_m_, crouch_ + crouch_v_ * dt_); ++crouch_ticks_; }   // the body comes down to the toe
        if (!cant_reach_down) tg.y -= float(v * dt_);
        else if (phase_ticks_ % 5 == 0) {                            // out of reach downward: the landing retreats toward the hip and the toe tries again
            const Vec3f hip = kin_.hip1(leg_); const double dx = hip.x - tg.x, dz = hip.z - tg.z, dn = std::hypot(dx, dz);
            if (dn > kin_.p.reach_min_m + retreat_m_) { const Vec3f b = tg; tg.x += float(dx / dn * retreat_m_); tg.z += float(dz / dn * retreat_m_); reloc_add(RelRetreat, b, tg); ++retreats_; ++lower_retreats_; }
        }
        if (depth_max_m_ > 0.0 && tg.y < float(-depth_max_m_)) { tg.y = float(-depth_max_m_); ++depth_caps_; }   // never into the pole
        const double h_cmd = double(cmd.y) + stand_h_m_ + in_.gravity_h(Vec3f(cmd.x, float(-stand_h_m_), cmd.z));   // the commanded toe's height in the stance's terms
        const bool raw_touch = in_.have_contact_ && in_.contact_[size_t(leg_)] > 0.5f && (!in_.have_load_ || in_.load_[size_t(leg_)] >= float(touch_load_));
        touch_run_ = raw_touch ? touch_run_ + 1 : 0;
        const bool touched = touch_run_ >= touch_ticks_ && phase_ticks_ > 3;
        const bool sunk = h_cmd < land_h_ - sink_max_m_ || phase_ticks_ > (miss_reacquire_ > 0.0 && miss_ticks_ > 0 ? miss_ticks_ : 4 * phase_max_ticks_) || (miss_reacquire_ > 0.0 && lower_retreats_ >= 6);
        if (touched || sunk) {
            if (sunk) ++lower_sunk_;
            // the toe's height where it stopped, against the toes already down: its relative elevation; and the memory's
            // prediction of the ground here against where the toe met it -- the careful stepper's error
            const double h_toe = (touched && in_.have_y_) ? double(in_.y_[size_t(leg_)]) : h_cmd;
            ground_h_[size_t(leg_)] = h_toe;
            elev_landed_[size_t(leg_)] = double(cmd.y) + stand_h_m_;   // the body-frame height the command had at the touch: the toe stands here
            if (touched) hang_count_[size_t(leg_)] = 0;
            free_pending_ = 0; free_retract_left_ = 0.0; free_out_dx_ = free_out_dy_ = 0.0;
            const bool null_step = miss_reacquire_ > 0.0 && std::hypot(double(cmd.x) - lift_from_.x, double(cmd.z) - lift_from_.z) < null_step_m_;
            if (touched) { land_err_last_ = h_toe - land_h_; land_err_ms_ += 0.1 * (land_err_last_ * land_err_last_ - land_err_ms_); ++landings_; if (!null_step) miss_count_[size_t(leg_)] = 0; else { ++null_steps_; ++miss_count_[size_t(leg_)]; } }
            // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): reseeding_ below (miss_reacquire's walk is the re-seed's).
            else { ++missed_; if (miss_reacquire_ > 0.0) { ++miss_count_[size_t(leg_)]; reseeding_[size_t(leg_)] = true; } }   // re-acquire: the leg walks to its neutral pose, its next attempt differs
            book_.last_landed_ = leg_;   // the unloaded-toe rule reads what this leg carries before its next turn
            const bool tracked = reloc_active_;
            reloc_add(RelIk, tg, cmd); reloc_finish(cmd);   // the tag closes at the touch (an instrument)
            record_outcome(tracked, touched, h_toe, cmd);   // the step's outcome (an instrument)
            tg = cmd;   // the stance takes it from here
            ++steps_[size_t(leg_)]; last_step_tick_[size_t(leg_)] = int64_t(tick_now_);
            cycles_ = *std::min_element(steps_.begin(), steps_.end());   // full crawl cycles: every leg has stepped this many times
            if (relief_for_ >= 0 && relief_for_ != leg_) { leg_ = relief_for_; relief_for_ = -1; plan_active_ = false; } else { relief_for_ = -1; decide_next(tick_now_); }
            shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ = Phase::Shift; phase_ticks_ = 0;
        }
        break; }
    }
}

// the tick's sensing: the tick, and the leaky bearing estimate of each toe with hysteresis (moved from PoseSequencer::tick)
void StepExecutor::sense(uint64_t tick_id) {
    tick_now_ = tick_id;
    if (in_.have_contact_) {   // the leaky bearing estimate, with hysteresis
        const double a = 1.0 / bear_tau_ticks_;
        for (int i = 0; i < 4; ++i) {
            bear_[size_t(i)] += a * ((in_.contact_[size_t(i)] > 0.5f ? 1.0 : 0.0) - bear_[size_t(i)]);
            if (bear_[size_t(i)] > bear_on_) bearing_[size_t(i)] = true; else if (bear_[size_t(i)] < bear_off_) bearing_[size_t(i)] = false;
        }
    }
}

// the commanded toes to joint angles: one IK solve per leg per tick (moved from PoseSequencer::tick)
void StepExecutor::solve_ik() {
    double err = 0.0;
    for (int i = 0; i < 4; ++i) {
        // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): the re-seed walk.
        if (ik_reseed_ticks_ > 0 && reseeding_[size_t(i)]) {   // walking back to the neutral standing pose at the step cap; the IK takes over when it arrives or can reach
            const auto nt = kin_.angles_of_u(i, {0.0, 0.0, 0.3}); bool there = true;
            for (int j = 0; j < 3; ++j) { const double d = nt[size_t(j)] - angles_[size_t(i)][size_t(j)]; const double st = std::clamp(d, -ik_reseed_rate_, ik_reseed_rate_); angles_[size_t(i)][size_t(j)] += st; if (std::fabs(d) > ik_reseed_rate_) there = false; }
            if (there || len(sub(kin_.toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)])) < 2.0 * unreach_tol_m_) { reseeding_[size_t(i)] = false; pinned_ticks_[size_t(i)] = 0; }
            err += len(sub(kin_.toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)])) / 4.0;
            continue;
        }
        kin_.ik_solve(i, toe_tgt_[size_t(i)], angles_[size_t(i)], ik_multistart_ > 0.0);
        const double e_i = len(sub(kin_.toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)]));
        err += e_i / 4.0;
        // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): the re-seed trigger.
        if (ik_reseed_ticks_ > 0) {   // pinned: a joint at its stop with the toe far from its target
            const auto u = kin_.u_of_angles(i, angles_[size_t(i)]); bool at_stop = false;
            for (int j = 0; j < 3; ++j) if (std::fabs(u[size_t(j)]) >= 0.999) at_stop = true;
            if (at_stop && e_i > 2.0 * unreach_tol_m_) { if (++pinned_ticks_[size_t(i)] >= ik_reseed_ticks_) { reseeding_[size_t(i)] = true; ++reseeds_; } }
            else pinned_ticks_[size_t(i)] = 0;
        }
    }
    ik_err_ema_ += 0.05 * (err - ik_err_ema_);
}

// a decision point: the plan's state cleared and an abandoned step counted (the relocation tag), then the next intent applied
void StepExecutor::decide_next(uint64_t) {
    plan_active_ = false; plan_pending_ = false; plan_done_x_ = plan_done_z_ = 0.0; unload_shift_only_ = false; reloc_plan_leg_ = -1;
    if (reloc_active_) { ++reloc_n_[6]; reloc_active_ = false; }   // the tag: a step that went and never landed (a hang's re-shift passed the turn) is abandoned
    apply(intent_hook_());
}

void StepExecutor::apply(const StepIntent& in) {
    leg_ = in.leg;
    if (in.kind == StepIntent::Unload) { plan_active_ = true; plan_pending_ = false; plan_sx_ = in.sx; plan_sz_ = in.sz; unload_shift_only_ = true; }
    else if (in.kind == StepIntent::Plan) { reloc_plan_leg_ = in.leg; plan_land_ = in.land; plan_sx_ = in.sx; plan_sz_ = in.sz; plan_score_ = in.score; plan_active_ = true; plan_pending_ = true; }
}

// the plan drift: the last decision's predicted margin for the next leg against the margin found now, and which factor broke it
void StepExecutor::reloc_finish(const Vec3f& landed) {
    if (!reloc_active_) return;
    reloc_active_ = false;
    const double ox = double(landed.x) - reloc_from_.x, oz = double(landed.z) - reloc_from_.z, off = std::hypot(ox, oz);
    double vx = 0.0, vz = 0.0; for (auto const& v : reloc_vec_) { vx += v[0]; vz += v[1]; }
    reloc_resid_ = std::max(reloc_resid_, std::hypot(ox - vx, oz - vz));   // the closure: every move of the landing was tagged
    ++reloc_n_[0];
    if (!reloc_planned_) { ++reloc_n_[reloc_switched_ ? 3 : 2]; return; }
    ++reloc_n_[1]; reloc_off_ += off; if (reloc_hang_) ++reloc_n_[5];
    int top = -1; double tb = 0.0;
    for (int r = 0; r < RelN; ++r) { const double m = std::hypot(reloc_vec_[size_t(r)][0], reloc_vec_[size_t(r)][1]); reloc_sum_[size_t(r)] += m; if (m > tb) { tb = m; top = r; } }
    if (off > 0.02) { ++reloc_n_[4]; if (top >= 0) ++reloc_top_[size_t(top)]; }
}

// the step's outcome, from the relocation tag's record of it (reloc_finish leaves its fields in place)
void StepExecutor::record_outcome(bool tracked, bool touched, double h_touch, const Vec3f& landed) {
    StepOutcome& o = outcome_;
    o.seq = ++outcome_seq_; o.leg = leg_; o.kind = !tracked ? -1 : (reloc_planned_ ? 1 : (reloc_switched_ ? 2 : 0));
    o.touched = touched; o.hang = tracked && reloc_hang_; o.ticks = tracked ? int(tick_now_ - go_tick_) : 0;
    o.from = tracked ? reloc_from_ : landed; o.land = landed; o.h_pred = land_h_; o.h_touch = h_touch;
    o.moves = tracked ? reloc_vec_ : std::array<std::array<double, 2>, RelN>{};
}

} // namespace ogma::stepper
