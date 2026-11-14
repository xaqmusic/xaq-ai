#include "ogma/modules/PoseSequencer.hpp"
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
    throw std::invalid_argument("PoseSequencer: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PoseSequencer: param '" + k + "' must be a string");
}
using body::Vec3f;
Vec3f sub(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x - b.x, a.y - b.y, a.z - b.z); }
Vec3f add(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x + b.x, a.y + b.y, a.z + b.z); }
double len(const Vec3f& a) { return std::sqrt(double(a.x) * a.x + double(a.y) * a.y + double(a.z) * a.z); }
const char* phase_name(PoseSequencer::Phase p) {
    switch (p) { case PoseSequencer::Phase::Idle: return "idle"; case PoseSequencer::Phase::Shift: return "shift"; case PoseSequencer::Phase::Lift: return "lift";
                 case PoseSequencer::Phase::Swing: return "swing"; case PoseSequencer::Phase::Lower: return "lower"; case PoseSequencer::Phase::Settle: return "settle"; }
    return "?";
}
} // namespace

PoseSequencer::PoseSequencer()  = default;
PoseSequencer::~PoseSequencer() = default;
std::string_view PoseSequencer::type_name() const { return "PoseSequencer"; }

std::vector<TopicSpec> PoseSequencer::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : {model_topic_, support_topic_, patch_topic_, feet_xz_topic_, feet_y_topic_, up_topic_, imu_topic_, contact_topic_, load_topic_, odom_in_topic_})
        if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    for (auto const& t : gait_action_topics_) if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ActionOut)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> PoseSequencer::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
    if (!odom_out_topic_.empty()) v.push_back(TopicSpec{odom_out_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema PoseSequencer::params_schema() const {
    return {
        {"model_topic",   ParamMutability::ConstructionOnly, "ProprioToken [116]: the body's leg model (reality.proprio.leg_model) -- the shared FK's anchors and the action-to-angle map.", ParamValue{std::string("reality.proprio.leg_model")}},
        {"support_topic", ParamMutability::ConstructionOnly, "ProprioToken: SupportModel's support.state (the per-leg lift margins decide when a leg may leave).", ParamValue{std::string("support.state")}},
        {"patch_topic",   ParamMutability::ConstructionOnly, "ProprioToken: TerrainMemory's patch (the ground under a toe and at its landing point).", ParamValue{std::string("terrain.patch")}},
        {"feet_xz_topic", ParamMutability::ConstructionOnly, "ProprioToken [x, z] x 4: the body's own toes (the FK check).", ParamValue{std::string("reality.proprio.feet_xz")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4]: the toes' gravity heights (the FK check).", ParamValue{std::string("reality.proprio.toe_y")}},
        {"up_topic",      ParamMutability::ConstructionOnly, "ProprioToken [3]: the body's gravity-up in the body frame.", ParamValue{std::string("reality.proprio.up_body")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4]: the contact flags -- the sensor of a toe having landed.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"load_topic",    ParamMutability::ConstructionOnly, "ProprioToken [4]: the foot loads.", ParamValue{std::string("reality.proprio.foot_load")}},
        {"gait_action_topics", ParamMutability::ConstructionOnly, "The gait's 12 action topics (leg-major hip1, hip2, knee): the pose the sequencer starts from when it engages.", ParamValue{std::vector<std::string>{}}},
        {"output_topic",  ParamMutability::ConstructionOnly, "ProprioToken [engage, u x 12] for MotorEPMv2's pose_topic.", ParamValue{std::string("pose.targets")}},
        {"belly_topic",   ParamMutability::ConstructionOnly, "ProprioToken [m]: the belly clearance; the sequencer engages only with the body standing (>= engage_min_belly_m). \"\" = no check.", ParamValue{std::string("reality.proprio.belly_m")}},
        {"engage_min_belly_m", ParamMutability::HotMutable, "The belly clearance a standing body shows (m).", ParamValue{0.045}},
        {"engage_min_tick", ParamMutability::HotMutable, "No engage before this tick (the gait's warm-up).", ParamValue{int64_t{200}}},
        {"engage_min_contacts", ParamMutability::HotMutable, "Toes in contact needed at the moment of engaging.", ParamValue{int64_t{3}}},
        {"phase_max_ticks", ParamMutability::HotMutable, "A LIFT or SWING not arrived by then proceeds anyway (counted); 60 = 1.2 s.", ParamValue{int64_t{60}}},
        {"level_max_m",   ParamMutability::HotMutable, "The most the levelling moves any toe (m).", ParamValue{0.02}},
        {"level_gain",    ParamMutability::HotMutable, "The stance's levelling: the fraction of the geometric correction (the toe's height offset implied by the body's tilt) applied to each planted toe, low side deeper; 1 = the whole correction at once.", ParamValue{0.5}},
        {"stand_h_m",     ParamMutability::HotMutable, "The body's height above the planted toes' mean ground (m): planted toes are held there in the body frame.", ParamValue{0.085}},
        {"imu_topic",     ParamMutability::ConstructionOnly, "ProprioToken [ax, ay, az, gx, gy, gz] in the body frame (the body's imu6), for the backward surge of a hung toe. \"\" = none.", ParamValue{std::string("reality.proprio.imu6")}},
        {"hang_ticks",    ParamMutability::HotMutable, "A toe asked 2 cm up whose contact flag stays on this many ticks is caught (a riser's face): the body shifts back and lifts it again, higher and toward the hip; 0 = off.", ParamValue{int64_t{0}}},
        {"drag_back_m",   ParamMutability::HotMutable, "... or whose lift and swing move the body backward by this much, by the IMU's forward acceleration integrated (m); 0 = off (measured 2026-11-09: the integral's noise is 4 cm against a true 1 cm, correlation 0.03).", ParamValue{0.0}},
        {"free_back_m",   ParamMutability::HotMutable, "(unused since the second form; the body's shift back dragged it backward and it fell)", ParamValue{0.02}},
        {"free_extra_m",  ParamMutability::HotMutable, "... then lifts the toe this much higher ...", ParamValue{0.02}},
        {"free_retract_m", ParamMutability::HotMutable, "... after dragging the toe this far toward the hip along the ground, away from the face (twice on a second hang of the same leg).", ParamValue{0.02}},
        {"land_r_min_m",  ParamMutability::HotMutable, "A planned landing is kept at least this far from its hip (m); 0 = reach_min_m. A stance collapsed inward leaves no feasible body shift.", ParamValue{0.0}},
        {"overdue",       ParamMutability::HotMutable, "1 = while the wave's leg (or an urgent leg) has no feasible step, substitute steps may not reduce its slack to full extension and their landings are ranked first by leaving it liftable; 0 = off.", ParamValue{0.0}},
        {"ext_urgent_m",  ParamMutability::HotMutable, "The planted leg with the least slack to full extension below this (m) is tried before the wave's leg, without consuming the wave's turn; 0 = off.", ParamValue{0.0}},
        {"ext_lifting",   ParamMutability::HotMutable, "The leg about to lift keeps this fraction of ext_margin_m as slack during the shift that buys its lift margin (its reach and sector stay exempt); 0 = off, 1 = the full margin (a rear toe at the margin could then never lift).", ParamValue{0.0}},
        {"ext_margin_m",  ParamMutability::HotMutable, "A body shift may not leave a planted toe less than this short of its leg's full extension (m, hip2 to toe at the toe's depth); 0 = off. The horizontal reach_max_m is a straight leg at the stance's depth.", ParamValue{0.0}},
        {"unloaded_shift_m", ParamMutability::HotMutable, "A planted toe that touches but carries less than unloaded_share of the weight for unloaded_ticks: before any leg lifts the body shifts this far toward it (m); 0 = off.", ParamValue{0.0}},
        {"unloaded_share", ParamMutability::HotMutable, "... the share below which a touching toe counts as unloaded ...", ParamValue{0.08}},
        {"unloaded_ticks", ParamMutability::HotMutable, "... for this many ticks in a row.", ParamValue{int64_t{50}}},
        {"unloaded_skips_min", ParamMutability::HotMutable, "The time rule fires only when the last search skipped at least this many legs as infeasible (the unloaded toe is blocking the others); 0 = always.", ParamValue{int64_t{2}}},
        {"unloaded_landings_min", ParamMutability::HotMutable, "The leg that just landed triggers the shift only after this many landings in a row that never bore weight (one is common and benign).", ParamValue{int64_t{2}}},
        {"clip_ticks",    ParamMutability::HotMutable, "A LIFT or SWING whose commanded toe has been more than unreach_tol_m from its target for this many ticks takes the part of the arc it reached and moves on; 0 = wait for the phase timeout.", ParamValue{int64_t{0}}},
        {"miss_reacquire", ParamMutability::HotMutable, "1 = a missed landing walks the leg to its neutral pose and makes its next attempt different (the landing perturbed, the lift lower, the leg penalised in the search by 1 cm of margin per miss); 0 = off.", ParamValue{0.0}},
        {"null_step_m",   ParamMutability::HotMutable, "With miss_reacquire: a landing within this of the toe's lift-off point is a null step and counts as a miss.", ParamValue{0.01}},
        {"miss_ticks",    ParamMutability::HotMutable, "With miss_reacquire: a lowering without ground gives up after this many ticks (0 = 4 x phase_max_ticks) or six retreats.", ParamValue{int64_t{0}}},
        {"front_drop_gain", ParamMutability::HotMutable, "The front toes are held this much shallower per radian of nose-up pitch beyond front_drop_deadband (m/rad): the front of the chassis comes down and the centre of mass forward; 0 = off.", ParamValue{0.0}},
        {"front_drop_max_m", ParamMutability::HotMutable, "... by at most this (m).", ParamValue{0.04}},
        {"front_drop_tau_ticks", ParamMutability::HotMutable, "The front drop follows its pitch-derived target through a first-order filter of this many ticks (1 = none); the loop through the body's pitch bobs the planted front legs when it is fast.", ParamValue{1.0}},
        {"front_drop_deadband", ParamMutability::HotMutable, "Nose-up pitch below this (rad) is left to the levelling.", ParamValue{0.05}},
        {"depth_max_m",   ParamMutability::HotMutable, "No toe is asked more than this far below the body (m): the leg's pole, where the knee hyperextends and locks, lies beyond it; 0 = off.", ParamValue{0.0}},
        {"free_form",     ParamMutability::HotMutable, "The freeing move for a hung toe: 2 = retract along the ground then lift higher (shipped 2026-11-09); 3 = unload it by the margin shift (free_margin_m more than a step needs), lift to full height first, free_extra_m higher, the top free_out_m outward, a second hang adding free_noise_m of random perturbation.", ParamValue{int64_t{2}}},
        {"free_margin_m", ParamMutability::HotMutable, "Form 3: the margin asked of the shift before a freeing lift, over margin_min_m + margin_extra_m (m).", ParamValue{0.015}},
        {"free_out_m",    ParamMutability::HotMutable, "Form 3: the lift's top this far outward from the body, per attempt (m).", ParamValue{0.02}},
        {"free_noise_m",  ParamMutability::HotMutable, "Form 3: the random perturbation of the arc's height and side on the second attempt (m, uniform +/-).", ParamValue{0.01}},
        {"face_gap_m",    ParamMutability::HotMutable, "Lever C: a planned landing with a rise of land_edge_m or more within this far ahead of it (a face) is penalised; 0 = off.", ParamValue{0.0}},
        {"face_penalty_m", ParamMutability::HotMutable, "... by this much of the landing's score (m of margin).", ParamValue{0.02}},
        {"ik_multistart", ParamMutability::HotMutable, "1 = a leg the single-seed IK leaves more than 2 x ik_tol_m from its target is re-solved from seeds covering the leg's branches (neutral, femur down / up, knee folded / extended) and the best within the joint ranges is taken; 0 = off (byte-identical).", ParamValue{0.0}},
        {"ik_switch_m",   ParamMutability::HotMutable, "A branch from another seed is taken only when it beats the current solution's residual by this (m).", ParamValue{0.005}},
        {"ik_reseed_ticks", ParamMutability::HotMutable, "A leg whose IK sits at a joint stop with its toe more than 2 x unreach_tol_m from its target for this many ticks is walked back to the neutral standing pose and re-solved (the wrong branch: a femur folded up for a target below); 0 = off.", ParamValue{int64_t{0}}},
        {"ik_reseed_rate_rad", ParamMutability::HotMutable, "The re-seed walks the joints toward the neutral pose at this much per tick (rad); the IK's step cap flipped the body when a planted leg snapped.", ParamValue{0.04}},
        {"crouch_max_m",  ParamMutability::HotMutable, "A lowering toe at its downward reach without ground brings the body down: the planted legs fold by up to this (m); 0 = off.", ParamValue{0.0}},
        {"crouch_v_mps",  ParamMutability::HotMutable, "... at this rate while the deficit lasts ...", ParamValue{0.03}},
        {"crouch_relax_v_mps", ParamMutability::HotMutable, "... and rise back at this rate while every toe bears.", ParamValue{0.005}},
        {"elev_source",   ParamMutability::HotMutable, "\"patch\" = each planted toe's elevation from the terrain memory under its place (EMA, deadband); \"landed\" = the body-frame height its command had when it met the ground (the toe stands where it landed; the body parallel to the toes' plane). Both bounded by elev_max_m (0 = off).", ParamValue{std::string("patch")}},
        {"elev_max_m",    ParamMutability::HotMutable, "The most a planted toe may be held above or below the others' mean from the terrain patch's elevation (a riser); 0 = the stance ignores the patch (the default until the memory's frame is the sequencer's own: the patch held toes off the floor, 2026-11-08).", ParamValue{0.0}},
        {"reach_max_m",   ParamMutability::HotMutable, "A toe target is kept within this horizontal distance of the leg's hip (the full leg reaches ~0.143 at the standing height).", ParamValue{0.14}},
        {"bear_tau_ticks", ParamMutability::HotMutable, "The leaky estimate of each toe's bearing from the contact flag (ticks); the lift-off transient on the other legs does not reach the sequence.", ParamValue{8.0}},
        {"bear_on",       ParamMutability::HotMutable, "The estimate above which a toe bears ...", ParamValue{0.6}},
        {"bear_off",      ParamMutability::HotMutable, "... and below which it no longer does (hysteresis).", ParamValue{0.3}},
        {"touch_ticks",   ParamMutability::HotMutable, "A landing needs the raw flag on this many ticks in a row.", ParamValue{int64_t{3}}},
        {"touch_load",    ParamMutability::HotMutable, "... and, with a load topic, at least this fraction of the body's weight (0 = the flag alone; 0.05 missed landings whose toe had touched but not yet taken weight).", ParamValue{0.0}},
        {"creep_v_mps",   ParamMutability::HotMutable, "A planted toe without contact creeps down at this rate until it bears.", ParamValue{0.02}},
        {"creep_load_min", ParamMutability::HotMutable, "With a load topic: a planted toe creeps down while its FSR share of the weight is below this (a toe touching but unloaded on an over-determined stance); 0 = the contact flag decides (byte-identical).", ParamValue{0.0}},
        {"creep_load_relax", ParamMutability::HotMutable, "... and relaxes back only while its share is above this.", ParamValue{0.2}},
        {"creep_max_m",   ParamMutability::HotMutable, "... by at most this (m).", ParamValue{0.02}},
        {"creep_relax_v_mps", ParamMutability::HotMutable, "... and relaxes back at this rate while it bears.", ParamValue{0.005}},
        {"elev_deadband_m", ParamMutability::HotMutable, "A toe's ground elevation against the others (from the terrain patch) smaller than this is noise (kept 0); the memory's heights scatter by 2-3 cm on flat ground.", ParamValue{0.03}},
        {"elev_tau_ticks", ParamMutability::HotMutable, "EMA of each toe's ground elevation from the patch.", ParamValue{100.0}},
        {"air_ticks_min", ParamMutability::HotMutable, "A planted toe counts as airborne (placed first) after this many ticks without contact -- long enough for the creep to have tried.", ParamValue{int64_t{100}}},
        {"plan",          ParamMutability::HotMutable, "1 = the POSE SEARCH: the leg, its landing and the body's shift chosen together one step ahead (the shift within every planted toe's reach and giving the leg its margin; the landing leaving the next leg of the wave liftable by a feasible shift); 0 = the shift controller and the fixed order (byte-identical).", ParamValue{0.0}},
        {"plan_shift_m",  ParamMutability::HotMutable, "The search's body shifts: lateral and backward out to this (m) ...", ParamValue{0.03}},
        {"plan_shift_fwd_m", ParamMutability::HotMutable, "... forward out to this (m) ...", ParamValue{0.04}},
        {"plan_land_m",   ParamMutability::HotMutable, "... and landings out to this from the leg's neutral place (m), all on plan_cell_m.", ParamValue{0.04}},
        {"plan_cell_m",   ParamMutability::HotMutable, "The search grid (m).", ParamValue{0.01}},
        {"plan_adv_w",    ParamMutability::HotMutable, "The landing's forward offset is worth this much of the next leg's margin (per metre, both in metres): progress against the margin kept.", ParamValue{0.3}},
        {"relief",        ParamMutability::HotMutable, "1 = the RELIEF step: when the leg whose turn it is blocks the shift at its own reach limit and is unsafe to lift, another safe leg steps along its heading to where the stuck leg gains its margin without a shift, then the stuck leg's turn is retried; 0 = the turn passes (byte-identical).", ParamValue{0.0}},
        {"relief_step_m", ParamMutability::HotMutable, "The relief planner's candidate landings are this far apart along the leg's heading ...", ParamValue{0.01}},
        {"relief_cells",  ParamMutability::HotMutable, "... out to this many steps either side of the neutral place.", ParamValue{int64_t{4}}},
        {"advance_reach_m", ParamMutability::HotMutable, "The body does not advance while a planted toe is within this of reach_max_m (the shift still moves for the margin); 0 = off.", ParamValue{0.0}},
        {"advance_m",     ParamMutability::HotMutable, "Each SHIFT carries the body forward by this (m) before the leg may lift (a quarter stride).", ParamValue{0.015}},
        {"centre_gain",   ParamMutability::HotMutable, "During SHIFT the body also moves this fraction of its offset from the four toes' centroid per tick's bound.", ParamValue{0.5}},
        {"margin_extra_m", ParamMutability::HotMutable, "The shift aims for margin_min_m plus this (m).", ParamValue{0.005}},
        {"order",         ParamMutability::HotMutable, "The crawl's leg order (leg indices 0 FL, 1 FR, 2 RL, 3 RR); the default is the wave gait RL, FL, RR, FR. Empty = the greedy choice (the toe furthest behind its place).", ParamValue{std::vector<double>{2, 0, 3, 1}}},
        {"odom_in_topic", ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the body's dead-reckoned odometry; its yaw is the heading the hold keeps, and it is passed through while disengaged.", ParamValue{std::string("reality.proprio.odom")}},
        {"odom_out_topic", ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the sequencer's own odometry (the body's commanded motion over the planted toes, the body's yaw) for the terrain memory's frame. \"\" = not published.", ParamValue{std::string("reality.proprio.odom_seq")}},
        {"heading_hold",  ParamMutability::HotMutable, "1 = hold the heading the body had at engage (the planted toes rotated during SHIFT); 0 = off.", ParamValue{1.0}},
        {"heading_ref",   ParamMutability::HotMutable, "\"zero\" = hold the dead-reckoning frame's own heading (the body's facing at its anchor, the corridor's axis); \"engage\" = the heading the body had at engage.", ParamValue{std::string("zero")}},
        {"shift_realised", ParamMutability::HotMutable, "The fraction of the commanded shift the body realises, applied to the odometry (measured 0.82: the planted toes slip).", ParamValue{0.82}},
        {"heading_gain",  ParamMutability::HotMutable, "Yaw rate commanded per radian of heading error (1/s).", ParamValue{0.6}},
        {"yaw_rate_max",  ParamMutability::HotMutable, "The most the body is turned per second (rad/s).", ParamValue{0.3}},
        {"settle_v_mps",  ParamMutability::HotMutable, "On engage the planted toes slide to their neutral places at this rate before the first step.", ParamValue{0.05}},
        {"neutral_r_m",   ParamMutability::HotMutable, "The stance: each toe's neutral place is this far from its hip along the leg's heading (m); the legs step toward it from wherever the gait left them.", ParamValue{0.10}},
        {"land_band_lo_m", ParamMutability::HotMutable, "The memory's height at a landing point is believed down to this far below the stance (a drifted frame put landings 4 cm deep and out of reach, 2026-11-08) ...", ParamValue{0.02}},
        {"land_band_hi_m", ParamMutability::HotMutable, "... and up to this far above it (a riser); outside the band the stance height is used.", ParamValue{0.09}},
        {"sector_half_rad", ParamMutability::HotMutable, "A planted toe stays within this angle of the leg's neutral heading about its hip (rad); 0.6 blocked most shifts, 0.9 lets the body sway.", ParamValue{0.9}},
        {"land_sector_rad", ParamMutability::HotMutable, "A landing point stays within this angle of the heading (a swing to the standing sector's edge times out).", ParamValue{0.5}},
        {"reach_min_m",   ParamMutability::HotMutable, "... and no closer to the hip than this (m).", ParamValue{0.06}},
        {"retreat_m",     ParamMutability::HotMutable, "A lowering toe out of reach moves this far toward its hip and tries again.", ParamValue{0.01}},
        {"unreach_tol_m", ParamMutability::HotMutable, "A toe the IK leaves this far from its target is out of reach: a lowering toe is taken as landed where it is.", ParamValue{0.015}},
        {"engage",        ParamMutability::HotMutable, "1 = the sequencer owns the legs (ramped in over engage_ramp_ticks); 0 = the rhythm (the default; byte-identical).", ParamValue{0.0}},
        {"engage_ramp_ticks", ParamMutability::HotMutable, "Ticks over which the blend ramps in or out.", ParamValue{int64_t{50}}},
        {"shift_v_mps",   ParamMutability::HotMutable, "The body's advance over the planted toes during SHIFT (m/s).", ParamValue{0.03}},
        {"shift_max_m",   ParamMutability::HotMutable, "The most one SHIFT may advance before a leg is lifted anyway (the model's safest).", ParamValue{0.05}},
        {"margin_min_m",  ParamMutability::HotMutable, "The lift margin (m) the chosen leg needs before it may lift.", ParamValue{0.005}},
        {"lift_clear_m",  ParamMutability::HotMutable, "Clearance above the higher of the ground under the toe and at its landing point.", ParamValue{0.03}},
        {"lift_v_mps",    ParamMutability::HotMutable, "The toe's rise rate.", ParamValue{0.10}},
        {"swing_v_mps",   ParamMutability::HotMutable, "The toe's forward rate in the air.", ParamValue{0.12}},
        {"lower_v_mps",   ParamMutability::HotMutable, "The toe's descent rate (within the standoff above the predicted ground, or everywhere when lower_fast_v_mps is 0).", ParamValue{0.06}},
        {"lower_fast_v_mps", ParamMutability::HotMutable, "The descent rate down to the standoff above the memory's predicted ground at the landing; 0 = off (lower_v_mps throughout, byte-identical).", ParamValue{0.0}},
        {"lower_standoff_m", ParamMutability::HotMutable, "The fast descent stops this far above the predicted ground ...", ParamValue{0.01}},
        {"lower_standoff_k", ParamMutability::HotMutable, "... plus this many times the larger of the memory's uncertainty there and the running RMS landing error (m): confident ground is approached quickly, uncertain ground carefully.", ParamValue{2.0}},
        {"land_edge_m",   ParamMutability::HotMutable, "A landing point whose remembered neighbourhood (3 x 3 cells) spans this much height is a riser's edge and is moved along the heading to flat ground; 0 = off.", ParamValue{0.0}},
        {"land_edge_max_m", ParamMutability::HotMutable, "The most a landing is moved off an edge (m), within reach.", ParamValue{0.04}},
        {"stride_m",      ParamMutability::HotMutable, "A toe lands this far ahead of the place it stood at engage... per step: the landing is neutral + stride/2.", ParamValue{0.06}},
        {"sink_max_m",    ParamMutability::HotMutable, "A lowering toe this far below the predicted ground without contact is taken as landed; the FSR ends a lowering, the prediction only bounds it (0.03 missed landings on the stairs' approach, where the memory's ground sat high).", ParamValue{0.06}},
        {"tilt_disengage", ParamMutability::HotMutable, "Below this vertical component of the body's up the legs go back to the rhythm.", ParamValue{0.75}},
        {"dt_s",          ParamMutability::HotMutable, "The tick (s).", ParamValue{0.02}},
        {"ik_iters",      ParamMutability::HotMutable, "Damped least-squares iterations per tick per leg.", ParamValue{int64_t{4}}},
        {"ik_damping_m",  ParamMutability::HotMutable, "The IK's damping (m).", ParamValue{0.01}},
        {"ik_step_max_rad", ParamMutability::HotMutable, "The most a joint moves per tick (rad).", ParamValue{0.12}},
        {"ik_tol_m",      ParamMutability::HotMutable, "A toe within this of its target has arrived.", ParamValue{0.004}},
    };
}

ParamMap PoseSequencer::current_params() const {
    return {{"model_topic", ParamValue{model_topic_}}, {"support_topic", ParamValue{support_topic_}}, {"patch_topic", ParamValue{patch_topic_}},
            {"feet_xz_topic", ParamValue{feet_xz_topic_}}, {"feet_y_topic", ParamValue{feet_y_topic_}}, {"up_topic", ParamValue{up_topic_}},
            {"contact_topic", ParamValue{contact_topic_}}, {"load_topic", ParamValue{load_topic_}}, {"gait_action_topics", ParamValue{gait_action_topics_}}, {"output_topic", ParamValue{output_topic_}},
            {"engage", ParamValue{engage_}}, {"engage_ramp_ticks", ParamValue{int64_t(engage_ramp_ticks_)}}, {"belly_topic", ParamValue{belly_topic_}},
            {"stand_h_m", ParamValue{stand_h_m_}}, {"elev_source", ParamValue{elev_source_}}, {"crouch_max_m", ParamValue{crouch_max_m_}}, {"ik_reseed_ticks", ParamValue{int64_t(ik_reseed_ticks_)}}, {"ik_multistart", ParamValue{ik_multistart_}}, {"free_form", ParamValue{int64_t(free_form_)}}, {"front_drop_gain", ParamValue{front_drop_gain_}}, {"clip_ticks", ParamValue{int64_t(clip_ticks_)}}, {"unloaded_shift_m", ParamValue{unloaded_shift_m_}}, {"land_r_min_m", ParamValue{land_r_min_m_}}, {"ext_margin_m", ParamValue{ext_margin_m_}}, {"ext_lifting", ParamValue{ext_lifting_}}, {"overdue", ParamValue{overdue_}}, {"ext_urgent_m", ParamValue{ext_urgent_m_}}, {"unloaded_share", ParamValue{unloaded_share_}}, {"unloaded_ticks", ParamValue{int64_t(unloaded_ticks_)}}, {"unloaded_landings_min", ParamValue{int64_t(unloaded_landings_min_)}}, {"unloaded_skips_min", ParamValue{int64_t(unloaded_skips_min_)}}, {"miss_reacquire", ParamValue{miss_reacquire_}}, {"miss_ticks", ParamValue{int64_t(miss_ticks_)}}, {"null_step_m", ParamValue{null_step_m_}}, {"front_drop_max_m", ParamValue{front_drop_max_m_}}, {"front_drop_deadband", ParamValue{front_drop_deadband_}}, {"front_drop_tau_ticks", ParamValue{front_drop_tau_ticks_}}, {"depth_max_m", ParamValue{depth_max_m_}}, {"free_margin_m", ParamValue{free_margin_m_}}, {"free_out_m", ParamValue{free_out_m_}}, {"free_noise_m", ParamValue{free_noise_m_}}, {"face_gap_m", ParamValue{face_gap_m_}}, {"face_penalty_m", ParamValue{face_penalty_m_}}, {"ik_switch_m", ParamValue{ik_switch_m_}}, {"imu_topic", ParamValue{imu_topic_}}, {"hang_ticks", ParamValue{int64_t(hang_ticks_)}}, {"drag_back_m", ParamValue{drag_back_m_}}, {"free_back_m", ParamValue{free_back_m_}}, {"free_extra_m", ParamValue{free_extra_m_}}, {"free_retract_m", ParamValue{free_retract_m_}}, {"ik_reseed_rate_rad", ParamValue{ik_reseed_rate_}}, {"crouch_v_mps", ParamValue{crouch_v_}}, {"crouch_relax_v_mps", ParamValue{crouch_relax_v_}}, {"level_gain", ParamValue{level_gain_}}, {"level_max_m", ParamValue{level_max_m_}}, {"sector_half_rad", ParamValue{sector_half_rad_}}, {"land_sector_rad", ParamValue{land_sector_rad_}}, {"land_band_lo_m", ParamValue{land_band_lo_m_}}, {"land_band_hi_m", ParamValue{land_band_hi_m_}}, {"neutral_r_m", ParamValue{neutral_r_m_}}, {"settle_v_mps", ParamValue{settle_v_}}, {"odom_in_topic", ParamValue{odom_in_topic_}}, {"odom_out_topic", ParamValue{odom_out_topic_}}, {"heading_hold", ParamValue{heading_hold_}}, {"heading_ref", ParamValue{heading_ref_}}, {"heading_gain", ParamValue{heading_gain_}}, {"shift_realised", ParamValue{shift_realised_}}, {"yaw_rate_max", ParamValue{yaw_rate_max_}}, {"order", ParamValue{order_}}, {"centre_gain", ParamValue{centre_gain_}}, {"advance_m", ParamValue{advance_m_}}, {"margin_extra_m", ParamValue{margin_extra_m_}}, {"creep_v_mps", ParamValue{creep_v_}}, {"creep_load_min", ParamValue{creep_load_min_}}, {"advance_reach_m", ParamValue{advance_reach_m_}}, {"relief", ParamValue{relief_}}, {"plan", ParamValue{plan_}}, {"plan_shift_m", ParamValue{plan_shift_m_}}, {"plan_shift_fwd_m", ParamValue{plan_shift_fwd_m_}}, {"plan_land_m", ParamValue{plan_land_m_}}, {"plan_cell_m", ParamValue{plan_cell_m_}}, {"plan_adv_w", ParamValue{plan_adv_w_}}, {"relief_step_m", ParamValue{relief_step_m_}}, {"relief_cells", ParamValue{int64_t(relief_cells_)}}, {"creep_load_relax", ParamValue{creep_load_relax_}}, {"bear_tau_ticks", ParamValue{bear_tau_ticks_}}, {"bear_on", ParamValue{bear_on_}}, {"bear_off", ParamValue{bear_off_}}, {"touch_ticks", ParamValue{int64_t(touch_ticks_)}}, {"touch_load", ParamValue{touch_load_}}, {"creep_max_m", ParamValue{creep_max_m_}}, {"creep_relax_v_mps", ParamValue{creep_relax_v_}}, {"elev_deadband_m", ParamValue{elev_deadband_m_}}, {"elev_tau_ticks", ParamValue{elev_tau_ticks_}}, {"air_ticks_min", ParamValue{int64_t(air_ticks_min_)}}, {"reach_min_m", ParamValue{reach_min_m_}}, {"retreat_m", ParamValue{retreat_m_}}, {"elev_max_m", ParamValue{elev_max_m_}}, {"reach_max_m", ParamValue{reach_max_m_}}, {"unreach_tol_m", ParamValue{unreach_tol_m_}},
            {"engage_min_belly_m", ParamValue{engage_min_belly_m_}}, {"engage_min_tick", ParamValue{int64_t(engage_min_tick_)}}, {"engage_min_contacts", ParamValue{int64_t(engage_min_contacts_)}}, {"phase_max_ticks", ParamValue{int64_t(phase_max_ticks_)}}, {"shift_v_mps", ParamValue{shift_v_}}, {"shift_max_m", ParamValue{shift_max_m_}},
            {"margin_min_m", ParamValue{margin_min_m_}}, {"lift_clear_m", ParamValue{lift_clear_m_}}, {"lift_v_mps", ParamValue{lift_v_}}, {"swing_v_mps", ParamValue{swing_v_}},
            {"lower_v_mps", ParamValue{lower_v_}}, {"lower_fast_v_mps", ParamValue{lower_fast_v_}}, {"lower_standoff_m", ParamValue{lower_standoff_m_}}, {"lower_standoff_k", ParamValue{lower_standoff_k_}}, {"land_edge_m", ParamValue{land_edge_m_}}, {"land_edge_max_m", ParamValue{land_edge_max_m_}}, {"stride_m", ParamValue{stride_m_}}, {"sink_max_m", ParamValue{sink_max_m_}}, {"tilt_disengage", ParamValue{tilt_disengage_}},
            {"dt_s", ParamValue{dt_}}, {"ik_iters", ParamValue{int64_t(ik_iters_)}}, {"ik_damping_m", ParamValue{ik_damping_m_}}, {"ik_step_max_rad", ParamValue{ik_step_max_rad_}}, {"ik_tol_m", ParamValue{ik_tol_m_}}};
}

void PoseSequencer::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("PoseSequencer requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"model_topic", &model_topic_}, {"support_topic", &support_topic_}, {"patch_topic", &patch_topic_}, {"feet_xz_topic", &feet_xz_topic_},
            {"feet_y_topic", &feet_y_topic_}, {"up_topic", &up_topic_}, {"imu_topic", &imu_topic_}, {"contact_topic", &contact_topic_}, {"load_topic", &load_topic_}, {"output_topic", &output_topic_}, {"belly_topic", &belly_topic_}, {"odom_in_topic", &odom_in_topic_}, {"odom_out_topic", &odom_out_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    apply_param(params, "gait_action_topics", [&](auto const& v){ if (auto p = std::get_if<std::vector<std::string>>(&v)) gait_action_topics_ = *p; });
    apply_param(params, "order", [&](auto const& v){ if (auto p = std::get_if<std::vector<double>>(&v)) order_ = *p; });
    for (auto const& k : {"engage", "engage_ramp_ticks", "elev_source", "crouch_max_m", "ik_reseed_ticks", "ik_multistart", "free_form", "front_drop_gain", "clip_ticks", "unloaded_shift_m", "land_r_min_m", "ext_margin_m", "ext_lifting", "overdue", "ext_urgent_m", "unloaded_share", "unloaded_ticks", "unloaded_landings_min", "unloaded_skips_min", "miss_reacquire", "miss_ticks", "null_step_m", "front_drop_max_m", "front_drop_deadband", "front_drop_tau_ticks", "depth_max_m", "free_margin_m", "free_out_m", "free_noise_m", "face_gap_m", "face_penalty_m", "ik_switch_m", "hang_ticks", "drag_back_m", "free_back_m", "free_extra_m", "free_retract_m", "ik_reseed_rate_rad", "crouch_v_mps", "crouch_relax_v_mps", "stand_h_m", "level_gain", "level_max_m", "sector_half_rad", "land_sector_rad", "land_band_lo_m", "land_band_hi_m", "neutral_r_m", "settle_v_mps", "heading_hold", "heading_ref", "heading_gain", "yaw_rate_max", "shift_realised", "reach_min_m", "retreat_m", "centre_gain", "advance_m", "margin_extra_m", "bear_tau_ticks", "bear_on", "bear_off", "touch_ticks", "touch_load", "creep_v_mps", "creep_load_min", "advance_reach_m", "relief", "plan", "plan_shift_m", "plan_shift_fwd_m", "plan_land_m", "plan_cell_m", "plan_adv_w", "relief_step_m", "relief_cells", "creep_load_relax", "creep_max_m", "creep_relax_v_mps", "elev_deadband_m", "elev_tau_ticks", "air_ticks_min", "elev_max_m", "reach_max_m", "unreach_tol_m", "engage_min_belly_m", "engage_min_tick", "engage_min_contacts", "phase_max_ticks", "shift_v_mps", "shift_max_m", "margin_min_m", "lift_clear_m", "lift_v_mps", "swing_v_mps", "lower_v_mps", "lower_fast_v_mps", "lower_standoff_m", "lower_standoff_k", "land_edge_m", "land_edge_max_m",
                          "stride_m", "sink_max_m", "tilt_disengage", "dt_s", "ik_iters", "ik_damping_m", "ik_step_max_rad", "ik_tol_m"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which, int idx) {
        if (topic.empty()) return;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which, idx](std::string_view, MessagePtr p){ handle(which, idx, p); }));
    };
    sub(model_topic_, "model", 0); sub(support_topic_, "support", 0); sub(patch_topic_, "patch", 0); sub(feet_xz_topic_, "xz", 0); sub(feet_y_topic_, "y", 0);
    sub(up_topic_, "up", 0); sub(imu_topic_, "imu", 0); sub(contact_topic_, "contact", 0); sub(load_topic_, "load", 0); sub(belly_topic_, "belly", 0); sub(odom_in_topic_, "odom", 0);
    for (int i = 0; i < int(gait_action_topics_.size()) && i < 12; ++i) sub(gait_action_topics_[size_t(i)], "gait", i);
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct, [this](std::string_view topic, MessagePtr p){
        if (!input_allowed(p->producer_id)) return;
        auto e = std::dynamic_pointer_cast<const EnvEvent>(p); if (!e) return;
        const std::string name(topic.substr(std::string(topics::kEventsPrefix).size()));
        if (name == "reset" || name == "miss") { ++resets_seen_; if (phase_ != Phase::Idle) { phase_ = Phase::Idle; leg_ = -1; } engage_level_ = 0.0f; }
    }));
}

void PoseSequencer::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "engage")           engage_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "engage_ramp_ticks") engage_ramp_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "stand_h_m")        stand_h_m_ = std::max(0.02, get_double(value, k));
    else if (k == "level_gain")       level_gain_ = std::clamp(get_double(value, k), 0.0, 1.5);
    else if (k == "level_max_m")      level_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "sector_half_rad")  sector_half_rad_ = std::clamp(get_double(value, k), 0.1, 3.0);
    else if (k == "land_sector_rad")  land_sector_rad_ = std::clamp(get_double(value, k), 0.1, 3.0);
    else if (k == "land_band_lo_m")   land_band_lo_m_ = std::max(0.0, get_double(value, k));
    else if (k == "land_band_hi_m")   land_band_hi_m_ = std::max(0.0, get_double(value, k));
    else if (k == "neutral_r_m")      neutral_r_m_ = std::max(0.03, get_double(value, k));
    else if (k == "settle_v_mps")     settle_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "heading_hold")     heading_hold_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "heading_ref")      { const std::string v = get_string(value, k); if (v != "zero" && v != "engage") throw std::invalid_argument("PoseSequencer: heading_ref must be \"zero\" or \"engage\""); heading_ref_ = v; }
    else if (k == "heading_gain")     heading_gain_ = std::max(0.0, get_double(value, k));
    else if (k == "shift_realised")   shift_realised_ = std::clamp(get_double(value, k), 0.1, 1.5);
    else if (k == "yaw_rate_max")     yaw_rate_max_ = std::max(0.0, get_double(value, k));
    else if (k == "order")            { if (auto p = std::get_if<std::vector<double>>(&value)) order_ = *p; }
    else if (k == "centre_gain")      centre_gain_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "advance_m")        advance_m_ = std::max(0.0, get_double(value, k));
    else if (k == "bear_tau_ticks")   bear_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "bear_on")          bear_on_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "bear_off")         bear_off_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "touch_ticks")      touch_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "touch_load")       touch_load_ = std::max(0.0, get_double(value, k));
    else if (k == "plan")             plan_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "plan_shift_m")     plan_shift_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plan_shift_fwd_m") plan_shift_fwd_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plan_land_m")      plan_land_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plan_cell_m")      plan_cell_m_ = std::max(2e-3, get_double(value, k));
    else if (k == "plan_adv_w")       plan_adv_w_ = std::max(0.0, get_double(value, k));
    else if (k == "relief")           relief_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "relief_step_m")    relief_step_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "relief_cells")     relief_cells_ = std::clamp(int(get_double(value, k)), 0, 20);
    else if (k == "advance_reach_m")  advance_reach_m_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_load_min")   creep_load_min_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "creep_load_relax") creep_load_relax_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "creep_v_mps")      creep_v_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_max_m")      creep_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_relax_v_mps") creep_relax_v_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_deadband_m")  elev_deadband_m_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_tau_ticks")   elev_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "air_ticks_min")    air_ticks_min_ = std::max(1, int(get_double(value, k)));
    else if (k == "margin_extra_m")   margin_extra_m_ = std::max(0.0, get_double(value, k));
    else if (k == "reach_min_m")      reach_min_m_ = std::max(0.0, get_double(value, k));
    else if (k == "retreat_m")        retreat_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "ik_reseed_rate_rad") ik_reseed_rate_ = std::clamp(get_double(value, k), 1e-3, 0.5);
    else if (k == "hang_ticks")       hang_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "drag_back_m")      drag_back_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_back_m")      free_back_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_extra_m")     free_extra_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_retract_m")   free_retract_m_ = std::max(0.0, get_double(value, k));
    else if (k == "land_r_min_m")     land_r_min_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ext_margin_m")     ext_margin_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ext_lifting")      ext_lifting_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "overdue")          overdue_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "ext_urgent_m")     ext_urgent_m_ = std::max(0.0, get_double(value, k));
    else if (k == "unloaded_shift_m") unloaded_shift_m_ = std::max(0.0, get_double(value, k));
    else if (k == "unloaded_share")   unloaded_share_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "unloaded_skips_min") unloaded_skips_min_ = std::max(0, int(get_double(value, k)));
    else if (k == "unloaded_landings_min") unloaded_landings_min_ = std::max(1, int(get_double(value, k)));
    else if (k == "unloaded_ticks")   unloaded_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "clip_ticks")       clip_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "miss_reacquire")   miss_reacquire_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "null_step_m")      null_step_m_ = std::max(0.0, get_double(value, k));
    else if (k == "miss_ticks")       miss_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "front_drop_gain") front_drop_gain_ = std::max(0.0, get_double(value, k));
    else if (k == "front_drop_max_m") front_drop_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "front_drop_tau_ticks") front_drop_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "front_drop_deadband") front_drop_deadband_ = std::max(0.0, get_double(value, k));
    else if (k == "depth_max_m")      depth_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_form")        free_form_ = std::clamp(int(get_double(value, k)), 2, 3);
    else if (k == "free_margin_m")    free_margin_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_out_m")       free_out_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_noise_m")     free_noise_m_ = std::max(0.0, get_double(value, k));
    else if (k == "face_gap_m")       face_gap_m_ = std::max(0.0, get_double(value, k));
    else if (k == "face_penalty_m")   face_penalty_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ik_multistart")    ik_multistart_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "ik_switch_m")      ik_switch_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ik_reseed_ticks")  ik_reseed_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "crouch_max_m")     crouch_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "crouch_v_mps")     crouch_v_ = std::max(0.0, get_double(value, k));
    else if (k == "crouch_relax_v_mps") crouch_relax_v_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_source")      { const std::string x = get_string(value, k); if (x == "patch" || x == "landed") elev_source_ = x; }
    else if (k == "elev_max_m")       elev_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "reach_max_m")      reach_max_m_ = std::max(0.03, get_double(value, k));
    else if (k == "unreach_tol_m")    unreach_tol_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "engage_min_belly_m") engage_min_belly_m_ = get_double(value, k);
    else if (k == "engage_min_tick")  engage_min_tick_ = std::max(0, int(get_double(value, k)));
    else if (k == "engage_min_contacts") engage_min_contacts_ = std::clamp(int(get_double(value, k)), 0, 4);
    else if (k == "phase_max_ticks")  phase_max_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "shift_v_mps")      shift_v_ = std::max(0.0, get_double(value, k));
    else if (k == "shift_max_m")      shift_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "margin_min_m")     margin_min_m_ = get_double(value, k);
    else if (k == "lift_clear_m")     lift_clear_m_ = std::max(0.0, get_double(value, k));
    else if (k == "lift_v_mps")       lift_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "swing_v_mps")      swing_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "lower_v_mps")      lower_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "lower_fast_v_mps") lower_fast_v_ = std::max(0.0, get_double(value, k));
    else if (k == "lower_standoff_m") lower_standoff_m_ = std::max(0.0, get_double(value, k));
    else if (k == "lower_standoff_k") lower_standoff_k_ = std::max(0.0, get_double(value, k));
    else if (k == "land_edge_m")      land_edge_m_ = std::max(0.0, get_double(value, k));
    else if (k == "land_edge_max_m")  land_edge_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "stride_m")         stride_m_ = std::max(0.0, get_double(value, k));
    else if (k == "sink_max_m")       sink_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "tilt_disengage")   tilt_disengage_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "dt_s")             dt_ = std::max(1e-3, get_double(value, k));
    else if (k == "ik_iters")         ik_iters_ = std::clamp(int(get_double(value, k)), 1, 32);
    else if (k == "ik_damping_m")     ik_damping_m_ = std::max(1e-4, get_double(value, k));
    else if (k == "ik_step_max_rad")  ik_step_max_rad_ = std::max(1e-3, get_double(value, k));
    else if (k == "ik_tol_m")         ik_tol_m_ = std::max(1e-4, get_double(value, k));
}

void PoseSequencer::handle(std::string const& which, int idx, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    if (which == "gait") {
        auto a = std::dynamic_pointer_cast<const ActionOut>(p);
        if (!a || idx < 0 || idx >= 12) return;
        gait_u_[size_t(idx)] = a->accel; have_gait_ = true; return;
    }
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if (which == "model" && v.size() >= kModelFloats) {
        auto vec = [&](int k) { return Vec3f(v[k], v[k + 1], v[k + 2]); };
        for (int i = 0; i < 4; ++i) {
            const int b = 27 * i;
            auto& a = anchors_[size_t(i)];
            a.hip1_world = vec(b); a.hip2_world = vec(b + 3); a.knee_world = vec(b + 6);
            a.coxa_rest_origin = vec(b + 9); a.upper_rest_origin = vec(b + 12); a.lower_rest_origin = vec(b + 15);
            a.hip2_axis = vec(b + 18); a.knee_axis = vec(b + 21); toe_off_[size_t(i)] = vec(b + 24);
        }
        const int m = 108;
        hip1_rest_ = v[m]; hip1_range_ = v[m + 1]; hip2_rest_ = v[m + 2]; hip2_range_ = v[m + 3];
        knee_rest_ = v[m + 4]; knee_fold_ = v[m + 5]; knee_hyper_ = v[m + 6]; knee_widening_ = v[m + 7] > 0.5f;
        have_model_ = true;
        if (!ext_lmax_ok_) {   // the extension margin's reference, once: each leg's longest hip2-to-toe over the knee's range (rigid under hip1 and hip2)
            const double klo = knee_rest_ - (knee_widening_ ? knee_hyper_ : hip2_range_), khi = knee_rest_ + (knee_widening_ ? knee_fold_ : hip2_range_);
            for (int i = 0; i < 4; ++i) {
                double best = 0.0;
                for (int j = 0; j <= 256; ++j) { const double k = klo + (khi - klo) * j / 256.0; best = std::max(best, double(len(sub(toe_of(i, {0.0, 0.0, k}), anchors_[size_t(i)].hip2_world)))); }
                ext_lmax_[size_t(i)] = best;
            }
            ext_lmax_ok_ = true;
        }
    }
    else if (which == "support" && v.size() >= 26) { for (int i = 0; i < 26; ++i) support_[size_t(i)] = v[i]; have_support_ = true; }
    else if (which == "patch" && v.size() >= 4) {
        const int N = int(v[0]); if (N < 2 || v.size() < 4 + 2 * N * N) return;
        N_ = N; cell_ = std::max(1e-3, double(v[1])); patch_.assign(v.data(), v.data() + 4 + 2 * N * N); have_patch_ = true;
    }
    else if (which == "xz" && v.size() >= 8) { for (int i = 0; i < 8; ++i) xz_[size_t(i)] = v[i]; have_xz_ = true; }
    else if (which == "y" && v.size() >= 4) { for (int i = 0; i < 4; ++i) y_[size_t(i)] = v[i]; have_y_ = true; }
    else if (which == "contact" && v.size() >= 4) { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = v[i]; have_contact_ = true; }
    else if (which == "load" && v.size() >= 4) { for (int i = 0; i < 4; ++i) load_[size_t(i)] = v[i]; have_load_ = true; }
    else if (which == "belly" && v.size() >= 1) { belly_ = v[0]; have_belly_ = true; }
    else if (which == "odom" && v.size() >= 3) { odom_in_ = {v[0], v[1], v[2]}; have_odom_in_ = true; }
    else if (which == "imu" && v.size() >= 6) { for (int i = 0; i < 6; ++i) imu_[size_t(i)] = v[i]; have_imu_ = true; }
    else if (which == "up" && v.size() >= 3) { const double n = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]); if (n > 0.5) { up_ = {v[0] / n, v[1] / n, v[2] / n}; have_up_ = true; } }
}

// ---- the kinematics ----
Vec3f PoseSequencer::toe_of(int leg, const std::array<double, 3>& t) const {
    const auto pose = body::fk_leg(anchors_[size_t(leg)], t[0], t[1], t[2], 0.0);
    return pose.lower.xform(toe_off_[size_t(leg)]);
}
std::array<double, 3> PoseSequencer::angles_of_u(int leg, const std::array<double, 3>& u) const {
    (void)leg;
    const double kr = knee_widening_ ? (u[2] >= 0.0 ? knee_fold_ : knee_hyper_) : hip2_range_;
    return {u[0] * hip1_range_ + hip1_rest_, u[1] * hip2_range_ + hip2_rest_, u[2] * kr + knee_rest_};
}
std::array<double, 3> PoseSequencer::u_of_angles(int leg, const std::array<double, 3>& t) const {
    (void)leg;
    const double dk = t[2] - knee_rest_;
    const double kr = knee_widening_ ? (dk >= 0.0 ? knee_fold_ : knee_hyper_) : hip2_range_;
    return {std::clamp((t[0] - hip1_rest_) / hip1_range_, -1.0, 1.0), std::clamp((t[1] - hip2_rest_) / hip2_range_, -1.0, 1.0), std::clamp(dk / kr, -1.0, 1.0)};
}
// one damped least-squares step toward `target`: dt = J^T (J J^T + lambda^2 I)^-1 e, J by central differences, the step bounded
bool PoseSequencer::ik_step(int leg, const Vec3f& target, std::array<double, 3>& t) const {
    const Vec3f cur = toe_of(leg, t);
    const Vec3f e = sub(target, cur);
    const double en = len(e);
    if (en < ik_tol_m_) return true;
    double J[3][3]; const double h = 1e-3;
    for (int j = 0; j < 3; ++j) {
        auto tp = t, tm = t; tp[size_t(j)] += h; tm[size_t(j)] -= h;
        const Vec3f d = sub(toe_of(leg, tp), toe_of(leg, tm));
        J[0][j] = d.x / (2 * h); J[1][j] = d.y / (2 * h); J[2][j] = d.z / (2 * h);
    }
    // A = J J^T + lambda^2 I (3x3), solve A y = e, dt = J^T y
    const double l2 = ik_damping_m_ * ik_damping_m_;
    double A[9];
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) { double s = 0; for (int k = 0; k < 3; ++k) s += J[r][k] * J[c][k]; A[r * 3 + c] = s + (r == c ? l2 : 0.0); }
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) + A[2] * (A[3] * A[7] - A[4] * A[6]);
    if (std::fabs(det) < 1e-18) return false;
    const double id = 1.0 / det;
    const double Ai[9] = { (A[4] * A[8] - A[5] * A[7]) * id, -(A[1] * A[8] - A[2] * A[7]) * id,  (A[1] * A[5] - A[2] * A[4]) * id,
                          -(A[3] * A[8] - A[5] * A[6]) * id,  (A[0] * A[8] - A[2] * A[6]) * id, -(A[0] * A[5] - A[2] * A[3]) * id,
                           (A[3] * A[7] - A[4] * A[6]) * id, -(A[0] * A[7] - A[1] * A[6]) * id,  (A[0] * A[4] - A[1] * A[3]) * id };
    const double ev[3] = {e.x, e.y, e.z};
    double y[3]; for (int r = 0; r < 3; ++r) y[r] = Ai[r * 3] * ev[0] + Ai[r * 3 + 1] * ev[1] + Ai[r * 3 + 2] * ev[2];
    double dt[3]; double mx = 0.0;
    for (int j = 0; j < 3; ++j) { dt[j] = J[0][j] * y[0] + J[1][j] * y[1] + J[2][j] * y[2]; mx = std::max(mx, std::fabs(dt[j])); }
    const double sc = mx > ik_step_max_rad_ ? ik_step_max_rad_ / mx : 1.0;
    for (int j = 0; j < 3; ++j) t[size_t(j)] += dt[j] * sc;
    // joint limits, in action units
    const auto u = u_of_angles(leg, t); t = angles_of_u(leg, u);
    return false;
}

bool PoseSequencer::patch_h(double fwd, double lat, double& h) const { double sg; return patch_h(fwd, lat, h, sg); }

// the relief of the remembered 3 x 3 cells around a body-frame point (max - min height); false with fewer than two cells known
bool PoseSequencer::patch_edge(double fwd, double lat, double& relief) const {
    relief = 0.0;
    if (!have_patch_) return false;
    const int ci = int(std::floor(fwd / cell_)) + N_ / 2, cj = int(std::floor(lat / cell_)) + N_ / 2;
    double lo = 1e9, hi = -1e9; int n = 0;
    for (int di = -1; di <= 1; ++di) for (int dj = -1; dj <= 1; ++dj) {
        const int i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= N_ || j >= N_) continue;
        const float hv = patch_[size_t(4 + i * N_ + j)]; const double w = patch_[size_t(4 + N_ * N_ + i * N_ + j)];
        if (hv <= -98.0f || w <= 0.0) continue;
        lo = std::min(lo, double(hv)); hi = std::max(hi, double(hv)); ++n;
    }
    if (n < 2) return false;
    relief = hi - lo; return true;
}

bool PoseSequencer::patch_h(double fwd, double lat, double& h, double& sigma) const {
    h = 0.0; sigma = 0.0;
    if (!have_patch_) return false;
    const int ci = int(std::floor(fwd / cell_)) + N_ / 2, cj = int(std::floor(lat / cell_)) + N_ / 2;
    double sw = 0.0, swh = 0.0;
    for (int di = -1; di <= 1; ++di) for (int dj = -1; dj <= 1; ++dj) {
        const int i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= N_ || j >= N_) continue;
        const float hv = patch_[size_t(4 + i * N_ + j)]; const double w = patch_[size_t(4 + N_ * N_ + i * N_ + j)];
        if (hv <= -98.0f || w <= 0.0) continue;
        sw += w; swh += w * hv;
    }
    if (sw <= 0.0) return false;
    h = swh / sw; sigma = std::sqrt(1.0 / sw); return true;   // the precision-weighted mean's own uncertainty
}

// ---- the sequence ----
// the signed distance of (cx, cz) inside the triangle p (+ inside) and the inward unit normal of the edge nearest it
double PoseSequencer::tri_margin(const std::array<std::array<double, 2>, 3>& p, double cx, double cz, double& nx, double& nz) {
    // orient counter-clockwise in the (x, z) plane
    std::array<std::array<double, 2>, 3> q = p;
    const double area = (q[1][0] - q[0][0]) * (q[2][1] - q[0][1]) - (q[1][1] - q[0][1]) * (q[2][0] - q[0][0]);
    if (area < 0) std::swap(q[1], q[2]);
    double best = 1e9; nx = 0; nz = 0;
    for (int i = 0; i < 3; ++i) {
        const auto& a = q[size_t(i)]; const auto& b = q[size_t((i + 1) % 3)];
        const double ex = b[0] - a[0], ez = b[1] - a[1], L = std::hypot(ex, ez); if (L < 1e-9) continue;
        const double d = (ex * (cz - a[1]) - ez * (cx - a[0])) / L;   // + on the interior side of a CCW edge
        if (d < best) { best = d; nx = -ez / L; nz = ex / L; }         // the inward normal of that edge
    }
    return best;
}

// the leg's workspace: a sector about its neutral heading (hip1 -> hip2) from its hip, radius in [reach_min, reach_max]
bool PoseSequencer::sector_clamp(int leg, Vec3f& p) const {
    const Vec3f hip = hip1_of(leg); const Vec3f h2 = anchors_[size_t(leg)].hip2_world;
    const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::hypot(hx, hz);
    if (hn < 1e-6) return false;
    const double ux = hx / hn, uz = hz / hn;                      // the neutral heading
    double dx = p.x - hip.x, dz = p.z - hip.z; const double r = std::hypot(dx, dz);
    if (r < 1e-9) { p.x = float(hip.x + ux * reach_min_m_); p.z = float(hip.z + uz * reach_min_m_); return true; }
    const double ang = std::atan2(dx * uz - dz * ux, dx * ux + dz * uz);   // signed angle from the heading
    const double a = std::clamp(ang, -sector_half_rad_, sector_half_rad_), rr = std::clamp(r, reach_min_m_, reach_max_m_);
    const bool clamped = std::fabs(a - ang) > 1e-9 || std::fabs(rr - r) > 1e-9;
    if (!clamped) return false;
    const double c = std::cos(a), sn = std::sin(a);               // the heading rotated by a, in the x-z plane
    const double vx = ux * c + uz * sn, vz = uz * c - ux * sn;
    const double nx_ = hip.x + vx * rr, nz_ = hip.z + vz * rr;
    const bool moved = std::hypot(nx_ - p.x, nz_ - p.z) > 1e-4;    // blocked means the clamp took more than a tenth of a millimetre back
    p.x = float(nx_); p.z = float(nz_);
    return moved;
}

void PoseSequencer::engage_now(uint64_t tick_id) {
    for (int i = 0; i < 4; ++i) {
        std::array<double, 3> u{0.0, 0.0, 0.0};
        if (have_gait_) u = {gait_u_[size_t(3 * i)], gait_u_[size_t(3 * i + 1)], gait_u_[size_t(3 * i + 2)]};
        angles_[size_t(i)] = angles_of_u(i, u);
        const Vec3f toe = toe_of(i, angles_[size_t(i)]);
        // the stance: this toe's place in the plane, kept within reach of its hip, at the standing height
        Vec3f t(toe.x, float(-stand_h_m_), toe.z);
        if (sector_clamp(i, t)) {   // a toe at the edge of its reach gets a centimetre of room, so the first shift is not blocked by it
            const Vec3f hip = hip1_of(i); const double dx = hip.x - t.x, dz = hip.z - t.z, dn = std::max(1e-6, std::hypot(dx, dz));
            t.x += float(dx / dn * 0.01); t.z += float(dz / dn * 0.01);
        }
        toe_tgt_[size_t(i)] = t;
        // the neutral place: neutral_r along the leg's heading from its hip, at the standing height
        const Vec3f hip = hip1_of(i), h2 = anchors_[size_t(i)].hip2_world;
        const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::max(1e-6, std::hypot(hx, hz));
        neutral_[size_t(i)] = Vec3f(float(hip.x + hx / hn * neutral_r_m_), float(-stand_h_m_), float(hip.z + hz / hn * neutral_r_m_));
        elev_[size_t(i)] = 0.0; elev_landed_[size_t(i)] = 0.0; ground_h_[size_t(i)] = -stand_h_m_; creep_[size_t(i)] = 0.0; air_ticks_[size_t(i)] = 0;
    }
    shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ticks_ = 0; leg_ = -1; phase_ = Phase::Settle; engaged_at_ = int64_t(tick_id); crouch_ = 0.0;
    yaw_ref_ = (heading_ref_ == "engage" && have_odom_in_) ? odom_in_[2] : 0.0; yaw_err_ = 0.0;
}

// the planted toes' heights: stand_h below the body plus each toe's relative elevation, recentred on their mean
void PoseSequencer::hold_stance() {
    // each toe's ground elevation: where it landed (elev_source landed), or from the memory under its place
    if (elev_source_ == "landed") { for (int i = 0; i < 4; ++i) elev_[size_t(i)] = elev_landed_[size_t(i)]; }
    else if (have_patch_) {
        double hs[4]; bool ok[4]; int n_ok = 0; double sum = 0.0;
        for (int i = 0; i < 4; ++i) { ok[i] = patch_h(toe_tgt_[size_t(i)].z, toe_tgt_[size_t(i)].x, hs[i]); if (ok[i]) { sum += hs[i]; ++n_ok; } }
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
    if (front_drop_gain_ > 0.0 && have_up_) {   // lever 3: the nose-up pitch beyond the deadband drops the front, through a first-order filter
        const double nose_up = std::atan2(double(up_[2]), std::max(0.2, double(up_[1])));
        const double target = std::clamp(front_drop_gain_ * std::max(0.0, nose_up - front_drop_deadband_), 0.0, front_drop_max_m_);
        front_drop_ += (target - front_drop_) / front_drop_tau_ticks_;
    } else front_drop_ = 0.0;
    std::array<double, 4> lev{}; double lev_mean = 0.0; int n_lev = 0;
    if (have_up_) {
        for (int i = 0; i < 4; ++i) {
            lev[size_t(i)] = std::clamp(level_gain_ * (toe_tgt_[size_t(i)].x * up_[0] + toe_tgt_[size_t(i)].z * up_[2]) / std::max(0.2, up_[1]), -level_max_m_, level_max_m_);
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
        if (unloaded_shift_m_ > 0.0 && have_contact_ && have_load_) {   // the unloaded toe: touching (the flag on) but carrying nothing
            if (double(load_[size_t(i)]) < unloaded_share_) ++unloaded_run_[size_t(i)]; else unloaded_run_[size_t(i)] = 0;   // the flag may flicker on an unloaded toe; the load is the fact
            land_max_load_[size_t(i)] = std::max(land_max_load_[size_t(i)], double(load_[size_t(i)]));
            if (unload_cool_[size_t(i)] > 0) --unload_cool_[size_t(i)];
        }
        if (have_contact_) {
            if (!bearing(i)) ++air_ticks_[size_t(i)]; else air_ticks_[size_t(i)] = 0;   // airborne by the flag (the re-step's rule)
            if (creep_load_min_ > 0.0 && have_load_) {   // by the LOAD: down while unloaded, back only while it carries its share
                if (double(load_[size_t(i)]) < creep_load_min_) { creep_[size_t(i)] = std::min(creep_max_m_, creep_[size_t(i)] + creep_v_ * dt_); ++creep_load_ticks_; }
                else if (double(load_[size_t(i)]) > creep_load_relax_) creep_[size_t(i)] = std::max(0.0, creep_[size_t(i)] - creep_relax_v_ * dt_);
            } else {
                if (!bearing(i)) creep_[size_t(i)] = std::min(creep_max_m_, creep_[size_t(i)] + creep_v_ * dt_);
                else creep_[size_t(i)] = std::max(0.0, creep_[size_t(i)] - creep_relax_v_ * dt_);
            }
        }
        // BODY-frame heights (equal legs = a level body on flat ground) plus a LEVELLING term from the IMU: the side the
        // body's up vector leans away from is low, and its toes are sent deeper by level_gain of the geometric correction
        // (x ux + z uz) / uy.  (Equal GRAVITY heights were tried, 2026-11-08: four toes on a flat floor share one gravity
        // height at any roll, so that stance held whatever attitude the body had and it rolled to the disengage limit.)
        toe_tgt_[size_t(i)].y = float(-(stand_h_m_ - crouch_) + std::clamp(elev_[size_t(i)] - mean, -elev_max_m_, elev_max_m_) - creep_[size_t(i)] + (lev[size_t(i)] - lev_mean));
        if (front_drop_ > 0.0 && toe_tgt_[size_t(i)].z > 0.0f) toe_tgt_[size_t(i)].y += float(front_drop_);   // the front of the chassis comes down
        if (depth_max_m_ > 0.0 && toe_tgt_[size_t(i)].y < float(-depth_max_m_)) { toe_tgt_[size_t(i)].y = float(-depth_max_m_); ++depth_caps_; }
    }
}

// the leg whose toe is furthest behind its neutral place steps next (the crawl's order emerges); among equals, the model's
// safest
int PoseSequencer::choose_leg() {
    if (have_contact_) for (int i = 0; i < 4; ++i) if (!bearing(i) && air_ticks_[size_t(i)] >= air_ticks_min_) return i;   // a toe the creep could not bring to ground is placed first
    if (!order_.empty()) {   // the fixed order: the next leg whose turn it is
        order_pos_ = (order_pos_ + 1) % int(order_.size());
        return std::clamp(int(order_[size_t(order_pos_)]), 0, 3);
    }
    // the toe furthest from its neutral place steps next -- behind it (the body has advanced), or tucked in or out
    // (the body has swayed); the crawl's order emerges from the geometry, and the model's margin breaks ties
    int best = -1; double best_score = -1e9;
    for (int i = 0; i < 4; ++i) {
        const double dx = double(neutral_[size_t(i)].x - toe_tgt_[size_t(i)].x), dz = double(neutral_[size_t(i)].z - toe_tgt_[size_t(i)].z);
        (void)dx;
        const double away = std::max(0.0, dz);   // behind its place (a sway moves every toe sideways alike; a toe AHEAD of its place is fine where it is)
        const double margin = have_support_ ? double(support_[size_t(10 + 5 * i)]) : 0.0;
        const double age = double(int64_t(tick_now_) - last_step_tick_[size_t(i)]) * 1e-6;   // among equals, the leg that stepped longest ago
        const double score = away + 0.2 * margin + age;
        score_[size_t(i)] = score;
        if (score > best_score) { best_score = score; best = i; }
    }
    return best;
}

void PoseSequencer::step_machine() {
    ++phase_ticks_;
    const double adv = shift_v_ * dt_;
    switch (phase_) {
    case Phase::Idle: break;
    case Phase::Settle: {   // the toes slide to their neutral places; then the crawl begins
        bool all_there = true; const double step = settle_v_ * dt_;
        for (int i = 0; i < 4; ++i) {
            Vec3f& tg = toe_tgt_[size_t(i)]; const double dx = neutral_[size_t(i)].x - tg.x, dz = neutral_[size_t(i)].z - tg.z, dn = std::hypot(dx, dz);
            if (dn <= step) { tg.x = neutral_[size_t(i)].x; tg.z = neutral_[size_t(i)].z; } else { tg.x += float(dx / dn * step); tg.z += float(dz / dn * step); all_there = false; }
        }
        ++settle_ticks_;
        if (all_there && phase_ticks_ > 10) { decide_next(tick_now_); shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ = Phase::Shift; phase_ticks_ = 0; }
        break; }
    case Phase::Shift: {
        // the body's move: into the triangle the other toes make, by the margin still missing, and toward the toes' centroid
        const double cx = have_support_ ? double(support_[3]) : 0.0, cz = have_support_ ? double(support_[2]) : 0.0;   // the model's centre of mass (lat = x, fwd = z)
        std::array<std::array<double, 2>, 3> tri{}; int k = 0; double mx = 0, mz = 0;
        for (int i = 0; i < 4; ++i) { mx += toe_tgt_[size_t(i)].x * 0.25; mz += toe_tgt_[size_t(i)].z * 0.25; if (i != leg_ && k < 3) { tri[size_t(k)] = {double(toe_tgt_[size_t(i)].x), double(toe_tgt_[size_t(i)].z)}; ++k; } }
        double nx = 0, nz = 0; const double m_local = tri_margin(tri, cx, cz, nx, nz);
        const double model_margin = have_support_ ? double(support_[size_t(10 + 5 * leg_)]) : 1.0;
        const int bearing = have_support_ ? int(support_[1] + 0.5f) : 4;
        // the margin still missing, by the local geometry and -- when all four toes bear and the model disagrees -- by the model
        const double m_req = margin_min_m_ + margin_extra_m_ + (free_form_ == 3 && free_pending_ > 0 ? free_margin_m_ : 0.0);   // a freeing lift asks for more
        double need = std::max(0.0, m_req - m_local);
        if (bearing >= 4) need = std::max(need, m_req - model_margin);
        double fwd_left = std::max(0.0, advance_m_ - fwd_acc_);
        if (advance_reach_m_ > 0.0) {   // a planted toe near its reach limit: the body does not walk away from it
            bool stretched = false;
            for (int i = 0; i < 4; ++i) { if (i == leg_) continue; const Vec3f hip = hip1_of(i); if (std::hypot(toe_tgt_[size_t(i)].x - hip.x, toe_tgt_[size_t(i)].z - hip.z) >= reach_max_m_ - advance_reach_m_) stretched = true; }
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
            if (fwd_left > 0.0 && m_after < m_local && m_after < margin_min_m_ + margin_extra_m_) { dz -= std::min(adv, fwd_left); fwd_blocked_ = true; }
        }
        const double dn = std::hypot(dx, dz);
        if (dn > adv) { dx *= adv / dn; dz *= adv / dn; }
        shift_dx_ = dx; shift_dz_ = dz; fwd_acc_ += std::max(0.0, dz);
        {   // the sequencer's odometry: the body moved (dx, dz) in its own frame; forward along the dead-reckoning frame's heading
            const double yaw = have_odom_in_ ? odom_in_[2] : 0.0, c = std::cos(yaw), sn = std::sin(yaw);
            px_ += shift_realised_ * (dz * c - dx * sn); py_ += shift_realised_ * (dz * sn + dx * c);
        }
        if (heading_hold_ > 0.0 && have_odom_in_) {   // the heading hold: turn the body back by rotating the planted toes the other way
            double e = odom_in_[2] - yaw_ref_; while (e > M_PI) e -= 2 * M_PI; while (e < -M_PI) e += 2 * M_PI;
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
            const bool clamped = sector_clamp(i, toe_tgt_[size_t(i)]);
            if (clamped && !(plan_active_ && i == leg_)) { blocked = true; blocked_toe_[size_t(i)] = true; if (blocked_leg_ < 0) blocked_leg_ = i; }   // under a plan the lifting toe's own clamp does not block: it is about to leave the ground (it blocked 9 of 10 planned shifts)
        }   // the toes move the other way, as far as they reach
        shift_acc_ += std::hypot(dx, dz); ++shift_ticks_;
        if (blocked) ++shift_blocked_;
        // may the leg go?  By the geometry the shift is steering on; the model vetoes only when all four toes bear (with
        // fewer, a flickering flag makes every other leg's margin the floor, and the shift would run to its bound)
        const bool safe_now = m_local >= margin_min_m_ && (bearing < 4 || model_margin >= margin_min_m_);
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
            auto margin_of_leg = [&](int i) { return have_support_ ? double(support_[size_t(10 + 5 * i)]) : 1.0; };
            auto local_margin_of_leg = [&](int c) {
                std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
                for (int i = 0; i < 4; ++i) if (i != c && kk < 3) { t3[size_t(kk)] = {double(toe_tgt_[size_t(i)].x), double(toe_tgt_[size_t(i)].z)}; ++kk; }
                double a, b; return tri_margin(t3, cx, cz, a, b);
            };
            int cand = (blocked && blocked_leg_ >= 0) ? blocked_leg_ : -1;
            if (cand >= 0 && local_margin_of_leg(cand) < margin_min_m_) cand = -1;
            if (cand < 0 || margin_of_leg(cand) < margin_min_m_) {
                // the safest leg among those that NEED a step (behind their place): a toe already ahead has nothing to gain
                double bm = -1e9; cand = -1;
                for (int i = 0; i < 4; ++i) { const bool needs = neutral_[size_t(i)].z - toe_tgt_[size_t(i)].z > 0.0; if (needs && margin_of_leg(i) > bm) { bm = margin_of_leg(i); cand = i; } }
                if (cand < 0) for (int i = 0; i < 4; ++i) if (margin_of_leg(i) > bm) { bm = margin_of_leg(i); cand = i; }
                if (cand >= 0 && bm < margin_min_m_) cand = -1;
            }
            if (!order_.empty()) {   // under the fixed order the turn passes to the next leg; a toe at its edge takes it if it is safe
                int rl = -1; Vec3f rland; double rm = 0.0;
                if (cand >= 0 && cand == blocked_leg_ && cand != leg_) { leg_ = cand; ++shift_exhausted_; go = true; }
                else if (relief_ > 0.0 && blocked_toe_[size_t(leg_)] && relief_for_ < 0 && plan_relief(leg_, cx, cz, bearing, rl, rland, rm)) {
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
            if (relief_pending_) { land_ = Vec3f(relief_land_.x, cur.y, relief_land_.z); relief_pending_ = false; }
            else if (plan_pending_) { land_ = Vec3f(plan_land_.x, cur.y, plan_land_.z); plan_pending_ = false; }
            else land_ = Vec3f(neutral_[size_t(leg_)].x, cur.y, float(neutral_[size_t(leg_)].z + stride_m_ * 0.5));
            {   // a centimetre inside the reach, so the swing can finish and the toe can still lower
                const Vec3f hip = hip1_of(leg_); const double dx = land_.x - hip.x, dz = land_.z - hip.z, r = std::hypot(dx, dz), rm = reach_max_m_ - 0.01;
                if (r > rm) { land_.x = float(hip.x + dx * rm / r); land_.z = float(hip.z + dz * rm / r); }
                const double wide = sector_half_rad_; sector_half_rad_ = std::min(wide, land_sector_rad_); sector_clamp(leg_, land_); sector_half_rad_ = wide;
            }
            // the ground: under the toe now and at the landing, from the memory where it has it, else the stance's own height
            const double stance_h = gravity_h(Vec3f(cur.x, float(-stand_h_m_), cur.z));
            const double h_here = gravity_h(cur);                   // the toe stands here: the ground under it is where it is
            if (land_edge_m_ > 0.0) {   // a riser's edge under the landing: move it along the heading, a cell at a time, to flat remembered ground
                double relief; const Vec3f hip = hip1_of(leg_); Vec3f best = land_; bool moved = false;
                if (patch_edge(land_.z, land_.x, relief) && relief >= land_edge_m_) {
                    for (int dir : {+1, -1}) {   // forward first (onto the tread), then back
                        for (int k = 1; k * cell_ <= land_edge_max_m_ + 1e-9; ++k) {
                            Vec3f cand(land_.x, land_.y, float(land_.z + dir * k * cell_));
                            if (std::hypot(cand.x - hip.x, cand.z - hip.z) > reach_max_m_ - 0.01) break;
                            double r2; if (patch_edge(cand.z, cand.x, r2) && r2 < land_edge_m_) { best = cand; moved = true; break; }
                        }
                        if (moved) break;
                    }
                    if (moved) { land_ = best; ++edge_moves_; } else ++edge_stuck_;
                }
            }
            double h_land = stance_h, hp, sg = 0.0; land_sigma_ = 0.0;
            if (patch_h(land_.z, land_.x, hp, sg)) {                 // the patch's frame: fwd = z, lat = x, as TerrainMemory stores the toes
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
        if (hang_ticks_ > 0 && have_contact_) {   // THE HUNG TOE: asked up but never off the ground, or the body dragged back by it
            const bool up_asked = phase_ == Phase::Swing || double(toe_tgt_[size_t(leg_)].y) + stand_h_m_ >= 0.02;
            const bool on = contact_[size_t(leg_)] > 0.5f;
            if (up_asked && on && !left_ground_) ++hang_run_; else if (!on) hang_run_ = 0;
            if (have_imu_ && have_up_) {   // the forward linear acceleration (gravity removed by the body's up), integrated over this lift+swing
                const double a_lin_z = double(imu_[2]) - 9.81 * up_[2]; a_lin_z_ema_ += 0.01 * (a_lin_z - a_lin_z_ema_);
                swing_v_fwd_ += (a_lin_z - a_lin_z_ema_) * dt_; swing_d_fwd_ += swing_v_fwd_ * dt_;
            }
            const bool caught = hang_run_ >= hang_ticks_, dragged = drag_back_m_ > 0.0 && swing_d_fwd_ < -drag_back_m_;
            if ((caught || dragged) && free_pending_ < 3) {
                ++hangs_; if (caught) ++hangs_by_contact_; if (dragged) ++hangs_by_drag_; ++hang_count_[size_t(leg_)];
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
        if (have_contact_ && double(toe_tgt_[size_t(leg_)].y) + stand_h_m_ >= 0.02) {   // the obstacle reflex, armed once the toe is asked 2 cm up: it left the ground and something stopped it
            const bool on = contact_[size_t(leg_)] > 0.5f;
            if (!on) { left_ground_ = true; obstacle_run_ = 0; }
            else if (left_ground_ && ++obstacle_run_ >= 5) {   // sustained a tenth of a second: a brush of the floor is briefer
                ++obstacle_hits_; land_ = toe_tgt_[size_t(leg_)]; land_h_ = -stand_h_m_;
                phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; left_ground_ = false; obstacle_run_ = 0;
                break;
            }
        }
        if (phase_ == Phase::Lift) {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        if (free_retract_left_ > 0.0) {   // freeing: first drag the toe along the ground toward the hip, away from the face
            const Vec3f hip = hip1_of(leg_); const double dx = hip.x - tg.x, dz = hip.z - tg.z, dn = std::max(1e-6, std::hypot(dx, dz)), st = std::min(free_retract_left_, swing_v_ * dt_);
            tg.x += float(dx / dn * st); tg.z += float(dz / dn * st); free_retract_left_ -= st;
            break;
        }
        const bool freeing3 = free_form_ == 3 && free_pending_ > 0;
        Vec3f top(float(tg.x + (freeing3 ? free_out_dx_ : 0.0)), float(-stand_h_m_ + lift_dy_), tg.z);
        if (freeing3) { Vec3f probe = top; if (sector_clamp(leg_, probe)) top.x = probe.x; }   // outward only as far as the leg's sector allows
        const double dy = double(top.y - tg.y), dx = double(top.x - tg.x), step = lift_v_ * dt_;
        tg.y += float(std::fabs(dy) <= step ? dy : (dy > 0 ? step : -step));
        if (freeing3) tg.x += float(std::fabs(dx) <= step ? dx : (dx > 0 ? step : -step));
        const bool arrived = std::fabs(dy) <= step && (!freeing3 || std::fabs(dx) <= step);
        if (clip_ticks_ > 0) {   // the leg cannot follow: take the part of the arc it reached and move on
            const Vec3f cmd_now = toe_of(leg_, angles_[size_t(leg_)]);
            if (len(sub(cmd_now, tg)) > unreach_tol_m_) ++clip_run_; else clip_run_ = 0;
            if (clip_run_ >= clip_ticks_) { tg = cmd_now; lift_dy_ = double(tg.y) + stand_h_m_; ++clips_; clip_run_ = 0; phase_ = Phase::Swing; phase_ticks_ = 0; break; }
        }
        if (arrived && len(sub(toe_of(leg_, angles_[size_t(leg_)]), tg)) < 2.0 * ik_tol_m_) { phase_ = Phase::Swing; phase_ticks_ = 0; }
        else if (phase_ticks_ > phase_max_ticks_ * (freeing3 ? 2 : 1)) { ++phase_timeouts_; phase_ = Phase::Swing; phase_ticks_ = 0; }
        } else {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f goal(land_.x, float(-stand_h_m_ + lift_dy_), land_.z);
        const Vec3f d = sub(goal, tg); const double dn = len(d), step = swing_v_ * dt_;
        if (dn <= step) tg = goal; else tg = add(tg, Vec3f(float(d.x / dn * step), float(d.y / dn * step), float(d.z / dn * step)));
        if (clip_ticks_ > 0) {
            const Vec3f cmd_now = toe_of(leg_, angles_[size_t(leg_)]);
            if (len(sub(cmd_now, tg)) > unreach_tol_m_) ++clip_run_; else clip_run_ = 0;
            if (clip_run_ >= clip_ticks_) { tg = cmd_now; ++clips_; clip_run_ = 0; phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; break; }
        }
        if (dn <= step && len(sub(toe_of(leg_, angles_[size_t(leg_)]), tg)) < 2.0 * ik_tol_m_) { phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; }
        else if (phase_ticks_ > phase_max_ticks_) { ++phase_timeouts_; tg = goal; phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; }
        }
        break;
    case Phase::Lower: {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f cmd = toe_of(leg_, angles_[size_t(leg_)]);
        const bool unreachable = len(sub(cmd, tg)) > unreach_tol_m_;
        double v = lower_v_;
        if (lower_fast_v_ > 0.0 && land_sigma_ > 0.0) {            // the memory knows the ground at the landing: fast to a standoff above it, careful beneath
            const double h_tg = double(tg.y) + stand_h_m_ + gravity_h(Vec3f(tg.x, float(-stand_h_m_), tg.z));
            const double standoff = lower_standoff_m_ + lower_standoff_k_ * std::max(land_sigma_, std::sqrt(land_err_ms_));   // the memory's own uncertainty, or the sequencer's running landing error where the memory is overconfident (sigma_touch 5 mm against 1-3 cm of landing error)
            if (h_tg > land_h_ + standoff) { v = lower_fast_v_; ++fast_ticks_; } else ++slow_ticks_;
        }
        const bool cant_reach_down = unreachable && double(cmd.y) > double(tg.y) + 0.005;   // the leg is ABOVE its target and cannot get down
        if (unreachable && !cant_reach_down) ++lower_fold_ticks_;    // the leg is below a target it cannot fold up to: descending helps (a 50 mm riser left a front leg at both its stops with its target above the tread, 2026-11-09)
        if (cant_reach_down && crouch_max_m_ > 0.0) { crouch_ = std::min(crouch_max_m_, crouch_ + crouch_v_ * dt_); ++crouch_ticks_; }   // the body comes down to the toe
        if (!cant_reach_down) tg.y -= float(v * dt_);
        else if (phase_ticks_ % 5 == 0) {                            // out of reach downward: the landing retreats toward the hip and the toe tries again
            const Vec3f hip = hip1_of(leg_); const double dx = hip.x - tg.x, dz = hip.z - tg.z, dn = std::hypot(dx, dz);
            if (dn > reach_min_m_ + retreat_m_) { tg.x += float(dx / dn * retreat_m_); tg.z += float(dz / dn * retreat_m_); ++retreats_; ++lower_retreats_; }
        }
        if (depth_max_m_ > 0.0 && tg.y < float(-depth_max_m_)) { tg.y = float(-depth_max_m_); ++depth_caps_; }   // never into the pole
        const double h_cmd = double(cmd.y) + stand_h_m_ + gravity_h(Vec3f(cmd.x, float(-stand_h_m_), cmd.z));   // the commanded toe's height in the stance's terms
        const bool raw_touch = have_contact_ && contact_[size_t(leg_)] > 0.5f && (!have_load_ || load_[size_t(leg_)] >= float(touch_load_));
        touch_run_ = raw_touch ? touch_run_ + 1 : 0;
        const bool touched = touch_run_ >= touch_ticks_ && phase_ticks_ > 3;
        const bool sunk = h_cmd < land_h_ - sink_max_m_ || phase_ticks_ > (miss_reacquire_ > 0.0 && miss_ticks_ > 0 ? miss_ticks_ : 4 * phase_max_ticks_) || (miss_reacquire_ > 0.0 && lower_retreats_ >= 6);
        if (touched || sunk) {
            if (sunk) ++lower_sunk_;
            // the toe's height where it stopped, against the toes already down: its relative elevation; and the memory's
            // prediction of the ground here against where the toe met it -- the careful stepper's error
            const double h_toe = (touched && have_y_) ? double(y_[size_t(leg_)]) : h_cmd;
            ground_h_[size_t(leg_)] = h_toe;
            elev_landed_[size_t(leg_)] = double(cmd.y) + stand_h_m_;   // the body-frame height the command had at the touch: the toe stands here
            if (touched) hang_count_[size_t(leg_)] = 0;
            free_pending_ = 0; free_retract_left_ = 0.0; free_out_dx_ = free_out_dy_ = 0.0;
            const bool null_step = miss_reacquire_ > 0.0 && std::hypot(double(cmd.x) - lift_from_.x, double(cmd.z) - lift_from_.z) < null_step_m_;
            if (touched) { land_err_last_ = h_toe - land_h_; land_err_ms_ += 0.1 * (land_err_last_ * land_err_last_ - land_err_ms_); ++landings_; if (!null_step) miss_count_[size_t(leg_)] = 0; else { ++null_steps_; ++miss_count_[size_t(leg_)]; } }
            else { ++missed_; if (miss_reacquire_ > 0.0) { ++miss_count_[size_t(leg_)]; reseeding_[size_t(leg_)] = true; } }   // re-acquire: the leg walks to its neutral pose, its next attempt differs
            last_landed_ = leg_;   // the unloaded-toe rule reads what this leg carries before its next turn
            tg = cmd;   // the stance takes it from here
            ++steps_[size_t(leg_)]; last_step_tick_[size_t(leg_)] = int64_t(tick_now_);
            cycles_ = *std::min_element(steps_.begin(), steps_.end());   // full crawl cycles: every leg has stepped this many times
            if (relief_for_ >= 0 && relief_for_ != leg_) { leg_ = relief_for_; relief_for_ = -1; plan_active_ = false; } else { relief_for_ = -1; decide_next(tick_now_); }
            shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ = Phase::Shift; phase_ticks_ = 0;
        }
        break; }
    }
}

// the extension margin: the leg's full extension minus the hip2-to-toe distance for a toe at p, the leg's plane taken through the
// hip1 axis (hip2 sits L1h out from hip1 along the toe's bearing, at its fixed height)
double PoseSequencer::ext_slack(int leg, const Vec3f& p) const {
    if (!ext_lmax_ok_) return 1.0;
    const Vec3f hip = hip1_of(leg), h2 = anchors_[size_t(leg)].hip2_world;
    const double l1h = std::hypot(double(h2.x) - hip.x, double(h2.z) - hip.z), r = std::hypot(double(p.x) - hip.x, double(p.z) - hip.z);
    return ext_lmax_[size_t(leg)] - std::hypot(r - l1h, double(p.y) - h2.y);
}
// the same at the commanded angles by the true FK (an instrument: checks the plane approximation against the solved leg)
double PoseSequencer::ext_slack_fk(int leg) const {
    if (!ext_lmax_ok_) return 1.0;
    const auto& t = angles_[size_t(leg)];
    const Vec3f hip = hip1_of(leg), h2 = anchors_[size_t(leg)].hip2_world;
    const double c = std::cos(t[0]), sn = std::sin(t[0]), dx = double(h2.x) - hip.x, dz = double(h2.z) - hip.z;   // hip2 about the vertical hip1 axis (Godot's rotation about +Y)
    const Vec3f h2n(float(hip.x + c * dx + sn * dz), h2.y, float(hip.z - sn * dx + c * dz));
    return ext_lmax_[size_t(leg)] - double(len(sub(toe_of(leg, t), h2n)));
}

// is the body shift (sx, sz) feasible with `lifting` in the air: every other toe stays within its reach and sector
bool PoseSequencer::shift_feasible(int lifting, const std::array<Vec3f, 4>& toes, double sx, double sz) const {
    for (int i = 0; i < 4; ++i) {
        if (i == lifting) {   // the lifting toe's reach and sector do not block (it is about to leave the ground); under ext_lifting its extension does
            if (ext_lifting_ > 0.0 && ext_margin_m_ > 0.0) {
                const Vec3f q(float(toes[size_t(i)].x - sx), toes[size_t(i)].y, float(toes[size_t(i)].z - sz));
                const double sa = ext_slack(i, q);
                const double floor_m = ext_lifting_ * ext_margin_m_;
                if (sa < floor_m && sa < ext_slack(i, toes[size_t(i)]) - 1e-9) { ++ext_lift_rejects_; return false; }
            }
            continue;
        }
        Vec3f q(float(toes[size_t(i)].x - sx), toes[size_t(i)].y, float(toes[size_t(i)].z - sz));
        const Vec3f hip = hip1_of(i); const double r = std::hypot(q.x - hip.x, q.z - hip.z);
        if (r < reach_min_m_ || r > reach_max_m_) return false;
        if (i == protect_ && ext_slack(i, q) < ext_slack(i, toes[size_t(i)]) - 1e-4) { ++overdue_rejects_; return false; }   // the overdue leg is not stretched further
        Vec3f probe = q; if (sector_clamp(i, probe)) return false;
        if (ext_margin_m_ > 0.0) {   // nor closer to its full extension than the margin, unless it was already and the shift does not stretch it further
            const double sa = ext_slack(i, q);
            if (sa < ext_margin_m_ && sa < ext_slack(i, toes[size_t(i)]) - 1e-9) { ++ext_rejects_; return false; }
        }
    }
    return true;
}

// the shift test's first refusal, in shift_feasible's own order and without its counters (the blocker instrument): 0 = feasible
int PoseSequencer::shift_refusal(int lifting, const std::array<Vec3f, 4>& toes, double sx, double sz, int& toe) const {
    for (int i = 0; i < 4; ++i) {
        toe = i;
        const Vec3f q(float(toes[size_t(i)].x - sx), toes[size_t(i)].y, float(toes[size_t(i)].z - sz));
        if (i == lifting) {
            if (ext_lifting_ > 0.0 && ext_margin_m_ > 0.0) { const double sa = ext_slack(i, q); if (sa < ext_lifting_ * ext_margin_m_ && sa < ext_slack(i, toes[size_t(i)]) - 1e-9) return 6; }
            continue;
        }
        const Vec3f hip = hip1_of(i); const double r = std::hypot(q.x - hip.x, q.z - hip.z);
        if (r > reach_max_m_) return 1;
        if (r < reach_min_m_) return 2;
        if (i == protect_ && ext_slack(i, q) < ext_slack(i, toes[size_t(i)]) - 1e-4) return 3;
        Vec3f probe = q; if (sector_clamp(i, probe)) return 4;
        if (ext_margin_m_ > 0.0) { const double sa = ext_slack(i, q); if (sa < ext_margin_m_ && sa < ext_slack(i, toes[size_t(i)]) - 1e-9) return 5; }
    }
    toe = -1; return 0;
}

// the blocker instrument: of the shifts that would give `leg` its lift margin, the smallest one's refusal (or "grid")
void PoseSequencer::diagnose_block(int leg, double cx, double cz, double need) {
    const int nl = int(std::floor(plan_shift_m_ / plan_cell_m_ + 1e-9)), nf = int(std::floor(plan_shift_fwd_m_ / plan_cell_m_ + 1e-9));
    double best_d = 1e9, mmax = -1e9; int best_toe = -1, best_r = 0;
    for (int ix = -nl; ix <= nl; ++ix) for (int iz = -nl; iz <= nf; ++iz) {
        const double tx = ix * plan_cell_m_, tz = iz * plan_cell_m_;
        std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
        for (int i = 0; i < 4; ++i) if (i != leg && kk < 3) { t3[size_t(kk)] = {double(toe_tgt_[size_t(i)].x) - tx, double(toe_tgt_[size_t(i)].z) - tz}; ++kk; }
        double a, b; const double m = tri_margin(t3, cx, cz, a, b);
        mmax = std::max(mmax, m);
        if (m < need) continue;
        const double d = std::hypot(tx, tz); if (d >= best_d) continue;
        int toe; const int r = shift_refusal(leg, toe_tgt_, tx, tz, toe);
        best_d = d; best_r = r; best_toe = r == 0 ? leg : toe;
    }
    if (best_toe < 0) { ++block_[size_t(leg)][28]; block_short_[size_t(leg)] += need - mmax; return; }
    ++block_[size_t(leg)][size_t(best_toe * 7 + best_r)]; block_d_[size_t(leg)] += best_d;
}

// the best lift margin `leg` can be given by a feasible shift of the body over `toes` (the centre of mass at (cx, cz) in the
// body frame moves with the body; the toes move the other way); forward_first prefers, among shifts that give the margin
// asked, the most forward one, else the largest margin
double PoseSequencer::best_margin_over_shifts(int leg, const std::array<Vec3f, 4>& toes, double cx, double cz, double& sx, double& sz, bool forward_first) const {
    double best = -1e9, best_key = -1e9; sx = 0.0; sz = 0.0;
    const double need = margin_min_m_ + margin_extra_m_;
    const int nl = int(std::floor(plan_shift_m_ / plan_cell_m_ + 1e-9)), nf = int(std::floor(plan_shift_fwd_m_ / plan_cell_m_ + 1e-9));
    for (int ix = -nl; ix <= nl; ++ix) for (int iz = -nl; iz <= nf; ++iz) {
        const double tx = ix * plan_cell_m_, tz = iz * plan_cell_m_;
        if (!shift_feasible(leg, toes, tx, tz)) continue;
        std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
        for (int i = 0; i < 4; ++i) if (i != leg && kk < 3) { t3[size_t(kk)] = {double(toes[size_t(i)].x) - tx, double(toes[size_t(i)].z) - tz}; ++kk; }
        double a, b; const double m = tri_margin(t3, cx, cz, a, b);
        // the key: with the margin met, forward progress then the shortest shift; without it, the margin itself
        const double key = forward_first ? (m >= need ? 10.0 + tz - 0.1 * std::hypot(tx, tz) : m) : m;
        if (key > best_key) { best_key = key; best = m; sx = tx; sz = tz; }
    }
    return best;
}

bool PoseSequencer::plan_step(int& leg_out, Vec3f& land_out, double& sx_out, double& sz_out, double& score_out) {
    const double cx = have_support_ ? double(support_[3]) : 0.0, cz = have_support_ ? double(support_[2]) : 0.0;
    const double need = margin_min_m_ + margin_extra_m_;
    const int n_order = order_.empty() ? 4 : int(order_.size());
    double best_score = -1e9; leg_out = -1;
    const int64_t skips0 = plan_skips_;
    const int nland = int(std::floor(plan_land_m_ / plan_cell_m_ + 1e-9));
    protect_ = -1;
    const int wleg0 = order_.empty() ? 0 : std::clamp(int(order_[size_t((order_pos_ + 1) % n_order)]), 0, 3);
    int urgent = -1;   // the overdue leg's queue jump: the planted leg with the least slack below ext_urgent_m
    if (ext_urgent_m_ > 0.0 && ext_lmax_ok_) { double lo = ext_urgent_m_; for (int i = 0; i < 4; ++i) { const double sl = ext_slack(i, toe_tgt_[size_t(i)]); if (sl < lo) { lo = sl; urgent = i; } } }
    if (urgent == wleg0 || jumped_) urgent = -1;   // it is the wave's leg already, or a leg has jumped this turn
    for (int k = (urgent >= 0 ? -1 : 0); k < n_order; ++k) {   // the wave's order first (an urgent leg before it); a later leg only when an earlier one has no feasible step
        const bool jump = k < 0;
        const int pos = jump ? -1 : (order_.empty() ? k : (order_pos_ + 1 + k) % n_order);
        const int c = jump ? urgent : (order_.empty() ? k : std::clamp(int(order_[size_t(pos)]), 0, 3));
        if (!jump && c == urgent) continue;   // tried first, and infeasible
        if (have_contact_ && !bearing(c) && air_ticks_[size_t(c)] < air_ticks_min_ && air_ticks_[size_t(c)] > 0) {}   // (a toe in the air is still a candidate: placing it is a step)
        auto skipped = [&]{   // the leg has no feasible step: under overdue the steps after it serve it
            if (jump) { ++urgent_fails_; if (overdue_ > 0.0) protect_ = c; return; }
            ++plan_skips_;
            if (k == 0 && overdue_ > 0.0 && protect_ < 0) protect_ = c;
        };
        double sx, sz; const double m_now = best_margin_over_shifts(c, toe_tgt_, cx, cz, sx, sz, true);
        if (m_now < need) { ++skip_nomargin_[size_t(c)]; diagnose_block(c, cx, cz, need); skipped(); continue; }
        if (miss_reacquire_ > 0.0 && miss_count_[size_t(c)] >= 2 && k < n_order - 1) { ++miss_skips_; ++skip_miss_[size_t(c)]; continue; }   // let the others step first; the last candidate is never skipped
        // the leg the landing must leave liftable: the wave's own next leg (the one being skipped when c is a later leg --
        // the first form protected only the leg after c and marched two legs of one side forward while the other side's
        // never became feasible, 89 / 3 / 90 / 2); when c IS the wave's next leg, the one after it
        const int wpos = order_.empty() ? 0 : (order_pos_ + 1) % n_order;
        const int wleg = order_.empty() ? 0 : std::clamp(int(order_[size_t(wpos)]), 0, 3);
        const int npos = order_.empty() ? (c + 1) % 4 : (pos + 1) % n_order;
        int nleg = (c == wleg) ? (order_.empty() ? npos : std::clamp(int(order_[size_t(npos)]), 0, 3)) : wleg;
        if (protect_ >= 0 && c != protect_) nleg = protect_;   // under overdue the landing must leave the overdue leg liftable
        // the toes after the shift, c still to land
        std::array<Vec3f, 4> after = toe_tgt_; for (int i = 0; i < 4; ++i) { after[size_t(i)].x -= float(sx); after[size_t(i)].z -= float(sz); }
        const Vec3f hip = hip1_of(c), h2 = anchors_[size_t(c)].hip2_world;
        double hx = h2.x - hip.x, hz = h2.z - hip.z; const double hn = std::max(1e-6, std::hypot(hx, hz)); hx /= hn; hz /= hn;
        const double lx = -hz, lz = hx;   // the heading's lateral
        double leg_best = -1e9; Vec3f leg_land{};
        double jx = 0.0, jz = 0.0;   // re-acquisition: a leg that missed searches a grid shifted by up to 2 cm, and is penalised by 1 cm of margin per miss
        if (miss_reacquire_ > 0.0 && miss_count_[size_t(c)] > 0) { auto nz = [&]{ rng_ = rng_ * 1664525u + 1013904223u; return (double(rng_ >> 8) / 16777216.0 - 0.5) * 0.04; }; jx = nz(); jz = nz(); }
        const double miss_pen = miss_reacquire_ > 0.0 ? 0.01 * miss_count_[size_t(c)] : 0.0;
        for (int ia = -nland; ia <= nland; ++ia) for (int il = -nland / 2; il <= nland / 2; ++il) {
            const double da = ia * plan_cell_m_ + jz, dl = il * plan_cell_m_ + jx;
            Vec3f L(float(neutral_[size_t(c)].x + hx * da + lx * dl), toe_tgt_[size_t(c)].y, float(neutral_[size_t(c)].z + hz * da + lz * dl));
            const double r = std::hypot(L.x - hip.x, L.z - hip.z); if (r < reach_min_m_ || r > reach_max_m_ - 0.01) continue;
            if (land_r_min_m_ > 0.0 && r < land_r_min_m_) { ++land_r_rejects_; continue; }   // the base is kept wide enough that a shift stays feasible
            { const double wide = sector_half_rad_; Vec3f probe = L; const bool clamped = [&]{ auto* self = const_cast<PoseSequencer*>(this); self->sector_half_rad_ = std::min(wide, land_sector_rad_); const bool cl = sector_clamp(c, probe); self->sector_half_rad_ = wide; return cl; }(); if (clamped) continue; }
            {   // the landing must be reachable at the stance's depth plus the landing band (a toe at the horizontal reach cannot lower: 7 of 8 misses sank at r 0.12-0.13)
                const Vec3f deep(L.x, float(-stand_h_m_ - land_band_lo_m_), L.z); auto ang = angles_[size_t(c)]; bool ok = false;
                for (int it = 0; it < 8 && !ok; ++it) ok = ik_step(c, deep, ang);
                if (!ok && len(sub(toe_of(c, ang), deep)) > 0.008) continue;
            }
            std::array<Vec3f, 4> landed = after; landed[size_t(c)] = L;
            double nsx, nsz; const double m_next = best_margin_over_shifts(nleg, landed, cx, cz, nsx, nsz, false);
            const double fwd = double(L.z) - double(neutral_[size_t(c)].z);   // progress: the landing ahead of the neutral place
            double score = std::min(m_next, 0.03) + plan_adv_w_ * fwd - miss_pen;   // a margin beyond 3 cm buys nothing more
            if (protect_ >= 0 && c != protect_ && m_next >= need) score += 0.1;   // a landing that leaves the overdue leg liftable ranks above any that does not
            if (face_gap_m_ > 0.0) {   // lever C: a face just ahead of the landing (a riser the next lift must clear from too close)
                double h0, h1; if (patch_h(L.z, L.x, h0) && patch_h(L.z + face_gap_m_, L.x, h1) && h1 - h0 >= land_edge_m_) { score -= face_penalty_m_; ++face_penalties_; }
            }
            if (score > leg_best) { leg_best = score; leg_land = L; }
        }
        if (leg_best <= -1e8) { ++skip_noland_[size_t(c)]; skipped(); continue; }
        if (jump) { best_score = leg_best; leg_out = c; land_out = leg_land; sx_out = sx; sz_out = sz; ++urgent_jumps_; jumped_ = true; break; }   // the urgent leg steps; the wave's turn is not consumed
        if (leg_best > best_score) { best_score = leg_best; leg_out = c; land_out = leg_land; sx_out = sx; sz_out = sz; if (k == 0) { order_pos_ = pos; jumped_ = false; } }   // a later leg's step does not consume the wave's turn
        if (k == 0) break;   // the wave's own next leg is feasible: take it (the search ranks its landing and shift, not its turn)
    }
    if (protect_ >= 0) ++overdue_decisions_;
    protect_ = -1;
    score_out = best_score; last_plan_skips_ = int(plan_skips_ - skips0);
    return leg_out >= 0;
}

void PoseSequencer::decide_next(uint64_t) {
    plan_active_ = false; plan_pending_ = false; plan_done_x_ = plan_done_z_ = 0.0; unload_shift_only_ = false;
    if (unloaded_shift_m_ > 0.0) {   // a toe that touches but carries nothing: the body goes over it first
        for (int k = 0; k < 5; ++k) {
            // first the leg that just landed and never came to bear (the loop's signature: it would be lifted again at once), then any toe unloaded for unloaded_ticks
            int i; bool fire;
            if (k == 0) {
                i = last_landed_; fire = false;
                if (i >= 0 && have_load_) {
                    if (land_max_load_[size_t(i)] < unloaded_share_) ++unloaded_landings_[size_t(i)]; else unloaded_landings_[size_t(i)] = 0;
                    fire = unloaded_landings_[size_t(i)] >= unloaded_landings_min_ && unload_cool_[size_t(i)] <= 0;
                    land_max_load_[size_t(i)] = 0.0;
                }
                last_landed_ = -1;
            }
            else { i = k - 1; fire = unloaded_run_[size_t(i)] >= unloaded_ticks_ && unload_cool_[size_t(i)] <= 0 && last_plan_skips_ >= unloaded_skips_min_; }
            if (!fire) continue;
            double mx = 0, mz = 0; for (int j = 0; j < 4; ++j) { mx += toe_tgt_[size_t(j)].x * 0.25; mz += toe_tgt_[size_t(j)].z * 0.25; }
            const double dx = toe_tgt_[size_t(i)].x - mx, dz = toe_tgt_[size_t(i)].z - mz, dn = std::max(1e-6, std::hypot(dx, dz));
            leg_ = i; plan_active_ = true; plan_pending_ = false; plan_sx_ = dx / dn * unloaded_shift_m_; plan_sz_ = dz / dn * unloaded_shift_m_;
            unload_shift_only_ = true; unload_cool_[size_t(i)] = 4 * unloaded_ticks_; unloaded_run_[size_t(i)] = 0; ++unload_shifts_;
            return;
        }
    }
    if (plan_ > 0.0) {
        int l; Vec3f L; double sx, sz, sc;
        if (plan_step(l, L, sx, sz, sc)) { leg_ = l; plan_land_ = L; plan_sx_ = sx; plan_sz_ = sz; plan_score_ = sc; plan_active_ = true; plan_pending_ = true; ++plans_; return; }
        ++plan_none_;
    }
    leg_ = choose_leg();
}

bool PoseSequencer::ik_solve(int leg, const Vec3f& target, std::array<double, 3>& t, bool allow_multistart) {
    bool done = false;
    for (int k = 0; k < ik_iters_ && !done; ++k) done = ik_step(leg, target, t);
    if (done || !allow_multistart) return done;
    const double e_cur = len(sub(toe_of(leg, t), target));
    if (e_cur <= 2.0 * ik_tol_m_) return false;
    ++multistarts_;
    // the seeds: the branches of a three-joint leg, in action units
    static const std::array<std::array<double, 3>, 6> seeds = {{{0.0, 0.0, 0.3}, {0.0, -0.6, 0.3}, {0.0, 0.6, 0.3}, {0.0, -0.3, -0.6}, {0.0, 0.0, 0.8}, {0.0, -0.6, -0.3}}};
    std::array<double, 3> best = t; double e_best = e_cur;
    const auto u_cur = u_of_angles(leg, t);
    for (auto const& su : seeds) {
        std::array<double, 3> a = angles_of_u(leg, {u_cur[0], su[1], su[2]});   // the yaw stays: the branches are in the leg's plane
        bool ok = false; for (int k = 0; k < 12 && !ok; ++k) ok = ik_step(leg, target, a);
        const double e = len(sub(toe_of(leg, a), target));
        if (e < e_best - 1e-9) { e_best = e; best = a; }
    }
    if (e_best < e_cur - ik_switch_m_) { t = best; ++branch_switches_; return e_best < ik_tol_m_; }
    return false;
}

bool PoseSequencer::plan_relief(int stuck, double cx, double cz, int bearing, int& leg_out, Vec3f& land_out, double& margin_out) const {
    double best = -1e9; leg_out = -1;
    for (int c = 0; c < 4; ++c) {
        if (c == stuck) continue;
        {   // c must be safe to lift now, by the geometry and (with all four bearing) by the model
            std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
            for (int i = 0; i < 4; ++i) if (i != c && kk < 3) { t3[size_t(kk)] = {double(toe_tgt_[size_t(i)].x), double(toe_tgt_[size_t(i)].z)}; ++kk; }
            double a, b; const double ml = tri_margin(t3, cx, cz, a, b);
            const double mm = have_support_ ? double(support_[size_t(10 + 5 * c)]) : 1.0;
            if (ml < margin_min_m_ || (bearing >= 4 && mm < margin_min_m_)) continue;
        }
        const Vec3f hip = hip1_of(c), h2 = anchors_[size_t(c)].hip2_world;
        double hx = h2.x - hip.x, hz = h2.z - hip.z; const double hn = std::max(1e-6, std::hypot(hx, hz)); hx /= hn; hz /= hn;
        for (int k = -relief_cells_; k <= relief_cells_; ++k) {
            Vec3f cand(float(neutral_[size_t(c)].x + hx * k * relief_step_m_), toe_tgt_[size_t(c)].y, float(neutral_[size_t(c)].z + hz * k * relief_step_m_));
            const double r = std::hypot(cand.x - hip.x, cand.z - hip.z);
            if (r < reach_min_m_ || r > reach_max_m_ - 0.01) continue;
            Vec3f probe = cand; if (sector_clamp(c, probe)) continue;
            std::array<std::array<double, 2>, 3> t3{}; int kk = 0;   // the stuck leg's support after c has landed there
            for (int i = 0; i < 4; ++i) if (i != stuck && kk < 3) { const Vec3f& q = (i == c) ? cand : toe_tgt_[size_t(i)]; t3[size_t(kk)] = {double(q.x), double(q.z)}; ++kk; }
            double a, b; const double m_after = tri_margin(t3, cx, cz, a, b);
            if (m_after > best) { best = m_after; leg_out = c; land_out = cand; }
        }
    }
    margin_out = best;
    return leg_out >= 0 && best >= margin_min_m_ + margin_extra_m_;
}

void PoseSequencer::publish(uint64_t tick_id) {
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); out->sensor = "pose";
    out->values = Eigen::VectorXf::Zero(13);
    out->values[0] = engage_level_;
    for (int i = 0; i < 4; ++i) { const auto u = u_of_angles(i, angles_[size_t(i)]); for (int j = 0; j < 3; ++j) out->values[1 + 3 * i + j] = float(u[size_t(j)]); }
    bus_->publish(output_topic_, out);
}

void PoseSequencer::tick(uint64_t tick_id) {
    tick_now_ = tick_id;
    if (have_contact_) {   // the leaky bearing estimate, with hysteresis
        const double a = 1.0 / bear_tau_ticks_;
        for (int i = 0; i < 4; ++i) {
            bear_[size_t(i)] += a * ((contact_[size_t(i)] > 0.5f ? 1.0 : 0.0) - bear_[size_t(i)]);
            if (bear_[size_t(i)] > bear_on_) bearing_[size_t(i)] = true; else if (bear_[size_t(i)] < bear_off_) bearing_[size_t(i)] = false;
        }
    }
    // the FK check against the body's own toes (the model is the body's; this is the wiring's check), from the gait's command
    if (have_model_ && have_gait_ && have_xz_ && have_y_ && phase_ == Phase::Idle) {
        double e = 0.0;
        for (int i = 0; i < 4; ++i) {
            const Vec3f toe = toe_of(i, angles_of_u(i, {gait_u_[size_t(3 * i)], gait_u_[size_t(3 * i + 1)], gait_u_[size_t(3 * i + 2)]}));
            e += std::hypot(double(toe.x) - xz_[size_t(2 * i)], double(toe.z) - xz_[size_t(2 * i + 1)]) / 4.0;
        }
        fk_err_ema_ += 0.02 * (e - fk_err_ema_);
    }
    const bool upright = !have_up_ || up_[1] >= tilt_disengage_;
    const bool want = engage_ > 0.0 && have_model_ && upright;
    if (!upright && phase_ != Phase::Idle) ++disengaged_tilt_;
    if (want && phase_ == Phase::Idle) {   // from a standing body only
        int n_contact = 0; if (have_contact_) for (int i = 0; i < 4; ++i) n_contact += bearing(i);
        const bool standing = int64_t(tick_id) >= engage_min_tick_ && (!have_contact_ || n_contact >= engage_min_contacts_)
                              && (belly_topic_.empty() || (have_belly_ && belly_ >= float(engage_min_belly_m_)));
        if (standing) engage_now(tick_id);
    }
    if (!want && phase_ != Phase::Idle) { phase_ = Phase::Idle; leg_ = -1; }
    const float ramp = 1.0f / float(engage_ramp_ticks_);
    engage_level_ = std::clamp(engage_level_ + (phase_ != Phase::Idle ? ramp : -ramp), 0.0f, 1.0f);
    if (phase_ != Phase::Idle) {
        ++ticks_engaged_;
        step_machine();
        hold_stance();
        double err = 0.0;
        for (int i = 0; i < 4; ++i) {
            if (ik_reseed_ticks_ > 0 && reseeding_[size_t(i)]) {   // walking back to the neutral standing pose at the step cap; the IK takes over when it arrives or can reach
                const auto nt = angles_of_u(i, {0.0, 0.0, 0.3}); bool there = true;
                for (int j = 0; j < 3; ++j) { const double d = nt[size_t(j)] - angles_[size_t(i)][size_t(j)]; const double st = std::clamp(d, -ik_reseed_rate_, ik_reseed_rate_); angles_[size_t(i)][size_t(j)] += st; if (std::fabs(d) > ik_reseed_rate_) there = false; }
                if (there || len(sub(toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)])) < 2.0 * unreach_tol_m_) { reseeding_[size_t(i)] = false; pinned_ticks_[size_t(i)] = 0; }
                err += len(sub(toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)])) / 4.0;
                continue;
            }
            ik_solve(i, toe_tgt_[size_t(i)], angles_[size_t(i)], ik_multistart_ > 0.0);
            const double e_i = len(sub(toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)]));
            err += e_i / 4.0;
            if (ik_reseed_ticks_ > 0) {   // pinned: a joint at its stop with the toe far from its target
                const auto u = u_of_angles(i, angles_[size_t(i)]); bool at_stop = false;
                for (int j = 0; j < 3; ++j) if (std::fabs(u[size_t(j)]) >= 0.999) at_stop = true;
                if (at_stop && e_i > 2.0 * unreach_tol_m_) { if (++pinned_ticks_[size_t(i)] >= ik_reseed_ticks_) { reseeding_[size_t(i)] = true; ++reseeds_; } }
                else pinned_ticks_[size_t(i)] = 0;
            }
        }
        ik_err_ema_ += 0.05 * (err - ik_err_ema_);
    }
    if (!odom_out_topic_.empty()) {   // the odometry: the sequencer's own while engaged, the body's passed through otherwise
        if (phase_ == Phase::Idle || !odom_anchored_) { if (have_odom_in_) { px_ = odom_in_[0]; py_ = odom_in_[1]; } odom_anchored_ = have_odom_in_; }
        auto od = std::make_shared<ProprioToken>();
        od->tick_id = tick_id; od->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); od->sensor = "odom_seq";
        od->values = Eigen::VectorXf(3); od->values[0] = float(px_); od->values[1] = float(py_); od->values[2] = float(have_odom_in_ ? odom_in_[2] : 0.0);
        bus_->publish(odom_out_topic_, od);
    }
    if (engage_level_ > 0.0f || phase_ != Phase::Idle || engage_ > 0.0) publish(tick_id);
    else if (!output_topic_.empty()) {   // engage 0: the socket sees 0 and the gait is untouched
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); out->sensor = "pose";
        out->values = Eigen::VectorXf::Zero(13); bus_->publish(output_topic_, out);
    }
}

nlohmann::json PoseSequencer::diag_snapshot() const {
    nlohmann::json tg = nlohmann::json::array(), st = nlohmann::json::array(), gh = nlohmann::json::array(), nt = nlohmann::json::array(), sc = nlohmann::json::array(), uu = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) { const auto u = u_of_angles(i, angles_[size_t(i)]); uu.push_back({u[0], u[1], u[2]}); }
    for (int i = 0; i < 4; ++i) { tg.push_back({toe_tgt_[size_t(i)].x, toe_tgt_[size_t(i)].y, toe_tgt_[size_t(i)].z}); st.push_back(steps_[size_t(i)]); gh.push_back(ground_h_[size_t(i)]);
                                  nt.push_back({neutral_[size_t(i)].x, neutral_[size_t(i)].z}); sc.push_back(score_[size_t(i)]); }
    return nlohmann::json{{"phase", phase_name(phase_)}, {"phase_id", int(phase_)}, {"leg", leg_}, {"engage", engage_level_}, {"model", have_model_}, {"support", have_support_},
                          {"patch", have_patch_}, {"steps", st}, {"cycles", cycles_}, {"shift_exhausted", shift_exhausted_}, {"shift_ticks", shift_ticks_},
                          {"disengaged_tilt", disengaged_tilt_}, {"lower_sunk", lower_sunk_}, {"phase_timeouts", phase_timeouts_}, {"engaged_at", engaged_at_},
                          {"landings", landings_}, {"missed", missed_}, {"land_h_rejected", land_h_rejected_}, {"fast_ticks", fast_ticks_}, {"slow_ticks", slow_ticks_}, {"edge_moves", edge_moves_}, {"edge_stuck", edge_stuck_}, {"land_sigma", land_sigma_}, {"blocked_leg", blocked_leg_}, {"advance_yields", advance_yields_}, {"relief_steps", relief_steps_}, {"plans", plans_}, {"lower_fold_ticks", lower_fold_ticks_}, {"crouch", crouch_}, {"reseeds", reseeds_}, {"multistarts", multistarts_}, {"face_penalties", face_penalties_}, {"front_drop", front_drop_}, {"clips", clips_}, {"unload_shifts", unload_shifts_}, {"land_r_rejects", land_r_rejects_}, {"ext_rejects", ext_rejects_}, {"ext_lift_rejects", ext_lift_rejects_}, {"overdue", {overdue_decisions_, overdue_rejects_, urgent_jumps_, urgent_fails_}}, {"skip_nomargin", skip_nomargin_}, {"skip_noland", skip_noland_}, {"skip_miss", skip_miss_}, {"block", block_}, {"block_d", block_d_}, {"block_short", block_short_}, {"ext", {ext_slack(0, toe_tgt_[0]), ext_slack(1, toe_tgt_[1]), ext_slack(2, toe_tgt_[2]), ext_slack(3, toe_tgt_[3])}}, {"ext_fk", {ext_slack_fk(0), ext_slack_fk(1), ext_slack_fk(2), ext_slack_fk(3)}}, {"null_steps", null_steps_}, {"miss_skips", miss_skips_}, {"miss_count", {miss_count_[0], miss_count_[1], miss_count_[2], miss_count_[3]}}, {"depth_caps", depth_caps_}, {"free_out", {free_out_dx_, free_out_dy_}}, {"branch_switches", branch_switches_}, {"hangs", hangs_}, {"hangs_by_contact", hangs_by_contact_}, {"hangs_by_drag", hangs_by_drag_}, {"swing_d_fwd", swing_d_fwd_}, {"a_lin_z_ema", a_lin_z_ema_}, {"crouch_ticks", crouch_ticks_}, {"elev_landed", {elev_landed_[0], elev_landed_[1], elev_landed_[2], elev_landed_[3]}}, {"plan_skips", plan_skips_}, {"plan_none", plan_none_}, {"plan_reached", plan_reached_}, {"plan_shift", {plan_sx_, plan_sz_}}, {"plan_score", plan_score_}, {"relief_none", relief_none_}, {"relief_for", relief_for_}, {"creep_load_ticks", creep_load_ticks_}, {"creep", {creep_[0], creep_[1], creep_[2], creep_[3]}}, {"settle_ticks", settle_ticks_}, {"obstacle_hits", obstacle_hits_}, {"resets_seen", resets_seen_}, {"odom", {px_, py_}}, {"yaw_err", yaw_err_}, {"shift_blocked", shift_blocked_}, {"retreats", retreats_}, {"shift_d", {shift_dx_, shift_dz_}}, {"steps_no_advance", steps_no_advance_}, {"deadlock_ticks", deadlock_ticks_}, {"turns_passed", turns_passed_}, {"creep", {creep_[0], creep_[1], creep_[2], creep_[3]}}, {"bear", {bear_[0], bear_[1], bear_[2], bear_[3]}}, {"land_err", land_err_last_}, {"land_rms", std::sqrt(land_err_ms_)}, {"elev", {elev_[0], elev_[1], elev_[2], elev_[3]}}, {"ik_err", ik_err_ema_}, {"fk_err", fk_err_ema_}, {"ticks_engaged", ticks_engaged_},
                          {"toe_tgt", tg}, {"u", uu}, {"ground_h", gh}, {"neutral", nt}, {"score", sc}, {"air_ticks", {air_ticks_[0], air_ticks_[1], air_ticks_[2], air_ticks_[3]}}, {"lift_h", lift_h_}, {"land_h", land_h_}, {"shift_acc", shift_acc_}, {"phase_ticks", phase_ticks_}};
}

} // namespace ogma
