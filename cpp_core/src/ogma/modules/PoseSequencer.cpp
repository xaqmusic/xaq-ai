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
const char* phase_name(PoseSequencer::Phase p) {
    switch (p) { case PoseSequencer::Phase::Idle: return "idle"; case PoseSequencer::Phase::Shift: return "shift"; case PoseSequencer::Phase::Lift: return "lift";
                 case PoseSequencer::Phase::Swing: return "swing"; case PoseSequencer::Phase::Lower: return "lower"; case PoseSequencer::Phase::Settle: return "settle"; }
    return "?";
}
} // namespace

PoseSequencer::PoseSequencer()
    : executor_(in_, floor_, book_, kin_, rng_), planner_(in_, floor_, book_, kin_, rng_, executor_) {
    executor_.intent_hook_ = [this] {   // the planner's intent, its planned landing offset by the learned layer when that is on
        const stepper::StepIntent in = planner_.decide();
        return offset_.on() ? offset_.decide(in, in_, floor_, planner_, executor_) : in;
    };
    executor_.relief_hook_ = [this](int stuck, double cx, double cz, int bearing, int& leg_out, Vec3f& land_out, double& margin_out) {
        return planner_.plan_relief(stuck, cx, cz, bearing, leg_out, land_out, margin_out); };
    executor_.order_ref_ = &planner_.order_;
}
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
    if (!outcome_topic_.empty()) v.push_back(TopicSpec{outcome_topic_, std::type_index(typeid(ProprioToken))});
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
        {"plan_next_first", ParamMutability::HotMutable, "1 = in the pose search a landing that leaves the next leg liftable ranks above every landing that does not (the advance chooses among those); 0 = off.", ParamValue{0.0}},
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
        {"ik_knee_guard", ParamMutability::HotMutable, "THE KNEE GUARD: 1 = a solve that leaves a knee past straight (hyperextended) is discarded -- the leg is reset to the pose it had before the solve, its knee turned to the bent side, and solved again with the knee held there; 0 = off (byte-identical). A straight leg is the solver's singularity, and past it the solver finds a pose on the target that the body does not take.", ParamValue{0.0}},
        {"ik_switch_m",   ParamMutability::HotMutable, "A branch from another seed is taken only when it beats the current solution's residual by this (m).", ParamValue{0.005}},
        // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): the two re-seed params.
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
        {"plan_model_margin", ParamMutability::HotMutable, "THE PLAN PREDICTS ITS GATE: 1 = the pose search lowers its own lift margin for a leg by however much SupportModel's margin for that leg sits below the geometric one at the present pose (the executor lets a leg go only when the model agrees), so a plan the gate would refuse is not issued again and again; 0 = off (byte-identical).", ParamValue{0.0}},
        {"plan_shift_m",  ParamMutability::HotMutable, "The search's body shifts: lateral and backward out to this (m) ...", ParamValue{0.03}},
        {"plan_shift_fwd_m", ParamMutability::HotMutable, "... forward out to this (m) ...", ParamValue{0.04}},
        {"plan_land_m",   ParamMutability::HotMutable, "... and landings out to this from the leg's neutral place (m), all on plan_cell_m.", ParamValue{0.04}},
        {"plan_cell_m",   ParamMutability::HotMutable, "The search grid (m).", ParamValue{0.01}},
        {"lo_gain",       ParamMutability::HotMutable, "THE LANDING OFFSET (learned stepper plan Phase 4; stepper/LandingOffset.hpp): the authority of the learned layer over a planned landing's forward/back offset -- the offsets on offer span lo_gain x lo_max_m either side of the scaffold's landing, chosen by expected free energy (risk against the step's outcome priors minus the model's epistemic value) among those the floor admits. 0 = the scaffold's landing (byte-identical with lo_learn 0). Read at every decision.", ParamValue{0.0}},
        {"lo_learn",      ParamMutability::HotMutable, "> 0: the landing offset's outcome model learns from every planned step and reports (diag lo); with lo_gain 0 it learns in shadow from the scaffold's own landings and moves nothing.", ParamValue{0.0}},
        {"lo_max_m",      ParamMutability::HotMutable, "The landing offset's full range either side (m) at lo_gain 1; also the unit of the model's offset feature.", ParamValue{0.03}},
        {"lo_cell_m",     ParamMutability::HotMutable, "The landing offset's candidate spacing (m).", ParamValue{0.01}},
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
        {"outcome_topic", ParamMutability::ConstructionOnly, "ProprioToken [26]: each step's outcome at its touch (stepper::StepOutcome: the planned and realised landing, the ground predicted and met, the ticks, the hang, every correction by rule) -- the learned step generator's training signal. \"\" = not published.", ParamValue{std::string("")}},
        {"odom_out_topic", ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the sequencer's own odometry (the body's commanded motion over the planted toes, the body's yaw) for the terrain memory's frame. \"\" = not published.", ParamValue{std::string("reality.proprio.odom_seq")}},
        {"heading_hold",  ParamMutability::HotMutable, "1 = hold the heading the body had at engage (the planted toes rotated during SHIFT); 0 = off.", ParamValue{1.0}},
        {"heading_ref",   ParamMutability::HotMutable, "\"zero\" = hold the dead-reckoning frame's own heading (the body's facing at its anchor, the corridor's axis); \"engage\" = the heading the body had at engage.", ParamValue{std::string("zero")}},
        {"shift_realised", ParamMutability::HotMutable, "The fraction of the commanded shift the body realises, applied to the odometry (measured 0.82: the planted toes slip).", ParamValue{0.82}},
        {"heading_gain",  ParamMutability::HotMutable, "Yaw rate commanded per radian of heading error (1/s).", ParamValue{0.6}},
        {"yaw_rate_max",  ParamMutability::HotMutable, "The most the body is turned per second (rad/s).", ParamValue{0.3}},
        {"settle_v_mps",  ParamMutability::HotMutable, "On engage the planted toes slide to their neutral places at this rate before the first step.", ParamValue{0.05}},
        {"settle",        ParamMutability::HotMutable, "1 = on engage the toes slide to their neutral places before the first step; 0 = the crawl starts from the gait's pose (the prune's leave-one-out, 2026-11-14).", ParamValue{1.0}},
        {"land_clamp",    ParamMutability::HotMutable, "1 = a step's landing is clamped a centimetre inside the reach and into the landing sector when it goes; 0 = taken as given (the prune's leave-one-out, 2026-11-14).", ParamValue{1.0}},
        {"obstacle_ticks", ParamMutability::HotMutable, "The obstacle reflex: a toe that left the ground and is stopped (contact) this many ticks during LIFT or SWING is lowered where it is; 0 = off (the prune's leave-one-out, 2026-11-14).", ParamValue{int64_t{5}}},
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
            {"stand_h_m", ParamValue{executor_.stand_h_m_}}, {"elev_source", ParamValue{executor_.elev_source_}}, {"crouch_max_m", ParamValue{executor_.crouch_max_m_}}, {"ik_reseed_ticks", ParamValue{int64_t(executor_.ik_reseed_ticks_)}}, {"ik_multistart", ParamValue{executor_.ik_multistart_}}, {"free_form", ParamValue{int64_t(executor_.free_form_)}}, {"front_drop_gain", ParamValue{executor_.front_drop_gain_}}, {"clip_ticks", ParamValue{int64_t(executor_.clip_ticks_)}}, {"unloaded_shift_m", ParamValue{book_.unloaded_shift_m_}}, {"land_r_min_m", ParamValue{planner_.land_r_min_m_}}, {"ext_margin_m", ParamValue{planner_.ext_margin_m_}}, {"ext_lifting", ParamValue{planner_.ext_lifting_}}, {"overdue", ParamValue{planner_.overdue_}}, {"plan_next_first", ParamValue{planner_.plan_next_first_}}, {"ext_urgent_m", ParamValue{planner_.ext_urgent_m_}}, {"unloaded_share", ParamValue{book_.unloaded_share_}}, {"unloaded_ticks", ParamValue{int64_t(book_.unloaded_ticks_)}}, {"unloaded_landings_min", ParamValue{int64_t(book_.unloaded_landings_min_)}}, {"unloaded_skips_min", ParamValue{int64_t(book_.unloaded_skips_min_)}}, {"miss_reacquire", ParamValue{executor_.miss_reacquire_}}, {"miss_ticks", ParamValue{int64_t(executor_.miss_ticks_)}}, {"null_step_m", ParamValue{executor_.null_step_m_}}, {"front_drop_max_m", ParamValue{executor_.front_drop_max_m_}}, {"front_drop_deadband", ParamValue{executor_.front_drop_deadband_}}, {"front_drop_tau_ticks", ParamValue{executor_.front_drop_tau_ticks_}}, {"depth_max_m", ParamValue{executor_.depth_max_m_}}, {"free_margin_m", ParamValue{executor_.free_margin_m_}}, {"free_out_m", ParamValue{executor_.free_out_m_}}, {"free_noise_m", ParamValue{executor_.free_noise_m_}}, {"face_gap_m", ParamValue{planner_.face_gap_m_}}, {"face_penalty_m", ParamValue{planner_.face_penalty_m_}}, {"ik_switch_m", ParamValue{kin_.p.ik_switch_m}}, {"ik_knee_guard", ParamValue{kin_.p.knee_guard}}, {"imu_topic", ParamValue{imu_topic_}}, {"hang_ticks", ParamValue{int64_t(executor_.hang_ticks_)}}, {"drag_back_m", ParamValue{executor_.drag_back_m_}}, {"free_back_m", ParamValue{executor_.free_back_m_}}, {"free_extra_m", ParamValue{executor_.free_extra_m_}}, {"free_retract_m", ParamValue{executor_.free_retract_m_}}, {"ik_reseed_rate_rad", ParamValue{executor_.ik_reseed_rate_}}, {"crouch_v_mps", ParamValue{executor_.crouch_v_}}, {"crouch_relax_v_mps", ParamValue{executor_.crouch_relax_v_}}, {"level_gain", ParamValue{executor_.level_gain_}}, {"level_max_m", ParamValue{executor_.level_max_m_}}, {"sector_half_rad", ParamValue{kin_.p.sector_half_rad}}, {"land_sector_rad", ParamValue{executor_.land_sector_rad_}}, {"land_band_lo_m", ParamValue{executor_.land_band_lo_m_}}, {"land_band_hi_m", ParamValue{executor_.land_band_hi_m_}}, {"neutral_r_m", ParamValue{executor_.neutral_r_m_}}, {"settle_v_mps", ParamValue{executor_.settle_v_}}, {"settle", ParamValue{executor_.settle_}}, {"land_clamp", ParamValue{executor_.land_clamp_}}, {"obstacle_ticks", ParamValue{int64_t(executor_.obstacle_ticks_)}}, {"odom_in_topic", ParamValue{odom_in_topic_}}, {"odom_out_topic", ParamValue{odom_out_topic_}}, {"outcome_topic", ParamValue{outcome_topic_}}, {"heading_hold", ParamValue{executor_.heading_hold_}}, {"heading_ref", ParamValue{executor_.heading_ref_}}, {"heading_gain", ParamValue{executor_.heading_gain_}}, {"shift_realised", ParamValue{executor_.shift_realised_}}, {"yaw_rate_max", ParamValue{executor_.yaw_rate_max_}}, {"order", ParamValue{planner_.order_}}, {"centre_gain", ParamValue{executor_.centre_gain_}}, {"advance_m", ParamValue{executor_.advance_m_}}, {"margin_extra_m", ParamValue{floor_.margin_extra_m_}}, {"creep_v_mps", ParamValue{executor_.creep_v_}}, {"creep_load_min", ParamValue{executor_.creep_load_min_}}, {"advance_reach_m", ParamValue{executor_.advance_reach_m_}}, {"relief", ParamValue{executor_.relief_}}, {"plan", ParamValue{planner_.plan_}}, {"plan_model_margin", ParamValue{planner_.plan_model_margin_}}, {"plan_shift_m", ParamValue{planner_.plan_shift_m_}}, {"plan_shift_fwd_m", ParamValue{planner_.plan_shift_fwd_m_}}, {"plan_land_m", ParamValue{planner_.plan_land_m_}}, {"plan_cell_m", ParamValue{planner_.plan_cell_m_}}, {"plan_adv_w", ParamValue{planner_.plan_adv_w_}}, {"relief_step_m", ParamValue{planner_.relief_step_m_}}, {"relief_cells", ParamValue{int64_t(planner_.relief_cells_)}}, {"creep_load_relax", ParamValue{executor_.creep_load_relax_}}, {"bear_tau_ticks", ParamValue{executor_.bear_tau_ticks_}}, {"bear_on", ParamValue{executor_.bear_on_}}, {"bear_off", ParamValue{executor_.bear_off_}}, {"touch_ticks", ParamValue{int64_t(executor_.touch_ticks_)}}, {"touch_load", ParamValue{executor_.touch_load_}}, {"creep_max_m", ParamValue{executor_.creep_max_m_}}, {"creep_relax_v_mps", ParamValue{executor_.creep_relax_v_}}, {"elev_deadband_m", ParamValue{executor_.elev_deadband_m_}}, {"elev_tau_ticks", ParamValue{executor_.elev_tau_ticks_}}, {"air_ticks_min", ParamValue{int64_t(executor_.air_ticks_min_)}}, {"reach_min_m", ParamValue{kin_.p.reach_min_m}}, {"retreat_m", ParamValue{executor_.retreat_m_}}, {"elev_max_m", ParamValue{executor_.elev_max_m_}}, {"reach_max_m", ParamValue{kin_.p.reach_max_m}}, {"unreach_tol_m", ParamValue{executor_.unreach_tol_m_}},
            {"engage_min_belly_m", ParamValue{floor_.engage_min_belly_m_}}, {"engage_min_tick", ParamValue{int64_t(floor_.engage_min_tick_)}}, {"engage_min_contacts", ParamValue{int64_t(floor_.engage_min_contacts_)}}, {"phase_max_ticks", ParamValue{int64_t(executor_.phase_max_ticks_)}}, {"shift_v_mps", ParamValue{executor_.shift_v_}}, {"shift_max_m", ParamValue{executor_.shift_max_m_}},
            {"margin_min_m", ParamValue{floor_.margin_min_m_}}, {"lift_clear_m", ParamValue{executor_.lift_clear_m_}}, {"lift_v_mps", ParamValue{executor_.lift_v_}}, {"swing_v_mps", ParamValue{executor_.swing_v_}},
            {"lower_v_mps", ParamValue{executor_.lower_v_}}, {"lower_fast_v_mps", ParamValue{executor_.lower_fast_v_}}, {"lower_standoff_m", ParamValue{executor_.lower_standoff_m_}}, {"lower_standoff_k", ParamValue{executor_.lower_standoff_k_}}, {"land_edge_m", ParamValue{executor_.land_edge_m_}}, {"land_edge_max_m", ParamValue{executor_.land_edge_max_m_}}, {"stride_m", ParamValue{executor_.stride_m_}}, {"sink_max_m", ParamValue{executor_.sink_max_m_}}, {"tilt_disengage", ParamValue{floor_.tilt_disengage_}},
            {"dt_s", ParamValue{executor_.dt_}}, {"ik_iters", ParamValue{int64_t(kin_.p.ik_iters)}}, {"ik_damping_m", ParamValue{kin_.p.ik_damping_m}}, {"ik_step_max_rad", ParamValue{kin_.p.ik_step_max_rad}}, {"ik_tol_m", ParamValue{kin_.p.ik_tol_m}},
            {"lo_gain", ParamValue{offset_.gain_}}, {"lo_learn", ParamValue{offset_.learn_}}, {"lo_max_m", ParamValue{offset_.max_m_}}, {"lo_cell_m", ParamValue{offset_.cell_m_}}};
}

void PoseSequencer::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("PoseSequencer requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"model_topic", &model_topic_}, {"support_topic", &support_topic_}, {"patch_topic", &patch_topic_}, {"feet_xz_topic", &feet_xz_topic_},
            {"feet_y_topic", &feet_y_topic_}, {"up_topic", &up_topic_}, {"imu_topic", &imu_topic_}, {"contact_topic", &contact_topic_}, {"load_topic", &load_topic_}, {"output_topic", &output_topic_}, {"belly_topic", &belly_topic_}, {"odom_in_topic", &odom_in_topic_}, {"odom_out_topic", &odom_out_topic_}, {"outcome_topic", &outcome_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    apply_param(params, "gait_action_topics", [&](auto const& v){ if (auto p = std::get_if<std::vector<std::string>>(&v)) gait_action_topics_ = *p; });
    apply_param(params, "order", [&](auto const& v){ if (auto p = std::get_if<std::vector<double>>(&v)) planner_.order_ = *p; });
    for (auto const& k : {"engage", "engage_ramp_ticks", "elev_source", "crouch_max_m", "ik_reseed_ticks", "ik_multistart", "free_form", "front_drop_gain", "clip_ticks", "unloaded_shift_m", "land_r_min_m", "ext_margin_m", "ext_lifting", "overdue", "plan_next_first", "ext_urgent_m", "unloaded_share", "unloaded_ticks", "unloaded_landings_min", "unloaded_skips_min", "miss_reacquire", "miss_ticks", "null_step_m", "front_drop_max_m", "front_drop_deadband", "front_drop_tau_ticks", "depth_max_m", "free_margin_m", "free_out_m", "free_noise_m", "face_gap_m", "face_penalty_m", "ik_switch_m", "ik_knee_guard", "hang_ticks", "drag_back_m", "free_back_m", "free_extra_m", "free_retract_m", "ik_reseed_rate_rad", "crouch_v_mps", "crouch_relax_v_mps", "stand_h_m", "level_gain", "level_max_m", "sector_half_rad", "land_sector_rad", "land_band_lo_m", "land_band_hi_m", "neutral_r_m", "settle_v_mps", "settle", "land_clamp", "obstacle_ticks", "heading_hold", "heading_ref", "heading_gain", "yaw_rate_max", "shift_realised", "reach_min_m", "retreat_m", "centre_gain", "advance_m", "margin_extra_m", "bear_tau_ticks", "bear_on", "bear_off", "touch_ticks", "touch_load", "creep_v_mps", "creep_load_min", "advance_reach_m", "relief", "plan", "plan_model_margin", "plan_shift_m", "plan_shift_fwd_m", "plan_land_m", "plan_cell_m", "plan_adv_w", "relief_step_m", "relief_cells", "creep_load_relax", "creep_max_m", "creep_relax_v_mps", "elev_deadband_m", "elev_tau_ticks", "air_ticks_min", "elev_max_m", "reach_max_m", "unreach_tol_m", "engage_min_belly_m", "engage_min_tick", "engage_min_contacts", "phase_max_ticks", "shift_v_mps", "shift_max_m", "margin_min_m", "lift_clear_m", "lift_v_mps", "swing_v_mps", "lower_v_mps", "lower_fast_v_mps", "lower_standoff_m", "lower_standoff_k", "land_edge_m", "land_edge_max_m",
                          "stride_m", "sink_max_m", "tilt_disengage", "dt_s", "ik_iters", "ik_damping_m", "ik_step_max_rad", "ik_tol_m", "lo_gain", "lo_learn", "lo_max_m", "lo_cell_m"})
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
        if (name == "reset" || name == "miss") { ++resets_seen_; if (executor_.phase_ != Phase::Idle) { executor_.phase_ = Phase::Idle; executor_.leg_ = -1; } engage_level_ = 0.0f; }
    }));
}

void PoseSequencer::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "engage")           engage_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "engage_ramp_ticks") engage_ramp_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "stand_h_m")        executor_.stand_h_m_ = std::max(0.02, get_double(value, k));
    else if (k == "level_gain")       executor_.level_gain_ = std::clamp(get_double(value, k), 0.0, 1.5);
    else if (k == "level_max_m")      executor_.level_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "sector_half_rad")  kin_.p.sector_half_rad = std::clamp(get_double(value, k), 0.1, 3.0);
    else if (k == "land_sector_rad")  executor_.land_sector_rad_ = std::clamp(get_double(value, k), 0.1, 3.0);
    else if (k == "land_band_lo_m")   executor_.land_band_lo_m_ = std::max(0.0, get_double(value, k));
    else if (k == "land_band_hi_m")   executor_.land_band_hi_m_ = std::max(0.0, get_double(value, k));
    else if (k == "neutral_r_m")      executor_.neutral_r_m_ = std::max(0.03, get_double(value, k));
    else if (k == "settle_v_mps")     executor_.settle_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "settle")           executor_.settle_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "land_clamp")       executor_.land_clamp_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "obstacle_ticks")   executor_.obstacle_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "heading_hold")     executor_.heading_hold_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "heading_ref")      { const std::string v = get_string(value, k); if (v != "zero" && v != "engage") throw std::invalid_argument("PoseSequencer: heading_ref must be \"zero\" or \"engage\""); executor_.heading_ref_ = v; }
    else if (k == "heading_gain")     executor_.heading_gain_ = std::max(0.0, get_double(value, k));
    else if (k == "shift_realised")   executor_.shift_realised_ = std::clamp(get_double(value, k), 0.1, 1.5);
    else if (k == "yaw_rate_max")     executor_.yaw_rate_max_ = std::max(0.0, get_double(value, k));
    else if (k == "order")            { if (auto p = std::get_if<std::vector<double>>(&value)) planner_.order_ = *p; }
    else if (k == "centre_gain")      executor_.centre_gain_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "advance_m")        executor_.advance_m_ = std::max(0.0, get_double(value, k));
    else if (k == "bear_tau_ticks")   executor_.bear_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "bear_on")          executor_.bear_on_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "bear_off")         executor_.bear_off_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "touch_ticks")      executor_.touch_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "touch_load")       executor_.touch_load_ = std::max(0.0, get_double(value, k));
    else if (k == "plan")             planner_.plan_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "plan_model_margin") planner_.plan_model_margin_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "plan_shift_m")     planner_.plan_shift_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plan_shift_fwd_m") planner_.plan_shift_fwd_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plan_land_m")      planner_.plan_land_m_ = std::max(0.0, get_double(value, k));
    else if (k == "plan_cell_m")      planner_.plan_cell_m_ = std::max(2e-3, get_double(value, k));
    else if (k == "plan_adv_w")       planner_.plan_adv_w_ = std::max(0.0, get_double(value, k));
    else if (k == "lo_gain")          offset_.gain_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "lo_learn")         offset_.learn_ = std::max(0.0, get_double(value, k));
    else if (k == "lo_max_m")         offset_.max_m_ = std::clamp(get_double(value, k), 1e-3, 0.06);
    else if (k == "lo_cell_m")        offset_.cell_m_ = std::max(2e-3, get_double(value, k));
    else if (k == "relief")           executor_.relief_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "relief_step_m")    planner_.relief_step_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "relief_cells")     planner_.relief_cells_ = std::clamp(int(get_double(value, k)), 0, 20);
    else if (k == "advance_reach_m")  executor_.advance_reach_m_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_load_min")   executor_.creep_load_min_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "creep_load_relax") executor_.creep_load_relax_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "creep_v_mps")      executor_.creep_v_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_max_m")      executor_.creep_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_relax_v_mps") executor_.creep_relax_v_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_deadband_m")  executor_.elev_deadband_m_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_tau_ticks")   executor_.elev_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "air_ticks_min")    executor_.air_ticks_min_ = std::max(1, int(get_double(value, k)));
    else if (k == "margin_extra_m")   floor_.margin_extra_m_ = std::max(0.0, get_double(value, k));
    else if (k == "reach_min_m")      kin_.p.reach_min_m = std::max(0.0, get_double(value, k));
    else if (k == "retreat_m")        executor_.retreat_m_ = std::max(0.0, get_double(value, k));   // 0 = no retreat (the prune's leave-one-out, 2026-11-14)
    // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): (re-seed param)
    else if (k == "ik_reseed_rate_rad") executor_.ik_reseed_rate_ = std::clamp(get_double(value, k), 1e-3, 0.5);
    else if (k == "hang_ticks")       executor_.hang_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "drag_back_m")      executor_.drag_back_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_back_m")      executor_.free_back_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_extra_m")     executor_.free_extra_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_retract_m")   executor_.free_retract_m_ = std::max(0.0, get_double(value, k));
    else if (k == "land_r_min_m")     planner_.land_r_min_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ext_margin_m")     planner_.ext_margin_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ext_lifting")      planner_.ext_lifting_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "overdue")          planner_.overdue_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "plan_next_first")  planner_.plan_next_first_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "ext_urgent_m")     planner_.ext_urgent_m_ = std::max(0.0, get_double(value, k));
    else if (k == "unloaded_shift_m") book_.unloaded_shift_m_ = std::max(0.0, get_double(value, k));
    else if (k == "unloaded_share")   book_.unloaded_share_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "unloaded_skips_min") book_.unloaded_skips_min_ = std::max(0, int(get_double(value, k)));
    else if (k == "unloaded_landings_min") book_.unloaded_landings_min_ = std::max(1, int(get_double(value, k)));
    else if (k == "unloaded_ticks")   book_.unloaded_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "clip_ticks")       executor_.clip_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "miss_reacquire")   executor_.miss_reacquire_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "null_step_m")      executor_.null_step_m_ = std::max(0.0, get_double(value, k));
    else if (k == "miss_ticks")       executor_.miss_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "front_drop_gain") executor_.front_drop_gain_ = std::max(0.0, get_double(value, k));
    else if (k == "front_drop_max_m") executor_.front_drop_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "front_drop_tau_ticks") executor_.front_drop_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "front_drop_deadband") executor_.front_drop_deadband_ = std::max(0.0, get_double(value, k));
    else if (k == "depth_max_m")      executor_.depth_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_form")        executor_.free_form_ = std::clamp(int(get_double(value, k)), 2, 3);
    else if (k == "free_margin_m")    executor_.free_margin_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_out_m")       executor_.free_out_m_ = std::max(0.0, get_double(value, k));
    else if (k == "free_noise_m")     executor_.free_noise_m_ = std::max(0.0, get_double(value, k));
    else if (k == "face_gap_m")       planner_.face_gap_m_ = std::max(0.0, get_double(value, k));
    else if (k == "face_penalty_m")   planner_.face_penalty_m_ = std::max(0.0, get_double(value, k));
    else if (k == "ik_multistart")    executor_.ik_multistart_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "ik_switch_m")      kin_.p.ik_switch_m = std::max(0.0, get_double(value, k));
    else if (k == "ik_knee_guard")    kin_.p.knee_guard = std::clamp(get_double(value, k), 0.0, 1.0);
    // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): (re-seed param)
    else if (k == "ik_reseed_ticks")  executor_.ik_reseed_ticks_ = std::max(0, int(get_double(value, k)));
    else if (k == "crouch_max_m")     executor_.crouch_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "crouch_v_mps")     executor_.crouch_v_ = std::max(0.0, get_double(value, k));
    else if (k == "crouch_relax_v_mps") executor_.crouch_relax_v_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_source")      { const std::string x = get_string(value, k); if (x == "patch" || x == "landed") executor_.elev_source_ = x; }
    else if (k == "elev_max_m")       executor_.elev_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "reach_max_m")      kin_.p.reach_max_m = std::max(0.03, get_double(value, k));
    else if (k == "unreach_tol_m")    executor_.unreach_tol_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "engage_min_belly_m") floor_.engage_min_belly_m_ = get_double(value, k);
    else if (k == "engage_min_tick")  floor_.engage_min_tick_ = std::max(0, int(get_double(value, k)));
    else if (k == "engage_min_contacts") floor_.engage_min_contacts_ = std::clamp(int(get_double(value, k)), 0, 4);
    else if (k == "phase_max_ticks")  executor_.phase_max_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "shift_v_mps")      executor_.shift_v_ = std::max(0.0, get_double(value, k));
    else if (k == "shift_max_m")      executor_.shift_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "margin_min_m")     floor_.margin_min_m_ = get_double(value, k);
    else if (k == "lift_clear_m")     executor_.lift_clear_m_ = std::max(0.0, get_double(value, k));
    else if (k == "lift_v_mps")       executor_.lift_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "swing_v_mps")      executor_.swing_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "lower_v_mps")      executor_.lower_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "lower_fast_v_mps") executor_.lower_fast_v_ = std::max(0.0, get_double(value, k));
    else if (k == "lower_standoff_m") executor_.lower_standoff_m_ = std::max(0.0, get_double(value, k));
    else if (k == "lower_standoff_k") executor_.lower_standoff_k_ = std::max(0.0, get_double(value, k));
    else if (k == "land_edge_m")      executor_.land_edge_m_ = std::max(0.0, get_double(value, k));
    else if (k == "land_edge_max_m")  executor_.land_edge_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "stride_m")         executor_.stride_m_ = std::max(0.0, get_double(value, k));
    else if (k == "sink_max_m")       executor_.sink_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "tilt_disengage")   floor_.tilt_disengage_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "dt_s")             executor_.dt_ = std::max(1e-3, get_double(value, k));
    else if (k == "ik_iters")         kin_.p.ik_iters = std::clamp(int(get_double(value, k)), 1, 32);
    else if (k == "ik_damping_m")     kin_.p.ik_damping_m = std::max(1e-4, get_double(value, k));
    else if (k == "ik_step_max_rad")  kin_.p.ik_step_max_rad = std::max(1e-3, get_double(value, k));
    else if (k == "ik_tol_m")         kin_.p.ik_tol_m = std::max(1e-4, get_double(value, k));
}

void PoseSequencer::handle(std::string const& which, int idx, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    if (which == "gait") {
        auto a = std::dynamic_pointer_cast<const ActionOut>(p);
        if (!a || idx < 0 || idx >= 12) return;
        in_.gait_u_[size_t(idx)] = a->accel; in_.have_gait_ = true; return;
    }
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if (which == "model" && v.size() >= kModelFloats) { kin_.set_model(v.data(), std::size_t(v.size())); }   // the leg model: body::LegSolver
    else if (which == "support" && v.size() >= 26) { for (int i = 0; i < 26; ++i) in_.support_[size_t(i)] = v[i]; in_.have_support_ = true; }
    else if (which == "patch" && v.size() >= 4) {
        const int N = int(v[0]); if (N < 2 || v.size() < 4 + 2 * N * N) return;
        in_.N_ = N; in_.cell_ = std::max(1e-3, double(v[1])); in_.patch_.assign(v.data(), v.data() + 4 + 2 * N * N); in_.have_patch_ = true;
    }
    else if (which == "xz" && v.size() >= 8) { for (int i = 0; i < 8; ++i) in_.xz_[size_t(i)] = v[i]; in_.have_xz_ = true; }
    else if (which == "y" && v.size() >= 4) { for (int i = 0; i < 4; ++i) in_.y_[size_t(i)] = v[i]; in_.have_y_ = true; }
    else if (which == "contact" && v.size() >= 4) { for (int i = 0; i < 4; ++i) in_.contact_[size_t(i)] = v[i]; in_.have_contact_ = true; }
    else if (which == "load" && v.size() >= 4) { for (int i = 0; i < 4; ++i) in_.load_[size_t(i)] = v[i]; in_.have_load_ = true; }
    else if (which == "belly" && v.size() >= 1) { in_.belly_ = v[0]; in_.have_belly_ = true; }
    else if (which == "odom" && v.size() >= 3) { in_.odom_in_ = {v[0], v[1], v[2]}; in_.have_odom_in_ = true; }
    else if (which == "imu" && v.size() >= 6) { for (int i = 0; i < 6; ++i) in_.imu_[size_t(i)] = v[i]; in_.have_imu_ = true; }
    else if (which == "up" && v.size() >= 3) { const double n = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]); if (n > 0.5) { in_.up_ = {v[0] / n, v[1] / n, v[2] / n}; in_.have_up_ = true; } }
}

void PoseSequencer::publish(uint64_t tick_id) {
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); out->sensor = "pose";
    out->values = Eigen::VectorXf::Zero(13);
    out->values[0] = engage_level_;
    for (int i = 0; i < 4; ++i) { const auto u = kin_.u_of_angles(i, executor_.angles_[size_t(i)]); for (int j = 0; j < 3; ++j) out->values[1 + 3 * i + j] = float(u[size_t(j)]); }
    bus_->publish(output_topic_, out);
}

void PoseSequencer::tick(uint64_t tick_id) {
    executor_.sense(tick_id);   // the tick and the leaky bearing estimate
    // the FK check against the body's own toes (the model is the body's; this is the wiring's check), from the gait's command
    if (kin_.ready() && in_.have_gait_ && in_.have_xz_ && in_.have_y_ && executor_.phase_ == Phase::Idle) {
        double e = 0.0;
        for (int i = 0; i < 4; ++i) {
            const Vec3f toe = kin_.toe_of(i, kin_.angles_of_u(i, {in_.gait_u_[size_t(3 * i)], in_.gait_u_[size_t(3 * i + 1)], in_.gait_u_[size_t(3 * i + 2)]}));
            e += std::hypot(double(toe.x) - in_.xz_[size_t(2 * i)], double(toe.z) - in_.xz_[size_t(2 * i + 1)]) / 4.0;
        }
        fk_err_ema_ += 0.02 * (e - fk_err_ema_);
    }
    const bool upright = floor_.upright(in_);
    const bool want = engage_ > 0.0 && kin_.ready() && upright;
    if (!upright && executor_.phase_ != Phase::Idle) ++disengaged_tilt_;
    if (want && executor_.phase_ == Phase::Idle) {   // from a standing body only
        int n_contact = 0; if (in_.have_contact_) for (int i = 0; i < 4; ++i) n_contact += executor_.bearing(i);
        const bool standing = floor_.standing(int64_t(tick_id), in_, n_contact, !belly_topic_.empty());
        if (standing) { executor_.engage_now(tick_id); planner_.pred_leg_ = -1; }
    }
    if (!want && executor_.phase_ != Phase::Idle) { executor_.phase_ = Phase::Idle; executor_.leg_ = -1; }
    const float ramp = 1.0f / float(engage_ramp_ticks_);
    engage_level_ = std::clamp(engage_level_ + (executor_.phase_ != Phase::Idle ? ramp : -ramp), 0.0f, 1.0f);
    if (executor_.phase_ != Phase::Idle) {
        ++ticks_engaged_;
        executor_.step_machine();
        executor_.hold_stance();
        executor_.solve_ik();
        if (offset_.on()) offset_.absorb(executor_);   // a step that landed this tick is learned from
    }
    if (!odom_out_topic_.empty()) {   // the odometry: the sequencer's own while engaged, the body's passed through otherwise
        if (executor_.phase_ == Phase::Idle || !odom_anchored_) { if (in_.have_odom_in_) { executor_.px_ = in_.odom_in_[0]; executor_.py_ = in_.odom_in_[1]; } odom_anchored_ = in_.have_odom_in_; }
        auto od = std::make_shared<ProprioToken>();
        od->tick_id = tick_id; od->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); od->sensor = "odom_seq";
        od->values = Eigen::VectorXf(3); od->values[0] = float(executor_.px_); od->values[1] = float(executor_.py_); od->values[2] = float(in_.have_odom_in_ ? in_.odom_in_[2] : 0.0);
        bus_->publish(odom_out_topic_, od);
    }
    if (!outcome_topic_.empty() && executor_.outcome_seq_ != outcome_published_) {   // a step has landed since the last tick: its outcome
        outcome_published_ = executor_.outcome_seq_;
        auto oc = std::make_shared<ProprioToken>(); const auto f = executor_.outcome_.floats();
        oc->tick_id = tick_id; oc->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); oc->sensor = "step_outcome";
        oc->values = Eigen::VectorXf(int(f.size())); for (int i = 0; i < int(f.size()); ++i) oc->values[i] = f[size_t(i)];
        bus_->publish(outcome_topic_, oc);
    }
    if (engage_level_ > 0.0f || executor_.phase_ != Phase::Idle || engage_ > 0.0) publish(tick_id);
    else if (!output_topic_.empty()) {   // engage 0: the socket sees 0 and the gait is untouched
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); out->sensor = "pose";
        out->values = Eigen::VectorXf::Zero(13); bus_->publish(output_topic_, out);
    }
}

nlohmann::json PoseSequencer::diag_snapshot() const {
    nlohmann::json room = nlohmann::json::array();   // Phase 2's room measure: per leg [ext slack (m), knee to straight, knee to fold, hip1 to its stop, hip2 to its stop]
    for (int i = 0; i < 4; ++i) { const auto r = kin_.room(i, executor_.angles_[size_t(i)]); room.push_back({r.ext_m, r.knee_straight, r.knee_fold, r.hip1, r.hip2}); }
    nlohmann::json tg = nlohmann::json::array(), st = nlohmann::json::array(), gh = nlohmann::json::array(), nt = nlohmann::json::array(), sc = nlohmann::json::array(), uu = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) { const auto u = kin_.u_of_angles(i, executor_.angles_[size_t(i)]); uu.push_back({u[0], u[1], u[2]}); }
    for (int i = 0; i < 4; ++i) { tg.push_back({executor_.toe_tgt_[size_t(i)].x, executor_.toe_tgt_[size_t(i)].y, executor_.toe_tgt_[size_t(i)].z}); st.push_back(executor_.steps_[size_t(i)]); gh.push_back(executor_.ground_h_[size_t(i)]);
                                  nt.push_back({executor_.neutral_[size_t(i)].x, executor_.neutral_[size_t(i)].z}); sc.push_back(planner_.score_[size_t(i)]); }
    nlohmann::json out = nlohmann::json{{"phase", phase_name(executor_.phase_)}, {"phase_id", int(executor_.phase_)}, {"leg", executor_.leg_}, {"engage", engage_level_}, {"model", kin_.ready()}, {"support", in_.have_support_},
                          {"patch", in_.have_patch_}, {"steps", st}, {"cycles", executor_.cycles_}, {"shift_exhausted", executor_.shift_exhausted_}, {"shift_ticks", executor_.shift_ticks_},
                          {"disengaged_tilt", disengaged_tilt_}, {"lower_sunk", executor_.lower_sunk_}, {"phase_timeouts", executor_.phase_timeouts_}, {"engaged_at", executor_.engaged_at_},
                          {"landings", executor_.landings_}, {"missed", executor_.missed_}, {"land_h_rejected", executor_.land_h_rejected_}, {"fast_ticks", executor_.fast_ticks_}, {"slow_ticks", executor_.slow_ticks_}, {"edge_moves", executor_.edge_moves_}, {"edge_stuck", executor_.edge_stuck_}, {"land_sigma", executor_.land_sigma_}, {"blocked_leg", executor_.blocked_leg_}, {"advance_yields", executor_.advance_yields_}, {"relief_steps", executor_.relief_steps_}, {"plans", planner_.plans_}, {"lower_fold_ticks", executor_.lower_fold_ticks_}, {"crouch", executor_.crouch_}, {"reseeds", executor_.reseeds_}, {"multistarts", kin_.multistarts}, {"face_penalties", planner_.face_penalties_}, {"front_drop", executor_.front_drop_}, {"clips", executor_.clips_}, {"unload_shifts", planner_.unload_shifts_}, {"land_r_rejects", planner_.land_r_rejects_}, {"ext_rejects", planner_.ext_rejects_}, {"ext_lift_rejects", planner_.ext_lift_rejects_}, {"overdue", {planner_.overdue_decisions_, planner_.overdue_rejects_, planner_.urgent_jumps_, planner_.urgent_fails_}}, {"skip_nomargin", planner_.skip_nomargin_}, {"skip_noland", planner_.skip_noland_}, {"skip_miss", planner_.skip_miss_}, {"block", planner_.block_}, {"next_lift", planner_.next_lift_}, {"drift", planner_.drift_}, {"drift_cause", planner_.drift_cause_}, {"drift_mag", planner_.drift_mag_}, {"drift_mag_all", planner_.drift_mag_all_}, {"reloc_n", executor_.reloc_n_}, {"reloc_sum", executor_.reloc_sum_}, {"reloc_top", executor_.reloc_top_}, {"reloc_off", executor_.reloc_off_}, {"reloc_resid", executor_.reloc_resid_}, {"block_d", planner_.block_d_}, {"block_short", planner_.block_short_}, {"ext", {kin_.ext_slack(0, executor_.toe_tgt_[0]), kin_.ext_slack(1, executor_.toe_tgt_[1]), kin_.ext_slack(2, executor_.toe_tgt_[2]), kin_.ext_slack(3, executor_.toe_tgt_[3])}}, {"ext_fk", {ext_slack_fk(0), ext_slack_fk(1), ext_slack_fk(2), ext_slack_fk(3)}}, {"null_steps", executor_.null_steps_}, {"miss_skips", planner_.miss_skips_}, {"miss_count", {executor_.miss_count_[0], executor_.miss_count_[1], executor_.miss_count_[2], executor_.miss_count_[3]}}, {"depth_caps", executor_.depth_caps_}, {"free_out", {executor_.free_out_dx_, executor_.free_out_dy_}}, {"branch_switches", kin_.branch_switches}, {"hangs", executor_.hangs_}, {"hangs_by_contact", executor_.hangs_by_contact_}, {"hangs_by_drag", executor_.hangs_by_drag_}, {"swing_d_fwd", executor_.swing_d_fwd_}, {"a_lin_z_ema", executor_.a_lin_z_ema_}, {"crouch_ticks", executor_.crouch_ticks_}, {"elev_landed", {executor_.elev_landed_[0], executor_.elev_landed_[1], executor_.elev_landed_[2], executor_.elev_landed_[3]}}, {"plan_skips", planner_.plan_skips_}, {"plan_none", planner_.plan_none_}, {"plan_reached", executor_.plan_reached_}, {"plan_shift", {executor_.plan_sx_, executor_.plan_sz_}}, {"plan_score", executor_.plan_score_}, {"relief_none", executor_.relief_none_}, {"relief_for", executor_.relief_for_}, {"creep_load_ticks", executor_.creep_load_ticks_}, {"creep", {executor_.creep_[0], executor_.creep_[1], executor_.creep_[2], executor_.creep_[3]}}, {"settle_ticks", executor_.settle_ticks_}, {"obstacle_hits", executor_.obstacle_hits_}, {"resets_seen", resets_seen_}, {"odom", {executor_.px_, executor_.py_}}, {"yaw_err", executor_.yaw_err_}, {"shift_blocked", executor_.shift_blocked_}, {"retreats", executor_.retreats_}, {"shift_d", {executor_.shift_dx_, executor_.shift_dz_}}, {"steps_no_advance", executor_.steps_no_advance_}, {"deadlock_ticks", executor_.deadlock_ticks_}, {"turns_passed", executor_.turns_passed_}, {"creep", {executor_.creep_[0], executor_.creep_[1], executor_.creep_[2], executor_.creep_[3]}}, {"bear", {executor_.bear_[0], executor_.bear_[1], executor_.bear_[2], executor_.bear_[3]}}, {"land_err", executor_.land_err_last_}, {"land_rms", std::sqrt(executor_.land_err_ms_)}, {"elev", {executor_.elev_[0], executor_.elev_[1], executor_.elev_[2], executor_.elev_[3]}}, {"ik_err", executor_.ik_err_ema_}, {"fk_err", fk_err_ema_}, {"ticks_engaged", ticks_engaged_},
                          {"toe_tgt", tg}, {"u", uu}, {"ground_h", gh}, {"neutral", nt}, {"score", sc}, {"air_ticks", {executor_.air_ticks_[0], executor_.air_ticks_[1], executor_.air_ticks_[2], executor_.air_ticks_[3]}}, {"lift_h", executor_.lift_h_}, {"land_h", executor_.land_h_}, {"shift_acc", executor_.shift_acc_}, {"phase_ticks", executor_.phase_ticks_}, {"room", room}, {"outcome", executor_.outcome_.floats()}};
    if (planner_.plan_model_margin_ > 0.0) out["plan_model_margin"] = {double(planner_.model_bias_n_), planner_.model_bias_sum_};   // only when on: [decisions with the model's margin > 1 mm below the geometric one, the summed shortfall m]
    if (kin_.p.knee_guard > 0.0) out["knee_guard"] = {kin_.knee_retries, kin_.knee_unsolved};   // only when the guard is on (the default diag is unchanged)
    if (offset_.on()) { const nlohmann::json lo = offset_.diag(); for (auto const& [k, v] : lo.items()) out["lo_" + k] = v; }   // only when the layer is on (the default diag is unchanged); flat keys, because the host passes scalars and arrays to the body and drops a nested object
    return out;
}

nlohmann::json PoseSequencer::snapshot_state() const {
    if (!offset_.on()) return Module::snapshot_state();
    return nlohmann::json{{"version", 1}, {"lo", offset_.snapshot()}};
}
void PoseSequencer::restore_state(nlohmann::json const& s) {
    if (offset_.on() && s.is_object() && s.contains("lo")) offset_.restore(s.at("lo"));
}

} // namespace ogma
