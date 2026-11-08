#pragma once
// SupportModel -- the pose EVALUATOR of the careful stepper (2026-11-08): from where the toes are (the commanded FK) and
// what the terrain memory says is under them, predict what the body's own sensors will report -- which toes bear, how
// much each carries (the FSRs), and how far the body's weight sits inside the toes that hold it -- and, for each
// planted toe, whether the body could still stand if that toe left (the horizon-one question a step-up must answer
// before a rear foot lifts).
//
// WHY A MODEL AND NOT A REFLEX.  The backward flip on the staircase (ledger 2026-11-07) was four per-leg decisions
// each right on its own -- lift the hung rear toe higher, press the low front corner up -- and wrong together: with
// the body nose-up on a riser its weight was already behind the rear toes, and no leg's own sensor said so.  Only a
// model of the whole support -- all the toes and the centre of mass in one gravity-levelled frame -- carries that fact,
// so that is the prediction this module makes, and the FSRs and IMU grade it every tick.
//
// THE FRAME.  The body's gravity-up (up_body) levels everything: fwd and lat are horizontal, h is height along gravity
// with the body origin at 0.  A toe at body-frame (x, z) with gravity height toe_y is completed to a body point (its
// body y from the up vector) and levelled -- a toe 8 cm below a body pitched nose-up 20 deg sits 2.7 cm further FORWARD
// of the body's weight than its body-frame z says, which is the whole flip.
//
// THE PREDICTIONS (support.state, [margin, n_support, com_fwd, com_lat, gap_slope_fwd, gap_slope_lat] then per leg
// [gap, has, contact, load, lift_margin]):
//   gap_i      toe height minus the remembered ground under it (m; + = in the air); has_i = ground remembered there;
//   contact_i  gap_i <= gap_tol_m: the toe is predicted to bear;
//   load_i     the static share of the body's weight on toe i (fraction, sums to 1): the loads on the bearing toes that
//              balance the centre of mass (barycentric on three; the minimum-norm solution on four, a toe that would
//              pull is dropped as unloaded);
//   margin     the signed distance (m) of the centre of mass's projection inside the bearing toes' hull (+ inside);
//   lift_margin_i  the margin the OTHER bearing toes would give if toe i left now (the step's safety);
//   gap_slope  the plane through the bearing toes' gaps: how the body will rotate as they close (fwd slope: + = the
//              front toes hang higher, the nose will drop).
// The centre of mass: the chassis at com_offset plus each leg's mass at leg_com_frac of the way from its hip to its toe.
//
// THE ERRORS, every tick: load_i against the published foot load (reality.proprio.foot_load) on the toes that are in
// contact, as a running RMS and correlation; contact_i against the contact flag (agreement rate).  Nothing acts on the
// state yet: the module is admitted by those errors on the staircase, and by the lift margin's sign before the
// reproduced flip.  Absent = byte-identical.
#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class SupportModel : public Module {
public:
    SupportModel();
    ~SupportModel() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    struct Point { double fwd = 0.0, lat = 0.0; };
    struct Eval {   // one pose's evaluation
        double margin = 0.0, com_fwd = 0.0, com_lat = 0.0, slope_fwd = 0.0, slope_lat = 0.0; int n_support = 0;
        std::array<double, 4> gap{}, load{}, lift_margin{}; std::array<bool, 4> has{}, contact{}, gap_contact{};   // contact: the bearing set; gap_contact: the memory's prediction of it
    };
    // ---- the model, exposed for tests and for a searcher (pure functions of the inputs) ----
    // the support polygon's signed margin around c for the given points (<= 4); a segment or a point is negative
    static double margin_of(std::vector<Point> const& pts, Point c);
    // the static load shares on the points that balance c (sum 1; a point that would pull is dropped to 0)
    static std::vector<double> loads_of(std::vector<Point> const& pts, Point c);
    Eval const& last() const { return eval_; }
    double load_rms(int leg) const { return std::sqrt(load_ms_[size_t(leg)]); }
    double contact_agreement() const { return agree_n_ > 0 ? double(agree_hits_) / double(agree_n_) : 0.0; }

private:
    void handle(std::string const& which, MessagePtr p);
    bool patch_at(double fwd, double lat, double& h, double& prec, double& spread) const;   // pooled over the 3x3 cells
    void evaluate();

    std::string feet_xz_topic_ = "reality.proprio.feet_xz", feet_y_topic_ = "reality.proprio.toe_y", up_topic_ = "reality.proprio.up_body",
                contact_topic_ = "reality.proprio.foot_contact", load_topic_ = "reality.proprio.foot_load",
                patch_topic_ = "terrain.patch", output_topic_ = "support.state";
    double mass_chassis_ = 0.3, mass_leg_ = 0.075, leg_com_frac_ = 0.48;
    std::array<double, 3> com_offset_{0.0, 0.0, 0.0};          // the chassis's centre of mass in the body frame [x, y, z]
    double hip_lat_m_ = 0.0383, hip_fwd_m_ = 0.033;            // the hip rectangle's half spans; legs 0 and 2 sit at -x in the toes' frame (the sim's read-back, 2026-11-08), 1 and 3 at +x
    double gap_tol_m_ = 0.015, margin_floor_ = -0.10, err_alpha_ = 0.02, up_min_ = 0.5;
    // WHICH TOES BEAR, for the pose the body is in: "flag" (default) -- the contact flag, the FSR being the sensor of
    // bearing (the memory's gap cannot resolve the few millimetres a swing toe clears by, battery 2026-11-08: a 1.5 cm
    // tolerance caught 86 % of the bearing toes and 36 % of the airborne); "gap" -- the memory's prediction, as a
    // candidate pose would have to be judged.  The gap prediction is graded against the flag either way.
    std::string support_source_ = "flag";
    // THE FLAG'S OFF-DEBOUNCE (operator, 2026-11-08: lifting one leg unloads the others for a few ticks and the raw flag
    // drops; a bearing set read from it collapses to two toes and every margin to the floor for the duration): a toe
    // that bore stays in the bearing set for contact_off_ticks after its flag drops. 0 = the raw flag (byte-identical).
    int contact_off_ticks_ = 0; std::array<int, 4> off_count_{}; std::array<bool, 4> bearing_{};
    double unsafe_margin_m_ = -0.02;   // a toe that LEAVES while its lift margin is below this is an unsafe lift

    // inputs
    std::array<float, 8> xz_{}; std::array<float, 4> y_{}, contact_{}, load_{}; std::array<double, 3> up_{0.0, 1.0, 0.0};
    bool have_xz_ = false, have_y_ = false, have_up_ = false, have_contact_ = false, have_load_ = false;
    std::vector<float> patch_; bool have_patch_ = false; int N_ = 0; double cell_ = 0.03;

    // the evaluation and its errors
    Eval eval_; bool have_eval_ = false;
    std::array<double, 4> load_err_{}, load_ms_{}; std::array<bool, 4> load_err_valid_{};
    double load_corr_sxy_ = 0, load_corr_sxx_ = 0, load_corr_syy_ = 0, load_corr_sx_ = 0, load_corr_sy_ = 0; double load_corr_n_ = 0;
    int64_t agree_hits_ = 0, agree_n_ = 0, evals_ = 0, no_ground_ = 0;
    // the support over time: the fraction of ticks without static support (margin < 0), the current and longest such
    // streak, and the unsafe lifts per leg (the flag falling while that toe's lift margin was below unsafe_margin_m)
    double unsup_frac_ = 0.0; int64_t unsup_streak_ = 0, unsup_streak_max_ = 0; std::array<int64_t, 4> unsafe_lifts_{}; std::array<int64_t, 4> lifts_{};
    std::array<float, 4> prev_contact_{}; std::array<double, 4> prev_lift_margin_{}; bool have_prev_ = false;
    // the gap's statistics by the contact flag (the tolerance's calibration): EMA mean and mean square, toes with ground
    double gap_on_mean_ = 0, gap_on_ms_ = 0, gap_off_mean_ = 0, gap_off_ms_ = 0; int64_t gap_on_n_ = 0, gap_off_n_ = 0;
    // the load prediction against the FSR SHARE (each toe's load over the four's sum, when they carry > share_min of the weight)
    double share_sxy_ = 0, share_sxx_ = 0, share_syy_ = 0, share_sx_ = 0, share_sy_ = 0, share_n_ = 0; double share_min_ = 0.3;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
