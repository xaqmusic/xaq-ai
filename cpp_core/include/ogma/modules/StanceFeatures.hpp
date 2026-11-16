#pragma once
// =============================================================================
// StanceFeatures -- the careful stepper's stance, as one conditioned vector for a stance EPM (learned stepper plan Phase 3,
// 2026-11-15).  An INSTRUMENT: nothing acts on it; the stance EPM, the BeliefFilter and (Phase 4) the step generator read it.
// =============================================================================
//
// WHAT IT SEES (egocentric only; no feet_y, no ground truth): the stepper's COMMANDED pose (pose.targets: engage, then the
// twelve joints in action units), turned into each leg's kinematic terms by the same body::LegSolver the stepper uses; the
// FSR load shares and contact flags; the body's up vector; and TerrainMemory's patch around each toe.  Per leg l:
//   room (5)          extension slack to a straight leg (m), knee to straight, knee to fold, hip1 and hip2 to their stops
//   lift (1)          the commanded toe's height over the mean of the other three (m): a stepping toe stands out
//   load, contact (2) the FSR share and the raw flag
//   ground, relief (2) the remembered ground under the toe against the mean under the four (m), and the 3 x 3 cells' relief
// then the body's pitch and roll (rad): 4 x 10 + 2 = 42 values, published only while the stepper is engaged.
//
// CONDITIONING (CLAUDE.md §0 rule 2 and §5.5 -- adapt, never tune to a scale): every value is centred and scaled by its own
// running mean and variance, the rate count-annealed (1 / n down to 1 / n_max); z is clipped to +-clip.  A floor of n_max
// samples is a memory of n_max samples, NOT a settling: across carried runs (about 8 750 samples a run) the default 20 000 kept
// chasing the rotating bench and slid the input about one quantisation cell a run, so the stance EPM re-grew nodes for stances
// it already held (ledger, 2026-11-15 "Phase 3, continued").  n_max = 0 removes the floor: a pure 1 / n over the module's
// lifetime, which settles and holds.  Nothing is published for the first `warmup` samples.  Two topics:
//   output_topic   z plus one constant component sqrt(dim) -- the EPM input.  The jl_state encoder L2-normalises its input, which
//                  keeps only a vector's direction; the constant keeps its magnitude as an angle (a typical unit-variance vector
//                  sits at 45 degrees to it).  A geometric choice from the width, not a tuned scale.
//   obs_topic      z alone -- the BeliefFilter's observation.
//
// THE FLICKER TEST (flicker_p, default 0 = off, byte-identical): each contact flag is flipped with this probability per tick
// before it is featurised -- the injected FSR flicker of the Phase 3 gate (the belief must coast through it).
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"
#include "ogma/body/LegSolver.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"

namespace ogma {

class StanceFeatures : public Module {
public:
    static constexpr int kRaw = 4 * 10 + 2;
    StanceFeatures();
    ~StanceFeatures() override;
    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;
    nlohmann::json snapshot_state() const override;
    void restore_state(nlohmann::json const& s) override;

    // ---- for tests ----
    bool raw(Eigen::VectorXd& out) const;     // the raw feature vector from the current inputs; false when not ready
    const Eigen::VectorXd& mean() const { return mean_; }
    const Eigen::VectorXd& var()  const { return var_; }
    int64_t samples() const { return n_; }
    int64_t published() const { return published_; }

private:
    void handle(std::string const& which, MessagePtr p);
    // config
    std::string model_topic_ = "reality.proprio.leg_model", pose_topic_ = "pose.targets", load_topic_ = "reality.proprio.foot_load",
                contact_topic_ = "reality.proprio.foot_contact", up_topic_ = "reality.proprio.up_body", patch_topic_ = "terrain.patch",
                output_topic_ = "reality.proprio.stance_features", obs_topic_ = "reality.proprio.stance_obs";
    double engage_min_ = 0.99, clip_ = 4.0, flicker_p_ = 0.0;
    int n_max_ = 20000, warmup_ = 200;
    int64_t freeze_after_ = 0;   // > 0: the statistics stop updating after this many samples (the encoder held still); 0 = off
    uint32_t flicker_seed_ = 1u, rng_ = 1u;
    // inputs
    body::LegSolver kin_; stepper::StepperInputs patch_;
    std::array<float, 13> pose_{}; bool have_pose_ = false;
    std::array<float, 4> load_{}, contact_{}; bool have_load_ = false, have_contact_ = false;
    std::array<bool, 4> flip_{};   // this tick's injected flicker (the flicker test), applied where the flags are featurised
    std::array<double, 3> up_{0.0, 1.0, 0.0}; bool have_up_ = false;
    // the running statistics
    Eigen::VectorXd mean_ = Eigen::VectorXd::Zero(kRaw), var_ = Eigen::VectorXd::Ones(kRaw);
    int64_t n_ = 0, published_ = 0, flips_ = 0;
    Eigen::VectorXd last_z_ = Eigen::VectorXd::Zero(kRaw);
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
