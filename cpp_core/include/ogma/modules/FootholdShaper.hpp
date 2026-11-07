#pragma once
// FootholdShaper -- the careful stepper's first policy (2026-11-07): from TerrainMemory's per-leg prediction of what the
// next landing will meet, a per-leg swing target for MotorEPMv2's swing_target_topic: [lift_mult, reach_m] x 4.
//
//   lift: a predicted RISE from where THIS leg's toe last stood (h_pred minus its height at its last touchdown) lifts
//         that leg's swing by lift_gain per metre of rise (1 + gain x rise), capped -- a step is cleared where one is
//         predicted and nowhere else (the uniform lift was NULL at both servo speeds).  The first form measured the rise
//         from the planted toes' median and lifted the FRONT legs, which never hang, while a rear toe a step below the
//         body saw no rise at all (2026-11-07 battery: a REGRESSION at 38 / 50 mm);
//   reach: a landing whose prediction SPREADS more than edge_min_m (the pooled heights span a riser) AND rises (an
//         up-edge) is moved edge_shift_m forward, onto the tread -- where the foot is put down;
//   both ramp by the prediction's precision (prec_full: full effect at or above it) so a thin memory asks for little.
// Gains 0 = [1, 0] x 4 published, the gait unchanged; the socket absent from MotorEPMv2 = byte-identical.  Nothing is
// learned here yet; the outcome the policy is judged on is the foothold prediction error and the risers climbed.
#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class FootholdShaper : public Module {
public:
    FootholdShaper();
    ~FootholdShaper() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;
    // ---- for tests ----
    float lift_mult(int leg) const { return out_[size_t(2 * leg)]; }
    float reach_m(int leg) const { return out_[size_t(2 * leg + 1)]; }

private:
    void handle(std::string const& which, MessagePtr p);
    std::string predict_topic_ = "predict.foothold", contact_topic_ = "reality.proprio.foot_contact",
                feet_y_topic_ = "reality.proprio.toe_y", output_topic_ = "swing.targets";
    double lift_gain_ = 0.0, lift_max_ = 2.5, rise_min_m_ = 0.01;
    double edge_min_m_ = 0.02, edge_shift_m_ = 0.0, prec_full_ = 2000.0;
    std::array<float, 24> pred_{}; bool have_pred_ = false;
    std::array<float, 4> contact_{}, feet_y_{}; bool have_contact_ = false, have_y_ = false;
    std::array<float, 8> out_{1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f};
    std::array<float, 4> rise_{}; double ground_ = 0.0; bool have_ground_ = false;
    std::array<float, 4> touch_y_{}, prev_contact_{}; std::array<bool, 4> have_touch_{};   // each toe's height at its last touchdown
    int64_t lifts_ = 0, shifts_ = 0;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
