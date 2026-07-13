#pragma once

// =============================================================================
// BearingEstimator.hpp  --  perception-as-inference for the food bearing (Stage 3)
// =============================================================================
//
// De-scaffolds the hand-coded ScentCompass vector-sum: instead of an analytic
// reduction of the 8-nostril ring → [cx,cy], LEARN the percept. An EPM-style
// online vector-quantizer clusters the raw nostril ring into prototypes (learned
// categorical percepts); each prototype carries a bearing readout [cx,cy] that is
// DISTILLED (EMA) from the analytic compass acting as a TEACHER. The inferred
// bearing = the winning prototype's readout; the reconstruction distance is the
// inference error (TLE).
//
// The de-scaffold is the LESION: after lesion_after_ticks the readouts freeze and
// the teacher is no longer read — foraging then runs on the LEARNED readouts ALONE.
//
// FALSIFIED (2026-06-24): this distillation is NOT a genuine de-scaffold. It only
// MIMICS the teacher; with the compass removed the frozen VQ readouts don't
// generalize to the rings the bug then visits → the inferred bearing collapses
// (|b| 0.98 → 0.22) → foraging dies (13 → 0 hits). A teacher-distilled copy dies
// with its source — it never learned anything real.
//
// The genuine de-scaffold is a learned ring→bearing from ACTION-CONSEQUENCE
// (doctrine §4): "I moved heading H and my own scent/energy changed by ΔX" → the
// bearing is the heading that raises scent. That is IN-RUNTIME (an online value
// table, like HeadingController's learned advance table) — NO backprop / ONNX. It
// needs the EPM clustering a CONDITIONED (centred + normalized) ring (§5 EPM-first,
// §6 condition-the-input), credited off Δscent / the hit (§4), NOT a teacher.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class BearingEstimator : public Module {
public:
    BearingEstimator();
    ~BearingEstimator() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors (diag).
    int   n_prototypes() const { return int(protos_.size()); }
    int   last_winner()  const { return last_winner_; }
    float last_tle()     const { return last_tle_; }       // ring reconstruction error
    float inferred_cx()  const { return inf_cx_; }
    float inferred_cy()  const { return inf_cy_; }
    bool  lesioned()     const { return lesioned_; }

private:
    void handle_ring(MessagePtr payload);
    void handle_teacher(MessagePtr payload);

    std::string ring_topic_    = "reality.proprio.scent";     // raw 8-nostril ring
    std::string teacher_topic_ = "percept.scent_compass";     // analytic [cx,cy,(prox)] — distillation target
    std::string output_topic_  = "percept.bearing_inferred";  // → MotivationGate / HeadingController
    int         max_prototypes_ = 24;
    float       novelty_thresh_ = 0.05f;   // L2 distance above which a new prototype is grown
    float       proto_lr_       = 0.05f;   // VQ update rate (winner → ring)
    float       bearing_lr_     = 0.05f;   // readout EMA toward the teacher bearing
    int         lesion_after_ticks_ = -1;  // ≥0 → freeze readouts + stop reading teacher after N ticks
    bool        force_lesion_   = false;   // start lesioned (no teacher ever — the hard ablation)

    std::vector<std::vector<float>> protos_;   // [n][ring_dim] learned ring prototypes
    std::vector<float> proto_cx_;              // [n] per-prototype bearing readout
    std::vector<float> proto_cy_;
    std::vector<float> ring_;                  // latest raw ring
    bool   have_ring_ = false;
    float  teach_cx_ = 0.0f, teach_cy_ = 0.0f, teach_prox_ = 0.0f;
    bool   have_teacher_ = false;
    int    tick_count_ = 0;

    bool  lesioned_     = false;
    int   last_winner_  = -1;
    float last_tle_     = 0.0f;
    float inf_cx_       = 0.0f;
    float inf_cy_       = 0.0f;
};

} // namespace ogma
