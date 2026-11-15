#pragma once
// SupportFloor -- the stepper's reflex floor: a leg lifts only with a support margin; the body hands the legs back to the rhythm
// when it tilts; the sequencer engages only from a standing body.  Plain checks, no state; the margins are read by the executor
// and the planner alike.
// Split out of PoseSequencer byte-identical (learned stepper plan Phase 2, 2026-11-15): every function body is the
// monolith's, moved, with its members qualified by owner; the golden traces (scripts_tools/stepper_battery.sh +
// stepper_ident.py) are the gate.  The member comments are the monolith's, kept with the member they document.
#include <cstdint>
#include <string>
#include "ogma/modules/stepper/StepperCommon.hpp"

namespace ogma::stepper {

struct SupportFloor {
    double engage_min_belly_m_ = 0.045;
    int engage_min_tick_ = 200; int engage_min_contacts_ = 3;
    double margin_extra_m_ = 0.005;
    double margin_min_m_ = 0.005;
    double tilt_disengage_ = 0.75;
    // may the leg go?  By the local geometry; the model vetoes only when all four toes bear
    bool lift_safe(double m_local, double model_margin, int bearing) const { return m_local >= margin_min_m_ && (bearing < 4 || model_margin >= margin_min_m_); }
    // the body upright enough to keep the legs
    bool upright(const StepperInputs& in) const { return !in.have_up_ || in.up_[1] >= tilt_disengage_; }
    // a standing body: past the gait's warm-up, enough toes bearing, the belly clear (when a belly topic is given)
    bool standing(int64_t tick_id, const StepperInputs& in, int n_contact, bool belly_check) const {
        return tick_id >= engage_min_tick_ && (!in.have_contact_ || n_contact >= engage_min_contacts_)
               && (!belly_check || (in.have_belly_ && in.belly_ >= float(engage_min_belly_m_)));
    }
};

} // namespace ogma::stepper
