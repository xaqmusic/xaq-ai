#include "Recovery.hpp"

namespace mjhost {

const char* driver_name(Driver d) { return d == Driver::Brain ? "brain" : "scaffold"; }

Driver Recovery::update(const std::array<double, 3>& gravity, double dt) {
    handed_off_ = false;
    handed_back_ = false;
    const double gz = gravity[2];

    if (driver_ == Driver::Brain) {
        brain_s_ += dt;

        // Debounced in both directions, as duck-control's Safety does it: one
        // upright sample clears the accumulator. Without that, a single noisy
        // reading mid-stride would hand the body away.
        if (gz > c_.fallen_gravity_z) {
            down_for_ += dt;
        } else {
            down_for_ = 0.0;
        }

        if (down_for_ >= c_.fallen_debounce_s) {
            driver_ = Driver::Scaffold;
            handed_off_ = true;
            down_for_ = 0.0;
            up_for_ = 0.0;
            recovering_for_ = 0.0;
            ++rescues_;
        }
    } else {
        scaffold_s_ += dt;
        recovering_for_ += dt;

        // Near upright AND staying there. A body tumbling through upright on its
        // way to the other side would otherwise be handed back mid-fall.
        if (gz < c_.upright_gravity_z) {
            up_for_ += dt;
        } else {
            up_for_ = 0.0;
        }

        const bool recovered = up_for_ >= c_.upright_hold_s;
        const bool exhausted = recovering_for_ >= c_.give_up_s;

        if (recovered || exhausted) {
            if (recovering_for_ > longest_recovery_) longest_recovery_ = recovering_for_;
            if (exhausted && !recovered) ++gave_up_;
            driver_ = Driver::Brain;
            handed_back_ = true;
            up_for_ = 0.0;
            // The debounce starts clean: a brain handed a body that is still
            // marginal should get a fair 200 ms before being taken off it again.
            down_for_ = 0.0;
        }
    }
    return driver_;
}

}  // namespace mjhost
