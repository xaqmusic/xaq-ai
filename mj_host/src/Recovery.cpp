#include "Recovery.hpp"

#include <algorithm>
#include <cmath>

namespace mjhost {

const char* driver_name(Driver d) { return d == Driver::Brain ? "brain" : "scaffold"; }

Driver Recovery::update(const std::array<double, 3>& gravity,
                        const std::array<double, 3>& gyro, double dt) {
    handed_off_ = false;
    handed_back_ = false;
    const double gz = gravity[2];
    const double wmax = std::max({std::abs(gyro[0]), std::abs(gyro[1]), std::abs(gyro[2])});

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

        // The stuck-pose clock: not fallen, but not upright either, for a long
        // time.  See the config note — a stable sit is a prison, not a success.
        if (c_.stuck_s > 0.0 && gz > c_.stuck_gravity_z) {
            stuck_for_ += dt;
        } else {
            stuck_for_ = 0.0;
        }
        const bool stuck = c_.stuck_s > 0.0 && stuck_for_ >= c_.stuck_s;
        if (stuck) ++stuck_rescues_;

        if (down_for_ >= c_.fallen_debounce_s || stuck) {
            driver_ = Driver::Scaffold;
            handed_off_ = true;
            down_for_ = 0.0;
            stuck_for_ = 0.0;
            up_for_ = 0.0;
            recovering_for_ = 0.0;
            ++rescues_;
        }
    } else {
        scaffold_s_ += dt;
        recovering_for_ += dt;

        // Near upright AND staying there. A body tumbling through upright on its
        // way to the other side would otherwise be handed back mid-fall.
        // With the stillness criterion enabled, "there" means STANDING STILL —
        // see the config note: an 18°-and-rotating handback is an unwinnable
        // episode on a body with a 0.1 s topple clock.
        const bool still_on = c_.still_gyro > 0.0;
        const bool ok = still_on ? (gz < c_.still_gravity_z && wmax < c_.still_gyro)
                                 : (gz < c_.upright_gravity_z);
        if (ok) {
            up_for_ += dt;
        } else {
            up_for_ = 0.0;
        }

        const bool recovered = up_for_ >= (still_on ? c_.still_hold_s : c_.upright_hold_s);
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
