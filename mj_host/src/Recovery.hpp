#pragma once

// =============================================================================
// Recovery — hand the body to a scaffold when it goes down, take it back when up
// =============================================================================
//
// Phase A2 of the port plan, buildable before A1 because it needs a brain that
// FAILS rather than a brain that works.
//
// The arrangement: the brain drives the joints, and when the body ends up on the
// floor the standing scaffold picks it up and hands it back. That is the harness,
// not the controller — the same category as the Godot body's auto-reset teleport,
// and it is what gives a learning brain continuous operation on a body that has
// no passive standing equilibrium.
//
// Three decisions in here matter more than the plumbing.
//
// 1. THE TRIGGER IS EGOCENTRIC, AND IT IS THE LATE ONE.
//
//    `robotd` ships two fall detectors. `FallPredictor` fires early (~26° and
//    still tipping) so the gains can drop before impact; `Safety`'s verdict fires
//    late — projected gravity past -0.5, about 60° from upright, held 200 ms, by
//    which time the robot is on the floor.  This uses the LATE one, at the same
//    numbers, for a reason that is about learning rather than safety: THE FALL IS
//    THE PREDICTION ERROR. Rescue at 26° and the brain gets a truncated version of
//    the thing it most needs to feel.
//
//    Both read projected gravity and the gyro, so nothing here is a sim-only
//    oracle, and the criterion transfers to hardware unchanged. `tilt_deg()` is
//    NOT used: it reads the world frame, and a harness that works in sim and
//    cannot work on the robot is a harness that has to be written twice.
//
// 2. NOTHING LEARNS WHILE THE SCAFFOLD DRIVES.
//
//    During a recovery the brain is not in control. A controller that kept
//    updating on (action, outcome) pairs whose action was the scaffold's would be
//    learning the scaffold's policy — doctrine §5.6, distillation with extra
//    steps, and invisible: the brain would look like it was learning to get up.
//    So `BrainLike::set_learning(false)` is called for the whole window.
//
//    (There is a real subtlety deferred here. A FORWARD model learning from the
//    scaffold's actions is learning the BODY, which is legitimate off-policy data;
//    only the CONTROLLER copying them is copying the teacher. MotorEPM drives both
//    from one TLE and this branch may not edit it, so the safe default is to
//    freeze everything and make the split its own lever later.)
//
// 3. EVERY HAND-OFF AND HAND-BACK IS AN EVENT.
//
//    The picrawler paid for this one: its auto-reset teleport fired no bus event
//    and MotorEPM's leg-phase and EMAs survived fall-plus-respawn, so every
//    coherence and TLE trend spanning a reset was fake. Both edges are reported
//    here so a consumer can mask across them.

#include <array>
#include <string>

namespace mjhost {

// What the harness drives the body with this tick.
enum class Driver { Brain, Scaffold };

// The seam a real brain implements at A1. The stub implements it now, which is
// what makes this phase testable before there is anything to learn.
class BrainLike {
public:
    virtual ~BrainLike() = default;

    // One tick of joint targets, radians, in policy-joint order.
    virtual std::array<double, 14> act(const class DuckBody& body) = 0;

    // Called on both edges of a recovery. Clears whatever action feedback the
    // brain carries, so it does not resume against a stale observation — the same
    // reason `robotd`'s Controller::reset exists.
    virtual void on_reset() {}

    // Frozen for the whole time the scaffold drives. See (2) above.
    virtual void set_learning(bool) {}

    virtual const char* name() const { return "brain"; }
};

// At namespace scope rather than nested: a nested struct's default member
// initializers are not complete inside the enclosing class body, so it cannot be
// used as a default argument there.
struct RecoveryConfig {
    // Matches duck-control's SafetyConfig exactly, so the same criterion runs in
    // sim and on the robot. Upright is -1.0; -0.5 is about 60° over.
    double fallen_gravity_z = -0.5;
    double fallen_debounce_s = 0.2;

    // Back on its feet: near upright, and STAYING there. A tumble passing through
    // upright must not read as recovered.
    double upright_gravity_z = -0.95;   // about 18°
    double upright_hold_s = 0.4;

    // STILLNESS HANDBACK (2026-08-31).  Measured: at the old criterion the brain
    // could inherit a body up to 18° over and still rotating, on a body whose
    // passive topple clock from the home pose is ~0.1 s to 15° — every episode
    // began mid-fall, already past catchability, and every arm's rescue rate was
    // the topple clock in disguise (stub-amp-0: 15° crossed 0.08–0.14 s after
    // handback, handoff ~0.6 s, forever).  The scaffold itself settles to ~0.5°
    // (G2), so requiring its settle before handback costs nothing and hands the
    // brain a genuinely standing body: ≲2.5° AND rotationally quiet, held.
    // still_gyro <= 0 disables the stillness terms (the old criterion exactly).
    double still_gravity_z = -0.999;    // ≈ 2.5°
    double still_gyro      = 0.15;      // rad/s, max |ω| component
    double still_hold_s    = 0.5;

    // A recovery that runs this long is reported rather than waited on. The
    // measured range is 1.6–2.4 s, so this is generous on purpose.
    double give_up_s = 8.0;

    // STUCK-POSE RESCUE (2026-08-31).  Measured: a prior arm found a statically
    // stable SIT — tilt 15–60°, below the fall trigger — and parked there for
    // THIRTEEN MINUTES: zero rescues, zero upright time, zero authority to leave
    // (the model correctly learns a folded body cannot right itself, so the
    // descent damps to nothing).  A stable non-upright pose is a prison, not a
    // success, and thirteen minutes of it is thirteen minutes outside the
    // learnable regime.  Sustained sub-trigger tilt is therefore a fall.
    // stuck_s <= 0 disables.
    double stuck_gravity_z = -0.94;     // ≈ 20° — "not upright"
    double stuck_s = 5.0;               // sustained this long → rescue
};

class Recovery {
public:
    using Config = RecoveryConfig;

    explicit Recovery(Config config = Config()) : c_(config) {}

    // Feed it projected gravity (and the gyro, for the stillness handback), get
    // back who drives this tick.
    Driver update(const std::array<double, 3>& gravity,
                  const std::array<double, 3>& gyro, double dt);

    Driver driver() const { return driver_; }
    bool handed_off_this_tick() const { return handed_off_; }
    bool handed_back_this_tick() const { return handed_back_; }

    // Counters, for the summary.
    int  rescues() const { return rescues_; }
    int  stuck_rescues() const { return stuck_rescues_; }
    int  gave_up() const { return gave_up_; }
    double brain_seconds() const { return brain_s_; }
    double scaffold_seconds() const { return scaffold_s_; }
    double longest_recovery() const { return longest_recovery_; }

private:
    Config c_;
    Driver driver_ = Driver::Brain;
    double down_for_ = 0.0;       // debounce toward "fallen"
    double up_for_ = 0.0;         // hold toward "recovered"
    double recovering_for_ = 0.0;
    bool handed_off_ = false;
    bool handed_back_ = false;
    int  rescues_ = 0;
    int  stuck_rescues_ = 0;
    double stuck_for_ = 0.0;
    int  gave_up_ = 0;
    double brain_s_ = 0.0;
    double scaffold_s_ = 0.0;
    double longest_recovery_ = 0.0;
};

const char* driver_name(Driver d);

}  // namespace mjhost
