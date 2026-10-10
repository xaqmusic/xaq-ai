#pragma once
// TofRecovery — when to restart a stalled VL53L0X, and how hard.
//
// ⚠ WHY THERE IS A POLICY AT ALL.  Observed 2026-09-13: a deadman rescue (a twelve-channel
// move) dropped the part out of continuous mode, `read_ready()` returned false on every
// call thereafter, and the LAST GOOD READING kept being published -- `ok = true`,
// `status = valid`, and four minutes old.  The staleness was visible in `age_ms` and
// nothing acted on it.  `ground_clearance` is the PROMOTED height homeostat's input, so a
// silent freeze means defending a belly clearance the robot had minutes ago.
//
// The decision is separated from benchd so it can be tested: benchd's copy lives inside a
// thread-and-socket-laden struct that no unit test can reach, and "recovers correctly" is
// exactly the kind of claim that should not rest on reading the code.
//
// ESCALATION, CHEAPEST FIRST.  A stop/start of continuous mode is a couple of register
// writes; `init()` is the whole ~80-write boot sequence plus two reference calibrations.
// Both run under the bus mutex the servo tick shares, so the cheap one is tried first and
// the expensive one only when the cheap one demonstrably did not take.
//
// ⚠ The split is also DIAGNOSTIC and is the reason the two are counted separately: if a
// restart keeps working the part is losing its ranging state, and if only a full init
// works it is losing configuration.  Those point at different physical causes.

#include <cstdint>

namespace ogma::hw {

struct TofRecoveryPolicy {
    // 1 s is ~30 missed measurements at the 32.9 ms timing budget -- far past ambiguity,
    // and derived from the part's own rate rather than fitted to anything.
    int64_t stale_ms    = 1000;
    // A genuinely dead part must not be hammered once per frame; each attempt costs the
    // servo loop time, because the bus is shared and not thread-safe.
    int64_t cooldown_ms = 3000;

    enum class Action { None, Restart, Reinit };

    // age_ms            how long since a measurement actually landed
    // now_ms            monotonic now
    // last_attempt_ms   when recovery was last ATTEMPTED (0 = never)
    // attempts_since_fresh  recoveries tried since the last good reading (0 = none yet)
    Action decide(int64_t age_ms, int64_t now_ms,
                  int64_t last_attempt_ms, int attempts_since_fresh) const {
        if (age_ms <= stale_ms) return Action::None;
        // ⚠ `last_attempt_ms == 0` means "never tried", NOT "tried at time zero".  Without
        // this the first stall on a freshly-booted daemon waits out a cooldown it never had.
        if (last_attempt_ms != 0 && now_ms - last_attempt_ms <= cooldown_ms)
            return Action::None;
        return attempts_since_fresh > 0 ? Action::Reinit : Action::Restart;
    }
};

}  // namespace ogma::hw
