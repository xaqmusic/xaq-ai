#pragma once
// RailGuard — detect a NEW under-voltage event from vcgencmd's sticky throttle mask.
//
// ⚠ WHY STICKY AND NOT LIVE.  Measured 2026-09-13: across an entire run that ended in a
// hard Pi reset, not one 1 Hz poll ever caught the LIVE bits (0-3, "under-voltage now",
// "currently throttled") set — the dips are shorter than the poll interval.  Only the
// sticky history bits (16 "has occurred", 18 "throttling has occurred") were observed.
// A guard written against the live bits would therefore never fire, and would look
// correct while doing nothing.
//
// ⚠ WHY A BASELINE.  The sticky bits do not clear without a reboot, so their mere
// presence says only "sometime since boot".  What is actionable is a bit APPEARING.  A
// reboot clears them (confirmed: 0x0 after the reset), which is what makes a start-time
// baseline a real reference rather than an arbitrary one.
//
// The separation matters because the alternative failed in the field: the low-battery
// auto-safe watches pack voltage, and the Pi died at 7.66 V with 0.92 A flowing.  The
// pack and the bus current are both on the wrong side of the regulator that actually
// fails; this mask is not.

#include <cstdint>

namespace ogma::hw {

class RailGuard {
public:
    // Returns the NEWLY-set sticky bits, or 0 for "nothing new".  The first call only
    // establishes the baseline and can never report an event — bits already set at
    // startup are history, not something that just happened.
    unsigned update(unsigned throttled_mask) {
        if (!have_baseline_) { baseline_ = throttled_mask; have_baseline_ = true; return 0; }
        const unsigned fresh = throttled_mask & ~baseline_;
        if (fresh) baseline_ = throttled_mask;   // so one event fires once, not every poll
        return fresh;
    }
    unsigned baseline() const { return baseline_; }
    bool     armed()    const { return have_baseline_; }

private:
    unsigned baseline_ = 0;
    bool     have_baseline_ = false;
};

}  // namespace ogma::hw
