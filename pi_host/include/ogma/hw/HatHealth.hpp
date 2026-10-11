#pragma once
// HatHealth — telling a HAT that has reset from a bus that glitched (robot, 2026-10-05).
//
// A 19-minute brain run logged 8 "HAT resets".  The fast current capture showed none was
// current-triggered, and the record showed what did trigger them:
//   - 5 were a SINGLE implausible battery read (9-11 V from a 7.2 V pack), three of them with no
//     I2C error anywhere in the 2 s before; around two of them the INA219 on the same bus also
//     returned impossible readings — two chips corrupted at once, i.e. a corrupted read, not a
//     dead MCU (the INA kept sampling through every one);
//   - 3 were benchd's OWN MCU reset, which fired on every 20th CUMULATIVE bus error: ~600
//     sporadic, survivable NACKs over the run added up to deliberate resets.
// Each cost a ~3 s disarm-and-recover.  So:
//   - garbage_confirmed(): a garbage read is a reset only when re-reads agree (a rebooting MCU
//     answers garbage or NACKs consistently for tens of ms; a corrupted transaction does not);
//   - BusBurst: reset the MCU only on a BURST of errors with no successful transaction in
//     between — a hung MCU — never on a running count.
#include <array>
#include <cstdint>

namespace ogma::hw {

// re-reads: how many of the confirmation re-reads came back garbage or failed outright.
inline bool garbage_confirmed(int bad_rereads, int rereads = 3) { return bad_rereads * 2 > rereads; }

class BusBurst {
public:
    struct Policy {
        int     errors      = 20;      // this many ...
        int64_t window_ms   = 1000;    // ... within this window
        int64_t quiet_ok_ms = 500;     // and no successful transaction for this long
        int64_t min_gap_ms  = 5000;    // and not more than one reset per this
    };
    BusBurst() = default;
    explicit BusBurst(Policy p) : p_(p) {}

    void error(int64_t t) { ring_[head_] = t; head_ = (head_ + 1) % kCap; if (n_ < kCap) ++n_; }
    void ok(int64_t t) { last_ok_ = t; }
    int errors_in_window(int64_t t) const {
        int c = 0;
        for (int i = 0; i < n_; ++i) if (t - ring_[i] <= p_.window_ms) ++c;
        return c;
    }
    bool should_reset(int64_t t) const {
        return errors_in_window(t) >= p_.errors && t - last_ok_ >= p_.quiet_ok_ms
               && t - last_reset_ >= p_.min_gap_ms;
    }
    void did_reset(int64_t t) { last_reset_ = t; }
    int64_t since_ok(int64_t t) const { return t - last_ok_; }

private:
    static constexpr int kCap = 64;
    Policy p_{};
    std::array<int64_t, kCap> ring_{};
    int head_ = 0, n_ = 0;
    int64_t last_ok_ = 0, last_reset_ = -1000000;
};

} // namespace ogma::hw
