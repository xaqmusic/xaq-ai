#pragma once
// Actuation — the brain's action channels to HAT pulses, and who holds the servos.
//
// Two pieces, both pure so they test on the laptop:
//
//   * ActionMap + joint_targets_from_u + us_from_hinge_angles
//       The sim's u -> joint-target mapping (picrawler_body.gd, _discrete_joint_targets),
//       then the servo map's sign/origin at the measured us/rad: the exact inverse of
//       hinge_angles_from_us, which is what the brain's `joints` input already reads.  The
//       constants come from the sim's own export (body_*.json "action_map"), never retyped,
//       and the export carries u_check samples the sim computed with its own function so
//       the C++ is checked against the sim's numbers rather than against a derivation.
//
//   * BrainAuthority
//       The per-tick decision benchd makes about the brain's command stream: apply it, hold
//       where the body is because it went quiet, or escalate.  SPEC §4.2: the calibration
//       deadman does NOT apply to a brain run, so loss of the BRAIN's stream is its own
//       policy, set by the run mode (§4.2.1).
//
// The envelope (per-channel min/max) and the slew limit are NOT here: they live in
// ServoDriver, below the brain, where the brain cannot route around them (Phase 2).
#include "ogma/hw/BrainInputs.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace ogma::hw::brain {

// The 12 action topics in the sim's (leg, joint) order — the topics picrawler_body.gd
// registers with register_action_channel.  Index leg*3 + joint, like ServoMapping.
inline const std::array<std::string, 12>& action_topics() {
    static const std::array<std::string, 12> t = [] {
        std::array<std::string, 12> a;
        const char* legs[4] = {"fl", "fr", "rl", "rr"};
        const char* jts[3]  = {"hip1", "hip2", "knee"};
        for (int l = 0; l < 4; ++l)
            for (int j = 0; j < 3; ++j)
                a[size_t(l * 3 + j)] = std::string("action.") + legs[l] + "_" + jts[j];
        return a;
    }();
    return t;
}

struct ActionMap {
    double hip1_range = 0.0, hip2_range = 0.0;          // rad per unit u
    double knee_fold = 0.0, knee_hyperext = 0.0;        // asymmetric knee: u >= 0 / u < 0
    double knee_symmetric = 0.0;                        // used when knee_widening is off
    bool   knee_widening = true;
    double hip1_rest = 0.0, hip2_rest = 0.0, knee_rest = 0.0;
    std::array<double, 4> splay_out_sign{1, 1, 1, 1};
    nlohmann::json u_check = nlohmann::json::array();   // [{leg, u:[3], t:[3]}] from the sim
    bool ok = false;
    std::string why;

    static ActionMap from_body_json(const nlohmann::json& body) {
        ActionMap m;
        if (!body.contains("action_map")) {
            m.why = "no action_map block — re-export the body calib (export_body_calib.gd)";
            return m;
        }
        try {
            const auto& a = body.at("action_map");
            // Only the backend this mapping implements.  bernoulli_impulse integrates spikes
            // with an RNG; porting it would be a different mapping, not a parameter.
            const std::string backend = a.at("backend").get<std::string>();
            if (backend != "discrete") { m.why = "backend '" + backend + "' is not ported (discrete only)"; return m; }
            m.hip1_range     = a.at("hip1_range").get<double>();
            m.hip2_range     = a.at("hip2_range").get<double>();
            m.knee_fold      = a.at("knee_fold").get<double>();
            m.knee_hyperext  = a.at("knee_hyperext").get<double>();
            m.knee_symmetric = a.at("knee_symmetric").get<double>();
            m.knee_widening  = a.at("knee_widening").get<bool>();
            m.hip1_rest      = body.at("hip1_rest").get<double>();
            m.hip2_rest      = body.at("hip2_rest").get<double>();
            m.knee_rest      = body.at("knee_rest").get<double>();
            const auto& s = a.at("splay_out_sign");
            if (s.size() != 4) { m.why = "splay_out_sign must have 4 entries"; return m; }
            for (int l = 0; l < 4; ++l) m.splay_out_sign[size_t(l)] = s.at(size_t(l)).get<double>();
            if (a.contains("u_check")) m.u_check = a.at("u_check");
            m.ok = true;
        } catch (const std::exception& e) {
            m.why = std::string("action_map: ") + e.what();
        }
        return m;
    }
    static ActionMap load(const std::string& path) {
        std::ifstream f(path);
        if (!f) { ActionMap m; m.why = "cannot read " + path; return m; }
        return from_body_json(nlohmann::json::parse(f));
    }
};

// One leg's joint targets (rad, the sim's hinge convention) from its three u values.
// Mirrors picrawler_body.gd's brain path: clamp u to [-1, 1], apply the splay-out sign to
// hip1, then _discrete_joint_targets.  Order of operations is the sim's.
inline std::array<double, 3> leg_targets_from_u(int leg, double u1, double u2, double u3,
                                                const ActionMap& m) {
    u1 = std::clamp(u1, -1.0, 1.0);
    u2 = std::clamp(u2, -1.0, 1.0);
    u3 = std::clamp(u3, -1.0, 1.0);
    u1 *= m.splay_out_sign[size_t(leg)];
    const double knee_range = m.knee_widening ? (u3 >= 0.0 ? m.knee_fold : m.knee_hyperext)
                                              : m.knee_symmetric;
    return {u1 * m.hip1_range + m.hip1_rest,
            u2 * m.hip2_range + m.hip2_rest,
            u3 * knee_range + m.knee_rest};
}

inline std::array<double, 12> joint_targets_from_u(const std::array<double, 12>& u,
                                                   const ActionMap& m) {
    std::array<double, 12> t{};
    for (int l = 0; l < 4; ++l) {
        const auto lt = leg_targets_from_u(l, u[size_t(l * 3)], u[size_t(l * 3 + 1)],
                                           u[size_t(l * 3 + 2)], m);
        for (int j = 0; j < 3; ++j) t[size_t(l * 3 + j)] = lt[size_t(j)];
    }
    return t;
}

// The inverse of hinge_angles_from_us: us = origin + sign · angle · us_per_rad, by HAT
// channel.  Rounded to the nearest microsecond.  NOT clamped: the envelope belongs to
// ServoDriver, and clamping here would hide from the record what the brain asked for.
inline std::array<int, 12> us_from_hinge_angles(const std::array<double, 12>& angle_lj,
                                                const ServoMapping& map, double us_per_rad) {
    std::array<int, 12> us{};
    us.fill(-1);
    for (int k = 0; k < 12; ++k) {
        const ServoChannel& c = map.by_lj[size_t(k)];
        us[size_t(c.ch)] = int(std::lround(c.origin_us + double(c.sign) * angle_lj[size_t(k)] * us_per_rad));
    }
    return us;
}

// ---- who holds the servos -------------------------------------------------------------

enum class RunMode { Bench, Dev, Autonomous };

inline const char* mode_name(RunMode m) {
    switch (m) {
        case RunMode::Bench:      return "bench";
        case RunMode::Dev:        return "dev";
        case RunMode::Autonomous: return "autonomous";
    }
    return "?";
}
inline bool parse_mode(const std::string& s, RunMode& out) {
    if (s == "bench")      { out = RunMode::Bench; return true; }
    if (s == "dev")        { out = RunMode::Dev; return true; }
    if (s == "autonomous") { out = RunMode::Autonomous; return true; }
    return false;
}

// The brain's command stream, judged once per tick.
//
// ⚠ SILENCE BEFORE THE FIRST COMMAND IS NOT A FAULT.  Switching the daemon to `dev` before
// ogma_host is running must not latch a stop 200 ms later; loss is counted only once a
// stream has actually been applied since authority was granted.
//
// ⚠ STALENESS COUNTS FROM THE LATER OF the last command and the moment authority was
// (re)granted.  A resume after a stop arrives with the last command seconds old, and
// counting from that would call the stream lost on the first tick back.
class BrainAuthority {
public:
    struct Policy {
        int64_t hold_after_ms   = 200;    // 10 ticks: hold where the body is
        int64_t rescue_after_ms = 5000;   // autonomous only: still nothing -> rescue pose
    };
    enum class Event {
        None,      // nothing to do this tick
        Apply,     // a fresh command: drive it
        Hold,      // the stream went quiet: freeze every channel where it is now
        Fault,     // dev: the stream went quiet — freeze AND latch a stop (§4.2.1)
        Rescue,    // autonomous: quiet for rescue_after_ms — the rescue pose
    };

    // Two constructors, not `Policy p = {}`: GCC rejects a default argument built from a
    // nested struct's default member initialisers inside the enclosing class.
    BrainAuthority() = default;
    explicit BrainAuthority(Policy p) : p_(p) {}

    // Authority starts (mode entered, or resumed after a stop).
    void grant(int64_t now) { since_ms_ = now; have_stream_ = false; holding_ = false; rescued_ = false; }

    // blocked: something else owns the servos right now (stopped, rescue moving, low
    // battery, rail back-off).  A blocked tick never applies and never counts as silence.
    Event tick(RunMode mode, int64_t now, bool got_cmd, bool blocked) {
        if (mode == RunMode::Bench) return Event::None;
        if (blocked) { since_ms_ = now; return Event::None; }
        if (got_cmd) {
            last_cmd_ms_ = now;
            if (holding_) ++regains_;
            have_stream_ = true; holding_ = false; rescued_ = false;
            return Event::Apply;
        }
        if (!have_stream_) return Event::None;
        const int64_t age = now - std::max(last_cmd_ms_, since_ms_);
        if (!holding_ && age > p_.hold_after_ms) {
            holding_ = true; ++losses_;
            return mode == RunMode::Dev ? Event::Fault : Event::Hold;
        }
        if (mode == RunMode::Autonomous && holding_ && !rescued_ && age > p_.rescue_after_ms) {
            rescued_ = true;
            return Event::Rescue;
        }
        return Event::None;
    }

    bool    holding() const { return holding_; }
    bool    have_stream() const { return have_stream_; }
    int64_t age_ms(int64_t now) const { return have_stream_ ? now - last_cmd_ms_ : -1; }
    int     losses() const { return losses_; }
    int     regains() const { return regains_; }
    const Policy& policy() const { return p_; }

private:
    Policy  p_;
    int64_t since_ms_ = 0, last_cmd_ms_ = 0;
    bool    have_stream_ = false, holding_ = false, rescued_ = false;
    int     losses_ = 0, regains_ = 0;
};

// Calibration-channel stream guard (SPEC §1.1): "the daemon refuses a brain-rate command
// stream on the calibration channel outright".  Counts commanding verbs in a sliding
// window.  The bench dashboard throttles slider drags to 20 Hz on one channel and every
// tool is far below that; a brain is 50 Hz on twelve.
class RateGuard {
public:
    RateGuard(int max_per_window, int64_t window_ms) : max_(max_per_window), win_(window_ms) {}
    // Returns true if this call is allowed (and counts it).
    bool admit(int64_t now) {
        while (n_ > 0 && now - ring_[head_] >= win_) { head_ = (head_ + 1) % kCap; --n_; }
        if (n_ >= max_ || n_ >= kCap) { ++refused_; return false; }
        ring_[(head_ + n_) % kCap] = now; ++n_;
        return true;
    }
    int refused() const { return refused_; }
    int max_per_window() const { return max_; }
private:
    static constexpr int kCap = 256;
    std::array<int64_t, kCap> ring_{};
    int head_ = 0, n_ = 0, max_, refused_ = 0;
    int64_t win_;
};

} // namespace ogma::hw::brain
