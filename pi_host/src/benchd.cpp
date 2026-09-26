// ogma_benchd — the calibration / validation daemon (pi_host/PROTOCOL.md).
//
// Four threads share one state under one mutex:
//   tick        50 Hz, clock_nanosleep(TIMER_ABSTIME): re-issues the armed target while the
//               client's deadman is fresh, then ServoDriver::tick() (slew, watchdog, at-limit)
//   telemetry   10 Hz: ADC reads + the frame on the PUB socket + the local JSONL record
//   imu        225 Hz: the ICM-20948 on SPI, sampled and filtered on its OWN thread
//   main        the REP verb loop
// The bus (/dev/i2c-1) is not thread-safe; every I2C access happens under the mutex.
// ⚠ THE IMU IS THE EXCEPTION AND THAT IS THE WHOLE POINT OF PUTTING IT ON SPI.  It shares
// no bus with the servo writes, so it samples at its own rate on its own thread and takes
// the mutex only to publish a finished sample.  Sampling it from the telemetry frame
// instead ran the attitude filter at 10 Hz, which ALIASES the motion it exists to track;
// sampling it from the tick would put SPI inside the servo deadline.  Neither is right:
// port doc sec 2, "fidelity high, transport 50 Hz, LOOP UNTOUCHED".
// There is NO verb that starts a brain here, and none will be added (SPEC §1.1).
#include "ogma/hw/ServoDriver.hpp"
#include "ogma/hw/ResourceMonitor.hpp"
#include "ogma/hw/McuReset.hpp"
#include "ogma/hw/Ina219.hpp"
#include "ogma/hw/Vl53l0x.hpp"
#include "ogma/hw/TofRecovery.hpp"
#include "ogma/hw/RailGuard.hpp"
#include "ogma/body/StrideOdometry.hpp"   // ground_clearance_boom
#include "ogma/hw/Icm20948.hpp"
#include "ogma/hw/SensorCalib.hpp"

#include <nlohmann/json.hpp>
#include <zmq.h>

#include <algorithm>
#include <atomic>
#include <memory>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <thread>

using json = nlohmann::json;
using namespace ogma::hw;

namespace {

constexpr int    DEADMAN_MS      = 1000;
// How long arming stays refused after a rail under-voltage event.  Long enough that a
// caller cannot immediately re-load the rail that just dipped, short enough not to strand
// an operator; it is a back-off, not a lockout, because the sticky bits never clear
// without a reboot and a permanent refusal would need one.
constexpr int64_t RAIL_GUARD_MS = 5000;
// ⚠ TIME-BASED, NOT CALL-COUNTED.  These used to be "every 10th frame()", and frame() is
// called by the telemetry thread at 10 Hz AND by every status RPC -- so the throttle poll
// ran at (10 Hz + client poll rate)/10 and the guard got FASTER WHEN SOMEONE WAS WATCHING.
// Measured: a drill polling status at 50 Hz saw 80 ms median detection where an unattended
// robot gets 1 Hz.  Unattended is exactly when the guard has to work.
//
// The rates are set by cost, measured on this Pi: `vcgencmd get_throttled` is 1.5 ms
// through popen, so 10 Hz costs ~1.5 % of one core and bounds detection at ~100 ms.
// EXT5V is instrument-only and 2.9 ms, so it stays at 1 Hz -- there is nothing to react to.
constexpr int64_t THROTTLE_POLL_MS = 100;
constexpr int64_t EXT5V_POLL_MS    = 1000;   // default; ext5v.rate can raise it for a test
// ⚠ RUNTIME, and it has to be benchd that does the sampling.  A test script polling
// `vcgencmd pmic_read_adc` at 20 Hz alongside this daemon contends for the VideoCore
// mailbox: measured 2026-09-25, one call wedged and took the sampler thread with it
// (subprocess cleanup blocked on an uninterruptible child), which looked exactly like a
// quiet rail -- 1 sample across six pose cycles, reported as a clean PASS by the first
// version of the separation test.  One owner of the mailbox, and everyone else reads the
// record it writes.
int64_t g_ext5v_poll_ms = EXT5V_POLL_MS;
// ToF stall detection lives in ogma::hw::TofRecoveryPolicy (tested there).
constexpr int    CAL_TIMEOUT_MS  = 120000;
constexpr int    OPER_MIN_US     = 900;    // operating envelope until calibration narrows it
constexpr int    OPER_MAX_US     = 2100;
constexpr int    FULL_MIN_US     = 500;    // the servo's full travel — cal.begin only
constexpr int    FULL_MAX_US     = 2500;
constexpr double TICK_HZ         = 50.0;
constexpr double VBAT_LIMP_V     = 6.4;    // HAT minimum is 6.0: limp and refuse arming below this
constexpr double VBAT_RECOVER_V  = 6.7;    // hysteresis: arming allowed again above this
// Pose moves: twelve servos starting at once on the 5 V/3 A DC-DC the Pi shares browned the
// Pi out (2026-08-29, reproduced: telemetry gone 0.5 s after pose.set, Pi rebooted).  So a
// pose starts its channels one at a time and slews them gently; servo.set keeps the fast slew.
constexpr int NORMAL_SLEW_US     = 40;
// ⚠ The NORMAL slew is what the brain's commands ride — pose moves use g_pose_slew_us and
// are a different path.  It was a compile-time constant, which made the one setting a sim
// study actually recommends changing (40 -> 50, ledger 2026-09-13) untestable without a
// rebuild.  Flag, not a new default: the default is still NORMAL_SLEW_US.
int g_normal_slew_us = NORMAL_SLEW_US;
int g_pose_slew_us = 12;                   // 600 us/s: a 1000 us move takes ~1.7 s
int g_pose_stagger_ticks = 5;              // 100 ms between channel starts
// CALIBRATION DATA, not a constant (Ina219.hpp): at 10 mOhm the trace and solder are a
// large fraction of the part, and a meter cannot reach it through ~200 mOhm of leads.
// Placeholder until the bench fit; override with --r-shunt.
double g_r_shunt = 0.01;
bool   g_r_shunt_override = false;
bool   g_tof_override = false;
// CALIBRATION DATA too (Vl53l0x.hpp): the ToF is recessed up inside the chassis so the
// belly's 0-56 mm range clears the part's unreliable short end, and only a tape measure
// knows by how much.  0 = flush, which is the pre-bench default and not a fitted value.
double g_tof_offset_mm = 0.0;
// Boom geometry (BOM §9.1 "as built").  ⚠ sensor-above-belly is the SAME fitted number as
// the mount offset -- expressing the offset from the belly plane is what removes the need
// for a third constant the robot has never measured (see ground_clearance_boom).
double g_tof_boom_above_belly_m = 0.0648;   // = tof.mount_offset_mm, overridden from calib
double g_tof_boom_z_m           = -0.070;   // 70 mm AFT; forward is +Z, so negative
// ⚠ DO NOT fsync() THE RECORD FROM record().  It was tried 2026-09-08 and MEASURED: at a
// 1 s cadence, under the mutex the 50 Hz servo tick needs, an SD fsync costs ~80 ms and the
// loop fell to 35.8 Hz with 54 overruns in 12 s (worst tick 419 % of the 20 ms budget, against
// 27 % without).  Durability bought with the control loop is not a trade this daemon may make.
// The record surviving a brownout is a real requirement -- twice a crash has destroyed its own
// evidence -- but the answer is to record on ANOTHER MACHINE (tools/tele_record.py), which
// cannot share the fate of the one that died.  Leaving this comment so it is not retried.
// The local record's flush() reaches the page cache and no further, which is why /tmp lost
// everything (BOM 3.10) and $HOME still lost the tail of the stalled-leg shutdown.  That
// gap is real, and it is closed OFF-BOARD rather than here.

int64_t mono_ms() {
    timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}
std::string utc_now() {
    char buf[32]; time_t t = time(nullptr); tm g; gmtime_r(&t, &g);
    strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &g); return buf;
}
std::string stamp_now() {
    char buf[32]; time_t t = time(nullptr); tm l; localtime_r(&t, &l);
    strftime(buf, sizeof buf, "%Y%m%d_%H%M%S", &l); return buf;
}
const char* sim_leg_for(const std::string& phys) {
    // The leg-naming mirror (port doc): sim "fl" is the physical front-RIGHT.
    if (phys == "FL") return "fr";
    if (phys == "FR") return "fl";
    if (phys == "RL") return "rr";
    if (phys == "RR") return "rl";
    return "";
}

struct State {
    std::mutex m;
    LinuxI2cBus bus;
    RobotHat hat;
    ServoDriver driver;
    std::unique_ptr<McuReset> mcu;              // null if the GPIO line could not be taken
    std::string body;
    int  armed_ch = -1;                          // last channel set by servo.set, for the dashboard
    int  cal_ch = -1;
    int64_t cal_until_ms = 0;
    int64_t last_client_ms = 0;
    int  watchdog_trips = 0;
    int  overruns = 0;
    int  bus_errors = 0;
    json last_adc = json::array({0, 0, 0, 0, 0});
    bool low_battery = false;
    std::string rescue_name = "rescue";      // the pose that stands in for limp on this HAT
    int64_t rescue_until_ms = 0;              // while set, the tick keeps feeding the driver
    std::vector<std::pair<int,int>> pose_queue;  // (ch, us) still to start, in order
    int pose_stagger_left = 0;
    bool pose_move_active = false;
    bool deadman_tripped = false;
    std::string pi_throttled = "0x0";   // vcgencmd get_throttled, polled ~1 Hz
    // ---- 5 V RAIL GUARD (2026-09-13) -------------------------------------------------
    // ⚠ THE LOW-BATTERY AUTO-SAFE WATCHES THE WRONG RAIL.  Measured: the Pi hard-reset
    // with vbat at 7.66 V (auto-safe fires at 6.4) and only 0.92 A flowing — below what
    // the same run had already survived.  The cliff is the HAT's 5 V regulator, and the
    // INA219 sits on the BATTERY side of it, so neither instrument can see the fault.
    //
    // `get_throttled` bit 16 CAN: it watches the Pi's own supply, and in that run it went
    // non-zero 303 SECONDS before the reset.  Five minutes of warning, on a value this
    // daemon was already reading once a second and doing nothing with.
    //
    // ⚠ THE LIVE BITS (0-3) ARE USELESS AT 1 Hz — across the whole run not one poll caught
    // them set, because the events are shorter than the interval.  Only the STICKY history
    // bits were ever observed.  So the guard watches for sticky bits APPEARING against a
    // baseline taken at startup; a reboot clears them (confirmed 0x0 after the reset),
    // which is what makes a start-time baseline meaningful rather than arbitrary.
    RailGuard rail_guard{};              // sticky-mask transition detector (tested there)
    int      rail_events    = 0;         // how many NEW under-voltage/throttle events
    int64_t  rail_guard_until_ms = 0;    // arming refused while set, to let the rail recover
    // ⚠ FAULT INJECTION, and it MUST be visible in the frame.  This mask is OR'd into the
    // polled one, so a consumer that reads rail_events cannot otherwise tell a real rail
    // dip from a drill.  A confounded number looks exactly like a good one; publish the
    // state that invalidates it.
    unsigned rail_inject = 0;
    double   ext5v          = 0.0;       // the Pi's OWN measure of the rail the HAT feeds
    int64_t  ext5v_ms       = 0;
    int64_t throttled_next_ms = 0;   // deadlines, so the rate is OURS and not the client's
    int64_t ext5v_next_ms     = 0;
    // Whole-robot bus current (BOM 3).  Instrument only -- nothing here consumes it.
    // null when the part is absent, and benchd then behaves exactly as it did before.
    std::unique_ptr<Ina219> ina;
    bool    ina_ok      = false;
    bool    ina_resync  = false;      // another process had reprogrammed CONFIG
    double  ina_i       = 0.0;        // A, instantaneous (128-sample average on the part)
    double  ina_v       = 0.0;        // V, INA219's own bus channel -- independent of A4
    double  ina_i_ema   = 0.0;        // A, tau 30 s: the SLOW metric
    double  ina_i_peak  = 0.0;        // A, decaying peak-hold, tau 60 s: the RECENT worst
    double  ina_i_max   = 0.0;        // A, since start
    double  ina_charge  = 0.0;        // A*s drawn since start
    double  ina_energy  = 0.0;        // J drawn since start
    int64_t ina_last_ms = 0;
    int     ina_errors  = 0;
    // Belly clearance (BOM 2 #4).  Instrument only -- nothing in this daemon steers on
    // it.  null when the part is absent, and benchd then behaves exactly as before.
    std::unique_ptr<Vl53l0x> tof;
    bool    tof_ok        = false;
    uint16_t tof_raw_mm   = 0;        // as the chip reported it, before the offset
    double  tof_m         = 0.0;      // belly clearance, m -- raw minus the mount offset
    // ⚠ PUBLISHED ALONGSIDE tof_m, NEVER INSTEAD OF IT.  tof_m feeds the PROMOTED height
    // homeostat through ground_clearance(); swapping the boom correction in underneath it
    // would change a promoted input with no A/B, which is a lever masquerading as a fix.
    // So the corrected value ships as its own channel at gain 0 until it is measured.
    double  tof_m_comp    = 0.0;      // belly clearance with the boom's pitch lever removed
    double  tof_comp_delta = 0.0;     // comp - uncomp, m: how much the correction is doing
    bool    tof_comp_valid = false;   // false when the IMU has no attitude to correct with
    bool    tof_valid     = false;
    std::string tof_status = "noupdate";
    double  tof_signal    = 0.0;      // Mcps returned off the target
    double  tof_ambient   = 0.0;      // Mcps the room contributed
    double  tof_spads     = 0.0;
    double  tof_m_ema     = 0.0;      // m, tau 30 s: the SLOW metric -- how high it rides
    // The worst clearance, decaying.  A MIN-hold, not a peak: on this channel LOW is the
    // dangerous end, so the peak-hold that serves current would report the safe extreme.
    double  tof_m_min     = 0.0;      // m, tau 60 s decaying worst
    double  tof_m_min_all = 0.0;      // m, worst since start
    // Fraction of readings the part itself rejected, tau 30 s.  This is the channel's
    // own honesty meter: a belly sensor that is 40 % invalid is not a channel, and the
    // millimetres alone cannot say so -- an invalid reading looks like a good one.
    double  tof_bad_frac  = 0.0;
    int64_t tof_last_ms   = 0;
    int64_t tof_fresh_ms  = 0;        // when a measurement last actually arrived
    int     tof_errors    = 0;
    // ---- ToF stall recovery (2026-09-13) --------------------------------------------
    // ⚠ THE PART STOPS RANGING AND DOES NOT RESTART ITSELF.  Observed: a deadman rescue
    // (a twelve-channel move) dropped the VL53L0X out of continuous mode and read_ready()
    // returned false forever after, so the LAST GOOD READING stood while age_ms climbed
    // to four minutes -- ok=true, status=valid, and completely stale.  The INA219 on the
    // same bus and the same 3V3 was untouched throughout, so this is specific to the part
    // with the most state, not a bus or supply event.  ~1 in 600 pose moves.
    //
    // ground_clearance is the PROMOTED height homeostat's input, so a silent freeze means
    // defending a clearance the robot had minutes ago.  Recovery is not optional.
    int64_t tof_recover_ms   = 0;     // last recovery ATTEMPT, for the cooldown
    int     tof_since_fresh  = 0;     // recovery attempts since a measurement last landed
    int     tof_restarts     = 0;     // cheap: stop/start continuous
    int     tof_reinits      = 0;     // expensive: the full boot sequence
    int     tof_unreachable  = 0;     // model id did not answer -- not a ranging problem
    TickBudget  budget{TICK_HZ, 25};    // 25 ticks = 0.5 s: fast enough to read as a meter
    int  load_cpu_us   = 0;             // synthetic load, gain-0 by default (see the 'load' verb)
    int  load_block_us = 0;
    TickStats   tick_cost{};            // last closed window
    HostSample  host{};                 // sampled ~1 Hz OFF the tick thread
    double tick_hz_meas = 0.0;
    uint64_t seq = 0;
    int64_t t0_ms = mono_ms();
    json map = json::object();
    std::string map_path;
    json poses = json::object();                 // name -> {us:[12], saved_at}
    std::string poses_path;
    std::ofstream log;

    State(const std::string& dev, const std::string& body_, const std::string& map_path_, const std::string& poses_path_, const std::string& log_path)
        : bus(dev), hat(bus), driver(hat, ServoDriverConfig{40, int(TICK_HZ / 2), TICK_HZ}),
          body(body_), map_path(map_path_), poses_path(poses_path_), log(log_path, std::ios::app) {
        std::ifstream pf(poses_path); if (pf) { try { poses = json::parse(pf); } catch (...) { poses = json::object(); } }
        if (!poses.is_object()) poses = json::object();
        for (int c = 0; c < ServoDriver::N; ++c) driver.set_limits(c, {OPER_MIN_US, OPER_MAX_US});
        map = {{"version", 1}, {"body", body}, {"servos", json::array()}};
    }

    void record(const char* kind, const json& j) {
        if (!log) return;
        json line = {{"t_mono_ms", mono_ms()}, {"kind", kind}, {"data", j}};
        log << line.dump() << '\n';
        log.flush();
    }

    // ---- envelope bookkeeping (callers hold m) ----
    void end_cal(const char* why) {
        if (cal_ch < 0) return;
        driver.set_limits(cal_ch, oper_limits(cal_ch));
        record("cal.end", {{"ch", cal_ch}, {"why", why}});
        cal_ch = -1; cal_until_ms = 0;
    }
    ServoLimits oper_limits(int ch) const {
        for (auto& s : map["servos"])
            if (s.value("ch", -1) == ch) return {s.value("min_us", OPER_MIN_US), s.value("max_us", OPER_MAX_US)};
        return {OPER_MIN_US, OPER_MAX_US};
    }
    json& map_entry(int ch) {
        for (auto& s : map["servos"]) if (s.value("ch", -1) == ch) return s;
        map["servos"].push_back({{"ch", ch}});
        return map["servos"].back();
    }
    bool has_rescue() const { return poses.contains(rescue_name) && poses[rescue_name].contains("us") && poses[rescue_name]["us"].is_array() && poses[rescue_name]["us"].size() == size_t(ServoDriver::N); }

    // The SAFE ACTION.  Measured 2026-08-29: this HAT cannot de-energise a servo from
    // software — pulse 0/1/ARR are ignored, a stopped timer is ignored, and the MCU held
    // in reset for 30 s still leaves the servo powered.  So "limp" is a pose: every
    // channel goes to the saved `rescue` pose (slewed), which is what limp existed for —
    // no servo left straining into a hard stop.  Without a rescue pose the old register
    // write is issued and recorded as such (it does nothing on this hardware).
    // Caller holds m (the bus is not thread-safe).  Never throws out: a missing or
    // sulking INA219 must not cost the tick that keeps the servos fed.
    void sample_ina() {
        if (!ina) return;
        try {
            ina_resync = ina->ensure_configured();
            const auto s2 = ina->read();
            const int64_t now = mono_ms();
            ina_i = s2.current_a; ina_v = s2.bus_v; ina_ok = true;
            if (ina_i > ina_i_max) ina_i_max = ina_i;
            if (ina_last_ms) {
                const double dt = std::min(1.0, (now - ina_last_ms) / 1000.0);   // clamp a scheduling gap
                ina_charge += ina_i * dt;
                ina_energy += ina_i * ina_v * dt;
                // Time-constant form, so the numbers keep their meaning if the rate changes.
                const double a_ema = 1.0 - std::exp(-dt / 30.0);
                const double a_pk  = 1.0 - std::exp(-dt / 60.0);
                ina_i_ema += a_ema * (ina_i - ina_i_ema);
                // Peak means WORST DRAW, so it floors at zero: while the charger is on,
                // current is negative and a signed peak-hold would report the charge rate.
                ina_i_peak = std::max({0.0, ina_i, ina_i_peak - a_pk * ina_i_peak});
            } else {
                ina_i_ema = ina_i;
                ina_i_peak = std::max(0.0, ina_i);
            }
            ina_last_ms = now;
        } catch (const std::exception& e) {
            ina_ok = false;
            if (++ina_errors % 50 == 1) record("ina_error", {{"what", e.what()}, {"count", ina_errors}});
        }
    }

    // Caller holds m.  NON-BLOCKING BY CONSTRUCTION: the part free-runs at ~30 Hz and
    // this polls at 10, so a measurement is normally waiting -- but when it is not, this
    // returns immediately rather than waiting out a 33 ms conversion while holding the
    // bus mutex the 50 Hz servo tick needs.  A ToF must never cost a servo deadline.
    // --- IMU (instrument only) -------------------------------------------------------
    // ⚠ INSTRUMENT, NOT A LEVER.  Published so it can be WATCHED across hand-posed motions
    // before anything consumes it -- the same admission rule the ultrasonic is under (BOM
    // sec 7): it earns a lever only once someone can name the prediction error it reduces.
    // On SPI, so it does not share the servo-contended I2C bus and nothing here competes
    // with the 12 servo writes.
    std::unique_ptr<Icm20948> imu;
    ImuSample imu_s{};
    bool      imu_ok     = false;
    int       imu_errors = 0;

    // Escalating, cheapest first, and every step counted.  ⚠ THIS RUNS UNDER `m`, like
    // every other bus access (I2cBus is not thread-safe and the servo tick shares it), so
    // it stalls the servo loop for its duration.  That is why it escalates rather than
    // reaching for init(): a stop/start is a couple of register writes, while init() is
    // the whole ~80-write boot sequence plus two reference calibrations.  The driver
    // watchdog is 25 ticks (500 ms), so even the expensive path stays well inside it.
    //
    // ⚠ COUNTED AND PUBLISHED ON PURPOSE.  A silent auto-recovery would paper over the
    // electrical marginality that causes this instead of surfacing it -- and the SPLIT
    // between restarts and reinits is diagnostic: if a cheap restart keeps working, the
    // part is losing its ranging state; if only a full init does, it is losing config.
    TofRecoveryPolicy tof_policy{};

    void tof_recover(int64_t age_ms, bool full) {
        const int64_t t0 = mono_ms();
        try {
            if (!tof->model_id_ok()) {
                // Not a ranging failure -- the part is not answering at all.  Do not
                // hammer it: that is a wiring or power fault and a retry cannot fix it.
                ++tof_unreachable;
                record("tof_unreachable", {{"age_ms", age_ms}, {"count", tof_unreachable}});
                return;
            }
            if (full) { tof->init(); tof->start_continuous(); ++tof_reinits; }
            else      { tof->stop_continuous(); tof->start_continuous(); ++tof_restarts; }
            ++tof_since_fresh;
            record("tof_recover", {{"kind", full ? "reinit" : "restart"},
                                   {"age_ms", age_ms}, {"took_ms", mono_ms() - t0},
                                   {"restarts", tof_restarts}, {"reinits", tof_reinits}});
        } catch (const std::exception& e) {
            ++tof_errors;
            record("tof_recover_failed", {{"what", e.what()}, {"age_ms", age_ms}});
        }
    }

    void sample_tof() {
        if (!tof) return;
        try {
            Vl53l0x::Reading r;
            const int64_t now = mono_ms();
            if (!tof->read_ready(r)) {
                // Not an error BY ITSELF: the previous reading stands, and tof_fresh_ms
                // ages so a stalled part is visible as staleness rather than as a frozen
                // number.  But visible is not enough -- nothing was acting on it, which is
                // how a four-minute-old reading kept being published as healthy.
                tof_ok = true;
                const int64_t age = tof_fresh_ms ? now - tof_fresh_ms : 0;
                // 1 s is ~30 missed measurements at the 32.9 ms timing budget: far past
                // ambiguity, and not a number fitted to anything.
                if (tof_fresh_ms) {
                    const auto act = tof_policy.decide(age, now, tof_recover_ms, tof_since_fresh);
                    if (act != TofRecoveryPolicy::Action::None) {
                        tof_recover_ms = now;
                        tof_recover(age, act == TofRecoveryPolicy::Action::Reinit);
                    }
                }
                return;
            }
            tof_ok      = true;
            tof_since_fresh = 0;          // recovery worked (or was never needed)
            tof_raw_mm  = r.raw_mm;
            tof_m       = r.distance_m;
            // The boom correction (§9.9, ogma::body::ground_clearance_boom).  Needs attitude, so it
            // is only valid once the IMU filter has a fused up -- and says so rather than quietly
            // publishing the uncorrected number under the corrected name.
            // ⚠ imu_s.up_fused is a std::array<float,3> on the wire, not a Vec3f -- the
            // sample struct is plain data so it can cross the telemetry boundary.  Rebuild
            // the vector here rather than changing that struct.
            const auto& uf = imu_s.up_fused;
            const ogma::body::Vec3f up_b(uf[0], uf[1], uf[2]);
            const float up_len2 = uf[0]*uf[0] + uf[1]*uf[1] + uf[2]*uf[2];
            if (imu && imu_ok && up_len2 > 0.25f) {          // |up| > 0.5
                tof_m_comp = ogma::body::ground_clearance_boom(
                    double(tof_raw_mm) / 1000.0, up_b,
                    g_tof_boom_above_belly_m, g_tof_boom_z_m);
                tof_comp_delta = tof_m_comp - tof_m;
                tof_comp_valid = true;
            } else {
                tof_m_comp = tof_m; tof_comp_delta = 0.0; tof_comp_valid = false;
            }
            tof_valid   = r.valid;
            tof_status  = Vl53l0x::status_name(r.status);
            tof_signal  = r.signal_mcps;
            tof_ambient = r.ambient_mcps;
            tof_spads   = r.spads;
            const double dt = tof_last_ms ? std::min(1.0, (now - tof_last_ms) / 1000.0) : 0.0;
            const double a_bad = dt > 0.0 ? 1.0 - std::exp(-dt / 30.0) : 1.0;
            tof_bad_frac += a_bad * ((r.valid ? 0.0 : 1.0) - tof_bad_frac);
            // The slow metrics track only VALID readings.  Folding the far-limit stand-in
            // for a failed measurement into a clearance average would report the belly
            // rising every time the sensor lost the floor -- exactly backwards.
            if (r.valid) {
                if (dt > 0.0 && tof_fresh_ms) {
                    const double a_ema = 1.0 - std::exp(-dt / 30.0);
                    const double a_min = 1.0 - std::exp(-dt / 60.0);
                    tof_m_ema += a_ema * (tof_m - tof_m_ema);
                    // Decay the min-hold back UP toward the current reading, so it
                    // reports the recent worst rather than the worst ever.
                    tof_m_min = std::min(tof_m, tof_m_min + a_min * (tof_m - tof_m_min));
                } else {
                    tof_m_ema = tof_m; tof_m_min = tof_m; tof_m_min_all = tof_m;
                }
                if (tof_m < tof_m_min_all) tof_m_min_all = tof_m;
                tof_fresh_ms = now;
            }
            tof_last_ms = now;
        } catch (const std::exception& e) {
            tof_ok = false;
            if (++tof_errors % 50 == 1) record("tof_error", {{"what", e.what()}, {"count", tof_errors}});
        }
    }

    void rescue(const char* why) {
        end_cal(why);
        armed_ch = -1;
        if (has_rescue()) {
            const json& us = poses[rescue_name]["us"];
            std::vector<std::pair<int,int>> targets;
            for (int c = 0; c < ServoDriver::N; ++c)
                if (us[c].is_number() && us[c].get<int>() >= 0) targets.push_back({c, us[c].get<int>()});
            begin_pose_move(targets);
            rescue_until_ms = mono_ms() + 3000 + int64_t(targets.size()) * g_pose_stagger_ticks * 20 + 4000;   // stagger + travel
            record("rescue", {{"why", why}, {"pose", rescue_name}});
        } else {
            try { driver.limp_all(); } catch (...) {}
            record("limp", {{"why", why}, {"note", "no rescue pose saved; pulse 0 is ignored by this HAT"}});
        }
    }
    void limp_all(const char* why) { rescue(why); }

    // Begin a staggered, gentle move of many channels.  Channels are ordered by distance
    // to travel (shortest first) so the big swings start last, one every stagger period.
    void begin_pose_move(const std::vector<std::pair<int,int>>& targets) {
        pose_queue.clear();
        for (auto& t : targets) pose_queue.push_back(t);
        std::sort(pose_queue.begin(), pose_queue.end(), [&](const std::pair<int,int>& a, const std::pair<int,int>& b) {
            int da = std::abs(a.second - (driver.armed(a.first) ? driver.current_us(a.first) : a.second));
            int db = std::abs(b.second - (driver.armed(b.first) ? driver.current_us(b.first) : b.second));
            return da < db; });
        driver.set_slew_us_per_tick(g_pose_slew_us);
        pose_move_active = true;
        pose_stagger_left = 0;                                   // first channel starts this tick
    }
    // Called every tick (holding m): start the next queued channel when its time comes;
    // restore the fast slew once everything has landed.
    void service_pose_move() {
        if (!pose_move_active) return;
        if (!pose_queue.empty()) {
            if (pose_stagger_left <= 0) {
                auto [ch, us] = pose_queue.front(); pose_queue.erase(pose_queue.begin());
                try { driver.command(ch, us); } catch (...) {}
                pose_stagger_left = g_pose_stagger_ticks;
            } else --pose_stagger_left;
        } else if (driver.settled()) {
            driver.set_slew_us_per_tick(g_normal_slew_us);
            pose_move_active = false;
            record("pose.landed", {});
        }
    }

    json frame() {   // caller holds m
        const int64_t now = mono_ms();
        json servos = json::array();
        for (int c = 0; c < ServoDriver::N; ++c) {
            auto lim = driver.limits(c);
            servos.push_back({{"ch", c}, {"target_us", driver.target_us(c)}, {"current_us", driver.current_us(c)},
                              {"armed", driver.armed(c)}, {"at_limit_s", driver.time_at_limit_s(c)},
                              {"min_us", lim.min_us}, {"max_us", lim.max_us}});
        }
        json adc = json::array();
        try {
            for (int c = 0; c < RobotHat::N_ADC; ++c) adc.push_back(hat.adc_raw(c));
            const double v = adc[4].get<int>() * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
            if (v > 9.0) { record("adc_garbage", {{"vbat", v}}); adc = last_adc; }   // post-reset garbage
            else last_adc = adc;
        } catch (const std::exception& e) {
            ++bus_errors; adc = last_adc;                          // keep the last good reading
            if (bus_errors % 50 == 1) record("bus_error", {{"where", "adc"}, {"what", e.what()}, {"count", bus_errors}});
        }
        sample_ina();
        sample_tof();
        const double vbat = adc[4].get<int>() * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
        // SPEC 4.6 — low-voltage auto-safe.  The HAT powers the Pi too, so a dying pack
        // takes the whole robot down; go limp early and say so.
        if (!low_battery && vbat < VBAT_LIMP_V && vbat > 1.0) {
            low_battery = true; rescue("low battery");
            record("low_battery", {{"vbat", vbat}, {"limp_v", VBAT_LIMP_V}});
        } else if (low_battery && vbat > VBAT_RECOVER_V) {
            low_battery = false; record("battery_ok", {{"vbat", vbat}});
        }
        if (now >= throttled_next_ms) {
            throttled_next_ms = now + THROTTLE_POLL_MS;
            // EXT5V is the Pi's own reading of the 5 V input the HAT feeds — the rail that
            // actually fails.  INSTRUMENT ONLY for now: published so its behaviour under
            // load can be watched before any threshold is chosen from it, which is the
            // same admission rule every other sensor here is under.
            //
            // ⚠ Its own, SLOWER deadline.  Nothing reacts to it, so there is no latency
            // requirement — and it costs 2.9 ms against get_throttled's 1.5 ms.  (An
            // earlier note here said this call "runs at ~8 Hz".  That was the sample
            // spacing of the operator's logging script, not the cost of the call; the
            // call is 2.9 ms and would run at ~340 Hz.  See bom §3.8.8.4.)
            if (now >= ext5v_next_ms) {
                ext5v_next_ms = now + g_ext5v_poll_ms;
                if (FILE* f = popen("vcgencmd pmic_read_adc EXT5V_V 2>/dev/null", "r")) {
                    char b[128] = {0};
                    if (fgets(b, sizeof b, f)) {
                        std::string t(b); auto eq = t.find('=');
                        if (eq != std::string::npos) {
                            try { ext5v = std::stod(t.substr(eq + 1)); ext5v_ms = mono_ms(); }
                            catch (...) { /* a malformed line is not worth a tick */ }
                        }
                    }
                    pclose(f);
                }
                // In fast mode the 10 Hz telemetry frame is too coarse to be the record,
                // so each sample gets its own line.  Gated on the rate so normal running
                // does not grow the log by 10x for a channel nothing acts on.
                if (g_ext5v_poll_ms < 500)
                    record("ext5v", {{"v", ext5v}, {"vbat", vbat}, {"i_a", ina_i}});
            }
            if (FILE* f = popen("vcgencmd get_throttled 2>/dev/null", "r")) {
                char b[64] = {0}; if (fgets(b, sizeof b, f)) { std::string t(b); auto eq = t.find('='); if (eq != std::string::npos) { pi_throttled = t.substr(eq + 1); while (!pi_throttled.empty() && (pi_throttled.back() == '\n' || pi_throttled.back() == '\r')) pi_throttled.pop_back(); } }
                pclose(f);
            }
            // ---- the guard proper -------------------------------------------------
            unsigned thr = 0;
            try { thr = unsigned(std::stoul(pi_throttled, nullptr, 0)); } catch (...) { thr = 0; }
            // Injected bits join the mask HERE, upstream of the guard, so a drill runs the
            // identical path a real dip runs: same update(), same rescue, same back-off.
            // Injecting further down would test a copy of the mechanism, not the mechanism.
            thr |= rail_inject;
            if (unsigned fresh = rail_guard.update(thr)) {
                // A sticky bit that was NOT set at startup has appeared: the Pi's own
                // supply dipped just now.  Drop the load — the servos ARE the load — and
                // refuse arming briefly so the rail is not immediately re-loaded.
                ++rail_events;
                record("rail_undervolt", {{"throttled", pi_throttled}, {"new_bits", fresh},
                                          {"ext5v", ext5v}, {"vbat", vbat},
                                          {"count", rail_events},
                                          {"injected", (fresh & rail_inject) != 0}});
                rescue("rail under-voltage");
                // ⚠ The back-off must OUTLAST the recall it just started.  RAIL_GUARD_MS is
                // 5 s; the staggered rescue recall runs ~8.2 s (3000 + 12*stagger*20 + 4000,
                // measured).  servo.set checks neither pose_move_active nor rescue_active, so
                // a 5 s back-off leaves ~3 s in which a client can command a channel into a
                // rescue that is still moving -- fighting the very recovery the guard
                // ordered.  Take the later of the two.
                rail_guard_until_ms = std::max(mono_ms() + RAIL_GUARD_MS, rescue_until_ms);
            }
        }
        bool any_armed = false; for (int c = 0; c < ServoDriver::N; ++c) any_armed |= driver.armed(c);
        if (!any_armed) armed_ch = -1;
        const int64_t dm = any_armed ? std::max<int64_t>(0, DEADMAN_MS - (now - last_client_ms)) : 0;
        return {{"seq", ++seq}, {"t_mono_ms", now}, {"uptime_s", (now - t0_ms) / 1000.0}, {"mode", "bench"},
                {"body", body}, {"vbat", vbat}, {"adc", adc}, {"armed_ch", armed_ch}, {"cal_ch", cal_ch},
                {"cal_ms_left", cal_ch >= 0 ? std::max<int64_t>(0, cal_until_ms - now) : 0},
                {"deadman_ms_left", dm}, {"watchdog_trips", watchdog_trips}, {"tick_hz", tick_hz_meas},
                {"overruns", overruns}, {"bus_errors", bus_errors}, {"low_battery", low_battery}, {"rescue_pose", has_rescue() ? json(rescue_name) : json(nullptr)},
                {"rescue_active", mono_ms() < rescue_until_ms}, {"pose_move_active", pose_move_active}, {"pose_queue", pose_queue.size()},
                {"pi_throttled", pi_throttled},
                // Whole-robot current: Pi + the 5 V regulator + all 12 servos (BOM 3).
                // r_shunt is calibration data, so it rides with the numbers it derives.
                {"ext5v", ext5v}, {"ext5v_age_ms", ext5v_ms ? mono_ms() - ext5v_ms : -1},
                {"rail_events", rail_events},
                {"rail_guarded", mono_ms() < rail_guard_until_ms},
                {"rail_inject", rail_inject},
                // The guard's own reference.  Published because "nothing fired" is
                // ambiguous without it: a bit already IN the baseline is history by
                // design, not a broken guard.
                {"rail_baseline", rail_guard.baseline()},
                {"ina", ina ? json{{"ok", ina_ok}, {"i_a", ina_i}, {"v", ina_v},
                                   {"i_ema", ina_i_ema}, {"i_peak", ina_i_peak}, {"i_max", ina_i_max},
                                   {"charge_as", ina_charge}, {"energy_j", ina_energy},
                                   {"r_shunt", ina->r_shunt()}, {"resync", ina_resync},
                                   // Charge current flows BACKWARDS through the shunt, so the
                                   // sign is a plugged-in detector -- and a warning that every
                                   // energy number is confounded while it is true.
                                   {"charging", ina_i < -0.02},
                                   {"errors", ina_errors}}
                             : json(nullptr)},
                // Belly clearance.  raw_mm is the RECORD (the mount offset is a fit, and
                // re-fitting it must re-derive every sample); m is what a consumer reads.
                // status/signal/ambient ride WITH the number they qualify -- an invalid
                // ToF reading is not a large or small distance, it is an arbitrary one,
                // and it is indistinguishable from a good one at the consumer.
                {"tof", tof ? json{{"ok", tof_ok}, {"raw_mm", tof_raw_mm}, {"m", tof_m},
                                   {"valid", tof_valid}, {"status", tof_status},
                                   {"signal_mcps", tof_signal}, {"ambient_mcps", tof_ambient},
                                   {"spads", tof_spads},
                                   {"m_comp", tof_m_comp}, {"comp_delta", tof_comp_delta},
                                   {"comp_valid", tof_comp_valid},
                                   {"boom_z_m", g_tof_boom_z_m},
                                   {"m_ema", tof_m_ema}, {"m_min", tof_m_min}, {"m_min_all", tof_m_min_all},
                                   {"bad_frac", tof_bad_frac},
                                   // Recovery is counted so the marginality that causes
                                   // it stays visible instead of being papered over.
                                   {"restarts", tof_restarts}, {"reinits", tof_reinits},
                                   {"unreachable", tof_unreachable},
                                   {"offset_mm", tof->config().mount_offset_mm},
                                   // How long since a measurement actually landed.  A part
                                   // that stops ranging otherwise shows as a steady number.
                                   // ⚠ CLAMPED AT ZERO, and not defensively: `now` is taken
                                   // at the top of frame() and sample_tof() reads the clock
                                   // again a few ms later, so a fresh reading lands in the
                                   // FUTURE relative to this frame's timestamp.  Live
                                   // bring-up published age_ms -2, which every consumer here
                                   // reads as "no measurement yet" -- the self-check fails on
                                   // a healthy part and the trace draws nothing but gaps.
                                   {"age_ms", tof_fresh_ms ? std::max<int64_t>(0, now - tof_fresh_ms) : -1},
                                   {"errors", tof_errors}}
                             : json(nullptr)},
                // Attitude.  ⚠ There is NO ground truth on hardware, so the honest health
                // reading is disagree_deg -- accelerometer-only gravity-up against the
                // fused estimate, both computable on-robot.  When the filter is working
                // the two sit close and the fused one is visibly steadier; when it is
                // not, this is the number that says so.  a_norm_g is the second check
                // (1.0004 g at rest, measured) and it is what the trust gate rides on.
                // ⚠ gyro_bias is RE-ESTIMATED every run and deliberately never stored:
                // measured drift is 0.02-0.036 dps within a session but 0.15 dps across
                // 5 C, so a stored constant goes stale inside one warm-up.  bias_valid
                // false means the seed window is still filling -- and seeding assumes the
                // robot is STILL, so heading is not trustworthy until it flips true.
                {"imu", imu ? json{{"ok", imu_ok},
                                   {"up_fused", imu_s.up_fused}, {"up_accel", imu_s.up_accel},
                                   {"accel_body", imu_s.accel_body}, {"gyro_body", imu_s.gyro_body},
                                   {"a_norm_g", imu_s.a_norm_g}, {"trust", imu_s.trust},
                                   {"disagree_deg", imu_s.disagree_deg},
                                   {"temp_c", imu_s.temp_c}, {"dt_s", imu_s.dt_s},
                                   {"gyro_bias_dps", imu_s.gyro_bias_dps},
                                   {"bias_valid", imu_s.bias_valid},
                                   {"bias_samples", imu_s.bias_samples},
                                   {"who_am_i", imu->who_am_i()},
                                   {"errors", imu_errors}}
                             : json(nullptr)},
                // Cost of the loop, in the units a control loop cares about: per cent of
                // the tick BUDGET, with the tail (max) beside the middle because a mean
                // is blind to the spike that actually misses a deadline.  wall vs cpu
                // separates "blocked on the bus" from "out of compute" (ResourceMonitor).
                {"cpu", {{"budget_ms", tick_cost.budget_ms}, {"n", tick_cost.n},
                         {"wall_p50", tick_cost.wall_p50}, {"wall_p95", tick_cost.wall_p95},
                         {"wall_max", tick_cost.wall_max},
                         {"cpu_p50", tick_cost.cpu_p50}, {"cpu_p95", tick_cost.cpu_p95},
                         {"cpu_max", tick_cost.cpu_max},
                         {"proc_pct", host.proc_cpu_pct}, {"temp_c", host.cpu_temp_c}}},
                {"mem", {{"rss_mb", host.rss_mb}, {"swap_mb", host.swap_mb},
                         {"avail_mb", host.mem_avail_mb}, {"majflt", host.majflt},
                         {"growth_mb_min", host.rss_growth_mb_per_min}}},
                {"servos", servos}};
    }
};

std::atomic<bool> g_run{true};
void on_sig(int) { g_run = false; }

// Synthetic tick load — an instrument CALIBRATOR, not a feature.  A meter is checked
// against a known signal (a test tone), never against an uncontrolled real one, and
// there is no way to run a picrawler config on this machine yet anyway: that needs
// ogma_host (H3), and the deployed config does not yet run on legal inputs.
//   cpu_us   spins real FP work   -> drives wall ~= cpu   (compute-bound)
//   block_us sleeps               -> drives wall >> cpu   (blocked)
// Having both is what proves the wall-vs-cpu VERDICT, not merely the meter's range.
void burn_cpu_us(int us) {
    if (us <= 0) return;
    timespec a; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &a);
    const int64_t target_ns = int64_t(us) * 1000;
    static volatile double sink = 0.0;
    double x = 1.000001;
    for (;;) {
        for (int i = 0; i < 256; ++i) { x = x * 1.0000001 + 1e-9; sink = sink + x; }
        timespec b; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &b);
        if ((b.tv_sec - a.tv_sec) * 1000000000L + (b.tv_nsec - a.tv_nsec) >= target_ns) return;
    }
}

void tick_thread(State& S) {
    timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    const long period_ns = long(1e9 / TICK_HZ);
    int64_t win_start = mono_ms(); int win_ticks = 0;
    while (g_run) {
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; ++next.tv_sec; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        const long late_ns = (now.tv_sec - next.tv_sec) * 1000000000L + (now.tv_nsec - next.tv_nsec);
        // The budget span starts at WAKE, not after the lock: waiting for the mutex
        // spends the tick's budget just as surely as working does.
        timespec cpu0; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu0);
        std::lock_guard<std::mutex> lk(S.m);
        if (late_ns > period_ns) ++S.overruns;
        const int64_t ms = mono_ms();
        try {
            S.service_pose_move();
            bool any_armed = false; for (int c = 0; c < ServoDriver::N; ++c) any_armed |= S.driver.armed(c);
            const bool client_fresh = ms - S.last_client_ms <= DEADMAN_MS;
            if (any_armed && !client_fresh && !S.deadman_tripped && ms >= S.rescue_until_ms) {
                S.deadman_tripped = true; ++S.watchdog_trips;
                S.record("deadman", {{"trips", S.watchdog_trips}});
                S.rescue("deadman");                                       // the safe action, once
            }
            if (client_fresh) S.deadman_tripped = false;
            if (client_fresh || ms < S.rescue_until_ms || S.pose_move_active)   // keeps the driver watchdog fed
                for (int c = 0; c < ServoDriver::N; ++c)
                    if (S.driver.armed(c)) S.driver.command(c, S.driver.target_us(c));
            if (S.cal_ch >= 0 && ms > S.cal_until_ms) S.end_cal("timeout");
            S.driver.tick();
        } catch (const std::exception& e) {
            // A NACK that survived the bus retries.  Count it, log it, carry on: the next
            // tick rewrites every armed pulse anyway.  Dying here left the robot limp and
            // the operator disconnected mid-calibration (2026-08-28).
            ++S.bus_errors;
            if (S.bus_errors % 50 == 1) S.record("bus_error", {{"where", "tick"}, {"what", e.what()}, {"count", S.bus_errors}});
            if (S.bus_errors % 20 == 0 && S.mcu && S.mcu->ok()) {          // SunFounder's own recovery for a stuck MCU
                S.mcu->reset(); S.driver.forget_timers();
                S.record("mcu_reset", {{"why", "persistent bus errors"}, {"count", S.bus_errors}});
            }
        }
        if (S.driver.watchdog_tripped()) { S.armed_ch = -1; S.end_cal("watchdog"); }   // after a rescue has landed
        if (S.load_cpu_us > 0) burn_cpu_us(S.load_cpu_us);
        if (S.load_block_us > 0) { timespec b{0, long(S.load_block_us) * 1000L}; nanosleep(&b, nullptr); }
        timespec w1, c1;
        clock_gettime(CLOCK_MONOTONIC, &w1);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        if (S.budget.sample((w1.tv_sec - now.tv_sec) * 1000000000L + (w1.tv_nsec - now.tv_nsec),
                            (c1.tv_sec - cpu0.tv_sec) * 1000000000L + (c1.tv_nsec - cpu0.tv_nsec)))
            S.tick_cost = S.budget.last();
        if (++win_ticks >= 100) { S.tick_hz_meas = win_ticks * 1000.0 / double(ms - win_start); win_start = ms; win_ticks = 0; }
    }
}

// The IMU sampler.  ⚠ S.imu (the driver, and with it the filter state) is touched ONLY
// here -- construction finishes before this thread starts and nothing else calls sample().
// The mutex is taken just long enough to publish the finished sample, so the 50 Hz servo
// tick never waits on SPI.
void imu_thread(State& S) {
    if (!S.imu) return;
    const auto period = std::chrono::microseconds(1000000 / 225);
    auto next = std::chrono::steady_clock::now();
    while (g_run) {
        next += period;
        ImuSample s{};
        const bool ok = S.imu->sample(s);
        {
            std::lock_guard<std::mutex> lk(S.m);
            if (ok) { S.imu_s = s; S.imu_ok = true; }
            else    { S.imu_ok = false; ++S.imu_errors; }
        }
        std::this_thread::sleep_until(next);
    }
}

void telemetry_thread(State& S, void* pub) {
    HostStats hs;
    int host_poll = 0;
    while (g_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Read /proc BEFORE taking the lock.  It costs ~100 us; doing it while the
        // tick thread waits would make the instrument a cause of what it measures.
        HostSample fresh; bool have_fresh = false;
        if (++host_poll >= 10) { host_poll = 0; fresh = hs.sample(); have_fresh = fresh.ok; }
        std::string body;
        { std::lock_guard<std::mutex> lk(S.m); if (have_fresh) S.host = fresh;
          json f = S.frame(); body = f.dump(); S.record("telemetry", f); }
        // ONE frame: "bench " + JSON.  ZMQ_CONFLATE on the subscriber does not support
        // multi-part messages, and SUB filtering is a prefix match, so the topic rides in-band.
        const std::string msg = "bench " + body;
        zmq_send(pub, msg.data(), msg.size(), ZMQ_DONTWAIT);
    }
}

bool load_map(State& S, const std::string& path, std::string& why) {   // caller holds m
    std::ifstream f(path); if (!f) { why = "cannot read " + path; return false; }
    try { S.map = json::parse(f); } catch (const std::exception& e) { why = std::string("bad json: ") + e.what(); return false; }
    if (!S.map.contains("servos") || !S.map["servos"].is_array()) S.map["servos"] = json::array();
    for (auto& s : S.map["servos"]) { int c = s.value("ch", -1); if (c >= 0 && c < ServoDriver::N && c != S.cal_ch) S.driver.set_limits(c, S.oper_limits(c)); }
    return true;
}

json handle(State& S, const json& req) {   // caller holds m
    const std::string verb = req.value("verb", "");
    const int64_t now = mono_ms();
    S.last_client_ms = now;
    auto ok  = [](json extra = json::object()) { extra["ok"] = true; return extra; };
    auto err = [](const std::string& e) { return json{{"ok", false}, {"error", e}}; };
    auto ch_of = [&](const json& r, int& ch) -> bool { ch = r.value("ch", -1); return ch >= 0 && ch < ServoDriver::N; };
    int ch = -1;

    if (verb == "ping")   return ok({{"t_mono_ms", now}});
    if (verb == "status") { json f = S.frame(); f["map"] = S.map; return ok(f); }
    if (verb == "limp")   { S.rescue("verb"); return ok({{"rescue_pose", S.has_rescue() ? json(S.rescue_name) : json(nullptr)}}); }
    if (verb == "mode")   return req.value("mode", "") == "bench" ? ok({{"mode", "bench"}}) : err("only 'bench' exists here; the brain's modes live in ogma_host");
    if (verb == "servo.set" || verb == "pose.set") {
        // ⚠ Refuse to RE-LOAD a rail that just dipped.  The servos are the load, so the
        // back-off has to gate the two verbs that drive them; without this the guard drops
        // to rescue and the very next command puts the load straight back on.
        if (now < S.rail_guard_until_ms)
            return err("5 V rail under-voltage — backing off, retry shortly");
    }
    if (verb == "servo.set") {
        if (!ch_of(req, ch)) return err("bad ch");
        if (S.low_battery) return err("battery low — limp until it recovers above " + std::to_string(VBAT_RECOVER_V) + " V");
        const int us = req.value("us", -1);
        if (us < FULL_MIN_US || us > FULL_MAX_US) return err("us out of 500-2500");
        if (S.cal_ch >= 0 && S.cal_ch != ch) return err("channel " + std::to_string(S.cal_ch) + " is widened: one servo at a time until cal.end");
        S.armed_ch = ch;
        S.driver.command(ch, us);
        return ok({{"clamped_us", S.driver.target_us(ch)}});
    }
    if (verb == "servo.limits") {
        if (!ch_of(req, ch)) return err("bad ch");
        const int lo = req.value("min_us", OPER_MIN_US), hi = req.value("max_us", OPER_MAX_US);
        if (lo < FULL_MIN_US || hi > FULL_MAX_US || lo >= hi) return err("limits must satisfy 500 <= min < max <= 2500");
        json& e = S.map_entry(ch); e["min_us"] = lo; e["max_us"] = hi;
        if (S.cal_ch != ch) S.driver.set_limits(ch, {lo, hi});
        return ok();
    }
    if (verb == "tof.stall") {
        // ⚠ FAULT INJECTION, and it is here because the alternative is worse.  The ToF
        // drops out of continuous mode at roughly 1 in 600 pose moves, so verifying the
        // recovery path by WAITING for it needs ~1800 trips for 95 % confidence — hours of
        // servo cycling to exercise a few register writes.  Stopping ranging on demand
        // reproduces the observed failure exactly (the part stays addressable, it simply
        // stops producing measurements) and makes the recovery testable in seconds.
        //
        // Bench-only, like every verb on this channel: it cannot be reached by a brain,
        // and it needs `confirm` so it cannot be tripped by a fat-fingered dashboard.
        if (!S.tof) return err("no ToF");
        if (!req.value("confirm", false)) return err("tof.stall needs confirm=true — it deliberately breaks the sensor");
        try { S.tof->stop_continuous(); }
        catch (const std::exception& e) { return err(std::string("stop_continuous: ") + e.what()); }
        S.record("tof_stall_injected", {{"by", "verb"}});
        return ok();
    }
    if (verb == "limits.set") {
        // Bench-only: slew and pose stagger at runtime, so a ceiling sweep does not need a
        // service restart per point.  These were command-line flags only (--normal-slew,
        // --pose-stagger-ms), which made a sweep a sequence of restarts and lost the
        // daemon's own state between points.
        //
        // ⚠ THESE ARE THE BROWNOUT LEVERS.  Raising slew and shrinking the stagger is
        // exactly what took the Pi down on 2026-08-29; after Mod A the failure lands on the
        // HAT MCU instead, which is recoverable, but it IS still a failure.  Needs confirm.
        if (!req.value("confirm", false))
            return err("limits.set needs confirm=true — slew and stagger are the brownout levers");
        // ⚠ TWO SLEWS, AND POSE MOVES USE THE OTHER ONE.  begin_pose_move() calls
        // set_slew_us_per_tick(g_pose_slew_us) on every pose.set, so setting the NORMAL slew
        // (what the brain's own commands ride) and then driving poses tests nothing: the
        // pose path overwrites it immediately.  The first ceiling sweep did exactly that and
        // produced nine identical rows -- same current, same duration, 0.02 V of pack sag
        // across the whole ladder -- reading as "no failure up to slew 2000" when it was one
        // test run nine times at 600 us/s.  A sweep of the pose path must set pose_slew_us.
        if (req.contains("slew_us")) {
            const int v = req.value("slew_us", g_normal_slew_us);
            if (v < 1 || v > 4000) return err("slew_us out of 1-4000");
            g_normal_slew_us = v;
            S.driver.set_slew_us_per_tick(v);
        }
        if (req.contains("pose_slew_us")) {
            const int v = req.value("pose_slew_us", g_pose_slew_us);
            if (v < 1 || v > 4000) return err("pose_slew_us out of 1-4000");
            g_pose_slew_us = v;
        }
        if (req.contains("stagger_ms")) {
            const int v = req.value("stagger_ms", g_pose_stagger_ticks * 20);
            if (v < 0 || v > 2000) return err("stagger_ms out of 0-2000");
            g_pose_stagger_ticks = v / 20;
        }
        S.record("limits.set", {{"slew_us", g_normal_slew_us}, {"pose_slew_us", g_pose_slew_us},
                                {"stagger_ms", g_pose_stagger_ticks * 20}});
        return ok({{"slew_us", g_normal_slew_us}, {"pose_slew_us", g_pose_slew_us},
                   {"stagger_ms", g_pose_stagger_ticks * 20},
                   {"note", "pose.set uses pose_slew_us; slew_us is what the brain's commands ride"}});
    }
    if (verb == "ext5v.rate") {
        // Bench-only: raise the EXT5V sample rate for a measurement, then put it back.
        // Below 500 ms each sample is also written to the JSONL as its own record.
        // ⚠ THE FLOOR IS 100 ms AND IT IS NOT NEGOTIABLE HERE.  The deadline is checked
        // inside frame(), which the telemetry thread calls at 10 Hz -- so asking for 50 ms
        // delivers 10 Hz.  The first run of the separation test asked for 50 and got 10
        // without being told, which is the same class of lie as a starved sampler: a number
        // that is accepted and then quietly not honoured.  Refuse it instead.
        const int ms = req.value("ms", int(EXT5V_POLL_MS));
        if (ms < 100 || ms > 60000)
            return err("ms out of 100-60000 — the floor is frame()'s 10 Hz call rate, "
                       "not the cost of the call (which is 2.9 ms)");
        g_ext5v_poll_ms = ms;
        S.record("ext5v.rate", {{"ms", ms}});
        return ok({{"ms", g_ext5v_poll_ms}, {"effective_hz", 1000.0 / std::max<int64_t>(ms, 100)},
                   {"recording_each_sample", ms < 500}});
    }
    if (verb == "rail.inject") {
        // ⚠ FAULT INJECTION.  RailGuard's LOGIC has unit tests; what those cannot reach is
        // the WIRING — that the poll actually calls it, that rescue actually fires, that
        // servo.set/pose.set/cal.begin actually refuse afterwards.  Waiting for a real dip
        // to check that means waiting for the failure that hard-resets the Pi.
        //
        // Bits are OR'd into the polled mask, so this cannot CLEAR a real bit, only add.
        // Setting bits=0 removes the injection; it does not reset the guard's baseline,
        // because the baseline having moved is exactly what a real event leaves behind.
        if (!req.value("confirm", false)) return err("rail.inject needs confirm=true — it commands the rescue pose");
        const auto bits = req.value("bits", 0u);
        if (bits & 0xFFF00000u) return err("bits outside the throttle mask (0x000FFFFF)");
        S.rail_inject = bits;
        S.record("rail_inject", {{"bits", bits}});
        return ok({{"rail_inject", S.rail_inject}, {"baseline", S.rail_guard.baseline()},
                   {"note", "OR'd into the next ~1 Hz poll"}});
    }
    if (verb == "cal.begin") {
        if (!ch_of(req, ch)) return err("bad ch");
        if (S.low_battery) return err("battery low — no calibration until it recovers");
        if (now < S.rail_guard_until_ms) return err("5 V rail under-voltage — backing off");
        if (S.cal_ch >= 0 && S.cal_ch != ch) return err("channel " + std::to_string(S.cal_ch) + " is already widened; cal.end first");
        for (int c = 0; c < ServoDriver::N; ++c) if (c != ch && S.driver.armed(c)) { S.driver.limp_all(); S.record("one-at-a-time", {{"why", "cal.begin"}, {"ch", ch}}); break; }
        S.cal_ch = ch; S.cal_until_ms = now + CAL_TIMEOUT_MS;
        S.driver.set_limits(ch, {FULL_MIN_US, FULL_MAX_US});
        S.record("cal.begin", {{"ch", ch}, {"until_ms", S.cal_until_ms}});
        return ok({{"until_ms", S.cal_until_ms}});
    }
    if (verb == "load") {
        // ⚠ Interlock: a load at or above the budget starves the servo refresh and
        // trips the driver watchdog, which commands the rescue pose — the robot MOVES.
        // Refuse while anything is armed; this is a bench instrument, not a live knob.
        bool any_armed = false; for (int c = 0; c < ServoDriver::N; ++c) any_armed |= S.driver.armed(c);
        const int cpu_us   = std::clamp(req.value("cpu_us",   0), 0, 50000);
        const int block_us = std::clamp(req.value("block_us", 0), 0, 50000);
        if ((cpu_us > 0 || block_us > 0) && any_armed)
            return err("refusing a synthetic load while servos are armed: it can trip the watchdog into a rescue move");
        S.load_cpu_us = cpu_us; S.load_block_us = block_us;
        S.record("load", {{"cpu_us", cpu_us}, {"block_us", block_us}});
        return ok({{"cpu_us", cpu_us}, {"block_us", block_us}, {"budget_ms", 1000.0 / TICK_HZ}});
    }
    if (verb == "cal.end") { S.end_cal("verb"); return ok(); }
    if (verb == "cal.map") {
        if (!ch_of(req, ch)) return err("bad ch");
        const std::string phys = req.value("physical", ""), joint = req.value("joint", "");
        const char* sim = sim_leg_for(phys);
        if (!*sim) return err("physical must be FL/FR/RL/RR");
        if (joint != "hip1" && joint != "hip2" && joint != "knee") return err("joint must be hip1/hip2/knee");
        json& e = S.map_entry(ch);
        e["physical"] = phys; e["joint"] = joint; e["sim_leg"] = sim;
        e["sign"] = req.value("sign", 1) < 0 ? -1 : 1;
        e["origin_us"] = req.value("origin_us", 1500);
        if (req.contains("min_us")) e["min_us"] = req["min_us"];
        if (req.contains("max_us")) e["max_us"] = req["max_us"];
        S.record("cal.map", e);
        return ok();
    }
    if (verb == "cal.save") {
        const std::string path = req.value("path", S.map_path);
        S.map["saved_at"] = utc_now(); S.map["body"] = S.body;
        std::ofstream f(path); if (!f) return err("cannot write " + path);
        f << S.map.dump(2) << '\n';
        S.record("cal.save", {{"path", path}});
        return ok({{"path", path}});
    }
    if (verb == "cal.load") {
        const std::string path = req.value("path", S.map_path);
        std::string why;
        if (!load_map(S, path, why)) return err(why);
        return ok({{"path", path}, {"map", S.map}});
    }
    if (verb == "pose.set") {
        if (S.cal_ch >= 0) return err("channel " + std::to_string(S.cal_ch) + " is widened: cal.end before a pose");
        if (S.low_battery) return err("battery low");
        if (!req.contains("us") || !req["us"].is_array() || req["us"].size() != size_t(ServoDriver::N)) return err("us must be an array of 12");
        std::vector<std::pair<int,int>> targets; json listed = json::array();
        for (int c = 0; c < ServoDriver::N; ++c) {
            const int us = req["us"][c].is_number() ? req["us"][c].get<int>() : -1;
            if (us < 0) { listed.push_back(nullptr); continue; }           // null = leave this channel as it is
            if (us < FULL_MIN_US || us > FULL_MAX_US) return err("us[" + std::to_string(c) + "] out of 500-2500");
            targets.push_back({c, us}); listed.push_back(us);
        }
        S.begin_pose_move(targets);                                       // staggered + gentle: protects the Pi's rail
        S.armed_ch = -1;
        S.record("pose.set", {{"us", listed}, {"stagger_ticks", g_pose_stagger_ticks}, {"slew", g_pose_slew_us}});
        return ok({{"us", listed}, {"staggered", true}, {"eta_ms", int(targets.size()) * g_pose_stagger_ticks * 20 + 2000}});
    }
    if (verb == "pose.save") {
        const std::string name = req.value("name", "");
        if (name.empty() || name.size() > 40) return err("name required (<= 40 chars)");
        if (!req.contains("us") || !req["us"].is_array() || req["us"].size() != size_t(ServoDriver::N)) return err("us must be an array of 12");
        S.poses[name] = {{"us", req["us"]}, {"saved_at", utc_now()}};
        std::ofstream f(S.poses_path); if (!f) return err("cannot write " + S.poses_path);
        f << S.poses.dump(2) << '\n';
        S.record("pose.save", {{"name", name}, {"us", req["us"]}});
        return ok({{"name", name}, {"count", S.poses.size()}});
    }
    if (verb == "pose.list") { json names = json::array(); for (auto& [k, v] : S.poses.items()) names.push_back(k); return ok({{"poses", names}}); }
    if (verb == "pose.get") {
        const std::string name = req.value("name", "");
        if (!S.poses.contains(name)) return err("no pose '" + name + "'");
        return ok({{"name", name}, {"us", S.poses[name]["us"]}, {"saved_at", S.poses[name].value("saved_at", "")}});
    }
    if (verb == "pose.delete") {
        const std::string name = req.value("name", "");
        if (!S.poses.contains(name)) return err("no pose '" + name + "'");
        S.poses.erase(name);
        std::ofstream f(S.poses_path); if (f) f << S.poses.dump(2) << '\n';
        return ok({{"count", S.poses.size()}});
    }
    return err("unknown verb '" + verb + "'");
}

} // namespace

int main(int argc, char** argv) {
    std::string body = "measured", dev = "/dev/i2c-1", log_dir = "pi_host/log", map_path = "pi_host/calib/servo_map.json", poses_path = "pi_host/calib/poses.json";
    int rep_port = 5590, pub_port = 5591; std::string rescue_name_arg = "rescue";
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string a = argv[i];
        if (a == "--body") body = argv[i + 1]; else if (a == "--i2c") dev = argv[i + 1];
        else if (a == "--rep") rep_port = std::atoi(argv[i + 1]); else if (a == "--pub") pub_port = std::atoi(argv[i + 1]);
        else if (a == "--log-dir") log_dir = argv[i + 1]; else if (a == "--map") map_path = argv[i + 1];
        else if (a == "--poses") poses_path = argv[i + 1]; else if (a == "--rescue") rescue_name_arg = argv[i + 1];
        else if (a == "--pose-slew") g_pose_slew_us = std::max(1, std::atoi(argv[i + 1]));
        else if (a == "--pose-stagger-ms") g_pose_stagger_ticks = std::max(0, std::atoi(argv[i + 1]) / 20);
        else if (a == "--r-shunt") { g_r_shunt = std::atof(argv[i + 1]); g_r_shunt_override = true; }
        else if (a == "--tof-offset") { g_tof_offset_mm = std::atof(argv[i + 1]); g_tof_override = true; }
        else if (a == "--normal-slew") g_normal_slew_us = std::max(1, std::atoi(argv[i + 1]));
        else { std::fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
    }
    // ---- fitted constants: the calib FILE is the source, flags are the override ----
    // These lived only on ExecStart, and the checked-in unit did not carry the ToF
    // offset the live one did — so reinstalling from the repo silently dropped the belly
    // calibration while the channel went on looking healthy.  The receipt below is the
    // point: a run that does not say "sensors.json" was not using this robot's numbers.
    {
        const auto cal = ogma::hw::SensorCalib::load();
        if (!g_tof_override)     g_tof_offset_mm = cal.tof_mount_offset_mm;
        // ⚠ ONE NUMBER, TWO USES.  The boom correction's sensor-above-belly IS the mount
        // offset -- see ground_clearance_boom on why expressing it from the belly plane
        // removes the third constant.  Deriving it here rather than duplicating it in
        // sensors.json means the two cannot drift apart, which is the failure the calib
        // file was created to stop (its own header records the offset living in two places
        // and one of them silently losing it).
        g_tof_boom_above_belly_m = g_tof_offset_mm / 1000.0;
        if (!g_r_shunt_override) g_r_shunt       = cal.ina_r_shunt_ohm;
        std::printf("ogma_benchd: normal slew %d us/tick (%.2f rad/s at 545.2 us/rad)\n",
                    g_normal_slew_us, g_normal_slew_us * 50.0 / 545.2);
        std::printf("ogma_benchd: calib %s (%s) — tof_offset %.2f mm%s, r_shunt %.5f ohm%s\n",
                    cal.source.c_str(), cal.loaded ? "loaded" : "MISSING, using defaults",
                    g_tof_offset_mm, g_tof_override ? " [FLAG OVERRIDE]" : "",
                    g_r_shunt, g_r_shunt_override ? " [FLAG OVERRIDE]" : "");
    }

    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    const std::string log_path = log_dir + "/benchd_" + stamp_now() + ".jsonl";
    State S(dev, body, map_path, poses_path, log_path);
    S.rescue_name = rescue_name_arg;
    std::printf("ogma_benchd: rescue pose '%s' %s\n", S.rescue_name.c_str(), S.has_rescue() ? "loaded" : "NOT SAVED YET — limp is impossible on this HAT, save one");
    try { S.mcu = std::make_unique<McuReset>(); } catch (const std::exception& e) { std::fprintf(stderr, "benchd: no MCU reset line (%s) — limp will be register-only, which this HAT ignores\n", e.what()); }
    // Instrument only: nothing in this daemon reads the current back.  Absent part =>
    // the frame carries "ina": null and every other behaviour is unchanged.
    try {
        S.ina = std::make_unique<Ina219>(S.bus, g_r_shunt);
        S.ina->configure(ina219_telemetry_config());
        std::printf("ogma_benchd: INA219 0x40 r_shunt %.5f ohm (telemetry config)\n", g_r_shunt);
    } catch (const std::exception& e) {
        S.ina.reset();
        std::fprintf(stderr, "benchd: no INA219 (%s) — current telemetry disabled\n", e.what());
    }
    // Same contract: instrument only, absent part => "tof": null and nothing else moves.
    // init() is the whole ~80-write boot sequence; it either completes or throws, because
    // a half-configured VL53L0X ranges and is quietly wrong.
    try {
        Vl53l0xConfig tc; tc.mount_offset_mm = g_tof_offset_mm;
        S.tof = std::make_unique<Vl53l0x>(S.bus, tc);
        S.tof->init();
        S.tof->start_continuous();
        std::printf("ogma_benchd: VL53L0X 0x29 mount offset %.1f mm, budget %u us (continuous)\n",
                    g_tof_offset_mm, S.tof->timing_budget_us());
    } catch (const std::exception& e) {
        S.tof.reset();
        std::fprintf(stderr, "benchd: no VL53L0X (%s) — belly clearance disabled\n", e.what());
    }
    // Same contract again: absent or miswired part => "imu": null and nothing else moves.
    // begin() includes the WHO_AM_I check, so a bad bus fails HERE, loudly, instead of
    // producing plausible numbers that only look wrong after someone trusts them.
    {
        auto probe = std::make_unique<Icm20948>();
        std::string why;
        if (probe->begin(&why)) {
            S.imu = std::move(probe);
            std::printf("ogma_benchd: ICM-20948 SPI CE0, WHO_AM_I 0x%02X (instrument only)\n",
                        S.imu->who_am_i());
        } else {
            std::fprintf(stderr, "benchd: no ICM-20948 (%s) — attitude telemetry disabled\n",
                         why.c_str());
        }
    }
    if (!S.log) std::fprintf(stderr, "benchd: cannot open %s (continuing without the record)\n", log_path.c_str());
    { std::string why; if (load_map(S, map_path, why)) std::printf("ogma_benchd: loaded map %s (%zu servos)\n", map_path.c_str(), S.map["servos"].size()); else std::printf("ogma_benchd: no map loaded (%s)\n", why.c_str()); }
    S.record("start", {{"body", body}, {"i2c", dev}, {"rep", rep_port}, {"pub", pub_port}, {"vbat", S.hat.battery_volts()}, {"map_servos", S.map["servos"].size()}});

    void* ctx = zmq_ctx_new();
    void* rep = zmq_socket(ctx, ZMQ_REP);
    void* pub = zmq_socket(ctx, ZMQ_PUB);
    int hwm = 4; zmq_setsockopt(pub, ZMQ_SNDHWM, &hwm, sizeof hwm);
    if (zmq_bind(rep, ("tcp://*:" + std::to_string(rep_port)).c_str()) != 0 ||
        zmq_bind(pub, ("tcp://*:" + std::to_string(pub_port)).c_str()) != 0) {
        std::fprintf(stderr, "benchd: bind failed: %s\n", zmq_strerror(zmq_errno())); return 1;
    }
    std::printf("ogma_benchd: body=%s  rep :%d  pub :%d  vbat %.2f V  log %s\n", body.c_str(), rep_port, pub_port,
                S.hat.battery_volts(), log_path.c_str());
    std::fflush(stdout);

    std::thread tt(tick_thread, std::ref(S));
    std::thread tl(telemetry_thread, std::ref(S), pub);
    std::thread ti(imu_thread, std::ref(S));
    while (g_run) {
        zmq_pollitem_t items[] = {{rep, 0, ZMQ_POLLIN, 0}};
        if (zmq_poll(items, 1, 100) <= 0) continue;
        char buf[65536];
        int n = zmq_recv(rep, buf, sizeof buf - 1, 0);
        if (n < 0) continue;
        buf[std::min(n, int(sizeof buf) - 1)] = 0;
        json reply;
        try {
            json req = json::parse(buf);
            std::lock_guard<std::mutex> lk(S.m);
            try { reply = handle(S, req); }
            catch (const std::exception& e) { ++S.bus_errors; reply = {{"ok", false}, {"error", std::string("bus: ") + e.what()}}; }
            if (req.value("verb", "") != "ping") S.record("verb", {{"req", req}, {"reply", reply}});
        } catch (const std::exception& e) {
            reply = {{"ok", false}, {"error", std::string("bad request: ") + e.what()}};
        }
        std::string out = reply.dump();
        zmq_send(rep, out.data(), out.size(), 0);
    }
    tt.join(); tl.join(); ti.join();
    {
        std::lock_guard<std::mutex> lk(S.m);
        // Leave the ToF stopped rather than free-running after we are gone: the part
        // draws while it ranges, and the next daemon should meet an idle one.
        if (S.tof) { try { S.tof->stop_continuous(); } catch (const std::exception&) {} }
        S.record("shutdown", {});
    }
    zmq_close(rep); zmq_close(pub); zmq_ctx_term(ctx);
    return 0;
}
