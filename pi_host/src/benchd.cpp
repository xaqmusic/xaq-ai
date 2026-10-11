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
//
// THE BRAIN'S PATH TO THE SERVOS (2026-10-03) is not a verb on this channel either.  It is
// a separate command socket (--cmd-port, a SUB bound to LOOPBACK ONLY, so nothing off the
// robot can reach it) carrying ogma_host's 50 Hz pulse targets, applied only in the `dev`
// or `autonomous` run mode, which is set on a second loopback-only socket (--ctl-port).
// The calibration channel can see the mode, STOP the robot and resume it; it cannot set the
// mode, and it refuses a brain-rate command stream outright.  PROTOCOL.md "Run modes".
#include "ogma/hw/ServoDriver.hpp"
#include "ogma/hw/Actuation.hpp"
#include "ogma/hw/HatHealth.hpp"
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
#include <optional>
#include <set>
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
// ---- fast ADC sampling, bench only (bom §5.4 step E0) -------------------------------
// The ADC is read in frame(), which the telemetry thread calls at 10 Hz.  That is right
// for `vbat` and useless for characterising the foot FSRs: `foot_load` is a tick-rate
// channel, its divider's RC is the ONLY anti-alias filter anywhere in the chain, and the
// noise it has to reject -- servo PWM edges, the 5 V regulator -- is all above a 10 Hz
// record's 5 Hz Nyquist.  A 10 Hz log of a 50 Hz channel cannot measure what the filter
// is for.
//
// OFF by default.  At 0 nothing here executes and every existing path is untouched.
//
// ⚠ THE FLOOR IS THE TICK PERIOD, because this samples from tick_thread.  20 ms is 50 Hz
// and nothing faster can be honoured, so anything faster is REFUSED rather than accepted
// and quietly rounded -- see ext5v.rate's header for what that lie costs.
constexpr int64_t ADC_FAST_MIN_MS = 20;      // == 1000/TICK_HZ; static_assert below
int64_t g_adc_poll_ms = 0;                   // 0 = off
// ToF stall detection lives in ogma::hw::TofRecoveryPolicy (tested there).
constexpr int    CAL_TIMEOUT_MS  = 120000;
constexpr int    OPER_MIN_US     = 900;    // operating envelope until calibration narrows it
constexpr int    OPER_MAX_US     = 2100;
constexpr int    FULL_MIN_US     = 500;    // the servo's full travel — cal.begin only
constexpr int    FULL_MAX_US     = 2500;
constexpr double TICK_HZ         = 50.0;
static_assert(ADC_FAST_MIN_MS == int64_t(1000.0 / TICK_HZ),
              "adc.rate's floor is the tick period -- if TICK_HZ moves, move the floor");
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
// ---- the brain's state feed (port doc SPEC §1/§2) ------------------------------------
// ogma_host runs the brain; benchd owns the servos and the HAT's ADC, and the ADC's
// select-then-read register protocol must have exactly ONE process on it -- two
// interleaving would corrupt each other's reads.  So the brain's foot-load and commanded-
// pulse inputs come from HERE, at the tick rate, on their own PUB socket.  One frame per
// tick: "state " + JSON {seq, t, us[12] (driver output after slew, by HAT channel),
// armed (bitmask), fsr[4] (A0-A3 = physical FL, FR, RL, RR; -1 on a failed read), fsr_ok}.
// Raw channel order: the leg mapping (and the sim's leg-name mirror) belongs to the
// consumer's calibration, not to the wire.  Read-only for the subscriber: this socket
// carries no verbs, so it cannot become the brain-rate control path SPEC §1.1 forbids on
// the calibration channel.  0 = off (default): no socket, no extra bus reads, byte-identical.
// How long the HAT rail must stay below VBAT_LIMP_V before benchd limps (see frame()).
// 0 = the old instant trip.  --vbat-sustain-ms overrides.
int64_t  g_vbat_sustain_ms = 1000;
// The pulses this daemon last WROTE to the HAT, carried across restarts so a fresh benchd's
// first command ramps from where the servos are instead of jumping at full speed
// (ServoDriver::known_).  tmpfs on purpose: a reboot clears it, and after a reboot the HAT's
// state is genuinely unknown.  The boot id inside makes a stale file impossible to trust.
constexpr const char* kPulseStatePath = "/dev/shm/ogma_benchd_pulses.json";
std::string read_boot_id() {
    std::ifstream f("/proc/sys/kernel/random/boot_id"); std::string id; std::getline(f, id); return id;
}
int      g_state_pub_port = 0;
void*    g_state_pub      = nullptr;
uint64_t g_state_seq      = 0;
// ---- the brain's command path (SPEC §1.1 / §4.2) ---------------------------------------
// Both OFF by default: no socket, no new code path, and the daemon stays the bench daemon.
// LOOPBACK ONLY, by bind address: ogma_host runs on this Pi, and nothing on the network —
// the laptop's dashboard included — has any route to the servos through these.
std::string g_log_dir = "pi_host/log";
int   g_cmd_port = 0;            // SUB: "cmd " + {seq, tick, us[12]} from ogma_host, CONFLATE
int   g_ctl_port = 0;            // REP: mode.get / mode.set / stop / resume / status
void* g_cmd_sub  = nullptr;      // touched only by tick_thread once it starts
ogma::hw::brain::RunMode g_start_mode = ogma::hw::brain::RunMode::Bench;
// The calibration channel's stream guard: commanding verbs (servo.set, pose.set) per
// second.  The bench dashboard throttles slider drags to 20 Hz; a brain is 600/s.
// First-order lag on the pulse the HAT gets, in brain modes only (ServoDriver::set_output_lag).
// 0 = off, the default: byte-identical.  --servo-lag-alpha.  The sim's unloaded joint fits
// 0.22-0.28 per tick; the brain's own servo forward model is 0.2.
double g_lag_alpha = 0.0;
constexpr int     CAL_STREAM_MAX       = 30;
constexpr int64_t CAL_STREAM_WINDOW_MS = 1000;
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
    int     hat_resets = 0;           // HAT OUTAGES detected (a brownout reset, or the HAT switched off)
    // ---- HAT outage + recovery (operator, 2026-10-03: "HAT reset is going to be a common
    // theme on this robot — recover gracefully and continue the run") -------------------
    bool    hat_outage = false;       // from detection until the HAT has answered sanely for 500 ms
    int64_t last_bus_err_ms = -100000, last_garbage_ms = -100000;
    std::vector<std::pair<int,int>> recover_targets;   // (ch, pulse on the line) at the reset
    bool    recovering = false;       // staggered re-arm in progress
    bool    recover_resume = false;   // resume the brain when the re-arm lands
    int64_t recover_started_ms = 0;
    std::vector<int64_t> outage_starts;                // for the backoff: 3 in 60 s
    // ⚠ RECOVER TO A POSE, NOT TO THE SAVED PULSES, when one is set (operator, 2026-10-03).
    // Restoring the saved pulses faithfully restored whatever the brain was doing — in the run
    // that prompted this, the front-right hip1 was pinned at its limit before the reset, so the
    // robot came back circling.  A known pose (the run's start pose) resets the body; the saved
    // pulses are still used, as the place each servo RAMPS from, so nothing jumps far.
    std::string recover_pose;
    int     recoveries = 0;
    int64_t last_hat_reset_ms = -100000;
    int64_t last_mcu_reset_ms = -100000;   // when benchd itself last reset the MCU
    int  overruns = 0;
    int  bus_errors = 0;
    json last_adc = json::array({0, 0, 0, 0, 0});
    bool low_battery = false;
    // ---- tick split (instrument): where the 20 ms goes, per block, over a window --------
    struct Split { double sum = 0, max = 0; long n = 0;
                   void add(double us) { sum += us; max = std::max(max, us); ++n; }
                   json out() const { return {{"mean_us", n ? sum / n : 0.0}, {"max_us", max}, {"n", n}}; } };
    Split sp_servo, sp_adc, sp_tof, sp_servo_w, sp_adc_w, sp_tof_w;   // *_w = the window being filled
    Split sp_lock, sp_feed, sp_lock_w, sp_feed_w;   // mutex wait at wake; the whole feed block
    Split sp_pre, sp_total, sp_pre_w, sp_total_w;   // lock -> feed start; wake -> end (TickBudget's span)
    json last_saved_pulses;           // what save_known_pulses() last wrote
    int64_t vbat_low_since_ms = -1;   // start of the current below-limp stretch, -1 if none
    double  vbat_dip_min      = 99.0;
    std::string rescue_name = "rescue";      // the pose that stands in for limp on this HAT
    int64_t rescue_until_ms = 0;              // while set, the tick keeps feeding the driver
    std::vector<std::pair<int,int>> pose_queue;  // (ch, us) still to start, in order
    int pose_stagger_left = 0;
    bool pose_move_active = false;
    bool deadman_tripped = false;
    // ---- run mode, STOP, and the brain's stream (PROTOCOL.md "Run modes") -------------
    // ⚠ STOP FREEZES; IT DOES NOT MOVE THE ROBOT.  Every armed channel's target becomes the
    // pulse it is at NOW, any pose or rescue move is abandoned, and nothing commands a servo
    // until `resume`.  It is the operator's "do no more harm" key (spacebar on both
    // dashboards).  The rescue pose is a separate, deliberate act (`limp`), because a pose
    // move is itself motion and can catch a leg on whatever caused the stop.
    ogma::hw::brain::RunMode mode = ogma::hw::brain::RunMode::Bench;
    bool        stopped = false;
    std::string stop_why;
    int64_t     stopped_at_ms = 0;
    int         stops = 0;
    ogma::hw::brain::BrainAuthority auth;
    ogma::hw::brain::RateGuard cal_guard{CAL_STREAM_MAX, CAL_STREAM_WINDOW_MS};
    // brain-stream accounting, published in the frame
    long     cmd_frames = 0, cmd_applied = 0, cmd_blocked = 0, cmd_bad = 0, cmd_seq_gaps = 0;
    uint64_t cmd_last_seq = 0;
    int64_t  cmd_last_tick = -1;
    json     cmd_last_us = json::array();      // what the brain last ASKED for (pre-envelope)
    uint32_t cmd_clamped = 0;                  // channels the envelope clamped, last applied frame
    bool brain_mode() const { return mode != ogma::hw::brain::RunMode::Bench; }
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
    int64_t adc_fast_next_ms  = 0;   // fast ADC deadline; the rate is ours, not the client's
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
    // ---- FAST CAPTURE (2026-10-04): what actually browns out the HAT ---------------------
    // The telemetry config averages 128 samples on BOTH channels, so a reading lands every
    // ~140 ms: on the brain runs of 2026-10-04 ~440 readings above 2.4 A caused no reset and
    // looked exactly like the 5 that preceded one.  The reset is a faster transient than the
    // instrument.  `ina.capture` switches the part to single 12-bit conversions — sag mode,
    // current AND pack voltage, ~940 Hz; inrush mode, current only, ~1.9 kHz — and a thread
    // writes every raw sample (auto-stop at most 3600 s; the first cap, 900 s, cut a 19 min run) to
    // pi_host/log/inacap_<stamp>_<mode>.csv on CLOCK_MONOTONIC us
    // (the clock the state feed and the run recordings use).  Instrument only.
    int     cap_mode = 0;                 // 0 off, 1 inrush, 2 sag
    int64_t cap_until_ms = 0;
    std::string cap_file;
    long    cap_samples = 0, cap_clipped = 0;
    double  cap_peak_a = 0.0, cap_min_v = 99.0;
    int64_t cap_started_ms = 0;
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

    // Latch STOPPED without freezing (rescue() needs its move to run).  Callers hold m.
    void latch_stop(const char* why) {
        if (stopped) return;
        stopped = true; stop_why = why; stopped_at_ms = mono_ms(); ++stops;
        record("stop", {{"why", why}, {"mode", ogma::hw::brain::mode_name(mode)}, {"froze", false}});
    }
    // STOP: freeze every armed channel where it is now, abandon pose/rescue moves, latch.
    void stop(const char* why) {
        if (recovering) { recovering = false; recover_resume = false; record("hat_recover_cancelled", {{"why", why}}); }
        pose_queue.clear();
        if (pose_move_active) { pose_move_active = false; driver.set_slew_us_per_tick(g_normal_slew_us); }
        rescue_until_ms = 0;
        int frozen = 0;
        for (int c = 0; c < ServoDriver::N; ++c)
            if (driver.armed(c)) { driver.freeze(c); ++frozen; }     // at the pulse ON THE LINE
        const bool was = stopped;
        stopped = true;
        if (!was) { stop_why = why; stopped_at_ms = mono_ms(); ++stops; }
        record("stop", {{"why", why}, {"mode", ogma::hw::brain::mode_name(mode)},
                        {"froze", frozen}, {"already_stopped", was}});
    }
    // Returns "" on success, else why not.  Idempotent.
    std::string resume(const char* who) {
        if (!stopped) return "";
        if (low_battery) return "battery low — resume refused until it recovers above " + std::to_string(VBAT_RECOVER_V) + " V";
        const int64_t now = mono_ms();
        if (now < rail_guard_until_ms) return "5 V rail under-voltage back-off — retry shortly";
        // ⚠ EVERY CHANNEL MUST BE ARMED before the brain gets the servos.  An unknown pulse
        // would take the brain's first command at FULL servo speed; and a known-but-unarmed
        // one publishes 0 us on the state feed, so ogma_host withholds every tick and the
        // brain never drives at all (measured on the robot 2026-10-03: resume was accepted
        // and nothing happened).  Pose the robot first (e.g. pose.set rescue in bench mode).
        if (brain_mode()) {
            if (recovering) { recover_resume = true; return ""; }        // resumes when the re-arm lands
            std::string unarmed;
            for (int c = 0; c < ServoDriver::N; ++c)
                if (!driver.armed(c)) unarmed += (unarmed.empty() ? "" : ",") + std::to_string(c);
            if (!unarmed.empty()) {
                // After a HAT outage: re-arm the remembered pulses one channel at a time, and
                // resume the brain when they land.  The reply is ok with stopped still true.
                if (can_recover()) {
                    if (!hat_healthy(now)) return "the HAT is not answering — is it switched on? (wait a moment after power-on)";
                    start_recovery(true, who);
                    return "";
                }
                return "channel(s) " + unarmed + " not armed — the brain cannot see or safely drive them; "
                       "set a pose first (R in a dash run, or pose.set in bench mode)";
            }
        }
        record("resume", {{"who", who}, {"held_ms", now - stopped_at_ms}, {"why_stopped", stop_why},
                          {"mode", ogma::hw::brain::mode_name(mode)}});
        stopped = false; stop_why.clear();
        auth.grant(now);
        return "";
    }
    // Returns "" on success, else why not.
    std::string set_mode(ogma::hw::brain::RunMode m, const char* who) {
        using ogma::hw::brain::RunMode;
        if (m == mode) return "";
        if (m != RunMode::Bench) {
            if (!g_cmd_sub) return "no command socket — start benchd with --cmd-port to hear a brain";
            if (cal_ch >= 0) return "channel " + std::to_string(cal_ch) + " is widened (cal.begin): cal.end first";
        }
        const RunMode from = mode;
        // ⚠ ENTERING A BRAIN MODE LATCHES STOP.  The brain gets the servos only when the
        // operator resumes (spacebar), watching — never as a side effect of a mode change.
        // Leaving one freezes the body and starts the bench deadman's clock, so a robot
        // switched back to bench with no dashboard attached goes to rescue in ~1 s, which is
        // the bench contract.
        if (m != RunMode::Bench) stop("mode change");
        else { stop("mode change"); stopped = false; stop_why.clear(); last_client_ms = mono_ms(); }
        mode = m;
        recovering = false; recover_resume = false; recover_targets.clear();
        if (m == RunMode::Bench) recover_pose.clear();     // a run's recovery pose ends with the run
        // The output lag is a brain-mode property: calibration and pose moves stay unlagged.
        // stop() above froze every channel at its output, so this cannot jump anything.
        driver.set_output_lag(m != RunMode::Bench ? g_lag_alpha : 0.0);
        auth.grant(mono_ms());
        record("mode", {{"from", ogma::hw::brain::mode_name(from)}, {"to", ogma::hw::brain::mode_name(m)}, {"who", who}});
        return "";
    }

    void note_bus_error() { ++bus_errors; last_bus_err_ms = mono_ms(); bus_burst.error(last_bus_err_ms); }
    void note_bus_ok(int64_t t) { bus_burst.ok(t); }
    BusBurst bus_burst;                 // a HUNG MCU vs sporadic errors (HatHealth.hpp)
    int      hat_glitches = 0;          // garbage reads the re-reads did not confirm
    // ---- THE HAT's 3.3 V RAIL, estimated (2026-10-05) -----------------------------------
    // The HAT's ADC measures the battery divider (A4) against its OWN 3.3 V rail; the INA219
    // measures the same pack against its internal reference.  So
    //     rail ~= 3.3 * INA pack V / (A4 read as if the reference were 3.3 V)
    // and a sagging rail shows as A4 reading HIGH — the "garbage" 9-11 V battery reads were this
    // (ledger 2026-10-05 evening).  Instrument only.  Valid while the INA219 itself is powered
    // (it shares the rail: a deep enough sag kills both, which is itself the answer).
    double   rail_v = 0.0, rail_min_1s = 0.0, rail_min_all = 9.9;
    int64_t  rail_win_start = 0;
    double   rail_win_min = 9.9;
    long     rail_below_3v0 = 0, rail_samples = 0;
    std::optional<double> rail_from(int a4_raw) const {
        if (!ina_ok || ina_v < 5.0 || a4_raw <= 0) return std::nullopt;
        const double a4_v = a4_raw * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
        return 3.3 * ina_v / a4_v;
    }
    void note_rail(double v, int64_t t) {
        rail_v = v; ++rail_samples;
        if (v < 3.0) ++rail_below_3v0;
        rail_min_all = std::min(rail_min_all, v);
        if (t - rail_win_start >= 1000) { rail_min_1s = rail_win_min; rail_win_min = 9.9; rail_win_start = t; }
        rail_win_min = std::min(rail_win_min, v);
    }
    bool hat_healthy(int64_t now) const { return now - std::max(last_bus_err_ms, last_garbage_ms) > 500; }
    int  outages_last_60s(int64_t now) const {
        int n = 0; for (int64_t t : outage_starts) if (now - t <= 60000) ++n; return n;
    }

    // ⚠ A HAT MCU RESET IS A FAULT, NOT A LOG LINE (robot, 2026-10-03).  Servo current spikes of
    // 2.4-3.0 A browned out the HAT's own microcontroller mid-run.  Its servo timers came back
    // unprogrammed, the driver kept writing pulses into them, and the servos were driven to their
    // end stops.  So on a detected outage: remember the pulse each armed servo was last sent,
    // forget the timers, DISARM every channel (nothing is written: the servos go unpowered rather
    // than being driven anywhere) and, in a brain mode, latch STOP — which also pauses the brain.
    //
    // RECOVERY re-arms the remembered pulses ONE CHANNEL AT A TIME (the pose path's stagger):
    // each servo crosses only the little it sagged while unpowered, and never twelve at once,
    // which is the current spike that caused the reset.  Then the brain resumes, its learning
    // intact.  When it runs:
    //   - AUTONOMOUS, the stop came from the reset itself, fewer than 3 outages in 60 s:
    //     automatically, once the HAT has answered sanely for 500 ms;
    //   - otherwise when the operator resumes (SPACE).  So pause -> HAT off -> move the robot ->
    //     HAT on -> SPACE re-arms and continues; R first returns to the start pose instead.
    // One outage is one event however long it lasts: a HAT switched off for 30 s counts once.
    void on_hat_reset(const char* why, bool injected = false) {
        const int64_t now = mono_ms();
        if (hat_outage && !recovering) return;            // still the same outage
        const bool had_armed = [&] { for (int c = 0; c < ServoDriver::N; ++c) if (driver.armed(c)) return true; return false; }();
        if (had_armed && !recovering) {
            recover_targets.clear();
            for (int c = 0; c < ServoDriver::N; ++c)
                if (driver.armed(c)) recover_targets.push_back({c, driver.output_us(c)});
        }                                                 // a reset DURING recovery keeps the original targets
        hat_outage = true;
        last_hat_reset_ms = now;
        ++hat_resets;
        outage_starts.push_back(now);
        while (outage_starts.size() > 16) outage_starts.erase(outage_starts.begin());
        recovering = false;
        pose_queue.clear();
        if (pose_move_active) { pose_move_active = false; driver.set_slew_us_per_tick(g_normal_slew_us); }
        rescue_until_ms = 0;
        armed_ch = -1;
        driver.forget_timers();
        try { driver.limp_all(); } catch (...) {}
        // Seed each servo's known pulse with the one it was last sent: the recovery pose then
        // RAMPS from there at the pose slew (ServoDriver::command) instead of each servo jumping
        // to the pose at full speed from an unknown position.
        for (const auto& [c, us] : recover_targets) driver.seed_known_pulse(c, us);
        if (brain_mode()) latch_stop("HAT reset");
        const bool auto_ok = mode == ogma::hw::brain::RunMode::Autonomous && stopped && stop_why == "HAT reset"
                             && outages_last_60s(now) <= 3 && can_recover();
        recover_resume = auto_ok;
        record("hat_reset", {{"why", why}, {"injected", injected}, {"count", hat_resets},
                             {"mode", ogma::hw::brain::mode_name(mode)}, {"saved_channels", recover_targets.size()},
                             {"auto_recover", auto_ok}, {"outages_60s", outages_last_60s(now)},
                             {"action", "timers forgotten, all channels disarmed"}});
    }
    bool recover_pose_valid() const {
        return !recover_pose.empty() && poses.contains(recover_pose) && poses[recover_pose].contains("us")
               && poses[recover_pose]["us"].is_array() && poses[recover_pose]["us"].size() == size_t(ServoDriver::N);
    }
    bool can_recover() const { return recover_pose_valid() || !recover_targets.empty(); }
    // Start the staggered re-arm.  Caller holds m; the HAT must be healthy.
    bool start_recovery(bool resume_after, const char* who) {
        if (recovering) return true;
        std::vector<std::pair<int,int>> targets;
        if (recover_pose_valid()) {
            for (int c = 0; c < ServoDriver::N; ++c) {
                const json& v = poses[recover_pose]["us"][size_t(c)];
                if (v.is_number() && v.get<int>() >= FULL_MIN_US && v.get<int>() <= FULL_MAX_US) targets.push_back({c, v.get<int>()});
            }
        } else {
            targets = recover_targets;
        }
        if (targets.empty()) return false;
        begin_pose_move(targets);
        recovering = true; recover_resume = resume_after; recover_started_ms = mono_ms();
        record("hat_recover_start", {{"who", who}, {"channels", targets.size()}, {"resume_after", resume_after},
                                     {"to", recover_pose_valid() ? recover_pose : std::string("saved pulses")}});
        return true;
    }
    // Per tick (caller holds m): end an outage once the HAT is healthy, start an automatic
    // recovery, and resume when a re-arm has landed.
    void service_recovery(int64_t now) {
        if (hat_outage && hat_healthy(now)) {
            hat_outage = false;
            record("hat_back", {{"outage_ms", now - last_hat_reset_ms}});
            if (recover_resume && stopped && stop_why == "HAT reset") start_recovery(true, "auto");
        }
        if (recovering && !pose_move_active && pose_queue.empty()) {
            recovering = false;
            ++recoveries;
            record("hat_recovered", {{"took_ms", now - recover_started_ms}, {"recoveries", recoveries}});
            recover_targets.clear();
            if (recover_resume) { recover_resume = false; resume("hat recovery"); }
        }
    }

    void rescue(const char* why) {
        // In a brain mode a rescue also takes the servos from the brain until the operator
        // resumes: low battery, the rail guard and a lost stream must not hand control
        // straight back once the pose lands.
        if (brain_mode()) latch_stop(why);
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
    // Record the pulses last written to the HAT (caller holds m).  Atomic (write + rename),
    // and only on change, so the 10 Hz telemetry thread costs a tmpfs write at most.
    void save_known_pulses() {
        json us = json::array();
        for (int c = 0; c < ServoDriver::N; ++c) us.push_back(driver.last_sent_us(c));
        if (us == last_saved_pulses) return;
        const json f = {{"boot_id", read_boot_id()}, {"t_mono_ms", mono_ms()}, {"us", us}};
        const std::string tmp = std::string(kPulseStatePath) + ".tmp";
        { std::ofstream o(tmp); if (!o) return; o << f.dump(); }
        if (std::rename(tmp.c_str(), kPulseStatePath) == 0) last_saved_pulses = us;
    }

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
        // Publish the tick split for the last ~100 ms window and start a new one.
        sp_servo = sp_servo_w; sp_adc = sp_adc_w; sp_tof = sp_tof_w;
        sp_servo_w = {}; sp_adc_w = {}; sp_tof_w = {};
        sp_lock = sp_lock_w; sp_feed = sp_feed_w; sp_lock_w = {}; sp_feed_w = {};
        sp_pre = sp_pre_w; sp_total = sp_total_w; sp_pre_w = {}; sp_total_w = {};
        const int64_t now = mono_ms();
        json servos = json::array();
        for (int c = 0; c < ServoDriver::N; ++c) {
            auto lim = driver.limits(c);
            servos.push_back({{"ch", c}, {"target_us", driver.target_us(c)}, {"current_us", driver.current_us(c)},
                              {"out_us", driver.output_us(c)},
                              {"armed", driver.armed(c)}, {"at_limit_s", driver.time_at_limit_s(c)},
                              {"min_us", lim.min_us}, {"max_us", lim.max_us}});
        }
        json adc = json::array();
        try {
            for (int c = 0; c < RobotHat::N_ADC; ++c) adc.push_back(hat.adc_raw(c));
            const double v = adc[4].get<int>() * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
            if (v > 9.0) {                                                // post-reset / HAT-off garbage
                adc = last_adc;
                if (hat_outage) {
                    last_garbage_ms = mono_ms();                          // still out: extend it
                } else {
                    // ⚠ CONFIRM BEFORE DISARMING (HatHealth.hpp).  Three re-reads, 2 ms apart, under
                    // the lock (~6 ms once per event, inside the tick's budget).  A rebooting MCU
                    // answers garbage or NACKs every time; a corrupted transaction does not.
                    int bad = 0; json rr = json::array();
                    for (int k = 0; k < 3; ++k) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        try {
                            const double v2 = hat.adc_raw(4) * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
                            rr.push_back(std::round(v2 * 100) / 100);
                            if (v2 > 9.0 || v2 < 1.0) ++bad;
                        } catch (const std::exception&) { ++bad; rr.push_back(nullptr); note_bus_error(); }
                    }
                    if (garbage_confirmed(bad)) {
                        last_garbage_ms = mono_ms();
                        record("adc_garbage", {{"vbat", v}, {"rereads", rr}, {"confirmed", true}});
                        on_hat_reset("adc garbage, confirmed by re-reads");
                    } else {
                        ++hat_glitches;
                        record("hat_glitch", {{"vbat", v}, {"rereads", rr}, {"bad", bad}, {"count", hat_glitches}});
                    }
                }
            }
            else { last_adc = adc; note_bus_ok(mono_ms()); }
        } catch (const std::exception& e) {
            note_bus_error(); adc = last_adc;                      // keep the last good reading
            if (bus_errors % 50 == 1) record("bus_error", {{"where", "adc"}, {"what", e.what()}, {"count", bus_errors}});
        }
        sample_ina();
        sample_tof();
        const double vbat = adc[4].get<int>() * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
        // SPEC 4.6 — low-voltage auto-safe.  The HAT powers the Pi too, so a dying pack
        // takes the whole robot down; go limp early and say so.
        // ⚠ SUSTAINED, not instantaneous (operator, 2026-10-03).  With the Pi on its own BEC
        // the HAT rail can sag under servo inrush without browning the Pi out, and the HAT
        // rides a brownout far better than the Pi did.  The instant trip fired on a ~100 ms
        // inrush dip to 6.21 V on a bench supply (benchd log 2026-10-03 15:43) and threw a
        // standing robot into rescue mid-move.  So the voltage must stay below the limp line
        // for g_vbat_sustain_ms (10 Hz samples) before benchd limps and refuses arming; a dip
        // that recovers first is RECORDED (min volts, duration) so the sag stays visible
        // rather than silently absorbed.  g_vbat_sustain_ms = 0 is the old instant trip.
        if (vbat < VBAT_LIMP_V && vbat > 1.0) {
            if (vbat_low_since_ms < 0) { vbat_low_since_ms = now; vbat_dip_min = vbat; }
            vbat_dip_min = std::min(vbat_dip_min, vbat);
            if (!low_battery && now - vbat_low_since_ms >= g_vbat_sustain_ms) {
                low_battery = true; rescue("low battery");
                record("low_battery", {{"vbat", vbat}, {"limp_v", VBAT_LIMP_V},
                                       {"below_ms", now - vbat_low_since_ms}, {"min_v", vbat_dip_min},
                                       {"sustain_ms", g_vbat_sustain_ms}});
            }
        } else if (vbat_low_since_ms >= 0) {
            if (!low_battery)
                record("vbat_dip", {{"min_v", vbat_dip_min}, {"below_ms", now - vbat_low_since_ms},
                                    {"limp_v", VBAT_LIMP_V}, {"sustain_ms", g_vbat_sustain_ms}});
            vbat_low_since_ms = -1;
        }
        if (low_battery && vbat > VBAT_RECOVER_V) {
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
        // The deadman belongs to the calibration channel only (SPEC §4.2): in a brain mode
        // there is none, and the frame says so with null rather than a countdown.
        const json dm = brain_mode() ? json(nullptr)
                      : json(any_armed ? std::max<int64_t>(0, DEADMAN_MS - (now - last_client_ms)) : 0);
        json brain = nullptr;
        if (g_cmd_sub) {
            const auto& pol = auth.policy();
            brain = {{"frames", cmd_frames}, {"applied", cmd_applied}, {"blocked", cmd_blocked},
                     {"bad", cmd_bad}, {"seq_gaps", cmd_seq_gaps}, {"last_tick", cmd_last_tick},
                     {"age_ms", auth.age_ms(now)}, {"have_stream", auth.have_stream()},
                     {"holding", auth.holding()}, {"losses", auth.losses()}, {"regains", auth.regains()},
                     {"clamped_mask", cmd_clamped}, {"last_us", cmd_last_us},
                     {"hold_after_ms", pol.hold_after_ms}, {"rescue_after_ms", pol.rescue_after_ms}};
        }
        return {{"seq", ++seq}, {"t_mono_ms", now}, {"uptime_s", (now - t0_ms) / 1000.0},
                {"mode", ogma::hw::brain::mode_name(mode)},
                {"stopped", stopped}, {"stop_why", stopped ? json(stop_why) : json(nullptr)},
                {"stopped_ms", stopped ? now - stopped_at_ms : 0}, {"stops", stops},
                {"brain", brain}, {"cal_stream_refused", cal_guard.refused()},
                {"servo_lag_alpha", driver.output_lag()}, {"hat_resets", hat_resets}, {"hat_glitches", hat_glitches},
                {"rail", rail_samples ? json{{"v", rail_v}, {"min_1s", rail_min_1s}, {"min_all", rail_min_all},
                                             {"below_3v0", rail_below_3v0}, {"samples", rail_samples},
                                             {"how", "3.3 x INA pack V / A4 reading"}} : json(nullptr)},
                {"ina_capture", cap_mode ? json{{"mode", cap_mode == 1 ? "inrush" : "sag"}, {"file", cap_file},
                                                {"samples", cap_samples}, {"peak_a", cap_peak_a},
                                                {"min_v", cap_mode == 2 ? json(cap_min_v) : json(nullptr)},
                                                {"clipped", cap_clipped},
                                                {"rate_hz", now > cap_started_ms ? cap_samples * 1000.0 / double(now - cap_started_ms) : 0.0},
                                                {"ms_left", std::max<int64_t>(0, cap_until_ms - now)}}
                                         : json(nullptr)},
                {"hat", {{"outage", hat_outage}, {"healthy", hat_healthy(now)}, {"recovering", recovering},
                         {"recover_resume", recover_resume}, {"saved_channels", recover_targets.size()},
                         {"recover_pose", recover_pose_valid() ? json(recover_pose) : json(nullptr)},
                         {"outages_60s", outages_last_60s(now)}, {"recoveries", recoveries},
                         {"auto_recover", mode == ogma::hw::brain::RunMode::Autonomous},
                         {"last_reset_age_ms", hat_resets ? now - last_hat_reset_ms : -1}}},
                {"body", body}, {"vbat", vbat}, {"adc", adc}, {"armed_ch", armed_ch}, {"cal_ch", cal_ch},
                {"cal_ms_left", cal_ch >= 0 ? std::max<int64_t>(0, cal_until_ms - now) : 0},
                {"deadman_ms_left", dm}, {"watchdog_trips", watchdog_trips}, {"tick_hz", tick_hz_meas},
                {"overruns", overruns}, {"bus_errors", bus_errors},
                {"tick_split", {{"servo", sp_servo.out()}, {"adc4", sp_adc.out()}, {"tof", sp_tof.out()},
                                {"lock_wait", sp_lock.out()}, {"feed_total", sp_feed.out()},
                                {"wake_to_feed", sp_pre.out()}, {"tick_total", sp_total.out()}}},
                {"i2c_retries", {{"hat_0x14", bus.retries(0x14)}, {"tof_0x29", bus.retries(0x29)},
                                 {"ina_0x40", bus.retries(0x40)}}}, {"low_battery", low_battery}, {"rescue_pose", has_rescue() ? json(rescue_name) : json(nullptr)},
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
                // ⚠ wall_* / cpu_* are PERCENT OF budget_ms, not milliseconds (TickBudget).
                // Misread as ms on 2026-10-03, a 1.9 ms tick looked like 9.4 ms and an I2C
                // "problem" was chased that did not exist.  `units` says it in the frame.
                {"cpu", {{"units", "pct_of_budget"}, {"budget_ms", tick_cost.budget_ms}, {"n", tick_cost.n},
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
        // The brain's newest command (CONFLATE keeps one), received and parsed BEFORE the
        // lock so a malformed or slow frame never costs the bus its mutex.
        bool got_cmd = false, cmd_unparsed = false;
        json cmd;
        if (g_cmd_sub) {
            char cb[1024];
            const int n = zmq_recv(g_cmd_sub, cb, sizeof cb - 1, ZMQ_DONTWAIT);
            if (n > 4) {
                cb[std::min(n, int(sizeof cb) - 1)] = 0;
                try { cmd = json::parse(cb + 4); got_cmd = true; }      // after "cmd "
                catch (const std::exception&) { cmd_unparsed = true; }
            } else if (n >= 0) cmd_unparsed = true;
        }
        std::lock_guard<std::mutex> lk(S.m);
        { timespec lk1; clock_gettime(CLOCK_MONOTONIC, &lk1);
          S.sp_lock_w.add((lk1.tv_sec - now.tv_sec) * 1e6 + (lk1.tv_nsec - now.tv_nsec) / 1e3); }
        if (late_ns > period_ns) ++S.overruns;
        const int64_t ms = mono_ms();
        try {
            S.service_pose_move();
            S.service_recovery(ms);
            // ---- the brain's command stream (off unless --cmd-port) --------------------
            if (g_cmd_sub) {
                using ogma::hw::brain::BrainAuthority;
                std::array<int, ServoDriver::N> want{};
                bool valid = false;
                if (cmd_unparsed) ++S.cmd_bad;
                if (got_cmd) {
                    ++S.cmd_frames;
                    try {
                        const json& us = cmd.at("us");
                        if (!us.is_array() || us.size() != size_t(ServoDriver::N)) throw std::runtime_error("us must be 12");
                        // ⚠ OUT-OF-TRAVEL IS NOT MALFORMED.  The sim's knee range runs past what
                        // the servo can reach (u = -1 asks ~2836 us), so a frame is rejected
                        // only for a nonsense value; range is the driver's envelope's job.
                        valid = true;
                        for (int c = 0; c < ServoDriver::N; ++c) {
                            want[size_t(c)] = us[size_t(c)].get<int>();
                            if (want[size_t(c)] <= 0 || want[size_t(c)] > 10000) valid = false;
                        }
                        const uint64_t sq = cmd.value("seq", uint64_t(0));
                        if (S.cmd_last_seq && sq != S.cmd_last_seq + 1) ++S.cmd_seq_gaps;
                        S.cmd_last_seq = sq;
                        S.cmd_last_tick = cmd.value("tick", int64_t(-1));
                        S.cmd_last_us = us;
                    } catch (const std::exception&) { valid = false; }
                    if (!valid) ++S.cmd_bad;
                }
                // Something else owns the servos right now.  A blocked tick never applies
                // and never counts as the brain going quiet.
                const bool blocked = S.stopped || S.low_battery || ms < S.rail_guard_until_ms ||
                                     ms < S.rescue_until_ms || S.pose_move_active || S.cal_ch >= 0;
                if (valid && blocked && S.brain_mode()) ++S.cmd_blocked;
                const int regains_before = S.auth.regains();
                switch (S.auth.tick(S.mode, ms, valid, blocked)) {
                    case BrainAuthority::Event::Apply: {
                        // The envelope and the slew limit are the DRIVER's: command() clamps
                        // to the channel's calibrated range and tick() slews at the normal
                        // rate.  Nothing here can widen either.
                        uint32_t clamped = 0;
                        for (int c = 0; c < ServoDriver::N; ++c) {
                            S.driver.command(c, std::clamp(want[size_t(c)], FULL_MIN_US, FULL_MAX_US));
                            if (S.driver.target_us(c) != want[size_t(c)]) clamped |= (1u << c);
                        }
                        S.cmd_clamped = clamped;
                        ++S.cmd_applied;
                        if (S.auth.regains() != regains_before)
                            S.record("brain_stream_regained", {{"regains", S.auth.regains()}, {"tick", S.cmd_last_tick}});
                        break;
                    }
                    case BrainAuthority::Event::Hold:
                        for (int c = 0; c < ServoDriver::N; ++c) S.driver.freeze(c);
                        S.record("brain_stream_lost", {{"action", "hold"}, {"last_tick", S.cmd_last_tick},
                                                       {"losses", S.auth.losses()}});
                        break;
                    case BrainAuthority::Event::Fault:     // dev: freeze the evidence (§4.2.1)
                        S.record("brain_stream_lost", {{"action", "stop"}, {"last_tick", S.cmd_last_tick},
                                                       {"losses", S.auth.losses()}});
                        S.stop("brain stream lost");
                        break;
                    case BrainAuthority::Event::Rescue:    // autonomous: still nothing after the hold
                        S.record("brain_stream_lost", {{"action", "rescue"}, {"last_tick", S.cmd_last_tick},
                                                       {"losses", S.auth.losses()}});
                        S.rescue("brain stream lost");
                        break;
                    case BrainAuthority::Event::None: break;
                }
            }
            bool any_armed = false; for (int c = 0; c < ServoDriver::N; ++c) any_armed |= S.driver.armed(c);
            const bool client_fresh = ms - S.last_client_ms <= DEADMAN_MS;
            // The deadman is the CALIBRATION channel's (SPEC §4.2): in a brain mode the
            // dashboard is a viewer and closing it must not touch the robot.
            if (!S.brain_mode() && any_armed && !client_fresh && !S.deadman_tripped && ms >= S.rescue_until_ms) {
                S.deadman_tripped = true; ++S.watchdog_trips;
                S.record("deadman", {{"trips", S.watchdog_trips}});
                S.rescue("deadman");                                       // the safe action, once
            }
            if (client_fresh) S.deadman_tripped = false;
            // keeps the driver watchdog fed.  In a brain mode the daemon itself holds authority,
            // so armed channels are always fed — a STOPPED robot holds where it froze.
            if (S.brain_mode() || client_fresh || ms < S.rescue_until_ms || S.pose_move_active)
                for (int c = 0; c < ServoDriver::N; ++c)
                    if (S.driver.armed(c)) S.driver.command(c, S.driver.target_us(c));
            if (S.cal_ch >= 0 && ms > S.cal_until_ms) S.end_cal("timeout");
            { timespec a0, a1; clock_gettime(CLOCK_MONOTONIC, &a0);
              S.driver.tick();
              S.note_bus_ok(ms);              // the servo writes went through
              clock_gettime(CLOCK_MONOTONIC, &a1);
              S.sp_servo_w.add((a1.tv_sec - a0.tv_sec) * 1e6 + (a1.tv_nsec - a0.tv_nsec) / 1e3); }
        } catch (const std::exception& e) {
            // A NACK that survived the bus retries.  Count it, log it, carry on: the next
            // tick rewrites every armed pulse anyway.  Dying here left the robot limp and
            // the operator disconnected mid-calibration (2026-08-28).
            S.note_bus_error();
            if (S.bus_errors % 50 == 1) S.record("bus_error", {{"where", "tick"}, {"what", e.what()}, {"count", S.bus_errors}});
            // SunFounder's own recovery for a stuck MCU.  ⚠ RATE-LIMITED to one per 5 s: it fired
            // every 20 bus errors, and during a brownout (2026-10-03) that was ~20 resets in 4 s,
            // each restarting an MCU that was still coming up.
            // ⚠ A BURST WITH NOTHING GETTING THROUGH, not a running count (HatHealth.hpp): the old
            // rule fired on every 20th cumulative error and turned ~600 sporadic NACKs over a run
            // into three self-inflicted resets (2026-10-05).
            if (S.mcu && S.mcu->ok() && S.bus_burst.should_reset(ms)) {
                S.bus_burst.did_reset(ms);
                S.last_mcu_reset_ms = ms;
                S.mcu->reset();
                S.record("mcu_reset", {{"why", "HAT unresponsive: a burst of bus errors, nothing getting through"},
                                       {"count", S.bus_errors}, {"errors_1s", S.bus_burst.errors_in_window(ms)},
                                       {"since_ok_ms", S.bus_burst.since_ok(ms)}});
                S.on_hat_reset("benchd reset the MCU: HAT unresponsive");
            }
        }
        // ---- fast ADC sampling ------------------------------------------------
        // Gain-0: g_adc_poll_ms is 0 unless adc.rate asked for it, and at 0 this is one
        // integer compare.  It sits INSIDE the budget span on purpose -- the question E0
        // exists to answer is what four extra I2C reads per tick cost, so the cost has to
        // land in tick_cost and tick_hz where it can be read.  Its own try/catch so an ADC
        // NACK is not filed as a servo one, and its own `us` so the read cost is measured
        // rather than assumed.
        // The state feed reads the same four channels every tick, so when both are on the
        // reads are shared: the bus never pays twice for one sample.
        timespec feed0; clock_gettime(CLOCK_MONOTONIC, &feed0);
        S.sp_pre_w.add((feed0.tv_sec - now.tv_sec) * 1e6 + (feed0.tv_nsec - now.tv_nsec) / 1e3);
        const bool want_fast  = g_adc_poll_ms > 0 && ms >= S.adc_fast_next_ms;
        const bool want_state = g_state_pub != nullptr;
        // The brain needs the belly ToF at its own ~30 Hz, not frame()'s 10 Hz, and in
        // brain mode benchd owns the whole I2C bus (ToF included), so poll it from the
        // tick.  read_ready() is non-blocking: a tick with no new measurement costs one
        // status read.  Only with the state feed on; otherwise byte-identical.
        if (want_state) {
            timespec a0, a1; clock_gettime(CLOCK_MONOTONIC, &a0);
            // S0 of the power-budget work (2026-10-04): servo-branch current at the tick rate
            // for the brain's instrument topic.  The INA219 keeps its 128-sample telemetry
            // averaging (~68 ms), so this sees a 300-500 ms stall spike, not a 10 ms one; the
            // window rides in the frame (`ina_window_ms`) so nobody reads it as instantaneous.
            S.sample_ina();
            S.sample_tof();
            clock_gettime(CLOCK_MONOTONIC, &a1);
            S.sp_tof_w.add((a1.tv_sec - a0.tv_sec) * 1e6 + (a1.tv_nsec - a0.tv_nsec) / 1e3);
        }
        if (want_fast || want_state) {
            if (want_fast) S.adc_fast_next_ms = ms + g_adc_poll_ms;
            json a = json::array();
            bool fsr_ok = true;
            int tick_a4 = -1; double tick_rail = -1.0;
            try {
                timespec r0, r1;
                clock_gettime(CLOCK_MONOTONIC, &r0);
                for (int c = 0; c < 4; ++c) a.push_back(S.hat.adc_raw(c));   // A0-A3, the feet
                if (want_state) {                                            // A4: the 3.3 V rail estimate
                    const int a4 = S.hat.adc_raw(4);
                    tick_a4 = a4;
                    if (auto r = S.rail_from(a4)) { tick_rail = *r; S.note_rail(*r, ms); }
                }
                clock_gettime(CLOCK_MONOTONIC, &r1);
                S.sp_adc_w.add((r1.tv_sec - r0.tv_sec) * 1e6 + (r1.tv_nsec - r0.tv_nsec) / 1e3);
                if (want_fast)
                    S.record("adc_fast", {{"a", a},
                                          {"us", (r1.tv_sec - r0.tv_sec) * 1000000L
                                                 + (r1.tv_nsec - r0.tv_nsec) / 1000L}});
            } catch (const std::exception& e) {
                fsr_ok = false;
                S.note_bus_error();
                if (S.bus_errors % 50 == 1)
                    S.record("bus_error", {{"where", want_fast ? "adc_fast" : "state"},
                                           {"what", e.what()}, {"count", S.bus_errors}});
            }
            if (want_state) {
                // A failed read publishes -1s with fsr_ok=false, never a stale or zero value:
                // a confounded reading must say so in the channel.
                json fsr = json::array();
                for (int c = 0; c < 4; ++c)
                    fsr.push_back(fsr_ok && c < int(a.size()) ? a[size_t(c)] : json(-1));
                json us = json::array(), out = json::array();
                uint32_t armed = 0;
                for (int c = 0; c < ServoDriver::N; ++c) {
                    out.push_back(S.driver.output_us(c));   // the pulse on the line (== us, lag off)
                    us.push_back(S.driver.current_us(c));
                    if (S.driver.armed(c)) armed |= (1u << c);
                }
                // tof_m is benchd's raw-minus-mount-offset clearance (the arm the promoted
                // homeostat rides); tof_ms stamps the measurement so the consumer publishes
                // each NEW valid reading once, and nothing when the reading is invalid.
                // mode + stopped ride the feed so ogma_host can PAUSE the brain while the body
                // is frozen, instead of letting it learn that its actions do nothing.
                const json f = {{"seq", ++g_state_seq}, {"t", ms}, {"us", us}, {"armed", armed},
                                {"out", out},
                                {"i_a", S.ina_ok ? json(S.ina_i) : json(nullptr)}, {"ina_window_ms", 68},
                                {"a4", tick_a4}, {"rail_v", tick_rail > 0 ? json(std::round(tick_rail * 1000) / 1000) : json(nullptr)},
                                {"mode", ogma::hw::brain::mode_name(S.mode)}, {"stopped", S.stopped},
                                {"fsr", fsr}, {"fsr_ok", fsr_ok},
                                {"tof_m", S.tof_m}, {"tof_valid", S.tof_ok && S.tof_valid},
                                {"tof_ms", S.tof_last_ms}};
                const std::string msg = "state " + f.dump();
                zmq_send(g_state_pub, msg.data(), msg.size(), ZMQ_DONTWAIT);   // drops, never stalls
                timespec feed1; clock_gettime(CLOCK_MONOTONIC, &feed1);
                S.sp_feed_w.add((feed1.tv_sec - feed0.tv_sec) * 1e6 + (feed1.tv_nsec - feed0.tv_nsec) / 1e3);
            }
        }
        if (S.driver.watchdog_tripped()) { S.armed_ch = -1; S.end_cal("watchdog"); }   // after a rescue has landed
        if (S.load_cpu_us > 0) burn_cpu_us(S.load_cpu_us);
        if (S.load_block_us > 0) { timespec b{0, long(S.load_block_us) * 1000L}; nanosleep(&b, nullptr); }
        timespec w1, c1;
        clock_gettime(CLOCK_MONOTONIC, &w1);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        S.sp_total_w.add((w1.tv_sec - now.tv_sec) * 1e6 + (w1.tv_nsec - now.tv_nsec) / 1e3);
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
          json f = S.frame(); body = f.dump(); S.record("telemetry", f); S.save_known_pulses(); }
        // ONE frame: "bench " + JSON.  ZMQ_CONFLATE on the subscriber does not support
        // multi-part messages, and SUB filtering is a prefix match, so the topic rides in-band.
        const std::string msg = "bench " + body;
        zmq_send(pub, msg.data(), msg.size(), ZMQ_DONTWAIT);
    }
}

// The fast INA219 capture (see State::cap_*).  Takes the bus mutex only for each read (one or
// two register transactions, ~0.15-0.3 ms), so the servo tick waits at most one read; the file
// is written outside the lock.  Paced at the conversion time on an absolute deadline.
void ina_capture_thread(State& S) {
    std::FILE* f = nullptr;
    std::vector<char> buf;
    int mode = 0;
    auto next = std::chrono::steady_clock::now();
    while (g_run) {
        bool active; int64_t until; std::string file;
        { std::lock_guard<std::mutex> lk(S.m); active = S.cap_mode != 0; until = S.cap_until_ms; file = S.cap_file; mode = S.cap_mode; }
        if (!active) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
        if (!f) {
            f = std::fopen(file.c_str(), "w");
            if (f) {
                std::lock_guard<std::mutex> lk(S.m);
                std::fprintf(f, "# ina219 %s capture: r_shunt %.6f ohm, pga full scale %.3f V, conv_us %d, clock CLOCK_MONOTONIC us\n"
                                "t_us,shunt_raw,bus_raw,a4_raw\n", mode == 1 ? "inrush" : "sag", S.ina->r_shunt(),
                             Ina219::pga_full_scale_v(S.ina->pga()), mode == 1 ? 532 : 1064);
            }
            next = std::chrono::steady_clock::now();
        }
        const auto period = std::chrono::microseconds(mode == 1 ? 532 : 1064);
        const int64_t now_ms = mono_ms();
        if (now_ms >= until) {                       // done: telemetry config back, close
            {
                std::lock_guard<std::mutex> lk(S.m);
                try { S.ina->configure(ina219_telemetry_config()); } catch (...) {}
                S.record("ina_capture_done", {{"file", S.cap_file}, {"samples", S.cap_samples},
                                              {"seconds", (now_ms - S.cap_started_ms) / 1000.0},
                                              {"peak_a", S.cap_peak_a}, {"min_v", mode == 2 ? json(S.cap_min_v) : json(nullptr)},
                                              {"clipped", S.cap_clipped}});
                S.cap_mode = 0;
            }
            if (f) { std::fclose(f); f = nullptr; }
            continue;
        }
        int16_t sh = 0; uint16_t bu = 0; bool okr = true; int a4 = -1;
        static long cap_k = 0;
        {
            std::lock_guard<std::mutex> lk(S.m);
            try {
                if (mode == 1) sh = S.ina->read_shunt_raw();
                else { const auto smp = S.ina->read(); sh = smp.shunt_raw; bu = smp.bus_raw; }
                // the 3.3 V rail: A4 on every 4th sample (~235 Hz in sag mode), so a reset's droop
                // is resolved in milliseconds without halving the current rate
                if (mode == 2 && (++cap_k % 4) == 0) {
                    try { a4 = S.hat.adc_raw(4); } catch (const std::exception&) { a4 = -2; }
                }
                ++S.cap_samples;
                const double ia = S.ina->shunt_to_amps(sh);
                S.cap_peak_a = std::max(S.cap_peak_a, ia);
                if (mode == 2) S.cap_min_v = std::min(S.cap_min_v, (bu >> 3) * Ina219::BUS_LSB_V);
                if (std::abs(int(sh)) >= Ina219::pga_clip_counts(S.ina->pga())) ++S.cap_clipped;
            } catch (const std::exception&) { okr = false; }
        }
        if (okr && f) {
            timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
            std::fprintf(f, "%lld,%d,%u,%d\n", (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000, int(sh), unsigned(bu), a4);
        }
        next += period;
        const auto now_tp = std::chrono::steady_clock::now();
        if (next < now_tp - std::chrono::milliseconds(20)) next = now_tp;   // fell behind (a long tick): resync, don't burst
        std::this_thread::sleep_until(next);
    }
    if (f) std::fclose(f);
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
    // ⚠ ONLY A CONTROLLING CLIENT FEEDS THE DEADMAN.  This refreshed on EVERY verb, so a
    // read-only observer — picrawler_dash.py polling `status` at 2 Hz — kept armed servos
    // alive with no controlling client at all: on 2026-10-03 a released `stand` never went
    // to rescue and the robot was left standing, unsupervised, on the HAT's held pulses.
    // PROTOCOL.md always said `ping` feeds it.  Read-only verbs now do not; `ping` and every
    // verb that commands or reconfigures the robot still do.  Tools that hold a pose while
    // polling `status` were updated to `ping` explicitly.
    static const std::set<std::string> kObserverVerbs = {
        "status", "pose.get", "pose.list", "pose.save", "pose.delete", "mark", "adc.rate", "ina.capture"};
    if (!kObserverVerbs.count(verb)) S.last_client_ms = now;
    auto ok  = [](json extra = json::object()) { extra["ok"] = true; return extra; };
    auto err = [](const std::string& e) { return json{{"ok", false}, {"error", e}}; };
    auto ch_of = [&](const json& r, int& ch) -> bool { ch = r.value("ch", -1); return ch >= 0 && ch < ServoDriver::N; };
    int ch = -1;

    if (verb == "ping")   return ok({{"t_mono_ms", now}});
    if (verb == "ina.capture") {
        const std::string m = req.value("mode", "sag");
        if (!S.ina) return err("no INA219");
        if (m == "off") { S.cap_until_ms = 0; return ok({{"stopping", S.cap_mode != 0}}); }   // the thread closes the file
        if (m != "sag" && m != "inrush") return err("mode must be sag, inrush or off");
        if (S.cap_mode) return err("a capture is already running: " + S.cap_file);
        const double secs = std::clamp(req.value("seconds", 60.0), 1.0, 3600.0);   // an hour: ~70 MB of sag CSV
        S.cap_mode = m == "inrush" ? 1 : 2;
        S.cap_file = g_log_dir + "/inacap_" + stamp_now() + "_" + m + ".csv";
        S.cap_samples = S.cap_clipped = 0; S.cap_peak_a = 0.0; S.cap_min_v = 99.0;
        S.cap_started_ms = now; S.cap_until_ms = now + int64_t(secs * 1000.0);
        S.ina->configure(S.cap_mode == 1 ? ina219_capture_config() : ina219_sag_config());
        S.record("ina_capture_start", {{"mode", m}, {"file", S.cap_file}, {"seconds", secs}});
        return ok({{"file", S.cap_file}, {"mode", m}, {"seconds", secs},
                   {"conv_us", S.cap_mode == 1 ? 532 : 1064}});
    }
    if (verb == "status") { json f = S.frame(); f["map"] = S.map; return ok(f); }
    if (verb == "limp")   { S.rescue("verb"); return ok({{"rescue_pose", S.has_rescue() ? json(S.rescue_name) : json(nullptr)}}); }
    // ⚠ THE CALIBRATION CHANNEL CAN SEE THE MODE AND CANNOT SET IT (SPEC §1.1: no path from
    // the dashboard to starting the brain).  The mode is set on the robot, over the
    // loopback-only control socket (--ctl-port) or at start (--mode).
    if (verb == "mode") {
        const std::string want = req.value("mode", "");
        const std::string cur = ogma::hw::brain::mode_name(S.mode);
        if (want.empty() || want == cur) return ok({{"mode", cur}});
        return err("the run mode is set on the robot (benchd --mode, or the loopback control socket), "
                   "never from the calibration channel; it is '" + cur + "'");
    }
    // STOP / resume: the operator's spacebar.  Allowed in every mode and every state — the
    // calibration channel must always be able to stop the robot.  Resume only lifts a stop;
    // it cannot change the mode, so it cannot start a brain that is not already running.
    if (verb == "stop") {
        S.stop("operator");
        return ok({{"stopped", true}, {"mode", ogma::hw::brain::mode_name(S.mode)}});
    }
    if (verb == "resume") {
        const std::string why = S.resume("operator");
        if (!why.empty()) return err(why);
        return ok({{"stopped", S.stopped}, {"recovering", S.recovering}, {"mode", ogma::hw::brain::mode_name(S.mode)}});
    }
    {
        // While the brain holds the servos, the calibration channel's commanding and
        // envelope-changing verbs are refused: two writers on one servo is a fight, and §4.4
        // forbids widening the envelope while the control path is live.
        static const std::set<std::string> kCommanding = {
            "servo.set", "pose.set", "servo.limits", "cal.begin", "cal.map", "cal.load",
            "limits.set", "load", "rail.inject", "tof.stall"};
        if (S.brain_mode() && kCommanding.count(verb))
            return err(std::string("refused in '") + ogma::hw::brain::mode_name(S.mode) +
                       "' mode: the brain holds the servos (STOP, limp and status still work)");
        if (S.stopped && (verb == "servo.set" || verb == "pose.set" || verb == "cal.begin"))
            return err("STOPPED (" + S.stop_why + ") — resume first (spacebar on the dashboard)");
        // A brain-rate stream on this channel is refused outright (SPEC §1.1).
        if ((verb == "servo.set" || verb == "pose.set") && !S.cal_guard.admit(now)) {
            if (S.cal_guard.refused() % 50 == 1)
                S.record("cal_stream_refused", {{"verb", verb}, {"refused", S.cal_guard.refused()},
                                                {"max_per_s", CAL_STREAM_MAX}});
            return err("command stream refused: more than " + std::to_string(CAL_STREAM_MAX) +
                       " commanding verbs per second is a brain-rate stream, and this is the calibration channel");
        }
    }
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
    if (verb == "mark") {
        // A labelled fence post in the local record.  Physical changes -- a resistor
        // swapped, a cap fitted, the robot moved onto carpet -- leave NO trace in the log,
        // so a sweep's segments would otherwise have to be reconstructed afterwards from
        // wall-clock notes.  That reconstruction is where a sweep silently mislabels an arm.
        // One verb, and the evidence labels itself.
        const std::string text = req.value("text", "");
        if (text.empty() || text.size() > 200) return err("text required, 1-200 chars");
        S.record("mark", {{"text", text}});
        return ok({{"text", text}});
    }
    if (verb == "adc.rate") {
        // Bench-only: sample A0-A3 from the 50 Hz tick and write each sample to the JSONL,
        // so the FSR divider can be characterised at the rate it will actually run at.
        // `ms` = 0 turns it off, which is the default and the shipping state.
        //
        // ⚠ 20 ms floor, and it is the tick period rather than the cost of the read: this
        // runs once per tick, so 20 ms is 50 Hz and that is the end of it.  Asking for 10
        // and silently getting 20 is the failure ext5v.rate refuses, so refuse it here too.
        //
        // The read cost itself is NOT asserted anywhere.  Four channels is twelve ioctls on
        // a 400 kHz bus (bom §3's dtparam) -- order 1-2 ms once kernel overhead is counted,
        // but that is an estimate, which is exactly why every adc_fast record carries its
        // own measured `us`.  Read the number; do not trust this comment.
        const int ms = req.value("ms", 0);
        if (ms != 0 && (ms < int(ADC_FAST_MIN_MS) || ms > 60000))
            return err("ms must be 0 (off) or 20-60000 — the floor is the 50 Hz tick this "
                       "samples from, not the cost of the read (see each record's `us`)");
        g_adc_poll_ms = ms;
        S.record("adc.rate", {{"ms", ms}});
        return ok({{"ms", ms}, {"effective_hz", ms ? 1000.0 / ms : 0.0},
                   {"channels", "A0-A3"}, {"record_kind", "adc_fast"},
                   {"note", ms ? "each sample is its own JSONL line; watch tick_hz and "
                                 "overruns in `status` while it runs"
                               : "off — frame()'s 10 Hz read is the only ADC sampling"}});
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

// The CONTROL channel (--ctl-port, loopback only): the run mode, plus stop/resume so a
// script on the robot can do what the spacebar does.  Disjoint from handle(): no servo,
// pose or calibration verb exists here, and the mode verb exists nowhere else.
json handle_ctl(State& S, const json& req) {   // caller holds m
    const std::string verb = req.value("verb", "");
    auto ok  = [](json extra = json::object()) { extra["ok"] = true; return extra; };
    auto err = [](const std::string& e) { return json{{"ok", false}, {"error", e}}; };
    auto summary = [&]() {
        return json{{"mode", ogma::hw::brain::mode_name(S.mode)}, {"stopped", S.stopped},
                    {"stop_why", S.stopped ? json(S.stop_why) : json(nullptr)},
                    {"brain_age_ms", S.auth.age_ms(mono_ms())}, {"brain_applied", S.cmd_applied},
                    {"brain_frames", S.cmd_frames}, {"holding", S.auth.holding()},
                    {"pose_move_active", S.pose_move_active}, {"recovering", S.recovering},
                    {"hat_outage", S.hat_outage}};
    };
    if (verb == "ping" || verb == "mode.get") return ok(summary());
    if (verb == "status") { json f = S.frame(); return ok(f); }
    if (verb == "mode.set") {
        ogma::hw::brain::RunMode m;
        if (!ogma::hw::brain::parse_mode(req.value("mode", ""), m)) return err("mode must be bench, dev or autonomous");
        const std::string why = S.set_mode(m, "ctl");
        if (!why.empty()) return err(why);
        return ok(summary());
    }
    if (verb == "stop") { S.stop("ctl"); return ok(summary()); }
    // RESET: move a STOPPED brain-mode robot back to a saved pose (operator, 2026-10-03: the
    // robot froze in a bad pose and the only way out was to end the run).  The robot version
    // of the sim's body reset: the body goes home, the brain is NOT reset — it stays paused
    // (ogma_host does not tick while STOPPED) and resumes on `resume` with everything it
    // learned.  ⚠ STOPPED ONLY, and brain modes only: a pose move while the brain drives
    // would be two writers on one servo, and in bench mode pose.set already exists on the
    // calibration channel.  Staggered and gentle, the same path as pose.set; a `stop`
    // during the move abandons it.
    if (verb == "pose.recall") {
        const std::string name = req.value("name", "");
        if (!S.brain_mode()) return err("bench mode: use pose.set on the calibration channel");
        if (!S.stopped) return err("STOP first: a pose move must not fight the brain");
        if (S.low_battery) return err("battery low");
        if (mono_ms() < S.rail_guard_until_ms) return err("5 V rail under-voltage back-off — retry shortly");
        if (!S.hat_healthy(mono_ms())) return err("the HAT is not answering — is it switched on?");
        if (!S.poses.contains(name) || !S.poses[name].contains("us") || !S.poses[name]["us"].is_array() ||
            S.poses[name]["us"].size() != size_t(ServoDriver::N)) return err("no saved pose '" + name + "'");
        std::vector<std::pair<int,int>> targets;
        for (int c = 0; c < ServoDriver::N; ++c) {
            const json& v = S.poses[name]["us"][size_t(c)];
            if (v.is_number() && v.get<int>() >= FULL_MIN_US && v.get<int>() <= FULL_MAX_US) targets.push_back({c, v.get<int>()});
        }
        S.recover_targets.clear();          // the operator chose the start pose over the saved one
        S.recovering = false; S.recover_resume = false;
        S.begin_pose_move(targets);
        S.record("pose.recall", {{"name", name}, {"mode", ogma::hw::brain::mode_name(S.mode)}, {"channels", targets.size()}});
        json o = summary();
        o["eta_ms"] = int(targets.size()) * g_pose_stagger_ticks * 20 + 2000;
        return ok(o);
    }
    if (verb == "resume") {
        const std::string why = S.resume("ctl");
        if (!why.empty()) return err(why);
        return ok(summary());
    }
    // FAULT INJECTION: reset the HAT's MCU through its GPIO line — the same event a brownout
    // causes — so recovery can be proven on the robot without waiting for one.  Recorded as
    // injected: a drill must never be mistaken for a real reset in the record.
    // Which pose a HAT recovery returns to ("" = the saved pulses).  Cleared on return to bench.
    if (verb == "recover.pose") {
        const std::string name = req.value("name", "");
        if (!name.empty() && !S.poses.contains(name)) return err("no saved pose '" + name + "'");
        S.recover_pose = name;
        S.record("recover.pose", {{"name", name}});
        json o = summary(); o["recover_pose"] = name.empty() ? json(nullptr) : json(name);
        return ok(o);
    }
    if (verb == "hat.reset") {
        if (req.value("confirm", false) != true) return err("fault injection: send confirm=true");
        if (!S.mcu || !S.mcu->ok()) return err("no MCU reset line");
        S.mcu->reset();
        S.on_hat_reset("injected via ctl hat.reset", true);
        return ok(summary());
    }
    return err("unknown control verb '" + verb + "' (ping, mode.get, mode.set, stop, resume, pose.recall, recover.pose, hat.reset, status)");
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
        else if (a == "--state-pub") g_state_pub_port = std::max(0, std::atoi(argv[i + 1]));
        else if (a == "--vbat-sustain-ms") g_vbat_sustain_ms = std::max(0, std::atoi(argv[i + 1]));
        else if (a == "--cmd-port") g_cmd_port = std::max(0, std::atoi(argv[i + 1]));
        else if (a == "--servo-lag-alpha") g_lag_alpha = std::clamp(std::atof(argv[i + 1]), 0.0, 1.0);
        else if (a == "--ctl-port") g_ctl_port = std::max(0, std::atoi(argv[i + 1]));
        else if (a == "--mode") {
            if (!ogma::hw::brain::parse_mode(argv[i + 1], g_start_mode)) { std::fprintf(stderr, "--mode must be bench, dev or autonomous\n"); return 2; }
        }
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
    g_log_dir = log_dir;
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
        // ⚠ RETRIED: the probe intermittently reads WHO_AM_I = 0x00 on a restart (seen
        // 2026-10-03 and 2026-10-04) and the daemon then runs the whole session with no
        // attitude — which blinds the dash's tilt guard.  A second try after a pause has
        // been enough every time it was tried by hand.
        auto probe = std::make_unique<Icm20948>();
        std::string why;
        bool imu_up = false;
        for (int attempt = 1; attempt <= 4 && !imu_up; ++attempt) {
            imu_up = probe->begin(&why);
            if (!imu_up && attempt < 4) {
                std::fprintf(stderr, "benchd: ICM-20948 probe %d failed (%s) — retrying\n", attempt, why.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                probe = std::make_unique<Icm20948>();
            }
        }
        if (imu_up) {
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
    if (g_state_pub_port > 0) {
        void* sp = zmq_socket(ctx, ZMQ_PUB);
        int shwm = 8; zmq_setsockopt(sp, ZMQ_SNDHWM, &shwm, sizeof shwm);
        if (zmq_bind(sp, ("tcp://*:" + std::to_string(g_state_pub_port)).c_str()) != 0) {
            std::fprintf(stderr, "benchd: state-pub bind :%d failed: %s\n", g_state_pub_port,
                         zmq_strerror(zmq_errno()));
            return 1;
        }
        std::lock_guard<std::mutex> lk(S.m);   // the tick thread reads it under the lock
        g_state_pub = sp;
    }
    // The brain's command path.  ⚠ BOUND TO 127.0.0.1, never *: only a process on this Pi
    // (ogma_host) can reach the servos through it.
    void* ctl = nullptr;
    if (g_cmd_port > 0) {
        void* cs = zmq_socket(ctx, ZMQ_SUB);
        int conflate = 1, linger = 0;
        zmq_setsockopt(cs, ZMQ_CONFLATE, &conflate, sizeof conflate);   // newest command only
        zmq_setsockopt(cs, ZMQ_LINGER, &linger, sizeof linger);
        zmq_setsockopt(cs, ZMQ_SUBSCRIBE, "cmd ", 4);
        if (zmq_bind(cs, ("tcp://127.0.0.1:" + std::to_string(g_cmd_port)).c_str()) != 0) {
            std::fprintf(stderr, "benchd: cmd bind 127.0.0.1:%d failed: %s\n", g_cmd_port, zmq_strerror(zmq_errno()));
            return 1;
        }
        g_cmd_sub = cs;                        // tick_thread is not running yet
    }
    if (g_ctl_port > 0) {
        ctl = zmq_socket(ctx, ZMQ_REP);
        if (zmq_bind(ctl, ("tcp://127.0.0.1:" + std::to_string(g_ctl_port)).c_str()) != 0) {
            std::fprintf(stderr, "benchd: ctl bind 127.0.0.1:%d failed: %s\n", g_ctl_port, zmq_strerror(zmq_errno()));
            return 1;
        }
    }
    if (g_start_mode != ogma::hw::brain::RunMode::Bench) {
        std::lock_guard<std::mutex> lk(S.m);
        const std::string why = S.set_mode(g_start_mode, "--mode");
        if (!why.empty()) { std::fprintf(stderr, "benchd: --mode %s refused: %s\n", ogma::hw::brain::mode_name(g_start_mode), why.c_str()); return 2; }
    }
    // Seed the driver with the pulses a previous benchd left on the HAT (same boot only).
    {
        std::lock_guard<std::mutex> lk(S.m);
        int seeded = 0;
        std::string why = "no record (cold start)";
        std::ifstream pf(kPulseStatePath);
        if (pf) {
            try {
                const json j = json::parse(pf);
                if (j.value("boot_id", std::string()) != read_boot_id()) why = "record is from another boot";
                else {
                    const auto& us = j.at("us");
                    for (int c = 0; c < ServoDriver::N && c < int(us.size()); ++c)
                        if (us[size_t(c)].get<int>() > 0) { S.driver.seed_known_pulse(c, us[size_t(c)].get<int>()); ++seeded; }
                    why = "same boot";
                }
            } catch (const std::exception& e) { why = std::string("unreadable record: ") + e.what(); }
        }
        if (seeded)
            std::printf("ogma_benchd: servo start: %d/%d channels seeded from the last session's pulses (%s) — "
                        "first commands RAMP from there\n", seeded, ServoDriver::N, why.c_str());
        else
            std::printf("ogma_benchd: servo start: ⚠ pulses UNKNOWN (%s) — the FIRST command on each channel "
                        "moves at FULL servo speed; start from a pose near the robot's resting position\n", why.c_str());
        S.record("servo_start", {{"seeded", seeded}, {"why", why}});
    }
    std::printf("ogma_benchd: low-voltage limp below %.2f V sustained %lld ms%s\n", VBAT_LIMP_V,
                (long long)g_vbat_sustain_ms, g_vbat_sustain_ms == 0 ? " (INSTANT — legacy)" : "");
    std::printf("ogma_benchd: state feed %s\n", g_state_pub_port > 0
                ? ("ON  pub :" + std::to_string(g_state_pub_port) + "  (50 Hz: us[12], fsr[4], belly ToF)").c_str()
                : "off");
    std::printf("ogma_benchd: brain command path %s\n", g_cmd_port > 0
                ? ("ON  cmd 127.0.0.1:" + std::to_string(g_cmd_port) + (g_ctl_port > 0 ? "  ctl 127.0.0.1:" + std::to_string(g_ctl_port) : std::string("  (no ctl socket: mode fixed by --mode)"))).c_str()
                : "off (bench daemon only)");
    std::printf("ogma_benchd: servo output lag %s\n", g_lag_alpha > 0.0
                ? ("alpha " + std::to_string(g_lag_alpha) + " per tick, brain modes only").c_str() : "off");
    std::printf("ogma_benchd: MODE %s%s\n", ogma::hw::brain::mode_name(S.mode),
                S.mode == ogma::hw::brain::RunMode::Bench ? "  (calibration deadman ON)"
                : "  — STOPPED until resumed; NO calibration deadman (SPEC §4.2)");
    std::printf("ogma_benchd: body=%s  rep :%d  pub :%d  vbat %.2f V  log %s\n", body.c_str(), rep_port, pub_port,
                S.hat.battery_volts(), log_path.c_str());
    std::fflush(stdout);

    std::thread tt(tick_thread, std::ref(S));
    std::thread tl(telemetry_thread, std::ref(S), pub);
    std::thread ti(imu_thread, std::ref(S));
    std::thread tc(ina_capture_thread, std::ref(S));
    while (g_run) {
        zmq_pollitem_t items[] = {{rep, 0, ZMQ_POLLIN, 0}, {ctl, 0, ZMQ_POLLIN, 0}};
        if (zmq_poll(items, ctl ? 2 : 1, 100) <= 0) continue;
        for (int which = 0; which < (ctl ? 2 : 1); ++which) {
            if (!(items[which].revents & ZMQ_POLLIN)) continue;
            void* sock = which == 0 ? rep : ctl;
            char buf[65536];
            int n = zmq_recv(sock, buf, sizeof buf - 1, 0);
            if (n < 0) continue;
            buf[std::min(n, int(sizeof buf) - 1)] = 0;
            json reply;
            try {
                json req = json::parse(buf);
                std::lock_guard<std::mutex> lk(S.m);
                try { reply = which == 0 ? handle(S, req) : handle_ctl(S, req); }
                catch (const std::exception& e) { S.note_bus_error(); reply = {{"ok", false}, {"error", std::string("bus: ") + e.what()}}; }
                const std::string v = req.value("verb", "");
                if (v != "ping" && !(which == 1 && (v == "mode.get" || v == "status")))
                    S.record(which == 0 ? "verb" : "ctl", {{"req", req}, {"reply", reply}});
            } catch (const std::exception& e) {
                reply = {{"ok", false}, {"error", std::string("bad request: ") + e.what()}};
            }
            std::string out = reply.dump();
            zmq_send(sock, out.data(), out.size(), 0);
        }
    }
    tt.join(); tl.join(); ti.join(); tc.join();
    {
        std::lock_guard<std::mutex> lk(S.m);
        // Leave the ToF stopped rather than free-running after we are gone: the part
        // draws while it ranges, and the next daemon should meet an idle one.
        if (S.tof) { try { S.tof->stop_continuous(); } catch (const std::exception&) {} }
        S.save_known_pulses();   // the next benchd ramps from these (same boot)
        S.record("shutdown", {});
    }
    // ⚠ EVERY socket must be closed before zmq_ctx_term, which blocks until they are.  The
    // state-feed socket was not (2026-10-03): a feed-enabled benchd hung forever on SIGTERM
    // while still holding /dev/i2c-1, so no replacement could start.
    if (g_state_pub) { zmq_close(g_state_pub); g_state_pub = nullptr; }
    if (g_cmd_sub)   { zmq_close(g_cmd_sub);   g_cmd_sub = nullptr; }
    if (ctl)         { zmq_close(ctl); }
    zmq_close(rep); zmq_close(pub); zmq_ctx_term(ctx);
    return 0;
}
