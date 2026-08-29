// ogma_benchd — the calibration / validation daemon (pi_host/PROTOCOL.md).
//
// Three threads share one state under one mutex:
//   tick        50 Hz, clock_nanosleep(TIMER_ABSTIME): re-issues the armed target while the
//               client's deadman is fresh, then ServoDriver::tick() (slew, watchdog, at-limit)
//   telemetry   10 Hz: ADC reads + the frame on the PUB socket + the local JSONL record
//   main        the REP verb loop
// The bus (/dev/i2c-1) is not thread-safe; every I2C access happens under the mutex.
// There is NO verb that starts a brain here, and none will be added (SPEC §1.1).
#include "ogma/hw/ServoDriver.hpp"
#include "ogma/hw/McuReset.hpp"

#include <nlohmann/json.hpp>
#include <zmq.h>

#include <atomic>
#include <memory>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

using json = nlohmann::json;
using namespace ogma::hw;

namespace {

constexpr int    DEADMAN_MS      = 1000;
constexpr int    CAL_TIMEOUT_MS  = 120000;
constexpr int    OPER_MIN_US     = 900;    // operating envelope until calibration narrows it
constexpr int    OPER_MAX_US     = 2100;
constexpr int    FULL_MIN_US     = 500;    // the servo's full travel — cal.begin only
constexpr int    FULL_MAX_US     = 2500;
constexpr double TICK_HZ         = 50.0;
constexpr double VBAT_LIMP_V     = 6.4;    // HAT minimum is 6.0: limp and refuse arming below this
constexpr double VBAT_RECOVER_V  = 6.7;    // hysteresis: arming allowed again above this

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
    bool deadman_tripped = false;
    std::string pi_throttled = "0x0";   // vcgencmd get_throttled, polled ~1 Hz
    int  throttled_poll = 0;
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
    void rescue(const char* why) {
        end_cal(why);
        armed_ch = -1;
        if (has_rescue()) {
            const json& us = poses[rescue_name]["us"];
            for (int c = 0; c < ServoDriver::N; ++c)
                if (us[c].is_number() && us[c].get<int>() >= 0) { try { driver.command(c, us[c].get<int>()); } catch (...) {} }
            rescue_until_ms = mono_ms() + 3000;                    // keep feeding until it gets there
            record("rescue", {{"why", why}, {"pose", rescue_name}});
        } else {
            try { driver.limp_all(); } catch (...) {}
            record("limp", {{"why", why}, {"note", "no rescue pose saved; pulse 0 is ignored by this HAT"}});
        }
    }
    void limp_all(const char* why) { rescue(why); }

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
        const double vbat = adc[4].get<int>() * RobotHat::ADC_VREF / RobotHat::ADC_MAX * RobotHat::VBAT_DIV;
        // SPEC 4.6 — low-voltage auto-safe.  The HAT powers the Pi too, so a dying pack
        // takes the whole robot down; go limp early and say so.
        if (!low_battery && vbat < VBAT_LIMP_V && vbat > 1.0) {
            low_battery = true; rescue("low battery");
            record("low_battery", {{"vbat", vbat}, {"limp_v", VBAT_LIMP_V}});
        } else if (low_battery && vbat > VBAT_RECOVER_V) {
            low_battery = false; record("battery_ok", {{"vbat", vbat}});
        }
        if (++throttled_poll >= 10) {                          // once a second
            throttled_poll = 0;
            if (FILE* f = popen("vcgencmd get_throttled 2>/dev/null", "r")) {
                char b[64] = {0}; if (fgets(b, sizeof b, f)) { std::string t(b); auto eq = t.find('='); if (eq != std::string::npos) { pi_throttled = t.substr(eq + 1); while (!pi_throttled.empty() && (pi_throttled.back() == '\n' || pi_throttled.back() == '\r')) pi_throttled.pop_back(); } }
                pclose(f);
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
                {"rescue_active", mono_ms() < rescue_until_ms},
                {"pi_throttled", pi_throttled}, {"servos", servos}};
    }
};

std::atomic<bool> g_run{true};
void on_sig(int) { g_run = false; }

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
        std::lock_guard<std::mutex> lk(S.m);
        if (late_ns > period_ns) ++S.overruns;
        const int64_t ms = mono_ms();
        try {
            bool any_armed = false; for (int c = 0; c < ServoDriver::N; ++c) any_armed |= S.driver.armed(c);
            const bool client_fresh = ms - S.last_client_ms <= DEADMAN_MS;
            if (any_armed && !client_fresh && !S.deadman_tripped && ms >= S.rescue_until_ms) {
                S.deadman_tripped = true; ++S.watchdog_trips;
                S.record("deadman", {{"trips", S.watchdog_trips}});
                S.rescue("deadman");                                       // the safe action, once
            }
            if (client_fresh) S.deadman_tripped = false;
            if (client_fresh || ms < S.rescue_until_ms)                    // keeps the driver watchdog fed
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
        if (++win_ticks >= 100) { S.tick_hz_meas = win_ticks * 1000.0 / double(ms - win_start); win_start = ms; win_ticks = 0; }
    }
}

void telemetry_thread(State& S, void* pub) {
    while (g_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::string body;
        { std::lock_guard<std::mutex> lk(S.m); json f = S.frame(); body = f.dump(); S.record("telemetry", f); }
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
    if (verb == "cal.begin") {
        if (!ch_of(req, ch)) return err("bad ch");
        if (S.low_battery) return err("battery low — no calibration until it recovers");
        if (S.cal_ch >= 0 && S.cal_ch != ch) return err("channel " + std::to_string(S.cal_ch) + " is already widened; cal.end first");
        for (int c = 0; c < ServoDriver::N; ++c) if (c != ch && S.driver.armed(c)) { S.driver.limp_all(); S.record("one-at-a-time", {{"why", "cal.begin"}, {"ch", ch}}); break; }
        S.cal_ch = ch; S.cal_until_ms = now + CAL_TIMEOUT_MS;
        S.driver.set_limits(ch, {FULL_MIN_US, FULL_MAX_US});
        S.record("cal.begin", {{"ch", ch}, {"until_ms", S.cal_until_ms}});
        return ok({{"until_ms", S.cal_until_ms}});
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
        json clamped = json::array();
        for (int c = 0; c < ServoDriver::N; ++c) {
            const int us = req["us"][c].is_number() ? req["us"][c].get<int>() : -1;
            if (us < 0) { clamped.push_back(nullptr); continue; }          // null = leave this channel as it is
            if (us < FULL_MIN_US || us > FULL_MAX_US) return err("us[" + std::to_string(c) + "] out of 500-2500");
            S.driver.command(c, us); clamped.push_back(S.driver.target_us(c));
        }
        S.armed_ch = -1;
        S.record("pose.set", {{"clamped_us", clamped}});
        return ok({{"clamped_us", clamped}});
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
        else { std::fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
    }
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    const std::string log_path = log_dir + "/benchd_" + stamp_now() + ".jsonl";
    State S(dev, body, map_path, poses_path, log_path);
    S.rescue_name = rescue_name_arg;
    std::printf("ogma_benchd: rescue pose '%s' %s\n", S.rescue_name.c_str(), S.has_rescue() ? "loaded" : "NOT SAVED YET — limp is impossible on this HAT, save one");
    try { S.mcu = std::make_unique<McuReset>(); } catch (const std::exception& e) { std::fprintf(stderr, "benchd: no MCU reset line (%s) — limp will be register-only, which this HAT ignores\n", e.what()); }
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
    tt.join(); tl.join();
    { std::lock_guard<std::mutex> lk(S.m); S.record("shutdown", {}); }
    zmq_close(rep); zmq_close(pub); zmq_ctx_term(ctx);
    return 0;
}
