// hat_tool — bench CLI over the real driver (never a bypass of it: SPEC §4.4).
//   hat_tool vbat                      battery volts via A4 (x3 divider)
//   hat_tool adc                       raw + volts for A0..A4
//   hat_tool limp                      pulse 0 on all 12 channels
//   hat_tool pulse <ch> <us> [hold_s]  slew to <us>, hold, then limp (default hold 1 s)
//   hat_tool sweep <ch> <from> <to> <step_us> [dwell_s]   STAIRCASE: hop, hold, hop... then limp
//   hat_tool ramp  <ch> <from> <to> [slew_us_per_tick]     one continuous slew-limited move, then limp
//   hat_tool limptest <ch>          arm at 1500, then hold three candidate 'limp' register values
//                                   (0, 1, 4095) for 8 s each — feel the servo: which one goes slack?
// Every servo action goes through ServoDriver: clamp, slew, watchdog, time-at-limit.
// ROBOT ON A STAND for any servo verb.
#include "ogma/hw/ServoDriver.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdlib>
#include <string>
#include <thread>

using namespace ogma::hw;

static void run_ticks(ServoDriver& d, RobotHat& hat, int ch, int n, bool show_vbat) {
    const auto period = std::chrono::microseconds(20000);
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) {
        d.tick();
        if (show_vbat && i % 25 == 24)
            std::printf("  ch%d at %4d us   Vbat %.2f V   at-limit %.2f s\n", ch, d.current_us(ch),
                        hat.battery_volts(), d.time_at_limit_s(ch));
        next += period;
        std::this_thread::sleep_until(next);
    }
}

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: hat_tool vbat|adc|limp|pulse|sweep ...\n"); return 2; }
    const std::string verb = argv[1];
    try {
        LinuxI2cBus bus("/dev/i2c-1");
        RobotHat hat(bus);
        if (verb == "vbat") { std::printf("Vbat %.2f V\n", hat.battery_volts()); return 0; }
        if (verb == "adc") {
            for (int c = 0; c < RobotHat::N_ADC; ++c) {
                int raw = hat.adc_raw(c);
                std::printf("A%d raw %4d  %.3f V%s\n", c, raw, raw * RobotHat::ADC_VREF / RobotHat::ADC_MAX,
                            c == 4 ? "  (battery x3)" : "");
            }
            return 0;
        }
        if (verb == "limp") { hat.limp_all(); std::printf("all %d channels -> pulse 0 (limp)\n", RobotHat::N_SERVO); return 0; }
        if (verb == "limptest2" && argc >= 3) {
            // Discriminate: does the MCU default-pulse at boot?  Does stopping the TIMER release the servo?
            const int ch = std::atoi(argv[2]); const uint8_t t = uint8_t(ch / 4);
            auto arm = [&](int us, int secs) { hat.setup_servo_timer(ch); hat.set_pulse_us(ch, us); std::printf("ch%d -> %d us (%d s)\n", ch, us, secs); std::fflush(stdout); std::this_thread::sleep_for(std::chrono::seconds(secs)); };
            arm(1800, 3);
            std::printf("PHASE 1: MCU RESET with the knee at 1800 — does it SNAP TO CENTRE, stay stiff, or go floppy? (8 s)\n"); std::fflush(stdout);
            std::system("timeout 0.05 gpioset -c gpiochip0 5=0 >/dev/null 2>&1; (gpioset -c gpiochip0 5=1 & sleep 0.4; kill $! ) 2>/dev/null");
            std::this_thread::sleep_for(std::chrono::seconds(8));
            arm(1800, 3);
            std::printf("PHASE 2: timer %d ARR = 0 (period 0) — floppy? (8 s)\n", t); std::fflush(stdout);
            hat.write_raw(RobotHat::REG_ARR + t, 0);
            std::this_thread::sleep_for(std::chrono::seconds(8));
            arm(1800, 3);
            std::printf("PHASE 3: timer %d ARR = 1 (count 368 > ARR: constant level) — floppy? (8 s)\n", t); std::fflush(stdout);
            hat.write_raw(RobotHat::REG_ARR + t, 1);
            std::this_thread::sleep_for(std::chrono::seconds(8));
            arm(1800, 3);
            std::printf("PHASE 4: prescaler = 65535 (one pulse every ~3.7 s) — floppy between twitches? (8 s)\n"); std::fflush(stdout);
            hat.write_raw(RobotHat::REG_PSC + t, 0xFFFF);
            std::this_thread::sleep_for(std::chrono::seconds(8));
            hat.setup_servo_timer(ch); hat.set_pulse_us(ch, 0);
            std::printf("done (timer restored, count 0)\n");
            return 0;
        }
        if (verb == "limptest" && argc >= 3) {
            const int ch = std::atoi(argv[2]);
            hat.setup_servo_timer(ch);
            const int cands[3] = {0, 1, RobotHat::SERVO_ARR};
            const char* names[3] = {"A: count 0 (current limp)", "B: count 1 (~5 us pulse)", "C: count 4095 (always high, no edges)"};
            for (int i = 0; i < 3; ++i) {
                hat.set_pulse_us(ch, 1500);
                std::printf("ch%d -> 1500 us (armed, 3 s)  ...try to move it: it should resist\n", ch); std::fflush(stdout);
                std::this_thread::sleep_for(std::chrono::seconds(3));
                hat.set_pulse_raw(ch, cands[i]);
                std::printf("ch%d -> %s  (8 s)  ...try to move it now\n", ch, names[i]); std::fflush(stdout);
                std::this_thread::sleep_for(std::chrono::seconds(8));
            }
            hat.set_pulse_raw(ch, 0);
            std::printf("done; ch%d left at count 0\n", ch);
            return 0;
        }

        ServoDriverConfig cfg;             // driver defaults: slew 40 us/tick, watchdog 0.5 s
        ServoDriver d(hat, cfg);
        std::printf("frame %.2f Hz  slew %d us/tick  watchdog %d ticks\n", RobotHat::actual_frame_hz(),
                    cfg.slew_us_per_tick, cfg.watchdog_ticks);
        if (verb == "pulse" && argc >= 4) {
            const int ch = std::atoi(argv[2]), us = std::atoi(argv[3]);
            const double hold = argc >= 5 ? std::atof(argv[4]) : 1.0;
            std::printf("Vbat before %.2f V\n", hat.battery_volts());
            d.command(ch, us);
            // keep feeding the watchdog while we slew + hold
            const int ticks = static_cast<int>(hold * 50) + std::abs(us - d.current_us(ch)) / cfg.slew_us_per_tick + 1;
            for (int t = 0; t < ticks; t += 25) {
                d.command(ch, us);
                run_ticks(d, hat, ch, std::min(25, ticks - t), true);
            }
            d.limp_all();
            std::printf("ch%d -> limp   Vbat after %.2f V\n", ch, hat.battery_volts());
            return 0;
        }
        if (verb == "sweep" && argc >= 6) {
            const int ch = std::atoi(argv[2]), from = std::atoi(argv[3]), to = std::atoi(argv[4]);
            const int step = std::max(1, std::abs(std::atoi(argv[5])));
            const double dwell = argc >= 7 ? std::atof(argv[6]) : 0.5;
            const int dir = to >= from ? 1 : -1;
            for (int us = from; dir > 0 ? us <= to : us >= to; us += dir * step) {
                d.command(ch, us);
                const int ticks = static_cast<int>(dwell * 50) + step / cfg.slew_us_per_tick + 1;
                for (int t = 0; t < ticks; t += 25) { d.command(ch, us); run_ticks(d, hat, ch, std::min(25, ticks - t), false); }
                std::printf("ch%d %4d us   Vbat %.2f V   at-limit %.2f s\n", ch, d.current_us(ch), hat.battery_volts(), d.time_at_limit_s(ch));
            }
            d.limp_all();
            std::printf("ch%d -> limp\n", ch);
            return 0;
        }
        if (verb == "ramp" && argc >= 5) {
            const int ch = std::atoi(argv[2]), from = std::atoi(argv[3]), to = std::atoi(argv[4]);
            ServoDriverConfig rc = cfg;
            if (argc >= 6) rc.slew_us_per_tick = std::max(1, std::atoi(argv[5]));
            ServoDriver r(hat, rc);
            r.command(ch, from);
            run_ticks(r, hat, ch, 25, false);                 // settle at `from` for 0.5 s
            const int ticks = std::abs(to - from) / rc.slew_us_per_tick + 2;
            std::printf("ramp ch%d %d -> %d us at %d us/tick (%.1f s)\n", ch, from, to, rc.slew_us_per_tick, ticks / 50.0);
            for (int t = 0; t < ticks; t += 25) { r.command(ch, to); run_ticks(r, hat, ch, std::min(25, ticks - t), true); }
            run_ticks(r, hat, ch, 25, false);
            r.limp_all();
            std::printf("ch%d -> limp\n", ch);
            return 0;
        }
        std::fprintf(stderr, "bad arguments\n");
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "hat_tool: %s\n", e.what());
        return 1;
    }
}
