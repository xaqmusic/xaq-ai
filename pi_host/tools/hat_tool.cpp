// hat_tool — bench CLI over the real driver (never a bypass of it: SPEC §4.4).
//   hat_tool vbat                      battery volts via A4 (x3 divider)
//   hat_tool adc                       raw + volts for A0..A4
//   hat_tool limp                      pulse 0 on all 12 channels
//   hat_tool pulse <ch> <us> [hold_s]  slew to <us>, hold, then limp (default hold 1 s)
//   hat_tool sweep <ch> <from> <to> <step_us> [dwell_s]   STAIRCASE: hop, hold, hop... then limp
//   hat_tool ramp  <ch> <from> <to> [slew_us_per_tick]     one continuous slew-limited move, then limp
//   hat_tool ina probe [r_shunt]     INA219 at 0x40: bus V, current, and the A4 cross-check
//   hat_tool ina capture <sec> <file> [r_shunt]  shunt-only burst -> JSONL (the inrush record)
//   hat_tool limptest <ch>          arm at 1500, then hold three candidate 'limp' register values
//                                   (0, 1, 4095) for 8 s each — feel the servo: which one goes slack?
// Every servo action goes through ServoDriver: clamp, slew, watchdog, time-at-limit.
// ROBOT ON A STAND for any servo verb.
#include "ogma/hw/ServoDriver.hpp"
#include "ogma/hw/Ina219.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdlib>
#include <cmath>
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
        if (verb == "ina") {
            const std::string sub = argc > 2 ? argv[2] : "probe";
            // r_shunt is calibration data: the default is the R010 we fit inline,
            // but pass the real fitted value once it is measured on the bench.
            if (sub == "probe") {
                const double rs = argc > 3 ? std::atof(argv[3]) : 0.01;
                Ina219 ina(bus, rs);
                ina.configure(ina219_telemetry_config());
                std::this_thread::sleep_for(std::chrono::milliseconds(200));  // one 68 ms average
                const auto s2 = ina.read();
                const double a4 = hat.battery_volts();
                std::printf("INA219 0x40   r_shunt %.5f ohm   I_lsb %.1f uA   cal %u\n",
                            ina.r_shunt(), ina.current_lsb_a() * 1e6, ina.calibration_word());
                std::printf("  bus     %.3f V     shunt %+.3f mV (raw %+d)\n",
                            s2.bus_v, s2.shunt_v * 1e3, s2.shunt_raw);
                std::printf("  current %+.3f A    (chip reg %+.3f A, power %.2f W)\n",
                            s2.current_a, ina.chip_current_a(), ina.chip_power_w());
                std::printf("  A4      %.3f V     delta %+.3f V  -> %s\n", a4, s2.bus_v - a4,
                            std::fabs(s2.bus_v - a4) < 0.15 ? "AGREE (BOM 6.2 pass)" : "DISAGREE");
                if (s2.overflow)    std::printf("  ! OVF: chip current/power registers invalid\n");
                if (s2.pga_clipped) std::printf("  ! PGA CLIPPED: reading is a floor, not a measurement\n");
                return std::fabs(s2.bus_v - a4) < 0.15 ? 0 : 1;
            }
            if (sub == "capture") {
                if (argc < 5) { std::fprintf(stderr, "usage: hat_tool ina capture <sec> <file> [r_shunt]\n"); return 2; }
                const double  secs = std::atof(argv[3]);
                const char*   path = argv[4];
                const double  rs   = argc > 5 ? std::atof(argv[5]) : 0.01;
                Ina219 ina(bus, rs);
                const auto cfg = ina219_capture_config();
                ina.configure(cfg);
                const int period_us = Ina219::conversion_time_us(cfg.sadc);
                std::FILE* f = std::fopen(path, "w");
                if (!f) { std::fprintf(stderr, "cannot open %s\n", path); return 2; }
                // The record is raw counts + a timestamp.  r_shunt is written once as
                // metadata so a later bench re-fit re-derives every sample (SPEC 3).
                std::fprintf(f, "{\"kind\":\"ina219_capture\",\"r_shunt_ohm\":%.6f,"
                                "\"shunt_lsb_v\":%g,\"period_us\":%d,\"pga_clip_counts\":%d}\n",
                             rs, Ina219::SHUNT_LSB_V, period_us, Ina219::pga_clip_counts(cfg.pga));
                const auto t0 = std::chrono::steady_clock::now();
                auto next = t0;
                long n = 0; int peak = 0; bool clipped = false;
                while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
                    const int16_t raw = ina.read_shunt_raw();
                    const long us = std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::steady_clock::now() - t0).count();
                    std::fprintf(f, "{\"t_us\":%ld,\"shunt_raw\":%d}\n", us, raw);
                    if (std::abs(static_cast<int>(raw)) > std::abs(peak)) peak = raw;
                    if (std::abs(static_cast<int>(raw)) >= Ina219::pga_clip_counts(cfg.pga)) clipped = true;
                    ++n;
                    next += std::chrono::microseconds(period_us);
                    std::this_thread::sleep_until(next);
                }
                std::fclose(f);
                std::printf("%ld samples in %.1f s (%.0f Hz) -> %s\n", n, secs, n / secs, path);
                std::printf("peak %+d counts = %+.3f A\n", peak, ina.shunt_to_amps(static_cast<int16_t>(peak)));
                if (clipped) std::printf("! PGA CLIPPED -- the peak is a FLOOR.  Widen the range or fit a smaller shunt.\n");
                return 0;
            }
            std::fprintf(stderr, "usage: hat_tool ina probe|capture ...\n");
            return 2;
        }
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
            { int rc = std::system("timeout 0.05 gpioset -c gpiochip0 5=0 >/dev/null 2>&1; (gpioset -c gpiochip0 5=1 & sleep 0.4; kill $! ) 2>/dev/null"); (void)rc; }
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
