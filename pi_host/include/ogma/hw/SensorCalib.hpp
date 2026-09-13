#pragma once
// SensorCalib — the fitted constants for THIS physical robot, in one place.
//
// ⚠ THESE ARE MEASUREMENTS OF A DEVICE, NOT TUNING.  That is what keeps them on the
// right side of CLAUDE.md's prohibition 5 ("don't tune a constant to a signal's
// scale"): the ToF's mount offset and the INA219's shunt are properties of the
// assembled hardware, re-fitted whenever the hardware changes.
//
// WHY THIS FILE EXISTS.  Both lived on ogma_benchd's ExecStart as CLI flags, which the
// BOM doc flags as the wrong home twice over (§9.2, §3.3): invisible to anyone reading
// the documentation, and — worse — the CHECKED-IN systemd unit did not carry
// `--tof-offset 64.8` while the live one did, so reinstalling the unit from the repo
// silently dropped the belly calibration and left the channel reading ~65 mm short
// while looking perfectly healthy.  A calibration that only exists on one machine is
// not calibration.
//
// ⚠ AND A SECOND CONSUMER IS EXACTLY WHEN THIS BITES.  ogma_host now reads the belly
// ToF too; hardcoding 64.8 there would have made three copies of one fitted number,
// two of them invisible.  One file, one loader, both programs.
//
// CLI flags still override, because bench work needs to sweep a value without editing
// calibration.  An override is PRINTED, so a run whose log does not mention one was
// using the file.

#include <array>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

namespace ogma::hw {

struct SensorCalib {
    double tof_mount_offset_mm = 0.0;    // VL53L0X, anchored at belly-down
    double ina_r_shunt_ohm     = 0.01;   // INA219, trace+solder included
    double gc_stand_m          = 0.06;   // ground_clearance normalizer; matches the
                                         // sim's GROUND_CLEARANCE_STAND by contract
    // ICM-20948 chip-frame body-up, belly flat on a level floor.  Mount tilt + accel
    // bias, unsplit, accelerometer-only.  Default is the 2026-09-10 fit, which is also
    // Icm20948Config's default — so a missing calib file changes nothing here.
    std::array<float, 3> imu_level_ref = {-0.03493f, -0.00558f, +0.99937f};
    bool   loaded = false;
    std::string source;                  // path actually read, for the receipt

    // Missing file is NOT an error: the defaults above are the documented ones, and a
    // bench rig without a calib file should still run.  But `loaded` is reported, so a
    // run on defaults is distinguishable from a run on this robot's real numbers —
    // which is the difference between a measurement and a number that looks like one.
    static SensorCalib load(const std::string& path = "pi_host/calib/sensors.json") {
        SensorCalib c;
        c.source = path;
        std::ifstream f(path);
        if (!f) return c;
        try {
            nlohmann::json j; f >> j;
            if (j.contains("tof") && j["tof"].contains("mount_offset_mm"))
                c.tof_mount_offset_mm = j["tof"]["mount_offset_mm"].get<double>();
            if (j.contains("ina219") && j["ina219"].contains("r_shunt_ohm"))
                c.ina_r_shunt_ohm = j["ina219"]["r_shunt_ohm"].get<double>();
            if (j.contains("ground_clearance") && j["ground_clearance"].contains("stand_m"))
                c.gc_stand_m = j["ground_clearance"]["stand_m"].get<double>();
            if (j.contains("imu") && j["imu"].contains("level_ref")) {
                const auto& lr = j["imu"]["level_ref"];
                if (lr.is_array() && lr.size() == 3)
                    for (int i = 0; i < 3; ++i) c.imu_level_ref[size_t(i)] = lr[size_t(i)].get<float>();
            }
            c.loaded = true;
        } catch (...) {
            // A malformed calib file is worse than none: it means someone edited it and
            // believes it is in force.  Leave loaded=false so the receipt says so.
        }
        return c;
    }
};

}  // namespace ogma::hw
