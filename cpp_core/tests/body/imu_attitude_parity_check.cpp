// Parity check: ogma::body::ImuAttitude vs the GDScript original it was ported from.
//
// The port doc's gate for Order step (a) is that the sim stays BYTE-IDENTICAL across
// the swap, so this compares float32 BIT PATTERNS, not tolerances.  Two steps, because
// the oracle has to come from the engine itself:
//
//   godot4 --headless --path godot_host/project \
//           -s res://scripts_tools/imu_attitude_parity.gd        # writes /tmp/imu_parity.txt
//   g++ -O2 -std=c++17 -Icpp_core/include \
//       cpp_core/tests/body/imu_attitude_parity_check.cpp -o /tmp/p && /tmp/p
//
// Expected: "steps 800   bit-exact 800   mismatched 0".
//
// WHAT THIS CAUGHT, because it is the non-obvious part.  Every arithmetic operation was
// already bit-exact -- basis_axis_angle_parity.gd isolates Basis(axis,angle)*v and gets
// 0 ULP over 500 rows, and up_accel (length/normalize/sqrt) never mismatched once.  The
// divergence was entirely in CONSTANT WIDTH: GDScript's `float` is 64-bit, so its 9.81
// and 0.02 are double literals, while a C++ `float` 9.81f widens to 9.8100004196166992.
// Right formulas, different numbers, 1 ULP per step.  If this test ever regresses, look
// at the types of the constants before you look at the maths.
// ogma::body::ImuAttitude, and compares the outputs BIT FOR BIT.
#include "ogma/body/ImuAttitude.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
using namespace ogma::body;

static float unhex(const std::string& h) {
    uint32_t u = 0; std::sscanf(h.c_str(), "%x", &u);
    float f; std::memcpy(&f, &u, 4); return f;
}
static uint32_t bits(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

int main(int argc, char** argv) {
    std::ifstream in(argc > 1 ? argv[1] : "/tmp/imu_parity.txt");
    if (!in) { std::printf("cannot open oracle\n"); return 2; }
    ImuAttitude f;
    std::string line;
    int n = 0, bad = 0, first_bad = -1, bad_acc = 0, bad_fused_only = 0;
    double worst = 0.0;
    while (std::getline(in, line)) {
        for (char& c : line) if (c == '|') c = ' ';
        std::istringstream ss(line);
        std::string t[13];
        for (auto& s : t) ss >> s;
        const Vec3f accel(unhex(t[0]), unhex(t[1]), unhex(t[2]));
        const Vec3f gyro (unhex(t[3]), unhex(t[4]), unhex(t[5]));
        uint64_t dtu = 0; std::sscanf(t[6].c_str(), "%lx", &dtu); double dt; std::memcpy(&dt, &dtu, 8);
        const Vec3f up_e(unhex(t[7]), unhex(t[8]), unhex(t[9]));
        const Vec3f up_a(unhex(t[10]), unhex(t[11]), unhex(t[12]));

        f.step(accel, gyro, dt);
        const Vec3f g = f.up_fused(), a = f.up_accel();
        const bool same = bits(g.x) == bits(up_e.x) && bits(g.y) == bits(up_e.y) &&
                          bits(g.z) == bits(up_e.z) && bits(a.x) == bits(up_a.x) &&
                          bits(a.y) == bits(up_a.y) && bits(a.z) == bits(up_a.z);
        const bool acc_same = bits(a.x) == bits(up_a.x) && bits(a.y) == bits(up_a.y) && bits(a.z) == bits(up_a.z);
        if (!acc_same) ++bad_acc;
        if (acc_same && !same) ++bad_fused_only;
        if (!same) {
            if (first_bad < 0) {
                first_bad = n;
                std::printf("first mismatch at step %d\n", n);
                std::printf("  gd  up_fused %08x %08x %08x\n", bits(up_e.x), bits(up_e.y), bits(up_e.z));
                std::printf("  cpp up_fused %08x %08x %08x\n", bits(g.x), bits(g.y), bits(g.z));
            }
            ++bad;
        }
        double d = std::fabs(double(g.x) - up_e.x) + std::fabs(double(g.y) - up_e.y)
                 + std::fabs(double(g.z) - up_e.z);
        if (d > worst) worst = d;
        ++n;
    }
    std::printf("steps %d   bit-exact %d   mismatched %d   worst |delta| %.3e\n",
                n, n - bad, bad, worst);
    std::printf("  up_accel (sqrt + divide, NO trig) mismatched: %d\n", bad_acc);
    std::printf("  up_fused only (the Basis/trig path)        : %d\n", bad_fused_only);
    std::printf("RESULT: %s\n", bad == 0 ? "BYTE-IDENTICAL" : "NOT byte-identical");
    return bad == 0 ? 0 : 1;
}
