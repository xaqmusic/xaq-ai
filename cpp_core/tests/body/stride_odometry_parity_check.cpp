// Parity check: ogma::body::StrideOdometry vs picrawler_body.gd's stride block.
//
//   godot4 --headless --path godot_host/project \
//           -s res://scripts_tools/stride_odometry_parity.gd   # /tmp/strido_parity.txt
//   g++ -O2 -std=c++17 -Icpp_core/include \
//       cpp_core/tests/body/stride_odometry_parity_check.cpp -o /tmp/p && /tmp/p
//
// Expected: "steps 900   bit-exact 900   mismatched 0", and every per-field counter 0.
//
// ⚠ THE FIELD BREAKDOWN IS THE POINT, not decoration.  When this port was written the
// first failure was a single field (see the repo's own lesson: a port is the same
// numbers, not the same maths -- and bit-identity dies on constant width and
// substituted formulas, so what you need is to know WHICH operation).  A bare
// pass/fail tells you the module is wrong; the per-field counters tell you whether it
// is the servo lag (double), the cross product (float32), the Vector2 narrowing, or
// slip's double accumulation sitting right beside two float32 ones.
//
// ⚠ THE ORACLE ASSERTS ITS OWN BRANCH COVERAGE and prints it.  StrideV's coast branch
// fires only at stance 0; a "bit-exact" over an oracle that never reached it would be
// a green gate over untested code.
#include "ogma/body/StrideOdometry.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
using namespace ogma::body;

static float unhex32(const std::string& h) {
    uint32_t u = 0; std::sscanf(h.c_str(), "%x", &u);
    float f; std::memcpy(&f, &u, 4); return f;
}
static double unhex64(const std::string& h) {
    uint64_t u = 0; std::sscanf(h.c_str(), "%lx", &u);
    double d; std::memcpy(&d, &u, 8); return d;
}
static bool same32(float a, float b) {
    uint32_t x, y; std::memcpy(&x, &a, 4); std::memcpy(&y, &b, 4); return x == y;
}
static bool same64(double a, double b) {
    uint64_t x, y; std::memcpy(&x, &a, 8); std::memcpy(&y, &b, 8); return x == y;
}

int main(int argc, char** argv) {
    std::ifstream in(argc > 1 ? argv[1] : "/tmp/strido_parity.txt");
    if (!in) { std::printf("cannot open oracle\n"); return 2; }

    const double TAU = 0.02, LP_ALPHA = 0.2, LOAD_THRESH = 0.2, L3 = 0.0765;

    ServoForwardModel lp;
    StrideV sv;
    Vec3f prev_toe[4];
    bool  prev_loaded[4] = { false, false, false, false };

    int n = 0, bad = 0;
    int bad_lp = 0, bad_feety = 0, bad_vleg = 0, bad_ns = 0;
    int bad_alin = 0, bad_est = 0, bad_bias = 0, bad_slip = 0;
    int stance_hist[5] = { 0, 0, 0, 0, 0 };

    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::vector<std::string> t;
        for (std::string s; ss >> s; ) t.push_back(s);
        if (t.empty()) continue;

        int k = 0;
        auto v3 = [&]() { Vec3f v(unhex32(t[k]), unhex32(t[k+1]), unhex32(t[k+2])); k += 3; return v; };

        Vec3f toe[4];
        for (int i = 0; i < 4; ++i) toe[i] = v3();
        const Vec3f gyro  = v3();
        const Vec3f accel = v3();
        const Vec3f up    = v3();
        double eff[12];
        for (int j = 0; j < 12; ++j) eff[j] = unhex64(t[k++]);
        double loads[4];
        for (int i = 0; i < 4; ++i) loads[i] = unhex64(t[k++]);
        const bool prev_valid = (t[k++] == "1");
        if (t[k] == "|") ++k;

        // ---- replay ------------------------------------------------------------
        if (!lp.seeded()) lp.seed(eff); else lp.step(eff, LP_ALPHA);

        double feety[4];
        for (int i = 0; i < 4; ++i) feety[i] = feet_y_gravity(toe[i], up, L3);

        bool loaded[4];
        for (int i = 0; i < 4; ++i) loaded[i] = loads[i] >= LOAD_THRESH;

        Vec3f vleg[4];
        Vec3f stance_sum;
        int stance_n = 0;
        if (prev_valid) {
            for (int i = 0; i < 4; ++i) {
                vleg[i] = planted_foot_velocity(toe[i], prev_toe[i], gyro, TAU);
                if (loaded[i] && prev_loaded[i]) {
                    stance_sum = stance_sum + vleg[i];
                    ++stance_n;
                }
            }
        } else {
            for (int i = 0; i < 4; ++i) vleg[i] = Vec3f();
        }
        const Vec3f a_lin = sv.linear_accel(accel, up);
        sv.step(a_lin, stance_sum, stance_n, TAU);
        stance_hist[stance_n] += 1;

        // ---- compare, field by field -------------------------------------------
        bool row_bad = false;
        for (int j = 0; j < 12; ++j)
            if (!same64(lp[j], unhex64(t[k + j]))) { ++bad_lp; row_bad = true; }
        k += 12;
        for (int i = 0; i < 4; ++i)
            if (!same64(feety[i], unhex64(t[k + i]))) { ++bad_feety; row_bad = true; }
        k += 4;
        for (int i = 0; i < 4; ++i) {
            const Vec3f want = v3();
            if (!same32(vleg[i].x, want.x) || !same32(vleg[i].y, want.y) ||
                !same32(vleg[i].z, want.z)) { ++bad_vleg; row_bad = true; }
        }
        if (stance_n != std::atoi(t[k++].c_str())) { ++bad_ns; row_bad = true; }
        {
            const Vec3f want = v3();
            if (!same32(a_lin.x, want.x) || !same32(a_lin.y, want.y) ||
                !same32(a_lin.z, want.z)) { ++bad_alin; row_bad = true; }
        }
        if (!same32(sv.est().x,  unhex32(t[k])) ||
            !same32(sv.est().y,  unhex32(t[k+1]))) { ++bad_est; row_bad = true; }
        k += 2;
        if (!same32(sv.bias().x, unhex32(t[k])) ||
            !same32(sv.bias().y, unhex32(t[k+1]))) { ++bad_bias; row_bad = true; }
        k += 2;
        if (!same64(sv.slip(), unhex64(t[k++]))) { ++bad_slip; row_bad = true; }

        if (row_bad) { if (bad == 0) std::printf("first mismatch at step %d\n", n); ++bad; }
        ++n;

        for (int i = 0; i < 4; ++i) { prev_toe[i] = toe[i]; prev_loaded[i] = loaded[i]; }
    }

    std::printf("steps %d   bit-exact %d   mismatched %d\n", n, n - bad, bad);
    std::printf("  servo_lp %d  feet_y %d  v_leg %d  stance_n %d  a_lin %d  est %d  bias %d  slip %d\n",
                bad_lp, bad_feety, bad_vleg, bad_ns, bad_alin, bad_est, bad_bias, bad_slip);
    std::printf("  stance coverage  n0 %d  n1 %d  n2 %d  n3 %d  n4 %d\n",
                stance_hist[0], stance_hist[1], stance_hist[2], stance_hist[3], stance_hist[4]);
    // An oracle that never coasts cannot certify StrideV, however green it looks.
    bool gap = false;
    for (int i = 0; i < 5; ++i) if (stance_hist[i] == 0) gap = true;
    if (gap) { std::printf("  ⚠ BRANCH-COVERAGE GAP — this run does not certify StrideV\n"); return 3; }
    return bad == 0 ? 0 : 1;
}
