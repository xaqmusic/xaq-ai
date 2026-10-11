// Parity check: ogma::body::fk_leg vs picrawler_body.gd::_fk_leg.
//
//   godot4 --headless --path godot_host/project \
//           -s res://scripts_tools/leg_kinematics_parity.gd     # /tmp/legfk_parity.txt
//   g++ -O2 -std=c++17 -Icpp_core/include \
//       cpp_core/tests/body/leg_kinematics_parity_check.cpp -o /tmp/p && /tmp/p
//
// Expected: "cases 400   bit-exact 400   mismatched 0".
//
// ⚠ GDScript's Basis.x/.y/.z are COLUMNS, not rows -- Basis stores rows[3] and the
// scripting API binds x/y/z through set_column/get_column. The oracle therefore emits
// columns, and this compares columns. Compare rows against it and a CORRECT
// implementation fails on every non-symmetric matrix, which is most of them.
#include "ogma/body/LegKinematics.hpp"
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
static uint32_t bits(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

int main(int argc, char** argv) {
    std::ifstream in(argc > 1 ? argv[1] : "/tmp/legfk_parity.txt");
    if (!in) { std::printf("cannot open oracle\n"); return 2; }
    std::string line;
    int n = 0, bad = 0;
    int bad_coxa = 0, bad_upper = 0, bad_lower = 0, bad_basis = 0, bad_origin = 0;
    double worst = 0.0;
    while (std::getline(in, line)) {
        for (char& c : line) if (c == '|') c = ' ';
        std::istringstream ss(line);
        std::vector<std::string> t;
        for (std::string s; ss >> s; ) t.push_back(s);
        if (t.size() < 64) { std::printf("short line at case %d (%zu tokens)\n", n, t.size()); return 2; }

        int k = 0;
        auto v3 = [&]() { Vec3f v(unhex32(t[k]), unhex32(t[k+1]), unhex32(t[k+2])); k += 3; return v; };
        LegAnchors a;
        a.hip1_world = v3(); a.hip2_world = v3(); a.knee_world = v3();
        a.coxa_rest_origin = v3(); a.upper_rest_origin = v3(); a.lower_rest_origin = v3();
        a.hip2_axis = v3(); a.knee_axis = v3();
        const double t1 = unhex64(t[k++]), t2 = unhex64(t[k++]);
        const double t3 = unhex64(t[k++]), lift = unhex64(t[k++]);

        const LegPose p = fk_leg(a, t1, t2, t3, lift);
        const Xform3f* got[3] = { &p.coxa, &p.upper, &p.lower };

        bool case_bad = false;
        for (int s = 0; s < 3; ++s) {
            bool seg_basis_bad = false, seg_origin_bad = false;
            // three COLUMNS, then the origin -- the order the oracle emits
            for (int col = 0; col < 3; ++col) {
                const float want[3] = { unhex32(t[k]), unhex32(t[k+1]), unhex32(t[k+2]) };
                k += 3;
                const float mine[3] = { got[s]->basis.rows[0][col],
                                        got[s]->basis.rows[1][col],
                                        got[s]->basis.rows[2][col] };
                for (int e = 0; e < 3; ++e) {
                    if (bits(mine[e]) != bits(want[e])) seg_basis_bad = true;
                    const double d = std::fabs(double(mine[e]) - want[e]);
                    if (d > worst) worst = d;
                }
            }
            const float ow[3] = { unhex32(t[k]), unhex32(t[k+1]), unhex32(t[k+2]) };
            k += 3;
            const float om[3] = { got[s]->origin.x, got[s]->origin.y, got[s]->origin.z };
            for (int e = 0; e < 3; ++e) {
                if (bits(om[e]) != bits(ow[e])) seg_origin_bad = true;
                const double d = std::fabs(double(om[e]) - ow[e]);
                if (d > worst) worst = d;
            }
            if (seg_basis_bad)  { ++bad_basis;  }
            if (seg_origin_bad) { ++bad_origin; }
            if (seg_basis_bad || seg_origin_bad) {
                case_bad = true;
                if (s == 0) ++bad_coxa; else if (s == 1) ++bad_upper; else ++bad_lower;
                if (bad == 0) {
                    std::printf("first mismatch: case %d, segment %s\n", n,
                                s == 0 ? "coxa" : (s == 1 ? "upper" : "lower"));
                    std::printf("  gd  origin %08x %08x %08x\n", bits(ow[0]), bits(ow[1]), bits(ow[2]));
                    std::printf("  cpp origin %08x %08x %08x\n", bits(om[0]), bits(om[1]), bits(om[2]));
                }
            }
        }
        if (case_bad) ++bad;
        ++n;
    }
    std::printf("cases %d   bit-exact %d   mismatched %d   worst |delta| %.3e\n",
                n, n - bad, bad, worst);
    std::printf("  by segment: coxa %d  upper %d  lower %d\n", bad_coxa, bad_upper, bad_lower);
    std::printf("  by part   : basis %d  origin %d\n", bad_basis, bad_origin);
    std::printf("RESULT: %s\n", bad == 0 ? "BYTE-IDENTICAL" : "NOT byte-identical");
    return bad == 0 ? 0 : 1;
}
