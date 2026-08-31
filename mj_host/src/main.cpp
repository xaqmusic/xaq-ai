// =============================================================================
// mj_host — the Microduck MuJoCo host
// =============================================================================
//
// Phase S0 (docs/plans-and-designs/microduck_port_plan.md): load the vendored body
// and report what it is.  No brain, no stepping, no actuation.
//
// `--load-only` is not a debug dump, it is gate G1 with its working shown.  It
// answers three questions the plan needs answered before anything is built on top:
//
//   G1  does Pollen's MJCF load with zero local edits?
//   G3  is the 50 Hz brain tick an integer number of physics steps?
//   G4  does ctrl[i] drive the joint this project thinks it drives?
//
// G4 is the one that would otherwise fail silently on a robot.  The picrawler's
// leg-naming mirror (picrawler_sim2real_port.md) is the same trap: a mapping that
// is wrong produces a body that moves plausibly and incorrectly, and nothing in a
// metric catches it.  So the order is checked against the names, here, at startup.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace {

// The 14 joints every alpha policy drives, in ctrl order — `JOINT_NAMES` from
// duck-ipc-proto with the mouth (index 9) removed, which is the vector both the
// observation and the action use.  Transcribed once, and then only ever compared
// against, never used to index.
constexpr const char* kPolicyJoints[] = {
    "left_hip_yaw",  "left_hip_roll",  "left_hip_pitch",  "left_knee",  "left_ankle",
    "neck_pitch",    "head_pitch",     "head_yaw",        "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
};
constexpr int kNumPolicyJoints = int(sizeof(kPolicyJoints) / sizeof(kPolicyJoints[0]));

// The rate the brain runs at, and the rate `robotd` runs its control loop at.
constexpr double kBrainHz = 50.0;

const char* name_of(const mjModel* m, mjtObj type, int id) {
    const char* n = mj_id2name(m, type, id);
    return n ? n : "<unnamed>";
}

// G3 — the brain tick must be a whole number of physics steps.  A ratio that does
// not divide evenly means the tick length wobbles between two substep counts, and
// every learning rate in the graph is then quoted against a moving unit.
bool report_rate(const mjModel* m) {
    const double dt = m->opt.timestep;
    const double exact = (1.0 / kBrainHz) / dt;
    const int substeps = int(std::lround(exact));
    const bool integral = std::fabs(exact - substeps) < 1e-9;

    std::printf("\nRATE\n");
    std::printf("  physics timestep   %g s  (%.1f Hz)\n", dt, 1.0 / dt);
    std::printf("  brain tick         %g s  (%.1f Hz)\n", 1.0 / kBrainHz, kBrainHz);
    std::printf("  substeps per tick  %.6f -> %d   %s\n", exact, substeps,
                integral ? "[G3 PASS]" : "[G3 FAIL — not integral]");
    return integral;
}

// G4 — ctrl index to joint, by name.  Two failures are possible and they are
// different: an actuator driving a joint we did not expect, and a joint we expect
// that the model does not have.  Both are reported against the name, never the index.
bool report_actuators(const mjModel* m) {
    std::printf("\nACTUATORS  (ctrl index -> joint)\n");
    bool ok = m->nu == kNumPolicyJoints;
    if (!ok) {
        std::printf("  !! model has %lld actuators, this host expects %d\n", (long long)m->nu,
                    kNumPolicyJoints);
    }
    for (int i = 0; i < int(m->nu); ++i) {
        const int jid = m->actuator_trnid[2 * i];
        const char* jname = (jid >= 0) ? name_of(m, mjOBJ_JOINT, jid) : "<not a joint>";
        const char* want = (i < kNumPolicyJoints) ? kPolicyJoints[i] : "<beyond the policy vector>";
        const bool match = (i < kNumPolicyJoints) && std::strcmp(jname, want) == 0;
        ok = ok && match;
        std::printf("  %2d  %-18s -> %-18s ctrl[%g %g] force[%g %g]  %s\n", i,
                    name_of(m, mjOBJ_ACTUATOR, i), jname,
                    m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1],
                    m->actuator_forcerange[2 * i], m->actuator_forcerange[2 * i + 1],
                    match ? "" : (std::string("<-- expected ") + want).c_str());
    }
    // Every joint the host will need must resolve by name, whatever order it sits in.
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        if (mj_name2id(m, mjOBJ_JOINT, kPolicyJoints[i]) < 0) {
            std::printf("  !! joint '%s' is not in this model\n", kPolicyJoints[i]);
            ok = false;
        }
    }
    std::printf("  %s\n", ok ? "[G4 PASS — ctrl order matches the policy joint vector]"
                             : "[G4 FAIL — resolve by name, and find out why the order moved]");
    return ok;
}

void report_model(const mjModel* m, const char* path) {
    std::printf("MODEL  %s\n", path);
    // MuJoCo 3.12 widened the model dimensions to mjtSize (int64_t).  Printed
    // through long long rather than truncated to int, so a pin bump that changes
    // the width again is a compile error instead of a silent narrowing.
    std::printf("  nq %lld   nv %lld   nu %lld   nbody %lld   ngeom %lld   nsensor %lld   nkey %lld\n",
                (long long)m->nq, (long long)m->nv, (long long)m->nu, (long long)m->nbody,
                (long long)m->ngeom, (long long)m->nsensor, (long long)m->nkey);

    double mass = 0.0;
    for (mjtSize b = 0; b < m->nbody; ++b) mass += m->body_mass[b];
    std::printf("  total mass %.1f g\n", mass * 1000.0);

    std::printf("\nSENSORS\n");
    for (mjtSize i = 0; i < m->nsensor; ++i) {
        std::printf("  %lld  %-20s dim %d  adr %d\n", (long long)i,
                    name_of(m, mjOBJ_SENSOR, int(i)), m->sensor_dim[i], m->sensor_adr[i]);
    }

    std::printf("\nKEYFRAMES\n");
    for (mjtSize i = 0; i < m->nkey; ++i)
        std::printf("  %lld  %s\n", (long long)i, name_of(m, mjOBJ_KEY, int(i)));
    if (m->nkey == 0) std::printf("  (none)\n");
}

void usage() {
    std::printf(
        "ogma_mjhost — the Microduck MuJoCo host\n"
        "\n"
        "  ogma_mjhost --load-only [scene.xml]\n"
        "      Load the model and report it.  Checks gates G1 (loads unmodified),\n"
        "      G3 (integral substeps) and G4 (ctrl order matches the joint names).\n"
        "      Exits non-zero if any of them fails.\n"
        "\n"
        "  The scene defaults to " MJ_HOST_MODEL_DIR "/scene.xml.\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string scene = std::string(MJ_HOST_MODEL_DIR) + "/scene.xml";
    bool load_only = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--load-only") {
            load_only = true;
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else if (!a.empty() && a[0] != '-') {
            scene = a;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            usage();
            return 2;
        }
    }

    if (!load_only) {
        std::fprintf(stderr, "nothing to do yet — phase S0 implements --load-only only.\n\n");
        usage();
        return 2;
    }

    // G1.  A model that will not load is the whole answer; say why, in MuJoCo's own
    // words, rather than reporting that the host failed to start.
    char err[1024] = {0};
    mjModel* m = mj_loadXML(scene.c_str(), nullptr, err, sizeof(err));
    if (!m) {
        std::fprintf(stderr, "[G1 FAIL] %s\n  %s\n", scene.c_str(), err);
        return 1;
    }
    std::printf("[G1 PASS] loaded unmodified — MuJoCo library %s, header %d\n",
                mj_versionString(), mjVERSION_HEADER);
    if (mj_version() != mjVERSION_HEADER) {
        // Reported rather than fatal: it loaded, so say what it loaded against.
        std::printf("  !! header/library skew — built against %d, running %d\n",
                    mjVERSION_HEADER, mj_version());
    }
    std::printf("\n");

    report_model(m, scene.c_str());
    const bool g3 = report_rate(m);
    const bool g4 = report_actuators(m);

    mj_deleteModel(m);

    const bool ok = g3 && g4;
    std::printf("\n%s\n", ok ? "S0 gates: G1 PASS, G3 PASS, G4 PASS"
                             : "S0 gates: a check FAILED — see above");
    return ok ? 0 : 1;
}
