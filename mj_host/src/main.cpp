// =============================================================================
// mj_host — the Microduck MuJoCo host
// =============================================================================
//
// Phase S1 (docs/plans-and-designs/microduck_port_plan.md): the body and a run
// loop.  No OgmaInstance, no bus, no learning.  What runs here is the standing
// SCAFFOLD (models/microduck/scaffolds/), because this body has no passive
// standing equilibrium and the only honest no-brain baseline is an actively
// balanced one.
//
// Modes:
//   --load-only   gate G1 with its working shown, plus G3 and G4
//   --hold        run the standing scaffold and write one JSON object per tick
//   --gate-g2     the settle sweep: noise x seeds, PASS on tilt rather than height
//
// Every mode exits non-zero when a gate fails, so all of them belong in CI rather
// than in somebody's memory.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <thread>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "DuckBody.hpp"
#include "Observation.hpp"
#include "Odometry.hpp"
#include "IntentAdapter.hpp"
#include "Policy.hpp"
#include "Recovery.hpp"
#include "OgmaBrainAdapter.hpp"
#include "StubBrain.hpp"

namespace {

using namespace mjhost;

const std::string kModelDir = MJ_HOST_MODEL_DIR;
const std::string kDefaultScene  = kModelDir + "/scene.xml";
const std::string kStandScaffold = kModelDir + "/scaffolds/alpha_stand.onnx";
// The WALKER: Pollen's alpha_walking.onnx ("velstand" — walking on velocity commands and
// fall recovery in one network), the same 61-wide observation as the stander.  Driven as
// their runtime drives it (deploy/robotd.toml): actions scaled by 0.9, joint targets
// low-passed with blend 0.7 (legs) / 0.5 (head) — the values it was trained with.
const std::string kWalkScaffold  = kModelDir + "/scaffolds/alpha_walking.onnx";
constexpr double kWalkingActionScale = 0.9;
constexpr double kWalkLowpassLegs = 0.7, kWalkLowpassHead = 0.5;

// The standing policy is trained to be applied whole. `robotd` also low-passes the
// targets (head 0.5, legs 0.7) while microduck_rl trains and rehearses unfiltered;
// this host matches the rehearsal path (scripts/infer_policy.py), which is the one
// that was cross-checked against these numbers.
constexpr double kStandingActionScale = 1.0;

// A robot past this much tilt is on its way down, not standing.
constexpr double kFallenTiltDeg = 15.0;

// --realtime: the host paces ITSELF to the wall clock, one brain tick per 1/kBrainHz.
// Without it a viewer paces the host through the stdout pipe, and a pipe is read in
// 8 KB chunks: the host runs ~14 ticks in a burst and blocks, so every observer that
// listens at tick time — the inspector's diag stream above all — sees a 3.6 Hz jerk
// whatever rate it asked for (measured: gap median 1 ms, max 275 ms).  With the host
// pacing, ticks are 20 ms apart and the viewer's own pacing has nothing left to do.
bool g_realtime = false;
struct TickPacer {
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    void wait_for(int tick, double hz) const {
        if (!g_realtime) return;
        std::this_thread::sleep_until(start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                  std::chrono::duration<double>(tick / hz)));
    }
};

// --no-tilt-gate: disables the harness's learnable-regime gate (the hand rule
// the R1 regime banks are the learned replacement for).
bool g_no_tilt_gate = false;

// --freeze-after N: permanently freeze ALL learning after N brain-driven
// seconds — a DIAGNOSTIC (a lesion-as-test, never an operating mode).  The
// question it answers: seeds find standing in their first minutes and then
// lose it — is continued learning the destroyer?
int g_freeze_after_ticks = 0;

// --save-brain FILE / --load-brain FILE (2026-09-01, the tall-standing
// snapshot): checkpoint the WHOLE learned brain (every module's working state
// incl. earned consolidation, plus the bus cache) together with the exact
// physics state, so refinement can continue FROM a stable stance instead of
// re-earning it.  Load restores brain + body before the first tick; callers
// resuming a snapshot should not pass --ident flags (the babble is long done).
std::string g_save_brain, g_load_brain;

// --servo-filter: apply robotd's deployed joint-target low-pass (head 0.5,
// legs 0.7) to the ogma brain's commands.  Off = legacy raw targets.
bool g_servo_filter = false;

const char* name_of(const mjModel* m, mjtObj type, int id) {
    const char* n = mj_id2name(m, type, id);
    return n ? n : "<unnamed>";
}

// ---------------------------------------------------------------------------
// --load-only  (S0)
// ---------------------------------------------------------------------------

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

int cmd_load_only(const std::string& scene) {
    char err[1024] = {0};
    mjModel* m = mj_loadXML(scene.c_str(), nullptr, err, sizeof(err));
    if (!m) {
        std::fprintf(stderr, "[G1 FAIL] %s\n  %s\n", scene.c_str(), err);
        return 1;
    }
    std::printf("[G1 PASS] loaded unmodified — MuJoCo library %s, header %d\n", mj_versionString(),
                mjVERSION_HEADER);
    if (mj_version() != mjVERSION_HEADER) {
        std::printf("  !! header/library skew — built against %d, running %d\n", mjVERSION_HEADER,
                    mj_version());
    }
    std::printf("\nMODEL  %s\n", scene.c_str());
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

    const bool g3 = report_rate(m);
    const bool g4 = report_actuators(m);
    mj_deleteModel(m);

    const bool ok = g3 && g4;
    std::printf("\n%s\n", ok ? "S0 gates: G1 PASS, G3 PASS, G4 PASS"
                             : "S0 gates: a check FAILED — see above");
    return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// The run loop
// ---------------------------------------------------------------------------

// A robot is BACK once it is under this and stays there — passing through on the
// way down does not count, which is what the hold below is for.
constexpr double kRecoveredTiltDeg = 5.0;
constexpr double kRecoverHoldSecs  = 0.4;
constexpr double kRecoverWindowSecs = 4.0;

// What happened to one shove.
struct Shove {
    double t = 0.0;
    double peak_tilt = 0.0;
    double recovered_after = -1.0;   // seconds; negative if it did not
    bool   conclusive = false;       // was there room to judge this one at all?
    // Why not, when not. These are different problems with different fixes: one is
    // "run longer", the other is "shove less often", and reporting the wrong one
    // sends the reader to change the wrong knob.
    bool   cut_short_by_next = false;
    // Brain path only: the recovery harness handed the body to the scaffold inside
    // this shove's window. A shove the BRAIN caught and one the SCAFFOLD stood back
    // up are both "recovered"; only the first is evidence about the brain.
    bool   rescued = false;
};

struct RunResult {
    double tilt_max = 0.0;
    double z_end = 0.0;
    double travelled = 0.0;

    // The fix for the blind metric. `tilt_end` was read at the last tick, so it
    // could not tell UPRIGHT from STILL RECOVERING, and a run that stopped two
    // seconds after a shove reported a fall that never happened.
    //
    // Three things replace it, and the third is the one that was actually missing:
    //   * settled_tilt   — the worst tilt over the final window, not one sample
    //   * upright_frac   — how much of the run was spent up, which no instant knows
    //   * shoves         — per-shove recovery, WITH a conclusive/inconclusive flag
    //
    // A shove too near the end of the run is INCONCLUSIVE, never failed. Calling
    // "I did not watch long enough" a failure is how a metric lies in the
    // direction of its own convenience.
    double settled_tilt = 0.0;
    double upright_frac = 0.0;
    std::vector<Shove> shoves;

    int recovered() const {
        int n = 0;
        for (const auto& s : shoves) if (s.conclusive && s.recovered_after >= 0.0) ++n;
        return n;
    }
    int conclusive() const {
        int n = 0;
        for (const auto& s : shoves) if (s.conclusive) ++n;
        return n;
    }
    int inconclusive() const { return int(shoves.size()) - conclusive(); }

    // THREE outcomes, not two. A run that stopped while the robot was still getting
    // up has not failed and has not passed — it was not watched long enough, and
    // saying so is the whole point of this fix. Collapsing that into FAILED is how a
    // metric lies in the direction of its own convenience; collapsing it into PASSED
    // would be worse.
    enum class Verdict { Upright, Fallen, Inconclusive };

    Verdict verdict() const {
        // A definite failure outranks an unfinished one: if some shove that COULD be
        // judged was not recovered from, the run failed whatever else is in flight.
        for (const auto& s : shoves)
            if (s.conclusive && s.recovered_after < 0.0) return Verdict::Fallen;
        // Otherwise an unfinished shove means the settled window is measuring a
        // recovery in progress, and nothing can be concluded from it.
        if (inconclusive() > 0) return Verdict::Inconclusive;
        return settled_tilt < kFallenTiltDeg ? Verdict::Upright : Verdict::Fallen;
    }

    const char* verdict_name() const {
        switch (verdict()) {
            case Verdict::Upright: return "UPRIGHT";
            case Verdict::Fallen:  return "FALLEN";
            default:               return "INCONCLUSIVE";
        }
    }

    // 0 pass, 1 a real failure, 3 "not watched long enough" — distinct so a gate can
    // tell a broken robot from a badly set up run.
    int exit_code() const {
        switch (verdict()) {
            case Verdict::Upright: return 0;
            case Verdict::Fallen:  return 1;
            default:               return 3;
        }
    }
};

// Turn a tilt trace plus the ticks at which shoves landed into the verdict above.
// Post-processing rather than online detection, because "recovered" is a statement
// about a window and cannot be decided at the tick it starts.
RunResult analyse(const std::vector<double>& tilt, const std::vector<int>& shove_ticks) {
    RunResult r;
    if (tilt.empty()) return r;

    const int hold_ticks   = std::max(1, int(kRecoverHoldSecs * kBrainHz));
    const int window_ticks = int(kRecoverWindowSecs * kBrainHz);
    const int total = int(tilt.size());

    int up = 0;
    for (double v : tilt) {
        r.tilt_max = std::fmax(r.tilt_max, v);
        if (v < kFallenTiltDeg) ++up;
    }
    r.upright_frac = double(up) / total;

    // The final window, so the verdict is not one sample's opinion.
    const int settle_from = std::max(0, total - hold_ticks);
    for (int i = settle_from; i < total; ++i) r.settled_tilt = std::fmax(r.settled_tilt, tilt[i]);

    for (size_t k = 0; k < shove_ticks.size(); ++k) {
        const int from = shove_ticks[k];
        // Judge each shove up to the next one, or to the end of its window.
        const int next = (k + 1 < shove_ticks.size()) ? shove_ticks[k + 1] : total;
        const int until = std::min({total, from + window_ticks, next});

        Shove sh;
        sh.t = from / kBrainHz;
        for (int i = from; i < until; ++i) sh.peak_tilt = std::fmax(sh.peak_tilt, tilt[i]);

        // Recovered = under the threshold and STAYS under it for the hold.
        for (int i = from; i + hold_ticks <= until; ++i) {
            bool held = true;
            for (int j = i; j < i + hold_ticks; ++j) {
                if (tilt[j] >= kRecoveredTiltDeg) { held = false; break; }
            }
            if (held) { sh.recovered_after = (i + hold_ticks - from) / kBrainHz; break; }
        }
        // Enough room to have seen a recovery, had one happened?
        sh.conclusive = (sh.recovered_after >= 0.0) || (until - from >= window_ticks);
        sh.cut_short_by_next = !sh.conclusive && (next < total) && (next - from < window_ticks);
        r.shoves.push_back(sh);
    }
    return r;
}

struct PushPlan {
    double newtons = 0.0;      // 0 disables
    double every_s = 3.0;      // how often
    double hold_s  = 0.1;      // how long each shove lasts
    double from_s  = 0.0;      // no shove before this (let a brain consolidate first)
};

// The STEP HAND-OFF (phase 0 of the intent boundary, 2026-09-03).  The brain's
// in-place catch has a measured limit: across six R19 brains and 72 shoves, a
// caught shove never leans past 5.4 deg, and every 3 N topple passes 5.8 deg by
// 300 ms with the lean still rising, 8 deg at 400 ms, 10 deg at 500 ms — about
// 700 ms before the fall detector fires.  Past that angle the only recovery is a
// step, which the brain does not have and the trained walker does.  So: when the
// lean exceeds lean_deg for confirm_ticks consecutive rising ticks while the brain
// drives, hand the joints to the walker (the same standing network, given a
// twist along the lean), for twist_s, then let it settle and hand back through
// the identification path's own settle/handback machinery.  Learning is frozen
// for the whole excursion and the pairing is invalidated at both edges, exactly
// as for a rescue: nothing the walker does is learned from.  A harness rule, like
// the fall detector, on the same egocentric projected gravity.  0 disables and
// the build is byte-identical.
// A WALK on request (phase 1b): the joints go to the walker with a given twist for a
// given time, then settle and hand back — the same excursion as a step, triggered by an
// INTENT (here a scripted one, from the command line; later the level-2 brain's) rather
// than by the lean.  A scaffold for testing the behaviour hand-off, named as such.
struct WalkPlan {
    double from_s = -1.0;      // < 0 disables
    double secs = 2.0;
    double vx = 0.2, vy = 0.0, vyaw = 0.0;
};

struct StepPlan {
    double lean_deg = 0.0;     // 0 disables (the harness's own lean rule)
    double att = 0.0;          // > 0: trigger on the BRAIN's attitude-prior instant error instead
                               // (phase 1a: the saturation signal is the brain's, the threshold the
                               // harness's, measured on the same 72 shoves as the lean rule)
    int    confirm_ticks = 3;  // consecutive rising ticks above lean_deg
    double twist = 0.0;        // m/s along the lean direction; measured: the walker's own stagger
                               // recovers 3 N and 5 N at zero twist as well as at 0.2, and 0.35 or
                               // the wrong sign cost one in six — the hand-off is what matters
    double twist_s = 0.5;      // how long the twist is commanded
    double settle_s = 2.0;     // then wait for stillness, at most this long
};

// One episode of the standing scaffold. `emit` writes the per-tick JSONL when asked.
RunResult run_hold(DuckBody& body, Policy& policy, double seconds, double noise, uint64_t seed,
                   bool emit, const PushPlan& pushes = {}) {
    body.reset("STAND", noise, seed);
    const auto start = body.trunk_position();

    std::array<float, kActionLen> last_action{};
    const Command command{};  // stand still, head level, nominal stance

    std::vector<double> tilt_trace;
    std::vector<int> shove_ticks;
    const int ticks = int(seconds * kBrainHz);
    tilt_trace.reserve(ticks);
    const int push_period = int(pushes.every_s * kBrainHz);
    const int push_hold   = std::max(1, int(pushes.hold_s * kBrainHz));
    const int push_from   = int(pushes.from_s * kBrainHz);
    int push_index = 0;
    TickPacer pacer;

    for (int t = 0; t < ticks; ++t) {
        pacer.wait_for(t, kBrainHz);
        // Shove on a fixed schedule, rotating the direction so the controller is
        // asked to recover from every side rather than from a favourite one.
        //
        // NOTHING IS SHOVED INSIDE THE FINAL RECOVERY WINDOW. Otherwise every run
        // ends mid-getup and reports INCONCLUSIVE, and the operator is left doing
        // arithmetic to get an answer out of the tool. A conclusive run should be
        // what you get by default; an inconclusive one should take effort.
        const bool room_to_recover = (ticks - t) >= int(kRecoverWindowSecs * kBrainHz);
        if (pushes.newtons > 0.0 && push_period > 0 && t > 0 && t >= push_from &&
            t % push_period == 0 && room_to_recover) {
            static const double dirs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
            const auto& d = dirs[push_index++ % 4];
            body.push({pushes.newtons * d[0], pushes.newtons * d[1], 0.0}, push_hold);
            shove_ticks.push_back(t);
        }
        const auto action = policy.infer(build_observation(body, last_action, command));
        last_action = action;

        std::array<double, kNumPolicyJoints> ctrl{};
        for (int i = 0; i < kNumPolicyJoints; ++i)
            ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
        body.step(ctrl);

        const double tilt = body.tilt_deg();
        tilt_trace.push_back(tilt);

        if (emit) {
            const auto p = body.trunk_position();
            const auto g = body.gravity();
            const auto q = body.joint_positions();
            // One object per line, the shape the picrawler harness already parses.
            // Instrumentation fields (x/y/z, tilt) are world-frame and are for the
            // reader; no brain ever subscribes to them.
            std::printf("{\"t\":%.3f,\"tick\":%d,\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,\"tilt\":%.3f,"
                        "\"grav\":[%.4f,%.4f,%.4f],\"push\":[%.2f,%.2f,%.2f],\"q\":[",
                        body.time(), t, p[0], p[1], p[2], tilt, g[0], g[1], g[2],
                        body.active_push()[0], body.active_push()[1], body.active_push()[2]);
            for (int i = 0; i < kNumPolicyJoints; ++i)
                std::printf("%s%.4f", i ? "," : "", q[i]);
            // Full generalized position last, so a viewer can draw exactly this
            // pose instead of running a second copy of the dynamics to guess it.
            std::printf("],\"qpos\":[");
            const auto full = body.qpos();
            for (size_t i = 0; i < full.size(); ++i)
                std::printf("%s%.6f", i ? "," : "", full[i]);
            std::printf("]}\n");
        }
    }

    const auto end = body.trunk_position();
    RunResult r = analyse(tilt_trace, shove_ticks);
    r.z_end = end[2];
    r.travelled = std::hypot(end[0] - start[0], end[1] - start[1]);
    return r;
}

int cmd_hold(const std::string& scene, double seconds, double noise, uint64_t seed,
             const PushPlan& pushes) {
    DuckBody body(scene);
    Policy policy(kStandScaffold);
    const RunResult r = run_hold(body, policy, seconds, noise, seed, /*emit=*/true, pushes);
    if (pushes.newtons > 0.0 && r.shoves.empty()) {
        std::fprintf(stderr, "no room to shove: a run needs more than %.1f s so a recovery can "
                             "finish inside it\n", kRecoverWindowSecs);
    }

    std::fprintf(stderr, "held %.1f s — settled %.2f deg, peak %.2f deg, upright %.1f%% of the run,"
                         " z %.4f m, drift %.3f m\n",
                 seconds, r.settled_tilt, r.tilt_max, 100.0 * r.upright_frac, r.z_end, r.travelled);

    for (size_t i = 0; i < r.shoves.size(); ++i) {
        const auto& sh = r.shoves[i];
        if (!sh.conclusive) {
            // The whole point of the fix: no room to judge is its own answer, and it
            // is not a failure.
            std::fprintf(stderr, "  shove %zu at %5.2fs: peak %6.2f deg — INCONCLUSIVE, %s\n",
                         i + 1, sh.t, sh.peak_tilt,
                         sh.cut_short_by_next ? "the next shove landed before it could recover"
                                              : "the run ended too soon after it");
        } else if (sh.recovered_after >= 0.0) {
            std::fprintf(stderr, "  shove %zu at %5.2fs: peak %6.2f deg — recovered in %.2f s\n",
                         i + 1, sh.t, sh.peak_tilt, sh.recovered_after);
        } else {
            std::fprintf(stderr, "  shove %zu at %5.2fs: peak %6.2f deg — NOT RECOVERED\n",
                         i + 1, sh.t, sh.peak_tilt);
        }
    }
    if (!r.shoves.empty()) {
        std::fprintf(stderr, "  %d/%d judged shoves recovered", r.recovered(), r.conclusive());
        if (r.inconclusive() > 0) std::fprintf(stderr, ", %d not judged", r.inconclusive());
        std::fprintf(stderr, "\n");
    }
    std::fprintf(stderr, "%s\n", r.verdict_name());
    if (r.verdict() == RunResult::Verdict::Inconclusive) {
        bool crowded = false;
        for (const auto& sh : r.shoves) crowded = crowded || sh.cut_short_by_next;
        std::fprintf(stderr, "  (%s)\n",
                     crowded ? "space the shoves at least --push-every 4 apart, or a recovery "
                               "cannot finish before the next one"
                             : "give the run more time after the last shove");
    }
    return r.exit_code();
}

// ---------------------------------------------------------------------------
// --gate-g2
//
// The settle test.  CHECK TILT, NOT HEIGHT: a settle test that records only z
// reports a fallen robot as resting comfortably, which microduck_rl's own AGENTS.md
// names as a mistake that cost it days.
// ---------------------------------------------------------------------------

int cmd_gate_g2(const std::string& scene, double seconds) {
    DuckBody body(scene);
    Policy policy(kStandScaffold);

    std::printf("G2 — hold %s for %.0f s from noisy inits, under the standing scaffold\n",
                "STAND", seconds);
    std::printf("     PASS is tilt < %.0f deg across the final %.1f s. Height is not the test,\n"
                "     and neither is a single last sample.\n\n",
                kFallenTiltDeg, kRecoverHoldSecs);
    std::printf("  %10s %5s %9s %9s %9s  %s\n", "noise(rad)", "seed", "settled", "tilt_max",
                "z_end(m)", "verdict");

    bool all_ok = true;
    for (double noise : {0.0, 0.01, 0.03, 0.05}) {
        for (uint64_t seed = 0; seed < 3; ++seed) {
            const RunResult r = run_hold(body, policy, seconds, noise, seed, /*emit=*/false);
            const bool ok = r.verdict() == RunResult::Verdict::Upright;
            all_ok = all_ok && ok;
            std::printf("  %10.2f %5llu %9.2f %9.2f %9.4f  %s\n", noise, (unsigned long long)seed,
                        r.settled_tilt, r.tilt_max, r.z_end, ok ? "PASS" : "FALLEN");
        }
    }
    std::printf("\n%s\n", all_ok ? "[G2 PASS — the standing scaffold holds this body]"
                                 : "[G2 FAIL — see above]");
    return all_ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// --stub  (A2, exercised before A1 exists)
//
// A brain that falls over, a scaffold that picks it up, and the hand-off between
// them.  What is under test is the HARNESS: does the trigger fire on the right
// egocentric signal, does learning stop while the scaffold drives, does every edge
// announce itself, and does the body keep running instead of lying on the floor.
// ---------------------------------------------------------------------------

// ident_every/ident_until (0 = off): IDENTIFICATION EPISODES — during the first
// ident_until brain-driven ticks, insert a scaffold re-settle after every
// ident_every of them.  The probe's remaining trick, moved into the harness: the
// module's antisymmetric-pair babble cancels drift WITHIN a pair, but pairs that
// start from a toppling body still ride it; a settle between pairs hands each
// pair a still start, which is exactly how the probe achieved clean columns.
// Settle edges publish events.reset (pairing invalidation) and freeze learning.
//
// pushes: the (d) perturbation test on the BRAIN's stance.  The same rotating
// schedule as the scaffold's --hold, with two differences that matter: a shove is
// delivered only on a brain-driven tick (a shove landing mid-rescue would test the
// scaffold, so the slot is skipped, not deferred — the schedule stays readable),
// and each shove is reported as caught-by-the-brain or rescued-by-the-scaffold.
int run_with_brain(const std::string& scene, double seconds, uint64_t seed, BrainLike& brain,
                   bool emit, int ident_every = 0, int ident_until = 0,
                   const PushPlan& pushes = {}, const StepPlan& step = {},
                   const WalkPlan& walk = {}) {
    DuckBody body(scene);
    Policy scaffold(kStandScaffold);
    std::unique_ptr<Policy> walker;          // loaded only if a step or walk can happen
    std::array<double, kNumPolicyJoints> walk_targets{};   // the walker's low-passed targets
    std::array<float, kActionLen> walk_last_action{};
    Recovery recovery;

    body.reset("STAND", 0.0, seed);

    if (!g_load_brain.empty()) {
        if (auto* og = dynamic_cast<OgmaBrainAdapter*>(&brain)) {
            std::ifstream in(g_load_brain);
            if (!in) throw std::runtime_error("--load-brain: cannot open " + g_load_brain);
            nlohmann::json snap; in >> snap;
            og->restore_brain_state(snap.at("graph"));
            body.set_full_state(snap.at("qpos").get<std::vector<double>>(),
                                snap.at("qvel").get<std::vector<double>>());
            std::fprintf(stderr, "brain+body restored from %s\n", g_load_brain.c_str());
        } else {
            throw std::runtime_error("--load-brain requires the ogma brain");
        }
    }

    std::array<float, kActionLen> last_action{};
    Command command{};
    const double dt = 1.0 / kBrainHz;
    const int ticks = int(seconds * kBrainHz);

    int frozen_ticks = 0;
    int brain_ticks_seen = 0, settle_left = 0;

    // Step hand-off state (see StepPlan).
    const int step_twist_ticks  = std::max(1, int(step.twist_s * kBrainHz));
    const int step_settle_ticks = std::max(1, int(step.settle_s * kBrainHz));
    int step_left = 0, step_confirm = 0;
    int steps_started = 0, steps_ended = 0, steps_rescued = 0;
    double last_lean = 0.0;
    std::array<double, 3> step_twist{0.0, 0.0, 0.0};   // the twist commanded during the excursion
    int step_twist_left = 0;                            // ticks of twist remaining in it
    bool stepping = false, walking = false;
    Odometry odom;                          // the robot's own estimate of where it is
    int walks_started = 0, walks_ended = 0, walks_rescued = 0;
    const int walk_tick = walk.from_s >= 0.0 ? int(walk.from_s * kBrainHz) : -1;
    const char* step_event = "";

    std::vector<double> tilt_trace;
    std::vector<int> shove_ticks, handoff_ticks;
    const int push_period = int(pushes.every_s * kBrainHz);
    const int push_hold   = std::max(1, int(pushes.hold_s * kBrainHz));
    const int push_from   = int(pushes.from_s * kBrainHz);
    int push_index = 0, push_skipped = 0;
    if (pushes.newtons > 0.0) tilt_trace.reserve(ticks);

    TickPacer pacer;
    for (int t = 0; t < ticks; ++t) {
        pacer.wait_for(t, kBrainHz);
        Driver driver = recovery.update(body.gravity(), body.gyro(), dt);

        // Diagnostic consolidation freeze (see g_freeze_after_ticks).
        static bool frozen_forever = false;
        if (g_freeze_after_ticks > 0 && !frozen_forever && t >= g_freeze_after_ticks) {
            brain.set_learning(false);
            frozen_forever = true;
            std::fprintf(stderr, "  [freeze-after] all learning frozen at tick %d\n", t);
        }

        // Identification-episode scheduling (see the note above the function).
        if (ident_every > 0 && driver == Driver::Brain) {
            if (settle_left > 0) {
                const auto g = body.gravity();
                const auto w = body.gyro();
                const bool still = g[2] < -0.999
                                   && std::max({std::fabs(w[0]), std::fabs(w[1]),
                                                std::fabs(w[2])}) < 0.15;
                --settle_left;
                if (still || settle_left == 0) {
                    settle_left = 0;
                    brain.on_reset();
                    if (!frozen_forever) brain.set_learning(true);
                } else {
                    driver = Driver::Scaffold;   // host override: keep settling
                }
            } else if (brain_ticks_seen < ident_until
                       && brain_ticks_seen > 0
                       && brain_ticks_seen % ident_every == 0) {
                settle_left = 100;               // up to 2 s; usually ends at stillness
                brain.set_learning(false);
                brain.on_reset();
                driver = Driver::Scaffold;
            }
            if (driver == Driver::Brain) ++brain_ticks_seen;
        }

        // The step hand-off (see StepPlan).  Only while the brain would otherwise
        // drive: a rescue in progress outranks it, and a rescue that starts during
        // a step ends the step (counted below).
        stepping = false;
        step_event = "";
        if (!walker && (walk_tick >= 0 || step.lean_deg > 0.0 || step.att > 0.0))
            walker = std::make_unique<Policy>(kWalkScaffold);
        // A walk on request starts an excursion exactly like a step, with its own twist.
        if (walk_tick >= 0 && t == walk_tick && driver == Driver::Brain && step_left == 0) {
            step_twist = {walk.vx, walk.vy, walk.vyaw};
            step_twist_left = std::max(1, int(walk.secs * kBrainHz));
            step_left = step_twist_left + step_settle_ticks;
            walking = true;
            walk_targets = body.joint_positions();
            walk_last_action.fill(0.0f);
            brain.set_learning(false);
            brain.on_reset();
            step_event = "walk:start";
            ++walks_started;
        }
        if ((step.lean_deg > 0.0 || step.att > 0.0 || step_left > 0) && driver == Driver::Brain) {
            const auto g = body.gravity();
            // The signal: the brain's own attitude error when asked for (max over its
            // MotorEPMs), else the harness's lean in degrees.  One threshold each.
            double lean = std::atan2(std::hypot(g[0], g[1]), -g[2]) * (180.0 / 3.14159265358979323846);
            double thresh = step.lean_deg;
            if (step.att > 0.0) {
                lean = 0.0;
                if (auto* og = dynamic_cast<OgmaBrainAdapter*>(&brain))
                    for (double a : og->attitude_error()) lean = std::max(lean, a);
                thresh = step.att;
            }
            if (step_left > 0) {
                --step_left;
                const auto w = body.gyro();
                const bool still = g[2] < -0.999
                                   && std::max({std::fabs(w[0]), std::fabs(w[1]), std::fabs(w[2])}) < 0.15;
                const bool twisting = step_left > step_settle_ticks;
                if (twisting) {
                    command.twist = step_twist;
                    driver = Driver::Scaffold;
                    stepping = true;
                } else if (still || step_left == 0) {
                    step_left = 0;
                    command.twist = {0.0, 0.0, 0.0};
                    brain.on_reset();
                    if (!frozen_forever) brain.set_learning(true);
                    last_action.fill(0.0f);
                    step_event = walking ? "walk:end" : "step:end";
                    if (walking) ++walks_ended; else ++steps_ended;
                    walking = false;
                } else {
                    command.twist = {0.0, 0.0, 0.0};
                    driver = Driver::Scaffold;
                    stepping = true;
                }
            } else if (step.lean_deg > 0.0 || step.att > 0.0) {
                if (lean > thresh && lean > last_lean) ++step_confirm; else step_confirm = 0;
                if (step_confirm >= step.confirm_ticks) {
                    step_confirm = 0;
                    const double n = std::hypot(g[0], g[1]);
                    step_twist = {step.twist * g[0] / n, step.twist * g[1] / n, 0.0};
                    step_left = step_twist_ticks + step_settle_ticks;
                    walk_targets = body.joint_positions();
                    walk_last_action.fill(0.0f);
                    brain.set_learning(false);
                    brain.on_reset();
                    command.twist = step_twist;
                    driver = Driver::Scaffold;
                    stepping = true;
                    step_event = "step:start";
                    ++steps_started;
                }
            }
            last_lean = lean;
        }

        // Both edges: tell the brain, and stop or start its learning. Freeze BEFORE
        // the scaffold ever acts, resume only once the body is back.
        if (recovery.handed_off_this_tick()) {
            if (step_left > 0) {              // the excursion ended in a fall
                step_left = 0;
                command.twist = {0.0, 0.0, 0.0};
                if (walking) ++walks_rescued; else ++steps_rescued;
                walking = false;
            }
            brain.set_learning(false);
            brain.on_reset();
        } else if (recovery.handed_back_this_tick()) {
            brain.on_reset();
            if (!frozen_forever) brain.set_learning(true);
            // The scaffold's own action feedback must not follow the brain back in.
            last_action.fill(0.0f);
        }
        const bool learning_now = (driver == Driver::Brain);
        if (!learning_now) ++frozen_ticks;
        // Posture-bucketed TLE, on the same -0.5 gravity threshold the harness
        // uses for "down", so the two agree about what a fall is.
        if (auto* og = dynamic_cast<OgmaBrainAdapter*>(&brain)) {
            og->sample_tle(body.gravity()[2] < -0.5);
            // The learnable-regime gate: the model learns only near-upright
            // (~25°); it always ACTS.  See the adapter's note for the measured
            // motivation (a mixture-poisoned A with half its signs wrong).
            // --no-tilt-gate disables it — the R1 regime banks are the LEARNED
            // replacement for this hand rule, and their gate is precisely
            // "identification holds without it".
            if (!g_no_tilt_gate)
                og->set_regime_learning(body.gravity()[2] < -0.90);
        }

        if (recovery.handed_off_this_tick()) handoff_ticks.push_back(t);

        // The (d) shove, on the brain's stance only (see the note above the function).
        if (pushes.newtons > 0.0 && push_period > 0 && t > 0 && t >= push_from &&
            t % push_period == 0 &&
            (ticks - t) >= int(kRecoverWindowSecs * kBrainHz)) {
            if (driver == Driver::Brain) {
                static const double dirs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
                const auto& d = dirs[push_index++ % 4];
                body.push({pushes.newtons * d[0], pushes.newtons * d[1], 0.0}, push_hold);
                shove_ticks.push_back(t);
            } else {
                ++push_skipped;
            }
        }

        std::array<double, kNumPolicyJoints> ctrl{};
        if (stepping && walker) {
            // The walker, as their runtime runs it: 0.9 scale, low-passed targets.
            const auto action = walker->infer(build_observation(body, walk_last_action, command));
            walk_last_action = action;
            for (int i = 0; i < kNumPolicyJoints; ++i) {
                const double target = kHomePose[i] + kWalkingActionScale * action[i];
                const double a = (i >= 5 && i <= 8) ? kWalkLowpassHead : kWalkLowpassLegs;
                walk_targets[i] += a * (target - walk_targets[i]);
                ctrl[i] = walk_targets[i];
            }
        } else if (driver == Driver::Scaffold) {
            const auto action = scaffold.infer(build_observation(body, last_action, command));
            last_action = action;
            for (int i = 0; i < kNumPolicyJoints; ++i)
                ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
        } else {
            ctrl = brain.act(body);
        }
        body.step(ctrl);
        {
            std::array<SitePose, 2> feet;
            body.site_pose_trunk("left_foot", feet[0].pos, feet[0].quat);
            body.site_pose_trunk("right_foot", feet[1].pos, feet[1].quat);
            odom.update(feet, body.imu_quat());
        }
        if (pushes.newtons > 0.0) tilt_trace.push_back(body.tilt_deg());

        if (emit) {
            const auto p = body.trunk_position();
            const auto g = body.gravity();
            const auto f = body.active_push();
            std::printf("{\"t\":%.3f,\"tick\":%d,\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,\"tilt\":%.3f,"
                        "\"grav\":[%.4f,%.4f,%.4f],\"push\":",
                        body.time(), t, p[0], p[1], p[2], body.tilt_deg(), g[0], g[1], g[2]);
            // Literal zeros when nothing is pushing, so a run without shoves stays
            // byte-identical to the pre-push host (the guard in §3 of CLAUDE.md).
            if (f[0] == 0.0 && f[1] == 0.0 && f[2] == 0.0) std::printf("[0,0,0]");
            else std::printf("[%.2f,%.2f,%.2f]", f[0], f[1], f[2]);
            std::printf(",\"drive\":\"%s\",\"u\":%.4f,"
                        "\"rg\":%d,\"rtle\":%.3f,"
                        "\"learning\":%s,\"event\":\"%s\",",
                        stepping ? (walking ? "walk" : "step") : driver_name(driver), brain.last_cmd_mag(),
                        brain.regime_id(), brain.regime_tle(),
                        learning_now ? "true" : "false",
                        recovery.handed_off_this_tick()    ? "reset:handoff"
                        : recovery.handed_back_this_tick() ? "reset:handback"
                        : step_event[0]                    ? step_event
                                                           : "");
            // Earned consolidation per MotorEPM module, in graph order — the state
            // the (d) test is about: it must collapse on a real perturbation and
            // re-earn itself afterwards.  Empty for a brain that has none.
            std::printf("\"cons\":[");
            if (auto* og = dynamic_cast<OgmaBrainAdapter*>(&brain)) {
                const auto cs = og->consolidation();
                for (size_t i = 0; i < cs.size(); ++i) std::printf("%s%.3f", i ? "," : "", cs[i]);
            }
            // The attitude prior's instant error per MotorEPM: the brain's own saturation
            // signal (phase 1a), logged so the step threshold can be measured on it.
            std::printf("],\"att\":[");
            if (auto* og = dynamic_cast<OgmaBrainAdapter*>(&brain)) {
                const auto as = og->attitude_error();
                for (size_t i = 0; i < as.size(); ++i) std::printf("%s%.3f", i ? "," : "", as[i]);
            }
            // The robot's own dead-reckoned pose (contact odometry, Pollen's algorithm):
            // x, y in the boot frame, yaw.  Compare with x/y/qpos above, which are truth.
            const auto op = odom.position();
            std::printf("],\"odom\":[%.4f,%.4f,%.4f],\"q\":[", op[0], op[1], odom.yaw());
            const auto q = body.joint_positions();
            for (int i = 0; i < kNumPolicyJoints; ++i) std::printf("%s%.4f", i ? "," : "", q[i]);
            std::printf("],\"qpos\":[");
            const auto full = body.qpos();
            for (size_t i = 0; i < full.size(); ++i) std::printf("%s%.6f", i ? "," : "", full[i]);
            std::printf("]}\n");
        }
    }

    if (!g_save_brain.empty()) {
        if (auto* og = dynamic_cast<OgmaBrainAdapter*>(&brain)) {
            nlohmann::json snap;
            snap["graph"] = og->brain_state();
            snap["qpos"]  = body.qpos();
            snap["qvel"]  = body.qvel();
            std::ofstream out(g_save_brain);
            out << snap;
            std::fprintf(stderr, "brain+body snapshot -> %s\n", g_save_brain.c_str());
        }
    }

    const double total = recovery.brain_seconds() + recovery.scaffold_seconds();
    std::fprintf(stderr,
                 "%s %.0f s — %d rescues, %.0f%% of the run driven by the brain, "
                 "longest recovery %.2f s\n",
                 brain.name(), seconds, recovery.rescues(),
                 100.0 * recovery.brain_seconds() / total, recovery.longest_recovery());
    std::fprintf(stderr, "  learning frozen for %.0f%% of ticks (must equal the scaffold's share)\n",
                 100.0 * frozen_ticks / ticks);
    if (recovery.stuck_rescues() > 0)
        std::fprintf(stderr, "  %d of the rescues were STUCK-POSE rescues (stable sub-trigger "
                             "tilt held %d s) rather than falls\n",
                     recovery.stuck_rescues(), 5);
    if (recovery.gave_up() > 0) {
        std::fprintf(stderr, "  %d recoveries timed out — the scaffold could not stand it up\n",
                     recovery.gave_up());
    }

    if (step.lean_deg > 0.0 || step.att > 0.0) {
        if (step.att > 0.0)
            std::fprintf(stderr, "  step hand-off on the brain's attitude error > %.3f", step.att);
        else
            std::fprintf(stderr, "  step hand-off at %.1f deg", step.lean_deg);
        std::fprintf(stderr, " (twist %.2f m/s for %.1f s): %d steps started, "
                             "%d handed back upright, %d rescued mid-step\n",
                     step.twist, step.twist_s, steps_started, steps_ended, steps_rescued);
    }
    if (walk.from_s >= 0.0) {
        std::fprintf(stderr, "  walk on request at %.1f s (vx %.2f vy %.2f vyaw %.2f for %.1f s): %d started, "
                             "%d handed back upright, %d rescued mid-walk\n",
                     walk.from_s, walk.vx, walk.vy, walk.vyaw, walk.secs, walks_started, walks_ended, walks_rescued);
    }
    if (pushes.newtons > 0.0) {
        RunResult r = analyse(tilt_trace, shove_ticks);
        int caught = 0, rescued = 0;
        for (size_t k = 0; k < r.shoves.size(); ++k) {
            auto& sh = r.shoves[k];
            const int from = shove_ticks[k];
            const int next = (k + 1 < shove_ticks.size()) ? shove_ticks[k + 1] : ticks;
            const int until = std::min({ticks, from + int(kRecoverWindowSecs * kBrainHz), next});
            for (int h : handoff_ticks) sh.rescued = sh.rescued || (h >= from && h < until);
            const char* how = !sh.conclusive           ? "INCONCLUSIVE"
                              : sh.rescued             ? "RESCUED by the scaffold"
                              : sh.recovered_after >= 0 ? "caught by the brain"
                                                       : "NOT RECOVERED";
            if (sh.conclusive && !sh.rescued && sh.recovered_after >= 0) ++caught;
            if (sh.rescued) ++rescued;
            std::fprintf(stderr, "  shove %zu at %7.2fs: peak %6.2f deg — %s", k + 1, sh.t,
                         sh.peak_tilt, how);
            if (sh.conclusive && !sh.rescued && sh.recovered_after >= 0)
                std::fprintf(stderr, ", back under %.0f deg in %.2f s", kRecoveredTiltDeg,
                             sh.recovered_after);
            std::fprintf(stderr, "\n");
        }
        std::fprintf(stderr, "  %.1f N shoves every %.1f s from %.0f s: %zu delivered, %d skipped "
                             "(landed mid-rescue) — %d caught by the brain, %d rescued, %d not judged\n",
                     pushes.newtons, pushes.every_s, pushes.from_s, shove_ticks.size(),
                     push_skipped, caught, rescued, r.inconclusive());
    }

    // What "working" means here: the body kept running. A harness that never fired
    // proves nothing, and one that never handed back has stopped being a harness.
    const bool handing_back = recovery.gave_up() < recovery.rescues();
    if (recovery.rescues() == 0) {
        // Never falling is a result, not a failure: for the stub it means nothing
        // was tested, for a real brain it would be the whole point.
        std::fprintf(stderr, "NO RESCUES — the body never went down.\n");
        return 0;
    }
    std::fprintf(stderr, "%s\n", handing_back ? "HARNESS OK" : "HARNESS STUCK");
    return handing_back ? 0 : 1;
}

int cmd_stub(const std::string& scene, double seconds, uint64_t seed, double amplitude,
             double drift, bool emit) {
    StubBrain brain(amplitude, drift, seed);
    return run_with_brain(scene, seconds, seed, brain, emit);
}

// ---------------------------------------------------------------------------
// --probe  (DIAGNOSTIC BENCH, 2026-08-31) — is a catch policy IN the linear class?
//
// Every learned arm converges to ~22 rescues/min while the trained scaffold holds
// the body with 0.0002 rad corrections.  Before concluding anything about the
// learners, the question underneath them has to be answered: does the policy
// class they search — linear feedback on (pitch, roll, rates) — contain a catch
// policy on this body AT ALL?  This mode measures each joint's authority over
// pitch/roll empirically (short pulses from the calibrated stand), then runs a
// Jacobian-transpose PD with swept hand gains under the SAME recovery harness
// and metrics as every learned arm.  A test instrument, never an operating mode:
// hand gains are exactly what the doctrine forbids shipping, and exactly what a
// ceiling measurement needs.
// ---------------------------------------------------------------------------

namespace {

class ProbeBrain final : public BrainLike {
public:
    ProbeBrain(std::array<double, kNumPolicyJoints> home,
               std::array<double, kNumPolicyJoints> jp,
               std::array<double, kNumPolicyJoints> jr,
               double kp, double kd)
        : home_(home), jp_(jp), jr_(jr), kp_(kp), kd_(kd) {}

    std::array<double, kNumPolicyJoints> act(const DuckBody& body) override {
        const auto g = body.gravity();
        const auto w = body.gyro();
        const double ep = kp_ * g[0] + kd_ * 0.3 * w[1];   // pitch error signal
        const double er = kp_ * g[1] + kd_ * 0.3 * w[0];   // roll error signal
        std::array<double, kNumPolicyJoints> target{};
        for (int i = 0; i < kNumPolicyJoints; ++i) {
            const double u = std::clamp(-(jp_[size_t(i)] * ep + jr_[size_t(i)] * er), -1.0, 1.0);
            target[size_t(i)] = home_[size_t(i)] + 0.35 * u;
        }
        return target;
    }
    const char* name() const override { return "probe"; }

private:
    std::array<double, kNumPolicyJoints> home_, jp_, jr_;
    double kp_, kd_;
};

}  // namespace

int cmd_probe(const std::string& scene, double seconds, uint64_t seed) {
    DuckBody body(scene);
    Policy scaffold(kStandScaffold);

    // 1. Calibrated stand (same as cmd_brain).
    std::array<double, kNumPolicyJoints> stand{};
    {
        body.reset("STAND", 0.0, seed);
        std::array<float, kActionLen> last_action{};
        const Command command{};
        const int settle = int(3.0 * kBrainHz), avg_from = int(2.0 * kBrainHz);
        int n_avg = 0;
        for (int t = 0; t < settle; ++t) {
            const auto action = scaffold.infer(build_observation(body, last_action, command));
            last_action = action;
            std::array<double, kNumPolicyJoints> ctrl{};
            for (int i = 0; i < kNumPolicyJoints; ++i)
                ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
            body.step(ctrl);
            if (t >= avg_from) {
                const auto q = body.joint_positions();
                for (int i = 0; i < kNumPolicyJoints; ++i) stand[size_t(i)] += q[i];
                ++n_avg;
            }
        }
        for (int i = 0; i < kNumPolicyJoints; ++i) stand[size_t(i)] /= double(n_avg);
    }

    // 2. Empirical authority: pulse each joint ±0.08 rad for 6 ticks from the
    //    settled stand and read the pitch/roll response, antisymmetrised.
    std::array<double, kNumPolicyJoints> Jp{}, Jr{};
    const double delta = 0.08;
    const int pulse_ticks = 6;
    for (int j = 0; j < kNumPolicyJoints; ++j) {
        double dp[2] = {0, 0}, dr[2] = {0, 0};
        for (int sgn = 0; sgn < 2; ++sgn) {
            // fresh settle per pulse so probes never contaminate each other
            body.reset("STAND", 0.0, seed);
            std::array<float, kActionLen> last_action{};
            const Command command{};
            for (int t = 0; t < int(2.0 * kBrainHz); ++t) {
                const auto action = scaffold.infer(build_observation(body, last_action, command));
                last_action = action;
                std::array<double, kNumPolicyJoints> ctrl{};
                for (int i = 0; i < kNumPolicyJoints; ++i)
                    ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
                body.step(ctrl);
            }
            const auto g0 = body.gravity();
            std::array<double, kNumPolicyJoints> ctrl{};
            for (int i = 0; i < kNumPolicyJoints; ++i) ctrl[i] = stand[size_t(i)];
            ctrl[size_t(j)] += (sgn ? -delta : delta);
            for (int t = 0; t < pulse_ticks; ++t) body.step(ctrl);
            const auto g1 = body.gravity();
            dp[sgn] = g1[0] - g0[0];
            dr[sgn] = g1[1] - g0[1];
        }
        Jp[size_t(j)] = (dp[0] - dp[1]) / (2.0 * delta);
        Jr[size_t(j)] = (dr[0] - dr[1]) / (2.0 * delta);
    }
    {
        double np = 0, nr = 0;
        for (int j = 0; j < kNumPolicyJoints; ++j) { np += Jp[size_t(j)] * Jp[size_t(j)];
                                                     nr += Jr[size_t(j)] * Jr[size_t(j)]; }
        np = std::sqrt(np); nr = std::sqrt(nr);
        std::fprintf(stderr, "probe authority (d gravity / d rad, normalised):\n  Jp:");
        for (int j = 0; j < kNumPolicyJoints; ++j) {
            if (np > 1e-9) Jp[size_t(j)] /= np;
            std::fprintf(stderr, " %+.2f", Jp[size_t(j)]);
        }
        std::fprintf(stderr, "\n  Jr:");
        for (int j = 0; j < kNumPolicyJoints; ++j) {
            if (nr > 1e-9) Jr[size_t(j)] /= nr;
            std::fprintf(stderr, " %+.2f", Jr[size_t(j)]);
        }
        std::fprintf(stderr, "\n");
    }

    // 3. Gain grid under the same harness and metric as every learned arm.
    std::fprintf(stderr, "\n  %6s %6s | rescues/min  brain%%\n", "kp", "kd");
    double best = 1e9; double best_kp = 0, best_kd = 0;
    for (double kp : {5.0, 10.0, 20.0, 40.0, 80.0}) {
        for (double kd : {2.0, 4.0, 6.0, 8.0}) {
            ProbeBrain brain(stand, Jp, Jr, kp, kd);
            DuckBody b2(scene);
            Policy sc2(kStandScaffold);
            Recovery recovery;
            b2.reset("STAND", 0.0, seed);
            std::array<float, kActionLen> last_action{};
            const Command command{};
            const double dt = 1.0 / kBrainHz;
            const int ticks = int(seconds * kBrainHz);
            for (int t = 0; t < ticks; ++t) {
                const Driver driver = recovery.update(b2.gravity(), b2.gyro(), dt);
                std::array<double, kNumPolicyJoints> ctrl{};
                if (driver == Driver::Scaffold) {
                    const auto action = sc2.infer(build_observation(b2, last_action, command));
                    last_action = action;
                    for (int i = 0; i < kNumPolicyJoints; ++i)
                        ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
                } else {
                    ctrl = brain.act(b2);
                }
                b2.step(ctrl);
            }
            const double rpm = recovery.rescues() * 60.0 / seconds;
            std::fprintf(stderr, "  %6.1f %6.1f | %6.1f       %3.0f%%\n", kp, kd, rpm,
                         100.0 * recovery.brain_seconds()
                             / (recovery.brain_seconds() + recovery.scaffold_seconds()));
            if (rpm < best) { best = rpm; best_kp = kp; best_kd = kd; }
        }
    }
    std::fprintf(stderr, "\nbest: kp %.1f kd %.1f -> %.1f rescues/min\n", best_kp, best_kd, best);
    // Confirmation run at the best gains through the SAME emitting path as every
    // brain arm — so the operator can watch the class ceiling (and record it).
    ProbeBrain brain(stand, Jp, Jr, best_kp, best_kd);
    return run_with_brain(scene, seconds, seed, brain, /*emit=*/true);
}

// ---------------------------------------------------------------------------
// --brain  (A1)
// ---------------------------------------------------------------------------

int cmd_brain(const std::string& scene, const std::string& graph, double seconds, uint64_t seed,
              double amplitude, bool emit, int ident_every = 0, int ident_until = 0,
              const PushPlan& pushes = {}, const StepPlan& step = {}, const WalkPlan& walk = {}) {
    DuckBody probe(scene);   // for the joint ranges the adapter reads by name

    // STAND CALIBRATION (2026-08-31).  The brain's command origin is the SCAFFOLD'S
    // measured equilibrium, not the STAND keyframe: the keyframe is up to 0.10 rad
    // from where alpha_stand actually balances, so u = 0 at the keyframe is a pose
    // the body topples from in ~0.1 s and every handback began with a step-change
    // lurch toward it.  Three scaffold-driven seconds, mean q over the final one.
    // A scaffold-derived origin is a calibration in the same category as reading
    // joint ranges from the model instead of transcribing them.
    std::vector<double> stand_home(kNumPolicyJoints, 0.0);
    std::vector<double> stand_hcom(2, 0.0);
    {
        Policy scaffold(kStandScaffold);
        probe.reset("STAND", 0.0, seed);
        std::array<float, kActionLen> last_action{};
        const Command command{};
        const int settle = int(3.0 * kBrainHz), avg_from = int(2.0 * kBrainHz);
        int n_avg = 0;
        for (int t = 0; t < settle; ++t) {
            const auto action = scaffold.infer(build_observation(probe, last_action, command));
            last_action = action;
            std::array<double, kNumPolicyJoints> ctrl{};
            for (int i = 0; i < kNumPolicyJoints; ++i)
                ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
            probe.step(ctrl);
            if (t >= avg_from) {
                const auto q = probe.joint_positions();
                for (int i = 0; i < kNumPolicyJoints; ++i) stand_home[size_t(i)] += q[i];
                const auto hc = probe.head_com_trunk();
                stand_hcom[0] += hc[0]; stand_hcom[1] += hc[1];
                ++n_avg;
            }
        }
        double dmax = 0.0;
        for (int i = 0; i < kNumPolicyJoints; ++i) {
            stand_home[size_t(i)] /= double(n_avg);
            dmax = std::max(dmax, std::fabs(stand_home[size_t(i)] - kHomePose[i]));
        }
        stand_hcom[0] /= double(n_avg); stand_hcom[1] /= double(n_avg);
        std::fprintf(stderr, "stand calibration: origin = scaffold equilibrium "
                             "(max |delta| from keyframe %.4f rad; head CoM %+0.4f %+0.4f m)\n",
                     dmax, stand_hcom[0], stand_hcom[1]);
    }

    OgmaBrainAdapter brain(probe, {graph, seed, amplitude, stand_home, stand_hcom});
    if (g_servo_filter) brain.set_servo_filter(true);

    std::fprintf(stderr, "graph %s\n  modules:", graph.c_str());
    for (const auto& id : brain.module_ids()) std::fprintf(stderr, " %s", id.c_str());
    std::fprintf(stderr, "\n");

    const int rc = run_with_brain(scene, seconds, seed, brain, emit, ident_every, ident_until,
                                  pushes, step, walk);
    std::fprintf(stderr, "  mean |action| %.4f over %llu brain ticks\n", brain.mean_abs_action(),
                 (unsigned long long)brain.ticks());
    for (const auto& line : brain.diagnostics()) std::fprintf(stderr, "  %s\n", line.c_str());
    std::fprintf(stderr, "  motor_tle upright %.4f | down %.4f  -> %s\n", brain.tle_upright(),
                 brain.tle_down(),
                 brain.tle_down() < brain.tle_upright()
                     ? "!! DOWN IS THE QUIETER STATE — check for a lying-down attractor"
                     : "upright is not the noisier state (good)");
    return rc;
}

// ---------------------------------------------------------------------------
// --level2  (the intent boundary, phase 1e): the brain one level up.  The walker
// drives the joints throughout, the level-2 brain commands its twist and senses
// the body's own velocity (contact odometry + gyro); the rescue harness stands it
// up after a fall exactly as for the joint-level brain, with the level-2 learning
// frozen and its pairing invalidated at both edges.  Identification first: the
// gate is the sign and dominance of the identified A's velocity rows per command.
// ---------------------------------------------------------------------------

int cmd_level2(const std::string& scene, const std::string& graph, double seconds, uint64_t seed,
               bool emit, const PushPlan& pushes = {}, const std::array<double, 3>* open_loop = nullptr) {
    DuckBody body(scene);
    Policy scaffold(kStandScaffold);
    Policy walker(kWalkScaffold);
    Recovery recovery;
    IntentAdapter brain(graph, seed);
    if (open_loop) brain.set_override(*open_loop);
    Odometry odom;
    const int push_period = int(pushes.every_s * kBrainHz);
    const int push_hold   = std::max(1, int(pushes.hold_s * kBrainHz));
    const int push_from   = int(pushes.from_s * kBrainHz);
    int push_index = 0, pushes_delivered = 0;

    body.reset("STAND", 0.0, seed);
    std::fprintf(stderr, "level-2 graph %s%s\n", graph.c_str(), open_loop ? "  (open-loop override)" : "");

    std::array<float, kActionLen> scaffold_last{}, walker_last{};
    std::array<double, kNumPolicyJoints> walk_targets = body.joint_positions();
    Command command{};
    const double dt = 1.0 / kBrainHz;
    const int ticks = int(seconds * kBrainHz);

    // The body's own velocity: odometry differenced in the world-of-boot frame and
    // rotated into the body frame by the odometry's own yaw; the yaw rate from the
    // gyro.  Both smoothed over ~10 ticks — contact odometry steps at anchor switches.
    std::array<double, 3> prev_odom{}; bool have_prev = false;
    std::array<double, 3> vel_body{};
    constexpr double kVelEma = 0.1;
    int frozen_ticks = 0;
    TickPacer pacer;

    for (int t = 0; t < ticks; ++t) {
        pacer.wait_for(t, kBrainHz);
        Driver driver = recovery.update(body.gravity(), body.gyro(), dt);
        if (recovery.handed_off_this_tick()) {
            brain.set_learning(false);
            brain.on_reset();
            command.twist = {0.0, 0.0, 0.0};
        } else if (recovery.handed_back_this_tick()) {
            brain.on_reset();
            brain.set_learning(true);
            walker_last.fill(0.0f);
            walk_targets = body.joint_positions();
        }
        const bool learning_now = (driver == Driver::Brain);
        if (!learning_now) ++frozen_ticks;

        // The level-2 brain ticks every host tick (it observes the rescue too, frozen).
        const auto g = body.gravity();
        const auto w = body.gyro();
        const auto a = body.accel();
        const auto twist = brain.tick(vel_body, g, w, a, odom.yaw());
        if (driver == Driver::Brain) command.twist = twist;

        if (pushes.newtons > 0.0 && push_period > 0 && t > 0 && t >= push_from && t % push_period == 0
            && driver == Driver::Brain && (ticks - t) >= int(kRecoverWindowSecs * kBrainHz)) {
            static const double dirs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
            const auto& d = dirs[push_index++ % 4];
            body.push({pushes.newtons * d[0], pushes.newtons * d[1], 0.0}, push_hold);
            ++pushes_delivered;
        }

        std::array<double, kNumPolicyJoints> ctrl{};
        if (driver == Driver::Scaffold) {
            const auto action = scaffold.infer(build_observation(body, scaffold_last, Command{}));
            scaffold_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i)
                ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
        } else {
            const auto action = walker.infer(build_observation(body, walker_last, command));
            walker_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i) {
                const double target = kHomePose[i] + kWalkingActionScale * action[i];
                const double alpha = (i >= 5 && i <= 8) ? kWalkLowpassHead : kWalkLowpassLegs;
                walk_targets[i] += alpha * (target - walk_targets[i]);
                ctrl[i] = walk_targets[i];
            }
        }
        body.step(ctrl);

        {
            std::array<SitePose, 2> feet;
            body.site_pose_trunk("left_foot", feet[0].pos, feet[0].quat);
            body.site_pose_trunk("right_foot", feet[1].pos, feet[1].quat);
            odom.update(feet, body.imu_quat());
            const auto p = odom.position();
            const double yaw = odom.yaw();
            if (have_prev) {
                const double vx_w = (p[0] - prev_odom[0]) * kBrainHz, vy_w = (p[1] - prev_odom[1]) * kBrainHz;
                const double c = std::cos(yaw), s = std::sin(yaw);
                const double vx_b = c * vx_w + s * vy_w, vy_b = -s * vx_w + c * vy_w;
                vel_body[0] += kVelEma * (vx_b - vel_body[0]);
                vel_body[1] += kVelEma * (vy_b - vel_body[1]);
            }
            vel_body[2] += kVelEma * (w[2] - vel_body[2]);
            prev_odom = {p[0], p[1], yaw};
            have_prev = true;
        }

        if (emit) {
            const auto p = body.trunk_position();
            const auto op = odom.position();
            const auto sensed = brain.last_sensed();
            const auto f = body.active_push();
            std::printf("{\"t\":%.3f,\"tick\":%d,\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,\"tilt\":%.3f,"
                        "\"grav\":[%.4f,%.4f,%.4f],\"push\":", body.time(), t, p[0], p[1], p[2], body.tilt_deg(), g[0], g[1], g[2]);
            if (f[0] == 0.0 && f[1] == 0.0 && f[2] == 0.0) std::printf("[0,0,0]");
            else std::printf("[%.2f,%.2f,%.2f]", f[0], f[1], f[2]);
            std::printf(",\"drive\":\"%s\","
                        "\"twist\":[%.3f,%.3f,%.3f],\"sensed\":[%.3f,%.3f,%.3f],"
                        "\"learning\":%s,\"event\":\"%s\",\"odom\":[%.4f,%.4f,%.4f],\"q\":[",
                        driver == Driver::Brain ? "walk" : "scaffold",
                        command.twist[0], command.twist[1], command.twist[2],
                        sensed[0], sensed[1], sensed[2],
                        learning_now ? "true" : "false",
                        recovery.handed_off_this_tick()    ? "reset:handoff"
                        : recovery.handed_back_this_tick() ? "reset:handback" : "",
                        op[0], op[1], odom.yaw());
            const auto q = body.joint_positions();
            for (int i = 0; i < kNumPolicyJoints; ++i) std::printf("%s%.4f", i ? "," : "", q[i]);
            std::printf("],\"qpos\":[");
            const auto full = body.qpos();
            for (size_t i = 0; i < full.size(); ++i) std::printf("%s%.6f", i ? "," : "", full[i]);
            std::printf("]}\n");
        }
    }

    if (!g_save_brain.empty()) {
        nlohmann::json snap;
        snap["graph"] = brain.brain_state();
        snap["qpos"]  = body.qpos();
        snap["qvel"]  = body.qvel();
        std::ofstream out(g_save_brain);
        out << snap;
        std::fprintf(stderr, "level-2 brain+body snapshot -> %s\n", g_save_brain.c_str());
    }

    // The gate: the identified A's velocity rows against the commands.  The bridge
    // lays the intent module's state out as [pos, act, delta] per motor, so row 3i
    // is the sensed velocity of axis i; column j is command j.  Column-major, as the
    // snapshot stores it.
    {
        const auto snap = brain.brain_state();
        const auto& mods = snap.at("modules");
        for (auto it = mods.begin(); it != mods.end(); ++it) {
            if (!it.value().contains("legs")) continue;
            const auto& leg = it.value().at("legs").at(0);
            const int rows = leg.at("rows_A").get<int>(), cols = leg.at("cols_A").get<int>();
            const auto A = leg.at("A").get<std::vector<double>>();
            if (rows < 9 || cols != 3) continue;
            std::fprintf(stderr, "  identified A of %s (sensed velocity row i vs command j; want a positive, "
                                 "dominant diagonal):\n", it.key().c_str());
            static const char* const kAxes[3] = {"vx  ", "vy  ", "vyaw"};
            for (int i = 0; i < 3; ++i) {
                std::fprintf(stderr, "    %s :", kAxes[i]);
                for (int j = 0; j < 3; ++j) std::fprintf(stderr, " %+.4f", A[size_t(3 * i) + size_t(rows) * size_t(j)]);
                std::fprintf(stderr, "\n");
            }
        }
    }
    if (pushes.newtons > 0.0)
        std::fprintf(stderr, "  %.1f N shoves every %.1f s from %.0f s: %d delivered\n", pushes.newtons,
                     pushes.every_s, pushes.from_s, pushes_delivered);
    const double total = recovery.brain_seconds() + recovery.scaffold_seconds();
    std::fprintf(stderr, "level-2 %.0f s — %d rescues, %.0f%% of the run walker-driven; learning frozen %.0f%%\n",
                 seconds, recovery.rescues(), 100.0 * recovery.brain_seconds() / std::max(total, 1e-9),
                 100.0 * frozen_ticks / ticks);
    for (const auto& line : brain.diagnostics()) std::fprintf(stderr, "  %s\n", line.c_str());
    return 0;
}

void usage() {
    std::printf(
        "ogma_mjhost — the Microduck MuJoCo host\n"
        "\n"
        "  ogma_mjhost --load-only [scene.xml]\n"
        "      Load and report. Gates G1 (loads unmodified), G3 (integral substeps),\n"
        "      G4 (ctrl order matches the joint names).\n"
        "\n"
        "  ogma_mjhost --hold [scene.xml] [--secs S] [--noise R] [--seed N]\n"
        "                     [--push N] [--push-every S] [--push-hold S] [--push-from S]\n"
        "      Run the standing scaffold. One JSON object per tick on stdout, a\n"
        "      summary on stderr. Exits non-zero if the robot ends up down.\n"
        "      --push shoves the trunk on a rotating heading: the cheapest form of\n"
        "      the perturb-and-recover test, and the thing an eye can judge.\n"
        "\n"
        "  ogma_mjhost --gate-g2 [scene.xml] [--secs S]\n"
        "      The settle sweep: four noise levels x three seeds, judged on TILT.\n"
        "\n"
        "  ogma_mjhost --stub [scene.xml] [--secs S] [--seed N]\n"
        "                     [--stub-amp R] [--stub-drift R]\n"
        "      A brain that falls over, the scaffold that picks it up, and the\n"
        "      hand-off between them. Tests the RECOVERY HARNESS, not the substrate.\n"
        "\n"
        "  ogma_mjhost --level2 [scene.xml] --graph L2.json [--secs S] [--seed N] [--save-brain F]\n"
        "      The brain one level up: the walker drives, the level-2 graph commands its\n"
        "      twist and senses the body's own velocity (contact odometry + gyro).  Prints\n"
        "      the identified A's velocity rows against the commands at the end.  --push works\n"
        "      here too; --l2-twist VX VY VYAW replaces the brain's command (an open-loop baseline).\n"
        "\n"
        "  ogma_mjhost --brain [scene.xml] [--graph G.json] [--secs S] [--seed N] [--amp R]\n"
        "                      [--load-brain F] [--save-brain F]\n"
        "                      [--push N] [--push-every S] [--push-hold S] [--push-from S]\n"
        "                      [--step-lean DEG | --step-att E] [--step-twist V] [--step-twist-secs S]\n"
        "                      [--walk-from S --walk-secs S --walk-vx V --walk-vy V --walk-vyaw W]\n"
        "      Phase A1: an OgmaInstance driving the joints, inside the same harness.\n"
        "      --step-lean hands the joints to the walker when the brain's lean exceeds DEG\n"
        "      and is still rising (a step along the lean at V m/s for S s, then settle and\n"
        "      hand back; learning frozen throughout).  JSONL drive \"step\", events\n"
        "      step:start / step:end.  0 = off.\n"
        "      --push runs the same shove schedule against the BRAIN's stance: shoves\n"
        "      land only on brain-driven ticks, and each is reported as caught by the\n"
        "      brain or rescued by the scaffold.  The JSONL carries the active force\n"
        "      in \"push\" and each MotorEPM's earned consolidation in \"cons\".\n"
        "\n"
        "  --realtime paces any mode to the wall clock (one tick per 20 ms); the viewer's live\n"
        "  mode passes it, so the inspector sees smooth ticks rather than pipe-sized bursts.\n"
        "\n"
        "  The scene defaults to %s\n"
        "  The standing scaffold is %s\n",
        kDefaultScene.c_str(), kStandScaffold.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    std::string scene = kDefaultScene;
    std::string mode;
    double seconds = 3.0, noise = 0.0;
    uint64_t seed = 0;
    PushPlan pushes;
    StepPlan step;
    WalkPlan walk;
    std::array<double, 3> l2_twist{};
    bool l2_open_loop = false;
    double stub_amp = 0.25, stub_drift = 0.08;
    int ident_every = 0, ident_until = 0;
    (void)0;
    std::string graph = std::string(MJ_HOST_CONFIG_DIR) + "/a1_motor_epm.json";
    double amplitude = 0.35;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(what) + " needs a value");
            return argv[++i];
        };
        if (a == "--load-only" || a == "--hold" || a == "--gate-g2" || a == "--stub" ||
            a == "--brain" || a == "--probe" || a == "--level2") {
            mode = a;
        } else if (a == "--secs") {
            seconds = std::stod(next("--secs"));
        } else if (a == "--noise") {
            noise = std::stod(next("--noise"));
        } else if (a == "--seed") {
            seed = std::stoull(next("--seed"));
        } else if (a == "--ident-every") {
            ident_every = std::stoi(next("--ident-every"));
        } else if (a == "--ident-until") {
            ident_until = std::stoi(next("--ident-until"));
        } else if (a == "--no-tilt-gate") {
            g_no_tilt_gate = true;
        } else if (a == "--freeze-after") {
            g_freeze_after_ticks = int(std::stod(next("--freeze-after")) * kBrainHz);
        } else if (a == "--servo-filter") {
            g_servo_filter = true;
        } else if (a == "--realtime") {
            g_realtime = true;
        } else if (a == "--save-brain") {
            g_save_brain = next("--save-brain");
        } else if (a == "--load-brain") {
            g_load_brain = next("--load-brain");
        } else if (a == "--push") {
            pushes.newtons = std::stod(next("--push"));
        } else if (a == "--push-every") {
            pushes.every_s = std::stod(next("--push-every"));
        } else if (a == "--push-hold") {
            pushes.hold_s = std::stod(next("--push-hold"));
        } else if (a == "--push-from") {
            pushes.from_s = std::stod(next("--push-from"));
        } else if (a == "--step-lean") {
            step.lean_deg = std::stod(next("--step-lean"));
        } else if (a == "--step-att") {
            step.att = std::stod(next("--step-att"));
        } else if (a == "--walk-from") {
            walk.from_s = std::stod(next("--walk-from"));
        } else if (a == "--walk-secs") {
            walk.secs = std::stod(next("--walk-secs"));
        } else if (a == "--walk-vx") {
            walk.vx = std::stod(next("--walk-vx"));
        } else if (a == "--walk-vy") {
            walk.vy = std::stod(next("--walk-vy"));
        } else if (a == "--walk-vyaw") {
            walk.vyaw = std::stod(next("--walk-vyaw"));
        } else if (a == "--l2-twist") {
            l2_twist[0] = std::stod(next("--l2-twist")); l2_twist[1] = std::stod(next("--l2-twist"));
            l2_twist[2] = std::stod(next("--l2-twist")); l2_open_loop = true;
        } else if (a == "--step-twist") {
            step.twist = std::stod(next("--step-twist"));
        } else if (a == "--step-twist-secs") {
            step.twist_s = std::stod(next("--step-twist-secs"));
        } else if (a == "--step-settle-secs") {
            step.settle_s = std::stod(next("--step-settle-secs"));
        } else if (a == "--step-confirm") {
            step.confirm_ticks = std::stoi(next("--step-confirm"));
        } else if (a == "--stub-amp") {
            stub_amp = std::stod(next("--stub-amp"));
        } else if (a == "--stub-drift") {
            stub_drift = std::stod(next("--stub-drift"));
        } else if (a == "--graph") {
            graph = next("--graph");
        } else if (a == "--amp") {
            amplitude = std::stod(next("--amp"));
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

    if (mode.empty()) {
        std::fprintf(stderr, "pick a mode.\n\n");
        usage();
        return 2;
    }

    try {
        if (mode == "--load-only") return cmd_load_only(scene);
        if (mode == "--hold") return cmd_hold(scene, seconds, noise, seed, pushes);
        if (mode == "--gate-g2") return cmd_gate_g2(scene, seconds);
        if (mode == "--stub") return cmd_stub(scene, seconds, seed, stub_amp, stub_drift, true);
        if (mode == "--brain") return cmd_brain(scene, graph, seconds, seed, amplitude, true,
                                                ident_every, ident_until, pushes, step, walk);
        if (mode == "--probe") return cmd_probe(scene, seconds, seed);
        if (mode == "--level2") return cmd_level2(scene, graph, seconds, seed, true, pushes,
                                                  l2_open_loop ? &l2_twist : nullptr);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 2;
}
