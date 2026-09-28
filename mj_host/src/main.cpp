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
#include <functional>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <map>
#include <random>
#include <set>
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
#include "HeadAdapter.hpp"
#include "Tof.hpp"
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
// --fast-until S (with --realtime): run unpaced until S seconds, then pace to the wall
// clock from there. For watching a level-2 run whose first 600 s are the identification
// babble: the run is the SAME run tick for tick (the pacer only sleeps), the babble goes by
// in seconds, and the tour is watched at real time.
int g_fast_until_ticks = 0;

struct TickPacer {
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    bool rebased = false;
    void wait_for(int tick, double hz) {
        if (!g_realtime) return;
        if (tick < g_fast_until_ticks) return;
        if (!rebased) {                       // the clock starts when the pacing does
            start = std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                            std::chrono::duration<double>(tick / hz));
            rebased = true;
        }
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

// STAND CALIBRATION (2026-08-31), shared by the joint-level run and the level-2 stops: the joint
// brain's command origin is the SCAFFOLD'S measured equilibrium, not the STAND keyframe (see the note
// in cmd_brain).  Three scaffold-driven seconds on a probe body, mean q over the final one.
void calibrate_stand_home(DuckBody& probe, uint64_t seed, std::vector<double>& stand_home,
                          std::vector<double>& stand_hcom) {
    stand_home.assign(kNumPolicyJoints, 0.0);
    stand_hcom.assign(2, 0.0);
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
}

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
    std::vector<double> stand_home, stand_hcom;
    calibrate_stand_home(probe, seed, stand_home, stand_hcom);

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

double g_arena_shift_s = -1.0;   // > 0: at this time move wall_px from x = 1.0 to x = 0.5 (the (d) test)
// --move NAME X Y AT_S: the playroom's (d) test — relocate a body or geom mid-run (repeatable).
struct MoveOp { std::string name; double x, y, at_s; bool done = false; };
std::vector<MoveOp> g_moves;
// THE TRAIN (chasing moving things, stage 0, 2026-09-27; the operator: "a toy train or car on a track that stops
// and starts at regular intervals").  --train SPEED RUN_S STOP_S drives the scene's `mov_train0` body around the
// closed track the scene carries in its <custom><numeric name="train_path"> block (playroom_gen.py --train):
// SPEED m/s for RUN_S seconds, still for STOP_S, repeating.  A stopped train is a remembered thing; a moving one
// is the stimulus; each transition is a (d) test.  The phase of the schedule and the start along the track are
// spread by the SEED (six seeds meet the train at six points of its cycle) unless --train-phase S pins them.
// Kinematic: the body's pose and velocity are written every tick, so the walker meets a mover that does not
// yield.  Off = byte-identical.  The record carries "train": [x, y, yaw, vx, vy, moving] -- truth, for the scorer.
struct TrainPlan { double speed = 0.0, run_s = 8.0, stop_s = 8.0, phase_s = -1.0; };
TrainPlan g_train;
// --log-movers WINDOW_S (stage 0's instrument): on every cast tick with a cloud open, the stack rule's clusters
// over only the voxels seen in the last WINDOW_S seconds (CloudMap::cluster_recent) -- "mvc": [[cx, cy, ext,
// top, ncols, hits, small, fresh, age_s, age_w_s, vacated], ...] in the cloud's frame (fresh: the share of the cluster's voxels
// first seen inside the window; age_s: their mean age; age_w_s: the same weighted by hits), "mva": [wx, wy, wyaw] the anchor's world pose (truth,
// for the scorer's labels) and "mvw": 1 for a walking cloud.  Off = byte-identical.
double g_log_movers_s = 0.0;
// A closed track: an ellipse of semi-axes a (along yaw) and b about (cx, cy), walked by ARC LENGTH so the
// train's speed is what the flag says everywhere on it.
struct TrackPath {
    double cx = 0.0, cy = 0.0, a = 0.0, b = 0.0, yaw = 0.0, length = 0.0;
    std::vector<double> cum, ang;   // cumulative arc length at each sampled parameter
    void build(const std::vector<double>& v) {
        cx = v[0]; cy = v[1]; a = v[2]; b = v[3]; yaw = v[4];
        const int n = 720;
        cum.assign(n + 1, 0.0); ang.assign(n + 1, 0.0);
        double px = 0.0, py = 0.0;
        for (int i = 0; i <= n; ++i) {
            const double th = 2.0 * M_PI * double(i) / double(n);
            const double x = a * std::cos(th), y = b * std::sin(th);
            ang[size_t(i)] = th;
            if (i) cum[size_t(i)] = cum[size_t(i - 1)] + std::hypot(x - px, y - py);
            px = x; py = y;
        }
        length = cum.back();
    }
    // world pose and unit tangent at arc length s (wrapped)
    void at(double s, double& x, double& y, double& tyaw) const {
        s = std::fmod(s, length); if (s < 0.0) s += length;
        const auto it = std::upper_bound(cum.begin(), cum.end(), s);
        const size_t i = size_t(std::max<long>(1, it - cum.begin()));
        const double f = (cum[i] - cum[i - 1]) > 1e-12 ? (s - cum[i - 1]) / (cum[i] - cum[i - 1]) : 0.0;
        const double th = ang[i - 1] + f * (ang[i] - ang[i - 1]);
        const double lx = a * std::cos(th), ly = b * std::sin(th);
        const double tx = -a * std::sin(th), ty = b * std::cos(th);
        const double c = std::cos(yaw), sn = std::sin(yaw);
        x = cx + c * lx - sn * ly; y = cy + sn * lx + c * ly;
        tyaw = std::atan2(sn * tx + c * ty, c * tx - sn * ty);
    }
};
constexpr double kClockRadPerS = 2.0 * M_PI / 20.0;   // the clock hand: one turn per 20 s, visible at the camera's rate
// --head-graph H.json: the head loop (playroom plan, H line) — a second brain whose motors are
// the walker's four head commands and whose senses are the head IMU. Absent: byte-identical.
std::string g_head_graph;
// --save-head F / --load-head F: the head brain's state alone (no body): H1 identifies the head
// on a standing body and saves; H2 loads that into a walking run with the prior on.
std::string g_save_head, g_load_head;
bool g_no_backing = false;   // --no-backing: the twist brain's forward command clamped at zero (no rear sensor)
bool g_seek_gate = false;    // --seek-gate: while the seek loop holds the reference, its target's ToF sector reads free (things phase T2)
bool g_ref_unwrap = false;   // --ref-unwrap (2026-09-19): the heading reference continuous modulo 2 pi (see IntentAdapter::set_ref_unwrap)
double g_ref_free = 0.0;     // --ref-free P (2026-09-19): a bearing into a ToF sector nearer than P is not held as the reference
bool g_stuck_progress = false;   // --stuck-progress (2026-09-19): the stall is no progress toward the reference, not low forward speed
double g_stuck_escape_s = 0.0;   // --stuck-escape SECS (2026-09-19): after a stuck stop, hold the reference at the cloud view's freest sector for SECS
double g_hr_tau = 0.0, g_hr_damp = 0.0, g_hr_gate = 1.0;   // --heading-reflex TAU DAMP GATE (2026-09-17): a hold on own yaw through action.vyaw
// SKILLS AT THE INTENT BOUNDARY (2026-09-17, register O54).  A skill is one of Pollen's one-shot networks,
// requested BY NAME and run as their daemon runs it (robotd/src/control.rs): a window of `duration` seconds
// in which the network sees an all-zero command and drives every joint at standing tuning, then the gait
// resumes from where the body was left.  The simulator stands in for the daemon; on the robot the same
// request is `robot.do{skill}`.  Nothing here is a trajectory of ours: the network is theirs and named as
// a scaffold.  --skill-on-arrive NAME fires one when the seek loop reaches its target (the first test of
// "does the thing answer"); --skill-at SECS NAME fires one on the clock (a scripted check).
std::string g_skill_on_arrive;           // "" = off
// --skill-unwind VX SECS: the daemon's `unwind` -- after a skill fired at a stop, the body backs off at VX for
// SECS, then stops again to look.  A kicked thing sits under the beak, below a level gaze (§17.36); half a
// metre back it is in the field, so the kick's OUTCOME can be seen (the outcome loop's honest signal).
double g_skill_unwind_vx = 0.0, g_skill_unwind_s = 0.0;
double g_skill_unwind_aim = 0.0;
// --skill-on-spin NAME TURNS NET SECS (2026-09-23, §17.47): the frustration the operator saw at 750-850 s of
// R72 -- the body turning in place near the blocks, play's reference turning with it -- as a detector on the
// body's own odometry: over the last SECS of walking the heading turned more than TURNS full turns while the
// dead-reckoned position moved less than NET metres.  The response is a skill by name at the boundary (the
// roulade: a novel orientation), at most one per 30 s.  Off = byte-identical.
std::string g_spin_skill; double g_spin_turns = 1.0, g_spin_net = 0.5, g_spin_secs = 20.0;
// --push-reach VX MAX_S (O59, §17.48): the push's window walks the SEEN distance -- (range + 0.15 m) / VX seconds,
// between 0.6 s and MAX_S, from the outcome loop's need topic position -- and its yaw keeps the nose on the
// thing (the unwind aim's gain).  R72's 1.2 s push at 0.25 m/s travelled 0.06 m.  Off = the table's window.
double g_push_vx = 0.0, g_push_max_s = 0.0;
// --skill-approach REACH VX (O59's last form, 2026-09-27): the peck and the kick reach a thing only from about
// 8 cm off its edge (R79: every moved thing was at 0.05-0.08 m; 7 pecks within 0.15 m moved nothing).  When
// one is asked for and the seen thing's centre is beyond REACH, the duck first STEPS onto it -- the walker at
// VX for (range - REACH) / VX seconds, nose on the thing, at most 3 s -- and the skill follows at once, from
// wherever the step left the body (the operator: contact even at the price of a fall).  Off = as before.
double g_approach_reach = 0.0, g_approach_vx = 0.0; std::string skill_after; bool skill_from_step = false;
// --skill-now (2026-09-27, the operator: "contact even if it results in a fall"): a module's request at the
// arrival tick fires AT ONCE, from the walk, with the body still closing on the thing -- instead of waiting for
// the arrival stop's hand-back (R64k measured the mid-walk kick at 28 % falls and the thing answering 19 %;
// from standing 0 falls, and R79's kicks reach only from 0.05-0.08 m).  Off = the deferred form.
bool g_skill_now = false;   // --skill-unwind-aim GAIN: the unwind yaws the nose toward the kicked thing and the look stop's sweep is centred on it
// --skill-when-down NAME (2026-09-19, the roulade experiment): when the recovery declares the body DOWN, fire
// the named skill first (its window drives the joints instead of the scaffold); if the body is not upright
// when the window ends, the scaffold's rescue continues as before.  Counts rises inside the window.
std::string g_skill_when_down;
double      g_skill_at_s = 0.0; std::string g_skill_at_name;
// A skill is a window; the ground pick (here "peck") is a PHASE the daemon drives: the observation's
// twist slots carry [cos 2*pi*phase, sin 2*pi*phase, 0] while phase runs 0 -> end_phase (0.7) over
// `period` (4 s), i.e. a 2.8 s window (robotd/src/control.rs; robotd-params DEFAULT_GROUND_PICK_END_PHASE).
// Nothing is grasped in simulation (no MJCF has the mouth hinge); the operator's framing (2026-09-18):
// the peck is another way to explore -- a reach-down, like a kick -- and the outcome loop chooses which.
// The PUSH (2026-09-22, §17.46): not a network but the walker with a forward command for a window -- the
// robot's own `move` intent, from standing, into the thing.  The one intent in the runtime that moves a thing
// every time it is pointed at one (the base walk moves objects 3.5 m a run by stumbling; a kick from standing
// one time in five).  file = nullptr marks it; push_vx is its command.
struct SkillDef { const char* name; const char* file; double duration_s; double phase_period_s; double push_vx; };
constexpr SkillDef kSkills[] = {
    {"kick_left",  "ball_kick_left.onnx",    0.5, 0.0, 0.0},
    {"kick_right", "ball_kick_right.onnx",   0.5, 0.0, 0.0},
    {"roulade",    "roulade.onnx",           1.0, 0.0, 0.0},
    {"peck",       "alpha_ground_pick.onnx", 2.8, 4.0, 0.0},
    {"push",       nullptr,                  1.2, 0.0, 0.25},
};
const SkillDef* skill_def(const std::string& n) {
    for (const auto& d : kSkills) if (n == d.name) return &d;
    return nullptr;
}
// Two INSTRUMENTS for the ToF studies (2026-09-12), both gated so every existing log stays
// byte-comparable and the physics is untouched either way:
//   --log-motor-tle   adds "mtle": the twist brain's own forward-model surprise, per tick.  The
//                     stumble channel: nothing else predicts the body while the walker drives.
//   --log-tof-cloud   adds "tofp": [[zone, x, y, z], ...] for every Hit/Floor zone on a cast
//                     tick -- the return points ALREADY in the TRUNK frame (Tof::TofZone::point),
//                     so a gaze babble at a stop composes them into an egocentric point cloud
//                     with no extra geometry.  ~1.5 kB per cast at 12.5 Hz.
bool g_log_motor_tle = false, g_log_tof_cloud = false;
// --cloud [VOXEL_M]: accumulate the stop's point cloud in the host (CloudMap) -- gravity-levelled,
// de-rotated by the duck's own odometry yaw, opened when the body comes to rest and closed when the
// stop ends.  Its reduced break profile goes to reality.proprio.cloud_in for any graph EPM that
// declares it, and its size/mass/change signal to the JSONL as "cld".  Absent: byte-identical.
double g_cloud_voxel = 0.0;      // > 0 = on
// --body-predicts (2026-09-12, §17.28's missing channel): the joint brain ticks on EVERY tick,
// not only inside a stop, so it has an honest forward-model residual while the walker drives the
// legs.  Its learning stays off throughout -- PREDICTING is not IDENTIFYING, and §17.10's
// drifting model was a model identified under another driver's closed loop, which this is not.
// Its command is never applied outside the stop it already owns.  Absent: byte-identical.
bool g_body_predicts = false;
bool   g_log_cloud_profile = false;   // --log-cloud-profile: the 36 dims per cast, for the bench
// --map-view cloud (2026-09-13, the operator: "feed the map the cloud"): the place map's view slots -- head yaw and the
// eight column ranges -- carry the stop's CLOUD as a gaze-invariant view (ogma::CloudMap::view: nearest off-floor
// return per sector over +-64 deg / 4 m) instead of the frame in front of a moving head; head yaw reads 0.  While no
// cloud is open (the walk) the slots hold the last cloud's view, so the walk is matched by its pose against the
// places the stops learned.  R52 measured why: a sweeping gaze is a view that never repeats (novel 18 -> 34 %).
bool   g_map_view_cloud = false;
bool   g_log_com = false;   // --log-com (O36): the record carries "com":[forward, left, height] of the whole body over the feet
double g_head_vor_tau = 0.0, g_head_vor_lead = 0.0;   // --head-vor TAU LEAD: the yaw reflex in the head adapter
double g_head_rate_k = 0.0, g_head_rate_tau = 0.0;    // --head-rate K TAU: the rate loop on the head's own gyro
// --head-joints (Track A at the head, 2026-09-10): the head brain's four commands become the head
// JOINT TARGETS (HOME + command), written over the walker's head outputs while the walker keeps the
// legs. The actuator is then the servo, not the policy: no policy jitter on the head, a lag of a few
// ticks instead of 120–160 ms. On the robot this needs a joints intent Pollen's daemon does not have.
bool g_head_joints = false;
double g_head_phase_lead = 0.0, g_head_phase_learn = 0.0;   // --head-phase LEAD_TICKS LEARN_S: the gait-phase feed-forward
double g_wander_bored_s = 0.0, g_wander_turn_deg = 90.0;   // --wander-bored S [--wander-turn DEG]
// --stop-every S --stop-secs S [--stop-from S]: the walk-stop-look line's stimulus (playroom plan
// §12.7, W1).  A scheduled stop: the twist is zeroed and the walker stands; once the body is still the
// LEGS are handed to the joint brain (--stop-brain CFG --stop-load CKPT, the R19 stander) if its own
// attitude error is below --stop-att (the hand-back gate, the brain's own "I know this pose"; 0 = no
// gate); the walker takes the legs back on the same signal the step hand-off uses (--stop-handoff-att,
// else --stop-handoff-lean degrees) and at the end of the stop.  By default the joint brain owns all
// fourteen joints at a stop (the regime it was validated in); --stop-keep-head leaves the head brain
// on the head (Track A) across the stop.  Both brains that do not drive are frozen: the twist brain's
// command is not applied during a stop and the joint brain's is not applied during a walk, so neither
// may fit the pairing (the H2 lesson: a model identified under another driver's loop drifts).  Absent:
// byte-identical.  A scaffold schedule, named as such; §12.2's stop is the map's, and this measures
// the transition it will need.
struct StopPlan {
    double every_s = 0.0, secs = 0.0, from_s = 0.0;
    double att_gate = 0.0;          // hand-back only if the joint brain's attitude error is below this
    double handoff_att = 0.0;       // > 0: the walker takes the legs back on the brain's attitude error
    double handoff_lean = 6.5;      // else on the lean, degrees (the step hand-off's own threshold)
    double settle_s = 2.0;          // at most this long waiting for stillness before the hand-back
    int    confirm_ticks = 3;
    bool   keep_head = false;
    bool   freeze_head = false;     // --stop-freeze-head: the head brain's learning off through a stop even when it keeps the head
    double scan_amp = 0.0, scan_hold_s = 1.0;
    // --stop-look AMP HOLD MAX (W3): the look in place of the scan — bearings 0, +AMP, −AMP; each held
    // at least HOLD s and, while the map calls the view novel, up to MAX s; a full round with no
    // novelty ends the stop early (the stop's length becomes the map's, not the schedule's).
    double look_amp = 0.0, look_hold_s = 1.0, look_max_s = 4.0;
    bool   map_on_stop = false;     // --map-on-stop: the map EPM learns only while the body stands and looks
    // --stop-gaze YAW PITCH HOLD MAX QUIET (W3b, the operator: babble, don't scan): at a stop the gaze takes
    // random steps (sd YAW, PITCH rad; pitch below level only up to PITCH, above it to PITCH/3) inside the
    // head's range, each held HOLD s and, while the view's winner is unbaked, up to MAX s; QUIET consecutive
    // known gazes end the stop.  The move is exploration, the dwell is the map's error.  PITCH 0 = yaw only.
    double gaze_yaw_sd = 0.0, gaze_pitch_sd = 0.0, gaze_hold_s = 0.5, gaze_max_s = 6.0; int gaze_quiet = 6;
    // --stop-gaze-residual K: the dwell's novelty is the token's residual against its own expectation —
    // the view is held while quant_error > K × expected_error (the channel's running TLE) — instead of the
    // bake flag, which saturates once baking is fast (R43).  0 = the bake flag.
    double gaze_residual_k = 0.0;
    // --stop-gaze-learn F: the dwell's signal is LEARNING PROGRESS — a view that surprised the map on arrival
    // (quant_error > K × expected_error, K from --stop-gaze-residual) is held while its error is still above
    // F × its arrival value (the prototype has not yet moved to it), up to MAX.  A known view ends at HOLD.
    // R44: the plain residual dwell is a knife edge (K 1.0 → the duck stands two thirds of the run and the
    // stops never shorten; 1.5 → 10 s stops); progress is what the map's learning actually produces.
    double gaze_learn_frac = 0.0;
    // --stop-gaze-slew RAD_PER_S (2026-09-12, O36's own hypothesis + §17.28's need): the gaze
    // override moves toward each new bearing at this rate instead of stepping to it.  0 = step.
    double gaze_slew = 0.0;
    // --stop-gaze-sweep SPEED YAW_MAX (2026-09-13, the operator watching R46: the babble "moves to an angle,
    // pauses, moves to another angle, pauses" and never covers the angles the head can traverse, while the
    // cloud could accumulate the whole time the head moves).  The gaze never holds.  It moves at SPEED rad/s
    // toward a cell of a yaw x pitch grid over its range (0.1 rad cells; yaw within +-YAW_MAX, pitch within
    // the babble's band), the target drawn at random among the cells this stop has looked at LEAST -- the
    // error it descends is the stop's own coverage deficit, so it neither replays a fixed scan nor babbles
    // back over what it has seen.  Arriving draws the next.  The hold's novelty rule runs unchanged on
    // HOLD-long windows of the moving view; the dwell becomes speed: a novel window slows the move to a
    // quarter (up to MAX s) instead of freezing it, and a window with nothing novel counts toward QUIET as a
    // known gaze did.  At 12.5 Hz of ToF and 0.3 rad/s the beams advance 1.4 deg a frame against 5.6 deg
    // between them.  Needs --stop-gaze.  0 = step-and-hold, byte-identical.
    double gaze_sweep = 0.0, gaze_sweep_yaw = 0.7;
    // --stop-gaze-sweep-slow F: the sweep's speed while the map finds the window's view novel, as a fraction of SPEED.
    // 0.25 is R52; 1 = one speed throughout.  R52's head sat at the quarter speed on 69 % of stop ticks, because a
    // moving frame kept the map surprised -- the slow-down, not SPEED, set how fast it looked around.
    double gaze_sweep_slow = 0.25;
    // --stop-cloud-end F (2026-09-13, the operator: "end stops on cloud growth"): a stop ends once the open cloud's
    // growth -- new voxels over the last 2 s -- has stayed below F x the highest growth this stop has shown, for a
    // further 2 s.  Scale-free: each stop is judged against its own peak, never a voxel count.  It replaces the
    // gaze's quiet rule; a cloud filed mid-stop (the trunk moved) starts the judgement over.  0 = off, byte-identical.
    double cloud_end_frac = 0.0;
    // --stop-gaze-down RAD (2026-09-12, §17.28's geometry): the CENTRE of the pitch babble,
    // positive down.  Widening the babble does not aim it: at sd 0.2 the gaze already reaches
    // 23 deg down, and that looks at the floor 0.2 m from the duck's feet, where nothing is.
    // A 4 cm block at 1 m sits ~7 deg below level, so what finds it is a NARROW band held
    // there, not a wide sweep.  0 = centred on level, byte-identical.
    double gaze_down = 0.0;
    bool   on_arrive = false;      // --stop-on-arrive (things phase T4): a stop starts when the seek loop reaches its target
    bool   gaze_at_thing = false;  // --stop-gaze-at-thing (T3): at an arrival stop the sweep's band is centred on the reached thing's bearing and elevation
    double on_stuck = 0.0;         // --stop-on-stuck K: a stop starts when a forward stall exceeds K x the body's own median stall (0 = off)
    // --stop-orient K TURN_VX WALK_VX SECS (the orienting reflex, agreed 2026-09-12): while the gaze is still, a
    // view whose winner switches to an EXISTING node, or whose error jumps K spreads above the hold's own
    // running mean (sampled at the map's rate, the spread floored at 5 % of the mean, two samples in a row),
    // means the world changed, not the duck.  The stop ends, the body PIVOTS to the gaze's bearing — the walker
    // does not turn on a yaw command alone (0.03 rad/s); at TURN_VX m/s with full yaw it turns 0.5-0.8 rad/s
    // nearly in place — then walks (WALK_VX, a P on the dead-reckoned yaw) until the ToF sees a hit ahead
    // within 0.4 m (arrived -> a new stop, to look at it) or SECS pass.  The twist brain is frozen through it.
    // A scaffold approach controller, named as such (W5's turn-in-place).
    double orient_k = 0.0, orient_turn_vx = 0.2, orient_vx = 0.25, orient_secs = 8.0;
    // --roll-past DELAY SPEED: at every stop, DELAY s after the hand-back, a ball (obj_ball0) is placed 1.2 m
    // along the gaze, half a metre to its right, and rolls left across the view at SPEED m/s -- the stimulus.
    double roll_delay_s = 0.0, roll_speed = 0.0;
    // --walk-past DELAY SPEED: as --roll-past, but a person-sized mover — furn_chair0 carried across the gaze
    // at SPEED m/s (a static body moved every tick), 1.2 m out, from 0.6 m right to 0.6 m left; then put back.
    double walk_delay_s = 0.0, walk_speed = 0.0;   // --stop-scan AMP HOLD: the look-around stimulus at a stop (W2) — the
                                                // head brain's yaw steps through 0, +AMP, 0, −AMP, each held HOLD s,
                                                // from the hand-back (or the walker's hold) to the stop's end. A
                                                // scaffold for the channel; §12.2's target is the map's residual (W3).
};
StopPlan g_stop;
std::string g_stop_brain, g_stop_load;

int cmd_level2(const std::string& scene, const std::string& graph, double seconds, uint64_t seed,
               bool emit, const PushPlan& pushes = {}, const std::array<double, 3>* open_loop = nullptr,
               double reset_noise = 0.0) {
    DuckBody body(scene);
    Policy scaffold(kStandScaffold);
    Policy walker(kWalkScaffold);
    Recovery recovery;
    IntentAdapter brain(graph, seed);
    if (open_loop) brain.set_override(*open_loop);
    if (g_wander_bored_s > 0.0) brain.set_wander(g_wander_bored_s, g_wander_turn_deg, seed);
    if (g_no_backing) brain.set_no_backing(true);
    if (g_stuck_progress) { brain.set_stuck_progress(true); std::fprintf(stderr, "  stuck by progress: a stall is no progress toward the reference (a slide along a surface counts)\n"); }
    if (g_stuck_escape_s > 0.0) std::fprintf(stderr, "  stuck escape: after a stuck stop the reference is held at the cloud view's freest sector for %.1f s\n", g_stuck_escape_s);
    if (g_ref_free > 0.0) { brain.set_ref_free(g_ref_free); std::fprintf(stderr, "  ref free-space gate: a bearing into a ToF sector with proximity above %.2f releases the reference\n", g_ref_free); }
    if (g_ref_unwrap) { brain.set_ref_unwrap(true); std::fprintf(stderr, "  ref unwrap: the heading reference is continuous modulo 2 pi (a bearing behind the body no longer flips it)\n"); }
    if (g_seek_gate) { brain.set_seek_gate(true); std::fprintf(stderr, "  seek gate: the seek target's ToF sector reads free while seek holds the reference\n"); }
    if (g_stop.on_arrive) std::fprintf(stderr, "  stop on arrive: a stop starts when the seek loop reaches its target (the timer stays as the floor)\n");
    if (g_stop.gaze_at_thing) std::fprintf(stderr, "  gaze at the thing: at an arrival stop the sweep's pitch band is centred on the reached thing's elevation (+-0.12 rad) and its bearing\n");
    if (g_stop.on_stuck > 0.0) { brain.set_stuck(g_stop.on_stuck); std::fprintf(stderr, "  stop on stuck: a stop starts when a forward stall exceeds %.1f x the body's own median stall length\n", g_stop.on_stuck); }
    if (!g_skill_on_arrive.empty()) std::fprintf(stderr, "  skill on arrive: %s (Pollen's network, a window at standing tuning with a zero command) fired from standing at the arrival stop's hand-back\n", g_skill_on_arrive.c_str());
    if (g_skill_at_s > 0.0) std::fprintf(stderr, "  skill at %.1f s: %s\n", g_skill_at_s, g_skill_at_name.c_str());
    if (!g_skill_when_down.empty()) std::fprintf(stderr, "  skill when down: %s drives the joints when the body goes down, before the scaffold's rescue\n", g_skill_when_down.c_str());
    if (g_skill_unwind_s > 0.0) std::fprintf(stderr, "  skill unwind: after a skill at a stop the body backs off at %.2f for %.1f s, then stops to look\n", g_skill_unwind_vx, g_skill_unwind_s);
    if (g_skill_unwind_aim > 0.0 && g_skill_unwind_s <= 0.0) throw std::runtime_error("--skill-unwind-aim needs --skill-unwind (the unwind it aims)");
    if (!g_spin_skill.empty() && !skill_def(g_spin_skill)) throw std::runtime_error("--skill-on-spin: unknown skill " + g_spin_skill);
    if (g_push_vx > 0.0) std::fprintf(stderr, "  push reach: the push walks the seen distance at %.2f m/s (window (range + 0.15) / vx, 0.6 to %.1f s) with its nose on the thing\n", g_push_vx, g_push_max_s);
    if (g_approach_reach > 0.0) std::fprintf(stderr, "  approach: a kick or a peck asked for with the thing beyond %.2f m begins as a step onto it (%.2f m/s, at most 3 s)\n", g_approach_reach, g_approach_vx);
    if (g_skill_now) std::fprintf(stderr, "  skill now: a request at the arrival tick fires at once, from the walk (contact even at the price of a fall)\n");
    if (!g_spin_skill.empty()) std::fprintf(stderr, "  skill on spin: %s when the heading turns more than %.1f turns in %.0f s of walking with under %.2f m of travel (at most one per 30 s)\n", g_spin_skill.c_str(), g_spin_turns, g_spin_secs, g_spin_net);
    if (g_skill_unwind_aim > 0.0) std::fprintf(stderr, "  unwind aim: the unwind's yaw keeps the nose on the kicked thing (gain %.2f on its bearing) and the look stop's sweep is centred on it\n", g_skill_unwind_aim);
    if (g_hr_tau > 0.0) { brain.set_heading_reflex(g_hr_tau, g_hr_damp, g_hr_gate);
        std::fprintf(stderr, "  heading reflex: while a loop holds the reference, vyaw closes the heading error in %.2f s (damping %.2f on the sensed rate), mixed with the brain's yaw by proximity (gate %.2f)\n", g_hr_tau, g_hr_damp, g_hr_gate); }
    Odometry odom;
    Tof tof;                                  // the 8x8 depth matrix, cast every 4 ticks (12.5 Hz, the real sensor's rate)
    // The cloud lives in the graph now (ogma::CloudMap).  The host's job is to hand it one cast
    // at a time and to read its numbers back out for the log; --cloud turns the PUBLICATION on.
    const bool cloud_on = g_cloud_voxel > 0.0;
    if (cloud_on)
        std::fprintf(stderr, "  cloud: publishing ToF return points on reality.proprio.tof_points%s\n",
                     brain.cloud_present() ? " (a CloudMap module is listening)" : " — NO CloudMap in the graph, nothing will accumulate");
    // THINGS: the graph asked CloudMap for a things or thing-bearing topic (the things phase, T1); the host
    // then logs the attended thing ("thg"), the thing EPM ("tepm") and the filed clusters ("things").
    const bool things_on = cloud_on && brain.cloud_things_on();
    if (things_on) std::fprintf(stderr, "  things: CloudMap runs the stack rule on the open cloud; logging thg / tepm / things\n");
    std::array<float, 4> tof_summary{};
    PlaceInputs place{};                      // the pose and the ToF in both reductions; the adapter picks the form
    std::fprintf(stderr, "place vector: %s\n", brain.place_form_desc().c_str());
    int tof_ticks = 0;
    bool shifted = false;
    const int push_period = int(pushes.every_s * kBrainHz);
    const int push_hold   = std::max(1, int(pushes.hold_s * kBrainHz));
    const int push_from   = int(pushes.from_s * kBrainHz);
    int push_index = 0, pushes_delivered = 0;

    body.reset("STAND", reset_noise, seed);   // l2_sweep: --noise varies the start (was hardcoded 0: seeds only seeded the babble)
    std::fprintf(stderr, "level-2 graph %s%s\n", graph.c_str(), open_loop ? "  (open-loop override)" : "");
    {
        // A generated scene carries a manifest beside it (playroom_gen.py): echo its seed and
        // hash so no two "varied" rooms can silently share a layout (the Cell's pillar trap).
        const std::string man = scene.substr(0, scene.rfind('.')) + ".manifest.json";
        std::ifstream in(man);
        if (in) {
            try {
                nlohmann::json j; in >> j;
                std::fprintf(stderr, "scene manifest: seed %lld  sha %s  objects %zu  half %.2f m\n",
                             (long long)j.value("seed", -1), j.value("xml_sha256", "?").c_str(),
                             j.value("objects", nlohmann::json::array()).size(), j.value("half", 0.0));
            } catch (const std::exception& e) {
                std::fprintf(stderr, "scene manifest: unreadable (%s)\n", e.what());
            }
        }
    }
    const bool has_clock = body.has_joint("clock_hand");
    // the train: its track from the scene, its schedule from the flag, its phase from the seed
    const bool train_on = g_train.speed > 0.0;
    TrackPath track;
    double train_s = 0.0, train_x = 0.0, train_y = 0.0, train_yaw = 0.0, train_vx = 0.0, train_vy = 0.0;
    bool train_moving = false;
    double train_phase = 0.0;
    if (train_on) {
        const auto v = body.numeric("train_path");
        if (v.size() < 5) throw std::runtime_error("--train: the scene has no train_path (generate it with playroom_gen.py --train)");
        track.build(v);
        const double cycle = g_train.run_s + g_train.stop_s;
        train_phase = g_train.phase_s >= 0.0 ? g_train.phase_s : double(seed % 6) * cycle / 6.0;
        train_s = g_train.phase_s >= 0.0 ? 0.0 : double(seed % 6) * track.length / 6.0;
        track.at(train_s, train_x, train_y, train_yaw);
        body.place_free_body("mov_train0", train_x, train_y, 0.03, train_yaw, 0.0, 0.0, 0.0);
        std::fprintf(stderr, "  train: %.2f m/s, runs %.0f s / stops %.0f s (cycle %.0f s), track centre (%.2f, %.2f) %.2f x %.2f m yaw %.2f, "
                             "perimeter %.2f m; phase %.1f s, start %.2f m along (seed %llu)\n",
                     g_train.speed, g_train.run_s, g_train.stop_s, cycle, track.cx, track.cy, track.a, track.b, track.yaw,
                     track.length, train_phase, train_s, (unsigned long long)seed);
    }
    const bool chase_on = brain.chase_present();
    if (chase_on) std::fprintf(stderr, "  chase: the seek loop chases a mover of the cloud (young voxels, confirmed by its own prediction); logging \"chase\"\n");
    if (g_log_movers_s > 0.0) std::fprintf(stderr, "  movers: logging the cloud's clusters through a %.2f s recency window on every cast (mvc / mva / mvw)\n", g_log_movers_s);
    const bool has_objects = body.n_objects() > 0;
    std::unique_ptr<HeadAdapter> head;
    if (!g_head_graph.empty()) {
        head = std::make_unique<HeadAdapter>(g_head_graph, seed);
        if (g_head_joints) std::fprintf(stderr, "head joints: the head brain writes the four head joint targets (Track A at the head)\n");
        if (g_head_phase_lead > 0.0) { head->set_phase(g_head_phase_lead, g_head_phase_learn);
            std::fprintf(stderr, "head phase feed-forward: lead %.0f ticks, learning %.0f s after the babble\n", g_head_phase_lead, g_head_phase_learn); }
        if (g_head_rate_k > 0.0) { head->set_rate_loop(g_head_rate_k, g_head_rate_tau);
            std::fprintf(stderr, "head rate loop: K %.2f, tau %.2f s\n", g_head_rate_k, g_head_rate_tau); }
        if (g_head_vor_tau > 0.0) { head->set_vor(g_head_vor_tau, g_head_vor_lead);
            std::fprintf(stderr, "head VOR: tau %.2f s, lead %.3f s\n", g_head_vor_tau, g_head_vor_lead); }
        std::fprintf(stderr, "head graph %s  (head gyro sensor: %s)\n", g_head_graph.c_str(),
                     body.has_head_gyro() ? "present" : "ABSENT — the scene has no head IMU; head gyro slots read zero");
        if (!g_load_head.empty()) {
            std::ifstream in(g_load_head);
            if (!in) throw std::runtime_error("--load-head: cannot open " + g_load_head);
            nlohmann::json snap; in >> snap;
            head->restore_brain_state(snap.at("graph"));
            std::fprintf(stderr, "head brain restored from %s\n", g_load_head.c_str());
        }
    }

    std::array<float, kActionLen> scaffold_last{}, walker_last{};
    // the skill window (see g_skill_on_arrive): the network, loaded on first use; the window's own last action
    std::unique_ptr<Policy> skill_policy; std::string skill_policy_name;
    bool skill_active = false; int skill_left = 0, skill_total = 0; std::string skill_name; std::array<float, kActionLen> skill_last{};
    double skill_phase_period = 0.0; double skill_push_vx = 0.0;
    int skills_fired = 0, skills_requested = 0; bool skill_then_stop = false; std::string skill_pending;
    int approaches = 0;                              // --skill-approach: steps taken onto a thing before a kick or a peck
    int unwind_left = 0, unwinds = 0; bool unwind_then_stop = false;
    int down_skills = 0, down_rises = 0; bool down_skill_running = false; int down_rise_watch = 0;
    std::function<bool(const std::string&)> skill_start;
    skill_start = [&](const std::string& name) -> bool {
        const SkillDef* d = skill_def(name);
        if (!d) return false;
        // the closing step: a kick or a peck asked for with the thing seen beyond its reach begins as a step onto it
        if (g_approach_reach > 0.0 && d->file && (name == "peck" || name.rfind("kick", 0) == 0) && brain.thing_pos_present()
            && brain.thing_range() > g_approach_reach && skill_after.empty() && !skill_from_step) {   // one step per request (R81: a loop of 300)
            const SkillDef* push = skill_def("push");
            skill_active = true; skill_name = "approach"; skill_last.fill(0.0f); skill_phase_period = 0.0;
            skill_push_vx = g_approach_vx > 0.0 ? g_approach_vx : push->push_vx;
            // the window is a ceiling: the step ends by the odometry when the centre is within reach; the walker from
            // the stand makes ~0.1 m/s over its first seconds, so the ceiling is sized on that, at most 3 s
            // ...and on the turn first: the thing is 37-49 deg off the nose when a peck is asked for (R81b), and a
            // peck straight ahead misses a thing beside it -- the step faces it (in place) before it walks
            const double secs = std::clamp(std::fabs(brain.thing_ego()) / 0.8 + (brain.thing_range() - g_approach_reach) / 0.1, 0.5, 4.0);
            skill_left = std::max(1, int(secs * kBrainHz)); skill_total = skill_left;
            skill_after = name; ++approaches;
            return true;
        }
        if (d->file && skill_policy_name != name) { skill_policy = std::make_unique<Policy>(kModelDir + "/scaffolds/" + d->file); skill_policy_name = name; }
        skill_active = true; skill_left = std::max(1, int(d->duration_s * kBrainHz)); skill_total = skill_left; skill_name = name; skill_last.fill(0.0f);
        skill_phase_period = d->phase_period_s; skill_push_vx = d->file ? 0.0 : d->push_vx;
        if (!d->file && g_push_vx > 0.0) {
            skill_push_vx = g_push_vx;
            const double secs = brain.thing_pos_present() ? std::clamp((brain.thing_range() + 0.15) / g_push_vx, 0.6, g_push_max_s) : d->duration_s;
            skill_left = std::max(1, int(secs * kBrainHz)); skill_total = skill_left;
        }
        ++skills_fired;
        return true;
    };
    std::array<double, kNumPolicyJoints> walk_targets = body.joint_positions();
    Command command{};
    const double dt = 1.0 / kBrainHz;
    const int ticks = int(seconds * kBrainHz);

    // The joint brain at the stops (W1).  Calibrated and restored exactly as --brain does it; its
    // learning is off until it drives.
    const bool stop_on = g_stop.every_s > 0.0 && g_stop.secs > 0.0;
    std::unique_ptr<OgmaBrainAdapter> stander;
    const bool stop_walker_holds = (g_stop_brain == "walker");   // the control: the same stops, the walker stands them
    if (stop_on && stop_walker_holds) {
        std::fprintf(stderr, "stops: every %.0f s for %.0f s from %.0f s; the WALKER holds them (the control arm)\n",
                     g_stop.every_s, g_stop.secs, g_stop.from_s);
    } else if (stop_on) {
        if (g_stop_brain.empty()) throw std::runtime_error("--stop-every needs --stop-brain CFG (or 'walker' for the control)");
        DuckBody probe(scene);
        std::vector<double> stand_home, stand_hcom;
        calibrate_stand_home(probe, seed, stand_home, stand_hcom);
        stander = std::make_unique<OgmaBrainAdapter>(probe, OgmaBrainAdapter::Config{g_stop_brain, seed, 0.35, stand_home, stand_hcom});
        if (g_servo_filter) stander->set_servo_filter(true);
        if (!g_stop_load.empty()) {
            std::ifstream in(g_stop_load);
            if (!in) throw std::runtime_error("--stop-load: cannot open " + g_stop_load);
            nlohmann::json snap; in >> snap;
            stander->restore_brain_state(snap.at("graph"));
        }
        stander->set_learning(false);
        if (g_stop.keep_head) stander->freeze_module("motor_epm_head");   // the head is the head brain's: this module's commands are not applied
        std::fprintf(stderr, "stops: every %.0f s for %.0f s from %.0f s; the joint brain %s%s%s takes the legs when still",
                     g_stop.every_s, g_stop.secs, g_stop.from_s, g_stop_brain.c_str(),
                     g_stop_load.empty() ? "" : " restored from ", g_stop_load.c_str());
        if (g_stop.att_gate > 0.0) std::fprintf(stderr, " and its attitude error < %.3f", g_stop.att_gate);
        std::fprintf(stderr, "; the walker takes them back %s%.3g%s; head %s\n",
                     g_stop.handoff_att > 0.0 ? "on attitude error > " : "past ",
                     g_stop.handoff_att > 0.0 ? g_stop.handoff_att : g_stop.handoff_lean,
                     g_stop.handoff_att > 0.0 ? "" : " deg of lean",
                     g_stop.keep_head ? "stays the head brain's" : "is the joint brain's during the stand");
    }
    enum class StopPhase { None, Settle, Brain, Walker };
    StopPhase stop_phase = StopPhase::None;
    const int stop_period = int(g_stop.every_s * kBrainHz), stop_ticks = int(g_stop.secs * kBrainHz);
    const int stop_from = int(g_stop.from_s * kBrainHz), stop_settle_ticks = std::max(1, int(g_stop.settle_s * kBrainHz));
    int stop_left = 0, stop_settle_left = 0, stop_confirm = 0;
    int stops_started = 0, stop_handbacks = 0, stop_refused = 0, stop_handoffs = 0, stop_rescued = 0, stop_survived = 0;
    int stops_arrive = 0;                                   // of those, started by the seek loop's arrival (T4)
    int stops_stuck = 0;                                    // ...and by a stall longer than the body's own (the stuck stop)
    long stand_ticks = 0;
    double stop_last_lean = 0.0;
    const char* stop_event = "";
    const bool scan_on = stop_on && g_stop.scan_amp > 0.0;
    if (scan_on && !head) throw std::runtime_error("--stop-scan needs --head-graph (the head brain carries the yaw)");
    if (scan_on) std::fprintf(stderr, "  scan at stops: head yaw 0, %+.2f, 0, %+.2f rad, each held %.1f s\n", g_stop.scan_amp, -g_stop.scan_amp, g_stop.scan_hold_s);
    const int scan_hold_ticks = std::max(1, int(g_stop.scan_hold_s * kBrainHz));
    int scan_idx = 0, scan_left = 0; double scan_target = 0.0; bool scanning = false;
    auto scan_stop = [&]() { scanning = false; scan_target = 0.0; if (head) head->set_yaw_override(false, 0.0); };
    const bool look_on = stop_on && g_stop.look_amp > 0.0;
    if (look_on && !head) throw std::runtime_error("--stop-look needs --head-graph");
    if (look_on && scan_on) throw std::runtime_error("--stop-look and --stop-scan are alternatives");
    if (look_on) std::fprintf(stderr, "  look at stops: head yaw 0, %+.2f, %+.2f rad; hold %.1f s, up to %.1f s while the map calls the view novel; a quiet round ends the stop\n",
                              g_stop.look_amp, -g_stop.look_amp, g_stop.look_hold_s, g_stop.look_max_s);
    if (g_stop.map_on_stop) { brain.set_map_learning(false); std::fprintf(stderr, "  map learns only at stops (insertion, adaptation and pruning off on the walk)\n"); }
    const int look_hold_ticks = std::max(1, int(g_stop.look_hold_s * kBrainHz)), look_max_ticks = std::max(1, int(g_stop.look_max_s * kBrainHz));
    int look_idx = 0, look_held = 0; bool look_novel_seen = false, look_round_novel = false, looking = false;
    int saccades = 0, novel_holds = 0, stops_bored = 0; long look_ticks = 0; double stop_len_sum = 0.0; int stop_len_n = 0; int stop_started_tick = 0;
    std::set<int> baked_ids;                       // the map's baked winners, from the token's just_baked (a lookup keyed by the EPM's own ids)
    int nodes_walk0 = 0, grown_walk = 0, grown_stop = 0, nodes_mark = 0;   // the gate's read-back: map growth on walks vs at stops
    auto look_stop = [&]() { looking = false; scan_target = 0.0; if (head) { head->set_yaw_override(false, 0.0); head->set_pitch_override(false, 0.0); } };
    const bool gaze_on = stop_on && g_stop.gaze_yaw_sd > 0.0;
    if (gaze_on && !head) throw std::runtime_error("--stop-gaze needs --head-graph");
    if (gaze_on && (look_on || scan_on)) throw std::runtime_error("--stop-gaze, --stop-look and --stop-scan are alternatives");
    if (g_stop.gaze_slew > 0.0) {
        if (!head) throw std::runtime_error("--stop-gaze-slew needs --head-graph");
        head->set_override_slew(g_stop.gaze_slew);
        std::fprintf(stderr, "  gaze slew: the override moves at most %.2f rad/s (O36: pitch SPEED, not pitch)\n", g_stop.gaze_slew);
    }
    if (gaze_on) std::fprintf(stderr, "  gaze babble at stops: steps sd yaw %.2f pitch %.2f rad, hold %.1f s, up to %.1f s while the view is %s; %d known gazes in a row end the stop\n",
                              g_stop.gaze_yaw_sd, g_stop.gaze_pitch_sd, g_stop.gaze_hold_s, g_stop.gaze_max_s,
                              g_stop.gaze_learn_frac > 0.0 ? "still being learned (its error above F x arrival)" : g_stop.gaze_residual_k > 0.0 ? "more surprising than the map expects" : "unbaked", g_stop.gaze_quiet);
    std::mt19937 gaze_rng(uint32_t(seed * 7919u + 17u));
    std::normal_distribution<double> gaze_n(0.0, 1.0);
    const int gaze_hold_ticks = std::max(1, int(g_stop.gaze_hold_s * kBrainHz)), gaze_max_ticks = std::max(1, int(g_stop.gaze_max_s * kBrainHz));
    double gaze_yaw = 0.0, gaze_pitch = 0.0; int gaze_quiet_run = 0;
    double gaze_qe0 = 0.0; bool gaze_arrival_novel = false;
    // the change detector (per hold, after arrival) and the reflex's state
    const bool orient_on = stop_on && g_stop.orient_k > 0.0;
    const bool roll_on = stop_on && g_stop.roll_speed > 0.0 && body.n_objects() > 0;
    const bool walk_on = stop_on && g_stop.walk_speed > 0.0 && body.n_objects() > 0;
    if (walk_on) std::fprintf(stderr, "  walk-past: %.1f s into every stop furn_chair0 is carried across the gaze at %.2f m/s\n", g_stop.walk_delay_s, g_stop.walk_speed);
    const bool sweep_on = gaze_on && g_stop.gaze_sweep > 0.0;
    if (g_stop.gaze_sweep > 0.0 && !gaze_on) throw std::runtime_error("--stop-gaze-sweep needs --stop-gaze (its pitch band, HOLD, MAX, QUIET and novelty rule)");
    if (sweep_on && (orient_on || roll_on || walk_on)) throw std::runtime_error("--stop-gaze-sweep does not drive the change detector: no --stop-orient, --roll-past or --walk-past with it");
    if (sweep_on && g_stop.gaze_slew > 0.0) throw std::runtime_error("--stop-gaze-sweep sets the override's slew itself; drop --stop-gaze-slew");
    const double sweep_p_lo0 = g_stop.gaze_pitch_sd > 0.0 ? g_stop.gaze_down - g_stop.gaze_pitch_sd * 0.7 : g_stop.gaze_down;
    const double sweep_p_hi0 = g_stop.gaze_pitch_sd > 0.0 ? g_stop.gaze_down + g_stop.gaze_pitch_sd * 2.0 : g_stop.gaze_down;
    const int sweep_ny = std::max(1, int(std::ceil(2.0 * g_stop.gaze_sweep_yaw / 0.1 - 1e-9)));
    const int sweep_np = std::max(1, int(std::ceil((sweep_p_hi0 - sweep_p_lo0) / 0.1 - 1e-9)));
    const double sweep_wy = 2.0 * g_stop.gaze_sweep_yaw / sweep_ny;
    // The band is PER STOP (T3, --stop-gaze-at-thing): at an arrival stop it is centred on the reached thing's
    // elevation (the sensor sits about 0.2 m up; a thing at the arrival range is 25-35 deg below level) and
    // its bearing; at every other stop it is the default.  The grid keeps its cell count; the width follows.
    double sweep_p_lo = sweep_p_lo0, sweep_p_hi = sweep_p_hi0, sweep_wp = (sweep_p_hi0 - sweep_p_lo0) / sweep_np, sweep_yc = 0.0;
    bool stop_is_arrive = false, stop_is_stuck = false; int escapes = 0;
    bool stop_is_look = false; int look_aimed = 0;   // the look stop after an unwind (--skill-unwind-aim)
    // the spin detector's ring of the odometry pose (x, y, unwrapped heading) over g_spin_secs
    const int spin_win = std::max(1, int(g_spin_secs * kBrainHz));
    std::vector<std::array<double, 3>> spin_ring(size_t(spin_win), std::array<double, 3>{0.0, 0.0, 0.0});
    double spin_heading = 0.0, spin_prev_yaw = 0.0; bool spin_have_yaw = false; int spin_walk_run = 0, spins = 0, spin_rolls = 0, spin_last_roll = -100000;
    std::vector<int> sweep_count(sweep_on ? size_t(sweep_ny * sweep_np) : 0, 0);
    double sweep_ty = 0.0, sweep_tp = 0.0, sweep_cover_sum = 0.0; bool sweep_have_target = false; int sweep_moves = 0, sweep_cover_n = 0;
    if (sweep_on) std::fprintf(stderr, "  gaze SWEEP at stops: never holds; %.2f rad/s (a quarter while the view is novel) toward the least-looked-at of %d x %d gaze cells, yaw +-%.2f rad, pitch %+.3f..%+.3f\n",
                               g_stop.gaze_sweep, sweep_ny, sweep_np, g_stop.gaze_sweep_yaw, sweep_p_lo, sweep_p_hi);
    if (sweep_on && g_stop.gaze_sweep_slow != 0.25)
        std::fprintf(stderr, "  gaze sweep while the view is novel: %.2f x the speed\n", g_stop.gaze_sweep_slow);
    const bool cloud_end_on = stop_on && g_stop.cloud_end_frac > 0.0;
    if (g_stop.cloud_end_frac > 0.0 && !(cloud_on && brain.cloud_present()))
        throw std::runtime_error("--stop-cloud-end needs --cloud and a CloudMap in the graph");
    if (cloud_end_on)
        std::fprintf(stderr, "  stops end on the cloud: once its growth over 2 s stays below %.2f of this stop's own peak for 2 s more (the quiet rule is off)\n",
                     g_stop.cloud_end_frac);
    const int kCloudWin = int(2.0 * kBrainHz);
    std::vector<int> cg_vox; double cg_peak = 0.0; int cg_below = 0, stops_cloud_ended = 0;
    if (g_map_view_cloud && !(cloud_on && brain.cloud_present()))
        throw std::runtime_error("--map-view cloud needs --cloud and a CloudMap in the graph");
    if (g_map_view_cloud)
        std::fprintf(stderr, "  map view: the stop's CLOUD (nearest off-floor return per sector over +-64 deg / 4 m), held through the walk; head yaw reads 0\n");
    std::array<float, 8> map_view_held; map_view_held.fill(1.0f);
    int walk_left = 0; double walk_x = 0.0, walk_y = 0.0, walk_dx = 0.0, walk_dy = 0.0; std::array<double, 2> chair_home{};
    if (walk_on) chair_home = body.body_xy("furn_chair0");
    if (orient_on) std::fprintf(stderr, "  orienting reflex: a change at a still gaze (winner switch to a known node, or error > %.1f spreads above the hold's mean) ends the stop; pivot at %.2f m/s with full yaw, walk %.2f m/s, up to %.0f s\n",
                                g_stop.orient_k, g_stop.orient_turn_vx, g_stop.orient_vx, g_stop.orient_secs);
    if (roll_on) std::fprintf(stderr, "  roll-past: %.1f s into every stop obj_ball0 rolls across the gaze at %.2f m/s\n", g_stop.roll_delay_s, g_stop.roll_speed);
    int hold_hits = 0, hold_n = 0; double hold_mean = 0.0, hold_var = 0.0; int max_id_seen = -1;
    std::array<double, Tof::kCols> hold_cols{}; std::set<int> hit_cols;
    enum class Orient { None, Turn, Walk }; Orient orient = Orient::None; double orient_bearing = 0.0; int orient_left = 0; double orient_x0 = 0.0, orient_y0 = 0.0;
    int changes = 0, changes_prompted = 0, orientations = 0, arrivals = 0, orient_timeouts = 0, rolls = 0;
    int last_roll_tick = -1000000, ticks_in_stop = 0, rolls_skipped = 0; bool rolled_this_stop = false, ball_stopped = true; double reach_sum = 0.0; int reach_n = 0; double roll_x = 0.0, roll_y = 0.0;
    const char* orient_event = "";
    auto wrap_pi = [](double a) { while (a > M_PI) a -= 2.0 * M_PI; while (a < -M_PI) a += 2.0 * M_PI; return a; };
    // the map's bookkeeping at stops (the operator, 2026-09-12: watch the baking): views inserted and baked at
    // stops, prunes, and whether a pruned id was ever baked (must stay 0 — the map's health sweep spares baked)
    int ins_stop = 0, bake_stop = 0, ins_walk = 0, bake_walk = 0, pruned_total = 0, pruned_baked = 0, node_mark2 = 0, baked_mark = 0;
    long cloud_vox_sum = 0; int cloud_n = 0;   // the cloud's own read-back, per stop
    // the world pose the open cloud was anchored on, latched on the module's open edge: the
    // viewer's only way to place a body-anchored cloud beside the room.  Instrumentation.
    double cloud_anchor_wx = 0.0, cloud_anchor_wy = 0.0, cloud_anchor_wyaw = 0.0;
    // the anchor of the cloud just FILED, for the replay payload: with the walking cloud a cloud files
    // and the next opens on the same tick (no open edge), so the open cloud's latch must not be the
    // filed cloud's pose (R70: every cloud after the first was drawn at the first one's pose, walls 45 deg off)
    double cloud_filed_wx = 0.0, cloud_filed_wy = 0.0, cloud_filed_wyaw = 0.0;
    bool   cloud_was_open = false;
    auto end_stop_drive = [&](bool to_walker) {
        // the joint brain stops driving: freeze it, invalidate its pairing, and give the walker a
        // clean start from the pose the body is actually in (as the rescue hand-back does)
        if (stander) { stander->set_learning(false); stander->on_reset(); }
        if (to_walker) { walker_last.fill(0.0f); walk_targets = body.joint_positions(); }
    };


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
        stop_event = ""; orient_event = "";
        if (recovery.handed_off_this_tick()) {
            brain.set_learning(false);
            brain.on_reset();
            if (!g_skill_when_down.empty() && !skill_active && skill_start(g_skill_when_down)) { ++down_skills; down_skill_running = true; stop_event = "skill:down"; }
            command.twist = {0.0, 0.0, 0.0};
            if (head) { head->set_learning(false); head->on_reset(); command.head = {0.0, 0.0, 0.0, 0.0}; }
            if (orient != Orient::None) { orient = Orient::None; ++orient_timeouts; }
            if (stop_phase != StopPhase::None) {          // the stop ended in a fall
                scan_stop(); if (looking) look_stop();
                if (g_stop.map_on_stop) brain.set_map_learning(false);
                if (stop_phase == StopPhase::Brain) end_stop_drive(false);
                stop_phase = StopPhase::None; stop_left = 0; ++stop_rescued; stop_event = "stop:rescued";
            }
        } else if (recovery.handed_back_this_tick()) {
            if (down_rise_watch > 0) { ++down_rises; down_rise_watch = 0; }   // upright within the watch after the roll: the roll rose
            brain.on_reset();
            brain.set_learning(true);
            walker_last.fill(0.0f);
            walk_targets = body.joint_positions();
            if (head) { head->on_reset(); head->set_learning(true); }
        }
        const bool learning_now = (driver == Driver::Brain);
        if (!learning_now) ++frozen_ticks;

        // The level-2 brain ticks every host tick (it observes the rescue too, frozen).
        const auto g = body.gravity();
        const auto w = body.gyro();
        const auto a = body.accel();
        if (g_arena_shift_s > 0.0 && !shifted && t >= int(g_arena_shift_s * kBrainHz)) {
            body.move_geom("wall_px", {0.5, 0.0, 0.15});
            shifted = true;
            std::fprintf(stderr, "  arena shift at %.0f s: wall_px moved to x = 0.5\n", g_arena_shift_s);
        }
        for (auto& mv : g_moves) {
            if (mv.done || t < int(mv.at_s * kBrainHz)) continue;
            body.move_body(mv.name.c_str(), mv.x, mv.y);
            mv.done = true;
            std::fprintf(stderr, "  move at %.0f s: %s -> (%.2f, %.2f)\n", mv.at_s, mv.name.c_str(), mv.x, mv.y);
        }
        if (has_clock) body.spin_joint("clock_hand", kClockRadPerS);
        if (train_on) {
            const double cycle = g_train.run_s + g_train.stop_s;
            const double at = std::fmod(double(t) / kBrainHz + train_phase, cycle);
            train_moving = at < g_train.run_s;
            if (train_moving) train_s += g_train.speed / kBrainHz;
            double nx, ny, nyaw;
            track.at(train_s, nx, ny, nyaw);
            const double sp = train_moving ? g_train.speed : 0.0;
            train_vx = sp * std::cos(nyaw); train_vy = sp * std::sin(nyaw);
            // the yaw rate along the track, for contacts: the heading change per metre times the speed
            double dyaw = nyaw - train_yaw; while (dyaw > M_PI) dyaw -= 2.0 * M_PI; while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            train_x = nx; train_y = ny; train_yaw = nyaw;
            body.place_free_body("mov_train0", train_x, train_y, 0.03, train_yaw, train_vx, train_vy, train_moving ? dyaw * kBrainHz : 0.0);
        }
        // The body predictor: observe every tick, frozen, so a stumble has somewhere to register.
        // Skipped while the joint brain already drives (StopPhase::Brain ticks it itself below).
        if (g_body_predicts && stander && stop_phase != StopPhase::Brain) (void)stander->act(body);
        const auto twist = brain.tick(vel_body, g, w, a, odom.yaw(), tof_summary, &place);
        // latch the world pose on the module's open edge (the anchor the viewer places a cloud at)
        if (cloud_on) {
            const bool now_open = brain.cloud_open();
            if (brain.cloud_just_closed()) { cloud_filed_wx = cloud_anchor_wx; cloud_filed_wy = cloud_anchor_wy; cloud_filed_wyaw = cloud_anchor_wyaw; }
            if (now_open && (!cloud_was_open || brain.cloud_just_closed())) {
                const auto wp = body.trunk_position();
                const auto q = body.imu_quat();
                cloud_anchor_wx = wp[0]; cloud_anchor_wy = wp[1];
                cloud_anchor_wyaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                                               1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
            }
            cloud_was_open = now_open;
        }
        if (driver == Driver::Brain) command.twist = twist;
        if (stop_on) {
            if (roll_on && !ball_stopped && t - last_roll_tick >= 75) { const auto b = body.body_xy("obj_ball0"); body.roll_body("obj_ball0", b[0], b[1], 0.0, 0.0); ball_stopped = true; }
            if (walk_on && walk_left > 0) {                       // the chair carried across, then put back home
                walk_x += walk_dx; walk_y += walk_dy; body.move_body("furn_chair0", walk_x, walk_y);
                if (--walk_left == 0) body.move_body("furn_chair0", chair_home[0], chair_home[1]);
            }
            if (brain.map_baked_now() && brain.map_winner() >= 0) baked_ids.insert(brain.map_winner());
            const int nn = brain.map_nodes();
            if (nn > nodes_mark) { (stop_phase == StopPhase::None ? grown_walk : grown_stop) += nn - nodes_mark; }
            nodes_mark = nn;
            // from the token, every tick: insertions and bakes by phase, prunes and whether a baked id died
            const int nc = brain.map_node_count(), bc = brain.map_baked_count();
            for (int pid : brain.map_pruned_ids()) { ++pruned_total; if (baked_ids.count(pid)) ++pruned_baked; }
            const int pr = int(brain.map_pruned_ids().size());
            if (nc + pr > node_mark2) (stop_phase == StopPhase::None ? ins_walk : ins_stop) += nc + pr - node_mark2;
            if (bc > baked_mark) (stop_phase == StopPhase::None ? bake_walk : bake_stop) += bc - baked_mark;
            node_mark2 = nc; baked_mark = bc;
        }
        if (stop_on && driver == Driver::Brain) {
            // T4 (things phase): a stop starts on an ERROR as well as on the timer -- the seek loop reaching the
            // thing it walked to.  The timer stays as the floor (a duck that has seen nothing still glances).
            // a skill at arrival: the window runs first, and the arrival stop (if any) starts when it ends
            bool skill_arrive_done = false;
            bool req_consumed = false;   // a module's request already started this tick (so the deferral below does not double it)
            if (skill_active && skill_left == 0 && !skill_after.empty()) {
                // the step is done: the skill it was for runs now, from where the step left the body
                const std::string nxt = skill_after; skill_after.clear(); skill_active = false;
                skill_from_step = true; if (skill_start(nxt)) stop_event = "skill:reach"; skill_from_step = false;
            }
            if (skill_active && skill_left == 0) {
                skill_active = false;
                if (down_skill_running) { down_skill_running = false; down_rise_watch = int(2.0 * kBrainHz); }
                if (skill_then_stop) { skill_then_stop = false; skill_arrive_done = true; }
                // the unwind and the look follow a skill from standing -- and, with --skill-now, one fired from the walk
                // (R82: without it the arrival stop never came, and 2 of 39 answers were seen)
                if (g_skill_unwind_s > 0.0 && (stop_phase == StopPhase::Brain || (g_skill_now && stop_phase == StopPhase::None && skill_name != "approach"))) { stop_left = 0; unwind_left = int(g_skill_unwind_s * kBrainHz); unwind_then_stop = true; ++unwinds; }
            }
            if (!skill_active && stop_phase == StopPhase::None && t >= stop_from) {
                const int req = brain.skill_request();      // a module's request through the bus, by name (id)
                if (req >= 0 && req < int(sizeof(kSkills) / sizeof(kSkills[0]))) {
                    // fired from standing, at the next stop's hand-back (a kick fired into a walk falls, §17.39);
                    // with no stop due, the daemon's way: at once
                    if (g_stop.on_arrive && !g_skill_now) { skill_pending = kSkills[req].name; stop_event = "skill:request"; ++skills_requested; }
                    else if (skill_start(kSkills[req].name)) { stop_event = "skill:request"; ++skills_requested; }
                }
                else if (!g_skill_on_arrive.empty() && brain.seek_arrived()) {
                    // The kick is fired FROM STANDING, after the arrival stop's hand-back (measured 2026-09-17:
                    // fired mid-walk, a zero-command window toppled the body on 28 % of kicks); the side follows
                    // the thing's bearing ("kick" = left when the thing is to the left).  The stop starts now.
                    skill_pending = g_skill_on_arrive == "kick" ? (brain.seek_ego() < 0.0 ? "kick_left" : "kick_right") : g_skill_on_arrive;
                    stop_event = "skill:arrive";
                }
                else if (g_skill_at_s > 0.0 && t == int(g_skill_at_s * kBrainHz) && skill_start(g_skill_at_name)) { stop_event = "skill:at"; }
                else if (!g_spin_skill.empty() && unwind_left == 0) {
                    // the spin: the body's own odometry, unwrapped, against its value spin_win ticks ago, on a run of walking ticks
                    const auto op = odom.position(); const double yw = odom.yaw();
                    if (spin_have_yaw) { double d = yw - spin_prev_yaw; while (d > M_PI) d -= 2.0 * M_PI; while (d < -M_PI) d += 2.0 * M_PI; spin_heading += d; }
                    spin_prev_yaw = yw; spin_have_yaw = true;
                    const auto old = spin_ring[size_t(t % spin_win)];
                    spin_ring[size_t(t % spin_win)] = {op[0], op[1], spin_heading};
                    ++spin_walk_run;
                    if (spin_walk_run >= spin_win && std::fabs(spin_heading - old[2]) > g_spin_turns * 2.0 * M_PI
                        && std::hypot(op[0] - old[0], op[1] - old[1]) < g_spin_net) {
                        ++spins;
                        if (t - spin_last_roll > int(30.0 * kBrainHz) && skill_start(g_spin_skill)) { ++spin_rolls; spin_last_roll = t; stop_event = "skill:spin"; }
                        spin_walk_run = 0;   // one detection per window
                    }
                }
            }
            const bool arrive_now = g_stop.on_arrive && stop_phase == StopPhase::None && t >= stop_from && !skill_active
                                    && (brain.seek_arrived() || skill_arrive_done);
            const bool stuck_now = g_stop.on_stuck > 0.0 && stop_phase == StopPhase::None && t >= stop_from && brain.stuck_now();
            if (stop_phase == StopPhase::None && ((stop_period > 0 && t >= stop_from && (t - stop_from) % stop_period == 0) || arrive_now || stuck_now)
                && (ticks - t) > stop_ticks) {
                stop_phase = StopPhase::Settle; stop_left = stop_ticks; stop_settle_left = stop_settle_ticks;
                ++stops_started; stop_event = arrive_now ? "stop:arrive" : (stuck_now ? "stop:stuck" : "stop:start"); stop_started_tick = t;
                stop_is_arrive = arrive_now; stop_is_stuck = stuck_now && !arrive_now; stop_is_look = false; spin_walk_run = 0;
                if (arrive_now) ++stops_arrive;
                if (stuck_now && !arrive_now) ++stops_stuck;
                brain.set_learning(false);                 // its command is not applied during the stop
                if (stander) stander->on_reset();          // a fresh pairing after the walk
                if (head && ((stander && !g_stop.keep_head) || g_stop.freeze_head)) head->set_learning(false);
            }
            if (stop_phase != StopPhase::None) {
                --stop_left;
                const bool still = g[2] < -0.999 && std::max({std::fabs(w[0]), std::fabs(w[1]), std::fabs(w[2])}) < 0.15;
                const double lean = std::atan2(std::hypot(g[0], g[1]), -g[2]) * (180.0 / M_PI);
                if (stop_phase == StopPhase::Settle) {
                    --stop_settle_left;
                    if (stander) (void)stander->act(body); // observes, frozen: its attitude error is then fresh
                    if (still || stop_settle_left == 0) {
                        double att = 0.0;
                        if (stander) for (double v : stander->attitude_error()) att = std::max(att, v);
                        if (scan_on) { scanning = true; scan_idx = 0; scan_left = scan_hold_ticks; }
                        if (look_on) { looking = true; look_idx = 0; look_held = 0; look_novel_seen = false; look_round_novel = false; scan_target = 0.0; }
                        if (gaze_on) { looking = true; look_held = 0; look_novel_seen = false; gaze_quiet_run = 0; gaze_yaw = 0.0; gaze_pitch = g_stop.gaze_down; scan_target = 0.0; }
                        if (sweep_on) {
                            std::fill(sweep_count.begin(), sweep_count.end(), 0); sweep_have_target = false;
                            if (g_stop.gaze_at_thing && stop_is_arrive) {
                                const double centre = std::clamp(std::atan2(0.2, std::max(0.05, brain.seek_range()) + 0.15), 0.2, 0.55);
                                sweep_p_lo = centre - 0.12; sweep_p_hi = centre + 0.12; sweep_yc = std::clamp(brain.seek_ego(), -0.5, 0.5);
                            } else if (g_skill_unwind_aim > 0.0 && stop_is_look && brain.thing_pos_present()) {
                                // the look stop after an unwind: the sweep's yaw is centred on the kicked thing's
                                // remembered bearing (the pitch band stays: T3's pitch cost the stand, §17.40)
                                sweep_p_lo = sweep_p_lo0; sweep_p_hi = sweep_p_hi0; sweep_yc = std::clamp(brain.thing_ego(), -0.5, 0.5); ++look_aimed;
                            } else { sweep_p_lo = sweep_p_lo0; sweep_p_hi = sweep_p_hi0; sweep_yc = 0.0; }
                            sweep_wp = (sweep_p_hi - sweep_p_lo) / sweep_np;
                        }
                        ticks_in_stop = 0; rolled_this_stop = false;
                        if (g_stop.map_on_stop) brain.set_map_learning(true);
                        if (!stander) {
                            stop_phase = StopPhase::Walker; stop_event = "stop:walker";
                        } else if (g_stop.att_gate <= 0.0 || att < g_stop.att_gate) {
                            stop_phase = StopPhase::Brain; ++stop_handbacks; stop_event = "stop:handback";
                            if (!skill_pending.empty()) { if (skill_start(skill_pending)) stop_event = "skill:stand"; skill_pending.clear(); }
                            stander->on_reset(); stander->set_learning(true); stop_confirm = 0; stop_last_lean = lean;
                        } else {
                            stop_phase = StopPhase::Walker; ++stop_refused; stop_event = "stop:refused";
                        }
                    }
                } else if (stop_phase == StopPhase::Brain) {
                    // a module's request DURING the stop (SkillOutcomeLoop.reach_m: it asks when it sees the thing within
                    // reach, which is at a stop): the skill starts from standing at once, as a deferred one does
                    if (!skill_active && unwind_left == 0) {
                        const int req = brain.skill_request();
                        if (req >= 0 && req < int(sizeof(kSkills) / sizeof(kSkills[0])) && skill_start(kSkills[req].name)) { stop_event = "skill:stand"; ++skills_requested; req_consumed = true; }
                    }
                    double sig = lean, thresh = g_stop.handoff_lean;
                    if (g_stop.handoff_att > 0.0) {
                        sig = 0.0; for (double v : stander->attitude_error()) sig = std::max(sig, v);
                        thresh = g_stop.handoff_att;
                    }
                    if (sig > thresh && sig > stop_last_lean) ++stop_confirm; else stop_confirm = 0;
                    stop_last_lean = sig;
                    if (stop_confirm >= g_stop.confirm_ticks) {
                        end_stop_drive(true);
                        stop_phase = StopPhase::Walker; ++stop_handoffs; stop_event = "stop:handoff";
                    }
                }
                if (!req_consumed && ((stop_phase != StopPhase::None && stop_phase != StopPhase::Brain) || (stop_phase == StopPhase::Brain && (skill_active || unwind_left > 0)))) {
                    // a request that lands while the stop settles, or during a skill's own window or its unwind, is not lost:
                    // it fires at the next hand-back like a deferred arrival request (R78 dropped 9 of 21 requests here)
                    const int req = brain.skill_request();
                    if (req >= 0 && req < int(sizeof(kSkills) / sizeof(kSkills[0])) && skill_pending.empty()) { skill_pending = kSkills[req].name; stop_event = "skill:request"; ++skills_requested; }
                }
                if (scanning) {
                    static const double kSeq[4] = {0.0, 1.0, 0.0, -1.0};
                    if (--scan_left <= 0) { scan_idx = (scan_idx + 1) % 4; scan_left = scan_hold_ticks; }
                    scan_target = kSeq[scan_idx] * g_stop.scan_amp;
                    head->set_yaw_override(true, scan_target);
                }
                if (looking && gaze_on && sweep_on) {
                    // the gaze where the head has actually got to (the slewed override), counted into its cell
                    const auto hc = head->last_command();
                    const double cp = g_stop.gaze_pitch_sd > 0.0 ? hc[1] : sweep_p_lo;
                    const int cy_i = std::clamp(int(std::floor((hc[2] - sweep_yc + g_stop.gaze_sweep_yaw) / sweep_wy)), 0, sweep_ny - 1);
                    const int cp_i = sweep_np == 1 ? 0 : std::clamp(int(std::floor((cp - sweep_p_lo) / sweep_wp)), 0, sweep_np - 1);
                    if (ticks_in_stop > 0) ++sweep_count[size_t(cp_i * sweep_ny + cy_i)];
                    const bool arrived = std::fabs(hc[2] - sweep_ty) < 1e-3 && (g_stop.gaze_pitch_sd <= 0.0 || std::fabs(hc[1] - sweep_tp) < 1e-3);
                    if (!sweep_have_target || arrived) {        // the next target: at random among the least-looked-at cells
                        const int least = *std::min_element(sweep_count.begin(), sweep_count.end());
                        std::vector<int> ties;
                        for (int c = 0; c < int(sweep_count.size()); ++c) if (sweep_count[size_t(c)] == least) ties.push_back(c);
                        const int pick = ties[std::uniform_int_distribution<size_t>(0, ties.size() - 1)(gaze_rng)];
                        sweep_ty = sweep_yc - g_stop.gaze_sweep_yaw + (pick % sweep_ny + 0.5) * sweep_wy;
                        sweep_tp = sweep_p_lo + (pick / sweep_ny + 0.5) * sweep_wp;
                        sweep_have_target = true; ++sweep_moves;
                    }
                    head->set_yaw_override(true, sweep_ty);
                    if (g_stop.gaze_pitch_sd > 0.0) head->set_pitch_override(true, sweep_tp);
                    scan_target = hc[2];
                    ++look_held; ++look_ticks; ++ticks_in_stop;
                    if (brain.map_winner() > max_id_seen) max_id_seen = brain.map_winner();
                    // the hold's novelty rule, on a HOLD-long window of the moving view
                    bool view_novel;
                    if (g_stop.gaze_learn_frac > 0.0) {
                        if (look_held == 11) { gaze_qe0 = brain.map_quant_error(); gaze_arrival_novel = gaze_qe0 > g_stop.gaze_residual_k * brain.map_expected_error(); }
                        view_novel = look_held > 10 && gaze_arrival_novel && brain.map_quant_error() > g_stop.gaze_learn_frac * gaze_qe0;
                    } else if (g_stop.gaze_residual_k > 0.0) {
                        view_novel = brain.map_quant_error() > g_stop.gaze_residual_k * brain.map_expected_error();
                    } else {
                        view_novel = brain.map_winner() >= 0 && !baked_ids.count(brain.map_winner());
                    }
                    if (look_held > 10 && view_novel) look_novel_seen = true;
                    const bool slow = look_novel_seen && view_novel && look_held < gaze_max_ticks;   // the dwell, as speed
                    head->set_override_slew(slow ? g_stop.gaze_sweep_slow * g_stop.gaze_sweep : g_stop.gaze_sweep);
                    if (look_held >= gaze_hold_ticks && !slow) {
                        if (look_novel_seen) { ++novel_holds; gaze_quiet_run = 0; } else ++gaze_quiet_run;
                        ++saccades; look_held = 0; look_novel_seen = false;
                        if (!cloud_end_on && gaze_quiet_run >= g_stop.gaze_quiet) stop_left = 0;   // nothing new in a while: the stop ends
                    }
                } else if (looking && gaze_on) {
                    head->set_yaw_override(true, gaze_yaw);
                    if (g_stop.gaze_pitch_sd > 0.0) head->set_pitch_override(true, gaze_pitch);
                    scan_target = gaze_yaw;
                    ++look_held; ++look_ticks; ++ticks_in_stop;
                    if (brain.map_winner() > max_id_seen) max_id_seen = brain.map_winner();
                    // The expectation is the HOLD's: the running mean and spread of the quant error since the
                    // head arrived (~10 ticks) and the token caught up (~10 more), armed after three samples at
                    // the map's rate — 0.7 s into a hold; a ball crossing takes ~1.2 s.  A per-view memory kept
                    // across stops was tried and over-fires (the same node reached from another pose has another
                    // error level); the winner-switch rule was dropped (the map flickers between two nodes on
                    // its own at a still gaze).  The hit must repeat on two consecutive samples.
                    if (look_held == 21) { hold_n = 0; hold_mean = brain.map_quant_error(); hold_var = 0.0; hold_hits = 0; hold_cols = tof.column_hit(); hit_cols.clear(); }
                    if (look_held > 21 && t % 5 == 0) {          // the map's own rate (process_every_n_ticks 5)
                        const double qe = brain.map_quant_error();
                        bool change = false;
                        if (hold_n >= 3) {
                            const double sd = std::max(std::sqrt(std::max(hold_var, 0.0)), 0.05 * hold_mean);
                            const bool jumped = qe > hold_mean + g_stop.orient_k * sd;
                            // the sensor flickers at edges on a still scene (a grazing ray flips a zone, and the nearest
                            // hit per column carries it): a thing that MOVES changes different columns from one sample
                            // to the next, flicker stays in one.  The hit's changed columns are collected; a change
                            // needs two consecutive hits touching at least two distinct columns.
                            if (jumped) {
                                const auto cols = tof.column_hit();
                                for (int c = 0; c < Tof::kCols; ++c) if (std::fabs(cols[size_t(c)] - hold_cols[size_t(c)]) > 0.10) hit_cols.insert(c);
                                ++hold_hits;
                            } else { hold_hits = 0; hit_cols.clear(); }
                            change = orient_on && hold_hits >= 2 && hit_cols.size() >= 2;
                        }
                        if (hold_hits == 0) { hold_mean += 0.2 * (qe - hold_mean); hold_var += 0.2 * ((qe - hold_mean) * (qe - hold_mean) - hold_var); ++hold_n; }
                        if (change) {
                            ++changes; if (t - last_roll_tick <= 150) ++changes_prompted;
                            orient_event = "change";
                            // the reflex: the stop ends, the body goes where the gaze was
                            orient_bearing = odom.yaw() + (body.joint_positions()[7] - kHomePose[7]);
                            orient = Orient::Turn; orient_left = int(g_stop.orient_secs * kBrainHz); ++orientations;
                            stop_left = 0;
                        }
                    }
                    // the ball rolls while the duck is LOOKING: the first tick at or after the delay with the detector
                    // armed (three clean samples into a hold), once per stop; and it is stopped 1.5 s later so a
                    // later re-entry cannot count as an unprompted change
                    // the movers start once the hold's baseline is armed (three clean samples) and just inside the
                    // view, so the first armed sample sees them; a first hit latches the gaze (extend) for the confirmation
                    if (walk_on && !rolled_this_stop && ticks_in_stop >= int(g_stop.walk_delay_s * kBrainHz) && hold_n == 3 && hold_hits == 0) {
                        const auto cols0 = tof.column_hit();
                        const double ahead0 = std::min(cols0[3], cols0[4]);
                        const double dist = std::min(1.2, 0.7 * ahead0);
                        if (dist >= 0.42) {
                            const auto pd = body.trunk_position();
                            const double th = body.trunk_yaw() + (body.joint_positions()[7] - kHomePose[7]);
                            walk_x = pd[0] + dist * std::cos(th) + (0.35 * dist + 0.2) * std::sin(th); walk_y = pd[1] + dist * std::sin(th) - (0.35 * dist + 0.2) * std::cos(th);   // its near edge just inside the view
                            walk_dx = -g_stop.walk_speed * std::sin(th) / kBrainHz; walk_dy = g_stop.walk_speed * std::cos(th) / kBrainHz;
                            walk_left = int(1.2 / g_stop.walk_speed * kBrainHz);
                            ++rolls; last_roll_tick = t; orient_event = "roll"; roll_x = pd[0] + dist * std::cos(th); roll_y = pd[1] + dist * std::sin(th); rolled_this_stop = true;
                        } else { ++rolls_skipped; orient_event = "roll:skipped"; }
                    }
                    if (roll_on && !rolled_this_stop && ticks_in_stop >= int(g_stop.roll_delay_s * kBrainHz) && hold_n == 3 && hold_hits == 0) {
                        // the ball is placed inside the free space the gaze sees: at 1.2 m, or 0.7 of the range the
                        // ToF's centre columns report; a gaze at a surface closer than 0.6 m gets no roll (skipped)
                        const auto cols0 = tof.column_hit();
                        const double ahead0 = std::min(cols0[3], cols0[4]);
                        const double dist = std::min(1.2, 0.7 * ahead0);
                        if (dist >= 0.42) {
                            const auto pd = body.trunk_position();
                            const double th = body.trunk_yaw() + (body.joint_positions()[7] - kHomePose[7]);
                            const double bx = pd[0] + dist * std::cos(th) + 0.35 * dist * std::sin(th), by = pd[1] + dist * std::sin(th) - 0.35 * dist * std::cos(th);   // just inside the view's right edge (tan 22.5° = 0.41)
                            body.roll_body("obj_ball0", bx, by, -g_stop.roll_speed * std::sin(th), g_stop.roll_speed * std::cos(th));
                            ++rolls; last_roll_tick = t; orient_event = "roll"; roll_x = bx; roll_y = by; ball_stopped = false;
                            rolled_this_stop = true;
                        } else { ++rolls_skipped; orient_event = "roll:skipped"; }
                    }
                    bool view_novel;
                    if (g_stop.gaze_learn_frac > 0.0) {
                        // arrival: the view surprised the map (relative to the channel's expectation); then: still learning it
                        if (look_held == 11) { gaze_qe0 = brain.map_quant_error(); gaze_arrival_novel = gaze_qe0 > g_stop.gaze_residual_k * brain.map_expected_error(); }
                        view_novel = look_held > 10 && gaze_arrival_novel && brain.map_quant_error() > g_stop.gaze_learn_frac * gaze_qe0;
                    } else if (g_stop.gaze_residual_k > 0.0) {
                        view_novel = brain.map_quant_error() > g_stop.gaze_residual_k * brain.map_expected_error();
                    } else {
                        view_novel = brain.map_winner() >= 0 && !baked_ids.count(brain.map_winner());
                    }
                    if (look_held > 10 && view_novel) look_novel_seen = true;
                    const bool done_min = look_held >= gaze_hold_ticks;
                    // a first hit of the change detector freezes the gaze on the surprise so the confirming sample
                    // can come (six of fifteen rolls were seen and lost to a gaze step mid-crossing)
                    const bool extend = (look_novel_seen && look_held < gaze_max_ticks && view_novel) || hold_hits >= 1;
                    if (done_min && !extend) {
                        if (look_novel_seen) { ++novel_holds; gaze_quiet_run = 0; } else ++gaze_quiet_run;
                        ++saccades; look_held = 0; look_novel_seen = false;
                        gaze_yaw   = std::clamp(gaze_yaw + g_stop.gaze_yaw_sd * gaze_n(gaze_rng), -0.7, 0.7);
                        if (g_stop.gaze_pitch_sd > 0.0)
                            gaze_pitch = std::clamp(gaze_pitch + g_stop.gaze_pitch_sd * gaze_n(gaze_rng),
                                                    g_stop.gaze_down - g_stop.gaze_pitch_sd * 0.7,
                                                    g_stop.gaze_down + g_stop.gaze_pitch_sd * 2.0);   // + is down (measured: the ToF's floor fraction rises with the joint)
                        if (!cloud_end_on && gaze_quiet_run >= g_stop.gaze_quiet) stop_left = 0;   // nothing new in a while: the stop ends
                    }
                } else if (looking) {
                    static const double kSeq[3] = {0.0, 1.0, -1.0};
                    scan_target = kSeq[look_idx] * g_stop.look_amp;
                    head->set_yaw_override(true, scan_target);
                    ++look_held; ++look_ticks;
                    // the head takes ~10 ticks to arrive; a view is novel while its winner is not a baked node
                    // (§12.2: hold until the view bakes) — the token's adaptive is_novel is a percentile, not this
                    if (look_held > 10 && brain.map_winner() >= 0 && !baked_ids.count(brain.map_winner())) { look_novel_seen = true; look_round_novel = true; }
                    const bool done_min = look_held >= look_hold_ticks;
                    const bool extend = look_novel_seen && look_held < look_max_ticks;
                    if (done_min && !extend) {
                        if (look_novel_seen) ++novel_holds;
                        look_idx = (look_idx + 1) % 3; look_held = 0; look_novel_seen = false; ++saccades;
                        if (look_idx == 0) {                              // a full round
                            if (!look_round_novel) stop_left = 0;         // nothing novel anywhere: the stop ends
                            look_round_novel = false;
                        }
                    }
                }
                if (cloud_end_on && stop_phase != StopPhase::None && stop_left > 0) {
                    // the growth judged is the STOP's cloud: a walking cloud (walk_cloud) still open at the stop's
                    // start is not it (R74: its file-and-reopen read as "stopped growing" and ended every stop at 1.4 s)
                    if (brain.cloud_open() && !brain.cloud_walking()) {
                        cg_vox.push_back(brain.cloud_voxels());
                        const size_t n = cg_vox.size();
                        if (n > size_t(kCloudWin)) {
                            const double growth = double(cg_vox[n - 1] - cg_vox[n - 1 - size_t(kCloudWin)]) / 2.0;   // voxels a second
                            cg_peak = std::max(cg_peak, growth);
                            cg_below = (cg_peak > 0.0 && growth < g_stop.cloud_end_frac * cg_peak) ? cg_below + 1 : 0;
                            if (cg_below >= kCloudWin) { stop_left = 0; ++stops_cloud_ended; }   // the cloud has stopped growing
                        }
                    } else if (!cg_vox.empty()) {
                        cg_vox.clear(); cg_peak = 0.0; cg_below = 0;           // the cloud was filed mid-stop: judge the next afresh
                    }
                }
                if (stop_left <= 0 && stop_phase != StopPhase::None) {
                    scan_stop();
                    if (looking && stop_left <= 0 && stop_event[0] == '\0') { /* ended by the look: named below */ }
                    if (looking && sweep_on) { int seen = 0; for (int v : sweep_count) seen += v > 0; sweep_cover_sum += double(seen) / double(sweep_count.size()); ++sweep_cover_n; }
                    if (looking) { look_stop(); }
                    if (g_stop.map_on_stop) brain.set_map_learning(false);
                    stop_len_sum += (t - stop_started_tick) / kBrainHz; ++stop_len_n;
                    if (stop_phase == StopPhase::Brain) { ++stop_survived; end_stop_drive(true); }
                    const bool bored = (look_on || gaze_on) && (t - stop_started_tick) < stop_ticks - 1 && orient == Orient::None;
                    if (bored) ++stops_bored;
                    stop_phase = StopPhase::None; stop_event = orient != Orient::None ? "stop:orient" : bored ? "stop:bored" : "stop:end";
                    if (stop_is_stuck && g_stuck_escape_s > 0.0) {
                        // the escape: the freest sector of the stop's cloud view (8 sectors across +-64 deg, 1 = nothing
                        // off the floor within 4 m; sector 0 on the right), ties to the one nearest straight ahead
                        const auto view = brain.cloud_view();
                        if (view.size() == 8) {
                            int best = -1; double bv = -1.0;
                            for (int k = 0; k < 8; ++k) { const double v = view[size_t(k)] - 0.01 * std::fabs(k - 3.5); if (v > bv) { bv = v; best = k; } }
                            const double bearing = (-64.0 + (best + 0.5) * 16.0) * M_PI / 180.0;   // + = right, as the loops' bearings
                            brain.set_ref_hold(bearing, int(g_stuck_escape_s * kBrainHz)); ++escapes; stop_event = "stop:escape";
                        }
                        stop_is_stuck = false;
                    }
                    brain.set_learning(true);
                    if (head && ((stander && !g_stop.keep_head) || g_stop.freeze_head)) { head->on_reset(); head->set_learning(true); }
                }
            }
            if (stop_phase != StopPhase::None) command.twist = {0.0, 0.0, 0.0};
            // the unwind: back off, then a stop to look at what the kick did
            if (unwind_left > 0 && stop_phase == StopPhase::None) {
                command.twist = {-std::fabs(g_skill_unwind_vx), 0.0, 0.0};
                // --skill-unwind-aim: back off with the nose kept on the kicked thing (its bearing + = right; the
                // twist's yaw + = left), so the look stop that follows has it ahead instead of off the sweep
                if (g_skill_unwind_aim > 0.0 && brain.thing_pos_present())
                    command.twist[2] = std::clamp(-g_skill_unwind_aim * brain.thing_ego(), -1.0, 1.0);
                brain.set_learning(false);
                if (--unwind_left == 0 && unwind_then_stop && (ticks - t) > stop_ticks) {
                    unwind_then_stop = false;
                    stop_phase = StopPhase::Settle; stop_left = stop_ticks; stop_settle_left = stop_settle_ticks;
                    ++stops_started; ++stops_arrive; stop_event = "stop:look"; stop_started_tick = t; stop_is_arrive = true; stop_is_look = true;
                    if (stander) stander->on_reset();
                    if (head && ((stander && !g_stop.keep_head) || g_stop.freeze_head)) head->set_learning(false);
                }
            }
            if (orient != Orient::None && stop_phase == StopPhase::None) {
                brain.set_learning(false);                     // its command is not applied through the approach
                const double e = wrap_pi(orient_bearing - odom.yaw());
                const auto cols = tof.column_hit();
                const double ahead = std::min(cols[3], cols[4]);   // the two centre columns, metres (max range if empty)
                --orient_left;
                const auto op = odom.position();
                if (orient == Orient::Turn) {
                    command.twist = {g_stop.orient_turn_vx, 0.0, e > 0.0 ? 1.0 : -1.0};   // the pivot
                    if (std::fabs(e) < 0.15) { orient = Orient::Walk; orient_event = "orient:turned"; orient_x0 = op[0]; orient_y0 = op[1]; }
                } else {
                    command.twist = {g_stop.orient_vx, 0.0, std::clamp(1.5 * e, -0.6, 0.6)};
                }
                const double walked = orient == Orient::Walk ? std::hypot(op[0] - orient_x0, op[1] - orient_y0) : 0.0;
                bool done = false;
                // arrived: it has walked to where the change was (~1 m), or reached a surface after at least 0.4 m
                if (orient == Orient::Walk && (walked >= 0.9 || (ahead > 0.0 && ahead < 0.4 && walked >= 0.4))) { ++arrivals; orient_event = "orient:arrived"; done = true; }
                else if (orient_left <= 0) { ++orient_timeouts; orient_event = "orient:timeout"; done = true; }
                if (done) {
                    if (roll_on && t - last_roll_tick < 30 * 50) { const auto pd = body.trunk_position(); reach_sum += std::hypot(roll_x - pd[0], roll_y - pd[1]); ++reach_n; }   // distance to where the ball WAS rolled
                    orient = Orient::None; command.twist = {0.0, 0.0, 0.0};
                    brain.on_reset(); brain.set_learning(true);
                    if (orient_event == std::string("orient:arrived") && stop_period > 0 && (ticks - t) > stop_ticks) {
                        // arrived: a stop, to look at it
                        stop_phase = StopPhase::Settle; stop_left = stop_ticks; stop_settle_left = stop_settle_ticks;
                        ++stops_started; stop_started_tick = t; brain.set_learning(false);
                        if (stander) stander->on_reset();
                        if (head && ((stander && !g_stop.keep_head) || g_stop.freeze_head)) head->set_learning(false);
                        command.twist = {0.0, 0.0, 0.0};
                    }
                }
            }
        }
        if (head) {
            // The head loop: the four head joints (policy indices 5-8) relative to HOME are its
            // sensed "joints"; the head IMU and the trunk IMU its senses; its output the head
            // block of the walker's command vector.  The walker keeps every other joint.
            const auto q = body.joint_positions();
            std::array<double, 4> head_q{};
            for (int i = 0; i < 4; ++i) head_q[size_t(i)] = q[size_t(5 + i)] - kHomePose[size_t(5 + i)];
            const auto hcmd = head->tick(head_q, body.head_gravity(), body.head_gyro(), g, w,
                                         q[2] - kHomePose[2]);            // the left hip pitch: the stride clock
            if (driver == Driver::Brain) command.head = hcmd;
        }
        std::array<double, 4> head_targets{};
        bool head_owns_joints = false;
        if (head && g_head_joints && driver == Driver::Brain) {
            const auto hcmd = head->last_command();
            for (int i = 0; i < 4; ++i) head_targets[size_t(i)] = kHomePose[size_t(5 + i)] + hcmd[size_t(i)];
            head_owns_joints = true;
            command.head = {0.0, 0.0, 0.0, 0.0};          // the walker is told nothing about the head
        }

        if (pushes.newtons > 0.0 && push_period > 0 && t > 0 && t >= push_from && t % push_period == 0
            && driver == Driver::Brain && (ticks - t) >= int(kRecoverWindowSecs * kBrainHz)) {
            static const double dirs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
            const auto& d = dirs[push_index++ % 4];
            body.push({pushes.newtons * d[0], pushes.newtons * d[1], 0.0}, push_hold);
            ++pushes_delivered;
        }

        if (down_rise_watch > 0) --down_rise_watch;
        std::array<double, kNumPolicyJoints> ctrl{};
        if (driver == Driver::Scaffold && skill_active && down_skill_running) {
            // the roll drives the joints while the body is down; the scaffold waits for the window to end
            Command sc{};
            if (skill_phase_period > 0.0) { const double phase = double(skill_total - skill_left) / kBrainHz / skill_phase_period; sc.twist = {std::cos(2.0 * M_PI * phase), std::sin(2.0 * M_PI * phase), 0.0}; }
            const auto action = skill_policy->infer(build_observation(body, skill_last, sc));
            skill_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i) ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
            scaffold_last.fill(0.0f);
            if (skill_left > 0) --skill_left;
        } else if (driver == Driver::Scaffold) {
            const auto action = scaffold.infer(build_observation(body, scaffold_last, Command{}));
            scaffold_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i)
                ctrl[i] = kHomePose[i] + kStandingActionScale * action[i];
        } else if (skill_active && skill_push_vx > 0.0) {
            // the PUSH's window: the walker, a forward command, exactly the walk's own drive (scale, low-pass)
            Command pc{}; pc.twist = {skill_push_vx, 0.0, 0.0};
            if ((g_push_vx > 0.0 || skill_name == "approach") && g_skill_unwind_aim > 0.0 && brain.thing_pos_present())
                pc.twist[2] = std::clamp(-g_skill_unwind_aim * brain.thing_ego(), -1.0, 1.0);   // the nose on the thing
            if (skill_name == "approach" && brain.thing_pos_present()) {
                // the closing step: FACE the thing first (turn in place while it is more than 0.2 rad off the nose),
                // then walk onto it; it ends by the odometry -- centre within reach and nose on it -- or at the ceiling
                const double ego = brain.thing_ego();
                if (std::fabs(ego) > 0.2) pc.twist = {0.0, 0.0, std::clamp(-1.5 * ego, -1.0, 1.0)};
                if (brain.thing_range() <= g_approach_reach && std::fabs(ego) < 0.3 && skill_left > 1) skill_left = 1;
            }
            const auto action = walker.infer(build_observation(body, walker_last, pc));
            walker_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i) {
                const double target = kHomePose[i] + kWalkingActionScale * action[i];
                const double alpha = (i >= 5 && i <= 8) ? kWalkLowpassHead : kWalkLowpassLegs;
                walk_targets[i] += alpha * (target - walk_targets[i]);
                ctrl[i] = walk_targets[i];
            }
            if (head_owns_joints)
                for (int i = 0; i < 4; ++i) { ctrl[5 + i] = head_targets[size_t(i)]; walk_targets[5 + i] = head_targets[size_t(i)]; }
            if (skill_left > 0) --skill_left;
            if (skill_left == 0 && stop_phase == StopPhase::Brain && stander) stander->on_reset();
        } else if (skill_active) {
            // a skill's window, from standing or from the walk: the network drives every joint at standing tuning;
            // a phase-driven skill (the peck) sees its phase in the twist slots, as the daemon feeds it
            Command sc{};
            if (skill_phase_period > 0.0) {
                const double phase = double(skill_total - skill_left) / kBrainHz / skill_phase_period;
                sc.twist = {std::cos(2.0 * M_PI * phase), std::sin(2.0 * M_PI * phase), 0.0};
            }
            const auto action = skill_policy->infer(build_observation(body, skill_last, sc));
            skill_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i) { ctrl[i] = kHomePose[i] + kStandingActionScale * action[i]; walk_targets[i] = ctrl[i]; }
            if (skill_left > 0) --skill_left;
            if (skill_left == 0 && stop_phase == StopPhase::Brain && stander) stander->on_reset();   // the stander resumes from where the kick left the body
        } else if (stop_phase == StopPhase::Brain) {
            // The joint brain stands (W1): the legs are its; the head too unless --stop-keep-head.
            // The learnable-regime gate as --brain applies it (near-upright only).
            stander->set_regime_learning(g[2] < -0.90);
            stander->sample_tle(g[2] < -0.5);
            ctrl = stander->act(body);
            if (head_owns_joints && g_stop.keep_head)
                for (int i = 0; i < 4; ++i) ctrl[5 + i] = head_targets[size_t(i)];
            walk_targets = ctrl;                 // the walker resumes from where the body is left
            ++stand_ticks;
        } else {
            const auto action = walker.infer(build_observation(body, walker_last, command));
            walker_last = action;
            for (int i = 0; i < kNumPolicyJoints; ++i) {
                const double target = kHomePose[i] + kWalkingActionScale * action[i];
                const double alpha = (i >= 5 && i <= 8) ? kWalkLowpassHead : kWalkLowpassLegs;
                walk_targets[i] += alpha * (target - walk_targets[i]);
                ctrl[i] = walk_targets[i];
            }
            if (head_owns_joints)
                for (int i = 0; i < 4; ++i) { ctrl[5 + i] = head_targets[size_t(i)]; walk_targets[5 + i] = head_targets[size_t(i)]; }
        }
        body.step(ctrl);

        {
            std::array<SitePose, 2> feet;
            body.site_pose_trunk("left_foot", feet[0].pos, feet[0].quat);
            body.site_pose_trunk("right_foot", feet[1].pos, feet[1].quat);
            odom.update(feet, body.imu_quat());
            const auto p = odom.position();
            const double yaw = odom.yaw();
            if (t % 4 == 0) {
                tof.sense(body, p[2]);
                tof_summary = tof.summary();
                ++tof_ticks;
            }
            {
                place.pose = {float(p[0] / 2.0), float(p[1] / 2.0), float(std::cos(yaw)), float(std::sin(yaw))};
                place.head_yaw = float((body.joint_positions()[7] - kHomePose[7]) / 1.4);   // the head-yaw joint / its range
                // one cast for the CloudMap module: stillness, the heading it de-rotates against,
                // the trunk height, then every return already levelled with z above the floor
                place.tof_points_valid = cloud_on;
                if (cloud_on) {
                    const bool still = g[2] < -0.999 && std::max({std::fabs(w[0]), std::fabs(w[1]), std::fabs(w[2])}) < 0.15;
                    place.tof_points[0] = still ? 1.0f : 0.0f;
                    place.tof_points[1] = float(yaw);
                    place.tof_points[2] = float(p[2]);
                    place.tof_points[3] = float(p[0]);      // the dead-reckoned position: what lines
                    place.tof_points[4] = float(p[1]);      // two visits to one place up
                    const auto& zz = tof.zones();
                    for (int i = 0; i < Tof::kZones; ++i) {
                        const bool ok = zz[size_t(i)].cls == TofZone::Hit || zz[size_t(i)].cls == TofZone::Floor;
                        place.tof_points[size_t(5 + 3 * i + 0)] = ok ? float(zz[size_t(i)].point_level[0]) : std::numeric_limits<float>::quiet_NaN();
                        place.tof_points[size_t(5 + 3 * i + 1)] = ok ? float(zz[size_t(i)].point_level[1]) : std::numeric_limits<float>::quiet_NaN();
                        place.tof_points[size_t(5 + 3 * i + 2)] = ok ? float(zz[size_t(i)].point_level[2] + p[2]) : std::numeric_limits<float>::quiet_NaN();
                    }
                    const auto& og = tof.origin_level();   // the ray origin, same frame, z above the floor
                    place.tof_points[size_t(5 + 3 * Tof::kZones + 0)] = float(og[0]);
                    place.tof_points[size_t(5 + 3 * Tof::kZones + 1)] = float(og[1]);
                    place.tof_points[size_t(5 + 3 * Tof::kZones + 2)] = float(og[2] + p[2]);
                }
                const auto col = tof.column_hit();
                for (int i = 0; i < Tof::kCols; ++i) place.cols[size_t(i)] = float(col[size_t(i)] / Tof::kMaxRangeM);
                if (g_map_view_cloud) {                        // --map-view cloud: the stop's cloud is the view
                    if (brain.cloud_open() && !brain.cloud_walking()) {   // a walking cloud (walk_cloud) is the seek loop's, not the map's
                        const auto cv = brain.cloud_view();
                        if (cv.size() == map_view_held.size()) std::copy(cv.begin(), cv.end(), map_view_held.begin());
                    }
                    place.head_yaw = 0.0f;
                    place.cols = map_view_held;
                }
                const auto& z = tof.zones();
                for (int i = 0; i < Tof::kZones; ++i) {
                    const double r = z[size_t(i)].range < 0.0 ? Tof::kMaxRangeM : z[size_t(i)].range;
                    place.zones[size_t(i)] = float(std::clamp(r / Tof::kMaxRangeM, 0.0, 1.0));
                }
            }
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
                        driver != Driver::Brain ? "scaffold" : stop_phase == StopPhase::Brain ? "stand" : "walk",
                        command.twist[0], command.twist[1], command.twist[2],
                        sensed[0], sensed[1], sensed[2],
                        learning_now ? "true" : "false",
                        recovery.handed_off_this_tick()    ? "reset:handoff"
                        : recovery.handed_back_this_tick() ? "reset:handback"
                        : stop_event[0]                    ? stop_event
                        : orient_event[0]                  ? orient_event : "",
                        op[0], op[1], odom.yaw());
            const auto q = body.joint_positions();
            for (int i = 0; i < kNumPolicyJoints; ++i) std::printf("%s%.4f", i ? "," : "", q[i]);
            std::printf("],\"qpos\":[");
            const auto full = body.qpos();
            for (size_t i = 0; i < full.size(); ++i) std::printf("%s%.6f", i ? "," : "", full[i]);
            // The ToF: 64 zone classes as one string (0 Empty, 1 TooClose, 2 Floor, 3 Hit),
            // the 64 slant ranges, and the four-slot summary the brain sees.
            std::printf("],\"tofz\":\"");
            for (const auto& z : tof.zones()) std::printf("%d", int(z.cls));
            std::printf("\",\"tofr\":[");
            for (int i = 0; i < Tof::kZones; ++i) std::printf("%s%.2f", i ? "," : "", tof.zones()[i].range);
            std::printf("],\"tofs\":[%.2f,%.2f,%.2f,%.2f],\"wall\":%d,\"steer\":%d,\"map\":[%.3f,%d,%d,%d],\"hdg\":[%.3f,%.3f]", tof_summary[0], tof_summary[1], tof_summary[2], tof_summary[3],
                        body.touching_wall() ? 1 : 0, brain.last_steer(), brain.map_tle(), brain.map_novel() ? 1 : 0, brain.map_winner(),
                        (t % 25 == 0) ? brain.map_nodes() : -1, brain.heading(), brain.heading_ref());
            if (has_objects) std::printf(",\"obj\":%d", body.touching_object() ? 1 : 0);
            // the seek loop, if the graph has one (things phase T2): its need and the range left to its target
            if (brain.seek_present()) std::printf(",\"seek\":[%.3f,%.3f,%d]", brain.seek_value(), brain.seek_range(), brain.seek_gated());
            if (g_log_com) { const auto cm = body.com_over_feet(); std::printf(",\"com\":[%.4f,%.4f,%.4f]", cm[0], cm[1], cm[2]); }
            if (train_on) std::printf(",\"train\":[%.4f,%.4f,%.4f,%.3f,%.3f,%d]", train_x, train_y, train_yaw, train_vx, train_vy, train_moving ? 1 : 0);
            // THE CHASE (stage 1): [chasing, sightings held, target x, y (odometry frame), its velocity, a mover seen this tick]
            if (chase_on) std::printf(",\"chase\":[%d,%d,%.3f,%.3f,%.3f,%.3f,%d]", brain.chase_active() ? 1 : 0, brain.chase_n(),
                                      brain.seek_target_x(), brain.seek_target_y(), brain.chase_vx(), brain.chase_vy(), brain.mover_seen() ? 1 : 0);
            // MOVERS (stage 0): the clusters through the recency window, on cast ticks with a cloud open
            if (g_log_movers_s > 0.0 && cloud_on && t % 4 == 0 && brain.cloud_open()) {
                const auto th = brain.cloud_things_recent(uint64_t(std::lround(g_log_movers_s * kBrainHz)));
                std::printf(",\"mvw\":%d,\"mva\":[%.4f,%.4f,%.4f],\"mvc\":[", brain.cloud_walking() ? 1 : 0,
                            cloud_anchor_wx, cloud_anchor_wy, cloud_anchor_wyaw);
                for (size_t k = 0; k < th.size(); ++k)
                    std::printf("%s[%.3f,%.3f,%.3f,%.3f,%d,%.0f,%d,%.2f,%.2f,%.2f,%d]", k ? "," : "", th[k].cx, th[k].cy, th[k].ext, th[k].top,
                                th[k].ncols, th[k].hits, th[k].small ? 1 : 0, th[k].fresh, th[k].age / kBrainHz, th[k].age_w / kBrainHz, th[k].vacated);
                std::printf("]");
            }
            if (g_hr_tau > 0.0) std::printf(",\"hr\":%.2f", brain.heading_reflex_share());
            if (g_ref_free > 0.0 && t % 50 == 0) std::printf(",\"rfree\":%d", brain.ref_released());
            if (g_stop.on_stuck > 0.0) std::printf(",\"stall\":[%.2f,%.2f]", brain.stall_s(), brain.stall_median_s());
            if (skill_active) std::printf(",\"skill\":\"%s\"", skill_name.c_str());
            // the play loop, for the record: its bearing every tick, its state every 10 (2026-09-19, the orbit)
            { const auto pb = brain.play_bearing(); std::printf(",\"pb\":[%.3f,%.3f]", pb[0], pb[1]);
              if (t % 10 == 0) { const auto ps = brain.play_state();
                  if (!ps.is_null()) std::printf(",\"pl\":[%d,%d,%d,%.2f]", ps.value("climbing", false) ? 1 : 0, ps.value("wandering", false) ? 1 : 0, ps.value("next_node", -1), ps.value("explore_dir", 0.0)); } }
            { const auto oc = brain.outcome_now();   // the outcome loop: [node, predicted, observed, surprise, samples]
              if (!oc.empty()) std::printf(",\"outc\":[%d,%.3f,%.3f,%.2f,%d,%d]", int(oc[0]), oc[1], oc[2], oc[3], int(oc[4]), oc.size() > 5 ? int(oc[5]) : 0); }
            // live changes a client made through the inspector this tick: the run's record must show them
            {
                const auto pe = brain.take_inspector_events();
                if (!pe.empty()) {
                    std::printf(",\"patch\":[");
                    for (size_t k = 0; k < pe.size(); ++k) std::printf("%s\"%s\"", k ? "," : "", pe[k].c_str());
                    std::printf("]");
                }
            }
            if (g_log_motor_tle) {
                std::printf(",\"mtle\":%.5f", brain.motor_tle());
                if (stander) std::printf(",\"btle\":%.5f", stander->motor_tle());
            }
            if (cloud_on && brain.cloud_open())
                std::printf(",\"cld\":[%d,%d,%.4f,%.4f,%d]", brain.cloud_voxels(), brain.cloud_break(),
                            brain.cloud_newfrac(), brain.cloud_revisit(), brain.cloud_cached());
            // --log-cloud-profile: the MODULE's 36-dim break profile on every cast while a cloud is open —
            // what an object EPM subscribed to reality.proprio.cloud receives, for the offline bench.
            if (g_log_cloud_profile && cloud_on && brain.cloud_open() && t % 4 == 0) {
                const auto prof = brain.cloud_profile();
                std::printf(",\"cldp\":[");
                for (size_t k = 0; k < prof.size(); ++k) std::printf("%s%.4f", k ? "," : "", prof[k]);
                std::printf("]");
            }
            // THINGS (the things phase, T1).  While a cloud is open and the graph asks for things, every
            // compute tick logs the attended thing -- in the CLOUD's frame (the anchor's), so the scorer
            // labels it with the anchor the next "cloudv" record carries -- and the thing EPM's token:
            //   "thg": [things, small, attended, cx, cy, rng, ext, top, ncols, hits, chain, vx, vy, prox]
            //   "tepm": [winner, tle, nodes]  (only on ticks the EPM published)
            if (cloud_on && things_on && brain.cloud_open() && t % 4 == 0) {
                const auto th = brain.cloud_things();
                const int at = brain.cloud_attended();
                const int small = int(std::count_if(th.begin(), th.end(), [](const ogma::CloudMap::Thing& x) { return x.small; }));
                const auto b = brain.cloud_thing_bearing();
                if (at >= 0 && at < int(th.size())) {
                    const auto& x = th[size_t(at)];
                    std::printf(",\"thg\":[%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.0f,%d,%.3f,%.3f,%.3f]",
                                int(th.size()), small, at, x.cx, x.cy, x.rng, x.ext, x.top, x.ncols, x.hits, x.chain, b[0], b[1], b[2]);
                } else {
                    std::printf(",\"thg\":[%d,%d,-1]", int(th.size()), small);
                }
                if (brain.thing_seen()) std::printf(",\"tepm\":[%d,%.4f,%d]", brain.thing_winner(), brain.thing_tle(), brain.thing_nodes());
                if (brain.kind_seen()) std::printf(",\"tkind\":[%d,%.4f,%d]", brain.kind_winner(), brain.kind_tle(), brain.kind_nodes());
            }
            // THE REPLAY PAYLOAD.  On the tick a cloud is filed, its whole voxel set goes to the log
            // once, with the world pose it was anchored on so a viewer can place it beside the
            // furniture.  That pose is INSTRUMENTATION for the viewer; no brain reads the log.
            if (cloud_on && brain.cloud_just_closed()) {
                const auto vx = brain.cloud_filed_voxels();
                std::printf(",\"cloudv\":{\"place\":%d,\"voxel_m\":%.4f,\"revisit\":%.4f,\"revisit_dist\":%.4f,"
                            "\"anchor\":[%.4f,%.4f,%.4f],\"vox\":[",
                            brain.cloud_place(), brain.cloud_voxel_m(), brain.cloud_revisit(), brain.cloud_revisit_dist(),
                            cloud_filed_wx, cloud_filed_wy, cloud_filed_wyaw);
                for (size_t k = 0; k + 4 < vx.size(); k += 5)        // [ix, iy, iz, hits, mean height mm]
                    std::printf("%s[%d,%d,%d,%d,%d]", k ? "," : "", vx[k], vx[k + 1], vx[k + 2], vx[k + 3], vx[k + 4]);
                std::printf("]}");
                // the module's clusters of the filed cloud, for the faithfulness check against the offline rule:
                //   "things": [[cx, cy, rng, ext, top, ncols, hits, chain, small], ...]
                if (things_on) {
                    const auto th = brain.cloud_filed_things();
                    std::printf(",\"things\":[");
                    for (size_t k = 0; k < th.size(); ++k)
                        std::printf("%s[%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.0f,%d,%d]", k ? "," : "", th[k].cx, th[k].cy, th[k].rng,
                                    th[k].ext, th[k].top, th[k].ncols, th[k].hits, th[k].chain, th[k].small ? 1 : 0);
                    std::printf("]");
                }
                ++cloud_n; cloud_vox_sum += int(vx.size() / 5);
            }
            if (g_log_tof_cloud && t % 4 == 0) {
                std::printf(",\"tofp\":[");
                const auto& zz = tof.zones();
                bool first = true;
                for (int i = 0; i < Tof::kZones; ++i) {
                    if (zz[size_t(i)].cls != TofZone::Hit && zz[size_t(i)].cls != TofZone::Floor) continue;
                    std::printf("%s[%d,%.4f,%.4f,%.4f]", first ? "" : ",", i,
                                zz[size_t(i)].point_level[0], zz[size_t(i)].point_level[1], zz[size_t(i)].point_level[2]);
                    first = false;
                }
                std::printf("]");
            }
            if (stop_on) {
                // the stop phase (0 none, 1 settling under the walker, 2 the joint brain stands, 3 the walker
                // holds it: gate refused or handed back) and the joint brain's own attitude error
                double att = 0.0;
                if (stander) for (double v : stander->attitude_error()) att = std::max(att, v);
                std::printf(",\"stop\":%d,\"satt\":%.3f", int(stop_phase), att);
                if (scan_on || look_on) std::printf(",\"scan\":%.2f", (scanning || looking) ? scan_target : 0.0);
                if (orient_on) std::printf(",\"orient\":%d,\"mqe\":[%.4f,%.4f,%.4f,%d]", int(orient), brain.map_quant_error(), brain.map_expected_error(), brain.map_transition(), look_held);
            }
            if (head) {
                const auto hg = body.head_gravity(); const auto hw = body.head_gyro(); const auto hc = head->last_command();
                std::printf(",\"head\":[%.4f,%.4f,%.4f,%.4f],\"hg\":[%.4f,%.4f,%.4f],\"hw\":[%.4f,%.4f,%.4f]",
                            hc[0], hc[1], hc[2], hc[3], hg[0], hg[1], hg[2], hw[0], hw[1], hw[2]);
            }
            std::printf("}\n");
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
    if (head) {
        for (const auto& line : head->readback()) std::fprintf(stderr, "  %s\n", line.c_str());
        for (const auto& line : head->phase_report()) std::fprintf(stderr, "  %s\n", line.c_str());
        for (const auto& line : head->diagnostics()) std::fprintf(stderr, "  %s\n", line.c_str());
        if (!g_save_head.empty()) {
            nlohmann::json snap;
            snap["graph"] = head->brain_state();
            snap["head_graph"] = g_head_graph;
            std::ofstream out(g_save_head);
            out << snap;
            std::fprintf(stderr, "head brain snapshot -> %s\n", g_save_head.c_str());
        }
    }
    if (g_no_backing)
        std::fprintf(stderr, "  no-backing: %d ticks of backward command clamped to zero\n", brain.backing_clamped());
    if (g_wander_bored_s > 0.0)
        std::fprintf(stderr, "  wander: %d heading changes of %.0f deg after %.0f s of familiarity\n",
                     brain.wander_turns(), g_wander_turn_deg, g_wander_bored_s);
    if (pushes.newtons > 0.0)
        std::fprintf(stderr, "  %.1f N shoves every %.1f s from %.0f s: %d delivered\n", pushes.newtons,
                     pushes.every_s, pushes.from_s, pushes_delivered);
    if (stop_on) {
        std::fprintf(stderr, "  stops: %d started, %d hand-backs, %d refused by the gate, %d survived to the end, "
                             "%d handed back to the walker, %d rescued; the joint brain stood %.1f s\n",
                     stops_started, stop_handbacks, stop_refused, stop_survived, stop_handoffs, stop_rescued,
                     stand_ticks / kBrainHz);
        if (g_stop.on_arrive) std::fprintf(stderr, "  arrival stops: %d of %d started when the seek loop reached its target\n", stops_arrive, stops_started);
        if (chase_on) std::fprintf(stderr, "  chases: %d started, %d mover candidates seen by the cloud\n", brain.chases(), brain.mover_cands());
        if (g_stop.on_stuck > 0.0) std::fprintf(stderr, "  stuck stops: %d of %d started when a forward stall exceeded %.1f x the body's own median stall; %d escapes\n", stops_stuck, stops_started, g_stop.on_stuck, escapes);
        if (!g_skill_on_arrive.empty() || g_skill_at_s > 0.0 || skills_requested > 0) std::fprintf(stderr, "  skills: %d fired (%d requested by the graph), %d unwinds\n", skills_fired, skills_requested, unwinds);
        if (g_skill_unwind_aim > 0.0) std::fprintf(stderr, "  unwind aim: %d look stops had the sweep centred on the kicked thing's bearing (gain %.2f on the unwind's yaw)\n", look_aimed, g_skill_unwind_aim);
        if (g_approach_reach > 0.0) std::fprintf(stderr, "  approach: %d steps onto a thing before a kick or a peck\n", approaches);
        if (!g_spin_skill.empty()) std::fprintf(stderr, "  spins: %d detected (> %.1f turns in %.0f s with under %.2f m of travel); %d %s fired\n", spins, g_spin_turns, g_spin_secs, g_spin_net, spin_rolls, g_spin_skill.c_str());
        if (!g_skill_when_down.empty()) std::fprintf(stderr, "  skill when down: %s fired %d times, the body upright within 2 s of the window %d times\n", g_skill_when_down.c_str(), down_skills, down_rises);
        if (g_stop.map_on_stop || look_on || gaze_on) {
            std::fprintf(stderr, "  map growth: %d nodes on walks, %d at stops; %zu baked ids seen\n", grown_walk, grown_stop, baked_ids.size());
            std::fprintf(stderr, "  map baking: inserted %d at stops / %d on walks, baked %d at stops / %d on walks, pruned %d (of which baked %d); %d nodes, %d baked at the end\n",
                         ins_stop, ins_walk, bake_stop, bake_walk, pruned_total, pruned_baked, brain.map_node_count(), brain.map_baked_count());
        }
        if (cloud_n > 0)
            std::fprintf(stderr, "  cloud: %d clouds filed, mean %ld voxels; %d cached by place\n",
                         cloud_n, cloud_vox_sum / cloud_n, brain.cloud_cached());
        if (orient_on || roll_on || walk_on)
            std::fprintf(stderr, "  orient: %d rolls (%d skipped at a surface), %d changes (%d within 3 s of a roll, %d unprompted), %d orientations, %d arrivals, %d timeouts, mean reach %.2f m to the roll\n",
                         rolls, rolls_skipped, changes, changes_prompted, changes - changes_prompted, orientations, arrivals, orient_timeouts, reach_n ? reach_sum / reach_n : 0.0);
        if (cloud_end_on)
            std::fprintf(stderr, "  cloud end: %d of %d stops ended when the cloud stopped growing\n", stops_cloud_ended, stops_started);
        if (sweep_on)
            std::fprintf(stderr, "  sweep: %d moves; the gaze grid covered %.0f %% of its cells per stop (%d stops); 'saccades' below count HOLD-long windows\n",
                         sweep_moves, 100.0 * sweep_cover_sum / std::max(1, sweep_cover_n), sweep_cover_n);
        if (look_on || gaze_on)
            std::fprintf(stderr, "  look: %d saccades, %d holds extended by novelty, %d of %d stops ended by a quiet round; mean stop %.1f s; the head looked for %.1f s\n",
                         saccades, novel_holds, stops_bored, stop_len_n, stop_len_n ? stop_len_sum / stop_len_n : 0.0, look_ticks / kBrainHz);
        if (stander) for (const auto& line : stander->diagnostics()) std::fprintf(stderr, "  stander: %s\n", line.c_str());
    }
    const double total = recovery.brain_seconds() + recovery.scaffold_seconds();
    std::fprintf(stderr, "level-2 %.0f s — %d rescues, %.0f%% of the run walker-driven; learning frozen %.0f%%\n",
                 seconds, recovery.rescues(), 100.0 * recovery.brain_seconds() / std::max(total, 1e-9),
                 100.0 * frozen_ticks / ticks);
    if (recovery.gave_up() > 0)                        // a rescue the stand policy could not finish in 8 s: wedged, most likely
        std::fprintf(stderr, "  rescues given up: %d of %d (longest %.1f s) — the body stayed down through them\n",
                     recovery.gave_up(), recovery.rescues(), recovery.longest_recovery());
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
        "      --head-graph H.json adds the head loop: a second brain on the walker's four head commands,\n"
        "      sensing the head IMU (the playroom overlay's); its identified rows are printed at the end.\n"
        "      --save-head F / --load-head F: the head brain's state alone (identify standing, act walking).\n"
        "      --no-backing: the forward command clamped at zero — no rear sensor, no step into the unseen.\n"
        "      --head-vor TAU LEAD: the yaw reflex in the head loop — minus the trunk's integrated yaw rate,\n"
        "      leaking to centre in TAU s, plus LEAD s of the rate itself (0 0 = off).\n"
        "      --head-phase LEAD_TICKS LEARN_S: the gait-phase feed-forward on yaw — the stride clock from the hip\n"
        "      pitch, a 16-bin table of the head's yaw rate learned for LEARN_S s, then the opposite angle LEAD early.\n"
        "      --stop-every S --stop-secs S [--stop-from S] --stop-brain CFG [--stop-load CKPT] [--stop-att X]\n"
        "          [--stop-handoff-att Y | --stop-handoff-lean DEG] [--stop-settle-secs S] [--stop-keep-head]\n"
        "          [--stop-scan AMP HOLD_S | --stop-look AMP HOLD_S MAX_S | --stop-gaze YAW_SD PITCH_SD HOLD_S MAX_S QUIET]\n"
        "          [--stop-gaze-sweep SPEED YAW_MAX]  (with --stop-gaze: never hold; move toward the least-looked-at gaze)\n"
        "          [--stop-gaze-sweep-slow F] [--stop-cloud-end F] [--map-view cloud|frame]\n"
        "          [--map-on-stop]:\n"
        "          scheduled stops (W1): the twist zeroed, and once still the legs handed to the joint brain\n"
        "          if its attitude error is below X; the walker takes them back on Y / DEG and at the end.\n"
        "      --head-joints: Track A at the head — the head brain's commands become the head JOINT targets\n"
        "      (HOME + command) over the walker's head outputs; the walker keeps the legs.\n"
        "      --head-rate K TAU: the yaw command integrates minus K times the head's OWN yaw rate (its gyro),\n"
        "      leaking to centre in TAU s — counters whatever moves the head (0 0 = off).\n"
        "      --fast-until S (with --realtime): unpaced until S s, then real time — watch the tour, skip the babble.\n"
        "      --arena-shift S moves wall_px at S s; --move NAME X Y S relocates a playroom body or\n"
        "      geom at S s (repeatable) — the (d) tests.  A generated scene's manifest is echoed.\n"
        "      --train SPEED RUN_S STOP_S drives the scene's toy train (playroom_gen.py --train) round its track\n"
        "      at SPEED m/s for RUN_S s, still for STOP_S s, repeating; the phase and the start along the track\n"
        "      follow the seed unless --train-phase S.  --log-movers WINDOW_S logs the cloud's clusters through a\n"
        "      recency window on every cast (\"mvc\"), the stage-0 instrument for chasing moving things.\n"
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
        } else if (a == "--fast-until") {
            g_fast_until_ticks = int(std::stod(next("--fast-until")) * kBrainHz);
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
        } else if (a == "--wander-bored") {
            g_wander_bored_s = std::stod(next("--wander-bored"));
        } else if (a == "--wander-turn") {
            g_wander_turn_deg = std::stod(next("--wander-turn"));
        } else if (a == "--arena-shift") {
            g_arena_shift_s = std::stod(next("--arena-shift"));
        } else if (a == "--head-graph") {
            g_head_graph = next("--head-graph");
        } else if (a == "--head-phase") {
            g_head_phase_lead = std::stod(next("--head-phase")); g_head_phase_learn = std::stod(next("--head-phase"));
        } else if (a == "--stop-every") {
            g_stop.every_s = std::stod(next("--stop-every"));
        } else if (a == "--stop-secs") {
            g_stop.secs = std::stod(next("--stop-secs"));
        } else if (a == "--stop-from") {
            g_stop.from_s = std::stod(next("--stop-from"));
        } else if (a == "--stop-brain") {
            g_stop_brain = next("--stop-brain");
        } else if (a == "--stop-load") {
            g_stop_load = next("--stop-load");
        } else if (a == "--stop-att") {
            g_stop.att_gate = std::stod(next("--stop-att"));
        } else if (a == "--stop-handoff-att") {
            g_stop.handoff_att = std::stod(next("--stop-handoff-att"));
        } else if (a == "--stop-handoff-lean") {
            g_stop.handoff_lean = std::stod(next("--stop-handoff-lean"));
        } else if (a == "--stop-settle-secs") {
            g_stop.settle_s = std::stod(next("--stop-settle-secs"));
        } else if (a == "--stop-keep-head") {
            g_stop.keep_head = true;
        } else if (a == "--stop-freeze-head") {
            g_stop.freeze_head = true;
        } else if (a == "--stop-orient") {
            g_stop.orient_k = std::stod(next("--stop-orient")); g_stop.orient_turn_vx = std::stod(next("--stop-orient"));
            g_stop.orient_vx = std::stod(next("--stop-orient")); g_stop.orient_secs = std::stod(next("--stop-orient"));
        } else if (a == "--walk-past") {
            g_stop.walk_delay_s = std::stod(next("--walk-past")); g_stop.walk_speed = std::stod(next("--walk-past"));
        } else if (a == "--roll-past") {
            g_stop.roll_delay_s = std::stod(next("--roll-past")); g_stop.roll_speed = std::stod(next("--roll-past"));
        } else if (a == "--stop-gaze-learn") {
            g_stop.gaze_learn_frac = std::stod(next("--stop-gaze-learn"));
        } else if (a == "--stop-gaze-down") {
            g_stop.gaze_down = std::stod(next("--stop-gaze-down"));
        } else if (a == "--stop-gaze-sweep") {
            g_stop.gaze_sweep = std::stod(next("--stop-gaze-sweep")); g_stop.gaze_sweep_yaw = std::stod(next("--stop-gaze-sweep"));
        } else if (a == "--stop-gaze-slew") {
            g_stop.gaze_slew = std::stod(next("--stop-gaze-slew"));
        } else if (a == "--stop-gaze-residual") {
            g_stop.gaze_residual_k = std::stod(next("--stop-gaze-residual"));
        } else if (a == "--stop-gaze-down") {
            g_stop.gaze_down = std::stod(next("--stop-gaze-down"));   // the sweep's pitch centre at every stop (+ is down); 0 = level
        } else if (a == "--log-com") {
            g_log_com = true;
        } else if (a == "--stop-gaze") {
            g_stop.gaze_yaw_sd = std::stod(next("--stop-gaze")); g_stop.gaze_pitch_sd = std::stod(next("--stop-gaze"));
            g_stop.gaze_hold_s = std::stod(next("--stop-gaze")); g_stop.gaze_max_s = std::stod(next("--stop-gaze"));
            g_stop.gaze_quiet = std::stoi(next("--stop-gaze"));
        } else if (a == "--stop-look") {
            g_stop.look_amp = std::stod(next("--stop-look")); g_stop.look_hold_s = std::stod(next("--stop-look")); g_stop.look_max_s = std::stod(next("--stop-look"));
        } else if (a == "--map-on-stop") {
            g_stop.map_on_stop = true;
        } else if (a == "--stop-scan") {
            g_stop.scan_amp = std::stod(next("--stop-scan")); g_stop.scan_hold_s = std::stod(next("--stop-scan"));
        } else if (a == "--head-joints") {
            g_head_joints = true;
        } else if (a == "--head-rate") {
            g_head_rate_k = std::stod(next("--head-rate")); g_head_rate_tau = std::stod(next("--head-rate"));
        } else if (a == "--head-vor") {
            g_head_vor_tau = std::stod(next("--head-vor")); g_head_vor_lead = std::stod(next("--head-vor"));
        } else if (a == "--body-predicts") {
            g_body_predicts = true;
        } else if (a == "--cloud") {
            g_cloud_voxel = 0.04;
            if (i + 1 < argc && argv[i + 1][0] != '-') g_cloud_voxel = std::stod(next("--cloud"));
        } else if (a == "--map-view") {
            const std::string v = next("--map-view");
            if (v != "cloud" && v != "frame") throw std::runtime_error("--map-view takes cloud or frame");
            g_map_view_cloud = v == "cloud";
        } else if (a == "--stop-cloud-end") {
            g_stop.cloud_end_frac = std::stod(next("--stop-cloud-end"));
        } else if (a == "--stop-gaze-sweep-slow") {
            g_stop.gaze_sweep_slow = std::stod(next("--stop-gaze-sweep-slow"));
        } else if (a == "--log-cloud-profile") {
            g_log_cloud_profile = true;
        } else if (a == "--log-motor-tle") {
            g_log_motor_tle = true;
        } else if (a == "--log-tof-cloud") {
            g_log_tof_cloud = true;
        } else if (a == "--seek-gate") {
            g_seek_gate = true;
        } else if (a == "--ref-unwrap") {
            g_ref_unwrap = true;
        } else if (a == "--stuck-progress") {
            g_stuck_progress = true;
        } else if (a == "--stuck-escape") {
            g_stuck_escape_s = std::stod(next("--stuck-escape"));
        } else if (a == "--ref-free") {
            g_ref_free = std::stod(next("--ref-free"));
        } else if (a == "--stop-on-arrive") {
            g_stop.on_arrive = true;
        } else if (a == "--stop-gaze-at-thing") {
            g_stop.gaze_at_thing = true;
        } else if (a == "--stop-on-stuck") {
            g_stop.on_stuck = std::stod(next("--stop-on-stuck"));
        } else if (a == "--skill-on-arrive") {
            g_skill_on_arrive = next("--skill-on-arrive");
            if (g_skill_on_arrive != "kick" && !skill_def(g_skill_on_arrive)) throw std::runtime_error("--skill-on-arrive: unknown skill " + g_skill_on_arrive + " (kick = the side by the thing's bearing, kick_left, kick_right, roulade, peck)");
        } else if (a == "--skill-when-down") {
            g_skill_when_down = next("--skill-when-down");
            if (!skill_def(g_skill_when_down)) throw std::runtime_error("--skill-when-down: unknown skill " + g_skill_when_down);
        } else if (a == "--skill-now") {
            g_skill_now = true;
        } else if (a == "--skill-approach") {
            g_approach_reach = std::stod(next("--skill-approach")); g_approach_vx = std::stod(next("--skill-approach"));
        } else if (a == "--push-reach") {
            g_push_vx = std::stod(next("--push-reach")); g_push_max_s = std::stod(next("--push-reach"));
        } else if (a == "--skill-on-spin") {
            g_spin_skill = next("--skill-on-spin"); g_spin_turns = std::stod(next("--skill-on-spin")); g_spin_net = std::stod(next("--skill-on-spin")); g_spin_secs = std::stod(next("--skill-on-spin"));
        } else if (a == "--skill-unwind-aim") {
            g_skill_unwind_aim = std::stod(next("--skill-unwind-aim"));
        } else if (a == "--skill-unwind") {
            g_skill_unwind_vx = std::stod(next("--skill-unwind")); g_skill_unwind_s = std::stod(next("--skill-unwind"));
        } else if (a == "--skill-at") {
            g_skill_at_s = std::stod(next("--skill-at")); g_skill_at_name = next("--skill-at");
            if (!skill_def(g_skill_at_name)) throw std::runtime_error("--skill-at: unknown skill " + g_skill_at_name);
        } else if (a == "--heading-reflex") {
            g_hr_tau = std::stod(next("--heading-reflex")); g_hr_damp = std::stod(next("--heading-reflex")); g_hr_gate = std::stod(next("--heading-reflex"));
        } else if (a == "--no-backing") {
            g_no_backing = true;
        } else if (a == "--save-head") {
            g_save_head = next("--save-head");
        } else if (a == "--load-head") {
            g_load_head = next("--load-head");
        } else if (a == "--move") {
            MoveOp mv;
            mv.name = next("--move"); mv.x = std::stod(next("--move")); mv.y = std::stod(next("--move"));
            mv.at_s = std::stod(next("--move"));
            g_moves.push_back(mv);
        } else if (a == "--train") {
            g_train.speed = std::stod(next("--train")); g_train.run_s = std::stod(next("--train")); g_train.stop_s = std::stod(next("--train"));
        } else if (a == "--train-phase") {
            g_train.phase_s = std::stod(next("--train-phase"));
        } else if (a == "--log-movers") {
            g_log_movers_s = std::stod(next("--log-movers"));
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
                                                  l2_open_loop ? &l2_twist : nullptr, noise);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 2;
}
