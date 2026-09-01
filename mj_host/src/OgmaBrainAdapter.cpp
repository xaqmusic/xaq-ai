#include "OgmaBrainAdapter.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <functional>
#include <variant>

#include <ogma/GraphConfig.hpp>
#include <ogma/InProcessBus.hpp>
#include <ogma/Module.hpp>
#include <ogma/OgmaInstance.hpp>
#include <ogma/Rng.hpp>
#include <ogma/Topics.hpp>

#include <nlohmann/json.hpp>

namespace mjhost {

namespace {

// Every module that owns an RNG names the param one of these two. Rewriting only
// the first is how the picrawler spent a campaign with a seed flag that changed
// nothing: MotorEPM's stream is called "seed", so OGMA_SEED moved no numbers.
const char* const kSeedParams[] = {"master_seed", "seed"};

}  // namespace

OgmaBrainAdapter::OgmaBrainAdapter(const DuckBody& body, Config config) : c_(std::move(config)) {
    for (int i = 0; i < kNumPolicyJoints; ++i)
        home_[size_t(i)] = (int(c_.home.size()) == kNumPolicyJoints) ? c_.home[size_t(i)]
                                                                     : kHomePose[size_t(i)];

    auto cfg = ogma::GraphConfig::load_from_file(c_.graph_path);

    if (c_.seed != 0) {
        for (auto& m : cfg.modules) {
            for (const char* pname : kSeedParams) {
                auto it = m.params.find(pname);
                if (it == m.params.end()) continue;
                it->second = ogma::ParamValue{int64_t(ogma::namespace_seed(c_.seed, m.id))};
            }
        }
    }

    instance_ = std::make_unique<ogma::OgmaInstance>(std::move(cfg),
                                                     std::make_unique<ogma::InProcessBus>());

    // The action topics this host polls, one per policy joint, named after the
    // joint itself. The graph config must publish exactly these.
    for (int i = 0; i < kNumPolicyJoints; ++i) action_topics_[i] = std::string("action.") + kPolicyJoints[i];

    // Joint travel, read from the model rather than transcribed, so a command can
    // never be sent outside the range the body actually has.
    const mjModel* m = body.model();
    range_.resize(kNumPolicyJoints);
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, kPolicyJoints[i]);
        range_[i] = {m->jnt_range[2 * jid], m->jnt_range[2 * jid + 1]};
    }
}

OgmaBrainAdapter::~OgmaBrainAdapter() = default;

void OgmaBrainAdapter::publish_sensors(const DuckBody& body) {
    auto* bus = instance_->bus();

    const auto publish = [&](const char* sensor, const std::vector<float>& values) {
        auto p = std::make_shared<ogma::ProprioToken>();
        p->tick_id = tick_id_;
        p->producer_id = "host";
        p->sensor = sensor;
        p->values.resize(int(values.size()));
        for (size_t i = 0; i < values.size(); ++i) p->values[int(i)] = values[i];
        bus->publish(std::string("reality.proprio.") + sensor, p);
    };

    // joints — centred on the home pose and scaled by the command amplitude, so
    // the resting pose is the origin and sensor and action share a unit. Clamped
    // because the bridge's own channel is [-1, 1] and a value outside it would be
    // silently squashed there instead of visibly here.
    const auto q = body.joint_positions();
    std::vector<float> joints(kNumPolicyJoints);
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const double norm = (q[i] - home_[size_t(i)]) / c_.amplitude;
        joints[size_t(i)] = float(std::clamp(norm, -1.0, 1.0));
    }
    publish("joints", joints);

    // imu — projected gravity and the gyro, the two channels the real robot
    // publishes. Egocentric, and the same six numbers a hardware brain would see.
    const auto g = body.gravity();
    const auto w = body.gyro();
    publish("imu", {float(g[0]), float(g[1]), float(g[2]),
                    float(w[0]), float(w[1]), float(w[2])});

    // lean — SIGNED FORE/AFT LEAN, the channel A1 was missing.
    //
    // Each motor group's state was [pos, action, delta] per joint: angles, actions,
    // joint velocities, and nothing else. A duck at its exact rest pose has the same
    // proprioceptive state standing up as lying on its side, so an inverted pendulum
    // cannot be balanced from it — which is also why the postural reflex changed
    // nothing, since it pulls toward a joint pose the body was already in.
    //
    // Doctrine §1 step 2: the fix for that is a SENSOR, not a smarter policy.
    // The bridge's load socket appends this as the trailing element of every group's
    // vector, and MotorEPM sizes its state from the arriving vector (`L.x =
    // pt->values`), so the forward model learns how its own actions move the lean
    // without any module being edited.
    //
    // x is fore/aft in the trunk frame (the hip bodies are separated on y), and the
    // same value goes to every group on purpose: each leg owns its own C, so the two
    // legs LEARN opposite responses rather than being told them. LEARNED cooperates,
    // IMPOSED fights.
    //
    // Roll is deliberately absent — the socket is one element wide, and widening it
    // is a module change and therefore a separate conversation.
    publish("lean", {float(g[0]), float(g[0])});

    // lean2 — pitch AND roll, for configs whose bridges set load_slots: 2.  The
    // fore/aft-only prior measured a real degenerate: a duck on its SIDE has
    // g_x = 0 exactly like a standing one, and the prior arms' rescues dropped
    // while tilt ROSE and down-became-quieter in 4/6 seeds — fewer rescues earned
    // by finding poses the lean channel cannot see.  Both components, each signed
    // and linear (a magnitude would be sign-degenerate at 0 and average opposing
    // authorities to zero in a linear model).  Layout is group-major
    // [g0_pitch, g0_roll, g1_pitch, g1_roll]; the head bridge (one group) reads
    // the first two.  The old `lean` topic stays exactly as it was, so every
    // existing config is bit-identical.
    publish("lean2", {float(g[0]), float(g[1]), float(g[0]), float(g[1])});

    // lean4 — pitch, roll, AND THEIR RATES, for configs whose bridges set
    // load_slots: 4.  The measured motivation (stage-3 triage, 2026-08-31): every
    // position-only prior arm ARRESTS falls (tilt held sub-trigger) without ever
    // restoring upright, across prior strengths and HK amplitudes — which is what
    // P-only feedback does on an inverted pendulum: without rate feedback there is
    // no damping, so the loop can slow a fall but not stabilise a point.  The rates
    // were on the bus all along (the gyro), just never in any motor group's state —
    // doctrine §1 step 2, the third time this project has found it: the fix is a
    // SENSOR, not a smarter policy.  Rotation about x tips the body sideways (roll
    // rate, w[0]) and rotation about y tips it fore/aft (pitch rate, w[1]).  Rates
    // are scaled by 0.3 so ±3 rad/s spans the channel (a fall tips at ~1.5 rad/s),
    // then clamped like every other channel.  Group-major, 4 per group.
    const float pr = std::clamp(0.3 * w[1], -1.0, 1.0);
    const float rr = std::clamp(0.3 * w[0], -1.0, 1.0);
    publish("lean4", {float(g[0]), float(g[1]), pr, rr,
                      float(g[0]), float(g[1]), pr, rr});

    // sense — THE FULL IMU VECTOR (2026-09-01): everything the one trunk IMU and
    // the joint encoders can jointly say, conditioned to comparable scale, for
    // configs whose bridges set load_slots: 12.  Twelve slots per group:
    //   0 pitch g_x        1 roll g_y          2 pitch rate ·0.3   3 roll rate ·0.3
    //   4 yaw rate ·0.3    5 accel x /20       6 accel y /20       7 accel z /20
    //   8 head-frame pitch 9 head-frame roll  10 head-CoM Δx /0.1 11 head-CoM Δy /0.1
    // Slots 8–11 are FK reductions of IMU+encoders (see DuckBody's notes): the
    // head-frame attitude the dropped v1 head IMU would have sensed, and the
    // head-CoM offset the trunk IMU is structurally blind to (38 % of the mass
    // can crane forward while trunk lean reads zero).  Head-CoM is an
    // OBSERVATION only — a prior there would fight the counterweight strategy
    // the head measurably runs.  Same values to every group, as with lean.
    const auto a  = body.accel();
    const auto hg = body.head_gravity();
    const auto hc = body.head_com_trunk();
    const float c0 = c_.head_com0.size() == 2 ? float(c_.head_com0[0]) : 0.0f;
    const float c1 = c_.head_com0.size() == 2 ? float(c_.head_com0[1]) : 0.0f;
    const float yr  = std::clamp(0.3 * w[2], -1.0, 1.0);
    const float ax  = std::clamp(a[0] / 20.0, -1.0, 1.0);
    const float ay  = std::clamp(a[1] / 20.0, -1.0, 1.0);
    const float az  = std::clamp(a[2] / 20.0, -1.0, 1.0);
    const float hgx = std::clamp(hg[0], -1.0, 1.0);
    const float hgy = std::clamp(hg[1], -1.0, 1.0);
    const float hcx = std::clamp((hc[0] - c0) / 0.1, -1.0, 1.0);
    const float hcy = std::clamp((hc[1] - c1) / 0.1, -1.0, 1.0);
    std::vector<float> sense = {float(g[0]), float(g[1]), pr, rr, yr, ax, ay, az,
                                hgx, hgy, hcx, hcy};
    std::vector<float> sense2;
    sense2.reserve(24);
    for (int rep = 0; rep < 2; ++rep) sense2.insert(sense2.end(), sense.begin(), sense.end());
    publish("sense", sense2);
}

std::array<double, kNumPolicyJoints> OgmaBrainAdapter::act(const DuckBody& body) {
    publish_sensors(body);

    instance_->tick();

    // Poll AFTER the tick and BEFORE bumping the tick id: modules stamped their
    // tokens with the Scheduler's current tick, and bumping first shifts the
    // comparison frame so every fresh token reads as stale by one.
    auto* bus = instance_->bus();
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        if (auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
                bus->last_value(action_topics_[size_t(i)]))) {
            last_u_[size_t(i)] = double(a->accel);
        }
    }
    ++tick_id_;

    std::array<double, kNumPolicyJoints> target{};
    double mag = 0.0;
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const double u = std::clamp(last_u_[size_t(i)], -1.0, 1.0);
        action_abs_sum_ += std::fabs(u);
        mag += std::fabs(u);
        ++action_samples_;
        const double want = home_[size_t(i)] + c_.amplitude * u;
        target[size_t(i)] = std::clamp(want, range_[size_t(i)].first, range_[size_t(i)].second);
    }
    last_cmd_mag_ = mag / kNumPolicyJoints;
    return target;
}

void OgmaBrainAdapter::on_reset() {
    // The picrawler's auto-reset fired no event and MotorEPM's phase and EMAs
    // survived fall-plus-respawn, so every trend spanning a reset was fake. This
    // is that bug's fix, on the bus where a consumer can actually mask on it.
    auto ev = std::make_shared<ogma::EnvEvent>();
    ev->tick_id = tick_id_;
    ev->producer_id = "host";
    ev->name = "reset";
    ev->intensity = 1.0f;
    instance_->bus()->publish("events.reset", ev);
    last_u_.fill(0.0);
}

void OgmaBrainAdapter::set_learning(bool on) {
    learning_ = on;
    apply_freeze_state();
}

void OgmaBrainAdapter::set_regime_learning(bool on) {
    regime_ok_ = on;
    apply_freeze_state();
}

void OgmaBrainAdapter::apply_freeze_state() {
    const bool on = learning_ && regime_ok_;   // frozen iff EITHER axis says frozen
    if (on == !frozen_now_) return;
    frozen_now_ = !on;

    // Freeze through PARAMETERS rather than by skipping ticks. The brain must keep
    // observing while the scaffold drives — the fall is the most informative thing
    // that happens — but it must not fit the scaffold's policy (§5.6). All four
    // rates are HotMutable, so no module is edited to do this.
    // state_prior_lr joined 2026-08-31: the prior's C/h descent is a learner like the
    // other four, and one that must not integrate the scaffold's rescues (§5.6) — a
    // fall's railed error would wind the prior's bias through every hand-off otherwise.
    static const char* const kRates[] = {"model_lr", "ctrl_lr", "bias_lr", "sat_lr",
                                         "state_prior_lr", "state_prior_h_lr",
                                         "state_model_lr"};
    bool reported = false;
    for (auto* module : instance_->modules()) {
        const std::string type(module->type_name());
        if (type != "MotorEPM" && type != "MotorEPMv2") continue;
        const auto params = module->current_params();
        const std::string id(module->id());
        for (const char* rate : kRates) {
            auto it = params.find(rate);
            if (it == params.end()) continue;
            const std::string key = id + ":" + rate;
            // §3.2 rule 5: verify the consumer fired. Reported once, the first time
            // a freeze happens, because "I called set_param" and "the value changed"
            // are different claims and only the second one matters.
            if (!announced_freeze_ && !on) {
                if (const double* v = std::get_if<double>(&it->second))
                    std::fprintf(stderr, "  freeze: %s %s %.4f -> 0\n", id.c_str(), rate, *v);
            }
            if (on) {
                auto saved = frozen_rates_.find(key);
                if (saved != frozen_rates_.end())
                    module->on_param_change(rate, ogma::ParamValue{saved->second});
            } else {
                if (const double* v = std::get_if<double>(&it->second)) frozen_rates_[key] = *v;
                module->on_param_change(rate, ogma::ParamValue{0.0});
                reported = true;
            }
        }
    }
    if (reported) announced_freeze_ = true;

    // And confirm it landed, by reading the params back rather than assuming.
    if (!on) {
        for (auto* module : instance_->modules()) {
            const std::string type(module->type_name());
            if (type != "MotorEPM" && type != "MotorEPMv2") continue;
            const auto after = module->current_params();
            for (const char* rate : kRates) {
                auto it = after.find(rate);
                if (it == after.end()) continue;
                const double* v = std::get_if<double>(&it->second);
                if (v && *v != 0.0) {
                    std::fprintf(stderr, "  !! freeze did NOT take: %s %s is still %.4f\n",
                                 std::string(module->id()).c_str(), rate, *v);
                }
            }
        }
    }
}

void OgmaBrainAdapter::sample_tle(bool upright) {
    double tle = 0.0;
    int n = 0;
    for (auto* m : instance_->modules()) {
        const std::string type(m->type_name());
        if (type != "MotorEPM" && type != "MotorEPMv2") continue;
        const auto d = m->diag_lite();
        if (!d.contains("motor_tle")) continue;
        tle += d["motor_tle"].get<double>();
        ++n;
    }
    if (!n) return;
    tle /= n;
    if (upright) { tle_up_sum_ += tle; ++tle_up_n_; }
    else         { tle_down_sum_ += tle; ++tle_down_n_; }
}

double OgmaBrainAdapter::tle_upright() const {
    return tle_up_n_ ? tle_up_sum_ / double(tle_up_n_) : 0.0;
}
double OgmaBrainAdapter::tle_down() const {
    return tle_down_n_ ? tle_down_sum_ / double(tle_down_n_) : 0.0;
}

std::vector<std::string> OgmaBrainAdapter::diagnostics() const {
    std::vector<std::string> out;
    for (auto* m : instance_->modules()) {
        const auto d = m->diag_lite();
        if (d.is_null() || d.empty()) continue;
        std::string line = std::string(m->id()) + " " + d.dump();
        // clip_duty is measured PRE-bound (how often the request exceeds |1|), so it
        // stays meaningful whether the bound is a hard clamp or a squash.
        {
            const auto snap = m->diag_snapshot();
            if (snap.contains("clip_duty")) {
                char buf[48];
                std::snprintf(buf, sizeof buf, "  clip_duty=%.3f",
                              snap["clip_duty"].get<double>());
                line += buf;
            }
            if (snap.contains("state_prior_active") && snap["state_prior_active"].get<bool>()) {
                char buf[80];
                std::snprintf(buf, sizeof buf, "  sp_err=%.3f sp_applied=%d calm=%.2f",
                              snap["state_prior_err"].get<double>(),
                              snap["state_prior_applied"].get<int>(),
                              snap.value("state_prior_calm_mult", 1.0));
                line += buf;
                char buf2[48];
                std::snprintf(buf2, sizeof buf2, " cpm=%.2f",
                              snap.value("state_prior_cp_norm", 0.0));
                line += buf2;
            }
        }
        // The operating point, read back from the module rather than from the config
        // that was supposed to set it — a run that prints its own arm cannot be a
        // silent-confound run.
        {
            const auto params = m->current_params();
            for (const char* k : {"motor_gain", "c_init", "cmd_squash", "sat_lr",
                                  "state_prior_gain", "state_prior_lr"}) {
                auto it = params.find(k);
                if (it == params.end()) continue;
                if (const double* v = std::get_if<double>(&it->second)) {
                    char buf[64];
                    std::snprintf(buf, sizeof buf, "  %s=%.2f", k, *v);
                    line += buf;
                }
            }
        }
        // §3.2 rule 5 again: the state width, read from the module rather than
        // inferred from the config. An appended sensor channel that never widened
        // the model is a knob that cannot act, and it would look causal anyway.
        // Search the snapshot for the forward model's own row count rather than
        // guessing at its layout: A maps action -> state, so rows_A IS the state
        // width the model was built to.
        std::function<int(const nlohmann::json&)> find_rows = [&](const nlohmann::json& j) -> int {
            if (j.is_object()) {
                auto it = j.find("rows_A");
                if (it != j.end() && it->is_number()) return it->get<int>();
                for (auto& kv : j.items()) { int r = find_rows(kv.value()); if (r) return r; }
            } else if (j.is_array()) {
                for (auto& e : j) { int r = find_rows(e); if (r) return r; }
            }
            return 0;
        };
        if (const int rows = find_rows(m->snapshot_state())) line += "  state_dim=" + std::to_string(rows);
        // The fight for C, made visible (2026-08-31): the prior writes feedback into
        // C's trailing (lean) columns while HK's own dC keeps rewriting all of C.
        // Whether the prior's columns SURVIVE is the mechanism question every duck
        // A/B keeps asking, so the host prints it: per leg, the Frobenius norm of
        // the last-4 columns of C, and the largest |h|.
        {
            const auto snap = m->snapshot_state();
            if (snap.contains("legs") && snap["legs"].is_array()) {
                std::string extra;
                for (const auto& lj : snap["legs"]) {
                    if (!lj.contains("C") || !lj.contains("h")) continue;
                    const auto C = lj["C"].get<std::vector<float>>();
                    const auto h = lj["h"].get<std::vector<float>>();
                    const int mm = int(h.size());
                    if (mm == 0 || C.size() % size_t(mm) != 0) continue;
                    const int nn = int(C.size()) / mm;
                    double c4 = 0.0;
                    for (int col = std::max(0, nn - 4); col < nn; ++col)
                        for (int j = 0; j < mm; ++j)
                            c4 += double(C[size_t(col) * size_t(mm) + size_t(j)])
                                  * double(C[size_t(col) * size_t(mm) + size_t(j)]);
                    double hmax = 0.0;
                    for (float v : h) hmax = std::max(hmax, double(std::fabs(v)));
                    // Cp: the prior's OWN feedback columns, resolved from the
                    // module's state_prior_indices — the "effective kp" the
                    // gain-gap study watches grow (or stall).
                    double cp = 0.0;
                    {
                        const auto params = m->current_params();
                        auto it = params.find("state_prior_indices");
                        if (it != params.end())
                            if (auto* v = std::get_if<std::vector<double>>(&it->second))
                                for (double di : *v) {
                                    int idx = int(di); if (idx < 0) idx += nn;
                                    if (idx < 0 || idx >= nn) continue;
                                    for (int j = 0; j < mm; ++j)
                                        cp += double(C[size_t(idx) * size_t(mm) + size_t(j)])
                                              * double(C[size_t(idx) * size_t(mm) + size_t(j)]);
                                }
                    }
                    char buf[96];
                    std::snprintf(buf, sizeof buf, " C4=%.2f Cp=%.2f hmax=%.2f",
                                  std::sqrt(c4), std::sqrt(cp), hmax);
                    extra += buf;
                }
                if (!extra.empty()) line += " |" + extra;
                // One-off model-inspection dump (OGMA_DUMP_MODEL=1): the learned
                // A's trailing-4 rows (the model's authority estimate over the
                // lean block) — directly comparable against --probe's measured J.
                if (std::getenv("OGMA_DUMP_MODEL")) {
                    std::string dump = "\n    A(trailing rows):";
                    int legidx = 0;
                    for (const auto& lj : snap["legs"]) {
                        if (!lj.contains("A") || !lj.contains("h")) continue;
                        const auto A = lj["A"].get<std::vector<float>>();
                        const int mm = int(lj["h"].get<std::vector<float>>().size());
                        if (mm == 0 || A.size() % size_t(mm) != 0) continue;
                        const int nn = int(A.size()) / mm;   // A is n x m col-major
                        char hdr[32]; std::snprintf(hdr, sizeof hdr, "\n      leg%d:", legidx++);
                        dump += hdr;
                        for (int row = std::max(0, nn - 4); row < nn; ++row) {
                            dump += " [";
                            for (int j = 0; j < mm; ++j) {
                                char buf[16];
                                std::snprintf(buf, sizeof buf, "%+.3f%s",
                                              A[size_t(j) * size_t(nn) + size_t(row)],
                                              j + 1 < mm ? " " : "");
                                dump += buf;
                            }
                            dump += "]";
                        }
                    }
                    line += dump;
                }
            }
        }
        out.push_back(line);
    }
    return out;
}

double OgmaBrainAdapter::mean_abs_action() const {
    return action_samples_ ? action_abs_sum_ / double(action_samples_) : 0.0;
}

std::vector<std::string> OgmaBrainAdapter::module_ids() const {
    std::vector<std::string> out;
    for (auto* m : instance_->modules()) out.emplace_back(m->id());
    return out;
}

}  // namespace mjhost
