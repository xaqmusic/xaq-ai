// =============================================================================
// epm_kalman_bench.cpp — synthetic bench for the Kalman-lessons campaign
// =============================================================================
//
// Charter: docs/plans-and-designs/epm_kalman_lessons_plan.md (Stage 0.1).
//
// Drives the REAL modules (EPM, DescendingPredictor, LateralVoter) on an
// InProcessBus against streams whose optimal estimator is known in closed
// form, and emits one JSON line per tick so tools/epm_bench/analyze.py can
// compare the substrate against the Kalman reference.  Nothing here is a
// test; nothing here changes behaviour.  The EPM runs with encoder_kind =
// identity so prototype space == observation space and "prototype MSE
// against the true mean" means what it says.
//
// Scenarios (--scenario):
//   S1   one static Gaussian cluster           → sample mean is optimal
//   S1m  K static clusters, Markov visitation   → per-cluster means; purity
//   S2   drifting mean (random walk, --q)       → scalar KF steady-state gain
//   S3   2-axis damped-rotation target, closed predictor↔EPM pair
//                                              → exact matrix KF innovation
//   S4   3 states in a deterministic cycle with random teleports
//                                              → "expected" vs "unexpected"
//                                                 transitions
//   S5   two sensors of one state, R_b = ratio²·R_a, fused by the voter
//                                              → inverse-variance weights;
//                                                 arms: --dead_at, --placeholder_at,
//                                                 --subrate
//   S9   K clusters, Markov visitation (as S1m), plus a TARGET channel: a value per cluster + noise; a Generator
//        decodes the EPM's nodes into the target `--horizon` ticks ahead → the exact floor is
//        target noise + the spread the switching adds over the horizon (generator_and_belief_filter.md, G1)
//   S10  K overlapping clusters, sticky Markov switching, optional periodic DROPOUT (--dropout_every / --dropout_len):
//        the BeliefFilter over the EPM's nodes vs the raw reading, the EPM winner's prototype, and the forward filter
//        given the TRUE model (the optimum) → MSE of the estimate against the true cluster mean (G2)
//   S6   a ring of K poses visited IN ORDER (dwell ticks each), the whole ring
//        rotating slowly (--omega rad/tick)   → the gait-like stream the picrawler
//                                                 body_pose EPM sees; measures
//                                                 vocabulary TURNOVER (Stage 1 gap)
//
// Noise convention: --sigma is the EXPECTED NOISE NORM, so per-dim sd is
// sigma/sqrt(dim).  With the default sigma 0.1 the per-tick squared
// quantisation error of a converged node is ~0.01, below the GNG's default
// min_insertion_error of 0.02 (which gates on SQUARED distance), so nodes can
// bake.  Raise sigma above ~0.14 and the default gate will refuse to bake —
// that is §0 rule 2 in miniature, and insertion_autotune is the honest fix.
//
// Param overrides: --set <module>.<key>=<json>   e.g. --set epm.baking_threshold=30
//   module ∈ {epm, epm_a, epm_b, pred, voter}; "epm" applies to every EPM.
//
// Usage:
//   epm_kalman_bench --scenario S1 --seed 3 --ticks 4000 --out s1_3.jsonl
//                    [--dim 8] [--sigma 0.1] [--K 3] [--q 0.0] [--p_switch 0.02]
//                    [--dwell 20] [--sigma_b_ratio 3] [--dead_at T]
//                    [--placeholder_at T] [--subrate N] [--dump_every 50]
//                    [--set module.key=json]*

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/DescendingPredictor.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/modules/Generator.hpp"
#include "ogma/modules/BeliefFilter.hpp"

using json = nlohmann::json;

namespace {

struct Args {
    std::string scenario   = "S1";
    uint64_t    seed       = 0;
    int         ticks      = 4000;
    std::string out;
    int         dim        = 8;
    double      sigma      = 0.1;     // expected noise norm
    int         K          = 3;       // S1m clusters
    int         horizon    = 0;       // S9 generator horizon (ticks)
    int         dropout_every = 0;    // S10: a silent window starts every N ticks (0 = none)
    int         dropout_len   = 0;    // S10: silent ticks per window
    double      target_sigma = 0.1;   // S9 target noise (sd)
    double      p_stay     = 0.9;     // S1m Markov self-probability
    double      q          = 0.0;     // S2/S5 random-walk variance per tick (norm²); S3 process noise
    double      p_switch   = 0.02;    // S4 teleport probability per transition
    int         dwell      = 20;      // S4 ticks per state
    double      sigma_b_ratio = 3.0;  // S5 sensor b noise / sensor a noise
    int         dead_at        = -1;  // S5: sensor b goes constant from this tick
    int         placeholder_at = -1;  // S5: sensor b publishes nothing on this tick
    int         subrate        = 1;   // S5: sensor b EPM process_every_n_ticks
    int         dump_every     = 50;  // node dumps cadence (0 = only on bake)
    double      omega          = 0.0; // S6: ring rotation, rad per tick (0 = stationary ring)
    double      theta          = 0.15; // S3: per-tick rotation of the damped target (fast = exposes a one-tick lag)
    double      rho            = 0.98; // S3: per-tick damping
    std::map<std::string, std::map<std::string, json>> sets;  // module → key → value
};

[[noreturn]] void die(std::string const& msg) {
    std::cerr << "epm_kalman_bench: " << msg << "\n";
    std::exit(2);
}

Args parse_args(int argc, char** argv) {
    Args a;
    auto need = [&](int& i) -> std::string {
        if (i + 1 >= argc) die(std::string("missing value after ") + argv[i]);
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if      (k == "--scenario")       a.scenario = need(i);
        else if (k == "--seed")           a.seed = std::stoull(need(i));
        else if (k == "--ticks")          a.ticks = std::stoi(need(i));
        else if (k == "--out")            a.out = need(i);
        else if (k == "--dim")            a.dim = std::stoi(need(i));
        else if (k == "--sigma")          a.sigma = std::stod(need(i));
        else if (k == "--K")              a.K = std::stoi(need(i));
        else if (k == "--horizon")        a.horizon = std::stoi(need(i));
        else if (k == "--dropout_every")  a.dropout_every = std::stoi(need(i));
        else if (k == "--dropout_len")    a.dropout_len = std::stoi(need(i));
        else if (k == "--target_sigma")   a.target_sigma = std::stod(need(i));
        else if (k == "--p_stay")         a.p_stay = std::stod(need(i));
        else if (k == "--q")              a.q = std::stod(need(i));
        else if (k == "--p_switch")       a.p_switch = std::stod(need(i));
        else if (k == "--dwell")          a.dwell = std::stoi(need(i));
        else if (k == "--sigma_b_ratio")  a.sigma_b_ratio = std::stod(need(i));
        else if (k == "--dead_at")        a.dead_at = std::stoi(need(i));
        else if (k == "--placeholder_at") a.placeholder_at = std::stoi(need(i));
        else if (k == "--subrate")        a.subrate = std::stoi(need(i));
        else if (k == "--dump_every")     a.dump_every = std::stoi(need(i));
        else if (k == "--omega")          a.omega = std::stod(need(i));
        else if (k == "--theta")          a.theta = std::stod(need(i));
        else if (k == "--rho")            a.rho = std::stod(need(i));
        else if (k == "--set") {
            std::string kv = need(i);
            auto eq = kv.find('=');
            auto dot = kv.find('.');
            if (eq == std::string::npos || dot == std::string::npos || dot > eq)
                die("--set expects <module>.<key>=<json>, got " + kv);
            std::string mod = kv.substr(0, dot);
            std::string key = kv.substr(dot + 1, eq - dot - 1);
            std::string raw = kv.substr(eq + 1);
            json v;
            try { v = json::parse(raw); } catch (...) { v = raw; }   // bare string
            a.sets[mod][key] = v;
        }
        else die("unknown argument " + k);
    }
    if (a.out.empty()) die("--out is required");
    return a;
}

ogma::ParamValue to_param(json const& v) {
    if (v.is_boolean())        return ogma::ParamValue{v.get<bool>()};
    if (v.is_number_integer()) return ogma::ParamValue{v.get<int64_t>()};
    if (v.is_number_float())   return ogma::ParamValue{v.get<double>()};
    if (v.is_string())         return ogma::ParamValue{v.get<std::string>()};
    if (v.is_array()) {
        if (!v.empty() && v[0].is_string()) return ogma::ParamValue{v.get<std::vector<std::string>>()};
        return ogma::ParamValue{v.get<std::vector<double>>()};
    }
    die("unsupported --set value type: " + v.dump());
}

// Apply overrides addressed to `name`; "epm" also reaches every EPM instance.
void apply_sets(ogma::ParamMap& p, Args const& a, std::string const& name, bool is_epm) {
    auto apply = [&](std::string const& mod) {
        auto it = a.sets.find(mod);
        if (it == a.sets.end()) return;
        for (auto const& [k, v] : it->second) p[k] = to_param(v);
    };
    if (is_epm) apply("epm");
    apply(name);
}

ogma::ParamMap epm_params(Args const& a, std::string const& name, std::string const& group,
                          std::string const& modality, std::string const& input_topic,
                          int dim, bool subtract_prediction) {
    ogma::ParamMap p{
        {"modality_group",  std::string(group)},
        {"modality_name",   std::string(modality)},
        {"encoder_kind",    std::string("identity")},
        {"input_topic",     std::string(input_topic)},
        {"projection_dim",  int64_t{dim}},
        {"subtract_descending_prediction", subtract_prediction},
        // Stated explicitly because the runtime passes config params VERBATIM
        // (OgmaInstance.cpp:45) and never merges params_schema() defaults: an
        // EPM whose config omits this key runs GNG::Config's own default of
        // 100, not the 50 the schema and EPM.md advertise (found 2026-09-05,
        // Stage 0 of the Kalman-lessons campaign).  The live picrawler stack
        // sets 50; the bench baseline is the documented substrate.
        {"baking_threshold", int64_t{50}},
    };
    apply_sets(p, a, name, /*is_epm=*/true);
    return p;
}

// ---- RNG helpers ----------------------------------------------------------

Eigen::VectorXf gauss(std::mt19937_64& rng, int dim, double sd_per_dim) {
    std::normal_distribution<double> nd(0.0, 1.0);
    Eigen::VectorXf v(dim);
    for (int i = 0; i < dim; ++i) v(i) = float(nd(rng) * sd_per_dim);
    return v;
}

Eigen::VectorXf unit_vec(std::mt19937_64& rng, int dim) {
    Eigen::VectorXf v = gauss(rng, dim, 1.0);
    float n = v.norm();
    return n > 1e-9f ? Eigen::VectorXf(v / n) : v;
}

// ---- JSON helpers ---------------------------------------------------------

json vec_json(Eigen::VectorXf const& v) {
    json a = json::array();
    for (int i = 0; i < v.size(); ++i) a.push_back(v(i));
    return a;
}

json token_json(std::shared_ptr<const ogma::RealityToken> const& t) {
    if (!t) return nullptr;
    return json{
        {"winner",     t->winner_id},
        {"qe",         t->quant_error},
        {"eerr",       t->expected_error},
        {"ts",         t->transition_surp},
        {"tle",        t->tle},
        {"nodes",      t->node_count},
        {"baked",      t->baked_count},
        {"novel",      t->is_novel},
        {"just_baked", t->just_baked},
        {"mitosis",    t->mitosis_count},
        {"drift",      t->drift_count},
    };
}

json nodes_json(ogma::EPM const& epm) {
    json snap = epm.snapshot_state();
    json out = json::array();
    if (!snap.contains("gng") || snap["gng"].is_null()) return out;
    for (auto const& n : snap["gng"]["nodes"]) {
        out.push_back(json{
            {"id",        n.value("id", -1)},
            {"visits",    n.value("visits", 0)},
            {"ema_error", n.value("ema_error", 0.0)},
            {"health",    n.value("health", 0.0)},
            {"proto",     n["prototype"]},
        });
    }
    return out;
}

std::shared_ptr<ogma::ConsensusToken> make_obs(Eigen::VectorXf const& v, uint64_t t) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->tick_id         = t;
    c->producer_id     = "bench";
    c->fused_embedding = v;
    return c;
}

std::shared_ptr<const ogma::RealityToken> last_token(ogma::Bus& bus, std::string const& topic) {
    return std::dynamic_pointer_cast<const ogma::RealityToken>(bus.last_value(topic));
}

struct Emitter {
    std::ofstream f;
    explicit Emitter(std::string const& path) : f(path) {
        if (!f) die("cannot open " + path);
    }
    void line(json const& j) { f << j.dump() << '\n'; }
};

json header(Args const& a, json extra) {
    json sets = json::object();
    for (auto const& [m, kv] : a.sets) for (auto const& [k, v] : kv) sets[m][k] = v;
    json h{
        {"event", "header"}, {"scenario", a.scenario}, {"seed", a.seed},
        {"ticks", a.ticks}, {"dim", a.dim}, {"sigma", a.sigma}, {"sets", sets},
    };
    for (auto& [k, v] : extra.items()) h[k] = v;
    return h;
}

// One EPM on one observation stream; the common skeleton for S1/S1m/S2/S4.
struct SingleEpm {
    ogma::InProcessBus bus;
    ogma::EPM          epm;
    Args const&        a;
    Emitter&           em;
    int                dim;
    bool               baked_dumped = false;

    SingleEpm(Args const& args, Emitter& e, int d) : a(args), em(e), dim(d) {
        epm.set_id("epm");
        epm.on_setup(&bus, epm_params(a, "epm", "bench", "s", "obs.s", dim, /*subtract=*/false));
    }

    // Publish x, tick, log.  `truth` is scenario-specific and spliced into the line.
    void step(uint64_t t, Eigen::VectorXf const& x, json truth) {
        bus.begin_tick(t);
        bus.publish("obs.s", make_obs(x, t));
        epm.tick(t);
        bus.end_tick();
        auto tok = last_token(bus, "reality.bench.s");
        json line{{"t", t}, {"x", vec_json(x)}, {"tok", token_json(tok)}};
        for (auto& [k, v] : truth.items()) line[k] = v;
        em.line(line);
        bool dump = (a.dump_every > 0 && t % uint64_t(a.dump_every) == 0) || (tok && tok->just_baked);
        if (dump) em.line(json{{"event", "nodes"}, {"t", t}, {"nodes", nodes_json(epm)}});
    }
};

// ---- S1: one static cluster ------------------------------------------------

int run_S1(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const double sd = a.sigma / std::sqrt(double(a.dim));
    Eigen::VectorXf mu = unit_vec(rng, a.dim);
    em.line(header(a, json{{"mu", vec_json(mu)}, {"sd_per_dim", sd}}));
    SingleEpm s(a, em, a.dim);
    for (int t = 1; t <= a.ticks; ++t) {
        Eigen::VectorXf x = mu + gauss(rng, a.dim, sd);
        s.step(uint64_t(t), x, json::object());
    }
    return 0;
}

// ---- S1m: K clusters, Markov visitation ------------------------------------

int run_S1m(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const double sd = a.sigma / std::sqrt(double(a.dim));
    std::vector<Eigen::VectorXf> mus;
    json mus_j = json::array();
    for (int k = 0; k < a.K; ++k) { mus.push_back(unit_vec(rng, a.dim)); mus_j.push_back(vec_json(mus.back())); }
    em.line(header(a, json{{"K", a.K}, {"mus", mus_j}, {"p_stay", a.p_stay}, {"sd_per_dim", sd}}));
    SingleEpm s(a, em, a.dim);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::uniform_int_distribution<int> pick(0, a.K - 1);
    int k = pick(rng);
    for (int t = 1; t <= a.ticks; ++t) {
        if (u(rng) > a.p_stay) { int nk = pick(rng); if (nk == k) nk = (nk + 1) % a.K; k = nk; }
        Eigen::VectorXf x = mus[k] + gauss(rng, a.dim, sd);
        s.step(uint64_t(t), x, json{{"k", k}});
    }
    return 0;
}

// ---- S9: decode a target from the EPM's nodes (Generator, stage G1) ----------

int run_S9(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const double sd = a.sigma / std::sqrt(double(a.dim));
    const int K = a.K, h = a.horizon;
    std::vector<Eigen::VectorXf> mus;
    std::vector<double> tv;
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::uniform_int_distribution<int> pick(0, K - 1);
    for (int k = 0; k < K; ++k) { mus.push_back(unit_vec(rng, a.dim)); tv.push_back(2.0 * u(rng) - 1.0); }
    // The exact floor: knowing the true cluster c, the best prediction of the target h ticks on is
    // m_c = sum_j P^h(c, j) t_j, and its mean squared error is target noise + sum_j P^h(c, j) (t_j - m_c)^2.
    Eigen::MatrixXd T = Eigen::MatrixXd::Constant(K, K, (1.0 - a.p_stay) / double(K - 1));
    for (int k = 0; k < K; ++k) T(k, k) = a.p_stay;
    Eigen::MatrixXd Ph = Eigen::MatrixXd::Identity(K, K);
    for (int i = 0; i < h; ++i) Ph = Ph * T;
    double floor = 0.0, var_all = 0.0, mean_all = 0.0;
    for (int k = 0; k < K; ++k) mean_all += tv[size_t(k)] / K;
    for (int c = 0; c < K; ++c) {
        double m = 0.0; for (int j = 0; j < K; ++j) m += Ph(c, j) * tv[size_t(j)];
        double v = 0.0; for (int j = 0; j < K; ++j) v += Ph(c, j) * (tv[size_t(j)] - m) * (tv[size_t(j)] - m);
        floor += v / K;
        var_all += (tv[size_t(c)] - mean_all) * (tv[size_t(c)] - mean_all) / K;
    }
    floor += a.target_sigma * a.target_sigma;
    const double mean_base = var_all + a.target_sigma * a.target_sigma;
    em.line(header(a, json{{"K", K}, {"p_stay", a.p_stay}, {"horizon", h}, {"target_sigma", a.target_sigma},
                           {"targets", tv}, {"floor_mse", floor}, {"mean_baseline_mse", mean_base}}));
    ogma::InProcessBus bus;
    ogma::EPM epm; epm.set_id("epm");
    epm.on_setup(&bus, epm_params(a, "epm", "bench", "s", "obs.s", a.dim, /*subtract=*/false));
    ogma::Generator gen; gen.set_id("gen");
    gen.on_setup(&bus, ogma::ParamMap{{"reality_topic", std::string("reality.bench.s")},
                                      {"target_topics", std::vector<std::string>{"target.s"}},
                                      {"horizon", int64_t(h)}, {"warmup", int64_t(20)}});
    std::normal_distribution<double> tn(0.0, a.target_sigma);
    std::map<int, std::vector<int>> wc;   // winner -> counts per true cluster (purity, second half)
    double se2 = 0.0; int n2 = 0;         // second-half squared error of the published prediction (horizon 0 only)
    int k = pick(rng);
    for (int t = 1; t <= a.ticks; ++t) {
        if (u(rng) > a.p_stay) { int nk = pick(rng); if (nk == k) nk = (nk + 1) % K; k = nk; }
        Eigen::VectorXf x = mus[size_t(k)] + gauss(rng, a.dim, sd);
        bus.begin_tick(uint64_t(t));
        bus.publish("obs.s", make_obs(x, uint64_t(t)));
        epm.tick(uint64_t(t));
        auto tgt = std::make_shared<ogma::ProprioToken>();
        tgt->tick_id = uint64_t(t); tgt->producer_id = "bench"; tgt->values.resize(1);
        tgt->values[0] = float(tv[size_t(k)] + tn(rng));
        bus.publish("target.s", tgt);
        gen.tick(uint64_t(t));
        auto gp = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("gen.gen"));
        bus.end_tick();
        if (h == 0 && t > a.ticks / 2 && gp && gp->tick_id == uint64_t(t)) {
            const double e = double(gp->values[0]) - double(tgt->values[0]); se2 += e * e; ++n2;
        }
        if (t > a.ticks / 2) {
            auto tok = last_token(bus, "reality.bench.s");
            if (tok) { auto& v = wc[tok->winner_id]; if (v.empty()) v.assign(size_t(K), 0); v[size_t(k)]++; }
        }
    }
    int imp = 0, tot = 0;
    for (auto const& [w, v] : wc) { int mx = 0, sm = 0; for (int c : v) { mx = std::max(mx, c); sm += c; } imp += sm - mx; tot += sm; }
    auto d = gen.diag_snapshot();
    em.line(json{{"event", "summary"}, {"gen_mse", d["mse"]}, {"floor_mse", floor}, {"mean_baseline_mse", mean_base},
                 {"gen_mean_baseline_measured", d["mse_mean_baseline"]}, {"gen_persistence", d["mse_persistence"]},
                 {"nis", d["nis"]}, {"nodes_tracked", d["nodes_tracked"]}, {"scored", d["scored"]},
                 {"excess_over_floor", double(d["mse"]) / floor - 1.0},
                 {"impure_frac", tot > 0 ? double(imp) / tot : 0.0},
                 {"second_half_mse", n2 > 0 ? se2 / n2 : -1.0}});
    return 0;
}

// ---- S10: the belief filter vs the true-model forward filter (G2) ------------

int run_S10(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const int K = a.K, D = a.dim;
    const double sd = a.sigma / std::sqrt(double(D));
    std::vector<Eigen::VectorXf> mus;
    for (int k = 0; k < K; ++k) mus.push_back(unit_vec(rng, D));
    em.line(header(a, json{{"K", K}, {"p_stay", a.p_stay}, {"sd_per_dim", sd}, {"dropout_every", a.dropout_every},
                           {"dropout_len", a.dropout_len}}));
    ogma::InProcessBus bus;
    ogma::EPM epm; epm.set_id("epm");
    epm.on_setup(&bus, epm_params(a, "epm", "bench", "s", "obs.s", D, /*subtract=*/false));
    ogma::BeliefFilter bf; bf.set_id("bf");
    bf.on_setup(&bus, ogma::ParamMap{{"reality_topic", std::string("reality.bench.s")}, {"obs_topic", std::string("obs.raw")}});
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::uniform_int_distribution<int> pick(0, K - 1);
    // the oracle: the forward filter with the true transition matrix, means and noise
    std::vector<double> ob(size_t(K), 1.0 / K);
    const double pswitch = (1.0 - a.p_stay) / double(K - 1);
    int k = pick(rng);
    Eigen::VectorXf last_proto = Eigen::VectorXf::Zero(D), last_obs = Eigen::VectorXf::Zero(D);
    double se_raw = 0, se_epm = 0, se_bf = 0, se_or = 0; int n_on = 0;
    double de_epm = 0, de_bf = 0, de_or = 0; int n_off = 0;
    double ent_on = 0, ent_off = 0;
    // A single-hypothesis Kalman filter (random-walk state, per dimension, known measurement noise), at several process
    // noises; the best one is reported, so the comparison favours it.
    const std::vector<double> kq = {1e-5, 1e-4, 1e-3, 3e-3, 1e-2, 3e-2, 1e-1};
    std::vector<Eigen::VectorXf> kx(kq.size(), Eigen::VectorXf::Zero(D));
    std::vector<double> kp(kq.size(), 1.0), kse_on(kq.size(), 0.0), kse_off(kq.size(), 0.0);
    for (int t = 1; t <= a.ticks; ++t) {
        if (u(rng) > a.p_stay) { int nk = pick(rng); if (nk == k) nk = (nk + 1) % K; k = nk; }
        const bool silent = a.dropout_every > 0 && (t % a.dropout_every) < a.dropout_len && t > a.ticks / 4;
        Eigen::VectorXf x = mus[size_t(k)] + gauss(rng, D, sd);
        // oracle predict
        std::vector<double> pr(size_t(K), 0.0);
        for (int i = 0; i < K; ++i) for (int j = 0; j < K; ++j) pr[size_t(j)] += ob[size_t(i)] * (i == j ? a.p_stay : pswitch);
        if (!silent) {
            double z = 0.0;
            for (int j = 0; j < K; ++j) {
                const double d2 = double((x - mus[size_t(j)]).squaredNorm());
                pr[size_t(j)] *= std::exp(-0.5 * d2 / (sd * sd)); z += pr[size_t(j)];
            }
            for (double& v : pr) v /= z;
        }
        ob = pr;
        Eigen::VectorXf oest = Eigen::VectorXf::Zero(D);
        for (int j = 0; j < K; ++j) oest += float(ob[size_t(j)]) * mus[size_t(j)];
        bus.begin_tick(uint64_t(t));
        if (!silent) {
            bus.publish("obs.s", make_obs(x, uint64_t(t)));
            epm.tick(uint64_t(t));
            auto ro = std::make_shared<ogma::ProprioToken>();
            ro->tick_id = uint64_t(t); ro->producer_id = "bench"; ro->values = x;
            bus.publish("obs.raw", ro);
            auto tok = last_token(bus, "reality.bench.s");
            if (tok && tok->winner_prototype.size() == D) last_proto = tok->winner_prototype;
            last_obs = x;
        }
        bf.tick(uint64_t(t));
        bus.end_tick();
        for (size_t qi = 0; qi < kq.size(); ++qi) {
            kp[qi] += kq[qi];
            if (!silent) {
                const double r = sd * sd, g = kp[qi] / (kp[qi] + r);
                kx[qi] += float(g) * (x - kx[qi]); kp[qi] *= (1.0 - g);
            }
            if (t > a.ticks / 2) {
                const double e = double((kx[qi] - mus[size_t(k)]).squaredNorm()) / D;
                if (!silent) kse_on[qi] += e; else kse_off[qi] += e;
            }
        }
        if (t > a.ticks / 2 && bf.estimate().size() == D) {
            const Eigen::VectorXf& m = mus[size_t(k)];
            if (!silent) {
                se_raw += (x - m).squaredNorm() / D; se_epm += (last_proto - m).squaredNorm() / D;
                se_bf += (bf.estimate() - m).squaredNorm() / D; se_or += (oest - m).squaredNorm() / D; ++n_on;
                ent_on += bf.entropy();
            } else {
                de_epm += (last_proto - m).squaredNorm() / D; de_bf += (bf.estimate() - m).squaredNorm() / D;
                de_or += (oest - m).squaredNorm() / D; ++n_off; ent_off += bf.entropy();
            }
        }
    }
    json sm{{"event", "summary"}, {"n_on", n_on}, {"n_off", n_off}};
    if (n_on) { sm["mse_raw"] = se_raw / n_on; sm["mse_epm"] = se_epm / n_on; sm["mse_bf"] = se_bf / n_on; sm["mse_oracle"] = se_or / n_on; sm["entropy_on"] = ent_on / n_on; }
    if (n_off) { sm["drop_mse_epm"] = de_epm / n_off; sm["drop_mse_bf"] = de_bf / n_off; sm["drop_mse_oracle"] = de_or / n_off; sm["entropy_off"] = ent_off / n_off; }
    if (n_on) { double b = 1e300; for (double v : kse_on) b = std::min(b, v / n_on); sm["mse_kf_best"] = b; }
    if (n_off) { double b = 1e300; for (double v : kse_off) b = std::min(b, v / n_off); sm["drop_mse_kf_best"] = b; }
    auto d = bf.diag_snapshot();
    sm["bf_nodes"] = d["nodes"]; sm["agree_with_epm"] = d["agree_with_epm"];
    em.line(sm);
    return 0;
}

// ---- S2: drifting mean (random walk) -----------------------------------------

int run_S2(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const double sd  = a.sigma / std::sqrt(double(a.dim));
    const double q   = a.q > 0.0 ? a.q : 1e-4;          // norm² of the drift per tick
    const double qsd = std::sqrt(q / double(a.dim));
    Eigen::VectorXf mu = unit_vec(rng, a.dim);
    em.line(header(a, json{{"q", q}, {"sd_per_dim", sd}, {"qsd_per_dim", qsd}}));
    SingleEpm s(a, em, a.dim);
    for (int t = 1; t <= a.ticks; ++t) {
        mu += gauss(rng, a.dim, qsd);
        Eigen::VectorXf x = mu + gauss(rng, a.dim, sd);
        s.step(uint64_t(t), x, json{{"mu", vec_json(mu)}});
    }
    return 0;
}

// ---- S4: deterministic cycle with teleports ----------------------------------

int run_S4(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const double sd = a.sigma / std::sqrt(double(a.dim));
    const int K = 3;
    std::vector<Eigen::VectorXf> mus;
    json mus_j = json::array();
    for (int k = 0; k < K; ++k) { mus.push_back(unit_vec(rng, a.dim)); mus_j.push_back(vec_json(mus.back())); }
    em.line(header(a, json{{"K", K}, {"mus", mus_j}, {"dwell", a.dwell}, {"p_switch", a.p_switch}, {"sd_per_dim", sd}}));
    SingleEpm s(a, em, a.dim);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    int state = 0, in_state = 0;
    for (int t = 1; t <= a.ticks; ++t) {
        int transition = 0;          // 0 none, 1 expected (cycle), 2 teleport
        if (in_state >= a.dwell) {
            in_state = 0;
            if (u(rng) < a.p_switch) {
                // A teleport goes somewhere the cycle would NOT go: neither the
                // current state nor the cycle-next one.  With K = 3 that is one
                // state; the draw keeps the code general for a larger K.
                int ns;
                do { ns = int(u(rng) * K) % K; } while (ns == state || ns == (state + 1) % K);
                state = ns; transition = 2;
            } else {
                state = (state + 1) % K; transition = 1;
            }
        }
        ++in_state;
        Eigen::VectorXf x = mus[state] + gauss(rng, a.dim, sd);
        s.step(uint64_t(t), x, json{{"k", state}, {"trans", transition}});
    }
    return 0;
}

// ---- S6: a ring of poses visited in order, slowly rotating -------------------
//
// The picrawler's body_pose EPM sees a gait: a closed ring of poses visited in
// sequence, never a static cluster.  Under the uncapped Kalman gain that stream
// produced node TURNOVER (insert, chase, prune, re-insert) and the walk fell,
// while S1–S5 showed nothing.  K poses sit on a circle in a random 2-D plane of
// the input space; the agent dwells `--dwell` ticks on each and moves to the
// next; the whole ring rotates by --omega rad per tick (0 = stationary).

int run_S6(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const int K = a.K >= 4 ? a.K : 8;
    const double sd = a.sigma / std::sqrt(double(a.dim));
    Eigen::VectorXf e1 = unit_vec(rng, a.dim);
    Eigen::VectorXf e2 = unit_vec(rng, a.dim);
    e2 -= e1 * e1.dot(e2); e2 /= std::max(e2.norm(), 1e-6f);          // Gram–Schmidt
    em.line(header(a, json{{"K", K}, {"dwell", a.dwell}, {"omega", a.omega}, {"sd_per_dim", sd},
                           {"e1", vec_json(e1)}, {"e2", vec_json(e2)}}));
    SingleEpm s(a, em, a.dim);
    int state = 0, in_state = 0;
    for (int t = 1; t <= a.ticks; ++t) {
        if (in_state >= a.dwell) { in_state = 0; state = (state + 1) % K; }
        ++in_state;
        const double phi = a.omega * t + 2.0 * M_PI * state / K;
        Eigen::VectorXf mu = float(std::cos(phi)) * e1 + float(std::sin(phi)) * e2;
        Eigen::VectorXf x = mu + gauss(rng, a.dim, sd);
        s.step(uint64_t(t), x, json{{"k", state}, {"phi", phi}});
    }
    return 0;
}

// ---- S3: damped-rotation target, closed predictor↔EPM pair ------------------
//
// Per axis the hidden state is a 2-vector rotated by theta and damped by rho
// each tick (a stable linear system, so the stream is stationary and the
// steady-state Kalman filter exists); the observation is its first component
// plus noise.  A constant-velocity target was tried first and rejected: its
// position is unbounded, so the predictor's SGD diverged as the context norm
// grew — a scenario defect, not a substrate result.

int run_S3(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const int obs_dim = 2, ctx_dim = 4;
    const double r_sd = a.sigma / std::sqrt(2.0);          // per-axis observation sd
    const double qsd  = std::sqrt(a.q > 0.0 ? a.q : 1e-3); // per-component process sd
    const double rho = a.rho, theta = a.theta;
    const double c = rho * std::cos(theta), sn = rho * std::sin(theta);
    Eigen::Vector4f st = Eigen::Vector4f::Zero();
    st(0) = 0.5f; st(2) = -0.5f;

    ogma::InProcessBus bus;
    ogma::EPM epm; epm.set_id("epm");
    ogma::DescendingPredictor pred; pred.set_id("pred");
    {
        auto pe = epm_params(a, "epm", "bench", "s", "obs.s", obs_dim, /*subtract=*/true);
        epm.on_setup(&bus, pe);
        ogma::ParamMap pp{
            {"consensus_topic",    std::string("ctx.s3")},
            {"targets",            std::vector<std::string>{"reality.bench.s"}},
            {"learning_rate",      0.01},
            {"init_noise_scale",   0.0},
            {"target_is_residual", true},
        };
        apply_sets(pp, a, "pred", false);
        pred.on_setup(&bus, pp);
    }
    em.line(header(a, json{{"obs_dim", obs_dim}, {"ctx_dim", ctx_dim}, {"r_sd_axis", r_sd}, {"q_sd", qsd},
                           {"rho", rho}, {"theta", theta},
                           {"A_axis", json::array({json::array({c, sn}), json::array({-sn, c})})}}));

    Eigen::VectorXf y_prev = Eigen::VectorXf::Zero(obs_dim);
    Eigen::VectorXf y      = Eigen::VectorXf::Zero(obs_dim);
    std::normal_distribution<double> nd(0.0, 1.0);
    for (int t = 1; t <= a.ticks; ++t) {
        for (int ax = 0; ax < 2; ++ax) {
            const double p = st(2 * ax), v = st(2 * ax + 1);
            st(2 * ax)     = float( c * p + sn * v + nd(rng) * qsd);
            st(2 * ax + 1) = float(-sn * p +  c * v + nd(rng) * qsd);
        }
        Eigen::VectorXf y_new(obs_dim);
        y_new(0) = st(0) + float(nd(rng) * r_sd);
        y_new(1) = st(2) + float(nd(rng) * r_sd);
        y_prev = y; y = y_new;
        Eigen::VectorXf ctx(ctx_dim);
        ctx << y(0), y(1), y_prev(0), y_prev(1);     // [y_t, y_{t-1}] predicts y_{t+1}

        bus.begin_tick(uint64_t(t));
        bus.publish("ctx.s3", make_obs(ctx, uint64_t(t)));
        bus.publish("obs.s",  make_obs(y,   uint64_t(t)));
        pred.tick(uint64_t(t));
        epm.tick(uint64_t(t));
        bus.end_tick();

        auto tok = last_token(bus, "reality.bench.s");
        auto pt  = std::dynamic_pointer_cast<const ogma::PredictionToken>(bus.last_value("prediction.bench.s"));
        json line{
            {"t", t}, {"y", vec_json(y)}, {"truth", json::array({st(0), st(1), st(2), st(3)})},
            {"tok", token_json(tok)},
            {"resid", tok ? vec_json(tok->latent) : json(nullptr)},
            {"pred",  pt ? vec_json(pt->predicted_latent) : json(nullptr)},
            {"pred_conf", pt ? pt->confidence : 0.0f},
            {"dp_err", pred.target_err_ema(0)}, {"dp_norm", pred.target_norm_ema(0)},
        };
        em.line(line);
        if ((a.dump_every > 0 && t % a.dump_every == 0) || (tok && tok->just_baked))
            em.line(json{{"event", "nodes"}, {"t", t}, {"nodes", nodes_json(epm)}});
    }
    return 0;
}

// ---- S5: two sensors fused by the voter --------------------------------------

int run_S5(Args const& a, Emitter& em) {
    std::mt19937_64 rng(a.seed);
    const double sd_a = a.sigma / std::sqrt(double(a.dim));
    const double sd_b = sd_a * a.sigma_b_ratio;
    const double qsd  = a.q > 0.0 ? std::sqrt(a.q / double(a.dim)) : 0.0;
    Eigen::VectorXf s = unit_vec(rng, a.dim);

    ogma::InProcessBus bus;
    ogma::EPM epm_a; epm_a.set_id("epm_a");
    ogma::EPM epm_b; epm_b.set_id("epm_b");
    ogma::LateralVoter voter; voter.set_id("voter");
    {
        auto pa = epm_params(a, "epm_a", "sensor", "a", "obs.a", a.dim, false);
        auto pb = epm_params(a, "epm_b", "sensor", "b", "obs.b", a.dim, false);
        if (a.subrate > 1) pb["process_every_n_ticks"] = ogma::ParamValue{int64_t{a.subrate}};
        epm_a.on_setup(&bus, pa);
        epm_b.on_setup(&bus, pb);
        ogma::ParamMap pv{{"level", int64_t{0}}, {"input_pattern", std::string("reality.")}};
        apply_sets(pv, a, "voter", false);
        voter.on_setup(&bus, pv);
    }
    em.line(header(a, json{{"sd_a_per_dim", sd_a}, {"sd_b_per_dim", sd_b}, {"sigma_b_ratio", a.sigma_b_ratio},
                           {"q", a.q}, {"dead_at", a.dead_at}, {"placeholder_at", a.placeholder_at},
                           {"subrate", a.subrate}}));

    Eigen::VectorXf y_b_frozen;
    for (int t = 1; t <= a.ticks; ++t) {
        if (qsd > 0.0) s += gauss(rng, a.dim, qsd);
        Eigen::VectorXf y_a = s + gauss(rng, a.dim, sd_a);
        Eigen::VectorXf y_b = s + gauss(rng, a.dim, sd_b);
        if (a.dead_at >= 0 && t >= a.dead_at) {
            if (y_b_frozen.size() == 0) y_b_frozen = y_b;
            y_b = y_b_frozen;                       // a dead sensor: constant output
        }
        bool publish_b = !(a.placeholder_at >= 0 && t == a.placeholder_at);

        bus.begin_tick(uint64_t(t));
        bus.publish("obs.a", make_obs(y_a, uint64_t(t)));
        if (publish_b) bus.publish("obs.b", make_obs(y_b, uint64_t(t)));
        epm_a.tick(uint64_t(t));
        epm_b.tick(uint64_t(t));
        voter.tick(uint64_t(t));
        bus.end_tick();

        auto ta = last_token(bus, "reality.sensor.a");
        auto tb = last_token(bus, "reality.sensor.b");
        auto ct = std::dynamic_pointer_cast<const ogma::ConsensusToken>(bus.last_value("consensus.0"));
        json trust = json::object();
        if (ct) for (auto const& [k, v] : ct->trust_weights) trust[k] = v;
        json line{
            {"t", t}, {"s", vec_json(s)}, {"y_a", vec_json(y_a)}, {"y_b", vec_json(y_b)},
            {"tok_a", token_json(ta)}, {"tok_b", token_json(tb)},
            {"trust", trust},
            {"fused_tle", ct ? ct->fused_tle : 0.0f},
            {"fused", ct ? vec_json(ct->fused_embedding) : json(nullptr)},
            {"active", ct ? ct->active_modality : ""},
        };
        em.line(line);
        if (a.dump_every > 0 && t % a.dump_every == 0) {
            em.line(json{{"event", "nodes"}, {"t", t}, {"epm", "a"}, {"nodes", nodes_json(epm_a)}});
            em.line(json{{"event", "nodes"}, {"t", t}, {"epm", "b"}, {"nodes", nodes_json(epm_b)}});
        }
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Args a = parse_args(argc, argv);
    Emitter em(a.out);
    try {
        if (a.scenario == "S1")  return run_S1(a, em);
        if (a.scenario == "S1m") return run_S1m(a, em);
        if (a.scenario == "S2")  return run_S2(a, em);
        if (a.scenario == "S3")  return run_S3(a, em);
        if (a.scenario == "S4")  return run_S4(a, em);
        if (a.scenario == "S5")  return run_S5(a, em);
        if (a.scenario == "S6")  return run_S6(a, em);
        if (a.scenario == "S9")  return run_S9(a, em);
        if (a.scenario == "S10") return run_S10(a, em);
    } catch (std::exception const& e) {
        die(std::string("exception: ") + e.what());
    }
    die("unknown scenario " + a.scenario);
}
