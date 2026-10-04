// =============================================================================
// brain_replay.cpp  --  the replay benchmark: the brains alone, driven by a host's tape (2026-10-04, design doc §17.111)
// =============================================================================
//
// ogma_brain_replay TAPE [--set B:module.key=JSON]... [--only B] [--repeat N]
//
// Rebuilds each brain the tape opens (0 intent, 1 head, 2 stop) from the exact config the host built it from, replays every
// host call in order (tokens, reset events, param changes, restores, ticks), and times each tick in thread CPU time.  After
// each tick it compares the brain's action outputs with the live run's: a replay that reproduces them is the same brain on
// the same inputs, so its timings are the live run's cost; a --set variant is expected to diverge in its actions and is
// still driven by the same inputs (open loop) -- the controlled cost A/B a whole-run profile cannot give.
//
// No MuJoCo, no ONNX: it builds wherever ogma_core builds.  Memory: the brains plus one tape record at a time (streaming;
// the largest record is a checkpoint restore of a few MB) -- the tape itself is never loaded.
//
//   --set B:module.key=JSON   override one param of brain B's config before it is built (repeatable), e.g.
//                             --set 0:cloud.cast_once=2
//   --only B                  replay only brain B (the others' records are skipped)
//   --repeat N                replay the tape N times (more samples; each pass rebuilds the brains)
// Per brain it prints ticks, thread-CPU us per tick (mean, p50, p95, p99, max), the share of one core at 50 Hz, the action
// fidelity, and at the end the process's peak resident memory and heap in use.  OGMA_PROFILE=1 adds the per-module table.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <malloc.h>
#include <nlohmann/json.hpp>

#include "BrainTape.hpp"
#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/BearingSeekLoop.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

using namespace mjhost;

namespace {

double cpu_us() {
    timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return double(ts.tv_sec) * 1e6 + double(ts.tv_nsec) * 1e-3;
}

std::string proc_status(const char* key) {
    std::ifstream f("/proc/self/status"); std::string line;
    while (std::getline(f, line)) if (line.rfind(key, 0) == 0) return line.substr(std::string(key).size());
    return "?";
}

struct Override { int brain; std::string module, key, json; };

struct Brain {
    std::unique_ptr<ogma::OgmaInstance> inst;
    std::vector<float> cpu;                       // thread-CPU us per tick
    uint64_t compared = 0, mismatched = 0, digest_diff = 0, first_diff_tick = 0;
    double max_diff = 0.0;
    size_t heap_built_kb = 0;
};

void report(const char* name, Brain& b) {
    if (b.cpu.empty()) return;
    std::vector<float> w = b.cpu; std::sort(w.begin(), w.end());
    double sum = 0.0; for (float v : w) sum += v;
    const auto q = [&](double f) { return double(w[std::min(w.size() - 1, size_t(f * double(w.size())))]); };
    const double mean = sum / double(w.size());
    std::printf("%-6s %7zu ticks  CPU us/tick: mean %8.1f  p50 %8.1f  p95 %8.1f  p99 %8.1f  max %8.1f  |  %5.2f %% of a core at 50 Hz"
                "  |  over 20 ms: %zu\n",
                name, w.size(), mean, q(0.5), q(0.95), q(0.99), double(w.back()), 100.0 * mean / 20000.0,
                size_t(std::count_if(w.begin(), w.end(), [](float v) { return v > 20000.0f; })));
    std::printf("       fidelity: %llu ticks compared; actions differ on %llu (max |diff| %.3g); outputs differ on %llu%s%s\n",
                (unsigned long long)b.compared, (unsigned long long)b.mismatched, b.max_diff, (unsigned long long)b.digest_diff,
                b.digest_diff ? (" (first at tick " + std::to_string(b.first_diff_tick) + ")").c_str() : "",
                b.compared && !b.mismatched && !b.digest_diff ? "  -- REPRODUCES the live run" : "");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: ogma_brain_replay TAPE [--set B:module.key=JSON]... [--only B] [--repeat N]\n"); return 2; }
    const std::string tape = argv[1];
    std::vector<Override> overrides;
    int only = -1, repeat = 1;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--set" && i + 1 < argc) {
            const std::string s = argv[++i];
            const auto colon = s.find(':'), dot = s.find('.', colon + 1), eq = s.find('=', dot + 1);
            if (colon == std::string::npos || dot == std::string::npos || eq == std::string::npos) { std::fprintf(stderr, "bad --set %s\n", s.c_str()); return 2; }
            overrides.push_back({std::stoi(s.substr(0, colon)), s.substr(colon + 1, dot - colon - 1), s.substr(dot + 1, eq - dot - 1), s.substr(eq + 1)});
        } else if (a == "--only" && i + 1 < argc) only = std::stoi(argv[++i]);
        else if (a == "--repeat" && i + 1 < argc) repeat = std::max(1, std::stoi(argv[++i]));
        else { std::fprintf(stderr, "unknown argument %s\n", a.c_str()); return 2; }
    }
    static const char* const kNames[3] = {"intent", "head", "stop"};
    std::map<int, std::vector<float>> all_cpu;
    std::map<int, Brain> last;
    for (int pass = 0; pass < repeat; ++pass) {
        BrainTapeReader rd(tape);
        if (!rd.ok()) { std::fprintf(stderr, "cannot read tape %s\n", tape.c_str()); return 1; }
        std::map<int, Brain> brains;
        TapeRecord r;
        while (rd.next(r)) {
            if (only >= 0 && r.brain != only) continue;
            Brain& b = brains[r.brain];
            switch (r.kind) {
                case TapeKind::Config: {
                    auto cfg = ogma::GraphConfig::load_from_json(r.json);
                    for (auto const& o : overrides) {
                        if (o.brain != r.brain) continue;
                        bool hit = false;
                        for (auto& m : cfg.modules)
                            if (m.id == o.module) { m.params[o.key] = ogma::GraphConfig::param_from_json(nlohmann::json::parse(o.json)); hit = true; }
                        if (!hit) std::fprintf(stderr, "--set: no module %s in brain %d\n", o.module.c_str(), o.brain);
                        else if (pass == 0) std::printf("override: brain %d %s.%s = %s\n", o.brain, o.module.c_str(), o.key.c_str(), o.json.c_str());
                    }
                    b.inst = std::make_unique<ogma::OgmaInstance>(std::move(cfg), std::make_unique<ogma::InProcessBus>());
                    b.heap_built_kb = mallinfo2().uordblks / 1024;
                    break;
                }
                case TapeKind::Restore:
                    if (b.inst) b.inst->restore_state(nlohmann::json::parse(r.json));
                    break;
                case TapeKind::Token: {
                    if (!b.inst) break;
                    auto p = std::make_shared<ogma::ProprioToken>();
                    p->tick_id = r.tick; p->producer_id = r.c; p->sensor = r.b;
                    p->values = Eigen::VectorXf::Map(r.values.data(), long(r.values.size()));
                    b.inst->bus()->publish(r.a, p);
                    break;
                }
                case TapeKind::Event: {
                    if (!b.inst) break;
                    auto ev = std::make_shared<ogma::EnvEvent>();
                    ev->tick_id = r.tick; ev->producer_id = r.c; ev->name = r.b; ev->intensity = r.values.empty() ? 1.0f : r.values[0];
                    b.inst->bus()->publish(r.a, ev);
                    break;
                }
                case TapeKind::Param:
                    if (b.inst)
                        if (auto* m = b.inst->module(r.a)) m->on_param_change(r.b, ogma::GraphConfig::param_from_json(nlohmann::json::parse(r.json)));
                    break;
                case TapeKind::Call: {
                    if (!b.inst) break;
                    auto* m = b.inst->module(r.a);
                    if (r.b == "forget_target") { if (auto* q = dynamic_cast<ogma::BearingSeekLoop*>(m)) q->forget_target(); }
                    else if (r.b == "rebabble") { if (auto* w = dynamic_cast<ogma::MotorEPMv2*>(m)) w->rebabble(int(r.arg)); }
                    else std::fprintf(stderr, "tape: unknown call %s.%s\n", r.a.c_str(), r.b.c_str());
                    break;
                }
                case TapeKind::Tick: {
                    if (!b.inst) break;
                    const double t0 = cpu_us();
                    b.inst->tick();
                    b.cpu.push_back(float(cpu_us() - t0));
                    break;
                }
                case TapeKind::Actions: {
                    if (!b.inst) break;
                    bool differ = false;
                    for (auto const& [topic, v] : r.actions) {
                        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(b.inst->bus()->last_value(topic));
                        const double d = a ? std::fabs(double(a->accel) - double(v)) : 1e9;
                        b.max_diff = std::max(b.max_diff, d);
                        if (d > 0.0) differ = true;
                    }
                    ++b.compared; b.mismatched += differ;
                    if (brain_digest(*b.inst) != r.digest) { if (!b.digest_diff) b.first_diff_tick = b.compared; ++b.digest_diff; }
                    break;
                }
            }
        }
        if (pass == 0) std::printf("tape %s: %llu records\n", tape.c_str(), (unsigned long long)rd.records());
        for (auto& [id, b] : brains) {
            all_cpu[id].insert(all_cpu[id].end(), b.cpu.begin(), b.cpu.end());
            last[id].compared += b.compared; last[id].mismatched += b.mismatched; last[id].max_diff = std::max(last[id].max_diff, b.max_diff);
            last[id].digest_diff += b.digest_diff; if (!last[id].first_diff_tick) last[id].first_diff_tick = b.first_diff_tick;
            if (pass == 0) std::printf("%-6s heap in use after it was built: %zu kB (the process's)\n", id >= 0 && id < 3 ? kNames[id] : "?", b.heap_built_kb);
        }
        // the brains are destroyed here (OGMA_PROFILE prints their module tables)
    }
    for (auto& [id, b] : last) { b.cpu = std::move(all_cpu[id]); report(id >= 0 && id < 3 ? kNames[id] : "?", b); }
    std::printf("process: peak resident%s, resident now%s, heap in use %zu kB\n", proc_status("VmHWM:").c_str(),
                proc_status("VmRSS:").c_str(), mallinfo2().uordblks / 1024);
    return 0;
}
