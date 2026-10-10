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
//
// THE ARM MEASUREMENTS (2026-10-08, mj_host/tools/arm_bench/): the Pi 5's A76 is not Pollen's A55, so its microseconds do
// not transfer -- but the work does.  The same binary built for the A55 retires the same instructions on both cores; what
// differs is how fast, and how often the data falls out of cache.  So:
//   --counters        per-tick hardware counts around tick() only (perf_event_open, user space, this thread): instructions,
//                     cycles, and on aarch64 L1D/L2D/L3D refills, backend stalls and branch misses.  The cost of reading
//                     them is measured on an empty window first and subtracted.  Needs perf_event_paranoid <= 2.
//   --json FILE       the summary, machine-readable (quantiles of every per-tick series, memory, fidelity)
//   --ticks-csv FILE  one row per tick: pass, brain, tick, us, then each counter -- for a projection that needs the
//                     instructions and the misses of the SAME tick (its worst case)
//   --dump FILE --dump-ticks N   every module's every output for the first N ticks of each brain, floats in hex (%a):
//                     diff two machines' dumps to name the first module whose numbers differ (a port is the same
//                     numbers, not the same maths)
// None given: the output is what it was.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <linux/perf_event.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
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

// One perf_event group on this thread, user space only, read in one syscall.  Events the core does not have are skipped
// (named in the header line); a group the kernel had to multiplex is counted, because its numbers are then scaled guesses.
struct Counters {
    struct Ev { const char* name; uint32_t type; uint64_t config; };
    std::vector<std::string> names;
    std::vector<int> fds;
    std::vector<uint64_t> base;                   // the cost of a read-to-read window with nothing in it (median)
    uint64_t multiplexed = 0;
    std::string why;

    bool open() {
        std::vector<Ev> evs = {{"instructions", PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS},
                               {"cycles", PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES}};
#if defined(__aarch64__)
        // ARMv8 PMU common events (Arm ARM D7.10): the same numbers on the A76 and the A55
        evs.push_back({"l1d_refill", PERF_TYPE_RAW, 0x03});
        evs.push_back({"l2d_refill", PERF_TYPE_RAW, 0x17});
        evs.push_back({"l3d_refill", PERF_TYPE_RAW, 0x2A});
        evs.push_back({"stall_backend", PERF_TYPE_RAW, 0x24});
        evs.push_back({"br_mis_pred", PERF_TYPE_RAW, 0x10});
#endif
        for (auto const& e : evs) {
            perf_event_attr a; std::memset(&a, 0, sizeof a);
            a.size = sizeof a; a.type = e.type; a.config = e.config;
            a.exclude_kernel = 1; a.exclude_hv = 1;
            a.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_TOTAL_TIME_ENABLED | PERF_FORMAT_TOTAL_TIME_RUNNING;
            a.disabled = fds.empty() ? 1 : 0;
            const int fd = int(syscall(SYS_perf_event_open, &a, 0, -1, fds.empty() ? -1 : fds[0], 0));
            if (fd < 0) {
                if (fds.empty()) { why = std::string("perf_event_open: ") + std::strerror(errno) + " (perf_event_paranoid <= 2 needed)"; return false; }
                std::fprintf(stderr, "counters: %s unavailable here (%s), skipped\n", e.name, std::strerror(errno));
                continue;
            }
            fds.push_back(fd); names.push_back(e.name);
        }
        ioctl(fds[0], PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);
        // the empty window: what a read before and a read after cost when no tick runs between them
        std::vector<std::vector<uint64_t>> d(names.size());
        std::vector<uint64_t> a(names.size()), b(names.size());
        for (int i = 0; i < 2001; ++i) { read(a); read(b); for (size_t k = 0; k < a.size(); ++k) d[k].push_back(b[k] - a[k]); }
        multiplexed = 0;
        for (auto& v : d) { std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end()); base.push_back(v[v.size() / 2]); }
        return true;
    }
    // raw cumulative values in `out`; counts a sample the group did not run for all of
    void read(std::vector<uint64_t>& out) {
        uint64_t buf[3 + 16];
        if (::read(fds[0], buf, sizeof buf) < 0) return;
        if (buf[2] < buf[1]) ++multiplexed;
        for (size_t k = 0; k < out.size() && k < buf[0]; ++k) out[k] = buf[3 + k];
    }
};

struct Override { int brain; std::string module, key, json; };

struct Brain {
    std::unique_ptr<ogma::OgmaInstance> inst;
    std::vector<float> cpu;                       // thread-CPU us per tick
    std::vector<std::vector<float>> ctr;          // --counters: per counter, its count per tick (empty window subtracted)
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

struct Quant { double mean, p50, p95, p99, p999, max, sum; };

Quant quantiles(std::vector<float> w) {
    Quant r{0, 0, 0, 0, 0, 0, 0};
    if (w.empty()) return r;
    std::sort(w.begin(), w.end());
    for (float v : w) r.sum += v;
    const auto q = [&](double f) { return double(w[std::min(w.size() - 1, size_t(f * double(w.size())))]); };
    r.mean = r.sum / double(w.size()); r.p50 = q(0.5); r.p95 = q(0.95); r.p99 = q(0.99); r.p999 = q(0.999); r.max = w.back();
    return r;
}

nlohmann::json quant_json(const Quant& q) {
    return {{"mean", q.mean}, {"p50", q.p50}, {"p95", q.p95}, {"p99", q.p99}, {"p999", q.p999}, {"max", q.max}, {"sum", q.sum}};
}

void report_counters(const Brain& b, const std::vector<std::string>& names) {
    if (b.ctr.empty() || b.ctr[0].empty()) return;
    std::map<std::string, Quant> q;
    for (size_t k = 0; k < names.size(); ++k) q[names[k]] = quantiles(b.ctr[k]);
    const auto has = [&](const char* n) { return q.count(n) != 0; };
    const double ins = q["instructions"].sum;
    std::printf("       counters: instructions/tick mean %.0f p99 %.0f max %.0f", q["instructions"].mean, q["instructions"].p99,
                q["instructions"].max);
    if (has("cycles"))
        std::printf("  |  cycles/tick mean %.0f p99 %.0f max %.0f  |  IPC %.2f", q["cycles"].mean, q["cycles"].p99, q["cycles"].max,
                    q["cycles"].sum > 0 ? ins / q["cycles"].sum : 0.0);
    std::printf("\n");
    if (ins > 0 && (has("l1d_refill") || has("l2d_refill") || has("l3d_refill"))) {
        std::printf("                 per 1000 instructions:");
        for (const char* n : {"l1d_refill", "l2d_refill", "l3d_refill", "br_mis_pred"})
            if (has(n)) std::printf("  %s %.2f", n, 1000.0 * q[n].sum / ins);
        if (has("stall_backend") && has("cycles") && q["cycles"].sum > 0)
            std::printf("  |  backend-stalled %.0f %% of cycles", 100.0 * q["stall_backend"].sum / q["cycles"].sum);
        std::printf("  |  L2 refills/tick mean %.0f p99 %.0f\n", has("l2d_refill") ? q["l2d_refill"].mean : 0.0,
                    has("l2d_refill") ? q["l2d_refill"].p99 : 0.0);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: ogma_brain_replay TAPE [--set B:module.key=JSON]... [--only B] [--repeat N] [--counters]"
                             " [--json FILE] [--ticks-csv FILE]\n");
        return 2;
    }
    const std::string tape = argv[1];
    std::vector<Override> overrides;
    int only = -1, repeat = 1;
    bool want_counters = false;
    std::string json_out, csv_out, dump_out;
    uint64_t dump_ticks = 0;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--set" && i + 1 < argc) {
            const std::string s = argv[++i];
            const auto colon = s.find(':'), dot = s.find('.', colon + 1), eq = s.find('=', dot + 1);
            if (colon == std::string::npos || dot == std::string::npos || eq == std::string::npos) { std::fprintf(stderr, "bad --set %s\n", s.c_str()); return 2; }
            overrides.push_back({std::stoi(s.substr(0, colon)), s.substr(colon + 1, dot - colon - 1), s.substr(dot + 1, eq - dot - 1), s.substr(eq + 1)});
        } else if (a == "--only" && i + 1 < argc) only = std::stoi(argv[++i]);
        else if (a == "--repeat" && i + 1 < argc) repeat = std::max(1, std::stoi(argv[++i]));
        else if (a == "--counters") want_counters = true;
        else if (a == "--json" && i + 1 < argc) json_out = argv[++i];
        else if (a == "--ticks-csv" && i + 1 < argc) csv_out = argv[++i];
        else if (a == "--dump" && i + 1 < argc) dump_out = argv[++i];
        else if (a == "--dump-ticks" && i + 1 < argc) dump_ticks = std::stoull(argv[++i]);
        else { std::fprintf(stderr, "unknown argument %s\n", a.c_str()); return 2; }
    }
    static const char* const kNames[3] = {"intent", "head", "stop"};
    Counters ctr;
    const bool counting = want_counters && ctr.open();
    if (want_counters) {
        if (!counting) std::printf("counters: unavailable -- %s\n", ctr.why.c_str());
        else {
            std::printf("counters:");
            for (size_t k = 0; k < ctr.names.size(); ++k) std::printf(" %s (empty window %llu)", ctr.names[k].c_str(), (unsigned long long)ctr.base[k]);
            std::printf("\n");
        }
    }
    const size_t nctr = counting ? ctr.names.size() : 0;
    std::vector<uint64_t> c0(nctr), c1(nctr);
    std::FILE* csv = csv_out.empty() ? nullptr : std::fopen(csv_out.c_str(), "w");
    if (!csv_out.empty() && !csv) { std::fprintf(stderr, "cannot write %s\n", csv_out.c_str()); return 1; }
    if (csv) {
        std::fprintf(csv, "pass,brain,tick,us");
        for (size_t k = 0; k < nctr; ++k) std::fprintf(csv, ",%s", ctr.names[k].c_str());
        std::fprintf(csv, "\n");
    }
    std::FILE* dump = dump_out.empty() ? nullptr : std::fopen(dump_out.c_str(), "w");
    std::map<int, std::vector<float>> all_cpu;
    std::map<int, std::vector<std::vector<float>>> all_ctr;
    std::map<int, Brain> last;
    size_t heap_end_kb = 0, heap_end_mmap_kb = 0;      // pass 0, the brains still alive at the end of the tape
    std::vector<size_t> heap_trace_kb;                 // pass 0, arena + mmapped, every 50 intent ticks (1 s): bounded or growing?
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
                    if (counting) ctr.read(c0);
                    const double t0 = cpu_us();
                    b.inst->tick();
                    const double us = cpu_us() - t0;
                    if (counting) ctr.read(c1);
                    b.cpu.push_back(float(us));
                    if (pass == 0 && r.brain == 0 && b.cpu.size() % 50 == 0) {
                        const struct mallinfo2 mt = mallinfo2();
                        heap_trace_kb.push_back((mt.uordblks + mt.hblkhd) / 1024);
                    }
                    if (counting) {
                        if (b.ctr.empty()) b.ctr.resize(nctr);
                        for (size_t k = 0; k < nctr; ++k) {
                            const uint64_t d = c1[k] - c0[k];
                            b.ctr[k].push_back(float(d > ctr.base[k] ? d - ctr.base[k] : 0));
                        }
                    }
                    if (dump && pass == 0 && b.cpu.size() <= dump_ticks) {
                        for (auto* m : b.inst->modules())
                            for (auto const& spec : m->output_topics()) {
                                auto msg = b.inst->bus()->last_value(spec.name);
                                if (!msg) continue;
                                std::fprintf(dump, "%d %zu %s %s", r.brain, b.cpu.size(), std::string(m->id()).c_str(), spec.name.c_str());
                                if (auto p = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg))
                                    for (long k = 0; k < p->values.size(); ++k) std::fprintf(dump, " %a", double(p->values[k]));
                                else if (auto o = std::dynamic_pointer_cast<const ogma::ActionOut>(msg)) std::fprintf(dump, " %a", double(o->accel));
                                else if (auto t = std::dynamic_pointer_cast<const ogma::RealityToken>(msg))
                                    std::fprintf(dump, " w%d %a", int(t->winner_id), double(t->tle));
                                std::fprintf(dump, "\n");
                            }
                    }
                    if (csv) {
                        std::fprintf(csv, "%d,%d,%zu,%.2f", pass, r.brain, b.cpu.size(), us);
                        for (size_t k = 0; k < nctr; ++k) std::fprintf(csv, ",%.0f", double(b.ctr[k].back()));
                        std::fprintf(csv, "\n");
                    }
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
        if (pass == 0) {
            const struct mallinfo2 me = mallinfo2();
            heap_end_kb = me.uordblks / 1024; heap_end_mmap_kb = me.hblkhd / 1024;
            std::printf("tape %s: %llu records\n", tape.c_str(), (unsigned long long)rd.records());
            std::printf("heap with every brain alive at the end of the tape: %zu kB (+ %zu kB in mmapped blocks)\n", heap_end_kb,
                        heap_end_mmap_kb);
        }
        for (auto& [id, b] : brains) {
            all_cpu[id].insert(all_cpu[id].end(), b.cpu.begin(), b.cpu.end());
            if (!b.ctr.empty()) {
                auto& ac = all_ctr[id]; ac.resize(nctr);
                for (size_t k = 0; k < nctr; ++k) ac[k].insert(ac[k].end(), b.ctr[k].begin(), b.ctr[k].end());
            }
            last[id].heap_built_kb = b.heap_built_kb;
            last[id].compared += b.compared; last[id].mismatched += b.mismatched; last[id].max_diff = std::max(last[id].max_diff, b.max_diff);
            last[id].digest_diff += b.digest_diff; if (!last[id].first_diff_tick) last[id].first_diff_tick = b.first_diff_tick;
            if (pass == 0) std::printf("%-6s heap in use after it was built: %zu kB (the process's)\n", id >= 0 && id < 3 ? kNames[id] : "?", b.heap_built_kb);
        }
        // the brains are destroyed here (OGMA_PROFILE prints their module tables)
    }
    if (csv) std::fclose(csv);
    if (dump) std::fclose(dump);
    nlohmann::json js = {{"tape", tape}, {"repeat", repeat}, {"brains", nlohmann::json::object()}};
    for (auto& [id, b] : last) {
        b.cpu = std::move(all_cpu[id]); b.ctr = std::move(all_ctr[id]);
        const char* name = id >= 0 && id < 3 ? kNames[id] : "?";
        report(name, b);
        report_counters(b, ctr.names);
        nlohmann::json bj = {{"ticks", b.cpu.size()}, {"cpu_us", quant_json(quantiles(b.cpu))},
                             {"over_20ms", size_t(std::count_if(b.cpu.begin(), b.cpu.end(), [](float v) { return v > 20000.0f; }))},
                             {"heap_built_kb", b.heap_built_kb},
                             {"fidelity", {{"compared", b.compared}, {"actions_differ", b.mismatched}, {"max_diff", b.max_diff},
                                           {"outputs_differ", b.digest_diff}, {"first_diff_tick", b.first_diff_tick}}},
                             {"counters", nlohmann::json::object()}};
        for (size_t k = 0; k < b.ctr.size(); ++k) bj["counters"][ctr.names[k]] = quant_json(quantiles(b.ctr[k]));
        js["brains"][name] = bj;
    }
    if (counting) {
        if (ctr.multiplexed) std::printf("counters: WARNING the kernel multiplexed the group on %llu reads -- counts are scaled guesses\n",
                                         (unsigned long long)ctr.multiplexed);
        js["counters"] = {{"names", ctr.names}, {"empty_window", ctr.base}, {"multiplexed_reads", ctr.multiplexed}};
    } else if (want_counters) js["counters"] = {{"unavailable", ctr.why}};
    const auto kb = [](const std::string& s) { return std::atol(s.c_str()); };
    // uordblks is the arena only: glibc serves a block of >= 128 kB (M_MMAP_THRESHOLD) with its own mmap, counted in hblkhd
    const struct mallinfo2 mi = mallinfo2();
    std::printf("process: peak resident%s, resident now%s, heap in use %zu kB (+ %zu kB in mmapped blocks)\n",
                proc_status("VmHWM:").c_str(), proc_status("VmRSS:").c_str(), mi.uordblks / 1024, mi.hblkhd / 1024);
    js["process"] = {{"peak_rss_kb", kb(proc_status("VmHWM:"))}, {"rss_kb", kb(proc_status("VmRSS:"))},
                     {"peak_vm_kb", kb(proc_status("VmPeak:"))}, {"heap_in_use_kb", mi.uordblks / 1024},
                     {"heap_mmapped_kb", mi.hblkhd / 1024}, {"heap_end_of_tape_kb", heap_end_kb},
                     {"heap_end_of_tape_mmapped_kb", heap_end_mmap_kb}, {"heap_kb_each_second", heap_trace_kb},
                     {"page_size", sysconf(_SC_PAGESIZE)}};
    if (!json_out.empty()) {
        std::ofstream f(json_out);
        if (!f) { std::fprintf(stderr, "cannot write %s\n", json_out.c_str()); return 1; }
        f << js.dump(1) << "\n";
    }
    return 0;
}
