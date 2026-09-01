// ogma_host — the brain, on the robot.
//
// The third of the three programs in the port doc's SPEC §1: `ogma_host` owns the
// real-time loop on the Pi, `ogma_benchd` owns the servos and serves the bench, and
// Godot stays on the laptop.  Godot never runs here — on hardware it would be a
// rendering server and a physics server simulating a body that physically exists.
//
// What this binary is: a host, in exactly the sense OgmaInstance.hpp already names
// ("Godot Host, HAL Host, Debug Host").  It owns a Bus, builds one OgmaInstance from
// a GraphConfig, publishes real sensor frames onto reality.* topics, ticks the graph
// at a fixed rate, and exposes the inspector surfaces (control on OGMA_INSPECTOR_PORT,
// diag on port+1) that xaq_inspector and xaq_voice already speak.
//
// ⚠ SPEC §1.1 — THE BOUNDARY IS STRUCTURAL.  This program does not drive servos and
// has no path to.  Actuation is benchd's, over its own control verb set, and wiring it
// is a separate deliberate step; a sensory host that cannot move the robot is also the
// only kind that is safe to leave running unattended.
#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/DiagPublisher.hpp"
#include "control_server.hpp"
#include <zmq.h>
#include "ogma/Topics.hpp"
#include "ogma/hw/ResourceMonitor.hpp"
#include "ogma/hw/AudioCapture.hpp"
#include "ogma/hw/CameraCapture.hpp"
#include "ogma/hw/Ultrasonic.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <sched.h>
#include <time.h>

namespace {

std::atomic<bool> g_run{true};
void on_sig(int) { g_run = false; }

struct Args {
    std::string config;
    double  hz         = 50.0;
    long    max_ticks  = 0;       // 0 = forever
    bool    realtime   = false;   // SCHED_FIFO
    bool    quiet      = false;
    // Sensors are opt-in, one at a time -- the same discipline the BOM applies to
    // bring-up.  A host that silently enables everything makes a failure ambiguous.
    // Loopback by default: the diag stream is unauthenticated, so putting the brain
    // on the network is a deliberate act.  --listen 0.0.0.0 is how the laptop's
    // xaq_inspector attaches.  (ControlServer already binds INADDR_ANY regardless.)
    std::string listen = "127.0.0.1";
    bool    video      = false;   // opt-in: it is a viewer, not part of the loop
    bool    mic        = false;
    bool    camera     = false;
    bool    range      = false;
};

void usage() {
    std::fprintf(stderr,
        "usage: ogma_host --config <graph.json> [--hz 50] [--ticks N] [--rt] [--quiet]\n"
        "                 [--mic] [--camera] [--range] [--listen 0.0.0.0] [--video]\n"
        "  sensors are opt-in, one at a time: an unattributable failure is worse than a slow bring-up\n"
        "  topics: sense.audio (RawAudioFrame) sense.camera (RawImageFrame) sense.range (ProprioToken)\n"
        "  inspector: control = $OGMA_INSPECTOR_PORT (default 7400), diag = port+1\n"
        "  NO ACTUATION: this binary has no servo path (SPEC 1.1)\n");
}

// SCHED_FIFO is a request, not a requirement: without CAP_SYS_NICE it fails and the
// loop still runs, just at the mercy of CFS.  Say which one happened -- a host that
// silently lost its real-time priority is the kind of thing that shows up later as
// unexplained jitter.
bool try_realtime() {
    sched_param p{};
    p.sched_priority = 40;                      // below the kernel's own threads
    return ::sched_setscheduler(0, SCHED_FIFO, &p) == 0;
}

} // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string v = argv[i];
        if (v == "--config" && i + 1 < argc)      a.config = argv[++i];
        else if (v == "--hz" && i + 1 < argc)     a.hz = std::atof(argv[++i]);
        else if (v == "--ticks" && i + 1 < argc)  a.max_ticks = std::atol(argv[++i]);
        else if (v == "--rt")                     a.realtime = true;
        else if (v == "--quiet")                  a.quiet = true;
        else if (v == "--listen" && i + 1 < argc)  a.listen = argv[++i];
        else if (v == "--video")                  a.video = true;
        else if (v == "--mic")                    a.mic = true;
        else if (v == "--camera")                 a.camera = true;
        else if (v == "--range")                  a.range = true;
        else { usage(); return 2; }
    }
    if (a.config.empty() || a.hz <= 0.0) { usage(); return 2; }

    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    try {
        auto cfg = ogma::GraphConfig::load_from_file(a.config);
        auto instance = std::make_unique<ogma::OgmaInstance>(
            std::move(cfg), std::make_unique<ogma::InProcessBus>());

        // Sensors start BEFORE the loop so a failure is a startup error the operator
        // sees, not a channel that is quietly dead for the whole run.
        ogma::hw::AudioCapture  mic{ogma::hw::AudioCapture::Config{}};
        ogma::hw::CameraCapture cam{ogma::hw::CameraCapture::Config{}};
        ogma::hw::Ultrasonic    rangefinder{ogma::hw::Ultrasonic::Config{}};
        if (a.mic && !mic.start())
            std::fprintf(stderr, "ogma_host: mic: %s\n", mic.last_error().c_str());
        if (a.camera && !cam.start())
            std::fprintf(stderr, "ogma_host: camera: %s\n", cam.last_error().c_str());
        if (a.range && !rangefinder.start())
            std::fprintf(stderr, "ogma_host: rangefinder: %s\n", rangefinder.last_error().c_str());
        std::printf("ogma_host: sensors mic=%s camera=%s range=%s\n",
                    a.mic ? (mic.running() ? "up" : "FAILED") : "off",
                    a.camera ? (cam.running() ? "up" : "FAILED") : "off",
                    a.range ? (rangefinder.running() ? "up" : "FAILED") : "off");


        uint16_t control_port = 7400;
        if (const char* env = std::getenv("OGMA_INSPECTOR_PORT")) {
            const int p = std::atoi(env);
            if (p > 1024 && p < 65534) control_port = uint16_t(p);
        }
        const uint16_t diag_port  = uint16_t(control_port + 1);
        const uint16_t video_port = uint16_t(control_port + 2);
        ogma::DiagPublisher diag(diag_port, a.listen);
        diag.start();

        // The inspector/voice protocol, same three verbs and same shapes the Godot host
        // serves — xaq_voice discovers modules over this socket and then reads the diag
        // PUB stream, so a host without it is mute to every existing tool.
        // The lock guards against the tick thread mutating the module list under a verb.
        std::mutex inst_mtx;
        // The raw sensor values, so a human can see what the encoders were handed.
        // Debugging an encoder through its embedding alone is guesswork: the range
        // channel looked like a change detector for a week and the rangefinder was
        // fine the whole time -- what was missing was the number beside the embedding.
        struct RawSense {
            double range_m = 0, range_rate = 0; bool range_valid = false; uint64_t range_seq = 0;
            double cam_mean = 0; uint64_t cam_frames = 0;
            double mic_peak = 0; uint64_t mic_windows = 0;
        } raw;
        ami_ogma::control::ControlServer control(control_port);
        control.start();
        control.set_command_handler(
            [&](nlohmann::json const& req) -> nlohmann::json {
                std::lock_guard<std::mutex> lk(inst_mtx);
                const std::string verb = req.value("verb", std::string());
                if (verb == "ping")
                    return {{"status","ok"}, {"engine","ogma_host"}, {"ticks", instance->tick_count()}};
                if (verb == "list_modules") {
                    nlohmann::json mods = nlohmann::json::array();
                    for (auto* m : instance->modules())
                        mods.push_back({{"id", std::string(m->id())}, {"type", std::string(m->type_name())}});
                    return {{"status","ok"}, {"modules", mods}};
                }
                if (verb == "module_snapshot") {
                    const std::string id = req.value("id", std::string());
                    auto* m = instance->module(id);
                    if (!m) return {{"status","error"},{"message","unknown module: " + id}};
                    return {{"status","ok"}, {"module_id", id}, {"snapshot", m->snapshot_state()}};
                }
                if (verb == "module_subscribe_diag") {
                    const std::string id    = req.value("id", std::string());
                    const std::string topic = req.value("topic", std::string());
                    const double      hz    = req.value("hz", 30.0);
                    if (!instance->module(id)) return {{"status","error"},{"message","unknown module: " + id}};
                    const int sub_id = diag.subscribe(id, topic, hz);
                    return {{"status","ok"}, {"sub_id", sub_id}, {"diag_port", diag.port()},
                            {"topic_prefix", "diag." + std::to_string(sub_id) + "."}};
                }
                if (verb == "unsubscribe") { diag.unsubscribe(req.value("sub_id", 0)); return {{"status","ok"}}; }
                if (verb == "host_sensors") {
                    // Raw, pre-encoder.  Paired with each channel's health counters so
                    // "no reading" and "a reading of zero" stay distinguishable.
                    return {{"status","ok"},
                            {"range", {{"m", raw.range_m}, {"cm", raw.range_m * 100.0},
                                       {"rate_mps", raw.range_rate}, {"valid", raw.range_valid},
                                       {"seq", raw.range_seq},
                                       {"hz", rangefinder.running() ? 20.0 : 0.0},
                                       {"pings", rangefinder.pings()},
                                       {"timeouts", rangefinder.timeouts()}}},
                            {"camera", {{"mean_level", raw.cam_mean}, {"frames", raw.cam_frames},
                                        {"stride", cam.stride()}, {"frame_bytes", cam.frame_bytes()},
                                        {"out_size", cam.out_size()}}},
                            {"mic", {{"peak", raw.mic_peak}, {"windows", raw.mic_windows},
                                     {"xruns", mic.xruns()}, {"rate", mic.rate()}}}};
                }
                return {{"status","error"}, {"message","unknown verb: " + verb}};
            });

        // ---- video PUB ------------------------------------------------------------
        // A VIEWER, never a participant.  It reads the camera through snapshot(), which
        // does not consume the frame's freshness, so a subscriber cannot starve the EPM
        // of an observation; and it is ZMQ_CONFLATE + DONTWAIT, so a slow or absent
        // viewer costs the tick nothing and never blocks it.  Two planes per message:
        // what the BRAIN sees (32x32, centre-cropped, the encoder's actual input) and
        // what the CAMERA sees (native aspect) -- the pair is the point, because a
        // fault in the pipeline shows as the two disagreeing.
        void*  zmq_ctx = nullptr;
        void*  vid_pub = nullptr;
        if (a.video) {
            zmq_ctx = zmq_ctx_new();
            vid_pub = zmq_socket(zmq_ctx, ZMQ_PUB);
            int conflate = 1, linger = 0, sndhwm = 2;
            zmq_setsockopt(vid_pub, ZMQ_CONFLATE, &conflate, sizeof conflate);
            zmq_setsockopt(vid_pub, ZMQ_LINGER,   &linger,   sizeof linger);
            zmq_setsockopt(vid_pub, ZMQ_SNDHWM,   &sndhwm,   sizeof sndhwm);
            char ep[64];
            std::snprintf(ep, sizeof ep, "tcp://%s:%u", a.listen.c_str(), unsigned(video_port));
            if (zmq_bind(vid_pub, ep) != 0) {
                std::fprintf(stderr, "ogma_host: video bind %s failed: %s\n", ep, zmq_strerror(errno));
                zmq_close(vid_pub); vid_pub = nullptr;
            } else {
                std::printf("ogma_host: video PUB on %s\n", ep);
            }
        }

        const bool rt = a.realtime ? try_realtime() : false;
        std::printf("ogma_host: config=%s hz=%.2f diag=%u %s\n",
                    a.config.c_str(), a.hz, unsigned(diag_port),
                    a.realtime ? (rt ? "SCHED_FIFO" : "SCHED_FIFO DENIED (need CAP_SYS_NICE) -- running SCHED_OTHER")
                               : "SCHED_OTHER");
        std::printf("ogma_host: inspector control=%u (0.0.0.0) diag=%u (%s)\n",
                    unsigned(control_port), unsigned(diag_port), a.listen.c_str());
        std::fflush(stdout);

        // Same absolute-deadline loop shape as benchd's, and the same instrument on
        // it: tick cost as a fraction of the budget, wall split from cpu so "blocked"
        // stays distinguishable from "out of compute" as the graph grows.
        // The EPM's own diagnostics are the first-class instruments (CLAUDE.md §0 rule 4):
        // nodes, baked fraction and TLE. "Never baking, never growing, or growing
        // unbounded" is a conditioning diagnosis, not a verdict -- but it can only be
        // read if the host prints it, so print it.
        const std::vector<std::string> epm_topics = {
            "reality.audio.cochlear", "reality.video.retinal", "reality.proprio.range"};

        ogma::hw::TickBudget budget(a.hz, 25);
        const long period_ns = long(1e9 / a.hz);
        timespec next{};
        clock_gettime(CLOCK_MONOTONIC, &next);
        long ticks = 0, overruns = 0;

        while (g_run && (a.max_ticks == 0 || ticks < a.max_ticks)) {
            next.tv_nsec += period_ns;
            while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; ++next.tv_sec; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

            timespec w0, c0;
            clock_gettime(CLOCK_MONOTONIC, &w0);
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
            const long late_ns = (w0.tv_sec - next.tv_sec) * 1000000000L + (w0.tv_nsec - next.tv_nsec);
            if (late_ns > period_ns) ++overruns;

            // Bridge whatever is NEW onto the Bus, then tick.  Nothing stale is
            // republished: feeding the GNG the same window twice would bake a
            // vocabulary out of the sensor standing still, not out of the world.
            auto* bus = instance->bus();
            if (mic.running()) {
                static std::vector<float> pcm;
                if (mic.latest(pcm) && !pcm.empty()) {
                    { std::lock_guard<std::mutex> lk(inst_mtx);
                      raw.mic_peak = mic.peak(); raw.mic_windows = mic.windows(); }
                    auto f = std::make_shared<ogma::RawAudioFrame>();
                    f->tick_id = uint64_t(ticks); f->producer_id = "host";
                    f->samples = pcm; f->n_samples = int(pcm.size()); f->channels = 1;
                    bus->publish("sense.audio", f);
                }
            }
            if (cam.running()) {
                static std::vector<uint8_t> px;
                if (cam.latest(px) && !px.empty()) {
                    { std::lock_guard<std::mutex> lk(inst_mtx);
                      raw.cam_mean = cam.mean_level(); raw.cam_frames = cam.frames(); }
                    auto f = std::make_shared<ogma::RawImageFrame>();
                    f->tick_id = uint64_t(ticks); f->producer_id = "host";
                    f->pixels = px; f->height = cam.out_size(); f->width = cam.out_size(); f->channels = 1;
                    bus->publish("sense.camera", f);
                }
            }
            if (rangefinder.running()) {
                ogma::hw::Ultrasonic::Reading r;
                if (rangefinder.latest(r)) {
                    { std::lock_guard<std::mutex> lk(inst_mtx);
                      raw.range_m = r.distance_m; raw.range_rate = r.rate_mps;
                      raw.range_valid = r.valid;  raw.range_seq = r.seq; }
                    auto f = std::make_shared<ogma::ProprioToken>();
                    f->tick_id = uint64_t(ticks); f->producer_id = "host"; f->sensor = "range";
                    f->values.resize(2);
                    // [distance, validity].  ⚠ The closing-RATE channel was REMOVED
                    // 2026-09-01: with the target static to within 11 mm it still swung
                    // +/-0.08 m/s, i.e. it was encoding sensor noise, and against the
                    // configured ranges it carried 12x more normalised variance than the
                    // distance it sat beside -- which is why the vocabulary behaved like a
                    // change detector.  The EPM's dual TLE already asks "did I predict
                    // where I would go next?" via transition_surprise, so a hand-computed
                    // derivative was duplicating that in its noisiest form.
                    // Validity stays a CHANNEL, not a silent substitution: an out-of-range
                    // ping and a wall at 0 m are different facts.
                    // distance_m is already max_range_m on a no-echo ping, so there is
                    // no substitution to make here -- the driver owns that meaning.
                    f->values[0] = float(r.distance_m);
                    f->values[1] = r.valid ? 1.0f : 0.0f;
                    bus->publish("sense.range", f);
                }
            }

            {
                std::lock_guard<std::mutex> lk(inst_mtx);
                instance->tick();
            }
            ++ticks;
            if (diag.running()) diag.publish_tick(uint64_t(ticks), *instance);

            if (vid_pub) {
                static uint64_t last_vid_seq = 0;
                static std::vector<uint8_t> vsmall, vprev;
                int pw = 0, ph = 0; uint64_t vseq = 0;
                if (cam.snapshot(vsmall, vprev, pw, ph, vseq) && vseq != last_vid_seq) {
                    last_vid_seq = vseq;
                    // One frame, because ZMQ_CONFLATE does not do multipart: a topic
                    // prefix, a self-describing JSON header, a newline, then the raw
                    // planes back to back.  Sizes travel with the data so a viewer never
                    // has to be told the geometry out of band -- the exact assumption
                    // that broke the camera reader in the first place.
                    char hdr[256];
                    const int n = std::snprintf(hdr, sizeof hdr,
                        "video {\"seq\":%llu,\"tick\":%ld,"
                        "\"brain\":{\"w\":%d,\"h\":%d,\"off\":0},"
                        "\"view\":{\"w\":%d,\"h\":%d,\"off\":%zu},"
                        "\"src\":{\"w\":%d,\"h\":%d},\"fmt\":\"L8\"}\n",
                        (unsigned long long)vseq, ticks,
                        cam.out_size(), cam.out_size(),
                        pw, ph, vsmall.size(),
                        cam.src_width(), cam.src_height());
                    std::string msg(hdr, size_t(n));
                    msg.append(reinterpret_cast<const char*>(vsmall.data()), vsmall.size());
                    msg.append(reinterpret_cast<const char*>(vprev.data()), vprev.size());
                    zmq_send(vid_pub, msg.data(), msg.size(), ZMQ_DONTWAIT);
                }
            }

            timespec w1, c1;
            clock_gettime(CLOCK_MONOTONIC, &w1);
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
            if (budget.sample((w1.tv_sec - w0.tv_sec) * 1000000000L + (w1.tv_nsec - w0.tv_nsec),
                              (c1.tv_sec - c0.tv_sec) * 1000000000L + (c1.tv_nsec - c0.tv_nsec))
                && !a.quiet && (ticks % int(a.hz * 5) < 25)) {
                const auto& s = budget.last();
                std::printf("{\"kind\":\"host_cost\",\"tick\":%ld,\"budget_ms\":%.2f,"
                            "\"wall_p50\":%.3f,\"wall_p95\":%.3f,\"wall_max\":%.3f,"
                            "\"cpu_p50\":%.3f,\"overruns\":%ld,"
                            "\"mic_windows\":%llu,\"mic_peak\":%.4f,\"mic_xruns\":%llu,"
                            "\"cam_frames\":%llu,\"cam_mean\":%.1f,"
                            "\"range_pings\":%llu,\"range_timeouts\":%llu}\n",
                            ticks, s.budget_ms, s.wall_p50, s.wall_p95, s.wall_max, s.cpu_p50, overruns,
                            (unsigned long long)mic.windows(), double(mic.peak()), (unsigned long long)mic.xruns(),
                            (unsigned long long)cam.frames(), double(cam.mean_level()),
                            (unsigned long long)rangefinder.pings(), (unsigned long long)rangefinder.timeouts());
                for (auto const& t : epm_topics) {
                    auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
                        instance->bus()->last_value(t));
                    if (!rt) { std::printf("  %-26s <no token yet>\n", t.c_str()); continue; }
                    std::printf("  %-26s nodes=%-4d winner=%-4d tle=%.4f novel=%d\n",
                                t.c_str(), rt->node_count, rt->winner_id, double(rt->tle),
                                int(rt->is_novel));
                }
                std::fflush(stdout);
            }
        }
        // A channel that died mid-run must SAY so.  "0 windows" is indistinguishable
        // from a silent room, a lens cap, and an empty corridor unless the error shows.
        mic.stop(); cam.stop(); rangefinder.stop();
        if (vid_pub) zmq_close(vid_pub);
        if (zmq_ctx) zmq_ctx_term(zmq_ctx);
        if (!mic.last_error().empty())         std::fprintf(stderr, "ogma_host: mic died: %s\n", mic.last_error().c_str());
        if (!cam.last_error().empty())         std::fprintf(stderr, "ogma_host: camera died: %s\n", cam.last_error().c_str());
        if (!rangefinder.last_error().empty()) std::fprintf(stderr, "ogma_host: rangefinder died: %s\n", rangefinder.last_error().c_str());
        std::printf("ogma_host: stopped after %ld ticks (%ld overruns)\n", ticks, overruns);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ogma_host: %s\n", e.what());
        return 1;
    }
}
