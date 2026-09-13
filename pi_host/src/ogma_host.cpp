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
#include <sys/stat.h>
#include <limits.h>

#ifndef OGMA_HOST_GIT_SHA
#define OGMA_HOST_GIT_SHA "unknown"
#endif
#include "ogma/Topics.hpp"
#include "ogma/hw/ResourceMonitor.hpp"
#include "ogma/hw/AudioCapture.hpp"
#include "ogma/hw/CameraCapture.hpp"
#include "ogma/hw/Ultrasonic.hpp"
#include "ogma/hw/Vl53l0x.hpp"
#include "ogma/hw/Icm20948.hpp"
#include "ogma/hw/I2cBus.hpp"
#include "ogma/hw/SensorCalib.hpp"
#include "ogma/body/StrideOdometry.hpp"

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
#include <fstream>
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
    bool    video_mono = false;   // drop the preview's chroma (1.5x -> 1x on a weak link)
    bool    mic        = false;
    bool    camera     = false;
    bool    range      = false;
    // ⚠ OPT-IN, AND IT TAKES A BUS benchd HOLDS.  The VL53L0X is on /dev/i2c-1 with the
    // HAT, and the README's standing rule is that only one process owns that bus at a
    // time (it already says to stop benchd before hat_tool).  The two systemd units now
    // declare Conflicts= so this cannot be arranged by accident, but the flag stays
    // default-off so the bench keeps the belly readout unless the brain is asked for it.
    bool    tof        = false;
    // ⚠ SPI, NOT I2C — so unlike --tof this contends with nothing.  benchd reads the
    // same part today; both can hold /dev/spidev0.0 without interleaving because each
    // transfer is one full-duplex ioctl, not a pointer-write followed by a read.
    bool    imu        = false;
};

void usage() {
    std::fprintf(stderr,
        "usage: ogma_host --config <graph.json> [--hz 50] [--ticks N] [--rt] [--quiet]\n"
        "                 [--mic] [--camera] [--range] [--listen 0.0.0.0] [--video] [--video-mono]\n"
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
        else if (v == "--video-mono")             a.video_mono = true;
        else if (v == "--mic")                    a.mic = true;
        else if (v == "--camera")                 a.camera = true;
        else if (v == "--range")                  a.range = true;
        else if (v == "--tof")                    a.tof = true;
        else if (v == "--imu")                    a.imu = true;
        else { usage(); return 2; }
    }
    if (a.config.empty() || a.hz <= 0.0) { usage(); return 2; }

    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    try {
        // The config's own name, so the dash can say WHICH graph is loaded rather than
        // only which path — two checkouts can have the same filename.
        std::string cfg_name, cfg_phase;
        try {
            std::ifstream cf(a.config);
            nlohmann::json cj; cf >> cj;
            cfg_name  = cj.value("metadata", nlohmann::json::object()).value("name", std::string());
            cfg_phase = cj.value("metadata", nlohmann::json::object()).value("phase_tag", std::string());
        } catch (...) { /* metadata is a convenience; a config without it still runs */ }

        auto cfg = ogma::GraphConfig::load_from_file(a.config);
        auto instance = std::make_unique<ogma::OgmaInstance>(
            std::move(cfg), std::make_unique<ogma::InProcessBus>());

        // Sensors start BEFORE the loop so a failure is a startup error the operator
        // sees, not a channel that is quietly dead for the whole run.
        ogma::hw::AudioCapture  mic{ogma::hw::AudioCapture::Config{}};
        ogma::hw::CameraCapture cam{[&]{
            ogma::hw::CameraCapture::Config c;
            c.preview_color = !a.video_mono;   // the brain's plane is luma-only either way
            return c;
        }()};
        ogma::hw::Ultrasonic    rangefinder{ogma::hw::Ultrasonic::Config{}};

        // ---- belly ToF (VL53L0X) -------------------------------------------------
        // The `gc_raw` channel the PROMOTED height homeostat rides — the lever that
        // replaced the god's-eye chassis_y_norm and solved the hump.  Opened here so a
        // failure is a startup error, and only when asked (it takes benchd's bus).
        const auto calib = ogma::hw::SensorCalib::load();
        std::unique_ptr<ogma::hw::LinuxI2cBus> i2c;
        std::unique_ptr<ogma::hw::Vl53l0x>     tof;
        if (a.tof) {
            i2c = std::make_unique<ogma::hw::LinuxI2cBus>("/dev/i2c-1");
            ogma::hw::Vl53l0xConfig tc;
            tc.mount_offset_mm = calib.tof_mount_offset_mm;
            tof = std::make_unique<ogma::hw::Vl53l0x>(*i2c, tc);
            tof->init();
            tof->start_continuous();
            std::fprintf(stderr,
                "ogma_host: VL53L0X 0x29 ready — calib %s (%s), mount_offset %.2f mm, "
                "gc_stand %.3f m\n",
                calib.source.c_str(), calib.loaded ? "loaded" : "MISSING, using defaults",
                calib.tof_mount_offset_mm, calib.gc_stand_m);
        }

        // ---- IMU (ICM-20948, SPI) ------------------------------------------------
        // Publishes `upright` and `tilt` from the FUSED gravity estimate — the honest
        // forms, not an exact basis, which is all a robot has.  Step (c) measured that
        // substitution in sim as behaviourally free (ledger 2026-09-11), so this is the
        // validated channel rather than a hopeful one.
        std::unique_ptr<ogma::hw::Icm20948> imu;
        if (a.imu) {
            ogma::hw::Icm20948Config ic;
            ic.level_ref = calib.imu_level_ref;
            imu = std::make_unique<ogma::hw::Icm20948>(ic);
            std::string err;
            if (!imu->begin(&err)) {
                // A dead IMU must be a startup failure, not a silently absent topic:
                // `upright` gates keyframe baking, and a consumer that never sees it
                // behaves differently from one that sees it wrong — but neither should
                // be discovered halfway through a run.
                std::fprintf(stderr, "ogma_host: ICM-20948 begin failed: %s\n", err.c_str());
                return 3;
            }
            std::fprintf(stderr,
                "ogma_host: ICM-20948 ready (who_am_i 0x%02X) — level_ref [%.5f %.5f %.5f] "
                "from %s (%s)\n",
                imu->who_am_i(), ic.level_ref[0], ic.level_ref[1], ic.level_ref[2],
                calib.source.c_str(), calib.loaded ? "loaded" : "MISSING, using defaults");
        }
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
        const auto t_start = std::chrono::steady_clock::now();
        // Declared here, assigned below: the control handler captures by reference and
        // reports it, and SCHED_FIFO cannot be requested until the sensors are up.
        bool rt = false;
        // The raw sensor values, so a human can see what the encoders were handed.
        // Debugging an encoder through its embedding alone is guesswork: the range
        // channel looked like a change detector for a week and the rangefinder was
        // fine the whole time -- what was missing was the number beside the embedding.
        struct RawSense {
            double range_m = 0, range_rate = 0; bool range_valid = false; uint64_t range_seq = 0;
            double cam_mean = 0; uint64_t cam_frames = 0;
            double mic_peak = 0; uint64_t mic_windows = 0; uint64_t mic_delivered = 0;
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
                if (verb == "host_info") {
                    // "What is actually running here" — asked of the process itself
                    // rather than inferred from a repo the operator is standing in,
                    // which may be a different checkout from the one that built this.
                    auto stat_of = [](const std::string& path) {
                        struct stat st{};
                        nlohmann::json j;
                        if (::stat(path.c_str(), &st) == 0) {
                            j["bytes"] = int64_t(st.st_size);
                            char buf[32];
                            std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M",
                                          std::localtime(&st.st_mtime));
                            j["mtime"] = buf;
                        } else {
                            j["missing"] = true;
                        }
                        return j;
                    };
                    char exe[PATH_MAX] = {0};
                    const ssize_t elen = ::readlink("/proc/self/exe", exe, sizeof exe - 1);
                    const std::string exe_path = elen > 0 ? std::string(exe, size_t(elen)) : "?";
                    char cwd[PATH_MAX] = {0};
                    if (!::getcwd(cwd, sizeof cwd)) cwd[0] = 0;
                    nlohmann::json mods = nlohmann::json::array();
                    for (auto* m : instance->modules())
                        mods.push_back({{"id", std::string(m->id())}, {"type", std::string(m->type_name())}});
                    return {{"status","ok"},
                            {"git_sha", OGMA_HOST_GIT_SHA},
                            {"binary", {{"path", exe_path}, {"stat", stat_of(exe_path)}}},
                            {"config", {{"path", a.config}, {"stat", stat_of(a.config)},
                                        {"name", cfg_name}, {"phase_tag", cfg_phase}}},
                            {"cwd", std::string(cwd)},
                            {"hz", a.hz},
                            {"realtime", rt},
                            {"listen", a.listen},
                            {"ports", {{"control", control_port}, {"diag", diag_port},
                                       {"video", a.video ? video_port : 0}}},
                            {"sensors", {{"mic", a.mic}, {"camera", a.camera}, {"range", a.range}}},
                            {"modules", mods}};
                }
                if (verb == "host_sensors") {
                    const double up_s = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t_start).count();
                    // Raw, pre-encoder.  Paired with each channel's health counters so
                    // "no reading" and "a reading of zero" stay distinguishable.
                    // Rates, not cumulative totals: "104569 frames" says how long the
                    // process has been up, which `uptime_s` already says once.  What a
                    // reader needs is whether the channel is keeping pace NOW, and
                    // whether anything was dropped -- a producer overwriting an unread
                    // observation is the sensor equivalent of a tick overrun.
                    auto per_s = [&](uint64_t n) { return up_s > 0.5 ? double(n) / up_s : 0.0; };
                    return {{"status","ok"},
                            {"uptime_s", up_s},
                            {"range", {{"m", raw.range_m}, {"cm", raw.range_m * 100.0},
                                       {"rate_mps", raw.range_rate}, {"valid", raw.range_valid},
                                       {"up", rangefinder.running()},
                                       {"hz", per_s(rangefinder.pings() + rangefinder.timeouts())},
                                       {"pings", rangefinder.pings()},
                                       {"timeouts", rangefinder.timeouts()},
                                       {"dropped", rangefinder.dropped()}}},
                            {"camera", {{"mean_level", raw.cam_mean},
                                        {"up", cam.running()},
                                        {"fps", per_s(cam.frames())},
                                        {"dropped", cam.dropped()},
                                        {"stride", cam.stride()}, {"frame_bytes", cam.frame_bytes()},
                                        {"out_size", cam.out_size()}}},
                            // ⚠ `hz` is the PRODUCER (ALSA reads); `delivered_hz` is what
                            // actually reached the Bus.  They must track.  They did not:
                            // 46.75 vs 39.90 measured 2026-09-05, a 14.7 % loss that
                            // `dropped` reported as 0 because it could not see it.  Both
                            // are published so the gap is visible without inference.
                            {"mic", {{"peak", raw.mic_peak},
                                     {"up", mic.running()},
                                     {"hz", per_s(raw.mic_windows)},
                                     {"delivered_hz", per_s(raw.mic_delivered)},
                                     {"windows", raw.mic_windows},
                                     {"delivered", raw.mic_delivered},
                                     {"xruns", mic.xruns()}, {"dropped", mic.dropped()},
                                     {"rate", mic.rate()}}}};
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

        rt = a.realtime ? try_realtime() : false;
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
        // ⚠ A SENSOR THAT PUBLISHED NOTHING LOOKS EXACTLY LIKE A HEALTHY ONE from
        // outside the process, so the belly channel counts itself and says so at exit.
        // "It ran with --tof and did not crash" is not evidence that a reading reached
        // the bus (CLAUDE.md §3.2: did the consumer actually fire?).
        long tof_reads = 0, tof_valid = 0;
        ogma::hw::Vl53l0x::Reading tof_last{};
        long imu_reads = 0;
        ogma::hw::ImuSample imu_last{};

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
                      raw.mic_peak = mic.peak(); raw.mic_windows = mic.windows();
                      raw.mic_delivered = mic.delivered(); }
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
            if (tof) {
                ogma::hw::Vl53l0x::Reading r;
                if (tof->read_ready(r)) {
                    ++tof_reads;
                    if (r.status == ogma::hw::Vl53l0x::Status::Valid) ++tof_valid;
                    tof_last = r;
                    // ⚠ THE STATUS IS THE CHANNEL, not a detail.  A reading that failed
                    // the part's sigma/signal checks is not large or small — it is
                    // ARBITRARY, and at the consumer it looks exactly like a good one.
                    // So the raw reading and its status ALWAYS go out for diagnosis...
                    auto d = std::make_shared<ogma::ProprioToken>();
                    d->tick_id = uint64_t(ticks); d->producer_id = "host";
                    d->sensor = "belly";
                    d->values.resize(6);
                    d->values[0] = float(r.distance_m);   // offset-corrected metres
                    d->values[1] = float(r.raw_mm);       // as the chip reported it
                    d->values[2] = float(static_cast<uint8_t>(r.status));
                    d->values[3] = float(r.signal_mcps);  // signal/ambient move BEFORE
                    d->values[4] = float(r.ambient_mcps); // the status flips
                    d->values[5] = float(r.spads);        // drop = fouled aperture
                    bus->publish("sense.belly", d);

                    // ...but the PROMOTED topic is published only on a good reading, and
                    // is simply ABSENT otherwise.  That is this project's own rule for an
                    // exactly-round null (the beacon publisher states it): a consumer that
                    // needs the channel then fails loudly instead of reading a plausible
                    // zero, which is the failure shape that has produced false verdicts
                    // here before.  Never substitute a number for a missing measurement.
                    // ⚠ `valid` and `distance_m` disagree DELIBERATELY on a bad read:
                    // the driver reports max_range_m there, not zero, because zero maps
                    // "saw nothing" onto "something against the belly" — the opposite
                    // extreme and the worst available answer.  Gating on the status is
                    // what keeps that honest floor out of a promoted lever.
                    if (r.status == ogma::hw::Vl53l0x::Status::Valid) {
                        auto f2 = std::make_shared<ogma::ProprioToken>();
                        f2->tick_id = uint64_t(ticks); f2->producer_id = "host";
                        f2->sensor = "ground_clearance";
                        // ONE value, and the same normalizer the sim uses — the token is
                        // shape- and scale-identical to reality.proprio.ground_clearance
                        // there, so a consumer cannot tell which body it is attached to.
                        f2->values.resize(1);
                        f2->values[0] = float(ogma::body::ground_clearance(
                                            r.distance_m, calib.gc_stand_m));
                        bus->publish("reality.proprio.ground_clearance", f2);
                    }
                }
            }
            if (imu) {
                ogma::hw::ImuSample m;
                if (imu->sample(m) && m.ok) {
                    ++imu_reads;
                    imu_last = m;
                    const ogma::body::Vec3f up(m.up_fused[0], m.up_fused[1], m.up_fused[2]);

                    // `upright` — the SAME SCALAR the sim publishes from basis.y.y, via
                    // the shared contract, so a consumer cannot tell the bodies apart.
                    auto u = std::make_shared<ogma::ProprioToken>();
                    u->tick_id = uint64_t(ticks); u->producer_id = "host";
                    u->sensor = "upright";
                    u->values.resize(1);
                    u->values[0] = float(ogma::body::upright_from_up(up));
                    bus->publish("reality.proprio.upright", u);

                    // `tilt` — [sin p, cos p, sin r, cos r].  The unit-circle encoding is
                    // the sim's and is not decoration: raw radians wrap at ±π and an EPM
                    // reads that discontinuity as a jump in the world.
                    double pitch = 0.0, roll = 0.0;
                    ogma::body::pitch_roll_from_up(up, pitch, roll);
                    auto t = std::make_shared<ogma::ProprioToken>();
                    t->tick_id = uint64_t(ticks); t->producer_id = "host";
                    t->sensor = "tilt";
                    t->values.resize(4);
                    t->values[0] = float(std::sin(pitch)); t->values[1] = float(std::cos(pitch));
                    t->values[2] = float(std::sin(roll));  t->values[3] = float(std::cos(roll));
                    bus->publish("reality.proprio.tilt", t);

                    // ⚠ THE HEALTH SIGNAL IS A CHANNEL.  There is no ground-truth attitude
                    // on a robot, so accel-vs-fused disagreement is the only thing that
                    // says whether the filter is working — and the gyro bias estimator's
                    // convergence is the only thing that says whether to believe it yet.
                    // Both ride here rather than being inferred from the attitude itself.
                    auto h = std::make_shared<ogma::ProprioToken>();
                    h->tick_id = uint64_t(ticks); h->producer_id = "host";
                    h->sensor = "imu_health";
                    h->values.resize(5);
                    h->values[0] = m.disagree_deg;
                    h->values[1] = m.a_norm_g;
                    h->values[2] = m.trust;
                    h->values[3] = m.bias_valid ? 1.0f : 0.0f;
                    h->values[4] = float(m.bias_samples);
                    bus->publish("sense.imu_health", h);
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
                static ogma::hw::CameraCapture::Frame vf;
                if (cam.snapshot(vf) && vf.seq != last_vid_seq) {
                    last_vid_seq = vf.seq;
                    // One frame, because ZMQ_CONFLATE does not do multipart: a topic
                    // prefix, a self-describing JSON header, a newline, then the raw
                    // planes back to back.  Every size travels with the data so a viewer
                    // never has to be told the geometry out of band -- the exact
                    // assumption that broke the camera reader in the first place.
                    //
                    // The preview carries CHROMA when the camera kept it, as YUV420
                    // rather than RGB: the chroma planes are quarter-size, so it is 1.5x
                    // the bytes instead of 3x, it is what the camera actually produced
                    // (no lossy conversion here), and the RGB conversion lands on the
                    // viewer's machine instead of on this 20 ms tick.
                    // The BRAIN's plane stays luma-only: the retinal encoder is 32x32x1.
                    const size_t off_y = vf.small.size();
                    const size_t off_u = off_y + vf.view_y.size();
                    const size_t off_v = off_u + vf.view_u.size();
                    const bool   color = vf.chroma_w > 0 && !vf.view_u.empty() && !vf.view_v.empty();
                    char hdr[384];
                    int n = std::snprintf(hdr, sizeof hdr,
                        "video {\"seq\":%llu,\"tick\":%ld,"
                        "\"brain\":{\"w\":%d,\"h\":%d,\"off\":0},"
                        "\"view\":{\"w\":%d,\"h\":%d,\"off\":%zu},",
                        (unsigned long long)vf.seq, ticks,
                        cam.out_size(), cam.out_size(),
                        vf.view_w, vf.view_h, off_y);
                    if (color)
                        n += std::snprintf(hdr + n, sizeof hdr - size_t(n),
                            "\"view_u\":{\"w\":%d,\"h\":%d,\"off\":%zu},"
                            "\"view_v\":{\"w\":%d,\"h\":%d,\"off\":%zu},",
                            vf.chroma_w, vf.chroma_h, off_u,
                            vf.chroma_w, vf.chroma_h, off_v);
                    n += std::snprintf(hdr + n, sizeof hdr - size_t(n),
                        "\"src\":{\"w\":%d,\"h\":%d},\"fmt\":\"%s\"}\n",
                        cam.src_width(), cam.src_height(), color ? "YUV420" : "L8");
                    std::string msg(hdr, size_t(n));
                    msg.append(reinterpret_cast<const char*>(vf.small.data()),  vf.small.size());
                    msg.append(reinterpret_cast<const char*>(vf.view_y.data()), vf.view_y.size());
                    if (color) {
                        msg.append(reinterpret_cast<const char*>(vf.view_u.data()), vf.view_u.size());
                        msg.append(reinterpret_cast<const char*>(vf.view_v.data()), vf.view_v.size());
                    }
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
                            "\"mic_windows\":%llu,\"mic_delivered\":%llu,"
                            "\"mic_peak\":%.4f,\"mic_xruns\":%llu,"
                            "\"cam_frames\":%llu,\"cam_mean\":%.1f,"
                            "\"range_pings\":%llu,\"range_timeouts\":%llu}\n",
                            ticks, s.budget_ms, s.wall_p50, s.wall_p95, s.wall_max, s.cpu_p50, overruns,
                            (unsigned long long)mic.windows(), (unsigned long long)mic.delivered(),
                            double(mic.peak()), (unsigned long long)mic.xruns(),
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
        if (imu) {
            // Same reason the belly channel counts itself: from outside the process an
            // IMU publishing nothing looks exactly like one publishing well.  bias_valid
            // is the number to read first — attitude from an unconverged gyro bias is a
            // confident wrong answer, and the estimator needs ~8 s of stillness.
            std::printf("ogma_host: IMU — %ld samples, disagree %.3f deg, |a| %.4f g, "
                        "trust %.4f, bias %s (%d samples) [%.3f %.3f %.3f dps], "
                        "up_fused [%.4f %.4f %.4f] -> upright %.4f\n",
                        imu_reads, imu_last.disagree_deg, imu_last.a_norm_g, imu_last.trust,
                        imu_last.bias_valid ? "CONVERGED" : "not converged",
                        imu_last.bias_samples,
                        imu_last.gyro_bias_dps[0], imu_last.gyro_bias_dps[1],
                        imu_last.gyro_bias_dps[2],
                        imu_last.up_fused[0], imu_last.up_fused[1], imu_last.up_fused[2],
                        ogma::body::upright_from_up(ogma::body::Vec3f(
                            imu_last.up_fused[0], imu_last.up_fused[1], imu_last.up_fused[2])));
        }
        if (tof) {
            std::printf("ogma_host: belly ToF — %ld reads, %ld valid (%.1f%%), last "
                        "raw %u mm -> %.3f m (status %s, signal %.2f ambient %.2f "
                        "spads %.1f) -> ground_clearance %.4f\n",
                        tof_reads, tof_valid,
                        tof_reads ? 100.0 * double(tof_valid) / double(tof_reads) : 0.0,
                        unsigned(tof_last.raw_mm), tof_last.distance_m,
                        ogma::hw::Vl53l0x::status_name(tof_last.status),
                        tof_last.signal_mcps, tof_last.ambient_mcps, tof_last.spads,
                        ogma::body::ground_clearance(tof_last.distance_m, calib.gc_stand_m));
        }
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ogma_host: %s\n", e.what());
        return 1;
    }
}
