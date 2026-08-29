// xaq_voice — sonification of the brain's error signal.
//
// An INSTRUMENT, not a behaviour: the brain does not control this and cannot hear it.
// It subscribes to the diag stream (control socket: newline JSON on OGMA_INSPECTOR_PORT,
// diag: ZMQ PUB 2-part frames on port+1 — exactly what tools/xaq_inspector reads), so it
// costs the brain nothing and works unchanged against the sim and against ogma_host.
//
// This file is the wiring: command line, the brain connection, source discovery, and the
// two threads that meet in Engine.  The instrument itself lives in
//   patch.hpp/.cpp    what the synth IS, as data — sources, routes, destinations
//   dsp.hpp           oscillators, the state-variable filter, the vowel bank
//   engine.hpp/.cpp   the running instrument: frames in, sound out
//   control.hpp/.cpp  the studio's request/reply + meter stream
//
// Launch with no --config and the engine discovers what the brain publishes and builds a
// patch from it, reproducing the mapping this tool has always had: TLE to pitch through
// a running median/MAD, TLE against the module's own novelty threshold to volume, GNG
// life events as chirps.  Everything past that is the patch's business, not this file's.
#include <alsa/asoundlib.h>
#include <nlohmann/json.hpp>
#include <zmq.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "control.hpp"
#include "engine.hpp"
#include "patch.hpp"

using json = nlohmann::json;
using namespace xv;

namespace {

std::atomic<bool> g_run{true};
void on_signal(int) { g_run = false; }      // always reach the unsubscribe path

// ---------------------------------------------------------------- control socket
int tcp_connect(const std::string& host, int port) {
    addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res     = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) return -1;
    int fd = -1;
    for (addrinfo* p = res; p; p = p->ai_next) {
        fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd >= 0 && ::connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
        if (fd >= 0) { close(fd); fd = -1; }
    }
    freeaddrinfo(res);
    return fd;
}

json control_call(int fd, const json& req) {
    const std::string s = req.dump() + "\n";
    if (send(fd, s.data(), s.size(), 0) < 0) return {{"status", "error"}, {"message", "send"}};
    std::string buf;
    char        c;
    while (recv(fd, &c, 1, 0) == 1) {
        if (c == '\n') break;
        buf += c;
    }
    try { return json::parse(buf); } catch (...) { return {{"status", "error"}, {"message", "bad json"}}; }
}

// ---------------------------------------------------------------- subscriptions
struct Sub {
    std::string module, type, prefix;
    int         sub_id = -1;
};

// The brain decimates by whole ticks: interval = round(host_hz / requested), clamped at 1.
// So a request for 50 Hz against a 60 Hz host silently becomes 60.  Report what will
// actually arrive rather than what was asked for — a rate that is quietly 20% off is
// exactly the kind of thing that gets blamed on the synth later.
double achievable_hz(double requested, double host_hz = 60.0) {
    if (requested <= 0.0) return host_hz;
    const double interval = std::max(1.0, std::round(host_hz / requested));
    return host_hz / interval;
}

// ---------------------------------------------------------------- audio
void audio_thread(Engine* engine, std::string device, int rate, bool no_audio) {
    snd_pcm_t* pcm = nullptr;
    if (!no_audio) {
        if (snd_pcm_open(&pcm, device.c_str(), SND_PCM_STREAM_PLAYBACK, 0) < 0 ||
            snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 2,
                               rate, 1, 40000) < 0) {
            std::fprintf(stderr,
                         "xaq_voice: ALSA open/setup failed on '%s' — running silent "
                         "(--no-audio to hide this)\n",
                         device.c_str());
            pcm = nullptr;
        }
    }
    const int             N = 256;                 // ~5.3 ms at 48 kHz
    std::vector<int16_t>  buf(size_t(N) * 2);      // interleaved stereo
    while (g_run) {
        engine->render(buf.data(), N);
        if (pcm) {
            const snd_pcm_sframes_t w = snd_pcm_writei(pcm, buf.data(), N);
            if (w < 0) snd_pcm_recover(pcm, int(w), 1);
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(N * 1000000 / rate));
        }
    }
    if (pcm) { snd_pcm_drain(pcm); snd_pcm_close(pcm); }
}

// ---------------------------------------------------------------- patch files
bool write_patch(const Patch& p, const std::string& path, std::string& err) {
    std::ofstream f(path);
    if (!f) { err = "cannot open " + path + " for writing"; return false; }
    f << to_json(p).dump(2) << "\n";
    if (!f) { err = "write failed: " + path; return false; }
    return true;
}

bool read_patch(const std::string& path, Patch& out, std::string& err) {
    std::ifstream f(path);
    if (!f) { err = "cannot open " + path; return false; }
    try {
        json j;
        f >> j;
        out = from_json(j);
        return true;
    } catch (const std::exception& e) {
        err = std::string("parse failed: ") + e.what();
        return false;
    }
}

// ---------------------------------------------------------------- keyboard
struct KeyCtx {
    Engine*     engine   = nullptr;
    std::string save_path;
    int         selected = 0;
};
KeyCtx g_keys;

void print_voices() {
    const Patch p = g_keys.engine->patch();
    std::printf("  voices:");
    for (size_t i = 0; i < p.voices.size(); ++i)
        std::printf(" %zu:%s@%s", i + 1, p.voices[i].id.c_str(),
                    hz_to_note(float(p.voices[i].osc.base_hz)).c_str());
    std::printf("\n");
}

// Retune the selected voice's base pitch.  The studio owns tuning now, but a terminal-only
// session on the Pi still needs to be able to move a voice off another one.
void retune(float semis) {
    Patch p = g_keys.engine->patch();
    if (p.voices.empty()) return;
    const int i = std::clamp(g_keys.selected, 0, int(p.voices.size()) - 1);
    p.voices[size_t(i)].osc.base_hz *= std::pow(2.0, semis / 12.0);
    g_keys.engine->set_patch(p);
    std::printf("[voice %d %s base -> %s %.1f Hz]\n", i + 1, p.voices[size_t(i)].id.c_str(),
                hz_to_note(float(p.voices[size_t(i)].osc.base_hz)).c_str(),
                p.voices[size_t(i)].osc.base_hz);
    print_voices();
}

void save_now() {
    if (g_keys.save_path.empty()) {
        std::printf("[no --save path; use --save <file.json> or the studio]\n");
        return;
    }
    std::string err;
    if (write_patch(g_keys.engine->patch(), g_keys.save_path, err))
        std::printf("[saved %s]\n", g_keys.save_path.c_str());
    else
        std::printf("[save failed: %s]\n", err.c_str());
}

void keyboard_thread() {
    termios old{};
    const bool tty = isatty(0);
    if (tty) {
        tcgetattr(0, &old);
        termios raw = old;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &raw);
    }
    while (g_run) {
        char c;
        if (read(0, &c, 1) != 1) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
        Engine* e = g_keys.engine;
        Patch   p = e->patch();
        if (c == 'q') {
            p.master.quantize = !p.master.quantize;
            e->set_patch(p);
            std::printf("[pitch %s]\n", p.master.quantize ? "QUANTISED" : "RAW");
        } else if (c == '+' || c == '=') {
            p.master.volume = std::min(1.0, p.master.volume + 0.1);
            e->set_patch(p);
            std::printf("[master %.1f]\n", p.master.volume);
        } else if (c == '-') {
            p.master.volume = std::max(0.0, p.master.volume - 0.1);
            e->set_patch(p);
            std::printf("[master %.1f]\n", p.master.volume);
        } else if (c == 'm') {
            e->set_mute(!e->muted());
            std::printf("[%s]\n", e->muted() ? "MUTE" : "unmute");
        } else if (c == 't') {
            e->set_tone_enabled(!e->tone_enabled());
            std::printf("[TLE tone %s]\n", e->tone_enabled() ? "on" : "OFF");
        } else if (c >= '1' && c <= '9') {
            g_keys.selected = c - '1';
            if (g_keys.selected < int(p.voices.size()))
                std::printf("[tuning voice %d: %s]\n", g_keys.selected + 1,
                            p.voices[size_t(g_keys.selected)].id.c_str());
            else
                g_keys.selected = 0;
        } else if (c == '<') { retune(-1.f); }
        else if (c == '>')   { retune(+1.f); }
        else if (c == '{')   { retune(-12.f); }
        else if (c == '}')   { retune(+12.f); }
        else if (c == 'w')   { save_now(); }
        else if (c == 'v')   { print_voices(); }
        else if (c == 3 || c == 'x') { g_run = false; }
        std::fflush(stdout);
    }
    if (tty) tcsetattr(0, TCSANOW, &old);
}

// The tuning flags this tool has always had.  They are no longer the model — a patch is —
// but they remain the fastest way to say "louder sooner" without opening a GUI, and the
// tool has spent its life telling the operator to paste a `--base …` line into the next
// launch.  They are applied as overrides ON TOP of whatever patch was built or loaded, so
// they mean the same thing in both cases.
struct LegacyTuning {
    double gate = -1, full = -1, gamma = -1, span = -1;
    std::map<std::string, double> base;      // voice id -> Hz
    bool any() const { return gate >= 0 || full >= 0 || gamma >= 0 || span >= 0 || !base.empty(); }
};

void apply_legacy(Patch& p, const LegacyTuning& t) {
    for (auto& v : p.voices) {
        if (auto it = t.base.find(v.id); it != t.base.end()) v.osc.base_hz = it->second;
        for (auto& r : v.routes) {
            if (r.dest == Dest::Pitch && t.span >= 0) r.depth = t.span;
            if (r.dest == Dest::Amp) {
                if (t.gate  >= 0) r.norm.gate = t.gate;
                if (t.full  >= 0) r.norm.full = t.full;
                if (t.gamma >= 0) r.curve     = t.gamma;
            }
        }
    }
    if (t.span >= 0) p.master.span = t.span;
}

void usage() {
    std::fprintf(stderr,
        "usage: xaq_voice [--host H] [--port 7400] [--device default] [--rate 48000]\n"
        "                 [--modules a,b] [--hz 30] [--discover-ms 1200]\n"
        "                 [--config patch.json] [--save patch.json] [--vary]\n"
        "                 [--control-port 7460] [--no-control] [--bind 127.0.0.1]\n"
        "                 [--volume 0.5] [--raw] [--no-audio] [--verbose]\n"
        "                 [--gate 1.4] [--full 2.0] [--gamma 0.5] [--span 24]\n"
        "                 [--base id=C3,id=A4,...]\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string host = "127.0.0.1", device = "default", modules_arg, config_path, save_path;
    std::string bind_host = "127.0.0.1";
    int    port = 7400, rate = 48000, control_port = 7460, discover_ms = 1200;
    double hz = 30.0;
    bool   no_audio = false, quiet = true, no_control = false, vary = false, raw_pitch = false;
    double volume = -1.0;
    LegacyTuning legacy;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if      (a == "--host")         host = next();
        else if (a == "--port")         port = std::stoi(next());
        else if (a == "--device")       device = next();
        else if (a == "--rate")         rate = std::stoi(next());
        else if (a == "--hz")           hz = std::stod(next());
        else if (a == "--modules")      modules_arg = next();
        else if (a == "--config")       config_path = next();
        else if (a == "--save")         save_path = next();
        else if (a == "--discover-ms")  discover_ms = std::stoi(next());
        else if (a == "--control-port") control_port = std::stoi(next());
        else if (a == "--bind")         bind_host = next();
        else if (a == "--no-control")   no_control = true;
        else if (a == "--vary")         vary = true;
        else if (a == "--raw")          raw_pitch = true;
        else if (a == "--volume")       volume = std::stod(next());
        else if (a == "--no-audio")     no_audio = true;
        else if (a == "--verbose")      quiet = false;
        else if (a == "--quiet")        quiet = true;
        else if (a == "--gate")         legacy.gate = std::stod(next());
        else if (a == "--full")         legacy.full = std::stod(next());
        else if (a == "--gamma")        legacy.gamma = std::stod(next());
        else if (a == "--span")         legacy.span = std::stod(next());
        else if (a == "--base") {
            const std::string b = next();
            size_t st = 0;
            while (st <= b.size()) {
                size_t e = b.find(',', st);
                if (e == std::string::npos) e = b.size();
                const std::string kv = b.substr(st, e - st);
                const size_t eq = kv.find('=');
                if (eq != std::string::npos) {
                    const float f = note_to_hz(kv.substr(eq + 1));
                    if (f > 0) legacy.base[kv.substr(0, eq)] = f;
                }
                st = e + 1;
            }
        }
        else { usage(); return 2; }
    }
    if (const char* e = std::getenv("OGMA_INSPECTOR_PORT")) if (port == 7400) port = std::atoi(e);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGHUP, on_signal);

    // ---- brain ----
    int ctl = tcp_connect(host, port);
    if (ctl < 0) {
        std::fprintf(stderr, "xaq_voice: cannot reach control socket %s:%d (is the brain running?)\n",
                     host.c_str(), port);
        return 1;
    }
    const json mods = control_call(ctl, {{"verb", "list_modules"}});
    if (mods.value("status", "") != "ok") {
        std::fprintf(stderr, "xaq_voice: list_modules failed: %s\n", mods.dump().c_str());
        return 1;
    }

    std::vector<std::string> wanted;
    if (!modules_arg.empty()) {
        size_t s = 0;
        while (s <= modules_arg.size()) {
            size_t e = modules_arg.find(',', s);
            if (e == std::string::npos) e = modules_arg.size();
            if (e > s) wanted.push_back(modules_arg.substr(s, e - s));
            s = e + 1;
        }
    }

    // Subscribe EVERY module unless told otherwise.  The old filter took only types whose
    // name contained "EPM", which both missed sources the brain does publish and selected
    // three types that publish nothing at all.  A "lite" payload is a few hundred bytes;
    // the tool cannot know which signal will turn out to be the expressive one, and the
    // studio can only offer what was actually subscribed.
    Engine engine(rate);
    std::vector<Sub> subs;
    for (const auto& m : mods["modules"]) {
        const std::string id = m.value("id", ""), type = m.value("type", "");
        if (id.empty()) continue;
        if (!wanted.empty() && std::find(wanted.begin(), wanted.end(), id) == wanted.end()) continue;
        const json r = control_call(
            ctl, {{"verb", "module_subscribe_diag"}, {"id", id}, {"topic", "lite"}, {"hz", hz}});
        if (r.value("status", "") != "ok") {
            std::fprintf(stderr, "xaq_voice: subscribe %s failed: %s\n", id.c_str(), r.dump().c_str());
            continue;
        }
        Sub s;
        s.module = id;
        s.type   = type;
        s.sub_id = r.value("sub_id", -1);
        s.prefix = r.value("topic_prefix", "diag." + std::to_string(s.sub_id) + ".");
        subs.push_back(s);
    }
    if (subs.empty()) {
        std::fprintf(stderr, "xaq_voice: no modules subscribed\n");
        close(ctl);
        return 1;
    }
    const int diag_port = mods.contains("diag_port") ? mods["diag_port"].get<int>() : port + 1;

    // Unsubscribing on EVERY exit path is not hygiene, it is a hard requirement: a leaked
    // subscription costs the sim on every tick for the life of the process, and they stack
    // across restarts of this tool.
    auto unsubscribe_all = [&]() {
        for (const auto& s : subs) control_call(ctl, {{"verb", "unsubscribe"}, {"sub_id", s.sub_id}});
        subs.clear();
    };

    void* ctx = zmq_ctx_new();
    void* sub = zmq_socket(ctx, ZMQ_SUB);
    int   to = 200;
    zmq_setsockopt(sub, ZMQ_RCVTIMEO, &to, sizeof to);
    int hwm = 8;
    zmq_setsockopt(sub, ZMQ_RCVHWM, &hwm, sizeof hwm);      // never build a backlog
    for (const auto& s : subs) zmq_setsockopt(sub, ZMQ_SUBSCRIBE, s.prefix.data(), s.prefix.size());
    zmq_connect(sub, ("tcp://" + host + ":" + std::to_string(diag_port)).c_str());

    for (const auto& s : subs) engine.note_module_seen(s.module, s.type);

    const double real_hz = achievable_hz(hz);
    std::printf("xaq_voice: %zu module(s) on %s:%d, diag %d @ %.0f Hz%s\n", subs.size(),
                host.c_str(), port, diag_port, real_hz,
                std::fabs(real_hz - hz) > 0.5 ? "  (the brain decimates by whole ticks)" : "");

    // ---- control server ----
    ControlServer control(engine, bind_host, control_port);
    bool control_up = false;
    if (!no_control) {
        control.set_host_verb([&](const std::string& verb, const json& req) -> json {
            if (verb == "auto_patch") {
                Patch base = engine.patch();
                const Patch p = auto_patch(engine.observed(), base, req.value("vary", false));
                engine.set_patch(p);
                return json{{"status", "ok"}, {"patch", to_json(p)}};
            }
            if (verb == "save") {
                const std::string path = req.value("path", save_path);
                if (path.empty()) return json{{"status", "error"}, {"message", "no path given"}};
                std::string err;
                if (!write_patch(engine.patch(), path, err))
                    return json{{"status", "error"}, {"message", err}};
                return json{{"status", "ok"}, {"path", path}};
            }
            if (verb == "load") {
                const std::string path = req.value("path", "");
                Patch       p;
                std::string err;
                if (!read_patch(path, p, err)) return json{{"status", "error"}, {"message", err}};
                engine.set_patch(p);
                return json{{"status", "ok"}, {"patch", to_json(p)}};
            }
            return json();      // not handled
        });
        control.set_info({{"pid", int(getpid())},
                          {"brain_host", host},
                          {"brain_port", port},
                          {"modules", int(subs.size())}});
        control_up = control.start();
        if (control_up)
            std::printf("           studio on tcp://%s:%d  (meters on %d)\n", bind_host.c_str(),
                        control_port, control_port + 1);
        else
            // Keep playing — audio never needed a studio.  But say plainly that a studio
            // pointed here will reach the OTHER engine, because the symptom otherwise is
            // an operator's tuning changing under them with no visible cause.
            std::fprintf(stderr,
                         "xaq_voice: no control socket — another engine already holds %d.\n"
                         "           This one plays but cannot be tuned; a studio on %d will\n"
                         "           reach THAT engine.  Use --control-port to pick another.\n",
                         control_port, control_port);
    }

    std::thread ta(audio_thread, &engine, device, rate, no_audio);
    g_keys.engine    = &engine;
    g_keys.save_path = save_path;
    std::thread tk(keyboard_thread);

    // ---- discovery, then the patch ----
    const auto  t0 = std::chrono::steady_clock::now();
    bool        patched = false;
    std::vector<char> frame(1 << 20);
    auto        last_frame_t = std::chrono::steady_clock::now();
    auto        last_status  = std::chrono::steady_clock::now();

    while (g_run) {
        int n = zmq_recv(sub, frame.data(), frame.size() - 1, 0);
        if (n >= 0) {
            int    more = 0;
            size_t ml   = sizeof more;
            zmq_getsockopt(sub, ZMQ_RCVMORE, &more, &ml);
            if (more) n = zmq_recv(sub, frame.data(), frame.size() - 1, 0);
            if (n >= 0) {
                frame[std::min<size_t>(size_t(n), frame.size() - 1)] = 0;
                try {
                    const json j = json::parse(frame.data());
                    const int  sid = j.value("sub_id", -1);
                    const json& snap =
                        (j.contains("snapshot") && j["snapshot"].is_object()) ? j["snapshot"]
                                                                             : json::object();
                    for (const auto& s : subs) {
                        if (s.sub_id != sid) continue;
                        const auto now = std::chrono::steady_clock::now();
                        const double dt =
                            std::chrono::duration<double>(now - last_frame_t).count();
                        last_frame_t = now;
                        engine.on_frame(s.module, s.type, snap, std::clamp(dt, 1e-4, 1.0));
                        break;
                    }
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "xaq_voice: bad frame: %s\n", e.what());
                }
            }
        }

        if (!patched &&
            std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(discover_ms)) {
            patched = true;
            Patch p;
            if (!config_path.empty()) {
                std::string err;
                if (!read_patch(config_path, p, err)) {
                    std::fprintf(stderr, "xaq_voice: %s — falling back to auto-patch\n", err.c_str());
                    p = auto_patch(engine.observed(), Patch{}, vary);
                } else {
                    std::printf("           patch: %s\n", config_path.c_str());
                }
            } else {
                p = auto_patch(engine.observed(), Patch{}, vary);
            }
            apply_legacy(p, legacy);
            if (volume >= 0.0)  p.master.volume   = volume;
            if (raw_pitch)      p.master.quantize = false;
            p.brain.host = host;
            p.brain.port = port;
            p.brain.hz   = real_hz;
            engine.set_patch(p);

            std::printf("           %zu voice(s):", p.voices.size());
            for (const auto& v : p.voices)
                std::printf(" %s@%s", v.id.c_str(), hz_to_note(float(v.osc.base_hz)).c_str());
            std::printf("\n");
            // A module that was subscribed and said nothing is reported by name: that is a
            // missing diag_lite() override, and naming it is the difference between a
            // silent voice and a known gap.
            std::string silent;
            for (const auto& om : engine.observed())
                if (om.sources.empty()) silent += (silent.empty() ? "" : ", ") + om.module;
            if (!silent.empty())
                std::printf("           no diag_lite(): %s\n", silent.c_str());
            std::printf("keys: q quantise  t tone  +/- volume  m mute  1-9 voice  < > { } pitch  "
                        "w save  v voices  x quit\n");
            if (!save_path.empty()) {
                std::string err;
                if (write_patch(engine.patch(), save_path, err))
                    std::printf("           saved %s\n", save_path.c_str());
                else
                    std::fprintf(stderr, "xaq_voice: %s\n", err.c_str());
            }
            std::fflush(stdout);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!quiet && patched && now - last_status > std::chrono::seconds(1)) {
            last_status = now;
            const json st = engine.state_json();
            for (const auto& v : st["voices"])
                std::printf("  %-18s %7.1f Hz %-4s amp %.2f  cut %6.0f  q %4.1f\n",
                            v.value("id", "").c_str(), v.value("hz", 0.0),
                            v.value("note", "").c_str(), v.value("amp", 0.0),
                            v.value("cutoff", 0.0), v.value("q", 0.0));
            std::fflush(stdout);
        }
    }

    if (ta.joinable()) ta.join();
    if (tk.joinable()) tk.join();
    if (control_up) control.stop();
    unsubscribe_all();
    close(ctl);
    zmq_close(sub);
    zmq_ctx_term(ctx);
    return 0;
}
