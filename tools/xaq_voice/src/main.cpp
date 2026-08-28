// xaq_voice — sonification of the brain's error signal.
//
// An INSTRUMENT, not a behaviour: the brain does not control this and cannot hear it.
// It subscribes to the diag stream (control socket: newline JSON on OGMA_INSPECTOR_PORT,
// diag: ZMQ PUB 2-part frames on port+1 — exactly what tools/xaq_inspector reads), so
// it costs the brain nothing and works unchanged against the sim and against ogma_host.
//
//   pitch   <- TLE, normalised by its own running median/MAD (so the mapping survives
//              the GNG baking and the scale of any module), on a log scale:
//              0..2 octaves above the voice's base as z goes 0..4.  Quantised to a
//              major pentatonic (voices an octave apart form chords) or RAW (continuous).
//   volume  <- TLE relative to the novelty threshold (GNG EPMs publish
//              novelty_threshold_now; MotorEPMv2 has none, so median + 1 MAD): silent
//              until tle > GATE x threshold (1.4), full at FULL x threshold (2.0),
//              x^gamma between (0.5; tuned by ear by the operator 2026-08-28).  Gate,
//              full and gamma are live-adjustable.  Silence = "I know this".
//   chirps  <- GNG life events: a node BAKING (rising chirp), MITOSIS (two notes).
//   voices  <- one square-wave oscillator per subscribed module, each on its own octave.
//
// Quiet by default: one header line + the key hint.  --verbose adds the per-second
// per-voice numbers and the event lines (bake / mitosis / prune).
// Keys while running:  q quantised/raw  t TLE tone  b bake  n mitosis  p prune  +/- volume  m mute  g gamma  x quit
#include <alsa/asoundlib.h>
#include <nlohmann/json.hpp>
#include <zmq.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <map>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>

using json = nlohmann::json;

namespace {

// Snapshots are hand-built per module; read numbers tolerantly (a bool or a string
// where a number was expected must never take the voice down).
double num(const json& j, const char* key, double def) {
    auto it = j.find(key); return (it != j.end() && it->is_number()) ? it->get<double>() : def;
}

// ---------------------------------------------------------------- running stats
struct RunningStats {           // median + MAD over a sliding window (seconds of ticks)
    std::deque<float> win; size_t cap;
    explicit RunningStats(size_t cap_) : cap(cap_) {}
    void push(float v) { win.push_back(v); if (win.size() > cap) win.pop_front(); }
    bool ready() const { return win.size() >= 25; }
    float median() const {
        std::vector<float> t(win.begin(), win.end()); size_t k = t.size() / 2;
        std::nth_element(t.begin(), t.begin() + k, t.end()); return t[k];
    }
    float mad() const {
        float m = median(); std::vector<float> t; t.reserve(win.size());
        for (float v : win) t.push_back(std::fabs(v - m));
        size_t k = t.size() / 2; std::nth_element(t.begin(), t.begin() + k, t.end());
        return std::max(t[k], 1e-6f);
    }
};

// ---------------------------------------------------------------- a voice
struct Voice {
    std::string module, type, prefix;
    int sub_id = -1;
    std::atomic<float> base_hz{130.81f};     // C3; tunable live
    RunningStats stats{500};                 // ~10 s at 50 Hz
    // shared with the audio thread
    std::atomic<float> target_hz{130.81f}, target_amp{0.f};
    std::atomic<int>   chirp{0};             // 1 = bake, 2 = mitosis (consumed by audio)
    // last seen, for status + event detection
    float tle = 0.f, z = 0.f, z_s = 0.f, thresh = 0.f, margin = 0.f, ratio = 0.f;
    float note_semis = 0.f;                    // current quantised note (hysteresis)
    int mitosis_count = -1; int nodes = 0, baked = 0;
    // audio-side state
    double phase = 0.0, hz_now = 130.81, amp_now = 0.0;
    double chirp_t = -1.0; int chirp_kind = 0;
};

std::atomic<bool> g_run{true}, g_quantize{true}, g_mute{false};
std::atomic<bool> g_tone{true}, g_bake{true}, g_mitosis{true}, g_prune{true};   // per-event toggles
void on_signal(int) { g_run = false; }   // always reach the unsubscribe path
std::atomic<float> g_master{0.5f};
std::atomic<float> g_gamma{0.5f};     // volume curve exponent (operator-tuned 2026-08-28)
std::atomic<float> g_gate{1.4f};      // silent until tle > gate * threshold
std::atomic<float> g_full{2.0f};      // full volume at tle >= full * threshold
std::atomic<float> g_span{24.f};      // semitones of pitch range above base (z 0..4)
std::atomic<int>   g_sel{0};          // voice selected for live tuning (1-9 keys)
struct Voice; std::vector<Voice*>* g_voices = nullptr;   // for the tuning keys
void print_tuning();

// note names <-> Hz (A4 = 440).  "C3", "A#4", "Db5", or a plain number of Hz.
float note_to_hz(const std::string& t) {
    if (t.empty()) return 0.f;
    if (std::isdigit((unsigned char)t[0])) return std::stof(t);
    static const int idx[7] = {9, 11, 0, 2, 4, 5, 7};     // A B C D E F G
    char L = char(std::toupper((unsigned char)t[0])); if (L < 'A' || L > 'G') return 0.f;
    int semi = idx[L - 'A']; size_t i = 1;
    if (i < t.size() && (t[i] == '#' || t[i] == 's')) { ++semi; ++i; } else if (i < t.size() && t[i] == 'b') { --semi; ++i; }
    int oct = std::atoi(t.c_str() + i);
    return 440.f * std::pow(2.f, ((oct + 1) * 12 + semi - 69) / 12.f);
}
std::string hz_to_note(float hz) {
    static const char* N[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    int midi = int(std::lround(69 + 12 * std::log2(hz / 440.f)));
    char b[16]; std::snprintf(b, sizeof b, "%s%d", N[((midi % 12) + 12) % 12], midi / 12 - 1); return b;
}

const int PENTA[] = {0, 2, 4, 7, 9};        // major pentatonic
float quantize_semis(float s) {
    int oct = int(std::floor(s / 12.f)); float r = s - oct * 12.f; float best = 0; float bd = 99;
    for (int p : PENTA) { float d = std::fabs(r - p); if (d < bd) { bd = d; best = p; } }
    if (std::fabs(r - 12.f) < bd) { best = 12.f; }
    return oct * 12.f + best;
}

// ---------------------------------------------------------------- mapping (network thread)
void map_voice(Voice& v, float tle, float thresh_from_module, bool has_thresh) {
    v.stats.push(tle);
    v.tle = tle;
    if (!v.stats.ready()) { v.target_amp = 0.f; return; }
    const float med = v.stats.median(), mad = v.stats.mad();
    v.z = (tle - med) / mad;
    v.z_s += 0.3f * (v.z - v.z_s);                                     // ~60 ms: keeps spikes, kills tick jitter
    v.thresh = has_thresh ? thresh_from_module : med + mad;
    v.ratio  = v.thresh > 1e-9f ? tle / v.thresh : 0.f;
    v.margin = v.ratio - 1.f;
    // pitch from the SMOOTHED surprise; quantised notes only change past a half-step + hysteresis
    float semis = g_span.load() * std::clamp(v.z_s / 4.f, 0.f, 1.f);
    if (g_quantize) {
        const float q = quantize_semis(semis);
        if (std::fabs(q - v.note_semis) >= 1.f || std::fabs(semis - v.note_semis) > 1.6f) v.note_semis = q;
        semis = v.note_semis;
    }
    v.target_hz = v.base_hz.load() * std::pow(2.f, semis / 12.f);
    // volume: silent below gate*threshold, full at full*threshold, steep curve in between
    const float gate = g_gate.load(), full = std::max(g_full.load(), gate + 0.1f);
    const float x = std::clamp((v.ratio - gate) / (full - gate), 0.f, 1.f);
    v.target_amp = std::pow(x, g_gamma.load());
}

// ---------------------------------------------------------------- audio thread
float poly_blep(double t, double dt) {          // anti-aliased step for the square
    if (t < dt) { t /= dt; return float(t + t - t * t - 1.0); }
    if (t > 1.0 - dt) { t = (t - 1.0) / dt; return float(t * t + t + t + 1.0); }
    return 0.f;
}

void audio_thread(std::vector<Voice*>* voices, std::mutex* vm, const std::string& device, int rate, bool no_audio) {
    snd_pcm_t* pcm = nullptr;
    if (!no_audio) {
        if (snd_pcm_open(&pcm, device.c_str(), SND_PCM_STREAM_PLAYBACK, 0) < 0 ||
            snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 1, rate, 1, 40000) < 0) {
            std::fprintf(stderr, "xaq_voice: ALSA open/setup failed on '%s' — running silent (--no-audio to hide this)\n", device.c_str());
            pcm = nullptr;
        }
    }
    const int N = 256; std::vector<int16_t> buf(N);
    const double dt_s = 1.0 / rate;
    const double glide = std::exp(-dt_s / 0.030), attack = std::exp(-dt_s / 0.020), release = std::exp(-dt_s / 0.150);
    while (g_run) {
        std::fill(buf.begin(), buf.end(), 0);
        {
            std::lock_guard<std::mutex> lk(*vm);
            const float master = g_mute ? 0.f : g_master.load();
            for (Voice* v : *voices) {
                const double th = v->target_hz.load(); const double ta = v->target_amp.load();
                int ck = v->chirp.exchange(0);
                if (ck) { v->chirp_kind = ck; v->chirp_t = 0.0; }
                for (int i = 0; i < N; ++i) {
                    v->hz_now = th + (v->hz_now - th) * glide;
                    v->amp_now = ta > v->amp_now ? ta + (v->amp_now - ta) * attack : ta + (v->amp_now - ta) * release;
                    double hz = v->hz_now, amp = g_tone ? v->amp_now : 0.0;
                    if (v->chirp_t >= 0.0) {                         // overlay a chirp
                        const double T = v->chirp_kind == 1 ? 0.08 : (v->chirp_kind == 2 ? 0.16 : 0.06);
                        double u = v->chirp_t / T;
                        if (v->chirp_kind == 1) hz = v->base_hz.load() * 2.0 * std::pow(2.0, u);                 // bake: rising octave
                        else if (v->chirp_kind == 2) hz = v->base_hz.load() * (u < 0.5 ? 3.0 : 4.0);             // mitosis: two notes
                        else hz = v->base_hz.load() * 0.5;                                                     // prune: low blip
                        amp = std::max(amp, 0.45 * (1.0 - u));
                        v->chirp_t += dt_s; if (v->chirp_t > T) v->chirp_t = -1.0;
                    }
                    const double dtp = hz * dt_s;
                    v->phase += dtp; if (v->phase >= 1.0) v->phase -= 1.0;
                    float s = v->phase < 0.5 ? 1.f : -1.f;
                    s += poly_blep(v->phase, dtp);
                    double t2 = v->phase + 0.5; if (t2 >= 1.0) t2 -= 1.0;
                    s -= poly_blep(t2, dtp);
                    float mixed = buf[i] / 32767.f + s * float(amp) * master * 0.25f;
                    mixed = std::tanh(mixed);                                                      // soft clip
                    buf[i] = int16_t(std::clamp(mixed, -1.f, 1.f) * 32767.f);
                }
            }
        }
        if (pcm) {
            snd_pcm_sframes_t w = snd_pcm_writei(pcm, buf.data(), N);
            if (w < 0) snd_pcm_recover(pcm, int(w), 1);
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(N * 1000000 / rate));
        }
    }
    if (pcm) { snd_pcm_drain(pcm); snd_pcm_close(pcm); }
}

// ---------------------------------------------------------------- control socket (newline JSON)
int tcp_connect(const std::string& host, int port) {
    addrinfo hints{}; hints.ai_family = AF_UNSPEC; hints.ai_socktype = SOCK_STREAM; addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) return -1;
    int fd = -1;
    for (addrinfo* p = res; p; p = p->ai_next) {
        fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd >= 0 && ::connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
        if (fd >= 0) { close(fd); fd = -1; }
    }
    freeaddrinfo(res); return fd;
}
json control_call(int fd, const json& req) {
    std::string s = req.dump() + "\n";
    if (send(fd, s.data(), s.size(), 0) < 0) return {{"status", "error"}, {"message", "send"}};
    std::string buf; char c;
    while (recv(fd, &c, 1, 0) == 1) { if (c == '\n') break; buf += c; }
    try { return json::parse(buf); } catch (...) { return {{"status", "error"}, {"message", "bad json"}}; }
}

void print_tuning() {
    if (!g_voices) return;
    std::printf("  --base ");
    for (size_t i = 0; i < g_voices->size(); ++i) std::printf("%s%s=%s", i ? "," : "", (*g_voices)[i]->module.c_str(), hz_to_note((*g_voices)[i]->base_hz.load()).c_str());
    std::printf("   --span %.0f\n", g_span.load());
}
void retune(float semis) {
    if (!g_voices || g_voices->empty()) return;
    Voice* v = (*g_voices)[std::clamp(g_sel.load(), 0, int(g_voices->size()) - 1)];
    v->base_hz = v->base_hz.load() * std::pow(2.f, semis / 12.f);
    std::printf("[voice %d %s base -> %s %.1f Hz]\n", g_sel.load() + 1, v->module.c_str(), hz_to_note(v->base_hz.load()).c_str(), v->base_hz.load());
    print_tuning();
}
void keyboard_thread() {
    termios old{}; bool tty = isatty(0);
    if (tty) { tcgetattr(0, &old); termios raw = old; raw.c_lflag &= ~(ICANON | ECHO); tcsetattr(0, TCSANOW, &raw); }
    while (g_run) {
        char c; if (read(0, &c, 1) != 1) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
        if (c == 'q') { g_quantize = !g_quantize; std::printf("[pitch %s]\n", g_quantize ? "QUANTISED (pentatonic)" : "RAW"); }
        else if (c == '+' || c == '=') { g_master = std::min(1.f, g_master + 0.1f); std::printf("[master %.1f]\n", g_master.load()); }
        else if (c == '-') { g_master = std::max(0.f, g_master - 0.1f); std::printf("[master %.1f]\n", g_master.load()); }
        else if (c == 'm') { g_mute = !g_mute; std::printf("[%s]\n", g_mute ? "MUTE" : "unmute"); }
        else if (c == ',') { g_gate = std::max(1.0f, g_gate - 0.1f); std::printf("[gate %.1fx threshold]\n", g_gate.load()); }
        else if (c == '.') { g_gate = g_gate + 0.1f; std::printf("[gate %.1fx threshold]\n", g_gate.load()); }
        else if (c == ';') { g_full = std::max(g_gate + 0.1f, g_full - 0.5f); std::printf("[full volume at %.1fx threshold]\n", g_full.load()); }
        else if (c == '\'') { g_full = g_full + 0.5f; std::printf("[full volume at %.1fx threshold]\n", g_full.load()); }
        else if (c == '[') { g_gamma = std::max(0.25f, g_gamma - 0.25f); std::printf("[gamma %.2f]\n", g_gamma.load()); }
        else if (c == ']') { g_gamma = g_gamma + 0.25f; std::printf("[gamma %.2f]\n", g_gamma.load()); }
        else if (c == 't') { g_tone = !g_tone; std::printf("[TLE tone %s]\n", g_tone ? "on" : "OFF"); }
        else if (c == 'b') { g_bake = !g_bake; std::printf("[bake chirps %s]\n", g_bake ? "on" : "OFF"); }
        else if (c == 'n') { g_mitosis = !g_mitosis; std::printf("[mitosis chirps %s]\n", g_mitosis ? "on" : "OFF"); }
        else if (c == 'p') { g_prune = !g_prune; std::printf("[prune blips %s]\n", g_prune ? "on" : "OFF"); }
        else if (c >= '1' && c <= '9') { g_sel = c - '1'; if (g_voices && g_sel < int(g_voices->size())) std::printf("[tuning voice %d: %s, base %s]\n", g_sel.load() + 1, (*g_voices)[g_sel]->module.c_str(), hz_to_note((*g_voices)[g_sel]->base_hz.load()).c_str()); else g_sel = 0; }
        else if (c == '<') retune(-1.f);  else if (c == '>') retune(+1.f);
        else if (c == '{') retune(-12.f); else if (c == '}') retune(+12.f);
        else if (c == '(') { g_span = std::max(6.f, g_span - 6.f); std::printf("[span %.0f semitones]\n", g_span.load()); print_tuning(); }
        else if (c == ')') { g_span = std::min(48.f, g_span + 6.f); std::printf("[span %.0f semitones]\n", g_span.load()); print_tuning(); }
        else if (c == 3 || c == 'x') { g_run = false; }
        std::fflush(stdout);
    }
    if (tty) tcsetattr(0, TCSANOW, &old);
}

} // namespace

int main(int argc, char** argv) {
    std::string host = "127.0.0.1", device = "default", modules_arg;
    std::map<std::string, float> base_override;          // --base id=note|hz,...
    int port = 7400, rate = 48000; double hz = 50.0; bool no_audio = false, quiet = true;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i]; auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (a == "--host") host = next(); else if (a == "--port") port = std::stoi(next());
        else if (a == "--device") device = next(); else if (a == "--rate") rate = std::stoi(next());
        else if (a == "--hz") hz = std::stod(next()); else if (a == "--modules") modules_arg = next();
        else if (a == "--raw") g_quantize = false; else if (a == "--no-audio") no_audio = true;
        else if (a == "--verbose") quiet = false; else if (a == "--quiet") quiet = true;
        else if (a == "--volume") g_master = std::stof(next());
        else if (a == "--gate") g_gate = std::stof(next()); else if (a == "--full") g_full = std::stof(next());
        else if (a == "--gamma") g_gamma = std::stof(next());
        else if (a == "--span") g_span = std::stof(next());
        else if (a == "--base") {
            std::string b = next(); size_t st = 0;
            while (st <= b.size()) {
                size_t e = b.find(',', st); if (e == std::string::npos) e = b.size();
                std::string kv = b.substr(st, e - st); size_t eq = kv.find('=');
                if (eq != std::string::npos) { float hz = note_to_hz(kv.substr(eq + 1)); if (hz > 0) base_override[kv.substr(0, eq)] = hz; }
                st = e + 1;
            }
        }
        else { std::fprintf(stderr, "usage: xaq_voice [--host H] [--port 7400] [--device default] [--modules a,b] [--hz 50] [--raw] [--volume 0.5] [--gate 1.4] [--full 2.0] [--gamma 0.5] [--span 24] [--base id=C3,id=A4,...] [--no-audio] [--verbose]\n"); return 2; }
    }
    if (const char* e = std::getenv("OGMA_INSPECTOR_PORT")) if (port == 7400) port = std::atoi(e);
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGHUP, on_signal);

    int ctl = tcp_connect(host, port);
    if (ctl < 0) { std::fprintf(stderr, "xaq_voice: cannot reach control socket %s:%d (is the brain running?)\n", host.c_str(), port); return 1; }
    json mods = control_call(ctl, {{"verb", "list_modules"}});
    if (mods.value("status", "") != "ok") { std::fprintf(stderr, "xaq_voice: list_modules failed: %s\n", mods.dump().c_str()); return 1; }

    // Choose voices: explicit --modules, else every module that carries a TLE.
    std::vector<Voice*> voices; std::mutex vm;
    std::vector<std::string> wanted;
    if (!modules_arg.empty()) { size_t s = 0; while (s <= modules_arg.size()) { size_t e = modules_arg.find(',', s); if (e == std::string::npos) e = modules_arg.size(); if (e > s) wanted.push_back(modules_arg.substr(s, e - s)); s = e + 1; } }
    const float OCTAVES[] = {130.81f, 261.63f, 523.25f, 1046.5f, 65.41f, 2093.0f};   // C3, C4, C5, C6, C2, C7
    int nv = 0;
    for (auto& m : mods["modules"]) {
        const std::string id = m.value("id", ""), type = m.value("type", "");
        const bool tle_type = (type.find("EPM") != std::string::npos || type == "SequenceGNG");
        if (!wanted.empty() ? std::find(wanted.begin(), wanted.end(), id) == wanted.end() : !tle_type) continue;
        json r = control_call(ctl, {{"verb", "module_subscribe_diag"}, {"id", id}, {"topic", "lite"}, {"hz", hz}});
        if (r.value("status", "") != "ok") { std::fprintf(stderr, "xaq_voice: subscribe %s failed: %s\n", id.c_str(), r.dump().c_str()); continue; }
        Voice* v = new Voice; v->module = id; v->type = type; v->sub_id = r.value("sub_id", -1);
        v->prefix = r.value("topic_prefix", "diag." + std::to_string(v->sub_id) + ".");
        v->base_hz = base_override.count(id) ? base_override[id] : OCTAVES[std::min(nv, 5)]; v->target_hz = v->base_hz.load(); ++nv;
        voices.push_back(v);
        if (!quiet) std::printf("voice %d: %-22s %-12s base %s %.0f Hz  (%s)\n", nv, id.c_str(), type.c_str(), hz_to_note(v->base_hz.load()).c_str(), v->base_hz.load(), v->prefix.c_str());
    }
    if (voices.empty()) { std::fprintf(stderr, "xaq_voice: no TLE-carrying modules found (use --modules)\n"); return 1; }
    const int diag_port = mods.contains("diag_port") ? mods["diag_port"].get<int>() : port + 1;

    void* ctx = zmq_ctx_new(); void* sub = zmq_socket(ctx, ZMQ_SUB);
    int to = 200; zmq_setsockopt(sub, ZMQ_RCVTIMEO, &to, sizeof to);
    int hwm = 8;  zmq_setsockopt(sub, ZMQ_RCVHWM, &hwm, sizeof hwm);      // never build a backlog
    for (Voice* v : voices) zmq_setsockopt(sub, ZMQ_SUBSCRIBE, v->prefix.data(), v->prefix.size());
    zmq_connect(sub, ("tcp://" + host + ":" + std::to_string(diag_port)).c_str());
    std::printf("xaq_voice: %zu voice(s) [", voices.size());
    for (size_t i = 0; i < voices.size(); ++i) std::printf("%s%zu:%s@%s", i ? ", " : "", i + 1, voices[i]->module.c_str(), hz_to_note(voices[i]->base_hz.load()).c_str());
    std::printf("]  pitch %s  master %.1f  gate %.1fx  full %.1fx  gamma %.2f\n"
                "keys: q quantised/raw   t tone  b bake  n mitosis  p prune   , . gate   ; ' full   [ ] gamma   +/- volume   m mute   x quit\n"
                "tune: 1-9 select voice   < > semitone   { } octave   ( ) span%s\n",
                g_quantize ? "quantised" : "raw", g_master.load(), g_gate.load(), g_full.load(), g_gamma.load(), quiet ? "   (--verbose for the live numbers)" : "");
    std::fflush(stdout);

    g_voices = &voices;
    std::thread ta(audio_thread, &voices, &vm, device, rate, no_audio);
    std::thread tk(keyboard_thread);
    auto last_status = std::chrono::steady_clock::now();
    char frame[1 << 20];
    while (g_run) {
        int n = zmq_recv(sub, frame, sizeof frame - 1, 0);
        if (n < 0) continue;
        int more = 0; size_t ml = sizeof more; zmq_getsockopt(sub, ZMQ_RCVMORE, &more, &ml);
        if (more) { n = zmq_recv(sub, frame, sizeof frame - 1, 0); if (n < 0) continue; }
        frame[std::min(n, int(sizeof frame) - 1)] = 0;
        json j; try { j = json::parse(frame); } catch (...) { continue; }
        const int sid = int(num(j, "sub_id", -1));
        const json& s = (j.contains("snapshot") && j["snapshot"].is_object()) ? j["snapshot"] : json::object();
        std::lock_guard<std::mutex> lk(vm);
        try {
        for (Voice* v : voices) {
            if (v->sub_id != sid) continue;
            if (s.contains("motor_tle")) {                       // MotorEPM/v2: no novelty threshold
                map_voice(*v, float(num(s, "motor_tle", 0.0)), 0.f, false);
            } else {
                map_voice(*v, float(num(s, "last_tle", 0.0)), float(num(s, "novelty_threshold_now", 0.0)), s.contains("novelty_threshold_now") && s["novelty_threshold_now"].is_number());
                // lite payload: nodes/baked/mitosis_count/baked_now are top-level scalars
                const json& g = s.contains("gng") && s["gng"].is_object() ? s["gng"] : s;
                int mc = int(num(g, "mitosis_count", 0));
                if (v->mitosis_count >= 0 && mc > v->mitosis_count) { if (g_mitosis) v->chirp = 2; if (!quiet) std::printf("  ♪ %s MITOSIS (%d)\n", v->module.c_str(), mc); }
                v->mitosis_count = mc;
                auto bn = g.find("baked_now");
                if (bn != g.end() && bn->is_boolean() && bn->get<bool>()) { if (g_bake) v->chirp = 1; if (!quiet) std::printf("  ♪ %s bake\n", v->module.c_str()); }
                const int nodes_now = int(num(g, "nodes", 0));
                if (v->nodes > 0 && nodes_now < v->nodes) { if (g_prune) v->chirp = 3; if (!quiet) std::printf("  ♪ %s prune (%d -> %d)\n", v->module.c_str(), v->nodes, nodes_now); }
                v->nodes = nodes_now;
                v->baked = int(num(g, "baked", 0));
            }
        }
        } catch (const std::exception& e) { std::fprintf(stderr, "xaq_voice: bad frame: %s\n", e.what()); }
        auto now = std::chrono::steady_clock::now();
        if (!quiet && now - last_status > std::chrono::seconds(1)) {
            last_status = now;
            for (Voice* v : voices)
                std::printf("  %-18s tle %.3f  z %+5.2f  thr %.3f  ratio %4.2fx  -> %6.1f Hz  amp %.2f%s\n", v->module.c_str(), v->tle, v->z, v->thresh, v->ratio,
                            v->target_hz.load(), v->target_amp.load(), v->nodes ? (" nodes " + std::to_string(v->nodes) + "/" + std::to_string(v->baked) + " baked").c_str() : "");
            std::fflush(stdout);
        }
    }
    ta.join(); tk.join();
    for (Voice* v : voices) control_call(ctl, {{"verb", "unsubscribe"}, {"sub_id", v->sub_id}});
    close(ctl); zmq_close(sub); zmq_ctx_term(ctx);
    for (Voice* v : voices) delete v;
    return 0;
}
