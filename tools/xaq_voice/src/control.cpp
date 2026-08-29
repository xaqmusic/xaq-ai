#include "control.hpp"

#include <zmq.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "engine.hpp"

namespace xv {

using json = nlohmann::json;

ControlServer::ControlServer(Engine& engine, const std::string& bind_host, int port,
                             double state_hz)
    : engine_(engine), bind_host_(bind_host), port_(port), state_hz_(state_hz) {}

ControlServer::~ControlServer() { stop(); }

bool ControlServer::start() {
    ctx_ = zmq_ctx_new();
    if (!ctx_) return false;
    rep_ = zmq_socket(ctx_, ZMQ_REP);
    pub_ = zmq_socket(ctx_, ZMQ_PUB);
    if (!rep_ || !pub_) return false;

    int linger = 0;
    zmq_setsockopt(rep_, ZMQ_LINGER, &linger, sizeof linger);
    zmq_setsockopt(pub_, ZMQ_LINGER, &linger, sizeof linger);
    // Never build a backlog: a studio that stalls must fall behind and catch up on the
    // newest state, not replay a queue of stale meters.
    int hwm = 8;
    zmq_setsockopt(pub_, ZMQ_SNDHWM, &hwm, sizeof hwm);

    const std::string rep_ep = "tcp://" + bind_host_ + ":" + std::to_string(port_);
    const std::string pub_ep = "tcp://" + bind_host_ + ":" + std::to_string(port_ + 1);
    if (zmq_bind(rep_, rep_ep.c_str()) != 0 || zmq_bind(pub_, pub_ep.c_str()) != 0) {
        std::fprintf(stderr, "xaq_voice: control sockets could not bind %s / %s (%s)\n",
                     rep_ep.c_str(), pub_ep.c_str(), zmq_strerror(zmq_errno()));
        zmq_close(rep_); zmq_close(pub_); zmq_ctx_term(ctx_);
        rep_ = pub_ = ctx_ = nullptr;
        return false;
    }

    stop_    = false;
    running_ = true;
    thread_  = std::thread(&ControlServer::run, this);
    return true;
}

void ControlServer::stop() {
    if (!running_.load()) return;
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (rep_) { zmq_close(rep_); rep_ = nullptr; }
    if (pub_) { zmq_close(pub_); pub_ = nullptr; }
    if (ctx_) { zmq_ctx_term(ctx_); ctx_ = nullptr; }
    running_ = false;
}

void ControlServer::run() {
    const auto   period = std::chrono::duration<double>(1.0 / std::max(1.0, state_hz_));
    auto         next   = std::chrono::steady_clock::now();
    std::vector<char> buf(1 << 20);

    while (!stop_.load()) {
        // REP is strictly alternating, so a reply is sent for EVERY request received —
        // including malformed ones.  Skipping a reply would wedge the client's REQ socket
        // permanently, which reads to the operator as "the studio froze".
        const int n = zmq_recv(rep_, buf.data(), buf.size() - 1, ZMQ_DONTWAIT);
        if (n >= 0) {
            buf[std::min<size_t>(size_t(n), buf.size() - 1)] = 0;
            json reply;
            try {
                const json req = json::parse(buf.data());
                reply = dispatch(req);
            } catch (const std::exception& e) {
                reply = {{"status", "error"}, {"message", std::string("bad request: ") + e.what()}};
            }
            const std::string s = reply.dump();
            zmq_send(rep_, s.data(), s.size(), 0);
            continue;                       // drain pending requests before sleeping
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next) {
            next = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            try {
                const std::string body = engine_.state_json().dump();
                const char* topic = "state";
                zmq_send(pub_, topic, std::strlen(topic), ZMQ_SNDMORE | ZMQ_DONTWAIT);
                zmq_send(pub_, body.data(), body.size(), ZMQ_DONTWAIT);
            } catch (const std::exception&) {
                // A meter frame is never worth taking the engine down for.
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

json ControlServer::dispatch(const json& req) {
    const std::string verb = req.value("verb", "");

    if (verb == "ping") return {{"status", "ok"}, {"engine", "xaq_voice"}};

    // The studio asks the engine what it can do rather than hardcoding the lists, so a
    // waveform or filter mode added here reaches the UI without a matching Python edit.
    if (verb == "hello") {
        return {{"status", "ok"},
                {"engine", "xaq_voice"},
                {"version", 1},
                {"waveforms", wave_names()},
                {"filter_modes", filter_mode_names()},
                {"destinations", dest_names()},
                {"norm_modes", norm_mode_names()},
                {"triggers", trigger_names()},
                {"event_sounds", event_sound_names()},
                {"vowels", vowel_names()},
                {"scales", scale_names()},
                {"state_port", port_ + 1}};
    }

    if (verb == "get_patch") return {{"status", "ok"}, {"patch", to_json(engine_.patch())}};

    if (verb == "set_patch") {
        if (!req.contains("patch") || !req["patch"].is_object())
            return {{"status", "error"}, {"message", "set_patch requires a 'patch' object"}};
        engine_.set_patch(from_json(req["patch"]));
        return {{"status", "ok"}};
    }

    if (verb == "patch") {
        std::string err;
        std::lock_guard<std::mutex> lk(engine_.mtx_);
        if (!engine_.apply_ops_locked(req.value("ops", json::array()), err))
            return {{"status", "error"}, {"message", err}};
        return {{"status", "ok"}};
    }

    if (verb == "get_sources") return {{"status", "ok"}, {"modules", engine_.sources_json()}};
    if (verb == "get_state")   return {{"status", "ok"}, {"state", engine_.state_json()}};

    if (verb == "set_mute") { engine_.set_mute(req.value("value", false)); return {{"status", "ok"}}; }
    if (verb == "set_tone") { engine_.set_tone_enabled(req.value("value", true)); return {{"status", "ok"}}; }

    if (host_verb_) {
        const json r = host_verb_(verb, req);
        if (!r.is_null()) return r;
    }
    return {{"status", "error"}, {"message", "unknown verb: " + verb}};
}

}  // namespace xv
