// control.hpp — how the studio talks to the engine.
//
// Deliberately the SAME SHAPE as the brain's own inspector interface: a request/reply
// socket for commands, and a publish socket one port up for the stream of live numbers.
// That is not imitation for its own sake — it means the studio's transport is the
// inspector's transport (tools/xaq_inspector/transport.py) with a different port, and
// nobody has to write or debug a second client.
//
// Why the studio talks to the ENGINE and never to the brain: the engine is already
// subscribed to every module, so a second subscriber would double the sim's per-tick
// serialisation cost for nothing.  It also makes a leaked brain subscription
// structurally impossible from the GUI, and a leaked subscription costs the sim on every
// tick, forever, and they stack across restarts.
#pragma once

#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace xv {

class Engine;

class ControlServer {
  public:
    // Verbs the engine cannot answer alone — they need the brain connection main owns.
    // Return a null json to mean "not handled", which becomes an unknown-verb error.
    using HostVerb = std::function<nlohmann::json(const std::string& verb, const nlohmann::json& req)>;

    ControlServer(Engine& engine, const std::string& bind_host, int port, double state_hz = 15.0);
    ~ControlServer();

    void set_host_verb(HostVerb h) { host_verb_ = std::move(h); }

    // Returns false if either socket could not bind — the engine keeps running headless
    // rather than failing, exactly as the brain does with its own ports.
    bool start();
    void stop();

    int  port()       const { return port_; }
    int  state_port() const { return port_ + 1; }
    bool running()    const { return running_.load(); }

  private:
    void run();
    nlohmann::json dispatch(const nlohmann::json& req);

    Engine&           engine_;
    std::string       bind_host_;
    int               port_;
    double            state_hz_;
    HostVerb          host_verb_;
    void*             ctx_ = nullptr;
    void*             rep_ = nullptr;
    void*             pub_ = nullptr;
    std::thread       thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
};

}  // namespace xv
