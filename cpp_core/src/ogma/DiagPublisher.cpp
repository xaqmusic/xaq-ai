#include "ogma/DiagPublisher.hpp"

#include <cstring>
#include <iostream>

#include <zmq.h>
#include <nlohmann/json.hpp>

#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"

namespace ogma {

DiagPublisher::DiagPublisher(uint16_t port) : port_(port) {}

DiagPublisher::~DiagPublisher() { stop(); }

bool DiagPublisher::start() {
    if (running_.load()) return true;
    ctx_ = zmq_ctx_new();
    if (ctx_ == nullptr) {
        std::cerr << "DiagPublisher: zmq_ctx_new failed\n";
        return false;
    }
    sock_ = zmq_socket(ctx_, ZMQ_PUB);
    if (sock_ == nullptr) {
        std::cerr << "DiagPublisher: zmq_socket failed\n";
        zmq_ctx_term(ctx_);
        ctx_ = nullptr;
        return false;
    }
    int hwm = 64;
    zmq_setsockopt(sock_, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    char addr[64];
    std::snprintf(addr, sizeof(addr), "tcp://127.0.0.1:%u", port_);
    if (zmq_bind(sock_, addr) != 0) {
        std::cerr << "DiagPublisher: zmq_bind " << addr << " failed: "
                  << zmq_strerror(zmq_errno()) << "\n";
        zmq_close(sock_); sock_ = nullptr;
        zmq_ctx_term(ctx_); ctx_ = nullptr;
        return false;
    }
    running_.store(true);
    std::cout << "DiagPublisher PUB bound on " << addr << "\n";
    return true;
}

void DiagPublisher::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (sock_) { zmq_close(sock_); sock_ = nullptr; }
    if (ctx_)  { zmq_ctx_term(ctx_); ctx_  = nullptr; }
    std::lock_guard lk(subs_mtx_);
    subs_.clear();
}

int DiagPublisher::subscribe(std::string module_id, std::string topic, double hz) {
    std::lock_guard lk(subs_mtx_);
    int id = next_id_++;
    Subscription s;
    s.id        = id;
    s.module_id = std::move(module_id);
    s.topic     = std::move(topic);
    s.hz        = hz > 0.0 ? hz : 30.0;
    subs_.emplace(id, std::move(s));
    return id;
}

void DiagPublisher::unsubscribe(int sub_id) {
    std::lock_guard lk(subs_mtx_);
    subs_.erase(sub_id);
}

std::vector<DiagPublisher::Subscription>
DiagPublisher::active_subscriptions() const {
    std::lock_guard lk(subs_mtx_);
    std::vector<Subscription> out;
    out.reserve(subs_.size());
    for (auto const& [_, s] : subs_) out.push_back(s);
    return out;
}

void DiagPublisher::publish_tick(uint64_t tick_id, OgmaInstance& instance) {
    if (!running_.load() || sock_ == nullptr) return;
    std::lock_guard lk(subs_mtx_);
    for (auto& [id, s] : subs_) {
        // Decide whether to fire this tick: every N ticks where
        // N = round(host_tick_hz / s.hz), clamped to >= 1.
        int interval = int(host_tick_hz_ / s.hz + 0.5);
        if (interval < 1) interval = 1;
        if (s.have_pubbed && (tick_id - s.last_pub_tick) < uint64_t(interval))
            continue;
        auto* m = instance.module(s.module_id);
        if (m == nullptr) continue;
        nlohmann::json payload = {
            {"sub_id",    s.id},
            {"module_id", s.module_id},
            {"topic",     s.topic},
            {"tick_id",   tick_id},
            {"snapshot",  s.topic == "lite" ? m->diag_lite() : m->diag_snapshot()},
        };
        std::string topic_str = "diag." + std::to_string(s.id) + ".";
        std::string body = payload.dump();
        // Two-frame ZMQ message: topic prefix, then body.
        zmq_send(sock_, topic_str.c_str(), topic_str.size(), ZMQ_SNDMORE | ZMQ_DONTWAIT);
        zmq_send(sock_, body.c_str(),      body.size(),      ZMQ_DONTWAIT);
        s.last_pub_tick = tick_id;
        s.have_pubbed   = true;
    }
}

} // namespace ogma
