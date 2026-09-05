#pragma once
// A client for the hosts' control socket: TCP, one newline-delimited JSON
// request → one JSON reply, the same protocol tools/xaq_inspector speaks.
// One connection, its own; the inspector keeps its own.
#include <cstdint>
#include <mutex>
#include <string>
#include <nlohmann/json.hpp>

namespace bb {

class LiveClient {
public:
    ~LiveClient();
    bool connect(std::string const& host, uint16_t port, std::string* err);
    void close();
    bool connected() const { return fd_ >= 0; }
    std::string endpoint() const { return host_ + ":" + std::to_string(port_); }
    // Transport failure → {"status":"error","message":...} and the socket is closed.
    nlohmann::json call(nlohmann::json const& req);

private:
    int         fd_ = -1;
    std::string host_;
    uint16_t    port_ = 0;
    std::string buf_;
    std::mutex  mtx_;
};

// "host:port" or "port" → (host, port); "" → 127.0.0.1:7400.
bool parse_endpoint(std::string const& text, std::string& host, uint16_t& port);

} // namespace bb
