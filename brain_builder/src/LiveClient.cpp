#include "LiveClient.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace bb {

bool parse_endpoint(std::string const& text, std::string& host, uint16_t& port) {
    host = "127.0.0.1"; port = 7400;
    if (text.empty()) return true;
    auto colon = text.rfind(':');
    std::string p = colon == std::string::npos ? text : text.substr(colon + 1);
    if (colon != std::string::npos && colon > 0) host = text.substr(0, colon);
    try { int v = std::stoi(p); if (v <= 0 || v > 65535) return false; port = uint16_t(v); }
    catch (...) { return false; }
    return true;
}

LiveClient::~LiveClient() { close(); }

bool LiveClient::connect(std::string const& host, uint16_t port, std::string* err) {
    close();
    addrinfo hints{}; hints.ai_family = AF_UNSPEC; hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res); rc != 0 || !res) {
        if (err) *err = std::string("resolve ") + host + ": " + gai_strerror(rc);
        return false;
    }
    int fd = -1;
    for (addrinfo* a = res; a; a = a->ai_next) {
        fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) continue;
        timeval tv{5, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        if (::connect(fd, a->ai_addr, a->ai_addrlen) == 0) break;
        ::close(fd); fd = -1;
    }
    freeaddrinfo(res);
    if (fd < 0) { if (err) *err = "connect " + host + ":" + std::to_string(port) + ": " + std::strerror(errno); return false; }
    fd_ = fd; host_ = host; port_ = port; buf_.clear();
    return true;
}

void LiveClient::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

nlohmann::json LiveClient::call(nlohmann::json const& req) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (fd_ < 0) return {{"status", "error"}, {"message", "not connected"}};
    std::string msg = req.dump() + "\n";
    size_t sent = 0;
    while (sent < msg.size()) {
        ssize_t n = ::send(fd_, msg.data() + sent, msg.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) { close(); return {{"status", "error"}, {"message", std::string("send: ") + std::strerror(errno)}}; }
        sent += size_t(n);
    }
    char chunk[8192];
    while (true) {
        auto nl = buf_.find('\n');
        if (nl != std::string::npos) {
            std::string line = buf_.substr(0, nl);
            buf_.erase(0, nl + 1);
            nlohmann::json r = nlohmann::json::parse(line, nullptr, false);
            if (r.is_discarded()) return {{"status", "error"}, {"message", "unparseable reply"}};
            return r;
        }
        ssize_t n = ::recv(fd_, chunk, sizeof chunk, 0);
        if (n <= 0) { close(); return {{"status", "error"}, {"message", n == 0 ? "host closed the connection" : std::string("recv: ") + std::strerror(errno)}}; }
        buf_.append(chunk, size_t(n));
        if (buf_.size() > 64 * 1024 * 1024) { close(); return {{"status", "error"}, {"message", "reply too large"}}; }
    }
}

} // namespace bb
