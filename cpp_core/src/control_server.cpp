#include "../include/control_server.hpp"
#include <iostream>
#include <cstring>
#include <vector>

#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#endif

namespace ami_ogma {
namespace control {

ControlServer::ControlServer(uint16_t port) : port_(port), is_running_(false), server_fd_(-1) {
    handler_ = [](const nlohmann::json& req) {
        return nlohmann::json{{"status", "error"}, {"message", "No handler registered"}};
    };
}

ControlServer::~ControlServer() {
    stop();
}

void ControlServer::set_command_handler(CommandCallback handler) {
    handler_ = handler;
}

void ControlServer::start() {
    if (is_running_) return;

#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ == 0) {
        std::cerr << "Failed to create control socket." << std::endl;
        return;
    }

    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        std::cerr << "setsockopt SO_REUSEADDR failed." << std::endl;
    }

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Control socket bind failed on port " << port_ << std::endl;
        return;
    }

    if (listen(server_fd_, 3) < 0) {
        std::cerr << "Control socket listen failed." << std::endl;
        return;
    }
    
    is_running_ = true;
    listen_thread_ = std::thread(&ControlServer::run_loop, this);
    std::cout << "Control Server listening on port " << port_ << std::endl;
#else
    std::cerr << "Control Server non-POSIX implementation missing!" << std::endl;
#endif
}

void ControlServer::stop() {
    if (!is_running_) return;
    is_running_ = false;

#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
    if (server_fd_ != -1) {
        close(server_fd_);
        server_fd_ = -1;
    }
#endif

    if (listen_thread_.joinable()) {
        listen_thread_.join();
    }
}

void ControlServer::run_loop() {
#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    while (is_running_) {
        // Use poll instead of select — no FD_SETSIZE (1024) limitation,
        // and handles fd=-1 race during shutdown gracefully.
        struct pollfd pfd;
        pfd.fd = server_fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;

        if (pfd.fd < 0) break;  // server_fd_ closed during shutdown

        int activity = poll(&pfd, 1, 1000);  // 1s timeout

        if (activity < 0) {
            if (is_running_) std::cerr << "Control socket poll error" << std::endl;
            continue;
        }

        if (activity == 0) continue; // Timeout, loop and check `is_running_` again

        if (pfd.revents & POLLIN) {
            int new_socket = accept(server_fd_, (struct sockaddr*)&address, &addrlen);
            if (new_socket < 0) {
                if (is_running_) std::cerr << "Control socket accept failed." << std::endl;
                continue;
            }

            // Disable Nagle's algorithm for low-latency loopback communication
            int one = 1;
            setsockopt(new_socket, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

            // Handle each client on a detached thread — allows persistent connections
            // (Python's _TCPClient reuses the same socket across ticks) while still
            // accepting new connections if the old client disconnects.
            std::thread([this, new_socket]() {
                handle_client(new_socket);
                close(new_socket);
            }).detach();
        }
    }
#endif
}

void ControlServer::handle_client(int client_socket) {
#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
    // Persistent-connection loop: handle multiple sequential requests on the same
    // socket until the client disconnects.  This eliminates the per-tick TCP
    // handshake that occurred when the connection was closed after each request.
    char chunk[4096];
    std::string buf;
    buf.reserve(4096);

    while (is_running_) {
        // Read until we have a complete newline-delimited JSON message
        while (true) {
            // Check if we already have a complete message buffered
            auto nl = buf.find('\n');
            if (nl != std::string::npos) break;

            ssize_t n = read(client_socket, chunk, sizeof(chunk));
            if (n <= 0) return;  // client disconnected — exit loop
            buf.append(chunk, n);

            // Safety cap: 4 MB
            if (buf.size() > 4 * 1024 * 1024) return;
        }

        // Extract the first complete message
        auto nl = buf.find('\n');
        std::string raw_request = buf.substr(0, nl + 1);
        buf.erase(0, nl + 1);

        try {
            auto request_json  = nlohmann::json::parse(raw_request);
            auto response_json = handler_(request_json);
            std::string raw_response = response_json.dump() + "\n";
            send(client_socket, raw_response.c_str(), raw_response.size(), 0);
        } catch (const std::exception& e) {
            std::string err = nlohmann::json{{"status", "error"}, {"message", e.what()}}.dump() + "\n";
            send(client_socket, err.c_str(), err.size(), 0);
        }
    }
#endif
}

} // namespace control
} // namespace ami_ogma
