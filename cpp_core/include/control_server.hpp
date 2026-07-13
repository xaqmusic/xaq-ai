#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <nlohmann/json.hpp>

// Note: To keep dependencies lightweight on embedded systems, we use POSIX sockets for Linux/macOS.
#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#endif

namespace ami_ogma {
namespace control {

// Callback type for when the server receives a valid JSON command.
// Returns a JSON response object to send back to the client.
using CommandCallback = std::function<nlohmann::json(const nlohmann::json&)>;

class ControlServer {
public:
    ControlServer(uint16_t port);
    ~ControlServer();

    // Start the background listening thread.
    void start();

    // Stop the server and join the thread.
    void stop();

    // Register a handler function to be executed when a command is received.
    void set_command_handler(CommandCallback handler);

private:
    uint16_t port_;
    std::atomic<bool> is_running_;
    std::thread listen_thread_;
    CommandCallback handler_;
    
#if defined(__unix__) || defined(__APPLE__) || defined(__linux__)
    int server_fd_;
#endif

    void run_loop();
    void handle_client(int client_socket);
};

} // namespace control
} // namespace ami_ogma
