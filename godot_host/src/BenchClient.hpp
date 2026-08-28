#pragma once
// =============================================================================
// BenchClient.hpp  --  Godot-side client for ogma_benchd (pi_host/PROTOCOL.md)
// =============================================================================
//
// Two ZMQ sockets, no threads:
//   REQ  -> tcp://host:rep_port   verbs, one JSON object each way, 500 ms timeout.
//                                 A REQ that missed a reply is stuck by design, so on
//                                 timeout the socket is closed and recreated.
//   SUB  -> tcp://host:pub_port   topic "bench", ZMQ_CONFLATE so the viewer always
//                                 sees the newest frame and never a backlog.
// The dashboard speaks the calibration verb set ONLY (SPEC §1.1) — there is no
// path from here to starting the brain, and none must ever be added.

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

#include <string>

namespace godot {

class BenchClient : public Node {
    GDCLASS(BenchClient, Node)

public:
    BenchClient();
    ~BenchClient() override;

    bool       connect_to(String const& host, int rep_port, int pub_port);
    void       disconnect_from();
    bool       is_connected() const { return req_ != nullptr; }
    Dictionary request(Dictionary const& request);
    Dictionary poll_telemetry();
    String     last_error() const { return String(last_error_.c_str()); }

protected:
    static void _bind_methods();

private:
    bool  open_req();
    void  close_req();

    void*       ctx_  = nullptr;
    void*       req_  = nullptr;
    void*       sub_  = nullptr;
    std::string req_endpoint_;
    std::string last_error_;
};

} // namespace godot
