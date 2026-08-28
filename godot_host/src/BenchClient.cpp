#include "BenchClient.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <nlohmann/json.hpp>
#include <zmq.h>

#include <cstring>

using json = nlohmann::json;

namespace godot {

// ---- Variant <-> json ------------------------------------------------------

static json variant_to_json(Variant const& v) {
    switch (v.get_type()) {
        case Variant::NIL:    return nullptr;
        case Variant::BOOL:   return bool(v);
        case Variant::INT:    return int64_t(v);
        case Variant::FLOAT:  return double(v);
        case Variant::STRING: return std::string(String(v).utf8().get_data());
        case Variant::ARRAY: {
            json a = json::array();
            Array arr = v;
            for (int i = 0; i < arr.size(); ++i) a.push_back(variant_to_json(arr[i]));
            return a;
        }
        case Variant::DICTIONARY: {
            json o = json::object();
            Dictionary d = v;
            Array keys = d.keys();
            for (int i = 0; i < keys.size(); ++i)
                o[std::string(String(keys[i]).utf8().get_data())] = variant_to_json(d[keys[i]]);
            return o;
        }
        default:
            return std::string(String(v).utf8().get_data());
    }
}

static Variant json_to_variant(json const& j) {
    if (j.is_null())    return Variant();
    if (j.is_boolean()) return Variant(j.get<bool>());
    if (j.is_number_integer()) return Variant(int64_t(j.get<int64_t>()));
    if (j.is_number_float())   return Variant(j.get<double>());
    if (j.is_string()) return Variant(String(j.get<std::string>().c_str()));
    if (j.is_array()) {
        Array a;
        for (auto const& e : j) a.push_back(json_to_variant(e));
        return a;
    }
    if (j.is_object()) {
        Dictionary d;
        for (auto it = j.begin(); it != j.end(); ++it) d[String(it.key().c_str())] = json_to_variant(it.value());
        return d;
    }
    return Variant();
}

// ---- lifecycle ---------------------------------------------------------------

BenchClient::BenchClient() {}

BenchClient::~BenchClient() {
    disconnect_from();
    if (ctx_) { zmq_ctx_term(ctx_); ctx_ = nullptr; }
}

void BenchClient::_bind_methods() {
    ClassDB::bind_method(D_METHOD("connect_to", "host", "rep_port", "pub_port"), &BenchClient::connect_to);
    ClassDB::bind_method(D_METHOD("disconnect_from"),                            &BenchClient::disconnect_from);
    ClassDB::bind_method(D_METHOD("is_connected"),                               &BenchClient::is_connected);
    ClassDB::bind_method(D_METHOD("request", "request"),                         &BenchClient::request);
    ClassDB::bind_method(D_METHOD("poll_telemetry"),                             &BenchClient::poll_telemetry);
    ClassDB::bind_method(D_METHOD("last_error"),                                 &BenchClient::last_error);
}

bool BenchClient::open_req() {
    close_req();
    req_ = zmq_socket(ctx_, ZMQ_REQ);
    if (!req_) { last_error_ = std::string("zmq_socket REQ: ") + zmq_strerror(zmq_errno()); return false; }
    int timeout = 500, linger = 0;
    zmq_setsockopt(req_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(req_, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(req_, ZMQ_LINGER,   &linger,  sizeof(linger));
    if (zmq_connect(req_, req_endpoint_.c_str()) != 0) {
        last_error_ = std::string("zmq_connect REQ: ") + zmq_strerror(zmq_errno());
        close_req();
        return false;
    }
    return true;
}

void BenchClient::close_req() {
    if (req_) { zmq_close(req_); req_ = nullptr; }
}

bool BenchClient::connect_to(String const& host, int rep_port, int pub_port) {
    disconnect_from();
    if (!ctx_) ctx_ = zmq_ctx_new();
    std::string h(host.utf8().get_data());
    req_endpoint_ = "tcp://" + h + ":" + std::to_string(rep_port);
    if (!open_req()) return false;

    sub_ = zmq_socket(ctx_, ZMQ_SUB);
    if (!sub_) { last_error_ = std::string("zmq_socket SUB: ") + zmq_strerror(zmq_errno()); close_req(); return false; }
    int conflate = 1, linger = 0;
    zmq_setsockopt(sub_, ZMQ_CONFLATE,  &conflate, sizeof(conflate));   // BEFORE connect
    zmq_setsockopt(sub_, ZMQ_LINGER,    &linger,   sizeof(linger));
    zmq_setsockopt(sub_, ZMQ_SUBSCRIBE, "bench", 5);
    std::string pub_ep = "tcp://" + h + ":" + std::to_string(pub_port);
    if (zmq_connect(sub_, pub_ep.c_str()) != 0) {
        last_error_ = std::string("zmq_connect SUB: ") + zmq_strerror(zmq_errno());
        zmq_close(sub_); sub_ = nullptr; close_req();
        return false;
    }
    last_error_.clear();
    return true;
}

void BenchClient::disconnect_from() {
    close_req();
    if (sub_) { zmq_close(sub_); sub_ = nullptr; }
}

// ---- verbs ---------------------------------------------------------------------

Dictionary BenchClient::request(Dictionary const& request) {
    Dictionary out;
    if (!req_) { out["ok"] = false; out["error"] = "not connected"; return out; }
    std::string body = variant_to_json(request).dump();
    if (zmq_send(req_, body.data(), body.size(), 0) < 0) {
        last_error_ = std::string("send: ") + zmq_strerror(zmq_errno());
        open_req();                                          // reset the REQ state machine
        out["ok"] = false; out["error"] = last_error_.c_str(); return out;
    }
    zmq_msg_t msg; zmq_msg_init(&msg);
    int n = zmq_msg_recv(&msg, req_, 0);
    if (n < 0) {
        zmq_msg_close(&msg);
        last_error_ = (zmq_errno() == EAGAIN) ? "timeout" : std::string("recv: ") + zmq_strerror(zmq_errno());
        open_req();                                          // a REQ that missed a reply is stuck by design
        out["ok"] = false; out["error"] = last_error_.c_str(); return out;
    }
    std::string reply(static_cast<char*>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    try {
        Variant v = json_to_variant(json::parse(reply));
        if (v.get_type() == Variant::DICTIONARY) return v;
        out["ok"] = false; out["error"] = "reply is not an object"; return out;
    } catch (std::exception const& e) {
        last_error_ = std::string("bad reply json: ") + e.what();
        out["ok"] = false; out["error"] = last_error_.c_str(); return out;
    }
}

// ---- telemetry -------------------------------------------------------------------

Dictionary BenchClient::poll_telemetry() {
    Dictionary out;
    if (!sub_) return out;
    // Drain to the newest frame.  With ZMQ_CONFLATE there is at most one queued;
    // without it (a daemon that sends multipart) we still end on the last one.
    std::string body; bool got = false;
    for (;;) {
        zmq_msg_t msg; zmq_msg_init(&msg);
        int n = zmq_msg_recv(&msg, sub_, ZMQ_DONTWAIT);
        if (n < 0) { zmq_msg_close(&msg); break; }
        std::string part(static_cast<char*>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
        int more = 0; size_t sz = sizeof(more);
        zmq_getsockopt(sub_, ZMQ_RCVMORE, &more, &sz);
        zmq_msg_close(&msg);
        if (more) continue;                                  // topic frame of a 2-part message
        body = part; got = true;
    }
    if (!got) return out;
    // Single-frame form "bench {json}" (what CONFLATE needs): strip the topic prefix.
    size_t brace = body.find('{');
    if (brace == std::string::npos) return out;
    try {
        Variant v = json_to_variant(json::parse(body.substr(brace)));
        if (v.get_type() == Variant::DICTIONARY) return v;
    } catch (std::exception const& e) {
        last_error_ = std::string("bad telemetry json: ") + e.what();
    }
    return out;
}

} // namespace godot
