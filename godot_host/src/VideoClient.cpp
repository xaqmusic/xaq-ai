#include "VideoClient.hpp"

#include <godot_cpp/classes/json.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>

#include <nlohmann/json.hpp>
#include <zmq.h>

using json = nlohmann::json;

namespace godot {

void VideoClient::_bind_methods() {
    ClassDB::bind_method(D_METHOD("connect_to", "host", "video_port"), &VideoClient::connect_to);
    ClassDB::bind_method(D_METHOD("disconnect_from"), &VideoClient::disconnect_from);
    ClassDB::bind_method(D_METHOD("is_connected"), &VideoClient::is_connected);
    ClassDB::bind_method(D_METHOD("poll"), &VideoClient::poll);
    ClassDB::bind_method(D_METHOD("brain_image"), &VideoClient::brain_image);
    ClassDB::bind_method(D_METHOD("view_image"), &VideoClient::view_image);
    ClassDB::bind_method(D_METHOD("info"), &VideoClient::info);
    ClassDB::bind_method(D_METHOD("last_error"), &VideoClient::last_error);
}

VideoClient::VideoClient() = default;
VideoClient::~VideoClient() { disconnect_from(); }

bool VideoClient::connect_to(String const& host, int video_port) {
    disconnect_from();
    if (!ctx_) ctx_ = zmq_ctx_new();
    sub_ = zmq_socket(ctx_, ZMQ_SUB);
    if (!sub_) { last_error_ = std::string("zmq_socket SUB: ") + zmq_strerror(zmq_errno()); return false; }
    int conflate = 1, linger = 0;
    zmq_setsockopt(sub_, ZMQ_CONFLATE,  &conflate, sizeof(conflate));   // BEFORE connect
    zmq_setsockopt(sub_, ZMQ_LINGER,    &linger,   sizeof(linger));
    zmq_setsockopt(sub_, ZMQ_SUBSCRIBE, "video", 5);
    const std::string ep = "tcp://" + std::string(host.utf8().get_data()) + ":" + std::to_string(video_port);
    if (zmq_connect(sub_, ep.c_str()) != 0) {
        last_error_ = std::string("zmq_connect ") + ep + ": " + zmq_strerror(zmq_errno());
        zmq_close(sub_); sub_ = nullptr;
        return false;
    }
    last_error_.clear();
    return true;
}

void VideoClient::disconnect_from() {
    if (sub_) { zmq_close(sub_); sub_ = nullptr; }
    if (ctx_) { zmq_ctx_shutdown(ctx_); zmq_ctx_term(ctx_); ctx_ = nullptr; }
}

bool VideoClient::poll() {
    if (!sub_) return false;
    std::string body;
    bool got = false;
    for (;;) {                                   // drain to the newest
        zmq_msg_t msg; zmq_msg_init(&msg);
        const int n = zmq_msg_recv(&msg, sub_, ZMQ_DONTWAIT);
        if (n < 0) { zmq_msg_close(&msg); break; }
        body.assign(static_cast<char*>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
        zmq_msg_close(&msg);
        got = true;
    }
    if (!got) return false;

    const size_t nl = body.find('\n');
    const size_t brace = body.find('{');
    if (nl == std::string::npos || brace == std::string::npos || brace > nl) {
        last_error_ = "malformed video frame (no JSON header)";
        return false;
    }
    json h;
    try { h = json::parse(body.substr(brace, nl - brace)); }
    catch (const std::exception& e) { last_error_ = std::string("header: ") + e.what(); return false; }

    const char* const payload = body.data() + nl + 1;
    const size_t      avail   = body.size() - nl - 1;

    auto plane = [&](const char* key) -> Ref<Image> {
        if (!h.contains(key)) return Ref<Image>();
        const int    w   = h[key].value("w", 0);
        const int    ht  = h[key].value("h", 0);
        const size_t off = h[key].value("off", 0);
        const size_t need = size_t(w) * size_t(ht);
        // Trust the header's sizes but VERIFY them against what arrived; a plane that
        // does not fit is dropped rather than read past the buffer.
        if (w <= 0 || ht <= 0 || off + need > avail) return Ref<Image>();
        PackedByteArray bytes;
        bytes.resize(int64_t(need));
        std::memcpy(bytes.ptrw(), payload + off, need);
        return Image::create_from_data(w, ht, false, Image::FORMAT_L8, bytes);
    };

    Ref<Image> b = plane("brain");
    Ref<Image> v = plane("view");
    if (b.is_valid()) brain_ = b;
    if (v.is_valid()) view_  = v;

    Dictionary d;
    d["seq"]  = int64_t(h.value("seq", 0));
    d["tick"] = int64_t(h.value("tick", 0));
    if (h.contains("src")) {
        d["src_w"] = int(h["src"].value("w", 0));
        d["src_h"] = int(h["src"].value("h", 0));
    }
    d["bytes"] = int64_t(body.size());
    info_ = d;
    last_error_.clear();
    return brain_.is_valid() || view_.is_valid();
}

} // namespace godot
