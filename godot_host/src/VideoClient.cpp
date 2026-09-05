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
    if (b.is_valid()) brain_ = b;

    // The preview carries chroma when the camera kept it (YUV420, quarter-size U and V).
    // Converted here rather than on the Pi so the robot's 20 ms tick pays nothing for a
    // picture only this machine looks at.  Falls back to the luma plane whenever chroma
    // is absent or does not fit, so an older or greyscale publisher still renders.
    Ref<Image> v;
    if (h.contains("view_u") && h.contains("view_v")) v = view_rgb(h, payload, avail);
    if (!v.is_valid()) v = plane("view");
    if (v.is_valid()) view_ = v;

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

Ref<Image> VideoClient::view_rgb(const nlohmann::json& h, const char* payload, size_t avail) {
    const int    yw = h["view"].value("w", 0),   yh = h["view"].value("h", 0);
    const size_t yo = h["view"].value("off", 0);
    const int    cw = h["view_u"].value("w", 0), ch = h["view_u"].value("h", 0);
    const size_t uo = h["view_u"].value("off", 0), vo = h["view_v"].value("off", 0);
    if (yw <= 0 || yh <= 0 || cw <= 0 || ch <= 0) return Ref<Image>();
    const size_t ysz = size_t(yw) * size_t(yh), csz = size_t(cw) * size_t(ch);
    // Verify every plane against what actually arrived before indexing into it.
    if (yo + ysz > avail || uo + csz > avail || vo + csz > avail) return Ref<Image>();

    const uint8_t* Y = reinterpret_cast<const uint8_t*>(payload) + yo;
    const uint8_t* U = reinterpret_cast<const uint8_t*>(payload) + uo;
    const uint8_t* V = reinterpret_cast<const uint8_t*>(payload) + vo;

    PackedByteArray rgb;
    rgb.resize(int64_t(ysz * 3));
    uint8_t* out = rgb.ptrw();
    // BT.601 full-range, which is what the ISP emits for YUV420 here.  Chroma is
    // nearest-neighbour upsampled: the plane is a 128x96 preview, and a bilinear pass
    // would cost more than it shows.
    for (int y = 0; y < yh; ++y) {
        const int cy = (ch == yh) ? y : (y * ch) / yh;
        for (int x = 0; x < yw; ++x) {
            const int cx = (cw == yw) ? x : (x * cw) / yw;
            const float luma = float(Y[size_t(y) * size_t(yw) + size_t(x)]);
            const float cu   = float(U[size_t(cy) * size_t(cw) + size_t(cx)]) - 128.0f;
            const float cv   = float(V[size_t(cy) * size_t(cw) + size_t(cx)]) - 128.0f;
            auto clamp8 = [](float f) -> uint8_t {
                return uint8_t(f < 0.0f ? 0.0f : (f > 255.0f ? 255.0f : f));
            };
            const size_t o = (size_t(y) * size_t(yw) + size_t(x)) * 3;
            out[o + 0] = clamp8(luma + 1.402f * cv);
            out[o + 1] = clamp8(luma - 0.344136f * cu - 0.714136f * cv);
            out[o + 2] = clamp8(luma + 1.772f * cu);
        }
    }
    return Image::create_from_data(yw, yh, false, Image::FORMAT_RGB8, rgb);
}

} // namespace godot
