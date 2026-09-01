#pragma once
// =============================================================================
// VideoClient.hpp  --  Godot-side subscriber for ogma_host's camera stream
// =============================================================================
//
// ⚠ WHY THIS IS NOT IN BenchClient, AND WHY IT DOES NOT BREACH SPEC §1.1.
// BenchClient speaks ogma_benchd's CALIBRATION verb set and must keep no path to
// the brain.  This class is a different thing entirely: one SUB socket onto
// ogma_host's video PUB, receive-only.  There is no request path, no verb, no way
// to command anything — it cannot start, stop or steer the brain, only watch what
// the camera produced.  Observation is exactly what the inspector already does;
// what §1.1 forbids is a CONTROL path from the dashboard, and there is none here.
//
// The brain's module state stays the inspector's job.  Video is Godot's, because
// Godot is where a picture belongs.
//
// Wire format (single frame, because ZMQ_CONFLATE does not do multipart):
//     "video " <json header> "\n" <brain plane bytes> <view plane bytes>
// The header carries every size, so geometry never has to be agreed out of band —
// assuming geometry instead of reading it is precisely what broke the camera
// reader on the Pi (the ISP pads the Y stride).
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

#include <string>

namespace godot {

class VideoClient : public Node {
    GDCLASS(VideoClient, Node)

public:
    VideoClient();
    ~VideoClient() override;

    bool       connect_to(String const& host, int video_port);
    void       disconnect_from();
    bool       is_connected() const { return sub_ != nullptr; }

    // Drains to the newest frame.  True when a NEW one arrived, at which point
    // brain_image()/view_image()/info() reflect it.  Cheap to call every _process.
    bool       poll();

    Ref<Image> brain_image() const { return brain_; }   // 32x32 — the encoder's input
    Ref<Image> view_image()  const { return view_; }    // native aspect — for humans
    Dictionary info()        const { return info_; }
    String     last_error()  const { return String(last_error_.c_str()); }

protected:
    static void _bind_methods();

private:
    void*       ctx_ = nullptr;
    void*       sub_ = nullptr;
    Ref<Image>  brain_;
    Ref<Image>  view_;
    Dictionary  info_;
    std::string last_error_;
};

} // namespace godot
