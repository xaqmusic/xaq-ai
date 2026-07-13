#pragma once

#include <vector>
#include <string>
#include <memory>

namespace ami_ogma {
namespace sensors {

// Abstract Audio Interface
class AudioStream {
public:
    virtual ~AudioStream() = default;

    // Fills the provided buffer with the next chunk of float32 audio samples.
    // Blocks if data is not immediately available.
    // Returns true on success, false on EOF or stream failure.
    virtual bool read_chunk(std::vector<float>& out_buffer) = 0;
};

// Abstract Video Interface
class VideoStream {
public:
    virtual ~VideoStream() = default;

    // Fills the provided tensor with the next frame.
    // Returns true on success, false on EOF or stream failure.
    virtual bool read_frame(std::vector<float>& out_tensor) = 0;
};

// Factory functions
std::unique_ptr<AudioStream> create_live_audio(size_t sample_rate, size_t channels);
std::unique_ptr<AudioStream> create_mock_audio(const std::string& wav_file_path);

std::unique_ptr<VideoStream> create_live_video(int device_id, size_t width, size_t height);
std::unique_ptr<VideoStream> create_mock_video(const std::string& mp4_file_path, size_t width, size_t height);
std::unique_ptr<VideoStream> create_window_video(const std::string& window_id, size_t width, size_t height);

} // namespace sensors
} // namespace ami_ogma
