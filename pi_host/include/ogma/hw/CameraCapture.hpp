#pragma once
// CameraCapture — the OV5647, as a sensory channel.
//
// Feeds an EPM configured `encoder_kind = jl`, modality `retinal`, which expects
// 32x32x1 grayscale (cpp_core/src/v3/encoder_jl.cpp: get_modality_spec).
//
// Frames come from `rpicam-vid` over a pipe rather than from libcamera's C++ API.
// That is a deliberate trade: libcamera is a heavy dependency with a moving ABI, and
// the supported Pi camera stack is exactly what rpicam-apps wraps.  A pipe of raw
// YUV420 is a stable contract, the subprocess is isolated from the brain's address
// space, and if the camera stack dies the host sees EOF instead of a segfault.
//
// ⚠ The camera was once an ORACLE in this project — the sim's "camera" tested collider
// identity and never sampled a material, and its optics were invented rather than the
// OV5647's real 53.5 x 41.4 degrees.  This path reads photons, which is the point, but
// what is DONE with them must stay a transparent sensor reduction: grayscale downsample
// only, no thresholding for a target, no colour keying.  Selection and belief are the
// brain's (CLAUDE.md §5.2).
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ogma::hw {

class CameraCapture {
public:
    struct Config {
        int src_width  = 160;      // multiples of 32 avoid ISP stride padding
        int src_height = 120;
        int out_size   = 32;       // the retinal encoder's spec
        int fps        = 15;       // well under the 50 Hz tick; frames are held, not faked
        std::string binary = "rpicam-vid";
    };

    explicit CameraCapture(Config cfg);
    ~CameraCapture();
    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;

    bool start();
    void stop();
    bool running() const { return running_.load(); }

    // Newest frame as out_size*out_size grayscale.  False when nothing new since
    // the last call — the caller must not republish a stale frame (see AudioCapture).
    bool latest(std::vector<uint8_t>& out);

    const std::string& last_error() const { return err_; }
    uint64_t frames()      const { return frames_; }
    int      out_size()    const { return cfg_.out_size; }
    // Mean brightness of the last frame: the "is the lens cap on?" check.
    float    mean_level()  const { return mean_; }

    // Pure helper, exposed for the tests: centre-crop to square, then area-average
    // down to out*out.  Area-average (not nearest) because a 5x decimation that
    // samples one pixel in twenty five is an aliasing filter, not a downsample.
    static void reduce(const uint8_t* y, int w, int h, int out, std::vector<uint8_t>& dst);

private:
    void run();

    Config      cfg_;
    int         pipe_fd_ = -1;
    int         child_   = -1;
    std::thread th_;
    mutable std::mutex m_;
    std::vector<uint8_t> frame_;
    bool        fresh_   = false;
    std::atomic<bool> running_{false};
    uint64_t    frames_  = 0;
    float       mean_    = 0.0f;
    std::string err_;
};

} // namespace ogma::hw
