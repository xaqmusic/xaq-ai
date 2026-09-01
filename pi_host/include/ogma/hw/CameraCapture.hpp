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
        // ⚠ 256x192 because it is STRIDE-CLEAN.  The ISP pads the Y-plane stride up to a
        // multiple of 128 (measured 2026-09-01: 160 -> 256, 192 -> 256, 320 -> 384, while
        // 128/256/640 are already clean).  The stride is measured at startup regardless,
        // so any size works -- this default just avoids paying for padding.
        int src_width  = 256;
        int src_height = 192;
        int out_size   = 32;       // the retinal encoder's spec (32x32x1, modality "retinal")
        int fps        = 15;       // well under the 50 Hz tick; frames are held, not faked
        int src_stride = 0;        // 0 = probe it; non-zero overrides the probe
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
    int      stride()      const { return stride_; }     // measured, not assumed
    size_t   frame_bytes() const { return frame_bytes_; }
    // Mean brightness of the last frame: the "is the lens cap on?" check.
    float    mean_level()  const { return mean_; }

    // Pure helpers, exposed for the tests.
    //
    // reduce(): centre-crop to square, then area-average down to out*out.  Area-average
    // (not nearest) because a 6x decimation that samples one pixel in thirty six is an
    // aliasing filter, not a downsample.
    // ⚠ `stride` is the ROW PITCH IN BYTES, which is NOT the width: the ISP pads it.
    // Treating stride as width shears the image and, worse, makes the frame size wrong.
    static void reduce(const uint8_t* y, int w, int h, int stride, int out,
                       std::vector<uint8_t>& dst);

    // Y-plane row pitch for a given width, per the ISP's measured alignment rule.
    // Used only as the fallback when the startup probe cannot run.
    static int  stride_for_width(int width);
    // True bytes per YUV420 frame given a row pitch: Y + quarter-size U and V.
    static size_t frame_bytes_for(int stride, int height);

private:
    void run();

    Config      cfg_;
    int         stride_      = 0;     // measured at start(), never assumed
    size_t      frame_bytes_ = 0;
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
