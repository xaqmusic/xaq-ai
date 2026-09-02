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
        // A human-viewable plane beside the 32x32 the brain gets.  Integer divisor of
        // the source so it is an exact box average and keeps the native aspect (the
        // 32x32 is centre-cropped square, which is right for the encoder and wrong for
        // judging where the camera is pointed).
        int preview_div = 2;       // 256x192 -> 128x96; 0 disables the preview
        // Keep the chroma planes for the preview.  They cost NOTHING at the camera --
        // YUV420 already carries them and the reader already fetches the whole frame --
        // so this only decides whether they are downsampled and handed on.  The BRAIN's
        // plane stays luma-only regardless: the retinal encoder is 32x32x1 by spec.
        bool preview_color = true;
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
    // ⚠ CONSUMES the freshness flag: exactly one consumer may use this, and it is the
    // brain path.  Anything else watching the camera uses snapshot() below.
    bool latest(std::vector<uint8_t>& out);

    // What an observer gets: the brain's plane, plus a preview that may carry chroma.
    struct Frame {
        std::vector<uint8_t> small;             // out_size^2, L8 — the encoder's input
        std::vector<uint8_t> view_y, view_u, view_v;
        int      view_w = 0, view_h = 0;        // luma dimensions
        int      chroma_w = 0, chroma_h = 0;    // 0 = no chroma, preview is greyscale
        uint64_t seq = 0;
    };

    // Non-consuming read for observers (the video stream).  Fills `out` without touching
    // `fresh_`, so a viewer cannot starve the EPM of an observation by being scheduled
    // first — the bug this shape prevents.  Compare `seq` to detect a new frame.
    bool snapshot(Frame& out) const;

    const std::string& last_error() const { return err_; }
    uint64_t frames()      const { return frames_; }
    int      out_size()    const { return cfg_.out_size; }
    int      stride()      const { return stride_; }     // measured, not assumed
    int      src_width()   const { return cfg_.src_width; }
    int      src_height()  const { return cfg_.src_height; }
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

    // Exact NxN box average of the whole frame, aspect preserved — the preview path.
    static void box_downsample(const uint8_t* y, int w, int h, int stride, int div,
                               std::vector<uint8_t>& dst, int& out_w, int& out_h);

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
    std::vector<uint8_t> preview_, preview_u_, preview_v_;
    int         preview_w_ = 0, preview_h_ = 0;
    int         chroma_w_  = 0, chroma_h_  = 0;
    bool        fresh_   = false;
    std::atomic<bool> running_{false};
    uint64_t    frames_  = 0;
    float       mean_    = 0.0f;
    std::string err_;
};

} // namespace ogma::hw
