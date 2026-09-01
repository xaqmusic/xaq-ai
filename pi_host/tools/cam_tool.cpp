// cam_tool — look at what the retinal EPM actually sees.
//   cam_tool grab <n> <out.pgm> [width height]
// Writes the n downsampled 32x32 frames as one horizontal PGM montage and prints
// per-frame statistics.  Exists because "the encoder output is a repeating pattern
// that ignores the scene" is a claim about pixels, and the cheapest way to settle a
// claim about pixels is to put them on screen (2026-09-01: it was ISP stride padding).
#include "ogma/hw/CameraCapture.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

using namespace ogma::hw;

int main(int argc, char** argv) {
    if (argc < 4 || std::string(argv[1]) != "grab") {
        std::fprintf(stderr, "usage: cam_tool grab <n> <out.pgm> [width height]\n");
        return 2;
    }
    const int n = std::atoi(argv[2]);
    const char* out = argv[3];
    CameraCapture::Config cfg;
    if (argc >= 6) { cfg.src_width = std::atoi(argv[4]); cfg.src_height = std::atoi(argv[5]); }
    CameraCapture cam(cfg);
    if (!cam.start()) { std::fprintf(stderr, "cam: %s\n", cam.last_error().c_str()); return 1; }

    const int S = cam.out_size();
    std::vector<std::vector<uint8_t>> frames;
    std::vector<uint8_t> f, prev;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (int(frames.size()) < n && std::chrono::steady_clock::now() < deadline) {
        if (!cam.latest(f)) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
        double mean = 0, var = 0, delta = 0;
        for (uint8_t v : f) mean += v;
        mean /= double(f.size());
        for (uint8_t v : f) var += (v - mean) * (v - mean);
        var = std::sqrt(var / double(f.size()));
        if (!prev.empty())
            for (size_t i = 0; i < f.size(); ++i) delta += std::fabs(double(f[i]) - double(prev[i]));
        delta /= double(f.size());
        std::printf("  frame %2zu  mean %6.1f  sd %6.1f  mean|delta vs prev| %6.2f\n",
                    frames.size(), mean, var, delta);
        prev = f;
        frames.push_back(f);
    }
    cam.stop();
    if (frames.empty()) { std::fprintf(stderr, "no frames: %s\n", cam.last_error().c_str()); return 1; }

    // One wide PGM: frames left to right, so a repeating pattern is obvious by eye.
    FILE* fp = std::fopen(out, "wb");
    if (!fp) { std::fprintf(stderr, "cannot write %s\n", out); return 1; }
    const int W = S * int(frames.size());
    std::fprintf(fp, "P5\n%d %d\n255\n", W, S);
    for (int y = 0; y < S; ++y)
        for (auto const& fr : frames)
            std::fwrite(fr.data() + size_t(y) * size_t(S), 1, size_t(S), fp);
    std::fclose(fp);
    std::printf("wrote %s  (%d frames of %dx%d, stride was %d, %zu bytes/frame)\n",
                out, int(frames.size()), S, S, cam.stride(), cam.frame_bytes());
    return 0;
}
