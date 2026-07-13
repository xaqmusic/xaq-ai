#pragma once

// =============================================================================
// Replay.hpp  --  OGFS (Ogma Golden Frame Stream) reader
// =============================================================================
//
// Phase 2 deliverable per docs/primitives/_phase2_replay.md.  Reads the
// versioned binary stream produced by scripts/capture_golden_frames.py and
// surfaces it as a sequence of Tick records the host can bridge onto Bus
// topics.
//
// Format (v1):
//   40-byte header:  magic="OGFS", format_version, reserved, tick_count,
//                    image w/h/c, proprio_dim, master_seed, crc32 over body.
//   per-tick record: tick_id (u64), image bytes (W*H*C), proprio (P*float32),
//                    event_count (u8), events [name_len, name, intensity],
//                    record_crc32.

#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace ogma::golden {

struct Header {
    uint16_t format_version = 0;
    uint64_t tick_count     = 0;
    uint32_t image_width    = 0;
    uint32_t image_height   = 0;
    uint32_t image_channels = 0;
    uint32_t proprio_dim    = 0;
    uint32_t master_seed    = 0;
};

struct Event {
    std::string name;
    float       intensity = 1.0f;
};

struct Tick {
    uint64_t              tick_id = 0;
    std::vector<uint8_t>  image;          // size = w*h*c
    std::vector<float>    proprio;        // size = proprio_dim
    std::vector<Event>    events;
};

class StreamReader {
public:
    explicit StreamReader(std::string path);

    Header const& header() const { return header_; }
    bool          eof()    const;

    // Reads the next tick.  Returns false at EOF or on read error.
    bool next(Tick& out);

    std::string const& path() const { return path_; }

private:
    std::string   path_;
    std::ifstream in_;
    Header        header_;
    uint64_t      tick_index_ = 0;
};

// Convenience: returns false if `path` does not exist on disk.  Used by
// integration tests that gracefully skip when the OGFS capture hasn't been
// regenerated locally.
bool file_exists(std::string const& path);

} // namespace ogma::golden
