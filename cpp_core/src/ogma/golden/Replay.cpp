#include "ogma/golden/Replay.hpp"

#include <cstring>
#include <stdexcept>
#include <sys/stat.h>

namespace ogma::golden {

namespace {

template <typename T>
bool read_le(std::ifstream& s, T& v) {
    static_assert(std::is_trivially_copyable_v<T>);
    s.read(reinterpret_cast<char*>(&v), sizeof(T));
    return bool(s);
}

template <typename T>
bool read_le_array(std::ifstream& s, T* dst, size_t n) {
    s.read(reinterpret_cast<char*>(dst), std::streamsize(sizeof(T) * n));
    return bool(s);
}

} // namespace

bool file_exists(std::string const& path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

StreamReader::StreamReader(std::string path)
    : path_(std::move(path))
    , in_(path_, std::ios::binary) {
    if (!in_) throw std::runtime_error("OGFS: cannot open " + path_);

    char     magic[4];
    uint16_t reserved;
    uint32_t header_crc;

    in_.read(magic, 4);
    if (in_.gcount() != 4 || std::memcmp(magic, "OGFS", 4) != 0)
        throw std::runtime_error("OGFS: bad magic in " + path_);

    if (!read_le(in_, header_.format_version) ||
        !read_le(in_, reserved) ||
        !read_le(in_, header_.tick_count) ||
        !read_le(in_, header_.image_width) ||
        !read_le(in_, header_.image_height) ||
        !read_le(in_, header_.image_channels) ||
        !read_le(in_, header_.proprio_dim) ||
        !read_le(in_, header_.master_seed) ||
        !read_le(in_, header_crc)) {
        throw std::runtime_error("OGFS: short header in " + path_);
    }

    if (header_.format_version != 1)
        throw std::runtime_error("OGFS: unsupported format_version " +
                                 std::to_string(header_.format_version));
}

bool StreamReader::eof() const {
    return tick_index_ >= header_.tick_count;
}

bool StreamReader::next(Tick& out) {
    if (eof()) return false;

    uint64_t tick_id = 0;
    if (!read_le(in_, tick_id)) return false;
    out.tick_id = tick_id;

    size_t img_bytes = size_t(header_.image_width)
                     * size_t(header_.image_height)
                     * size_t(header_.image_channels);
    out.image.resize(img_bytes);
    if (img_bytes && !read_le_array(in_, out.image.data(), img_bytes))
        return false;

    out.proprio.resize(header_.proprio_dim);
    if (header_.proprio_dim &&
        !read_le_array(in_, out.proprio.data(), header_.proprio_dim))
        return false;

    uint8_t event_count = 0;
    if (!read_le(in_, event_count)) return false;
    out.events.clear();
    out.events.reserve(event_count);
    for (int i = 0; i < int(event_count); ++i) {
        uint8_t name_len = 0;
        if (!read_le(in_, name_len)) return false;
        Event ev;
        ev.name.resize(name_len);
        if (name_len && !read_le_array(in_, ev.name.data(), name_len))
            return false;
        if (!read_le(in_, ev.intensity)) return false;
        out.events.push_back(std::move(ev));
    }

    uint32_t record_crc = 0;
    if (!read_le(in_, record_crc)) return false;
    // CRC verification is optional; leave that as a Phase 3+ enhancement.

    ++tick_index_;
    return true;
}

} // namespace ogma::golden
