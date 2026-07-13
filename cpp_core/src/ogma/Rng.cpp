#include "ogma/Rng.hpp"

#include "../../third_party/sha256.h"

namespace ogma {

uint64_t namespace_seed(uint64_t master_seed, std::string_view ns) {
    uint8_t digest[32];
    sha256(ns, digest);

    // First 8 bytes of the digest, big-endian, == int(hexdigest()[:16], 16).
    uint64_t h = 0;
    for (int i = 0; i < 8; ++i) {
        h = (h << 8) | uint64_t(digest[i]);
    }
    return master_seed ^ h;
}

std::mt19937_64 derive_rng(uint64_t master_seed, std::string_view ns) {
    return std::mt19937_64(namespace_seed(master_seed, ns));
}

} // namespace ogma
