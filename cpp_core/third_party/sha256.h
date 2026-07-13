// =============================================================================
// sha256.h  --  Minimal single-header SHA-256 implementation
// =============================================================================
//
// Public-domain implementation of SHA-256 per FIPS 180-4 (RFC 6234).  This is
// the dependency for ogma/Rng.hpp's namespace_seed() so the C++ derivation
// matches v3's Python `hashlib.sha256(...)` byte-for-byte.
//
// Tested against the byte-exact values produced by Python's hashlib (see
// cpp_core/tests/ogma/test_rng_parity.cpp for known-answer tests).
//
// Usage:
//   uint8_t digest[32];
//   ogma::sha256(reinterpret_cast<const uint8_t*>(input), len, digest);
//
// No allocations.  Stack-only.  ~150 lines.
//
// =============================================================================

#pragma once

#include <cstdint>
#include <cstring>
#include <string_view>

namespace ogma {
namespace detail {

constexpr uint32_t kSha256K[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

inline uint32_t rotr(uint32_t x, unsigned n) { return (x >> n) | (x << (32u - n)); }

inline void sha256_compress(uint32_t state[8], const uint8_t block[64]) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
        w[i] =  (uint32_t(block[4*i + 0]) << 24)
              | (uint32_t(block[4*i + 1]) << 16)
              | (uint32_t(block[4*i + 2]) <<  8)
              | (uint32_t(block[4*i + 3]) <<  0);
    }
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = rotr(w[i-15], 7)  ^ rotr(w[i-15], 18) ^ (w[i-15] >> 3);
        uint32_t s1 = rotr(w[i-2], 17)  ^ rotr(w[i-2], 19)  ^ (w[i-2]  >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];

    for (int i = 0; i < 64; ++i) {
        uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t T1 = h + S1 + ch + kSha256K[i] + w[i];
        uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t T2 = S0 + mj;
        h = g; g = f; f = e; e = d + T1;
        d = c; c = b; b = a; a = T1 + T2;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

} // namespace detail

// Compute SHA-256 of `len` bytes starting at `data`.  Writes 32 bytes into `out`.
inline void sha256(const uint8_t* data, std::size_t len, uint8_t out[32]) {
    uint32_t state[8] = {
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
    };

    uint64_t bit_len = uint64_t(len) * 8u;

    // Process full 64-byte blocks.
    std::size_t i = 0;
    while (i + 64 <= len) {
        detail::sha256_compress(state, data + i);
        i += 64;
    }

    // Final block(s): copy remaining bytes, append 0x80, pad with zeros, append length.
    uint8_t tail[128] = {0};
    std::size_t rem = len - i;
    std::memcpy(tail, data + i, rem);
    tail[rem] = 0x80u;

    std::size_t tail_len = (rem < 56) ? 64u : 128u;
    // Append big-endian 64-bit length at the end of the (final) block.
    for (int b = 0; b < 8; ++b) {
        tail[tail_len - 1 - b] = uint8_t((bit_len >> (b * 8)) & 0xffu);
    }

    detail::sha256_compress(state, tail);
    if (tail_len == 128u) {
        detail::sha256_compress(state, tail + 64);
    }

    // Serialize state to big-endian output.
    for (int j = 0; j < 8; ++j) {
        out[4*j + 0] = uint8_t((state[j] >> 24) & 0xffu);
        out[4*j + 1] = uint8_t((state[j] >> 16) & 0xffu);
        out[4*j + 2] = uint8_t((state[j] >>  8) & 0xffu);
        out[4*j + 3] = uint8_t((state[j] >>  0) & 0xffu);
    }
}

inline void sha256(std::string_view input, uint8_t out[32]) {
    sha256(reinterpret_cast<const uint8_t*>(input.data()), input.size(), out);
}

} // namespace ogma
