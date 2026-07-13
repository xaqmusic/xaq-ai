#pragma once

// =============================================================================
// Rng.hpp  --  Per-component RNG derivation for reproducible v4 experiments
// =============================================================================
//
// Mirrors v3's Python `src/ami_ogma_v3/_rng.py`.  Contract documented in
// `docs/primitives/_rng.md`.
//
// One master seed is split into many independent streams — one per stochastic
// component — by hashing the namespace string with SHA-256 and XORing the
// first 8 bytes (interpreted big-endian) with the master seed.  Same master
// seed + same namespace → same stream, across runs and across code changes
// that don't touch the namespaced component.
//
// Cross-language byte-exact RNG is NOT required (Python uses PCG64, C++ uses
// std::mt19937_64) — but `namespace_seed()` is byte-exact between the two
// implementations.  See `docs/primitives/_rng.md` for the full contract.

#include <cstdint>
#include <random>
#include <string_view>

namespace ogma {

// Derive a 64-bit seed from a master seed and a namespace string.
// Algorithm (must match Python `src/ami_ogma_v3/_rng.py:namespace_seed`):
//   1. digest = SHA-256(namespace.utf8_bytes())
//   2. h = first 8 bytes of digest, interpreted big-endian as uint64
//   3. return (master_seed ^ h)  & 0xFFFFFFFFFFFFFFFF
uint64_t namespace_seed(uint64_t master_seed, std::string_view ns);

// Construct a std::mt19937_64 seeded with namespace_seed(master_seed, ns).
// Convenience wrapper for components that need a Generator object.
std::mt19937_64 derive_rng(uint64_t master_seed, std::string_view ns);

} // namespace ogma
