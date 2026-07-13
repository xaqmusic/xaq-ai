// =============================================================================
// test_rng_parity.cpp  --  Cross-language RNG derivation parity test
// =============================================================================
//
// Verifies that ogma::namespace_seed() produces byte-identical outputs to
// v3 Python's `_rng.namespace_seed()` for a fixed table of (master_seed,
// namespace) inputs.  Reference values were produced by running the v3
// Python implementation; see docs/primitives/_rng.md for the algorithm.
//
// `derive_rng()` is also exercised, but only for same-language determinism —
// std::mt19937_64 produces a different stream than Python's PCG64 by design
// (documented non-requirement in _rng.md).

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <string_view>

#include "ogma/Rng.hpp"

namespace {

struct ParityVector {
    uint64_t         master_seed;
    std::string_view ns;
    uint64_t         expected_seed;        // Python namespace_seed(master, ns)
    uint64_t         expected_sha_top64;   // int(SHA256(ns).hexdigest()[:16], 16)
};

// Reference values produced by:
//   conda run -n ami-ogma python -c "import sys; sys.path.insert(0,'src');
//     from ami_ogma_v3._rng import namespace_seed; ..."
// (See the test_rng_parity commit message for the exact invocation.)
constexpr ParityVector kParityVectors[] = {
    {          0ull, "",                                          0xe3b0c44298fc1c14ull, 0xe3b0c44298fc1c14ull },
    {          0ull, "decoder.action_decoder.probe",              0x6fc4d6c6cc1ce1b3ull, 0x6fc4d6c6cc1ce1b3ull },
    {         42ull, "decoder.action_decoder.probe",              0x6fc4d6c6cc1ce199ull, 0x6fc4d6c6cc1ce1b3ull },
    {         42ull, "epm.epm_retinal.jl_matrix",                 0xe07ee6b46becff3bull, 0xe07ee6b46becff11ull },
    {         42ull, "voter.voter_0.trust_tiebreak",              0x7b7847a9bb4ffae9ull, 0x7b7847a9bb4ffac3ull },
    {         42ull, "rollout.roller_0.trajectories",             0xef099a15a03d7611ull, 0xef099a15a03d763bull },
    { 1234567890ull, "seqgng.seq_consensus.jl_matrix",            0xd4cbbb377f46754bull, 0xd4cbbb3736d07799ull },
};

} // namespace

TEST(RngParity, NamespaceSeedMatchesPython) {
    for (auto const& v : kParityVectors) {
        EXPECT_EQ(ogma::namespace_seed(v.master_seed, v.ns), v.expected_seed)
            << "  master_seed=" << v.master_seed
            << "  namespace=\""  << v.ns << '"';
    }
}

TEST(RngParity, ZeroSeedRevealsRawHash) {
    // With master_seed=0, namespace_seed equals the SHA-256 first-8-byte
    // big-endian read.  This is what the Python derivation does too; the
    // parity test for seed=0 directly exercises the SHA-256 implementation.
    for (auto const& v : kParityVectors) {
        if (v.master_seed != 0) continue;
        EXPECT_EQ(v.expected_seed, v.expected_sha_top64);
        EXPECT_EQ(ogma::namespace_seed(0u, v.ns), v.expected_sha_top64);
    }
}

TEST(RngParity, XorIsExactWithMasterSeed) {
    // For non-zero master seeds, namespace_seed = sha_top64 ^ master_seed.
    for (auto const& v : kParityVectors) {
        if (v.master_seed == 0) continue;
        EXPECT_EQ(v.expected_seed, v.expected_sha_top64 ^ v.master_seed);
    }
}

TEST(RngParity, DifferentNamespacesProduceDifferentSeeds) {
    // Sanity: 50 distinct namespaces with a fixed master seed all produce
    // distinct seeds (no collisions in this small set).
    std::set<uint64_t> seeds;
    for (int i = 0; i < 50; ++i) {
        std::string ns = "module." + std::to_string(i) + ".purpose";
        seeds.insert(ogma::namespace_seed(42u, ns));
    }
    EXPECT_EQ(seeds.size(), 50u);
}

TEST(RngParity, DeriveRngIsDeterministic) {
    // Same master_seed + same namespace → identical mt19937_64 stream.
    auto rng_a = ogma::derive_rng(42u, "decoder.foo.probe");
    auto rng_b = ogma::derive_rng(42u, "decoder.foo.probe");
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(rng_a(), rng_b());
    }
}

TEST(RngParity, DeriveRngStreamsAreIndependent) {
    // Different namespaces → independent streams (no obvious correlation).
    auto a = ogma::derive_rng(42u, "module.a.purpose");
    auto b = ogma::derive_rng(42u, "module.b.purpose");
    bool seen_diff = false;
    for (int i = 0; i < 10; ++i) {
        if (a() != b()) { seen_diff = true; break; }
    }
    EXPECT_TRUE(seen_diff);
}
