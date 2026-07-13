// =============================================================================
// test_embedding_registry.cpp
//   Phase 6.6.E — EmbeddingRegistry single-owner cache.
//
// Verifies the four guarantees of the registry: it populates from observed
// reality.* tokens, returns the SAME shared_ptr on repeated lookups (zero-copy
// across consumers), evicts on `pruned_ids`, and tolerates concurrent reads
// alongside a write stream.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EmbeddingRegistry.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::RealityToken>
make_token(int winner_id, Eigen::VectorXf prototype,
           std::vector<int> pruned = {}) {
    auto t = std::make_shared<ogma::RealityToken>();
    t->winner_id        = winner_id;
    t->winner_prototype = std::move(prototype);
    t->pruned_ids       = std::move(pruned);
    t->latent           = Eigen::VectorXf::Zero(t->winner_prototype.size());
    return t;
}

ogma::ParamMap default_params() {
    return { {"input_pattern", std::string("reality.")} };
}

struct RegistryFixture {
    ogma::InProcessBus     bus;
    ogma::EmbeddingRegistry reg;
    explicit RegistryFixture(ogma::ParamMap const& p = default_params()) {
        reg.set_id("registry");
        reg.on_setup(&bus, p);
    }
    template <typename F>
    void run_tick(uint64_t t, F&& f) {
        bus.begin_tick(t);
        f();
        bus.end_tick();
    }
};

} // namespace

// =============================================================================
// 1. Populate from a stream of unique winners.
// =============================================================================

TEST(EmbeddingRegistry, PopulatesFromRealityStream) {
    RegistryFixture f;

    Eigen::VectorXf p0 = Eigen::VectorXf::Constant(4, 1.0f);
    Eigen::VectorXf p1 = Eigen::VectorXf::Constant(4, 2.0f);
    Eigen::VectorXf p2 = Eigen::VectorXf::Constant(4, 3.0f);

    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal", make_token(0, p0));
        f.bus.publish("reality.video.retinal", make_token(1, p1));
        f.bus.publish("reality.video.retinal", make_token(2, p2));
    });

    EXPECT_EQ(f.reg.size("reality.video.retinal"), 3u);
    EXPECT_EQ(f.reg.total_size(), 3u);

    auto e1 = f.reg.get("reality.video.retinal", 1);
    ASSERT_NE(e1, nullptr);
    EXPECT_FLOAT_EQ((*e1)(0), 2.0f);
    EXPECT_FLOAT_EQ((*e1)(3), 2.0f);
}

// =============================================================================
// 2. Zero-copy: two get() calls return the SAME pointer.
//    The whole point of the registry is to avoid duplicating embeddings
//    across consumers.  shared_ptr identity is the contract.
// =============================================================================

TEST(EmbeddingRegistry, GetReturnsSamePointer) {
    RegistryFixture f;
    Eigen::VectorXf p = Eigen::VectorXf::Constant(8, 0.5f);
    f.run_tick(0, [&]() {
        f.bus.publish("reality.proprio.imu", make_token(7, p));
    });

    auto a = f.reg.get("reality.proprio.imu", 7);
    auto b = f.reg.get("reality.proprio.imu", 7);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(a.get(), b.get())
        << "Repeated get() of an unchanged entry must return the same pointer";
}

// =============================================================================
// 3. Eviction on pruned_ids.
//    When the EPM publishes a token whose pruned_ids includes a previously
//    cached node, the registry must drop that entry.  Topic-scoped: pruning
//    on one modality does not touch another modality's cache.
// =============================================================================

TEST(EmbeddingRegistry, EvictsOnPrunedIds) {
    RegistryFixture f;
    Eigen::VectorXf p = Eigen::VectorXf::Constant(4, 1.0f);

    // Populate two distinct nodes on TWO different topics.
    f.run_tick(0, [&]() {
        f.bus.publish("reality.video.retinal",  make_token(10, p));
        f.bus.publish("reality.video.retinal",  make_token(11, p));
        f.bus.publish("reality.proprio.imu",    make_token(10, p));
    });
    EXPECT_EQ(f.reg.size("reality.video.retinal"), 2u);
    EXPECT_EQ(f.reg.size("reality.proprio.imu"),   1u);

    // Now publish a retinal token that prunes node 10 (and reports a fresh
    // winner with no overlap).  Node 10 should disappear from retinal but
    // remain in proprio.
    f.run_tick(1, [&]() {
        f.bus.publish("reality.video.retinal", make_token(12, p, /*pruned=*/{10}));
    });
    EXPECT_EQ(f.reg.get("reality.video.retinal", 10), nullptr);
    EXPECT_NE(f.reg.get("reality.video.retinal", 11), nullptr);
    EXPECT_NE(f.reg.get("reality.video.retinal", 12), nullptr);
    EXPECT_NE(f.reg.get("reality.proprio.imu",  10), nullptr)
        << "Pruning on retinal must not evict proprio's cache entry";
}

// =============================================================================
// 4. Thread-safety smoke: concurrent reader + writer.
//    No correctness assertion; the test passes if there's no data race
//    (ThreadSanitizer would flag) and no crash.  Sanity check that the
//    shared_mutex is in place.
// =============================================================================

TEST(EmbeddingRegistry, ConcurrentReadersDoNotCrash) {
    RegistryFixture f;
    Eigen::VectorXf p = Eigen::VectorXf::Constant(16, 1.0f);

    std::atomic<bool> stop{false};
    std::thread reader([&]() {
        std::size_t reads = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            for (int id = 0; id < 50; ++id) {
                volatile auto e = f.reg.get("reality.video.retinal", id);
                (void)e;
                ++reads;
            }
        }
        EXPECT_GT(reads, 0u);
    });

    for (int t = 0; t < 200; ++t) {
        f.run_tick(uint64_t(t), [&]() {
            f.bus.publish("reality.video.retinal", make_token(t % 50, p));
        });
    }
    stop.store(true);
    reader.join();

    EXPECT_GT(f.reg.size("reality.video.retinal"), 0u);
}
