#pragma once

// =============================================================================
// EmbeddingRegistry.hpp  --  Phase 6.6.E shared embedding cache
// =============================================================================
//
// Single-owner cache mapping (modality_topic, gng_node_id) → shared_ptr to
// that node's prototype embedding.  Subscribes to a configurable reality.*
// prefix and writes one entry per unique winner observed; evicts on
// `pruned_ids`.  Other modules (LateralVoter surprise modulator, future
// Premotor predicted-pathway consumers) inject a shared_ptr to this
// registry via the graph wiring and read by `get(topic, node_id)`.
//
// Why a dedicated module:
//   - The InProcessBus already hands out shared_ptr<const Message>; reusing
//     that idiom for embeddings means a per-subscriber cache would duplicate
//     ~MB-scale tensors across every consumer with no shared invalidation.
//   - Centralising eviction on `pruned_ids` keeps cache coherence in one
//     place rather than relying on every consumer to track GNG mutations.
//
// Phase 1 scope: stores a heap copy of `winner_prototype` at insert time.
// Phase 2 zero-copy: when EPM holds shared_ptr<const Eigen::VectorXf> per
// node in its own GNG and ships the same pointer in winner_prototype, the
// registry can hold the same pointer — no extra allocation.

#include <memory>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>

#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class EmbeddingRegistry : public Module {
public:
    EmbeddingRegistry();
    ~EmbeddingRegistry() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t /*tick_id*/) override {}   // pure handler-driven; no per-tick work

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // Lookup.  Returns nullptr on miss.  Thread-safe; safe to call from any
    // module's tick().  Two consecutive `get(topic, id)` calls return the
    // same shared_ptr (zero-copy across consumers) until that node is
    // overwritten or evicted.
    std::shared_ptr<const Eigen::VectorXf>
        get(std::string_view topic, int node_id) const;

    // Telemetry: number of cached entries for one topic (0 if unknown).
    std::size_t size(std::string_view topic) const;

    // Telemetry: total cached entries across all topics.
    std::size_t total_size() const;

private:
    void handle_reality(std::string_view topic, MessagePtr payload);

    std::string  input_pattern_ = "reality.";  // trailing-dot prefix subscribe

    mutable std::shared_mutex mtx_;
    std::unordered_map<std::string,
        std::unordered_map<int,
            std::shared_ptr<const Eigen::VectorXf>>> cache_;
};

} // namespace ogma
