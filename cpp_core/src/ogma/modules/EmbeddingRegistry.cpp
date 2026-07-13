#include "ogma/modules/EmbeddingRegistry.hpp"

#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("EmbeddingRegistry: param '" + key + "' must be string");
}

} // namespace

EmbeddingRegistry::EmbeddingRegistry()  = default;
EmbeddingRegistry::~EmbeddingRegistry() = default;

std::string_view EmbeddingRegistry::type_name() const { return "EmbeddingRegistry"; }

std::vector<TopicSpec> EmbeddingRegistry::input_topics() const {
    return {
        TopicSpec{input_pattern_,
                  std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> EmbeddingRegistry::output_topics() const {
    return {};   // pure consumer; no published topics
}

ParamSchema EmbeddingRegistry::params_schema() const {
    return {
        {"input_pattern", ParamMutability::ConstructionOnly,
            "Trailing-dot prefix subscribed for population (default reality.)",
            ParamValue{std::string("reality.")}},
    };
}

ParamMap EmbeddingRegistry::current_params() const {
    ParamMap m;
    m["input_pattern"] = ParamValue{input_pattern_};
    return m;
}

void EmbeddingRegistry::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("EmbeddingRegistry requires a non-null Bus");

    apply_param(params, "input_pattern", [&](auto const& v){
        input_pattern_ = get_string(v, "input_pattern");
    });
    if (input_pattern_.empty() || input_pattern_.back() != '.')
        throw std::invalid_argument("EmbeddingRegistry: input_pattern must be a trailing-dot prefix");

    sub_ids_.push_back(bus_->subscribe(input_pattern_, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ this->handle_reality(t, p); }));
}

void EmbeddingRegistry::handle_reality(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;

    // Eviction first: if the EPM just pruned nodes, drop them before the
    // current winner write so a recycled ID can't briefly point at stale
    // data.  Topic-scoped eviction; pruned_ids on one modality never
    // affects another modality's cache.
    if (!rt->pruned_ids.empty()) {
        std::unique_lock lock(mtx_);
        auto cit = cache_.find(std::string(topic));
        if (cit != cache_.end()) {
            for (int pid : rt->pruned_ids) cit->second.erase(pid);
        }
    }

    // Cache populate.  Skip placeholder/bootstrap tokens (winner_id < 0)
    // and any token without a populated prototype.
    if (rt->winner_id < 0 || rt->winner_prototype.size() == 0) return;

    auto entry = std::make_shared<const Eigen::VectorXf>(rt->winner_prototype);
    std::unique_lock lock(mtx_);
    cache_[std::string(topic)][rt->winner_id] = std::move(entry);
}

std::shared_ptr<const Eigen::VectorXf>
EmbeddingRegistry::get(std::string_view topic, int node_id) const {
    if (node_id < 0) return nullptr;
    std::shared_lock lock(mtx_);
    auto cit = cache_.find(std::string(topic));
    if (cit == cache_.end()) return nullptr;
    auto nit = cit->second.find(node_id);
    if (nit == cit->second.end()) return nullptr;
    return nit->second;
}

std::size_t EmbeddingRegistry::size(std::string_view topic) const {
    std::shared_lock lock(mtx_);
    auto cit = cache_.find(std::string(topic));
    return cit == cache_.end() ? 0 : cit->second.size();
}

std::size_t EmbeddingRegistry::total_size() const {
    std::shared_lock lock(mtx_);
    std::size_t n = 0;
    for (auto const& [_, m] : cache_) n += m.size();
    return n;
}

nlohmann::json EmbeddingRegistry::snapshot_state() const {
    std::shared_lock lk(mtx_);
    nlohmann::json topics = nlohmann::json::object();
    for (auto const& [topic, inner] : cache_) {
        nlohmann::json m = nlohmann::json::object();
        for (auto const& [node_id, vec] : inner) {
            if (!vec) continue;
            nlohmann::json a = nlohmann::json::array();
            for (int i = 0; i < vec->size(); ++i) a.push_back((*vec)(i));
            m[std::to_string(node_id)] = std::move(a);
        }
        topics[topic] = std::move(m);
    }
    return nlohmann::json{
        {"version", 1},
        {"cache",   topics},
    };
}

void EmbeddingRegistry::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("EmbeddingRegistry::restore_state: unknown version " +
                                 std::to_string(version));
    }
    std::unique_lock lk(mtx_);
    cache_.clear();
    if (s.contains("cache") && s["cache"].is_object()) {
        for (auto it1 = s["cache"].begin(); it1 != s["cache"].end(); ++it1) {
            auto& inner = cache_[it1.key()];
            for (auto it2 = it1.value().begin(); it2 != it1.value().end(); ++it2) {
                auto vec = std::make_shared<Eigen::VectorXf>(int(it2.value().size()));
                for (size_t i = 0; i < it2.value().size(); ++i)
                    (*vec)(int(i)) = it2.value()[i].get<float>();
                inner[std::stoi(it2.key())] = vec;
            }
        }
    }
}

} // namespace ogma
