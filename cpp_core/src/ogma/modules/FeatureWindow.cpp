#include "ogma/modules/FeatureWindow.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("FeatureWindow param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("FeatureWindow param '" + key + "' must be a string");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("FeatureWindow param '" + key + "' must be a list of strings");
}
} // namespace

FeatureWindow::FeatureWindow()  = default;
FeatureWindow::~FeatureWindow() = default;

std::string_view FeatureWindow::type_name() const { return "FeatureWindow"; }

std::vector<TopicSpec> FeatureWindow::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& s : src_) v.push_back(TopicSpec{s.topic, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> FeatureWindow::output_topics() const {
    return {TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))}};
}

ParamSchema FeatureWindow::params_schema() const {
    return {
        {"sources", ParamMutability::ConstructionOnly,
         "Channels to summarise: 'topic' (every value) or 'topic:i,j' (those indices), each a ProprioToken topic.",
         ParamValue{std::vector<std::string>{}}},
        {"output_topic", ParamMutability::ConstructionOnly, "Published ProprioToken: per source, its channels' means then sds.",
         ParamValue{std::string("reality.proprio.gait_window")}},
        {"window", ParamMutability::HotMutable, "Ticks summarised.", ParamValue{int64_t{100}}},
        {"publish_every", ParamMutability::HotMutable, "Publish once every this many ticks (once the window is full).", ParamValue{int64_t{25}}},
    };
}

ParamMap FeatureWindow::current_params() const {
    return {{"sources", ParamValue{spec_}}, {"output_topic", ParamValue{output_topic_}},
            {"window", ParamValue{int64_t(window_)}}, {"publish_every", ParamValue{int64_t(publish_every_)}}};
}

void FeatureWindow::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("FeatureWindow requires a non-null Bus");
    apply_param(params, "sources",       [&](auto const& v){ spec_ = get_string_vec(v, "sources"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "window",        [&](auto const& v){ window_ = std::max(2, int(get_int(v, "window"))); });
    apply_param(params, "publish_every", [&](auto const& v){ publish_every_ = std::max(1, int(get_int(v, "publish_every"))); });
    if (spec_.empty()) throw std::invalid_argument("FeatureWindow: sources must name at least one channel");
    for (auto const& sp : spec_) {
        Source s;
        const auto c = sp.find(':');
        s.topic = sp.substr(0, c);
        if (c != std::string::npos) {
            std::stringstream ss(sp.substr(c + 1));
            std::string tok;
            while (std::getline(ss, tok, ',')) if (!tok.empty()) s.idx.push_back(std::stoi(tok));
        }
        src_.push_back(std::move(s));
    }
    for (size_t i = 0; i < src_.size(); ++i)
        sub_ids_.push_back(bus_->subscribe(src_[i].topic, SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ handle(i, p); }));
}

void FeatureWindow::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "window")        window_ = std::max(2, int(get_int(value, k)));
    else if (k == "publish_every") publish_every_ = std::max(1, int(get_int(value, k)));
    else throw std::invalid_argument("FeatureWindow param '" + k + "' is ConstructionOnly or unknown");
}

void FeatureWindow::handle(size_t s, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() == 0) return;
    Source& src = src_[s];
    if (src.idx.empty()) { src.last = pt->values; }
    else {
        src.last.resize(int(src.idx.size()));
        for (size_t j = 0; j < src.idx.size(); ++j) {
            const int k = src.idx[j];
            src.last[int(j)] = (k >= 0 && k < pt->values.size()) ? pt->values[k] : 0.0f;
        }
    }
    src.seen = true;
}

void FeatureWindow::tick(uint64_t tick_id) {
    ++ticks_;
    // Every source contributes its latest value each tick (held between readings), once each has been seen.
    for (auto const& s : src_) if (!s.seen) return;
    for (auto& s : src_) {
        s.win.push_back(s.last);
        while (int(s.win.size()) > window_) s.win.pop_front();
    }
    if (int(src_[0].win.size()) < window_ || ticks_ % publish_every_ != 0) return;
    std::vector<float> out;
    for (auto const& s : src_) {
        const int d = int(s.last.size());
        Eigen::VectorXf m = Eigen::VectorXf::Zero(d), q = Eigen::VectorXf::Zero(d);
        for (auto const& v : s.win) if (v.size() == d) { m += v; q += v.cwiseProduct(v); }
        const float n = float(s.win.size());
        m /= n;
        const Eigen::VectorXf sdv = (q / n - m.cwiseProduct(m)).cwiseMax(0.0f).cwiseSqrt();
        for (int i = 0; i < d; ++i) out.push_back(m[i]);
        for (int i = 0; i < d; ++i) out.push_back(sdv[i]);
    }
    out_dim_ = int(out.size());
    auto tok = std::make_shared<ProprioToken>();
    tok->tick_id = tick_id; tok->producer_id = id().empty() ? std::string("feature_window") : std::string(id());
    tok->sensor = "feature_window";
    tok->values = Eigen::Map<Eigen::VectorXf>(out.data(), Eigen::Index(out.size()));
    last_out_ = tok->values;
    bus_->publish(output_topic_, tok);
    ++published_;
}

nlohmann::json FeatureWindow::diag_snapshot() const {
    return nlohmann::json{{"output_dim", out_dim_}, {"published", published_}, {"ticks", ticks_},
                          {"last", std::vector<double>(last_out_.data(), last_out_.data() + last_out_.size())}};
}

} // namespace ogma
