#include "Order.hpp"

#include <map>
#include <queue>
#include <set>

namespace bb {

OrderSuggestion topological_order(Graph const& g, Wiring const& w) {
    OrderSuggestion s;
    int n = int(g.size());
    std::map<std::string, int> index;
    for (int i = 0; i < n; ++i) index[g.id_of(size_t(i))] = i;

    std::vector<std::set<int>> succ;
    succ.resize(size_t(n));
    std::vector<int> indeg(size_t(n), 0);
    for (auto const& l : w.links) {
        if (l.feedback) continue;
        auto a = index.find(l.from_node), b = index.find(l.to_node);
        if (a == index.end() || b == index.end() || a->second == b->second) continue;
        if (succ[size_t(a->second)].insert(b->second).second) ++indeg[size_t(b->second)];
    }

    std::priority_queue<int, std::vector<int>, std::greater<int>> ready;
    std::vector<bool> done(size_t(n), false);
    for (int i = 0; i < n; ++i) if (indeg[size_t(i)] == 0) ready.push(i);
    int remaining = n;
    while (remaining > 0) {
        int u;
        if (ready.empty()) {
            // A cycle: keep the earliest remaining module where it is.
            u = -1;
            for (int i = 0; i < n; ++i) if (!done[size_t(i)]) { u = i; break; }
            std::string others;
            for (int v : succ[size_t(u)]) if (!done[size_t(v)] && succ[size_t(v)].count(u)) others += (others.empty() ? "" : ", ") + g.id_of(size_t(v));
            s.notes.push_back("cycle: " + g.id_of(size_t(u)) + (others.empty() ? " stays where it is" : " kept before " + others + " (mutual links, last-value reads)"));
            indeg[size_t(u)] = 0;
        } else {
            u = ready.top();
            ready.pop();
            if (done[size_t(u)]) continue;
        }
        done[size_t(u)] = true;
        --remaining;
        s.order.push_back(u);
        for (int v : succ[size_t(u)])
            if (!done[size_t(v)] && --indeg[size_t(v)] == 0) ready.push(v);
    }
    for (int i = 0; i < n; ++i) if (s.order[size_t(i)] != i) { s.changed = true; break; }
    if (!s.changed) s.notes.push_back("already in a topological order");
    return s;
}

} // namespace bb
