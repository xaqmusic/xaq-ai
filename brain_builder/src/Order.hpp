#pragma once
// Execution order is behaviour: the scheduler ticks modules in array order,
// one level.  This suggests a topological order over the forward links and
// says where a cycle kept the present order; applying it is the operator's
// choice and one undo step.
#include <string>
#include <vector>

#include "Graph.hpp"
#include "Wiring.hpp"

namespace bb {

struct OrderSuggestion {
    std::vector<int>         order;     // new position → old module index
    std::vector<std::string> notes;
    bool                     changed = false;
};

OrderSuggestion topological_order(Graph const& g, Wiring const& w);

} // namespace bb
