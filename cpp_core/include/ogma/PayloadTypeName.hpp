#pragma once

// =============================================================================
// PayloadTypeName.hpp  --  std::type_index → canonical short string
// =============================================================================
//
// TopicSpec carries a `std::type_index` for the Bus-payload type so the
// scheduler can validate publisher↔subscriber type agreement at construction
// time.  For UI/diag surfaces (graph panel ports coloured by payload type,
// inspector dispatch by module type) we need a friendly, stable string name
// per payload type.  RTTI mangled names are compiler-specific; this file is
// the canonical mapping the host bindings expose to GDScript / Qt.
//
// Returns the short type name (e.g. "RealityToken", "ConsensusToken").
// Returns "Unknown" for any type_index not registered here — host code
// should treat that as a graceful fallback (e.g. paint the port grey).

#include <typeindex>
#include <string>

namespace ogma {

std::string payload_type_name(std::type_index t);

} // namespace ogma
