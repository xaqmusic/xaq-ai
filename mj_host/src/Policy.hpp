#pragma once

// =============================================================================
// Policy — one ONNX network, validated at load
// =============================================================================
//
// A thin wrapper over an ONNX Runtime session, and deliberately thin: this is a
// SCAFFOLD (models/microduck/scaffolds/README.md), not part of the brain, and it
// should stay small enough that removing it later is obvious.
//
// Everything is checked at load rather than at inference, on the same reasoning
// duck-control/src/policy.rs gives: a bundle with the wrong observation width must
// fail while the robot is standing still and the caller can be told why — not
// sixty ticks later, mid-stride.

#include <array>
#include <memory>
#include <string>

#include "Observation.hpp"

namespace mjhost {

class Policy {
public:
    // Throws std::runtime_error naming both widths on a shape mismatch, which is
    // what turns "wrong policy file" and "wrong host build" from an indistinguishable
    // pair into a diagnosis.
    explicit Policy(const std::string& onnx_path);
    ~Policy();

    Policy(const Policy&) = delete;
    Policy& operator=(const Policy&) = delete;

    std::array<float, kActionLen> infer(const std::array<float, kObsLen>& obs);

    const std::string& path() const { return path_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::string path_;
};

}  // namespace mjhost
