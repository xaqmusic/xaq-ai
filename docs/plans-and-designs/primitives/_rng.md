# Cross-Cutting: RNG Determinism Contract

**Applies to:** every Phase 1+ Ogma Core module that uses randomness.

---

## The Rule

Every stochastic operation in the Ogma Core MUST seed its RNG via `derive_rng(master_seed, namespace)` — NOT via `std::random_device`, NOT via thread-default seeding, NOT via wall-clock entropy. This is the contract that lets two runs with the same master seed and the same Bus inputs produce bit-identical outputs.

The C++ form mirrors v3's Python `src/ami_ogma_v3/_rng.py`:

```
ogma::namespace_seed(master_seed, namespace) -> uint64_t
ogma::derive_rng    (master_seed, namespace) -> std::mt19937_64
```

Algorithm (must match Python implementation byte-for-byte):

1. Compute `SHA-256(namespace.utf8_bytes())`.
2. Read the first 16 hex characters of the lowercase hex digest as a 64-bit unsigned integer (big-endian — i.e. `digest[0..7]` = high byte first).
3. XOR with `(uint64_t)master_seed`.
4. Mask to 64 bits (`& 0xFFFFFFFFFFFFFFFF`).

The result is the namespace-derived seed. `derive_rng()` constructs a `std::mt19937_64` seeded with this value; `namespace_seed()` returns the integer for components that need to pass it into a different RNG.

---

## Same-Language vs. Cross-Language Determinism

**Same-language determinism is required.** Two C++ runs with the same master seed and identical Bus inputs MUST produce bit-identical module outputs. This is a Phase 1 acceptance gate for every module's VV&A (and is exercised by `verify_determinism.py`'s C++ analog in Phase 2).

**Cross-language byte-exact determinism (Python ↔ C++) is NOT required** and not feasible without engineering investment we don't need:

- Python's `np.random.default_rng()` uses **PCG64**, not MT19937. Matching PCG64 in C++ would require pulling in pcg-cpp.
- Eigen and numpy do not guarantee bit-exact floating-point order even for matrix-vector multiplies of identical inputs (different reduction orders, different SIMD paths).
- Per the planning resolution, the Phase 2 thin-slice replay gate is **multi-criteria integration soundness + competence sanity**, not strict numerical parity. So strict cross-language RNG matching is the wrong investment.

What we DO require for Phase 2 cross-language replay:

1. The same `master_seed` produces *statistically* equivalent stochastic decisions on both sides (e.g. probe rates within 1%, motif baking timings within 10% of each other on identical input streams).
2. Within each language, identical seeds → identical outputs.

If a future phase demands strict cross-language determinism (e.g. for a hybrid Python-orchestrator/C++-runtime configuration), the upgrade path is well-defined: replace `std::mt19937_64` with a PCG64 in C++, plus reorder critical reductions in Eigen to match numpy's pairwise-summation. Not Phase 0–4 work.

---

## Namespace Conventions

Every module declares its RNG namespace at construction time. The convention is:

```
"<module_class>.<module_id>.<purpose>"
```

Examples:

| Module class | Purpose | Full namespace example |
|---|---|---|
| `ActionDecoder` | probe RNG | `decoder.action_decoder.probe` |
| `ActionDecoder` | EFE softmax tie-break | `decoder.action_decoder.efe_tiebreak` |
| `EPM` | JL random matrix | `epm.epm_retinal.jl_matrix` |
| `LateralVoter` | trust-tie-break | `voter.voter_0.trust_tiebreak` |
| `GNGRollout` | trajectory sampling | `rollout.roller_0.trajectories` |
| `SequenceGNG` | JL random matrix | `seqgng.seq_consensus.jl_matrix` |
| `DescendingPredictor` | weight init | `predictor.predictor_0.weights_init` |
| `NeurochemState` | (no stochastic ops) | n/a |
| `HomeostaticDrive` | (no stochastic ops) | n/a |
| `MotorRepertoire` | eviction tie-break | `motor_repertoire.repertoire.eviction_tiebreak` |

Adding a stochastic operation in a module REQUIRES adding a new namespace string. Reusing an existing namespace for a new purpose drifts the stream and breaks reproducibility for any test pinned to that namespace.

---

## C++ Header (Phase 0 deliverable, Phase 1 implementation)

The C++ entry point lives at `cpp_core/include/ogma/Rng.hpp` (Phase 1 file; Phase 0 deliverable is just this contract):

```cpp
namespace ogma {

uint64_t       namespace_seed(uint64_t master_seed, std::string_view ns);
std::mt19937_64 derive_rng   (uint64_t master_seed, std::string_view ns);

}  // namespace ogma
```

SHA-256 is taken from a vendored single-file implementation (e.g. `picosha2`) — no OpenSSL dependency on the hot path. Construction-time only, so the SHA cost is irrelevant.

---

## Verification

Phase 0 deliverable: a Python ↔ C++ comparison test under `cpp_core/tests/ogma/test_rng_parity.cpp`:

1. For seed = 42 and namespaces `["decoder.foo.probe", "epm.bar.jl_matrix", "voter.0.trust_tiebreak"]`, the C++ `namespace_seed` returns the integer that `python -c "from ami_ogma_v3._rng import namespace_seed; print(namespace_seed(42, '<ns>'))"` prints. **Required to pass byte-exact** — the SHA-256 + XOR derivation is bit-exact; only the post-derivation RNG differs.

2. With the same derived integer, `derive_rng(42, ns)` in Python yields a `np.random.Generator` and in C++ yields a `std::mt19937_64`. They produce DIFFERENT streams (different RNG algorithms). Document the streams' first 10 draws on each side as the canonical reproducibility table; future code changes that touch this contract must be checked against this table.

3. For each Phase 1 module, the per-module unit test pins one namespace draw at construction and asserts the value matches the canonical table.

The parity test is a Phase 0 deliverable — it ships with Section D and gets re-run on every commit thereafter.
