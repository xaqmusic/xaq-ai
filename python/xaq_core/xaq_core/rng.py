"""Per-component RNG derivation for reproducible v3 experiments.

A single master seed (from `--seed` CLI) is split into many independent
streams — one per stochastic component — so that adding a random call in
one component does not shift another's sequence. Same master seed +
same namespace → same stream, across runs and across code changes that
don't touch the namespaced component.

Use `hashlib.sha256` rather than the built-in `hash()` because the latter
is salted per Python process (PYTHONHASHSEED) and won't match across
runs.
"""
from __future__ import annotations

import hashlib

import numpy as np


def derive_rng(master_seed: int, namespace: str) -> np.random.Generator:
    """Return a `np.random.Generator` seeded deterministically from
    (master_seed, namespace). Different namespaces produce statistically
    independent streams from the same master."""
    h = int(hashlib.sha256(namespace.encode("utf-8")).hexdigest()[:16], 16)
    # XOR master into the namespace hash so seed=0 still varies per component.
    return np.random.default_rng((int(master_seed) ^ h) & 0xFFFFFFFFFFFFFFFF)


def namespace_seed(master_seed: int, namespace: str) -> int:
    """Same derivation as derive_rng, returned as a 64-bit integer. Use
    this when a component needs its own integer seed (e.g. to pass into
    an `std::mt19937` via C++ bindings, or into another library's RNG)."""
    h = int(hashlib.sha256(namespace.encode("utf-8")).hexdigest()[:16], 16)
    return (int(master_seed) ^ h) & 0xFFFFFFFFFFFFFFFF
