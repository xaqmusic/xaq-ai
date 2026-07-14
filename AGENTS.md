# AGENTS.md — orientation for coding agents

## Naming: `ami_ogma` == `ogma` == Zanshin

Zanshin began as an internal project codenamed **AMI-Ogma** ("ogma" /
"ami_ogma") and was renamed **Zanshin** for its public release. To avoid
churning stable, hard-to-migrate identifiers, the following intentionally keep
the old name and are **not** bugs:

- The C++ namespace `ami_ogma::` (all of `cpp_core/`).
- The Godot addon path `res://addons/ami_ogma/`, the built extension
  `ami_ogma_host.so`, and the GDExtension entry symbol `ami_ogma_library_init`.
- Environment variables and config keys prefixed `OGMA_` (e.g. `OGMA_CELL_CONFIG`).

If you see `ami_ogma` or `ogma` anywhere in this repo, read it as **Zanshin** —
they refer to the same project. New Python code uses the `zanshin` /
`zanshin_core` packages; there is no plan to rename the C++/Godot identifiers.
(Canonical copy: [docs/NAMING.md](docs/NAMING.md).)

## Repo orientation

- `cpp_core/` — the C++ cognitive runtime (`ami_ogma::` namespace): message bus,
  EPM nodes, GNG, Lateral Voter, active inference, motor/nav. Builds with CMake +
  FetchContent (needs ZeroMQ). Generic audio front-end is `encoder_stft.*`.
- `godot_host/` — Godot 4.6 GDExtension embodying the runtime (the Cell env). The
  `.so` is rebuilt from source (gitignored).
- `python/zanshin_core/` — shared substrate (bus, torch predictor/memory, RNG,
  logging protocol).
- `python/zanshin/` — the Python engine + the generic STFT audio encoder + the
  `make_encoder` registry (`zanshin.encoders` entry-point seam).
- `docs/` — the Cell report (`docs/reports/`), the method
  (`docs/brain_building_doctrine.md`), research summaries of the papers Zanshin
  draws on (`docs/research-summaries/`), and this naming note (`docs/NAMING.md`).

## Build & test (quick)

```sh
cmake -S cpp_core -B cpp_core/build && cmake --build cpp_core/build -j
cmake -S godot_host -B godot_host/build && cmake --build godot_host/build -j
pip install -e python/zanshin_core -e python/zanshin && pytest python/zanshin/tests
```

## Conventions

- Contributions are under Apache-2.0 with DCO sign-off (`git commit -s`). See
  [CONTRIBUTING.md](CONTRIBUTING.md).
- The bio-mimetic audio encoder is developed separately (private); audio work
  here builds on the generic STFT path or plugs in via the `zanshin.encoders`
  entry-point group. Do not add proprietary audio code to this repo.
