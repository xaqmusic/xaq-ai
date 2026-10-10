# AGENTS.md — orientation for coding agents

> **Start with [`CLAUDE.md`](CLAUDE.md).** This file covers repo layout, naming, and the
> generic build. CLAUDE.md covers *how to actually build in this project* — the rewrite
> rule (the design habit this framework requires and that a coding agent's defaults get
> wrong), the A/B protocol, and the picrawler build/run recipe. Then
> [`docs/brain_building_doctrine.md`](docs/brain_building_doctrine.md) for the method.

## Naming: `ami_ogma` == `ogma` == xaq

xaq began as an internal project codenamed **AMI-Ogma** ("ogma" /
"ami_ogma") and was renamed **xaq** for its public release. To avoid
churning stable, hard-to-migrate identifiers, the following intentionally keep
the old name and are **not** bugs:

- The C++ namespace `ami_ogma::` (all of `cpp_core/`).
- The Godot addon path `res://addons/ami_ogma/`, the built extension
  `ami_ogma_host.so`, and the GDExtension entry symbol `ami_ogma_library_init`.
- Environment variables and config keys prefixed `OGMA_` (e.g. `OGMA_CELL_CONFIG`).

If you see `ami_ogma` or `ogma` anywhere in this repo, read it as **xaq** —
they refer to the same project. New Python code uses the `xaq` /
`xaq_core` packages; there is no plan to rename the C++/Godot identifiers.
(Canonical copy: [docs/NAMING.md](docs/NAMING.md).)

## Repo orientation

- `cpp_core/` — the C++ cognitive runtime (`ami_ogma::` namespace): message bus,
  EPM nodes, GNG, Lateral Voter, active inference, motor/nav. Builds with CMake +
  FetchContent (needs ZeroMQ). Generic audio front-end is `encoder_stft.*`.
- `godot_host/` — Godot 4.6 GDExtension embodying the runtime (the Cell env). The
  `.so` is rebuilt from source (gitignored).
- `brain_builder/` — the Dear ImGui desktop app for wiring a brain from scratch
  (palette of every registered module, the body's sources/sinks as nodes,
  drag-to-wire, validate, dry-run, publish). Links `cpp_core` directly; edits the
  GraphConfig JSON the hosts run unchanged.
- `python/xaq_core/` — shared substrate (bus, torch predictor/memory, RNG,
  logging protocol).
- `python/xaq/` — the Python engine + the generic STFT audio encoder + the
  `make_encoder` registry (`xaq.encoders` entry-point seam).
- `docs/` — the Cell report (`docs/reports/`), the method
  (`docs/brain_building_doctrine.md`), research summaries of the papers xaq
  draws on (`docs/research-summaries/`), and this naming note (`docs/NAMING.md`).

## Build & test (quick)

**First-time system prerequisites** (none of this is auto-installed by the commands
below):

- `libzmq3-dev` (ZeroMQ headers + pkg-config file — `cpp_core`'s CMake does
  `pkg_check_modules(ZMQ REQUIRED libzmq)` and fails without it even if the
  `libzmq5` runtime package is already present). `sudo apt install libzmq3-dev`.
- A **Godot 4.6.2** binary, on `PATH` as `godot4`. It is not packaged by apt/snap in
  a version that matches — download `Godot_v4.6.2-stable_linux.x86_64` from
  godotengine.org and symlink it: `ln -s /path/to/Godot_v4.6.2-stable_linux.x86_64
  ~/.local/bin/godot4` (must be exactly 4.6.2 — `godot_host/extension_api.json` and
  `gdextension_interface.h` were dumped from that binary; see the comment atop
  `godot_host/CMakeLists.txt`).
- `pytest` itself — it is intentionally **not** a declared dependency of either
  Python package (it's a dev/test tool, not runtime), so `pip install -e ...` alone
  will not provide it: `pip install pytest`.

```sh
cmake -S cpp_core -B cpp_core/build && cmake --build cpp_core/build -j
cmake -S godot_host -B godot_host/build && cmake --build godot_host/build -j
python3 -m venv .venv && source .venv/bin/activate   # repo root; .venv/ is gitignored
pip install -e python/xaq_core -e python/xaq pytest && pytest python/xaq/tests
```

The `godot_host` build fetches `godot-cpp` from GitHub (FetchContent) and compiles
its full binding set — expect several minutes on a first build even though only
`src/AmiOgmaPlugin.cpp` and `src/OgmaBrain.cpp` are this project's own code. The
resulting `ami_ogma_host.so` auto-copies into `godot_host/project/addons/ami_ogma/`.

## Conventions

- Contributions are under Apache-2.0 with DCO sign-off (`git commit -s`). See
  [CONTRIBUTING.md](CONTRIBUTING.md).
