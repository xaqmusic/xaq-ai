# cpp_core — Zanshin C++ cognitive runtime

The C++ runtime: an in-process message bus, EPM (Episodic Predictive Module)
nodes, a Growing Neural Gas topology, a Lateral Voter, homeostatic drive, active
inference, and motor/navigation modules. Three libraries — `ogma_core` (v4
cognitive stack), `ogma_v3` (geometric encoders), `ogma_infra` (sensors +
control server).

## Naming: `ami_ogma::` is Zanshin

The C++ namespace `ami_ogma::` (and the `ogma`/`OGMA_` identifiers) is Zanshin's
**original internal codename**, retained in code for build/config stability.
`ami_ogma` == `ogma` == **Zanshin** — same project. See
[../docs/NAMING.md](../docs/NAMING.md).

## Build

```sh
cmake -S . -B build            # from cpp_core/, or -S cpp_core from the repo root
cmake --build build -j
```

Requires CMake ≥ 3.14, a C++17 compiler, and ZeroMQ. Eigen, nlohmann-json, and
GoogleTest are fetched automatically. OpenCV and X11 are optional.

The generic, IP-free audio front-end is `encoder_stft.{hpp,cpp}` (a `TODO`
notes the path to a full windowed-STFT + mel + JL pipeline). The bio-mimetic
cochlear encoder is not part of this repo.
