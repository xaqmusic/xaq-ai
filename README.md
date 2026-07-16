# xaq

**xaq** (pronounced "ex-ay-cue") is an open research framework for **embodied
active inference**: a System-1
"subconscious" substrate for robots and simulated agents, rather than a
language-model deliberator. It learns online by minimising prediction error, is
driven by internal homeostatic needs instead of an external reward signal, and
touches the world only through its own sensors and actuators.

The core idea is that a competent agent can be assembled from several small
inference loops — each responsible for one hidden feature of the world —
coordinated by a shared, trust-weighted state bus and a single policy selector.

## What's here

| Layer | What it is |
|---|---|
| **`cpp_core/`** | The C++ cognitive runtime: an in-process message bus, EPM (Episodic Predictive Module) nodes, a Growing Neural Gas topology, a Lateral Voter (Hebbian cross-modal consensus), homeostatic drive, active-inference policy selection, and motor/navigation modules. |
| **`godot_host/`** | A Godot 4.6 GDExtension that embodies the runtime in simulated worlds — including **the Cell**, a single-celled forager used as a falsification testbed (see the report below). |
| **`python/xaq_core/`** | Shared substrate: the socket/ZeroMQ message bus, a torch predictor + episodic memory, export-parity checks, and the logging protocol. |
| **`python/xaq/`** | The Python engine: EPM nodes, GNG, lateral voting, active inference / global workspace, motor & navigation, and a **generic STFT audio modality** so multi-modal fusion is demonstrable out of the box. |

## A note on the name

You will see `ami_ogma` and `ogma` in the code — the C++ namespace `ami_ogma::`,
the Godot addon `addons/ami_ogma/` and `ami_ogma_host.so`, and `OGMA_*` env vars.
That is xaq's **original internal codename**, kept in those identifiers so
existing builds and saved configs stay stable. **`ami_ogma` and xaq are the
same project.** New Python code lives in the `xaq` / `xaq_core` packages.
See [docs/NAMING.md](docs/NAMING.md).

## Start here

- **[docs/reports/cell_markov_blanket_loops_report.md](docs/reports/cell_markov_blanket_loops_report.md)** —
  *The Cell Navigator: A Falsification Testbed for Markov-Blanket-Loop Active
  Inference.* An evidence report whose contribution is a **discipline** for
  building embodied agents — one that overturns two of its own headline claims
  once they are properly powered.
- **[docs/brain_building_doctrine.md](docs/brain_building_doctrine.md)** — the
  method the report tests: how to compose predictive loops without fooling
  yourself.
- **[docs/](docs/)** — full documentation index, including
  [research summaries](docs/research-summaries/) of the papers xaq builds on.

## Build

**C++ runtime** (needs CMake ≥ 3.14, a C++17 compiler, and ZeroMQ; Eigen /
nlohmann-json / GoogleTest are fetched automatically):

```sh
cmake -S cpp_core -B cpp_core/build
cmake --build cpp_core/build -j
```

**Godot host** (rebuilds the GDExtension `.so`, which is intentionally not
committed — it is regenerated from source on every clone):

```sh
cmake -S godot_host -B godot_host/build
cmake --build godot_host/build -j     # produces + installs project/addons/ami_ogma/*.so
```

**Python engine:**

```sh
pip install -e python/xaq_core -e python/xaq
python -c "from xaq.encoders import make_encoder; import numpy as np; \
           print(make_encoder('audio').encode(np.zeros(4800,'float32')).shape)"
```

## Extending perception: the encoder registry

Perception in xaq is pluggable. Every sensory modality — vision, touch,
proprioception, audio, or something you invent — enters the system through a
*frozen encoder* that maps raw input to a latent vector, and encoders are
resolved through a small registry (`xaq.encoders`) keyed by modality name
and group.

xaq ships generic, dependency-free encoders out of the box (Johnson–
Lindenstrauss projections for visual modalities, a proprioceptive encoder, and a
plain STFT encoder for audio), so multi-modal fusion works with nothing else
installed. Any package can register additional encoders — a new sensor, a
higher-fidelity front-end, or an entirely new modality — through the
`xaq.encoders` setuptools entry-point group, with **no change to xaq
itself**; `make_encoder("<modality>")` then resolves to whatever is registered.
See `python/xaq/xaq/encoders/`.

## Intended use and ethical stance

xaq is licensed under the [Apache License 2.0](LICENSE), which places **no
restriction on the field of use**. The statement below is an expression of the
project's values and a request to the people who build with it. It is **not an
additional license term, not a condition of the license, and does not limit any
right the Apache License grants you.**

We built xaq to advance open research in active inference and embodied
intelligence — and to help people build robots that are capable, transparent
partners: assistive devices, cooperative and educational machines, scientific
instruments, and defensive or safety-critical systems that keep people out of
harm's way.

In that spirit, we ask that you **do not use xaq to develop autonomous
weapons** — systems that select and apply force to human targets without
meaningful human control — nor for mass surveillance or any application designed
to harm, suppress, or deny the rights of people.

We chose a permissive license on purpose. A use-restriction in the license would
only burden the good-faith users who read it, while doing nothing to stop anyone
determined to ignore it; it would also make xaq harder to build on, teach
with, and contribute to. We would rather trust our community and state our
position plainly than encumber everyone to constrain a few.

## License

Apache-2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE). Contributions are
accepted under the Developer Certificate of Origin; see
[CONTRIBUTING.md](CONTRIBUTING.md).

*xaq is a research framework, provided as-is. It is not legal or safety
advice, and nothing here is a warranty of fitness for any particular purpose.*
