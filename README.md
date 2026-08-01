# xaq-ai

**xaq** (pronounced "ex-ay-cue") is an open research framework for **embodied
active inference**: a System-1
"subconscious" substrate for robots and simulated agents, rather than a
language-model deliberator. It learns online by minimizing prediction error, is
driven by internal homeostatic needs instead of an external reward signal, and
touches the world only through its own sensors and actuators.

The core idea is that a competent agent can be assembled from few or many small
inference loops — each responsible for one hidden feature of the world —
coordinated by a shared, trust-weighted state bus and a policy selector.

## Why?

This project began as an effort to build a self organizing smart sensor.  But my personal interests in the nature of intelligence, origin of life, and robotics took over and began the relentless pursuit of a truly hard problem: applying the principles and metaphors of Active Inference, Assembly Theory, and The Playful Machine into a working foundation for physical agents.  I am impressed yet bored with LLMs.  Reinforcement Learning works, but it's clunky.  I want to see a physical agent that is truly a member of this universe that learns by palpating its environment at every moment to build its own robust yet flexible model of its world in order to resolve internal states by acting on external states.  A key constraint is I want this substrate to use commodity computational resources (low power mobile processors, single board computers, digital signal processors) while delivering an "aliveness" and utility that extends the physical structure of the mechanism beyond what we see today in robotics.  I see behaviors in the insects that inhabit my back yard that put the capabilities of every drone and humanoid robot to shame.  I feel those behaviors are possible with the right dose of creativity, humility, and perseverance. 

It is hard to quantify the advantage Nature has in solving these problems by Her use of large numbers (in all dimensions) but I feel our level of computation has reached a point where, with the proper recipe and plumbing, the analogies life has presented to us can be represented in silicon.  This project is biomimetic at high scope and von Neumann at the lowest with some modules having vague analogies to biology (EPM ~ cortical column) while others used biologic plausibility as a foundation for their architecture.  The prize is a functional agent today, not an exploration in biological simulation. 

These are lofty goals but I feel the components are right in front of us.  I recognize I am standing on the shoulders of giants much taller than myself to enable this projects existence.  I am not a mathematician, a machine learning academic, or a competent coder.  I am a musician, a signal processing designer, tinkerer, builder, and in possession of a wide yet shallow body of knowledge with decades of experience in tech.  With that being said, my approach to math and statistical concepts in this project are from first principles and black-box probing to fully understand the applications and benefits for their target functions.  My current coding and academic partners are Claude and the scientific method.  I hope you can join me as I sure as hell need the help.

The cool thing about very hard problems that vex the greatest minds on this planet is almost anyone can take a solid crack at them while knowing success is far from guaranteed.  There is an element of danger (looking stupid in front of people you respect) that is balanced by the lure of sweet novelty.  Therefore we go forth and build like a proper Fool.

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
  yourself.  This is a living document.
- **[docs/](docs/)** — full documentation index, including
  [research summaries](docs/research-summaries/) of the papers xaq builds on.

## Where we are going

This project is in very early stages (started at the beginning of 2026) so there are a lot of areas that need work and questions unanswered:

- What is the recipe for simple loops scale automatically (mitosis) for improved reasoning?
- How can we build an evolutionary scaffold to accelerate development?
- How can the substrate and each module be optimized for current hardware?
- What does an ASIC for this substrate look like?
- How can we leverage decentralized compute for this substrate (think octopus brains)?
- How can Reality Token fusion (lateral voting) be improved or changed for improved representations?
- What is the best way for slow and fast loops to interact (jitter buffers etc)?
- What is the most transferable method of action decoding?
- How can we setup proper internal goals and rewards to give physical agents utility in human endeavors?

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
