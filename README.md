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

YouTube Playlist https://www.youtube.com/watch?v=862uRoQOtKk&list=PLRTI993J63iI&index=1

## Where it stands (October 2026)

Two robots carry the work now. Both run the same brain code, and neither is
trained with a reward.

### The PiCrawler: A quadruped that learns how to walk

The Sunfounder PiCrawler is a small 12-servo quadruped on a Raspberry Pi 5. In simulation
(Godot) the brain learned a walking coordination based on its own body dynamics.  The hard parts turned out to be senses rather than policies: 
it climbed a hump only after it was given a belly rangefinder, and it held a heading
once it could integrate its own yaw.  Modifications and sensors then needed to be made
to the PiCrawler hardware to match the required senses needed in sim. 

<img width="262" height="146" alt="picrawler_rooty" src="https://github.com/user-attachments/assets/4e3a5f84-8197-40f8-ba36-9faaba5679d4" />

In October 2026 that brain, with its gains frozen from simulation and fed only
what the physical robot can measure (its own servo commands, an IMU, force
sensors in its feet, a belly time-of-flight sensor), walked the real robot, with
a gait that looks like the simulated one. The current limit is power, not the
brain: fast moves draw more current than the servo board can supply, and the
board resets about once every 10 to 40 seconds.

- [docs/reports/picrawler_gait_loop_findings.md](docs/reports/picrawler_gait_loop_findings.md):
  the emergent, self-rescuing gait.
- [docs/reports/picrawler_lever_ledger.md](docs/reports/picrawler_lever_ledger.md):
  every change tried, what was kept, what was refuted, and in what context.
- [docs/plans-and-designs/picrawler_sim2real_port.md](docs/plans-and-designs/picrawler_sim2real_port.md)
  and [pi_host/](pi_host/README.md): the hardware port and the code that runs on the robot.

### The Microduck: ten minutes of fun in a playroom

[Microduck](https://github.com/pollen-robotics/microduck) is Pollen Robotics'
25 cm, 800 g two-legged robot. Pollen ships trained neural networks that walk it,
kick, and reach down to the floor. xaq-ai does not replace those. It sits above them
as the part that decides what to do: where to look, where to walk, what to walk
up to, and when to give up on something.

<img width="1668" height="936" alt="Screenshot From 2026-10-11 19-39-02" src="https://github.com/user-attachments/assets/cd1a5211-b2aa-4712-a06f-b6570dec6075" />


In simulation (MuJoCo, on Pollen's own model files), the duck is left alone in a
4 m playroom with toys, furniture and a moving toy train. It sees through the
8×8 depth sensor in its head, builds its own map of what is there, tells small
things from walls, walks over to them, kicks and pecks them, chases the train,
looks around at parts of the room it hasn't seen, and when it gets stuck it sits,
looks, and walks out. Behaviour is scored by how much of a ten-minute run is
spent doing something interesting versus staring at a wall, over 24 seeded runs
per candidate, and nothing is promoted until it has also been watched.

Earlier, the same approach learned to stand with no standing policy at all: it
babbles each joint, learns what each one does to its balance, and ends up
catching a 2 N shove 35 times out of 36
([the standing report](docs/reports/microduck_standing_report.md)).

- The design, proposed to Pollen: [pollen-robotics/microduck#381](https://github.com/pollen-robotics/microduck/pull/381),
  with narrated videos of [the proposal and a demo](https://youtu.be/W2jMf6EhW0k)
  and of [the interpretability and debugging tools](https://youtu.be/F9kEaafD-vo),
  and a companion walkthrough, [Inside the MicroDuck Brain](https://claude.ai/artifact/Dirv1ZWg62pjQVM3SCvkzh).
- How Pollen's runtime works, from the outside in:
  [docs/reports/microduck_pollen_architecture_report.md](docs/reports/microduck_pollen_architecture_report.md).
- The working record: [the port plan](docs/plans-and-designs/microduck/port_plan.md) (start at "Resume here").

Simulation only so far.  A proper planning and object permanance loop is in progress and two
things it uses (moving the head directly, standing still during a stop) are not
yet part of Pollen's public API.

### Where it started: the Cell

Before either robot, a simulated single-celled forager was used to test the
method itself, and the report on it overturns two of its own headline claims
once they are properly powered. That discipline is what both robots are built
under ([the Cell report](docs/reports/cell_markov_blanket_loops_report.md)).

  <img width="1172" height="403" alt="cel1" src="https://github.com/user-attachments/assets/1a554b9d-d21b-44c1-9e40-676e2f03c795" />

## Why?

This project began as an effort to build a self organizing smart sensor.  But my personal interests in the nature of intelligence, origin of life, and robotics took over and began the relentless pursuit of a truly hard problem: applying the principles and metaphors of Active Inference, Assembly Theory, and The Playful Machine into a working foundation for physical agents.  I am impressed yet bored with LLMs.  Reinforcement Learning works, but it's clunky.  I want to see a physical agent that is truly a member of this universe that learns by palpating its environment at every moment to build its own robust yet flexible model of its world in order to resolve internal states by acting on external states.  A key constraint is I want this substrate to use commodity computational resources (low power mobile processors, single board computers, digital signal processors) while delivering an "aliveness" and utility that extends the physical structure of the mechanism beyond what we see today in robotics.  I see behaviors in the insects that inhabit my back yard that put the capabilities of every drone and humanoid robot to shame.  I feel those behaviors are possible with the right dose of creativity, humility, and perseverance. 

It is hard to quantify the advantage Nature has in solving these problems by Her use of large numbers (in all dimensions) but I feel our level of computation has reached a point where, with the proper recipe and plumbing, the analogies life has presented to us can be represented in silicon.  This project is biomimetic at high scope and von Neumann at the lowest with some modules having vague analogies to biology (EPM ~ cortical column) while others used biologic plausibility as a foundation for their architecture.  The prize is a functional agent today, not an exploration in biological simulation. 

These are lofty goals but I feel the components are right in front of us.  I recognize I am standing on the shoulders of giants much taller than myself to enable this projects existence.  I am not a mathematician, a machine learning academic, or a competent coder.  I am a musician, a signal processing designer, tinkerer, builder, and in possession of a wide yet shallow body of knowledge with decades of experience in tech.  With that being said, my approach to math and statistical concepts in this project are from first principles and black-box probing to fully understand the applications and benefits for their target functions.  My current coding and academic partners are Claude and the scientific method.  I hope you can join me as I sure as hell need the help.

The cool thing about very hard problems that vex the greatest minds on this planet is almost anyone can take a solid crack at them while knowing success is far from guaranteed.  There is an element of danger (looking stupid in front of people you respect) that is balanced by the lure of sweet novelty.  Therefore we go forth and build like a proper Fool.

## What's here

**All development is currently happening in Linux.**  Windows support (runtime and inspector) is coming soon.

| Layer | What it is |
|---|---|
| **`cpp_core/`** | The C++ cognitive runtime: an in-process message bus, EPM (Episodic Predictive Module) nodes, a Growing Neural Gas topology, a Lateral Voter (Hebbian cross-modal consensus), homeostatic drive, active-inference policy selection, and motor/navigation modules. Every host below runs this same core. |
| **`mj_host/`** | The **Microduck** host: MuJoCo physics on Pollen's unmodified model files, Pollen's trained policies run through ONNX Runtime, and the brain above them. `./mj_host/run.sh gates` is its health check. See [mj_host/README.md](mj_host/README.md). |
| **`godot_host/`** | A Godot 4.6 GDExtension that embodies the runtime in simulated worlds: **the PiCrawler** quadruped and **the Cell**, a single-celled forager used as a falsification testbed. |
| **`pi_host/`** | The **PiCrawler** hardware host on a Raspberry Pi 5: the servo driver and its safety envelope, the bench daemon, and the path that lets a brain drive the real robot. See [pi_host/README.md](pi_host/README.md). |
| **`brain_builder/`** | A graphical editor for wiring a brain: every module type on a palette, each body's sensors and motors as nodes, drag-to-wire, validate, dry-run. See [brain_builder/README.md](brain_builder/README.md). |
| **`tools/`** | The live [inspector](tools/xaq_inspector/README.md) (watch any module inside a running brain), the duck's launcher and viewer, and [xaq_voice](tools/xaq_voice/README.md), which turns the brain's prediction error into sound. |
| **`python/xaq_core/`** | Shared substrate: the socket/ZeroMQ message bus, a torch predictor + episodic memory, export-parity checks, and the logging protocol. |
| **`python/xaq/`** | The Python engine: EPM nodes, GNG, lateral voting, active inference / global workspace, motor & navigation, and a **generic STFT audio modality** so multi-modal fusion is demonstrable out of the box. |

<img width="1854" height="1043" alt="Screenshot From 2026-10-11 19-42-28" src="https://github.com/user-attachments/assets/beb0a92b-2818-4027-bdc4-3264f7616888" />

<img width="1751" height="1010" alt="Screenshot From 2026-10-11 19-40-13" src="https://github.com/user-attachments/assets/3f0c3c17-3811-4053-b3b6-9323c72c7f20" />



<img width="482" height="497" alt="inspector" src="https://github.com/user-attachments/assets/e21358e1-9723-4a2f-9985-696dc5302f9f" /> <img width="482" height="497" alt="picrawler3" src="https://github.com/user-attachments/assets/8b453783-1193-4d3c-8e5e-d1e746b3135a" />

## A note on the name

You will see `ami_ogma` and `ogma` in the code — the C++ namespace `ami_ogma::`,
the Godot addon `addons/ami_ogma/` and `ami_ogma_host.so`, and `OGMA_*` env vars.
That is xaq's **original internal codename**, kept in those identifiers so
existing builds and saved configs stay stable. **`ami_ogma` and xaq are the
same project.** New Python code lives in the `xaq` / `xaq_core` packages.
See [docs/NAMING.md](docs/NAMING.md).

## Start here

- **[docs/brain_building_doctrine.md](docs/brain_building_doctrine.md)** — the
  method: how to compose predictive loops into an agent without fooling
  yourself. A living document, and the thing every result below is tested under.
- **[docs/glossary.md](docs/glossary.md)** — plain-language definitions of the
  whole vocabulary, if active inference is new to you.
- **The Microduck** — [the standing report](docs/reports/microduck_standing_report.md),
  then the [design proposed to Pollen](https://github.com/pollen-robotics/microduck/pull/381).
- **The PiCrawler** — [the gait findings](docs/reports/picrawler_gait_loop_findings.md),
  then [the lever ledger](docs/reports/picrawler_lever_ledger.md).
- **[docs/reports/cell_markov_blanket_loops_report.md](docs/reports/cell_markov_blanket_loops_report.md)** —
  *The Cell Navigator: A Falsification Testbed for Markov-Blanket-Loop Active
  Inference.* The report whose contribution is the discipline itself, including
  the two headline claims it overturns.
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

> First time in this repo? [AGENTS.md](AGENTS.md#build--test-quick) has the full
> first-time setup — including system prerequisites the snippets below assume
> are already installed — [CLAUDE.md](CLAUDE.md#4-build--run-picrawler) covers
> actually *running* the Godot host (the `.so` below only gets built, not launched),
> and [mj_host/README.md](mj_host/README.md) covers the Microduck host.

**C++ runtime** (needs CMake ≥ 3.14, a C++17 compiler, and ZeroMQ's **dev**
package — e.g. `libzmq3-dev` on Debian/Ubuntu; the ZeroMQ runtime library alone
is not enough. Eigen / nlohmann-json / GoogleTest are fetched automatically):

```sh
cmake -S cpp_core -B cpp_core/build
cmake --build cpp_core/build -j
```

**Godot host** (rebuilds the GDExtension `.so`, which is intentionally not
committed — it is regenerated from source on every clone; building it does
**not** require a Godot binary, only *running* it does):

```sh
cmake -S godot_host -B godot_host/build
cmake --build godot_host/build -j     # produces + installs project/addons/ami_ogma/*.so
```

**Microduck host** (fetches MuJoCo and ONNX Runtime, each pinned by version and
checksum; Pollen's model files are vendored in `mj_host/models/microduck/`, and
their trained policies are downloaded on first run):

```sh
./mj_host/run.sh build
./mj_host/run.sh gates                # every gate; non-zero exit on any failure
./mj_host/run.sh watch --secs 30      # a live window
```

**PiCrawler hardware host** (builds natively on the Raspberry Pi 5; see
[pi_host/README.md](pi_host/README.md) before running anything that moves a servo):

```sh
cmake -S pi_host -B pi_host/build && cmake --build pi_host/build -j2
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

## Sponsors

Bongiovi Media & Technology
https://bongiovimt.com
