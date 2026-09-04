# CLAUDE.md — how to build in this repo

**This is the operating layer: *how* to work here, right now.** The *why* lives in
[`docs/brain_building_doctrine.md`](docs/brain_building_doctrine.md) — the method, its
principles, and the evidence for each. Read this file first, then the doctrine.

This project builds **embodied active-inference agents**: brains that act to minimize
their own prediction error, with no external reward. That is far outside the usual
distribution for coding assistants, and the default instinct — *"want behavior X? write
code that produces X"* — is **wrong here and produces work that gets thrown away**. The
rewrite rule below is the correction. Internalize it before touching a control law.

---

## 0. The EPM — the substrate's predictive coarse-grainer

**Keep this in working context at all times.** The EPM (**Episodic Predictive Module**) is
the foundational component of this architecture and remains the default answer **wherever
predictive coarse-graining is required** — that is, wherever a continuous, high-dimensional,
noisy stream must become a discrete, addressable, *predictive* vocabulary that carries its
own error. That operation is what this brain runs on, at every level.

### What it is

Three parts, one contract (`docs/plans-and-designs/primitives/EPM.md`):

1. **A frozen encoder** — modality-shaped, not learned (JL projection for visual, Hopf
   filterbank for cochlear, RBF grid for proprioceptive, identity for stacking).
2. **A GNG topology** — Growing Neural Gas: grows nodes where the input is novel, **bakes**
   them once revisited enough, **splits** them (mitosis) where error persists after baking,
   and prunes the stale. The vocabulary is *earned*, never declared.
3. **A dual TLE** — `tle = α·quant_error + β·transition_surprise` (0.7 / 0.3 by default).
   Two distinct questions in one scalar: *am I in territory I know?* and *did I predict
   where I would go next?*

It publishes one `RealityToken` per tick — `winner_id`, `latent`, `tle`, `is_novel` — on
`reality.<group>.<modality>`.

### Why it dominates

- **It topologizes SURPRISE, not raw observation.** With a descending predictor present, the
  prediction is *subtracted before the GNG sees the input*
  (`gng_input = encode(obs) − predicted_latent`) — predictive coding in the substrate, not a
  metaphor for it. Its TLE is then the universal learning signal (doctrine §1): if a module
  needs a "how well do I know this / how surprising is this" scalar, **that is TLE**.
- **It self-sizes, and it stacks for free.** Baking / mitosis / pruning make resolution
  follow the data (spawn where error is persistent, irreducible and localized — doctrine §5).
  A Level-N EPM is *the same code* with `input_topic` on `consensus.0` and an identity
  encoder, so hierarchy is configuration: the cognitive **map** is a slow EPM over consensus.
- **It is precision-weightable.** `1/(tle+ε)` is what the LateralVoter fuses on — trust is
  earned from predictive accuracy, never assigned by a designer.

### The rules

1. **EPM first — never hand-roll a clusterer.** No bespoke VQ, k-means, binner, discretizer,
   or confidence scalar. A raw L2-VQ was measured against it on the same signal and lost.
2. **Condition the input, or discretization will hide your signal.** *This is where EPM use
   actually goes wrong* (doctrine §6): a small directional signal on a large common-mode gets
   collapsed to one node by the insertion gate **while the encoder and PCA still show the
   structure clearly**. Centre out the common-mode, normalize for scale. If node count says
   "one thing" and the PCA scatter says "several," believe the scatter.
3. **Feed it phase.** `Clock/CPG → EPM → LateralVoter` is the shared temporal context and is
   **load-bearing** — a legged controller *only worked with it; the ablation broke it*.
4. **Its diagnostics are first-class instruments** — `nodes`, `baked`, `tle`, `is_novel`,
   mitosis. Never baking, never growing, or growing unbounded is usually a conditioning or
   gating diagnosis (rule 2), not a verdict on the idea.
5. **Don't average raw features into it and call it learned.** EMA-ing a raw feature on
   reward events learns the background (doctrine §4).

**The through-line:** when something must become discrete, comparable, and predictive, **the
EPM is the answer until proven otherwise — and "otherwise" requires a measurement.**

---

## 1. The rewrite rule — the one thing to get right

> **Never implement a behavior. Implement the error the behavior minimizes.**

Given a desired behavior X:

1. **Ask what prediction error X would reduce.** If X isn't the descent direction of some
   error, you are about to write a script, not a brain.
2. **Check the sensory channel FIRST.** Does any egocentric observation actually carry the
   signal that error needs? *If not, the fix is a new **sensor**, not a smarter policy.* A
   hidden state the agent gets no observation of cannot be reached by a better policy — only
   stumbled onto. (Doctrine §2.1. Confirmed twice, on two different creatures.)
3. **Hand the owning module that observation + the objective.** Not a trajectory, not a
   joint bias, not a schedule. A *drive*, *reflex*, or *objective* that motion emerges from.
4. **Gate the bias by the state it exploits.** The gate is the design; the magnitude is
   just tuning. An ungated DC bias fights the loop it rides on.
5. **Ship it gain-0-guarded, then A/B it** (§3).

**Worked examples from this repo's own history — all three started as the wrong instinct:**

| Goal | The script instinct (wrong) | What actually worked |
|---|---|---|
| Hold a heading | steering script / waypoint follower | **PD on dead-reckoned own-yaw** (integrated egocentric yaw, Markov-compliant) through the *authoritative* skid-steer channel |
| Climb the hump | a climb policy, leg-timing (Cruse) | **A new sensory channel** — belly ToF rangefinder — plus fixing a homeostat windup that fought locomotion. It was a *missing observation*, not a missing policy |
| Get the belly off the ground | scripted lift / DC knee bias | **Stance-gated knee tuck** — bias applied only to planted legs, swing legs untouched. A blind DC knee bias kills the gait; **the gating is the whole trick** |

**The through-line, which explains nearly every result here:
LEARNED cooperates, IMPOSED fights.** Every mechanism that let the body *discover* won;
every mechanism that *dictated to* the body lost the same way — chaos, collision, or
stiffness. When choosing between a contextual/learned form and a commanded one, reach for
the learned one.

---

## 2. Read order

| When | Read |
|---|---|
| Always, first | this file — **especially §0 (the EPM), which stays in working context** |
| Building/extending any perceptual or coarse-graining path | [`docs/plans-and-designs/primitives/EPM.md`](docs/plans-and-designs/primitives/EPM.md) — the full EPM contract (params, invariants, failure modes, VV&A) |
| Before any design decision | [`docs/brain_building_doctrine.md`](docs/brain_building_doctrine.md) — the method ("the bible") |
| Before proposing a picrawler lever | [`docs/reports/picrawler_lever_ledger.md`](docs/reports/picrawler_lever_ledger.md) — **what is already refuted** |
| Before trusting ANY picrawler sensor | [`docs/plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md`](docs/plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md) — **per-topic legality audit + a LIVE god's-eye dependency in the deployed gait.** `feet_y` is absolute world-Y |
| Working on the picrawler | [`plan`](docs/plans-and-designs/picrawler_active_inference_plan.md) + [`gait findings`](docs/reports/picrawler_gait_loop_findings.md) |
| Working on the **microduck** | [`microduck port plan`](docs/plans-and-designs/microduck_port_plan.md) — **start at "▶ Resume here"**. The MuJoCo host (`mj_host`), the duck's sensorimotor surface, and **two tracks**: framework work at the joints, and a community contribution at the intent boundary that augments Pollen's RL stack rather than competing with it. Branch `microduck`, **simulation only**; `./mj_host/run.sh gates` is the health check |
| **Wiring a brain from scratch**, or checking what a config actually subscribes to | [`brain_builder/README.md`](brain_builder/README.md) — the Dear ImGui **brain builder**: the whole registry on a palette, the body's sources and sinks as nodes, drag-to-wire, validate, dry-run, publish. `./brain_builder/run.sh open <config>`; `brain_builder --validate <config>` from a script. Design in [`docs/plans-and-designs/brain_builder_plan.md`](docs/plans-and-designs/brain_builder_plan.md) |
| Working on the Cell | [`cell report`](docs/reports/cell_markov_blanket_loops_report.md) |
| Building a level ABOVE the fast loops (hierarchy, a slow EPM, fusing several EPMs) | [`slow-loop notes`](docs/plans-and-designs/slow_loop_design_notes.md) + [`fusion notes`](docs/plans-and-designs/fusion_notes.md) — **design-stage, nothing measured yet.** Both mark which parts are already shipped machinery (`consensus.0` stacking, `KeyframeAverager`, `GNGRollout`, a level ≥ 1 LateralVoter) and which are proposals |
| Working on **xaq_voice** (sonification of TLE) | [`tools/xaq_voice/README.md`](tools/xaq_voice/README.md) — an **instrument, not a behaviour**; its only contract with the brain is the `lite` diag topic. Tune it with [the studio](tools/xaq_voice_studio/README.md); a new signal to sonify is a one-line, O(1) addition to a module's `diag_lite()` |
| **Writing anything that LEAVES this repo** — a formal report (`docs/reports/`), or a PR / issue / commit in someone else's repository | [`REPORTS.md`](REPORTS.md) — **audience, structure, and the banned "Claudese".** §9 covers outward-facing work: **our jargon does not travel, one thing per PR, and opening one is always the operator's call**. Read it BEFORE the first line |
| Repo layout, naming, licence | [`AGENTS.md`](AGENTS.md) — **`ami_ogma` == `ogma` == xaq**, intentionally |
| The vocabulary is new to you | [`docs/glossary.md`](docs/glossary.md) — plain-language, concepts-first |
| Body geometry / servo model | [`docs/operational/`](docs/operational/README.md) — CAD-derived ground truth + protocols |
| "Was this tried in the RL era?" | [`docs/reports/archive/`](docs/reports/archive/README.md) — pre-doctrine record. **Verdicts there do NOT transfer**; its failure patterns and measurement lessons do |

**Do not propose a lever without checking the ledger.** A large fraction of plausible ideas
here have already been built and falsified; re-proposing one is the most common way to
waste a session.

---

## 3. The A/B protocol — non-negotiable

Every behavioral change is a **lever**, and every lever is evaluated the same way:

1. **One lever at a time.** Never two.
2. **Gain-0-guarded.** With its gain at 0 the build must be **byte-identical** to before.
   New modules default OFF so existing envs don't move.
3. **Seed-averaged.** Single-seed results are **noise**, full stop. Use the harness (§4).
4. **Judged on the FULL metric set — never one number.** Ask: *what degenerate behavior would
   also score well here?* If one exists you have a **blind metric** — add its complement
   first. Repeat offenders: `turns` is blind to a body that swings and nets ~0 (use
   `straight` = net_disp/path_len); chassis height is blind to belly-drag (measure belly
   clearance); `fwd_v` mean is oscillation-dominated (read it with net_disp — high fwd_v +
   low net_disp = fast circling). And no gait is good if the chassis collides.
5. **Observe in the UI before promoting.** Aggregate metrics hide what watching catches.
6. **Nothing is ever dead — only refuted in the context it was tried** (§3.1).
7. **Scale the claim to the power.** n=4–6 fixed-seed is a **signal** — enough to
   promote-or-kill a direction, not a finding, and especially not a defensible null. A
   finding needs n≥20, varied world seeds, and the (d) perturbation test.

### 3.1 Verdicts: nothing is dead

**A refutation is a statement about a mechanism IN A CONTEXT — never about the mechanism.**
Record the context as part of the verdict. Three ways a negative result is about something
other than the idea: the **scenario** was wrong for it, the **baseline** was degenerate, or
what was built was a **weakened slice** of the real mechanism. Each has burned this project
at scale — see the ledger §6–7 for the cases.

**Use the explicit verdict vocabulary** (`BASELINE` / `WORKING` / `PARTIAL` / `NULL` /
`REGRESSION` / `TAUTOLOGY` / `DEAD_CODE` / `ABLATED` / `DEFERRED` / `IN_FLIGHT` — defined in
the ledger's header). Note that several are **not verdicts on the idea at all**:
`TAUTOLOGY`, `DEAD_CODE`, and a `NULL`-against-a-broken-baseline are measurement outcomes.

**Every refuted lever carries a re-use context** — what would justify retrying it. "Refuted,
revisit if X" is a complete verdict; "dead" is not one.

### 3.2 Before you record a negative verdict — or trust one

1. **Tautology.** Read the running params — is the "new" knob already at that value?
2. **Dead code.** Is the affected path actually live in this config?
3. **Guards.** Do execution guards admit the sign/range you are testing?
4. **Baseline validity.** Is the control healthy, or itself at a degenerate attractor?
5. **Consumer.** Did the consumer actually fire? Verify with telemetry.
6. **Faithfulness.** Did you build the mechanism, or a weakened slice of it?
7. **Silent confound.** Did the arm you *think* you ran actually load?

**Failing any of these means you measured your harness, not your idea.** Every one of these
has produced a false verdict here; the ledger §7 has the cases.

### 3.3 How loud should a real result be?

Two rules look opposed and are not. **Seed-average everything** (§3 rule 3), because a single
seed is noise. But also: **a real capability is LOUD** — the operator's anchor is that when
the substrate genuinely works, standing emerges *in minutes, seed-robustly*. A mechanism
needing n=10 over long runs to show a sub-σ delta is not a capability, it is a slightly
different stumbling pattern.

**The reconciliation: seed-averaging is a filter against false positives, not a microscope
for finding small ones.** Use it to confirm a *loud* effect is real, not to excavate a
marginal one. If a lever only appears after heavy averaging, prefer killing it and finding a
bigger idea over powering it further.

**What counts as loud** is adaptation, not displacement — distance metrics can reward dead
drift. The three the operator reads as real: **heading regulation** (re-corrects when noise
turns the body), **proto-gait steps**, and **obstacle-triggered adaptation** (error spikes on
contact, the body feels around and sometimes traverses — proving plasticity is still live
late in a run).

---

## 4. Build & run (picrawler)

**First run on a new machine:** `godot4` below is not a package your OS provides —
it must be a **Godot 4.6.2** binary on `PATH` (exact version; `godot_host/`'s
`extension_api.json` / `gdextension_interface.h` were dumped from it). Download
`Godot_v4.6.2-stable_linux.x86_64` and symlink it: `ln -s
/path/to/Godot_v4.6.2-stable_linux.x86_64 ~/.local/bin/godot4`. Full first-time
system/Python prerequisites (ZeroMQ dev headers, the Python venv, `pytest`) are in
[`AGENTS.md`](AGENTS.md#build--test-quick).

```sh
# Build (MotorEPM-only edit ≈ 30 s; the .so auto-copies into project/addons/ami_ogma/)
cmake --build godot_host/build --target ami_ogma_host -j8

# Seed-averaged A/B — USE THIS FOR EVERY COMPARISON
python3 godot_host/project/scripts_tools/seedavg.py <config.json> [n=6] [steps=12000] [diff=0.3]
#   prints net_z / max_z / net_disp / straight / fwd_v / turns / falls  (mean ± std)
#   logs go to /tmp/xaq_seedavg, or set SEEDAVG_OUT=<dir>

# Single headless run
OGMA_PICRAWLER_GYM=corridor \
OGMA_PICRAWLER_CONFIG=res://addons/ami_ogma/configs/<config.json> \
OGMA_RESET_MODE=continuous OGMA_PICRAWLER_MAX_STEPS=<N> OGMA_PICRAWLER_GYM_DIFFICULTY=0.3 \
godot4 --headless --fixed-fps 60 --quit-after 4000000 \
       --path . res://scenes/the_picrawler.tscn      # run from godot_host/project
```

- ~52 ticks/s. Run concurrent seeds on distinct `OGMA_INSPECTOR_PORT`s.
- Body diagnostics are JSON-per-line on stdout (`fwd_v`, `gc_raw`/`gc_norm`/`cy_norm`,
  `h_ema`/`h_max`/`h_bias`, chassis `y`, `knee[]`, `feet_y[]`, `auto_reset_count`).
- **UI:** `[P]` toggles the per-metre path trail; `[1]`/`[2]` live-swap arena/corridor; the
  MOTOR-EPM panel has live sliders for every lever. The live viewer is
  [`tools/xaq_inspector`](tools/xaq_inspector/README.md). **Operator/UI-driven diagnosis is
  first-class** — not a fallback.
- Seed tooling was fixed 2026-07-23 (`OGMA_SEED` now actually varies the MotorEPM RNG).
  **Pre-2026-07-23 single-seed results are noise — re-measure before trusting them.**

C++/Python build and test: see [`AGENTS.md`](AGENTS.md). Commits are DCO-signed (`git commit -s`).

---

## 5. Hard prohibitions

1. **No reward shaping.** Ever. Intrinsic homeostatic valence only. This is the framework's
   whole premise, and violating it is what turned the first picrawler attempt into a
   brittle RL stack (see [`docs/the-picrawler-detour.md`](docs/the-picrawler-detour.md)).
2. **No oracle.** Perception may be a transparent sensor reduction (a bootstrap, named as a
   scaffold). **Selection and belief — where to go, where things are — must be learned.**
   Ground-truth bearing → proportional steer is cybernetics, not inference.
3. **Markov-blanket discipline: egocentric signals only.** A god's-eye quantity is not an
   observation. (The god's-eye `chassis_y_norm` was retired for the belly rangefinder —
   god's-eye height is blind to belly grounding and cannot solve terrain.)
4. **Never disable a working loop.** A closed sensorimotor loop is foundational substrate;
   build the next layer *on* it. A lesion is a **test**, never the operating mode. Feed the
   FULL lower loop, never a stripped one.
5. **Don't tune a constant to a signal's scale — adapt it** from the system's own running
   dynamics (running variance, spread-normalized softmax, count-annealed exploration).
6. **Don't distill a percept from a teacher.** A copy dies with its source. A learned
   pathway with no prediction graded by its own honest error is an anti-pattern wearing a
   new name.
7. **Don't inject a rhythm or impose a coordination topology** — give the brain a
   *prediction to fulfill* and let the coordination emerge.
8. **Don't hand-roll a clusterer, quantizer, or bespoke confidence scalar** — use the EPM and
   its TLE (§0).

---

## 6. Working vocabulary

| Term | Meaning |
|---|---|
| **EPM** | Episodic Predictive Module — encoder + GNG + dual TLE; the predictive coarse-grainer everything is built on (§0) |
| **TLE** | Time-Loop Error — `α·quant_error + β·transition_surprise`; the universal learning and confidence signal |
| **RealityToken** | An EPM's per-tick output: `winner_id`, `latent`, `tle`, `is_novel` |
| **baking / mitosis / prune** | The GNG earning a node (revisited enough), splitting one whose error persists after baking, and dropping the stale |
| **coarse-graining** | Turning a continuous high-dimensional stream into a discrete, addressable, predictive vocabulary — the EPM's job |
| **consensus** | The LateralVoter's precision-weighted (`1/(tle+ε)`) fusion of many EPMs into one shared belief |
| **lever** | One isolated, gain-guarded behavioral change — the unit of work and of A/B |
| **gain-0 guard** | The lever's gain at 0 leaves the build byte-identical; how we ship safely |
| **seed-avg** | Running N seeds and reporting mean ± std; the minimum bar for any comparison |
| **blind metric** | A metric a degenerate behavior can satisfy (`turns` vs swinging; chassis height vs belly-drag) |
| **refuted (in context)** | Falsified *in the scenario, at the power, and against the baseline stated*. Never a global verdict — nothing here is "dead" (§3.1) |
| **re-use context** | The conditions that would justify retrying a refuted lever — part of a complete verdict |
| **promote-or-kill** | The fast-fail gate at the end of a staged iteration |
| **the stack** | The current deployed composite of promoted levers on a base config |
| **belly-up** | Locomoting with the belly held clear of the ground (belly-drag would grind a real chassis) |
| **scaffold / de-scaffold** | A temporary teacher/oracle prop, named as such — and its removal, showing the loop still stands |
| **strange loop** | A closed sensorimotor loop that becomes foundational substrate for the next layer |
| **the (a)–(d) bar** | The defensibility checklist: inferred-not-oracle · action-reduces-own-error · loop-isolation controls · perturbation→re-inference |
| **(d) test** | Perturb mid-episode (relocate the goal, drop a sensor) and show re-inference + recovery — the sharpest single evidence |
| **signal vs finding** | n=4–6 fixed-seed = *signal* (promote-or-kill only); n≥20 varied seeds + (d) = *finding* |

---

## 7. Reporting

**For a FORMAL report — anything destined for `docs/reports/` and an outside reader —
read [`REPORTS.md`](REPORTS.md) first.** It sets the audience (intelligent, outside
active inference and machine learning), the structure (executive summary written as an
abstract for that audience, then monotonically narrowing so a reader can stop at any
depth), the tone (positive and inviting, including where a hypothesis was falsified), and
the list of machine-writing tics to avoid. Two rules from it are worth repeating here
because they are violated by default: **never pre-qualify your own honesty** ("the honest
take", "to be fair") — it implies the surrounding text is less honest; and **process
mistakes belong in the ledger, not the report** — a falsified hypothesis is a result and
stays, but a wrong turn during the investigation goes to the ledger.

**REPORTS.md §9 extends the same standard to anything sent OUTSIDE this project** — a pull
request, issue, or commit message in someone else's repository. Three of its rules bind
hardest: **our vocabulary does not travel** (no EPM, TLE, or Markov blanket in someone else's
PR, and a change that needs our framework to look worthwhile is the wrong change to send);
**one thing per PR**; and **opening one is always the operator's call** — prepare it locally,
run their gates, and show them.

Scale claims to evidence. If it ties the baseline, **say "ties."** Name scaffolds as
scaffolds. A graceful-degradation result is not a "beats both" result. **A falsified
hypothesis is not waste — it is the gradient**; always diagnose *why* a prediction failed
before the next attempt, and record it in the ledger.

When a lever is proven or falsified, fold the reusable principle back into the doctrine and
the specific verdict into the ledger.
