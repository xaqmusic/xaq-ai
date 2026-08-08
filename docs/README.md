# xaq documentation

> **Building here? Read [`../CLAUDE.md`](../CLAUDE.md) first** — the operating layer (the
> rewrite rule, the A/B protocol, the build recipe, the vocabulary). The docs below are the
> *why*; CLAUDE.md is the *how, right now*.

- **[reports/picrawler_lever_ledger.md](reports/picrawler_lever_ledger.md)** — **the
  promoted-vs-refuted ledger for the live picrawler work. Read it before proposing a
  lever**; most plausible ideas have already been built and falsified, and each verdict is
  recorded with the scenario it was decided in.
- **[reports/picrawler_gait_loop_findings.md](reports/picrawler_gait_loop_findings.md)** —
  the active-inference gait: the emergent, self-rescuing gait and what was learned building
  it. Companion to
  [plans-and-designs/picrawler_active_inference_plan.md](plans-and-designs/picrawler_active_inference_plan.md).
- **[reports/cell_markov_blanket_loops_report.md](reports/cell_markov_blanket_loops_report.md)**
  — *The Cell Navigator: A Falsification Testbed for Markov-Blanket-Loop Active
  Inference.* The flagship evidence report: what xaq's architecture is, and a
  discipline for testing embodied active-inference agents without fooling yourself.
- **[brain_building_doctrine.md](brain_building_doctrine.md)** — the method the
  report tests: reusable, creature-agnostic principles for composing predictive
  loops into a competent agent.
- **[research-summaries/](research-summaries/)** — distillations of the external
  papers xaq draws on (active inference, JEPA / fractal-JEPA, hyperdimensional
  consensus, morphodynamics, homeokinesis / the playful machine, SIGReg). These
  summarize outside work and note where xaq uses, defers, or diverges from it.
- **[the-picrawler-detour.md](the-picrawler-detour.md)** — the honest origin story:
  the framework was first pushed into a complex quadruped body too early, ended up a
  fragile RL mess, and that failure is why the Cell was rebuilt with rigour. Explains
  the `the_picrawler_*` configs still in the repo.
- **[glossary.md](glossary.md)** — plain-language glossary of the whole conceptual
  stack (active inference, free energy, predictive coding, JEPA, HDC, homeokinesis…).
  The best starting point if the vocabulary is new.
- **[operational/](operational/README.md)** — protocols and physical ground truth: the
  CAD-derived [picrawler geometry](operational/picrawler_geometry.md), the
  [servo model](servo_dynamics.md), the contamination discipline, and the still-open
  [aliveness-vs-distance metric question](operational/aliveness_metric_protocol.md).
- **[reports/archive/](reports/archive/README.md)** — the **reward-shaped RL era** of the
  picrawler, kept as an honest record. Not baselines and not method: its individual
  mechanism verdicts do not transfer, but its failure patterns and measurement lessons do
  (baseline validity, silent confounds, Goodhart in the wild). The
  [mechanism registry](reports/archive/mechanism_registry_rl_era.md) is the ancestor of the
  current lever ledger.
- **[NAMING.md](NAMING.md)** — why the code says `ami_ogma`/`ogma` (xaq's
  original internal codename).
