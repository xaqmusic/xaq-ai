# Archive — the reward-shaped RL era of the picrawler

> **These are not baselines, not method, and not the way to learn this project.** They document
> the first picrawler attempt: the framework pushed into a complex body too early, propped up
> with hand-tuned gait priors, coupling terms, and — the cardinal sin for a framework built on
> intrinsic motivation — **reward shaping**. That story, and why it is kept, is
> [`../../the-picrawler-detour.md`](../../the-picrawler-detour.md).

Ported from the pre-split `ami-ogma` repo on 2026-07-25 (the split was incidental, not a
curation decision). Naming: `ami_ogma` / `ogma` / `AMI-Ogma` == **xaq**.

| Doc | What it is |
|---|---|
| [`mechanism_registry_rl_era.md`](mechanism_registry_rl_era.md) | **The most valuable one.** The canonical ledger of every mechanism shipped in that era, its outcome at the most rigorous power measured, and the mechanistic reason. The direct ancestor of [`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md) |
| [`picrawler_stand_diagnostic_rl_era.md`](picrawler_stand_diagnostic_rl_era.md) | Stage A: substrate validated, brain-driven standing demonstrated at n=20 |
| [`picrawler_brain_config_matrix_rl_era.md`](picrawler_brain_config_matrix_rl_era.md) | Stage B: brain-config A/B matrix, n=20 paired seeds |
| [`picrawler_exploration_knobs_rl_era.md`](picrawler_exploration_knobs_rl_era.md) | Stage C: exploration-knob A/Bs (epistemic gain, eligibility λ) |

## What transfers and what doesn't

**Does NOT transfer — the individual mechanism verdicts.** Different substrate (REINFORCE
Premotors vs. the homeokinetic Motor-EPM), different objective (shaped reward vs. intrinsic
prediction error), different baseline. A `NULL` recorded there says nothing about the same idea
on the current stack. Current verdicts live in
[`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md).

**DOES transfer — the failure shapes and the measurement lessons.** These were paid for at
enormous cost and several have already recurred on the reward-free stack:

- **The named failure patterns (A–G)** — constrains-without-unlocking, stability-bought-with-
  exploration, dead code, tautology, activity-without-aliveness, open-loop-primitive-without-
  differential-outcome, and the withdrawn Pattern G. Distilled into
  [`../picrawler_lever_ledger.md`](../picrawler_lever_ledger.md) §7.
- **Baseline validity** — the 2026-05-31 audit found **84 % of analyzable historical runs had a
  single channel taking >60 % of the signal**. The controls themselves were degenerate, which
  reopened an entire era of "null" verdicts. This is the deep reason **nothing in this project is
  ever "dead"** — only refuted in the context, at the power, and against the baseline it was tried
  with ([`../../../CLAUDE.md`](../../../CLAUDE.md) §3.1).
- **Silent confounds** — a harness flag that was never set meant a whole family of A/Bs quietly
  ran the wrong stage for weeks. Now check 7 in [`../../../CLAUDE.md`](../../../CLAUDE.md) §3.2.
- **Faithfulness** — a structural-prior "null" once rested on an implementation carrying **1 of
  the theory's 6 rules**. That falsifies the slice, not the theory.
- **Goodhart, concretely** — a reward channel left ungated accumulated 86 % of all reward across
  a 3-hour run while the body lay on its belly twitching, undetected. Every degenerate solution in
  this archive was *found by the agent*, not designed.

## The one-line summary

This archive is the empirical case for the discipline in
[`../../brain_building_doctrine.md`](../../brain_building_doctrine.md). It is what the method
costs when you don't have it.
