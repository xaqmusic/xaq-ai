# Embodied Scaffolding Principles

**Status:** north-star doctrine. **Date:** 2026-06-17. **Author:** Joseph Butera III.

This is the single authority document for *what this project is building, what claims it is
allowed to make, and what counts as a scaffold versus a result.* Every paper, result doc, and
canonical config should link here. The adversarial audit
([`docs/2026_06_17_adversarial_audit_report.md`](../2026_06_17_adversarial_audit_report.md)),
the substrate critique
([`docs/2026_06_17_substrate_idea_critique_and_next_steps.md`](../2026_06_17_substrate_idea_critique_and_next_steps.md)),
and the mechanism registry act as **enforcement layers** against this doctrine. This document
was created because the project's principles were real but scattered across the Playful Machine
notes, the v4 position paper, the proprioception roadmap, the MotorEPM plan, and the aliveness
protocol — and that scatter let a contradiction ("reward-free" + an internal agency reward)
live in the canonical line.

---

## 1. The big-picture goal

Build **embodied, continuously-learning intelligence from real-time sensorimotor loops** — a
System-1 substrate that learns by closing prediction/action loops in a body, *not* by offline
reward optimization over a fixed dataset. The north star is **aliveness** (closed-loop,
adaptive, perturbation-robust, Playful-Machine / active-inference behavior), not a scalar like
max-distance. See [`feedback: aliveness-over-distance`]. Distance, visits, and dense reward are
diagnostics, never the objective.

The honest contribution claim is **methodological and integrative**: this is a serious embodied
*falsification substrate* that records which active-inference-shaped mechanisms survive contact
with a real action loop, which fail, and what scaffold accounting is required before a claim is
defensible. It is **not** a new active-inference theory, and should never be framed as one.

---

## 2. What "active inference" means *here*

A module inventory is not active inference. An active-inference **claim** requires all of:

1. a **perception variable**,
2. a **prediction / error variable**,
3. an **action path** that can change that variable,
4. an **open-loop or severed-loop control**, and
5. an **ablation showing the loop — not just locomotion — caused the effect.**

The MotorEPM **A1** result (the perceptual/cognitive EPM stack can be bit-identical inert while
locomotion continues) is exactly why this bar matters: presence of EPMs/voters/predictors does
**not** license a cognitive-substrate claim for the canonical controller.

---

## 3. Claim vocabulary (use these terms; do not blur them)

Every experiment and every reward-like signal must be filed under exactly one of these. Do not
collapse them under one "reward-free" heading.

| Term | Definition | Example in this project |
|---|---|---|
| **external reward** | environment-injected scalar fed to a learning rule | `events.hit`, dopamine, distance/progress bonuses |
| **internal objective / agency-fitness search** | an intrinsic optimizer over the agent's *own* state | `coord_reward_drive` ((1+1) search on forward proprioceptive thrust) |
| **homeokinetic drive** | TLE / self-model error descent; no objective scalar | MotorEPM HK core |
| **inherited scaffold** | fixed feedback law / reflex / prior; no reward term | postural tone, Kuramoto coupling, stroke drive, homeostats |
| **active target-bearing loop** | direct perception→steering closure | `target_compass`→`nav_gain` steering (currently *oracle* bearing) |
| **prediction-error minimization** | action chosen to reduce inferred error | (claim-gated per §2) |
| **pure observer / instrumentation** | reads state, changes nothing | telemetry, summaries |

**"Reward-free" is retired as a system-level claim.** It is permitted only as shorthand for
*external-reward-free*, and only when the surrounding text defines it that way. A config or
controller that runs *any* internal objective search is **not** "reward-free."

---

## 4. Allowed biological priors

These are legitimate and need no apology — they are the substrate, not cheating:

- proprioception, efference copy, reflex arcs
- limb topology and morphology-aware control
- signed inter-leg coupling (Cruse/Walknet-style), CPG/phase coordinates
- vestibular balance, postural tone, homeostats (height, amplitude)
- target-bearing steering, panic/escape subsumption
- slow/fast loop separation (spinal vs. cognitive timescales)

---

## 5. Allowed scaffolds — and the rule that makes them honest

Oracle signals, hand-coded gates, tuned reflexes, CPG/bootstrap phases, and task-specific
diagnostics are **allowed**, but only when labeled. Every load-bearing scaffold must carry:

1. a **name**,
2. a **config flag**,
3. an **ablation status** (does removing it collapse the claimed behavior?), and
4. an **intended retirement path** — or an explicit statement that it is a permanent embodied prior.

If an ablation shows `-stroke`, `-coupling`, or `-agency` collapses forward velocity, those are
**load-bearing design elements**, not incidental biological color, and the result is
**scaffolded**, not **emergent**.

---

## 6. Disallowed hacks

- external reward shaping disguised as intrinsic drive
- hard-coded task solutions presented as emergence
- hidden privileged/oracle state inside a claim-grade config without disclosure
- metrics that invite Goodhart (raw distance, raw visits, gait-cycle pulses, progress-PB)
- promoting n=1 / low-n UI breakthroughs before a powered gate (n≥20) catches the regression
- "consumer present" treated as "consumer matters" (a published token is not evidence it is used)
- keeping stale docs alive after later evidence supersedes them

---

## 7. Claim discipline

- **"Scaffolded embodied control"** is a valid, publishable claim when stated plainly.
- **"Pure reward-free active inference"** is valid *only* when the exact config and ablations
  support it — which, for the current canonical MotorEPM line, they do **not**.
- Internal objective search (e.g. `coord_reward_drive`) is **not exempt** from Goodhart risk
  just because it is intrinsic. It is an optimizer and is owed the same controls (shuffled /
  lagged / wrong-sign / orbit-vs-arrive). Forward-thrust maximization is *constraint-robust* on
  a flat-ground legged body — but report its Goodhart failure modes as first-class results.
- A mechanism is **mechanism-validated** (publishes correct tokens, passes unit tests) until a
  closed causal loop and ablation make it **behavior-validated**. Do not conflate the two.

---

## 8. Consolidation / plasticity policy

Reward-supervised modules need explicit consolidation; self-supervised EPMs may self-stabilize.
"Freeze everything" is not a solution (it produces circling / stale behavior). For
MotorEPM/hybrids, state which components are plastic at which timescale: motor self-model, phase
offsets, amplitude, bearing, slow cognitive policy. (See frozen-brain findings.)

---

## 9. Enforcement

- **Config/doc linter** — ✅ IMPLEMENTED: `scripts/audit_claim_metadata.py` (test:
  `tests/test_claim_metadata_linter.py`). Fails on "reward-free" with `coord_reward_drive > 0`,
  a `reward_class` inconsistent with the active objective, a "vision-grounded" claim still riding
  the oracle `target_compass`, canonical tier configs missing tier/reward_class metadata, and the
  retired headline phrase in the claim docs. (Graph-edge and chunk-consumer checks: future scope.)
- **Mechanism registry** → promote to a per-claim ledger: claim id, exact config, exact commit,
  result path, seed count, required controls, caveats, status (proposed / supported / falsified /
  superseded / claim-grade), architecture family (Cell / Premotor-per-servo / MotorEPM / hybrid).
- **Claim-run mode** (audit F5) — ✅ IMPLEMENTED: `scripts/picrawler_run.py --claim-mode` lints
  the config pre-launch, then **fails** the run on reward dominance, missing/failed seeds,
  stripped trajectories, or `/tmp` artifacts. Publication artifacts land under
  `results/publication_motor_epm/<tier>/<timestamp>/` with config SHA + git commit in the manifest.

---

## 10. Canonical MotorEPM config tiers

✅ IMPLEMENTED — generated by `scripts/make_motor_epm_tiers.py` (single source of truth =
the validated `the_picrawler_motor_epm_minimal.json`). Each config carries self-describing
`metadata.tier` / `reward_class` / `scaffolds_active`, enforced by the §9 linter. A nested ladder:

- `motor_epm_pure_hk` — HK core only; no scaffolds, no objective, no nav (`reward_class: homeokinetic`)
- `motor_epm_spinal_hybrid` — + postural/coupling/stroke/balance/homeostats/panic (`homeokinetic`)
- `motor_epm_agency` — + `coord_reward_drive` internal agency-fitness search (`internal_objective`)
- `motor_epm_nav_oracle` — + `nav_gain` steering on the oracle `target_compass` (`internal_objective`)
- `motor_epm_vision_nav` — *future*: bearing estimated from vision/perception (not yet built)

`the_picrawler_motor_epm_minimal.json` IS the `nav_oracle` tier (stamped in place, validated params
preserved).

---

## 11. Best public framing

> This project is an embodied active-inference and homeokinetic-control research substrate. Its
> main contribution is showing which active-inference-shaped mechanisms survive contact with an
> embodied action loop, which fail, and what scaffold accounting is required before claims become
> defensible. The current canonical line is a **scaffolded embodied motor controller**: per-leg
> homeokinetic self-modeling + load-bearing biological priors + an optional internal
> agency-fitness search + direct (oracle) target-bearing steering. It proves motor-level loop
> closure and scaffolded navigation — not pure reward-free cognitive active inference.
