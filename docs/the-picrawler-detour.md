# The picrawler detour — why the Cell is the starting point

Before the Cell, we tried to run this framework in a hard body. It went badly, and
that failure is the reason the rest of this project looks the way it does.

## What we tried

The **picrawler** is a twelve-servo quadruped. We pushed xaq's active-inference
substrate straight into it, expecting the same reward-free, homeostatic machinery we
believed in to learn to stand and walk. It was the wrong order of operations.

A complex body is unforgiving. To get *anything* to move, scaffolding crept in:
hand-tuned gait priors, coupling terms, and — the cardinal sin for a framework built
on intrinsic motivation — reward shaping. Each patch bought a little motion and a lot
of fragility. What we ended up with was not the transparent, reward-free agent the
framework promised; it was a brittle reinforcement-learning stack propped up by
scaffolds, whose headline claims ("reward-free locomotion") ran ahead of what the
evidence could actually support. When we audited our own results honestly, several of
them did not survive.

## The lesson

That is the most useful thing the picrawler gave us:

> A hard body hides the difference between a real capability and a propped-up one.
> Without the discipline to try to falsify your own claims, you cannot tell which you
> have — and complexity hands you endless places to fool yourself.

## What we did instead

We did the opposite of "add more body." We stripped the embodiment down to the
simplest thing that can still forage, sense, and have needs — **the Cell**, a
single-celled swimmer in a 2-D arena — and rebuilt from there under one rule: every
claim gets pre-registered, powered, and adversarially checked before we believe it.

That discipline, not any single architecture, is the actual contribution. It is
written down in [`brain_building_doctrine.md`](brain_building_doctrine.md) and
stress-tested in [the Cell report](reports/cell_markov_blanket_loops_report.md) —
which, true to the rule, overturns two of its own headline results once they are
properly powered.

## What this means for the repo

The picrawler still lives here. The Godot host carries its body, its scenes, and a
large set of `the_picrawler_*` configs (plus an `archive/` of older runs and a
`BASELINES.md` that predates the current package layout). We keep them **on purpose**:
as an honest record of the detour, and as a harder problem to return to *once the
method has earned it*.

They are **not recommended baselines and not the way to learn this project.** Some of
them encode exactly the scaffolded, reward-shaped setups the discipline now warns
against — that is what they are for. If you are new here, **start with the Cell.** The
picrawler is where this framework went before it learned to be careful; the Cell is
where it learned.

The simulation and environments built for the picrawler are solid. We will be revisiting
it shortly in preparation for moving the xaq framework onto the Raspberry Pi.
