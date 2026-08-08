# MotorEPMv2 — a proactive motor layer, built as a SEPARATE module

**Status:** plan, 2026-08-06. **Decision (operator):** *"Our current MotorEPM is functional
and is our current benchmark, so I'd rather build MotorEPMv2 so A/B is simpler and less
error prone."*

Read first: [`motor_layer_is_reactive.md`](motor_layer_is_reactive.md) — the evidence that
the motor layer predicts in order to *learn* and never in order to *act*.

---

## 0. Why a new module rather than more parameters

`MotorEPM.cpp` is ~4 300 lines carrying **~60 gain-0-guarded levers**, most of them refuted,
several load-bearing. This session alone added five parameters to it and produced two
confident regressions. The costs of continuing that way are concrete:

- **Guard interference.** Every new lever must be byte-identical at 0 *against every
  combination of the other 60*. That is untested and untestable in practice.
- **A/B ambiguity.** `phase_vel_smooth` changed the meaning of `commit_prec_gain`'s units
  mid-session; the arm that produced `+5.7 %` ceased to exist and could not be re-measured.
- **The benchmark moves.** The deployed gait is the control for everything. Editing it to
  test an alternative to it is a conflict of interest.

**As a separate module the A/B is a one-line config swap**, the benchmark is frozen by
construction, and a v2 defect cannot regress the deployed body.

## 1. The invariant that makes this trustworthy

> **v2 with every new feature at 0 must be BYTE-IDENTICAL to MotorEPM.**

Not "close" — identical. This is the whole basis of the comparison, and it is *testable*:
run both modules on the same seed and diff the motor command stream tick-by-tick.

**Ship the differ first** (`scripts_tools/moduledif.py`): dump per-tick `y[]` from each and
assert exact equality over ≥3 000 ticks on ≥3 seeds. Until that passes, no v2 result means
anything. Any divergence is a v2 bug, never a finding.

Practically this means v2 **starts as a copy** of MotorEPM, not a rewrite. The proactive
machinery is added on top of a verified-identical base. Refactoring and behaviour change in
the same commit is how a substrate becomes unfalsifiable.

## 2. What v2 inherits unchanged

Everything currently load-bearing, per doctrine §4 (*never disable a working loop; feed the
FULL lower loop*): the HK core (`A/b` model, `C/h` controller, TLE), motor babble warmup,
DEP, the stroke and its `stroke_signs`, Kuramoto coupling, the amplitude homeostat, the
heading PD with `goal_bearing_topic`, panic, commit, `stroke12` amplitude, and
**`intent_yaw_gain = 0`** — the ground-only intent error, which is the session's best arm on
net displacement (5.11 m vs 4.92 m).

## 3. Rung 1 — act on the predicted state

The forward model `x̂ = A·y + b` is already learned online. Today it is consumed only to
form the residual. v2 feeds it to the controller:

```
x_eff = (1 − λ)·x + λ·x̂        →        y = tanh(C·x_eff + h + Cφ·φ + Cv·φ)
```

`λ = 0` byte-identical. **This is the operator's own observation** — *"lag compensation
sounds like prediction"* — generalized: every loop has delay (sensing, filtering,
actuation), and a controller reading `x_t` acts on stale state. One mechanism covers all of
it, including the phase-filter failure that motivated it.

⚠ **The naive form is circular.** `x̂_{t+1} = A·y_t + b` needs `y_t`, which is what we are
computing. Two honest resolutions, both cheap, and **v2 must implement (b)**:

- **(a)** use `y_{t−1}` — assumes action continuity; one line; a weaker lookahead.
- **(b)** one fixed-point iteration: compute `y⁰` from `x`, form `x̂ = A·y⁰ + b`, recompute
  `y¹` from `x̂`. A true one-step lookahead for one extra matmul per leg.

Ship (b) with (a) available as `lookahead_mode` — (a) is the natural **control arm**: if the
cheap version matches the real one, the effect is not lookahead.

**Controls:** `λ = 0` (identity), **`λ < 0` (wrong-sign — must regress)**, and
`A := 0` (predict-no-change — isolates "acting on a *prediction*" from "acting on a
*different vector*"). The resonance lever died to a missing wrong-sign control; do not repeat
that.

**Gate:** net_disp and `straight` on n≥6, then replicate on unseen seeds. Promote-or-kill.

## 4. Rung 2 — a real EPM at the motor layer

Only if rung 1 shows the proactive direction is real. Today `MotorEPM` has **no GNG at all**
— no node vocabulary, no edges, no `transition_surprise`, and a *synthetic* GNG payload
built purely for the inspector widget (`MotorEPM.cpp:4564`, `edges` always empty). Rung 2
makes the name true, per CLAUDE.md §0:

- frozen encoder over the per-leg sensorimotor vector (identity or RBF — modality-shaped);
- GNG with baking / mitosis / pruning, so the motor vocabulary is **earned**;
- the **dual** TLE — `α·quant_error + β·transition_surprise` — which is the piece that gives
  "which node comes next", the structure the operator assumed already existed;
- ⚠ **§0 rule 2 is the live risk here**: condition the input or the insertion gate collapses
  the stride manifold to one node. Check the PCA scatter against node count *before* reading
  any behavioural number.

Then `couple_prec_gain` and `commit_prec_gain` finally have a **real EPM TLE** to weight
by — today there is none at this layer, which is why the whole precision family read flat.

## 5. Rung 3 — expected-free-energy action selection

Roll the loop forward N steps over candidate actions; pick by predicted error. Deferred:
a linear one-step model rolled far forward will not support it. Needs rung 2's discrete
states first.

## 6. Staging and gates

| stage | deliverable | gate |
|---|---|---|
| 0 | `moduledif.py`; v2 = verified copy | **byte-identical, 3 seeds × 3 000 ticks** |
| 1 | rung 1 + the three controls | net_disp/straight n≥6, then unseen-seed replication |
| 2 | UI observation | operator: proactive, or merely different? |
| 3 | rung 2 (real GNG) | node count vs PCA scatter *first*; then behaviour |

## 7. Measurement rules for this plan

1. **`net_disp` and `straight` are the headline. Path length is never reported as progress.**
2. Before running an arm, name the degenerate behavior that would score well on its metric.
3. Sub-10 % deltas on any arm perturbing the trajectory are **unmeasured**, not null.
4. Every lever ships with a **wrong-sign control**; a lever whose sign does not matter is
   not a mechanism.
5. Verify the consumer fired *and* that the diagnostic reports the same quantity the
   mechanism uses.
