# The motor layer predicts in order to LEARN, never in order to ACT

**Status:** analysis, 2026-08-06. Prompted by the operator: *"Are we running solid forward
predictors for each leg? ... Our robot should be acting into the future in order for the
present to be correct ... it has a reactive aspect, which is why its obstacle traversal is
so good, but it does not have a proactive component."*

Verified against `cpp_core/src/ogma/modules/MotorEPM.cpp`. The instinct is correct, and two
of the premises behind it are not — which changes what should be built.

## 1. There is no GNG at the motor layer

`MotorEPM` contains no Growing Neural Gas: no node vocabulary, no edges, no `winner_id`, no
`is_novel`, and **no transition surprise** — the second half of the dual TLE (CLAUDE.md §0).
The only `gng` in the file is a *synthetic payload built for the inspector* (`:4564-4587`):

```cpp
// GNG-shaped payload so the EPM's PCA-scatter widget renders the self-model
gng["nodes"] = {ONE node} ;   // = the current 1-step prediction x̂
gng["edges"] = array();       // literally always empty
```

So the UI renders an EPM-shaped widget for a mechanism that is not an EPM. **There is no
node history and nothing "pointing toward the next group of nodes"** at this layer. Any
reasoning that assumes a transition graph here is reasoning about a structure that does not
exist. (This is a measurement-hygiene problem in its own right: the visualization implies a
vocabulary the substrate lacks.)

## 2. There IS a forward model — one per leg, and it is real

```cpp
x̂  = L.A * L.prev_y + L.b;     // forward model:  motor → next sensor
ξ  = L.x - x̂;                  // the motor TLE
L.A += η_M * ξ * prev_yᵀ;      // it learns online
```

Linear, one-step, per-leg, trained continuously. It genuinely predicts the body.

## 3. But the prediction never reaches the action

`x_hat` occurs on **four lines in the whole file, all inside the learning block.** It
predicts the *current* sensor from the *previous* command — it is retrospective. Its only
consumers are the model update and `tle_ema`.

The live command (`:3107`) is:

```cpp
z = L.C * L.x + L.h  (+ Cphi·φ + Cvel·φ)      y = tanh(z)
```

— a function of the **current** sensor. So the loop is:

> **predict → grade the past → update weights → act on the present**

That is a reactive controller with a predictive *learning rule*. It explains both operator
observations exactly: reaction is fast and well-tuned (hence the strong obstacle traversal),
and nothing anticipates.

## 4. This subsumes the phase-lag problem

The 2026-08-05/06 phase-filter failures (`phase_vel_smooth`, `phase_sym_smooth`: net
displacement −57% and −39%) happened because `L.phase` *times the power stroke*, so any
filter delay makes the stroke fire late and push backward through part of the stride.
"Compensate the filter's group delay" was a patch for one instance of a general defect:
**every loop has delay — sensing, filtering, actuation — and a controller that reads `x_t`
always acts on stale state.** Acting on `x̂` compensates all of it with a quantity already
computed every tick.

## 5. The ladder

**Rung 1 — act on the predicted state.** `x_eff = (1-λ)·x + λ·x̂`, feeding the existing
controller. λ=0 byte-identical. Uses only machinery that exists.

⚠ **The naive form is circular** and this is the part to get right: `x̂_{t+1} = A·y_t + b`
needs `y_t`, which is what we are computing. Two honest resolutions — (a) use `y_{t-1}`
(assumes action continuity, one line), or (b) one fixed-point iteration: compute `y` from
`x`, form `x̂` from that `y`, recompute `y`. (b) is a true one-step lookahead and costs one
extra matrix multiply per leg.

**Rung 2 — a real EPM at the motor layer.** A GNG over sensorimotor state, giving an actual
node vocabulary and transition graph, so "which node comes next" becomes answerable and
`transition_surprise` exists here at all. This is the structure the operator assumed was
already present. Substantial build; §0 says it is the right substrate.

**Rung 3 — expected-free-energy action selection.** Roll the loop forward N steps over
candidate actions. Only meaningful once rung 2 supplies discrete states; a linear one-step
model rolled far forward will not support it.

**Recommendation: rung 1 first** — not because it is the best idea, but because it is the
cheapest *test of the premise*. If acting one step ahead changes nothing, rung 2's cost is
not yet justified. If it moves net displacement, the proactive direction is real and rung 2
becomes the obvious investment.

**Judge on `net_disp` and `straight`, never path length** (see the 2026-08-06 retraction).
