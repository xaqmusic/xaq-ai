# HeadingController — Primitive Contract

**Lineage:** added 2026-06-21 (commits `f31f812`, then learned-advance `57fa3df`) as the
operator's decomposition of the monolithic coxswain (`ActionDecoder` in joint-action mode).
**Header:** `cpp_core/include/ogma/modules/HeadingController.hpp`
**Impl:** `cpp_core/src/ogma/modules/HeadingController.cpp`
**Inspector:** `tools/v4_inspector/widgets/heading_controller_inspector.py` (heading dial +
learned-advance policy grid + `k_body`/thrust/velocity traces).

---

## Purpose

The HeadingController is the **action half** of the Cell's `nav → action` decomposition:

- **nav** ("create a heading toward food") produces a desired egocentric heading;
- **action** (this module) "**acts on a heading**" — turns the body to face it and advances
  along it.

It replaces the monolithic coxswain, which learned a tangled `(bearing-state) → (turn × thrust
that climbs a scent value)` map and acquired a *perverse* thrust policy (it backed away when
facing food, because the `cy` value rewarded **facing**, not **reaching**). Splitting the two
concerns gives each a clean, independently-learnable goal: the action layer's goal — "achieve
the commanded heading" — is **food-independent and dense**, so it learns from any heading
commands (no cold-start deadlock) and, once reliable, gives the nav layer a clean credit signal.

Everything it learns comes from the body's **own sensorimotor signals** — no oracle, no
hand-set P-gain. It advances **bar-(b) at the motor level** (a forward model of the body's own
dynamics, inverted to null prediction error).

---

## The learning mechanism

The HeadingController learns **two** things.

### 1. Turn forward-model `k_body` (always on)

The body publishes its yaw rate ω on `reality.proprio.ang_vel` — a *clean* rotation signal
(only steering rotates the body; translating does not, unlike the egocentric bearing).

- **Learn:** each tick, pair the steer commanded **last** tick with the ω observed **this**
  tick: `k = |ω| / |steer|` — yaw produced per unit steer = this body's turn responsiveness.
  Fold into an EMA: `k_body += gain_lr · (k − k_body)`. Updated only when `|prev_steer| > 0.1`
  (signal-to-noise). `k_body` converges to a body-specific value rather than railing.
- **Invert to act:** to null a fraction `turn_fraction` of the heading error per tick,
  `steer = clamp(turn_fraction · bearing / k_body, ±max_steer)`. A sluggish body (low `k_body`)
  automatically gets *more* steer; a twitchy one gets *less*. The P-gain is set by the body's
  measured dynamics → it transfers across bodies / friction / fields (the no-tuning principle).
  Before `k_body` is learned, fall back to `gain_init`.

`bearing = atan2(cx, cy) / π ∈ [−1, 1]` (0 = facing the desired heading, ±1 = directly behind),
read from the desired-heading token `[cx = +right, cy = +forward]`.

### 2. Advance policy `V[err_bin][thrust]` (opt-in: `learn_advance`)

When `learn_advance = false` (default — keeps every legacy config byte-identical), thrust is a
**hand-designed cos braking gate**: `thrust = max_thrust · clamp((cos(bearing·π) − cos(align_angle))
/ (1 − cos(align_angle)), −1, 1)` — full forward within `align_angle_deg` of facing, brake/reverse
beyond. This is a scaffold (a hand-set alignment threshold).

When `learn_advance = true`, the thrust policy is **learned** so that brake-turn-charge *emerges*:

- **State** = heading-error bin: `err_bin = min(n_err_bins−1, ⌊|bearing| · n_err_bins⌋)`
  (bin 0 = facing, 0–45° for `n_err_bins=4`; higher bins = more off-axis; last = behind).
- **Action** = a thrust level `a ∈ {0 … n_thrust_acts−1}` mapped to
  `thrust = −max_thrust + 2·max_thrust · a/(n_thrust_acts−1)` (3 levels = reverse / stop / forward).
- **Value** `V[err_bin][a]` = EMA of the **reward**: `V += advance_lr · (reward − V)`.
- **Reward** (credited one tick later, to the thrust that produced the motion):
  `reward = max(0, vel_fwd) · cos(bearing·π) − effort_cost · |vel_fwd|`
  — **forward** progress *projected onto the commanded heading*, minus an energy cost. Built only
  from the body's own egocentric velocity (`reality.proprio.vel_ego = [v_right, v_fwd]`) and its
  own commanded bearing → **food-independent**. The `max(0, ·)` encodes the front-mouth
  morphology (the bug eats going forward, so reversing toward a behind-target must not be
  rewarded); the `effort_cost` term breaks the off-axis tie toward **stop** (vs. reverse-away).
- **Selection** = UCB: `score[a] = V[err_bin][a] + ucb_c · √(ln(N_bin + 1) / (n[err_bin][a] + 1))`
  → pick the highest-value action plus an exploration bonus for under-tried actions that
  **self-anneals** as visits accumulate (the adaptive mechanism — no fixed ε / temperature to tune).

**What emerges:** at bin 0 (facing), forward thrust → `vel_fwd > 0`, `cos ≈ 1` → positive reward
→ **forward wins**. Off-axis (`cos ≤ 0`), forward → ≤ 0, reverse → `−effort`, stop → ~0 → **stop
wins** (let the `k_body` steer rotate the body to face, *then* charge). So the admired
"brake-turn-then-charge" is **learned from the body's dynamics**, not a hand-set gate.

**The two together:** `k_body` decides *how hard to turn* to face the heading; the advance policy
decides *whether to drive forward* given how aligned the body is.

---

## Input Topics

| Topic (default) | Kind | Payload | Required | Notes |
|---|---|---|---|---|
| `input_topic` = `percept.scent_compass` | Direct | `ProprioToken` `[cx=+right, cy=+forward, …]` | yes | The **desired** egocentric heading. In the de-scaffold stack this is rewired to `percept.bearing_inferred` (Stage 3) or `percept.motivated_heading` (Stage 2). |
| `ang_vel_topic` = `reality.proprio.ang_vel` | Direct | `ProprioToken` (scalar ω, rad/tick) | yes | Clean steer-driven yaw rate; the signal `k_body` is learned from. |
| `vel_topic` = `reality.proprio.vel_ego` | Direct | `ProprioToken` `[v_right, v_fwd]` (move_speed-norm) | only if `learn_advance` | Egocentric afferent velocity (actual displacement, frame-correct via the body's basis) — the food-independent basis of the advance reward. |

## Output Topics

| Topic (default) | Payload | Cadence |
|---|---|---|
| `steer_output_topic` = `cog.steer` | `ActionOut` (turn) | every tick |
| `thrust_output_topic` = `cog.thrust` | `ActionOut` (advance) | every tick |

Both feed the `MotorBus` `steer_thrust` channel → `action.left/right`. The body
(`bidirectional_paddler`) decodes turn (differential) and forward/reverse (common-mode)
independently, so **in-place rotation is physically available** (the brake-turn-charge depends
on this).

---

## Parameter Schema

| Key | Mutability | Default | Description |
|---|---|---|---|
| `input_topic` | ConstructionOnly | `percept.scent_compass` | Desired-heading source `[cx, cy]`. |
| `cx_index` / `cy_index` | ConstructionOnly | 0 / 1 | Component indices in the input token. |
| `steer_output_topic` / `thrust_output_topic` | ConstructionOnly | `cog.steer` / `cog.thrust` | ActionOut outputs. |
| `ang_vel_topic` | ConstructionOnly | `reality.proprio.ang_vel` | Yaw-rate source for `k_body`. |
| `gain_init` | ConstructionOnly | 0.5 | Fallback turn gain before `k_body` is learned (not behavioral tuning). |
| `gain_lr` | HotMutable | 0.05 | EMA rate for `k_body`. |
| `turn_fraction` | HotMutable | 0.6 (cell configs use 2.5) | Fraction of heading error to null per tick (control stability). |
| `gain_min` / `gain_max` | HotMutable | 0.05 / 4.0 | Safety rails on the effective turn gain. |
| `max_steer` / `max_thrust` | HotMutable | 4.0 / 4.0 | Output ranges (match the MotorBus accel range). |
| `min_signal` | HotMutable | 0.1 | `|heading| <` this → no confident heading → `nav_on = false` (no steer/advance). |
| `align_angle_deg` | HotMutable | 30.0 | Hand-gate alignment cone (used **only** when `learn_advance = false`). |
| `learn_advance` | ConstructionOnly | **false** | true → learn the thrust policy (below); false → hand cos gate (byte-identical legacy). |
| `vel_topic` | ConstructionOnly | `reality.proprio.vel_ego` | Egocentric velocity for the advance reward. |
| `n_err_bins` | ConstructionOnly | 4 | Heading-error bins (advance value-table rows). |
| `n_thrust_acts` | ConstructionOnly | 3 | Thrust levels spanning `[−max_thrust, +max_thrust]` (cols; 3 = reverse/stop/forward). |
| `advance_lr` | HotMutable | 0.1 | EMA rate for the advance value table. |
| `ucb_c` | HotMutable | 0.4 | UCB exploration weight (self-annealing — not a tuned ε). |
| `effort_cost` | HotMutable | 0.15 | Energy prior on `|vel_fwd|`; makes the off-axis brake resolve to **stop**, not reverse-away. |

---

## Diagnostics (`diag_snapshot` → v4_inspector / body JSONL)

`bearing`, `gain` (effective turn gain), `k_body`, `steer` (`hc_steer`), `thrust` (`hc_thrust`),
`nav_on`. When `learn_advance`: `err_bin`, `thrust_act`, `adv_reward`, `adv_spread`
(max−min of the current row; rising = learning), `adv_cov` (fraction of cells visited),
`n_err_bins`, `n_thrust_acts`, `max_thrust`, `vel_fwd`, and the full `adv_value` / `adv_visits`
tables (the inspector's policy grid). The inspector marks the argmax action per row (`▶` = the
learned policy) and rings the live `(err_bin, thrust_act)` cell — watch the **forward** column
light up at bin 0 and **stop** at the higher bins as `adv_spread` rises from ~0.

---

## Invariants

1. Publishes exactly one `cog.steer` and one `cog.thrust` per tick.
2. `nav_on` ⇔ `|[cx, cy]| > min_signal`; when off, both outputs are 0 (idle) and no advance
   credit is taken (no learning across a nav gap).
3. `learn_advance = false` is **byte-identical** to the pre-learned module (the cos gate) and to
   non-cell hosts (CartPole / MountainCar / picrawler never set it).
4. The advance reward credits the thrust chosen **last** tick with the velocity observed **this**
   tick (one-tick deferral); the steer (turn) is independent of the advance value table.
5. `k_body` and the advance tables persist via `snapshot_state` / `restore_state`.

## Failure modes

| Trigger | Behaviour |
|---|---|
| No `ang_vel` yet / `|steer|` always tiny | `k_body` stays 0 → effective gain = `gain_init` (still steers, just un-calibrated). |
| `learn_advance` with `vel_topic` never delivered | `vel_fwd = 0` → reward ≈ 0 everywhere → policy stays flat (no charge). Check the body publishes `reality.proprio.vel_ego`. |
| Desired-heading token shorter than `cy_index+1` | Missing component read as 0. |

---

## Status / findings (open arena, turn rig)

- **Hand gate (`learn_advance=false`):** 296 eats / 10 min (5 seeds), genuine turn-and-approach.
- **Learned advance:** **TRACKS** any commanded heading in isolation (`HeadingProbe`, no food,
  n=3: steady-state heading error 0.04–0.11, incl. large turns) — so "act on a heading" is
  genuinely achieved. In-arena it forages and the brake-turn-charge **emerges** (best seed clean
  `b0→forward, b1→stop`), but is **seed-dependent** (mean ~96 eats; 2/3 seeds *orbit* — charge
  while still ~30–45° off → arc → never settle dead-on). The orbit is an advance-policy
  consistency issue (a per-tick myopic reward can't learn "align *fully* before charging"); it is
  **not** a tracking failure (isolation proves alignment works) and motivation (Stage 2) makes it
  non-critical (forage *enough* to stay sated → efficiency stops mattering).
- **Scaffold status:** the turn forward-model (`k_body`) is genuinely learned cognition at the
  motor level; the learned advance removes the hand-set alignment gate; the desired *heading
  selection* (where to go) lives upstream (nav) and de-scaffolds in the maze.

See `docs/findings/cell_heading_follower_milestone.md` and
`docs/plans-and-designs/cell_descaffolding_plan.md`.
