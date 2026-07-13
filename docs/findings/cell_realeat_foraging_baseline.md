# Cell foraging baseline — real-eat calibration + desperation exploration

*Session 2026-07-01/02, branch `cell-maze`. The state this documents is a **baseline for
improvement**, not a finished design; the next direction is the explicit-EFE / precision
refactor in [`../plans-and-designs/efe_arbiter_precision_refactor_plan.md`](../plans-and-designs/efe_arbiter_precision_refactor_plan.md).*

## The discovery that ties it together: `events.hit` is overloaded

In the Cell, `events.hit` is **not** a clean consummatory signal. `body_controller.gd`
publishes it on **two** occasions with the same intensity (1.0):

1. the genuine nutrient collision (`on_nutrient_hit`), and
2. a **scent-progress inference** — `short_ema(scent) > long_ema(scent) × 1.5` while moving
   forward — which fires ~205 times per 500 ticks all through an approach.

Every module that learned from `events.hit` was therefore learning mostly from an
inference, not from eating. Two modules did:

- **klino (`RunTumbleNav`)** — its self-reported confidence denominator.
- **planner (`PlaceGraphPlanner`)** — its food memory: `food[node] += food_reward` on each
  event → **`Fmax` reached 147 from a single real eat**, inflating phantom caches the
  planner then obsessively circled.

**Fix (both modules): a distinct ground-truth event `events.eat`**, published *only* by
`on_nutrient_hit`. klino and the planner subscribe `events.eat` (param `eat_topic`,
default `events.eat`) instead of the polluted `events.hit`. Planner `Fmax` 147 → 1.0.

## Klino owns the close — eat-calibrated confidence

**Problem (operator, live UI):** near the food the planner re-won and turned the bug away
before the eat. Root cause: klino's proximity level `clamp(hunger·scent)` is structurally
capped ≈0.73 (the scent field is source-normalised but the bug samples a sub-1 value at its
closest approach), while the planner self-normalises to ≈1 — so the two oscillated.

**Mechanism:** klino learns `eat_scent_ = EMA(scent at its own real eats)` (captured *at the
eat moment* in `handle_eat`, because the food moves the instant it eats) and self-reports
`cap = clamp(scent / eat_scent)` ∈ [0,1] — ≈1 the instant it is in its own eating range. The
arbiter consumes `cap` as a **MAX'd proximity-level boost** on `v_klino`
(`max(z_spike, clamp(hunger·scent), clamp(hunger·cap))`) — a boost that can only raise
`v_klino` near food, never silence a far/blind klino (the reverted multiplicative gate that
killed the forager). The interim absolute planner-cede `×(1−hunger·scent)` is kept as a
jitter margin.

**Result (seed 42, 18k ticks, A/B vs cap-boost ablated):** klino owns the close (scent > 0.6)
**100 % (32/32) with the boost vs 43.7 % (31/71) without** — where the planner otherwise
re-wins the majority of near-food ticks. Honest nuance: at these deterministic eats the eat
*instant* is won by the approach z-spike in both arms; the calibration's demonstrated win is
the *sustained* near-food ownership, and the settled-close flip is proven by the unit test.

## Planner breaks the "circle one area until it starves" cycle — desperation disconfirmation

**Problem (operator):** after a few eats (dense map + food caches) a bored-and-hungry bug
oscillates in one area until it starves. Even with clean food memory (Fix above), the caches
always give the value field an uphill gradient → `route_exists` is always true → the planner
never enters its fast run-and-tumble wander branch, and it routes/circles to starvation.

**Mechanism (param `disconfirm`, 0 = off; `0.1` in the arbiter config):** a remembered cache
the bug **dwells on but does not eat at** has its food belief decayed by
`food[cur] *= (1 − clamp(disconfirm · hunger · habituation))` — faster the longer it camps
and the hungrier it is. This is belief updating from prediction error (active inference), not
a hunger threshold. Empty caches fade → the value field shifts → the hungry bug systematically
re-checks and spreads across the maze; a cache where it *does* eat is re-reinforced on the
hit, so productive caches persist.

**Result (seed 42, deep-hunger phase E < 0.5, A/B vs `disconfirm=0`):**

| signal | baseline | disconfirm=0.1 |
|---|---|---|
| explored bounding-box area | 131 | **380** (~3×) |
| coverage (2 m cells) | 34 | **48** (+41 %) |
| position spread (σx+σy) | 7.1 | **11.7** (+65 %) |

The spatial confinement is broken — a hungry bug searches the whole map.

**A dead end worth recording:** adding a hunger-scaled *explore gradient* to the value field
(`+ desperation · explore_gain`) **backfired** — it keeps `route_exists` true, so the bug
crawls along the staleness gradient (slow graph-routing) instead of wandering, and coverage
went *down*. Fading the caches so the field **flattens** is what triggers the fast wander.
Also falsified: a TLE-novelty "boredom" gate (`1 − local_tle/self-cal-peak`) stays ~0 because
the self-calibrated peak decays to track the current TLE, so `desperation = hunger` directly.

## Honest limitations / next leads

- **Survival is not improved.** The wider search happens via slow graph-*routing* (crawl),
  not the fast wander, so the bug explores ~3× wider but too slowly to reach food before
  starving. **The exploit-route crawl locomotion is now the gating blocker** — make routing
  move at run-speed and the wider search should convert into more eats.
- **n = 1 deterministic env** (seeds 42/43/44 give identical macro behavior). Rank by the
  structural signals above (the ~3× bbox and 100 %-vs-44 % close ownership are large effects),
  not brittle hit-counts.
- **The arbiter is still a value race**, patched with the eat-cal boost + the absolute cede.
  The doctrine now calls this out (§2.2/§2.3) and the precision-refactor plan re-expresses the
  same signals as an explicit-EFE precision problem — the intended next step.

## HUD (demo)

The Cell HUD was cleaned for demos: a unified top-left meter panel (motor → energy → arbiter),
all one font size, with the arbiter shown as a klino-vs-planner tug-of-war and `desperation`
exposed in the planner inspector.

## Telemetry gotcha (cost real time twice)

`OgmaBrain::get_module_metrics` exposes each module via **specific accessors**, not
`diag_snapshot()`. New fields (`eat_scent`, `desperation`) read as 0 in the JSONL until an
accessor line is added there — even though the live module state is correct. Arbiter fields
already use live accessors, so arbiter analysis was always valid.
