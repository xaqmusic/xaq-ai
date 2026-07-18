# Cell Navigator — Strange-Loops Plan

*The nav architecture for the Cell, from the operator's 2026-06-25 schematic. Built to
`docs/brain_building_doctrine.md` — especially §5 (layered/additive, strange loops),
§2 (the EFE arbiter chooses a pathway into the future), §8 (predictable env for
measurable epistemic foraging). Agile: the wiring is tinkerable — per-loop voters mean a
bad feeder EPM only poisons its own loop, so we re-wire as we go rather than hold to spec.*

## The idea (Friston's strange loops)
- **When a loop is working, do not disable it.** Build the next layer ON it. Every early
  layer is foundational substrate.
- **The brain chooses a pathway into the future** — the EFE arbiter carries that decision.
- **Epistemic foraging = reducing free energy.** It is messy; the *environment* is the
  fixed point so progress is measurable.

## Architecture

```
INPUTS         per-input EPM      per-LOOP voter → heading generator         ARBITER            ACTION (foundational, always ON)

IMU    ─► EPM ─┐
SCENT  ─► EPM ─┤   ┌─ PLAY      : (sensorimotor)──►voter──► homeokinesis ───┐
VISION ─► EPM ─┼─► ┤  SCENT-GRAD: scent ─────────►voter──► gradient heading ┤─► EFE ARBITER ─► HEADING+ADVANCE ─► MOTOR ─► L
COMPASS─► EPM ─┤   └─ PLACE-PLAN: window-avg ─────►voter──► place-graph route┘   IMPORTANT?       CONTROLLER       MIX BUS ─► R
CPG    ─► EPM ─┘                                                                 HEADING? GO!                       ▲
                                                                                                      WHISKER/STUCK REFLEX ─┘
   ▲                                                                                                          │
   └──────────────── DESCENDING PREDICTOR : consensus → predict each EPM's next latent → EPM subtracts (pred-error) ───┘
                                                  (the strange loop: top-down prediction, action makes it come true)
```

**A loop = {a group of inputs → its own voter → a heading-generator}.** All loops are
heading generators feeding the EFE arbiter. Per-loop voters isolate faults (drop a bad
EPM from one loop's voter without touching the others).

## The loops (each stands alone, with a straightforward metric — §3)

### PLAY — pure homeokinesis (epistemic; the floor)
- **Always alive**, every tick, off the bug's raw sensorimotor interaction. Reward-free.
  The arbiter *exposes* it (lets it drive) when the other loops lack confidence; it is
  never gated off. When scent is gone its output looks like a random walk — but it *falls
  out of homeokinesis*, it is not a hardcoded "wander."
- **Predicts:** the next sensorimotor state (reafference); acts to keep the loop
  predictable-yet-exploring (edge of chaos).
- **Honest signal:** the homeokinetic objective (self-motion prediction error) — no reward.
- **Gate / metric:** coverage of the env + the homeokinetic objective holding; reward-free.
- **Status:** NEW. Build on the existing `HomeokineticExploration` / `MotorEPM` substrate
  (not the coin-flip tumble). The scalar run-and-tumble's *exploration* half graduates here.

### SCENT-GRADIENT — honest scent-climb (pragmatic, local)
- **Inputs:** SCALAR scent concentration (no 8-nostril homing ring — §4 morphology
  honesty) + egomotion heading.
- **Heading-generator:** run-when-rising (the run-and-tumble's *scent* half) → a heading.
- **Predicts:** Δscent of the chosen heading (action-consequence forward model, §1).
- **Honest signal:** the bug's own Δscent / the eat (§4).
- **Gate / metric:** beats a TRUE RANDOM WALK; eat-rate tracks scent.
- **Status:** PARTIAL — `RunTumbleNav` is the scalar mechanism; split its scent half here.

### PLACE-PLANNER — map route to remembered food (pragmatic, global)
- **Inputs:** window-averaged (keyframe) consensus = the slow map stream.
- **Heading-generator:** `PlaceGraphPlanner` — place-graph + food-memory → route → heading.
- **Predicts:** the route's value (value-iteration over the place graph).
- **Honest signal:** food-memory (events.hit) + learned transitions.
- **Gate / metric:** routes to remembered food (the D-value rescue).
- **Status:** EXISTS (Pathway D). Reuse.

## EFE ARBITER (chooses a pathway into the future — §2)
- Inputs: each loop's **(heading, confidence)** + the homeostatic state (hunger).
- **importance(loop) = confidence(loop) × urgency(hunger)**, weighing **epistemic** (PLAY:
  "I'd resolve uncertainty") against **pragmatic** (SCENT/PLACE: "I'd reach food"). Pick
  the max → emit its heading. *IMPORTANT? → HEADING? → GO!*
- PLAY is the **floor**: when SCENT and PLACE have no confidence and hunger is rising, the
  homeokinetic loop wins — exploration *emerges*, it isn't selected by a hardcoded rule.
- **Status:** NEW. Generalize B.3's scent-progress × hunger to rank N loops.

## ACTION LOOP (foundational, always ON)
- `HeadingController` (learned turn-gain **+ advance**) → `MotorBus` → MOTOR L/R. Whisker /
  stuck-escape reflexes subsume directly at the bus. A higher layer feeds the **full** loop
  — never a stripped one (§5).
- **Status:** EXISTS. Turn the learned advance back ON in the nav configs.

## DESCENDING PREDICTOR (the feedback — closes the strange loop)
- `DescendingPredictor`: `predicted_latent = W·consensus + b` per target; the EPM
  subscribes via Feedback and subtracts the prediction before its GNG step → its residual
  IS the prediction error; SGD on the error. Top-down prediction meets bottom-up
  sensation; **action makes the prediction come true** — the active-inference loop.
- **Status:** EXISTS. Wire it in once the loops + arbiter are up.

## Measurement environment (§8)
- **2 hard-coded food spawn locations, one each side of the L-bend wall** — one direct, one
  occluded. Fixed (never random respawn) so the bug can come to KNOW its world.
- **Success signature = eat-rate RISING over time** as the bug learns/predicts the env —
  not a high instantaneous rate, and explicitly NOT "beats the reflex."
- Floor control: a true random walk (PLAY with learning off / shuffle).

## Build order (layered — every lower loop stays ON)
- **L0 (exists, ON):** action loop (HeadingController + advance) + MotorBus + reflexes.
- **L1 — each loop ALONE, gated** (any order; start where the operator points):
  - SCENT-GRADIENT → beats random walk.
  - PLACE-PLANNER → routes to remembered food (D).
  - PLAY → coverage + homeokinetic objective.
- **L2 — EFE ARBITER:** rank the loops; expose PLAY as the floor.
- **L3 — DESCENDING PREDICTOR:** close the top-down loop.
- Each step keeps everything below it running. Lesions are *tests* (does the new layer
  stand alone?), never the operating mode.

## What exists vs. new
- Exist/reuse: EPM, LateralVoter (per-loop), KeyframeAverager (window-avg), PlaceGraphPlanner,
  HeadingController (+advance), MotorBus, WhiskerSteer/StuckEscape, DescendingPredictor,
  RunTumbleNav (scalar mechanism), HomeokineticExploration/MotorEPM (PLAY substrate).
- New: the EFE ARBITER (N-loop), PLAY-as-a-heading-generator, the 2-fixed-food L-bend env,
  and the per-loop voter wiring.

*Living plan — re-wire as the loops teach us (agile, not rigid to this spec).*
