# Brain-Building Doctrine

*The reusable principles for building a new xaq brain, distilled from
building embodied active-inference agents. This is the "bible" — read it
before standing up a new capability. The Cell navigator is the running
example, but the principles are creature-agnostic.*

---

## 0. What we are building

A **System-1, embodied, active-inference** brain. The agent survives by **intrinsic
homeostatic valence** (food = energy, wall = pain), not external reward, and must
demonstrate **genuine problem-solving**, not reflexes. A wall-following escape + random
goal-stumbling is cheating. The bar is *reasoning*.

---

## 0.5 The rewrite rule — how to turn a wanted behavior into a legal build

*Everything below this section says what NOT to do. This section says what to do instead.
It is the positive form of the whole doctrine, and it is the step most often skipped.*

The universal wrong instinct — for a human engineer and for a coding agent alike — is
*"I want behavior X, so I will write something that produces X."* That yields a script:
it works once, generalizes to nothing, cannot improve, and cannot be de-scaffolded. The
correction:

> **Never implement a behavior. Implement the error the behavior minimizes.**

Given a desired behavior X:

1. **Name the error X descends.** What prediction, held by which module, would be *less
   wrong* if X happened? If X is not the descent direction of some module's own error, you
   are writing a trajectory, not a brain — stop.
2. **Check the SENSORY channel before designing any POLICY** (§2.1). Does an egocentric
   observation actually carry the signal that error needs? If not, **the fix is a new
   sensor — a new blanket channel — not a smarter policy.** A hidden state the blanket
   carries no observation of cannot be reached by a better policy, only stumbled onto.
3. **Give the owning module the observation AND the objective — never the motion.** A
   drive, a reflex, or a soft predictive target the controller descends toward; never a
   trajectory, a joint bias, or a schedule (§2.4: objective, not injection).
4. **Gate the term by the state it exploits — the gate is the design; the magnitude is
   only tuning.** An ungated bias fights the loop it rides on; the same term, gated on the
   state that makes it valid, is absorbed by the existing rhythm.
5. **Ship it gain-0-guarded and A/B it** (§8) — off must be byte-identical.

**Three worked examples, all of which began as the wrong instinct** (legged build, 2026-07):

| Goal | The script instinct (wrong) | What worked, and why |
|---|---|---|
| Hold a heading | a steering script / waypoint follower | **PD on the dead-reckoned OWN yaw** (integrated egocentric rotation — Markov-compliant, no external compass) routed through the *authoritative* locomotion channel. Straightness 0.05→0.53. Step 1: the error was "my believed bearing vs my intended bearing," which the agent can measure itself. |
| Climb a ridge the body high-centres on | a climb policy; imposing inter-leg timing (Cruse rules) | **A new sensory channel** — a belly-mounted rangefinder — plus removing a homeostat windup that fought locomotion. Step 2 was the whole answer: this looked like a policy problem and was a **missing-observation** problem. The imposed leg-timing was refuted twice (flat *and* incline). |
| Get the belly off the ground | a scripted lift; a constant knee bias | **A stance-gated knee tuck** — the bias applied only to *planted* legs (swing legs untouched). A blind DC knee bias destroys the gait; **the stance-gating is the entire trick** (step 4). Belly clearance doubled, ~20 % faster, zero falls, and it still cleared the ridge. |

**The through-line — LEARNED cooperates, IMPOSED fights.** Across the whole legged build,
every mechanism that let the body *discover* was promoted, and every mechanism that
*dictated to* the body failed in one of three signatures: **chaos, collision, or
stiffness.** When a contextual/learned form and a commanded form are both available, the
commanded one is the one that will be refuted. This is the compressed statement of §5
(imposed rhythms), §4 (teacher-distillation), and §6 (tuned constants) — three faces of the
same error.

---

## 1. Prime directive — always be predicting

Every module is a **predictive model**. The EPM's dual-TLE (forward + backward
prediction error) is the universal learning signal. Perception, the map, and action are
all **prediction-error minimization**. Predict in **latent space** — never reconstruct
pixels (JEPA). If a module isn't predicting something and being graded on the error,
question why it exists.

---

## 2. The defensibility bar (a–d) — the skeptic's checklist

Any "this is AI / active inference" claim must clear all four:

- **(a) Inferred, not oracle.** Perception is a latent *estimate* with its own error, not
  a ground-truth computation handed to the agent.
- **(b) Action reduces the agent's OWN inferred error.** A generative model predicts the
  next observation; action minimizes *that* predicted error / expected free energy —
  measured as such, not as ground-truth range. **← the crux.**
- **(c) Loop-isolation controls.** Severed / shuffled / lagged / wrong-sign / ablated
  arms. Every learned part ships with its ablation.
- **(d) Perturbation → re-inference → recovery.** Relocate the food / drop a sensor
  mid-episode; a real loop re-infers and re-acts. The sharpest single evidence. **Passed for the
  visual loop 2026-07-11**: a mid-episode sensor-DROPOUT window (lesion vision, then restore)
  degraded food-approach and recovered it, against a no-lesion control (§8 for the dense-proxy
  and load-bearing-regime technique that made it measurable).

Cybernetic feedback (oracle-bearing → proportional steer) **fails (a) and (b)**. It looks
like navigation; it is not inference.

- **The arbiter chooses a pathway INTO THE FUTURE.** Action is *policy selection*, not
  reaction: the EFE arbiter picks the policy (a future trajectory) that minimizes EXPECTED
  free energy — pragmatic (reach food) + epistemic (resolve uncertainty) — over futures,
  not just the best instantaneous heading. In this brain the **EFE arbiter carries that
  prospective decision** (it is where "choosing a future" lives), selecting among the nav
  components below it.

### 2.1 The Markov blanket & epistemic foraging — the deepest test

A loop (and the whole agent) is a **Markov blanket**: **sensory** states (world→agent) +
**active** states (agent→world), with internal states (the agent's *belief*) and external
states (the *hidden* world) conditionally independent **given the blanket**. The agent only
ever touches the world through this interface.

- **Hidden external states cannot be read — they must be INFERRED through action.** A
  scalar gradient's *direction*, the agent's *position* — these live OUTSIDE the blanket.
  They are not on the sensory side; no encoder can compute them from a snapshot. The agent
  holds a *belief* about them on the internal side, and the only way to gather the
  observations that sharpen that belief is to **act**.
- **Epistemic foraging (Friston).** Movement in service of inference: the agent ACTS to
  harvest the observations that reduce its uncertainty about hidden states. The klinotaxis
  weave (act to reveal a gradient's direction), the saccadic learning-walk (§7), run-and-
  tumble — all epistemic foraging. It is the *epistemic* term of expected free energy made
  physical. The predictable env (§8) is what makes epistemic foraging *measurable*.
- **The honesty test (predicts failures in advance).** A loop is honest when it **respects
  its blanket**: hidden states are inferred THROUGH action, never smuggled onto the sensory
  side. The canonical violation: trying to *read* a hidden external state from an
  instantaneous percept. (Falsified 2026-06-26: a static forward-model that tried to read a
  scalar gradient's *direction* from the instantaneous consensus — the direction is external
  and hidden, so the map was irreducibly ambiguous; it baked a useless average or never
  crystallized. Klinotaxis fixed it by *acting* — weave + lock-in — to infer the direction.)
- **Nested blankets ("blankets of blankets").** Each strange loop is a sub-agent inferring
  its level's hidden state through its own blanket; the descending predictor (§5) is the
  generative model on the sensory side; the EFE arbiter chooses which epistemic/pragmatic
  action to run. A foraging **path** is the trajectory the blanket traces under EFE descent
  — the epistemic term (weave, to infer) and the pragmatic term (steer, to reach) woven
  together.
- **Check the SENSORY channel before adding a POLICY — a failure may be SIGNAL-limited, not
  policy-limited.** A hidden state the blanket carries NO observation of cannot be reached by a
  better policy, only stumbled onto. Before decomposing a new selection loop to reach a goal, ask:
  *does any sensory channel carry the observation that would guide it?* If not, the fix is a new
  SENSOR (a new blanket channel), not a smarter policy. Falsified 2026-07-10: the eats bottleneck
  looked like a discovery/policy problem, so we improved the explore loop's coverage 2.5×, and eats
  did NOT move — because where scent was blocked the agent had **no food observation at all**, so
  more/better exploration only re-covered blind ground. Adding the missing channel (a visual food
  bearing) is what moved eats. Improving a policy is wasted effort if the observation it needs never
  crosses the blanket. (Corollary to the honesty test: a *missing* channel and a *smuggled* channel
  are the two blanket failures — one starves inference, the other fakes it.)
  **Independently re-confirmed on a SECOND creature (2026-07-23, legged).** A body that high-centred
  on a ridge looked like a locomotion-policy failure; several policies (imposed inter-leg timing,
  stuck→explore, a forward-flow homeostat) were built and all refuted. The actual fix was a **new
  blanket channel** — a belly-mounted rangefinder giving an egocentric ground-clearance observation,
  replacing a god's-eye chassis height that was blind to belly grounding and could not represent
  terrain at all. Two creatures, two modalities, same verdict: **check the sensory channel BEFORE
  designing a policy.** Because it now holds across creatures, treat this as a first-order rule
  (§0.5 step 2), not a special case.

### 2.2 How expected free energy is actually scored — the arbiter's honest ledger

The (b) bar demands action minimize the agent's OWN predicted expected free energy,
**measured as such**. A hand-scaled comparison of two heuristic value scalars (a "value
race" between a proximity level and a planner value) is NOT that — it is the mis-scaled
proxy §6 forbids, run on two incommensurable scales, so no fixed mixing constant is
stable. (This is the mechanism behind the near-food oscillation: a source-normalized
proximity that caps out vs. a self-normalized planner value that saturates near 1 cannot
be reconciled by a fixed cede factor.) When the arbiter scores a policy, three quantities
must be EXPLICIT and measured, never proxied:

- **Pragmatic term = divergence from the homeostatic prior.** The energy set-point is a
  PRIOR over preferred observations (the "C" vector). Pragmatic value is how far a
  *predicted* future observation sits from that preference — this is where intrinsic
  homeostatic valence enters the math, not as a side reward channel.
- **Epistemic term = expected reduction in belief entropy** about the hidden state (where
  food is) under each candidate policy. Movement that sharpens the belief scores here —
  epistemic foraging (§2.1) made quantitative, not a hand-set exploration bonus.
- **Horizon = how many steps the generative model is rolled forward. State it.** A
  one-step arbiter is GREEDY, not "choosing a pathway into the future" — name it as such
  (§8 honest reporting). "Into the future" requires a multi-step rollout of the generative
  model under each candidate policy; anything shallower is an instantaneous value
  comparison wearing prospective language.

The set-point itself may be fixed or slowly adapt (allostasis); if it adapts it is a
higher-level prior and must carry its own error.

- **A loop's value must be scaled in ITS OWN inference units — match the geometry of what it
  infers.** A DIRECTIONAL sense (a bearing) infers a *direction*, which is scale/distance-
  independent; scaling its confidence by apparent SIZE (a range/reach proxy) mis-weights it,
  making it confident only when already close — i.e. redundant with a true range sense, and mute
  exactly in the long-range regime that is its reason to exist. Fixed 2026-07-11: the visual
  loop's value had to be a **detection/direction confidence** (do I clearly SEE food), not the
  eat-calibrated *reach* (how big is the blob); the reach form let vision win only when klino
  already smelled the food, squeezing it out of its own seen-but-not-smelled regime. Give a range
  sense a range-scaled value and a bearing sense a detection-scaled value; the shared unit the
  arbiter needs is [0,1] confidence, not a common physical scale.

### 2.3 Precision is a controlled variable — the principled arbitration

Active inference is precision-weighting at its core: the balance between **sensory
precision** (trust the senses) and **prior/model precision** (trust the model) decides
whether the agent acts on what it sees or on what it believes. Precision is not only the
voter's static `1/(tle+ε)` trust — it is a gain the agent MODULATES with context (the
arousal/valence "neurochemical" state is precision control in its natural role, not a mood
decoration).

- **The near-food arbitration IS a precision problem.** Close to food, sensory precision
  (direct sensing) should RISE and prior precision (the planner's route) should FALL; far
  away it inverts. Cast the arbiter's mix as precision-weighting rather than a value race
  and "which one wins" answers itself from context instead of from a tuned constant.
- **Drive precision from the agent's own dynamics — never a hand-tuned constant** (§6).
  Running TLE, a channel's eat-calibrated confidence, or arousal set the gain. A channel
  earning confidence from its OWN consequences (its eats) is legitimate precision
  self-calibration; a designer picking the crossover point is the anti-pattern (the
  interim `×(1−…)` cede factor).
- **A remembered belief must be DISCONFIRMABLE and must yield to the epistemic term — an
  over-committed belief inverts precision-weighting.** A persisted target or a cached route is a
  *prior*; when it is UNCONFIRMED, sensory precision should RISE (look again), not the stale prior
  dominate. A belief that holds through mere decay — never actively disconfirmed by the agent's own
  looking — is a prior masquerading as inference, and it suppresses the epistemic foraging that
  would correct it. Falsified 2026-07-11: **visual target persistence** (remember the food's bearing
  and keep homing through occlusion — the visual analog of a run-commit) was NET-NEGATIVE on eats
  at every decay rate, because it over-committed to a STALE target (it rotation-corrected but never
  translation-corrected, and held through decay after the food was gone), muting the exploration
  that finds food. The purely **reactive** loop (re-acquire on each sighting) was strictly better —
  fast re-inference beats a held belief when the world is glimpsed often. A memory earns its keep
  only if it (i) decays FAST on disconfirmation (the agent acts to check and drops it when the check
  fails) and (ii) never silences the epistemic term while unconfirmed.

---

## 3. The decomposition methodology — how to build a capability

**Split the capability into parts that each (i) have an easily-learnable structure,
(ii) carry their own honest learning signal, and (iii) pass an independent gate. Prove
each ALONE. Let them become emergently complementary.**

- **A goal is learnable only where it has CONTENT.** Folding two goals into one selector
  → neither learns. Canonical split: *nav* ("create a heading toward food") vs *action*
  ("act on a heading") — two clean, independently-learnable objectives. It is not denser
  reward; it is a *cleaner* one.
- **Isolate first.** Prove a part in the simplest env that gives it content (the corridor
  turn-rig), where confounds are absent. The complex env (the maze) is the **final
  transfer falsifier**, never the development env — 10× harder, hides the signal.
- **Each part declares its gate up front** (e.g. "recover N eats with the scaffold
  removed", "same place → same node on revisit"). No gate, no part.
- **Measure a loop's VALUE by its goal-CONTRIBUTION (leave-one-out), NOT its arbiter win-fraction.**
  In an N-loop assembly the winner histogram is a DIAGNOSTIC (who drives when), not a value measure.
  A **specialist** loop can be decisive at tiny tick-share: the visual loop wins only ~2 % of ticks
  (its signal is FOV-gated — food is in view seldom) yet it roughly DOUBLES eats in its regime. Its
  value is in the eats it enables, not the fraction it wins. The honest value test is the leave-one-
  out ablation — assembly vs assembly-minus-loop — measured on the goal (eats), not on authority.
  (An always-explorer like PLAY will dominate the win-fraction because it wins whenever the agent is
  sated; that is its job, not its importance.)
- **Redundant-modality loops PARTITION by regime — isolate the regime to prove one.** Two loops that
  serve the same pragmatic goal through different senses (a scent closer and a sight closer) are
  REDUNDANT: whichever modality the environment favours dominates, and the other looks idle. To
  demonstrate a loop's value you must craft the env that ISOLATES its regime — a scent-poor,
  line-of-sight world to expose the visual closer — not a balanced env where the co-modal loop hides
  it. Corollary for the arbiter: redundant loops currently RACE (winner-take-all), so a redundant
  co-modal signal is discarded rather than fused; whether same-goal loops should be FUSED (confidence-
  weighted bearing average, as the LateralVoter does for perception) rather than raced is an OPEN
  architectural question (§5's voter-fusion vs §2's policy-selection) surfaced by the vision work.

---

## 4. The de-scaffold discipline — honest signals only

- **The honest, de-scaffoldable learning signal is ACTION-CONSEQUENCE / EGOMOTION-
  PREDICTION.** "I moved this way and my own sensor/energy changed thus." Analytic or
  teacher-distilled percepts are **bootstraps** — legitimate to start with, but flagged
  as scaffolds and de-scaffolded *where the action-consequence signal exists* (often only
  in the richer env / the map; teacher-free perception cannot be learned in an open arena
  with no such signal — proven).
- **The oracle line:** **Perception** (a transparent sensor reduction — a retina computing
  edges, a compass summing a nostril ring) can be a bootstrap. **Selection and belief**
  (where to go, where things are) must be **learned** — never hand the agent the answer.
  The violation to hunt is a decorative inference path with the real decision hard-coded.
- **Credit off the agent's OWN reward.** Interoception (energy gained, a hit event), not a
  mis-scaled proxy. A fixed threshold on an inferred proxy will mis-fire at tabula rasa and
  can invert the sign (punishing success). Use the ground-truth-to-the-agent signal it
  actually feels.
- **A belief's "honesty" is ENV-RELATIVE — match its persistence to the world's stationarity.**
  A persistent food-cache is a *lie* only if the food is NON-STATIONARY; against a stationary or
  periodic world it is a CORRECT prior, and forgetting it is the error. Observed 2026-07-11: the
  "dishonest immortal-cache" planner (`PlaceGraphPlanner`) BEAT the "honest forget-on-arrival"
  planner (`PlaceNav`) in a 2-fixed-site alternating-food env — because the food really does return,
  so the cache was accurate and the honest planner's caution left it winning ~0 %. The rule is not
  "always forget" or "always persist"; it is *hold a belief exactly as long as the hidden state it
  tracks stays put*. A held belief and a disconfirmable one (§2.3) are the same principle at
  different world-timescales: persistence should equal the world's own autocorrelation time.
- **Sensor honesty = morphology honesty.** Match senses to the creature. An insect /
  compound eye has **no instantaneous depth** — that is lidar/sonar, a different creature.
  Insect depth comes from **motion**: optic flow / parallax, looming (LGMD), active
  peering. Never add a sensor that silently redefines the creature and that you could not
  later remove — that is building a corner you cannot de-scaffold from.
- **Falsified anti-patterns (2026-06-24) — a percept that doesn't PREDICT with its own
  error is doomed.** Two attempts to *learn* a perceptual front-end both looked plausible
  and both collapsed, because neither implemented the action-consequence signal above:
  - **Teacher-distillation is a photocopy, not a de-scaffold.** A learned percept that
    EMAs toward an analytic teacher's output (`BearingEstimator` distilling the scent
    compass) has *no error of its own*. Lesion the teacher and the frozen readouts can't
    generalize to the inputs the agent then visits → the percept collapses (inferred
    |bearing| 0.98 → 0.22) → behavior dies. **A copy dies with its source.**
  - **Eat-association on a raw feature learns the background.** A learned food-detector
    that EMAs the central-frame *mean colour* on each eat (`VisualBearing`) captures
    whatever fills the view — mostly background — and homes toward it (walls), cutting
    foraging. The *consequence* (the eat) was right; the *feature* was junk. Cluster with
    the EPM and learn the feature that **predicts** the consequence; don't average raw pixels.
  The fix for both is identical: predict with the agent's **own action-consequence signal**
  (§1), EPM-clustered on a **conditioned** input (§5, §6) — in-runtime, no teacher, no
  backprop. If a "learned" pathway has no prediction graded by its own honest error, it is
  one of these anti-patterns wearing a new name.

---

## 5. Proven substrate patterns — reuse, don't reinvent

- **Add DRIVES, REFLEXES and OBJECTIVES that motion EMERGES from — never scripted
  TRAJECTORIES.** The operational form of "learned cooperates, imposed fights" (§0.5).
  A term that specifies *what to achieve* is absorbed and improved by the loops below it;
  a term that specifies *what to do* fights them. Imposing inter-leg timing on an emergent
  gait was refuted twice; the same energy spent as a *predictive target* was promoted.
- **Gate a bias by the state that makes it valid — the gate IS the design.** An always-on
  additive term rides on top of the loop and fights it; the identical term, gated on the
  state it is meant to exploit, is absorbed by the existing rhythm and compounds with it.
  Proven 2026-07-24: a constant knee-tuck bias destroyed a working gait, while the *same
  bias gated to planted (stance) legs only* — leaving the swing phase untouched — raised
  the body clear of the ground, moved ~20 % faster, and cost nothing elsewhere. When a
  bias fails, ask what state should have gated it before concluding the idea is dead.
- **Layered, additive construction — "strange loops" (Friston).** A closed sensorimotor
  loop, once it works, is FOUNDATIONAL substrate — it is **never disabled**. Build the next
  layer ON it; do not replace it or switch it off. The brain grows in layers and every
  early layer is foundational (the action→heading loop, the CPG temporal loop, …). De-
  scaffolding removes a temporary TEACHER/ORACLE *prop*, not a working loop — and a lesion
  is a *test* ("does the new layer stand alone?"), **never the operating mode**. Corollary:
  a higher layer must feed the **full** lower loop, not a stripped one. (Driving a new nav
  module into a HeadingController with its learned advance switched OFF was this mistake.)
- **EPM as the first input module, always.** The GNG + dual-TLE perceptual unit is proven;
  defer to it and tweak for the use case rather than hand-rolling a clusterer. (A raw
  L2-VQ underperformed the EPM's encoding on the same signal.)
- **Clock / CPG → EPM → LateralVoter = shared temporal context. LOAD-BEARING.** A simple
  oscillator's reality token, voted alongside everything else, stamps every contribution
  with *phase / when*. In a legged-standing build this was decisive — a
  controller **only worked with it; the ablation broke it.** Phase is the shared
  coordinate that lets downstream bind events in time.
  Deeper role: the CPG is also a **demodulation reference**. When a signal cannot be read
  instantaneously (a scalar gradient's *direction*), a self-generated rhythm lets the agent
  ACT to reveal it — weave at the CPG frequency, then **lock-in detect** (correlate the
  noisy scalar with the weave phase, low-passed over a few cycles): the component varying AT
  the rhythm's frequency is extracted while out-of-band noise averages out. Klinotaxis is
  this. The CPG is how you act to make an unreadable signal readable.
- **LateralVoter = precision-weighting + informativeness gate.** Fusion lives here:
  trust = 1/(tle+ε), dead/degenerate channels stripped by baked-node informativeness.
  Multi-modal fusion is consensus, not a hand-weighted sum. Precision here is a STATIC
  trust; §2.3 covers precision as a CONTROLLED variable the agent modulates with context.
- **MotorBus subsumption + authority-gated learning.** Reflexes override the cognitive
  channel on the bus *and* suppress its learning by authority share, so reflex-driven
  motion isn't miscredited to the policy. Proportional turn-priority mixing keeps each
  channel's authority at speed.
- **Hierarchical / slow EPM on the consensus stream = higher-level abstraction.** A slow
  EPM (rate-limited by a KeyframeAverager) clustering consensus tokens forms higher-order
  nodes — e.g. the cognitive **map** (nodes = places, transitions = edges, TLE = predict-
  next-place).
- **Error attribution across the hierarchy — a low-level failure must not be credited to a
  high-level policy.** With nested blankets (§2.1), a failed prediction belongs to the level
  that owns the wrong belief: a map misprediction is the map's error (or the perception
  feeding it), NOT the policy that consumed the map. Miscrediting a perception/locomotion
  failure to the cognitive policy is a known hierarchical-predictive-coding trap and a
  recurring one here (reflex motion mis-scored to the policy → the MotorBus authority gate).
  Each level grades itself on ITS OWN prediction, taking the level below as its observation.
- **The growth trigger — when to add a loop vs. tune one.** Spawn a new node/layer only
  where prediction error is PERSISTENT, IRREDUCIBLE, and LOCALIZED to a region of state
  space — the existing loops have converged yet still mispredict there. Everywhere else,
  adapt the loops you have (§6). This is the spawn criterion behind mitosis / fractal
  expansion: irreducible residual TLE in a state region is the gradient that earns a new
  sub-agent; transient or globally-diffuse error does not.

---

## 6. The scale-sensitivity laws — tabula-rasa traps

Tabula-rasa signals are tiny; fixed scales and thresholds hide them. The same disease
recurs at every layer.

- **Node-creation sensitivity hides detail.** The encoder/PCA can show clear
  differentiation while the GNG insertion gate (fixed quant-error / TLE threshold)
  collapses it to one node — because a tiny directional signal rides on a large
  common-mode. **Condition the input** (center to remove common-mode, normalize for
  scale) so the differentiation survives discretization.
- **Value / reward scale.** A tiny progress signal (ΔΧ per step) is swamped by the
  exploration bonus / softmax temperature. **Whiten the reward by its own running
  magnitude** so the value table lives in O(1) units comparable to exploration.
- **No tuning — adapt instead.** Never tune a static parameter to a signal's scale. Add
  the mechanism that sets it from the system's own running dynamics (running variance,
  spread-normalized softmax, count-annealed exploration).

---

## 7. Place-coding & mapping doctrine

- **Saccadic active sensing = learning-walks.** On arriving somewhere novel, pivot in
  place (phase-locked to the clock) to scan the surround. The pivot is an **epistemic
  action** — movement in service of perception (the (b) bar). This is how ants/bees
  memorize a place.
- **A place ("cylinder") = a heading-indexed, time-smoothed panorama.** Per-tick heading
  indexes the sweep; time-smoothing makes the embedding robust; the whole-surround
  signature disambiguates where a single frame aliases (the insect *snapshot* model).
- **A place needs four axes**, or it aliases:
  1. **appearance** (vision — what the walls look like),
  2. **orientation** (heading, encoded sin/cos so ±π doesn't fracture a zone),
  3. **metric / translation** (**path-integration** of self-motion — the grid-cell role;
     drifts, corrected by visual loop-closure. NOT a depth sensor),
  4. **temporal / transition context** (the EPM's "where I came from" resolves residual
     aliasing — HMM-style).
- **Transitions must be action-conditioned — `P(place' | place, action)`.** A place graph
  whose edges aren't labeled with the action that traverses them is a RECOGNIZER, not a
  plannable substrate: the nav planner ("create a heading toward food") has nothing to
  select over. Predict-next-place (§5) must be conditioned on the egomotion between places,
  or the map cannot support policy selection.
- **Perceptual aliasing is the fundamental SLAM failure.** "Twisty passages all alike" →
  every place collapses to one node. **Design the world to be informative** (distinct
  per-zone wall colour/texture) — places cannot coalesce into zones until they look
  different. This is a prerequisite, not a polish step.
- **View-dependent (place+heading) nodes first; view-invariance later.** More nodes, each
  stable; the transition graph still works.

- **★ Two clocks a controller ASSUMES are one will beat — and the beat looks like the
  behaviour "keeps losing it." Measure the phase-lock between them.** A quadruped's power
  stroke was timed by a phase read off the knee while the leg's actual step cycle ran ~25 %
  slower. Nothing forced them to agree, so the fraction of *stance* spent in the stroke's
  propulsive half was 0.512 and over *swing* 0.513 — push direction statistically
  independent of ground contact, half the stroke spent in the air. The two clocks beat at
  ~2.5 s, which is exactly the period at which the operator saw the gait alternate between
  "that worked" and "stumbling." **Whenever one signal times an action and a different
  signal decides when that action is valid, they are two oscillators until proven otherwise;
  a phase-locking value at the event that matters (here: touchdown) is a cheap, decisive
  test.** The deeper cost is silent: EIGHT successive levers re-tuned coordination *between*
  limbs while the thrust↔support relation *within* a limb stayed random, and every one of
  them measured null. **A coupling parameter cannot help while the thing being coupled is
  internally incoherent** — check the within-unit relation before tuning the between-unit one.
- **★ Before optimizing a posture, check whether the JOINT AXIS can even produce the change you
  want.** A quadruped's feet planted at 170 mm against a 166 mm total leg reach — straight-legged,
  maximum moment arm — and the obvious fix was "shorten the stride so the feet land closer in."
  A 1.8× stride sweep moved the foot radius by **under 1 mm**, because the fore-aft joint rotates
  about the **vertical** axis: it sweeps the foot along an arc at *constant radius*. Stride length
  and foot radius were kinematically independent the whole time. **Read the axis before designing
  the lever** — an hour with the CAD table would have refuted that pathway before a line of code.
  Corollary, which is the more valuable half: **when a desired posture is blocked, enumerate every
  joint that could produce it and check each against a measurement.** Here all three routes closed
  and closed *differently* (hip2 trades against ride height, the knee trades against feet-in,
  stride cannot move it at all) — and only running all three showed the constraint was the LIMB
  GEOMETRY, not any control law. That is a body-design finding, and no amount of tuning reaches it.
- **★ Confirming an operator's CONCLUSION is not confirming their MECHANISM — separate them, and
  say which one survived.** Three times in one session the proposed action helped and the proposed
  reason did not: "the swing leg spins the chassis" (yaw impulse was measured *lower* during swing
  than during full support, yet lifting the limb still quadrupled the step rate — the real
  mechanism was foot clearance); "a vertical shank gives mechanical advantage" (the moment arm
  barely moved, yet verticality tracked `scrub` and straightness monotonically — the real mechanism
  was that a vertical shank stops pushing sideways); "shorter steps bring the feet in" (foot radius
  invariant, yet a shorter stroke walked 12 % further — the body was simply over-striding). **A
  lever adopted for the wrong reason will be generalized in the wrong direction**, so the mechanism
  has to be instrumented separately from the outcome, and a null on the mechanism reported even
  when the outcome is a win.

- **★ A magnitude gate cannot fix a timing problem.** Gating the same stroke by measured
  per-leg load — the doctrinally correct "gate the bias by the state that makes it valid" —
  fired cleanly across a 6.6× authority range and moved nothing: it suppresses thrust spent
  in the air, but *during stance* it scales the push and drag halves equally, and that
  balance was the 50/50 defect. **When a quantity is mistimed rather than mis-sized, scaling
  it is the wrong axis, however well the gate is built.** The corollary is a useful filter:
  before building a gain, ask whether the failure is one of AMOUNT or of WHEN.

---

## 8. Process discipline

- **★ A SCENARIO CAN SUPPRESS THE FAILURE YOU ARE TRYING TO FIX — check the environment's
  geometry before trusting the metric.** Every measurement tool in a legged-robot project defaulted
  to a corridor gym, and that corridor was built *"inside self-centering 30° walls"*: the geometry
  actively re-centred the body, so a heading disturbance was corrected by the wall before it could
  reach `straight` or `turns`. An entire family of anti-yaw levers was being scored in the one
  arena where its target failure could not appear. The same baseline measured `step_bal` **0.44 in
  the corridor and 0.07 on open ground** — the deficit was 6× worse in the gym nobody was
  measuring. **Two rules: run the lever in the scenario where its failure is VISIBLE, and when two
  scenarios disagree, that disagreement is data about the scenarios, not noise to average away.**
  (It cuts the other way too: a 7× improvement on open ground shrank to a tie in the corridor,
  because the corridor baseline had no headroom left. Neither number was wrong; a single-gym
  verdict was.)
- **★ KNOW YOUR HARNESS'S MEASURABLE RANGE, AND GUARD IT — a too-long run penalizes the
  arm that works.** A 9.5 m curriculum sat on a 20×20 floor. Run long enough and the fastest
  arm walks off the end: it posted the best distance in the whole campaign *while its chassis
  was 39 m below the floor*, and the harness charged it a `fall` and a poisoned height for
  doing so. Re-run inside the gym, the same arm showed no gain and no falls — so the long run
  would have recorded a false positive AND a false negative **on the same lever**. Note which
  way the bias points: **the boundary is hit first by the fastest arm, i.e. precisely the one
  a propulsion lever exists to demonstrate**, so the error systematically rejects real wins.
  Two rules follow. *Measure the environment's limits before trusting a metric that
  accumulates against them*, and *put the check in the tool*: emit a loud warning naming the
  offending seeds rather than relying on anyone to remember. A standard protocol's run length
  is often an undocumented boundary constraint — find out before you "improve" on it.
- **Predictable env for measurable epistemic foraging.** Epistemic foraging — acting to
  resolve uncertainty about *where food is* — is intrinsically MESSY. To measure learning,
  the environment must be PREDICTABLE: deterministic structure the agent can come to KNOW
  (a small set of **FIXED** food locations, never random respawn). The success signature is
  the **eat rate RISING over time** as the agent predicts its world — not a high
  instantaneous rate, and not beating the reflex. A rising rate is **necessary but not sufficient**: a
  deterministic env with fixed food admits a memorized OPEN-LOOP action sequence that lifts
  the rate with zero re-inference, so the rise counts as inference only when it SURVIVES the
  (d) perturbation — relocate the food mid-episode and confirm it re-infers. (Cell
  measurement env: **2 fixed food
  spawns, one each side of the L-bend wall** — one direct, one occluded.)
- **Fast-fail staged iteration:** pilot (wiring) → signal → direction → powered (n≥10/20),
  with explicit promote-or-kill gates. Never skip the wiring or the powered stage.
- **Scale the CLAIM to the POWER — a single-seed n=5 result is a SIGNAL, not a finding.** Rapid
  iteration on the Cell runs at n=4–6 with a **FIXED world seed**, varying only the policy RNG.
  That is adequate to PROMOTE-OR-KILL a direction; it is NOT a defensible effect size, and it is
  especially not a defensible **NULL**. Two traps recur: (i) a fixed world seed leaves **env
  variance unsampled** (one pillar layout, one trajectory family) while the eat metric swings wildly
  (0→6 at fixed seed), so a paired-seed win can be a lucky configuration; (ii) a **claimed NULL**
  ("X doesn't move eats") is a Type-II risk until powered — an underpowered null is not evidence of
  no effect. The powered stage for any loop claim is: **n≥20, VARIED WORLD SEEDS, paired-t +
  bootstrap CI, AND the (d) perturbation** (§2). Until then, report "signal," not "result" — most
  current Cell loop numbers (a +2.5× discovery, a +170% eats, a persistence NULL) are signals
  pending powering. Peer-review honesty is refusing to promote a signal to the doctrine as a finding.
- **Judge on the FULL metric set, never one number — hunt the BLIND METRIC.** A metric that
  a *degenerate* behavior can satisfy will certify that degenerate behavior. This trap has
  been hit repeatedly, at every layer, and each time the fix was adding the metric that
  *sees* the failure — not re-tuning against the blind one:
  - net-rotation is **blind to a body that swings its heading and nets ~0** → use
    straightness (net displacement / path length).
  - chassis height is **blind to belly-drag** (the chassis rides high while the belly
    grinds) → measure the ground clearance of the part that actually contacts.
  - a mean forward speed is **oscillation-dominated** → read it *with* net displacement;
    high speed + low displacement is fast circling.
  - a fast-traversing gait scored well on speed + joint-coherence while **slamming the
    chassis into the ground** — the collision was invisible until a chassis-height metric
    was added.
  The rule: before trusting a result, ask *what degenerate behavior would also score well
  here?* If one exists, you have a blind metric — add its complement first. Corollary: a
  metric set is only honest once no single number can be Goodharted alone.
- **A NUMBER OUTLIVES THE BODY IT WAS MEASURED ON — carry provenance, or a rejected result
  will be re-promoted.** When a configuration is rejected, its *metrics do not die with it*:
  they survive in summary tables, scorecards and handoff notes, detached from the verdict on
  the thing that produced them. One layer of summarization later, nobody can see that the
  headline number came from a body already judged non-viable. Observed 2026-07-25 (legged): a
  "+65 % distance" speed lever was carried forward as a banked win for months — it had been
  measured on an **open-loop servo sequencer that was rejected the previous day** on operator
  observation (it slammed the chassis into the ground every step). The number was real; the body
  was not viable; and the *operator's memory of how the gait looked* — sequenced and
  paddle-like, not alive — was what caught it, long after the metric had passed unchallenged.
  Two rules follow: **(i) every recorded number names the configuration it was measured on**,
  so a rejection can be propagated to everything downstream of it; and **(ii) when a config is
  rejected, sweep its metrics** — anything still circulating from it is now a claim without a
  body. Corollary: this is the strongest argument for §8's operator/UI diagnosis being
  first-class. A qualitative memory of *how the thing moved* is a durable check that aggregate
  metrics, once detached from provenance, cannot provide.
- **A NULL IS A CLAIM ABOUT THE MEASUREMENT AS MUCH AS THE MECHANISM — validate the
  BASELINE before believing one.** *When the control sits at a degenerate attractor, "the
  mechanism didn't lift the mean" is a fact about the control, not the mechanism.* This is
  the most expensive lesson in this project's history. A systematic audit (2026-05-31) found
  **84 % of analyzable historical runs had a single channel taking >60 % of the signal** —
  the controls themselves were degenerate — which put an entire era of "this mechanism is
  null" verdicts back in question. Two were re-tested against a recovered baseline; one null
  held, and the framing "we can't make the body do X" turned out to be a *measurement*
  problem, not a substrate problem. Corollaries, each earned:
  - **Check the control is HEALTHY before interpreting a null.** A degenerate control makes
    every mechanism look inert.
  - **A negative result can be a TAUTOLOGY (the knob was already on — the variant was
    byte-identical) or DEAD CODE (the path wasn't live in that config).** Neither is a
    verdict on the idea. Read the running params and verify the consumer fires *first*.
  - **A weakened slice is not the mechanism.** A structural-prior null once rested on an
    implementation carrying 1 of the theory's 6 rules; that falsifies the slice.
  - **A silent confound invalidates wholesale and invisibly.** A harness flag that never got
    set meant an entire family of A/Bs silently ran the wrong stage for weeks. When a result
    family looks strangely flat, suspect the harness before the substrate.
  The operating consequence: **no mechanism here is ever "dead" — only refuted in the
  context, at the power, and against the baseline it was tried with.** Record all three with
  every verdict, and record the *re-use context* that would justify trying it again.
- **Refutation is SCENARIO-SCOPED — "refuted" without a regime is not a verdict.** A lever
  falsified in one regime may be load-bearing in another, so a refutation must name the
  scenario it was run in and be re-tested when the regime changes. (Inter-leg-timing rules
  were refuted on flat ground, then deliberately **re-tested on the incline** where their
  premise — grip — should have applied, and refuted again; that second test is what made
  the verdict trustworthy. Conversely a stuck→explore lever was "refuted" on flat, which was
  simply the wrong scenario to test it in.) Treat every idea as valid until falsified **in a
  similar scenario**; keep a ledger of verdicts *with their regimes*, or the same idea will
  be re-proposed and re-killed indefinitely.
- **Verify the CONSUMER fires, not just the producer.** A published topic nobody acts on
  is a silent dead channel.
- **Running the (d) perturbation test — two techniques that make it measurable.** (i) When the
  goal is SPARSE (eats come ~1 per tens of seconds), a phase-binned eat *rate* cannot be populated
  in a reasonable episode — use a DENSE goal-approach PROXY instead: **mean distance-to-goal per
  phase** (thousands of samples), which directly measures the loop's causal role in *reaching*, not
  the rare arrival. (ii) A dropout test only shows signal in the regime where the loop is
  LOAD-BEARING: if a floor loop (PLAY's random walk) can COMPENSATE for the lesioned loss — as it
  does for the visual closer in a *small* arena — the lesion produces no degradation, and you would
  wrongly read the loop as decorative. Size the perturbation env to the loop's own regime (§3
  regime-isolation applies to (d) tests too: a 40 m scent-poor room, where random exploration cannot
  cover the ground, exposed the visual loop the 16 m room hid). A specialist loop's (d) effect is
  MODEST but real — the visual dropout shifted food-approach ~10 % (it acts only ~5 % of ticks); the
  (d) PASS is in the SIGNATURE (degradation + recovery + control-isolation), not a large effect size.
  Pair the lesion arm with a no-lesion CONTROL to subtract over-time drift.
- **Honest reporting.** Scale claims to evidence; if it ties the baseline, say "ties." Name
  scaffolds as scaffolds. A graceful-degradation result is not a "beats both" result.
- **Downvoting is as important as upvoting — and it is NOT reward shaping.** Learning from a
  FAILED prediction (a negative OWN-consequence: Δscalar fell, the eat didn't come) is the
  honest prediction-error signal in the negative direction — as essential as the positive.
  This does not violate no-reward-shaping: shaping injects an EXTERNAL designed reward;
  downvoting on the agent's own consequence is the real error. Always diagnose the ELEMENTS
  and CAUSES of a failed prediction before the next attempt — for the agent (its TLE) and
  for US: we discover each loop's recipe by descending our OWN epistemic EFE, so a falsified
  hypothesis is not waste, it is the gradient. Look back at why it failed; let it shape the
  next prediction.
- **Operator meter/UI-driven diagnosis is first-class.** Watching the inspector (PCA
  scatter, trust weights, value landscape) catches what aggregate metrics hide — the
  node-creation-sensitivity insight came from the PCA showing detail the node count didn't.
- **Default-off / opt-in.** Every new module defaults off so existing envs stay
  byte-identical; the env-target guardrails (allowlist, claim-mode) port across creatures.
- **State the principle abstractly; name the biological instance as the example.** This
  doctrine is creature-agnostic only when the reusable mechanism is separable from its
  biological label: write "act to demodulate an unreadable signal via a self-generated
  rhythm + lock-in detection," THEN "(klinotaxis)". Principle first, instance as
  illustration — it ports across creatures and survives renaming.

---

*Living document. When a new principle is proven (or falsified) on a build, fold it back
here — the principle stated abstractly (§8), the creature as the illustration. Two builds
currently exercise and stress-test this doctrine:*

- [`reports/cell_markov_blanket_loops_report.md`](reports/cell_markov_blanket_loops_report.md)
  *— the Cell (swimmer): where the method was learned, and where two of its own headline
  results are overturned once properly powered.*
- [`reports/picrawler_gait_loop_findings.md`](reports/picrawler_gait_loop_findings.md) +
  [`reports/picrawler_lever_ledger.md`](reports/picrawler_lever_ledger.md) *— the picrawler
  (quadruped): the method applied to a hard body, and the per-lever verdict ledger. **Read
  the ledger before proposing a lever** — most plausible ideas here have already been tried.*

*Operating procedure for working in this repo — build recipe, the A/B protocol, the
vocabulary — is in [`CLAUDE.md`](../CLAUDE.md).*
