# Slow-Loop Design Notes

*Working notes on hierarchical predictive coding for xaq — the interface between fast
Episodic Predictive Modules (EPMs) and a slower, higher-level loop; when a slow loop
should be spawned; where its output plugs back into the agent; and how to make its own
predictions better. Companion to `brain_building_doctrine.md`, not a replacement for it —
every mechanism below should clear the same defensibility bar (doctrine §2) and growth
criterion (doctrine §5) before it's implemented.*

*References written `doctrine §N` point at `brain_building_doctrine.md`. A bare `§N` is a
section of this file. The two numberings collide at §2, §3 and §5, so the prefix is
load-bearing.*

---

## 1. The fast → slow interface

**What ascends.** Subscribe to `consensus.0`. The `ConsensusToken` already carries the
fused embedding, `fused_tle`, and a `trust_weights` map from each contributing topic to the
weight it was given (`primitives/LateralVoter.md`), so per-channel precision is available
without bypassing the voter.

Bypassing it would cost two load-bearing things. Modality-group balancing, which the
voter's contract calls its single most load-bearing piece, keeps a high-cardinality group
from drowning a low-cardinality one; and the informativeness gate strips dead and
degenerate channels by baked-node informativeness before anything is weighted
(doctrine §5). Feeding the slow loop from raw per-EPM tokens hands it precisely the
channels the voter exists to remove, against doctrine §5's rule that a higher layer feeds
the **full** lower loop rather than a stripped one.

Subscribing to consensus is also the cheapest possible build: a Level-1 EPM is the same
code as a Level-0 EPM with `input_topic` on `consensus.0` and an identity encoder
(`primitives/EPM.md`). Hierarchy is configuration.

Pooling and recompression become necessary only when the fast-EPM roster starts growing
(§2), since a fixed-slot representation would need resizing on every spawn. At that point
the aggregator has to be frozen and group-balanced — `fusion_notes.md` §3 covers why the
textbook Deep Sets form fails on both counts. Until then it is width the design does not
need.

**Sampling: replace time-averaging with event-gated keyframes.** The existing sampler is
`KeyframeAverager`: a rolling per-tick mean over a window of N ticks, published as a
ProprioToken for a slow EPM running `process_every_n_ticks`
(`primitives/KeyframeAverager.md`; `KeyframePeakDetector` is the same shape with a peak
instead of a mean). A rolling mean blends together states that were genuinely different,
and the hypothesis under test here is that this is the source of the slow level's noise.
It is a hypothesis, not a diagnosis — gate 1 below is what would confirm it.

Two triggers, both derivable from signals already on the bus:

- Take a keyframe when the fast loop is **settled** — consensus Time-Loop Error (TLE, the
  dual forward/backward prediction error every EPM already produces) low and stable for
  some duration. Sample when confident.
- Take a keyframe when TLE **spikes** — a sudden jump signaling a genuine context or
  place transition. This is the event-segmentation trigger: update on accumulated error
  crossing a threshold, not on a clock.

The settled trigger needs a second condition or it inverts. A frozen or dead channel
produces the lowest and steadiest TLE in the system, so "sample when confident" samples a
stalled agent hardest and fills the slow map with keyframes of nothing happening. That is
the blind metric CLAUDE.md §3 rule 4 demands a complement for, and it is doctrine §2.3's
law in sampling form: prediction error is not a proxy for competence, activity is. Gate the
settled trigger on low TLE *and* non-trivial latent displacement since the last keyframe.

Neither threshold may be a constant (doctrine §6), and both already have homes.
`neuro.state` scales the EPM's `novelty_threshold`, `mitosis_error_threshold`, `epsilon_b`
and `min_insertion_error` per tick, and the LateralVoter already raises a flag when
`fused_tle` crosses its own `novelty_threshold` — the flag Phase 4 fractal mitosis
consumes. The spike trigger should be that flag rather than a second comparison beside it.

**Gate the slow update by precision.** The slow EPM's own update should be scaled by the
pooled confidence — a Kalman-style gain — so a keyframe captured under high fast-level
confidence moves the slow belief more than one captured during an ambiguous state.
`trust_weights` supplies the per-channel term. Without this the slow loop inherits the same
noise problem one level up.

This weight owes an activity term like every other (doctrine §2.3). Confidence read from
TLE alone trusts a channel most once it has stopped moving: a killed picrawler leg's own
residual fell to 26 % of baseline, so error-weighted trust would have handed it *more*
authority. Name the activity signal for this stage — per-channel latent displacement
between keyframes is the candidate — and keep a wrong-sign arm, which is what separates a
weighting that helps from one that merely stiffens the system.

**Keep error attribution local.** The slow EPM forms its own dual-TLE, predicting the
*next* keyframe (or next place, in the map case), graded on its own accuracy. The fast
loop's TLE is a *gate* — when to sample, how much to trust a sample — never the slow
loop's training signal directly. This is the existing hierarchical error-attribution rule
(doctrine §5): each level grades itself on its own prediction, taking the level below as
its observation.

**Order of build.** This section is five levers — consensus subscription, event-gated
sampling, precision gating, pooling, recompression — and CLAUDE.md §3 rule 1 admits one at
a time. Suggested order, promote-or-kill between each:

1. Event-gated sampling alone, against the `KeyframeAverager` baseline on the existing
   `consensus.0` path. Cheapest, gain-0-guardable, and it tests this section's central
   empirical claim.
2. Precision-gated slow update, with its activity term and a wrong-sign arm.
3. Group-balanced pooling and recompression, once the roster actually grows (§2), not
   before.

---

## 2. When does a slow loop get spawned

The doctrine's existing growth criterion (doctrine §5, "the growth trigger") is stricter
than raw entropy in the consensus graph: spawn only where prediction error is
**persistent, irreducible, and localized** to a state region the existing loops have
already converged on. Proposed refinement — three gates, in order:

1. **Localization.** Is the residual concentrated in a region of state space, or diffuse?
2. **Persistence.** Does it stay flat despite continued exposure — ruling out "just needs
   more time to converge"?
3. **Instrumental value.** Would resolving this uncertainty actually change the arbiter's
   policy choice? Reuse the epistemic-value computation the Expected Free Energy (EFE)
   arbiter already runs (doctrine §2.2) rather than building a second uncertainty metric.

Entropy's proper role is as a cheap **localizer** — where to look — not a gate for whether
to commit. A purely stochastic, unlearnable channel will show high entropy indefinitely; it
passes gate 1 but fails gate 2 (never converges) and should fail gate 3 (irrelevant to any
policy). Gate 3 is the one that stops the agent from spending substrate on noise it can
never model away — the same failure the doctrine's scale-sensitivity laws (doctrine §6)
warn about for other metrics that a degenerate case can satisfy.

Gate 1 has a cheap existing input: the LateralVoter already flags consensus ticks whose
`fused_tle` exceeds its novelty threshold as candidates for higher-level mitosis. That flag
is a localizer in exactly the role gate 1 wants — it says where to look, never whether to
commit.

---

## 3. Where the slow loop's output plugs back in

**Multi-step rollout / subgoal generation — the highest-leverage return path.** The
doctrine's own honesty ledger (doctrine §2.2) already names the gap: a one-step arbiter is
greedy, not prospective, and "choosing a pathway into the future" needs a multi-step
rollout of the generative model. Rolling the fast, continuous dynamics forward N steps is
expensive and compounds error. A slow EPM's graph — coarse nodes, action-conditioned
transitions — is cheap to roll forward many steps, because each node already absorbs a lot
of fast-timescale variability.

The rollout machinery exists. `GNGRollout` samples K trajectories M steps forward by
weighted edge traversal over any EPM's GNG topology, returning `terminal_values` as
expected drive-error reduction and `entropy` over the K samples, answered as a REQ/REP
service (`primitives/GNGRollout.md`). Pointing it at a slow EPM's graph is configuration,
not a new mechanism. Proposed use: the arbiter searches paths through the slow graph
(shortest-EFE-path) and hands down a **subgoal** — a target node several steps out — as a
temporary preference for the fast arbiter to pursue with its ordinary one-step machinery.
Slow loop picks *where*; fast loop picks *how to get there right now*.

Two pieces of prior art bear on this, and neither is favourable by default.

*Horizon > 1 has already measured worse.* GNGRollout's own contract records the v3
observation that raising the horizon hurt performance, with the diagnosis — chained
argmaxes over a frequency-based transition table amplify noise as the horizon grows — and
the fix, which is the stochastic sampling GNGRollout does instead. The re-use context for
retrying it is this section's own argument: a slow graph's coarse nodes absorb
fast-timescale variability, so the per-step noise being compounded is smaller. That is a
claim to measure, not an assumption to build on (doctrine §3.1).

*A held subgoal is a held belief, and the nearest thing tried was net-negative.* Doctrine
§2.3 records visual target persistence — remember the food's bearing and keep homing
through occlusion — as losing eats at every decay rate, because it over-committed to a
stale target and muted the exploration that would have corrected it; the purely reactive
loop was strictly better. The same bullet gives the two conditions a memory must meet: it
decays fast on disconfirmation, and it never silences the epistemic term while
unconfirmed. A subgoal held for N steps is that mechanism one level up. It has to be
droppable the moment the fast loop's own observations contradict the route, and the
arbiter's epistemic term has to stay live underneath it.

There is also a rival answer to this section's premise. If the objection to fast-level
rollout is cost, `SequenceGNG` motif-teleport already collapses M fast ticks into one O(1)
lookup whenever a baked motif matches — planning via chunks. The slow loop's distinct claim
is therefore not cheapness but abstraction: motifs are action-sequence chunks, whereas slow
nodes are places or regimes. Worth stating, because if cheapness were the whole argument
the chunking path is further along.

**Allostatic set-point ownership.** The doctrine already notes that if the homeostatic
preference (the "C" vector) adapts over time, that adaptation is a higher-level prior and
must carry its own error (doctrine §2.2). That's a second natural attachment point: the
slow loop owns slow drift in what the agent wants, not only where it's going.

**Top-down precision modulation.** The slow loop's current belief about context or regime
(home territory vs. unexplored, say) is exactly the contextual signal the
precision-as-controlled-variable principle (doctrine §2.3) calls for. Let it bias which
fast channel's trust gets boosted or suppressed, extending precision control to a genuine
top-down channel rather than one internal to the LateralVoter. The activity-term
requirement from §1 applies here unchanged, and doctrine §2.3's other lesson applies to the
A/B: precision-weighting buys nothing on a healthy system, so measure it under damage.

---

## 4. Improving the slow loop's own predictive coding

Roughly in order of leverage:

1. **Action-conditioned transitions on every slow EPM, not just the place graph.**
   `P(node' | node, action)` is the difference between a plannable substrate and a
   smoothed recognizer — the place-coding doctrine (doctrine §7) already requires this for
   spatial maps; generalize it to any slow loop's nodes.
2. **Multi-step latent rollout.** Predict several keyframes ahead, not just the next one,
   so the slow loop can simulate candidate futures cheaply. The simulation side is
   `GNGRollout` configured against the slow graph (§3) rather than new code; what is new is
   the M-step-ahead keyframe prediction the slow EPM's own dual-TLE grades itself on. This
   is what makes the arbiter's rollout honestly prospective rather than shallow.
3. **Offline replay / consolidation.** During low-homeostatic-drive periods, resample
   buffered keyframes to merge aliased nodes and prune poorly-supported ones — the fix
   the place-coding doctrine already names for perceptual aliasing (doctrine §7, "twisty
   passages all alike"), run as a scheduled offline pass instead of only online.
4. **Recursive LateralVoter reuse.** If multiple slow EPMs eventually need fusing into one
   higher-order consensus, reuse the proven trust-weighted voting pattern rather than
   inventing a new fusion mechanism (doctrine §5's reuse discipline). This costs no new
   machinery — a level ≥ 1 voter subscribes to `consensus.0.` by configuration
   (`primitives/LateralVoter.md`) — but check `fusion_notes.md` first for whether the slow
   EPMs in question are redundant or complementary; the answer decides whether voting or
   concatenation is the right pattern to reuse.

---

## 5. Gates for each piece

Following the decomposition methodology (doctrine §3) — every part declares its gate up
front, and every gate names the degenerate behaviour that would also satisfy it. Each
mechanism ships gain-0-guarded and default-OFF (CLAUDE.md §3 rule 2). All numbers are
seed-averaged: n=4–6 fixed-seed is a signal, enough to promote or kill a direction; a
finding needs n≥20, varied world seeds, and the (d) test (CLAUDE.md §3.3).

| Mechanism | Gain-0 form | Gate |
|---|---|---|
| Event-gated sampling | Both triggers disabled ⇒ fires on the clock, byte-identical to the `KeyframeAverager` path | Slow-loop TLE variance drops against the KeyframeAverager baseline at matched sampling budget, **reported with node count, baked count and mitosis count** — a slow EPM collapsed to one node has zero TLE variance and would win on the headline number alone. Second arm: the settled trigger fires no more often on a deliberately stalled body than on a moving one |
| Precision-gated slow update | Gain 0 ⇒ constant update rate, identical to today | Measured under damage rather than on a healthy stack (doctrine §2.3): a degraded channel loses authority over the slow belief. Wrong-sign arm mandatory — it is what separates "weights in the right direction" from "any unevenness stiffens the system" |
| Three-gate spawn check | Trigger disabled ⇒ no spawns, as today | Fewer wasted spawns on an injected pure-noise channel than an entropy-only trigger, with no missed spawns on a genuinely learnable-but-complex region |
| Subgoal / multi-step rollout | Horizon 0 ⇒ the current one-step arbiter | (d)-style perturbation: relocate the goal mid-episode — a real multi-step planner re-routes through the graph, a decorative one doesn't. Second arm from doctrine §2.3: remove the subgoal's target mid-route and the agent drops it and re-explores rather than homing on a stale node |
| Replay / consolidation | Pass disabled ⇒ no offline update | Same place → same node on revisit **and** distinct places stay distinct (doctrine §7). Node count alone is blind: hard pruning reduces it while destroying the map |

---

## Open questions

- A cheap way to estimate gate 3 (instrumental value) at spawn-decision time, short of a
  full EFE rollout for every candidate region. Partial answer already in the repo:
  `GNGRollout` returns `entropy` over its K sampled trajectories, with K and M set
  per-query and `max_concurrent_queries` bounding the cost, so a low-K short-horizon probe
  against the candidate region is a cheap epistemic estimate. Still open: whether that
  entropy tracks instrumental value closely enough to gate a spawn on.
- Whether a slow-loop subgoal should override or blend with the base homeostatic C-vector,
  and by what rule — this looks like the doctrine §2.3 precision-arbitration problem
  recurring one level up.
- Whether multiple slow EPMs should race (winner-take-all) or fuse (LateralVoter-style) —
  already an open question at the fast level (doctrine §3); unclear if the answer differs
  here. The fuse arm needs no new machinery (§4 item 4); the race arm is
  `fusion_notes.md` §4.
