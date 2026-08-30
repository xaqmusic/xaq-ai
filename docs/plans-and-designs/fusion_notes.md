# Fusion Notes

*Working notes on combining multiple Episodic Predictive Module (EPM) outputs — when
precision-weighted averaging is the right operation, when it destroys information, and
what to do instead. Feeds the fast-level LateralVoter today and the fast → slow interface
(see `slow_loop_design_notes.md`); relevant again wherever a future layer needs to fuse
multiple slow EPMs.*

*References written `doctrine §N` point at `brain_building_doctrine.md`. A bare `§N` is a
section of this file.*

---

## 1. Redundant vs. complementary — the operation depends on the relationship

**Precision-weighted averaging is correct when channels estimate the same hidden
variable.** This is standard Bayesian cue integration: two noisy estimates of one
underlying quantity, combined by inverse-variance weighting, genuinely reduces the
uncertainty of the combined estimate below either alone (visual-haptic depth judgments;
superior-colliculus multisensory neurons enhancing their response when cues agree
spatially; population-vector readout in motor cortex, many differently-tuned units voting
on one intended direction).

**Averaging complementary channels destroys information rather than sharpening
anything.** If two EPMs encode different aspects of the hidden state — proprioception and
audio, say — averaging their embeddings dimension-for-dimension only makes sense if
dimension *i* means the same thing in both, which generally isn't true unless the space
was explicitly trained to be shared. Cortex keeps the ventral ("what") and dorsal
("where") visual streams largely separate well downstream for exactly this reason:
blending object-identity information with spatial information into one vector would be
useless to whatever needs to query either independently.

**Practical rule: fuse redundant channels by precision-weighted averaging, concatenate
complementary ones — decided per pair, not as a global policy.** This is the resolution to
the doctrine's own open question (doctrine §3) on whether same-goal loops should race or
fuse.

Whichever side a pair falls on, a new `1/(tle+ε)` weight is not free. Doctrine §2.3 makes
every precision weight carry an activity term and puts the burden on the proposal to name
its own. The evidence is a killed picrawler leg whose forward-model residual fell to 26 %
of baseline — a limb that moves less is easier to predict — so error-weighted trust alone
would have raised its authority at the moment it became useless; `amp/(tle+ε)` gave a 12×
down-weight, produced entirely by the activity term. The fast level already has a
structural version of this defence: the LateralVoter strips dead and degenerate channels
by baked-node informativeness before weighting anything (doctrine §5). Any fusion stage
proposed here has to name the signal that goes to zero when its channel stops, and measure
it rather than assume it. Per-channel latent displacement between samples is the obvious
candidate.

---

## 2. Concatenate, then recompress through an EPM — not PCA

Growing width going up a hierarchy has real precedent (feature pyramids increasing
channel depth as spatial resolution drops; cerebellar granule-cell expansion recoding), so
widening at higher levels isn't the wrong instinct. The problem is that literal unbounded
doubling collides with the low-power constraint.

**Fix: concatenate to preserve distinctness, then run a fresh EPM (GNG + dual-TLE) on the
concatenated vector to recompress to a fixed working size.** This is the shipped Level-N
pattern rather than new machinery: a Level-1 EPM is the same code as a Level-0 EPM with
`input_topic` on `consensus.0` and an identity encoder (`primitives/EPM.md`).

The tempting argument for EPM over PCA — PCA keeps the highest-variance directions while
TLE-based clustering keeps whatever is hardest to predict — overstates the case, and
doctrine §6 records the counter-example. The GNG's insertion gate is a quantization-error
threshold, so it is variance-sensitive too, and the measured failure ran the *other* way:
the encoder and a PCA scatter showed clear differentiation while the insertion gate
collapsed it to a single node, because a small directional signal rode on a large
common-mode. Discretization is not automatically more selective than projection.

What makes recompression survivable is conditioning, and it belongs in this design rather
than in a later repair. Centre out each channel's common mode and normalize per channel
before concatenating, so that no channel's scale sets the insertion gate for all of them
(doctrine §6; CLAUDE.md §0 rule 2). With conditioning in place, "a sparse but decisive
channel survives" becomes a measurable claim instead of a premise — it is gate 2 below.
(Doctrine §3's visual loop is the analogy, not the evidence: it roughly doubles eats in
its regime while winning only ~2 % of arbiter ticks. That is a win-fraction, not a
variance, so it motivates the worry without measuring it.)

The reasons to prefer the EPM here are cheaper and hold regardless: it is the pattern
already shipped, it self-sizes through baking and mitosis, and its output carries a TLE
the next level can gate on. A PCA truncation hands up a latent with no error attached.

Bonus: this resolves the redundant/complementary distinction automatically. If two
channels really are redundant, the clustering discovers the shared structure and collapses
them into fewer effective nodes on its own — no need to hand-classify every pair. That
convenience is also why gate 1 cannot be scored on dimensionality alone: clustering
collapsing redundancy is close to what clustering *means*.

---

## 3. Permutation invariance — required wherever the channel roster can grow

Literal concatenation hard-codes one slot per channel at a fixed width. The growth
trigger (`slow_loop_design_notes.md` §2) would spawn new fast EPMs over the agent's
life, and every spawn would require resizing and retraining a fixed-slot concatenation.

**Fix: a permutation-invariant aggregator — a frozen per-channel projection followed by
pooling — instead of fixed-slot concatenation.** Two constraints, both of which the
textbook Deep Sets formulation gets wrong in this substrate.

*The per-channel encoder must be frozen.* EPM encoders are modality-shaped and not
learned, and a Level-N encoder is identity or a fixed JL rotation (`primitives/EPM.md`). A
learned φ trained by descent from the slow EPM's error would be a learned pathway with no
prediction graded by its own honest error — the anti-pattern CLAUDE.md §5 prohibition 6
names — and there is no gradient path through the substrate to train it with anyway. A
fixed JL projection per channel plus a sum is permutation-invariant, needs no training,
and is already the repo's idiom: SequenceGNG encodes by windowed concatenation followed by
a JL projection. Attention-based pooling stays out until someone can name the error that
grades it.

*The pool must be group-balanced.* The LateralVoter's contract calls modality-group
balancing "the single most load-bearing piece of the voter" — it is what stops a
high-cardinality group (several visual EPMs) drowning a low-cardinality one (a single
cochlear EPM). A flat sum or max over the whole roster reintroduces exactly that failure,
and it gets worse as the roster grows, which is the scenario this section exists to serve.
Pool within each modality group, then weight the groups equally. That remains
permutation-invariant within a group, which is where the roster actually changes.

Biological analogue: population-vector readout in motor cortex (the good case for
averaging, §1) is itself permutation-invariant — a weighted sum doesn't care how many
units are contributing, so gaining or losing neurons doesn't require rewiring the readout.
Concatenation is the one operation in this whole discussion that lacks that property, and
channel count is exactly what isn't fixed in this architecture.

Sequencing: this section pays a cost for a capability that does not exist yet. Runtime EPM
spawn is itself a proposal (`slow_loop_design_notes.md` §2), and gate 3 below cannot be run
until it lands. Build the pooling stage after the growth trigger, not before it.

---

## 4. A third paradigm: competitive selection (Global Workspace)

Both averaging and concatenation are continuous-blending operations. The third option is
competitive selection of which channel wins broadcast access at a given moment, rather
than blending all of them. It suits channels representing *mutually exclusive*
interpretations rather than redundant or complementary estimates of one thing.

Competitive selection is not new to this architecture, and that is a point in its favour.
Redundant loops already race winner-take-all at the arbiter (doctrine §3), and MotorBus
subsumption already lets a reflex override the cognitive channel outright (doctrine §5).
Both work. What is absent is competitive selection at the *perceptual* level — deciding
which EPM's token reaches the consensus at all, rather than which policy wins the motor
bus. That is the version worth a dedicated look, and its natural home is the LateralVoter's
existing informativeness gate promoted from a filter into a selector.

---

## Where this applies

- The LateralVoter's fusion of fast EPM outputs, today.
- The fast → slow interface (`slow_loop_design_notes.md` §1) — resolved there as
  subscription to `consensus.0`, whose `ConsensusToken` already carries per-channel
  `trust_weights` alongside the fused embedding, with group-balanced pooling and
  recompression held back until the roster starts growing.
- Any future fusion of multiple slow EPMs into a higher-order consensus
  (`slow_loop_design_notes.md` §4 item 4, and its Open questions) — the same
  redundant-vs-complementary question one level up, unresolved. The fuse arm costs no new
  machinery: a level ≥ 1 LateralVoter subscribes to `consensus.0.` by configuration
  (`primitives/LateralVoter.md`). The race arm is §4 above.

---

## Gates

Each mechanism ships gain-0-guarded and default-OFF (CLAUDE.md §3 rule 2), and every
number is seed-averaged. n=4–6 fixed-seed is a signal, enough to promote or kill; a
finding needs n≥20 with varied world seeds (CLAUDE.md §3.3).

| Mechanism | Gain-0 form | Gate |
|---|---|---|
| Redundant vs. complementary fusion rule | One channel in the pool ⇒ output identical to that channel's token | Leave-one-out measured on the **goal**, not on the representation (doctrine §3): dropping one of a known-redundant pair costs little behaviourally, dropping one of a known-complementary pair costs a lot. Effective dimensionality under recompression is reported alongside as the mechanism check — on its own it is near-tautological |
| EPM recompression vs. PCA | Recompression EPM absent ⇒ the concatenated vector passes through unchanged | A channel active on a few percent of ticks still moves the recompressed latent once conditioning is in place, scored by that channel's leave-one-out effect on behaviour. Held-out TLE is reported with node count and baked count, never alone: a representation collapsed to one node scores a perfect TLE |
| Frozen, group-balanced pooling | A single modality group ⇒ identical to today's LateralVoter path | Under a deliberately unbalanced roster (n visual EPMs, 1 cochlear), the cochlear channel's leave-one-out effect holds as n grows 1 → 4. A flat sum fails this by construction, which makes it the wrong-sign arm |
| Permutation invariance | — (blocked: needs runtime spawn) | Adding a channel mid-run leaves the pooling stage untouched, and the pre-existing channels' trust weights move by less than their own run-to-run spread. Not runnable until the growth trigger lands |
