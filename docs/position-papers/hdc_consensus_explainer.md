# HDC Consensus — First-Principles Explainer + Presentation Aid

**Date:** 2026-05-15
**Author:** Joseph Butera III
**Status:** Reference / presentation prep
**Companion:** `docs/active_inference.md` (the production spec), `docs/v4_position.md` (the broader signal-flow argument), `docs/ogma_in_context.md` (layman framing + landscape position vs. pretrained encoders / VLAs / world-model thesis), `godot_host/project/scenes/hdc_demo_view_a.tscn` (interactive demo)

This is the canonical explanation of the HDC superposition + temporal binding layer used in the Ami-Ogma consensus pipeline. It is written for two audiences: future-Joseph who needs to defend the design without re-deriving it, and an outside reader (e.g. OpenAI-class technical audience) who needs to grok the construction in 10 minutes. Geometric intuition only; no probability theory or matrix calculus.

---

## 1. The one fact the whole construction rests on

**In high-dimensional space, two random unit vectors are almost certainly pointing in different directions.**

The dot product of two random unit vectors has expected value zero, and its standard deviation is approximately `1/√d` where d is the dimension. Concretely:

| dimension | typical dot product spread | "coincidental cos > 0.2" |
|---|---|---|
| 2 | ±0.707 | extremely common |
| 16 | ±0.25 | common |
| 64 | ±0.125 | uncommon |
| 128 | ±0.088 | rare but possible |
| 768 | ±0.036 | astronomically rare |

In 768D, if you generate 100,000 random unit vectors, essentially all pairs are "nearly orthogonal" to all others. You have effectively unlimited room for distinct symbols. This is the only piece of math the rest of the construction needs.

The interactive demo (View A, §8) is precisely a visualisation of this fact. Drag the dimension slider from 2 to 768 and watch the histogram of pairwise dot products collapse to a needle at zero.

---

## 2. Three operations on these vectors

HDC is built on three operations. Each has a specific role and they compose without interfering.

### 2.1 Binding — element-wise multiplication (`A ⊙ B`)

Take two vectors A and B of equal length and multiply element-by-element. The result C = A ⊙ B has three properties:

1. Same dimension as inputs.
2. Dissimilar to both A and B (dot product ≈ 0).
3. Reversible: C ⊙ B ≈ A (exactly so for bipolar ±1 vectors; approximate but reliable for Gaussian-derived unit vectors at high d).

**Geometric reasoning:** when you compute `C·A = Σ aᵢ² bᵢ`, the `aᵢ²` are all positive but `bᵢ` is randomly signed, so the sum is a random walk centred at zero. C is therefore dissimilar to A. Same logic for C·B.

**Role:** pair two things into a new symbol without losing the ability to recover either piece. Used to bind "filler" (a concept) to "role" (its slot in a structure).

### 2.2 Superposition — addition (`A + B + C`, then normalise)

Sum two or more vectors and renormalise to unit length. The sum M is similar to *every* component vector simultaneously.

**Geometric reasoning:** `M·A = A·A + B·A + C·A`. The first term is large (≈ 1 for unit vectors), the others are random pairs at near-zero. A's signature survives the sum because B's and C's contributions are statistically near zero in A's direction.

**Role:** pile multiple bound pairs into one fixed-dimension vector while keeping every component recoverable. This is how a single vector can carry an unordered *set* of symbols.

### 2.3 Permutation — circular shift (`roll(A, k)`)

Shift the elements of A by k positions, wrapping around. The result is nearly orthogonal to A for any nonzero k (it's the autocorrelation of a random sequence at non-zero lag) and is reversible by rolling by −k.

**Role:** create a position-tagged version of a symbol. "A at slot 1" and "A at slot 2" become distinguishable, recoverable vectors. Used for ordinal positions like trajectory step indices.

---

## 3. Why we need two different binding operations

Tokens in Ogma's consensus layer have two distinct kinds of "position" that must be encoded without interfering with each other:

| kind of position | operation used | why |
|---|---|---|
| step index in a trajectory history (0, 1, 2, …) | circular shift / permutation | discrete ordinal positions; shifts are natural for integer indices |
| arrival time within the sync window (continuous t ∈ [0, 1]) | multiplicative binding to interpolated time-anchor vectors | continuous quantity; interpolation gives smooth distance falloff between near-simultaneous and far-apart firings |

These two binding modes are themselves nearly orthogonal in the dimension space — a token's trajectory-history-shift and its window-time-binding can both be applied without obscuring each other.

---

## 4. The Ogma construction, step by step

Every token published by an EPM goes through these four steps to contribute to the sync-window consensus vector.

### Step A — Resolve the symbol

Each `(agent_id, node_id)` pair is hashed with SHA-256 and the digest is used as the seed for a deterministic random unit vector in 128D (production) or 768D (the Python prototype — see §7). Same input always yields the same vector across processes, restarts, and machines.

This is the symbol table of the system: every concept the system can name has a fixed direction drawn once from the warehouse of almost-orthogonal vectors.

### Step B — Encode the trajectory history

The EPM doesn't just say "node 47 is firing now." It says "node 47 is firing now, having just visited 12, 89, 31, …" The recent path through its own concept graph is folded into the token vector by superposing rolled versions of each historical step's symbol, with exponentially-decaying weight:

```
Z_token = Z_current + Σ_{i=1..10} roll(Z_history[i], i) × 0.5^i
```

Most recent step (i=1) rolled by 1, weighted by 0.5. Two steps back, rolled by 2, weighted by 0.25. And so on.

**Why this matters:** if the trajectory is `A → B`, the token is `Z_B + roll(Z_A, 1)·0.5`. If it's `B → A`, the token is `Z_A + roll(Z_B, 1)·0.5`. These vectors are distinguishable. Without the shift, both trajectories would collapse to the same `Z_A + Z_B`. **Order is preserved geometrically.**

### Step C — Bind the token to its time within the sync window

Each token has an arrival time within the sync window, normalised to t ∈ [0, 1]. A fixed bank of 12 anchor unit vectors (seeded deterministically with `0xDEADBEEF`) spans the window. For a given t, we linearly interpolate between the two nearest anchors to get T(t).

Then we bind the token's symbol vector to its time anchor:

```
B_i = Z_token_i ⊙ T(t_i)
```

Two tokens at the same time get identical T vectors and identical bindings. Tokens 80 ms apart get different T vectors and different bindings. Near-simultaneity decays smoothly with the time gap — there's no hard cutoff. **Coincidence is preserved geometrically.**

### Step D — Superpose with neurotransmitter confidence weighting

Each EPM publishes per-token confidence scalars: dopamine (authority) and serotonin (stability). They combine into one weight:

```
w_i = (1 + dopa_i) × (0.1 + sero_i)
```

All bound tokens are summed with their weights and the result is normalised to a unit vector:

```
Z_window = normalize( Σ_i w_i · B_i )
```

That's the consensus vector for the sync window. One unit vector that encodes *which* symbols fired, *when* they fired within the window, *what trajectory each came from*, and *how confident each was*.

---

## 5. The emergent property

**Two windows that share k of n symbol-and-time pairs have consensus cosine similarity ≈ k/n.**

This is guaranteed by construction, with no training. Shared `(symbol, time)` pairs contribute identical bound vectors to both sums. Different pairs contribute near-orthogonal vectors that don't pull the two sums toward each other. The dot product of the two normalised sums tracks the fraction of shared content.

This is the property the previous implementation (a learned weighted average of EPM embeddings) could not give us. With weighted averaging, two windows containing completely different symbols could land near each other in latent space if their embeddings happened to cancel arithmetically. After the HDC rewrite, the geometric link between symbol overlap and vector overlap is mathematically forced.

The downstream meta-EPM's knowledge graph asks "which existing concept does this window look most like?" For that question to give a useful answer from the first session, the consensus space must already be compositional. HDC gives compositional geometry on tick one; Hebbian-warped distance (see `docs/active_inference.md` §3.B) then warps this base geometry with experience.

**Slogan:** HDC gives you compositional geometry for free; Hebbian warping gives you experience-shaped geometry on top of it.

---

## 6. Why the C++ core uses 128D, not 768D

The production substrate (v4/v5, `cpp_core/`) uses `projection_dim = 128` across encoders, EPM latents, fused embeddings, and GNG inputs. The 768D figure that appears in `docs/active_inference.md` is from an earlier Python prototype and should be read as historical.

### What you lose at 128D vs 768D

The near-orthogonality guarantee gets noisier — typical dot-product spread goes from ±0.036 to ±0.088. Coincidental similarities above 0.2 become rare-but-possible instead of astronomically rare. Capacity for superposed bound pairs drops from "many hundreds before interference dominates" to "many tens." The `cos ≈ k/n` property still holds at 128D but with a noticeable noise floor of about ±0.1 per measurement.

### What you gain at 128D

1. **6× cheaper memory.** A vector is 512 bytes instead of 3 KB. Per-tick state across all EPMs, jitter buffer, and meta-EPM graph nodes adds up.
2. **6× cheaper compute.** Dot product, binding, summing all scale linearly with dimension. The Lateral Voter runs the consensus operation every sync window — it is a hot path.
3. **SIMD alignment.** 128 floats fit cleanly into ARM NEON (Pi5) and AVX-512 SIMD pipelines.
4. **GNG node-count budget.** Each concept-graph node stores a centroid; smaller centroids mean more nodes per modality fit in the per-tick lookup budget.
5. **ONNX export size.** Encoder/predictor weights export at the projection dim. Smaller exports ship more easily to embedded targets.

### The design bet behind the choice

A sync window typically carries on the order of 10 token firings, not 100. The capacity envelope at 128D is well above our actual load, and the noise floor on `cos ≈ k/n` is absorbed downstream by the Hebbian-warped distance (which adds an experience-driven correction term anyway). For the 5–9 modality count the current substrate runs, 128 is a comfortable fit. If the sync window scales to dozens of concurrent modalities in a future revision, the dimension should be revisited.

### Presentation framing for this point

"We run at 128 dimensions. Orthogonality is statistical — at 128D you get a typical cross-similarity noise floor of about ±0.09. We sized the dimension for the actual load and the Pi5 deployment target. Going larger buys margin we don't need yet and costs us 6× on the hottest path."

---

## 7. Interactive demo tool (Godot)

A standalone Godot 4 scene at `godot_host/project/scenes/hdc_demo_view_a.tscn` visualises the foundational claim of §1. Pure GDScript, no dependency on the rest of the Ogma substrate, runs in the editor or via the CLI.

### How to run

In the editor: open `godot_host/project`, open the scene, press F6.

From the CLI:
```bash
godot4 --path godot_host/project res://scenes/hdc_demo_view_a.tscn
```

### What View A shows

- **Dimension slider** snapped to `{2, 4, 8, 16, 32, 64, 128, 256, 512, 768, 1024}`.
- **Samples slider** controls how many random unit-vector pairs are drawn (100–5000).
- **Regenerate** re-seeds the RNG so the audience sees the histogram shape is stable, not cherry-picked.
- **Histogram** of pairwise dot products with 80 bins on the range [−1, +1].
- **Shaded band** marks the theoretical ±1/√d region. The histogram always lives inside it.
- **Stats line** shows observed mean / std / max-|dot| versus the theoretical std.

### Recommended demo sequence (≈ 90 seconds)

1. Start at **d = 2**. Histogram is wide and fat. Max |dot| routinely above 0.7. *"In low dimensions, two random arrows often point the same way."*
2. Step to **d = 16**. Histogram concentrates visibly. Std ≈ 0.25. *"It improves with dimension."*
3. Step to **d = 128** (production). Std ≈ 0.088. Tight spike with thin tails. *"This is where we live. We get enough orthogonality margin to do useful work."*
4. Step to **d = 768**. Std ≈ 0.036. Histogram is a needle. *"The Python prototype ran here. Almost-perfect orthogonality, but six times the per-vector cost."*
5. Click **Regenerate** a few times. *"The shape is stable across draws. This is a property of the geometry, not the random seed."*

The shaded ±1/√d band shrinking as you drag the dimension slider is the cleanest single visual on the underlying math. *That* is the slide.

### View D — the k/n property (ready)

`godot_host/project/scenes/hdc_demo_view_d.tscn`

This is the headline demo. Two sync windows side by side; Window 2 shares k of n (symbol, time) pairs with Window 1. Drag the **k slider** from 0 to n and watch the cosine similarity track k/n live. The scatter panel on the right accumulates every (k/n, cos) point so the diagonal *emerges* as you drag.

Controls:
- **n** — tokens per window (2 to 20)
- **k** — shared pairs (0 to n)
- **d** — dimension (16 to 768); clears the scatter on change so the noise band shrink is visible
- **time jitter** — perturbs the t coordinate of shared pairs in Window 2; show that near-simultaneity decays smoothly instead of cliff-edging
- **Regenerate tokens** — new random seed, keeps scatter (each regen adds one more point at the current k/n, revealing the noise distribution)
- **Clear scatter** — explicit reset

Recommended demo sequence (≈ 90 seconds):
1. **Start at d = 128, n = 8, k = 4**. Cosine ≈ 0.5. Single point on the scatter.
2. **Slide k from 0 to 8**. Diagonal fills in live. Cosine reads ≈ k/n at every stop. *This is the punchline.*
3. **Regenerate 5–10 times** at a fixed k. Watch a vertical cloud of points appear at k/n — that's the natural noise band at d = 128.
4. **Drop d to 16**. Scatter clears. Slide k again — cloud is visibly fluffier, noise band fattens to ±0.25.
5. **Bump d to 768**. Scatter clears. Slide k — cloud collapses onto the diagonal. ±1/√d ≈ ±0.036.
6. **Push jitter up to 0.2**. Cosine drops below k/n smoothly — the *time component* of binding matters and degrades continuously, no hard cutoff.

The combination of A (foundational orthogonality fact) and D (emergent k/n property) is the full mathematical pitch for the consensus layer in two demos.

### View B — the three operations (ready)

`godot_host/project/scenes/hdc_demo_view_b.tscn`

Three stacked panels demonstrating each HDC primitive on 128-D unit vectors, with strips visualised as heatmaps (blue negative, orange positive, auto-scaled per strip).

1. **Binding (⊙)** — four strips: A, B, A⊙B, (A⊙B)⊙B. Readouts show `dot(A, A⊙B) ≈ 0`, `dot(B, A⊙B) ≈ 0`, `dot(A, (A⊙B)⊙B) ≈ 1`. Regenerate button reshuffles A, B.
2. **Superposition (+)** — K input strips and the normalised sum M. Slider for K (2 to 6). Readouts show `dot(M, Aᵢ)` for every input (all positive) and `dot(M, Q)` for an unrelated random Q (near zero).
3. **Permutation (roll)** — three strips: A, roll(A, k), recovered. Slider for k. Readouts show that the rolled vector is dissimilar to A and rolling by `-k` recovers it.

### View C — building a consensus vector (ready)

`godot_host/project/scenes/hdc_demo_view_c.tscn`

Step-by-step walk through the construction in §4 of this doc. Four tokens at d = 64 (for visual clarity). Each step adds one token's `Z_sym ⊙ T(t)` factor to the running sum; the final step normalises. Reset / Step / ▶ Play controls. The audience can see exactly what each row of the formula does.

### View E — the Hebbian-warped metric (ready)

`godot_host/project/scenes/hdc_demo_view_e.tscn`

Visual demonstration of the experience-warped distance from §3.B of `docs/active_inference.md` and §4 of `docs/ogma_in_context.md`. Six knowledge-graph nodes sit at fixed positions on a ring around the query W. Each node has a Hebbian affinity that the user can grow by clicking. Real positions never move — but each node's **ghost ring** (its effective position under the warp `lerp(real, W, λA/(1+λA))`) drifts toward W as affinity accumulates. The nearest node by warped distance is highlighted gold.

Controls:
- Click any node → +0.5 affinity (animated)
- λ slider (warp strength, 0 to 5)
- ↺ Reset all affinities
- Scenario button — pre-built narrative that pumps node 3 to show nearest-node hand-off
- Per-node stats line: Euclidean, affinity, warped distance, nearest marker

Recommended demo sequence (≈ 60 seconds):
1. Open at default state. All affinities are zero. Whichever node has the smallest Euclidean distance is nearest.
2. Click node 3 four times. Watch its ghost ring surge toward W and become the new nearest, even though no node moved.
3. Drag λ up to 5.0. Smaller affinity now drags the ghost in further — the warp is more aggressive.
4. Drag λ down to 0. All ghosts snap back onto their real positions — no warp.
5. Reset, then click "Scenario: pump node 3". Same effect in one click — useful for a demo.

The narrative line: *Memory is not a database. It is the shape of the space the system looks things up in.*

### Slideshow navigation

All five views are wired into a slideshow chain (`A → B → C → D → E`) via Prev/Next buttons at the bottom of each scene. Open any view and navigate without returning to the editor.

---

## 8. How to present this in the OpenAI talk

If you have three minutes for this section, hit three beats:

1. **The dimension claim** (slide + live View A): high-d unit vectors are nearly orthogonal. *"This is the only piece of math the rest of the layer needs."*
2. **The three operations** (one slide): binding (⊙), superposition (+), permutation (roll). One sentence each.
3. **The emergent property** (slide + planned View D, or a static plot if D isn't built): `cos(W₁, W₂) ≈ k/n` without learning.

If you have six minutes, add:

4. **The full construction** (formula slide): walk through the four-step pipeline of §4. One pass, slow.
5. **The 128D production setting** (§6): brief mention of the tradeoff for credibility.

The pitch line for the slide deck:

> "We replaced a learned, brittle weighted average with a deterministic, compositional construction. The consensus layer does not need to be trained. The geometry it gives the meta-EPM is correct from tick one — and the math says it will stay correct as we add modalities."

---

## 9. Anticipated questions, brief defensible answers

- **"Why not learn the consensus mapping with an autoencoder?"** — You could. It requires training data the system doesn't have at startup, and the resulting space has no guarantees of compositional structure. HDC gives compositional geometry by construction. We use learned representations *inside* the EPMs; the consensus layer is intentionally architectural.

- **"How is this different from attention?"** — Attention learns the mixing weights with backprop and is O(N²) in window length. HDC binding and superposition are deterministic and O(N). Attention asks *what to mix*; we already know what to mix and only need *how to combine without losing structure*.

- **"What's the capacity limit?"** — Capacity grows with dimension. At 128D you can superpose on the order of tens of bound pairs before interference dominates. A typical sync window has ~10 firings. We are well below ceiling.

- **"Why 128 specifically?"** — See §6. Sized for actual load (≈10 firings/window) and the Pi5 SIMD pipeline. Going larger costs 6× on a hot path for margin we don't need.

- **"Why multiplicative binding, not circular convolution (HRR)?"** — Multiplicative binding is cheaper to compute and sufficient at this scale. HRR has slightly nicer algebraic properties but costs more per operation. For an embedded substrate, the cost matters.

- **"Is this just a hash function?"** — The symbol vectors are SHA-seeded — that part is a hash. But the operations on them — binding, superposition, permutation — are what give you the compositional structure. A hash on its own gives you distinct symbols; the algebra on top is what gives you meaning.

- **"What if two random vectors happen to be similar?"** — At 128D the probability of two random unit vectors having cosine above 0.2 is rare but not zero. The Hebbian-warped distance downstream absorbs this noise via the experience-driven warp term, so a single noisy collision does not propagate.

- **"This sounds embedded-first."** — Correct. Multiplicative binding, sum, normalise, dot product are all SIMD-friendly operations that map cleanly to a Pi5 or a neuromorphic chip. Compositional geometry on a tiny compute budget is precisely what an embodied agent needs.

---

## 10. References

- `docs/active_inference.md` — production spec, including §2 (Structural Consensus Fusion), §3.B (Hebbian-warped distance), §7 (token contracts).
- `docs/v4_position.md` — the broader signal-flow argument the consensus layer sits inside.
- `cpp_core/include/ogma/Topics.hpp:134` — `fused_embedding` declared as `Eigen::VectorXf // typically 128-D`.
- `cpp_core/include/ogma/modules/EPM.hpp:94`, `cpp_core/include/ogma/modules/SequenceGNG.hpp:76` — `projection_dim_ = 128`.
- `godot_host/project/scenes/hdc_demo_view_a.tscn`, `scripts/hdc_demo_view_a.gd` — the interactive demo.
- Plate, T. *Holographic Reduced Representation: Distributed Representation for Cognitive Structures.* CSLI, 2003. (HRR, the convolution-binding variant.)
- Kanerva, P. *Hyperdimensional Computing: An Introduction to Computing in Distributed Representation with High-Dimensional Random Vectors.* Cognitive Computation, 1(2):139–159, 2009. (The foundational HDC paper.)
- Rachkovskij, D. A. and Kussul, E. M. *Binding and Normalization of Binary Sparse Distributed Representations by Context-Dependent Thinning.* Neural Computation, 13(2):411–452, 2001. (MAP-B-style binding.)
