# Morphodynamic Information Processing — Summary & AMI-Ogma Implications

**Source:** Juusola et al., *Theory of morphodynamic information processing: Linking sensing to behaviour*, Vision Research 227 (2025) 108537 (`docs/morphodynamic_information_processing.pdf`).

**Why we read this:** AMI-Ogma's substrate is biomimetic; the paper supplies first-principles arguments that several mechanisms we're considering should be at the *sensor* and *body-state* level rather than at the brain level. Active inference is bloody hard, and most failure modes turn out to be misplaced computation. This paper says *where* the computation should live.

---

## 1. The thesis in one paragraph

Brain function is conventionally explained by chemical and electrical processes. The paper argues that this is incomplete: **mechanical movements of neurons, sensors, and synapses** ("morphodynamics") are an irreducible third channel that *enables* perception below the static optical limits of the body. Photoreceptors in fly eyes physically twitch — these microsaccades reshape and re-aim receptive fields, and the phasic, mirror-symmetric, stochastic sampling that results is what produces hyperacute vision. The same principle extends from photoreceptors up through retinal muscles, head movements, and whole-body saccades. **Sensing requires motion; motion improves sensing.**

The paper proposes that morphodynamic processing evolved to drive predictive coding, synchronising cognitive processes across neural networks to match behavioural demands. The implication for AI is direct: static sensor arrays + static computational graphs miss the actively-sampled, bursty, mirror-symmetric, stochastic structure of biological perception.

---

## 2. Mechanisms relevant to AMI-Ogma

### 2.1 Sensing requires motion (§2, Fig 1)

A static compound eye is bounded by its ommatidial spacing — for *Drosophila*, ~4-7°. The fly resolves features 4-10× finer than this. The mechanism is not better optics, more receptors, or longer integration: it is the photoreceptor's continuous twitching, which time-multiplexes its receptive field across a smaller effective area than static physics permits.

**Implication for AMI-Ogma.** A 24×24 raycast at a fixed position is the static eye. To approach hyperacute behaviour we need:
- Per-tick stochastic jitter on ray directions (sub-pixel sampling).
- Active control of the sampling pattern by the brain (top-down direction of *where to look*).
- Burst-mode captures aligned with body motion phases, not a constant per-tick rate.

### 2.2 Local + global sampling motion (Box 1)

Two interlocking scales:
- **Local** — photoreceptor microsaccades within an ommatidium. Microsecond-to-tens-of-ms duration, contracted axially + swung sideways. Receptive field both **moves and narrows** during the saccade. The local motion direction is genetically fixed at development, mirror-symmetric across the two eyes.
- **Global** — retinal muscles, head movements, body saccades. Slower, voluntarily controlled by the brain. Globally moves *all* receptive fields together while local micro-movements continue independently.

The combined sampling matrix is what produces hyperacuity, anti-aliasing, and depth perception. Either alone is insufficient.

**Implication for AMI-Ogma.** Our current `_publish_video_frame()` does only the global pose (camera tracks the body). There is no local sampling jitter at the per-ray level. A faithful analog: each tick, perturb each ray's direction by a small Gaussian (the local microsaccade) AND let the brain command the *global* camera pose (head/eye saccade).

### 2.3 Mirror-symmetric microsaccades enable hyperacute stereo (§2.4-2.5, Fig 4-5)

Left and right eyes' photoreceptors saccade in *mirror-symmetric* directions in response to the same illumination changes. The resulting phase differences encode depth at sub-pixel resolution. Behavioural experiments (head-immobilised flies in flight simulators) confirm flies depend on this: monocular flies (one eye painted) cannot learn to distinguish 3D objects from 2D, even when the same retinal contrast is presented.

**Implication for AMI-Ogma.** We just split whisker EPMs into bilateral left/right pipelines (6.6.M). The paper says this is the right shape architecturally, and that the sampling within each side should be mirror-symmetric to exploit stereo. Vision should follow the same template: separate `epm_color_left` / `epm_color_right` pipelines with mirror-symmetric sampling jitter, fused only at the top voter (or never fused — left/right disagreement *is* depth).

### 2.4 Stochasticity combats aliasing (§2.6, Fig 6)

Photoreceptor sizes vary, pigment distributions are random, microsaccade waveforms are variable. The paper argues this is **not noise**: it is structural anti-aliasing. A regular sampling grid produces ghost rings ("ghosts") when sampling under the Nyquist rate; a random sampling grid produces broadband noise (uniformly bad) but no aliasing artefacts. Animals encode in noise, not in ghosts.

**Implication for AMI-Ogma.** We added a Gaussian-noise babbler at the actuator (`MotorFader.noise_amplitude`), gated by `1 - surprise_ema`. The 180s A/B was null. Per the paper: stochastic noise belongs at the **sensor**, not at the **actuator**. Stochastic perturbation of raycast directions is anti-aliasing; stochastic perturbation of motor outputs is just motor noise. We may have been adding noise in the wrong place.

### 2.5 Top-down brain control over sampling (Box 1B-i; §3.2, Fig 8)

The fly brain exerts global control over visual information *before the photoreceptors fire*: retinal motor neurons command the eye-muscle system based on attentional state and behavioural intent. This is not a feedback loop on the response; it is a feed-forward command on the sampling. The paper documents experiments where competing visual stimuli to the left and right eyes produce *suppressed* response on the unattended side — the brain is gating the input at the sensor.

**Implication for AMI-Ogma.** Our brain has no top-down channel into the body's sampling system. It consumes raycast frames passively. The architecturally honest closed loop is:
- Brain publishes `attention.target` (a desired gaze direction or sensor weight)
- Body's `_publish_video_frame()` consults this target to bias its raycast pattern (more rays toward target, fewer away)
- The brain's top-down signal becomes part of the active inference loop

Without this, the brain is a passive observer of body-controlled sampling. The paper says this is the wrong default.

### 2.6 Phasic, time-locked, synchronised brain response (§3.2, Fig 8)

When a fly sees a moving stimulus, the response across multiple synapses (retina → lamina → lobula plate → central brain) emerges within ~15-20 ms of stimulus onset, near-simultaneously across the whole pathway. This is not sequential domino propagation. It is "synchronised minimal-delay activity" — different layers fire in concert because morphodynamic events at one layer trigger micro-tension changes at every layer.

**Implication for AMI-Ogma.** Our pipeline is a sequential DAG: raycast → encoder → GNG → voter → consensus → predictor → fader → motor. Each module ticks once. There is no synchronisation primitive. The paper argues that biological systems achieve fast holistic perception by *not* being sequential. Whether this matters at our 60 Hz tick rate is open, but it raises the question: is our DAG architecture the right one, or should perception happen as a synchronised pulse across all modalities at once?

### 2.7 Anti-aliasing as the reason for refractoriness (§2.7-3.3, Fig 7)

After firing, photoreceptors enter a refractory period (300 ms in *Drosophila*). The paper interprets this as adaptive: refractory periods enforce stochastic resampling and prevent repetitive over-sampling of static content. Combined with the stochastic mismatch between adjacent photoreceptors, this is what aliasing-free encoding looks like.

**Implication for AMI-Ogma.** Our EPMs have no refractory period. Each tick they encode and step the GNG, regardless of whether the input has changed. This is exactly the regime that produces aliasing artefacts. Adding a per-EPM refractory (skip GNG step if input is too similar to recent input, with stochastic break-out) would mirror the bio principle.

### 2.8 Information packing in time (§3.4)

For two brains of equal size, the one using morphodynamic timing has higher information processing capacity. **Time is a dimension to encode in, not just a substrate to encode through.** The fly compresses high-resolution spatial information into temporal phase by saccading rapidly across a target.

**Implication for AMI-Ogma.** We currently use no temporal codes. EPM nodes are static centroids; voter trust shares are scalars. The paper suggests there is a free dimensionality boost available by encoding *when* a node fires within the post-stimulus window. This is closer to the spike-timing literature than what we've built.

---

## 3. Mechanisms NOT relevant (out of scope)

- The microvilli quantum-bump mechanics (Box 2). Substrate-level physics; useful as inspiration but not portable to a digital simulator at 60 Hz.
- The genetic encoding of microsaccade direction. We don't have a developmental phase.
- Anti-aliasing via stochastic refractory at the photon level. Our raycasts are deterministic colour classifications; refractory at the photon level is moot.

These are interesting but lower-priority for what AMI-Ogma can do near-term.

---

## 4. Three concrete proposals for AMI-Ogma derived from the paper

These are not endorsements — they are what the paper would suggest if applied to our substrate. Whether to ship any of them is a separate decision, made *after* the audit.

### 4.1 Sensor-side stochastic jitter (anti-aliasing primitive)

Replace the regular 24×24 raycast grid with a stochastic-perturbed grid:
- Each tick, perturb each ray's `(u, v)` angle by `~N(0, σ²)` with σ ≈ 0.5 / VIS_RES (sub-pixel).
- Bonus: vary the per-ray "size" (the cone half-angle) by a small stochastic jitter — analogous to varying photoreceptor receptive-field widths.
- This costs ~one extra `randn()` per ray per tick. ~600 floats. Negligible.

**Test:** A/B vs current static-grid raycast. Hypothesis: under high-frequency visual content (pillar edges, food at distance), stochastic-jitter raycast produces richer surprise EMA on `epm_color`.

### 4.2 Bilateral camera pipeline (mirror-symmetric stereo analog)

Split the camera into left/right halves (each 24×12 or 12×24, addressable). Each side has its own EPM. Mirror-symmetric jitter pattern: where left rays jitter rightward, right rays jitter leftward. Fuse at top voter only. Phase-difference between the two side EPMs implicitly encodes depth.

**Test:** Compare hits in pillared Cell (where depth matters) — bilateral camera vs unified.

### 4.3 Top-down sensor attention

Premotor (or a new module) publishes `attention.gaze` — a 2D scalar (azimuth, elevation, in degrees) representing where the brain wants to look. Body's `_publish_video_frame()` weights raycasts by proximity to this target (more rays near target, fewer away). Brain's gaze policy is learned by Hebbian over hits.

**Test:** A/B vs no-attention. Hypothesis: in pillared/food-occluded Cell, brain learns to gaze toward food before scent reaches; pre-scent-arrival hit rate rises.

---

## 5. Ranking against AMI-Ogma's current bottleneck

The current bottleneck (per the audit notes) is **brain policy quality**, not sensor richness. The brain doesn't yet outperform reflex on easy Cell.

The paper's proposals are *upstream* — they enrich the latent the brain consumes. They probably won't fix the policy-quality bottleneck on their own. But they might unlock the brain by:
- 4.1 (anti-aliasing): producing more discriminable consensus latents → better Hebbian credit assignment.
- 4.2 (bilateral camera): same architectural lift we already saw from bilateral whiskers, applied to vision.
- 4.3 (top-down attention): closing the loop that makes vision-driven action possible. The brain *already* has access to consensus + drive; without 4.3 it cannot *act* on perception except through motor primitives.

(4.3) is the highest-leverage of the three and is also the most architecturally novel for our substrate.

---

## 6. The bigger lesson

The paper's principle — **the body is part of the brain's computation** — is consistent with AMI-Ogma's reflex-floor result: in our experiments, *the body's reflex pipeline outperforms the disembodied brain on every test*. This is not a bug. It is the paper's thesis empirically validated in our substrate. The body's stuck-pulse + scent-gradient + whisker-aversion *is* a morphodynamic computation, and the brain's architectural job is not to replace it but to learn higher-order patterns *on top of it* (the v4_phase6_5_results.md "no env yet forces vision to be load-bearing" finding). 

The pending Phase 6.6 work should keep this in mind: the brain's value is unlocked by giving it inputs reflex cannot use (top-down attention, bilateral stereo, anti-aliased latents) and tasks reflex cannot solve (vision-occluded scent, pillared mazes), not by adding more gating mechanisms onto a brain whose perceptual surface is already richer than its policy can exploit.

---

## 7. Document scope note

This summary covers what the paper says about morphodynamic processing as an *architectural principle* and how it maps onto AMI-Ogma. The paper itself is much broader (covers reproductive behaviour, retinal genetics, evolutionary arguments, neuromorphic hardware implications). Those sections are not summarised here because they are out of scope for the current audit.
