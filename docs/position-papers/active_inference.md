# AMI-Ogma: Active Inference Framework Specification

## 1. Overview
The Active Inference component is the "Global Workspace" of the AMI-Ogma architecture. It integrates multi-modal sensory signals into a unified latent belief state and uses temporal predictions to drive autonomous behavior (motor control) without traditional reinforcement learning.

---

## 2. Structural Consensus Fusion
EPMs transmit structural node IDs (and trajectory history) rather than raw embeddings. The Brain Server reconstructs a compositionally-structured latent space using a three-layer HDC pipeline.

### A. Per-Token Embedding (Structural Resolver)
Each incoming reality token is mapped to a stable 768-dimensional unit vector:

- **Base Vector**: Each unique `(agent_id, node_id)` pair is mapped to a stable random unit vector via SHA-256 seeding. The same pair always produces the same vector, across restarts.
- **Pathway Encoding (HDC Permutation)**: Temporal order in the trajectory history is encoded using circular shifts:

  $$Z_{token} = \hat{Z}_{current} + \sum_{i=1}^{10} \text{roll}(\hat{Z}_{history[i]},\ i) \times 0.5^i$$

  This makes `A→B` geometrically distinct from `B→A` in the latent signature.

### B. HDC Window Composition (replaces weighted average)
Once per-token embeddings are computed, the consensus embedding for the full jitter-buffer window is formed by **HDC superposition with temporal binding** rather than a weighted average.

**Key property**: Two windows sharing $k$ of $n$ EPM symbols have cosine similarity $\approx k/n$ in the resulting space. A novel window that differs by one column lands geometrically near its closest familiar neighbor, without any learned metric.

**Procedure:**
1. **Temporal Position Hypervectors**: A fixed bank of 12 anchor unit vectors (seeded deterministically at `0xDEADBEEF`) spans the sync window. A temporal position $t \in [0, 1]$ (normalised against the fixed `sync_window_ms`, not the per-window span) is resolved by linear interpolation between adjacent anchors.

2. **Binding**: Each token's per-token embedding is bound to its temporal position via element-wise multiplication (HDC binding):

   $$B_i = Z_{token_i} \odot T(t_i)$$

   where $T(t_i)$ is the interpolated temporal position hypervector for token $i$'s arrival time within the window. Tokens that fire simultaneously (tight coincidence) get nearly identical $T$ vectors; tokens 80 ms apart get geometrically distinct $T$ vectors.

3. **Superposition**: All bound contributions are summed with neurotransmitter confidence weighting and normalised:

   $$Z_{window} = \hat{\left( \sum_i w_i \cdot B_i \right)}, \quad w_i = (1 + \text{dopa}_i)(0.1 + \text{sero}_i)$$

**Why this matters**: The previous weighted average destroyed compositional geometry — two windows with completely different symbols could land arbitrarily close if their embeddings happened to cancel. The HDC superposition preserves the distance structure of the symbol space, making the meta-EPM's knowledge graph meaningful from the first session.

---

## 3. Meta-EPM (Workspace Model)

### A. Temporal Predictor
- **Architecture**: A GRU-based temporal predictor that learns the dynamics of the 768-dim fused latent space.
- **Future Projection**: The model hallucinates $N$ steps into the future (Prediction Horizon) to determine a target goal state $z_{target}$.
- **Surprise (TLE)**: Temporal Loss Error measures the MSE between the predicted and actual consensus embedding at each tick.

### B. Knowledge Graph Lookup — Hebbian-Warped Distance (Proposal 3)
The meta-EPM knowledge graph organises crystallised multi-modal states (baked concept nodes). Lookup (finding the nearest node to the incoming consensus embedding) uses a **Hebbian-warped Euclidean distance** rather than plain Euclidean:

$$d_{warped}(W, k) = \frac{\|W - C_k\|_2}{1 + \lambda \cdot A(W, k)}$$

where:
- $C_k$ is the centroid of node $k$
- $\lambda$ is `warp_strength` (default 1.0)
- $A(W, k)$ is the **Hebbian affinity** between the current window's EPM contributors and the founders of node $k$:

  $$A(W, k) = 0.5 \cdot |\text{contributors}(W) \cap \text{sig}(k)| + \sum_{\substack{(s_a, n_a) \in W \\ (s_b, n_b) \in \text{sig}(k) \\ n_a \neq n_b}} \text{assoc}[\min(n_a,n_b)][\max(n_a,n_b)]$$

  - The first term is a direct-overlap bonus (same `(source, node_id)` in both window and node signature)
  - The second term accumulates the lateral voter's Hebbian association weights between cross-modal node ID pairs

**Node signatures**: When mitosis creates a new meta-EPM node, the `(source_id, node_id)` pairs active in that founding window are stored as the node's *signature*. This records the semantic origin of each crystallised concept without requiring raw embeddings to be re-transmitted.

**Emergence**: The geometry starts from the HDC compositional space (Proposal 1). Experience, encoded in the lateral voter's Hebbian matrix, continuously warps the effective distances so that concepts the system has discovered to be related are pulled geometrically closer — without any external supervision or pre-specified semantics.

**Key properties**:
- Warp only pulls nodes closer, never pushes them away ($d_{warped} \leq d_{Euclidean}$)
- With an empty Hebbian matrix: direct overlap still provides a bonus, so well-known concepts cluster from session one
- With a rich Hebbian matrix: cross-modal concepts discovered through co-occurrence migrate toward each other over time

---

## 4. Embodied Agency (Motor Control)

### A. Blind Calibration
- **Variance Analysis**: During the passive training phase, the global workspace accumulates consensus embeddings and identifies the latent dimension with the highest variance as the primary spatial axis.
- **Zero-Knowledge**: No raw coordinates (e.g., ball/paddle positions) are used for control; the system discovers the "body" mapping purely from latent fluctuations.
- **Manual Mirroring**: A "Flip Direction" control allows correcting the identified polarity if the motor mapping is inverted.

### B. Action Decoding
- **P-Control**: Paddle velocity is proportional to the latent error along the calibrated axis between $z_{target}$ and $z_{real}$.
- **Action Stiffness**: Adjustable gain for the motor response (default 2.0 after sweep optimisation — lower stiffness reduces noise amplification from random-hash structural embeddings).
- **High-Gain Multiplier**: 200× base multiplier to translate small high-dimensional variances into assertive physical movement.
- **Dopamine Modulation**: Effective stiffness is scaled by $(1 + \text{dopamine})$, increasing responsiveness after paddle hits.

---

## 5. Export Parity Pipeline
To ensure C++ EPM behaviour matches Python training:

- At export time, `main_window.py` saves both `encoder_weights.pt` **and** `predictor_weights.pt` (the trained inner forward model, MLP or GRU).
- `sync_models.py` detects the predictor architecture from state dict key names and exports the **trained** predictor to ONNX (not a fresh random one).
- For each modality, 5 deterministic golden samples are generated from the PyTorch export wrapper **before** ONNX conversion, then re-run through the ONNX model after. Parity is verified by:
  - **Max absolute error** < 5×10⁻⁴ per element
  - **Cosine similarity** > 0.9999 between PyTorch and ONNX outputs
- Results are reported per-sample as `[PASS]` / `[FAIL]` in the export dialog.

---

## 6. Diagnostic Suite
- **Resonance Plot**: Real-time tracking of Global Resonance and Meta-EPM TLE.
- **Dynamics Viz**: Visualises the $z_{real}$ vs $z_{target}$ projection and the hallucinated future path.
- **Latent Vector History**: A scrolling 768-dim heatmap showing high-depth latent transitions.

---

## 7. Communication Protocol

### Reality Token (EPM → Brain)
| Field | Description |
|---|---|
| `agent_id` | Identifier of the sensory source |
| `current_id` | Currently active node in the EPM graph |
| `trajectory` | List of recent node IDs (pathway history) |
| `neurotransmitters` | `{dopamine: [0..1], serotonin: [0..1]}` |

### Consensus Token (Lateral Voter → Meta-EPM)
| Field | Description |
|---|---|
| `fused_embedding` | HDC window composition of all active EPM tokens (768-dim unit vector) |
| `resonance_score` | Aggregate pairwise resonance across active EPMs |
| `contributing_sources` | List of agent IDs active in this window |
| `contributing_ids` | Raw node IDs (used for Hebbian warping in meta-EPM lookup) |

### Action Hook (Brain → Game)
| Field | Description |
|---|---|
| `velocity` | Floating-point motor command for the paddle |
