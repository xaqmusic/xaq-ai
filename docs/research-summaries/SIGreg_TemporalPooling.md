# ARCHITECTURE SPECIFICATION: EPM Latent Space Enhancements
**Target Subsystem:** Episodic Predictive Memory (EPM) Module
**Objective:** Upgrade the EPM latent processing pipeline to prevent anisotropic representation collapse (via SIGReg) and achieve temporal shift invariance for transient events (via Temporal TLE Pooling).
**Prerequisites:** Frozen audio encoders (STFT filterbank) are implemented and actively broadcasting latent streams.

> **Status: speculative design proposal — not yet built.** This document is a forward-looking design sketch informed by external work (LeJEPA; Balestriero & LeCun, 2025), not a description of shipped xaq behaviour. None of it is implemented in the current substrate.

---

## 1. Sketched Isotropic Gaussian Regularization (Rolling SIGReg)
**Reference:** LeJEPA (Balestriero & LeCun, 2025).
**Goal:** Mathematically force the EPM's continuous latent embedding space into an isotropic Gaussian distribution. This prevents "pancaking" (anisotropic collapse) where continuous background noise geometrically dominates rare transient events.



### 1.1 Data Structure
* Implement a continuous First-In, First-Out (FIFO) buffer $B$ within the EPM to store a rolling temporal window of the incoming latent embeddings $S_t$.

### 1.2 Mathematical Implementation (Linear Complexity $O(D)$)
To avoid the $O(D^2)$ computational explosion of standard covariance-matrix regularization, implement SIGReg using 1D random projections.

1.  **Random Slicing:** At regular training intervals, generate a set of random projection vectors $v$, sampled uniformly from the unit sphere: $v \sim \mathcal{S}^{d-1}$.
2.  **Projection:** Project the latent buffer $B$ onto these 1D slices to obtain scalar distributions: 
    $$S_{proj} = v^T B$$
3.  **Distribution Matching Loss:** Apply a regularization loss (e.g., 1D Wasserstein distance or standard Mean Squared Error on the sorted distributions) that penalizes the deviation of $S_{proj}$ from a standard 1D normal distribution $\mathcal{N}(0,1)$.
4.  **Gradient Update:** Backpropagate this regularization loss through the EPM's localized projection head to continuously "inflate" the latent space into a perfect sphere.

> *Caveat: this step assumes backpropagation. xaq's current C++ substrate is not backprop-based, so gradient-based regularization as written here is aspirational — it is not how the shipped substrate learns.*

---

## 2. Temporal Pooling of the Time-Loop Error (TLE)
**Reference:** xaq Homeokinetic Predictor / The Playful Machine (Der & Martius).
**Goal:** Achieve "Temporal Shift Invariance." The clustering mechanism (Concept Crystallization) must cluster the *prediction error delta* rather than the raw latent state $S_t$. This isolates transient transient events (e.g., a bark) regardless of their temporal placement within the rolling window.



### 2.1 Error Delta Calculation
Inside the EPM's Predictor block, calculate the non-autoregressive prediction errors for each time step in the rolling window $W$:

* **Forward Prediction (Time-Loop Error):**
    $$L_{TLE}(t) = \|P(S_{t}) - S_{t+1}\|^{2}$$
    *(Where $P$ is the localized EPM forward predictor).*
* **Backward Prediction (Consistency Gate):**
    $$L_{consistency}(t) = \|B(S_{t}) - S_{t-1}\|^{2}$$
    *(Where $B$ is the EPM backward predictor).*

### 2.2 Temporal Max-Pooling Logic
To process the window for the Mitosis Gatekeeper and Concept Crystallization:

1.  **Isolate the Transient:** Apply Temporal Max-Pooling across the array of calculated errors in the current sliding window $W$.
    $$E_{spike} = \max_{t \in W} (L_{TLE}(t))$$
2.  **Alignment:** Instead of feeding the absolute latent array to the clustering algorithm, center the window's data around the temporal index of $E_{spike}$. 
3.  **Clustering Input:** Pass the isolated $E_{spike}$ signature (the delta) and its surrounding context to the Concept Crystallization layer. This collapses temporally shifted instances of the same sound into a single, dense SuperNode.

---

## 3. Mitosis Gatekeeper Integration (Updated)
The Global Workspace evaluates the output of the Temporal Pooling layer to determine neurogenesis.

**Spawning Trigger Conditions (Logical AND):**
1.  **Predictive Resonance:** The pooled error baseline drops below the configured stability threshold (`pooled_TLE < TLE_STABILITY_THRESHOLD`).
2.  **Structured Complexity:** The Algorithmic Complexity (High-Order Entropy) of the aligned $E_{spike}$ sequence exceeds the randomness threshold (`high_order_entropy > ASSEMBLY_THRESHOLD`).

*Action:* If both conditions are met, freeze the current dynamics and spawn a new EPM column assigned to the isolated transient signature.

---

## 4. Ablation Study Toggles
To facilitate future ablation studies, the following features should be implemented with functional toggles (e.g., in the configuration file) to allow them to be independently enabled or disabled:
- [ ] Enable SIGReg (Sketched Isotropic Gaussian Regularization)
- [ ] Enable Temporal Pooling of the Time-Loop Error