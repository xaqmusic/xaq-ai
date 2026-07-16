import torch
import torch.nn as nn
import numpy as np
import time
import zlib
from collections import deque

from xaq_core.predictor_torch import HomeokineticPredictor
from xaq_core.memory import KnowledgeGraph

class MetaEPM:
    """
    A Global Workspace Meta-EPM that processes multi-modal consensus tokens.
    It predicts the trajectory of the unified world state using a 
    temporal Homeokinetic Predictor.
    """
    def __init__(self, embedding_dim=768, hidden_dim=768, device='cpu'):
        self.device = torch.device(device)
        self.embedding_dim = embedding_dim
        
        # 1. The Workspace Predictor (Temporal GRU)
        self.predictor = HomeokineticPredictor(
            embedding_dim=embedding_dim,
            model_type="gru",
            learning_rate=0.05 # Boosted from 0.01 to adapt to new topology faster
        ).to(self.device)
        
        # 2. The Global Memory (Crystallized multi-modal states)
        self.memory = KnowledgeGraph(
            embedding_dim=embedding_dim,
            max_nodes=5000,
            device=self.device
        )
        
        # State tracking
        self.prev_embedding = None
        self.prev_hidden_state = None
        self.current_hidden_state = None
        self.last_tle = 0.0
        self.dopamine = 0.0
        self.serotonin = 1.0
        
        # Buffers for regularization and mitosis
        self.latent_buffer = deque(maxlen=20) # Increased for entropy calc
        self.resonance_buffer = deque(maxlen=5) # Window for stability check

        # Crystallization Hyperparameters
        self.novelty_threshold = 0.5
        self.stability_threshold = 0.05  # Baseline TLE for "Resonance" (was 0.005 — too tight,
                                         # Meta-EPM never reached resonance and formed 0 nodes)
        self.complexity_threshold = 0.15 # Minimum Normalized LZW compression ratio
        self.hebbian_lr = 0.1

        # Complexity gate: only recalculate every N ticks
        self._complexity_tick = 0
        self._complexity_interval = 5
        self._last_complexity = 0.0
        
    def process_consensus(self, fused_embedding: np.ndarray, learning_enabled=True,
                          contributors=None, association_matrix=None):
        """
        Step the Meta-EPM with a new consensus vector.

        contributors:       optional list of (source_id, node_id) tuples from the
                            current lateral-voter window (Proposal 3)
        association_matrix: optional ref to voter's Hebbian association_matrix
                            used to warp knowledge-graph lookup distances
        """
        # Convert to torch tensor
        current_z = torch.from_numpy(fused_embedding).float().to(self.device)
        if current_z.dim() == 1:
            current_z = current_z.unsqueeze(0) # (1, Dim)

        # A. Memory State Tracking — use Hebbian-warped lookup when available
        if contributors and association_matrix is not None:
            node_id, dist = self.memory.find_nearest_concept_warped(
                current_z, contributors, association_matrix
            )
        else:
            node_id, dist = self.memory.find_nearest_concept(current_z)
        
        # B. Prediction & State Update
        if self.prev_embedding is None:
            self.prev_embedding = current_z
            return None, 0.0

        # Predict z_t from z_{t-1}
        predicted_current, next_hidden = self.predictor.model(self.prev_embedding, self.prev_hidden_state)
        
        # Calculate TLE
        tle = torch.nn.functional.mse_loss(predicted_current, current_z).item()
        self.last_tle = tle
        
        # C. Learning (Homeokinesis)
        if learning_enabled:
            # Predictor learns S -> S' mapping
            # NOTE: Dopamine no longer spikes the GRU learning rate (removed to maintain physics stability)
            
            self.predictor.learn(
                self.prev_embedding, 
                current_z, 
                prev_hidden_state=self.prev_hidden_state,
                latent_buffer=list(self.latent_buffer) if len(self.latent_buffer) > 1 else None,
                sigreg_enabled=True
            )
            
        # Update hidden state for next step
        self.prev_hidden_state = next_hidden.detach() if next_hidden is not None else None
        self.prev_embedding = current_z.detach()
        self.latent_buffer.append(current_z.detach())
        self.resonance_buffer.append(tle)
        
        # D. Mitosis Gatekeeper (Crystallization & Hebbian Refinement)
        if learning_enabled:
            now = time.time()
            
            # 1. Evaluate Resonance (Stable Low TLE)
            is_resonant = len(self.resonance_buffer) == self.resonance_buffer.maxlen and \
                          all(t < self.stability_threshold for t in self.resonance_buffer)
            
            # 2. Evaluate Complexity (High-Order Entropy) — gated to every N ticks
            self._complexity_tick += 1
            if self._complexity_tick >= self._complexity_interval:
                self._complexity_tick = 0
                self._last_complexity = self._calculate_complexity()
            complexity = self._last_complexity
            is_complex = complexity > self.complexity_threshold
            
            # 3. Trigger Mitosis?
            # Must be: NOVEL + RESONANT + COMPLEX
            # Bootstrap exception: bypass complexity gate for the first few nodes.
            # With an empty memory, the complexity estimator always returns 0
            # (all z map to node -1 → constant sequence → compresses perfectly),
            # which creates a deadlock where the first node can never form.
            bootstrapping = self.memory.node_count < 3
            should_spawn = (node_id is None or dist > self.novelty_threshold) and \
                           is_resonant and (is_complex or bootstrapping)

            if should_spawn:
                new_idx = self.memory.add_concept(current_z, now=now, signatures=contributors)
                self.memory.update_transition(new_idx, now=now)
                # print(f"MetaEPM: Mitosis! Created node {new_idx} (Complexity: {complexity:.2f}, TLE: {self.last_tle:.5f})")
            else:
                # Refine existing node position (Hebbian learning)
                if node_id is not None:
                    self.memory.update_centroid(node_id, current_z, lr=self.hebbian_lr, now=now)
                    self.memory.update_transition(node_id, now=now)
            
            # Update serotonin based on TLE (Confidence proxy)
            # Stability threshold is the "nominal" good TLE.
            self.serotonin = max(0.0, 1.0 - (self.last_tle / (self.stability_threshold * 10)))
        
        return predicted_current.detach().cpu().numpy(), tle

    def _calculate_complexity(self):
        """
        Estimates the Kolmogorov complexity proxy of the latent trajectory.
        Uses a Vector-Quantized Zlib compression ratio.
        """
        if len(self.latent_buffer) < self.latent_buffer.maxlen // 2:
            return 0.0
            
        # 1. Quantize the buffer into node IDs
        sequence = []
        for z in self.latent_buffer:
            nid, _ = self.memory.find_nearest_concept(z)
            sequence.append(str(nid if nid is not None else -1))
        
        # 2. Measure compression ratio (Lempel-Ziv proxy)
        data = ",".join(sequence).encode('utf-8')
        if not data: return 0.0
        
        compressed = zlib.compress(data)
        # Normalized ratio: 1.0 = High Entropy (Novel/Random), low = Redundant
        # We actually want 'Structured' but non-random. 
        # For our purposes, a ratio > 0.3 means we aren't just stuck on one node.
        ratio = len(compressed) / len(data)
        return ratio

    def predict_future(self, horizon=1):
        """
        Hallucinate future global states.
        """
        if self.prev_embedding is None:
            return None
            
        future_path = []
        curr = self.prev_embedding
        h = self.prev_hidden_state
        
        with torch.no_grad():
            for _ in range(horizon):
                pred, h = self.predictor.model(curr, h)
                future_path.append(pred.cpu().numpy())
                curr = pred
                
        return future_path

    def set_learning_rate(self, lr):
        """Update the base learning rate of the underlying predictor."""
        self.predictor.learning_rate = lr
        for param_group in self.predictor.optimizer.param_groups:
            param_group['lr'] = lr
        print(f"Meta-EPM: Base learning rate updated to {lr}")

    def reset_state(self):
        """Reset the GRU hidden state (used on catastrophic misses)."""
        self.prev_hidden_state = None
        self.current_hidden_state = None
        self.predictor.reset_state()
        print("Meta-EPM: Temporal state reset.")

    def reset(self):
        self.prev_embedding = None
        self.prev_hidden_state = None
        self.current_hidden_state = None
        self.latent_buffer.clear()
        self.memory.reset()
        self.predictor.reset_state()
