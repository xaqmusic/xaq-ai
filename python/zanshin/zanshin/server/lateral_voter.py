import time
import collections
from collections import deque, defaultdict, Counter
import numpy as np
from .structures import RealityToken, ConsensusToken

class LateralVoterNode:
    def __init__(self, sync_window_ms=250, hebbian_rate=0.05, decay_rate=0.0, threshold=0.3):
        self.sync_window_ms = sync_window_ms
        self.hebbian_rate = hebbian_rate
        self.decay_rate = decay_rate
        self.threshold = threshold
        self.lookback_window = 25
        # Baseline score for any two EPMs arriving in the same 25Hz consensus frame.
        # This bootstraps Hebbian learning from pure temporal co-occurrence — the design
        # intent is for co-firing modalities to cross the threshold "natively" before any
        # trajectory agreement is established.  Set > threshold so cold-start Hebbian fires.
        self.temporal_coincidence = 0.1
        self.agent_timeout = 0.7   # 700ms — increased from 0.4s; inference resonance dropped
                                   # from 0.60 → 0.25 with 0.4s, too many EPMs timing out;
                                   # 700ms gives 2-3 tokens margin for all EPMs (slowest ~3Hz)
        
        # Fine-grained voting logic weights
        self.trajectory_weight = 0.75
        self.instant_match_weight = 0.0
        self.node_weight = 3.0
        self.neurotransmitter_weight = 4.0
        
        # Buffers: source_id -> deque of RealityTokens
        self.input_buffers = defaultdict(deque)
        
        # Association Matrix: [NodeA][NodeB] -> Weight
        # We use a nested default dict for sparse storage
        self.association_matrix = defaultdict(lambda: defaultdict(float))
        
        # State
        self.last_consensus = None
        self.max_consensus_count = 5000
        self.consensus_history = deque(maxlen=self.max_consensus_count)
        self.active_sources = set()
        self.last_received_time = defaultdict(float)
        
        # Structural Resolver: (source, node) -> 768-dim vector
        self.node_embeddings = {}
        self.embedding_dim = 768
        self.last_fused_z = None
        self.ema_alpha = 0.8

        # Semantic resonance: weight for cosine similarity between centroid embeddings
        # in the pairwise pair_score. 0.0 = off (pure symbolic); >0 uses real centroids.
        self.semantic_resonance_weight = 0.0

        # Centroid cache: source_id -> np.ndarray (n_nodes, dim) loaded from graph_state.bin
        # None means "tried to load but file not found" (will SHA-256 fallback for all nodes)
        self._centroid_cache = {}
        self._exports_dir = None  # resolved lazily on first use

        # HDC temporal position anchors (fixed, reproducible)
        # 12 anchor hypervectors spanning the sync window; interpolated at query time
        _rng = np.random.default_rng(0xDEADBEEF)
        _anchors = _rng.standard_normal((12, self.embedding_dim))
        self._temporal_anchors = _anchors / np.linalg.norm(_anchors, axis=1, keepdims=True)

        # Persistent Logging Logic (Op B)
        self.total_consensus_count = 0
        self.concept_counts = Counter()
        self.concept_labels = {}
        
        # Callbacks for UI/Workspace updates
        self.ui_callback = None
        self.consensus_subscribers = []

    def reset_cognitive_state(self):
        """Trigger a full reset of the latent world model (useful when topology changes)."""
        self.last_fused_z = None
        self.node_embeddings = {}
        # The meta-epm is reset via the harness

    def _resolve_exports_dir(self):
        """Lazily resolve the exports directory relative to this file."""
        import os
        if self._exports_dir is None:
            self._exports_dir = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '../../../exports')
            )
        return self._exports_dir

    def _load_centroids_for_source(self, source_id):
        """Load graph_state.bin for source_id and cache the node centroids.

        Format (matches C++ epm.cpp and Python brain.py):
          header : int32[2]  = [n_nodes, embedding_dim]
          body   : float32[n_nodes × embedding_dim]  row-major

        Sets self._centroid_cache[source_id] to the ndarray, or None on failure.
        """
        import os
        exports_dir = self._resolve_exports_dir()
        bin_path = os.path.join(exports_dir, source_id, 'graph_state.bin')
        if not os.path.exists(bin_path):
            print(f"[LateralVoter] No graph_state.bin for '{source_id}' — SHA-256 fallback")
            self._centroid_cache[source_id] = None
            return
        try:
            with open(bin_path, 'rb') as f:
                header = np.frombuffer(f.read(8), dtype=np.int32)
                n_nodes, dim = int(header[0]), int(header[1])
                if n_nodes > 0 and dim == self.embedding_dim:
                    raw = np.frombuffer(f.read(n_nodes * dim * 4), dtype=np.float32)
                    centroids = raw.reshape(n_nodes, dim).copy()
                else:
                    centroids = np.zeros((0, self.embedding_dim), dtype=np.float32)
            self._centroid_cache[source_id] = centroids
            print(f"[LateralVoter] Loaded {n_nodes} centroids for '{source_id}' from graph_state.bin")
        except Exception as e:
            print(f"[LateralVoter] Failed to load centroids for '{source_id}': {e} — SHA-256 fallback")
            self._centroid_cache[source_id] = None

    def _get_structural_embedding(self, source_id, node_id):
        """Map a (source, node_id) pair to a stable 768-dim unit vector.

        Priority:
          1. Exported graph centroid (real learned visual feature from graph_state.bin)
          2. SHA-256-seeded random vector with Hebbian neighbor blending (fallback for
             nodes formed after export — new nodes created during live inference)
        """
        if node_id == -1:
            return np.zeros(self.embedding_dim)

        key = (source_id, node_id)
        if key not in self.node_embeddings:
            # Ensure centroid table is loaded for this source
            if source_id not in self._centroid_cache:
                self._load_centroids_for_source(source_id)

            centroids = self._centroid_cache.get(source_id)
            if centroids is not None and node_id < len(centroids):
                # Real learned centroid — unit-normalise and cache
                v = centroids[node_id].astype(np.float64)
                norm = np.linalg.norm(v)
                self.node_embeddings[key] = v / norm if norm > 1e-6 else v
            else:
                # SHA-256 fallback for post-export nodes
                import hashlib
                neighbor_weights = []
                if node_id in self.association_matrix:
                    for other_id, weight in self.association_matrix[node_id].items():
                        if weight > 0.1 and (source_id, other_id) in self.node_embeddings:
                            neighbor_weights.append((weight, self.node_embeddings[(source_id, other_id)]))
                neighbor_weights.sort(key=lambda x: x[0], reverse=True)

                seed_bytes = f"{source_id}_{node_id}".encode('utf-8')
                sha = hashlib.sha256(seed_bytes).digest()
                seed = int.from_bytes(sha[:4], 'big') % (2**32)
                rng = np.random.default_rng(seed)
                innovation = rng.standard_normal(self.embedding_dim)
                innovation /= np.linalg.norm(innovation)

                if neighbor_weights:
                    top_neighbor_v = neighbor_weights[0][1]
                    v = 0.7 * top_neighbor_v + 0.3 * innovation
                else:
                    v = innovation

                self.node_embeddings[key] = v / np.linalg.norm(v)

        return self.node_embeddings[key]

    def _get_temporal_position_hv(self, t_norm):
        """Interpolated hypervector for a normalized temporal position [0, 1].

        Early tokens (t_norm≈0) and late tokens (t_norm≈1) within the same window
        get distinct but continuous hypervectors, so a window where audio fired
        50ms before optical_flow is geometrically different from one where they fired
        simultaneously — without any hand-coded spatial priors.
        """
        t_norm = float(np.clip(t_norm, 0.0, 1.0))
        n = len(self._temporal_anchors)
        idx_f = t_norm * (n - 1)
        idx_lo = int(idx_f)
        idx_hi = min(idx_lo + 1, n - 1)
        alpha = idx_f - idx_lo
        hv = (1.0 - alpha) * self._temporal_anchors[idx_lo] + alpha * self._temporal_anchors[idx_hi]
        norm = np.linalg.norm(hv)
        return hv / norm if norm > 1e-6 else hv

    def _compose_window_hdc(self, candidates):
        """HDC compositional window embedding.

        Each candidate's per-token embedding (which already encodes its own
        node + trajectory via permutation in ingest_token) is bound with its
        temporal position inside the current window, then all are superposed.

        Key property: two windows sharing k of n EPM symbols have cosine
        similarity ≈ k/n in this space — without any learned metric.
        A novel window that differs by one column lands geometrically near
        its closest familiar neighbor, allowing the meta-EPM knowledge graph
        to place it correctly without requiring an exact match.
        """
        if not candidates:
            return np.zeros(self.embedding_dim)

        times = [c.received_time for c in candidates]
        t_min = min(times)
        # Normalize against the fixed sync window size so that absolute timing
        # differences are preserved. Tokens 5ms apart map to nearly the same
        # temporal bin; tokens 80ms apart map to distinct bins.
        t_scale = self.sync_window_ms / 1000.0  # seconds

        composed = np.zeros(self.embedding_dim)
        total_weight = 0.0

        for token in candidates:
            if token.embedding_vector is None:
                continue

            t_norm = np.clip((token.received_time - t_min) / t_scale, 0.0, 1.0)
            t_hv = self._get_temporal_position_hv(t_norm)

            # HDC binding: element-wise multiply associates the symbol with its
            # temporal slot. bind(A, T1) and bind(A, T2) are ~orthogonal when
            # T1 ≠ T2, so tight vs loose coincidence produces distinct embeddings.
            bound = token.embedding_vector * t_hv

            # Confidence weighting: baked/supernodes (high dopamine/serotonin)
            # contribute more to the window composition
            w = (1.0 + token.dopamine_level) * (0.1 + token.serotonin_level)
            composed += bound * w
            total_weight += w

        if total_weight < 1e-6:
            return np.zeros(self.embedding_dim)

        composed /= total_weight
        norm = np.linalg.norm(composed)
        return composed / norm if norm > 1e-6 else composed

    def register_callback(self, callback):
        self.ui_callback = callback
        
    def subscribe_consensus(self, callback):
        """Allow an external workspace module to subscribe to consensus updates (e.g. for Active Inference)."""
        if callback not in self.consensus_subscribers:
            self.consensus_subscribers.append(callback)

    def set_threshold(self, value):
        self.threshold = value

    def set_max_consensus_count(self, max_val):
        self.max_consensus_count = int(max_val)
        # Create a new deque with the new maxlen, copying over existing elements
        # If new max_val is smaller, extending will implicitly drop the oldest elements
        new_deque = deque(self.consensus_history, maxlen=self.max_consensus_count)
        self.consensus_history = new_deque

    def ingest_token(self, token_data: dict):
        """
        Ingest a raw dictionary from the socket, convert to RealityToken, 
        and store in buffer.
        """
        try:
            # Parse payload
            payload = token_data.get('payload', {})
            header = token_data.get('header', {})
            source_id = header.get('agent_id', 'unknown')
            
            # Extract fields
            # Note: The Brain attributes might be nested differently based on socket_server.py
            # Let's align with the spec and current Brain output
            
            # Trajectory
            traj = payload.get('trajectory', [])
            history = [t.get('id') for t in traj if isinstance(t, dict)]
            
            # Neurotransmitters
            nt = payload.get('neurotransmitters', {})
            dopa = nt.get('dopamine', 0.0)
            sero = nt.get('serotonin', 0.0)
            
            # Embedding Logic (Structural or Payload)
            emb = payload.get('embedding')
            if emb:
                emb = np.array(emb)
            else:
                # Generate structural embedding from ID + Pathway
                base_emb = self._get_structural_embedding(source_id, payload.get('current_id', -1))
                
                # Trajectory Integration (HDC Permutation Encoding)
                path_emb = np.zeros(self.embedding_dim)
                decay = 0.5
                # history[:10] contains the most recent nodes in the pathway
                # We apply i+1 circular shifts to the i-th historical node
                for i, prev_id in enumerate(reversed(history)):
                    if i >= 10: break # Limit depth for performance
                    h_emb = self._get_structural_embedding(source_id, prev_id)
                    # Permute: np.roll(vector, shift)
                    permuted = np.roll(h_emb, i + 1)
                    path_emb += permuted * (decay ** (i + 1))
                
                emb = base_emb + path_emb
                norm = np.linalg.norm(emb)
                if norm > 1e-6:
                    emb = emb / norm
            
            token = RealityToken(
                source_id=source_id,
                timestamp=header.get('timestamp', time.time()),
                received_time=time.time(),
                active_node_id=payload.get('current_id', -1),
                history_trace=history,
                dopamine_level=dopa,
                serotonin_level=sero,
                confidence=1.0, 
                embedding_vector=emb,
                text_label=payload.get('text_label', "")
            )
            
            self.input_buffers[source_id].append(token)
            self.active_sources.add(source_id)
            self.last_received_time[source_id] = time.time()
            
            # Prune old tokens safely (retain 5 seconds max of continuous 25Hz history)
            self._prune_buffers(source_id)
            
        except Exception as e:
            print(f"Error ingesting token: {e}")

    def reset_consensus(self):
        import gc
        self.consensus_history.clear()
        self.association_matrix.clear()
        self.input_buffers.clear()
        self.active_sources.clear()
        
        # Reset permanent tracking
        self.total_consensus_count = 0
        self.concept_counts.clear()
        self.concept_labels.clear()
        
        self.last_consensus = None
        gc.collect()

    def _prune_buffers(self, source_id):
        buf = self.input_buffers[source_id]
        while len(buf) > 150:
            buf.popleft()

    def process_consensus(self):
        """
        Main loop to find consensus among buffered tokens.
        Executed typically at 25Hz.
        """
        current_time = time.time()
        
        # 1. New Data Check
        if self.last_consensus:
            new_data = False
            for source in self.active_sources:
                if (self.input_buffers[source] and 
                    self.input_buffers[source][-1].timestamp > self.last_consensus.timestamp):
                    new_data = True
                    break
            
            # Fallback: if we haven't processed in 1 second, force one to keep UI alive
            if not new_data and (current_time - self.last_consensus.timestamp < 1.0):
                return None 

        candidates = []
        
        for source in list(self.active_sources):
            if not self.input_buffers[source]:
                continue
            
            # If server physically received a token from this agent within the dynamic timeout window
            if current_time - self.last_received_time[source] < self.agent_timeout:
                latest = self.input_buffers[source][-1]
                candidates.append(latest)
        
        if len(candidates) < 1:
            return None
        
        # 2. Resonance Calculation
        resonance = 0.0

        # HDC window composition applies regardless of candidate count —
        # single-source windows still go through the same compositor so the
        # meta-EPM always sees the same embedding format.
        weighted_vector = self._compose_window_hdc(candidates)
        self.last_fused_z = weighted_vector

        # If single source, resonance is its own confidence/serotonin
        if len(candidates) == 1:
            token = candidates[0]
            resonance = token.serotonin_level * token.dopamine_level
            # Avoid zeroes
            resonance = max(resonance, 0.1)
        else:
            # Pairwise comparison
            pair_scores = []
            contributors = set()
            
            for i in range(len(candidates)):
                for j in range(i + 1, len(candidates)):
                    tA = candidates[i]
                    tB = candidates[j]
                    
                    # Trajectory Match: weighted by position (recency)
                    # history_trace is [oldest -> newest]
                    hA = tA.history_trace[-5:]
                    hB = tB.history_trace[-5:]
                    
                    # Zip from the end (newest)
                    match_count = 0
                    for nA, nB in zip(reversed(hA), reversed(hB)):
                        if nA == nB and nA != -1:
                            match_count += 1
                        else:
                            break # Sequence broken
                    
                    overlap_score = match_count / 5.0 # Max 1.0
                    
                    # Hebbian boost
                    # Always query the upper-triangular index
                    min_id, max_id = min(tA.active_node_id, tB.active_node_id), max(tA.active_node_id, tB.active_node_id)
                    hebbian_boost = self.association_matrix[min_id][max_id]
                    
                    # Instantaneous Match
                    instant_match = 1.0 if (tA.active_node_id == tB.active_node_id and tA.active_node_id != -1) else 0.0
                    
                    # Semantic cosine similarity between centroid embeddings (Fix A)
                    # Only meaningful when graph_state.bin centroids are loaded.
                    sem_score = 0.0
                    if self.semantic_resonance_weight > 0.0:
                        eA = self._get_structural_embedding(tA.source_id, tA.active_node_id)
                        eB = self._get_structural_embedding(tB.source_id, tB.active_node_id)
                        cos_sim = float(np.dot(eA, eB))  # both unit vectors
                        sem_score = self.semantic_resonance_weight * max(0.0, cos_sim)

                    # Temporal Co-occurrence (They are firing at the exact same time!)
                    # We give a baseline score just for arriving together in the same 25Hz consensus frame
                    # This allows mathematically disconnected modalities to cross the hebbian threshold natively
                    #
                    # Hebbian boost is capped at 1.0 so unbounded matrix growth can't inflate pair_score.
                    hebbian_boost_capped = min(hebbian_boost, 1.0)
                    pair_score = (overlap_score * self.trajectory_weight) + (instant_match * self.instant_match_weight) + (hebbian_boost_capped * 0.4) + self.temporal_coincidence + sem_score
                    # Clip to [0, 1] so resonance naturally lives in [0, 1] without needing a hard clamp.
                    pair_score = min(max(pair_score, 0.0), 1.0)

                    # NOTE: Neurotransmitter trust (wA/wB) intentionally removed from the resonance
                    # calculation.  Chemicals should influence which modalities dominate the HDC
                    # fused_embedding (_compose_window_hdc already does this), but they must NOT
                    # amplify the agreement metric — that causes pair_scores to exceed 1.0 whenever
                    # any well-trained EPM has high serotonin, making resonance trivially = 1.0 always.
                    pair_scores.append(pair_score)

            resonance = sum(pair_scores) / max(len(pair_scores), 1)
            # resonance is naturally in [0, 1] because pair_score is clipped above.
            # The old avg_chem fallback is removed: if there is no trajectory/Hebbian agreement,
            # resonance should be low regardless of individual modality confidence.
            
        # 3. Update Hebbian Weights (if high resonance)
        if resonance > self.threshold and len(candidates) > 1:
             import logging
             matrix_updated = False
             for i in range(len(candidates)):
                for j in range(i + 1, len(candidates)):
                    idA = candidates[i].active_node_id
                    idB = candidates[j].active_node_id
                    if idA != -1 and idB != -1:
                        # Enforce upper-triangular matrix to prevent symmetrical mirror clutter
                        min_id, max_id = min(idA, idB), max(idA, idB)
                        # Cap at 1.0 so hebbian_boost * 0.4 stays bounded in pair_score
                        self.association_matrix[min_id][max_id] = min(
                            self.association_matrix[min_id][max_id] + self.hebbian_rate,
                            1.0
                        )
                        matrix_updated = True
             
             if matrix_updated:
                 logging.getLogger("LateralVoter").debug(f"Hebbian Matrix updated! Resonance: {resonance:.3f} pushed for {len(candidates)} candidate matching instances.")
        
        # 4. Decay Matrix Weights Systematically
        if self.decay_rate > 0.0:
            nodes_to_prune = []
            for nA in list(self.association_matrix.keys()):
                for nB in list(self.association_matrix[nA].keys()):
                    new_weight = self.association_matrix[nA][nB] - self.decay_rate
                    if new_weight <= 0.001:
                        nodes_to_prune.append((nA, nB))
                    else:
                        self.association_matrix[nA][nB] = new_weight
                        
            for nA, nB in nodes_to_prune:
                if nB in self.association_matrix[nA]:
                    del self.association_matrix[nA][nB]
                if nA in self.association_matrix.get(nB, {}):
                    del self.association_matrix[nB][nA]
                
                # Clean up empty rows
                if nA in self.association_matrix and not self.association_matrix[nA]:
                    del self.association_matrix[nA]
                if nB in self.association_matrix and not self.association_matrix[nB]:
                    del self.association_matrix[nB]
        
        # 5. Create Consensus Token
        consensus = ConsensusToken(
            timestamp=max(c.timestamp for c in candidates) if candidates else current_time,
            fused_embedding=weighted_vector,
            resonance_score=resonance,
            contributing_sources=[c.source_id for c in candidates],
            contributing_ids=[c.active_node_id for c in candidates],
            text_labels=[c.text_label for c in candidates if c.text_label]
        )
        
        import logging
        if resonance > 0.1:
            logging.getLogger("LateralVoter").debug(f"Consensus Token Generated | Sources: {[c.source_id for c in candidates]} | Resonance: {resonance:.3f}")
        
        self.last_consensus = consensus
        self.consensus_history.append(consensus)
        
        # Permanent Logging Updates
        self.total_consensus_count += 1
        
        if consensus.contributing_ids:
            group_key = tuple(sorted(consensus.contributing_ids))
            self.concept_counts[group_key] += 1
            # Store the text labels for the UI mapper
            if group_key not in self.concept_labels and consensus.text_labels:
                self.concept_labels[group_key] = " + ".join(consensus.text_labels)
        
        # Notify UI and Subscribers
        if self.ui_callback:
            self.ui_callback({
                'consensus': consensus,
                'candidates': candidates,
                'matrix_size': len(self.association_matrix)
            })
            
        for subscriber in self.consensus_subscribers:
            try:
                subscriber(consensus, candidates)
            except Exception as e:
                import logging
                logging.getLogger("LateralVoter").error(f"Error notifying consensus subscriber: {e}")
            
        return consensus

    def retroactive_boost_pathways(self, factor=0.2):
        """Retroactively boost associations for nodes that coordinated leading up to a reward."""
        import logging
        logger = logging.getLogger("LateralVoter")
        history_len = min(self.lookback_window, len(self.consensus_history))
        if history_len == 0:
            return
            
        recent_tokens = list(self.consensus_history)[-history_len:]
        boosted_count = 0
        
        for token in recent_tokens:
            ids = token.contributing_ids
            for i in range(len(ids)):
                for j in range(i + 1, len(ids)):
                    if ids[i] != -1 and ids[j] != -1:
                        min_id, max_id = min(ids[i], ids[j]), max(ids[i], ids[j])
                        self.association_matrix[min_id][max_id] = min(
                            self.association_matrix[min_id][max_id] + factor,
                            1.0
                        )
                        boosted_count += 1
                        
        logger.info(f"Retroactive Hebbian Boost: applied +{factor} to {boosted_count} pairwise associations over the last {history_len} frames.")
        
    def retroactive_penalize_pathways(self, factor=0.2):
        """Retroactively penalize associations for nodes that led to a failure state."""
        import logging
        logger = logging.getLogger("LateralVoter")
        history_len = min(self.lookback_window, len(self.consensus_history))
        if history_len == 0:
            return
            
        recent_tokens = list(self.consensus_history)[-history_len:]
        penalized_count = 0
        
        for token in recent_tokens:
            ids = token.contributing_ids
            for i in range(len(ids)):
                for j in range(i + 1, len(ids)):
                    if ids[i] != -1 and ids[j] != -1:
                        min_id, max_id = min(ids[i], ids[j]), max(ids[i], ids[j])
                        new_weight = self.association_matrix[min_id][max_id] - factor
                        # Do not clamp so bad associations drop below the pruning threshold natively
                        self.association_matrix[min_id][max_id] = new_weight
                        penalized_count += 1
                        
        logger.warning(f"Retroactive Hebbian Penalty: applied -{factor} to {penalized_count} pairwise associations over the last {history_len} frames.")
