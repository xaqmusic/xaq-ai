import torch
import torch.nn.functional as F
import numpy as np

class KnowledgeGraph:
    """
    The 'Memory'.
    Stores Concept Attractors (Nodes) and Transitions (Edges) on GPU.
    """
    def __init__(self, embedding_dim=768, max_nodes=100000, device='cuda'):
        self.device = device
        self.embedding_dim = embedding_dim
        self.max_nodes = max_nodes
        self.distance_metric = "euclidean" # default
        
        # Pre-allocate tensors for zero-copy efficiency
        # Nodes: (Max_Nodes, Embedding_Dim)
        self.nodes = torch.zeros((max_nodes, embedding_dim), device=device)
        
        # Edges: Transition counts/probs (Max, Max)
        self.edges = torch.zeros((max_nodes, max_nodes), device=device)
        
        # Stability: How "baked" is a node? (0 to 1)
        # nodes with high trigger counts become more stable.
        self.trigger_counts = torch.zeros(max_nodes, device=device)
        self.last_trigger_times = torch.zeros(max_nodes, device=device)
        self.salience = torch.zeros(max_nodes, device=device) # Leaky accumulator for pruning
        self.creation_times = torch.zeros(max_nodes, device=device) # For eviction logic
        
        # New: Crystallization / Baking
        self.baking_threshold = 100 # default from spec: 100
        self.baked = torch.zeros(max_nodes, dtype=torch.bool, device=device)
        
        # Consistency Tracker (Backward TLE)
        # consistency_score = consistency_accumulator / item_consistency_counts
        self.consistency_accumulator = torch.zeros(max_nodes, device=device)
        self.consistency_counts = torch.zeros(max_nodes, device=device)
        self.consistency_threshold = 0.15
        
        # Track counts
        self.node_count = 0
        
        # Labels for nodes (User defined)
        # Map: index -> label_string
        self.labels = {}
        # Optimization: Cached Label Mask (True if node has label)
        self.label_mask = torch.zeros(max_nodes, dtype=torch.bool, device=device)
        
        # RL Label Tracking: Maps NodeID -> {Label -> Count}
        self.label_history = {}
        
        # Track active node (last matched concept)
        self.last_node_idx = None
        self.last_transition = None # (source, target) indices

        # Colors for nodes (User defined)
        # Map: index -> hex_color_string
        # Colors for nodes (User defined)
        # Map: index -> hex_color_string
        self.colors = {}

        # UI Layout: Map: index -> {x, y, fx, fy}
        self.layout = {}

        # Proposal 3: EPM contributor signatures per node.
        # Stores the (source_id, node_id) pairs that were active in the window
        # that founded each meta-EPM graph node.  Used by find_nearest_concept_warped
        # to pull Hebbian-associated nodes geometrically closer at lookup time.
        self.node_signatures = {}  # node_idx -> frozenset of (source_id, node_id)

        # Index Recycling
        self.free_indices = []
        
        # Topological Refinement (SuperNodes)
        self.supernodes = {} # ID -> {centroid, label, children:[]}
        self.super_edges = {} # (src_id, tgt_id) -> weight
        self.node_supernode_map = {} # atomic_node_idx -> supernode_id
        self.hidden_nodes = torch.zeros(max_nodes, dtype=torch.bool, device=device)
        
    def reset(self):
        """Clear all nodes, edges, and labels."""
        self.nodes.zero_()
        self.edges.zero_()
        self.trigger_counts.zero_()
        self.last_trigger_times.zero_()
        self.salience.zero_()
        self.creation_times.zero_()
        self.baked.zero_()
        self.consistency_accumulator.zero_()
        self.consistency_counts.zero_()
        self.node_count = 0
        self.labels = {}
        self.label_mask.zero_()
        self.label_history = {}
        self.colors = {}
        self.layout = {}
        self.last_node_idx = None
        self.node_signatures = {}
        self.free_indices = []

        # Reset Refinement
        self.supernodes = {}
        self.super_edges = {}
        self.node_supernode_map = {}
        self.hidden_nodes.zero_()

    def find_nearest_concept(self, embedding):
        """
        Finds the closest existing node to the input embedding.
        Returns: (index, distance)
        If empty, returns (None, infinity)
        """
        if self.node_count == 0:
            return None, float('inf')
            
        # Limit search to active nodes
        active_nodes = self.nodes[:self.node_count]
        
        if self.distance_metric == "euclidean":
            # Calculate distances (Euclidean)
            # embedding: (1, Dim)
            # active_nodes: (N, Dim) -> dists: (N)
            dists = torch.norm(active_nodes - embedding, dim=1)
            min_dist, idx = torch.min(dists, dim=0)
            return idx.item(), min_dist.item()
        else:
            # Calculate similarities (Cosine)
            # F.cosine_similarity: (N, Dim), (1, Dim) -> (N)
            # Returns values in [-1, 1], where 1 is identical.
            # Mirror the 'distance' concept: 1.0 - similarity. 
            # Scale by 10.0 to match Euclidean magnitude and increase sensitivity.
            sims = F.cosine_similarity(active_nodes, embedding, dim=1)
            max_sim, idx = torch.max(sims, dim=0)
            return idx.item(), 10.0 * (1.0 - max_sim.item())

    def find_nearest_concept_warped(self, embedding, contributors, association_matrix, warp_strength=1.0):
        """
        Hebbian-warped nearest-concept lookup (Proposal 3).

        The base Euclidean distance to each meta-EPM node is divided by
        (1 + warp_strength × hebbian_affinity), so nodes whose founding
        symbol-set is strongly associated with the current window's symbols
        are pulled geometrically closer — without changing their actual centroid.

        contributors:       list of (source_id, node_id) tuples for this window
        association_matrix: lateral voter's Hebbian matrix
                            defaultdict(defaultdict(float)), keyed by raw node_id
        warp_strength:      scale of the warp; 0 = no warp (pure Euclidean)

        Falls back to plain Euclidean if no signatures are stored yet.
        """
        if self.node_count == 0:
            return None, float('inf')

        active_nodes = self.nodes[:self.node_count]
        dists = torch.norm(active_nodes - embedding, dim=1).cpu().numpy()

        if contributors and association_matrix is not None:
            contributor_set = frozenset(contributors)
            for k in range(self.node_count):
                sig = self.node_signatures.get(k)
                if not sig:
                    continue

                affinity = 0.0

                # Direct overlap: same (source, node) in both windows
                # (already captured by HDC similarity, but a small bonus
                #  reinforces well-known concepts without double-counting)
                affinity += 0.5 * len(contributor_set & sig)

                # Cross-modal Hebbian affinity: use the lateral voter's
                # accumulated co-occurrence weights between node IDs.
                # Source is intentionally ignored here — the association
                # matrix records cross-EPM co-firing by raw node ID, which
                # is how the voter builds it.
                for (_, nid_a) in contributors:
                    for (_, nid_b) in sig:
                        if nid_a == nid_b:
                            continue  # Counted in direct overlap above
                        min_id = min(nid_a, nid_b)
                        max_id = max(nid_a, nid_b)
                        affinity += association_matrix[min_id][max_id]

                if affinity > 0.0:
                    dists[k] /= (1.0 + warp_strength * affinity)

        best_idx = int(np.argmin(dists))
        return best_idx, float(dists[best_idx])

    def add_concept(self, embedding, now=0, signatures=None):
        """
        Creates a new node from the embedding.

        signatures: optional iterable of (source_id, node_id) tuples representing
                    the EPM contributors active when this node was crystallised.
                    Stored for use by find_nearest_concept_warped.
        """
        if self.free_indices:
            idx = self.free_indices.pop()
        elif self.node_count < self.max_nodes:
            idx = self.node_count
            self.node_count += 1
        else:
            print(f"KnowledgeGraph: CRITICAL - Max nodes ({self.max_nodes}) reached. Scaling limit hit.")
            return self.node_count - 1

        self.nodes[idx] = embedding.detach()
        self.last_trigger_times[idx] = now
        self.creation_times[idx] = now
        self.salience[idx] = 1.0
        self.trigger_counts[idx] = 0

        # Clear any stale metadata
        self.labels.pop(idx, None)
        self.label_history.pop(idx, None)
        self.label_mask[idx] = False
        self.colors.pop(idx, None)
        if str(idx) in self.layout:
            del self.layout[str(idx)]

        # Store EPM contributor signature for Hebbian warping
        self.node_signatures[idx] = frozenset(signatures) if signatures else frozenset()

        return idx

    def update_centroid(self, node_idx, embedding, lr=0.1, now=0):
        """
        Pull an existing node slightly toward the current embedding.
        Stability reduces the learning rate (baking).
        """
        if 0 <= node_idx < self.node_count:
            # Update trigger time and salience
            self.last_trigger_times[node_idx] = now
            self.salience[node_idx] = torch.clamp(self.salience[node_idx] + 0.2, 0, 1.0) # Accumulate
            
            # If Baked, LOCKED. No centroid update.
            if self.baked[node_idx]:
                 return True
                 
            # Calculate stability [0.0 to 1.0] for transient nodes
            stability = torch.clamp(self.trigger_counts[node_idx] / self.baking_threshold, 0.0, 0.9)
            
            # Damped learning rate
            effective_lr = lr * (1.0 - stability)
            
            # Running average update
            self.nodes[node_idx] = (1 - effective_lr) * self.nodes[node_idx] + effective_lr * embedding.detach()
            return True
        return False

    def update_transition(self, current_node_idx, now=0):
        """
        Updates the edge weight from the previous node to the current node.
        """
        if self.last_node_idx is not None and current_node_idx is not None:
             # Increment transition count
             self.edges[self.last_node_idx, current_node_idx] += 1
             self.last_transition = (self.last_node_idx, current_node_idx)
             
        if current_node_idx is not None:
             # Increment trigger count for the destination node (always)
             self.trigger_counts[current_node_idx] += 1
             self.last_trigger_times[current_node_idx] = now
             # Salience is already boosted in update_centroid/add_concept
             self.salience[current_node_idx] = torch.clamp(self.salience[current_node_idx] + 0.1, 0, 1.0)
        
        self.last_node_idx = current_node_idx

    def reinforce_transition(self, src, tgt, factor=1.5):
        """
        Boosts the weight of a specific transition (Hebbian reward).
        """
        if src is not None and tgt is not None and 0 <= src < self.node_count and 0 <= tgt < self.node_count:
            self.edges[src, tgt] *= factor
            # Ensure we don't overflow or become too dominant too fast, 
            # though edges are raw counts, scaling them is a proxy for frequency.
            self.edges[src, tgt] = torch.clamp(self.edges[src, tgt], 0, 10000)
            # Also boost salience of involved nodes
            self.salience[src] = torch.clamp(self.salience[src] + 0.2, 0, 1.0)
            self.salience[tgt] = torch.clamp(self.salience[tgt] + 0.2, 0, 1.0)
            return True
        return False

    def penalize_transition(self, src, tgt, factor=0.5):
        """
        Reduces the weight of a specific transition (Hebbian punishment).
        """
        if src is not None and tgt is not None and 0 <= src < self.node_count and 0 <= tgt < self.node_count:
            self.edges[src, tgt] *= factor
            return True
        return False

    def update_consistency(self, node_idx, backward_tle):
        """
        Updates the consistency tracker for a specific node.
        """
        if 0 <= node_idx < self.node_count:
            self.consistency_accumulator[node_idx] += backward_tle
            self.consistency_counts[node_idx] += 1

    def check_baking(self, node_idx, dynamic_threshold=None):
        """
        Runs the Baking Gates (Frequency + Consistency).
        Returns: 'pass' (Baked), 'fail' (Pruned), 'wait' (Transient)
        """
        if not (0 <= node_idx < self.node_count):
            return "wait"
            
        if self.baked[node_idx]:
            return "pass" # Already baked

        triggers = self.trigger_counts[node_idx].item()
        
        # Gate 1: Frequency
        if triggers < self.baking_threshold:
            return "wait"
            
        # Gate 2: Consistency
        c_counts = self.consistency_counts[node_idx].item()
        if c_counts == 0:
            return "wait" # Should not happen if called correctly
            
        score = self.consistency_accumulator[node_idx].item() / c_counts
        
        threshold_to_use = dynamic_threshold if dynamic_threshold is not None else self.consistency_threshold
        
        if score <= threshold_to_use:
            # Pass
            self.baked[node_idx] = True
            return "pass"
        else:
            # Fail
            return "fail"

    def get_label(self, node_idx):
        """Robustly retrieve label for atomic or supernode."""
        if node_idx is None: return None
        
        # 1. Direct Lookup
        if node_idx in self.labels:
            return self.labels[node_idx]
        if str(node_idx) in self.labels:
            return self.labels[str(node_idx)]
            
        # 2. Supernode Fallback
        try:
            val = int(node_idx)
            if val < 0:
                sn_id = self.get_sn_id_from_visual_id(val)
                if sn_id and sn_id in self.supernodes:
                    return self.supernodes[sn_id].get('label')
        except:
             pass
        return None

    def set_label(self, node_idx, label):
        """Sets a human-readable label for a concept node."""
        try:
            target_idx = int(node_idx)
            if 0 <= target_idx < self.node_count:
                # Atomic Node
                if target_idx in self.free_indices: return False
                self.labels[target_idx] = label
                self.label_mask[target_idx] = True
                return True
            elif target_idx < 0: 
                # SuperNode ID (Visual ID)
                # Propagate label to ALL children to persist it as a User Group
                sn_id = self.get_sn_id_from_visual_id(target_idx)
                if sn_id and sn_id in self.supernodes:
                    children = self.supernodes[sn_id].get('children', [])
                    for child_idx in children:
                         self.labels[int(child_idx)] = label
                    # Also label the supernode itself for immediate UI feedback
                    self.supernodes[sn_id]['label'] = label
                    return True
                
                # Fallback: just set in labels (might be just a stray ID)
                self.labels[target_idx] = label
                return True
        except (ValueError, TypeError):
            # String ID
            if isinstance(node_idx, str):
                 # Try to find if it matches a supernode key
                 if node_idx in self.supernodes:
                     children = self.supernodes[node_idx].get('children', [])
                     for child_idx in children:
                         self.labels[int(child_idx)] = label
                     self.supernodes[node_idx]['label'] = label
                     return True
                     
                 self.labels[node_idx] = label
                 return True
        return False

    def record_label_vote(self, node_idx, external_label):
        """Records a vote for an external label on a specific node."""
        if not external_label: return 0
        try:
            target_idx = int(node_idx)
            if 0 <= target_idx < self.node_count:
                if target_idx not in self.label_history:
                    self.label_history[target_idx] = {}
                if external_label not in self.label_history[target_idx]:
                    self.label_history[target_idx][external_label] = 0
                self.label_history[target_idx][external_label] += 1
                return self.label_history[target_idx][external_label]
        except (ValueError, TypeError):
            pass
        return 0

    def set_color(self, node_idx, color):
        """Sets a custom color for a concept node."""
        try:
            target_idx = int(node_idx)
            if 0 <= target_idx < self.node_count:
                if target_idx in self.free_indices: return False
                self.colors[target_idx] = color
                return True
            elif target_idx < 0: # SuperNode ID (Visual)
                self.colors[target_idx] = color
                return True
        except (ValueError, TypeError):
             if isinstance(node_idx, str):
                self.colors[node_idx] = color
                return True
        return False

    def set_anchor(self, node_idx, pos_tuple):
        """
        Updates the layout persistence with anchor status.
        pos_tuple: (x, y) or None to unanchor.
        """
        sid = str(node_idx)
        if pos_tuple:
            if sid not in self.layout:
                self.layout[sid] = {}
            # Check if this node is already known to physics?
            # We just store fx, fy (fixed positions) in layout map
            self.layout[sid]['fx'] = float(pos_tuple[0])
            self.layout[sid]['fy'] = float(pos_tuple[1])
        else:
            if sid in self.layout:
                if 'fx' in self.layout[sid]: del self.layout[sid]['fx']
                if 'fy' in self.layout[sid]: del self.layout[sid]['fy']
        return True

    def set_layout(self, layout_data):
        """
        Updates the UI layout positions.
        layout_data: Dict { node_id: {x,y,fx,fy} ... }
        """
        self.layout = layout_data
        return True

    def decay_salience(self, factor=0.99):
        """
        Leak the accumulator for all nodes.
        Bypasses decay for 'baked' or 'labeled' nodes to keep them at full brightness.
        """
        if self.node_count > 0:
            # Identify protected nodes
            baked_mask = (self.trigger_counts[:self.node_count] >= self.baking_threshold)
            
            protected_mask = baked_mask
            
            # Optimization: Use broadcasted multiplicative mask instead of boolean indexing
            # Multiply by factor if NOT protected, else multiply by 1.0 (no-op)
            mult = torch.ones(self.node_count, device=self.device)
            mult[~protected_mask] = factor
            self.salience[:self.node_count] *= mult

    def decay_memory(self, factor=0.999):
        """
        Slowly decay all transition weights, protecting 'baked' nodes.
        This factor should be very close to 1.0 (e.g., 0.999) if called frequently.
        """
        if self.node_count == 0:
            return

        # A transition is considered "baked" if the destination node is stable (reached threshold)
        # We start protecting slightly early (at 80% baking) for stability.
        # stability = torch.clamp(self.trigger_counts[:self.node_count] / self.baking_threshold, 0.0, 1.0)
        # baked_mask = stability > 0.8
        
        # New Logic: Use baked mask directly
        baked_mask = self.baked[:self.node_count]
        
        # Protect edges leading into stable nodes
        # Optimization: Use broadcasted multiply by target_baked status
        # mult has shape (1, node_count). When multiplied with (node_count, node_count),
        # it applies to columns (incoming edges).
        # We multiply by 1.0 if target is baked, else by factor.
        mult = torch.where(baked_mask, 1.0, factor).unsqueeze(0)
        self.edges[:self.node_count, :self.node_count] *= mult

    def prune_topology(self, edge_min=0.5):
        """
        Remove edges that have fallen below the survival threshold.
        Protects edges leading into 'baked' nodes.
        """
        if self.node_count == 0:
            return
            
        # Define baked threshold (matching decay_memory's early protection)
        # stability = torch.clamp(self.trigger_counts[:self.node_count] / self.baking_threshold, 0.0, 1.0)
        # baked_mask = stability > 0.8
        baked_mask = self.baked[:self.node_count]
        
        # Only prune edges where target is NOT baked
        prune_candidate_mask = self.edges[:self.node_count, :self.node_count] < edge_min
        protected_mask = baked_mask.unsqueeze(0).expand(self.node_count, self.node_count)
        
        final_prune_mask = prune_candidate_mask & ~protected_mask
        
        # Optimization: Zero out using multiplicative mask
        self.edges[:self.node_count, :self.node_count] *= (~final_prune_mask).float()

    def prune_stale_nodes(self, now, min_salience=0.01):
        """
        Prunes nodes individually based on their leaky salience accumulator.
        If a node's salience falls below threshold and not baked/labeled, its edges are zeroed.
        """
        if self.node_count == 0:
            return

        # Check staleness per node based on salience
        stale_mask = (self.salience[:self.node_count] < min_salience)
        
        # Protective mask (Don't kill baked nodes)
        # baked_mask = (self.trigger_counts[:self.node_count] >= self.baking_threshold)
        baked_mask = self.baked[:self.node_count]
        
        # Final kill list: Stale AND NOT Baked
        kill_mask = stale_mask & ~baked_mask
        
        if kill_mask.any():
            killed_indices = torch.where(kill_mask)[0].cpu().tolist()
            return self.delete_nodes(killed_indices)
        return False

    def prune_unbaked_nodes(self):
        """
        Immediately removes all nodes that have not reached the baking threshold
        and do not have a label.
        """
        if self.node_count == 0:
            return

        # Identification mask
        # baked_mask = (self.trigger_counts[:self.node_count] >= self.baking_threshold)
        baked_mask = self.baked[:self.node_count]
        
        # Nodes to KILL: NOT Baked
        kill_mask = ~baked_mask
        
        if kill_mask.any():
            killed_indices = torch.where(kill_mask)[0].cpu().tolist()
            return self.delete_nodes(killed_indices)
        return False

    def delete_nodes(self, node_ids, shrink=True):
        """
        Immediately removes specific nodes by their IDs.
        shrink: If True, attempts to reduce node_count if tail nodes are freed.
        """
        if not node_ids:
            return False

        # Convert to list of ints and filter valid ones
        valid_ids = []
        for nid in node_ids:
            try:
                if 0 <= int(nid) < self.node_count:
                    valid_ids.append(int(nid))
            except:
                continue
        
        if not valid_ids:
            return False

        # Create kill mask
        kill_mask = torch.zeros(self.node_count, dtype=torch.bool, device=self.device)
        kill_mask[valid_ids] = True
        
        # Optimization: Zero out all edges connected to these specific nodes via broadcasted multiply
        # This is much faster than boolean indexing [kill_mask, :] = 0
        live_mask = (~kill_mask).float()
        
        # Zero out rows (outgoing): Multiply each row i by live_mask[i]
        self.edges[:self.node_count, :self.node_count] *= live_mask.view(-1, 1)
        # Zero out cols (incoming): Multiply each col j by live_mask[j]
        self.edges[:self.node_count, :self.node_count] *= live_mask.view(1, -1)
        
        # Reset state for these slots
        self.salience[:self.node_count] *= live_mask
        self.trigger_counts[:self.node_count] *= live_mask
        
        # We can't multiply self.nodes by 0 easily if they contain inf
        self.nodes[:self.node_count][kill_mask] = float('inf')
        
        # Reset Consistency Stats
        self.baked[:self.node_count][kill_mask] = False
        self.consistency_accumulator[:self.node_count][kill_mask] = 0
        self.consistency_counts[:self.node_count][kill_mask] = 0
        
        # Cleanup metadata and recycle indices
        for idx in valid_ids:
            self.labels.pop(idx, None)
            self.label_history.pop(idx, None)
            self.label_mask[idx] = False
            self.colors.pop(idx, None)
            if str(idx) in self.layout:
                del self.layout[str(idx)]
            if int(idx) in self.layout: # Just in case it's stored as int
                del self.layout[int(idx)]
            
            if idx not in self.free_indices:
                self.free_indices.append(idx)
        
        if self.last_node_idx in valid_ids:
            self.last_node_idx = None

        # Sort to reuse lower indices first (keeps tensors denser)
        self.free_indices.sort(reverse=True)
        
        # Optimization: Shrink node_count if the highest indices were pruned
        if shrink:
            while self.node_count > 0 and (self.node_count - 1) in self.free_indices:
                last_idx = self.node_count - 1
                self.free_indices.remove(last_idx)
                self.node_count -= 1

        return True

    def get_visual_id(self, atomic_idx):
        """
        Returns the ID used for visualization.
        If node is hidden within a supernode, returns the supernode's visual_id (negative).
        Otherwise returns the atomic_idx.
        """
        if 0 <= atomic_idx < self.node_count:
            if self.hidden_nodes[atomic_idx]:
                 sn_id = self.node_supernode_map.get(atomic_idx)
                 if sn_id and sn_id in self.supernodes:
                     return self.supernodes[sn_id].get('visual_id', -1) # Ensure valid int
            return atomic_idx
        return None

    def get_topology(self, now=0, ttl=60, ttl_enabled=True):
        """
        Returns data for visualization, filtering out isolated 'orphan' nodes 
        and stale nodes (decayed).
        Returns: (kept_indices, filtered_nodes, filtered_edges, labels, counts, colors)
        """
        count = self.node_count
        if count == 0:
            return np.array([]), np.array([]), np.array([]), {}, np.array([]), {}, {}, np.array([]), np.array([]), None

        # 1. Calculate degrees
        edge_slice = self.edges[:count, :count]
        
        # print(f"Memory: get_topology NodeCount={self.node_count}, TotalEdges={edge_slice.sum().item()}") # DEBUG
        
        row_sums = edge_slice.sum(dim=1)
        col_sums = edge_slice.sum(dim=0)
        
        # 2. Determine which nodes to keep
        # Logic: Keep if (connected) AND (not stale OR active OR baked OR labeled)
        
        # A. Connection mask
        connected_mask = (row_sums + col_sums > 1e-3)
        
        # B. Temporal mask (not stale based on salience)
        if ttl_enabled:
            stale_mask = (self.salience[:count] < 0.01)
        else:
            stale_mask = torch.zeros(count, dtype=torch.bool, device=self.device)
            
        # C. Protective mask (Don't prune baked or active nodes even if stale)
        # baked_mask = (self.trigger_counts[:count] >= self.baking_threshold)
        baked_mask = self.baked[:count]
        active_mask = torch.zeros(count, dtype=torch.bool, device=self.device)
        if self.last_node_idx is not None and self.last_node_idx < count:
            active_mask[self.last_node_idx] = True
        # v27: Also protect the source of the last transition to keep the highlight visible
        if self.last_transition:
            s_idx, t_idx = self.last_transition
            if s_idx < count: active_mask[s_idx] = True
            if t_idx < count: active_mask[t_idx] = True
                
        # Final keep logic: Keep if (Connected AND NOT stale) OR (Active) OR (Baked)
        # This ensures baked nodes are never pruned from view even if isolated.
        # AND ensure we respect the explicit hidden_nodes mask (for SuperNode encapsulation)
        is_hidden = self.hidden_nodes[:count]
        
        # Soft-Pruned nodes are inf
        is_finite = torch.isfinite(self.nodes[:count]).all(dim=1)
        
        keep_mask = ((connected_mask & ~stale_mask) | active_mask | baked_mask) & ~is_hidden & is_finite
        keep_mask_np = keep_mask.cpu().numpy()
        
        # 3. Filter
        kept_indices = np.where(keep_mask_np)[0]
        
        filtered_nodes = self.nodes[kept_indices].cpu().numpy()
        # Send only kept-to-kept edges to reduce data size
        filtered_edges = edge_slice[keep_mask, :][:, keep_mask].cpu().numpy()
        filtered_counts = self.trigger_counts[kept_indices].cpu().numpy()
        filtered_salience = self.salience[kept_indices].cpu().numpy()
        filtered_baked = self.baked[kept_indices].cpu().numpy()
        
        # 4. Inject SuperNodes (Visual Refinement)
        final_labels = self.labels.copy()
        
        if self.supernodes:
            super_list = list(self.supernodes.values())
            n_super = len(super_list)
            
            s_nodes = []
            s_counts = []
            s_indices = []
            s_salience = []
            s_baked = []
            s_offsets = {} # super_id -> index in new concatenated array
            
            old_N = len(filtered_nodes)
            start_id = -1
            
            for i, sn in enumerate(super_list):
                # Use pre-assigned stable visual ID if available, else sequential negative ID
                temp_id = sn.get('visual_id', start_id - i)
                s_indices.append(temp_id)
                
                # Centroid
                s_nodes.append(sn['centroid'].cpu().numpy())
                
                # Count
                children_counts = self.trigger_counts[sn['children']].sum().item()
                s_counts.append(children_counts)
                
                # Salience (Supernodes are always distinct)
                s_salience.append(1.0)
                
                # Baked (Supernodes are implicitly baked/stable structures)
                s_baked.append(True)
                
                # Offset for edge matrix
                s_offsets[sn['id']] = old_N + i
                
                # Label - Prioritize manual edits in self.labels
                final_labels[temp_id] = self.labels.get(temp_id, self.labels.get(str(temp_id), f"{sn['label']}"))
            
            if s_nodes:
                filtered_nodes = np.concatenate([filtered_nodes, np.array(s_nodes)], axis=0)
                filtered_counts = np.concatenate([filtered_counts, np.array(s_counts)])
                filtered_salience = np.concatenate([filtered_salience, np.array(s_salience)])
                filtered_baked = np.concatenate([filtered_baked, np.array(s_baked)])
                kept_indices = np.concatenate([kept_indices, np.array(s_indices)])
                
                # Resize Edges
                total_N = old_N + n_super
                new_adj = np.zeros((total_N, total_N))
                
                # Copy atomic edges
                if old_N > 0:
                    new_adj[:old_N, :old_N] = filtered_edges
                
                # Create Atomic Offset Map (for looking up visible atomic nodes)
                atomic_offsets = { int(idx): i for i, idx in enumerate(kept_indices[:old_N]) }

                # Fill Macro Edges
                for (src_key, tgt_key), w in self.super_edges.items():
                    r = -1
                    c = -1
                    
                    # Resolve Source
                    if isinstance(src_key, str): # SuperNode ID
                        r = s_offsets.get(src_key, -1)
                    else: # Atomic ID (Int)
                        r = atomic_offsets.get(src_key, -1)
                        
                    # Resolve Target
                    if isinstance(tgt_key, str): # SuperNode ID
                        c = s_offsets.get(tgt_key, -1)
                    else: # Atomic ID (Int)
                        c = atomic_offsets.get(tgt_key, -1)
                        
                    if r != -1 and c != -1:
                        # Normalize macro edge weight similar to atomic? 
                        # Or just pass raw. Usually raw.
                        val = w.item() if torch.is_tensor(w) else w
                        new_adj[r, c] += val
                        
                filtered_edges = new_adj

        mapped_transition = None
        if self.last_transition:
            s, t = self.last_transition
            mapped_transition = {
                "source": str(self.get_visual_id(s)),
                "target": str(self.get_visual_id(t))
            }

        return kept_indices, filtered_nodes, filtered_edges, final_labels, filtered_counts, self.colors, self.layout, filtered_salience, filtered_baked, mapped_transition

    def get_sn_id_from_visual_id(self, visual_id):
        """Map negative visual ID back to internal string sn_id."""
        try:
            vid = int(visual_id)
            for sn_id, sn in self.supernodes.items():
                if sn.get('visual_id') == vid:
                    return sn_id
        except: pass
        return None

    def dissolve_supernode(self, sn_id):
        """Disassemble a supernode back into its atomic children."""
        # Handle visual IDs (negative ints)
        actual_sn_id = sn_id
        if not isinstance(sn_id, str):
            actual_sn_id = self.get_sn_id_from_visual_id(sn_id)
        elif sn_id not in self.supernodes:
            # Maybe it's a string representation of a negative int
            try:
                vid = int(sn_id)
                if vid < 0:
                    actual_sn_id = self.get_sn_id_from_visual_id(vid)
            except: pass

        if not actual_sn_id or actual_sn_id not in self.supernodes:
            print(f"Memory Warning: Cannot dissolve supernode {sn_id} (Not found).")
            return False
        
        sn = self.supernodes[actual_sn_id]
        child_indices = sn['children']
        
        # 1. Unhide children
        self.hidden_nodes[child_indices] = False
        
        # 2. Remove from map
        for cid in child_indices:
            cid_item = cid.item() if hasattr(cid, 'item') else cid
            if cid_item in self.node_supernode_map:
                del self.node_supernode_map[cid_item]
        
        # 3. Delete supernode
        del self.supernodes[actual_sn_id]
        
        # 4. Cleanup edges
        to_del = [k for k in self.super_edges if k[0] == actual_sn_id or k[1] == actual_sn_id]
        for k in to_del:
            del self.super_edges[k]
            
        print(f"Memory: Dissolved supernode {actual_sn_id}.")
        return True

    def dissolve_all_supernodes(self):
        """Total disassembly of all supernodes."""
        self.hidden_nodes.zero_()
        self.node_supernode_map = {}
        self.supernodes = {}
        self.super_edges = {}
        print("Memory: All supernodes dissolved.")
        return True

        


    def get_state(self):
        """
        Export full internal state for saving.
        """
        serialized_supernodes = self._serialize_supernodes()
        return {
            "node_count": self.node_count,
            "nodes": self.nodes[:self.node_count].cpu().tolist(),
            "edges": self.edges[:self.node_count, :self.node_count].cpu().tolist(),
            "trigger_counts": self.trigger_counts[:self.node_count].cpu().tolist(),
            "last_trigger_times": self.last_trigger_times[:self.node_count].cpu().tolist(),
            "creation_times": self.creation_times[:self.node_count].cpu().tolist(),
            "salience": self.salience[:self.node_count].cpu().tolist(),
            "baked": self.baked[:self.node_count].cpu().tolist(),
            "consistency_accumulator": self.consistency_accumulator[:self.node_count].cpu().tolist(),
            "consistency_counts": self.consistency_counts[:self.node_count].cpu().tolist(),
            "labels": self.labels,
            "label_history": self.label_history,
            "colors": self.colors,
            "layout": self.layout,
            "last_node_idx": self.last_node_idx,
            "baking_threshold": self.baking_threshold,
            "supernodes": serialized_supernodes,
            "super_edges_keys": [list(k) for k in self.super_edges.keys()], 
            "super_edges_values": [float(v) if not torch.is_tensor(v) else float(v.item()) for v in self.super_edges.values()],
            "node_supernode_map": self.node_supernode_map,
            "hidden_nodes": self.hidden_nodes[:self.node_count].cpu().tolist()
        }

    def _serialize_supernodes(self):
        """Helper to make supernodes JSON serializable."""
        serializable = {}
        for sn_id, sn in self.supernodes.items():
            serializable[sn_id] = {
                "id": sn['id'],
                "visual_id": sn['visual_id'],
                "label": sn['label'],
                "centroid": sn['centroid'].cpu().tolist(),
                "children": sn['children'].cpu().tolist() if torch.is_tensor(sn['children']) else sn['children']
            }
        return serializable

    def set_state(self, data):
        """
        Restore internal state from saved data.
        """
        self.reset()
        
        try:
            nc = data.get('node_count', 0)
            if nc > self.max_nodes:
                print(f"Warning: Saved graph has {nc} nodes, but max is {self.max_nodes}. Truncating.")
                nc = self.max_nodes
                
            self.node_count = nc
            self.last_node_idx = data.get('last_node_idx')
            self.baking_threshold = data.get('baking_threshold', 100)
            
            # Load Labels/Colors with mixed key support (Atomic=int, Super=visual_id or sn_id)
            self.labels = {}
            for k, v in data.get('labels', {}).items():
                if k in self.labels: del self.labels[k]
                # Try to convert key to int if it looks like one
                try:
                    k_int = int(k)
                    self.labels[k_int] = v
                except:
                    self.labels[k] = v
            
            # Load Label History
            self.label_history = {}
            for k, v in data.get('label_history', {}).items():
                try:
                    k_int = int(k)
                    self.label_history[k_int] = v
                except:
                    self.label_history[k] = v
            
            # Restore Colors (same int key logic)
            # Restore Colors (same int key logic)
            self.colors = {}
            for k, v in data.get('colors', {}).items():
                if k in self.colors: del self.colors[k]
                try:
                    k_int = int(k)
                    self.colors[k_int] = v
                except:
                    self.colors[k] = v
                    
            # Restore Tensors
            if nc > 0:
                if 'nodes' in data:
                    self.nodes[:nc] = torch.tensor(data['nodes'], device=self.device)
                if 'edges' in data:
                    self.edges[:nc, :nc] = torch.tensor(data['edges'], device=self.device)
                if 'trigger_counts' in data:
                    self.trigger_counts[:nc] = torch.tensor(data['trigger_counts'], device=self.device)
                if 'last_trigger_times' in data:
                    self.last_trigger_times[:nc] = torch.tensor(data['last_trigger_times'], device=self.device)
                if 'creation_times' in data:
                    self.creation_times[:nc] = torch.tensor(data['creation_times'], device=self.device)
                if 'salience' in data:
                    self.salience[:nc] = torch.tensor(data['salience'], device=self.device)
                if 'baked' in data:
                    self.baked[:nc] = torch.tensor(data['baked'], device=self.device, dtype=torch.bool)
                if 'consistency_accumulator' in data:
                    self.consistency_accumulator[:nc] = torch.tensor(data['consistency_accumulator'], device=self.device)
                if 'consistency_counts' in data:
                    self.consistency_counts[:nc] = torch.tensor(data['consistency_counts'], device=self.device)
                if 'hidden_nodes' in data:
                    self.hidden_nodes[:nc] = torch.tensor(data['hidden_nodes'], device=self.device, dtype=torch.bool)
                
            # Restore Layout
            self.layout = data.get('layout', {})
            
            # Restore SuperNodes (Complex)
            # Re-initialize empty structures first
            self.supernodes = {}
            self.node_supernode_map = {}
            self.hidden_nodes.zero_()
            self.super_edges = {}

            if 'supernodes' in data:
                # 1. Restore the dict structure, converting lists back to tensors where needed
                raw_sn = data['supernodes']
                for sn_id, sn_data in raw_sn.items():
                    self.supernodes[sn_id] = {
                        "id": sn_data['id'],
                        "visual_id": sn_data['visual_id'],
                        "label": sn_data['label'],
                        "centroid": torch.tensor(sn_data['centroid'], device=self.device),
                        "children": torch.tensor(sn_data['children'], device=self.device, dtype=torch.long)
                    }
                    
                # 2. Restore Mapping (JSON keys are always strings, need ints)
                raw_map = data.get('node_supernode_map', {})
                self.node_supernode_map = {int(k): v for k, v in raw_map.items()}
                
                # 4. Restore SuperEdges
                keys = data.get('super_edges_keys', [])
                vals = data.get('super_edges_values', [])
                if len(keys) == len(vals):
                    for k_list, v in zip(keys, vals):
                        # Config: Key is [src, tgt]. Src/Tgt can be string (SN) or int (Atomic)
                        src = k_list[0]
                        tgt = k_list[1]
                        # Our memory.super_edges logic handles keys as (str, str), (str, int) etc.
                        self.super_edges[(src, tgt)] = v
            
            # Rebuild Label Mask
            for idx in self.labels:
                 if isinstance(idx, int) and 0 <= idx < self.max_nodes:
                     self.label_mask[idx] = True
            
            return True
        except Exception as e:
            print(f"Memory Error during set_state: {e}")
            import traceback
            traceback.print_exc()
            return False
