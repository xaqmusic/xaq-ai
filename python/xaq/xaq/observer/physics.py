
import torch
import numpy as np
import time

class GraphPhysics:
    """
    GPU-accelerated Semantic Physics Engine.
    Calculates forces based on:
    1. Semantic Attraction: Nodes with high cosine similarity attract.
    2. Repulsion: Nodes repel each other (Coulomb's Law-like).
    3. Clustering: Repulsion is disabled if similarity is high (allowing clusters).
    """
    def __init__(self, device='cuda' if torch.cuda.is_available() else 'cpu'):
        self.device = device
        self.clustering_threshold = 0.6
        self.attraction_strength = 0.5
        # Physics constants calibrated for PCA coordinate space ±20
        # (v3 GNG prototype PCA is scaled to ±20 in adapter.py).
        # v2 had ±30 PCA and these large values worked; with ±1 PCA
        # every node was inside the collision radius of every other node → blob.
        self.repulsion_strength = 8.0    # was 50 — semantic repulsion, gentle spread
        self.gravity = 0.0
        self.dt = 0.1
        self.damping = 0.75              # was 0.8 — slightly more damped for stability

        self.layout_mode = 'physics'
        self.horizon = 1
        self.hebbian_strength = 0.3      # was 0.001 — actually pulls connected nodes together
        self.pca_strength = 2.0
        self.collision_strength = 8.0    # was 100 — close-range push between overlapping nodes
        self.max_speed = 3.0             # was 10 — calmer movement 
        self.ids = []


        self.positions = None # (N, 2)
        self.pca_targets = None # (N, 2) Target positions from PCA
        self.velocities = None # (N, 2)
        self.embeddings = None # (N, D)
        self.ids = []
        self.adj_matrix = None # (N, N) Adjacency
        self.anchored_ids = set() # Set of Node IDs that are locked

    def sync_nodes(self, node_list, indices=None, embeddings_matrix=None, pca_coords=None):
        """
        Syncs the internal physics state with the Brain's node list.
        Args:
            node_list: List of dicts with 'id', optionally 'x', 'y'
            indices: Optional (N,) numpy array of int IDs (from Brain)
            embeddings_matrix: Optional (N, D) numpy array of embeddings
            pca_coords: Optional (N, 2) numpy array of PCA coordinates
        """
        if not node_list:
            return
            
        t_start = time.time()
        sub_times = {}

        # 1. Prepare New IDs
        t0 = time.time()
        if indices is not None:
             new_ids_int = indices.astype(np.int64)
             new_ids_str = [str(int(i)) for i in new_ids_int]
        else:
             def get_id(node):
                 if isinstance(node, dict): return node.get('id')
                 return getattr(node, 'id', None)
             new_ids_int = np.array([int(get_id(n)) for n in node_list if get_id(n) is not None], dtype=np.int64)
             new_ids_str = [str(i) for i in new_ids_int]
        n = len(new_ids_int)
        sub_times['ids'] = (time.time() - t0) * 1000
        
        # 2. Maintain internal int IDs
        t0 = time.time()
        if not hasattr(self, 'id_indices') or self.id_indices is None or len(self.id_indices) != len(self.ids):
             try:
                 self.id_indices = np.array([int(i) for i in self.ids], dtype=np.int64)
             except:
                 self.id_indices = np.zeros(len(self.ids), dtype=np.int64)
        sub_times['id_indices_sync'] = (time.time() - t0) * 1000

        # 3. Vectorized Remapping Logic
        t0 = time.time()
        new_pos = torch.zeros((n, 2), device=self.device)
        new_vel = torch.zeros((n, 2), device=self.device)
        new_pca = torch.zeros((n, 2), device=self.device)
        Needs_random_init = np.ones(n, dtype=bool)

        # Correct & Fast Remapping via intersect1d
        ids_common, idx_old, idx_new = np.intersect1d(self.id_indices, new_ids_int, return_indices=True)
        
        if len(ids_common) > 0:
            t_old = torch.as_tensor(idx_old, device=self.device)
            t_new = torch.as_tensor(idx_new, device=self.device)
            
            if self.positions is not None:
                new_pos[t_new] = self.positions[t_old]
                new_vel[t_new] = self.velocities[t_old]
                if self.pca_targets is not None:
                     n_old = self.pca_targets.shape[0]
                     valid = idx_old < n_old
                     if valid.any():
                         new_pca[t_new[valid]] = self.pca_targets[t_old[valid]]
            
            Needs_random_init[idx_new] = False
        sub_times['remapping'] = (time.time() - t0) * 1000

        # 4. Handle PCA Coordinate Injection (Vectorized)
        t0 = time.time()
        is_static = getattr(self, 'layout_mode', 'physics') == 'static'
        
        if pca_coords is not None:
             pca_torch = torch.from_numpy(pca_coords).to(self.device).float()
             # PCA targets are always updated from the payload
             new_pca = pca_torch 
             
             # If static or purely new node, force PCA position
             mask_init = torch.from_numpy(Needs_random_init).to(self.device)
             if is_static:
                  new_pos = new_pca.clone()
                  Needs_random_init[:] = False
             elif mask_init.any():
                  new_pos[mask_init] = new_pca[mask_init]
                  Needs_random_init[Needs_random_init] = False
        
        # 4b. Fallback for manual overrides in node_list (rare)
        # Note: We only do this if node_list actually has x/y to avoid 500ms stall
        # In practice, get_visualization_data only puts x/y if it's new or static.
        # But we now have bulk PCA, so we can skip this loop if bulk is present.
        if pca_coords is None:
            for i, node in enumerate(node_list):
                if not isinstance(node, dict): continue
                nx, ny = node.get('x'), node.get('y')
                if nx is not None and ny is not None:
                    new_pca[i, 0] = nx
                    new_pca[i, 1] = ny
                    if Needs_random_init[i] or is_static:
                         new_pos[i, 0] = nx
                         new_pos[i, 1] = ny
                         Needs_random_init[i] = False
        sub_times['pca_injection'] = (time.time() - t0) * 1000

        # 5. Fill remaining positions with random
        t0 = time.time()
        if Needs_random_init.any():
            count = Needs_random_init.sum()
            # Spawn within ±5 of origin — inside the ±20 PCA space, not outside it.
            # Old value of 50 sent new nodes far beyond the coordinate space.
            new_pos[Needs_random_init] = torch.randn((count, 2), device=self.device) * 5.0
        sub_times['random_init'] = (time.time() - t0) * 1000

        # 6. Bulk Sync Embeddings
        t0 = time.time()
        if embeddings_matrix is not None:
             if isinstance(embeddings_matrix, np.ndarray):
                 self.embeddings = torch.from_numpy(embeddings_matrix).to(self.device).float()
             else:
                 self.embeddings = embeddings_matrix.to(self.device).float()
        else:
             self.embeddings = torch.zeros((n, 1), device=self.device)
        sub_times['emb_sync'] = (time.time() - t0) * 1000

        self.positions = new_pos
        self.pca_targets = new_pca
        self.velocities = new_vel
        self.ids = new_ids_str
        self.id_indices = new_ids_int
        
        self.last_sync_profile = sub_times

        
        # Resize Adjacency if needed
        if self.adj_matrix is None or self.adj_matrix.shape[0] != n:
             self.adj_matrix = torch.zeros((n, n), device=self.device)

    def update_edges(self, edges):
        """
        Ingest edges from Brain.
        edges: Adjacency Matrix (Numpy Array or Torch Tensor)
        """
        if not self.ids: return
        
        n = len(self.ids)
        
        # If passed nothing or empty list, clear
        if edges is None or len(edges) == 0:
             if self.adj_matrix is not None: self.adj_matrix.zero_()
             return

        # If it's the old list format (legacy safety), skip or warn
        if isinstance(edges, list):
             # print("Physics: Warning - Received legacy edge list. Ignoring.")
             return
             
        # It's an array/tensor
        if isinstance(edges, np.ndarray):
            t_edges = torch.from_numpy(edges).to(self.device).float()
        else:
            t_edges = edges.to(self.device).float()
            
        # Ensure shape match (Brain might have N nodes, we might have synced N nodes)
        # They should match if synced correctly.
        if t_edges.shape[0] == n and t_edges.shape[1] == n:
            self.adj_matrix = t_edges
        else:
            # Resize? Or just zero?
            # If shape mismatch, likely sync lag. Safe to zero or ignore.
            pass
                
    def set_anchor(self, node_id, is_anchored):
        """Enable or disable anchoring for a node."""
        if is_anchored:
            self.anchored_ids.add(node_id)
        else:
            self.anchored_ids.discard(node_id)

    @torch.no_grad()
    def step(self):
        """
        Performs one physics step (Runge-Kutta or Euler).
        Computes forces and updates positions.
        """
        t0 = time.time()
        if self.positions is None or len(self.positions) == 0:
            return [], np.zeros((0, 2))

        n = self.positions.shape[0]
        
        # Static/PCA Mode: Skip Force Calculation
        if self.layout_mode == 'static':
            # Just export current positions
            return self.ids, self.positions.cpu().numpy()

        if n == 1:
            return self.ids, self.positions.cpu().numpy()

        pos = self.positions
        vel = self.velocities

        # 1. Compute Semantic Similarity Grid (N, N)
        t_pre_mm = time.time()
        if self.embeddings.ndim == 2:
            emb_norm = torch.nn.functional.normalize(self.embeddings, p=2, dim=1)
            sim_matrix = torch.mm(emb_norm, emb_norm.t()) # (N, N) Cosine Sim
        else:
            sim_matrix = torch.zeros((n, n), device=self.device)
        t_post_mm = time.time()

        # 2. Compute Distances
        diff = pos.unsqueeze(1) - pos.unsqueeze(0) # Vectors from j to i
        dist_sq = diff.pow(2).sum(-1) # (N, N) distance squared
        dist = dist_sq.sqrt() + 1e-6 
        dist_sq_clamped = torch.clamp(dist_sq, min=0.01)
        t_dist = time.time()

        # 3. Forces
        
        # A. Priority 1: Hebbian Attraction (Horizon)
        # F = k * (pos_j - pos_i) for connected nodes.
        # We compute connectivity up to Horizon.
        
        # Base S (Direct)
        S = self.adj_matrix
        if S is None: S = torch.zeros((n, n), device=self.device)
        
        # Horizon Expansion
        # If Horizon > 1, we want connectivity C = S + gamma*S^2 + ...
        # For efficiency, we just do matrix multiplications on demand or specialized logic.
        # Simple Approach: A_k = S; Accumulate Forces for k=1..H
        
        f_hebbian = torch.zeros_like(pos)
        
        current_adj = S
        for h in range(1, self.horizon + 1):
            # Calculate weights for this level
            # Using current_adj directly (weights) decayed by h
            W = current_adj * (1.0 / h)
            
            # Diagonal should be 0 (no self-attraction)
            W.fill_diagonal_(0)
            
            if W.sum() > 0:
                # Restoring additive logic but keeping it cleaner
                neighbor_pull = torch.mm(W, pos) # (N, 2)
                sum_W = W.sum(dim=1, keepdim=True) # (N, 1)
                
                # F = Sum_j( W_ij * (p_j - p_i) ) = Sum(W_ij * p_j) - p_i * Sum(W_ij)
                f_hebbian += (neighbor_pull - pos * sum_W) * self.hebbian_strength
            
            if self.horizon > h:
                # Expand horizon. Binarize intermediate step to identify reachability, 
                # but we'll use weights for the final level of W.
                # Actually, let's just keep the multiplication of weights for consistency.
                current_adj = torch.mm(current_adj, S)
                # Clamp to avoid numerical explosion in dense graphs
                current_adj = torch.clamp(current_adj, max=10.0)
        
        f_total = f_hebbian

        # B. Priority 2: PCA Attraction (Global Layout)
        # Pull towards original PCA coords (self.pca_targets)
        # F_pca = k * (Target - Pos)
        if self.pca_targets is not None:
             f_pca = (self.pca_targets - pos) * self.pca_strength
             f_total += f_pca

        # C. Priority 3: Collision Repulsion (Short Range)
        # Coulomb Repulsion but only when very close
        
        # dist_sq already computed.
        # Repel if dist < radius
        # IDs < 0 are supernodes
        ids_pt = torch.from_numpy(self.id_indices).to(self.device).float()
        is_super = (ids_pt < 0).float() # (N,)
        
        # Radii: collision zone per node. Normal nodes get 3 units (in ±20 space
        # that's ~7% of the coordinate range — prevents overlap without blanketing
        # the whole graph). Supernodes get 3× that.
        node_radii = 3.0 + is_super * 7.0 # (N,)
        
        # Pairwise max radius: if either node is a supernode, they repel at a larger range
        R_ij = torch.max(node_radii.unsqueeze(1), node_radii.unsqueeze(0)) # (N, N)
        
        # Handle nodes at the same location by adding tiny random jitter
        is_zero = (dist < 1e-6).unsqueeze(-1)
        noise = torch.randn_like(diff) * 1e-4
        diff_effective = diff + noise * is_zero
        dist_effective = diff_effective.pow(2).sum(-1).sqrt() + 1e-8
        
        # Soft Repulsion: Force fades as d approaches R
        # This prevents the binary "kick" at the boundary
        dist_norm = dist_effective.unsqueeze(-1) / (R_ij.unsqueeze(-1) + 1e-9)
        falloff = torch.clamp(1.0 - dist_norm, min=0.0) # (N, N, 1)
        
        # mag = Strength / (d + eps)
        mag = self.collision_strength / (dist_effective.unsqueeze(-1) + 0.01)
        
        # Direction AWAY from neighbor j to node I
        dir_norm = diff_effective / dist_effective.unsqueeze(-1)
        
        f_collide = dir_norm * mag * falloff
        f_collide_sum = f_collide.sum(dim=1)
        f_total += f_collide_sum

        # D. Semantic Repulsion (Original, lower priority/strength)
        sim_mask = (sim_matrix < self.clustering_threshold).float()
        # diff is (pos_i - pos_j), so this pushes AWAY
        f_sem_repel = self.repulsion_strength * 0.5 * (1.0 - sim_matrix) / dist_sq_clamped * sim_mask
        f_sem_repel.fill_diagonal_(0)
        f_total += (f_sem_repel.unsqueeze(-1) * diff).sum(dim=1)

        t_forces = time.time()
        # 4. Update Dynamics (Euler Integration)
        f_total = torch.clamp(f_total, min=-50.0, max=50.0) # Tighten clamping
        self.velocities = (vel + f_total * self.dt) * self.damping
        
        # Velocity Capping
        speed = self.velocities.norm(dim=1, keepdim=True)
        is_overspeed = (speed > self.max_speed).float()
        self.velocities = self.velocities * (1.0 - is_overspeed) + (self.velocities / (speed + 1e-6) * self.max_speed) * is_overspeed
        
        if hasattr(self, 'anchored_ids') and self.anchored_ids:
            anchored_mask = torch.zeros(n, dtype=torch.bool, device=self.device)
            for i, uid in enumerate(self.ids):
                if uid in self.anchored_ids: anchored_mask[i] = True
            self.velocities[anchored_mask] = 0
            
        self.positions += self.velocities * self.dt
        t_integ = time.time()

        # 5. Export (GPU -> CPU)
        ids_out = self.ids
        pos_out = self.positions.cpu().numpy()
        t_cpu = time.time()
        
        # Save Profile
        self.last_step_profile = {
            'mm': (t_post_mm - t_pre_mm) * 1000,
            'dist': (t_dist - t_post_mm) * 1000,
            'forces': (t_forces - t_dist) * 1000,
            'integ': (t_integ - t_forces) * 1000,
            'cpu': (t_cpu - t_integ) * 1000,
            'total': (t_cpu - t0) * 1000,
            'n': n
        }
        
        return ids_out, pos_out
