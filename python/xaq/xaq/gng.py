"""
Growing Neural Gas (GNG) — from-scratch implementation for AMI-Ogma v3.

Reference: Fritzke (1995) "A Growing Neural Gas Network Learns Topologies"

Role in the pipeline:
    encoder output (128D)
        → GNG.step(x)
        → (winner_id, quantization_error, prototype_vector)

The GNG maps the topology of the latent space. It solves three problems:
    1. Spatial denoising   — clusters noisy encoder outputs into stable node IDs
    2. Adaptive resolution — edge-splitting inserts nodes where TLE is highest
    3. Manifold mapping    — GNG-distance spikes when input "teleports" (frame skip)

Intended use: one GNG instance per EPM modality.

Design notes:
    - Node IDs are monotonically increasing and never reused, matching the C++ pattern.
    - Internal storage uses compact numpy arrays indexed by position; a separate id_map
      maps stable node IDs → array positions.
    - Edges are stored as {frozenset({a_pos, b_pos}): age} where a_pos/b_pos are
      array positions (not IDs), keeping the hot path O(degree).
    - All state can be serialised with to_dict() / from_dict() for checkpointing.
"""

import numpy as np
from collections import defaultdict, deque
from typing import Tuple, Optional, Dict, Any


class GNG:
    """
    Growing Neural Gas.

    Parameters
    ----------
    dim : int
        Dimensionality of the input space (must match encoder output dim).
    max_nodes : int
        Hard cap on the number of nodes. Growth stops when reached.
    epsilon_b : float
        Winner learning rate — how far the winner moves toward the input.
    epsilon_n : float
        Neighbour learning rate — how far the winner's neighbours move.
    max_age : int
        Edges older than this are removed. Increase for slower topology forgetting.
    lambda_new : int
        Number of steps between node-insertion events.
    alpha : float
        Error reduction factor applied to the nodes involved in a split.
    beta : float
        Global error decay factor applied every step. (d in Fritzke's notation)
    baking_threshold : int
        Minimum visits for a node to be considered "crystallised".
    """

    def __init__(self,
                 dim: int = 128,
                 max_nodes: int = 2000,
                 epsilon_b: float = 0.05,
                 epsilon_n: float = 0.003,
                 max_age: int = 88,
                 lambda_new: int = 25,
                 alpha: float = 0.5,
                 beta: float = 0.0005,
                 baking_threshold: int = 100,
                 min_insertion_error: float = 0.02,
                 freeze_min_insertion_error: bool = False):
        self.dim = dim
        self.max_nodes = max_nodes
        self.epsilon_b = epsilon_b
        self.epsilon_n = epsilon_n
        self.max_age = max_age
        self.lambda_new = lambda_new
        self.alpha = alpha
        self.beta = beta
        self.baking_threshold = baking_threshold

        # --- Stale-prune controls (wired to UI decay sliders) ---
        # stale_prune_enabled: when False, _prune_stale_unbaked() is a no-op.
        #   Controlled by the "Active Decay" checkbox.  Disable when you want
        #   nodes to survive indefinitely (e.g. between breakout episodes).
        # stale_window_factor: stale window = baking_threshold × factor (steps).
        #   Controlled by the "Decay (s)" slider (range 1–500; divide by 10
        #   to get the factor).  Default slider=60 → factor=6 → 6×threshold steps,
        #   more generous than the hardcoded 3× to suit video/game modalities.
        self.stale_prune_enabled: bool  = True
        self.stale_window_factor: float = 12000.0  # absolute steps (~400s at 30fps)

        # Absolute threshold on per-visit **squared** quantization error.
        # A new node is inserted only if the worst node's mean squared error
        # per visit exceeds this value.
        #
        # Calibrated to soft-normalised encoder output (energies / (||e|| + eps)):
        #   - Converged cluster (d1 ≈ 0.03) → per-visit error ≈ 0.001  (below)
        #   - Uncovered region (d1 ≈ 0.15–0.5) → per-visit error ≈ 0.02–0.25 (above)
        #
        # Lowered from 0.1 → 0.02 to give finer concept resolution for voice/audio.
        # This implements the homeokinetic principle from the patent: growth is
        # driven by sustained high TLE (surprise) and suppressed once a region
        # has been adequately mapped (low TLE / stable resonance).
        self.min_insertion_error = min_insertion_error

        # Ecological-agency self-tuning: the GNG is an agent whose reward is
        # good novelty detection. It tunes its own insertion threshold from
        # the 30th-percentile of its own recent TLE distribution, so the
        # split between "stable / don't grow" and "surprising / grow" tracks
        # whatever input density the environment actually presents.
        # freeze_min_insertion_error=True restores the old fixed-threshold
        # behaviour for debugging / regression runs.
        self.freeze_min_insertion_error = bool(freeze_min_insertion_error)
        self._tle_sq_history: deque = deque(maxlen=1000)
        self._tle_warmup: int = 100

        # Running mean **squared** quantization error (EMA over all steps).
        # Diagnostic / UI display only.
        self.running_mean_error: float = 1.0

        # --- node storage (compact numpy arrays, row per node) ---
        # Allocate with headroom; grow dynamically if needed.
        self._capacity = min(256, max_nodes)
        self._prototypes = np.zeros((self._capacity, dim), dtype=np.float32)
        self._errors    = np.zeros(self._capacity, dtype=np.float64)
        # Per-node short-term EMA of squared quantization error (α=0.1).
        # Reflects RECENT error so the insertion guard reacts to current
        # surprise rather than stale accumulated history.
        self._ema_errors = np.zeros(self._capacity, dtype=np.float64)
        self._visits    = np.zeros(self._capacity, dtype=np.int64)
        self._alive     = np.zeros(self._capacity, dtype=bool)
        # Last step at which each node was the winner.
        # Used to prune non-baked nodes that stop attracting input —
        # a prerequisite for clean EPM maturity / Mitosis Gatekeeper signals.
        self._last_visited_step = np.zeros(self._capacity, dtype=np.int64)

        # Stable ID bookkeeping
        self._id_to_pos: Dict[int, int]  = {}   # stable node ID → array pos
        self._pos_to_id: Dict[int, int]  = {}   # array pos → stable node ID
        self._next_id:   int             = 0

        # Edge storage: frozenset({pos_a, pos_b}) → age
        self._edges: Dict[frozenset, int] = {}
        # Adjacency: pos → set of neighbouring positions (for fast lookup)
        self._adj: Dict[int, set] = defaultdict(set)

        # Step counter
        self._step = 0

        # Recent node ID history (for history_trace in RealityToken)
        self._history: list = []
        self._history_maxlen = 32

        # Initialise with 2 random prototypes (will be replaced on first 2 inputs)
        self._bootstrapped = False
        self._bootstrap_buf: list = []
        self._last_x: Optional[np.ndarray] = None  # most recent input, for insertion placement

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def node_count(self) -> int:
        return int(self._alive.sum())

    @property
    def baked_count(self) -> int:
        """Number of crystallised (baked) nodes."""
        alive = np.where(self._alive)[0]
        return int((self._visits[alive] >= self.baking_threshold).sum())

    @property
    def crystallization_ratio(self) -> float:
        """
        Fraction of alive nodes that are crystallised.

        Ranges 0.0 → 1.0.  The Mitosis Gatekeeper uses this alongside
        sustained low TLE to declare an EPM 'mature'.  A mature EPM with
        a spike in context_novelty (min distance from current input to any
        baked prototype) signals a context change → spawn a new EPM.
        """
        n = self.node_count
        if n == 0:
            return 0.0
        return self.baked_count / n

    def context_novelty(self, x: np.ndarray) -> float:
        """
        Minimum Euclidean distance from x to any BAKED prototype.

        This is the signal the Mitosis Gatekeeper watches:
          - Low (< ~0.15): input fits within the crystallised context
          - High (> ~0.35): input is genuinely outside the EPM's knowledge
            → candidate for spawning a new specialist EPM

        Returns inf if no baked nodes exist yet (EPM still growing).
        """
        alive = np.where(self._alive)[0]
        baked = alive[self._visits[alive] >= self.baking_threshold]
        if len(baked) == 0:
            return float('inf')
        diffs = self._prototypes[baked] - x
        dists = np.sqrt((diffs ** 2).sum(axis=1))
        return float(dists.min())

    def step(self, x: np.ndarray) -> Tuple[int, float]:
        """
        Process one input vector. Updates topology and returns the winner.

        Args:
            x: np.ndarray shape (dim,)

        Returns:
            (winner_node_id, quantization_error)
            winner_node_id — stable integer ID of the winning prototype
            quantization_error — Euclidean distance from x to the winner prototype
        """
        x = np.asarray(x, dtype=np.float32)

        # --- bootstrap: seed first 2 nodes from real inputs ---
        if not self._bootstrapped:
            self._bootstrap_buf.append(x.copy())
            if len(self._bootstrap_buf) >= 2:
                self._add_node(self._bootstrap_buf[0])
                self._add_node(self._bootstrap_buf[1])
                self._bootstrapped = True
                self._bootstrap_buf.clear()
            # Return dummy values during bootstrap
            return 0, 0.0

        self._step += 1
        self._last_x = x  # retained for _insert_node() placement

        # 1. Find winner (s1) and runner-up (s2)
        s1_pos, s2_pos, d1, d2 = self._find_two_nearest(x)

        s1_id = self._pos_to_id[s1_pos]

        # 2. Accumulate error at winner and update short-term EMA
        self._errors[s1_pos] += d1 * d1
        self._ema_errors[s1_pos] = 0.9 * self._ema_errors[s1_pos] + 0.1 * (d1 * d1)

        # 3. Move winner toward input — baked prototypes are frozen.
        # Stability-dampened learning rate mirrors v2: nodes decelerate into
        # crystallisation rather than snapping from fully plastic to fully frozen.
        # effective_ε = ε_b * (1 − 0.9 * stability), so at baking_threshold visits
        # the rate has dropped to 0.1 * ε_b — prototype is nearly settled before lock.
        if self._visits[s1_pos] < self.baking_threshold:
            stability = self._visits[s1_pos] / self.baking_threshold   # 0.0 → <1.0
            effective_eb = self.epsilon_b * (1.0 - 0.9 * stability)
            self._prototypes[s1_pos] += effective_eb * (x - self._prototypes[s1_pos])

        # 4. Move neighbours of winner toward input — skip baked neighbours.
        # Same stability dampening applied per-neighbour using their own visit count.
        for nb_pos in list(self._adj[s1_pos]):
            if self._alive[nb_pos] and self._visits[nb_pos] < self.baking_threshold:
                nb_stability = self._visits[nb_pos] / self.baking_threshold
                effective_en = self.epsilon_n * (1.0 - 0.9 * nb_stability)
                self._prototypes[nb_pos] += effective_en * (x - self._prototypes[nb_pos])

        # 5. Update / create edge between s1 and s2; increment ages of s1's edges.
        #    Edges touching a baked node are exempt — they form the stable skeleton
        #    of crystallized memory and must not decay away.
        s1_baked = self._visits[s1_pos] >= self.baking_threshold
        edges_to_remove = []
        for edge_key in list(self._edges):
            if s1_pos in edge_key:
                other = next(p for p in edge_key if p != s1_pos)
                nb_baked = (self._alive[other] and
                            self._visits[other] >= self.baking_threshold)
                if s1_baked or nb_baked:
                    continue  # baked edge — never age out
                self._edges[edge_key] += 1
                if self._edges[edge_key] > self.max_age:
                    edges_to_remove.append(edge_key)

        for ek in edges_to_remove:
            self._remove_edge(ek)

        # Refresh / create s1–s2 edge
        self._add_edge(s1_pos, s2_pos)

        # 6. Remove isolated nodes (no neighbours)
        self._remove_isolated()

        # 7. Periodic node insertion + non-baked decay pruning
        if self._step % self.lambda_new == 0:
            if self.node_count < self.max_nodes:
                self._insert_node()
            self._prune_stale_unbaked()

        # 8. Global error decay
        self._errors[self._alive] *= (1.0 - self.beta)

        # 8a. Record squared TLE sample for adaptive threshold (Step 2 pivot).
        self._tle_sq_history.append(d1 * d1)

        # 8b. Track running mean **squared** quantization error (EMA, α=0.05).
        # Squared units match _errors[] accumulation so the surprise guard in
        # _insert_node() can compare them directly.
        self.running_mean_error = 0.95 * self.running_mean_error + 0.05 * (d1 * d1)

        # 9. Update visit count, last-visited step, and history
        self._visits[s1_pos] += 1
        self._last_visited_step[s1_pos] = self._step
        self._history.append(s1_id)
        if len(self._history) > self._history_maxlen:
            self._history.pop(0)

        # 10. Consistency gate — fires exactly when a node crosses baking_threshold.
        #
        # Mirrors v2's two-gate baking system:
        #   Gate 1 (Frequency): visits >= baking_threshold          ← just passed
        #   Gate 2 (Consistency): ema_error < min_insertion_error   ← evaluated here
        #
        # ema_error at this moment reflects the short-term average squared distance
        # of recent inputs to this prototype.  A low value means the node covers a
        # tight, coherent cluster — a real concept.  A high value means it's still
        # acting as a broad catch-all and should not be frozen yet.
        #
        # On failure (demotion):
        #   visits knocked back to baking_threshold − 3  → 3 more visits to retry
        #   ema_error halved                             → fresh evaluation window
        #   accumulated _errors halved                  → reduces insertion pressure
        #     from this node so a new node can resolve the ambiguous region
        #
        # The node stays alive and plastic; stability dampening keeps its learning
        # rate low (it's near-baked) so the prototype settles during the retry window.
        if self._visits[s1_pos] == self.baking_threshold:
            if self._ema_errors[s1_pos] >= self.min_insertion_error:
                # Demotion — concept not yet tight enough
                self._visits[s1_pos] = max(0, self.baking_threshold - 3)
                self._ema_errors[s1_pos] *= 0.5
                self._errors[s1_pos] *= 0.5

        return s1_id, float(d1)

    def get_history(self, n: int = 5) -> list:
        """Return the last n visited node IDs (oldest first)."""
        return self._history[-n:]

    def get_prototype(self, node_id: int) -> Optional[np.ndarray]:
        """Return the prototype vector for a given stable node ID, or None."""
        pos = self._id_to_pos.get(node_id)
        if pos is None or not self._alive[pos]:
            return None
        return self._prototypes[pos].copy()

    def get_visit_count(self, node_id: int) -> int:
        pos = self._id_to_pos.get(node_id)
        if pos is None:
            return 0
        return int(self._visits[pos])

    def is_crystallised(self, node_id: int) -> bool:
        """True if node has been visited enough to be considered stable."""
        return self.get_visit_count(node_id) >= self.baking_threshold

    def prune_unbaked(self) -> int:
        """
        Remove all nodes that have not yet reached baking_threshold visits.
        Always keeps at least 2 nodes so the GNG can continue operating.

        Returns the number of nodes pruned.
        """
        if self.node_count <= 2:
            return 0
        pruned = 0
        for node_id, pos in list(self._id_to_pos.items()):
            if not self._alive[pos]:
                continue
            if self._visits[pos] < self.baking_threshold:
                # Remove all edges touching this node first
                for nb_pos in list(self._adj.get(pos, set())):
                    ek = frozenset({pos, nb_pos})
                    self._remove_edge(ek)
                self._kill_node(pos)
                pruned += 1
                if self.node_count <= 2:
                    break
        return pruned

    def get_topology(self) -> Dict[str, Any]:
        """
        Return a snapshot of the current GNG topology for visualisation.

        Returns dict with:
            nodes: list of {id, prototype (list), visits, crystallised,
                            error (accumulated), ema_error (short-term)}
            edges: list of {a, b, age}
        """
        nodes = []
        for node_id, pos in self._id_to_pos.items():
            if self._alive[pos]:
                nodes.append({
                    "id": node_id,
                    "prototype": self._prototypes[pos].tolist(),
                    "visits": int(self._visits[pos]),
                    "crystallised": bool(self._visits[pos] >= self.baking_threshold),
                    "error": float(self._errors[pos]),
                    "ema_error": float(self._ema_errors[pos]),
                })

        edges = []
        for edge_key, age in self._edges.items():
            positions = list(edge_key)
            if len(positions) == 2:
                a_id = self._pos_to_id.get(positions[0])
                b_id = self._pos_to_id.get(positions[1])
                if a_id is not None and b_id is not None:
                    edges.append({"a": a_id, "b": b_id, "age": age})

        return {"nodes": nodes, "edges": edges}

    def to_dict(self) -> Dict[str, Any]:
        """Serialise full GNG state to a JSON-safe dict.

        Includes all fields needed for an exact hot-reload:
          - per-node ema_error   → consistency gate resumes mid-evaluation
          - stale_prune_enabled  → Active Decay checkbox state preserved
          - stale_window_factor  → Decay slider value preserved
          - hyperparams          → GNG settings preserved across reload
        """
        alive_positions = [p for p, a in enumerate(self._alive) if a]
        return {
            # --- Schema version (bump when format changes) ---
            "schema": 2,
            # --- Hyperparameters ---
            "dim":               self.dim,
            "baking_threshold":  self.baking_threshold,
            "min_insertion_error": self.min_insertion_error,
            "lambda_new":        self.lambda_new,
            "max_age":           self.max_age,
            # --- Decay / pruning controls ---
            "stale_prune_enabled": self.stale_prune_enabled,
            "stale_window_factor": self.stale_window_factor,
            # --- Runtime counters ---
            "step":              self._step,
            "next_id":           self._next_id,
            # --- Node state ---
            "nodes": [
                {
                    "id":               self._pos_to_id[p],
                    "prototype":        self._prototypes[p].tolist(),
                    "error":            float(self._errors[p]),
                    "ema_error":        float(self._ema_errors[p]),
                    "visits":           int(self._visits[p]),
                    "last_visited_step": int(self._last_visited_step[p]),
                }
                for p in alive_positions
            ],
            # --- Edge state ---
            "edges": [
                {"positions": list(ek), "age": age}
                for ek, age in self._edges.items()
            ],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any], **kwargs) -> "GNG":
        """Restore GNG state from a serialised dict (schema 1 or 2)."""
        # dim is authoritative from the dict; drop it from kwargs if the
        # caller passed it redundantly (e.g. adapter.load_state passes dim=).
        kwargs.pop("dim", None)
        gng = cls(dim=d["dim"], **kwargs)
        gng._step    = d["step"]
        gng._next_id = d["next_id"]
        gng._bootstrapped = True

        # Restore hyperparams saved in schema 2 (fall back to constructor defaults
        # for schema 1 dicts that predate these fields).
        if "baking_threshold"    in d: gng.baking_threshold    = d["baking_threshold"]
        if "min_insertion_error" in d: gng.min_insertion_error = d["min_insertion_error"]
        if "lambda_new"          in d: gng.lambda_new          = d["lambda_new"]
        if "max_age"             in d: gng.max_age             = d["max_age"]
        if "stale_prune_enabled" in d: gng.stale_prune_enabled = d["stale_prune_enabled"]
        if "stale_window_factor" in d: gng.stale_window_factor = d["stale_window_factor"]

        for node in d["nodes"]:
            pos = gng._alloc_position()
            gng._prototypes[pos]       = np.array(node["prototype"], dtype=np.float32)
            gng._errors[pos]           = node["error"]
            gng._ema_errors[pos]       = node.get("ema_error", 0.0)   # schema 2+
            gng._visits[pos]           = node["visits"]
            gng._alive[pos]            = True
            gng._last_visited_step[pos] = node.get("last_visited_step", gng._step)
            gng._id_to_pos[node["id"]] = pos
            gng._pos_to_id[pos]        = node["id"]

        for edge in d["edges"]:
            pos_a, pos_b = edge["positions"]
            ek = frozenset({pos_a, pos_b})
            gng._edges[ek] = edge["age"]
            gng._adj[pos_a].add(pos_b)
            gng._adj[pos_b].add(pos_a)
        return gng

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _find_two_nearest(self, x: np.ndarray) -> Tuple[int, int, float, float]:
        """
        Return (s1_pos, s2_pos, dist1, dist2) — the two nearest prototype positions.
        """
        alive_mask = self._alive
        alive_positions = np.where(alive_mask)[0]

        if len(alive_positions) == 0:
            raise RuntimeError("GNG has no alive nodes.")
        if len(alive_positions) == 1:
            p = alive_positions[0]
            d = float(np.linalg.norm(x - self._prototypes[p]))
            return p, p, d, d

        # Vectorised distance computation
        protos = self._prototypes[alive_positions]  # shape (n_alive, dim)
        diffs = protos - x                           # shape (n_alive, dim)
        dists = np.einsum('ij,ij->i', diffs, diffs)  # squared distances

        # Get two smallest
        if len(dists) == 2:
            idx_sorted = [0, 1] if dists[0] <= dists[1] else [1, 0]
        else:
            idx_sorted = np.argpartition(dists, 2)[:2]
            if dists[idx_sorted[0]] > dists[idx_sorted[1]]:
                idx_sorted[[0, 1]] = idx_sorted[[1, 0]]

        s1_pos = int(alive_positions[idx_sorted[0]])
        s2_pos = int(alive_positions[idx_sorted[1]])
        d1 = float(np.sqrt(max(dists[idx_sorted[0]], 0.0)))
        d2 = float(np.sqrt(max(dists[idx_sorted[1]], 0.0)))
        return s1_pos, s2_pos, d1, d2

    def _add_node(self, prototype: np.ndarray) -> int:
        """Allocate a new node and return its stable ID."""
        pos = self._alloc_position()
        self._prototypes[pos] = prototype.astype(np.float32)
        self._errors[pos] = 0.0
        self._ema_errors[pos] = 0.0
        self._visits[pos] = 0
        self._alive[pos] = True
        self._last_visited_step[pos] = self._step  # born now — grace period starts here

        node_id = self._next_id
        self._next_id += 1
        self._id_to_pos[node_id] = pos
        self._pos_to_id[pos] = node_id
        return node_id

    def _alloc_position(self) -> int:
        """Find a free position in the arrays, growing them if needed."""
        # Try to find an existing dead slot
        dead = np.where(~self._alive)[0]
        if len(dead) > 0:
            return int(dead[0])
        # Grow arrays
        old_cap = self._capacity
        new_cap = min(old_cap * 2, self.max_nodes + 64)
        self._prototypes = np.vstack([
            self._prototypes,
            np.zeros((new_cap - old_cap, self.dim), dtype=np.float32)
        ])
        self._errors     = np.concatenate([self._errors,     np.zeros(new_cap - old_cap)])
        self._ema_errors = np.concatenate([self._ema_errors, np.zeros(new_cap - old_cap)])
        self._visits     = np.concatenate([self._visits,     np.zeros(new_cap - old_cap, dtype=np.int64)])
        self._alive      = np.concatenate([self._alive,      np.zeros(new_cap - old_cap, dtype=bool)])
        self._last_visited_step = np.concatenate([
            self._last_visited_step, np.zeros(new_cap - old_cap, dtype=np.int64)])
        self._capacity = new_cap
        return old_cap  # first new position

    def _add_edge(self, pos_a: int, pos_b: int):
        if pos_a == pos_b:
            return
        ek = frozenset({pos_a, pos_b})
        self._edges[ek] = 0
        self._adj[pos_a].add(pos_b)
        self._adj[pos_b].add(pos_a)

    def _remove_edge(self, edge_key: frozenset):
        positions = list(edge_key)
        if edge_key in self._edges:
            del self._edges[edge_key]
        if len(positions) == 2:
            a, b = positions
            self._adj[a].discard(b)
            self._adj[b].discard(a)

    def _remove_isolated(self):
        """
        Remove nodes with no edges, preserving at least 2.

        Crystallised (baked) nodes are NEVER removed by isolation — they
        represent accumulated concepts and must survive periods of silence or
        input changes that temporarily break their edge connections.
        """
        if self.node_count <= 2:
            return
        for pos in list(self._pos_to_id.keys()):
            if not self._alive[pos]:
                continue
            # Baked nodes are immune to isolation pruning
            if self._visits[pos] >= self.baking_threshold:
                continue
            if len(self._adj[pos]) == 0:
                self._kill_node(pos)
                if self.node_count <= 2:
                    break

    def _prune_stale_unbaked(self):
        """
        Remove non-baked nodes that have not won recently.

        A non-baked node that hasn't attracted any input in the last
        `stale_window` steps is a candidate concept that failed to
        crystallise — it represents a region of latent space the current
        input distribution no longer visits.  Pruning it keeps the topology
        lean and ensures the EPM maturity / Mitosis Gatekeeper signals stay
        clean: a mature EPM's crystallization_ratio only reads high if the
        non-baked clutter has been removed.

        stale_window = stale_window_factor (absolute steps):
          Independent of baking_threshold so that changing bake sensitivity
          does not inadvertently speed up or slow down node decay.
          Default 12000 ≈ 400s at 30fps.  Overridden by the UI decay slider.

        Baked nodes and the last 2 alive nodes are always preserved.
        """
        if not self.stale_prune_enabled or self.node_count <= 2:
            return
        # stale_window is absolute steps — independent of baking_threshold
        stale_window = int(self.stale_window_factor)
        cutoff = self._step - stale_window
        pruned = 0
        for pos in list(self._pos_to_id.keys()):
            if not self._alive[pos]:
                continue
            if self._visits[pos] >= self.baking_threshold:
                continue  # baked — immune
            if self._last_visited_step[pos] < cutoff:
                # Remove edges first, then kill the node
                for nb_pos in list(self._adj.get(pos, set())):
                    self._remove_edge(frozenset({pos, nb_pos}))
                self._kill_node(pos)
                pruned += 1
                if self.node_count <= 2:
                    break
        return pruned

    def _kill_node(self, pos: int):
        """Permanently remove a node at the given array position."""
        self._alive[pos] = False
        node_id = self._pos_to_id.pop(pos, None)
        if node_id is not None:
            self._id_to_pos.pop(node_id, None)
        # Clean up adjacency (edges should already be gone)
        for nb in list(self._adj.get(pos, set())):
            self._adj[nb].discard(pos)
        if pos in self._adj:
            del self._adj[pos]

    def _insert_node(self):
        """
        Insert a new node between the highest-error node (q) and its highest-error
        neighbour (f). Halve errors of q and f; set new node error to q's new error.

        Convergence guard: skip insertion if the per-visit error of the worst node
        is below min_insertion_error. This causes GNG to plateau when the topology
        already represents the input distribution well — the "Convergence Test" from
        the v3 roadmap.
        """
        alive_positions = np.where(self._alive)[0]
        if len(alive_positions) < 2:
            return

        # Ecological self-tuning: the GNG agent picks its own insertion floor
        # from the 30th-percentile of its recent squared-TLE distribution.
        # Below that, errors are "typical" for this environment and no growth
        # is warranted; above it, the input is in the upper 70% of surprise.
        if (not self.freeze_min_insertion_error
                and len(self._tle_sq_history) >= self._tle_warmup):
            self.min_insertion_error = float(
                np.quantile(self._tle_sq_history, 0.30)
            )

        # Find q: node with maximum accumulated error
        q_pos = int(alive_positions[np.argmax(self._errors[alive_positions])])

        # Homeokinetic guard — mirrors the patent's Mitosis Gatekeeper.
        # Use the per-node short-term EMA of squared quantization error rather
        # than accumulated totals. The EMA reflects *current* surprise (high
        # TLE → high ema_error) and drops quickly when a region stabilises
        # (low TLE → ema_error falls to noise floor within ~20 visits).
        # Above min_insertion_error → still surprising → insert.
        # Below it → stable resonance achieved → suppress growth.
        q_ema_err = self._ema_errors[q_pos]
        if q_ema_err < self.min_insertion_error:
            return  # TLE too low — stable resonance achieved, no growth needed

        # Find f: neighbour of q with maximum accumulated error
        neighbours = [p for p in self._adj[q_pos] if self._alive[p]]
        if not neighbours:
            return
        f_pos = max(neighbours, key=lambda p: self._errors[p])

        # Determine insertion position.
        # Standard GNG: midpoint of q and f — works when prototypes drift.
        # But baked nodes are FROZEN so q and f stay in the "old concept" region.
        # Inserting at the midpoint of two synthetic nodes won't cover a new voice
        # concept that lands far away.  When q is baked, insert at the last input
        # instead — right where the current concept lives.
        q_is_baked = self._visits[q_pos] >= self.baking_threshold
        if q_is_baked and self._last_x is not None:
            new_proto = self._last_x.copy()
        else:
            new_proto = 0.5 * (self._prototypes[q_pos] + self._prototypes[f_pos])

        new_id = self._add_node(new_proto)
        new_pos = self._id_to_pos[new_id]

        # Edges: for baked-q insertions connect to the runner-up (s2) if available,
        # otherwise to q itself so the new node isn't isolated.
        if q_is_baked:
            self._add_edge(new_pos, q_pos)
            if f_pos != q_pos:
                self._add_edge(new_pos, f_pos)
        else:
            # Standard: remove q–f edge; add q–r and r–f edges
            ek_qf = frozenset({q_pos, f_pos})
            self._remove_edge(ek_qf)
            self._add_edge(q_pos, new_pos)
            self._add_edge(new_pos, f_pos)

        # Redistribute errors
        self._errors[q_pos] *= self.alpha
        self._errors[f_pos] *= self.alpha
        self._errors[new_pos] = self._errors[q_pos]
