"""
Topological Transition Predictor — AMI-Ogma v3.

Learns the dynamics of the environment by tracking transitions between GNG nodes.
Calculates 'Prediction TLE' (Surprise) based on Euclidean distance in latent space.
"""

import numpy as np
import threading
from collections import defaultdict
from typing import Dict, Tuple, Optional, Any

class TopologicalPredictor:
    """
    Learns P(node_t+1 | node_t, modality) transition probabilities.
    
    Attributes:
        _transitions: (modality, prev_id) -> {next_id: count}
        _last_node: modality -> prev_id
    """
    def __init__(self, ema_alpha: float = 0.1):
        self._transitions: Dict[Tuple[str, int], Dict[int, int]] = defaultdict(lambda: defaultdict(int))
        self._last_node: Dict[str, int] = {}
        self._lock = threading.Lock()
        
        # Performance Tracking
        self._tle_ema: Dict[str, float] = defaultdict(float)
        self._ema_alpha = ema_alpha
        
        # Meta-Stats for UI
        self.total_transitions = 0
        self.learned_nodes = set()

    def learn(self, modality: str, current_node: int, prototype: Optional[np.ndarray] = None):
        """
        Record a transition and update TLE.
        """
        if current_node < 0:
            return

        with self._lock:
            prev_node = self._last_node.get(modality)
            self.learned_nodes.add((modality, current_node))

            if prev_node is not None and prev_node != -1:
                # 1. Update counts
                self._transitions[(modality, prev_node)][current_node] += 1
                self.total_transitions += 1

            self._last_node[modality] = current_node

    def predict(self, modality: str, current_node: int) -> Optional[int]:
        """
        Predict the most likely next node ID.
        """
        with self._lock:
            return self._predict_internal(modality, current_node)

    def _predict_internal(self, modality: str, current_node: int) -> Optional[int]:
        counts = self._transitions.get((modality, current_node))
        if not counts:
            return None
        return max(counts, key=counts.get)

    def calculate_surprise(self, 
                           modality: str, 
                           actual_node: int, 
                           actual_prototype: np.ndarray,
                           get_prototype_fn) -> float:
        """
        Calculates the Euclidean distance (TLE) between the predicted next prototype
        and the actual winner prototype.
        
        Args:
            modality: The sensory stream name.
            actual_node: The ID of the node that just won.
            actual_prototype: The latent vector of the actual winner.
            get_prototype_fn: Callable(modality, node_id) -> np.ndarray
        """
        with self._lock:
            prev_node = self._last_node.get(modality)
            if prev_node is None:
                return 0.0
            
            # 1. Get prediction for THIS step (made based on previous step's winner)
            predicted_id = self._predict_internal(modality, prev_node)
            if predicted_id is None or predicted_id == actual_node:
                tle = 0.0
            else:
                pred_proto = get_prototype_fn(modality, predicted_id)
                if pred_proto is not None:
                    # Euclidean distance in latent space
                    tle = np.linalg.norm(actual_prototype - pred_proto)
                else:
                    tle = 1.0 # Max surprise if prediction vanished
            
            # 2. Update EMA
            self._tle_ema[modality] = (1 - self._ema_alpha) * self._tle_ema[modality] + \
                                      self._ema_alpha * tle
            
            return float(self._tle_ema[modality])

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "total_transitions": self.total_transitions,
                "unique_states":    len(self.learned_nodes),
                "avg_tle":          round(float(np.mean(list(self._tle_ema.values()))) if self._tle_ema else 0.0, 4),
            }

    def to_dict(self) -> dict:
        """Serialisation for checkpointing."""
        with self._lock:
            # Convert default dict to serialisable format
            serial_trans = {}
            for k, v in self._transitions.items():
                # k is (mod, prev_id)
                key_str = f"{k[0]}:{k[1]}"
                serial_trans[key_str] = dict(v)
            
            return {
                "transitions": serial_trans,
                "tle_ema":     dict(self._tle_ema),
                "total":       self.total_transitions
            }
