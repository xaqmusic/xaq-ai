from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
import time
import numpy as np

@dataclass
class RealityToken:
    """Standardized packet of semantic reality from an EPM."""
    source_id: str
    timestamp: float = 0.0 # Origin time (sender clock)
    received_time: float = 0.0 # Server arrival time (server clock)
    
    # The "Story" (Trajectory Binding)
    active_node_id: int = -1
    history_trace: List[int] = field(default_factory=list)
    
    # The "Chemistry" (Trust & Stability)
    dopamine_level: float = 0.0 # 0.0=Unbaked, 0.5=Baked, 1.0=SuperNode
    serotonin_level: float = 0.0 # Inverse TLE
    confidence: float = 0.0
    
    # The "Payload"
    embedding_vector: Optional[np.ndarray] = None
    text_label: str = ""
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "source_id": self.source_id,
            "timestamp": self.timestamp,
            "active_node_id": self.active_node_id,
            "history_trace": self.history_trace,
            "dopamine_level": self.dopamine_level,
            "serotonin_level": self.serotonin_level,
            "confidence": self.confidence,
            "text_label": self.text_label,
            # embeddings are usually not sent back for display purely, or need serialization
        }

@dataclass
class ConsensusToken:
    """The fused output representing the Agreed Reality."""
    timestamp: float
    fused_embedding: np.ndarray
    resonance_score: float
    contributing_sources: List[str]
    contributing_ids: List[int] = field(default_factory=list) # Raw IDs of active nodes
    text_labels: List[str] = field(default_factory=list)
    is_hallucination: bool = False
