from abc import ABC, abstractmethod

class WorkspaceHarness(ABC):
    """
    Abstract Base Class for modular Global Workspace harnesses.
    A Harness bridge the gap between abstract Consensus Tokens and 
    environment-specific actions and homeostatic evaluations.
    """
    
    def __init__(self, voter, server):
        self.voter = voter
        self.server = server
        self.voter.subscribe_consensus(self.on_consensus)

    @abstractmethod
    def on_consensus(self, consensus, candidates):
        """
        Triggered when a new Consensus Token is formed.
        This is where the Active Inference loop should live.
        """
        pass

    @abstractmethod
    def evaluate_stability(self, raw_state=None):
        """
        Calculates the internal homeostatic 'Stability' belief.
        Can be based on raw_state from the environment or internal TLE.
        """
        pass

    @abstractmethod
    def on_raw_state(self, data):
        """
        Optional: Handle raw telemetry from the environment (scores, physics).
        """
        pass
