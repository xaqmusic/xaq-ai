import time
import numpy as np
import logging
from collections import deque
from zanshin.server.benchmarks.workspace_interface import WorkspaceHarness

logger = logging.getLogger("BreakoutHarness")

class BreakoutHarness(WorkspaceHarness):
    """
    Game Harness for the Break-Pong benchmark.
    Calculates internal Stability based on raw game telemetry and TLE.
    """
    def __init__(self, voter, server):
        super().__init__(voter, server)
        
        # Internal Homeostatic Beliefs (z_belief)
        self.stability = 1.0
        self.pressure = 0.0
        
        # Physical Beliefs
        self.ball_pos = (320, 240)
        self.p1_x = 320
        self.score_gap = 0
        
        # Meta-EPM Parameters
        self.prediction_horizon = 20
        self.stiffness = 0.8
        
    def on_raw_state(self, data):
        """
        Ingest telemetry from the 'dumb' environment.
        """
        self.ball_pos = data.get('ball', (320, 240))
        self.p1_x = data.get('p1_x', 320)
        scores = data.get('score', (0, 0))
        self.score_gap = scores[1] - scores[0] # P2 - P1
        
        # Calculate Stability (The Brain's interpretation of these facts)
        self.evaluate_stability(data)

    def evaluate_stability(self, raw_state=None):
        """
        Homeostatic Evaluation.
        Derives stability from physical safety and competitive stress.
        """
        # 1. Base Decay
        self.stability -= 0.0005
        
        # 2. Score Pressure
        if self.score_gap > 0:
            self.stability -= 0.0002 * self.score_gap
        elif self.score_gap < 0:
            self.stability += 0.0001 * abs(self.score_gap)
            
        # 3. Proximity Danger (Bottom Pit)
        ball_y = self.ball_pos[1]
        if ball_y > 400: # Ball is getting close to losing
            danger = (ball_y - 400) / 80.0
            self.stability -= 0.001 * danger
            
        self.stability = max(0.0, min(1.0, self.stability))

    def on_consensus(self, consensus, candidates):
        """
        Active Inference Loop.
        """
        # In a real setup, we use consensus to update ball_pos belief.
        # For the benchmark refactor, we rely on on_raw_state for the 'physics'
        # but the decision logic remains the same.
        
        # 1. Forward Prediction
        # (Simple linear projector for this benchmark)
        # Using internal belief of ball_pos updated by telemetry
        pred_x = self.ball_pos[0] # Simplified projection
        
        # 2. Select Action to minimize future Instability
        # Target: Match paddle X to predicted ball X
        error = pred_x - (self.p1_x + 50) # 50 is half paddle width
        velocity = error * self.stiffness
        velocity = max(-15, min(15, velocity))
        
        # 3. Relay Action back to Game via Server
        if self.server:
            import asyncio
            asyncio.run_coroutine_threadsafe(
                self.server.broadcast_action(player=1, velocity=velocity),
                asyncio.get_event_loop()
            )
            
    def get_stats(self):
        """Harness API for the AI Supervision Dashboard"""
        return {
            'internal_stability': self.stability,
            'score_pressure': self.score_gap * 0.0002 if self.score_gap > 0 else 0,
            'belief_ball_pos': self.ball_pos
        }
