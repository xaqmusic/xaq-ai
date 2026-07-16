import time
import numpy as np
import logging
from collections import deque

logger = logging.getLogger("BreakoutWorkspace")

class BreakoutWorkspace:
    """
    The Meta-EPM and AI Interaction Harness for the Breakout benchmark.
    This component receives consensus tokens, manages the global state belief,
    and performs Active Inference to drive game actions.
    """
    def __init__(self, voter, server):
        self.voter = voter
        self.server = server
        
        # State Belief (z_real)
        self.ball_pos_belief = deque(maxlen=10) # Track trajectory
        self.paddle_pos_belief = 320 # Center
        
        # Meta-EPM Memory (Crystallized global concepts)
        self.global_concepts = {} # (traj_hash) -> belief_weight
        
        # Homeostatic State (z_target)
        self.serotonin = 1.0 # Predictive stability
        self.dopamine = 0.5 # Novelty / Authority
        self.target_y_setpoint = 100 # Keep ball high
        
        # Active Inference Parameters (Fine-tunable via Harness)
        self.prediction_horizon = 20 # frames
        self.action_stiffness = 0.8
        self.learning_rate = 0.05
        
        # Subscribe to voter
        self.voter.subscribe_consensus(self.on_consensus_received)
        
    def on_consensus_received(self, consensus, candidates):
        """
        Main cognitive loop triggered by the Lateral Voting consensus.
        """
        # 1. Update Belief from Consensus (z_real)
        # In a real setup, we'd map the fused_embedding or contributing_ids to game coordinates.
        # For the benchmark harness, we assume the EPMs have learned to map specific 
        # sensory nodes to 'Ball X' and 'Ball Y' roughly.
        
        # Mapped coordinate estimation (Simplified for benchmark)
        current_belief = self._map_consensus_to_physics(consensus)
        if current_belief:
            self.ball_pos_belief.append(current_belief)
            
        # 2. Forward Prediction (Meta-EPM Projector)
        z_pred = self._forward_predict(self.ball_pos_belief)
        
        # 3. Active Inference Calculation
        # Minimize Error = z_target - z_pred
        # Goal: Intercept the ball's predicted X at the paddle's Y
        action_p1 = self._calculate_action(z_pred)
        
        # 4. Action Execution
        # Relay to server to broadcast to the game client
        if self.server:
            import asyncio
            # We are likely in a synchronous thread from process_consensus, 
            # but the server broadcast is async.
            asyncio.run_coroutine_threadsafe(
                self.server.broadcast_action(player=1, velocity=action_p1),
                asyncio.get_event_loop()
            )

    def _map_consensus_to_physics(self, consensus):
        """
        Maps abstract consensus token to physical game state.
        This represents the EPM's 'learned' understanding of reality.
        """
        # For benchmark purposes, we skip the heavy latent mapping and 
        # simulate the retrieval from the Lateral Voter's concept counts.
        # In the whitepaper, this is Claim 3.
        return (320, 240) # Placeholder

    def _forward_predict(self, trajectory):
        """
        Meta-EPM Forward Predictor. 
        Projects the current trajectory into the future.
        """
        if len(trajectory) < 2:
            return (320, 240) # Static belief
            
        # Linear extrapolation (Simplest Meta-EPM implementation)
        last = trajectory[-1]
        prev = trajectory[-2]
        vx = last[0] - prev[0]
        vy = last[1] - prev[1]
        
        # Predict where ball will be in N frames
        pred_x = last[0] + vx * self.prediction_horizon
        pred_y = last[1] + vy * self.prediction_horizon
        
        return (pred_x, pred_y)

    def _calculate_action(self, z_pred):
        """
        Active Inference Engine: Generates torque/velocity commands
        to minimize future predictive error.
        """
        # Goal: Paddle X == Ball Predicted X
        target_x = z_pred[0]
        error_x = target_x - self.paddle_pos_belief
        
        # Adjust velocity based on error (P-control as a proxy for Active Inference)
        velocity = error_x * self.action_stiffness
        
        # Clamp to realistic limits
        velocity = max(-15, min(15, velocity))
        
        # Update internal belief of paddle position (Homeokinesis)
        self.paddle_pos_belief += velocity
        
        return velocity

    # --- AI INTERACTION HARNESS API ---
    
    def get_harness_stats(self):
        """
        Returns a rich state observation for the 'Agentic Finetuner'.
        """
        return {
            'workspace': {
                'serotonin': self.serotonin,
                'dopamine': self.dopamine,
                'prediction_error': 0.0, # Computed during simulation
                'active_concepts': len(self.global_concepts)
            },
            'harness_params': {
                'prediction_horizon': self.prediction_horizon,
                'action_stiffness': self.action_stiffness
            }
        }
        
    def tune_parameter(self, param_name, value):
        """
        Allows an external AI agent to fine-tune the Workspace's cognitive params.
        """
        if hasattr(self, param_name):
            setattr(self, param_name, value)
            logger.info(f"Param {param_name} tuned to {value}")
            return True
        return False
