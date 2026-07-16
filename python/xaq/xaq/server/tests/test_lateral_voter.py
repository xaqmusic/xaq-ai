import sys
import os
import time

# Ensure importability
sys.path.append(os.getcwd())

from xaq.server.lateral_voter import LateralVoterNode
from xaq.server.structures import RealityToken

def test_lateral_voting():
    print("Initialize Lateral Voter...")
    voter = LateralVoterNode(sync_window_ms=500, hebbian_rate=0.1, threshold=0.1)
    
    # Mock Data
    token_a = {
        'header': {'agent_id': 'agent_A', 'timestamp': time.time()},
        'payload': {
            'trajectory': [{'id': 1}, {'id': 2}, {'id': 3}],
            'current_id': 3,
            'neurotransmitters': {'dopamine': 0.8, 'serotonin': 0.9}
        }
    }
    
    token_b = {
        'header': {'agent_id': 'agent_B', 'timestamp': time.time()},
        'payload': {
            'trajectory': [{'id': 10}, {'id': 2}, {'id': 3}], # Overlap at 2, 3
            'current_id': 3,
            'neurotransmitters': {'dopamine': 0.5, 'serotonin': 0.5}
        }
    }
    
    print("Ingesting Token A...")
    voter.ingest_token(token_a)
    
    print("Ingesting Token B...")
    voter.ingest_token(token_b)
    
    print("Processing Consensus...")
    consensus = voter.process_consensus()
    
    if consensus:
        print(f"Consensus Reached! Resonance: {consensus.resonance_score:.4f}")
        print(f"Contributors: {consensus.contributing_sources}")
    else:
        print("No Consensus Reached.")
        
    # Check Matrix
    weight = voter.association_matrix[3][3] # Self association? Or cross?
    # Actually logic uses active_node_id of pairs. Both are 3.
    # So association_matrix[3][3] should increase.
    print(f"Hebbian Weight [3][3]: {weight:.4f}")
    
    return True

def test_tuning_parameters():
    voter = LateralVoterNode()
    
    token_a = {
        'header': {'agent_id': 'agent_A', 'timestamp': time.time()},
        'payload': {
            'trajectory': [{'id': 1}, {'id': 2}, {'id': 3}],
            'current_id': 3,
            'neurotransmitters': {'dopamine': 1.0, 'serotonin': 1.0}
        }
    }
    
    token_b = {
        'header': {'agent_id': 'agent_B', 'timestamp': time.time()},
        'payload': {
            'trajectory': [{'id': 1}, {'id': 2}, {'id': 3}],
            'current_id': 3,
            'neurotransmitters': {'dopamine': 1.0, 'serotonin': 1.0}
        }
    }
    
    voter.ingest_token(token_a)
    voter.ingest_token(token_b)
    
    c1 = voter.process_consensus()
    res1 = c1.resonance_score if c1 else 0.0
    
    # Increase neuro weight
    voter.neurotransmitter_weight = 3.0
    voter.ingest_token(token_a)
    voter.ingest_token(token_b)
    c2 = voter.process_consensus()
    res2 = c2.resonance_score if c2 else 0.0
    
    print(f"Default Resonance: {res1:.4f}, Boosted Resonance: {res2:.4f}")
    assert res2 > res1 or res1 == 1.0 # 1.0 is max naturally

if __name__ == "__main__":
    test_lateral_voting()
    test_tuning_parameters()
