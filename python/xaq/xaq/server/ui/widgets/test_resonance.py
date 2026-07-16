from xaq.server.lateral_voter import LateralVoterNode
import time

voter = LateralVoterNode(sync_window_ms=500, hebbian_rate=0.1, threshold=0.1) # use their low threshold
token_a = {
    'header': {'agent_id': 'agent_A', 'timestamp': time.time()},
    'payload': {
        'trajectory': [{'id': 1}, {'id': 2}, {'id': 3}],
        'current_id': 3,
        'neurotransmitters': {} # Empty to simulate missing
    }
}

token_b = {
    'header': {'agent_id': 'agent_B', 'timestamp': time.time()},
    'payload': {
        'trajectory': [{'id': 1}, {'id': 2}, {'id': 3}],
        'current_id': 3,
        'neurotransmitters': {}
    }
}

voter.ingest_token(token_a)
voter.ingest_token(token_b)
c = voter.process_consensus()
if c:
    print(f"Resonance with no NTs: {c.resonance_score}")
else:
    print("No consensus")
