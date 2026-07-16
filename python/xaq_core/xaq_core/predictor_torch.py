import torch
import torch.nn as nn
import torch.optim as optim

class MLPPredictor(nn.Module):
    """
    Simple MLP-based predictor.
    Predicts S_{t+1} from S_t.
    """
    def __init__(self, embedding_dim=768):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(embedding_dim, embedding_dim),
            nn.LayerNorm(embedding_dim),
            nn.ReLU(),
            nn.Linear(embedding_dim, embedding_dim)
        )
    
    def forward(self, x, state=None):
        return self.net(x), None

class GRUPredictor(nn.Module):
    """
    Temporal predictor using a GRU.
    Predicts S_{t+1} from S_t and hidden state h_t.
    """
    def __init__(self, embedding_dim=768, hidden_dim=768, num_layers=1):
        super().__init__()
        self.gru = nn.GRU(embedding_dim, hidden_dim, num_layers, batch_first=True)
        self.ln = nn.LayerNorm(hidden_dim) # Added for stability
        self.out = nn.Linear(hidden_dim, embedding_dim)
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

    def forward(self, x, state=None):
        # x shape: (Batch, Dim) -> (Batch, 1, Dim) for GRU
        if x.dim() == 2:
            x = x.unsqueeze(1)
            
        # state is hidden state
        output, next_state = self.gru(x, state)
        # output shape: (Batch, 1, HiddenDim)
        
        # Apply LayerNorm to the GRU output before the projection
        output = self.ln(output.squeeze(1))
        
        pred = self.out(output)
        return pred, next_state

class HomeokineticPredictor(nn.Module):
    """
    A wrapper for different predictor architectures.
    Handles the learning and optimization.
    """
    def __init__(self, embedding_dim=768, model_type="mlp", learning_rate=0.01):
        super().__init__()
        self.embedding_dim = embedding_dim
        self.model_type = model_type
        self.learning_rate = learning_rate
        
        if model_type == "mlp":
            self.model = MLPPredictor(embedding_dim)
            self.backward_model = MLPPredictor(embedding_dim)
        elif model_type == "gru":
            self.model = GRUPredictor(embedding_dim)
            # Use MLP for backward even if forward is GRU to keep it lightweight/stateless for now
            # Or use GRU reversed? Simple MLP is safer for conceptual consistency check.
            self.backward_model = MLPPredictor(embedding_dim) 
        else:
            raise ValueError(f"Unknown predictor type: {model_type}")
            
        self.optimizer = optim.Adam(self.parameters(), lr=learning_rate)
        self.state = None # Hidden state for temporal models
        self.prediction_horizon = 1 # Steps into future/past
        
    def reset_state(self):
        self.state = None

    def forward(self, current_embedding):
        """
        Predicts next embedding.
        Updates internal state if temporal.
        """
        pred, next_state = self.model(current_embedding, self.state)
        # We don't update self.state here if we are just 'looking ahead'
        # BUT in the brain loop, forward is usually followed by learn.
        return pred

    def predict_and_update_state(self, current_embedding):
        """Used in the main loop to step the hidden state."""
        pred, next_state = self.model(current_embedding, self.state)
        self.state = next_state
        if self.state is not None:
            self.state = self.state.detach() # Don't backprop through state indefinitely
        return pred

    def predict_backward(self, next_embedding):
        """
        Predicts previous embedding from current (next) embedding.
        Used for Consistency Check.
        """
        pred, _ = self.backward_model(next_embedding)
        return pred

    def calculate_sigreg_loss(self, latent_buffer):
        """
        Calculates the Sketched Isotropic Gaussian Regularization (SIGReg) loss.
        """
        if not latent_buffer:
            return torch.tensor(0.0, device=next(self.parameters()).device)
            
        if isinstance(latent_buffer, list):
            B = torch.stack(latent_buffer)
        else:
            B = latent_buffer
            
        if B.dim() == 3: # (W, Batch, Dim)
            B = B.squeeze(1)
            
        device = B.device
        D = B.size(-1)
        num_projections = min(8, D)
        
        # 1. Random Slicing: Sample uniformly from unit sphere
        v = torch.randn(D, num_projections, device=device)
        v = torch.nn.functional.normalize(v, p=2, dim=0)
        
        # 2. Projection: B: (W, D), v: (D, P) -> S_proj: (W, P)
        S_proj = torch.matmul(B, v)
        
        # 3. Distribution Matching Loss: Match mean 0, variance 1
        mean = S_proj.mean(dim=0)
        # Avoid division by zero if W <= 1
        var = S_proj.var(dim=0, unbiased=False) if S_proj.size(0) > 1 else torch.zeros_like(mean)
        
        mean_loss = torch.mean(mean**2)
        var_loss = torch.mean((var - 1.0)**2)
        
        return mean_loss + var_loss

    def learn(self, current_state, actual_next_state, prev_hidden_state=None, latent_buffer=None, sigreg_enabled=False):
        """
        The Homeokinetic update step.
        Minimizes TLE = ||Predicted - Actual||^2 (Forward + Backward)
        
        If prediction_horizon > 1, also enforces 'Dream Consistency' (Cycle Loss):
        S_t -> Pred -> ... -> S_{t+N} -> BackPred -> ... -> S'_{t}
        """
        self.train()
        self.optimizer.zero_grad()
        
        # 1. Forward Loss (1-step Ground Truth)
        predicted_next, _ = self.model(current_state, prev_hidden_state)
        loss_fwd = nn.functional.mse_loss(predicted_next, actual_next_state)
        
        # 2. Backward Loss (1-step Ground Truth)
        predicted_past, _ = self.backward_model(actual_next_state)
        loss_bwd = nn.functional.mse_loss(predicted_past, current_state)
        
        loss_cycle = torch.tensor(0.0, device=current_state.device)
        
        # 3. Cycle Consistency (Multi-step hallucination reversibility)
        if self.prediction_horizon > 1:
            # Forward Chain
            curr = current_state
            h = prev_hidden_state
            
            # We only have ground truth for step 1, so step 1 is anchored.
            # But the horizon implies we want the model's *predictions* to be consistent.
            
            path = [curr]
            
            # Unroll Forward
            for _ in range(self.prediction_horizon):
                curr, h = self.model(curr, h)
                path.append(curr)
                
            # Unroll Backward from the hallucinated tip
            rev = path[-1]
            for _ in range(self.prediction_horizon):
                rev, _ = self.backward_model(rev)
                
            # Cycle Loss: The reconstructed start should match the actual start
            loss_cycle = nn.functional.mse_loss(rev, current_state) * 0.5 # Weight it less than ground truth

        loss_sigreg = torch.tensor(0.0, device=current_state.device)
        if sigreg_enabled and latent_buffer and len(latent_buffer) > 1:
            loss_sigreg = self.calculate_sigreg_loss(latent_buffer) * 0.1 # Weight the regularization

        # Joint Optimization
        total_loss = loss_fwd + loss_bwd + loss_cycle + loss_sigreg
        total_loss.backward()
        
        # Gradient Clipping to prevent explosion in deep unrolls
        if self.prediction_horizon > 5:
            torch.nn.utils.clip_grad_norm_(self.parameters(), 1.0)
            
        self.optimizer.step()
        
        return loss_fwd.item(), loss_bwd.item()
