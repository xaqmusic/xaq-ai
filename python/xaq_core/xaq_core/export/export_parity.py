"""
AMI-Ogma Export Parity Checker

At export time the Python app holds the ground truth: the trained encoder,
trained predictor, and baked knowledge graph.  This module:

  1. generate_golden_samples()  — runs N deterministic dummy inputs through a
     PyTorch model and records (input, pytorch_output) pairs.

  2. check_onnx_parity()        — loads the exported ONNX model with
     onnxruntime, runs the same inputs, and compares outputs against the
     golden set.  Returns structured PASS/FAIL results.

  3. save / load helpers for the golden sample npz file.

  4. build_predictor_from_weights() — reconstructs the correct predictor
     architecture (MLP vs GRU) from a saved state dict and wraps it in a
     stateless ONNX-export-friendly module.

Usage in sync_models.py:
    from xaq_core.export.export_parity import (
        generate_golden_samples, check_onnx_parity,
        save_golden_samples, build_predictor_export_wrapper
    )

Usage in main_window.py (Python app):
    torch.save(brain.predictor.model.state_dict(),
               os.path.join(modality_dir, "predictor_weights.pt"))
"""

import json
import os
import numpy as np
import torch
import torch.nn as nn

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

NUM_GOLDEN_SAMPLES = 5
ENCODER_ABS_TOL   = 5e-4   # float32 ONNX precision for encoder outputs
PREDICTOR_ABS_TOL = 5e-4   # float32 ONNX precision for predictor outputs
COSINE_SIM_MIN    = 0.9999  # embeddings must be nearly identical in direction


# ─────────────────────────────────────────────────────────────────────────────
# Predictor reconstruction
# ─────────────────────────────────────────────────────────────────────────────

class _PredictorExportWrapper(nn.Module):
    """Stateless single-in / single-out wrapper for ONNX tracing.

    The C++ EPM calls the predictor as a pure function: predicted = f(z).
    The inner model (MLP or GRU) returns a (pred, state) tuple and may
    expect a hidden state argument.  This wrapper hides both details so
    ONNX export sees a clean (embedding_input → predicted_embedding) graph.
    """
    def __init__(self, inner_model: nn.Module):
        super().__init__()
        self.inner = inner_model

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        pred, _ = self.inner(x, None)
        # GRUPredictor adds a sequence dim; squeeze it back out
        if pred.dim() == 3:
            pred = pred.squeeze(1)
        return pred


def build_predictor_export_wrapper(weights_path: str, embedding_dim: int = 768):
    """Load a saved predictor state dict and return an ONNX-ready wrapper.

    Architecture is detected automatically from the state dict key names:
      - keys containing 'gru'  → GRUPredictor
      - otherwise              → MLPPredictor  (default)

    If weights_path does not exist, returns a fresh MLPPredictor wrapper
    (random weights) and logs a warning — same behaviour as before the fix.
    """
    try:
        from xaq_core.predictor_torch import MLPPredictor, GRUPredictor
    except ImportError:
        from .predictor import MLPPredictor, GRUPredictor

    if not os.path.exists(weights_path):
        print(f"  [parity] WARNING: {weights_path} not found — "
              "exporting UNTRAINED predictor (random weights).")
        inner = MLPPredictor(embedding_dim=embedding_dim)
    else:
        state_dict = torch.load(weights_path, map_location='cpu', weights_only=True)
        is_gru = any('gru' in k for k in state_dict.keys())
        if is_gru:
            print(f"  [parity] Detected GRUPredictor — loading trained weights.")
            inner = GRUPredictor(embedding_dim=embedding_dim, hidden_dim=embedding_dim)
        else:
            print(f"  [parity] Detected MLPPredictor — loading trained weights.")
            inner = MLPPredictor(embedding_dim=embedding_dim)
        missing, unexpected = inner.load_state_dict(state_dict, strict=False)
        if missing:
            print(f"  [parity] WARNING: missing keys: {missing}")
        if unexpected:
            print(f"  [parity] WARNING: unexpected keys: {unexpected}")

    inner.eval()
    return _PredictorExportWrapper(inner)


# ─────────────────────────────────────────────────────────────────────────────
# Golden sample generation
# ─────────────────────────────────────────────────────────────────────────────

def generate_golden_samples(pytorch_model: nn.Module,
                            dummy_input_fn,
                            n: int = NUM_GOLDEN_SAMPLES) -> dict:
    """Run a PyTorch model on N deterministic dummy inputs and record outputs.

    Args:
        pytorch_model:  eval()-mode PyTorch nn.Module (the export wrapper)
        dummy_input_fn: callable(seed) → tuple of tensors (or single tensor)
                        Must return the same type/shape as the ONNX model's
                        expected inputs.
        n:              number of samples

    Returns:
        dict with:
            'inputs':   list of n numpy arrays (or list-of-arrays for multi-input)
            'outputs':  list of n numpy arrays  (768-dim float32)
    """
    pytorch_model.eval()
    inputs_list  = []
    outputs_list = []

    with torch.no_grad():
        for i in range(n):
            inp = dummy_input_fn(seed=i)
            if isinstance(inp, (list, tuple)):
                out = pytorch_model(*inp)
                inputs_list.append([x.cpu().numpy() for x in inp])
            else:
                out = pytorch_model(inp)
                inputs_list.append(inp.cpu().numpy())

            if out.dim() == 3:
                out = out.squeeze(1)
            outputs_list.append(out.squeeze(0).cpu().numpy())

    return {'inputs': inputs_list, 'outputs': outputs_list}


# ─────────────────────────────────────────────────────────────────────────────
# Save / load
# ─────────────────────────────────────────────────────────────────────────────

def save_golden_samples(samples: dict, path: str, meta: dict = None):
    """Save golden samples to a .npz file alongside optional metadata."""
    arrays = {}
    for i, inp in enumerate(samples['inputs']):
        if isinstance(inp, list):
            for j, x in enumerate(inp):
                arrays[f'input_{i}_{j}'] = x
            arrays[f'input_{i}_count'] = np.array([len(inp)])
        else:
            arrays[f'input_{i}'] = inp
        arrays[f'output_{i}'] = samples['outputs'][i]

    arrays['n_samples'] = np.array([len(samples['inputs'])])
    if meta:
        arrays['meta_json'] = np.array([json.dumps(meta)])
    np.savez_compressed(path, **arrays)
    print(f"  [parity] Golden samples saved -> {path}  ({len(samples['inputs'])} samples)")


def load_golden_samples(path: str) -> dict:
    """Load a golden samples npz file.  Returns same format as generate_golden_samples."""
    data = np.load(path, allow_pickle=False)
    n = int(data['n_samples'][0])
    inputs_list  = []
    outputs_list = []

    for i in range(n):
        if f'input_{i}' in data:
            inputs_list.append(data[f'input_{i}'])
        else:
            # Multi-input
            count = int(data[f'input_{i}_count'][0])
            inputs_list.append([data[f'input_{i}_{j}'] for j in range(count)])
        outputs_list.append(data[f'output_{i}'])

    meta = None
    if 'meta_json' in data:
        try:
            meta = json.loads(str(data['meta_json'][0]))
        except Exception:
            pass

    return {'inputs': inputs_list, 'outputs': outputs_list, 'meta': meta}


# ─────────────────────────────────────────────────────────────────────────────
# ONNX parity check
# ─────────────────────────────────────────────────────────────────────────────

class ParityResult:
    """Holds the result of a single parity check."""
    def __init__(self, name: str, passed: bool, max_abs_err: float,
                 cosine_sim: float, detail: str = ""):
        self.name       = name
        self.passed     = passed
        self.max_abs_err = max_abs_err
        self.cosine_sim = cosine_sim
        self.detail     = detail

    def __str__(self):
        status = "PASS" if self.passed else "FAIL"
        return (f"  [{status}] {self.name}: "
                f"max_abs_err={self.max_abs_err:.2e}  "
                f"cosine_sim={self.cosine_sim:.6f}"
                + (f"  ({self.detail})" if self.detail else ""))


def check_onnx_parity(onnx_path: str,
                      golden: dict,
                      input_names: list,
                      abs_tol: float = ENCODER_ABS_TOL,
                      label: str = "encoder") -> list:
    """
    Run ONNX model against golden samples and return ParityResult per sample.

    Args:
        onnx_path:   path to the .onnx file
        golden:      output of generate_golden_samples() or load_golden_samples()
        input_names: list of ONNX model input names (in order)
        abs_tol:     maximum allowed absolute error per element
        label:       prefix for result names (e.g. 'encoder', 'predictor')
    """
    try:
        import onnxruntime as ort
    except ImportError:
        print("  [parity] onnxruntime not installed — skipping ONNX parity check.")
        return []

    session = ort.InferenceSession(onnx_path,
                                   providers=['CPUExecutionProvider'])
    results = []

    for i, (inp, ref_out) in enumerate(zip(golden['inputs'], golden['outputs'])):
        # Build feed dict
        if isinstance(inp, list):
            feed = {name: arr.astype(np.float32)
                    for name, arr in zip(input_names, inp)}
        else:
            feed = {input_names[0]: inp.astype(np.float32)}

        try:
            onnx_outs = session.run(None, feed)
            onnx_out  = onnx_outs[0].squeeze()

            ref = ref_out.squeeze().astype(np.float32)

            max_abs_err = float(np.max(np.abs(onnx_out - ref)))

            # Cosine similarity
            ref_norm  = ref  / (np.linalg.norm(ref)  + 1e-12)
            onnx_norm = onnx_out / (np.linalg.norm(onnx_out) + 1e-12)
            cosine_sim = float(np.dot(ref_norm, onnx_norm))

            passed = (max_abs_err <= abs_tol) and (cosine_sim >= COSINE_SIM_MIN)
            results.append(ParityResult(
                name=f"{label}_sample_{i}",
                passed=passed,
                max_abs_err=max_abs_err,
                cosine_sim=cosine_sim,
                detail="" if passed else f"tol={abs_tol:.0e}"
            ))
        except Exception as e:
            results.append(ParityResult(
                name=f"{label}_sample_{i}",
                passed=False,
                max_abs_err=float('inf'),
                cosine_sim=0.0,
                detail=str(e)
            ))

    return results


def print_parity_report(results: list, modality: str):
    """Print a compact parity report and return overall pass/fail."""
    if not results:
        return True
    n_pass = sum(r.passed for r in results)
    n_total = len(results)
    status = "PASS" if n_pass == n_total else "FAIL"
    print(f"\n  ── Parity Check: {modality} [{status}] {n_pass}/{n_total} ──")
    for r in results:
        print(str(r))
    return n_pass == n_total
