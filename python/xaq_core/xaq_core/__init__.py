"""xaq_core — the shared substrate: message bus, torch predictor/memory,
export parity, seeded RNG, and the event-logging protocol. Depended on by the
public `xaq` engine and by the private AMI-Awen audio product."""
__all__ = ["rng", "memory", "predictor_torch", "bus", "logging", "export"]
