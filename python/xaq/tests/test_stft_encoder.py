"""Tests for the generic STFT audio encoder and the make_encoder registry.

These prove the xaq audio slot is IP-free, deterministic, and API-compatible
with the EPM, and that the registry degrades gracefully with no plugin present.
"""

import numpy as np
import pytest

from xaq.encoders import make_encoder
from xaq.encoders.audio import FrozenSTFTEncoder


def _tone(freq=440.0, n=4800, sr=48000):
    return np.sin(2 * np.pi * freq * np.arange(n) / sr).astype(np.float32)


def test_shape_and_l2():
    enc = FrozenSTFTEncoder("audio", projection_dim=128)
    v = enc.encode(_tone())
    assert v.shape == (128,)
    assert abs(np.linalg.norm(v) - 1.0) < 1e-4  # L2-normalised
    assert enc.output_dim() == 128


def test_determinism_same_seed():
    a = FrozenSTFTEncoder("audio")
    b = FrozenSTFTEncoder("audio")
    np.testing.assert_array_equal(a.R, b.R)  # seeded JL is reproducible
    np.testing.assert_allclose(a.encode(_tone()), b.encode(_tone()), atol=1e-6)


def test_last_encoded_frame_is_logmel():
    enc = FrozenSTFTEncoder("audio", n_mels=128)
    enc.encode(_tone())
    assert enc.last_encoded_frame is not None
    assert enc.last_encoded_frame.shape[0] == 128  # (n_mels, T)
    assert enc.last_mu == 0.0  # inert in the generic path


def test_stereo_folds_to_mono():
    enc = FrozenSTFTEncoder("audio")
    stereo = np.stack([_tone(), _tone()], axis=0)  # (2, N)
    v = enc.encode(stereo)
    assert v.shape == (128,)


def test_empty_input():
    enc = FrozenSTFTEncoder("audio")
    v = enc.encode(np.zeros(0, dtype=np.float32))
    assert v.shape == (128,)


def test_registry_audio_is_stft():
    e = make_encoder("audio")
    assert isinstance(e, FrozenSTFTEncoder)


def test_cochlear_degrades_without_plugin():
    # With no AMI-Awen plugin installed, a legacy audio name must degrade
    # to the generic STFT rather than raising.
    e = make_encoder("cochlear")
    assert isinstance(e, FrozenSTFTEncoder)
