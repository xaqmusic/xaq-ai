"""Generic, IP-free audio encoder for the Zanshin EPM audio slot.

`FrozenSTFTEncoder` mirrors `FrozenProjectionEncoder`'s API exactly so that
`make_encoder` and `EPM_V3` consume it unchanged: it exposes `encode()`
returning an L2-normalised ``(projection_dim,)`` vector, plus
`last_encoded_frame` (a log-mel spectrogram for the 2-D inspector) and
`last_mu` (inert here — there is no efferent/MOC control in the generic path).

Pipeline (fully determined at construction — no learned parameters):

  raw audio -> torch.stft magnitude -> fixed mel filterbank -> log1p
            -> time-pool (mean over frames) -> L2
            -> seeded Johnson-Lindenstrauss projection -> L2

The seeded JL matrix uses the SAME ``_modality_seed`` scheme as
`FrozenProjectionEncoder`, so the GNG downstream needs no reconfiguration.
Stereo input is folded to mono (mean) — ITD/binaural processing is a
bio-mimetic concern that lives in the private AMI-Awen tree, not here.

This is the Python analogue of ``cpp_core/src/v3/encoder_stft.cpp``.
"""

from typing import Optional

import numpy as np

from .projection import _modality_seed


def _mel_filterbank(n_fft: int, n_mels: int, sample_rate: int,
                    f_min: float, f_max: float) -> np.ndarray:
    """Fixed triangular mel filterbank, shape (n_mels, n_fft // 2 + 1)."""
    def hz_to_mel(f):
        return 2595.0 * np.log10(1.0 + f / 700.0)

    def mel_to_hz(m):
        return 700.0 * (10.0 ** (m / 2595.0) - 1.0)

    n_bins = n_fft // 2 + 1
    mel_pts = np.linspace(hz_to_mel(f_min), hz_to_mel(f_max), n_mels + 2)
    hz_pts = mel_to_hz(mel_pts)
    bin_pts = np.floor((n_fft + 1) * hz_pts / sample_rate).astype(int)
    bin_pts = np.clip(bin_pts, 0, n_bins - 1)

    fb = np.zeros((n_mels, n_bins), dtype=np.float32)
    for m in range(1, n_mels + 1):
        lo, ctr, hi = bin_pts[m - 1], bin_pts[m], bin_pts[m + 1]
        for k in range(lo, ctr):
            if ctr > lo:
                fb[m - 1, k] = (k - lo) / (ctr - lo)
        for k in range(ctr, hi):
            if hi > ctr:
                fb[m - 1, k] = (hi - k) / (hi - ctr)
    return fb


class FrozenSTFTEncoder:
    """Generic STFT -> mel -> JL audio encoder. No learned parameters."""

    def __init__(self, modality: str = "audio", projection_dim: int = 128,
                 sample_rate: int = 48000, n_fft: int = 1024,
                 hop_length: int = 512, n_mels: int = 128,
                 f_min: float = 20.0, f_max: Optional[float] = None):
        self.modality = modality
        self.projection_dim = int(projection_dim)
        self.sample_rate = int(sample_rate)
        self.n_fft = int(n_fft)
        self.hop_length = int(hop_length)
        self.n_mels = int(n_mels)
        self.f_min = float(f_min)
        self.f_max = float(f_max) if f_max is not None else sample_rate / 2.0

        self._mel_fb = _mel_filterbank(self.n_fft, self.n_mels,
                                       self.sample_rate, self.f_min, self.f_max)
        # Same seeded JL construction as FrozenProjectionEncoder.
        rng = np.random.default_rng(_modality_seed(modality))
        self.R = (rng.standard_normal((self.n_mels, self.projection_dim))
                  / np.sqrt(self.projection_dim)).astype(np.float32)

        # API parity with FrozenProjectionEncoder / FrozenHopfEncoder:
        self.last_encoded_frame: Optional[np.ndarray] = None  # (n_mels, T) log-mel
        self.last_mu: float = 0.0  # inert — no MOC in the generic path

    def _stft_mag(self, x: np.ndarray) -> np.ndarray:
        """Magnitude STFT, shape (n_fft // 2 + 1, T). torch if present, else numpy."""
        try:
            import torch
            xt = torch.as_tensor(x, dtype=torch.float32)
            spec = torch.stft(xt, n_fft=self.n_fft, hop_length=self.hop_length,
                              window=torch.hann_window(self.n_fft),
                              return_complex=True)
            return spec.abs().cpu().numpy()
        except Exception:
            # numpy fallback: framed Hann-windowed rFFT
            win = np.hanning(self.n_fft).astype(np.float32)
            if len(x) < self.n_fft:
                x = np.pad(x, (0, self.n_fft - len(x)))
            n_frames = 1 + (len(x) - self.n_fft) // self.hop_length
            frames = np.stack([
                x[i * self.hop_length: i * self.hop_length + self.n_fft] * win
                for i in range(max(n_frames, 1))
            ], axis=1)
            return np.abs(np.fft.rfft(frames, axis=0))

    def encode(self, audio_chunk: np.ndarray) -> np.ndarray:
        x = np.asarray(audio_chunk, dtype=np.float32)
        if x.ndim == 2:  # stereo -> mono (no ITD; that is AMI-Awen)
            x = x.mean(axis=0 if x.shape[0] in (1, 2) else 1)
        x = x.reshape(-1)
        if x.size == 0:
            return np.zeros(self.projection_dim, dtype=np.float32)

        spec = self._stft_mag(x)                       # (F, T)
        mel = self._mel_fb @ spec                      # (n_mels, T)
        logmel = np.log1p(mel).astype(np.float32)
        self.last_encoded_frame = logmel

        feat = logmel.mean(axis=1)                     # (n_mels,)
        feat = feat / (np.linalg.norm(feat) + 1e-6)
        proj = feat @ self.R                           # (projection_dim,)
        proj = proj / (np.linalg.norm(proj) + 1e-6)
        return proj.astype(np.float32)

    def output_dim(self) -> int:
        return self.projection_dim
