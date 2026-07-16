"""xaq encoder factory + registry bootstrap.

`make_encoder(modality, ...)` is the single entry point the EPM uses. Built-in
generic encoders are registered here at import time; external providers (e.g.
the private AMI-Awen cochlear encoder) plug in through the ``xaq.encoders``
setuptools entry-point group, so:

* with no plugin installed, ``make_encoder("audio")`` returns the generic STFT
  encoder, and an unknown/legacy audio name degrades to it rather than crashing;
* with the AMI-Awen wheel installed, ``make_encoder("cochlear")`` resolves to
  the bio-mimetic Hopf encoder automatically — with zero code change in xaq.
"""

from .registry import register, get_factory, group_of, groups, registered
from .projection import (
    FrozenProjectionEncoder,
    ProprioceptiveEncoder,
    _SPATIAL_RES,
    _modality_seed,
)
from .audio import FrozenSTFTEncoder

__all__ = [
    "make_encoder", "register", "group_of", "groups", "registered",
    "FrozenProjectionEncoder", "ProprioceptiveEncoder", "FrozenSTFTEncoder",
]


def _register_builtins() -> None:
    # Generic audio slot (IP-free). Multi-modal fusion keeps an audio modality.
    register("audio",
             lambda m, **k: FrozenSTFTEncoder(
                 m, projection_dim=k.get("projection_dim", 128),
                 sample_rate=k.get("sample_rate", 48000)),
             group="audio")
    # Visual modalities -> frozen JL projection.
    for _m in _SPATIAL_RES:
        register(_m,
                 lambda m, **k: FrozenProjectionEncoder(
                     modality=m,
                     projection_dim=k.get("projection_dim", 128),
                     inject_centroid=k.get("inject_centroid", False),
                     centroid_gain=k.get("centroid_gain", 22.6)),
                 group="video")
    # Proprioception.
    register("proprioceptive",
             lambda m, **k: ProprioceptiveEncoder(
                 projection_dim=k.get("projection_dim", 128)),
             group="proprio")


def _load_plugins() -> None:
    """Load external encoder providers via the ``xaq.encoders`` group."""
    try:
        from importlib.metadata import entry_points
    except Exception:  # pragma: no cover
        return
    try:
        eps = entry_points(group="xaq.encoders")
    except TypeError:  # Python < 3.10 API
        eps = entry_points().get("xaq.encoders", [])
    for ep in eps:
        try:
            ep.load()(register)
        except Exception as exc:  # a broken plugin must not break xaq
            import warnings
            warnings.warn(f"xaq.encoders plugin '{ep.name}' failed: {exc}")


_register_builtins()
_load_plugins()


def make_encoder(modality: str, projection_dim: int = 128,
                 sample_rate: int = 48000, inject_centroid: bool = False,
                 centroid_gain: float = 22.6):
    """Return the encoder registered for ``modality``.

    Resolution order:
      1. exact registered modality (built-in or plugin);
      2. if the name looks like audio but has no provider, the generic STFT;
      3. otherwise a generic JL projection (visual fallback).
    """
    kw = dict(projection_dim=projection_dim, sample_rate=sample_rate,
              inject_centroid=inject_centroid, centroid_gain=centroid_gain)

    factory = get_factory(modality)
    if factory is not None:
        return factory(modality, **kw)

    # Unknown modality: degrade sensibly instead of raising.
    audio_aliases = {"cochlear", "hubert", "tiny_ast", "stft", "mic"}
    if modality in audio_aliases or group_of(modality) == "audio":
        return FrozenSTFTEncoder(modality, projection_dim=projection_dim,
                                 sample_rate=sample_rate)
    return FrozenProjectionEncoder(modality=modality,
                                   projection_dim=projection_dim,
                                   inject_centroid=inject_centroid,
                                   centroid_gain=centroid_gain)
