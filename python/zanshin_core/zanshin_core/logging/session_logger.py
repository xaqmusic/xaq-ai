"""Substrate logging protocol.

The engine records session events (node bakes, mode changes, labels) through an
`EventLogger`. Zanshin ships only the no-op `NullSessionLogger`; the private
AMI-Awen product injects a real logger (Audacity label tracks + Dream-mode
replay). Keeping the protocol here lets `brain_v3` depend on the substrate, not
on any audio/Dreamer implementation.
"""

from typing import Any, Optional

try:
    from typing import Protocol, runtime_checkable
except ImportError:  # pragma: no cover
    Protocol = object  # type: ignore

    def runtime_checkable(x):  # type: ignore
        return x


@runtime_checkable
class EventLogger(Protocol):
    """Minimal event-logging surface the engine relies on."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def log_event(self, name: str, t: Optional[float] = None,
                  **fields: Any) -> None: ...


class NullSessionLogger:
    """Inert logger — the Zanshin default. Every call is a no-op."""

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def log_event(self, name: str, t: Optional[float] = None, **fields: Any) -> None:
        pass

    # Tolerate any other method the audio product's logger exposes.
    def __getattr__(self, _name: str):
        def _noop(*_args, **_kwargs):
            return None
        return _noop
