"""Encoder registry — the decoupling seam between Zanshin and plugins.

Zanshin registers its built-in generic encoders here. Third-party packages
(e.g. the private AMI-Awen audio product) contribute additional encoders via
the setuptools entry-point group ``zanshin.encoders`` — Zanshin never imports
or names them. Each encoder self-declares a modality *group* ("audio", "video",
"proprio", ...) so consumers like the LateralVoter can reason about groups
without hard-coding modality names.
"""

from typing import Callable, Dict

# modality/tag -> factory(modality, **kwargs) -> encoder
_REGISTRY: Dict[str, Callable] = {}
# modality/tag -> group name
_GROUPS: Dict[str, str] = {}


def register(name: str, factory: Callable, group: str = "other") -> None:
    _REGISTRY[name] = factory
    _GROUPS[name] = group


def get_factory(name: str):
    return _REGISTRY.get(name)


def group_of(name: str) -> str:
    return _GROUPS.get(name, "other")


def groups() -> Dict[str, set]:
    """Return {group: {modality, ...}} — used to derive LateralVoter groupings."""
    out: Dict[str, set] = {}
    for name, grp in _GROUPS.items():
        out.setdefault(grp, set()).add(name)
    return out


def registered() -> Dict[str, str]:
    return dict(_GROUPS)
