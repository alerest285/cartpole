"""Controller registry."""
from __future__ import annotations

from ..params import Plant
from .base import Controller, Param, Ref
from .classic import PID, Excite, Idle, Manual, StateFeedback
from .swingup import EnergySwingUp, SineSwingUp

REGISTRY: dict[str, type[Controller]] = {}


def register(cls):
    REGISTRY[cls.key] = cls
    return cls


for _c in (Idle, Manual, PID, StateFeedback, EnergySwingUp, SineSwingUp, Excite):
    register(_c)

try:  # the learned controller needs the policy code; keep the rest usable without it
    from .learned import PPOController
    register(PPOController)
except ImportError:  # pragma: no cover
    pass

# Append new modes: old run files store registry indices.
from .swingup import ManualSwingUp
register(ManualSwingUp)
from .classic import SineTest
register(SineTest)


def make(key: str, plant: Plant) -> Controller:
    return REGISTRY[key](plant)


def schemas() -> list[dict]:
    return [c.schema() for c in REGISTRY.values()]
