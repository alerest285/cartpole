"""Controller interface.

A controller maps the estimated state s = [x, alpha, x_dot, alpha_dot]
(alpha wrapped, 0 upright) to a motor voltage. It declares its tunable
parameters as a schema; the web UI renders the schema, so adding a parameter
here is all it takes to make it tunable live.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..params import Plant


@dataclass
class Param:
    name: str
    label: str
    default: float
    lo: float = -1e9
    hi: float = 1e9
    step: float = 0.1
    kind: str = "float"          # float | int | bool | choice
    choices: list = field(default_factory=list)
    group: str = ""
    help: str = ""

    def coerce(self, v):
        if self.kind == "bool":
            return bool(v)
        if self.kind == "choice":
            v = str(v)
            return v if v in self.choices else self.default
        v = float(v)
        v = min(max(v, self.lo), self.hi)
        return int(round(v)) if self.kind == "int" else v

    def schema(self) -> dict:
        return {k: getattr(self, k) for k in
                ("name", "label", "default", "lo", "hi", "step", "kind", "choices", "group", "help")}


@dataclass
class Ref:
    """Operator inputs that arrive from the UI."""
    x_ref: float = 0.0        # cart setpoint [m]
    manual: float = 0.0       # manual command in [-1, 1]


class Controller:
    key = "base"
    label = "Base"
    blurb = ""
    PARAMS: list[Param] = []

    def __init__(self, plant: Plant):
        self.plant = plant
        self.p = {q.name: q.default for q in self.PARAMS}
        self.info: dict = {}
        self.on_params()

    # ------------------------------------------------------------------
    def set_params(self, values: dict):
        byname = {q.name: q for q in self.PARAMS}
        for k, v in (values or {}).items():
            if k in byname:
                self.p[k] = byname[k].coerce(v)
        self.on_params()

    def set_plant(self, plant: Plant):
        self.plant = plant
        self.on_params()

    def on_params(self):
        """Recompute derived quantities (gains) after a parameter change."""

    def reset(self, s: np.ndarray):
        """Called when the controller is (re)activated."""

    def step(self, s: np.ndarray, t: float, ref: Ref) -> float:
        raise NotImplementedError

    def observe_voltage(self, requested: float, applied: float):
        """Optional anti-windup feedback from the common safety/output layer."""

    # ------------------------------------------------------------------
    @classmethod
    def schema(cls) -> dict:
        return {"key": cls.key, "label": cls.label, "blurb": cls.blurb,
                "params": [q.schema() for q in cls.PARAMS]}

    def state(self) -> dict:
        return {"key": self.key, "params": dict(self.p), "info": self.info}


def deg(x: float) -> float:
    return float(np.deg2rad(x))
