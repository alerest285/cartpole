"""The layer every voltage passes through, whatever the controller.

1. Voltage cap (<= hardware limit).
2. Track barrier (control barrier function, handout step 6): with
   h = x_soft - x >= 0 and the exponential CBF h'' + k1 h' + k0 h >= 0,
   x'' <= k0 (x_soft - x) - k1 x'. x'' is affine in V (exact for the model), so
   the QP min |V - V_des|^2 s.t. the two half-space constraints is a clamp.
3. Hard faults -> zero volts and disarm: |x| beyond x_hard (inside the lab's
   0.2 m watchdog), encoder jumps, NaNs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .dynamics import PlantArrays, accel
from .params import Plant


@dataclass
class SafetyConfig:
    v_cap: float = 10.0
    cbf: bool = True
    x_soft: float = 0.17
    wn: float = 12.0           # barrier bandwidth: k0 = wn^2, k1 = 2 wn
    x_hard: float = 0.19
    max_ad: float = 60.0       # rad/s; beyond this is an encoder fault


class Safety:
    def __init__(self, plant: Plant, cfg: SafetyConfig | None = None):
        self.cfg = cfg or SafetyConfig()
        self.set_plant(plant)
        self.last = {"clamped": False, "fault": None}

    def set_plant(self, plant: Plant):
        self.P = PlantArrays(plant)

    def bounds(self, s) -> tuple[float, float]:
        c = self.cfg
        lo, hi = -c.v_cap, c.v_cap
        if not c.cbf:
            return lo, hi
        x, xd = s[0], s[2]
        k0, k1 = c.wn ** 2, 2 * c.wn
        a0, _ = accel(s, 0.0, self.P)
        a1, _ = accel(s, 1.0, self.P)
        beta = float(a1 - a0)                       # > 0 always
        up = (k0 * (c.x_soft - x) - k1 * xd - float(a0)) / beta
        dn = (-k0 * (c.x_soft + x) - k1 * xd - float(a0)) / beta
        if dn > up:                                 # infeasible: brake toward centre
            mid = 0.5 * (up + dn)
            return mid, mid
        return max(lo, dn), min(hi, up)

    def fault(self, s) -> str | None:
        if not np.all(np.isfinite(s)):
            return "non-finite state"
        if abs(s[0]) > self.cfg.x_hard:
            return f"cart at {s[0]*100:+.1f} cm, beyond the {self.cfg.x_hard*100:.1f} cm hard limit"
        if abs(s[3]) > self.cfg.max_ad:
            return f"pole speed {s[3]:.0f} rad/s, encoder fault?"
        return None

    def __call__(self, s, v_des: float) -> tuple[float, str | None]:
        f = self.fault(s)
        if f:
            self.last = {"clamped": True, "fault": f}
            return 0.0, f
        lo, hi = self.bounds(s)
        lo, hi = max(lo, -self.cfg.v_cap), min(hi, self.cfg.v_cap)
        if lo > hi:
            # The model may demand more braking than the amplifier can supply.
            # Apply maximum available braking, never exceed the voltage cap.
            lo = hi = float(np.clip(0.5 * (lo + hi), -self.cfg.v_cap, self.cfg.v_cap))
        v = float(min(max(v_des, lo), hi)) if math.isfinite(v_des) else 0.0
        self.last = {"clamped": abs(v - v_des) > 1e-6, "fault": None}
        return v, None
