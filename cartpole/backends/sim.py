"""Simulated rig: the nonlinear model behind a fake pair of encoders.

Sensor model: both encoders quantised to their real resolution. Actuator:
optional delay and the amplifier's +-10 V saturation. Starts like the lab:
cart centred, pole hanging, encoders reading zero.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from ..dynamics import PlantArrays, rk4
from ..params import H, Hardware, Plant
from .base import Backend, Pacer


class SimPlant:
    """The physics plus the encoder model, no timing."""

    def __init__(self, plant: Plant | None = None, hw: Hardware | None = None, substeps: int = 2):
        self.plant = plant or Plant()
        self.hw = hw or Hardware()
        self.P = PlantArrays(self.plant)
        self.substeps = substeps
        self.reset()

    def reset(self, s0=None):
        """Default start = the lab's: cart centred, pole hanging (alpha = -pi),
        encoders at zero. A custom s0 keeps zero offsets, so the reading equals
        the true state."""
        self.s = np.array(s0, dtype=float) if s0 is not None else np.array([0.0, -math.pi, 0.0, 0.0])
        self.queue = deque([0.0] * max(0, self.plant.delay_steps))
        self.c0 = 0
        self.p0 = 0

    def set_plant(self, plant: Plant):
        self.plant = plant
        self.P = PlantArrays(plant)

    def _raw_cart(self) -> int:
        return int(round(self.s[0] / self.hw.cart_m_per_count))

    def _raw_pole(self) -> int:
        return int(round((self.s[1] + math.pi) / self.hw.pole_rad_per_count))

    def counts(self) -> tuple[int, int]:
        return self._raw_cart() - self.c0, self._raw_pole() - self.p0

    def zero(self):
        """Like restarting a lab model: wherever the cart is becomes x = 0 and
        the pole is declared hanging. Wrong if the pole is not actually hanging,
        exactly as on the rig."""
        self.c0 = self._raw_cart()
        self.p0 = self._raw_pole()

    def step(self, v: float):
        v = float(np.clip(v, -self.hw.v_limit, self.hw.v_limit))
        if self.queue:
            self.queue.append(v)
            v = self.queue.popleft()
        dt = H / self.substeps
        for _ in range(self.substeps):
            self.s = rk4(self.s, v, self.P, dt)

    def poke(self, dv: float = 1.5):
        """A tap on the pole: an instant change of angular velocity."""
        self.s[3] += dv


class SimBackend(Backend):
    name = "sim"
    is_sim = True

    def __init__(self, plant: Plant | None = None, hw: Hardware | None = None, realtime: bool = True):
        super().__init__(hw)
        self.sim = SimPlant(plant, self.hw)
        self.realtime = realtime
        self.pacer = Pacer()
        self.v = 0.0

    def read_counts(self):
        return self.sim.counts()

    def write_volts(self, v: float):
        self.v = v

    def wait_tick(self):
        self.sim.step(self.v)
        if self.realtime:
            self.pacer.wait()

    def zero(self):
        self.sim.zero()

    def info(self):
        return {"name": "simulator", "sim": True, "overruns": self.pacer.overruns}
