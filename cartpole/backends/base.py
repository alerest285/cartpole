"""Backend interface: something with two encoders and one voltage output.

Both the simulator and the Quanser board speak raw encoder counts, so the
conversion counts -> (x, alpha) that runs on the rig is the one tested here.
"""
from __future__ import annotations

import math
import time

from ..params import H, Hardware


class Backend:
    name = "base"
    is_sim = False

    def __init__(self, hw: Hardware | None = None):
        self.hw = hw or Hardware()

    def open(self): ...
    def close(self): ...

    def read_counts(self) -> tuple[int, int]:
        raise NotImplementedError

    def write_volts(self, v: float):
        raise NotImplementedError

    def zero(self):
        """Declare the current pose as cart-centre / pole-hanging."""
        raise NotImplementedError

    def wait_tick(self):
        """Block until the next 2 ms sample."""
        raise NotImplementedError

    def info(self) -> dict:
        return {"name": self.name, "sim": self.is_sim}

    # --- conversion (shared) --------------------------------------------
    def counts_to_xa(self, cart: int, pole: int) -> tuple[float, float]:
        x = cart * self.hw.cart_m_per_count
        alpha_u = pole * self.hw.pole_rad_per_count - math.pi
        return x, alpha_u


class Pacer:
    """Software 500 Hz pacing: sleep most of the gap, spin the rest."""

    def __init__(self, period: float = H):
        self.period = period
        self.next = None
        self.overruns = 0

    def wait(self):
        now = time.perf_counter()
        if self.next is None:
            self.next = now + self.period
            return
        gap = self.next - now
        if gap < -self.period:          # fell behind by more than a tick: resync
            self.overruns += 1
            self.next = now + self.period
            return
        if gap > 0.0015:
            time.sleep(gap - 0.001)
        while time.perf_counter() < self.next:
            pass
        self.next += self.period
