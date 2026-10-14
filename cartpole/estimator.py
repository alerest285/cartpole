"""State estimation from the two encoders.

The lab models differentiate x and alpha through a filtered derivative
w^2 s / (s^2 + sqrt(2) w s + w^2) with w = 60 rad/s, discretised at 500 Hz. The
same filter is used here (Tustin), so the controllers see what the Simulink
controllers see. A Kalman option (constant-velocity model per axis) is offered
for less noisy velocities.

Inputs are x [m] and the *unwrapped* angle alpha_u [rad] (counts * k - pi, which
starts at -pi with the pole hanging). Output state: [x, wrap(alpha), x_dot, alpha_dot].
"""
from __future__ import annotations

import math

import numpy as np

from .dynamics import wrap
from .params import H


class FilteredDerivative:
    """Second-order filtered differentiator, Tustin-discretised biquad."""

    def __init__(self, w: float = 60.0, zeta: float = math.sqrt(0.5), h: float = H):
        K = 2.0 / h
        a0 = K * K + 2 * zeta * w * K + w * w
        self.b = np.array([w * w * K, 0.0, -w * w * K]) / a0
        self.a = np.array([1.0, (2 * w * w - 2 * K * K) / a0, (K * K - 2 * zeta * w * K + w * w) / a0])
        self.reset(0.0)

    def reset(self, u0: float):
        self.u1 = self.u2 = u0
        self.y1 = self.y2 = 0.0

    def __call__(self, u: float) -> float:
        b, a = self.b, self.a
        y = b[0] * u + b[1] * self.u1 + b[2] * self.u2 - a[1] * self.y1 - a[2] * self.y2
        self.u2, self.u1 = self.u1, u
        self.y2, self.y1 = self.y1, y
        return y


class CVKalman:
    """Steady-state-capable 2-state Kalman filter: position + velocity, white
    acceleration noise of density q, measurement variance r (quantisation)."""

    def __init__(self, q: float, r: float, h: float = H):
        self.F = np.array([[1.0, h], [0.0, 1.0]])
        self.Q = q * np.array([[h ** 3 / 3, h ** 2 / 2], [h ** 2 / 2, h]])
        self.r = r
        self.reset(0.0)

    def reset(self, z0: float):
        self.x = np.array([z0, 0.0])
        self.P = np.diag([self.r, 1.0])

    def __call__(self, z: float):
        x = self.F @ self.x
        P = self.F @ self.P @ self.F.T + self.Q
        S = P[0, 0] + self.r
        Kg = P[:, 0] / S
        x = x + Kg * (z - x[0])
        P = P - np.outer(Kg, P[0, :])
        self.x, self.P = x, P
        return x[0], x[1]


class Estimator:
    KINDS = ("derivative", "kalman")

    def __init__(self, kind: str = "kalman", w: float = 60.0,
                 q_x: float = 2.0, q_a: float = 0.3,
                 x_res: float = 2.2749e-5, a_res: float = 2 * math.pi / 4096):
        self.kind = kind
        self.w = w
        self.q_x, self.q_a = q_x, q_a
        self.x_res, self.a_res = x_res, a_res
        self._build()

    def _build(self):
        if self.kind == "kalman":
            self.fx = CVKalman(self.q_x, self.x_res ** 2 / 12)
            self.fa = CVKalman(self.q_a, self.a_res ** 2 / 12)
        else:
            self.fx = FilteredDerivative(self.w)
            self.fa = FilteredDerivative(self.w)
        self.started = False

    def configure(self, kind: str | None = None, w: float | None = None):
        if kind:
            self.kind = kind
        if w:
            self.w = float(w)
        self._build()

    def reset(self, x: float, alpha_u: float):
        self.fx.reset(x)
        self.fa.reset(alpha_u)
        self.started = True

    def __call__(self, x: float, alpha_u: float) -> np.ndarray:
        if not self.started:
            self.reset(x, alpha_u)
        if self.kind == "kalman":
            xf, xd = self.fx(x)
            af, ad = self.fa(alpha_u)
            return np.array([xf, float(wrap(af)), xd, ad])
        return np.array([x, float(wrap(alpha_u)), self.fx(x), self.fa(alpha_u)])
