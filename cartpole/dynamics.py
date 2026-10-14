"""Nonlinear cart-pole model in the lab's convention and its linearisation.

State s = [x, alpha, x_dot, alpha_dot]; alpha = 0 upright, counter-clockwise
positive (the PDF's equations L1, L2):

    M_tot x'' - ml cos(a) a'' + ml sin(a) a'^2 = F
    J a''     - ml cos(a) x''                 = m g l sin(a) - B_p a'
    F = k_amp a V - (b + B_f) x' - F_coul tanh(x'/v_c)

Every function accepts a single state (shape (4,)) or a batch (shape (N, 4)),
so the same code drives the real-time simulator and the vectorised RL envs.
Plant parameters may be scalars or arrays of shape (N,) for domain randomisation.
"""
from __future__ import annotations

import numpy as np

from .params import H, Plant

V_COUL = 0.01  # smoothing velocity of the Coulomb term [m/s]


class PlantArrays:
    """Plant parameters as (possibly batched) arrays, precomputed once."""

    def __init__(self, p: Plant | None = None, **over):
        p = p or Plant()
        def get(name, default):
            return np.asarray(over.get(name, default), dtype=float)
        self.g = p.g
        self.ml = get("ml", p.ml)
        self.M_tot = get("M_tot", p.M_tot)
        self.J = get("J", p.J)
        self.a = get("a", p.a)
        self.k_amp = get("k_amp", p.k_amp)
        self.c_v = get("c_v", p.b + p.B_f)          # total viscous [N s/m]
        self.B_p = get("B_p", p.B_p)
        self.F_coul = get("F_coul", p.F_coul)

    @classmethod
    def from_plant(cls, p: Plant) -> "PlantArrays":
        return cls(p)


def accel(s, V, P: PlantArrays):
    """Accelerations (x'', alpha'') for state(s) s and voltage(s) V."""
    s = np.asarray(s, dtype=float)
    xd, al, ad = s[..., 2], s[..., 1], s[..., 3]
    c, sn = np.cos(al), np.sin(al)
    F = P.k_amp * P.a * V - P.c_v * xd - P.F_coul * np.tanh(xd / V_COUL)
    r1 = F - P.ml * sn * ad * ad
    r2 = P.ml * P.g * sn - P.B_p * ad
    D = P.M_tot * P.J - (P.ml * c) ** 2
    xdd = (P.J * r1 + P.ml * c * r2) / D
    add = (P.ml * c * r1 + P.M_tot * r2) / D
    return xdd, add


def deriv(s, V, P: PlantArrays):
    xdd, add = accel(s, V, P)
    s = np.asarray(s, dtype=float)
    return np.stack([s[..., 2], s[..., 3], xdd, add], axis=-1)


def rk4(s, V, P: PlantArrays, dt: float = H):
    """One RK4 step with V held constant (zero-order hold)."""
    k1 = deriv(s, V, P)
    k2 = deriv(s + 0.5 * dt * k1, V, P)
    k3 = deriv(s + 0.5 * dt * k2, V, P)
    k4 = deriv(s + dt * k3, V, P)
    return s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def wrap(a):
    """Angle to (-pi, pi]."""
    return np.pi - np.mod(np.pi - a, 2 * np.pi)


def linearize(p: Plant):
    """Continuous A, B at upright (analytic, Coulomb term dropped)."""
    P = PlantArrays(p)
    M, J, ml, g = float(P.M_tot), float(P.J), float(P.ml), P.g
    c_v, Bp, ka = float(P.c_v), float(P.B_p), float(P.k_amp * P.a)
    D0 = M * J - ml ** 2
    A = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, ml * ml * g / D0, -J * c_v / D0, -ml * Bp / D0],
        [0, M * ml * g / D0, -ml * c_v / D0, -M * Bp / D0],
    ])
    B = np.array([[0.0], [0.0], [J * ka / D0], [ml * ka / D0]])
    return A, B


def linearize_numeric(p: Plant, eps: float = 1e-6):
    """Finite-difference Jacobian, used to check `linearize`."""
    P = PlantArrays(p)
    s0 = np.zeros(4)
    A = np.zeros((4, 4))
    for i in range(4):
        d = np.zeros(4); d[i] = eps
        A[:, i] = (deriv(s0 + d, 0.0, P) - deriv(s0 - d, 0.0, P)) / (2 * eps)
    B = ((deriv(s0, eps, P) - deriv(s0, -eps, P)) / (2 * eps)).reshape(4, 1)
    return A, B


def energy(s, p: Plant):
    """Pole energy relative to upright: 0 at rest upright, -2mgl hanging."""
    s = np.asarray(s, dtype=float)
    return 0.5 * p.J * s[..., 3] ** 2 + p.ml * p.g * (np.cos(s[..., 1]) - 1.0)
