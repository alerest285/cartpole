"""Vectorised cart-pole environment for PPO, built to match deployment:

  - the same nonlinear model (dynamics.accel), integrated at the 500 Hz tick;
  - the same encoder quantisation and the same Kalman estimator (its
    steady-state gain, vectorised), so the policy sees what it will see on the rig;
  - the same barrier filter (safety.py) between policy and motor;
  - domain randomisation around a centre plant (the latest sysid fit).

The policy acts at 50 Hz (every 10 ticks), output in [-1, 1] x v_max.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..dynamics import PlantArrays, accel, rk4
from ..estimator import CVKalman
from ..params import H, Hardware, Plant
from ..safety import SafetyConfig
from .policy import obs_from_state

DECIM = 10


@dataclass
class Randomisation:
    """Multiplicative (lo, hi) ranges around the centre plant, plus absolute ones."""
    c_v: tuple = (0.8, 1.25)       # total viscous (back-EMF + B_f)
    k_amp: tuple = (0.9, 1.1)
    M_tot: tuple = (0.95, 1.05)
    B_p: tuple = (0.5, 2.5)
    F_coul: tuple = (0.0, 0.3)     # absolute [N]
    delay_max: int = 1             # ticks

    def scaled(self, f: float) -> "Randomisation":
        """Shrink every range toward 1 by factor f (0 = no randomisation)."""
        def s(r):
            return (1 + (r[0] - 1) * f, 1 + (r[1] - 1) * f)
        return Randomisation(s(self.c_v), s(self.k_amp), s(self.M_tot), s(self.B_p),
                             (self.F_coul[0] * f, self.F_coul[1] * f), self.delay_max)


def steady_kalman_gain(q, r, n=4000):
    k = CVKalman(q, r)
    for _ in range(n):
        k(0.0)
    # one more predict/update to read the gain
    P = k.F @ k.P @ k.F.T + k.Q
    return P[:, 0] / (P[0, 0] + k.r)


def reward(s, u, prev_u, x_fail: float = 0.18):
    """Per policy step, from the state reached. Shared by the simulator and the
    hardware-episode update (improve.py), so both optimise the same thing."""
    s = np.atleast_2d(s)
    a = np.pi - np.mod(np.pi - s[:, 1], 2 * np.pi)
    up = 0.5 * (1 + np.cos(a))
    r = up ** 2
    r = r - 0.5 * (s[:, 0] / x_fail) ** 2
    r = r - 0.02 * u ** 2 - 0.02 * (u - prev_u) ** 2
    near = np.abs(a) < np.deg2rad(12)
    r = r + near * (0.5 - 0.02 * s[:, 3] ** 2 - 0.2 * s[:, 2] ** 2)
    failed = np.abs(s[:, 0]) > x_fail
    r = np.where(failed, r - 25.0, r)
    return r, failed, near


@dataclass
class EnvConfig:
    n: int = 64
    v_max: float = 8.0
    T: float = 8.0
    p_hanging: float = 0.7          # fraction of episodes that start hanging
    rand: Randomisation = field(default_factory=Randomisation)
    rand_scale: float = 1.0
    x_fail: float = 0.18
    safety: SafetyConfig = field(default_factory=SafetyConfig)


class VecEnv:
    def __init__(self, centre: Plant, cfg: EnvConfig | None = None, seed: int = 0):
        self.c = centre
        self.cfg = cfg or EnvConfig()
        self.rng = np.random.default_rng(seed)
        self.hw = Hardware()
        n = self.cfg.n
        self.n = n
        est = __import__("cartpole.estimator", fromlist=["Estimator"]).Estimator()
        self.Kx = steady_kalman_gain(est.q_x, est.x_res ** 2 / 12)
        self.Ka = steady_kalman_gain(est.q_a, est.a_res ** 2 / 12)
        self.steps = int(round(self.cfg.T / (H * DECIM)))
        self.s = np.zeros((n, 4))
        self.zh = np.zeros((n, 4))       # estimator state: [x, xd, a_u, ad]
        self.prev_u = np.zeros(n)
        self.t = np.zeros(n, dtype=int)
        self.P = None
        self.delay = np.zeros(n, dtype=int)
        self.vq = np.zeros((n, 2))       # last two applied volts (delay line)
        self.reset_all()

    # ------------------------------------------------------------ plants
    def _sample_plants(self, idx):
        r = self.cfg.rand.scaled(self.cfg.rand_scale)
        base = PlantArrays(self.c)
        u = lambda lo_hi: self.rng.uniform(lo_hi[0], lo_hi[1], len(idx))  # noqa: E731
        if self.P is None:
            self.P = PlantArrays(self.c, ml=np.full(self.n, float(base.ml)), M_tot=np.full(self.n, float(base.M_tot)),
                                 J=np.full(self.n, float(base.J)), a=np.full(self.n, float(base.a)),
                                 k_amp=np.full(self.n, float(base.k_amp)), c_v=np.full(self.n, float(base.c_v)),
                                 B_p=np.full(self.n, float(base.B_p)), F_coul=np.full(self.n, float(base.F_coul)))
        self.P.c_v[idx] = float(base.c_v) * u(r.c_v)
        self.P.k_amp[idx] = float(base.k_amp) * u(r.k_amp)
        self.P.M_tot[idx] = float(base.M_tot) * u(r.M_tot)
        self.P.B_p[idx] = max(float(base.B_p), 1e-4) * u(r.B_p)
        self.P.F_coul[idx] = float(base.F_coul) + u(r.F_coul)
        self.delay[idx] = self.rng.integers(0, r.delay_max + 1, len(idx))

    # ------------------------------------------------------------ resets
    def reset_all(self):
        return self.reset(np.arange(self.n))

    def reset(self, idx):
        idx = np.asarray(idx)
        if len(idx) == 0:
            return
        self._sample_plants(idx)
        k = len(idx)
        hang = self.rng.random(k) < self.cfg.p_hanging
        s = np.zeros((k, 4))
        s[:, 0] = self.rng.uniform(-0.05, 0.05, k)
        s[:, 1] = np.where(hang, -np.pi + self.rng.normal(0, 0.05, k), self.rng.uniform(-np.pi, np.pi, k))
        s[:, 2] = np.where(hang, 0.0, self.rng.normal(0, 0.2, k))
        s[:, 3] = np.where(hang, self.rng.normal(0, 0.1, k), self.rng.normal(0, 2.0, k))
        self.s[idx] = s
        self.zh[idx] = np.stack([s[:, 0], s[:, 2], s[:, 1], s[:, 3]], 1)
        self.prev_u[idx] = 0.0
        self.t[idx] = 0
        self.vq[idx] = 0.0

    # ------------------------------------------------------------- pieces
    def _measure(self):
        """Quantised encoders -> steady-state Kalman, vectorised."""
        xr, ar = self.hw.cart_m_per_count, self.hw.pole_rad_per_count
        xm = np.round(self.s[:, 0] / xr) * xr
        am = np.round((self.s[:, 1] + math.pi) / ar) * ar - math.pi
        z = self.zh
        # predict
        z[:, 0] += H * z[:, 1]
        z[:, 2] += H * z[:, 3]
        ex = xm - z[:, 0]
        ea = am - z[:, 2]
        z[:, 0] += self.Kx[0] * ex; z[:, 1] += self.Kx[1] * ex
        z[:, 2] += self.Ka[0] * ea; z[:, 3] += self.Ka[1] * ea

    def est_state(self):
        z = self.zh
        a = np.pi - np.mod(np.pi - z[:, 2], 2 * np.pi)
        return np.stack([z[:, 0], a, z[:, 1], z[:, 3]], 1)

    def _barrier(self, s, v):
        c = self.cfg.safety
        if not c.cbf:
            return np.clip(v, -c.v_cap, c.v_cap)
        k0, k1 = c.wn ** 2, 2 * c.wn
        a0, _ = accel(s, 0.0, self.P)
        a1, _ = accel(s, 1.0, self.P)
        beta = a1 - a0
        up = (k0 * (c.x_soft - s[:, 0]) - k1 * s[:, 2] - a0) / beta
        dn = (-k0 * (c.x_soft + s[:, 0]) - k1 * s[:, 2] - a0) / beta
        lo = np.maximum(dn, -c.v_cap); hi = np.minimum(up, c.v_cap)
        bad = lo > hi
        mid = 0.5 * (lo + hi)
        lo = np.where(bad, mid, lo); hi = np.where(bad, mid, hi)
        return np.clip(v, lo, hi)

    def obs(self):
        return obs_from_state(self.est_state(), self.prev_u)

    # -------------------------------------------------------------- step
    def step(self, u):
        """u in [-1, 1] (clipped). Returns obs, reward, done, truncated, info."""
        u = np.clip(u, -1, 1)
        v_des = u * self.cfg.v_max
        r = np.zeros(self.n)
        for _ in range(DECIM):
            sh = self.est_state()
            v = self._barrier(sh, v_des)              # barrier sees the estimate, like on the rig
            self.vq = np.roll(self.vq, 1, axis=1); self.vq[:, 0] = v
            v_app = np.where(self.delay == 0, v, self.vq[:, 1])
            self.s = rk4(self.s, v_app, self.P, H)
            self._measure()
        s = self.s
        r, failed, near = reward(s, u, self.prev_u, self.cfg.x_fail)
        a = np.pi - np.mod(np.pi - s[:, 1], 2 * np.pi)
        self.prev_u = u
        self.t += 1
        trunc = self.t >= self.steps
        done = failed | trunc
        info = {"failed": failed, "upright": near & (np.abs(s[:, 3]) < 1.5), "trunc": trunc & ~failed}
        return self.obs(), r, failed, trunc & ~failed, info
