"""Headless closed loop, faster than real time: true plant -> encoders ->
estimator -> controller -> safety -> plant. Used by tests, tuning and the
controller benchmark (`python -m cartpole bench`)."""
from __future__ import annotations

import math

import numpy as np

from .backends.sim import SimPlant
from .controllers import make
from .controllers.base import Ref
from .estimator import Estimator
from .params import H, Hardware, Plant
from .safety import Safety, SafetyConfig


def closed_loop(ctrl, plant_true: Plant, T: float, s0=None, est: str = "kalman",
                safety: SafetyConfig | None = None, x_ref=lambda t: 0.0,
                pokes=(), stop_on_fault: bool = True, sensor_bias=(0.0, 0.0),
                motor_deadband=0.0, manual=lambda t: 0.0):
    hw = Hardware()
    sim = SimPlant(plant_true, hw)
    sim.reset(s0)
    estimator = Estimator(est)
    saf = Safety(ctrl.plant, safety or SafetyConfig())
    n = int(round(T / H))
    log = np.zeros((n, 10))
    ctrl.reset(None)
    pokes = sorted(pokes)
    fault = None
    ref = Ref()
    for k in range(n):
        t = k * H
        while pokes and pokes[0][0] <= t:
            sim.poke(pokes.pop(0)[1])
        c, p = sim.counts()
        x, au = c * hw.cart_m_per_count, p * hw.pole_rad_per_count - math.pi
        x += sensor_bias[0]; au += sensor_bias[1]
        s_hat = estimator(x, au) if est != "true" else sim.s.copy()
        if est == "true":
            s_hat[1] = float(np.pi - np.mod(np.pi - s_hat[1], 2 * np.pi))
        ref.x_ref = x_ref(t)
        ref.manual = manual(t)
        v_des = ctrl.step(s_hat, t, ref)
        v, f = saf(s_hat, v_des)
        ctrl.observe_voltage(v_des, v)
        log[k] = [t, *sim.s, *s_hat[2:4], v_des, v, 1.0 if ctrl.info.get("phase") == "balancing" else 0.0]
        if f:
            fault = (t, f)
            if stop_on_fault:
                log = log[:k + 1]
                break
        # Test-only actuator mismatch; logged v remains the commanded voltage.
        applied = np.sign(v) * max(0.0, abs(v) - motor_deadband)
        sim.step(applied)
    return log, fault


COLS = ["t", "x", "alpha", "xd", "ad", "xd_hat", "ad_hat", "v_des", "v", "balancing"]


def summary(log, fault=None) -> dict:
    t, x, a, v, bal = log[:, 0], log[:, 1], log[:, 2], log[:, 8], log[:, 9]
    aw = np.abs(np.pi - np.mod(np.pi - a, 2 * np.pi))
    up = np.where((aw < np.deg2rad(10)) & (bal > 0))[0]
    t_up = float(t[up[0]]) if len(up) else None
    last = t >= t[-1] - 2.0
    return {
        "T": float(t[-1]),
        "fault": fault[1] if fault else None,
        "t_upright": t_up,
        "balanced_at_end": bool(np.all(aw[last] < np.deg2rad(5))) and fault is None,
        "max_abs_x_cm": float(np.abs(x).max() * 100),
        "end_abs_alpha_deg": float(np.rad2deg(aw[-1])),
        "rms_alpha_end_deg": float(np.rad2deg(np.sqrt(np.mean(aw[last] ** 2)))),
        "max_abs_v": float(np.abs(v).max()),
    }


def run(key: str, plant_model: Plant, plant_true: Plant, T: float, params=None, **kw):
    ctrl = make(key, plant_model)
    if params:
        ctrl.set_params(params)
    log, fault = closed_loop(ctrl, plant_true, T, **kw)
    return log, summary(log, fault), ctrl
