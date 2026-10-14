"""Idle, manual, PID and full state feedback (pole placement, LQR)."""
from __future__ import annotations

import numpy as np

from .. import lin
from ..dynamics import linearize
from ..params import H
from .base import Controller, Param, Ref, deg


class Idle(Controller):
    key = "idle"
    label = "Idle"
    blurb = "Zero volts. The pole hangs."
    PARAMS = []

    def step(self, s, t, ref):
        return 0.0


class Manual(Controller):
    key = "manual"
    label = "Manual"
    blurb = ("Drive the cart yourself. 'voltage': the arrow keys or the slider set the "
             "motor voltage directly (the handout's mouse step). 'position': the slider "
             "is a cart setpoint and a PD loop follows it.")
    PARAMS = [
        Param("how", "Input", "voltage", kind="choice", choices=["voltage", "position"]),
        Param("v_max", "Max voltage", 4.0, 0.5, 10.0, 0.5, help="volts at full deflection"),
        Param("kp", "Position kp", 40.0, 0, 200, 1, group="position"),
        Param("kd", "Position kd", 4.0, 0, 50, 0.5, group="position"),
    ]

    def step(self, s, t, ref):
        if self.p["how"] == "position":
            return self.p["kp"] * (ref.x_ref - s[0]) - self.p["kd"] * s[2]
        return self.p["v_max"] * float(np.clip(ref.manual, -1, 1))


class SineTest(Controller):
    key = "sine_test"
    label = "Step 1: sinusoidal model test"
    blurb = "Course model-comparison input: 5 V at 10 Hz by default. The voltage cap and track limits still apply."
    PARAMS = [Param("amp", "Amplitude [V]", 5., 0., 10., .1),
              Param("freq", "Frequency [Hz]", 10., .1, 20., .1)]

    def reset(self, s):
        self.t0 = None

    def on_params(self):
        self.reset(None)

    def step(self, s, t, ref):
        if self.t0 is None:
            self.t0 = t
        self.info['phase'] = 'exciting'
        return self.p['amp']*np.sin(2*np.pi*self.p['freq']*(t-self.t0))


class Excite(Controller):
    key = "excite"
    label = "Identify (excitation)"
    blurb = ("Automates the handout's step 2. With the pole hanging, drives the cart with "
             "a slow chirp plus random steps around the centre, so the logged run has "
             "the motion system identification needs (cart and pole). Then run Sysid "
             "or Improve. Record 20–30 s of varied motion, then validate the fit. "
             "Start with reduced amplitude on the rig; track limits remain active.")
    PARAMS = [
        Param("amp", "Amplitude [V]", 3.0, 0.5, 6, 0.1),
        Param("f0", "Chirp start [Hz]", 0.3, 0.05, 5, 0.05),
        Param("f1", "Chirp end [Hz]", 4.0, 0.2, 10, 0.1),
        Param("period", "Sweep period [s]", 15.0, 3, 60, 1),
        Param("noise", "Random steps [V]", 1.5, 0, 5, 0.1),
        Param("kp", "Centering [V/m]", 25.0, 0, 100, 1),
        Param("kd", "Centering damping [V s/m]", 2.0, 0, 20, 0.5),
    ]

    def reset(self, s):
        self.t0 = None
        self.rng = np.random.default_rng(0)
        self.step_v = 0.0
        self.next_step = 0.0

    def on_params(self):
        if not hasattr(self, "rng"):
            self.reset(None)

    def step(self, s, t, ref):
        if self.t0 is None:
            self.t0 = t
        tau = (t - self.t0) % self.p["period"]
        f0, f1, T = self.p["f0"], self.p["f1"], self.p["period"]
        phase = 2 * np.pi * (f0 * tau + 0.5 * (f1 - f0) / T * tau * tau)
        if t >= self.next_step:
            self.step_v = self.rng.uniform(-1, 1) * self.p["noise"]
            self.next_step = t + self.rng.uniform(0.1, 0.6)
        self.info = {"phase": "exciting", "f_hz": round(f0 + (f1 - f0) * tau / T, 2)}
        return (self.p["amp"] * np.sin(phase) + self.step_v
                - self.p["kp"] * (s[0] - ref.x_ref) - self.p["kd"] * s[2])


class PID(Controller):
    key = "pid"
    label = "PID (two loops)"
    blurb = ("Handout step 3: one PID on the angle, one on the cart position, summed. "
             "u = Kp e + Ki int(e) + Kd de/dt with e = r - y, derivative from the "
             "estimator's filtered velocity. Only active inside the catch window; "
             "integrators are frozen and cleared while inactive. The position loop "
             "needs NEGATIVE gains: to move the cart right you first move it left.")
    PARAMS = [
        Param("kp_a", "Angle Kp", 42.0, 0, 120, 1, group="angle"),
        Param("ki_a", "Angle Ki", 0.0, 0, 2, 0.05, group="angle"),
        Param("kd_a", "Angle Kd", 5.5, 0, 60, 0.1, group="angle"),
        Param("kp_x", "Position Kp", -10.0, -60, 60, 0.5, group="position"),
        Param("ki_x", "Position Ki", 0.0, -5, 5, 0.05, group="position"),
        Param("kd_x", "Position Kd", -16.0, -60, 60, 0.5, group="position"),
        Param("window", "Active within [deg]", 5.0, 1, 45, 1),
    ]

    def reset(self, s):
        self.ia = self.ix = 0.0
        self.old_integrals = (0.0, 0.0)
        self.active = False

    def on_params(self):
        if not hasattr(self, "ia"):
            self.reset(None)

    def step(self, s, t, ref):
        x, a, xd, ad = s
        inside = abs(a) < deg(self.p["window"]) if not self.active else abs(a) < deg(max(self.p["window"], 25))
        if not inside:
            self.active = False
            self.ia = self.ix = 0.0
            self.info = {"phase": "waiting (lift the pole by hand)"}
            return 0.0
        self.active = True
        ea, ex = -a, ref.x_ref - x
        self.old_integrals = (self.ia, self.ix)
        self.ia += ea * H
        self.ix += ex * H
        u = (self.p["kp_a"] * ea + self.p["ki_a"] * self.ia - self.p["kd_a"] * ad
             + self.p["kp_x"] * ex + self.p["ki_x"] * self.ix - self.p["kd_x"] * xd)
        self.info = {"phase": "balancing"}
        return u

    def observe_voltage(self, requested, applied):
        if not self.active:
            return
        old_a, old_x = self.old_integrals
        increment = self.p["ki_a"]*(self.ia-old_a) + self.p["ki_x"]*(self.ix-old_x)
        if (requested-applied)*increment > 0:
            self.ia, self.ix = old_a, old_x


class StateFeedback(Controller):
    key = "statefb"
    label = "State feedback (LQR / poles)"
    blurb = ("Handout step 4: u = -K (s - s_ref). K from LQR (Q diagonal, R) or from "
             "pole placement (Ackermann). 'discrete' designs on the 500 Hz ZOH model "
             "(what actually runs); 'continuous' matches MATLAB's lqr()/place().")
    PARAMS = [
        Param("method", "Design", "lqr", kind="choice", choices=["lqr", "place"]),
        Param("time", "Time domain", "discrete", kind="choice", choices=["discrete", "continuous"]),
        Param("q_x", "Q x", 10.0, 0, 1e4, 1, group="lqr"),
        Param("q_a", "Q alpha", 40.0, 0, 1e4, 1, group="lqr"),
        Param("q_xd", "Q x_dot", 0.0, 0, 1e3, 0.1, group="lqr"),
        Param("q_ad", "Q alpha_dot", 0.0, 0, 1e3, 0.1, group="lqr"),
        Param("r", "R", 0.1, 1e-4, 100, 0.01, group="lqr"),
        Param("p1", "Pole 1", -5.0, -40, -0.1, 0.1, group="place"),
        Param("p2", "Pole 2", -6.0, -40, -0.1, 0.1, group="place"),
        Param("p3", "Pole 3", -7.0, -40, -0.1, 0.1, group="place"),
        Param("p4", "Pole 4", -8.0, -40, -0.1, 0.1, group="place"),
        Param("window", "Catch within [deg]", 5.0, 1, 45, 1,
              help="switches on inside this angle, off beyond 30 deg"),
    ]

    def on_params(self):
        self.K = design_gain(self.plant, self.p)
        A, B = linearize(self.plant)
        if self.p["time"] == "discrete":
            Ad, Bd = lin.c2d(A, B, H)
            cl = np.abs(np.linalg.eigvals(Ad - Bd @ self.K))
            poles = np.log(np.linalg.eigvals(Ad - Bd @ self.K).astype(complex)) / H
        else:
            poles = np.linalg.eigvals(A - B @ self.K)
        self.info = {"K": [round(float(k), 3) for k in self.K.ravel()],
                     "poles": [f"{z.real:.2f}{z.imag:+.2f}j" if abs(z.imag) > 1e-6 else f"{z.real:.2f}"
                               for z in np.sort_complex(np.asarray(poles, dtype=complex))]}
        self.active = False

    def reset(self, s):
        self.active = False

    def step(self, s, t, ref):
        a = s[1]
        lim_on, lim_off = deg(self.p["window"]), deg(30)
        self.active = abs(a) < (lim_off if self.active else lim_on)
        self.info["phase"] = "balancing" if self.active else "waiting (lift the pole by hand)"
        if not self.active:
            return 0.0
        e = s - np.array([ref.x_ref, 0.0, 0.0, 0.0])
        return float(-(self.K @ e)[0])


def design_gain(plant, p: dict) -> np.ndarray:
    """K for u = -K s from the parameter dict of StateFeedback (also used by
    the swing-up controllers to build their catcher)."""
    A, B = linearize(plant)
    if p.get("method", "lqr") == "place":
        poles = [p.get(k, d) for k, d in (("p1", -5), ("p2", -6), ("p3", -7), ("p4", -8))]
        if len(set(poles)) < 4:   # Ackermann tolerates repeats, nudge for conditioning
            poles = [q - 1e-3 * i for i, q in enumerate(poles)]
        K = lin.place(A, B, poles)
        if p.get("time", "discrete") == "discrete":
            Ad, Bd = lin.c2d(A, B, H)
            K = lin.place(Ad, Bd, np.exp(np.asarray(poles) * H))
        return np.atleast_2d(K)
    Q = np.diag([p.get("q_x", 10), p.get("q_a", 40), p.get("q_xd", 0), p.get("q_ad", 0)])
    R = np.array([[p.get("r", 0.1)]])
    if p.get("time", "discrete") == "discrete":
        Ad, Bd = lin.c2d(A, B, H)
        K, _ = lin.dlqr(Ad, Bd, Q * H, R * H)
    else:
        K, _ = lin.lqr(A, B, Q, R)
    return np.atleast_2d(K)


DEFAULT_CATCH = {"method": "lqr", "time": "discrete", "q_x": 10, "q_a": 40, "q_xd": 0, "q_ad": 0, "r": 0.1}
