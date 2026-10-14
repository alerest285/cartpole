"""Swing-up: energy pumping (handout's Lyapunov step) and the sinusoid, each
handing over to an LQR catcher near upright."""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from ..dynamics import PlantArrays, accel
from .base import Controller, Param, Ref, deg
from .classic import DEFAULT_CATCH, design_gain

CATCH_PARAMS = [
    Param("catch_deg", "Catch inside [deg]", 35.0, 5, 45, 1, group="catch"),
    Param("drop_deg", "Give up beyond [deg]", 45.0, 15, 90, 1, group="catch"),
    Param("cq_x", "Catch Q x", 5.0, 0, 1e4, 1, group="catch"),
    Param("cq_a", "Catch Q alpha", 100.0, 0, 1e4, 1, group="catch"),
    Param("cr", "Catch R", 0.05, 1e-3, 100, 0.01, group="catch"),
]


class Catching(Controller):
    """Base for anything that swings up and then hands over to LQR."""

    def on_params(self):
        self.Kc = design_gain(self.plant, {**DEFAULT_CATCH, "q_x": self.p["cq_x"],
                                           "q_a": self.p["cq_a"], "r": self.p["cr"]})
        self.P = PlantArrays(self.plant)
        self.caught = False

    def reset(self, s):
        self.caught = False
        self.t0 = None

    def catch_or(self, s, ref, swing_fn):
        a = abs(s[1])
        if self.caught and a > deg(self.p["drop_deg"]):
            self.caught = False
        elif not self.caught and a < deg(self.p["catch_deg"]):
            self.caught = True
        if self.caught:
            self.info["phase"] = "balancing"
            e = s - np.array([ref.x_ref, 0, 0, 0])
            return float(-(self.Kc @ e)[0])
        self.info["phase"] = "swinging"
        return swing_fn()

    def volts_for_accel(self, s, xdd_des: float) -> float:
        """Invert the cart equation: the voltage that gives x'' = xdd_des now.
        x'' is affine in V, so two model evaluations give it exactly."""
        x0, _ = accel(s, 0.0, self.P)
        x1, _ = accel(s, 1.0, self.P)
        return float((xdd_des - x0) / (x1 - x0))


class ManualSwingUp(Catching):
    key = "manual_swing"
    label = "Swing-up: mouse / keys + LQR"
    blurb = ("Handout step 5: drag in the manual pad, use the slider or arrow keys to pump "
             "the pendulum. LQR takes over inside the catch angle. Releasing the input "
             "sets the manual voltage to zero; STOP always disarms.")
    PARAMS = [Param("v_max", "Manual voltage [V]", 4., .5, 10., .5)] + CATCH_PARAMS

    def step(self, s, t, ref):
        return self.catch_or(s, ref, lambda: self.p["v_max"]*float(np.clip(ref.manual, -1, 1)))


class EnergySwingUp(Catching):
    key = "energy"
    label = "Swing-up: energy (Lyapunov)"
    blurb = ("Handout step 5. E = 1/2 J a'^2 / mgl + cos a - 1 is 0 upright and -2 "
             "hanging; dE/dt = (ml/mgl) cos(a) a' x''. With w = a' cos(a), the pump "
             "x'' = k (E_t - E) w only ever adds energy until E = E_t. Cart centering "
             "is weighted by eps/(w^2 + eps), so it acts only when it cannot change the "
             "pole's energy (pole at the top of its swing or crossing horizontal). The "
             "desired x'' becomes volts by inverting the model; near upright an LQR "
             "takes over.")
    PARAMS = [
        Param("k_e", "Energy gain", 5.0, 0, 50, 0.1, help="m/s^2 per (mgl x rad/s)"),
        Param("a_max", "Max pumping accel [m/s^2]", 5.0, 0.5, 20, 0.5),
        Param("k_x", "Centering stiffness [1/s^2]", 2.0, 0, 200, 1),
        Param("k_v", "Centering damping [1/s]", 2.0, 0, 50, 0.5),
        Param("eps", "Centering window [(rad/s)^2]", 3.0, 0.01, 100, 0.1,
              help="larger = centering acts more often, at some energy cost"),
        Param("kick", "Start kick [m/s^2]", 2.0, 0, 10, 0.5,
              help="pushes once when hanging still, energy pumping needs motion"),
        Param("e_over", "Energy target [x mgl]", 0.0, -0.5, 0.5, 0.01,
              help="aim slightly above upright energy to arrive with margin"),
        Param("adapt", "Adapt target per swing", 0.0, 0, 2, 0.05,
              help="raise the target by this x the energy an apex was short; 0 = off"),
    ] + CATCH_PARAMS

    def reset(self, s):
        super().reset(s)
        self.bias = 0.0
        self.prev_ad = 0.0

    def step(self, s, t, ref):
        def swing():
            x, a, xd, ad = s
            pl = self.plant
            E = (0.5 * pl.J * ad * ad) / (pl.ml * pl.g) + (math.cos(a) - 1.0)
            bias = getattr(self, "bias", 0.0)
            # apex in the upper half that missed the catch window: raise the target
            if self.prev_ad * ad < 0 and abs(a) < math.pi / 2 and abs(a) > deg(self.p["catch_deg"]):
                bias = min(0.5, bias + self.p["adapt"] * (1.0 - math.cos(a)) * 0.5)
            self.prev_ad = ad
            self.bias = bias
            e_t = self.p["e_over"] + bias
            self.info["energy"] = round(E, 3)
            self.info["target"] = round(e_t, 3)
            w = ad * math.cos(a)
            if abs(ad) < 0.3 and abs(a) > deg(170) and E < -1.9:
                pump = self.p["kick"]
            else:
                pump = self.p["k_e"] * (e_t - E) * w
            pump = float(np.clip(pump, -self.p["a_max"], self.p["a_max"]))
            centre = -self.p["k_x"] * (x - ref.x_ref) - self.p["k_v"] * xd
            acc = pump + centre * self.p["eps"] / (w * w + self.p["eps"])
            return self.volts_for_accel(s, acc)
        return self.catch_or(s, ref, swing)


def _ellipk(k: float) -> float:
    """Complete elliptic integral of the first kind K(k) by the AGM."""
    a, b = 1.0, math.sqrt(max(1e-12, 1.0 - k * k))
    for _ in range(30):
        a, b = 0.5 * (a + b), math.sqrt(a * b)
    return math.pi / (2 * a)


class SineSwingUp(Catching):
    key = "sine"
    label = "Swing-up: sinusoid"
    blurb = ("Handout step 5, second method. u = A sin(phase), three ways to get the "
             "phase. 'fixed': the handout's open-loop sinusoid at one frequency; it "
             "stalls because a pendulum slows down as it swings higher. 'tracking': "
             "open loop, but the frequency follows the swing amplitude th0 (read from "
             "the energy) as w0 pi / (2 K(sin(th0/2))). 'locked': the phase is the "
             "pendulum's own (phi = th0 cos psi, phi' = -th0 w sin psi), so the drive "
             "stays in resonance by construction; in time it is still a sinusoid of "
             "amplitude A. An LQR catches near upright.")
    PARAMS = [
        Param("how", "Phase from", "locked", kind="choice", choices=["locked", "tracking", "fixed"]),
        Param("amp", "Amplitude [V]", 8.0, 0, 10, 0.1),
        Param("freq", "Small-swing frequency [Hz]", 1.05, 0.3, 3, 0.01),
        Param("lead", "Phase lead [deg]", -40.0, -180, 180, 1, help="locked only"),
        Param("k_x", "Centering [V/m]", 5.0, 0, 100, 1),
        Param("k_v", "Damping [V s/m]", 1.0, 0, 30, 0.5),
    ] + [replace(q, default=2.0) if q.name == "cq_x" else q for q in CATCH_PARAMS]

    def reset(self, s):
        super().reset(s)
        self.phase = 0.0
        self.t_last = None

    def step(self, s, t, ref):
        if getattr(self, "t_last", None) is None:
            self.t_last, self.phase = t, 0.0
        dt = t - self.t_last
        self.t_last = t
        x, a, xd, ad = s
        pl = self.plant
        w0 = 2 * math.pi * self.p["freq"]
        e_h = 0.5 * pl.J * ad * ad / (pl.ml * pl.g) + (1.0 + math.cos(a))   # 0 hanging, 2 upright
        th0 = math.acos(max(-1.0, min(1.0, 1.0 - e_h)))
        w = w0 * math.pi / (2 * _ellipk(math.sin(th0 / 2))) if self.p["how"] != "fixed" else w0
        self.info["swing_deg"] = round(math.degrees(th0), 1)
        self.info["freq_hz"] = round(w / (2 * math.pi), 3)
        if self.p["how"] == "locked":
            phi = math.pi - ((math.pi - (a - math.pi)) % (2 * math.pi))       # 0 hanging
            psi = math.atan2(-ad / w, phi) if th0 > 1e-3 else 0.0
            if th0 < math.radians(2):                                         # nothing to lock to yet
                psi = self.phase
            phase = psi + math.radians(self.p["lead"])
        else:
            phase = self.phase
        self.phase += w * dt
        def swing():
            u = self.p["amp"] * math.sin(phase)
            return u - self.p["k_x"] * (x - ref.x_ref) - self.p["k_v"] * xd
        return self.catch_or(s, ref, swing)
