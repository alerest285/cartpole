"""Physical parameters and the hardware map.

Sources:
  - handout PS_Mission_Impossible_CartPole4_0.pdf, table 2, and system_setup.m
  - the course Simulink models (R2024b), read on 2026-10-01: Q2-USB board 0,
    encoders [0 1], analog out 0, digital out [0:3] = 1 enables the VoltPAQ,
    x = -counts * 2.2749e-5, alpha = mod(counts * 2*pi/4096, 2*pi) - pi,
    500 Hz, watchdog at |x| >= 0.2 m.
  - B_f, B_p and F_coul are priors, not measurements; `sysid` replaces them
    with values fitted from logged runs (data/params_fitted.json).
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("CARTPOLE_DATA", ROOT / "data"))
RUNS = Path(os.environ.get("CARTPOLE_RUNS", ROOT / "runs"))
FITTED = DATA / "params_fitted.json"

H = 0.002  # control period [s], same as the lab models


@dataclass
class Plant:
    g: float = 9.81
    m_p: float = 0.127        # pole mass [kg]
    m_c: float = 0.57         # cart mass [kg]
    l: float = 0.1778         # pivot to pole centre of mass [m]
    I_factor: float = 1.0 / 3.0  # I = I_factor * m_p * l^2 (uniform rod)
    K_g: float = 3.71
    K_m: float = 0.00767
    K_t: float = 0.00767
    J_m: float = 3.9e-7
    r_mp: float = 6.35e-3
    R_m: float = 2.6
    # Things the handout does not give. Priors, refit by sysid.
    B_f: float = 2.0          # cart viscous friction on top of back-EMF [N s/m]
    B_p: float = 0.0024       # pole bearing friction [N m s/rad]
    F_coul: float = 0.0       # cart Coulomb friction [N]
    k_amp: float = 1.0        # actual volts per commanded volt
    rotor_mass: bool = True   # include J_m reflected through the gear (0.133 kg)
    delay_steps: int = 0      # actuation delay in control periods

    # --- derived -----------------------------------------------------------
    @property
    def a(self) -> float:
        """Force per volt [N/V]."""
        return self.K_g * self.K_t / (self.R_m * self.r_mp)

    @property
    def b(self) -> float:
        """Back-EMF damping [N s/m]."""
        return self.K_g ** 2 * self.K_t * self.K_m / (self.R_m * self.r_mp ** 2)

    @property
    def m_rot(self) -> float:
        return self.J_m * self.K_g ** 2 / self.r_mp ** 2 if self.rotor_mass else 0.0

    @property
    def M_tot(self) -> float:
        """Everything that translates with the cart, pole included."""
        return self.m_c + self.m_rot + self.m_p

    @property
    def J(self) -> float:
        """Pole inertia about the pivot."""
        return (self.I_factor + 1.0) * self.m_p * self.l ** 2

    @property
    def ml(self) -> float:
        return self.m_p * self.l

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Hardware:
    board_type: str = "q2_usb"
    board_id: str = "0"
    enc_channels: tuple = (0, 1)        # cart, pole
    ao_channel: int = 0
    do_channels: tuple = (0, 1, 2, 3)   # VoltPAQ enable, as in swingup_mouse.slx
    enable_amp: bool = True
    cart_m_per_count: float = -2.2749e-05   # sign from the Gain -1 block
    pole_rad_per_count: float = 2 * math.pi / 4096
    volts_sign: float = 1.0             # flip if +V moves the cart toward -x
    v_limit: float = 10.0               # amplifier / DAC limit
    x_watchdog: float = 0.2             # the lab's stop limit
    hil_watchdog_s: float = 0.1         # board zeroes outputs if the loop stalls


@dataclass
class Config:
    plant: Plant = field(default_factory=Plant)
    hw: Hardware = field(default_factory=Hardware)


HW_FILE = DATA / "hardware.json"


def load_hardware() -> Hardware:
    """Hardware map, with overrides the probe wrote (e.g. volts_sign)."""
    h = Hardware()
    try:
        over = json.loads(HW_FILE.read_text())
        h = replace(h, **{k: (tuple(v) if isinstance(getattr(h, k, None), tuple) else v)
                          for k, v in over.items() if hasattr(h, k)})
    except (OSError, ValueError, TypeError):
        pass
    return h


def save_hardware_override(**kv):
    try:
        cur = json.loads(HW_FILE.read_text())
    except (OSError, ValueError):
        cur = {}
    cur.update(kv)
    HW_FILE.parent.mkdir(parents=True, exist_ok=True)
    HW_FILE.write_text(json.dumps(cur, indent=1))


def nominal() -> Plant:
    return Plant()


def load_plant(use_fitted: bool = True) -> Plant:
    """Nominal plant, overridden by the latest sysid fit when one exists."""
    p = Plant()
    if use_fitted and FITTED.exists():
        try:
            fit = json.loads(FITTED.read_text())["current"]["params"]
            p = replace(p, **{k: v for k, v in fit.items() if hasattr(p, k)})
        except (KeyError, ValueError, TypeError):
            pass
    return p


def describe(p: Plant) -> str:
    return (f"a={p.a:.3f} N/V  b={p.b:.3f} N s/m  B_f={p.B_f:.3f}  B_p={p.B_p:.4f}  "
            f"m_rot={p.m_rot:.3f} kg  M_tot={p.M_tot:.3f} kg  J={p.J:.3e}  k_amp={p.k_amp:.3f}  "
            f"F_coul={p.F_coul:.3f}  delay={p.delay_steps}")
