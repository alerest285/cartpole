"""PPO policy as a controller.

Acts every 10 ticks (50 Hz), holding its voltage in between, exactly as in
training. 'catch' hands over to the LQR near upright (the hybrid);
'explore' samples from the Gaussian instead of using its mean and records
(obs, action, log-prob) so a hardware session can be learned from.
"""
from __future__ import annotations

import json
import math

import numpy as np

from ..params import DATA
from ..rl.env import DECIM
from ..rl.policy import Policy, obs_from_state
from .base import Controller, Param, deg
from .classic import DEFAULT_CATCH, design_gain

POLICIES = DATA / "policies"
REGISTRY = POLICIES / "registry.json"


def registry() -> dict:
    try:
        return json.loads(REGISTRY.read_text())
    except (OSError, ValueError):
        return {"current": None, "versions": {}}


def policy_path(version: str):
    return POLICIES / f"{version}.npz"


class PPOController(Controller):
    key = "ppo"
    label = "PPO policy (learned)"
    blurb = ("A neural network (2 x 64) trained with PPO in the randomised simulator, "
             "and fine-tuned from every hardware session (Learn tab). It swings up and "
             "balances by itself. 'LQR catch' hands the last few degrees to the LQR; "
             "'explore' adds the policy's own noise and records the episode so the next "
             "Improve run can learn from the real rig.")
    PARAMS = [
        Param("version", "Policy version", "current", kind="choice", choices=["current"]),
        Param("catch", "LQR catch near upright", False, kind="bool"),
        Param("catch_deg", "Catch inside [deg]", 15.0, 3, 45, 1),
        Param("explore", "Explore (record for learning)", False, kind="bool"),
        Param("seed", "Exploration seed", 0, 0, 10 ** 6, 1, kind="int"),
    ]

    @classmethod
    def from_policy(cls, pol: Policy, plant, **params) -> "PPOController":
        """Wrap an in-memory policy (evaluation during training)."""
        c = cls(plant)
        c.pol, c.version = pol, pol.meta.get("version", "candidate")
        c.v_max = float(pol.meta.get("v_max", 8.0))
        c.p.update(params)
        c.p["version"] = "__fixed__"
        c.reset(None)
        return c

    @classmethod
    def schema(cls) -> dict:
        d = super().schema()
        reg = registry()
        for q in d["params"]:
            if q["name"] == "version":
                q["choices"] = ["current"] + sorted(reg.get("versions", {}))
        return d

    def on_params(self):
        self.Kc = design_gain(self.plant, DEFAULT_CATCH)
        want = self.p.get("version", "current")
        if want == "__fixed__":
            return
        reg = registry()
        v = reg.get("current") if want == "current" else want
        if v != getattr(self, "version", None) or not hasattr(self, "pol"):
            self.version = v
            self.pol = Policy.load(policy_path(v)) if v and policy_path(v).exists() else None
        self.v_max = float(self.pol.meta.get("v_max", 8.0)) if self.pol else 8.0
        self.rng = np.random.default_rng(int(self.p.get("seed", 0)))
        self.info = {"version": self.version or "none trained yet"}
        self.reset(None)

    def reset(self, s):
        self.k = 0
        self.u = 0.0
        self.prev_u = 0.0
        self.caught = False
        self.buf = {"obs": [], "act": [], "logp": [], "tick": [], "state": []}

    def step(self, s, t, ref):
        if self.pol is None:
            self.info["phase"] = "no policy: train one first"
            return 0.0
        a = abs(s[1])
        if self.p["catch"]:
            if self.caught and a > deg(30):
                self.caught = False
            elif not self.caught and a < deg(self.p["catch_deg"]):
                self.caught = True
            if self.caught:
                self.info["phase"] = "balancing"
                self.k += 1
                return float(-(self.Kc @ (s - np.array([ref.x_ref, 0, 0, 0])))[0])
        if self.k % DECIM == 0:
            s_rel = s.copy()
            s_rel[0] -= ref.x_ref
            o = obs_from_state(s_rel, self.prev_u)
            if self.p["explore"]:
                act, lp = self.pol.act(o, self.rng)
                self.buf["obs"].append(o); self.buf["act"].append(float(act[0]))
                self.buf["logp"].append(float(lp[0])); self.buf["tick"].append(self.k)
                self.buf["state"].append(s.copy())
                u = float(act[0])
            else:
                u = float(self.pol.mean(o)[0])
            self.u = float(np.clip(u, -1, 1))
            self.prev_u = self.u
        self.k += 1
        self.info["phase"] = "balancing" if a < deg(12) else ("exploring" if self.p["explore"] else "swinging")
        return self.u * self.v_max

    def flush_episode(self) -> dict | None:
        """Called by the runtime when a run ends: the explore buffer, if any."""
        if not self.buf["obs"]:
            return None
        out = {k: np.asarray(v) for k, v in self.buf.items()}
        out["version"] = np.array(self.version or "")
        self.buf = {"obs": [], "act": [], "logp": [], "tick": [], "state": []}
        return out
