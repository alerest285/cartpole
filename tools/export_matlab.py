"""Export one model/controller contract plus Python reference vectors for MATLAB.

Run after sysid or policy promotion: python tools/export_matlab.py
No MATLAB, scipy, or hardware connection is needed.
"""
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from cartpole.controllers import make
from cartpole.controllers.base import Ref
from cartpole.controllers.classic import design_gain
from cartpole.controllers.learned import policy_path, registry
from cartpole.dynamics import PlantArrays, deriv, linearize, rk4
from cartpole.estimator import Estimator
from cartpole.params import H, load_hardware, load_plant
from cartpole.rl.policy import Policy
from cartpole.safety import Safety


def export(destination=None):
    out = Path(destination or ROOT / "matlab" / "generated")
    out.mkdir(parents=True, exist_ok=True)
    p, hw = load_plant(), load_hardware()
    A, B = linearize(p)
    gains = {"lqr": design_gain(p, {}), "place": design_gain(p, {"method": "place"}),
             "continuous": design_gain(p, {"time": "continuous"}),
             "energy": make("energy", p).Kc, "sine": make("sine", p).Kc}
    reg = registry()
    pol = Policy.load(policy_path(reg["current"]))
    cfg = {"schema": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
           "h": H, "plant": p.to_dict(), "A": A.tolist(), "B": B.tolist(),
           "hardware": hw.__dict__, "gains": {k: v.ravel().tolist() for k, v in gains.items()},
           "policy": {"version": reg["current"], "v_max": pol.meta.get("v_max", 8.0),
                      **{f"W{i+1}": w.tolist() for i, w in enumerate(pol.pi.W)},
                      **{f"b{i+1}": b.tolist() for i, b in enumerate(pol.pi.b)}}}
    (out / "contract.json").write_text(json.dumps(cfg, indent=2) + "\n")
    rng = np.random.default_rng(42)
    states = rng.uniform([-0.18, -np.pi, -1.5, -12], [0.18, np.pi, 1.5, 12], (100, 4))
    volts = rng.uniform(-10, 10, len(states))
    P, safety = PlantArrays(p), Safety(p)
    rows = [[*s, v, *deriv(s, v, P), *rk4(s, v, P), safety(s, v)[0]] for s, v in zip(states, volts)]
    np.savetxt(out / "model_vectors.csv", rows, delimiter=",", header="x,alpha,xd,ad,v,dx,da,dxd,dad,nx,na,nxd,nad,safe_v", comments="")
    est = Estimator()
    rows = []
    for s in states:
        x = round(s[0]/hw.cart_m_per_count)*hw.cart_m_per_count
        a = round((s[1]+np.pi)/hw.pole_rad_per_count)*hw.pole_rad_per_count-np.pi
        rows.append([*s, *est(x,a)])
    np.savetxt(out / "estimator_vectors.csv", rows, delimiter=",", header="x,alpha,xd,ad,xhat,ahat,xdhat,adhat", comments="")
    # Stateful replay checks the exact controller equations and PPO hold period.
    for mode in ("lqr", "place", "pid", "energy", "sine", "ppo"):
        c = make("statefb" if mode in ("lqr", "place") else mode, p)
        if mode == "place":
            c.set_params({"method": "place"})
        c.reset(None)
        replay = np.vstack([np.zeros((5, 4)), states, np.zeros((20, 4))])
        rows = [[k*H, *s, c.step(s, k*H, Ref())] for k, s in enumerate(replay)]
        np.savetxt(out / f"controller_{mode}.csv", rows, delimiter=",", header="t,x,alpha,xd,ad,v", comments="")
    print(f"MATLAB contract: {out} (policy {reg['current']}, period {H:g} s)")
    return cfg


if __name__ == "__main__":
    export()
