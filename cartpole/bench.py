"""Every controller, every mismatched plant, one table (python -m cartpole bench).

Balancers start tilted 3 deg and get tapped (pole speed +1 rad/s) at t = 3 s;
swing-ups start hanging. A case passes if the pole is within 5 deg of upright
for the last 2 s and nothing faulted.
"""
from __future__ import annotations

import json

from .controllers import make
from .controllers.learned import registry
from .params import DATA, load_plant
from .rl.train import eval_plants
from .simulate import closed_loop, summary

CASES = [
    ("pid", {}, "tilted"),
    ("statefb", {"method": "place"}, "tilted"),
    ("statefb", {"method": "lqr"}, "tilted"),
    ("energy", {}, "hanging"),
    ("sine", {}, "hanging"),
    ("ppo", {}, "hanging"),
    ("ppo", {"catch": True}, "hanging"),
]


def bench(T: float = 12.0, write: bool = True) -> list[dict]:
    centre = load_plant()
    plants = eval_plants(centre)
    rows = []
    for key, params, start in CASES:
        if key == "ppo" and not registry().get("current"):
            continue
        name = key + ("" if not params else " " + ",".join(f"{k}={v}" for k, v in params.items()))
        res = []
        for pname, pt in plants.items():
            c = make(key, centre)
            c.set_params(params)
            s0 = [0.0, 0.05, 0.0, 0.0] if start == "tilted" else None
            pokes = [(3.0, 1.0)] if start == "tilted" else []
            log, fault = closed_loop(c, pt, T, s0=s0, pokes=pokes)
            sm = summary(log, fault)
            res.append({"plant": pname, "ok": sm["balanced_at_end"], "t_up": sm["t_upright"],
                        "x_max_cm": round(sm["max_abs_x_cm"], 1)})
        ok = sum(r["ok"] for r in res)
        tups = [r["t_up"] for r in res if r["ok"] and r["t_up"] is not None]
        row = {"controller": name, "start": start, "passed": ok, "of": len(res),
               "t_up_mean": round(sum(tups) / len(tups), 2) if tups and start == "hanging" else None,
               "x_max_cm": max(r["x_max_cm"] for r in res), "cases": res}
        rows.append(row)
        print(f"{name:28s} {start:8s} {ok}/{len(res)}  t_up {row['t_up_mean']}  x_max {row['x_max_cm']} cm  "
              + " ".join(("ok" if r["ok"] else "--") for r in res), flush=True)
    if write:
        (DATA / "bench.json").write_text(json.dumps(rows, indent=1, default=float))
    return rows
