"""Evidence for 'it gets better with more deployments', in simulation.

A simulated rig whose physics the model does not know. Each round:
deploy the current policy (no LQR catch) -> score it on the rig -> record an
Identify run and exploration episodes -> improve -> next round.
Runs in a scratch copy of data/ (CARTPOLE_DATA / CARTPOLE_RUNS), so the real
registry is not touched. Usage: python tools/deployment_loop_demo.py [rounds]
"""
import os
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
scratch = Path(tempfile.mkdtemp(prefix="cartpole-loop-"))
shutil.copytree(ROOT / "data" / "policies", scratch / "data" / "policies")
os.environ["CARTPOLE_DATA"] = str(scratch / "data")
os.environ["CARTPOLE_RUNS"] = str(scratch / "runs")
sys.path.insert(0, str(ROOT))

import json  # noqa: E402
import time  # noqa: E402
from dataclasses import replace  # noqa: E402

import numpy as np  # noqa: E402

from cartpole.backends.sim import SimBackend  # noqa: E402
from cartpole.controllers.learned import registry  # noqa: E402
from cartpole.params import Plant, load_plant  # noqa: E402
from cartpole.rl.improve import improve  # noqa: E402
from cartpole.runtime import Runtime  # noqa: E402

RIG = replace(Plant(), B_f=5.5, k_amp=0.8, B_p=0.005, F_coul=0.4, delay_steps=1, m_c=0.63)


def wait(rt, secs):
    t0 = rt.t
    while rt.t < t0 + secs and rt.armed:
        time.sleep(0.002)


def score_on_rig(trials=5, T=10.0):
    """The current policy (no LQR catch) on the rig from hanging, exact per-tick
    closed loop (same estimator, safety, quantisation as the runtime)."""
    from cartpole.controllers.learned import PPOController, policy_path
    from cartpole.rl.policy import Policy
    from cartpole.simulate import closed_loop, summary
    pol = Policy.load(policy_path(registry()["current"]))
    ok, tups, xs = 0, [], []
    for d in np.linspace(-0.05, 0.05, trials):
        c = PPOController.from_policy(pol, load_plant())
        log, f = closed_loop(c, RIG, T, s0=[0.0, -np.pi + d, 0.0, 0.0])
        sm = summary(log, f)
        ok += sm["balanced_at_end"]; xs.append(sm["max_abs_x_cm"])
        if sm["balanced_at_end"] and sm["t_upright"] is not None:
            tups.append(sm["t_upright"])
    return {"balanced": f"{ok}/{trials}", "t_up": round(float(np.mean(tups)), 2) if tups else None,
            "x_max_cm": round(max(xs), 1)}


def deploy_and_record(explore_eps=8):
    # real time on purpose: episode lengths (and so the amount of real data) match a lab session
    rt = Runtime(SimBackend(RIG, realtime=True), plant=load_plant(), log=lambda m: None)
    rt.start()
    rt.set_mode("excite"); rt.arm(); wait(rt, 30); rt.disarm()
    rt.set_mode("ppo"); rt.set_params("ppo", {"explore": True, "catch": False, "version": "current"})
    for i in range(explore_eps):
        with rt.lock:
            rt.backend.sim.reset(); rt.est.started = False
        rt.set_params("ppo", {"seed": i}); rt.arm(); wait(rt, 8); rt.disarm()
        time.sleep(0.05)
    time.sleep(0.5); rt.stop(); time.sleep(0.3)


if __name__ == "__main__":
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"rig (unknown to the model): {RIG}\nscratch: {scratch}\n", flush=True)
    rows = []
    for r in range(rounds + 1):
        reg = registry()
        s = score_on_rig()
        s["version"] = reg["current"]
        s["model"] = {k: round(v, 3) for k, v in load_plant().to_dict().items() if k in ("B_f", "k_amp", "B_p", "F_coul", "m_c", "delay_steps")}
        rows.append(s)
        print(f"round {r}: {json.dumps(s)}", flush=True)
        if r == rounds:
            break
        deploy_and_record()
        rep = improve(iters=250, include_sim=True, log=lambda m: print("   ", m, flush=True))
    print("\nSUMMARY")
    for i, s in enumerate(rows):
        print(f"  round {i}  {s['version']}  rig balanced {s['balanced']}  t_up {s['t_up']} s  x_max {s['x_max_cm']} cm  model {s['model']}")
    (ROOT / "docs" / "evidence").mkdir(parents=True, exist_ok=True)
    (ROOT / "docs" / "evidence" / "deployment_loop.json").write_text(json.dumps({"rig": RIG.to_dict(), "rounds": rows}, indent=1))
