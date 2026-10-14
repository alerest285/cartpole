"""Train, evaluate and version PPO policies.

Every policy is evaluated through the real deployment path (PPOController ->
Kalman estimator -> safety -> quantised encoders, simulate.closed_loop) on a
fixed set of plants around the current model, next to the energy swing-up as a
baseline. A new version is promoted to 'current' only if it is at least as good.
"""
from __future__ import annotations

import json
import time
from dataclasses import replace
from datetime import datetime

import numpy as np

from ..controllers.learned import POLICIES, REGISTRY, PPOController, policy_path, registry
from ..params import Plant, load_plant
from ..simulate import closed_loop, summary
from .env import EnvConfig, VecEnv
from .policy import Policy
from .ppo import PPOConfig, train


def eval_plants(centre: Plant) -> dict[str, Plant]:
    c = centre
    return {
        "model": c,
        "friction -40%": replace(c, B_f=c.B_f * 0.6),
        "friction +60%": replace(c, B_f=c.B_f * 1.6),
        "gain -10%": replace(c, k_amp=c.k_amp * 0.9),
        "gain +10%": replace(c, k_amp=c.k_amp * 1.1),
        "delay 2 ms": replace(c, delay_steps=c.delay_steps + 1),
        "coulomb 0.3 N": replace(c, F_coul=c.F_coul + 0.3),
    }


def evaluate(pol: Policy, centre: Plant, T: float = 12.0, catch: bool = False, starts=("hanging", "tilted")) -> dict:
    res = {}
    for name, pt in eval_plants(centre).items():
        for st in starts:
            s0 = None if st == "hanging" else [0.0, 0.3, 0.0, 0.0]
            ctrl = PPOController.from_policy(pol, centre, catch=catch)
            log, fault = closed_loop(ctrl, pt, T, s0=s0)
            sm = summary(log, fault)
            res[f"{name} / {st}"] = {"ok": sm["balanced_at_end"], "t_up": sm["t_upright"],
                                     "x_max_cm": round(sm["max_abs_x_cm"], 1), "fault": sm["fault"]}
    oks = [r["ok"] for r in res.values()]
    tups = [r["t_up"] for r in res.values() if r["ok"] and r["t_up"] is not None]
    return {"success": float(np.mean(oks)), "n": len(oks),
            "t_up_mean": float(np.mean(tups)) if tups else None, "cases": res}


def score(ev: dict) -> float:
    """Higher is better: success first, then speed."""
    return ev["success"] * 100 - (ev["t_up_mean"] or 20.0)


def next_version(reg: dict) -> str:
    n = 1 + max([int(v[1:]) for v in reg.get("versions", {})] or [0])
    return f"v{n:03d}"


def save_version(pol: Policy, meta: dict, promote_if_better: bool = True, log=print) -> tuple[str, bool]:
    reg = registry()
    v = next_version(reg)
    pol.meta.update(meta, version=v)
    pol.save(policy_path(v))
    cur = reg.get("current")
    cur_score = score(reg["versions"][cur]["eval"]) if cur and "eval" in reg["versions"].get(cur, {}) else -1e9
    new_score = score(meta["eval"])
    promote = (not promote_if_better) or new_score >= cur_score - 1e-9
    entry = {k: meta[k] for k in meta if k not in ("curve",)}
    entry["curve"] = meta.get("curve", [])[-200:]
    entry["score"] = new_score
    reg.setdefault("versions", {})[v] = entry
    if promote:
        reg["current"] = v
    POLICIES.mkdir(parents=True, exist_ok=True)
    REGISTRY.write_text(json.dumps(reg, indent=1, default=float))
    log(f"saved {v}: success {meta['eval']['success']:.0%}, t_up {meta['eval']['t_up_mean']}, "
        f"score {new_score:.1f} vs current {cur_score:.1f} -> {'PROMOTED' if promote else 'kept ' + str(cur)}")
    return v, promote


def run_training(iters: int = 600, warm: str | None = None, centre: Plant | None = None,
                 rand_scale: float = 1.0, seed: int = 0, n_envs: int = 64, v_max: float = 8.0,
                 progress=None, should_stop=lambda: False, note: str = "", lr: float = 3e-4,
                 extra_update=None, pre_update=None, log=print) -> tuple[str, bool]:
    centre = centre or load_plant()
    if warm == "checkpoint":
        pol = Policy.load(POLICIES / "checkpoint.npz")
        parent = pol.meta.get("parent")
    elif warm:
        reg = registry()
        wv = reg.get("current") if warm == "current" else warm
        pol = Policy.load(policy_path(wv)) if wv else Policy(seed=seed)
        parent = wv
    else:
        pol, parent = Policy(seed=seed), None
    pol.meta["v_max"] = v_max
    if pre_update:
        pre_update(pol)
    env = VecEnv(centre, EnvConfig(n=n_envs, v_max=v_max, rand_scale=rand_scale), seed=seed)
    curve = []

    def prog(row):
        curve.append({k: row[k] for k in ("iter", "steps", "sec", "ep_return", "fail_rate", "upright_end", "rew_per_step")})
        if progress:
            progress({"phase": "train", **row, "iters": iters})

    t0 = time.time()
    ckpt = POLICIES / "checkpoint.npz"

    def save_ckpt(p, it):
        p.save(ckpt, {"checkpoint_iter": it, "parent": parent, "v_max": v_max})

    hist = train(env, pol, iters, PPOConfig(lr=lr), seed=seed, progress=prog, should_stop=should_stop,
                 checkpoint=save_ckpt)
    if extra_update:
        extra_update(pol)
    if progress:
        progress({"phase": "evaluate"})
    ev = evaluate(pol, centre)
    meta = {"created": datetime.now().isoformat(timespec="seconds"), "parent": parent, "iters": len(hist),
            "steps": hist[-1]["steps"] if hist else 0, "train_s": round(time.time() - t0, 1),
            "rand_scale": rand_scale, "plant": centre.to_dict(), "eval": ev, "curve": curve,
            "v_max": v_max, "note": note}
    return save_version(pol, meta, log=log)


def baseline(centre: Plant | None = None) -> dict:
    """The energy swing-up on the same plants, for comparison in the UI."""
    from ..controllers import make
    centre = centre or load_plant()
    res = {}
    for name, pt in eval_plants(centre).items():
        ctrl = make("energy", centre)
        log, fault = closed_loop(ctrl, pt, 12.0)
        sm = summary(log, fault)
        res[name] = {"ok": sm["balanced_at_end"], "t_up": sm["t_upright"]}
    return res
