"""The deployment loop: every session on the rig makes the next policy better.

    hardware runs (any mode)            -> sysid refits the plant model
    amount of real data                 -> how wide the randomisation stays
    hardware PPO episodes (explore on)  -> one PPO update on real transitions
    refit simulator                     -> PPO fine-tune, warm-started
    evaluation on the refit model       -> promote only if not worse
    hardware runs per policy version    -> the real-world score, per version

Run from the UI (Learn tab -> Improve) or `python -m cartpole improve`.
"""
from __future__ import annotations

import json
import math
from dataclasses import replace

import numpy as np

from .. import sysid
from ..controllers.learned import REGISTRY, policy_path, registry
from ..params import DATA, H, RUNS, load_plant
from .env import DECIM, reward
from .policy import Adam, Policy
from .ppo import PPOConfig, gae, ppo_update
from .train import evaluate, run_training, score

CONSUMED = DATA / "consumed_episodes.json"


def hardware_episodes(include_sim: bool, version: str | None):
    """Logged exploration episodes, newest first, from the current lineage."""
    try:
        used = set(json.loads(CONSUMED.read_text()))
    except (OSError, ValueError):
        used = set()
    out = []
    for f in sorted(RUNS.glob("*.ppo.npz"), reverse=True):
        rid = f.name[:-len(".ppo.npz")]
        if rid in used:
            continue
        try:
            meta = json.loads((RUNS / f"{rid}.json").read_text())
        except (OSError, ValueError):
            continue
        if (meta.get("sim") or "fake" in str(meta.get("backend", "")).lower()) and not include_sim:
            continue
        if version is not None and meta.get("policy") != version:
            continue  # PPO importance ratios require the current behavior policy.
        z = np.load(f)
        if "tick" in z and np.any(np.diff(z["tick"]) != DECIM):
            continue  # Hybrid catcher gaps are not contiguous PPO transitions.
        out.append((rid, meta, {k: z[k] for k in z.files}))
    return out


def real_batch(pol: Policy, episodes, gamma=0.99, lam=0.95):
    """(obs, act, logp, adv, ret) from hardware episodes; rewards from the
    logged states with the same reward function as the simulator."""
    O, A, L, ADV, RET = [], [], [], [], []
    for rid, meta, ep in episodes:
        obs, act, logp, st = ep["obs"], ep["act"], ep["logp"], ep["state"]
        if len(obs) < 3:
            continue
        u = np.clip(act, -1, 1)
        prev = np.concatenate([[0.0], u[:-1]])
        r, failed, _ = reward(np.vstack([st[1:], st[-1:]]), u, prev)
        end = str(meta.get("end", ""))
        term = np.zeros(len(obs), bool)
        if "cart" in end or "hard limit" in end or "barrier" in end:
            term[-1] = True
            r[-1] -= 25.0
        val = pol.value(obs)
        adv, ret = gae(r[:, None], val[:, None], np.array([val[-1]]), term[:, None],
                       np.zeros((len(obs), 1), bool), gamma, lam)
        O.append(obs); A.append(act); L.append(logp); ADV.append(adv[:, 0]); RET.append(ret[:, 0])
    if not O:
        return None
    return [np.concatenate(z) for z in (O, A, L, ADV, RET)]


def hardware_scores(include_sim: bool = False) -> dict:
    """Per policy version: how its PPO runs on the rig went."""
    out: dict[str, dict] = {}
    for j in sorted(RUNS.glob("*.json")):
        try:
            m = json.loads(j.read_text())
        except (OSError, ValueError):
            continue
        simulated = m.get("sim") or "fake" in str(m.get("backend", "")).lower()
        if m.get("mode") != "ppo" or (simulated and not include_sim):
            continue
        f = RUNS / f"{m['id']}.npy"
        if not f.exists():
            continue
        arr = np.load(f)
        a = np.abs(np.pi - np.mod(np.pi - arr[:, 2], 2 * np.pi))
        up = np.flatnonzero(a < np.deg2rad(10))
        last = arr[-int(1.0 / H):, 2] if len(arr) > int(1.0 / H) else arr[:, 2]
        ok = bool(np.all(np.abs(np.pi - np.mod(np.pi - last, 2 * np.pi)) < np.deg2rad(8)))
        v = m.get("policy") or "?"
        d = out.setdefault(v, {"runs": 0, "balanced": 0, "t_up": [], "seconds": 0.0})
        d["runs"] += 1
        d["balanced"] += ok
        d["seconds"] += m.get("duration_s", 0)
        if len(up):
            d["t_up"].append(float(arr[up[0], 0] - arr[0, 0]))
    for d in out.values():
        d["t_up_mean"] = float(np.mean(d["t_up"])) if d["t_up"] else None
        d["success"] = d["balanced"] / d["runs"] if d["runs"] else None
        del d["t_up"]
    return out


def improve(iters: int = 300, include_sim: bool = False, real_update: bool = True,
            progress=lambda d: None, log=print) -> dict:
    report: dict = {}
    # 1. system identification
    progress({"phase": "sysid"})
    fit = sysid.fit(include_sim=include_sim, log=log)
    accepted_fit = None
    if fit:
        saved = sysid.save(fit, log=log)
        accepted_fit = fit if saved else None
        report["sysid"] = {"saved": saved, "r2_cart": fit["r2_cart"], "r2_pole": fit["r2_pole"],
                           "params": fit["params"], "problems": fit["problems"]}
    centre = load_plant()
    scale = sysid.uncertainty_scale(accepted_fit)
    report["rand_scale"] = scale
    log(f"randomisation scale {scale:.2f} (1 = prior only, shrinks with real data)")

    reg = registry()
    cur = reg.get("current")
    # 2. re-score the current policy on the refit model, so the comparison is fair
    if cur and policy_path(cur).exists():
        progress({"phase": "rescore", "version": cur})
        ev = evaluate(Policy.load(policy_path(cur)), centre)
        reg["versions"][cur]["eval"] = ev
        reg["versions"][cur]["score"] = score(ev)
        REGISTRY.write_text(json.dumps(reg, indent=1, default=float))
        log(f"current {cur} on the refit model: success {ev['success']:.0%}")

    # 3. learn from hardware episodes
    used_ids = []

    def real_update_fn(pol: Policy):
        if not real_update:
            return
        eps = hardware_episodes(include_sim, cur)
        if not eps:
            log("no new exploration episodes from the rig; skipping the real-data update")
            return
        batch = real_batch(pol, eps)
        if batch is None:
            return
        cfg = PPOConfig(lr=5e-5, epochs=4, minibatch=256)
        st = ppo_update(pol, Adam(pol.pi.params() + [pol.log_std], lr=cfg.lr), Adam(pol.vf.params(), lr=3e-4),
                        *batch, cfg, np.random.default_rng(0))
        used_ids.extend(rid for rid, _, _ in eps)
        report["real_update"] = {"episodes": len(eps), "transitions": int(len(batch[0])), **st}
        log(f"PPO update on {len(eps)} real episodes ({len(batch[0])} transitions): kl {st['kl']:.4f}")

    # 4. fine-tune in the refit simulator (warm start), real update first
    pre = None
    if cur and policy_path(cur).exists():
        def pre(pol):  # noqa: E306
            real_update_fn(pol)
    progress({"phase": "train", "iter": 0, "iters": iters})
    v, promoted = run_training(iters=iters, warm="current" if cur else None, centre=centre, rand_scale=scale,
                               progress=progress, note="improve: sysid + real episodes + sim fine-tune",
                               lr=1e-4 if cur else 3e-4, pre_update=pre, log=log)
    report["version"] = v
    report["promoted"] = promoted
    if used_ids:
        try:
            done = json.loads(CONSUMED.read_text())
        except (OSError, ValueError):
            done = []
        CONSUMED.write_text(json.dumps(done + used_ids))
    # 5. hardware scoreboard
    hs = hardware_scores(include_sim)
    reg = registry()
    for ver, d in hs.items():
        if ver in reg.get("versions", {}):
            reg["versions"][ver]["hardware"] = d
    REGISTRY.write_text(json.dumps(reg, indent=1, default=float))
    report["hardware"] = hs
    progress({"phase": "done", "report": report})
    return report
