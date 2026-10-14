"""PPO (clipped surrogate, GAE, separate value net), numpy only.

`train` runs from scratch or warm-starts from an existing policy; `ppo_update`
is shared with the real-data update in improve.py (hardware episodes logged
with their action log-probabilities).
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .env import VecEnv
from .policy import Adam, Policy


@dataclass
class PPOConfig:
    steps_per_env: int = 200        # rollout length per env (x n envs per update)
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    epochs: int = 8
    minibatch: int = 1024
    lr: float = 3e-4
    vf_lr: float = 1e-3
    ent_coef: float = 0.0
    target_kl: float = 0.03
    min_log_std: float = -2.5


def gae(rew, val, last_val, term, trunc, gamma, lam):
    """rew/val/term/trunc: (T, N). Truncated steps bootstrap from the value
    of the observation reached (stored in last_val for the final step and in
    val[t+1] otherwise: envs reset in place, so for truncation we approximate
    the bootstrap with the current value)."""
    T, N = rew.shape
    adv = np.zeros((T, N))
    g = np.zeros(N)
    for t in reversed(range(T)):
        nv = last_val if t == T - 1 else val[t + 1]
        # after a reset the next value belongs to a new episode: cut on term,
        # bootstrap with own value on trunc
        nv = np.where(term[t], 0.0, np.where(trunc[t], val[t], nv))
        delta = rew[t] + gamma * nv - val[t]
        g = delta + gamma * lam * np.where(term[t] | trunc[t], 0.0, g)
        adv[t] = g
    return adv, adv + val


def ppo_update(pol: Policy, opt_pi: Adam, opt_vf: Adam, obs, act, logp_old, adv, ret,
               cfg: PPOConfig, rng) -> dict:
    n = len(obs)
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    stats = {"kl": 0.0, "clipfrac": 0.0, "vloss": 0.0, "epochs": 0}
    for ep in range(cfg.epochs):
        idx = rng.permutation(n)
        kls = []
        for s0 in range(0, n, cfg.minibatch):
            b = idx[s0:s0 + cfg.minibatch]
            o, a, lo, A, R = obs[b], act[b], logp_old[b], adv[b], ret[b]
            B = len(b)
            # --- policy
            mu, hs = pol.pi.forward(o)
            mu = mu[:, 0]
            ls = pol.log_std[0]
            std = np.exp(ls)
            lp = -0.5 * ((a - mu) / std) ** 2 - ls - 0.5 * np.log(2 * np.pi)
            ratio = np.exp(lp - lo)
            unclipped = ratio * A
            clipped = np.clip(ratio, 1 - cfg.clip, 1 + cfg.clip) * A
            use = unclipped <= clipped                       # min() picks unclipped
            dL_dlp = -(A * ratio * use) / B                  # d(-mean(min))/d logp
            dlp_dmu = (a - mu) / std ** 2
            dlp_dls = ((a - mu) / std) ** 2 - 1.0
            g_mu = (dL_dlp * dlp_dmu)[:, None]
            grads = pol.pi.backward(hs, g_mu)
            g_ls = np.array([float((dL_dlp * dlp_dls).sum()) - cfg.ent_coef])
            opt_pi.step(grads + [g_ls])
            if pol.log_std[0] < cfg.min_log_std:
                pol.log_std[0] = cfg.min_log_std
            # --- value
            v, hv = pol.vf.forward(o)
            dv = ((v[:, 0] - R) / B)[:, None]
            opt_vf.step(pol.vf.backward(hv, dv), max_norm=5.0)
            kls.append(float(np.mean(lo - lp)))
            stats["clipfrac"] = float(np.mean(~use))
            stats["vloss"] = float(np.mean((v[:, 0] - R) ** 2))
        stats["kl"] = float(np.mean(kls))
        stats["epochs"] = ep + 1
        if stats["kl"] > 1.5 * cfg.target_kl:
            break
    return stats


def collect(env: VecEnv, pol: Policy, T: int, rng):
    N = env.n
    obs = np.zeros((T, N, 6)); act = np.zeros((T, N)); logp = np.zeros((T, N))
    rew = np.zeros((T, N)); val = np.zeros((T, N))
    term = np.zeros((T, N), bool); trunc = np.zeros((T, N), bool)
    ep_ret = getattr(env, "_ep_ret", np.zeros(N))
    finished = []
    o = env.obs()
    for t in range(T):
        a, lp = pol.act(o, rng)
        obs[t], act[t], logp[t], val[t] = o, a, lp, pol.value(o)
        o2, r, te, tr, info = env.step(a)
        rew[t], term[t], trunc[t] = r, te, tr
        ep_ret += r
        done = te | tr
        if done.any():
            d = np.flatnonzero(done)
            for i in d:
                finished.append((ep_ret[i], bool(te[i]), bool(info["upright"][i])))
            ep_ret[d] = 0.0
            env.reset(d)
            o2 = env.obs()
        o = o2
    env._ep_ret = ep_ret
    last_val = pol.value(o)
    return obs, act, logp, rew, val, term, trunc, last_val, finished


def train(env: VecEnv, pol: Policy, iters: int, cfg: PPOConfig | None = None, seed: int = 0,
          progress=None, should_stop=lambda: False, checkpoint=None, every: int = 100) -> list[dict]:
    cfg = cfg or PPOConfig()
    rng = np.random.default_rng(seed)
    opt_pi = Adam(pol.pi.params() + [pol.log_std], lr=cfg.lr)
    opt_vf = Adam(pol.vf.params(), lr=cfg.vf_lr)
    hist = []
    t0 = time.time()
    steps = 0
    window: list = []
    for it in range(iters):
        if should_stop():
            break
        obs, act, logp, rew, val, term, trunc, last_val, fin = collect(env, pol, cfg.steps_per_env, rng)
        adv, ret = gae(rew, val, last_val, term, trunc, cfg.gamma, cfg.lam)
        st = ppo_update(pol, opt_pi, opt_vf, obs.reshape(-1, 6), act.ravel(), logp.ravel(),
                        adv.ravel(), ret.ravel(), cfg, rng)
        steps += obs.shape[0] * obs.shape[1]
        window = (window + fin)[-200:]
        rets = [f[0] for f in window]
        row = {"iter": it, "steps": steps, "sec": round(time.time() - t0, 1),
               "ep_return": float(np.mean(rets)) if rets else None,
               "fail_rate": float(np.mean([f[1] for f in window])) if window else None,
               "upright_end": float(np.mean([f[2] for f in window])) if window else None,
               "rew_per_step": float(rew.mean()), "log_std": float(pol.log_std[0]), **st}
        hist.append(row)
        if progress:
            progress(row)
        if checkpoint and (it + 1) % every == 0:
            checkpoint(pol, it + 1)
    return hist
