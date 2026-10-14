"""Gaussian MLP policy and value function in numpy, with manual backprop.

Small on purpose (2 x 64 tanh): it runs at 50 Hz on any lab PC with numpy only,
and the whole thing (forward, backward, Adam) is ~150 lines you can read.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

OBS_DIM = 6
LOG2PI = np.log(2 * np.pi)


def obs_from_state(s, prev_u):
    """s: (..., 4) [x, alpha, x_dot, alpha_dot]; prev_u in [-1, 1].
    Fixed scaling, identical in training and on the rig."""
    s = np.asarray(s, dtype=float)
    return np.stack([s[..., 0] / 0.2, np.sin(s[..., 1]), np.cos(s[..., 1]),
                     s[..., 2] / 1.0, s[..., 3] / 10.0, np.asarray(prev_u, dtype=float)], axis=-1)


class MLP:
    def __init__(self, sizes, rng, out_scale=1.0):
        self.W, self.b = [], []
        for i, (a, b) in enumerate(zip(sizes[:-1], sizes[1:])):
            scale = np.sqrt(2.0 / a) if i < len(sizes) - 2 else out_scale / np.sqrt(a)
            self.W.append(rng.normal(0, scale, (a, b)))
            self.b.append(np.zeros(b))

    def params(self):
        return self.W + self.b

    def forward(self, x):
        hs = [x]
        h = x
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = h @ W + b
            h = np.tanh(z) if i < len(self.W) - 1 else z
            hs.append(h)
        return h, hs

    def backward(self, hs, dout):
        """Gradients of sum(dout * output) w.r.t. W, b (same order as params())."""
        gW, gb = [None] * len(self.W), [None] * len(self.b)
        d = dout
        for i in reversed(range(len(self.W))):
            gW[i] = hs[i].T @ d
            gb[i] = d.sum(0)
            if i > 0:
                d = (d @ self.W[i].T) * (1 - hs[i] ** 2)
        return gW + gb


class Policy:
    def __init__(self, hidden=64, seed=0, log_std=-0.5):
        rng = np.random.default_rng(seed)
        self.pi = MLP([OBS_DIM, hidden, hidden, 1], rng, out_scale=0.01)
        self.vf = MLP([OBS_DIM, hidden, hidden, 1], rng, out_scale=1.0)
        self.log_std = np.array([log_std])
        self.meta: dict = {}

    # ---------------------------------------------------------------- act
    def mean(self, obs):
        mu, _ = self.pi.forward(np.atleast_2d(obs))
        return mu[:, 0]

    def act(self, obs, rng=None, deterministic=False):
        obs = np.atleast_2d(obs)
        mu, _ = self.pi.forward(obs)
        mu = mu[:, 0]
        if deterministic:
            return mu, np.zeros_like(mu)
        std = np.exp(self.log_std[0])
        a = mu + std * (rng or np.random.default_rng()).standard_normal(mu.shape)
        return a, self.logp(a, mu)

    def logp(self, a, mu):
        ls = self.log_std[0]
        return -0.5 * ((a - mu) / np.exp(ls)) ** 2 - ls - 0.5 * LOG2PI

    def value(self, obs):
        v, _ = self.vf.forward(np.atleast_2d(obs))
        return v[:, 0]

    # ----------------------------------------------------------- persist
    def save(self, path: Path, meta: dict | None = None):
        d = {f"pi_{i}": p for i, p in enumerate(self.pi.params())}
        d.update({f"vf_{i}": p for i, p in enumerate(self.vf.params())})
        d["log_std"] = self.log_std
        m = dict(self.meta)
        m.update(meta or {})
        d["meta"] = np.array(json.dumps(m, default=float))
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **d)

    @classmethod
    def load(cls, path: Path) -> "Policy":
        z = np.load(path, allow_pickle=False)
        pi_n = len([k for k in z.files if k.startswith("pi_")]) // 2
        hidden = z["pi_0"].shape[1]
        p = cls(hidden=hidden)
        for i in range(pi_n):
            p.pi.W[i] = z[f"pi_{i}"]; p.pi.b[i] = z[f"pi_{i + pi_n}"]
            p.vf.W[i] = z[f"vf_{i}"]; p.vf.b[i] = z[f"vf_{i + pi_n}"]
        p.log_std = z["log_std"]
        p.meta = json.loads(str(z["meta"]))
        return p

    def copy(self) -> "Policy":
        q = Policy(hidden=self.pi.W[0].shape[1])
        q.pi.W = [w.copy() for w in self.pi.W]; q.pi.b = [b.copy() for b in self.pi.b]
        q.vf.W = [w.copy() for w in self.vf.W]; q.vf.b = [b.copy() for b in self.vf.b]
        q.log_std = self.log_std.copy(); q.meta = dict(self.meta)
        return q


class Adam:
    def __init__(self, params, lr=3e-4, b1=0.9, b2=0.999, eps=1e-8):
        self.params = params
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, grads, max_norm=0.5):
        norm = np.sqrt(sum(float((g ** 2).sum()) for g in grads))
        if norm > max_norm:
            grads = [g * (max_norm / (norm + 1e-12)) for g in grads]
        self.t += 1
        for p, g, m, v in zip(self.params, grads, self.m, self.v):
            m *= self.b1; m += (1 - self.b1) * g
            v *= self.b2; v += (1 - self.b2) * g * g
            mh = m / (1 - self.b1 ** self.t)
            vh = v / (1 - self.b2 ** self.t)
            p -= self.lr * mh / (np.sqrt(vh) + self.eps)
        return norm
