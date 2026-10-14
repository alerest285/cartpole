"""Handout Step 2: B_f regression, Coulomb comparison and held-out validation.

Known masses/motor constants are held fixed here. This complements sysid's
joint mass/gain/damping fit. It never installs an estimate automatically.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .params import H, Plant
from .sysid import derivs, smooth


def load_csv(path):
    """SI columns t,x,alpha and v (logical volts) or force (net motor force)."""
    data = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8")
    names = data.dtype.names or ()
    if not {"t", "x", "alpha"}.issubset(names) or not ({"v", "force"} & set(names)):
        raise ValueError("CSV needs t [s], x [m], alpha [rad, upright=0], and v [V] or force [N]")
    return {k: np.atleast_1d(data[k]) for k in names}


def regression_data(data, plant=None, delay_steps=0):
    p = plant or Plant()
    t, x, a = (np.asarray(data[k], dtype=float).ravel() for k in ("t", "x", "alpha"))
    if len(t) < 1000 or len(x) != len(t) or len(a) != len(t):
        raise ValueError("Need at least 1,000 aligned samples (2 seconds at 500 Hz)")
    dt = np.diff(t); h = float(np.median(dt))
    if not np.isfinite([*t, *x, *a]).all() or h <= 0 or np.max(np.abs(dt - h)) > h * 0.02:
        raise ValueError("Timestamps must be finite, increasing and uniformly sampled (within 2%)")
    if h > 0.01:
        raise ValueError("At least 100 Hz is required for encoder-based acceleration fitting")
    sigma = max(1.0, 0.012 / h)
    _, xd, xdd, ok = derivs(x, h, sigma)
    af, ad, add, oka = derivs(np.unwrap(a), h, sigma)
    if "v" in data:
        v = np.asarray(data["v"], dtype=float).ravel()
        if len(v) != len(t) or not np.isfinite(v).all():
            raise ValueError("Voltage samples must be finite and aligned")
        # Sample k is read before command k; align the held command to acceleration.
        v = np.interp(np.arange(len(v)) - delay_steps - 0.5, np.arange(len(v)), v)
        force = p.a * p.k_amp * smooth(v, sigma) - p.b * xd
    else:
        force = np.asarray(data["force"], dtype=float).ravel()
        if len(force) != len(t) or not np.isfinite(force).all():
            raise ValueError("Force samples must be finite and aligned")
        force = smooth(force, sigma)  # force already includes motor back-EMF
    # M xdd - ml cos(a) add + ml sin(a) ad^2 = F_motor - B_f xd.
    y = force - p.M_tot * xdd + p.ml * (np.cos(af) * add - np.sin(af) * ad**2)
    ok &= oka & (np.abs(x) < 0.17) & (np.abs(xd) > 0.015)
    # Guard the chronological split so the two filtered subsets do not overlap.
    cut = int(0.65 * len(t)); guard = int(np.ceil(4 * sigma)) + 2
    train = ok & (np.arange(len(t)) < cut - guard)
    valid = ok & (np.arange(len(t)) > cut + guard)
    if train.sum() < 300 or valid.sum() < 150:
        raise ValueError("Insufficient moving samples: record motion in both directions for 20–30 s")
    for mask in (train, valid):
        if np.sum(xd[mask] > 0.015) < 40 or np.sum(xd[mask] < -0.015) < 40:
            raise ValueError("Both estimation and validation segments need motion in both directions")
    return t, xd, y, train, valid, h


def estimate(data, plant=None, delay_steps=0):
    t, xd, y, train, valid, h = regression_data(data, plant, delay_steps)
    models = {}
    for name, X in (("viscous", xd[:, None]),
                    ("viscous_coulomb", np.column_stack([xd, np.tanh(xd / 0.01)]))):
        A, b = X[train], y[train]
        scales = np.sqrt(np.mean(A*A, axis=0))
        condition = float(np.linalg.cond(A / scales))
        theta = np.linalg.lstsq(A, b, rcond=None)[0]
        residual = y[valid] - X[valid] @ theta
        rmse = float(np.sqrt(np.mean(residual**2)))
        r2 = float(1 - np.sum(residual**2) / max(1e-12, np.sum((y[valid] - y[valid].mean())**2)))
        # Block estimates expose time/direction dependence; these are not iid CIs.
        blocks = []
        for indices in np.array_split(np.flatnonzero(train), 6):
            if len(indices) >= 40 and np.linalg.matrix_rank(X[indices]) == X.shape[1]:
                blocks.append(np.linalg.lstsq(X[indices], y[indices], rcond=None)[0].tolist())
        problems = []
        if not np.isfinite(theta).all() or theta[0] < 0 or theta[0] > 30:
            problems.append("B_f outside 0–30 N s/m; check units, signs, mass and motor gain")
        if len(theta) > 1 and not 0 <= theta[1] <= 3:
            problems.append("Coulomb friction outside 0–3 N")
        if condition > 100:
            problems.append("Poor parameter separation; vary cart speed, not only its direction")
        if r2 < 0.6:
            problems.append("Held-out fit is weak; collect another excitation record")
        models[name] = {"B_f": float(theta[0]), "F_coul": float(theta[1]) if len(theta) > 1 else 0.,
                        "validation_rmse_N": rmse, "validation_r2": r2,
                        "scaled_condition": condition, "block_estimates": blocks,
                        "accepted": not problems, "problems": problems}
    simple, extended = models["viscous"], models["viscous_coulomb"]
    chosen = "viscous_coulomb" if extended["accepted"] and (
        not simple["accepted"] or extended["validation_rmse_N"] < 0.85 * simple["validation_rmse_N"]
    ) else "viscous"
    return {"schema": 1, "exercise_dimension": 1, "duration_s": float(t[-1] - t[0]),
            "sample_period_s": h, "n_train": int(train.sum()), "n_validation": int(valid.sum()),
            "delay_steps_assumed": delay_steps, "models": models, "selected": chosen,
            "accepted": models[chosen]["accepted"], "params": {k: models[chosen][k] for k in ("B_f", "F_coul")},
            "note": "Known masses and motor gain assumed. Confirm on a separate run; no parameters installed."}


def main(path, output=None, delay_steps=0):
    result = estimate(load_csv(path), delay_steps=delay_steps)
    destination = Path(output or str(Path(path).with_suffix(".friction.json")))
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    print(f"Report: {destination}; model not changed")
    return 0 if result["accepted"] else 2
