"""System identification from logged runs (the handout's Step 2, generalised).

Both equations of motion are linear in the unknowns once x, alpha and their
derivatives are known, so ordinary least squares does it:

  cart:  M_tot x'' - k_amp (a V) + c_v x' + F_coul tanh(x'/v_c) = ml (cos a a'' - sin a a'^2)
         unknowns M_tot, k_amp, c_v (= back-EMF + B_f), F_coul
  pole:  J a'' + B_p a' = ml (g sin a + cos a x'')
         unknowns J, B_p

Derivatives: one zero-phase Gaussian (12 ms) applied to the counts AND the
voltage, then exact differentiation (offline, so no lag, and no filter mismatch). The actuation delay is the one (0..4 ticks) with the best
cart fit. Standard errors from the residual variance; physically impossible
fits are rejected rather than written.
"""
from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np

from .dynamics import V_COUL
from .params import FITTED, H, RUNS, Hardware, Plant, load_hardware, load_plant

COL = {c: i for i, c in enumerate(["t", "x", "alpha", "xd", "ad", "v_des", "v", "cart_counts", "pole_counts", "mode", "phase", "extra"])}


def savgol_coeffs(half: int, order: int, deriv: int) -> np.ndarray:
    """Convolution kernel for the deriv-th derivative (per sample^deriv)."""
    k = np.arange(-half, half + 1)
    A = np.vander(k, order + 1, increasing=True)
    pinv = np.linalg.pinv(A)
    return pinv[deriv] * math.factorial(deriv)


SIGMA = 6  # samples (12 ms): one Gaussian for every signal


def gauss_kernel(sigma: float = SIGMA) -> np.ndarray:
    half = int(4 * sigma)
    k = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma) ** 2)
    return k / k.sum()


def smooth(y: np.ndarray, sigma: float = SIGMA) -> np.ndarray:
    return np.convolve(y, gauss_kernel(sigma), mode="same")


def derivs(y: np.ndarray, h: float = H, sigma: float = SIGMA):
    """Smoothed signal and its first two derivatives. One Gaussian, then exact
    differentiation of the smooth result: the same linear filter reaches every
    term of the equation, so a step-shaped input (the PPO policy holds its
    voltage for 20 ms) cannot bias the fit. (A Savitzky-Golay version did:
    its derivative kernels are not the derivatives of its smoothing kernel.)"""
    f = smooth(y, sigma)
    d1 = np.gradient(f, h)
    d2 = np.gradient(d1, h)
    v = np.ones_like(y, dtype=bool)
    edge = int(4 * sigma) + 2
    v[:edge] = v[-edge:] = False
    return f, d1, d2, v


def load_runs(include_sim: bool = False, only_ids=None):
    runs = []
    if not RUNS.exists():
        return runs
    for j in sorted(RUNS.glob("*.json")):
        try:
            meta = json.loads(j.read_text())
        except (OSError, ValueError):
            continue
        if (meta.get("sim") or "fake" in str(meta.get("backend", "")).lower()) and not include_sim:
            continue
        if only_ids and meta["id"] not in only_ids:
            continue
        f = RUNS / f"{meta['id']}.npy"
        if f.exists():
            runs.append((meta, np.load(f)))
    return runs


def build(runs, hw: Hardware, delay: int):
    Xc, yc, Xp, yp = [], [], [], []
    p0 = Plant()
    a_const, ml, g = p0.a, p0.ml, p0.g
    for meta, arr in runs:
        if len(arr) < 200:
            continue
        run_hw = Hardware(**meta["hardware"]) if "hardware" in meta else hw
        x = arr[:, COL["cart_counts"]] * run_hw.cart_m_per_count
        al = arr[:, COL["pole_counts"]] * run_hw.pole_rad_per_count - math.pi
        # Runtime logs logical volts BEFORE the backend applies volts_sign.
        V = arr[:, COL["v"]]
        # row k holds the encoders read BEFORE v_k was written: the voltage acting
        # around sample k is the mean of v_{k-1} and v_k (half a sample), plus any
        # extra actuator delay
        V = 0.5 * (V + np.concatenate([[0.0], V[:-1]]))
        if delay:
            V = np.concatenate([np.zeros(delay), V[:-delay]])
        x0, x1, x2, ok = derivs(x)
        a0, a1, a2, ok2 = derivs(al)
        # the input must pass the same smoothing as the outputs, or the two
        # sides of the equation see different filters and the fit is biased
        # (measured: -15 % on every cart parameter without this line)
        V = smooth(V)
        ok &= ok2
        ok &= np.abs(x) < 0.185                     # barrier / watchdog region is not free motion
        ok &= np.abs(V) < 9.8                       # amplifier saturation
        c, s = np.cos(a0), np.sin(a0)
        Xc.append(np.stack([x2, -a_const * V, x1, np.tanh(x1 / V_COUL)], 1)[ok])
        yc.append((ml * (c * a2 - s * a1 ** 2))[ok])
        okp = ok & (np.abs(a1) > 0.05)              # need pole motion to see J and B_p
        Xp.append(np.stack([a2, a1], 1)[okp])
        yp.append((ml * (g * s + c * x2))[okp])
    if not Xc:
        return None
    return np.concatenate(Xc), np.concatenate(yc), np.concatenate(Xp), np.concatenate(yp)


def lstsq_se(X, y):
    th, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ th
    n, k = X.shape
    s2 = float(r @ r) / max(1, n - k)
    cov = s2 * np.linalg.pinv(X.T @ X)
    r2 = 1 - float(r @ r) / max(1e-12, float(((y - y.mean()) ** 2).sum()))
    return th, np.sqrt(np.clip(np.diag(cov), 0, None)), r2


def fit(include_sim: bool = False, only_ids=None, log=print) -> dict | None:
    hw = load_hardware()
    runs = load_runs(include_sim, only_ids)
    if not runs:
        log("sysid: no runs to fit" + ("" if include_sim else " (hardware runs only; --include-sim to use simulator runs)"))
        return None
    # Split before filtering/differentiation: no filtered sample may leak into
    # the validation segment. Delay is chosen on estimation data only.
    training, validation = [], []
    for meta, arr in runs:
        cut = int(len(arr)*0.65)
        training.append((meta, arr[:cut]))
        validation.append((meta, arr[cut:]))
    best = None
    for d in range(5):
        data = build(training, hw, d)
        if data is None:
            continue
        Xc, yc, Xp, yp = data
        if len(yc) < 500 or not np.all(np.isfinite(Xc)) or not np.all(np.isfinite(yc)):
            continue
        if np.linalg.matrix_rank(Xc) < Xc.shape[1]:
            continue
        thc, sec, r2c = lstsq_se(Xc, yc)
        if best is None or r2c > best["r2c"]:
            best = {"delay": d, "thc": thc, "sec": sec, "r2c": r2c, "data": data}
    if best is None:
        log("sysid: insufficient finite, independently excited samples (need >=500)")
        return None
    Xc, yc, Xp, yp = best["data"]
    heldout = build(validation, hw, best["delay"])
    validation_r2 = None
    if heldout is not None and len(heldout[1]) >= 200:
        Xv, yv = heldout[:2]
        residual = yv-Xv@best["thc"]
        validation_r2 = 1-float(residual@residual)/max(1e-12,float(((yv-yv.mean())**2).sum()))
    if len(yp) > 500:
        thp, sep, r2p = lstsq_se(Xp, yp)
    else:
        thp, sep, r2p = None, None, None
    p = Plant()
    M_tot, k_amp, c_v, F_c = best["thc"]
    res = {"n_samples": int(len(yc)), "n_pole_samples": int(len(yp)), "runs": [m["id"] for m, _ in runs],
           "delay_steps": best["delay"], "r2_cart": round(best["r2c"], 4), "r2_pole": None if r2p is None else round(r2p, 4),
           "raw": {"M_tot": M_tot, "k_amp": k_amp, "c_v": c_v, "F_coul": F_c},
           "se": dict(zip(["M_tot", "k_amp", "c_v", "F_coul"], map(float, best["sec"])))}
    res["validation_r2_cart"] = None if validation_r2 is None else round(validation_r2, 4)
    res["n_validation"] = 0 if heldout is None else len(heldout[1])
    scales = np.sqrt(np.mean(Xc**2, axis=0))
    res["scaled_condition"] = float(np.linalg.cond(Xc / np.maximum(scales, 1e-12)))
    problems = []
    if validation_r2 is None or not np.isfinite(validation_r2) or validation_r2 < 0.75:
        problems.append("held-out cart fit too low or unavailable; collect a longer varied Identify run")
    if res["scaled_condition"] > 1000:
        problems.append("parameter separation implausible: excitation is nearly collinear")
    if best["r2c"] < 0.8:
        problems.append(f"cart fit R2 {best['r2c']:.3f} is too low; collect a clean Identify run")
    if not (0.4 < M_tot < 2.0):
        problems.append(f"moving mass {M_tot:.3f} kg is implausible")
    if not (0.5 < k_amp < 1.5):
        problems.append(f"amplifier gain {k_amp:.3f} is implausible (check volts_sign / channel)")
    if not (p.b * 0.5 < c_v < 40):
        problems.append(f"viscous friction {c_v:.2f} N s/m is implausible")
    if c_v < p.b - 0.5 or not -0.05 <= F_c <= 3:
        problems.append("negative or excessive mechanical friction is implausible; check model/calibration")
    params = {
        "m_c": float(M_tot - p.m_p - p.m_rot),
        "k_amp": float(k_amp),
        "B_f": float(max(0.0, c_v - p.b)),
        "F_coul": float(max(0.0, F_c)),
        "delay_steps": int(best["delay"]),
    }
    if thp is not None:
        J, B_p = thp
        res["raw"].update({"J": float(J), "B_p": float(B_p)})
        res["se"].update({"J": float(sep[0]), "B_p": float(sep[1])})
        if 0.5 * p.J < J < 2.0 * p.J and B_p > -1e-3:
            params["I_factor"] = float(J / (p.m_p * p.l ** 2) - 1.0)
            params["B_p"] = float(max(1e-5, B_p))
        else:
            problems.append(f"pole fit J={J:.2e} B_p={B_p:.2e} rejected, kept prior")
    res["params"] = params
    res["problems"] = problems
    res["time"] = datetime.now().isoformat(timespec="seconds")
    for k in ("raw", "se"):
        res[k] = {a: float(b) for a, b in res[k].items()}
    return res


def save(res: dict, log=print) -> bool:
    if res.get("problems") and any("implausible" in q or "too low" in q for q in res["problems"]):
        log("sysid: NOT saved: " + "; ".join(res["problems"]))
        return False
    try:
        cur = json.loads(FITTED.read_text())
    except (OSError, ValueError):
        cur = {"history": []}
    if cur.get("current"):
        cur["history"].append(cur["current"])
    cur["current"] = res
    FITTED.parent.mkdir(parents=True, exist_ok=True)
    FITTED.write_text(json.dumps(cur, indent=1))
    p = load_plant()
    log(f"sysid saved ({res['n_samples']} samples, R2 cart {res['r2_cart']}, pole {res['r2_pole']}): "
        + ", ".join(f"{k}={v:.4g}" for k, v in res["params"].items()))
    return True


def uncertainty_scale(res: dict | None) -> float:
    """How wide the training randomisation should be, from how much real data
    backs the model: 1.0 with no fit, shrinking toward 0.35 with ~5 minutes of
    good hardware data. Never zero: the rig drifts (temperature, belt)."""
    if not res:
        return 1.0
    minutes = res.get("n_samples", 0) * H / 60
    return float(max(0.35, 1.0 / math.sqrt(1.0 + minutes)))
