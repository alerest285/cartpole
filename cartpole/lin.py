"""Linear control design in numpy only: c2d, lqr, dlqr, place, ctrb, obsv.

Mirrors the MATLAB functions the handout uses, so the lab PC needs nothing but
numpy. Checked against each other in tests/test_lin.py.
"""
from __future__ import annotations

import numpy as np


def expm(M: np.ndarray) -> np.ndarray:
    """Matrix exponential by scaling and squaring with a Taylor series."""
    M = np.asarray(M, dtype=float)
    norm = np.linalg.norm(M, 1)
    k = max(0, int(np.ceil(np.log2(norm))) + 1) if norm > 0.5 else 0
    X = M / (2 ** k)
    E = np.eye(M.shape[0]); term = np.eye(M.shape[0])
    for i in range(1, 20):
        term = term @ X / i
        E = E + term
    for _ in range(k):
        E = E @ E
    return E


def c2d(A, B, h):
    """Zero-order-hold discretisation."""
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A; M[:n, n:] = B
    E = expm(M * h)
    return E[:n, :n], E[:n, n:]


def ctrb(A, B):
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])


def obsv(A, C):
    n = A.shape[0]
    return np.vstack([C @ np.linalg.matrix_power(A, k) for k in range(n)])


def dare(A, B, Q, R, tol=1e-12, maxit=200):
    """Discrete algebraic Riccati equation by the structure-preserving doubling
    algorithm: P = A'PA - A'PB (R + B'PB)^-1 B'PA + Q."""
    n = A.shape[0]
    Ak = A.copy()
    G = B @ np.linalg.solve(R, B.T)
    Hk = Q.copy()
    I = np.eye(n)
    for _ in range(maxit):
        W = I + G @ Hk
        Winv_A = np.linalg.solve(W, Ak)
        Winv_G = np.linalg.solve(W, G)
        Hn = Hk + Ak.T @ Hk @ Winv_A
        Gn = G + Ak @ Winv_G @ Ak.T
        An = Ak @ Winv_A
        done = np.linalg.norm(Hn - Hk, 1) <= tol * max(1.0, np.linalg.norm(Hn, 1))
        Ak, G, Hk = An, Gn, Hn
        if done:
            break
    return 0.5 * (Hk + Hk.T)


def dlqr(Ad, Bd, Q, R):
    P = dare(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K, P


def care(A, B, Q, R):
    """Continuous ARE via the stable invariant subspace of the Hamiltonian."""
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    Ham = np.block([[A, -B @ Rinv @ B.T], [-Q, -A.T]])
    w, V = np.linalg.eig(Ham)
    stable = V[:, w.real < 0]
    U1, U2 = stable[:n], stable[n:]
    P = np.real(U2 @ np.linalg.inv(U1))
    return 0.5 * (P + P.T)


def lqr(A, B, Q, R):
    """Continuous LQR, same convention as MATLAB: u = -K x."""
    P = care(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K, P


def place(A, B, poles):
    """Single-input pole placement by Ackermann's formula (MATLAB's acker).
    Poles must be real or come in conjugate pairs."""
    n = A.shape[0]
    coeffs = np.real(np.poly(np.asarray(poles, dtype=complex)))
    phi = sum(c * np.linalg.matrix_power(A, n - i) for i, c in enumerate(coeffs))
    e = np.zeros((1, n)); e[0, -1] = 1.0
    return e @ np.linalg.solve(ctrb(A, B), phi)
