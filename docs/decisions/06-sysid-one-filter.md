# 06 — System identification: one Gaussian filter for every signal, half-sample alignment

**Date:** 2026-10-01 · **Status:** accepted

## Context
Least squares on both equations of motion (cart: M, k_amp, c_v, F_coul; pole: J, B_p), tested
by hiding a plant in the simulator and fitting runs recorded through the real runtime.
1. Savitzky–Golay derivatives, raw voltage: every cart parameter −15 % (filter mismatch between
   the two sides of the equation).
2. SG on voltage too: fine for smooth excitation, collapsed on PPO exploration runs (voltage held
   in 20 ms steps): fitted mass 0.16 kg; residual with the TRUE parameters 1.25 against a 0.65
   signal. SG derivative kernels are not derivatives of the SG smoothing kernel.
3. One Gaussian (σ = 12 ms) on everything, then exact differentiation: residual 0.26.
4. Row k holds encoders read before v_k is written, so the voltage acting at sample k is the
   mean of v_{k−1}, v_k: residual 0.027; fit m_c 0.567/0.57, k_amp 0.993/1.0, B_f 2.92/3.0.

## Decision
3 + 4, integer delay 0–4 by best R², plausibility gates (mass, gain, friction) that refuse to save
a fit. Test `Sysid.test_recovers_a_hidden_plant`.

## Consequences
The gates matter: case 2 would have written a 0.16 kg cart into the model. Real data adds
backlash and belt stretch the model has no term for; read `r2_cart` after the first session.
